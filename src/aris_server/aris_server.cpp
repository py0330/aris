#ifdef WIN32
#define rt_printf printf
#include <windows.h>
#undef CM_NONE
#endif
#ifdef UNIX
#include "rtdk.h"
#include "unistd.h"
#endif


#include <cstring>
#include <thread>
#include <algorithm>
#include <memory>

#include "aris_core.h"
#include "aris_control.h"
#include "aris_server.h"

namespace aris
{
	namespace server
    {
		class ControlServer::Imp
		{
		public:
			auto start()->void;
			auto stop()->void;
			auto onReceiveMsg(const aris::core::Msg &msg)->aris::core::Msg;
			auto sendParam(const std::string &cmd, const std::map<std::string, std::string> &params)->void;

			auto tg()->void;
            auto checkError()->int;
            auto checkMotionStatus()->void;
			auto executeCmd()->int;
			auto enable()->int;
			auto disable()->int;
			auto home()->int;
			auto run()->int;
			auto onRunError()->int;

            Imp(ControlServer *server) :server_(server) {}
			Imp(const Imp&) = delete;

		private:
			enum RobotCmdID
			{
				ENABLE,
				DISABLE,
				HOME,
				RUN_GAIT,

				ROBOT_CMD_COUNT
			};

		private:
			std::atomic_bool is_running_{false};
			
			ControlServer *server_;

			// 实时循环中的步态参数 //
			enum { CMD_POOL_SIZE = 50 };
			char cmd_queue_[CMD_POOL_SIZE][aris::core::MsgRT::RT_MSG_LENGTH];
			int current_cmd_{ 0 }, cmd_num_{ 0 }, count_{ 0 };

			// 储存上一次slave的数据 //
			std::vector<std::unique_ptr<aris::control::Slave::TxType> > last_data_vec_tx_;
			std::vector<std::unique_ptr<aris::control::Slave::RxType> > last_data_vec_rx_;

			// 以下储存所有的命令 //
			std::map<std::string, int> cmd_id_map_;//store gait id in follow vector
			std::vector<dynamic::PlanFunc> plan_vec_;// store plan func
			std::vector<ParseFunc> parser_vec_; // store parse func

			// 储存特殊命令的parse_func //
			static auto defaultBasicParse(const ControlServer &cs, const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)->void;
			ParseFunc parse_enable_func_ = defaultBasicParse;
			ParseFunc parse_disable_func_ = defaultBasicParse;
			ParseFunc parse_home_func_ = defaultBasicParse;

			// pipe //
			aris::control::Pipe<aris::core::Msg> msg_pipe_;

			// socket //
			aris::core::Socket server_socket_;
			std::string server_socket_ip_, server_socket_port_;

            // 储存模型、控制器和传感器 command parser //
			std::unique_ptr<aris::dynamic::Model> model_;
			std::unique_ptr<aris::sensor::SensorRoot> sensor_root_;
			std::unique_ptr<aris::control::Controller> controller_;
            aris::core::CommandParser parser_;

			// 结束时的callback //
			std::function<void(void)> on_exit_callback_{nullptr};

			friend class ControlServer;
		};
		auto ControlServer::Imp::start()->void
		{
			if (!is_running_)
			{
				last_data_vec_tx_.clear();
				last_data_vec_rx_.clear();
				for (auto &sla : controller_->slavePool())
				{
					last_data_vec_tx_.push_back(std::unique_ptr<aris::control::Slave::TxType>(reinterpret_cast<aris::control::Slave::TxType*>(new char[sla.txTypeSize()])));
					last_data_vec_rx_.push_back(std::unique_ptr<aris::control::Slave::RxType>(reinterpret_cast<aris::control::Slave::RxType*>(new char[sla.rxTypeSize()])));
				}
				
				is_running_ = true;
				controller_->start();
				sensor_root_->start();
			}
		}
		auto ControlServer::Imp::stop()->void
		{
			if (is_running_)
			{
				controller_->stop();
				sensor_root_->stop();
				is_running_ = false;
			}
		}
		auto ControlServer::Imp::onReceiveMsg(const aris::core::Msg &msg)->aris::core::Msg
		{
			try
			{
                std::string cmd;
                std::map<std::string, std::string> params;
				if (msg.data()[msg.size() - 1] != '\0')
				{
					throw std::runtime_error(std::string("invaild command message:command message must be terminated by CHAR \"0\""));
				}

				try
				{
					std::string input{ msg.data() };
					parser_.parse(input, cmd, params);

					std::cout << cmd << std::endl;
					int paramPrintLength;
					if (params.empty())
					{
						paramPrintLength = 2;
					}
					else
					{
						paramPrintLength = std::max_element(params.begin(), params.end(), [](decltype(*params.begin()) a, decltype(*params.begin()) b)
						{
							return a.first.length() < b.first.length();
						})->first.length() + 2;
					}
					for (auto &i : params)
					{
						std::cout << std::string(paramPrintLength - i.first.length(), ' ') << i.first << " : " << i.second << std::endl;
					}

					std::cout << std::endl;
				}
				catch (std::exception &e)
				{
					std::cout << e.what() << std::endl << std::endl;
				}

				if (cmd == "start")
				{
					if (is_running_)throw std::runtime_error("server already started, thus ignore command \"start\"");
					start();
					return aris::core::Msg();
				}
				if (cmd == "stop")
				{
					if (!is_running_)throw std::runtime_error("server already stopped, thus ignore command \"stop\"");
					stop();
					return aris::core::Msg();
				}
				if (cmd == "exit")
				{
					if (is_running_)stop();

					std::thread exit_callback([this]() 
					{
						aris::core::msSleep(1000);
						if (on_exit_callback_)on_exit_callback_();
					});

					exit_callback.detach();

					return aris::core::Msg();
				}
				
				if (!is_running_)throw std::runtime_error("can't execute command, because the server is not STARTED, please start it first");
				sendParam(cmd, params);

				return aris::core::Msg();
			}
			catch (std::exception &e)
			{
				std::cout << aris::core::log(e.what()) << std::endl;
				
				aris::core::Msg error_msg;
				error_msg.copy(e.what());
				return error_msg;
			}
			catch (...)
			{
				std::cout << aris::core::log("unknown exception") << std::endl;
				aris::core::Msg error_msg;
				error_msg.copy("unknown exception");
				return error_msg;
			}
		}
		auto ControlServer::Imp::sendParam(const std::string &cmd, const std::map<std::string, std::string> &params)->void
		{
			aris::core::Msg cmd_msg;

			if (cmd == "en")
			{
				parse_enable_func_(*server_, cmd, params, cmd_msg);
				if (cmd_msg.size() != sizeof(BasicFunctionParam))throw std::runtime_error("invalid msg length of parse function for en");
				reinterpret_cast<BasicFunctionParam *>(cmd_msg.data())->cmd_type_ = ControlServer::Imp::ENABLE;
			}
			else if (cmd == "ds")
			{
				parse_disable_func_(*server_, cmd, params, cmd_msg);
				if (cmd_msg.size() != sizeof(BasicFunctionParam))throw std::runtime_error("invalid msg length of parse function for ds");
				reinterpret_cast<BasicFunctionParam *>(cmd_msg.data())->cmd_type_ = ControlServer::Imp::DISABLE;
			}
			else if (cmd == "hm")
			{
				parse_home_func_(*server_, cmd, params, cmd_msg);
				if (cmd_msg.size() != sizeof(BasicFunctionParam))throw std::runtime_error("invalid msg length of parse function for hm");
				reinterpret_cast<BasicFunctionParam *>(cmd_msg.data())->cmd_type_ = ControlServer::Imp::HOME;
			}
			else
			{
				auto cmdPair = this->cmd_id_map_.find(cmd);

				if (cmdPair == this->cmd_id_map_.end())
				{
					throw std::runtime_error(std::string("command \"") + cmd + "\" does not have gait function, please AddCmd() first");
				}

				this->parser_vec_.at(cmdPair->second).operator()(*server_, cmd, params, cmd_msg);

				if (cmd_msg.size() < sizeof(GaitParamBase))
				{
					throw std::runtime_error(std::string("parse function of command \"") + cmdPair->first + "\" failed: because it returned invalid cmd_msg");
				}

				reinterpret_cast<GaitParamBase *>(cmd_msg.data())->cmd_type_ = RUN_GAIT;
				reinterpret_cast<GaitParamBase *>(cmd_msg.data())->gait_id_ = cmdPair->second;

				if (plan_vec_.at(cmdPair->second) == nullptr) return;
			}

			cmd_msg.setMsgID(0);
			msg_pipe_.sendToRT(cmd_msg);
		}
		auto ControlServer::Imp::tg()->void
        {
			// 检查是否出错 //
            if (checkError())return;
            //checkMotionStatus();

			// 查看是否有新cmd //
            if (msg_pipe_.recvInRT(aris::core::MsgRT::instance[0]) > 0)
			{
                if (cmd_num_ >= CMD_POOL_SIZE)
				{
					rt_printf("cmd pool is full, thus ignore last command\n");
				}
				else
				{
                    aris::core::MsgRT::instance[0].paste(cmd_queue_[(current_cmd_ + cmd_num_) % CMD_POOL_SIZE]);
                    ++cmd_num_;
				}
			}
			
			// 执行cmd queue中的cmd //
            if (cmd_num_ > 0)
			{
				if (executeCmd())
				{
                    if (++count_ % 1000 == 0)rt_printf("execute cmd in count: %d\n", count_);
				}
				else
				{
                    rt_printf("cmd finished, spend %d counts\n\n", count_ + 1);
                    count_ = 0;
                    current_cmd_ = (current_cmd_ + 1) % CMD_POOL_SIZE;
                    --cmd_num_;
				}
            }

			// 储存上次的slave数据 //
			for (std::size_t i = 0; i < controller_->slavePool().size(); ++i)
			{
				controller_->slavePool().at(i).getTxData(std::ref(*last_data_vec_tx_.at(i)));
				controller_->slavePool().at(i).getRxData(std::ref(*last_data_vec_rx_.at(i)));
			}
		}
		auto ControlServer::Imp::checkError()->int
		{
			static int fault_count = 0;
			
            if (fault_count || std::find_if(controller_->slavePool().begin(), controller_->slavePool().end(), [](const aris::control::Slave &slave) {return slave.rxData().ret < 0; }) != controller_->slavePool().end())
			{
				if (fault_count++ % 1000 == 0)
				{
                    rt_printf("ret of physic ethercat ring using SLA sequence are: ");
                    for (auto &slave : controller_->slavePool())rt_printf("%d ", slave.rxData().ret);
					rt_printf("\n");
					rt_printf("Some slave is in fault, now try to disable all motors\n");
					rt_printf("All commands in command queue are discarded\n");
				}

                for(std::size_t i=0; i<model_->motionPool().size();i++)
                {
                    std::size_t slaID=model_->motionPool().at(i).slaID();
                    static_cast<aris::control::TxMotionData&>(controller_->txDataPool().at(slaID)).cmd= aris::control::Motion::DISABLE;
                }

				cmd_num_ = 0;
				count_ = 0;
			}
			else
			{
				fault_count = 0;
			}

			return fault_count;
		}
        auto ControlServer::Imp::checkMotionStatus()->void
        {
            for (auto &motion : model_->motionPool())
            {
                auto &rx_motion_data=static_cast<aris::control::RxMotionData&>(controller_->rxDataPool().at(motion.slaID()));
                if(rx_motion_data.fault_warning==0)
                    continue;
                else
                {
                    rt_printf("Motor %d (sla id) has wrong status:%d\n", motion.slaID(), rx_motion_data.fault_warning);
                    rt_printf("The status using ABS sequence are:\n");
                    for (auto &motion : model_->motionPool())
                    {
                        auto &rx_data=static_cast<aris::control::RxMotionData&>(controller_->rxDataPool().at(motion.slaID()));
                        rt_printf("%d\t", rx_data.fault_warning);
                    }
                    rt_printf("\n");
                    return;
                }
            }

        }

		auto ControlServer::Imp::executeCmd()->int
		{
			aris::dynamic::PlanParamBase *param = reinterpret_cast<aris::dynamic::PlanParamBase *>(cmd_queue_[current_cmd_]);
			param->count_ = count_;

			switch (param->cmd_type_)
			{
			case ENABLE:
				return enable();
			case DISABLE:
				return disable();
			case HOME:
				return home();
			case RUN_GAIT:
				return run();
			default:
				rt_printf("unknown cmd type\n");
				return 0;
			}
		}
		auto ControlServer::Imp::enable()->int 
		{
			BasicFunctionParam *param = reinterpret_cast<BasicFunctionParam *>(cmd_queue_[current_cmd_]);

			bool is_all_enabled = true;
            for(std::size_t i=0; i<model_->motionPool().size();i++)
            {
                std::size_t slaID=model_->motionPool().at(i).slaID();
                if(param->active_motor_[i])
                {
                    auto &tx_motion_data=static_cast<aris::control::TxMotionData&>(controller_->txDataPool().at(slaID));
                    auto &rx_motion_data=static_cast<aris::control::RxMotionData&>(controller_->rxDataPool().at(slaID));
                    //判断是否已经Enable了
                    if ((param->count_ != 0) && (rx_motion_data.ret == 0))
                    {
                        // 判断是否为第一次走到enable,否则什么也不做，这样就会继续刷上次的值
                        if (tx_motion_data.cmd == aris::control::Motion::ENABLE)
                        {
							tx_motion_data.cmd = aris::control::Motion::RUN;
							tx_motion_data.mode = aris::control::Motion::POSITION;
							tx_motion_data.target_pos = rx_motion_data.feedback_pos;
							tx_motion_data.target_vel = 0;
							tx_motion_data.target_tor = 0;
                        }
                    }
                    else
                    {
                        is_all_enabled = false;
						tx_motion_data.cmd = aris::control::Motion::ENABLE;
						tx_motion_data.mode = aris::control::Motion::POSITION;

                        if (param->count_ % 1000 == 0)
                        {
                            rt_printf("Unenabled motor, slave id: %d, absolute id: %d, ret: %d\n", slaID, i, rx_motion_data.ret);
                        }
                    }
                }
            }
			return is_all_enabled ? 0 : 1;
        }
        auto ControlServer::Imp::disable()->int
        {
			BasicFunctionParam *param = reinterpret_cast<BasicFunctionParam *>(cmd_queue_[current_cmd_]);
			
			bool is_all_disabled = true;
            for(std::size_t i=0; i<model_->motionPool().size();i++)
            {
                std::size_t slaID=model_->motionPool().at(i).slaID();
                if(param->active_motor_[i])
                {
					auto &tx_motion_data = static_cast<aris::control::TxMotionData&>(controller_->txDataPool().at(slaID));
					auto &rx_motion_data = static_cast<aris::control::RxMotionData&>(controller_->rxDataPool().at(slaID));
                    //判断是否已经Disable了
                    if ((param->count_ != 0) && (rx_motion_data.ret == 0))
                    {
                        // 如果已经disable了，那么什么都不做
                    }
                    else
                    {
                        // 否则往下刷disable指令
                        is_all_disabled = false;
						tx_motion_data.cmd = aris::control::Motion::DISABLE;

                        if (param->count_ % 1000 == 0)
                        {
                            rt_printf("Undisabled motor, slave id: %d, absolute id: %d, ret: %d\n", slaID, i, rx_motion_data.ret);
                        }
                    }
                }
            }
            return is_all_disabled ? 0 : 1;
        }
        auto ControlServer::Imp::home()->int
        {
			BasicFunctionParam *param = reinterpret_cast<BasicFunctionParam *>(cmd_queue_[current_cmd_]);
			
			bool is_all_homed = true;
            for(std::size_t i=0; i<model_->motionPool().size();i++)
            {
                std::size_t slaID=model_->motionPool().at(i).slaID();
                if(param->active_motor_[i])
                {
                    auto &txmotiondata=static_cast<aris::control::TxMotionData&>(controller_->txDataPool().at(slaID));
                    auto &rxmotiondata=static_cast<aris::control::RxMotionData&>(controller_->rxDataPool().at(slaID));
                    // 根据返回值来判断是否走到home了
                    if ((param->count_ != 0) && (rxmotiondata.ret == 0))
                    {
                        // 判断是否为第一次走到home,否则什么也不做，这样就会继续刷上次的值
                        if (txmotiondata.cmd == aris::control::Motion::HOME)
                        {
                            txmotiondata.cmd = aris::control::Motion::RUN;
                            txmotiondata.target_pos = rxmotiondata.feedback_pos;
                            txmotiondata.target_vel = 0;
                            txmotiondata.target_tor = 0;
                        }
                    }
                    else
                    {
                        is_all_homed = false;
                        txmotiondata.cmd = aris::control::Motion::HOME;

                        if (param->count_ % 1000 == 0)
                        {
                            rt_printf("Unenabled motor, slave id: %d, absolute id: %d, ret: %d\n", slaID, i,rxmotiondata.ret);
                        }
                    }
                }
            }
            return is_all_homed ? 0 : 1;
        }
        auto ControlServer::Imp::run()->int
        {
            GaitParamBase *param = reinterpret_cast<GaitParamBase *>(cmd_queue_[current_cmd_]);
			param->cs_ = server_;

            // 执行gait函数 //
            int ret = this->plan_vec_.at(param->gait_id_).operator()(*model_.get(), *param);

            // 向下写入输入位置 //
			for (std::size_t i = 0; i<model_->motionPool().size(); ++i)
			{
				std::size_t sla_id = model_->motionPool().at(i).slaID();
                if (param->active_motor_[i])
				{
					auto &tx_motion_data = static_cast<aris::control::TxMotionData&>(controller_->txDataPool().at(sla_id));
					tx_motion_data.cmd = aris::control::Motion::RUN;
					tx_motion_data.target_pos = model_->motionPool().at(i).motPos();
                    //tx_motion_data.vel_offset = model_->motionPool().at(i).motVel();
				}
			}

            // 检查电机命令是否超限 //
            for (std::size_t i = 0; i<model_->motionPool().size()&&param->active_motor_[i]; ++i)
            {
				std::size_t abs_id = model_->motionPool().at(i).absID();
				std::size_t sla_id = model_->motionPool().at(i).slaID();
				std::size_t phy_id = model_->motionPool().at(i).phyID();
				auto &tx_motion_data = static_cast<aris::control::TxMotionData&>(controller_->txDataPool().at(sla_id));
                auto &last_tx_motion_data = static_cast<aris::control::TxMotionData&>(*last_data_vec_tx_.at(sla_id));
				auto &control_motion=static_cast<aris::control::Motion&>(controller_->slavePool().at(sla_id));
                
				if (tx_motion_data.cmd == aris::control::Motion::RUN && param->active_motor_[sla_id])
                {
                    // check max pos //
					if (param->if_check_pos_max_ && (tx_motion_data.target_pos > control_motion.maxPos()))
                    {
						rt_printf("Motor %d %d %d (abs phy and sla id) target position is bigger than its MAX permitted value in count:%d\n", abs_id, phy_id, sla_id, count_);
						rt_printf("The min, max and current count using ABS sequence are:\n");
						for (auto &motion : model_->motionPool())
						{
							auto &control_motion = static_cast<aris::control::Motion&>(controller_->slavePool().at(motion.slaID()));
							rt_printf("%lf   %lf   %lf\n", control_motion.minPos(), control_motion.maxPos(), control_motion.txData().target_pos);
						}
						onRunError();
						return 0;
                    }

					// check min pos //
                    if (param->if_check_pos_min_ && (tx_motion_data.target_pos < control_motion.minPos()))
                    {
                        rt_printf("Motor %d %d %d (abs phy and sla id) target position is smaller than its MIN permitted value in count:%d\n", abs_id, phy_id, sla_id, count_);
						rt_printf("The min, max and current count using ABS sequence are:\n");
						for (auto &motion : model_->motionPool())
						{
							auto &control_motion = static_cast<aris::control::Motion&>(controller_->slavePool().at(motion.slaID()));
							rt_printf("%lf   %lf   %lf\n", control_motion.minPos(), control_motion.maxPos(), control_motion.txData().target_pos);
						}
						onRunError();
                        return 0;
                    }

					// check pos continuous //
                    if (param->if_check_pos_continuous_ && (std::abs(tx_motion_data.target_pos - last_tx_motion_data.target_pos)>0.0012*control_motion.maxVel()))
                    {
                        rt_printf("Motor %d %d %d (abs phy and sla id) target position is not continuous in count:%d\n", abs_id, phy_id, sla_id, count_);
						rt_printf("The pin of last and this count using ABS sequence are:\n");
						for (auto &motion : model_->motionPool())
						{
							auto &control_motion = static_cast<aris::control::Motion&>(controller_->slavePool().at(motion.slaID()));
							rt_printf("%lf   %lf\n", last_tx_motion_data.target_pos, tx_motion_data.target_pos);
						}
						onRunError();
						return 0;
                    }
                }
            }

            return ret;
        }
		auto ControlServer::Imp::onRunError()->int
		{
			
			rt_printf("All commands in command queue are discarded, please try to RECOVER\n");
			cmd_num_ = 1;//因为这里为0退出，因此之后在tg中回递减cmd_num_,所以这里必须为1
			count_ = 0;

			// 发现不连续，那么使用上一个成功的cmd，以便等待修复 //
			for (std::size_t i = 0; i < controller_->txDataPool().size(); ++i)
			{
				controller_->slavePool().at(i).setTxData(*last_data_vec_tx_.at(i));
			}

			return 0;

		}
		auto ControlServer::Imp::defaultBasicParse(const ControlServer &cs, const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)->void
		{
			BasicFunctionParam param;

			for (auto &i : params)
			{
				if (i.first == "all")
				{
					std::fill(param.active_motor_, param.active_motor_ + MAX_MOTOR_NUM, true);
				}
				else if (i.first == "motion_id")
				{
					std::size_t id{ std::stoul(i.second) };
					if (id < 0 || id > cs.model().motionPool().size())throw std::runtime_error("invalid param in basic parse func in param \"" + i.first + "\"");
					std::fill(param.active_motor_, param.active_motor_ + MAX_MOTOR_NUM, false);
					param.active_motor_[cs.model().motionAtAbs(id).absID()] = true;
				}
				else if (i.first == "physical_id")
				{
					std::size_t id{ std::stoul(i.second) };
					if (id < 0 || id > cs.model().motionPool().size())throw std::runtime_error("invalid param in basic parse func in param \"" + i.first + "\"");
					std::fill(param.active_motor_, param.active_motor_ + MAX_MOTOR_NUM, false);
					param.active_motor_[cs.model().motionAtPhy(id).absID()] = true;
				}
				else if (i.first == "slave_id")
				{
					std::size_t id{ std::stoul(i.second) };
					if (id < 0 || id > cs.controller().slavePool().size())throw std::runtime_error("invalid param in basic parse func in param \"" + i.first + "\"");
					std::fill(param.active_motor_, param.active_motor_ + MAX_MOTOR_NUM, false);
					if(cs.model().motionAtSla(id).absID() >= cs.model().motionPool().size())throw std::runtime_error("invalid param in basic parse func in param \"" + i.first + "\", this slave is not motion");
					param.active_motor_[cs.model().motionAtSla(id).absID()] = true;
				}
				else
				{
					throw std::runtime_error("unknown param in basic parse func in param \"" + i.first + "\"");
				}
			}

			msg.copyStruct(param);
		}

		ControlServer &ControlServer::instance()
		{
			static ControlServer instance;
			return std::ref(instance);
		}
		ControlServer::ControlServer() :imp_(new Imp(this)) {}
		ControlServer::~ControlServer() {}
		auto ControlServer::createModel(dynamic::Model *model_)->void
		{
			if (imp_->model_)throw std::runtime_error("control sever can't create model because it already has one");
			imp_->model_.reset(model_);
		}
		auto ControlServer::createController(control::Controller *controller)->void
		{
			if (imp_->controller_)throw std::runtime_error("control sever can't create controller because it already has one");
			imp_->controller_.reset(controller);
		}
		auto ControlServer::createSensorRoot(sensor::SensorRoot *sensor_root)->void
		{
			if (imp_->sensor_root_)throw std::runtime_error("control sever can't create sensor_root because it already has one");
			imp_->sensor_root_.reset(sensor_root);
		}
		auto ControlServer::loadXml(const char *fileName)->void
		{
			aris::core::XmlDocument doc;

			if (doc.LoadFile(fileName) != 0)
			{
				throw std::logic_error((std::string("could not open file:") + std::string(fileName)));
			}

			loadXml(doc);
		}
		auto ControlServer::loadXml(const aris::core::XmlDocument &xml_doc)->void 
        {
			/// load robot model_ ///
            imp_->model_->loadXml(xml_doc);
            imp_->controller_->loadXml(xml_doc);
            imp_->sensor_root_->loadXml(xml_doc);
            imp_->parser_.loadXml(xml_doc);

			/// load connection param ///
			imp_->server_socket_ip_ = xml_doc.RootElement()->FirstChildElement("Server")->Attribute("ip");
			imp_->server_socket_port_ = xml_doc.RootElement()->FirstChildElement("Server")->Attribute("port");

			/// Set socket connection callback function ///
			imp_->server_socket_.setOnReceivedConnection([](aris::core::Socket *pConn, const char *pRemoteIP, int remotePort)
			{
				aris::core::log(std::string("received connection, the server_socket_ip_ is: ") + pRemoteIP);
				return 0;
			});
			imp_->server_socket_.setOnReceivedRequest([this](aris::core::Socket *pConn, aris::core::Msg &msg)
			{
				return imp_->onReceiveMsg(msg);
			});
			imp_->server_socket_.setOnLoseConnection([this](aris::core::Socket *socket)
			{
				aris::core::log("lost connection");
				while (true)
				{
					try
					{
						socket->startServer(imp_->server_socket_port_.c_str());
						break;
					}
					catch (aris::core::Socket::StartServerError &e)
					{
						std::cout << e.what() << std::endl << "will try to restart server socket in 1s" << std::endl;
						aris::core::msSleep(1000);
					}
				}
				aris::core::log("restart server socket successful");

				return 0;
			});

            ///set the tg///
            imp_->controller_->setControlStrategy([this]()
            {
                this->imp_->tg();
            });



		}
		auto ControlServer::model()->dynamic::Model&
		{
			return std::ref(*imp_->model_.get());
		}
		auto ControlServer::controller()->control::Controller&
		{
			return std::ref(*imp_->controller_);
		}
		auto ControlServer::sensorRoot()->sensor::SensorRoot&
		{
			return *imp_->sensor_root_;
		}
		auto ControlServer::addCmd(const std::string &cmd_name, const ParseFunc &parse_func, const aris::dynamic::PlanFunc &gait_func)->void
		{
			if (cmd_name == "en")
			{
				if (gait_func)throw std::runtime_error("you can not set plan_func for \"en\" command");
				imp_->parse_enable_func_ = parse_func;
			}
			else if (cmd_name == "ds")
			{
				if (gait_func)throw std::runtime_error("you can not set plan_func for \"ds\" command");
				imp_->parse_disable_func_ = parse_func;
			}
			else if (cmd_name == "hm")
			{
				if (gait_func)throw std::runtime_error("you can not set plan_func for \"hm\" command");
				imp_->parse_home_func_ = parse_func;
			}
			else
			{
				if (imp_->cmd_id_map_.find(cmd_name) != imp_->cmd_id_map_.end())
				{
					throw std::runtime_error(std::string("failed to add command, because \"") + cmd_name + "\" already exists");
				}
				else if (imp_->parser_.commandPool().findByName(cmd_name) == imp_->parser_.end())
				{
					throw std::runtime_error(std::string("failed to add command, because xml does not have \"") + cmd_name + "\" node");
				}
				else
				{
					imp_->plan_vec_.push_back(gait_func);
					imp_->parser_vec_.push_back(parse_func);
					imp_->cmd_id_map_.insert(std::make_pair(cmd_name, imp_->plan_vec_.size() - 1));

					std::cout << cmd_name << ":" << imp_->cmd_id_map_.at(cmd_name) << std::endl;
				}
			}
		}
		auto ControlServer::open()->void 
		{
			for (;;)
			{
				try
				{
					imp_->server_socket_.startServer(imp_->server_socket_port_.c_str());
					break;
				}
				catch (aris::core::Socket::StartServerError &e)
				{
					std::cout << e.what() << std::endl << "will try to restart server socket in 1s" << std::endl;
					aris::core::msSleep(1000);
				}
			}
			std::cout << aris::core::log("server open successful") << std::endl;
		}
		auto ControlServer::close()->void 
		{
			imp_->server_socket_.stop();
		}
		auto ControlServer::setOnExit(std::function<void(void)> callback_func)->void
		{
			this->imp_->on_exit_callback_ = callback_func;
		}
	}
}








