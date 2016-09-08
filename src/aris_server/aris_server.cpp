#ifdef WIN32
#define rt_printf printf
//#include <windows.h>
//#undef CM_NONE
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
			auto sendParam(const std::string &cmd, const std::map<std::string, std::string> &params)->void;

			auto tg()->void;
            auto checkError()->int;
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
			char cmd_queue_[CMD_POOL_SIZE][MAX_PLAN_PARAM_SIZE];
			int current_cmd_{ 0 }, cmd_num_{ 0 }, count_{ 0 };

			// 储存上一次slave的数据 //
			std::vector<std::unique_ptr<aris::control::Slave::TxType> > last_data_vec_tx_;
			std::vector<std::unique_ptr<aris::control::Slave::RxType> > last_data_vec_rx_;

			// 以下储存所有的命令的parse和plan函数 //
			std::map<std::string, int> cmd_id_map_;//store gait id in follow vector
			std::vector<dynamic::PlanFunc> plan_vec_;// store plan func
			std::vector<ParseFunc> parser_vec_; // store parse func

			// 储存特殊命令的parse_func //
            static auto defaultBasicParse(ControlServer &cs, const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)->void;
			ParseFunc parse_enable_func_ = defaultBasicParse;
			ParseFunc parse_disable_func_ = defaultBasicParse;
			ParseFunc parse_home_func_ = defaultBasicParse;

            // 储存Model, Controller, SensorRoot, WidgetRoot //
			std::unique_ptr<aris::dynamic::Model> model_;
			std::unique_ptr<aris::sensor::SensorRoot> sensor_root_;
			std::unique_ptr<aris::control::Controller> controller_;
			std::unique_ptr<aris::server::WidgetRoot> widget_root_;

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
			server_->widgetRoot().cmdPipe().sendMsg(cmd_msg);
		}
		auto ControlServer::Imp::tg()->void
        {
			static aris::core::MsgFix<MAX_PLAN_PARAM_SIZE> recv_msg;

			// 检查是否出错 //
            if (checkError())return;

			// 查看是否有新cmd //
            if (server_->widgetRoot().cmdPipe().recvMsg(recv_msg))
			{
                if (cmd_num_ >= CMD_POOL_SIZE)
				{
					rt_printf("cmd pool is full, thus ignore last command\n");
				}
				else
				{
					recv_msg.paste(cmd_queue_[(current_cmd_ + cmd_num_) % CMD_POOL_SIZE]);
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

			// 向外发送msg //
			server_->widgetRoot().mout().update();
			if (!server_->widgetRoot().msgOut().empty())
			{
				server_->widgetRoot().msgPipe().sendMsg(server_->widgetRoot().msgOut());
				server_->widgetRoot().msgOut().resize(0);
				server_->widgetRoot().mout().resetBuf();
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
                        // 判断是否为第一次走到enable,否则什么也不做,这样就会继续刷上次的值
                        if (tx_motion_data.cmd == aris::control::Motion::ENABLE)
                        {
							tx_motion_data.cmd = aris::control::Motion::RUN;
							tx_motion_data.target_pos = rx_motion_data.feedback_pos;
							tx_motion_data.target_vel = 0;
							tx_motion_data.target_tor = 0;
                        }
                    }
                    else
                    {
                        is_all_enabled = false;
						tx_motion_data.cmd = aris::control::Motion::ENABLE;

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
                        // 如果已经disable了,那么什么都不做
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
                        // 判断是否为第一次走到home,否则什么也不做,这样就会继续刷上次的值
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
                            rt_printf("Unhomeed motor, slave id: %d, absolute id: %d, ret: %d\n", slaID, i,rxmotiondata.ret);
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

            // input position/velocity/torque //
			for (std::size_t i = 0; i<model_->motionPool().size(); ++i)
			{
				std::size_t sla_id = model_->motionPool().at(i).slaID();
                if (param->active_motor_[i])
				{
					auto &tx_motion_data = static_cast<aris::control::TxMotionData&>(controller_->txDataPool().at(sla_id));
                    auto &rx_motion_data = static_cast<aris::control::RxMotionData&>(controller_->rxDataPool().at(sla_id));
					tx_motion_data.cmd = aris::control::Motion::RUN;
                    if(rx_motion_data.mode==8)
                    {
                        //position
                        tx_motion_data.target_pos = model_->motionPool().at(i).motPos();
                    }
                    else if(rx_motion_data.mode==9)
                    {
                        //velocity
                        tx_motion_data.target_vel = model_->motionPool().at(i).motVel();
                    }
                    else if(rx_motion_data.mode==10)
                    {
                        //torque
                        tx_motion_data.target_tor = model_->motionPool().at(i).motAcc();
                    }
                    else
                    {
                        rt_printf("Invalid mode.\n");
                    }

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
			cmd_num_ = 1;//因为这里为0退出,因此之后在tg中回递减cmd_num_,所以这里必须为1
			count_ = 0;

			// 发现不连续,那么使用上一个成功的cmd,以便等待修复 //
			for (std::size_t i = 0; i < controller_->txDataPool().size(); ++i)
			{
				controller_->slavePool().at(i).setTxData(*last_data_vec_tx_.at(i));
			}

			return 0;
		}
        auto ControlServer::Imp::defaultBasicParse(ControlServer &cs, const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)->void
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
		auto ControlServer::instance()->ControlServer & { static ControlServer instance; return instance; }
		auto ControlServer::resetModel(dynamic::Model *model)->void{ imp_->model_.reset(model);}
		auto ControlServer::resetController(control::Controller *controller)->void{ imp_->controller_.reset(controller); }
		auto ControlServer::resetSensorRoot(sensor::SensorRoot *sensor_root)->void{ imp_->sensor_root_.reset(sensor_root); }
		auto ControlServer::resetWidgetRoot(server::WidgetRoot *widget_root)->void{	imp_->widget_root_.reset(widget_root); }
		auto ControlServer::widgetRoot()->WidgetRoot&{	return std::ref(*imp_->widget_root_); }
		auto ControlServer::model()->dynamic::Model& { return std::ref(*imp_->model_); }
		auto ControlServer::controller()->control::Controller& { return std::ref(*imp_->controller_); }
		auto ControlServer::sensorRoot()->sensor::SensorRoot& { return std::ref(*imp_->sensor_root_); }
		auto ControlServer::loadXml(const char *file_name)->void
		{
			aris::core::XmlDocument doc;

			if (doc.LoadFile(file_name) != 0)
			{
				throw std::logic_error((std::string("could not open file:") + std::string(file_name)));
			}

			loadXml(doc);
		}
		auto ControlServer::loadXml(const aris::core::XmlDocument &xml_doc)->void
		{
			// load robot model_ //
			imp_->model_->loadXml(xml_doc);
			imp_->controller_->loadXml(xml_doc);
			imp_->sensor_root_->loadXml(xml_doc);
			imp_->widget_root_->loadXml(xml_doc);

			// set the tg //
			imp_->controller_->setControlStrategy([this]() {this->imp_->tg(); });
		}
		auto ControlServer::addCmd(const std::string &cmd_name, const ParseFunc &parse_func, const aris::dynamic::PlanFunc &plan_func)->void
		{
			if (cmd_name == "en")
			{
				if (plan_func)throw std::runtime_error("you can not set plan_func for \"en\" command");
				imp_->parse_enable_func_ = parse_func;
			}
			else if (cmd_name == "ds")
			{
				if (plan_func)throw std::runtime_error("you can not set plan_func for \"ds\" command");
				imp_->parse_disable_func_ = parse_func;
			}
			else if (cmd_name == "hm")
			{
				if (plan_func)throw std::runtime_error("you can not set plan_func for \"hm\" command");
				imp_->parse_home_func_ = parse_func;
			}
			else
			{
				if (imp_->cmd_id_map_.find(cmd_name) != imp_->cmd_id_map_.end())
				{
					throw std::runtime_error(std::string("failed to add command, because \"") + cmd_name + "\" already exists");
				}
                else if (widgetRoot().cmdParser().commandPool().findByName(cmd_name) == widgetRoot().cmdParser().commandPool().end())
				{
					throw std::runtime_error(std::string("failed to add command, because xml does not have \"") + cmd_name + "\" node");
				}
				else
				{
					imp_->plan_vec_.push_back(plan_func);
					imp_->parser_vec_.push_back(parse_func);
					imp_->cmd_id_map_.insert(std::make_pair(cmd_name, imp_->plan_vec_.size() - 1));

					std::cout << cmd_name << ":" << imp_->cmd_id_map_.at(cmd_name) << std::endl;
				}
			}
		}
		auto ControlServer::executeCmd(const std::string &cmd_string)->void
		{
			std::string cmd;
			std::map<std::string, std::string> params;

			aris::core::log(cmd_string);

			widgetRoot().cmdParser().parse(cmd_string, cmd, params);

			// print cmd and params //
			auto print_size = params.empty() ? 2 : 2 + std::max_element(params.begin(), params.end(), [](const auto& a, const auto& b)
			{
				return a.first.length() < b.first.length();
			})->first.length();
			std::cout << cmd << std::endl;
			for (auto &p : params)
			{
				std::cout << std::string(print_size - p.first.length(), ' ') << p.first << " : " << p.second << std::endl;
			}
			std::cout << std::endl;
			// print over //

			if (cmd == "start")
			{
				if (imp_->is_running_)throw std::runtime_error("server already started, thus ignore command \"start\"");
				imp_->start();
				return;
			}
			else if (cmd == "stop")
			{
				if (!imp_->is_running_)throw std::runtime_error("server already stopped, thus ignore command \"stop\"");
				imp_->stop();
				return;
			}
			else if (cmd == "exit")
			{
				if (imp_->is_running_)imp_->stop();

				std::thread exit_callback([this]()
				{
					std::this_thread::sleep_for(std::chrono::seconds(1));
					if (imp_->on_exit_callback_)imp_->on_exit_callback_();
				});

				exit_callback.detach();
				return;
			}

			if (!imp_->is_running_)throw std::runtime_error("can't execute command, because the server is not STARTED, please start it first");
			imp_->sendParam(cmd, params);
		}
		auto ControlServer::setOnExit(std::function<void(void)> callback_func)->void{ imp_->on_exit_callback_ = callback_func;}
		ControlServer::~ControlServer() = default;
		ControlServer::ControlServer() :imp_(new Imp(this)) 
		{
			// create instance //
			makeModel<aris::dynamic::Model>();
			makeController<aris::control::Controller>();
			makeSensorRoot<aris::sensor::SensorRoot>();
			makeWidgetRoot<aris::server::WidgetRoot>();
		}
	}
}








