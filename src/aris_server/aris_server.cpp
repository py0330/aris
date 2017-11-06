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
		auto default_command_root()->const aris::core::Object &	{
			static aris::core::Object root;
			if (root.children().size() == 0)
			{
				root.registerType<aris::core::Command>();
				root.registerType<aris::core::Param>();
				root.registerType<aris::core::GroupParam>();
				root.registerType<aris::core::UniqueParam>();
				
				auto &en = root.add<aris::core::Command>("en", "", "");
				auto &en_group = en.add<aris::core::GroupParam>("group", "");
				auto &en_active_motion = en_group.add<aris::core::UniqueParam>("active", "all", "");
				en_active_motion.add<aris::core::Param>("all", "", "", 'a');
				en_active_motion.add<aris::core::Param>("motion_id", "0", "", 'm');
				en_active_motion.add<aris::core::Param>("physical_id", "0", "", 'p');
				en_active_motion.add<aris::core::Param>("slave_id", "0", "", 's');
				auto &en_limit_time = en_group.add<aris::core::Param>("limit_time", "10000", "", 'l');

				auto &ds = root.add<aris::core::Command>("ds", "", "");
				auto &ds_group = ds.add<aris::core::GroupParam>("group", "");
				auto &ds_active_motion = ds_group.add<aris::core::UniqueParam>("active", "all", "");
				ds_active_motion.add<aris::core::Param>("all", "", "", 'a');
				ds_active_motion.add<aris::core::Param>("motion_id", "0", "", 'm');
				ds_active_motion.add<aris::core::Param>("physical_id", "0", "", 'p');
				ds_active_motion.add<aris::core::Param>("slave_id", "0", "", 's');
				auto &ds_limit_time = ds_group.add<aris::core::Param>("limit_time", "10000", "", 'l');

				auto &hm = root.add<aris::core::Command>("hm", "", "");
				auto &hm_group = hm.add<aris::core::GroupParam>("group", "");
				auto &hm_active_motion = hm_group.add<aris::core::UniqueParam>("active", "all", "");
				hm_active_motion.add<aris::core::Param>("all", "", "", 'a');
				hm_active_motion.add<aris::core::Param>("motion_id", "0", "", 'm');
				hm_active_motion.add<aris::core::Param>("physical_id", "0", "", 'p');
				hm_active_motion.add<aris::core::Param>("slave_id", "0", "", 's');
				auto &hm_limit_time = hm_group.add<aris::core::Param>("limit_time", "10000", "", 'l');

				auto &md = root.add<aris::core::Command>("md", "", "");
				auto &md_group = md.add<aris::core::GroupParam>("group", "");
				auto &md_active_motion = md_group.add<aris::core::UniqueParam>("active", "all", "");
				md_active_motion.add<aris::core::Param>("all", "", "", 'a');
				md_active_motion.add<aris::core::Param>("motion_id", "0", "", 'm');
				md_active_motion.add<aris::core::Param>("physical_id", "0", "", 'p');
				md_active_motion.add<aris::core::Param>("slave_id", "0", "", 's');
				auto &md_limit_time = md_group.add<aris::core::Param>("limit_time", "10000", "", 'l');
			}
			return root;
		}
		
		auto default_parse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg_out)->void
		{
			DefaultParam param;

			auto &cs = aris::server::ControlServer::instance();

			for (auto &i : params)
			{
				if (i.first == "all")
				{
					std::fill(param.active_motor_, param.active_motor_ + MAX_MOTOR_NUM, true);
				}
				else if (i.first == "motion_id")
				{
					std::size_t id{ std::stoul(i.second) };
					if (id > cs.controller().motionPool().size())throw std::runtime_error("invalid param in basic parse func in param \"" + i.first + "\"");
					std::fill(param.active_motor_, param.active_motor_ + MAX_MOTOR_NUM, false);
					param.active_motor_[cs.controller().motionAtAbs(id).motId()] = true;
				}
				else if (i.first == "physical_id")
				{
					std::size_t id{ std::stoul(i.second) };
					if (id > cs.controller().motionPool().size())throw std::runtime_error("invalid param in basic parse func in param \"" + i.first + "\"");
					std::fill(param.active_motor_, param.active_motor_ + MAX_MOTOR_NUM, false);
					param.active_motor_[cs.controller().motionAtPhy(id).motId()] = true;
				}
				else if (i.first == "slave_id")
				{
					std::size_t id{ std::stoul(i.second) };
					if (id > cs.controller().slavePool().size())throw std::runtime_error("invalid param in basic parse func in param \"" + i.first + "\"");
					std::fill(param.active_motor_, param.active_motor_ + MAX_MOTOR_NUM, false);
					param.active_motor_[cs.controller().motionAtSla(id).motId()] = true;
				}
				else if (i.first == "limit_time")
				{
					param.limit_time_ = std::stoul(i.second);
				}
				else
				{
					throw std::runtime_error("unknown param in basic parse func in param \"" + i.first + "\"");
				}
			}
			msg_out.header().reserved1_ = 0;
			msg_out.header().reserved2_ = ControlServer::EXECUTE_RT_PLAN;
			msg_out.header().reserved3_ = ControlServer::WAIT_FOR_RT_PLAN_FINISHED;
			msg_out.copyStruct(param);
		}
		auto default_enable_plan(const aris::dynamic::PlanParam &plan_param)->int
		{
			auto &cs = aris::server::ControlServer::instance();
			auto param = reinterpret_cast<DefaultParam*>(plan_param.param_);

			bool is_all_enabled = true;
			for (std::size_t i = 0; i < cs.controller().motionPool().size(); ++i)
			{
				auto &cm = cs.controller().motionPool().at(i);
				if (param->active_motor_[i])
				{
					auto ret = cm.enable();
					if (ret)
					{
						is_all_enabled = false;

						if (plan_param.count_ % 1000 == 0)
						{
							cs.controller().mout() << "Unenabled motor, slave id: " << cm.id() << ", absolute id: " << i << ", ret: " << ret << '\0';
							cs.controller().mout().update();
							cs.controller().sendOut();
						}
					}
				}
			}

			return (is_all_enabled || param->limit_time_ <= plan_param.count_) ? 0 : 1;
		}
		auto default_disable_plan(const aris::dynamic::PlanParam &plan_param)->int
		{
			auto &cs = aris::server::ControlServer::instance();
			auto param = reinterpret_cast<DefaultParam*>(plan_param.param_);

			bool is_all_disabled = true;
			for (std::size_t i = 0; i < cs.controller().motionPool().size(); ++i)
			{
				auto &cm = cs.controller().motionPool().at(i);
				if (param->active_motor_[i])
				{
					auto ret = cm.disable();
					if (ret)
					{
						is_all_disabled = false;

						if (plan_param.count_ % 1000 == 0)
						{
							cs.controller().mout() << "Undisabled motor, slave id: " << cm.id() << ", absolute id: " << i << ", ret: " << ret << '\0';
							cs.controller().mout().update();
							cs.controller().sendOut();
						}
					}
				}
			}

			return (is_all_disabled || param->limit_time_ <= plan_param.count_) ? 0 : 1;
		}
		auto default_mode_plan(const aris::dynamic::PlanParam &plan_param)->int
		{
			auto &cs = aris::server::ControlServer::instance();
			auto param = reinterpret_cast<DefaultParam*>(plan_param.param_);

			bool is_all_moded = true;
			for (std::size_t i = 0; i < cs.controller().motionPool().size(); ++i)
			{
				auto &cm = cs.controller().motionPool().at(i);
				if (param->active_motor_[i])
				{
					auto ret = cm.mode(8);
					if (ret)
					{
						is_all_moded = false;

						if (plan_param.count_ % 1000 == 0)
						{
							cs.controller().mout() << "Unmoded motor, slave id: " << cm.id() << ", absolute id: " << i << ", ret: " << ret << '\0';
							cs.controller().mout().update();
							cs.controller().sendOut();
						}
					}
				}
			}

			return (is_all_moded || param->limit_time_ <= plan_param.count_) ? 0 : 1;
		}
		auto default_home_plan(const aris::dynamic::PlanParam &plan_param)->int
		{
			auto &cs = aris::server::ControlServer::instance();
			auto param = reinterpret_cast<DefaultParam*>(plan_param.param_);

			bool is_all_homed = true;
			for (std::size_t i = 0; i < cs.controller().motionPool().size(); ++i)
			{
				auto &cm = cs.controller().motionPool().at(i);
				if (param->active_motor_[i])
				{
					auto ret = cm.home();
					if (ret)
					{
						is_all_homed = false;

						if (plan_param.count_ % 1000 == 0)
						{
							cs.controller().mout() << "Unmoded motor, slave id: " << cm.id() << ", absolute id: " << i << ", ret: " << ret << '\0';
							cs.controller().mout().update();
							cs.controller().sendOut();
						}
					}
				}
			}

			return (is_all_homed || param->limit_time_ <= plan_param.count_) ? 0 : 1;
		}
		auto default_enable_command()->const aris::core::Command &{ return static_cast<const aris::core::Command &>(default_command_root().children().at(0)); }
		auto default_disable_command()->const aris::core::Command &{ return static_cast<const aris::core::Command &>(default_command_root().children().at(1)); }
		auto default_home_command()->const aris::core::Command &{ return static_cast<const aris::core::Command &>(default_command_root().children().at(2)); }
		auto default_mode_command()->const aris::core::Command &{ return static_cast<const aris::core::Command &>(default_command_root().children().at(3)); }

		class ControlServer::Imp
		{
		public:
			auto tg()->void;
			auto executeCmd()->int;
			auto onRunError()->int;

			Imp(ControlServer *server) :server_(server) {}
			Imp(const Imp&) = delete;

		private:
			std::recursive_mutex mu_running_;
			std::atomic_bool is_running_{ false };

			ControlServer *server_;

			// 实时循环中的步态参数 //
			enum { CMD_POOL_SIZE = 50 };
			aris::core::MsgFix<aris::control::Master::MAX_MSG_SIZE> msg_queue_[CMD_POOL_SIZE];
			int current_cmd_{ 0 }, cmd_num_{ 0 };
			std::uint32_t count_{ 1 };

			// 储存上一次motion的数据 //
			struct PVC { double p; double v; double c; };
			std::vector<PVC> last_target_motion_data_vec_;

			// 以下储存所有的命令的parse和plan函数 //
			std::map<std::string, int> cmd_id_map_;//store gait id in follow vector
			std::vector<dynamic::PlanFunction> plan_vec_;// store plan func
			std::vector<ParseFunc> parser_vec_; // store parse func

			// 储存Model, Controller, SensorRoot, WidgetRoot //
			aris::dynamic::Model* model_;
			aris::sensor::SensorRoot* sensor_root_;
			aris::control::Controller* controller_;
			aris::server::WidgetRoot* widget_root_;

			// 结束时的callback //
			std::function<void(void)> on_exit_callback_{ nullptr };

			friend class ControlServer;
		};
		auto ControlServer::Imp::tg()->void
		{
			// 查看是否有新cmd //
			if (server_->controller().recvIn())
			{
				if (cmd_num_ >= CMD_POOL_SIZE)
				{
					server_->controller().mout() << "cmd pool is full, thus ignore last command\n";
					// 结束同步调用的等待 //
					auto promise = reinterpret_cast<std::promise<void>*&>(server_->controller().msgIn().header().reserved3_);
					if (promise)promise->set_value();
				}
				else
				{
					msg_queue_[(current_cmd_ + cmd_num_) % CMD_POOL_SIZE] = server_->controller().msgIn();
					++cmd_num_;
				}
			}

			// 执行cmd queue中的cmd //
			if (cmd_num_ > 0)
			{
				if (executeCmd())
				{
					if (++count_ % 1000 == 0) server_->controller().mout() << "execute cmd in count: " << count_;
				}
				else
				{
					server_->controller().mout() << "cmd finished, spend " << count_ << " counts\n\n";
					count_ = 1;
					current_cmd_ = (current_cmd_ + 1) % CMD_POOL_SIZE;
					--cmd_num_;
				}
			}

			// 向外发送msg //
			server_->controller().mout().update();
			if(!server_->controller().msgOut().empty()) server_->controller().mout() << '\0';
			server_->controller().mout().update();
			server_->controller().sendOut();
		}
		auto ControlServer::Imp::executeCmd()->int
		{
			aris::dynamic::PlanParam plan_param{ model_, count_, msg_queue_[current_cmd_].data(), static_cast<std::uint32_t>(msg_queue_[current_cmd_].size()) };

			// 执行plan函数 //
			int ret = this->plan_vec_.at(static_cast<std::size_t>(msg_queue_[current_cmd_].header().reserved2_)).operator()(plan_param);

			// 控制电机 //
			for (std::size_t i = 0; i < controller_->motionPool().size(); ++i)
			{
				auto &cm = controller_->motionPool().at(i);
				auto &mm = model_->motionPool().at(i);
				
				if (mm.active())
				{
					if ((msg_queue_[current_cmd_].header().reserved1_ & USING_TARGET_POS))cm.setTargetPos(mm.mp());
					if ((msg_queue_[current_cmd_].header().reserved1_ & USING_TARGET_VEL))cm.setTargetVel(mm.mv());
					if ((msg_queue_[current_cmd_].header().reserved1_ & USING_TARGET_CUR))cm.setTargetCur(mm.mf());
					if ((msg_queue_[current_cmd_].header().reserved1_ & USING_VEL_OFFSET))cm.setOffsetVel(mm.mv());
					if ((msg_queue_[current_cmd_].header().reserved1_ & USING_CUR_OFFSET))cm.setOffsetCur(mm.mf());
				}
			}

			// 检查规划的指令是否合理（包括电机是否已经跟随上） //
			for (std::size_t i = 0; i < controller_->motionPool().size(); ++i)
			{
				auto &cm = controller_->motionPool().at(i);
				
				// check max pos //
				if ((msg_queue_[current_cmd_].header().reserved1_ & CHECK_POS_MAX) && (cm.targetPos() > cm.maxPos()))
				{
					server_->controller().mout() << "Motor " << cm.id() << " (sla id) target position is bigger than its MAX permitted value in count: " << count_ << "\n";
					server_->controller().mout() << "The min, max and current count using ABS sequence are:\n";
					for (auto &cm1 : controller_->motionPool())server_->controller().mout() << cm1.minPos() << "\t" << cm1.maxPos() << "\t" << cm1.targetPos() << "\n";
					onRunError();
					return 0;
				}

				// check min pos //
				if ((msg_queue_[current_cmd_].header().reserved1_ & CHECK_POS_MIN) && (cm.targetPos() < cm.minPos()))
				{
					server_->controller().mout() << "Motor " << cm.id() << " (sla id) target position is smaller than its MIN permitted value in count: " << count_ << "\n";
					server_->controller().mout() << "The min, max and current count using ABS sequence are:\n";
					for (auto &cm1 : controller_->motionPool())server_->controller().mout() << cm1.minPos() << "\t" << cm1.maxPos() << "\t" << cm1.targetPos() << "\n";
					onRunError();
					return 0;
				}

				// check plan pos continuous //
				if ((msg_queue_[current_cmd_].header().reserved1_ & CHECK_POS_PLAN_CONTINUOUS) && (std::abs(cm.targetPos() - last_target_motion_data_vec_.at(i).p) > 0.0012 * cm.maxVel()))
				{
					server_->controller().mout() << "Motor " << cm.id() << " (sla id) target position is not continuous in count: " << count_ << "\n";
					server_->controller().mout() << "The pin of last and this count using ABS sequence are:\n";
					for (std::size_t i = 0; i < controller_->motionPool().size(); ++i)server_->controller().mout() << last_target_motion_data_vec_.at(i).p << "\t" << controller_->motionPool().at(i).targetPos() << "\n";
					onRunError();
					return 0;
				}

				// check target and feedback pos //
				if ((msg_queue_[current_cmd_].header().reserved1_ & CHECK_POS_FOLLOWING_ERROR) && (std::abs(cm.targetPos() - cm.actualPos()) > 0.1 * cm.maxVel()))
				{
					server_->controller().mout() << "Motor " << cm.id() << " (sla id) target and feedback positions are not near in count: " << count_ << "\n";
					server_->controller().mout() << "The pin of target and feedback using ABS sequence are:\n";
					for (auto &cmp : controller_->motionPool())server_->controller().mout() << cmp.targetPos() << "\t" << cmp.actualPos() << "\n";
					onRunError();
					return 0;
				}
			}

			// 储存电机指令 //
			for (std::size_t i = 0; i < controller_->motionPool().size(); ++i)
			{
				last_target_motion_data_vec_.at(i).p = controller_->motionPool().at(i).targetPos();
				last_target_motion_data_vec_.at(i).v = controller_->motionPool().at(i).targetVel();
				last_target_motion_data_vec_.at(i).c = controller_->motionPool().at(i).targetCur();
			}

			// 如果是同步指令，那么通知等待线程结束 //
			auto promise = reinterpret_cast<std::promise<void>*&>(msg_queue_[current_cmd_].header().reserved3_);
			if (ret == 0 && promise)promise->set_value();
			return ret;
		}
		auto ControlServer::Imp::onRunError()->int
		{
			// 恢复电机状态 //
			for (std::size_t i = 0; i < controller_->motionPool().size(); ++i)
			{
				auto &cm = controller_->motionPool().at(i);
				switch (cm.modeOfDisplay())
				{
				case 8:
					cm.setTargetPos(cm.actualPos());
					break;
				case 9:
					cm.setTargetVel(0.0);
					break;
				case 10:
					cm.setTargetCur(0.0);
					break;
				default:
					cm.setTargetPos(cm.actualPos());
					cm.setTargetVel(0.0);
					cm.setTargetCur(0.0);
				}
			}
			
			// 结束所有等待的指令 //
			for (int i = -1; ++i < cmd_num_;)
			{
				auto promise = reinterpret_cast<std::promise<void>*&>(msg_queue_[current_cmd_ + i].header().reserved3_);
				if (promise)promise->set_value();
			}

			// 清理命令 //
			server_->controller().mout() << "All commands in command queue are discarded, please try to RECOVER\n";
			cmd_num_ = 1;//因为这里为0退出,因此之后在tg中回递减cmd_num_,所以这里必须为1
			count_ = 1;

			return 0;
		}
		auto ControlServer::instance()->ControlServer & { static ControlServer instance; return instance; }
		auto ControlServer::resetModel(dynamic::Model *model)->void 
		{ 
			auto iter = std::find_if(children().begin(), children().end(), [](const aris::core::Object &obj) { return obj.name() == "model"; });
			if(iter != children().end())children().erase(iter);
			children().push_back_ptr(model);
			imp_->model_ = model;
		}
		auto ControlServer::resetController(control::Controller *controller)->void 
		{
			auto iter = std::find_if(children().begin(), children().end(), [](const aris::core::Object &obj) { return obj.name() == "controller"; });
			if (iter != children().end())children().erase(iter);
			children().push_back_ptr(controller);
			imp_->controller_ = controller;
		}
		auto ControlServer::resetSensorRoot(sensor::SensorRoot *sensor_root)->void 
		{
			auto iter = std::find_if(children().begin(), children().end(), [](const aris::core::Object &obj) { return obj.name() == "sensor_root"; });
			if (iter != children().end())children().erase(iter);
			children().push_back_ptr(sensor_root);
			imp_->sensor_root_ = sensor_root;
		}
		auto ControlServer::resetWidgetRoot(server::WidgetRoot *widget_root)->void 
		{
			auto iter = std::find_if(children().begin(), children().end(), [](const aris::core::Object &obj) { return obj.name() == "widget_root"; });
			if (iter != children().end())children().erase(iter);
			children().push_back_ptr(widget_root);
			imp_->widget_root_ = widget_root;
		}
		auto ControlServer::widgetRoot()->WidgetRoot& { return *imp_->widget_root_; }
		auto ControlServer::model()->dynamic::Model& { return *imp_->model_; }
		auto ControlServer::controller()->control::Controller& { return *imp_->controller_; }
		auto ControlServer::sensorRoot()->sensor::SensorRoot& { return *imp_->sensor_root_; }
		auto ControlServer::loadXml(const aris::core::XmlElement &xml_ele)->void
		{
			Object::loadXml(xml_ele);
			imp_->controller_ = findOrInsert<aris::control::Controller>("controller");
			imp_->model_ = findOrInsert<aris::dynamic::Model>("model");
			imp_->sensor_root_ = findOrInsert<aris::sensor::SensorRoot>("sensor_root");
			imp_->widget_root_ = findOrInsert<aris::server::WidgetRoot>("widget_root");
		}
		auto ControlServer::addCmd(const std::string &cmd_name, const ParseFunc &parse_func, const aris::dynamic::PlanFunction &plan_func)->void
		{
			std::unique_lock<std::recursive_mutex> running_lck(imp_->mu_running_);
			if (imp_->is_running_)throw std::runtime_error("failed to ControlServer::addCmd, because it's already started");
			
			if (imp_->cmd_id_map_.find(cmd_name) != imp_->cmd_id_map_.end())
			{
				throw std::runtime_error(std::string("failed to add command, because \"") + cmd_name + "\" already exists");
			}
			else if (widgetRoot().cmdParser().commandPool().findByName(cmd_name) == widgetRoot().cmdParser().commandPool().end())
			{
				throw std::runtime_error(std::string("failed to add command, because ControlServer does not have \"") + cmd_name + "\" node");
			}
			else
			{
				imp_->plan_vec_.push_back(plan_func);
				imp_->parser_vec_.push_back(parse_func);
				imp_->cmd_id_map_.insert(std::make_pair(cmd_name, static_cast<int>(imp_->plan_vec_.size() - 1)));

				std::cout << cmd_name << ":" << imp_->cmd_id_map_.at(cmd_name) << std::endl;
			}
		}
		auto ControlServer::executeCmd(const std::string &cmd_string)->void
		{
			std::unique_lock<std::recursive_mutex> running_lck(imp_->mu_running_);
			if (!imp_->is_running_)throw std::runtime_error("failed in ControlServer::executeCmd, because ControlServer is not running");

			aris::core::log(cmd_string);

			std::string cmd;
			std::map<std::string, std::string> params;
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

			// parse msg //
			aris::core::Msg cmd_msg;
			auto cmd_pair = imp_->cmd_id_map_.find(cmd);
			if (cmd_pair == imp_->cmd_id_map_.end())throw std::runtime_error(std::string("command \"") + cmd + "\" does not have gait function, please AddCmd() first");
			cmd_msg.header().reserved1_ = CHECK_POS_MAX | CHECK_POS_MIN | CHECK_POS_PLAN_CONTINUOUS | CHECK_POS_FOLLOWING_ERROR | CHECK_VEL_PLAN_CONTINUOUS | CHECK_VEL_FOLLOWING_ERROR;
			cmd_msg.header().reserved2_ = EXECUTE_RT_PLAN;
			cmd_msg.header().reserved3_ = WAIT_FOR_RT_PLAN_FINISHED;
			imp_->parser_vec_.at(cmd_pair->second).operator()(cmd, params, cmd_msg);
			if (!(cmd_msg.header().reserved2_ & EXECUTE_RT_PLAN)) return;
			cmd_msg.header().reserved2_ = cmd_pair->second;// using reserved 2 to store gait id
			if (imp_->plan_vec_.at(cmd_pair->second) == nullptr)throw std::runtime_error(std::string("command \"") + cmd + "\" have invalid gait function, it's nullptr");

			// sync or async //
			if (cmd_msg.header().reserved3_ & WAIT_FOR_RT_PLAN_FINISHED)
			{
				std::promise<void> cmd_finish_promise;
				auto fut = cmd_finish_promise.get_future();
				reinterpret_cast<std::promise<void> *&>(cmd_msg.header().reserved3_) = &cmd_finish_promise;
				controller().sendIn(cmd_msg);
				fut.wait();
			}
			else
			{
				reinterpret_cast<std::promise<void> *&>(cmd_msg.header().reserved3_) = nullptr;
				controller().sendIn(cmd_msg);
			}
		}
		auto ControlServer::start()->void
		{
			std::unique_lock<std::recursive_mutex> running_lck(imp_->mu_running_);
			if (imp_->is_running_)throw std::runtime_error("failed to ControlServer::start, because it's already started");
			imp_->is_running_ = true;

			// 得到电机向量以及数据 //
			imp_->last_target_motion_data_vec_.clear();
			imp_->last_target_motion_data_vec_.resize(controller().slavePool().size(), Imp::PVC{ 0,0,0 });

			controller().setControlStrategy([this]() {this->imp_->tg(); });

			sensorRoot().start();
			controller().start();
		}
		auto ControlServer::stop()->void
		{
			std::unique_lock<std::recursive_mutex> running_lck(imp_->mu_running_);
			if (!imp_->is_running_)throw std::runtime_error("failed to ControlServer::stop, because it's not started");
			imp_->is_running_ = false;

			controller().stop();
			sensorRoot().stop();
		}
		ControlServer::~ControlServer() = default;
		ControlServer::ControlServer() :imp_(new Imp(this))
		{
			registerType<aris::dynamic::Model>();
			registerType<aris::control::Controller>();
			registerType<aris::sensor::SensorRoot>();
			registerType<aris::server::WidgetRoot>();

			// create instance //
			makeModel<aris::dynamic::Model>("model");
			makeController<aris::control::Controller>("controller");
			makeSensorRoot<aris::sensor::SensorRoot>("sensor_root");
			makeWidgetRoot<aris::server::WidgetRoot>("widget_root");
		}
	}
}
