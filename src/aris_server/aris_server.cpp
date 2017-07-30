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
		auto default_command_root()->const aris::core::Root &	{
			static aris::core::Root root;
			if (root.children().size() == 0)
			{
				root.registerChildType<aris::core::Command>();
				root.registerChildType<aris::core::Param>();
				root.registerChildType<aris::core::GroupParam>();
				root.registerChildType<aris::core::UniqueParam>();
				
				auto &en = root.add<aris::core::Command>("en", "all", "");
				en.add<aris::core::Param>("all", "", "", 'a');
				en.add<aris::core::Param>("motion_id", "0", "", 'm');
				en.add<aris::core::Param>("physical_id", "0", "", 'p');
				en.add<aris::core::Param>("slave_id", "0", "", 's');

				auto &ds = root.add<aris::core::Command>("ds", "all", "");
				ds.add<aris::core::Param>("all", "", "", 'a');
				ds.add<aris::core::Param>("motion_id", "0", "", 'm');
				ds.add<aris::core::Param>("physical_id", "0", "", 'p');
				ds.add<aris::core::Param>("slave_id", "0", "", 's');

				auto &hm = root.add<aris::core::Command>("hm", "all", "");
				hm.add<aris::core::Param>("all", "", "", 'a');
				hm.add<aris::core::Param>("motion_id", "0", "", 'm');
				hm.add<aris::core::Param>("physical_id", "0", "", 'p');
				hm.add<aris::core::Param>("slave_id", "0", "", 's');

				auto &md = root.add<aris::core::Command>("md", "all", "");
				md.add<aris::core::Param>("all", "", "", 'a');
				md.add<aris::core::Param>("motion_id", "0", "", 'm');
				md.add<aris::core::Param>("physical_id", "0", "", 'p');
				md.add<aris::core::Param>("slave_id", "0", "", 's');
			}
			return root;
		}
		
		auto default_parse(ControlServer &cs, const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg_out)->void
		{
			DefaultParam param;

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
					if (id < 0 || id > cs.master().slavePool().size())throw std::runtime_error("invalid param in basic parse func in param \"" + i.first + "\"");
					std::fill(param.active_motor_, param.active_motor_ + MAX_MOTOR_NUM, false);
					if (cs.model().motionAtSla(id).absID() >= cs.model().motionPool().size())throw std::runtime_error("invalid param in basic parse func in param \"" + i.first + "\", this slave is not motion");
					param.active_motor_[cs.model().motionAtSla(id).absID()] = true;
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
			for (std::size_t i = 0; i < plan_param.model_->motionPool().size(); ++i)
			{
				auto &cm = static_cast<aris::control::EthercatMotion&>(cs.master().slavePool().at(plan_param.model_->motionPool().at(i).slaID()));
				if (param->active_motor_[i])
				{
					auto ret = cm.enable();

					// 使能出错，直接返回并错误处理 //
					if (ret < 0)return -1;
					// 已经正常使能，什么都不做 //
					else if (ret == 0);
					// 还在使能过程中 //
					else
					{
						is_all_enabled = false;

						if (plan_param.count_ % 1000 == 0)
						{
							cs.master().mout() << "Unenabled motor, slave id: " << cm.id() << ", absolute id: " << i << ", ret: " << ret << "\n" << '\0';
							cs.master().mout().update();
							cs.master().sendOut();
						}
					}
				}
			}

			return is_all_enabled ? 0 : 1;
		}
		auto default_disable_plan(const aris::dynamic::PlanParam &plan_param)->int
		{
			auto &cs = aris::server::ControlServer::instance();
			auto param = reinterpret_cast<DefaultParam*>(plan_param.param_);

			bool is_all_enabled = true;
			for (std::size_t i = 0; i < plan_param.model_->motionPool().size(); ++i)
			{
				auto &cm = static_cast<aris::control::EthercatMotion&>(cs.master().slavePool().at(plan_param.model_->motionPool().at(i).slaID()));
				if (param->active_motor_[i])
				{
					auto ret = cm.disable();

					// 使能出错，直接返回并错误处理 //
					if (ret < 0)return -1;
					// 已经正常使能，什么都不做 //
					else if (ret == 0);
					// 还在使能过程中 //
					else
					{
						is_all_enabled = false;

						if (plan_param.count_ % 1000 == 0)
						{
							cs.master().mout() << "Undisabled motor, slave id: " << cm.id() << ", absolute id: " << i << ", ret: " << ret << "\n";
						}
					}
				}
			}

			return is_all_enabled ? 0 : 1;
		}
		auto default_mode_plan(const aris::dynamic::PlanParam &plan_param)->int
		{
			auto &cs = aris::server::ControlServer::instance();
			auto param = reinterpret_cast<DefaultParam*>(plan_param.param_);

			bool is_all_moded = true;
			for (std::size_t i = 0; i < plan_param.model_->motionPool().size(); ++i)
			{
				auto &cm = static_cast<aris::control::EthercatMotion&>(cs.master().slavePool().at(plan_param.model_->motionPool().at(i).slaID()));
				if (param->active_motor_[i])
				{
					auto ret = cm.mode(8);

					// 使能出错，直接返回并错误处理 //
					if (ret < 0)return -1;
					// 已经正常使能，什么都不做 //
					else if (ret == 0);
					// 还在使能过程中 //
					else
					{
						is_all_moded = false;

						if (plan_param.count_ % 1000 == 0)
						{
							cs.master().mout() << "Unmoded motor, slave id: " << cm.id() << ", absolute id: " << i << ", ret: " << ret << "\n" << '\0';
							cs.master().mout().update();
							cs.master().sendOut();
						}
					}
				}
			}

			return is_all_moded ? 0 : 1;
		}
		auto default_home_plan(const aris::dynamic::PlanParam &plan_param)->int
		{
			auto &cs = aris::server::ControlServer::instance();
			auto param = reinterpret_cast<DefaultParam*>(plan_param.param_);

			bool is_all_homed = true;
			for (std::size_t i = 0; i < plan_param.model_->motionPool().size(); ++i)
			{
				auto &cm = static_cast<aris::control::EthercatMotion&>(cs.master().slavePool().at(plan_param.model_->motionPool().at(i).slaID()));
				if (param->active_motor_[i])
				{
					auto ret = cm.home();

					// 使能出错，直接返回并错误处理 //
					if (ret < 0)return -1;
					// 已经正常使能，什么都不做 //
					else if (ret == 0);
					// 还在使能过程中 //
					else
					{
						is_all_homed = false;

						if (plan_param.count_ % 1000 == 0)
						{
							cs.master().mout() << "Unmoded motor, slave id: " << cm.id() << ", absolute id: " << i << ", ret: " << ret << "\n" << '\0';
							cs.master().mout().update();
							cs.master().sendOut();
						}
					}
				}
			}

			return is_all_homed ? 0 : 1;
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
			int current_cmd_{ 0 }, cmd_num_{ 0 }, count_{ 1 };

			// 储存上一次motion的数据 //
			struct PVC { double p; double v; double c; };
			std::vector<PVC> last_target_motion_data_vec_;
			std::vector<aris::control::EthercatMotion *> cm_vec_;


			// 以下储存所有的命令的parse和plan函数 //
			std::map<std::string, int> cmd_id_map_;//store gait id in follow vector
			std::vector<dynamic::PlanFunction> plan_vec_;// store plan func
			std::vector<ParseFunc> parser_vec_; // store parse func

			// 储存Model, Controller, SensorRoot, WidgetRoot //
			std::unique_ptr<aris::dynamic::Model> model_;
			std::unique_ptr<aris::sensor::SensorRoot> sensor_root_;
			std::unique_ptr<aris::control::Master> master_;
			std::unique_ptr<aris::server::WidgetRoot> widget_root_;

			// 结束时的callback //
			std::function<void(void)> on_exit_callback_{ nullptr };

			friend class ControlServer;
		};
		auto ControlServer::Imp::tg()->void
		{
			// 查看是否有新cmd //
			if (server_->master().recvIn())
			{
				if (cmd_num_ >= CMD_POOL_SIZE)
				{
					server_->master().mout() << "cmd pool is full, thus ignore last command\n";
					// 结束同步调用的等待 //
					auto promise = reinterpret_cast<std::promise<void>*&>(server_->master().msgIn().header().reserved3_);
					if (promise)promise->set_value();
				}
				else
				{
					msg_queue_[(current_cmd_ + cmd_num_) % CMD_POOL_SIZE] = server_->master().msgIn();
					++cmd_num_;
				}
			}

			// 执行cmd queue中的cmd //
			if (cmd_num_ > 0)
			{
				if (executeCmd())
				{
					if (++count_ % 1000 == 0) server_->master().mout() << "execute cmd in count: " << count_;
				}
				else
				{
					server_->master().mout() << "cmd finished, spend " << count_ << " counts\n\n";
					count_ = 1;
					current_cmd_ = (current_cmd_ + 1) % CMD_POOL_SIZE;
					--cmd_num_;
				}
			}

			// 向外发送msg //
			server_->master().mout().update();
			if(!server_->master().msgOut().empty()) server_->master().mout() << '\0';
			server_->master().mout().update();
			server_->master().sendOut();
		}
		auto ControlServer::Imp::executeCmd()->int
		{
			aris::dynamic::PlanParam plan_param{ model_.get(), count_, msg_queue_[current_cmd_].data(), msg_queue_[current_cmd_].size() };

			// 执行plan函数 //
			int ret = this->plan_vec_.at(static_cast<std::size_t>(msg_queue_[current_cmd_].header().reserved2_)).operator()(plan_param);

			// 控制电机 //
			for (std::size_t i = 0; i < cm_vec_.size(); ++i)
			{
				auto &cm = *cm_vec_.at(i);
				auto &mm = model_->motionAtPhy(i);
				
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
			for (std::size_t i = 0; i < cm_vec_.size(); ++i)
			{
				auto &cm = *cm_vec_.at(i);
				
				// check max pos //
				if ((msg_queue_[current_cmd_].header().reserved1_ & CHECK_POS_MAX) && (cm.targetPos() > cm.maxPos()))
				{
					server_->master().mout() << "Motor " << cm.id() << " (sla id) target position is bigger than its MAX permitted value in count: " << count_ << "\n";
					server_->master().mout() << "The min, max and current count using ABS sequence are:\n";
					for (auto &cmp : cm_vec_)server_->master().mout() << cmp->minPos() << "\t" << cmp->maxPos() << "\t" << cmp->targetPos() << "\n";
					onRunError();
					return 0;
				}

				// check min pos //
				if ((msg_queue_[current_cmd_].header().reserved1_ & CHECK_POS_MIN) && (cm.targetPos() < cm.minPos()))
				{
					server_->master().mout() << "Motor " << cm.id() << " (sla id) target position is smaller than its MIN permitted value in count: " << count_ << "\n";
					server_->master().mout() << "The min, max and current count using ABS sequence are:\n";
					for (auto &cmp : cm_vec_)server_->master().mout() << cmp->minPos() << "\t" << cmp->maxPos() << "\t" << cmp->targetPos() << "\n";
					onRunError();
					return 0;
				}

				// check plan pos continuous //
				if ((msg_queue_[current_cmd_].header().reserved1_ & CHECK_POS_PLAN_CONTINUOUS) && (std::abs(cm.targetPos() - last_target_motion_data_vec_.at(i).p) > 0.0012 * cm.maxVel()))
				{
					server_->master().mout() << "Motor " << cm.id() << " (sla id) target position is not continuous in count: " << count_ << "\n";
					server_->master().mout() << "The pin of last and this count using ABS sequence are:\n";
					for (std::size_t i = 0; i < cm_vec_.size(); ++i)server_->master().mout() << last_target_motion_data_vec_.at(i).p << "\t" << cm_vec_[i]->targetPos() << "\n";
					onRunError();
					return 0;
				}

				// check target and feedback pos //
				if ((msg_queue_[current_cmd_].header().reserved1_ & CHECK_POS_FOLLOWING_ERROR) && (std::abs(cm.targetPos() - cm.actualPos()) > 0.1 * cm.maxVel()))
				{
					server_->master().mout() << "Motor " << cm.id() << " (sla id) target and feedback positions are not near in count: " << count_ << "\n";
					server_->master().mout() << "The pin of target and feedback using ABS sequence are:\n";
					for (auto &cmp : cm_vec_)server_->master().mout() << cmp->targetPos() << "\t" << cmp->actualPos() << "\n";
					onRunError();
					return 0;
				}
			}

			// 储存电机指令 //
			for (std::size_t i = 0; i < cm_vec_.size(); ++i)
			{
				last_target_motion_data_vec_.at(i).p = cm_vec_.at(i)->targetPos();
				last_target_motion_data_vec_.at(i).v = cm_vec_.at(i)->targetVel();
				last_target_motion_data_vec_.at(i).c = cm_vec_.at(i)->targetCur();
			}

			// 如果是同步指令，那么通知等待线程结束 //
			auto promise = reinterpret_cast<std::promise<void>*&>(msg_queue_[current_cmd_].header().reserved3_);
			if (ret == 0 && promise)promise->set_value();
			return ret;
		}
		auto ControlServer::Imp::onRunError()->int
		{
			// 恢复电机状态 //
			for (std::size_t i = 0; i < cm_vec_.size(); ++i)
			{
				auto &cm = *cm_vec_.at(i);
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
			server_->master().mout() << "All commands in command queue are discarded, please try to RECOVER\n";
			cmd_num_ = 1;//因为这里为0退出,因此之后在tg中回递减cmd_num_,所以这里必须为1
			count_ = 1;

			return 0;
		}
		auto ControlServer::instance()->ControlServer & { static ControlServer instance; return instance; }
		auto ControlServer::resetModel(dynamic::Model *model)->void { imp_->model_.reset(model); }
		auto ControlServer::resetMaster(control::Master *controller)->void { imp_->master_.reset(controller); }
		auto ControlServer::resetSensorRoot(sensor::SensorRoot *sensor_root)->void { imp_->sensor_root_.reset(sensor_root); }
		auto ControlServer::resetWidgetRoot(server::WidgetRoot *widget_root)->void { imp_->widget_root_.reset(widget_root); }
		auto ControlServer::widgetRoot()->WidgetRoot& { return std::ref(*imp_->widget_root_); }
		auto ControlServer::model()->dynamic::Model& { return std::ref(*imp_->model_); }
		auto ControlServer::master()->control::Master& { return std::ref(*imp_->master_); }
		auto ControlServer::sensorRoot()->sensor::SensorRoot& { return std::ref(*imp_->sensor_root_); }
		auto ControlServer::saveXml(const char *file_name)->void
		{
			aris::core::XmlDocument xml_doc;
			saveXml(xml_doc);
			xml_doc.SaveFile(file_name);
		}
		auto ControlServer::saveXml(aris::core::XmlDocument &xml_doc)->void
		{
			xml_doc.Clear();

			auto header_xml_ele = xml_doc.NewDeclaration("xml version=\"1.0\" encoding=\"UTF-8\" ");
			xml_doc.InsertEndChild(header_xml_ele);

			auto root_xml_ele = xml_doc.NewElement("Root");
			xml_doc.InsertEndChild(root_xml_ele);

			saveXml(*root_xml_ele);
		}
		auto ControlServer::saveXml(aris::core::XmlElement &xml_ele)->void
		{
			xml_ele.DeleteChildren();
			
			auto model_xml_ele = xml_ele.GetDocument()->NewElement("model");
			xml_ele.InsertEndChild(model_xml_ele);
			model().saveXml(*model_xml_ele);

			auto master_xml_ele = xml_ele.GetDocument()->NewElement("master");
			xml_ele.InsertEndChild(master_xml_ele);
			master().saveXml(*master_xml_ele);
			
			auto sensor_root_xml_ele = xml_ele.GetDocument()->NewElement("sensor_root");
			xml_ele.InsertEndChild(sensor_root_xml_ele);
			sensorRoot().saveXml(*sensor_root_xml_ele);

			auto widget_root_xml_ele = xml_ele.GetDocument()->NewElement("widget_root");
			xml_ele.InsertEndChild(widget_root_xml_ele);
			widgetRoot().saveXml(*widget_root_xml_ele);
		}
		auto ControlServer::loadXml(const char *file_name)->void
		{
			aris::core::XmlDocument doc;

			if (doc.LoadFile(file_name) != 0)throw std::logic_error((std::string("could not open file:") + std::string(file_name)));

			loadXml(doc);
		}
		auto ControlServer::loadXml(const aris::core::XmlDocument &xml_doc)->void
		{
			// load robot model_ //
			imp_->model_->loadXml(xml_doc);
			imp_->master_->loadXml(xml_doc);
			imp_->sensor_root_->loadXml(xml_doc);
			imp_->widget_root_->loadXml(xml_doc);
		}
		auto ControlServer::addCmd(const std::string &cmd_name, const ParseFunc &parse_func, const aris::dynamic::PlanFunction &plan_func)->void
		{
			std::unique_lock<std::recursive_mutex> running_lck(imp_->mu_running_);
			if (imp_->is_running_)throw std::runtime_error("ControlServer already running, so cannot addCmd");
			
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
			imp_->parser_vec_.at(cmd_pair->second).operator()(*this, cmd, params, cmd_msg);
			if (!(cmd_msg.header().reserved2_ & EXECUTE_RT_PLAN)) return;
			cmd_msg.header().reserved2_ = cmd_pair->second;// using reserved 2 to store gait id
			if (imp_->plan_vec_.at(cmd_pair->second) == nullptr)throw std::runtime_error(std::string("command \"") + cmd + "\" have invalid gait function, it's nullptr");

			// sync or async //
			if (cmd_msg.header().reserved3_ & WAIT_FOR_RT_PLAN_FINISHED)
			{
				std::promise<void> cmd_finish_promise;
				auto fut = cmd_finish_promise.get_future();
				reinterpret_cast<std::promise<void> *&>(cmd_msg.header().reserved3_) = &cmd_finish_promise;
				master().sendIn(cmd_msg);
				fut.wait();
			}
			else
			{
				reinterpret_cast<std::promise<void> *&>(cmd_msg.header().reserved3_) = nullptr;
				master().sendIn(cmd_msg);
			}
		}
		auto ControlServer::start()->void
		{
			std::unique_lock<std::recursive_mutex> running_lck(imp_->mu_running_);
			if (imp_->is_running_)throw std::runtime_error("ControlServer already running, so cannot start");
			imp_->is_running_ = true;

			// 得到电机向量以及数据 //
			imp_->cm_vec_.clear();
			for (auto &cm : master().slavePool())if (dynamic_cast<aris::control::EthercatMotion*>(&cm))imp_->cm_vec_.push_back(dynamic_cast<aris::control::EthercatMotion*>(&cm));
			imp_->last_target_motion_data_vec_.clear();
			imp_->last_target_motion_data_vec_.resize(imp_->cm_vec_.size(), Imp::PVC{ 0,0,0 });

			master().setControlStrategy([this]() {this->imp_->tg(); });

			master().start();
			sensorRoot().start();
		}
		auto ControlServer::stop()->void
		{
			std::unique_lock<std::recursive_mutex> running_lck(imp_->mu_running_);
			if (!imp_->is_running_)throw std::runtime_error("ControlServer is not running, so can't stop");
			imp_->is_running_ = false;

			master().stop();
			sensorRoot().stop();
		}
		ControlServer::~ControlServer() = default;
		ControlServer::ControlServer() :imp_(new Imp(this))
		{
			// create instance //
			makeModel<aris::dynamic::Model>();
			makeMaster<aris::control::Master>();
			makeSensorRoot<aris::sensor::SensorRoot>();
			makeWidgetRoot<aris::server::WidgetRoot>();
		}
	}
}
