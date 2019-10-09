#include <cstring>
#include <thread>
#include <algorithm>
#include <memory>
#include <cinttypes>
#include <queue>

#include <aris/core/core.hpp>
#include <aris/control/control.hpp>

#include "aris/server/ui.hpp"
#include "aris/server/control_server.hpp"

namespace aris::plan
{
	struct Plan::Imp
	{
		std::int64_t count_;

		aris::dynamic::Model *model_;
		aris::control::Master *master_;
		aris::control::Controller *controller_;
		aris::control::EthercatMaster *ec_master_;
		aris::control::EthercatController *ec_controller_;
		aris::server::ControlServer *cs_;

		std::uint64_t option_;
		std::vector<std::uint64_t> mot_options_;

		std::string cmd_msg_;
		std::string cmd_name_;
		std::map<std::string, std::string> cmd_params_;

		std::int64_t begin_global_count_;
		std::uint64_t command_id_;
		aris::control::Master::RtStasticsData rt_stastic_;

		std::any param;
		std::any ret;
		std::int32_t ret_code;
		char ret_msg[1024];
	};
}

namespace aris::server
{
	struct ControlServer::Imp
	{
		struct InternalData
		{
			std::shared_ptr<aris::plan::Plan> plan_;
			std::function<void(aris::plan::Plan&)> post_callback_;

			~InternalData() { if (post_callback_)post_callback_(*plan_); }
		};
		
		auto tg()->void;
		auto executeCmd(aris::plan::Plan &plan)->int;
		auto checkMotion(const std::uint64_t *mot_options, char *error_msg, std::int64_t count_)->int;
		auto startReturnThread()->void;

		Imp(ControlServer *server) :server_(server) {}
		Imp(const Imp&) = delete;

		std::recursive_mutex mu_running_;
		std::atomic_bool is_running_{ false };

		ControlServer *server_;

		// mem pool //
		std::vector<char> mempool_;


		// 实时循环中的轨迹参数 //
		enum { CMD_POOL_SIZE = 1000 };
		std::shared_ptr<InternalData> internal_data_queue_[CMD_POOL_SIZE];
		
		// 全局count //
		std::atomic<std::int64_t> global_count_{ 0 };

		// cmd系列参数
		std::atomic<std::int64_t> cmd_now_, cmd_end_, cmd_collect_;

		// collect系列参数
		std::thread collect_thread_;
		std::atomic_bool is_collect_running_;

		// 储存上一次motion的数据 //
		struct PVC { double p; double v; double c; };
		PVC *last_pvc_, *last_last_pvc_;

		// 全局的电机check选项
		std::uint64_t *global_mot_options_;

		// 储存Model, Controller, SensorRoot, PlanRoot //
		aris::dynamic::Model* model_;
		aris::control::Controller* controller_;
		aris::sensor::SensorRoot* sensor_root_;
		aris::plan::PlanRoot* plan_root_;
		aris::core::ObjectPool<aris::server::Interface> *interface_pool_;
		InterfaceRoot *interface_root_;

		// 打洞，读取数据 //
		std::atomic_bool if_get_data_{ false }, if_get_data_ready_{ false };
		const std::function<void(ControlServer&, const aris::plan::Plan *, std::any&)>* get_data_func_;
		std::any *get_data_;

		// callbacks //
		std::atomic<PreCallback> pre_callback_{ nullptr };
		std::atomic<PostCallback> post_callback_{ nullptr };

		// execute in cmd line
		std::function<void(aris::plan::Plan&)> cmdline_post_callback_;
		aris::core::Msg cmdline_msg_;
		std::atomic_bool cmdline_msg_received_ = false;
		std::shared_ptr<std::promise<std::shared_ptr<aris::plan::Plan>>> cmdline_execute_promise_;
	};
	auto ControlServer::Imp::tg()->void
	{
		// pre callback //
		if (auto call = pre_callback_.load())call(ControlServer::instance());
		
		// get atomic variables
		auto global_count = ++global_count_; // 原子操作
		auto cmd_now = cmd_now_.load();//原子操作
		auto cmd_end = cmd_end_.load();//原子操作

		// global error code, 存储上一次的错误 //
		static int idle_error_code{ 0 };

		char errmsg[1024];
		// 执行cmd queue中的cmd //
		if (cmd_end > cmd_now)
		{
			auto &plan = *internal_data_queue_[cmd_now % CMD_POOL_SIZE]->plan_;

			// 在第一回合初始化，包括log，初始化target等 //
			if (++plan.imp_->count_ == 1)
			{
				// 初始化target
				plan.imp_->begin_global_count_ = global_count;

				// 创建rt_log文件 //
				char name[1000];
				std::sprintf(name, "%" PRId64 "", plan.commandId());
				server_->controller().logFile(name);

				// 初始化统计数据 //
				server_->controller().resetRtStasticData(&plan.rtStastic(), true);
			}

			// 执行命令
			auto ret = executeCmd(plan);

			// 检查错误 //
			if (auto check_ret = checkMotion(plan.motionOptions().data(), plan.imp_->ret_msg, plan.count()); check_ret < 0)
			{
				// print info //
				server_->controller().mout() << "check failed, cmd queue cleared\n";

				// finish //
				plan.imp_->ret_code = check_ret;
				cmd_now_.store(cmd_end);// 原子操作
				server_->controller().resetRtStasticData(nullptr, false);
				server_->controller().lout() << std::flush;
			}
			// 非正常结束 //
			else if (ret < 0)
			{
				// print info //
				server_->controller().mout() << "user execute failed, cmd queue cleared\n";
				
				// finish //
				plan.imp_->ret_code = ret;
				cmd_now_.store(cmd_end);// 原子操作
				server_->controller().resetRtStasticData(nullptr, false);
				server_->controller().lout() << std::flush;
			}
			// 命令正常结束，结束统计数据 //
			else if (ret == 0)
			{
				// print info //
				if (!(plan.option() & aris::plan::Plan::NOT_PRINT_EXECUTE_COUNT))
					server_->controller().mout() << "cmd finished, spend " << plan.imp_->count_ << " counts\n\n";
				
				// finish //
				plan.imp_->ret_code = aris::plan::Plan::SUCCESS;
				cmd_now_.store(cmd_now + 1);//原子操作
				server_->controller().resetRtStasticData(nullptr, false);
				server_->controller().lout() << std::flush;
			}
			// 命令仍在执行 //
			else
			{
				// print info //
				if (plan.imp_->count_ % 1000 == 0 && !(plan.option() & aris::plan::Plan::NOT_PRINT_EXECUTE_COUNT))
					server_->controller().mout() << "execute cmd in count: " << plan.imp_->count_ << "\n";
			}
		}
		else if (auto error_code = idle_error_code; idle_error_code = checkMotion(global_mot_options_, errmsg, 0) && idle_error_code != error_code)
		{
			// 只有错误代码改变时，才会打印 //
			server_->controller().mout() << "failed when idle " << idle_error_code << "\n";
		}

		// 给与外部想要的数据 //
		if (if_get_data_.exchange(false)) // 原子操作
		{
			get_data_func_->operator()(ControlServer::instance(), cmd_end > cmd_now ? &*internal_data_queue_[cmd_now % CMD_POOL_SIZE]->plan_ : nullptr, *get_data_);
			if_get_data_ready_.store(true); // 原子操作
		}

		// post callback //
		if (auto call = post_callback_.load())call(ControlServer::instance());
	}
	auto ControlServer::Imp::executeCmd(aris::plan::Plan &plan)->int
	{
		//target.count = count_;
		aris::plan::Plan target;

		// 执行plan函数 //
		int ret = plan.executeRT();

		// 控制电机 //
		for (std::size_t i = 0; i < std::min(controller_->motionPool().size(), model_->motionPool().size()); ++i)
		{
			auto &cm = controller_->motionPool()[i];
			auto &mm = model_->motionPool()[i];

			if (mm.active())
			{
				if ((plan.motionOptions()[i] & aris::plan::Plan::USE_TARGET_POS))cm.setTargetPos(mm.mp());
				if ((plan.motionOptions()[i] & aris::plan::Plan::USE_TARGET_VEL))cm.setTargetVel(mm.mv());
				if ((plan.motionOptions()[i] & aris::plan::Plan::USE_TARGET_TOQ))cm.setTargetToq(mm.mf());
				if ((plan.motionOptions()[i] & aris::plan::Plan::USE_OFFSET_VEL))cm.setOffsetVel(mm.mv());
				if ((plan.motionOptions()[i] & aris::plan::Plan::USE_OFFSET_TOQ))cm.setOffsetToq(mm.mf());
			}
		}

		return ret;
	}
	auto ControlServer::Imp::checkMotion(const std::uint64_t *mot_options, char *error_msg, std::int64_t count_)->int
	{
		static int error_code = aris::plan::Plan::SUCCESS;
		static bool is_correcting{ false };
		if (is_correcting)goto FAILED;

		error_code = aris::plan::Plan::SUCCESS;

		// 检查规划的指令是否合理（包括电机是否已经跟随上） //
		for (std::size_t i = 0; i < controller_->motionPool().size(); ++i)
		{
			auto &cm = controller_->motionPool()[i];
			auto &ld = last_pvc_[i];
			auto &lld = last_last_pvc_[i];
			auto option = mot_options[i];

#ifndef WIN32
			if (!(option & aris::plan::Plan::NOT_CHECK_ENABLE)
				&& ((cm.statusWord() & 0x6f) != 0x27))
			{
				error_code = aris::plan::Plan::MOTION_NOT_ENABLED;
				sprintf(error_msg, "%s_%d:\nMotion %zd is not in OPERATION_ENABLE mode in count %zd\n", __FILE__, __LINE__, i, count_);
				goto FAILED;
			}
#endif
			if ((cm.statusWord() & 0x6f) == 0x27)
			{
				switch (cm.modeOfOperation())
				{
				case 8:
				{
					// check pos max //
					if (!(option & aris::plan::Plan::NOT_CHECK_POS_MAX)
						&& (cm.targetPos() > cm.maxPos())
						&& (cm.targetPos() > ld.p))
					{
						error_code = aris::plan::Plan::MOTION_POS_BEYOND_MAX;
						sprintf(error_msg, "%s_%d:\nMotion %zu target position beyond MAX in count %zu:\nmax: %lf\tnow: %lf\n", __FILE__, __LINE__, i, count_, cm.maxPos(), cm.targetPos());
						goto FAILED;
					}

					// check pos min //
					if (!(option & aris::plan::Plan::NOT_CHECK_POS_MIN)
						&& (cm.targetPos() < cm.minPos())
						&& (cm.targetPos() < ld.p))
					{
						error_code = aris::plan::Plan::MOTION_POS_BEYOND_MIN;
						sprintf(error_msg, "%s_%d:\nMotion %zu target position beyond MIN in count %zu:\nmin: %lf\tnow: %lf\n", __FILE__, __LINE__, i, count_, cm.minPos(), cm.targetPos());
						goto FAILED;
					}

					// check pos continuous //
					if (!(option & aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS)
						&& ((cm.targetPos() - ld.p) > 0.001 * cm.maxVel() || (cm.targetPos() - ld.p) < 0.001 * cm.minVel()))
					{
						error_code = aris::plan::Plan::MOTION_POS_NOT_CONTINUOUS;
						sprintf(error_msg, "%s_%d:\nMotion %zu target position NOT CONTINUOUS in count %zu:\nlast: %lf\tnow: %lf\n", __FILE__, __LINE__, i, count_, last_pvc_[i].p, cm.targetPos());
						goto FAILED;
					}

					// check pos continuous second order //
					if (!(option & aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER)
						&& ((cm.targetPos() + lld.p - 2 * ld.p) > 1e-6 * cm.maxAcc() || (cm.targetPos() + lld.p - 2 * ld.p) < 1e-6 * cm.minAcc()))
					{
						error_code = aris::plan::Plan::MOTION_POS_NOT_CONTINUOUS_SECOND_ORDER;
						sprintf(error_msg, "%s_%d:\nMotion %zu target position NOT SECOND CONTINUOUS in count %zu:\nlast last: %lf\tlast: %lf\tnow: %lf\n", __FILE__, __LINE__, i, count_, lld.p, last_pvc_[i].p, cm.targetPos());
						goto FAILED;
					}

					// check pos following error //
					if (!(option & aris::plan::Plan::NOT_CHECK_POS_FOLLOWING_ERROR)
						&& (std::abs(cm.targetPos() - cm.actualPos()) > cm.maxPosFollowingError()))
					{
						error_code = aris::plan::Plan::MOTION_POS_FOLLOWING_ERROR;
						sprintf(error_msg, "%s_%d:\nMotion %zu target position has FOLLOW ERROR in count %zu:\nactual: %lf\ttarget: %lf\n", __FILE__, __LINE__, i, count_, cm.actualPos(), cm.targetPos());
						goto FAILED;
					}

					break;
				}
				case 9:
				{
					// check vel max //
					if (!(option & aris::plan::Plan::NOT_CHECK_VEL_MAX)
						&& (cm.targetVel() > cm.maxVel()))
					{
						error_code = aris::plan::Plan::MOTION_VEL_BEYOND_MAX;
						sprintf(error_msg, "%s_%d:\nMotion %zu target velocity beyond MAX in count %zu:\nmax: %lf\tnow: %lf\n", __FILE__, __LINE__, i, count_, cm.maxVel(), cm.targetVel());
						goto FAILED;
					}

					// check vel min //
					if (!(option & aris::plan::Plan::NOT_CHECK_VEL_MIN)
						&& (cm.targetVel() < cm.minVel()))
					{
						error_code = aris::plan::Plan::MOTION_VEL_BEYOND_MIN;
						sprintf(error_msg, "%s_%d:\nMotion %zu target velocity beyond MIN in count %zu:\nmin: %lf\tnow: %lf\n", __FILE__, __LINE__, i, count_, cm.minVel(), cm.targetVel());
						goto FAILED;
					}

					// check vel continuous //
					if (!(option & aris::plan::Plan::NOT_CHECK_VEL_CONTINUOUS)
						&& ((cm.targetVel() - ld.v) > 0.001 * cm.maxAcc() || (cm.targetVel() - ld.v) < 0.001 * cm.minAcc()))
					{
						error_code = aris::plan::Plan::MOTION_VEL_NOT_CONTINUOUS;
						sprintf(error_msg, "%s_%d:\nMotion %zu target velocity NOT CONTINUOUS in count %zu:\nlast: %lf\tnow: %lf\n", __FILE__, __LINE__, i, count_, last_pvc_[i].v, cm.targetVel());
						goto FAILED;
					}

					// check vel following error //
					if (!(option & aris::plan::Plan::NOT_CHECK_VEL_FOLLOWING_ERROR)
						&& (std::abs(cm.targetVel() - cm.actualVel()) > cm.maxVelFollowingError()))
					{
						error_code = aris::plan::Plan::MOTION_VEL_FOLLOWING_ERROR;
						sprintf(error_msg, "%s_%d:\nMotion %zu target velocity has FOLLOW ERROR in count %zu:\nactual: %lf\ttarget: %lf\n", __FILE__, __LINE__, i, count_, cm.actualVel(), cm.targetVel());
						goto FAILED;
					}

					break;
				}
				case 10:
				{
					// check pos max //
					if (!(option & aris::plan::Plan::NOT_CHECK_POS_MAX)
						&& (cm.actualPos() > cm.maxPos()))
					{
						error_code = aris::plan::Plan::MOTION_POS_BEYOND_MAX;
						sprintf(error_msg, "%s_%d:\nMotion %zu target position beyond MAX in count %zu:\nmax: %lf\tnow: %lf\n", __FILE__, __LINE__, i, count_, cm.maxPos(), cm.targetPos());
						goto FAILED;
					}

					// check pos min //
					if (!(option & aris::plan::Plan::NOT_CHECK_POS_MIN)
						&& (cm.actualPos() < cm.minPos()))
					{
						error_code = aris::plan::Plan::MOTION_POS_BEYOND_MIN;
						sprintf(error_msg, "%s_%d:\nMotion %zu target position beyond MIN in count %zu:\nmin: %lf\tnow: %lf\n", __FILE__, __LINE__, i, count_, cm.minPos(), cm.targetPos());
						goto FAILED;
					}

					// check vel max //
					if (!(option & aris::plan::Plan::NOT_CHECK_VEL_MAX)
						&& (cm.actualVel() > cm.maxVel()))
					{
						error_code = aris::plan::Plan::MOTION_VEL_BEYOND_MAX;
						sprintf(error_msg, "%s_%d:\nMotion %zu target velocity beyond MAX in count %zu:\nmax: %lf\tnow: %lf\n", __FILE__, __LINE__, i, count_, cm.maxVel(), cm.actualVel());
						goto FAILED;
					}

					// check vel min //
					if (!(option & aris::plan::Plan::NOT_CHECK_VEL_MIN)
						&& (cm.actualVel() < cm.minVel()))
					{
						error_code = aris::plan::Plan::MOTION_VEL_BEYOND_MIN;
						sprintf(error_msg, "%s_%d:\nMotion %zu target velocity beyond MIN in count %zu:\nmin: %lf\tnow: %lf\n", __FILE__, __LINE__, i, count_, cm.minVel(), cm.actualVel());
						goto FAILED;
					}

					// check vel continuous //
					if (!(option & aris::plan::Plan::NOT_CHECK_VEL_CONTINUOUS)
						&& ((cm.actualVel() - ld.v) > 0.001 * cm.maxAcc() || (cm.actualVel() - ld.v) < 0.001 * cm.minAcc()))
					{
						error_code = aris::plan::Plan::MOTION_VEL_NOT_CONTINUOUS;
						sprintf(error_msg, "%s_%d:\nMotion %zu velocity NOT CONTINUOUS in count %zu:\nlast: %lf\tnow: %lf\n", __FILE__, __LINE__, i, count_, last_pvc_[i].p, cm.targetPos());
						goto FAILED;
					}
					break;
				}
				default:
				{
					// invalid mode //
					error_code = aris::plan::Plan::MOTION_INVALID_MODE;
					sprintf(error_msg, "%s_%d:\nMotion %zu MODE INVALID in count %zu:\nmode: %d\n", __FILE__, __LINE__, i, count_, cm.modeOfOperation());
					goto FAILED;
				}
				}
			}
		}

		// 储存电机指令 //
		for (std::size_t i = 0; i < controller_->motionPool().size(); ++i)
		{
			last_last_pvc_[i].p = controller_->motionPool()[i].targetPos();
			last_last_pvc_[i].v = controller_->motionPool()[i].targetVel();
			last_last_pvc_[i].c = controller_->motionPool()[i].targetToq();
		}
		std::swap(last_pvc_, last_last_pvc_);
		return 0;

	FAILED:
		is_correcting = false;
		for (std::size_t i = 0; i < controller_->motionPool().size(); ++i)
		{
			// correct
			auto &cm = controller_->motionPool().at(i);
			switch (cm.modeOfOperation())
			{
			case 8:
				cm.setTargetPos(cm.actualPos());
				//is_correcting = cm.disable();
				break;
			case 9:
				cm.setTargetVel(0.0);
				//is_correcting = cm.disable();
				break;
			case 10:
				cm.setTargetToq(0.0);
				is_correcting = cm.disable() || is_correcting;
				break;
			default:
				is_correcting = cm.disable() || is_correcting;
			}
			
			// store correct data
			last_pvc_[i].p = last_last_pvc_[i].p = controller_->motionPool().at(i).targetPos();
			last_pvc_[i].v = last_last_pvc_[i].v = controller_->motionPool().at(i).targetVel();
			last_pvc_[i].c = last_last_pvc_[i].c = controller_->motionPool().at(i).targetToq();
		}
		return error_code;
	}
	auto ControlServer::instance()->ControlServer & { static ControlServer instance; return instance; }
	auto ControlServer::loadXml(const aris::core::XmlElement &xml_ele)->void
	{
		Object::loadXml(xml_ele);
		imp_->controller_ = findOrInsertType<aris::control::Controller>();
		imp_->model_ = findOrInsertType<aris::dynamic::Model>();
		imp_->sensor_root_ = findOrInsertType<aris::sensor::SensorRoot>();
		imp_->plan_root_ = findOrInsertType<aris::plan::PlanRoot>();
		imp_->interface_root_ = findOrInsertType<aris::server::InterfaceRoot>();
		imp_->interface_pool_ = findOrInsertType<aris::core::ObjectPool<aris::server::Interface>>();
	}
	auto ControlServer::resetModel(dynamic::Model *model)->void
	{
		auto iter = std::find_if(children().begin(), children().end(), [&](const aris::core::Object &obj) 
		{ 
			auto found = dynamic_cast<const aris::dynamic::Model*>(&obj);
			return found && imp_->model_ == found;
		});
		if (iter != children().end())children().erase(iter);
		add(model);
		imp_->model_ = model;
	}
	auto ControlServer::resetController(control::Controller *controller)->void
	{
		auto iter = std::find_if(children().begin(), children().end(), [&](const aris::core::Object &obj) 
		{ 
			auto found = dynamic_cast<const aris::control::Controller*>(&obj);
			return found && imp_->controller_ == found;
		});
		if (iter != children().end())children().erase(iter);
		add(controller);
		imp_->controller_ = controller;
	}
	auto ControlServer::resetSensorRoot(sensor::SensorRoot *sensor_root)->void
	{
		auto iter = std::find_if(children().begin(), children().end(), [&](const aris::core::Object &obj) 
		{ 
			auto found = dynamic_cast<const aris::sensor::SensorRoot*>(&obj);
			return found && imp_->sensor_root_ == found;
		});
		if (iter != children().end())children().erase(iter);
		add(sensor_root);
		imp_->sensor_root_ = sensor_root;
	}
	auto ControlServer::resetPlanRoot(plan::PlanRoot *plan_root)->void
	{
		auto iter = std::find_if(children().begin(), children().end(), [&](const aris::core::Object &obj) 
		{
			auto found = dynamic_cast<const aris::plan::PlanRoot*>(&obj);
			return found && imp_->plan_root_ == found;
		});
		if (iter != children().end())children().erase(iter);
		add(plan_root);
		imp_->plan_root_ = plan_root;
	}
	auto ControlServer::model()->dynamic::Model& { return *imp_->model_; }
	auto ControlServer::controller()->control::Controller& { return *imp_->controller_; }
	auto ControlServer::sensorRoot()->sensor::SensorRoot& { return *imp_->sensor_root_; }
	auto ControlServer::planRoot()->plan::PlanRoot& { return *imp_->plan_root_; }
	auto ControlServer::interfacePool()->aris::core::ObjectPool<aris::server::Interface>& { return *imp_->interface_pool_; }
	auto ControlServer::interfaceRoot()->InterfaceRoot& { return *imp_->interface_root_; }
	auto ControlServer::setRtPlanPreCallback(PreCallback pre_callback)->void { imp_->pre_callback_.store(pre_callback); }
	auto ControlServer::setRtPlanPostCallback(PostCallback post_callback)->void { imp_->post_callback_.store(post_callback); }
	auto ControlServer::running()->bool { return imp_->is_running_; }
	auto ControlServer::globalCount()->std::int64_t { return imp_->global_count_.load(); }
	auto ControlServer::open()->void { for (auto &inter : interfacePool()) inter.open(); }
	auto ControlServer::close()->void { for (auto &inter : interfacePool()) inter.close(); }
	auto ControlServer::runCmdLine()->void
	{
		auto ret = std::async(std::launch::async, []()->std::string
		{
			std::string command_in;
			std::getline(std::cin, command_in);
			return command_in;
		});

		for (;;)
		{
			// 检测是否有数据从executeCmdInMain过来
			if (imp_->cmdline_msg_received_)
			{
				try
				{
					auto target = executeCmd(imp_->cmdline_msg_, imp_->cmdline_post_callback_);
					imp_->cmdline_msg_received_ = false;
					imp_->cmdline_execute_promise_->set_value(target);
				}
				catch (...)
				{
					imp_->cmdline_msg_received_ = false;
					imp_->cmdline_execute_promise_->set_exception(std::current_exception());
				}
			}
			// 检测是否有数据从command line过来
			else if (ret.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready)
			{
				try
				{
					executeCmd(aris::core::Msg(ret.get()), [](aris::plan::Plan &plan)->void
					{
						if (auto str = std::any_cast<std::string>(&plan.ret()))
						{
							std::cout << *str << std::endl;
						}
						else if (auto js = std::any_cast<std::vector<std::pair<std::string, std::any>>>(&plan.ret()))
						{
							js->push_back(std::make_pair<std::string, std::any>("return_code", plan.retCode()));
							js->push_back(std::make_pair<std::string, std::any>("return_message", std::string(plan.retMsg())));
							std::cout << parse_ret_value(*js) << std::endl;
						}
					});
				}
				catch (std::exception &e)
				{
					std::vector<std::pair<std::string, std::any>> ret_pair;
					ret_pair.push_back(std::make_pair<std::string, std::any>("return_code", int(aris::plan::Plan::PARSE_EXCEPTION)));
					ret_pair.push_back(std::make_pair<std::string, std::any>("return_message", std::string(e.what())));
					std::string ret_str = parse_ret_value(ret_pair);
					
					std::cout << ret_str << std::endl;
					LOG_ERROR << ret_str << std::endl;
				}

				ret = std::async(std::launch::async, []()->std::string
				{
					std::string command_in;
					std::getline(std::cin, command_in);
					return command_in;
				});
			}
			// 休息
			else
			{
				std::this_thread::sleep_for(std::chrono::milliseconds(1));
			}
		}
	}
	auto ControlServer::executeCmd(const aris::core::Msg &msg, std::function<void(aris::plan::Plan&)> post_callback)->std::shared_ptr<aris::plan::Plan>
	{
		std::unique_lock<std::recursive_mutex> running_lck(imp_->mu_running_);

		static std::uint64_t cmd_id{ 0 };
		++cmd_id;

		LOG_INFO << "server execute cmd " << std::to_string(cmd_id) << " : " << msg.toString() << std::endl;
		auto cmd_end = imp_->cmd_end_.load();

		// 找到命令对应的plan //
		std::string cmd;
		std::map<std::string, std::string> params;
		planRoot().planParser().parse(msg.toString(), cmd, params);
		auto plan_iter = std::find_if(planRoot().planPool().begin(), planRoot().planPool().end(), [&](const plan::Plan &p) {return p.command().name() == cmd; });

		// 初始化plan //
		auto internal_data = std::make_shared<Imp::InternalData>(Imp::InternalData{
			std::shared_ptr<aris::plan::Plan>(dynamic_cast<aris::plan::Plan*>(plan_iter->getTypeInfo(plan_iter->type())->copy_construct_func(*plan_iter))),
			post_callback
			});
		auto plan = internal_data->plan_;
		plan->imp_->count_ = 0;
		plan->imp_->model_ = imp_->model_;
		plan->imp_->master_ = imp_->controller_;
		plan->imp_->controller_ = dynamic_cast<aris::control::Controller*>(plan->imp_->master_);
		plan->imp_->ec_master_ = dynamic_cast<aris::control::EthercatMaster*>(plan->imp_->master_);
		plan->imp_->ec_controller_ = dynamic_cast<aris::control::EthercatController*>(plan->imp_->master_);
		plan->imp_->cs_ = this;
		plan->imp_->option_ = 0;
		plan->imp_->mot_options_.resize(plan->imp_->controller_->motionPool().size(), 0);
		plan->imp_->cmd_msg_ = msg.toString();
		plan->imp_->cmd_name_ = std::move(cmd);
		plan->imp_->cmd_params_ = std::move(params);
		plan->imp_->begin_global_count_ = 0;
		plan->imp_->command_id_ = cmd_id;
		plan->imp_->rt_stastic_ = aris::control::Master::RtStasticsData{ 0,0,0,0x8fffffff,0,0,0 };

		// prepair //
		LOG_INFO << "server prepair cmd " << std::to_string(cmd_id) << std::endl;
		plan->prepairNrt(plan->cmdParams());

		// print and log cmd info /////////////////////////////////////////////////////////////////////////////////////////////////////////////
		auto print_size = plan->cmdParams().empty() ? 2 : 2 + std::max_element(plan->cmdParams().begin(), plan->cmdParams().end(), [](const auto& a, const auto& b)
		{
			return a.first.length() < b.first.length();
		})->first.length();
		// print
		if (!(plan->option() & aris::plan::Plan::NOT_PRINT_CMD_INFO))
		{
			std::cout << plan->cmdName() << std::endl;
			for (auto &p : plan->cmdParams())std::cout << std::string(print_size - p.first.length(), ' ') << p.first << " : " << p.second << std::endl;
			std::cout << std::endl;
		}
		// log
		if (!(plan->option() & aris::plan::Plan::NOT_LOG_CMD_INFO))
		{
			auto &log = LOG_INFO << plan->cmdName() << std::endl;
			for (auto &p : plan->cmdParams())
			{
				log << std::setw(aris::core::LOG_SPACE_WIDTH) << '|' << std::string(print_size - p.first.length(), ' ') << p.first << " : " << p.second << std::endl;
			}
		}
		// print over ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

		// execute //
		if (!(plan->option() & aris::plan::Plan::NOT_RUN_EXECUTE_FUNCTION))
		{
			// 只有实时循环才需要 server 已经在运行
			if (!imp_->is_running_)LOG_AND_THROW(std::runtime_error("failed to execute command, because ControlServer is not running"));
			
			// 等待所有任务完成 //
			if (plan->option() & aris::plan::Plan::EXECUTE_WHEN_ALL_PLAN_EXECUTED) waitForAllExecution();

			// 等待所有任务收集 //
			if (plan->option() & aris::plan::Plan::EXECUTE_WHEN_ALL_PLAN_COLLECTED) waitForAllCollection();

			// 判断是否等待命令池清空 //
			if ((!(plan->option() & aris::plan::Plan::WAIT_IF_CMD_POOL_IS_FULL)) && (cmd_end - imp_->cmd_collect_.load()) >= Imp::CMD_POOL_SIZE)//原子操作(cmd_now)
				LOG_AND_THROW(std::runtime_error("failed to execute plan, because command pool is full"));
			else
				while ((cmd_end - imp_->cmd_collect_.load()) >= Imp::CMD_POOL_SIZE)std::this_thread::sleep_for(std::chrono::milliseconds(1));

			// 添加命令 //
			LOG_INFO << "server execute rt cmd " << std::to_string(cmd_id) << std::endl;
			imp_->internal_data_queue_[cmd_end % Imp::CMD_POOL_SIZE] = internal_data;
			imp_->cmd_end_.store(++cmd_end); // 原子操作 //

			// 等待当前任务完成 //
			if(plan->option() & aris::plan::Plan::WAIT_FOR_EXECUTION)waitForAllExecution();
		}

		// collect //
		if (!(plan->option() & aris::plan::Plan::NOT_RUN_COLLECT_FUNCTION))
		{
			// 没有实时规划的轨迹，直接同步收集 //
			if (plan->option() & aris::plan::Plan::NOT_RUN_EXECUTE_FUNCTION)
			{
				// 等待所有任务完成，原子操作 //
				while ((plan->option() & aris::plan::Plan::COLLECT_WHEN_ALL_PLAN_EXECUTED) && (cmd_end != imp_->cmd_now_.load()))std::this_thread::sleep_for(std::chrono::milliseconds(1));

				// 等待所有任务收集，原子操作 //
				while ((plan->option() & aris::plan::Plan::COLLECT_WHEN_ALL_PLAN_COLLECTED) && (cmd_end != imp_->cmd_collect_.load()))std::this_thread::sleep_for(std::chrono::milliseconds(1));

				LOG_INFO << "server collect cmd " << plan->commandId() << std::endl;
				plan->collectNrt();
				plan->retCode() = aris::plan::Plan::SUCCESS;
			}
			// 等待当前实时任务收集 //
			else
			{
				// 等待当前任务收集 //
				if (plan->option() & aris::plan::Plan::WAIT_FOR_COLLECTION)waitForAllCollection();
			}
		}
		else
		{
			if (plan->option() & aris::plan::Plan::NOT_RUN_EXECUTE_FUNCTION)
			{
				plan->retCode() = aris::plan::Plan::SUCCESS;
			}
		}

		return plan;
	}
	auto ControlServer::executeCmdInCmdLine(const aris::core::Msg &cmd_string, std::function<void(aris::plan::Plan&)> post_callback)->std::shared_ptr<aris::plan::Plan>
	{
		static std::mutex mu_;
		std::unique_lock<std::mutex> lck(mu_);

		imp_->cmdline_execute_promise_ = std::make_shared<std::promise<std::shared_ptr<aris::plan::Plan>>>();
		auto ret = imp_->cmdline_execute_promise_->get_future();
		imp_->cmdline_msg_ = aris::core::Msg(cmd_string);
		imp_->cmdline_post_callback_ = post_callback;
		imp_->cmdline_msg_received_ = true;

		return ret.get();
	}
	auto ControlServer::start()->void
	{
		std::unique_lock<std::recursive_mutex> running_lck(imp_->mu_running_);
		if (imp_->is_running_)LOG_AND_THROW(std::runtime_error("failed to start server, because it is already started "));
		imp_->is_running_ = true;

		// 分配所需要的内存 //
		Size mem_size = 0;

		core::allocMem(mem_size, imp_->last_pvc_, controller().slavePool().size());
		core::allocMem(mem_size, imp_->last_last_pvc_, controller().slavePool().size());
		core::allocMem(mem_size, imp_->global_mot_options_, controller().slavePool().size());

		imp_->mempool_.resize(mem_size, char(0));

		imp_->last_pvc_ = core::getMem(imp_->mempool_.data(), imp_->last_pvc_);
		imp_->last_last_pvc_ = core::getMem(imp_->mempool_.data(), imp_->last_last_pvc_);
		imp_->global_mot_options_ = core::getMem(imp_->mempool_.data(), imp_->global_mot_options_);
		std::fill_n(imp_->global_mot_options_, controller().slavePool().size(), aris::plan::Plan::NOT_CHECK_ENABLE | aris::plan::Plan::NOT_CHECK_POS_MAX | aris::plan::Plan::NOT_CHECK_POS_MIN);

		// 赋予初值 //
		controller().setControlStrategy([this]() {this->imp_->tg(); }); // controller可能被reset，因此这里必须重新设置//

		imp_->cmd_now_.store(0);
		imp_->cmd_end_.store(0);
		imp_->cmd_collect_.store(0);

		// start collect thread //
		imp_->is_collect_running_ = true;
		imp_->collect_thread_ = std::thread([this]()
		{
			while (this->imp_->is_collect_running_)
			{
				auto cmd_collect = imp_->cmd_collect_.load();//原子操作
				auto cmd_now = imp_->cmd_now_.load();//原子操作

				if (cmd_collect < cmd_now)
				{
					auto internal_data = std::move(imp_->internal_data_queue_[cmd_collect % Imp::CMD_POOL_SIZE]);
					auto &plan = *internal_data->plan_;

					while (globalCount() < plan.beginGlobalCount() + plan.count()) { std::this_thread::sleep_for(std::chrono::milliseconds(1)); }
					LOG_INFO << "cmd " << plan.commandId() << " stastics:" << std::endl
						<< std::setw(aris::core::LOG_SPACE_WIDTH) << '|' << std::setw(20) << "avg time(ns):" << std::int64_t(plan.rtStastic().avg_time_consumed) << std::endl
						<< std::setw(aris::core::LOG_SPACE_WIDTH) << '|' << std::setw(20) << "max time(ns):" << plan.rtStastic().max_time_consumed << std::endl
						<< std::setw(aris::core::LOG_SPACE_WIDTH) << '|' << std::setw(20) << "in count:" << plan.rtStastic().max_time_occur_count << std::endl
						<< std::setw(aris::core::LOG_SPACE_WIDTH) << '|' << std::setw(20) << "min time(ns):" << plan.rtStastic().min_time_consumed << std::endl
						<< std::setw(aris::core::LOG_SPACE_WIDTH) << '|' << std::setw(20) << "in count:" << plan.rtStastic().min_time_occur_count << std::endl
						<< std::setw(aris::core::LOG_SPACE_WIDTH) << '|' << std::setw(20) << "total count:" << plan.rtStastic().total_count << std::endl
						<< std::setw(aris::core::LOG_SPACE_WIDTH) << '|' << std::setw(20) << "overruns:" << plan.rtStastic().overrun_count << std::endl;

					if (!(plan.option() & aris::plan::Plan::NOT_RUN_COLLECT_FUNCTION)) 
					{
						LOG_INFO << "server collect cmd " << plan.commandId() << std::endl;
						plan.collectNrt();
					}
					internal_data.reset();
					imp_->cmd_collect_.store(cmd_collect + 1);
				}
				else
				{
					std::this_thread::sleep_for(std::chrono::milliseconds(1));
				}
			}
		});

		sensorRoot().start();
		controller().start();
	}
	auto ControlServer::stop()->void
	{
		std::unique_lock<std::recursive_mutex> running_lck(imp_->mu_running_);
		if (!imp_->is_running_)LOG_AND_THROW(std::runtime_error("failed to stop server, because it is not running"));
		imp_->is_running_ = false;

		// 清除所有指令，并回收所有指令 //
		imp_->cmd_now_.store(imp_->cmd_end_.load());
		while (imp_->cmd_collect_.load() < imp_->cmd_end_.load()) { std::this_thread::yield(); }
		imp_->is_collect_running_ = false;
		imp_->collect_thread_.join();

		// 停止控制器 //
		controller().stop();
		sensorRoot().stop();
	}
	auto ControlServer::waitForAllExecution()->void 
	{
		std::unique_lock<std::recursive_mutex> running_lck(imp_->mu_running_);
		auto cmd_end = imp_->cmd_end_.load();//原子操作
		while (cmd_end != imp_->cmd_now_.load())std::this_thread::sleep_for(std::chrono::milliseconds(1));//原子操作
	}
	auto ControlServer::waitForAllCollection()->void 
	{
		std::unique_lock<std::recursive_mutex> running_lck(imp_->mu_running_);
		auto cmd_end = imp_->cmd_end_.load();//原子操作
		while (cmd_end != imp_->cmd_collect_.load()) std::this_thread::sleep_for(std::chrono::milliseconds(1));//原子操作
	}
	auto ControlServer::getRtData(const std::function<void(ControlServer&, const aris::plan::Plan *target, std::any&)>& get_func, std::any& data)->void
	{
		std::unique_lock<std::recursive_mutex> running_lck(imp_->mu_running_);
		if (!imp_->is_running_)LOG_AND_THROW(std::runtime_error(std::string("failed") + __FILE__ + std::to_string(__LINE__)));

		imp_->get_data_func_ = &get_func;
		imp_->get_data_ = &data;

		imp_->if_get_data_ready_.store(false);
		imp_->if_get_data_.store(true);

		while (!imp_->if_get_data_ready_.load()) std::this_thread::sleep_for(std::chrono::milliseconds(1));

		imp_->if_get_data_ready_.store(false);
	}
	ControlServer::~ControlServer() 
	{ 
		close();
		if(running())stop();
	}
	ControlServer::ControlServer() :imp_(new Imp(this))
	{
		// create members //
		makeModel<aris::dynamic::Model>("model");
		makeController<aris::control::Controller>("controller");
		makeSensorRoot<aris::sensor::SensorRoot>("sensor_root");
		makePlanRoot<aris::plan::PlanRoot>("plan_root");
		
		// interface pool //
		this->registerType<aris::core::ObjectPool<Interface> >();
		imp_->interface_pool_ = &this->add<aris::core::ObjectPool<Interface>>();

		// create ui //
		auto ins = new InterfaceRoot;
		children().push_back_ptr(ins);
		imp_->interface_root_ = ins;
		this->interfaceRoot().loadXmlStr("<InterfaceRoot/>");
	}
}