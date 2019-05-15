#include <cstring>
#include <thread>
#include <algorithm>
#include <memory>
#include <cinttypes>
#include <queue>

#include <aris/core/core.hpp>
#include <aris/control/control.hpp>

#include "aris/server/server.hpp"

namespace aris::server
{
	const std::uint64_t global_option = 
		aris::plan::Plan::NOT_CHECK_OPERATION_ENABLE | 
		aris::plan::Plan::NOT_CHECK_POS_MAX | 
		aris::plan::Plan::NOT_CHECK_POS_MIN;
	
	struct ControlServer::Imp
	{
		struct InternalData
		{
			std::shared_ptr<aris::plan::PlanTarget> target;
			std::promise<void> ret_promise;
		};
		
		auto tg()->void;
		auto executeCmd(aris::plan::PlanTarget &target)->int;
		auto checkMotion(std::uint64_t option)->int;

		Imp(ControlServer *server) :server_(server) {}
		Imp(const Imp&) = delete;

		std::recursive_mutex mu_running_;
		std::atomic_bool is_running_{ false };

		ControlServer *server_;

		// 实时循环中的轨迹参数 //
		enum { CMD_POOL_SIZE = 1000 };
		std::shared_ptr<InternalData> internal_data_queue_[CMD_POOL_SIZE];
		
		// 全局count //
		std::atomic<std::int64_t> global_count_{ 0 };

		// cmd系列参数
		std::atomic<std::int64_t> cmd_now_, cmd_end_, cmd_collect_;
		std::uint32_t count_{ 1 };

		// collect系列参数
		std::thread collect_thread_;
		std::atomic_bool is_collect_running_;

		// 储存上一次motion的数据 //
		struct PVC { double p; double v; double c; };
		std::vector<PVC> last_pvc, last_last_pvc;

		// 储存Model, Controller, SensorRoot, PlanRoot //
		aris::dynamic::Model* model_;
		aris::control::Controller* controller_;
		aris::sensor::SensorRoot* sensor_root_;
		aris::plan::PlanRoot* plan_root_;
		InterfaceRoot *interface_root_;

		// 打洞，读取数据 //
		std::atomic_bool if_get_data_{ false }, if_get_data_ready_{ false };
		const std::function<void(ControlServer&, std::any&)>* get_data_func_;
		std::any *get_data_;
	};
	auto ControlServer::Imp::tg()->void
	{
		auto global_count = ++global_count_; // 原子操作

		auto cmd_now = cmd_now_.load();//原子操作
		auto cmd_end = cmd_end_.load();//原子操作

		// global error code, 存储上一次的错误 //
		static int idle_error_code{ 0 };

		// 执行cmd queue中的cmd //
		if (cmd_end > cmd_now)
		{
			auto &target = *internal_data_queue_[cmd_now % CMD_POOL_SIZE]->target;
			
			// 在第一回合初始化，包括log，初始化target等 //
			if (count_ == 1)
			{
				// 初始化target
				target.begin_global_count = global_count;
				
				// 创建rt_log文件 //
				char name[1000];
				std::sprintf(name, "%" PRId64 "", target.command_id);
				server_->controller().logFile(name);

				// 初始化统计数据 //
				server_->controller().resetRtStasticData(&target.rt_stastic, true);
			}

			// 执行命令
			auto ret = executeCmd(target);

			// 检查错误 //
			if (ret < 0 || checkMotion(target.option))
			{
				target.ret_code = aris::plan::PlanTarget::ERROR;
				
				server_->controller().mout() << "failed, cmd queue cleared\n";
				count_ = 1;
				cmd_now_.store(cmd_end);//原子操作

				server_->controller().resetRtStasticData(nullptr, false);
			}
			// 命令正常结束，结束统计数据 //
			else if (ret == 0)
			{
				server_->controller().lout() << std::flush;
				
				target.ret_code = aris::plan::PlanTarget::SUCCESS;
				
				if(!(target.option & aris::plan::Plan::NOT_PRINT_EXECUTE_COUNT))
					server_->controller().mout() << "cmd finished, spend " << count_ << " counts\n\n";
				count_ = 1;
				cmd_now_.store(cmd_now + 1);//原子操作

				server_->controller().resetRtStasticData(nullptr, false);
			}
			// 命令仍在执行 //
			else
			{
				if (++count_ % 1000 == 0 && !(target.option & aris::plan::Plan::NOT_PRINT_EXECUTE_COUNT))
					server_->controller().mout() << "execute cmd in count: " << count_ << "\n";
			}
		}
		else if (auto error_code = idle_error_code; idle_error_code = checkMotion(global_option) && idle_error_code != error_code)
		{
			// 只有错误代码改变时，才会打印 //
			server_->controller().mout() << "failed when idle " << idle_error_code << "\n";
		}

		// 给与外部想要的数据 //
		if (if_get_data_.exchange(false))// 原子操作
		{
			get_data_func_->operator()(ControlServer::instance(), *get_data_);
			if_get_data_ready_.store(true); // 原子操作
		}
	}
	auto ControlServer::Imp::executeCmd(aris::plan::PlanTarget &target)->int
	{
		target.count = count_;

		// 执行plan函数 //
		int ret = target.plan->executeRT(target);

		// 控制电机 //
		for (std::size_t i = 0; i < std::min(controller_->motionPool().size(), model_->motionPool().size()); ++i)
		{
			auto &cm = controller_->motionPool().at(i);
			auto &mm = model_->motionPool().at(i);

			if (mm.active())
			{
				if ((target.option & aris::plan::Plan::USE_TARGET_POS))cm.setTargetPos(mm.mp());
				if ((target.option & aris::plan::Plan::USE_TARGET_VEL))cm.setTargetVel(mm.mv());
				if ((target.option & aris::plan::Plan::USE_TARGET_CUR))cm.setTargetCur(mm.mf());
				if ((target.option & aris::plan::Plan::USE_VEL_OFFSET))cm.setOffsetVel(mm.mv());
				if ((target.option & aris::plan::Plan::USE_CUR_OFFSET))cm.setOffsetCur(mm.mf());
			}
		}

		return ret;
	}
	auto ControlServer::Imp::checkMotion(std::uint64_t option)->int
	{
		static bool is_correcting{ false };
		if (is_correcting)goto FAILED;

		// 检查规划的指令是否合理（包括电机是否已经跟随上） //
		for (std::size_t i = 0; i < controller_->motionPool().size(); ++i)
		{
			auto &cm = controller_->motionPool().at(i);
			auto &ld = last_pvc.at(i);
			auto &lld = last_last_pvc.at(i);

#ifndef WIN32
			/*
			if (!(option & aris::plan::Plan::NOT_CHECK_OPERATION_ENABLE)
				&& ((cm.statusWord() & 0x6f) != 0x27))
			{
				server_->controller().mout() << __FILE__ << __LINE__ << "\n";
				server_->controller().mout() << "Motor " << i << " not in operation enable mode in count " << count_ << ":\n";
				server_->controller().mout() << "max: " << cm.maxPos() << "\t" << "now: " << cm.targetPos() << "\n";
				goto FAILED;
			}
			*/
#endif

			if ((cm.statusWord() & 0x6f) == 0x27)
			{
				if (cm.modeOfOperation() == 8)
				{
					// check pos max //
					if (!(option & aris::plan::Plan::NOT_CHECK_POS_MAX)
						&& (cm.targetPos() > cm.maxPos()))
					{
						server_->controller().mout() << __FILE__ << __LINE__ << "\n";
						server_->controller().mout() << "Motor " << i << " target position beyond MAX in count " << count_ << ":\n";
						server_->controller().mout() << "max: " << cm.maxPos() << "\t" << "now: " << cm.targetPos() << "\n";
						goto FAILED;
					}

					// check pos min //
					if (!(option & aris::plan::Plan::NOT_CHECK_POS_MIN)
						&& (cm.targetPos() < cm.minPos()))
					{
						server_->controller().mout() << __FILE__ << __LINE__ << "\n";
						server_->controller().mout() << "Motor " << i << " target position beyond MIN in count " << count_ << ":\n";
						server_->controller().mout() << "min: " << cm.minPos() << "\t" << "now: " << cm.targetPos() << "\n";
						goto FAILED;
					}

					// check pos continuous //
					if (!(option & aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS)
						&& count_ > 1
						&& ((cm.targetPos() - ld.p) > 0.001 * cm.maxVel() || (cm.targetPos() - ld.p) < 0.001 * cm.minVel()))
					{
						server_->controller().mout() << __FILE__ << __LINE__ << "\n";
						server_->controller().mout() << "Motor " << i << " target position NOT CONTINUOUS in count " << count_ << "\n";
						server_->controller().mout() << "last: " << last_pvc.at(i).p << "\t" << "now: " << cm.targetPos() << "\n";
						goto FAILED;
					}

					// check pos continuous at start //
					if (!(option & aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_AT_START)
						&& count_ <= 1
						&& ((cm.targetPos() - ld.p) > 0.001 * cm.maxVel() || (cm.targetPos() - ld.p) < 0.001 * cm.minVel()))
					{
						server_->controller().mout() << __FILE__ << __LINE__ << "\n";
						server_->controller().mout() << "Motor " << i << " target position NOT CONTINUOUS in count " << count_ << "\n";
						server_->controller().mout() << "last: " << last_pvc.at(i).p << "\t" << "now: " << cm.targetPos() << "\n";
						goto FAILED;
					}

					// check pos continuous second order //
					if (!(option & aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER)
						&& count_ > 2
						&& ((cm.targetPos() + lld.p - 2 * ld.p) > 1e-6 * cm.maxAcc() || (cm.targetPos() + lld.p - 2 * ld.p) < 1e-6 * cm.minAcc()))
					{
						server_->controller().mout() << __FILE__ << __LINE__ << "\n";
						server_->controller().mout() << "Motor " << i << " target position NOT SECOND CONTINUOUS in count " << count_ << "\n";
						server_->controller().mout() << "last last: " << lld.p << "\tlast:" << ld.p << "\t" << "now: " << cm.targetPos() << "\n";
						goto FAILED;
					}

					// check pos continuous second order at start //
					if (!(option & aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER_AT_START)
						&& count_ <= 2
						&& ((cm.targetPos() + lld.p - 2 * ld.p) > 1e-6 * cm.maxAcc() || (cm.targetPos() + lld.p - 2 * ld.p) < 1e-6 * cm.minAcc()))
					{
						server_->controller().mout() << __FILE__ << __LINE__ << "\n";
						server_->controller().mout() << "Motor " << i << " target position NOT SECOND CONTINUOUS in count " << count_ << "\n";
						server_->controller().mout() << "last last: " << lld.p << "\tlast:" << ld.p << "\t" << "now: " << cm.targetPos() << "\n";
						goto FAILED;
					}

					// check pos following error //
					if (!(option & aris::plan::Plan::NOT_CHECK_POS_FOLLOWING_ERROR)
						&& (std::abs(cm.targetPos() - cm.actualPos()) > cm.maxPosFollowingError()))
					{
						server_->controller().mout() << __FILE__ << __LINE__ << "\n";
						server_->controller().mout() << "Motor " << i << " target position has FOLLOW ERROR: " << count_ << "\n";
						server_->controller().mout() << "target: " << cm.targetPos() << "\t" << "actual: " << cm.actualPos() << "\n";
						goto FAILED;
					}
				}
				else if (cm.modeOfDisplay() == 9)
				{
					// check vel max //
					if (!(option & aris::plan::Plan::NOT_CHECK_VEL_MAX)
						&& (cm.targetVel() > cm.maxVel()))
					{
						server_->controller().mout() << __FILE__ << __LINE__ << "\n";
						server_->controller().mout() << "Motor " << i << " target velocity beyond MAX in count " << count_ << ":\n";
						server_->controller().mout() << "max: " << cm.maxVel() << "\t" << "now: " << cm.targetVel() << "\n";
						goto FAILED;
					}

					// check vel min //
					if (!(option & aris::plan::Plan::NOT_CHECK_VEL_MIN)
						&& (cm.targetVel() < cm.minVel()))
					{
						server_->controller().mout() << __FILE__ << __LINE__ << "\n";
						server_->controller().mout() << "Motor " << i << " target veolcity beyond MIN in count " << count_ << ":\n";
						server_->controller().mout() << "min: " << cm.minVel() << "\t" << "now: " << cm.targetVel() << "\n";
						goto FAILED;
					}

					// check vel continuous //
					if (!(option & aris::plan::Plan::NOT_CHECK_VEL_CONTINUOUS)
						&& count_ > 1
						&& ((cm.targetVel() - ld.v) > 0.001 * cm.maxAcc() || (cm.targetVel() - ld.v) < 0.001 * cm.minAcc()))
					{
						server_->controller().mout() << __FILE__ << __LINE__ << "\n";
						server_->controller().mout() << "Motor " << i << " target velocity NOT CONTINUOUS in count " << count_ << "\n";
						server_->controller().mout() << "last: " << last_pvc.at(i).v << "\t" << "now: " << controller_->motionPool().at(i).targetVel() << "\n";
						goto FAILED;
					}

					// check vel continuous at start //
					if (!(option & aris::plan::Plan::NOT_CHECK_VEL_CONTINUOUS_AT_START)
						&& count_ <= 1
						&& ((cm.targetVel() - ld.v) > 0.001 * cm.maxAcc() || (cm.targetVel() - ld.v) < 0.001 * cm.minAcc()))
					{
						server_->controller().mout() << __FILE__ << __LINE__ << "\n";
						server_->controller().mout() << "Motor " << i << " target velocity NOT CONTINUOUS in count " << count_ << "\n";
						server_->controller().mout() << "last: " << last_pvc.at(i).v << "\t" << "now: " << controller_->motionPool().at(i).targetVel() << "\n";
						goto FAILED;
					}

					// check vel following error //
					if (!(option & aris::plan::Plan::NOT_CHECK_VEL_FOLLOWING_ERROR)
						&& (std::abs(cm.targetVel() - cm.actualVel()) > cm.maxVelFollowingError()))
					{
						server_->controller().mout() << __FILE__ << __LINE__ << "\n";
						server_->controller().mout() << "Motor " << i << " target velocity has FOLLOW ERROR: " << count_ << "\n";
						server_->controller().mout() << "target: " << cm.targetVel() << "\t" << "actual: " << cm.actualVel() << "\n";
						goto FAILED;
					}
				}
				else if (cm.modeOfDisplay() == 10)
				{
					// check pos max //
					if (!(option & aris::plan::Plan::NOT_CHECK_POS_MAX)
						&& (cm.actualPos() > cm.maxPos()))
					{
						server_->controller().mout() << __FILE__ << __LINE__ << "\n";
						server_->controller().mout() << "Motor " << i << " target position beyond MAX in count " << count_ << ":\n";
						server_->controller().mout() << "max: " << cm.maxPos() << "\t" << "now: " << cm.targetPos() << "\n";
						goto FAILED;
					}

					// check pos min //
					if (!(option & aris::plan::Plan::NOT_CHECK_POS_MIN)
						&& (cm.actualPos() < cm.minPos()))
					{
						server_->controller().mout() << __FILE__ << __LINE__ << "\n";
						server_->controller().mout() << "Motor " << i << " target position beyond MIN in count " << count_ << ":\n";
						server_->controller().mout() << "min: " << cm.minPos() << "\t" << "now: " << cm.targetPos() << "\n";
						goto FAILED;
					}

					// check vel max //
					if (!(option & aris::plan::Plan::NOT_CHECK_VEL_MAX)
						&& (cm.actualVel() > cm.maxVel()))
					{
						server_->controller().mout() << __FILE__ << __LINE__ << "\n";
						server_->controller().mout() << "Motor " << i << " target velocity beyond MAX in count " << count_ << ":\n";
						server_->controller().mout() << "max: " << cm.maxVel() << "\t" << "now: " << cm.targetVel() << "\n";
						goto FAILED;
					}

					// check vel min //
					if (!(option & aris::plan::Plan::NOT_CHECK_VEL_MIN)
						&& (cm.actualVel() < cm.minVel()))
					{
						server_->controller().mout() << __FILE__ << __LINE__ << "\n";
						server_->controller().mout() << "Motor " << i << " target veolcity beyond MIN in count " << count_ << ":\n";
						server_->controller().mout() << "min: " << cm.minVel() << "\t" << "now: " << cm.targetVel() << "\n";
						goto FAILED;
					}

					// check vel continuous //
					if (!(option & aris::plan::Plan::NOT_CHECK_VEL_CONTINUOUS)
						&& count_ > 1
						&& ((cm.actualVel() - ld.v) > 0.001 * cm.maxAcc() || (cm.actualVel() - ld.v) < 0.001 * cm.minAcc()))
					{
						server_->controller().mout() << __FILE__ << __LINE__ << "\n";
						server_->controller().mout() << "Motor " << i << " target velocity NOT CONTINUOUS in count " << count_ << "\n";
						server_->controller().mout() << "last: " << last_pvc.at(i).v << "\t" << "now: " << controller_->motionPool().at(i).targetVel() << "\n";
						goto FAILED;
					}

					// check vel continuous at start //
					if (!(option & aris::plan::Plan::NOT_CHECK_VEL_CONTINUOUS_AT_START)
						&& count_ <= 1
						&& ((cm.actualVel() - ld.v) > 0.001 * cm.maxAcc() || (cm.actualVel() - ld.v) < 0.001 * cm.minAcc()))
					{
						server_->controller().mout() << __FILE__ << __LINE__ << "\n";
						server_->controller().mout() << "Motor " << i << " target velocity NOT CONTINUOUS in count " << count_ << "\n";
						server_->controller().mout() << "last: " << last_pvc.at(i).v << "\t" << "now: " << controller_->motionPool().at(i).targetVel() << "\n";
						goto FAILED;
					}
				}
			}
			
		}

		// 储存电机指令 //
		for (std::size_t i = 0; i < controller_->motionPool().size(); ++i)
		{
			last_last_pvc.at(i).p = controller_->motionPool().at(i).targetPos();
			last_last_pvc.at(i).v = controller_->motionPool().at(i).targetVel();
			last_last_pvc.at(i).c = controller_->motionPool().at(i).targetCur();
		}
		std::swap(last_pvc, last_last_pvc);
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
				cm.setTargetCur(0.0);
				is_correcting = cm.disable();
				break;
			default:
				is_correcting = cm.disable();
			}
			
			// store correct data
			last_pvc.at(i).p = last_last_pvc.at(i).p = controller_->motionPool().at(i).targetPos();
			last_pvc.at(i).v = last_last_pvc.at(i).v = controller_->motionPool().at(i).targetVel();
			last_pvc.at(i).c = last_last_pvc.at(i).c = controller_->motionPool().at(i).targetCur();
		}
		return -1;
	}
	auto ControlServer::instance()->ControlServer & { static ControlServer instance; return instance; }
	auto ControlServer::resetModel(dynamic::Model *model)->void
	{
		auto iter = std::find_if(children().begin(), children().end(), [&](const aris::core::Object &obj) 
		{ 
			return imp_->model_ == dynamic_cast<const aris::dynamic::Model*>(&obj);
		});
		if (iter != children().end())children().erase(iter);
		add(model);
		imp_->model_ = model;
	}
	auto ControlServer::resetController(control::Controller *controller)->void
	{
		auto iter = std::find_if(children().begin(), children().end(), [&](const aris::core::Object &obj) 
		{ 
			return imp_->controller_ == dynamic_cast<const aris::control::Controller*>(&obj);
		});
		if (iter != children().end())children().erase(iter);
		add(controller);
		imp_->controller_ = controller;
	}
	auto ControlServer::resetSensorRoot(sensor::SensorRoot *sensor_root)->void
	{
		auto iter = std::find_if(children().begin(), children().end(), [&](const aris::core::Object &obj) 
		{ 
			return imp_->sensor_root_ == dynamic_cast<const aris::sensor::SensorRoot*>(&obj);
		});
		if (iter != children().end())children().erase(iter);
		add(sensor_root);
		imp_->sensor_root_ = sensor_root;
	}
	auto ControlServer::resetPlanRoot(plan::PlanRoot *plan_root)->void
	{
		auto iter = std::find_if(children().begin(), children().end(), [&](const aris::core::Object &obj) 
		{
			return imp_->plan_root_ == dynamic_cast<const aris::plan::PlanRoot*>(&obj);
		});
		if (iter != children().end())children().erase(iter);
		add(plan_root);
		imp_->plan_root_ = plan_root;
	}
	auto ControlServer::model()->dynamic::Model& { return *imp_->model_; }
	auto ControlServer::controller()->control::Controller& { return *imp_->controller_; }
	auto ControlServer::sensorRoot()->sensor::SensorRoot& { return *imp_->sensor_root_; }
	auto ControlServer::planRoot()->plan::PlanRoot& { return *imp_->plan_root_; }
	auto ControlServer::interfaceRoot()->InterfaceRoot& { return *imp_->interface_root_; }
	auto ControlServer::loadXml(const aris::core::XmlElement &xml_ele)->void
	{
		Object::loadXml(xml_ele);
		imp_->controller_ = findOrInsertType<aris::control::Controller>();
		imp_->model_ = findOrInsertType<aris::dynamic::Model>();
		imp_->sensor_root_ = findOrInsertType<aris::sensor::SensorRoot>();
		imp_->plan_root_ = findOrInsertType<aris::plan::PlanRoot>();
		imp_->interface_root_ = findOrInsertType<aris::server::InterfaceRoot>();
	}
	auto ControlServer::executeCmd(const aris::core::Msg &msg)->std::shared_ptr<aris::plan::PlanTarget>
	{
		std::unique_lock<std::recursive_mutex> running_lck(imp_->mu_running_);

		static std::uint64_t cmd_id{ 0 };
		++cmd_id;

		LOG_INFO << "server receive cmd " << std::to_string(cmd_id) << " : " << msg.toString() << std::endl;
		auto cmd_end = imp_->cmd_end_.load();

		// 找到命令对应的plan //
		std::string cmd;
		std::map<std::string, std::string> params;
		planRoot().planParser().parse(msg.toString(), cmd, params);
		auto plan_iter = std::find_if(planRoot().planPool().begin(), planRoot().planPool().end(), [&](const plan::Plan &p) {return p.command().name() == cmd; });

		// 初始化plan target //
		auto internal_data = std::make_shared<Imp::InternalData>(Imp::InternalData{
			std::make_shared<aris::plan::PlanTarget>(
				aris::plan::PlanTarget
				{ 
					&*plan_iter, 
					this, 
					&model(), 
					&controller(), 
					cmd_id, 
					static_cast<std::uint64_t>(msg.header().reserved1_), 
					std::any(), 
					0, 
					0, 
					aris::control::Master::RtStasticsData{ 0,0,0,0x8fffffff,0,0,0 },
					std::any(),
					aris::plan::PlanTarget::CANCELLED,
					std::future<void>()
				}),
			std::promise<void>()
			});
		auto &target = internal_data->target;
		target->finished = internal_data->ret_promise.get_future();

		// prepair //
		LOG_INFO << "server prepair cmd " << std::to_string(cmd_id) << std::endl;
		plan_iter->prepairNrt(params, *target);

		// print and log cmd info /////////////////////////////////////////////////////////////////////////////////////////////////////////////
		auto print_size = params.empty() ? 2 : 2 + std::max_element(params.begin(), params.end(), [](const auto& a, const auto& b)
		{
			return a.first.length() < b.first.length();
		})->first.length();
		if (!(target->option & aris::plan::Plan::NOT_PRINT_CMD_INFO))std::cout << cmd << std::endl;

		auto &log = LOG_INFO << cmd << std::endl;
		for (auto &p : params)
		{
			if (!(target->option & aris::plan::Plan::NOT_PRINT_CMD_INFO))
				std::cout << std::string(print_size - p.first.length(), ' ') << p.first << " : " << p.second << std::endl;
			if (!(target->option & aris::plan::Plan::NOT_LOG_CMD_INFO))
				log << std::setw(aris::core::LOG_SPACE_WIDTH) << '|' << std::string(print_size - p.first.length(), ' ') << p.first << " : " << p.second << std::endl;
		}
		if (!(target->option & aris::plan::Plan::NOT_PRINT_CMD_INFO))std::cout << std::endl;
		// print over ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

		// execute //
		if (!(target->option & aris::plan::Plan::NOT_RUN_EXECUTE_FUNCTION))
		{
			// 只有实时循环才需要 server 已经在运行
			if (!imp_->is_running_)LOG_AND_THROW(std::runtime_error("failed to execute command, because ControlServer is not running"));
			
			// 等待所有任务完成 //
			if (target->option & aris::plan::Plan::EXECUTE_WHEN_ALL_PLAN_EXECUTED)waitForAllExecution();

			// 等待所有任务收集 //
			if (target->option & aris::plan::Plan::EXECUTE_WHEN_ALL_PLAN_COLLECTED)waitForAllCollection();

			// 判断是否等待命令池清空 //
			if ((!(target->option & aris::plan::Plan::WAIT_IF_CMD_POOL_IS_FULL)) && (cmd_end - imp_->cmd_collect_.load()) >= Imp::CMD_POOL_SIZE)//原子操作(cmd_now)
				LOG_AND_THROW(std::runtime_error("failed to execute plan, because command pool is full"));
			else
				while ((cmd_end - imp_->cmd_collect_.load()) >= Imp::CMD_POOL_SIZE)std::this_thread::sleep_for(std::chrono::milliseconds(1));

			// 添加命令 //
			LOG_INFO << "server execute cmd " << std::to_string(cmd_id) << std::endl;
			imp_->internal_data_queue_[cmd_end % Imp::CMD_POOL_SIZE] = internal_data;
			imp_->cmd_end_.store(++cmd_end); // 原子操作 //

			// 等待当前任务完成 //
			if(target->option & aris::plan::Plan::WAIT_FOR_EXECUTION)waitForAllExecution();
		}

		// collect //
		if (!(target->option & aris::plan::Plan::NOT_RUN_COLLECT_FUNCTION))
		{
			// 没有实时规划的轨迹，直接同步收集 //
			if (target->option & aris::plan::Plan::NOT_RUN_EXECUTE_FUNCTION)
			{
				// 等待所有任务完成，原子操作 //
				while ((target->option & aris::plan::Plan::COLLECT_WHEN_ALL_PLAN_EXECUTED) && (cmd_end != imp_->cmd_now_.load()))std::this_thread::sleep_for(std::chrono::milliseconds(1));

				// 等待所有任务收集，原子操作 //
				while ((target->option & aris::plan::Plan::COLLECT_WHEN_ALL_PLAN_COLLECTED) && (cmd_end != imp_->cmd_collect_.load()))std::this_thread::sleep_for(std::chrono::milliseconds(1));

				LOG_INFO << "server collect cmd " << target->command_id << std::endl;
				plan_iter->collectNrt(*target);
				internal_data->target->ret_code = aris::plan::PlanTarget::SUCCESS;
				internal_data->ret_promise.set_value();
			}
			// 等待当前实时任务收集 //
			else
			{
				// 等待当前任务收集 //
				if (target->option & aris::plan::Plan::WAIT_FOR_COLLECTION)waitForAllCollection();
			}
		}
		else
		{
			if (target->option & aris::plan::Plan::NOT_RUN_EXECUTE_FUNCTION)
			{
				internal_data->target->ret_code = aris::plan::PlanTarget::SUCCESS;
				internal_data->ret_promise.set_value();
			}
		}

		return target;
	}
	auto ControlServer::globalCount()->std::int64_t { return imp_->global_count_.load(); }
	auto ControlServer::currentExecuteId()->std::int64_t
	{
		std::unique_lock<std::recursive_mutex> running_lck(imp_->mu_running_);
		if (!imp_->is_running_)LOG_AND_THROW(std::runtime_error("failed to get current execute ID, because ControlServer is not running"));

		// 只有execute_cmd函数才可能会改变cmd_queue中的数据
		auto cmd_end = imp_->cmd_end_.load();
		auto cmd_now = imp_->cmd_now_.load();

		return cmd_now<cmd_end ? imp_->internal_data_queue_[cmd_now % Imp::CMD_POOL_SIZE]->target->command_id : 0;
	}
	auto ControlServer::currentCollectId()->std::int64_t
	{
		std::unique_lock<std::recursive_mutex> running_lck(imp_->mu_running_);
		if (!imp_->is_running_)LOG_AND_THROW(std::runtime_error("failed to get current collect ID, because ControlServer is not running"));

		// 只有execute_cmd函数才可能会改变cmd_queue中的数据
		auto cmd_end = imp_->cmd_end_.load();
		auto cmd_collect = imp_->cmd_collect_.load();

		return cmd_collect<cmd_end ? imp_->internal_data_queue_[cmd_collect % Imp::CMD_POOL_SIZE]->target->command_id : 0;
	}
	auto ControlServer::start()->void
	{
		std::unique_lock<std::recursive_mutex> running_lck(imp_->mu_running_);
		if (imp_->is_running_)LOG_AND_THROW(std::runtime_error("failed to start server, because it is already started "));
		imp_->is_running_ = true;

		// 得到电机向量以及数据 //
		imp_->last_pvc.clear();
		imp_->last_pvc.resize(controller().slavePool().size(), Imp::PVC{ 0,0,0 });
		imp_->last_last_pvc.clear();
		imp_->last_last_pvc.resize(controller().slavePool().size(), Imp::PVC{ 0,0,0 });

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
					auto internal_data = imp_->internal_data_queue_[cmd_collect % Imp::CMD_POOL_SIZE];
					auto &target = *internal_data->target;

					while (globalCount() < target.begin_global_count + target.count) { std::this_thread::sleep_for(std::chrono::milliseconds(1)); }
					LOG_INFO << "cmd " << target.command_id << " stastics:" << std::endl
						<< std::setw(aris::core::LOG_SPACE_WIDTH) << '|' << std::setw(20) << "avg time(ns):" << std::int64_t(target.rt_stastic.avg_time_consumed) << std::endl
						<< std::setw(aris::core::LOG_SPACE_WIDTH) << '|' << std::setw(20) << "max time(ns):" << target.rt_stastic.max_time_consumed << std::endl
						<< std::setw(aris::core::LOG_SPACE_WIDTH) << '|' << std::setw(20) << "in count:" << target.rt_stastic.max_time_occur_count << std::endl
						<< std::setw(aris::core::LOG_SPACE_WIDTH) << '|' << std::setw(20) << "min time(ns):" << target.rt_stastic.min_time_consumed << std::endl
						<< std::setw(aris::core::LOG_SPACE_WIDTH) << '|' << std::setw(20) << "in count:" << target.rt_stastic.min_time_occur_count << std::endl
						<< std::setw(aris::core::LOG_SPACE_WIDTH) << '|' << std::setw(20) << "total count:" << target.rt_stastic.total_count << std::endl
						<< std::setw(aris::core::LOG_SPACE_WIDTH) << '|' << std::setw(20) << "overruns:" << target.rt_stastic.overrun_count << std::endl;

					if (!(target.option & aris::plan::Plan::NOT_RUN_COLLECT_FUNCTION)) 
					{
						LOG_INFO << "server collect cmd " << target.command_id << std::endl;
						target.plan->collectNrt(target);
					}
					internal_data->ret_promise.set_value();
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
	auto ControlServer::running()->bool { return imp_->is_running_; }
	auto ControlServer::waitForAllExecution()->void 
	{
		auto cmd_end = imp_->cmd_end_.load();//原子操作
		while (cmd_end != imp_->cmd_now_.load())std::this_thread::sleep_for(std::chrono::milliseconds(1));//原子操作
	}
	auto ControlServer::waitForAllCollection()->void 
	{
		auto cmd_end = imp_->cmd_end_.load();//原子操作
		while (cmd_end != imp_->cmd_collect_.load()) std::this_thread::sleep_for(std::chrono::milliseconds(1));//原子操作
	}
	auto ControlServer::getRtData(const std::function<void(ControlServer&, std::any&)>& get_func, std::any& data)->void
	{
		std::unique_lock<std::recursive_mutex> running_lck(imp_->mu_running_);
		if (!imp_->is_running_)LOG_AND_THROW(std::runtime_error(std::string("failed") + __FILE__));

		imp_->get_data_func_ = &get_func;
		imp_->get_data_ = &data;

		imp_->if_get_data_ready_.store(false);
		imp_->if_get_data_.store(true);

		while (!imp_->if_get_data_ready_.load()) std::this_thread::sleep_for(std::chrono::milliseconds(1));

		imp_->if_get_data_ready_.store(false);
	}
	ControlServer::~ControlServer() = default;
	ControlServer::ControlServer() :imp_(new Imp(this))
	{
		// create instance //
		makeModel<aris::dynamic::Model>("model");
		makeController<aris::control::Controller>("controller");
		makeSensorRoot<aris::sensor::SensorRoot>("sensor_root");
		makePlanRoot<aris::plan::PlanRoot>("plan_root");
		
		auto ins = new InterfaceRoot;
		children().push_back_ptr(ins);
		imp_->interface_root_ = ins;
		this->interfaceRoot().loadXmlStr("<InterfaceRoot/>");
	}
}
