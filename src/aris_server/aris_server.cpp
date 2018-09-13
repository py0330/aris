#include <cstring>
#include <thread>
#include <algorithm>
#include <memory>
#include <cinttypes>

#include "aris_core.h"
#include "aris_control.h"
#include "aris_server.h"

namespace aris::server
{
	struct ControlServer::Imp
	{
		auto tg()->void;
		auto executeCmd(std::pair<aris::plan::Plan *, aris::plan::PlanTarget> &plan_and_target)->int;
		auto onRunError()->int;

		Imp(ControlServer *server) :server_(server) {}
		Imp(const Imp&) = delete;

		std::recursive_mutex mu_running_;
		std::atomic_bool is_running_{ false };

		ControlServer *server_;

		// 实时循环中的轨迹参数 //
		enum { CMD_POOL_SIZE = 50 };
		std::pair<aris::plan::Plan *, aris::plan::PlanTarget> plan_and_target_queue_[CMD_POOL_SIZE];

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

		// hack 储存model中所有杆件的位姿参数 //
		std::vector<double> part_pm_vec_;
		std::atomic_bool if_need_part_pm_{ false }, if_part_pm_ready_{ false };
	};
	auto ControlServer::Imp::tg()->void
	{
		auto cmd_now = cmd_now_.load();//原子操作
		auto cmd_end = cmd_end_.load();//原子操作

		// 执行cmd queue中的cmd //
		if (cmd_end > cmd_now)
		{
			// 创建log文件
			if (count_ == 1)
			{
				char name[1000];
				std::sprintf(name, "%" PRId64 "", plan_and_target_queue_[cmd_now % CMD_POOL_SIZE].second.command_id);
				server_->controller().logFile(name);
			}

			// 执行命令
			auto ret = executeCmd(plan_and_target_queue_[cmd_now % CMD_POOL_SIZE]);

			// 命令正常运行，打印信息
			if (ret > 0)
			{
				if (++count_ % 1000 == 0) server_->controller().mout() << "execute cmd in count: " << count_;
			}
			// 命令正常结束
			else if (ret == 0)
			{
				server_->controller().mout() << "cmd finished, spend " << count_ << " counts\n\n";
				count_ = 1;
				cmd_now_.store(cmd_now + 1);//原子操作
			}
			// 命令出现错误
			else
			{
				server_->controller().mout() << "All commands in command queue are discarded, please try to RECOVER\n";
				count_ = 1;
				cmd_now_.store(cmd_end);//原子操作
			}
		}

		// 把杆件信息更新到外部 //
		if (if_need_part_pm_.load())// 原子操作
		{
			for (Size i(-1); ++i < model_->partPool().size();)model_->partPool().at(i).getPm(part_pm_vec_.data() + i * 16);

			if_part_pm_ready_.store(true); // 原子操作
			if_need_part_pm_.store(false); // 原子操作
		}
	}
	auto ControlServer::Imp::executeCmd(std::pair<aris::plan::Plan *, aris::plan::PlanTarget> &plan_and_target)->int
	{
		plan_and_target.second.count = count_;

		// 执行plan函数 //
		int ret = reinterpret_cast<aris::plan::Plan *>(plan_and_target.first)->executeRT(plan_and_target.second);

		// 控制电机 //
		for (std::size_t i = 0; i < controller_->motionPool().size(); ++i)
		{
			auto &cm = controller_->motionPool().at(i);
			auto &mm = model_->motionPool().at(i);

			if (mm.active())
			{
				if ((plan_and_target.second.option & aris::plan::Plan::USE_TARGET_POS))cm.setTargetPos(mm.mp());
				if ((plan_and_target.second.option & aris::plan::Plan::USE_TARGET_VEL))cm.setTargetVel(mm.mv());
				if ((plan_and_target.second.option & aris::plan::Plan::USE_TARGET_CUR))cm.setTargetCur(mm.mf());
				if ((plan_and_target.second.option & aris::plan::Plan::USE_VEL_OFFSET))cm.setOffsetVel(mm.mv());
				if ((plan_and_target.second.option & aris::plan::Plan::USE_CUR_OFFSET))cm.setOffsetCur(mm.mf());
			}
		}

		// 检查规划的指令是否合理（包括电机是否已经跟随上） //
		for (std::size_t i = 0; i < controller_->motionPool().size(); ++i)
		{
			auto &cm = controller_->motionPool().at(i);
			auto &ld = last_pvc.at(i);
			auto &lld = last_last_pvc.at(i);

			// check pos max //
			if (!(plan_and_target.second.option & aris::plan::Plan::NOT_CHECK_POS_MAX) && (cm.targetPos() > cm.maxPos()))
			{
				server_->controller().mout() << __FILE__ << __LINE__ << "\n";
				server_->controller().mout() << "Motor " << cm.id() << " (sla id) target position is bigger than its MAX permitted value in count: " << count_ << "\n";
				server_->controller().mout() << "The min, max and current count using ABS sequence are:\n";
				for (auto &cm1 : controller_->motionPool())server_->controller().mout() << cm1.minPos() << "\t" << cm1.maxPos() << "\t" << cm1.targetPos() << "\n";
				onRunError();
				ret = -1;
				break;
			}

			// check pos min //
			if (!(plan_and_target.second.option & aris::plan::Plan::NOT_CHECK_POS_MIN) && (cm.targetPos() < cm.minPos()))
			{
				server_->controller().mout() << __FILE__ << __LINE__ << "\n";
				server_->controller().mout() << "Motor " << cm.id() << " (sla id) target position is smaller than its MIN permitted value in count: " << count_ << "\n";
				server_->controller().mout() << "The min, max and current count using ABS sequence are:\n";
				for (auto &cm1 : controller_->motionPool())server_->controller().mout() << cm1.minPos() << "\t" << cm1.maxPos() << "\t" << cm1.targetPos() << "\n";
				onRunError();
				ret = -1;
				break;
			}

			// check pos continuous //
			if (!(plan_and_target.second.option & aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS) && count_ > 1 && ((cm.targetPos() - ld.p) > 0.001 * cm.maxVel() || (cm.targetPos() - ld.p) < 0.001 * cm.minVel()))
			{
				server_->controller().mout() << __FILE__ << __LINE__ << "\n";
				server_->controller().mout() << "Motor " << cm.id() << " (sla id) target position is not continuous in count: " << count_ << "\n";
				server_->controller().mout() << "The pin of last and this count using ABS sequence are:\n";
				for (std::size_t i = 0; i < controller_->motionPool().size(); ++i)server_->controller().mout() << last_pvc.at(i).p << "\t" << controller_->motionPool().at(i).targetPos() << "\n";
				onRunError();
				ret = -1;
				break;
			}

			// check pos continuous at start //
			if (!(plan_and_target.second.option & aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_AT_START) && count_ <= 1 && ((cm.targetPos() - ld.p) > 0.001 * cm.maxVel() || (cm.targetPos() - ld.p) < 0.001 * cm.minVel()))
			{
				server_->controller().mout() << __FILE__ << __LINE__ << "\n";
				server_->controller().mout() << "Motor " << cm.id() << " (sla id) target position is not continuous in count: " << count_ << "\n";
				server_->controller().mout() << "The pin of last and this count using ABS sequence are:\n";
				for (std::size_t i = 0; i < controller_->motionPool().size(); ++i)server_->controller().mout() << last_pvc.at(i).p << "\t" << controller_->motionPool().at(i).targetPos() << "\n";
				onRunError();
				ret = -1;
				break;
			}

			// check pos continuous second order //
			if (!(plan_and_target.second.option & aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER)
				&& count_ > 2
				&& ((cm.targetPos() + lld.p - 2 * ld.p) > 1e-6 * cm.maxAcc() || (cm.targetPos() + lld.p - 2 * ld.p) < 1e-6 * cm.minAcc()))
			{
				server_->controller().mout() << __FILE__ << __LINE__ << "\n";
				server_->controller().mout() << "Motor " << cm.id() << " (sla id) target position is not continuous in second order in count: " << count_ << "\n";
				server_->controller().mout() << "The pin of last and this count using ABS sequence are:\n";
				for (std::size_t i = 0; i < controller_->motionPool().size(); ++i)server_->controller().mout() << last_pvc.at(i).p << "\t" << controller_->motionPool().at(i).targetPos() << "\n";
				onRunError();
				ret = -1;
				break;
			}

			// check pos continuous second order at start //
			if (!(plan_and_target.second.option & aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER_AT_START)
				&& count_ <= 2
				&& ((cm.targetPos() + lld.p - 2 * ld.p) > 1e-6 * cm.maxAcc() || (cm.targetPos() + lld.p - 2 * ld.p) < 1e-6 * cm.minAcc()))
			{
				server_->controller().mout() << __FILE__ << __LINE__ << "\n";
				server_->controller().mout() << "Motor " << cm.id() << " (sla id) target position is not continuous in second order in count: " << count_ << "\n";
				server_->controller().mout() << "The pin of last and this count using ABS sequence are:\n";
				for (std::size_t i = 0; i < controller_->motionPool().size(); ++i)server_->controller().mout() << last_pvc.at(i).p << "\t" << controller_->motionPool().at(i).targetPos() << "\n";
				onRunError();
				ret = -1;
				break;
			}

			// check pos following error //
			if (!(plan_and_target.second.option & aris::plan::Plan::NOT_CHECK_POS_FOLLOWING_ERROR) && (std::abs(cm.targetPos() - cm.actualPos()) > cm.maxPosFollowingError()))
			{
				server_->controller().mout() << __FILE__ << __LINE__ << "\n";
				server_->controller().mout() << "Motor " << cm.id() << " (sla id) target and feedback positions are not near in count: " << count_ << "\n";
				server_->controller().mout() << "The pin of target and feedback using ABS sequence are:\n";
				for (auto &cmp : controller_->motionPool())server_->controller().mout() << cmp.targetPos() << "\t" << cmp.actualPos() << "\n";
				onRunError();
				ret = -1;
				break;
			}

			// check vel max //
			if (!(plan_and_target.second.option & aris::plan::Plan::NOT_CHECK_VEL_MAX) && (cm.targetVel() > cm.maxVel()))
			{
				server_->controller().mout() << __FILE__ << __LINE__ << "\n";
				server_->controller().mout() << "Motor " << cm.id() << " (sla id) target velocity is bigger than its MAX permitted value in count: " << count_ << "\n";
				server_->controller().mout() << "The min, max and current count using ABS sequence are:\n";
				for (auto &cm1 : controller_->motionPool())server_->controller().mout() << cm1.minVel() << "\t" << cm1.maxVel() << "\t" << cm1.targetVel() << "\n";
				onRunError();
				ret = -1;
				break;
			}

			// check vel min //
			if (!(plan_and_target.second.option & aris::plan::Plan::NOT_CHECK_VEL_MIN) && (cm.targetVel() < cm.minVel()))
			{
				server_->controller().mout() << __FILE__ << __LINE__ << "\n";
				server_->controller().mout() << "Motor " << cm.id() << " (sla id) target veolcity is smaller than its MIN permitted value in count: " << count_ << "\n";
				server_->controller().mout() << "The min, max and current count using ABS sequence are:\n";
				for (auto &cm1 : controller_->motionPool())server_->controller().mout() << cm1.minVel() << "\t" << cm1.maxVel() << "\t" << cm1.targetVel() << "\n";
				onRunError();
				ret = -1;
				break;
			}

			// check vel continuous //
			if (!(plan_and_target.second.option & aris::plan::Plan::NOT_CHECK_VEL_CONTINUOUS) && count_ > 1 && ((cm.targetVel() - ld.v) > 0.001 * cm.maxAcc() || (cm.targetVel() - ld.v) < 0.001 * cm.minAcc()))
			{
				server_->controller().mout() << __FILE__ << __LINE__ << "\n";
				server_->controller().mout() << "Motor " << cm.id() << " (sla id) target velocity is not continuous in count: " << count_ << "\n";
				server_->controller().mout() << "The pin of last and this count using ABS sequence are:\n";
				for (std::size_t i = 0; i < controller_->motionPool().size(); ++i)server_->controller().mout() << last_pvc.at(i).v << "\t" << controller_->motionPool().at(i).targetVel() << "\n";
				onRunError();
				ret = -1;
				break;
			}

			// check vel continuous at start //
			if (!(plan_and_target.second.option & aris::plan::Plan::NOT_CHECK_VEL_CONTINUOUS_AT_START) && count_ <= 1 && ((cm.targetVel() - ld.v) > 0.001 * cm.maxAcc() || (cm.targetVel() - ld.v) < 0.001 * cm.minAcc()))
			{
				server_->controller().mout() << __FILE__ << __LINE__ << "\n";
				server_->controller().mout() << "Motor " << cm.id() << " (sla id) target velocity is not continuous in count: " << count_ << "\n";
				server_->controller().mout() << "The pin of last and this count using ABS sequence are:\n";
				for (std::size_t i = 0; i < controller_->motionPool().size(); ++i)server_->controller().mout() << last_pvc.at(i).v << "\t" << controller_->motionPool().at(i).targetVel() << "\n";
				onRunError();
				ret = -1;
				break;
			}

			// check vel following error //
			if (!(plan_and_target.second.option & aris::plan::Plan::NOT_CHECK_VEL_FOLLOWING_ERROR) && (std::abs(cm.targetVel() - cm.actualVel()) > cm.maxVelFollowingError()))
			{
				server_->controller().mout() << __FILE__ << __LINE__ << "\n";
				server_->controller().mout() << "Motor " << cm.id() << " (sla id) target and feedback velocities are not near in count: " << count_ << "\n";
				server_->controller().mout() << "The pin of target and feedback using ABS sequence are:\n";
				for (auto &cmp : controller_->motionPool())server_->controller().mout() << cmp.targetVel() << "\t" << cmp.actualVel() << "\n";
				onRunError();
				ret = -1;
				break;
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
		return 0;
	}
	auto ControlServer::instance()->ControlServer & { static ControlServer instance; return instance; }
	auto ControlServer::resetModel(dynamic::Model *model)->void
	{
		auto iter = std::find_if(children().begin(), children().end(), [](const aris::core::Object &obj) { return obj.name() == "model"; });
		if (iter != children().end())children().erase(iter);
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
	auto ControlServer::resetPlanRoot(plan::PlanRoot *plan_root)->void
	{
		auto iter = std::find_if(children().begin(), children().end(), [](const aris::core::Object &obj) { return obj.name() == "plan_root"; });
		if (iter != children().end())children().erase(iter);
		children().push_back_ptr(plan_root);
		imp_->plan_root_ = plan_root;
	}
	auto ControlServer::model()->dynamic::Model& { return *imp_->model_; }
	auto ControlServer::controller()->control::Controller& { return *imp_->controller_; }
	auto ControlServer::sensorRoot()->sensor::SensorRoot& { return *imp_->sensor_root_; }
	auto ControlServer::planRoot()->plan::PlanRoot& { return *imp_->plan_root_; }
	auto ControlServer::loadXml(const aris::core::XmlElement &xml_ele)->void
	{
		Object::loadXml(xml_ele);
		imp_->controller_ = findOrInsert<aris::control::Controller>("controller");
		imp_->model_ = findOrInsert<aris::dynamic::Model>("model");
		imp_->sensor_root_ = findOrInsert<aris::sensor::SensorRoot>("sensor_root");
		imp_->plan_root_ = findOrInsert<aris::plan::PlanRoot>("plan_root");
	}
	auto ControlServer::executeCmd(const aris::core::Msg &msg)->std::int64_t
	{
		std::unique_lock<std::recursive_mutex> running_lck(imp_->mu_running_);
		if (!imp_->is_running_)LOG_AND_THROW(std::runtime_error("failed to execute command, because ControlServer is not running"));

		static std::int64_t cmd_id{ 0 };
		++cmd_id;

		LOG_INFO << "server receive cmd " << std::to_string(cmd_id) << " : " << msg.toString() << std::endl;
		aris::plan::PlanTarget target{ &model(), &controller(), cmd_id, msg.header().reserved1_, 0, std::any() };
		auto cmd_end = imp_->cmd_end_.load();

		// 找到命令对应的plan //
		std::string cmd;
		std::map<std::string, std::string> params;
		planRoot().planParser().parse(msg.toString(), cmd, params);
		auto plan_iter = std::find_if(planRoot().planPool().begin(), planRoot().planPool().end(), [&](const plan::Plan &p) {return p.command().name() == cmd; });

		// print cmd and params /////////////////////////////////////////////////////////////////////////////////////////////////////////////
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
		// print over ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

		// prepair //
		if (!(target.option & aris::plan::Plan::NOT_RUN_PREPAIR_FUNCTION))
		{
			// 等待所有任务完成 //
			while ((target.option & aris::plan::Plan::PREPAIR_WHEN_ALL_PLAN_EXECUTED) && (cmd_end != imp_->cmd_now_.load()))std::this_thread::sleep_for(std::chrono::milliseconds(1));//原子操作

			// 等待所有任务收集 //
			while ((target.option & aris::plan::Plan::PREPAIR_WHEN_ALL_PLAN_COLLECTED) && (cmd_end != imp_->cmd_collect_.load()))std::this_thread::sleep_for(std::chrono::milliseconds(1));//原子操作

			LOG_INFO << "server prepair cmd " << std::to_string(cmd_id) << std::endl;
			plan_iter->prepairNrt(params, target);
		}

		// execute //
		if (!(target.option & aris::plan::Plan::NOT_RUN_EXECUTE_FUNCTION))
		{
			// 等待所有任务完成 //
			while ((target.option & aris::plan::Plan::EXECUTE_WHEN_ALL_PLAN_EXECUTED) && (cmd_end != imp_->cmd_now_.load()))std::this_thread::sleep_for(std::chrono::milliseconds(1));//原子操作

			// 等待所有任务收集 //
			while ((target.option & aris::plan::Plan::EXECUTE_WHEN_ALL_PLAN_COLLECTED) && (cmd_end != imp_->cmd_collect_.load()))std::this_thread::sleep_for(std::chrono::milliseconds(1));//原子操作

			// 判断是否等待命令池清空 //
			if ((!(target.option & aris::plan::Plan::WAIT_IF_CMD_POOL_IS_FULL)) && (cmd_end - imp_->cmd_collect_.load()) >= Imp::CMD_POOL_SIZE)//原子操作(cmd_now)
				LOG_AND_THROW(std::runtime_error("failed to execute plan, because command pool is full"));
			else
				while ((cmd_end - imp_->cmd_collect_.load()) >= Imp::CMD_POOL_SIZE)std::this_thread::yield();

			// 添加命令 //
			LOG_INFO << "server execute cmd " << std::to_string(cmd_id) << std::endl;
			imp_->plan_and_target_queue_[cmd_end % Imp::CMD_POOL_SIZE] = std::pair<aris::plan::Plan*, aris::plan::PlanTarget>{ &*plan_iter, std::move(target) };
			imp_->cmd_end_.store(++cmd_end);//原子操作

			// 等待当前任务完成 //
			while ((target.option & aris::plan::Plan::WAIT_FOR_EXECUTION) && (cmd_end != imp_->cmd_now_.load()))std::this_thread::sleep_for(std::chrono::milliseconds(1));//原子操作
		}

		// collect //
		if (!(target.option & aris::plan::Plan::NOT_RUN_COLLECT_FUNCTION))
		{
			// 没有实时规划的轨迹，直接同步收集 //
			if (target.option & aris::plan::Plan::NOT_RUN_EXECUTE_FUNCTION)
			{
				// 等待所有任务完成，原子操作 //
				while ((target.option & aris::plan::Plan::COLLECT_WHEN_ALL_PLAN_EXECUTED) && (cmd_end != imp_->cmd_now_.load()))std::this_thread::sleep_for(std::chrono::milliseconds(1));

				// 等待所有任务收集，原子操作 //
				while ((target.option & aris::plan::Plan::COLLECT_WHEN_ALL_PLAN_COLLECTED) && (cmd_end != imp_->cmd_collect_.load()))std::this_thread::sleep_for(std::chrono::milliseconds(1));

				plan_iter->collectNrt(target);
			}
			// 等待当前实时任务收集 //
			else
			{
				// 等待当前任务收集 //
				while ((target.option & aris::plan::Plan::WAIT_FOR_COLLECTION) && (cmd_end != imp_->cmd_collect_.load())) std::this_thread::sleep_for(std::chrono::milliseconds(1));//原子操作
			}
		}

		return cmd_id;
	}
	auto ControlServer::currentExecuteId()->std::int64_t
	{
		std::unique_lock<std::recursive_mutex> running_lck(imp_->mu_running_);
		if (!imp_->is_running_)LOG_AND_THROW(std::runtime_error("failed to get current execute ID, because ControlServer is not running"));

		// 只有execute_cmd函数才可能会改变cmd_queue中的数据
		auto cmd_end = imp_->cmd_end_.load();
		auto cmd_now = imp_->cmd_now_.load();

		return cmd_now<cmd_end ? imp_->plan_and_target_queue_[cmd_now % Imp::CMD_POOL_SIZE].second.command_id : 0;
	}
	auto ControlServer::currentCollectId()->std::int64_t
	{
		std::unique_lock<std::recursive_mutex> running_lck(imp_->mu_running_);
		if (!imp_->is_running_)LOG_AND_THROW(std::runtime_error("failed to get current collect ID, because ControlServer is not running"));

		// 只有execute_cmd函数才可能会改变cmd_queue中的数据
		auto cmd_end = imp_->cmd_end_.load();
		auto cmd_collect = imp_->cmd_collect_.load();

		return cmd_collect<cmd_end ? imp_->plan_and_target_queue_[cmd_collect % Imp::CMD_POOL_SIZE].second.command_id : 0;
	}
	auto ControlServer::start()->void
	{
		std::unique_lock<std::recursive_mutex> running_lck(imp_->mu_running_);
		if (imp_->is_running_)LOG_AND_THROW(std::runtime_error("failed to start server, because it is already started "));
		imp_->is_running_ = true;

		// 分配model的内存 //
		for (auto &solver : imp_->model_->solverPool())solver.allocateMemory();

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
					auto &plan_and_target = imp_->plan_and_target_queue_[cmd_collect % Imp::CMD_POOL_SIZE];
					LOG_INFO << "server collect cmd " << plan_and_target.second.command_id << std::endl;
					if (!(plan_and_target.second.option & aris::plan::Plan::NOT_RUN_COLLECT_FUNCTION))plan_and_target.first->collectNrt(plan_and_target.second);
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

		controller().stop();
		sensorRoot().stop();

		// stop collect thread //
		imp_->cmd_now_.store(imp_->cmd_end_.load());
		while (imp_->cmd_collect_.load() < imp_->cmd_end_.load())std::this_thread::yield();
		imp_->is_collect_running_ = false;
		imp_->collect_thread_.join();
	}
	auto ControlServer::getPartPm()->std::vector<double>
	{
		std::unique_lock<std::recursive_mutex> running_lck(imp_->mu_running_);

		if (imp_->is_running_)
		{
			imp_->part_pm_vec_.resize(imp_->model_->partPool().size() * 16);

			imp_->if_part_pm_ready_.store(false);
			imp_->if_need_part_pm_.store(true);

			while (!imp_->if_part_pm_ready_.load()) std::this_thread::sleep_for(std::chrono::milliseconds(1));

			imp_->if_part_pm_ready_.store(false);
		}
		else
		{
			for (Size i(-1); ++i < imp_->model_->partPool().size();)imp_->model_->partPool().at(i).getPm(imp_->part_pm_vec_.data() + i * 16);
		}



		return imp_->part_pm_vec_;
	}
	ControlServer::~ControlServer() = default;
	ControlServer::ControlServer() :imp_(new Imp(this))
	{
		registerType<aris::dynamic::Model>();
		registerType<aris::control::Controller>();
		registerType<aris::sensor::SensorRoot>();
		registerType<aris::plan::PlanRoot>();
		registerType<aris::control::EthercatController>();

		// create instance //
		makeModel<aris::dynamic::Model>("model");
		makeController<aris::control::Controller>("controller");
		makeSensorRoot<aris::sensor::SensorRoot>("sensor_root");
		makePlanRoot<aris::plan::PlanRoot>("plan_root");
	}
}
