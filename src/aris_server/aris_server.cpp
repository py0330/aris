#include <cstring>
#include <thread>
#include <algorithm>
#include <memory>
#include <cinttypes>
#include <queue>

#include "aris_core.h"
#include "aris_control.h"
#include "aris_server.h"

namespace aris::server
{
	struct ControlServer::Imp
	{
		struct InternalData 
		{
			aris::plan::Plan * plan;
			aris::plan::PlanTarget target;
			std::atomic_bool is_stastic_ready;
		};

		auto tg()->void;
		auto executeCmd(InternalData &internal_data)->int;
		auto onRunError()->int;

		Imp(ControlServer *server) :server_(server) {}
		Imp(const Imp&) = delete;

		std::recursive_mutex mu_running_;
		std::atomic_bool is_running_{ false };

		ControlServer *server_;

		// 实时循环中的轨迹参数 //
		enum { CMD_POOL_SIZE = 1000 };
		InternalData internal_data_queue_[CMD_POOL_SIZE];
		
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

		// hack 储存model中所有杆件的位姿参数 //
		std::atomic_bool if_get_data_{ false }, if_get_data_ready_{ false };
		const std::function<void(ControlServer&, std::any&)>* get_data_func_;
		std::any *get_data_;
	};
	auto ControlServer::Imp::tg()->void
	{
		auto global_count = ++global_count_; // 原子操作

		auto cmd_now = cmd_now_.load();//原子操作
		auto cmd_end = cmd_end_.load();//原子操作

		// 执行cmd queue中的cmd //
		if (cmd_end > cmd_now)
		{
			// 在第一回合初始化，包括log，初始化target等 //
			if (count_ == 1)
			{
				// 初始化target
				internal_data_queue_[cmd_now % CMD_POOL_SIZE].target.begin_global_count = global_count;
				
				// 创建rt_log文件 //
				char name[1000];
				std::sprintf(name, "%" PRId64 "", internal_data_queue_[cmd_now % CMD_POOL_SIZE].target.command_id);
				server_->controller().logFile(name);

				// 初始化统计数据 //
				server_->controller().resetRtStasticData(&internal_data_queue_[cmd_now % CMD_POOL_SIZE].target.rt_stastic, true);
			}

			// 执行命令
			auto ret = executeCmd(internal_data_queue_[cmd_now % CMD_POOL_SIZE]);

			// 命令正常运行，打印信息
			if (ret > 0)
			{
				if (++count_ % 1000 == 0) server_->controller().mout() << "execute cmd in count: " << count_ <<"\n";
			}
			// 命令正常结束，结束统计数据
			else if (ret == 0)
			{
				server_->controller().mout() << "cmd finished, spend " << count_ << " counts\n\n";
				count_ = 1;
				cmd_now_.store(cmd_now + 1);//原子操作

				server_->controller().resetRtStasticData(nullptr, false);
			}
			// 命令出现错误，结束统计数据
			else
			{
				server_->controller().mout() << "cmd queue cleared\n";
				count_ = 1;
				cmd_now_.store(cmd_end);//原子操作

				server_->controller().resetRtStasticData(nullptr, false);
			}
		}

		// 给与外部想要的数据 //
		if (if_get_data_.exchange(false))// 原子操作
		{
			get_data_func_->operator()(ControlServer::instance(), *get_data_);
			if_get_data_ready_.store(true); // 原子操作
			if_get_data_.store(false); // 原子操作
		}
	}
	auto ControlServer::Imp::executeCmd(InternalData &internal_data)->int
	{
		internal_data.target.count = count_;

		// 执行plan函数 //
		int ret = reinterpret_cast<aris::plan::Plan *>(internal_data.plan)->executeRT(internal_data.target);

		// 控制电机 //
		for (std::size_t i = 0; i < std::min(controller_->motionPool().size(), model_->motionPool().size()); ++i)
		{
			auto &cm = controller_->motionPool().at(i);
			auto &mm = model_->motionPool().at(i);

			if (mm.active())
			{
				if ((internal_data.target.option & aris::plan::Plan::USE_TARGET_POS))cm.setTargetPos(mm.mp());
				if ((internal_data.target.option & aris::plan::Plan::USE_TARGET_VEL))cm.setTargetVel(mm.mv());
				if ((internal_data.target.option & aris::plan::Plan::USE_TARGET_CUR))cm.setTargetCur(mm.mf());
				if ((internal_data.target.option & aris::plan::Plan::USE_VEL_OFFSET))cm.setOffsetVel(mm.mv());
				if ((internal_data.target.option & aris::plan::Plan::USE_CUR_OFFSET))cm.setOffsetCur(mm.mf());
			}
		}

		// 检查规划的指令是否合理（包括电机是否已经跟随上） //
		for (std::size_t i = 0; i < controller_->motionPool().size(); ++i)
		{
			auto &cm = controller_->motionPool().at(i);
			auto &ld = last_pvc.at(i);
			auto &lld = last_last_pvc.at(i);

			if (cm.modeOfOperation() == 8)
			{
				// check pos max //
				if (!(internal_data.target.option & aris::plan::Plan::NOT_CHECK_POS_MAX) && (cm.targetPos() > cm.maxPos()))
				{
					server_->controller().mout() << __FILE__ << __LINE__ << "\n";
					server_->controller().mout() << "Motor " << i << " target position beyond MAX in count " << count_ << ":\n";
					server_->controller().mout() << "max: " << cm.maxPos() << "\t" << "now: " << cm.targetPos() << "\n";
					onRunError();
					ret = -1;
					break;
				}

				// check pos min //
				if (!(internal_data.target.option & aris::plan::Plan::NOT_CHECK_POS_MIN) && (cm.targetPos() < cm.minPos()))
				{
					server_->controller().mout() << __FILE__ << __LINE__ << "\n";
					server_->controller().mout() << "Motor " << i << " target position beyond MIN in count " << count_ << ":\n";
					server_->controller().mout() << "min: " << cm.minPos() << "\t" << "now: " << cm.targetPos() << "\n";
					onRunError();
					ret = -1;
					break;
				}

				// check pos continuous //
				if (!(internal_data.target.option & aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS) && count_ > 1 && ((cm.targetPos() - ld.p) > 0.001 * cm.maxVel() || (cm.targetPos() - ld.p) < 0.001 * cm.minVel()))
				{
					server_->controller().mout() << __FILE__ << __LINE__ << "\n";
					server_->controller().mout() << "Motor " << i << " target position NOT CONTINUOUS in count " << count_ << "\n";
					server_->controller().mout() << "last: " << last_pvc.at(i).p << "\t" << "now: " << cm.targetPos() << "\n";
					onRunError();
					ret = -1;
					break;
				}

				// check pos continuous at start //
				if (!(internal_data.target.option & aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_AT_START) && count_ <= 1 && ((cm.targetPos() - ld.p) > 0.001 * cm.maxVel() || (cm.targetPos() - ld.p) < 0.001 * cm.minVel()))
				{
					server_->controller().mout() << __FILE__ << __LINE__ << "\n";
					server_->controller().mout() << "Motor " << i << " target position NOT CONTINUOUS in count " << count_ << "\n";
					server_->controller().mout() << "last: " << last_pvc.at(i).p << "\t" << "now: " << cm.targetPos() << "\n";
					onRunError();
					ret = -1;
					break;
				}

				// check pos continuous second order //
				if (!(internal_data.target.option & aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER)
					&& count_ > 2
					&& ((cm.targetPos() + lld.p - 2 * ld.p) > 1e-6 * cm.maxAcc() || (cm.targetPos() + lld.p - 2 * ld.p) < 1e-6 * cm.minAcc()))
				{
					server_->controller().mout() << __FILE__ << __LINE__ << "\n";
					server_->controller().mout() << "Motor " << i << " target position NOT SECOND CONTINUOUS in count " << count_ << "\n";
					server_->controller().mout() << "last last: " << lld.p << "\tlast:" << ld.p << "\t" << "now: " << cm.targetPos() << "\n";					
					onRunError();
					ret = -1;
					break;
				}

				// check pos continuous second order at start //
				if (!(internal_data.target.option & aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER_AT_START)
					&& count_ <= 2
					&& ((cm.targetPos() + lld.p - 2 * ld.p) > 1e-6 * cm.maxAcc() || (cm.targetPos() + lld.p - 2 * ld.p) < 1e-6 * cm.minAcc()))
				{
					server_->controller().mout() << __FILE__ << __LINE__ << "\n";
					server_->controller().mout() << "Motor " << i << " target position NOT SECOND CONTINUOUS in count " << count_ << "\n";
					server_->controller().mout() << "last last: " << lld.p << "\tlast:" << ld.p << "\t" << "now: " << cm.targetPos() << "\n";
					onRunError();
					ret = -1;
					break;
				}

				// check pos following error //
				if (!(internal_data.target.option & aris::plan::Plan::NOT_CHECK_POS_FOLLOWING_ERROR) && (std::abs(cm.targetPos() - cm.actualPos()) > cm.maxPosFollowingError()))
				{
					server_->controller().mout() << __FILE__ << __LINE__ << "\n";
					server_->controller().mout() << "Motor " << i << " target position has FOLLOW ERROR: " << count_ << "\n";
					server_->controller().mout() << "target: " << cm.targetPos() << "\t" << "actual: " << cm.actualPos() << "\n";
					onRunError();
					ret = -1;
					break;
				}
			}
			if (cm.modeOfDisplay() == 9)
			{
				// check vel max //
				if (!(internal_data.target.option & aris::plan::Plan::NOT_CHECK_VEL_MAX) && (cm.targetVel() > cm.maxVel()))
				{
					server_->controller().mout() << __FILE__ << __LINE__ << "\n";
					server_->controller().mout() << "Motor " << i << " target velocity beyond MAX in count " << count_ << ":\n";
					server_->controller().mout() << "max: " << cm.maxVel() << "\t" << "now: " << cm.targetVel() << "\n";
					onRunError();
					ret = -1;
					break;
				}

				// check vel min //
				if (!(internal_data.target.option & aris::plan::Plan::NOT_CHECK_VEL_MIN) && (cm.targetVel() < cm.minVel()))
				{
					server_->controller().mout() << __FILE__ << __LINE__ << "\n";
					server_->controller().mout() << "Motor " << i << " target veolcity beyond MIN in count " << count_ << ":\n";
					server_->controller().mout() << "min: " << cm.minVel() << "\t" << "now: " << cm.targetVel() << "\n";
					onRunError();
					ret = -1;
					break;
				}

				// check vel continuous //
				if (!(internal_data.target.option & aris::plan::Plan::NOT_CHECK_VEL_CONTINUOUS) && count_ > 1 && ((cm.targetVel() - ld.v) > 0.001 * cm.maxAcc() || (cm.targetVel() - ld.v) < 0.001 * cm.minAcc()))
				{
					server_->controller().mout() << __FILE__ << __LINE__ << "\n";
					server_->controller().mout() << "Motor " << i << " target velocity NOT CONTINUOUS in count " << count_ << "\n";
					server_->controller().mout() << "last: " << last_pvc.at(i).v << "\t" << "now: " << controller_->motionPool().at(i).targetVel() << "\n";
					onRunError();
					ret = -1;
					break;
				}

				// check vel continuous at start //
				if (!(internal_data.target.option & aris::plan::Plan::NOT_CHECK_VEL_CONTINUOUS_AT_START) && count_ <= 1 && ((cm.targetVel() - ld.v) > 0.001 * cm.maxAcc() || (cm.targetVel() - ld.v) < 0.001 * cm.minAcc()))
				{
					server_->controller().mout() << __FILE__ << __LINE__ << "\n";
					server_->controller().mout() << "Motor " << i << " target velocity NOT CONTINUOUS in count " << count_ << "\n";
					server_->controller().mout() << "last: " << last_pvc.at(i).v << "\t" << "now: " << controller_->motionPool().at(i).targetVel() << "\n";
					onRunError();
					ret = -1;
					break;
				}

				// check vel following error //
				if (!(internal_data.target.option & aris::plan::Plan::NOT_CHECK_VEL_FOLLOWING_ERROR) && (std::abs(cm.targetVel() - cm.actualVel()) > cm.maxVelFollowingError()))
				{
					server_->controller().mout() << __FILE__ << __LINE__ << "\n";
					server_->controller().mout() << "Motor " << i << " target velocity has FOLLOW ERROR: " << count_ << "\n";
					server_->controller().mout() << "target: " << cm.targetVel() << "\t" << "actual: " << cm.actualVel() << "\n";
					onRunError();
					ret = -1;
					break;
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

		static std::uint64_t cmd_id{ 0 };
		++cmd_id;

		LOG_INFO << "server receive cmd " << std::to_string(cmd_id) << " : " << msg.toString() << std::endl;
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

		auto &log = LOG_INFO << cmd << std::endl;
		for (auto &p : params)
		{
			std::cout << std::string(print_size - p.first.length(), ' ') << p.first << " : " << p.second << std::endl;
			log << std::setw(aris::core::LOG_SPACE_WIDTH) << '|' << std::string(print_size - p.first.length(), ' ') << p.first << " : " << p.second << std::endl;
		}
		std::cout << std::endl;
		// print over ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

		aris::plan::PlanTarget target{ &model(), &controller(), cmd_id, static_cast<std::uint64_t>(msg.header().reserved1_), std::any(), 0, 0, aris::control::Master::RtStasticsData{ 0,0,0,0x8fffffff,0,0,0 } };
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
				while ((cmd_end - imp_->cmd_collect_.load()) >= Imp::CMD_POOL_SIZE)std::this_thread::sleep_for(std::chrono::milliseconds(1));

			// 添加命令 //
			LOG_INFO << "server execute cmd " << std::to_string(cmd_id) << std::endl;
			imp_->internal_data_queue_[cmd_end % Imp::CMD_POOL_SIZE].plan = &*plan_iter;
			imp_->internal_data_queue_[cmd_end % Imp::CMD_POOL_SIZE].target = std::move(target);
			imp_->internal_data_queue_[cmd_end % Imp::CMD_POOL_SIZE].is_stastic_ready.store(false);
			imp_->cmd_end_.store(++cmd_end); // 原子操作 //

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

				LOG_INFO << "server collect cmd " << target.command_id << std::endl;
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
	auto ControlServer::globalCount()->std::int64_t { return imp_->global_count_.load(); }
	auto ControlServer::currentExecuteId()->std::int64_t
	{
		std::unique_lock<std::recursive_mutex> running_lck(imp_->mu_running_);
		if (!imp_->is_running_)LOG_AND_THROW(std::runtime_error("failed to get current execute ID, because ControlServer is not running"));

		// 只有execute_cmd函数才可能会改变cmd_queue中的数据
		auto cmd_end = imp_->cmd_end_.load();
		auto cmd_now = imp_->cmd_now_.load();

		return cmd_now<cmd_end ? imp_->internal_data_queue_[cmd_now % Imp::CMD_POOL_SIZE].target.command_id : 0;
	}
	auto ControlServer::currentCollectId()->std::int64_t
	{
		std::unique_lock<std::recursive_mutex> running_lck(imp_->mu_running_);
		if (!imp_->is_running_)LOG_AND_THROW(std::runtime_error("failed to get current collect ID, because ControlServer is not running"));

		// 只有execute_cmd函数才可能会改变cmd_queue中的数据
		auto cmd_end = imp_->cmd_end_.load();
		auto cmd_collect = imp_->cmd_collect_.load();

		return cmd_collect<cmd_end ? imp_->internal_data_queue_[cmd_collect % Imp::CMD_POOL_SIZE].target.command_id : 0;
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
					auto &internal_data = imp_->internal_data_queue_[cmd_collect % Imp::CMD_POOL_SIZE];

					while (globalCount() < internal_data.target.begin_global_count + internal_data.target.count)std::this_thread::sleep_for(std::chrono::milliseconds(1));
					LOG_INFO << "cmd " << internal_data.target.command_id << " stastics:" << std::endl
						<< std::setw(aris::core::LOG_SPACE_WIDTH) << '|' << std::setw(20) << "avg time(ns):" << std::int64_t(internal_data.target.rt_stastic.avg_time_consumed) << std::endl
						<< std::setw(aris::core::LOG_SPACE_WIDTH) << '|' << std::setw(20) << "max time(ns):" << internal_data.target.rt_stastic.max_time_consumed << std::endl
						<< std::setw(aris::core::LOG_SPACE_WIDTH) << '|' << std::setw(20) << "in count:" << internal_data.target.rt_stastic.max_time_occur_count << std::endl
						<< std::setw(aris::core::LOG_SPACE_WIDTH) << '|' << std::setw(20) << "min time(ns):" << internal_data.target.rt_stastic.min_time_consumed << std::endl
						<< std::setw(aris::core::LOG_SPACE_WIDTH) << '|' << std::setw(20) << "in count:" << internal_data.target.rt_stastic.min_time_occur_count << std::endl
						<< std::setw(aris::core::LOG_SPACE_WIDTH) << '|' << std::setw(20) << "total count:" << internal_data.target.rt_stastic.total_count << std::endl
						<< std::setw(aris::core::LOG_SPACE_WIDTH) << '|' << std::setw(20) << "overruns:" << internal_data.target.rt_stastic.overrun_count << std::endl;



					if (!(internal_data.target.option & aris::plan::Plan::NOT_RUN_COLLECT_FUNCTION)) 
					{
						LOG_INFO << "server collect cmd " << internal_data.target.command_id << std::endl;
						internal_data.plan->collectNrt(internal_data.target);
					}
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
