#include <iostream>
#include <condition_variable>
#include <future>
#include <aris.hpp>

#include "test_control_server.h"

void test_server_option()
{
	auto&cs = aris::server::ControlServer::instance();
	
	// test NOT_RUN_..._FUNCTION
	{
		cs.resetController(new aris::control::EthercatController);
		cs.resetModel(new aris::dynamic::Model);
		cs.resetSensorRoot(new aris::sensor::SensorRoot);
		cs.resetPlanRoot(new aris::plan::PlanRoot);

		std::int64_t option = 0;

		std::atomic_bool is_plan_prepareed{ false }, is_plan_executed{ false }, is_plan_collected{ false };
		cs.planRoot().planPool().add<aris::plan::UniversalPlan>("test", [&](aris::plan::Plan* plan)->void
		{
			plan->option() = option;
			is_plan_prepareed = true;
		}, [&](const aris::plan::Plan* plan)->int
		{
			is_plan_executed = true;
			return 0;
		}, [&](aris::plan::Plan* plan)->void
		{
			is_plan_collected = true;
		}, "<Command name=\"test_NOT_RUN_FUNCTION\"/>");
		cs.open();
		cs.start();

		std::string cmd("test_NOT_RUN_FUNCTION");

		is_plan_collected = is_plan_executed = is_plan_prepareed = false;
		option = aris::plan::Plan::NOT_RUN_EXECUTE_FUNCTION;
		cs.executeCmd(cmd);
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		if ((is_plan_prepareed == false) || (is_plan_executed == true) || (is_plan_collected == false))	std::cout << __FILE__ << " " << __LINE__ << ":test NOT_RUN_..._FUNCTION option failed" << std::endl;

		is_plan_collected = is_plan_executed = is_plan_prepareed = false;
		option = aris::plan::Plan::NOT_RUN_COLLECT_FUNCTION;
		cs.executeCmd(cmd);
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		if ((is_plan_prepareed == false) || (is_plan_executed == false) || (is_plan_collected == true))	std::cout << __FILE__ << " " << __LINE__ << ":test NOT_RUN_..._FUNCTION option failed" << std::endl;

		cs.stop();
	}

	// test EXECUTE_WHEN_ALL_PLAN_EXECUTED //
	{
		cs.resetController(new aris::control::EthercatController);
		cs.resetModel(new aris::dynamic::Model);
		cs.resetSensorRoot(new aris::sensor::SensorRoot);
		cs.resetPlanRoot(new aris::plan::PlanRoot);

		std::int64_t option = 0;

		cs.planRoot().planPool().add<aris::plan::UniversalPlan>("test", [&](aris::plan::Plan* plan)->void
		{
			plan->option() = option;
		}, [&](aris::plan::Plan* plan)->int
		{
			return 100 - plan->count();
		}, [&](aris::plan::Plan* plan)->void
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}, "<Command name=\"test_EXECUTE_WHEN_ALL_PLAN_EXECUTED\"/>");
		cs.open();
		cs.start();

		std::string cmd("test_EXECUTE_WHEN_ALL_PLAN_EXECUTED");
		option = 0;
		cs.executeCmd(cmd);
		auto ret = cs.executeCmd(cmd);
		if ((!cs.currentExecutePlan()) || cs.currentExecutePlan()->cmdId() == ret->cmdId()) std::cout << __FILE__ << " " << __LINE__ << ":test EXECUTE_WHEN_ALL_PLAN_EXECUTED option failed" << std::endl;

		option = aris::plan::Plan::EXECUTE_WHEN_ALL_PLAN_EXECUTED;
		ret = cs.executeCmd(cmd);
		if (cs.currentExecutePlan() && cs.currentExecutePlan()->cmdId() != ret->cmdId()) std::cout << __FILE__ << " " << __LINE__ << ":test EXECUTE_WHEN_ALL_PLAN_EXECUTED option failed" << std::endl;

		cs.stop();
	}

	// test EXECUTE_WHEN_ALL_PLAN_COLLECTED //
	{
		cs.resetController(new aris::control::EthercatController);
		cs.resetModel(new aris::dynamic::Model);
		cs.resetSensorRoot(new aris::sensor::SensorRoot);
		cs.resetPlanRoot(new aris::plan::PlanRoot);

		std::int64_t option = 0;

		cs.planRoot().planPool().add<aris::plan::UniversalPlan>("test", [&](aris::plan::Plan* plan)->void
		{
			plan->option() = option;
		}, [&](aris::plan::Plan* plan)->int
		{
			return 100 - plan->count();
		}, [&](aris::plan::Plan* plan)->void
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(200));
		}, "<Command name=\"test_EXECUTE_WHEN_ALL_PLAN_COLLECTED\"/>");
		cs.open();
		cs.start();

		std::string cmd("test_EXECUTE_WHEN_ALL_PLAN_COLLECTED");
		option = aris::plan::Plan::EXECUTE_WHEN_ALL_PLAN_EXECUTED;
		cs.executeCmd(cmd);
		auto ret = cs.executeCmd(cmd);
		if (cs.currentCollectPlan()->cmdId() != ret->cmdId() - 1) std::cout << __FILE__ << " " << __LINE__ << ":test EXECUTE_WHEN_ALL_PLAN_COLLECTED option failed" << std::endl;

		option = aris::plan::Plan::EXECUTE_WHEN_ALL_PLAN_COLLECTED;
		ret = cs.executeCmd(cmd);
		if (cs.currentCollectPlan()->cmdId() != ret->cmdId()) std::cout << __FILE__ << " " << __LINE__ << ":test EXECUTE_WHEN_ALL_PLAN_COLLECTED option failed" << std::endl;

		cs.stop();
	}

	// test COLLECT_WHEN_ALL_PLAN_EXECUTED //
	{
		cs.resetController(new aris::control::EthercatController);
		cs.resetModel(new aris::dynamic::Model);
		cs.resetSensorRoot(new aris::sensor::SensorRoot);
		cs.resetPlanRoot(new aris::plan::PlanRoot);

		std::int64_t option = 0;
		cs.planRoot().planPool().add<aris::plan::UniversalPlan>("test", [&](aris::plan::Plan* plan)->void
		{
			plan->option() = option;
		}, [&](aris::plan::Plan* plan)->int
		{
			return 100 - plan->count();
		}, [&](aris::plan::Plan* plan)->void
		{
		}, "<Command name=\"test_COLLECT_WHEN_ALL_PLAN_EXECUTED\"/>");
		cs.open();
		cs.start();

		std::string cmd("test_COLLECT_WHEN_ALL_PLAN_EXECUTED");
		cs.executeCmd(cmd);
		option = aris::plan::Plan::NOT_RUN_EXECUTE_FUNCTION;
		cs.executeCmd(cmd);
		if (!cs.currentExecutePlan()) std::cout << __FILE__ << " " << __LINE__ << ":test COLLECT_WHEN_ALL_PLAN_EXECUTED option failed" << std::endl;

		cs.executeCmd(cmd);
		option = aris::plan::Plan::NOT_RUN_EXECUTE_FUNCTION | aris::plan::Plan::COLLECT_WHEN_ALL_PLAN_EXECUTED;
		cs.executeCmd(cmd);
		if (cs.currentExecutePlan()) std::cout << __FILE__ << " " << __LINE__ << ":test COLLECT_WHEN_ALL_PLAN_EXECUTED option failed" << std::endl;

		cs.stop();
	}

	// test COLLECT_WHEN_ALL_PLAN_COLLECTED //
	{
		cs.resetController(new aris::control::EthercatController);
		cs.resetModel(new aris::dynamic::Model);
		cs.resetSensorRoot(new aris::sensor::SensorRoot);
		cs.resetPlanRoot(new aris::plan::PlanRoot);

		std::int64_t option = 0;
		int collect_time = 100;
		cs.planRoot().planPool().add<aris::plan::UniversalPlan>("test", [&](aris::plan::Plan* plan)->void
		{
			plan->option() = option | aris::plan::Plan::NOT_PRINT_CMD_INFO | aris::plan::Plan::NOT_PRINT_EXECUTE_COUNT;
			plan->param() = collect_time;
		}, [&](aris::plan::Plan* plan)->int
		{
			return 100 - plan->count();
		}, [&](aris::plan::Plan* plan)->void
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(std::any_cast<int>(plan->param())));
		}, "<Command name=\"test_COLLECT_WHEN_ALL_PLAN_COLLECTED\"/>");
		cs.open();
		cs.start();

		std::string cmd("test_COLLECT_WHEN_ALL_PLAN_COLLECTED");
		cs.executeCmd(cmd);
		option = aris::plan::Plan::NOT_RUN_EXECUTE_FUNCTION;
		collect_time = 0;
		cs.executeCmd(cmd);
		if (!cs.currentCollectPlan()) std::cout << __FILE__ << " " << __LINE__ << ":test COLLECT_WHEN_ALL_PLAN_COLLECTED option failed" << std::endl;

		collect_time = 100;
		cs.executeCmd(cmd);
		option = aris::plan::Plan::NOT_RUN_EXECUTE_FUNCTION | aris::plan::Plan::COLLECT_WHEN_ALL_PLAN_COLLECTED;
		collect_time = 0;
		cs.executeCmd(cmd);
		if (cs.currentCollectPlan()) std::cout << __FILE__ << " " << __LINE__ << ":test COLLECT_WHEN_ALL_PLAN_COLLECTED option failed" << std::endl;

		cs.stop();
	}

	// test WAIT_FOR_EXECUTION //
	{
		cs.resetController(new aris::control::EthercatController);
		cs.resetModel(new aris::dynamic::Model);
		cs.resetSensorRoot(new aris::sensor::SensorRoot);
		cs.resetPlanRoot(new aris::plan::PlanRoot);

		std::int64_t option = 0;
		cs.planRoot().planPool().add<aris::plan::UniversalPlan>("test", [&](aris::plan::Plan* plan)->void
		{
			plan->option() = option | aris::plan::Plan::NOT_PRINT_CMD_INFO | aris::plan::Plan::NOT_PRINT_EXECUTE_COUNT;
		}, [&](aris::plan::Plan* plan)->int
		{
			return 100 - plan->count();
		}, [&](aris::plan::Plan* plan)->void
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}, "<Command name=\"test_WAIT_FOR_EXECUTION\"/>");
		cs.open();
		cs.start();

		std::string cmd("test_WAIT_FOR_EXECUTION");
		option = 0;
		cs.executeCmd(cmd);
		cs.executeCmd(cmd);
		if (!cs.currentExecutePlan()) std::cout << __FILE__ << " " << __LINE__ << ":test WAIT_FOR_EXECUTION option failed" << std::endl;

		option = aris::plan::Plan::WAIT_FOR_EXECUTION;
		cs.executeCmd(cmd);
		if (cs.currentExecutePlan()|| (!cs.currentCollectPlan())) std::cout << __FILE__ << " " << __LINE__ << ":test WAIT_FOR_EXECUTION option failed" << std::endl;

		cs.stop();
	}

	// test WAIT_FOR_COLLECTION //
	{
		cs.resetController(new aris::control::EthercatController);
		cs.resetModel(new aris::dynamic::Model);
		cs.resetSensorRoot(new aris::sensor::SensorRoot);
		cs.resetPlanRoot(new aris::plan::PlanRoot);

		std::int64_t option = 0;
		cs.planRoot().planPool().add<aris::plan::UniversalPlan>("test", [&](aris::plan::Plan* plan)->void
		{
			plan->option() = option | aris::plan::Plan::NOT_PRINT_CMD_INFO | aris::plan::Plan::NOT_PRINT_EXECUTE_COUNT;
		}, [&](aris::plan::Plan* plan)->int
		{
			return 100 - plan->count();
		}, [&](aris::plan::Plan* plan)->void
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}, "<Command name=\"test_WAIT_FOR_COLLECTION\"/>");
		cs.open();
		cs.start();

		std::string cmd("test_WAIT_FOR_COLLECTION");
		option = 0;
		cs.executeCmd(cmd);
		cs.executeCmd(cmd);
		if (!cs.currentCollectPlan()) std::cout << __FILE__ << " " << __LINE__ << ":test WAIT_FOR_COLLECTION option failed" << std::endl;

		option = aris::plan::Plan::WAIT_FOR_COLLECTION;
		cs.executeCmd(cmd);
		if (cs.currentCollectPlan()) std::cout << __FILE__ << " " << __LINE__ << ":test WAIT_FOR_COLLECTION option failed" << std::endl;

		cs.stop();
	}
	
	// test WAIT_IF_CMD_POOL_IS_FULL //
	{
		cs.resetController(new aris::control::EthercatController);
		cs.resetModel(new aris::dynamic::Model);
		cs.resetSensorRoot(new aris::sensor::SensorRoot);
		cs.resetPlanRoot(new aris::plan::PlanRoot);
		
		std::int64_t option = 0;
		cs.planRoot().planPool().add<aris::plan::UniversalPlan>("test", [&](aris::plan::Plan* plan)->void
		{
			plan->option() = option | aris::plan::Plan::NOT_PRINT_CMD_INFO | aris::plan::Plan::NOT_PRINT_EXECUTE_COUNT;
		}, [&](aris::plan::Plan* plan)->int
		{
			return 1000 - plan->count();
		}, [&](const aris::plan::Plan* plan)->void
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		}, "<Command name=\"test_WAIT_IF_CMD_POOL_IS_FULL_1\"/>");

		cs.planRoot().planPool().add<aris::plan::UniversalPlan>("test", [&](aris::plan::Plan* plan)->void
		{
			plan->option() =option | aris::plan::Plan::NOT_PRINT_CMD_INFO | aris::plan::Plan::NOT_PRINT_EXECUTE_COUNT | aris::plan::Plan::NOT_PRINT_EXECUTE_COUNT;
		}, [&](aris::plan::Plan* plan)->int
		{
			static int i = 0;
			//std::cout <<"e"<< ++i << std::endl;
			return 0;
		}, [&](const aris::plan::Plan* plan)->void
		{
			static int i = 0;
			//std::cout << "c" << ++i << std::endl;

		}, "<Command name=\"test_WAIT_IF_CMD_POOL_IS_FULL_2\"/>");
		cs.open();
		cs.start();

		std::string cmd("test_WAIT_IF_CMD_POOL_IS_FULL_1");
		option = 0;
		cs.executeCmd(cmd);
		cmd = "test_WAIT_IF_CMD_POOL_IS_FULL_2";
		for (auto i = 0; i < 999; ++i)cs.executeCmd(cmd);
		auto p = cs.executeCmd(cmd);
		if (p->retCode() == 0)
			std::cout << __FILE__ << " " << __LINE__ << ":test WAIT_IF_CMD_POOL_IS_FULL option failed" << std::endl;


		option = aris::plan::Plan::WAIT_IF_CMD_POOL_IS_FULL;
		p = cs.executeCmd(cmd);
		if (p->retCode() != 0)std::cout << __FILE__ << " " << __LINE__ << ":test WAIT_IF_CMD_POOL_IS_FULL option failed" << std::endl;

		cs.waitForAllCollection();

		cs.stop();
	}
}



void test_control_server()
{
	test_server_option();
}

