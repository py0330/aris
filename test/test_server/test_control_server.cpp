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

		std::atomic_bool is_plan_prepaired{ false }, is_plan_executed{ false }, is_plan_collected{ false };
		cs.planRoot().planPool().add<aris::plan::UniversalPlan>("test", [&](const std::map<std::string, std::string> &, aris::plan::PlanTarget &target)->void
		{
			target.option = option;
			
			is_plan_prepaired = true;
		}, [&](const aris::plan::PlanTarget &)->int
		{
			is_plan_executed = true;
			return 0;
		}, [&](aris::plan::PlanTarget &)->void
		{
			is_plan_collected = true;
		}, "<Command name=\"test_NOT_RUN_FUNCTION\"/>");

		cs.start();

		aris::core::Msg cmd("test_NOT_RUN_FUNCTION");

		is_plan_collected = is_plan_executed = is_plan_prepaired = false;
		option = aris::plan::Plan::NOT_RUN_EXECUTE_FUNCTION;
		cs.executeCmd(cmd);
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		if ((is_plan_prepaired == false) || (is_plan_executed == true) || (is_plan_collected == false))	std::cout << __FILE__ << " " << __LINE__ << ":test NOT_RUN_..._FUNCTION option failed" << std::endl;

		is_plan_collected = is_plan_executed = is_plan_prepaired = false;
		option = aris::plan::Plan::NOT_RUN_COLLECT_FUNCTION;
		cs.executeCmd(cmd);
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		if ((is_plan_prepaired == false) || (is_plan_executed == false) || (is_plan_collected == true))	std::cout << __FILE__ << " " << __LINE__ << ":test NOT_RUN_..._FUNCTION option failed" << std::endl;

		cs.stop();
	}

	// test EXECUTE_WHEN_ALL_PLAN_EXECUTED //
	{
		cs.resetController(new aris::control::EthercatController);
		cs.resetModel(new aris::dynamic::Model);
		cs.resetSensorRoot(new aris::sensor::SensorRoot);
		cs.resetPlanRoot(new aris::plan::PlanRoot);

		std::int64_t option = 0;

		cs.planRoot().planPool().add<aris::plan::UniversalPlan>("test", [&](const std::map<std::string, std::string> &, aris::plan::PlanTarget &target)->void
		{
			target.option = option;
		}, [&](const aris::plan::PlanTarget &param)->int
		{
			return 100 - param.count;
		}, [&](aris::plan::PlanTarget &)->void
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}, "<Command name=\"test_EXECUTE_WHEN_ALL_PLAN_EXECUTED\"/>");

		cs.start();

		aris::core::Msg cmd("test_EXECUTE_WHEN_ALL_PLAN_EXECUTED");
		option = 0;
		cs.executeCmd(cmd);
		auto ret = cs.executeCmd(cmd);
		if (cs.currentExecuteId() == ret->command_id || cs.currentExecuteId() == 0) std::cout << __FILE__ << " " << __LINE__ << ":test EXECUTE_WHEN_ALL_PLAN_EXECUTED option failed" << std::endl;

		option = aris::plan::Plan::EXECUTE_WHEN_ALL_PLAN_EXECUTED;
		ret = cs.executeCmd(cmd);
		if (cs.currentExecuteId() != ret->command_id && cs.currentExecuteId() != 0) std::cout << __FILE__ << " " << __LINE__ << ":test EXECUTE_WHEN_ALL_PLAN_EXECUTED option failed" << std::endl;

		cs.stop();
	}

	// test EXECUTE_WHEN_ALL_PLAN_COLLECTED //
	{
		cs.resetController(new aris::control::EthercatController);
		cs.resetModel(new aris::dynamic::Model);
		cs.resetSensorRoot(new aris::sensor::SensorRoot);
		cs.resetPlanRoot(new aris::plan::PlanRoot);

		std::int64_t option = 0;

		cs.planRoot().planPool().add<aris::plan::UniversalPlan>("test", [&](const std::map<std::string, std::string> &, aris::plan::PlanTarget &target)->void
		{
			target.option = option;
		}, [&](const aris::plan::PlanTarget &param)->int
		{
			return 100 - param.count;
		}, [&](aris::plan::PlanTarget &)->void
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}, "<Command name=\"test_EXECUTE_WHEN_ALL_PLAN_COLLECTED\"/>");

		cs.start();

		aris::core::Msg cmd("test_EXECUTE_WHEN_ALL_PLAN_COLLECTED");
		option = aris::plan::Plan::EXECUTE_WHEN_ALL_PLAN_EXECUTED;
		cs.executeCmd(cmd);
		auto ret = cs.executeCmd(cmd);
		if (cs.currentCollectId() != ret->command_id - 1) std::cout << __FILE__ << " " << __LINE__ << ":test EXECUTE_WHEN_ALL_PLAN_COLLECTED option failed" << std::endl;

		option = aris::plan::Plan::EXECUTE_WHEN_ALL_PLAN_COLLECTED;
		ret = cs.executeCmd(cmd);
		if (cs.currentCollectId() != ret->command_id) std::cout << __FILE__ << " " << __LINE__ << ":test EXECUTE_WHEN_ALL_PLAN_COLLECTED option failed" << std::endl;

		cs.stop();
	}

	// test COLLECT_WHEN_ALL_PLAN_EXECUTED //
	{
		cs.resetController(new aris::control::EthercatController);
		cs.resetModel(new aris::dynamic::Model);
		cs.resetSensorRoot(new aris::sensor::SensorRoot);
		cs.resetPlanRoot(new aris::plan::PlanRoot);

		std::int64_t option = 0;
		cs.planRoot().planPool().add<aris::plan::UniversalPlan>("test", [&](const std::map<std::string, std::string> &, aris::plan::PlanTarget &target)->void
		{
			target.option = option;
		}, [&](const aris::plan::PlanTarget &param)->int
		{
			return 100 - param.count;
		}, [&](aris::plan::PlanTarget &)->void
		{
		}, "<Command name=\"test_COLLECT_WHEN_ALL_PLAN_EXECUTED\"/>");

		cs.start();

		aris::core::Msg cmd("test_COLLECT_WHEN_ALL_PLAN_EXECUTED");
		cs.executeCmd(cmd);
		option = aris::plan::Plan::NOT_RUN_EXECUTE_FUNCTION;
		cs.executeCmd(cmd);
		if (cs.currentExecuteId() == 0) std::cout << __FILE__ << " " << __LINE__ << ":test COLLECT_WHEN_ALL_PLAN_EXECUTED option failed" << std::endl;

		cs.executeCmd(cmd);
		option = aris::plan::Plan::NOT_RUN_EXECUTE_FUNCTION | aris::plan::Plan::COLLECT_WHEN_ALL_PLAN_EXECUTED;
		cs.executeCmd(cmd);
		if (cs.currentExecuteId() != 0) std::cout << __FILE__ << " " << __LINE__ << ":test COLLECT_WHEN_ALL_PLAN_EXECUTED option failed" << std::endl;

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
		cs.planRoot().planPool().add<aris::plan::UniversalPlan>("test", [&](const std::map<std::string, std::string> &, aris::plan::PlanTarget &t)->void
		{
			t.option = option | aris::plan::Plan::NOT_PRINT_CMD_INFO | aris::plan::Plan::NOT_PRINT_EXECUTE_COUNT;
			t.param = collect_time;
		}, [&](const aris::plan::PlanTarget &param)->int
		{
			return 100 - param.count;
		}, [&](aris::plan::PlanTarget &param)->void
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(std::any_cast<int>(param.param)));
		}, "<Command name=\"test_COLLECT_WHEN_ALL_PLAN_COLLECTED\"/>");

		cs.start();

		aris::core::Msg cmd("test_COLLECT_WHEN_ALL_PLAN_COLLECTED");
		cs.executeCmd(cmd);
		option = aris::plan::Plan::NOT_RUN_EXECUTE_FUNCTION;
		collect_time = 0;
		cs.executeCmd(cmd);
		if (cs.currentCollectId() == 0) std::cout << __FILE__ << " " << __LINE__ << ":test COLLECT_WHEN_ALL_PLAN_COLLECTED option failed" << std::endl;

		collect_time = 100;
		cs.executeCmd(cmd);
		option = aris::plan::Plan::NOT_RUN_EXECUTE_FUNCTION | aris::plan::Plan::COLLECT_WHEN_ALL_PLAN_COLLECTED;
		collect_time = 0;
		cs.executeCmd(cmd);
		if (cs.currentCollectId() != 0) std::cout << __FILE__ << " " << __LINE__ << ":test COLLECT_WHEN_ALL_PLAN_COLLECTED option failed" << std::endl;

		cs.stop();
	}

	// test WAIT_FOR_EXECUTION //
	{
		cs.resetController(new aris::control::EthercatController);
		cs.resetModel(new aris::dynamic::Model);
		cs.resetSensorRoot(new aris::sensor::SensorRoot);
		cs.resetPlanRoot(new aris::plan::PlanRoot);

		std::int64_t option = 0;
		cs.planRoot().planPool().add<aris::plan::UniversalPlan>("test", [&](const std::map<std::string, std::string> &, aris::plan::PlanTarget &t)->void
		{
			t.option = option | aris::plan::Plan::NOT_PRINT_CMD_INFO | aris::plan::Plan::NOT_PRINT_EXECUTE_COUNT;
		}, [&](const aris::plan::PlanTarget &param)->int
		{
			return 100 - param.count;
		}, [&](aris::plan::PlanTarget &)->void
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}, "<Command name=\"test_WAIT_FOR_EXECUTION\"/>");

		cs.start();

		aris::core::Msg cmd("test_WAIT_FOR_EXECUTION");
		option = 0;
		cs.executeCmd(cmd);
		cs.executeCmd(cmd);
		if (cs.currentExecuteId() == 0) std::cout << __FILE__ << " " << __LINE__ << ":test WAIT_FOR_EXECUTION option failed" << std::endl;

		option = aris::plan::Plan::WAIT_FOR_EXECUTION;
		cs.executeCmd(cmd);
		if (cs.currentExecuteId() != 0 || cs.currentCollectId() == 0) std::cout << __FILE__ << " " << __LINE__ << ":test WAIT_FOR_EXECUTION option failed" << std::endl;

		cs.stop();
	}

	// test WAIT_FOR_COLLECTION //
	{
		cs.resetController(new aris::control::EthercatController);
		cs.resetModel(new aris::dynamic::Model);
		cs.resetSensorRoot(new aris::sensor::SensorRoot);
		cs.resetPlanRoot(new aris::plan::PlanRoot);

		std::int64_t option = 0;
		cs.planRoot().planPool().add<aris::plan::UniversalPlan>("test", [&](const std::map<std::string, std::string> &, aris::plan::PlanTarget &t)->void
		{
			t.option = option | aris::plan::Plan::NOT_PRINT_CMD_INFO | aris::plan::Plan::NOT_PRINT_EXECUTE_COUNT;
		}, [&](const aris::plan::PlanTarget &param)->int
		{
			return 100 - param.count;
		}, [&](aris::plan::PlanTarget &)->void
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}, "<Command name=\"test_WAIT_FOR_COLLECTION\"/>");

		cs.start();

		aris::core::Msg cmd("test_WAIT_FOR_COLLECTION");
		option = 0;
		cs.executeCmd(cmd);
		cs.executeCmd(cmd);
		if (cs.currentCollectId() == 0) std::cout << __FILE__ << " " << __LINE__ << ":test WAIT_FOR_COLLECTION option failed" << std::endl;

		option = aris::plan::Plan::WAIT_FOR_COLLECTION;
		cs.executeCmd(cmd);
		if (cs.currentCollectId() != 0) std::cout << __FILE__ << " " << __LINE__ << ":test WAIT_FOR_COLLECTION option failed" << std::endl;

		cs.stop();
	}
	
	// test WAIT_IF_CMD_POOL_IS_FULL //
	{
		cs.resetController(new aris::control::EthercatController);
		cs.resetModel(new aris::dynamic::Model);
		cs.resetSensorRoot(new aris::sensor::SensorRoot);
		cs.resetPlanRoot(new aris::plan::PlanRoot);
		
		std::int64_t option = 0;
		cs.planRoot().planPool().add<aris::plan::UniversalPlan>("test", [&](const std::map<std::string, std::string> &, aris::plan::PlanTarget &t)->void
		{
			t.option = option | aris::plan::Plan::NOT_PRINT_CMD_INFO | aris::plan::Plan::NOT_PRINT_EXECUTE_COUNT;
		}, [&](const aris::plan::PlanTarget &param)->int
		{
			return 1000 - param.count;
		}, [&](const aris::plan::PlanTarget &)->void
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		}, "<Command name=\"test_WAIT_IF_CMD_POOL_IS_FULL_1\"/>");

		cs.planRoot().planPool().add<aris::plan::UniversalPlan>("test", [&](const std::map<std::string, std::string> &, aris::plan::PlanTarget &t)->void
		{
			t.option =option | aris::plan::Plan::NOT_PRINT_CMD_INFO | aris::plan::Plan::NOT_PRINT_EXECUTE_COUNT | aris::plan::Plan::NOT_PRINT_EXECUTE_COUNT;
		}, [&](const aris::plan::PlanTarget &param)->int
		{
			return 0;
		}, [&](const aris::plan::PlanTarget &)->void
		{
		}, "<Command name=\"test_WAIT_IF_CMD_POOL_IS_FULL_2\"/>");

		cs.start();

		aris::core::Msg cmd("test_WAIT_IF_CMD_POOL_IS_FULL_1");
		option = 0;
		cs.executeCmd(cmd);
		cmd.copy("test_WAIT_IF_CMD_POOL_IS_FULL_2");
		for (auto i = 0; i<999; ++i)cs.executeCmd(cmd);
		try 
		{
			cs.executeCmd(cmd);
			std::cout << __FILE__ << " " << __LINE__ << ":test WAIT_IF_CMD_POOL_IS_FULL option failed" << std::endl;
		}
		catch (std::exception &) {}

		try 
		{
			option = aris::plan::Plan::WAIT_IF_CMD_POOL_IS_FULL;
			cs.executeCmd(cmd);
		}
		catch (std::exception &) 
		{
			std::cout << __FILE__ << " " << __LINE__ << ":test WAIT_IF_CMD_POOL_IS_FULL option failed" << std::endl;
		}

		while (cs.currentCollectId() != 0)std::this_thread::yield();


		cs.stop();
	}
}



void test_control_server()
{
	test_server_option();
}

