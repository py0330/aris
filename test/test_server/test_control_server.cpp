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
		cs.resetMaster(new aris::control::Master);
		cs.resetController(new aris::control::Controller);
		cs.resetModel(new aris::dynamic::Model);
		cs.resetPlanRoot(new aris::plan::PlanRoot);

		std::int64_t option = 0;

		std::atomic_int prepare_num{ 0 }, execute_num{ 0 }, collect_num{ 0 };
		cs.planRoot().planPool().add<aris::plan::UniversalPlan>("test", [&](aris::plan::Plan* plan)->void{
			plan->option() = option;
			++prepare_num;
		}, [&](const aris::plan::Plan* plan)->int{
			++execute_num;
			return 0;
		}, [&](aris::plan::Plan* plan)->void{
			++collect_num;
		}, "<Command name=\"test_NOT_RUN_FUNCTION\"/>");
		cs.init();
		cs.open();
		cs.start();

		std::string cmd("test_NOT_RUN_FUNCTION");

		option = aris::plan::Plan::NOT_RUN_EXECUTE_FUNCTION;
		cs.executeCmd(cmd);
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		if ((prepare_num != 1) || (execute_num != 0) || (collect_num != 1))	std::cout << __FILE__ << " " << __LINE__ << ":test NOT_RUN_..._FUNCTION option failed" << std::endl;

		option = aris::plan::Plan::NOT_RUN_COLLECT_FUNCTION;
		cs.executeCmd(cmd);
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		if ((prepare_num != 2) || (execute_num != 1) || (collect_num != 1))	std::cout << __FILE__ << " " << __LINE__ << ":test NOT_RUN_..._FUNCTION option failed" << std::endl;

		cs.stop();
	}

	// test WAIT_IF_CMD_POOL_IS_FULL //
	{
		cs.resetController(new aris::control::Controller);
		cs.resetModel(new aris::dynamic::Model);
		cs.resetPlanRoot(new aris::plan::PlanRoot);
		
		std::int64_t option = 0;
		cs.planRoot().planPool().add<aris::plan::UniversalPlan>("test", [&](aris::plan::Plan* plan)->void{
			plan->option() = option | aris::plan::Plan::NOT_PRINT_CMD_INFO | aris::plan::Plan::NOT_PRINT_EXECUTE_COUNT;
		}, [&](aris::plan::Plan* plan)->int{
			return 1000 - plan->count();
		}, [&](const aris::plan::Plan* plan)->void{
			std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		}, "<Command name=\"test_WAIT_IF_CMD_POOL_IS_FULL_1\"/>");

		cs.planRoot().planPool().add<aris::plan::UniversalPlan>("test", [&](aris::plan::Plan* plan)->void{
			plan->option() =option | aris::plan::Plan::NOT_PRINT_CMD_INFO | aris::plan::Plan::NOT_PRINT_EXECUTE_COUNT | aris::plan::Plan::NOT_PRINT_EXECUTE_COUNT;
		}, [&](aris::plan::Plan* plan)->int	{
			static int i = 0;
			return 0;
		}, [&](const aris::plan::Plan* plan)->void	{
			static int i = 0;
		}, "<Command name=\"test_WAIT_IF_CMD_POOL_IS_FULL_2\"/>");
		cs.init();
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

