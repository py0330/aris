#include <iostream>
#include <condition_variable>
#include <future>
#include <aris.hpp>

#include "test_control_server.h"

void test_server_option(){
	
	auto& cs = aris::server::ControlServer::instance();

	// 测试选项：NOT_RUN_EXECUTE_FUNCTION  &  NOT_RUN_COLLECT_FUNCTION
	{
		std::cout << "test plan params" << std::endl;
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

		auto opt = aris::plan::Plan::NOT_PRINT_CMD_INFO | aris::plan::Plan::NOT_PRINT_EXECUTE_COUNT;

		option = opt;
		cs.executeCmd(cmd);
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		if ((prepare_num != 1) || (execute_num != 1) || (collect_num != 1))	std::cout << __FILE__ << " " << __LINE__ << ":test NOT_RUN_..._FUNCTION option failed" << std::endl;

		option = opt | aris::plan::Plan::NOT_RUN_EXECUTE_FUNCTION;
		cs.executeCmd(cmd);
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		if ((prepare_num != 2) || (execute_num != 1) || (collect_num != 2))	std::cout << __FILE__ << " " << __LINE__ << ":test NOT_RUN_..._FUNCTION option failed" << std::endl;

		option = opt | aris::plan::Plan::NOT_RUN_COLLECT_FUNCTION;
		cs.executeCmd(cmd);
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		if ((prepare_num != 3) || (execute_num != 2) || (collect_num != 2))	std::cout << __FILE__ << " " << __LINE__ << ":test NOT_RUN_..._FUNCTION option failed" << std::endl;

		option = opt | aris::plan::Plan::NOT_RUN_EXECUTE_FUNCTION | aris::plan::Plan::NOT_RUN_COLLECT_FUNCTION;
		cs.executeCmd(cmd);
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		if ((prepare_num != 4) || (execute_num != 2) || (collect_num != 2))	std::cout << __FILE__ << " " << __LINE__ << ":test NOT_RUN_..._FUNCTION option failed" << std::endl;

		option = opt;
		cs.executeCmd(cmd);
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		if ((prepare_num != 5) || (execute_num != 3) || (collect_num != 3))	std::cout << __FILE__ << " " << __LINE__ << ":test NOT_RUN_..._FUNCTION option failed" << std::endl;

		option = opt | aris::plan::Plan::NOT_RUN_EXECUTE_FUNCTION;
		cs.executeCmd(cmd);
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		if ((prepare_num != 6) || (execute_num != 3) || (collect_num != 4))	std::cout << __FILE__ << " " << __LINE__ << ":test NOT_RUN_..._FUNCTION option failed" << std::endl;

		option = opt | aris::plan::Plan::NOT_RUN_COLLECT_FUNCTION;
		cs.executeCmd(cmd);
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		if ((prepare_num != 7) || (execute_num != 4) || (collect_num != 4))	std::cout << __FILE__ << " " << __LINE__ << ":test NOT_RUN_..._FUNCTION option failed" << std::endl;

		option = opt | aris::plan::Plan::NOT_RUN_EXECUTE_FUNCTION | aris::plan::Plan::NOT_RUN_COLLECT_FUNCTION;
		cs.executeCmd(cmd);
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		if ((prepare_num != 8) || (execute_num != 4) || (collect_num != 4))	std::cout << __FILE__ << " " << __LINE__ << ":test NOT_RUN_..._FUNCTION option failed" << std::endl;

		std::vector<std::pair<std::string, std::function<void(aris::plan::Plan&)>>> cmds(4, { cmd, nullptr });

		option = opt;
		cs.executeCmd(cmds);
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		if ((prepare_num != 12) || (execute_num != 8) || (collect_num != 8))	std::cout << __FILE__ << " " << __LINE__ << ":test NOT_RUN_..._FUNCTION option failed" << std::endl;

		option = opt | aris::plan::Plan::NOT_RUN_EXECUTE_FUNCTION;
		cs.executeCmd(cmds);
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		if ((prepare_num != 16) || (execute_num != 8) || (collect_num != 12))	std::cout << __FILE__ << " " << __LINE__ << ":test NOT_RUN_..._FUNCTION option failed" << std::endl;

		option = opt | aris::plan::Plan::NOT_RUN_COLLECT_FUNCTION;
		cs.executeCmd(cmds);
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		if ((prepare_num != 20) || (execute_num != 12) || (collect_num != 12))	std::cout << __FILE__ << " " << __LINE__ << ":test NOT_RUN_..._FUNCTION option failed" << std::endl;

		option = opt | aris::plan::Plan::NOT_RUN_EXECUTE_FUNCTION | aris::plan::Plan::NOT_RUN_COLLECT_FUNCTION;
		cs.executeCmd(cmds);
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		if ((prepare_num != 24) || (execute_num != 12) || (collect_num != 12))	std::cout << __FILE__ << " " << __LINE__ << ":test NOT_RUN_..._FUNCTION option failed" << std::endl;

		cs.stop();
		std::cout << "test plan params end" << std::endl;
	}


}
void test_server_parse_exception(){
	// 测试【Parse Exception】
	auto& cs = aris::server::ControlServer::instance();
	std::cout << "test parse exception" << std::endl;
	cs.resetMaster(new aris::control::Master);
	cs.resetController(new aris::control::Controller);
	cs.resetModel(new aris::dynamic::Model);
	cs.resetPlanRoot(new aris::plan::PlanRoot);

	std::int64_t option = aris::plan::Plan::NOT_PRINT_CMD_INFO | aris::plan::Plan::NOT_PRINT_EXECUTE_COUNT;
	std::atomic_int prepare_num{ 0 }, execute_num{ 0 }, collect_num{ 0 };

	cs.planRoot().planPool().add<aris::plan::UniversalPlan>("test", [&](aris::plan::Plan* plan)->void {
			plan->option() = option;
			++prepare_num;
		}, [&](const aris::plan::Plan* plan)->int {
			++execute_num;
			return 0;
		}, [&](aris::plan::Plan* plan)->void {
			++collect_num;
		}, "<Command name=\"test_PARSE_EXCEPTION\"/>");
	cs.init();
	cs.open();
	cs.start();

	std::string cmd1("test_PARSE_EXCEPTION");
	std::string cmd2("test_PARSE_EXCEPTION2");

	int exe_count = 0;
	aris::server::ControlServer::ExecuteCmdCallback callback = [&exe_count](aris::plan::Plan&) {
		exe_count++;
	};

	for (int i = 0; i < 2; ++i) {
		auto ret = cs.executeCmd(cmd1);
		if (!ret || ret->retCode() != aris::plan::Plan::SUCCESS) std::cout << __FILE__ << " " << __LINE__ << ":test NOT_RUN_..._FUNCTION option failed" << std::endl;

		ret = cs.executeCmd(cmd2);
		if (!ret || ret->retCode() != aris::plan::Plan::PARSE_EXCEPTION) std::cout << __FILE__ << " " << __LINE__ << ":test NOT_RUN_..._FUNCTION option failed" << std::endl;
	}

	std::this_thread::sleep_for(std::chrono::milliseconds(100));

	for (int i = 0; i < 2; ++i) {
		auto ret = cs.executeCmd({ {cmd1, callback}, {cmd1, callback}, {cmd1, callback}, {cmd2, callback}, {cmd1, callback}, {cmd2, callback} });
		if (ret.size() != 6 ||
			ret[0]->retCode() != aris::plan::Plan::PREPARE_CANCELLED ||
			ret[1]->retCode() != aris::plan::Plan::PREPARE_CANCELLED ||
			ret[2]->retCode() != aris::plan::Plan::PREPARE_CANCELLED ||
			ret[3]->retCode() != aris::plan::Plan::PARSE_EXCEPTION ||
			ret[4] ||
			ret[5] ||
			exe_count != (i + 1) * 6 || 
			prepare_num != 2 ||
			execute_num != 2 ||
			collect_num != 2
			) std::cout << __FILE__ << " " << __LINE__ << ":test PARSE EXCEPTION failed" << std::endl;
	}

	cs.stop();
	std::cout << "test parse exception end" << std::endl;
}

void test_control_server(){
	//test_server_option();
	test_server_parse_exception();
}

