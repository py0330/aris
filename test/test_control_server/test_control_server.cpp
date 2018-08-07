#include <iostream>
#include <condition_variable>
#include <future>
#include <aris.h>

#include "test_control_server.h"

//class TestPlan : public Plan
//{
//public:
//	static auto Type()->const std::string & { static const std::string type("EnablePlan"); return std::ref(type); }
//	auto virtual type() const->const std::string& override { return Type(); }
//	auto virtual prepairNrt(const PlanParam &param, const std::map<std::string, std::string> &cmd_params, aris::core::Msg &msg_out)->void override;
//	auto virtual runRT(const PlanParam &param)->int override;
//	auto virtual finishNrt()->void override;
//
//	virtual ~EnablePlan();
//	explicit EnablePlan(const std::string &name = "enable_plan");
//	EnablePlan(const EnablePlan &);
//	EnablePlan(EnablePlan &&);
//	EnablePlan& operator=(const EnablePlan &);
//	EnablePlan& operator=(EnablePlan &&);
//
//private:
//	struct Imp;
//	aris::core::ImpPtr<Imp> imp_;
//};


void test_server2()
{
	auto&cs = aris::server::ControlServer::instance();

	cs.resetController(new aris::control::EthercatController);
	cs.resetModel(new aris::dynamic::Model);
	cs.resetSensorRoot(new aris::sensor::SensorRoot);
	cs.resetPlanRoot(new aris::plan::PlanRoot);

	cs.planRoot().planPool().add<aris::plan::Plan>("en").command().loadXmlStr(
		"		<en default_child_type=\"Param\" default=\"all\">"
		"			<all abbreviation=\"a\"/>"
		"			<first abbreviation=\"f\"/>"
		"			<second abbreviation=\"s\"/>"
		"			<motion_id abbreviation=\"m\" default=\"0\"/>"
		"			<physical_id abbreviation=\"p\" default=\"0\"/>"
		"			<leg abbreviation=\"l\" default=\"0\"/>"
		"		</en>");

	cs.planRoot().planPool().add<aris::plan::Plan>("ds").command().loadXmlStr(
		"		<ds default_child_type=\"Param\" default=\"all\">"
		"			<all abbreviation=\"a\"/>"
		"			<first abbreviation=\"f\"/>"
		"			<second abbreviation=\"s\"/>"
		"			<motion_id abbreviation=\"m\" default=\"0\"/>"
		"			<physical_id abbreviation=\"p\" default=\"0\"/>"
		"			<leg abbreviation=\"l\" default=\"0\"/>"
		"		</ds>");


	

	// 接收命令 //
	for (std::string command_in; std::getline(std::cin, command_in);)
	{
		try
		{
			if (command_in == "start")
			{
				cs.start();
			}
			else if(command_in == "stop")
			{
				cs.stop();
			}
			else
			{
				cs.executeCmd(aris::core::Msg(command_in));
			}
		}
		catch (std::exception &e)
		{
			std::cout << e.what() << std::endl;
		}
	}


	cs.stop();
}


void test_server_option()
{
	auto&cs = aris::server::ControlServer::instance();

	cs.resetController(new aris::control::EthercatController);
	cs.resetModel(new aris::dynamic::Model);
	cs.resetSensorRoot(new aris::sensor::SensorRoot);
	cs.resetPlanRoot(new aris::plan::PlanRoot);

	std::atomic_bool is_plan_prepaired{ false }, is_plan_executed{ false }, is_plan_collected{ false };
	cs.planRoot().planPool().add<aris::plan::UniversalPlan>("test", [&](const aris::plan::PlanParam &, const std::map<std::string, std::string> &, aris::core::Msg &)->void
	{
		is_plan_prepaired = true;
	}, [&](const aris::plan::PlanParam &)->int
	{
		is_plan_executed = true;
		return 0;
	}, [&]()->void
	{
		is_plan_collected = true;
	},"<test/>");

	cs.start();

	{
		is_plan_collected = is_plan_executed = is_plan_prepaired = false;
		aris::core::Msg cmd("test");
		cmd.header().reserved1_ |= aris::plan::Plan::NOT_RUN_PREPAIR_FUNCTION;
		cs.executeCmd(cmd);
		std::this_thread::sleep_for(std::chrono::seconds(1));
		if ((is_plan_prepaired == true) || (is_plan_executed == false) || (is_plan_collected == false))	std::cout << __FILE__ << " " << __LINE__ << ":test option failed" << std::endl;
	}
	{
		is_plan_collected = is_plan_executed = is_plan_prepaired = false;
		aris::core::Msg cmd("test");
		cmd.header().reserved1_ |= aris::plan::Plan::NOT_RUN_EXECUTE_FUNCTION;
		cs.executeCmd(cmd);
		std::this_thread::sleep_for(std::chrono::seconds(1));
		if ((is_plan_prepaired == false) || (is_plan_executed == true) || (is_plan_collected == false))	std::cout << __FILE__ << " " << __LINE__ << ":test option failed" << std::endl;
	}
	{
		is_plan_collected = is_plan_executed = is_plan_prepaired = false;
		aris::core::Msg cmd("test");
		cmd.header().reserved1_ |= aris::plan::Plan::NOT_RUN_COLLECT_FUNCTION;
		cs.executeCmd(cmd);
		std::this_thread::sleep_for(std::chrono::seconds(1));
		if ((is_plan_prepaired == false) || (is_plan_executed == false) || (is_plan_collected == true))	std::cout << __FILE__ << " " << __LINE__ << ":test option failed" << std::endl;
	}
	



	//std::this_thread::sleep_for(std::chrono::seconds(3));
	cs.stop();

}


void test_server_ur()
{
	auto&cs = aris::server::ControlServer::instance();

	cs.resetController(aris::robot::createUr5Controller().release());
	cs.resetModel(aris::robot::createUr5Model().release());
	cs.resetSensorRoot(new aris::sensor::SensorRoot);
	cs.resetPlanRoot(new aris::plan::PlanRoot);

	cs.planRoot().planPool().add<aris::plan::EnablePlan>();
	cs.planRoot().planPool().add<aris::plan::MovePlan>();

	aris::core::Socket socket;
	socket.setOnReceivedMsg([&](aris::core::Socket *, aris::core::Msg &msg)->int
	{
		std::string msg_data(msg.data(), msg.size());
		std::cout << "received normal msg:" << std::endl;
		std::cout << "     size : " << msg.header().msg_size_ << std::endl;
		std::cout << "       id : " << msg.header().msg_id_ << std::endl;
		std::cout << "   option : " << msg.header().msg_type_ << std::endl;
		std::cout << "reserved1 : " << msg.header().reserved1_ << std::endl;
		std::cout << "reserved2 : " << msg.header().reserved2_ << std::endl;
		std::cout << "reserved3 : " << msg.header().reserved3_ << std::endl;
		std::cout << "     data : " << msg_data << std::endl;

		return 0;
	});
	socket.setOnReceivedRequest([&](aris::core::Socket *, aris::core::Msg &msg)
	{
		std::string msg_data(msg.data(), msg.size());
		
		std::cout << "received request msg:" << std::endl;
		std::cout << "     size : " << msg.header().msg_size_ << std::endl;
		std::cout << "       id : " << msg.header().msg_id_ << std::endl;
		std::cout << "   option : " << msg.header().msg_type_ << std::endl;
		std::cout << "reserved1 : " << msg.header().reserved1_ << std::endl;
		std::cout << "reserved2 : " << msg.header().reserved2_ << std::endl;
		std::cout << "reserved3 : " << msg.header().reserved3_ << std::endl;
		std::cout << "     data : " << msg_data << std::endl;

		if (msg.header().msg_id_ == 0)
		{
			try 
			{
				auto id = cs.executeCmd(aris::core::Msg(msg_data));
				std::cout << "command id:" << id << std::endl;
				return aris::core::Msg();
			}
			catch (std::exception &e)
			{
				return aris::core::Msg(e.what());
			}
		}
		else if (msg.header().msg_id_ == 1)
		{
			auto part_pm = cs.getPartPm();
			std::vector<double> part_pq(cs.model().partPool().size() * 7, 0.0);

			for (aris::Size i(-1); ++i < cs.model().partPool().size();)
			{
				aris::dynamic::s_pm2pq(part_pm.data() + i * 16, part_pq.data() + i * 7);
			}

			aris::core::Matrix mat(1, cs.model().partPool().size() * 7, part_pq.data());

			std::cout << mat.toString() << std::endl;

			return aris::core::Msg(mat.toString());
		}

		return aris::core::Msg("unknown msg id");
	});
	socket.setOnReceivedConnection([](aris::core::Socket *sock, const char *ip, int port)->int 
	{
		std::cout << "received connection" << std::endl;
		std::cout << "  ip:" << ip << std::endl;;
		std::cout << "port:" << port << std::endl;
		return 0;

	});
	socket.setOnLoseConnection([](aris::core::Socket *socket)
	{
		aris::core::log("lost connection");
		std::cout << "lost connection" << std::endl;
		while (true)
		{
			try
			{
				socket->startServer("5866");
				break;
			}
			catch (aris::core::Socket::StartServerError &e)
			{
				std::cout << e.what() << std::endl << "will try to restart server socket in 1s" << std::endl;
				std::this_thread::sleep_for(std::chrono::seconds(1));
			}
		}
		aris::core::log("restart server socket successful");

		return 0;
	});
	socket.startServer("5866");

	// 接收命令 //
	for (std::string command_in; std::getline(std::cin, command_in);)
	{
		try
		{
			if (command_in == "start")
			{
				cs.start();
			}
			else if (command_in == "stop")
			{
				cs.stop();
			}
			else
			{
				auto id = cs.executeCmd(aris::core::Msg(command_in));
				std::cout << "command id:" << id << std::endl;
			}
		}
		catch (std::exception &e)
		{
			std::cout << e.what() << std::endl;
		}
	}


	cs.stop();
	socket.stop();


}


void test_control_server()
{
	//test_xml();
	//test_construct();

	test_server_option();
	test_server_ur();
}

