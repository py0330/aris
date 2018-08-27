#include <iostream>
#include <aris.h>

using namespace aris::dynamic;
using namespace aris::robot;

int main()
{
	auto&cs = aris::server::ControlServer::instance();

	cs.resetController(aris::robot::createRokaeController().release());
	cs.resetModel(aris::robot::createRokaeModel().release());
	cs.resetSensorRoot(new aris::sensor::SensorRoot);
	cs.resetPlanRoot(new aris::plan::PlanRoot);

	dynamic_cast<aris::dynamic::PumaInverseKinematicSolver&>(cs.model().solverPool().at(0)).setWhichRoot(4);
	
	cs.planRoot().planPool().add<aris::plan::EnablePlan>();
	cs.planRoot().planPool().add<aris::plan::RecoverPlan>().command().loadXmlStr(
		"<rc default_child_type=\"Param\">"
		"	<group type=\"GroupParam\" default_child_type=\"Param\">"
		"		<position_unit default=\"m\"/>"
		"		<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"group\">"
		"			<pq default=\"{0.32,0,0.6295,0,0,0,1}\"/>"
		"			<pm default=\"{1,0,0,0,0,1,0,0.63,0,0,1,0.316,0,0,0,1}\"/>"
		"			<group type=\"GroupParam\" default_child_type=\"Param\">"
		"				<pe default=\"{0.32,0,0.6295,0,1.57,0}\"/>"
		"				<orientation_unit default=\"rad\"/>"
		"				<euler_type default=\"321\"/>"
		"			</group>"
		"		</unique>"
		"		<acceleration default=\"0.2\"/>"
		"		<velocity default=\"0.2\"/>"
		"		<deceleration default=\"0.2\"/>"
		"		<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_all\">"
		"			<check_all/>"
		"			<check_none/>"
		"			<group type=\"GroupParam\" default_child_type=\"Param\">"
		"				<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_pos\">"
		"					<check_pos/>"
		"					<not_check_pos/>"
		"					<group type=\"GroupParam\" default_child_type=\"Param\">"
		"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_pos_max\">"
		"							<check_pos_max/>"
		"							<not_check_pos_max/>"
		"						</unique>"
		"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_pos_min\">"
		"							<check_pos_min/>"
		"							<not_check_pos_min/>"
		"						</unique>"
		"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_pos_continuous\">"
		"							<check_pos_continuous/>"
		"							<not_check_pos_continuous/>"
		"						</unique>"
		"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_pos_continuous_at_start\">"
		"							<check_pos_continuous_at_start/>"
		"							<not_check_pos_continuous_at_start/>"
		"						</unique>"
		"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_pos_continuous_second_order\">"
		"							<check_pos_continuous_second_order/>"
		"							<not_check_pos_continuous_second_order/>"
		"						</unique>"
		"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_pos_continuous_second_order_at_start\">"
		"							<check_pos_continuous_second_order_at_start/>"
		"							<not_check_pos_continuous_second_order_at_start/>"
		"						</unique>"
		"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_pos_following_error\">"
		"							<check_pos_following_error/>"
		"							<not_check_pos_following_error />"
		"						</unique>"
		"					</group>"
		"				</unique>"
		"				<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_vel\">"
		"					<check_vel/>"
		"					<not_check_vel/>"
		"					<group type=\"GroupParam\" default_child_type=\"Param\">"
		"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_vel_max\">"
		"							<check_vel_max/>"
		"							<not_check_vel_max/>"
		"						</unique>"
		"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_vel_min\">"
		"							<check_vel_min/>"
		"							<not_check_vel_min/>"
		"						</unique>"
		"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_vel_continuous\">"
		"							<check_vel_continuous/>"
		"							<not_check_vel_continuous/>"
		"						</unique>"
		"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_vel_continuous_at_start\">"
		"							<check_vel_continuous_at_start/>"
		"							<not_check_vel_continuous_at_start/>"
		"						</unique>"
		"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_vel_following_error\">"
		"							<check_vel_following_error/>"
		"							<not_check_vel_following_error />"
		"						</unique>"
		"					</group>"
		"				</unique>"
		"			</group>"
		"		</unique>"
		"	</group>"
		"</rc>");


	cs.planRoot().planPool().add<aris::plan::MovePlan>();
	cs.planRoot().planPool().add<aris::plan::MoveJ>();

	aris::core::Socket socket;
	socket.setOnReceivedMsg([&](aris::core::Socket *, aris::core::Msg &msg)->int
	{
		std::string msg_data = msg.toString();

		LOG_INFO << "socket receive normal msg:\n"
			<< msg.header().msg_size_ << "&"
			<< msg.header().msg_id_ << "&"
			<< msg.header().msg_type_ << "&"
			<< msg.header().reserved1_ << "&"
			<< msg.header().reserved2_ << "&"
			<< msg.header().reserved3_ << ":"
			<< msg_data << std::endl;

		return 0;
	});
	socket.setOnReceivedRequest([&](aris::core::Socket *, aris::core::Msg &msg)
	{
		std::string msg_data = msg.toString();

		LOG_INFO_EVERY_N(10) << "socket receive request msg:" 
			<< msg.header().msg_size_ << "&" 
			<< msg.header().msg_id_	<< "&" 
			<< msg.header().msg_type_ << "&" 
			<< msg.header().reserved1_ << "&" 
			<< msg.header().reserved2_ << "&" 
			<< msg.header().reserved3_ << ":" 
			<< msg_data << std::endl;

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
				LOG_ERROR << e.what() << std::endl;
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

			return aris::core::Msg(mat.toString());
		}

		return aris::core::Msg("unknown msg id");
	});
	socket.setOnReceivedConnection([](aris::core::Socket *sock, const char *ip, int port)->int
	{
		LOG_INFO << "socket receive connection:\n"
			<< std::setw(aris::core::LOG_SPACE_WIDTH) << "|" << "  ip:" << ip << "\n"
			<< std::setw(aris::core::LOG_SPACE_WIDTH) << "|" << "port:" << port << std::endl;
		return 0;
	});
	socket.setOnLoseConnection([](aris::core::Socket *socket)
	{
		LOG_INFO << "socket lose connection" << std::endl;
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
		LOG_INFO << "socket restart successful" << std::endl;

		return 0;
	});

	// 接收命令 //
	for (std::string command_in; std::getline(std::cin, command_in);)
	{
		try
		{
			if (command_in == "start")
			{
				cs.start();
				socket.startServer("5866");
			}
			else if (command_in == "stop")
			{
				cs.stop();
				socket.stop();
			}
			else
			{
				auto id = cs.executeCmd(aris::core::Msg(command_in));
				std::cout << "command id:" << id << std::endl;
			}
		}
		catch (std::exception &e)
		{
			LOG_ERROR << e.what() << std::endl;
		}
	}

	return 0;
}

