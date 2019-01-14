#include <iostream>
#include <aris.h>

using namespace aris::dynamic;
using namespace aris::robot;

auto inline outputCsByPq(const aris::server::ControlServer &cs, std::string file_path)->void
{
	aris::core::XmlDocument doc;
	
	cs.saveXmlDoc(doc);

	auto part_pool = doc.FirstChildElement()->FirstChildElement("model")->FirstChildElement("part_pool");

	for (auto prt = part_pool->FirstChildElement(); prt; prt = prt->NextSiblingElement())
	{
		aris::core::Calculator c;
		auto mat = c.calculateExpression(prt->Attribute("pe"));
		prt->DeleteAttribute("pe");

		double pq[7];
		s_pe2pq(mat.data(), pq);
		prt->SetAttribute("pq", aris::core::Matrix(1, 7, pq).toString().c_str());

		for (auto geo = prt->FirstChildElement("geometry_pool")->FirstChildElement(); geo; geo = geo->NextSiblingElement())
		{
			auto mat = c.calculateExpression(geo->Attribute("pe"));
			geo->DeleteAttribute("pe");

			double pq[7];
			s_pe2pq(mat.data(), pq);
			geo->SetAttribute("pq", aris::core::Matrix(1, 7, pq).toString().c_str());
		}
	}


	doc.SaveFile(file_path.c_str());
}

int main(int argc, char *argv[])
{
	double robot_pm[16];
	std::string robot_name = argc < 2 ? "stewart" : argv[1];
	auto port = argc < 3 ? 5866 : std::stoi(argv[2]);
	aris::dynamic::s_pq2pm(argc < 4 ? nullptr : aris::core::Calculator().calculateExpression(argv[3]).data(), robot_pm);

	auto&cs = aris::server::ControlServer::instance();

	if (robot_name == "ur5")
	{
		cs.resetController(createControllerUr5().release());
		cs.resetModel(createModelUr5(robot_pm).release());
		cs.resetPlanRoot(createPlanRootUr5().release());
		cs.resetSensorRoot(new aris::sensor::SensorRoot);
	}
	else if (robot_name == "rokae_xb4")
	{
		cs.resetController(createControllerRokaeXB4().release());
		cs.resetModel(aris::robot::createModelRokaeXB4(robot_pm).release());
		cs.resetPlanRoot(createPlanRootRokaeXB4().release());
		cs.resetSensorRoot(new aris::sensor::SensorRoot);
	}
	else if (robot_name == "servo_press")
	{
		cs.resetController(createControllerServoPress().release());
		cs.resetModel(aris::robot::createModelServoPress(robot_pm).release());
		cs.resetPlanRoot(createPlanRootServoPress().release());
		cs.resetSensorRoot(new aris::sensor::SensorRoot);
	}
	else if (robot_name == "stewart")
	{
		cs.resetController(createControllerStewart().release());
		cs.resetModel(aris::robot::createModelStewart(robot_pm).release());
		cs.resetPlanRoot(createPlanRootStewart().release());
		cs.resetSensorRoot(new aris::sensor::SensorRoot);
	}
	else
	{
		std::cout << "unknown robot:" << robot_name << std::endl;
		return -1;
	}
	std::cout << "this server robot   :" << robot_name << std::endl;
	std::cout << "this server port    :" << std::to_string(port) << std::endl;
	std::cout << "this server position:" << std::endl;
	dsp(4, 4, robot_pm);

	cs.model().generalMotionPool()[0].setMpe(std::array<double, 6>{0, 1, 0, 0, 0, 0}.data(), "313");
	cs.model().solverPool()[0].kinPos();

	cs.saveXmlFile("C:\\Users\\py033\\Desktop\\test_pe.xml");
	outputCsByPq(cs, "C:\\Users\\py033\\Desktop\\test.xml");
	
	cs.start();

	//cs.executeCmd(aris::core::Msg("am --start"));

	aris::core::Socket socket("server", "", "5866", aris::core::Socket::WEB);
	socket.setOnReceivedMsg([&](aris::core::Socket *socket, aris::core::Msg &msg)->int
	{
		std::string msg_data = msg.toString();

		std::cout << "recv:" << msg_data << std::endl;

		if (msg.header().msg_id_ == 0)
		{
			LOG_INFO << "the request is cmd:"
				<< msg.header().msg_size_ << "&"
				<< msg.header().msg_id_ << "&"
				<< msg.header().msg_type_ << "&"
				<< msg.header().reserved1_ << "&"
				<< msg.header().reserved2_ << "&"
				<< msg.header().reserved3_ << ":"
				<< msg_data << std::endl;

			try
			{
				try
				{
					auto id = cs.executeCmd(aris::core::Msg(msg_data));
					std::cout << "command id:" << id << std::endl;
				}
				catch (std::exception &e)
				{
					std::cout << e.what() << std::endl;
					LOG_ERROR << e.what() << std::endl;
					socket->sendMsg(aris::core::Msg());
				}
			}
			catch (std::exception &e)
			{
				std::cout << e.what() << std::endl;
				LOG_ERROR << e.what() << std::endl;
			}
		}
		else if (msg.header().msg_id_ == 1)
		{
			LOG_INFO_EVERY_N(10) << "socket receive request msg:"
				<< msg.header().msg_size_ << "&"
				<< msg.header().msg_id_ << "&"
				<< msg.header().msg_type_ << "&"
				<< msg.header().reserved1_ << "&"
				<< msg.header().reserved2_ << "&"
				<< msg.header().reserved3_ << ":"
				<< msg_data << std::endl;

			auto part_pm_vec = std::make_any<std::vector<double> >(cs.model().partPool().size() * 16);
			cs.getRtData([](aris::server::ControlServer& cs, std::any& data)
			{
				for (aris::Size i(-1); ++i < cs.model().partPool().size();)
					cs.model().partPool().at(i).getPm(std::any_cast<std::vector<double>& >(data).data() + i * 16);
			}, part_pm_vec);

			std::vector<double> part_pq(cs.model().partPool().size() * 7);
			for (aris::Size i(-1); ++i < cs.model().partPool().size();)
			{
				aris::dynamic::s_pm2pq(std::any_cast<std::vector<double>& >(part_pm_vec).data() + i * 16, part_pq.data() + i * 7);
			}

			//// return binary ////
			aris::core::Msg msg;
			msg.copy(part_pq.data(), static_cast<aris::core::MsgSize>(part_pq.size() * 8));

			try
			{
				socket->sendMsg(msg);
			}
			catch (std::exception &e)
			{
				std::cout << e.what() << std::endl;
				LOG_ERROR << e.what() << std::endl;
			}
		}

		return 0;
	});
	socket.setOnReceivedConnection([](aris::core::Socket *sock, const char *ip, int port)->int
	{
		std::cout << "socket receive connection" << std::endl;
		LOG_INFO << "socket receive connection:\n"
			<< std::setw(aris::core::LOG_SPACE_WIDTH) << "|" << "  ip:" << ip << "\n"
			<< std::setw(aris::core::LOG_SPACE_WIDTH) << "|" << "port:" << port << std::endl;
		return 0;
	});
	socket.setOnLoseConnection([](aris::core::Socket *socket)
	{
		std::cout << "socket lose connection" << std::endl;
		LOG_INFO << "socket lose connection" << std::endl;
		for (;;)
		{
			try
			{
				socket->startServer("5866");
				break;
			}
			catch (std::runtime_error &e)
			{
				std::cout << e.what() << std::endl << "will try to restart server socket in 1s" << std::endl;
				LOG_ERROR << e.what() << std::endl << "will try to restart server socket in 1s" << std::endl;
				std::this_thread::sleep_for(std::chrono::seconds(1));
			}
		}
		std::cout << "socket restart successful" << std::endl;
		LOG_INFO << "socket restart successful" << std::endl;

		return 0;
	});
	socket.startServer(std::to_string(port));

	aris::core::Socket udp_socket("server", "", "5867", aris::core::Socket::UDP_RAW);
	udp_socket.setOnReceivedRawData([&](aris::core::Socket *socket, const char *data, int size)->int
	{
		try
		{
			std::string msg_data(data, size);

			aris::core::Calculator c;

			auto mat = c.calculateExpression(msg_data);

			double value[6]{ 2147483647, 2147483647, 2147483647, 2147483647, 2147483647, 2147483647 };
			s_vs(6, value, mat.data());
			s_nv(6, 1.0 / 2147483647.0, mat.data());

			auto cmd = "am --pe=" + mat.toString();
			
			static int i = 0;
			if (++i % 100 == 0)
			{
				std::cout << cmd << std::endl;
			}


			cs.executeCmd(aris::core::Msg(cmd));
		}
		catch (std::runtime_error &e)
		{
			std::cout << e.what() << std::endl;
		}


		//std::cout << "recv:" << mat.toString() << std::endl;
		





		return 0;
	});
	udp_socket.setOnReceivedConnection([](aris::core::Socket *sock, const char *ip, int port)->int
	{
		std::cout << "socket receive connection" << std::endl;
		LOG_INFO << "socket receive connection:\n"
			<< std::setw(aris::core::LOG_SPACE_WIDTH) << "|" << "  ip:" << ip << "\n"
			<< std::setw(aris::core::LOG_SPACE_WIDTH) << "|" << "port:" << port << std::endl;
		return 0;
	});
	udp_socket.setOnLoseConnection([](aris::core::Socket *socket)
	{
		std::cout << "socket lose connection" << std::endl;
		LOG_INFO << "socket lose connection" << std::endl;
		for (;;)
		{
			try
			{
				socket->startServer("5866");
				break;
			}
			catch (std::runtime_error &e)
			{
				std::cout << e.what() << std::endl << "will try to restart server socket in 1s" << std::endl;
				LOG_ERROR << e.what() << std::endl << "will try to restart server socket in 1s" << std::endl;
				std::this_thread::sleep_for(std::chrono::seconds(1));
			}
		}
		std::cout << "socket restart successful" << std::endl;
		LOG_INFO << "socket restart successful" << std::endl;

		return 0;
	});
	udp_socket.startServer();

	// 接收命令 //
	for (std::string command_in; std::getline(std::cin, command_in);)
	{
		try
		{
			if (command_in == "start")
			{
				cs.start();
				socket.startServer(std::to_string(port));
			}
			else if (command_in == "stop")
			{
				cs.stop();
				socket.stop();
			}
			else
			{
				auto id = cs.executeCmd(aris::core::Msg(command_in));
				//std::cout << "command id:" << id << std::endl;
			}
		}
		catch (std::exception &e)
		{
			std::cout << e.what() << std::endl;
			LOG_ERROR << e.what() << std::endl;
		}
	}

	return 0;
}