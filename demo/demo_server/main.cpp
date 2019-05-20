#include <iostream>
#include <aris.hpp>

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
	std::string robot_name = argc < 2 ? "rokae_xb4" : argv[1];
	auto port = argc < 3 ? 5866 : std::stoi(argv[2]);
	aris::dynamic::s_pq2pm(argc < 4 ? nullptr : aris::core::Calculator().calculateExpression(argv[3]).data(), robot_pm);

	auto&cs = aris::server::ControlServer::instance();
	cs.setName(robot_name);
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
		cs.interfaceRoot().loadXmlStr(aris::robot::createRokaeXB4Interface());
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
		cs.interfaceRoot().loadXmlStr(aris::robot::createRokaeXB4Interface());

		// init model pos //
		//cs.model().generalMotionPool()[0].setMpe(std::array<double, 6>{0, 0, 0.5, 0, 0, 0}.data(), "313");
		//cs.model().solverPool()[0].kinPos();

		//cs.saveXmlFile("C:\\Users\\py033\\Desktop\\stewart.xml");
		
		//cs.loadXmlFile(ARIS_INSTALL_PATH + std::string("/resource/demo_server/stewart.xml"));
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

	//std::cout << m1.slavePool().ancestor<aris::control::Master>() << std::endl;
	//std::cout << check_master_pdos.slavePool().ancestor<aris::control::Master>() << std::endl;



	////////////////////////////////////////////////////////////////////////////////////
	//aris::dynamic::SevenAxisParam param;

	//param.d1 = 0.3705;
	//param.d3 = 0.330;
	//param.d5 = 0.320;
	//param.tool0_pe[2] = 0.2205;

	//auto m = aris::dynamic::createModelSevenAxis(param);
	//cs.resetModel(m.release());
	//dynamic_cast<aris::control::EthercatMotion&>(cs.controller().slaveAtAbs(1)).setMinPos(-0.1);
	//dynamic_cast<aris::control::EthercatMotion&>(cs.controller().slaveAtAbs(1)).setMaxPos(0.1);
	////////////////////////////////////////////////////////////////////////////////////

	// make log file has enough space
	cs.planRoot().planPool().add<aris::plan::RemoveFile>("remove_file");
	cs.start();
	try
	{
		cs.executeCmd(aris::core::Msg("rmFi --filePath=/home/kaanh/log --memo=20000"));
	}
	catch (std::exception &e)
	{
		std::cout << e.what() << std::endl;
	}
	

	// interaction //
	std::list<std::tuple<aris::core::Msg, std::shared_ptr<aris::plan::PlanTarget>>> result_list;
	std::mutex result_mutex;
	
	aris::core::Socket socket("server", "", "5866", aris::core::Socket::WEB);
	socket.setOnReceivedMsg([&](aris::core::Socket *socket, aris::core::Msg &msg)->int
	{
		std::string msg_data = msg.toString();

		static int cout_count = 0;
		if(++cout_count%10 == 0)
			std::cout << "recv:" << msg_data << std::endl;

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
			std::stringstream ss(msg_data);
			for (std::string cmd; std::getline(ss, cmd);)
			{
				auto result = cs.executeCmd(aris::core::Msg(cmd));

				std::unique_lock<std::mutex> l(result_mutex);
				result_list.push_back(std::make_tuple(msg, result));
			}
		}
		catch (std::exception &e)
		{
			std::cout << e.what() << std::endl;
			LOG_ERROR << e.what() << std::endl;

			try
			{
				aris::core::Msg m;
				m.setMsgID(msg.header().msg_id_);
				m.setType(msg.header().msg_type_);
				m.header().reserved1_ = msg.header().reserved1_;
				m.header().reserved2_ = msg.header().reserved2_;
				m.header().reserved3_ = msg.header().reserved3_;
				socket->sendMsg(m);
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
	
	std::thread result_thread([&]()
	{
		while (true)
		{
			std::unique_lock<std::mutex> lck(result_mutex);
			for (auto result = result_list.begin(); result != result_list.end();)
			{
				auto cmd_ret = std::get<1>(*result);
				if (cmd_ret->finished.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready)
				{
					auto ret = cmd_ret->ret;
					auto &msg = std::get<0>(*result);

					if (auto str = std::any_cast<std::string>(&ret))
					{
						try
						{
							aris::core::Msg ret_msg(*str);
							
							ret_msg.setMsgID(msg.header().msg_id_);
							ret_msg.setType(msg.header().msg_type_);
							ret_msg.header().reserved1_ = msg.header().reserved1_;
							ret_msg.header().reserved2_ = msg.header().reserved2_;
							ret_msg.header().reserved3_ = msg.header().reserved3_;
							socket.sendMsg(ret_msg);
						}
						catch (std::exception &e)
						{
							std::cout << e.what() << std::endl;
							LOG_ERROR << e.what() << std::endl;
						}
					}
					else
					{
						try
						{
							aris::core::Msg ret_msg;
							ret_msg.setMsgID(msg.header().msg_id_);
							ret_msg.setType(msg.header().msg_type_);
							ret_msg.header().reserved1_ = msg.header().reserved1_;
							ret_msg.header().reserved2_ = msg.header().reserved2_;
							ret_msg.header().reserved3_ = msg.header().reserved3_;
							socket.sendMsg(ret_msg);
						}
						catch (std::exception &e)
						{
							std::cout << e.what() << std::endl;
							LOG_ERROR << e.what() << std::endl;
						}
					}
				}

				result_list.erase(result++);
			}
			lck.unlock();

			std::this_thread::sleep_for(std::chrono::milliseconds(1));
		}
	});
	
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

			// xy 客户和simtool不一样 //
			std::swap(mat.data()[0], mat.data()[1]);
			mat.data()[0] = -mat.data()[0];

			std::swap(mat.data()[3], mat.data()[4]);
			mat.data()[3] = -mat.data()[3];

			mat.data()[0] *= 0.035;
			mat.data()[1] *= 0.035;
			mat.data()[2] *= 0.037;
			mat.data()[3] *= 0.08;
			mat.data()[4] *= 0.08;
			mat.data()[5] *= 0.04;

			// 向上的轴加1.0，为默认位置 //
			mat.data()[2] += 0.513;
			mat.data()[1] -= 0.0103;

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
			auto target = cs.executeCmd(aris::core::Msg(command_in));
			//target->finished.get();
			//if (auto str = std::any_cast<std::string>(&target->ret))
			//{
			//	std::cout << *str << std::endl;
			//}
		}
		catch (std::exception &e)
		{
			std::cout << e.what() << std::endl;
			LOG_ERROR << e.what() << std::endl;
		}
	}
	
	return 0;
}