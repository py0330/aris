#include <iostream>
#include <aris.hpp>

// 示例轨迹规划 //
class MyPlan :public aris::plan::Plan
{
public:
	auto virtual prepairNrt(const std::map<std::string, std::string> &params, aris::plan::PlanTarget &target)->void override
	{
	}
	auto virtual executeRT(aris::plan::PlanTarget &target)->int override
	{
		return 0;
	}
	auto virtual collectNrt(aris::plan::PlanTarget &target)->void override 
	{
	}

	explicit MyPlan(const std::string &name = "my_plan")
	{
		command().loadXmlStr(
			"<Command name=\"my_plan\">"
			"</Command>"
		);
	}
	ARIS_REGISTER_TYPE(MyPlan);
};

int main(int argc, char *argv[])
{
	// 从xml中读取机器人，仅包含基本的轨迹规划 //
	auto&cs = aris::server::ControlServer::instance();
	cs.loadXmlFile(ARIS_INSTALL_PATH + std::string("/resource/demo_rokae_server/rokae.xml"));

	// 添加自己的轨迹规划 //
	cs.planRoot().planPool().add<MyPlan>();

	// 启动 server //
	cs.start();

	// web socket 及其处理 //
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
	socket.startServer();
	
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
	
	// 命令行 //
	for (std::string command_in; std::getline(std::cin, command_in);)
	{
		try
		{
			auto target = cs.executeCmd(aris::core::Msg(command_in));
		}
		catch (std::exception &e)
		{
			std::cout << e.what() << std::endl;
			LOG_ERROR << e.what() << std::endl;
		}
	}
	
	return 0;
}