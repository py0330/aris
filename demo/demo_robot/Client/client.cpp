#include <aris.h>

//int sendRequest(int argc, char *argv[], const char *xmlFileName)
int sendRequest(int argc, char *argv[])
{
	// 需要去除命令名的路径和扩展名 //
	std::string cmdName(argv[0]);

#ifdef WIN32
	if (cmdName.rfind('\\'))
	{
		cmdName = cmdName.substr(cmdName.rfind('\\') + 1, cmdName.npos);
	}
#endif
#ifdef UNIX
	if (cmdName.rfind('/'))
	{
		cmdName = cmdName.substr(cmdName.rfind('/') + 1, cmdName.npos);
	}
#endif

	if (cmdName.rfind('.'))
	{
		cmdName = cmdName.substr(0, cmdName.rfind('.'));
	}

	// 添加命令的所有参数 //
	for (int i = 1; i < argc; ++i)
	{
		cmdName = cmdName + " " + argv[i];
	}

	// 构造msg，这里需要先copy命令名称，然后依次copy各个参数 //
	aris::core::Msg msg;
	msg.copy(cmdName.c_str());

    std::cout<<"test\t"<<cmdName.c_str()<<std::endl;

	// 连接并发送msg //
    aris::core::Root root;
    root.registerChildType<aris::core::Socket>();
    auto& client = root.add<aris::core::Socket>("client");
    /*
	aris::core::XmlDocument doc;

	if (doc.LoadFile(xmlFileName) != 0)	throw std::logic_error("failed to read configuration xml file");

    std::string ip = doc.RootElement()->FirstChildElement("Server")->Attribute("ip");
    std::string port = doc.RootElement()->FirstChildElement("Server")->Attribute("port");
    */

    client.setRemoteIP("127.0.0.1");
    client.setPort("5866");

	while (true)
	{
		try
		{
            client.connect();
			break;
		}
		catch (std::exception &)
		{
			std::cout << "failed to connect server, will retry in 1 second" << std::endl;
			aris::core::msSleep(1000);
		}

	}

    std::cout<<"MSG\t"<<msg.data()<<std::endl;
    aris::core::Msg ret = client.sendRequest(msg);

	/*错误处理*/
	if (ret.size() > 0)
	{
		std::cout << "cmd has fault, please regard to following information:" << std::endl;
		std::cout << "    " << ret.data() << std::endl;
	}
	else
	{
		std::cout << "send command successful" << std::endl;
	}

	return 0;
}

int main(int argc, char *argv[])
{
	if (argc <= 1)throw std::runtime_error("please input the cmd name");

/*
#ifdef UNIX
    sendRequest(argc - 1, argv + 1, "/usr/aris/robot/resource/robot.xml");
#endif
#ifdef WIN32
    sendRequest(argc - 1, argv + 1, "C:\\aris\\robot\\resource\\robot.xml");
#endif
*/
    sendRequest(argc-1, argv+1);

	return 0;
}
