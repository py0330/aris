#ifdef WIN32
#define rt_printf printf
#include <windows.h>
#undef CM_NONE
#endif
#ifdef UNIX
#include "rtdk.h"
#include "unistd.h"
#endif


#include <cstring>
#include <thread>

#include <aris_core.h>
#include <aris_socket.h>
#include <aris_exp_cal.h>
#include <aris_plan.h>
#include <aris_motion.h>
#include "aris_control_server.h"

namespace Aris
{
	class Node
	{
	public:
		Node* AddChildGroup(const char *Name);
		Node* AddChildUnique(const char *Name);
		Node* AddChildParam(const char *Name);
		Node* FindChild(const char *Name)
		{
			auto result = std::find_if(children.begin(), children.end(), [Name](std::unique_ptr<Node> &node)
			{
				return (!std::strcmp(node->name.c_str(), Name));
			});

			if (result != children.end())
			{
				return result->get();
			}
			else
			{
				return nullptr;
			}
		};

		bool IsTaken() { return isTaken; };
		void Take();
		void Reset()
		{
			this->isTaken = false;
			for (auto &child : children)
			{
				child->Reset();
			}
		};

	public:
		Node(Node*father, const char *Name) :name(Name) { this->father = father; };
		virtual ~Node() {};

	private:
		std::string name;
		Node* father;
		std::vector<std::unique_ptr<Node> > children;

		bool isTaken{ false };

		friend void AddAllParams(const Aris::Core::XmlElement *pEle, Node *pNode, std::map<std::string, Node *> &allParams, std::map<char, std::string>& shortNames);
		friend void AddAllDefault(Node *pNode, std::map<std::string, std::string> &params);
	};
	class RootNode :public Node
	{
	public:
		RootNode(const char *Name) :Node(nullptr, Name) {};

	private:
		Node *pDefault;

		friend void AddAllParams(const Aris::Core::XmlElement *pEle, Node *pNode, std::map<std::string, Node *> &allParams, std::map<char, std::string>& shortNames);
		friend void AddAllDefault(Node *pNode, std::map<std::string, std::string> &params);
	};
	class GroupNode :public Node
	{
	public:
		GroupNode(Node*father, const char *Name) :Node(father, Name) {};
	};
	class UniqueNode :public Node
	{
	public:
		UniqueNode(Node*father, const char *Name) :Node(father, Name) {};

	private:
		Node *pDefault;

		friend void AddAllParams(const Aris::Core::XmlElement *pEle, Node *pNode, std::map<std::string, Node *> &allParams, std::map<char, std::string>& shortNames);
		friend void AddAllDefault(Node *pNode, std::map<std::string, std::string> &params);
	};
	class ParamNode :public Node
	{
	public:
		ParamNode(Node*father, const char *Name) :Node(father, Name) {};
	private:
		std::string type;
		std::string defaultValue;
		std::string minValue, maxValue;

		friend void AddAllParams(const Aris::Core::XmlElement *pEle, Node *pNode, std::map<std::string, Node *> &allParams, std::map<char, std::string>& shortNames);
		friend void AddAllDefault(Node *pNode, std::map<std::string, std::string> &params);
	};

	Node* Node::AddChildGroup(const char *Name)
	{
		this->children.push_back(std::unique_ptr<Node>(new GroupNode(this, Name)));
		return children.back().get();
	};
	Node* Node::AddChildUnique(const char *Name)
	{
		this->children.push_back(std::unique_ptr<Node>(new UniqueNode(this, Name)));
		return children.back().get();
	}
	Node* Node::AddChildParam(const char *Name)
	{
		this->children.push_back(std::unique_ptr<Node>(new ParamNode(this, Name)));
		return children.back().get();
	}
	void Node::Take()
	{
		if (dynamic_cast<RootNode*>(this))
		{
			if (this->isTaken)
			{
				throw std::logic_error(std::string("Param ") + this->name + " has been inputed twice");
			}
			else
			{
				this->isTaken = true;
				return;
			}
		}
		else if (dynamic_cast<GroupNode*>(this))
		{
			if (this->isTaken)
			{
				return;
			}
			else
			{
				this->isTaken = true;
				father->Take();
			}
		}
		else
		{
			if (this->isTaken)
			{
				throw std::logic_error(std::string("Param ") + this->name + " has been inputed twice");
			}
			else
			{
				this->isTaken = true;
				father->Take();
			}
		}
	}

	void AddAllParams(const Aris::Core::XmlElement *pEle, Node *pNode, std::map<std::string, Node *> &allParams, std::map<char, std::string>& shortNames)
	{
		/*add all children*/
		for (auto pChild = pEle->FirstChildElement();pChild != nullptr;	pChild = pChild->NextSiblingElement())
		{
			/*check if children already has this value*/
			if (pNode->FindChild(pChild->Name()))
			{
				throw std::logic_error(std::string("XML file has error: node \"") + pChild->Name() + "\" already exist");
			}

			/*set all children*/
			if (pChild->Attribute("type", "group"))
			{
				AddAllParams(pChild, pNode->AddChildGroup(pChild->Name()), allParams, shortNames);
			}
			else if (pChild->Attribute("type", "unique"))
			{
				AddAllParams(pChild, pNode->AddChildUnique(pChild->Name()), allParams, shortNames);
			}
			else
			{
				/*now the pChild is a param_node*/
				Node * insertNode;

				if (allParams.find(std::string(pChild->Name())) != allParams.end())
				{
					throw std::logic_error(std::string("XML file has error: node \"") + pChild->Name() + "\" already exist");
				}
				else
				{
					insertNode = pNode->AddChildParam(pChild->Name());
					allParams.insert(std::pair<std::string, Node *>(std::string(pChild->Name()), insertNode));
				}

				/*set abbreviation*/
				if (pChild->Attribute("abbreviation"))
				{
					if (shortNames.find(*pChild->Attribute("abbreviation")) != shortNames.end())
					{
						throw std::logic_error(std::string("XML file has error: abbreviations \"")+ pChild->Attribute("abbreviation") + "\" already exist");
					}
					else
					{
						char abbr = *pChild->Attribute("abbreviation");
						shortNames.insert(std::pair<char, std::string>(abbr, std::string(pChild->Name())));
					}
				}

				/*set values*/
				if (pChild->Attribute("type"))
				{
					dynamic_cast<ParamNode*>(insertNode)->type = std::string(pChild->Attribute("type"));
				}
				else
				{
					dynamic_cast<ParamNode*>(insertNode)->type = "";
				}

				if (pChild->Attribute("default"))
				{
					dynamic_cast<ParamNode*>(insertNode)->defaultValue = std::string(pChild->Attribute("default"));
				}
				else
				{
					dynamic_cast<ParamNode*>(insertNode)->defaultValue = "";
				}

				if (pChild->Attribute("maxValue"))
				{
					dynamic_cast<ParamNode*>(insertNode)->maxValue = std::string(pChild->Attribute("maxValue"));
				}
				else
				{
					dynamic_cast<ParamNode*>(insertNode)->maxValue = "";
				}

				if (pChild->Attribute("minValue"))
				{
					dynamic_cast<ParamNode*>(insertNode)->minValue = std::string(pChild->Attribute("minValue"));
				}
				else
				{
					dynamic_cast<ParamNode*>(insertNode)->minValue = "";
				}
			}
		}

		/*set all values*/
		if (dynamic_cast<RootNode*>(pNode))
		{
			if (pEle->Attribute("default"))
			{
				if (pNode->FindChild(pEle->Attribute("default")))
				{
					dynamic_cast<RootNode*>(pNode)->pDefault = pNode->FindChild(pEle->Attribute("default"));
				}
				else
				{
					throw std::logic_error(std::string("XML file has error: \"") + pNode->name + "\" can't find default param");
				}
			}
			else
			{
				dynamic_cast<RootNode*>(pNode)->pDefault = nullptr;
			}
		}

		if (dynamic_cast<UniqueNode*>(pNode))
		{
			if (pEle->Attribute("default"))
			{
				if (pNode->FindChild(pEle->Attribute("default")))
				{
					dynamic_cast<UniqueNode*>(pNode)->pDefault = pNode->FindChild(pEle->Attribute("default"));
				}
				else
				{
					throw std::logic_error(std::string("XML file has error: \"") + pNode->name + "\" can't find default param");
				}
			}
			else
			{
				if (pNode->children.empty())
				{
					throw std::logic_error(std::string("XML file has error: unique node \"") + pNode->name + "\" must have more than 1 child");
				}
				else
				{
					dynamic_cast<UniqueNode*>(pNode)->pDefault = nullptr;
				}
			}
		}
	}
	void AddAllDefault(Node *pNode, std::map<std::string, std::string> &params)
	{
		if (pNode->isTaken)
		{
			if (dynamic_cast<RootNode*>(pNode))
			{
				auto found = find_if(pNode->children.begin(), pNode->children.end(), [](std::unique_ptr<Node> &a)
				{
					return a->isTaken;
				});

				AddAllDefault(found->get(), params);
			}

			if (dynamic_cast<UniqueNode*>(pNode))
			{
				auto found = find_if(pNode->children.begin(), pNode->children.end(), [](std::unique_ptr<Node> &a)
				{
					return a->isTaken;
				});

				AddAllDefault(found->get(), params);
			}

			if (dynamic_cast<GroupNode*>(pNode))
			{
				for (auto &i : pNode->children)
					AddAllDefault(i.get(), params);
			}

			if (dynamic_cast<ParamNode*>(pNode))
			{
				if (params.at(pNode->name) == "")
				{
					params.at(pNode->name) = dynamic_cast<ParamNode*>(pNode)->defaultValue;
				}
				return;
			}
		}
		else
		{
			if (dynamic_cast<RootNode*>(pNode))
			{
				if (!pNode->children.empty())
				{
					if ((dynamic_cast<RootNode*>(pNode)->pDefault))
					{
						AddAllDefault(dynamic_cast<RootNode*>(pNode)->pDefault, params);
					}
					else
					{
						throw std::logic_error(std::string("cmd \"") + pNode->name + "\" has no default param");
					}
				}

				pNode->isTaken = true;
			}

			if (dynamic_cast<UniqueNode*>(pNode))
			{
				if (!pNode->children.empty())
				{
					if (dynamic_cast<UniqueNode*>(pNode)->pDefault)
					{
						AddAllDefault(dynamic_cast<UniqueNode*>(pNode)->pDefault, params);
					}
					else
					{
						throw std::logic_error(std::string("param \"") + pNode->name + "\" has no default sub-param");
					}
				}

				pNode->isTaken = true;
			}

			if (dynamic_cast<GroupNode*>(pNode))
			{
				for (auto &i : pNode->children)
				{
					AddAllDefault(i.get(), params);
				}
					

				pNode->isTaken = true;
			}

			if (dynamic_cast<ParamNode*>(pNode))
			{
				params.insert(make_pair(pNode->name, dynamic_cast<ParamNode*>(pNode)->defaultValue));
				pNode->isTaken = true;
			}
		}
	}

	struct CommandStruct
	{
		std::unique_ptr<RootNode> root;
		std::map<std::string, Node *> allParams{};
		std::map<char, std::string> shortNames{};

		CommandStruct(const std::string &name) 
			:root(new RootNode(name.c_str()))
		{
		}
	};
	
	class ControlServer::Imp
	{
	public:
		void LoadXml(const Aris::Core::XmlDocument &doc);
		void AddGait(std::string cmdName, DynKer::PlanFunc gaitFunc, ParseFunc parseFunc);
		void Start();
		void Stop();

		Imp(ControlServer *pServer) 
		{ 
			this->pServer = pServer;
#ifdef UNIX
			this->pController = Aris::Control::EthercatController::CreateMaster<Aris::Control::EthercatController>();
#endif
		};
	private:
		Imp(const Imp&) = delete;

		void DecodeMsg2Param(const Aris::Core::Msg &msg, std::string &cmd, std::map<std::string, std::string> &params);
		void SendParam2RT(const std::string &cmd, const std::map<std::string, std::string> &params);
		void OnReceiveMsg(const Aris::Core::Msg &m, Aris::Core::Msg &retError);

		int home(const BasicFunctionParam &param, Aris::Control::EthercatController::Data &data);
		int enable(const BasicFunctionParam &param, Aris::Control::EthercatController::Data &data);
		int disable(const BasicFunctionParam &param, Aris::Control::EthercatController::Data &data);
		int run(GaitParamBase &param, Aris::Control::EthercatController::Data &data);

		int execute_cmd(int count, char *cmd, Aris::Control::EthercatController::Data &data);
		static int tg(Aris::Control::EthercatController::Data &data);

	private:
		enum RobotCmdID
		{
			ENABLE,
			DISABLE,
			HOME,
			RUN_GAIT,

			ROBOT_CMD_COUNT
		};

	private:
		ControlServer *pServer;
		std::map<std::string, int> mapName2ID;//store gait id in follow vector
		std::vector<DynKer::PlanFunc> allGaits;
		std::vector<ParseFunc> allParsers;

		ParseFunc parse_enable_func_{ [this](const std::string &cmd, const std::map<std::string, std::string> &params, Aris::Core::Msg &msg) 
		{
			BasicFunctionParam param;
			param.cmd_type = Imp::RobotCmdID::ENABLE;
			std::fill_n(param.isMotorActive, this->pModel_->MotionNum(), true);
			msg.CopyStruct(param);
		} };
		ParseFunc parse_disable_func_{ [this](const std::string &cmd, const std::map<std::string, std::string> &params, Aris::Core::Msg &msg)
		{
			BasicFunctionParam param;
			param.cmd_type = Imp::RobotCmdID::DISABLE;
			std::fill_n(param.isMotorActive, this->pModel_->MotionNum(), true);
			msg.CopyStruct(param);
		} };
		ParseFunc parse_home_func_{ [this](const std::string &cmd, const std::map<std::string, std::string> &params, Aris::Core::Msg &msg)
		{
			BasicFunctionParam param;
			param.cmd_type = Imp::RobotCmdID::HOME;
			std::fill_n(param.isMotorActive, this->pModel_->MotionNum(), true);
			msg.CopyStruct(param);
		} };

		std::map<std::string, std::unique_ptr<CommandStruct> > mapCmd;//store Node of command

		Aris::Core::Socket server;
		std::string ip, port;

		Aris::Control::EthercatController *pController;
		std::unique_ptr<Aris::DynKer::Model> pModel_;

		std::unique_ptr<Aris::Sensor::IMU> pImu;
		friend class ControlServer;
	};

	void ControlServer::Imp::LoadXml(const Aris::Core::XmlDocument &doc)
	{
		/*load robot model*/
		pModel_->LoadXml(doc);
		
		/*begin to create imu*/
		if (doc.RootElement()->FirstChildElement("Server")->FirstChildElement("Sensors")->FirstChildElement("IMU")->Attribute("active", "true"))
		{
			std::cout<<"imu found"<<std::endl;			
			pImu.reset(new Aris::Sensor::IMU(doc.RootElement()->FirstChildElement("Server")->FirstChildElement("Sensors")->FirstChildElement("IMU")));
		}
		else
		{
			std::cout<<"imu not find"<<std::endl;
		}

		/*begin to load controller*/
#ifdef UNIX
		pController->LoadXml(std::ref(*doc.RootElement()->FirstChildElement("Controller")->FirstChildElement("EtherCat")));
		pController->SetControlStrategy(tg);
#endif

		/*load connection param*/
		auto pConnEle = doc.RootElement()->FirstChildElement("Server")->FirstChildElement("Connection");
		ip = pConnEle->Attribute("IP");
		port = pConnEle->Attribute("Port");

		/*begin to copy client and insert cmd nodes*/
		const int TASK_NAME_LEN = 1024;
		char path_char[TASK_NAME_LEN] = { 0 };
#ifdef WIN32
		GetModuleFileName(NULL, path_char, TASK_NAME_LEN);
		std::string path(path_char);
		path = path.substr(0, path.rfind('\\'));
#endif
#ifdef UNIX
		char cParam[100] = { 0 };
		sprintf(cParam, "/proc/%d/exe", getpid());
		auto count = readlink(cParam, path_char, TASK_NAME_LEN);
		std::string path(path_char);
		path = path.substr(0, path.rfind('/'));
#endif

		auto pCmds = doc.RootElement()->FirstChildElement("Server")->FirstChildElement("Commands");

		if (pCmds == nullptr) throw std::logic_error("invalid xml file, because it contains no commands information");

		mapCmd.clear();
		for (auto pChild = pCmds->FirstChildElement(); pChild != nullptr; pChild = pChild->NextSiblingElement())
		{
#ifdef WIN32
			std::string fullpath = std::string("copy ") + path + std::string("\\Client.exe ") + path + "\\" + pChild->Name() + ".exe";
#endif
#ifdef UNIX
			std::string fullpath = std::string("cp ") + path + std::string("/Client ") + path + "/" + pChild->Name();
#endif
			auto ret = system(fullpath.c_str());

			if (mapCmd.find(pChild->Name())!=mapCmd.end())
				throw std::logic_error(std::string("command ")+ pChild->Name() +" is already existed, please rename it");

			mapCmd.insert(std::make_pair(std::string(pChild->Name()), std::unique_ptr<CommandStruct>(new CommandStruct(pChild->Name()))));
			AddAllParams(pChild, mapCmd.at(pChild->Name())->root.get(), mapCmd.at(pChild->Name())->allParams, mapCmd.at(pChild->Name())->shortNames);
		}

		/*Set socket connection callback function*/
		server.SetOnReceivedConnection([](Aris::Core::Socket *pConn, const char *pRemoteIP, int remotePort)
		{
			Aris::Core::log(std::string("received connection, the ip is: ") + pRemoteIP);
			return 0;
		});
		server.SetOnReceiveRequest([this](Aris::Core::Socket *pConn, Aris::Core::Msg &msg)
		{
			Aris::Core::Msg ret;
			this->OnReceiveMsg(msg, ret);

			return ret;
		});
		server.SetOnLoseConnection([this](Aris::Core::Socket *pConn)
		{
			Aris::Core::log("lost connection");
			while (true)
			{
				try
				{
					pConn->StartServer(this->port.c_str());
					break;
				}
				catch (Aris::Core::Socket::StartServerError &e)
				{
					std::cout << e.what() << std::endl << "will try to restart in 1s" << std::endl;
					Aris::Core::Sleep(1000);
				}
			}

			return 0;
		});
	}
	void ControlServer::Imp::AddGait(std::string cmdName, DynKer::PlanFunc gaitFunc, ParseFunc parseFunc)
	{
		if (mapName2ID.find(cmdName) != mapName2ID.end())
		{
			throw std::runtime_error(std::string("failed to add gait, because \"")+cmdName+"\" already exists");
		}
		else
		{
			allGaits.push_back(gaitFunc);
			allParsers.push_back(parseFunc);

			mapName2ID.insert(std::make_pair(cmdName, allGaits.size() - 1));

			std::cout << cmdName << ":" << mapName2ID.at(cmdName) << std::endl;
		}
	};
	void ControlServer::Imp::Start()
	{
		/*start sensors*/
		if (pImu)
		{
			pImu->Start();
		}

		while (true)
		{
			try
			{
				server.StartServer(port.c_str());
				break;
			}
			catch (Aris::Core::Socket::StartServerError &e)
			{
				std::cout << e.what() << std::endl << "will restart in 1s" << std::endl;
				Aris::Core::Sleep(1000);
			}
		}

#ifdef UNIX
		pController->Start();
#endif
	}
	void ControlServer::Imp::Stop()
	{
		server.Close();

#ifdef UNIX
		pController->Stop();
#endif
		if (pImu)
		{
			pImu->Stop();
		}
	}

	void ControlServer::Imp::OnReceiveMsg(const Aris::Core::Msg &msg, Aris::Core::Msg &retError)
	{
		try
		{
			std::string cmd;
			std::map<std::string, std::string> params;

			DecodeMsg2Param(msg, cmd, params);
			SendParam2RT(cmd, params);
		}
		catch (std::exception &e)
		{
			retError.Copy(e.what());
		}
	}
	void ControlServer::Imp::DecodeMsg2Param(const Aris::Core::Msg &msg, std::string &cmd, std::map<std::string, std::string> &params)
	{
		std::vector<std::string> paramVector;
		int paramNum{0};

		/*将msg转换成cmd和一系列参数，不过这里的参数为原生字符串，既包括名称也包含值，例如“-heigt=0.5”*/
		if (msg.GetDataAddress()[msg.GetLength() - 1] == '\0')
		{
			std::string input{ msg.GetDataAddress() };
			std::stringstream inputStream{ input };
			std::string word;

			if (!(inputStream >> cmd))
			{
				throw std::logic_error(Aris::Core::log("invalid message from client, please at least contain a word"));
			};
			Aris::Core::log(std::string("received command string:")+msg.GetDataAddress());

			while (inputStream >> word)
			{
				paramVector.push_back(word);
				++paramNum;
			}
		}
		else
		{
			throw std::logic_error(Aris::Core::log("invalid message from client, please be sure that the command message end with char \'\\0\'"));
		}

		if (mapCmd.find(cmd) != mapCmd.end())
		{
			mapCmd.at(cmd)->root->Reset();
		}
		else
		{
			throw std::logic_error(Aris::Core::log(std::string("invalid command name, server does not have command \"") + cmd + "\""));
		}

		for (int i = 0; i<paramNum; ++i)
		{
			std::string str{ paramVector[i] };
			std::string paramName, paramValue;
			if (str.find("=") == std::string::npos)
			{
				paramName = str;
				paramValue = "";
			}
			else
			{
				paramName.assign(str, 0, str.find("="));
				paramValue.assign(str, str.find("=") + 1, str.size() - str.find("="));
			}

			if (paramName.size() == 0)
				throw std::logic_error("invalid param: what the hell, param should not start with '='");

			/*not start with '-'*/
			if (paramName.data()[0] != '-')
			{
				if (paramValue != "")
				{
					throw std::logic_error("invalid param: only param start with - or -- can be assigned a value");
				}

				for (auto c : paramName)
				{
					if (mapCmd.at(cmd)->shortNames.find(c) != mapCmd.at(cmd)->shortNames.end())
					{
						params.insert(make_pair(mapCmd.at(cmd)->shortNames.at(c), paramValue));
						mapCmd.at(cmd)->allParams.at(mapCmd.at(cmd)->shortNames.at(c))->Take();
					}
					else
					{
						throw std::logic_error(std::string("invalid param: param \"") + c + "\" is not a abbreviation of any valid param" );
					}
				}

				continue;
			}

			/*all following part start with at least one '-'*/
			if (paramName.size() == 1)
			{
				throw std::logic_error("invalid param: symbol \"-\" must be followed by an abbreviation of param");
			}

			/*start with '-', but only one '-'*/
			if (paramName.data()[1] != '-')
			{
				if (paramName.size() != 2)
				{
					throw std::logic_error("invalid param: param start with single '-' must be an abbreviation");
				}

				char c = paramName.data()[1];

				if (mapCmd.at(cmd)->shortNames.find(c) != mapCmd.at(cmd)->shortNames.end())
				{
					params.insert(make_pair(mapCmd.at(cmd)->shortNames.at(c), paramValue));
					mapCmd.at(cmd)->allParams.at(mapCmd.at(cmd)->shortNames.at(c))->Take();
				}
				else
				{
					throw std::logic_error(std::string("invalid param: param \"") + c + "\" is not a abbreviation of any valid param");
				}

				continue;
			}
			else
			{
				/*start with '--'*/
				if (paramName.size()<3)
				{
					throw std::logic_error("invalid param: symbol \"--\" must be followed by a full name of param");
				}

				std::string str = paramName;
				paramName.assign(str, 2, str.size() - 2);

				if (mapCmd.at(cmd)->allParams.find(paramName) != mapCmd.at(cmd)->allParams.end())
				{
					params.insert(make_pair(paramName, paramValue));
					mapCmd.at(cmd)->allParams.at(paramName)->Take();
				}
				else
				{
					throw std::logic_error(std::string("invalid param: param \"") + paramName + "\" is not a valid param");
				}

				

				continue;
			}
		}

		AddAllDefault(mapCmd.at(cmd)->root.get(), params);

		std::cout << cmd << std::endl;

		int paramPrintLength;
		if (params.empty())
		{
			paramPrintLength = 2;
		}
		else
		{
			paramPrintLength = std::max_element(params.begin(), params.end(), [](decltype(*params.begin()) a, decltype(*params.begin()) b)
			{
				return a.first.length() < b.first.length();
			})->first.length() + 2;
		}

		int maxParamNameLength{ 0 };
		for (auto &i : params)
		{
			std::cout << std::string(paramPrintLength-i.first.length(),' ') << i.first << " : " << i.second << std::endl;
		}
	}
	void ControlServer::Imp::SendParam2RT(const std::string &cmd, const std::map<std::string, std::string> &params)
	{
		Aris::Core::Msg cmd_msg;
		
		if (cmd == "en")
		{
			parse_enable_func_(cmd, params, cmd_msg);
			if (cmd_msg.GetLength() != sizeof(BasicFunctionParam))throw std::runtime_error("invalid msg length of parse function for en");
			reinterpret_cast<BasicFunctionParam *>(cmd_msg.GetDataAddress())->cmd_type = Aris::ControlServer::Imp::ENABLE;
		}
		else if (cmd == "ds")
		{
			parse_disable_func_(cmd, params, cmd_msg);
			if (cmd_msg.GetLength() != sizeof(BasicFunctionParam))throw std::runtime_error("invalid msg length of parse function for ds");
			reinterpret_cast<BasicFunctionParam *>(cmd_msg.GetDataAddress())->cmd_type = Aris::ControlServer::Imp::DISABLE;
		}
		else if (cmd == "hm")
		{
			parse_home_func_(cmd, params, cmd_msg);
			if (cmd_msg.GetLength() != sizeof(BasicFunctionParam))throw std::runtime_error("invalid msg length of parse function for hm");
			reinterpret_cast<BasicFunctionParam *>(cmd_msg.GetDataAddress())->cmd_type = Aris::ControlServer::Imp::HOME;
		}
		else
		{
			auto cmdPair = this->mapName2ID.find(cmd);

			if (cmdPair == this->mapName2ID.end())
			{
				throw std::logic_error(std::string("command \"") + cmdPair->first + "\" does not have gait function, please AddGait() first");
			}

			this->allParsers.at(cmdPair->second).operator()(cmd, params, cmd_msg);

			if (cmd_msg.GetLength() < sizeof(GaitParamBase))
			{
				throw std::logic_error(std::string("parse function of command \"") + cmdPair->first + "\" failed: because it returned invalid cmd_msg");
			}

			reinterpret_cast<GaitParamBase *>(cmd_msg.GetDataAddress())->cmd_type = RUN_GAIT;
			reinterpret_cast<GaitParamBase *>(cmd_msg.GetDataAddress())->cmd_ID = cmdPair->second;

			if (allGaits.at(cmdPair->second) == nullptr) return;
		}

		cmd_msg.SetMsgID(0);
#ifdef UNIX
		this->pController->MsgPipe().SendToRT(cmd_msg);
#endif
	}
	
	int ControlServer::Imp::home(const BasicFunctionParam &param, Aris::Control::EthercatController::Data &data)
	{
		bool isAllHomed = true;

		for (std::size_t i = 0; i < pController->MotNum(); ++i)
		{
			if (param.isMotorActive[i])
			{
				/*根据返回值来判断是否走到home了*/
				if ((param.count != 0) && (data.pMotionData->operator[](i).ret == 0))
				{
					/*判断是否为第一次走到home,否则什么也不做，这样就会继续刷上次的值*/
					if (data.pMotionData->operator[](i).cmd ==  Aris::Control::EthercatMotion::HOME)
					{
						data.pMotionData->operator[](i).cmd =  Aris::Control::EthercatMotion::RUN;
						data.pMotionData->operator[](i).targetPos = data.pMotionData->operator[](i).feedbackPos;
						data.pMotionData->operator[](i).targetVel = 0;
						data.pMotionData->operator[](i).targetCur = 0;
					}
				}
				else
				{
					isAllHomed = false;
					data.pMotionData->operator[](i).cmd =  Aris::Control::EthercatMotion::HOME;

					if (param.count % 1000 == 0)
					{
						rt_printf("Unhomed motor, physical id: %d, absolute id: %d\n", this->pController->a2p(i), i);
					}
				}
			}
		}

		return isAllHomed ? 0 : 1;
	};
	int ControlServer::Imp::enable(const BasicFunctionParam &param, Aris::Control::EthercatController::Data &data)
	{
		bool isAllEnabled = true;

		for (std::size_t  i = 0; i < pController->MotNum(); ++i)
		{
			if (param.isMotorActive[i])
			{
				/*判断是否已经Enable了*/
				if ((param.count != 0) && (data.pMotionData->operator[](i).ret == 0))
				{
					/*判断是否为第一次走到enable,否则什么也不做，这样就会继续刷上次的值*/
					if (data.pMotionData->operator[](i).cmd ==  Aris::Control::EthercatMotion::ENABLE)
					{
						data.pMotionData->operator[](i).cmd =  Aris::Control::EthercatMotion::RUN;
						data.pMotionData->operator[](i).mode =  Aris::Control::EthercatMotion::POSITION;
						data.pMotionData->operator[](i).targetPos = data.pMotionData->operator[](i).feedbackPos;
						data.pMotionData->operator[](i).targetVel = 0;
						data.pMotionData->operator[](i).targetCur = 0;
					}
				}
				else
				{
					isAllEnabled = false;
					data.pMotionData->operator[](i).cmd =  Aris::Control::EthercatMotion::ENABLE;
					data.pMotionData->operator[](i).mode =  Aris::Control::EthercatMotion::POSITION;

					if (param.count % 1000 == 0)
					{
						rt_printf("Unenabled motor, physical id: %d, absolute id: %d\n", this->pController->a2p(i), i);
					}
				}
			}
		}

		return isAllEnabled ? 0 : 1;
	};
	int ControlServer::Imp::disable(const BasicFunctionParam &param, Aris::Control::EthercatController::Data &data)
	{
		bool isAllDisabled = true;

		for (std::size_t  i = 0; i < pController->MotNum(); ++i)
		{
			if (param.isMotorActive[i])
			{
				/*判断是否已经Disabled了*/
				if ((param.count != 0) && (data.pMotionData->operator[](i).ret == 0))
				{
					/*如果已经disable了，那么什么都不做*/
				}
				else
				{
					/*否则往下刷disable指令*/
					isAllDisabled = false;
					data.pMotionData->operator[](i).cmd =  Aris::Control::EthercatMotion::DISABLE;

					if (param.count % 1000 == 0)
					{
						rt_printf("Undisabled motor, physical id: %d, absolute id: %d\n", this->pController->a2p(i), i);
					}
				}
			}
		}

		return isAllDisabled ? 0 : 1;
	}
	int ControlServer::Imp::run(GaitParamBase &param, Aris::Control::EthercatController::Data &data)
	{
		//获取陀螺仪传感器数据
		Aris::Sensor::SensorData<Aris::Sensor::ImuData> imuDataProtected;
		if (pImu) imuDataProtected = pImu->GetSensorData();
		param.pImuData = &imuDataProtected.Get();

		//获取力传感器数据
		param.pForceData = data.pForceSensorData;

		//执行gait函数
		int ret = this->allGaits.at(param.cmd_ID).operator()(*pModel_.get(),param);

		//向下写入输入位置
		for (std::size_t  i = 0; i<pController->MotNum(); ++i)
		{
			data.pMotionData->operator[](i).cmd =  Aris::Control::EthercatMotion::RUN;
			data.pMotionData->operator[](i).targetPos = static_cast<std::int32_t>(pModel_->MotionAt(i).MotPos() * pController->MotionAtAbs(i).Input2Count());
		}

		return ret;
	}
	
	int ControlServer::Imp::execute_cmd(int count, char *cmd, Aris::Control::EthercatController::Data &data)
	{
		int ret;
		Aris::DynKer::PlanParamBase *pParam = reinterpret_cast<Aris::DynKer::PlanParamBase *>(cmd);
		pParam->count = count;

		switch (pParam->cmd_type)
		{
		case ENABLE:
			ret = enable(static_cast<BasicFunctionParam &>(*pParam), data);
			break;
		case DISABLE:
			ret = disable(static_cast<BasicFunctionParam &>(*pParam), data);
			break;
		case HOME:
			ret = home(static_cast<BasicFunctionParam &>(*pParam), data);
			break;
		case RUN_GAIT:
			ret = run(static_cast<GaitParamBase &>(*pParam), data);
			break;
		default:
			rt_printf("unknown cmd type\n");
			ret = 0;
			break;
		}

		return ret;
	}
	int ControlServer::Imp::tg(Aris::Control::EthercatController::Data &data)
	{
		enum { CMD_POOL_SIZE = 50 };
		
		static char cmdQueue[CMD_POOL_SIZE][Aris::Core::MsgRT::RT_MSG_LENGTH];

		static int currentCmd{ 0 };
		static int cmdNum{ 0 };
		static int count{ 0 };
		static ControlServer::Imp *pImp;
		pImp = ControlServer::Instance().pImp.get();

		/*static int dspNum = 0;
		if (++dspNum % 1000 == 0)
		{
			rt_printf("pos is:%d \n",data.pMotionData->at(0).feedbackPos);
		}*/

		/*检查是否出错*/
		bool isAllNormal = true;
		for (auto &motData : *data.pMotionData)
		{
			if (motData.ret < 0)
			{
				isAllNormal = false;
				break;
			}
		}
		static int faultCount = 0;
		if (isAllNormal)
		{
			faultCount = 0;
		}
		else
		{
			if (faultCount++ % 1000 == 0)
			{
				for (auto &motData : *data.pMotionData)
				{
					rt_printf("%d ", motData.ret);
				}				
				rt_printf("\n");

				rt_printf("Some motor is in fault, now try to disable all motors\n");
				rt_printf("All commands in command queue are discarded\n");
			}
			for (auto &motData : *data.pMotionData)
			{
				motData.cmd =  Aris::Control::EthercatMotion::DISABLE;
			}
			
			cmdNum = 0;
			count = 0;
			return 0;
		}

		//查看是否有新cmd
		if (data.pMsgRecv)
		{
			if (cmdNum >= CMD_POOL_SIZE)
			{
				rt_printf("cmd pool is full, thus ignore last one\n");
			}
			else
			{
				data.pMsgRecv->Paste(cmdQueue[(currentCmd + cmdNum) % CMD_POOL_SIZE]);
				++cmdNum;
			}
		}

		//执行cmd queue中的cmd
		if (cmdNum>0)
		{
			if (pImp->execute_cmd(count, cmdQueue[currentCmd], data) == 0)
			{
				rt_printf("cmd finished, spend %d counts\n\n", count + 1);				
				count = 0;
				currentCmd = (currentCmd + 1) % CMD_POOL_SIZE;
				--cmdNum;				
			}
			else
			{
				++count;
				if (count % 1000 == 0)
				{
					rt_printf("execute cmd in count: %d\n", count);
				}
			}
		}

		//检查连续
		for (std::size_t i = 0; i<pImp->pController->MotNum(); ++i)
		{
			if ((data.pLastMotionData->at(i).cmd ==  Aris::Control::EthercatMotion::RUN)
				&& (data.pMotionData->at(i).cmd ==  Aris::Control::EthercatMotion::RUN)
				&& (std::abs(data.pLastMotionData->at(i).targetPos - data.pMotionData->at(i).targetPos)>1.2*pImp->pController->MotionAtAbs(i).MaxSpeed()))
			{
				rt_printf("Data not continuous in count:%d\n", count);

				rt_printf("The input of last and this count are:\n");
				for (std::size_t i = 0; i<pImp->pController->MotNum(); ++i)
				{
					rt_printf("%d   %d\n", data.pLastMotionData->at(i).targetPos, data.pMotionData->at(i).targetPos);
				}

				rt_printf("All commands in command queue are discarded\n");
				cmdNum = 0;
				count = 0;

				/*发现不连续，那么使用上一个成功的cmd，以便等待修复*/
				for (int i = 0; i < 18; ++i)
				{
					data.pMotionData->operator[](i) = data.pLastMotionData->operator[](i);
				}


				return 0;
			}
		}

		return 0;
	}

	ControlServer &ControlServer::Instance()
	{
		static ControlServer instance;
		return std::ref(instance);
	}
	ControlServer::ControlServer():pImp(new Imp(this)){}
	ControlServer::~ControlServer(){}
	void ControlServer::CreateModel(DynKer::Model *pModel)
	{
		if (pImp->pModel_)throw std::runtime_error("control sever can't create model because it already has one");

		pImp->pModel_.reset(pModel);
	}
	void ControlServer::LoadXml(const char *fileName)
	{
		Aris::Core::XmlDocument doc;

		if (doc.LoadFile(fileName) != 0)
		{
			throw std::logic_error((std::string("could not open file:") + std::string(fileName)));
		}
		
		pImp->LoadXml(doc);
	}
	void ControlServer::LoadXml(const Aris::Core::XmlDocument &xmlDoc)
	{
		pImp->LoadXml(xmlDoc);
	}
	void ControlServer::AddGait(std::string cmdName, DynKer::PlanFunc gaitFunc, ParseFunc parseFunc)
	{
		pImp->AddGait(cmdName, gaitFunc, parseFunc);
	}
	void ControlServer::Start()
	{
		pImp->Start();
	}
	void ControlServer::Stop()
	{
		this->pImp->Stop();
	}
	void ControlServer::SetParseFunc(const std::string &cmd, const ParseFunc &parse_func)
	{
		if (cmd == "en")
		{
			pImp->parse_enable_func_ = parse_func;
		}
		else if (cmd == "ds")
		{
			pImp->parse_disable_func_ = parse_func;
		}
		else if (cmd == "hm")
		{
			pImp->parse_home_func_ = parse_func;
		}
		else
		{
			throw std::runtime_error("you can not set parse function for normal gait");
		}

	}

}








