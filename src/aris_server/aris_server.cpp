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

#include "aris_core.h"
#include "aris_control.h"
#include "aris_server.h"

namespace Aris
{
	namespace Server
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
			for (auto pChild = pEle->FirstChildElement(); pChild != nullptr; pChild = pChild->NextSiblingElement())
			{
				/*check if children already has this value*/
				if (pNode->FindChild(pChild->name()))
				{
					throw std::logic_error(std::string("XML file has error: node \"") + pChild->name() + "\" already exist");
				}

				/*set all children*/
				if (pChild->Attribute("type", "group"))
				{
					AddAllParams(pChild, pNode->AddChildGroup(pChild->name()), allParams, shortNames);
				}
				else if (pChild->Attribute("type", "unique"))
				{
					AddAllParams(pChild, pNode->AddChildUnique(pChild->name()), allParams, shortNames);
				}
				else
				{
					/*now the pChild is a param_node*/
					Node * insertNode;

					if (allParams.find(std::string(pChild->name())) != allParams.end())
					{
						throw std::logic_error(std::string("XML file has error: node \"") + pChild->name() + "\" already exist");
					}
					else
					{
						insertNode = pNode->AddChildParam(pChild->name());
						allParams.insert(std::pair<std::string, Node *>(std::string(pChild->name()), insertNode));
					}

					/*set abbreviation*/
					if (pChild->Attribute("abbreviation"))
					{
						if (shortNames.find(*pChild->Attribute("abbreviation")) != shortNames.end())
						{
							throw std::logic_error(std::string("XML file has error: abbreviations \"") + pChild->Attribute("abbreviation") + "\" already exist");
						}
						else
						{
							char abbr = *pChild->Attribute("abbreviation");
							shortNames.insert(std::pair<char, std::string>(abbr, std::string(pChild->name())));
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

			CommandStruct(const std::string &name) :root(new RootNode(name.c_str())) {};
		};

		class ControlServer::Imp
		{
		public:
			auto loadXml(const Aris::Core::XmlDocument &doc)->void;
			auto addCmd(const std::string &cmd_name, const ParseFunc &parse_func, const Aris::Dynamic::PlanFunc &gait_func)->void;
			auto start()->void;
			auto stop()->void;

			Imp(ControlServer *server)
			{
				this->server_ = server;
#ifdef UNIX
				this->controller_ = Aris::Control::EthercatController::createInstance<Aris::Control::EthercatController>();
#endif
			};
		private:
			Imp(const Imp&) = delete;

			auto OnReceiveMsg(const Aris::Core::Msg &msg)->Aris::Core::Msg;
			auto DecodeMsg2Param(const Aris::Core::Msg &msg, std::string &cmd, std::map<std::string, std::string> &params)->void;
			auto SendParam2RT(const std::string &cmd, const std::map<std::string, std::string> &params)->void;

			auto home(const BasicFunctionParam &param, Aris::Control::EthercatController::Data &data)->int;
			auto enable(const BasicFunctionParam &param, Aris::Control::EthercatController::Data &data)->int;
			auto disable(const BasicFunctionParam &param, Aris::Control::EthercatController::Data &data)->int;
			auto run(GaitParamBase &param, Aris::Control::EthercatController::Data &data)->int;

			auto execute_cmd(int count, char *cmd, Aris::Control::EthercatController::Data &data)->int;
			static auto tg(Aris::Control::EthercatController::Data &data)->int;

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
			std::atomic_bool is_running_{false};
			
			ControlServer *server_;

			//以下储存所有的命令//
			std::map<std::string, int> map_cmd2id_;//store gait id in follow vector
			std::vector<Dynamic::PlanFunc> plan_vec_;
			std::vector<ParseFunc> parser_vec_;
			std::map<std::string, std::unique_ptr<CommandStruct> > mapCmd;//store Node of command

			//储存特殊命令的parse_func//
			ParseFunc parse_enable_func_{ [this](const std::string &cmd, const std::map<std::string, std::string> &params, Aris::Core::Msg &msg)
			{
				BasicFunctionParam param;
				param.cmd_type = Imp::RobotCmdID::ENABLE;
				std::fill_n(param.active_motor, this->model_->motionPool().size(), true);
				msg.copyStruct(param);
			} };
			ParseFunc parse_disable_func_{ [this](const std::string &cmd, const std::map<std::string, std::string> &params, Aris::Core::Msg &msg)
			{
				BasicFunctionParam param;
				param.cmd_type = Imp::RobotCmdID::DISABLE;
				std::fill_n(param.active_motor, this->model_->motionPool().size(), true);
				msg.copyStruct(param);
			} };
			ParseFunc parse_home_func_{ [this](const std::string &cmd, const std::map<std::string, std::string> &params, Aris::Core::Msg &msg)
			{
				BasicFunctionParam param;
				param.cmd_type = Imp::RobotCmdID::HOME;
				std::fill_n(param.active_motor, this->model_->motionPool().size(), true);
				msg.copyStruct(param);
			} };

			//socket//
			Aris::Core::Socket server_socket_;
			std::string server_socket_ip_, server_socket_port_;

			//储存控制器等//
			Aris::Control::EthercatController *controller_;
			std::unique_ptr<Aris::Dynamic::Model> model_;
			std::unique_ptr<Aris::Sensor::IMU> imu_;


			std::vector<double> motion_pos_;
			friend class ControlServer;
		};

		auto ControlServer::Imp::loadXml(const Aris::Core::XmlDocument &doc)->void
		{
			/*load robot model_*/
			model_->loadXml(doc);

			/*begin to create imu*/
			if (doc.RootElement()->FirstChildElement("Server")->FirstChildElement("Sensors")->FirstChildElement("IMU")->Attribute("active", "true"))
			{
				std::cout << "imu found" << std::endl;
				imu_.reset(new Aris::Sensor::IMU(doc.RootElement()->FirstChildElement("Server")->FirstChildElement("Sensors")->FirstChildElement("IMU")));
			}
			else
			{
				std::cout << "imu not find" << std::endl;
			}

			/*begin to load controller_*/
#ifdef UNIX
			controller_->loadXml(std::ref(*doc.RootElement()->FirstChildElement("Controller")->FirstChildElement("EtherCat")));
			controller_->setControlStrategy(tg);
#endif

			/*load connection param*/
			auto pConnEle = doc.RootElement()->FirstChildElement("Server")->FirstChildElement("Connection");
			server_socket_ip_ = pConnEle->Attribute("IP");
			server_socket_port_ = pConnEle->Attribute("Port");

			/*begin to insert cmd nodes*/
			auto pCmds = doc.RootElement()->FirstChildElement("Server")->FirstChildElement("Commands");

			if (pCmds == nullptr) throw std::runtime_error("invalid xml file, because it contains no commands information");
			mapCmd.clear();
			for (auto pChild = pCmds->FirstChildElement(); pChild != nullptr; pChild = pChild->NextSiblingElement())
			{
				if (mapCmd.find(pChild->name()) != mapCmd.end())
					throw std::logic_error(std::string("command ") + pChild->name() + " is already existed, please rename it");

				mapCmd.insert(std::make_pair(std::string(pChild->name()), std::unique_ptr<CommandStruct>(new CommandStruct(pChild->name()))));
				AddAllParams(pChild, mapCmd.at(pChild->name())->root.get(), mapCmd.at(pChild->name())->allParams, mapCmd.at(pChild->name())->shortNames);
			}

			/*Set socket connection callback function*/
			server_socket_.setOnReceivedConnection([](Aris::Core::Socket *pConn, const char *pRemoteIP, int remotePort)
			{
				Aris::Core::log(std::string("received connection, the server_socket_ip_ is: ") + pRemoteIP);
				return 0;
			});
			server_socket_.setOnReceivedRequest([this](Aris::Core::Socket *pConn, Aris::Core::Msg &msg)
			{
				return OnReceiveMsg(msg);
			});
			server_socket_.setOnLoseConnection([this](Aris::Core::Socket *pConn)
			{
				std::cout << Aris::Core::log("lost connection") << std::endl;
				while (true)
				{
					try
					{
						pConn->startServer(this->server_socket_port_.c_str());
						break;
					}
					catch (Aris::Core::Socket::StartServerError &e)
					{
						std::cout << e.what() << std::endl << "will try to restart server socket in 1s" << std::endl;
						Aris::Core::msSleep(1000);
					}
				}
				std::cout << Aris::Core::log("restart server socket successful") << std::endl;

				return 0;
			});
		}
		auto ControlServer::Imp::addCmd(const std::string &cmd_name, const ParseFunc &parse_func, const Aris::Dynamic::PlanFunc &gait_func)->void
		{
			if (cmd_name == "en")
			{
				if (gait_func)throw std::runtime_error("you can not set plan_func for enable command");
				this->parse_enable_func_ = parse_func;
			}
			else if (cmd_name == "ds")
			{
				if (gait_func)throw std::runtime_error("you can not set plan_func for disable command");
				this->parse_disable_func_ = parse_func;
			}
			else if (cmd_name == "hm")
			{
				if (gait_func)throw std::runtime_error("you can not set plan_func for home command");
				this->parse_home_func_ = parse_func;
			}
			else
			{
				if (map_cmd2id_.find(cmd_name) != map_cmd2id_.end())
				{
					throw std::runtime_error(std::string("failed to add command, because \"") + cmd_name + "\" already exists");
				}
				else
				{
					plan_vec_.push_back(gait_func);
					parser_vec_.push_back(parse_func);

					map_cmd2id_.insert(std::make_pair(cmd_name, plan_vec_.size() - 1));

					std::cout << cmd_name << ":" << map_cmd2id_.at(cmd_name) << std::endl;
				}
			}
		};
		auto ControlServer::Imp::start()->void
		{
			if (!is_running_)
			{
				is_running_ = true;
				motion_pos_.resize(controller_->motionNum());
				if (imu_)imu_->start();
#ifdef UNIX
				controller_->start();
#endif
			}
		}
		auto ControlServer::Imp::stop()->void
		{
			if (is_running_)
			{
#ifdef UNIX
				controller_->stop();
#endif
				if (imu_)imu_->stop();
				is_running_ = false;
			}

		}
		auto ControlServer::Imp::OnReceiveMsg(const Aris::Core::Msg &msg)->Aris::Core::Msg
		{
			try
			{
				std::string cmd;
				std::map<std::string, std::string> params;

				DecodeMsg2Param(msg, cmd, params);
				
				if (cmd == "start")
				{
					if (is_running_)throw std::runtime_error("server already started, thus ignore command \"start\"");
					start();
					std::cout << "server started" << std::endl;
					return Aris::Core::Msg();
				}
				if (cmd == "stop")
				{
					if (!is_running_)throw std::runtime_error("server already stopped, thus ignore command \"start\"");
					stop();
					std::cout << "server stopped" << std::endl;
					return Aris::Core::Msg();
				}
				if (cmd == "exit")
				{
					stop();
					server_socket_.stop();
					std::cout << "server exited" << std::endl;
					return Aris::Core::Msg();
				}
				

				if (is_running_)SendParam2RT(cmd, params);

				return Aris::Core::Msg();
			}
			catch (std::exception &e)
			{
				std::cout << Aris::Core::log(e.what()) << std::endl;
				
				Aris::Core::Msg error_msg;
				error_msg.copy(e.what());
				return error_msg;
			}
			catch (...)
			{
				std::cout << Aris::Core::log("unknown exception") << std::endl;
				Aris::Core::Msg error_msg;
				error_msg.copy("unknown exception");
				return error_msg;
			}
		}
		auto ControlServer::Imp::DecodeMsg2Param(const Aris::Core::Msg &msg, std::string &cmd, std::map<std::string, std::string> &params)->void
		{
			std::vector<std::string> paramVector;
			int paramNum{ 0 };

			/*将msg转换成cmd和一系列参数，不过这里的参数为原生字符串，既包括名称也包含值，例如“-heigt=0.5”*/
			if (msg.data()[msg.size() - 1] == '\0')
			{
				std::string input{ msg.data() };
				std::stringstream inputStream{ input };
				std::string word;

				if (!(inputStream >> cmd))
				{
					throw std::runtime_error(Aris::Core::log("invalid message from client, please at least contain a word"));
				};
				Aris::Core::log(std::string("received command string:") + msg.data());

				while (inputStream >> word)
				{
					paramVector.push_back(word);
					++paramNum;
				}
			}
			else
			{
				throw std::runtime_error(Aris::Core::log("invalid message from client, please be sure that the command message end with char \'\\0\'"));
			}

			if (mapCmd.find(cmd) != mapCmd.end())
			{
				mapCmd.at(cmd)->root->Reset();
			}
			else
			{
				throw std::runtime_error(Aris::Core::log(std::string("invalid command name, server does not have command \"") + cmd + "\""));
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
					throw std::runtime_error("invalid param: what the hell, param should not start with '='");

				/*not start with '-'*/
				if (paramName.data()[0] != '-')
				{
					if (paramValue != "")
					{
						throw std::runtime_error("invalid param: only param start with - or -- can be assigned a value");
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
							throw std::runtime_error(std::string("invalid param: param \"") + c + "\" is not a abbreviation of any valid param");
						}
					}

					continue;
				}

				/*all following part start with at least one '-'*/
				if (paramName.size() == 1)
				{
					throw std::runtime_error("invalid param: symbol \"-\" must be followed by an abbreviation of param");
				}

				/*start with '-', but only one '-'*/
				if (paramName.data()[1] != '-')
				{
					if (paramName.size() != 2)
					{
						throw std::runtime_error("invalid param: param start with single '-' must be an abbreviation");
					}

					char c = paramName.data()[1];

					if (mapCmd.at(cmd)->shortNames.find(c) != mapCmd.at(cmd)->shortNames.end())
					{
						params.insert(make_pair(mapCmd.at(cmd)->shortNames.at(c), paramValue));
						mapCmd.at(cmd)->allParams.at(mapCmd.at(cmd)->shortNames.at(c))->Take();
					}
					else
					{
						throw std::runtime_error(std::string("invalid param: param \"") + c + "\" is not a abbreviation of any valid param");
					}

					continue;
				}
				else
				{
					/*start with '--'*/
					if (paramName.size()<3)
					{
						throw std::runtime_error("invalid param: symbol \"--\" must be followed by a full name of param");
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
						throw std::runtime_error(std::string("invalid param: param \"") + paramName + "\" is not a valid param");
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
				std::cout << std::string(paramPrintLength - i.first.length(), ' ') << i.first << " : " << i.second << std::endl;
			}
		}
		auto ControlServer::Imp::SendParam2RT(const std::string &cmd, const std::map<std::string, std::string> &params)->void
		{
			Aris::Core::Msg cmd_msg;

			if (cmd == "en")
			{
				parse_enable_func_(cmd, params, cmd_msg);
				if (cmd_msg.size() != sizeof(BasicFunctionParam))throw std::runtime_error("invalid msg length of parse function for en");
				reinterpret_cast<BasicFunctionParam *>(cmd_msg.data())->cmd_type = ControlServer::Imp::ENABLE;
			}
			else if (cmd == "ds")
			{
				parse_disable_func_(cmd, params, cmd_msg);
				if (cmd_msg.size() != sizeof(BasicFunctionParam))throw std::runtime_error("invalid msg length of parse function for ds");
				reinterpret_cast<BasicFunctionParam *>(cmd_msg.data())->cmd_type = ControlServer::Imp::DISABLE;
			}
			else if (cmd == "hm")
			{
				parse_home_func_(cmd, params, cmd_msg);
				if (cmd_msg.size() != sizeof(BasicFunctionParam))throw std::runtime_error("invalid msg length of parse function for hm");
				reinterpret_cast<BasicFunctionParam *>(cmd_msg.data())->cmd_type = ControlServer::Imp::HOME;
			}
			else
			{
				auto cmdPair = this->map_cmd2id_.find(cmd);

				if (cmdPair == this->map_cmd2id_.end())
				{
					throw std::runtime_error(std::string("command \"") + cmd + "\" does not have gait function, please AddCmd() first");
				}

				this->parser_vec_.at(cmdPair->second).operator()(cmd, params, cmd_msg);

				if (cmd_msg.size() < sizeof(GaitParamBase))
				{
					throw std::runtime_error(std::string("parse function of command \"") + cmdPair->first + "\" failed: because it returned invalid cmd_msg");
				}

				reinterpret_cast<GaitParamBase *>(cmd_msg.data())->cmd_type = RUN_GAIT;
				reinterpret_cast<GaitParamBase *>(cmd_msg.data())->gait_id = cmdPair->second;

				if (plan_vec_.at(cmdPair->second) == nullptr) return;
			}

			cmd_msg.setMsgID(0);
#ifdef UNIX
			this->controller_->msgPipe().sendToRT(cmd_msg);
#endif
		}
		auto ControlServer::Imp::home(const BasicFunctionParam &param, Aris::Control::EthercatController::Data &data)->int
		{
			bool isAllHomed = true;

			for (std::size_t i = 0; i < controller_->motionNum(); ++i)
			{
				if (param.active_motor[i])
				{
					/*根据返回值来判断是否走到home了*/
					if ((param.count != 0) && (data.motion_rawdata->operator[](i).ret == 0))
					{
						/*判断是否为第一次走到home,否则什么也不做，这样就会继续刷上次的值*/
						if (data.motion_rawdata->operator[](i).cmd == Aris::Control::EthercatMotion::HOME)
						{
							data.motion_rawdata->operator[](i).cmd = Aris::Control::EthercatMotion::RUN;
							data.motion_rawdata->operator[](i).target_pos = data.motion_rawdata->operator[](i).feedback_pos;
							data.motion_rawdata->operator[](i).target_vel = 0;
							data.motion_rawdata->operator[](i).target_cur = 0;
						}
					}
					else
					{
						isAllHomed = false;
						data.motion_rawdata->operator[](i).cmd = Aris::Control::EthercatMotion::HOME;

						if (param.count % 1000 == 0)
						{
							rt_printf("Unhomed motor, physical id: %d, absolute id: %d\n", this->controller_->motionAtAbs(i).phyID(), i);
						}
					}
				}
			}

			return isAllHomed ? 0 : 1;
		};
		auto ControlServer::Imp::enable(const BasicFunctionParam &param, Aris::Control::EthercatController::Data &data)->int
		{
			bool isAllEnabled = true;

			for (std::size_t i = 0; i < controller_->motionNum(); ++i)
			{
				if (param.active_motor[i])
				{
					/*判断是否已经Enable了*/
					if ((param.count != 0) && (data.motion_rawdata->operator[](i).ret == 0))
					{
						/*判断是否为第一次走到enable,否则什么也不做，这样就会继续刷上次的值*/
						if (data.motion_rawdata->operator[](i).cmd == Aris::Control::EthercatMotion::ENABLE)
						{
							data.motion_rawdata->operator[](i).cmd = Aris::Control::EthercatMotion::RUN;
							data.motion_rawdata->operator[](i).mode = Aris::Control::EthercatMotion::POSITION;
							data.motion_rawdata->operator[](i).target_pos = data.motion_rawdata->operator[](i).feedback_pos;
							data.motion_rawdata->operator[](i).target_vel = 0;
							data.motion_rawdata->operator[](i).target_cur = 0;
						}
					}
					else
					{
						isAllEnabled = false;
						data.motion_rawdata->operator[](i).cmd = Aris::Control::EthercatMotion::ENABLE;
						data.motion_rawdata->operator[](i).mode = Aris::Control::EthercatMotion::POSITION;

						if (param.count % 1000 == 0)
						{
							rt_printf("Unenabled motor, physical id: %d, absolute id: %d\n", this->controller_->motionAtAbs(i).phyID(), i);
						}
					}
				}
			}

			return isAllEnabled ? 0 : 1;
		};
		auto ControlServer::Imp::disable(const BasicFunctionParam &param, Aris::Control::EthercatController::Data &data)->int
		{
			bool isAllDisabled = true;

			for (std::size_t i = 0; i < controller_->motionNum(); ++i)
			{
				if (param.active_motor[i])
				{
					/*判断是否已经Disabled了*/
					if ((param.count != 0) && (data.motion_rawdata->operator[](i).ret == 0))
					{
						/*如果已经disable了，那么什么都不做*/
					}
					else
					{
						/*否则往下刷disable指令*/
						isAllDisabled = false;
						data.motion_rawdata->operator[](i).cmd = Aris::Control::EthercatMotion::DISABLE;

						if (param.count % 1000 == 0)
						{
							rt_printf("Undisabled motor, physical id: %d, absolute id: %d\n", this->controller_->motionAtAbs(i).phyID(), i);
						}
					}
				}
			}

			return isAllDisabled ? 0 : 1;
		}
		auto ControlServer::Imp::run(GaitParamBase &param, Aris::Control::EthercatController::Data &data)->int
		{
			//获取陀螺仪传感器数据
			Aris::Sensor::SensorData<Aris::Sensor::ImuData> imuDataProtected;
			if (imu_) imuDataProtected = imu_->getSensorData();
			param.imu_data = &imuDataProtected.get();

			//获取力传感器数据与电机数据
			param.force_data = data.force_sensor_data;
			param.motion_raw_data = data.motion_rawdata;
			param.motion_feedback_pos = &this->motion_pos_;

			for (std::size_t i = 0; i < data.motion_rawdata->size(); ++i)
			{
				this->motion_pos_[i] = static_cast<double>(data.motion_rawdata->at(i).feedback_pos) / controller_->motionAtAbs(i).pos2countRatio();
			}

			//执行gait函数
			int ret = this->plan_vec_.at(param.gait_id).operator()(*model_.get(), param);

			//向下写入输入位置
			for (std::size_t i = 0; i<controller_->motionNum(); ++i)
			{
				data.motion_rawdata->operator[](i).cmd = Aris::Control::EthercatMotion::RUN;
				data.motion_rawdata->operator[](i).target_pos = static_cast<std::int32_t>(model_->motionPool().at(i).motPos() * controller_->motionAtAbs(i).pos2countRatio());
			}

			return ret;
		}
		auto ControlServer::Imp::execute_cmd(int count, char *cmd, Aris::Control::EthercatController::Data &data)->int
		{
			int ret;
			Aris::Dynamic::PlanParamBase *param = reinterpret_cast<Aris::Dynamic::PlanParamBase *>(cmd);
			param->count = count;

			switch (param->cmd_type)
			{
			case ENABLE:
				ret = enable(static_cast<BasicFunctionParam &>(*param), data);
				break;
			case DISABLE:
				ret = disable(static_cast<BasicFunctionParam &>(*param), data);
				break;
			case HOME:
				ret = home(static_cast<BasicFunctionParam &>(*param), data);
				break;
			case RUN_GAIT:
				ret = run(static_cast<GaitParamBase &>(*param), data);
				break;
			default:
				rt_printf("unknown cmd type\n");
				ret = 0;
				break;
			}

			return ret;
		}
		auto ControlServer::Imp::tg(Aris::Control::EthercatController::Data &data)->int
		{
			enum { CMD_POOL_SIZE = 50 };
			static char cmdQueue[CMD_POOL_SIZE][Aris::Core::MsgRT::RT_MSG_LENGTH];
			static int currentCmd{ 0 }, cmdNum{ 0 }, count{ 0 };
			static ControlServer::Imp *imp = ControlServer::instance().imp.get();

			/*static int dspNum = 0;
			if (++dspNum % 1000 == 0)
			{
			rt_printf("pos is:%d \n",data.pMotionData->at(0).feedbackPos);
			}*/

			//检查是否出错//
			bool is_all_normal = data.motion_rawdata->end() == std::find_if(data.motion_rawdata->begin(), data.motion_rawdata->end(), [](const Aris::Control::EthercatMotion::RawData &data) {return data.ret < 0; });

			static int faultCount = 0;
			if (is_all_normal)
			{
				faultCount = 0;
			}
			else
			{
				if (faultCount++ % 1000 == 0)
				{
					for (auto &motData : *data.motion_rawdata)
					{
						rt_printf("%d ", motData.ret);
					}
					rt_printf("\n");

					rt_printf("Some motor is in fault, now try to disable all motors\n");
					rt_printf("All commands in command queue are discarded\n");
				}
				for (auto &motData : *data.motion_rawdata)
				{
					motData.cmd = Aris::Control::EthercatMotion::DISABLE;
				}

				cmdNum = 0;
				count = 0;
				return 0;
			}

			//查看是否有新cmd//
			if (data.msg_recv)
			{
				if (cmdNum >= CMD_POOL_SIZE)
				{
					rt_printf("cmd pool is full, thus ignore last one\n");
				}
				else
				{
					data.msg_recv->paste(cmdQueue[(currentCmd + cmdNum) % CMD_POOL_SIZE]);
					++cmdNum;
				}
			}

			//执行cmd queue中的cmd//
			if (cmdNum>0)
			{
				if (imp->execute_cmd(count, cmdQueue[currentCmd], data) == 0)
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

			//检查连续//
			for (std::size_t i = 0; i<imp->controller_->motionNum(); ++i)
			{
				if ((data.last_motion_rawdata->at(i).cmd == Aris::Control::EthercatMotion::RUN)
					&& (data.motion_rawdata->at(i).cmd == Aris::Control::EthercatMotion::RUN)
					&& (std::abs(data.last_motion_rawdata->at(i).target_pos - data.motion_rawdata->at(i).target_pos)>1.2*imp->controller_->motionAtAbs(i).maxVelCount()))
				{
					rt_printf("Data not continuous in count:%d\n", count);

					rt_printf("The input of last and this count are:\n");
					for (std::size_t i = 0; i<imp->controller_->motionNum(); ++i)
					{
						rt_printf("%d   %d\n", data.last_motion_rawdata->at(i).target_pos, data.motion_rawdata->at(i).target_pos);
					}

					rt_printf("All commands in command queue are discarded\n");
					cmdNum = 0;
					count = 0;

					/*发现不连续，那么使用上一个成功的cmd，以便等待修复*/
					for (int i = 0; i < 18; ++i)
					{
						data.motion_rawdata->operator[](i) = data.last_motion_rawdata->operator[](i);
					}


					return 0;
				}
			}

			return 0;
		}

		ControlServer &ControlServer::instance()
		{
			static ControlServer instance;
			return std::ref(instance);
		}
		ControlServer::ControlServer() :imp(new Imp(this)) {}
		ControlServer::~ControlServer() {}
		auto ControlServer::createModel(Dynamic::Model *model_)->void
		{
			if (imp->model_)throw std::runtime_error("control sever can't create model_ because it already has one");

			imp->model_.reset(model_);
		}
		auto ControlServer::loadXml(const char *fileName)->void
		{
			Aris::Core::XmlDocument doc;

			if (doc.LoadFile(fileName) != 0)
			{
				throw std::logic_error((std::string("could not open file:") + std::string(fileName)));
			}

			loadXml(doc);
		}
		auto ControlServer::loadXml(const Aris::Core::XmlDocument &xmlDoc)->void {	imp->loadXml(xmlDoc);}
		auto ControlServer::addCmd(const std::string &cmd_name, const ParseFunc &parse_func, const Aris::Dynamic::PlanFunc &gait_func)->void
		{
			imp->addCmd(cmd_name, parse_func, gait_func);
		}
		auto ControlServer::open()->void 
		{
			while (true)
			{
				try
				{
					imp->server_socket_.startServer(imp->server_socket_port_.c_str());
					break;
				}
				catch (Aris::Core::Socket::StartServerError &e)
				{
					std::cout << e.what() << std::endl << "will try to restart server socket in 1s" << std::endl;
					Aris::Core::msSleep(1000);
				}
			}
			std::cout << Aris::Core::log("restart server socket successful") << std::endl;
		};
		auto ControlServer::close()->void 
		{
			imp->server_socket_.stop();
		};

		
	}
	
}








