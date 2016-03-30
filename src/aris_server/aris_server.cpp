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

namespace aris
{
	namespace server
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

			friend void AddAllParams(const aris::core::XmlElement *pEle, Node *pNode, std::map<std::string, Node *> &allParams, std::map<char, std::string>& shortNames);
			friend void AddAllDefault(Node *pNode, std::map<std::string, std::string> &params);
		};
		class RootNode :public Node
		{
		public:
			RootNode(const char *Name) :Node(nullptr, Name) {};

		private:
			Node *pDefault;

			friend void AddAllParams(const aris::core::XmlElement *pEle, Node *pNode, std::map<std::string, Node *> &allParams, std::map<char, std::string>& shortNames);
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

			friend void AddAllParams(const aris::core::XmlElement *pEle, Node *pNode, std::map<std::string, Node *> &allParams, std::map<char, std::string>& shortNames);
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

			friend void AddAllParams(const aris::core::XmlElement *pEle, Node *pNode, std::map<std::string, Node *> &allParams, std::map<char, std::string>& shortNames);
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
		auto Node::Take()->void
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

		auto AddAllParams(const aris::core::XmlElement *pEle, Node *pNode, std::map<std::string, Node *> &allParams, std::map<char, std::string>& shortNames)->void
		{
			//add all children//
			for (auto pChild = pEle->FirstChildElement(); pChild != nullptr; pChild = pChild->NextSiblingElement())
			{
				//check if children already has this value//
				if (pNode->FindChild(pChild->name()))
				{
					throw std::runtime_error(std::string("XML file has error: node \"") + pChild->name() + "\" already exist");
				}

				//set all children//
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
					//now the pChild is a param_node//
					Node * insertNode;

					if (allParams.find(std::string(pChild->name())) != allParams.end())
					{
						throw std::runtime_error(std::string("XML file has error: node \"") + pChild->name() + "\" already exist");
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
							throw std::runtime_error(std::string("XML file has error: abbreviation \"") + pChild->Attribute("abbreviation") + "\" already exist");
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
		auto AddAllDefault(Node *pNode, std::map<std::string, std::string> &params)->void
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
					for (auto &i : pNode->children)	AddAllDefault(i.get(), params);
				}

				if (dynamic_cast<ParamNode*>(pNode))
				{
					if (params.at(pNode->name) == "")params.at(pNode->name) = dynamic_cast<ParamNode*>(pNode)->defaultValue;

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
			auto loadXml(const aris::core::XmlDocument &doc)->void;
			auto addCmd(const std::string &cmd_name, const ParseFunc &parse_func, const aris::dynamic::PlanFunc &gait_func)->void;
			auto start()->void;
			auto stop()->void;

			Imp(ControlServer *server)
			{
				this->server_ = server;
				this->controller_ = aris::control::EthercatController::createInstance<aris::control::EthercatController>();
			};
		private:
			Imp(const Imp&) = delete;

			auto onReceiveMsg(const aris::core::Msg &msg)->aris::core::Msg;
			auto decodeMsg2Param(const aris::core::Msg &msg, std::string &cmd, std::map<std::string, std::string> &params)->void;
			auto sendParam(const std::string &cmd, const std::map<std::string, std::string> &params)->void;

			static auto tg(aris::control::EthercatController::Data &data)->int;
			auto run(GaitParamBase &param, aris::control::EthercatController::Data &data)->int;
			auto execute_cmd(int count, char *cmd, aris::control::EthercatController::Data &data)->int;
			auto enable(const BasicFunctionParam &param, aris::control::EthercatController::Data &data)->int;
			auto disable(const BasicFunctionParam &param, aris::control::EthercatController::Data &data)->int;
			auto home(const BasicFunctionParam &param, aris::control::EthercatController::Data &data)->int;
			auto fake_home(const BasicFunctionParam &param, aris::control::EthercatController::Data &data)->int;

		private:
			enum RobotCmdID
			{
				ENABLE,
				DISABLE,
				HOME,
				RUN_GAIT,
				FAKE_HOME,

				ROBOT_CMD_COUNT
			};

		private:
			std::atomic_bool is_running_{false};
			
			ControlServer *server_;

			// 实时循环中的步态参数 //
			enum { CMD_POOL_SIZE = 50 };
			char cmd_queue_[CMD_POOL_SIZE][aris::core::MsgRT::RT_MSG_LENGTH];
			int current_cmd_{ 0 }, cmd_num_{ 0 }, count_{ 0 };

			// 以下储存所有的命令 //
			std::map<std::string, int> cmd_id_map_;//store gait id in follow vector
			std::vector<dynamic::PlanFunc> plan_vec_;// store plan func
			std::vector<ParseFunc> parser_vec_; // store parse func
			std::map<std::string, std::unique_ptr<CommandStruct> > cmd_struct_map_;//store Node of command

			// 储存特殊命令的parse_func //
			ParseFunc parse_enable_func_{ [this](const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)
			{
				BasicFunctionParam param;
				param.cmd_type = Imp::RobotCmdID::ENABLE;
				std::fill_n(param.active_motor, this->model_->motionPool().size(), true);
				msg.copyStruct(param);
			} };
			ParseFunc parse_disable_func_{ [this](const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)
			{
				BasicFunctionParam param;
				param.cmd_type = Imp::RobotCmdID::DISABLE;
				std::fill_n(param.active_motor, this->model_->motionPool().size(), true);
				msg.copyStruct(param);
			} };
			ParseFunc parse_home_func_{ [this](const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)
			{
				BasicFunctionParam param;
				param.cmd_type = Imp::RobotCmdID::HOME;
				std::fill_n(param.active_motor, this->model_->motionPool().size(), true);
				msg.copyStruct(param);
			} };
			ParseFunc parse_fake_home_func_{ [this](const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)
			{
				BasicFunctionParam param;
				param.cmd_type = Imp::RobotCmdID::FAKE_HOME;
				std::fill_n(param.active_motor, this->model_->motionPool().size(), true);
				msg.copyStruct(param);
			} };

			// socket //
			aris::core::Socket server_socket_;
			std::string server_socket_ip_, server_socket_port_;

			// 储存控制器等 //
			aris::control::EthercatController *controller_;
			std::unique_ptr<aris::dynamic::Model> model_;
			std::unique_ptr<aris::sensor::IMU> imu_;

			// 结束时的callback //
			std::function<void(void)> on_exit_callback_{nullptr};

			std::vector<double> motion_pos_;
			friend class ControlServer;
		};

		auto ControlServer::Imp::loadXml(const aris::core::XmlDocument &doc)->void
		{
			/*load robot model_*/
			model_->loadXml(doc);

			/*begin to create imu*/
			if (doc.RootElement()->FirstChildElement("Sensors")->FirstChildElement("IMU")->Attribute("active", "true"))
			{
				std::cout << "imu found" << std::endl;
				imu_.reset(new aris::sensor::IMU(doc.RootElement()->FirstChildElement("Sensors")->FirstChildElement("IMU")));
			}
			else
			{
				std::cout << "imu not find" << std::endl;
			}

			/*begin to load controller_*/
			controller_->loadXml(std::ref(*doc.RootElement()->FirstChildElement("Controller")->FirstChildElement("EtherCat")));
			controller_->setControlStrategy(tg);

			/*load connection param*/
			server_socket_ip_ = doc.RootElement()->FirstChildElement("Server")->Attribute("ip");
			server_socket_port_ = doc.RootElement()->FirstChildElement("Server")->Attribute("port");

			/*begin to insert cmd nodes*/
			auto pCmds = doc.RootElement()->FirstChildElement("Server")->FirstChildElement("Commands");

			if (pCmds == nullptr) throw std::runtime_error("invalid xml file, because it contains no commands information");
			cmd_struct_map_.clear();
			for (auto pChild = pCmds->FirstChildElement(); pChild != nullptr; pChild = pChild->NextSiblingElement())
			{
				if (cmd_struct_map_.find(pChild->name()) != cmd_struct_map_.end())
					throw std::logic_error(std::string("command ") + pChild->name() + " is already existed, please rename it");

				cmd_struct_map_.insert(std::make_pair(std::string(pChild->name()), std::unique_ptr<CommandStruct>(new CommandStruct(pChild->name()))));
				AddAllParams(pChild, cmd_struct_map_.at(pChild->name())->root.get(), cmd_struct_map_.at(pChild->name())->allParams, cmd_struct_map_.at(pChild->name())->shortNames);
			}

			/*Set socket connection callback function*/
			server_socket_.setOnReceivedConnection([](aris::core::Socket *pConn, const char *pRemoteIP, int remotePort)
			{
				aris::core::log(std::string("received connection, the server_socket_ip_ is: ") + pRemoteIP);
				return 0;
			});
			server_socket_.setOnReceivedRequest([this](aris::core::Socket *pConn, aris::core::Msg &msg)
			{
				return onReceiveMsg(msg);
			});
			server_socket_.setOnLoseConnection([this](aris::core::Socket *socket)
			{
				aris::core::log("lost connection");
				while (true)
				{
					try
					{
						socket->startServer(this->server_socket_port_.c_str());
						break;
					}
					catch (aris::core::Socket::StartServerError &e)
					{
						std::cout << e.what() << std::endl << "will try to restart server socket in 1s" << std::endl;
						aris::core::msSleep(1000);
					}
				}
				aris::core::log("restart server socket successful");

				return 0;
			});
		}
		auto ControlServer::Imp::addCmd(const std::string &cmd_name, const ParseFunc &parse_func, const aris::dynamic::PlanFunc &gait_func)->void
		{
			if (cmd_name == "en")
			{
				if (gait_func)throw std::runtime_error("you can not set plan_func for \"en\" command");
				this->parse_enable_func_ = parse_func;
			}
			else if (cmd_name == "ds")
			{
				if (gait_func)throw std::runtime_error("you can not set plan_func for \"ds\" command");
				this->parse_disable_func_ = parse_func;
			}
			else if (cmd_name == "hm")
			{
				if (gait_func)throw std::runtime_error("you can not set plan_func for \"hm\" command");
				this->parse_home_func_ = parse_func;
			}
			else if (cmd_name == "fake_home")
			{
				if (gait_func)throw std::runtime_error("you can not set plan_func for \"fake_home\" command");
				this->parse_home_func_ = parse_func;
			}
			else
			{
				if (cmd_id_map_.find(cmd_name) != cmd_id_map_.end())
				{
					throw std::runtime_error(std::string("failed to add command, because \"") + cmd_name + "\" already exists");
				}
				else
				{
					plan_vec_.push_back(gait_func);
					parser_vec_.push_back(parse_func);

					cmd_id_map_.insert(std::make_pair(cmd_name, plan_vec_.size() - 1));

					std::cout << cmd_name << ":" << cmd_id_map_.at(cmd_name) << std::endl;
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
				controller_->start();
			}
		}
		auto ControlServer::Imp::stop()->void
		{
			if (is_running_)
			{
				controller_->stop();
				if (imu_)imu_->stop();
				is_running_ = false;
			}
		}
		auto ControlServer::Imp::onReceiveMsg(const aris::core::Msg &msg)->aris::core::Msg
		{
			try
			{
				std::string cmd;
				std::map<std::string, std::string> params;

				decodeMsg2Param(msg, cmd, params);
				
				if (cmd == "start")
				{
					if (is_running_)throw std::runtime_error("server already started, thus ignore command \"start\"");
					start();
					return aris::core::Msg();
				}
				if (cmd == "stop")
				{
					if (!is_running_)throw std::runtime_error("server already stopped, thus ignore command \"stop\"");
					stop();
					return aris::core::Msg();
				}
				if (cmd == "exit")
				{
					if (is_running_)stop();

					std::thread exit_callback([this]() 
					{
						aris::core::msSleep(1000);
						if (on_exit_callback_)on_exit_callback_();
					});

					exit_callback.detach();

					return aris::core::Msg();
				}
				
				if (!is_running_)throw std::runtime_error("can't execute command, because the server is not STARTED, please start it first");
				sendParam(cmd, params);

				return aris::core::Msg();
			}
			catch (std::exception &e)
			{
				std::cout << aris::core::log(e.what()) << std::endl;
				
				aris::core::Msg error_msg;
				error_msg.copy(e.what());
				return error_msg;
			}
			catch (...)
			{
				std::cout << aris::core::log("unknown exception") << std::endl;
				aris::core::Msg error_msg;
				error_msg.copy("unknown exception");
				return error_msg;
			}
		}
		auto ControlServer::Imp::decodeMsg2Param(const aris::core::Msg &msg, std::string &cmd, std::map<std::string, std::string> &params)->void
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
					throw std::runtime_error(aris::core::log("invalid message from client, please at least contain a word"));
				};
				aris::core::log(std::string("received command string:") + msg.data());

				while (inputStream >> word)
				{
					paramVector.push_back(word);
					++paramNum;
				}
			}
			else
			{
				throw std::runtime_error(aris::core::log("invalid message from client, please be sure that the command message end with char \'\\0\'"));
			}

			if (cmd_struct_map_.find(cmd) != cmd_struct_map_.end())
			{
				cmd_struct_map_.at(cmd)->root->Reset();
			}
			else
			{
				throw std::runtime_error(aris::core::log(std::string("invalid command name, server does not have command \"") + cmd + "\""));
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
						if (cmd_struct_map_.at(cmd)->shortNames.find(c) != cmd_struct_map_.at(cmd)->shortNames.end())
						{
							params.insert(make_pair(cmd_struct_map_.at(cmd)->shortNames.at(c), paramValue));
							cmd_struct_map_.at(cmd)->allParams.at(cmd_struct_map_.at(cmd)->shortNames.at(c))->Take();
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

					if (cmd_struct_map_.at(cmd)->shortNames.find(c) != cmd_struct_map_.at(cmd)->shortNames.end())
					{
						params.insert(make_pair(cmd_struct_map_.at(cmd)->shortNames.at(c), paramValue));
						cmd_struct_map_.at(cmd)->allParams.at(cmd_struct_map_.at(cmd)->shortNames.at(c))->Take();
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

					if (cmd_struct_map_.at(cmd)->allParams.find(paramName) != cmd_struct_map_.at(cmd)->allParams.end())
					{
						params.insert(make_pair(paramName, paramValue));
						cmd_struct_map_.at(cmd)->allParams.at(paramName)->Take();
					}
					else
					{
						throw std::runtime_error(std::string("invalid param: param \"") + paramName + "\" is not a valid param");
					}



					continue;
				}
			}

			AddAllDefault(cmd_struct_map_.at(cmd)->root.get(), params);

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
		auto ControlServer::Imp::sendParam(const std::string &cmd, const std::map<std::string, std::string> &params)->void
		{
			aris::core::Msg cmd_msg;

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
			else if (cmd == "fake_home")
			{
				parse_fake_home_func_(cmd, params, cmd_msg);
				if (cmd_msg.size() != sizeof(BasicFunctionParam))throw std::runtime_error("invalid msg length of parse function for fake_home");
				reinterpret_cast<BasicFunctionParam *>(cmd_msg.data())->cmd_type = ControlServer::Imp::FAKE_HOME;
			}
			else
			{
				auto cmdPair = this->cmd_id_map_.find(cmd);

				if (cmdPair == this->cmd_id_map_.end())
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
			controller_->msgPipe().sendToRT(cmd_msg);
		}
		
		auto ControlServer::Imp::tg(aris::control::EthercatController::Data &data)->int
		{
			static ControlServer::Imp *imp = ControlServer::instance().imp.get();

			// 检查是否出错 //
			static int fault_count = 0;
			auto error_motor = std::find_if(data.motion_raw_data->begin(), data.motion_raw_data->end(), [](const aris::control::EthercatMotion::RawData &data) {return data.ret < 0; });
			if (error_motor != data.motion_raw_data->end())
			{
				if (fault_count++ % 1000 == 0)
				{
					for (auto &mot_data : *data.motion_raw_data)rt_printf("%d ", mot_data.ret);

					rt_printf("\n");

					rt_printf("Some motor is in fault, now try to disable all motors\n");
					rt_printf("All commands in command queue are discarded\n");
				}
				for (auto &mot_data : *data.motion_raw_data)
				{
					mot_data.cmd = aris::control::EthercatMotion::DISABLE;
				}

				imp->cmd_num_ = 0;
				imp->count_ = 0;
				return 0;
			}
			else
			{
				fault_count = 0;
			}

			// 查看是否有新cmd //
			if (data.msg_recv)
			{
				if (imp->cmd_num_ >= CMD_POOL_SIZE)
				{
					rt_printf("cmd pool is full, thus ignore last one\n");
				}
				else
				{
					data.msg_recv->paste(imp->cmd_queue_[(imp->current_cmd_ + imp->cmd_num_) % CMD_POOL_SIZE]);
					++imp->cmd_num_;
				}
			}

			// 执行cmd queue中的cmd //
			if (imp->cmd_num_ > 0)
			{
				if (imp->execute_cmd(imp->count_, imp->cmd_queue_[imp->current_cmd_], data) == 0)
				{
					rt_printf("cmd finished, spend %d counts\n\n", imp->count_ + 1);
					imp->count_ = 0;
					imp->current_cmd_ = (imp->current_cmd_ + 1) % CMD_POOL_SIZE;
					--imp->cmd_num_;
				}
				else
				{
					if (++imp->count_ % 1000 == 0)rt_printf("execute cmd in count: %d\n", imp->count_);
				}
			}

			return 0;
		}
		auto ControlServer::Imp::execute_cmd(int count, char *cmd_param, aris::control::EthercatController::Data &data)->int
		{
			int ret;
			aris::dynamic::PlanParamBase *param = reinterpret_cast<aris::dynamic::PlanParamBase *>(cmd_param);
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
			case FAKE_HOME:
				ret = fake_home(static_cast<BasicFunctionParam &>(*param), data);
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
		auto ControlServer::Imp::enable(const BasicFunctionParam &param, aris::control::EthercatController::Data &data)->int
		{
			bool is_all_enabled = true;

			for (std::size_t i = 0; i < controller_->motionNum(); ++i)
			{
				if (param.active_motor[i])
				{
					/*判断是否已经Enable了*/
					if ((param.count != 0) && (data.motion_raw_data->operator[](i).ret == 0))
					{
						/*判断是否为第一次走到enable,否则什么也不做，这样就会继续刷上次的值*/
						if (data.motion_raw_data->operator[](i).cmd == aris::control::EthercatMotion::ENABLE)
						{
							data.motion_raw_data->operator[](i).cmd = aris::control::EthercatMotion::RUN;
							data.motion_raw_data->operator[](i).mode = aris::control::EthercatMotion::POSITION;
							data.motion_raw_data->operator[](i).target_pos = data.motion_raw_data->operator[](i).feedback_pos;
							data.motion_raw_data->operator[](i).target_vel = 0;
							data.motion_raw_data->operator[](i).target_cur = 0;
						}
					}
					else
					{
						is_all_enabled = false;
						data.motion_raw_data->operator[](i).cmd = aris::control::EthercatMotion::ENABLE;
						data.motion_raw_data->operator[](i).mode = aris::control::EthercatMotion::POSITION;

						if (param.count % 1000 == 0)
						{
							rt_printf("Unenabled motor, physical id: %d, absolute id: %d\n", this->controller_->motionAtAbs(i).phyID(), i);
						}
					}
				}
			}

			return is_all_enabled ? 0 : 1;
		};
		auto ControlServer::Imp::disable(const BasicFunctionParam &param, aris::control::EthercatController::Data &data)->int
		{
			bool is_all_disabled = true;

			for (std::size_t i = 0; i < controller_->motionNum(); ++i)
			{
				if (param.active_motor[i])
				{
					/*判断是否已经Disabled了*/
					if ((param.count != 0) && (data.motion_raw_data->operator[](i).ret == 0))
					{
						/*如果已经disable了，那么什么都不做*/
					}
					else
					{
						/*否则往下刷disable指令*/
						is_all_disabled = false;
						data.motion_raw_data->operator[](i).cmd = aris::control::EthercatMotion::DISABLE;

						if (param.count % 1000 == 0)
						{
							rt_printf("Undisabled motor, physical id: %d, absolute id: %d\n", this->controller_->motionAtAbs(i).phyID(), i);
						}
					}
				}
			}

			return is_all_disabled ? 0 : 1;
		}
		auto ControlServer::Imp::home(const BasicFunctionParam &param, aris::control::EthercatController::Data &data)->int
		{
			bool is_all_homed = true;

			for (std::size_t i = 0; i < controller_->motionNum(); ++i)
			{
				if (param.active_motor[i])
				{
					// 将电机的偏置置为0 //
					controller_->motionAtAbs(i).setPosOffset(0);
					
					// 根据返回值来判断是否走到home了 //
					if ((param.count != 0) && (data.motion_raw_data->operator[](i).ret == 0))
					{
						// 判断是否为第一次走到home,否则什么也不做，这样就会继续刷上次的值 //
						if (data.motion_raw_data->operator[](i).cmd == aris::control::EthercatMotion::HOME)
						{
							data.motion_raw_data->operator[](i).cmd = aris::control::EthercatMotion::RUN;
							data.motion_raw_data->operator[](i).target_pos = data.motion_raw_data->operator[](i).feedback_pos;
							data.motion_raw_data->operator[](i).target_vel = 0;
							data.motion_raw_data->operator[](i).target_cur = 0;
						}
					}
					else
					{
						is_all_homed = false;
						data.motion_raw_data->operator[](i).cmd = aris::control::EthercatMotion::HOME;

						if (param.count % 1000 == 0)
						{
							rt_printf("Unhomed motor, physical id: %d, absolute id: %d\n", this->controller_->motionAtAbs(i).phyID(), i);
						}
					}
				}
			}

			return is_all_homed ? 0 : 1;
		};
		auto ControlServer::Imp::fake_home(const BasicFunctionParam &param, aris::control::EthercatController::Data &data)->int
		{
			for (std::size_t i = 0; i < model_->motionPool().size(); ++i)
			{
				model_->motionPool().at(i).update();
				
				controller_->motionAtAbs(i).setPosOffset(static_cast<std::int32_t>(controller_->motionAtAbs(i).posOffset() +
					model_->motionPool().at(i).motPos()*controller_->motionAtAbs(i).pos2countRatio() - data.motion_raw_data->at(i).feedback_pos
					));
			}

			rt_printf("feedback:\n%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d\n"
				, data.motion_raw_data->at(0).feedback_pos
				, data.motion_raw_data->at(1).feedback_pos
				, data.motion_raw_data->at(2).feedback_pos
				, data.motion_raw_data->at(3).feedback_pos
				, data.motion_raw_data->at(4).feedback_pos
				, data.motion_raw_data->at(5).feedback_pos
				, data.motion_raw_data->at(6).feedback_pos
				, data.motion_raw_data->at(7).feedback_pos
				, data.motion_raw_data->at(8).feedback_pos
				, data.motion_raw_data->at(9).feedback_pos
				, data.motion_raw_data->at(10).feedback_pos
				, data.motion_raw_data->at(11).feedback_pos
				, data.motion_raw_data->at(12).feedback_pos
				, data.motion_raw_data->at(13).feedback_pos
				, data.motion_raw_data->at(14).feedback_pos
				, data.motion_raw_data->at(15).feedback_pos
				, data.motion_raw_data->at(16).feedback_pos
				, data.motion_raw_data->at(17).feedback_pos);

			rt_printf("pos_offset:\n%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d\n"
				, controller_->motionAtAbs(0).posOffset()
				, controller_->motionAtAbs(1).posOffset()
				, controller_->motionAtAbs(2).posOffset()
				, controller_->motionAtAbs(3).posOffset()
				, controller_->motionAtAbs(4).posOffset()
				, controller_->motionAtAbs(5).posOffset()
				, controller_->motionAtAbs(6).posOffset()
				, controller_->motionAtAbs(7).posOffset()
				, controller_->motionAtAbs(8).posOffset()
				, controller_->motionAtAbs(9).posOffset()
				, controller_->motionAtAbs(10).posOffset()
				, controller_->motionAtAbs(11).posOffset()
				, controller_->motionAtAbs(12).posOffset()
				, controller_->motionAtAbs(13).posOffset()
				, controller_->motionAtAbs(14).posOffset()
				, controller_->motionAtAbs(15).posOffset()
				, controller_->motionAtAbs(16).posOffset()
				, controller_->motionAtAbs(17).posOffset());

			return 0;
		};
		auto ControlServer::Imp::run(GaitParamBase &param, aris::control::EthercatController::Data &data)->int
		{
			static ControlServer::Imp *imp = ControlServer::instance().imp.get();
			
			// 获取陀螺仪传感器数据 //
			aris::sensor::SensorData<aris::sensor::ImuData> imuDataProtected;
			if (imu_) imuDataProtected = imu_->getSensorData();
			param.imu_data = &imuDataProtected.get();

			// 获取力传感器数据与电机数据 //
			param.force_data = data.force_sensor_data;
			param.motion_raw_data = data.motion_raw_data;
			param.last_motion_raw_data = data.last_motion_raw_data;
			param.motion_feedback_pos = &this->motion_pos_;

			for (std::size_t i = 0; i < data.motion_raw_data->size(); ++i)
			{
				this->motion_pos_[i] = static_cast<double>(data.motion_raw_data->at(i).feedback_pos) / controller_->motionAtAbs(i).pos2countRatio();
			}

			// 执行gait函数 //
			int ret = this->plan_vec_.at(param.gait_id).operator()(*model_.get(), param);

			// 向下写入输入位置 //
			for (std::size_t i = 0; i < controller_->motionNum(); ++i)
			{
				if (param.active_motor[i])
				{
					data.motion_raw_data->operator[](i).cmd = aris::control::EthercatMotion::RUN;
					data.motion_raw_data->operator[](i).target_pos = static_cast<std::int32_t>(model_->motionPool().at(i).motPos() * controller_->motionAtAbs(i).pos2countRatio());
				}
			}

			// 检查位置极限和速度是否连续 //
			for (std::size_t i = 0; i<imp->controller_->motionNum(); ++i)
			{
				if (data.last_motion_raw_data->at(i).cmd == aris::control::EthercatMotion::RUN)
				{
					if (param.if_check_pos_max && (data.motion_raw_data->at(i).target_pos > imp->controller_->motionAtAbs(i).maxPosCount()))
					{
						rt_printf("Motor %i's target position is bigger than its MAX permitted value in count:%d\n", i, imp->count_);
						rt_printf("The min, max and current count are:\n");
						for (std::size_t i = 0; i<imp->controller_->motionNum(); ++i)
						{
							rt_printf("%d   %d   %d\n", imp->controller_->motionAtAbs(i).minPosCount(), imp->controller_->motionAtAbs(i).maxPosCount(), data.motion_raw_data->at(i).target_pos);
						}
						rt_printf("All commands in command queue are discarded, please try to RECOVER\n");
						imp->cmd_num_ = 1;//因为这里为0退出，因此之后在tg中回递减cmd_num_,所以这里必须为1
						imp->count_ = 0;

						// 发现不连续，那么使用上一个成功的cmd，以便等待修复 //
						for (std::size_t i = 0; i < imp->controller_->motionNum(); ++i)data.motion_raw_data->operator[](i) = data.last_motion_raw_data->operator[](i);
						
						return 0;
					}

					if (param.if_check_pos_min && (data.motion_raw_data->at(i).target_pos < imp->controller_->motionAtAbs(i).minPosCount()))
					{
						rt_printf("Motor %i's target position is smaller than its MIN permitted value in count:%d\n", i, imp->count_);
						rt_printf("The min, max and current count are:\n");
						for (std::size_t i = 0; i<imp->controller_->motionNum(); ++i)
						{
							rt_printf("%d   %d   %d\n", imp->controller_->motionAtAbs(i).minPosCount(), imp->controller_->motionAtAbs(i).maxPosCount(), data.motion_raw_data->at(i).target_pos);
						}
						rt_printf("All commands in command queue are discarded, please try to RECOVER\n");
						imp->cmd_num_ = 1;//因为这里为0退出，因此之后在tg中回递减cmd_num_,所以这里必须为1
						imp->count_ = 0;

						// 发现不连续，那么使用上一个成功的cmd，以便等待修复 //
						for (std::size_t i = 0; i < imp->controller_->motionNum(); ++i)data.motion_raw_data->operator[](i) = data.last_motion_raw_data->operator[](i);

						return 0;
					}

					if (param.if_check_pos_continuous && (std::abs(data.last_motion_raw_data->at(i).target_pos - data.motion_raw_data->at(i).target_pos)>0.0012*imp->controller_->motionAtAbs(i).maxVelCount()))
					{
						rt_printf("Motor %i's target position is not continuous in count:%d\n", i, imp->count_);

						rt_printf("The input of last and this count are:\n");
						for (std::size_t i = 0; i<imp->controller_->motionNum(); ++i)
						{
							rt_printf("%d   %d\n", data.last_motion_raw_data->at(i).target_pos, data.motion_raw_data->at(i).target_pos);
						}

						rt_printf("All commands in command queue are discarded, please try to RECOVER\n");
						imp->cmd_num_ = 1;//因为这里为0退出，因此之后在tg中回递减cmd_num_,所以这里必须为1
						imp->count_ = 0;

						// 发现不连续，那么使用上一个成功的cmd，以便等待修复 //
						for (std::size_t i = 0; i < imp->controller_->motionNum(); ++i)data.motion_raw_data->operator[](i) = data.last_motion_raw_data->operator[](i);

						return 0;
					}
				}
			}

			return ret;
		}

		ControlServer &ControlServer::instance()
		{
			static ControlServer instance;
			return std::ref(instance);
		}
		ControlServer::ControlServer() :imp(new Imp(this)) {}
		ControlServer::~ControlServer() {}
		auto ControlServer::createModel(dynamic::Model *model_)->void
		{
			if (imp->model_)throw std::runtime_error("control sever can't create model because it already has one");

			imp->model_.reset(model_);
		}
		auto ControlServer::loadXml(const char *fileName)->void
		{
			aris::core::XmlDocument doc;

			if (doc.LoadFile(fileName) != 0)
			{
				throw std::logic_error((std::string("could not open file:") + std::string(fileName)));
			}

			loadXml(doc);
		}
		auto ControlServer::loadXml(const aris::core::XmlDocument &xmlDoc)->void {	imp->loadXml(xmlDoc);}
		auto ControlServer::model()->dynamic::Model&
		{
			return std::ref(*imp->model_.get());
		};
		auto ControlServer::controller()->control::EthercatController&
		{
			return std::ref(*imp->controller_);
		}
		auto ControlServer::addCmd(const std::string &cmd_name, const ParseFunc &parse_func, const aris::dynamic::PlanFunc &gait_func)->void
		{
			imp->addCmd(cmd_name, parse_func, gait_func);
		}
		auto ControlServer::open()->void 
		{
			for (;;)
			{
				try
				{
					imp->server_socket_.startServer(imp->server_socket_port_.c_str());
					break;
				}
				catch (aris::core::Socket::StartServerError &e)
				{
					std::cout << e.what() << std::endl << "will try to restart server socket in 1s" << std::endl;
					aris::core::msSleep(1000);
				}
			}
			std::cout << aris::core::log("server open successful") << std::endl;
		};
		auto ControlServer::close()->void 
		{
			imp->server_socket_.stop();
		};
		auto ControlServer::setOnExit(std::function<void(void)> callback_func)->void
		{
			this->imp->on_exit_callback_ = callback_func;
		}
	}
}








