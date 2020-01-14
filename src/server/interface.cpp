#include <cstring>
#include <thread>
#include <algorithm>
#include <memory>
#include <cinttypes>
#include <queue>
#include <unordered_map>

#include <aris/core/core.hpp>
#include <aris/control/control.hpp>

#include "aris/server/interface.hpp"
#include "aris/server/control_server.hpp"

#include "json.hpp"
#include "fifo_map.hpp"

namespace aris::server
{
	auto InterfaceRoot::saveXml(aris::core::XmlElement &xml_ele) const->void
	{
		auto ins = doc_.RootElement()->DeepClone(xml_ele.GetDocument());
		xml_ele.Parent()->InsertAfterChild(&xml_ele, ins);
		xml_ele.Parent()->DeleteChild(&xml_ele);
	}
	auto InterfaceRoot::loadXml(const aris::core::XmlElement &xml_ele)->void
	{
		doc_.Clear();
		auto root = xml_ele.DeepClone(&doc_);
		doc_.InsertEndChild(root);
	}
	Interface::Interface(const std::string &name) :Object(name) {}

	auto parse_ret_value(std::vector<std::pair<std::string, std::any>> &ret)->std::string
	{
		nlohmann::json js;
		
		for (auto &key_value : ret)
		{
#define ARIS_SET_TYPE(TYPE) if (auto value = std::any_cast<TYPE>(&key_value.second)) js[key_value.first] = *value; else

			ARIS_SET_TYPE(bool)
			ARIS_SET_TYPE(int)
			ARIS_SET_TYPE(double)
			ARIS_SET_TYPE(std::string)
			ARIS_SET_TYPE(std::vector<bool>)
			ARIS_SET_TYPE(std::vector<int>)
			ARIS_SET_TYPE(std::vector<double>)
			ARIS_SET_TYPE(std::vector<std::string>)
			{
				ARIS_COUT << "unrecognized return value" << std::endl;
			}

#undef ARIS_SET_TYPE
		}
		
		return  js.dump(2);
	}
	auto onReceivedMsg(aris::core::Socket *socket, aris::core::Msg &msg)->int
	{
		ARIS_COUT << "received " << std::endl;
		
		auto msg_data = std::string_view(msg.data(), msg.size());

		LOG_INFO << "receive cmd:"
			<< msg.header().msg_size_ << "&"
			<< msg.header().msg_id_ << "&"
			<< msg.header().msg_type_ << "&"
			<< msg.header().reserved1_ << "&"
			<< msg.header().reserved2_ << "&"
			<< msg.header().reserved3_ << ":"
			<< msg_data << std::endl;

		try
		{
			aris::server::ControlServer::instance().executeCmdInCmdLine(std::string_view(msg.data(), msg.size()), [socket, msg](aris::plan::Plan &plan)->void
			{
				// make return msg
				aris::core::Msg ret_msg(msg);

				// only copy if it is a str
				if (auto str = std::any_cast<std::string>(&plan.ret()))
				{
					ret_msg.copy(*str);
				}
				else if (auto js = std::any_cast<std::vector<std::pair<std::string, std::any>>>(&plan.ret()))
				{
					js->push_back(std::make_pair<std::string, std::any>("return_code", plan.retCode()));
					js->push_back(std::make_pair<std::string, std::any>("return_message", std::string(plan.retMsg())));
					ret_msg.copy(parse_ret_value(*js));
				}

				// return back to source
				try
				{
					socket->sendMsg(ret_msg);
				}
				catch (std::exception &e)
				{
					ARIS_COUT << e.what() << std::endl;
					LOG_ERROR << e.what() << std::endl;
				}
			});
		}
		catch (std::exception &e)
		{
			std::vector<std::pair<std::string, std::any>> ret_pair;
			ret_pair.push_back(std::make_pair<std::string, std::any>("return_code", int(aris::plan::Plan::PARSE_EXCEPTION)));
			ret_pair.push_back(std::make_pair<std::string, std::any>("return_message", std::string(e.what())));
			std::string ret_str = parse_ret_value(ret_pair);

			ARIS_COUT << ret_str << std::endl;
			LOG_ERROR << ret_str << std::endl;

			try
			{
				aris::core::Msg m = msg;
				m.copy(ret_str);
				socket->sendMsg(m);
			}
			catch (std::exception &e)
			{
				ARIS_COUT << e.what() << std::endl;
				LOG_ERROR << e.what() << std::endl;
			}
		}

		return 0;
	}
	auto onReceivedConnection(aris::core::Socket *sock, const char *ip, int port)->int
	{
		ARIS_COUT << "socket receive connection" << std::endl;
		LOG_INFO << "socket receive connection:\n"
			<< std::setw(aris::core::LOG_SPACE_WIDTH) << "|" << "  ip:" << ip << "\n"
			<< std::setw(aris::core::LOG_SPACE_WIDTH) << "|" << "port:" << port << std::endl;
		return 0;
	}
	auto onLoseConnection(aris::core::Socket *socket)->int
	{
		ARIS_COUT << "socket lose connection" << std::endl;
		LOG_INFO << "socket lose connection" << std::endl;
		for (;;)
		{
			try
			{
				socket->startServer(socket->port());
				break;
			}
			catch (std::runtime_error &e)
			{
				ARIS_COUT << e.what() << std::endl << "will try to restart server socket in 1s" << std::endl;
				LOG_ERROR << e.what() << std::endl << "will try to restart server socket in 1s" << std::endl;
				std::this_thread::sleep_for(std::chrono::seconds(1));
			}
		}
		ARIS_COUT << "socket restart successful" << std::endl;
		LOG_INFO << "socket restart successful" << std::endl;

		return 0;
	}
	auto WebInterface::open()->void { sock_->startServer(); }
	auto WebInterface::close()->void { sock_->stop(); }
	auto WebInterface::loadXml(const aris::core::XmlElement &xml_ele)->void
	{
		Interface::loadXml(xml_ele);
		this->sock_ = findOrInsertType<aris::core::Socket>("socket", "", "5866", aris::core::Socket::WEB);

		sock_->setOnReceivedMsg(onReceivedMsg);
		sock_->setOnReceivedConnection(onReceivedConnection);
		sock_->setOnLoseConnection(onLoseConnection);
	}
	WebInterface::WebInterface(const std::string &name, const std::string &port, aris::core::Socket::TYPE type):Interface(name)
	{
		sock_ = &add<aris::core::Socket>("socket", "", port, type);
		
		sock_->setOnReceivedMsg(onReceivedMsg);
		sock_->setOnReceivedConnection(onReceivedConnection);
		sock_->setOnLoseConnection(onLoseConnection);
	}
#define ARIS_PRO_COUT ARIS_COUT << "pro "
	struct ProgramWebInterface::Imp
	{
		aris::core::CommandParser command_parser_;
		aris::core::LanguageParser language_parser_;
		aris::core::Calculator calculator_;
		std::thread auto_thread_;
		std::atomic<int> current_line_{0};
		bool is_auto_mode_{ false };

		std::string last_error_;
		int last_error_code_{ 0 }, last_error_line_{0};

		std::atomic_bool is_stop_{ false }, is_pause_{ false };

		std::function<int(aris::core::Socket*, aris::core::Msg &)> onReceiveMsg_;
		std::function<int(aris::core::Socket*, const char *data, int size)> onReceiveConnection_;
		std::function<int(aris::core::Socket*)> onLoseConnection_;
	};
	auto ProgramWebInterface::open()->void { sock_->startServer(); }
	auto ProgramWebInterface::close()->void { sock_->stop(); }
	auto ProgramWebInterface::loadXml(const aris::core::XmlElement &xml_ele)->void
	{
		Interface::loadXml(xml_ele);
		this->sock_ = findOrInsertType<aris::core::Socket>("socket", "", "5866", aris::core::Socket::WEB);

		sock_->setOnReceivedMsg(imp_->onReceiveMsg_);
		sock_->setOnReceivedConnection(imp_->onReceiveConnection_);
		sock_->setOnLoseConnection(imp_->onLoseConnection_);
	}
	auto ProgramWebInterface::lastError()->std::string { return imp_->last_error_; }
	auto ProgramWebInterface::lastErrorCode()->int { return imp_->last_error_code_; }
	auto ProgramWebInterface::lastErrorLine()->int { return imp_->last_error_line_; }
	auto ProgramWebInterface::isAutoRunning()->bool { return imp_->auto_thread_.joinable(); }
	auto ProgramWebInterface::isAutoMode()->bool { return imp_->is_auto_mode_; }
	auto ProgramWebInterface::currentLine()->int { return imp_->current_line_.load(); }
	ProgramWebInterface::ProgramWebInterface(const std::string &name, const std::string &port, aris::core::Socket::TYPE type) :Interface(name), imp_(new Imp)
	{
		aris::core::Command program;
		program.loadXmlStr(
			"<Command name=\"program\">"
			"	<Param name=\"set_auto\"/>"
			"	<Param name=\"set_manual\"/>"
			"	<Param name=\"content\"/>"
			"	<Param name=\"goto\" default=\"2\"/>"
			"	<Param name=\"goto_main\"/>"
			"	<Param name=\"start\"/>"
			"	<Param name=\"pause\"/>"
			"	<Param name=\"stop\"/>"
			"	<Param name=\"clear_error\"/>"
			"	<Param name=\"forward\"/>"
			"</Command>");
		imp_->command_parser_.commandPool().add<aris::core::Command>(program);
		imp_->command_parser_.init();

		sock_ = &add<aris::core::Socket>("socket", "", port, type);

		imp_->onReceiveMsg_ = [this](aris::core::Socket *socket, aris::core::Msg &msg)->int
		{
			auto send_ret = [socket](aris::core::Msg &ret_msg)->void
			{
				try
				{
					socket->sendMsg(ret_msg);
				}
				catch (std::exception &e)
				{
					ARIS_COUT << e.what() << std::endl;
					LOG_ERROR << e.what() << std::endl;
				}
			};
			auto send_code_and_msg = [send_ret, msg](int code, const std::string& ret_msg_str)
			{
				nlohmann::json js;
				js["return_code"] = code;///////////////////////////////////////////////////////////
				js["return_message"] = ret_msg_str;

				aris::core::Msg ret_msg = msg;
				ret_msg.copy(js.dump(-1));

				ARIS_PRO_COUT << "---" << ret_msg.toString() << std::endl;

				send_ret(ret_msg);
			};

			auto msg_data = std::string_view(msg.data(), msg.size());

			LOG_INFO << "receive cmd:"
				<< msg.header().msg_size_ << "&"
				<< msg.header().msg_id_ << "&"
				<< msg.header().msg_type_ << "&"
				<< msg.header().reserved1_ << "&"
				<< msg.header().reserved2_ << "&"
				<< msg.header().reserved3_ << ":"
				<< msg_data << std::endl;

			std::string_view cmd;
			std::map<std::string_view, std::string_view> params;
			try { std::tie(cmd, params) = imp_->command_parser_.parse(msg_data);}
			catch (std::exception &) {};

			if (cmd == "program")
			{
				ARIS_PRO_COUT <<"---"<< msg_data << std::endl;
				LOG_INFO << "pro ---" << msg_data << std::endl;
				
				for (auto &[param, value] : params)
				{
					if (param == "set_auto")
					{
						imp_->is_auto_mode_ = true;
						send_code_and_msg(0, "");
						return 0;
					}
					else if (param == "set_manual")
					{
						if (isAutoRunning())
						{
							send_code_and_msg(aris::plan::Plan::PROGRAM_EXCEPTION, "can not set manual when auto running");
							return 0;
						}
						else
						{
							imp_->is_auto_mode_ = false;
							send_code_and_msg(0, "");
							return 0;
						}
					}
					else if (param == "content")
					{
						if (isAutoRunning())
						{
							send_code_and_msg(aris::plan::Plan::PROGRAM_EXCEPTION, "can not set content when auto running");
							return 0;
						}
						else
						{
							imp_->last_error_.clear();
							imp_->last_error_code_ = 0;
							imp_->last_error_line_ = 0;
							
							auto begin_pos = value.find("{");
							auto end_pos = value.rfind("}");
							auto cmd_str = value.substr(begin_pos + 1, end_pos - 1 - begin_pos);

							try
							{
								imp_->calculator_ = aris::server::ControlServer::instance().model().calculator();
								auto &c = imp_->calculator_;
								
								imp_->language_parser_.setProgram(cmd_str);
								imp_->language_parser_.parseLanguage();

								for (auto &str : imp_->language_parser_.varPool())
								{
									auto cut_str = [](std::string_view &input, const char *c)->std::string_view
									{
										// 此时c中字符是或的关系 //
										auto point = input.find_first_of(c);
										auto ret = input.substr(0, point);
										input = point == std::string::npos ? std::string_view() : input.substr(point);
										return ret;
									};
									auto trim_left = [](std::string_view &input, const char *c)->std::string_view
									{
										auto point = input.find_first_not_of(c);
										return point == std::string::npos ? std::string_view() : input.substr(point, std::string::npos);
									};
									
									std::string_view input = str;
									if (auto var = cut_str(input, " "); cmd.empty())THROW_FILE_LINE("invalid command string: please at least contain a word");
									input = trim_left(input, " ");

									auto type = cut_str(input, " ");
									input = trim_left(input, " ");

									auto name = cut_str(input, " =");
									input = trim_left(input, " =");

									auto value = input;
									c.addVariable(name, type, c.calculateExpression(std::string(type) + "(" + std::string(value) + ")").second);
								}

								send_code_and_msg(0, std::string());
								return 0;
							}
							catch (std::exception &e)
							{
								ARIS_COUT << e.what() << std::endl;
								LOG_ERROR << "pro ---" << msg_data << std::endl;
								send_code_and_msg(aris::plan::Plan::PROGRAM_EXCEPTION, e.what());
								return 0;
							}
						}
					}
					else if (param == "goto")
					{
						if (!isAutoMode())
						{
							send_code_and_msg(aris::plan::Plan::PROGRAM_EXCEPTION, "can not goto in manual mode");
							return 0;
						}
						else if (isAutoRunning())
						{
							send_code_and_msg(aris::plan::Plan::PROGRAM_EXCEPTION, "can not goto when running");
							return 0;
						}
						else
						{
							imp_->language_parser_.gotoLine(std::stoi(std::string(value)));
							send_code_and_msg(0, "");
							return 0;
						}
					}
					else if (param == "goto_main")
					{
						if (!isAutoMode())
						{
							send_code_and_msg(aris::plan::Plan::PROGRAM_EXCEPTION, "can not goto in manual mode");
							return 0;
						}
						else if (isAutoRunning())
						{
							send_code_and_msg(aris::plan::Plan::PROGRAM_EXCEPTION, "can not goto when running");
							return 0;
						}
						else
						{
							imp_->language_parser_.gotoMain();
							imp_->current_line_.store(imp_->language_parser_.currentLine());
							send_code_and_msg(0, "");
							return 0;
						}
					}
					else if (param == "forward")
					{
					}
					else if (param == "start")
					{
						if (!isAutoMode())
						{
							send_code_and_msg(aris::plan::Plan::PROGRAM_EXCEPTION, "can not start program in manual mode");
							return 0;
						}
						else if (isAutoRunning())
						{
							imp_->is_pause_.store(false);
							send_code_and_msg(0, "");
							return 0;
						}
						else if (lastErrorCode())
						{
							send_code_and_msg(lastErrorCode(), lastError());
						}
						else
						{
							imp_->is_pause_.store(false);
							imp_->is_stop_.store(false);

							imp_->auto_thread_ = std::thread([&]()->void
							{
								std::swap(imp_->calculator_, aris::server::ControlServer::instance().model().calculator());
								auto &c = aris::server::ControlServer::instance().model().calculator();

								auto &cs = aris::server::ControlServer::instance();
								imp_->current_line_.store(imp_->language_parser_.currentLine());

								for (std::atomic_bool is_error{ false }; (!is_error.load()) && (!imp_->language_parser_.isEnd());)
								{
									if (imp_->is_stop_.load() == true)break;
									if (imp_->is_pause_.load() == true)
									{
										std::this_thread::sleep_for(std::chrono::milliseconds(1));
										continue;
									}

									ARIS_PRO_COUT << imp_->language_parser_.currentLine() << "---" << imp_->language_parser_.currentCmd() << std::endl;
									LOG_INFO << "pro " << imp_->language_parser_.currentLine() << "---" << imp_->language_parser_.currentCmd() << std::endl;

									////////////  to check if is set ////////////////////
									auto cut_str = [](std::string_view &input, const char *c)->std::string_view
									{
										// 此时c中字符是或的关系 //
										auto point = input.find_first_of(c);
										auto ret = input.substr(0, point);
										input = point == std::string::npos ? std::string_view() : input.substr(point);
										return ret;
									};
									auto trim_left = [](std::string_view &input, const char *c)->std::string_view
									{
										auto point = input.find_first_not_of(c);
										return point == std::string::npos ? std::string_view() : input.substr(point, std::string::npos);
									};
									std::string_view whole_cmd = imp_->language_parser_.currentCmd();
									auto cmd = cut_str(whole_cmd, " ");
									if (cmd == "set")
									{
										try
										{
											cs.waitForAllCollection();
											c.calculateExpression(whole_cmd);
											imp_->language_parser_.forward();
											imp_->current_line_.store(imp_->language_parser_.currentLine());
										}
										catch (std::exception &e)
										{
											imp_->last_error_code_ = aris::plan::Plan::PROGRAM_EXCEPTION;
											imp_->last_error_ = e.what();
											imp_->last_error_line_ = imp_->language_parser_.currentLine();
											ARIS_PRO_COUT << imp_->last_error_line_ << "---err_code:" << imp_->last_error_code_ << "  err_msg:" << imp_->last_error_ << std::endl;
											LOG_ERROR << "pro " << imp_->last_error_line_ << "---err_code:" << imp_->last_error_code_ << "  err_msg:" << imp_->last_error_ << std::endl;
											is_error.store(true);
										}

										continue;
									}
									///////////////////////////////// check finished ///////////////////////////////

									if (imp_->language_parser_.isCurrentLineKeyWord())
									{
										cs.waitForAllCollection();

										auto cmd_name = imp_->language_parser_.currentCmd().substr(0, imp_->language_parser_.currentCmd().find_first_of(" \t\n\r\f\v("));
										auto cmd_value = imp_->language_parser_.currentCmd().find_first_of(" \t\n\r\f\v(") == std::string::npos ? "" :
											imp_->language_parser_.currentCmd().substr(imp_->language_parser_.currentCmd().find_first_of(" \t\n\r\f\v("), std::string::npos);

										if (cmd_name == "if" || cmd_name == "while")
										{
											try 
											{
												auto ret = c.calculateExpression(cmd_value);

												if (auto ret_double = std::any_cast<double>(&ret.second))
												{
													imp_->language_parser_.forward(*ret_double != 0.0);
													imp_->current_line_.store(imp_->language_parser_.currentLine());
												}
												else if (auto ret_mat = std::any_cast<aris::core::Matrix>(&ret.second))
												{
													imp_->language_parser_.forward(ret_mat->toDouble() != 0.0);
													imp_->current_line_.store(imp_->language_parser_.currentLine());
												}
												else
												{
													
													imp_->last_error_code_ = aris::plan::Plan::PROGRAM_EXCEPTION;
													imp_->last_error_ = "invalid expresion";
													imp_->last_error_line_ = imp_->language_parser_.currentLine();
													ARIS_PRO_COUT << imp_->last_error_line_ << "---err_code:" << imp_->last_error_code_ << "  err_msg:" << imp_->last_error_ << std::endl;
													break;
												}
											}
											catch (std::exception &e)
											{
												imp_->last_error_code_ = -10;
												imp_->last_error_ = e.what();
												imp_->last_error_line_ = imp_->language_parser_.currentLine();
												ARIS_PRO_COUT << imp_->last_error_line_ << "---err_code:" << imp_->last_error_code_ << "  err_msg:" << imp_->last_error_ << std::endl;
												break;
											}
										}
										else
										{
											imp_->language_parser_.forward();
											imp_->current_line_.store(imp_->language_parser_.currentLine());
										}
									}
									else if (imp_->language_parser_.isCurrentLineFunction())
									{
										cs.waitForAllCollection();
										imp_->language_parser_.forward();
										imp_->current_line_.store(imp_->language_parser_.currentLine());
									}
									else
									{
										auto cmd = imp_->language_parser_.currentCmd();
										auto current_line = imp_->language_parser_.currentLine();
										imp_->language_parser_.forward();
										auto next_line = imp_->language_parser_.currentLine();
										auto ret = cs.executeCmdInCmdLine(cmd, [&, current_line, next_line](aris::plan::Plan &plan)->void
										{
											imp_->current_line_.store(next_line);

											if (plan.retCode())
											{
												imp_->last_error_code_ = plan.retCode();
												imp_->last_error_ = plan.retMsg();
												imp_->last_error_line_ = current_line;
												ARIS_PRO_COUT << imp_->last_error_line_ << "---" << plan.cmdId() << "---err_code:" << imp_->last_error_code_ << "  err_msg:" << imp_->last_error_ << std::endl;
												LOG_ERROR << "pro " << imp_->last_error_line_ << "---" << plan.cmdId() << "---err_code:" << imp_->last_error_code_ << "  err_msg:" << imp_->last_error_ << std::endl;
												is_error.store(true);
											}
										});
										ARIS_PRO_COUT << current_line << "---" << ret->cmdId() << "---" << ret->cmdString() << std::endl;
										LOG_INFO << "pro " << current_line << "---" << ret->cmdId() << "---" << ret->cmdString() << std::endl;
									}
								}

								cs.waitForAllCollection();
								imp_->current_line_.store(imp_->language_parser_.currentLine());

								std::swap(imp_->calculator_, aris::server::ControlServer::instance().model().calculator());
								ARIS_PRO_COUT << "---" << (imp_->is_stop_.load() ? "program stopped" : "program finished") << std::endl;
								LOG_INFO << "pro " << "---" << (imp_->is_stop_.load() ? "program stopped" : "program finished") << std::endl;

								while (!imp_->auto_thread_.joinable());// for windows bug:if thread init too fast, it may fail
								imp_->auto_thread_.detach();
							});
							send_code_and_msg(0, "");
							return 0;
						}
					}
					else if (param == "stop")
					{
						if (!isAutoMode())
						{
							send_code_and_msg(aris::plan::Plan::PROGRAM_EXCEPTION, "can not stop program in manual mode");
							return 0;
						}
						else
						{
							imp_->is_stop_.store(true);
							send_code_and_msg(0, "");
							return 0;
						}
					}
					else if (param == "pause")
					{
						if (!isAutoMode())
						{
							send_code_and_msg(aris::plan::Plan::PROGRAM_EXCEPTION, "can not stop program in manual mode");
							return 0;
						}
						else if (!isAutoRunning())
						{
							send_code_and_msg(aris::plan::Plan::PROGRAM_EXCEPTION, "can not stop program when not running");
							return 0;
						}
						else
						{
							imp_->is_pause_.store(true);
							send_code_and_msg(0, "");
							return 0;
						}
					}
					else if (param == "clear_error")
					{
						if (isAutoRunning())
						{
							send_code_and_msg(aris::plan::Plan::PROGRAM_EXCEPTION, "can not clear error when running");
							return 0;
						}
						else
						{
							imp_->last_error_.clear();
							imp_->last_error_code_ = 0;
							imp_->last_error_line_ = 0;
							send_code_and_msg(0, "");
							return 0;
						}
					}
					else
					{
						send_code_and_msg(aris::plan::Plan::PROGRAM_EXCEPTION, "invalid program option");
						return 0;
					}
				}
			}
			else
			{
				aris::server::ControlServer::instance().executeCmdInCmdLine(std::string_view(msg.data(), msg.size()), [socket, msg, send_ret](aris::plan::Plan &plan)->void
				{
					// make return msg
					aris::core::Msg ret_msg(msg);
					// only copy if it is a str
					if (auto js = std::any_cast<std::vector<std::pair<std::string, std::any>>>(&plan.ret()))
					{
						js->push_back(std::make_pair<std::string, std::any>("return_code", plan.retCode()));
						js->push_back(std::make_pair<std::string, std::any>("return_message", std::string(plan.retMsg())));
						ret_msg.copy(aris::server::parse_ret_value(*js));
					}
					else
					{
						std::vector<std::pair<std::string, std::any>> ret_js;
						ret_js.push_back(std::make_pair<std::string, std::any>("return_code", plan.retCode()));
						ret_js.push_back(std::make_pair<std::string, std::any>("return_message", std::string(plan.retMsg())));
						ret_msg.copy(aris::server::parse_ret_value(ret_js));
					}
					// return back to source
					send_ret(ret_msg);
				});
			}

			return 0;
		};
		imp_->onReceiveConnection_ = [](aris::core::Socket *sock, const char *ip, int port)->int
		{
			ARIS_COUT << "socket receive connection" << std::endl;
			LOG_INFO << "socket receive connection:\n"
				<< std::setw(aris::core::LOG_SPACE_WIDTH) << "|" << "  ip:" << ip << "\n"
				<< std::setw(aris::core::LOG_SPACE_WIDTH) << "|" << "port:" << port << std::endl;
			return 0;
		};
		imp_->onLoseConnection_ = [](aris::core::Socket *socket)->int
		{
			ARIS_COUT << "socket lose connection" << std::endl;
			LOG_INFO << "socket lose connection" << std::endl;
			for (;;)
			{
				try
				{
					socket->startServer(socket->port());
					break;
				}
				catch (std::runtime_error &e)
				{
					ARIS_COUT << e.what() << std::endl << "will try to restart server socket in 1s" << std::endl;
					LOG_ERROR << e.what() << std::endl << "will try to restart server socket in 1s" << std::endl;
					std::this_thread::sleep_for(std::chrono::seconds(1));
				}
			}
			ARIS_COUT << "socket restart successful" << std::endl;
			LOG_INFO << "socket restart successful" << std::endl;

			return 0;
		};

		sock_->setOnReceivedMsg(imp_->onReceiveMsg_);
		sock_->setOnReceivedConnection(imp_->onReceiveConnection_);
		sock_->setOnLoseConnection(imp_->onLoseConnection_);
	}
	ProgramWebInterface::ProgramWebInterface(ProgramWebInterface && other) = default;
	ProgramWebInterface& ProgramWebInterface::operator=(ProgramWebInterface&& other) = default;

	auto GetInfo::prepareNrt()->void
	{
		auto &cs = *controlServer();
		auto &inter = dynamic_cast<aris::server::ProgramWebInterface&>(cs.interfacePool().at(0));

		std::vector<std::pair<std::string, std::any>> ret;
		ret.push_back(std::make_pair(std::string("cs_err_code"), std::make_any<int>(cs.errorCode())));
		ret.push_back(std::make_pair(std::string("cs_err_msg"), std::make_any<std::string>(cs.errorMsg())));
		ret.push_back(std::make_pair(std::string("pro_err_code"), std::make_any<int>(inter.lastErrorCode())));
		ret.push_back(std::make_pair(std::string("pro_err_msg"), std::make_any<std::string>(inter.lastError())));
		ret.push_back(std::make_pair(std::string("pro_err_line"), std::make_any<int>(inter.lastErrorLine())));
		ret.push_back(std::make_pair(std::string("auto_mode"), std::make_any<int>(inter.isAutoMode())));
		ret.push_back(std::make_pair(std::string("auto_run"), std::make_any<int>(inter.isAutoRunning())));
		ret.push_back(std::make_pair(std::string("line"), std::make_any<int>(inter.currentLine())));

		auto ret_str = parse_ret_value(ret);
		std::copy(ret_str.begin(), ret_str.end(), const_cast<char*>(this->retMsg()));

		this->option() = aris::plan::Plan::NOT_RUN_EXECUTE_FUNCTION | aris::plan::Plan::NOT_RUN_COLLECT_FUNCTION | aris::plan::Plan::NOT_PRINT_CMD_INFO;
	}
}


//////////////////////////////////////////////////////////////////////////////////////////////////////
#include "mongoose.h"

#ifdef WIN32
#undef min
#undef max
#endif
/////////////////////////////////////////////////////////////////////////////////////////////////////
namespace aris::server
{
	template<class K, class V, class dummy_compare, class A>
	using my_workaround_fifo_map = nlohmann::fifo_map<K, V, nlohmann::fifo_map_compare<K>, A>;
	using my_json = nlohmann::basic_json<my_workaround_fifo_map>;
	
	struct HttpInterface::Imp
	{
		std::thread http_thread_;
		std::string port_;
		std::string document_root_;
		
		struct mg_mgr mgr;
		struct mg_connection *nc;
		struct mg_bind_opts bind_opts;
		struct mg_serve_http_opts s_http_server_opts;
		const char *err_str;

		std::mutex mu_running_;
		std::atomic_bool is_running_{ false };
	};
	auto HttpInterface::open()->void
	{
		std::unique_lock<std::mutex> running_lck(imp_->mu_running_);
		
		aris::core::XmlDocument doc;
		std::filesystem::path interface_path(imp_->document_root_);
		interface_path  = interface_path / "../robot/interface.xml";
		doc.LoadFile(interface_path.string().c_str());

		my_json js;
		for (auto ele = doc.RootElement()->FirstChildElement(); ele; ele = ele->NextSiblingElement())
		{
			if (ele->Name() == std::string("Dashboard"))
			{
				my_json js1;
				js1["name"] = std::string(ele->Attribute("name"));
				js1["editable"] = std::string(ele->Attribute("editable")) == "true";
				js1["i"] = std::string(ele->Attribute("id"));
				js1["cells"] = std::vector<std::string>();
				for (auto e1 = ele->FirstChildElement(); e1; e1 = e1->NextSiblingElement())
				{
					my_json j2;//{"name":"Ethercat配置","type":"EthercatConfiguration","i":"EMlxGXxpwDGgz","w":48,"h":23,"x":0,"y":0,"options":"{}"}
					j2["name"] = e1->Attribute("name");
					j2["type"] = e1->Attribute("type");
					j2["i"] = e1->Attribute("id");
					j2["w"] = e1->IntAttribute("width");
					j2["h"] = e1->IntAttribute("height");
					j2["x"] = e1->IntAttribute("x");
					j2["y"] = e1->IntAttribute("y");
					j2["options"] = e1->Attribute("options");
					js1["cells"].push_back(j2);
				}
				js["dashboards"].push_back(js1);
			}
			else if (ele->Name() == std::string("WebSocket"))
			{
				js["ws"]["url"] = ele->Attribute("url");
				js["ws"]["commandSendInterval"] = ele->IntAttribute("commandSendInterval");
				js["ws"]["commandSendDelay"] = ele->IntAttribute("commandSendDelay");
				js["ws"]["getInterval"] = ele->IntAttribute("getInterval");
				js["ws"]["unityUpdateInterval"] = ele->IntAttribute("unityUpdateInterval");
			}
			else if (ele->Name() == std::string("LayoutConfig"))
			{
				js["layoutConfig"]["cols"] = ele->IntAttribute("cols");
				js["layoutConfig"]["rowHeight"] = ele->IntAttribute("rowHeight");
				js["layoutConfig"]["margin"] = ele->IntAttribute("margin");
				js["layoutConfig"]["containerPadding"] = ele->IntAttribute("containerPadding");
				js["layoutConfig"]["theme"] = ele->Attribute("theme");
			}
		}

		auto str = js.dump(-1);
		std::ofstream f;
		std::filesystem::create_directories("C:\\Users\\py033\\Desktop\\distUI_darkColor_1208\\www\\api\\config\\");
		f.open("C:\\Users\\py033\\Desktop\\distUI_darkColor_1208\\www\\api\\config\\interface");
		f << str;
		f.close();

		if (imp_->is_running_.exchange(true) == false)
		{
			mg_mgr_init(&imp_->mgr, NULL);

			// Set HTTP server options //
			memset(&imp_->bind_opts, 0, sizeof(imp_->bind_opts));
			imp_->bind_opts.error_string = &imp_->err_str;

			imp_->nc = mg_bind_opt(&imp_->mgr, imp_->port_.c_str(), [](struct mg_connection *nc, int ev, void *ev_data)
			{
				struct http_message *hm = (struct http_message *) ev_data;

				switch (ev) {
				case MG_EV_HTTP_REQUEST:
				{
					/*std::cout << std::string(hm->method.p, hm->method.len) <<":"<< std::string(hm->uri.p, hm->uri.len) << std::endl;
					
					if (std::string(hm->method.p, hm->method.len) == "POST")
					{

					}
					else if (std::string(hm->method.p, hm->method.len) == "PUT")
					{
					}
					else if (std::string(hm->method.p, hm->method.len) == "DELETE")
					{
					}
					else
					{
						
					}*/
					mg_serve_http(nc, hm, *reinterpret_cast<mg_serve_http_opts*>(nc->user_data));
				}
				default:

					break;
				}
			}, imp_->bind_opts);
			if (imp_->nc == NULL) {
				fprintf(stderr, "Error starting server on http\n");
				exit(1);
			}

			mg_set_protocol_http_websocket(imp_->nc);
			std::memset(&imp_->s_http_server_opts, 0, sizeof(mg_serve_http_opts));
			imp_->s_http_server_opts.document_root = imp_->document_root_.c_str();
			imp_->s_http_server_opts.enable_directory_listing = "yes";
			imp_->nc->user_data = &imp_->s_http_server_opts;
			imp_->http_thread_ = std::thread([this]()
			{
				for (; imp_->is_running_; ) { mg_mgr_poll(&imp_->mgr, 1000); }
				mg_mgr_free(&imp_->mgr);
			});
		}
	}
	auto HttpInterface::close()->void 
	{
		std::unique_lock<std::mutex> running_lck(imp_->mu_running_);
		if (imp_->is_running_.exchange(false) == true){	imp_->http_thread_.join();	}
	}
	auto HttpInterface::saveXml(aris::core::XmlElement &xml_ele) const->void
	{
		Interface::saveXml(xml_ele);
		xml_ele.SetAttribute("document_root", imp_->document_root_.c_str());
		xml_ele.SetAttribute("port", imp_->port_.c_str());
	}
	auto HttpInterface::loadXml(const aris::core::XmlElement &xml_ele)->void
	{
		Interface::loadXml(xml_ele);
		imp_->document_root_ = Object::attributeString(xml_ele, "document_root", "./");
		imp_->port_ = Object::attributeString(xml_ele, "port", "8000");
	}
	HttpInterface::~HttpInterface() = default;
	HttpInterface::HttpInterface(HttpInterface && other) = default;
	HttpInterface& HttpInterface::operator=(HttpInterface&& other) = default;
	HttpInterface::HttpInterface(const std::string &name, const std::string &port, const std::string &document_root) :Interface(name), imp_(new Imp)
	{
		imp_->document_root_ = document_root;
		imp_->port_ = port;
	}
}