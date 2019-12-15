#include <cstring>
#include <thread>
#include <algorithm>
#include <memory>
#include <cinttypes>
#include <queue>

#include <aris/core/core.hpp>
#include <aris/control/control.hpp>

#include "aris/server/interface.hpp"
#include "aris/server/control_server.hpp"

#include "json.hpp"




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
				std::cout << "unrecognized return value" << std::endl;
			}

#undef ARIS_SET_TYPE
		}
		
		return  js.dump(2);
	}
	auto onReceivedMsg(aris::core::Socket *socket, aris::core::Msg &msg)->int
	{
		std::string msg_data = msg.toString();

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
			aris::server::ControlServer::instance().executeCmdInCmdLine(aris::core::Msg(msg), [socket, msg](aris::plan::Plan &plan)->void
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
					std::cout << e.what() << std::endl;
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

			std::cout << ret_str << std::endl;
			LOG_ERROR << ret_str << std::endl;

			try
			{
				aris::core::Msg m = msg;
				m.copy(ret_str);
				socket->sendMsg(m);
			}
			catch (std::exception &e)
			{
				std::cout << e.what() << std::endl;
				LOG_ERROR << e.what() << std::endl;
			}
		}

		return 0;
	}
	auto onReceivedConnection(aris::core::Socket *sock, const char *ip, int port)->int
	{
		std::cout << "socket receive connection" << std::endl;
		LOG_INFO << "socket receive connection:\n"
			<< std::setw(aris::core::LOG_SPACE_WIDTH) << "|" << "  ip:" << ip << "\n"
			<< std::setw(aris::core::LOG_SPACE_WIDTH) << "|" << "port:" << port << std::endl;
		return 0;
	}
	auto onLoseConnection(aris::core::Socket *socket)->int
	{
		std::cout << "socket lose connection" << std::endl;
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
				std::cout << e.what() << std::endl << "will try to restart server socket in 1s" << std::endl;
				LOG_ERROR << e.what() << std::endl << "will try to restart server socket in 1s" << std::endl;
				std::this_thread::sleep_for(std::chrono::seconds(1));
			}
		}
		std::cout << "socket restart successful" << std::endl;
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
		
		if (imp_->is_running_.exchange(true) == false)
		{
			mg_mgr_init(&imp_->mgr, NULL);
			std::memset(&imp_->s_http_server_opts, 0, sizeof(mg_serve_http_opts));
			imp_->s_http_server_opts.document_root = imp_->document_root_.c_str();

			// Set HTTP server options //
			memset(&imp_->bind_opts, 0, sizeof(imp_->bind_opts));
			imp_->bind_opts.error_string = &imp_->err_str;

			imp_->nc = mg_bind_opt(&imp_->mgr, imp_->port_.c_str(), [](struct mg_connection *nc, int ev, void *ev_data)
			{
				struct http_message *hm = (struct http_message *) ev_data;

				switch (ev) {
				case MG_EV_HTTP_REQUEST:
				{
					std::cout << std::string(hm->uri.p, hm->uri.len) << std::endl;

					if (mg_vcmp(&hm->uri, "/api/config/interface") == 0)
					{
						mg_http_serve_file(nc, hm, "C:\\Users\\py033\\Desktop\\interface111.txt",
							mg_mk_str("text/plain; charset=utf-8"), mg_mk_str(""));
					}
					else
					{
						mg_serve_http(nc, hm, *reinterpret_cast<mg_serve_http_opts*>(nc->user_data));
					}
					break;
				}
				default:

					break;
				}
			}, imp_->bind_opts);
			if (imp_->nc == NULL) {
				//fprintf(stderr, "Error starting server on port %s: %s\n", s_http_port, *imp_->bind_opts.error_string);
				exit(1);
			}

			mg_set_protocol_http_websocket(imp_->nc);
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

	struct ProgramWebInterface::Imp
	{
		aris::core::CommandParser command_parser_;
		aris::plan::LanguageParser language_parser_;
		std::thread auto_thread_;
		std::atomic<int> current_line_;
		bool is_auto_mode_{ false };

		std::atomic_bool is_stop_{ false }, is_pause_{ false };

	};
	auto ProgramWebInterface::open()->void { sock_->startServer(); }
	auto ProgramWebInterface::close()->void { sock_->stop(); }
	auto ProgramWebInterface::loadXml(const aris::core::XmlElement &xml_ele)->void
	{
		Interface::loadXml(xml_ele);
		this->sock_ = findOrInsertType<aris::core::Socket>("socket", "", "5866", aris::core::Socket::WEB);
	}
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
			"</Command>");
		imp_->command_parser_.commandPool().add<aris::core::Command>(program);

		sock_ = &add<aris::core::Socket>("socket", "", port, type);
		sock_->setOnReceivedMsg([this](aris::core::Socket *socket, aris::core::Msg &msg)->int
		{
			auto send_ret = [&](aris::core::Msg &ret_msg)->void
			{
				try
				{
					socket->sendMsg(ret_msg);
				}
				catch (std::exception &e)
				{
					std::cout << e.what() << std::endl;
					LOG_ERROR << e.what() << std::endl;
				}
			};
			auto send_code_and_msg = [&](int code, const std::string& ret_msg_str) 
			{
				nlohmann::json js;
				js["return_code"] = code;///////////////////////////////////////////////////////////
				js["return_message"] = ret_msg_str;

				aris::core::Msg ret_msg = msg;
				ret_msg.copy(js.dump(2));

				std::cout << ret_msg.toString() << std::endl;

				send_ret(ret_msg);
			};

			std::string msg_data = msg.toString();

			LOG_INFO << "receive cmd:"
				<< msg.header().msg_size_ << "&"
				<< msg.header().msg_id_ << "&"
				<< msg.header().msg_type_ << "&"
				<< msg.header().reserved1_ << "&"
				<< msg.header().reserved2_ << "&"
				<< msg.header().reserved3_ << ":"
				<< msg_data << std::endl;

			std::string cmd;
			std::map<std::string, std::string> params;
			try { imp_->command_parser_.parse(msg_data, cmd, params); }
			catch (std::exception &) {};

			if (cmd == "program")
			{
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
							send_code_and_msg(-4, "can not set manual when auto running");
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
						if (!isAutoMode())
						{
							send_code_and_msg(-4, "can not set content in manual mode");
							return 0;
						}
						else if (isAutoRunning())
						{
							send_code_and_msg(-4, "can not set content when auto running");
							return 0;
						}
						else
						{
							auto begin_pos = value.find("{");
							auto end_pos = value.rfind("}");
							auto cmd_str = value.substr(begin_pos + 1, end_pos - 1 - begin_pos);

							/*
							std::cout << cmd_str << std::endl;
							cmd_str =
								"0  :var\r\n"
								"1  :function aaa\r\n"
								"2  :	sl --count=1000\r\n"
								"3  :	if\r\n"
								"4  :		bbb\r\n"
								"5  :		sl --count=1000\r\n"
								"6  :		sl --count=1000\r\n"
								"7  :		sl --count=1000\r\n"
								"8  :	endif\r\n"
								"9  :endfunction\r\n"
								"10 :var\r\n"
								"11 :function bbb\r\n"
								"17 :endfunction\r\n"
								"20 :var\r\n"
								"21 :main\r\n"
								"22 :   sl --count=1000\r\n"
								"23 :	if\r\n"
								"24 :		sl --count=1000\r\n"
								"45 :		if\r\n"
								"46 :			sl --count=1000\r\n"
								"47 :		else\r\n"
								"48 :			sl --count=1000\r\n"
								"49 :		endif\r\n"
								"61 :		if\r\n"
								"62 :			sl --count=1000\r\n"
								"63 :		elseif\r\n"
								"64 :			sl --count=1000\r\n"
								"75 :		endif\r\n"
								"81 :		if\r\n"
								"82 :		elseif\r\n"
								"83 :		elseif\r\n"
								"84 :		else\r\n"
								"85 :		endif\r\n"
								"101:		if\r\n"
								"102:			aaa\r\n"
								"103:		elseif\r\n"
								"104:			sl --count=1000\r\n"
								"105:		elseif\r\n"
								"106:			sl --count=1000\r\n"
								"107:		else\r\n"
								"108:			sl --count=1000\r\n"
								"109:		endif\r\n"
								"200:		sl --count=1000\r\n"
								"201:	endif\r\n"
								"202:while\r\n"
								"203:\r\n"
								"204:endwhile\r\n"
								"205:aaa\r\n"
								"300:endmain \r\n";*/
							try 
							{
								imp_->language_parser_.setProgram(cmd_str);
								imp_->language_parser_.parseLanguage();

								// 设置变量 ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
								for (auto &str : imp_->language_parser_.varPool())
								{
									std::cout << str << std::endl;
									//////////////////////////////////////////////////////////////////////////////////////
									try
									{
										aris::server::ControlServer::instance().executeCmd(aris::core::Msg(str));
									}
									catch(std::exception &e)
									{
									}
									//////////////////////////////////////////////////////////////////////////////////////
								}
								send_code_and_msg(0, std::string());
								return 0;
							}
							catch (std::exception &e)
							{
								std::cout << e.what() << std::endl;
								send_code_and_msg(-4, e.what());
								return 0;
							}
							
							
						}
					}
					else if (param == "start")
					{
						if (!isAutoMode())
						{
							send_code_and_msg(-4, "can not start program in manual mode");
							return 0;
						}
						else if (isAutoRunning())
						{
							send_code_and_msg(-4, "can not start program when running");
							return 0;
						}
						else
						{
							imp_->auto_thread_ = std::thread([&]()->void
							{
								auto&cs = aris::server::ControlServer::instance();
								imp_->current_line_.store(imp_->language_parser_.currentLine());

								for (; !imp_->language_parser_.isEnd();)
								{
									std::cout << "auto:" << std::setw(5) << imp_->language_parser_.currentLine() << ":" << imp_->language_parser_.currentCmd() << std::endl;

									if (imp_->language_parser_.isCurrentLineKeyWord())
									{
										cs.waitForAllCollection();

										auto cmd_name = imp_->language_parser_.currentCmd().substr(0, imp_->language_parser_.currentCmd().find_first_of(" \t\n\r\f\v("));
										auto cmd_value = imp_->language_parser_.currentCmd().find_first_of(" \t\n\r\f\v(") == std::string::npos ? "" :
											imp_->language_parser_.currentCmd().substr(imp_->language_parser_.currentCmd().find_first_of(" \t\n\r\f\v("), std::string::npos);

										if (imp_->language_parser_.currentCmd() == "if")
										{
											std::promise<aris::core::Matrix> promise_value;
											std::future<aris::core::Matrix> future_value = promise_value.get_future();
											
											// 这里把evaluate这个指令写好 //
											try
											{
												aris::server::ControlServer::instance().executeCmd(aris::core::Msg("evaluate --value={" + cmd_value + "}"), [&](aris::plan::Plan& plan)->void
												{
													aris::core::Matrix m(1.0);
													promise_value.set_value(m);
												});
											}
											catch (std::exception &e)
											{
												aris::core::Matrix m(1.0);
												promise_value.set_value(m);
											}
											// 计算完毕 //

											auto value = future_value.get();
											imp_->language_parser_.forward(value.toDouble() != 0.0);
										}
										else if (imp_->language_parser_.currentCmd() == "while")
										{
											std::promise<aris::core::Matrix> promise_value;
											std::future<aris::core::Matrix> future_value = promise_value.get_future();

											// 这里把evaluate这个指令写好 //
											try 
											{
												aris::server::ControlServer::instance().executeCmd(aris::core::Msg("evaluate --value={" + cmd_value + "}"), [&](aris::plan::Plan& plan)->void
												{
													aris::core::Matrix m(0.0);
													promise_value.set_value(m);
												});
											}
											catch (std::exception &e)
											{
												aris::core::Matrix m(0.0);
												promise_value.set_value(m);
											}
											// 计算完毕 //

											auto value = future_value.get();
											imp_->language_parser_.forward(value.toDouble() != 0.0);
										}
										else
										{
											imp_->language_parser_.forward();
										}

										
									}
									else if (imp_->language_parser_.isCurrentLineFunction())
									{
										cs.waitForAllCollection();
										imp_->language_parser_.forward();
									}
									else
									{
										auto cmd = imp_->language_parser_.currentCmd();
										imp_->language_parser_.forward();

										try
										{
											auto current_line = imp_->language_parser_.currentLine();
											cs.executeCmd(aris::core::Msg(cmd), [&, current_line](aris::plan::Plan &plan)->void
											{
												imp_->current_line_.store(current_line);
											});
										}
										catch (std::exception &e)
										{
											std::cout << e.what() << std::endl;
										}
									}
								}

								cs.waitForAllCollection();

								while (!imp_->auto_thread_.joinable());
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
							send_code_and_msg(-4, "can not stop program in manual mode");
							return 0;
						}
						else if (!isAutoRunning())
						{
							send_code_and_msg(-4, "can not stop program when not running");
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
							send_code_and_msg(-4, "can not stop program in manual mode");
							return 0;
						}
						else if (!isAutoRunning())
						{
							send_code_and_msg(-4, "can not stop program when not running");
							return 0;
						}
						else
						{
							imp_->is_pause_.store(true);
							send_code_and_msg(0, "");
							return 0;
						}
					}
					else if (param == "goto")
					{
						if (!isAutoMode())
						{
							send_code_and_msg(-4, "can not goto in manual mode");
							return 0;
						}
						else if (isAutoRunning())
						{
							send_code_and_msg(-4, "can not goto when running");
							return 0;
						}
						else
						{
							imp_->language_parser_.gotoLine(std::stoi(value));
							send_code_and_msg(0, "");
							return 0;
						}
					}
					else if (param == "goto_main")
					{
						if (!isAutoMode())
						{
							send_code_and_msg(-4, "can not goto in manual mode");
							return 0;
						}
						else if (isAutoRunning())
						{
							send_code_and_msg(-4, "can not goto when running");
							return 0;
						}
						else
						{
							imp_->language_parser_.gotoMain();
							send_code_and_msg(0, "");
							return 0;
						}
					}
					else if (param == "forward")
					{
					}
				}
			}
			else
			{
				try
				{
					aris::server::ControlServer::instance().executeCmdInCmdLine(aris::core::Msg(msg), [socket, msg, send_ret](aris::plan::Plan &plan)->void
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
							ret_msg.copy(aris::server::parse_ret_value(*js));
						}

						// return back to source
						send_ret(ret_msg);
					});
				}
				catch (std::exception &e)
				{
					std::vector<std::pair<std::string, std::any>> ret_pair;
					ret_pair.push_back(std::make_pair<std::string, std::any>("return_code", int(aris::plan::Plan::PARSE_EXCEPTION)));
					ret_pair.push_back(std::make_pair<std::string, std::any>("return_message", std::string(e.what())));
					std::string ret_str = aris::server::parse_ret_value(ret_pair);

					//std::cout << ret_str << std::endl;
					LOG_ERROR << ret_str << std::endl;

					aris::core::Msg m = msg;
					m.copy(ret_str);

					send_ret(m);
				}
			}

			return 0;
		});
		sock_->setOnReceivedConnection([](aris::core::Socket *sock, const char *ip, int port)->int
		{
			std::cout << "socket receive connection" << std::endl;
			LOG_INFO << "socket receive connection:\n"
				<< std::setw(aris::core::LOG_SPACE_WIDTH) << "|" << "  ip:" << ip << "\n"
				<< std::setw(aris::core::LOG_SPACE_WIDTH) << "|" << "port:" << port << std::endl;
			return 0;
		});
		sock_->setOnLoseConnection([](aris::core::Socket *socket)->int
		{
			std::cout << "socket lose connection" << std::endl;
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
					std::cout << e.what() << std::endl << "will try to restart server socket in 1s" << std::endl;
					LOG_ERROR << e.what() << std::endl << "will try to restart server socket in 1s" << std::endl;
					std::this_thread::sleep_for(std::chrono::seconds(1));
				}
			}
			std::cout << "socket restart successful" << std::endl;
			LOG_INFO << "socket restart successful" << std::endl;

			return 0;
		});
	}
	ProgramWebInterface::ProgramWebInterface(ProgramWebInterface && other) = default;
	ProgramWebInterface& ProgramWebInterface::operator=(ProgramWebInterface&& other) = default;
}