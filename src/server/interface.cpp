#include <cstring>
#include <thread>
#include <algorithm>
#include <memory>
#include <cinttypes>
#include <queue>
#include <unordered_map>
#include <cstdio>

#include <aris/core/core.hpp>
#include <aris/control/control.hpp>

#include "aris/server/interface.hpp"
#include "aris/server/control_server.hpp"

#include "md5.h"
#include "json.hpp"
#include "fifo_map.hpp"

namespace aris::server
{
	Interface::Interface(const std::string &name){}

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
	
	struct WebInterface::Imp
	{
		std::unique_ptr<aris::core::Socket> sock_{ new aris::core::Socket };
	};
	auto WebInterface::resetSocket(aris::core::Socket *sock)->void
	{
		imp_->sock_.reset(sock);
		socket().setOnReceivedMsg(onReceivedMsg);
		socket().setOnReceivedConnection(onReceivedConnection);
		socket().setOnLoseConnection(onLoseConnection);
	}
	auto WebInterface::socket()->aris::core::Socket& { return *imp_->sock_; }
	auto WebInterface::open()->void { socket().startServer(); }
	auto WebInterface::close()->void { socket().stop(); }
	WebInterface::~WebInterface() = default;
	WebInterface::WebInterface(const std::string &name, const std::string &port, aris::core::Socket::TYPE type):Interface(name), imp_(new Imp)
	{
		resetSocket(new aris::core::Socket("socket", "", port, type));
		socket().setOnReceivedMsg(onReceivedMsg);
		socket().setOnReceivedConnection(onReceivedConnection);
		socket().setOnLoseConnection(onLoseConnection);
	}

#define ARIS_PRO_COUT ARIS_COUT << "pro "
	struct ProgramWebInterface::Imp
	{
		std::unique_ptr<aris::core::Socket> sock_{new aris::core::Socket};
		
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
	auto ProgramWebInterface::resetSocket(aris::core::Socket *sock)->void 
	{ 
		imp_->sock_.reset(sock);
		socket().setOnReceivedMsg(imp_->onReceiveMsg_);
		socket().setOnReceivedConnection(imp_->onReceiveConnection_);
		socket().setOnLoseConnection(imp_->onLoseConnection_);
	}
	auto ProgramWebInterface::socket()->aris::core::Socket& { return *imp_->sock_; }
	auto ProgramWebInterface::open()->void { socket().startServer(); }
	auto ProgramWebInterface::close()->void { socket().stop(); }
	auto ProgramWebInterface::lastError()->std::string { return imp_->last_error_; }
	auto ProgramWebInterface::lastErrorCode()->int { return imp_->last_error_code_; }
	auto ProgramWebInterface::lastErrorLine()->int { return imp_->last_error_line_; }
	auto ProgramWebInterface::isAutoMode()->bool { return imp_->is_auto_mode_; }
	auto ProgramWebInterface::isAutoRunning()->bool { return imp_->auto_thread_.joinable(); }
	auto ProgramWebInterface::isAutoPaused()->bool { return imp_->is_pause_.load(); }
	auto ProgramWebInterface::isAutoStopped()->bool { return imp_->is_stop_.load(); }
	auto ProgramWebInterface::currentLine()->int { return imp_->current_line_.load(); }
	ProgramWebInterface::ProgramWebInterface(const std::string &name, const std::string &port, aris::core::Socket::TYPE type) :Interface(name), imp_(new Imp)
	{
		aris::core::Command program;
		aris::core::fromXmlString(program,
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
		imp_->command_parser_.commandPool().push_back(program);
		imp_->command_parser_.init();

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
						if (!isAutoMode())
						{
							send_code_and_msg(aris::plan::Plan::PROGRAM_EXCEPTION, "can not foward in manual mode");
							return 0;
						}
						else if (isAutoRunning())
						{
							send_code_and_msg(aris::plan::Plan::PROGRAM_EXCEPTION, "can not foward when running");
							return 0;
						}
						else if (lastErrorCode())
						{
							send_code_and_msg(lastErrorCode(), lastError());
							return 0;
						}
						else
						{
							std::swap(imp_->calculator_, aris::server::ControlServer::instance().model().calculator());
							auto &c = aris::server::ControlServer::instance().model().calculator();
							auto &cs = aris::server::ControlServer::instance();
							
							if (imp_->language_parser_.isEnd())
							{

							}
							else if (imp_->language_parser_.isCurrentLineKeyWord())
							{
								ARIS_PRO_COUT << imp_->language_parser_.currentLine() << "---" << imp_->language_parser_.currentCmd() << std::endl;
								LOG_INFO << "pro " << imp_->language_parser_.currentLine() << "---" << imp_->language_parser_.currentCmd() << std::endl;
								if (imp_->language_parser_.currentWord() == "if" || imp_->language_parser_.currentWord() == "while")
								{
									try
									{
										auto ret = c.calculateExpression(imp_->language_parser_.currentParamStr());

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
								ARIS_PRO_COUT << imp_->language_parser_.currentLine() << "---" << imp_->language_parser_.currentCmd() << std::endl;
								LOG_INFO << "pro " << imp_->language_parser_.currentLine() << "---" << imp_->language_parser_.currentCmd() << std::endl;
								imp_->language_parser_.forward();
								imp_->current_line_.store(imp_->language_parser_.currentLine());
							}
							else if (imp_->language_parser_.currentWord() == "set")
							{
								ARIS_PRO_COUT << imp_->language_parser_.currentLine() << "---" << imp_->language_parser_.currentCmd() << std::endl;
								LOG_INFO << "pro " << imp_->language_parser_.currentLine() << "---" << imp_->language_parser_.currentCmd() << std::endl;
								try
								{
									c.calculateExpression(imp_->language_parser_.currentParamStr());
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
								}
							}
							else
							{
								auto &cmd = imp_->language_parser_.currentCmd();
								auto current_line = imp_->language_parser_.currentLine();
								imp_->language_parser_.forward();
								auto next_line = imp_->language_parser_.currentLine();

								auto ret = cs.executeCmdInCmdLine(cmd, [&, current_line, next_line](aris::plan::Plan &plan)->void
								{
									imp_->current_line_.store(next_line);
								});

								ARIS_PRO_COUT << current_line << "---" << ret->cmdId() << "---" << ret->cmdString() << std::endl;
								LOG_INFO << "pro " << current_line << "---" << ret->cmdId() << "---" << ret->cmdString() << std::endl;

								cs.waitForAllCollection();

								// 如果因为其他轨迹出错而取消 //
								if (ret->retCode() == aris::plan::Plan::PREPARE_CANCELLED || ret->retCode() == aris::plan::Plan::EXECUTE_CANCELLED)
								{
									ARIS_PRO_COUT << current_line << "---" << ret->cmdId() << "---canceled" << std::endl;
									LOG_ERROR << "pro " << current_line << "---" << ret->cmdId() << "---canceled" << std::endl;
								}
								else if (ret->retCode() < 0)
								{
									imp_->last_error_code_ = ret->retCode();
									imp_->last_error_ = ret->retMsg();
									imp_->last_error_line_ = current_line;
									ARIS_PRO_COUT << imp_->last_error_line_ << "---" << ret->cmdId() << "---err_code:" << imp_->last_error_code_ << "  err_msg:" << imp_->last_error_ << std::endl;
									LOG_ERROR << "pro " << imp_->last_error_line_ << "---" << ret->cmdId() << "---err_code:" << imp_->last_error_code_ << "  err_msg:" << imp_->last_error_ << std::endl;
								}

							}

							std::swap(imp_->calculator_, aris::server::ControlServer::instance().model().calculator());
							send_code_and_msg(imp_->last_error_code_, imp_->last_error_);
							return 0;
						}
					}
					else if (param == "start")
					{
						LOG_INFO << "pro now start" << std::endl;
						
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
							return 0;
						}
						else
						{
							imp_->is_pause_.store(false);
							imp_->is_stop_.store(false);

							imp_->auto_thread_ = std::thread([&]()->void
							{
								// 交换calculator，保证每个程序开始时的变量都是之前的 //
								std::swap(imp_->calculator_, aris::server::ControlServer::instance().model().calculator());
								auto &c = aris::server::ControlServer::instance().model().calculator();
								auto &cs = aris::server::ControlServer::instance();
								imp_->current_line_.store(imp_->language_parser_.currentLine());
								std::vector < std::pair<std::string_view, std::function<void(aris::plan::Plan&)>> > cmd_vec;
								std::vector <int> lines;

								for (int has_error{ 0 }; has_error == 0 && (!imp_->language_parser_.isEnd());)
								{
									if (imp_->is_stop_.load() == true)break;
									if (imp_->is_pause_.load() == true)
									{
										std::this_thread::sleep_for(std::chrono::milliseconds(1));
										continue;
									}

									// 碰到断点时才真正执行 //
									auto server_execute = [&]() ->int
									{
										auto plans = cs.executeCmdInCmdLine(cmd_vec);
										for (int i = 0; i < plans.size(); ++i)
										{
											ARIS_PRO_COUT << lines[i] << "---" << plans[i]->cmdId() << "---" << plans[i]->cmdString() << std::endl;
											LOG_INFO << "pro " << lines[i] << "---" << plans[i]->cmdId() << "---" << plans[i]->cmdString() << std::endl;
										}
										cs.waitForAllCollection();
										for (int i = 0; i < plans.size(); ++i)
										{
											// 如果因为其他轨迹出错而取消 //
											if (plans[i]->retCode() == aris::plan::Plan::PREPARE_CANCELLED || plans[i]->retCode() == aris::plan::Plan::EXECUTE_CANCELLED)
											{
												ARIS_PRO_COUT << lines[i] << "---" << plans[i]->cmdId() << "---canceled" << std::endl;
												LOG_ERROR << "pro " << lines[i] << "---" << plans[i]->cmdId() << "---canceled" << std::endl;
											}
											else if (plans[i]->retCode() < 0)
											{
												imp_->last_error_code_ = plans[i]->retCode();
												imp_->last_error_ = plans[i]->retMsg();
												imp_->last_error_line_ = lines[i];
												ARIS_PRO_COUT << imp_->last_error_line_ << "---" << plans[i]->cmdId() << "---err_code:" << imp_->last_error_code_ << "  err_msg:" << imp_->last_error_ << std::endl;
												LOG_ERROR << "pro " << imp_->last_error_line_ << "---" << plans[i]->cmdId() << "---err_code:" << imp_->last_error_code_ << "  err_msg:" << imp_->last_error_ << std::endl;
												has_error = -1;
											}
										}
										cmd_vec.clear();
										lines.clear();
										plans.clear();
										return has_error;
									};

									if (imp_->language_parser_.isCurrentLineKeyWord())
									{
										if (server_execute())continue;
										ARIS_PRO_COUT << imp_->language_parser_.currentLine() << "---" << imp_->language_parser_.currentCmd() << std::endl;
										LOG_INFO << "pro " << imp_->language_parser_.currentLine() << "---" << imp_->language_parser_.currentCmd() << std::endl;
										if (imp_->language_parser_.currentWord() == "if" || imp_->language_parser_.currentWord() == "while")
										{
											try 
											{
												auto ret = c.calculateExpression(imp_->language_parser_.currentParamStr());

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
										if (server_execute())continue;
										ARIS_PRO_COUT << imp_->language_parser_.currentLine() << "---" << imp_->language_parser_.currentCmd() << std::endl;
										LOG_INFO << "pro " << imp_->language_parser_.currentLine() << "---" << imp_->language_parser_.currentCmd() << std::endl;
										imp_->language_parser_.forward();
										imp_->current_line_.store(imp_->language_parser_.currentLine());
									}
									else if (imp_->language_parser_.currentWord() == "set")
									{
										if (server_execute())continue;
										ARIS_PRO_COUT << imp_->language_parser_.currentLine() << "---" << imp_->language_parser_.currentCmd() << std::endl;
										LOG_INFO << "pro " << imp_->language_parser_.currentLine() << "---" << imp_->language_parser_.currentCmd() << std::endl;
										try
										{
											c.calculateExpression(imp_->language_parser_.currentParamStr());
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
											has_error = -1;
										}
									}
									else
									{
										auto &cmd = imp_->language_parser_.currentCmd();
										auto current_line = imp_->language_parser_.currentLine();
										imp_->language_parser_.forward();
										auto next_line = imp_->language_parser_.currentLine();

										cmd_vec.push_back(std::pair<std::string_view, std::function<void(aris::plan::Plan&)>>(std::string_view(cmd), [&, current_line, next_line](aris::plan::Plan &plan)->void
										{
											imp_->current_line_.store(next_line);
										}));
										lines.push_back(current_line);
									}

									if (imp_->language_parser_.isEnd()) server_execute();
								}
								
								cs.waitForAllCollection();
								imp_->current_line_.store(imp_->language_parser_.currentLine());

								std::swap(imp_->calculator_, aris::server::ControlServer::instance().model().calculator());
								ARIS_PRO_COUT << "---" << (imp_->is_stop_.load() ? "program stopped" : "program finished") << std::endl;
								LOG_INFO << "pro " << "---" << (imp_->is_stop_.load() ? "program stopped" : "program finished") << std::endl;
								
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

		resetSocket(new aris::core::Socket("socket", "", port, type));
		imp_->sock_->setOnReceivedMsg(imp_->onReceiveMsg_);
		imp_->sock_->setOnReceivedConnection(imp_->onReceiveConnection_);
		imp_->sock_->setOnLoseConnection(imp_->onLoseConnection_);
	}
	ProgramWebInterface::ProgramWebInterface(ProgramWebInterface && other) = default;
	ProgramWebInterface& ProgramWebInterface::operator=(ProgramWebInterface&& other) = default;
	ProgramWebInterface::~ProgramWebInterface() = default;


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

	//static const char send_back_data[] = "{ \"name\":\"model\",\"VariablePool\" : {\"name\":\"variable_pool\",\"matrixVariables\" : [{\"name\":\"v5\",\"innerValue\" : \"{0.01,0.005,3.49065850398866,0,0}\"},{ \"name\":\"v10\",\"innerValue\" : \"{0.03,0.01,3.49065850398866,0,0}\" },{ \"name\":\"v25\",\"innerValue\" : \"{0.05,0.025,3.49065850398866,0,0}\" },{ \"name\":\"v30\",\"innerValue\" : \"{0.05,0.03,3.49065850398866,0,0}\" },{ \"name\":\"v40\",\"innerValue\" : \"{0.05,0.04,3.49065850398866,0,0}\" },{ \"name\":\"v50\",\"innerValue\" : \"{0.08,0.05,3.49065850398866,0,0}\" },{ \"name\":\"v60\",\"innerValue\" : \"{0.08,0.06,3.49065850398866,0,0}\" },{ \"name\":\"v80\",\"innerValue\" : \"{0.08,0.08,3.49065850398866,0,0}\" },{ \"name\":\"v100\",\"innerValue\" : \"{0.1,0.1,3.49065850398866,0,0}\" },{ \"name\":\"v150\",\"innerValue\" : \"{0.15,0.15,3.49065850398866,0,0}\" },{ \"name\":\"v200\",\"innerValue\" : \"{0.2,0.2,3.49065850398866,0,0}\" },{ \"name\":\"v300\",\"innerValue\" : \"{0.3,0.3,3.49065850398866,0,0}\" }]},\"partPool\" : {\"name\":\"part_pool\",\"parts\" : [{\"name\":\"ground\",\"markerPool\" : [{\"name\":\"marker_pool\",\"marker\" : [{\"name\":\"joint_0_j\",\"pe\" : \"{0,0,0.3295,0.785398163397448,0,0.785398163397448}\"},{ \"name\":\"wobj0\",\"pe\" : \"{0,0,0,-0,0,-0}\" },{ \"name\":\"wobj1\",\"pe\" : \"{0,0,0,-0,0,-0}\" },{ \"name\":\"wobj2\",\"pe\" : \"{0,0,0,-0,0,-0}\" },{ \"name\":\"wobj3\",\"pe\" : \"{0,0,0,-0,0,-0}\" },{ \"name\":\"wobj4\",\"pe\" : \"{0,0,0,-0,0,-0}\" },{ \"name\":\"wobj5\",\"pe\" : \"{0,0,0,-0,0,-0}\" },{ \"name\":\"wobj6\",\"pe\" : \"{0,0,0,-0,0,-0}\" },{ \"name\":\"wobj7\",\"pe\" : \"{0,0,0,-0,0,-0}\" },{ \"name\":\"wobj8\",\"pe\" : \"{0,0,0,-0,0,-0}\" },{ \"name\":\"wobj9\",\"pe\" : \"{0,0,0,-0,0,-0}\" },{ \"name\":\"wobj10\",\"pe\" : \"{0,0,0,-0,0,-0}\" },{ \"name\":\"wobj11\",\"pe\" : \"{0,0,0,-0,0,-0}\" },{ \"name\":\"wobj12\",\"pe\" : \"{0,0,0,-0,0,-0}\" },{ \"name\":\"wobj13\",\"pe\" : \"{0,0,0,-0,0,-0}\" },{ \"name\":\"wobj14\",\"pe\" : \"{0,0,0,-0,0,-0}\" },{ \"name\":\"wobj15\",\"pe\" : \"{0,0,0,-0,0,-0}\" },{ \"name\":\"wobj16\",\"pe\" : \"{0,0,0,-0,0,-0}\" },{ \"name\":\"wobj17\",\"pe\" : \"{0,0,0,-0,0,-0}\" },{ \"name\":\"wobj18\",\"pe\" : \"{0,0,0,-0,0,-0}\" },{ \"name\":\"wobj19\",\"pe\" : \"{0,0,0,-0,0,-0}\" },{ \"name\":\"wobj20\",\"pe\" : \"{0,0,0,-0,0,-0}\" },{ \"name\":\"wobj21\",\"pe\" : \"{0,0,0,-0,0,-0}\" },{ \"name\":\"wobj22\",\"pe\" : \"{0,0,0,-0,0,-0}\" },{ \"name\":\"wobj23\",\"pe\" : \"{0,0,0,-0,0,-0}\" },{ \"name\":\"wobj24\",\"pe\" : \"{0,0,0,-0,0,-0}\" },{ \"name\":\"wobj25\",\"pe\" : \"{0,0,0,-0,0,-0}\" },{ \"name\":\"wobj26\",\"pe\" : \"{0,0,0,-0,0,-0}\" },{ \"name\":\"wobj27\",\"pe\" : \"{0,0,0,-0,0,-0}\" },{ \"name\":\"wobj28\",\"pe\" : \"{0,0,0,-0,0,-0}\" },{ \"name\":\"wobj29\",\"pe\" : \"{0,0,0,-0,0,-0}\" },{ \"name\":\"wobj30\",\"pe\" : \"{0,0,0,-0,0,-0}\" },{ \"name\":\"wobj31\",\"pe\" : \"{0,0,0,-0,0,-0}\" },{ \"name\":\"wobj32\",\"pe\" : \"{0,0,0,-0,0,-0}\" }]}]},{ \"name\":\"L1\",\"markerPool\" : [{\"name\":\"marker_pool\",\"marker\" : [{\"name\":\"joint_0_i\",\"pe\" : \"{0,0,0.3295,0.785398163397448,0,0.785398163397448}\"},{ \"name\":\"joint_1_j\",\"pe\" : \"{0.04,0,0.3295,3.14159265358979,1.5707963267949,4.71238898038469}\" }]}] },{ \"name\":\"L2\",\"markerPool\" : [{\"name\":\"marker_pool\",\"marker\" : [{\"name\":\"joint_1_i\",\"pe\" : \"{0.04,0,0.3295,3.14159265358979,1.5707963267949,4.71238898038469}\"},{ \"name\":\"joint_2_j\",\"pe\" : \"{0.04,0,0.6045,3.14159265358979,1.5707963267949,4.71238898038469}\" }]}] },{ \"name\":\"L3\",\"markerPool\" : [{\"name\":\"marker_pool\",\"marker\" : [{\"name\":\"joint_2_i\",\"pe\" : \"{0.04,0,0.6045,3.14159265358979,1.5707963267949,4.71238898038469}\"},{ \"name\":\"joint_3_j\",\"pe\" : \"{0.32,0,0.6295,1.5707963267949,1.5707963267949,1.5707963267949}\" }]}] },{ \"name\":\"L4\",\"markerPool\" : [{\"name\":\"marker_pool\",\"marker\" : [{\"name\":\"joint_3_i\",\"pe\" : \"{0.32,0,0.6295,1.5707963267949,1.5707963267949,1.5707963267949}\"},{ \"name\":\"joint_4_j\",\"pe\" : \"{0.32,0,0.6295,3.14159265358979,1.5707963267949,4.71238898038469}\" }]}] },{ \"name\":\"L5\",\"markerPool\" : [{\"name\":\"marker_pool\",\"marker\" : [{\"name\":\"joint_4_i\",\"pe\" : \"{0.32,0,0.6295,3.14159265358979,1.5707963267949,4.71238898038469}\"},{ \"name\":\"joint_5_j\",\"pe\" : \"{0.32,0,0.6295,1.5707963267949,1.5707963267949,1.5707963267949}\" }]}] },{ \"name\":\"L6\",\"markerPool\" : [{\"name\":\"marker_pool\",\"marker\" : [{\"name\":\"joint_5_i\",\"pe\" : \"{0.32,0,0.6295,1.5707963267949,1.5707963267949,1.5707963267949}\"},{ \"name\":\"tool0\",\"pe\" : \"{0.398,0,0.6295,1.5707963267949,1.5707963267949,4.71238898038469}\" },{ \"name\":\"tool1\",\"pe\" : \"{0.398,0,0.6295,1.5707963267949,1.5707963267949,4.71238898038469}\" },{ \"name\":\"tool2\",\"pe\" : \"{0.398,0,0.6295,1.5707963267949,1.5707963267949,4.71238898038469}\" },{ \"name\":\"tool3\",\"pe\" : \"{0.398,0,0.6295,1.5707963267949,1.5707963267949,4.71238898038469}\" },{ \"name\":\"tool4\",\"pe\" : \"{0.398,0,0.6295,1.5707963267949,1.5707963267949,4.71238898038469}\" },{ \"name\":\"tool5\",\"pe\" : \"{0.398,0,0.6295,1.5707963267949,1.5707963267949,4.71238898038469}\" },{ \"name\":\"tool6\",\"pe\" : \"{0.398,0,0.6295,1.5707963267949,1.5707963267949,4.71238898038469}\" },{ \"name\":\"tool7\",\"pe\" : \"{0.398,0,0.6295,1.5707963267949,1.5707963267949,4.71238898038469}\" },{ \"name\":\"tool8\",\"pe\" : \"{0.398,0,0.6295,1.5707963267949,1.5707963267949,4.71238898038469}\" },{ \"name\":\"tool9\",\"pe\" : \"{0.398,0,0.6295,1.5707963267949,1.5707963267949,4.71238898038469}\" },{ \"name\":\"tool10\",\"pe\" : \"{0.398,0,0.6295,1.5707963267949,1.5707963267949,4.71238898038469}\" },{ \"name\":\"tool11\",\"pe\" : \"{0.398,0,0.6295,1.5707963267949,1.5707963267949,4.71238898038469}\" },{ \"name\":\"tool12\",\"pe\" : \"{0.398,0,0.6295,1.5707963267949,1.5707963267949,4.71238898038469}\" },{ \"name\":\"tool13\",\"pe\" : \"{0.398,0,0.6295,1.5707963267949,1.5707963267949,4.71238898038469}\" },{ \"name\":\"tool14\",\"pe\" : \"{0.398,0,0.6295,1.5707963267949,1.5707963267949,4.71238898038469}\" },{ \"name\":\"tool15\",\"pe\" : \"{0.398,0,0.6295,1.5707963267949,1.5707963267949,4.71238898038469}\" },{ \"name\":\"tool16\",\"pe\" : \"{0.398,0,0.6295,1.5707963267949,1.5707963267949,4.71238898038469}\" }]}] }]},\"jointPool\" : {\"name\":\"joint_pool\",\"joints\" : null},\"calibratorPool\" : {\"name\":\"calibrator_pool\",\"calibrators\" : null},\"simResultPool\" : {\"name\":\"sim_result_pool\",\"simResults\" : null},\"simulatorPool\" : {\"name\":\"simulator_pool\",\"simulators\" : null},\"solverPool\" : {\"name\":\"solver_pool\",\"solvers\" : null},\"forcePool\" : {\"name\":\"force_pool\",\"forces\" : null},\"generalMotionPool\" : {\"name\":\"general_motion_pool\",\"generalMotions\" : [{\"name\":\"ee\"}]},\"motionPool\" : {\"name\":\"motion_pool\",\"motions\" : [{\"name\":\"motion_0\"},{ \"name\":\"motion_1\" },{ \"name\":\"motion_2\" },{ \"name\":\"motion_3\" },{ \"name\":\"motion_4\" },{ \"name\":\"motion_5\" }]} }";
	//static const char esi_path[] = "{\"path\":\"C:\\Users\\py033\\Desktop\\UI_DarkColor_English-0103_panbo\\UI_DarkColor_English-0103_panbo\\robot\\esi\"}";
	//static const char robots[] = "{ \"A\":{\"A\":{\"brand\":\"A\",\"model\" : \"A\",\"path\" : \"RobotGallery/A/A\",\"parts\" : {\"GROUND\":\"\",\"L1\" : \"\",\"L2\" : \"\",\"L3\" : \"\",\"L4\" : \"\",\"L5\" : \"\",\"L6\" : \"\"}}},\"APE\" : {\"450\":{\"brand\":\"APE\",\"model\" : \"450\",\"path\" : \"RobotGallery/APE/450\",\"parts\" : {\"GROUND\":\"\",\"L1\" : \"l1.data\",\"L2\" : \"l2.data\",\"L3\" : \"l3.data\",\"L4\" : \"l4.data\",\"L5\" : \"l5.data\",\"L6\" : \"l6.data\"}}},\"AnyRobot\" : {\"WebPlayer\":{\"brand\":\"AnyRobot\",\"model\" : \"WebPlayer\",\"path\" : \"RobotGallery/AnyRobot/WebPlayer\",\"parts\" : {\"GROUND\":\"ground.data\",\"L1\" : \"l1.data\",\"L2\" : \"\",\"L3\" : \"\",\"L4\" : \"\",\"L5\" : \"\",\"L6\" : \"\"}}},\"Daye\" : {\"MJ08\":{\"brand\":\"Daye\",\"model\" : \"MJ08\",\"path\" : \"RobotGallery/Daye/MJ08\",\"parts\" : {\"GROUND\":\"\",\"L1\" : \"l1.data\",\"L2\" : \"l2.data\",\"L3\" : \"l3.data\",\"L4\" : \"l4.data\",\"L5\" : \"l5.data\",\"L6\" : \"l6.data\"}}},\"Qifan\" : {\"MR16\":{\"brand\":\"Qifan\",\"model\" : \"MR16\",\"path\" : \"RobotGallery/Qifan/MR16\",\"parts\" : {\"GROUND\":\"\",\"L1\" : \"l1.data\",\"L2\" : \"l2.data\",\"L3\" : \"l3.data\",\"L4\" : \"l4.data\",\"L5\" : \"l5.data\",\"L6\" : \"l6.data\"}}},\"Rokae\" : {\"XB4\":{\"brand\":\"Rokae\",\"model\" : \"XB4\",\"path\" : \"RobotGallery/Rokae/XB4\",\"parts\" : {\"GROUND\":\"\",\"L1\" : \"l1.data\",\"L2\" : \"l2.data\",\"L3\" : \"l3.data\",\"L4\" : \"l4.data\",\"L5\" : \"l5.data\",\"L6\" : \"l6.data\"}}},\"SJTU\" : {\"BBL\":{\"brand\":\"SJTU\",\"model\" : \"BBL\",\"path\" : \"RobotGallery/SJTU/BBL\",\"parts\" : {\"GROUND\":\"ground.data\",\"L1\" : \"l1.data\",\"L2\" : \"l2.data\",\"L3\" : \"l3.data\",\"L4\" : \"l4.data\",\"L5\" : \"l5.data\",\"L6\" : \"l6.data\"}}},\"Sanxiang\" : {\"S1\":{\"brand\":\"Sanxiang\",\"model\" : \"S1\",\"path\" : \"RobotGallery/Sanxiang/S1\",\"parts\" : {\"GROUND\":\"\",\"L1\" : \"l1.data\",\"L2\" : \"l2.data\",\"L3\" : \"l3.data\",\"L4\" : \"l4.data\",\"L5\" : \"l5.data\",\"L6\" : \"l6.data\"}}},\"XX\" : {\"XXMODEL\":{\"brand\":\"XX\",\"model\" : \"XXMODEL\",\"path\" : \"RobotGallery/XX/XXMODEL\",\"parts\" : {\"GROUND\":\"\",\"L1\" : \"\",\"L2\" : \"\",\"L3\" : \"\",\"L4\" : \"\",\"L5\" : \"\",\"L6\" : \"\"}}} }";
	//static const char programs[] = "{\"111\":{\"name\":\"111\",\"path\":\"/program/111\",\"mode\":2147484159,\"modTime\":\"2020-01-05T21:42:48+08:00\",\"isDir\":true,\"files\":[{\"name\":\"111.dat\",\"path\":\"/program/111/111.dat\",\"size\":1930,\"mode\":438,\"modTime\":\"2020-01-05T21:42:48+08:00\",\"content\":\"\\u003cxml xmlns=\\\"https://developers.google.com/blockly/xml\\\"\\u003e\\u003cblock type=\\\"jointtarget_variate\\\" id=\\\"SS/~|B}sk[]Y:dJ?yes8\\\" x=\\\"130\\\" y=\\\"90\\\"\\u003e\\u003cfield name=\\\"FIELD_NAME\\\"\\u003ehome\\u003c/field\\u003e\\u003cfield name=\\\"motionPos0\\\"\\u003e0.000000\\u003c/field\\u003e\\u003cfield name=\\\"motionPos1\\\"\\u003e0.000000\\u003c/field\\u003e\\u003cfield name=\\\"motionPos2\\\"\\u003e0.000000\\u003c/field\\u003e\\u003cfield name=\\\"motionPos3\\\"\\u003e28.647890\\u003c/field\\u003e\\u003cfield name=\\\"motionPos4\\\"\\u003e0.000000\\u003c/field\\u003e\\u003cfield name=\\\"motionPos5\\\"\\u003e0.000000\\u003c/field\\u003e\\u003cnext\\u003e\\u003cblock type=\\\"jointtarget_variate\\\" id=\\\"_Y,KM$KqVcBaK~AJ6zTd\\\"\\u003e\\u003cfield name=\\\"FIELD_NAME\\\"\\u003ehome\\u003c/field\\u003e\\u003cfield name=\\\"motionPos0\\\"\\u003e0.000000\\u003c/field\\u003e\\u003cfield name=\\\"motionPos1\\\"\\u003e0.000000\\u003c/field\\u003e\\u003cfield name=\\\"motionPos2\\\"\\u003e0.000000\\u003c/field\\u003e\\u003cfield name=\\\"motionPos3\\\"\\u003e28.647890\\u003c/field\\u003e\\u003cfield name=\\\"motionPos4\\\"\\u003e0.000000\\u003c/field\\u003e\\u003cfield name=\\\"motionPos5\\\"\\u003e0.000000\\u003c/field\\u003e\\u003cnext\\u003e\\u003cblock type=\\\"robtarget_variate\\\" id=\\\"7mtA#Y8`/Ep9MKX6!K$U\\\"\\u003e\\u003cfield name=\\\"FIELD_NAME\\\"\\u003epB1\\u003c/field\\u003e\\u003cfield name=\\\"END_PQ0\\\"\\u003e0.000000\\u003c/field\\u003e\\u003cfield name=\\\"END_PQ1\\\"\\u003e0.000000\\u003c/field\\u003e\\u003cfield name=\\\"END_PQ2\\\"\\u003e0.000000\\u003c/field\\u003e\\u003cfield name=\\\"END_PQ3\\\"\\u003e0.500000\\u003c/field\\u003e\\u003cfield name=\\\"END_PQ4\\\"\\u003e0.000000\\u003c/field\\u003e\\u003cfield name=\\\"END_PQ5\\\"\\u003e0.000000\\u003c/field\\u003e\\u003cfield name=\\\"END_PQ6\\\"\\u003e0.000000\\u003c/field\\u003e\\u003cnext\\u003e\\u003cblock type=\\\"pose_variate\\\" id=\\\":aUhP8]pG=ac?Oif4KGE\\\"\\u003e\\u003cfield name=\\\"FIELD_NAME\\\"\\u003epose1\\u003c/field\\u003e\\u003cfield name=\\\"END_PQ0\\\"\\u003e0.000000\\u003c/field\\u003e\\u003cfield name=\\\"END_PQ1\\\"\\u003e0.000000\\u003c/field\\u003e\\u003cfield name=\\\"END_PQ2\\\"\\u003e0.000000\\u003c/field\\u003e\\u003cfield name=\\\"END_PQ3\\\"\\u003e0.500000\\u003c/field\\u003e\\u003cfield name=\\\"END_PQ4\\\"\\u003e0.000000\\u003c/field\\u003e\\u003cfield name=\\\"END_PQ5\\\"\\u003e0.000000\\u003c/field\\u003e\\u003cfield name=\\\"END_PQ6\\\"\\u003e0.000000\\u003c/field\\u003e\\u003cnext\\u003e\\u003cblock type=\\\"pose_variate\\\" id=\\\".%-Bm5(UwV87k_4G}P#S\\\"\\u003e\\u003cfield name=\\\"FIELD_NAME\\\"\\u003epose1\\u003c/field\\u003e\\u003cfield name=\\\"END_PQ0\\\"\\u003e0.000000\\u003c/field\\u003e\\u003cfield name=\\\"END_PQ1\\\"\\u003e0.000000\\u003c/field\\u003e\\u003cfield name=\\\"END_PQ2\\\"\\u003e0.000000\\u003c/field\\u003e\\u003cfield name=\\\"END_PQ3\\\"\\u003e0.500000\\u003c/field\\u003e\\u003cfield name=\\\"END_PQ4\\\"\\u003e0.000000\\u003c/field\\u003e\\u003cfield name=\\\"END_PQ5\\\"\\u003e0.000000\\u003c/field\\u003e\\u003cfield name=\\\"END_PQ6\\\"\\u003e0.000000\\u003c/field\\u003e\\u003c/block\\u003e\\u003c/next\\u003e\\u003c/block\\u003e\\u003c/next\\u003e\\u003c/block\\u003e\\u003c/next\\u003e\\u003c/block\\u003e\\u003c/next\\u003e\\u003c/block\\u003e\\u003c/xml\\u003e\"},{\"name\":\"111.pro\",\"path\":\"/program/111/111.pro\",\"size\":56,\"mode\":438,\"modTime\":\"2020-01-05T21:42:48+08:00\",\"content\":\"\\u003cxml xmlns=\\\"https://developers.google.com/blockly/xml\\\"/\\u003e\"}]},\"Scale\":{\"name\":\"Scale\",\"path\":\"/program/Scale\",\"mode\":2147484159,\"modTime\":\"2020-01-05T21:42:48+08:00\",\"isDir\":true,\"files\":[{\"name\":\"Scale.dat\",\"path\":\"/program/Scale/Scale.dat\",\"size\":11,\"mode\":438,\"modTime\":\"2020-01-05T21:42:48+08:00\",\"content\":\"\\u003cxml\\u003e\\u003c/xml\\u003e\"},{\"name\":\"Scale.pro\",\"path\":\"/program/Scale/Scale.pro\",\"size\":1031,\"mode\":438,\"modTime\":\"2020-01-05T21:42:48+08:00\",\"content\":\"\\u003cxml xmlns=\\\"https://developers.google.com/blockly/xml\\\"\\u003e\\u003cblock type=\\\"robotcontrol_movej\\\" id=\\\"~~b2VUYx`@Rm;G[rcQGH\\\" x=\\\"10\\\" y=\\\"50\\\"\\u003e\\u003cmutation xmlns=\\\"http://www.w3.org/1999/xhtml\\\" topoint=\\\"select\\\" speed=\\\"select\\\" zone=\\\"select\\\" tool=\\\"select\\\" wobj=\\\"select\\\"\\u003e\\u003c/mutation\\u003e\\u003cfield name=\\\"DROPDOWN1\\\"\\u003eselect\\u003c/field\\u003e\\u003cfield name=\\\"DROPDOWN2\\\"\\u003eselect\\u003c/field\\u003e\\u003cfield name=\\\"DROPDOWN3\\\"\\u003eselect\\u003c/field\\u003e\\u003cfield name=\\\"DROPDOWN4\\\"\\u003eselect\\u003c/field\\u003e\\u003cfield name=\\\"DROPDOWN5\\\"\\u003eselect\\u003c/field\\u003e\\u003cnext\\u003e\\u003cblock type=\\\"robotcontrol_movej\\\" id=\\\".=YH,s0OBV`{!rLv#ps5\\\"\\u003e\\u003cmutation xmlns=\\\"http://www.w3.org/1999/xhtml\\\" topoint=\\\"select\\\" speed=\\\"select\\\" zone=\\\"select\\\" tool=\\\"select\\\" wobj=\\\"select\\\"\\u003e\\u003c/mutation\\u003e\\u003cfield name=\\\"DROPDOWN1\\\"\\u003eselect\\u003c/field\\u003e\\u003cfield name=\\\"DROPDOWN2\\\"\\u003eselect\\u003c/field\\u003e\\u003cfield name=\\\"DROPDOWN3\\\"\\u003eselect\\u003c/field\\u003e\\u003cfield name=\\\"DROPDOWN4\\\"\\u003eselect\\u003c/field\\u003e\\u003cfield name=\\\"DROPDOWN5\\\"\\u003eselect\\u003c/field\\u003e\\u003cnext\\u003e\\u003cblock type=\\\"fcpressl\\\" id=\\\"fZE;DLRPV.gWP!+l.T@f\\\"\\u003e\\u003cfield name=\\\"FORCE\\\"\\u003eForce\\u003c/field\\u003e\\u003cfield name=\\\"TOOL\\\"\\u003eTool\\u003c/field\\u003e\\u003cfield name=\\\"WOBJ\\\"\\u003eWobj\\u003c/field\\u003e\\u003c/block\\u003e\\u003c/next\\u003e\\u003c/block\\u003e\\u003c/next\\u003e\\u003c/block\\u003e\\u003c/xml\\u003e\"}]},\"Wheel\":{\"name\":\"Wheel\",\"path\":\"/program/Wheel\",\"mode\":2147484159,\"modTime\":\"2020-01-05T21:42:48+08:00\",\"isDir\":true,\"files\":[{\"name\":\"Wheel.dat\",\"path\":\"/program/Wheel/Wheel.dat\",\"size\":11,\"mode\":438,\"modTime\":\"2020-02-20T17:56:15+08:00\",\"content\":\"\\u003cxml\\u003e\\u003c/xml\\u003e\"},{\"name\":\"Wheel.pro\",\"path\":\"/program/Wheel/Wheel.pro\",\"size\":2818,\"mode\":438,\"modTime\":\"2020-02-20T17:56:15+08:00\",\"content\":\"\\u003cxml xmlns=\\\"https://developers.google.com/blockly/xml\\\"\\u003e\\u003cblock type=\\\"robotcontrol_movej\\\" id=\\\"AlIl~vttKXoDmEpS_ikv\\\" x=\\\"10\\\" y=\\\"110\\\"\\u003e\\u003cmutation xmlns=\\\"http://www.w3.org/1999/xhtml\\\" topoint=\\\"select\\\" speed=\\\"select\\\" zone=\\\"select\\\" tool=\\\"select\\\" wobj=\\\"select\\\"\\u003e\\u003c/mutation\\u003e\\u003cfield name=\\\"DROPDOWN1\\\"\\u003eselect\\u003c/field\\u003e\\u003cfield name=\\\"DROPDOWN2\\\"\\u003eselect\\u003c/field\\u003e\\u003cfield name=\\\"DROPDOWN3\\\"\\u003eselect\\u003c/field\\u003e\\u003cfield name=\\\"DROPDOWN4\\\"\\u003eselect\\u003c/field\\u003e\\u003cfield name=\\\"DROPDOWN5\\\"\\u003eselect\\u003c/field\\u003e\\u003cnext\\u003e\\u003cblock type=\\\"robotcontrol_movej\\\" id=\\\"g~=p7xsQq*A-7M^(Rs[z\\\"\\u003e\\u003cmutation xmlns=\\\"http://www.w3.org/1999/xhtml\\\" topoint=\\\"select\\\" speed=\\\"select\\\" zone=\\\"select\\\" tool=\\\"select\\\" wobj=\\\"select\\\"\\u003e\\u003c/mutation\\u003e\\u003cfield name=\\\"DROPDOWN1\\\"\\u003eselect\\u003c/field\\u003e\\u003cfield name=\\\"DROPDOWN2\\\"\\u003eselect\\u003c/field\\u003e\\u003cfield name=\\\"DROPDOWN3\\\"\\u003eselect\\u003c/field\\u003e\\u003cfield name=\\\"DROPDOWN4\\\"\\u003eselect\\u003c/field\\u003e\\u003cfield name=\\\"DROPDOWN5\\\"\\u003eselect\\u003c/field\\u003e\\u003cnext\\u003e\\u003cblock type=\\\"robotcontrol_movej\\\" id=\\\"f|GShByvvUAeEGZv*R8{\\\"\\u003e\\u003cmutation xmlns=\\\"http://www.w3.org/1999/xhtml\\\" topoint=\\\"select\\\" speed=\\\"select\\\" zone=\\\"select\\\" tool=\\\"select\\\" wobj=\\\"select\\\"\\u003e\\u003c/mutation\\u003e\\u003cfield name=\\\"DROPDOWN1\\\"\\u003eselect\\u003c/field\\u003e\\u003cfield name=\\\"DROPDOWN2\\\"\\u003eselect\\u003c/field\\u003e\\u003cfield name=\\\"DROPDOWN3\\\"\\u003eselect\\u003c/field\\u003e\\u003cfield name=\\\"DROPDOWN4\\\"\\u003eselect\\u003c/field\\u003e\\u003cfield name=\\\"DROPDOWN5\\\"\\u003eselect\\u003c/field\\u003e\\u003cnext\\u003e\\u003cblock type=\\\"fcpressp\\\" id=\\\"(~Vo=!Nv!pKymj49x:lg\\\"\\u003e\\u003cfield name=\\\"P1\\\"\\u003ep1\\u003c/field\\u003e\\u003cfield name=\\\"P2\\\"\\u003ep2\\u003c/field\\u003e\\u003cfield name=\\\"P3\\\"\\u003ep3\\u003c/field\\u003e\\u003cfield name=\\\"P4\\\"\\u003ep4\\u003c/field\\u003e\\u003cfield name=\\\"P5\\\"\\u003ep5\\u003c/field\\u003e\\u003cfield name=\\\"FORCE\\\"\\u003eForce\\u003c/field\\u003e\\u003cfield name=\\\"TOOL\\\"\\u003eTool\\u003c/field\\u003e\\u003cfield name=\\\"WOBJ\\\"\\u003eWobj\\u003c/field\\u003e\\u003cnext\\u003e\\u003cblock type=\\\"robotcontrol_movej\\\" id=\\\"x},iBu:=#1xe_g~-zcz?\\\"\\u003e\\u003cmutation xmlns=\\\"http://www.w3.org/1999/xhtml\\\" topoint=\\\"select\\\" speed=\\\"select\\\" zone=\\\"select\\\" tool=\\\"select\\\" wobj=\\\"select\\\"\\u003e\\u003c/mutation\\u003e\\u003cfield name=\\\"DROPDOWN1\\\"\\u003eselect\\u003c/field\\u003e\\u003cfield name=\\\"DROPDOWN2\\\"\\u003eselect\\u003c/field\\u003e\\u003cfield name=\\\"DROPDOWN3\\\"\\u003eselect\\u003c/field\\u003e\\u003cfield name=\\\"DROPDOWN4\\\"\\u003eselect\\u003c/field\\u003e\\u003cfield name=\\\"DROPDOWN5\\\"\\u003eselect\\u003c/field\\u003e\\u003cnext\\u003e\\u003cblock type=\\\"robotcontrol_movej\\\" id=\\\"W``C?uixyW$y@u;6{~-:\\\"\\u003e\\u003cmutation xmlns=\\\"http://www.w3.org/1999/xhtml\\\" topoint=\\\"select\\\" speed=\\\"select\\\" zone=\\\"select\\\" tool=\\\"select\\\" wobj=\\\"select\\\"\\u003e\\u003c/mutation\\u003e\\u003cfield name=\\\"DROPDOWN1\\\"\\u003eselect\\u003c/field\\u003e\\u003cfield name=\\\"DROPDOWN2\\\"\\u003eselect\\u003c/field\\u003e\\u003cfield name=\\\"DROPDOWN3\\\"\\u003eselect\\u003c/field\\u003e\\u003cfield name=\\\"DROPDOWN4\\\"\\u003eselect\\u003c/field\\u003e\\u003cfield name=\\\"DROPDOWN5\\\"\\u003eselect\\u003c/field\\u003e\\u003cnext\\u003e\\u003cblock type=\\\"movec\\\" id=\\\"!jF8g|ECav2#;g/%EmV`\\\"\\u003e\\u003cmutation xmlns=\\\"http://www.w3.org/1999/xhtml\\\" auxpoint=\\\"select\\\" topoint=\\\"select\\\" speed=\\\"select\\\" zone=\\\"select\\\" tool=\\\"select\\\" wobj=\\\"select\\\"\\u003e\\u003c/mutation\\u003e\\u003cfield name=\\\"DROPDOWN1\\\"\\u003eselect\\u003c/field\\u003e\\u003cfield name=\\\"DROPDOWN2\\\"\\u003eselect\\u003c/field\\u003e\\u003cfield name=\\\"DROPDOWN3\\\"\\u003eselect\\u003c/field\\u003e\\u003cfield name=\\\"DROPDOWN4\\\"\\u003eselect\\u003c/field\\u003e\\u003cfield name=\\\"DROPDOWN5\\\"\\u003eselect\\u003c/field\\u003e\\u003cfield name=\\\"DROPDOWN6\\\"\\u003eselect\\u003c/field\\u003e\\u003c/block\\u003e\\u003c/next\\u003e\\u003c/block\\u003e\\u003c/next\\u003e\\u003c/block\\u003e\\u003c/next\\u003e\\u003c/block\\u003e\\u003c/next\\u003e\\u003c/block\\u003e\\u003c/next\\u003e\\u003c/block\\u003e\\u003c/next\\u003e\\u003c/block\\u003e\\u003c/xml\\u003e\"}]}}";

	struct HttpInterface::Imp
	{
		std::thread http_thread_;
		std::string port_;
		std::string document_root_;
		std::string dav_root_;

		std::map<std::string, std::function<std::string(void)>> gets_, posts_, deletes_, puts_;

		struct mg_mgr mgr;
		struct mg_connection *nc;
		struct mg_bind_opts bind_opts;
		struct mg_serve_http_opts s_http_server_opts;
		const char *err_str;

		std::mutex mu_running_;
		std::atomic_bool is_running_{ false };

		static void event_handle_for_aris_ui(struct mg_connection *nc, int ev, void *ev_data)
		{
			struct http_message *hm = (struct http_message *) ev_data;

			switch (ev) {
			case MG_EV_HTTP_REQUEST:
			{
				std::cout << std::string(hm->message.p, hm->message.len) << std::endl;

				auto http_method = std::string(hm->method.p, hm->method.len);
				auto http_uri = std::string(hm->uri.p, hm->uri.len);
				auto http_body = std::string(hm->body.p, hm->body.len);

				auto imp = reinterpret_cast<Imp*>(nc->user_data);
				struct mg_serve_http_opts opts;
				std::memset(&opts, 0, sizeof(mg_serve_http_opts));
				opts.document_root = imp->document_root_.c_str();
				opts.dav_document_root = imp->dav_root_.c_str();
				opts.enable_directory_listing = "yes";
				opts.dav_auth_file = ".htpasswd";
				opts.global_auth_file = "D:\\Private\\aris-ui\\build\\.htpasswd";

				auto get_realm = [](std::string_view username, std::string_view file_name)->std::string
				{
					std::fstream auth_file{ std::string(file_name) };
					while (!auth_file.eof())
					{
						std::string line;
						std::getline(auth_file, line);
						if (username == line.substr(0, line.find_first_of(':')))
							return line.substr(line.find_first_of(':') + 1, line.find_last_of(':') - line.find_first_of(':') - 1);
					}
					return "";
				};
				auto get_http_header2 = [](http_message*hm, std::string_view header_name, std::string_view second_name)->std::string
				{
					struct mg_str *hdr = mg_get_http_header(hm, header_name.data());
					if (!hdr) return std::string();

					char buf[1024];
					char *data = buf;

					auto size = mg_http_parse_header2(hdr, second_name.data(), &data, 1024);

					std::string ret(data, size);
					if (data != buf)free(buf);
					return ret;
				};

				if (http_method == "POST" && http_uri == "/api/login/fetch")
				{
					auto js = my_json::parse(http_body, nullptr, false);
					std::string username = "";
					std::string realm = js.is_discarded() || !js.contains("username") ? "" : get_realm(js["username"].get_to(username), opts.global_auth_file);

					mg_printf(nc,
						"HTTP/1.1 200 Unauthorized\r\n"
						"WWW-Authenticate: Digest qop=\"auth\", "
						"realm=\"%s\", nonce=\"%lx\"\r\n"
						"Content-Length: 0\r\n\r\n",
						realm.c_str(), (unsigned long)mg_time());

					break;
				}
				else if (http_method == "POST" && http_uri == "/api/login")
				{
					auto realm = get_http_header2(hm, "Authorization", "realm");
					if (mg_http_is_authorized(hm, mg_mk_str(""), realm.data(), opts.global_auth_file, MG_AUTH_FLAG_IS_GLOBAL_PASS_FILE))
					{
						mg_printf(nc,
							"HTTP/1.1 202 Accepted\r\n"
							"Content-Length: 0\r\n\r\n");
						break;
					}
					else
					{
						mg_printf(nc,
							"HTTP/1.1 200 Unauthorized\r\n"
							"Content-Length: 0\r\n\r\n");
						break;
					}
				}
				else if (http_method == "GET")
				{
					mg_serve_http(nc, hm, opts);
				}
				else if (http_method == "POST")
				{
					auto fp = mg_fopen(opts.global_auth_file, "r");

					auto username = get_http_header2(hm, "Cookie", "username");
					auto cnonce = get_http_header2(hm, "Cookie", "cnonce");
					auto response = get_http_header2(hm, "Cookie", "response");
					auto qop = get_http_header2(hm, "Cookie", "qop");
					auto uri = get_http_header2(hm, "Cookie", "uri");
					auto ncount = get_http_header2(hm, "Cookie", "nc");
					auto nonce = get_http_header2(hm, "Cookie", "nonce");
					auto realm = get_http_header2(hm, "Cookie", "realm");

					auto ret = mg_check_digest_auth(
						mg_mk_str("POST"), mg_mk_str(uri.data()),
						mg_mk_str(username.data()), mg_mk_str(cnonce.data()), mg_mk_str(response.data()),
						mg_mk_str(qop.data()), mg_mk_str(ncount.data()), mg_mk_str(nonce.data()), mg_mk_str(realm.data()),
						fp);

					fclose(fp);

					// 检查失败 //
					if (ret == 0)
					{
						mg_printf(nc,
							"HTTP/1.1 200 Unauthorized\r\n"
							"Content-Length: 0\r\n\r\n");
					}

					opts.auth_domain = realm.data();
					mg_serve_http(nc, hm, opts);
				}
				else
				{
					std::cout << "unknown cmd" << std::endl;
				}
			}
			default:
				break;
			}
		}
		static void event_handle_for_old(struct mg_connection *nc, int ev, void *ev_data)
		{
			struct http_message *hm = (struct http_message *) ev_data;

			switch (ev) {
			case MG_EV_HTTP_REQUEST:
			{
				std::cout << std::string(hm->message.p, hm->message.len) << std::endl;

				auto http_method = std::string(hm->method.p, hm->method.len);
				auto http_uri = std::string(hm->uri.p, hm->uri.len);
				auto http_body = std::string(hm->body.p, hm->body.len);

				auto imp = reinterpret_cast<Imp*>(nc->user_data);
				struct mg_serve_http_opts opts;
				std::memset(&opts, 0, sizeof(mg_serve_http_opts));
				opts.document_root = imp->document_root_.c_str();
				opts.dav_document_root = imp->dav_root_.c_str();
				opts.enable_directory_listing = "yes";

				if (http_method == "GET" && http_uri == "/api/login/fetch")
				{
				}
				else if (http_method == "POST" && http_uri == "/api/login")
				{

				}
				else if (http_method == "GET")
				{
				}
				else if (http_method == "POST")
				{

				}
				else
				{
					std::cout << "unknown cmd" << std::endl;
				}
			}
			default:
				break;
			}
		}
	};
	auto HttpInterface::open()->void
	{
		std::unique_lock<std::mutex> running_lck(imp_->mu_running_);

		tinyxml2::XMLDocument doc;
		std::filesystem::path interface_path(imp_->document_root_);
		//interface_path = interface_path / "../robot/interface.xml";
		interface_path = "C:/Users/py033/Desktop/UI_DarkColor_English-0103_panbo/UI_DarkColor_English-0103_panbo/robot/interface.xml";
		doc.LoadFile(interface_path.string().c_str());

		//my_json js;
		//for (auto ele = doc.RootElement()->FirstChildElement(); ele; ele = ele->NextSiblingElement())
		//{
		//	if (ele->Name() == std::string("Dashboard"))
		//	{
		//		my_json js1;
		//		js1["name"] = std::string(ele->Attribute("name"));
		//		js1["editable"] = std::string(ele->Attribute("editable")) == "true";
		//		js1["i"] = std::string(ele->Attribute("id"));
		//		js1["cells"] = std::vector<std::string>();
		//		for (auto e1 = ele->FirstChildElement(); e1; e1 = e1->NextSiblingElement())
		//		{
		//			my_json j2;//{"name":"Ethercat配置","type":"EthercatConfiguration","i":"EMlxGXxpwDGgz","w":48,"h":23,"x":0,"y":0,"options":"{}"}
		//			j2["name"] = e1->Attribute("name");
		//			j2["type"] = e1->Attribute("type");
		//			j2["i"] = e1->Attribute("id");
		//			j2["w"] = e1->IntAttribute("width");
		//			j2["h"] = e1->IntAttribute("height");
		//			j2["x"] = e1->IntAttribute("x");
		//			j2["y"] = e1->IntAttribute("y");
		//			j2["options"] = e1->Attribute("options");
		//			js1["cells"].push_back(j2);
		//		}
		//		js["dashboards"].push_back(js1);
		//	}
		//	else if (ele->Name() == std::string("WebSocket"))
		//	{
		//		js["ws"]["url"] = ele->Attribute("url");
		//		js["ws"]["commandSendInterval"] = ele->IntAttribute("commandSendInterval");
		//		js["ws"]["commandSendDelay"] = ele->IntAttribute("commandSendDelay");
		//		js["ws"]["getInterval"] = ele->IntAttribute("getInterval");
		//		js["ws"]["unityUpdateInterval"] = ele->IntAttribute("unityUpdateInterval");
		//	}
		//	else if (ele->Name() == std::string("LayoutConfig"))
		//	{
		//		js["layoutConfig"]["cols"] = ele->IntAttribute("cols");
		//		js["layoutConfig"]["rowHeight"] = ele->IntAttribute("rowHeight");
		//		js["layoutConfig"]["margin"] = ele->IntAttribute("margin");
		//		js["layoutConfig"]["containerPadding"] = ele->IntAttribute("containerPadding");
		//		js["layoutConfig"]["theme"] = ele->Attribute("theme");
		//	}
		//}

		//auto str = js.dump(2);
		//std::ofstream f;
		//std::filesystem::create_directories("C:\\Users\\py033\\Desktop\\UI_DarkColor_English-0103_panbo\\UI_DarkColor_English-0103_panbo\\www\\api\\config\\");
		//f.open("C:\\Users\\py033\\Desktop\\UI_DarkColor_English-0103_panbo\\UI_DarkColor_English-0103_panbo\\www\\api\\config\\interface");
		//f << str;
		//f.close();

		//f.open("C:\\Users\\py033\\Desktop\\UI_DarkColor_English-0103_panbo\\UI_DarkColor_English-0103_panbo\\www\\api\\config\\programs");
		//f << programs;
		//f.close();
		////std::cout << send_back_data << std::endl;
		////std::cout << esi_path << std::endl;
		////std::cout << robots << std::endl;
		//std::cout << programs << std::endl;

		//js.clear();
		//js.parse(programs);

		//std::cout << js.dump(2) << std::endl;




		if (imp_->is_running_.exchange(true) == false)
		{
			mg_mgr_init(&imp_->mgr, NULL);

			// Set HTTP server options //
			memset(&imp_->bind_opts, 0, sizeof(imp_->bind_opts));
			imp_->bind_opts.error_string = &imp_->err_str;

			imp_->nc = mg_bind_opt(&imp_->mgr, imp_->port_.c_str(), Imp::event_handle_for_aris_ui, imp_->bind_opts);

			if (imp_->nc == NULL) {
				fprintf(stderr, "Error starting server on http\n");
				exit(1);
			}

			mg_set_protocol_http_websocket(imp_->nc);
			
			imp_->nc->user_data = imp_.get();
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
		if (imp_->is_running_.exchange(false) == true) { imp_->http_thread_.join(); }
	}
	auto HttpInterface::documentRoot()->std::string { return imp_->document_root_; }
	auto HttpInterface::setDocumentRoot(const std::string &root)->void { imp_->document_root_ = root; }
	auto HttpInterface::setApiGet(std::string_view uri, std::function<std::string(void)> func)->void
	{
		imp_->gets_[std::string(uri)] = func;
	}
	HttpInterface::~HttpInterface() = default;
	HttpInterface::HttpInterface(HttpInterface && other) = default;
	HttpInterface& HttpInterface::operator=(HttpInterface&& other) = default;
	HttpInterface::HttpInterface(const std::string &name, const std::string &port, const std::string &document_root) :Interface(name), imp_(new Imp)
	{
		imp_->document_root_ = document_root;
		imp_->dav_root_ = document_root;
		imp_->port_ = port;
	}

	ARIS_REGISTRATION{

		aris::core::class_<GetInfo>("GetInfo")
			.inherit<aris::plan::Plan>()
			;
		
		aris::core::class_<aris::core::PointerArray<aris::server::Interface>>("InterfacePoolObject")
			.asRefArray()
			;

		aris::core::class_<Interface>("Interface")
			;
		
		aris::core::class_<WebInterface>("WebInterface")
			.inherit<Interface>()
			.prop("socket", &WebInterface::resetSocket, &WebInterface::socket)
			;
		
		aris::core::class_<ProgramWebInterface>("ProgramWebInterface")
			.inherit<Interface>()
			.prop("socket", &ProgramWebInterface::resetSocket, &ProgramWebInterface::socket)
			;
	}
}