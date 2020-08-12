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
#include "aris/server/api.hpp"
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
			ARIS_SET_TYPE(nlohmann::json)
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
			aris::server::ControlServer::instance().executeCmdInCmdLine(std::string(msg.data(), msg.size()), [socket, msg](aris::plan::Plan &plan)->void
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
		std::mutex auto_mu_;

		int current_line_{0};
		std::string current_file_;
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
	//auto ProgramWebInterface::currentLine()->int { return imp_->current_line_.load(); }
	auto ProgramWebInterface::currentFileLine()->std::tuple<std::string, int> 
	{
		std::unique_lock<std::mutex> lck(imp_->auto_mu_);
		return std::make_tuple(imp_->current_file_, imp_->current_line_);
	}

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
			auto send_code_and_msg = [send_ret, &msg](int code, const std::string& ret_msg_str)
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

			if (msg_data.size() > 100)
			{
				std::cout << msg_data << std::endl;
			}

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
								
								auto js = nlohmann::json::parse(cmd_str);

								std::cout << js << std::endl;

								std::map<std::string, std::string> files;
								for (auto &node : js)
								{
									files[node["name"].get<std::string>()] = node["content"].get<std::string>();
									std::cout << node["content"].get<std::string>() << std::endl;
								}

								imp_->language_parser_.setProgram(files);
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
							auto file = value.substr(0, value.find_first_of("."));
							auto line = std::stoi(std::string(value.substr(value.find_first_of(".") + 1)));
							
							imp_->language_parser_.gotoFileLine(std::string(file) + ".aris", line);
							std::unique_lock<std::mutex> lck(this->imp_->auto_mu_);
							imp_->current_line_ = imp_->language_parser_.currentLine();
							imp_->current_file_ = imp_->language_parser_.currentFile();
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
							std::unique_lock<std::mutex> lck(this->imp_->auto_mu_);
							imp_->current_line_ = imp_->language_parser_.currentLine();
							imp_->current_file_ = imp_->language_parser_.currentFile();
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
											std::unique_lock<std::mutex> lck(this->imp_->auto_mu_);
											imp_->current_line_ = imp_->language_parser_.currentLine();
										}
										else if (auto ret_mat = std::any_cast<aris::core::Matrix>(&ret.second))
										{
											imp_->language_parser_.forward(ret_mat->toDouble() != 0.0);
											std::unique_lock<std::mutex> lck(this->imp_->auto_mu_);
											imp_->current_line_ = imp_->language_parser_.currentLine();
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
									std::unique_lock<std::mutex> lck(this->imp_->auto_mu_);
									imp_->current_line_ = imp_->language_parser_.currentLine();
								}
							}
							else if (imp_->language_parser_.isCurrentLineFunction())
							{
								ARIS_PRO_COUT << imp_->language_parser_.currentLine() << "---" << imp_->language_parser_.currentCmd() << std::endl;
								LOG_INFO << "pro " << imp_->language_parser_.currentLine() << "---" << imp_->language_parser_.currentCmd() << std::endl;
								imp_->language_parser_.forward();
								std::unique_lock<std::mutex> lck(this->imp_->auto_mu_);
								imp_->current_line_ = imp_->language_parser_.currentLine();
								imp_->current_file_ = imp_->language_parser_.currentFile();
							}
							else if (imp_->language_parser_.currentWord() == "set")
							{
								ARIS_PRO_COUT << imp_->language_parser_.currentLine() << "---" << imp_->language_parser_.currentCmd() << std::endl;
								LOG_INFO << "pro " << imp_->language_parser_.currentLine() << "---" << imp_->language_parser_.currentCmd() << std::endl;
								try
								{
									c.calculateExpression(imp_->language_parser_.currentParamStr());
									imp_->language_parser_.forward();
									std::unique_lock<std::mutex> lck(this->imp_->auto_mu_);
									imp_->current_line_ = imp_->language_parser_.currentLine();
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
									std::unique_lock<std::mutex> lck(imp_->auto_mu_);
									imp_->current_line_ = next_line;
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
						else if (!imp_->language_parser_.hasCursor())
						{
							send_code_and_msg(-1, "please goto main or lines");
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
								std::unique_lock<std::mutex> lck(imp_->auto_mu_);
								imp_->current_line_ = imp_->language_parser_.currentLine();
								imp_->current_file_ = imp_->language_parser_.currentFile();
								lck.unlock();
								std::vector < std::pair<std::string, std::function<void(aris::plan::Plan&)>> > cmd_vec;
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
													std::unique_lock<std::mutex> lck(this->imp_->auto_mu_);
													imp_->current_line_ = imp_->language_parser_.currentLine();
												}
												else if (auto ret_mat = std::any_cast<aris::core::Matrix>(&ret.second))
												{
													imp_->language_parser_.forward(ret_mat->toDouble() != 0.0);
													std::unique_lock<std::mutex> lck(this->imp_->auto_mu_);
													imp_->current_line_ = imp_->language_parser_.currentLine();
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
											std::unique_lock<std::mutex> lck(this->imp_->auto_mu_);
											imp_->current_line_ = imp_->language_parser_.currentLine();
											imp_->current_file_ = imp_->language_parser_.currentFile();
										}
									}
									else if (imp_->language_parser_.isCurrentLineFunction())
									{
										if (server_execute())continue;
										ARIS_PRO_COUT << imp_->language_parser_.currentLine() << "---" << imp_->language_parser_.currentCmd() << std::endl;
										LOG_INFO << "pro " << imp_->language_parser_.currentLine() << "---" << imp_->language_parser_.currentCmd() << std::endl;
										imp_->language_parser_.forward();
										std::unique_lock<std::mutex> lck(imp_->auto_mu_);
										imp_->current_line_ = imp_->language_parser_.currentLine();
										imp_->current_file_ = imp_->language_parser_.currentFile();
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
											std::unique_lock<std::mutex> lck(this->imp_->auto_mu_);
											imp_->current_line_ = imp_->language_parser_.currentLine();
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

										cmd_vec.push_back(std::pair<std::string, std::function<void(aris::plan::Plan&)>>(cmd, [&, current_line, next_line](aris::plan::Plan &plan)->void
										{
											std::unique_lock<std::mutex> lck(this->imp_->auto_mu_);
											imp_->current_line_ = next_line;
										}));
										lines.push_back(current_line);
									}

									if (imp_->language_parser_.isEnd()) server_execute();
								}
								
								cs.waitForAllCollection();
								lck.lock();
								imp_->current_line_ = imp_->language_parser_.currentLine();
								lck.unlock();

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
				aris::server::ControlServer::instance().executeCmdInCmdLine(std::string(msg.data(), msg.size()), [socket, msg, send_ret](aris::plan::Plan &plan)->void
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
		std::string dav_root_;

		//std::map<std::string, std::function<std::string(void)>> gets_, posts_, deletes_, puts_;

		struct mg_mgr mgr;
		struct mg_connection *nc;
		struct mg_bind_opts bind_opts;
		struct mg_serve_http_opts s_http_server_opts;
		const char *err_str;

		std::mutex mu_running_;
		std::atomic_bool is_running_{ false };

		static void event_handle_for_aris_ui(struct mg_connection *nc, int ev, void *ev_data)
		{
			try 
			{
				struct http_message *hm = (struct http_message *) ev_data;

				switch (ev) {
				case MG_EV_HTTP_REQUEST:
				{
					auto method = std::string(hm->method.p, hm->method.len);
					auto uri = std::string(hm->uri.p, hm->uri.len);

					std::cout << method << "    " << uri << std::endl;

					if (method == "GET" && uri == "/api/config/interface")
					{
						auto ret = fetchInterfaceConfig();

						mg_printf(nc,
							"HTTP/1.1 200 OK\r\n"
							"Content-Type: application/json; charset=utf-8\r\n"
							"Content-Length: %ld\r\n\r\n",
							ret.size()
						);

						mg_send(nc, ret.c_str(), ret.size());
						break;
					}
					else if (method == "PUT" && uri.find("/api/dashboards") != std::string::npos)
					{
						auto ret = updateDashboard(uri.substr(16), std::string(hm->body.p, hm->body.len));

						mg_printf(nc,
							"HTTP/1.1 200 OK\r\n"
							"Content-Type: application/json; charset=utf-8\r\n"
							"Content-Length: %ld\r\n\r\n",
							ret.size()
						);

						mg_send(nc, ret.c_str(), ret.size());
						break;
					}
					else if (method == "POST" && uri.find("/api/dashboards") != std::string::npos && uri.find("cells") != std::string::npos)
					{
						auto dashid = uri.substr(16, uri.substr(16).find_first_of('/'));
						auto ret = createCell(dashid, std::string(hm->body.p, hm->body.len));

						mg_printf(nc,
							"HTTP/1.1 200 OK\r\n"
							"Content-Type: application/json; charset=utf-8\r\n"
							"Content-Length: %ld\r\n\r\n",
							ret.size()
						);

						mg_send(nc, ret.c_str(), ret.size());
						break;
					}
					else if (method == "DELETE" && uri.find("/api/dashboards") != std::string::npos	&& uri.find("cells") != std::string::npos)
					{
						auto ret = deleteCell(uri.substr(16, uri.substr(16).find("/cells")), uri.substr(uri.find("cells/") + 6));

						mg_printf(nc,
							"HTTP/1.1 200 OK\r\n"
							"Content-Type: application/json; charset=utf-8\r\n"
							"Content-Length: %ld\r\n\r\n",
							ret.size()
						);

						mg_send(nc, ret.c_str(), ret.size());
						break;
					}
					else if (method == "GET" && uri == "/api/programs")
					{
						auto ret = fetchPrograms();

						mg_printf(nc,
							"HTTP/1.1 200 OK\r\n"
							"Content-Type: text/plain; charset=utf-8\r\n"
							"Content-Length: %ld\r\n\r\n",
							ret.size()
						);

						mg_send(nc, ret.c_str(), ret.size());
						break;
					}
					else if (method == "POST" && uri == "/api/programs")
					{
						auto ret = createProgram(std::string(hm->body.p, hm->body.len));

						mg_printf(nc,
							"HTTP/1.1 200 OK\r\n"
							"Content-Type: application/json; charset=utf-8\r\n"
							"Content-Length: %ld\r\n\r\n",
							ret.size()
						);

						mg_send(nc, ret.c_str(), ret.size());
						break;
					}
					else if (method == "PUT" && uri.size() > 13 && uri.substr(0, 13) == "/api/programs")
					{
						auto ret = updateProgram(uri.substr(14), std::string(hm->body.p, hm->body.len));

						mg_printf(nc,
							"HTTP/1.1 200 OK\r\n"
							"Content-Type: application/json; charset=utf-8\r\n"
							"Content-Length: %ld\r\n\r\n",
							ret.size()
						);

						mg_send(nc, ret.c_str(), ret.size());
						break;
					}
					else if (method == "DELETE" && uri.size() > 13 && uri.substr(0, 13) == "/api/programs")
					{
						auto ret = deleteProgram(uri.substr(14));

						if (ret.empty())
						{
							mg_printf(nc,
								"HTTP/1.1 500 failed create\r\n"
								"Content-Type: application/json; charset=utf-8\r\n"
								"Content-Length: %ld\r\n\r\n",
								ret.size()
							);

							mg_send(nc, ret.c_str(), ret.size());
						}
						else
						{
							mg_printf(nc,
								"HTTP/1.1 200 OK\r\n"
								"Content-Type: application/json; charset=utf-8\r\n"
								"Content-Length: %ld\r\n\r\n",
								ret.size()
							);

							mg_send(nc, ret.c_str(), ret.size());
						}

						break;
					}
					else if (method == "PATCH" && uri.size() > 13 && uri.substr(0, 13) == "/api/programs")
					{
						auto ret = renameProgram(uri.substr(14), std::string(hm->body.p, hm->body.len));

						if (ret.empty())
						{
							mg_printf(nc,
								"HTTP/1.1 500 failed create\r\n"
								"Content-Type: application/json; charset=utf-8\r\n"
								"Content-Length: %ld\r\n\r\n",
								ret.size()
							);

							mg_send(nc, ret.c_str(), ret.size());
						}
						else
						{
							mg_printf(nc,
								"HTTP/1.1 200 OK\r\n"
								"Content-Type: application/json; charset=utf-8\r\n"
								"Content-Length: %ld\r\n\r\n",
								ret.size()
							);

							mg_send(nc, ret.c_str(), ret.size());
						}

						break;
					}
					else if (method == "GET" && uri == "/api/config/xml")
					{
						auto ret = fetchConfigXml();

						mg_printf(nc,
							"HTTP/1.1 200 OK\r\n"
							"Accept-Ranges: bytes"
							"Content-Type: text/xml; charset=utf-8\r\n"
							"Content-Length: %ld\r\n\r\n",
							ret.size()
						);

						mg_send(nc, ret.c_str(), ret.size());
						break;
					}
					else if (method == "GET" && uri == "/api/esi/path")
					{
						auto ret = fetchESIPath();

						mg_printf(nc,
							"HTTP/1.1 200 OK\r\n"
							"Content-Type: application/json; charset=utf-8\r\n"
							"Content-Length: %ld\r\n\r\n",
							ret.size()
						);

						mg_send(nc, ret.c_str(), ret.size());
						break;
					}
					else if (method == "GET" && uri == "/api/obj_picture_list")
					{
						auto ret = fetchObjPictureList();

						mg_printf(nc,
							"HTTP/1.1 200 OK\r\n"
							"Content-Type: application/json; charset=utf-8\r\n"
							"Content-Length: %ld\r\n\r\n",
							ret.size()
						);

						mg_send(nc, ret.c_str(), ret.size());
						break;
					}
					else if (method == "POST" && uri == "/api/obj_picture")
					{
						//std::cout << std::string(hm->body.p, hm->body.len) << std::endl;
						
						
						auto ret = postObjPicture(std::string(hm->body.p, hm->body.len));

						mg_printf(nc,
							"HTTP/1.1 200 OK\r\n"
							"Content-Type: application/json; charset=utf-8\r\n"
							"Content-Length: %ld\r\n\r\n",
							ret.size()
						);

						mg_send(nc, ret.c_str(), ret.size());
						break;
					}
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
					else
					{
						mg_serve_http(nc, hm, reinterpret_cast<Imp*>(nc->user_data)->s_http_server_opts);
					}
				}
				default:
					break;
				}
			}
			catch (std::exception &e)
			{
				std::cout << "http error:" << e.what() << std::endl;
			}
			catch (...)
			{
				std::cout << "http error: unknown" << std::endl;
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

		setRootPath(imp_->document_root_);

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
			std::memset(&imp_->s_http_server_opts, 0, sizeof(mg_serve_http_opts));
			imp_->s_http_server_opts.document_root = imp_->document_root_.c_str();
			imp_->s_http_server_opts.enable_directory_listing = "yes";
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
	auto HttpInterface::port()->std::string { return imp_->port_; }
	auto HttpInterface::setPort(const std::string &port)->void { imp_->port_ = port; }
	auto HttpInterface::setDocumentRoot(const std::string &root)->void { imp_->document_root_ = root; }
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

		aris::core::class_<HttpInterface>("HttpInterfaceInterface")
			.inherit<Interface>()
			.prop("document_root", &HttpInterface::setDocumentRoot, &HttpInterface::documentRoot)
			.prop("port", &HttpInterface::setPort, &HttpInterface::port)
			;
	}
}