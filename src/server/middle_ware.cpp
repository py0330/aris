#include "aris/server/middle_ware.hpp"

#include "aris/core/core.hpp"
#include "aris/server/control_server.hpp"

#include "aris/ext/json.hpp"
#include "aris/server/api.hpp"

namespace aris::server {

	std::uint64_t MiddleWare::cmd_id_ = 0;

	auto MiddleWare::executeCmd(std::string_view str, std::function<void(std::string)> send_ret, Interface* interface)->int {

		std::string cmd(str);

		auto cmd_id = MiddleWare::cmd_id_++;

		std::cout << "INTERFACE -- " << interface->name() << std::endl;
		std::cout << "       ID -- " << cmd_id << std::endl;
		std::cout << "      CMD -- " << cmd << std::endl;
		std::cout << std::endl;

		auto send_and_print = [cmd_id, send_ret](std::string ret)->void {
			std::cout << "    ID -- " << cmd_id << std::endl;
			std::cout << "RETURN -- " << ret << std::endl;
			std::cout << std::endl;

			send_ret(ret);
		};

		aris::server::ControlServer::instance().executeCmdInCmdLine(cmd, [send_and_print](aris::plan::Plan& plan)->void
			{
				//LOG_INFO << "cmd " << plan.cmdId() << " return code   :" << plan.retCode() << "\n" << std::setw(aris::core::LOG_SPACE_WIDTH) << '|' << "return message:" << plan.retMsg() << std::endl;

				// prepare error
				if (plan.prepareRetCode() != aris::plan::Plan::RetStatus::SUCCESS) {
					std::vector<std::pair<std::string, std::any>> ret_js;
					ret_js.push_back(std::make_pair<std::string, std::any>("return_code", plan.prepareRetCode()));
					ret_js.push_back(std::make_pair<std::string, std::any>("return_message", std::string(plan.prepareRetMsg())));
					send_and_print(aris::server::parse_ret_value(ret_js));
				}
				else if(auto js = std::any_cast<std::vector<std::pair<std::string, std::any>>>(&plan.ret()))
				{
					js->push_back(std::make_pair<std::string, std::any>("return_code", plan.executeRetCode()));
					js->push_back(std::make_pair<std::string, std::any>("return_message", std::string(plan.executeRetMsg())));
					send_and_print(aris::server::parse_ret_value(*js));
				}
				else
				{
					std::vector<std::pair<std::string, std::any>> ret_js;
					ret_js.push_back(std::make_pair<std::string, std::any>("return_code", plan.executeRetCode()));
					ret_js.push_back(std::make_pair<std::string, std::any>("return_message", std::string(plan.executeRetMsg())));
					send_and_print(aris::server::parse_ret_value(ret_js));
				}
			});

		return 0;
	}
	auto MiddleWare::executeCmd(std::string_view str, Interface* interface)->void {
		executeCmd(str, [](std::string ret) {}, interface);
	}
	auto MiddleWare::executeCmd(std::string_view cmd_str)->void {
		static InternalInterface interface;
		executeCmd(cmd_str, [](std::string ret) {}, &interface);
	}

#define ARIS_PRO_COUT ARIS_COUT << "pro "
	struct ProgramMiddleware::Imp {
		std::unique_ptr<aris::core::Socket> sock_{ new aris::core::Socket };

		aris::core::CommandParser command_parser_;
		aris::core::LanguageParser language_parser_;
		aris::core::Calculator calculator_;
		std::thread auto_thread_;
		std::mutex auto_mu_;

		int current_line_{ 0 };
		std::string current_file_;
		bool is_auto_mode_{ false };

		std::string last_error_;
		int last_error_code_{ 0 }, last_error_line_{ 0 };

		std::atomic_bool is_stop_{ false }, is_pause_{ false };

	};
	auto ProgramMiddleware::lastError()->std::string { return imp_->last_error_; }
	auto ProgramMiddleware::lastErrorCode()->int { return imp_->last_error_code_; }
	auto ProgramMiddleware::lastErrorLine()->int { return imp_->last_error_line_; }
	auto ProgramMiddleware::isAutoMode()->bool { return imp_->is_auto_mode_; }
	auto ProgramMiddleware::isAutoRunning()->bool { return imp_->auto_thread_.joinable(); }
	auto ProgramMiddleware::isAutoPaused()->bool { return imp_->is_pause_.load(); }
	auto ProgramMiddleware::isAutoStopped()->bool { return imp_->is_stop_.load(); }
	auto ProgramMiddleware::currentFileLine()->std::tuple<std::string, int> {
		std::unique_lock<std::mutex> lck(imp_->auto_mu_);
		return std::make_tuple(imp_->current_file_, imp_->current_line_);
	}
	auto ProgramMiddleware::executeCmd(std::string_view str, std::function<void(std::string)> send_ret, Interface* interface)->int
	{
		(void)(interface);
		auto send_code_and_msg = [send_ret](int code, const std::string& ret_msg_str)->int
		{
			nlohmann::json js;
			js["return_code"] = code;///////////////////////////////////////////////////////////
			js["return_message"] = ret_msg_str;

			auto ret_str = js.dump(-1, ' ', true);

			ARIS_PRO_COUT << "---" << ret_str << std::endl;

			send_ret(ret_str);
			return 0;
		};

		std::string_view cmd;
		std::map<std::string_view, std::string_view> params;
		try { std::tie(cmd, params) = imp_->command_parser_.parse(str); }
		catch (std::exception&) {};

		if (cmd == "program") {
			ARIS_PRO_COUT << "---" << str << std::endl;
			ARIS_LOG(aris::core::LogLvl::kInfo, 0, { "pro ---%s" }, str.data());

			for (auto& [param, value] : params) {
				if (param == "set_auto") {
					imp_->is_auto_mode_ = true;
					return send_code_and_msg(0, "");
				}
				else if (param == "set_manual") {
					if (isAutoRunning()) {
						return send_code_and_msg(aris::plan::Plan::PROGRAM_EXCEPTION, "can not set manual when auto running");
					}
					else {
						imp_->is_auto_mode_ = false;
						return send_code_and_msg(0, "");
					}
				}
				else if (param == "content")
				{
					if (isAutoRunning()) {
						return send_code_and_msg(aris::plan::Plan::PROGRAM_EXCEPTION, "can not set content when auto running");
					}
					else {
						imp_->last_error_.clear();
						imp_->last_error_code_ = 0;
						imp_->last_error_line_ = 0;

						auto begin_pos = value.find("{");
						auto end_pos = value.rfind("}");
						auto cmd_str = value.substr(begin_pos + 1, end_pos - 1 - begin_pos);

						try
						{
							//imp_->calculator_ = aris::server::ControlServer::instance().model().calculator();
							auto& c = imp_->calculator_;

							auto js = nlohmann::json::parse(cmd_str);

							std::cout << js << std::endl;

							std::map<std::string, std::string> files;
							for (auto& node : js) {
								files[node["name"].get<std::string>()] = node["content"].get<std::string>();
								std::cout << node["content"].get<std::string>() << std::endl;
							}

							imp_->language_parser_.setProgram(files);
							imp_->language_parser_.parseLanguage();

							for (auto& str : imp_->language_parser_.varPool())
							{
								auto cut_str = [](std::string_view& input, const char* c)->std::string_view
								{
									// 此时c中字符是或的关系 //
									auto point = input.find_first_of(c);
									auto ret = input.substr(0, point);
									input = point == std::string::npos ? std::string_view() : input.substr(point);
									return ret;
								};
								auto trim_left = [](std::string_view& input, const char* c)->std::string_view
								{
									auto point = input.find_first_not_of(c);
									return point == std::string::npos ? std::string_view() : input.substr(point, std::string::npos);
								};

								std::string_view input = str;
								if (auto var = cut_str(input, " "); var.empty())THROW_FILE_LINE("invalid command string: please at least contain a word");
								input = trim_left(input, " ");

								auto type = cut_str(input, " ");
								input = trim_left(input, " ");

								auto name = cut_str(input, " =");
								input = trim_left(input, " =");

								auto value = input;
								c.addVariable(name, type, c.calculateExpression(std::string(type) + "(" + std::string(value) + ")").second);
							}

							return send_code_and_msg(0, std::string());
						}
						catch (std::exception& e)
						{
							ARIS_COUT << e.what() << std::endl;
							ARIS_LOG(aris::core::LogLvl::kError, 0, { "pro ---%s" }, e.what());
							return send_code_and_msg(aris::plan::Plan::PROGRAM_EXCEPTION, e.what());
						}
					}
				}
				else if (param == "goto")
				{
					if (!isAutoMode())
					{
						return send_code_and_msg(aris::plan::Plan::PROGRAM_EXCEPTION, "can not goto in manual mode");
					}
					else if (isAutoRunning())
					{
						return send_code_and_msg(aris::plan::Plan::PROGRAM_EXCEPTION, "can not goto when running");
					}
					else
					{
						try
						{
							auto file = value.substr(0, value.find_first_of("."));
							auto line = std::stoi(std::string(value.substr(value.find_first_of(".") + 1)));

							imp_->language_parser_.gotoFileLine(std::string(file) + ".aris", line);
							std::unique_lock<std::mutex> lck(this->imp_->auto_mu_);
							imp_->current_line_ = imp_->language_parser_.currentLine();
							imp_->current_file_ = imp_->language_parser_.currentFile();
							return send_code_and_msg(0, "");
						}
						catch (std::exception& e)
						{
							return send_code_and_msg(-1, e.what());
						}
					}
				}
				else if (param == "goto_main")
				{
					if (!isAutoMode())
					{
						return send_code_and_msg(aris::plan::Plan::PROGRAM_EXCEPTION, "can not goto in manual mode");
					}
					else if (isAutoRunning())
					{
						return send_code_and_msg(aris::plan::Plan::PROGRAM_EXCEPTION, "can not goto when running");
					}
					else
					{
						imp_->language_parser_.gotoMain();
						std::unique_lock<std::mutex> lck(this->imp_->auto_mu_);
						imp_->current_line_ = imp_->language_parser_.currentLine();
						imp_->current_file_ = imp_->language_parser_.currentFile();
						return send_code_and_msg(0, "");
					}
				}
				else if (param == "forward")
				{
					if (!isAutoMode())
					{
						return send_code_and_msg(aris::plan::Plan::PROGRAM_EXCEPTION, "can not foward in manual mode");
					}
					else if (isAutoRunning())
					{
						return send_code_and_msg(aris::plan::Plan::PROGRAM_EXCEPTION, "can not foward when running");
					}
					else if (lastErrorCode())
					{
						return send_code_and_msg(lastErrorCode(), lastError());
					}
					else
					{
						std::swap(imp_->calculator_, dynamic_cast<aris::dynamic::Model&>(aris::server::ControlServer::instance().model()).calculator());
						auto& c = dynamic_cast<aris::dynamic::Model&>(aris::server::ControlServer::instance().model()).calculator();
						auto& cs = aris::server::ControlServer::instance();

						if (imp_->language_parser_.isEnd())
						{

						}
						else if (imp_->language_parser_.isCurrentLineKeyWord())
						{
							ARIS_PRO_COUT << imp_->language_parser_.currentLine() << "---" << imp_->language_parser_.currentCmd() << std::endl;
							//LOG_INFO << "pro " << imp_->language_parser_.currentLine() << "---" << imp_->language_parser_.currentCmd() << std::endl;
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
								catch (std::exception& e)
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
							//LOG_INFO << "pro " << imp_->language_parser_.currentLine() << "---" << imp_->language_parser_.currentCmd() << std::endl;
							imp_->language_parser_.forward();
							std::unique_lock<std::mutex> lck(this->imp_->auto_mu_);
							imp_->current_line_ = imp_->language_parser_.currentLine();
							imp_->current_file_ = imp_->language_parser_.currentFile();
						}
						else if (imp_->language_parser_.currentWord() == "set")
						{
							ARIS_PRO_COUT << imp_->language_parser_.currentLine() << "---" << imp_->language_parser_.currentCmd() << std::endl;
							ARIS_LOG(aris::core::LogLvl::kError, 0, { "pro %5d---%s" }, imp_->language_parser_.currentLine(), imp_->language_parser_.currentCmd());
							try
							{
								c.calculateExpression(imp_->language_parser_.currentParamStr());
								imp_->language_parser_.forward();
								std::unique_lock<std::mutex> lck(this->imp_->auto_mu_);
								imp_->current_line_ = imp_->language_parser_.currentLine();
							}
							catch (std::exception& e)
							{
								imp_->last_error_code_ = aris::plan::Plan::PROGRAM_EXCEPTION;
								imp_->last_error_ = e.what();
								imp_->last_error_line_ = imp_->language_parser_.currentLine();
								ARIS_PRO_COUT << imp_->last_error_line_ << "---err_code:" << imp_->last_error_code_ << "  err_msg:" << imp_->last_error_ << std::endl;
								ARIS_LOG(aris::core::LogLvl::kError, 0, { "pro %5d---err_code:%s  err_msg:" }, imp_->last_error_line_, imp_->last_error_code_, imp_->last_error_);
							}
						}
						else
						{
							auto cmd = imp_->language_parser_.currentCmd();
							auto current_line = imp_->language_parser_.currentLine();
							imp_->language_parser_.forward();
							auto next_line = imp_->language_parser_.currentLine();

							auto ret = cs.executeCmdInCmdLine(cmd, [&, current_line, next_line](aris::plan::Plan& plan)->void
								{
									std::unique_lock<std::mutex> lck(imp_->auto_mu_);
									imp_->current_line_ = next_line;
								});

							ARIS_PRO_COUT << current_line << "---" << ret->cmdId() << "---" << ret->cmdString() << std::endl;
							//LOG_INFO << "pro " << current_line << "---" << ret->cmdId() << "---" << ret->cmdString() << std::endl;

							cs.waitForAllCollection();

							// 如果因为其他轨迹出错而取消 //
							if (ret->prepareRetCode() == aris::plan::Plan::PREPARE_CANCELLED || ret->executeRetCode() == aris::plan::Plan::EXECUTE_CANCELLED)
							{
								ARIS_PRO_COUT << current_line << "---" << ret->cmdId() << "---canceled" << std::endl;
								//LOG_ERROR << "pro " << current_line << "---" << ret->cmdId() << "---canceled" << std::endl;
							}
							else if (ret->executeRetCode() < 0)
							{
								imp_->last_error_code_ = ret->executeRetCode();
								imp_->last_error_ = ret->executeRetMsg();
								imp_->last_error_line_ = current_line;
								ARIS_PRO_COUT << imp_->last_error_line_ << "---" << ret->cmdId() << "---err_code:" << imp_->last_error_code_ << "  err_msg:" << imp_->last_error_ << std::endl;
								//LOG_ERROR << "pro " << imp_->last_error_line_ << "---" << ret->cmdId() << "---err_code:" << imp_->last_error_code_ << "  err_msg:" << imp_->last_error_ << std::endl;
							}

						}

						std::swap(imp_->calculator_, dynamic_cast<aris::dynamic::Model&>(aris::server::ControlServer::instance().model()).calculator());
						return send_code_and_msg(imp_->last_error_code_, imp_->last_error_);
					}
				}
				else if (param == "start")
				{
					ARIS_LOG(aris::core::LogLvl::kInfo, 0, { "pro now start" });

					if (!isAutoMode())
					{
						return send_code_and_msg(aris::plan::Plan::PROGRAM_EXCEPTION, "can not start program in manual mode");
					}
					else if (isAutoRunning())
					{
						imp_->is_pause_.store(false);
						return send_code_and_msg(0, "");
					}
					else if (lastErrorCode())
					{
						return send_code_and_msg(lastErrorCode(), lastError());
					}
					else if (!imp_->language_parser_.hasCursor())
					{
						return send_code_and_msg(-1, "please goto main or lines");
					}
					else
					{
						imp_->is_pause_.store(false);
						imp_->is_stop_.store(false);

						imp_->auto_thread_ = std::thread([&]()->void
							{
								// 交换calculator，保证每个程序开始时的变量都是之前的 //
								std::swap(imp_->calculator_, dynamic_cast<aris::dynamic::Model&>(aris::server::ControlServer::instance().model()).calculator());
								auto& c = dynamic_cast<aris::dynamic::Model&>(aris::server::ControlServer::instance().model()).calculator();
								auto& cs = aris::server::ControlServer::instance();
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
											//LOG_INFO << "pro " << lines[i] << "---" << plans[i]->cmdId() << "---" << plans[i]->cmdString() << std::endl;
										}
										cs.waitForAllCollection();
										for (int i = 0; i < plans.size(); ++i)
										{
											// 如果因为其他轨迹出错而取消 //
											if (plans[i]->prepareRetCode() == aris::plan::Plan::PREPARE_CANCELLED || plans[i]->executeRetCode() == aris::plan::Plan::EXECUTE_CANCELLED)
											{
												ARIS_PRO_COUT << lines[i] << "---" << plans[i]->cmdId() << "---canceled" << std::endl;
												//LOG_ERROR << "pro " << lines[i] << "---" << plans[i]->cmdId() << "---canceled" << std::endl;
											}
											else if (plans[i]->executeRetCode() < 0)
											{
												imp_->last_error_code_ = plans[i]->executeRetCode();
												imp_->last_error_ = plans[i]->executeRetMsg();
												imp_->last_error_line_ = lines[i];
												ARIS_PRO_COUT << imp_->last_error_line_ << "---" << plans[i]->cmdId() << "---err_code:" << imp_->last_error_code_ << "  err_msg:" << imp_->last_error_ << std::endl;
												//LOG_ERROR << "pro " << imp_->last_error_line_ << "---" << plans[i]->cmdId() << "---err_code:" << imp_->last_error_code_ << "  err_msg:" << imp_->last_error_ << std::endl;
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
										//LOG_INFO << "pro " << imp_->language_parser_.currentLine() << "---" << imp_->language_parser_.currentCmd() << std::endl;
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
											catch (std::exception& e)
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
										//LOG_INFO << "pro " << imp_->language_parser_.currentLine() << "---" << imp_->language_parser_.currentCmd() << std::endl;
										imp_->language_parser_.forward();
										std::unique_lock<std::mutex> lck(imp_->auto_mu_);
										imp_->current_line_ = imp_->language_parser_.currentLine();
										imp_->current_file_ = imp_->language_parser_.currentFile();
									}
									else if (imp_->language_parser_.currentWord() == "set")
									{
										if (server_execute())continue;
										ARIS_PRO_COUT << imp_->language_parser_.currentLine() << "---" << imp_->language_parser_.currentCmd() << std::endl;
										//LOG_INFO << "pro " << imp_->language_parser_.currentLine() << "---" << imp_->language_parser_.currentCmd() << std::endl;
										try
										{
											c.calculateExpression(imp_->language_parser_.currentParamStr());
											imp_->language_parser_.forward();
											std::unique_lock<std::mutex> lck(this->imp_->auto_mu_);
											imp_->current_line_ = imp_->language_parser_.currentLine();
										}
										catch (std::exception& e)
										{
											imp_->last_error_code_ = aris::plan::Plan::PROGRAM_EXCEPTION;
											imp_->last_error_ = e.what();
											imp_->last_error_line_ = imp_->language_parser_.currentLine();
											ARIS_PRO_COUT << imp_->last_error_line_ << "---err_code:" << imp_->last_error_code_ << "  err_msg:" << imp_->last_error_ << std::endl;
											//LOG_ERROR << "pro " << imp_->last_error_line_ << "---err_code:" << imp_->last_error_code_ << "  err_msg:" << imp_->last_error_ << std::endl;
											has_error = -1;
										}
									}
									else
									{
										auto cmd = imp_->language_parser_.currentCmd();
										auto current_line = imp_->language_parser_.currentLine();
										imp_->language_parser_.forward();
										auto next_line = imp_->language_parser_.currentLine();

										cmd_vec.push_back(std::pair<std::string, std::function<void(aris::plan::Plan&)>>(cmd, [&, current_line, next_line](aris::plan::Plan& plan)->void
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

								//std::swap(imp_->calculator_, aris::server::ControlServer::instance().model().calculator());
								ARIS_PRO_COUT << "---" << (imp_->is_stop_.load() ? "program stopped" : "program finished") << std::endl;
								//LOG_INFO << "pro " << "---" << (imp_->is_stop_.load() ? "program stopped" : "program finished") << std::endl;

								while (!imp_->auto_thread_.joinable());
								imp_->auto_thread_.detach();
							});
						return send_code_and_msg(0, "");
					}
				}
				else if (param == "stop")
				{
					if (!isAutoMode())
					{
						return send_code_and_msg(aris::plan::Plan::PROGRAM_EXCEPTION, "can not stop program in manual mode");
					}
					else
					{
						imp_->is_stop_.store(true);
						return send_code_and_msg(0, "");
					}
				}
				else if (param == "pause")
				{
					if (!isAutoMode())
					{
						return send_code_and_msg(aris::plan::Plan::PROGRAM_EXCEPTION, "can not stop program in manual mode");
					}
					else if (!isAutoRunning())
					{
						return send_code_and_msg(aris::plan::Plan::PROGRAM_EXCEPTION, "can not stop program when not running");
					}
					else
					{
						imp_->is_pause_.store(true);
						return send_code_and_msg(0, "");
					}
				}
				else if (param == "clear_error")
				{
					if (isAutoRunning())
					{
						return send_code_and_msg(aris::plan::Plan::PROGRAM_EXCEPTION, "can not clear error when running");
					}
					else
					{
						imp_->last_error_.clear();
						imp_->last_error_code_ = 0;
						imp_->last_error_line_ = 0;
						return send_code_and_msg(0, "");
					}
				}
				else
				{
					return send_code_and_msg(aris::plan::Plan::PROGRAM_EXCEPTION, "invalid program option");
				}
			}
		}
		else if (cmd == "program_file") {
			for (auto& [param, value] : params) {
				if (param == "get") {
					auto ret = fetchPrograms();
					return send_code_and_msg(0, ret);
				}
				else if (param == "post") {
					auto ret = createProgram(std::string(params.at("data")));
					return send_code_and_msg(0, ret);
				}
				else if (param == "put") {

					return send_code_and_msg(0, "");
				}
				else if (param == "delete") {

					return send_code_and_msg(0, "");
				}
				else if (param == "patch") {

					return send_code_and_msg(0, "");
				}
				else
				{
					return send_code_and_msg(aris::plan::Plan::PROGRAM_EXCEPTION, "invalid program option");
				}
			}
		}
		else
		{
			aris::server::ControlServer::instance().executeCmdInCmdLine(std::string(str), [send_ret](aris::plan::Plan& plan)->void
				{
					// only copy if it is a str
					if (auto js = std::any_cast<std::vector<std::pair<std::string, std::any>>>(&plan.ret()))
					{
						js->push_back(std::make_pair<std::string, std::any>("return_code", plan.executeRetCode()));
						js->push_back(std::make_pair<std::string, std::any>("return_message", std::string(plan.executeRetMsg())));
						send_ret(aris::server::parse_ret_value(*js));
					}
					else
					{
						std::vector<std::pair<std::string, std::any>> ret_js;
						ret_js.push_back(std::make_pair<std::string, std::any>("return_code", plan.executeRetCode()));
						ret_js.push_back(std::make_pair<std::string, std::any>("return_message", std::string(plan.executeRetMsg())));
						send_ret(aris::server::parse_ret_value(ret_js));
					}
				});
		}

		return 0;
	}
	ProgramMiddleware::ProgramMiddleware() :imp_(new Imp)
	{
		aris::core::Command cmd;
		aris::core::fromXmlString(cmd,
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
		imp_->command_parser_.commandPool().push_back(cmd);

		aris::core::fromXmlString(cmd,
			"<Command name=\"program_file\">"
			"	<Param name=\"get\"/>"
			"	<Param name=\"post\"/>"
			"	<Param name=\"put\"/>"
			"	<Param name=\"delete\"/>"
			"	<Param name=\"patch\"/>"
			"</Command>");

		imp_->command_parser_.init();
	}
	ProgramMiddleware::ProgramMiddleware(ProgramMiddleware&& other) = default;
	ProgramMiddleware& ProgramMiddleware::operator=(ProgramMiddleware&& other) = default;
	ProgramMiddleware::~ProgramMiddleware() = default;

	ARIS_REGISTRATION{
		aris::core::class_<MiddleWare>("MiddleWare")
			;

		aris::core::class_<ProgramMiddleware>("ProgramMiddleware")
			.inherit<MiddleWare>()
			;
	}

}   // namespace aris::server