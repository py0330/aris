#include "aris_core_command.h"

#include <map>
#include <string>
#include <sstream>
#include <iostream>
#include <functional>

namespace aris
{
	namespace core
	{
		class ParamBase;
		class Param;
		
		
		struct ParamBase::Imp 
		{ 
			bool is_taken_{ false };
		};
		auto ParamBase::command()const->const Command &
		{
			if (dynamic_cast<const Command *>(&father()))
				return static_cast<const Command &>(father());
			else if (dynamic_cast<const ParamBase *>(&father()))
				return static_cast<const ParamBase &>(father()).command();
			else
				throw std::runtime_error("failed to find father command, please check the command tree");
		};
		auto ParamBase::isTaken()->bool { return imp_->is_taken_; }
		auto ParamBase::take()->void 
		{
			imp_->is_taken_ = true;
			if (dynamic_cast<Command *>(&father()))
				dynamic_cast<Command *>(&father())->take();
			else if (dynamic_cast<ParamBase *>(&father()))
				dynamic_cast<ParamBase *>(&father())->take();
			else
				throw std::runtime_error("wrong command setting: unknown father type.");
		}
		auto ParamBase::reset()->void { imp_->is_taken_ = false; }
		ParamBase::ParamBase(Object &father, std::size_t id, const std::string &name) :ObjectPool(father, id, name) {}
		ParamBase::ParamBase(Object &father, std::size_t id, const aris::core::XmlElement &xml_ele) :ObjectPool(father, id, xml_ele) {}
		
		auto GroupParam::take()->void {
			if (!isTaken())ParamBase::take();
		}
		auto GroupParam::reset()->void { for (auto &child : *this)dynamic_cast<ParamBase&>(child).reset(); ParamBase::reset(); };
		auto GroupParam::addDefaultParam(std::map<std::string, std::string> &param_map_out)->void 
		{
			for (auto &sub_param : *this) { sub_param.addDefaultParam(param_map_out); }
		}
		GroupParam::~GroupParam() = default;
		GroupParam::GroupParam(Object &father, std::size_t id, const std::string &name) :ParamBase(father, id, name) {}
		GroupParam::GroupParam(Object &father, std::size_t id, const aris::core::XmlElement &xml_ele) :ParamBase(father, id, xml_ele) {}
		GroupParam::GroupParam(const GroupParam &) = default;
		GroupParam::GroupParam(GroupParam &&) = default;
		GroupParam& GroupParam::operator=(const GroupParam &) = default;
		GroupParam& GroupParam::operator=(GroupParam &&) = default;

		struct Param::Imp
		{
			char abbreviation_{ 0 };
			std::string default_value_{ "" };
			std::string help_{ "" };
		};
		auto Param::defaultParam()const->const std::string &{ return imp_->default_value_; }
		auto Param::help()const->const std::string &{ return imp_->help_; }
		auto Param::take()->void {
			if (isTaken())throw std::runtime_error("parse command error: command \"" + command().name() + "\"'s param \"" + name() + "\" has been set more than once");
			ParamBase::take();
		};
		auto Param::reset()->void { ParamBase::reset(); }
		auto Param::addDefaultParam(std::map<std::string, std::string> &param_map_out)->void 
		{
			if (!isTaken())param_map_out.insert(std::make_pair(name(), imp_->default_value_));
		}
        auto Param::abbreviation()->char { return imp_->abbreviation_; }
		Param::~Param() = default;
		Param::Param(Object &father, std::size_t id, const std::string &name) :ParamBase(father, id, name) {}
		Param::Param(Object &father, std::size_t id, const aris::core::XmlElement &xml_ele) :ParamBase(father, id, xml_ele)
		{
			imp_->abbreviation_ = attributeChar(xml_ele, "abbreviation", imp_->abbreviation_);
			imp_->default_value_ = attributeString(xml_ele, "default", imp_->default_value_);
			imp_->help_ = attributeString(xml_ele, "help", imp_->help_);
		}
		Param::Param(const Param&) = default;
		Param::Param(Param&&) = default;
		Param& Param::operator=(const Param&) = default;
		Param& Param::operator=(Param&&) = default;

		struct UniqueParam::Imp
		{
			std::string default_value_{ "" };
		};
		auto UniqueParam::defaultParam()const->const std::string &{ return imp_->default_value_; }
		auto UniqueParam::take()->void 
		{
			if (isTaken())throw std::runtime_error("parse command error: command \"" + command().name() + "\"'s UNIQUE param \"" + name() + "\" has been set more than once");
			ParamBase::take();
		}
        auto UniqueParam::reset()->void { for (auto &child : *this)dynamic_cast<ParamBase&>(child).reset(); ParamBase::reset(); }
		auto UniqueParam::addDefaultParam(std::map<std::string, std::string> &param_map_out)->void 
		{
			if (size() == 0)return;
			
			auto default_param_iter = std::find_if(begin(), end(), [](ParamBase &param)->bool { return param.isTaken(); });
			auto default_param_ptr = imp_->default_value_ == "" ? nullptr : &*findByName(imp_->default_value_);
			auto default_param = default_param_iter == end() ? default_param_ptr : &*default_param_iter;
			default_param = size() == 1 ? &front() : default_param;

			if (!default_param)throw std::runtime_error("failed to find default param in command \"" + command().name() + "\" param \"" + name() + "\"");

			default_param->addDefaultParam(param_map_out);
		}
		UniqueParam::~UniqueParam() = default;
		UniqueParam::UniqueParam(Object &father, std::size_t id, const std::string &name) :ParamBase(father, id, name) {}
		UniqueParam::UniqueParam(Object &father, std::size_t id, const aris::core::XmlElement &xml_ele) :ParamBase(father, id, xml_ele)
		{
			imp_->default_value_ = attributeString(xml_ele, "default", imp_->default_value_);
		}
		UniqueParam::UniqueParam(const UniqueParam &) = default;
		UniqueParam::UniqueParam(UniqueParam &&) = default;
		UniqueParam& UniqueParam::operator=(const UniqueParam &) = default;
		UniqueParam& UniqueParam::operator=(UniqueParam &&) = default;

		struct Command::Imp
		{
			bool is_taken_;
			std::string default_value_{ "" };
			std::string help_{ "" };
			std::map<std::string, Param*> param_map_;
			std::map<char, std::string> abbreviation_map_;
		};
		auto Command::defaultParam()const->const std::string &{ return imp_->default_value_; }
		auto Command::reset()->void 
		{ 
			imp_->is_taken_ = false;
			for (auto &child : *this)dynamic_cast<ParamBase&>(child).reset();
		}
		auto Command::take()->void
		{
			if (imp_->is_taken_) throw std::runtime_error("invalid param: some params of command \"" + name() + "\" has been set more than once");
			imp_->is_taken_ = true;
		};
		auto Command::addDefaultParam(std::map<std::string, std::string> &param_map_out)->void
		{
			if (size() == 0)return;
			
			auto default_param_iter = std::find_if(begin(), end(), [](ParamBase &param)->bool{ return param.isTaken(); });
			auto default_param_ptr = imp_->default_value_ == "" ? nullptr : &*findByName(imp_->default_value_);
			auto default_param = default_param_iter == end() ? default_param_ptr : &*default_param_iter;
			default_param = size() == 1 ? &front() : default_param;

			if (!default_param)throw std::runtime_error("failed to find default param in command \"" + name() +"\"");

			default_param->addDefaultParam(param_map_out);
		}
		auto Command::help()const->std::string
		{
			int maxPrintLength = 55;
            std::string helpstring{};

			int commandLength = imp_->help_.length();
			int count = 0;
            helpstring= name() + " : ";
			while (commandLength>maxPrintLength)
			{
                helpstring+= imp_->help_.substr(maxPrintLength*count, maxPrintLength*(count + 1));
                helpstring+= "\n" + std::string(name().length() + 2, ' ');
				count += 1;
				commandLength -= maxPrintLength;
			}
            helpstring+= imp_->help_.substr(maxPrintLength*count, std::string::npos) + "\n";

			if (imp_->param_map_.size() != 0)
			{
                helpstring+= "\n  usage: [command] [param1]=[value1] [param2]=[value2]....\n";
				int paramPrintLength = std::max_element(imp_->param_map_.begin(), imp_->param_map_.end(), [](decltype(*imp_->param_map_.begin()) a, decltype(*imp_->param_map_.begin()) b)
				{
					return a.second->name().length() < b.second->name().length();
				})->second->name().length() + 6;
                helpstring+= "\n  params help: [param] : [abbr]  [help]\n";
				for (auto &param : imp_->param_map_)
				{
					int helpStringLength = param.second->help().length();

					int count = 0;
                    helpstring+= std::string(paramPrintLength - param.second->name().length(), ' ') + param.second->name() + " : " + param.second->abbreviation() + "  ";
					while (helpStringLength>maxPrintLength)
					{
                        helpstring+= param.second->help().substr(maxPrintLength*count, maxPrintLength*(count + 1));
                        helpstring+= "\n" + std::string(paramPrintLength + 6, ' ');
						count += 1;
						helpStringLength -= maxPrintLength;
					}
                    helpstring+= param.second->help().substr(maxPrintLength*count, std::string::npos) + "\n";
				}
			}

            return helpstring;
		}
		Command::~Command() = default;
        Command::Command(Object &father, std::size_t id, const std::string &name) :ObjectPool(father, id, name) {}
		Command::Command(Object &father, std::size_t id, const aris::core::XmlElement &xml_ele) :ObjectPool(father, id, xml_ele)
		{
			imp_->default_value_ = attributeString(xml_ele, "default", imp_->default_value_);
			imp_->help_ = attributeString(xml_ele, "help", imp_->help_);
			
			if(imp_->default_value_ != "" && (findByName(imp_->default_value_) == end()))
				throw std::runtime_error("Command \"" + name() + "\" does not has valid default param name");

			std::function<void(ParamBase &)> add_param_map_and_check_default = [this, &add_param_map_and_check_default](ParamBase &param)
			{
				auto unique = dynamic_cast<UniqueParam*>(&param);
				if (unique && (unique->imp_->default_value_!="") && (unique->findByName(unique->imp_->default_value_)== unique->end()))
				{
					throw std::runtime_error("Unique param \"" + unique->name() + "\" does not has valid default param name");
				}
				
				if (dynamic_cast<Param*>(&param))
				{
					if (imp_->param_map_.find(param.name()) != imp_->param_map_.end())
						throw std::runtime_error("failed to add param \"" + param.name() +"\" to cmd \"" + this->name() + "\", because this param already exists");
					if (imp_->abbreviation_map_.find(dynamic_cast<Param&>(param).abbreviation())!= imp_->abbreviation_map_.end())
						throw std::runtime_error("failed to add param \"" + param.name() + "\" to cmd \"" + this->name() + "\", because its abbreviation already exists");

					imp_->param_map_.insert(std::make_pair(param.name(), dynamic_cast<Param*>(&param)));
					imp_->abbreviation_map_.insert(std::make_pair(dynamic_cast<Param&>(param).abbreviation(), param.name()));
				}
				else
					for (auto &sub_param : param) add_param_map_and_check_default(sub_param);
			};
			for (auto &param : *this) add_param_map_and_check_default(param);
		}
		Command::Command(const Command &)=default;
		Command::Command(Command &&) = default;
		Command& Command::operator=(const Command &) = default;
		Command& Command::operator=(Command &&) = default;

		struct CommandParser::Imp
		{
			ObjectPool<Command>* command_pool_;
		};
		auto CommandParser::parse(const std::string &command_string, std::string &cmd_out, std::map<std::string, std::string> &param_map_out)->void
		{
			try
			{
				/// 将msg转换成cmd和一系列参数，不过这里的参数为原生字符串，既包括名称也包含值，例如“-heigt=0.5” ///
				std::stringstream input_stream{ command_string };
				std::string word;
				std::vector<std::string> param_vec;
				std::size_t param_num{ 0 };

				if (!(input_stream >> cmd_out))
				{
					throw std::runtime_error("invalid command string: please at least contain a word");
				};

				auto command = imp_->command_pool_->findByName(cmd_out);
				if (command == imp_->command_pool_->end())
					throw std::runtime_error("invalid command name: server does not have this command \"" + cmd_out + "\"");
				command->reset();

				while (input_stream >> word)
				{
					std::string str{ word };
					std::string param_name, param_value;
					if (str.find("=") == std::string::npos)
					{
						param_name = str;
						param_value = "";
					}
					else
					{
						param_name.assign(str, 0, str.find("="));
						param_value.assign(str, str.find("=") + 1, str.size() - str.find("="));
					}

					if (param_name.size() == 0)
						throw std::runtime_error("invalid param: what the hell, param should not start with '='");

					/// not start with '-' ///
					if (param_name.data()[0] != '-')
					{
						if (param_value != "")
						{
							throw std::runtime_error("invalid param: only param start with - or -- can be assigned a value");
						}

						for (auto c : param_name)
						{
							if (command->imp_->abbreviation_map_.find(c) != command->imp_->abbreviation_map_.end())
							{
								param_map_out.insert(make_pair(command->imp_->abbreviation_map_.at(c), param_value));
								command->imp_->param_map_.at(command->imp_->abbreviation_map_.at(c))->take();
							}
							else
							{
								throw std::runtime_error(std::string("invalid param: param \"") + c + "\" is not a abbreviation of any valid param");
							}
						}

						continue;
					}

					/// all following part start with at least one '-' ///
					if (param_name.size() == 1)
					{
						throw std::runtime_error("invalid param: symbol \"-\" must be followed by an abbreviation of param");
					}

					/// start with '-', but only one '-' ///
					if (param_name.data()[1] != '-')
					{
						if (param_name.size() != 2)
						{
							throw std::runtime_error("invalid param: param start with single '-' must be an abbreviation");
						}

						char c = param_name.data()[1];

						if (command->imp_->abbreviation_map_.find(c) != command->imp_->abbreviation_map_.end())
						{
							param_map_out.insert(make_pair(command->imp_->abbreviation_map_.at(c), param_value));
							command->imp_->param_map_.at(command->imp_->abbreviation_map_.at(c))->take();
						}
						else
						{
							throw std::runtime_error(std::string("invalid param: param \"") + c + "\" is not a abbreviation of any valid param");
						}

						continue;
					}
					else
					{
						/// start with '--' ///
						if (param_name.size()<3)
						{
							throw std::runtime_error("invalid param: symbol \"--\" must be followed by a full name of param");
						}

						std::string str = param_name;
						param_name.assign(str, 2, str.size() - 2);

						if (command->imp_->param_map_.find(param_name) != command->imp_->param_map_.end())
						{
							param_map_out.insert(make_pair(param_name, param_value));
							command->imp_->param_map_.at(param_name)->take();
						}
						else
						{
							throw std::runtime_error(std::string("invalid param: param \"") + param_name + "\" is not a valid param");
						}

						continue;
					}
				}

				command->addDefaultParam(param_map_out);
			}
			catch (std::exception &e)
			{
				throw std::runtime_error(e.what() + std::string(", when parsing command string \"" + command_string +"\""));
			}
		}
        auto CommandParser::commandPool()->ObjectPool<Command> & { return *imp_->command_pool_; }
        auto CommandParser::commandPool()const->const ObjectPool<Command> &{ return *imp_->command_pool_; }
        auto CommandParser::help()const->std::string
        {			
			int maxPrintLength = 55;
            std::string helpstring{};

            helpstring="all command: \n";
			int commandPrintLength = std::max_element(imp_->command_pool_->begin(), imp_->command_pool_->end(), [](decltype(*imp_->command_pool_->begin()) a, decltype(*imp_->command_pool_->begin()) b)
			{
				return a.name().length() < b.name().length();
			})->name().length() + 2;
			for (auto &command : *imp_->command_pool_)
			{
				int helpStringLength = command.help().length();
				int count = 0;
                helpstring+=std::string(commandPrintLength - command.name().length(), ' ') + command.name() + " : ";
				while (helpStringLength>maxPrintLength)
				{
                    helpstring+=command.help().substr(maxPrintLength*count, maxPrintLength*(count + 1));
                    helpstring+="\n" + std::string(commandPrintLength + 2, ' ');
					count += 1;
					helpStringLength -= maxPrintLength;
				}
                helpstring+=command.help().substr(maxPrintLength*count, std::string::npos) + "\n";
			}
            helpstring+="\nAttention:\n";
            helpstring+="the param '--help(-h)' can get more help about the specify command. etc: en -h \n";

            return helpstring;
        }
		CommandParser::~CommandParser() = default;
		CommandParser::CommandParser(Object &father, std::size_t id, const std::string &name):Object(father, id, name)
		{ 
			imp_->command_pool_ = &add<aris::core::ObjectPool<Command> >("command_pool");
		}
		CommandParser::CommandParser(Object &father, std::size_t id, const aris::core::XmlElement &xml_ele) :Object(father, id, xml_ele) 
		{
			imp_->command_pool_ = findByName("command_pool") == end() ? &add<aris::core::ObjectPool<Command>>("command_pool") : static_cast<aris::core::ObjectPool<Command> *>(&(*findByName("command_pool")));
		}
		CommandParser::CommandParser(const CommandParser &) = default;
		CommandParser::CommandParser(CommandParser &&) = default;
		CommandParser& CommandParser::operator=(const CommandParser &) = default;
		CommandParser& CommandParser::operator=(CommandParser &&) = default;
	}
}

