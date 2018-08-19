#include "aris_core_command.h"

#include <map>
#include <string>
#include <sstream>
#include <iostream>
#include <functional>

namespace aris::core
{
	class ParamBase;
	class Param;

	auto formatString(std::string originalString, int begin)->std::string
	{
		std::string formatStr{};
		int maxPrintLength = 70;

		int count = 0;
		std::size_t width = maxPrintLength - begin;
		std::size_t StringLength = originalString.length();
		while (StringLength>width)
		{
			formatStr += std::string(begin, ' ') + originalString.substr(width*count, width*(count + 1)) + "\n";
			count += 1;
			StringLength -= width;
		}
		formatStr += std::string(begin, ' ') + originalString.substr(width*count, std::string::npos) + "\n";
		return formatStr;
	}

	struct ParamBase::Imp
	{
		bool is_taken_{ false };
		std::string help_{ "" };

		Imp(const std::string &help = std::string("")) :help_(help) {}
	};
	auto ParamBase::saveXml(aris::core::XmlElement &xml_ele) const->void
	{
		ObjectPool<ParamBase>::saveXml(xml_ele);
		if (!imp_->help_.empty())xml_ele.SetAttribute("help", imp_->help_.c_str());
	}
	auto ParamBase::loadXml(const aris::core::XmlElement &xml_ele)->void
	{
		ObjectPool<ParamBase>::loadXml(xml_ele);
		imp_->help_ = attributeString(xml_ele, "help", imp_->help_);
	}
	auto ParamBase::command()const->const Command & {
		if (dynamic_cast<const Command *>(&father()))
			return static_cast<const Command &>(father());
		else if (dynamic_cast<const ParamBase *>(&father()))
			return static_cast<const ParamBase &>(father()).command();
		else
			throw std::runtime_error("failed to find father command, please check the command tree");
	};
	auto ParamBase::simpleHelp()const->const std::string & { return imp_->help_; }
	ParamBase::~ParamBase() = default;
	ParamBase::ParamBase(const std::string &name, const std::string &help) :ObjectPool(name), imp_(new Imp(help)) {}
	ParamBase::ParamBase(const ParamBase&) = default;
	ParamBase::ParamBase(ParamBase&&) = default;
	ParamBase& ParamBase::operator=(const ParamBase&) = default;
	ParamBase& ParamBase::operator=(ParamBase&&) = default;

	struct Param::Imp
	{
		std::string default_value_{ "" };
		char abbreviation_{ 0 };

		Imp(const std::string &default_param = std::string(""), char abbrev = 0) :default_value_(default_param), abbreviation_(abbrev) {}
	};
	auto Param::saveXml(aris::core::XmlElement &xml_ele) const->void
	{
		ParamBase::saveXml(xml_ele);
		char abbrev[2]{ imp_->abbreviation_,0 };
		if (imp_->abbreviation_)xml_ele.SetAttribute("abbreviation", abbrev);
		if (!imp_->default_value_.empty())xml_ele.SetAttribute("default", imp_->default_value_.c_str());
	}
	auto Param::loadXml(const aris::core::XmlElement &xml_ele)->void
	{
		ParamBase::loadXml(xml_ele);
		imp_->abbreviation_ = attributeChar(xml_ele, "abbreviation", imp_->abbreviation_);
		imp_->default_value_ = attributeString(xml_ele, "default", imp_->default_value_);
	}
	auto Param::defaultParam()const->const std::string & { return imp_->default_value_; }
	auto Param::abbreviation()const->char { return imp_->abbreviation_; }
	auto Param::help(bool isfull, int begin)const->std::string
	{
		std::string helpString = "[" + name() + "]";

		if (defaultParam() != "")
		{
			helpString += "[" + defaultParam() + "]";
		}
		helpString = formatString(helpString + " " + simpleHelp(), begin + 4);
		helpString.replace(begin, 4, std::string{ '-' } +abbreviation() + ": ");

		return helpString;
	}
	Param::~Param() = default;
	Param::Param(const std::string &name, const std::string &default_param, const std::string &help, char abbrev) :ParamBase(name, help), imp_(new Imp(default_param, abbrev)) {}
	Param::Param(const Param&) = default;
	Param::Param(Param&&) = default;
	Param& Param::operator=(const Param&) = default;
	Param& Param::operator=(Param&&) = default;

	auto GroupParam::help(bool isfull, int begin)const->std::string
	{
		std::string helpString = formatString("[" + name() + "] " + simpleHelp(), begin + 4);
		if (isfull)
		{
			for (auto &subparam : *this) helpString += subparam.help(true, begin + 4);
		}
		helpString += std::string(begin, ' ') + "g>\n";
		helpString.replace(begin, 4, "<g: ");

		return helpString;
	}
	GroupParam::~GroupParam() = default;
	GroupParam::GroupParam(const std::string &name, const std::string &help) :ParamBase(name, help) {}
	GroupParam::GroupParam(const GroupParam &) = default;
	GroupParam::GroupParam(GroupParam &&) = default;
	GroupParam& GroupParam::operator=(const GroupParam &) = default;
	GroupParam& GroupParam::operator=(GroupParam &&) = default;

	struct UniqueParam::Imp
	{
		std::string default_value_{ "" };
		Imp(const std::string &default_param = std::string("")) :default_value_(default_param) {}
	};
	auto UniqueParam::saveXml(aris::core::XmlElement &xml_ele) const->void
	{
		ParamBase::saveXml(xml_ele);
		if (!imp_->default_value_.empty())xml_ele.SetAttribute("default", imp_->default_value_.c_str());
	}
	auto UniqueParam::loadXml(const aris::core::XmlElement &xml_ele)->void
	{
		ParamBase::loadXml(xml_ele);
		imp_->default_value_ = attributeString(xml_ele, "default", imp_->default_value_);
	}
	auto UniqueParam::defaultParam()const->const std::string & { return imp_->default_value_; }
	auto UniqueParam::help(bool isfull, int begin)const->std::string
	{
		std::string helpString{};

		if (defaultParam() != "")
		{
			helpString = formatString("[" + name() + "][" + defaultParam() + "] " + simpleHelp(), begin + 4);
		}
		else
		{
			helpString = formatString("[" + name() + "] " + simpleHelp(), begin + 4);
		}
		if (isfull)
		{
			for (auto &subparam : *this)
			{
				helpString += subparam.help(true, begin + 4);
			}
		}
		helpString += std::string(begin, ' ') + "u>\n";
		helpString.replace(begin, 4, "<u: ");

		return helpString;
	}
	UniqueParam::~UniqueParam() = default;
	UniqueParam::UniqueParam(const std::string &name, const std::string &default_param, const std::string &help) :ParamBase(name, help), imp_(new Imp(default_param)) {}
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

		Imp(const std::string &default_param = std::string(""), const std::string &help = std::string("")) :help_(help), default_value_(default_param) {}

		static auto take(Object* param)->void
		{
			if (dynamic_cast<Param*>(param))
			{
				auto p = dynamic_cast<Param*>(param);
				if (p->ParamBase::imp_->is_taken_)
					throw std::runtime_error("parse command error: command \"" + p->command().name() + "\"'s param \"" + p->name() + "\" has been set more than once");
				p->ParamBase::imp_->is_taken_ = true;
				take(&param->father());
			}
			else if (dynamic_cast<GroupParam*>(param))
			{
				auto p = dynamic_cast<GroupParam*>(param);
				if (!p->ParamBase::imp_->is_taken_)
				{
					p->ParamBase::imp_->is_taken_ = true;
					take(&param->father());
				}
			}
			else if (dynamic_cast<UniqueParam*>(param))
			{
				auto p = dynamic_cast<UniqueParam*>(param);
				if (p->ParamBase::imp_->is_taken_)
					throw std::runtime_error("parse command error: command \"" + p->command().name() + "\"'s UNIQUE param \"" + p->name() + "\" has been set more than once");
				p->ParamBase::imp_->is_taken_ = true;
				take(&param->father());
			}
			else if (dynamic_cast<Command*>(param))
			{
				auto p = dynamic_cast<Command*>(param);
				if (p->imp_->is_taken_)
					throw std::runtime_error("invalid param: some params of command \"" + p->name() + "\" has been set more than once");
				p->imp_->is_taken_ = true;
			}
			else
			{
				throw std::runtime_error("wrong type when cmd parse in take");
			}
		}
		static auto reset(Object* param)->void
		{
			if (dynamic_cast<Param*>(param))
			{
				dynamic_cast<Param*>(param)->ParamBase::imp_->is_taken_ = false;
			}
			else if (dynamic_cast<GroupParam*>(param))
			{
				dynamic_cast<GroupParam*>(param)->ParamBase::imp_->is_taken_ = false;
				for (auto &child : *dynamic_cast<GroupParam*>(param))reset(&child);
			}
			else if (dynamic_cast<UniqueParam*>(param))
			{
				dynamic_cast<UniqueParam*>(param)->ParamBase::imp_->is_taken_ = false;
				for (auto &child : *dynamic_cast<UniqueParam*>(param))reset(&child);
			}
			else if (dynamic_cast<Command*>(param))
			{
				dynamic_cast<Command*>(param)->imp_->is_taken_ = false;
				for (auto &child : *dynamic_cast<Command*>(param))reset(&child);
			}
			else
			{
				throw std::runtime_error("wrong type when cmd parse in reset");
			}
		}
		static auto addDefaultParam(Object* param, std::map<std::string, std::string> &param_map_out)->void
		{
			if (dynamic_cast<Param*>(param))
			{
				auto p = dynamic_cast<Param*>(param);
				if (!p->ParamBase::imp_->is_taken_)param_map_out.insert(std::make_pair(p->name(), p->imp_->default_value_));
			}
			else if (dynamic_cast<GroupParam*>(param))
			{
				auto p = dynamic_cast<GroupParam*>(param);
				for (auto &child : *p) { addDefaultParam(&child, param_map_out); }
			}
			else if (dynamic_cast<UniqueParam*>(param))
			{
				auto p = dynamic_cast<UniqueParam*>(param);

				if (p->size() == 0)return;

				auto default_param_iter = std::find_if(p->begin(), p->end(), [](ParamBase &param)->bool { return param.imp_->is_taken_; });
				auto default_param_ptr = p->imp_->default_value_ == "" ? nullptr : &*p->findByName(p->imp_->default_value_);
				auto default_param = default_param_iter == p->end() ? default_param_ptr : &*default_param_iter;
				default_param = p->size() == 1 ? &p->front() : default_param;

				if (!default_param)throw std::runtime_error("failed to find default param in command \"" + p->command().name() + "\" param \"" + p->name() + "\"");

				addDefaultParam(default_param, param_map_out);
			}
			else if (dynamic_cast<Command*>(param))
			{
				auto p = dynamic_cast<Command*>(param);
				if (p->size() == 0)return;

				auto default_param_iter = std::find_if(p->begin(), p->end(), [](ParamBase &param)->bool { return param.imp_->is_taken_; });
				auto default_param_ptr = p->imp_->default_value_ == "" ? nullptr : &*p->findByName(p->imp_->default_value_);
				auto default_param = default_param_iter == p->end() ? default_param_ptr : &*default_param_iter;
				default_param = p->size() == 1 ? &p->front() : default_param;

				if (!default_param)throw std::runtime_error("failed to find default param in command \"" + p->name() + "\"");

				addDefaultParam(default_param, param_map_out);
			}
			else
			{
				throw std::runtime_error("wrong type when cmd parse in addDefaultParam");
			}
		}
		static auto add_param_map_and_check_default(Command *cmd, ParamBase &param)->void
		{
			if (dynamic_cast<Param*>(&param))
			{
				if (cmd->imp_->param_map_.find(param.name()) != cmd->imp_->param_map_.end())
					throw std::runtime_error("failed to add param \"" + param.name() + "\" to cmd \"" + cmd->name() + "\", because this param already exists");
				if (cmd->imp_->abbreviation_map_.find(dynamic_cast<Param&>(param).abbreviation()) != cmd->imp_->abbreviation_map_.end() && dynamic_cast<Param&>(param).abbreviation() != 0)
					throw std::runtime_error("failed to add param \"" + param.name() + "\" to cmd \"" + cmd->name() + "\", because its abbreviation already exists");

				cmd->imp_->param_map_.insert(std::make_pair(param.name(), dynamic_cast<Param*>(&param)));
				cmd->imp_->abbreviation_map_.insert(std::make_pair(dynamic_cast<Param&>(param).abbreviation(), param.name()));
			}
			else if (dynamic_cast<UniqueParam*>(&param))
			{
				auto unique = dynamic_cast<UniqueParam*>(&param);
				if ((unique->imp_->default_value_ != "") && (unique->findByName(unique->imp_->default_value_) == unique->end()))
				{
					throw std::runtime_error("Unique param \"" + unique->name() + "\" has invalid default param name");
				}
				for (auto &sub_param : param) add_param_map_and_check_default(cmd, sub_param);
			}
			else if (dynamic_cast<GroupParam*>(&param))
			{
				for (auto &sub_param : param) add_param_map_and_check_default(cmd, sub_param);
			}
		}
	};
	auto Command::saveXml(aris::core::XmlElement &xml_ele) const->void
	{
		ObjectPool<ParamBase>::saveXml(xml_ele);
		if (!imp_->default_value_.empty())xml_ele.SetAttribute("default", imp_->default_value_.c_str());
		if (!imp_->help_.empty())xml_ele.SetAttribute("help", imp_->help_.c_str());
	}
	auto Command::loadXml(const aris::core::XmlElement &xml_ele)->void
	{
		ObjectPool<ParamBase>::loadXml(xml_ele);
		imp_->default_value_ = attributeString(xml_ele, "default", imp_->default_value_);
		imp_->help_ = attributeString(xml_ele, "help", imp_->help_);
	}
	auto Command::defaultParam()const->const std::string & { return imp_->default_value_; }
	auto Command::help(bool isfull, int begin)const->std::string
	{
		std::string helpString = formatString(name() + ": " + imp_->help_, begin);

		if (isfull)
		{
			for (auto &param : *this)
			{
				helpString = helpString + param.help(true, 0);
			}
		}
		return helpString;
	}
	Command::~Command() = default;
	Command::Command(const std::string &name, const std::string &default_param, const std::string &help) :ObjectPool(name), imp_(new Imp(default_param, help))
	{
		registerType<aris::core::Param>();
		registerType<aris::core::UniqueParam>();
		registerType<aris::core::GroupParam>();
	}
	Command::Command(const Command &) = default;
	Command::Command(Command &&) = default;
	Command& Command::operator=(const Command &) = default;
	Command& Command::operator=(Command &&) = default;

	struct CommandParser::Imp { ObjectPool<Command>* command_pool_; };
	auto CommandParser::loadXml(const aris::core::XmlElement &xml_ele)->void
	{
		Object::loadXml(xml_ele);
		imp_->command_pool_ = findOrInsert<aris::core::ObjectPool<Command>>("command_pool");
	}
	auto CommandParser::parse(const std::string &command_string, std::string &cmd_out, std::map<std::string, std::string> &param_out)->void
	{
		try
		{
			std::string cmd;
			std::map<std::string, std::string> param_map;
			std::stringstream input_stream{ command_string };
			std::string word;

			if (!(input_stream >> cmd))throw std::runtime_error("invalid command string: please at least contain a word");

			auto command = imp_->command_pool_->findByName(cmd);
			if (command == imp_->command_pool_->end()) throw std::runtime_error("invalid command name: server does not have this command \"" + cmd + "\"");

			// make map and abbrev map //
			command->imp_->param_map_.clear();
			command->imp_->abbreviation_map_.clear();
			for (auto &param : *command) Command::Imp::add_param_map_and_check_default(&*command, param);

			Command::Imp::reset(&*command);
			while (input_stream >> word)
			{
				std::string param_name_origin = word.substr(0, word.find_first_of('='));

				if (param_name_origin == "")throw std::runtime_error("invalid param: param should not start with '='");
				else if (param_name_origin == "-")throw std::runtime_error("invalid param: symbol \"-\" must be followed by an abbreviation of param");
				else if (param_name_origin == "--")throw std::runtime_error("invalid param: symbol \"--\" must be followed by a full name of param");
				else if (param_name_origin.size() > 2 && param_name_origin.data()[0] == '-' && param_name_origin.data()[1] != '-')throw std::runtime_error("invalid param: param start with single '-' must be an abbreviation");
				else if (param_name_origin.size() == 2 && param_name_origin.data()[0] == '-' && param_name_origin.data()[1] != '-')
				{
					char abbrev = param_name_origin.data()[1];

					if (command->imp_->abbreviation_map_.find(abbrev) == command->imp_->abbreviation_map_.end())
						throw std::runtime_error(std::string("invalid param: param \"") + abbrev + "\" is not a abbreviation of any valid param");

					auto param = command->imp_->param_map_.at(command->imp_->abbreviation_map_.at(abbrev));
					auto param_name = command->imp_->abbreviation_map_.at(abbrev);
					auto param_value = word.find('=') == std::string::npos ? param->defaultParam() : word.substr(word.find('=') + 1, std::string::npos);

					param_map.insert(make_pair(param_name, param_value));
					Command::Imp::take(param);
				}
				else if (param_name_origin.data()[0] == '-' && param_name_origin.data()[1] == '-')
				{
					auto param_name = word.substr(2, word.find('=') - 2);

					if (command->imp_->param_map_.find(param_name) == command->imp_->param_map_.end())
						throw std::runtime_error(std::string("invalid param: param \"") + param_name + "\" is not a valid param");

					auto param = command->imp_->param_map_.at(param_name);
					auto param_value = word.find('=') == std::string::npos ? param->defaultParam() : word.substr(word.find('=') + 1, std::string::npos);

					param_map.insert(make_pair(param_name, param_value));
					Command::Imp::take(param);
				}
				else
				{
					for (auto abbrev : param_name_origin)
					{
						if (command->imp_->abbreviation_map_.find(abbrev) == command->imp_->abbreviation_map_.end())
							throw std::runtime_error(std::string("invalid param: param \"") + abbrev + "\" is not a abbreviation of any valid param");

						auto param = command->imp_->param_map_.at(command->imp_->abbreviation_map_.at(abbrev));
						auto param_name = command->imp_->abbreviation_map_.at(abbrev);
						auto param_value = param->defaultParam();
						param_map.insert(make_pair(param_name, param_value));
						Command::Imp::take(param);
					}
				}
			}
			Command::Imp::addDefaultParam(&*command, param_map);

			cmd_out = cmd;
			param_out = param_map;
		}
		catch (std::exception &e)
		{
			throw std::runtime_error(e.what() + std::string(", when parsing command string \"" + command_string + "\""));
		}
	}
	auto CommandParser::commandPool()->ObjectPool<Command> & { return *imp_->command_pool_; }
	auto CommandParser::commandPool()const->const ObjectPool<Command> & { return *imp_->command_pool_; }
	auto CommandParser::help()const->std::string
	{
		std::string help_string{};

		help_string = "All command: \n";
		for (auto &command : *imp_->command_pool_)
			help_string += command.help(false, 4) + "\n";

		return help_string;

	}
	CommandParser::~CommandParser() = default;
	CommandParser::CommandParser(const std::string &name) :Object(name)
	{
		registerType<aris::core::ObjectPool<Command> >();
		registerType<Command>();

		imp_->command_pool_ = &add<aris::core::ObjectPool<Command> >("command_pool");
	}
	CommandParser::CommandParser(const CommandParser &) = default;
	CommandParser::CommandParser(CommandParser &&) = default;
	CommandParser& CommandParser::operator=(const CommandParser &) = default;
	CommandParser& CommandParser::operator=(CommandParser &&) = default;
}

