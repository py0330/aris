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

	struct ParamBase::Imp { bool is_taken_{ false }; };
	auto ParamBase::command()const->const Command & {
		if (auto c = dynamic_cast<const Command *>(father()))
			return *c;
		else if (auto p = dynamic_cast<const ParamBase *>(father()))
			return p->command();
		else
			throw std::runtime_error("failed to find father command, please check the command tree");
	};
	ParamBase::~ParamBase() = default;
	ParamBase::ParamBase(const std::string &name) :ObjectPool(name), imp_(new Imp) {}
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
	Param::~Param() = default;
	Param::Param(const std::string &name, const std::string &default_param, char abbrev) :ParamBase(name), imp_(new Imp(default_param, abbrev)) {}
	Param::Param(const Param&) = default;
	Param::Param(Param&&) = default;
	Param& Param::operator=(const Param&) = default;
	Param& Param::operator=(Param&&) = default;

	GroupParam::~GroupParam() = default;
	GroupParam::GroupParam(const std::string &name) :ParamBase(name) {}
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
	UniqueParam::~UniqueParam() = default;
	UniqueParam::UniqueParam(const std::string &name, const std::string &default_param) :ParamBase(name), imp_(new Imp(default_param)) {}
	UniqueParam::UniqueParam(const UniqueParam &) = default;
	UniqueParam::UniqueParam(UniqueParam &&) = default;
	UniqueParam& UniqueParam::operator=(const UniqueParam &) = default;
	UniqueParam& UniqueParam::operator=(UniqueParam &&) = default;

	struct Command::Imp
	{
		bool is_taken_;
		std::string default_value_{ "" };
		std::map<std::string, Param*> param_map_;
		std::map<char, std::string> abbreviation_map_;

		Imp(const std::string &default_param = std::string("")) :default_value_(default_param) {}

		static auto take(Object* param)->void
		{
			if (auto p = dynamic_cast<Param*>(param))
			{
				if (p->ParamBase::imp_->is_taken_)
					throw std::runtime_error("parse command error: command \"" + p->command().name() + "\"'s param \"" + p->name() + "\" has been set more than once");
				p->ParamBase::imp_->is_taken_ = true;
				take(param->father());
			}
			else if (auto g = dynamic_cast<GroupParam*>(param))
			{
				if (!g->ParamBase::imp_->is_taken_)
				{
					g->ParamBase::imp_->is_taken_ = true;
					take(param->father());
				}
			}
			else if (auto u = dynamic_cast<UniqueParam*>(param))
			{
				if (u->ParamBase::imp_->is_taken_)
					throw std::runtime_error("parse command error: command \"" + u->command().name() + "\"'s UNIQUE param \"" + u->name() + "\" has been set more than once");
				u->ParamBase::imp_->is_taken_ = true;
				take(param->father());
			}
			else if (auto c = dynamic_cast<Command*>(param))
			{
				if (c->imp_->is_taken_)
					throw std::runtime_error("invalid param: some params of command \"" + c->name() + "\" has been set more than once");
				c->imp_->is_taken_ = true;
			}
			else
			{
				throw std::runtime_error("wrong type when cmd parse in take");
			}
		}
		static auto reset(Object* param)->void
		{
			if (auto p = dynamic_cast<Param*>(param))
			{
				p->ParamBase::imp_->is_taken_ = false;
			}
			else if (auto g = dynamic_cast<GroupParam*>(param))
			{
				g->ParamBase::imp_->is_taken_ = false;
				for (auto &child : *g)reset(&child);
			}
			else if (auto u = dynamic_cast<UniqueParam*>(param))
			{
				u->ParamBase::imp_->is_taken_ = false;
				for (auto &child : *u)reset(&child);
			}
			else if (auto c = dynamic_cast<Command*>(param))
			{
				c->imp_->is_taken_ = false;
				for (auto &child : *c)reset(&child);
			}
			else
			{
				throw std::runtime_error("wrong type when cmd parse in reset");
			}
		}
		static auto addDefaultParam(Object* param, std::map<std::string, std::string> &param_map_out)->void
		{
			if (auto p = dynamic_cast<Param*>(param))
			{
				if (!p->ParamBase::imp_->is_taken_)param_map_out.insert(std::make_pair(p->name(), p->imp_->default_value_));
			}
			else if (auto g = dynamic_cast<GroupParam*>(param))
			{
				for (auto &child : *g) { addDefaultParam(&child, param_map_out); }
			}
			else if (auto u = dynamic_cast<UniqueParam*>(param))
			{
				if (u->size() == 0)return;

				auto default_param_iter = std::find_if(u->begin(), u->end(), [](ParamBase &param)->bool { return param.imp_->is_taken_; });
				auto default_param_ptr = u->imp_->default_value_ == "" ? nullptr : &*u->findByName(u->imp_->default_value_);
				auto default_param = default_param_iter == u->end() ? default_param_ptr : &*default_param_iter;
				default_param = u->size() == 1 ? &u->front() : default_param;

				if (!default_param)throw std::runtime_error("failed to find default param in command \"" + u->command().name() + "\" param \"" + u->name() + "\"");

				addDefaultParam(default_param, param_map_out);
			}
			else if (auto c = dynamic_cast<Command*>(param))
			{
				if (c->size() == 0)return;

				auto default_param_iter = std::find_if(c->begin(), c->end(), [](ParamBase &param)->bool { return param.imp_->is_taken_; });
				auto default_param_ptr = c->imp_->default_value_ == "" ? nullptr : &*c->findByName(c->imp_->default_value_);
				auto default_param = default_param_iter == c->end() ? default_param_ptr : &*default_param_iter;
				default_param = c->size() == 1 ? &c->front() : default_param;

				if (!default_param)throw std::runtime_error("failed to find default param in command \"" + c->name() + "\"");

				addDefaultParam(default_param, param_map_out);
			}
			else
			{
				throw std::runtime_error("wrong type when cmd parse in addDefaultParam");
			}
		}
		static auto add_param_map_and_check_default(Command *cmd, ParamBase &param)->void
		{
			if (auto p = dynamic_cast<Param*>(&param))
			{
				if (cmd->imp_->param_map_.find(param.name()) != cmd->imp_->param_map_.end())
					throw std::runtime_error("failed to add param \"" + param.name() + "\" to cmd \"" + cmd->name() + "\", because this param already exists");
				if (cmd->imp_->abbreviation_map_.find(p->abbreviation()) != cmd->imp_->abbreviation_map_.end() && p->abbreviation() != 0)
					throw std::runtime_error("failed to add param \"" + param.name() + "\" to cmd \"" + cmd->name() + "\", because its abbreviation already exists");

				cmd->imp_->param_map_.insert(std::make_pair(param.name(), p));
				cmd->imp_->abbreviation_map_.insert(std::make_pair(p->abbreviation(), param.name()));
			}
			else if (auto u = dynamic_cast<UniqueParam*>(&param))
			{
				if ((u->imp_->default_value_ != "") && (u->findByName(u->imp_->default_value_) == u->end()))
				{
					throw std::runtime_error("Unique param \"" + u->name() + "\" has invalid default param name");
				}
				for (auto &sub_param : param) add_param_map_and_check_default(cmd, sub_param);
			}
			else if (auto g = dynamic_cast<GroupParam*>(&param))
			{
				for (auto &sub_param : param) add_param_map_and_check_default(cmd, sub_param);
			}
		}
	};
	auto Command::saveXml(aris::core::XmlElement &xml_ele) const->void
	{
		ObjectPool<ParamBase>::saveXml(xml_ele);
		if (!imp_->default_value_.empty())xml_ele.SetAttribute("default", imp_->default_value_.c_str());
	}
	auto Command::loadXml(const aris::core::XmlElement &xml_ele)->void
	{
		ObjectPool<ParamBase>::loadXml(xml_ele);
		imp_->default_value_ = attributeString(xml_ele, "default", imp_->default_value_);
	}
	auto Command::defaultParam()const->const std::string & { return imp_->default_value_; }
	Command::~Command() = default;
	Command::Command(const std::string &name, const std::string &default_param) :ObjectPool(name), imp_(new Imp(default_param))
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
				if (word == std::string(1, '\0')) break; // 这意味着结束
				
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

