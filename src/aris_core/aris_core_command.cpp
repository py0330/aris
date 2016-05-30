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
		
		class Command :public ObjectPool<ParamBase>
		{
		public:
			static auto Type()->const std::string &{ static const std::string type("command"); return std::ref(type); }
			virtual auto type() const->const std::string&{ return Type(); }
			Command(Object &father, std::size_t id, const std::string &name):ObjectPool(father, id, name) {};
			Command(Object &father, std::size_t id, const aris::core::XmlElement &xml_ele);
			auto reset()->void;
			auto take()->void {
				if (is_taken_) throw std::runtime_error("wrong command param setting: some params of command \"" + name() +"\" has been set more than once");
				is_taken_ = true;
			};
			auto addDefaultParam(std::map<std::string, std::string> &param_map_out);

		private:
			bool is_taken_;
			std::string default_value_{""};
			std::map<std::string, Param*> param_map_;
			std::map<char, std::string> abbreviation_map_;

			friend class CommandParser;
		};
		class ParamBase :public ObjectPool<ParamBase>
		{
		public:
			auto command()const->const Command &{
				if (dynamic_cast<const Command *>(&father()))
					return static_cast<const Command &>(father());
				else if (dynamic_cast<const ParamBase *>(&father()))
					return static_cast<const ParamBase &>(father()).command();
				else
					throw std::runtime_error("failed to find father command, please check the command tree");
			};
			auto isTaken()->bool { return is_taken_; }
			virtual auto take()->void { 
				is_taken_ = true;
				if (dynamic_cast<Command *>(&father()))
					dynamic_cast<Command *>(&father())->take();
				else if (dynamic_cast<ParamBase *>(&father()))
					dynamic_cast<ParamBase *>(&father())->take();
				else
					throw std::runtime_error("wrong command setting: unknown father type.");
			}
			virtual auto reset()->void { is_taken_ = false; }
			virtual auto addDefaultParam(std::map<std::string, std::string> &param_map_out)->void = 0;

			ParamBase(Object &father, std::size_t id, const std::string &name) :ObjectPool(father, id, name) {}
			ParamBase(Object &father, std::size_t id, const aris::core::XmlElement &xml_ele) :ObjectPool(father, id, xml_ele) {}

		private:
			bool is_taken_{ false };
		};
		class Param :public ParamBase
		{
		public:
			static auto Type()->const std::string &{ static const std::string type("param"); return std::ref(type); }
			virtual auto type() const->const std::string&{ return Type(); }
			virtual auto take()->void override final {
				if (isTaken())throw std::runtime_error("parse command error: command \"" + command().name() + "\"'s param \"" + name() + "\" has been set more than once");
				ParamBase::take();
			};
			virtual auto reset()->void override final { ParamBase::reset(); }
			virtual auto addDefaultParam(std::map<std::string, std::string> &param_map_out)->void final override{
				if (!isTaken())param_map_out.insert(std::make_pair(name(), default_value_));
			}
			auto abbreviation()->char { return abbreviation_; };
			Param(Object &father, std::size_t id, const std::string &name) :ParamBase(father, id, name) {}
			Param(Object &father, std::size_t id, const aris::core::XmlElement &xml_ele) :ParamBase(father, id, xml_ele) 
			{
				abbreviation_ = attributeChar(xml_ele, "abbreviation", abbreviation_);
				default_value_ = attributeString(xml_ele, "default", default_value_);
			}

		private:
			char abbreviation_{ 0 };
			std::string default_value_{""};
		};
		class UniqueParam :public ParamBase
		{
		public:
			static auto Type()->const std::string &{ static const std::string type("unique"); return std::ref(type); }
			virtual auto type() const->const std::string&{ return Type(); }
			virtual auto take()->void override final {
				if (isTaken())throw std::runtime_error("parse command error: command \"" + command().name() + "\"'s UNIQUE param \"" + name() + "\" has been set more than once");
				ParamBase::take();
			}
			virtual auto reset()->void override final { for (auto &child : *this)dynamic_cast<ParamBase&>(child).reset(); ParamBase::reset(); };
			virtual auto addDefaultParam(std::map<std::string, std::string> &param_map_out)->void final override {
				auto default_param_iter = std::find_if(begin(), end(), [](ParamBase &param)->bool { return param.isTaken(); });
				auto default_param_ptr = default_value_ == "" ? nullptr : &*findByName(default_value_);
				auto default_param = default_param_iter == end() ? default_param_ptr : &*default_param_iter;
				default_param = size() == 1 ? &front() : default_param;

				if (!default_param)throw std::runtime_error("failed to find default param in command \"" + command().name() + "\" param \"" + name() +"\"");

				default_param->addDefaultParam(param_map_out);
			}
			UniqueParam(Object &father, std::size_t id, const std::string &name) :ParamBase(father, id, name) {}
			UniqueParam(Object &father, std::size_t id, const aris::core::XmlElement &xml_ele) :ParamBase(father, id, xml_ele) 
			{
				default_value_ = attributeString(xml_ele, "default", default_value_);
			}

		private:
			std::string default_value_{""};

			friend class Command;
		};
		class GroupParam :public ParamBase
		{
		public:
			static auto Type()->const std::string &{ static const std::string type("group"); return std::ref(type); }
			virtual auto type() const->const std::string&{ return Type(); }
			virtual auto take()->void override final {
				if(!isTaken())ParamBase::take();
			}
			virtual auto reset()->void override final { for (auto &child : *this)dynamic_cast<ParamBase&>(child).reset(); ParamBase::reset(); };
			virtual auto addDefaultParam(std::map<std::string, std::string> &param_map_out)->void final override {
				for (auto &sub_param : *this) {	sub_param.addDefaultParam(param_map_out); }
			}
			GroupParam(Object &father, std::size_t id, const std::string &name) :ParamBase(father, id, name) {}
			GroupParam(Object &father, std::size_t id, const aris::core::XmlElement &xml_ele) :ParamBase(father, id, xml_ele) {}

		private:

		};
		
		auto Command::reset()->void 
		{ 
			is_taken_ = false;
			for (auto &child : *this)dynamic_cast<ParamBase&>(child).reset();
		}
		auto Command::addDefaultParam(std::map<std::string, std::string> &param_map_out)
		{
			auto default_param_iter = std::find_if(begin(), end(), [](ParamBase &param)->bool{ return param.isTaken(); });
			auto default_param_ptr = default_value_ == "" ? nullptr : &*findByName(default_value_);
			auto default_param = default_param_iter == end() ? default_param_ptr : &*default_param_iter;
			default_param = size() == 1 ? &front() : default_param;

			if (!default_param)throw std::runtime_error("failed to find default param in command \"" + name() +"\"");

			default_param->addDefaultParam(param_map_out);

		}
		Command::Command(Object &father, std::size_t id, const aris::core::XmlElement &xml_ele) :ObjectPool(father, id, xml_ele)
		{
			default_value_ = attributeString(xml_ele, "default", default_value_);
			
			if(default_value_ != "" && (findByName(default_value_) == end()))
				throw std::runtime_error("Command \"" + name() + "\" does not has valid default param name");

			std::function<void(ParamBase &)> add_param_map_and_check_default = [this, &add_param_map_and_check_default](ParamBase &param)
			{
				auto unique = dynamic_cast<UniqueParam*>(&param);
				if (unique && (unique->default_value_!="") && (unique->findByName(unique->default_value_)== unique->end()))
				{
					throw std::runtime_error("Unique param \"" + unique->name() + "\" does not has valid default param name");
				}
				
				if (dynamic_cast<Param*>(&param))
				{
					if (this->param_map_.find(param.name()) != this->param_map_.end())
						throw std::runtime_error("failed to add param \"" + param.name() +"\" to cmd \"" + this->name() + "\", because this param already exists");
					if (this->abbreviation_map_.find(dynamic_cast<Param&>(param).abbreviation())!= this->abbreviation_map_.end())
						throw std::runtime_error("failed to add param \"" + param.name() + "\" to cmd \"" + this->name() + "\", because its abbreviation already exists");

					this->param_map_.insert(std::make_pair(param.name(), dynamic_cast<Param*>(&param)));
					this->abbreviation_map_.insert(std::make_pair(dynamic_cast<Param&>(param).abbreviation(), param.name()));
				}
				else
					for (auto &sub_param : param) add_param_map_and_check_default(sub_param);
			};
			for (auto &param : *this) add_param_map_and_check_default(param);


		}

		struct CommandParser::Imp
		{
			ObjectPool<Command>* command_pool_;
		};
		auto CommandParser::loadXml(const XmlDocument &xml_doc)->void
		{
			auto xml_ele = xml_doc.RootElement()->FirstChildElement("Server");

			if (!xml_ele)throw std::runtime_error("can't find SensorRoot element in xml file");

			loadXml(*xml_ele);
		}
		auto CommandParser::loadXml(const XmlElement &xml_ele)->void
		{
			Root::loadXml(xml_ele);
			imp_->command_pool_ = findByName("Commands") == end() ? &add<aris::core::ObjectPool<Command> >("Commands") : static_cast<aris::core::ObjectPool<Command> *>(&(*findByName("Commands")));
		}
		auto CommandParser::parse(const std::string &command_string, std::string &cmd_out, std::map<std::string, std::string> &param_map_out)->void
		{
			/// 将msg转换成cmd和一系列参数，不过这里的参数为原生字符串，既包括名称也包含值，例如“-heigt=0.5” ///
			std::stringstream input_stream{ command_string };
			std::string word;
			std::vector<std::string> param_vec;
			std::size_t param_num{0};

			if (!(input_stream >> cmd_out))
			{
				throw std::runtime_error("invalid command string, please at least contain a word");
			};

			auto command = imp_->command_pool_->findByName(cmd_out);
			if (command == imp_->command_pool_->end())
				throw std::runtime_error("invalid command name, server does not have command \"" + cmd_out + "\"");
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
						if (command->abbreviation_map_.find(c) != command->abbreviation_map_.end())
						{
							param_map_out.insert(make_pair(command->abbreviation_map_.at(c), param_value));
							command->param_map_.at(command->abbreviation_map_.at(c))->take();
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

					if (command->abbreviation_map_.find(c) != command->abbreviation_map_.end())
					{
						param_map_out.insert(make_pair(command->abbreviation_map_.at(c), param_value));
						command->param_map_.at(command->abbreviation_map_.at(c))->take();
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

					if (command->param_map_.find(param_name) != command->param_map_.end())
					{
						param_map_out.insert(make_pair(param_name, param_value));
						command->param_map_.at(param_name)->take();
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
		CommandParser::~CommandParser(){}
		CommandParser::CommandParser():imp_(new Imp)
		{
			registerChildType<Command >();
			registerChildType<Param >();
			registerChildType<UniqueParam >();
			registerChildType<GroupParam >();
			registerChildType<ObjectPool<Command> >();
		}
	}
}

