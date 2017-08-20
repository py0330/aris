#ifndef ARIS_CORE_COMMAND_H_
#define ARIS_CORE_COMMAND_H_

#include <map>

#include <aris_core_xml.h>


namespace aris
{
	namespace core
	{
		class Command;
		
		class ParamBase :public ObjectPool<ParamBase>
		{
		public:
			static auto Type()->const std::string &{ static const std::string type("parambase"); return std::ref(type); }
			auto virtual type() const->const std::string& override{ return Type(); }
			auto virtual saveXml(aris::core::XmlElement &xml_ele) const->void override;
			auto virtual help(bool isfull, int begin)const->std::string{ return std::string{}; };
			auto simpleHelp()const->const std::string &;
			auto command()const->const Command &;
			virtual ~ParamBase();
			explicit ParamBase(Object &father, const aris::core::XmlElement &xml_ele);
			explicit ParamBase(const std::string &name, const std::string &help);
			ParamBase(const ParamBase&);
			ParamBase(ParamBase&&);
			ParamBase& operator=(const ParamBase&);
			ParamBase& operator=(ParamBase&&);

		protected:
			struct Imp;
			ImpPtr<Imp> imp_;
			friend class Command;
		};
		class Param final:public ParamBase
		{
		public:
			static auto Type()->const std::string &{ static const std::string type("Param"); return std::ref(type); }
			auto virtual type() const->const std::string& override{ return Type(); }
			auto virtual saveXml(aris::core::XmlElement &xml_ele) const->void override;
			auto virtual help(bool isfull, int begin)const->std::string override;
			auto abbreviation()const->char;
			auto defaultParam()const->const std::string &;

			virtual ~Param();
			explicit Param(const std::string &name, const std::string &default_param, const std::string &help, char abbrev = 0);
			explicit Param(Object &father, const aris::core::XmlElement &xml_ele);
			Param(const Param&);
			Param(Param&&);
			Param& operator=(const Param&);
			Param& operator=(Param&&);

		protected:
			struct Imp;
			ImpPtr<Imp> imp_;

			friend class Command;
			friend class CommandParser;
		};
		class UniqueParam final:public ParamBase
		{
		public:
			static auto Type()->const std::string &{ static const std::string type("UniqueParam"); return std::ref(type); }
			auto virtual type() const->const std::string& override{ return Type(); }
			auto virtual saveXml(aris::core::XmlElement &xml_ele) const->void override;
			auto virtual help(bool isfull, int begin)const->std::string override;
			auto defaultParam()const->const std::string &;

			virtual ~UniqueParam();
			explicit UniqueParam(const std::string &name, const std::string &default_param, const std::string &help);
			explicit UniqueParam(Object &father, const aris::core::XmlElement &xml_ele);
			UniqueParam(const UniqueParam &);
			UniqueParam(UniqueParam &&);
			UniqueParam& operator=(const UniqueParam &);
			UniqueParam& operator=(UniqueParam &&);

		protected:
			struct Imp;
			ImpPtr<Imp> imp_;

			friend class Command;
		};
		class GroupParam final: public ParamBase
		{
		public:
			static auto Type()->const std::string &{ static const std::string type("GroupParam"); return std::ref(type); }
			auto virtual type() const->const std::string& override{ return Type(); }
			auto virtual help(bool isfull, int begin)const->std::string override;
			
			virtual ~GroupParam();
			explicit GroupParam(const std::string &name, const std::string &help);
			explicit GroupParam(Object &father, const aris::core::XmlElement &xml_ele);
			GroupParam(const GroupParam &);
			GroupParam(GroupParam &&);
			GroupParam& operator=(const GroupParam &);
			GroupParam& operator=(GroupParam &&);
		};
		class Command :public ObjectPool<ParamBase>
		{
		public:
			static auto Type()->const std::string &{ static const std::string type("Command"); return std::ref(type); }
			auto virtual type() const->const std::string& override{ return Type(); }
			auto virtual saveXml(aris::core::XmlElement &xml_ele) const->void override;
			auto defaultParam()const->const std::string &;
			auto help(bool isfull, int begin)const->std::string;
			virtual ~Command();
			explicit Command(const std::string &name, const std::string &default_param, const std::string &help);
			explicit Command(Object &father, const aris::core::XmlElement &xml_ele);
			Command(const Command &);
			Command(Command &&);
			Command& operator=(const Command &);
			Command& operator=(Command &&);

		private:
			struct Imp;
			ImpPtr<Imp> imp_;

			friend class CommandParser;
			friend class ParamBase;
		};
		class CommandParser:public Object
		{
		public:
			static auto Type()->const std::string &{ static const std::string type("CommandParser"); return std::ref(type); }
			auto virtual type() const->const std::string& override{ return Type(); }
			auto parse(const std::string &command_string, std::string &cmd_out, std::map<std::string, std::string> &param_map_out)->void;
            auto help()const->std::string;
            auto commandPool()->ObjectPool<Command> &;
			auto commandPool()const->const ObjectPool<Command> &;

			virtual ~CommandParser();
			explicit CommandParser(const std::string &name);
			explicit CommandParser(Object &father, const aris::core::XmlElement &xml_ele);
			CommandParser(const CommandParser &);
			CommandParser(CommandParser &&);
			CommandParser& operator=(const CommandParser &);
			CommandParser& operator=(CommandParser &&);
		
		private:
			struct Imp;
			ImpPtr<Imp> imp_;
		};
	}
}


#endif
