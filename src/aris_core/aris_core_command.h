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
			virtual auto type() const->const std::string&{ return Type(); }
			auto command()const->const Command &;
			auto simpleHelp()const->const std::string &;
			virtual auto help(bool isfull, int begin)const->const std::string{ return std::string{}; };
			ParamBase(Object &father, std::size_t id, const std::string &name);
			ParamBase(Object &father, std::size_t id, const aris::core::XmlElement &xml_ele);

		protected:
			auto isTaken()->bool;

			virtual auto take()->void;
			virtual auto reset()->void;
			virtual auto addDefaultParam(std::map<std::string, std::string> &param_map_out)->void = 0;
		
			struct Imp;
			ImpPtr<Imp> imp_;
			friend class Command;
			friend class UniqueParam;
			friend class GroupParam;
		};
		class Param final:public ParamBase
		{
		public:
			static auto Type()->const std::string &{ static const std::string type("Param"); return std::ref(type); }
			virtual auto type() const->const std::string&{ return Type(); }
			auto abbreviation()->char;
			auto abbreviation()const->char;
			virtual auto help(bool isfull, int begin)const->const std::string override;
			auto defaultParam()const->const std::string &;
			
			virtual ~Param();
			Param(Object &father, std::size_t id, const std::string &name);
			Param(Object &father, std::size_t id, const aris::core::XmlElement &xml_ele);
			Param(const Param&);
			Param(Param&&);
			Param& operator=(const Param&);
			Param& operator=(Param&&);

		private:
			virtual auto take()->void override final;
			virtual auto reset()->void override final;
			virtual auto addDefaultParam(std::map<std::string, std::string> &param_map_out)->void final override;

			struct Imp;
			ImpPtr<Imp> imp_;

			friend class Command;
			friend class CommandParser;
		};
		class UniqueParam final:public ParamBase
		{
		public:
			static auto Type()->const std::string &{ static const std::string type("UniqueParam"); return std::ref(type); }
			virtual auto type() const->const std::string&{ return Type(); }
			virtual auto help(bool isfull, int begin)const->const std::string override;
			auto defaultParam()const->const std::string &;
			
			virtual ~UniqueParam();
			UniqueParam(Object &father, std::size_t id, const std::string &name);
			UniqueParam(Object &father, std::size_t id, const aris::core::XmlElement &xml_ele);
			UniqueParam(const UniqueParam &);
			UniqueParam(UniqueParam &&);
			UniqueParam& operator=(const UniqueParam &);
			UniqueParam& operator=(UniqueParam &&);

		private:
			virtual auto take()->void override final;
			virtual auto reset()->void override final;
			virtual auto addDefaultParam(std::map<std::string, std::string> &param_map_out)->void final override;
			
			struct Imp;
			ImpPtr<Imp> imp_;

			friend class Command;
		};
		class GroupParam final: public ParamBase
		{
		public:
			static auto Type()->const std::string &{ static const std::string type("GroupParam"); return std::ref(type); }
			virtual auto type() const->const std::string&{ return Type(); }
			virtual auto help(bool isfull, int begin)const->const std::string override;
			
			virtual ~GroupParam();
			GroupParam(Object &father, std::size_t id, const std::string &name);
			GroupParam(Object &father, std::size_t id, const aris::core::XmlElement &xml_ele);
			GroupParam(const GroupParam &);
			GroupParam(GroupParam &&);
			GroupParam& operator=(const GroupParam &);
			GroupParam& operator=(GroupParam &&);

		protected:
			virtual auto take()->void override final;
			virtual auto reset()->void override final;
			virtual auto addDefaultParam(std::map<std::string, std::string> &param_map_out)->void override final;
		};
		class Command :public ObjectPool<ParamBase>
		{
		public:
			static auto Type()->const std::string &{ static const std::string type("Command"); return std::ref(type); }
			virtual auto type() const->const std::string&{ return Type(); }
			auto defaultParam()const->const std::string &;
			auto help(bool isfull, int begin)const->std::string;
			virtual ~Command();
			Command(Object &father, std::size_t id, const std::string &name);
			Command(Object &father, std::size_t id, const aris::core::XmlElement &xml_ele);
			Command(const Command &);
			Command(Command &&);
			Command& operator=(const Command &);
			Command& operator=(Command &&);

		private:
			auto reset()->void;
			auto take()->void;
			auto addDefaultParam(std::map<std::string, std::string> &param_map_out)->void;

			struct Imp;
			ImpPtr<Imp> imp_;

			friend class CommandParser;
			friend class ParamBase;
		};
		class CommandParser:public Object
		{
		public:
			static auto Type()->const std::string &{ static const std::string type("CommandParser"); return std::ref(type); }
			virtual auto type() const->const std::string&{ return Type(); }
			auto parse(const std::string &command_string, std::string &cmd_out, std::map<std::string, std::string> &param_map_out)->void;
            auto help()const->std::string;
            auto commandPool()->ObjectPool<Command> &;
			auto commandPool()const->const ObjectPool<Command> &;

			virtual ~CommandParser();
			CommandParser(Object &father, std::size_t id, const std::string &name);
			CommandParser(Object &father, std::size_t id, const aris::core::XmlElement &xml_ele);
			CommandParser(const CommandParser &);
			CommandParser(CommandParser &&);
			CommandParser& operator=(const CommandParser &);
			CommandParser& operator=(CommandParser &&);
		
		private:
			struct Imp;
			ImpPtr<Imp> imp_;
		};

		auto formatString(std::string originalString, int begin)->std::string;

	}
}






#endif
