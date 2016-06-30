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
			auto command()const->const Command &;
			
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
			static auto Type()->const std::string &{ static const std::string type("param"); return std::ref(type); }
			virtual auto type() const->const std::string&{ return Type(); }
			auto abbreviation()->char;
			auto defaultParam()const->const std::string &;
			Param(Object &father, std::size_t id, const std::string &name);
			Param(Object &father, std::size_t id, const aris::core::XmlElement &xml_ele);

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
			static auto Type()->const std::string &{ static const std::string type("unique"); return std::ref(type); }
			virtual auto type() const->const std::string&{ return Type(); }
			auto defaultParam()const->const std::string &;
			UniqueParam(Object &father, std::size_t id, const std::string &name);
			UniqueParam(Object &father, std::size_t id, const aris::core::XmlElement &xml_ele);
			
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
			static auto Type()->const std::string &{ static const std::string type("group"); return std::ref(type); }
			virtual auto type() const->const std::string&{ return Type(); }
			GroupParam(Object &father, std::size_t id, const std::string &name);
			GroupParam(Object &father, std::size_t id, const aris::core::XmlElement &xml_ele);

		protected:
			virtual auto take()->void override final;
			virtual auto reset()->void override final;
			virtual auto addDefaultParam(std::map<std::string, std::string> &param_map_out)->void override final;
		};
		class Command :public ObjectPool<ParamBase>
		{
		public:
			static auto Type()->const std::string &{ static const std::string type("command"); return std::ref(type); }
			virtual auto type() const->const std::string&{ return Type(); }
			auto defaultParam()const->const std::string &;
			~Command();
			Command(Object &father, std::size_t id, const std::string &name);
			Command(Object &father, std::size_t id, const aris::core::XmlElement &xml_ele);

		private:
			auto reset()->void;
			auto take()->void;
			auto addDefaultParam(std::map<std::string, std::string> &param_map_out)->void;

			struct Imp;
			ImpPtr<Imp> imp_;

			friend class CommandParser;
			friend class ParamBase;
		};

		class CommandParser:public Root
		{
		public:
			using Root::loadXml;
			virtual auto loadXml(const XmlDocument &xml_doc)->void override;
			virtual auto loadXml(const XmlElement &xml_ele)->void override;
			auto parse(const std::string &command_string, std::string &cmd_out, std::map<std::string, std::string> &param_map_out)->void;
			auto commandPool()->ObjectPool<Command> &;
			auto commandPool()const->const ObjectPool<Command> &;

			~CommandParser();
			CommandParser();
		private:
			struct Imp;
			ImpPtr<Imp> imp_;
		};
	}
}






#endif
