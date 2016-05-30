#ifndef ARIS_CORE_COMMAND_H_
#define ARIS_CORE_COMMAND_H_

#include <map>

#include <aris_core_xml.h>


namespace aris
{
	namespace core
	{
		class CommandParser:public Root
		{
		public:
			using Root::loadXml;
			virtual auto loadXml(const XmlDocument &xml_doc)->void override;
			virtual auto loadXml(const XmlElement &xml_ele)->void override;
			auto parse(const std::string &command_string, std::string &cmd_out, std::map<std::string, std::string> &param_map_out)->void;

			~CommandParser();
			CommandParser();
		private:
			struct Imp;
			ImpPtr<Imp> imp_;
		};
	}
}






#endif
