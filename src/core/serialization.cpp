#include <string>
#include <iostream>
#include <functional>
#include <vector>
#include <map>
#include <string>
#include <algorithm>
#include <limits>
#include <regex>

#include "aris/core/object.hpp"
#include "aris/core/serialization.hpp"


namespace aris::core
{
	auto to_xml_ele(aris::core::Instance &ins, aris::core::XmlElement *ele)->void
	{
		if (ins.isBasic())
		{
			THROW_FILE_LINE("failed to serilize");
		}
		else if (ins.isArray())
		{
			for (auto &prop : ins.type()->properties())
			{
				auto v = prop.second.get(&ins);
				auto t = v.type();

				if (!v.isBasic())
					THROW_FILE_LINE("failed to serilize");

				ele->SetAttribute(prop.second.name().data(), v.toString().c_str());
			}

			for (auto i = 0; i < ins.size(); ++i)
			{
				auto insert_ele = ele->GetDocument()->NewElement(ins.type()->name().data());
				ele->InsertEndChild(insert_ele);
				auto child_ins = ins.at(i);
				to_xml_ele(child_ins, ele);
			}
		}
		else
		{
			for (auto &prop : ins.type()->properties())
			{
				auto v = prop.second.get(&ins);

				if (v.isBasic())
					ele->SetAttribute(prop.second.name().data(), v.toString().c_str());
				else
				{
					auto insert_ele = ele->GetDocument()->NewElement(v.type()->name().data());
					ele->InsertEndChild(insert_ele);
					to_xml_ele(v, insert_ele);
				}

			}
		}
	}
	auto toXmlString(aris::core::Instance ins)->std::string
	{
		aris::core::XmlDocument doc;

		auto root_xml_ele = doc.NewElement(ins.type()->name().data());
		doc.InsertEndChild(root_xml_ele);

		to_xml_ele(ins, root_xml_ele);

		tinyxml2::XMLPrinter printer;
		doc.Print(&printer);

		return std::string(printer.CStr());
	}

	auto from_xml_ele(aris::core::Instance &ins, aris::core::XmlElement *ele)->void
	{
		
	}
	auto fromXmlString(aris::core::Instance &ins, std::string_view xml_str)->void
	{
		aris::core::XmlDocument doc;
		auto ret = doc.Parse(xml_str.data(), xml_str.size());





		doc.RootElement();



	}
}