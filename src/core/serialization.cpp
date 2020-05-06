#include <string>
#include <iostream>
#include <functional>
#include <vector>
#include <map>
#include <string>
#include <algorithm>
#include <limits>
#include <regex>

#include "aris/core/tinyxml2.h"
#include "aris/core/object.hpp"
#include "aris/core/serialization.hpp"
#include "aris/core/expression_calculator.hpp"


namespace aris::core
{
	auto to_xml_ele(aris::core::Instance &ins, aris::core::XmlElement *ele)->void
	{
		// set text //
		if (!ins.toString().empty()) ele->SetText(ins.toString().data());
		
		// set children or props //
		if (ins.isArray())
		{
			for (auto prop : ins.type()->properties())
			{
				auto v = prop->get(&ins);

				if (!v.isBasic())
					THROW_FILE_LINE("failed to serilize");

				if (v.toString() != "")ele->SetAttribute(prop->name().data(), v.toString().c_str());
			}
			for (auto i = 0; i < ins.size(); ++i)
			{
				auto insert_ele = ele->GetDocument()->NewElement(ins.at(i).type()->name().data());
				ele->InsertEndChild(insert_ele);
				to_xml_ele(ins.at(i), insert_ele);
			}
		}
		else
		{
			for (auto &prop : ins.type()->properties())
			{
				auto v = prop->get(&ins);

				if (v.isBasic())
				{
					if(!v.toString().empty())
						ele->SetAttribute(prop->name().data(), v.toString().c_str());
				}
				else
				{
					auto insert_ele = ele->GetDocument()->NewElement(v.type()->name().data());
					ele->InsertEndChild(insert_ele);
					insert_ele->SetAttribute("name", prop->name().data());
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
		// from text //
		if (ele->GetText())	ins.fromString(ele->GetText());

		// 获取全部ele //
		std::vector<aris::core::XmlElement *> child_eles;
		std::vector<const aris::core::XmlAttribute *> attrs;
		for (auto child_ele = ele->FirstChildElement(); child_ele; child_ele = child_ele->NextSiblingElement())
		{
			child_eles.push_back(child_ele);
		}
		for (auto attr = ele->FirstAttribute(); attr; attr = attr->Next())
		{
			attrs.push_back(attr);
		}
		
		// 读写 //
		for (auto &prop : ins.type()->properties())
		{
			// basic type //
			if (prop->type()->isBasic())
			{
				auto found = std::find_if(attrs.begin(), attrs.end(), [&prop](const auto attr)->bool 
				{
					return attr->Name() == prop->name(); 
				});

				if (found == attrs.end()) 
				{
					std::cout << "WARNING:basic property not found : " << prop->name() << std::endl;
					continue;
				}

				auto [ptr, prop_ins] = prop->type()->create();
				prop_ins.fromString((*found)->Value());
				prop->set(&ins, prop_ins);
				attrs.erase(found);
				continue;
			}

			// non basic type, non ptr property //
			if (!prop->acceptPtr())
			{
				auto found = std::find_if(child_eles.begin(), child_eles.end(), [&prop](const auto ele)->bool 
				{
					return ele->Name() == prop->type()->name();
				});

				if (found == child_eles.end())
				{
					std::cout << "WARNING:element property not found : " << prop->name() << std::endl;
					continue;
				}

				auto[ptr, prop_ins] = prop->type()->create();
				from_xml_ele(prop_ins, *found);
				prop->set(&ins, prop_ins);
				child_eles.erase(found);
				continue;
			}

			// non basic type, accept ptr //
			if (prop->acceptPtr())
			{
				auto found = std::find_if(child_eles.begin(), child_eles.end(), [&prop](const auto ele)->bool 
				{
					return Type::isBaseOf(prop->type(), getType(ele->Name()));
				});

				if (found == child_eles.end())
				{
					std::cout << "WARNING:element property not found : " << prop->name() << std::endl;
					continue;
				}

				auto[ptr, prop_ins] = getType((*found)->Name())->create();
				from_xml_ele(prop_ins, *found);
				prop->set(&ins, prop_ins);
				ptr.release();
				child_eles.erase(found);
				continue;
			}
		}

		if (ins.isArray())
		{
			for (auto child_ele = ele->FirstChildElement(); child_ele; child_ele = child_ele->NextSiblingElement())
			{
				auto type = getType(child_ele->Name());
				if (!type) THROW_FILE_LINE("unrecognized type in xml");

				auto[ptr, attr_ins] = type->create();
				from_xml_ele(attr_ins, child_ele);
				ins.push_back(attr_ins);
				if (ins.type()->isRefArray())ptr.release();
			}
		}
	}
	auto fromXmlString(aris::core::Instance ins, std::string_view xml_str)->void
	{
		aris::core::XmlDocument doc;
		auto ret = doc.Parse(xml_str.data(), xml_str.size());

		if (ret != tinyxml2::XML_SUCCESS)
			THROW_FILE_LINE("load xml failed : " + std::to_string(ret));

		auto root_ele = doc.RootElement();

		if(root_ele->Name() != ins.type()->name())
			THROW_FILE_LINE("load xml failed : type not match" );

		from_xml_ele(ins, root_ele);
	}
}