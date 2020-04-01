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


namespace aris::core
{
	auto to_xml_ele(aris::core::Instance &ins, aris::core::XmlElement *ele)->void
	{
		if (ins.isBasic())
		{
			// 只能把结构序列化成xml //
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
				auto insert_ele = ele->GetDocument()->NewElement(ins.at(i).type()->name().data());
				ele->InsertEndChild(insert_ele);
				auto child_ins = ins.at(i);
				to_xml_ele(child_ins, insert_ele);
			}
		}
		else
		{
			for (auto &prop : ins.type()->properties())
			{
				auto v = prop.second.get(&ins);

				if (v.isBasic())
				{
					ele->SetAttribute(prop.second.name().data(), v.toString().c_str());
				}					
				else
				{
					auto insert_ele = ele->GetDocument()->NewElement(v.type()->name().data());
					ele->InsertEndChild(insert_ele);
					insert_ele->SetAttribute("name", prop.first.data());
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
		for (auto &[prop_name, prop] : ins.type()->properties())
		{
			// 此时为basic type //
			if (ele->Attribute(prop_name.data()))
			{
				auto pre_v = prop.get(&ins);
				auto [ptr, attr_ins] = pre_v.type()->create();
				attr_ins.fromString(ele->Attribute(prop_name.data()));
				prop.set(&ins, attr_ins);
				continue;
			}

			// 此时不是basic type, 可能发生重载 //
			for (auto child_ele = ele->FirstChildElement(); child_ele; child_ele = child_ele->NextSiblingElement())
			{
				if (child_ele->Attribute("name") && prop_name == child_ele->Attribute("name"))
				{
					auto type = getType(child_ele->Name());
					if (!type) THROW_FILE_LINE("unrecognized type in xml");

					auto[ptr, attr_ins] = type->create();
					from_xml_ele(attr_ins, child_ele);

					prop.set(&ins, attr_ins);

					// 对于set函数注册为指针的，不负责生命周期管理 //
					if (prop.acceptPtr()) 
						ptr.release();

					break;
				}
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