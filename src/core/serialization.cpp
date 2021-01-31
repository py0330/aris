#include <string>
#include <iostream>
#include <functional>
#include <vector>
#include <map>
#include <string>
#include <algorithm>
#include <limits>
#include <regex>
#include <fstream>

#include "aris/core/tinyxml2.h"
#include "aris/core/object.hpp"
#include "aris/core/serialization.hpp"
#include "aris/core/expression_calculator.hpp"

#include "../src/server/json.hpp"
#include "../src/server/fifo_map.hpp"

namespace aris::core
{
	auto typename_xml2c(const tinyxml2::XMLElement *ele)->std::string
	{
		return std::regex_replace(ele->Name(), std::regex("\\."), "::");
	}
	auto typename_c2xml(const aris::core::Type *c_type)->std::string
	{
		return std::regex_replace(c_type->name().data(), std::regex("\\::"), ".");
	}
	
	auto to_xml_ele(aris::core::Instance ins, tinyxml2::XMLElement *ele)->void
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
				auto insert_ele = ele->GetDocument()->NewElement(typename_c2xml(ins.at(i).type()).data());
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
					auto insert_ele = ele->GetDocument()->NewElement(typename_c2xml(v.type()).data());
					ele->InsertEndChild(insert_ele);
					//insert_ele->SetAttribute(":name", prop->name().data());
					to_xml_ele(v, insert_ele);
				}
			}
		}
	}
	auto toXmlString(aris::core::Instance ins)->std::string
	{
		tinyxml2::XMLDocument doc;

		auto root_xml_ele = doc.NewElement(typename_c2xml(ins.type()).data());
		doc.InsertEndChild(root_xml_ele);

		to_xml_ele(ins, root_xml_ele);

		tinyxml2::XMLPrinter printer;
		doc.Print(&printer);

		return std::string(printer.CStr());
	}

	auto from_xml_ele(aris::core::Instance &ins, tinyxml2::XMLElement *ele)->void
	{
		// from text //
		if (ele->GetText())	ins.fromString(ele->GetText());

		// 获取全部ele //
		std::vector<tinyxml2::XMLElement *> child_eles;
		std::vector<const tinyxml2::XMLAttribute *> attrs;
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
					return std::regex_replace(attr->Name(), std::regex("\\."), "::") == prop->name();
				});

				if (found == attrs.end()) 
				{
					//std::cout << "WARNING:basic prop not found : " << prop->name() << std::endl;
					continue;
				}

				auto [ptr, prop_ins] = prop->type()->create();
				prop_ins.fromString((*found)->Value());
				prop->set(&ins, prop_ins);
				attrs.erase(found);
				continue;
			}

			// non basic type, non ptr prop //
			if (!prop->acceptPtr())
			{
				auto found = std::find_if(child_eles.begin(), child_eles.end(), [&prop](const auto ele)->bool 
				{
					return typename_xml2c(ele) == prop->type()->name();
				});

				if (found == child_eles.end())
				{
					//std::cout << "WARNING:element prop not found : " << prop->name() << std::endl;
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
					return Type::isBaseOf(prop->type(), Type::getType(typename_xml2c(ele)));
				});

				if (found == child_eles.end())
				{
					//std::cout << "WARNING:element prop not found : " << prop->name() << std::endl;
					continue;
				}

				auto c_type = typename_xml2c(*found);
				auto[ptr, prop_ins] = Type::getType(c_type)->create();
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
				auto type = Type::getType(typename_xml2c(child_ele));
				if (!type) THROW_FILE_LINE("unrecognized type in xml : " + std::string(child_ele->Name()));

				auto[ptr, attr_ins] = type->create();
				from_xml_ele(attr_ins, child_ele);
				ins.push_back(attr_ins);
				if (ins.type()->isRefArray())ptr.release();
			}
		}
	}
	auto fromXmlString(aris::core::Instance ins, std::string_view xml_str)->void
	{
		tinyxml2::XMLDocument doc;
		auto ret = doc.Parse(xml_str.data(), xml_str.size());

		if (ret != tinyxml2::XML_SUCCESS)
			THROW_FILE_LINE("load xml failed : " + std::to_string(ret));

		auto root_ele = doc.RootElement();

		if(typename_xml2c(root_ele) != ins.type()->name())
			THROW_FILE_LINE("load xml failed : type not match:" + typename_xml2c(root_ele) + "  " + std::string(ins.type()->name()));

		from_xml_ele(ins, root_ele);
	}

	auto toXmlFile(aris::core::Instance ins, const std::filesystem::path &file)->void
	{
		std::ofstream fs(file, std::ios::trunc);

		fs << toXmlString(ins);

		fs.close();
	}
	auto fromXmlFile(aris::core::Instance ins, const std::filesystem::path &file)->void
	{
		std::ifstream fs(file);

		std::string str((std::istreambuf_iterator<char>(fs)), std::istreambuf_iterator<char>());

		fromXmlString(ins, str);

		fs.close();
	}


	template<class K, class V, class dummy_compare, class A>
	using my_workaround_fifo_map = nlohmann::fifo_map<K, V, nlohmann::fifo_map_compare<K>, A>;
	using my_json = nlohmann::basic_json<my_workaround_fifo_map>;
	
	auto typename_json2c(const std::string &key)->std::string{	return std::regex_replace(key, std::regex("\\."), "::");}
	auto typename_c2json(const aris::core::Type *c_type)->std::string{	return std::regex_replace(c_type->name().data(), std::regex("\\::"), ".");}

	auto to_json(aris::core::Instance ins)->my_json
	{
		my_json js;

		// set text //
		if (!ins.toString().empty()) js["#text"] = ins.toString();

		if (ins.isArray())
		{
			for (auto prop : ins.type()->properties())
			{
				auto v = prop->get(&ins);
				if (!v.isBasic())THROW_FILE_LINE("failed to serilize");
				if (v.toString() != "")js["@" + prop->name()] = v.toString();
			}
			for (auto i = 0; i < ins.size(); ++i)
			{
				my_json insert;
				insert[typename_c2json(ins.at(i).type())] = to_json(ins.at(i));
				js["#array"].push_back(insert);
			}
		}
		else
		{
			my_json::iterator basic_insert_pos = js.end();
			
			for (auto &prop : ins.type()->properties())
			{
				auto v = prop->get(&ins);

				if (v.isBasic())
				{
					if (!v.toString().empty())	js["@" + prop->name()] = v.toString();
					//js.insert(
				}
				else
				{
					auto ist = to_json(v);
					ist["#name"] = prop->name().data();
					js[typename_c2json(v.type()).data()] = ist;
				}
			}
		}

		return js;
	}
	auto toJsonString(aris::core::Instance ins)->std::string
	{
		my_json js;
		js[ins.type()->name().data()] = to_json(ins);
		return js.dump(2);
	}

	auto from_json(aris::core::Instance &ins, my_json &js)->void
	{
		// from text //
		if (js.find("#text") != js.end()) ins.fromString(js["#text"].get<std::string>());

		// 读写 //
		for (auto &prop : ins.type()->properties())	{
			// basic type //
			if (prop->type()->isBasic() && js.find("@" + prop->name()) != js.end())	{
				auto[ptr, prop_ins] = prop->type()->create();
				prop_ins.fromString(js["@" + prop->name()].get<std::string>());
				prop->set(&ins, prop_ins);
				js.erase(prop->name());
				//continue;
			}

			// non basic type, non ptr prop //
			else if (!prop->acceptPtr() && js.find("@" + prop->name()) != js.end())	{
				auto[ptr, prop_ins] = prop->type()->create();
				from_json(prop_ins, js["@" + prop->name()]);
				prop->set(&ins, prop_ins);
				js.erase(prop->name());
				//continue;
			}

			// non basic type, accept ptr //
			else if (prop->acceptPtr())	{
				my_json::iterator found = js.end();
				for (auto it = js.begin(); it != js.end(); ++it)
				{
					if (it.value().is_object() && Type::isBaseOf(prop->type(), Type::getType(typename_json2c(it.key()))))
					{
						found = it;
						break;
					};
				}
				
				if (found == js.end())
				{
					//std::cout << "WARNING:element prop not found : " << prop->name() << std::endl;
					continue;
				}

				auto[ptr, prop_ins] = Type::getType(found.key())->create();
				from_json(prop_ins, *found);
				prop->set(&ins, prop_ins);
				ptr.release();
				js.erase(found);
				//continue;
			}
		}

		if (ins.isArray() && js.find("#array") != js.end())	{
			auto &js_array = js["#array"];
			for (auto it = js_array.begin(); it != js_array.end(); ++it) {

				auto ele = it->begin();
				
				if (ele.key()[0] == '#' || ele.key()[0] == '@') continue;
				
				auto type = Type::getType(typename_json2c(ele.key()));
				if (!type) THROW_FILE_LINE("unrecognized type in json : " + ele.key());

				auto[ptr, attr_ins] = type->create();
				from_json(attr_ins, *ele);
				ins.push_back(attr_ins);
				if (ins.type()->isRefArray())ptr.release();
			}
		}
	}
	auto fromJsonString(aris::core::Instance ins, std::string_view xml_str)->void
	{
		my_json js = nlohmann::json::parse(xml_str);

		if (typename_json2c(js.begin().key()) != ins.type()->name())
			THROW_FILE_LINE("load xml failed : type not match");

		from_json(ins, js.front());
	}

	auto toJsonFile(aris::core::Instance ins, const std::filesystem::path &file)->void
	{
		std::ofstream fs(file, std::ios::trunc);

		fs << toJsonString(ins);

		fs.close();
	}
	auto fromJsonFile(aris::core::Instance ins, const std::filesystem::path &file)->void
	{
		std::ifstream fs(file);

		std::string str((std::istreambuf_iterator<char>(fs)), std::istreambuf_iterator<char>());

		fromJsonString(ins, str);

		fs.close();
	}

}