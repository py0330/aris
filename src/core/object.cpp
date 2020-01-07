#include <string>
#include <iostream>
#include <functional>
#include <vector>
#include <map>
#include <string>
#include <algorithm>
#include <limits>
#include <regex>

#include "aris/core/log.hpp"
#include "aris/core/object.hpp"

using namespace std;

namespace aris::core
{
	struct Object::Imp
	{
		static auto default_type_map()->std::map<std::string, TypeInfo> &
		{
			static std::map<std::string, TypeInfo> default_type_map_;
			return default_type_map_;
		}
		auto getTypeInfo(const std::string &type)const->const TypeInfo* 
		{
			// 从本object的map中寻找
			if (auto found = type_map_.find(type); found != type_map_.end())return &found->second;

			// 从祖先们的map中寻找
			if (father_) return father_->imp_->getTypeInfo(type);

			// 从全局map中寻找
			if (auto found = default_type_map().find(type); found != default_type_map().end())return &found->second;
			
			// 没有找到
			return nullptr;
		}

		// 不变属性 //
		Object *father_;
		std::size_t id_;

		// 可变属性 //
		std::string name_;
		std::map<std::string, TypeInfo> type_map_;
		ImpContainer<Object> children_;

		Imp() = default;
		Imp(const Imp &other) = delete;
		Imp(Imp &&other) = delete;
		Imp &operator=(const Imp &) = delete;
		Imp &operator=(Imp &&) = delete;
	};
	auto Object::attributeBool(const aris::core::XmlElement &xml_ele, const std::string &attribute_name)->bool
	{
		std::string error = "invalid bool attribute \"" + attribute_name + "\" in element \"" + xml_ele.Name() + "\":";

		if (!xml_ele.Attribute(attribute_name.c_str()))
			THROW_FILE_LINE(error + "no such attribute");

		if (xml_ele.Attribute(attribute_name.c_str(), "true"))
		{
			return true;
		}
		else if (xml_ele.Attribute(attribute_name.c_str(), "false"))
		{
			return false;
		}
		else
		{
			THROW_FILE_LINE(error + "invalid content:\"" + xml_ele.Attribute(attribute_name.c_str()) + "\"");
		}
	}
	auto Object::attributeBool(const aris::core::XmlElement &xml_ele, const std::string &attribute_name, bool default_value)->bool
	{
		return xml_ele.Attribute(attribute_name.c_str()) ? attributeBool(xml_ele, attribute_name) : default_value;
	}
	auto Object::attributeInt64(const aris::core::XmlElement &xml_ele, const std::string &attribute_name)->std::int64_t
	{
		std::string error = "failed to get int64 attribute \"" + attribute_name + "\" in element \"" + xml_ele.Name() + "\", because ";

		if (!xml_ele.Attribute(attribute_name.c_str()))
			THROW_FILE_LINE(error + "this attribute is not found in xml file");

		try
		{
			return std::stoll(xml_ele.Attribute(attribute_name.c_str()), nullptr, 0);
		}
		catch (std::exception &e)
		{
			THROW_FILE_LINE(error + "attribute text \"" + xml_ele.Attribute(attribute_name.c_str()) + "\" is invalid:" + e.what());
		}
	}
	auto Object::attributeInt64(const aris::core::XmlElement &xml_ele, const std::string &attribute_name, std::int64_t default_value)->std::int64_t
	{
		return xml_ele.Attribute(attribute_name.c_str()) ? attributeInt64(xml_ele, attribute_name) : default_value;
	}
	auto Object::attributeInt32(const aris::core::XmlElement &xml_ele, const std::string &attribute_name)->std::int32_t
	{
		std::string error = "failed to get int32 attribute \"" + attribute_name + "\" in element \"" + xml_ele.Name() + "\", because ";

		if (!xml_ele.Attribute(attribute_name.c_str()))	THROW_FILE_LINE(error + "this attribute is not found in xml file");

		long long value_ll;
		try
		{
			value_ll = std::stoll(xml_ele.Attribute(attribute_name.c_str()), nullptr, 0);
		}
		catch (std::exception &e)
		{
			THROW_FILE_LINE(error + "attribute text \"" + xml_ele.Attribute(attribute_name.c_str()) + "\" is invalid:" + e.what());
		}

		if (value_ll > std::numeric_limits<std::int32_t>::max() || value_ll < std::numeric_limits<std::int32_t>::min())
			THROW_FILE_LINE(error + "attribute text \"" + xml_ele.Attribute(attribute_name.c_str()) + "\" is invalid: the value is out of range");

		return static_cast<std::int32_t>(value_ll);
	}
	auto Object::attributeInt32(const aris::core::XmlElement &xml_ele, const std::string &attribute_name, std::int32_t default_value)->std::int32_t
	{
		return xml_ele.Attribute(attribute_name.c_str()) ? attributeInt32(xml_ele, attribute_name) : default_value;
	}
	auto Object::attributeInt16(const aris::core::XmlElement &xml_ele, const std::string &attribute_name)->std::int16_t
	{
		std::string error = "failed to get int16 attribute \"" + attribute_name + "\" in element \"" + xml_ele.Name() + "\", because ";

		if (!xml_ele.Attribute(attribute_name.c_str()))	THROW_FILE_LINE(error + "this attribute is not found in xml file");

		long long value_ll;
		try
		{
			value_ll = std::stoll(xml_ele.Attribute(attribute_name.c_str()), nullptr, 0);
		}
		catch (std::exception &e)
		{
			THROW_FILE_LINE(error + "attribute text \"" + xml_ele.Attribute(attribute_name.c_str()) + "\" is invalid:" + e.what());
		}

		if (value_ll > std::numeric_limits<std::int16_t>::max() || value_ll < std::numeric_limits<std::int16_t>::min())
			THROW_FILE_LINE(error + "attribute text \"" + xml_ele.Attribute(attribute_name.c_str()) + "\" is invalid: the value is out of range");

		return static_cast<std::int16_t>(value_ll);
	}
	auto Object::attributeInt16(const aris::core::XmlElement &xml_ele, const std::string &attribute_name, std::int16_t default_value)->std::int16_t
	{
		return xml_ele.Attribute(attribute_name.c_str()) ? attributeInt16(xml_ele, attribute_name) : default_value;
	}
	auto Object::attributeInt8(const aris::core::XmlElement &xml_ele, const std::string &attribute_name)->std::int8_t
	{
		std::string error = "failed to get int16 attribute \"" + attribute_name + "\" in element \"" + xml_ele.Name() + "\", because ";

		if (!xml_ele.Attribute(attribute_name.c_str()))	THROW_FILE_LINE(error + "this attribute is not found in xml file");

		long long value_ll;
		try
		{
			value_ll = std::stoll(xml_ele.Attribute(attribute_name.c_str()), nullptr, 0);
		}
		catch (std::exception &e)
		{
			THROW_FILE_LINE(error + "attribute text \"" + xml_ele.Attribute(attribute_name.c_str()) + "\" is invalid:" + e.what());
		}

		if (value_ll > std::numeric_limits<std::int8_t>::max() || value_ll < std::numeric_limits<std::int8_t>::min())
			THROW_FILE_LINE(error + "attribute text \"" + xml_ele.Attribute(attribute_name.c_str()) + "\" is invalid: the value is out of range");

		return static_cast<std::int8_t>(value_ll);
	}
	auto Object::attributeInt8(const aris::core::XmlElement &xml_ele, const std::string &attribute_name, std::int8_t default_value)->std::int8_t
	{
		return xml_ele.Attribute(attribute_name.c_str()) ? attributeInt8(xml_ele, attribute_name) : default_value;
	}
	auto Object::attributeUint64(const aris::core::XmlElement &xml_ele, const std::string &attribute_name)->std::uint64_t
	{
		std::string error = "failed to get uint64 attribute \"" + attribute_name + "\" in element \"" + xml_ele.Name() + "\", because ";

		if (!xml_ele.Attribute(attribute_name.c_str()))
			THROW_FILE_LINE(error + "this attribute is not found in xml file");

		try
		{
			return std::stoull(xml_ele.Attribute(attribute_name.c_str()), nullptr, 0);
		}
		catch (std::exception &e)
		{
			THROW_FILE_LINE(error + "attribute text \"" + xml_ele.Attribute(attribute_name.c_str()) + "\" is invalid:" + e.what());
		}
	}
	auto Object::attributeUint64(const aris::core::XmlElement &xml_ele, const std::string &attribute_name, std::uint64_t default_value)->std::uint64_t
	{
		return xml_ele.Attribute(attribute_name.c_str()) ? attributeUint64(xml_ele, attribute_name) : default_value;
	}
	auto Object::attributeUint32(const aris::core::XmlElement &xml_ele, const std::string &attribute_name)->std::uint32_t
	{
		std::string error = "failed to get uint32 attribute \"" + attribute_name + "\" in element \"" + xml_ele.Name() + "\", because ";

		if (!xml_ele.Attribute(attribute_name.c_str()))	THROW_FILE_LINE(error + "this attribute is not found in xml file");

		unsigned long long value_ull;
		try
		{
			value_ull = std::stoll(xml_ele.Attribute(attribute_name.c_str()), nullptr, 0);
		}
		catch (std::exception &e)
		{
			THROW_FILE_LINE(error + "attribute text \"" + xml_ele.Attribute(attribute_name.c_str()) + "\" is invalid:" + e.what());
		}

		if (value_ull > std::numeric_limits<std::uint32_t>::max())
			THROW_FILE_LINE(error + "attribute text \"" + xml_ele.Attribute(attribute_name.c_str()) + "\" is invalid: the value is out of range");

		return static_cast<std::uint32_t>(value_ull);
	}
	auto Object::attributeUint32(const aris::core::XmlElement &xml_ele, const std::string &attribute_name, std::uint32_t default_value)->std::uint32_t
	{
		return xml_ele.Attribute(attribute_name.c_str()) ? attributeUint32(xml_ele, attribute_name) : default_value;
	}
	auto Object::attributeUint16(const aris::core::XmlElement &xml_ele, const std::string &attribute_name)->std::uint16_t
	{
		std::string error = "failed to get uint16 attribute \"" + attribute_name + "\" in element \"" + xml_ele.Name() + "\", because ";

		if (!xml_ele.Attribute(attribute_name.c_str()))	THROW_FILE_LINE(error + "this attribute is not found in xml file");

		unsigned long long value_ull;
		try
		{
			value_ull = std::stoll(xml_ele.Attribute(attribute_name.c_str()), nullptr, 0);
		}
		catch (std::exception &e)
		{
			THROW_FILE_LINE(error + "attribute text \"" + xml_ele.Attribute(attribute_name.c_str()) + "\" is invalid:" + e.what());
		}

		if (value_ull > std::numeric_limits<std::uint16_t>::max())
			THROW_FILE_LINE(error + "attribute text \"" + xml_ele.Attribute(attribute_name.c_str()) + "\" is invalid: the value is out of range");

		return static_cast<std::uint16_t>(value_ull);
	}
	auto Object::attributeUint16(const aris::core::XmlElement &xml_ele, const std::string &attribute_name, std::uint16_t default_value)->std::uint16_t
	{
		return xml_ele.Attribute(attribute_name.c_str()) ? attributeUint16(xml_ele, attribute_name) : default_value;
	}
	auto Object::attributeUint8(const aris::core::XmlElement &xml_ele, const std::string &attribute_name)->std::uint8_t
	{
		std::string error = "failed to get uint8 attribute \"" + attribute_name + "\" in element \"" + xml_ele.Name() + "\", because ";

		if (!xml_ele.Attribute(attribute_name.c_str()))	THROW_FILE_LINE(error + "this attribute is not found in xml file");

		unsigned long long value_ull;
		try
		{
			value_ull = std::stoll(xml_ele.Attribute(attribute_name.c_str()), nullptr, 0);
		}
		catch (std::exception &e)
		{
			THROW_FILE_LINE(error + "attribute text \"" + xml_ele.Attribute(attribute_name.c_str()) + "\" is invalid:" + e.what());
		}

		if (value_ull > std::numeric_limits<std::uint8_t>::max())
			THROW_FILE_LINE(error + "attribute text \"" + xml_ele.Attribute(attribute_name.c_str()) + "\" is invalid: the value is out of range");

		return static_cast<std::uint8_t>(value_ull);
	}
	auto Object::attributeUint8(const aris::core::XmlElement &xml_ele, const std::string &attribute_name, std::uint8_t default_value)->std::uint8_t
	{
		return xml_ele.Attribute(attribute_name.c_str()) ? attributeUint8(xml_ele, attribute_name) : default_value;
	}
	auto Object::attributeFloat(const aris::core::XmlElement &xml_ele, const std::string &attribute_name)->float
	{
		std::string error = "failed to get double attribute \"" + attribute_name + "\" in element \"" + xml_ele.Name() + "\", because ";

		if (!xml_ele.Attribute(attribute_name.c_str()))
			THROW_FILE_LINE(error + "this attribute is not found in xml file");

		try
		{
			return std::stof(xml_ele.Attribute(attribute_name.c_str()));
		}
		catch (std::exception &e)
		{
			THROW_FILE_LINE(error + "attribute text \"" + xml_ele.Attribute(attribute_name.c_str()) + "\" is invalid:" + e.what());
		}
	}
	auto Object::attributeFloat(const aris::core::XmlElement &xml_ele, const std::string &attribute_name, float default_value)->float
	{
		return xml_ele.Attribute(attribute_name.c_str()) ? attributeFloat(xml_ele, attribute_name) : default_value;
	}
	auto Object::attributeDouble(const aris::core::XmlElement &xml_ele, const std::string &attribute_name)->double
	{
		std::string error = "failed to get double attribute \"" + attribute_name + "\" in element \"" + xml_ele.Name() + "\", because ";

		if (!xml_ele.Attribute(attribute_name.c_str()))
			THROW_FILE_LINE(error + "this attribute is not found in xml file");

		try
		{
			return std::stod(xml_ele.Attribute(attribute_name.c_str()));
		}
		catch (std::exception &e)
		{
			THROW_FILE_LINE(error + "attribute text \"" + xml_ele.Attribute(attribute_name.c_str()) + "\" is invalid:" + e.what());
		}
	}
	auto Object::attributeDouble(const aris::core::XmlElement &xml_ele, const std::string &attribute_name, double default_value)->double
	{
		return xml_ele.Attribute(attribute_name.c_str()) ? attributeDouble(xml_ele, attribute_name) : default_value;
	}
	auto Object::attributeString(const aris::core::XmlElement &xml_ele, const std::string &attribute_name)->std::string
	{
		std::string error = "failed to get string attribute \"" + attribute_name + "\" in element \"" + xml_ele.Name() + "\", because ";

		if (!xml_ele.Attribute(attribute_name.c_str()))
			THROW_FILE_LINE(error + "this attribute is not found in xml file");

		return xml_ele.Attribute(attribute_name.c_str());
	}
	auto Object::attributeString(const aris::core::XmlElement &xml_ele, const std::string &attribute_name, const std::string &default_value)->std::string
	{
		return xml_ele.Attribute(attribute_name.c_str()) ? attributeString(xml_ele, attribute_name) : default_value;
	}
	auto Object::attributeChar(const aris::core::XmlElement &xml_ele, const std::string &attribute_name)->char
	{
		std::string error = "failed to get char attribute \"" + attribute_name + "\" in element \"" + xml_ele.Name() + "\", because ";

		if (!xml_ele.Attribute(attribute_name.c_str()))
			THROW_FILE_LINE(error + "this attribute is not found in xml file");

		if (xml_ele.Attribute(attribute_name.c_str())[1] != 0)
			THROW_FILE_LINE(error + "this attribute string length is not 1");

		return xml_ele.Attribute(attribute_name.c_str())[0];
	}
	auto Object::attributeChar(const aris::core::XmlElement &xml_ele, const std::string &attribute_name, char default_value)->char
	{
		return xml_ele.Attribute(attribute_name.c_str()) ? attributeChar(xml_ele, attribute_name) : default_value;
	}
	auto Object::TypeInfo::registerTo(const std::string &type, Object &object)->void { object.imp_->type_map_.insert(std::make_pair(type, *this)); }
	auto Object::TypeInfo::registerTo(const std::string &type)->void { Object::Imp::default_type_map().insert(std::make_pair(type, *this)); }
	auto Object::getTypeInfo(const std::string &type_name)const->const TypeInfo* { return imp_->getTypeInfo(type_name); }
	auto Object::saveXml(aris::core::XmlElement &xml_ele) const->void
	{
		xml_ele.DeleteChildren();

		// xml namespace only contain one :, so change :: to :
		auto xml_type = std::regex_replace(type(), std::regex("\\::"), ":");
		xml_ele.SetName(xml_type.c_str());
		if(!name().empty())xml_ele.SetAttribute("name", name().c_str());
		for (auto &ele : children())
		{
			auto new_ele = xml_ele.GetDocument()->NewElement(ele.name().c_str());
			xml_ele.InsertEndChild(new_ele);
			ele.saveXml(*new_ele);
		}
	}
	auto Object::loadXml(const aris::core::XmlElement &xml_ele)->void
	{
		auto c_type = std::regex_replace(xml_ele.Name(), std::regex("\\:"), "::");
		
		if (type() != c_type) THROW_FILE_LINE("failed in Object::loadXml : you can't use a \"" + type() + "\" to load a \"" + xml_ele.Name() + "\" xml element");

		// set name and default child type //
		imp_->name_ = xml_ele.Attribute("name") ? xml_ele.Attribute("name") : "";

		// insert children //
		children().clear();
		for (auto ele = xml_ele.FirstChildElement(); ele; ele = ele->NextSiblingElement())
		{
			auto type = std::regex_replace(ele->Name(), std::regex("\\:"), "::");

			auto info = imp_->getTypeInfo(type);
			if (info == nullptr)THROW_FILE_LINE("unrecognized type \"" + type + "\" in Object::loadXml");
			if (info->default_construct_func == nullptr) THROW_FILE_LINE("no default ctor in Object::loadXml");

			children().push_back_ptr(info->default_construct_func());
			children().back().imp_->father_ = this;
			children().back().imp_->id_ = children().size() - 1;
			children().back().loadXml(*ele);
		}
	}
	auto Object::saveXmlFile(const std::string &filename) const->void
	{
		aris::core::XmlDocument doc;

		saveXmlDoc(doc);

		doc.SaveFile(filename.c_str());
	}
	auto Object::loadXmlFile(const std::string &filename)->void
	{
		aris::core::XmlDocument xmlDoc;

		if (xmlDoc.LoadFile(filename.c_str()) != 0)THROW_FILE_LINE((std::string("could not open file:") + std::string(filename)));

		loadXmlDoc(xmlDoc);
	}
	auto Object::loadXmlDoc(const aris::core::XmlDocument &xml_doc)->void 
	{ 
		if (!xml_doc.RootElement())THROW_FILE_LINE("empty xml doc");
		loadXml(*xml_doc.RootElement());
	}
	auto Object::saveXmlDoc(aris::core::XmlDocument &xml_doc)const->void
	{
		xml_doc.DeleteChildren();

		auto root_xml_ele = xml_doc.NewElement(name().c_str());
		xml_doc.InsertEndChild(root_xml_ele);

		saveXml(*root_xml_ele);
	}
	auto Object::xmlString()const->std::string
	{
		XmlDocument doc;
		saveXmlDoc(doc);

		tinyxml2::XMLPrinter printer;
		doc.Print(&printer);

		return std::string(printer.CStr());
	}
	auto Object::id()const->std::size_t { return imp_->id_; }
	auto Object::setName(const std::string& name)->void { imp_->name_ = name; }
	auto Object::name() const->const std::string& { return imp_->name_; }
	auto Object::root()->Object& { return imp_->father_ ? imp_->father_->root() : *this; }
	auto Object::father()->Object* { return imp_->father_; }
	auto Object::children()->ImpContainer<Object>& { return imp_->children_; }
	auto Object::findByName(const std::string &name)->ImpContainer<Object>::iterator
	{
		return std::find_if(children().begin(), children().end(), [&name, this](Object & p) {return (p.name() == name); });
	}
	auto Object::add(Object *obj)->Object &
	{
		children().push_back_ptr(obj);
		children().back().imp_->id_ = children().size() - 1;
		children().back().imp_->father_ = this;

		return children().back();
	}
	Object::~Object() = default;
	Object::Object(const std::string &name) :imp_(new Imp)
	{
		imp_->father_ = nullptr;
		imp_->id_ = 0;
		imp_->name_ = name;
		imp_->type_map_.insert(std::make_pair(Object::Type(), TypeInfo::CreateTypeInfo<Object>()));
	}
	Object::Object(const Object &other) :imp_(new Imp)
	{
		imp_->father_ = nullptr;
		imp_->id_ = 0;
		imp_->name_ = other.imp_->name_;
		imp_->type_map_ = other.imp_->type_map_;

		for (auto&child : other.children())
		{
			auto info = child.imp_->getTypeInfo(child.type());
			if (info == nullptr)THROW_FILE_LINE("unrecognized type \"" + child.type() + "\" in Object(const Object &other)");
			if (info->copy_construct_func == nullptr)THROW_FILE_LINE("type \"" + child.type() + "\" does not has copy function in Object(const Object &other)");

			children().push_back_ptr(info->copy_construct_func(child));
			children().back().imp_->father_ = this;
			children().back().imp_->id_ = children().size() - 1;
		}
	}
	Object::Object(Object &&other) : imp_(std::move(other.imp_))
	{
		imp_->father_ = nullptr;
		imp_->id_ = 0;
		for (auto &child : children())child.imp_->father_ = this;
	}
	Object& Object::operator=(const Object &other)
	{
		imp_->name_ = other.imp_->name_;
		imp_->type_map_ = other.imp_->type_map_;

		if (children().size() > other.children().size())
			children().erase(children().begin() + other.children().size(), children().end());

		for (std::size_t i = 0; i < other.children().size(); ++i)
		{
			auto info1 = other.children().at(i).imp_->getTypeInfo(other.children().at(i).type());
			auto info2 = (i >= children().size() || children().at(i).type() != other.children().at(i).type()) ? imp_->getTypeInfo(other.children().at(i).type()) : children().at(i).imp_->getTypeInfo(other.children().at(i).type());
			auto info = info1 ? info1 : info2;
			if (info == nullptr)THROW_FILE_LINE("unrecognized type \"" + other.children().at(i).type() + "\" in Object::operator=(const Object &other)");

			if (i >= children().size())
			{
				if (info->copy_construct_func == nullptr)THROW_FILE_LINE("type \"" + other.children().at(i).type() + "\" does not has copy construct function in Object::operator=(const Object &other)");
				children().push_back_ptr(info->copy_construct_func(other.children().at(i)));
			}
			else if (children().at(i).type() != other.children().at(i).type())
			{
				if (info->copy_construct_func == nullptr)THROW_FILE_LINE("type \"" + other.children().at(i).type() + "\" does not has copy construct function in Object::operator=(const Object &other)");
				children().container_.at(i).reset(info->copy_construct_func(other.children().at(i)));
			}
			else
			{
				if (info->copy_assign_func == nullptr)THROW_FILE_LINE("type \"" + other.children().at(i).type() + "\" does not has copy assign function in Object::operator=(const Object &other)");
				info->copy_assign_func(other.children().at(i), children().at(i));
			}

			children().at(i).imp_->id_ = i;
			children().at(i).imp_->father_ = this;
		}

		return *this;
	}
	Object& Object::operator=(Object &&other)
	{
		if (children().size() > other.children().size())
			children().erase(children().begin() + other.children().size(), children().end());

		for (std::size_t i = 0; i < other.children().size(); ++i)
		{
			auto info = other.children().at(i).imp_->getTypeInfo(other.children().at(i).type());
			if (info == nullptr)THROW_FILE_LINE("unrecognized type \"" + other.children().at(i).type() + "\" in Object::operator=(Object &&other)");

			if (i >= children().size())
			{
				if (info->move_construct_func == nullptr)THROW_FILE_LINE("type \"" + other.children().at(i).type() + "\" does not has move construct function in Object::operator=(Object &&other)");
				children().push_back_ptr(info->move_construct_func(std::move(other.children().at(i))));
			}
			else if (children().at(i).type() != other.children().at(i).type())
			{
				if (info->move_construct_func == nullptr)THROW_FILE_LINE("type \"" + other.children().at(i).type() + "\" does not has move construct function in Object::operator=(Object &&other)");
				children().container_.at(i).reset(info->move_construct_func(std::move(other.children().at(i))));
			}
			else
			{
				if (info->copy_assign_func == nullptr)THROW_FILE_LINE("type \"" + other.children().at(i).type() + "\" does not has move assign function in Object::operator=(Object &&other)");
				info->move_assign_func(std::move(other.children().at(i)), children().at(i));
			}

			children().at(i).imp_->id_ = i;
			children().at(i).imp_->father_ = this;
		}

		imp_->name_ = std::move(other.imp_->name_);
		imp_->type_map_ = std::move(other.imp_->type_map_);
		return *this;
	}
}