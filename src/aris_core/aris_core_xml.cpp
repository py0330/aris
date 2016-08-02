#include <string>
#include <iostream>
#include <functional>
#include <vector>
#include <map>
#include <string>
#include <algorithm>
#include <limits>

#include "aris_core_xml.h"

using namespace std;

namespace aris
{
	namespace core
	{
		template<> ImpPtr<Object>::ImpPtr(const ImpPtr &other) :data_unique_ptr_(other->root().childTypeMap().at(other->type()).copy_construct_func(*other)) {};
		template<> auto ImpPtr<Object>::operator=(const ImpPtr &other)->ImpPtr<Object>&
		{ 
			other->root().childTypeMap().at(other->type()).copy_assign_func(*other, **this);
			return *this;
		};
		template<> auto ImpContainer<Object>::operator=(const ImpContainer& other)->ImpContainer<Object>&
		{
			if (size() > other.size())erase(begin() + other.size(), end());
			for (std::size_t i = 0; i < size(); ++i)
			{
				if (at(i).type() != other.at(i).type())
				{
					this->container_.at(i).reset(other.at(i).root().childTypeMap().at(other.at(i).type()).copy_construct_func(other.at(i)));
				}
				else
				{
					other.at(i).root().childTypeMap().at(other.at(i).type()).copy_assign_func(other.at(i), at(i));
				}
			}
			return *this;
		}

		struct Object::Imp 
		{
			Object *father_;
			std::size_t id_;
			std::string name_;
			std::string default_type_{ Object::Type() };
			std::map<std::string, std::shared_ptr<Object> > save_data_map_;

			Imp(Object *father, std::size_t id, const std::string &name) :father_(father), name_(name), id_(id), default_type_(Object::Type()) {}
			Imp(Object *father, std::size_t id, const aris::core::XmlElement &xml_ele) :father_(father), name_(xml_ele.name()), id_(id), default_type_(xml_ele.Attribute("default_child_type")? xml_ele.Attribute("default_child_type"): Object::Type()){}
			Imp(const Imp &other) = default;
			Imp(Imp &&other) = default;
			Imp &operator=(const Imp &) = default;
			Imp &operator=(Imp &&) = default;
		};
		struct Root::Imp
		{
			std::map<std::string, TypeInfo> type_map_{ std::make_pair(Object::Type(),TypeInfo::CreateTypeInfo<Object>()) };

			Imp() {}
			Imp(const Imp &other) = default;
			Imp(Imp &&other) = default;
			Imp &operator=(const Imp &) = default;
			Imp &operator=(Imp &&) = default;
		};

		auto Object::root()->Root& { return dynamic_cast<Root *>(this) ? dynamic_cast<Root &>(*this) : father().root(); }
		auto Object::root()const->const Root&{ return dynamic_cast<const Root *>(this) ? dynamic_cast<const Root &>(*this) : father().root(); }
		auto Object::saveXml(aris::core::XmlElement &xml_ele) const->void
		{ 
			xml_ele.DeleteChildren();
			xml_ele.SetName(name().c_str());
			xml_ele.SetAttribute("type", this->type().c_str());

			for (auto &ele : *this)
			{
				auto new_ele = xml_ele.GetDocument()->NewElement(ele.name().c_str());
				ele.saveXml(*new_ele);
				xml_ele.InsertEndChild(new_ele);
			}
		}
		auto Object::id()const->std::size_t { return imp_->id_; }
		auto Object::name() const->const std::string&{ return imp_->name_; }
		auto Object::father()->Object& { return *imp_->father_; }
		auto Object::father()const->const Object&{ return *imp_->father_; }
		auto Object::save(const std::string &name, bool auto_override_save)->void
		{
			decltype(imp_->save_data_map_) tem = std::move(imp_->save_data_map_);
			
			if ((!auto_override_save) && (tem.find(name) != tem.end()))
				throw std::runtime_error("Object \"" + this->name() + "\" already has a save named \"" + name + "\", can't save");

			auto save_content = std::shared_ptr<Object>(root().childTypeMap().find(this->type())->second.copy_construct_func(*this));
			tem.insert(std::make_pair(name, save_content));

			imp_->save_data_map_ = std::move(tem);
		}
		auto Object::load(const std::string &name, bool auto_delete_save)->void
		{
			decltype(imp_->save_data_map_) tem = std::move(imp_->save_data_map_);
			
			if (tem.find(name) == tem.end())
				throw std::runtime_error("Object \"" + this->name() + "\" does not has a save named \"" + name + "\", can't load");
			root().childTypeMap().find(this->type())->second.copy_assign_func(*tem.at(name), std::ref(*this));
			if(auto_delete_save)tem.erase(name);

			imp_->save_data_map_ = std::move(tem);
		}
		auto Object::findByName(const std::string &name)->iterator
		{
			return std::find_if(this->begin(), this->end(), [&name, this](Object & p) {return (p.name() == name); });
		}
		auto Object::findByName(const std::string &name) const->const_iterator
		{
			return const_cast<Object *>(this)->findByName(name);
		}
		auto Object::add(const Object &object)->Object &
		{
			if (root().imp_->type_map_.find(object.type()) == root().imp_->type_map_.end())
			{
				throw std::runtime_error("unrecognized type \"" + object.type()	+ "\" when add object \"" + object.name() + "\" to \"" + name() + "\"");
			}
			else
			{
				push_back_ptr(root().imp_->type_map_.find(object.type())->second.copy_construct_func(object));
				back().imp_->id_ = size() - 1;
				back().imp_->name_ = object.name();
				back().imp_->father_ = this;

				return back();
			}
		}
		auto Object::add(Object &&object)->Object &
		{
			if (root().imp_->type_map_.find(object.type()) == root().imp_->type_map_.end())
			{
				throw std::runtime_error("unrecognized type \"" + object.type()	+ "\" when add object \"" + object.name() + "\" to \"" + name() + "\"");
			}
			else
			{
				push_back_ptr(root().imp_->type_map_.find(object.type())->second.move_construct_func(std::move(object)));
				back().imp_->id_ = size() - 1;
				back().imp_->name_ = object.name();
				back().imp_->father_ = this;

				return back();
			}
		}
		auto Object::add(Object *obj)->Object &
		{
			if (root().imp_->type_map_.find(obj->type()) == root().imp_->type_map_.end())
			{
				throw std::runtime_error("unrecognized type \"" + obj->type() + "\" when add object \"" + obj->name() + "\" to \"" + name() + "\"");
			}
			else
			{
				push_back_ptr(obj);
				back().imp_->id_ = size() - 1;
				back().imp_->father_ = this;

				return back();
			}
		}
		auto Object::add(const aris::core::XmlElement &xml_ele)->Object &
		{
			std::string type = xml_ele.Attribute("type") ? xml_ele.Attribute("type") : imp_->default_type_;

			if (root().imp_->type_map_.find(type) == root().imp_->type_map_.end())
			{
				throw std::runtime_error("unrecognized type \"" + type
					+ "\" when add xml element \"" + xml_ele.name() + "\" to \"" + name() + "\"");
			}
			else
			{
				push_back_ptr(root().imp_->type_map_.find(type)->second.xml_construct_func(*this, size(), xml_ele));
				return back();
			}
		}
		auto Object::attributeBool(const aris::core::XmlElement &xml_ele, const std::string &attribute_name)const->bool
		{
			std::string error = "failed to get bool attribute \"" + attribute_name + "\" in \"" + type() + "\" \"" + name() + "\", because ";
			
			if (!xml_ele.Attribute(attribute_name.c_str()))
				throw std::runtime_error(error + "this attribute is not found in xml file");

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
				throw std::runtime_error(error + "bool attribute must be \"true\" or \"false\", but the attribute text is \"" + xml_ele.Attribute(attribute_name.c_str()) + "\"");
			}
		}
		auto Object::attributeBool(const aris::core::XmlElement &xml_ele, const std::string &attribute_name, bool default_value)const->bool
		{
			if (!xml_ele.Attribute(attribute_name.c_str()))return default_value;
			else return attributeBool(xml_ele, attribute_name);
		}
		auto Object::attributeInt64(const aris::core::XmlElement &xml_ele, const std::string &attribute_name)const->std::int64_t
		{
			std::string error = "failed to get int64 attribute \"" + attribute_name + "\" in \"" + type() + "\" \"" + name() + "\", because ";
			
			if (!xml_ele.Attribute(attribute_name.c_str()))
				throw std::runtime_error(error + "this attribute is not found in xml file");

			try
			{
				return std::stoll(xml_ele.Attribute(attribute_name.c_str()), nullptr, 0);
			}
			catch (std::exception &e)
			{
				throw std::runtime_error(error + "attribute text \"" + xml_ele.Attribute(attribute_name.c_str()) + "\" is invalid:" + e.what());
			}
		}
		auto Object::attributeInt64(const aris::core::XmlElement &xml_ele, const std::string &attribute_name, std::int64_t default_value)const->std::int64_t
		{
			if (!xml_ele.Attribute(attribute_name.c_str()))return default_value;
			else return attributeInt64(xml_ele, attribute_name);
		}
		auto Object::attributeInt32(const aris::core::XmlElement &xml_ele, const std::string &attribute_name)const->std::int32_t
		{
			std::string error = "failed to get int32 attribute \"" + attribute_name + "\" in \"" + type() + "\" \"" + name() + "\", because ";

			if (!xml_ele.Attribute(attribute_name.c_str()))	throw std::runtime_error(error + "this attribute is not found in xml file");

			long long value_ll;
			try
			{
				value_ll = std::stoll(xml_ele.Attribute(attribute_name.c_str()), nullptr, 0);
			}
			catch (std::exception &e)
			{
				throw std::runtime_error(error + "attribute text \"" + xml_ele.Attribute(attribute_name.c_str()) + "\" is invalid:" + e.what());
			}

			if (value_ll > std::numeric_limits<std::int32_t>::max() || value_ll < std::numeric_limits<std::int32_t>::min())
				throw std::runtime_error(error + "attribute text \"" + xml_ele.Attribute(attribute_name.c_str()) + "\" is invalid: the value is out of range");

			return static_cast<std::int32_t>(value_ll);
		}
		auto Object::attributeInt32(const aris::core::XmlElement &xml_ele, const std::string &attribute_name, std::int32_t default_value)const->std::int32_t
		{
			if (!xml_ele.Attribute(attribute_name.c_str()))return default_value;
			else return attributeInt32(xml_ele, attribute_name);
		}
		auto Object::attributeInt16(const aris::core::XmlElement &xml_ele, const std::string &attribute_name)const->std::int16_t
		{
			std::string error = "failed to get int16 attribute \"" + attribute_name + "\" in \"" + type() + "\" \"" + name() + "\", because ";

			if (!xml_ele.Attribute(attribute_name.c_str()))	throw std::runtime_error(error + "this attribute is not found in xml file");

			long long value_ll;
			try
			{
				value_ll = std::stoll(xml_ele.Attribute(attribute_name.c_str()), nullptr, 0);
			}
			catch (std::exception &e)
			{
				throw std::runtime_error(error + "attribute text \"" + xml_ele.Attribute(attribute_name.c_str()) + "\" is invalid:" + e.what());
			}

			if (value_ll > std::numeric_limits<std::int16_t>::max() || value_ll < std::numeric_limits<std::int16_t>::min())
				throw std::runtime_error(error + "attribute text \"" + xml_ele.Attribute(attribute_name.c_str()) + "\" is invalid: the value is out of range");

			return static_cast<std::int16_t>(value_ll);
		}
		auto Object::attributeInt16(const aris::core::XmlElement &xml_ele, const std::string &attribute_name, std::int16_t default_value)const->std::int16_t
		{
			if (!xml_ele.Attribute(attribute_name.c_str()))return default_value;
			else return attributeInt16(xml_ele, attribute_name);
		}
		auto Object::attributeInt8(const aris::core::XmlElement &xml_ele, const std::string &attribute_name)const->std::int8_t
		{
			std::string error = "failed to get int16 attribute \"" + attribute_name + "\" in \"" + type() + "\" \"" + name() + "\", because ";

			if (!xml_ele.Attribute(attribute_name.c_str()))	throw std::runtime_error(error + "this attribute is not found in xml file");

			long long value_ll;
			try
			{
				value_ll = std::stoll(xml_ele.Attribute(attribute_name.c_str()), nullptr, 0);
			}
			catch (std::exception &e)
			{
				throw std::runtime_error(error + "attribute text \"" + xml_ele.Attribute(attribute_name.c_str()) + "\" is invalid:" + e.what());
			}

			if (value_ll > std::numeric_limits<std::int8_t>::max() || value_ll < std::numeric_limits<std::int8_t>::min())
				throw std::runtime_error(error + "attribute text \"" + xml_ele.Attribute(attribute_name.c_str()) + "\" is invalid: the value is out of range");

			return static_cast<std::int8_t>(value_ll);
		}
		auto Object::attributeInt8(const aris::core::XmlElement &xml_ele, const std::string &attribute_name, std::int8_t default_value)const->std::int8_t
		{
			if (!xml_ele.Attribute(attribute_name.c_str()))return default_value;
			else return attributeInt8(xml_ele, attribute_name);
		}
		auto Object::attributeUint64(const aris::core::XmlElement &xml_ele, const std::string &attribute_name)const->std::uint64_t
		{
			std::string error = "failed to get uint64 attribute \"" + attribute_name + "\" in \"" + type() + "\" \"" + name() + "\", because ";

			if (!xml_ele.Attribute(attribute_name.c_str()))
				throw std::runtime_error(error + "this attribute is not found in xml file");

			try
			{
				return std::stoull(xml_ele.Attribute(attribute_name.c_str()), nullptr, 0);
			}
			catch (std::exception &e)
			{
				throw std::runtime_error(error + "attribute text \"" + xml_ele.Attribute(attribute_name.c_str()) + "\" is invalid:" + e.what());
			}
		}
		auto Object::attributeUint64(const aris::core::XmlElement &xml_ele, const std::string &attribute_name, std::uint64_t default_value)const->std::uint64_t
		{
			if (!xml_ele.Attribute(attribute_name.c_str()))return default_value;
			else return attributeUint64(xml_ele, attribute_name);
		}
		auto Object::attributeUint32(const aris::core::XmlElement &xml_ele, const std::string &attribute_name)const->std::uint32_t
		{
			std::string error = "failed to get uint32 attribute \"" + attribute_name + "\" in \"" + type() + "\" \"" + name() + "\", because ";

			if (!xml_ele.Attribute(attribute_name.c_str()))	throw std::runtime_error(error + "this attribute is not found in xml file");

			unsigned long long value_ull;
			try
			{
				value_ull = std::stoll(xml_ele.Attribute(attribute_name.c_str()), nullptr, 0);
			}
			catch (std::exception &e)
			{
				throw std::runtime_error(error + "attribute text \"" + xml_ele.Attribute(attribute_name.c_str()) + "\" is invalid:" + e.what());
			}

			if (value_ull > std::numeric_limits<std::uint32_t>::max())
				throw std::runtime_error(error + "attribute text \"" + xml_ele.Attribute(attribute_name.c_str()) + "\" is invalid: the value is out of range");

			return static_cast<std::uint32_t>(value_ull);
		}
		auto Object::attributeUint32(const aris::core::XmlElement &xml_ele, const std::string &attribute_name, std::uint32_t default_value)const->std::uint32_t
		{
			if (!xml_ele.Attribute(attribute_name.c_str()))return default_value;
			else return attributeUint32(xml_ele, attribute_name);
		}
		auto Object::attributeUint16(const aris::core::XmlElement &xml_ele, const std::string &attribute_name)const->std::uint16_t
		{
			std::string error = "failed to get uint16 attribute \"" + attribute_name + "\" in \"" + type() + "\" \"" + name() + "\", because ";

			if (!xml_ele.Attribute(attribute_name.c_str()))	throw std::runtime_error(error + "this attribute is not found in xml file");

			unsigned long long value_ull;
			try
			{
				value_ull = std::stoll(xml_ele.Attribute(attribute_name.c_str()), nullptr, 0);
			}
			catch (std::exception &e)
			{
				throw std::runtime_error(error + "attribute text \"" + xml_ele.Attribute(attribute_name.c_str()) + "\" is invalid:" + e.what());
			}

			if (value_ull > std::numeric_limits<std::uint16_t>::max())
				throw std::runtime_error(error + "attribute text \"" + xml_ele.Attribute(attribute_name.c_str()) + "\" is invalid: the value is out of range");

			return static_cast<std::uint16_t>(value_ull);
		}
		auto Object::attributeUint16(const aris::core::XmlElement &xml_ele, const std::string &attribute_name, std::uint16_t default_value)const->std::uint16_t
		{
			if (!xml_ele.Attribute(attribute_name.c_str()))return default_value;
			else return attributeUint16(xml_ele, attribute_name);
		}
		auto Object::attributeUint8(const aris::core::XmlElement &xml_ele, const std::string &attribute_name)const->std::uint8_t
		{
			std::string error = "failed to get uint8 attribute \"" + attribute_name + "\" in \"" + type() + "\" \"" + name() + "\", because ";

			if (!xml_ele.Attribute(attribute_name.c_str()))	throw std::runtime_error(error + "this attribute is not found in xml file");

			unsigned long long value_ull;
			try
			{
				value_ull = std::stoll(xml_ele.Attribute(attribute_name.c_str()), nullptr, 0);
			}
			catch (std::exception &e)
			{
				throw std::runtime_error(error + "attribute text \"" + xml_ele.Attribute(attribute_name.c_str()) + "\" is invalid:" + e.what());
			}

			if (value_ull > std::numeric_limits<std::uint8_t>::max())
				throw std::runtime_error(error + "attribute text \"" + xml_ele.Attribute(attribute_name.c_str()) + "\" is invalid: the value is out of range");

			return static_cast<std::uint8_t>(value_ull);
		}
		auto Object::attributeUint8(const aris::core::XmlElement &xml_ele, const std::string &attribute_name, std::uint8_t default_value)const->std::uint8_t
		{
			if (!xml_ele.Attribute(attribute_name.c_str()))return default_value;
			else return attributeUint8(xml_ele, attribute_name);
		}
		auto Object::attributeFloat(const aris::core::XmlElement &xml_ele, const std::string &attribute_name)const->float
		{
			std::string error = "failed to get double attribute \"" + attribute_name + "\" in \"" + type() + "\" \"" + name() + "\", because ";

			if (!xml_ele.Attribute(attribute_name.c_str()))
				throw std::runtime_error(error + "this attribute is not found in xml file");

			try
			{
				return std::stof(xml_ele.Attribute(attribute_name.c_str()));
			}
			catch (std::exception &e)
			{
				throw std::runtime_error(error + "attribute text \"" + xml_ele.Attribute(attribute_name.c_str()) + "\" is invalid:" + e.what());
			}
		}
		auto Object::attributeFloat(const aris::core::XmlElement &xml_ele, const std::string &attribute_name, float default_value)const->float
		{
			if (!xml_ele.Attribute(attribute_name.c_str()))return default_value;
			else return attributeFloat(xml_ele, attribute_name);
		}
		auto Object::attributeDouble(const aris::core::XmlElement &xml_ele, const std::string &attribute_name)const->double
		{
			std::string error = "failed to get double attribute \"" + attribute_name + "\" in \"" + type() + "\" \"" + name() + "\", because ";

			if (!xml_ele.Attribute(attribute_name.c_str()))
				throw std::runtime_error(error + "this attribute is not found in xml file");

			try
			{
				return std::stod(xml_ele.Attribute(attribute_name.c_str()));
			}
			catch (std::exception &e)
			{
				throw std::runtime_error(error + "attribute text \"" + xml_ele.Attribute(attribute_name.c_str()) + "\" is invalid:" + e.what());
			}
		}
		auto Object::attributeDouble(const aris::core::XmlElement &xml_ele, const std::string &attribute_name, double default_value)const->double
		{
			if (!xml_ele.Attribute(attribute_name.c_str()))return default_value;
			else return attributeDouble(xml_ele, attribute_name);
		}
		auto Object::attributeString(const aris::core::XmlElement &xml_ele, const std::string &attribute_name)const->std::string
		{
			std::string error = "failed to get string attribute \"" + attribute_name + "\" in \"" + type() + "\" \"" + name() + "\", because ";

			if (!xml_ele.Attribute(attribute_name.c_str()))
				throw std::runtime_error(error + "this attribute is not found in xml file");

			return xml_ele.Attribute(attribute_name.c_str());
		}
		auto Object::attributeString(const aris::core::XmlElement &xml_ele, const std::string &attribute_name, const std::string &default_value)const->std::string
		{
			return xml_ele.Attribute(attribute_name.c_str()) ? attributeString(xml_ele, attribute_name) : default_value;
		}
		auto Object::attributeChar(const aris::core::XmlElement &xml_ele, const std::string &attribute_name)const->char
		{
			std::string error = "failed to get char attribute \"" + attribute_name + "\" in \"" + type() + "\" \"" + name() + "\", because ";

			if (!xml_ele.Attribute(attribute_name.c_str()))
				throw std::runtime_error(error + "this attribute is not found in xml file");

			if (xml_ele.Attribute(attribute_name.c_str())[1]!=0)
				throw std::runtime_error(error + "this attribute string length is not 1");

			return xml_ele.Attribute(attribute_name.c_str())[0];
		}
		auto Object::attributeChar(const aris::core::XmlElement &xml_ele, const std::string &attribute_name, char default_value)const->char
		{
			return xml_ele.Attribute(attribute_name.c_str()) ? attributeChar(xml_ele, attribute_name) : default_value;
		}
		auto Object::operator=(const Object &other)->Object &
		{
			Object *father = imp_->father_;
			std::size_t id = imp_->id_;

			imp_ = other.imp_;
			ImpContainer::operator=(other);

			imp_->father_ = father;
			imp_->id_ = id;

			for (std::size_t i = 0; i < size(); ++i)
			{
				at(i).imp_->id_ = i;
				at(i).imp_->father_ = this;
			}

			return *this;
		}
		auto Object::operator=(Object &&other)->Object &
		{
			Object *father = imp_->father_;
			std::size_t id = imp_->id_;

			imp_ = std::move(other.imp_);
			ImpContainer::operator=(std::move(other));

			imp_->father_ = father;
			imp_->id_ = id;

			for (std::size_t i = 0; i < size(); ++i)
			{
				at(i).imp_->id_ = i;
				at(i).imp_->father_ = this;
			}

			return *this;
		}
		Object::~Object() = default;
		Object::Object(const Object &other) :ImpContainer(other), imp_(other.imp_) { for (auto &child : *this)child.imp_->father_ = this; }
		Object::Object(Object &&other) : ImpContainer(std::move(other)), imp_(std::move(other.imp_)) { for (auto &child : *this)child.imp_->father_ = this; }
		Object::Object(Object &father, std::size_t id, const std::string &name) : imp_(new Imp(&father, id, name)) {}
		Object::Object(Object &father, std::size_t id, const aris::core::XmlElement &xml_ele) :imp_(new Imp(&father, id, xml_ele))
		{
			for (auto ele = xml_ele.FirstChildElement(); ele; ele = ele->NextSiblingElement())add(*ele);
		}

		auto Root::TypeInfo::registerTo(const std::string &type, Root &root)->void
		{
			root.imp_->type_map_.insert(std::make_pair(type, *this));
		}
		auto Root::childTypeMap()const ->const std::map<std::string, TypeInfo>&{return imp_->type_map_;	}
		auto Root::loadXml(const std::string &filename)->void
		{
			aris::core::XmlDocument xmlDoc;

			if (xmlDoc.LoadFile(filename.c_str()) != 0)
			{
				throw std::runtime_error((std::string("could not open file:") + std::string(filename)));
			}

			loadXml(xmlDoc);
		}
		auto Root::loadXml(const aris::core::XmlDocument &xml_doc)->void
		{
			loadXml(*xml_doc.RootElement());
		}
		auto Root::loadXml(const aris::core::XmlElement &xml_ele)->void
		{
			clear();
			for (auto ele = xml_ele.FirstChildElement(); ele; ele = ele->NextSiblingElement()) add(*ele);
		}
		auto Root::saveXml(const std::string &filename) const->void
		{
			aris::core::XmlDocument doc;

			saveXml(doc);

			doc.SaveFile(filename.c_str());
		}
		auto Root::saveXml(aris::core::XmlDocument &xml_doc)const->void
		{
			xml_doc.DeleteChildren();

			auto header_xml_ele = xml_doc.NewDeclaration("xml version=\"1.0\" encoding=\"UTF-8\" ");
			xml_doc.InsertEndChild(header_xml_ele);

			auto root_xml_ele = xml_doc.NewElement(this->name().c_str());
			xml_doc.InsertEndChild(root_xml_ele);

			saveXml(*root_xml_ele);
		}
		Root::~Root() {}
		Root::Root(const std::string &name) :Object(*this, 0, name) {}
		Root::Root(const aris::core::XmlElement &xml_ele) :Object(*this, 0, xml_ele) {}
	}
}