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
		struct Object::Imp 
		{
			// 这里添加的任何内容请在Object里手动构造
			Object *father_{nullptr};
			std::size_t id_{0};
			std::string name_{"object"};
			std::string default_type_{ Object::Type() };
			std::map<std::string, std::shared_ptr<Object> > save_data_map_;
			ImpContainer<Object> children_;

			Imp() = default;
			Imp(const std::string &name) :name_(name) {}
			Imp(Object *father, const aris::core::XmlElement &xml_ele) :father_(father), name_(xml_ele.Name()), default_type_(xml_ele.Attribute("default_child_type")? xml_ele.Attribute("default_child_type"): Object::Type()){}
			Imp(const Imp &other) = delete;
			Imp(Imp &&other) = default;
			Imp &operator=(const Imp &) = delete;
			Imp &operator=(Imp &&) = default;
		};
		auto Object::attributeBool(const aris::core::XmlElement &xml_ele, const std::string &attribute_name)->bool
		{
			std::string error = "failed to get bool attribute \"" + attribute_name + "\" in element \"" + xml_ele.Name() + "\", because ";

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
		auto Object::attributeBool(const aris::core::XmlElement &xml_ele, const std::string &attribute_name, bool default_value)->bool
		{
			return xml_ele.Attribute(attribute_name.c_str()) ? attributeBool(xml_ele, attribute_name) : default_value;
		}
		auto Object::attributeInt64(const aris::core::XmlElement &xml_ele, const std::string &attribute_name)->std::int64_t
		{
			std::string error = "failed to get int64 attribute \"" + attribute_name + "\" in element \"" + xml_ele.Name() + "\", because ";

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
		auto Object::attributeInt64(const aris::core::XmlElement &xml_ele, const std::string &attribute_name, std::int64_t default_value)->std::int64_t
		{
			return xml_ele.Attribute(attribute_name.c_str()) ? attributeInt64(xml_ele, attribute_name) : default_value;
		}
		auto Object::attributeInt32(const aris::core::XmlElement &xml_ele, const std::string &attribute_name)->std::int32_t
		{
			std::string error = "failed to get int32 attribute \"" + attribute_name + "\" in element \"" + xml_ele.Name() + "\", because ";

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
		auto Object::attributeInt32(const aris::core::XmlElement &xml_ele, const std::string &attribute_name, std::int32_t default_value)->std::int32_t
		{
			return xml_ele.Attribute(attribute_name.c_str()) ? attributeInt32(xml_ele, attribute_name) : default_value;
		}
		auto Object::attributeInt16(const aris::core::XmlElement &xml_ele, const std::string &attribute_name)->std::int16_t
		{
			std::string error = "failed to get int16 attribute \"" + attribute_name + "\" in element \"" + xml_ele.Name() + "\", because ";

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
		auto Object::attributeInt16(const aris::core::XmlElement &xml_ele, const std::string &attribute_name, std::int16_t default_value)->std::int16_t
		{
			return xml_ele.Attribute(attribute_name.c_str()) ? attributeInt16(xml_ele, attribute_name) : default_value;
		}
		auto Object::attributeInt8(const aris::core::XmlElement &xml_ele, const std::string &attribute_name)->std::int8_t
		{
			std::string error = "failed to get int16 attribute \"" + attribute_name + "\" in element \"" + xml_ele.Name() + "\", because ";

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
		auto Object::attributeInt8(const aris::core::XmlElement &xml_ele, const std::string &attribute_name, std::int8_t default_value)->std::int8_t
		{
			return xml_ele.Attribute(attribute_name.c_str()) ? attributeInt8(xml_ele, attribute_name) : default_value;
		}
		auto Object::attributeUint64(const aris::core::XmlElement &xml_ele, const std::string &attribute_name)->std::uint64_t
		{
			std::string error = "failed to get uint64 attribute \"" + attribute_name + "\" in element \"" + xml_ele.Name() + "\", because ";

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
		auto Object::attributeUint64(const aris::core::XmlElement &xml_ele, const std::string &attribute_name, std::uint64_t default_value)->std::uint64_t
		{
			return xml_ele.Attribute(attribute_name.c_str()) ? attributeUint64(xml_ele, attribute_name) : default_value;
		}
		auto Object::attributeUint32(const aris::core::XmlElement &xml_ele, const std::string &attribute_name)->std::uint32_t
		{
			std::string error = "failed to get uint32 attribute \"" + attribute_name + "\" in element \"" + xml_ele.Name() + "\", because ";

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
		auto Object::attributeUint32(const aris::core::XmlElement &xml_ele, const std::string &attribute_name, std::uint32_t default_value)->std::uint32_t
		{
			return xml_ele.Attribute(attribute_name.c_str()) ? attributeUint32(xml_ele, attribute_name) : default_value;
		}
		auto Object::attributeUint16(const aris::core::XmlElement &xml_ele, const std::string &attribute_name)->std::uint16_t
		{
			std::string error = "failed to get uint16 attribute \"" + attribute_name + "\" in element \"" + xml_ele.Name() + "\", because ";

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
		auto Object::attributeUint16(const aris::core::XmlElement &xml_ele, const std::string &attribute_name, std::uint16_t default_value)->std::uint16_t
		{
			return xml_ele.Attribute(attribute_name.c_str()) ? attributeUint16(xml_ele, attribute_name) : default_value;
		}
		auto Object::attributeUint8(const aris::core::XmlElement &xml_ele, const std::string &attribute_name)->std::uint8_t
		{
			std::string error = "failed to get uint8 attribute \"" + attribute_name + "\" in element \"" + xml_ele.Name() + "\", because ";

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
		auto Object::attributeUint8(const aris::core::XmlElement &xml_ele, const std::string &attribute_name, std::uint8_t default_value)->std::uint8_t
		{
			return xml_ele.Attribute(attribute_name.c_str()) ? attributeUint8(xml_ele, attribute_name) : default_value;
		}
		auto Object::attributeFloat(const aris::core::XmlElement &xml_ele, const std::string &attribute_name)->float
		{
			std::string error = "failed to get double attribute \"" + attribute_name + "\" in element \"" + xml_ele.Name() + "\", because ";

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
		auto Object::attributeFloat(const aris::core::XmlElement &xml_ele, const std::string &attribute_name, float default_value)->float
		{
			return xml_ele.Attribute(attribute_name.c_str()) ? attributeFloat(xml_ele, attribute_name) : default_value;
		}
		auto Object::attributeDouble(const aris::core::XmlElement &xml_ele, const std::string &attribute_name)->double
		{
			std::string error = "failed to get double attribute \"" + attribute_name + "\" in element \"" + xml_ele.Name() + "\", because ";

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
		auto Object::attributeDouble(const aris::core::XmlElement &xml_ele, const std::string &attribute_name, double default_value)->double
		{
			return xml_ele.Attribute(attribute_name.c_str()) ? attributeDouble(xml_ele, attribute_name) : default_value;
		}
		auto Object::attributeString(const aris::core::XmlElement &xml_ele, const std::string &attribute_name)->std::string
		{
			std::string error = "failed to get string attribute \"" + attribute_name + "\" in element \"" + xml_ele.Name() + "\", because ";

			if (!xml_ele.Attribute(attribute_name.c_str()))
				throw std::runtime_error(error + "this attribute is not found in xml file");

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
				throw std::runtime_error(error + "this attribute is not found in xml file");

			if (xml_ele.Attribute(attribute_name.c_str())[1] != 0)
				throw std::runtime_error(error + "this attribute string length is not 1");

			return xml_ele.Attribute(attribute_name.c_str())[0];
		}
		auto Object::attributeChar(const aris::core::XmlElement &xml_ele, const std::string &attribute_name, char default_value)->char
		{
			return xml_ele.Attribute(attribute_name.c_str()) ? attributeChar(xml_ele, attribute_name) : default_value;
		}
		auto Object::root()->Root& { return dynamic_cast<Root *>(this) ? dynamic_cast<Root &>(*this) : father().root(); }
		auto Object::root()const->const Root&{ return dynamic_cast<const Root *>(this) ? dynamic_cast<const Root &>(*this) : father().root(); }
		auto Object::saveXml(aris::core::XmlElement &xml_ele) const->void
		{ 
			xml_ele.DeleteChildren();
			xml_ele.SetName(name().c_str());
			xml_ele.SetAttribute("type", this->type().c_str());
			if (!imp_->default_type_.empty() && imp_->default_type_ != Object::Type())xml_ele.SetAttribute("default_child_type", imp_->default_type_.c_str());
			for (auto &ele : children())
			{
				auto new_ele = xml_ele.GetDocument()->NewElement(ele.name().c_str());
				ele.saveXml(*new_ele);
				xml_ele.InsertEndChild(new_ele);
			}
		}
		auto Object::xmlString()->std::string 
		{
			XmlDocument doc;
			auto new_ele = doc.NewElement("a");
			saveXml(*new_ele);
			doc.InsertEndChild(new_ele);
			
			tinyxml2::XMLPrinter printer;
			doc.Print(&printer);

			return std::string(printer.CStr());;
		}
		auto Object::id()const->std::size_t { return imp_->id_; }
		auto Object::name() const->const std::string&{ return imp_->name_; }
		auto Object::father()->Object& { return *imp_->father_; }
		auto Object::father()const->const Object&{ return *imp_->father_; }
		auto Object::children()const->const ImpContainer<Object>&{ return imp_->children_; };
		auto Object::children()->ImpContainer<Object>& { return imp_->children_; };
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
		auto Object::findByName(const std::string &name)->ImpContainer<Object>::iterator
		{
			return std::find_if(children().begin(), children().end(), [&name, this](Object & p) {return (p.name() == name); });
		}
		auto Object::findByName(const std::string &name) const->ImpContainer<Object>::const_iterator
		{
			return const_cast<Object *>(this)->findByName(name);
		}
		auto Object::add(Object *obj)->Object &
		{
			children().push_back_ptr(obj);
			children().back().imp_->id_ = children().size() - 1;
			children().back().imp_->father_ = this;

			return children().back();
		}
		auto Object::add(const aris::core::XmlElement &xml_ele)->Object &
		{
			std::string type = xml_ele.Attribute("type") ? xml_ele.Attribute("type") : imp_->default_type_;

			if (root().childTypeMap().find(type) == root().childTypeMap().end())
			{
				throw std::runtime_error("unrecognized type \"" + type + "\" when add xml element \"" + xml_ele.Name() + "\" to \"" + name() + "\"");
			}
			else
			{
				children().push_back_ptr(root().childTypeMap().find(type)->second.xml_construct_func(*this, xml_ele));
				children().back().imp_->id_ = children().size() - 1;
				children().back().imp_->father_ = this;

				return children().back();
			}
		}
		Object::~Object() = default;
		Object::Object(const std::string &name) : imp_(new Imp(name)) {}
		Object::Object(Object &father, const aris::core::XmlElement &xml_ele) : imp_(new Imp(&father, xml_ele))
		{
			for (auto ele = xml_ele.FirstChildElement(); ele; ele = ele->NextSiblingElement()) add(*ele);
		}
		Object::Object(const Object &other)
		{ 
			imp_->father_ = other.imp_->father_;
			imp_->id_ = other.imp_->id_;
			imp_->name_ = other.imp_->name_;
			imp_->default_type_ = other.imp_->default_type_;
			imp_->save_data_map_ = other.imp_->save_data_map_;

			for (auto&child : other.children())
			{
				children().push_back_ptr(other.root().childTypeMap().at(child.type()).copy_construct_func(child));
				children().back().imp_->father_ = this;
			}
		}
		Object::Object(Object &&other) : imp_(std::move(other.imp_)) { for (auto &child : children())child.imp_->father_ = this; }
		Object& Object::operator=(const Object &other)
		{
			imp_->name_ = other.imp_->name_;
			imp_->default_type_ = other.imp_->default_type_;
			imp_->save_data_map_ = other.imp_->save_data_map_;
			
			if (children().size() > other.children().size())
				children().erase(children().begin() + other.children().size(), children().end());

			for (std::size_t i = 0; i < other.children().size(); ++i)
			{
				if (i >= children().size())
				{
					children().push_back_ptr(other.root().childTypeMap().at(other.children().at(i).type()).copy_construct_func(other.children().at(i)));
				}
				else if (children().at(i).type() != other.children().at(i).type())
				{
					children().container_.at(i).reset(other.root().childTypeMap().at(other.children().at(i).type()).copy_construct_func(other.children().at(i)));
				}
				else
				{
					other.children().at(i).root().childTypeMap().at(other.children().at(i).type()).copy_assign_func(other.children().at(i), children().at(i));
				}

				children().at(i).imp_->id_ = i;
				children().at(i).imp_->father_ = this;
			}

			return *this;
		}
		Object& Object::operator=(Object &&other)
		{
			imp_->name_ = std::move(other.imp_->name_);
			imp_->default_type_ = std::move(other.imp_->default_type_);
			imp_->save_data_map_ = std::move(other.imp_->save_data_map_);

			if (children().size() > other.children().size())
				children().erase(children().begin() + other.children().size(), children().end());
			
			for (std::size_t i = 0; i < other.children().size(); ++i)
			{
				if (i >= children().size()) 
				{
					children().push_back_ptr(other.root().childTypeMap().at(other.children().at(i).type()).move_construct_func(std::move(other.children().at(i))));
				}
				else if (children().at(i).type() != other.children().at(i).type())
				{
					children().container_.at(i).reset(other.root().childTypeMap().at(other.children().at(i).type()).move_construct_func(std::move(other.children().at(i))));
				}
				else
				{
					other.children().at(i).root().childTypeMap().at(other.children().at(i).type()).move_assign_func(std::move(other.children().at(i)), children().at(i));
				}

				children().at(i).imp_->id_ = i;
				children().at(i).imp_->father_ = this;
			}

			return *this;
		}

		struct Root::Imp
		{
			std::map<std::string, TypeInfo> type_map_{ std::make_pair(Object::Type(),TypeInfo::CreateTypeInfo<Object>()) };

			Imp() = default;
			Imp(const Imp &other) = default;
			Imp(Imp &&other) = default;
			Imp &operator=(const Imp &) = default;
			Imp &operator=(Imp &&) = default;
		};
		auto Root::TypeInfo::registerTo(const std::string &type, Root &root)->void
		{
			root.imp_->type_map_.insert(std::make_pair(type, *this));
		}
		auto Root::childTypeMap()const ->const std::map<std::string, TypeInfo>&{return imp_->type_map_;	}
		auto Root::loadXml(const std::string &filename)->void
		{
			aris::core::XmlDocument xmlDoc;

			if (xmlDoc.LoadFile(filename.c_str()) != 0)throw std::runtime_error((std::string("could not open file:") + std::string(filename)));

			loadXml(xmlDoc);
		}
		auto Root::loadXml(const aris::core::XmlDocument &xml_doc)->void{loadXml(*xml_doc.RootElement());}
		auto Root::loadXml(const aris::core::XmlElement &xml_ele)->void
		{
			Object::imp_->name_ = xml_ele.Name();
			children().clear();
			for (auto ele = xml_ele.FirstChildElement(); ele; ele = ele->NextSiblingElement())add(*ele);
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

			auto root_xml_ele = xml_doc.NewElement(name().c_str());
			xml_doc.InsertEndChild(root_xml_ele);

			saveXml(*root_xml_ele);
		}
		auto Root::save(const std::string &name, bool auto_override_save)->void
		{
			decltype(Object::imp_->save_data_map_) tem = std::move(Object::imp_->save_data_map_);

			if ((!auto_override_save) && (tem.find(name) != tem.end()))
				throw std::runtime_error("Root \"" + this->name() + "\" already has a save named \"" + name + "\", can't save");

			auto save_content = std::shared_ptr<Object>(new Root(*this));
			tem.insert(std::make_pair(name, save_content));

			Object::imp_->save_data_map_ = std::move(tem);
		}
		auto Root::load(const std::string &name, bool auto_delete_save)->void
		{
			decltype(Object::imp_->save_data_map_) tem = std::move(Object::imp_->save_data_map_);

			if (tem.find(name) == tem.end())
				throw std::runtime_error("Object \"" + this->name() + "\" does not has a save named \"" + name + "\", can't load");
			*this = static_cast<Root&>(*tem.at(name));
			if (auto_delete_save)tem.erase(name);

			Object::imp_->save_data_map_ = std::move(tem);
		}
		Root::~Root() = default;
		Root::Root(const std::string &name) :Object(name) { registerChildType<Object>(); }
		Root::Root(const aris::core::XmlElement &xml_ele) :Object(*this, xml_ele) { registerChildType<Object>(); }
		Root::Root(const Root&) = default;
		Root::Root(Root&&) = default;
		Root& Root::operator=(const Root&) = default;
		Root& Root::operator=(Root&&) = default;
	}
}