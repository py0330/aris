#include <string>
#include <iostream>
#include <functional>
#include <vector>
#include <map>
#include <string>
#include <algorithm>
#include <limits>
#include <regex>

#include "aris/core/reflection.hpp"

namespace aris::core
{
	auto reflect_types()->std::map<std::size_t, Type>&
	{
		static std::map<std::size_t, Type> reflection_types_;
		return reflection_types_;
	}
	auto reflect_names()->std::map<std::string_view, std::size_t>&
	{
		static std::map<std::string_view, std::size_t> reflection_names_;
		return reflection_names_;
	}

	Variant::Variant(const std::any &value) :value_(value){}
	auto Variant::set(std::string_view prop_name, std::any arg)->void { type()->properties().at(std::string(prop_name)).set_(type()->any_to_void_(&value_), arg); }
	auto Variant::get(std::string_view prop_name)->Variant { return type()->properties().at(std::string(prop_name)).get_(type()->any_to_void_(&value_)); }
	auto Variant::type()->const Type* { return reflect_types().find(value_.type().hash_code()) == reflect_types().end() ? nullptr : &reflect_types().at(value_.type().hash_code()); }
	auto Variant::basic()->bool { return type()->is_basic_; }
	auto Variant::toString()->std::string { return type()->to_string_(type()->any_to_void_(&value_)); }
	auto Variant::fromString(std::string_view str)->void { return type()->from_string_(type()->any_to_void_(&value_), str); }


	auto Instance::toVoid()->void* {return isReference() ? std::any_cast<void*>(value_) : type()->any_to_void_(&value_);}
	auto Instance::set(std::string_view prop_name, std::any arg)->void
	{
		type()->properties().at(std::string(prop_name)).set_(toVoid(), arg);
	}
	auto Instance::get(std::string_view prop_name)->Instance
	{
		return type()->properties().at(std::string(prop_name)).get_(toVoid());
	}
	auto Instance::isReference()->bool { return std::any_cast<void**>(&value_) != nullptr; }
	auto Instance::type()->const Type* { return reflect_types().find(type_->hash_code()) == reflect_types().end() ? nullptr : &reflect_types().at(type_->hash_code()); }
	auto Instance::basic()->bool { return type()->is_basic_; }
	auto Instance::toString()->std::string { return type()->to_string_(toVoid()); }
	auto Instance::fromString(std::string_view str)->void { type()->from_string_(toVoid(), str); }

	auto Property::set(Instance *ins, std::any arg)->void 
	{
		ins->set(name(), arg);
	}
	auto Property::get(Instance *ins)->Instance
	{
		return ins->get(name());
	}
	auto Property::set(Variant *var, std::any arg)->void
	{
		var->set(name(), arg);
	}
	auto Property::get(Variant *var)->Variant
	{
		return var->get(name());
	}

	ARIS_REGISTRATION
	{
		aris::core::class_<int>("int")
			.basic([](void *v)->std::string
			{
				return std::to_string(*reinterpret_cast<int*>(v)); 
			},[](void *v,std::string_view str)->void
			{
				*reinterpret_cast<int*>(v) = std::stoi(std::string(str.data())); 
			});
	}
}