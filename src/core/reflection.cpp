#include <string>
#include <iostream>
#include <functional>
#include <vector>
#include <map>
#include <string>
#include <algorithm>
#include <limits>
#include <regex>
#include <charconv>

#include "aris/core/reflection.hpp"

namespace aris::core
{
	auto reflect_types()->std::map<std::size_t, Type>&
	{
		static std::map<std::size_t, Type> reflection_types_;
		return reflection_types_;
	}
	auto reflect_names()->std::map<std::string, std::size_t>&
	{
		static std::map<std::string, std::size_t> reflection_names_;
		return reflection_names_;
	}

	auto Type::create()const->std::tuple<std::unique_ptr<void, void(*)(void const*)>, Instance>
	{ 
		return default_ctor_();
	}

	auto Instance::toVoidPtr()->void* {return isReference() ? std::any_cast<InstanceRef>(&value_)->data_ : type()->any_to_void_(&value_);}
	auto Instance::set(std::string_view prop_name, const Instance &arg)->void
	{
		type()->properties().at(std::string(prop_name)).set_(toVoidPtr(), arg);
	}
	auto Instance::get(std::string_view prop_name)->Instance
	{
		return type()->properties().at(std::string(prop_name)).get_(toVoidPtr());
	}
	auto Instance::isReference()->bool { return std::any_cast<InstanceRef>(&value_); }
	auto Instance::type()const->const Type* 
	{ 
		auto type_info_ = std::any_cast<InstanceRef>(&value_) ? std::any_cast<InstanceRef>(&value_)->type_ : &value_.type();
		return reflect_types().find(type_info_->hash_code()) == reflect_types().end() ? nullptr : &reflect_types().at(type_info_->hash_code());
	}
	auto Instance::isBasic()->bool { return type()->is_basic_; }
	auto Instance::isArray()->bool { return type()->is_array_; }
	auto Instance::toString()->std::string 
	{ 
		if (!isBasic())THROW_FILE_LINE("instance is NOT basic type");
		return type()->to_string_(toVoidPtr()); 
	}
	auto Instance::fromString(std::string_view str)->void 
	{ 
		if (!isBasic())THROW_FILE_LINE("instance is NOT basic type");
		type()->from_string_(toVoidPtr(), str); 
	}
	auto Instance::size()->std::size_t
	{
		if (!isArray())THROW_FILE_LINE("instance is NOT array");
		return type()->size_func_(toVoidPtr());
	}
	auto Instance::at(std::size_t id)->Instance
	{
		if (!isArray())THROW_FILE_LINE("instance is NOT array");
		return type()->at_func_(toVoidPtr(), id);
	}
	auto Instance::push_back(const Instance &element)->void
	{
		if (!isArray())THROW_FILE_LINE("instance is NOT array");
		type()->push_back_func_(toVoidPtr(), element);
	}

	auto Property::set(Instance *ins, const Instance& arg)const->void{ set_(ins->toVoidPtr(), arg); }
	auto Property::get(Instance *ins)const->Instance { return get_(ins->toVoidPtr()); }
	auto Property::acceptPtr()const->bool { return accept_ptr_; }

	template<typename Type>
	auto inline get_chars(std::string_view param)->Type
	{
		Type ret;
		auto result = std::from_chars(param.data(), param.data() + param.size(), ret);
		if (result.ec == std::errc::invalid_argument) { THROW_FILE_LINE("invalid argument for param:" + std::string(param)); }
		
		return ret;
	}

#define ARIS_REGISTRATION_BASIC(T, NAME) \
	aris::core::class_<T>(NAME) \
		.asBasic([](void *v)->std::string \
		{ \
			return std::to_string(*reinterpret_cast<T*>(v)); \
		}, [](void *v, std::string_view str)->void \
		{ \
			auto result = std::from_chars(str.data(), str.data() + str.size(), *reinterpret_cast<T*>(v)); \
			if (result.ec == std::errc::invalid_argument) THROW_FILE_LINE("invalid string"); \
		});


	ARIS_REGISTRATION
	{
		ARIS_REGISTRATION_BASIC(std::int8_t, "int8");
		ARIS_REGISTRATION_BASIC(std::int16_t, "int16");
		ARIS_REGISTRATION_BASIC(std::int64_t, "int64");
		ARIS_REGISTRATION_BASIC(std::uint8_t, "uint8");
		ARIS_REGISTRATION_BASIC(std::uint16_t, "uint16");
		ARIS_REGISTRATION_BASIC(std::uint32_t, "uint32");
		ARIS_REGISTRATION_BASIC(std::uint64_t, "uint64");

		aris::core::class_<std::int32_t>("int32")
			.asBasic([](void *v)->std::string
			{
				return std::to_string(*reinterpret_cast<std::int32_t*>(v));
			},[](void *v,std::string_view str)->void
			{
				auto result = std::from_chars(str.data(), str.data() + str.size(), *reinterpret_cast<std::int32_t*>(v));
				if (result.ec == std::errc::invalid_argument) THROW_FILE_LINE("invalid string");
			})
			.alias("int");

		aris::core::class_<float>("float")
			.asBasic([](void *v)->std::string
			{
				return std::to_string(*reinterpret_cast<float*>(v));
			}, [](void *v, std::string_view str)->void
			{
				float result = std::stod(std::string(str));
				*reinterpret_cast<float*>(v) = result;
			});

		aris::core::class_<double>("double")
			.asBasic([](void *v)->std::string
			{
				return std::to_string(*reinterpret_cast<double*>(v));
			}, [](void *v, std::string_view str)->void
			{
				double result = std::stod(std::string(str));
				*reinterpret_cast<double*>(v) = result;
			});

		aris::core::class_<bool>("bool")
			.asBasic([](void *v)->std::string
			{
				return *reinterpret_cast<bool*>(v) ? "true" : "false";
			}, [](void *v, std::string_view str)->void
			{
				bool result;
				if (str == "true")
					result = true;
				else if (str == "false")
					result = false;
				else
					THROW_FILE_LINE("invalid string for bool");
				
				*reinterpret_cast<bool*>(v) = result;
			});

			
		aris::core::class_<std::string>("string")
			.asBasic([](void *v)->std::string
			{
				return *reinterpret_cast<std::string*>(v);
			}, [](void *v, std::string_view str)->void
			{
				*reinterpret_cast<std::string*>(v) = str;
			});
	}
}