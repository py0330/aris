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

	auto Property::set(Instance *ins, const Instance& arg)const->void { set_(ins->toVoidPtr(), arg); }
	auto Property::get(Instance *ins)const->Instance 
	{ 
		auto ret = get_(ins->toVoidPtr());
		ret.belong_to_ = this;
		return ret;
	}
	auto Property::acceptPtr()const->bool { return accept_ptr_; }

	auto Type::create()const->std::tuple<std::unique_ptr<void, void(*)(void const*)>, Instance>
	{ 
		return default_ctor_();
	}
	auto Type::inheritTypes()const->std::vector<const Type*>
	{
		if (inherit_types_.empty())
		{
			for (auto &t : inherit_type_infos_)
			{
				const_cast<std::vector<const Type *>&>(inherit_types_).push_back(&reflect_types().at(t->hash_code()));
			}
		}
		return inherit_types_;
	}
	auto Type::properties()const->const std::vector<Property*>&
	{
		// 第一次调用构造所有的property //
		if (!inited)
		{
			auto &vec = const_cast<std::vector<Property*>&>(properties_ptr_);
			
			// 防止重名的数据
			auto insert_prop = [&vec](Property* ist) 
			{
				auto found = std::find_if(vec.begin(), vec.end(), [ist](Property* prop) ->bool
				{
					return prop->name() == ist->name();
				});

				if (found == vec.end())
					vec.push_back(ist);
				else
					*found = ist;
			};

			for (auto t : inheritTypes())
			{
				for (auto ist : t->properties())
				{
					insert_prop(ist);
				}
			}

			for (auto &ist : this_properties_)
			{
				insert_prop(const_cast<Property*>(&ist));
			}

			const_cast<Type*>(this)->inited = true;
		}

		return properties_ptr_;
	};
	auto Type::propertyAt(std::string_view name)const->Property*
	{
		auto found = std::find_if(properties().begin(), properties().end(), [name](Property* prop) ->bool
		{
			return prop->name() == name;
		});

		return found == properties().end() ? nullptr : *found;
	}
	auto Instance::toVoidPtr()->void* {return isReference() ? std::any_cast<InstanceRef>(&value_)->data_ : type()->any_to_void_(&value_);}
	auto Instance::set(std::string_view prop_name, Instance arg)->void
	{
		type()->propertyAt(prop_name)->set(this, arg);
	}
	auto Instance::get(std::string_view prop_name)->Instance
	{
		return type()->propertyAt(prop_name)->get(this);
	}
	auto Instance::isReference()->bool { return std::any_cast<InstanceRef>(&value_); }
	auto Instance::type()->const Type* 
	{ 
		auto type_info_ = std::any_cast<InstanceRef>(&value_) ? std::any_cast<InstanceRef>(&value_)->type_ : &value_.type();
		return reflect_types().find(type_info_->hash_code()) == reflect_types().end() ? nullptr : &reflect_types().at(type_info_->hash_code());
	}
	auto Instance::isBasic()->bool { return type()->is_basic_; }
	auto Instance::isArray()->bool { return type()->is_array_; }
	auto Instance::toString()->std::string 
	{ 
		if (!isBasic())THROW_FILE_LINE("instance is NOT basic type");
		if (belong_to_ && belong_to_->to_str_func_)return belong_to_->to_str_func_(toVoidPtr());
		return type()->to_string_(toVoidPtr());
	}
	auto Instance::fromString(std::string_view str)->void 
	{ 
		if (!isBasic())THROW_FILE_LINE("instance is NOT basic type");
		if (belong_to_ && belong_to_->from_str_func_)return belong_to_->from_str_func_(toVoidPtr(), str);
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
	auto Instance::push_back(Instance element)->void
	{
		if (!isArray())THROW_FILE_LINE("instance is NOT array");
		type()->push_back_func_(toVoidPtr(), element);
	}

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
		ARIS_REGISTRATION_BASIC(std::int16_t, "int16");
		ARIS_REGISTRATION_BASIC(std::int64_t, "int64");
		ARIS_REGISTRATION_BASIC(std::uint8_t, "uint8");
		ARIS_REGISTRATION_BASIC(std::uint16_t, "uint16");
		ARIS_REGISTRATION_BASIC(std::uint32_t, "uint32");
		ARIS_REGISTRATION_BASIC(std::uint64_t, "uint64");

		aris::core::class_<char>("char")
			.asBasic([](void *v)->std::string
			{
				return std::to_string(*reinterpret_cast<char*>(v));
			}, [](void *v, std::string_view str)->void
			{
				auto result = std::from_chars(str.data(), str.data() + str.size(), *reinterpret_cast<char*>(v));
				if (result.ec == std::errc::invalid_argument) THROW_FILE_LINE("invalid string");
			})
			.alias("int8");

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