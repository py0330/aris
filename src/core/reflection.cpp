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
	auto reflect_types_raw()->std::map<std::size_t, Type>&
	{
		static std::map<std::size_t, Type> reflection_types_;
		return reflection_types_;
	}
	auto initAllTypes()->void
	{
		static bool init_all_ = false;

		if (init_all_)return;
		init_all_ = true;

		for (auto &t : reflect_types_raw())
		{
			t.second.init();
		}
	}
	auto reflect_types()->std::map<std::size_t, Type>&
	{
		initAllTypes();
		return reflect_types_raw();
	}
	auto reflect_names()->std::map<std::string, std::size_t>&
	{
		static std::map<std::string, std::size_t> reflection_names_;
		return reflection_names_;
	}

	auto registerType(std::size_t hash_code, std::string_view name, Type::DefaultCtor ctor)->Type*
	{
		auto[ins, ok] = reflect_types_raw().emplace(std::make_pair(hash_code, Type(name)));
		if (!ok) THROW_FILE_LINE("class already exist");
		auto[ins2, ok2] = reflect_names().emplace(std::make_pair(ins->second.name(), hash_code));
		if (!ok2) THROW_FILE_LINE("class name already exist");

		ins->second.default_ctor_ = ctor;

		return &ins->second;
	}
	auto alias_impl(Type*type, std::string_view alias_name)->void
	{
		auto code = reflect_names().at(std::string(type->name()));
		auto[ins2, ok2] = reflect_names().emplace(std::make_pair(std::string(alias_name), code));
		if (!ok2) THROW_FILE_LINE("class name already exist");
	}

	auto Property::set(Instance *ins, Instance arg)const->void { set_(ins, arg); }
	auto Property::get(Instance *ins)const->Instance 
	{ 
		auto ret = get_(ins);
		ret.belong_to_ = this;
		return ret;
	}
	auto Property::acceptPtr()const->bool { return accept_ptr_; }
	auto Property::type()->const Type* { return &reflect_types().at(type_info_->hash_code()); }
	auto Type::create()const->std::tuple<std::unique_ptr<void, void(*)(void const*)>, Instance>{ return default_ctor_();}
	auto Type::inheritTypes()const->std::vector<const Type*>{ return inherit_types_; }
	auto Type::properties()const->const std::vector<Property*>&{return properties_ptr_;};
	auto Type::propertyAt(std::string_view name)const->Property*
	{
		auto found = std::find_if(properties().begin(), properties().end(), [name](Property* prop) ->bool
		{
			return prop->name() == name;
		});
		return found == properties().end() ? nullptr : *found;
	}
	auto Type::init()const->void
	{
		if (!inited)
		{
			auto this_type = const_cast<Type*>(this);
			this_type->inited = true;
			
			// init inherit types //
			for (auto &t : inherit_type_infos_)
			{
				this_type->inherit_types_.push_back(&reflect_types().at(t->hash_code()));
				inherit_types_.back()->init();
			}
			
			// init inherit properties //
			auto insert_prop = [this_type](Property* ist)
			{
				// 防止重名的数据
				auto &vec = this_type->properties_ptr_;
				auto found = std::find_if(vec.begin(), vec.end(), [ist](Property* prop) ->bool
				{
					return prop->name() == ist->name();
				});

				if (found == vec.end())	vec.push_back(ist);
				else *found = ist;
			};
			for (auto t : inheritTypes())
			{
				for (auto ist : t->properties())
				{
					insert_prop(ist);
				}
			}
			for (auto &ist : this_properties_)insert_prop(const_cast<Property*>(&ist));

			// copy inherited, text methods & array_infos //
			for (auto &t : inherit_types_)
			{
				// copy array_infos //
				if (!isArray() && t->isArray())
				{
					this_type->array_data_.reset(new Type::ArrayData(*t->array_data_));
				}

				// copy text methods //
				if (!to_string_)this_type->to_string_ = t->to_string_;
				if (!from_string_)this_type->from_string_ = t->from_string_;
			}
		}
	}
	auto Type::isBaseOf(const Type* base, const Type* derived)->bool
	{
		if (base == derived || std::find(derived->inherit_types_.begin(), derived->inherit_types_.end(), base) != derived->inherit_types_.end())
		{
			return true;
		}

		for (auto t : derived->inherit_types_)
		{
			if (isBaseOf(base, t)) return true;
		}

		return false;
	}
	auto Instance::castToType(const Type*t)const->void* 
	{
		if (type() == t) return toVoidPtr();

		std::function<void*(const Type*, void*)> iterative_cast = [&](const Type*type, void* data)->void*
		{
			auto inherit_types = type->inheritTypes();

			for (Size i = 0; i < inherit_types.size(); ++i)
			{
				if (inherit_types[i] == t)
					return type->inherit_cast_vec_[i](data);
				
				if (auto success = iterative_cast(inherit_types[i], type->inherit_cast_vec_[i](data)))
					return success;
			}

			return nullptr;
		};

		return iterative_cast(type(), toVoidPtr());
	}
	auto Instance::toVoidPtr()const->void* {return isReference() ? std::get<InstanceRef>(data_).data_ : std::get<InstancePtr>(data_).data_.get();}
	auto Instance::set(std::string_view prop_name, Instance arg)->void
	{
		type()->propertyAt(prop_name)->set(this, arg);
	}
	auto Instance::get(std::string_view prop_name)->Instance
	{
		return type()->propertyAt(prop_name)->get(this);
	}
	auto Instance::isReference()const->bool { return data_.index() == 1; }
	auto Instance::type()const->const Type* 
	{ 
		if (isEmpty()) return nullptr;
		
		return reflect_types().find(type_info_->hash_code()) == reflect_types().end() ? nullptr : &reflect_types().at(type_info_->hash_code());
	}
	auto Instance::isBasic()const->bool { return type()->isBasic(); }
	auto Instance::isArray()const->bool { return type()->isArray(); }
	auto Instance::toString()->std::string 
	{ 
		if (belong_to_ && belong_to_->to_str_func_)return belong_to_->to_str_func_(toVoidPtr());
		if (type()->to_string_)return type()->to_string_(this);
		return "";
	}
	auto Instance::fromString(std::string_view str)->void 
	{ 
		if (belong_to_ && belong_to_->from_str_func_)return belong_to_->from_str_func_(toVoidPtr(), str);
		if (type()->from_string_)type()->from_string_(this, str);
	}
	auto Instance::size()->std::size_t
	{
		if (!isArray())THROW_FILE_LINE("instance is NOT array");
		return type()->array_data_->size_func_(this);
	}
	auto Instance::at(std::size_t id)->Instance
	{
		if (!isArray())THROW_FILE_LINE("instance is NOT array");
		return type()->array_data_->at_func_(this, id);
	}
	auto Instance::push_back(Instance element)->void
	{
		if (!isArray())THROW_FILE_LINE("instance is NOT array");
		type()->array_data_->push_back_func_(this, element);
	}

	auto uint64_to_str(std::uint64_t* value)->std::string { return std::to_string(*reinterpret_cast<std::uint64_t*>(value)); }
	auto uint64_from_str(std::uint64_t *v, std::string_view str)->void { *reinterpret_cast<std::uint64_t*>(v) = std::strtoull(str.data(), nullptr, 0); }
	auto uint32_to_str(std::uint32_t* value)->std::string { return std::to_string(*reinterpret_cast<std::uint32_t*>(value)); }
	auto uint32_from_str(std::uint32_t *v, std::string_view str)->void { *reinterpret_cast<std::uint32_t*>(v) = (std::uint32_t)std::strtoull(str.data(), nullptr, 0); }
	auto uint16_to_str(std::uint16_t* value)->std::string { return std::to_string(*reinterpret_cast<std::uint16_t*>(value)); }
	auto uint16_from_str(std::uint16_t *v, std::string_view str)->void { *reinterpret_cast<std::uint16_t*>(v) = (std::uint16_t)std::strtoull(str.data(), nullptr, 0); }
	auto uint8_to_str(std::uint8_t* value)->std::string { return std::to_string(*reinterpret_cast<std::uint8_t*>(value)); }
	auto uint8_from_str(std::uint8_t *v, std::string_view str)->void { *reinterpret_cast<std::uint8_t*>(v) = (std::uint8_t)std::strtoull(str.data(), nullptr, 0); }
	auto int64_to_str(std::int64_t* value)->std::string { return std::to_string(*reinterpret_cast<std::int64_t*>(value)); }
	auto int64_from_str(std::int64_t *v, std::string_view str)->void { *reinterpret_cast<std::int64_t*>(v) = std::strtoll(str.data(), nullptr, 0); }
	auto int32_to_str(std::int32_t* value)->std::string { return std::to_string(*reinterpret_cast<std::int32_t*>(value)); }
	auto int32_from_str(std::int32_t *v, std::string_view str)->void { *reinterpret_cast<std::int32_t*>(v) = (std::int32_t)std::strtoll(str.data(), nullptr, 0); }
	auto int16_to_str(std::int16_t* value)->std::string { return std::to_string(*reinterpret_cast<std::int16_t*>(value)); }
	auto int16_from_str(std::int16_t *v, std::string_view str)->void { *reinterpret_cast<std::int16_t*>(v) = (std::int16_t)std::strtoll(str.data(), nullptr, 0); }
	auto int8_to_str(std::int8_t* value)->std::string { return std::to_string(*reinterpret_cast<std::int8_t*>(value)); }
	auto int8_from_str(std::int8_t *v, std::string_view str)->void { *reinterpret_cast<std::int8_t*>(v) = (std::int8_t)std::strtoll(str.data(), nullptr, 0); }

	ARIS_REGISTRATION
	{
		aris::core::class_<char>("char")
			.textMethod([](char *v)->std::string
			{
				return std::string(1, *v);
			}, [](char *v, std::string_view str)->void
			{
				*v = str[0];
			})
			;
		
		aris::core::class_<std::int8_t>("int8")
			.textMethod(int8_to_str, int8_from_str)
			;

		aris::core::class_<std::int16_t>("int16")
			.textMethod(int16_to_str, int16_from_str);

		aris::core::class_<std::int32_t>("int32")
			.textMethod(int32_to_str, int32_from_str)
			.alias("int");

		aris::core::class_<std::int64_t>("int64")
			.textMethod(int64_to_str, int64_from_str);

		aris::core::class_<std::uint8_t>("uint8")
			.textMethod(uint8_to_str, uint8_from_str);

		aris::core::class_<std::uint16_t>("uint16")
			.textMethod(uint16_to_str, uint16_from_str);

		aris::core::class_<std::uint32_t>("uint32")
			.textMethod(uint32_to_str, uint32_from_str)
			.alias("uint");

		aris::core::class_<std::uint64_t>("uint64")
			.textMethod(uint64_to_str, uint64_from_str);

		aris::core::class_<float>("float")
			.textMethod([](float *v)->std::string
			{
				return std::to_string(*reinterpret_cast<float*>(v));
			}, [](float *v, std::string_view str)->void
			{
				float result = std::stod(std::string(str));
				*reinterpret_cast<float*>(v) = result;
			});

		aris::core::class_<double>("double")
			.textMethod([](void *v)->std::string
			{
				char buf[100]{0};
				std::sprintf(buf, "%.17g", *reinterpret_cast<double*>(v));
				return buf;
			}, [](void *v, std::string_view str)->void
			{
				double result = std::stod(std::string(str));
				*reinterpret_cast<double*>(v) = result;
			});

		aris::core::class_<bool>("bool")
			.textMethod([](void *v)->std::string
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
			.textMethod([](void *v)->std::string
			{
				return *reinterpret_cast<std::string*>(v);
			}, [](void *v, std::string_view str)->void
			{
				*reinterpret_cast<std::string*>(v) = str;
			});
	}
}