#ifndef ARIS_CORE_REFLECTION_H_
#define ARIS_CORE_REFLECTION_H_

#include <any>
#include <map>
#include <vector>
#include <memory>
#include <functional>
#include <string_view>

#include <aris/core/basic_type.hpp>
#include <aris/core/tinyxml2.h>
#include <aris/core/log.hpp>

#define ARIS_REFLECT_CAT(a, b) a##b

#define ARIS_REGISTRATION                                                                 \
static void aris_reflection_register_function_();                                         \
namespace                                                                                 \
{                                                                                         \
    struct aris_reflection_help_class__                                                   \
    {                                                                                     \
        aris_reflection_help_class__()                                                    \
        {                                                                                 \
            aris_reflection_register_function_();                                         \
        }                                                                                 \
    };                                                                                    \
}                                                                                         \
static const aris_reflection_help_class__ ARIS_REFLECT_CAT(auto_register__, __LINE__);    \
static void aris_reflection_register_function_()

namespace aris::core
{
	class Type;
	class Instance;
	class Variant;

	class Property
	{
	public:
		auto name()->std::string { return name_; };
		auto set(Instance *, std::any arg)->void;
		auto get(Instance *)->Instance;
		auto set(Variant *, std::any arg)->void;
		auto get(Variant *)->Variant;
		Property(std::string_view name, Type *type_belong_to, const std::type_info*self_type)
			:name_(name), type_belong_to_(type_belong_to), type_(self_type) {}

	private:
		std::string name_;
		std::function<std::any(void*)> get_;
		std::function<void(void*, const std::any &)> set_;
		Type *type_belong_to_;// which property belong to
		const std::type_info* type_;// cpp type of this property
		template<typename T> friend class class_;
		friend class Variant;
		friend class Instance;
	};
	class Type
	{
	public:
		auto name()->std::string_view { return name_; };
		auto properties()const->const std::map<std::string, Property, std::less<>>& { return properties_; };
		Type(std::string_view name, std::function<void*(std::any*)> any_to_void) :name_(name), any_to_void_(any_to_void) {}

	private:
		std::string name_;
		std::map<std::string, Property, std::less<>> properties_;
		bool is_basic_;
		std::function<std::string(void*)> to_string_;
		std::function<void(void*, std::string_view)> from_string_;
		std::function<void*(std::any*)> any_to_void_; // for variant cast to void*

		template<typename T> friend class class_;
		friend class Variant;
		friend class Instance;
	};
	// variant 负责所存储对象的生命周期，而Instance不用
	class Variant
	{
	public:
		template<typename T>
		auto to()->T& { return std::any_cast<T&>(value_); }
		auto set(std::string_view prop_name, std::any arg)->void;
		auto get(std::string_view prop_name)->Variant;
		auto type()->const Type*;
		auto basic()->bool;
		auto toString()->std::string;
		auto fromString(std::string_view str)->void;

		Variant(const std::any &value);

	private:
		std::any value_;
		friend class Property;
	};
	
	auto reflect_types()->std::map<std::size_t, Type>&;
	auto reflect_names()->std::map<std::string_view, std::size_t>&;
	
	class Instance
	{
	public:
		template<typename T>
		auto to()->T& { return std::any_cast<T&>(value_); }
		auto set(std::string_view prop_name, std::any arg)->void;
		auto get(std::string_view prop_name)->Instance;
		auto isReference()->bool;
		auto type()->const Type*;
		auto basic()->bool;
		auto toString()->std::string;
		auto fromString(std::string_view str)->void;

		template<typename T>
		Instance(T && t, std::enable_if_t<!std::is_same_v<T, Instance>> *s = nullptr, std::enable_if_t<std::is_polymorphic_v<T>> *a = nullptr)
		{
			if (auto found = reflect_types().find(typeid(t).hash_code()); found != reflect_types().end())
				type_ = &typeid(t);
			else if (found = reflect_types().find(typeid(T).hash_code()); found != reflect_types().end())
				type_ = &typeid(T);
			else
				type_ = &typeid(t);

			value_ = &t;
		}
		template<typename T>
		Instance(T && t, std::enable_if_t<!std::is_same_v<T, Instance>> *s = nullptr, std::enable_if_t<!std::is_polymorphic<T>::value> *a = nullptr)
		{
			type_ = &typeid(t);
			value_ = &t;
		}
		Instance(const Instance&) = default;
		Instance(Instance&&) = default;

	private:
		auto toVoid()->void*;
		std::any value_;
		const std::type_info* type_;
		friend class Property;
	};
	
	auto inline getType(std::string_view name)->Type& 
	{
		return reflect_types().at(reflect_names().at(name));
	}
	template<typename Class_Type>
	class class_
	{
	public:
		class_(std::string_view name)
		{
			std::cout << name << std::endl;
			
			//Type type(name);
			auto hash_code = typeid(Class_Type).hash_code();
			auto[ins, ok] = reflect_types().emplace(std::make_pair(hash_code, Type(name, [](std::any* input)->void* 
			{
				return reinterpret_cast<void*>(std::any_cast<Class_Type>(input));
			})));
			reflect_names().emplace(std::make_pair(ins->second.name(), hash_code));
		}

		auto basic(std::function<std::string(void*)> to_string, std::function<void(void*, std::string_view)> from_string)->class_<Class_Type>&
		{
			



			return *this;
		}

		template<typename ...Args>
		auto constructor()
		{
			std::function<void(Args...)> f = [](Args ...args)
			{
				new Class_Type(args...);
			};
			
			return *this;
		}

		// espect: Class_Type::v where v is a value
		template<typename Value>
		auto property(std::string_view name, Value v
			, std::enable_if_t<std::is_lvalue_reference_v<decltype((new Class_Type)->*v)>> *is_value = nullptr)
			->class_<Class_Type>&
		{
			using T = std::remove_reference_t<decltype((new Class_Type)->*v)>;
			
			auto &type = reflect_types().at(typeid(Class_Type).hash_code());
			auto prop = Property(name, &type, &typeid(T));
			prop.get_ = [v](void* ins)->std::any{ return reinterpret_cast<Class_Type*>(ins)->*v; };
			prop.set_ = [v](void* ins, const std::any &value){reinterpret_cast<Class_Type*>(ins)->*v = std::any_cast<const T&>(value);};
			auto [iter, ok] = type.properties_.emplace(std::make_pair(prop.name(), prop));
			
			return *this;
		}

		// espect: Class_Type::v() where v is a reference function
		template<typename Value>
		auto property(std::string_view name, Value v
			, std::enable_if_t<std::is_function_v<Value> && std::is_lvalue_reference_v<decltype((new Class_Type)->*v)()>> *is_lf_returned_function = nullptr)
			->class_<Class_Type>&
		{
			//using T = std::remove_reference_t<decltype((new Class_Type)->*v)>;

			//auto &type = reflect_types().at(typeid(Class_Type).hash_code());
			//auto prop = Property(name, &type, &typeid(T));
			//prop.get_ = [v](void* ins)->std::any { return reinterpret_cast<Class_Type*>(ins)->*v; };
			//prop.set_ = [v](void* ins, const std::any &value) {reinterpret_cast<Class_Type*>(ins)->*v = std::any_cast<const T&>(value); };
			//auto[iter, ok] = type.properties_.emplace(std::make_pair(prop.name(), prop));

			return *this;
		}

		// espect: Class_Type::setProp(T v)->void
		//         Class_Type::getProp()->T&
		template<typename SetFunc, typename GetFunc>
		auto property(std::string_view name, SetFunc s, GetFunc g
			, std::enable_if_t<std::is_lvalue_reference_v<decltype(((new Class_Type)->*g)())>> *is_lf_returned_get = nullptr)
			->class_<Class_Type>&
		{
			using T = std::remove_reference_t<decltype(((new Class_Type)->*g)())>;
			
			auto &type = reflect_types().at(typeid(Class_Type).hash_code());
			auto prop = Property(name, &type, &typeid(T));
			prop.get_ = [g](void* ins)->std::any {	return (reinterpret_cast<Class_Type*>(ins)->*g)(); };
			prop.set_ = [s](void* ins, const std::any &value) {(reinterpret_cast<Class_Type*>(ins)->*s)(std::any_cast<const T&>(value)); };
			auto[iter, ok] = type.properties_.emplace(std::make_pair(prop.name(), prop));

			return *this;
		}

		// espect: Class_Type::setProp(T v)->void
		//         Class_Type::getProp()->T
		template<typename SetFunc, typename GetFunc>
		auto property(std::string_view name, SetFunc s, GetFunc g
			, std::enable_if_t<!std::is_lvalue_reference_v<decltype(((new Class_Type)->*g)())>> *is_lf_returned_get = nullptr)
			->class_<Class_Type>&
		{
			using T = std::remove_reference_t<decltype(((new Class_Type)->*g)())>;

			auto &type = reflect_types().at(typeid(Class_Type).hash_code());
			auto prop = Property(name, &type, &typeid(T));
			prop.get_ = [g](void* ins)->std::any {	return (reinterpret_cast<Class_Type*>(ins)->*g)(); };
			prop.set_ = [s](void* ins, const std::any &value) {(reinterpret_cast<Class_Type*>(ins)->*s)(std::any_cast<const T&>(value));	};
			auto[iter, ok] = type.properties_.emplace(std::make_pair(prop.name(), prop));

			return *this;
		}
	};

}

#endif
