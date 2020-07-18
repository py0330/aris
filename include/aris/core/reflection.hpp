#ifndef ARIS_CORE_REFLECTION_H_
#define ARIS_CORE_REFLECTION_H_

#ifdef aris_lib_EXPORTS
	#define ARIS_EXTERN
#else
	#define ARIS_EXTERN extern
#endif

//#ifdef ARIS_CORE_REFLECTION_CPP_
//	#define ARIS_EXTERN
//#else
//	#define ARIS_EXTERN extern
//#endif

#include <any>
#include <map>
#include <vector>
#include <memory>
#include <functional>
#include <string_view>
#include <variant>


#include <aris_lib_export.h>
#include <aris/core/object.hpp>
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
	class Property;
	class Instance;

	class ARIS_API Property
	{
	public:
		auto name()const->std::string;
		auto set(Instance *obj, Instance value)const->void;
		auto get(Instance *obj)const->Instance;
		auto type()->const Type*;
		auto acceptPtr()const->bool;
		~Property();
		Property(std::string_view name, Type *type_belong_to, const std::type_info *type_self, bool accept_ptr, std::function<void(Instance *, Instance)>, std::function<Instance(Instance *)>);

	private:
		auto setToText(std::function<std::string(void*)>)->void;
		auto setFromText(std::function<void(void*, std::string_view)>)->void;

		template<typename T> friend class class_;
		friend class Instance;
		friend class Type;

		struct Imp;
		std::shared_ptr<Imp> imp_;
	};
	class ARIS_API Type
	{
	public:
		static auto isBaseOf(const Type* base, const Type* derived)->bool;
		static auto getType(std::string_view name)->Type*;

		auto create()const->std::tuple<std::unique_ptr<void, void(*)(void const*)>, Instance>;
		auto name()const->std::string_view;
		
		auto properties()const->const std::vector<Property*>&;
		auto propertyAt(std::string_view name)const->Property*;
		auto inheritTypes()const->std::vector<const Type*>;
		auto isRefArray()const->bool;
		auto isArray()const->bool;
		auto isBasic()const->bool;
		~Type();

	private:
		using DefaultCtor = std::function<std::tuple<std::unique_ptr<void, void(*)(void const*)>, Instance>()>;
		using CastFunc = std::function<void*(void*)>;

		Type(std::string_view name);
		auto this_properties()->std::vector<Property>&;
		auto init()const->void;
		auto inherit(const std::type_info *inherit_type_info, CastFunc cast_to_inherit)->void;
		auto text(std::function<std::string(Instance*)> to_string, std::function<void(Instance*, std::string_view)> from_string)->void;
		auto as_array(bool,std::function<std::size_t(Instance*)>,std::function<Instance(Instance*, std::size_t)>, std::function<void(Instance*, const Instance&)>,std::function<void(Instance*)> )->void;

		static auto registerType(bool is_polymophic, std::size_t hash_code, std::string_view name, DefaultCtor ctor)->Type*;
		static auto reflect_types()->std::map<std::size_t, Type>&;
		static auto reflect_names()->std::map<std::string, std::size_t>&;
		static auto alias_impl(Type*, std::string_view alias_name)->void;
		static auto initAllTypes()->void;
		
		template<typename T> friend class class_;
		friend class Instance;
		friend class Property;

		struct Imp;
		std::shared_ptr<Imp> imp_;
	};
	class ARIS_API Instance
	{
	public:
		auto isEmpty()const->bool;
		auto isReference()const->bool;
		auto isBasic()const->bool;
		auto isArray()const->bool;
		auto type()const->const Type*;

		template<typename T>
		auto castTo()const->T*
		{
			auto type = &Type::reflect_types().at(typeid(T).hash_code());
			return reinterpret_cast<T*>(castToType(type));
		}
		
		// only work for non-basic and non-array //
		auto set(std::string_view prop_name, Instance arg)->void;
		auto get(std::string_view prop_name)->Instance;
		
		// only work for basic //
		auto toString()->std::string;
		auto fromString(std::string_view str)->void;

		// only work for array //
		auto size()->std::size_t;
		auto at(std::size_t id)->Instance;
		auto push_back(Instance element)->void;
		auto clear()->void;

		// 绑定到左值引用, 多态类型 //
		template<typename T>
		Instance(T && t, std::enable_if_t<(!std::is_same_v<std::decay_t<T>, Instance>) && std::is_lvalue_reference_v<T &&> && std::is_polymorphic_v<std::decay_t<T>>> *s = nullptr)
			:Instance(&typeid(t), dynamic_cast<void*>(&const_cast<std::decay_t<T> &>(t)))
		{
			static_assert(!std::is_const_v<T>, "instance must bind to a non-const value");
			if (type() == nullptr) THROW_FILE_LINE("Unrecognized type : " + typeid(t).name());
		}

		// 绑定到左值引用，非多态类型 //
		template<typename T>
		Instance(T && t, std::enable_if_t<(!std::is_same_v<std::decay_t<T>, Instance>) && std::is_lvalue_reference_v<T &&> && !std::is_polymorphic_v<std::decay_t<T>>> *s = nullptr)
			:Instance(&typeid(t), reinterpret_cast<void*>(&const_cast<std::decay_t<T> &>(t)))
		{
			static_assert(!std::is_const_v<T>, "instance must bind to a non-const value");
			if (type() == nullptr) THROW_FILE_LINE("Unrecognized type : " + typeid(t).name());
		}
		
		// 根据右值来构造 //
		template<typename T>
		Instance(T && t, std::enable_if_t<(!std::is_same_v<std::decay_t<T>, Instance>) && std::is_rvalue_reference_v<T &&>> *s = nullptr)
			:Instance(&typeid(t), std::make_shared<std::decay_t<T>>(std::move(t)))
		{
			static_assert(std::is_copy_constructible_v<T> || std::is_move_constructible_v<T>, "Failed to construct : NO ctors");
			if (type() == nullptr) THROW_FILE_LINE("Unrecognized type : " + typeid(t).name());
		}

		~Instance();
		Instance();
		Instance(const Instance&);
		Instance(Instance&&);

	private:
		auto castToType(const Type* t)const->void*;
		auto toVoidPtr()const->void*;

		Instance(const std::type_info *info, std::shared_ptr<void> data);
		Instance(const std::type_info *info, void* data);

		struct Imp;
		aris::core::ImpPtr<Imp> imp_;

		friend class Property;
		template<typename Class_Type> friend class class_;
	};

	template<typename Class_Type>
	class class_
	{
	public:
		// 普通类型 //
		template <typename T = Class_Type>
		class_(std::string_view name, std::enable_if_t<!std::is_abstract_v<T> && std::is_default_constructible_v<T> && std::is_destructible_v<T>> *test = nullptr)
		{
			type_ = Type::registerType(std::is_polymorphic_v<Class_Type>, typeid(Class_Type).hash_code(), name, []()->std::tuple<std::unique_ptr<void, void(*)(void const*)>, Instance>
			{
				auto f = [](void const * data)->void{	delete static_cast<Class_Type const*>(data); };
				auto ptr = std::unique_ptr<void, void(*)(void const*)>(new Class_Type, f);
				Instance ins(*reinterpret_cast<Class_Type*>(ptr.get()));
				return std::tuple<std::unique_ptr<void, void(*)(void const*)>, Instance>(std::move(ptr), ins);
			});
		}

		// 纯虚类型 //
		template <typename T = Class_Type>
		class_(std::string_view name, std::enable_if_t<std::is_abstract_v<T> || !std::is_default_constructible_v<T> || !std::is_destructible_v<T>> *test = nullptr)
		{
			type_ = Type::registerType(std::is_polymorphic_v<Class_Type>, typeid(Class_Type).hash_code(), name, nullptr);
		}

		// 别名 //
		auto alias(std::string_view name) { Type::alias_impl(type_, name); }

		template<typename FatherType>
		auto inherit()->class_<Class_Type>&
		{
			if (!std::is_base_of_v<FatherType, Class_Type>)std::cout << (std::string("failed to inherit \"") + typeid(FatherType).name() + "\" for \"" + typeid(Class_Type).name() + "\"") << std::endl;
			if (!std::is_base_of_v<FatherType, Class_Type>)THROW_FILE_LINE(std::string("failed to inherit \"") + typeid(FatherType).name() + "\" for \"" + typeid(Class_Type).name() + "\"");

			type_->inherit(&typeid(FatherType),[](void* input)->void*{return dynamic_cast<FatherType*>(reinterpret_cast<Class_Type*>(input));});// 多继承可能改变指针所指向的位置，坑爹！！//

			return *this;
		};

		auto textMethod(std::function<std::string(Class_Type*)> to_string, std::function<void(Class_Type*, std::string_view)> from_string)
		{
			type_->text([=](Instance* ins)->std::string	{return to_string(ins->castTo<Class_Type>());},
						[=](Instance* ins, std::string_view str){from_string(ins->castTo<Class_Type>(), str);});
			return *this;
		}

		// espect: Class_Type::at(size_t id)->T or Class_Type::at(size_t id)->T&  
		//         Class_Type::push_back(T*)->void
		//         Class_Type::size()->size_t
		template<typename T = Class_Type>
		auto asRefArray()->std::enable_if_t<std::is_object_v<T>, class_<Class_Type>&>
		{
			auto size_func = [](Instance* ins)->std::size_t	{return ins->castTo<Class_Type>()->size();	};
			auto at_func = [](Instance* ins, std::size_t id)->Instance{	return ins->castTo<Class_Type>()->at(id);};
			auto push_back_func = [](Instance* ins, const Instance& value)->void{ins->castTo<Class_Type>()->push_back(value.castTo<Class_Type::value_type>());	};
			auto clear_func = [](Instance* ins)->void{	ins->castTo<Class_Type>()->clear();	};
			type_->as_array(true, size_func, at_func, push_back_func, clear_func);
			return *this;
		}

		// espect: Class_Type::at(size_t id)->T or Class_Type::at(size_t id)->T&  
		//         Class_Type::push_back(T)->void
		//         Class_Type::size()->size_t    
		template<typename T = Class_Type>
		auto asArray()->std::enable_if_t<std::is_object_v<T>, class_<Class_Type>&>
		{
			auto size_func = [](Instance* ins)->std::size_t	{	return ins->castTo<Class_Type>()->size();};
			auto at_func = [](Instance* ins, std::size_t id)->Instance	{	return ins->castTo<Class_Type>()->at(id);};
			auto push_back_func = [](Instance* ins, const Instance& value)->void{ins->castTo<Class_Type>()->push_back(*value.castTo<Class_Type::value_type>());	};
			auto clear_func = [](Instance* ins)->void{	ins->castTo<Class_Type>()->clear();	};
			type_->as_array(false, size_func, at_func, push_back_func, clear_func);
			return *this;
		}

		// espect: Class_Type::v where v is a value
		template<typename Value>
		auto prop(std::string_view name, Value v)->std::enable_if_t<
			std::is_member_object_pointer_v<Value>
			&& std::is_lvalue_reference_v<decltype((new Class_Type)->*v)>
			, class_<Class_Type>&>
		{
			using T = std::decay_t<decltype((new Class_Type)->*v)>;

			auto get = [v](Instance *obj)->Instance { return obj->castTo<Class_Type>()->*v; };
			auto set = [v](Instance *obj, Instance value) { obj->castTo<Class_Type>()->*v = *value.castTo<T>(); };
			auto &prop = type_->this_properties().emplace_back(name, type_, &typeid(T), false, set, get);

			return *this;
		}

		// espect: Class_Type::v()->T& where v is a reference function
		template<typename Value>
		auto prop(std::string_view name, Value v)->std::enable_if_t<
			std::is_member_function_pointer_v<Value>
			&& std::is_lvalue_reference_v<decltype(((new Class_Type)->*v)())>
			&& !std::is_const_v<decltype(((new Class_Type)->*v)())>
			, class_<Class_Type>&>
		{
			using T = std::decay_t<decltype(((new Class_Type)->*v)())>;

			auto get = [v](Instance *obj)->Instance { return (obj->castTo<Class_Type>()->*v)(); };
			auto set = [v](Instance *obj, Instance value) { (obj->castTo<Class_Type>()->*v)() = *value.castTo<T>(); };
			auto &prop = type_->this_properties().emplace_back(name, type_, &typeid(T), false, set, get);

			return *this;
		}

		// espect: Class_Type::setProp(T v)->void  
		//         Class_Type::getProp()->T
		template<typename SetFunc, typename GetFunc, typename C = Class_Type>
		auto prop(std::string_view name, SetFunc s, GetFunc g)->std::enable_if_t<
			std::is_class_v<C>
			&& std::is_member_function_pointer_v<SetFunc>
			&& std::is_member_function_pointer_v<GetFunc>
			&& (!std::is_same_v<SetFunc, void (C::*)(std::decay_t<decltype((((C*)(nullptr))->*g)())>*) >)
			, class_<Class_Type>& >
		{
			using T = std::decay_t<decltype(((C*)(nullptr)->*g)())>;

			auto get = [g](Instance *obj)->Instance {	return (obj->castTo<Class_Type>()->*g)(); };
			auto set = [s](Instance *obj, Instance value) { (obj->castTo<Class_Type>()->*s)(*value.castTo<T>()); };
			auto &prop = type_->this_properties().emplace_back(name, type_, &typeid(T), false, set, get);

			return *this;
		}

		// espect: Class_Type::setProp(T *v)->void  
		//         Class_Type::getProp()->T
		//
		// note  : set function will responsible for life-time management of v
		template<typename SetFunc, typename GetFunc, typename C = Class_Type>
		auto prop(std::string_view name, SetFunc s, GetFunc g)->std::enable_if_t<
			std::is_class_v<C>
			&& std::is_member_function_pointer_v<SetFunc>
			&& std::is_member_function_pointer_v<GetFunc>
			&& std::is_same_v<SetFunc, void (C::*)(std::decay_t<decltype((((C*)(nullptr))->*g)())>*) >
			, class_<Class_Type>& >
		{
			using T = std::decay_t<decltype((((C*)(nullptr))->*g)())>;

			auto get = [g](Instance *obj)->Instance {	return (obj->castTo<Class_Type>()->*g)();	};
			auto set = [s](Instance *obj, Instance value) {(obj->castTo<Class_Type>()->*s)(value.castTo<T>()); };
			auto &prop = type_->this_properties().emplace_back(name, type_, &typeid(T), true, set, get);

			return *this;
		}

		// espect: setProp(C *obj, T v)->void  
		//         getProp(C *obj)->T
		template<typename SetFunc, typename GetFunc, typename C = Class_Type>
		auto prop(std::string_view name, SetFunc s, GetFunc g)->std::enable_if_t<
			std::is_class_v<C>
			&& !std::is_member_function_pointer_v<SetFunc>
			&& !std::is_member_function_pointer_v<GetFunc>
			&& !std::is_same_v<SetFunc, void(*)(C*, std::decay_t<decltype((*g)((C*)(nullptr)))>*) >
			, class_<Class_Type>& >
		{
			using T = std::decay_t<decltype((*g)((C*)(nullptr)))>;

			auto get = [g](Instance *obj)->Instance {	return (*g)(obj->castTo<C>());	};
			auto set = [s](Instance *obj, Instance value) {(*s)(obj->castTo<C>(), *value.castTo<T>()); };
			auto &prop = type_->this_properties().emplace_back(name, type_, &typeid(T), false, set, get);

			return *this;
		}
		auto propertyToStrMethod(std::string_view prop_name, std::function<std::string(void* value)> func)
		{
			auto found = std::find_if(type_->this_properties().begin(), type_->this_properties().end(), [prop_name](Property&prop) {return prop.name() == prop_name; });
			found->setToText(func);
			return *this;
		}

		auto propertyFromStrMethod(std::string_view prop_name, std::function<void(void* value, std::string_view str)> func)
		{
			auto found = std::find_if(type_->this_properties().begin(), type_->this_properties().end(), [prop_name](Property&prop) {return prop.name() == prop_name; });
			found->setFromText(func);
			return *this;
		}

	private:
		Type * type_;
	};

	auto inline charToStr(void* value)->std::string
	{
		std::string ret;
		if(*reinterpret_cast<char*>(value) != '\0')ret.push_back(*reinterpret_cast<char*>(value));
		return ret;
	}
	auto inline strToChar(void* value, std::string_view str)->void
	{
		if (str.empty())*reinterpret_cast<char*>(value) = 0;
		else *reinterpret_cast<char*>(value) = str[0];
	}
}

#endif
