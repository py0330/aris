#ifndef ARIS_CORE_REFLECTION_H_
#define ARIS_CORE_REFLECTION_H_

#include <any>
#include <map>
#include <vector>
#include <memory>
#include <functional>
#include <string_view>
#include <variant>


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

extern int aaaaa_bbbbb_ccccc;

namespace aris::core
{
	class Type;
	class Property;
	class Instance;

	class Property
	{
	public:
		auto name()const->std::string { return name_; };
		auto set(Instance *obj, Instance value)const->void;
		auto get(Instance *obj)const->Instance;
		auto type()->const Type*;
		auto acceptPtr()const->bool;
		Property(std::string_view name, Type *type_belong_to):name_(name), type_belong_to_(type_belong_to) {}

	private:
		std::string name_;
		std::function<Instance(Instance *obj)> get_;
		std::function<void(Instance *obj, Instance prop)> set_;
		bool accept_ptr_{ false };// which means prop is a ptr, to support polymorphim
		Type *type_belong_to_;// obj type, which prop belong to
		Type *type_;// type of this prop
		const std::type_info *type_info_;// type info of this prop

		// only for basic type //
		std::function<std::string(void* value)> to_str_func_;
		std::function<void(void* value, std::string_view str)> from_str_func_;

		template<typename T> friend class class_;
		friend class Instance;
	};
	class Type
	{
	public:
		static auto isBaseOf(const Type* base, const Type* derived)->bool;

		auto create()const->std::tuple<std::unique_ptr<void, void(*)(void const*)>, Instance>;
		auto name()const->std::string_view { return type_name_; };
		auto properties()const->const std::vector<Property*>&;
		auto propertyAt(std::string_view name)const->Property*;
		auto inheritTypes()const->std::vector<const Type*>;
		auto isRefArray()const->bool { return array_data_.get() && array_data_->is_ref_array_; }
		auto isArray()const->bool { return array_data_.get(); }
		auto isBasic()const->bool { return (!is_polymophic_) && (!isArray()) && properties_ptr_.empty(); }
		Type(std::string_view name) :type_name_(name) {}

	private:
		auto init()const->void;
		bool inited{ false };  // init properties, for inheritance

		// properties //
		std::string type_name_;
		std::vector<Property> this_properties_;// 不包含父类properties
		std::vector<Property*> properties_ptr_;// 包含父类的properties，为了保持插入顺序，所以不得不用vector替代map

		// inherit types //
		std::vector<const std::type_info *> inherit_type_infos_;// 因为在注册过程中，无法确保所继承的类已经注册完毕，所以需要后续生成该向量
		std::vector<const Type *> inherit_types_;
		std::vector<std::function<void*(void*)>> inherit_cast_vec_; // 将自己cast成基类的函数

		// ctor //
		using DefaultCtor = std::function<std::tuple<std::unique_ptr<void, void(*)(void const*)>, Instance>()>;
		DefaultCtor default_ctor_;

		// to text // 
		std::function<std::string(Instance*)> to_string_;
		std::function<void(Instance*, std::string_view)> from_string_;

		// array //
		struct ArrayData
		{
			// for basic //
			bool is_ref_array_;
			const Type* array_type_{ nullptr };
			std::function<std::size_t(Instance*)> size_func_;
			std::function<Instance(Instance*, std::size_t id)> at_func_;
			std::function<void(Instance*, const Instance&)> push_back_func_;
			std::function<void(Instance*)> clear_func_;
		};

		std::unique_ptr<ArrayData> array_data_;
		bool is_polymophic_{ false };

		friend auto initAllTypes()->void;
		friend auto registerType(std::size_t hash_code, std::string_view name, DefaultCtor ctor)->Type*;
		template<typename T> friend class class_;
		friend class Instance;
	};
	class Instance
	{
	public:
		auto isEmpty()const->bool { return data_.index() == 0; }
		auto isReference()const->bool;
		auto isBasic()const->bool;
		auto isArray()const->bool;
		auto type()const->const Type*;

		template<typename T>
		auto castTo()const->T*
		{
			auto type = &reflect_types().at(typeid(T).hash_code());
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
		{
			static_assert(!std::is_const_v<T>, "instance must bind to a non-const value");
			auto &r = const_cast<std::decay_t<T> &>(t);
			data_ = InstanceRef{ dynamic_cast<void*>(&r) };// dynamic_cast to most derived class
			type_info_ = &typeid(r);

			if (type() == nullptr) THROW_FILE_LINE("Unrecognized type : " + type_info_->name());
		}

		// 绑定到左值引用，非多态类型 //
		template<typename T>
		Instance(T && t, std::enable_if_t<(!std::is_same_v<std::decay_t<T>, Instance>) && std::is_lvalue_reference_v<T &&> && !std::is_polymorphic_v<std::decay_t<T>>> *s = nullptr)
		{
			static_assert(!std::is_const_v<T>, "instance must bind to a non-const value");
			auto &r = const_cast<std::decay_t<T> &>(t);
			data_ = InstanceRef{ &r };
			type_info_ = &typeid(r);

			if (type() == nullptr) THROW_FILE_LINE("Unrecognized type : " + type_info_->name());
		}
		
		// 根据右值来构造 //
		template<typename T>
		Instance(T && t, std::enable_if_t<(!std::is_same_v<std::decay_t<T>, Instance>) && std::is_rvalue_reference_v<T &&>> *s = nullptr)
		{
			static_assert(std::is_copy_constructible_v<T> || std::is_move_constructible_v<T>, "Failed to construct : NO ctors");
			auto &r = const_cast<std::decay_t<T> &>(t);
			data_ = InstancePtr{ std::make_shared<std::decay_t<T>>(std::move(r)) };
			type_info_ = &typeid(r);

			if (type() == nullptr) THROW_FILE_LINE("Unrecognized type : " + type_info_->name());
		}
		Instance(const Instance&) = default;
		Instance(Instance&&) = default;

	private:
		auto castToType(const Type* t)const->void*;
		auto toVoidPtr()const->void*;
		const Property *belong_to_{ nullptr };
		const std::type_info *type_info_;

		struct InstanceRef{	void* data_; };
		struct InstancePtr{	std::shared_ptr<void> data_; };
		std::variant<std::monostate, InstanceRef, InstancePtr> data_;

		friend class Property;
		template<typename Class_Type> friend class class_;
	};

	auto reflect_types()->std::map<std::size_t, Type>&;
	auto reflect_names()->std::map<std::string, std::size_t>&;
	auto inline getType(std::string_view name)->Type*
	{
		auto found = reflect_names().find(std::string(name));
		return found == reflect_names().end() ? nullptr : &reflect_types().at(found->second);
	}

	auto registerType(std::size_t hash_code, std::string_view name, Type::DefaultCtor ctor)->Type*;
	auto alias_impl(Type*, std::string_view alias_name)->void;

	template<typename Class_Type>
	class class_
	{
	public:
		// 对于普通类型 //
		template <typename T = Class_Type>
		class_(std::string_view name, std::enable_if_t<!std::is_abstract_v<T> && std::is_default_constructible_v<T> && std::is_destructible_v<T>> *test = nullptr)
		{
			aaaaa_bbbbb_ccccc++;
			type_ = registerType(typeid(Class_Type).hash_code(), name, []()->std::tuple<std::unique_ptr<void, void(*)(void const*)>, Instance>
			{
				auto f = [](void const * data)->void{	delete static_cast<Class_Type const*>(data); };
				auto ptr = std::unique_ptr<void, void(*)(void const*)>(new Class_Type, f);
				Instance ins(*reinterpret_cast<Class_Type*>(ptr.get()));
				return std::tuple<std::unique_ptr<void, void(*)(void const*)>, Instance>(std::move(ptr), ins);
			});
			type_->is_polymophic_ = std::is_polymorphic_v<Class_Type>;
		}

		// 对于纯虚类型 //
		template <typename T = Class_Type>
		class_(std::string_view name, std::enable_if_t<std::is_abstract_v<T> || !std::is_default_constructible_v<T> || !std::is_destructible_v<T>> *test = nullptr)
		{
			aaaaa_bbbbb_ccccc++;
			type_ = registerType(typeid(Class_Type).hash_code(), name, nullptr);
			type_->is_polymophic_ = std::is_polymorphic_v<Class_Type>;
		}

		// 别名 //
		auto alias(std::string_view name)
		{
			alias_impl(type_, name);
		}

		template<typename FatherType>
		auto inherit()->class_<Class_Type>&
		{
			if (!std::is_base_of_v<FatherType, Class_Type>)std::cout << (std::string("failed to inherit \"") + typeid(FatherType).name() + "\" for \"" + typeid(Class_Type).name() + "\"") << std::endl;
			if (!std::is_base_of_v<FatherType, Class_Type>)THROW_FILE_LINE(std::string("failed to inherit \"") + typeid(FatherType).name() + "\" for \"" + typeid(Class_Type).name() + "\"");
			

			type_->inherit_type_infos_.push_back(&typeid(FatherType));
			type_->inherit_cast_vec_.push_back([](void* input)->void*
			{
				return dynamic_cast<FatherType*>(reinterpret_cast<Class_Type*>(input));
			});// 多继承可能改变指针所指向的位置，坑爹！！//

			return *this;
		};

		auto textMethod(std::function<std::string(Class_Type*)> to_string, std::function<void(Class_Type*, std::string_view)> from_string)
		{
			type_->to_string_ = [=](Instance* ins)->std::string
			{
				return to_string(ins->castTo<Class_Type>());
			};
			type_->from_string_ = [=](Instance* ins, std::string_view str)
			{
				from_string(ins->castTo<Class_Type>(), str);
			};
			return *this;
		}

		// espect: Class_Type::at(size_t id)->T or Class_Type::at(size_t id)->T&  
		//         Class_Type::push_back(T*)->void
		//         Class_Type::size()->size_t
		auto asRefArray()->class_<Class_Type>&
		{
			auto size_func = [](Instance* ins)->std::size_t
			{
				return ins->castTo<Class_Type>()->size();
			};
			auto at_func = [](Instance* ins, std::size_t id)->Instance
			{
				return ins->castTo<Class_Type>()->at(id);
			};
			auto push_back_func = [](Instance* ins, const Instance& value)->void
			{
				ins->castTo<Class_Type>()->push_back(value.castTo<Class_Type::value_type>());
			};
			auto clear_func = [](Instance* ins)->void
			{
				ins->castTo<Class_Type>()->clear();
			};

			type_->array_data_ = std::unique_ptr<Type::ArrayData>(new Type::ArrayData{ true, type_, size_func, at_func,push_back_func, clear_func });
			return *this;
		}

		// espect: Class_Type::at(size_t id)->T or Class_Type::at(size_t id)->T&  
		//         Class_Type::push_back(T)->void
		//         Class_Type::size()->size_t    
		auto asArray()->class_<Class_Type>&
		{
			auto size_func = [](Instance* ins)->std::size_t
			{
				return ins->castTo<Class_Type>()->size();
			};
			auto at_func = [](Instance* ins, std::size_t id)->Instance
			{
				return ins->castTo<Class_Type>()->at(id);
			};
			auto push_back_func = [](Instance* ins, const Instance& value)->void
			{
				ins->castTo<Class_Type>()->push_back(*value.castTo<Class_Type::value_type>());
			};
			auto clear_func = [](Instance* ins)->void
			{
				ins->castTo<Class_Type>()->clear();
			};

			type_->array_data_ = std::unique_ptr<Type::ArrayData>(new Type::ArrayData{ false, type_, size_func, at_func,push_back_func, clear_func });
			return *this;
		}

		// espect: Class_Type::v where v is a value
		template<typename Value>
		auto prop(std::string_view name, Value v)->std::enable_if_t<
				std::is_member_object_pointer_v<Value>
				&& std::is_lvalue_reference_v<decltype((reinterpret_cast<Class_Type*>(nullptr))->*v)>
			, class_<Class_Type>&>
		{
			using T = std::decay_t<decltype((reinterpret_cast<Class_Type*>(nullptr))->*v)>;

			auto &prop = type_->this_properties_.emplace_back(name, type_);
			prop.get_ = [v](Instance *obj)->Instance { return obj->castTo<Class_Type>()->*v; };
			prop.set_ = [v](Instance *obj, Instance value) { obj->castTo<Class_Type>()->*v = *value.castTo<T>(); };
			prop.type_info_ = &typeid(T);

			return *this;
		}

		// espect: Class_Type::v()->T& where v is a reference function
		template<typename Value>
		auto prop(std::string_view name, Value v)->std::enable_if_t<
				std::is_member_function_pointer_v<Value>
				&& std::is_lvalue_reference_v<decltype(((reinterpret_cast<Class_Type*>(nullptr))->*v)())>
				&& !std::is_const_v<decltype(((reinterpret_cast<Class_Type*>(nullptr))->*v)())>
			, class_<Class_Type>&>
		{
			using T = std::decay_t<decltype(((reinterpret_cast<Class_Type*>(nullptr))->*v)())>;

			auto &prop = type_->this_properties_.emplace_back(name, type_);
			prop.get_ = [v](Instance *obj)->Instance { return (obj->castTo<Class_Type>()->*v)(); };
			prop.set_ = [v](Instance *obj, Instance value) { (obj->castTo<Class_Type>()->*v)() = *value.castTo<T>(); };
			prop.type_info_ = &typeid(T);

			return *this;
		}

		// espect: Class_Type::setProp(T v)->void  
		//         Class_Type::getProp()->T
		template<typename SetFunc, typename GetFunc, typename C = Class_Type>
		auto prop(std::string_view name, SetFunc s, GetFunc g) -> std::enable_if_t< 
				std::is_class_v<C> 
				&& std::is_member_function_pointer_v<SetFunc>
				&& std::is_member_function_pointer_v<GetFunc>
				&& (!std::is_same_v<SetFunc, void (C::*)(std::decay_t<decltype(((reinterpret_cast<C*>(nullptr))->*g)())>*) >)
			, class_<Class_Type>& >
		{
			using T = std::decay_t<decltype(((reinterpret_cast<C*>(nullptr))->*g)())>;

			auto &prop = type_->this_properties_.emplace_back(name, type_);
			prop.get_ = [g](Instance *obj)->Instance{	return (obj->castTo<Class_Type>()->*g)(); };
			prop.set_ = [s](Instance *obj, Instance value){ (obj->castTo<Class_Type>()->*s)(*value.castTo<T>()); };
			prop.type_info_ = &typeid(T);

			return *this;
		}
		
		// espect: Class_Type::setProp(T *v)->void  
		//         Class_Type::getProp()->T
		//
		// note  : set function will responsible for life-time management of v
		template<typename SetFunc, typename GetFunc, typename C = Class_Type>
		auto prop(std::string_view name, SetFunc s, GetFunc g) -> std::enable_if_t< 
				std::is_class_v<C> 
				&& std::is_member_function_pointer_v<SetFunc>
				&& std::is_member_function_pointer_v<GetFunc>
				&& std::is_same_v<SetFunc, void (C::*)(std::decay_t<decltype(((reinterpret_cast<C*>(nullptr))->*g)())>*) >
			, class_<Class_Type>& >
		{
			using T = std::decay_t<decltype(((reinterpret_cast<C*>(nullptr))->*g)())>;

			auto &prop = type_->this_properties_.emplace_back(name, type_);
			prop.get_ = [g](Instance *obj)->Instance {	return (obj->castTo<Class_Type>()->*g)();	};
			prop.set_ = [s](Instance *obj, Instance value) {(obj->castTo<Class_Type>()->*s)(value.castTo<T>()); };
			prop.type_info_ = &typeid(T);
			prop.accept_ptr_ = true;

			return *this;
		}

		// espect: setProp(C *obj, T v)->void  
		//         getProp(C *obj)->T
		//
		// note  : set function will responsible for life-time management of v
		template<typename SetFunc, typename GetFunc, typename C = Class_Type>
		auto prop(std::string_view name, SetFunc s, GetFunc g)->std::enable_if_t<
				std::is_class_v<C>
				&& !std::is_member_function_pointer_v<SetFunc>
				&& !std::is_member_function_pointer_v<GetFunc>
				&& !std::is_same_v<SetFunc, void(*)(C*, std::decay_t<decltype((*g)(reinterpret_cast<C*>(nullptr)))>*) >
			, class_<Class_Type>& >
		{
			using T = std::decay_t<decltype((*g)(reinterpret_cast<C*>(nullptr)))>;

			auto &prop = type_->this_properties_.emplace_back(name, type_);
			prop.get_ = [g](Instance *obj)->Instance {	return (*g)(obj->castTo<C>());	};
			prop.set_ = [s](Instance *obj, Instance value) {(*s)(obj->castTo<C>(), *value.castTo<T>()); };
			prop.type_info_ = &typeid(T);

			return *this;
		}

		auto propertyToStrMethod(std::string_view prop_name, std::function<std::string(void* value)> func)
		{
			auto found = std::find_if(type_->this_properties_.begin(), type_->this_properties_.end(), [prop_name](Property&prop) {return prop.name() == prop_name; });
			auto &prop = *found;
			prop.to_str_func_ = func;
			return *this;
		}

		auto propertyFromStrMethod(std::string_view prop_name, std::function<void(void* value, std::string_view str)> func)
		{
			auto found = std::find_if(type_->this_properties_.begin(), type_->this_properties_.end(), [prop_name](Property&prop) {return prop.name() == prop_name; });
			auto &prop = *found;
			prop.from_str_func_ = func;
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
