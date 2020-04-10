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
	/*
	template<typename T>
	struct has_const_iterator
	{
	private:
		typedef char                      yes;
		typedef struct { char array[2]; } no;

		template<typename C> static yes test(typename C::const_iterator*);
		template<typename C> static no  test(...);
	public:
		static const bool value = sizeof(test<T>(0)) == sizeof(yes);
		typedef T type;
	};

	template <typename T>
	struct has_begin_end
	{
		template<typename C> static char(&f(typename std::enable_if<
			std::is_same<decltype(static_cast<typename C::const_iterator(C::*)() const>(&C::begin)),
			typename C::const_iterator(C::*)() const>::value, void>::type*))[1];

		template<typename C> static char(&f(...))[2];

		template<typename C> static char(&g(typename std::enable_if<
			std::is_same<decltype(static_cast<typename C::const_iterator(C::*)() const>(&C::end)),
			typename C::const_iterator(C::*)() const>::value, void>::type*))[1];

		template<typename C> static char(&g(...))[2];

		static bool const beg_value = sizeof(f<T>(0)) == 1;
		static bool const end_value = sizeof(g<T>(0)) == 1;
	};
	
	template<typename T>
	struct is_container : std::integral_constant<bool, has_const_iterator<T>::value && has_begin_end<T>::beg_value && has_begin_end<T>::end_value>{ };
	*/
	class Type;
	class Property;
	class Instance;

	class Property
	{
	public:
		auto name()const->std::string { return name_; };
		auto set(Instance *, const Instance&)const->void;
		auto get(Instance *)const->Instance;
		auto acceptPtr()const->bool;
		Property(std::string_view name, Type *type_belong_to)
			:name_(name), type_belong_to_(type_belong_to) {}

	private:
		std::string name_;
		std::function<Instance(void*)> get_;
		std::function<void(void*, const Instance &)> set_;
		bool accept_ptr_{ false };// which means property is a ptr, to support polymorphim
		Type *type_belong_to_;// which property belong to
		template<typename T> friend class class_;
		friend class Instance;
	};
	class Type
	{
	public:
		auto create()const->std::tuple<std::unique_ptr<void, void(*)(void const*)>, Instance>;
		auto name()const->std::string_view { return type_name_; };
		auto properties()const->const std::map<std::string, Property, std::less<>>&;
		auto inheritTypes()const->std::vector<const Type*>;
		auto isRefArray()const->bool { return is_ref_array_; }
		Type(std::string_view name, std::function<void*(std::any*)> any_to_void) :type_name_(name), any_to_void_(any_to_void) {}

	private:
		std::string type_name_;
		std::map<std::string, Property, std::less<>> this_properties_;// 因为在注册过程中，无法确保所继承的类已经注册完毕，所以需要后续生成该向量
		std::map<std::string, Property, std::less<>> properties_;
		bool is_basic_{ false };
		bool is_array_{ false };
		bool is_ref_array_{ false };
		std::function<void*(std::any*)> any_to_void_; // for any cast to void*
		std::vector<const std::type_info *> inherit_type_infos_;// 因为在注册过程中，无法确保所继承的类已经注册完毕，所以需要后续生成该向量
		std::vector<const Type *> inherit_types_;

		// ctor //
		std::function<std::tuple<std::unique_ptr<void, void(*)(void const*)>, Instance>()> default_ctor_;

		// for basic //
		std::function<std::string(void*)> to_string_;
		std::function<void(void*, std::string_view)> from_string_;
		
		// for array //
		std::function<std::size_t(void*)> size_func_;
		std::function<Instance(void*, std::size_t id)> at_func_;
		std::function<void(void*, const Instance&)> push_back_func_;

		template<typename T> friend class class_;
		friend class Variant;
		friend class Instance;
	};
	class Instance
	{
	public:
		auto isEmpty()->bool { return value_.has_value(); }
		auto isReference()->bool;
		auto isBasic()->bool;
		auto isArray()->bool;
		auto type()->const Type*;

		template<typename T>
		auto to()->T&
		{
			if (isReference())
			{
				auto ref = std::any_cast<InstanceRef>(&value_);
				if (typeid(T).hash_code() != ref->type_->hash_code()) THROW_FILE_LINE("invalid transfer");
				return *reinterpret_cast<T*>(ref->data_);
			}
			else
			{
				return std::any_cast<T&>(value_);
			}
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

		// 左值引用 //
		template<typename T>
		Instance(T && t, std::enable_if_t<!std::is_same_v<std::decay_t<T>, Instance>> *s = nullptr)
		{
			static_assert(!std::is_const_v<T>, "instance must bind to a non-const value");

			using RealType = std::decay_t<T>;
			auto &real_value = const_cast<RealType&>(t);
			
			// 左值 //
			if (std::is_lvalue_reference_v<decltype(t)>)
			{
				value_ = InstanceRef{ &real_value , &typeid(real_value) };
			}
			// 右值 //
			else
			{
				value_ = t;
			}
		}
		Instance(const Instance&) = default;
		Instance(Instance&&) = default;

	private:
		auto toVoidPtr()->void*;
		auto toVoidPtr()const->const void* { return const_cast<Instance*>(this)->toVoidPtr(); }
		std::any value_;
		friend class Property;
		template<typename Class_Type> friend class class_;

		struct InstanceRef
		{
			void* data_;
			const std::type_info *type_;
		};
	};

	template<typename Class_Type>
	class class_
	{
	public:
		class_(std::string_view name)
		{
			auto hash_code = typeid(Class_Type).hash_code();
			auto[ins, ok] = reflect_types().emplace(std::make_pair(hash_code, Type(name, [](std::any* input)->void* 
			{
				return reinterpret_cast<void*>(std::any_cast<Class_Type>(input));
			})));
			if (!ok) THROW_FILE_LINE("class already exist");
			auto[ins2, ok2] = reflect_names().emplace(std::make_pair(ins->second.name(), hash_code));
			if (!ok2) THROW_FILE_LINE("class name already exist");

			ins->second.is_basic_ = false;
			ins->second.default_ctor_ = []()->std::tuple<std::unique_ptr<void, void(*)(void const*)>, Instance>
			{
				auto f = [](void const * data){	delete static_cast<Class_Type const*>(data); };
				auto ptr = std::unique_ptr<void, void(*)(void const*)>(new Class_Type, f);
				Instance ins(*reinterpret_cast<Class_Type*>(ptr.get()));
				return std::tuple<std::unique_ptr<void, void(*)(void const*)>, Instance>(std::move(ptr), ins);
			};
		}

		template<typename FatherType>
		auto inherit()->class_<Class_Type>&
		{
			auto &type = reflect_types().at(typeid(Class_Type).hash_code());
			type.inherit_type_infos_.push_back(&typeid(FatherType));
			return *this;
		};

		auto asBasic(std::function<std::string(void*)> to_string, std::function<void(void*, std::string_view)> from_string)->class_<Class_Type>&
		{
			auto &type = reflect_types().at(typeid(Class_Type).hash_code());
			type.is_basic_ = true;
			type.to_string_ = to_string;
			type.from_string_ = from_string;

			return *this;
		}

		// espect: Class_Type::at(size_t id)->T or Class_Type::at(size_t id)->T&  
		//         Class_Type::push_back(T*)->void
		//         Class_Type::size()->size_t
		template<class A = Class_Type>
		auto asRefArray()
			->std::enable_if_t< true
			, class_<Class_Type>&>
		{
			auto &type = reflect_types().at(typeid(Class_Type).hash_code());
			type.is_array_ = true;
			type.is_ref_array_ = true;
			type.size_func_ = [](void* array_instance)->std::size_t
			{
				return reinterpret_cast<A*>(array_instance)->size();
			};
			type.at_func_ = [](void* array_instance, std::size_t id)->Instance
			{
				return reinterpret_cast<A*>(array_instance)->at(id);
			};
			type.push_back_func_ = [](void* array_instance, const Instance& value)->void
			{
				reinterpret_cast<A*>(array_instance)->push_back(
					reinterpret_cast<typename A::value_type*>(const_cast<void*>(value.toVoidPtr()))
				);
			};

			return *this;
		}

		// espect: Class_Type::at(size_t id)->T or Class_Type::at(size_t id)->T&  
		//         Class_Type::push_back(T)->void
		//         Class_Type::size()->size_t    
		template<class A = Class_Type>
		auto asArray()->class_<Class_Type>&
		{
			auto &type = reflect_types().at(typeid(Class_Type).hash_code());
			type.is_array_ = true;
			type.size_func_ = [](void* array_instance)->std::size_t
			{
				return reinterpret_cast<A*>(array_instance)->size();
			};
			type.at_func_ = [](void* array_instance, std::size_t id)->Instance 
			{
				return reinterpret_cast<A*>(array_instance)->at(id);
			};
			type.push_back_func_ = [](void* array_instance, const Instance& value)->void
			{
				return reinterpret_cast<A*>(array_instance)->push_back(
					*reinterpret_cast<const typename A::value_type*>(value.toVoidPtr())
				);
			};

			return *this;
		}

		auto alias(std::string_view name)
		{
			auto[ins2, ok2] = reflect_names().emplace(std::make_pair(std::string(name), typeid(Class_Type).hash_code()));
			if (!ok2) THROW_FILE_LINE("class name already exist");
		}

		// espect: Class_Type::v where v is a value
		template<typename Value>
		auto property(std::string_view name, Value v)
			->std::enable_if_t<std::is_lvalue_reference_v<decltype((new Class_Type)->*v)>, class_<Class_Type>&>
		{
			using T = std::remove_reference_t<decltype((new Class_Type)->*v)>;
			
			auto &type = reflect_types().at(typeid(Class_Type).hash_code());
			auto prop = Property(name, &type);
			prop.get_ = [v](void* ins)->Instance { return reinterpret_cast<Class_Type*>(ins)->*v; };
			prop.set_ = [v](void* ins, const Instance &value) { reinterpret_cast<Class_Type*>(ins)->*v = *reinterpret_cast<const T*>(value.toVoidPtr()); };
			auto [iter, ok] = type.properties_.emplace(std::make_pair(prop.name(), prop));
			
			return *this;
		}

		// espect: Class_Type::v()->T& where v is a reference function
		template<typename Value>
		auto property(std::string_view name, Value v)
			->std::enable_if_t<std::is_lvalue_reference_v<decltype(((new Class_Type)->*v)())>
			&& !std::is_const_v<decltype(((new Class_Type)->*v)())>
			, class_<Class_Type>&>
		{
			using T = std::remove_reference_t<decltype(((new Class_Type)->*v)())>;

			auto &type = reflect_types().at(typeid(Class_Type).hash_code());
			auto prop = Property(name, &type);
			prop.get_ = [v](void* ins)->Instance { return (reinterpret_cast<Class_Type*>(ins)->*v)(); };
			prop.set_ = [v](void* ins, const Instance &value) {	(reinterpret_cast<Class_Type*>(ins)->*v)() = *reinterpret_cast<const T*>(value.toVoidPtr()); };
			auto[iter, ok] = type.properties_.emplace(std::make_pair(prop.name(), prop));

			return *this;
		}

		// espect: Class_Type::setProp(T v)->void  
		//         Class_Type::getProp()->T
		template<typename SetFunc, typename GetFunc, typename C = Class_Type>
		auto property(std::string_view name, SetFunc s, GetFunc g) -> 				
			std::enable_if_t< !(std::is_class_v<C> &&
				std::is_same_v<SetFunc, void (C::*)(std::remove_reference_t<decltype(((new Class_Type)->*g)())>*) >)
			, class_<Class_Type>& >
		{
			using T = std::remove_reference_t<decltype(((new Class_Type)->*g)())>;

			auto &type = reflect_types().at(typeid(Class_Type).hash_code());
			auto prop = Property(name, &type);
			prop.get_ = [g](void* ins)->Instance {	return (reinterpret_cast<Class_Type*>(ins)->*g)(); };
			prop.set_ = [s](void* ins, const Instance &value){ (reinterpret_cast<Class_Type*>(ins)->*s)(*reinterpret_cast<const T*>(value.toVoidPtr())); };
			auto[iter, ok] = type.properties_.emplace(std::make_pair(prop.name(), prop));

			return *this;
		}
		
		// espect: Class_Type::setProp(T *v)->void  
		//         Class_Type::getProp()->T
		//
		// note  : set function will responsible for life-time management of v
		template<typename SetFunc, typename GetFunc, typename C = Class_Type>
		auto property(std::string_view name, SetFunc s, GetFunc g) -> 
				std::enable_if_t< std::is_class_v<C> && 
					std::is_same_v<SetFunc, void (C::*)(std::remove_reference_t<decltype(((new Class_Type)->*g)())>*) >
			, class_<Class_Type>& >
		{
			using T = std::remove_reference_t<decltype(((new Class_Type)->*g)())>;

			auto &type = reflect_types().at(typeid(Class_Type).hash_code());
			auto prop = Property(name, &type);
			prop.get_ = [g](void* ins)->Instance {	return (reinterpret_cast<Class_Type*>(ins)->*g)(); };
			prop.set_ = [s](void* ins, const Instance &value) {	(reinterpret_cast<Class_Type*>(ins)->*s)(const_cast<T*>(reinterpret_cast<const T*>(value.toVoidPtr()))); };
			prop.accept_ptr_ = true;
			auto[iter, ok] = type.properties_.emplace(std::make_pair(prop.name(), prop));

			return *this;
		}

		auto propertyToStrMethod(std::function<std::string(void* value)> func)
		{
			
			
			
			return *this;
		}
		auto propertyFromStrMethod(std::function<void(void* value, std::string_view str)> func)
		{
			return *this;
		}
	};


	auto reflect_types()->std::map<std::size_t, Type>&;
	auto reflect_names()->std::map<std::string, std::size_t>&;
	auto inline getType(std::string_view name)->Type*
	{
		auto found = reflect_names().find(std::string(name));
		return found == reflect_names().end() ? nullptr : &reflect_types().at(found->second);
	}
}

#endif
