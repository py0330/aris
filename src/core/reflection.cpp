#define ARIS_CORE_REFLECTION_CPP_

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

namespace aris::core{
	auto reflect_types_raw()->std::map<std::size_t, Type>&{
		static std::map<std::size_t, Type> reflection_types_;
		return reflection_types_;
	}

	struct Property::Imp {
		std::string name_;

		std::function<Instance(Instance *obj)> get_;
		std::function<void(Instance *obj, Instance prop)> set_;
		bool accept_ptr_{ false };// which means prop is a ptr, to support polymorphim
		Type* type_belong_to_{ nullptr };// obj type, which prop belong to
		Type* type_{ nullptr };// type of this prop
		const std::type_info* type_info_{ nullptr };// type info of this prop

		// only for basic type //
		std::function<std::string(void* value)> to_str_func_;
		std::function<void(void* value, std::string_view str)> from_str_func_;
	};
	struct Instance::Imp{
		const Property *belong_to_{ nullptr };
		const std::type_info* type_info_{ nullptr };

		struct InstanceRef { void* data_; };
		struct InstancePtr { std::shared_ptr<void> data_; };
		std::variant<std::monostate, InstanceRef, InstancePtr> data_;
	};
	struct Type::Imp{
		// init properties, for inheritance //
		bool inited{ false };

		// properties //
		std::string type_name_;
		std::vector<Property> this_properties_;// 不包含父类properties
		std::vector<Property*> properties_ptr_;// 包含父类的properties，为了保持插入顺序，所以不得不用vector替代map

		// inherit types //
		std::vector<const std::type_info *> inherit_type_infos_;// 因为在注册过程中，无法确保所继承的类已经注册完毕，所以需要后续生成该向量
		std::vector<const Type *> inherit_types_;
		std::vector<std::function<void*(void*)>> inherit_cast_vec_; // 将自己cast成基类的函数

		// ctor //
		DefaultCtor default_ctor_;

		// to text // 
		std::function<std::string(Instance*)> to_string_;
		std::function<void(Instance*, std::string_view)> from_string_;

		// array //
		struct ArrayData{
			// for basic //
			bool is_ref_array_{ false };
			const Type* array_type_{ nullptr };
			std::function<std::size_t(Instance*)> size_func_;
			std::function<Instance(Instance*, std::size_t id)> at_func_;
			std::function<void(Instance*, const Instance&)> push_back_func_;
			std::function<void(Instance*)> clear_func_;
		};
		std::unique_ptr<ArrayData> array_data_;

		bool is_polymophic_{ false };
	};

	auto Property::name()const->std::string { return imp_->name_; };
	auto Property::set(Instance *ins, Instance arg)const->void { imp_->set_(ins, arg); }
	auto Property::get(Instance *ins)const->Instance { 
		auto ret = imp_->get_(ins);
		ret.imp_->belong_to_ = this;
		return ret;
	}
	auto Property::acceptPtr()const->bool { return imp_->accept_ptr_; }
	auto Property::type()->const Type* { return &Type::reflect_types().at(imp_->type_info_->hash_code()); }
	auto Property::setToText(std::function<std::string(void*)> to_text)->void { imp_->to_str_func_ = to_text; }
	auto Property::setFromText(std::function<void(void*, std::string_view)> from_text)->void { imp_->from_str_func_ = from_text; }
	Property::~Property() = default;
	Property::Property(std::string_view name, Type *type_belong_to, const std::type_info *type_self, bool accept_ptr, std::function<void(Instance *, Instance)> set, std::function<Instance(Instance *)> get) :imp_(new Imp){
		imp_->name_ = name;
		imp_->type_belong_to_ = type_belong_to;
		imp_->type_info_ = type_self;
		imp_->accept_ptr_ = accept_ptr;
		imp_->set_ = set;
		imp_->get_ = get;
	}

	auto Type::reflect_types()->std::map<std::size_t, Type>&{
		initAllTypes();
		return reflect_types_raw();
	}
	auto Type::reflect_names()->std::map<std::string, std::size_t>&{
		static std::map<std::string, std::size_t> reflection_names_;
		return reflection_names_;
	}
	auto Type::registerType(bool is_polymophic, std::size_t hash_code, std::string_view name, DefaultCtor ctor)->Type*{
		auto[ins, ok] = reflect_types_raw().emplace(std::make_pair(hash_code, Type(name)));
		if (!ok) THROW_FILE_LINE("class already exist");
		auto[ins2, ok2] = reflect_names().emplace(std::make_pair(ins->second.name(), hash_code));
		if (!ok2) THROW_FILE_LINE("class name already exist");

		ins->second.imp_->default_ctor_ = ctor;
		ins->second.imp_->is_polymophic_ = is_polymophic;
		return &ins->second;
	}
	auto Type::alias_impl(Type*type, std::string_view alias_name)->void{
		auto code = reflect_names().at(std::string(type->name()));
		auto[ins2, ok2] = reflect_names().emplace(std::make_pair(std::string(alias_name), code));
		if (!ok2) THROW_FILE_LINE("class name already exist");
	}
	auto Type::getType(std::string_view name)->Type*{
		auto found = reflect_names().find(std::string(name));
		return found == reflect_names().end() ? nullptr : &reflect_types().at(found->second);
	}
	auto Type::create()const->std::tuple<std::unique_ptr<void, void(*)(void const*)>, Instance>{ return imp_->default_ctor_();}
	auto Type::name()const->std::string_view { return imp_->type_name_; };
	auto Type::isBasic()const->bool { return imp_->to_string_ || imp_->from_string_; }
	auto Type::isClass()const->bool { return (!isBasic()); }
	auto Type::isArray()const->bool { return imp_->array_data_.get(); }
	auto Type::isRefArray()const->bool { return imp_->array_data_.get() && imp_->array_data_->is_ref_array_; }
	auto Type::inheritTypes()const->std::vector<const Type*>{ return imp_->inherit_types_; }
	auto Type::properties()const->const std::vector<Property*>&{return imp_->properties_ptr_;};
	auto Type::propertyAt(std::string_view name)const->Property*{
		auto found = std::find_if(properties().begin(), properties().end(), [name](Property* prop) ->bool{
			return prop->name() == name;
		});
		return found == properties().end() ? nullptr : *found;
	}
	auto Type::inherit(const std::type_info *inherit_type_info, CastFunc func)->void{
		imp_->inherit_type_infos_.push_back(inherit_type_info);
		imp_->inherit_cast_vec_.push_back(func);// 多继承可能改变指针所指向的位置，坑爹！！//
	}
	auto Type::init()const->void{
		if (!imp_->inited){
			auto this_type = const_cast<Type*>(this);
			this_type->imp_->inited = true;
			
			// init inherit types //
			for (auto &t : imp_->inherit_type_infos_){
				if (reflect_types().find(t->hash_code()) == reflect_types().end()) std::cout << "Unregistered inherited type:" << t->name() << std::endl;
				if (reflect_types().find(t->hash_code()) == reflect_types().end())	THROW_FILE_LINE("Unregistered inherited type:" + t->name());
				this_type->imp_->inherit_types_.push_back(&reflect_types().at(t->hash_code()));
				imp_->inherit_types_.back()->init();
			}
			
			// init inherit properties //
			auto insert_prop = [this_type](Property* ist){
				// 防止重名的数据
				auto &vec = this_type->imp_->properties_ptr_;
				auto found = std::find_if(vec.begin(), vec.end(), [ist](Property* prop) ->bool{
					return prop->name() == ist->name();
				});

				if (found == vec.end())	vec.push_back(ist);
				else *found = ist;
			};
			for (auto t : inheritTypes()){
				for (auto ist : t->properties()){
					insert_prop(ist);
				}
			}
			for (auto &ist : this_type->this_properties())insert_prop(const_cast<Property*>(&ist));

			// copy inherited, text methods & array_infos //
			for (auto &t : imp_->inherit_types_){
				// copy array_infos //
				if (!isArray() && t->isArray()){
					this_type->imp_->array_data_.reset(new Type::Imp::ArrayData(*t->imp_->array_data_));
				}

				// copy text methods //
				if (!imp_->to_string_)this_type->imp_->to_string_ = t->imp_->to_string_;
				if (!imp_->from_string_)this_type->imp_->from_string_ = t->imp_->from_string_;
			}
		}
	}
	auto Type::initAllTypes()->void{
		static bool init_all_ = false;

		if (init_all_)return;
		init_all_ = true;

		for (auto &t : reflect_types_raw()){
			t.second.init();
		}
	}
	auto Type::isBaseOf(const Type* base, const Type* derived)->bool{
		if (base == nullptr) return true;
		if (derived == nullptr) return false;
		
		if (base == derived || std::find(derived->imp_->inherit_types_.begin(), derived->imp_->inherit_types_.end(), base) != derived->imp_->inherit_types_.end()){
			return true;
		}

		for (auto t : derived->imp_->inherit_types_){
			if (isBaseOf(base, t)) return true;
		}

		return false;
	}
	auto Type::this_properties()->std::vector<Property>& { return imp_->this_properties_; }
	auto Type::text(std::function<std::string(Instance*)> to_string, std::function<void(Instance*, std::string_view)> from_string)->void {
		imp_->to_string_ = to_string;
		imp_->from_string_ = from_string;
	}
	auto Type::as_array(bool is_ref, std::function<std::size_t(Instance*)> size_func, std::function<Instance(Instance*, std::size_t)>at_func, std::function<void(Instance*, const Instance&)>push_back_func, std::function<void(Instance*)>clear_func)->void{
		imp_->array_data_ = std::unique_ptr<Type::Imp::ArrayData>(new Type::Imp::ArrayData{ is_ref, this, size_func, at_func,push_back_func, clear_func });
	}
	Type::~Type() = default;
	Type::Type(std::string_view name) :imp_(new Imp) { imp_->type_name_ = name; }

	auto Instance::castToType(const Type*t)const->void* {
		if (type() == t) return toVoidPtr();

		std::function<void*(const Type*, void*)> iterative_cast = [&](const Type*type, void* data)->void* {
			auto inherit_types = type->inheritTypes();

			for (Size i = 0; i < inherit_types.size(); ++i)	{
				if (inherit_types[i] == t)
					return type->imp_->inherit_cast_vec_[i](data);
				
				if (auto success = iterative_cast(inherit_types[i], type->imp_->inherit_cast_vec_[i](data)))
					return success;
			}

			return nullptr;
		};

		return iterative_cast(type(), toVoidPtr());
	}
	auto Instance::toVoidPtr()const->void* {return isReference() ? std::get<Imp::InstanceRef>(imp_->data_).data_ : std::get<Imp::InstancePtr>(imp_->data_).data_.get();}
	auto Instance::set(std::string_view prop_name, Instance arg)->void {
		type()->propertyAt(prop_name)->set(this, arg);
	}
	auto Instance::get(std::string_view prop_name)->Instance {
		return type()->propertyAt(prop_name)->get(this);
	}
	auto Instance::type()const->const Type* {
		if (isEmpty()) return nullptr;
		return Type::reflect_types().find(imp_->type_info_->hash_code()) == Type::reflect_types().end() ? nullptr : &Type::reflect_types().at(imp_->type_info_->hash_code());
	}
	auto Instance::isEmpty()const->bool { return imp_->data_.index() == 0; }
	auto Instance::isReference()const->bool { return imp_->data_.index() == 1; }
	auto Instance::isSharedReference()const->bool { return imp_->data_.index() == 2; }
	auto Instance::toString()->std::string { 
		if (imp_->belong_to_ && imp_->belong_to_->imp_->to_str_func_)return imp_->belong_to_->imp_->to_str_func_(toVoidPtr());
		if (type()->imp_->to_string_)return type()->imp_->to_string_(this);
		return "";
	}
	auto Instance::fromString(std::string_view str)->void { 
		if (imp_->belong_to_ && imp_->belong_to_->imp_->from_str_func_)return imp_->belong_to_->imp_->from_str_func_(toVoidPtr(), str);
		if (type()->imp_->from_string_)type()->imp_->from_string_(this, str);
	}
	auto Instance::size()->std::size_t{
		if (!isArray())THROW_FILE_LINE("instance is NOT array");
		return type()->imp_->array_data_->size_func_(this);
	}
	auto Instance::at(std::size_t id)->Instance	{
		if (!isArray())THROW_FILE_LINE("instance is NOT array");
		return type()->imp_->array_data_->at_func_(this, id);
	}
	auto Instance::push_back(Instance element)->void {
		if (!isArray())THROW_FILE_LINE("instance is NOT array");
		type()->imp_->array_data_->push_back_func_(this, element);
	}
	auto Instance::clear()->void {
		if (!isArray())THROW_FILE_LINE("instance is NOT array");
		type()->imp_->array_data_->clear_func_(this);
	}
	Instance::Instance(const std::type_info *info, void* data):imp_(new Imp) {
		imp_->type_info_ = info;
		imp_->data_ = Imp::InstanceRef{ data };
	}
	Instance::Instance(const std::type_info *info, std::shared_ptr<void> data) : imp_(new Imp) {
		imp_->type_info_ = info;
		imp_->data_ = Imp::InstancePtr{ data };
	}
	Instance::~Instance() = default;
	Instance::Instance() :imp_(new Imp) {}
	Instance::Instance(const Instance&) = default;
	Instance::Instance(Instance&&) noexcept = default;

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

	ARIS_REGISTRATION{
		aris::core::class_<char>("char")
			.textMethod(
				[](char *v)->std::string{
					return std::string(1, *v);
				}, [](char *v, std::string_view str)->void{
					*v = str[0];
				})
			;
		
		aris::core::class_<std::int8_t>("int8")
			.textMethod(int8_to_str, int8_from_str)
			;

		aris::core::class_<std::int16_t>("int16")
			.textMethod(int16_to_str, int16_from_str)
			;

		aris::core::class_<std::int32_t>("int32")
			.textMethod(int32_to_str, int32_from_str)
			.alias("int")
			;

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
				float result = (float)std::stod(std::string(str));
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