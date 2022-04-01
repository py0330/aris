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
	auto reflect_types_raw()->std::map<std::size_t, Type>& {
		static std::map<std::size_t, Type> reflection_types_;
		return reflection_types_;
	}

	struct Property::Imp {
		std::string name_;
		bool accept_ptr_{ false };// which means prop is a ptr, to support polymorphim
		Type* type_belong_to_{ nullptr };// obj type, which prop belong to
		std::size_t type_hash_code_{0};// hash_code of this prop

		// set & get //
		std::function<Instance(Instance* obj)> get_;
		std::function<void(Instance* obj, Instance prop)> set_;

		// only for basic type //
		std::function<std::string(void* value)> to_str_func_;
		std::function<void(void* value, std::string_view str)> from_str_func_;
	};
	struct Instance::Imp{
		struct Reference { void* data_; std::size_t type_hash_code_; };
		struct Shared { std::shared_ptr<void> data_; std::size_t type_hash_code_;};
		struct Unique { 
			typedef void(*DeleteFunc)(void*);
			std::unique_ptr<void, DeleteFunc> data_; std::size_t type_hash_code_;
		};
		std::variant<std::monostate, Reference, Shared, Unique> data_;
	};
	struct Type::Imp{
		// init properties, for inheritance //
		bool inited{ false };

		// typeinfo //
		std::size_t hash_code_{ 0 };//equal to type_info.hash_code()

		// properties //
		std::string type_name_;
		std::vector<Property> this_properties_;// 不包含父类properties
		std::vector<Property*> properties_ptr_;// 包含父类的properties，为了保持插入顺序，所以不得不用vector替代map

		// inherit types //
		std::vector<const std::type_info *> inherit_type_infos_;// 因为在注册过程中，无法确保所继承的类已经注册完毕，所以需要后续生成该向量
		std::vector<const Type *> inherit_types_;
		std::vector<std::function<void*(void*)>> inherit_cast_vec_; // 将自己cast成基类的函数

		// ctor & dtor //
		DefaultCtor default_ctor_{ nullptr };
		DeleteFunc dtor_{ nullptr };

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

		auto init()->void {
			if (!this->inited) {
				this->inited = true;

				// init inherit types //
				for (auto& t : this->inherit_type_infos_) {
					if (reflect_types().find(t->hash_code()) == reflect_types().end()) std::cout << "Unregistered inherited type:" << t->name() << std::endl;
					if (reflect_types().find(t->hash_code()) == reflect_types().end())	THROW_FILE_LINE("Unregistered inherited type:" + t->name());
					this->inherit_types_.push_back(&reflect_types().at(t->hash_code()));
					this->inherit_types_.back()->imp_->init();
				}

				// init inherit properties //
				auto insert_prop = [this](Property* ist) {
					// 防止重名的数据
					auto& vec = this->properties_ptr_;
					auto found = std::find_if(vec.begin(), vec.end(), [ist](Property* prop) ->bool {
						return prop->name() == ist->name();
						});

					if (found == vec.end())	vec.push_back(ist);
					else *found = ist;
				};
				for (auto t : this->inherit_types_) {
					for (auto ist : t->properties()) {
						insert_prop(ist);
					}
				}
				for (auto& ist : this->this_properties_)insert_prop(const_cast<Property*>(&ist));

				// copy inherited, text methods & array_infos //
				for (auto& t : this->inherit_types_) {
					// copy array_infos //
					if (!array_data_ && t->imp_->array_data_) {
						this->array_data_.reset(new Type::Imp::ArrayData(*t->imp_->array_data_));
					}

					// copy text methods //
					if (!this->to_string_)this->to_string_ = t->imp_->to_string_;
					if (!this->from_string_)this->from_string_ = t->imp_->from_string_;
				}
			}
		}
		static auto initAllTypes()->void {
			static bool init_all_ = false;

			if (init_all_)return;
			init_all_ = true;

			for (auto& t : reflect_types_raw()) {
				t.second.imp_->init();
			}
		}
		static auto reflect_types()->std::map<std::size_t, Type>& {
			initAllTypes();
			return reflect_types_raw();
		}
		static auto reflect_names()->std::map<std::string, std::size_t>& {
			static std::map<std::string, std::size_t> reflection_names_;
			return reflection_names_;
		}
	};

	auto Property::name()const->std::string { return imp_->name_; };
	auto Property::type()->const Type* { return Type::getType(imp_->type_hash_code_); }
	auto Property::acceptPtr()const->bool { return imp_->accept_ptr_; }
	auto Property::set(Instance *ins, Instance arg)const->void { imp_->set_(ins, std::move(arg)); }
	auto Property::get(Instance *ins)const->Instance { return imp_->get_(ins); }
	auto Property::toString(Instance* obj)->std::string {
		if ((!type()->isBasic()) || (!obj->castToVoidPointer(imp_->type_belong_to_)))return "";
		auto prop_value = get(obj);
		return imp_->to_str_func_ ? imp_->to_str_func_(prop_value.castToVoidPointer(type())) : type()->imp_->to_string_(&prop_value);
	}
	auto Property::fromString(Instance* obj, std::string_view str)->void {
		if ((!type()->isBasic()) || (!obj->castToVoidPointer(imp_->type_belong_to_)))return;
		auto ins = type()->create();
		imp_->from_str_func_ ? imp_->from_str_func_(ins.castToVoidPointer(type()), str) : type()->imp_->from_string_(&ins, str);
		set(obj, ins);
	}
	auto Property::setToText(std::function<std::string(void*)> to_text)->void { imp_->to_str_func_ = to_text; }
	auto Property::setFromText(std::function<void(void*, std::string_view)> from_text)->void { imp_->from_str_func_ = from_text; }
	Property::~Property() = default;
	Property::Property(std::string_view name, Type *type_belong_to, std::size_t type_hash_code, bool accept_ptr, std::function<void(Instance *, Instance)> set, std::function<Instance(Instance *)> get) :imp_(new Imp){
		imp_->name_ = name;
		imp_->type_belong_to_ = type_belong_to;
		imp_->type_hash_code_ = type_hash_code;
		imp_->accept_ptr_ = accept_ptr;
		imp_->set_ = set;
		imp_->get_ = get;
	}

	auto Type::registerType(std::size_t hash_code, std::string_view name, DefaultCtor ctor, DeleteFunc dtor)->Type*{
		auto[ins, ok] = reflect_types_raw().emplace(std::make_pair(hash_code, Type(name)));
		if (!ok) THROW_FILE_LINE("class already exist");
		auto[ins2, ok2] = Imp::reflect_names().emplace(std::make_pair(ins->second.name(), hash_code));
		if (!ok2) THROW_FILE_LINE("class name already exist");

		ins->second.imp_->default_ctor_ = ctor;
		ins->second.imp_->dtor_ = dtor;
		ins->second.imp_->hash_code_ = hash_code;
		return &ins->second;
	}
	auto Type::alias_impl(Type*type, std::string_view alias_name)->void{
		auto code = Imp::reflect_names().at(std::string(type->name()));
		auto[ins2, ok2] = Imp::reflect_names().emplace(std::make_pair(std::string(alias_name), code));
		if (!ok2) THROW_FILE_LINE("class name already exist");
	}
	auto Type::getType(std::string_view name)->Type*{
		auto found = Imp::reflect_names().find(std::string(name));
		return found == Imp::reflect_names().end() ? nullptr : &Imp::reflect_types().at(found->second);
	}
	auto Type::getType(std::size_t hashcode)->Type* {
		auto found = Imp::reflect_types().find(hashcode);
		return found == Imp::reflect_types().end() ? nullptr : &found->second;
	}
	auto Type::create()const->Instance {
		return Instance(imp_->hash_code_, std::unique_ptr<void, void(*)(void*)>(imp_->default_ctor_(), imp_->dtor_));
	}
	auto Type::name()const->std::string_view { return imp_->type_name_; };
	auto Type::isBuiltIn()const->bool {
		static const std::set<std::size_t> built_in_types = {
			typeid(bool).hash_code(),
			typeid(char).hash_code(),
			typeid(std::string).hash_code(),
			typeid(std::int8_t).hash_code(),
			typeid(std::int16_t).hash_code(),
			typeid(std::int32_t).hash_code(),
			typeid(std::int64_t).hash_code(),
			typeid(std::uint8_t).hash_code(),
			typeid(std::uint16_t).hash_code(),
			typeid(std::uint32_t).hash_code(),
			typeid(std::uint64_t).hash_code(),
			typeid(float).hash_code(),
			typeid(double).hash_code(),
		};
		
		return built_in_types.find(imp_->hash_code_) == built_in_types.end();
	}
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

	// built in types transfer method //
	auto b2s(bool* v)->std::string { return *v ? "true" : "false"; }
	auto bfs(bool* v, std::string_view str)->void {
		std::string str_(str);
		str_.erase(0, str_.find_first_not_of(" \t\n\r\f\v"));// trim l
		str_.erase(str_.find_last_not_of(" \t\n\r\f\v") + 1);// trim r
		*v = str_ == "false" || str_ == "0" ? true : false;
	}

	auto c2s(char* v)->std::string { return std::string(1, *v); }
	auto cfs(char* v, std::string_view str)->void { *v = str[0]; }

	auto s2s(std::string* v)->std::string { return *v; }
	auto sfs(std::string* v, std::string_view str)->void { *v = str; }
	auto i82s(std::int8_t* v)->std::string { return std::to_string(*v); }
	auto i8fs(std::int8_t* v, std::string_view str)->void { *v = (std::int8_t)std::strtoll(str.data(), nullptr, 0); }
	auto i12s(std::int16_t* v)->std::string { return std::to_string(*v); }
	auto i1fs(std::int16_t* v, std::string_view str)->void { *v = (std::int16_t)std::strtoll(str.data(), nullptr, 0); }
	auto i32s(std::int32_t* v)->std::string { return std::to_string(*v); }
	auto i3fs(std::int32_t* v, std::string_view str)->void { *v = (std::int32_t)std::strtoll(str.data(), nullptr, 0); }
	auto i62s(std::int64_t* v)->std::string { return std::to_string(*v); }
	auto i6fs(std::int64_t* v, std::string_view str)->void { *v = std::strtoll(str.data(), nullptr, 0); }
	auto u82s(std::uint8_t* v)->std::string { return std::to_string(*v); }
	auto u8fs(std::uint8_t* v, std::string_view str)->void { *v = (std::uint8_t)std::strtoull(str.data(), nullptr, 0); }
	auto u12s(std::uint16_t* v)->std::string { return std::to_string(*v); }
	auto u1fs(std::uint16_t* v, std::string_view str)->void { *v = (std::uint16_t)std::strtoull(str.data(), nullptr, 0); }
	auto u32s(std::uint32_t* v)->std::string { return std::to_string(*v); }
	auto u3fs(std::uint32_t* v, std::string_view str)->void { *v = (std::uint32_t)std::strtoull(str.data(), nullptr, 0); }
	auto u62s(std::uint64_t* v)->std::string { return std::to_string(*v); }
	auto u6fs(std::uint64_t* v, std::string_view str)->void { *v = std::strtoull(str.data(), nullptr, 0); }

	auto f2s(float* v)->std::string { return std::to_string(*v); }
	auto ffs(float* v, std::string_view str)->void { *v = (float)std::stod(std::string(str)); }
	auto d2s(double* v)->std::string {
		char buf[100]{ 0 };
		std::sprintf(buf, "%.17g", *v);
		return buf;
	}
	auto dfs(double* v, std::string_view str)->void { *v = std::stod(std::string(str)); }


	// static cast //
	template<typename t1, typename t2>
	auto sc(Instance value)->Instance {
		return Instance(static_cast<t2>(*value.castToPointer<t1>()));
	}

	auto Instance::type()const->const Type* {
		switch (imp_->data_.index()) {
		case 0:return nullptr;
		case 1:return Type::getType(std::get<Imp::Reference>(imp_->data_).type_hash_code_);
		case 2:return Type::getType(std::get<Imp::Shared>(imp_->data_).type_hash_code_);
		case 3:return Type::getType(std::get<Imp::Unique>(imp_->data_).type_hash_code_);
		default:return nullptr;
		}
	}
	auto Instance::ownership()const noexcept->Ownership {
		static const Ownership owner_table[] = { Ownership::EMPTY,Ownership::REFERENCE,Ownership::SHARED,Ownership::UNIQUE };
		return owner_table[imp_->data_.index()];
	}
	auto Instance::release()->void* {
		if (ownership() == Ownership::UNIQUE) return std::get<Imp::Unique>(imp_->data_).data_.release();
		else THROW_FILE_LINE("Unable to release");
	}
	auto Instance::castToVoidPointer(const Type*t)const->void* {
		void* raw_pointer{nullptr};

		switch (imp_->data_.index()) {
		case 0:raw_pointer = nullptr; return nullptr;
		case 1:raw_pointer = std::get<Imp::Reference>(imp_->data_).data_; break;
		case 2:raw_pointer = std::get<Imp::Shared>(imp_->data_).data_.get(); break;
		case 3:raw_pointer = std::get<Imp::Unique>(imp_->data_).data_.get(); break;
		}

		if (type() == t) return raw_pointer;

		std::function<void*(const Type*, void*)> iterative_cast = [&](const Type*type, void* data)->void* {
			auto &&inherit_types = type->inheritTypes();

			for (Size i = 0; i < inherit_types.size(); ++i)	{
				if (inherit_types[i] == t)
					return type->imp_->inherit_cast_vec_[i](data);
				
				if (auto success = iterative_cast(inherit_types[i], type->imp_->inherit_cast_vec_[i](data)))
					return success;
			}

			return nullptr;
		};

		return iterative_cast(type(), raw_pointer);
	}
	auto Instance::castToVoidValue(const Type* t)const->Instance {
		using b = bool;
		using c = char;
		using s = std::string;
		using i8 = std::int8_t;
		using i1 = std::int16_t;
		using i3 = std::int32_t;
		using i6 = std::int64_t;
		using u8 = std::uint8_t;
		using u1 = std::uint16_t;
		using u3 = std::uint32_t;
		using u6 = std::uint64_t;
		using f = float;
		using d = double;

#define t2s(type) [](Instance v)->Instance{ return ARIS_REFLECT_CAT(type,2s)(v.castToPointer<type>()); }
#define s2t(type) [](Instance str)->Instance{ \
			type ret;\
			ARIS_REFLECT_CAT(type,fs)(&ret, *str.castToPointer<s>()); \
			return std::move(ret);\
		}

		static const std::map<const Type*, int> basic_type_pos = { 
			{Type::getType(typeid(bool)         .hash_code()),  0},
			{Type::getType(typeid(char)         .hash_code()),  1},
			{Type::getType(typeid(std::string)  .hash_code()),  2},
			{Type::getType(typeid(std::int8_t)  .hash_code()),  3},
			{Type::getType(typeid(std::int16_t) .hash_code()),  4},
			{Type::getType(typeid(std::int32_t) .hash_code()),  5},
			{Type::getType(typeid(std::int64_t) .hash_code()),  6},
			{Type::getType(typeid(std::uint8_t) .hash_code()),  7},
			{Type::getType(typeid(std::uint16_t).hash_code()),  8},
			{Type::getType(typeid(std::uint32_t).hash_code()),  9},
			{Type::getType(typeid(std::uint64_t).hash_code()), 10},
			{Type::getType(typeid(float)        .hash_code()), 11},
			{Type::getType(typeid(double)       .hash_code()), 12},
		};

		using CastFunc = std::function<Instance(Instance)>;

		static const CastFunc cast_func[13][13]{ 
			//   bool      char     string       int8      int16      int32      int64      uint8     uint16     uint32     uint64     float    double
			{sc<b ,b>, sc<b ,c>, t2s(b   ), sc<b ,i8>, sc<b ,i1>, sc<b ,i3>, sc<b ,i6>, sc<b ,u8>, sc<b ,u1>, sc<b ,u3>, sc<b ,u6>, sc<b ,f>, sc<b ,d>},  // bool
			{sc<c ,b>, sc<c ,c>, t2s(c   ), sc<c ,i8>, sc<c ,i1>, sc<c ,i3>, sc<c ,i6>, sc<c ,u8>, sc<c ,u1>, sc<c ,u3>, sc<c ,u6>, sc<c ,f>, sc<c ,d>},  // char
			{s2t(  b), s2t(  c), sc< s, s>, s2t(  i8), s2t(  i1), s2t(  i3), s2t(  i6), s2t(  u8), s2t(  u1), s2t(  u3), s2t(  u6), s2t(  f), s2t(  d)},  // string
			{sc<i8,b>, sc<i8,c>, t2s(i8  ), sc<i8,i8>, sc<i8,i1>, sc<i8,i3>, sc<i8,i6>, sc<i8,u8>, sc<i8,u1>, sc<i8,u3>, sc<i8,u6>, sc<i8,f>, sc<i8,d>},  // int8
			{sc<i1,b>, sc<i1,c>, t2s(i1  ), sc<i1,i8>, sc<i1,i1>, sc<i1,i3>, sc<i1,i6>, sc<i1,u8>, sc<i1,u1>, sc<i1,u3>, sc<i1,u6>, sc<i1,f>, sc<i1,d>},  // int16
			{sc<i3,b>, sc<i3,c>, t2s(i3  ), sc<i3,i8>, sc<i3,i1>, sc<i3,i3>, sc<i3,i6>, sc<i3,u8>, sc<i3,u1>, sc<i3,u3>, sc<i3,u6>, sc<i3,f>, sc<i3,d>},  // int32
			{sc<i6,b>, sc<i6,c>, t2s(i6  ), sc<i6,i8>, sc<i6,i1>, sc<i6,i3>, sc<i6,i6>, sc<i6,u8>, sc<i6,u1>, sc<i6,u3>, sc<i6,u6>, sc<i6,f>, sc<i6,d>},  // int64
			{sc<u8,b>, sc<u8,c>, t2s(u8  ), sc<u8,i8>, sc<u8,i1>, sc<u8,i3>, sc<u8,i6>, sc<u8,u8>, sc<u8,u1>, sc<u8,u3>, sc<u8,u6>, sc<u8,f>, sc<u8,d>},  // uint8
			{sc<u1,b>, sc<u1,c>, t2s(u1  ), sc<u1,i8>, sc<u1,i1>, sc<u1,i3>, sc<u1,i6>, sc<u1,u8>, sc<u1,u1>, sc<u1,u3>, sc<u1,u6>, sc<u1,f>, sc<u1,d>},  // uint16
			{sc<u3,b>, sc<u3,c>, t2s(u3  ), sc<u3,i8>, sc<u3,i1>, sc<u3,i3>, sc<u3,i6>, sc<u3,u8>, sc<u3,u1>, sc<u3,u3>, sc<u3,u6>, sc<u3,f>, sc<u3,d>},  // uint32
			{sc<u6,b>, sc<u6,c>, t2s(u6  ), sc<u6,i8>, sc<u6,i1>, sc<u6,i3>, sc<u6,i6>, sc<u6,u8>, sc<u6,u1>, sc<u6,u3>, sc<u6,u6>, sc<u6,f>, sc<u6,d>},  // uint64
			{sc<f ,b>, sc<f ,c>, t2s(f   ), sc<f ,i8>, sc<f ,i1>, sc<f ,i3>, sc<f ,i6>, sc<f ,u8>, sc<f ,u1>, sc<f ,u3>, sc<f ,u6>, sc<f ,f>, sc<f ,d>},  // float
			{sc<d ,b>, sc<d ,c>, t2s(d   ), sc<d ,i8>, sc<d ,i1>, sc<d ,i3>, sc<d ,i6>, sc<d ,u8>, sc<d ,u1>, sc<d ,u3>, sc<d ,u6>, sc<d ,f>, sc<d ,d>},  // double
		};
#undef t2s
#undef s2t
		
		return cast_func[basic_type_pos.at(this->type())]
						[basic_type_pos.at(t)]
						(*this);
	}
	auto Instance::set(std::string_view prop_name, Instance arg)->void {
		type()->propertyAt(prop_name)->set(this, std::move(arg));
	}
	auto Instance::get(std::string_view prop_name)->Instance {
		return type()->propertyAt(prop_name)->get(this);
	}
	auto Instance::toString()->std::string { return isBasic() ? type()->imp_->to_string_(this) : ""; }
	auto Instance::fromString(std::string_view str)->void { if(isBasic())type()->imp_->from_string_(this, str); }
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
	Instance::Instance(std::size_t type_hash_code, void* data):imp_(new Imp) {
		imp_->data_ = Imp::Reference{ data, type_hash_code };
	}
	Instance::Instance(std::size_t type_hash_code, std::unique_ptr<void, void (*)(void*)> data) {
		imp_->data_ = Imp::Unique{ std::move(data), type_hash_code };
	}
	Instance::~Instance() = default;
	Instance::Instance() :imp_(new Imp) {}
	Instance::Instance(const Instance&other){
		switch (other.ownership()) {
		case Ownership::EMPTY:
			break;
		case Ownership::REFERENCE:
			imp_->data_ = std::get<Imp::Reference>(other.imp_->data_);
			break;
		case Ownership::SHARED:
			imp_->data_ = std::get<Imp::Shared>(other.imp_->data_);
			break;
		case Ownership::UNIQUE: {
			auto type_hash_code = std::get<Imp::Unique>(other.imp_->data_).type_hash_code_;
			auto shared = std::shared_ptr<void>(
				std::get<Imp::Unique>(
					const_cast<Instance&>(other).imp_->data_
					)
				.data_.release(),
				std::get<Imp::Unique>(
					const_cast<Instance&>(other).imp_->data_
					).data_.get_deleter()
			);

			imp_->data_ = Imp::Shared{
				shared,
				type_hash_code
			};
			const_cast<Instance&>(other).imp_->data_ = std::get<Imp::Shared>(imp_->data_);
			break; 
		}
		default:
			return;
		}
		
	}
	Instance::Instance(Instance&&) noexcept = default;
	Instance& Instance::operator=(const Instance&other) {
		auto type_hash_code = std::get<Imp::Unique>(other.imp_->data_).type_hash_code_;
		auto shared = std::shared_ptr<void>(
			std::get<Imp::Unique>(
				const_cast<Instance&>(other).imp_->data_
				)
			.data_.release()
			);

		imp_->data_ = Imp::Shared{
			shared,
			type_hash_code
		};
		const_cast<Instance&>(other).imp_->data_ = std::get<Imp::Shared>(imp_->data_);
		return *this;
	}
	Instance& Instance::operator=(Instance&&)noexcept = default;

	ARIS_REGISTRATION{
		aris::core::class_<bool>("bool")
			.textMethod(b2s, bfs);

		aris::core::class_<char>("char")
			.textMethod(c2s, cfs);

		aris::core::class_<std::string>("string")
			.textMethod(s2s, sfs);
		
		aris::core::class_<std::int8_t>("int8")
			.textMethod(i82s, i8fs);

		aris::core::class_<std::int16_t>("int16")
			.textMethod(i12s, i1fs);

		aris::core::class_<std::int32_t>("int32")
			.textMethod(i32s, i3fs)
			.alias("int");

		aris::core::class_<std::int64_t>("int64")
			.textMethod(i62s, i6fs);

		aris::core::class_<std::uint8_t>("uint8")
			.textMethod(u82s, u8fs);

		aris::core::class_<std::uint16_t>("uint16")
			.textMethod(u12s, u1fs);

		aris::core::class_<std::uint32_t>("uint32")
			.textMethod(u32s, u3fs)
			.alias("uint");

		aris::core::class_<std::uint64_t>("uint64")
			.textMethod(u62s, u6fs);

		aris::core::class_<float>("float")
			.textMethod(f2s, ffs);

		aris::core::class_<double>("double")
			.textMethod(d2s, dfs);
	}
}