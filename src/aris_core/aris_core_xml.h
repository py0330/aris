#ifndef ARIS_CORE_XML_H_
#define ARIS_CORE_XML_H_

#include <tinyxml2.h>
#include <cstring>
#include <memory>
#include <vector>
#include <type_traits>
#include <functional>
#include <algorithm>
#include <map>
#include <utility>
#include <sstream>

#include <iostream>

namespace aris
{
	namespace core
	{
		using XmlDocument = tinyxml2::XMLDocument;
		using XmlDeclaration = tinyxml2::XMLDeclaration ;
		using XmlNode = tinyxml2::XMLNode;
		using XmlElement = tinyxml2::XMLElement;
		using XmlAttribute = tinyxml2::XMLAttribute;
		class Root;

		template<typename T> class ImpPtr
		{
		public:
			auto reset(T* p)->void { data_unique_ptr_.reset(p); }
			auto get()const->const T*{ return data_unique_ptr_.get(); }
			auto get()->T* { return data_unique_ptr_.get(); }
			auto operator->()const->const T*{ return data_unique_ptr_.get(); }
			auto operator->()->T*{ return data_unique_ptr_.get(); }
			auto operator*()const->const T& { return *data_unique_ptr_; }
			auto operator*()->T&{ return *data_unique_ptr_; }
			
			~ImpPtr() = default;
			explicit ImpPtr(T *data_ptr) :data_unique_ptr_(data_ptr) {}
			explicit ImpPtr() :data_unique_ptr_(new T) {}
			ImpPtr(const ImpPtr &other) :data_unique_ptr_(new T(*other.data_unique_ptr_)) {}
			ImpPtr(ImpPtr &&other)noexcept = default;
			ImpPtr& operator=(const ImpPtr &other) { *data_unique_ptr_ = *other.data_unique_ptr_; return *this; }
			ImpPtr& operator=(ImpPtr &&other)noexcept = default;
			
		private:
			std::unique_ptr<T> data_unique_ptr_;
		};
		template <class T, class A = std::allocator<T> >class ImpContainer 
		{
		public:
			using allocator_type = A;
			using value_type = typename A::value_type;
			using reference = typename A::reference;
			using const_reference = typename A::const_reference;
			using pointer = typename A::pointer;
			using const_pointer = typename A::const_pointer;
			using difference_type = typename A::difference_type;
			using size_type = typename A::size_type;

			class iterator
			{
			public:
				using difference_type = typename A::difference_type;
				using value_type = typename A::value_type;
				using reference = typename A::reference;
				using pointer = typename A::pointer;
				using iterator_category = std::random_access_iterator_tag; //or another tag

				auto operator=(const iterator&other)->iterator& = default;
				auto operator==(const iterator&other) const->bool { return iter_ == other.iter_; }
				auto operator!=(const iterator&other) const->bool { return iter_ != other.iter_; }
				auto operator<(const iterator&other) const->bool { return iter_ < other.iter_; } //optional
				auto operator>(const iterator&other) const->bool { return iter_ > other.iter_; } //optional
				auto operator<=(const iterator&other) const->bool { return iter_ <= other.iter_; } //optional
				auto operator>=(const iterator&other) const->bool { return iter_ >= other.iter_; } //optional

				auto operator++()->iterator& { ++iter_; return *this; }
				auto operator++(int)->iterator { iterator ret(*this); operator++(); return ret; } //optional
				auto operator--()->iterator& { --iter_; return *this; } //optional
				auto operator--(int)->iterator { iterator ret(*this); operator--(); return ret; } //optional
				auto operator+=(size_type size)->iterator& { iter_ += size; return *this; } //optional
				auto operator+(size_type size) const->iterator { return iter_ + size; } //optional
				friend auto operator+(size_type size, const iterator&iter)->iterator { return size + iter.iter_; } //optional
				auto operator-=(size_type size)->iterator& { iter_ -= size; return *this; } //optional
				auto operator-(size_type size) const->iterator { return iter_ - size; } //optional
				auto operator-(iterator iter) const->difference_type { return iter_ - iter.iter_; } //optional

				auto operator*() const->reference { return iter_->operator*(); }
				auto operator->() const->pointer { return iter_->operator->(); }
				auto operator[](size_type size) const->reference { return *iter_->operator[](size); } //optional

				~iterator() = default;
				iterator() = default;
				iterator(const iterator& other) = default;
				iterator(const typename std::vector<ImpPtr<T>>::iterator iter) :iter_(iter) {} //

			private:
				friend class ImpContainer::const_iterator;
				friend class ImpContainer<T,A>;
				typename std::vector<ImpPtr<T>>::iterator iter_;
			};
			class const_iterator 
			{
			public:
				using difference_type = typename A::difference_type;
				using value_type = typename A::value_type;
				using const_reference = typename A::const_reference;
				using const_pointer = typename A::const_pointer;
				using iterator_category = std::random_access_iterator_tag; //or another tag

				auto operator=(const const_iterator&)->const_iterator& = default;
				auto operator==(const const_iterator& other) const->bool { return iter_ == other.iter_; }
				auto operator!=(const const_iterator& other) const->bool { return iter_ != other.iter_; }
				auto operator<(const const_iterator& other) const->bool { return iter_ < other.iter_; } //optional
				auto operator>(const const_iterator& other) const->bool { return iter_ > other.iter_; } //optional
				auto operator<=(const const_iterator& other) const->bool { return iter_ <= other.iter_; } //optional
				auto operator>=(const const_iterator& other) const->bool { return iter_ >= other.iter_; } //optional

				auto operator++()->const_iterator& { ++iter_; return *this; }
				auto operator++(int)->const_iterator { const_iterator ret(*this); operator++(); return ret; }  //optional
				auto operator--()->const_iterator& { --iter_; return *this; } //optional
				auto operator--(int)->const_iterator { const_iterator ret(*this); operator--(); return ret; } //optional
				auto operator+=(size_type size)->const_iterator& { iter_ += size; return *this; } //optional
				auto operator+(size_type size) const->const_iterator { return iter_ + size; } //optional
				friend auto operator+(size_type size, const const_iterator& iter)->const_iterator { return size + iter.iter_; } //optional
				auto operator-=(size_type size)->const_iterator& { iter_ -= size; return *this; } //optional
				auto operator-(size_type size) const->const_iterator { return iter_ - size; } //optional
				auto operator-(const_iterator iter) const->difference_type { return iter_ - iter.iter_; } //optional

				auto operator*() const->const_reference { return iter_->operator*(); }
				auto operator->() const->const_pointer { return iter_->operator->(); }
				auto operator[](size_type size) const->const_reference { return *iter_->operator[](size); } //optional

				~const_iterator() = default;
				const_iterator() = default;
				const_iterator(const const_iterator&) = default;
				const_iterator(const iterator& other) :iter_(other.iter_) {}
				const_iterator(const typename std::vector<ImpPtr<T>>::const_iterator iter) :iter_(iter) {} //

			private:
				typename std::vector<ImpPtr<T>>::const_iterator iter_;
			};
			using reverse_iterator = std::reverse_iterator<iterator>; //optional
			using const_reverse_iterator = std::reverse_iterator<const_iterator>; //optional
			
			auto size()const->size_type { return container_.size(); }
			auto max_size()->size_type { return container_.max_size(); }
			auto empty()->bool { return container_.empty(); }

			auto begin()->iterator { return container_.begin(); }
			auto begin() const->const_iterator { return container_.begin(); }
			auto cbegin() const->const_iterator { return container_.cbegin(); }
			auto end()->iterator { return container_.end(); }
			auto end() const->const_iterator { return container_.end(); }
			auto cend() const->const_iterator { return container_.cend(); }
			auto rbegin()->reverse_iterator { return container_.rbegin(); } //optional
			auto rbegin() const->const_reverse_iterator { return container_.rbegin(); }; //optional
			auto crbegin() const->const_reverse_iterator { return container_.crbegin(); }; //optional
			auto rend()->reverse_iterator { return container_.rend(); } //optional
			auto rend() const->const_reverse_iterator { return container_.rend(); } //optional
			auto crend() const->const_reverse_iterator { return container_.crend(); } //optional

			auto front()->reference { return *begin(); } //optional
			auto front() const->const_reference { return *begin(); } //optional
			auto back()->reference { return *(end() - 1); } //optional
			auto back() const->const_reference { return *(end() - 1); } //optional
			auto at(size_type size)->reference { return *container_.at(size); } //optional
			auto at(size_type size) const->const_reference { return *container_.at(size); } //optional
			auto operator[](size_type size)->reference { return *container_.operator[](size); } //optional
			auto operator[](size_type size) const->const_reference { return *container_.operator[](size); } //optional

			auto pop_back()->void { container_.pop_back(); } //optional
			auto erase(iterator iter)->iterator { return container_.erase(iter.iter_); } //optional
			auto erase(iterator begin_iter, iterator end_iter)->iterator { return container_.erase(begin_iter.iter_, end_iter.iter_); } //optional
			auto clear()->void { container_.clear(); } //optional
			
			auto push_back_ptr(T*ptr)->void { container_.push_back(ImpPtr<T>(ptr)); }
			auto swap(ImpContainer& other)->void { return container_.swap(other.container_); }

			~ImpContainer() = default;
			ImpContainer() = default;
			ImpContainer(const ImpContainer&) = default;
			ImpContainer(ImpContainer&&other) = default;
			ImpContainer& operator=(const ImpContainer& other) = default;
			ImpContainer& operator=(ImpContainer&& other) = default;

		private:
			typename std::vector<ImpPtr<T>> container_;
			friend class Object;
		};

		class Object
		{
		public:
			static auto attributeBool(const aris::core::XmlElement &xml_ele, const std::string &attribute_name)->bool;
			static auto attributeBool(const aris::core::XmlElement &xml_ele, const std::string &attribute_name, bool default_value)->bool;
			static auto attributeInt64(const aris::core::XmlElement &xml_ele, const std::string &attribute_name)->std::int64_t;
			static auto attributeInt64(const aris::core::XmlElement &xml_ele, const std::string &attribute_name, std::int64_t default_value)->std::int64_t;
			static auto attributeInt32(const aris::core::XmlElement &xml_ele, const std::string &attribute_name)->std::int32_t;
			static auto attributeInt32(const aris::core::XmlElement &xml_ele, const std::string &attribute_name, std::int32_t default_value)->std::int32_t;
			static auto attributeInt16(const aris::core::XmlElement &xml_ele, const std::string &attribute_name)->std::int16_t;
			static auto attributeInt16(const aris::core::XmlElement &xml_ele, const std::string &attribute_name, std::int16_t default_value)->std::int16_t;
			static auto attributeInt8(const aris::core::XmlElement &xml_ele, const std::string &attribute_name)->std::int8_t;
			static auto attributeInt8(const aris::core::XmlElement &xml_ele, const std::string &attribute_name, std::int8_t default_value)->std::int8_t;
			static auto attributeUint64(const aris::core::XmlElement &xml_ele, const std::string &attribute_name)->std::uint64_t;
			static auto attributeUint64(const aris::core::XmlElement &xml_ele, const std::string &attribute_name, std::uint64_t default_value)->std::uint64_t;
			static auto attributeUint32(const aris::core::XmlElement &xml_ele, const std::string &attribute_name)->std::uint32_t;
			static auto attributeUint32(const aris::core::XmlElement &xml_ele, const std::string &attribute_name, std::uint32_t default_value)->std::uint32_t;
			static auto attributeUint16(const aris::core::XmlElement &xml_ele, const std::string &attribute_name)->std::uint16_t;
			static auto attributeUint16(const aris::core::XmlElement &xml_ele, const std::string &attribute_name, std::uint16_t default_value)->std::uint16_t;
			static auto attributeUint8(const aris::core::XmlElement &xml_ele, const std::string &attribute_name)->std::uint8_t;
			static auto attributeUint8(const aris::core::XmlElement &xml_ele, const std::string &attribute_name, std::uint8_t default_value)->std::uint8_t;
			static auto attributeFloat(const aris::core::XmlElement &xml_ele, const std::string &attribute_name)->float;
			static auto attributeFloat(const aris::core::XmlElement &xml_ele, const std::string &attribute_name, float default_value)->float;
			static auto attributeDouble(const aris::core::XmlElement &xml_ele, const std::string &attribute_name)->double;
			static auto attributeDouble(const aris::core::XmlElement &xml_ele, const std::string &attribute_name, double default_value)->double;
			static auto attributeString(const aris::core::XmlElement &xml_ele, const std::string &attribute_name)->std::string;
			static auto attributeString(const aris::core::XmlElement &xml_ele, const std::string &attribute_name, const std::string &default_value)->std::string;
			static auto attributeChar(const aris::core::XmlElement &xml_ele, const std::string &attribute_name)->char;
			static auto attributeChar(const aris::core::XmlElement &xml_ele, const std::string &attribute_name, char default_value)->char;
			static auto Type()->const std::string &{ static const std::string type("Object"); return std::ref(type); }
			auto virtual type() const->const std::string&{ return Type(); }
			auto virtual saveXml(aris::core::XmlElement &xml_ele) const->void;
			auto xmlString()->std::string;
			auto name() const->const std::string&;
			auto id()const->std::size_t;
			auto root()->Root&;
			auto root()const->const Root&;
			auto father()const->const Object&;
			auto father()->Object&;
			auto children()const->const ImpContainer<Object>&;
			auto children()->ImpContainer<Object>&;
			auto save(const std::string &name, bool auto_override_save = true)->void;
			auto load(const std::string &name, bool auto_delete_save = true)->void;
			auto findByName(const std::string &name)const->ImpContainer<Object>::const_iterator;
			auto findByName(const std::string &name)->ImpContainer<Object>::iterator;
			template<typename T = Object>
			auto findType(const std::string &name)const->const T*{ return const_cast<Object*>(this)->findType<T>(); };
			template<typename T = Object>
			auto findType(const std::string &name)->T* { auto iter = findByName(name); return iter != children().end() && dynamic_cast<T*>(&*iter) ? dynamic_cast<T*>(&*iter) : nullptr; }
			template<typename T = Object, typename ...Args>
			auto findOrInsert(const std::string &name, Args&&... args)-> T* { auto p = findType<T>(name); return p ? p : &add<T>(name, std::forward<Args>(args)...); }
			auto add(Object *obj)->Object &;
			auto add(const aris::core::XmlElement &xml_ele)->Object &;
			template<typename T, typename ...Args>
			auto add(Args&&... args)->T& { return static_cast<T&>(add(new T(std::forward<Args>(args)...))); }
			

			virtual ~Object();
			explicit Object(const std::string &name = "object");
			explicit Object(Object &father, const aris::core::XmlElement &xml_ele);
			Object(const Object &);
			Object(Object &&);
			Object& operator=(const Object &);
			Object& operator=(Object &&);

		private:
			struct Imp;
			ImpPtr<Imp> imp_;
			friend class Root;
		};
		class Root :public Object
		{
		public:
			struct TypeInfo
			{
				std::function<Object*(Object &father, const aris::core::XmlElement &xml_ele)> xml_construct_func;
				std::function<Object*(const Object &from_object)> copy_construct_func;
				std::function<Object*(Object &&from_object)> move_construct_func;
				std::function<Object&(const Object &from_object, Object &to_object)> copy_assign_func;
				std::function<Object&(Object &&from_object, Object &to_object)> move_assign_func;

				auto registerTo(const std::string &type, Root &object)->void;
				template<typename ChildType> static auto CreateTypeInfo()->TypeInfo
				{
					static_assert(std::is_base_of<Object, ChildType>::value, "failed to register type, because it is not inheritated from Object");

					TypeInfo info;
					info.xml_construct_func = [](Object &father, const aris::core::XmlElement &xml_ele)->Object* {return new ChildType(father, xml_ele); };
					info.copy_construct_func = CopyConstruct<ChildType, is_copy_constructible<ChildType>()>::func();
					info.move_construct_func = MoveConstruct<ChildType, is_copy_constructible<ChildType>(), is_move_constructible<ChildType>()>::func();
					info.copy_assign_func = CopyAssign<ChildType, is_copy_assignable<ChildType>()>::func();
					info.move_assign_func = MoveAssign<ChildType, is_copy_assignable<ChildType>(), TypeInfo::is_move_assignable<ChildType>()>::func();

					return info;
				}

			private:
				struct general {};
				struct special :public general {};
				template<typename T> static constexpr auto is_copy_constructible_(general)-> bool { return false; }
				template<typename T> static constexpr auto is_copy_constructible_(special, typename std::decay<decltype(T(*static_cast<const T*>(nullptr)))>* = nullptr)-> bool { return true; }
				template<typename T> static constexpr auto is_copy_constructible()-> bool { return is_copy_constructible_<T>(special()); }
				
				template<typename T> static constexpr auto is_move_constructible_(general)-> bool { return false; }
				template<typename T> static constexpr auto is_move_constructible_(special, typename std::decay<decltype(T(std::move(*static_cast<T*>(nullptr))))>* = nullptr)-> bool { return true; }
				template<typename T> static constexpr auto is_move_constructible()-> bool { return is_move_constructible_<T>(special()); }

				template<typename T> static constexpr auto is_copy_assignable_(general)-> bool { return false; }
				template<typename T> static constexpr auto is_copy_assignable_(special, typename std::decay<decltype(static_cast<T*>(nullptr)->operator=(*static_cast<const T*>(nullptr)))>::type* = nullptr)-> bool { return true; }
				template<typename T> static constexpr auto is_copy_assignable()-> bool { return is_copy_assignable_<T>(special()); }

				template<typename T> static constexpr auto is_move_assignable_(general)-> bool { return false; }
				template<typename T> static constexpr auto is_move_assignable_(special, typename std::decay<decltype(static_cast<T*>(nullptr)->operator=(std::move(*static_cast<T*>(nullptr))))>::type* = nullptr)-> bool { return true; }
				template<typename T> static constexpr auto is_move_assignable()-> bool { return is_move_assignable_<T>(special()); }

				template<typename ChildType, bool> struct CopyConstruct
				{
					static auto func()->decltype(copy_construct_func) 
					{
						return[](const Object &other)->Object*
						{
							if (!dynamic_cast<const ChildType *>(&other))throw std::runtime_error("can't create type \"" + ChildType::Type() + "\" because object is not the same type");
							return new ChildType(static_cast<const ChildType &>(other));
						};
					}
				};
				template<typename ChildType> struct CopyConstruct<ChildType, false>
				{
					static auto func()->decltype(copy_construct_func) { return nullptr; }
				};

				template<typename ChildType, bool is_copy_constructible, bool is_move_constructible> struct MoveConstruct
				{
					static auto func()->decltype(move_construct_func)
					{
						return[](Object &&other)->Object*
						{
							if (!dynamic_cast<ChildType *>(&other))throw std::runtime_error("can't create type \"" + ChildType::Type() + "\" because object is not the same type");
							return new ChildType(static_cast<ChildType &&>(other));
						};
					}
				};
				template<typename ChildType, bool is_copy_constructible> struct MoveConstruct<ChildType, is_copy_constructible, false>
				{
					static auto func()->decltype(move_construct_func)
					{
						return[](Object &&other)->Object*
						{
							if (!dynamic_cast<ChildType *>(&other))throw std::runtime_error("can't create type \"" + ChildType::Type() + "\" because object is not the same type");
							return new ChildType(static_cast<const ChildType &>(other));
						};
					}
				};
				template<typename ChildType> struct MoveConstruct<ChildType, false, false>
				{
					static auto func()->decltype(move_construct_func) { return nullptr; }
				};

				template<typename ChildType, bool> struct CopyAssign
				{
					static auto func()->decltype(copy_assign_func)
					{
						return[](const Object &from_object, Object &to_object)->Object&
						{
							if (!dynamic_cast<const ChildType *>(&from_object))throw std::runtime_error("can't assign type \"" + ChildType::Type() + "\" because object is not the same type");
							if (!dynamic_cast<ChildType *>(&to_object))throw std::runtime_error("can't assign type \"" + ChildType::Type() + "\" because object is not the same type");
							return static_cast<ChildType &>(to_object) = static_cast<const ChildType &>(from_object);
						};
					}
				};
				template<typename ChildType> struct CopyAssign<ChildType, false>
				{
					static auto func()->decltype(copy_assign_func) { return nullptr; }
				};

				template<typename ChildType, bool is_copy_assignable, bool is_move_assignable> struct MoveAssign
				{
					static auto func()->decltype(move_assign_func)
					{
						return[](Object &&from_object, Object &to_object)->Object&
						{
							if (!dynamic_cast<ChildType *>(&from_object))throw std::runtime_error("can't assign type \"" + ChildType::Type() + "\" because object is not the same type");
							if (!dynamic_cast<ChildType *>(&to_object))throw std::runtime_error("can't assign type \"" + ChildType::Type() + "\" because object is not the same type");
							return static_cast<ChildType &>(to_object) = static_cast<ChildType &&>(from_object);
						};
					}
				};
				template<typename ChildType, bool is_copy_assignable> struct MoveAssign<ChildType, is_copy_assignable, false>
				{
					static auto func()->decltype(move_assign_func)
					{
						return[](Object &&from_object, Object &to_object)->Object&
						{
							if (!dynamic_cast<ChildType *>(&from_object))throw std::runtime_error("can't assign type \"" + ChildType::Type() + "\" because object is not the same type");
							if (!dynamic_cast<ChildType *>(&to_object))throw std::runtime_error("can't assign type \"" + ChildType::Type() + "\" because object is not the same type");
							return static_cast<ChildType &>(to_object) = static_cast<const ChildType &>(from_object);
						};
					}
				};
				template<typename ChildType> struct MoveAssign<ChildType, false, false>
				{
					static auto func()->decltype(move_assign_func) { return nullptr; }
				};
			};
			using Object::saveXml;
			static auto Type()->const std::string &{ static const std::string type("Root"); return std::ref(type); }
			auto virtual type() const->const std::string&{ return Type(); }
			auto virtual loadXml(const char* filename)->void { loadXml(std::string(filename)); }
			auto virtual loadXml(const std::string &filename)->void;
			auto virtual loadXml(const aris::core::XmlDocument &xml_doc)->void;
			auto virtual loadXml(const aris::core::XmlElement &xml_ele)->void;
			auto virtual saveXml(const char *filename) const->void { saveXml(std::string(filename)); }
			auto virtual saveXml(const std::string &filename) const->void;
			auto virtual saveXml(aris::core::XmlDocument &xml_doc)const->void;
			template<typename ChildType>
			auto registerChildType()->void { TypeInfo::CreateTypeInfo<ChildType>().registerTo(ChildType::Type(), *this); }
			auto childTypeMap()const ->const std::map<std::string, TypeInfo>&;
			auto loadString(const std::string &xml_src)->void { aris::core::XmlDocument xml_doc; xml_doc.Parse(xml_src.c_str()); loadXml(xml_doc); };
			auto save(const std::string &name, bool auto_override_save = true)->void;
			auto load(const std::string &name, bool auto_delete_save = true)->void;

			virtual ~Root();
			explicit Root(const std::string &name = "Root");
			explicit Root(const aris::core::XmlElement &xml_ele);
			Root(const Root&);
			Root(Root&&);
			Root& operator=(const Root&);
			Root& operator=(Root&&);

		private:
			struct Imp;
			ImpPtr<Imp> imp_;
			
			friend class Object;
		};

		template <class T, class Base = Object> class ObjectPool: public Base
		{
		public:
			static_assert(std::is_base_of<Object, Base>::value, "template param \"Base\" of \"ObjectPool\" must be derived class of \"Object\"");

			using value_type = T;
			using reference = T&;
			using const_reference = const T&;
			using pointer = T*;
			using const_pointer = const T*;
			using difference_type = typename ImpContainer<Object>::difference_type;
			using size_type = typename ImpContainer<Object>::size_type;

			class iterator
			{
			public:
				using difference_type = typename ObjectPool::difference_type;
				using value_type = typename ObjectPool::value_type;
				using reference = typename ObjectPool::reference;
				using pointer = typename ObjectPool::pointer;
				using iterator_category = std::random_access_iterator_tag; //or another tag

				auto operator=(const iterator&other)->iterator& = default;
				auto operator==(const iterator&other) const->bool { return iter_ == other.iter_; }
				auto operator!=(const iterator&other) const->bool { return iter_ != other.iter_; }
				auto operator<(const iterator&other) const->bool { return iter_ < other.iter_; } //optional
				auto operator>(const iterator&other) const->bool { return iter_ > other.iter_; } //optional
				auto operator<=(const iterator&other) const->bool { return iter_ <= other.iter_; } //optional
				auto operator>=(const iterator&other) const->bool { return iter_ >= other.iter_; } //optional

				auto operator++()->iterator& { ++iter_; return *this; }
				auto operator++(int)->iterator { iterator ret(*this); operator++(); return ret; } //optional
				auto operator--()->iterator& { --iter_; return *this; } //optional
				auto operator--(int)->iterator { iterator ret(*this); operator--(); return ret; } //optional
				auto operator+=(size_type size)->iterator& { iter_ += size; return *this; } //optional
				auto operator+(size_type size) const->iterator { return iterator(iter_ + size); } //optional
				friend auto operator+(size_type size, const iterator&iter)->iterator { return size + iter.iter_; } //optional
				auto operator-=(size_type size)->iterator& { iter_ -= size; return *this; } //optional
				auto operator-(size_type size) const->iterator { return iterator(iter_ - size); } //optional
				auto operator-(iterator iter) const->difference_type { return iter_ - iter.iter_; } //optional

				auto operator*() const->reference { return static_cast<reference>(iter_.operator*());}
				auto operator->() const->pointer { return static_cast<pointer>(iter_.operator->());}
				auto operator[](size_type size) const->reference { return *iterator(this->iter_->operator[](size)); } //optional

				~iterator() = default;
				iterator() = default;
				iterator(const iterator& other) = default;
				iterator(typename ImpContainer<Object>::iterator iter) :iter_(iter) {} // 自己添加的

			private:
				typename ImpContainer<Object>::iterator iter_;
				friend class ObjectPool::const_iterator;
			};
			class const_iterator
			{
			public:
				using difference_type = typename ObjectPool::difference_type ;
				using value_type = typename ObjectPool::value_type ;
				using const_reference = typename ObjectPool::const_reference ;
				using const_pointer = typename ObjectPool::const_pointer ;
				using iterator_category = std::random_access_iterator_tag ; //or another tag

				auto operator=(const const_iterator&)->const_iterator& = default;
				auto operator==(const const_iterator& other) const->bool { return iter_ == other.iter_; }
				auto operator!=(const const_iterator& other) const->bool { return iter_ != other.iter_; }
				auto operator<(const const_iterator& other) const->bool { return iter_ < other.iter_; } //optional
				auto operator>(const const_iterator& other) const->bool { return iter_ > other.iter_; } //optional
				auto operator<=(const const_iterator& other) const->bool { return iter_ <= other.iter_; } //optional
				auto operator>=(const const_iterator& other) const->bool { return iter_ >= other.iter_; } //optional

				auto operator++()->const_iterator& { ++iter_; return *this; }
				auto operator++(int)->const_iterator { const_iterator ret(*this); operator++(); return ret; }  //optional
				auto operator--()->const_iterator& { --iter_; return *this; } //optional
				auto operator--(int)->const_iterator { const_iterator ret(*this); operator--(); return ret; } //optional
				auto operator+=(size_type size)->const_iterator& { iter_ += size; return *this; } //optional
				auto operator+(size_type size) const->const_iterator { return const_iterator(iter_ + size); } //optional
				friend auto operator+(size_type size, const const_iterator& iter)->const_iterator { return const_iterator(size + iter); } //optional
				auto operator-=(size_type size)->const_iterator& { iter_ -= size; return *this; } //optional
				auto operator-(size_type size) const->const_iterator { return const_iterator(iter_ - size); } //optional
				auto operator-(const_iterator iter) const->difference_type { return iter_ - iter.iter_; } //optional

				auto operator*() const->const_reference { return static_cast<const_reference>(iter_.operator*()); }
				auto operator->() const->const_pointer { return static_cast<const_pointer>(iter_.operator->()); }
				auto operator[](size_type size) const->const_reference { return *const_iterator(this->iter_->operator[](size)); } //optional

				~const_iterator() = default;
				const_iterator() = default;
				const_iterator(const const_iterator&) = default;
				const_iterator(const iterator& other) :iter_(other.iter_) {}
				const_iterator(typename ImpContainer<Object>::const_iterator iter) :iter_(iter) {} // 自己添加的

			private:
				typename ImpContainer<Object>::const_iterator iter_;
			};
			using reverse_iterator = std::reverse_iterator<iterator>; //optional
			using const_reverse_iterator = std::reverse_iterator<const_iterator>; //optional

			auto size()const->size_type { return Base::children().size(); }
			auto max_size()->size_type { return Base::children().max_size(); }
			auto empty()->bool { return Base::children().empty(); }

			auto begin()->iterator { return Base::children().begin(); }
			auto begin()const->const_iterator { return Base::children().begin(); }
			auto cbegin() const->const_iterator { return Base::children().cbegin(); }
			auto end()->iterator { return Base::children().end(); }
			auto end()const->const_iterator { return Base::children().end(); }
			auto cend() const->const_iterator { return Base::children().cend(); }
			auto rbegin()->reverse_iterator { return Base::children().rbegin(); } //optional
			auto rbegin() const->const_reverse_iterator { return Base::children().rbegin(); }; //optional
			auto crbegin() const->const_reverse_iterator { return Base::children().crbegin(); }; //optional
			auto rend()->reverse_iterator { return Base::children().rend(); } //optional
			auto rend() const->const_reverse_iterator { return Base::children().rend(); } //optional
			auto crend() const->const_reverse_iterator { return Base::children().crend(); } //optional

			auto front()->reference { return *begin(); } //optional
			auto front() const->const_reference { return *begin(); } //optional
			auto back()->reference { return *(end() - 1); } //optional
			auto back() const->const_reference { return *(end() - 1); } //optional
			auto at(std::size_t id) const->const_reference { return static_cast<const_reference>(Base::children().at(id)); }
			auto at(std::size_t id)->reference { return static_cast<reference>(Base::children().at(id)); }
			auto operator[](size_type size)->reference { return static_cast<reference>(Base::children().operator[](size)); } //optional
			auto operator[](size_type size) const->const_reference { return static_cast<const_reference>(Base::children().operator[](size)); } //optional

			auto pop_back()->void { Base::children().pop_back(); } //optional
			auto erase(iterator iter)->iterator { return Base::children().erase(iter.iter_); } //optional
			auto erase(iterator begin_iter, iterator end_iter)->iterator { return Base::children().erase(begin_iter.iter_, end_iter.iter_); } //optional
			auto clear()->void { Base::children().clear(); } //optional

			static auto Type()->const std::string &	{ 
				static const std::string type{ (&Type == &T::Type ? std::string("Noname") : T::Type()) + "Pool" + (&Type == &Base::Type ? std::string("Noname") : Base::Type()) };
				return type;
			}
			auto virtual type()const->const std::string & override{ return Type(); }
			auto findByName(const std::string &name)const->const_iterator { return Base::findByName(name); }
			auto findByName(const std::string &name)->iterator { return Base::findByName(name);}

			virtual ~ObjectPool() = default;

		protected:
			explicit ObjectPool(const std::string &name):Base(name) {}
			explicit ObjectPool(Object &father, const aris::core::XmlElement &xml_ele) :Base(father, xml_ele) {}
		
		private:
			friend class Object;
			friend class Root;
		};
		template <class T> class RefPool
		{
		public:
			using value_type = T;
			using reference = T&;
			using const_reference = const T&;
			using pointer = T*;
			using const_pointer = const T*;
			using difference_type = std::size_t;
			using size_type = std::size_t;

			class iterator
			{
			public:
				using difference_type = typename RefPool::difference_type;
				using value_type = typename RefPool::value_type;
				using reference = typename RefPool::reference;
				using pointer = typename RefPool::pointer;
				using iterator_category = std::random_access_iterator_tag; //or another tag

				auto operator=(const iterator&other)->iterator& = default;
				auto operator==(const iterator&other) const->bool { return iter_ == other.iter_; }
				auto operator!=(const iterator&other) const->bool { return iter_ != other.iter_; }
				auto operator<(const iterator&other) const->bool { return iter_ < other.iter_; } //optional
				auto operator>(const iterator&other) const->bool { return iter_ > other.iter_; } //optional
				auto operator<=(const iterator&other) const->bool { return iter_ <= other.iter_; } //optional
				auto operator>=(const iterator&other) const->bool { return iter_ >= other.iter_; } //optional

				auto operator++()->iterator& { ++iter_; return *this; }
				auto operator++(int)->iterator { iterator ret(*this); operator++(); return ret; } //optional
				auto operator--()->iterator& { --iter_; return *this; } //optional
				auto operator--(int)->iterator { iterator ret(*this); operator--(); return ret; } //optional
				auto operator+=(size_type size)->iterator& { iter_ += size; return *this; } //optional
				auto operator+(size_type size) const->iterator { return iterator(iter_ + size); } //optional
				friend auto operator+(size_type size, const iterator&iter)->iterator { return size + iter.iter_; } //optional
				auto operator-=(size_type size)->iterator& { iter_ -= size; return *this; } //optional
				auto operator-(size_type size) const->iterator { return iterator(iter_ - size); } //optional
				auto operator-(iterator iter) const->difference_type { return iter_ - iter.iter_; } //optional

				auto operator*() const->reference { return std::ref(**iter_); }
				auto operator->() const->pointer { return *iter_; }
				auto operator[](size_type size) const->reference { return *iter_->operator[](size); } //optional

				~iterator() = default;
				iterator() = default;
				iterator(const iterator& other) = default;
				iterator(typename std::vector<T*>::iterator iter) :iter_(iter) {} // 自己添加的

			private:
				typename std::vector<T*>::iterator iter_;
				friend class RefPool::const_iterator;
			};
			class const_iterator
			{
			public:
				using difference_type = typename RefPool::difference_type;
				using value_type = typename RefPool::value_type;
				using const_reference = typename RefPool::const_reference;
				using const_pointer = typename RefPool::const_pointer;
				using iterator_category = std::random_access_iterator_tag; //or another tag

				auto operator=(const const_iterator&)->const_iterator& = default;
				auto operator==(const const_iterator& other) const->bool { return iter_ == other.iter_; }
				auto operator!=(const const_iterator& other) const->bool { return iter_ != other.iter_; }
				auto operator<(const const_iterator& other) const->bool { return iter_ < other.iter_; } //optional
				auto operator>(const const_iterator& other) const->bool { return iter_ > other.iter_; } //optional
				auto operator<=(const const_iterator& other) const->bool { return iter_ <= other.iter_; } //optional
				auto operator>=(const const_iterator& other) const->bool { return iter_ >= other.iter_; } //optional

				auto operator++()->const_iterator& { ++iter_; return *this; }
				auto operator++(int)->const_iterator { const_iterator ret(*this); operator++(); return ret; }  //optional
				auto operator--()->const_iterator& { --iter_; return *this; } //optional
				auto operator--(int)->const_iterator { const_iterator ret(*this); operator--(); return ret; } //optional
				auto operator+=(size_type size)->const_iterator& { iter_ += size; return *this; } //optional
				auto operator+(size_type size) const->const_iterator { return const_iterator(iter_ + size); } //optional
				friend auto operator+(size_type size, const const_iterator& iter)->const_iterator { return const_iterator(size + iter); } //optional
				auto operator-=(size_type size)->const_iterator& { iter_ -= size; return *this; } //optional
				auto operator-(size_type size) const->const_iterator { return const_iterator(iter_ - size); } //optional
				auto operator-(const_iterator iter) const->difference_type { return iter_ - iter.iter_; } //optional

				auto operator*() const->const_reference { return **iter_; }
				auto operator->() const->const_pointer { return *iter_; }
				auto operator[](size_type size) const->const_reference { return *iter_->operator[](size); } //optional

				~const_iterator() = default;
				const_iterator() = default;
				const_iterator(const const_iterator&) = default;
				const_iterator(const iterator& other) :iter_(other.iter_) {}
				const_iterator(typename std::vector<T*>::const_iterator iter) :iter_(iter) {} // 自己添加的
				
			private:
				typename std::vector<T*>::const_iterator iter_;
			};
			using reverse_iterator = std::reverse_iterator<iterator>; //optional
			using const_reverse_iterator = std::reverse_iterator<const_iterator>; //optional

			auto swap(RefPool& other)->void { return container_.swap(other.container_); }
			auto size()const->size_type { return container_.size(); }
			auto max_size()->size_type { return container_.max_size(); }
			auto empty()->bool { return container_.empty(); }
			auto begin()->iterator { return container_.begin(); }
			auto begin()const->const_iterator { return container_.begin(); }
			auto cbegin() const->const_iterator { return container_.cbegin(); }
			auto end()->iterator { return container_.end(); }
			auto end()const->const_iterator { return container_.end(); }
			auto cend() const->const_iterator { return container_.cend(); }
			auto rbegin()->reverse_iterator { return container_.rbegin(); } //optional
			auto rbegin() const->const_reverse_iterator { return container_.rbegin(); } //optional
			auto crbegin() const->const_reverse_iterator { return container_.crbegin(); } //optional
			auto rend()->reverse_iterator { return container_.rend(); } //optional
			auto rend() const->const_reverse_iterator { return container_.rend(); } //optional
			auto crend() const->const_reverse_iterator { return container_.crend(); } //optional
			auto front()->reference { return *begin(); } //optional
			auto front() const->const_reference { return *begin(); } //optional
			auto back()->reference { return *(end() - 1); } //optional
			auto back() const->const_reference { return *(end() - 1); } //optional
			auto at(std::size_t id) const->const_reference { return static_cast<const_reference>(*container_.at(id)); }
			auto at(std::size_t id)->reference { return static_cast<reference>(*container_.at(id)); }
			auto operator[](size_type size)->reference { return static_cast<reference>(container_.operator[](size)); } //optional
			auto operator[](size_type size) const->const_reference { return static_cast<const_reference>(container_.operator[](size)); } //optional
			auto pop_back()->void { container_.pop_back(); } //optional
			auto erase(iterator iter)->iterator { return container_.erase(iter.iter_); } //optional
			auto erase(iterator begin_iter, iterator end_iter)->iterator { return container_.erase(begin_iter.iter_, end_iter.iter_); } //optional
			auto clear()->void { container_.clear(); } //optional
			auto findByName(const std::string &name)const->const_iterator { return std::find_if(begin(), end(), [&name, this](T &p) {return (p.name() == name); }); }
			auto findByName(const std::string &name)->iterator { return std::find_if(begin(), end(), [&name, this](T &p) {return (p.name() == name); }); }

			auto push_back_ptr(T*ptr)->void { container_.push_back(ptr); }
			template<class iter>
			auto push_back_ptr(iter &begin, iter &end)->void { for (auto i = begin; i != end; ++i)push_back_ptr(&(*begin)); }

		private:
			std::vector<T*> container_;
		};
	}
}

#endif
