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

namespace aris
{
	namespace core
	{
		typedef tinyxml2::XMLDocument XmlDocument;
		typedef tinyxml2::XMLDeclaration XmlDeclaration;
		typedef tinyxml2::XMLNode XmlNode;
		typedef tinyxml2::XMLElement XmlElement;
		typedef tinyxml2::XMLAttribute XmlAttribute;
		class Root;

		template<typename Data> class ImpPtr
		{
		public:
			auto get()const->const Data*{ return data_unique_ptr_.get(); }
			auto get()->Data* { return data_unique_ptr_.get(); }
			auto operator->()const->const Data*{ return data_unique_ptr_.get(); }
			auto operator->()->Data*{ return data_unique_ptr_.get(); }
			auto operator*()const->const Data& { return *data_unique_ptr_; }
			auto operator*()->Data&{ return *data_unique_ptr_; }
			auto operator=(const ImpPtr &other)->ImpPtr& { *data_unique_ptr_ = *other.data_unique_ptr_; return *this; }
			auto operator=(ImpPtr &&other)noexcept->ImpPtr& { *data_unique_ptr_ = std::move(*other.data_unique_ptr_); return *this; }
			~ImpPtr() = default;
			ImpPtr() :data_unique_ptr_(new Data) {}
			ImpPtr(Data *data_ptr) :data_unique_ptr_(data_ptr) {}
			ImpPtr(const ImpPtr &other) :data_unique_ptr_(new Data(*other.data_unique_ptr_)) {}
			ImpPtr(ImpPtr &&other)noexcept :data_unique_ptr_(std::move(other.data_unique_ptr_)) {}
			
		private:
			std::unique_ptr<Data> data_unique_ptr_;
		};
		template <class T, class A = std::allocator<T> >class ImpContainer 
		{
		public:
			typedef A allocator_type;
			typedef typename A::value_type value_type;
			typedef typename A::reference reference;
			typedef typename A::const_reference const_reference;
			typedef typename A::pointer pointer;
			typedef typename A::const_pointer const_pointer;
			typedef typename A::difference_type difference_type;
			typedef typename A::size_type size_type;

			class iterator
			{
			public:
				typedef typename A::difference_type difference_type;
				typedef typename A::value_type value_type;
				typedef typename A::reference reference;
				typedef typename A::pointer pointer;
				typedef std::random_access_iterator_tag iterator_category; //or another tag

				iterator() = default;
				iterator(const iterator& other) = default;
				iterator(const typename std::vector<ImpPtr<T>>::iterator iter):iter_(iter) {} // 自己添加的
				~iterator() = default;

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

				typename std::vector<ImpPtr<T>>::iterator iter_;
			};
			class const_iterator 
			{
			public:
				typedef typename A::difference_type difference_type;
				typedef typename A::value_type value_type;
				typedef typename A::const_reference const_reference;
				typedef typename A::const_pointer const_pointer;
				typedef std::random_access_iterator_tag iterator_category; //or another tag

				const_iterator() = default;
				const_iterator(const const_iterator&) = default;
				const_iterator(const iterator& other):iter_(other.iter_) {}
				const_iterator(const typename std::vector<ImpPtr<T>>::const_iterator iter) :iter_(iter) {} // 自己添加的
				~const_iterator() = default;

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

				typename std::vector<ImpPtr<T>>::const_iterator iter_;
			};
			typedef std::reverse_iterator<iterator> reverse_iterator; //optional
			typedef std::reverse_iterator<const_iterator> const_reverse_iterator; //optional
			
			auto swap(ImpContainer& other)->void { return container_.swap(other.container_); }
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

			auto operator=(const ImpContainer& other)->ImpContainer& { container_ = other.container_; }
			auto operator=(ImpContainer&& other)->ImpContainer& { container_ = std::move(other.container_); }
			
			~ImpContainer() = default;
			ImpContainer() = default;
			ImpContainer(const ImpContainer&) = default;
			ImpContainer(ImpContainer&&other) = default;

		private:
			typename std::vector<ImpPtr<T>> container_;
		};

		class Object: public ImpContainer<Object>
		{
		public:
			static auto Type()->const std::string &{ static const std::string type("object"); return std::ref(type); }
			virtual auto type() const->const std::string&{ return Type(); }
			virtual auto saveXml(aris::core::XmlElement &xml_ele) const->void;
			auto name() const->const std::string&;
			auto id()const->std::size_t;
			auto root()->Root&;
			auto root()const->const Root&;
			auto father()const->const Object&;
			auto father()->Object&;
			auto save(const std::string &name, bool auto_override_save = true)->void;
			auto load(const std::string &name, bool auto_delete_save = true)->void;
			auto findByName(const std::string &name)const->const_iterator;
			auto findByName(const std::string &name)->iterator;
			auto add(const aris::core::XmlElement &xml_ele)->Object &;
			auto add(const Object &object)->Object &;
			auto add(Object &&object)->Object &;
			template<typename T>
			auto add(const aris::core::XmlElement &xml_ele)->T& { return static_cast<T&>(add(T(*this, size(), xml_ele))); }
			template<typename T, typename ...Args>
			auto add(const std::string &name, Args... args)->T& { return static_cast<T&>(add(T(*this, size(), name, args...))); }

			auto operator=(const Object &)->Object &;
			auto operator=(Object &&)->Object &;
			virtual ~Object();
			
		protected:
			Object(const Object &);
			Object(Object &&);
			Object(Object &father, std::size_t id, const std::string &name);
			Object(Object &father, std::size_t id, const aris::core::XmlElement &xml_ele);

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
				std::function<Object*(Object &father, std::size_t id, const aris::core::XmlElement &xml_ele)> newFromXml;
				std::function<Object*(const Object &from_object)> newFromObject;
				std::function<Object*(Object &&from_object)> newFromObjectR;
				std::function<Object&(const Object &from_object, Object &to_object)> assign;
				std::function<Object&(Object &&from_object, Object &to_object)> assignR;

				auto registerTo(const std::string &type, Root &object)->void;
				template<typename ChildType, bool is_copyable = true, bool is_moveable = true, bool is_assignable = true, bool is_move_assignable = true> static auto CreateTypeInfo()->TypeInfo
				{
					static_assert(std::is_base_of<Object, ChildType>::value, "failed to register type, because it is not inheritated from Object");

					TypeInfo info;
					info.newFromXml = [](Object &father, std::size_t id, const aris::core::XmlElement &xml_ele)->Object*
					{
						return new ChildType(father, id, xml_ele);
					};
					info.newFromObject = newFromObjectStruct<ChildType, is_copyable>::func();
					info.newFromObjectR = newFromObjectRStruct<ChildType, is_copyable, is_moveable>::func();
					info.assign = assignStruct<ChildType, is_assignable>::func();
					info.assignR = assignRStruct<ChildType, is_assignable, is_move_assignable>::func();

					return info;
				}

			private:
				static auto alignChildID(Object &father, Object &child, std::size_t id)->void;

				template<typename ChildType, bool> struct newFromObjectStruct
				{
					static auto func()->decltype(newFromObject) 
					{
						return[](const Object &other)->Object*
						{
							if (!dynamic_cast<const ChildType *>(&other))throw std::runtime_error("can't create type \"" + ChildType::Type() + "\" because object is not the same type");
							return new ChildType(static_cast<const ChildType &>(other));
						};
					}
				};
				template<typename ChildType> struct newFromObjectStruct<ChildType, false>
				{
					static auto func()->decltype(newFromObject) { return nullptr; }
				};

				template<typename ChildType, bool is_copyable, bool is_moveable> struct newFromObjectRStruct
				{
					static auto func()->decltype(newFromObjectR)
					{
						return[](Object &&other)->Object*
						{
							if (!dynamic_cast<ChildType *>(&other))throw std::runtime_error("can't create type \"" + ChildType::Type() + "\" because object is not the same type");
							return new ChildType(static_cast<ChildType &&>(other));
						};
					}
				};
				template<typename ChildType, bool is_copyable> struct newFromObjectRStruct<ChildType, is_copyable, false>
				{
					static auto func()->decltype(newFromObjectR)
					{
						return[](Object &&other)->Object*
						{
							if (!dynamic_cast<ChildType *>(&other))throw std::runtime_error("can't create type \"" + ChildType::Type() + "\" because object is not the same type");
							return new ChildType(static_cast<const ChildType &>(other));
						};
					}
				};
				template<typename ChildType> struct newFromObjectRStruct<ChildType, false, false>
				{
					static auto func()->decltype(newFromObjectR) { return nullptr; }
				};

				template<typename ChildType, bool> struct assignStruct
				{
					static auto func()->decltype(assign)
					{
						return[](const Object &from_object, Object &to_object)->Object&
						{
							if (!dynamic_cast<const ChildType *>(&from_object))throw std::runtime_error("can't assign type \"" + ChildType::Type() + "\" because object is not the same type");
							if (!dynamic_cast<ChildType *>(&to_object))throw std::runtime_error("can't assign type \"" + ChildType::Type() + "\" because object is not the same type");
							return static_cast<ChildType &>(to_object) = static_cast<const ChildType &>(from_object);
						};
					}
				};
				template<typename ChildType> struct assignStruct<ChildType, false>
				{
					static auto func()->decltype(assign) { return nullptr; }
				};

				template<typename ChildType, bool is_assignable, bool is_move_assignable> struct assignRStruct
				{
					static auto func()->decltype(assignR)
					{
						return[](Object &&from_object, Object &to_object)->Object&
						{
							if (!dynamic_cast<ChildType *>(&from_object))throw std::runtime_error("can't assign type \"" + ChildType::Type() + "\" because object is not the same type");
							if (!dynamic_cast<ChildType *>(&to_object))throw std::runtime_error("can't assign type \"" + ChildType::Type() + "\" because object is not the same type");
							return static_cast<ChildType &>(to_object) = static_cast<ChildType &&>(from_object);
						};
					}
				};
				template<typename ChildType, bool is_assignable> struct assignRStruct<ChildType, is_assignable, false>
				{
					static auto func()->decltype(assignR)
					{
						return[](Object &&from_object, Object &to_object)->Object&
						{
							if (!dynamic_cast<ChildType *>(&from_object))throw std::runtime_error("can't assign type \"" + ChildType::Type() + "\" because object is not the same type");
							if (!dynamic_cast<ChildType *>(&to_object))throw std::runtime_error("can't assign type \"" + ChildType::Type() + "\" because object is not the same type");
							return static_cast<ChildType &>(to_object) = static_cast<const ChildType &>(from_object);
						};
					}
				};
				template<typename ChildType> struct assignRStruct<ChildType, false, false>
				{
					static auto func()->decltype(assignR) { return nullptr; }
				};

			};
			using Object::saveXml;
			template<typename ChildType, bool is_copyable = true, bool is_moveable = true, bool is_assignable = true, bool is_move_assignable = true> 
			auto registerChildType()->void { TypeInfo::CreateTypeInfo<ChildType, is_copyable, is_moveable, is_assignable, is_move_assignable>().registerTo(ChildType::Type(), *this); }
			static auto Type()->const std::string &{ static const std::string type("root"); return std::ref(type); }
			virtual auto type() const->const std::string&{ return Type(); }
			virtual auto loadXml(const char* filename)->void { loadXml(std::string(filename)); }
			virtual auto loadXml(const std::string &filename)->void;
			virtual auto loadXml(const aris::core::XmlDocument &xml_doc)->void;
			virtual auto loadXml(const aris::core::XmlElement &xml_ele)->void;
			virtual auto saveXml(const char *filename) const->void { saveXml(std::string(filename)); }
			virtual auto saveXml(const std::string &filename) const->void;
			virtual auto saveXml(aris::core::XmlDocument &xml_doc)const->void;
			auto childTypeMap()const ->const std::map<std::string, TypeInfo>&;

			virtual ~Root();
			Root(const std::string &name = "Root");
			Root(const aris::core::XmlElement &xml_ele);
		private:
			struct Imp;
			ImpPtr<Imp> imp_;
			
			friend class Object;
		};

		template <class T, class Base = Object> class ObjectPool: public Base
		{
		public:
			static_assert(std::is_base_of<Object, Base>::value, "template param \"Base\" of \"ObjectPool\" must be derived class of \"Object\"");

			typedef T value_type;
			typedef T& reference;
			typedef const T& const_reference;
			typedef T* pointer;
			typedef const T* const_pointer;
			typedef typename Base::size_type size_type;

			class iterator
			{
			public:
				typedef typename ObjectPool::difference_type difference_type;
				typedef typename ObjectPool::value_type value_type;
				typedef typename ObjectPool::reference reference;
				typedef typename ObjectPool::pointer pointer;
				typedef std::random_access_iterator_tag iterator_category; //or another tag

				iterator() = default;
				iterator(const iterator& other) = default;
				iterator(typename Base::iterator iter) :iter_(iter) {} // 自己添加的
				~iterator() = default;

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
				auto operator[](size_type size) const->reference { return *iter_->operator[](size); } //optional

				typename Base::iterator iter_;
			};
			class const_iterator
			{
			public:
				typedef typename ObjectPool::difference_type difference_type;
				typedef typename ObjectPool::value_type value_type;
				typedef typename ObjectPool::const_reference const_reference;
				typedef typename ObjectPool::const_pointer const_pointer;
				typedef std::random_access_iterator_tag iterator_category; //or another tag

				const_iterator() = default;
				const_iterator(const const_iterator&) = default;
				const_iterator(const iterator& other) :iter_(other.iter_) {}
				const_iterator(typename Base::const_iterator iter) :iter_(iter) {} // 自己添加的
				~const_iterator() = default;

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
				auto operator[](size_type size) const->const_reference { return *iter_->operator[](size); } //optional

				typename Base::const_iterator iter_;
			};
			typedef std::reverse_iterator<iterator> reverse_iterator; //optional
			typedef std::reverse_iterator<const_iterator> const_reverse_iterator; //optional

			auto begin()->iterator { return Base::begin(); }
			auto begin()const->const_iterator { return Base::begin(); }
			auto cbegin() const->const_iterator { return Base::cbegin(); }
			auto end()->iterator { return Base::end(); }
			auto end()const->const_iterator { return Base::end(); }
			auto cend() const->const_iterator { return Base::cend(); }
			auto rbegin()->reverse_iterator { return Base::rbegin(); } //optional
			auto rbegin() const->const_reverse_iterator { return Base::rbegin(); }; //optional
			auto crbegin() const->const_reverse_iterator { return Base::crbegin(); }; //optional
			auto rend()->reverse_iterator { return Base::rend(); } //optional
			auto rend() const->const_reverse_iterator { return Base::rend(); } //optional
			auto crend() const->const_reverse_iterator { return Base::crend(); } //optional

			auto front()->reference { return *begin(); } //optional
			auto front() const->const_reference { return *begin(); } //optional
			auto back()->reference { return *(end() - 1); } //optional
			auto back() const->const_reference { return *(end() - 1); } //optional
			auto at(std::size_t id) const->const_reference { return static_cast<const_reference>(Base::at(id)); }
			auto at(std::size_t id)->reference { return static_cast<reference>(Base::at(id)); }
			auto operator[](size_type size)->reference { return static_cast<reference>(Base::operator[](size)); } //optional
			auto operator[](size_type size) const->const_reference { return static_cast<const_reference>(Base::operator[](size)); } //optional

			static auto Type()->const std::string &{ static const std::string type{ T::Type() + "_pool_" + Base::Type() }; return type; }
			virtual auto type()const->const std::string & override{ return Type(); }
			auto findByName(const std::string &name)const->const_iterator { return Base::findByName(name); }
			auto findByName(const std::string &name)->iterator { return Base::findByName(name);}

			virtual ~ObjectPool() = default;

		protected:
			ObjectPool(Object &father, std::size_t id, const std::string &name):Base(father, id, name) {}
			ObjectPool(Object &father, std::size_t id, const aris::core::XmlElement &xml_ele) :Base(father, id, xml_ele) {}
		
		private:
			friend class Object;
			friend class Root;
		};
		template <class T> class RefPool
		{
		public:
			typedef T value_type;
			typedef T& reference;
			typedef const T& const_reference;
			typedef T* pointer;
			typedef const T* const_pointer;
			typedef std::size_t difference_type;
			typedef std::size_t size_type;

			class iterator
			{
			public:
				typedef typename RefPool::difference_type difference_type;
				typedef typename RefPool::value_type value_type;
				typedef typename RefPool::reference reference;
				typedef typename RefPool::pointer pointer;
				typedef std::random_access_iterator_tag iterator_category; //or another tag

				iterator() = default;
				iterator(const iterator& other) = default;
				iterator(typename std::vector<T*>::iterator iter) :iter_(iter) {} // 自己添加的
				~iterator() = default;

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

				typename std::vector<T*>::iterator iter_;
			};
			class const_iterator
			{
			public:
				typedef typename RefPool::difference_type difference_type;
				typedef typename RefPool::value_type value_type;
				typedef typename RefPool::const_reference const_reference;
				typedef typename RefPool::const_pointer const_pointer;
				typedef std::random_access_iterator_tag iterator_category; //or another tag

				const_iterator() = default;
				const_iterator(const const_iterator&) = default;
				const_iterator(const iterator& other) :iter_(other.iter_) {}
				const_iterator(typename std::vector<T*>::const_iterator iter) :iter_(iter) {} // 自己添加的
				~const_iterator() = default;

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

				typename std::vector<T*>::const_iterator iter_;
			};
			typedef std::reverse_iterator<iterator> reverse_iterator; //optional
			typedef std::reverse_iterator<const_iterator> const_reverse_iterator; //optional

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
			auto at(std::size_t id) const->const_reference { return static_cast<const_reference>(container_.at(id)); }
			auto at(std::size_t id)->reference { return static_cast<reference>(container_.at(id)); }
			auto operator[](size_type size)->reference { return static_cast<reference>(container_.operator[](size)); } //optional
			auto operator[](size_type size) const->const_reference { return static_cast<const_reference>(container_.operator[](size)); } //optional
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
