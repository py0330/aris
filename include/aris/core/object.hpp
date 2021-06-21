#ifndef ARIS_CORE_OBJECT_H_
#define ARIS_CORE_OBJECT_H_

#include <vector>
#include <memory>
#include <functional>

#include <aris_lib_export.h>
#include <aris/core/basic_type.hpp>
#include <aris/core/tinyxml2.h>
#include <aris/core/log.hpp>

namespace aris::core
{
	template<typename T> class ImpPtr{
	public:
		auto reset(T* p)->void { data_unique_ptr_.reset(p); }
		auto get()const->const T* { return data_unique_ptr_.get(); }
		auto get()->T* { return data_unique_ptr_.get(); }
		auto operator->()const->const T* { return data_unique_ptr_.get(); }
		auto operator->()->T* { return data_unique_ptr_.get(); }
		auto operator*()const->const T& { return *data_unique_ptr_; }
		auto operator*()->T& { return *data_unique_ptr_; }

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

	template <class Base>
	class CloneBase {public: auto virtual clone() const->Base* { return new Base(static_cast<const Base&>(*this)); }};

	template <class Derived, class Base>
	class CloneObject : public Base { public:virtual Base * clone() const { return new Derived(static_cast<Derived const&>(*this)); } };

	class ARIS_API NamedObject {
	public:
		auto name() const->const std::string& { return name_; }
		auto setName(const std::string& name) ->void { name_ = name; }
		NamedObject(const std::string& name = "object") :name_(name) {}

	private:
		std::string name_;
	};

	//template <class T, class Pool> class SubRefPool{
	//public:
	//	using value_type = T;
	//	using reference = T & ;
	//	using const_reference = const T&;
	//	using pointer = T * ;
	//	using const_pointer = const T*;
	//	using difference_type = std::size_t;
	//	using size_type = std::size_t;
	//	class const_iterator;

	//	class iterator{
	//	public:
	//		using difference_type = typename SubRefPool::difference_type;
	//		using value_type = typename SubRefPool::value_type;
	//		using reference = typename SubRefPool::reference;
	//		using pointer = typename SubRefPool::pointer;
	//		using iterator_category = std::random_access_iterator_tag; //or another tag

	//		auto operator=(const iterator&other)->iterator& = default;
	//		auto operator==(const iterator&other) const->bool { return iter_ == other.iter_; }
	//		auto operator!=(const iterator&other) const->bool { return iter_ != other.iter_; }
	//		auto operator<(const iterator&other) const->bool { return iter_ < other.iter_; } //optional
	//		auto operator>(const iterator&other) const->bool { return iter_ > other.iter_; } //optional
	//		auto operator<=(const iterator&other) const->bool { return iter_ <= other.iter_; } //optional
	//		auto operator>=(const iterator&other) const->bool { return iter_ >= other.iter_; } //optional

	//		auto operator++()->iterator& { ++iter_; return *this; }
	//		auto operator++(int)->iterator { iterator ret(*this); operator++(); return ret; } //optional
	//		auto operator--()->iterator& { --iter_; return *this; } //optional
	//		auto operator--(int)->iterator { iterator ret(*this); operator--(); return ret; } //optional
	//		auto operator+=(size_type size)->iterator& { iter_ += size; return *this; } //optional
	//		auto operator+(size_type size) const->iterator { return iterator(iter_ + size); } //optional
	//		friend auto operator+(size_type size, const iterator&iter)->iterator { return size + iter.iter_; } //optional
	//		auto operator-=(size_type size)->iterator& { iter_ -= size; return *this; } //optional
	//		auto operator-(size_type size) const->iterator { return iterator(iter_ - size); } //optional
	//		auto operator-(iterator iter) const->difference_type { return iter_ - iter.iter_; } //optional

	//		auto operator*() const->reference { return std::ref(**iter_); }
	//		auto operator->() const->pointer { return *iter_; }
	//		auto operator[](size_type size) const->reference { return *iter_->operator[](size); } //optional

	//		~iterator() = default;
	//		iterator() = default;
	//		iterator(const iterator& other) = default;
	//		iterator(typename std::vector<T*>::iterator iter) :iter_(iter) {} // 自己添加的

	//	private:
	//		typename std::vector<T*>::iterator iter_;
	//		friend class SubRefPool::const_iterator;
	//	};
	//	class const_iterator{
	//	public:
	//		using difference_type = typename SubRefPool::difference_type;
	//		using value_type = typename SubRefPool::value_type;
	//		using const_reference = typename SubRefPool::const_reference;
	//		using const_pointer = typename SubRefPool::const_pointer;
	//		using iterator_category = std::random_access_iterator_tag; //or another tag

	//		auto operator=(const const_iterator&)->const_iterator& = default;
	//		auto operator==(const const_iterator& other) const->bool { return iter_ == other.iter_; }
	//		auto operator!=(const const_iterator& other) const->bool { return iter_ != other.iter_; }
	//		auto operator<(const const_iterator& other) const->bool { return iter_ < other.iter_; } //optional
	//		auto operator>(const const_iterator& other) const->bool { return iter_ > other.iter_; } //optional
	//		auto operator<=(const const_iterator& other) const->bool { return iter_ <= other.iter_; } //optional
	//		auto operator>=(const const_iterator& other) const->bool { return iter_ >= other.iter_; } //optional

	//		auto operator++()->const_iterator& { ++iter_; return *this; }
	//		auto operator++(int)->const_iterator { const_iterator ret(*this); operator++(); return ret; }  //optional
	//		auto operator--()->const_iterator& { --iter_; return *this; } //optional
	//		auto operator--(int)->const_iterator { const_iterator ret(*this); operator--(); return ret; } //optional
	//		auto operator+=(size_type size)->const_iterator& { iter_ += size; return *this; } //optional
	//		auto operator+(size_type size) const->const_iterator { return const_iterator(iter_ + size); } //optional
	//		friend auto operator+(size_type size, const const_iterator& iter)->const_iterator { return const_iterator(size + iter); } //optional
	//		auto operator-=(size_type size)->const_iterator& { iter_ -= size; return *this; } //optional
	//		auto operator-(size_type size) const->const_iterator { return const_iterator(iter_ - size); } //optional
	//		auto operator-(const_iterator iter) const->difference_type { return iter_ - iter.iter_; } //optional

	//		auto operator*() const->const_reference { return **iter_; }
	//		auto operator->() const->const_pointer { return *iter_; }
	//		auto operator[](size_type size) const->const_reference { return *iter_->operator[](size); } //optional

	//		~const_iterator() = default;
	//		const_iterator() = default;
	//		const_iterator(const const_iterator&) = default;
	//		const_iterator(const iterator& other) :iter_(other.iter_) {}
	//		const_iterator(typename std::vector<T*>::const_iterator iter) :iter_(iter) {} // 自己添加的

	//	private:
	//		typename std::vector<T*>::const_iterator iter_;
	//	};
	//	using reverse_iterator = std::reverse_iterator<iterator>; //optional
	//	using const_reverse_iterator = std::reverse_iterator<const_iterator>; //optional

	//	auto size()const->size_type { return container_.size(); }
	//	auto max_size()->size_type { return container_.max_size(); }
	//	auto empty()->bool { return container_.empty(); }

	//	auto begin()->iterator { return container_.begin(); }
	//	auto begin()const->const_iterator { return container_.begin(); }
	//	auto cbegin() const->const_iterator { return container_.cbegin(); }
	//	auto end()->iterator { return container_.end(); }
	//	auto end()const->const_iterator { return container_.end(); }
	//	auto cend() const->const_iterator { return container_.cend(); }
	//	auto rbegin()->reverse_iterator { return container_.rbegin(); } //optional
	//	auto rbegin() const->const_reverse_iterator { return container_.rbegin(); } //optional
	//	auto crbegin() const->const_reverse_iterator { return container_.crbegin(); } //optional
	//	auto rend()->reverse_iterator { return container_.rend(); } //optional
	//	auto rend() const->const_reverse_iterator { return container_.rend(); } //optional
	//	auto crend() const->const_reverse_iterator { return container_.crend(); } //optional
	//	auto front()->reference { return *begin(); } //optional
	//	auto front() const->const_reference { return *begin(); } //optional
	//	auto back()->reference { return *(end() - 1); } //optional
	//	auto back() const->const_reference { return *(end() - 1); } //optional
	//	auto at(std::size_t id) const->const_reference { return static_cast<const_reference>(*container_.at(id)); }
	//	auto at(std::size_t id)->reference { return static_cast<reference>(*container_.at(id)); }
	//	auto operator[](size_type size)->reference { return static_cast<reference>(*container_.operator[](size)); } //optional
	//	auto operator[](size_type size) const->const_reference { return static_cast<const_reference>(*container_.operator[](size)); } //optional

	//public:
	//	auto update() {
	//		container_.clear();
	//		for (auto &obj : *pool_){
	//			if (auto child = dynamic_cast<T*>(&obj)){
	//				container_.push_back(child);
	//			}
	//		}
	//	}
	//	auto swap(SubRefPool& other)->void { return container_.swap(other.container_); }

	//	SubRefPool(Pool* target) { pool_ = target; }

	//private:
	//	std::vector<T*> container_;
	//	Pool* pool_;
	//};
	template <class T, class Pool> class ChildRefPool{
	public:
		using value_type = T;
		using reference = T & ;
		using const_reference = const T&;
		using pointer = T * ;
		using const_pointer = const T*;
		using difference_type = std::size_t;
		using size_type = std::size_t;
		class const_iterator;

		class iterator{
		public:
			using difference_type = typename ChildRefPool::difference_type;
			using value_type = typename ChildRefPool::value_type;
			using reference = typename ChildRefPool::reference;
			using pointer = typename ChildRefPool::pointer;
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

			auto operator*() const->reference { return dynamic_cast<T&>(*iter_); }
			auto operator->() const->pointer { return dynamic_cast<T*>(&*iter_); }
			auto operator[](size_type size) const->reference { return *iter_->operator[](size); } //optional

			~iterator() = default;
			iterator() = default;
			iterator(const iterator& other) = default;
			iterator(typename Pool::iterator iter) :iter_(iter) {} // 自己添加的

		private:
			typename Pool::iterator iter_;
			friend class ChildRefPool::const_iterator;
		};
		class const_iterator{
		public:
			using difference_type = typename ChildRefPool::difference_type;
			using value_type = typename ChildRefPool::value_type;
			using const_reference = typename ChildRefPool::const_reference;
			using const_pointer = typename ChildRefPool::const_pointer;
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
			const_iterator(typename Pool::const_iterator iter) :iter_(iter) {} // 自己添加的

		private:
			typename Pool::const_iterator iter_;
		};
		using reverse_iterator = std::reverse_iterator<iterator>; //optional
		using const_reverse_iterator = std::reverse_iterator<const_iterator>; //optional

		auto size()const->size_type { return pool_->size(); }
		auto max_size()->size_type { return pool_->max_size(); }
		auto empty()->bool { return pool_->empty(); }

		auto begin()->iterator { return pool_->begin();	}
		auto begin()const->const_iterator { return pool_->begin(); }
		auto cbegin() const->const_iterator { return pool_->cbegin(); }
		auto end()->iterator { return pool_->end(); }
		auto end()const->const_iterator { return pool_->end(); }
		auto cend() const->const_iterator { return pool_->cend(); }
		auto rbegin()->reverse_iterator { return pool_->rbegin(); } //optional
		auto rbegin() const->const_reverse_iterator { return pool_->rbegin(); } //optional
		auto crbegin() const->const_reverse_iterator { return pool_->crbegin(); } //optional
		auto rend()->reverse_iterator { return pool_->rend(); } //optional
		auto rend() const->const_reverse_iterator { return pool_->rend(); } //optional
		auto crend() const->const_reverse_iterator { return pool_->crend(); } //optional
		auto front()->reference { return *begin(); } //optional
		auto front() const->const_reference { return *begin(); } //optional
		auto back()->reference { return *(end() - 1); } //optional
		auto back() const->const_reference { return *(end() - 1); } //optional
		auto at(std::size_t id) const->const_reference { return dynamic_cast<const_reference>(pool_->at(id)); }
		auto at(std::size_t id)->reference { return dynamic_cast<reference>(pool_->at(id)); }
		auto operator[](size_type size)->reference { return dynamic_cast<reference>(pool_->operator[](size)); } //optional
		auto operator[](size_type size) const->const_reference { return dynamic_cast<const_reference>(pool_->operator[](size)); } //optional
		
		auto pop_back()->void { pool_->pop_back(); } //optional
		auto erase(iterator iter)->iterator { return pool_->erase(iter.iter_); } //optional
		auto erase(iterator begin_iter, iterator end_iter)->iterator { return pool_->erase(begin_iter.iter_, end_iter.iter_); } //optional
		auto clear()->void { pool_->clear(); } //optional

	public:
		auto add(T *obj)->T & { return dynamic_cast<T&>(pool_->add(obj)); }
		template<typename TT, typename ...Args>
		auto add(Args&&... args)->std::enable_if_t<std::is_base_of<T, TT>::value, TT>& { return dynamic_cast<TT&>(pool_->add(new TT(std::forward<Args>(args)...))); }
		template<typename ...Args>
		auto addChild(Args&&... args)->T& { return dynamic_cast<T&>(pool_->add(new T(std::forward<Args>(args)...))); }
		
		ChildRefPool(Pool* target) { pool_ = target; }

	private:
		Pool *pool_;
	};

	class ARIS_API PointerArrayBase {};

	template <class T, class Base = PointerArrayBase, class A = std::allocator<T>>
	class PointerArray : public Base{
	public:
		using pointer_type = std::unique_ptr<T>;
		using allocator_type = A;
		using value_type = typename std::allocator_traits<A>::value_type;
		using reference = T & ;
		using const_reference = const T&;
		using pointer = typename std::allocator_traits<A>::pointer;
		using const_pointer = typename std::allocator_traits<A>::const_pointer;
		using difference_type = typename std::allocator_traits<A>::difference_type;
		using size_type = typename std::allocator_traits<A>::size_type;
		class iterator;
		class const_iterator;

		class iterator{
		public:
			using difference_type = typename PointerArray::difference_type;
			using value_type = typename PointerArray::value_type;
			using reference = typename PointerArray::reference;
			using pointer = typename PointerArray::pointer;
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
			iterator(const typename std::vector<pointer_type>::iterator iter) :iter_(iter) {} //

		private:
			friend class PointerArray<T, Base, A>::const_iterator;
			friend class PointerArray<T, Base, A>;
			typename std::vector<pointer_type>::iterator iter_;
		};
		class const_iterator{
		public:
			using difference_type = typename PointerArray::difference_type;
			using value_type = typename PointerArray::value_type;
			using reference = typename PointerArray::const_reference;
			using pointer = typename PointerArray::const_pointer;
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
			const_iterator(const typename std::vector<pointer_type>::const_iterator iter) :iter_(iter) {} //

		private:
			typename std::vector<pointer_type>::const_iterator iter_;
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
		auto rbegin() const->const_reverse_iterator { 	return container_.rbegin();	} //optional
		auto crbegin() const->const_reverse_iterator { return container_.crbegin(); } //optional
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

		auto push_back(T*ptr)->void { container_.push_back(pointer_type(ptr)); }
		auto swap(PointerArray& other)->void { return container_.swap(other.container_); }

		//////////////////////////////////////////////////////////////////////////////////////////
		auto add(T *obj)->T & { push_back(obj); return dynamic_cast<T&>(back()); }
		template<typename TT, typename ...Args>
		auto add(Args&&... args)->std::enable_if_t<std::is_base_of<T, TT>::value, TT>& { return dynamic_cast<TT&>(add(new TT(std::forward<Args>(args)...))); }
		template<typename ...Args>
		auto addChild(Args&&... args)->T& { return dynamic_cast<T&>(add(new T(std::forward<Args>(args)...))); }
		//////////////////////////////////////////////////////////////////////////////////////////

		~PointerArray() = default;
		PointerArray() = default;
		PointerArray(const PointerArray&) = delete;
		PointerArray(PointerArray&&other) = default;
		PointerArray& operator=(const PointerArray& other) = delete;
		PointerArray& operator=(PointerArray&& other) = default;

	private:
		typename std::vector<pointer_type> container_;
	};

#define ARIS_DECLARE_BIG_FOUR(type_name) \
	type_name(const type_name &other); \
	type_name(type_name &&other); \
	type_name& operator=(const type_name &other); \
	type_name& operator=(type_name &&other);

#define ARIS_DEFINE_BIG_FOUR(type_name) \
	type_name(const type_name &other) = default; \
	type_name(type_name &&other) = default; \
	type_name& operator=(const type_name &other) = default; \
	type_name& operator=(type_name &&other) = default;

#define ARIS_DEFINE_BIG_FOUR_CPP(type_name) \
	type_name::type_name(const type_name &other) = default; \
	type_name::type_name(type_name &&other) = default; \
	type_name& type_name::operator=(const type_name &other) = default; \
	type_name& type_name::operator=(type_name &&other) = default;

#define ARIS_DELETE_BIG_FOUR(type_name) \
	type_name(const type_name &other) = delete; \
	type_name(type_name &&other) = delete; \
	type_name& operator=(const type_name &other) = delete; \
	type_name& operator=(type_name &&other) = delete;

	template<typename T>
	static auto allocMem(Size &mem_pool_size, T* &pointer, Size size)->void
	{
		*reinterpret_cast<Size*>(&pointer) = mem_pool_size;
		mem_pool_size += sizeof(T) * size;
	}
	template<typename T>
	static auto getMem(char *mem_pool, T* &pointer)->T*
	{
		return reinterpret_cast<T*>(mem_pool + *reinterpret_cast<Size*>(&pointer));
	}

	///
	///  @}
	///
}

#endif
