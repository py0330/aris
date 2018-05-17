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
		/// @defgroup xml_group Object模块
		///
		/// \ref aris::core::Object "Object" 类是aris最基本的模块，用户可以从该类继承并定义自己的类型。
		///
		/// ### 添加节点 ###
		///
		/// 假设现在有如下结构：
		/// 
		/// \dot aris使用树状数据结构
		/// digraph G{
		/// node[shape = record, fontname = Helvetica, fontsize = 10];
		/// family[label = "family"]
		/// father[label = "father"];
		/// uncle[label = "uncle"];
		/// tom[label = "tom"];
		/// bob[label = "bob"];
		/// family->father->tom[arrowhead = "open"];
		/// father->bob[arrowhead = "open"];
		/// family->uncle[arrowhead = "open"];
		/// }
		/// \enddot
		///
		/// 那么可以使用以下代码来构造该结构：
		///
		/// ~~~{.cpp}
		/// Object family1("family");
		/// auto &father1 = family1.add<Object>("father");
		/// family1.add<Object>("uncle");
		/// father1.add<Object>("tom");
		/// father1.add<Object>("bob");
		/// ~~~
		///
		/// ### 与xml交互 ###
		/// 
		/// aris的支持与xml文件的交互，以下可以将上述结构的xml字符串打印出来：
		///
		/// ~~~{.cpp}
		/// std::cout << family1.xmlString() << std::endl;
		/// ~~~
		/// 
		/// 屏幕输出：
		///
		/// ~~~
		/// <family1 type="Object">
		///		<father type="Object">
		///			<tom type="Object"/>
		///			<bob type="Object"/>
		///		</father>
		///		<uncle type="Object"/>
		///	</family1>
		/// ~~~
		///
		/// Object也可以通过xml文件来构造，如下：
		/// 
		/// ~~~{.cpp}
		/// Object family2;
		/// family2.loadXmlStr(
		///		"<family2 type=\"Object\">"
		///		"	<father type=\"Object\">"
		///		"		<tom type=\"Object\"/>"
		///		"		<bob type=\"Object\"/>"
		///		"	</father>"
		///		"	<uncle type=\"Object\"/>"
		///		"</family2>");
		/// ~~~
		///
		/// family2拥有和family完全相同的结构
		///
		/// ### 使用自定义类型 ###
		///
		/// 如果需要保存其他信息，那么需要让用户自己定义类型，假设father和uncle对应Man类型，有age和job两个属性，tom和bob是
		/// Child类型，只有age一个属性，那么新的数据结构如下：
		///
		/// \dot 自定义节点类
		/// digraph G{
		/// node[shape = record, fontname = Helvetica, fontsize = 10];
		/// family[label = "class Family \n name = \"family\""]
		/// father[label = "class Man \n name = \"father\" \n age = \"35\" job = \"teacher\""];
		/// uncle[label = "class Man \n name = \"uncle\" \n age = \"33\" job = \"policeman\""];
		/// tom[label = "class Boy \n name = \"tom\" \n age = \"8\""];
		/// bob[label = "class Boy \n name = \"bob\" \n age = \"6\""];
		/// family->father->tom[arrowhead = "open"];
		/// father->bob[arrowhead = "open"];
		/// family->uncle[arrowhead = "open"];
		/// }
		/// \enddot
		///
		/// 以下是Man类的代码，为了和xml文件交互，用户务必要写静态函数Type，重载虚函数type, saveXml, loadXml，以及默认构造函数：
		///
		/// ~~~{.cpp}		
		///	class Man :public Object
		///	{
		///	public:
		///		static auto Type()->const std::string &{ static const std::string type{ "Man" }; return type; }
		///		auto virtual type() const->const std::string& override{ return Type(); }
		///		auto virtual saveXml(aris::core::XmlElement &xml_ele) const->void override
		///		{
		///			Object::saveXml(xml_ele);
		///			xml_ele.SetAttribute("age", age_);
		///			xml_ele.SetAttribute("job", job_.c_str());
		///		}
		///		auto virtual loadXml(const aris::core::XmlElement &xml_ele)->void override
		///		{
		///			Object::loadXml(xml_ele);
		///			age_ = attributeInt32(xml_ele, "age");
		///			job_ = attributeString(xml_ele, "job");
		///		}
		///
		///		Man(const std::string &name = "man", int age = 0, const std::string job = "teacher") :Object(name), age_(age), job_(job)
		///		{
		///			registerType<Child>();
		///		};
		///	private:
		///		int age_;
		///		std::string job_;
		///	};
		/// ~~~
		///
		/// 这五个函数的作用为：
		/// + Type() : 该函数返回在xml中类型名的字符串，这个字符串不需要和c++代码中的类名保持一致（不过建议保持一致）
		/// + type() : 该函数重载基类的函数，返回Type()就可以，用于从指针获取类名
		/// + saveXml(aris::core::XmlElement &xml_ele) ： 该函数定义了c++对象到xml的转换,在实现中应该首先调用基类同名函数
		/// + loadXml(const aris::core::XmlElement &xml_ele) ： 该类型定义了xml到c++对象的转换,在实现中应该首先调用基类同名函数
		/// + Man(const std::string &name = "man", int age = 0, const std::string job = "teacher") ： 该函数为默认构造函数，用户务必保证形如Man()的函数可以调用，
		///	一般建议用户在该函数中注册对象可能用到的所有子类型。
		///
		/// 类似Man类，以下代码定义了Child类和Family类：
 		///
		/// ~~~{.cpp}
		/// class Child :public Object
		/// {
		/// public:
		///		static auto Type()->const std::string &{ static const std::string type{ "Child" }; return type; }
		///		auto virtual type() const->const std::string& override{ return Type(); }
		///		auto virtual saveXml(aris::core::XmlElement &xml_ele) const->void override
		///		{
		///			Object::saveXml(xml_ele);
		///			xml_ele.SetAttribute("age", age_);
		///		}
		///		auto virtual loadXml(const aris::core::XmlElement &xml_ele)->void override
		///		{
		///			Object::loadXml(xml_ele);
		///			age_ = attributeInt32(xml_ele, "age");
		///		}
		///	
		///		Child(const std::string &name = "child", int age = 0) :Object(name), age_(age) {};
		///
		///	private:
		///		int age_;
		///	};
		///	class Family :public Object
		///	{
		///	public:
		///		static auto Type()->const std::string &{ static const std::string type{ "Family" }; return type; }
		///		auto virtual type() const->const std::string& override{ return Type(); }
		///
		///		Family(const std::string &name = "family") :Object(name)
		///		{
		///			registerType<Man>();
		///		};
		/// };
		/// ~~~
		///
		/// 定义完类型后，就可以由代码来构造上述框图中的数据结构：
		/// ~~~{.cpp}		
		///	Family family3("family3");
		///	auto &father3 = family3.add<Man>("father", 35, "teacher");
		///	family3.add<Man>("uncle", 33, "policeman");
		///	father3.add<Child>("tom", 8);
		///	father3.add<Child>("bob", 6);
		/// 
		///	std::cout << family3.xmlString() << std::endl;
		/// ~~~
		///
		/// 以上代码输出：
		/// ~~~
		///	<family3 type="Family">
		///		<father type="Man" age="35" job="teacher">
        ///			<tom type="Child" age="8"/>
        ///			<bob type="Child" age="6"/>
		///		</father>
		///		<uncle type="Man" age="33" job="policeman"/>
		///	</family3>
		/// ~~~
		///
		/// ### 注册新类型 ###
		/// 在aris中，如果要使用xml转换的功能，需要提前注册。上文中类型注册发生在构造函数里，这样每个对象都知道自己在xml文件里可能使用到的类型的信息。
		/// 现在假设Family，Man，Child三个类型位于基础库中，而用户想要继续扩展一个Boy类：
		/// 
		/// ~~~{.cpp}		
		/// class Boy :public Child
		/// {
		/// public:
		///	static auto Type()->const std::string &{ static const std::string type{ "Boy" }; return type; }
		///	auto virtual type() const->const std::string& override{ return Type(); }
		///
		///	Boy(const std::string &name = "boy", int age = 0) :Child(name, age) {};
		/// };
		/// ~~~
		/// 
		/// 此时因为Family类型的构造函数内并未注册过Boy类，因此需要手动注册。手动注册有两种：
		/// + 全局注册：注册后所有对象都可以在xml中使用
		/// + 局部注册：针对某个对象来注册其可能使用的子类
		///
		/// 局部注册优先于全局注册，以下为全局注册，注册并不针对某个具体的对象，注册后添加了一个名为bill的Boy类型节点：
		///
		/// ~~~{.cpp}		
		/// Family family5;
		/// aris::core::Object::registerTypeGlobal<Boy>();
		/// family5.loadXmlStr(
		/// 	"<family5 type=\"Family\">"
		/// 	"	<father type=\"Man\" age=\"35\" job=\"teacher\">"
		/// 	"		<tom type=\"Child\" age=\"8\"/>"
		/// 	"		<bob type=\"Child\" age=\"6\"/>"
		/// 	"		<bill type=\"Boy\" age=\"3\"/>"
		/// 	"	</father>"
		/// 	"	<uncle type=\"Man\" age=\"33\" job=\"policeman\"/>"
		/// 	"</family5>");
		/// std::cout << family5.xmlString() << std::endl;
		/// ~~~
		///
		/// 以下为局部注册，注册只对family6对象有效：
		///
		/// ~~~{.cpp}		
		/// Family family6;
		/// family6.registerType<Boy>();
		/// family6.loadXmlStr(
		/// 	"<family6 type=\"Family\">"
		/// 	"	<father type=\"Man\" age=\"35\" job=\"teacher\">"
		/// 	"		<tom type=\"Child\" age=\"8\"/>"
		/// 	"		<bob type=\"Child\" age=\"6\"/>"
		/// 	"		<bill type=\"Boy\" age=\"3\"/>"
		/// 	"	</father>"
		/// 	"	<uncle type=\"Man\" age=\"33\" job=\"policeman\"/>"
		/// 	"</family6>");
		/// std::cout << family6.xmlString() << std::endl;
		/// ~~~
		///
		/// ### BIG 5的行为 ###
		/// 
		/// C++中的big 5是指默认构造函数，拷贝构造函数，移动构造函数，拷贝赋值函数和移动赋值函数。
		/// 这5个函数的使用地方为：
		///
		/// 父节点调用 | Default ctor | Copy ctor | Move ctor | Copy assignment | Move assignment |
		/// --------: | : -------- : | : ----- : | : ----- : | : ----------- : | : ----------- : |
		/// 子节点行为 | Default ctor | Copy ctor |     无，左侧直接接管右侧子节点    | 如果左右侧类型一致，则拷贝赋值；如果类型不一致，删除左侧，并拷贝构造； | 如果左侧和右侧类型一致，则移动赋值；如果类型不一致，删除左侧，并移动构造； |
		/// 子节点地址 | 创建         | 创建       | 左侧使用右侧 | 如果左右侧类型一致，则左侧地址不变；如果类型不一致，左侧地址改变； | 如果左右侧类型一致，则左侧地址不变；如果类型不一致，左侧地址改变； |
		///
		///
		///
		/// 
		/// 
		/// 
		/// @{
		/// 
		
		using XmlDocument = tinyxml2::XMLDocument;
		using XmlDeclaration = tinyxml2::XMLDeclaration ;
		using XmlNode = tinyxml2::XMLNode;
		using XmlElement = tinyxml2::XMLElement;
		using XmlAttribute = tinyxml2::XMLAttribute;

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
			class iterator;
			class const_iterator;

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
				friend class ImpContainer<T, A>::const_iterator;
				friend class ImpContainer<T, A>;
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
			auto rbegin() const->const_reverse_iterator { return container_.rbegin(); } //optional
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
		private:
			struct TypeInfo
			{
				using DefaultConstructor = std::function<Object*(void)>;
				using CopyConstructor = std::function<Object*(const Object &)>;
				using MoveConstructor = std::function<Object*(Object &&)>;
				using CopyAssign = std::function<Object&(const Object &, Object &)>;
				using MoveAssign = std::function<Object&(Object &&, Object &)>;

				DefaultConstructor default_construct_func;
				CopyConstructor copy_construct_func;
				MoveConstructor move_construct_func;
				CopyAssign copy_assign_func;
				MoveAssign move_assign_func;

				auto registerTo(const std::string &type, Object &object)->void;
				auto registerTo(const std::string &type)->void;
				template<typename ChildType> static auto inline CreateTypeInfo()->TypeInfo
				{
					static_assert(std::is_base_of<Object, ChildType>::value, "failed to register type, because it is not inheritated from Object");

					return TypeInfo
					{
						default_construct_func_<ChildType>(),
						copy_construct_func_<ChildType>(),
						move_construct_func_<ChildType>(),
						copy_assign_func_<ChildType>(),
						move_assign_func_<ChildType>()
					};
				}

			private:
				template<typename ChildType>
				static auto default_constructor_()->Object*
				{
					return new ChildType;
				}
				template<typename ChildType>
				static auto default_construct_func_(std::enable_if_t<std::is_default_constructible<ChildType>::value> *a = nullptr)->DefaultConstructor
				{
					return DefaultConstructor(TypeInfo::default_constructor_<ChildType>);
				}
				template<typename ChildType>
				static auto default_construct_func_(std::enable_if_t<!std::is_default_constructible<ChildType>::value> *a = nullptr)->DefaultConstructor
				{
					return nullptr;
				}

				template<typename ChildType>
				static auto copy_constructor_(const Object &other)->Object*
				{
					if (!dynamic_cast<const ChildType *>(&other))throw std::runtime_error("can't create type \"" + ChildType::Type() + "\" because object is not the same type");
					return new ChildType(static_cast<const ChildType &>(other));
				}
				template<typename ChildType>
				static auto copy_construct_func_(std::enable_if_t<std::is_copy_constructible<ChildType>::value> *a = nullptr)->CopyConstructor
				{
					return CopyConstructor(TypeInfo::copy_constructor_<ChildType>);
				}
				template<typename ChildType>
				static auto copy_construct_func_(std::enable_if_t<!std::is_copy_constructible<ChildType>::value> *a = nullptr)->CopyConstructor
				{
					return nullptr;
				}

				template<typename ChildType>
				static auto move_constructor_(Object &&other)->Object*
				{
					if (!dynamic_cast<ChildType *>(&other))throw std::runtime_error("can't create type \"" + ChildType::Type() + "\" because object is not the same type");
					return new ChildType(static_cast<ChildType &&>(other));
				}
				template<typename ChildType>
				static auto move_construct_func_(std::enable_if_t<std::is_move_constructible<ChildType>::value> *a = nullptr)->MoveConstructor
				{
					return MoveConstructor(move_constructor_<ChildType>);
				}
				template<typename ChildType>
				static auto move_construct_func_(std::enable_if_t<!std::is_move_constructible<ChildType>::value> *a = nullptr)->MoveConstructor
				{
					return nullptr;
				}

				template<typename ChildType>
				static auto copy_assign_(const Object &from_object, Object &to_object)->Object&
				{
					if (!dynamic_cast<const ChildType *>(&from_object))throw std::runtime_error("can't assign type \"" + ChildType::Type() + "\" because object is not the same type");
					if (!dynamic_cast<ChildType *>(&to_object))throw std::runtime_error("can't assign type \"" + ChildType::Type() + "\" because object is not the same type");
					return static_cast<ChildType &>(to_object) = static_cast<const ChildType &>(from_object);
				}
				template<typename ChildType>
				static auto copy_assign_func_(std::enable_if_t<std::is_copy_assignable<ChildType>::value> *a = nullptr)->CopyAssign
				{
					return CopyAssign(copy_assign_<ChildType>);
				}
				template<typename ChildType>
				static auto copy_assign_func_(std::enable_if_t<!std::is_copy_assignable<ChildType>::value> *a = nullptr)->CopyAssign
				{
					return nullptr;
				}

				template<typename ChildType>
				static auto move_assign_(Object &&from_object, Object &to_object)->Object&
				{
					if (!dynamic_cast<ChildType *>(&from_object))throw std::runtime_error("can't assign type \"" + ChildType::Type() + "\" because object is not the same type");
					if (!dynamic_cast<ChildType *>(&to_object))throw std::runtime_error("can't assign type \"" + ChildType::Type() + "\" because object is not the same type");
					return static_cast<ChildType &>(to_object) = static_cast<ChildType &&>(from_object);
				}
				template<typename ChildType>
				static auto move_assign_func_(std::enable_if_t<std::is_move_assignable<ChildType>::value> *a = nullptr)->MoveAssign
				{
					return std::function<Object&(Object &&, Object &)>(move_assign_<ChildType>);
				}
				template<typename ChildType>
				static auto move_assign_func_(std::enable_if_t<!std::is_move_assignable<ChildType>::value> *a = nullptr)->MoveAssign
				{
					return nullptr;
				}
			};

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
			template<typename ChildType>
			static auto registerTypeGlobal()->void { TypeInfo::CreateTypeInfo<ChildType>().registerTo(ChildType::Type()); }
			template<typename ChildType>
			auto registerType()->void { TypeInfo::CreateTypeInfo<ChildType>().registerTo(ChildType::Type(), *this); }
			static auto Type()->const std::string &{ static const std::string type("Object"); return std::ref(type); }
			auto virtual type() const->const std::string&{ return Type(); }
			auto virtual loadXml(const aris::core::XmlElement &xml_ele)->void;
			auto virtual saveXml(aris::core::XmlElement &xml_ele) const->void;
			auto loadXmlFile(const std::string &filename)->void;
			auto saveXmlFile(const std::string &filename) const->void;
			auto loadXmlDoc(const aris::core::XmlDocument &xml_doc)->void;
			auto saveXmlDoc(aris::core::XmlDocument &xml_doc)const->void;
			auto loadXmlStr(const std::string &xml_str)->void { aris::core::XmlDocument xml_doc; xml_doc.Parse(xml_str.c_str()); loadXmlDoc(xml_doc); };
			auto saveXmlStr(std::string &xml_str)const->void { xml_str = xmlString(); };
			auto xmlString()const->std::string;
			auto name()const->const std::string&;
			auto id()const->std::size_t;
			auto root()->Object&;
			auto root()const->const Object&{ return const_cast<std::decay_t<decltype(*this)> *>(this)->root(); }
			auto father()->Object&;
			auto father()const->const Object&{ return const_cast<std::decay_t<decltype(*this)> *>(this)->father(); }
			auto children()->ImpContainer<Object>&;
			auto children()const->const ImpContainer<Object>&{ return const_cast<std::decay_t<decltype(*this)> *>(this)->children(); }
			auto findByName(const std::string &name)const->ImpContainer<Object>::const_iterator { return const_cast<std::decay_t<decltype(*this)> *>(this)->findByName(name); }
			auto findByName(const std::string &name)->ImpContainer<Object>::iterator;
			template<typename T = Object>
			auto findType(const std::string &name)const->const T*{ return const_cast<Object*>(this)->findType<T>(); };
			template<typename T = Object>
			auto findType(const std::string &name)->T* { auto iter = findByName(name); return iter != children().end() && dynamic_cast<T*>(&*iter) ? dynamic_cast<T*>(&*iter) : nullptr; }
			template<typename T = Object, typename ...Args>
			auto findOrInsert(const std::string &name, Args&&... args)-> T* { auto p = findType<T>(name); return p ? p : &add<T>(name, std::forward<Args>(args)...); }
			auto add(Object *obj)->Object &;
			template<typename T, typename ...Args>
			auto add(Args&&... args)->std::enable_if_t<std::is_base_of<Object, T>::value, T>& { return dynamic_cast<T&>(add(new T(std::forward<Args>(args)...))); }
			template<typename T, typename ...Args>
			auto add(Args&&... args)->std::enable_if_t<!std::is_base_of<Object, T>::value, T>& { static_assert(false, "you must add object that inherited from class \"Object\""); }

			virtual ~Object();
			explicit Object(const std::string &name = "object");
			Object(const Object &);
			Object(Object &&);
			Object& operator=(const Object &);
			Object& operator=(Object &&);

		private:
			struct Imp;
			ImpPtr<Imp> imp_;
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
				friend class ObjectPool;
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

			auto add(T *obj)->T & { return dynamic_cast<T&>(Object::add(obj)); }
			template<typename TT, typename ...Args>
			auto add(Args&&... args)->std::enable_if_t<std::is_base_of<T, TT>::value, TT>& { return dynamic_cast<TT&>(add(new TT(std::forward<Args>(args)...))); }
			template<typename ...Args>
			auto addChild(Args&&... args)->T& { return dynamic_cast<T&>(add(new T(std::forward<Args>(args)...))); }

			virtual ~ObjectPool() = default;
			explicit ObjectPool(const std::string &name = "object_pool") :Base(name){}
		
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
			class const_iterator;

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
				friend class RefPool<T>::const_iterator;
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

		///
		///  @}
		///
	}
}

#endif
