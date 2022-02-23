#include <iostream>
#include <array>
#include <aris/core/reflection.hpp>
#include <aris/core/serialization.hpp>

// 基础类型无法包含属性，只可以直接转成字符串，例如转化规则为：
// Basic{"paul", "33"}  ->  "paul_33"
struct Basic {
	std::string name_;
	int age_;
};
// Basic 类型的转化函数
auto basicToStr(void* value)->std::string {
	// 转化得到 Basic 变量 //
	auto basic = reinterpret_cast<Basic*>(value);
	
	// 返回值 //
	return basic->name_ + "_" + std::to_string(basic->age_);
}
auto strToBasic(void* value, std::string_view str)->void {
	// 转化得到 Basic 变量 //
	auto basic = reinterpret_cast<Basic*>(value);

	// 构造 Basic 变量 //
	basic->name_ = str.substr(0, str.find_last_of('_'));
	basic->age_ = std::stoi(std::string(str.substr(str.find_last_of('_') + 1, std::string_view::npos)));
}


// 组合类型可以包含属性
class BaseClass {
public:
	// 属性1，使用成员变量定义
	int member;

	// 属性2，使用引用函数定义
	auto referenceMember()->std::string& { return reference_member_; }
	
	// 析构函数，此类可以由其他类继承 //
	virtual ~BaseClass() = default;

private:
	std::string reference_member_;
};

// 继承的类型，ARIS的反射共支持 5 种属性的反射方法
class ChildClass: public BaseClass {
public:
	// 属性1 & 2，继承自 BaseClass

	// 属性3，使用类中 set & get 函数定义
	auto setValue3(double v) { value_3_ = v; }
	auto getValue3()->double { return value_3_; }

	// 属性4，使用指针 set & get 定义，此时属性支持多态
	auto setValue4(BaseClass *v) { value_4_.reset(v);}
	auto getValue4()->BaseClass& { return *value_4_; }

	// 属性5，使用全局 set & get 函数定义，函数在类定义之后
	double global_value_;

	// 析构函数，此类可以由其他类继承 //
	virtual ~ChildClass() = default;

private:
	std::string reference_func_data_;
	double value_3_;
	std::unique_ptr<BaseClass> value_4_;
};
// 属性5的全局set & get函数
auto setValue5(ChildClass *c, double v)->void { c->global_value_ = v; }
auto getValue5(ChildClass *c)->double { return c->global_value_; }

ARIS_REGISTRATION{

	// 注册 Basic 类型，以及它和字符串的转换关系(此时不能有其他 prop 属性) //
	aris::core::class_<Basic>("Basic")
		.textMethod(basicToStr, strToBasic)
		;
	
	// 注册 BaseClass 类型 //
	aris::core::class_<BaseClass>("BaseClass")
		.prop("member", &BaseClass::member)
		.prop("referenceMember", &BaseClass::referenceMember)
		;

	// 注册 ChildClass 类型 //
	aris::core::class_<ChildClass>("ChildClass")
		.inherit<BaseClass>()                                          //此处继承 member 和 referenceMember
		.prop("value3", &ChildClass::setValue3, &ChildClass::getValue3)
		.prop("value4", &ChildClass::setValue4, &ChildClass::getValue4)
		.prop("value5", &setValue5, &getValue5)
		;

	// 注册 BaseClass Vector 类型 //
	aris::core::class_<std::vector<BaseClass> >("VecBase")
		.asArray()
		;

	aris::core::class_<std::vector<int> >("VecInt")
		.asArray()
		;
}


class func_ {
public:
	template <typename R, typename... Param>
	func_(std::string name, std::function<R(Param...)> f) {
		using Arguments = std::tuple < std::add_lvalue_reference_t<Param>...>;
		inside_func = [=](std::vector<aris::core::Instance> &param_ins_vec)->void* {
			auto t = make_param_tuple<Arguments>(
				param_ins_vec,
				std::make_index_sequence<std::tuple_size_v<Arguments>>{}
			);
			std::apply(f, t);
			return nullptr;
		};
	}

	template <typename... Param>
	auto invoke(Param&&... params)->void{
		auto ins_vec = std::vector<aris::core::Instance>({params...});
		inside_func(ins_vec);
	}

	auto invoke(std::vector<aris::core::Instance> params)->void {
		inside_func(params);
	}

private:
	std::function<void*(std::vector<aris::core::Instance> &)> inside_func;

	template <class MyTuple, std::size_t... I>
	auto make_param_tuple(std::vector<aris::core::Instance> &param_ins_vec, std::index_sequence<I...>)->MyTuple {
		return MyTuple(*param_ins_vec[I].castTo<std::remove_reference_t<std::tuple_element_t<I, MyTuple>>>()...);
	}
};

//template <typename R, typename... Param>
//auto select_overload(std::function<R(Param...)> f)->std::function<R(Param...)> { return f; }

template<typename Signature >
Signature* select_overload(Signature *func) { return func; }



void f1(int a) { std::cout << "f1:a  " << a << std::endl; }
void f1(double b) { std::cout << "f1:b  " << b << std::endl; }

int main(){
	std::cout << std::setiosflags(std::ios::left);

	//# 使用反射 #//
	{
		//## 基础类型 ##//
		// 基础类型可以和字符串交互，也可直接序列化
		Basic basic1{ "jack", 12 };
		std::cout << aris::core::toXmlString(basic1) << std::endl;

		// 使用 Instance 进行反射交互，Instance 是basic1的引用，不负责其生命周期
		aris::core::Instance ins = basic1;
		std::cout << std::setw(25) << "print basic text:" << ins.toString() << std::endl;
		ins.fromString("paul_21");
		std::cout << std::setw(25) << "data after reflection:" << basic1.name_ << "--" << basic1.age_ << std::endl;

		// 使用反射创建类型，其中ptr是智能指针，负责实际数据的生命周期，ins2 是Instance，为实际数据的引用，不负责生命周期
		auto[ptr, ins2] = aris::core::Type::getType("Basic")->create();
		ins2.fromString("bob_44");
		Basic* basic2_ptr = ins2.castTo<Basic>();
		std::cout << std::setw(25) << "data created:" << basic2_ptr->name_ << "--" << basic2_ptr->age_ << std::endl;
	}

	{
		std::cout << "--------------------------------------" << std::endl;
		
		//## 组合类型 ##//
		// 组合类型无法直接和字符串做交互，但可序列化成 xml/json 字符串 
		ChildClass value1;
		value1.member = 123;
		value1.referenceMember() = "aaa";
		value1.setValue3(1.28);
		value1.setValue4(new BaseClass);
		setValue5(&value1, 3.14);

		value1.getValue4().member = 456;
		value1.getValue4().referenceMember() = "reference";

		// 使用 Instance 进行反射交互，Instance 是引用，不负责引用对象生命周期
		// 读取内容
		aris::core::Instance ins = value1;
		std::cout << std::setw(25) << "read result:" <<std::endl
			<< std::setw(25) << "member:" << ins.get("member").toString() << std::endl
			<< std::setw(25) << "referenceMember:" << ins.get("referenceMember").toString() << std::endl
			<< std::setw(25) << "value3:" << ins.get("value3").toString() << std::endl
			<< std::setw(25) << "value4.member:" << ins.get("value4").get("member").toString() << std::endl
			<< std::setw(25) << "value4.referenceMember:" << ins.get("value4").get("referenceMember").toString() << std::endl
			<< std::setw(25) << "value5:" << ins.get("value5").toString() << std::endl;

		// 写入内容
		std::cout << "-------------------" << std::endl;
		ins.set("member", 456);
		ins.set("referenceMember", std::string("bbbb")); // set的类型必须和注册的类型一样
		ins.set("value3", 5.49);
		std::unique_ptr<BaseClass> value4_insert(new BaseClass);
		value4_insert->member = 1234567;
		value4_insert->referenceMember() = "refer";
		ins.set("value4", *value4_insert.release());  // ins 负责插入对象的生命周期
		ins.set("value5", 12.28);
		std::cout << std::setw(25) << "write result:" << std::endl
			<< std::setw(25) << "member:" << value1.member << std::endl
			<< std::setw(25) << "referenceMember:" << value1.referenceMember() << std::endl
			<< std::setw(25) << "value3:" << value1.getValue3() << std::endl
			<< std::setw(25) << "value4.member:" << value1.getValue4().member << std::endl
			<< std::setw(25) << "value4.referenceMember:" << value1.getValue4().referenceMember() << std::endl
			<< std::setw(25) << "value5:" << getValue5(&value1) << std::endl;

		// 使用反射创建类型，其中ptr是智能指针，负责实际数据的生命周期，ins2 是Instance，为实际数据的引用，不负责生命周期
		std::cout << "-------------------" << std::endl;
		auto[ptr, ins2] = aris::core::Type::getType("ChildClass")->create();
		ins2.set("member", 1234);
		ins2.set("referenceMember", std::string("cccc")); // set的类型必须和注册的类型一样
		ins2.set("value3", 6.25);
		std::unique_ptr<BaseClass> value4_insert2(new BaseClass);
		value4_insert2->member = 567;
		value4_insert2->referenceMember() = "refer2";
		ins2.set("value4", *value4_insert2.release());  // ins 负责插入对象的生命周期
		ins2.set("value5", 12.2833);

		ChildClass* created_ptr = ins2.castTo<ChildClass>();
		std::cout << std::setw(25) << "data created:" << std::endl
			<< std::setw(25) << "member:" << created_ptr->member << std::endl
			<< std::setw(25) << "referenceMember:" << created_ptr->referenceMember() << std::endl
			<< std::setw(25) << "value3:" << created_ptr->getValue3() << std::endl
			<< std::setw(25) << "value4.member:" << created_ptr->getValue4().member << std::endl
			<< std::setw(25) << "value4.referenceMember:" << created_ptr->getValue4().referenceMember() << std::endl
			<< std::setw(25) << "value5:" << getValue5(&value1) << std::endl;

		// 序列化，和 xml 字符串做交互
		std::cout << "-------------------" << std::endl;
		auto str = aris::core::toXmlString(ins2);
		std::cout << str << std::endl;

		std::cout << aris::core::toJsonString(ins2) << std::endl;
		ChildClass value3;
		aris::core::fromXmlString(value3, str);

		// 如果想手动管理所创建实例的生命周期，需release ptr //
		auto base_ptr = reinterpret_cast<BaseClass*>(ptr.release());
		delete base_ptr;
	}

	{
		std::cout << "--------------------------------------" << std::endl;

		//## 数组类型 ##//
		// 组合类型无法直接和字符串做交互，但可序列化成 xml/json 字符串 
		// 当Instance 对象为数组时，可以使用 at   size  push_back clear 等方法
		std::vector<BaseClass> value;
		value.push_back(BaseClass());
		value.back().member = 123;
		value.back().referenceMember() = std::string("aaaaa");
		value.push_back(BaseClass());
		value.back().member = 456;
		value.back().referenceMember() = std::string("bbbbb");


		// 使用 Instance 进行反射交互，Instance 是引用，不负责引用对象生命周期
		// 读取内容
		aris::core::Instance ins = value;
		std::cout << std::setw(25) << "read result:" << std::endl
			<< std::setw(25) << "member:" << ins.at(0).get("member").toString() << std::endl
			<< std::setw(25) << "referenceMember:" << ins.at(0).get("referenceMember").toString() << std::endl
			<< std::setw(25) << "member:" << ins.at(1).get("member").toString() << std::endl
			<< std::setw(25) << "referenceMember:" << ins.at(1).get("referenceMember").toString() << std::endl;


		// 写入内容
		std::cout << "-------------------" << std::endl;
		ins.clear();
		BaseClass value_insert;
		value_insert.member = 789;
		value_insert.referenceMember() = "refer_insert";
		ins.push_back(value_insert);
		ins.push_back(value_insert);
		ins.push_back(value_insert);
		std::cout << std::setw(25) << "write result:" << std::endl
			<< std::setw(25) << "member:" << value[0].member << std::endl
			<< std::setw(25) << "referenceMember:" << value[0].referenceMember() << std::endl
			<< std::setw(25) << "member:" << value[1].member << std::endl
			<< std::setw(25) << "referenceMember:" << value[1].referenceMember() << std::endl
			<< std::setw(25) << "member:" << value[2].member << std::endl
			<< std::setw(25) << "referenceMember:" << value[2].referenceMember() << std::endl;


		// 使用反射创建类型，其中ptr是智能指针，负责实际数据的生命周期，ins2 是Instance，为实际数据的引用，不负责生命周期
		std::cout << "-------------------" << std::endl;
		auto[ptr, ins2] = aris::core::Type::getType("VecBase")->create();
		ins2.push_back(value_insert);
		ins2.push_back(value_insert);
		std::vector<BaseClass>* basic2_ptr = ins2.castTo<std::vector<BaseClass>>();
		std::cout << std::setw(25) << "data created:" << std::endl
			<< std::setw(25) << "member:" << ins2.at(0).get("member").toString() << std::endl
			<< std::setw(25) << "referenceMember:" << ins2.at(0).get("referenceMember").toString() << std::endl
			<< std::setw(25) << "member:" << ins2.at(1).get("member").toString() << std::endl
			<< std::setw(25) << "referenceMember:" << ins2.at(1).get("referenceMember").toString() << std::endl;

		// 序列化，和 xml 字符串做交互
		std::cout << "-------------------" << std::endl;
		auto str = aris::core::toXmlString(ins2);
		std::cout << str << std::endl;
		std::vector<BaseClass> value3;
		aris::core::fromXmlString(value3, str);


		// 如果想手动管理所创建实例的生命周期，需release ptr //
		auto base_ptr = reinterpret_cast<std::vector<BaseClass>*>(ptr.release());
		delete base_ptr;
	}
	

	auto f = [](int &a, double b)->double {
		std::cout << "a:" << a << std::endl;
		std::cout << "b:" << b << std::endl;
		std::cout << &a << std::endl;
		a = 123456;
		b = 0.0001;
		return 0.0;
	};
	std::function<double(int&, double)> ff = f;


	func_ aaa("aaa", ff);

	auto ffff = select_overload<void(int)>(f1);

	std::function<void(int)> fffff = select_overload<void(int)>(f1);
	func_ f1_("f1", fffff);
	//func_ f1_("f1", select_overload<void(int)>(f1));

	auto func = select_overload<void(int)>(f1);

	int bc = 1;
	int &bd = bc;
	double ccc(145.67);

	std::cout << &bc << std::endl;
	std::cout << &bd << std::endl;
	aaa.invoke(bd, ccc);

	std::cout << bc << std::endl;
	std::cout << ccc << std::endl;
	bc = 100;
	std::vector<aris::core::Instance> ins_vec;
	ins_vec.push_back(bc);
	ins_vec.push_back(ccc);

	aaa.invoke(ins_vec);

	std::cout << bc << std::endl;
	std::cout << ccc << std::endl;

	std::cout << "demo_reflection finished, press any key to continue" << std::endl;
	std::cin.get();
	//return 0; 
}

