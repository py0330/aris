#include <iostream>
#include <aris/core/core.hpp>
#include "test_core_object.h"


using namespace aris::core;

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
class ChildClass : public BaseClass {
public:
	// 属性1 & 2，继承自 BaseClass

	// 属性3，使用类中 set & get 函数定义
	auto setValue3(double v) { value_3_ = v; }
	auto getValue3()->double { return value_3_; }

	// 属性4，使用指针 set & get 定义，此时属性支持多态
	auto setValue4(BaseClass *v) { value_4_.reset(v); }
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

void test_object()
{
	std::cout << std::endl << "-----------------test object---------------------" << std::endl;
	
	
	
	
	
	
	
	
	
	
	std::cout << "-----------------test object finished------------" << std::endl << std::endl;
}