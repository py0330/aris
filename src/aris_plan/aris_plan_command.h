#ifndef ARIS_PLAN_COMMAND_H_
#define ARIS_PLAN_COMMAND_H_

#include <list>
#include <cmath>
#include <iostream>

#include <aris_core.h>

namespace aris
{
	/// \brief 轨迹规划命名空间
	/// \ingroup aris
	/// 
	///
	///
	namespace plan 
	{
		/// @defgroup command_group 命令解析模块
		/// 本模块负责将形如“enable -m=1 --position”类linux命令字符串解析成命令与参数包，用户可以自定义解析规则、简写规则和默认值等。
		///
		/// ### 命令字符串的规则 ###
		///
		/// 所输入的字符串应该符合以下规则：
		/// - 第一个空格以前应为命令名字
		/// - 第一个空格以后的为命令参数，参数可以带值，那么参数名和参数值之间以“=”分割（不可以有空格），也可以不带值，那么系统会取默认值
		/// - 参数名若以“-”开始，那么该参数为简写，例如“-a”，“-m=1”
		/// - 参数名若以“--”开始，那么该参数为全写，例如“--all”,“--motion=1”
		/// - 参数名若以字母开始，那么这是一组复合参数，每个字符都为参数的简写，例如“ap”等同于“-a -p”
		///
		/// 假设对于命令“enable”，需要能让用户指定所需使能的电机以及运动模式。那么该命令有以下参数：
		/// - “all” 表示所有的电机都要使能，简写为 “a”, 无需默认值
		/// - “motion” 表示需要使能的电机ID，简写为“m”，默认值为“0”
		/// - “velocity” 表示使能成速度模式，简写为“v”，无需默认值
		/// - “position” 表示使能成位置模式，简写为“p”，无需默认值
		/// - “current” 表示使能成电流模式，无简写，无需默认值
		///
		/// 字符串“enable --all -p” 会被解析成：
		///
		/// 命令：enable
		///
		/// 参数：
		/// - all = 
		/// - position = 
		///
		/// 字符串“enable -m=1 --position” 会被解析成：
		///
		/// 命令：enable
		///
		/// 参数：
		/// - motion = 1
		/// - position = 
		///
		/// 字符串“enable ap” 会被解析成：
		///
		/// 命令：enable
		///
		/// 参数：
		/// - all =
		/// - position = 
		/// 
		/// ### 命令解析的规则 ###
		///
		/// 开发者可以根据需要建立自己的命令解析规则，这主要包括：
		/// - 建立参数的简写规则以及默认值
		/// - 指定参数之间的关系
		///
		/// 在上例中，参数之间应该有如下关系：
		/// - 参数“all”和“motion”不能共存，用户不能即指定某个电机动，又指定所有的电机都动
		/// - 参数“position”、“velocity”，“current”不能共存
		/// - 当用户没有指定模式时，系统自动指定模式为“position”
		/// 
		/// 那么以下字符串是合法的：
		/// - “enable --all --velocity” ，意味着所有电机都使能成速度模式
		/// - “enable -m=1 -p”,意味着电机1使能成位置模式
		/// - “enable ap”,等同于“enable -a -p”,将所有电机使能成位置模式
		/// - “enable -a”,将所有电机使能成位置模式
		/// 
		/// 以下字符串非法：
		/// - “enable -a -m=1 --position”，因为不能即指定所有电机，又指定电机1
		/// - “enable -p”,因为不知道是使能所有电机还是某一个电机
		///
		/// 可知，开发者需要根据业务场景来设定参数之间的逻辑与分层关系，在aris中，用户通过UniqueParam和GroupParam两种节点来指定参数之间的关系，
		/// 再加上具体的参数节点，共有三种节点：
		/// - \ref aris::core::Param "Param" ：参数节点，无子节点，可以设定简写，可以设定默认值
		/// - \ref aris::core::UniqueParam "UniqueParam"：是父节点，各个子节点只能出现一个，无简写，如果有默认值，默认值必须为某个子节点的名字
		/// - \ref aris::core::GroupParam "GroupParam"：是父节点，所有子节点必须全部出现，无简写，无默认值
		///
		/// 那么“enable”命令的节点设置模式可以用以下XML文件来表示：
		///
		/// ~~~{.xml}
		///	<enable type="Command" default="group">
		///		<group type="GroupParam">
		///			<unique1 type="UniqueParam">
		///				<all type="Param" abbreviation="a"/>
		///				<motion type="Param" abbreviation="m" default="0"/>
		///			</unique1>
		///			<unique2 type="UniqueParam" default="position">
		///				<position type="Param" abbreviation="p"/>
		///				<velocity type="Param" abbreviation="v"/>
		///				<current type="Param"/>
		///			</unique2>
		///		</group>
		///	</enable>
		/// ~~~
		///
		/// 以上xml节点确定了enable命令的解析规则，首先该命令包含一个名为“group”的“GroupParam”
		/// 这说明了这个节点以下的所有参数（unique1和unique2）都必须有值，而unique1和unique2都是“UniqueParam”
		/// 这说明这两个节点下的所有子节点只能出现一个，于是all和motion不能共存，position、velocity和current也不能共存。
		/// unique2有名为“default”的参数，它指向position，这意味着当用户没有指定unique2中的参数时，parser会自动选择position作为
		/// 默认参数。
		///  
		/// ### 代码实现 ###
		///
		///	\ref demo_command_parser_xml/main.cpp "基于xml和c++的代码示例"
		///
		///	\ref demo_command_parser_cpp/main.cpp "基于纯c++的代码示例"
		///

		/// @{
		///


		class Command;

		class ParamBase :public aris::core::ObjectPool<ParamBase>
		{
		public:
			static auto Type()->const std::string &{ static const std::string type("parambase"); return std::ref(type); }
			auto virtual type() const->const std::string& override{ return Type(); }
			auto virtual saveXml(aris::core::XmlElement &xml_ele) const->void override;
			auto virtual loadXml(const aris::core::XmlElement &xml_ele)->void override;
			auto virtual help(bool isfull, int begin)const->std::string { return std::string{}; };
			auto simpleHelp()const->const std::string &;
			auto command()const->const Command &;
			virtual ~ParamBase();
			explicit ParamBase(const std::string &name = "param_base", const std::string &help = "");
			ParamBase(const ParamBase&);
			ParamBase(ParamBase&&);
			ParamBase& operator=(const ParamBase&);
			ParamBase& operator=(ParamBase&&);

		protected:
			struct Imp;
			aris::core::ImpPtr<Imp> imp_;
			friend class Command;
		};

		/// \class aris::core::Param
		///  命令解析的参数节点
		///
		/// 
		///
		class Param final :public ParamBase
		{
		public:
			static auto Type()->const std::string &{ static const std::string type("Param"); return std::ref(type); }
			auto virtual type() const->const std::string& override{ return Type(); }
			auto virtual saveXml(aris::core::XmlElement &xml_ele) const->void override;
			auto virtual loadXml(const aris::core::XmlElement &xml_ele)->void override;
			auto virtual help(bool isfull, int begin)const->std::string override;
			auto abbreviation()const->char;
			auto defaultParam()const->const std::string &;

			virtual ~Param();
			explicit Param(const std::string &name = "param", const std::string &default_param = "", const std::string &help = "", char abbrev = 0);
			Param(const Param&);
			Param(Param&&);
			Param& operator=(const Param&);
			Param& operator=(Param&&);

		protected:
			struct Imp;
			aris::core::ImpPtr<Imp> imp_;

			friend class Command;
			friend class CommandParser;
		};
		class UniqueParam final :public ParamBase
		{
		public:
			static auto Type()->const std::string &{ static const std::string type("UniqueParam"); return std::ref(type); }
			auto virtual type() const->const std::string& override{ return Type(); }
			auto virtual saveXml(aris::core::XmlElement &xml_ele) const->void override;
			auto virtual loadXml(const aris::core::XmlElement &xml_ele)->void override;
			auto virtual help(bool isfull, int begin)const->std::string override;
			auto defaultParam()const->const std::string &;

			virtual ~UniqueParam();
			explicit UniqueParam(const std::string &name = "unique_param", const std::string &default_param = "", const std::string &help = "");
			UniqueParam(const UniqueParam &);
			UniqueParam(UniqueParam &&);
			UniqueParam& operator=(const UniqueParam &);
			UniqueParam& operator=(UniqueParam &&);

		protected:
			struct Imp;
			aris::core::ImpPtr<Imp> imp_;

			friend class Command;
		};
		class GroupParam final : public ParamBase
		{
		public:
			static auto Type()->const std::string &{ static const std::string type("GroupParam"); return std::ref(type); }
			auto virtual type() const->const std::string& override{ return Type(); }
			auto virtual help(bool isfull, int begin)const->std::string override;

			virtual ~GroupParam();
			explicit GroupParam(const std::string &name = "group_param", const std::string &help = "");
			GroupParam(const GroupParam &);
			GroupParam(GroupParam &&);
			GroupParam& operator=(const GroupParam &);
			GroupParam& operator=(GroupParam &&);
		};
		class Command :public aris::core::ObjectPool<ParamBase>
		{
		public:
			static auto Type()->const std::string &{ static const std::string type("Command"); return std::ref(type); }
			auto virtual type() const->const std::string& override{ return Type(); }
			auto virtual saveXml(aris::core::XmlElement &xml_ele) const->void override;
			auto virtual loadXml(const aris::core::XmlElement &xml_ele)->void override;
			auto defaultParam()const->const std::string &;
			auto help(bool isfull, int begin)const->std::string;

			virtual ~Command();
			explicit Command(const std::string &name = "command", const std::string &default_param = "", const std::string &help = "");
			Command(const Command &);
			Command(Command &&);
			Command& operator=(const Command &);
			Command& operator=(Command &&);

		private:
			struct Imp;
			aris::core::ImpPtr<Imp> imp_;

			friend class CommandParser;
			friend class ParamBase;
		};
		class CommandParser :public aris::core::Object
		{
		public:
			static auto Type()->const std::string &{ static const std::string type("CommandParser"); return std::ref(type); }
			auto virtual type() const->const std::string& override{ return Type(); }
			auto virtual loadXml(const aris::core::XmlElement &xml_ele)->void override;
			auto parse(const std::string &command_string, std::string &cmd_out, std::map<std::string, std::string> &param_map_out)->void;
			auto help()const->std::string;
			auto commandPool()->aris::core::ObjectPool<Command> &;
			auto commandPool()const->const aris::core::ObjectPool<Command> &{ return const_cast<std::decay_t<decltype(*this)> *>(this)->commandPool(); }

			virtual ~CommandParser();
			explicit CommandParser(const std::string &name = "command_parser");
			CommandParser(const CommandParser &);
			CommandParser(CommandParser &&);
			CommandParser& operator=(const CommandParser &);
			CommandParser& operator=(CommandParser &&);

		private:
			struct Imp;
			aris::core::ImpPtr<Imp> imp_;
		};

		/// @}
	}
}


#endif