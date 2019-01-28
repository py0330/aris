﻿#ifndef ARIS_PLAN_ROOT_H_
#define ARIS_PLAN_ROOT_H_

#include <list>
#include <cmath>
#include <iostream>
#include <functional>
#include <map>
#include <any>
#include <atomic>

#include <aris_core.h>
#include <aris_control.h>
#include <aris_dynamic.h>

/// \brief 轨迹规划命名空间
/// \ingroup aris
/// 
///
///
namespace aris::plan
{
	struct PlanTarget
	{
		aris::dynamic::Model* model;                      // prepair/execute/collect  get&set(but be careful when prepair)
		aris::control::Master* master;                    // prepair/execute/collect  get&set(but be careful when prepair)
		std::uint64_t command_id;                         // prepair/execute/collect  get
		std::uint64_t option;                             // prepair/execute/collect  get&set when prepair, get when execute and collect
		std::any param;                                   // prepair/execute/collect  set when prepair, get when execute, destroy when collect
		std::int32_t count;                               //         execute/collect  get
		std::int64_t begin_global_count;                  //         execute/collect  get
		aris::control::Master::RtStasticsData rt_stastic; //                /collect  get
	};
	class Plan :public aris::core::Object
	{
	public:
		enum Option : std::uint64_t
		{
			NOT_RUN_PREPAIR_FUNCTION = 0x01ULL << 0,
			PREPAIR_WHEN_ALL_PLAN_EXECUTED = 0x01ULL << 1,
			PREPAIR_WHEN_ALL_PLAN_COLLECTED = 0x01ULL << 2,

			NOT_RUN_EXECUTE_FUNCTION = 0x01ULL << 3,
			EXECUTE_WHEN_ALL_PLAN_EXECUTED = 0x01ULL << 4,
			EXECUTE_WHEN_ALL_PLAN_COLLECTED = 0x01ULL << 5,
			WAIT_FOR_EXECUTION = 0x01ULL << 6,
			WAIT_IF_CMD_POOL_IS_FULL = 0x01ULL << 7,

			NOT_RUN_COLLECT_FUNCTION = 0x01ULL << 8,
			COLLECT_WHEN_ALL_PLAN_EXECUTED = 0x01ULL << 9,
			COLLECT_WHEN_ALL_PLAN_COLLECTED = 0x01ULL << 10,
			WAIT_FOR_COLLECTION = 0x01ULL << 11,

			USE_TARGET_POS = 0x01ULL << 16,
			USE_TARGET_VEL = 0x01ULL << 17,
			USE_TARGET_CUR = 0x01ULL << 18,
			USE_VEL_OFFSET = 0x01ULL << 19,
			USE_CUR_OFFSET = 0x01ULL << 20,

			NOT_CHECK_POS_MIN = 0x01ULL << 24,
			NOT_CHECK_POS_MAX = 0x01ULL << 25,
			NOT_CHECK_POS_CONTINUOUS = 0x01ULL << 26,
			NOT_CHECK_POS_CONTINUOUS_AT_START = 0x01ULL << 27,
			NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER = 0x01ULL << 28,
			NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER_AT_START = 0x01ULL << 29,
			NOT_CHECK_POS_FOLLOWING_ERROR = 0x01ULL << 30,

			NOT_CHECK_VEL_MIN = 0x01ULL << 31,
			NOT_CHECK_VEL_MAX = 0x01ULL << 32,
			NOT_CHECK_VEL_CONTINUOUS = 0x01ULL << 33,
			NOT_CHECK_VEL_CONTINUOUS_AT_START = 0x01ULL << 34,
			NOT_CHECK_VEL_FOLLOWING_ERROR = 0x01ULL << 35,

			NOT_PRINT_CMD_INFO = 0x01ULL << 40,
			NOT_PRINT_EXECUTE_COUNT = 0x01ULL << 41,
			NOT_LOG_CMD_INFO = 0x01ULL << 45,

		};

		static auto Type()->const std::string & { static const std::string type("Plan"); return std::ref(type); }
		auto virtual type() const->const std::string& override { return Type(); }
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void {}
		auto virtual executeRT(PlanTarget &target)->int { return 0; }
		auto virtual collectNrt(PlanTarget &target)->void {}
		auto command()->aris::core::Command &;
		auto command()const->const aris::core::Command & { return const_cast<std::decay_t<decltype(*this)> *>(this)->command(); }

		virtual ~Plan();
		explicit Plan(const std::string &name = "plan");
		Plan(const Plan &);
		Plan(Plan &&);
		Plan& operator=(const Plan &);
		Plan& operator=(Plan &&);

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};
	class PlanRoot :public aris::core::Object
	{
	public:
		static auto Type()->const std::string & { static const std::string type("PlanRoot"); return std::ref(type); }
		auto virtual type() const->const std::string& override { return Type(); }
		auto planPool()->aris::core::ObjectPool<Plan> &;
		auto planPool()const->const aris::core::ObjectPool<Plan> & { return const_cast<std::decay_t<decltype(*this)> *>(this)->planPool(); }
		auto planParser()->aris::core::CommandParser;

		virtual ~PlanRoot();
		explicit PlanRoot(const std::string &name = "plan_root");
		PlanRoot(const PlanRoot &);
		PlanRoot(PlanRoot &&);
		PlanRoot& operator=(const PlanRoot &);
		PlanRoot& operator=(PlanRoot &&);

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};
	
	/// \brief 让电机使能
	///
	/// ### 参数定义 ###
	/// 指定电机，默认指定所有电机：
	/// + 指定所有电机：“en -a” 或 “en --all” 或 “en”
	/// + 按照绝对地址（absID），例如绝对的 0 号电机：“en -m=0” 或 “en --motion_id=0”
	/// + 按照物理地址（phyID），例如物理的 2 号电机：“en -p=2” 或 “en --physical_id=2”
	/// + 按照从站地址（slaID），例如 5 号从站：“en -s=5” 或 “en --slave_id=5”
	/// 
	/// 指定本指令的最长运行时间（默认为5000ms）：
	/// + 使能0号电机，并指定其最长时间为5000ms： “en -m=0 --limit_time=5000”
	/// 
	class Enable : public Plan
	{
	public:
		static auto Type()->const std::string & { static const std::string type("Enable"); return std::ref(type); }
		auto virtual type() const->const std::string& override { return Type(); }
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void override;
		auto virtual executeRT(PlanTarget &target)->int override;
		auto virtual collectNrt(PlanTarget &target)->void override;

		virtual ~Enable();
		explicit Enable(const std::string &name = "enable_plan");
		Enable(const Enable &);
		Enable(Enable &&);
		Enable& operator=(const Enable &);
		Enable& operator=(Enable &&);

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};
	/// \brief 让电机去使能
	/// 
	/// ### 参数定义 ###
	/// 指定电机，默认指定所有电机：
	/// + 指定所有电机：“ds -a” 或 “ds --all” 或 “ds”
	/// + 按照绝对地址（absID），例如绝对的 0 号电机：“ds -m=0” 或 “ds --motion_id=0”
	/// + 按照物理地址（phyID），例如物理的 2 号电机：“ds -p=2” 或 “ds --physical_id=2”
	/// + 按照从站地址（slaID），例如 5 号从站：“ds -s=5” 或 “ds --slave_id=5”
	/// 
	/// 指定本指令的最长运行时间（默认为5000ms）：
	/// + 去使能0号电机，并指定其最长时间为5000ms： “ds -m=0 --limit_time=5000”
	/// 
	class Disable : public Plan
	{
	public:
		static auto Type()->const std::string & { static const std::string type("Enable"); return std::ref(type); }
		auto virtual type() const->const std::string& override { return Type(); }
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void override;
		auto virtual executeRT(PlanTarget &target)->int override;
		auto virtual collectNrt(PlanTarget &target)->void override;

		virtual ~Disable();
		explicit Disable(const std::string &name = "enable_plan");
		Disable(const Disable &);
		Disable(Disable &&);
		Disable& operator=(const Disable &);
		Disable& operator=(Disable &&);

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};
	/// \brief 让电机回零
	/// 
	/// ### 参数定义 ###
	/// 指定电机，默认指定所有电机：
	/// + 指定所有电机：“hm -a” 或 “hm --all” 或 “hm”
	/// + 按照绝对地址（absID），例如绝对的 0 号电机：“en -m=0” 或 “en --motion_id=0”
	/// + 按照物理地址（phyID），例如物理的 2 号电机：“en -p=2” 或 “en --physical_id=2”
	/// + 按照从站地址（slaID），例如 5 号从站：“en -s=5” 或 “en --slave_id=5”
	/// 
	/// 指定本指令的最长运行时间（默认为5000ms）：
	/// + 使能0号电机，并指定其最长时间为5000ms： “en -m=0 --limit_time=5000”
	/// 
	class Home : public Plan
	{
	public:
		static auto Type()->const std::string & { static const std::string type("Home"); return std::ref(type); }
		auto virtual type() const->const std::string& override { return Type(); }
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void override;
		auto virtual executeRT(PlanTarget &target)->int override;
		auto virtual collectNrt(PlanTarget &target)->void override;

		virtual ~Home();
		explicit Home(const std::string &name = "home_plan");
		Home(const Home &);
		Home(Home &&);
		Home& operator=(const Home &);
		Home& operator=(Home &&);

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};
	/// \brief 让电机切换模式
	/// 
	/// ### 参数定义 ###
	/// 指定电机，默认指定所有电机：
	/// + 指定所有电机：“md -a” 或 “md --all” 或 “md”
	/// + 按照绝对地址（absID），例如绝对的 0 号电机：“md -m=0” 或 “md --motion_id=0”
	/// + 按照物理地址（phyID），例如物理的 2 号电机：“md -p=2” 或 “md --physical_id=2”
	/// + 按照从站地址（slaID），例如 5 号从站：“md -s=5” 或 “md --slave_id=5”
	/// 
	/// 指定本指令的最长运行时间（默认为5000ms）：
	/// + 使能0号电机，并指定其最长时间为5000ms： “md -m=0 --limit_time=5000”
	/// 
	/// 指定模式（请参考canopen DS402标准，默认为8，同步位置控制模式）：
	/// + 使能0号电机，并指定模式为9： “md -m=0 --mode=9 --limit_time=5000”
	/// 
	class Mode : public Plan
	{
	public:
		static auto Type()->const std::string & { static const std::string type("Mode"); return std::ref(type); }
		auto virtual type() const->const std::string& override { return Type(); }
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void override;
		auto virtual executeRT(PlanTarget &target)->int override;
		auto virtual collectNrt(PlanTarget &target)->void override;

		virtual ~Mode();
		explicit Mode(const std::string &name = "mode_plan");
		Mode(const Mode &);
		Mode(Mode &&);
		Mode& operator=(const Mode &);
		Mode& operator=(Mode &&);

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};
	/// \brief 复位，机器人从轴空间按照指定速度运行到指定位置处
	///
	/// 指令不使用正反解，结束后Model和Controller位置不对应，此时需用rc指令来让两者重合
	///
	/// ### 参数定义 ###
	/// 指定电机，默认指定所有电机：
	/// + 指定所有电机：“rs -a” 或 “rs --all” 或 “rs”
	/// + 按照绝对地址（absID），例如绝对的 0 号电机：“rs -m=0” 或 “rs --motion_id=0”
	/// + 按照物理地址（phyID），例如物理的 2 号电机：“rs -p=2” 或 “rs --physical_id=2”
	/// + 按照从站地址（slaID），例如 5 号从站：“rs -s=5” 或 “rs --slave_id=5”
	/// 
	/// 指定复位位置，0为最小位置，1为最大位置，参数应在0到1之间，默认为0.5：
	/// + 将1号电机走到最大位置：“rs -m=1 --pos=1”
	/// + 将2号电机走到中间位置：“rs -m=2 --pos=0.5”
	/// + 将所有电机走到最小位置：“rs -a --pos=0”
	/// + 假设系统中有6个电机，分别走到0.1到0.6，注意大括号中不能有空格：“rs --pos={0.1,0.2,0.3,0.4,0.5,0.6}”
	/// 
	/// 指定复位速度，单位是额定速度的倍数，参数应在0到1之间，默认为0.1：
	/// + 将1号电机复位到0.5，速度为额定速度的20\%：“rs -m=1 --pos=0.5 --vel=0.2”
	/// + 将所有电机复位到0处，速度为额定的5\%：“rs -a --pos=0 --vel=0.05”
	/// + 假设系统中有6个电机，速度和位置都是0.1到0.6：“rs --pos={0.1,0.2,0.3,0.4,0.5,0.6} --vel={0.1,0.2,0.3,0.4,0.5,0.6}”
	///
	/// 指定复位加速度，单位是额定加速度的倍数，参数应在0到1之间，默认为0.1：
	/// + 将1号电机复位到0.5，加速度为额定速度的\20%：“rs -m=1 --pos=0.5 --acc=0.2”
	/// + 假设系统中有6个电机，加速度是0.1到0.6：“rs --acc={0.1,0.2,0.3,0.4,0.5,0.6}”
	/// 
	/// 指定复位减速度，单位是额定加速度的倍数，参数应在0到1之间，默认为0.1：
	/// + 将1号电机复位到0.5，减速度为额定速度的\20%：“rs -m=1 --pos=0.5 --dec=0.2”
	/// + 假设系统中有6个电机，减速度是0.1到0.6：“rs --dec={0.1,0.2,0.3,0.4,0.5,0.6}”
	/// 
	class Reset : public Plan
	{
	public:
		static auto Type()->const std::string & { static const std::string type("Reset"); return std::ref(type); }
		auto virtual type() const->const std::string& override { return Type(); }
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void override;
		auto virtual executeRT(PlanTarget &target)->int override;
		auto virtual collectNrt(PlanTarget &target)->void override;

		virtual ~Reset();
		explicit Reset(const std::string &name = "reset_plan");
		Reset(const Reset &);
		Reset(Reset &&);
		Reset& operator=(const Reset &);
		Reset& operator=(Reset &&);

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};
	/// \brief 恢复，消除错误但不运动
	/// 
	/// 在报错后，让模型（Model）根据控制器（Controller）读到的电机位置来更新。需要在非实时循环
	/// 计算正解（使用1号求解器），实时循环内无计算。
	///
	/// ### 参数定义 ###
	/// 无
	class Recover : public Plan
	{
	public:
		static auto Type()->const std::string & { static const std::string type("Recover"); return type; }
		auto virtual type() const->const std::string& override { return Type(); }
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void override;
		auto virtual executeRT(PlanTarget &target)->int override;
		auto virtual collectNrt(PlanTarget &target)->void override;

		virtual ~Recover();
		explicit Recover(const std::string &name = "recover_plan");
		Recover(const Recover &);
		Recover(Recover &&);
		Recover& operator=(const Recover &);
		Recover& operator=(Recover &&);
	};
	/// \brief 实时循环内休息指定时间
	/// 
	///
	/// ### 参数定义 ###
	/// 指定休息时间，单位是count（1ms），默认为1000：
	/// + 休息5000ms：“sl -c=5000” 或 “sl --count=5000”
	class Sleep : public Plan
	{
	public:
		static auto Type()->const std::string & { static const std::string type("Sleep"); return std::ref(type); }
		auto virtual type() const->const std::string& override { return Type(); }
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void override;
		auto virtual executeRT(PlanTarget &target)->int override;
		auto virtual collectNrt(PlanTarget &target)->void override;

		virtual ~Sleep();
		explicit Sleep(const std::string &name = "sleep_plan");
		Sleep(const Sleep &);
		Sleep(Sleep &&);
		Sleep& operator=(const Sleep &);
		Sleep& operator=(Sleep &&);

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};
	/// \brief 打印机器人当前所有轴的位置
	/// 
	/// 
	class Show :public Plan
	{
	public:
		static auto Type()->const std::string & { static const std::string type("Show"); return std::ref(type); }
		auto virtual type() const->const std::string& override { return Type(); }
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void override;
		auto virtual executeRT(PlanTarget &target)->int override;
		auto virtual collectNrt(PlanTarget &target)->void override;

		explicit Show(const std::string &name = "show");
	};
	/// \brief 将机器人的某根或全部轴移动到指定位置。
	/// 
	/// 
	/// ### 参数定义 ###
	///
	/// 指定电机，默认指定所有电机：
	/// + 指定所有电机：“mvaj -a” 或 “mvaj --all” 或 “mvaj”
	/// + 按照绝对地址（absID），例如绝对的 0 号电机：“mvaj -m=0” 或 “mvaj --motion_id=0”
	/// + 按照物理地址（phyID），例如物理的 2 号电机：“mvaj -p=2” 或 “mvaj --physical_id=2”
	/// + 按照从站地址（slaID），例如 5 号从站：“mvaj -s=5” 或 “mvaj --slave_id=5”
	/// 
	/// 指定目标位置，单位和电机定义有关，一般是 m （直线电机）或 rad （转动电机），默认为0.0
	/// + 指定单个电机的位置，例如指定 0 号电机走到 1.5 处：“mvaj -m=0 --pos=1.5”
	/// + 指定所有电机的位置，例如：“mvaj -a --pos={1.5,1.2,1.5,1.3,1.2,0.6}”
	///
	/// 指定目标速度，单位一般是 m/s 或 rad/s ，应永远为正数，默认为1.0
	/// + 指定单个电机的速度，例如指定 0 号电机走到 1.5 处，速度为 0.5：“mvaj -m=0 --pos=1.5 --vel=0.5”
	/// + 指定所有电机的位置，例如：“mvaj -a --pos={1.5,1.2,1.5,1.3,1.2,0.6} --vel={1.5,1.2,1.5,1.3,1.2,0.6}”
	///
	/// 指定目标加速度，单位一般是 m/s^2 或 rad/s^2 ，应永远为正数，默认为1.0
	/// + 指定单个电机的加速度，例如指定 0 号电机走到 1.5 处，速度为 0.5 ，加速度为 0.3 ：“mvaj -m=0 --pos=1.5 --vel=0.5 --acc=0.3”
	/// + 指定所有电机的加速度，例如：“mvaj -a --acc={1.5,1.2,1.5,1.3,1.2,0.6}”
	///
	/// 指定目标减速度，单位一般是 m/s^2 或 rad/s^2 ，应永远为正数，默认为1.0
	/// + 指定单个电机的加速度，例如指定 0 号电机走到 1.5 处，速度为 0.5 ，加速度为 0.3 ，减速度为 0.2 ：“mvaj -m=0 --pos=1.5 --vel=0.5 --acc=0.3 --dec=0.2”
	/// + 指定所有电机的加速度，例如：“mvaj -a --dec={1.5,1.2,1.5,1.3,1.2,0.6}”
	///
	class MoveAbsJ :public Plan
	{
	public:
		

		static auto Type()->const std::string & { static const std::string type("MoveAbsJ"); return std::ref(type); }
		auto virtual type() const->const std::string& override { return Type(); }
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void override;
		auto virtual executeRT(PlanTarget &target)->int override;
		auto virtual collectNrt(PlanTarget &target)->void override;

		virtual ~MoveAbsJ();
		explicit MoveAbsJ(const std::string &name = "move_abs_j");
		MoveAbsJ(const MoveAbsJ &);
		MoveAbsJ(MoveAbsJ &&);
		MoveAbsJ& operator=(const MoveAbsJ &);
		MoveAbsJ& operator=(MoveAbsJ &&);

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};
	/// \brief 将机器人从轴空间移动到某个位姿。
	/// 
	/// 
	/// ### 参数定义 ###
	///
	/// 指定目标位姿，可以使用以下参数：
	/// + 位置与四元数，例如位置 xyz 为[0 , 0.5 , 1.1]，姿态为原始姿态[0 , 0 , 0 , 1]：“mvj --pe={0,0.5,1.1,0,0,0,1}”
	/// + 位姿矩阵，位姿仍然如上：“mvj --pm={1,0,0,0,0,1,0,0.5,0,0,1,1.1,0,0,0,1}”
	/// + 位置与欧拉角，位置姿态仍然如上：“mvj --pm={0,0.5,1.1,0,0,0}”
	/// 
	/// 指定目标速度，单位一般是 m/s 或 rad/s ，应永远为正数，默认为0.1
	/// + 指定单个电机的速度，例如指定 0 号电机走到 1.5 处，速度为 0.5：“mvaj -m=0 --pos=1.5 --vel=0.5”
	/// + 指定所有电机的位置，例如：“mvaj -a --pos={1.5,1.2,1.5,1.3,1.2,0.6} --vel={1.5,1.2,1.5,1.3,1.2,0.6}”
	///
	/// 指定目标加速度，单位一般是 m/s^2 或 rad/s^2 ，应永远为正数，默认为0.1
	/// + 指定单个电机的加速度，例如指定 0 号电机走到 1.5 处，速度为 0.5 ，加速度为 0.3 ：“mvaj -m=0 --pos=1.5 --vel=0.5 --acc=0.3”
	/// + 指定所有电机的加速度，例如：“mvaj -a --acc={1.5,1.2,1.5,1.3,1.2,0.6}”
	///
	/// 指定目标减速度，单位一般是 m/s^2 或 rad/s^2 ，应永远为正数，默认为0.1
	/// + 指定单个电机的加速度，例如指定 0 号电机走到 1.5 处，速度为 0.5 ，加速度为 0.3 ，减速度为 0.2 ：“mvaj -m=0 --pos=1.5 --vel=0.5 --acc=0.3 --dec=0.2”
	/// + 指定所有电机的加速度，例如：“mvaj -a --dec={1.5,1.2,1.5,1.3,1.2,0.6}”
	///
	class MoveJ : public Plan
	{
	public:
		static auto Type()->const std::string & { static const std::string type("MoveJ"); return std::ref(type); }
		auto virtual type() const->const std::string& override { return Type(); }
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void override;
		auto virtual executeRT(PlanTarget &target)->int override;
		auto virtual collectNrt(PlanTarget &target)->void override;

		virtual ~MoveJ();
		explicit MoveJ(const std::string &name = "moveL");
		MoveJ(const MoveJ &);
		MoveJ(MoveJ &&);
		MoveJ& operator=(const MoveJ &);
		MoveJ& operator=(MoveJ &&);

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};
	/// \brief 将机器人从笛卡尔空间线性移动到某个位姿。
	/// 
	/// 
	/// ### 参数定义 ###
	///
	/// 指定目标位姿，可以使用以下参数：
	/// + 位置与四元数，例如位置 xyz 为[0 , 0.5 , 1.1]，姿态为原始姿态[0 , 0 , 0 , 1]：“mvl --pq={0,0.5,1.1,0,0,0,1}”
	/// + 位姿矩阵，位姿仍然如上：“mvl --pm={1,0,0,0,0,1,0,0.5,0,0,1,1.1,0,0,0,1}”
	/// + 位置与欧拉角，位置姿态仍然如上：“mvl --pe={0,0.5,1.1,0,0,0}”
	///
	/// 此外，还可以指定位置和角度的单位，长度单位默认为 m ，角度默认为 rad ：
	/// + 指定位置单位，例如将单位设置为 m （米）：“mvl --pq={0,0.5,1.1,0,0,0,1} --pos_unit=m”
	/// + 指定角度单位，例如将单位设置为 rad ：“mvl --pe={0,0.5,1.1,0,0,0} --ori_unit=rad”
	///
	/// 还可以指定欧拉角的种类，可以是 321 313 123 212 ... 等任意类型的欧拉角，默认为 321 的欧拉角
	/// + 指定欧拉角为 123 的，“mvl --pe={0,0.5,1.1,0,0,0} --ori_unit=rad --eul_type=321”
	///
	/// 指定目标速度以及角速度，单位一般是 m/s 或 rad/s ，应永远为正数，默认为0.1
	/// + 指定末端的速度和角速度，例如指定末端速度为 0.5，角速度为 0.3：“mvl --pe={0.1,1.2,0,0,0,0} --vel=0.5 --angular_vel=0.3”
	///
	/// 指定目标加速度以及角加速度，单位一般是 m/s^2 或 rad/s^2 ，应永远为正数，默认为0.1
	/// + 指定末端的加速度和角加速度，例如指定末端加速度为 0.5，角加速度为 0.3：“mvl --pe={0.1,1.2,0,0,0,0} --acc=0.5 --angular_acc=0.3”
	///
	/// 指定目标减速度以及角减速度，单位一般是 m/s^2 或 rad/s^2 ，应永远为正数，默认为0.1
	/// + 指定末端的减速度和角减速度，例如指定末端减速度为 0.5，角减速度为 0.3：“mvl --pe={0.1,1.2,0,0,0,0} --dec=0.5 --angular_dec=0.3”
	class MoveL : public Plan
	{
	public:
		static auto Type()->const std::string & { static const std::string type("MoveL"); return std::ref(type); }
		auto virtual type() const->const std::string& override { return Type(); }
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void override;
		auto virtual executeRT(PlanTarget &target)->int override;
		auto virtual collectNrt(PlanTarget &target)->void override;

		virtual ~MoveL();
		explicit MoveL(const std::string &name = "moveL");
		MoveL(const MoveL &);
		MoveL(MoveL &&);
		MoveL& operator=(const MoveL &);
		MoveL& operator=(MoveL &&);

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};
	/// \brief 让机器人自动运行，随时可以改变其目标位姿。
	/// 
	/// 
	/// ### 参数定义 ###
	///
	/// 指定目标位姿，可以使用以下参数：
	/// + 位置与四元数，例如位置 xyz 为[0 , 0.5 , 1.1]，姿态为原始姿态[0 , 0 , 0 , 1]：“mvl --pq={0,0.5,1.1,0,0,0,1}”
	/// + 位姿矩阵，位姿仍然如上：“mvl --pm={1,0,0,0,0,1,0,0.5,0,0,1,1.1,0,0,0,1}”
	/// + 位置与欧拉角，位置姿态仍然如上：“mvl --pe={0,0.5,1.1,0,0,0}”
	///
	/// 此外，还可以指定位置和角度的单位，长度单位默认为 m ，角度默认为 rad ：
	/// + 指定位置单位，例如将单位设置为 m （米）：“mvl --pq={0,0.5,1.1,0,0,0,1} --pos_unit=m”
	/// + 指定角度单位，例如将单位设置为 rad ：“mvl --pe={0,0.5,1.1,0,0,0} --ori_unit=rad”
	///
	/// 还可以指定欧拉角的种类，可以是 321 313 123 212 ... 等任意类型的欧拉角，默认为 321 的欧拉角
	/// + 指定欧拉角为 123 的，“mvl --pe={0,0.5,1.1,0,0,0} --ori_unit=rad --eul_type=321”
	///
	/// 指定目标速度以及角速度，单位一般是 m/s 或 rad/s ，应永远为正数，默认为0.1
	/// + 指定末端的速度和角速度，例如指定末端速度为 0.5，角速度为 0.3：“mvl --pe={0.1,1.2,0,0,0,0} --vel=0.5 --angular_vel=0.3”
	///
	/// 指定目标加速度以及角加速度，单位一般是 m/s^2 或 rad/s^2 ，应永远为正数，默认为0.1
	/// + 指定末端的加速度和角加速度，例如指定末端加速度为 0.5，角加速度为 0.3：“mvl --pe={0.1,1.2,0,0,0,0} --acc=0.5 --angular_acc=0.3”
	///
	/// 指定目标减速度以及角减速度，单位一般是 m/s^2 或 rad/s^2 ，应永远为正数，默认为0.1
	/// + 指定末端的减速度和角减速度，例如指定末端减速度为 0.5，角减速度为 0.3：“mvl --pe={0.1,1.2,0,0,0,0} --dec=0.5 --angular_dec=0.3”
	class AutoMove :public Plan
	{
	public:
		static auto Type()->const std::string & { static const std::string type("AutoMove"); return std::ref(type); }
		auto virtual type() const->const std::string& override { return Type(); }
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void override;
		auto virtual executeRT(PlanTarget &target)->int override;
		auto virtual collectNrt(PlanTarget &target)->void override;

		virtual ~AutoMove();
		explicit AutoMove(const std::string &name = "am");
		AutoMove(const AutoMove &);
		AutoMove(AutoMove &&);
		AutoMove& operator=(const AutoMove &);
		AutoMove& operator=(AutoMove &&);

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};
	
	
	
	class ManualMove :public Plan
	{
	public:
		static auto Type()->const std::string & { static const std::string type("ManualMove"); return std::ref(type); }
		auto virtual type() const->const std::string& override { return Type(); }
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void override;
		auto virtual executeRT(PlanTarget &target)->int override;
		auto virtual collectNrt(PlanTarget &target)->void override;

		virtual ~ManualMove();
		explicit ManualMove(const std::string &name = "mm");
		ManualMove(const ManualMove &);
		ManualMove(ManualMove &&);
		ManualMove& operator=(const ManualMove &);
		ManualMove& operator=(ManualMove &&);

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};








	class UniversalPlan :public Plan
	{
	public:
		using PrepairFunc = std::function<void(const std::map<std::string, std::string> &params, PlanTarget &target)>;
		using ExecuteFunc = std::function<int(const PlanTarget &param)>;
		using CollectFunc = std::function<void(PlanTarget &param)>;

		static auto Type()->const std::string & { static const std::string type("UniversalPlan"); return std::ref(type); }
		auto virtual type() const->const std::string& override { return Type(); }
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void override;
		auto virtual executeRT(PlanTarget &target)->int override;
		auto virtual collectNrt(PlanTarget &target)->void override;
		auto virtual setPrepairFunc(PrepairFunc func)->void;
		auto virtual setExecuteFunc(ExecuteFunc func)->void;
		auto virtual setCollectFunc(CollectFunc func)->void;

		virtual ~UniversalPlan();
		explicit UniversalPlan(const std::string &name = "universal_plan", PrepairFunc prepair_func = nullptr, ExecuteFunc execute_func = nullptr, CollectFunc collect_func = nullptr, const std::string & cmd_xml_str = "<universal_plan/>");
		UniversalPlan(const UniversalPlan &);
		UniversalPlan(UniversalPlan &&);
		UniversalPlan& operator=(const UniversalPlan &);
		UniversalPlan& operator=(UniversalPlan &&);

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};
}

#endif