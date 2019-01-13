#ifndef ARIS_PLAN_ROOT_H_
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
	/// 让电机使能，可以按照以下参数指定电机：
	/// + 使能所有电机：“en -a” 或 “en --all”
	/// + 按照绝对地址（absID），例如绝对的 0 号电机：“en -m=0” 或 “en --motion_id=0”
	/// + 按照物理地址（phyID），例如物理的 2 号电机：“en -p=2” 或 “en --physical_id=2”
	/// + 按照从站地址（slaID），例如 5 号从站：“en -s=5” 或 “en --slave_id=5”
	/// 
	/// 指定本指令的最长运行时间（默认为5000ms）：
	/// + 使能0号电机，并指定其最长时间为5000ms： “en -m=0 --limit_time=5000”
	/// 
	class EnablePlan : public Plan
	{
	public:
		static auto Type()->const std::string & { static const std::string type("EnablePlan"); return std::ref(type); }
		auto virtual type() const->const std::string& override { return Type(); }
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void override;
		auto virtual executeRT(PlanTarget &target)->int override;
		auto virtual collectNrt(PlanTarget &target)->void override;

		virtual ~EnablePlan();
		explicit EnablePlan(const std::string &name = "enable_plan");
		EnablePlan(const EnablePlan &);
		EnablePlan(EnablePlan &&);
		EnablePlan& operator=(const EnablePlan &);
		EnablePlan& operator=(EnablePlan &&);

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};
	/// \brief 让电机去使能
	/// 
	/// 让电机去使能，可以按照以下参数指定电机：
	/// + 使能所有电机：“ds -a” 或 “ds --all”
	/// + 按照绝对地址（absID），例如绝对的 0 号电机：“ds -m=0” 或 “ds --motion_id=0”
	/// + 按照物理地址（phyID），例如物理的 2 号电机：“ds -p=2” 或 “ds --physical_id=2”
	/// + 按照从站地址（slaID），例如 5 号从站：“ds -s=5” 或 “ds --slave_id=5”
	/// 
	/// 指定本指令的最长运行时间（默认为5000ms）：
	/// + 去使能0号电机，并指定其最长时间为5000ms： “ds -m=0 --limit_time=5000”
	/// 
	class DisablePlan : public Plan
	{
	public:
		static auto Type()->const std::string & { static const std::string type("EnablePlan"); return std::ref(type); }
		auto virtual type() const->const std::string& override { return Type(); }
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void override;
		auto virtual executeRT(PlanTarget &target)->int override;
		auto virtual collectNrt(PlanTarget &target)->void override;

		virtual ~DisablePlan();
		explicit DisablePlan(const std::string &name = "enable_plan");
		DisablePlan(const DisablePlan &);
		DisablePlan(DisablePlan &&);
		DisablePlan& operator=(const DisablePlan &);
		DisablePlan& operator=(DisablePlan &&);

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};
	/// \brief 让电机回零
	/// 
	/// 让电机使能，可以按照以下参数指定电机：
	/// + 使能所有电机：“hm -a” 或 “en --all”
	/// + 按照绝对地址（absID），例如绝对的 0 号电机：“en -m=0” 或 “en --motion_id=0”
	/// + 按照物理地址（phyID），例如物理的 2 号电机：“en -p=2” 或 “en --physical_id=2”
	/// + 按照从站地址（slaID），例如 5 号从站：“en -s=5” 或 “en --slave_id=5”
	/// 
	/// 指定本指令的最长运行时间（默认为5000ms）：
	/// + 使能0号电机，并指定其最长时间为5000ms： “en -m=0 --limit_time=5000”
	/// 
	class HomePlan : public Plan
	{
	public:
		static auto Type()->const std::string & { static const std::string type("HomePlan"); return std::ref(type); }
		auto virtual type() const->const std::string& override { return Type(); }
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void override;
		auto virtual executeRT(PlanTarget &target)->int override;
		auto virtual collectNrt(PlanTarget &target)->void override;

		virtual ~HomePlan();
		explicit HomePlan(const std::string &name = "home_plan");
		HomePlan(const HomePlan &);
		HomePlan(HomePlan &&);
		HomePlan& operator=(const HomePlan &);
		HomePlan& operator=(HomePlan &&);

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};
	/// \brief 让电机切换模式
	/// 
	/// 让电机切换模式，可以按照以下参数指定电机：
	/// + 切换所有电机：“md -a” 或 “md --all”
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
	class ModePlan : public Plan
	{
	public:
		static auto Type()->const std::string & { static const std::string type("ModePlan"); return std::ref(type); }
		auto virtual type() const->const std::string& override { return Type(); }
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void override;
		auto virtual executeRT(PlanTarget &target)->int override;
		auto virtual collectNrt(PlanTarget &target)->void override;

		virtual ~ModePlan();
		explicit ModePlan(const std::string &name = "mode_plan");
		ModePlan(const ModePlan &);
		ModePlan(ModePlan &&);
		ModePlan& operator=(const ModePlan &);
		ModePlan& operator=(ModePlan &&);

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};
	/// \brief 复位，机器人从轴空间按照指定速度运行到指定位置处
	/// 
	/// 
	class ResetPlan : public Plan
	{
	public:
		static auto Type()->const std::string & { static const std::string type("ResetPlan"); return std::ref(type); }
		auto virtual type() const->const std::string& override { return Type(); }
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void override;
		auto virtual executeRT(PlanTarget &target)->int override;
		auto virtual collectNrt(PlanTarget &target)->void override;

		virtual ~ResetPlan();
		explicit ResetPlan(const std::string &name = "reset_plan");
		ResetPlan(const ResetPlan &);
		ResetPlan(ResetPlan &&);
		ResetPlan& operator=(const ResetPlan &);
		ResetPlan& operator=(ResetPlan &&);

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};
	/// \brief 恢复，消除错误但不运动，让机器人恢复
	/// 
	/// 在报错后，让模型（Model）根据控制器（Controller）读到的电机位置来更新。需要在非实时循环
	/// 计算正解（使用1号求解器），实时循环内无计算。
	///
	/// 指令无参数
	class RecoverPlan : public Plan
	{
	public:
		static auto Type()->const std::string & { static const std::string type("RecoverPlan"); return type; }
		auto virtual type() const->const std::string& override { return Type(); }
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void override;
		auto virtual executeRT(PlanTarget &target)->int override;
		auto virtual collectNrt(PlanTarget &target)->void override;

		virtual ~RecoverPlan();
		explicit RecoverPlan(const std::string &name = "recover_plan");
		RecoverPlan(const RecoverPlan &);
		RecoverPlan(RecoverPlan &&);
		RecoverPlan& operator=(const RecoverPlan &);
		RecoverPlan& operator=(RecoverPlan &&);
	};
	class SleepPlan : public Plan
	{
	public:
		static auto Type()->const std::string & { static const std::string type("SleepPlan"); return std::ref(type); }
		auto virtual type() const->const std::string& override { return Type(); }
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void override;
		auto virtual executeRT(PlanTarget &target)->int override;
		auto virtual collectNrt(PlanTarget &target)->void override;

		virtual ~SleepPlan();
		explicit SleepPlan(const std::string &name = "sleep_plan");
		SleepPlan(const SleepPlan &);
		SleepPlan(SleepPlan &&);
		SleepPlan& operator=(const SleepPlan &);
		SleepPlan& operator=(SleepPlan &&);

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};
	
	class MovePlan : public Plan
	{
	public:
		static auto Type()->const std::string & { static const std::string type("MovePlan"); return std::ref(type); }
		auto virtual type() const->const std::string& override { return Type(); }
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void override;
		auto virtual executeRT(PlanTarget &target)->int override;
		auto virtual collectNrt(PlanTarget &target)->void override;

		virtual ~MovePlan();
		explicit MovePlan(const std::string &name = "move_plan");
		MovePlan(const MovePlan &);
		MovePlan(MovePlan &&);
		MovePlan& operator=(const MovePlan &);
		MovePlan& operator=(MovePlan &&);

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

	class MoveJ :public Plan
	{
	public:
		struct Param
		{
			double vel, acc, dec;
			std::vector<double> joint_pos_vec, begin_joint_pos_vec;
			std::vector<bool> joint_active_vec;
		};
		
		static auto Type()->const std::string & { static const std::string type("MoveJ"); return std::ref(type); }
		auto virtual type() const->const std::string& override { return Type(); }
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void override;
		auto virtual executeRT(PlanTarget &target)->int override;
		auto virtual collectNrt(PlanTarget &target)->void override;

		virtual ~MoveJ();
		explicit MoveJ(const std::string &name = "move_plan");
		MoveJ(const MoveJ &);
		MoveJ(MoveJ &&);
		MoveJ& operator=(const MoveJ &);
		MoveJ& operator=(MoveJ &&);
		
	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};
	class Show :public Plan
	{
	public:
		static auto Type()->const std::string & { static const std::string type("MoveJ"); return std::ref(type); }
		auto virtual type() const->const std::string& override { return Type(); }
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void override;
		auto virtual executeRT(PlanTarget &target)->int override;
		auto virtual collectNrt(PlanTarget &target)->void override;

		explicit Show(const std::string &name = "move_plan");
	};

	class MvL :public Plan 
	{
	public:
		static auto Type()->const std::string & { static const std::string type("MvL"); return std::ref(type); }
		auto virtual type() const->const std::string& override { return Type(); }
		auto virtual prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void override;
		auto virtual executeRT(PlanTarget &target)->int override;
		auto virtual collectNrt(PlanTarget &target)->void override;

		virtual ~MvL();
		explicit MvL(const std::string &name = "mvl");
		MvL(const MvL &);
		MvL(MvL &&);
		MvL& operator=(const MvL &);
		MvL& operator=(MvL &&);


	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;



	};

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


}

#endif