#ifndef ARIS_PLAN_ROOT_H_
#define ARIS_PLAN_ROOT_H_

#include <list>
#include <cmath>
#include <iostream>
#include <functional>
#include <map>
#include <any>

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
		aris::dynamic::Model* model;
		aris::control::Master* master;
		std::int64_t command_id;
		std::int64_t option;
		std::uint32_t count;
		std::any param;
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
	class HomePlan : public Plan{};
	// rs 会让电机走到指定位置处，同时将模型中的电机位置也变成指定位置，过程中会计算正解 //
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
	// rc 不会让电机动，只会重新排列正反解
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
		explicit SleepPlan(const std::string &name = "reset_plan");
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
}

#endif