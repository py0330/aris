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
	class RecoverPlan : public Plan
	{
	public:
		static auto Type()->const std::string & { static const std::string type("RecoverPlan"); return std::ref(type); }
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


	using PathPlanFunction = std::function<void(double s, double ds, aris::dynamic::Model *model)>;
	class OptimalTrajectory
	{
	public:
		struct MotionLimit { double max_vel, min_vel, max_acc, min_acc, max_tor, min_tor, max_jerk, min_jerk; };
		struct Node { double time, s, ds, dds; };

		template <typename LimitArray>
		auto setMotionLimit(LimitArray limits)->void { motor_limits.assign(limits.begin(), limits.end()); }
		auto setBeginNode(Node node)->void { beg_ = node; }
		auto setEndNode(Node node)->void { end_ = node; }
		auto setFunction(const PathPlanFunction &path_plan)->void { this->plan = path_plan; }
		auto setSolver(aris::dynamic::InverseKinematicSolver *solver)->void { this->solver = solver; }
		auto setModel(aris::dynamic::Model *model)->void { this->model = model; }
		auto result()->std::vector<double>& { return result_; }
		auto run()->void 
		{
			// 初始化 //
			list.clear();
			list.push_front(beg_);

			l_ = list.begin();
			l_beg_ = l_;

			failed_s = 0;

			while (l_->s < 1.0 && failed_s < 1.0)
			{
				l_ = list.insert(list.end(), Node());
				l_->ds = std::prev(l_)->ds + std::prev(l_)->dds * dt;
				l_->s = std::prev(l_)->s + std::prev(l_)->ds * dt + 0.5 * std::prev(l_)->dds * dt * dt;

				double max_dds, min_dds;
				if (cptDdsConstraint(l_->s, l_->ds, max_dds, min_dds))
				{
					l_->dds = max_dds;
				}
				else
				{
					testForward();
				}

				std::cout << 1.0 - l_->s << std::endl;
			}

			join();
		}

		virtual ~OptimalTrajectory() = default;
		explicit OptimalTrajectory() = default;
		OptimalTrajectory(const OptimalTrajectory&) = default;
		OptimalTrajectory(OptimalTrajectory&&) = default;
		OptimalTrajectory& operator=(const OptimalTrajectory&) = default;
		OptimalTrajectory& operator=(OptimalTrajectory&&) = default;

	public:
		auto testForward()->void
		{
			failed_s = l_->s;
			auto l_end_ = l_;

			Size test_distance;
			// 二分法，结束时，l_beg_为正好可以成功的，l_end_为正好不能成功的
			while (std::distance(l_beg_, l_end_) > 1)
			{
				// init //
				std::list<Node> test_list;
				auto mid_iter = std::next(l_beg_, std::distance(l_beg_, l_end_) / 2);
				test_list.push_back(*mid_iter);

				// 测试mid_iter是否会成功 //
				bool test_successful{ true };
				while (test_list.back().ds > 0.0)
				{
					Node node;
					node.ds = test_list.back().ds + test_list.back().dds * dt;
					node.s = test_list.back().s + test_list.back().ds * dt + 0.5 * test_list.back().dds * dt * dt;

					test_list.push_back(node);

					double max_dds, min_dds;
					if (cptDdsConstraint(node.s, node.ds, max_dds, min_dds))
					{
						test_list.back().dds = min_dds;
					}
					else
					{
						test_successful = false;
						break;
					}
				}

				// 测试成功，则左侧置为test_iter,否则右侧置为test_iter
				if (test_successful)
				{
					l_beg_ = mid_iter;
				}
				else
				{
					l_end_ = mid_iter;
					failed_s = test_list.back().s;
					test_distance = test_list.size();
				}
			}

			// 消除原先的iter们，开始新的iter//
			list.erase(std::next(l_beg_), list.end());
			l_ = l_beg_;

			for (Size i = 0; i < test_distance && l_->ds > 0.0 && l_->s < failed_s; ++i)
			{
				l_ = list.insert(list.end(), Node());

				l_->ds = std::prev(l_)->ds + std::prev(l_)->dds * dt;
				l_->s = std::prev(l_)->s + std::prev(l_)->ds * dt + 0.5 * std::prev(l_)->dds * dt * dt;

				double max_dds, min_dds;
				cptDdsConstraint(l_->s, l_->ds, max_dds, min_dds);

				l_->dds = min_dds;
			}

			l_beg_ = l_;
		}
		auto join()->void
		{
			auto coe = 1.0 / list.back().s;
			for (auto &node : list)node.s *= coe;
		}
		auto cptDdsConstraint(double s, double ds, double &max_dds, double &min_dds)->bool 
		{
			plan(s, ds, model);
			solver->kinPos();
			solver->kinVel();
			solver->cptJacobi();

			aris::dynamic::dsp(solver->mJi(), solver->nJi(), solver->Ji());

			max_dds = 0.1 + 0.2*s + 4 * (s - 0.5)*(s - 0.5) - ds * 1 - ds * ds*0.2 - sin(ds)*0.2;
			min_dds = 0 - 0.3*(1 - s) - 4 * (s - 0.5)*(s - 0.5) + ds * 0.1 + ds * ds*0.8 + sin(ds)*0.1;

			return max_dds > min_dds && s < 1.0;
		}

		double failed_s;

		PathPlanFunction plan;
		aris::dynamic::InverseKinematicSolver *solver;
		aris::dynamic::Model *model;

		Node beg_, end_;
		std::list<Node> list;
		std::list<Node>::iterator l_beg_, l_;
		std::vector<MotionLimit> motor_limits;

		std::vector<double> Ji_data_;

		std::vector<double> result_;


		static inline const double dt = 1e-3;
	};



}

#endif