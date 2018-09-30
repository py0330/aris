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


	using PathPlanFunction = std::function<void(double s, double ds, std::vector<double> &x, std::vector<double> &dx_ds, std::vector<double> &ddx_ds2)>;
	class OptimalTrajectory
	{
	public:
		struct MotionLimit { double max_vel, min_vel, max_acc, min_acc, max_tor, min_tor, max_jerk, min_jerk; };
		struct Node { double s, ds, dds; };

		template <typename LimitArray>
		auto setMotionLimit(LimitArray limits)->void { motor_limits.assign(limits.begin(), limits.end()); }
		auto setBeginNode(Node node)->void { beg_ = node; }
		auto setEndNode(Node node)->void { end_ = node; }
		auto setFunction(const PathPlanFunction &path_plan)->void { this->plan = path_plan; }
		auto setSolver(aris::dynamic::InverseKinematicSolver *solver)->void { this->solver = solver; }
		auto setModel(aris::dynamic::Model *model)->void { this->model = model; }
		auto run()->void;
		auto result()->std::list<Node> { return list_; };

		virtual ~OptimalTrajectory();
		explicit OptimalTrajectory();
		OptimalTrajectory(const OptimalTrajectory&);
		OptimalTrajectory(OptimalTrajectory&&);
		OptimalTrajectory& operator=(const OptimalTrajectory&);
		OptimalTrajectory& operator=(OptimalTrajectory&&);

	public:
		auto testForward()->void
		{
			double failed_s;
			Size test_distance;
			// 二分法求得肯定能成功的节点 //
			l_sure_ = std::prev(std::upper_bound(l_sure_, l.end(), nullptr, [&](const nullptr_t &a, const Node &b)
			{
				std::list<Node> test_list;
				test_list.push_back(b);

				bool test_successful{ true };
				while (test_list.back().ds > 0.0)
				{
					double max_dds, min_dds;
					if (cptDdsConstraint(test_list.back().s, test_list.back().ds, max_dds, min_dds, true))
					{
						test_list.back().dds = min_dds;
						Node node;
						node.ds = test_list.back().ds + test_list.back().dds * dt;
						node.s = test_list.back().s + test_list.back().ds * dt + 0.5 * test_list.back().dds * dt * dt;
						test_list.push_back(node);
					}
					else
					{
						test_successful = false;
						break;
					}
				}

				if (!test_successful) 
				{
					test_distance = test_list.size(); 
					failed_s = test_list.back().s;
				}

				return !test_successful;
			}));

			// 消除原先的iter们，开始新的iter//
			l.erase(std::next(l_sure_), l.end());
			for (Size i = 0; i < test_distance && l.back().ds > 0.0 && l.back().s < failed_s; ++i)
			{
				double max_dds, min_dds;
				cptDdsConstraint(l.back().s, l.back().ds, max_dds, min_dds, true);
				l.back().dds = min_dds;

				auto ins = l.insert(l.end(), Node());
				ins->ds = std::prev(ins)->ds + std::prev(ins)->dds * dt;
				ins->s = std::prev(ins)->s + std::prev(ins)->ds * dt + 0.5 * std::prev(ins)->dds * dt * dt;
			}
		}
		auto testBackward()->void
		{
			double failed_s;
			Size test_distance;
			// 二分法求得肯定能成功的节点 //
			r_sure_ = std::lower_bound(r.begin(), r_sure_, nullptr, [&](const Node &b, const nullptr_t &a)
			{
				std::list<Node> test_list;
				test_list.push_back(b);

				bool test_successful{ true };
				while (test_list.front().ds > 0.0)
				{
					double max_dds, min_dds;
					if (cptDdsConstraint(test_list.front().s, test_list.front().ds, max_dds, min_dds, false))
					{
						auto ins = test_list.insert(test_list.begin(), Node());
						ins->dds = max_dds;
						ins->ds = std::next(ins)->ds - ins->dds * dt;
						ins->s = std::next(ins)->s - std::next(ins)->ds * dt + 0.5 * ins->dds * dt * dt;
					}
					else
					{
						test_successful = false;
						break;
					}
				}

				if (!test_successful)
				{
					test_distance = test_list.size();
					failed_s = test_list.front().s;
				}

				return !test_successful;
			});

			// 消除原先的iter们，开始新的iter//
			r.erase(r.begin(), r_sure_);
			for (Size i = 0; i < test_distance && r.front().ds > 0.0 && r.front().s > failed_s; ++i)
			{
				double max_dds, min_dds;
				cptDdsConstraint(r.front().s, r.front().ds, max_dds, min_dds, false);
				

				auto ins = r.insert(r.begin(), Node());
				ins->dds = max_dds;
				ins->ds = std::next(ins)->ds - ins->dds * dt;
				ins->s = std::next(ins)->s - std::next(ins)->ds * dt + 0.5 * ins->dds * dt * dt;
			}
		}
		auto join()->void
		{
			// 让最后两个相邻节点的ds连续 //
			for (;;)
			{
				// 判断是否满足连续的条件 //
				double max_dds, min_dds;
				cptDdsConstraint(l.back().s, l.back().ds, max_dds, min_dds, true);
				double require_dds = (r.front().ds - l.back().ds) / dt;
				if (require_dds < max_dds && require_dds > min_dds) 
				{
					l.back().dds = require_dds;
					break;
				}

				// 开始清理并迭代连接 //
				if (l.back().ds > r.front().ds)
				{
					if (l.size() == 1)throw std::runtime_error(std::string("failed:") + __FILE__);
					
					l.erase(std::prev(l.end()));

					while (r.front().s > l.back().s)
					{
						cptDdsConstraint(r.front().s, r.front().ds, max_dds, min_dds, false);
						auto ins = r.insert(r.begin(), Node());
						ins->dds = min_dds;
						ins->ds = std::next(ins)->ds - ins->dds * dt;
						ins->s = std::next(ins)->s - std::next(ins)->ds * dt + 0.5 * ins->dds * dt * dt;
					}

					r.erase(r.begin());
				}
				else
				{
					if (r.size() == 1)throw std::runtime_error(std::string("failed:") + __FILE__);

					r.erase(r.begin());

					while (r.front().s > l.back().s)
					{
						cptDdsConstraint(l.back().s, l.back().ds, max_dds, min_dds, true);
						l.back().dds = max_dds;
						auto ins = l.insert(l.end(), Node());
						ins->ds = std::prev(ins)->ds + std::prev(ins)->dds * dt;
						ins->s = std::prev(ins)->s + std::prev(ins)->ds * dt + 0.5 * std::prev(ins)->dds * dt * dt;
					}

					l.erase(std::prev(l.end()));
				}
			}

			// 让最后两个相邻节点的s连续 //
			double error = l.back().s + l.back().ds * dt + 0.5 * l.back().dds * dt * dt - r.front().s;
			for (auto &node : r)node.s += error;

			list_ = std::move(l);
			list_.splice(list_.end(), r);

			for (auto &node : list_)
			{
				node.s *= 1.0 / (1.0 + error);
			}
		}
		auto cptDdsConstraint(double s, double ds, double &max_dds, double &min_dds, bool is_forward)->bool
		{
			// 计算雅可比 //
			std::vector<double> pq, dpq_ds, ddpq_ds2;
			plan(s, ds, pq, dpq_ds, ddpq_ds2);

			for (Size i = 0; i < model->generalMotionPool().size(); ++i)
			{
				double vs[6], as[6];
				aris::dynamic::s_aq2as(pq.data() + 7 * i, dpq_ds.data() + 7 * i, ddpq_ds2.data() + 7 * i, as, vs);

				double real_vs[6];
				aris::dynamic::s_vc(6, ds, vs, real_vs);

				model->generalMotionPool().at(i).setMpq(pq.data() + 7 * i);
				model->generalMotionPool().at(i).setMvs(real_vs);
			}

			solver->kinPos();
			solver->kinVel();
			solver->cptJacobi();

			// 开始计算不等式 //
			std::vector<double> amin, amax, vmin, vmax;
			for (auto &limit : motor_limits)
			{
				amin.push_back(limit.min_acc);
				amax.push_back(limit.max_acc);
				vmin.push_back(limit.min_vel);
				vmax.push_back(limit.max_vel);
			}

			// 根据加速度限制列不等式 //
			auto la = amin;
			aris::dynamic::s_mma(solver->mJi(), 1, solver->nJi(), -ds * ds, solver->Ji(), ddpq_ds2.data(), la.data());
			aris::dynamic::s_vs(solver->mJi(), solver->ci(), la.data());

			auto ra = amax;
			aris::dynamic::s_mma(solver->mJi(), 1, solver->nJi(), -ds * ds, solver->Ji(), ddpq_ds2.data(), ra.data());
			aris::dynamic::s_vs(solver->mJi(), solver->ci(), ra.data());

			std::vector<double> A(solver->mJi());
			aris::dynamic::s_mm(solver->mJi(), 1, solver->nJi(), solver->Ji(), dpq_ds.data(), A.data());

			// 根据速度限制列不等式 //
			auto lv = vmin;
			aris::dynamic::s_va(solver->mJi(), is_forward ? -ds : ds, A.data(), lv.data());

			auto rv = vmax;
			aris::dynamic::s_va(solver->mJi(), is_forward ? -ds : ds, A.data(), rv.data());

			std::vector<double> B(solver->mJi());
			aris::dynamic::s_vc(solver->mJi(), dt, A.data(), B.data());

			// 求解不等式 //
			max_dds = std::numeric_limits<double>::max();
			min_dds = std::numeric_limits<double>::lowest();
			for (aris::Size i = 0; i < solver->mJi(); ++i)
			{
				if (std::abs(A[i]) > 1e-10)
				{
					max_dds = A[i] > 0.0 ? std::min(ra[i] / A[i], max_dds) : std::min(la[i] / A[i], max_dds);
					min_dds = A[i] > 0.0 ? std::max(la[i] / A[i], min_dds) : std::max(ra[i] / A[i], min_dds);
				}
				if (std::abs(B[i]) > 1e-10)
				{
					max_dds = B[i] > 0.0 ? std::min(rv[i] / B[i], max_dds) : std::min(lv[i] / B[i], max_dds);
					min_dds = B[i] > 0.0 ? std::max(lv[i] / B[i], min_dds) : std::max(rv[i] / B[i], min_dds);
				}
			}

			return max_dds > min_dds;
		}

		
		aris::dynamic::Model *model;
		aris::dynamic::InverseKinematicSolver *solver;
		
		PathPlanFunction plan;
		Node beg_, end_;
		std::vector<MotionLimit> motor_limits;

		static inline const double dt = 1e-3;

		std::list<Node> list_;
		std::list<Node> l, r;
		std::list<Node>::iterator l_sure_, r_sure_;

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};
}

#endif