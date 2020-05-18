#ifndef ARIS_PLAN_ALGORITHM_H_
#define ARIS_PLAN_ALGORITHM_H_

#include <list>
#include <cmath>
#include <iostream>
#include <functional>
#include <map>
#include <any>

//#include <aris/core/core.hpp>
//#include <aris/control/control.hpp>
#include <aris/dynamic/dynamic.hpp>

/// \brief 轨迹规划命名空间
/// \ingroup aris
/// 
///
///
namespace aris::plan
{
	using PathPlanFunction = std::function<void(double s, double ds, std::vector<double> &x, std::vector<double> &dx_ds, std::vector<double> &ddx_ds2)>;
	class ARIS_API OptimalTrajectory
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
					if (l.size() == 1)THROW_FILE_LINE("failed");
					
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
					if (r.size() == 1)THROW_FILE_LINE(std::string("failed:") + __FILE__);

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