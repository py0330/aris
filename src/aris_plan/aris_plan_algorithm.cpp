#include <algorithm>

#include"aris_plan_algorithm.h"

namespace aris::plan
{
	struct OptimalTrajectory::Imp
	{
		
	};
	auto OptimalTrajectory::run()->void
	{
		// 初始化 //
		l.clear();
		l.push_back(beg_);
		r.clear();
		r.push_back(end_);
		l_sure_ = l.begin(); // 肯定可以成功的节点
		r_sure_ = r.begin(); // 肯定可以成功的节点

		// 开始计算 //
		while (l.back().s < r.front().s)
		{
			if (l.back().ds < r.front().ds)
			{
				double max_dds, min_dds;
				if (cptDdsConstraint(l.back().s, l.back().ds, max_dds, min_dds, true))
				{
					l.back().dds = max_dds;
					auto ins = l.insert(l.end(), Node());
					ins->ds = std::prev(ins)->ds + std::prev(ins)->dds * dt;
					ins->s = std::prev(ins)->s + std::prev(ins)->ds * dt + 0.5 * std::prev(ins)->dds * dt * dt;
				}
				else
				{
					testForward();
				}
			}
			else
			{
				double max_dds, min_dds;
				if (cptDdsConstraint(r.front().s, r.front().ds, max_dds, min_dds, false))
				{
					// 反向略有不同：加速度都放在前面的节点处
					auto ins = r.insert(r.begin(), Node());
					ins->dds = min_dds;
					ins->ds = std::next(ins)->ds - ins->dds * dt;
					ins->s = std::next(ins)->s - std::next(ins)->ds * dt + 0.5 * ins->dds * dt * dt;
				}
				else
				{
					testBackward();
				}
			}

			std::cout << r.front().s - l.back().s << std::endl;
		}

		// 连接前后轨迹 //
		join();
	}
	OptimalTrajectory::~OptimalTrajectory() = default;
	OptimalTrajectory::OptimalTrajectory() = default;
	OptimalTrajectory::OptimalTrajectory(const OptimalTrajectory&) = default;
	OptimalTrajectory::OptimalTrajectory(OptimalTrajectory&&) = default;
	OptimalTrajectory& OptimalTrajectory::operator=(const OptimalTrajectory&) = default;
	OptimalTrajectory& OptimalTrajectory::operator=(OptimalTrajectory&&) = default;
}
