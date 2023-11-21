#include <algorithm>

#include"aris/plan/algorithm.hpp"
#include"aris/dynamic/math_matrix.hpp"
#include"aris/plan/function.hpp"

namespace aris::plan{

	struct WalkParam {
		int count;
		std::int32_t totalCount{ 3000 };
		std::int32_t n{ 2 };
		double d{ 0.5 };
		double h{ 0.05 };
		double alpha{ 0.3 };
		double beta{ 0.3 };
	};

	auto walkGait(int leg_num, const double *init_feet_pos,
		int total_count, int step_num, double step_length, double step_height, double dir_alpha, double dir_beta,
		int count, double *body_pe, double* feet_pos) -> int
	{
		// 起始位置
		const double *beginPee = init_feet_pos;
		//if (param.count % total_count == 0)
		//{
		//	beginMak.setPrtPm(*robot.body().pm());
		//	beginMak.update();
		//	robot.GetPee(beginPee, beginMak);
		//}

		//以下设置各个阶段的身体的真实初始位置
		const double a = dir_alpha;
		const double b = dir_beta;
		const double d = step_length;
		const double h = step_height;

		const double front[3]{ -std::sin(a),0,-std::cos(a) };
		const double left[3]{ -std::cos(a),0,std::sin(a) };
		const double up[3]{ 0,1,0 };

		int period_count = count % total_count;
		const double s = -(PI / 2) * cos(PI * (period_count + 1) / total_count) + PI / 2;//s 从0到PI. 

		double Peb[6], Pee[18];
		std::fill(Peb, Peb + 6, 0);
		std::copy(beginPee, beginPee + 18, Pee);


		double pq_b[7]{ 0,0,0,std::sin(b / 2) * up[0],std::sin(b / 2) * up[1],std::sin(b / 2) * up[2],std::cos(b / 2) };
		double pq_b_half[7]{ 0,0,0,std::sin(b / 4) * up[0],std::sin(b / 4) * up[1],std::sin(b / 4) * up[2],std::cos(b / 4) };
		double pq_b_quad[7]{ 0,0,0,std::sin(b / 8) * up[0],std::sin(b / 8) * up[1],std::sin(b / 8) * up[2],std::cos(b / 8) };
		double pq_b_eighth[7]{ 0,0,0,std::sin(b / 16) * up[0],std::sin(b / 16) * up[1],std::sin(b / 16) * up[2],std::cos(b / 16) };
		double pm_b[16], pm_b_half[16], pm_b_quad[16], pm_b_eighth[16];

		aris::dynamic::s_pq2pm(pq_b, pm_b);
		aris::dynamic::s_pq2pm(pq_b_half, pm_b_half);
		aris::dynamic::s_pq2pm(pq_b_quad, pm_b_quad);
		aris::dynamic::s_pq2pm(pq_b_eighth, pm_b_eighth);

		const int leg_begin_id = (count / total_count) % 2 == 1 ? 3 : 0;

		if ((count / total_count) == 0)//加速段
		{
			//规划腿
			for (int i = leg_begin_id; i < 18; i += 6)
			{
				//单腿运动需要分解成延圆周的直线运动，还有延自身的转动
				double leg_forward_dir[3], forward_d[3];
				aris::dynamic::s_pm_dot_v3(pm_b_quad, front, leg_forward_dir);

				aris::dynamic::s_pm_dot_v3(pm_b_half, beginPee + i, forward_d);
				aris::dynamic::s_va(3, -1, beginPee + i, 1, forward_d, 1);
				aris::dynamic::s_va(3, d / 2, leg_forward_dir, 1, forward_d, 1);

				for (int j = 0; j < 3; ++j)
				{
					Pee[i + j] = beginPee[i + j] + (1 - std::cos(s)) / 2 * forward_d[j] + h * up[j] * std::sin(s);
				}
			}

			//规划身体位置
			double body_forward_dir[3], body_left_dir[3];
			aris::dynamic::s_pm_dot_v3(pm_b_eighth, front, body_forward_dir);
			aris::dynamic::s_pm_dot_v3(pm_b_eighth, left, body_left_dir);

			for (int i = 0; i < 3; ++i)
			{
				Peb[i] = left[i] * aris::plan::s_interp(total_count, period_count + 1, 0, d * std::tan(b / 8) / 4 / std::cos(b / 8), 0, d / 2 / total_count / std::cos(b / 2) * std::sin(b / 4))
					+ front[i] * aris::plan::s_interp(total_count, period_count + 1, 0, d / 4 / std::cos(b / 4), 0, d / 2 / total_count / std::cos(b / 2) * std::cos(b / 4));
			}

			//规划身体姿态
			double s_acc = aris::plan::acc_even(total_count, period_count + 1);
			double pq[7] = { 0,0,0,std::sin(s_acc * b / 8) * up[0],std::sin(s_acc * b / 8) * up[1] ,std::sin(s_acc * b / 8) * up[2],std::cos(s_acc * b / 8) };
			double pe[6];
			aris::dynamic::s_pq2pe(pq, pe);
			std::copy(pe + 3, pe + 6, Peb + 3);
		}
		else if ((count / total_count) == (step_num * 2 - 1))//减速段
		{
			//规划腿
			for (int i = leg_begin_id; i < 18; i += 6)
			{
				//单腿运动需要分解成延圆周的直线运动，还有延自身的转动
				double leg_forward_dir[3], forward_d[3];
				aris::dynamic::s_pm_dot_v3(pm_b_quad, front, leg_forward_dir);

				aris::dynamic::s_pm_dot_v3(pm_b_half, beginPee + i, forward_d);
				aris::dynamic::s_va(3, -1, beginPee + i, 1, forward_d, 1);
				aris::dynamic::s_va(3, d / 2, leg_forward_dir, 1, forward_d, 1);

				for (int j = 0; j < 3; ++j)
				{
					Pee[i + j] = beginPee[i + j] + (1 - std::cos(s)) / 2 * forward_d[j] + h * up[j] * std::sin(s);
				}
			}

			//规划身体位置
			double body_forward_dir[3], body_left_dir[3];
			aris::dynamic::s_pm_dot_v3(pm_b_eighth, front, body_forward_dir);
			aris::dynamic::s_pm_dot_v3(pm_b_eighth, left, body_left_dir);

			for (int i = 0; i < 3; ++i)
			{
				Peb[i] = left[i] * aris::plan::s_interp(total_count, period_count + 1, 0, d * std::tan(b / 8) / 4 / std::cos(b / 8), 0, 0)
					+ front[i] * aris::plan::s_interp(total_count, period_count + 1, 0, d / 4 / std::cos(b / 4), d / 2 / total_count / std::cos(b / 2), 0);
			}

			//规划身体姿态
			double s_dec = aris::plan::dec_even(total_count, period_count + 1);
			double pq[7] = { 0,0,0,std::sin(s_dec * b / 8) * up[0],std::sin(s_dec * b / 8) * up[1] ,std::sin(s_dec * b / 8) * up[2],std::cos(s_dec * b / 8) };
			double pe[6];
			aris::dynamic::s_pq2pe(pq, pe);
			std::copy(pe + 3, pe + 6, Peb + 3);
		}
		else//匀速段
		{
			//规划腿
			for (int i = leg_begin_id; i < 18; i += 6)
			{
				//单腿运动需要分解成延圆周的直线运动，还有延自身的转动
				double leg_forward_dir[3], forward_d[3];
				aris::dynamic::s_pm_dot_v3(pm_b_half, front, leg_forward_dir);

				aris::dynamic::s_pm_dot_v3(pm_b, beginPee + i, forward_d);
				aris::dynamic::s_va(3, -1.0, beginPee + i, 1, forward_d, 1);
				aris::dynamic::s_va(3, d, leg_forward_dir, 1, forward_d, 1);

				for (int j = 0; j < 3; ++j)
				{
					Pee[i + j] = beginPee[i + j] + (1 - std::cos(s)) / 2 * forward_d[j] + h * up[j] * std::sin(s);
				}
			}

			//规划身体位置
			double d2 = d / 2 / std::cos(b / 4);
			for (int i = 0; i < 3; ++i)
			{
				Peb[i] = left[i] * aris::plan::s_interp(total_count, period_count + 1, 0, d2 * std::sin(b / 4), 0, d / 2 / total_count / std::cos(b / 2) * std::sin(b / 2))
					+ front[i] * aris::plan::s_interp(total_count, period_count + 1, 0, d / 2, d / 2 / total_count / std::cos(b / 2), d / 2 / total_count / std::cos(b / 2) * std::cos(b / 2));
			}

			//规划身体姿态
			double s_even = aris::plan::even(total_count, period_count + 1);
			double pq[7] = { 0,0,0,std::sin(s_even * b / 4) * up[0],std::sin(s_even * b / 4) * up[1] ,std::sin(s_even * b / 4) * up[2],std::cos(s_even * b / 4) };
			double pe[6];
			aris::dynamic::s_pq2pe(pq, pe);
			std::copy(pe + 3, pe + 6, Peb + 3);
		}

		//robot.SetPeb(Peb, beginMak);
		//robot.SetPee(Pee, beginMak);

		return 2 * step_num * total_count - count - 1;
	}



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
