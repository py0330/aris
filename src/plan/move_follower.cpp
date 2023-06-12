#include"aris/plan/move_follower.hpp"
#include"aris/plan/function.hpp"

//#define ARIS_DEBUG_SINGULAR_PROCESSOR

namespace aris::plan {

	using namespace aris::dynamic;

	auto s_is_in_vavg_boundage(const double* v0, const double* v1, const double *v_avg, double a, double dt, double *dis = nullptr, double *r = nullptr)->bool {
		// input check //
		double dis_, r_;
		dis = dis ? dis : &dis_;
		r = r ? r : &r_;

		// cpt r //
		double v_diff[3]{ v1[0] - v0[0],v1[1] - v0[1] ,v1[2] - v0[2] };
		double t1 = aris::dynamic::s_norm(3, v_diff) / a;
		*r = (dt - t1) / 2 * a;

		// too small margin //
		if (std::abs(dt - t1) < 1e-10) {
			double result[3]{
				v_avg[0] - (v0[0] + v1[0]) / 2,
				v_avg[1] - (v0[1] + v1[1]) / 2,
				v_avg[2] - (v0[2] + v1[2]) / 2,
			};
			*dis = aris::dynamic::s_norm(3, result);
			return *dis <= *r;
		}
			
		// normal check //
		// 
		// 若 va 是从 v0 加速到 v1 的平均速度，vb 是其他过程的平均速度
		// 于是针对平均速度 v_avg，有
		// v_avg * dt = va *t1 + vb*(dt - t1)
		//
		// 继而：
		// vb = （v_avg * dt - va *t1）/ (dt - t1)
		//
		double vavg_left[3]{
			(v_avg[0] * dt - (v0[0] + v1[0]) / 2 * t1) / (dt - t1),
			(v_avg[1] * dt - (v0[1] + v1[1]) / 2 * t1) / (dt - t1),
			(v_avg[2] * dt - (v0[2] + v1[2]) / 2 * t1) / (dt - t1),
		};

		// 正常情况需求 vb 偏离  v0 -> v1 这根轴的距离
		double a_dir[3]{
			v_avg[0] - v0[0],
			v_avg[1] - v0[1],
			v_avg[2] - v0[2],
		};
		double b_dir[3]{
			v_avg[0] - v1[0],
			v_avg[1] - v1[1],
			v_avg[2] - v1[2],
		};
		double c_dir[3]{
			v1[0] - v0[0],
			v1[1] - v0[1],
			v1[2] - v0[2],
		};

		if (s_norm(3, c_dir) < 1e-10 || s_vv(3, a_dir, c_dir) < 0.0) {
			*dis = s_norm(3, a_dir);
		}
		else if (s_vv(3, b_dir, c_dir) > 0.0) {
			*dis = s_norm(3, b_dir);
		}
		else {
			auto n = s_norm(3, c_dir);
			double result[3];
			s_c3(a_dir, c_dir, result);
			*dis = s_norm(3, result) / n;
		}

		return *dis < *r;
	}

	struct MoveFollower::Imp {
		double v_{ 1.0 }, w_{ 1.0 }, a_{ 10.0 }, x_{ 10.0 };
		double target_pm_[16]{0.0};
		double target_va_[6]{ 0.0 };
		double follow_pm_[16]{ 0.0 };
		double follow_va_[6]{ 0.0 };

		double estimate_t_{ 0.0 };
		double dt_{ 0.001 };
	};

	auto MoveFollower::setDt(double dt) -> void {
		imp_->dt_ = dt;
	}

	auto MoveFollower::setMaxV(double v)->void {
		imp_->v_ = v;
	}
	auto MoveFollower::setMaxW(double w)->void {
		imp_->w_ = w;
	}
	auto MoveFollower::setMaxA(double a)->void {
		imp_->a_ = a;
	}
	auto MoveFollower::setMaxX(double x)->void {
		imp_->x_ = x;
	}

	auto MoveFollower::setTargetPm(const double* pm)->void {
		aris::dynamic::s_vc(16, pm, imp_->target_pm_);
	}
	auto MoveFollower::setFollowPm(const double* pm)->void {
		aris::dynamic::s_vc(16, pm, imp_->follow_pm_);
	}
	auto MoveFollower::setTargetVa(const double* va)->void {
		aris::dynamic::s_vc(6, va, imp_->target_va_);
	}
	auto MoveFollower::setFollowVa(const double* va)->void {
		aris::dynamic::s_vc(6, va, imp_->follow_va_);
	}

	// 移动一个步长，并获取结果 //
	auto MoveFollower::moveDtAndGetResult(double* follow_pm, double* follow_va)->void {
		double dt = imp_->dt_;
		
		double target_xyz[3]{ imp_->target_pm_[3], imp_->target_pm_[7], imp_->target_pm_[11] };
		double follow_xyz[3]{ imp_->follow_pm_[3], imp_->follow_pm_[7], imp_->follow_pm_[11] };
		double target_v[3]{ imp_->target_va_[0], imp_->target_va_[1], imp_->target_va_[2] };
		double follow_v[3]{ imp_->follow_va_[0], imp_->follow_va_[1], imp_->follow_va_[2] };

		// 真正的期望速度 //
		// 包含两部分，一部分是目标当前的速度，另外一部分是补偿位置差的速度 //
		// 第一部分就是 target_v
		// 第二部分如下计算 : 
		//         此时应有 位置差（diff_p） 从 v 以 a 匀减速，正好走完
		//         因此应有 diff_p * 2 / v == v / a   左侧为匀减速完位置所需时间，右侧为减速完速度所需时间
		//              v = sqrt(2 * diff_p * a)
		//
		// target_v 是指，本周期执行完，所应该到达的速度，因此如果根据上一轮follow_v计算出来的值，还应该再加 dt
		
		
		// 真正期望速度的第一部分 //
		double real_target_v1[3];
		aris::dynamic::s_vc(3, target_v, real_target_v1);
		if (auto n = s_norm(3, real_target_v1); n > imp_->v_)
			s_nv(3, imp_->v_ / n, real_target_v1);

		// 叠加真正期望速度的第二部分 //
		double real_target_v2[3]{ 0,0,0 };
		double diff_xyz[3]{ 
			target_xyz[0] - follow_xyz[0] - dt * target_v[0],
			target_xyz[1] - follow_xyz[1] - dt * target_v[1],
			target_xyz[2] - follow_xyz[2] - dt * target_v[2] 
		};
		if (auto l = aris::dynamic::s_norm(3, diff_xyz); l > 1e-10) {
			// 归一化 //
			//double diff_xyz_dir[3];
			//aris::dynamic::s_vc(3, 1.0 / l, diff_xyz, diff_xyz_dir);

			// 计算目标速度的绝对值，用上述公式 //
			auto real_target_v2_norm = std::sqrt(2.0 * l * imp_->a_);
			imp_->estimate_t_ = real_target_v2_norm / imp_->a_;

			// 得到 real_target_v2 //
			aris::dynamic::s_vc(3, real_target_v2_norm / l, diff_xyz, real_target_v2);

			// 叠加 dt //
			double t_left = s_norm(3, real_target_v2) / imp_->a_;
			if (t_left > dt) {
				real_target_v2[0] = real_target_v2[0] - diff_xyz[0] / l * imp_->a_ * dt;
				real_target_v2[1] = real_target_v2[1] - diff_xyz[1] / l * imp_->a_ * dt;
				real_target_v2[2] = real_target_v2[2] - diff_xyz[2] / l * imp_->a_ * dt;
			}

			// 让它最大不超过 v //
			if (auto n = s_norm(3, real_target_v2); n > imp_->v_)
				s_nv(3, imp_->v_ / n, real_target_v2);
		}

		// 期望速度不应超过最大速度 //
		double real_target_v[3];
		s_vc(3, real_target_v1, real_target_v);
		s_va(3, real_target_v2, real_target_v);
		if (auto n = aris::dynamic::s_norm(3, real_target_v); n > imp_->v_) {
			aris::dynamic::s_nv(3, imp_->v_ / n, real_target_v);
		}

		// 计算加速度 //
		double diff_v[3]{ real_target_v[0] - follow_v[0], real_target_v[1] - follow_v[1], real_target_v[2] - follow_v[2], };
		auto n = aris::dynamic::s_norm(3, diff_v);
		double v_avg[3];
		s_vc(3, 1.0 / dt, diff_xyz, v_avg);
		s_va(3, real_target_v1, v_avg);

		// 此时速度基本已经跟上目标速度，首先check 位置是否也可以满足条件
		if (n < imp_->a_*dt && s_is_in_vavg_boundage(follow_v, real_target_v, v_avg, imp_->a_, dt)) {
			s_vc(3, real_target_v, follow_v);
			s_vc(3, target_xyz, follow_xyz);
			imp_->estimate_t_ = 0.0;
		}
		else {
			double follow_a[3];

			s_vc(3, n < imp_->a_ * dt ? 1.0 / dt : imp_->a_ / n, diff_v, follow_a);

			for (int i = 0; i < 3; ++i) {
				follow_xyz[i] = follow_xyz[i] + follow_v[i] * dt + follow_a[i] * dt * dt / 2;
				follow_v[i] = follow_v[i] + follow_a[i] * dt;
			}
		}

		// 更新数据 //
		s_pp2pm(follow_xyz, imp_->follow_pm_);
		s_vc(3, follow_v, imp_->follow_va_);



		// 返回 follow_xyz //
		s_vc(16, imp_->follow_pm_, follow_pm);
		s_vc(6, imp_->follow_va_, follow_va);

		
	}
	// 估算完全追踪上所需要的时间 //
	auto MoveFollower::estimateLeftT()->double {
		double dt = imp_->dt_;
		
		double target_xyz[3]{ imp_->target_pm_[3], imp_->target_pm_[7], imp_->target_pm_[11] };
		double follow_xyz[3]{ imp_->follow_pm_[3], imp_->follow_pm_[7], imp_->follow_pm_[11] };
		double target_v[3]{ imp_->target_va_[0], imp_->target_va_[1], imp_->target_va_[2] };
		double follow_v[3]{ imp_->follow_va_[0], imp_->follow_va_[1], imp_->follow_va_[2] };
		
		double diff_xyz[3]{
			target_xyz[0] - follow_xyz[0],
			target_xyz[1] - follow_xyz[1],
			target_xyz[2] - follow_xyz[2]
		};
		
		double diff_v[3]{ target_v[0] - follow_v[0], target_v[1] - follow_v[1], target_v[2] - follow_v[2], };


		auto t1 = s_norm(3, diff_v) / imp_->a_;

		auto l = s_norm(3, diff_xyz) + 0.5 * t1 * t1 * imp_->a_;
		auto t2 = std::sqrt(2.0 * l * imp_->a_)/ imp_->a_;

		return t1 + t2;

		return imp_->estimate_t_;
	}




	MoveFollower::~MoveFollower() = default;
	MoveFollower::MoveFollower() :imp_(new Imp) {}
}
