#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <list>
#include <vector>

#include <aris/plan/plan.hpp>

// 前向声明 scurve.cpp 中未导出的内部函数
namespace aris::plan {
	auto s_scurve_smooth(SCurveParam& param) -> void;
}

namespace {

struct LimitTol {
	double v_tol{1e-8};
	double a_tol{1e-6};
	double j_tol{1e-3};
};

auto validate_scurve_profile(const std::list<aris::plan::SCurveNode> &scurve) -> void {
	ASSERT_FALSE(scurve.empty());
	ASSERT_FALSE(scurve.begin()->params_.empty());

	const LimitTol tol;
	const double dt = 0.01;
	const int dim = static_cast<int>(scurve.begin()->params_.size());

	std::vector<aris::plan::LargeNum> p(dim), p1(dim), p2(dim), p3(dim);
	std::vector<double> v(dim, 0.0), a(dim, 0.0), j(dim, 0.0);

	for (int i = 0; i < dim; ++i) {
		p[i] = scurve.begin()->params_[i].pa_;
		p1[i] = scurve.begin()->params_[i].pa_;
		p2[i] = scurve.begin()->params_[i].pa_;
		p3[i] = scurve.begin()->params_[i].pa_;
	}

	auto iter = scurve.begin();
	aris::plan::LargeNum t = iter->params_[0].t0_;

	while (iter != scurve.end()) {
		while (iter->params_[0].t0_ + iter->params_[0].T_ > t) {
			std::swap(p3, p2);
			std::swap(p2, p1);
			std::swap(p1, p);

			for (int i = 0; i < dim; ++i) {
				aris::plan::s_scurve_at(iter->params_[i], t, &p[i], &v[i], &a[i], &j[i]);

				double max_v = iter->params_[i].vc_max_;
				double max_a = iter->params_[i].a_;
				double max_j = iter->params_[i].j_;

				for (auto it = iter; it != scurve.begin(); it = std::prev(it)) {
					if (t - it->params_[0].t0_ < 4 * dt) {
						max_v = std::max(std::prev(it)->params_[i].vc_max_, max_v);
						max_a = std::max(std::prev(it)->params_[i].a_, max_a);
						max_j = std::max(std::prev(it)->params_[i].j_, max_j);
					} else {
						break;
					}
				}

				if (iter->params_[0].T_ - t < 4 * dt && std::next(iter) != scurve.end()) {
					max_v = std::max(std::next(iter)->params_[i].vc_max_, max_v);
					max_a = std::max(std::next(iter)->params_[i].a_, max_a);
					max_j = std::max(std::next(iter)->params_[i].j_, max_j);
				}

				double pc = static_cast<double>(p[i]);
				double pp = static_cast<double>(p1[i]);
				double ppp = static_cast<double>(p2[i]);
				double pppp = static_cast<double>(p3[i]);

				double v_cur = (pc - pp) / dt;
				double v_pre = (pp - ppp) / dt;
				double v_pre2 = (ppp - pppp) / dt;
				double a_cur = (v_cur - v_pre) / dt;
				double a_pre = (v_pre - v_pre2) / dt;
				double j_cur = (a_cur - a_pre) / dt;

				EXPECT_LE(std::abs(v[i]), max_v + tol.v_tol * std::max(1.0, max_v));
				EXPECT_LE(std::abs(v_cur), max_v + tol.v_tol * std::max(1.0, max_v));
				EXPECT_LE(std::abs(v[i] - v_cur), max_a * dt + tol.v_tol);

				EXPECT_LE(std::abs(a[i]), max_a + tol.a_tol * std::max(1.0, max_a));
				EXPECT_LE(std::abs(a_cur), max_a + tol.a_tol * std::max(1.0, max_a));
				EXPECT_LE(std::abs(a[i] - a_cur), max_j * dt + tol.a_tol);

				EXPECT_LE(std::abs(j[i]), max_j + tol.j_tol * std::max(1.0, max_j));
				EXPECT_LE(std::abs(j_cur), max_j + tol.j_tol * std::max(1.0, max_j));
			}

			t = t + dt;
		}

		for (int i = 0; i < dim; ++i) {
			aris::plan::LargeNum p_end, p_last;
			aris::plan::s_scurve_at(iter->params_[i], iter->params_[i].t0_ + iter->params_[i].T_, &p_end, &v[i], &a[i], &j[i]);
			aris::plan::s_scurve_at(iter->params_[i], std::max(iter->params_[i].t0_, iter->params_[i].t0_ + iter->params_[i].T_ - dt), &p_last, &v[i], &a[i], &j[i]);

			double end_vel_est = std::abs(static_cast<double>(p_end - p_last)) / dt;
			EXPECT_LE(end_vel_est, iter->params_[i].vb_ + iter->params_[i].a_ * dt + tol.v_tol);
		}

		++iter;
	}
}

} // namespace

TEST(SCurveTest, MakeNodesAndCheckDynamicsSmallDeterministicCase) {
	constexpr int m = 8;
	constexpr int n = 2;

	double pb[m][n]{
		{0.3, 0.2},
		{0.7, 0.5},
		{1.2, 0.9},
		{1.8, 1.3},
		{2.5, 1.8},
		{3.0, 2.1},
		{3.6, 2.5},
		{4.0, 2.9}
	};

	double vb_max[m][n]{
		{0.0, 0.0},
		{0.3, 0.2},
		{0.2, 0.3},
		{0.4, 0.2},
		{0.3, 0.4},
		{0.2, 0.2},
		{0.1, 0.3},
		{0.0, 0.0}
	};

	double vc_max[m][n]{
		{1.6, 1.2},
		{1.7, 1.3},
		{1.8, 1.4},
		{1.9, 1.5},
		{2.0, 1.6},
		{1.9, 1.5},
		{1.8, 1.4},
		{1.7, 1.3}
	};

	double a[m][n]{
		{4.0, 3.0},
		{4.5, 3.5},
		{5.0, 4.0},
		{4.8, 3.6},
		{4.2, 3.2},
		{4.6, 3.8},
		{4.4, 3.4},
		{4.0, 3.0}
	};

	double j[m][n]{
		{20.0, 15.0},
		{22.0, 16.0},
		{24.0, 18.0},
		{23.0, 17.0},
		{21.0, 16.0},
		{22.0, 17.0},
		{21.0, 16.0},
		{20.0, 15.0}
	};

	std::list<aris::plan::SCurveNode> scurve;
	for (int i = 0; i < m; ++i) {
		scurve.push_back(aris::plan::SCurveNode{});
		for (int k = 0; k < n; ++k) {
			auto &param = scurve.back().params_.emplace_back();
			param.pb_ = pb[i][k];
			param.vb_max_ = vb_max[i][k];
			param.vc_max_ = vc_max[i][k];
			param.a_ = a[i][k];
			param.j_ = j[i][k];
			if (i == 0) param.pa_ = 0.0;
		}
	}

	auto ret = aris::plan::s_scurve_make_nodes(scurve.begin(), scurve.end());
	EXPECT_GE(ret, 0);

	validate_scurve_profile(scurve);
}

// 辅助：构造 SCurveParam 并调用 s_scurve_smooth
// 返回 smooth_Ta_, smooth_Tb_, smooth_vc_
auto make_param_and_smooth(double T, double Ta, double Tb,
                             double va, double vb, double vc,
                             double vc_max, int mode,
                             double& out_Ta, double& out_Tb, double& out_vc) -> void {
	aris::plan::SCurveParam p;
	p.T_ = T; p.Ta_ = Ta; p.Tb_ = Tb;
	p.va_ = va; p.vb_ = vb; p.vc_ = vc;
	p.vc_max_ = vc_max; p.mode_ = mode;
	aris::plan::s_scurve_smooth(p);
	out_Ta = p.smooth_Ta_;
	out_Tb = p.smooth_Tb_;
	out_vc = p.smooth_vc_;
}

// 路径1: mode=1 → smooth_Ta = Ta-min(Ta,Tb), smooth_Tb = Tb-min(Ta,Tb)
TEST(SCurveTest, Smooth_Mode1) {
	double Ta, Tb, vc;
	make_param_and_smooth(2.0, 0.5, 0.3, 0, 0, 1.0, 2.0, 1, Ta, Tb, vc);
	// Ta_smooth = 0.5 - min(0.5,0.3) = 0.2, Tb_smooth = 0.3 - min(0.5,0.3) = 0.0
	EXPECT_NEAR(Ta, 0.2, 1e-10);
	EXPECT_NEAR(Tb, 0.0, 1e-10);
}

// 路径2: vc <= va → 无优化，返回原值
TEST(SCurveTest, Smooth_VcNotAboveVa) {
	double Ta, Tb, vc;
	make_param_and_smooth(2.0, 0.4, 0.3, 0.5, 0, 0.5, 2.0, 0, Ta, Tb, vc);
	EXPECT_NEAR(Ta, 0.4, 1e-10);
	EXPECT_NEAR(Tb, 0.3, 1e-10);
}

// 路径3: vc <= vb → 无优化，返回原值
TEST(SCurveTest, Smooth_VcNotAboveVb) {
	double Ta, Tb, vc;
	make_param_and_smooth(2.0, 0.4, 0.3, 0, 0.6, 0.6, 2.0, 0, Ta, Tb, vc);
	EXPECT_NEAR(Ta, 0.4, 1e-10);
	EXPECT_NEAR(Tb, 0.3, 1e-10);
}

// 路径4: Ta+Tb > f*T → solve_velocity_scale 提前返回（原值）
TEST(SCurveTest, Smooth_NoConstTime) {
	// T=1.0, f=0.8, 需要 Ta+Tb <= 0.8, 但 Ta+Tb=0.9 > 0.8
	double Ta, Tb, vc;
	make_param_and_smooth(1.0, 0.5, 0.4, 0, 0, 0.8, 2.0, 0, Ta, Tb, vc);
	EXPECT_NEAR(Ta, 0.5, 1e-10);
	EXPECT_NEAR(Tb, 0.4, 1e-10);
}

// 路径5: 正常优化 — 有匀速段，D1>0, D>=0, step1 限制 vc_new
TEST(SCurveTest, Smooth_NormalOptimization) {
	// T=2.0, Ta=0.2, Tb=0.2, vc=1.0, vc_max=5.0
	// Ta+Tb=0.4 < 0.8*T=1.6, 满足优化条件
	double Ta, Tb, vc;
	make_param_and_smooth(2.0, 0.2, 0.2, 0, 0, 1.0, 5.0, 0, Ta, Tb, vc);
	// 优化后 Ta,Tb 应增大（加速更平缓）
	EXPECT_GE(Ta, 0.2);
	EXPECT_GE(Tb, 0.2);
	EXPECT_LE(Ta + Tb, 2.0);
}

// 路径5b: step2 限制 vc_new（非 step1）→ vc_new/vc_max == a1_new/a1_old == a2_new/a2_old
TEST(SCurveTest, Smooth_Step2LimitsVc) {
	// vc_max=2.0 较小，step2 解 vc=1.125 < step1 的 vc_limit_1=1.5
	// 三个缩放比例应该严格相等
	double T=2.0, Ta_old=0.2, Tb_old=0.2, va=0, vb=0, vc_old=1.0, vc_max=2.0;
	double Ta, Tb, vc_new;
	make_param_and_smooth(T, Ta_old, Tb_old, va, vb, vc_old, vc_max, 0, Ta, Tb, vc_new);

	double a1_old = (vc_old - va) / Ta_old;
	double a2_old = (vc_old - vb) / Tb_old;

	double k = (vc_new - va) / (a1_old * Ta);           // a1_new/a1_old
	double ratio_v = vc_new / vc_max;                    // vc_new/vc_max
	double a1_new = (vc_new - va) / Ta;
	double a2_new = (vc_new - vb) / Tb;

	EXPECT_NEAR(ratio_v, k, 1e-10);
	EXPECT_NEAR(a1_new / a1_old, k, 1e-10);
	EXPECT_NEAR(a2_new / a2_old, k, 1e-10);

	EXPECT_GE(Ta, Ta_old);
	EXPECT_GE(Tb, Tb_old);
	EXPECT_LE(Ta + Tb, T);
}

// 路径6: 正常优化 — va > vb 情况（测试 k 的分支选择）
TEST(SCurveTest, Smooth_VaGreaterThanVb) {
	double Ta, Tb, vc;
	make_param_and_smooth(2.0, 0.15, 0.25, 0.2, 0, 1.0, 5.0, 0, Ta, Tb, vc);
	EXPECT_GE(Ta, 0.15);
	EXPECT_GE(Tb, 0.25);
	EXPECT_LE(Ta + Tb, 2.0);
}

// 路径7: D<0（step2 跳过），仅用 step1 的 vc_limit_1
// vc_max 很小导致二次方程无实根
TEST(SCurveTest, Smooth_Step2NoSolution) {
	// vc_max 接近 vc，D 容易 < 0
	double Ta, Tb, vc;
	make_param_and_smooth(0.8, 0.4, 0.3, 0, 0.3, 0.86, 1.7, 0, Ta, Tb, vc);
	// 即使 D<0，也不应崩溃，返回合理值
	EXPECT_GE(Ta, 0);
	EXPECT_GE(Tb, 0);
	EXPECT_LE(Ta + Tb, 0.8);
}

// 路径8: D1<=0 → vc_limit_1 用 vc_max
// 极小 a1_avg/a2_avg 使判别式负数
TEST(SCurveTest, Smooth_Step1DiscNegative) {
	// 极小 Ta 导致 a1_avg 极大，D1 可能 <= 0
	double Ta, Tb, vc;
	make_param_and_smooth(2.0, 0.01, 0.01, 0, 0, 1.0, 5.0, 0, Ta, Tb, vc);
	EXPECT_GE(Ta, 0.01);
	EXPECT_GE(Tb, 0.01);
}
