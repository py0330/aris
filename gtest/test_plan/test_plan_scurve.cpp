#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <list>
#include <vector>

#include <aris/plan/plan.hpp>

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
