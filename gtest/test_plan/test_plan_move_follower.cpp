#include <gtest/gtest.h>

#include <cmath>

#include <aris/plan/plan.hpp>

namespace {

auto norm3(const double *v) -> double {
	return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

auto make_identity_pm(double x, double y, double z, double *pm) -> void {
	double pe[6]{ x, y, z, 0.0, 0.0, 0.0 };
	aris::dynamic::s_pe2pm(pe, pm, "321");
}

} // namespace

TEST(MoveFollowerTest, DynamicTargetTrackingRespectsLinearLimits) {
	aris::plan::MoveFollower follower;

	constexpr double dt = 0.002;
	constexpr double max_a = 12.0;
	constexpr double max_v = 2.5;
	follower.setMaxA(max_a);
	follower.setMaxV(max_v);
	follower.setDt(dt);

	double begin_follow_pm[16]{
		1, 0, 0, -0.15,
		0, 1, 0, 0.01,
		0, 0, 1, 0.05,
		0, 0, 0, 1
	};
	double begin_follow_v[6]{ 0, 0, 0, 0, 0, 0 };
	follower.setFollowPm(begin_follow_pm);
	follower.setFollowVa(begin_follow_v);

	double target_pm[16]{
		1, 0, 0, -0.2,
		0, 1, 0, 0.02,
		0, 0, 1, 0.3,
		0, 0, 0, 1
	};
	double target_v[6]{ 0.3, 0.01, -0.4, 0, 0, 0 };

	double prev_follow_v[6]{ 0, 0, 0, 0, 0, 0 };
	bool has_prev = false;

	for (int i = 0; i < 1200; ++i) {
		target_pm[3] += target_v[0] * dt;
		target_pm[7] += target_v[1] * dt;
		target_pm[11] += target_v[2] * dt;

		follower.setTargetPm(target_pm);
		follower.setTargetVa(target_v);

		double follow_pm[16]{};
		double follow_v[6]{};
		follower.moveDtAndGetResult(follow_pm, follow_v);

		EXPECT_LE(norm3(follow_v), max_v * 1.001) << "Linear speed exceeds limit";
		if (has_prev) {
			double acc[3]{
				(follow_v[0] - prev_follow_v[0]) / dt,
				(follow_v[1] - prev_follow_v[1]) / dt,
				(follow_v[2] - prev_follow_v[2]) / dt
			};
			EXPECT_LE(norm3(acc), max_a * 1.01) << "Linear acceleration exceeds limit";
		}

		for (int k = 0; k < 6; ++k) prev_follow_v[k] = follow_v[k];
		has_prev = true;
	}

	EXPECT_GE(follower.estimateLeftT(), 0.0);
}

TEST(MoveFollowerTest, StaticTargetConvergesToPoint) {
	aris::plan::MoveFollower follower;

	constexpr double dt = 0.002;
	follower.setMaxA(12.0);
	follower.setMaxV(2.5);
	follower.setDt(dt);

	double begin_follow_pm[16];
	make_identity_pm(-0.15, 0.01, 0.05, begin_follow_pm);
	double begin_follow_v[6]{ 0, 0, 0, 0, 0, 0 };
	follower.setFollowPm(begin_follow_pm);
	follower.setFollowVa(begin_follow_v);

	double target_pm[16];
	make_identity_pm(0.5, 0.6, 0.8, target_pm);
	double target_v[6]{ 0, 0, 0, 0, 0, 0 };

	double follow_pm[16]{};
	double follow_v[6]{};
	for (int i = 0; i < 3000; ++i) {
		follower.setTargetPm(target_pm);
		follower.setTargetVa(target_v);
		follower.moveDtAndGetResult(follow_pm, follow_v);
		if (follower.estimateLeftT() < 1e-8) break;
	}

	EXPECT_NEAR(follow_pm[3], target_pm[3], 1e-6);
	EXPECT_NEAR(follow_pm[7], target_pm[7], 1e-6);
	EXPECT_NEAR(follow_pm[11], target_pm[11], 1e-6);
	EXPECT_LE(norm3(follow_v), 1e-6);
	EXPECT_LE(follower.estimateLeftT(), 1e-8);
}
