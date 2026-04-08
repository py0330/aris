#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

#include <aris/plan/plan.hpp>

namespace {

auto pe321_to_pm(const double *pe, double *pm) -> void {
	aris::dynamic::s_pe2pm(pe, pm, "321");
}

auto expect_pose_near(const double *pe_actual, const double *pe_expected, double tol = 1e-5) -> void {
	double pm_actual[16]{};
	double pm_expected[16]{};
	pe321_to_pm(pe_actual, pm_actual);
	pe321_to_pm(pe_expected, pm_expected);
	for (int i = 0; i < 16; ++i) {
		EXPECT_NEAR(pm_actual[i], pm_expected[i], tol) << "Pose matrix mismatch at index " << i;
	}
}

} // namespace

TEST(TrajectoryTest, LineSequenceFinishesAndReachesLastTarget) {
	aris::plan::TrajectoryGenerator tg;
	tg.setPosTypes({aris::dynamic::PosType::PE321});
	tg.setDt(0.001);

	double p0[6]{0.45, 0.0, 0.75, aris::PI / 2.0, 0.0, aris::PI / 2.0};
	double p1[6]{0.46, 0.02, 0.74, aris::PI / 2.0, 0.0, aris::PI / 2.0};
	double p2[6]{0.43, -0.03, 0.76, aris::PI / 2.0, 0.0, aris::PI / 2.0};
	double vel[2]{0.2, 0.8};
	double acc[2]{1.0, 5.0};
	double jerk[2]{10.0, 20.0};
	double zone[2]{0.001, 0.001};

	tg.insertLinePos(1, p0, vel, acc, jerk, zone);
	double out[6]{};
	(void)tg.getEePosAndMoveDt(out);

	tg.insertLinePos(2, p1, vel, acc, jerk, zone);
	tg.insertLinePos(3, p2, vel, acc, jerk, zone);

	bool seen_2 = false;
	bool seen_3 = false;
	bool finished = false;
	std::int64_t ret = -1;
	for (int i = 0; i < 100000; ++i) {
		ret = tg.getEePosAndMoveDt(out);
		if (ret == 2) seen_2 = true;
		if (ret == 3) seen_3 = true;
		if (ret == 0) {
			finished = true;
			break;
		}
	}

	EXPECT_TRUE(seen_2);
	EXPECT_TRUE(seen_3);
	EXPECT_TRUE(finished);
	expect_pose_near(out, p2, 1e-4);
}

TEST(TrajectoryTest, CircleMotionAndNodeManagementWorks) {
	aris::plan::TrajectoryGenerator tg;
	tg.setPosTypes({aris::dynamic::PosType::PE321});
	tg.setDt(0.001);

	double p0[6]{0.40, 0.00, 0.70, aris::PI / 2.0, 0.0, aris::PI / 2.0};
	double mid[6]{0.45, 0.05, 0.70, aris::PI / 2.0, 0.0, aris::PI / 2.0};
	double p2[6]{0.50, 0.00, 0.70, aris::PI / 2.0, 0.0, aris::PI / 2.0};
	double vel[2]{0.2, 0.8};
	double acc[2]{1.0, 5.0};
	double jerk[2]{10.0, 20.0};
	double zone[2]{0.0, 0.0};

	tg.insertInitPos(10, p0);
	tg.insertCirclePos(11, p2, mid, vel, acc, jerk, zone);

	EXPECT_GE(tg.unusedPosNum(), 1);
	auto ids = tg.unusedNodeIds();
	EXPECT_FALSE(ids.empty());

	double out[6]{};
	bool seen_circle = false;
	bool finished = false;
	for (int i = 0; i < 100000; ++i) {
		auto ret = tg.getEePosAndMoveDt(out);
		if (ret == 11) seen_circle = true;
		if (ret == 0) {
			finished = true;
			break;
		}
	}

	EXPECT_TRUE(seen_circle);
	EXPECT_TRUE(finished);
	expect_pose_near(out, p2, 1e-4);

	tg.clearUsedPos();
	EXPECT_GE(tg.unusedPosNum(), 0);
	tg.clearAllPos();
	EXPECT_EQ(tg.unusedPosNum(), 0);
}
