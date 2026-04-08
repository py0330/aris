#include <gtest/gtest.h>

#include <array>
#include <string>
#include <vector>

#include <aris/plan/plan.hpp>

namespace {

struct OneDCase {
	double v0;
	double v1;
	double vavg;
	double amax;
	double amin;
	double dt;
	bool expected;
};

struct VecCase {
	std::array<double, 3> v0;
	std::array<double, 3> v1;
	std::array<double, 3> vavg;
	double a;
	double dt;
	bool expected;
};

} // namespace

TEST(FunctionTest, IsInVavgBoundageOneDofCases) {
	const std::vector<OneDCase> cases{
		{0.0, 1.0, 0.124999999, 100.0, -100.0, 0.02, false},
		{0.0, 1.0, 0.125000001, 100.0, -100.0, 0.02, true},
		{0.0, 1.0, 0.874999999, 100.0, -100.0, 0.02, true},
		{0.0, 1.0, 0.875000001, 100.0, -100.0, 0.02, false},
		{0.0, 1.0, 0.9499999999, 100.0, -50.0, 0.025, true},
		{0.0, 1.0, 0.950000001, 100.0, -50.0, 0.025, false},
		{1.0, 0.0, 0.9499999999, 50.0, -100.0, 0.025, true},
		{1.0, 0.0, 0.950000001, 50.0, -100.0, 0.025, false},
	};

	for (std::size_t i = 0; i < cases.size(); ++i) {
		const auto &c = cases[i];
		auto got = aris::plan::s_is_in_vavg_boundage(c.v0, c.v1, c.vavg, c.amax, c.amin, c.dt);
		EXPECT_EQ(got, c.expected) << "Case index " << i << " failed";
	}
}

TEST(FunctionTest, IsInVavgBoundageVectorCases) {
	const std::vector<VecCase> cases{
		{{0.0, 0.1, 0.2}, {1.0, 0.1, 0.2}, {0.124999999, 0.1, 0.2}, 100.0, 0.02, false},
		{{0.0, 0.1, 0.2}, {1.0, 0.1, 0.2}, {0.1250000001, 0.1, 0.2}, 100.0, 0.02, true},
		{{0.0, 0.1, 0.2}, {1.0, 0.1, 0.2}, {0.87500001, 0.1, 0.2}, 100.0, 0.02, false},
		{{0.0, 0.1, 0.2}, {1.0, 0.1, 0.2}, {0.874999999, 0.1, 0.2}, 100.0, 0.02, true},
		{{0.0, 0.1, 0.2}, {1.0, 0.1, 0.2}, {0.5, 0.2250000001, 0.2}, 100.0, 0.02, false},
		{{0.0, 0.1, 0.2}, {1.0, 0.1, 0.2}, {0.5, 0.224999999, 0.2}, 100.0, 0.02, true},
	};

	for (std::size_t i = 0; i < cases.size(); ++i) {
		const auto &c = cases[i];
		auto got = aris::plan::s_is_in_vavg_boundage(
			const_cast<double *>(c.v0.data()),
			const_cast<double *>(c.v1.data()),
			const_cast<double *>(c.vavg.data()),
			c.a,
			c.dt);
		EXPECT_EQ(got, c.expected) << "Vector case index " << i << " failed";
	}
}
