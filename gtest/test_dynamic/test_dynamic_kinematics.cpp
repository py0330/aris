#include <gtest/gtest.h>

#include <aris/dynamic/dynamic.hpp>
#include <aris/dynamic/kinematics.hpp>

namespace {

TEST(Kinematics, EyeInHandCalibration) {
	using namespace aris::dynamic;

	const double pe_eye_in_tool[6]{ 0.1, 0.2, 0.3, 0.111, 0.221, 0.832 };
	double pq_eye_in_tool[7];
	s_pe2pq(pe_eye_in_tool, pq_eye_in_tool, "321");

	const double pe_obj_in_base[6]{ 1.23, 2.2, 4.3, 0.511, 0.321, 0.932 };
	double pq_obj_in_base[7];
	s_pe2pq(pe_obj_in_base, pq_obj_in_base, "321");

	const int n = 10;
	double pe_tool_in_base[n][6]{
		{0.1, 0.2, 0.1, 0.1, 0.2, 0.3},
		{0.2, 0.4, -0.3, 0.5, 0.2, 0.3},
		{0.3, 0.6, 0.3, 0.8, 0.2, 0.3},
		{0.3, 0.8, -0.3, 0.1, 0.2, 0.3},
		{0.2, 0.9, 0.3, 0.1, 0.5, 0.3},
		{0.1, 0.9, -0.3, 0.1, 0.8, 0.3},
		{0.1, 0.8, 0.3, 0.1, 0.2, 0.3},
		{0.2, 0.7, -0.3, 0.1, 0.2, 0.6},
		{0.3, 0.6, 0.3, 0.1, 0.2, 0.9},
	};

	double pq_tool_in_base[n][7];
	double pq_obj_in_eye[n][7];

	for (int i = 0; i < n; ++i) {
		s_pe2pq(pe_tool_in_base[i], pq_tool_in_base[i], "123");

		double pq_eye_in_base[7];
		s_pq_dot_pq(pq_tool_in_base[i], pq_eye_in_tool, pq_eye_in_base);

		double pm_eye_in_base[16];
		s_pq2pm(pq_eye_in_base, pm_eye_in_base);
		s_inv_pq2pq(pm_eye_in_base, pq_obj_in_base, pq_obj_in_eye[i]);
	}

	double pq_eye_in_tool_result[7];
	double mem[16 * n * n];
	s_eye_in_hand_calib(n, *pq_obj_in_eye, *pq_tool_in_base, pq_eye_in_tool_result, mem);

	EXPECT_TRUE(s_is_equal(7, 1, pq_eye_in_tool_result, pq_eye_in_tool, 1e-10));
}

TEST(Kinematics, EyeToHandCalibration) {
	using namespace aris::dynamic;

	const double pe_eye_in_tool[6]{ 0.1, 0.2, 0.3, 0.111, 0.221, 0.832 };
	double pq_eye_in_base[7];
	s_pe2pq(pe_eye_in_tool, pq_eye_in_base, "321");

	const double pe_obj_in_tool[6]{ 1.23, 2.2, 4.3, 0.511, 0.321, 0.932 };
	double pq_obj_in_tool[7];
	s_pe2pq(pe_obj_in_tool, pq_obj_in_tool, "321");

	const int n = 10;
	double pe_tool_in_base[n][6]{
		{0.1, 0.2, 0.1, 0.1, 0.2, 0.3},
		{0.2, 0.4, -0.3, 0.5, 0.2, 0.3},
		{0.3, 0.6, 0.3, 0.8, 0.2, 0.3},
		{0.3, 0.8, -0.3, 0.1, 0.2, 0.3},
		{0.2, 0.9, 0.3, 0.1, 0.5, 0.3},
		{0.1, 0.9, -0.3, 0.1, 0.8, 0.3},
		{0.1, 0.8, 0.3, 0.1, 0.2, 0.3},
		{0.2, 0.7, -0.3, 0.1, 0.2, 0.6},
		{0.3, 0.6, 0.3, 0.1, 0.2, 0.9},
	};

	double pq_tool_in_base[n][7];
	double pq_obj_in_eye[n][7];

	for (int i = 0; i < n; ++i) {
		s_pe2pq(pe_tool_in_base[i], pq_tool_in_base[i], "123");

		double pq_obj_in_base[7];
		s_pq_dot_pq(pq_tool_in_base[i], pq_obj_in_tool, pq_obj_in_base);
		s_inv_pq_dot_pq(pq_eye_in_base, pq_obj_in_base, pq_obj_in_eye[i]);
	}

	double pq_eye_in_base_result[7];
	double mem[16 * n * n];
	s_eye_to_hand_calib(n, *pq_obj_in_eye, *pq_tool_in_base, pq_eye_in_base_result, mem);

	EXPECT_TRUE(s_is_equal(7, 1, pq_eye_in_base_result, pq_eye_in_base, 1e-10));
}

} // namespace
