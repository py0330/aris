#include <gtest/gtest.h>

#include <aris/dynamic/dynamic.hpp>
#include <aris/core/core.hpp>

using namespace aris::dynamic;

namespace {

constexpr double error = 1e-10;

} // namespace




TEST(DynamicPoseTest, PoseOperation) {
	const double pe313[6] = { 0.1, 0.2, 0.3,0.000423769269879415,   1.38980987554835,   1.79253453841257 };
	const double pe321[6] = { 0.1, 0.2, 0.3,2.46823966120654, -1.28551725555848,  5.40636866254317 };
	const double pq[7] = { 0.1, 0.2, 0.3,0.4,-0.5, 0.6, std::sqrt(1 - 0.4 * 0.4 - 0.5 * 0.5 - 0.6 * 0.6) };
	const double pm[16] = { -0.22, -0.975499782797526,   0.000416847668728071, 0.1,
		0.175499782797526, -0.04, -0.983666521865018, 0.2,
		0.959583152331272, -0.216333478134982,   0.18, 0.3,
		0,0,0,1 };

	const double inv_pm[16]{ -0.22,0.175499782797526,0.959583152331272, -0.300974902258887,
		-0.975499782797526, -0.04, -0.216333478134982,0.170450021720247,
		0.000416847668728071, -0.983666521865018,0.18,0.142691619606131,
		0,0,0,1 };

	const double pe313_2[6] = { 0.1,   0.2,   0.3,   0.4,   0.5,   0.6 };
	const double pe321_2[6] = { 0.1,   0.2,   0.3,   0.9407036672071061, - 0.2741242841334880,   0.4235879050181210 };
	const double pq2[7]{ 0.1000000000000000,   0.2000000000000000,   0.3000000000000000,   0.2461679699645254, - 0.0246991825443315,   0.4645213596389284,   0.8503006452922329 };
	const double pm2[16]{ 0.567219713641686,-0.802125918959455,0.186697098503681,0.1,
		0.77780532845257,0.447242474005492,-0.441580163137156,0.2,
		0.270704021926224,0.395686971707304,0.877582561890373,0.3,
		0,0,0,1 };

	const double pe313_3[6] = { 0.11,0.22, - 0.33,   0.3429889306768038,   0.9896374366179882,   6.0793638836224995 };
	const double pe321_3[6] = { 0.11,0.22, -0.33,   0.2300000000000000,   0.1699999999999999,   0.9799999999999995 };
	const double pq3[7]{ 0.11,0.22, -0.33,  0.4572339663606721 ,0.1282210261505954,0.0611881113319086,0.8779251012988581 };
	const double pm3[16]{ 0.959630766969045,0.00981685903664117,0.281091480420624,0.11,
		0.224691174394048,0.574386230075444,-0.787143147622555,0.22,
		-0.169182349066996,0.818525557315332,0.548992936917952,-0.33,
		0,0,0,1 };

	const double pm_dot_pm2[16]{ -0.883424423624956,-0.259652292887403,0.390053809802222,-0.116974902258887,
		-0.197847760298552,-0.547886650720495,-0.812820079542285,-0.0855499782797527,
		0.424755872811384,-0.79523638294572,0.432644824020405,0.406691619606131,
		0,0,0,1, };
	const double pm_dot_pm2_dot_pm3[16]{ -0.972093055630252,0.16145585737304,0.170197230633687, -0.399992850527594,
		-0.175451282517831, -0.981954799994014, -0.0705805797055172,0.0403823311778517,
		0.155730339556197, -0.0984722137897536,0.982879079263088,0.135689969440591,
		0,0,0,1 };

	const double from_v1[]{ 0.45,0.65,0.13 };
	const double to_v1[]{ -0.733020668621457,-0.0749017455835656,0.314595657761334 };

	double result[36];
	// test s_pose2pose
	{
		s_pose2pose(pq, "pq", result, "pm");
		EXPECT_TRUE(s_is_equal(16, result, pm, error)) << "\"s_pose2pose pq pm\" failed";

		s_pose2pose(pm, "pm", result, "pq");
		EXPECT_TRUE(s_is_equal(7, result, pq, error)) << "\"s_pose2pose pm pq\" failed";

		s_pose2pose(pq, "ipq", result, "pm");
		EXPECT_TRUE(s_is_equal(16, result, inv_pm, error)) << "\"s_pose2pose ipq pm\" failed";

		s_pose2pose(inv_pm, "pm", result, "ipq");
		EXPECT_TRUE(s_is_equal(7, result, pq, error)) << "\"s_pose2pose pm ipq\" failed";

		s_pose2pose(pq, "pq", result, "ipm");
		EXPECT_TRUE(s_is_equal(16, result, inv_pm, error)) << "\"s_pose2pose pq ipm\" failed";

		s_pose2pose(inv_pm, "ipm", result, "pq");
		EXPECT_TRUE(s_is_equal(7, result, pq, error)) << "\"s_pose2pose ipm pq\" failed";

		s_pose2pose(pq, "ipq", result, "ipm");
		EXPECT_TRUE(s_is_equal(16, result, pm, error)) << "\"s_pose2pose ipq ipm\" failed";

		s_pose2pose(pm, "ipm", result, "ipq");
		EXPECT_TRUE(s_is_equal(7, result, pq, error)) << "\"s_pose2pose ipm ipq\" failed";

		// 

		s_pose2pose(pm, "pm", result, "pe313");
		EXPECT_TRUE(s_is_equal(6, result, pe313, error)) << "\"s_pose2pose ipm pe313\" failed";

		s_pose2pose(pe313, "pe313", result, "pm");
		EXPECT_TRUE(s_is_equal(16, result, pm, error)) << "\"s_pose2pose pe313 pm\" failed";

		s_pose2pose(inv_pm, "ipm", result, "pe313");
		EXPECT_TRUE(s_is_equal(6, result, pe313, error)) << "\"s_pose2pose ipm pe313\" failed";

		s_pose2pose(pe313, "pe313", result, "ipm");
		EXPECT_TRUE(s_is_equal(16, result, inv_pm, error)) << "\"s_pose2pose pe313 ipm\" failed";

		s_pose2pose(inv_pm, "pm", result, "ipe313");
		EXPECT_TRUE(s_is_equal(6, result, pe313, error)) << "\"s_pose2pose pm ipe313\" failed";

		s_pose2pose(pe313, "ipe313", result, "pm");
		EXPECT_TRUE(s_is_equal(16, result, inv_pm, error)) << "\"s_pose2pose ipe313 pm\" failed";

		s_pose2pose(pm, "ipm", result, "ipe313");
		EXPECT_TRUE(s_is_equal(6, result, pe313, error)) << "\"s_pose2pose ipm pe313\" failed";

		s_pose2pose(pe313, "ipe313", result, "ipm");
		EXPECT_TRUE(s_is_equal(16, result, pm, error)) << "\"s_pose2pose ipe313 ipm\" failed";

		// 

		s_pose2pose(pm, "pm", result, "pe321");
		EXPECT_TRUE(s_is_equal(6, result, pe321, error)) << "\"s_pose2pose ipm pe321\" failed";

		s_pose2pose(pe321, "pe321", result, "pm");
		EXPECT_TRUE(s_is_equal(16, result, pm, error)) << "\"s_pose2pose pe321 pm\" failed";

		s_pose2pose(inv_pm, "ipm", result, "pe321");
		EXPECT_TRUE(s_is_equal(6, result, pe321, error)) << "\"s_pose2pose ipm pe321\" failed";

		s_pose2pose(pe321, "pe321", result, "ipm");
		EXPECT_TRUE(s_is_equal(16, result, inv_pm, error)) << "\"s_pose2pose pe321 ipm\" failed";

		s_pose2pose(inv_pm, "pm", result, "ipe321");
		EXPECT_TRUE(s_is_equal(6, result, pe321, error)) << "\"s_pose2pose pm ipe321\" failed";

		s_pose2pose(pe321, "ipe321", result, "pm");
		EXPECT_TRUE(s_is_equal(16, result, inv_pm, error)) << "\"s_pose2pose ipe321 pm\" failed";

		s_pose2pose(pm, "ipm", result, "ipe321");
		EXPECT_TRUE(s_is_equal(6, result, pe321, error)) << "\"s_pose2pose ipm pe321\" failed";

		s_pose2pose(pe321, "ipe321", result, "ipm");
		EXPECT_TRUE(s_is_equal(16, result, pm, error)) << "\"s_pose2pose ipe321 ipm\" failed";
	}

	// test s_pose_dot_pose
	{
		s_pose_dot_pose(pm, "pm", pm2, "pm", result, "pm");
		EXPECT_TRUE(s_is_equal(16, result, pm_dot_pm2, error)) << "\"s_pose_dot_pose pm pm pm\" failed";

		s_pose_dot_pose(pm, "pm", pm2, "pm", pm3, "pm", result, "pm");
		EXPECT_TRUE(s_is_equal(16, result, pm_dot_pm2_dot_pm3, error)) << "\"s_pose_dot_pose pm pm pm pm\" failed";

		s_pose_dot_pose(pe313, "pe313", pe313_2, "pe313", result, "pm");
		EXPECT_TRUE(s_is_equal(16, result, pm_dot_pm2, error)) << "\"s_pose_dot_pose pe313 pe313 pm\" failed";

		s_pose_dot_pose(pe313_3, "ipe313", pe313_2, "ipe313", pe313, "ipe313", result, "ipm");
		EXPECT_TRUE(s_is_equal(16, result, pm_dot_pm2_dot_pm3, error)) << "\"s_pose_dot_pose ipe313 ipe313 ipe313 pm\" failed";

		s_pose_dot_pose(pe321, "pe321", pe321_2, "pe321", result, "pm");
		EXPECT_TRUE(s_is_equal(16, result, pm_dot_pm2, error)) << "\"s_pose_dot_pose pe321 pe321 pm\" failed";

		s_pose_dot_pose(pe321_3, "ipe321", pe321_2, "ipe321", pe321, "ipe321", result, "ipm");
		EXPECT_TRUE(s_is_equal(16, result, pm_dot_pm2_dot_pm3, error)) << "\"s_pose_dot_pose ipe321 ipe321 ipe321 pm\" failed";

		s_pose_dot_pose(pq, "pq", pq2, "pq", result, "pm");
		EXPECT_TRUE(s_is_equal(16, result, pm_dot_pm2, error)) << "\"s_pose_dot_pose pq pq pm\" failed";

		s_pose_dot_pose(pq3, "ipq", pq2, "ipq", pq, "ipq", result, "ipm");
		EXPECT_TRUE(s_is_equal(16, result, pm_dot_pm2_dot_pm3, error)) << "\"s_pose_dot_pose ipq ipq ipq pm\" failed";
	}

	// test s_pose_dot_v3
	{
		s_pose_dot_v3(pm, "pm", from_v1, result);
		EXPECT_TRUE(s_is_equal(3, result, to_v1, error)) << "\"s_pose_dot_v3 pm\" failed";

		s_pose_dot_v3(pm, "ipm", to_v1, result);
		EXPECT_TRUE(s_is_equal(3, result, from_v1, error)) << "\"s_pose_dot_v3 ipm\" failed";

		s_pose_dot_v3(pq, "pq", from_v1, result);
		EXPECT_TRUE(s_is_equal(3, result, to_v1, error)) << "\"s_pose_dot_v3 pq\" failed";

		s_pose_dot_v3(pq, "ipq", to_v1, result);
		EXPECT_TRUE(s_is_equal(3, result, from_v1, error)) << "\"s_pose_dot_v3 ipq\" failed";

		s_pose_dot_v3(pe321, "pe321", from_v1, result);
		EXPECT_TRUE(s_is_equal(3, result, to_v1, error)) << "\"s_pose_dot_v3 pe321\" failed";

		s_pose_dot_v3(pe321, "ipe321", to_v1, result);
		EXPECT_TRUE(s_is_equal(3, result, from_v1, error)) << "\"s_pose_dot_v3 ipe321\" failed";

		s_pose_dot_v3(pe313, "pe313", from_v1, result);
		EXPECT_TRUE(s_is_equal(3, result, to_v1, error)) << "\"s_pose_dot_v3 pe313\" failed";

		s_pose_dot_v3(pe313, "ipe313", to_v1, result);
		EXPECT_TRUE(s_is_equal(3, result, from_v1, error)) << "\"s_pose_dot_v3 ipe313\" failed";
	}
}
TEST(DynamicPoseTest, RotationMatrix) {
	const double rm[9]{ -0.22, -0.975499782797526,   0.000416847668728071,
		0.175499782797526, -0.04, -0.983666521865018, 
		0.959583152331272, -0.216333478134982,   0.18 };
	const double rm2[9]{ 0.567219713641686,-0.802125918959455,0.186697098503681,
		0.77780532845257,0.447242474005492,-0.441580163137156,
		0.270704021926224,0.395686971707304,0.877582561890373 };
	const double rm3[9]{ 0.959630766969045,0.00981685903664117,0.281091480420624,
		0.224691174394048,0.574386230075444,-0.787143147622555,
		-0.169182349066996,0.818525557315332,0.548992936917952 };
	const double rm_dot_rm2[9]{ -0.883424423624956,-0.259652292887403,0.390053809802222,
		-0.197847760298552,-0.547886650720495,-0.812820079542285,
		0.424755872811384,-0.79523638294572,0.432644824020405 };
	const double rm_dot_rm2_dot_rm3[9]{ -0.972093055630252,0.16145585737304,0.170197230633687,
		-0.175451282517831, -0.981954799994014, -0.0705805797055172,
		0.155730339556197, -0.0984722137897536,0.982879079263088 };
	const double inv_rm[9]{ -0.22,0.175499782797526,0.959583152331272,
		-0.975499782797526, -0.04, -0.216333478134982,
		0.000416847668728071, -0.983666521865018,0.18 };
	const double inv_rm2[9]{ 0.567219713641686,0.77780532845257,0.270704021926224,
		-0.802125918959455,0.447242474005492,0.395686971707304,
		0.186697098503681, -0.441580163137156,0.877582561890373 };

	const double from_v1[]{ 0.45,0.65,0.13 };
	const double from_v1_ld[]{ 0.45,0,0.65,0,0.13,0 };
	const double to_v1[]{ -0.733020668621457,-0.0749017455835656,0.314595657761334 };
	const double to_v1_ld[]{ -0.733020668621457,0,0,-0.0749017455835656,0,0,0.314595657761334,0,0 };

	double result[36];

	s_inv_rm(rm, result);
	EXPECT_TRUE(s_is_equal(9, result, inv_rm, error)) << "\"s_inv_rm\" failed";

	s_rm_dot_rm(rm, rm2, result);
	EXPECT_TRUE(s_is_equal(9, result, rm_dot_rm2, error)) << "\"s_rm_dot_rm\" failed";

	//s_rm_dot_rm(rm, rm2, rm3, result);
	//if (!s_is_equal(9, result, rm_dot_rm2_dot_rm3, error))std::cout << "\"s_rm_dot_rm\" failed" << std::endl;

	s_inv_rm_dot_rm(inv_rm, rm2, result);
	EXPECT_TRUE(s_is_equal(9, result, rm_dot_rm2, error)) << "\"s_inv_rm_dot_rm\" failed";

	s_rm_dot_inv_rm(rm, inv_rm2, result);
	EXPECT_TRUE(s_is_equal(9, result, rm_dot_rm2, error)) << "\"s_rm_dot_inv_rm\" failed";

	s_rm_dot_v3(rm, from_v1, result);
	EXPECT_TRUE(s_is_equal(3, result, to_v1, error)) << "\"s_rm_dot_v3\" failed";

	//s_fill(9, 1, 0, result, 1);
	//s_rm_dot_v3(rm, from_v1_ld, 2, result, 3);
	//if (!s_is_equal(3, result, to_v1_ld, error))std::cout << "\"s_rm_dot_v3 with ld\" failed" << std::endl;

	s_inv_rm_dot_v3(inv_rm, from_v1, result);
	EXPECT_TRUE(s_is_equal(3, result, to_v1, error)) << "\"s_rm_dot_v3\" failed";

	//s_fill(9, 1, 0, result, 1);
	//s_inv_rm_dot_v3(inv_rm, from_v1_ld, 2, result, 3);
	//if (!s_is_equal(3, result, to_v1_ld, error))std::cout << "\"s_rm_dot_v3 with ld\" failed" << std::endl;
}
TEST(DynamicPoseTest, PoseMatrix) {
	const double pm[16] { -0.22, -0.975499782797526,   0.000416847668728071, 0.1,
		0.175499782797526, -0.04, -0.983666521865018, 0.2,
		0.959583152331272, -0.216333478134982,   0.18, 0.3,
		0,0,0,1 };
	const double pm2[16]{ 0.567219713641686,-0.802125918959455,0.186697098503681,0.1,
		0.77780532845257,0.447242474005492,-0.441580163137156,0.2,
		0.270704021926224,0.395686971707304,0.877582561890373,0.3,
		0,0,0,1 };
	const double pm3[16]{ 0.959630766969045,0.00981685903664117,0.281091480420624,0.11,
		0.224691174394048,0.574386230075444,-0.787143147622555,0.22,
		-0.169182349066996,0.818525557315332,0.548992936917952,-0.33,
		0,0,0,1 };
	const double pm_dot_pm2[16]{ -0.883424423624956,-0.259652292887403,0.390053809802222,-0.116974902258887,
		- 0.197847760298552,-0.547886650720495,-0.812820079542285,-0.0855499782797527,
		0.424755872811384,-0.79523638294572,0.432644824020405,0.406691619606131,
		0,0,0,1, };
	const double pm_dot_pm2_dot_pm3[16]{ -0.972093055630252,0.16145585737304,0.170197230633687, - 0.399992850527594,
		- 0.175451282517831, - 0.981954799994014, - 0.0705805797055172,0.0403823311778517,
		0.155730339556197, - 0.0984722137897536,0.982879079263088,0.135689969440591,
		0,0,0,1 };
	const double inv_pm[16]{ -0.22,0.175499782797526,0.959583152331272, -0.300974902258887,
		- 0.975499782797526, -0.04, -0.216333478134982,0.170450021720247,
		0.000416847668728071, -0.983666521865018,0.18,0.142691619606131,
		0,0,0,1 };
	const double inv_pm2[16]{ 0.567219713641686,0.77780532845257,0.270704021926224,- 0.29349424363255,
		- 0.802125918959455,0.447242474005492,0.395686971707304, - 0.127941994417344,
		0.186697098503681, - 0.441580163137156,0.877582561890373, - 0.193628445790049,
		0,0,0,1 };

	const double from_v1[]{ 0.45,0.65,0.13 };
	const double from_v1_ld[]{ 0.45,0,0.65,0,0.13,0 };
	const double to_v1[]{ -0.733020668621457,-0.0749017455835656,0.314595657761334 };
	const double to_v1_ld[]{ -0.733020668621457,0,0,-0.0749017455835656,0,0,0.314595657761334,0,0 };

	double result[36];

	s_inv_pm(pm, result);
	EXPECT_TRUE(s_is_equal(16, result, inv_pm, error)) << "\"s_inv_pm\" failed";

	s_pm_dot_pm(pm, pm2, result);
	EXPECT_TRUE(s_is_equal(16, result, pm_dot_pm2, error)) << "\"s_pm_dot_pm\" failed";

	s_pm_dot_pm(pm, pm2, pm3, result);
	EXPECT_TRUE(s_is_equal(16, result, pm_dot_pm2_dot_pm3, error)) << "\"s_pm_dot_pm\" failed";

	s_inv_pm_dot_pm(inv_pm, pm2, result);
	EXPECT_TRUE(s_is_equal(16, result, pm_dot_pm2, error)) << "\"s_inv_pm_dot_pm\" failed";

	s_pm_dot_inv_pm(pm, inv_pm2, result);
	EXPECT_TRUE(s_is_equal(16, result, pm_dot_pm2, error)) << "\"s_pm_dot_inv_pm\" failed";

	s_pm_dot_v3(pm, from_v1, result);
	EXPECT_TRUE(s_is_equal(3, result, to_v1, error)) << "\"s_pm_dot_v3\" failed";

	s_fill(16, 1, 0, result, 1);
	s_pm_dot_v3(pm, from_v1_ld, 2, result, 3);
	EXPECT_TRUE(s_is_equal(3, result, to_v1_ld, error)) << "\"s_pm_dot_v3 with ld\" failed";

	s_inv_pm_dot_v3(inv_pm, from_v1, result);
	EXPECT_TRUE(s_is_equal(3, result, to_v1, error)) << "\"s_inv_pm_dot_v3\" failed";

	s_fill(16, 1, 0, result, 1);
	s_inv_pm_dot_v3(inv_pm, from_v1_ld, 2, result, 3);
	EXPECT_TRUE(s_is_equal(3, result, to_v1_ld, error)) << "\"s_inv_pm_dot_v3 with ld\" failed";
}
TEST(DynamicPoseTest, RotationQuaternion) {
	const double rm[9]{ -0.22, -0.975499782797526,   0.000416847668728071,
		0.175499782797526, -0.04, -0.983666521865018,
		0.959583152331272, -0.216333478134982,   0.18 };
	const double rm2[9]{ 0.567219713641686,-0.802125918959455,0.186697098503681,
		0.77780532845257,0.447242474005492,-0.441580163137156,
		0.270704021926224,0.395686971707304,0.877582561890373 };
	const double rm3[9]{ 0.959630766969045,0.00981685903664117,0.281091480420624,
		0.224691174394048,0.574386230075444,-0.787143147622555,
		-0.169182349066996,0.818525557315332,0.548992936917952 };
	const double rm_dot_rm2[9]{ -0.883424423624956,-0.259652292887403,0.390053809802222,
		-0.197847760298552,-0.547886650720495,-0.812820079542285,
		0.424755872811384,-0.79523638294572,0.432644824020405 };
	const double rm_dot_rm2_dot_rm3[9]{ -0.972093055630252,0.16145585737304,0.170197230633687,
		-0.175451282517831, -0.981954799994014, -0.0705805797055172,
		0.155730339556197, -0.0984722137897536,0.982879079263088 };
	const double inv_rm[9]{ -0.22,0.175499782797526,0.959583152331272,
		-0.975499782797526, -0.04, -0.216333478134982,
		0.000416847668728071, -0.983666521865018,0.18 };
	const double inv_rm2[9]{ 0.567219713641686,0.77780532845257,0.270704021926224,
		-0.802125918959455,0.447242474005492,0.395686971707304,
		0.186697098503681, -0.441580163137156,0.877582561890373 };

	const double from_v1[]{ 0.45,0.65,0.13 };
	const double from_v1_ld[]{ 0.45,0,0.65,0,0.13,0 };
	const double to_v1[]{ -0.733020668621457,-0.0749017455835656,0.314595657761334 };
	const double to_v1_ld[]{ -0.733020668621457,0,0,-0.0749017455835656,0,0,0.314595657761334,0,0 };

	double result[36];

	double rq[7], rq2[7], rq3[7], rq_dot_rq2[7], inv_rq[7], inv_rq2[7];
	s_rm2rq(rm, rq);
	s_rm2rq(rm2, rq2);
	s_rm2rq(rm3, rq3);
	s_rm2rq(rm_dot_rm2, rq_dot_rq2);
	s_rm2rq(inv_rm, inv_rq);
	s_rm2rq(inv_rm2, inv_rq2);



	s_inv_rq(rq, result);
	EXPECT_TRUE(s_is_equal(4, result, inv_rq, error)) << "\"s_inv_rq\" failed";

	s_rq_dot_rq(rq, rq2, result);
	EXPECT_TRUE(s_is_equal(4, result, rq_dot_rq2, error)) << "\"s_rq_dot_rq\" failed";

	//s_rq_dot_rq(rq, rq2, rq3, result);
	//if (!s_is_equal(4, result, rq_dot_rq2_dot_rq3, error))std::cout << "\"s_rq_dot_rq\" failed" << std::endl;

	s_inv_rq_dot_rq(inv_rq, rq2, result);
	if (result[3] < 0) s_iv(4, result);
	EXPECT_TRUE(s_is_equal(4, result, rq_dot_rq2, error)) << "\"s_inv_rq_dot_rq\" failed";

	s_rq_dot_inv_rq(rq, inv_rq2, result);
	if (result[3] < 0) s_iv(4, result);
	EXPECT_TRUE(s_is_equal(4, result, rq_dot_rq2, error)) << "\"s_rq_dot_inv_rq\" failed";

	s_rq_dot_v3(rq, from_v1, result);
	EXPECT_TRUE(s_is_equal(3, result, to_v1, error)) << "\"s_rq_dot_v3\" failed";

	//s_fill(4, 1, 0, result, 1);
	//s_rq_dot_v3(rq, from_v1_ld, 2, result, 3);
	//if (!s_is_equal(3, result, to_v1_ld, error))std::cout << "\"s_rq_dot_v3 with ld\" failed" << std::endl;

	s_inv_rq_dot_v3(inv_rq, from_v1, result);
	EXPECT_TRUE(s_is_equal(3, result, to_v1, error)) << "\"s_rq_dot_v3\" failed";

	//s_fill(9, 1, 0, result, 1);
	//s_inv_rq_dot_v3(inv_rq, from_v1_ld, 2, result, 3);
	//if (!s_is_equal(3, result, to_v1_ld, error))std::cout << "\"s_rq_dot_v3 with ld\" failed" << std::endl;
}
TEST(DynamicPoseTest, PoseQuaternion) {
	const double pm[16]{ -0.22, -0.975499782797526,   0.000416847668728071, 0.1,
		0.175499782797526, -0.04, -0.983666521865018, 0.2,
		0.959583152331272, -0.216333478134982,   0.18, 0.3,
		0,0,0,1 };
	const double pm2[16]{ 0.567219713641686,-0.802125918959455,0.186697098503681,0.1,
		0.77780532845257,0.447242474005492,-0.441580163137156,0.2,
		0.270704021926224,0.395686971707304,0.877582561890373,0.3,
		0,0,0,1 };
	const double pm3[16]{ 0.959630766969045,0.00981685903664117,0.281091480420624,0.11,
		0.224691174394048,0.574386230075444,-0.787143147622555,0.22,
		-0.169182349066996,0.818525557315332,0.548992936917952,-0.33,
		0,0,0,1 };
	const double pm_dot_pm2[16]{ -0.883424423624956,-0.259652292887403,0.390053809802222,-0.116974902258887,
		-0.197847760298552,-0.547886650720495,-0.812820079542285,-0.0855499782797527,
		0.424755872811384,-0.79523638294572,0.432644824020405,0.406691619606131,
		0,0,0,1, };
	const double pm_dot_pm2_dot_pm3[16]{ -0.972093055630252,0.16145585737304,0.170197230633687, -0.399992850527594,
		-0.175451282517831, -0.981954799994014, -0.0705805797055172,0.0403823311778517,
		0.155730339556197, -0.0984722137897536,0.982879079263088,0.135689969440591,
		0,0,0,1 };
	const double inv_pm[16]{ -0.22,0.175499782797526,0.959583152331272, -0.300974902258887,
		-0.975499782797526, -0.04, -0.216333478134982,0.170450021720247,
		0.000416847668728071, -0.983666521865018,0.18,0.142691619606131,
		0,0,0,1 };
	const double inv_pm2[16]{ 0.567219713641686,0.77780532845257,0.270704021926224,-0.29349424363255,
		-0.802125918959455,0.447242474005492,0.395686971707304, -0.127941994417344,
		0.186697098503681, -0.441580163137156,0.877582561890373, -0.193628445790049,
		0,0,0,1 };

	const double from_v1[]{ 0.45,0.65,0.13 };
	const double from_v1_ld[]{ 0.45,0,0.65,0,0.13,0 };
	const double to_v1[]{ -0.733020668621457,-0.0749017455835656,0.314595657761334 };
	const double to_v1_ld[]{ -0.733020668621457,0,0,-0.0749017455835656,0,0,0.314595657761334,0,0 };

	double result[36];

	double pq[7], pq2[7], pq3[7], pq_dot_pq2[7], inv_pq[7], inv_pq2[7];
	s_pm2pq(pm, pq);
	s_pm2pq(pm2, pq2);
	s_pm2pq(pm3, pq3);
	s_pm2pq(pm_dot_pm2, pq_dot_pq2);
	s_pm2pq(inv_pm, inv_pq);
	s_pm2pq(inv_pm2, inv_pq2);
	

	s_inv_pq(pq, result);
	EXPECT_TRUE(s_is_equal(7, result, inv_pq, error)) << "\"s_inv_pq\" failed";

	s_pq_dot_pq(pq, pq2, result);
	EXPECT_TRUE(s_is_equal(7, result, pq_dot_pq2, error)) << "\"s_pq_dot_pq\" failed";

	//s_pq_dot_pq(pq, pq2, pq3, result);
	//if (!s_is_equal(7, result, pq_dot_pq2_dot_pq3, error))std::cout << "\"s_pq_dot_pq\" failed" << std::endl;

	s_inv_pq_dot_pq(inv_pq, pq2, result);
	if (result[6] < 0) s_iv(4, result + 3);
	EXPECT_TRUE(s_is_equal(7, result, pq_dot_pq2, error)) << "\"s_inv_pq_dot_pq\" failed";

	s_pq_dot_inv_pq(pq, inv_pq2, result);
	if (result[6] < 0) s_iv(4, result + 3);
	EXPECT_TRUE(s_is_equal(7, result, pq_dot_pq2, error)) << "\"s_pq_dot_inv_pq\" failed";

	s_pq_dot_v3(pq, from_v1, result);
	EXPECT_TRUE(s_is_equal(3, result, to_v1, error)) << "\"s_pq_dot_v3\" failed";

	//s_fill(7, 1, 0, result, 1);
	//s_pq_dot_v3(pq, from_v1_ld, 2, result, 3);
	//if (!s_is_equal(3, result, to_v1_ld, error))std::cout << "\"s_pq_dot_v3 with ld\" failed" << std::endl;

	s_inv_pq_dot_v3(inv_pq, from_v1, result);
	EXPECT_TRUE(s_is_equal(3, result, to_v1, error)) << "\"s_inv_pq_dot_v3\" failed";

	//s_fill(7, 1, 0, result, 1);
	//s_inv_pq_dot_v3(inv_pq, from_v1_ld, 2, result, 3);
	//if (!s_is_equal(3, result, to_v1_ld, error))std::cout << "\"s_pq_dot_v3 with ld\" failed" << std::endl;
}
TEST(DynamicPoseTest, Transform) {
	double result[36];

	const double pm[16] = { -0.22, -0.975499782797526,   0.000416847668728071, 0.1,
		0.175499782797526, -0.04, -0.983666521865018, 0.2,
		0.959583152331272, -0.216333478134982,   0.18, 0.3,
		0,0,0,1 };
	const double inv_pm[16]{ -0.22,0.175499782797526,0.959583152331272, - 0.300974902258887,
		- 0.975499782797526,-0.04,-0.216333478134982,0.170450021720247,
		0.000416847668728071,-0.983666521865018,0.18,0.142691619606131,
		0,0,0,1 };
	const double tmf[36]{ -0.22, - 0.975499782797526,0.000416847668728071,0,0,0,
		0.175499782797526, - 0.04, - 0.983666521865018,0,0,0,
		0.959583152331272, - 0.216333478134982,0.18,0,0,0,
		0.139266695626997, - 0.0312666956269964,0.331099956559505, - 0.22, - 0.975499782797526,0.000416847668728071,
		- 0.161958315233127, - 0.27101658702576, - 0.0178749456993816,0.175499782797526, - 0.04, - 0.983666521865018,
		0.0615499782797526,0.191099956559505, - 0.0984500217202474,0.959583152331272, - 0.216333478134982,0.18, };
	const double tmv[36]{ -0.22, - 0.975499782797526,0.000416847668728071,0.139266695626997, - 0.0312666956269964,0.331099956559505,
		0.175499782797526, - 0.04, - 0.983666521865018, - 0.161958315233127, - 0.27101658702576, - 0.0178749456993816,
		0.959583152331272, - 0.216333478134982,0.18,0.0615499782797526,0.191099956559505, - 0.0984500217202474,
		0,0,0, - 0.22, - 0.975499782797526,0.000416847668728071,
		0,0,0,0.175499782797526, - 0.04, - 0.983666521865018,
		0,0,0,0.959583152331272, - 0.216333478134982,0.18, };

	const double iv_from[10]{ 0.765 , -0.62 , 0.13 , -0.58 , 105.0 , 116.25 , 100.28 , 20.11015 , 12.2000345614 , 0.58539 };
	const double im_from[36]{ 0.765, 0.00, 0.00, 0.00, -0.58, -0.13,
		0.00, 0.765, 0.00, 0.58, 0.00, -0.62,
		0.00, 0.00, 0.765, 0.13, 0.62, 0.00,
		0.00, 0.58, 0.13, 105.00, 20.11015, 12.2000345614,
		-0.58, 0.00, 0.62, 20.11015, 116.25, 0.58539,
		-0.13, -0.62, 0.00, 12.2000345614, 0.58539, 100.28 };

	const double iv_to[10]{ 0.765 ,  0.0858432565884590,0.6095167173472437, - 0.4979649066029362,124.1800379045616154,95.6442751549216865,101.4158797291064076,0.3149734017776580, - 16.0277810847312772, - 11.6496725642042094 };
	const double im_to[36]{ 0.7649999999999996,   0.0000000000000000, - 0.0000000000000004, - 0.0000000000000001, - 0.4979649066029362, - 0.6095167173472437,
		0.0000000000000000,   0.7650000000000006, - 0.0000000000000003,   0.4979649066029360,   0.0000000000000000,   0.0858432565884590,
		- 0.0000000000000004, - 0.0000000000000003,   0.7649999999999999,   0.6095167173472437, - 0.0858432565884590,   0.0000000000000001,
		- 0.0000000000000001,   0.4979649066029360,   0.6095167173472438 ,  124.1800379045616154,   0.3149734017776580, - 16.0277810847312772,
		- 0.4979649066029363,   0.0000000000000000, - 0.0858432565884591,   0.3149734017776592,   95.6442751549216865, - 11.6496725642042094,
		- 0.6095167173472436,   0.0858432565884590 ,  0.0000000000000000, - 16.0277810847312772, - 11.6496725642042058,   101.4158797291064076 };

	s_im2im(pm, im_from, result);
	EXPECT_TRUE(s_is_equal(36, result, im_to, error)) << "\"s_im2im\" failed";

	s_inv_im2im(pm, im_to, result);
	EXPECT_TRUE(s_is_equal(36, result, im_from, error)) << "\"s_inv_im2im\" failed";

	s_iv2iv(pm, iv_from, result);
	EXPECT_TRUE(s_is_equal(10, result, iv_to, error)) << "\"s_iv2iv\" failed";

	s_inv_iv2iv(pm, iv_to, result);
	EXPECT_TRUE(s_is_equal(10, result, iv_from, error)) << "\"s_inv_iv2iv\" failed";

	const double fs_from[]{0.685,0.747,-0.321,0.985,-0.444,0.333};
	const double fs_to[]{ -0.879532145851414,0.406094304734976,0.43793335118009,0.182319092651313,-0.444586644598921,1.31769732898202 };
	const double fs_ld_from[]{ 0.685,0,0,0.747,0,0,-0.321,0,0,0.985,0,0,-0.444,0,0,0.333,0,0 };
	const double fs_ld_to[]{ -0.879532145851414,0,0.406094304734976,0,0.43793335118009,0,0.182319092651313,0,-0.444586644598921,0,1.31769732898202,0 };
	const double fs_alpha_from[]{ 0.685,0.747,-0.321,0.985,-0.444,0.333 };
	const double fs_alpha_to[]{ -0.219883036462854,0.101523576183744,0.109483337795023,0.0455797731628282,-0.11114666114973,0.329424332245505 };
	const double fs_alpha_ld_from[]{ 0.685,0,0,0.747,0,0,-0.321,0,0,0.985,0,0,-0.444,0,0,0.333,0,0 };
	const double fs_alpha_ld_to[]{ -0.219883036462854,0,0.101523576183744,0,0.109483337795023,0,0.0455797731628282,0,-0.11114666114973,0,0.329424332245505,0 };
	const double fsn_from[]{ 0.685,0.825,0.747,-0.133,-0.321,0.256,0.985,0.236,-0.444,-0.187,0.333,0.365 };
	const double fsn_to[]{ -0.879532145851414,-0.0516518158847347,0.406094304734976,-0.101711308789486,0.43793335118009,0.866508453265252,
		0.182319092651313,0.334465692072119, - 0.444586644598921, - 0.412286721832461,1.31769732898202,0.33277521665942 };
	const double fsn_ld_from[]{ 0.685,0.825,0,0,0.747,-0.133,0,0,-0.321,0.256,0,0,0.985,0.236,0,0,-0.444,-0.187,0,0,0.333,0.365,0,0 };
	const double fsn_ld_to[]{ -0.879532145851414,-0.0516518158847347,0,0.406094304734976,-0.101711308789486,0,0.43793335118009,0.866508453265252,0,
		0.182319092651313,0.334465692072119,0,-0.444586644598921, -0.412286721832461,0,1.31769732898202,0.33277521665942,0 };
	const double fsn_alpha_from[]{ 0.685,0.825,0.747,-0.133,-0.321,0.256,0.985,0.236,-0.444,-0.187,0.333,0.365 };
	const double fsn_alpha_to[]{ -0.219883036462854, - 0.0129129539711837,0.101523576183744, - 0.0254278271973715,0.109483337795023,0.216627113316313,
		0.0455797731628282,0.0836164230180298, - 0.11114666114973, - 0.103071680458115,0.329424332245505,0.083193804164855 };
	const double fsn_alpha_ld_from[]{ 0.685,0.825,0,0,0.747,-0.133,0,0,-0.321,0.256,0,0,0.985,0.236,0,0,-0.444,-0.187,0,0,0.333,0.365,0,0 };
	const double fsn_alpha_ld_to[]{ -0.219883036462854, -0.0129129539711837,0,0.101523576183744, -0.0254278271973715,0,0.109483337795023,0.216627113316313,0,
		0.0455797731628282,0.0836164230180298,0, -0.11114666114973,-0.103071680458115,0,0.329424332245505,0.083193804164855,0 };
	
	const double fsa_from[]{ 0.685,0.747,-0.321,0.985,-0.444,0.333 };
	const double fsa_to[]{ -0.979532145851414,0.206094304734976,0.13793335118009,-0.217680907348687,-0.944586644598921,0.71769732898202 };
	double fsa_from_original[]{ 0.300974902258887,-0.170450021720247,-0.142691619606131,0.575499782797526,-0.54,-0.383666521865018 };
	double fsa_to_original[]{ -0.1,-0.2,-0.3,-0.4,-0.5,-0.6 };
	const double fsa_ld_from[]{ 0.685,0,0,0.747,0,0,-0.321,0,0,0.985,0,0,-0.444,0,0,0.333,0,0 };
	const double fsa_ld_to[]{ -0.979532145851414,0,0.206094304734976,0,0.13793335118009,0,-0.217680907348687,0,-0.944586644598921,0,0.71769732898202,0 };
	double fsa_ld_from_original[]{ 0.300974902258887,0,0,-0.170450021720247,0,0,-0.142691619606131,0,0,0.575499782797526,0,0,-0.54,0,0,-0.383666521865018,0,0 };
	double fsa_ld_to_original[]{ -0.1,0,-0.2,0,-0.3,0,-0.4,0,-0.5,0,-0.6,0 };
	const double fsa_alpha_from[]{ 0.685,0.747,-0.321,0.985,-0.444,0.333 };
	const double fsa_alpha_to[]{ -0.319883036462854,-0.098476423816256,-0.190516662204977,-0.354420226837172,-0.61114666114973,-0.270575667754495 };
	double fsa_alpha_from_original[]{ 1.20389960903555,-0.681800086880989,-0.570766478424523,2.3019991311901,-2.16,-1.53466608746007 };
	double fsa_alpha_to_original[]{ -0.1,-0.2,-0.3,-0.4,-0.5,-0.6 };
	const double fsa_alpha_ld_from[]{ 0.685,0,0,0.747,0,0,-0.321,0,0,0.985,0,0,-0.444,0,0,0.333,0,0 };
	const double fsa_alpha_ld_to[]{ -0.319883036462854,0,-0.098476423816256,0,-0.190516662204977,0,-0.354420226837172,0,-0.61114666114973,0,-0.270575667754495,0 };
	double fsa_alpha_ld_from_original[]{ 1.20389960903555,0,0,-0.681800086880989,0,0,-0.570766478424523,0,0,2.3019991311901,0,0,-2.16,0,0,-1.53466608746007,0,0 };
	double fsa_alpha_ld_to_original[]{ -0.1,0,-0.2,0,-0.3,0,-0.4,0,-0.5,0,-0.6,0 };
	const double fsan_from[]{ 0.685,0.825,0.747,-0.133,-0.321,0.256,0.985,0.236,-0.444,-0.187,0.333,0.365 };
	const double fsan_to[]{ -0.979532145851414,0.0583481841152653,0.206094304734976,0.018288691210514,0.13793335118009,0.996508453265252,
		- 0.21768090734868,0.474465692072119, - 0.944586644598921, - 0.262286721832461,0.71769732898202,0.49277521665942 };
	double fsan_from_original[]{ 0.300974902258887, - 0.121605783738768, - 0.170450021720247,0.140228328265276, - 0.142691619606131,0.0945941293802421,
		0.575499782797526, - 0.152944107659995, - 0.54,0.188301658702576, - 0.383666521865018,0.0972141206921431 };
	double fsan_to_original[]{ -0.1,0.11,-0.2,0.12,-0.3,0.13,-0.4,0.14,-0.5,0.15,-0.6,0.16 };
	const double fsan_ld_from[]{ 0.685,0.825,0,0,0.747,-0.133,0,0,-0.321,0.256,0,0,0.985,0.236,0,0,-0.444,-0.187,0,0,0.333,0.365,0,0 };
	const double fsan_ld_to[]{ -0.979532145851414,0.0583481841152653,0,0.206094304734976,0.018288691210514,0,0.13793335118009,0.996508453265252,0,
		-0.21768090734868,0.474465692072119,0,-0.944586644598921,-0.262286721832461,0,0.71769732898202,0.49277521665942,0 };
	double fsan_ld_from_original[]{ 0.300974902258887, -0.121605783738768,0,0, -0.170450021720247,0.140228328265276,0,0, -0.142691619606131,0.0945941293802421,0,0,
		0.575499782797526, -0.152944107659995,0,0, -0.54,0.188301658702576,0,0, -0.383666521865018,0.0972141206921431,0,0 };
	double fsan_ld_to_original[]{ -0.1,0.11,0,-0.2,0.12,0,-0.3,0.13,0,-0.4,0.14,0,-0.5,0.15,0,-0.6,0.16,0 };
	const double fsan_alpha_from[]{ 0.685,0.825,0.747,-0.133,-0.321,0.256,0.985,0.236,-0.444,-0.187,0.333,0.365 };
	const double fsan_alpha_to[]{ -0.319883036462854,0.0970870460288163,-0.098476423816256,0.0945721728026285,-0.190516662204977,0.346627113316313,
		-0.354420226837172,0.22361642301803,-0.61114666114973,0.046928319541885,-0.270575667754495,0.243193804164855 };
	double fsan_alpha_from_original[]{ 1.20389960903555, - 0.486423134955074, - 0.681800086880989,0.560913313061102, - 0.570766478424523,0.378376517520968,
		2.3019991311901, - 0.611776430639978, - 2.16,0.753206634810303, - 1.53466608746007,0.388856482768572 };
	double fsan_alpha_to_original[]{ -0.1,0.11,-0.2,0.12,-0.3,0.13,-0.4,0.14,-0.5,0.15,-0.6,0.16 };
	const double fsan_alpha_ld_from[]{ 0.685,0.825,0,0,0.747,-0.133,0,0,-0.321,0.256,0,0,0.985,0.236,0,0,-0.444,-0.187,0,0,0.333,0.365,0,0 };
	const double fsan_alpha_ld_to[]{ -0.319883036462854,0.0970870460288163,0,-0.098476423816256,0.0945721728026285,0,-0.190516662204977,0.346627113316313,0,
		-0.354420226837172,0.22361642301803,0,-0.61114666114973,0.046928319541885,0,-0.270575667754495,0.243193804164855,0 };
	double fsan_alpha_ld_from_original[]{ 1.20389960903555, -0.486423134955074,0,0, -0.681800086880989,0.560913313061102,0,0, -0.570766478424523,0.378376517520968,0,0,
		2.3019991311901, -0.611776430639978,0,0, -2.16,0.753206634810303,0,0, -1.53466608746007,0.388856482768572,0,0 };
	double fsan_alpha_ld_to_original[]{ -0.1,0.11,0,-0.2,0.12,0,-0.3,0.13,0,-0.4,0.14,0,-0.5,0.15,0,-0.6,0.16,0 };

	s_tmf(pm, result);
	EXPECT_TRUE(s_is_equal(36, result, tmf, error)) << "\"s_tmf\" failed";

	s_tf(pm, fs_from, result);
	EXPECT_TRUE(s_is_equal(6, result, fs_to, error)) << "\"s_tf\" failed";

	std::fill_n(result, 36, 0);
	s_tf(pm, fs_ld_from, 3, result, 2);
	EXPECT_TRUE(s_is_equal(12, result, fs_ld_to, error)) << "\"s_tf with ld\" failed";

	s_tf(0.25, pm, fs_alpha_from, result);
	EXPECT_TRUE(s_is_equal(6, result, fs_alpha_to, error)) << "\"s_tf\" failed";

	std::fill_n(result, 36, 0);
	s_tf(0.25, pm, fs_alpha_ld_from, 3, result, 2);
	EXPECT_TRUE(s_is_equal(12, result, fs_alpha_ld_to, error)) << "\"s_tf with ld\" failed";

	s_tf_n(2, pm, fsn_from, result);
	EXPECT_TRUE(s_is_equal(12, result, fsn_to, error)) << "\"s_tf_n\" failed";

	std::fill_n(result, 36, 0);
	s_tf_n(2, pm, fsn_ld_from, 4, result, 3);
	EXPECT_TRUE(s_is_equal(18, result, fsn_ld_to, error)) << "\"s_tf_n with ld\" failed";

	s_tf_n(2, 0.25, pm, fsn_alpha_from, result);
	EXPECT_TRUE(s_is_equal(12, result, fsn_alpha_to, error)) << "\"s_tf_n\" failed";

	std::fill_n(result, 36, 0);
	s_tf_n(2, 0.25, pm, fsn_alpha_ld_from, 4, result, 3);
	EXPECT_TRUE(s_is_equal(18, result, fsn_alpha_ld_to, error)) << "\"s_tf_n with ld\" failed";

	s_tfa(pm, fsa_from, fsa_to_original);
	EXPECT_TRUE(s_is_equal(6, fsa_to_original, fsa_to, error)) << "\"s_tfa\" failed";

	s_tfa(pm, fsa_ld_from, 3, fsa_ld_to_original, 2);
	EXPECT_TRUE(s_is_equal(12, fsa_ld_to_original, fsa_ld_to, error)) << "\"s_tfa with ld\" failed";

	s_tfa(0.25, pm, fsa_alpha_from, fsa_alpha_to_original);
	EXPECT_TRUE(s_is_equal(6, fsa_alpha_to_original, fsa_alpha_to, error)) << "\"s_tfa\" failed";

	s_tfa(0.25, pm, fsa_alpha_ld_from, 3, fsa_alpha_ld_to_original, 2);
	EXPECT_TRUE(s_is_equal(12, fsa_alpha_ld_to_original, fsa_alpha_ld_to, error)) << "\"s_tfa with ld\" failed";

	s_tfa_n(2, pm, fsan_from, fsan_to_original);
	EXPECT_TRUE(s_is_equal(12, fsan_to_original, fsan_to, error)) << "\"s_tfa_n\" failed";

	s_tfa_n(2, pm, fsan_ld_from, 4, fsan_ld_to_original, 3);
	EXPECT_TRUE(s_is_equal(18, fsan_ld_to_original, fsan_ld_to, error)) << "\"s_tfa_n with ld\" failed";

	s_tfa_n(2, 0.25, pm, fsan_alpha_from, fsan_alpha_to_original);
	EXPECT_TRUE(s_is_equal(12, fsan_alpha_to_original, fsan_alpha_to, error)) << "\"s_tfa_n\" failed";

	s_tfa_n(2, 0.25, pm, fsan_alpha_ld_from, 4, fsan_alpha_ld_to_original, 3);
	EXPECT_TRUE(s_is_equal(18, fsan_alpha_ld_to_original, fsan_alpha_ld_to, error)) << "\"s_tfa_n with ld\" failed";

	s_inv_tf(pm, fs_to, result);
	EXPECT_TRUE(s_is_equal(6, result, fs_from, error)) << "\"s_inv_tf\" failed";

	std::fill_n(result, 18, 0);
	s_inv_tf(pm, fs_ld_to, 2, result, 3);
	EXPECT_TRUE(s_is_equal(18, result, fs_ld_from, error)) << "\"s_inv_tf with ld\" failed";

	s_inv_tf(4.0, pm, fs_alpha_to, result);
	EXPECT_TRUE(s_is_equal(6, result, fs_alpha_from, error)) << "\"s_inv_tf\" failed";

	std::fill_n(result, 18, 0);
	s_inv_tf(4.0, pm, fs_alpha_ld_to, 2, result, 3);
	EXPECT_TRUE(s_is_equal(18, result, fs_alpha_ld_from, error)) << "\"s_inv_tf with ld\" failed";

	s_inv_tf_n(2, pm, fsn_to, result);
	EXPECT_TRUE(s_is_equal(12, result, fsn_from, error)) << "\"s_tf_n\" failed";

	std::fill_n(result, 36, 0);
	s_inv_tf_n(2, pm, fsn_ld_to, 3, result, 4);
	EXPECT_TRUE(s_is_equal(24, result, fsn_ld_from, error)) << "\"s_tf_n with ld\" failed";

	s_inv_tf_n(2, 4.0, pm, fsn_alpha_to, result);
	EXPECT_TRUE(s_is_equal(12, result, fsn_alpha_from, error)) << "\"s_tf_n\" failed";

	std::fill_n(result, 36, 0);
	s_inv_tf_n(2, 4.0, pm, fsn_alpha_ld_to, 3, result, 4);
	EXPECT_TRUE(s_is_equal(24, result, fsn_alpha_ld_from, error)) << "\"s_tf_n with ld\" failed";

	s_inv_tfa(pm, fsa_to, fsa_from_original);
	EXPECT_TRUE(s_is_equal(6, fsa_from_original, fsa_from, error)) << "\"s_inv_tfa\" failed";

	s_inv_tfa(pm, fsa_ld_to, 2, fsa_ld_from_original, 3);
	EXPECT_TRUE(s_is_equal(18, fsa_ld_from_original, fsa_ld_from, error)) << "\"s_inv_tfa with ld\" failed";

	s_inv_tfa(4.0, pm, fsa_alpha_to, fsa_alpha_from_original);
	EXPECT_TRUE(s_is_equal(6, fsa_alpha_from_original, fsa_alpha_from, error)) << "\"s_inv_tfa\" failed";

	s_inv_tfa(4.0, pm, fsa_alpha_ld_to, 2, fsa_alpha_ld_from_original, 3);
	EXPECT_TRUE(s_is_equal(18, fsa_alpha_ld_from_original, fsa_alpha_ld_from, error)) << "\"s_inv_tfa with ld\" failed";

	s_inv_tfa_n(2, pm, fsan_to, fsan_from_original);
	EXPECT_TRUE(s_is_equal(12, fsan_from_original, fsan_from, error)) << "\"s_inv_tfa\" failed";

	s_inv_tfa_n(2, pm, fsan_ld_to, 3, fsan_ld_from_original, 4);
	EXPECT_TRUE(s_is_equal(24, fsan_ld_from_original, fsan_ld_from, error)) << "\"s_inv_tfa with ld\" failed";

	s_inv_tfa_n(2, 4.0, pm, fsan_alpha_to, fsan_alpha_from_original);
	EXPECT_TRUE(s_is_equal(12, fsan_alpha_from_original, fsan_alpha_from, error)) << "\"s_inv_tfa\" failed";

	s_inv_tfa_n(2, 4.0, pm, fsan_alpha_ld_to, 3, fsan_alpha_ld_from_original, 4);
	EXPECT_TRUE(s_is_equal(24, fsan_alpha_ld_from_original, fsan_alpha_ld_from, error)) << "\"s_inv_tfa with ld\" failed";

	const double vs_from[]{ 0.685,0.747,-0.321,0.985,-0.444,0.333 };
	const double vs_to[]{ -0.61821575226612,0.360944371951889,0.380927841840383,0.216560713835788, - 0.136933665725488,1.10118146933823 };
	const double vs_ld_from[]{ 0.685,0,0,0.747,0,0,-0.321,0,0,0.985,0,0,-0.444,0,0,0.333,0,0 };
	const double vs_ld_to[]{ -0.61821575226612,0,0.360944371951889,0,0.380927841840383,0,0.216560713835788,0,-0.136933665725488,0,1.10118146933823,0 };
	const double vs_alpha_from[]{ 0.685,0.747,-0.321,0.985,-0.444,0.333 };
	const double vs_alpha_to[]{ -0.15455393806653,0.0902360929879723,0.0952319604600957,0.054140178458947, - 0.034233416431372,0.275295367334558 };
	const double vs_alpha_ld_from[]{ 0.685,0,0,0.747,0,0,-0.321,0,0,0.985,0,0,-0.444,0,0,0.333,0,0 };
	const double vs_alpha_ld_to[]{ -0.15455393806653,0,0.0902360929879723,0,0.0952319604600957,0,0.054140178458947,0, -0.034233416431372,0,0.275295367334558,0 };
	const double vsn_from[]{ 0.685,0.825,0.747,-0.133,-0.321,0.256,0.985,0.236,-0.444,-0.187,0.333,0.365 };
	const double vsn_to[]{ -0.61821575226612,0.107913480509704,0.360944371951889, - 0.0957777245909609,0.380927841840383,0.809364298334756,
		0.216560713835788,0.130650608782223, - 0.136933665725488, - 0.310140331740515,1.10118146933823,0.332615984361422 };
	const double vsn_ld_from[]{ 0.685,0.825,0,0,0.747,-0.133,0,0,-0.321,0.256,0,0,0.985,0.236,0,0,-0.444,-0.187,0,0,0.333,0.365,0,0 };
	const double vsn_ld_to[]{ -0.61821575226612,0.107913480509704,0,0.360944371951889, -0.0957777245909609,0,0.380927841840383,0.809364298334756,0,
		0.216560713835788,0.130650608782223,0, -0.136933665725488, -0.310140331740515,0,1.10118146933823,0.332615984361422,0 };
	const double vsn_alpha_from[]{ 0.685,0.825,0.747,-0.133,-0.321,0.256,0.985,0.236,-0.444,-0.187,0.333,0.365 };
	const double vsn_alpha_to[]{ -0.15455393806653,0.026978370127426,0.0902360929879723, - 0.0239444311477402,0.0952319604600957,0.202341074583689,
		0.054140178458947,0.0326626521955558, - 0.034233416431372, - 0.0775350829351287,0.275295367334558,0.0831539960903555 };
	const double vsn_alpha_ld_from[]{ 0.685,0.825,0,0,0.747,-0.133,0,0,-0.321,0.256,0,0,0.985,0.236,0,0,-0.444,-0.187,0,0,0.333,0.365,0,0 };
	const double vsn_alpha_ld_to[]{ -0.15455393806653,0.026978370127426,0,0.0902360929879723, -0.0239444311477402,0,0.0952319604600957,0.202341074583689,0,
		0.054140178458947,0.0326626521955558,0, -0.034233416431372, -0.0775350829351287,0,0.275295367334558,0.0831539960903555,0 };

	const double vsa_from[]{ 0.685,0.747,-0.321,0.985,-0.444,0.333 };
	const double vsa_to[]{ -0.71821575226612,0.160944371951889,0.080927841840383,-0.183439286164212,-0.636933665725488,0.50118146933823 };
	double vsa_from_original[]{ 0.312632409860973,-0.203805019548222,-0.0782591228641679,0.575499782797526,-0.54,-0.383666521865018 };
	double vsa_to_original[]{ -0.1,-0.2,-0.3,-0.4,-0.5,-0.6 };
	const double vsa_ld_from[]{ 0.685,0,0,0.747,0,0,-0.321,0,0,0.985,0,0,-0.444,0,0,0.333,0,0 };
	const double vsa_ld_to[]{ -0.71821575226612,0,0.160944371951889,0,0.080927841840383,0,-0.183439286164212,0,-0.636933665725488,0,0.50118146933823,0 };
	double vsa_ld_from_original[]{ 0.312632409860973,0,0,-0.203805019548222,0,0,-0.0782591228641679,0,0,0.575499782797526,0,0,-0.54,0,0,-0.383666521865018,0,0 };
	double vsa_ld_to_original[]{ -0.1,0,-0.2,0,-0.3,0,-0.4,0,-0.5,0,-0.6,0 };
	const double vsa_alpha_from[]{ 0.685,0.747,-0.321,0.985,-0.444,0.333 };
	const double vsa_alpha_to[]{ -0.25455393806653,-0.109763907012028,-0.204768039539904,-0.345859821541053,-0.534233416431372,-0.324704632665443 };
	double vsa_alpha_from_original[]{ 1.25052963944389, - 0.81522007819289, - 0.313036491456672,2.3019991311901, - 2.16, - 1.53466608746007 };
	double vsa_alpha_to_original[]{ -0.1,-0.2,-0.3,-0.4,-0.5,-0.6 };
	const double vsa_alpha_ld_from[]{ 0.685,0,0,0.747,0,0,-0.321,0,0,0.985,0,0,-0.444,0,0,0.333,0,0 };
	const double vsa_alpha_ld_to[]{ -0.25455393806653,0,-0.109763907012028,0,-0.204768039539904,0,-0.345859821541053,0,-0.534233416431372,0,-0.324704632665443,0 };
	double vsa_alpha_ld_from_original[]{ 1.25052963944389,0,0,-0.81522007819289,0,0,-0.313036491456672,0,0,2.3019991311901,0,0,-2.16,0,0,-1.53466608746007,0,0 };
	double vsa_alpha_ld_to_original[]{ -0.1,0,-0.2,0,-0.3,0,-0.4,0,-0.5,0,-0.6,0 };
	const double vsan_from[]{ 0.685,0.825,0.747,-0.133,-0.321,0.256,0.985,0.236,-0.444,-0.187,0.333,0.365 };
	const double vsan_to[]{ -0.71821575226612,0.217913480509704,0.160944371951889,0.0242222754090391,0.080927841840383,0.939364298334756,
		- 0.183439286164212,0.270650608782223, - 0.636933665725488, - 0.160140331740515,0.50118146933823,0.492615984361422 };
	double vsan_from_original[]{ 0.312632409860973, - 0.126657370366339, - 0.203805019548222,0.154682160657398, - 0.0782591228641679,0.0666733807920581,
		0.575499782797526, - 0.149058271792632, - 0.54,0.177183326093251, - 0.383666521865018,0.118691619606131 };
	double vsan_to_original[]{ -0.1,0.11,-0.2,0.12,-0.3,0.13,-0.4,0.14,-0.5,0.15,-0.6,0.16 };
	const double vsan_ld_from[]{ 0.685,0.825,0,0,0.747,-0.133,0,0,-0.321,0.256,0,0,0.985,0.236,0,0,-0.444,-0.187,0,0,0.333,0.365,0,0 };
	const double vsan_ld_to[]{ -0.71821575226612,0.217913480509704,0,0.160944371951889,0.0242222754090391,0,0.080927841840383,0.939364298334756,0,
		-0.183439286164212,0.270650608782223,0, -0.636933665725488, -0.160140331740515,0,0.50118146933823,0.492615984361422,0 };
	double vsan_ld_from_original[]{ 0.312632409860973, -0.126657370366339,0,0, -0.203805019548222,0.154682160657398,0,0, -0.0782591228641679,0.0666733807920581,0,0,
		0.575499782797526, -0.149058271792632,0,0, -0.54,0.177183326093251,0,0, -0.383666521865018,0.118691619606131,0,0 };
	double vsan_ld_to_original[]{ -0.1,0.11,0,-0.2,0.12,0,-0.3,0.13,0,-0.4,0.14,0,-0.5,0.15,0,-0.6,0.16,0 };
	const double vsan_alpha_from[]{ 0.685,0.825,0.747,-0.133,-0.321,0.256,0.985,0.236,-0.444,-0.187,0.333,0.365 };
	const double vsan_alpha_to[]{ -0.25455393806653,0.136978370127426,-0.109763907012028,0.0960555688522598,-0.204768039539904,0.332341074583689,
		-0.345859821541053,0.172662652195556,-0.534233416431372,0.0724649170648713,-0.324704632665443,0.243153996090355 };
	double vsan_alpha_from_original[]{ 1.25052963944389, - 0.506629481465357, - 0.81522007819289,0.618728642629592, - 0.313036491456672,0.266693523168233,
		2.3019991311901, - 0.59623308717053, - 2.16,0.708733304373003, - 1.53466608746007,0.474766478424523 };
	double vsan_alpha_to_original[]{ -0.1,0.11,-0.2,0.12,-0.3,0.13,-0.4,0.14,-0.5,0.15,-0.6,0.16 };
	const double vsan_alpha_ld_from[]{ 0.685,0.825,0,0,0.747,-0.133,0,0,-0.321,0.256,0,0,0.985,0.236,0,0,-0.444,-0.187,0,0,0.333,0.365,0,0 };
	const double vsan_alpha_ld_to[]{ -0.25455393806653,0.136978370127426,0,-0.109763907012028,0.0960555688522598,0,-0.204768039539904,0.332341074583689,0,
		-0.345859821541053,0.172662652195556,0,-0.534233416431372,0.0724649170648713,0,-0.324704632665443,0.243153996090355,0 };
	double vsan_alpha_ld_from_original[]{ 1.25052963944389,-0.506629481465357,0,0,-0.81522007819289,0.618728642629592,0,0,-0.313036491456672,0.266693523168233,0,0,
		2.3019991311901,-0.59623308717053,0,0,-2.16,0.708733304373003,0,0,-1.53466608746007,0.474766478424523,0,0 };
	double vsan_alpha_ld_to_original[]{ -0.1,0.11,0,-0.2,0.12,0,-0.3,0.13,0,-0.4,0.14,0,-0.5,0.15,0,-0.6,0.16,0 };

	s_tmv(pm, result);
	EXPECT_TRUE(s_is_equal(36, result, tmv, error)) << "\"s_tmv\" failed";

	s_tv(pm, vs_from, result);
	EXPECT_TRUE(s_is_equal(6, result, vs_to, error)) << "\"s_tv\" failed";

	std::fill_n(result, 36, 0);
	s_tv(pm, vs_ld_from, 3, result, 2);
	EXPECT_TRUE(s_is_equal(12, result, vs_ld_to, error)) << "\"s_tv with ld\" failed";

	s_tv(0.25, pm, vs_alpha_from, result);
	EXPECT_TRUE(s_is_equal(6, result, vs_alpha_to, error)) << "\"s_tv\" failed";

	std::fill_n(result, 36, 0);
	s_tv(0.25, pm, vs_alpha_ld_from, 3, result, 2);
	EXPECT_TRUE(s_is_equal(12, result, vs_alpha_ld_to, error)) << "\"s_tv with ld\" failed";

	s_tv_n(2, pm, vsn_from, result);
	EXPECT_TRUE(s_is_equal(12, result, vsn_to, error)) << "\"s_tv_n\" failed";

	std::fill_n(result, 36, 0);
	s_tv_n(2, pm, vsn_ld_from, 4, result, 3);
	EXPECT_TRUE(s_is_equal(18, result, vsn_ld_to, error)) << "\"s_tv_n with ld\" failed";

	s_tv_n(2, 0.25, pm, vsn_alpha_from, result);
	EXPECT_TRUE(s_is_equal(12, result, vsn_alpha_to, error)) << "\"s_tv_n\" failed";

	std::fill_n(result, 36, 0);
	s_tv_n(2, 0.25, pm, vsn_alpha_ld_from, 4, result, 3);
	EXPECT_TRUE(s_is_equal(18, result, vsn_alpha_ld_to, error)) << "\"s_tv_n with ld\" failed";

	s_tva(pm, vsa_from, vsa_to_original);
	EXPECT_TRUE(s_is_equal(6, vsa_to_original, vsa_to, error)) << "\"s_tva\" failed";

	s_tva(pm, vsa_ld_from, 3, vsa_ld_to_original, 2);
	EXPECT_TRUE(s_is_equal(12, vsa_ld_to_original, vsa_ld_to, error)) << "\"s_tva with ld\" failed";

	s_tva(0.25, pm, vsa_alpha_from, vsa_alpha_to_original);
	EXPECT_TRUE(s_is_equal(6, vsa_alpha_to_original, vsa_alpha_to, error)) << "\"s_tva\" failed";

	s_tva(0.25, pm, vsa_alpha_ld_from, 3, vsa_alpha_ld_to_original, 2);
	EXPECT_TRUE(s_is_equal(12, vsa_alpha_ld_to_original, vsa_alpha_ld_to, error)) << "\"s_tva with ld\" failed";

	s_tva_n(2, pm, vsan_from, vsan_to_original);
	EXPECT_TRUE(s_is_equal(12, vsan_to_original, vsan_to, error)) << "\"s_tva_n\" failed";

	s_tva_n(2, pm, vsan_ld_from, 4, vsan_ld_to_original, 3);
	EXPECT_TRUE(s_is_equal(18, vsan_ld_to_original, vsan_ld_to, error)) << "\"s_tva_n with ld\" failed";

	s_tva_n(2, 0.25, pm, vsan_alpha_from, vsan_alpha_to_original);
	EXPECT_TRUE(s_is_equal(12, vsan_alpha_to_original, vsan_alpha_to, error)) << "\"s_tva_n\" failed";

	s_tva_n(2, 0.25, pm, vsan_alpha_ld_from, 4, vsan_alpha_ld_to_original, 3);
	EXPECT_TRUE(s_is_equal(18, vsan_alpha_ld_to_original, vsan_alpha_ld_to, error)) << "\"s_tva_n with ld\" failed";


	s_inv_tv(pm, vs_to, result);
	EXPECT_TRUE(s_is_equal(6, result, vs_from, error)) << "\"s_inv_tv\" failed";

	std::fill_n(result, 18, 0);
	s_inv_tv(pm, vs_ld_to, 2, result, 3);
	EXPECT_TRUE(s_is_equal(18, result, vs_ld_from, error)) << "\"s_inv_tv with ld\" failed";

	s_inv_tv(4.0, pm, vs_alpha_to, result);
	EXPECT_TRUE(s_is_equal(6, result, vs_alpha_from, error)) << "\"s_inv_tv\" failed";

	std::fill_n(result, 18, 0);
	s_inv_tv(4.0, pm, vs_alpha_ld_to, 2, result, 3);
	EXPECT_TRUE(s_is_equal(18, result, vs_alpha_ld_from, error)) << "\"s_inv_tv with ld\" failed";

	s_inv_tv_n(2, pm, vsn_to, result);
	EXPECT_TRUE(s_is_equal(12, result, vsn_from, error)) << "\"s_tv_n\" failed";

	std::fill_n(result, 36, 0);
	s_inv_tv_n(2, pm, vsn_ld_to, 3, result, 4);
	EXPECT_TRUE(s_is_equal(24, result, vsn_ld_from, error)) << "\"s_tv_n with ld\" failed";

	s_inv_tv_n(2, 4.0, pm, vsn_alpha_to, result);
	EXPECT_TRUE(s_is_equal(12, result, vsn_alpha_from, error)) << "\"s_tv_n\" failed";

	std::fill_n(result, 36, 0);
	s_inv_tv_n(2, 4.0, pm, vsn_alpha_ld_to, 3, result, 4);
	EXPECT_TRUE(s_is_equal(24, result, vsn_alpha_ld_from, error)) << "\"s_tv_n with ld\" failed";

	s_inv_tva(pm, vsa_to, vsa_from_original);
	EXPECT_TRUE(s_is_equal(6, vsa_from_original, vsa_from, error)) << "\"s_inv_tva\" failed";

	s_inv_tva(pm, vsa_ld_to, 2, vsa_ld_from_original, 3);
	EXPECT_TRUE(s_is_equal(18, vsa_ld_from_original, vsa_ld_from, error)) << "\"s_inv_tva with ld\" failed";

	s_inv_tva(4.0, pm, vsa_alpha_to, vsa_alpha_from_original);
	EXPECT_TRUE(s_is_equal(6, vsa_alpha_from_original, vsa_alpha_from, error)) << "\"s_inv_tva\" failed";

	s_inv_tva(4.0, pm, vsa_alpha_ld_to, 2, vsa_alpha_ld_from_original, 3);
	EXPECT_TRUE(s_is_equal(18, vsa_alpha_ld_from_original, vsa_alpha_ld_from, error)) << "\"s_inv_tva with ld\" failed";

	s_inv_tva_n(2, pm, vsan_to, vsan_from_original);
	EXPECT_TRUE(s_is_equal(12, vsan_from_original, vsan_from, error)) << "\"s_inv_tva\" failed";

	s_inv_tva_n(2, pm, vsan_ld_to, 3, vsan_ld_from_original, 4);
	EXPECT_TRUE(s_is_equal(24, vsan_ld_from_original, vsan_ld_from, error)) << "\"s_inv_tva with ld\" failed";

	s_inv_tva_n(2, 4.0, pm, vsan_alpha_to, vsan_alpha_from_original);
	EXPECT_TRUE(s_is_equal(12, vsan_alpha_from_original, vsan_alpha_from, error)) << "\"s_inv_tva\" failed";

	s_inv_tva_n(2, 4.0, pm, vsan_alpha_ld_to, 3, vsan_alpha_ld_from_original, 4);
	EXPECT_TRUE(s_is_equal(24, vsan_alpha_ld_from_original, vsan_alpha_ld_from, error)) << "\"s_inv_tva with ld\" failed";

	double fsa_to_original11[]{ -0.1,0.11,-0.2,0.12,-0.3,0.13,-0.4,0.14,-0.5,0.15,-0.6,0.16 };
	s_inv_tv_n(2, -1.0, pm, fsa_to_original11, result);


}
TEST(DynamicPoseTest, VariableChange) {
	const double pp[3] = { 0.1, 0.2, 0.3 };
	const double re313[3] = { 0.000423769269879415,   1.38980987554835,   1.79253453841257 };
	const double re321[3] = { 2.46823966120654, -1.28551725555848,  5.40636866254317 };
	const double ra[3] = { 0.9760647868936518, - 1.2200809836170647,   1.4640971803404776 };
	const double rq[4] = { 0.4,-0.5, 0.6, std::sqrt(1 - 0.4*0.4 - 0.5*0.5 - 0.6*0.6) };
	const double rm[9] = { -0.22, -0.975499782797526,   0.000416847668728071,
		0.175499782797526, -0.04, -0.983666521865018,
		0.959583152331272, -0.216333478134982,   0.18 };
	const double pe313[6] = { 0.1, 0.2, 0.3,0.000423769269879415,   1.38980987554835,   1.79253453841257 };
	const double pe321[6] = { 0.1, 0.2, 0.3,2.46823966120654, -1.28551725555848,  5.40636866254317 };
	const double pq[7] = { 0.1, 0.2, 0.3,0.4,-0.5, 0.6, std::sqrt(1 - 0.4*0.4 - 0.5*0.5 - 0.6*0.6) };
	const double pa[6] = { 0.1, 0.2, 0.3,0.9760647868936518, -1.2200809836170647,   1.4640971803404776 };
	const double ps[6]{ 0.4137969022551177,0.1579034055412022,0.0557215697809234,0.9760647868936518, -1.2200809836170647,1.4640971803404776 };
	const double pm[16] = { -0.22, -0.975499782797526,   0.000416847668728071, 0.1,
		0.175499782797526, -0.04, -0.983666521865018, 0.2,
		0.959583152331272, -0.216333478134982,   0.18, 0.3,
		0,0,0,1 };

	const double vp[3] = { 0.307558670154491,   1.2433000508379, -1.04895965543501 };
	const double we313[3] = { -0.644213536852877, - 0.245050866834802, - 1.27836042009784 };
	const double we321[3] = { -4.19969388864156, -0.83045134600268,   3.46543753721832 };
	const double wq[4] = { 0.1, 0.2, -0.4, -(rq[0] * 0.1 + rq[1] * 0.2 - rq[2] * 0.4) / rq[3] };
	const double wm[9] = { 1.36, -0.30698536874045, -0.633709981238717,
		0.426985368740452,   0.8,   0.0436487757967661,
		0.233709981238715,   1.23635122420323,   0.24 , };
	const double ve313[6] = { 0.307558670154491,   1.2433000508379, -1.04895965543501,-0.644213536852877, -0.245050866834802, -1.27836042009784 };
	const double ve321[6] = { 0.307558670154491,   1.2433000508379, -1.04895965543501,-4.19969388864156, -0.83045134600268,   3.46543753721832 };
	const double vq[7] = { 0.307558670154491,   1.2433000508379, -1.04895965543501, 0.1, 0.2, -0.4, -(rq[0] * 0.1 + rq[1] * 0.2 - rq[2] * 0.4) / rq[3] };
	const double vm[16] = { 1.36, -0.30698536874045, -0.633709981238717,0.307558670154491,
		0.426985368740452,   0.8,   0.0436487757967661,1.2433000508379,
		0.233709981238715,   1.23635122420323,   0.24 , -1.04895965543501,
		0,0,0,0 };
	const double wa[3] = { -0.244517963270725,	1.25737650310373,	-0.874318412470487 };
	const double va[6] = { 0.307558670154491,   1.2433000508379, -1.04895965543501, -0.244517963270725,	1.25737650310373,	-0.874318412470487 };
	const double vs[6] = { -0.244517963270725,	1.25737650310373,	-0.874318412470487, -0.244517963270725,	1.25737650310373,	-0.874318412470487 };
	
	const double ap[3] = { 2.2628985000154, -0.843606386309081, -0.248846478459814 };
	const double xe313[3] = { 1.51734920338156,   1.71538128045296,   1.3693196878275 };
	const double xe321[3] = { -15.6049676192293,   4.50445705187534,   16.9352080725126 };
	const double xq[4] = { -0.033,   0.022, 0.011,   -(wq[0] * wq[0] + wq[1] * wq[1] + wq[2] * wq[2] + wq[3] * wq[3] + rq[0] * (-0.033) + rq[1] * (0.022) + rq[2] * (0.011)) / rq[3] };
	const double xm[9] = { -0.782400000000002,   2.58144759895694,   1.54784395313479,
		-2.32024759895695, -0.653600000000002,   0.450521351741563,
		-1.92944395313478, -1.05972135174157, -0.103200000000001 };
	const double ae313[6] = { 2.2628985000154, -0.843606386309081, -0.248846478459814, 1.51734920338156,   1.71538128045296,   1.3693196878275 };
	const double ae321[6] = { 2.2628985000154, -0.843606386309081, -0.248846478459814, -15.6049676192293,   4.50445705187534,   16.9352080725126 };
	const double aq[7] = { 2.2628985000154, -0.843606386309081, -0.248846478459814, -0.033,   0.022, 0.011,   -(wq[0] * wq[0] + wq[1] * wq[1] + wq[2] * wq[2] + wq[3] * wq[3] + rq[0] * (-0.033) + rq[1] * (0.022) + rq[2] * (0.011)) / rq[3] };
	const double am[16] = { -0.782400000000002,   2.58144759895694,   1.54784395313479,2.2628985000154,
		-2.32024759895695, -0.653600000000002,   0.450521351741563,-0.843606386309081,
		-1.92944395313478, -1.05972135174157, -0.103200000000001,-0.248846478459814,
		0,0,0,0 };
	const double xa[3] = { 0.904633672502324, -1.24440604199266,   1.45568007018557 };
	const double aa[6] = { 2.2628985000154, -0.843606386309081, -0.248846478459814, 0.904633672502324, -1.24440604199266,   1.45568007018557 };
	const double as[6] = { 3.15925342342501, -0.192390604845803,   0.136512424183815,   0.904633672502324, -1.24440604199266,   1.45568007018557 };

	const double iv[10]{ 1 , -0.62 , 0.13 , -0.58 , 105.0 , 116.25 , 100.28 , 20.11015 , 12.2000345614 , 0.58539 };
	const double im[36]{ 1.00, 0.00, 0.00, 0.00, - 0.58, - 0.13,
		0.00,   1.00,   0.00,   0.58,   0.00, - 0.62,
		0.00,   0.00,   1.00,   0.13,   0.62,   0.00,
		0.00,   0.58,   0.13,   105.00,   20.11015,   12.2000345614,
		- 0.58,   0.00,   0.62,   20.11015,   116.25,   0.58539,
		- 0.13, - 0.62,   0.00,   12.2000345614,   0.58539,   100.28 };


	double result_pm_for_position[16] = { -0.22, -0.975499782797526,   0.000416847668728071, 0,
		0.175499782797526, -0.04, -0.983666521865018, 0,
		0.959583152331272, -0.216333478134982,   0.18,0,
		0,0,0,1 };
	double result_vs_for_position[6] = { 0,	0,	0, -0.244517963270725,	1.25737650310373,	-0.874318412470487 };
	double result_as_for_position[6] = { 0,  0,   0,   0.904633672502324, -1.24440604199266,   1.45568007018557 };
	double result_pm_for_angle[16] = { 1,0,0,0.1,
		0,1,0,0.2,
		0,0,1,0.3,
		0,0,0,1 };
	double result_vs_for_angle[6] = { -0.244517963270725,	1.25737650310373,	-0.874318412470487, 0,0,0 };
	double result_as_for_angle[6] = { 3.15925342342501, -0.192390604845803,   0.136512424183815,   0,0,0 };
	double result[36];
	double result1[36];


	s_ra2rm(ra, result);
	EXPECT_TRUE(s_is_equal(9, result, rm, error)) << "\"s_ra2rm\" failed";

	s_rm2ra(rm, result);
	EXPECT_TRUE(s_is_equal(3, result, ra, error)) << "\"s_rm2ra\" failed";

	s_re2rm(re321, result, "321");
	EXPECT_TRUE(s_is_equal(9, result, rm, error)) << "\"s_re2rm 321\" failed";

	s_re2rm(re313, result);
	EXPECT_TRUE(s_is_equal(9, result, rm, error)) << "\"s_re2rm 313\" failed";

	s_rm2re(rm, result, "321");
	EXPECT_TRUE(s_is_equal(3, result, re321, error)) << "\"s_rm2re 321\" failed";

	s_rm2re(rm, result);
	EXPECT_TRUE(s_is_equal(3, result, re313, error)) << "\"s_rm2re 313\" failed";

	s_rq2rm(rq, result);
	EXPECT_TRUE(s_is_equal(9, result, rm, error)) << "\"s_rq2rm\" failed";

	s_rm2rq(rm, result);
	EXPECT_TRUE(s_is_equal(4, result, rq, error)) << "\"s_rm2rq\" failed";

	s_pp2pm(pp, result_pm_for_position);
	EXPECT_TRUE(s_is_equal(16, result_pm_for_position, pm, error)) << "\"s_pp2pm\" failed";

	s_pm2pp(pm, result);
	EXPECT_TRUE(s_is_equal(3, result, pp, error)) << "\"s_pm2pp\" failed";
	
	s_re2pm(re321, result_pm_for_angle, "321");
	EXPECT_TRUE(s_is_equal(16, result_pm_for_angle, pm, error)) << "\"s_re2pm\" failed";

	s_re2pm(re313, result_pm_for_angle);
	EXPECT_TRUE(s_is_equal(16, result_pm_for_angle, pm, error)) << "\"s_re2pm\" failed";
	
	s_pm2re(pm, result, "321");
	EXPECT_TRUE(s_is_equal(3, result, re321, error)) << "\"s_pm2re\" failed";

	s_pm2re(pm, result);
	EXPECT_TRUE(s_is_equal(3, result, re313, error)) << "\"s_pm2re\" failed";

	s_ra2pm(ra, result_pm_for_angle);
	EXPECT_TRUE(s_is_equal(16, result_pm_for_angle, pm, error)) << "\"s_ra2pm\" failed";

	s_pm2ra(pm, result);
	EXPECT_TRUE(s_is_equal(3, result, ra, error)) << "\"s_pm2ra\" failed";

	s_rq2pm(rq, result_pm_for_angle);
	EXPECT_TRUE(s_is_equal(16, result_pm_for_angle, pm, error)) << "\"s_rq2pm\" failed";

	s_pm2rq(pm, result);
	EXPECT_TRUE(s_is_equal(4, result, rq, error)) << "\"s_pm2rq\" failed";

	s_rm2pm(rm, result_pm_for_angle);
	EXPECT_TRUE(s_is_equal(16, result_pm_for_angle, pm, error)) << "\"s_rm2pm\" failed";

	s_pm2rm(pm, result);
	EXPECT_TRUE(s_is_equal(9, result, rm, error)) << "\"s_pm2rm\" failed";

	s_pe2pm(pe321, result, "321");
	EXPECT_TRUE(s_is_equal(16, pm, result, error)) << "\"s_pe2pm\" failed";

	s_pe2pm(pe313, result);
	EXPECT_TRUE(s_is_equal(16, result, pm, error)) << "\"s_pe2pm\" failed";
	
	s_pm2pe(pm, result, "321");
	EXPECT_TRUE(s_is_equal(6, result, pe321, error)) << "\"s_pm2pe\" failed";

	s_pm2pe(pm, result);
	EXPECT_TRUE(s_is_equal(6, result, pe313, error)) << "\"s_pm2pe\" failed";
	
	s_pq2pm(pq, result);
	EXPECT_TRUE(s_is_equal(16, pm, result, error)) << "\"s_pq2pm\" failed";

	s_pm2pq(pm, result);
	EXPECT_TRUE(s_is_equal(7, pq, result, error)) << "\"s_pm2pq\" failed";

	s_pa2pm(pa, result);
	EXPECT_TRUE(s_is_equal(16, pm, result, error)) << "\"s_pa2pm\" failed";

	s_pm2pa(pm, result);
	EXPECT_TRUE(s_is_equal(6, pa, result, error)) << "\"s_pm2pa\" failed";

	s_ps2pm(ps, result);
	EXPECT_TRUE(s_is_equal(16, pm, result, error)) << "\"s_ps2pm\" failed";

	s_pm2ps(pm, result);
	EXPECT_TRUE(s_is_equal(6, ps, result, error)) << "\"s_pm2ps\" failed";


	s_wa2we(wa, re313, result, "313");
	EXPECT_TRUE(s_is_equal(3, result, we313, error)) << "\"s_wa2we 313\" failed";

	s_wa2we(wa, re321, result, "321");
	EXPECT_TRUE(s_is_equal(3, result, we321, error)) << "\"s_wa2we 321\" failed";

	s_we2wa(re313, we313, result, "313");
	EXPECT_TRUE(s_is_equal(3, result, wa, error)) << "\"s_we2wa 313\" failed";

	s_we2wa(re321, we321, result, "321");
	EXPECT_TRUE(s_is_equal(3, result, wa, error)) << "\"s_we2wa 321\" failed";

	s_wq2wa(rq, wq, result);
	EXPECT_TRUE(s_is_equal(3, result, wa, error)) << "\"s_wq2wa\" failed";

	s_wa2wq(wa, rq, result);
	EXPECT_TRUE(s_is_equal(4, result, wq, error)) << "\"s_wa2wq\" failed";

	s_wa2wm(wa, rm, result);
	EXPECT_TRUE(s_is_equal(9, result, wm, error)) << "\"s_wa2wm\" failed";

	s_wm2wa(rm, wm, result);
	EXPECT_TRUE(s_is_equal(3, result, wa, error)) << "\"s_wm2wa\" failed";

	s_vs2vp(vs, pp, result);
	EXPECT_TRUE(s_is_equal(3, result, vp, error)) << "\"s_vs2vp\" failed";

	s_vp2vs(pp, vp, result_vs_for_position);
	EXPECT_TRUE(s_is_equal(6, result_vs_for_position, vs, error)) << "\"s_vp2vs\" failed";

	s_we2vs(re313, we313, result, "313");
	EXPECT_TRUE(s_is_equal(3, result + 3, vs + 3, error)) << "\"s_we2vs 313\" failed";

	s_we2vs(re321, we321, result, "321");
	EXPECT_TRUE(s_is_equal(3, result + 3, vs + 3, error)) << "\"s_we2vs 321\" failed";
	
	s_vs2we(vs, re313, result, "313");
	EXPECT_TRUE(s_is_equal(3, result, we313, error)) << "\"s_vs2we 313\" failed";

	s_vs2we(vs, re321, result, "321");
	EXPECT_TRUE(s_is_equal(3, result, we321, error)) << "\"s_vs2we 321\" failed";

	s_wq2vs(rq, wq, result_vs_for_angle);
	EXPECT_TRUE(s_is_equal(6, result_vs_for_angle, vs, error)) << "\"s_wq2vs\" failed";

	s_vs2wq(vs, rq, result);
	EXPECT_TRUE(s_is_equal(3, result, wq, error)) << "\"s_vs2wq\" failed";

	s_wm2vs(rm, wm, result_vs_for_angle);
	EXPECT_TRUE(s_is_equal(6, result_vs_for_angle, vs, error)) << "\"s_wm2vs\" failed";

	s_vs2wm(vs, rm, result);
	EXPECT_TRUE(s_is_equal(9, result, wm, error)) << "\"s_vs2wm\" failed";

	s_wa2vs(wa, result_vs_for_angle);
	EXPECT_TRUE(s_is_equal(6, result_vs_for_angle, vs, error)) << "\"s_wa2vs\" failed";

	s_vs2wa(vs, result);
	EXPECT_TRUE(s_is_equal(3, result, wa, error)) << "\"s_vs2wa\" failed";

	s_ve2vs(pe313, ve313, result, "313");
	EXPECT_TRUE(s_is_equal(6, result, vs, error)) << "\"s_ve2vs 313\" failed";

	s_ve2vs(pe321, ve321, result, "321");
	EXPECT_TRUE(s_is_equal(6, result, vs, error)) << "\"s_ve2vs 321\" failed";

	s_vs2ve(vs, pe313, result, "313");
	EXPECT_TRUE(s_is_equal(3, result, ve313, error)) << "\"s_vs2ve 313\" failed";

	s_vs2ve(vs, pe321, result, "321");
	EXPECT_TRUE(s_is_equal(3, result, ve321, error)) << "\"s_vs2ve 321\" failed";

	s_vs2vq(vs, pq, result);
	EXPECT_TRUE(s_is_equal(7, result, vq, error)) << "\"s_vs2vq\" failed";

	s_vq2vs(pq, vq, result);
	EXPECT_TRUE(s_is_equal(6, result, vs, error)) << "\"s_vq2vs\" failed";

	s_vs2vm(vs, pm, result);
	EXPECT_TRUE(s_is_equal(16, result, vm, error)) << "\"s_vs2vm\" failed";

	s_vm2vs(pm, vm, result);
	EXPECT_TRUE(s_is_equal(6, result, vs, error)) << "\"s_vm2vs\" failed";

	s_vs2va(vs, pp, result);
	EXPECT_TRUE(s_is_equal(6, result, va, error)) << "\"s_vs2va\" failed";

	s_va2vs(pp, va, result);
	EXPECT_TRUE(s_is_equal(6, result, vs, error)) << "\"s_va2vs\" failed";


	s_xa2xe(wa, xa, re313, result, result1, "313");
	EXPECT_TRUE(s_is_equal(3, result, xe313, error)&&(s_is_equal(3, result1, we313, error))) << "\"s_xa2xe 313\" failed";

	s_xa2xe(wa, xa, re321, result, result1, "321");
	EXPECT_TRUE(s_is_equal(3, result, xe321, error)&&(s_is_equal(3, result1, we321, error))) << "\"s_xa2xe 321\" failed";

	s_xe2xa(re313, we313, xe313, result, result1, "313");
	EXPECT_TRUE(s_is_equal(3, result, xa, error)&&(s_is_equal(3, result1, wa, error))) << "\"s_xe2xa 313\" failed";

	s_xe2xa(re321, we321, xe321, result, result1, "321");
	EXPECT_TRUE(s_is_equal(3, result, xa, error)&&(s_is_equal(3, result1, wa, error))) << "\"s_xe2xa 321\" failed";

	s_xq2xa(rq, wq, xq, result);
	EXPECT_TRUE(s_is_equal(3, result, xa, error)) << "\"s_xq2xa\" failed";

	s_xa2xq(wa, xa, rq, result);
	EXPECT_TRUE(s_is_equal(4, result, xq, error)) << "\"s_xa2xq\" failed";

	s_xa2xm(wa, xa, rm, result);
	EXPECT_TRUE(s_is_equal(9, result, xm, error)) << "\"s_xa2xm\" failed";

	s_xm2xa(rm, wm, xm, result);
	EXPECT_TRUE(s_is_equal(3, result, xa, error)) << "\"s_xm2xa\" failed";

	s_as2ap(vs, as, pp, result);
	EXPECT_TRUE(s_is_equal(3, result, ap, error)) << "\"s_as2ap\" failed";

	s_ap2as(pp, vp, ap, result_as_for_position, result_vs_for_position);
	EXPECT_TRUE(s_is_equal(6, result_as_for_position, as, error) && (s_is_equal(6, result_vs_for_position, vs, error))) << "\"s_ap2as\" failed";

	s_xe2as(re313, we313, xe313, result, result1);
	EXPECT_TRUE(s_is_equal(3, result + 3, as + 3, error) && (s_is_equal(3, result1 + 3, vs + 3, error))) << "\"s_xe2as 313\" failed";

	s_xe2as(re321, we321, xe321, result, result1, "321");
	EXPECT_TRUE(s_is_equal(3, result + 3, as + 3, error) && (s_is_equal(3, result1 + 3, vs + 3, error))) << "\"s_xe2as 321\" failed";

	s_as2xe(vs, as, re313, result, result1);
	EXPECT_TRUE(s_is_equal(3, result, xe313, error)) << "\"s_as2xe 313\" failed";

	s_as2xe(vs, as, re321, result, result1, "321");
	EXPECT_TRUE(s_is_equal(3, result, xe321, error)) << "\"s_as2xe 321\" failed";

	s_xq2as(rq, wq, xq, result_as_for_angle);
	EXPECT_TRUE(s_is_equal(6, result_as_for_angle, as, error)) << "\"s_xq2as\" failed";

	s_as2xq(vs, as, rq, result);
	EXPECT_TRUE(s_is_equal(3, result, xq, error)) << "\"s_as2xq\" failed";

	s_xm2as(rm, wm, xm, result_as_for_angle);
	EXPECT_TRUE(s_is_equal(6, result_as_for_angle, as, error)) << "\"s_xm2as\" failed";

	s_as2xm(vs, as, rm, result);
	EXPECT_TRUE(s_is_equal(9, result, xm, error)) << "\"s_as2xm\" failed";

	s_xa2as(xa, result_as_for_angle);
	EXPECT_TRUE(s_is_equal(6, result_as_for_angle, as, error)) << "\"s_xa2as\" failed";

	s_as2xa(as, result);
	EXPECT_TRUE(s_is_equal(3, result, xa, error)) << "\"s_as2xa\" failed";

	s_as2ae(vs, as, pe313, result, result1);
	EXPECT_TRUE(s_is_equal(6, result, ae313, error)&&s_is_equal(6, result1, ve313, error)) << "\"s_as2ae\" failed";

	s_as2ae(vs, as, pe321, result, result1, "321");
	EXPECT_TRUE(s_is_equal(6, result, ae321, error) && s_is_equal(6, result1, ve321, error)) << "\"s_as2ae\" failed";

	s_ae2as(pe313, ve313, ae313, result, result1);
	EXPECT_TRUE(s_is_equal(6, result, as, error) && s_is_equal(6, result1, vs, error)) << "\"s_ae2as\" failed";

	s_ae2as(pe321, ve321, ae321, result, result1, "321");
	EXPECT_TRUE(s_is_equal(6, result, as, error) && s_is_equal(6, result1, vs, error)) << "\"s_ae2as\" failed";

	s_as2aq(vs, as, pq, result);
	EXPECT_TRUE(s_is_equal(7, result, aq, error)) << "\"s_as2aq\" failed";

	s_aq2as(pq, vq, aq, result);
	EXPECT_TRUE(s_is_equal(6, result, as, error)) << "\"s_aq2as\" failed";

	s_as2am(vs, as, pm, result);
	EXPECT_TRUE(s_is_equal(16, result, am, error)) << "\"s_as2am\" failed";

	s_am2as(pm, vm, am, result);
	EXPECT_TRUE(s_is_equal(6, result, as, error)) << "\"s_am2as\" failed";

	s_as2aa(vs, as, pp, result);
	EXPECT_TRUE(s_is_equal(6, result, aa, error)) << "\"s_as2aa\" failed";

	s_aa2as(pp, va, aa, result);
	EXPECT_TRUE(s_is_equal(6, result, as, error)) << "\"s_aa2as\" failed";


	s_iv2im(iv, result);
	EXPECT_TRUE(s_is_equal(36, result, im, error)) << "\"s_iv2im\" failed";

	s_im2iv(im, result);
	EXPECT_TRUE(s_is_equal(10, result, iv, error)) << "\"s_im2iv\" failed";

}
TEST(DynamicPoseTest, CoordinateTransform) {
	const double relative_vs[16] = { 0.12, -0.35, 0.26, 0.58, 0.36, -0.135 };
	const double relative_as[16] = { 0.14, 1.35, -0.35, -0.56, -0.34, 0.14 };
	const double relative_pm[16] = { -0.22, -0.975499782797526,   0.000416847668728071,   0.1,
		0.175499782797526, -0.04, -0.983666521865018,   0.2,
		0.959583152331272, -0.216333478134982,   0.18,   0.3,
		0,   0,   0,   1 };

	const double from_pp[3]{ 0.13, -0.22, 0.45 };
	const double to_pp[3]{ 0.286197533666383, -0.21103496307558,   0.553339174992761 };
	const double from_re_313[3]{ 4.83, 0.76, 0.45 };
	const double to_re_321[3]{ 6.03703796978214, -0.696087712802565,   2.29525788843731 };
	const double from_re_321[3]{ 3.856, -0.696087712802565,   2.29525788843731 };
	const double to_re_313[3]{ 3.42785042695091,   2.77225969913703,   4.30966052384328 };
	const double from_rq[4]{ 0.4,-0.5, 0.6, std::sqrt(1 - 0.4*0.4 - 0.5*0.5 - 0.6*0.6) };
	const double to_rq[4]{ -0.383666521865017,   0.479583152331272, -0.575499782797526,   0.54 };
	const double from_rm[9]{ 0.808307066774345, -0.072065911490471, 0.584333971461272,
		0.341746746490328,   0.865601553329486, -0.365982393206091,
		-0.479425538604203,   0.495520388354132,   0.724300143351802 };
	const double to_rm[9]{ -0.511401279081528, -0.828333070215518,   0.228764194184996,
		0.599782696845049, -0.534698430872375, -0.595280021996288,
		0.615409983928647, -0.16721815933072,   0.770265304210822 };
	const double from_pe_313[6]{ 0.13, -0.22, 0.45, 4.83, 0.76, 0.45 };
	const double to_pe_321[6]{ 0.286197533666383, -0.21103496307558,   0.553339174992761,6.03703796978214, -0.696087712802565,   2.29525788843731 };
	const double from_pe_321[6]{ 0.13, -0.22, 0.45, 3.856, -0.696087712802565,   2.29525788843731 };
	const double to_pe_313[6]{ 0.286197533666383, -0.21103496307558,   0.553339174992761,3.42785042695091,   2.77225969913703,   4.30966052384328 };
	const double from_pq[7]{ 0.13, -0.22, 0.45, 0.4,-0.5, 0.6, std::sqrt(1 - 0.4*0.4 - 0.5*0.5 - 0.6*0.6) };
	const double to_pq[7]{ 0.286197533666383, -0.21103496307558,   0.553339174992761, -0.383666521865017,   0.479583152331272, -0.575499782797526,   0.54 };
	const double from_pm[16]{ 0.808307066774345, -0.072065911490471, 0.584333971461272, 0.13,
		0.341746746490328,   0.865601553329486, -0.365982393206091,-0.22,
		-0.479425538604203,   0.495520388354132,   0.724300143351802,0.45,
		0,0,0,1 };
	const double to_pm[16]{ -0.511401279081528, -0.828333070215518,   0.228764194184996, 0.286197533666383,
		0.599782696845049, -0.534698430872375, -0.595280021996288, -0.21103496307558,
		0.615409983928647, -0.16721815933072,   0.770265304210822,   0.553339174992761,
		0 ,  0 ,  0,   1 };

	const double from_vp[3]{ 0.131, -0.221, 0.451 };
	const double to_vp[3]{ 0.47766583327904, -1.12137651835541,   0.289263700919493 };
	const double from_we_313[3]{ 1.03, 0.73, 0.25 };
	const double to_we_321[3]{ -0.677310010504109, -0.42894848193491,   1.82130169173659 };
	const double from_we_321[3]{ 2.15, 0.76, 1.25 };
	const double to_we_313[3]{ -8.60099441931481, - 2.91077197886926, - 9.69351986665146 };
	const double from_wq[4]{ 0.1, 0.2, -0.4, -(from_rq[0] * 0.1 + from_rq[1] * 0.2 - from_rq[2] * 0.4) / from_rq[3] };
	const double to_wq[4]{ -0.292793710222811,   0.286847417856529, -0.141803596258453, -0.613907911417607 };
	const double from_wm[9] = { 0.0291954874793394, -0.182528252419306, -0.0628972223520089,
		0.196923491067634, -0.108112316607135, -0.0718182822377014,
		0.189595409314217,   0.162310423942327,   0.0144536170779726, };
	const double to_wm[9]{ 0.104075460683106,   0.0133045900952464,   0.280834839144192,
		-0.477150412775713,   0.0214429907194124, -0.500020299468546,
		0.551519952476166, -0.134472030683956, -0.469834287697671 };
	const double from_wa[3]{ -0.918, 0.928, 0.458 };
	const double to_wa[3]{ -0.123112882203827, -0.288748067622307, -1.13421480154937 };
	const double from_ve_313[6]{ 0.131, -0.221, 0.451, 1.03, 0.73, 0.25 };
	const double to_ve_321[6]{ 0.47766583327904, -1.12137651835541,   0.289263700919493, -0.677310010504109, -0.42894848193491,   1.82130169173659 };
	const double from_ve_321[6]{ 0.131, -0.221, 0.451, 2.15, 0.76, 1.25 };
	const double to_ve_313[6]{ 0.47766583327904, -1.12137651835541,   0.289263700919493, -8.60099441931481, -2.91077197886926, -9.69351986665146 };
	const double from_vq[7] = { 0.03,   0.1525,   0.355,    0.1, 0.2, -0.4, -(from_rq[0] * 0.1 + from_rq[1] * 0.2 - from_rq[2] * 0.4) / from_rq[3] };
	const double to_vq[7] = { 0.135496647027967, -1.05961001031892,   0.0942652484506193,   -0.292793710222811,   0.286847417856529, -0.141803596258453, -0.613907911417607 };
	const double from_vm[16] = { 0.0291954874793394, -0.182528252419306, -0.0628972223520089,   0.03,
		0.196923491067634, -0.108112316607135, -0.0718182822377014,   0.1525,
		0.189595409314217,   0.162310423942327,   0.0144536170779726,   0.355,
		0 ,  0,   0,   0 };
	const double to_vm[16] = { 0.104075460683106,   0.0133045900952464,   0.280834839144192,   0.135496647027967,
		-0.477150412775713,   0.0214429907194124, -0.500020299468546, -1.05961001031892,
		0.551519952476166, -0.134472030683956, -0.469834287697671,   0.0942652484506192,
		0,   0 ,  0,   0 };
	const double from_va[6] = { 0.089,   0.26325 ,  0.3935 ,  0.2, -0.15,   0.125 };
	const double to_va[6] = { 0.0144960947183866, -1.09155668422567,   0.133851721734715,   0.68237707337822,   0.278141641326378,   0.111866652186502 };
	const double from_vs[6] = { -0.1873, -0.1412,   0.1365,   0.04982,   0.1345,   0.03744 };
	const double to_vs[6] = { 0.314132747625686, -0.556683321739415,   0.160469757942957,   0.43785048599045,   0.326534924600346, -0.109551220160011 };
	
	const double from_ap[3]{ 0.12,   0.13, -0.14 };
	const double to_ap[3]{ -0.183017842291836,   1.44829117674852, -1.20119377113485 };
	const double from_xe_313[3]{ 4.83, 0.76, 0.45 };
	const double to_xe_321[3]{ -0.916190408904469, - 4.34620837330883,   2.91658574391217 };
	const double from_xe_321[3]{ 3.856, -0.696087712802565,   2.29525788843731 };
	const double to_xe_313[3]{ 188.69146781591,   49.4283048797986,   191.948584567729 };
	const double from_xq[4]{ -0.033,   0.022, 0.011,   -(from_wq[0] * from_wq[0] + from_wq[1] * from_wq[1] + from_wq[2] * from_wq[2] + from_wq[3] * from_wq[3] + from_rq[0] * (-0.033) + from_rq[1] * 0.022 + from_rq[2] * 0.011) / from_rq[3] };
	const double to_xq[4]{ 0.0195967544652706, - 0.915878348828718,   0.85596957486878,   0.693274013402014 };
	const double from_xm[9] = { -0.19607150371156, -0.0824023375945621,   0.195817097864919,
		0.0345175399147836,   0.0110332817274978,   0.210315261589722,
		-0.148327659454663, -0.175246744763661, -0.0645778320357833};
	const double to_xm[9]{ -0.168189089112595,   0.0240777134273968, -0.681683032081897,
		0.0646710887930607,   0.0754796609525905,   0.800601976050679,
		-1.08460897398942, -0.248679220742461,   0.107617812109963};
	const double from_xa[3]{ -0.16,   0.17,   0.18 };
	const double to_xa[3]{ -1.13785824818199,   0.122524884812844, - 0.141063237283511 };
	const double from_aa[6] = { -0.28023125, -0.165775,   0.1247, -0.16,   0.17,   0.18 };
	const double to_aa[6] = { 0.0898265507450043,   1.43483887623279, -1.10554368182179, -0.612738814129007, -0.708943502357538, -0.10224359010281 };
	const double from_ae_313[6]{ 0.12,   0.13, -0.14,4.83, 0.76, 0.45 };
	const double to_ae_321[6]{ -0.183017842291836,   1.44829117674852, -1.20119377113485,-0.916190408904469, -4.34620837330883,   2.91658574391217 };
	const double from_ae_321[6]{ 0.12,   0.13, -0.14,3.856, -0.696087712802565,   2.29525788843731 };
	const double to_ae_313[6]{ -0.183017842291836,   1.44829117674852, -1.20119377113485,188.69146781591,   49.4283048797986,   191.948584567729 };
	const double from_aq[7] = { -0.1873125, -0.14125,   0.136,   from_xq[0],   from_xq[1],   from_xq[2],   from_xq[3] };
	const double to_aq[7] = { 0.025588345140825,   1.4522998248645, -1.06971424926844,   to_xq[0],   to_xq[1],   to_xq[2],   to_xq[3] };
	const double from_am[16] = { -0.19607150371156, -0.0824023375945621,   0.195817097864919, -0.1873125,
		0.0345175399147836,   0.0110332817274978,   0.210315261589722, -0.14125,
		-0.148327659454663, -0.175246744763661, -0.0645778320357833,   0.136,
		0,   0,   0,   0 };
	const double to_am[16] = { -0.168189089112595,   0.0240777134273968, -0.681683032081897,   0.025588345140825,
		0.0646710887930607,   0.0754796609525905,   0.800601976050679,   1.4522998248645,
		-1.08460897398942, -0.248679220742461,   0.107617812109963, -1.06971424926844,
		0,   0,   0,   0 };
	const double from_as[6] = { -0.1899, -0.1475,   0.3165,   0.9482,   0.3145,   0.7344 };
	const double to_as[6] = { 0.627235974450473,   0.750818078071529, -0.640716977516731, -1.07044877319847, -0.904145907524959,   1.1457959474787 };

	double result[16], result2[16], result3[16];

	s_pp2pp(relative_pm, from_pp, result);
	EXPECT_TRUE(s_is_equal(3, to_pp, result, error)) << "\"s_pp2pp\" failed";

	s_inv_pp2pp(relative_pm, to_pp, result);
	EXPECT_TRUE(s_is_equal(3, from_pp, result, error)) << "\"s_inv_pp2pp\" failed";

	s_re2re(relative_pm, from_re_313, result, "313", "321");
	EXPECT_TRUE(s_is_equal(3, to_re_321, result, error)) << "\"s_re2re\" failed";

	s_inv_re2re(relative_pm, to_re_321, result, "321", "313");
	EXPECT_TRUE(s_is_equal(3, from_re_313, result, error)) << "\"s_inv_re2re\" failed";

	s_re2re(relative_pm, from_re_321, result, "321", "313");
	EXPECT_TRUE(s_is_equal(3, to_re_313, result, error)) << "\"s_re2re\" failed";

	s_inv_re2re(relative_pm, to_re_313, result, "313", "321");
	EXPECT_TRUE(s_is_equal(3, from_re_321, result, error)) << "\"s_inv_re2re\" failed";

	s_rq2rq(relative_pm, from_rq, result);
	EXPECT_TRUE(s_is_equal(4, to_rq, result, error)) << "\"s_rq2rq\" failed";

	s_inv_rq2rq(relative_pm, to_rq, result);
	EXPECT_TRUE(s_is_equal(4, from_rq, result, error)) << "\"s_inv_rq2rq\" failed";

	s_rm2rm(relative_pm, from_rm, result);
	EXPECT_TRUE(s_is_equal(3, to_rm, result, error)) << "\"s_rm2rm\" failed";

	s_inv_rm2rm(relative_pm, to_rm, result);
	EXPECT_TRUE(s_is_equal(3, from_rm, result, error)) << "\"s_inv_rm2rm\" failed";

	s_pe2pe(relative_pm, from_pe_313, result, "313", "321");
	EXPECT_TRUE(s_is_equal(6, to_pe_321, result, error)) << "\"s_pe2pe 313 to 321\" failed";

	s_inv_pe2pe(relative_pm, to_pe_321, result, "321", "313");
	EXPECT_TRUE(s_is_equal(6, from_pe_313, result, error)) << "\"s_inv_pe2pe 321 to 313\" failed";

	s_pe2pe(relative_pm, from_pe_321, result, "321", "313");
	EXPECT_TRUE(s_is_equal(6, to_pe_313, result, error)) << "\"s_pe2pe 321 to 313\" failed";

	s_inv_pe2pe(relative_pm, to_pe_313, result, "313", "321");
	EXPECT_TRUE(s_is_equal(6, from_pe_321, result, error)) << "\"s_inv_pe2pe 313 to 321\" failed";

	s_pq2pq(relative_pm, from_pq, result);
	EXPECT_TRUE(s_is_equal(7, to_pq, result, error)) << "\"s_pq2pq\" failed";

	s_inv_pq2pq(relative_pm, to_pq, result);
	EXPECT_TRUE(s_is_equal(7, from_pq, result, error)) << "\"s_inv_pq2pq\" failed";

	s_pm2pm(relative_pm, from_pm, result);
	EXPECT_TRUE(s_is_equal(16, to_pm, result, error)) << "\"s_pm2pm\" failed";

	s_inv_pm2pm(relative_pm, to_pm, result);
	EXPECT_TRUE(s_is_equal(16, from_pm, result, error)) << "\"s_inv_pm2pm\" failed";



	s_vp2vp(relative_pm, relative_vs, from_pp, from_vp, result, result2);
	EXPECT_TRUE(s_is_equal(3, to_vp, result, error) && s_is_equal(3, to_pp, result2, error)) << "\"s_vp2vp\" failed";

	s_inv_vp2vp(relative_pm, relative_vs, to_pp, to_vp, result, result2);
	EXPECT_TRUE(s_is_equal(3, from_vp, result, error) && s_is_equal(3, from_pp, result2, error)) << "\"s_inv_vp2vp\" failed";

	s_we2we(relative_pm, relative_vs, from_re_313, from_we_313, result, result2, "313", "321");
	EXPECT_TRUE(s_is_equal(3, to_we_321, result, error) && s_is_equal(3, to_re_321, result2, error)) << "\"s_we2we 313 to 321\" failed";

	s_inv_we2we(relative_pm, relative_vs, to_re_321, to_we_321, result, result2, "321", "313");
	EXPECT_TRUE(s_is_equal(3, from_we_313, result, error) && s_is_equal(3, from_re_313, result2, error)) << "\"s_inv_we2we 313\" failed";

	s_we2we(relative_pm, relative_vs, from_re_321, from_we_321, result, result2, "321", "313");
	EXPECT_TRUE(s_is_equal(3, to_we_313, result, error) && s_is_equal(3, to_re_313, result2, error)) << "\"s_we2we 313 to 321\" failed";

	s_inv_we2we(relative_pm, relative_vs, to_re_313, to_we_313, result, result2, "313", "321");
	EXPECT_TRUE(s_is_equal(3, from_we_321, result, error) && s_is_equal(3, from_re_321, result2, error)) << "\"s_inv_we2we 313\" failed";

	s_wq2wq(relative_pm, relative_vs, from_rq, from_wq, result, result2);
	EXPECT_TRUE(s_is_equal(4, to_wq, result, error) && s_is_equal(4, to_rq, result2, error)) << "\"s_wq2wq\" failed";

	s_inv_wq2wq(relative_pm, relative_vs, to_rq, to_wq, result, result2);
	EXPECT_TRUE(s_is_equal(4, from_wq, result, error) && s_is_equal(4, from_rq, result2, error)) << "\"s_inv_wq2wq\" failed";

	s_wm2wm(relative_pm, relative_vs, from_rm, from_wm, result, result2);
	EXPECT_TRUE(s_is_equal(9, to_wm, result, error) && s_is_equal(9, to_rm, result2, error)) << "\"s_wm2wm\" failed";

	s_inv_wm2wm(relative_pm, relative_vs, to_rm, to_wm, result, result2);
	EXPECT_TRUE(s_is_equal(9, from_wm, result, error) && s_is_equal(9, from_rm, result2, error)) << "\"s_inv_wm2wm\" failed";

	s_wa2wa(relative_pm, relative_vs, from_wa, result);
	EXPECT_TRUE(s_is_equal(3, to_wa, result, error)) << "\"s_wa2wa\" failed";

	s_inv_wa2wa(relative_pm, relative_vs, to_wa, result);
	EXPECT_TRUE(s_is_equal(3, from_wa, result, error)) << "\"s_inv_wa2wa\" failed";

	s_ve2ve(relative_pm, relative_vs, from_pe_313, from_ve_313, result, result2, "313", "321");
	EXPECT_TRUE(s_is_equal(6, to_ve_321, result, error) && s_is_equal(3, to_pe_321, result2, error)) << "\"s_ve2ve 313 to 321\" failed";

	s_inv_ve2ve(relative_pm, relative_vs, to_pe_321, to_ve_321, result, result2, "321", "313");
	EXPECT_TRUE(s_is_equal(6, from_ve_313, result, error) && s_is_equal(3, from_pe_313, result2, error)) << "\"s_inv_ve2ve 313\" failed";

	s_ve2ve(relative_pm, relative_vs, from_pe_321, from_ve_321, result, result2, "321", "313");
	EXPECT_TRUE(s_is_equal(6, to_ve_313, result, error) && s_is_equal(3, to_pe_313, result2, error)) << "\"s_ve2ve 313 to 321\" failed";

	s_inv_ve2ve(relative_pm, relative_vs, to_pe_313, to_ve_313, result, result2, "313", "321");
	EXPECT_TRUE(s_is_equal(6, from_ve_321, result, error) && s_is_equal(3, from_pe_321, result2, error)) << "\"s_inv_ve2ve 313\" failed";

	s_vq2vq(relative_pm, relative_vs, from_pq, from_vq, result, result2);
	EXPECT_TRUE(s_is_equal(7, to_vq, result, error) && s_is_equal(7, to_pq, result2, error)) << "\"s_vq2vq\" failed";

	s_inv_vq2vq(relative_pm, relative_vs, to_pq, to_vq, result, result2);
	EXPECT_TRUE(s_is_equal(7, from_vq, result, error) && s_is_equal(7, from_pq, result2, error)) << "\"s_inv_vq2vq\" failed";

	s_vm2vm(relative_pm, relative_vs, from_pm, from_vm, result, result2);
	EXPECT_TRUE(s_is_equal(16, to_vm, result, error) && s_is_equal(16, to_pm, result2, error)) << "\"s_vm2vm\" failed";

	s_inv_vm2vm(relative_pm, relative_vs, to_pm, to_vm, result, result2);
	EXPECT_TRUE(s_is_equal(16, from_vm, result, error) && s_is_equal(16, from_pm, result2, error)) << "\"s_inv_vm2vm\" failed";

	s_va2va(relative_pm, relative_vs, from_pp, from_va, result, result2);
	EXPECT_TRUE(s_is_equal(6, to_va, result, error) && s_is_equal(3, to_pp, result2, error)) << "\"s_va2va\" failed";

	s_inv_va2va(relative_pm, relative_vs, to_pp, to_va, result, result2);
	EXPECT_TRUE(s_is_equal(6, from_va, result, error) && s_is_equal(3, from_pp, result2, error)) << "\"s_inv_va2va\" failed";

	s_vs2vs(relative_pm, relative_vs, from_vs, result);
	EXPECT_TRUE(s_is_equal(6, to_vs, result, error)) << "\"s_vs2vs\" failed";

	s_inv_vs2vs(relative_pm, relative_vs, to_vs, result);
	EXPECT_TRUE(s_is_equal(6, from_vs, result, error)) << "\"s_inv_vs2vs\" failed";



	s_ap2ap(relative_pm, relative_vs, relative_as, from_pp, from_vp, from_ap, result, result2, result3);
	EXPECT_TRUE(s_is_equal(3, to_ap, result, error) && s_is_equal(3, to_vp, result2, error) && s_is_equal(3, to_pp, result3, error)) << "\"s_ap2ap\" failed";

	s_inv_ap2ap(relative_pm, relative_vs, relative_as, to_pp, to_vp, to_ap, result, result2, result3);
	EXPECT_TRUE(s_is_equal(3, from_ap, result, error) && s_is_equal(3, from_vp, result2, error) && s_is_equal(3, from_pp, result3, error)) << "\"s_inv_ap2ap\" failed";

	s_xe2xe(relative_pm, relative_vs, relative_as, from_re_313, from_we_313, from_xe_313, result, result2, result3, "313", "321");
	EXPECT_TRUE(s_is_equal(3, to_xe_321, result, error) && s_is_equal(3, to_we_321, result2, error) && s_is_equal(3, to_re_321, result3, error)) << "\"s_xe2xe 313 to 321\" failed";

	s_inv_xe2xe(relative_pm, relative_vs, relative_as, to_re_321, to_we_321, to_xe_321, result, result2, result3, "321", "313");
	EXPECT_TRUE(s_is_equal(3, from_xe_313, result, error) && s_is_equal(3, from_we_313, result2, error) && s_is_equal(3, from_re_313, result3, error)) << "\"s_inv_xe2xe 321 to 313\" failed";
	
	s_xe2xe(relative_pm, relative_vs, relative_as, from_re_321, from_we_321, from_xe_321, result, result2, result3, "321", "313");
	EXPECT_TRUE(s_is_equal(3, to_xe_313, result, error) && s_is_equal(3, to_we_313, result2, error) && s_is_equal(3, to_re_313, result3, error)) << "\"s_xe2xe 321 to 313\" failed";

	s_inv_xe2xe(relative_pm, relative_vs, relative_as, to_re_313, to_we_313, to_xe_313, result, result2, result3, "313", "321");
	EXPECT_TRUE(s_is_equal(3, from_xe_321, result, error) && s_is_equal(3, from_we_321, result2, error) && s_is_equal(3, from_re_321, result3, error)) << "\"s_inv_xe2xe 313 to 321\" failed";

	s_xq2xq(relative_pm, relative_vs, relative_as, from_rq, from_wq, from_xq, result, result2, result3);
	EXPECT_TRUE(s_is_equal(4, to_xq, result, error) && s_is_equal(4, to_wq, result2, error) && s_is_equal(4, to_rq, result3, error)) << "\"s_xq2xq\" failed";

	s_inv_xq2xq(relative_pm, relative_vs, relative_as, to_rq, to_wq, to_xq, result, result2, result3);
	EXPECT_TRUE(s_is_equal(4, from_xq, result, error) && s_is_equal(4, from_wq, result2, error) && s_is_equal(4, from_rq, result3, error)) << "\"s_inv_xq2xq\" failed";

	s_xm2xm(relative_pm, relative_vs, relative_as, from_rm, from_wm, from_xm, result, result2, result3);
	EXPECT_TRUE(s_is_equal(9, to_xm, result, error) && s_is_equal(9, to_wm, result2, error) && s_is_equal(9, to_rm, result3, error)) << "\"s_xm2xm\" failed";

	s_inv_xm2xm(relative_pm, relative_vs, relative_as, to_rm, to_wm, to_xm, result, result2, result3);
	EXPECT_TRUE(s_is_equal(9, from_xm, result, error) && s_is_equal(9, from_wm, result2, error) && s_is_equal(9, from_rm, result3, error)) << "\"s_inv_xm2xm\" failed";

	s_xa2xa(relative_pm, relative_vs, relative_as, from_wa, from_xa, result, result2);
	EXPECT_TRUE(s_is_equal(3, to_xa, result, error)) << "\"s_xa2xa\" failed";

	s_inv_xa2xa(relative_pm, relative_vs, relative_as, to_wa, to_xa, result, result2);
	EXPECT_TRUE(s_is_equal(3, from_xa, result, error)) << "\"s_inv_xa2xa\" failed";
	
	s_ae2ae(relative_pm, relative_vs, relative_as, from_pe_313, from_ve_313, from_ae_313, result, result2, result3, "313", "321");
	EXPECT_TRUE(s_is_equal(6, to_ae_321, result, error) && s_is_equal(6, to_ve_321, result2, error) && s_is_equal(6, to_pe_321, result3, error)) << "\"s_ae2ae 313 to 321\" failed";

	s_inv_ae2ae(relative_pm, relative_vs, relative_as, to_pe_321, to_ve_321, to_ae_321, result, result2, result3, "321", "313");
	EXPECT_TRUE(s_is_equal(6, from_ae_313, result, error) && s_is_equal(6, from_ve_313, result2, error) && s_is_equal(6, from_pe_313, result3, error)) << "\"s_inv_ae2ae 321 to 313\" failed";
	
	s_ae2ae(relative_pm, relative_vs, relative_as, from_pe_321, from_ve_321, from_ae_321, result, result2, result3, "321", "313");
	EXPECT_TRUE(s_is_equal(6, to_ae_313, result, error) && s_is_equal(6, to_ve_313, result2, error) && s_is_equal(6, to_pe_313, result3, error)) << "\"s_ae2ae 321 to 313\" failed";

	s_inv_ae2ae(relative_pm, relative_vs, relative_as, to_pe_313, to_ve_313, to_ae_313, result, result2, result3, "313", "321");
	EXPECT_TRUE(s_is_equal(6, from_ae_321, result, error) && s_is_equal(6, from_ve_321, result2, error) && s_is_equal(6, from_pe_321, result3, error)) << "\"s_inv_ae2ae 313 to 321\" failed";

	s_aq2aq(relative_pm, relative_vs, relative_as, from_pq, from_vq, from_aq, result, result2, result3);
	EXPECT_TRUE(s_is_equal(7, to_aq, result, error) && s_is_equal(7, to_vq, result2, error) && s_is_equal(7, to_pq, result3, error)) << "\"s_aq2aq\" failed";

	s_inv_aq2aq(relative_pm, relative_vs, relative_as, to_pq, to_vq, to_aq, result, result2, result3);
	EXPECT_TRUE(s_is_equal(7, from_aq, result, error) && s_is_equal(7, from_vq, result2, error) && s_is_equal(7, from_pq, result3, error)) << "\"s_inv_aq2aq\" failed";

	s_am2am(relative_pm, relative_vs, relative_as, from_pm, from_vm, from_am, result, result2, result3);
	EXPECT_TRUE(s_is_equal(16, to_am, result, error) && s_is_equal(16, to_vm, result2, error) && s_is_equal(16, to_pm, result3, error)) << "\"s_am2am\" failed";

	s_inv_am2am(relative_pm, relative_vs, relative_as, to_pm, to_vm, to_am, result, result2, result3);
	EXPECT_TRUE(s_is_equal(16, from_am, result, error) && s_is_equal(16, from_vm, result2, error) && s_is_equal(16, from_pm, result3, error)) << "\"s_inv_am2am\" failed";

	s_aa2aa(relative_pm, relative_vs, relative_as, from_pp, from_va, from_aa, result, result2, result3);
	EXPECT_TRUE(s_is_equal(6, to_aa, result, error) && s_is_equal(6, to_va, result2, error) && s_is_equal(3, to_pp, result3, error)) << "\"s_aa2aa\" failed";

	s_inv_aa2aa(relative_pm, relative_vs, relative_as, to_pp, to_va, to_aa, result, result2, result3);
	EXPECT_TRUE(s_is_equal(6, from_aa, result, error) && s_is_equal(6, from_va, result2, error) && s_is_equal(3, from_pp, result3, error)) << "\"s_inv_aa2aa\" failed";

	s_as2as(relative_pm, relative_vs, relative_as, from_vs, from_as, result, result2);
	EXPECT_TRUE(s_is_equal(6, to_as, result, error) && s_is_equal(6, to_vs, result2, error)) << "\"s_as2as\" failed";

	s_inv_as2as(relative_pm, relative_vs, relative_as, to_vs, to_as, result, result2);
	EXPECT_TRUE(s_is_equal(6, from_as, result, error) && s_is_equal(6, from_vs, result2, error)) << "\"s_inv_as2as\" failed";

}
TEST(DynamicPoseTest, Solve) {
	double result[36], result2[36], result3[36];
	
	const double pm[16]{ -0.22, -0.975499782797526,   0.000416847668728071, 0.1,
		0.175499782797526, -0.04, -0.983666521865018, 0.2,
		0.959583152331272, -0.216333478134982,   0.18, 0.3,
		0,0,0,1 };
	const double origin[]{ 0.1,0.2,0.3 };
	const double first_pnt[]{ -0.12,0.375499782797526, 1.259583152331272 };
	const double second_pnt[]{ -0.875499782797526,0.16, 0.083666521865018 };
	const double origin_1[]{ 0.1,0,0.2,0,0.3,0 };
	const double first_pnt_1[]{ -0.12,0,0,0.375499782797526,0,0, 1.259583152331272,0,0 };
	const double second_pnt_1[]{ -0.875499782797526,0,0,0,0.16,0,0,0, 0.083666521865018,0,0,0 };


	const double pm_2[]{ -0.220000000000000, -0.974711186851828, -0.039218646405775,0.100000000000000,
		0.175499782797526,   0.000000000000000, -0.984479469688434,0.200000000000000,
		0.959583152331272, -0.223468347257282,   0.171061601582815,0.300000000000000,
		0.000000000000000,0.000000000000000,0.000000000000000,1.000000000000000 };
	const double origin_2[]{ 0.1,0.2,0.3 };
	const double first_pnt_2[]{ -0.12,0.375499782797526, 1.259583152331272 };
	const double second_pnt_2[]{ -0.12,0.375499782797526, 1.259583152331272 };
	const double pm_3[]{ -0.974711186851828, - 0.220000000000000,0.039218646405775,0.100000000000000,
		0.000000000000000,0.175499782797526,0.984479469688434,0.200000000000000,
		- 0.223468347257282,0.959583152331272, - 0.171061601582815,0.300000000000000,
		0.000000000000000,0.000000000000000,0.000000000000000,1.000000000000000 };
	const double origin_3[]{ 0.1,0,0.2,0,0.3,0 };
	const double first_pnt_3[]{ -0.12,0,0,0.375499782797526,0,0, 1.259583152331272,0,0 };
	const double second_pnt_3[]{ -0.12,0,0,0,0.375499782797526,0,0,0, 1.259583152331272,0,0,0 };

	// Degenerate cases (first and second points overlap) are under-constrained and can have multiple valid orientations.
	// Validate geometric invariants instead of binding to one specific matrix branch.
	auto check_degenerate_pnts2pm = [&](const double *pm_now, const double *origin_now, const double *axis_now, int axis_id)->::testing::AssertionResult {
		double x[3]{ pm_now[0], pm_now[4], pm_now[8] };
		double y[3]{ pm_now[1], pm_now[5], pm_now[9] };
		double z[3]{ pm_now[2], pm_now[6], pm_now[10] };

		double axis_norm = std::sqrt(axis_now[0] * axis_now[0] + axis_now[1] * axis_now[1] + axis_now[2] * axis_now[2]);
		double axis_unit[3]{ axis_now[0] / axis_norm, axis_now[1] / axis_norm, axis_now[2] / axis_norm };
		const double *main_axis = (axis_id == 0) ? x : y;
		double axis_dot = main_axis[0] * axis_unit[0] + main_axis[1] * axis_unit[1] + main_axis[2] * axis_unit[2];

		double main_axis_norm = std::sqrt(main_axis[0] * main_axis[0] + main_axis[1] * main_axis[1] + main_axis[2] * main_axis[2]);
		double z_norm = std::sqrt(z[0] * z[0] + z[1] * z[1] + z[2] * z[2]);

		bool tr_ok = std::abs(pm_now[3] - origin_now[0]) <= error && std::abs(pm_now[7] - origin_now[1]) <= error && std::abs(pm_now[11] - origin_now[2]) <= error;
		bool last_row_ok = std::abs(pm_now[12]) <= error && std::abs(pm_now[13]) <= error && std::abs(pm_now[14]) <= error && std::abs(pm_now[15] - 1.0) <= error;
		bool axis_ok = std::abs(axis_dot - 1.0) <= 1e-10 && std::abs(main_axis_norm - 1.0) <= 1e-10;
		bool z_ok = z_norm > 1e-12;
		bool finite_ok = std::isfinite(x[0]) && std::isfinite(x[1]) && std::isfinite(x[2])
			&& std::isfinite(y[0]) && std::isfinite(y[1]) && std::isfinite(y[2])
			&& std::isfinite(z[0]) && std::isfinite(z[1]) && std::isfinite(z[2]);

		if (!(tr_ok && last_row_ok && axis_ok && z_ok && finite_ok)) {
			return ::testing::AssertionFailure()
				<< "degenerate pnts2pm constraints violated"
				<< " tr_ok=" << tr_ok
				<< " last_row_ok=" << last_row_ok
				<< " axis_ok=" << axis_ok << " axis_dot=" << axis_dot << " main_axis_norm=" << main_axis_norm
				<< " z_ok=" << z_ok << " z_norm=" << z_norm
				<< " finite_ok=" << finite_ok;
		}
		return ::testing::AssertionSuccess();
	};

	const double deg_axis_dense[]{ first_pnt_2[0] - origin[0], first_pnt_2[1] - origin[1], first_pnt_2[2] - origin[2] };
	const double deg_axis_ld[]{ first_pnt_3[0] - origin_3[0], first_pnt_3[3] - origin_3[2], first_pnt_3[6] - origin_3[4] };
	const double origin_ld[]{ origin_3[0], origin_3[2], origin_3[4] };

	s_sov_pnts2pm(origin, first_pnt, second_pnt, result, "xy");
	EXPECT_TRUE(s_is_equal(16, pm, result, error)) << "\"s_sov_pnts2pm\" failed";

	s_sov_pnts2pm(origin, second_pnt, first_pnt, result, "yx");
	EXPECT_TRUE(s_is_equal(16, pm, result, error)) << "\"s_sov_pnts2pm\" failed";

	s_sov_pnts2pm(origin_1, 2, first_pnt_1, 3, second_pnt_1, 4, result, "xy");
	EXPECT_TRUE(s_is_equal(16, pm, result, error)) << "\"s_sov_pnts2pm\" failed";

	s_sov_pnts2pm(origin_1, 2, second_pnt_1, 4, first_pnt_1, 3, result, "yx");
	EXPECT_TRUE(s_is_equal(16, pm, result, error)) << "\"s_sov_pnts2pm\" failed";

	s_sov_pnts2pm(origin, first_pnt_2, second_pnt_2, result, "xy");
	EXPECT_TRUE(check_degenerate_pnts2pm(result, origin, deg_axis_dense, 0)) << "\"s_sov_pnts2pm\" failed";

	s_sov_pnts2pm(origin, second_pnt_2, first_pnt_2, result, "yx");
	EXPECT_TRUE(check_degenerate_pnts2pm(result, origin, deg_axis_dense, 1)) << "\"s_sov_pnts2pm\" failed";

	s_sov_pnts2pm(origin_1, 2, first_pnt_3, 3, second_pnt_3, 4, result, "xy");
	EXPECT_TRUE(check_degenerate_pnts2pm(result, origin_ld, deg_axis_ld, 0)) << "\"s_sov_pnts2pm\" failed";

	s_sov_pnts2pm(origin_1, 2, second_pnt_3, 4, first_pnt_3, 3, result, "yx");
	EXPECT_TRUE(check_degenerate_pnts2pm(result, origin_ld, deg_axis_ld, 1)) << "\"s_sov_pnts2pm\" failed";


	const double ab[2] = { 2.46823966120654, -1.28551725555848 };
	const double vab[2] = { 5.886, -2.65 };
	const double aab[2] = { -1.234, 3.875 };
	const double pp31[3]{ -0.175499782797524,-0.220000000000002, -0.959583152331272 };
	const double vp31[3]{ 2.88069659757518,   0.954878838208465, - 0.745776923914583 };
	const double ap31[3]{ -18.6788800017547,   25.144386102382,   7.82919554768749 };
	
	s_sov_ab(pp31, result, "31");
	EXPECT_TRUE(s_is_equal(2, ab, result, error)) << "\"s_sov_ab\" failed";

	s_sov_vab(pp31, vp31, result, result2, "31");
	EXPECT_TRUE(s_is_equal(2, vab, result, error) && s_is_equal(2, ab, result2, error)) << "\"s_sov_vab\" failed";

	s_sov_aab(pp31, vp31, ap31, result, result2, result3, "31");
	EXPECT_TRUE(s_is_equal(2, aab, result, error) && s_is_equal(2, vab, result2, error) && s_is_equal(2, ab, result3, error)) << "\"s_sov_aab\" failed";

	const double from_pm[]{ -0.22, -0.975499782797526,   0.000416847668728071, 0.1,
		0.175499782797526, -0.04, -0.983666521865018, 0.2,
		0.959583152331272, -0.216333478134982,   0.18, 0.3,
		0,0,0,1 };
	const double to_pm[]{ -0.845157467344504, - 0.534513733550073, - 0.00198091852032335, - 0.781284637271869,
		0.403251168533375, - 0.635167604402343, - 0.658749276579685, - 0.752794893570788,
		0.350852320026947, - 0.5575456779172,0.752359931527164,0.428527346872256,
		0,0,0,1 };
	const double axis_distance[]{ 0.15,0.87,0.96,0.42617682954868,0.599370266248722,0.785593367378141 };


	for (int i = 0; i < 6; ++i) result[i]=s_sov_axis_distance(from_pm, to_pm, i);
	EXPECT_TRUE(s_is_equal(6, axis_distance, result, error)) << "\"s_sov_axis_distance\" failed";
	
	double calib_tool_two_pnts[6]{ 0.5,0.4,0.3,0.6,0.4,0.9 };
	const double calib_tool_two_pnts_result[2]{ 0.09553364891256,   0.13964114742507 };
	s_calib_tool_two_pnts(calib_tool_two_pnts, result);
	EXPECT_TRUE(s_is_equal(2, calib_tool_two_pnts_result, result, error)) << "\"s_sov_axis_distance\" failed";
}
TEST(DynamicPoseTest, Collision) {
	
	{
		double reference_pe[6]{ 0.1,0.2,0.3,-aris::PI/2,0,0 };
		double reference_pm[16];
		s_pe2pm(reference_pe, reference_pm, "321");

		double eul[3]{ aris::PI/2,0,0 };
		double point1_xyz[3]{1,2,3};
		double point2_xyz[3]{-0.5,0.6,0.2};

		double box_center[3], box_eul[3], box_length_xyz[3];
		s_generate_box(reference_pm, eul, point1_xyz, point2_xyz, box_center, box_eul, box_length_xyz);
	}

	//#define DEBUG_COLLIDE_CHECK_BOX2BOX
	
	// test box & box // 
	auto test_boxes_collide = [&](const double* box1_center, const double* box1_321_eul, const double* box1_length_xyz,
		const double* box2_center, const double* box2_321_eul, const double* box2_length_xyz,
		int result_should_be)
	{
		auto result = aris::dynamic::s_collide_check_box2box(box1_center, box1_321_eul, box1_length_xyz, box2_center, box2_321_eul, box2_length_xyz);
		EXPECT_EQ(result, result_should_be) << "\"s_collide_check_box2box\" failed";
	};

	auto test_boxes_devide    = [](auto test_collide_func)->void {
		// 测试A B完全分离
		{
			//// A B 分离，且仅符合叉乘判断
			{
				const double box1_center[3]{ 0.1,0.2,0.3 };
				const double box1_eur[3]{ 0.3,0.1,0.2 };
				const double box1_length[3]{ 0.5,0.2,0.8 };
				const double box2_center[3]{ 0.3,0.4,0.87 };
				const double box2_eur[3]{ 0.1,0.8,0.7 };
				const double box2_length[3]{ 0.3,0.3,0.3 };
				test_collide_func(box1_center, box1_eur, box1_length, box2_center, box2_eur, box2_length, 0);
			}

			//// A B 分离，且仅符合OBB2方向判断
			{
				const double box1_center[3]{ 0.1,0.2,0.3 };
				const double box1_eur[3]{ 0.3,0.1,0.2 };
				const double box1_length[3]{ 0.5,0.2,0.8 };
				const double box2_center[3]{ 0.3,0.4,0.95 };
				const double box2_eur[3]{ 0.1,0.8,0.7 };
				const double box2_length[3]{ 0.3,0.3,0.3 };
				test_collide_func(box1_center, box1_eur, box1_length, box2_center, box2_eur, box2_length, 0);
			}

			//// A B 分离，且仅符合OBB1方向判断
			{
				const double box1_center[3]{ 0.1,0.2,0.3 };
				const double box1_eur[3]{ 0.3,0.1,0.2 };
				const double box1_length[3]{ 0.5,0.2,0.8 };
				const double box2_center[3]{ 0.3,0.4,1.0 };
				const double box2_eur[3]{ 0.1,0.8,0.7 };
				const double box2_length[3]{ 0.3,0.3,0.3 };
				test_collide_func(box1_center, box1_eur, box1_length, box2_center, box2_eur, box2_length, 0);
			}
		}
	};
	auto test_boxes_interfere = [](auto test_collide_func) {
		// 测试有干涉无包含
		{
			// 各干涉一个点
			{
				const double box1_center[3]{ 0.1,0.2,0.3 };
				const double box1_eur[3]{ 0.3,0.1,0.2 };
				const double box1_length[3]{ 0.5,0.2,0.8 };
				const double box2_center[3]{ 0.3,0.4,0.6 };
				const double box2_eur[3]{ 0.1,0.1,0.5 };
				const double box2_length[3]{ 0.3,0.3,0.3 };
				test_collide_func(box1_center, box1_eur, box1_length, box2_center, box2_eur, box2_length, 1);
			}

			// A中仅仅1个点被B包含
			{
				const double box1_center[3]{ 0.1,0.2,0.3 };
				const double box1_eur[3]{ 0.3,0.1,0.2 };
				const double box1_length[3]{ 0.5,0.2,0.8 };
				const double box2_center[3]{ 0.3,0.4,0.65 };
				const double box2_eur[3]{ 0.1,0.1,0.5 };
				const double box2_length[3]{ 0.3,0.3,0.3 };
				test_collide_func(box1_center, box1_eur, box1_length, box2_center, box2_eur, box2_length, 1);
			}

			// A B 干涉，且无任何点被对方包含
			{
				const double box1_center[3]{ 0.1,0.2,0.3 };
				const double box1_eur[3]{ 0.3,0.1,0.2 };
				const double box1_length[3]{ 0.5,0.2,0.8 };
				const double box2_center[3]{ 0.3,0.4,0.86 };
				const double box2_eur[3]{ 0.1,0.8,0.7 };
				const double box2_length[3]{ 0.3,0.3,0.3 };
				test_collide_func(box1_center, box1_eur, box1_length, box2_center, box2_eur, box2_length, 1);
			}
		}
	};
	auto test_boxes_contain   = [](auto test_collide_func) {
		// 测试A包含B、B包含A
		{
			// A包含B，B包含A
			{
				const double box1_center[3]{ 0.1,0.2,0.3 };
				const double box1_eur[3]{ 0.3,0.1,0.2 };
				const double box1_length[3]{ 0.5,0.4,0.8 };
				const double box2_center[3]{ 0.1,0.2,0.25 };
				const double box2_eur[3]{ 0.1,0.1,0.5 };
				const double box2_length[3]{ 0.1,0.15,0.2 };
				// A 包含 B
				test_collide_func(box1_center, box1_eur, box1_length, box2_center, box2_eur, box2_length, 2);
				// B 包含 A
				test_collide_func(box2_center, box2_eur, box2_length, box1_center, box1_eur, box1_length, 3);
			}

			// A中仅仅1个点被B包含
			{
				//const double box1_center[3]{ 0.1,0.2,0.3 };
				//const double box1_eur[3]{ 0.3,0.1,0.2 };
				//const double box1_length[3]{ 0.5,0.2,0.8 };
				//const double box2_center[3]{ 0.3,0.4,0.65 };
				//const double box2_eur[3]{ 0.1,0.1,0.5 };
				//const double box2_length[3]{ 0.3,0.3,0.3 };
				//test_collide(box1_center, box1_eur, box1_length, box2_center, box2_eur, box2_length, 1);
			}

			// A B 干涉，且无任何点被对方包含
			{
				//const double box1_center[3]{ 0.1,0.2,0.3 };
				//const double box1_eur[3]{ 0.3,0.1,0.2 };
				//const double box1_length[3]{ 0.5,0.2,0.8 };
				//const double box2_center[3]{ 0.3,0.4,0.86 };
				//const double box2_eur[3]{ 0.1,0.8,0.7 };
				//const double box2_length[3]{ 0.3,0.3,0.3 };
				//test_collide(box1_center, box1_eur, box1_length, box2_center, box2_eur, box2_length, 1);
			}
		}
	};
	
	test_boxes_devide(test_boxes_collide);
	test_boxes_interfere(test_boxes_collide);
	test_boxes_contain(test_boxes_collide);


	// test sphere & box // 
	auto test_sphere_box_collide = [](const double* sphere1_center, double radius,
		const double* box2_center, const double* box2_321_eul, const double* box2_length_xyz,
		int result_should_be) 
	{
		auto result = aris::dynamic::s_collide_check_sphere2box(sphere1_center, radius, box2_center, box2_321_eul, box2_length_xyz);
		EXPECT_EQ(result, result_should_be) << "\"s_collide_check_sphere2box\" failed";
	};
	auto test_sphere_box_devide = [](auto test_collide_func) {
		// 测试A B完全分离
		{
			//// A B 分离，且仅符合叉乘判断
			{
				double radius{ 0.1 };
				const double sphere_center[3]{ 0.1,0.2,0.3 };
				const double box2_center[3]{ 0.3,0.4,0.87 };
				const double box2_eur[3]{ 0.1,0.8,0.7 };
				const double box2_length[3]{ 0.3,0.4,0.5 };
				test_collide_func(sphere_center, radius, box2_center, box2_eur, box2_length, 0);
			}

			//// A B 分离，且仅符合叉乘判断
			{
				double radius{ 0.1 };
				const double sphere_center[3]{ -0.05,0.3,0.9 };
				const double box2_center[3]{ 0.3,0.4,0.87 };
				const double box2_eur[3]{ 0.1,0.8,0.7 };
				const double box2_length[3]{ 0.3,0.4,0.5 };
				test_collide_func(sphere_center, radius, box2_center, box2_eur, box2_length, 0);
			}

			//// A B 分离，且仅符合叉乘判断
			{
				double radius{ 0.1 };
				const double sphere_center[3]{ 0.2, 0.4, 1.13 };
				const double box2_center[3]{ 0.3,0.4,0.87 };
				const double box2_eur[3]{ 0.1,0.8,0.7 };
				const double box2_length[3]{ 0.3,0.4,0.5 };
				test_collide_func(sphere_center, radius, box2_center, box2_eur, box2_length, 0);
			}

			//// A B 碰撞，球面仅仅包含一个面
			{
				double radius{ 0.1 };
				const double sphere_center[3]{ 0.2, 0.4, 1.1 };
				const double box2_center[3]{ 0.3,0.4,0.87 };
				const double box2_eur[3]{ 0.1,0.8,0.7 };
				const double box2_length[3]{ 0.3,0.4,0.5 };
				test_collide_func(sphere_center, radius, box2_center, box2_eur, box2_length, 1);
			}

			//// A B 碰撞，球面仅仅包含一条边
			{
				double radius{ 0.1 };
				const double sphere_center[3]{ -0.01,0.3,0.9 };
				const double box2_center[3]{ 0.3,0.4,0.87 };
				const double box2_eur[3]{ 0.1,0.8,0.7 };
				const double box2_length[3]{ 0.3,0.4,0.5 };
				test_collide_func(sphere_center, radius, box2_center, box2_eur, box2_length, 1);
			}
			
			//// A B 碰撞，球面仅仅包含一个点
			{
				double radius{ 0.1 };
				const double sphere_center[3]{ -0.05,0.35,0.7 };
				const double box2_center[3]{ 0.3,0.4,0.87 };
				const double box2_eur[3]{ 0.1,0.8,0.7 };
				const double box2_length[3]{ 0.3,0.4,0.5 };
				test_collide_func(sphere_center, radius, box2_center, box2_eur, box2_length, 1);
			}

			//// A B 碰撞，球面仅仅包含一个点
			{
				double radius{ 0.2 };
				const double sphere_center[3]{ -0.05,0.35,0.7 };
				const double box2_center[3]{ 0.3,0.4,0.87 };
				const double box2_eur[3]{ 0.1,0.8,0.7 };
				const double box2_length[3]{ 0.3,0.4,0.5 };
				test_collide_func(sphere_center, radius, box2_center, box2_eur, box2_length, 1);
			}

			//// A B 碰撞，球面仅仅包含一堆点
			{
				double radius{ 0.7 };
				const double sphere_center[3]{ -0.05,0.35,0.7 };
				const double box2_center[3]{ 0.3,0.4,0.87 };
				const double box2_eur[3]{ 0.1,0.8,0.7 };
				const double box2_length[3]{ 0.3,0.4,0.5 };
				test_collide_func(sphere_center, radius, box2_center, box2_eur, box2_length, 1);
			}

			//// A B 碰撞，球面包含长方体
			{
				double radius{ 0.75 };
				const double sphere_center[3]{ -0.05,0.35,0.7 };
				const double box2_center[3]{ 0.3,0.4,0.87 };
				const double box2_eur[3]{ 0.1,0.8,0.7 };
				const double box2_length[3]{ 0.3,0.4,0.5 };
				test_collide_func(sphere_center, radius, box2_center, box2_eur, box2_length, 2);
			}

			//// A B 碰撞，长方体包含球面
			{
				double radius{ 0.1 };
				const double sphere_center[3]{ 0.29,0.4,0.8 };
				const double box2_center[3]{ 0.3,0.4,0.87 };
				const double box2_eur[3]{ 0.1,0.8,0.7 };
				const double box2_length[3]{ 0.3,0.4,0.5 };
				test_collide_func(sphere_center, radius, box2_center, box2_eur, box2_length, 3);
			}

			//// A B 碰撞，长方体包含球面
			{
				double radius{ 0.1 };
				const double sphere_center[3]{ 0.3,0.4,0.8 };
				const double box2_center[3]{ 0.3,0.4,0.87 };
				const double box2_eur[3]{ 0.1,0.8,0.7 };
				const double box2_length[3]{ 0.3,0.4,0.5 };
				test_collide_func(sphere_center, radius, box2_center, box2_eur, box2_length, 1);
			}
		}
	};
	test_sphere_box_devide(test_sphere_box_collide);

	// test sphere & sphere // 
	auto test_spheres_collide = [&](const double* sphere1_center, double sphere1_radius,
		const double* sphere2_center, double sphere2_radius,
		int result_should_be)
	{
		auto result = aris::dynamic::s_collide_check_sphere2sphere(sphere1_center, sphere1_radius, sphere2_center, sphere2_radius);
		EXPECT_EQ(result, result_should_be) << "\"s_collide_check_sphere2sphere\" failed";
	};
	auto test_spheres_devide = [](auto test_collide_func) {
		// 测试A B完全分离
		{
			//// A B 分离
			{
				double sphere1_radius{ 0.1 };
				const double sphere1_center[3]{ 0.1,0.2,0.3 };
				double sphere2_radius{ 0.3 };
				const double sphere2_center[3]{ 0.4,0.6,0.1 };
				test_collide_func(sphere1_center, sphere1_radius, sphere2_center, sphere2_radius, 0);
			}

			//// A B 分离
			{
				double sphere1_radius{ 0.1 };
				const double sphere1_center[3]{ 0.1,0.2,0.3 };
				double sphere2_radius{ 0.3 };
				const double sphere2_center[3]{ 0.4,0.45,0.2 };
				test_collide_func(sphere1_center, sphere1_radius, sphere2_center, sphere2_radius, 0);
			}

			//// A B 干涉
			{
				double sphere1_radius{ 0.1 };
				const double sphere1_center[3]{ 0.1,0.2,0.3 };
				double sphere2_radius{ 0.3 };
				const double sphere2_center[3]{ 0.4,0.44,0.2 };
				test_collide_func(sphere1_center, sphere1_radius, sphere2_center, sphere2_radius, 1);
			}

			//// A B 干涉
			{
				double sphere1_radius{ 0.1 };
				const double sphere1_center[3]{ 0.3,0.297,0.3 };
				double sphere2_radius{ 0.3 };
				const double sphere2_center[3]{ 0.4,0.44,0.2 };
				test_collide_func(sphere1_center, sphere1_radius, sphere2_center, sphere2_radius, 1);
			}

			//// B 包含 A
			{
				double sphere1_radius{ 0.1 };
				const double sphere1_center[3]{ 0.3,0.3,0.3 };
				double sphere2_radius{ 0.3 };
				const double sphere2_center[3]{ 0.4,0.44,0.2 };
				test_collide_func(sphere1_center, sphere1_radius, sphere2_center, sphere2_radius, 3);
			}

			//// A 包含 B
			{
				double sphere1_radius{ 0.1 };
				const double sphere1_center[3]{ 0.3,0.3,0.3 };
				double sphere2_radius{ 0.3 };
				const double sphere2_center[3]{ 0.4,0.44,0.2 };
				test_collide_func(sphere2_center, sphere2_radius, sphere1_center, sphere1_radius, 2);
			}
		}
	};

	test_spheres_devide(test_spheres_collide);

}
