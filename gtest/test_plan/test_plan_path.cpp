#include <gtest/gtest.h>

#include <array>
#include <cmath>

#include <aris/plan/plan.hpp>

namespace {

constexpr double kTol = 1e-10;

} // namespace

TEST(PathTest, BlendBezierKnownCases) {
	{
		double p0[3]{0.3, 0.8, -0.3};
		double p1[3]{-0.5, 0.3, -0.3};
		double p2[3]{0.3, -0.5, 0.2};
		double p[3], dp[3], d2p[3];
		aris::plan::s_bezier3_blend_line_line(0.56, p0, p1, p2, p, dp, d2p);
		const double p_ref[3]{-0.29136, 0.2020992, -0.212192};
		const double dp_ref[3]{0.288, -1.04304, 0.4704};
		const double d2p_ref[3]{4.8, -1.368, 1.68};
		EXPECT_TRUE(aris::dynamic::s_is_equal(3, p, p_ref, kTol));
		EXPECT_TRUE(aris::dynamic::s_is_equal(3, dp, dp_ref, kTol));
		EXPECT_TRUE(aris::dynamic::s_is_equal(3, d2p, d2p_ref, kTol));
	}

	{
		double theta = aris::PI / 3;
		double center[3]{-1.5, 0.8, 0.2};
		double ax[3]{0.224859506698758, 0.374765844497931, 0.899438026795034};
		double p0[3]{3.5, 0.7, -2.8};
		double p1[3]{-1.5, 2, -0.3};
		double p[3], dp[3], d2p[3];
		aris::plan::s_bezier3_blend_line_circle(0.38, p0, p1, center, ax, theta, p, dp, d2p);
		const double p_ref[3]{-0.381107335021726, 1.6946498923771, -0.879498288068363};
		const double dp_ref[3]{-6.33968881728938, 1.51881554657884, 3.01823239324783};
		const double d2p_ref[3]{15.5955561862872, -4.98042456129022, -8.4887121460342};
		EXPECT_TRUE(aris::dynamic::s_is_equal(3, p, p_ref, kTol));
		EXPECT_TRUE(aris::dynamic::s_is_equal(3, dp, dp_ref, kTol));
		EXPECT_TRUE(aris::dynamic::s_is_equal(3, d2p, d2p_ref, kTol));
	}

	{
		double p1[3]{0.8, 0.3, -1.5};
		double c1[3]{-1.5, 0.8, 0.2};
		double ax1[3]{-0.0385744143167205, 0.94331976829071, -0.329635904161066};
		double theta1 = aris::PI / 6;
		double c2[3]{1.2, 0.4, -0.3};
		double ax2[3]{-0.948683298050514, 0, 0.316227766016838};
		double theta2 = aris::PI * 2 / 3;
		double p[3], dp[3], d2p[3];
		aris::plan::s_bezier3_blend_circle_circle(0.68, p1, c1, ax1, theta1, c2, ax2, theta2, p, dp, d2p);
		const double p_ref[3]{0.933000643153169, -0.438965706137216, -1.15386150235178};
		const double dp_ref[3]{0.505784869512604, -2.86256037663394, 2.00103347181547};
		const double d2p_ref[3]{6.49008174398832, 0.025236537507917, 16.6711242011704};
		EXPECT_TRUE(aris::dynamic::s_is_equal(3, p, p_ref, kTol));
		EXPECT_TRUE(aris::dynamic::s_is_equal(3, dp, dp_ref, kTol));
		EXPECT_TRUE(aris::dynamic::s_is_equal(3, d2p, d2p_ref, kTol));
	}

	{
		double q0[4]{0.409512331537456, 0.283529229761645, 0.0437129979471227, 0.866025403784439};
		double q1[4]{0.0534299365105471, 0.0637953790354169, 0.0259136063121744, 0.996194698091746};
		double q2[4]{0.147477479964183, 0.082527864550064, 0.039915336441851, 0.984807753012208};
		double q[4], dq[4], d2q[4];
		aris::plan::s_bezier3_blend_quaternion(0.66, q0, q1, q2, q, dq, d2q);
		const double q_ref[4]{0.095189031999334, 0.0783817543850458, 0.0307778043574334, 0.991891161128824};
		const double dq_ref[4]{-0.00596189256812635, -0.0559156109598799, 0.0112331920023273, 0.00464218017226164};
		const double d2q_ref[4]{1.13118014727655, 0.547101513143992, 0.0971962711225237, -0.158142392813288};
		EXPECT_TRUE(aris::dynamic::s_is_equal(4, q, q_ref, kTol));
		EXPECT_TRUE(aris::dynamic::s_is_equal(4, dq, dq_ref, kTol));
		EXPECT_TRUE(aris::dynamic::s_is_equal(4, d2q, d2q_ref, kTol));
	}

	{
		double q0[4]{std::sin(0.1), 0.0, 0.0, std::cos(0.1)};
		double q1[4]{0, 0, 0, 1};
		double q2[4]{std::sin(0.1), 0.0, 0.0, std::cos(0.1)};
		double q[4], dq[4], d2q[4];
		aris::plan::s_bezier3_blend_quaternion(0.5, q0, q1, q2, q, dq, d2q);
		const double q_ref[4]{0.024997395914712336, 0.0, 0.0, 0.99968751627570263};
		const double dq_ref[4]{0, 0, 0, 0};
		const double d2q_ref[4]{0.59981250976542155, 0, 0, -0.0149984375488274};
		EXPECT_TRUE(aris::dynamic::s_is_equal(4, q, q_ref, kTol));
		EXPECT_TRUE(aris::dynamic::s_is_equal(4, dq, dq_ref, kTol));
		EXPECT_TRUE(aris::dynamic::s_is_equal(4, d2q, d2q_ref, kTol));
	}
}

TEST(PathTest, ArcEstimateKnownTable) {
	double theta[12]{
		0, 0.285599332144527, 0.571198664289053, 0.85679799643358, 1.14239732857811,
		1.42799666072263, 1.71359599286716, 1.99919532501169, 2.28479465715621, 2.57039398930074,
		2.85599332144527, 3.14159265358979
	};
	double r = 2.0;
	const double result[12][12]{
		{0.0,0.0,-0.0,0.0,-0.0,0.0,3.8197186342054881,0.9549296585513720,0.0,1.5,0.0,-0.0},
		{0.0,0.0016408191865958,-0.0024612287798937,-0.0113416528701198,-0.0581789454140779,0.0290894727070390,3.8922054455093149,0.9721817285337183,0.0149475525453577,1.5383452525800176,-0.0695205982841978,-0.0049224575597874},
		{0.0,0.0252004408385808,-0.0378006612578712,-0.0276746761702011,-0.1668536799384117,0.0834268399692058,4.0076097175288785,0.9949556075928574,0.0416342138329016,1.5952926151248019,-0.1945283561086128,-0.0756013225157425},
		{0.0,0.1191211129727874,-0.1786816694591811,0.0621938936256773,-0.3049977392859476,0.1524988696429738,4.0914620306261051,1.0001294733799710,0.0745449272174413,1.6310365648161866,-0.2428038456602704,-0.3573633389183621},
		{0.0,0.3417396070235881,-0.5126094105353822,0.3558276004820765,-0.4488908172347860,0.2244454086173930,4.0558572513594999,0.9642823459190296,0.1106771736318679,1.6050412207879736,-0.0930632167527095,-1.0252188210707645},
		{0.0,0.7356238366461750,-1.1034357549692628,0.9291862951728079,-0.5627221693647922,0.2813610846823961,3.8139569451616455,0.8704635901052006,0.1475428740952771,1.4841063688844232,0.3664641258080157,-2.2068715099385257},
		{0.0,1.3048831897393214,-1.9573247846089821,1.7960859784118886,-0.6060970941617105,0.3030485470808553,3.3134507639906747,0.7174952860724076,0.1829202053486183,1.2574504387939851,1.1899888842501780,-3.9146495692179641},
		{0.0,2.0033996590311394,-3.0050994885467102,2.8826067892210996,-0.5522606494353610,0.2761303247176805,2.5707446124886935,0.5240466650014497,0.2148251704010097,0.9465310521910918,2.3303461397857386,-6.0101989770934203},
		{0.0,2.7385640487539189,-4.1078460731308786,4.0338836761437200,-0.4077861966998187,0.2038930983499094,1.6882591649279632,0.3235671282703929,0.2415424155077628,0.6025341043457653,3.6260974794439012,-8.2156921462617571},
		{0.0,3.3902145721633055,-5.0853218582449582,5.0525523144144584,-0.2198559745873882,0.1099279872936941,0.8401962920683771,0.1525188862999211,0.2616721552604708,0.2918211753077677,4.8326963398270699,-10.1706437164899164},
		{0.0,3.8396127136445881,-5.7594190704668824,5.7515081681098970,-0.0616914230485975,0.0308457115242988,0.2250063055000464,0.0393372429330973,0.2741764187963381,0.0767121484005984,5.6898167450612993,-11.5188381409337648},
		{0.0,4.0,-6.0,6.0,0.0,0.0,0.0,-0.0,0.2784163998415854,-0.0,6.0,-12.0}
	};

	for (int i = 0; i < 12; ++i) {
		double p0[3]{r, 0, 0};
		double p1[3]{0, 0, 0};
		double p2[3]{r * std::cos(theta[i]), r * std::sin(theta[i]), 0};
		double p[3], dp[3], d2p[3];
		double darc0, d2arc0, darc1, d2arc1, darc50, d2arc50;
		double ds_darc, d2s_darc2;

		aris::plan::s_bezier3_blend_line_line(0.0, p0, p1, p2, p, dp, d2p);
		aris::plan::s_bezier3_darc_ds(3, dp, d2p, darc0, d2arc0, ds_darc, d2s_darc2);
		aris::plan::s_bezier3_blend_line_line(1.0, p0, p1, p2, p, dp, d2p);
		aris::plan::s_bezier3_darc_ds(3, dp, d2p, darc1, d2arc1, ds_darc, d2s_darc2);
		aris::plan::s_bezier3_blend_line_line(0.5, p0, p1, p2, p, dp, d2p);
		aris::plan::s_bezier3_darc_ds(3, dp, d2p, darc50, d2arc50, ds_darc, d2s_darc2);

		aris::plan::EstimateBezierArcParam param;
		aris::plan::s_bezier3_estimate_arc_param(darc0, d2arc0, darc1, d2arc1, darc50, param);
		double param_result[12]{param.A, param.B, param.C, param.D, param.E, param.F, param.G, param.H, param.h, param.X, param.Y, param.Z};

		EXPECT_TRUE(aris::dynamic::s_is_equal(12, param_result, result[i], 1e-10)) << "Arc estimate case " << i << " failed";
	}
}
