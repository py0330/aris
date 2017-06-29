#include "test_dynamic_model.h"
#include <iostream>
#include <aris.h>

#include<type_traits>

using namespace aris::dynamic;

auto inline pm_hand_change(const double *lh_pm, double *rh_pm = nullptr)->double* 
{
	static double rh_pm_default[16];
	rh_pm = rh_pm ? rh_pm : rh_pm_default;

	s_vc(16, lh_pm, rh_pm);
	rh_pm[1] = -rh_pm[1];
	rh_pm[2] = -rh_pm[2];
	rh_pm[3] = -rh_pm[3];
	rh_pm[4] = -rh_pm[4];
	rh_pm[8] = -rh_pm[8];
	return rh_pm;
}
auto inline pp_hand_change(const double *lh_pp, double *rh_pp = nullptr)->double*
{
	static double rh_pp_default[3];
	rh_pp = rh_pp ? rh_pp : rh_pp_default;
	
	s_vc(3, lh_pp, rh_pp);
	rh_pp[0] = -rh_pp[0];
	return rh_pp;
}



void test_simple_model()
{
	std::cout << std::endl << "-----------------test simple model---------------------" << std::endl;

	const double body_ee_pm[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
	const double body_pm[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
	
	const double la_j1_pos[]{ 0.5, -0.8, 0.5 };
	const double la_j1_axis[]{ 1.072884E-06,-0.9999999,-2.682209E-07 };
	const double la_j2_pos[]{ 0.5824012,-1.192,0.382309 };
	const double la_j2_axis[]{ -0.9995906,-1.172337E-06,-0.02860951 };
	const double la_j3_pos[]{ 0.6181284,-1.192,-0.1668934 };
	const double la_j3_axis[]{ -0.9995906,-9.224335E-07,-0.02860951 };
	const double la_j4_pos[]{ 0.5183579,-1.256,-0.2287729 };
	const double la_j4_axis[]{ -0.0286096,-5.690381E-07,0.9995906 };
	const double la_j5_pos[]{ 0.6114721,-1.207999,-0.7033033 };
	const double la_j5_axis[]{ -0.9995906,-7.122022E-07,-0.02860987 };
	const double la_j6_pos[]{ 0.534779,-1.255999,-0.7850308 };
	const double la_j6_axis[]{ -0.02860966,-1.937587E-07,0.9995906 };
	const double la_ee_pm[]{ -0.999590754508972,-1.05657898075151E-06,0.0286096595227718,0.534778952598572,-1.04215007468156E-06,1,5.36441802978516E-07,-1.25599956512451,-0.0286096669733524,5.36368588655023E-07,-0.999590516090393,-0.785030841827393,0,0,0,1 };
	const double la_p1_pm[]{-0.9995908, -0.02860966, -1.075344E-06, 0.5000002, -1.043067E-06, -1.160307E-06, 1, -1.016, -0.02860967, 0.9995906, 1.159952E-06, 0.4999999, 0, 0, 0, 1, };
	const double la_p2_pm[]{-0.02860954, -9.025127E-07, -0.9995908, 0.6181284, -3.8743E-07, 1, -9.229408E-07, -1.192, 0.9995906, 3.618911E-07, -0.02860952, -0.1668934, 0, 0, 0, 1, };
	const double la_p3_pm[]{-0.02860966, 1.081253E-06, 0.9995908, 0.5673991, -3.278255E-07, -1, 1.04215E-06, -1.192, 0.9995906, -3.567753E-07, 0.02860964, -0.1683453, 0, 0, 0, 1, };
	const double la_p4_pm[]{0.0286099, 6.905713E-07, -0.9995908, 0.6114721, 2.384188E-07, -1, -6.845221E-07, -1.207999, -0.9995905, -2.486991E-07, -0.02860991, -0.7033032, 0, 0, 0, 1, };
	const double la_p5_pm[]{0.9995908, 1.056075E-06, 0.02861046, 0.5325189, 1.041646E-06, -1, 4.76838E-07, -1.256, 0.02861047, -5.36383E-07, -0.9995905, -0.7060632, 0, 0, 0, 1, };
	const double la_p6_pm[]{-0.999590754508972, -1.05657898075151E-06, 0.0286096595227718, 0.534778952598572, -1.04215007468156E-06, 1, 5.36441802978516E-07, -1.25599956512451, -0.0286096669733524, 5.36368588655023E-07, -0.999590516090393, -0.785030841827393, 0, 0, 0, 1, };

	const double ra_j1_pos[]{ 0.5, 0.8, 0.5 };
	const double ra_j1_axis[]{ 2.980232E-08, 0.9999999, -2.980232E-08 };
	const double ra_j2_pos[]{ 0.4150226, 1.192, 0.3841558 };
	const double ra_j2_axis[]{ 0.9987161, 1.27963E-07, -0.05065691 };
	const double ra_j3_pos[]{ 0.3671869, 1.192, -0.1641248 };
	const double ra_j3_axis[]{ 0.9987162, -1.227562E-07, -0.05065691 };
	const double ra_j4_pos[]{ 0.4655678, 1.256, -0.2281908 };
	const double ra_j4_axis[]{ 0.05065697, 3.129244E-07, 0.9987161 };
	const double ra_j5_pos[]{ 0.3620064, 1.208, -0.700551 };
	const double ra_j5_axis[]{ 0.9987161, -1.296035E-07, -0.05065727 };
	const double ra_j6_pos[]{ 0.4368778, 1.256, -0.7839509 };
	const double ra_j6_axis[]{ 0.05065703, -2.385077E-07, 0.9987161 };
	const double ra_ee_pm[]{ 0.998716115951538, 7.00018798482915E-09, -0.0506570376455784, 0.436877846717834, -5.33688870874016E-09, -1.00000011920929, -2.98023167033534E-07, 1.2559996843338, -0.0506570339202881, 2.98122614594831E-07, -0.998716235160828, -0.783950924873352, 0, 0, 0, 1, };
	const double ra_p1_pm[]{ 0.9987161, 0.05065704, 3.749908E-08, 0.5, -3.95994E-10, 8.026786E-07, -1, 1.016, -0.05065703, 0.9987163, 8.018798E-07, 0.4999999, 0, 0, 0, 1, };
	const double ra_p2_pm[]{ 0.05065692, -1.413697E-07, 0.9987161, 0.3671869, 2.682209E-07, -1, -1.210719E-07, 1.192, 0.9987163, 2.459671E-07, -0.05065688, -0.164125, 0, 0, 0, 1, };
	const double ra_p3_pm[]{ 0.05065668, -2.783465E-07, -0.9987161, 0.4178718, 2.682209E-07, 1, -2.365561E-07, 1.192, 0.9987163, -2.843594E-07, 0.05065668, -0.1666957, 0, 0, 0, 1, };
	const double ra_p4_pm[]{ -0.05065716, 1.413697E-07, 0.9987161, 0.3620064, -2.086163E-07, 1, -1.21072E-07, 1.208, -0.9987163, -2.459671E-07, -0.05065718, -0.700551, 0, 0, 0, 1, };
	const double ra_p5_pm[]{ -0.9987163, -1.411792E-09, -0.05065785, 0.4408797, 1.862837E-09, 1, -3.57628E-07, 1.256, 0.05065785, -1.193624E-07, -0.9987164, -0.7050525, 0, 0, 0, 1, };
	const double ra_p6_pm[]{ 0.998716115951538, 7.00018798482915E-09, -0.0506570376455784, 0.436877846717834, -5.33688870874016E-09, -1.00000011920929, -2.98023167033534E-07, 1.2559996843338, -0.0506570339202881, 2.98122614594831E-07, -0.998716235160828, -0.783950924873352, 0, 0, 0, 1, };
	//Target: {0.9747857, 4.169117E-08, -0.2231429, -0.7490435, -1.047021E-08, -1, -2.325747E-07, 0.9762796, -0.2231429, 2.290469E-07, -0.9747859, -0.5835092, 0, 0, 0, 1, }

	const double ll_j1_pos[]{ 0.5, -0.5, -1.052204 };
	const double ll_j1_axis[]{1.034124E-06, 1.202899E-07, -1};
	const double ll_j2_pos[]{ 0.7529517, -0.5974261, -1.644 };
	const double ll_j2_axis[]{-0.1155416, 0.9933027, -1.997687E-07};
	const double ll_j3_pos[]{ 0.7575741, -0.637158, -2.744 };
	const double ll_j3_axis[]{0.1155414, -0.9933026, -4.470348E-07};
	const double ll_j4_pos[]{ 0.6081899, -0.4607367, -2.862 };
	const double ll_j4_axis[]{ -1.275331E-06, 2.9751E-07, 1 };
	const double ll_j5_pos[]{ 0.7207637, -0.5976466, -3.8155 };
	const double ll_j5_axis[]{ 0.1155413, -0.9933027, -4.470348E-07 };
	const double ll_j6_pos[]{ 0.6084799, -0.4632197, -3.963706 };
	const double ll_j6_axis[]{ -1.831427E-06, -2.130324E-07, 1 };
	const double ll_ee_pm[]{ 0.115540817379951, -0.993302881717682, 1.03412457974628E-06, 0.608479976654053, -0.993302881717682, -0.115540876984596, 1.20289996630163E-07, -0.463219732046127, -7.29018245510665E-13, -1.04109722087742E-06, -1, -3.98429584503174, 0, 0, 0, 1, };
	const double ll_p1_pm[]{ -0.1155414, -0.9933028, -1.831428E-06, 0.5552197, 0.9933028, -0.1155415, -2.130326E-07, -0.5019535, -9.097994E-15, -1.843776E-06, 1, -1.367632, 0, 0, 0, 1, };
	const double ll_p2_pm[]{ -1.054785E-06, 0.9933028, -0.1155414, 0.7561938, 5.73265E-08, 0.1155415, 0.9933027, -0.625294, 0.9999999, 1.041097E-06, -1.788139E-07, -2.468108, 0, 0, 0, 1, };
	const double ll_p3_pm[]{ -1.075445E-06, -0.9933027, 0.1155422, 0.714063, 2.349429E-07, -0.1155422, -0.9933026, -0.4989428, 0.9999999, -1.041097E-06, 3.576287E-07, -2.741974, 0, 0, 0, 1, };
	const double ll_p4_pm[]{ 1.027238E-06, -0.9933029, -0.1155408, 0.6843457, 1.794953E-07, -0.1155409, 0.9933029, -0.5585557, -1, -1.041097E-06, 1.192099E-07, -3.547571, 0, 0, 0, 1, };
	const double ll_p5_pm[]{ 0.1155408, -0.9933029, 1.034125E-06, 0.6320234, -0.9933029, -0.1155409, 1.2029E-07, -0.4607983, -7.049992E-13, -1.041097E-06, -1, -3.815875, 0, 0, 0, 1, };
	const double ll_p6_pm[]{ -0.1155414, -0.9933028, -1.831428E-06, 0.6084799, 0.9933028, -0.1155415, -2.130326E-07, -0.4632197, -9.097994E-15, -1.843776E-06, 1, -3.963706, 0, 0, 0, 1, };
	//Target: {0.1155407, -0.9782426, -0.1723138, -0.7567107, -0.9933029, -0.1137892, -0.02004363, -0.7878593, 1.266599E-07, 0.1734756, -0.9848385, -3.752094, 0, 0, 0, 1, };

	const double rl_j1_pos[]{ 0.5000001, 0.5, -1.052204 };
	const double rl_j1_axis[]{ 1.040781E-06, -2.566087E-08, -1 };
	const double rl_j2_pos[]{ 0.7368224, 0.3681228, -1.644 };
	const double rl_j2_axis[]{ 0.02464771, 0.9996962, 5.459879E-08 };
	const double rl_j3_pos[]{ 0.7358373, 0.3281348, -2.744 };
	const double rl_j3_axis[]{ -0.02464798, -0.9996961, -5.960464E-08 };
	const double rl_j4_pos[]{ 0.6126211, 0.5237312, -2.862 };
	const double rl_j4_axis[]{ -1.084796E-06, 4.846983E-07, 1 };
	const double rl_j5_pos[]{ 0.7049203, 0.3724104, -3.8155 };
	const double rl_j5_axis[]{ -0.02464801, -0.9996963, -5.960464E-08 };
	const double rl_j6_pos[]{ 0.6125606, 0.521232, -3.963706 };
	const double rl_j6_axis[]{ -1.843215E-06, 4.544523E-08, 1 };
	const double rl_ee_pm[]{ -0.0246485471725464, -0.99969619512558, 1.04078060303436E-06, 0.612560629844666, -0.99969619512558, 0.0246484875679016, -2.56608210236209E-08, 0.521232008934021, -6.7217471822964E-13, -1.04109687981691E-06, -1, -3.98429584503174, 0, 0, 0, 1, };
	const double rl_p1_pm[]{ 0.02464795, -0.9996962, -1.843215E-06, 0.5544024, 0.9996962, 0.02464789, 4.544521E-08, 0.4903356, -1.071151E-14, -1.843776E-06, 1, -1.367632, 0, 0, 0, 1, };
	const double rl_p2_pm[]{ -1.036373E-06, 0.9996962, 0.02464794, 0.7361314, 2.044205E-07, -0.02464789, 0.9996961, 0.3400754, 0.9999999, 1.041097E-06, -1.788139E-07, -2.468108, 0, 0, 0, 1, };
	const double rl_p3_pm[]{ -1.031966E-06, -0.9996962, -0.02464721, 0.7121031, 3.831801E-07, 0.02464715, -0.9996961, 0.4710802, 0.9999999, -1.041097E-06, 3.576286E-07, -2.741974, 0, 0, 0, 1, };
	const double rl_p4_pm[]{ 1.04225E-06, -0.9996962, 0.02464858, 0.6743334, 3.392566E-08, 0.02464852, 0.9996962, 0.4162145, -1, -1.041097E-06, 1.192099E-07, -3.547571, 0, 0, 0, 1, };
	const double rl_p5_pm[]{ -0.02464858, -0.9996962, 1.040781E-06, 0.6362112, -0.9996962, 0.02464852, -2.566085E-08, 0.5203338, -6.674427E-13, -1.041097E-06, -1, -3.815875, 0, 0, 0, 1, };
	const double rl_p6_pm[]{ 0.02464795, -0.9996962, -1.843215E-06, 0.6125606, 0.9996962, 0.02464789, 4.544521E-08, 0.521232, -1.071151E-14, -1.843776E-06, 1, -3.963706, 0, 0, 0, 1, };
	//Target: {-0.02464855, -0.9845391, -0.1734229, -0.7556161, -0.9996963, 0.02427465, 0.004275769, 0.2511067, 1.341105E-07, 0.1734756, -0.9848384, -3.757817, 0, 0, 0, 1, }

	aris::dynamic::SimpleModel m;

	auto body = m.addPart(pm_hand_change(body_pm));
	auto body_ee = m.addEndEffector(body, body_ee_pm);

	double tem1[3], tem2[3];

	auto la_p1 = m.addPart(pm_hand_change(la_p1_pm));
	auto la_p2 = m.addPart(pm_hand_change(la_p2_pm));
	auto la_p3 = m.addPart(pm_hand_change(la_p3_pm));
	auto la_p4 = m.addPart(pm_hand_change(la_p4_pm));
	auto la_p5 = m.addPart(pm_hand_change(la_p5_pm));
	auto la_p6 = m.addPart(pm_hand_change(la_p6_pm));
	auto la_j1 = m.addRevoluteJoint(body, la_p1, pp_hand_change(la_j1_pos, tem1), pp_hand_change(la_j1_axis, tem2));
	auto la_j2 = m.addRevoluteJoint(la_p1, la_p2, pp_hand_change(la_j2_pos, tem1), pp_hand_change(la_j2_axis, tem2));
	auto la_j3 = m.addRevoluteJoint(la_p2, la_p3, pp_hand_change(la_j3_pos, tem1), pp_hand_change(la_j3_axis, tem2));
	auto la_j4 = m.addRevoluteJoint(la_p3, la_p4, pp_hand_change(la_j4_pos, tem1), pp_hand_change(la_j4_axis, tem2));
	auto la_j5 = m.addRevoluteJoint(la_p4, la_p5, pp_hand_change(la_j5_pos, tem1), pp_hand_change(la_j5_axis, tem2));
	auto la_j6 = m.addRevoluteJoint(la_p5, la_p6, pp_hand_change(la_j6_pos, tem1), pp_hand_change(la_j6_axis, tem2));
	auto la_m1 = m.addMotion(la_j1);
	auto la_m2 = m.addMotion(la_j2);
	auto la_m3 = m.addMotion(la_j3);
	auto la_m4 = m.addMotion(la_j4);
	auto la_m5 = m.addMotion(la_j5);
	auto la_m6 = m.addMotion(la_j6);
	auto la_ee = m.addEndEffector(la_p6, pm_hand_change(la_ee_pm));

	auto ra_p1 = m.addPart(pm_hand_change(ra_p1_pm));
	auto ra_p2 = m.addPart(pm_hand_change(ra_p2_pm));
	auto ra_p3 = m.addPart(pm_hand_change(ra_p3_pm));
	auto ra_p4 = m.addPart(pm_hand_change(ra_p4_pm));
	auto ra_p5 = m.addPart(pm_hand_change(ra_p5_pm));
	auto ra_p6 = m.addPart(pm_hand_change(ra_p6_pm));
	auto ra_j1 = m.addRevoluteJoint(body, ra_p1, pp_hand_change(ra_j1_pos, tem1), pp_hand_change(ra_j1_axis, tem2));
	auto ra_j2 = m.addRevoluteJoint(ra_p1, ra_p2, pp_hand_change(ra_j2_pos, tem1), pp_hand_change(ra_j2_axis, tem2));
	auto ra_j3 = m.addRevoluteJoint(ra_p2, ra_p3, pp_hand_change(ra_j3_pos, tem1), pp_hand_change(ra_j3_axis, tem2));
	auto ra_j4 = m.addRevoluteJoint(ra_p3, ra_p4, pp_hand_change(ra_j4_pos, tem1), pp_hand_change(ra_j4_axis, tem2));
	auto ra_j5 = m.addRevoluteJoint(ra_p4, ra_p5, pp_hand_change(ra_j5_pos, tem1), pp_hand_change(ra_j5_axis, tem2));
	auto ra_j6 = m.addRevoluteJoint(ra_p5, ra_p6, pp_hand_change(ra_j6_pos, tem1), pp_hand_change(ra_j6_axis, tem2));
	auto ra_m1 = m.addMotion(ra_j1);
	auto ra_m2 = m.addMotion(ra_j2);
	auto ra_m3 = m.addMotion(ra_j3);
	auto ra_m4 = m.addMotion(ra_j4);
	auto ra_m5 = m.addMotion(ra_j5);
	auto ra_m6 = m.addMotion(ra_j6);
	auto ra_ee = m.addEndEffector(ra_p6, pm_hand_change(ra_ee_pm));

	auto ll_p1 = m.addPart(pm_hand_change(ll_p1_pm));
	auto ll_p2 = m.addPart(pm_hand_change(ll_p2_pm));
	auto ll_p3 = m.addPart(pm_hand_change(ll_p3_pm));
	auto ll_p4 = m.addPart(pm_hand_change(ll_p4_pm));
	auto ll_p5 = m.addPart(pm_hand_change(ll_p5_pm));
	auto ll_p6 = m.addPart(pm_hand_change(ll_p6_pm));
	auto ll_j1 = m.addRevoluteJoint(body, ll_p1, pp_hand_change(ll_j1_pos, tem1), pp_hand_change(ll_j1_axis, tem2));
	auto ll_j2 = m.addRevoluteJoint(ll_p1, ll_p2, pp_hand_change(ll_j2_pos, tem1), pp_hand_change(ll_j2_axis, tem2));
	auto ll_j3 = m.addRevoluteJoint(ll_p2, ll_p3, pp_hand_change(ll_j3_pos, tem1), pp_hand_change(ll_j3_axis, tem2));
	auto ll_j4 = m.addRevoluteJoint(ll_p3, ll_p4, pp_hand_change(ll_j4_pos, tem1), pp_hand_change(ll_j4_axis, tem2));
	auto ll_j5 = m.addRevoluteJoint(ll_p4, ll_p5, pp_hand_change(ll_j5_pos, tem1), pp_hand_change(ll_j5_axis, tem2));
	auto ll_j6 = m.addRevoluteJoint(ll_p5, ll_p6, pp_hand_change(ll_j6_pos, tem1), pp_hand_change(ll_j6_axis, tem2));
	auto ll_m1 = m.addMotion(ll_j1);
	auto ll_m2 = m.addMotion(ll_j2);
	auto ll_m3 = m.addMotion(ll_j3);
	auto ll_m4 = m.addMotion(ll_j4);
	auto ll_m5 = m.addMotion(ll_j5);
	auto ll_m6 = m.addMotion(ll_j6);
	auto ll_ee = m.addEndEffector(ll_p6, pm_hand_change(ll_ee_pm));

	auto rl_p1 = m.addPart(pm_hand_change(rl_p1_pm));
	auto rl_p2 = m.addPart(pm_hand_change(rl_p2_pm));
	auto rl_p3 = m.addPart(pm_hand_change(rl_p3_pm));
	auto rl_p4 = m.addPart(pm_hand_change(rl_p4_pm));
	auto rl_p5 = m.addPart(pm_hand_change(rl_p5_pm));
	auto rl_p6 = m.addPart(pm_hand_change(rl_p6_pm));
	auto rl_j1 = m.addRevoluteJoint(body, rl_p1, pp_hand_change(rl_j1_pos, tem1), pp_hand_change(rl_j1_axis, tem2));
	auto rl_j2 = m.addRevoluteJoint(rl_p1, rl_p2, pp_hand_change(rl_j2_pos, tem1), pp_hand_change(rl_j2_axis, tem2));
	auto rl_j3 = m.addRevoluteJoint(rl_p2, rl_p3, pp_hand_change(rl_j3_pos, tem1), pp_hand_change(rl_j3_axis, tem2));
	auto rl_j4 = m.addRevoluteJoint(rl_p3, rl_p4, pp_hand_change(rl_j4_pos, tem1), pp_hand_change(rl_j4_axis, tem2));
	auto rl_j5 = m.addRevoluteJoint(rl_p4, rl_p5, pp_hand_change(rl_j5_pos, tem1), pp_hand_change(rl_j5_axis, tem2));
	auto rl_j6 = m.addRevoluteJoint(rl_p5, rl_p6, pp_hand_change(rl_j6_pos, tem1), pp_hand_change(rl_j6_axis, tem2));
	auto rl_m1 = m.addMotion(rl_j1);
	auto rl_m2 = m.addMotion(rl_j2);
	auto rl_m3 = m.addMotion(rl_j3);
	auto rl_m4 = m.addMotion(rl_j4);
	auto rl_m5 = m.addMotion(rl_j5);
	auto rl_m6 = m.addMotion(rl_j6);
	auto rl_ee = m.addEndEffector(rl_p6, pm_hand_change(rl_ee_pm));


	double tem_pm[16];
	s_inv_pm(pm_hand_change(body_pm), tem_pm);
	body->geometryPool().add<ParasolidGeometry>("solid", "C:\\Users\\py033\\Desktop\\xt\\body.x_t", tem_pm);

	s_inv_pm(pm_hand_change(la_p1_pm), tem_pm);
	la_p1->geometryPool().add<ParasolidGeometry>("solid", "C:\\Users\\py033\\Desktop\\xt\\la_p1.x_t", tem_pm);
	s_inv_pm(pm_hand_change(la_p2_pm), tem_pm);
	la_p2->geometryPool().add<ParasolidGeometry>("solid", "C:\\Users\\py033\\Desktop\\xt\\la_p2.x_t", tem_pm);
	s_inv_pm(pm_hand_change(la_p3_pm), tem_pm);
	la_p3->geometryPool().add<ParasolidGeometry>("solid", "C:\\Users\\py033\\Desktop\\xt\\la_p3.x_t", tem_pm);
	s_inv_pm(pm_hand_change(la_p4_pm), tem_pm);
	la_p4->geometryPool().add<ParasolidGeometry>("solid", "C:\\Users\\py033\\Desktop\\xt\\la_p4.x_t", tem_pm);
	s_inv_pm(pm_hand_change(la_p5_pm), tem_pm);
	la_p5->geometryPool().add<ParasolidGeometry>("solid", "C:\\Users\\py033\\Desktop\\xt\\la_p5.x_t", tem_pm);
	s_inv_pm(pm_hand_change(la_p6_pm), tem_pm);
	la_p6->geometryPool().add<ParasolidGeometry>("solid", "C:\\Users\\py033\\Desktop\\xt\\la_p6.x_t", tem_pm);

	s_inv_pm(pm_hand_change(ra_p1_pm), tem_pm);
	ra_p1->geometryPool().add<ParasolidGeometry>("solid", "C:\\Users\\py033\\Desktop\\xt\\ra_p1.x_t", tem_pm);
	s_inv_pm(pm_hand_change(ra_p2_pm), tem_pm);
	ra_p2->geometryPool().add<ParasolidGeometry>("solid", "C:\\Users\\py033\\Desktop\\xt\\ra_p2.x_t", tem_pm);
	s_inv_pm(pm_hand_change(ra_p3_pm), tem_pm);
	ra_p3->geometryPool().add<ParasolidGeometry>("solid", "C:\\Users\\py033\\Desktop\\xt\\ra_p3.x_t", tem_pm);
	s_inv_pm(pm_hand_change(ra_p4_pm), tem_pm);
	ra_p4->geometryPool().add<ParasolidGeometry>("solid", "C:\\Users\\py033\\Desktop\\xt\\ra_p4.x_t", tem_pm);
	s_inv_pm(pm_hand_change(ra_p5_pm), tem_pm);
	ra_p5->geometryPool().add<ParasolidGeometry>("solid", "C:\\Users\\py033\\Desktop\\xt\\ra_p5.x_t", tem_pm);
	s_inv_pm(pm_hand_change(ra_p6_pm), tem_pm);
	ra_p6->geometryPool().add<ParasolidGeometry>("solid", "C:\\Users\\py033\\Desktop\\xt\\ra_p6.x_t", tem_pm);

	s_inv_pm(pm_hand_change(ll_p1_pm), tem_pm);
	ll_p1->geometryPool().add<ParasolidGeometry>("solid", "C:\\Users\\py033\\Desktop\\xt\\ll_p1.x_t", tem_pm);
	s_inv_pm(pm_hand_change(ll_p2_pm), tem_pm);
	ll_p2->geometryPool().add<ParasolidGeometry>("solid", "C:\\Users\\py033\\Desktop\\xt\\ll_p2.x_t", tem_pm);
	s_inv_pm(pm_hand_change(ll_p3_pm), tem_pm);
	ll_p3->geometryPool().add<ParasolidGeometry>("solid", "C:\\Users\\py033\\Desktop\\xt\\ll_p3.x_t", tem_pm);
	s_inv_pm(pm_hand_change(ll_p4_pm), tem_pm);
	ll_p4->geometryPool().add<ParasolidGeometry>("solid", "C:\\Users\\py033\\Desktop\\xt\\ll_p4.x_t", tem_pm);
	s_inv_pm(pm_hand_change(ll_p5_pm), tem_pm);
	ll_p5->geometryPool().add<ParasolidGeometry>("solid", "C:\\Users\\py033\\Desktop\\xt\\ll_p5.x_t", tem_pm);
	s_inv_pm(pm_hand_change(ll_p6_pm), tem_pm);
	ll_p6->geometryPool().add<ParasolidGeometry>("solid", "C:\\Users\\py033\\Desktop\\xt\\ll_p6.x_t", tem_pm);

	s_inv_pm(pm_hand_change(rl_p1_pm), tem_pm);
	rl_p1->geometryPool().add<ParasolidGeometry>("solid", "C:\\Users\\py033\\Desktop\\xt\\rl_p1.x_t", tem_pm);
	s_inv_pm(pm_hand_change(rl_p2_pm), tem_pm);
	rl_p2->geometryPool().add<ParasolidGeometry>("solid", "C:\\Users\\py033\\Desktop\\xt\\rl_p2.x_t", tem_pm);
	s_inv_pm(pm_hand_change(rl_p3_pm), tem_pm);
	rl_p3->geometryPool().add<ParasolidGeometry>("solid", "C:\\Users\\py033\\Desktop\\xt\\rl_p3.x_t", tem_pm);
	s_inv_pm(pm_hand_change(rl_p4_pm), tem_pm);
	rl_p4->geometryPool().add<ParasolidGeometry>("solid", "C:\\Users\\py033\\Desktop\\xt\\rl_p4.x_t", tem_pm);
	s_inv_pm(pm_hand_change(rl_p5_pm), tem_pm);
	rl_p5->geometryPool().add<ParasolidGeometry>("solid", "C:\\Users\\py033\\Desktop\\xt\\rl_p5.x_t", tem_pm);
	s_inv_pm(pm_hand_change(rl_p6_pm), tem_pm);
	rl_p6->geometryPool().add<ParasolidGeometry>("solid", "C:\\Users\\py033\\Desktop\\xt\\rl_p6.x_t", tem_pm);


	double la_ee_pm1[16], ra_ee_pm1[16], ll_ee_pm1[16], rl_ee_pm1[16];
	
	la_ee->updMpm();
	la_ee->getMpm(la_ee_pm1);
	ra_ee->updMpm();
	ra_ee->getMpm(ra_ee_pm1);
	ll_ee->updMpm();
	ll_ee->getMpm(ll_ee_pm1);
	rl_ee->updMpm();
	rl_ee->getMpm(rl_ee_pm1);

	m.allocateMemory();
	
	//std::cout << "time consumed" << aris::core::benchmark(1, [&]()
	//{
	//	for (int i = 0; i < 1000; ++i)
	//	{
	//		body_ee->setMpe(std::array<double, 6>{0, 0, -3e-4*i, 0, 0, 0}.data(), "313");
	//		la_ee->setMpm(pm_hand_change(la_ee_pm));
	//		ra_ee->setMpm(pm_hand_change(ra_ee_pm));
	//		ll_ee->setMpm(pm_hand_change(ll_ee_pm));
	//		rl_ee->setMpm(pm_hand_change(rl_ee_pm));
	//		if (!m.inverseKinematic())
	//		{
	//			std::cout << "inverseKinematic failed" << std::endl;
	//			break;
	//		}
	//	}
	//}) << std::endl;
	

	body_ee->setMpe(std::array<double, 6>{0, 0, -1.5e-2, 0, 0, 0}.data(), "313");
	la_ee->setMpm(pm_hand_change(la_ee_pm));
	ra_ee->setMpm(pm_hand_change(ra_ee_pm));
	ll_ee->setMpm(pm_hand_change(ll_ee_pm));
	rl_ee->setMpm(pm_hand_change(rl_ee_pm));
	if (!m.inverseKinematic(100, 1e-8))
	{
		std::cout << "inverseKinematic failed" << std::endl;
		//break;
	}
	////
	//m.saveXml("C:\\Users\\py033\\Desktop\\m.xml");


	//aris::dynamic::Model m1;
	//m1.loadXml("C:\\Users\\py033\\Desktop\\m.xml");
	//m1.saveAdams("C:\\Users\\py033\\Desktop\\m.cmd");



	std::cout << "-----------------test simple model finished------------" << std::endl << std::endl;
}

