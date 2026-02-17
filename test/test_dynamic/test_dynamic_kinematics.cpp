//#define ARIS_DEBUG

#include "test_dynamic_kinematics.h"
#include <iostream>
#include <aris/dynamic/dynamic.hpp>
#include <aris/dynamic/kinematics.hpp>
#include <random>


using namespace aris::dynamic;

void test_eye_in_hand(){
	const double pe_eye_in_tool[6]{ 0.1,0.2,0.3,0.111,0.221,0.832 };
	double pq_eye_in_tool[7];
	s_pe2pq(pe_eye_in_tool, pq_eye_in_tool, "321");

	const double pe_obj_in_base[6]{ 1.23,2.2,4.3,0.511,0.321,0.932 };
	double pq_obj_in_base[7];
	s_pe2pq(pe_obj_in_base, pq_obj_in_base, "321");

	// make data
	const int n = 10;
	double pe_tool_in_base[n][6]{
		{0.1,0.2,0.1,0.1,0.2,0.3},
		{0.2,0.4,-0.3,0.5,0.2,0.3},
		{0.3,0.6,0.3,0.8,0.2,0.3},
		{0.3,0.8,-0.3,0.1,0.2,0.3},
		{0.2,0.9,0.3,0.1,0.5,0.3},
		{0.1,0.9,-0.3,0.1,0.8,0.3},
		{0.1,0.8,0.3,0.1,0.2,0.3},
		{0.2,0.7,-0.3,0.1,0.2,0.6},
		{0.3,0.6,0.3,0.1,0.2,0.9},
	};


	double pq_tool_in_base[n][7];
	double pq_obj_in_eye[n][7];

	for (int i= 0; i < n; ++i) {
		s_pe2pq(pe_tool_in_base[i], pq_tool_in_base[i], "123");


		double pq_eye_in_base[7];
		s_pq_dot_pq(pq_tool_in_base[i], pq_eye_in_tool, pq_eye_in_base);

		double pm_eye_in_base[16];
		s_pq2pm(pq_eye_in_base, pm_eye_in_base);
		
		s_inv_pq2pq(pm_eye_in_base, pq_obj_in_base, pq_obj_in_eye[i]);
	}


	//dsp(1, 4, plane2);
	double pq_eye_in_tool_result[7];
	double mem[16 * n * n];
	aris::dynamic::s_eye_in_hand_calib(n, *pq_obj_in_eye, *pq_tool_in_base, pq_eye_in_tool_result, mem);

	if (!aris::dynamic::s_is_equal(7, 1, pq_eye_in_tool_result, pq_eye_in_tool, 1e-10))
		std::cout << __FILE__ << "  " << __LINE__ << "  test_eye_in_hand error" << std::endl;
}
void test_eye_in_hand2(){
	// make data
	const double pq_eye_in_tool[7]{ 27.9248522252293867, -141.3005301794123056,   82.1071095579682009, -0.0499607194721478,   0.2653266569221161, -0.9627559257860469,0.0143777258358482 };

	const int n = 8;
	double pe_tool_in_base[n][6]{
		{805.274911,315.296968,622.923222,270.089615,7.576354,143.475021},
		{807.528099,192.358122,533.766378,252.773708,16.775748,139.864493 },
		{810.656227,193.447522,555.544276,252.160947,17.216653,139.284136},
		{703.055386,393.973543,635.753282,297.275858,-2.776476,146.597051},
		{736.764133,407.004243,576.898704,297.385539,-2.511494,144.171319},
		{774.181753,376.340377,550.681197,294.633840,-2.338603,142.601433},
		{708.394648,560.365043,546.019583,327.438039,-15.554671,127.257024},
		{788.483623,369.193844,556.525055,291.617520,0.320245,141.774271}
	};
	double pe_obj_in_eye[n][6]{
		{ -8.54, 14.123, 516.975, 1.64404, 0.083299, 3.1057},
		{24.097, 4.539, 440.932, 1.3541, 0.096654, 2.90711},
		{13.153, 9.99, 461.617, 1.34519, 0.102088, 2.89634},
		{-13.778, 29.961, 544.16, 2.10252, - 0.057346, - 3.00314},
		{-22.242, 17.293, 483.206, 2.10872, - 0.018245, - 2.98404},
		{17.09, 15.964, 449.284, 2.06448, 0.016135, - 2.97464},
		{9.191, 26.467, 499.036, 2.55876, - 0.138528, - 2.63453},
		{3.022, 18.44, 452.709, 2.01848, 0.057736, - 3.01188},
	};

	double pq_tool_in_base[n][7];
	double pq_obj_in_eye[n][7];

	for (int i = 0; i < n; ++i) {
		pe_tool_in_base[i][3] *= aris::PI / 180;
		pe_tool_in_base[i][4] *= aris::PI / 180;
		pe_tool_in_base[i][5] *= aris::PI / 180;

		s_pe2pq(pe_tool_in_base[i], pq_tool_in_base[i], "321");
		s_pe2pq(pe_obj_in_eye[i], pq_obj_in_eye[i], "321");
	}
	
	double pq_eye_in_tool_result[7];
	double mem[16 * n * n];
	aris::dynamic::s_eye_in_hand_calib(n, *pq_obj_in_eye, *pq_tool_in_base, pq_eye_in_tool_result, mem);

	if (!aris::dynamic::s_is_equal(7, 1, pq_eye_in_tool_result, pq_eye_in_tool, 1e-10))
		std::cout << __FILE__ << "  " << __LINE__ << "  test_eye_in_hand error" << std::endl;
}
void test_eye_to_hand() {
	const double pe_eye_in_tool[6]{ 0.1,0.2,0.3,0.111,0.221,0.832 };
	double pq_eye_in_base[7];
	s_pe2pq(pe_eye_in_tool, pq_eye_in_base, "321");

	const double pe_obj_in_tool[6]{ 1.23,2.2,4.3,0.511,0.321,0.932 };
	double pq_obj_in_tool[7];
	s_pe2pq(pe_obj_in_tool, pq_obj_in_tool, "321");

	// make data
	const int n = 10;
	double pe_tool_in_base[n][6]{
		{0.1,0.2,0.1,0.1,0.2,0.3},
		{0.2,0.4,-0.3,0.5,0.2,0.3},
		{0.3,0.6,0.3,0.8,0.2,0.3},
		{0.3,0.8,-0.3,0.1,0.2,0.3},
		{0.2,0.9,0.3,0.1,0.5,0.3},
		{0.1,0.9,-0.3,0.1,0.8,0.3},
		{0.1,0.8,0.3,0.1,0.2,0.3},
		{0.2,0.7,-0.3,0.1,0.2,0.6},
		{0.3,0.6,0.3,0.1,0.2,0.9},
	};


	double pq_tool_in_base[n][7];
	double pq_obj_in_eye[n][7];

	for (int i = 0; i < n; ++i) {
		s_pe2pq(pe_tool_in_base[i], pq_tool_in_base[i], "123");

		double pq_obj_in_base[7];
		s_pq_dot_pq(pq_tool_in_base[i], pq_obj_in_tool, pq_obj_in_base);
		s_inv_pq_dot_pq(pq_eye_in_base, pq_obj_in_base, pq_obj_in_eye[i]);
	}


	//dsp(1, 4, plane2);
	double pq_eye_in_base_result[7];
	double mem[16 * n * n];
	aris::dynamic::s_eye_to_hand_calib(n, *pq_obj_in_eye, *pq_tool_in_base, pq_eye_in_base_result, mem);

	if (!aris::dynamic::s_is_equal(7, 1, pq_eye_in_base_result, pq_eye_in_base, 1e-10))
		std::cout << __FILE__ << "  " << __LINE__ << "  test_eye_to_hand error" << std::endl;
}
void test_eye_to_hand2() {
	// make data
	const double pq_eye_in_base[7]{ 27.9248522252293867, -141.3005301794123056,   82.1071095579682009, -0.0499607194721478,   0.2653266569221161, -0.9627559257860469,0.0143777258358482 };

	const int n = 8;
	double pe_base_in_tool[n][6]{
		{805.274911,315.296968,622.923222,270.089615,7.576354,143.475021},
		{807.528099,192.358122,533.766378,252.773708,16.775748,139.864493 },
		{810.656227,193.447522,555.544276,252.160947,17.216653,139.284136},
		{703.055386,393.973543,635.753282,297.275858,-2.776476,146.597051},
		{736.764133,407.004243,576.898704,297.385539,-2.511494,144.171319},
		{774.181753,376.340377,550.681197,294.633840,-2.338603,142.601433},
		{708.394648,560.365043,546.019583,327.438039,-15.554671,127.257024},
		{788.483623,369.193844,556.525055,291.617520,0.320245,141.774271}
	};
	double pe_obj_in_eye[n][6]{
		{ -8.54, 14.123, 516.975, 1.64404, 0.083299, 3.1057},
		{24.097, 4.539, 440.932, 1.3541, 0.096654, 2.90711},
		{13.153, 9.99, 461.617, 1.34519, 0.102088, 2.89634},
		{-13.778, 29.961, 544.16, 2.10252, -0.057346, -3.00314},
		{-22.242, 17.293, 483.206, 2.10872, -0.018245, -2.98404},
		{17.09, 15.964, 449.284, 2.06448, 0.016135, -2.97464},
		{9.191, 26.467, 499.036, 2.55876, -0.138528, -2.63453},
		{3.022, 18.44, 452.709, 2.01848, 0.057736, -3.01188},
	};

	double pq_tool_in_base[n][7];
	double pq_obj_in_eye[n][7];

	for (int i = 0; i < n; ++i) {
		pe_base_in_tool[i][3] *= aris::PI / 180;
		pe_base_in_tool[i][4] *= aris::PI / 180;
		pe_base_in_tool[i][5] *= aris::PI / 180;

		double pq_base_in_tool[7];

		s_pe2pq(pe_base_in_tool[i], pq_base_in_tool, "321");
		s_inv_pq(pq_base_in_tool, pq_tool_in_base[i]);
		s_pe2pq(pe_obj_in_eye[i], pq_obj_in_eye[i], "321");
	}

	double pq_eye_in_base_result[7];
	double mem[16 * n * n];
	aris::dynamic::s_eye_to_hand_calib(n, *pq_obj_in_eye, *pq_tool_in_base, pq_eye_in_base_result, mem);

	if (!aris::dynamic::s_is_equal(7, 1, pq_eye_in_base_result, pq_eye_in_base, 1e-10))
		std::cout << __FILE__ << "  " << __LINE__ << "  test_eye_in_hand error" << std::endl;
}
void test_eye_to_hand3() {
	// make data
	const double pq_eye_in_base[7]{ 27.9248522252293867, -141.3005301794123056,   82.1071095579682009, -0.0499607194721478,   0.2653266569221161, -0.9627559257860469,0.0143777258358482 };

	const int n = 8;
	double pe_base_in_tool[n][6]{
	{152.096, -342.435, 454.516, 102.799, 3.886, 154.250} ,
	{139.332, -346.088, 400.398, 97.881, 6.300, 162.170},
	{267.857, -423.881, 360.515, 133.752, 23.827, 159.881},
	{267.860, -397.674, 312.110, 128.260, 32.698, 148.348},
	{267.873, -397.662, 312.103, 136.811, 46.879, 133.790},
	{211.447, -406.843, 304.343, 115.380, 59.267, 106.440},
	{211.435, -406.846, 261.977, 231.302, 69.489, 227.376},
	{211.440, -338.798, 280.090, 236.417, 63.872, 222.855}
	};
	double pe_obj_in_eye[n][6]{
	{-0.02105898, -0.18919055, 0.662908, -1.76902863, 1.43579416, 0.75142922},
	{-0.03308171, -0.12526707, 0.67971084, -1.65377137, 1.33307705, 0.92968707},
	{-0.09910326, -0.1261038, 0.50099353, -2.14455275, 0.79622204, 0.48095992},
	{-0.07029129, -0.09550262, 0.48024125, -2.33913934, 0.87678114, 0.42668346},
	{-0.04270613, -0.12599228, 0.46413674, -2.70376415, 0.77715338, -0.42233157},
	{-0.05018178, -0.14545301, 0.5312406, -2.62012681, 0.57101761, 0.23607775},
	{-0.03434216, -0.09710595, 0.53011277, -2.94501414, -0.41539958, 0.02554787},
	{0.05179333, -0.10664992, 0.5319408, -2.91138296, -0.51282174, -0.3350395}
	};

	double pq_tool_in_base[n][7];
	double pq_obj_in_eye[n][7];

	for (int i = 0; i < n; ++i) {
		pe_base_in_tool[i][0] /= 1000.0;
		pe_base_in_tool[i][1] /= 1000.0;
		pe_base_in_tool[i][2] /= 1000.0;
		pe_base_in_tool[i][3] *= aris::PI / 180;
		pe_base_in_tool[i][4] *= aris::PI / 180;
		pe_base_in_tool[i][5] *= aris::PI / 180;

		double pq_base_in_tool[7];

		s_pe2pq(pe_base_in_tool[i], pq_base_in_tool, "321");
		s_inv_pq(pq_base_in_tool, pq_tool_in_base[i]);
		s_pe2pq(pe_obj_in_eye[i], pq_obj_in_eye[i], "321");
	}

	double pq_eye_in_base_result[7];
	double mem[16 * n * n];
	aris::dynamic::s_eye_to_hand_calib(n, *pq_obj_in_eye, *pq_tool_in_base, pq_eye_in_base_result, mem);

	aris::dynamic::dsp(1, 7, pq_eye_in_base_result);


	if (!aris::dynamic::s_is_equal(7, 1, pq_eye_in_base_result, pq_eye_in_base, 1e-10))
		std::cout << __FILE__ << "  " << __LINE__ << "  test_eye_in_hand error" << std::endl;
}

void test_kinematics(){
	std::cout << std::endl << "-----------------test kinematics--------------------" << std::endl;

	test_eye_in_hand();
	//test_eye_in_hand2();
	test_eye_to_hand();
	//test_eye_to_hand3();

	std::cout << "-----------------test kinematics finished-----------" << std::endl << std::endl;
}