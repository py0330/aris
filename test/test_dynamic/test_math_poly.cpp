#include "test_math_poly.h"
#include <iostream>
#include <aris/dynamic/dynamic.hpp>

using namespace aris::dynamic;


void test_poly3() {
	{
		double k[4]{ 1,0,0,-1 };
		double x[3]{ 1 };
		double result[6];

		auto ret = aris::dynamic::s_poly3_solve(k[0], k[1], k[2], k[3], result);

		if (!s_is_equal(1, 1, result, x, 1e-10)) {
			std::cout << "poly 6 error 3:multiply error!" << std::endl;
			dsp(1, 2, x);
			dsp(1, 2, result);
			std::exit(0);
		}
	}
	{
		double k[4]{ 1,1,0,-2 };
		double x[3]{ 1 };
		double result[6];

		auto ret = aris::dynamic::s_poly3_solve(k[0], k[1], k[2], k[3], result);

		if (!s_is_equal(1, 1, result, x, 1e-10)) {
			std::cout << "poly 6 error 1:multiply error!" << std::endl;
			dsp(1, 2, x);
			dsp(1, 2, result);
			std::exit(0);
		}
	}
	{
		double k[4]{ 1,-3,-8,-10 };
		double x[3]{ 5 };
		double result[6];

		auto ret = aris::dynamic::s_poly3_solve(k[0], k[1], k[2], k[3], result);

		if (!s_is_equal(1, 1, result, x, 1e-10)) {
			std::cout << "poly 6 error 1:multiply error!" << std::endl;
			dsp(1, 2, x);
			dsp(1, 2, result);
			std::exit(0);
		}
	}
	{
		double k[4]{ 1,-3,-13,15 };
		double x[3]{ -3,1,5 };
		double result[6];

		auto ret = aris::dynamic::s_poly3_solve(k[0], k[1], k[2], k[3], result);

		if (!s_is_equal(1, 1, result, x, 1e-10)) {
			std::cout << "poly 6 error 1:multiply error!" << std::endl;
			dsp(1, 2, x);
			dsp(1, 2, result);
			std::exit(0);
		}
	}
	{
		double k[4]{ -1,3,13,-15 };
		double x[3]{ -3,1,5 };
		double result[6];

		auto ret = aris::dynamic::s_poly3_solve(k[0], k[1], k[2], k[3], result);

		if (!s_is_equal(1, 1, result, x, 1e-10)) {
			std::cout << "poly 6 error 1:multiply error!" << std::endl;
			dsp(1, 2, x);
			dsp(1, 2, result);
			std::exit(0);
		}
	}
}

void test_polyn() {
	double k[7]{ 0.3, -0.5, 1.2, 2.6, 6, -3.2, 0.2 };
	double x[6]{ 0.0727452285777216,  0.374341805701567 };
	double mem[36], result[6];
	
	auto ret = aris::dynamic::s_poly_solve(6, k, result, mem);

	if (!s_is_equal(1, 2, result, x, 1e-10)) {
		std::cout << "poly 6 error 1:multiply error!" << std::endl;
		dsp(1, 2, x);
		dsp(1, 2, result);
		std::exit(0);
	}
}

void test_poly_ieq() {
	
	const double p1[]{1, 2, -1, 4, 8, -3, 2};
	const double p2[]{ 3, 11, -15, 4 };
	
	double k[7]{ 0.3, -0.5, 1.2, 2.6, 6, -3.2, 0.2 };
	double x[6]{ 0.0727452285777216,  0.374341805701567 };
	double mem[36], result[6];

	auto ret = aris::dynamic::s_poly_ieq_solve(6, 3, p1, p2, result, mem);

	if (!s_is_equal(1, 2, result, x, 1e-10)) {
		std::cout << "poly 6 error 1:multiply error!" << std::endl;
		dsp(1, 2, x);
		dsp(1, 2, result);
		std::exit(0);
	}
}

void test_poly(){
	std::cout << std::endl << "-----------------test poly--------------------" << std::endl;

	test_poly3();
	test_polyn();

	std::cout << "-----------------test poly finished-----------" << std::endl << std::endl;
}