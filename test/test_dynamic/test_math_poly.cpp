#include "test_math_poly.h"
#include <iostream>
#include <aris/dynamic/dynamic.hpp>

using namespace aris::dynamic;


void test_poly3() {
	{
		double k[4]{ 1,0,0,-1 };
		double x[3]{ 1 };
		double result[6];

		aris::Size root_num;
		aris::dynamic::s_poly3_solve(k[0], k[1], k[2], k[3], &root_num, result);

		if (root_num!= 1 || (!s_is_equal(1, 1, result, x, 1e-10))) {
			std::cout << "poly 3 error 3:multiply error!" << std::endl;
			dsp(1, 2, x);
			dsp(1, 2, result);
			std::exit(0);
		}
	}
	{
		double k[4]{ 1,1,0,-2 };
		double x[3]{ 1 };
		double result[6];

		aris::Size root_num;
		aris::dynamic::s_poly3_solve(k[0], k[1], k[2], k[3], &root_num, result);

		if (root_num != 1 || (!s_is_equal(1, 1, result, x, 1e-10))) {
			std::cout << "poly 3 error 1:multiply error!" << std::endl;
			dsp(1, 2, x);
			dsp(1, 2, result);
			std::exit(0);
		}
	}
	{
		double k[4]{ 1,-3,-8,-10 };
		double x[3]{ 5 };
		double result[6];

		aris::Size root_num;
		aris::dynamic::s_poly3_solve(k[0], k[1], k[2], k[3], &root_num, result);

		if (root_num != 1 || (!s_is_equal(1, 1, result, x, 1e-10))) {
			std::cout << "poly 3 error 1:multiply error!" << std::endl;
			dsp(1, 2, x);
			dsp(1, 2, result);
			std::exit(0);
		}
	}
	{
		double k[4]{ 1,-3,-13,15 };
		double x[3]{ -3,1,5 };
		double result[6];

		aris::Size root_num;
		aris::dynamic::s_poly3_solve(k[0], k[1], k[2], k[3], &root_num, result);

		if (root_num!= 3 || (!s_is_equal(1, 1, result, x, 1e-10))) {
			std::cout << "poly 3 error 1:multiply error!" << std::endl;
			dsp(1, 3, x);
			dsp(1, 3, result);
			std::exit(0);
		}
	}
	{
		double k[4]{ -1,3,13,-15 };
		double x[3]{ -3,1,5 };
		double result[6];

		aris::Size root_num;
		aris::dynamic::s_poly3_solve(k[0], k[1], k[2], k[3], &root_num, result);

		if (root_num != 3 || (!s_is_equal(1, 1, result, x, 1e-10))) {
			std::cout << "poly 3 error 1:multiply error!" << std::endl;
			dsp(1, 3, x);
			dsp(1, 3, result);
			std::exit(0);
		}
	}
}

void test_polyn() {
	double k[7]{ 0.3, -0.5, 1.2, 2.6, 6, -3.2, 0.2 };
	double x[6]{ 0.0727452285777216,  0.374341805701567 };
	double mem[36], result[6];
	
	aris::Size root_num;
	auto ret = aris::dynamic::s_poly_solve(6, k, &root_num, result, mem);

	if ((root_num != 2) || (!s_is_equal(1, 2, result, x, 1e-10))) {
		std::cout << "poly 6 error 1:multiply error!" << std::endl;
		dsp(1, 2, x);
		dsp(1, 2, result);
		std::exit(0);
	}
}

void test_poly_ieq() {
	{
		const double p1[]{ 1, 2, -1, 4, 8, -3, 2 };
		const double p2[]{ 3, 11, -15, 4 };

		double x[6]{ -std::numeric_limits<double>::infinity(), -4.77280128827758,
			 -2.42023309482179, -1.60698712975347,
			  0.39020938729981,         0.715925234311104 };
		double mem[36], result[6];

		aris::Size solution_num;
		auto ret = aris::dynamic::s_poly_ieq_solve(6, 3, p1, p2, &solution_num, result, mem);

		if ((solution_num != 3) || (!s_is_equal(1, solution_num*2, result, x, 1e-10))) {
			std::cout << "poly 6 error 1:multiply error!" << std::endl;
			dsp(1, 2, x);
			dsp(1, 2, result);
			std::exit(0);
		}
	}

	{
		const double p1[]{ 0, 1, 2, -1, 4, 8, -3, 2 };
		const double p2[]{ 0, 3, 11, -15, 4 };

		double x[6]{ -std::numeric_limits<double>::infinity(), -4.77280128827758,
			 -2.42023309482179, -1.60698712975347,
			  0.39020938729981,         0.715925234311104 };
		double mem[36], result[6];

		aris::Size solution_num;
		auto ret = aris::dynamic::s_poly_ieq_solve(7, 4, p1, p2, &solution_num, result, mem);

		if ((solution_num != 3) || (!s_is_equal(1, solution_num*2, result, x, 1e-10))) {
			std::cout << "poly 6 error 1:multiply error!" << std::endl;
			dsp(1, 2, x);
			dsp(1, 2, result);
			std::exit(0);
		}
	}

	{
		const double p1[]{ 0, 1 };
		const double p2[]{ 0, 1 };

		double x[6]{ -std::numeric_limits<double>::infinity(), -4.77280128827758,
			 -2.42023309482179, -1.60698712975347,
			  0.39020938729981,         0.715925234311104 };
		double mem[36], result[6];

		aris::Size solution_num;
		auto ret = aris::dynamic::s_poly_ieq_solve(1, 1, p1, p2, &solution_num, result, mem);

		if ((solution_num != 0) || (!s_is_equal(1, solution_num*2, result, x, 1e-10))) {
			std::cout << "poly 6 error 1:multiply error!" << std::endl;
			dsp(1, 2, x);
			dsp(1, 2, result);
			std::exit(0);
		}
	}

	{
		const double p1[]{ 0, -1 };
		const double p2[]{ 0, 1 };

		double x[6]{ -std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity() };
		double mem[36], result[6];

		aris::Size solution_num;
		auto ret = aris::dynamic::s_poly_ieq_solve(1, 1, p1, p2, &solution_num, result, mem);

		if ((solution_num != 1) || (!s_is_equal(1, solution_num*2, result, x, 1e-10))) {
			std::cout << "poly 6 error 1:multiply error!" << std::endl;
			dsp(1, 2, x);
			dsp(1, 2, result);
			std::exit(0);
		}
	}

	{
		const double p1[]{ 0, -1 };
		const double p2[]{ 0, 0 };

		double x[6]{ -std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity() };
		double mem[36], result[6];

		aris::Size solution_num;
		auto ret = aris::dynamic::s_poly_ieq_solve(1, 1, p1, p2, &solution_num, result, mem);

		if ((solution_num != 0) || (!s_is_equal(1, solution_num * 2, result, x, 1e-10))) {
			std::cout << "poly 6 error 1:multiply error!" << std::endl;
			dsp(1, 2, x);
			dsp(1, 2, result);
			std::exit(0);
		}
	}

	{
		const double p1[]{ 0, -1 };
		const double p2[]{ 0, 0 };

		double x[6]{ -std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity() };
		double mem[36], result[6];

		aris::Size solution_num;
		auto ret = aris::dynamic::s_poly_ieq_solve(1, 1, p1, p2, &solution_num, result, mem);

		if ((solution_num != 0) || (!s_is_equal(1, solution_num * 2, result, x, 1e-10))) {
			std::cout << "poly 6 error 1:multiply error!" << std::endl;
			dsp(1, 2, x);
			dsp(1, 2, result);
			std::exit(0);
		}
	}

}

void test_poly_conv() {

	const double p1[]{ 1, 2, -1, 4, 8, -3, 2 };
	const double p2[]{ 3, 11, -15, 4 };

	double x[10]{ 3.0,   17.0,   4.0, - 25.0,   91.0,   15, - 131,   99, - 42,   8.000000 };
	double mem[36], result[10];

	aris::dynamic::s_conv(6, 3, p1, p2, result);

	if ((!s_is_equal(1, 10, result, x, 1e-10))) {
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
	test_poly_ieq();
	test_poly_conv();

	std::cout << "-----------------test poly finished-----------" << std::endl << std::endl;
}