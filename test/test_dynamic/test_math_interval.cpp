#include "test_math_interval.h"
#include <iostream>
#include <aris/dynamic/dynamic.hpp>

using namespace aris::dynamic;


void test_interval_intersect() {
	
	const int size_a = 8;
	const int size_b = 11;

	double set_a[2*size_a]{
		-100.0, -99,
		-10.0, -5,
		0.0, 0.5,
		10, 11,
		12, 13,
		14, 15,
		16, 17,
		50, 100,
	};

	double set_b[2 * size_b]{
		-105.0, -99.5,
		-50.0, -40,
		-35, -33.5,
		-32, -31.5,
		-1, 5,
		9, 16.5,
		45, 49,
		49.5, 51,
		60, 67,
		70,90,
		95, 105
	};

	double set_c[(size_a + size_b) * 2]{
			-100.000000, - 99.500000,
		0.000000,   0.500000,
		10.000000,   11.000000,
		12.000000,   13.000000,
		14.000000,   15.000000,
		16.000000,   16.500000,
		50.000000,   51.000000,
		60.000000,   67.000000,
		70.000000,   90.000000,
		95.000000,   100.000000,
	};


	double result[(size_a + size_b) * 2];
	auto size_c = s_interval_intersect(size_a, set_a, size_b, set_b, result);

	if (size_c != 10 || (!s_is_equal(size_c, 2, set_c, result, 1e-10))) {
		std::cout << "test_interval_intersect error:multiply error!" << std::endl;
		dsp(size_c, 2, result);
		dsp(size_c, 2, set_c);
		std::exit(0);
	}

}


void test_interval(){
	std::cout << std::endl << "-----------------test poly--------------------" << std::endl;

	test_interval_intersect();

	std::cout << "-----------------test poly finished-----------" << std::endl << std::endl;
}