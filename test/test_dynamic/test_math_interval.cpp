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


	double result[(size_a + size_b -1)*2];
	aris::Size size_c;

	s_interval_intersect(size_a, size_b, set_a, set_b, size_c, result);
	if (size_c != 10 || (!s_is_equal(size_c, 2, set_c, result, 1e-10))) {
		std::cout << "test_interval_intersect error:multiply error!" << std::endl;
		dsp(size_c, 2, result);
		dsp(size_c, 2, set_c);
		std::exit(0);
	}

	s_interval_intersect(size_b, size_a, set_b, set_a, size_c, result);
	if (size_c != 10 || (!s_is_equal(size_c, 2, set_c, result, 1e-10))) {
		std::cout << "test_interval_intersect error:multiply error!" << std::endl;
		dsp(size_c, 2, result);
		dsp(size_c, 2, set_c);
		std::exit(0);
	}

	s_interval_intersect(size_a - 1, size_b, set_a, set_b, size_c, result);
	if (size_c != 6 || (!s_is_equal(size_c, 2, set_c, result, 1e-10))) {
		std::cout << "test_interval_intersect error:multiply error!" << std::endl;
		dsp(size_c, 2, result);
		dsp(size_c, 2, set_c);
		std::exit(0);
	}
	
	s_interval_intersect(size_b, size_a - 1, set_b, set_a, size_c, result);
	if (size_c != 6 || (!s_is_equal(size_c, 2, set_c, result, 1e-10))) {
		std::cout << "test_interval_intersect error:multiply error!" << std::endl;
		dsp(size_c, 2, result);
		dsp(size_c, 2, set_c);
		std::exit(0);
	}
}
void test_interval_union() {

	const int size_a = 8;
	const int size_b = 11;

	double set_a[2 * size_a]{
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
		 -105.0000, - 99.0000,
  - 50.0000, - 40.0000,
  - 35.0000, - 33.5000,
  - 32.0000, - 31.5000,
  - 10.0000, - 5.0000,
   - 1.0000,    5.0000,
	9.0000,   17.0000,
   45.0000 ,  49.0000,
   49.5000,  105.0000,
	};


	double result[(size_a + size_b)*2];
	aris::Size size_c;
	
	s_interval_union(size_a, size_b, set_a, set_b, size_c, result);
	if (size_c != 9 || (!s_is_equal(size_c, 2, set_c, result, 1e-10))) {
		std::cout << "test_interval_intersect error:multiply error!" << std::endl;
		dsp(size_c, 2, result);
		dsp(size_c, 2, set_c);
		std::exit(0);
	}

	s_interval_union(size_b, size_a, set_b, set_a, size_c, result);
	if (size_c != 9 || (!s_is_equal(size_c, 2, set_c, result, 1e-10))) {
		std::cout << "test_interval_intersect error:multiply error!" << std::endl;
		dsp(size_c, 2, result);
		dsp(size_c, 2, set_c);
		std::exit(0);
	}

}
void test_interval_inverse() {

	const aris::Size size_a = 5;
	constexpr double inf = std::numeric_limits<double>::infinity();
	const double set_a[size_a * 2]{ -100.0, -99,
		-10.0, -5,
		0.0, 0.5,
		10, 11,
		50, 100, };

	const double set_b[size_a * 2]{ -inf, -99,
		-10.0, -5,
		0.0, 0.5,
		10, 11,
		50, 100, };

	const double set_c[size_a * 2]{ -100.0, -99,
		-10.0, -5,
		0.0, 0.5,
		10, 11,
		50, inf, };

	const double set_d[size_a * 2]{ -inf, -99,
		-10.0, -5,
		0.0, 0.5,
		10, 11,
		50, inf, };

	const double inv_a[]{
			  -inf, -100.0000,
		  - 99.0000, - 10.0000,
		   - 5.0000,         0,
			0.5000,   10.0000,
		   11.0000,   50.0000,
		  100.0000,       inf,
	};

	const double inv_b[]{
		  -99.0000, -10.0000,
		   -5.0000,         0,
			0.5000,   10.0000,
		   11.0000,   50.0000,
		  100.0000,       inf,
	};

	const double inv_c[]{
			  -inf, -100.0000,
		  -99.0000, -10.0000,
		   -5.0000,         0,
			0.5000,   10.0000,
		   11.0000,   50.0000,
	};

	const double inv_d[]{
			-99.0000, -10.0000,
		-5.0000,         0,
		 0.5000,   10.0000,
		11.0000 ,  50.0000,
	};

	double result[(size_a + 1) * 2];
	aris::Size size_c;

	s_interval_inverse(size_a, set_a, size_c, result);
	if (size_c != 6 || (!s_is_equal(size_c, 2, inv_a, result, 1e-10))) {
		std::cout << "test_interval_intersect error:multiply error!" << std::endl;
		dsp(size_c, 2, result);
		dsp(size_c, 2, inv_a);
		std::exit(0);
	}

	s_interval_inverse(size_a, set_b, size_c, result);
	if (size_c != 5 || (!s_is_equal(size_c, 2, inv_b, result, 1e-10))) {
		std::cout << "test_interval_intersect error:multiply error!" << std::endl;
		dsp(size_c, 2, result);
		dsp(size_c, 2, inv_a);
		std::exit(0);
	}

	s_interval_inverse(size_a, set_c, size_c, result);
	if (size_c != 5 || (!s_is_equal(size_c, 2, inv_c, result, 1e-10))) {
		std::cout << "test_interval_intersect error:multiply error!" << std::endl;
		dsp(size_c, 2, result);
		dsp(size_c, 2, inv_c);
		std::exit(0);
	}

	s_interval_inverse(size_a, set_d, size_c, result);
	if (size_c != 4 || (!s_is_equal(size_c, 2, inv_d, result, 1e-10))) {
		std::cout << "test_interval_intersect error:multiply error!" << std::endl;
		dsp(size_c, 2, result);
		dsp(size_c, 2, inv_a);
		std::exit(0);
	}
}

void test_interval(){
	std::cout << std::endl << "-----------------test intervals--------------------" << std::endl;

	test_interval_intersect();
	test_interval_union();
	test_interval_inverse();

	std::cout << "-----------------test intervals finished-----------" << std::endl << std::endl;
}