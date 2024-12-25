#ifndef ARIS_DYNAMIC_MATH_SOLVER_H_
#define ARIS_DYNAMIC_MATH_SOLVER_H_

#include <vector>
#include <numeric>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <algorithm>
#include <cmath>
#include <iterator>
#include <functional>


#include <aris_lib_export.h>
#include <aris/core/basic_type.hpp>

namespace aris::dynamic {
	auto s_newton_raphson_binary_search(std::function<double(double)> f, double x_below, double x_upper) -> double;

	// solve k2*x^2 + k1*x + k0 == 0
	//
	// return solution num
	auto s_poly2_solve(double k2, double k1, double k0, double* x, double zero_check = 1e-10) -> int;

	// solve k3*x^3 + k2*x^2 + k1*x + k0 == 0
	//
	// return solution num
	auto s_poly3_solve(double k3, double k2, double k1, double k0, double* x, double zero_check = 1e-10) -> int;


}

#endif
