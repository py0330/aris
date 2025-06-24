#ifndef ARIS_DYNAMIC_MATH_POLY_H_
#define ARIS_DYNAMIC_MATH_POLY_H_

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
	// solve k2*x^2 + k1*x + k0 == 0
	//
	// return solution num
	auto ARIS_API s_poly2_solve(double k2, double k1, double k0, double* x, double zero_check = 1e-10) -> int;

	// solve k3*x^3 + k2*x^2 + k1*x + k0 == 0
	//
	// return solution num
	auto ARIS_API s_poly3_solve(double k3, double k2, double k1, double k0, double* x, double zero_check = 1e-10) -> int;

	// solve k[0]*x^n + k[1]*x^(n-1) ... + k[n-1]*x + k[n] == 0
	// 
	// mem should be n*n
	// 
	// return solution num
	auto ARIS_API s_polyn_solve(Size n, const double *k, double* x, double* mem, double zero_check = 1e-10) -> int;
}

#endif
