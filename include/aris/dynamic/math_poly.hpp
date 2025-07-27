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
	auto ARIS_API s_poly2_solve(double k2, double k1, double k0, Size* solution_num, double* x, double zero_check = 1e-10) -> void;

	// solve k3*x^3 + k2*x^2 + k1*x + k0 == 0
	//
	// return solution num
	auto ARIS_API s_poly3_solve(double k3, double k2, double k1, double k0, Size* solution_num, double* x, double zero_check = 1e-10) -> void;

	// solve k[0]*x^n + k[1]*x^(n-1) ... + k[n-1]*x + k[n] == 0
	// 
	// mem should be n*n
	// 
	// return solution num
	auto ARIS_API s_poly_solve(Size n, const double *k, Size* solution_num, double* x, double* mem, double zero_check = 1e-10) -> int;

	// 
	// 
	// solve 
	// 
	// (f[0]*x^m + ... f[m-1]*x + f[m]) < 0
	// 
	// give x:
	// [ l1 r1 ]
	// | l2 r2 |
	// |  ...  |
	// [ lk rk ]
	// 
	// return interval num, less than (m + n + 2)/2，m+n为奇数时，为(m+n+1)/2, 为偶数时，为(m+n+2)/2
	//
	// x should be larger than (m + n + 2)
	auto ARIS_API s_poly_ieq_solve(Size m, const double* f, Size* solution_num, double* x, double* mem, double zero_check = 1e-10) -> int;

	// 
	// 
	// solve 
	// 
	// (f[0]*x^m + ... f[m-1]*x + f[m]) * (g[0]*x^n + ... g[n-1]*x + g[n]) < 0
	// or
	// (f[0]*x^m + ... f[m-1]*x + f[m]) / (g[0]*x^n + ... g[n-1]*x + g[n]) < 0
	// 
	// give x:
	// [ l1 r1 ]
	// | l2 r2 |
	// |  ...  |
	// [ lk rk ]
	// 
	// return interval num, less than (m + n + 2)/2，m+n为奇数时，为(m+n+1)/2, 为偶数时，为(m+n+2)/2
	//
	// x should be larger than (m + n + 2)
	auto ARIS_API s_poly_ieq_solve(Size m, Size n, const double* f, const double* g, Size *solution_num, double* x, double* mem, double zero_check = 1e-10) -> int;

	// 返回向量 u 和 v 的卷积。如果 u 和 v 是多项式系数的向量，对其卷积与将
	// 这两个多项式相乘等效。
	//
	// result 的长度为 m + n - 1
	auto ARIS_API s_conv(Size m, Size n, const double* u, const double* v, double* result)->void;

	// 返回向量 u 和 v 的卷积。如果 u 和 v 是多项式系数的向量，对其卷积与将
	// 这两个多项式相乘等效。
	//
	// result 的长度为 m + n - 1
	auto ARIS_API s_conv(Size m, Size n, double alpha, const double* u, const double* v, double* result)->void;

	// 返回向量 u 和 v 的卷积，并加到结果上。如果 u 和 v 是多项式系数的向量，对其卷积与将
	// 这两个多项式相乘等效。
	//
	// result 的长度为 m + n - 1
	auto ARIS_API s_conv_add(Size m, Size n, const double* u, const double* v, double* result)->void;

	// 返回向量 u 和 v 的卷积，并加到结果上。如果 u 和 v 是多项式系数的向量，对其卷积与将
// 这两个多项式相乘等效。
//
// result 的长度为 m + n - 1
	auto ARIS_API s_conv_add(Size m, Size n, double alpha, const double* u, const double* v, double* result)->void;
}

#endif
