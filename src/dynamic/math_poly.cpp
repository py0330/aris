#include <cmath>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <algorithm>
#include <limits>
#include <sstream>
#include <regex>
#include <limits>
#include <type_traits>
#include <ios>

#include "aris/dynamic/math_poly.hpp"
#include "aris/dynamic/math_solver.hpp"
#include "aris/dynamic/math_matrix.hpp"

namespace aris::dynamic{

	auto ARIS_API s_poly2_solve(double k2, double k1, double k0, Size *solution_num, double* x, double zero_check) -> void {
		if (std::abs(k2) < zero_check) {
			if (std::abs(k1) < zero_check) {
				*solution_num = 0;
				return;
			}

			x[0] = -k0 / k1;
			*solution_num = 1;
			return;
		}

		auto b2_4ac = k1 * k1 - 4 * k2 * k0;
		if (b2_4ac < 0) {
			*solution_num = 0;
			return;
		}

		x[0] = (-k1 - std::sqrt(b2_4ac)) / (2 * k2);
		x[1] = (-k1 + std::sqrt(b2_4ac)) / (2 * k2);

		if (k2 < 0)
			std::swap(x[0], x[1]);

		*solution_num = 2;
		return;
	}

	auto ARIS_API s_poly3_solve(double k3, double k2, double k1, double k0, Size *solution_num, double* x, double zero_check) -> void {
		if (std::abs(k3) < zero_check) {
			return s_poly2_solve(k2, k1, k0, solution_num, x, zero_check);
		}

		// diff is:
		// 3*k3*x^2 + 2*k2*x + k1

		double A = 3.0 * k3;
		double B = 2.0 * k2;
		double C = k1;

		auto func = [k3, k2, k1, k0](double x) -> double {
			return ((k3*x + k2)*x + k1)*x + k0;
		};


		double ext[2]{};
		// 导数没有为0的解，原方程一个解 //
		Size ext_num;
		s_poly2_solve(A, B, C, &ext_num, ext, zero_check);
		if (ext_num < 2) {
			double lhs = -1.0, rhs = 1.0;
			while (func(lhs) * func(rhs) > 0) {
				lhs *= 2;
				rhs *= 2;
			}

			x[0] = s_newton_raphson_binary_search(func, lhs, rhs);
			*solution_num = 1;
			return;
		}
		else {
			double fe1 = func(ext[0]);
			double fe2 = func(ext[1]);
			
			if (fe1 * fe2 > 0.0) {
				// 1个根 //
				double value = s_sgn2(k3 * fe1); 
				auto ev = (ext[0] + ext[1]) / 2;
				double bound(ev - value);
				while (func(bound)*fe1 > 0) {
					value *= 2;
					bound -= value;
				}

				x[0] = s_newton_raphson_binary_search(func, std::min(bound, ev), std::max(bound, ev));
				*solution_num = 1;
				return;
			}
			else {
				// 3个根
				double lhs(ext[0] - 1), rhs(ext[1] + 1);

				double value = 1.0;
				while (func(lhs) * fe1 > 0) {
					value *= 2;
					lhs -= value;
				}

				value = 1.0;
				while (func(rhs) * fe2 > 0) {
					value *= 2;
					rhs += value;
				}

				x[0] = s_newton_raphson_binary_search(func, lhs, ext[0]);
				x[1] = s_newton_raphson_binary_search(func, ext[0], ext[1]);
				x[2] = s_newton_raphson_binary_search(func, ext[1], rhs);
				*solution_num = 3;
				return;
			}
		}
	}

	// 
	// mem should be n * n
	auto ARIS_API s_poly_solve(Size n, const double* k, Size *solution_num, double* x, double *mem, double zero_check) -> int {
		double* A = mem;

		if (n == 0) {
			*solution_num = 0;
			return 0;
		}
			
		
		// 多项式首个系数对求解影响巨大，不可以使用zero_check
		if (k[0] == 0.0)
			return s_poly_solve(n - 1, k + 1, solution_num, x, mem, zero_check);

		if (n == 1) {
			if (k[0] == 0) {
				*solution_num = 0;
				return 0;
			}
			else {
				x[0] = -k[1] / k[0];
				*solution_num = 1;
				return 0;
			}
		}

		if (n == 2) {
			s_poly2_solve(k[0], k[1], k[2], solution_num, x, zero_check);
			return 0;
		}
		
		s_vc(n, -1.0 / k[0], k + 1, A);
		s_fill(n - 1, n, 0.0, A + n);
		s_eye(n - 1, A + n, n);

		auto ret = s_schur(n, A, A, nullptr, 100, zero_check);
		
		// 特征分解失败 //
		if (ret)
			return ret;


		// find roots //
		Size &root_num = *solution_num;
		root_num = 0;
		for (Size i = 0; i < std::min(n, n - 1); ) {
			if (std::abs(A[aris::dynamic::at(i + 1, i, n)]) <= zero_check) {
				x[root_num] = A[aris::dynamic::at(i, i, n)];
				root_num++;
				i++;
			}
			else {
				i += 2;
			}
		}

		if (std::abs(A[aris::dynamic::at(n - 1, n - 2, n)]) <= zero_check) {
			x[root_num] = A[aris::dynamic::at(n - 1, n - 1, n)];
			root_num++;
		}

		std::sort(x, x + root_num);

		return 0;
	}

	auto ARIS_API s_poly_ieq_solve(Size m, const double* f, Size* solution_num, double* x, double* mem, double zero_check) -> int {
		{
			Size real_m = m + 1;
			for (int i = 0; f[i] == 0.0 && i < m + 1; ++i)
				real_m--;

			if (real_m == 0) {
				*solution_num = 0;
				return 0;
			}

			f = f + (m - real_m + 1);
			m = real_m - 1;
		}

		Size x_size = 0;

		if ((f[0] >= 0 && (m % 2 == 1)) || (f[0] < 0 && (m % 2 == 0))) {
			x[0] = -std::numeric_limits<double>::infinity();
			x_size += 1;
		}

		Size root_size = 0;
		if (auto ret = s_poly_solve(m, f, &root_size, x + x_size, mem, zero_check))
			return ret;
		x_size += root_size;

		if (f[0] < 0) {
			x[x_size] = std::numeric_limits<double>::infinity();
			x_size += 1;
		}

		*solution_num = x_size / 2;
		return 0;
	}

	// mem should be max(m,n) * max(m,n)
	auto ARIS_API s_poly_ieq_solve(Size m, Size n, const double* f, const double* g, Size* solution_num, double* x, double* mem, double zero_check) -> int {
		
		// get real m and n, make sure that real_m & n > 0
		{
			Size real_m = m + 1, real_n = n + 1;
			for (int i = 0; f[i] == 0.0 && i < m + 1; ++i)
				real_m--;
			for (int i = 0; g[i] == 0.0 && i < n + 1; ++i)
				real_n--;

			if (real_m == 0 || real_n == 0) {
				*solution_num = 0;
				return 0;
			}

			f = f + (m - real_m + 1);
			g = g + (n - real_n + 1);
			m = real_m - 1;
			n = real_n - 1;
		}


		Size x_size = 0;

		

		// 考虑 x = -inf 时多项式的符号
		//% (f1*(-inf)^m) / (g1*(-inf)^n) 
		if ((f[0] * g[0] >= 0 && ((m + n) % 2 == 1)) || (f[0] * g[0] < 0 && ((m + n) % 2 == 0))) {
			x[0] = -std::numeric_limits<double>::infinity();
			x_size += 1;
		}
		
		Size root_size = 0;
		if (auto ret = s_poly_solve(m, f, &root_size, x + x_size, mem, zero_check))
			return ret;
		x_size += root_size;
		if (auto ret = s_poly_solve(n, g, &root_size, x + x_size, mem, zero_check))
			return ret;
		x_size += root_size;

		// 考虑 x = inf 时多项式的符号
		//% (f1*(inf)^m) / (g1*(inf)^n) 
		if (f[0] * g[0] < 0) {
			x[x_size] = std::numeric_limits<double>::infinity();
			x_size += 1;
		}

		std::sort(x, x + x_size);
		*solution_num = x_size / 2;

		return 0;
	}

	auto ARIS_API s_conv(Size m, Size n, const double* u, const double* v, double* result)->void {
		std::fill_n(result, m + n + 1, 0.0);
		s_conv_add(m, n, u, v, result);
	}

	auto ARIS_API s_conv(Size m, Size n, double alpha, const double* u, const double* v, double* result)->void {
		s_conv(m, n, u, v, result);
		s_nv(m + n + 1, alpha, result);
	}

	// 返回向量 u 和 v 的卷积，并加到结果上。如果 u 和 v 是多项式系数的向量，对其卷积与将
	// 这两个多项式相乘等效。
	//
	// result 的长度为 m + n - 1
	auto ARIS_API s_conv_add(Size m, Size n, const double* u, const double* v, double* result)->void {
		for (Size i = 0; i < m + 1; ++i) {
			for (Size j = 0; j < n + 1; ++j) {
				result[i + j] += u[i] * v[j];
			}
		}
	}

	// 返回向量 u 和 v 的卷积，并加到结果上。如果 u 和 v 是多项式系数的向量，对其卷积与将
// 这两个多项式相乘等效。
//
// result 的长度为 m + n - 1
	auto ARIS_API s_conv_add(Size m, Size n, double alpha, const double* u, const double* v, double* result)->void {
		for (Size i = 0; i < m + 1; ++i) {
			for (Size j = 0; j < n + 1; ++j) {
				result[i + j] += alpha * u[i] * v[j];
			}
		}
	}
}
