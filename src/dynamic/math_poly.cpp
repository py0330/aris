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

	auto ARIS_API s_poly2_solve(double k2, double k1, double k0, double* x, double zero_check) -> int {
		if (std::abs(k2) < zero_check) {
			if (std::abs(k1) < zero_check)
				return 0;

			x[0] = -k0 / k1;
			return 1;
		}

		auto b2_4ac = k1 * k1 - 4 * k2 * k0;
		if (b2_4ac < 0) {
			return 0;
		}

		x[0] = (-k1 - std::sqrt(b2_4ac)) / (2 * k2);
		x[1] = (-k1 + std::sqrt(b2_4ac)) / (2 * k2);

		if (k2 < 0)
			std::swap(x[0], x[1]);

		return 2;
	}

	auto ARIS_API s_poly3_solve(double k3, double k2, double k1, double k0, double* x, double zero_check) -> int {
		if (std::abs(k3) < zero_check) {
			return s_poly2_solve(k2, k1, k0, x, zero_check);
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
		if (auto ext_num = s_poly2_solve(A, B, C, ext, zero_check); ext_num < 2) {
			double lhs = -1.0, rhs = 1.0;
			while (func(lhs) * func(rhs) > 0) {
				lhs *= 2;
				rhs *= 2;
			}

			x[0] = s_newton_raphson_binary_search(func, lhs, rhs);

			return 1;
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
				return 1;
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
				return 3;
			}
		}
	}

	// 
	// mem should be n * n
	auto ARIS_API s_polyn_solve(Size n, const double* k, double* x, double *mem, double zero_check) -> int {
		double* A = mem;

		if (n == 0) 
			return 0;
		
		if (std::abs(k[0]) < zero_check)
			return s_polyn_solve(n - 1, k + 1, x, mem, zero_check);

		if (n == 1) {
			x[0] = -k[1] / k[0];
			return 1;
		}

		if (n == 2) {
			return s_poly2_solve(k[0], k[1], k[2], x, zero_check);
		}
		
		s_vc(n, -1.0 / k[0], k + 1, A);
		s_fill(n - 1, n, 0.0, A + n);
		s_eye(n - 1, A + n, n);

		s_eigen(n, A, A, nullptr, zero_check);

		// find roots //
		int root_num = 0;
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

		return root_num;
	}


}
