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

#include "aris/dynamic/math_solver.hpp"
#include "aris/dynamic/math_matrix.hpp"

namespace aris::dynamic{

	auto s_newton_raphson_binary_search(std::function<double(double)> f, double x_below, double x_upper) -> double {
		double f_upper = f(x_upper);
		double f_below = f(x_below);

		double fsig = aris::dynamic::s_sgn2(f_upper - f_below);
		double xsig = aris::dynamic::s_sgn2(x_upper - x_below);

		if (aris::dynamic::s_sgn2(f_upper * f_below) >= 0)
			return std::abs(f_upper) < std::abs(f_below) ? x_upper : x_below;

		double diff = std::abs(x_upper - x_below);
		double diff_last = 10 * diff;

		while (diff < diff_last) {
			diff_last = diff;

			double x_mid = x_below + (x_upper - x_below) / 2;
			double f_mid = f(x_mid);

			if (aris::dynamic::s_sgn2(f_mid) == fsig) {
				x_upper = x_mid;
				f_upper = f_mid;
			}
			else {
				x_below = x_mid;
				f_below = f_mid;
			}

			double x1 = (x_mid * f_below - x_below * f_mid) / (f_below - f_mid);
			if (xsig * x1 <= xsig * x_upper && xsig * x1 >= xsig * x_below) {
				double fx1 = f(x1);
				if (aris::dynamic::s_sgn2(fx1) == fsig) {
					x_upper = x1;
					f_upper = fx1;
				}
				else {
					x_below = x1;
					f_below = fx1;
				}
			}

			double x2 = (x_mid * f_upper - x_upper * f_mid) / (f_upper - f_mid);
			if (xsig * x2 <= xsig * x_upper && xsig * x2 >= xsig * x_below) {
				double fx2 = f(x2);
				if (aris::dynamic::s_sgn2(fx2) == fsig) {
					x_upper = x2;
					f_upper = fx2;
				}
				else {
					x_below = x2;
					f_below = fx2;
				}

			}

			diff = std::abs(x_upper - x_below);
		}
		return (x_below + x_upper) / 2;
	}

	auto s_poly2_solve(double k2, double k1, double k0, double* x, double zero_check) -> int {
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

	}

	auto s_poly3_solve(double k3, double k2, double k1, double k0, double* x, double zero_check) -> int {
		if (std::abs(k3) < zero_check) {
			return s_poly2_solve(k2, k1, k0, x, zero_check);
		}
		
		
		// diff is:
		// 3*k3*x^2 + 2*k2*x + k1

		double A = 3 * k3;
		double B = 2 * k2;
		double C = k1;

		auto func = [k3, k2, k1, k0](double x) -> double {
			return k3 * x * x * x + k2 * x * x + k1 * x + k0;
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
				double last_bound = ext[0];
				double bound(last_bound - value);
				while (func(bound) * fe1 > 0) {
					value *= 2;
					bound -= value;
					last_bound = bound;
				}

				x[0] = s_newton_raphson_binary_search(func, std::min(bound, last_bound), std::max(bound, last_bound));
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
}
