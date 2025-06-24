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

	auto ARIS_API s_newton_raphson_binary_search(std::function<double(double)> f, double x_below, double x_upper) -> double {
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

}
