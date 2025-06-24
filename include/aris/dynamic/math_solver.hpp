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
	auto ARIS_API s_newton_raphson_binary_search(std::function<double(double)> f, double x_below, double x_upper) -> double;
}

#endif
