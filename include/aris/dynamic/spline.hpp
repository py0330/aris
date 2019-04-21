#ifndef ARIS_DYNAMIC_SPLINE_
#define ARIS_DYNAMIC_SPLINE_

#include <aris/core/basic_type.hpp>

namespace aris::dynamic
{
	auto s_akima(Size n, const double *x, const double *y, double *p1, double *p2, double *p3, double zero_check = 1e-10)->void;
	auto s_akima_at(Size n, const double *x, const double *y, const double *p1, const double *p2, const double *p3, double x_1, const char order = '0')->double;
}

#endif
