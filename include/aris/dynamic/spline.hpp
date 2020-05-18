#ifndef ARIS_DYNAMIC_SPLINE_H_
#define ARIS_DYNAMIC_SPLINE_H_

#include <aris_lib_export.h>
#include <aris/core/basic_type.hpp>

namespace aris::dynamic
{
	auto ARIS_API s_akima(Size n, const double *x, const double *y, double *p1, double *p2, double *p3, double zero_check = 1e-10)->void;
	auto ARIS_API s_akima_at(Size n, const double *x, const double *y, const double *p1, const double *p2, const double *p3, double x_1, const char order = '0')->double;
}

#endif
