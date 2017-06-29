#ifndef ARIS_DYNAMIC_SPLINE_
#define ARIS_DYNAMIC_SPLINE_

#ifndef PI
#define PI 3.141592653589793
#endif

#include <vector>
#include <map>
#include <string>
#include <memory>
#include <functional>
#include <algorithm>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <list>

#include <aris_core_basic_type.h>

namespace aris
{
	namespace dynamic
	{
		auto s_akima(Size n, const double *x, const double *y, double *p1, double *p2, double *p3, double zero_check = 1e-10)->void;
		auto s_akima_at(Size n, const double *x, const double *y, const double *p1, const double *p2, const double *p3, double x_1, const char order = '0')->double;
	}
}




























#endif
