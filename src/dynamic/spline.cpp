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

#include "aris/dynamic/spline.hpp"

namespace aris::dynamic
{
	auto s_akima(Size n, const double *x, const double *y, double *p1, double *p2, double *p3, double zero_check)->void
	{
		// using p2 to store s //
		// using p3 to store ds //

		// p2_i is actually s_i+2, so s_i is actually p2_i-2
		for (Size i(-1); ++i < n - 1; )
		{
			p2[i] = (y[i + 1] - y[i]) / (x[i + 1] - x[i]);
		}
		double s1 = 2 * p2[0] - p2[1];
		double s0 = 2 * s1 - p2[0];
		double sn1 = 2 * p2[n - 2] - p2[n - 3];
		double sn2 = 2 * sn1 - p2[n - 2];


		// p3_i is actually ds_i+2, so ds_i is actually p3_i-2
		for (Size i(-1); ++i < n - 2; )
		{
			p3[i] = std::abs(p2[i + 1] - p2[i]);
		}
		double ds0 = std::abs(s1 - s0);
		double ds1 = std::abs(p2[0] - s1);
		double dsn = std::abs(sn1 - p2[n - 2]);
		double dsn1 = std::abs(sn2 - sn1);

		// p1 is actually t
		for (Size i(1); ++i < n - 2; )
		{
			p1[i] = (p3[i - 2] < zero_check && p3[i] < zero_check) ? (p2[i - 1] + p2[i]) / 2.0 : (p3[i] * p2[i - 1] + p3[i - 2] * p2[i]) / (p3[i - 2] + p3[i]);
		}
		p1[0] = (ds0 < zero_check && p3[0] < zero_check) ? (s1 + p2[0]) / 2.0 : (p3[0] * s1 + ds0 * p2[0]) / (ds0 + p3[0]);
		p1[1] = (ds1 < zero_check && p3[1] < zero_check) ? (p2[0] + p2[1]) / 2.0 : (p3[1] * p2[0] + ds1 * p2[1]) / (ds1 + p3[1]);
		p1[n - 2] = (p3[n - 4] < zero_check && dsn < zero_check) ? (p2[n - 3] + p2[n - 2]) / 2.0 : (dsn * p2[n - 3] + p3[n - 4] * p2[n - 2]) / (p3[n - 4] + dsn);
		double t = (p3[n - 3] < zero_check && dsn1 < zero_check) ? (p2[n - 2] + sn1) / 2.0 : (dsn1 * p2[n - 2] + p3[n - 3] * sn1) / (p3[n - 3] + dsn1);

		// p3
		for (Size i(-1); ++i < n - 2; )
		{
			p3[i] = (p1[i] + p1[i + 1] - 2 * p2[i]) / (x[i + 1] - x[i]) / (x[i + 1] - x[i]);
		}
		p3[n - 2] = (p1[n - 2] + t - 2 * p2[n - 2]) / (x[n - 1] - x[n - 2]) / (x[n - 1] - x[n - 2]);

		// p2
		for (Size i(-1); ++i < n - 2; )
		{
			p2[i] = (3 * p2[i] - 2 * p1[i] - p1[i + 1]) / (x[i + 1] - x[i]);
		}
		p2[n - 2] = (3 * p2[n - 2] - 2 * p1[n - 2] - t) / (x[n - 1] - x[n - 2]);
	}
	auto s_akima_at(Size n, const double *x, const double *y, const double *p1, const double *p2, const double *p3, double xt, const char order)->double
	{
		// 寻找第一个大于x的位置 //
		auto pos = std::upper_bound(x, x + n - 1, xt);
		Size id = pos == x ? 0 : pos - x - 1;

		double w = xt - x[id];

		switch (order)
		{
		case '1':
			return (3 * w*p3[id] + 2 * p2[id])*w + p1[id];
		case '2':
			return (6 * w*p3[id] + 2 * p2[id]);
		case '0':
		default:
			return ((w*p3[id] + p2[id])*w + p1[id])*w + y[id];
		}
	}
}
