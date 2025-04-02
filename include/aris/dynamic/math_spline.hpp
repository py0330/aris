#ifndef ARIS_DYNAMIC_MATH_SPLINE_H_
#define ARIS_DYNAMIC_MATH_SPLINE_H_

#include <aris_lib_export.h>
#include <aris/core/basic_type.hpp>

namespace aris::dynamic{
	
	auto ARIS_API s_scurve_p2p(double T, double p0, double p1, double t_at, double *p_at, double* v_at, double* a_at)->void;
	auto ARIS_API s_scurve_v2v(double T, double v0, double v1, double t_at, double* p_at, double* v_at, double* a_at)->void;
	auto ARIS_API s_scurve_a2a(double T, double a0, double a1, double t_at, double* p_at, double* v_at, double* a_at)->void;
	
	
	
	
	auto ARIS_API s_akima(Size n, const double *x, const double *y, double *p1, double *p2, double *p3, double zero_check = 1e-10)->void;
	auto ARIS_API s_akima_at(Size n, const double *x, const double *y, const double *p1, const double *p2, const double *p3, double x_1, const char order = '0')->double;
}

#endif
