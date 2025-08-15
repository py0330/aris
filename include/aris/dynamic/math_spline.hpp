#ifndef ARIS_DYNAMIC_MATH_SPLINE_H_
#define ARIS_DYNAMIC_MATH_SPLINE_H_

#include <aris_lib_export.h>
#include <aris/core/basic_type.hpp>

namespace aris::dynamic{
	
	// 
	// 用 s 曲线插值 y(u)
	// 
	// u，y 为 6维，并且 u 依次增加
	// u0  u1  u2  u_at   u3  u4  u5
	// y0  y1  y2         y3  y4  y5
	// 
	// u2 < u_at < u3
	// 
	// 
	// 
	auto ARIS_API s_interp_scurve(const double *x, const double *y, double x_at, double& y_at)->void;

	auto ARIS_API s_interp_scurve2(const double* x, const double* y, double x_at, double& y_at)->void;


	// 
	// u:  u0  u1  u2  u3  u4  u5 
	// y:  y0  y1  y2  y3  y4  y5
	// 
	// 根据 u2 u3 中 y 的限制，求 u5 可行的范围
	// u5_range_num 最大为 69，mem最大应为69*2=138
	auto ARIS_API s_interp_scurve_u5_range(const double* x, const double* y, 
		double dy_min, double dy_max, double d2y_min, double d2y_max, double d3y_min, double d3y_max,
		Size &u5_range_num,double *u5_range)->void;

	auto ARIS_API s_interp_scurve_u3_range(double min_du, const double* u, const double* p,
		double dp_min, double dp_max, double d2p_min, double d2p_max, double d3p_min, double d3p_max,
		Size& u3_range_num, double* u5_range)->void;

	auto ARIS_API s_scurve_p2p(double T, double p0, double p1, double t_at, double *p_at, double* v_at, double* a_at)->void;
	auto ARIS_API s_scurve_v2v(double T, double v0, double v1, double t_at, double* p_at, double* v_at, double* a_at)->void;
	auto ARIS_API s_scurve_a2a(double T, double a0, double a1, double t_at, double* p_at, double* v_at, double* a_at)->void;
	
	
	
	
	auto ARIS_API s_akima(Size n, const double *x, const double *y, double *p1, double *p2, double *p3, double zero_check = 1e-10)->void;
	auto ARIS_API s_akima_at(Size n, const double *x, const double *y, const double *p1, const double *p2, const double *p3, double x_1, const char order = '0')->double;
}

#endif
