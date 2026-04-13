#ifndef ARIS_PLAN_PATH_H_
#define ARIS_PLAN_PATH_H_

#include <list>
#include <cmath>
#include <iostream>
#include <functional>
#include <map>
#include <any>

//#include <aris/core/core.hpp>
//#include <aris/control/control.hpp>
#include <aris/dynamic/dynamic.hpp>

/// \brief 轨迹规划命名空间
/// \ingroup aris
/// 
///
///
namespace aris::plan{

	// 使用3阶bezier曲线，控制点为：[p0, p1, p1, p2]
	auto ARIS_API s_bezier3_blend_line_line(double s,
		const double* p0, const double* p1, const double* p2,
		double* p, double* dp, double* d2p, double *d3p = nullptr)noexcept->void;

	// p0-p1 为直线
	// p1 位于圆弧上，center为圆心，axis垂直于圆弧所处平面
	auto ARIS_API s_bezier3_blend_line_circle(double s,
		const double* p0, const double* p1, const double* center, const double* axis, double theta,
		double* p, double* dp, double* d2p, double *d3p = nullptr)noexcept->void;

	// p1 两圆弧交点
	// c1 c2为圆心，ax1 ax2垂直于圆弧所处平面，theta1 theta2 为圆弧角度
	auto ARIS_API s_bezier3_blend_circle_circle(double s, const double* p1,
		const double* c1, const double* ax1, double theta1,
		const double* c2, const double* ax2, double theta2,
		double* p, double* dp, double* d2p, double *d3p = nullptr)noexcept->void;

	auto ARIS_API s_bezier3_blend_quaternion(double s,
		const double* q0_input, const double* q1_input, const double* q2_input,
		double* q, double* dq, double* d2q, double *d3p = nullptr)noexcept->void;

	auto ARIS_API s_bezier3_darc_ds(Size dim, const double* dp_ds_input, const double* d2p_ds2_input,
		double& darc_ds, double& d2arc_ds2, double& ds_darc, double& d2s_darc2)noexcept->void;

	auto ARIS_API s_bezier3_darc_ds(Size dim, const double* dp_ds_input, const double* d2p_ds2_input, const double* d3p_ds3_input,
		double& darc_ds, double& d2arc_ds2, double& d3arc_ds3, double& ds_darc, double& d2s_darc2, double& d3s_darc3)noexcept->void;

	// 计算某个点最大的可能速度，从而让加速度不超最大加速度
	auto ARIS_API s_bezier3_max_v_at(Size dim, const double* dp_ds_input, const double* d2p_ds2_input,
		double max_a, double& v)noexcept->void;

	// 计算某个点最大的可能速度，从而让加速度不超最大加速度，同时考虑jerk的限制
	auto ARIS_API s_bezier3_max_v_at(Size dim, const double* dp_ds_input, const double* d2p_ds2_input, const double* d3p_ds3_input,
		double max_a, double max_j, double& v)noexcept->void;

	struct EstimateBezierArcParam {
		double h;
		double A, B, C, D, E, F, G, H, I, X, Y, Z;
	};
	auto ARIS_API s_bezier3_estimate_arc_param(double darc0, double d2arc0, double darc1, double d2arc1, double darc50,
		EstimateBezierArcParam &bezier_arc_param)noexcept->void;

	auto ARIS_API s_bezier3_s2arc(double s, const EstimateBezierArcParam& param, double& arc, double& darc, double &d2arc)noexcept->void;

	auto ARIS_API s_bezier3_arc2s(double arc, double darc, double d2arc, const EstimateBezierArcParam& param, double& s, double &ds, double &d2s)noexcept->void;
}

#endif