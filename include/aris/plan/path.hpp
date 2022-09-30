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
		double* p, double* dp, double* d2p)noexcept->void;

	// p0-p1 为直线
	// p1 位于圆弧上，center为圆心，axis垂直于圆弧所处平面
	auto ARIS_API s_bezier3_blend_line_circle(double s,
		const double* p0, const double* p1, const double* center, const double* axis, double theta,
		double* p, double* dp, double* d2p)noexcept->void;

	// p1 两圆弧交点
	// c1 c2为圆心，ax1 ax2垂直于圆弧所处平面，theta1 theta2 为圆弧角度
	auto ARIS_API s_bezier3_blend_circle_circle(double s, const double* p1,
		const double* c1, const double* ax1, double theta1,
		const double* c2, const double* ax2, double theta2,
		double* p, double* dp, double* d2p)noexcept->void;

	auto ARIS_API s_bezier3_blend_quaternion(double s,
		const double* q0_input, const double* q1_input, const double* q2_input,
		double* q, double* dq, double* d2q)noexcept->void;

	auto ARIS_API s_bezier3_darc_ds(Size dim, const double* dp_ds_input, const double* d2p_ds2_input,
		double& darc_ds, double& d2arc_ds2, double& ds_darc, double& d2s_darc2)noexcept->void;

	// 计算某个点最大的可能速度，从而让加速度不超最大加速度
	auto inline ARIS_API s_bezier3_max_v_at(Size dim, const double* dp_ds_input, const double* d2p_ds2_input,
		double max_a, double& v)noexcept->void
	{
		//
		// 此外可以根据以下公式计算：
		//
		// dp_darc   = dp_ds * ds_darc
		// d2p_darc2 = d2p_ds2 * ds_darc^2 + dp_ds * d2s_darc2
		// 
		// dp_dt   = dp_darc * da_dt
		// d2p_dt2 = d2p_darc2 * darc_dt^2 + dp_darc * d2arc_dt2
		//
		// 因为 d2p_darc2 垂直与 dp_darc
		// 
		// a^2 = d2p_dt2^T * d2p_dt2
		//     = d2p_darc2^T * d2p_darc2 * darc_dt^4 + dp_darc^T*dp_darc * d2arc_dt2^2
		//     = k1 * darc_dt^4 + k2 * d2arc_dt2^2
		//
		// 假设曲线上的线速度大小不变，方向改变，求此时的 v, 即darc_dt
		// 
		// v = sqrt(a/sqrt(k1));
		

		double darc_ds, d2arc_ds2, ds_darc, d2s_darc2;
		s_bezier3_darc_ds(dim, dp_ds_input, d2p_ds2_input, darc_ds, d2arc_ds2, ds_darc, d2s_darc2);

		double dp_darc[4];
		aris::dynamic::s_vc(dim, ds_darc, dp_ds_input, dp_darc);

		double d2p_darc2[4]; 
		aris::dynamic::s_vc(dim, d2s_darc2, dp_ds_input, d2p_darc2);
		aris::dynamic::s_va(dim, ds_darc * ds_darc, d2p_ds2_input, d2p_darc2);

		double k1 = aris::dynamic::s_vv(dim, d2p_darc2, d2p_darc2);
		double k2 = aris::dynamic::s_vv(dim, dp_darc, dp_darc);
		

		v = darc_ds < 1e-7? std::numeric_limits<double>::max() : std::sqrt(max_a / std::sqrt(k1));
	}


	auto ARIS_API s_bezier3_estimate_arc_param(double darc0, double d2arc0, double darc1, double d2arc1, double darc50,
		double& a, double& b, double& c, double &d, double& e)noexcept->void;

	auto ARIS_API s_bezier3_s2arc(double s, double a, double b, double c, double d, double e, double& arc, double& darc, double &d2arc)noexcept->void;

	auto ARIS_API s_bezier3_arc2s(double arc, double a, double b, double c, double d, double e,	double& s)noexcept->void;
}

#endif