#include "aris/plan/path.hpp"

#include <aris/dynamic/dynamic.hpp>

namespace aris::plan{
	// 使用3阶bezier曲线，控制点为：[p0, p1, p1, p2]
	auto s_bezier3_blend_line_line(double s,
		const double* p0, const double* p1, const double* p2,
		double* p, double* dp, double* d2p, double *d3p)noexcept->void
	{
		//p = (p2 - p0).*s. ^ 3 - 3 * (p1 - p0).*s. ^ 2 + 3 * (p1 - p0).*s + p0;
		//dp = 3 * (p2 - p0).*s. ^ 2 - 6 * (p1 - p0).*s + 3 * (p1 - p0);
		//ddp = 6 * (p2 - p0).*s - 6 * (p1 - p0);

		double p2_minus_p0[3]{
			p2[0] - p0[0],
			p2[1] - p0[1],
			p2[2] - p0[2],
		};

		double p1_minus_p0[3]{
			p1[0] - p0[0],
			p1[1] - p0[1],
			p1[2] - p0[2],
		};

		// make p //
		double j1 = s * s * s;
		double j2 = 3.0 * s - 3.0 * s * s;
		p[0] = p0[0] + j1 * p2_minus_p0[0] + j2 * p1_minus_p0[0];
		p[1] = p0[1] + j1 * p2_minus_p0[1] + j2 * p1_minus_p0[1];
		p[2] = p0[2] + j1 * p2_minus_p0[2] + j2 * p1_minus_p0[2];

		// make dp //
		double k1 = 3 - 6 * s;
		double k2 = 3 * s * s;
		dp[0] = k1 * p1_minus_p0[0] + k2 * p2_minus_p0[0];
		dp[1] = k1 * p1_minus_p0[1] + k2 * p2_minus_p0[1];
		dp[2] = k1 * p1_minus_p0[2] + k2 * p2_minus_p0[2];

		// make d2p //
		d2p[0] = -6 * p1_minus_p0[0] + 6 * s * p2_minus_p0[0];
		d2p[1] = -6 * p1_minus_p0[1] + 6 * s * p2_minus_p0[1];
		d2p[2] = -6 * p1_minus_p0[2] + 6 * s * p2_minus_p0[2];

		// make d3p //
		if (d3p) {
			d3p[0] = 6 * p2_minus_p0[0];
			d3p[1] = 6 * p2_minus_p0[1];
			d3p[2] = 6 * p2_minus_p0[2];
		}
	}

	// p0-p1 为直线
	// p1 位于圆弧上，center为圆心，axis垂直于圆弧所处平面
	auto s_bezier3_blend_line_circle(double s,
		const double* p0, const double* p1, const double* center, const double* axis, double theta,
		double* p, double* dp, double* d2p, double *d3p)noexcept->void
	{
		const double p1_minus_p0[3]{ p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2] };

		// rx & ry // 
		const double rx[3]{
			p1[0] - center[0],
			p1[1] - center[1],
			p1[2] - center[2],
		};
		const double ry[3]{
			-axis[2] * rx[1] + axis[1] * rx[2],
			axis[2] * rx[0] - axis[0] * rx[2],
			-axis[1] * rx[0] + axis[0] * rx[1],
		};

		// line part
		double k = s * s * s - 3 * s * s + 3 * s;
		double dk = 3 * s * s - 6 * s + 3;
		double d2k = 6 * s - 6;
		double d3k = 6;

		// circle part
		double s3t = s * s * s * theta;
		double ds3t = 3 * s * s * theta;
		double d2s3t = 6 * s * theta;
		double d3s3t = 6 * theta;

		double co = std::cos(s3t);
		double si = std::sin(s3t);
		double dco = -ds3t * si;
		double dsi = ds3t * co;
		double d2co = -d2s3t * si - ds3t * dsi;
		double d2si = d2s3t * co + ds3t * dco;
		double d3co = -d3s3t * si - 2 * d2s3t * dsi - ds3t * d2si;
		double d3si = d3s3t * co + 2 * d2s3t * dco + ds3t * d2co;

		// p //
		p[0] = p0[0] - p1[0] + k*p1_minus_p0[0] + si * ry[0] + co * rx[0];
		p[1] = p0[1] - p1[1] + k*p1_minus_p0[1] + si * ry[1] + co * rx[1];
		p[2] = p0[2] - p1[2] + k*p1_minus_p0[2] + si * ry[2] + co * rx[2];

		// dp //
		dp[0] = dk * p1_minus_p0[0] + dsi * ry[0] + dco * rx[0];
		dp[1] = dk * p1_minus_p0[1] + dsi * ry[1] + dco * rx[1];
		dp[2] = dk * p1_minus_p0[2] + dsi * ry[2] + dco * rx[2];

		// d2p //
		d2p[0] = d2k * p1_minus_p0[0] + d2si * ry[0] + d2co * rx[0];
		d2p[1] = d2k * p1_minus_p0[1] + d2si * ry[1] + d2co * rx[1];
		d2p[2] = d2k * p1_minus_p0[2] + d2si * ry[2] + d2co * rx[2];

		// make d3p //
		if (d3p) {
			d3p[0] = d3k * p1_minus_p0[0] + d3si * ry[0] + d3co * rx[0];
			d3p[1] = d3k * p1_minus_p0[1] + d3si * ry[1] + d3co * rx[1];
			d3p[2] = d3k * p1_minus_p0[2] + d3si * ry[2] + d3co * rx[2];
		}
	}

	// p1 两圆弧交点
	// c1 c2为圆心，ax1 ax2垂直于圆弧所处平面，theta1 theta2 为圆弧角度
	auto s_bezier3_blend_circle_circle(double s, const double* p1,
		const double* c1, const double* ax1, double theta1,
		const double* c2, const double* ax2, double theta2,
		double* p, double* dp, double* d2p, double *d3p)noexcept->void
	{
		// rx & ry // 
		const double rx1[3]{
			p1[0] - c1[0],
			p1[1] - c1[1],
			p1[2] - c1[2],
		};
		const double ry1[3]{
			-ax1[2] * rx1[1] + ax1[1] * rx1[2],
			 ax1[2] * rx1[0] - ax1[0] * rx1[2],
			-ax1[1] * rx1[0] + ax1[0] * rx1[1],
		};

		const double rx2[3]{
			p1[0] - c2[0],
			p1[1] - c2[1],
			p1[2] - c2[2],
		};
		const double ry2[3]{
			-ax2[2] * rx2[1] + ax2[1] * rx2[2],
			 ax2[2] * rx2[0] - ax2[0] * rx2[2],
			-ax2[1] * rx2[0] + ax2[0] * rx2[1],
		};

		// circle 1 & 2 
		double s3t1 = (s - 1) * (s - 1) * (s - 1) * theta1;
		double ds3t1 = -3 * (s - 1) * (s - 1) * theta1;
		double d2s3t1 = 6 * (s - 1) * theta1;
		double d3s3t1 = -6 * theta1;

		double co1 = std::cos(s3t1);
		double si1 = std::sin(s3t1);
		double dco1 = -ds3t1 * si1;
		double dsi1 = ds3t1 * co1;
		double d2co1 = -d2s3t1 * si1 - ds3t1 * dsi1;
		double d2si1 = d2s3t1 * co1 + ds3t1 * dco1;
		double d3co1 = -d3s3t1 * si1 - 2 * d2s3t1 * dsi1 - ds3t1 * d2si1;
		double d3si1 = d3s3t1 * co1 + 2 * d2s3t1 * dco1 + ds3t1 * d2co1;

		double s3t2 = s * s * s * theta2;
		double ds3t2 = 3 * s * s * theta2;
		double d2s3t2 = 6 * s * theta2;
		double d3s3t2 = 6 * theta2;

		double co2 = std::cos(s3t2);
		double si2 = std::sin(s3t2);
		double dco2 = -ds3t2 * si2;
		double dsi2 = ds3t2 * co2;
		double d2co2 = -d2s3t2 * si2 - ds3t2 * dsi2;
		double d2si2 = d2s3t2 * co2 + ds3t2 * dco2;
		double d3co2 = -d3s3t2 * si2 - 2 * d2s3t2 * dsi2 - ds3t2 * d2si2;
		double d3si2 = d3s3t2 * co2 + 2 * d2s3t2 * dco2 + ds3t2 * d2co2;



		double circle1[3]{
			c1[0] + si1 * ry1[0] + co1 * rx1[0],
			c1[1] + si1 * ry1[1] + co1 * rx1[1],
			c1[2] + si1 * ry1[2] + co1 * rx1[2],
		};

		double circle2[3]{
			c2[0] + si2 * ry2[0] + co2 * rx2[0],
			c2[1] + si2 * ry2[1] + co2 * rx2[1],
			c2[2] + si2 * ry2[2] + co2 * rx2[2],
		};

		// p //
		p[0] = c1[0] + c2[0] - p1[0] + si1*ry1[0] + co1 * rx1[0] + si2 * ry2[0] + co2 * rx2[0];
		p[1] = c1[1] + c2[1] - p1[1] + si1*ry1[1] + co1 * rx1[1] + si2 * ry2[1] + co2 * rx2[1];
		p[2] = c1[2] + c2[2] - p1[2] + si1*ry1[2] + co1 * rx1[2] + si2 * ry2[2] + co2 * rx2[2];

		// dp //
		dp[0] = dsi1 * ry1[0] + dco1 * rx1[0] + dsi2 * ry2[0] + dco2 * rx2[0];
		dp[1] = dsi1 * ry1[1] + dco1 * rx1[1] + dsi2 * ry2[1] + dco2 * rx2[1];
		dp[2] = dsi1 * ry1[2] + dco1 * rx1[2] + dsi2 * ry2[2] + dco2 * rx2[2];

		// d2p //
		d2p[0] = d2si1 * ry1[0] + d2co1 * rx1[0] + d2si2 * ry2[0] + d2co2 * rx2[0];
		d2p[1] = d2si1 * ry1[1] + d2co1 * rx1[1] + d2si2 * ry2[1] + d2co2 * rx2[1];
		d2p[2] = d2si1 * ry1[2] + d2co1 * rx1[2] + d2si2 * ry2[2] + d2co2 * rx2[2];

		// make d3p //
		if (d3p) {
			d3p[0] = d3si1 * ry1[0] + d3co1 * rx1[0] + d3si2 * ry2[0] + d3co2 * rx2[0];
			d3p[1] = d3si1 * ry1[1] + d3co1 * rx1[1] + d3si2 * ry2[1] + d3co2 * rx2[1];
			d3p[2] = d3si1 * ry1[2] + d3co1 * rx1[2] + d3si2 * ry2[2] + d3co2 * rx2[2];
		}
	}

	// 【注】：这里的 theta 并非转角，而是转角的 2 倍
	//
	//  要使用本函数，需保证 q[3] > 0
	auto inline s_rq_2_theta_v(const double* q, const double* dq, const double* ddq,
		double& theta, double* v, double& dtheta, double* dv, double& d2theta, double* d2v)noexcept->void
	{
		//
		// q   = [ q1 ] = [ s * v ]
		//       [ q2 ]   [ c     ]
		//
		// dq  = [dq1 ] = [ ds * v + s * dv ]
		//       [dq2 ]   [ dc              ]
		//
		// d2q = [d2q1] = [ d2s * v + 2 * ds * dv + s * d2v ]
		//       [d2q2] = [ d2c                             ]
		//
		// 其中：
		// s   =  sin(theta)
		// ds  =  cos(theta) * dtheta
		// d2s = -sin(theta) * dtheta^2 + cos(theta) * d2theta
		// c   =  cos(theta)
		// dc  = -sin(theta) * dtheta
		// d2c = -cos(theta) * dtheta^2 - sin(theta) * d2theta
		//
		// 且应该有：
		//   v'*v  = 1 
		//   v'*dv = 0  
		// 
		// 需求： theta dtheta d2theta v dv d2v
		//   
		//////////////////////////////////////////////////////////////////////////////////////////////
		//  
		// CASE 1：||q1|| > zero_check, 此时 theta 不接近 0
		//   【 theta 】 = atan2(||q1||, q2)
		//   【   v   】 = q1 / s
		//   【dtheta 】 = -dq2 / s
		//   【  dv   】 = (dq1 - ds * v) / s 
		//   【d2theta】 = -(d2q2 + c * dtheta^2)/s
		//   【  d2v  】 = (d2q1 - d2s * v - 2 * ds * dv)/s
		//
		//////////////////////////////////////////////////////////////////////////////////////////////
		// 
		// CASE 2：||q1|| <= zero_check && ||dq1|| > zero_check, 此时 theta 接近 0, 但 dtheta 不接近 0
		//   此时问题退化成：
		// q   = [ q1 ] = [ 0 ]
		//       [ q2 ]   [ c ]
		//
		// dq  = [dq1 ] = [ ds * v ]
		//       [dq2 ]   [ 0     ]
		//
		// d2q = [d2q1] = [ d2s * v + 2 * ds * dv ]
		//       [d2q2] = [ d2c                   ]
		// 
		//   其中：
		// s   =  0
		// ds  =  cos(theta) * dtheta
		// d2s =  cos(theta) * d2theta
		// c   =  cos(theta)
		// dc  =  0
		// d2c = -cos(theta) * dtheta^2
		// 
		//   【 theta 】 = atan2(||q1||, q2)
		//   dq1' * dq1 = ds^2
		//   【dtheta 】 = sqrt(dq1'*dq1/c^2)
		//   【   v   】 = dq1 / ds
		//   d2q1' * dq1 = d2s * ds
		//   【d2theta】 = (d2q1'* dq1)/ds/c
		//   【  dv   】 = (d2q1 - d2s * v)/(2*ds)
		//   【  d2v  】 = [0;0;0]
		// 
		////////////////////////////////////////////////////////////////////////////////////////////// 
		// 
		// CASE 3：||q1|| <= zero_check && ||dq1|| <= zero_check && ||d2q1|| > zero_check, 此时 theta 和 dtheta 接近 0，但 d2theta 不接近0 
		//   此时问题退化成：
		// q   = [ q1 ] = [ 0 ]
		//       [ q2 ]   [ c ]
		//
		// dq  = [dq1 ] = [ 0 ]
		//       [dq2 ]   [ 0 ]
		//
		// d2q = [d2q1] = [ d2s * v ]
		//       [d2q2] = [ 0       ]
		// 
		//   其中：
		// s   =  0
		// ds  =  0
		// d2s =  cos(theta) * d2theta
		// c   =  cos(theta)
		// dc  =  0
		// d2c =  0
		// 
		//   【 theta 】 = atan2(||q1||, q2)
		//   【dtheta 】 = sqrt(dq1'*dq1/c^2)
		//    d2q1'*d2q1 = d2s^2 = c^2 * d2theta^2
		//   【d2theta】 = sqrt(d2q1'*d2q1/c^2)
		//   【   v   】 = d2q1 / d2s
		//   【  dv   】 = [0;0;0]
		//   【  d2v  】 = [0;0;0]
		// 
		////////////////////////////////////////////////////////////////////////////////////////////// 
		// 
		// CASE 4：||q1|| <= zero_check && ||dq1|| <= zero_check && ||d2q1|| <= zero_check, 此时 theta 、 dtheta、 d2theta 都接近0 
		//   此时问题退化成：
		// q   = [ q1 ] = [ 0 ]
		//       [ q2 ]   [ c ]
		//
		// dq  = [dq1 ] = [ 0 ]
		//       [dq2 ]   [ 0 ]
		//
		// d2q = [d2q1] = [ d2s * v ]
		//       [d2q2] = [ 0       ]
		// 
		//   其中：
		// s   =  0
		// ds  =  0
		// d2s =  cos(theta) * d2theta
		// c   =  cos(theta)
		// dc  =  0
		// d2c =  0
		// 
		//   【 theta 】 = atan2(||q1||, q2)
		//   【dtheta 】 = sqrt(dq1'*dq1/c^2)
		//    d2q1'*d2q1 = d2s^2 = c^2 * d2theta^2
		//   【d2theta】 = sqrt(d2q1'*d2q1/c^2)
		//   【   v   】 = d2q1 / d2s
		//   【  dv   】 = [0;0;0]
		//   【  d2v  】 = [0;0;0]


		auto c = q[3];
		auto s = aris::dynamic::s_norm(3, q);
		theta = std::atan2(s, c);// s >= 0 保证了 theta >= 0

		// CASE 1 //
		if (theta > 1e-7) {
			aris::dynamic::s_vc(3, 1.0 / s, q, v);

			dtheta = -dq[3] / s;
			double ds = c * dtheta;
			dv[0] = (dq[0] - v[0] * ds) / s;
			dv[1] = (dq[1] - v[1] * ds) / s;
			dv[2] = (dq[2] - v[2] * ds) / s;

			d2theta = -(ddq[3] + c * dtheta * dtheta) / s;
			double d2s = -s * dtheta * dtheta + c * d2theta;
			d2v[0] = (ddq[0] - d2s * v[0] - 2 * ds * dv[0]) / (s);
			d2v[1] = (ddq[1] - d2s * v[1] - 2 * ds * dv[1]) / (s);
			d2v[2] = (ddq[2] - d2s * v[2] - 2 * ds * dv[2]) / (s);

			return;
		}

		// CASE 2 //
		dtheta = std::sqrt((dq[0] * dq[0] + dq[1] * dq[1] + dq[2] * dq[2]) / c / c);
		if (dtheta > 1e-7) {
			//   【   v   】 = dq1 / ds
			//   d2q1' * dq1 = d2s * ds
			//   【d2theta】 = (d2q1'* dq1)/ds/c
			//   【  dv   】 = (d2q1 - d2s * v)/(2*ds)
			//   【  d2v  】 = [0;0;0]

			double ds = c * dtheta;
			v[0] = dq[0] / ds;
			v[1] = dq[1] / ds;
			v[2] = dq[2] / ds;

			d2theta = (ddq[0] * dq[0] + ddq[1] * dq[1] + ddq[2] * dq[2]) / ds / c;
			double d2s = c * d2theta;
			dv[0] = (ddq[0] - d2s * v[0]) / (2 * ds);
			dv[1] = (ddq[1] - d2s * v[1]) / (2 * ds);
			dv[2] = (ddq[2] - d2s * v[2]) / (2 * ds);

			d2v[0] = 0.0;
			d2v[1] = 0.0;
			d2v[2] = 0.0;
			return;
		}

		d2theta = std::sqrt((ddq[0] * ddq[0] + ddq[1] * ddq[1] + ddq[2] * ddq[2]) / c / c);
		if (d2theta > 1e-7) {
			double d2s = c * d2theta;
			v[0] = ddq[0] / d2s;
			v[1] = ddq[1] / d2s;
			v[2] = ddq[2] / d2s;

			dv[0] = 0.0;
			dv[1] = 0.0;
			dv[2] = 0.0;

			d2v[0] = 0.0;
			d2v[1] = 0.0;
			d2v[2] = 0.0;
			return;
		}

		std::fill_n(v, 3, 0.0);
		std::fill_n(dv, 3, 0.0);
		std::fill_n(d2v, 3, 0.0);
	}


	auto s_bezier3_blend_quaternion(double s,
		const double* q0_input, const double* q1_input, const double* q2_input,
		double* q, double* dq, double* d2q, double *d3p)noexcept->void
	{
		// 将q1用单位四元数代替 //
		double q0[4], q2[4], inv_q1[4];
		aris::dynamic::s_inv_rq(q1_input, inv_q1);
		aris::dynamic::s_rq_dot_rq(inv_q1, q0_input, q0);
		aris::dynamic::s_rq_dot_rq(inv_q1, q2_input, q2);

		// 将q0 和 q2 放到距离 q1较小的arc上
		if (q0[3] < 0)
			aris::dynamic::s_iv(4, q0);
		if (q2[3] < 0)
			aris::dynamic::s_iv(4, q2);


		////////////////////////  sin（theta0）= 0时, v0 的计算似乎有问题 /////////
		double theta0 = std::atan2(std::sqrt(q0[0] * q0[0] + q0[1] * q0[1] + q0[2] * q0[2]), q0[3]);
		double theta2 = std::atan2(std::sqrt(q2[0] * q2[0] + q2[1] * q2[1] + q2[2] * q2[2]), q2[3]);
		double v0[3], v2[3];
		if (theta0 < 1e-7)
			aris::dynamic::s_vc(3, q0, v0);
		else
			aris::dynamic::s_vc(3, 1 / std::sin(theta0), q0, v0);

		if (theta2 < 1e-7)
			aris::dynamic::s_vc(3, q2, v2);
		else
			aris::dynamic::s_vc(3, 1 / std::sin(theta2), q2, v2);

		// qa dqa d2qa //
		double st1_s2 = std::sin(theta0 * (1 - s) * (1 - s));
		double ct1_s2 = std::cos(theta0 * (1 - s) * (1 - s));
		double qa[4]{
			st1_s2 * v0[0],
			st1_s2 * v0[1],
			st1_s2 * v0[2],
			ct1_s2
		};

		double t1_sct1_s2 = theta0 * (2 * s - 2) * ct1_s2;
		double t1_sst1_s2 = theta0 * (2 * s - 2) * st1_s2;
		double dqa[4]{
			t1_sct1_s2 * v0[0],
			t1_sct1_s2 * v0[1],
			t1_sct1_s2 * v0[2],
			-t1_sst1_s2
		};

		double j1 = theta0 * 2 * ct1_s2 - theta0 * (2 * s - 2) * t1_sst1_s2;
		double j2 = -theta0 * 2 * st1_s2 - theta0 * (2 * s - 2) * t1_sct1_s2;
		double d2qa[4]{
			j1 * v0[0],
			j1 * v0[1],
			j1 * v0[2],
			j2
		};

		// qb1 dqb1 d2qb1 //
		double qb1[4]{
			-qa[0],
			-qa[1],
			-qa[2],
			qa[3]
		};

		double dqb1[4]{
			-dqa[0],
			-dqa[1],
			-dqa[2],
			dqa[3]
		};

		double d2qb1[4]{
			-d2qa[0],
			-d2qa[1],
			-d2qa[2],
			d2qa[3]
		};

		// qb2 dqb2 d2qb2 //
		double sts2 = std::sin(theta2 * s * s);
		double cts2 = std::cos(theta2 * s * s);

		double qb2[4]{
			sts2 * v2[0],
			sts2 * v2[1],
			sts2 * v2[2],
			cts2
		};

		double tscts2 = theta2 * 2 * s * cts2;
		double tssts2 = theta2 * 2 * s * sts2;
		double dqb2[4]{
			tscts2 * v2[0],
			tscts2 * v2[1],
			tscts2 * v2[2],
			-tssts2
		};

		double k1 = theta2 * 2 * cts2 - theta2 * 2 * s * tssts2;
		double k2 = -theta2 * 2 * sts2 - theta2 * 2 * s * tscts2;

		double d2qb2[4]{
			k1 * v2[0],
			k1 * v2[1],
			k1 * v2[2],
			k2
		};

		// qb3 dqb3 d2qb3 // 
		double qb3[4], dqb3[4], d2qb3[4];
		aris::dynamic::s_rq_dot_rq(qb1, qb2, qb3);

		double tem4[4];
		aris::dynamic::s_rq_dot_rq(qb1, dqb2, dqb3);
		aris::dynamic::s_rq_dot_rq(dqb1, qb2, tem4);
		aris::dynamic::s_va(4, tem4, dqb3);

		aris::dynamic::s_rq_dot_rq(qb1, d2qb2, d2qb3);
		aris::dynamic::s_rq_dot_rq(d2qb1, qb2, tem4);
		aris::dynamic::s_va(4, tem4, d2qb3);
		aris::dynamic::s_rq_dot_rq(dqb1, dqb2, tem4);
		aris::dynamic::s_va(4, 2.0, tem4, d2qb3);

		// theta_b3 & vb3 //
		double theta_b3, vb3[3], dtheta_b3, dvb3[3], d2theta_b3, d2vb3[3];
		
		// 这里保证了 qb3[3] > 0, 因为 q1 为[0,0,0,1], 而 q0, q2 都是距离 q1 较近的四元数
		s_rq_2_theta_v(qb3, dqb3, d2qb3, theta_b3, vb3, dtheta_b3, dvb3, d2theta_b3, d2vb3);

		// qb3 dqb3 d2qb3 //
		double qb[4], dqb[4], d2qb[4];
		double stb3s = std::sin(theta_b3 * s);
		double ctb3s = std::cos(theta_b3 * s);

		qb[0] = stb3s * vb3[0];
		qb[1] = stb3s * vb3[1];
		qb[2] = stb3s * vb3[2];
		qb[3] = ctb3s;

		double l1 = ctb3s * (theta_b3 + s * dtheta_b3);
		double l2 = stb3s * (theta_b3 + s * dtheta_b3);

		dqb[0] = l1 * vb3[0] + stb3s * dvb3[0];
		dqb[1] = l1 * vb3[1] + stb3s * dvb3[1];
		dqb[2] = l1 * vb3[2] + stb3s * dvb3[2];
		dqb[3] = -l2;

		double m1 = -l2 * (theta_b3 + s * dtheta_b3) + ctb3s * (2.0 * dtheta_b3 + s * d2theta_b3);
		double m2 = 2.0 * l1;

		d2qb[0] = m1 * vb3[0] + m2 * dvb3[0] + stb3s * d2vb3[0];
		d2qb[1] = m1 * vb3[1] + m2 * dvb3[1] + stb3s * d2vb3[1];
		d2qb[2] = m1 * vb3[2] + m2 * dvb3[2] + stb3s * d2vb3[2];
		d2qb[3] = -l1 * (theta_b3 + s * dtheta_b3) - stb3s * (2.0 * dtheta_b3 + s * d2theta_b3);

		// q dq d2q //
		double q_[4], dq_[4], d2q_[4];


		aris::dynamic::s_rq_dot_rq(qa, qb, q_);

		aris::dynamic::s_rq_dot_rq(qa, dqb, dq_);
		aris::dynamic::s_rq_dot_rq(dqa, qb, tem4);
		aris::dynamic::s_va(4, tem4, dq_);

		aris::dynamic::s_rq_dot_rq(qa, d2qb, d2q_);
		aris::dynamic::s_rq_dot_rq(d2qa, qb, tem4);
		aris::dynamic::s_va(4, tem4, d2q_);
		aris::dynamic::s_rq_dot_rq(dqa, dqb, tem4);
		aris::dynamic::s_va(4, 2.0, tem4, d2q_);

		// make real q1 //
		aris::dynamic::s_rq_dot_rq(q1_input, q_, q);
		aris::dynamic::s_rq_dot_rq(q1_input, dq_, dq);
		aris::dynamic::s_rq_dot_rq(q1_input, d2q_, d2q);

		if (q[3] < 0) {
			aris::dynamic::s_iv(4, q);
			aris::dynamic::s_iv(4, dq);
			aris::dynamic::s_iv(4, d2q);
		}


	}

	// 计算某个点最大的可能速度，从而让加速度不超最大加速度
	auto ARIS_API s_bezier3_max_v_at(Size dim, const double* dp_ds_input, const double* d2p_ds2_input,
		double max_a, double& v)noexcept->void
	{
		// see below comments

		double darc_ds, d2arc_ds2, ds_darc, d2s_darc2;
		s_bezier3_darc_ds(dim, dp_ds_input, d2p_ds2_input, darc_ds, d2arc_ds2, ds_darc, d2s_darc2);

		double dp_darc[4];
		aris::dynamic::s_vc(dim, ds_darc, dp_ds_input, dp_darc);

		double d2p_darc2[4]; 
		aris::dynamic::s_vc(dim, d2s_darc2, dp_ds_input, d2p_darc2);
		aris::dynamic::s_va(dim, ds_darc * ds_darc, d2p_ds2_input, d2p_darc2);

		double k1 = aris::dynamic::s_vv(dim, d2p_darc2, d2p_darc2);
		double k2 = aris::dynamic::s_vv(dim, dp_darc, dp_darc);

		v = darc_ds < 1e-7 ? 0.0 : std::pow((max_a*max_a / k1), 0.25);
	}

	// 	// 计算某个点最大的可能速度，从而让加速度不超最大加速度
	auto ARIS_API s_bezier3_max_v_at(Size dim, const double* dp_ds_input, const double* d2p_ds2_input, const double* d3p_ds3_input,
		double max_a, double max_j, double& v)noexcept->void
	{
		//
		// 此外可以根据以下公式计算：
		//
		// dp_darc   = dp_ds * ds_darc
		// d2p_darc2 = d2p_ds2 * ds_darc^2 + dp_ds * d2s_darc2
		// d3p_darc3 = d3p_ds3 * ds_darc^3 + 3 * d2p_ds2 * ds_darc * d2s_darc2 + dp_ds * d3s_darc3
		// 
		// dp_dt   = dp_darc * darc_dt
		// d2p_dt2 = d2p_darc2 * darc_dt^2 + dp_darc * d2arc_dt2
		// d3p_dt3 = d3p_darc3 * darc_dt^3 + 3 * d2p_darc2 * darc_dt * d2arc_dt2 + dp_darc * d3arc_dt3
		// 
		// 因为 d2p_darc2 垂直于 dp_darc
		// 
		// ^T 表示转置
		//
		// a^2 = d2p_dt2^T * d2p_dt2
		//     = d2p_darc2^T * d2p_darc2 * darc_dt^4 + dp_darc^T*dp_darc * d2arc_dt2^2
		//     = k2 * darc_dt^4 + k1 * d2arc_dt2^2
		//
		// 假设曲线上的线速度大小不变，方向改变，求此时的 v, 即darc_dt
		// 
		// v = sqrt(a/sqrt(k2));
		//
		// j^2 = d3p_dt3^T * d3p_dt3
		//   = d3p_darc3^T * d3p_darc3 * darc_dt^6 + 9 * d2p_darc2^T * d2p_darc2 * darc_dt^2 * d2arc_dt2^2 + dp_darc^T * dp_darc * d3arc_dt3^2
		//   = d3p_darc3^T * d3p_darc3 * darc_dt^6 
		//     + 9 * d2p_darc2^T * d2p_darc2 * darc_dt^2 * d2arc_dt2^2 
		//     + dp_darc^T * dp_darc * d3arc_dt3^2
		//     + 6 * d2p_darc2^T * d3p_darc3 * darc_dt^4 * d2arc_dt2
		//     + 2 * dp_darc^T * d3p_darc3 * darc_dt^3 * d3arc_dt3
		//     + 6 * dp_darc^T * d2p_darc2 * darc_dt * d2arc_dt2 * d3arc_dt3
		//   = k3*darc_dt^6 + 9*k2*darc_dt^2*d2arc_dt2^2 + k1*d3arc_dt3^2 + 6*k6*darc_dt^4*d2arc_dt2 + 2*k5*darc_dt^3*d3arc_dt3 + 6*k4*darc_dt*d2arc_dt2*d3arc_dt3
		// 
		// 考虑到 k1 = 1，k4 = 0, 结束时, d2arc_dt2 = 0, d3arc_dt3 = 0 所以：
		//   
		// j^2 = k3*darc_dt^6
		//
		// where: 
		// k1 = dp_darc^T * dp_darc = 1
		// k2 = d2p_darc2^T * d2p_darc2
		// k3 = d3p_darc3^T * d3p_darc3
		// k4 = dp_darc^T * d2p_darc2 = 0
		// k5 = dp_darc^T * d3p_darc3
		// k6 = d2p_darc2^T * d3p_darc3
		//
		// 结束时，d2arc_dt2 = 0 所以：
		// v = min{ (max_a*max_a / k2)^(1/4)，(max_j*max_j/k3)^(1/6) }
		// 
		// 结束时，d3arc_dt3 = max_j，
		

		double darc_ds, d2arc_ds2, ds_darc, d2s_darc2, d3arc_ds3, d3s_darc3;
		s_bezier3_darc_ds(dim, dp_ds_input, d2p_ds2_input, d3p_ds3_input, darc_ds, d2arc_ds2, d3arc_ds3, ds_darc, d2s_darc2, d3s_darc3);

		double dp_darc[4];
		aris::dynamic::s_vc(dim, ds_darc, dp_ds_input, dp_darc);

		double d2p_darc2[4]; 
		aris::dynamic::s_vc(dim, d2s_darc2, dp_ds_input, d2p_darc2);
		aris::dynamic::s_va(dim, ds_darc * ds_darc, d2p_ds2_input, d2p_darc2);

		// d3p_darc3 = d3p_ds3 * ds_darc^3 + 3 * d2p_ds2 * ds_darc * d2s_darc2 + dp_ds * d3s_darc3
		double d3p_darc3[4];
		aris::dynamic::s_vc(dim, ds_darc * ds_darc * ds_darc, d3p_ds3_input, d3p_darc3);
		aris::dynamic::s_va(dim, 3 * ds_darc * d2s_darc2, d2p_ds2_input, d3p_darc3);
		aris::dynamic::s_va(dim, d3s_darc3, dp_ds_input, d3p_darc3);

		// k1 = 1.0 because dp_darc is unit vector
		double k1 = 1.0;
		double k2 = aris::dynamic::s_vv(dim, d2p_darc2, d2p_darc2);
		double k3 = aris::dynamic::s_vv(dim, d3p_darc3, d3p_darc3);

		// double k4 = aris::dynamic::s_vv(dim, dp_darc, d2p_darc2); // 因为 d2p_darc2 垂直于 dp_darc，所以 k4 = dp_darc^T*d2p_darc2 = 0
		// double k5 = aris::dynamic::s_vv(dim, dp_darc, d3p_darc3);
		// double k6 = aris::dynamic::s_vv(dim, d2p_darc2, d3p_darc3);

		v = darc_ds < 1e-7 ? 0.0 : std::min(std::pow((max_a*max_a / k2), 0.25), std::pow(((max_j*max_j) / k3), 0.16666666666666666));

	}

	auto s_bezier3_darc_ds(Size dim, const double* dp_ds_input, const double* d2p_ds2_input,
		double& darc_ds, double& d2arc_ds2, double& ds_darc, double& d2s_darc2)noexcept->void
	{
		darc_ds = aris::dynamic::s_norm(dim, dp_ds_input);

		d2arc_ds2 = darc_ds > 1e-8
			? aris::dynamic::s_vv(dim, dp_ds_input, d2p_ds2_input) / darc_ds
			: aris::dynamic::s_norm(dim, d2p_ds2_input);

		//
		// ds / dA = 1 / (dA / ds)
		// d2s / dA2 = -[d(dA / ds) / dA] / (dA / ds) ^ 2
		//			 = -[d2A / ds2 * ds / dA] / (dA / ds) ^ 2
		//			 = -(d2A / ds2) / (dA / ds) ^ 3

		ds_darc = 1 / darc_ds;
		d2s_darc2 = -d2arc_ds2 / darc_ds / darc_ds / darc_ds;
	}

	auto s_bezier3_darc_ds(Size dim, const double* dp_ds_input, const double* d2p_ds2_input, const double* d3p_ds3_input,
		double& darc_ds, double& d2arc_ds2, double& d3arc_ds3, double& ds_darc, double& d2s_darc2, double& d3s_darc3)noexcept->void
	{
		
		// darc_ds = sqrt(dp_ds*dp_ds)
		darc_ds = aris::dynamic::s_norm(dim, dp_ds_input);


		// d2arc_ds2 = (dp_ds·d2p_ds2) / darc_ds
		d2arc_ds2 = darc_ds > 1e-8
			? aris::dynamic::s_vv(dim, dp_ds_input, d2p_ds2_input) / darc_ds
			: aris::dynamic::s_norm(dim, d2p_ds2_input);

		// d3arc_ds3 = ((d2p_ds2·d2p_ds2 + dp_ds*d3p_ds3)*darc_ds - (dp_ds·d2p_ds2)*d2arc_ds2) / darc_ds^2
		if(darc_ds > 1e-8){
			double v_dot_j = aris::dynamic::s_vv(dim, dp_ds_input, d3p_ds3_input);
			double a_dot_a = aris::dynamic::s_vv(dim, d2p_ds2_input, d2p_ds2_input);
			double a_dot_v = aris::dynamic::s_vv(dim, d2p_ds2_input, dp_ds_input);
			d3arc_ds3 = ((a_dot_a + v_dot_j) * darc_ds - a_dot_v * d2arc_ds2)/ (darc_ds * darc_ds);
		}
		else{
			d3arc_ds3 = 0;
		}

		
		//
		// ds / dA = 1 / (dA / ds)
		// d2s / dA2 = -[d(dA / ds) / dA] / (dA / ds) ^ 2
		//			 = -[d2A / ds2 * ds / dA] / (dA / ds) ^ 2
		//			 = -(d2A / ds2) / (dA / ds) ^ 3

		if(darc_ds > 1e-8){
			ds_darc = 1 / darc_ds;
			d2s_darc2 = -d2arc_ds2 / darc_ds / darc_ds / darc_ds;
			d3s_darc3 = (3*d2arc_ds2*d2arc_ds2 - darc_ds*d3arc_ds3)/(std::pow(darc_ds,5));
		}
		else{
			ds_darc = 1e10;
			d2s_darc2 = 1e10;
			d3s_darc3 = 1e10;
		}
	}

	auto s_bezier3_estimate_arc_param(double darc0, double d2arc0, double darc1, double d2arc1, double darc50, EstimateBezierArcParam& param)noexcept->void{
		//% 对于 theta 较大时，以下式子估算较为准确
		//% 使用1元3次方程 a s^3 + b s^2 + c s + d  模拟在s=0和s=1处的dp
		//% [ 0 0 0 1 ] [ a ] = [ darc0  ] 
		//% | 1 1 1 1 | | b |   | darc1  |
		//% | 0 0 1 0 | | c |   | d2arc0 |
		//% [ 3 2 1 0 ] [ d ]   [ d2arc1 ]
		//% 
		//% 系数矩阵的逆为：
		//% [  2 -2  1  1 ]
		//% | -3  3 -2 -1 |
		//% |  0  0  1  0 | 
		//% [  1  0  0  0 ]
		//%
		//% 因而：
		//% a =  2*darc0 - 2*darc1 +   d2arc0 + d2arc1;
		//% b = -3*darc0 + 3*darc1 - 2*d2arc0 - d2arc1;
		//% c =                        d2arc0         ;
		//% d =    darc0                              ;
		//%
		//% 后续需要修正d，因此先计算前三项。

		double a =  2*darc0 - 2*darc1 +   d2arc0 + d2arc1;
		double b = -3*darc0 + 3*darc1 - 2*d2arc0 - d2arc1;
		double c =                        d2arc0         ;
		double d =    darc0                              ;

		//% 对于theta较小时，以下式子估算较为准确
		//% 用经验公式 atan2(x, h) *i 来模拟 d2arc, 其中 x = s - 0.5
		//% 对其积分，可得：
		//% darc = (x*atan2(x, h) - h/2*log(x*x + h*h))*i + j
		//%      = x*d2darc - (h/2*log(x*x + h*h))*i + j;
		//% 【参考】https://zh.m.wikipedia.org/zh-hant/%E5%8F%8D%E4%B8%89%E8%A7%92%E5%87%BD%E6%95%B0%E7%A7%AF%E5%88%86%E8%A1%A8
		//%
		//% 进一步积分，可得：
		//% arc = i*( (x*x+h*h)/2*atan2(x, h)-h*x/2 - (x*log(x*x + h*h)+2*h*atan2(x,h)-2*x)*h/2 ) + k
		//%     = k*( (x*x-h*h)*i*atan2(x, h))/2 + (h*i*x)/2 - (h*i*x*log(h^2 + x^2))/2
		//% 【参考】https://www.symbolab.com/solver/step-by-step/%5Cint%20ln%5Cleft(x%5E%7B2%7D%2Bc%5E%7B2%7D%5Cright)%20dx?or=input

		double h = std::pow((darc50 / (std::max(darc0 + darc1, std::numeric_limits<double>::epsilon()))) * 4.0 * aris::PI, 1.5) * 0.05;   //% 经验公式
		double i = (std::abs(d2arc1) + std::abs(d2arc0)) / 2.0 / std::atan2(1.0, 2.0 * h); //% 经验公式
		double j = -(0.5 * std::atan2(0.5, h) - h / 2 * std::log(0.25 + h * h)) * i + darc0; //% darc的常数项
		double k = -i * (h / 4.0 - (h * (1.0 + 2.0 * h * std::atan2(-0.25, h) - std::log(h * h + 0.25) / 2.0)) / 2.0 + std::atan2(-0.5, h) * (h * h / 2.0 + 0.125)) + 0.5 * j; //% arc 的常数项

		//% 上述两方程，无法保证s = 0.5时，darc的正确性，因此需修正
		//% 使用 cos 函数，在不改变0，1处的darc的前提下，将s=0.5处的darc修正到正确值
		//% 修正函数为 (1-cos(2 pi s))/2 * darc_error_at_0.5
		//% 而 darc_error_at_0.5 = dp50 - 0.125.*a - 0.25.*b - 0.5.*c - darc0;
		//% 
		//% 将上述等式化简，可得修正之后的一次项d，以及cos函数的系数 e：

		double ratio = std::pow(4.0 * darc50 / (darc0 + darc1), 4);
		ratio = std::isfinite(ratio) ? ratio : 0.5;
		ratio = std::max(ratio, 0.0);
		ratio = std::min(ratio, 1.0);

		double lh = std::max(std::log(h), -100.0);

		double darcA50 = 0.125 * a + 0.25 * b + 0.5 * c + d;
		double darcB50 = -lh * h * i + j;
		double darcE50 = darc50 - (ratio * darcA50 + (1.0 - ratio) * darcB50);

		param.A = ((a * ratio) / 4.0);
		param.B = ((b * ratio) / 3.0);
		param.C = (c * ratio) / 2.0;
		param.D = darc50 / 2.0 - (ratio - 1.0) * (j + (h * i) / 2.0) + d * ratio - (ratio * (a / 8.0 + b / 4.0 + c / 2.0 + d)) / 2.0 + ((j - h * i * lh) * (ratio - 1.0)) / 2.0;
		param.E = h * i / 2.0 * (ratio - 1.0);
		param.F = h * i / 2.0 * (1.0 - ratio) / 2.0;
		param.G = i * (1.0 - ratio) / 2.0;
		param.H = i * (h * h / 2.0 - 0.125) * (ratio - 1.0);
		param.I = -(darc50 - ratio * (a / 8.0 + b / 4.0 + c / 2.0 + d) + (j - h * i * lh) * (ratio - 1.0)) / (4.0 * aris::PI);
		param.h = h;

		param.X = -(param.F * std::log(0.25 + h * h) + param.H * std::atan2(-0.5, h)); //% 将 s = 0 带入，arc应该为0
		param.Y = param.D + (ratio - 1.0) * ((h * i) / 2.0);
		param.Z = c * ratio;
	}

	auto s_bezier3_s2arc(double s, const EstimateBezierArcParam& param,	double& arc, double& darc, double &d2arc)noexcept->void{
		double lx = std::max(std::log((s - 0.5) * (s - 0.5) + param.h * param.h), -100.0);
		double at = std::atan2(s - 0.5, param.h);
		
		arc = param.A * s * s * s * s + param.B * s * s * s + param.C * s * s + param.D * s + param.E * lx * s + param.F * lx + param.G * at * s * s - param.G * at * s + param.H * at + param.I * std::sin(2.0 * aris::PI * s) + param.X;
		darc = 4 * param.A * s * s * s + 3 * param.B * s * s + 2 * param.C * s + 2 * param.G * at * s - param.G * at + param.E * lx + param.I * 2 * aris::PI * std::cos(2 * aris::PI * s) + param.Y;
		d2arc = 12 * param.A * s * s + 6 * param.B * s + 2 * param.G * at + 4 * param.I * std::sin(2 * aris::PI * s) + param.Z;
	}

	
	auto s_bezier3_arc2s(double arc, double darc, double d2arc, const EstimateBezierArcParam& param, double& s, double& ds, double& d2s)noexcept->void {
		s = aris::dynamic::s_newton_raphson_binary_search([&param,arc](double s)->double {
			double lx = std::max(std::log((s - 0.5) * (s - 0.5) + param.h * param.h), -100.0);
			double at = std::atan2(s - 0.5, param.h);
			return param.A * s * s * s * s + param.B * s * s * s + param.C * s * s + param.D * s + param.E * lx * s + param.F * lx + param.G * at * s * s - param.G * at * s + param.H * at + param.I * std::sin(2.0 * aris::PI * s) + param.X - arc;
			}, 0.0, 1.0);

		double lx = std::max(std::log((s - 0.5) * (s - 0.5) + param.h * param.h), -100.0);
		double at = std::atan2(s - 0.5, param.h);

		double darc_ds = 4 * param.A * s * s * s + 3 * param.B * s * s + 2 * param.C * s + 2 * param.G * at * s - param.G * at + param.E * lx + param.I * 2 * aris::PI * std::cos(2 * aris::PI * s) + param.Y;
		double d2arc_ds2 = 12 * param.A * s * s + 6 * param.B * s + 2 * param.G * at + 4 * param.I * std::sin(2 * aris::PI * s) + param.Z;
		if (std::abs(darc_ds) < 1e-10) {
			ds = 0.0;
			d2s = 0.0;
		}
		else {
			ds = darc / darc_ds;
			d2s = (d2arc - d2arc_ds2 * ds * ds) / darc_ds;
		}
		
	}
}
