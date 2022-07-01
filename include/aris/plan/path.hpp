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
	auto inline s_blend_line_line_bezier3(double s, 
		const double *p0, const double* p1, const double* p2, 
		double *p, double *dp, double *d2p)noexcept->void 
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
	}

	// p0-p1 为直线
	// p1 位于圆弧上，center为圆心，axis垂直于圆弧所处平面
	auto inline s_blend_line_circle_bezier3(double s,
		const double* p0, const double* p1, const double* center, const double *axis, double theta,
		double* p, double* dp, double* d2p)noexcept->void
	{
		double p1_minus_p0[3]{ p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2] };
		
		// rx & ry // 
		double rx[3]{
			p1[0] - center[0],
			p1[1] - center[1],
			p1[2] - center[2],
		};
		double ry[3]{
			-axis[2] * rx[1] + axis[1] * rx[2],
			axis[2] * rx[0] - axis[0] * rx[2],
			-axis[1] * rx[0] + axis[0] * rx[1],
		};

		// line part
		double tem = s * s * s - 3 * s * s + 3 * s;
		double line_part[3]{
			p1_minus_p0[0] * tem + p0[0],
			p1_minus_p0[1] * tem + p0[1],
			p1_minus_p0[2] * tem + p0[2],
		};


		// circle part
		double s3t = s * s * s * theta;
		double co = std::cos(s3t);
		double si = std::sin(s3t);
		double circle_part[3]{
			center[0] + si * ry[0] + co * rx[0],
			center[1] + si * ry[1] + co * rx[1],
			center[2] + si * ry[2] + co * rx[2],
		};

		// p //
		p[0] = line_part[0] + circle_part[0] - p1[0];
		p[1] = line_part[1] + circle_part[1] - p1[1];
		p[2] = line_part[2] + circle_part[2] - p1[2];

		// dp //
		double j1 =  3 * s * s - 6*s +3;
		double j2 =  3 * s * s * theta * co;
		double j3 = -3 * s * s * theta * si;

		dp[0] = j1 * p1_minus_p0[0] + j2 * ry[0] + j3 * rx[0];
		dp[1] = j1 * p1_minus_p0[1] + j2 * ry[1] + j3 * rx[1];
		dp[2] = j1 * p1_minus_p0[2] + j2 * ry[2] + j3 * rx[2];

		// d2p //
		double k1 = 6 * s * theta;
		double k2 = 9 * s * s * s * s * theta * theta;
		double k3 = k1 * co - k2 * si;
		double k4 = -k2 * co - k1 * si;
		double k5 = 6 * s - 6;

		d2p[0] = k5 * p1_minus_p0[0] + k3 * ry[0] + k4 * rx[0];
		d2p[1] = k5 * p1_minus_p0[1] + k3 * ry[1] + k4 * rx[1];
		d2p[2] = k5 * p1_minus_p0[2] + k3 * ry[2] + k4 * rx[2];
	}

	// p1 两圆弧交点
	// c1 c2为圆心，ax1 ax2垂直于圆弧所处平面，theta1 theta2 为圆弧角度
	auto inline s_blend_circle_circle_bezier3(double s, const double* p1,
		const double* c1, const double* ax1, double theta1,
		const double* c2, const double* ax2, double theta2,
		double* p, double* dp, double* d2p)noexcept->void
	{
		// rx & ry // 
		double rx1[3]{
			p1[0] - c1[0],
			p1[1] - c1[1],
			p1[2] - c1[2],
		};
		double ry1[3]{
			-ax1[2] * rx1[1] + ax1[1] * rx1[2],
			 ax1[2] * rx1[0] - ax1[0] * rx1[2],
			-ax1[1] * rx1[0] + ax1[0] * rx1[1],
		};

		double rx2[3]{
			p1[0] - c2[0],
			p1[1] - c2[1],
			p1[2] - c2[2],
		};
		double ry2[3]{
			-ax2[2] * rx2[1] + ax2[1] * rx2[2],
			 ax2[2] * rx2[0] - ax2[0] * rx2[2],
			-ax2[1] * rx2[0] + ax2[0] * rx2[1],
		};

		// circle 1 & 2 
		double s3t1 = (s - 1)* (s - 1)* (s - 1) * theta1;
		double co1 = std::cos(s3t1);
		double si1 = std::sin(s3t1);


		double s3t2 = s*s*s * theta2;
		double co2 = std::cos(s3t2);
		double si2 = std::sin(s3t2);



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
		p[0] = circle1[0] + circle2[0] - p1[0];
		p[1] = circle1[1] + circle2[1] - p1[1];
		p[2] = circle1[2] + circle2[2] - p1[2];

		// dp //
		double m1 =  3 * (s - 1) * (s - 1) * theta1 * co1;
		double m2 = -3 * (s - 1) * (s - 1) * theta1 * si1;
		double m3 =  3 * s * s * theta2 * co2;
		double m4 = -3 * s * s * theta2 * si2;

		dp[0] = m1 * ry1[0] + m2 * rx1[0] + m3 * ry2[0] + m4 * rx2[0];
		dp[1] = m1 * ry1[1] + m2 * rx1[1] + m3 * ry2[1] + m4 * rx2[1];
		dp[2] = m1 * ry1[2] + m2 * rx1[2] + m3 * ry2[2] + m4 * rx2[2];

		// d2p //
		double j1 = 6 * (s - 1) * theta1;
		double k1 = 9 * (s - 1) * (s - 1) * (s - 1) * (s - 1) * theta1 * theta1;
		double j2 = 6 * s * theta2;
		double k2 = 9 * s * s * s * s * theta2 * theta2;

		double n1 =  j1 * co1 - k1 * si1;
		double n2 = -k1 * co1 - j1 * si1;
		double n3 =  j2 * co2 - k2 * si2;
		double n4 = -k2 * co2 - j2 * si2;

		
		d2p[0] = n1 * ry1[0] + n2 * rx1[0] + n3 * ry2[0] + n4 * rx2[0];
		d2p[1] = n1 * ry1[1] + n2 * rx1[1] + n3 * ry2[1] + n4 * rx2[1];
		d2p[2] = n1 * ry1[2] + n2 * rx1[2] + n3 * ry2[2] + n4 * rx2[2];
	}

	auto inline s_blend_quaternion_bezier3(double s, 
		const double* q0_input, const double* q1_input, const double* q2_input,
		double* q, double* dq, double* d2q)noexcept->void
	{
		// 将q1用单位四元数代替 //
		double q0[4], q2[4], inv_q1[4];
		aris::dynamic::s_inv_q(q1_input, inv_q1);
		aris::dynamic::s_q_dot_q(inv_q1, q0_input, q0);
		aris::dynamic::s_q_dot_q(inv_q1, q2_input, q2);

		// 将q0 和 q2 放到距离 q1较小的arc上
		if (q0[3] < 0)
			aris::dynamic::s_iv(4, q0);
		if (q2[3] < 0)
			aris::dynamic::s_iv(4, q2);


		////////////////////////  sin（theta0）= 0时 有问题 /////////
		double theta0 = std::acos(q0[3]);
		double theta2 = std::acos(q2[3]);
		double v0[3],v2[3];
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
			st1_s2* v0[0],
			st1_s2* v0[1],
			st1_s2* v0[2],
			ct1_s2
		};

		double t1_sct1_s2 = theta0 * (2 * s - 2) * ct1_s2;
		double t1_sst1_s2 = theta0 * (2 * s - 2) * st1_s2;
		double dqa[4]{
			t1_sct1_s2* v0[0],
			t1_sct1_s2* v0[1],
			t1_sct1_s2* v0[2],
			-t1_sst1_s2
		};

		double j1 =  theta0 * 2 * ct1_s2 - theta0 * (2 * s - 2) * t1_sst1_s2;
		double j2 = -theta0 * 2 * st1_s2 - theta0 * (2 * s - 2) * t1_sct1_s2;
		double d2qa[4]{
			j1* v0[0],
			j1* v0[1],
			j1* v0[2],
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
			tscts2* v2[0],
			tscts2* v2[1],
			tscts2* v2[2],
			tssts2
		};

		double k1 =  theta2 * 2 * cts2 - theta2 * 2 * s * tssts2;
		double k2 = -theta2 * 2 * sts2 - theta2 * 2 * s * tscts2;

		double d2qb2[4]{
			k1* v2[0],
			k1* v2[1],
			k1* v2[2],
			k2
		};

		// qb3 dqb3 d2qb3 // 
		
		// 
		// 
		//// p //
		//p[0] = circle1[0] + circle2[0] - p1[0];
		//p[1] = circle1[1] + circle2[1] - p1[1];
		//p[2] = circle1[2] + circle2[2] - p1[2];

		//// dp //
		//double m1 = 3 * (s - 1) * (s - 1) * theta1 * co1;
		//double m2 = -3 * (s - 1) * (s - 1) * theta1 * si1;
		//double m3 = 3 * s * s * theta2 * co2;
		//double m4 = -3 * s * s * theta2 * si2;

		//dp[0] = m1 * ry1[0] + m2 * rx1[0] + m3 * ry2[0] + m4 * rx2[0];
		//dp[1] = m1 * ry1[1] + m2 * rx1[1] + m3 * ry2[1] + m4 * rx2[1];
		//dp[2] = m1 * ry1[2] + m2 * rx1[2] + m3 * ry2[2] + m4 * rx2[2];

		//// d2p //
		//double j1 = 6 * (s - 1) * theta1;
		//double k1 = 9 * (s - 1) * (s - 1) * (s - 1) * (s - 1) * theta1 * theta1;
		//double j2 = 6 * s * theta2;
		//double k2 = 9 * s * s * s * s * theta2 * theta2;

		//double n1 = j1 * co1 - k1 * si1;
		//double n2 = -k1 * co1 - j1 * si1;
		//double n3 = j2 * co2 - k2 * si2;
		//double n4 = -k2 * co2 - j2 * si2;


		//d2p[0] = n1 * ry1[0] + n2 * rx1[0] + n3 * ry2[0] + n4 * rx2[0];
		//d2p[1] = n1 * ry1[1] + n2 * rx1[1] + n3 * ry2[1] + n4 * rx2[1];
		//d2p[2] = n1 * ry1[2] + n2 * rx1[2] + n3 * ry2[2] + n4 * rx2[2];
	}

}

#endif