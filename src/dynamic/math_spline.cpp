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

#include "aris/dynamic/math_spline.hpp"
#include "aris/dynamic/math_matrix.hpp"
#include "aris/dynamic/math_poly.hpp"
#include "aris/dynamic/math_interval.hpp"

namespace aris::dynamic{
	auto s_scurve_p2p(double T, double p0, double p1, double t_at, double* p_at, double* v_at, double* a_at) -> void {
		double j = 32 /(T*T*T) * (p1 - p0);
		double temp;

		if (t_at < T / 4) {
			*(p_at ? p_at : &temp) = p0 + j / 6 * t_at * t_at * t_at;
			*(v_at ? v_at : &temp) = j / 2 * t_at * t_at;
			*(a_at ? a_at : &temp) = j * t_at;
		}
		else if (t_at < T * 3 / 4) {
			*(p_at ? p_at : &temp) = p0 + j*((T*T*T)/192 - (T*T*t_at)/16 + (T*t_at*t_at)/4 - (t_at*t_at*t_at)/6);
			*(v_at ? v_at : &temp) = j*(-(T*T)/16 + (T*t_at)/2 - (t_at*t_at)/2);
			*(a_at ? a_at : &temp) = j*(T/2 - t_at);
		}
		else {
			*(p_at ? p_at : &temp) = p1 - j/6* (T - t_at)* (T - t_at)* (T - t_at);
			*(v_at ? v_at : &temp) = j/2*(T - t_at)*(T - t_at);
			*(a_at ? a_at : &temp) = -j*(T - t_at);
		}
	}
	auto s_scurve_v2v(double T, double v0, double v1, double t_at, double* p_at, double* v_at, double* a_at) -> void {
		
	}
	auto s_scurve_a2a(double T, double a0, double a1, double t_at, double* p_at, double* v_at, double* a_at) -> void {
		
	}
	
	auto ARIS_API s_interp_scurve(const double* u, const double* y, double u_at, double &y_at)->void {
		// 给定间隔，求解 dy_ds, d2y_ds2, d3y_ds3 在 2处的值
		//% syms x y0 y1 y2 y3 y4
		//% 
		//% syms T
		//% x0 = 0;
		//% x1 = T;
		//% x2 = 2*T;
		//% x3 = 3*T;
		//% x4 = 4*T;
		//% 
		//% A = [x0^4, x0^3, x0^2, x0, 1
		//%     x1^4, x1^3, x1^2, x1, 1
		//%     x2^4, x2^3, x2^2, x2, 1
		//%     x3^4, x3^3, x3^2, x3, 1
		//%     x4^4, x4^3, x4^2, x4, 1];
		//% 
		//% p = inv(A)*[y0;y1;y2;y3;y4];
		//% 
		//% k4 = p(1);
		//% k3 = p(2);
		//% k2 = p(3);
		//% k1 = p(4);
		//% k0 = p(5);
		//% 
		//% f = k4*x^4 + k3*x^3 + k2*x^2 + k1*x^1 + k0
		//% 
		//% % gives dy_at_x2
		//% df(x) = diff(f,x)
		//% collect(expand(df(x2)), T)
		//% 
		//% % gives d2y_at_x2
		//% d2f(x) = diff(df,x)
		//% collect(expand(d2f(x2)), T)
		//% 
		//% % gives d3y_at_x2
		//% d3f(x) = diff(d2f,x)
		//% collect(expand(d3f(x2)), T)
		
		auto cpt_diff = [](const double *y, double &dy, double &d2y)->void {
			dy = (y[0] - 8 * y[1] + 8 * y[3] - y[4]) /12;
			d2y = (-y[0] + 16 * y[1] - 30 * y[2] + 16 * y[3] - y[4])/12;
			//double d3y_at_x2 = (-y[0] + 2 * y[1] - 2 * y[3] + y[4])/2;
		};

		double du_ds_s2, d2u_ds2_s2, du_ds_s3, d2u_ds2_s3, dp_ds_s2, d2p_ds2_s2, dp_ds_s3, d2p_ds2_s3;
		
		cpt_diff(u, du_ds_s2, d2u_ds2_s2);
		cpt_diff(u + 1, du_ds_s3, d2u_ds2_s3);
		cpt_diff(y, dp_ds_s2, d2p_ds2_s2);
		cpt_diff(y + 1, dp_ds_s3, d2p_ds2_s3);

		//%%
		//% dx_dt   = 1/dt_dx
		//% d2x_dt2 = -1/(dt_dx)^2 * d2t_dx2 * dx_dt
		//%         = -d2t_dx2 / (dt_dx)^3
		//% d3x_dt3 = (3*(d2t_dx2)^2 - dt_dx * d3t_dx3) / (dt_dx)^5
		//
		//% 已知 p(s) u(s), 求 dp_du d2p_du2 d3p_du3
		//%
		//% dp_du   = dp_ds * ds_du = dp_ds / du_ds
		//% d2p_du2 = d2p_ds2 * ds_du^2 + dp_ds * d2s_du2
		//%         = d2p_ds2 / (du_ds^2) + dp_ds * d2s_du2
		//%         = (d2p_ds2 * du_ds - dp_ds * d2u_ds2)/(du_ds^3)
		//% d3p_du3 = (d3p_ds3*du_ds - dp_ds*d3u_ds2)*(du_ds^3) - (d2p_ds2*du_ds - dp_ds*d2u_ds2)*3*(du_ds^2)*d2u_ds2
		//%           /(du_ds^6)
		//% 
		//% 
		//% d3p_du3 = (d3p_ds3 * du_ds^2 - 3*d2p_ds2*du_ds*d2u_ds2 +
		//%           dp_ds*3*(d2u_ds2)^2 - du_ds * d3u_ds3)/(du_ds^5)
		double dp_du_2 = dp_ds_s2 / du_ds_s2;
		double d2p_du2_2 = (d2p_ds2_s2 * du_ds_s2 - dp_ds_s2 * d2u_ds2_s2) / (du_ds_s2* du_ds_s2* du_ds_s2);

		double dp_du_3 = dp_ds_s3 / du_ds_s3;
		double d2p_du2_3 = (d2p_ds2_s3 * du_ds_s3 - dp_ds_s3 * d2u_ds2_s3) / (du_ds_s3* du_ds_s3* du_ds_s3);

		//% clear
		//% syms T p0 v0 a0 p1 v1 a1 t_at
		//% 
		//% 
		//% j1 = (a1 - a0)/T;
		//% pa = 1.0/2*a0*t_at*t_at + 1.0/6*j1*t_at*t_at*t_at;
		//% va = a0*t_at + 1.0/2*j1*t_at*t_at;
		//% aa = a0 + j1*t_at;
		//% 
		//% pa_end = (a1/6+a0/3)*T*T;
		//% va_end = a0*T + 1.0/2*j1*T*T;
		//% 
		//% 
		//% vb1 = v1 - va_end;
		//% j2 = 4*(vb1 - v0)/T^2;
		//% pb_end = (T^3*j2)/8 + v0 * T;
		//% 
		//% pc1 = p1 - pa_end - pb_end;
		//% j3 = 32 /(T*T*T) * (pc1-p0);
		//%
		//%%% when t < T/4
		//% pb = j2/6*t_at*t_at*t_at + v0*t_at;
		//% vb = j2/2*t_at*t_at + v0;
		//% ab = j2*t_at;
		//% pc = p0 + j3/6*t_at*t_at*t_at;
		//% vc = j3/2*t_at*t_at;
		//% ac = j3*t_at;
		//%
		//%%% when t > T/4
		//% pb = j2/6*t_at*t_at*t_at + v0*t_at;
		//% vb = j2/2*t_at*t_at + v0;
		//% ab = j2*t_at;
		//% pc = p0 + j3*((T*T*T)/192 - (T*T*t_at)/16 + (T*t_at*t_at)/4 - (t_at*t_at*t_at)/6);
		//% vc = j3*(-(T*T)/16 + (T*t_at)/2 - (t_at*t_at)/2);
		//% ac = j3*(T/2 - t_at);
		//%
		//%%% when t < T*3/4
		//% pb = pb_end + 1.0/6*j2*(T-t_at)*(T-t_at)*(T-t_at) - (vb1)*(T - t_at);
		//% vb = -1.0/2*j2*(T-t_at)*(T-t_at) + vb1;
		//% ab = j2*(T-t_at);
		//% pc = p0 + j3*((T*T*T)/192 - (T*T*t_at)/16 + (T*t_at*t_at)/4 - (t_at*t_at*t_at)/6);
		//% vc = j3*(-(T*T)/16 + (T*t_at)/2 - (t_at*t_at)/2);
		//% ac = j3*(T/2 - t_at);
		//%
		//%%% when t < T
		//% pb = pb_end + 1.0/6*j2*(T-t_at)*(T-t_at)*(T-t_at) - (vb1)*(T - t_at);
		//% vb = -1.0/2*j2*(T-t_at)*(T-t_at) + vb1;
		//% ab = j2*(T-t_at);
		//% pc = pc1 - j3/6* (T - t_at)* (T - t_at)* (T - t_at);
		//% vc = j3/2*(T - t_at)*(T - t_at);
		//% ac = -j3*(T - t_at);
		//% 
		//%
		//%
		//%%%%
		//% p_at = pa + pb + pc
		//% v_at = va + vb + vc
		//% a_at = aa + ab + ac
		//%
		//% collect(p_at, t_at)
		//%
		//% then get k0 k1 k2 k3:
		//%
		//% if(t_at < T/4)
		//%     k0 = p0;
		//%     k1 = v0;
		//%     k2 = a0/2;
		//%     k3 = (a1*5/18-a0*17/18)/T + (p1-p0)*16/3/(T^3) - (v0*10/3+v1*2)/(T^2);
		//% elseif(t_at < T/2)
		//%     k0 = (p1/6+p0*5/6) - (v0+v1)/12*T + (a1-a0)/72*T^2;
		//%     k1 = (a0-a1)/6*T + (p0-p1)*2/T + 2*v0+v1;
		//%     k2 =  (a1*2/3-a0/6) + (p1-p0)*8/T^2 - (v0+v1)*4/T;
		//%     k3 = (-a0/18-a1*11/18)/T + (p0-p1)*16/3/T^3 + (v0*2+v1*10/3)/T^2;
		//% elseif(t_at < T*3/4)
		//%     k0 = (p0*5/6+p1/6) + (v1/12-v0/4)*T - (a0*7/72+a1*5/72)*T^2;
		//%     k1 = 3*v0 + (a0*2/3+a1/3)*T + (p0-p1)/T*2;
		//%     k2 = (p1-p0)*8/T^2 -a1/3-a0*7/6 - (6*v0+2*v1)/T;
		//%     k3 = (a0*11/18+a1/18)/T + (p0-p1)*16/3/(T^3) + (v0*10/3+v1*2)/(T^2);
		//% else
		//%     k0 = p0*16/3-p1*13/3 + (2*v0+v1*7/3)*T + (a0*5/18-a1*4/9)*T^2;
		//%     k1 = (a1*11/6-a0*5/6)*T -9*v1-6*v0 + (p1-p0)*16/T;
		//%     k2 = (a0*5/6-a1*7/3) + (p0-p1)*16/T^2 + (6*v0+v1*10)/T;
		//%     k3 = (17*a1-5*a0)/(18*T) + (p1-p0)*16/3/(T^3) - (v0*2+10*v1/3)/T^2;
		//% end
		//%
		//% which equals:
		//% [k0] = A * [p0]
		//% |k1|       |v0|
		//% |k2|       |a0|
		//% [k3]       |p1|
		//%            |v1|
		//%            [a1]
		//%
		//% where:
		//% if(t_at < T/4)
		//%     A = [1,0,0,0,0,0;]
		//%           
		//% elseif(t_at < T/2)
		//%     k0 = (p1/6+p0*5/6) - (v0+v1)/12*T + (a1-a0)/72*T^2;
		//%     k1 = (a0-a1)/6*T + (p0-p1)*2/T + 2*v0+v1;
		//%     k2 =  (a1*2/3-a0/6) + (p1-p0)*8/T^2 - (v0+v1)*4/T;
		//%     k3 = (-a0/18-a1*11/18)/T + (p0-p1)*16/3/T^3 + (v0*2+v1*10/3)/T^2;
		//% elseif(t_at < T*3/4)
		//%     k0 = (p0*5/6+p1/6) + (v1/12-v0/4)*T - (a0*7/72+a1*5/72)*T^2;
		//%     k1 = 3*v0 + (a0*2/3+a1/3)*T + (p0-p1)/T*2;
		//%     k2 = (p1-p0)*8/T^2 -a1/3-a0*7/6 - (6*v0+2*v1)/T;
		//%     k3 = (a0*11/18+a1/18)/T + (p0-p1)*16/3/(T^3) + (v0*10/3+v1*2)/(T^2);
		//% else
		//%     k0 = p0*16/3-p1*13/3 + (2*v0+v1*7/3)*T + (a0*5/18-a1*4/9)*T^2;
		//%     k1 = (a1*11/6-a0*5/6)*T -9*v1-6*v0 + (p1-p0)*16/T;
		//%     k2 = (a0*5/6-a1*7/3) + (p0-p1)*16/T^2 + (6*v0+v1*10)/T;
		//%     k3 = (17*a1-5*a0)/(18*T) + (p1-p0)*16/3/(T^3) - (v0*2+10*v1/3)/T^2;
		//% end
		//% 
		//% 
		double u_in = u_at - u[2];

		double T = u[3] - u[2];
		double T2 = T * T;
		double T3 = T2 * T;

		const double pva23[6]{ y[2], dp_du_2, d2p_du2_2, y[3], dp_du_3, d2p_du2_3 };
		double k[4];

		if(u_in < T/4){
			const double A[24]{1, 0, 0,   0, 0, 0,
			 0, 1, 0,   0, 0, 0,
			 0, 0, 0.5, 0, 0, 0,
			 -16.0/3/T3, -10.0/3/T2, -17.0/18/T, 16.0/3/T3, -2.0/T2, 5.0/18/T};
			s_mm(4, 1, 6, A, pva23, k);
		}
		else if (u_in < T/2) {
			const double A[24]{ 5.0/6, -1.0/12*T, -1.0/72*T2,   1.0/6, -1.0/12*T, 1.0/72*T2,
			 2.0/T, 2.0, 1.0/6*T,   -2.0/T, 1, -1.0/6*T,
			 -8.0/T2, -4.0/T, -1.0/6, 8.0/T2, -4.0/T, 2.0/3,
			 16.0/3/T3, 2.0/T2, -1.0/18/T, -16.0/3/T3, 10.0/3/T2, -11.0/18/T};
			s_mm(4, 1, 6, A, pva23, k);
		}
		else if(u_in < T * 3 / 4) {
			const double A[24]{ 5.0/6, -1.0/4*T, -7.0/72*T2,   1.0/6, 1.0/12*T, -5.0/72*T2,
			 2.0/T, 3.0, 2.0/3*T,   -2.0/T, 0, 1.0/3*T,
			 -8.0/T2, -6.0/T, -7.0/6, 8.0/T2, -2.0/T, -1.0/3,
			 16.0/3/T3, 10.0/3/T2, 11.0/18/T, -16.0/3/T3, 2/T2, 1.0/18/T};
			s_mm(4, 1, 6, A, pva23, k);
		}
		else {
			const double A[24]{ 16.0/3, 2.0*T, 5.0/18*T2,   -13.0/3, 7.0/3*T, -4.0/9*T2,
			 -16.0/T, -6, -5.0/6*T,   16.0/T, -9.0, 11.0/6*T,
			 16.0/T2, 6.0/T, 5.0/6, -16.0/T2, 10.0/T, -7.0/3,
			 -16.0/3/T3, -2.0/T2, -5.0/18/T, 16.0/3/T3, -10.0/3/T2, 17.0/18/T};
			s_mm(4, 1, 6, A, pva23, k);
		}

		y_at = ((k[3]*u_in + k[2])*u_in + k[1])*u_in + k[0];
		//dy_at = 3*k3*u_in^2 + 2*k2*u_in + k1;
		//d2y_at = 6*k3*u_in + 2*k2;
	}

	auto ARIS_API s_interp_scurve_u5_range(const double* u, const double* p,
		double dp_min, double dp_max, double d2p_min, double d2p_max, double d3p_min, double d3p_max,
		double* u5_range)->void
	{
		//% 
		//% 引入虚拟的s0 s1 ... s6，其中间隔相同，即 ds 相同，这里设为 1
		//% s0  s1  s2  s3  s4  s5
		//% u0  u1  u2  u3  u4  u5
		//% p0  p1  p2  p3  p4  p5
		//%
		//% -------------------- PART 1 计算 dp_du d2p_du2 在s2 s3处的值--------------
		//%
		//% 于是：
		//% [  dp_ds_2  ] = B * [ p0 ]
		//% | d2p_ds2_2 |       | p1 |
		//% [ d3p_ds3_2 ]       | p2 |
		//%                     | p3 |
		//%                     [ p4 ]
		//%                
		//% 其中：
		//% B = [[ 1, -8,   0,  8, -1]/(12)
		//%     [-1, 16, -30, 16, -1]/(12)
		//%     [-1,  2,   0, -2,  1]/(2)];
		//%
		//% du、d2u、d3u也可按照上式求出，进一步求出 du3 d2u3 关于 u5 的表达式
		//% 
		//% du_ds_3   = c1 - u5/12
		//% d2u_ds2_3 = c3 - u5/12
		//%           = c3 + (-u5/12 + c1 - c1)
		//%           = c3 - c1 + du_ds_3 
		//%
		//% 其中：
		//% c1 = u1/12 - (2*u2)/3 + (2*u4)/3
		//% c3 = (4*u2)/3 - u1/12 - (5*u3)/2 + (4*u4)/3
		//%
		//% 进一步带入得到 p 和 u 的关系
		//%
		//% dp_du_3 = dp_ds_3 / du_ds_3
		//% d2p_du2_3 = (d2p_ds2_3 * du_ds_3 - dp_ds_3 * d2u_ds2_3)/(du_ds_3^3)
		//%
		//% =>
		//%
		//% dp_du_3   = C * y
		//% d2p_du2_3 = D * y^2 + E * y^3
		//% 
		//% 其中：
		//% y = 12/(12*c1 - u5);
		//% C = dp_ds_3;
		//% D = d2p_ds2_3 - dp_ds_3;
		//% E = dp_ds_3*(c1 - c3);
		//% 
		//% 另由于 y = 12/(8*(u4-u2) - (u5-u1))
		//% 因此 y 随 u 递增
		//% 其中 (u5 - u1) 约为 4倍 du, 8*(u4 - u2) 约为 16倍 du，du不应该在几个周期
		//% 内变化太大，因此强制限制 8*(u4-u2) - (u5-u1) > 0
		//% 此外 u5 > u4，综上：
		//% y 的取值范围为[ 12/(u1 + 7*u4 - 8*u2), inf ]
		//%
		//% -------------------- PART 2 对 s2 s3 之间的数据进行插值--------------
		//% 对 s2 s3 之间的数据进行插值，应有：
		//%
		//% p(u) = k3*u^3 + k2*u^2 + k1*u + k0
		//%
		//% 其中：
		//% 
		//% [ k0 ] = A * [    p_2    ]
		//% | k1 |       |  dp_du_2  |
		//% | k2 |       | d2p_du2_2 |
		//% [ k3 ]       |    p_3    |
		//%              |  dp_du_3  |
		//%              [ d2p_du2_3 ]
		//%
		//% =>
		//%
		//% [ k0 ] = W * [ y^3 ]
		//% | k1 |       | y^2 |
		//% | k2 |       |  y  |
		//% [ k3 ]       [  1  ]
		//%
		//% 其中:
		//% W = [a16*E, a16*D, C*a15, a13*d2p_du2_2+a12*dp_du_2+a11*p2+a14*p3]
		//%     |a26*E, a26*D, C*a25, a23*d2p_du2_2+a22*dp_du_2+a21*p2+a24*p3|
		//%     |a36*E, a36*D, C*a35, a33*d2p_du2_2+a32*dp_du_2+a31*p2+a34*p3|
		//%     [a46*E, a46*D, C*a45, a43*d2p_du2_2+a42*dp_du_2+a41*p2+a44*p3]
		//%
		//% v(u) = 3*k3*u^2 + 2*k2*u + k1
		//% a(u) = 6*k3*u + 2*k2
		//% j(u) = 6*k3
		//% 
		//% =>
		//% [ p(u,y) ] = U * W * [ y^3 ]
		//% | v(u,y) |           | y^2 |
		//% | a(u,y) |           |  y  |
		//% [ j(u,y) ]           [  1  ]
		//%
		//% 其中：
		//% U = [ 1, u, u^2,  u^3   ]
		//%     | 0, 1, 2*u,  3*u^2 |
		//%     | 0, 0, 2,    6*u   |
		//%     [ 0, 0, 0,    6     ]
		//% 
		//% -------------------- PART 3 求解 v,a,j的极值--------------
		//% ---- Part 3.1 求解 v 的极值 ----
		//% 对于 a,j 来说，其极值一定位于 u 的端点处，对于 v 来说，需要
		//% 比较端点处的 v 以及可能出现的中间的极值点.
		//% 
		//% v(u,y) = (i2*y^3 + j2*y^3 + k2*y + h2) * u^2
		//%        + (i1*y^3 + j1*y^3 + k1*y + h1) * u
		//%        + (i0*y^3 + j0*y^3 + k0*y + h0)
		//%        = f2 * u^2 + f1 * u + f0
		//%
		//% 当 v(u) 的极值点在 u 的区间内 [ ul, ur ]中时，有：
		//%
		//% ul < -f1/(2*f2) < ur
		//% dp_min < (4*f2*f0 - f1*f1)/(4*f2) < dp_max
		//%
		//% 等同于求解以下不等式
		//% 
		//% ieq1: poly_ieq(-0.5*ijkh1-ur*ijkh2,ijkh2)
		//% ieq2: poly_ieq( 0.5*ijkh1+ul*ijkh2,ijkh2)
		//% ieq3: poly_ieq( conv(ijkh2,ijkh0) - conv(ijkh1,ijkh1)/4 - ur*[zeros(1,3),ijkh2],ijkh2)
		//% ieq4: poly_ieq(-conv(ijkh2,ijkh0) + conv(ijkh1,ijkh1)/4 + ul*[zeros(1,3),ijkh2],ijkh2)
		//%
		//% 其中 
		//% ijkh0 = [i0,j0,k0,h0]
		//% ijkh1 = [i1,j1,k1,h1]
		//% ijkh2 = [i2,j2,k2,h2]
		//%
		//% ---- Part 3.2 求解 v 在某个 ur 处的值 ----
		//% 此时：
		//% v(u_at,y) = f2*u^2 + f1*u + f0
		//%           = q3*y^3 + q2*y^2 + q1*y + q0
		//%
		//% 其中
		//% q3 = (i2*ur*ur + i1*ur + i0);
		//% q2 = (j2*ur*ur + j1*ur + j0);
		//% q1 = (k2*ur*ur + k1*ur + k0);
		//% q0 = (h2*ur*ur + h1*ur + h0);
		//%
		//% 于是应有：
		//% ieq5: poly_ieq( [q3,q2,q1,q0-dp_max],1)
		//% ieq6: poly_ieq(-[q3,q2,q1,q0-dp_min],1)
		//%
		//% ---- Part 3.3 求解 a 在某个 ur 处的值 ----
		//% 此时：
		//% a(u_at,y) = 2*f2*u + f1
		//%           = q3*y^3 + q2*y^2 + q1*y + q0
		//%
		//% 其中
		//% q3 = (2*i2*ur + i1);
		//% q2 = (2*j2*ur + j1);
		//% q1 = (2*k2*ur + k1);
		//% q0 = (2*h2*ur + h1);
		//%
		//% 于是应有：
		//% ieq7: poly_ieq( [q3,q2,q1,q0-d2p_max],1)
		//% ieq8: poly_ieq(-[q3,q2,q1,q0-d2p_min],1)
		//%
		//% ---- Part 3.4 求解 j 在某个 ur 处的值 ----
		//% 此时：
		//% a(u_at,y) = 2*f2*u + f1
		//%           = q3*y^3 + q2*y^2 + q1*y + q0
		//%
		//% 其中
		//% q3 = 2*i2;
		//% q2 = 2*j2;
		//% q1 = 2*k2;
		//% q0 = 2*h2;
		//%
		//% 于是应有：
		//% ieq9 : poly_ieq( [q3,q2,q1,q0-d3p_max],1)
		//% ieq10: poly_ieq(-[q3,q2,q1,q0-d3p_min],1)

		// ------------------------ PART 1 ------------------------------------- //
		auto cpt_diff = [](const double* y, double& dy, double& d2y)->void {
			dy = (y[0] - 8 * y[1] + 8 * y[3] - y[4]) / 12;
			d2y = (-y[0] + 16 * y[1] - 30 * y[2] + 16 * y[3] - y[4]) / 12;
			//double d3y_at_x2 = (-y[0] + 2 * y[1] - 2 * y[3] + y[4])/2;
			};

		double du_ds_s2, d2u_ds2_s2, du_ds_s3, d2u_ds2_s3, dp_ds_s2, d2p_ds2_s2, dp_ds_s3, d2p_ds2_s3;

		cpt_diff(u, du_ds_s2, d2u_ds2_s2);
		//cpt_diff(u + 1, du_ds_s3, d2u_ds2_s3);
		cpt_diff(p, dp_ds_s2, d2p_ds2_s2);
		cpt_diff(p + 1, dp_ds_s3, d2p_ds2_s3);

		double dp_du_2 = dp_ds_s2 / du_ds_s2;
		double d2p_du2_2 = (d2p_ds2_s2 * du_ds_s2 - dp_ds_s2 * d2u_ds2_s2) / (du_ds_s2 * du_ds_s2 * du_ds_s2);

		double c1 = u[1] / 12 - (2 * u[2]) / 3 + (2 * u[4]) / 3;
		double c3 = (4 * u[2]) / 3 - u[1] / 12 - (5 * u[3]) / 2 + (4 * u[4]) / 3;

		double C = dp_ds_s3;
		double D = d2p_ds2_s3 - dp_ds_s3;
		double E = dp_ds_s3 * (c1 - c3);

		// ------------------------ PART 2 ------------------------------------- //
		double T = u[3] - u[2];
		double T2 = T * T;
		double T3 = T2 * T;

		const double A_content[4][4][6]{ { {1, 0, 0,   0, 0, 0,},
			{0, 1, 0,   0, 0, 0, },
			{0, 0, 0.5, 0, 0, 0,},
			{-16.0 / 3 / T3, -10.0 / 3 / T2, -17.0 / 18 / T, 16.0 / 3 / T3, -2.0 / T2, 5.0 / 18 / T} },
		
			{ {5.0 / 6, -1.0 / 12 * T, -1.0 / 72 * T2,   1.0 / 6, -1.0 / 12 * T, 1.0 / 72 * T2,},
			{2.0 / T, 2.0, 1.0 / 6 * T,   -2.0 / T, 1, -1.0 / 6 * T, },
			{-8.0 / T2, -4.0 / T, -1.0 / 6, 8.0 / T2, -4.0 / T, 2.0 / 3,},
			{16.0 / 3 / T3, 2.0 / T2, -1.0 / 18 / T, -16.0 / 3 / T3, 10.0 / 3 / T2, -11.0 / 18 / T } },

			{ { 5.0 / 6, -1.0 / 4 * T, -7.0 / 72 * T2,   1.0 / 6, 1.0 / 12 * T, -5.0 / 72 * T2,},
			{ 2.0 / T, 3.0, 2.0 / 3 * T,   -2.0 / T, 0, 1.0 / 3 * T, },
			{ -8.0 / T2, -6.0 / T, -7.0 / 6, 8.0 / T2, -2.0 / T, -1.0 / 3, },
			{ 16.0 / 3 / T3, 10.0 / 3 / T2, 11.0 / 18 / T, -16.0 / 3 / T3, 2 / T2, 1.0 / 18 / T } },

			{ { 16.0 / 3, 2.0 * T, 5.0 / 18 * T2,   -13.0 / 3, 7.0 / 3 * T, -4.0 / 9 * T2,},
			{ -16.0 / T, -6, -5.0 / 6 * T,   16.0 / T, -9.0, 11.0 / 6 * T, },
			{ 16.0 / T2, 6.0 / T, 5.0 / 6, -16.0 / T2, 10.0 / T, -7.0 / 3, },
			{-16.0 / 3 / T3, -2.0 / T2, -5.0 / 18 / T, 16.0 / 3 / T3, -10.0 / 3 / T2, 17.0 / 18 / T }, }
		};

		const double u_at_range[8]{ 0, T / 4,
						T / 4, T / 2,
						T / 2, T * 3 / 4,
						T * 3 / 4,T };


		constexpr double inf = std::numeric_limits<double>::infinity();
		double y_range[2]{12.0 / (u[1] + 7 * u[4] - 8 * u[2]), inf};

		// ------------------------ PART 3 ------------------------------------- //
		for (int i = 0; i < 4; ++i) {
			auto A = A_content[i];
			auto u_range = u_at_range + i * 2;
			auto ul = u_at_range[0];
			auto ur = u_at_range[1];

			double W[4][6]{ {A[0][5] * E, A[0][5] * D, C * A[0][4], A[0][2] * d2p_du2_2 + A[0][1] * dp_du_2 + A[0][0] * p[2] + A[0][3] * p[3]},
				{A[1][5] * E, A[1][5] * D, C * A[1][4], A[1][2] * d2p_du2_2 + A[1][1] * dp_du_2 + A[1][0] * p[2] + A[1][3] * p[3] },
				{A[2][5] * E, A[2][5] * D, C * A[2][4], A[2][2] * d2p_du2_2 + A[2][1] * dp_du_2 + A[2][0] * p[2] + A[2][3] * p[3] },
				{A[3][5] * E, A[3][5] * D, C * A[3][4], A[3][2] * d2p_du2_2 + A[3][1] * dp_du_2 + A[3][0] * p[2] + A[3][3] * p[3] } };

			// ------------------------ PART 3.1 ------------------------------------- //
			const double i2 = 3 * W[3][0];
			const double j2 = 3 * W[3][1];
			const double k2 = 3 * W[3][2];
			const double h2 = 3 * W[3][3];

			const double i1 = 2 * W[2][0];
			const double j1 = 2 * W[2][1];
			const double k1 = 2 * W[2][2];
			const double h1 = 2 * W[2][3];

			const double i0 = W[1][0];
			const double j0 = W[1][1];
			const double k0 = W[1][2];
			const double h0 = W[1][3];

			const double ijkh0[]{ i0, j0, k0, h0 };
			const double ijkh1[]{ i1, j1, k1, h1 };
			const double ijkh2[]{ i2, j2, k2, h2 };

			const double ijkh012[] = { i0, j0, k0, h0,
				i1, j1, k1, h1,
				i2, j2, k2, h2 };


			//% ieq1: poly_ieq(-0.5*ijkh1-ur*ijkh2,ijkh2)
			//% ieq2: poly_ieq( 0.5*ijkh1+ul*ijkh2,ijkh2)
			//% ieq3: poly_ieq( conv(ijkh2,ijkh0) - conv(ijkh1,ijkh1)/4 - ur*ijkh2,ijkh2)
			//% ieq4: poly_ieq(-conv(ijkh2,ijkh0) + conv(ijkh1,ijkh1)/4 + ul*ijkh2,ijkh2)
			const double zero_check = 1e-10;

			double f[7], g[4], mem[36], x[8], x2[8], x_result1_mem[100], x_result2_mem[100];
			aris::Size solution_num, solution_num2;

			auto x_result1 = x_result1_mem;
			auto x_result2 = x_result2_mem;

			//ieq1: poly_ieq(-W[2] - 3.0*ur*W[3], W[3])
			s_vi(4, W[2], f);
			s_va(4, -3.0*ur, W[3], f);
			s_vc(4, W[3], g);
			s_poly_ieq_solve(3, 3, f, g, &solution_num2, x_result2, mem, zero_check);

			std::cout << "ieq1---------------" << std::endl;
			dsp(solution_num2, 2, x_result2);

			//ieq2: poly_ieq(W[2] - 3.0*ur*W[3], W[3])
			s_vc(4, W[2], f);
			s_va(4, 3.0 * ul, W[3], f);
			//s_vc(4, W[3], g);   // g is same
			s_poly_ieq_solve(3, 3, f, g, &solution_num, x, mem, zero_check);
			std::swap(x_result1, x_result2);
			s_interval_intersect(solution_num, solution_num2, x, x_result1, solution_num2, x_result2);

			std::cout << "ieq2---------------" << std::endl;
			dsp(solution_num, 2, x);
			dsp(solution_num2, 2, x_result2);

			//ieq3: poly_ieq(W[2] - 3.0*ur*W[3], W[3])
			s_conv(3, 3, W[1], W[3], f);
			s_conv_add(3, 3, -1.0/3.0, W[2], W[2], f);
			s_va(4, -dp_max, W[3], f + 3);
			//s_vc(4, W[3], g);   // g is same
			s_poly_ieq_solve(6, 3, f, g, &solution_num, x, mem, zero_check);
			std::swap(x_result1, x_result2);
			s_interval_intersect(solution_num, solution_num2, x, x_result1, solution_num2, x_result2);

			std::cout << "ieq3---------------" << std::endl;
			dsp(solution_num, 2, x);
			dsp(solution_num, 2, x_result2);

			//s_interval_intersect(4, )

			//aris::dynamic::dsp(4, 2, x);


			//s_vc(3, -ur / 3.0, W[3], coeff1);
			//s_poly_ieq_solve(m, n, f, g, solution_num, x, mem, zero_check);

//eq1_ans = s_poly_ieq_solve(-0.5*ijkh1-ur*ijkh2,ijkh2);
//eq2_ans = s_poly_ieq_solve(0.5*ijkh1+ul*ijkh2,ijkh2);
//eq3_ans = s_poly_ieq_solve(conv(ijkh2,ijkh0) - conv(ijkh1,ijkh1)/4 - dp_max*[zeros(1,3),ijkh2],ijkh2);
//eq4_ans = s_poly_ieq_solve(-conv(ijkh2,ijkh0) + conv(ijkh1,ijkh1)/4 + dp_min*[zeros(1,3),ijkh2],ijkh2);
//
//eq_ans_has_ext = s_interval_intersect(eq1_ans, eq2_ans);
//eq_ans_not_has_ext = s_interval_inverse(eq_ans_has_ext);
//
//eq_ans_has_ext = s_interval_intersect(eq_ans_has_ext, eq3_ans);
//eq_ans_has_ext = s_interval_intersect(eq_ans_has_ext, eq4_ans);
//
//eq_ans = s_interval_union(eq_ans_not_has_ext,eq_ans_has_ext);


		}
	}

	auto s_akima(Size n, const double *x, const double *y, double *p1, double *p2, double *p3, double zero_check)->void	{
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
	auto s_akima_at(Size n, const double *x, const double *y, const double *p1, const double *p2, const double *p3, double xt, const char order)->double{
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
