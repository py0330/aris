#include <cmath>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <algorithm>
#include <limits>
#include <sstream>
#include <cstddef>
#include <array>
#include <list>

#include "aris/dynamic/math_matrix.hpp"
#include "aris/dynamic/screw.hpp"
#include "aris/dynamic/kinematics.hpp"


namespace aris::dynamic{
	auto s_put_into_period(double value, double which_period, double period)->double {
		// 获取当前值的
		auto ret = std::fmod(value, period);

		// 整数周期 
		auto t = std::trunc(which_period);
		// 取余的周期
		auto mod = which_period - t;

		while (ret > (mod + 0.5) * period) ret -= period;
		while (ret < (mod - 0.5) * period) ret += period;

		ret += t * period;

		return ret;
	}

	auto s_sov_pnts2pm(const double* origin, Size origin_ld, const double* first_pnt, Size first_ld, const double* second_pnt, Size second_ld, double* pm_out, const char* axis_order) noexcept->void {
		pm_out[12] = 0;
		pm_out[13] = 0;
		pm_out[14] = 0;
		pm_out[15] = 1;

		// 以下求解位置 //
		s_vc(3, origin, origin_ld, pm_out + 3, 4);

		// 以下求解角度 //
		int order[3]{ axis_order[0] - 'x', axis_order[1] - 'x', 3 + 'x' + 'x' - axis_order[0] - axis_order[1] };

		s_vc(3, first_pnt, first_ld, pm_out + order[0], 4);
		s_vc(3, second_pnt, second_ld, pm_out + order[1], 4);

		s_vs(3, origin, origin_ld, pm_out + order[0], 4);
		s_vs(3, origin, origin_ld, pm_out + order[1], 4);

		double alpha = (((order[1] - order[0] + 3) % 3) == 1) ? 1.0 : -1.0;

		s_c3(alpha, pm_out + order[0], 4, pm_out + order[1], 4, pm_out + order[2], 4);

		double nrm = s_norm(3, pm_out + order[2], 4);
		if (nrm == 0){
			if (s_norm(3, pm_out + order[0], 4) == 0){
				s_mc(3, 3, default_rm(), 3, pm_out, 4);
				return;
			}
			else{
				s_nv(3, 1.0 / s_norm(3, pm_out + order[0], 4), pm_out + order[0], 4);

				double rm[9];
				s_c3_n(3, pm_out + order[0], 4, default_rm(), 3, rm, 3);
				double norm[3];
				norm[0] = s_norm(3, rm, 3);
				norm[1] = s_norm(3, rm + 1, 3);
				norm[2] = s_norm(3, rm + 2, 3);

				Size max_id = std::max_element(norm, norm + 3) - norm;
				s_vc(3, 1.0 / norm[max_id], rm + max_id, 3, pm_out + order[1], 4);
				s_c3(alpha, pm_out + order[0], 4, pm_out + order[1], 4, pm_out + order[2], 4);
			}
		}
		else{
			s_nv(3, 1.0 / nrm, pm_out + order[2], 4);
			s_nv(3, 1.0 / s_norm(3, pm_out + order[0], 4), pm_out + order[0], 4);

			s_c3(alpha, pm_out + order[2], 4, pm_out + order[0], 4, pm_out + order[1], 4);
		}
	}
	auto s_sov_axes2pm(const double* origin, Size origin_ld, const double* first_axis, Size first_ld, const double* second_axis, Size second_ld, double* pm_out, const char* axis_order) noexcept->void {
		double origin_zero[3]{ 0,0,0 };

		s_sov_pnts2pm(origin_zero, 1, first_axis, first_ld, second_axis, second_ld, pm_out, axis_order);
		s_vc(3, origin, origin_ld, pm_out + 3, 4);
	}
	auto s_sov_theta(double k1, double k2, double b, double* theta_out)noexcept->int {
		double K = std::sqrt(k1 * k1 + k2 * k2);
		double rhs = b / K;

		if (std::abs(rhs) > 1.0) {
			return -1;
		}
		else if (std::abs(rhs) < 0.7) {
			double alpha_plus_theta = std::asin(rhs);
			double alpha = std::atan2(k2, k1);
			theta_out[0] = alpha_plus_theta - alpha;
			theta_out[1] = PI - alpha_plus_theta - alpha;
		}
		else {
			double alpha_plus_theta = std::acos(rhs);
			double alpha = std::atan2(-k1, k2);
			theta_out[0] = alpha_plus_theta - alpha;
			theta_out[1] = -alpha_plus_theta - alpha;
		}

		if (theta_out[0] > PI)theta_out[0] -= 2 * PI;
		if (theta_out[1] > PI)theta_out[1] -= 2 * PI;
		if (theta_out[0] < -PI)theta_out[0] += 2 * PI;
		if (theta_out[1] < -PI)theta_out[1] += 2 * PI;

		return 0;
	}
	auto s_sov_ab(const double* pp, double* ab, const char* order)noexcept->void {
		// 补充默认参数 //
		static const double default_pp[3]{ 1,0,0 };
		double default_ab[3];
		pp = pp ? pp : default_pp;
		ab = ab ? ab : default_ab;

		// 正式开始计算 //
		const Size a = Size(order[0]) - '1';
		const Size b = Size(order[1]) - '1';
		const Size c = 3 - a - b;
		const double pa = pp[a];
		const double pb = pp[b];
		const double pc = pp[c];
		const double Pbc = P()[b][c];
		const double Pac = P()[a][c];

		const double k = std::sqrt(pb * pb + pc * pc);

		ab[0] = std::atan2(Pbc * pb, pc);
		ab[1] = std::atan2(Pac * pa, k);
	}
	auto s_sov_vab(const double* pp, const double* vp, double* vab, double* ab, const char* order)noexcept->void {
		// 补充默认参数 //
		static const double default_pp[3]{ 1,0,0 };
		static const double default_vp[3]{ 0,0,0 };
		double default_vab[3], default_ab[3];
		pp = pp ? pp : default_pp;
		vp = vp ? vp : default_vp;
		vab = vab ? vab : default_vab;
		ab = ab ? ab : default_ab;

		// 正式开始计算 //
		const Size a = Size(order[0]) - '1';
		const Size b = Size(order[1]) - '1';
		const Size c = 3 - a - b;
		const double pa = pp[a];
		const double pb = pp[b];
		const double pc = pp[c];
		const double Pbc = P()[b][c];
		const double Pac = P()[a][c];

		const double k = std::sqrt(pb * pb + pc * pc);
		ab[0] = std::atan2(Pbc * pb, pc);
		ab[1] = std::atan2(Pac * pa, k);

		const double c1 = std::cos(ab[0]);
		const double c2 = std::cos(ab[1]);
		const double vpa = vp[a];
		const double vpb = vp[b];
		const double vpc = vp[c];

		const double vk = (pb * vpb + pc * vpc) / k;
		const double q1 = vpb * pc - vpc * pb;
		const double q2 = vpa * k - vk * pa;
		vab[0] = Pbc * q1 * c1 * c1 / (pc * pc);
		vab[1] = Pac * q2 * c2 * c2 / (k * k);
	}
	auto s_sov_aab(const double* pp, const double* vp, const double* ap, double* aab, double* vab, double* ab, const char* order)noexcept->void {
		// 补充默认参数 //
		static const double default_pp[3]{ 1,0,0 };
		static const double default_vp[3]{ 0,0,0 };
		static const double default_ap[3]{ 0,0,0 };
		double default_aab[3], default_vab[3], default_ab[3];
		pp = pp ? pp : default_pp;
		vp = vp ? vp : default_vp;
		ap = ap ? ap : default_ap;
		aab = vab ? aab : default_aab;
		vab = vab ? vab : default_vab;
		ab = ab ? ab : default_ab;

		// 正式开始计算 //
		const Size a = Size(order[0]) - '1';
		const Size b = Size(order[1]) - '1';
		const Size c = 3 - a - b;
		const double pa = pp[a];
		const double pb = pp[b];
		const double pc = pp[c];
		const double Pbc = P()[b][c];
		const double Pac = P()[a][c];

		const double k = std::sqrt(pb * pb + pc * pc);
		ab[0] = std::atan2(Pbc * pb, pc);
		ab[1] = std::atan2(Pac * pa, k);

		const double c1 = std::cos(ab[0]);
		const double c2 = std::cos(ab[1]);
		const double vpa = vp[a];
		const double vpb = vp[b];
		const double vpc = vp[c];

		const double vk = (pb * vpb + pc * vpc) / k;
		const double q1 = vpb * pc - vpc * pb;
		const double q2 = vpa * k - vk * pa;
		vab[0] = Pbc * q1 * c1 * c1 / (pc * pc);
		vab[1] = Pac * q2 * c2 * c2 / (k * k);

		const double s1 = std::sin(ab[0]);
		const double s2 = std::sin(ab[1]);
		const double apa = ap[a];
		const double apb = ap[b];
		const double apc = ap[c];
		const double ak = (pb * apb + vpb * vpb + pc * apc + vpc * vpc - vk * vk) / k;
		const double vq1 = apb * pc - apc * pb;
		const double vq2 = apa * k - ak * pa;

		aab[0] = Pbc * ((vq1 * c1 * c1 - 2 * q1 * c1 * s1 * vab[0]) * pc - 2 * vpc * q1 * c1 * c1) / (pc * pc * pc);
		aab[1] = Pac * ((vq2 * c2 * c2 - 2 * q2 * c2 * s2 * vab[1]) * k - 2 * vk * q2 * c2 * c2) / (k * k * k);
	}
	auto s_sov_ab_arbitrary(const double* pp0, const double* pp, double* alpha, double* beta, const char* order)noexcept->int {
		// 补充默认参数 //
		static const double default_pp[3]{ 1,0,0 };
		pp0 = pp0 ? pp0 : default_pp;
		pp = pp ? pp : default_pp;

		// 正式开始计算 //
		const Size a = Size(order[0]) - '1';
		const Size b = Size(order[1]) - '1';
		const Size c = 3 - a - b;
		const double xa = pp0[a];
		const double xb = pp0[b];
		const double xc = pp0[c];
		const double ya = pp[a];
		const double yb = pp[b];
		const double yc = pp[c];
		const double Pbc = P()[b][c];
		const double Pcb = P()[c][b];
		const double Pac = P()[a][c];
		const double Pca = P()[c][a];

		if (s_sov_theta(Pac * xc, xa, ya, beta))return -1;
		for (int i = 0; i < 2; ++i) {
			const auto s2 = std::sin(beta[i]);
			const auto c2 = std::cos(beta[i]);

			const auto k1 = Pbc * Pca * s2 * xa + Pbc * c2 * xc;
			const auto k2 = xb;
			const auto k3 = Pcb * xb;
			const auto k4 = Pca * s2 * xa + c2 * xc;

			// 符号会影响 atan2 的计算 //
			const auto sig = s_sgn2(k1 * k4 - k2 * k3);
			alpha[i] = std::atan2((k4 * yb - k2 * yc) * sig, (k1 * yc - k3 * yb) * sig);
		}

		return 0;
	}
	auto s_sov_axis_distance(const double* from_pm, const double* to_pm, Size axis)noexcept->double {
		if (axis < 3) {
			double dx{ to_pm[3] - from_pm[3] }, dy{ to_pm[7] - from_pm[7] }, dz{ to_pm[11] - from_pm[11] };
			return from_pm[axis] * dx + from_pm[axis + 4] * dy + from_pm[axis + 8] * dz;
		}
		else {
			Size b{ (axis - 2) % 3 }, c{ (axis - 1) % 3 };

			double Pbb = from_pm[b] * to_pm[b] + from_pm[b + 4] * to_pm[b + 4] + from_pm[b + 8] * to_pm[b + 8];
			double Pcc = from_pm[c] * to_pm[c] + from_pm[c + 4] * to_pm[c + 4] + from_pm[c + 8] * to_pm[c + 8];
			double Pbc = from_pm[b] * to_pm[c] + from_pm[b + 4] * to_pm[c + 4] + from_pm[b + 8] * to_pm[c + 8];
			double Pcb = from_pm[c] * to_pm[b] + from_pm[c + 4] * to_pm[b + 4] + from_pm[c + 8] * to_pm[b + 8];

			return std::atan2(Pcb - Pbc, Pbb + Pcc);
		}
	}


	auto s_calib_tool_two_pnts(const double* input, double* result, double mini_angle)noexcept->int {
		// check diff angle
		auto diff = input[5] - input[2];

		while (diff > aris::PI) diff -= 2 * aris::PI;
		while (diff < -aris::PI)diff += 2 * aris::PI;

		if (std::abs(diff) < mini_angle)return -1;

		auto c1 = std::cos(input[2]);
		auto s1 = std::sin(input[2]);
		auto c2 = std::cos(input[5]);
		auto s2 = std::sin(input[5]);

		const double A[4]{ c1 - c2, -s1 + s2, s1 - s2, c1 - c2 };
		const double tem = 1.0 / (A[0] * A[3] - A[1] * A[2]);

		double inv_A[4]{ A[3] * tem, -A[1] * tem, -A[2] * tem, A[0] * tem };

		double b[2]{ input[3] - input[0], input[4] - input[1] };
		s_mm(2, 1, 2, inv_A, b, result);
		return 0;
	}
}
