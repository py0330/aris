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

#include "aris/core/reflection.hpp"

#include "aris/dynamic/math_matrix.hpp"
#include "aris/dynamic/pose.hpp"
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
	auto s_put_near_value(double value, double current_value, double period)->double {
		if (std::isfinite(value) && std::isfinite(current_value) && std::isfinite(period) && period!=0) {
			period = std::abs(period);
			double diff_v = value - current_value;
			auto compensate_v = std::fmod(diff_v, period);
			return current_value + (std::abs(compensate_v) <= period/2 ? compensate_v : -s_sgn2(compensate_v)*(period - std::abs(compensate_v)));
		}
		return value;
	}
	auto s_put_into_range(double value, double period, double range_left, double range_right, double &result)->int {
		if (value <= range_right && value >= range_left) {
			result = value;
			return 0;
		}
			
		if (std::isfinite(value) && std::isfinite(period) && period != 0) {
			auto target = (period < range_right - range_left) 
				? value < range_left ? range_left + period / 2 : range_right - period / 2
				: (range_left + range_right) / 2;

			double diff_v = value - target;
			auto compensate_v = std::fmod(diff_v, period);
			result = target + (std::abs(compensate_v) <= period / 2 ? compensate_v : -s_sgn2(compensate_v) * (period - std::abs(compensate_v)));

			return range_left <= result && result <= range_right ? 0 : -1;
		}
		return -1;
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

	auto ARIS_API s_pos2pm(PosType type, const double* pos, double* pm)noexcept->void {
		switch (type) {
		case PosType::PE121:
			s_pe2pm(pos, pm, "121");
			return;
		case PosType::PE123:
			s_pe2pm(pos, pm, "123");
			return;
		case PosType::PE131:
			s_pe2pm(pos, pm, "131");
			return;
		case PosType::PE132:
			s_pe2pm(pos, pm, "132");
			return;
		case PosType::PE212:
			s_pe2pm(pos, pm, "212");
			return;
		case PosType::PE213:
			s_pe2pm(pos, pm, "213");
			return;
		case PosType::PE231:
			s_pe2pm(pos, pm, "231");
			return;
		case PosType::PE232:
			s_pe2pm(pos, pm, "232");
			return;
		case PosType::PE312:
			s_pe2pm(pos, pm, "313");
			return;
		case PosType::PE313:
			s_pe2pm(pos, pm, "313");
			return;
		case PosType::PE321:
			s_pe2pm(pos, pm, "321");
			return;
		case PosType::PE323:
			s_pe2pm(pos, pm, "323");
			return;
		case PosType::PQ:
			s_pq2pm(pos, pm);
			return;
		case PosType::PM:
			s_vc(16, pos, pm);
			return;
		case PosType::RE121:
			s_eye(4, pm);
			s_re2pm(pos, pm, "121");
			return;
		case PosType::RE123:
			s_eye(4, pm);
			s_re2pm(pos, pm, "123");
			return;
		case PosType::RE131:
			s_eye(4, pm);
			s_re2pm(pos, pm, "131");
			return;
		case PosType::RE132:
			s_eye(4, pm);
			s_re2pm(pos, pm, "132");
			return;
		case PosType::RE212:
			s_eye(4, pm);
			s_re2pm(pos, pm, "212");
			return;
		case PosType::RE213:
			s_eye(4, pm);
			s_re2pm(pos, pm, "213");
			return;
		case PosType::RE231:
			s_eye(4, pm);
			s_re2pm(pos, pm, "231");
			return;
		case PosType::RE232:
			s_eye(4, pm);
			s_re2pm(pos, pm, "232");
			return;
		case PosType::RE312:
			s_eye(4, pm);
			s_re2pm(pos, pm, "312");
			return;
		case PosType::RE313:
			s_eye(4, pm);
			s_re2pm(pos, pm, "313");
			return;
		case PosType::RE321:
			s_eye(4, pm);
			s_re2pm(pos, pm, "321");
			return;
		case PosType::RE323:
			s_eye(4, pm);
			s_re2pm(pos, pm, "323");
			return;
		case PosType::RQ:
			s_eye(4, pm);
			s_rq2pm(pos, pm);
			return;
		case PosType::RM:
			s_eye(4, pm);
			s_rm2pm(pos, pm);
			return;
		case PosType::XYZT:
			s_eye(4, pm);
			s_pp2pm(pos, pm);
			s_rmz(pos[3], pm, 4);
			return;
		case PosType::XYZ:
			s_eye(4, pm);
			s_pp2pm(pos, pm);
			s_rmz(pos[3], pm, 4);
			return;
		case PosType::RTZ:
			s_eye(4, pm);
			pm[3] = pos[0] * std::cos(pos[1]);
			pm[7] = pos[0] * std::sin(pos[1]);
			pm[11] = pos[2];
			s_rmz(pos[1], pm, 4);
			return;
		case PosType::XYT:
			s_eye(4, pm);
			pm[3] = pos[0];
			pm[7] = pos[1];
			s_rmz(pos[2], pm, 4);
			return;
		case PosType::XY:
			s_eye(4, pm);
			pm[3] = pos[0];
			pm[7] = pos[1];
			return;
		case PosType::RT:
			s_eye(4, pm);
			pm[3] = pos[0] * std::cos(pos[1]);
			pm[7] = pos[0] * std::sin(pos[1]);
			s_rmz(pos[1], pm, 4);
			return;
		case PosType::X:
			s_eye(4, pm);
			pm[3] = pos[0];
			return;
		case PosType::Y:
			s_eye(4, pm);
			pm[7] = pos[0];
			return;
		case PosType::Z:
			s_eye(4, pm);
			pm[10] = pos[0];
			return;
		case PosType::A:
			s_eye(4, pm);
			s_rmx(pos[0], pm, 4);
			return;
		case PosType::B:
			s_eye(4, pm);
			s_rmy(pos[0], pm, 4);
			return;
		case PosType::C:
			s_eye(4, pm);
			s_rmz(pos[0], pm, 4);
			return;
		case PosType::UNKNOWN:
			return;
		default:
			return;
		}
	}
	auto ARIS_API s_pm2pos(const double* pm, PosType type, double* pos)noexcept->void {
		switch (type) {
		case PosType::PE121:
			s_pm2pe(pm, pos, "121");
			return;
		case PosType::PE123:
			s_pm2pe(pm, pos, "123");
			return;
		case PosType::PE131:
			s_pm2pe(pm, pos, "131");
			return;
		case PosType::PE132:
			s_pm2pe(pm, pos, "132");
			return;
		case PosType::PE212:
			s_pm2pe(pm, pos, "212");
			return;
		case PosType::PE213:
			s_pm2pe(pm, pos, "213");
			return;
		case PosType::PE231:
			s_pm2pe(pm, pos, "231");
			return;
		case PosType::PE232:
			s_pm2pe(pm, pos, "232");
			return;
		case PosType::PE312:
			s_pm2pe(pm, pos, "312");
			return;
		case PosType::PE313:
			s_pm2pe(pm, pos, "313");
			return;
		case PosType::PE321:
			s_pm2pe(pm, pos, "321");
			return;
		case PosType::PE323:
			s_pm2pe(pm, pos, "323");
			return;
		case PosType::PQ:
			s_pm2pq(pm, pos);
			return;
		case PosType::PM:
			s_vc(16, pm, pos);
			return;
		case PosType::RE121:
			s_pm2re(pm, pos, "121");
			return;
		case PosType::RE123:
			s_pm2re(pm, pos, "123");
			return;
		case PosType::RE131:
			s_pm2re(pm, pos, "131");
			return;
		case PosType::RE132:
			s_pm2re(pm, pos, "132");
			return;
		case PosType::RE212:
			s_pm2re(pm, pos, "212");
			return;
		case PosType::RE213:
			s_pm2re(pm, pos, "213");
			return;
		case PosType::RE231:
			s_pm2re(pm, pos, "231");
			return;
		case PosType::RE232:
			s_pm2re(pm, pos, "232");
			return;
		case PosType::RE312:
			s_pm2re(pm, pos, "312");
			return;
		case PosType::RE313:
			s_pm2re(pm, pos, "313");
			return;
		case PosType::RE321:
			s_pm2re(pm, pos, "321");
			return;
		case PosType::RE323:
			s_pm2re(pm, pos, "323");
			return;
		case PosType::RQ:
			s_pm2rq(pm, pos);
			return;
		case PosType::RM:
			s_pm2rm(pm, pos);
			return;
		case PosType::XYZT:
			s_pm2pp(pm, pos);
			pos[3] = s_rmz_theta(pm, 4);
			return;
		case PosType::XYZ:
			s_pm2pp(pm, pos);
			return;
		case PosType::RTZ:
			pos[0] = std::sqrt(pm[3] * pm[3] + pm[7] * pm[7]);
			pos[1] = std::atan2(pm[3], pm[7]);
			pos[2] = pm[11];
			return;
		case PosType::XYT:
			pos[0] = pm[3];
			pos[1] = pm[7];
			pos[2] = s_rmz_theta(pm, 4);
			return;
		case PosType::XY:
			pos[0] = pm[3];
			pos[1] = pm[7];
			return;
		case PosType::RT:
			pos[0] = std::sqrt(pm[3] * pm[3] + pm[7] * pm[7]);
			pos[1] = std::atan2(pm[3], pm[7]);
			return;
		case PosType::X:
			pos[0] = pm[3];
			return;
		case PosType::Y:
			pos[0] = pm[7];
			return;
		case PosType::Z:
			pos[0] = pm[11];
			return;
		case PosType::A:
			pos[0] = s_rmx_theta(pm, 4);
			return;
		case PosType::B:
			pos[0] = s_rmy_theta(pm, 4);
			return;
		case PosType::C:
			pos[0] = s_rmz_theta(pm, 4);
			return;
		case PosType::UNKNOWN:
			return;
		default:
			return;
		}
	}
	auto ARIS_API s_pos2pos(PosType p1_t, const double* pos1, PosType p2_t, double* pos2)->void {
		if (p1_t == p2_t) {
			s_vc(s_pos_type_size(p1_t), pos1, pos2);
			return;
		}

		double pm[16];
		s_pos2pm(p1_t, pos1, pm);
		s_pm2pos(pm, p2_t, pos2);
	}
	auto ARIS_API s_pos2pos(aris::Size n, const PosType* p1_t, const double* p1, const PosType* p2_t, double* p2) -> void {
		aris::Size idx1{ 0 }, idx2{ 0 };
		for (aris::Size i = 0; i < n; ++i) {
			s_pos2pos(p1_t[i], p1 + idx1, p2_t[i], p2 + idx2);
			idx1 += aris::dynamic::s_pos_type_size(p1_t[i]);
			idx2 += aris::dynamic::s_pos_type_size(p2_t[i]);
		}
	}

	auto ARIS_API s_vel2vs(PosType p_t, const double* pos, VelType v_t, const double* vel, double* vs)noexcept->void {

		switch (v_t) {
		case VelType::VA: {
			double pp[3];
			s_pos2pos(p_t, pos, PosType::XYZ, pp);
			s_va2vs(pp, vel, vs);
			return;
		}
		case VelType::VS: {
			s_vc(6, vel, vs);
			return;
		}
		case VelType::VE121: {
			double pe[6];
			s_pos2pos(p_t, pos, PosType::PE121, pe);
			s_ve2vs(pe, vel, vs, "121");
			return;
		}
		case VelType::VE123: {
			double pe[6];
			s_pos2pos(p_t, pos, PosType::PE123, pe);
			s_ve2vs(pe, vel, vs, "123");
			return;
		}
		case VelType::VE131: {
			double pe[6];
			s_pos2pos(p_t, pos, PosType::PE131, pe);
			s_ve2vs(pe, vel, vs, "131");
			return;
		}
		case VelType::VE132: {
			double pe[6];
			s_pos2pos(p_t, pos, PosType::PE132, pe);
			s_ve2vs(pe, vel, vs, "132");
			return;
		}
		case VelType::VE212: {
			double pe[6];
			s_pos2pos(p_t, pos, PosType::PE212, pe);
			s_ve2vs(pe, vel, vs, "212");
			return;
		}
		case VelType::VE213: {
			double pe[6];
			s_pos2pos(p_t, pos, PosType::PE213, pe);
			s_ve2vs(pe, vel, vs, "213");
			return;
		}
		case VelType::VE231: {
			double pe[6];
			s_pos2pos(p_t, pos, PosType::PE231, pe);
			s_ve2vs(pe, vel, vs, "231");
			return;
		}
		case VelType::VE232: {
			double pe[6];
			s_pos2pos(p_t, pos, PosType::PE232, pe);
			s_ve2vs(pe, vel, vs, "232");
			return;
		}
		case VelType::VE312: {
			double pe[6];
			s_pos2pos(p_t, pos, PosType::PE312, pe);
			s_ve2vs(pe, vel, vs, "312");
			return;
		}
		case VelType::VE313: {
			double pe[6];
			s_pos2pos(p_t, pos, PosType::PE313, pe);
			s_ve2vs(pe, vel, vs, "313");
			return;
		}
		case VelType::VE321: {
			double pe[6];
			s_pos2pos(p_t, pos, PosType::PE321, pe);
			s_ve2vs(pe, vel, vs, "321");
			return;
		}
		case VelType::VE323: {
			double pe[6];
			s_pos2pos(p_t, pos, PosType::PE323, pe);
			s_ve2vs(pe, vel, vs, "323");
			return;
		}
		case VelType::VQ: {
			double pq[7];
			s_pos2pos(p_t, pos, PosType::PQ, pq);
			s_vq2vs(pq, vel, vs);
			return;
		}
		case VelType::VM: {
			double pm[16];
			s_pos2pos(p_t, pos, PosType::PM, pm);
			s_vm2vs(pm, vel, vs);
			return;
		}
		case VelType::WE121: {
			double re[3];
			s_pos2pos(p_t, pos, PosType::RE121, re);
			s_we2vs(re, vel, vs, "121");
			return;
		}
		case VelType::WE123: {
			double re[3];
			s_pos2pos(p_t, pos, PosType::RE123, re);
			s_we2vs(re, vel, vs, "123");
			return;
		}
		case VelType::WE131: {
			double re[3];
			s_pos2pos(p_t, pos, PosType::RE131, re);
			s_we2vs(re, vel, vs, "131");
			return;
		}
		case VelType::WE132: {
			double re[3];
			s_pos2pos(p_t, pos, PosType::RE132, re);
			s_we2vs(re, vel, vs, "132");
			return;
		}
		case VelType::WE212: {
			double re[3];
			s_pos2pos(p_t, pos, PosType::RE212, re);
			s_we2vs(re, vel, vs, "212");
			return;
		}
		case VelType::WE213: {
			double re[3];
			s_pos2pos(p_t, pos, PosType::RE213, re);
			s_we2vs(re, vel, vs, "213");
			return;
		}
		case VelType::WE231: {
			double re[3];
			s_pos2pos(p_t, pos, PosType::RE231, re);
			s_we2vs(re, vel, vs, "231");
			return;
		}
		case VelType::WE232: {
			double re[3];
			s_pos2pos(p_t, pos, PosType::RE232, re);
			s_we2vs(re, vel, vs, "232");
			return;
		}
		case VelType::WE312: {
			double re[3];
			s_pos2pos(p_t, pos, PosType::RE312, re);
			s_we2vs(re, vel, vs, "312");
			return;
		}
		case VelType::WE313: {
			double re[3];
			s_pos2pos(p_t, pos, PosType::RE313, re);
			s_we2vs(re, vel, vs, "313");
			return;
		}
		case VelType::WE321: {
			double re[3];
			s_pos2pos(p_t, pos, PosType::RE321, re);
			s_we2vs(re, vel, vs, "321");
			return;
		}
		case VelType::WE323: {
			double re[3];
			s_pos2pos(p_t, pos, PosType::RE323, re);
			s_we2vs(re, vel, vs, "323");
			return;
		}
		case VelType::WQ: {
			double rq[4];
			s_pos2pos(p_t, pos, PosType::RQ, rq);
			s_wq2vs(rq, vel, vs);
			return;
		}
		case VelType::WM: {
			double rm[9];
			s_pos2pos(p_t, pos, PosType::RM, rm);
			s_wm2vs(rm, vel, vs);
			return;
		}
		case VelType::DXYZT: {
			double p[6]{ 0.0 };
			s_pos2pos(p_t, pos, PosType::XYZT, p);
			s_ve2vs(p, vel, vs, "321");
			return;
		}
		case VelType::DXYZ: {
			double p[3];
			s_pos2pos(p_t, pos, PosType::XYZ, p);
			s_vp2vs(p, vel, vs);
			return;
		}
		case VelType::DRTZ: {
			// x = r*cos(t)
			// y = r*sin(t)
			//
			// dx = dr*cos(t) - r*sin(t)*dt
			// dy = dr*sin(t) + r*cos(t)*dt
			//
			
			// to va //
			double va[6]{ 0.0 };
			{
				double rtz[3];
				s_pos2pos(p_t, pos, PosType::RTZ, rtz);

				double& r = rtz[0];
				double& t = rtz[1];
				const double& dr = vel[0];
				const double& dt = vel[1];
				va[0] = dr * std::cos(t) - r * std::sin(t) * dt;
				va[1] = dr * std::sin(t) + r * std::cos(t) * dt;
				va[5] = dt;
			}

			// to vs //
			double pp[3];
			s_pos2pos(p_t, pos, PosType::XYZ, pp);
			s_va2vs(pp, va, vs);
			return;
		}
		case VelType::DXYT: {
			double va[6]{ vel[0], vel[1], 0,0,0, vel[2]};

			double pp[3];
			s_pos2pos(p_t, pos, PosType::XYZ, pp);
			s_va2vs(pp, va, vs);
			return;
		}
		case VelType::DXY: {
			double va[6]{ vel[0], vel[1], 0,0,0,0 };

			double pp[3];
			s_pos2pos(p_t, pos, PosType::XYZ, pp);
			s_va2vs(pp, va, vs);
			return;
		}
		case VelType::DRT: {
			// to va //
			double va[6]{ 0.0 };
			{
				double rtz[3];
				s_pos2pos(p_t, pos, PosType::RTZ, rtz);

				double& r = rtz[0];
				double& t = rtz[1];
				const double& dr = vel[0];
				const double& dt = vel[1];
				va[0] = dr * std::cos(t) - r * std::sin(t) * dt;
				va[1] = dr * std::sin(t) + r * std::cos(t) * dt;
			}

			// to vs //
			double pp[3];
			s_pos2pos(p_t, pos, PosType::XYZ, pp);
			s_va2vs(pp, va, vs);
			return;
		}
		case VelType::DX: {
			double va[6]{ vel[0],0,0,0,0,0 };
			
			// to vs //
			double pp[3];
			s_pos2pos(p_t, pos, PosType::XYZ, pp);
			s_va2vs(pp, va, vs);
			return;
		}
		case VelType::DY: {
			double va[6]{ 0,vel[0],0,0,0,0 };

			// to vs //
			double pp[3];
			s_pos2pos(p_t, pos, PosType::XYZ, pp);
			s_va2vs(pp, va, vs);
			return;
		}
		case VelType::DZ: {
			double va[6]{ 0,0,vel[0],0,0,0 };

			// to vs //
			double pp[3];
			s_pos2pos(p_t, pos, PosType::XYZ, pp);
			s_va2vs(pp, va, vs);
			return;
		}
		case VelType::DA: {
			std::fill_n(vs, 6, 0.0);
			vs[3] = vel[0];
			return;
		}
		case VelType::DB: {
			std::fill_n(vs, 6, 0.0);
			vs[4] = vel[0];
			return;
		}
		case VelType::DC: {
			std::fill_n(vs, 6, 0.0);
			vs[5] = vel[0];
			return;
		}
		case VelType::UNKNOWN:
			return;
		default:
			return;
		}
	
	}
	auto ARIS_API s_vs2vel(PosType p_t, const double* pos, const double* vs, VelType v_t, double* vel)noexcept->void {
		switch (v_t) {
		case VelType::VA: {
			double pp[3];
			s_pos2pos(p_t, pos, PosType::XYZ, pp);
			s_vs2va(vs, pp, vel);
			return;
		}
		case VelType::VS: {
			s_vc(6, vs, vel);
			return;
		}
		case VelType::VE121: {
			double pe[6];
			s_pos2pos(p_t, pos, PosType::PE121, pe);
			s_vs2ve(vs, pe, vel, "121");
			return;
		}
		case VelType::VE123: {
			double pe[6];
			s_pos2pos(p_t, pos, PosType::PE123, pe);
			s_vs2ve(vs, pe, vel, "123");
			return;
		}
		case VelType::VE131: {
			double pe[6];
			s_pos2pos(p_t, pos, PosType::PE131, pe);
			s_vs2ve(vs, pe, vel, "131");
			return;
		}
		case VelType::VE132: {
			double pe[6];
			s_pos2pos(p_t, pos, PosType::PE132, pe);
			s_vs2ve(vs, pe, vel, "132");
			return;
		}
		case VelType::VE212: {
			double pe[6];
			s_pos2pos(p_t, pos, PosType::PE212, pe);
			s_vs2ve(vs, pe, vel, "212");
			return;
		}
		case VelType::VE213: {
			double pe[6];
			s_pos2pos(p_t, pos, PosType::PE213, pe);
			s_vs2ve(vs, pe, vel, "213");
			return;
		}
		case VelType::VE231: {
			double pe[6];
			s_pos2pos(p_t, pos, PosType::PE231, pe);
			s_vs2ve(vs, pe, vel, "231");
			return;
		}
		case VelType::VE232: {
			double pe[6];
			s_pos2pos(p_t, pos, PosType::PE232, pe);
			s_vs2ve(vs, pe, vel, "232");
			return;
		}
		case VelType::VE312: {
			double pe[6];
			s_pos2pos(p_t, pos, PosType::PE312, pe);
			s_vs2ve(vs, pe, vel, "312");
			return;
		}
		case VelType::VE313: {
			double pe[6];
			s_pos2pos(p_t, pos, PosType::PE313, pe);
			s_vs2ve(vs, pe, vel, "313");
			return;
		}
		case VelType::VE321: {
			double pe[6];
			s_pos2pos(p_t, pos, PosType::PE321, pe);
			s_vs2ve(vs, pe, vel, "321");
			return;
		}
		case VelType::VE323: {
			double pe[6];
			s_pos2pos(p_t, pos, PosType::PE323, pe);
			s_vs2ve(vs, pe, vel, "323");
			return;
		}
		case VelType::VQ: {
			double pq[7];
			s_pos2pos(p_t, pos, PosType::PQ, pq);
			s_vs2vq(vs, pq, vel);
			return;
		}
		case VelType::VM: {
			double pm[16];
			s_pos2pos(p_t, pos, PosType::PM, pm);
			s_vs2vm(vs, pm, vel);
			return;
		}
		case VelType::WE121: {
			double re[3];
			s_pos2pos(p_t, pos, PosType::RE121, re);
			s_vs2we(vs, re, vel, "121");
			return;
		}
		case VelType::WE123: {
			double re[3];
			s_pos2pos(p_t, pos, PosType::RE123, re);
			s_vs2we(vs, re, vel, "123");
			return;
		}
		case VelType::WE131: {
			double re[3];
			s_pos2pos(p_t, pos, PosType::RE131, re);
			s_vs2we(vs, re, vel, "131");
			return;
		}
		case VelType::WE132: {
			double re[3];
			s_pos2pos(p_t, pos, PosType::RE132, re);
			s_vs2we(vs, re, vel, "132");
			return;
		}
		case VelType::WE212: {
			double re[3];
			s_pos2pos(p_t, pos, PosType::RE212, re);
			s_vs2we(vs, re, vel, "212");
			return;
		}
		case VelType::WE213: {
			double re[3];
			s_pos2pos(p_t, pos, PosType::RE213, re);
			s_vs2we(vs, re, vel, "213");
			return;
		}
		case VelType::WE231: {
			double re[3];
			s_pos2pos(p_t, pos, PosType::RE231, re);
			s_vs2we(vs, re, vel, "231");
			return;
		}
		case VelType::WE232: {
			double re[3];
			s_pos2pos(p_t, pos, PosType::RE232, re);
			s_vs2we(vs, re, vel, "232");
			return;
		}
		case VelType::WE312: {
			double re[3];
			s_pos2pos(p_t, pos, PosType::RE312, re);
			s_vs2we(vs, re, vel, "312");
			return;
		}
		case VelType::WE313: {
			double re[3];
			s_pos2pos(p_t, pos, PosType::RE313, re);
			s_vs2we(vs, re, vel, "313");
			return;
		}
		case VelType::WE321: {
			double re[3];
			s_pos2pos(p_t, pos, PosType::RE321, re);
			s_vs2we(vs, re, vel, "321");
			return;
		}
		case VelType::WE323: {
			double re[3];
			s_pos2pos(p_t, pos, PosType::RE323, re);
			s_vs2we(vs, re, vel, "323");
			return;
		}
		case VelType::WQ: {
			double rq[4];
			s_pos2pos(p_t, pos, PosType::RQ, rq);
			s_vs2wq(vs, rq, vel);
			return;
		}
		case VelType::WM: {
			double rm[9];
			s_pos2pos(p_t, pos, PosType::RM, rm);
			s_vs2wm(vs, rm, vel);
			return;
		}
		case VelType::DXYZT: {
			double p[6]{ 0.0 };
			s_pos2pos(p_t, pos, PosType::XYZT, p);
			s_vs2ve(vs, p, vel, "321");
			return;
		}
		case VelType::DXYZ: {
			double p[3];
			s_pos2pos(p_t, pos, PosType::XYZ, p);
			s_vs2vp(vs, p, vel);
			return;
		}
		case VelType::DRTZ: {
			// x = r*cos(t)
			// y = r*sin(t)
			//
			// dx = dr*cos(t) - r*sin(t)*dt
			// dy = dr*sin(t) + r*cos(t)*dt
			//

			// to va //
			double pp[3];
			s_pos2pos(p_t, pos, PosType::XYZ, pp);

			double va[6]{ 0.0 };
			s_vs2va(vs, pp, va);

			
			double rtz[3];
			s_pos2pos(p_t, pos, PosType::RTZ, rtz);
			double& r = rtz[0];
			double& t = rtz[1];

			if (rtz[0] < 1e-7) {
				double dr = std::sqrt(va[0] * va[0] + va[1] * va[1]);
				double dt = 0.0;
				vel[0] = dr;
				vel[1] = dt;
				vel[2] = va[2];
			}
			else {

				double A[4]{ cos(t), -r * sin(t), sin(t) , r * cos(t) };
				double det = A[0] * A[3] - A[1] * A[2];
				double inv_A[4]{
					A[3] / det, A[1] / det, A[2] / det, A[0] / det
				};

				vel[0] = inv_A[0] * va[0] + inv_A[1] * va[1];
				vel[1] = inv_A[2] * va[0] + inv_A[3] * va[1];
				vel[2] = va[2];
			}
			return;
		}
		case VelType::DXYT: {
			double va[6];
			double pp[3];
			s_pos2pos(p_t, pos, PosType::XYZ, pp);
			s_vs2va(vs, pp, va);

			vel[0] = va[0];
			vel[1] = va[1];
			vel[2] = va[5];
			return;
		}
		case VelType::DXY: {
			double va[6];
			double pp[3];
			s_pos2pos(p_t, pos, PosType::XYZ, pp);
			s_vs2va(vs, pp, va);

			vel[0] = va[0];
			vel[1] = va[1];
			return;
		}
		case VelType::DRT: {
			// x = r*cos(t)
			// y = r*sin(t)
			//
			// dx = dr*cos(t) - r*sin(t)*dt
			// dy = dr*sin(t) + r*cos(t)*dt
			//

			// to va //
			double pp[3];
			s_pos2pos(p_t, pos, PosType::XYZ, pp);

			double va[6]{ 0.0 };
			s_vs2va(vs, pp, va);


			double rt[2];
			s_pos2pos(p_t, pos, PosType::RT, rt);
			double& r = rt[0];
			double& t = rt[1];

			if (rt[0] < 1e-7) {
				double dr = std::sqrt(va[0] * va[0] + va[1] * va[1]);
				double dt = 0.0;
				vel[0] = dr;
				vel[1] = dt;
			}
			else {

				double A[4]{ cos(t), -r * sin(t), sin(t) , r * cos(t) };
				double det = A[0] * A[3] - A[1] * A[2];
				double inv_A[4]{
					A[3] / det, A[1] / det, A[2] / det, A[0] / det
				};

				vel[0] = inv_A[0] * va[0] + inv_A[1] * va[1];
				vel[1] = inv_A[2] * va[0] + inv_A[3] * va[1];
			}
			return;
		}
		case VelType::DX: {
			// to va //
			double pp[3];
			s_pos2pos(p_t, pos, PosType::XYZ, pp);

			double va[6]{ 0.0 };
			s_vs2va(vs, pp, va);
			vel[0] = va[0];
			return;
		}
		case VelType::DY: {
			// to va //
			double pp[3];
			s_pos2pos(p_t, pos, PosType::XYZ, pp);

			double va[6]{ 0.0 };
			s_vs2va(vs, pp, va);
			vel[0] = va[1];
			return;
		}
		case VelType::DZ: {
			// to va //
			double pp[3];
			s_pos2pos(p_t, pos, PosType::XYZ, pp);

			double va[6]{ 0.0 };
			s_vs2va(vs, pp, va);
			vel[0] = va[2];
			return;
		}
		case VelType::DA: {
			// to va //
			double pp[3];
			s_pos2pos(p_t, pos, PosType::XYZ, pp);

			double va[6]{ 0.0 };
			s_vs2va(vs, pp, va);
			vel[0] = va[3];
			return;
		}
		case VelType::DB: {
			// to va //
			double pp[3];
			s_pos2pos(p_t, pos, PosType::XYZ, pp);

			double va[6]{ 0.0 };
			s_vs2va(vs, pp, va);
			vel[0] = va[4];
			return;
		}
		case VelType::DC: {
			// to va //
			double pp[3];
			s_pos2pos(p_t, pos, PosType::XYZ, pp);

			double va[6]{ 0.0 };
			s_vs2va(vs, pp, va);
			vel[0] = va[5];
			return;
		}
		case VelType::UNKNOWN:
			return;
		default:
			return;
		}
	}
	auto ARIS_API s_vel2vel(PosType p1_t, const double* pos1, VelType v1_t, const double* vel1, VelType v2_t, double* vel2)->void {
		if (v1_t == v2_t) {
			s_vc(s_vel_type_size(v1_t), vel1, vel2);
			return;
		}

		double vs[6];
		s_vel2vs(p1_t, pos1, v1_t, vel1, vs);
		s_vs2vel(p1_t, pos1, vs, v2_t, vel2);
	}
	auto ARIS_API s_vel2vel(Size n, const PosType* p1_t, const double* p1, const VelType* v1_t, const double* v1, const VelType* v2_t, double* v2)->void {
		aris::Size pidx1{ 0 }, vidx1{ 0 }, vidx2{ 0 };
		for (aris::Size i = 0; i < n; ++i) {
			s_vel2vel(p1_t[i], p1 + pidx1, v1_t[i], v1 + vidx1, v2_t[i], v2 + vidx2);
			pidx1 += aris::dynamic::s_pos_type_size(p1_t[i]);
			vidx1 += aris::dynamic::s_vel_type_size(v1_t[i]);
			vidx2 += aris::dynamic::s_vel_type_size(v2_t[i]);
		}
	}

	auto ARIS_API s_acc2as(PosType p_t, const double* pos, VelType v_t, const double* vel, AccType a_t, const double* acc, double* as)noexcept->void {
		switch (a_t) {
		case AccType::AA: {
			double pp[3];
			s_pos2pos(p_t, pos, PosType::XYZ, pp);

			double va[6];
			s_vel2vel(p_t, pos, v_t, vel, VelType::VA, va);

			s_aa2as(pp, va, acc, as);
			return;
		}
		case AccType::AS: {
			s_vc(6, acc, as);
			return;
		}
		case AccType::AE121: {
			double pe[6];
			s_pos2pos(p_t, pos, PosType::PE121, pe);

			double ve[6];
			s_vel2vel(p_t, pos, v_t, vel, VelType::VE121, ve);

			s_ae2as(pe, ve, acc, as, nullptr, "121");
			return;
		}
		case AccType::AE123: {
			double pe[6];
			s_pos2pos(p_t, pos, PosType::PE123, pe);

			double ve[6];
			s_vel2vel(p_t, pos, v_t, vel, VelType::VE123, ve);

			s_ae2as(pe, ve, acc, as, nullptr, "123");
			return;
		}
		case AccType::AE131: {
			double pe[6];
			s_pos2pos(p_t, pos, PosType::PE131, pe);

			double ve[6];
			s_vel2vel(p_t, pos, v_t, vel, VelType::VE131, ve);

			s_ae2as(pe, ve, acc, as, nullptr, "131");
			return;
		}
		case AccType::AE132: {
			double pe[6];
			s_pos2pos(p_t, pos, PosType::PE132, pe);

			double ve[6];
			s_vel2vel(p_t, pos, v_t, vel, VelType::VE132, ve);

			s_ae2as(pe, ve, acc, as, nullptr, "132");
			return;
		}
		case AccType::AE212: {
			double pe[6];
			s_pos2pos(p_t, pos, PosType::PE212, pe);

			double ve[6];
			s_vel2vel(p_t, pos, v_t, vel, VelType::VE212, ve);

			s_ae2as(pe, ve, acc, as, nullptr, "212");
			return;
		}
		case AccType::AE213: {
			double pe[6];
			s_pos2pos(p_t, pos, PosType::PE213, pe);
			
			double ve[6];
			s_vel2vel(p_t, pos, v_t, vel, VelType::VE213, ve);

			s_ae2as(pe, ve, acc, as, nullptr, "213");
			return;
		}
		case AccType::AE231: {
			double pe[6];
			s_pos2pos(p_t, pos, PosType::PE231, pe);

			double ve[6];
			s_vel2vel(p_t, pos, v_t, vel, VelType::VE231, ve);

			s_ae2as(pe, ve, acc, as, nullptr, "231");
			return;
		}
		case AccType::AE232: {
			double pe[6];
			s_pos2pos(p_t, pos, PosType::PE232, pe);

			double ve[6];
			s_vel2vel(p_t, pos, v_t, vel, VelType::VE232, ve);

			s_ae2as(pe, ve, acc, as, nullptr, "232");
			return;
		}
		case AccType::AE312: {
			double pe[6];
			s_pos2pos(p_t, pos, PosType::PE312, pe);

			double ve[6];
			s_vel2vel(p_t, pos, v_t, vel, VelType::VE312, ve);

			s_ae2as(pe, ve, acc, as, nullptr, "312");
			return;
		}
		case AccType::AE313: {
			double pe[6];
			s_pos2pos(p_t, pos, PosType::PE313, pe);

			double ve[6];
			s_vel2vel(p_t, pos, v_t, vel, VelType::VE313, ve);

			s_ae2as(pe, ve, acc, as, nullptr, "313");
			return;
		}
		case AccType::AE321: {
			double pe[6];
			s_pos2pos(p_t, pos, PosType::PE321, pe);

			double ve[6];
			s_vel2vel(p_t, pos, v_t, vel, VelType::VE321, ve);

			s_ae2as(pe, ve, acc, as, nullptr, "321");
			return;
		}
		case AccType::AE323: {
			double pe[6];
			s_pos2pos(p_t, pos, PosType::PE323, pe);

			double ve[6];
			s_vel2vel(p_t, pos, v_t, vel, VelType::VE323, ve);

			s_ae2as(pe, ve, acc, as, nullptr, "323");
			return;
		}
		case AccType::AQ: {
			double pq[7];
			s_pos2pos(p_t, pos, PosType::PQ, pq);

			double vq[7];
			s_vel2vel(p_t, pos, v_t, vel, VelType::VQ, vq);

			s_aq2as(pq, vq, acc, as);
			return;
		}
		case AccType::AM: {
			double pm[16];
			s_pos2pos(p_t, pos, PosType::PM, pm);
			
			double vm[16];
			s_vel2vel(p_t, pos, v_t, vel, VelType::VM, vm);

			s_am2as(pm, vm, acc, as);
			return;
		}
		case AccType::XE121: {
			double re[3];
			s_pos2pos(p_t, pos, PosType::RE121, re);

			double we[3];
			s_vel2vel(p_t, pos, v_t, vel, VelType::WE121, we);

			s_xe2as(re, we, acc, as, nullptr, "121");
			return;
		}
		case AccType::XE123: {
			double re[3];
			s_pos2pos(p_t, pos, PosType::RE123, re);

			double we[3];
			s_vel2vel(p_t, pos, v_t, vel, VelType::WE123, we);

			s_xe2as(re, we, acc, as, nullptr, "123");
			return;
		}
		case AccType::XE131: {
			double re[3];
			s_pos2pos(p_t, pos, PosType::RE131, re);
			
			double we[3];
			s_vel2vel(p_t, pos, v_t, vel, VelType::WE131, we);

			s_xe2as(re, we, acc, as, nullptr, "131");
			return;
		}
		case AccType::XE132: {
			double re[3];
			s_pos2pos(p_t, pos, PosType::RE132, re);

			double we[3];
			s_vel2vel(p_t, pos, v_t, vel, VelType::WE132, we);

			s_xe2as(re, we, acc, as, nullptr, "132");
			return;
		}
		case AccType::XE212: {
			double re[3];
			s_pos2pos(p_t, pos, PosType::RE212, re);

			double we[3];
			s_vel2vel(p_t, pos, v_t, vel, VelType::WE212, we);

			s_xe2as(re, we, acc, as, nullptr, "212");
			return;
		}
		case AccType::XE213: {
			double re[3];
			s_pos2pos(p_t, pos, PosType::RE213, re);

			double we[3];
			s_vel2vel(p_t, pos, v_t, vel, VelType::WE213, we);

			s_xe2as(re, we, acc, as, nullptr, "213");
			return;
		}
		case AccType::XE231: {
			double re[3];
			s_pos2pos(p_t, pos, PosType::RE231, re);

			double we[3];
			s_vel2vel(p_t, pos, v_t, vel, VelType::WE231, we);

			s_xe2as(re, we, acc, as, nullptr, "231");
			return;
		}
		case AccType::XE232: {
			double re[3];
			s_pos2pos(p_t, pos, PosType::RE232, re);

			double we[3];
			s_vel2vel(p_t, pos, v_t, vel, VelType::WE232, we);

			s_xe2as(re, we, acc, as, nullptr, "232");
			return;
		}
		case AccType::XE312: {
			double re[3];
			s_pos2pos(p_t, pos, PosType::RE312, re);

			double we[3];
			s_vel2vel(p_t, pos, v_t, vel, VelType::WE312, we);

			s_xe2as(re, we, acc, as, nullptr, "312");
			return;
		}
		case AccType::XE313: {
			double re[3];
			s_pos2pos(p_t, pos, PosType::RE313, re);

			double we[3];
			s_vel2vel(p_t, pos, v_t, vel, VelType::WE313, we);

			s_xe2as(re, we, acc, as, nullptr, "313");
			return;
		}
		case AccType::XE321: {
			double re[3];
			s_pos2pos(p_t, pos, PosType::RE321, re);

			double we[3];
			s_vel2vel(p_t, pos, v_t, vel, VelType::WE321, we);

			s_xe2as(re, we, acc, as, nullptr, "321");
			return;
		}
		case AccType::XE323: {
			double re[3];
			s_pos2pos(p_t, pos, PosType::RE323, re);

			double we[3];
			s_vel2vel(p_t, pos, v_t, vel, VelType::WE323, we);

			s_xe2as(re, we, acc, as, nullptr, "323");
			return;
		}
		case AccType::XQ: {
			double rq[4];
			s_pos2pos(p_t, pos, PosType::RQ, rq);

			double wq[4];
			s_vel2vel(p_t, pos, v_t, vel, VelType::WQ, wq);

			s_xe2as(rq, wq, acc, as);
			return;
		}
		case AccType::XM: {
			double rm[9];
			s_pos2pos(p_t, pos, PosType::RM, rm);

			double wm[9];
			s_vel2vel(p_t, pos, v_t, vel, VelType::WM, wm);

			s_xe2as(rm, wm, acc, as);
			return;
		}
		case AccType::D2XYZT: {
			double pp[3];
			s_pos2pos(p_t, pos, PosType::XYZ, pp);

			double vp[3];
			s_vel2vel(p_t, pos, v_t, vel, VelType::DXYZ, vp);

			s_ap2as(pp, vp, acc, as);
			as[5] = acc[3];
			return;
		}
		case AccType::D2XYZ: {
			double pp[3];
			s_pos2pos(p_t, pos, PosType::XYZ, pp);

			double vp[3];
			s_vel2vel(p_t, pos, v_t, vel, VelType::DXYZ, vp);

			s_ap2as(pp, vp, acc, as);
			return;
		}
		case AccType::D2RTZ: {
			// tbd //
			return;
		}
		case AccType::D2XYT: {
			// tbd //
			return;
		}
		case AccType::D2XY: {
			// tbd //
			return;
		}
		case AccType::D2RT: {
			// tbd //
			return;
		}
		case AccType::D2X: {
			std::fill_n(as, 6, 0.0);
			as[0] = acc[0];
			return;
		}
		case AccType::D2Y: {
			std::fill_n(as, 6, 0.0);
			as[1] = acc[0];
			return;
		}
		case AccType::D2Z: {
			std::fill_n(as, 6, 0.0);
			as[2] = acc[0];
			return;
		}
		case AccType::D2A: {
			std::fill_n(as, 6, 0.0);
			as[3] = acc[0];
			return;
		}
		case AccType::D2B: {
			std::fill_n(as, 6, 0.0);
			as[4] = acc[0];
			return;
		}
		case AccType::D2C: {
			std::fill_n(as, 6, 0.0);
			as[5] = acc[0];
			return;
		}
		case AccType::UNKNOWN:
			return;
		default:
			return;
		}
	}
	auto ARIS_API s_as2acc(PosType p_t, const double* pos, VelType v_t, const double* vel, const double* as, AccType a_t, double* acc)noexcept->void {
		switch (a_t) {
		case AccType::AS: {
			s_vc(6, as, acc);
			return;
		}
		case AccType::AA: {
			double pp[3];
			s_pos2pos(p_t, pos, PosType::XYZ, pp);

			double vs[6];
			s_vel2vel(p_t, pos, v_t, vel, VelType::VS, vs);

			s_as2aa(vs, as, pp, acc);
			return;
		}
		case AccType::AE121: {
			double pe[6];
			s_pos2pos(p_t, pos, PosType::PE121, pe);

			double vs[6];
			s_vel2vel(p_t, pos, v_t, vel, VelType::VS, vs);

			s_as2ae(vs, as, pe, acc, nullptr, "121");
			return;
		}
		case AccType::AE123: {
			double pe[6];
			s_pos2pos(p_t, pos, PosType::PE123, pe);

			double vs[6];
			s_vel2vel(p_t, pos, v_t, vel, VelType::VS, vs);

			s_as2ae(vs, as, pe, acc, nullptr, "123");
			return;
		}
		case AccType::AE131: {
			double pe[6];
			s_pos2pos(p_t, pos, PosType::PE131, pe);

			double vs[6];
			s_vel2vel(p_t, pos, v_t, vel, VelType::VS, vs);

			s_as2ae(vs, as, pe, acc, nullptr, "131");
			return;
		}
		case AccType::AE132: {
			double pe[6];
			s_pos2pos(p_t, pos, PosType::PE132, pe);

			double vs[6];
			s_vel2vel(p_t, pos, v_t, vel, VelType::VS, vs);

			s_as2ae(vs, as, pe, acc, nullptr, "132");
			return;
		}
		case AccType::AE212: {
			double pe[6];
			s_pos2pos(p_t, pos, PosType::PE212, pe);

			double vs[6];
			s_vel2vel(p_t, pos, v_t, vel, VelType::VS, vs);

			s_as2ae(vs, as, pe, acc, nullptr, "212");
			return;
		}
		case AccType::AE213: {
			double pe[6];
			s_pos2pos(p_t, pos, PosType::PE213, pe);

			double vs[6];
			s_vel2vel(p_t, pos, v_t, vel, VelType::VS, vs);

			s_as2ae(vs, as, pe, acc, nullptr, "213");
			return;
		}
		case AccType::AE231: {
			double pe[6];
			s_pos2pos(p_t, pos, PosType::PE231, pe);

			double vs[6];
			s_vel2vel(p_t, pos, v_t, vel, VelType::VS, vs);

			s_as2ae(vs, as, pe, acc, nullptr, "231");
			return;
		}
		case AccType::AE232: {
			double pe[6];
			s_pos2pos(p_t, pos, PosType::PE232, pe);

			double vs[6];
			s_vel2vel(p_t, pos, v_t, vel, VelType::VS, vs);

			s_as2ae(vs, as, pe, acc, nullptr, "232");
			return;
		}
		case AccType::AE312: {
			double pe[6];
			s_pos2pos(p_t, pos, PosType::PE312, pe);

			double vs[6];
			s_vel2vel(p_t, pos, v_t, vel, VelType::VS, vs);

			s_as2ae(vs, as, pe, acc, nullptr, "312");
			return;
		}
		case AccType::AE313: {
			double pe[6];
			s_pos2pos(p_t, pos, PosType::PE313, pe);

			double vs[6];
			s_vel2vel(p_t, pos, v_t, vel, VelType::VS, vs);

			s_as2ae(vs, as, pe, acc, nullptr, "313");
			return;
		}
		case AccType::AE321: {
			double pe[6];
			s_pos2pos(p_t, pos, PosType::PE321, pe);

			double vs[6];
			s_vel2vel(p_t, pos, v_t, vel, VelType::VS, vs);

			s_as2ae(vs, as, pe, acc, nullptr, "321");
			return;
		}
		case AccType::AE323: {
			double pe[6];
			s_pos2pos(p_t, pos, PosType::PE323, pe);

			double vs[6];
			s_vel2vel(p_t, pos, v_t, vel, VelType::VS, vs);

			s_as2ae(vs, as, pe, acc, nullptr, "323");
			return;
		}
		case AccType::AQ: {
			double pq[7];
			s_pos2pos(p_t, pos, PosType::PQ, pq);

			double vs[6];
			s_vel2vel(p_t, pos, v_t, vel, VelType::VS, vs);

			s_as2aq(vs, as, pq, acc);
			return;
		}
		case AccType::AM: {
			double pm[16];
			s_pos2pos(p_t, pos, PosType::PM, pm);
			
			double vs[6];
			s_vel2vel(p_t, pos, v_t, vel, VelType::VS, vs);

			s_as2am(vs, as, pm, acc);
			return;
		}
		case AccType::XE121: {
			double re[3];
			s_pos2pos(p_t, pos, PosType::RE121, re);

			double vs[6];
			s_vel2vel(p_t, pos, v_t, vel, VelType::VS, vs);

			s_as2xe(vs, as, re, acc, nullptr, "121");
			return;
		}
		case AccType::XE123: {
			double re[3];
			s_pos2pos(p_t, pos, PosType::RE123, re);

			double vs[6];
			s_vel2vel(p_t, pos, v_t, vel, VelType::VS, vs);

			s_as2xe(vs, as, re, acc, nullptr, "123");
			return;
		}
		case AccType::XE131: {
			double re[3];
			s_pos2pos(p_t, pos, PosType::RE131, re);

			double vs[6];
			s_vel2vel(p_t, pos, v_t, vel, VelType::VS, vs);

			s_as2xe(vs, as, re, acc, nullptr, "131");
			return;
		}
		case AccType::XE132: {
			double re[3];
			s_pos2pos(p_t, pos, PosType::RE132, re);

			double vs[6];
			s_vel2vel(p_t, pos, v_t, vel, VelType::VS, vs);

			s_as2xe(vs, as, re, acc, nullptr, "132");
			return;
		}
		case AccType::XE212: {
			double re[3];
			s_pos2pos(p_t, pos, PosType::RE212, re);

			double vs[6];
			s_vel2vel(p_t, pos, v_t, vel, VelType::VS, vs);

			s_as2xe(vs, as, re, acc, nullptr, "212");
			return;
		}
		case AccType::XE213: {
			double re[3];
			s_pos2pos(p_t, pos, PosType::RE213, re);

			double vs[6];
			s_vel2vel(p_t, pos, v_t, vel, VelType::VS, vs);

			s_as2xe(vs, as, re, acc, nullptr, "213");
			return;
		}
		case AccType::XE231: {
			double re[3];
			s_pos2pos(p_t, pos, PosType::RE231, re);

			double vs[6];
			s_vel2vel(p_t, pos, v_t, vel, VelType::VS, vs);

			s_as2xe(vs, as, re, acc, nullptr, "231");
			return;
		}
		case AccType::XE232: {
			double re[3];
			s_pos2pos(p_t, pos, PosType::RE232, re);

			double vs[6];
			s_vel2vel(p_t, pos, v_t, vel, VelType::VS, vs);

			s_as2xe(vs, as, re, acc, nullptr, "232");
			return;
		}
		case AccType::XE312: {
			double re[3];
			s_pos2pos(p_t, pos, PosType::RE312, re);

			double vs[6];
			s_vel2vel(p_t, pos, v_t, vel, VelType::VS, vs);

			s_as2xe(vs, as, re, acc, nullptr, "312");
			return;
		}
		case AccType::XE313: {
			double re[3];
			s_pos2pos(p_t, pos, PosType::RE313, re);

			double vs[6];
			s_vel2vel(p_t, pos, v_t, vel, VelType::VS, vs);

			s_as2xe(vs, as, re, acc, nullptr, "313");
			return;
		}
		case AccType::XE321: {
			double re[3];
			s_pos2pos(p_t, pos, PosType::RE321, re);

			double vs[6];
			s_vel2vel(p_t, pos, v_t, vel, VelType::VS, vs);

			s_as2xe(vs, as, re, acc, nullptr, "321");
			return;
		}
		case AccType::XE323: {
			double re[3];
			s_pos2pos(p_t, pos, PosType::RE323, re);

			double vs[6];
			s_vel2vel(p_t, pos, v_t, vel, VelType::VS, vs);

			s_as2xe(vs, as, re, acc, nullptr, "323");
			return;
		}
		case AccType::XQ: {
			double rq[4];
			s_pos2pos(p_t, pos, PosType::RQ, rq);

			double vs[6];
			s_vel2vel(p_t, pos, v_t, vel, VelType::VS, vs);

			s_as2xq(vs, as, rq, acc);
			return;
		}
		case AccType::XM: {
			double rm[9];
			s_pos2pos(p_t, pos, PosType::RM, rm);

			double vs[6];
			s_vel2vel(p_t, pos, v_t, vel, VelType::VS, vs);

			s_as2xm(vs, as, rm, acc);
			return;
		}
		case AccType::D2XYZT: {
			double pp[3];
			s_pos2pos(p_t, pos, PosType::XYZ, pp);

			double vs[6];
			s_vel2vel(p_t, pos, v_t, vel, VelType::VS, vs);

			s_as2ap(vs, as, pp, acc);
			acc[3] = as[5];
			return;
		}
		case AccType::D2XYZ: {
			double pp[3];
			s_pos2pos(p_t, pos, PosType::XYZ, pp);

			double vs[6];
			s_vel2vel(p_t, pos, v_t, vel, VelType::VS, vs);

			s_as2ap(vs, as, pp, acc);
			return;
		}
		case AccType::D2RTZ: {
			// tbd //
			return;
		}
		case AccType::D2XYT: {
			// tbd //
			return;
		}
		case AccType::D2XY: {
			// tbd //
			return;
		}
		case AccType::D2RT: {
			// tbd //
			return;
		}
		case AccType::D2X: {
			acc[0] = as[0];
			return;
		}
		case AccType::D2Y: {
			acc[0] = as[1];
			return;
		}
		case AccType::D2Z: {
			acc[0] = as[2];
			return;
		}
		case AccType::D2A: {
			acc[0] = as[3];
			return;
		}
		case AccType::D2B: {
			acc[0] = as[4];
			return;
		}
		case AccType::D2C: {
			acc[0] = as[5];
			return;
		}
		case AccType::UNKNOWN:
			return;
		default:
			return;
		}
	}
	auto ARIS_API s_acc2acc(PosType p1_t, const double* pos1, VelType v1_t, const double* vel1, AccType a1_t, const double* acc1, AccType a2_t, double* acc2)->void {
		if (a1_t == a2_t) {
			s_vc(s_acc_type_size(a1_t), acc1, acc2);
			return;
		}

		double as[6];
		s_acc2as(p1_t, pos1, v1_t, vel1, a1_t, acc1, as);
		s_as2acc(p1_t, pos1, v1_t, vel1, as, a2_t, acc2);
	}
	auto ARIS_API s_acc2acc(Size n, const PosType* p1_t, const double* p1, const VelType* v1_t, const double* v1, const AccType* a1_t, const double* a1, const AccType *a2_t, double* a2)->void {
		aris::Size pidx1{ 0 }, vidx1{ 0 }, aidx1{ 0 }, aidx2{ 0 };
		for (aris::Size i = 0; i < n; ++i) {
			s_acc2acc(p1_t[i], p1 + pidx1, v1_t[i], v1 + vidx1, a1_t[i], a1 + aidx1, a2_t[i], a2 + aidx2);
			pidx1 += aris::dynamic::s_pos_type_size(p1_t[i]);
			vidx1 += aris::dynamic::s_vel_type_size(v1_t[i]);
			aidx1 += aris::dynamic::s_acc_type_size(a1_t[i]);
			aidx2 += aris::dynamic::s_acc_type_size(a2_t[i]);
		}
	}

	auto s_ik(int root_size, int root_num, const void* dh, IkFunc func, int which_root, const double* ee_pos, double* input_pos, double* roots_mem
		, const double* root_periods, const double* current_root, const double* input_min, const double* input_max) -> int {
		if (which_root >= root_num || which_root < 0) {
			int solution_num = 0;
			double max_diff_norm = std::numeric_limits<double>::infinity();
			for (int i = 0; i < root_num; ++i) {
				if (func(dh, ee_pos, current_root, i, roots_mem) >= 0) {
					// 采用 无穷 范数来比较两组向量，即只看差值最大的那一个数据
					double this_norm = 0;
					
					// 如果有周期，根据周期进行设置 //
					if (root_periods) {
						for (int j = 0; j < root_size; ++j) {
							// 放置到当前根所在的周期 //
							if (current_root && std::isfinite(root_periods[j]) && std::isfinite(current_root[j]))
								roots_mem[j] = s_put_near_value(roots_mem[j], current_root[j], root_periods[j]);

							// 放置到限制范围内
							auto min_p = input_min ? input_min[j] : std::numeric_limits<double>::lowest();
							auto max_p = input_max ? input_max[j] : std::numeric_limits<double>::max();
							if (s_put_into_range(roots_mem[j], root_periods[j], min_p, max_p, roots_mem[j])) {
								// 无法放到限制范围内 //
								this_norm = std::numeric_limits<double>::infinity();
								solution_num--;
								continue;
							}

							auto diff = current_root ? std::abs(roots_mem[j] - current_root[j]) : std::abs(roots_mem[j]);
							this_norm = std::max(diff, this_norm);
						}
					}

					if (max_diff_norm > this_norm) {
						max_diff_norm = this_norm;
						s_vc(root_size, roots_mem, input_pos);
					}

					++solution_num;
				}
			}
			return solution_num > 0 ? 0: -1;
		}
		else {
			if (func(dh, ee_pos, current_root, which_root, input_pos) >= 0) {
				int ret = 0;
				// 如果有周期，根据周期进行设置 //
				if (root_periods) {
					for (int j = 0; j < root_size; ++j) {
						// 放置到当前根所在的周期 //
						if (current_root && std::isfinite(root_periods[j]) && std::isfinite(current_root[j]))
							input_pos[j] = s_put_near_value(input_pos[j], current_root[j], root_periods[j]);

						// 放置到限制范围内
						auto min_p = input_min ? input_min[j] : std::numeric_limits<double>::lowest();
						auto max_p = input_max ? input_max[j] : std::numeric_limits<double>::max();
						if (s_put_into_range(input_pos[j], root_periods[j], min_p, max_p, input_pos[j])) {
							// 无法放到限制范围内，暂不处理 //
							ret = 1;
						}
					}
				}
				return ret;
			}
			else
				return -2;
		}

	}

	auto s_eye_in_hand_calib(int n, const double* pq_obj_in_eye, const double* pq_tool_in_base, double* eye_in_tool, double* mem_need)->void  {
		// see https://zhuanlan.zhihu.com/p/683246806?utm_medium=social&utm_psn=1774712167046189056&utm_source=wechat_session
		
		for (int i = 1; i < n; ++i) {
			double* M = mem_need + 4 * 4 * (i-1);
			double L[7], R[7];

			s_inv_pq_dot_pq(pq_tool_in_base, pq_tool_in_base + 7 * i, L);
			s_pq_dot_inv_pq(pq_obj_in_eye, pq_obj_in_eye + 7 * i, R);
			
			if (L[6] < 0.0) {
				s_iv(4, L + 3);
			}
			if (R[6] < 0.0) {
				s_iv(4, R + 3);
			}

			auto sa = L[6];
			auto sb = R[6];
			auto va = L + 3;
			auto vb = R + 3;

			M[0] = sa - sb;
			M[1] = -(va[0] - vb[0]);
			M[2] = -(va[1] - vb[1]);
			M[3] = -(va[2] - vb[2]);

			M[4] = (va[0] - vb[0]);
			M[8] = (va[1] - vb[1]);
			M[12] = (va[2] - vb[2]);

			double v1 = va[0] + vb[0];
			double v2 = va[1] + vb[1];
			double v3 = va[2] + vb[2];

			M[5] = sa - sb;
			M[6] = -v3;
			M[7] = v2;

			M[9] = v3;
			M[10] = sa - sb;
			M[11] = -v1;
			M[13] = -v2;
			M[14] = v1;
			M[15] = sa - sb;
		}

		double V[16];
		s_svd((n - 1) * 4, 4, mem_need, mem_need + (n - 1) * 4 *4, mem_need, V);

		// U size : (n - 1) * 4 *(n - 1) * 4 = 16 * (n-1)^2
		// S size : (n - 1) * 4 * 4 = 16*(n-1)

		// total : 16 * n * (n-1)
		eye_in_tool[3] = V[7];
		eye_in_tool[4] = V[11];
		eye_in_tool[5] = V[15];
		eye_in_tool[6] = V[3];

		s_nv(4, (eye_in_tool[6] < 0 ? -1.0 : 1.0) / s_vv(4, eye_in_tool + 3, eye_in_tool + 3), eye_in_tool + 3);

		// CALIB xyz //
		auto A = mem_need;
		auto b = mem_need + n * 9;
		for (int i = 1; i < n; ++i) {
			double L[7], R[7];
			s_inv_pq_dot_pq(pq_tool_in_base, pq_tool_in_base + 7 * i, L);
			s_pq_dot_inv_pq(pq_obj_in_eye, pq_obj_in_eye + 7 * i, R);
		
			s_rq2rm(L + 3, A + (i - 1) * 9);
			A[(i - 1) * 9 + 0 * 4] -= 1.0;
			A[(i - 1) * 9 + 1 * 4] -= 1.0;
			A[(i - 1) * 9 + 2 * 4] -= 1.0;

			s_pq_dot_v3(eye_in_tool, R, b + (i - 1) * 3);
			b[(i - 1) * 3 + 0] -= L[0];
			b[(i - 1) * 3 + 1] -= L[1];
			b[(i - 1) * 3 + 2] -= L[2];
		}

		s_householder_ut(3 * (n - 1), 3, A, A, b + 3*(n-1));
		s_householder_ut_sov(3 * (n - 1), 3, 1, A, b + 3*(n-1), b, b);

		eye_in_tool[0] = b[0];
		eye_in_tool[1] = b[1];
		eye_in_tool[2] = b[2];
	}
	auto s_eye_to_hand_calib(int n, const double* pq_obj_in_eye, const double* pq_tool_in_base, double* eye_in_base, double* mem_need) -> void {
		// see https://zhuanlan.zhihu.com/p/683246806?utm_medium=social&utm_psn=1774712167046189056&utm_source=wechat_session

		// 似乎把 base 看成 tool, 把 tool 看成 base，就变成了 s_eye_in_hand_calib 的问题


		for (int i = 1; i < n; ++i) {
			double* M = mem_need + 4 * 4 * (i - 1);
			double L[7], R[7];

			s_pq_dot_inv_pq(pq_tool_in_base, pq_tool_in_base + 7 * i, L);
			s_pq_dot_inv_pq(pq_obj_in_eye, pq_obj_in_eye + 7 * i, R);

			if (L[6] < 0.0) {
				s_iv(4, L + 3);
			}
			if (R[6] < 0.0) {
				s_iv(4, R + 3);
			}

			auto sa = L[6];
			auto sb = R[6];
			auto va = L + 3;
			auto vb = R + 3;

			M[0] = sa - sb;
			M[1] = -(va[0] - vb[0]);
			M[2] = -(va[1] - vb[1]);
			M[3] = -(va[2] - vb[2]);

			M[4] = (va[0] - vb[0]);
			M[8] = (va[1] - vb[1]);
			M[12] = (va[2] - vb[2]);

			double v1 = va[0] + vb[0];
			double v2 = va[1] + vb[1];
			double v3 = va[2] + vb[2];

			M[5] = sa - sb;
			M[6] = -v3;
			M[7] = v2;

			M[9] = v3;
			M[10] = sa - sb;
			M[11] = -v1;
			M[13] = -v2;
			M[14] = v1;
			M[15] = sa - sb;
		}

		double V[16];
		s_svd((n - 1) * 4, 4, mem_need, mem_need + (n - 1) * 4 * 4, mem_need, V);

		// U size : (n - 1) * 4 *(n - 1) * 4 = 16 * (n-1)^2
		// S size : (n - 1) * 4 * 4 = 16*(n-1)

		// total : 16 * n * (n-1)
		eye_in_base[3] = V[7];
		eye_in_base[4] = V[11];
		eye_in_base[5] = V[15];
		eye_in_base[6] = V[3];

		s_nv(4, (eye_in_base[6] < 0 ? -1.0 : 1.0) / s_vv(4, eye_in_base + 3, eye_in_base + 3), eye_in_base + 3);

		// CALIB xyz //
		auto A = mem_need;
		auto b = mem_need + n * 9;
		for (int i = 1; i < n; ++i) {
			double L[7], R[7];
			s_pq_dot_inv_pq(pq_tool_in_base, pq_tool_in_base + 7 * i, L);
			s_pq_dot_inv_pq(pq_obj_in_eye, pq_obj_in_eye + 7 * i, R);

			s_rq2rm(L + 3, A + (i - 1) * 9);
			A[(i - 1) * 9 + 0 * 4] -= 1.0;
			A[(i - 1) * 9 + 1 * 4] -= 1.0;
			A[(i - 1) * 9 + 2 * 4] -= 1.0;

			s_pq_dot_v3(eye_in_base, R, b + (i - 1) * 3);
			b[(i - 1) * 3 + 0] -= L[0];
			b[(i - 1) * 3 + 1] -= L[1];
			b[(i - 1) * 3 + 2] -= L[2];
		}

		s_householder_ut(3 * (n - 1), 3, A, A, b + 3 * (n - 1));
		s_householder_ut_sov(3 * (n - 1), 3, 1, A, b + 3 * (n - 1), b, b);

		eye_in_base[0] = b[0];
		eye_in_base[1] = b[1];
		eye_in_base[2] = b[2];
	}

	ARIS_REGISTRATION{

		aris::core::class_<PosType>("PosType")
			.textMethod([](PosType* type)->std::string {
				switch (*type) {
				case PosType::PE121:return "PE121";
				case PosType::PE123:return "PE123";
				case PosType::PE131:return "PE131";
				case PosType::PE132:return "PE132";
				case PosType::PE212:return "PE212";
				case PosType::PE213:return "PE213";
				case PosType::PE231:return "PE231";
				case PosType::PE232:return "PE232";
				case PosType::PE312:return "PE312";
				case PosType::PE313:return "PE313";
				case PosType::PE321:return "PE321";
				case PosType::PE323:return "PE323";
				case PosType::PQ:return "PQ";
				case PosType::PM:return "PM";
				case PosType::RE121:return "RE121";
				case PosType::RE123:return "RE123";
				case PosType::RE131:return "RE131";
				case PosType::RE132:return "RE132";
				case PosType::RE212:return "RE212";
				case PosType::RE213:return "RE213";
				case PosType::RE231:return "RE231";
				case PosType::RE232:return "RE232";
				case PosType::RE312:return "RE312";
				case PosType::RE313:return "RE313";
				case PosType::RE321:return "RE321";
				case PosType::RE323:return "RE323";
				case PosType::RQ:return "RQ";
				case PosType::RM:return "RM";
				case PosType::XYZT:return "XYZT";
				case PosType::XYZ:return "XYZ";
				case PosType::RTZ:return "RTZ";
				case PosType::XYT:return "XYT";
				case PosType::XY:return "XY";
				case PosType::RT:return "RT";
				case PosType::X:return "X";
				case PosType::Y:return "Y";
				case PosType::Z:return "Z";
				case PosType::A:return "A";
				case PosType::B:return "B";
				case PosType::C:return "C";
				case PosType::UNKNOWN:return "UNKNOWN";
				default:return "UNKNOWN";
				}
			}, [](PosType* type, std::string_view name)->void {
				if (name == "PE121")*type = PosType::PE121;
				if (name == "PE123")*type = PosType::PE123;
				if (name == "PE131")*type = PosType::PE131;
				if (name == "PE132")*type = PosType::PE132;
				if (name == "PE212")*type = PosType::PE212;
				if (name == "PE213")*type = PosType::PE213;
				if (name == "PE231")*type = PosType::PE231;
				if (name == "PE232")*type = PosType::PE232;
				if (name == "PE312")*type = PosType::PE312;
				if (name == "PE313")*type = PosType::PE313;
				if (name == "PE321")*type = PosType::PE321;
				if (name == "PE323")*type = PosType::PE323;
				if (name == "PQ")*type = PosType::PQ;
				if (name == "PM")*type = PosType::PM;
				if (name == "RE121")*type = PosType::RE121;
				if (name == "RE123")*type = PosType::RE123;
				if (name == "RE131")*type = PosType::RE131;
				if (name == "RE132")*type = PosType::RE132;
				if (name == "RE212")*type = PosType::RE212;
				if (name == "RE213")*type = PosType::RE213;
				if (name == "RE231")*type = PosType::RE231;
				if (name == "RE232")*type = PosType::RE232;
				if (name == "RE312")*type = PosType::RE312;
				if (name == "RE313")*type = PosType::RE313;
				if (name == "RE321")*type = PosType::RE321;
				if (name == "RE323")*type = PosType::RE323;
				if (name == "RQ")*type = PosType::RQ;
				if (name == "RM")*type = PosType::RM;
				if (name == "XYZT")*type = PosType::XYZT;
				if (name == "XYZ")*type = PosType::XYZ;
				if (name == "RTZ")*type = PosType::RTZ;
				if (name == "XYT")*type = PosType::XYT;
				if (name == "XY")*type = PosType::XY;
				if (name == "RT")*type = PosType::RT;
				if (name == "X")*type = PosType::X;
				if (name == "Y")*type = PosType::Y;
				if (name == "Z")*type = PosType::Z;
				if (name == "A")*type = PosType::A;
				if (name == "B")*type = PosType::B;
				if (name == "C")*type = PosType::C;
				if (name == "UNKNOWN")*type = PosType::UNKNOWN;
			});

		aris::core::class_<VelType>("VelType")
			.textMethod([](VelType* type)->std::string {
				switch (*type) {
				case VelType::VA:return "VA";
				case VelType::VS:return "VS";
				case VelType::VE313:return "PE313";
				case VelType::VE321:return "PE321";
				case VelType::VE123:return "PE123";
				case VelType::VQ:return "PQ";
				case VelType::VM:return "PM";
				case VelType::WA:return "WA";
				case VelType::WE313:return "WE313";
				case VelType::WE321:return "WE321";
				case VelType::WE123:return "WE123";
				case VelType::WQ:return "WQ";
				case VelType::WM:return "WM";
				case VelType::DXYZT:return "DXYZT";
				case VelType::DXYZ:return "DXYZ";
				case VelType::DRTZ:return "DRTZ";
				case VelType::DXYT:return "DXYT";
				case VelType::DXY:return "DXY";
				case VelType::DRT:return "DRT";
				case VelType::DX:return "DX";
				case VelType::DA:return "DA";
				case VelType::UNKNOWN:return "UNKNOWN";
				default:return "UNKNOWN";
				}
			}, [](VelType* type, std::string_view name)->void {
				if (name == "VA")*type = VelType::VA;
				if (name == "VS")*type = VelType::VS;
				if (name == "VE121")*type = VelType::VE121;
				if (name == "VE123")*type = VelType::VE123;
				if (name == "VE131")*type = VelType::VE131;
				if (name == "VE132")*type = VelType::VE132;
				if (name == "VE212")*type = VelType::VE212;
				if (name == "VE213")*type = VelType::VE213;
				if (name == "VE231")*type = VelType::VE231;
				if (name == "VE232")*type = VelType::VE232;
				if (name == "VE312")*type = VelType::VE312;
				if (name == "VE313")*type = VelType::VE313;
				if (name == "VE321")*type = VelType::VE321;
				if (name == "VE323")*type = VelType::VE323;
				if (name == "VQ")*type = VelType::VQ;
				if (name == "VM")*type = VelType::VM;
				if (name == "WA")*type = VelType::WA;
				if (name == "WE121")*type = VelType::WE121;
				if (name == "WE123")*type = VelType::WE123;
				if (name == "WE131")*type = VelType::WE131;
				if (name == "WE132")*type = VelType::WE132;
				if (name == "WE212")*type = VelType::WE212;
				if (name == "WE213")*type = VelType::WE213;
				if (name == "WE231")*type = VelType::WE231;
				if (name == "WE232")*type = VelType::WE232;
				if (name == "WE312")*type = VelType::WE312;
				if (name == "WE313")*type = VelType::WE313;
				if (name == "WE321")*type = VelType::WE321;
				if (name == "WE323")*type = VelType::WE323;
				if (name == "WQ")*type = VelType::WQ;
				if (name == "WM")*type = VelType::WM;
				if (name == "DXYZT")*type = VelType::DXYZT;
				if (name == "DXYZ")*type = VelType::DXYZ;
				if (name == "DRTZ")*type = VelType::DRTZ;
				if (name == "DXYT")*type = VelType::DXYT;
				if (name == "DXY")*type = VelType::DXY;
				if (name == "DRT")*type = VelType::DRT;
				if (name == "DX")*type = VelType::DX;
				if (name == "DY")*type = VelType::DY;
				if (name == "DZ")*type = VelType::DZ;
				if (name == "DA")*type = VelType::DA;
				if (name == "DB")*type = VelType::DB;
				if (name == "DC")*type = VelType::DC;
				if (name == "UNKNOWN")*type = VelType::UNKNOWN;
			});
	
		aris::core::class_<AccType>("AccType")
			.textMethod([](AccType* type)->std::string {
				switch (*type) {
				case AccType::AA:return "AA";
				case AccType::AS:return "AS";
				case AccType::AE313:return "AE313";
				case AccType::AE321:return "AE321";
				case AccType::AE123:return "AE123";
				case AccType::AQ:return "AQ";
				case AccType::AM:return "AM";
				case AccType::XA:return "XA";
				case AccType::XE313:return "XE313";
				case AccType::XE321:return "XE321";
				case AccType::XE123:return "XE123";
				case AccType::XQ:return "XQ";
				case AccType::XM:return "XM";
				case AccType::D2XYZT:return "D2XYZT";
				case AccType::D2XYZ:return "D2XYZ";
				case AccType::D2RTZ:return "D2RTZ";
				case AccType::D2XYT:return "D2XYT";
				case AccType::D2XY:return "D2XY";
				case AccType::D2RT:return "D2RT";
				case AccType::D2X:return "D2X";
				case AccType::D2A:return "D2A";
				case AccType::UNKNOWN:return "UNKNOWN";
				default:return "UNKNOWN";
				}
			}, [](AccType* type, std::string_view name)->void {
				if (name == "AA")*type = AccType::AA;
				if (name == "AS")*type = AccType::AS;
				if (name == "AE121")*type = AccType::AE121;
				if (name == "AE123")*type = AccType::AE123;
				if (name == "AE131")*type = AccType::AE131;
				if (name == "AE132")*type = AccType::AE132;
				if (name == "AE212")*type = AccType::AE212;
				if (name == "AE213")*type = AccType::AE213;
				if (name == "AE231")*type = AccType::AE231;
				if (name == "AE232")*type = AccType::AE232;
				if (name == "AE312")*type = AccType::AE312;
				if (name == "AE313")*type = AccType::AE313;
				if (name == "AE321")*type = AccType::AE321;
				if (name == "AE323")*type = AccType::AE323;
				if (name == "AQ")*type = AccType::AQ;
				if (name == "AM")*type = AccType::AM;
				if (name == "XA")*type = AccType::XA;
				if (name == "XE121")*type = AccType::XE121;
				if (name == "XE123")*type = AccType::XE123;
				if (name == "XE131")*type = AccType::XE131;
				if (name == "XE132")*type = AccType::XE132;
				if (name == "XE212")*type = AccType::XE212;
				if (name == "XE213")*type = AccType::XE213;
				if (name == "XE231")*type = AccType::XE231;
				if (name == "XE232")*type = AccType::XE232;
				if (name == "XE312")*type = AccType::XE312;
				if (name == "XE313")*type = AccType::XE313;
				if (name == "XE321")*type = AccType::XE321;
				if (name == "XE323")*type = AccType::XE323;
				if (name == "XQ")*type = AccType::XQ;
				if (name == "XM")*type = AccType::XM;
				if (name == "D2XYZT")*type = AccType::D2XYZT;
				if (name == "D2XYZ")*type = AccType::D2XYZ;
				if (name == "D2RTZ")*type = AccType::D2RTZ;
				if (name == "D2XYT")*type = AccType::D2XYT;
				if (name == "D2XY")*type = AccType::D2XY;
				if (name == "D2RT")*type = AccType::D2RT;
				if (name == "D2X")*type = AccType::D2X;
				if (name == "D2Y")*type = AccType::D2Y;
				if (name == "D2Z")*type = AccType::D2Z;
				if (name == "D2A")*type = AccType::D2A;
				if (name == "D2B")*type = AccType::D2B;
				if (name == "D2C")*type = AccType::D2C;
				if (name == "UNKNOWN")*type = AccType::UNKNOWN;
			});

		aris::core::class_<FceType>("FceType")
			.textMethod([](FceType* type)->std::string {
				switch (*type) {
				case FceType::FT:return "FT";
				case FceType::FS:return "FS";
				case FceType::FXYZ_TZ:return "FXYZ_TZ";
				case FceType::FXYZ:return "FXYZ";
				case FceType::TXYZ:return "TXYZ";
				case FceType::FXY_TZ:return "FXY_TZ";
				case FceType::FXY:return "FXY";
				case FceType::FX:return "FX";
				case FceType::FY:return "FY";
				case FceType::FZ:return "FZ";
				case FceType::TX:return "TX";
				case FceType::TY:return "TY";
				case FceType::TZ:return "TZ";
				case FceType::UNKNOWN:return "UNKNOWN";
				default:return "UNKNOWN";
				}
			}, [](FceType* type, std::string_view name)->void {
				if (name == "FT")*type = FceType::FT;
				if (name == "FS")*type = FceType::FS;
				if (name == "FXYZ_TZ")*type = FceType::FXYZ_TZ;
				if (name == "TXYZ")*type = FceType::TXYZ;
				if (name == "FXYZ")*type = FceType::FXYZ;
				if (name == "FXY_TZ")*type = FceType::FXY_TZ;
				if (name == "FXY")*type = FceType::FXY;
				if (name == "FX")*type = FceType::FX;
				if (name == "FY")*type = FceType::FY;
				if (name == "FZ")*type = FceType::FZ;
				if (name == "TX")*type = FceType::TX;
				if (name == "TY")*type = FceType::TY;
				if (name == "TZ")*type = FceType::TZ;
				if (name == "UNKNOWN")*type = FceType::UNKNOWN;
			});

	}		
		
}
