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

	auto ARIS_API s_ee_pos2pm(EEType type, const double* pos, double* pm)noexcept->void {
		switch (type) {
		case EEType::PE313:
			s_pe2pm(pos, pm, "313");
			return;
		case EEType::PE321:
			s_pe2pm(pos, pm, "321");
			return;
		case EEType::PE123:
			s_pe2pm(pos, pm, "123");
			return;
		case EEType::PQ:
			s_pq2pm(pos, pm);
			return;
		case EEType::PM:
			s_vc(16, pos, pm);
			return;
		case EEType::RE313:
			s_eye(4, pm);
			s_re2pm(pos, pm, "313");
			return;
		case EEType::RE321:
			s_eye(4, pm);
			s_re2pm(pos, pm, "321");
			return;
		case EEType::RE123:
			s_eye(4, pm);
			s_re2pm(pos, pm, "123");
			return;
		case EEType::RQ:
			s_eye(4, pm);
			s_rq2pm(pos, pm);
			return;
		case EEType::RM:
			s_eye(4, pm);
			s_rm2pm(pos, pm);
			return;
		case EEType::XYZT:
			s_eye(4, pm);
			s_pp2pm(pos, pm);
			s_rmz(pos[3], pm, 4);
			return;
		case EEType::XYZ:
			s_eye(4, pm);
			s_pp2pm(pos, pm);
			s_rmz(pos[3], pm, 4);
			return;
		case EEType::RTZ:
			s_eye(4, pm);
			pm[3] = pos[0] * std::cos(pos[1]);
			pm[7] = pos[0] * std::sin(pos[1]);
			pm[11] = pos[2];
			s_rmz(pos[1], pm, 4);
			return;
		case EEType::XYT:
			s_eye(4, pm);
			pm[3] = pos[0];
			pm[7] = pos[1];
			s_rmz(pos[2], pm, 4);
			return;
		case EEType::XY:
			s_eye(4, pm);
			pm[3] = pos[0];
			pm[7] = pos[1];
			return;
		case EEType::RT:
			s_eye(4, pm);
			pm[3] = pos[0] * std::cos(pos[1]);
			pm[7] = pos[0] * std::sin(pos[1]);
			s_rmz(pos[1], pm, 4);
			return;
		case EEType::X:
			s_eye(4, pm);
			pm[3] = pos[0];
			return;
		case EEType::A:
			s_rmz(pos[0], pm, 4);
			return;
		case EEType::UNKNOWN:
			return;
		default:
			return;
		}
	}
	auto ARIS_API s_ee_pm2pos(EEType type, const double* pm, double* pos)noexcept->void {
		switch (type) {
		case EEType::PE313:
			s_pm2pe(pm, pos, "313");
			return;
		case EEType::PE321:
			s_pm2pe(pm, pos, "321");
			return;
		case EEType::PE123:
			s_pm2pe(pm, pos, "123");
			return;
		case EEType::PQ:
			s_pm2pq(pm, pos);
			return;
		case EEType::PM:
			s_vc(16, pm, pos);
			return;
		case EEType::RE313:
			s_pm2re(pm, pos, "313");
			return;
		case EEType::RE321:
			s_pm2re(pm, pos, "321");
			return;
		case EEType::RE123:
			s_pm2re(pm, pos, "123");
			return;
		case EEType::RQ:
			s_pm2rq(pm, pos);
			return;
		case EEType::RM:
			s_pm2rm(pm, pos);
			return;
		case EEType::XYZT:
			s_pm2pp(pm, pos);
			pos[3] = s_rmz_theta(pm, 4);
			return;
		case EEType::XYZ:
			s_pm2pp(pm, pos);
			return;
		case EEType::RTZ:
			pos[0] = std::sqrt(pm[3] * pm[3] + pm[7] * pm[7]);
			pos[1] = std::atan2(pm[3], pm[7]);
			pos[2] = pm[11];
			return;
		case EEType::XYT:
			pos[0] = pm[3];
			pos[1] = pm[7];
			pos[2] = s_rmz_theta(pm, 4);
			return;
		case EEType::XY:
			pos[0] = pm[3];
			pos[1] = pm[7];
			return;
		case EEType::RT:
			pos[0] = std::sqrt(pm[3] * pm[3] + pm[7] * pm[7]);
			pos[1] = std::atan2(pm[3], pm[7]);
			return;
		case EEType::X:
			pos[0] = pm[3];
			return;
		case EEType::A:
			pos[0] = s_rmz_theta(pm, 4);
			return;
		case EEType::UNKNOWN:
			return;
		default:
			return;
		}
	}

	auto s_ik(int root_size, int root_num, const void* dh, IkFunc func, int which_root, const double* ee_pos, double* input_pos, double* roots_mem, const double* root_periods, const double* current_root) -> int {
		if (which_root >= root_num || which_root < 0) {
			int solution_num = 0;
			double max_diff_norm = std::numeric_limits<double>::infinity();
			for (int i = 0; i < root_num; ++i) {
				if (func(dh, ee_pos, current_root, i, roots_mem) >= 0) {
					// 采用 无穷 范数来比较两组向量，即只看差值最大的那一个数据
					double this_norm = 0;
					for (int j = 0; j < root_size; ++j) {
						// 放置到当前根所在的周期
						if (root_periods && current_root && std::isfinite(root_periods[j]) && std::isfinite(current_root[j]))
							roots_mem[j] = s_put_near_value(roots_mem[j], current_root[j], root_periods[j]);

						auto diff = current_root ? std::abs(roots_mem[j] - current_root[j]) : std::abs(roots_mem[j]);
						this_norm = std::max(diff, this_norm);
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
				// 贴近当前根
				for (int j = 0; j < root_size; ++j) {
					if (root_periods && current_root && std::isfinite(root_periods[j]) && std::isfinite(current_root[j]))
						input_pos[j] = s_put_near_value(input_pos[j], current_root[j], root_periods[j]);
				}
				return 0;
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
}
