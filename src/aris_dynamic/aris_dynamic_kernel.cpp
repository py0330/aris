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

#include "aris_dynamic_kernel.h"

namespace aris
{
	namespace dynamic
	{
		inline auto default_pp()->const double* { static double value[3]{ 0,0,0 }; return value; }
		inline auto default_re()->const double* { static double value[3]{ 0,0,0 }; return value; }
		inline auto default_rq()->const double* { static double value[4]{ 0,0,0,1 };	return value; }
		inline auto default_rm()->const double* { static double value[9]{ 1,0,0,0,1,0,0,0,1 };	return value; }
		inline auto default_pe()->const double* { static double value[6]{ 0,0,0,0,0,0 };	return value; }
		inline auto default_pq()->const double* { static double value[7]{ 0,0,0,0,0,0,1 };	return value; }
		inline auto default_pm()->const double* { static double value[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 }; return value; }

		inline auto default_vp()->const double* { static double value[3]{ 0,0,0 }; return value; }
		inline auto default_we()->const double* { static double value[3]{ 0,0,0 };	return value; }
		inline auto default_wq()->const double* { static double value[4]{ 0,0,0,0 };	return value; }
		inline auto default_wm()->const double* { static double value[9]{ 0,0,0,0,0,0,0,0,0 };	return value; }
		inline auto default_ve()->const double* { static double value[6]{ 0,0,0,0,0,0 };	return value; }
		inline auto default_vq()->const double* { static double value[7]{ 0,0,0,0,0,0,0 };	return value; }
		inline auto default_vm()->const double* { static double value[16]{ 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 }; return value; }
		inline auto default_wa()->const double* { static double value[3]{ 0,0,0 }; return value; }
		inline auto default_va()->const double* { static double value[6]{ 0,0,0,0,0,0 };	return value; }
		inline auto default_vs()->const double* { static double value[6]{ 0,0,0,0,0,0 };	return value; }

		inline auto default_ap()->const double* { static double value[3]{ 0,0,0 }; return value; }
		inline auto default_xe()->const double* { static double value[3]{ 0,0,0 };	return value; }
		inline auto default_xq()->const double* { static double value[4]{ 0,0,0,0 };	return value; }
		inline auto default_xm()->const double* { static double value[9]{ 0,0,0,0,0,0,0,0,0 };	return value; }
		inline auto default_ae()->const double* { static double value[6]{ 0,0,0,0,0,0 };	return value; }
		inline auto default_aq()->const double* { static double value[7]{ 0,0,0,0,0,0,0 };	return value; }
		inline auto default_am()->const double* { static double value[16]{ 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 }; return value; }
		inline auto default_xa()->const double* { static double value[3]{ 0,0,0 }; return value; }
		inline auto default_aa()->const double* { static double value[6]{ 0,0,0,0,0,0 };	return value; }
		inline auto default_as()->const double* { static double value[6]{ 0,0,0,0,0,0 };	return value; }

		inline auto default_fs()->const double* { static double value[6]{ 0,0,0,0,0,0 }; return value; }
		inline auto default_is()->const double* { static double value[36]{ 0 }; return value; }
		inline auto default_iv()->const double* { static double value[16]{ 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 }; return value; }
		inline auto default_in()->const double* { static double value[9]{ 0,0,0,0,0,0,0,0,0 }; return value; }

		auto s_re2rm(const double *re_in, double *rm_out, const char *eu_type_in, std::size_t rm_ld) noexcept->void
		{
			// 补充默认参数 //
			re_in = re_in ? re_in : default_re();
			double rm_out_default[9];
			rm_out = rm_out ? rm_out : rm_out_default;

			// 正式开始计算 //
			static const double P[3][3] = { { 0, -1, 1 },{ 1, 0, -1 },{ -1, 1, 0 } };
			static const double Q[3][3] = { { 1, 0, 0 },{ 0, 1, 0 },{ 0, 0, 1 } };

			double Abb, Add, Abd, Adb;
			double Bac, Bae, Bdc, Bde;
			double Cbb, Cee, Cbe, Ceb;
			double s_, c_;

			const int a = eu_type_in[0] - '1';
			const int b = eu_type_in[1] - '1';
			const int c = eu_type_in[2] - '1';
			const int d = 3 - a - b;
			const int e = 3 - b - c;

			c_ = std::cos(re_in[0]);
			s_ = std::sin(re_in[0]);
			Abb = c_;
			Add = Abb;
			Abd = P[b][d] * s_;
			Adb = -Abd;

			s_ = std::sin(re_in[1]);
			c_ = std::cos(re_in[1]);
			Bac = P[a][c] * s_ + Q[a][c] * c_;
			Bae = P[a][e] * s_ + Q[a][e] * c_;
			Bdc = P[d][c] * s_ + Q[d][c] * c_;
			Bde = P[d][e] * s_ + Q[d][e] * c_;

			c_ = std::cos(re_in[2]);
			s_ = std::sin(re_in[2]);
			Cbb = c_;
			Cee = Cbb;
			Cbe = P[b][e] * s_;
			Ceb = -Cbe;

			rm_out[a * rm_ld + c] = Bac;
			rm_out[a * rm_ld + b] = Bae * Ceb;
			rm_out[a * rm_ld + e] = Bae * Cee;
			rm_out[b * rm_ld + c] = Abd * Bdc;
			rm_out[b * rm_ld + b] = Abb * Cbb + Abd * Bde * Ceb;
			rm_out[b * rm_ld + e] = Abb * Cbe + Abd * Bde * Cee;
			rm_out[d * rm_ld + c] = Add * Bdc;
			rm_out[d * rm_ld + b] = Adb * Cbb + Add * Bde * Ceb;
			rm_out[d * rm_ld + e] = Adb * Cbe + Add * Bde * Cee;
		}
		auto s_rm2re(const double *rm_in, double *re_out, const char *eu_type_in, std::size_t rm_ld) noexcept->void
		{
			// 补充默认参数 //
			rm_in = rm_in ? rm_in : default_rm();
			double re_out_default[3];
			re_out = re_out ? re_out : re_out_default;

			// 正式开始计算 //
			static const double P[3][3] = { { 0, -1, 1 },{ 1, 0, -1 },{ -1, 1, 0 } };
			static const double Q[3][3] = { { 1, 0, 0 },{ 0, 1, 0 },{ 0, 0, 1 } };

			const int a = eu_type_in[0] - '1';
			const int b = eu_type_in[1] - '1';
			const int c = eu_type_in[2] - '1';
			const int d = 3 - a - b;
			const int e = 3 - b - c;

			// 计算phi2 //
			double s_ = std::sqrt((rm_in[rm_ld * a + b] * rm_in[rm_ld * a + b] + rm_in[rm_ld * a + e] * rm_in[rm_ld * a + e]
				+ rm_in[rm_ld * b + c] * rm_in[rm_ld * b + c] + rm_in[rm_ld * d + c] * rm_in[rm_ld * d + c]) / 2);

			double c_ = rm_in[rm_ld * a + c];
			re_out[1] = (a == c ? std::atan2(s_, c_) : std::atan2(P[a][c] * c_, s_));

			// 计算phi1和phi3 //
			double phi13 = std::atan2(rm_in[rm_ld * b + e] - rm_in[rm_ld * d + b], rm_in[rm_ld * b + b] + rm_in[rm_ld * d + e]);
			double phi31 = std::atan2(rm_in[rm_ld * b + e] + rm_in[rm_ld * d + b], rm_in[rm_ld * b + b] - rm_in[rm_ld * d + e]);

			re_out[0] = P[b][d] * (phi13 - phi31) / 2;
			re_out[2] = P[b][e] * (phi13 + phi31) / 2;

			// 检查 //
			double sig[4];
			sig[0] = (P[a][e] + Q[a][e])*P[e][b] * rm_in[a * rm_ld + b] * std::sin(re_out[2]);
			sig[1] = (P[a][e] + Q[a][e])*rm_in[rm_ld * a + e] * std::cos(re_out[2]);
			sig[2] = (P[d][c] + Q[d][c])*P[b][d] * rm_in[b * rm_ld + c] * std::sin(re_out[0]);
			sig[3] = (P[d][c] + Q[d][c])*rm_in[rm_ld * d + c] * std::cos(re_out[0]);

			if (*std::max_element(sig, sig + rm_ld, [](double d1, double d2) {return (std::abs(d1) < std::abs(d2)); })<0)
			{
				re_out[0] += PI;
				re_out[2] += PI;
			}

			re_out[0] = (re_out[0] < 0 ? re_out[0] + 2 * PI : re_out[0]);
			re_out[2] = (re_out[2] < 0 ? re_out[2] + 2 * PI : re_out[2]);
		}
		auto s_rq2rm(const double *rq_in, double *rm_out, std::size_t rm_ld) noexcept->void
		{
			// 补充默认参数 //
			rq_in = rq_in ? rq_in : default_rq();
			double rm_out_default[9];
			rm_out = rm_out ? rm_out : rm_out_default;

			// 正式开始计算 //
			
			rm_out[0 * rm_ld + 0] = 1 - 2 * rq_in[1] * rq_in[1] - 2 * rq_in[2] * rq_in[2];
			rm_out[0 * rm_ld + 1] = 2 * rq_in[0] * rq_in[1] - 2 * rq_in[3] * rq_in[2];
			rm_out[0 * rm_ld + 2] = 2 * rq_in[0] * rq_in[2] + 2 * rq_in[3] * rq_in[1];

			rm_out[1 * rm_ld + 0] = 2 * rq_in[0] * rq_in[1] + 2 * rq_in[3] * rq_in[2];
			rm_out[1 * rm_ld + 1] = 1 - 2 * rq_in[0] * rq_in[0] - 2 * rq_in[2] * rq_in[2];
			rm_out[1 * rm_ld + 2] = 2 * rq_in[1] * rq_in[2] - 2 * rq_in[3] * rq_in[0];

			rm_out[2 * rm_ld + 0] = 2 * rq_in[0] * rq_in[2] - 2 * rq_in[3] * rq_in[1];
			rm_out[2 * rm_ld + 1] = 2 * rq_in[1] * rq_in[2] + 2 * rq_in[3] * rq_in[0];
			rm_out[2 * rm_ld + 2] = 1 - 2 * rq_in[0] * rq_in[0] - 2 * rq_in[1] * rq_in[1];
		}
		auto s_rm2rq(const double *rm_in, double *rq_out, std::size_t rm_ld) noexcept->void
		{
			// 补充默认参数 //
			rm_in = rm_in ? rm_in : default_rm();
			double rq_out_default[4];
			rq_out = rq_out ? rq_out : rq_out_default;

			// 正式开始计算 //
			static const double T[4][4]{ { 0,1,1,-1 },{ 1,0,1,-1 },{ 1,1,0,-1 },{ -1,-1,-1,0 } };
			static const int P[4][4]{ { -1,0,0,2 },{ 1,-1,1,0 },{ 2,2,-1,1 },{ 2,0,1,-1 } };
			static const int Q[4][4]{ { -1,1,2,1 },{ 0,-1,2,2 },{ 0,1,-1,0 },{ 1,2,0,-1 } };

			double qt_square[4];

			qt_square[0] = (1 + rm_in[0] - rm_in[rm_ld + 1] - rm_in[2 * rm_ld + 2]) / 4;
			qt_square[1] = (1 + rm_in[rm_ld + 1] - rm_in[0] - rm_in[2 * rm_ld + 2]) / 4;
			qt_square[2] = (1 + rm_in[2 * rm_ld + 2] - rm_in[0] - rm_in[rm_ld + 1]) / 4;
			qt_square[3] = (1 + rm_in[0] + rm_in[rm_ld + 1] + rm_in[2 * rm_ld + 2]) / 4;

			for (int i = 0; i < 4; ++i)rq_out[i] = qt_square[i] < 0 ? 0 : std::sqrt(qt_square[i]);

			int i = std::max_element(rq_out, rq_out + 4) - rq_out;
			int jkl[3]{ (i + 1) % 4 ,(i + 2) % 4 ,(i + 3) % 4 };
			for (auto m : jkl) 
			{
				rq_out[m] = (rm_in[P[i][m] * rm_ld + Q[i][m]] + T[i][m] * rm_in[Q[i][m] * rm_ld + P[i][m]])<0 ? -rq_out[m] : rq_out[m];
			}
			
			// 将rq_out[3]置为正
			for (auto m = 0; m < 4;++m)rq_out[m] = rq_out[3] < 0 ? -rq_out[m] : rq_out[m];
		}
		auto s_pp2pm(const double *pp_in, double *pm_out) noexcept->void
		{
			// 补充默认参数 //
			pp_in = pp_in ? pp_in : default_pp();
			double pm_out_default[16];
			pm_out = pm_out ? pm_out : pm_out_default;

			// 正式开始计算 //
			pm_out[3] = pp_in[0];
			pm_out[7] = pp_in[1];
			pm_out[11] = pp_in[2];
		}
		auto s_pm2pp(const double *pm_in, double *pp_out) noexcept->void
		{
			// 补充默认参数 //
			pm_in = pm_in ? pm_in : default_pm();
			double pp_out_default[3];
			pp_out = pp_out ? pp_out : pp_out_default;

			// 正式开始计算 //
			pp_out[0] = pm_in[3];
			pp_out[1] = pm_in[7];
			pp_out[2] = pm_in[11];
		}
		auto s_re2pm(const double *re_in, double *pm_out, const char *eu_type_in) noexcept->void 
		{ 
			// 补充默认参数 //
			re_in = re_in ? re_in : default_re();
			double pm_out_default[16]{1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};
			pm_out = pm_out ? pm_out : pm_out_default;

			// 正式开始计算 //
			s_re2rm(re_in, pm_out, eu_type_in, 4); 
		};
		auto s_pm2re(const double *pm_in, double *re_out, const char *eu_type_in) noexcept->void 
		{ 
			// 补充默认参数 //
			pm_in = pm_in ? pm_in : default_pm();
			double re_out_default[3];
			re_out = re_out ? re_out : re_out_default;

			// 正式开始计算 //
			s_rm2re(pm_in, re_out, eu_type_in, 4); 
		};
		auto s_rq2pm(const double *rq_in, double *pm_out) noexcept->void 
		{ 
			// 补充默认参数 //
			rq_in = rq_in ? rq_in : default_rq();
			double pm_out_default[16];
			pm_out = pm_out ? pm_out : pm_out_default;

			// 正式开始计算 //
			s_rq2rm(rq_in, pm_out, 4); 
		};
		auto s_pm2rq(const double *pm_in, double *rq_out) noexcept->void 
		{ 
			// 补充默认参数 //
			pm_in = pm_in ? pm_in : default_pm();
			double rq_out_default[4];
			rq_out = rq_out ? rq_out : rq_out_default;

			// 正式开始计算 //
			s_rm2rq(pm_in, rq_out, 4); 
		};
		auto s_rm2pm(const double *rm_in, double *pm_out, std::size_t rm_ld) noexcept->void
		{
			// 补充默认参数 //
			rm_in = rm_in ? rm_in : default_rm();
			double pm_out_default[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
			pm_out = pm_out ? pm_out : pm_out_default;
			
			// 正式开始计算 //
			std::copy(rm_in, rm_in + 3, pm_out);
			std::copy(rm_in + rm_ld, rm_in + rm_ld + 3, pm_out + 4);
			std::copy(rm_in + rm_ld * 2, rm_in + rm_ld * 2 + 3, pm_out + 8);
		}
		auto s_pm2rm(const double *pm_in, double *rm_out, std::size_t rm_ld) noexcept->void
		{
			// 补充默认参数 //
			pm_in = pm_in ? pm_in : default_pm();
			double rm_out_default[9];
			rm_out = rm_out ? rm_out : rm_out_default;

			// 正式开始计算 //
			std::copy(pm_in, pm_in + 3, rm_out);
			std::copy(pm_in + 4, pm_in + 7, rm_out + rm_ld);
			std::copy(pm_in + 8, pm_in + 11, rm_out + rm_ld * 2);
		}
		auto s_pe2pm(const double *pe_in, double *pm_out, const char *eu_type_in) noexcept->void
		{
			// 补充默认参数 //
			pe_in = pe_in ? pe_in : default_pe();
			double pm_out_default[16];
			pm_out = pm_out ? pm_out : pm_out_default;

			// 正式开始计算 //
			s_pp2pm(pe_in, pm_out);
			s_re2pm(pe_in + 3, pm_out, eu_type_in);

			pm_out[12] = 0;
			pm_out[13] = 0;
			pm_out[14] = 0;
			pm_out[15] = 1;
		}
		auto s_pm2pe(const double *pm_in, double *pe_out, const char *eu_type_in) noexcept->void
		{
			// 补充默认参数 //
			pm_in = pm_in ? pm_in : default_pm();
			double pe_out_default[9];
			pe_out = pe_out ? pe_out : pe_out_default;
			
			// 正式开始计算 //
			s_pm2pp(pm_in, pe_out);
			s_pm2re(pm_in, pe_out + 3, eu_type_in);
		}
		auto s_pq2pm(const double *pq_in, double *pm_out) noexcept->void
		{
			// 补充默认参数 //
			pq_in = pq_in ? pq_in : default_pq();
			double pm_out_default[16];
			pm_out = pm_out ? pm_out : pm_out_default;

			// 正式开始计算 //
			s_pp2pm(pq_in, pm_out);
			s_rq2pm(pq_in + 3, pm_out);

			pm_out[12] = 0;
			pm_out[13] = 0;
			pm_out[14] = 0;
			pm_out[15] = 1;
		}
		auto s_pm2pq(const double *pm_in, double *pq_out) noexcept->void
		{
			// 补充默认参数 //
			pm_in = pm_in ? pm_in : default_pm();
			double pq_out_default[7];
			pq_out = pq_out ? pq_out : pq_out_default;

			// 正式开始计算 //
			s_pm2pp(pm_in, pq_out);
			s_pm2rq(pm_in, pq_out + 3);
		}

		auto s_we2wa(const double *re_in, const double *we_in, double *wa_out, const char *eu_type_in) noexcept->void 
		{
			// 补充默认参数 //
			re_in = re_in ? re_in : default_re();
			we_in = we_in ? we_in : default_we();
			double wa_out_default[3];
			wa_out = wa_out ? wa_out : wa_out_default;

			// 正式开始计算 //
			static const double P[3][3] = { { 0, -1, 1 },{ 1, 0, -1 },{ -1, 1, 0 } };
			static const double Q[3][3] = { { 1, 0, 0 },{ 0, 1, 0 },{ 0, 0, 1 } };

			const int a = eu_type_in[0] - '1';
			const int b = eu_type_in[1] - '1';
			const int c = eu_type_in[2] - '1';
			const int d = 3 - a - b;
			const int e = 3 - b - c;

			double axis[3][3];

			axis[a][0] = 1.0;
			axis[b][0] = 0.0;
			axis[d][0] = 0.0;

			axis[a][1] = 0;
			axis[b][1] = std::cos(re_in[0]);
			axis[d][1] = P[d][b] * std::sin(re_in[0]);

			axis[a][2] = c == a ? std::cos(re_in[1]) : P[a][d] * std::sin(re_in[1]);
			axis[b][2] = c == a ? -std::sin(re_in[0])*std::sin(re_in[1]) : P[b][d] * std::sin(re_in[0])* std::cos(re_in[1]);
			axis[d][2] = c == a ? P[d][a] * std::cos(re_in[0])* std::sin(re_in[1]) : std::cos(re_in[0])* std::cos(re_in[1]);

			s_mdm(3, 1, 3, *axis, 3, we_in, 1, wa_out, 1);
		}
		auto s_wa2we(const double *wa_in, const double *re_in, double *we_out, const char *eu_type_in) noexcept->void
		{
			// 补充默认参数 //
			wa_in = wa_in ? wa_in : default_wa();
			re_in = re_in ? re_in : default_re();
			double we_out_default[3];
			we_out = we_out ? we_out : we_out_default;

			// 正式开始计算 //
			static const double P[3][3] = { { 0, -1, 1 },{ 1, 0, -1 },{ -1, 1, 0 } };
			static const double Q[3][3] = { { 1, 0, 0 },{ 0, 1, 0 },{ 0, 0, 1 } };

			const int a = eu_type_in[0] - '1';
			const int b = eu_type_in[1] - '1';
			const int c = eu_type_in[2] - '1';
			const int d = 3 - a - b;
			const int e = 3 - b - c;

			double axis[3][3];

			axis[a][0] = 1.0;
			axis[b][0] = 0.0;
			axis[d][0] = 0.0;

			axis[a][1] = 0;
			axis[b][1] = std::cos(re_in[0]);
			axis[d][1] = P[d][b] * std::sin(re_in[0]);

			axis[a][2] = c == a ? std::cos(re_in[1]) : P[a][d] * std::sin(re_in[1]);
			axis[b][2] = c == a ? -std::sin(re_in[0])*std::sin(re_in[1]) : P[b][d] * std::sin(re_in[0])* std::cos(re_in[1]);
			axis[d][2] = c == a ? P[d][a] * std::cos(re_in[0])* std::sin(re_in[1]) : std::cos(re_in[0])* std::cos(re_in[1]);

			we_out[1] = (wa_in[b] * axis[d][2] - wa_in[d] * axis[b][2]) / (axis[b][1] * axis[d][2] - axis[d][1] * axis[b][2]);
			we_out[2] = (wa_in[d] * axis[b][1] - wa_in[b] * axis[d][1]) / (axis[d][2] * axis[b][1] - axis[b][2] * axis[d][1]);
			we_out[0] = (wa_in[a] - axis[a][2] * we_out[2]);
		}
		auto s_wq2wa(const double *rq_in, const double *wq_in, double *wa_out) noexcept->void
		{
			// 补充默认参数 //
			rq_in = rq_in ? rq_in : default_rq();
			wq_in = wq_in ? wq_in : default_wq();
			double wa_out_default[3];
			wa_out = wa_out ? wa_out : wa_out_default;

			// 正式开始计算 //
			double p11 = 2 * wq_in[0] * rq_in[0];
			double p22 = 2 * wq_in[1] * rq_in[1];
			double p33 = 2 * wq_in[2] * rq_in[2];
			double p12 = wq_in[0] * rq_in[1] + rq_in[0] * wq_in[1];
			double p13 = wq_in[0] * rq_in[2] + rq_in[0] * wq_in[2];
			double p23 = wq_in[1] * rq_in[2] + rq_in[1] * wq_in[2];
			double p41 = wq_in[3] * rq_in[0] + rq_in[3] * wq_in[0];
			double p42 = wq_in[3] * rq_in[1] + rq_in[3] * wq_in[1];
			double p43 = wq_in[3] * rq_in[2] + rq_in[3] * wq_in[2];

			double rm[3][3];
			s_rq2rm(rq_in, *rm);

			wa_out[0] = 2 * ((p13 - p42)*rm[1][0] + (p23 + p41)*rm[1][1] - (p11 + p22)*rm[1][2]);
			wa_out[1] = 2 * (-(p22 + p33)*rm[2][0] + (p12 - p43)*rm[2][1] + (p13 + p42)*rm[2][2]);
			wa_out[2] = 2 * ((p12 + p43)*rm[0][0] - (p11 + p33)*rm[0][1] + (p23 - p41)*rm[0][2]);
		}
		auto s_wa2wq(const double *wa_in, const double *rq_in, double *wq_out) noexcept->void
		{
			// 补充默认参数 //
			wa_in = wa_in ? wa_in : default_wa();
			rq_in = rq_in ? rq_in : default_rq();
			double wq_out_default[3];
			wq_out = wq_out ? wq_out : wq_out_default;

			// 正式开始计算 //

			static const double S[4][3]{ { 1,-1,-1 },{ -1,1,-1 },{ -1,-1,1 },{ 1,1,1 } };
			static const double T[4][4]{ { 0,1,1,-1 },{ 1,0,1,-1 },{ 1,1,0,-1 },{ -1,-1,-1,0 } };
			static const int P[4][4]{ { -1,0,0,2 },{ 1,-1,1,0 },{ 2,2,-1,1 },{ 2,0,1,-1 } };
			static const int Q[4][4]{ { -1,1,2,1 },{ 0,-1,2,2 },{ 0,1,-1,0 },{ 1,2,0,-1 } };

			double rm[3][3], wm[3][3];

			s_rq2rm(rq_in, *rm);
			s_wa2wm(wa_in, *rm, *wm);

			int i = std::max_element(rq_in, rq_in + 4, [](double a, double b) {return std::abs(a) < std::abs(b); }) - rq_in;
			int jkl[3]{ (i + 1) % 4 ,(i + 2) % 4 ,(i + 3) % 4 };

			wq_out[i] = (S[i][0] * wm[0][0] + S[i][1] * wm[1][1] + S[i][2] * wm[2][2]) / 8 / rq_in[i];

			for (auto m : jkl) wq_out[m] = (wm[P[i][m]][Q[i][m]] + T[i][m] * wm[Q[i][m]][P[i][m]] - 4 * rq_in[m] * wq_out[i]) / 4 / rq_in[i];
		}
		auto s_wm2wa(const double *rm_in, const double *wm_in, double *wa_out, std::size_t rm_ld, std::size_t wm_ld) noexcept->void
		{
			// 补充默认参数 //
			rm_in = rm_in ? rm_in : default_rm();
			wm_in = wm_in ? wm_in : default_wm();
			double wa_out_default[3];
			wa_out = wa_out ? wa_out : wa_out_default;

			// 正式开始计算 //

			// dR = w x R
			// w x = dR * R' 

			wa_out[0] = wm_in[2 * wm_ld + 0] * rm_in[1 * rm_ld + 0] + wm_in[2 * wm_ld + 1] * rm_in[1 * rm_ld + 1] + wm_in[2 * wm_ld + 2] * rm_in[1 * rm_ld + 2];
			wa_out[1] = wm_in[0 * wm_ld + 0] * rm_in[2 * rm_ld + 0] + wm_in[0 * wm_ld + 1] * rm_in[2 * rm_ld + 1] + wm_in[0 * wm_ld + 2] * rm_in[2 * rm_ld + 2];
			wa_out[2] = wm_in[1 * wm_ld + 0] * rm_in[0 * rm_ld + 0] + wm_in[1 * wm_ld + 1] * rm_in[0 * rm_ld + 1] + wm_in[1 * wm_ld + 2] * rm_in[0 * rm_ld + 2];
		}
		auto s_wa2wm(const double *wa_in, const double *rm_in, double *wm_out, std::size_t rm_ld, std::size_t wm_ld) noexcept->void
		{
			// 补充默认参数 //
			wa_in = wa_in ? wa_in : default_wa();
			rm_in = rm_in ? rm_in : default_rm();
			double wm_out_default[9];
			wm_out = wm_out ? wm_out : wm_out_default;

			// 正式开始计算 //
			s_c3_n(3, wa_in, rm_in, rm_ld, wm_out, wm_ld);
		}
		auto s_vp2vs(const double *pp_in, const double *vp_in, double *vs_out) noexcept->void
		{
			// 补充默认参数 //
			vp_in = vp_in ? vp_in : default_vp();
			pp_in = pp_in ? pp_in : default_pp();
			double vs_out_default[6]{ 0,0,0,0,0,0 };
			vs_out = vs_out ? vs_out : vs_out_default;

			// 正式开始计算 //
			s_c3(pp_in, vs_out + 3, vs_out);
			s_va(3, vp_in, vs_out);
		}
		auto s_vs2vp(const double *vs_in, const double *pp_in, double *vp_out) noexcept->void
		{
			// 补充默认参数 //
			vs_in = vs_in ? vs_in : default_vs();
			pp_in = pp_in ? pp_in : default_pp();
			double vp_out_default[3];
			vp_out = vp_out ? vp_out : vp_out_default;

			// 正式开始计算 //
			s_c3(vs_in + 3, pp_in, vp_out);
			s_va(3, vs_in, vp_out);
		}
		auto s_we2vs(const double *re_in, const double *we_in, double *vs_out, const char *eu_type_in) noexcept->void
		{
			// 补充默认参数 //
			re_in = re_in ? re_in : default_re();
			we_in = we_in ? we_in : default_we();
			double vs_out_default[6]{ 0,0,0,0,0,0 };
			vs_out = vs_out ? vs_out : vs_out_default;

			// 正式开始计算 //
			s_we2wa(re_in, we_in, vs_out + 3, eu_type_in);
		}
		auto s_vs2we(const double *vs_in, const double *re_in, double *we_out, const char *eu_type_in) noexcept->void
		{
			// 补充默认参数 //
			vs_in = vs_in ? vs_in : default_vs();
			re_in = re_in ? re_in : default_re();
			double we_out_default[3];
			we_out = we_out ? we_out : we_out_default;

			// 正式开始计算 //
			s_wa2we(vs_in + 3, re_in, we_out, eu_type_in);
		}
		auto s_wq2vs(const double *rq_in, const double *wq_in, double *vs_out) noexcept->void
		{
			// 补充默认参数 //
			rq_in = rq_in ? rq_in : default_rq();
			wq_in = wq_in ? wq_in : default_wq();
			double vs_out_default[6]{ 0,0,0,0,0,0 };
			vs_out = vs_out ? vs_out : vs_out_default;

			// 正式开始计算 //
			s_wq2wa(rq_in, wq_in, vs_out + 3);
		}
		auto s_vs2wq(const double *vs_in, const double *rq_in, double *wq_out) noexcept->void
		{
			// 补充默认参数 //
			vs_in = vs_in ? vs_in : default_vs();
			rq_in = rq_in ? rq_in : default_rq();
			double wq_out_default[4];
			wq_out = wq_out ? wq_out : wq_out_default;

			// 正式开始计算 //
			s_wa2wq(vs_in + 3, rq_in, wq_out);
		}
		auto s_wm2vs(const double *rm_in, const double *wm_in, double *vs_out, std::size_t rm_ld, std::size_t wm_ld) noexcept->void
		{
			// 补充默认参数 //
			rm_in = rm_in ? rm_in : default_rm();
			wm_in = wm_in ? wm_in : default_wm();
			double vs_out_default[6]{ 0,0,0,0,0,0 };
			vs_out = vs_out ? vs_out : vs_out_default;

			// 正式开始计算 //
			s_wm2wa(rm_in, wm_in, vs_out + 3, rm_ld, wm_ld);
		}
		auto s_vs2wm(const double *vs_in, const double *rm_in, double *wm_out, std::size_t rm_ld, std::size_t wm_ld) noexcept->void
		{
			// 补充默认参数 //
			vs_in = vs_in ? vs_in : default_vs();
			rm_in = rm_in ? rm_in : default_rm();
			double wm_out_default[9];
			wm_out = wm_out ? wm_out : wm_out_default;

			// 正式开始计算 //
			s_wa2wm(vs_in + 3, rm_in, wm_out, rm_ld, wm_ld);
		}
		auto s_wa2vs(const double *wa_in, double *vs_out) noexcept->void
		{
			// 补充默认参数 //
			wa_in = wa_in ? wa_in : default_wa();
			double vs_out_default[6];
			vs_out = vs_out ? vs_out : vs_out_default;

			// 正式开始计算 //
			std::copy(wa_in, wa_in + 3, vs_out + 3);
		}
		auto s_vs2wa(const double *vs_in, double *wa_out) noexcept->void
		{
			// 补充默认参数 //
			vs_in = vs_in ? vs_in : default_vs();
			double wa_out_default[3];
			wa_out = wa_out ? wa_out : wa_out_default;

			// 正式开始计算 //
			std::copy(vs_in + 3, vs_in + 6, wa_out);
		}
		auto s_ve2vs(const double *pe_in, const double *ve_in, double *vs_out, const char *eu_type_in) noexcept->void
		{
			// 补充默认参数 //
			pe_in = pe_in ? pe_in : default_pe();
			ve_in = ve_in ? ve_in : default_ve();
			double vs_out_default[6];
			vs_out = vs_out ? vs_out : vs_out_default;

			// 正式开始计算 //
			s_we2vs(pe_in + 3, ve_in + 3, vs_out, eu_type_in);
			s_vp2vs(pe_in, ve_in, vs_out);
		}
		auto s_vs2ve(const double *vs_in, const double *pe_in, double *ve_out, const char *eu_type_in) noexcept->void
		{
			// 补充默认参数 //
			vs_in = vs_in ? vs_in : default_vs();
			pe_in = pe_in ? pe_in : default_pe();
			double ve_out_default[6];
			ve_out = ve_out ? ve_out : ve_out_default;

			// 正式开始计算 //
			s_vs2we(vs_in, pe_in + 3, ve_out + 3, eu_type_in);
			s_vs2vp(vs_in, pe_in, ve_out);
		}
		auto s_vq2vs(const double *pq_in, const double *vq_in, double *vs_out) noexcept->void
		{
			// 补充默认参数 //
			pq_in = pq_in ? pq_in : default_pq();
			vq_in = vq_in ? vq_in : default_vq();
			double vs_out_default[6];
			vs_out = vs_out ? vs_out : vs_out_default;

			// 正式开始计算 //
			s_wq2vs(pq_in + 3, vq_in + 3, vs_out);
			s_vp2vs(pq_in, vq_in, vs_out);
		}
		auto s_vs2vq(const double *vs_in, const double *pq_in, double *vq_out) noexcept->void
		{
			// 补充默认参数 //
			vs_in = vs_in ? vs_in : default_vs();
			pq_in = pq_in ? pq_in : default_pq();
			double vq_out_default[7];
			vq_out = vq_out ? vq_out : vq_out_default;

			// 正式开始计算 //
			s_vs2wq(vs_in, pq_in + 3, vq_out + 3);
			s_vs2vp(vs_in, pq_in, vq_out);
		}
		auto s_vm2vs(const double *pm_in, const double *vm_in, double *vs_out) noexcept->void
		{
			// 补充默认参数 //
			pm_in = pm_in ? pm_in : default_pm();
			vm_in = vm_in ? vm_in : default_vm();
			double vs_out_default[6];
			vs_out = vs_out ? vs_out : vs_out_default;

			// 正式开始计算 //
			double pp[3]{ pm_in[3],pm_in[7],pm_in[11] };
			double vp[3]{ vm_in[3],vm_in[7],vm_in[11] };
			s_wm2vs(pm_in, vm_in, vs_out, 4, 4);
			s_vp2vs(pp, vp, vs_out);
		}
		auto s_vs2vm(const double *vs_in, const double *pm_in, double *vm_out) noexcept->void
		{
			// 补充默认参数 //
			vs_in = vs_in ? vs_in : default_vs();
			pm_in = pm_in ? pm_in : default_pm();
			double vm_out_default[16];
			vm_out = vm_out ? vm_out : vm_out_default;

			// 正式开始计算 //
			double pp[3]{ pm_in[3],pm_in[7],pm_in[11] };
			double vp[3];
			s_vs2wm(vs_in, pm_in, vm_out, 4, 4);
			s_vs2vp(vs_in, pp, vp);
			
			vm_out[3] = vp[0];
			vm_out[7] = vp[1];
			vm_out[11] = vp[2];

			std::fill(vm_out + 12, vm_out + 16, 0);
		}
		auto s_va2vs(const double *pp_in, const double *va_in, double *vs_out) noexcept->void
		{
			// 补充默认参数 //
			pp_in = pp_in ? pp_in : default_pp();
			va_in = va_in ? va_in : default_va();
			double vs_out_default[6];
			vs_out = vs_out ? vs_out : vs_out_default;

			// 正式开始计算 //
			s_wa2vs(va_in + 3, vs_out);
			s_vp2vs(pp_in, va_in, vs_out);
		}
		auto s_vs2va(const double *vs_in, const double *pp_in, double *va_out) noexcept->void
		{
			// 补充默认参数 //
			vs_in = vs_in ? vs_in : default_vs();
			pp_in = pp_in ? pp_in : default_pp();
			double ve_out_default[6];
			va_out = va_out ? va_out : ve_out_default;

			// 正式开始计算 //
			s_vs2wa(vs_in, va_out + 3);
			s_vs2vp(vs_in, pp_in, va_out);
		}

		auto s_xe2xa(const double *re_in, const double *we_in, const double *xe_in, double *xa_out, double *wa_out, const char *eu_type_in) noexcept->void
		{
			// 补充默认参数 //
			re_in = re_in ? re_in : default_re();
			we_in = we_in ? we_in : default_we();
			xe_in = xe_in ? xe_in : default_xe();
			double wa_out_default[3], xa_out_default[3];
			wa_out = wa_out ? wa_out : wa_out_default;
			xa_out = xa_out ? xa_out : xa_out_default;

			// 正式开始计算 //
			static const double P[3][3] = { { 0, -1, 1 },{ 1, 0, -1 },{ -1, 1, 0 } };
			static const double Q[3][3] = { { 1, 0, 0 },{ 0, 1, 0 },{ 0, 0, 1 } };

			const int a = eu_type_in[0] - '1';
			const int b = eu_type_in[1] - '1';
			const int c = eu_type_in[2] - '1';
			const int d = 3 - a - b;
			const int e = 3 - b - c;

			const double s1 = std::sin(re_in[0]);
			const double c1 = std::cos(re_in[0]);
			const double s2 = std::sin(re_in[1]);
			const double c2 = std::cos(re_in[1]);

			const double ds1 = c1*we_in[0];
			const double dc1 = -s1*we_in[0];
			const double ds2 = c2*we_in[1];
			const double dc2 = -s2*we_in[1];
			
			const double Ab1 = c1;
			const double Ad1 = P[d][b] * s1;
			const double Aa2 = c == a ? c2 : P[a][d] * s2;
			const double Ab2 = c == a ? -s1*s2 : P[b][d]*s1*c2;
			const double Ad2 = c == a ? P[d][a] * c1* s2 : c1* c2;

			const double dAb1 = dc1;
			const double dAd1 = P[d][b] * ds1;
			const double dAa2 = c == a ? dc2 : P[a][d] * ds2;
			const double dAb2 = c == a ? -ds1*s2 - s1*ds2 : P[b][d] * (ds1*c2 + s1*dc2);
			const double dAd2 = c == a ? P[d][a] * (dc1* s2 + c1*ds2) : dc1*c2 + c1*dc2;

			wa_out[a] = we_in[0] + Aa2*we_in[2];
			wa_out[b] = Ab1*we_in[1] + Ab2*we_in[2];
			wa_out[d] = Ad1*we_in[1] + Ad2*we_in[2];

			xa_out[a] = xe_in[0] + Aa2*xe_in[2] + dAa2*we_in[2];
			xa_out[b] = Ab1*xe_in[1] + Ab2*xe_in[2] + dAb1*we_in[1] + dAb2*we_in[2];
			xa_out[d] = Ad1*xe_in[1] + Ad2*xe_in[2] + dAd1*we_in[1] + dAd2*we_in[2];
		}
		auto s_xa2xe(const double *wa_in, const double *xa_in, const double *re_in, double *xe_out, double *we_out, const char *eu_type_in) noexcept->void
		{
			// 补充默认参数 //
			wa_in = wa_in ? wa_in : default_wa();
			xa_in = xa_in ? xa_in : default_xa();
			re_in = re_in ? re_in : default_re();
			
			double we_out_default[3], xe_out_default[3];
			we_out = we_out ? we_out : we_out_default;
			xe_out = xe_out ? xe_out : xe_out_default;

			// 正式开始计算 //
			static const double P[3][3] = { { 0, -1, 1 },{ 1, 0, -1 },{ -1, 1, 0 } };
			static const double Q[3][3] = { { 1, 0, 0 },{ 0, 1, 0 },{ 0, 0, 1 } };

			const int a = eu_type_in[0] - '1';
			const int b = eu_type_in[1] - '1';
			const int c = eu_type_in[2] - '1';
			const int d = 3 - a - b;
			const int e = 3 - b - c;

			const double s1 = std::sin(re_in[0]);
			const double c1 = std::cos(re_in[0]);
			const double s2 = std::sin(re_in[1]);
			const double c2 = std::cos(re_in[1]);

			const double Ab1 = c1;
			const double Ad1 = P[d][b] * s1;
			const double Aa2 = c == a ? c2 : P[a][d] * s2;
			const double Ab2 = c == a ? -s1*s2 : P[b][d] * s1*c2;
			const double Ad2 = c == a ? P[d][a] * c1* s2 : c1* c2;

			const double M = (Ab1 * Ad2 - Ad1 * Ab2);
			const double N = (Ad2 * Ab1 - Ab2 * Ad1);

			we_out[1] = (wa_in[b] * Ad2 - wa_in[d] * Ab2) / M;
			we_out[2] = (wa_in[d] * Ab1 - wa_in[b] * Ad1) / N;
			we_out[0] = (wa_in[a] - Aa2 * we_out[2]);

			const double ds1 = c1*we_out[0];
			const double dc1 = -s1*we_out[0];
			const double ds2 = c2*we_out[1];
			const double dc2 = -s2*we_out[1];

			const double dAb1 = dc1;
			const double dAd1 = P[d][b] * ds1;
			const double dAa2 = c == a ? dc2 : P[a][d] * ds2;
			const double dAb2 = c == a ? -ds1*s2 - s1*ds2 : P[b][d] * (ds1*c2 + s1*dc2);
			const double dAd2 = c == a ? P[d][a] * (dc1* s2 + c1*ds2) : dc1*c2 + c1*dc2;

			const double ba = xa_in[a] - dAa2* we_out[2];
			const double bb = xa_in[b] - dAb1* we_out[1] - dAb2* we_out[2];
			const double bd = xa_in[d] - dAd1* we_out[1] - dAd2* we_out[2];

			xe_out[1] = (bb * Ad2 - bd * Ab2) / M;
			xe_out[2] = (bd * Ab1 - bb * Ad1) / N;
			xe_out[0] = (ba - Aa2 * xe_out[2]);
		}
		auto s_xq2xa(const double *rq_in, const double *wq_in, const double *xq_in, double *xa_out, double *wa_out) noexcept->void
		{
			// 补充默认参数 //
			rq_in = rq_in ? rq_in : default_rq();
			wq_in = wq_in ? wq_in : default_wq();
			xq_in = xq_in ? xq_in : default_xq();
			double wa_out_default[3];
			double xa_out_default[3];
			wa_out = wa_out ? wa_out : wa_out_default;
			xa_out = xa_out ? xa_out : xa_out_default;

			// 正式开始计算 //
			double p11 = 2 * wq_in[0] * rq_in[0];
			double p22 = 2 * wq_in[1] * rq_in[1];
			double p33 = 2 * wq_in[2] * rq_in[2];
			double p12 = wq_in[0] * rq_in[1] + rq_in[0] * wq_in[1];
			double p13 = wq_in[0] * rq_in[2] + rq_in[0] * wq_in[2];
			double p23 = wq_in[1] * rq_in[2] + rq_in[1] * wq_in[2];
			double p41 = wq_in[3] * rq_in[0] + rq_in[3] * wq_in[0];
			double p42 = wq_in[3] * rq_in[1] + rq_in[3] * wq_in[1];
			double p43 = wq_in[3] * rq_in[2] + rq_in[3] * wq_in[2];

			double rm[3][3], wm[3][3];
			s_rq2rm(rq_in, *rm);

			wa_out[0] = 2 * ((p13 - p42)*rm[1][0] + (p23 + p41)*rm[1][1] - (p11 + p22)*rm[1][2]);
			wa_out[1] = 2 * (-(p22 + p33)*rm[2][0] + (p12 - p43)*rm[2][1] + (p13 + p42)*rm[2][2]);
			wa_out[2] = 2 * ((p12 + p43)*rm[0][0] - (p11 + p33)*rm[0][1] + (p23 - p41)*rm[0][2]);

			s_wa2wm(wa_out, *rm, *wm);

			double t11 = 2 * (xq_in[0] * rq_in[0] + wq_in[0] * wq_in[0]);
			double t22 = 2 * (xq_in[1] * rq_in[1] + wq_in[1] * wq_in[1]);
			double t33 = 2 * (xq_in[2] * rq_in[2] + wq_in[2] * wq_in[2]);
			double t12 = xq_in[0] * rq_in[1] + 2 * wq_in[0] * wq_in[1] + rq_in[0] * xq_in[1];
			double t13 = xq_in[0] * rq_in[2] + 2 * wq_in[0] * wq_in[2] + rq_in[0] * xq_in[2];
			double t23 = xq_in[1] * rq_in[2] + 2 * wq_in[1] * wq_in[2] + rq_in[1] * xq_in[2];
			double t41 = xq_in[3] * rq_in[0] + 2 * wq_in[3] * wq_in[0] + rq_in[3] * xq_in[0];
			double t42 = xq_in[3] * rq_in[1] + 2 * wq_in[3] * wq_in[1] + rq_in[3] * xq_in[1];
			double t43 = xq_in[3] * rq_in[2] + 2 * wq_in[3] * wq_in[2] + rq_in[3] * xq_in[2];

			xa_out[0] = 2 * ((t13 - t42)*rm[1][0] + (t23 + t41)*rm[1][1] - (t11 + t22)*rm[1][2]) + 2 * ((p13 - p42)*wm[1][0] + (p23 + p41)*wm[1][1] - (p11 + p22)*wm[1][2]);
			xa_out[1] = 2 * (-(t22 + t33)*rm[2][0] + (t12 - t43)*rm[2][1] + (t13 + t42)*rm[2][2]) + 2 * (-(p22 + p33)*wm[2][0] + (p12 - p43)*wm[2][1] + (p13 + p42)*wm[2][2]);
			xa_out[2] = 2 * ((t12 + t43)*rm[0][0] - (t11 + t33)*rm[0][1] + (t23 - t41)*rm[0][2]) + 2 * ((p12 + p43)*wm[0][0] - (p11 + p33)*wm[0][1] + (p23 - p41)*wm[0][2]);
		}
		auto s_xa2xq(const double *wa_in, const double *xa_in, const double *rq_in, double *xq_out, double *wq_out) noexcept->void
		{
			// 补充默认参数 //
			wa_in = wa_in ? wa_in : default_wa();
			xa_in = xa_in ? xa_in : default_xa();
			rq_in = rq_in ? rq_in : default_rq();
			double xq_out_default[4];
			double wq_out_default[4];
			xq_out = xq_out ? xq_out : xq_out_default;
			wq_out = wq_out ? wq_out : wq_out_default;

			// 正式开始计算 //
			static const double S[4][3]{ { 1,-1,-1 },{ -1,1,-1 },{ -1,-1,1 },{ 1,1,1 } };
			static const double T[4][4]{ { 0,1,1,-1 },{ 1,0,1,-1 },{ 1,1,0,-1 },{ -1,-1,-1,0 } };
			static const int P[4][4]{ { -1,0,0,2 },{ 1,-1,1,0 },{ 2,2,-1,1 },{ 2,0,1,-1 } };
			static const int Q[4][4]{ { -1,1,2,1 },{ 0,-1,2,2 },{ 0,1,-1,0 },{ 1,2,0,-1 } };

			double rm[3][3], wm[3][3], xm[3][3];

			s_rq2rm(rq_in, *rm);
			s_xa2xm(wa_in, xa_in, *rm, *xm, *wm);

			int i = std::max_element(rq_in, rq_in + 4, [](double a, double b) {return std::abs(a) < std::abs(b); }) - rq_in;
			int jkl[3]{ (i + 1) % 4 ,(i + 2) % 4 ,(i + 3) % 4 };

			wq_out[i] = (S[i][0] * wm[0][0] + S[i][1] * wm[1][1] + S[i][2] * wm[2][2]) / 8 / rq_in[i];
			xq_out[i] = (S[i][0] * xm[0][0] + S[i][1] * xm[1][1] + S[i][2] * xm[2][2] - 8 * wq_out[i] * wq_out[i]) / 8 / rq_in[i];

			for (auto m : jkl)
			{
				wq_out[m] = (wm[P[i][m]][Q[i][m]] + T[i][m] * wm[Q[i][m]][P[i][m]] - 4 * rq_in[m] * wq_out[i]) / 4 / rq_in[i];
				xq_out[m] = (xm[P[i][m]][Q[i][m]] + T[i][m] * xm[Q[i][m]][P[i][m]] - 8 * wq_out[m] * wq_out[i] - 4 * rq_in[m] * xq_out[i]) / 4 / rq_in[i];
			}
		}
		auto s_xm2xa(const double *rm_in, const double *wm_in, const double *xm_in, double *xa_out, double *wa_out, std::size_t rm_ld, std::size_t wm_ld, std::size_t xm_ld) noexcept->void
		{
			// 补充默认参数 //
			rm_in = rm_in ? rm_in : default_rm();
			wm_in = wm_in ? wm_in : default_wm();
			xm_in = xm_in ? xm_in : default_xm();
			double wa_out_default[3];
			double xa_out_default[3];
			wa_out = wa_out ? wa_out : wa_out_default;
			xa_out = xa_out ? xa_out : xa_out_default;

			// 正式开始计算 //

			// ddR = x x R + w x dR
			// x x = (ddR - w x dR) * R' 

			s_wm2wa(rm_in, wm_in, wa_out, rm_ld, wm_ld);

			double tem[9];
			std::copy(xm_in + 0 * xm_ld, xm_in + 0 * xm_ld + 3, tem + 0);
			std::copy(xm_in + 1 * xm_ld, xm_in + 1 * xm_ld + 3, tem + 3);
			std::copy(xm_in + 2 * xm_ld, xm_in + 2 * xm_ld + 3, tem + 6);
			
			s_c3_n(3, -1, wa_out, wm_in, wm_ld, 1, tem, 3);

			xa_out[0] = tem[2 * 3 + 0] * rm_in[1 * rm_ld + 0] + tem[2 * 3 + 1] * rm_in[1 * rm_ld + 1] + tem[2 * 3 + 2] * rm_in[1 * rm_ld + 2];
			xa_out[1] = tem[0 * 3 + 0] * rm_in[2 * rm_ld + 0] + tem[0 * 3 + 1] * rm_in[2 * rm_ld + 1] + tem[0 * 3 + 2] * rm_in[2 * rm_ld + 2];
			xa_out[2] = tem[1 * 3 + 0] * rm_in[0 * rm_ld + 0] + tem[1 * 3 + 1] * rm_in[0 * rm_ld + 1] + tem[1 * 3 + 2] * rm_in[0 * rm_ld + 2];
		}
		auto s_xa2xm(const double *wa_in, const double *xa_in, const double *rm_in, double *xm_out, double *wm_out, std::size_t rm_ld, std::size_t wm_ld, std::size_t xm_ld) noexcept->void
		{
			// 补充默认参数 //
			wa_in = wa_in ? wa_in : default_wa();
			xa_in = xa_in ? xa_in : default_xa();
			rm_in = rm_in ? rm_in : default_rm();
			double wm_out_default[9];
			double xm_out_default[9];
			wm_out = wm_out ? wm_out : wm_out_default;
			xm_out = xm_out ? xm_out : xm_out_default;
			
			// 正式开始计算 //

			// ddR = x x R + w x dR
			// w x = dR * R' 

			s_wa2wm(wa_in, rm_in, wm_out, rm_ld, wm_ld);
			
			s_c3_n(3, wa_in, wm_out, wm_ld, xm_out, xm_ld);
			s_c3_n(3, 1, xa_in, rm_in, rm_ld, 1, xm_out, xm_ld);
		}
		auto s_ap2as(const double *pp_in, const double *vp_in, const double *ap_in, double *as_out, double *vs_out) noexcept->void
		{
			// 补充默认参数 //
			pp_in = pp_in ? pp_in : default_pp();
			vp_in = vp_in ? vp_in : default_vp();
			ap_in = ap_in ? ap_in : default_ap();
			double as_out_default[6]{ 0,0,0,0,0,0 };
			double vs_out_default[6]{ 0,0,0,0,0,0 };
			as_out = as_out ? as_out : as_out_default;
			vs_out = vs_out ? vs_out : vs_out_default;

			// 正式开始计算 //
			s_vp2vs(pp_in, vp_in, vs_out);

			std::copy(ap_in, ap_in + 3, as_out);
			s_c3(-1, vs_out + 3, vp_in, 1, as_out);
			s_c3(-1, as_out + 3, pp_in, 1, as_out);
		}
		auto s_as2ap(const double *vs_in, const double *as_in, const double *pp_in, double *ap_out, double *vp_out) noexcept->void
		{
			// 补充默认参数 //
			vs_in = vs_in ? vs_in : default_vs();
			as_in = as_in ? as_in : default_as();
			pp_in = pp_in ? pp_in : default_pp();
			double vp_out_default[3];
			double ap_out_default[3];
			ap_out = ap_out ? ap_out : ap_out_default;
			vp_out = vp_out ? vp_out : vp_out_default;

			// 正式开始计算 //
			s_vs2vp(vs_in, pp_in, vp_out);

			std::copy(as_in, as_in + 3, ap_out);
			s_c3(1, vs_in + 3, vp_out, 1, ap_out);
			s_c3(1, as_in + 3, pp_in, 1, ap_out);
		}
		auto s_xe2as(const double *re_in, const double *we_in, const double *xe_in, double *as_out, double *vs_out, const char *eu_type_in) noexcept->void
		{
			// 补充默认参数 //
			re_in = re_in ? re_in : default_re();
			we_in = we_in ? we_in : default_we();
			xe_in = xe_in ? xe_in : default_xe();
			double as_out_default[6]{ 0,0,0,0,0,0 };
			double vs_out_default[6]{ 0,0,0,0,0,0 };
			vs_out = vs_out ? vs_out : vs_out_default;
			as_out = as_out ? as_out : as_out_default;

			// 正式开始计算 //
			s_xe2xa(re_in, we_in, xe_in, as_out + 3, vs_out + 3, eu_type_in);
		}
		auto s_as2xe(const double *vs_in, const double *as_in, const double *re_in, double *xe_out, double *we_out, const char *eu_type_in) noexcept->void
		{
			// 补充默认参数 //
			vs_in = vs_in ? vs_in : default_vs();
			as_in = as_in ? as_in : default_as();
			re_in = re_in ? re_in : default_re();
			double we_out_default[3];
			double xe_out_default[3];
			we_out = we_out ? we_out : we_out_default;
			xe_out = xe_out ? xe_out : xe_out_default;

			// 正式开始计算 //
			s_xa2xe(vs_in + 3, as_in + 3, re_in, xe_out, we_out, eu_type_in);
		}
		auto s_xq2as(const double *rq_in, const double *wq_in, const double *xq_in, double *as_out, double *vs_out) noexcept->void
		{
			// 补充默认参数 //
			rq_in = rq_in ? rq_in : default_rq();
			wq_in = wq_in ? wq_in : default_wq();
			xq_in = xq_in ? xq_in : default_xq();
			double as_out_default[6]{ 0,0,0,0,0,0 };
			double vs_out_default[6]{ 0,0,0,0,0,0 };
			vs_out = vs_out ? vs_out : vs_out_default;
			as_out = as_out ? as_out : as_out_default;

			// 正式开始计算 //
			s_xq2xa(rq_in, wq_in, xq_in, as_out + 3, vs_out + 3);
		}
		auto s_as2xq(const double *vs_in, const double *as_in, const double *rq_in, double *xq_out, double *wq_out) noexcept->void
		{
			// 补充默认参数 //
			vs_in = vs_in ? vs_in : default_vs();
			as_in = as_in ? as_in : default_as();
			rq_in = rq_in ? rq_in : default_rq();
			double wq_out_default[4];
			double xq_out_default[4];
			wq_out = wq_out ? wq_out : wq_out_default;
			xq_out = xq_out ? xq_out : xq_out_default;

			// 正式开始计算 //
			s_xa2xq(vs_in + 3, as_in +3, rq_in, xq_out, wq_out);
		}
		auto s_xm2as(const double *rm_in, const double *wm_in, const double *xm_in, double *as_out, double *vs_out, std::size_t rm_ld, std::size_t wm_ld, std::size_t xm_ld) noexcept->void
		{
			// 补充默认参数 //
			rm_in = rm_in ? rm_in : default_rm();
			wm_in = wm_in ? wm_in : default_wm();
			xm_in = xm_in ? xm_in : default_xm();
			double vs_out_default[6]{ 0,0,0,0,0,0 };
			double as_out_default[6]{ 0,0,0,0,0,0 };
			vs_out = vs_out ? vs_out : vs_out_default;
			as_out = as_out ? as_out : as_out_default;

			// 正式开始计算 //
			s_xm2xa(rm_in, wm_in, xm_in, as_out + 3, vs_out + 3, rm_ld, wm_ld, xm_ld);
		}
		auto s_as2xm(const double *vs_in, const double *as_in, const double *rm_in, double *xm_out, double *wm_out, std::size_t rm_ld, std::size_t wm_ld, std::size_t xm_ld) noexcept->void
		{
			// 补充默认参数 //
			vs_in = vs_in ? vs_in : default_vs();
			as_in = as_in ? as_in : default_as();
			rm_in = rm_in ? rm_in : default_rm();
			double wm_out_default[9];
			double xm_out_default[9];
			wm_out = wm_out ? wm_out : wm_out_default;
			xm_out = xm_out ? xm_out : xm_out_default;

			// 正式开始计算 //
			s_xa2xm(vs_in + 3, as_in + 3, rm_in, xm_out, wm_out, rm_ld, wm_ld, xm_ld);
		}
		auto s_xa2as(const double *xa_in, double *as_out) noexcept->void
		{
			// 补充默认参数 //
			xa_in = xa_in ? xa_in : default_xa();
			double as_out_default[6]{ 0,0,0,0,0,0 };
			as_out = as_out ? as_out : as_out_default;

			// 正式开始计算 //
			std::copy(xa_in, xa_in + 3, as_out + 3);
		}
		auto s_as2xa(const double *as_in, double *xa_out) noexcept->void
		{
			// 补充默认参数 //
			as_in = as_in ? as_in : default_as();
			double xa_out_default[3];
			xa_out = xa_out ? xa_out : xa_out_default;

			// 正式开始计算 //
			std::copy(as_in + 3, as_in + 6, xa_out);
		}
		auto s_ae2as(const double *pe_in, const double *ve_in, const double *ae_in, double *as_out, double *vs_out, const char *eu_type_in) noexcept->void
		{
			// 补充默认参数 //
			pe_in = pe_in ? pe_in : default_pe();
			ve_in = ve_in ? ve_in : default_ve();
			ae_in = ae_in ? ae_in : default_ae();
			double vs_out_default[6];
			double as_out_default[6];
			vs_out = vs_out ? vs_out : vs_out_default;
			as_out = as_out ? as_out : as_out_default;

			// 正式开始计算 //
			s_xe2as(pe_in + 3, ve_in + 3, ae_in + 3, as_out, vs_out, eu_type_in);
			s_ap2as(pe_in, ve_in, ae_in, as_out, vs_out);
		}
		auto s_as2ae(const double *vs_in, const double *as_in, const double *pe_in, double *ae_out, double *ve_out, const char *eu_type_in) noexcept->void
		{
			// 补充默认参数 //
			vs_in = vs_in ? vs_in : default_vs();
			as_in = as_in ? as_in : default_as();
			pe_in = pe_in ? pe_in : default_pe();
			double ve_out_default[6];
			double ae_out_default[6];
			ve_out = ve_out ? ve_out : ve_out_default;
			ae_out = ae_out ? ae_out : ae_out_default;

			// 正式开始计算 //
			s_as2xe(vs_in, as_in, pe_in + 3, ae_out + 3, ve_out + 3, eu_type_in);
			s_as2ap(vs_in, as_in, pe_in, ae_out, ve_out);
		}
		auto s_aq2as(const double *pq_in, const double *vq_in, const double *aq_in, double *as_out, double *vs_out) noexcept->void
		{
			// 补充默认参数 //
			pq_in = pq_in ? pq_in : default_pq();
			vq_in = vq_in ? vq_in : default_vq();
			aq_in = aq_in ? aq_in : default_aq();
			double vs_out_default[6];
			double as_out_default[6];
			vs_out = vs_out ? vs_out : vs_out_default;
			as_out = as_out ? as_out : as_out_default;

			// 正式开始计算 //
			s_xq2as(pq_in + 3, vq_in + 3, aq_in + 3, as_out, vs_out);
			s_ap2as(pq_in, vq_in, aq_in, as_out, vs_out);

		}
		auto s_as2aq(const double *vs_in, const double *as_in, const double *pq_in, double *aq_out, double *vq_out) noexcept->void
		{
			// 补充默认参数 //
			vs_in = vs_in ? vs_in : default_vs();
			as_in = as_in ? as_in : default_as();
			pq_in = pq_in ? pq_in : default_pq();
			double vq_out_default[7];
			double aq_out_default[7];
			vq_out = vq_out ? vq_out : vq_out_default;
			aq_out = aq_out ? aq_out : aq_out_default;

			// 正式开始计算 //
			s_as2xq(vs_in, as_in, pq_in + 3, aq_out + 3, vq_out + 3);
			s_as2ap(vs_in, as_in, pq_in, aq_out, vq_out);
		}
		auto s_am2as(const double *pm_in, const double *vm_in, const double *am_in, double *as_out, double *vs_out) noexcept->void
		{
			// 补充默认参数 //
			pm_in = pm_in ? pm_in : default_pm();
			vm_in = vm_in ? vm_in : default_vm();
			am_in = am_in ? am_in : default_am();
			double vs_out_default[6];
			double as_out_default[6];
			vs_out = vs_out ? vs_out : vs_out_default;
			as_out = as_out ? as_out : as_out_default;

			// 正式开始计算 //
			double pp[3]{ pm_in[3],pm_in[7],pm_in[11] };
			double vp[3]{ vm_in[3],vm_in[7],vm_in[11] };
			double ap[3]{ am_in[3],am_in[7],am_in[11] };

			s_xm2as(pm_in, vm_in, am_in, as_out, vs_out, 4, 4, 4);
			s_ap2as(pp, vp, ap, as_out, vs_out);
		}
		auto s_as2am(const double *vs_in, const double *as_in, const double *pm_in, double *am_out, double *vm_out) noexcept->void
		{
			// 补充默认参数 //
			vs_in = vs_in ? vs_in : default_vs();
			as_in = as_in ? as_in : default_as();
			pm_in = pm_in ? pm_in : default_pm();
			double vm_out_default[16];
			double am_out_default[16];
			vm_out = vm_out ? vm_out : vm_out_default;
			am_out = am_out ? am_out : am_out_default;

			// 正式开始计算 //
			double pp[3]{ pm_in[3], pm_in[7], pm_in[11] };
			double vp[3], ap[3];

			s_as2ap(vs_in, as_in, pp, ap, vp);
			s_as2xm(vs_in, as_in, pm_in, am_out, vm_out, 4, 4, 4);

			vm_out[3] = vp[0];
			vm_out[7] = vp[1];
			vm_out[11] = vp[2];
			std::fill(vm_out + 12, vm_out + 16, 0);

			am_out[3] = ap[0];
			am_out[7] = ap[1];
			am_out[11] = ap[2];
			std::fill(am_out + 12, am_out + 16, 0);
		}
		auto s_aa2as(const double *pp_in, const double *va_in, const double *aa_in, double *as_out, double *vs_out) noexcept->void
		{
			// 补充默认参数 //
			pp_in = pp_in ? pp_in : default_pp();
			va_in = va_in ? va_in : default_va();
			aa_in = aa_in ? aa_in : default_aa();
			double as_out_default[6];
			double vs_out_default[6];
			vs_out = vs_out ? vs_out : vs_out_default;
			as_out = as_out ? as_out : as_out_default;

			// 正式开始计算 //
			s_wa2vs(va_in + 3, vs_out);
			s_xa2as(aa_in + 3, as_out);
			s_ap2as(pp_in, va_in, aa_in, as_out, vs_out);
		}
		auto s_as2aa(const double *vs_in, const double *as_in, const double *pp_in, double *aa_out, double *va_out) noexcept->void
		{
			// 补充默认参数 //
			vs_in = vs_in ? vs_in : default_vs();
			as_in = as_in ? as_in : default_as();
			pp_in = pp_in ? pp_in : default_pp();
			double ve_out_default[6];
			double ae_out_default[6];
			va_out = va_out ? va_out : ve_out_default;
			aa_out = aa_out ? aa_out : ae_out_default;

			// 正式开始计算 //
			s_vs2wa(vs_in, va_out + 3);
			s_as2xa(as_in, aa_out + 3);
			s_as2ap(vs_in, as_in, pp_in, aa_out, va_out);
		}
				
		auto s_pq2pe(const double *pq_in, double *pe_out, const char *eu_type) noexcept->void
		{
			// 补充默认参数 //
			pq_in = pq_in ? pq_in : default_pq();
			double pe_out_default[6];
			pe_out = pe_out ? pe_out : pe_out_default;

			// 正式开始计算 //
			
			double pm[16];
			s_pq2pm(pq_in, pm);
			s_pm2pe(pm, pe_out, eu_type);
		}
		auto s_pe2pq(const double *pe_in, double *pq_out, const char *eu_type) noexcept->void
		{
			// 补充默认参数 //
			pe_in = pe_in ? pe_in : default_pe();
			double pq_out_default[6];
			pq_out = pq_out ? pq_out : pq_out_default;

			// 正式开始计算 //
			double pm[16];
			s_pe2pm(pe_in, pm, eu_type);
			s_pm2pq(pm, pq_out);
		}
		auto s_iv2is(const double * iv_in, double *is_out) noexcept->void
		{
			// 补充默认参数 //
			iv_in = iv_in ? iv_in : default_iv();
			double is_out_default[10];
			is_out = is_out ? is_out : is_out_default;

			// 正式开始计算 //
			
			std::fill_n(is_out, 36, 0);

			is_out[0] = iv_in[0];
			is_out[7] = iv_in[0];
			is_out[14] = iv_in[0];

			is_out[4] = iv_in[3];
			is_out[5] = -iv_in[2];
			is_out[9] = -iv_in[3];
			is_out[11] = iv_in[1];
			is_out[15] = iv_in[2];
			is_out[16] = -iv_in[1];

			is_out[19] = -iv_in[3];
			is_out[20] = iv_in[2];
			is_out[24] = iv_in[3];
			is_out[26] = -iv_in[1];
			is_out[30] = -iv_in[2];
			is_out[31] = iv_in[1];

			is_out[21] = iv_in[4];
			is_out[22] = iv_in[7];
			is_out[23] = iv_in[8];
			is_out[27] = iv_in[7];
			is_out[28] = iv_in[5];
			is_out[29] = iv_in[9];
			is_out[33] = iv_in[8];
			is_out[34] = iv_in[9];
			is_out[35] = iv_in[6];
		}
		auto s_is2iv(const double * is_in, double *iv_out) noexcept->void
		{
			// 补充默认参数 //
			is_in = is_in ? is_in : default_is();
			double iv_out_default[10];
			iv_out = iv_out ? iv_out : iv_out_default;

			// 正式开始计算 //
			iv_out[0] = is_in[0];
			iv_out[1] = is_in[11];
			iv_out[2] = is_in[15];
			iv_out[3] = is_in[4];
			iv_out[4] = is_in[21];
			iv_out[5] = is_in[28];
			iv_out[6] = is_in[35];
			iv_out[7] = is_in[22];
			iv_out[8] = is_in[23];
			iv_out[9] = is_in[29];
		}
		auto s_im2is(const double mass_in, const double * in_in, const double *pm_in, double *is_out) noexcept->void
		{
			// 补充默认参数 //
			in_in = in_in ? in_in : default_in();
			pm_in = pm_in ? pm_in : default_pm();
			double is_out_default[36];
			is_out = is_out ? is_out : is_out_default;

			// 正式开始计算 //
			double tem[6][6], tmf[6][6];

			is_out[0] = mass_in;
			is_out[7] = mass_in;
			is_out[14] = mass_in;

			is_out[21] = in_in[0];
			is_out[22] = in_in[1];
			is_out[23] = in_in[2];
			is_out[27] = in_in[3];
			is_out[28] = in_in[4];
			is_out[29] = in_in[5];
			is_out[33] = in_in[6];
			is_out[34] = in_in[7];
			is_out[35] = in_in[8];

			s_tmf(pm_in, *tmf);
			s_mdm(6, 6, 6, *tmf, 6, is_out, 6, *tem, 6);
			s_mdm(6, 6, 6, *tem, 6, *tmf, 6, is_out, 6);
		}

		auto s_pp2pp(const double *relative_pm, const double *from_pp, double *to_pp) noexcept->void
		{
			// 补充默认参数 //
			relative_pm = relative_pm ? relative_pm : default_pm();
			from_pp = from_pp ? from_pp : default_pp();
			double default_to_pp[3];
			to_pp = to_pp ? to_pp : default_to_pp;

			// 正式开始计算 //
			to_pp[0] = relative_pm[0] * from_pp[0] + relative_pm[1] * from_pp[1] + relative_pm[2] * from_pp[2] + relative_pm[3];
			to_pp[1] = relative_pm[4] * from_pp[0] + relative_pm[5] * from_pp[1] + relative_pm[6] * from_pp[2] + relative_pm[7];
			to_pp[2] = relative_pm[8] * from_pp[0] + relative_pm[9] * from_pp[1] + relative_pm[10] * from_pp[2] + relative_pm[11];
		}
		auto s_inv_pp2pp(const double *inv_relative_pm, const double *from_pp, double *to_pp) noexcept->void
		{
			// 补充默认参数 //
			inv_relative_pm = inv_relative_pm ? inv_relative_pm : default_pm();
			from_pp = from_pp ? from_pp : default_pp();
			double default_to_pp[3];
			to_pp = to_pp ? to_pp : default_to_pp;

			// 正式开始计算 //
			double tem[3]{ from_pp[0] - inv_relative_pm[3] ,from_pp[1] - inv_relative_pm[7] ,from_pp[2] - inv_relative_pm[11] };

			to_pp[0] = inv_relative_pm[0] * tem[0] + inv_relative_pm[4] * tem[1] + inv_relative_pm[8] * tem[2];
			to_pp[1] = inv_relative_pm[1] * tem[0] + inv_relative_pm[5] * tem[1] + inv_relative_pm[9] * tem[2];
			to_pp[2] = inv_relative_pm[2] * tem[0] + inv_relative_pm[6] * tem[1] + inv_relative_pm[10] * tem[2];
		}
		auto s_re2re(const double *relative_pm, const double *from_re, double *to_re, const char *from_re_type, const char *to_re_type) noexcept->void
		{
			// 补充默认参数 //
			relative_pm = relative_pm ? relative_pm : default_pm();
			from_re = from_re ? from_re : default_re();
			double default_to_re[3];
			to_re = to_re ? to_re : default_to_re;

			// 正式开始计算 //
			double from_rm[3][3], to_rm[3][3];
			s_re2rm(from_re, *from_rm, from_re_type);
			s_rm2rm(relative_pm, *from_rm, *to_rm);
			s_rm2re(*to_rm, to_re, to_re_type);
		}
		auto s_inv_re2re(const double *inv_relative_pm, const double *from_re, double *to_re, const char *from_re_type, const char *to_re_type) noexcept->void
		{
			// 补充默认参数 //
			inv_relative_pm = inv_relative_pm ? inv_relative_pm : default_pm();
			from_re = from_re ? from_re : default_re();
			double default_to_re[3];
			to_re = to_re ? to_re : default_to_re;

			// 正式开始计算 //
			double from_rm[3][3], to_rm[3][3];
			s_re2rm(from_re, *from_rm, from_re_type);
			s_inv_rm2rm(inv_relative_pm, *from_rm, *to_rm);
			s_rm2re(*to_rm, to_re, to_re_type);
		}
		auto s_rq2rq(const double *relative_pm, const double *from_rq, double *to_rq) noexcept->void
		{
			// 补充默认参数 //
			relative_pm = relative_pm ? relative_pm : default_pm();
			from_rq = from_rq ? from_rq : default_rq();
			double default_to_rq[4];
			to_rq = to_rq ? to_rq : default_to_rq;

			// 正式开始计算 //
			double from_rm[3][3], to_rm[3][3];
			s_rq2rm(from_rq, *from_rm);
			s_rm2rm(relative_pm, *from_rm, *to_rm);
			s_rm2rq(*to_rm, to_rq);
		}
		auto s_inv_rq2rq(const double *inv_relative_pm, const double *from_rq, double *to_rq) noexcept->void
		{
			// 补充默认参数 //
			inv_relative_pm = inv_relative_pm ? inv_relative_pm : default_pm();
			from_rq = from_rq ? from_rq : default_rq();
			double default_to_rq[4];
			to_rq = to_rq ? to_rq : default_to_rq;

			// 正式开始计算 //
			double from_rm[3][3], to_rm[3][3];
			s_rq2rm(from_rq, *from_rm);
			s_inv_rm2rm(inv_relative_pm, *from_rm, *to_rm);
			s_rm2rq(*to_rm, to_rq);
		}
		auto s_rm2rm(const double *relative_pm, const double *from_rm, double *to_rm, std::size_t from_rm_ld, std::size_t to_rm_ld) noexcept->void
		{
			// 补充默认参数 //
			relative_pm = relative_pm ? relative_pm : default_pm();
			from_rm = from_rm ? from_rm : default_rm();
			double default_to_rm[3];
			to_rm = to_rm ? to_rm : default_to_rm;

			// 正式开始计算 //
			s_mdm(3, 3, 3, relative_pm, 4, from_rm, from_rm_ld, to_rm, to_rm_ld);
		}
		auto s_inv_rm2rm(const double *inv_relative_pm, const double *from_rm, double *to_rm, std::size_t from_rm_ld, std::size_t to_rm_ld) noexcept->void
		{
			// 补充默认参数 //
			inv_relative_pm = inv_relative_pm ? inv_relative_pm : default_pm();
			from_rm = from_rm ? from_rm : default_rm();
			double default_to_rm[3];
			to_rm = to_rm ? to_rm : default_to_rm;

			// 正式开始计算 //
			s_mdmTN(3, 3, 3, inv_relative_pm, 4, from_rm, from_rm_ld, to_rm, to_rm_ld);
		}
		auto s_pe2pe(const double *relative_pm, const double *from_pe, double *to_pe, const char *from_pe_type, const char *to_pe_type) noexcept->void
		{
			// 补充默认参数 //
			relative_pm = relative_pm ? relative_pm : default_pm();
			from_pe = from_pe ? from_pe : default_pe();
			double default_to_pe[6];
			to_pe = to_pe ? to_pe : default_to_pe;

			// 正式开始计算 //
			double from_pm[4][4], to_pm[4][4];
			s_pe2pm(from_pe, *from_pm, from_pe_type);
			s_pm2pm(relative_pm, *from_pm, *to_pm);
			s_pm2pe(*to_pm, to_pe, to_pe_type);
		}
		auto s_inv_pe2pe(const double *inv_relative_pm, const double *from_pe, double *to_pe, const char *from_pe_type, const char *to_pe_type) noexcept->void
		{
			// 补充默认参数 //
			inv_relative_pm = inv_relative_pm ? inv_relative_pm : default_pm();
			from_pe = from_pe ? from_pe : default_pe();
			double default_to_pe[6];
			to_pe = to_pe ? to_pe : default_to_pe;

			// 正式开始计算 //
			double from_pm[4][4], to_pm[4][4];
			s_pe2pm(from_pe, *from_pm, from_pe_type);
			s_inv_pm2pm(inv_relative_pm, *from_pm, *to_pm);
			s_pm2pe(*to_pm, to_pe, to_pe_type);
		}
		auto s_pq2pq(const double *relative_pm, const double *from_pq, double *to_pq) noexcept->void
		{
			// 补充默认参数 //
			relative_pm = relative_pm ? relative_pm : default_pm();
			from_pq = from_pq ? from_pq : default_pq();
			double default_to_pq[7];
			to_pq = to_pq ? to_pq : default_to_pq;

			// 正式开始计算 //
			double from_pm[4][4], to_pm[4][4];
			s_pq2pm(from_pq, *from_pm);
			s_pm2pm(relative_pm, *from_pm, *to_pm);
			s_pm2pq(*to_pm, to_pq);
		}
		auto s_inv_pq2pq(const double *inv_relative_pm, const double *from_pq, double *to_pq) noexcept->void
		{
			// 补充默认参数 //
			inv_relative_pm = inv_relative_pm ? inv_relative_pm : default_pm();
			from_pq = from_pq ? from_pq : default_pq();
			double default_to_pq[7];
			to_pq = to_pq ? to_pq : default_to_pq;

			// 正式开始计算 //
			double from_pm[4][4], to_pm[4][4];
			s_pq2pm(from_pq, *from_pm);
			s_inv_pm2pm(inv_relative_pm, *from_pm, *to_pm);
			s_pm2pq(*to_pm, to_pq);
		}
		auto s_pm2pm(const double *relative_pm, const double *from_pm, double *to_pm) noexcept->void
		{
			// 补充默认参数 //
			relative_pm = relative_pm ? relative_pm : default_pm();
			from_pm = from_pm ? from_pm : default_pm();
			double default_to_pm[16];
			to_pm = to_pm ? to_pm : default_to_pm;

			// 正式开始计算 //
			s_mdm(3, 4, 3, relative_pm, 4, from_pm, 4, to_pm, 4);

			to_pm[3] += relative_pm[3];
			to_pm[7] += relative_pm[7];
			to_pm[11] += relative_pm[11];

			to_pm[12] = 0;
			to_pm[13] = 0;
			to_pm[14] = 0;
			to_pm[15] = 1;
		}
		auto s_inv_pm2pm(const double *inv_relative_pm, const double *from_pm, double *to_pm) noexcept->void
		{
			// 补充默认参数 //
			inv_relative_pm = inv_relative_pm ? inv_relative_pm : default_pm();
			from_pm = from_pm ? from_pm : default_pm();
			double default_to_pm[16];
			to_pm = to_pm ? to_pm : default_to_pm;

			// 正式开始计算 //
			s_mdmTN(3, 4, 3, inv_relative_pm, 4, from_pm, 4, to_pm, 4);

			to_pm[3] += -inv_relative_pm[0] * inv_relative_pm[3] - inv_relative_pm[4] * inv_relative_pm[7] - inv_relative_pm[8] * inv_relative_pm[11];
			to_pm[7] += -inv_relative_pm[1] * inv_relative_pm[3] - inv_relative_pm[5] * inv_relative_pm[7] - inv_relative_pm[9] * inv_relative_pm[11];
			to_pm[11] += -inv_relative_pm[2] * inv_relative_pm[3] - inv_relative_pm[6] * inv_relative_pm[7] - inv_relative_pm[10] * inv_relative_pm[11];

			to_pm[12] = 0;
			to_pm[13] = 0;
			to_pm[14] = 0;
			to_pm[15] = 1;
		}

		auto s_vp2vp(const double *relative_pm, const double *relative_vs, const double *from_pp, const double *from_vp,double *to_vp, double *to_pp) noexcept->void
		{
			// 补充默认参数 //
			relative_pm = relative_pm ? relative_pm : default_pm();
			relative_vs = relative_vs ? relative_vs : default_vs();
			from_pp = from_pp ? from_pp : default_pp();
			from_vp = from_vp ? from_vp : default_pp();
			double default_to_vp[3];
			double default_to_pp[3];
			to_vp = to_vp ? to_vp : default_to_vp;
			to_pp = to_pp ? to_pp : default_to_pp;

			// 正式开始计算 //
			s_pp2pp(relative_pm, from_pp, to_pp);
			s_c3(relative_vs + 3, to_pp, to_vp);
			s_mdm(3, 1, 3, 1, relative_pm, 4, from_vp, 1, 1, to_vp, 1);
			s_va(3, relative_vs, to_vp);
		}
		auto s_inv_vp2vp(const double *inv_relative_pm, const double *inv_relative_vs, const double *from_pp, const double *from_vp, double *to_vp, double *to_pp) noexcept->void
		{
			// 补充默认参数 //
			inv_relative_pm = inv_relative_pm ? inv_relative_pm : default_pm();
			inv_relative_vs = inv_relative_vs ? inv_relative_vs : default_vs();
			from_pp = from_pp ? from_pp : default_pp();
			from_vp = from_vp ? from_vp : default_vp();
			double default_to_vp[3];
			double default_to_pp[3];
			to_vp = to_vp ? to_vp : default_to_vp;
			to_pp = to_pp ? to_pp : default_to_pp;

			// 正式计算开始 //
			s_inv_pp2pp(inv_relative_pm, from_pp, to_pp);

			double tem[3];
			std::copy_n(from_vp, 3, tem);
			s_c3(-1, inv_relative_vs + 3, from_pp, 1, tem);
			s_va(3, -1, inv_relative_vs, 1, tem);
			s_mdmTN(3, 1, 3, inv_relative_pm, 4, tem, 1, to_vp, 1);
		}
		auto s_we2we(const double *relative_pm, const double *relative_vs, const double *from_re, const double *from_we, double *to_we, double *to_re, const char *from_re_type, const char *to_re_type) noexcept->void
		{
			// 补充默认参数 //
			relative_pm = relative_pm ? relative_pm : default_pm();
			relative_vs = relative_vs ? relative_vs : default_vs();
			from_re = from_re ? from_re : default_re();
			from_we = from_we ? from_we : default_we();
			double default_to_we[3];
			double default_to_re[3];
			to_we = to_we ? to_we : default_to_we;
			to_re = to_re ? to_re : default_to_re;

			// 正式开始计算 //
			s_re2re(relative_pm, from_re, to_re, from_re_type, to_re_type);

			double from_wa[3], to_wa[3];
			s_we2wa(from_re, from_we, from_wa, from_re_type);
			s_wa2wa(relative_pm, relative_vs, from_wa, to_wa);
			s_wa2we(to_wa, to_re, to_we, to_re_type);
		}
		auto s_inv_we2we(const double *inv_relative_pm, const double *inv_relative_vs, const double *from_re, const double *from_we, double *to_we, double *to_re, const char *from_re_type, const char *to_re_type) noexcept->void
		{
			// 补充默认参数 //
			inv_relative_pm = inv_relative_pm ? inv_relative_pm : default_pm();
			inv_relative_vs = inv_relative_vs ? inv_relative_vs : default_vs();
			from_re = from_re ? from_re : default_re();
			from_we = from_we ? from_we : default_we();
			double default_to_we[3];
			double default_to_re[3];
			to_we = to_we ? to_we : default_to_we;
			to_re = to_re ? to_re : default_to_re;

			// 正式开始计算 //
			s_inv_re2re(inv_relative_pm, from_re, to_re, from_re_type, to_re_type);

			double from_wa[3], to_wa[3];
			s_we2wa(from_re, from_we, from_wa, from_re_type);
			s_inv_wa2wa(inv_relative_pm, inv_relative_vs, from_wa, to_wa);
			s_wa2we(to_wa, to_re, to_we, to_re_type);
		}
		auto s_wq2wq(const double *relative_pm, const double *relative_vs, const double *from_rq, const double *from_wq, double *to_wq, double *to_rq) noexcept->void
		{
			// 补充默认参数 //
			relative_pm = relative_pm ? relative_pm : default_pm();
			relative_vs = relative_vs ? relative_vs : default_vs();
			from_rq = from_rq ? from_rq : default_rq();
			from_wq = from_wq ? from_wq : default_wq();
			double default_to_wq[4];
			double default_to_rq[4];
			to_wq = to_wq ? to_wq : default_to_wq;
			to_rq = to_rq ? to_rq : default_to_rq;

			// 正式开始计算 //
			s_rq2rq(relative_pm, from_rq, to_rq);

			double from_wa[3], to_wa[3];
			s_wq2wa(from_rq, from_wq, from_wa);
			s_wa2wa(relative_pm, relative_vs, from_wa, to_wa);
			s_wa2wq(to_wa, to_rq, to_wq);
		}
		auto s_inv_wq2wq(const double *inv_relative_pm, const double *inv_relative_vs, const double *from_rq, const double *from_wq, double *to_wq, double *to_rq) noexcept->void
		{
			// 补充默认参数 //
			inv_relative_pm = inv_relative_pm ? inv_relative_pm : default_pm();
			inv_relative_vs = inv_relative_vs ? inv_relative_vs : default_vs();
			from_rq = from_rq ? from_rq : default_rq();
			from_wq = from_wq ? from_wq : default_wq();
			double default_to_wq[4];
			double default_to_rq[4];
			to_wq = to_wq ? to_wq : default_to_wq;
			to_rq = to_rq ? to_rq : default_to_rq;

			// 正式开始计算 //
			s_inv_rq2rq(inv_relative_pm, from_rq, to_rq);

			double from_wa[3], to_wa[3];
			s_wq2wa(from_rq, from_wq, from_wa);
			s_inv_wa2wa(inv_relative_pm, inv_relative_vs, from_wa, to_wa);
			s_wa2wq(to_wa, to_rq, to_wq);
		}
		auto s_wm2wm(const double *relative_pm, const double *relative_vs, const double *from_rm, const double *from_wm, double *to_wm, double *to_rm) noexcept->void
		{
			// 补充默认参数 //
			relative_pm = relative_pm ? relative_pm : default_pm();
			relative_vs = relative_vs ? relative_vs : default_vs();
			from_rm = from_rm ? from_rm : default_rm();
			from_wm = from_wm ? from_wm : default_wm();
			double default_to_rm[9];
			double default_to_wm[9];
			to_wm = to_wm ? to_wm : default_to_wm;
			to_rm = to_rm ? to_rm : default_to_rm;

			// 正式开始计算 //
			s_rm2rm(relative_pm, from_rm, to_rm);

			double from_wa[3], to_wa[3];
			s_wm2wa(from_rm, from_wm, from_wa);
			s_wa2wa(relative_pm, relative_vs, from_wa, to_wa);
			s_wa2wm(to_wa, to_rm, to_wm);
		}
		auto s_inv_wm2wm(const double *inv_relative_pm, const double *inv_relative_vs, const double *from_rm, const double *from_wm, double *to_wm, double *to_rm) noexcept->void
		{
			// 补充默认参数 //
			inv_relative_pm = inv_relative_pm ? inv_relative_pm : default_pm();
			inv_relative_vs = inv_relative_vs ? inv_relative_vs : default_vs();
			from_rm = from_rm ? from_rm : default_rm();
			from_wm = from_wm ? from_wm : default_wm();
			double default_to_rm[9];
			double default_to_wm[9];
			to_wm = to_wm ? to_wm : default_to_wm;
			to_rm = to_rm ? to_rm : default_to_rm;

			// 正式开始计算 //
			s_inv_rm2rm(inv_relative_pm, from_rm, to_rm);

			double from_wa[3], to_wa[3];
			s_wm2wa(from_rm, from_wm, from_wa);
			s_inv_wa2wa(inv_relative_pm, inv_relative_vs, from_wa, to_wa);
			s_wa2wm(to_wa, to_rm, to_wm);
		}
		auto s_wa2wa(const double *relative_pm, const double *relative_vs, const double *from_wa, double *to_wa) noexcept->void
		{
			// 补充默认参数 //
			relative_pm = relative_pm ? relative_pm : default_pm();
			relative_vs = relative_vs ? relative_vs : default_vs();
			from_wa = from_wa ? from_wa : default_wa();
			double default_to_we[3];
			to_wa = to_wa ? to_wa : default_to_we;

			// 正式开始计算 //
			s_mdm(3, 1, 3, relative_pm, 4, from_wa, 1, to_wa, 1);
			s_va(3, relative_vs + 3, to_wa);
		}
		auto s_inv_wa2wa(const double *inv_relative_pm, const double *inv_relative_vs, const double *from_wa, double *to_wa) noexcept->void
		{
			// 补充默认参数 //
			inv_relative_pm = inv_relative_pm ? inv_relative_pm : default_pm();
			inv_relative_vs = inv_relative_vs ? inv_relative_vs : default_pm();
			from_wa = from_wa ? from_wa : default_wa();
			double default_to_we[3];
			to_wa = to_wa ? to_wa : default_to_we;

			// 正式计算开始 //
			double tem[3]{ -inv_relative_vs[3],-inv_relative_vs[4],-inv_relative_vs[5] };
			s_va(3, from_wa, tem);
			s_mdmTN(3, 1, 3, inv_relative_pm, 4, tem, 1, to_wa, 1);
		}
		auto s_va2va(const double *relative_pm, const double *relative_vs, const double *from_pp, const double *from_va, double *to_va, double *to_pp) noexcept->void
		{
			// 补充默认参数 //
			relative_pm = relative_pm ? relative_pm : default_pm();
			relative_vs = relative_vs ? relative_vs : default_vs();
			from_pp = from_pp ? from_pp : default_pp();
			from_va = from_va ? from_va : default_va();
			double default_to_pp[3];
			double default_to_vw[6];
			to_pp = to_pp ? to_pp : default_to_pp;
			to_va = to_va ? to_va : default_to_vw;

			// 正式开始计算 //
			s_pp2pp(relative_pm, from_pp, to_pp);

			double from_vs[6], to_vs[6];
			s_va2vs(from_pp, from_va, from_vs);
			s_vs2vs(relative_pm, relative_vs, from_vs, to_vs);
			s_vs2va(to_vs, to_pp, to_va);
		}
		auto s_inv_va2va(const double *inv_relative_pm, const double *inv_relative_vs, const double *from_pp, const double *from_va, double *to_va, double *to_pp) noexcept->void
		{
			// 补充默认参数 //
			inv_relative_pm = inv_relative_pm ? inv_relative_pm : default_pm();
			inv_relative_vs = inv_relative_vs ? inv_relative_vs : default_vs();
			from_pp = from_pp ? from_pp : default_pp();
			from_va = from_va ? from_va : default_va();
			double default_to_pp[3];
			double default_to_vw[6];
			to_pp = to_pp ? to_pp : default_to_pp;
			to_va = to_va ? to_va : default_to_vw;

			// 正式开始计算 //
			s_inv_pp2pp(inv_relative_pm, from_pp, to_pp);

			double from_vs[6], to_vs[6];
			s_va2vs(from_pp, from_va, from_vs);
			s_inv_vs2vs(inv_relative_pm, inv_relative_vs, from_vs, to_vs);
			s_vs2va(to_vs, to_pp, to_va);
		}
		auto s_ve2ve(const double *relative_pm, const double *relative_vs, const double *from_pe, const double *from_ve, double *to_ve, double *to_pe, const char *from_re_type, const char *to_re_type) noexcept->void
		{
			// 补充默认参数 //
			relative_pm = relative_pm ? relative_pm : default_pm();
			relative_vs = relative_vs ? relative_vs : default_vs();
			from_pe = from_pe ? from_pe : default_pe();
			from_ve = from_ve ? from_ve : default_ve();
			double default_to_pe[6];
			double default_to_ve[6];
			to_ve = to_ve ? to_ve : default_to_ve;
			to_pe = to_pe ? to_pe : default_to_pe;

			// 正式开始计算 //
			s_pe2pe(relative_pm, from_pe, to_pe, from_re_type, to_re_type);

			double from_vs[6], to_vs[6];
			s_ve2vs(from_pe, from_ve, from_vs, from_re_type);
			s_vs2vs(relative_pm, relative_vs, from_vs, to_vs);
			s_vs2ve(to_vs, to_pe, to_ve, to_re_type);
		}
		auto s_inv_ve2ve(const double *inv_relative_pm, const double *inv_relative_vs, const double *from_pe, const double *from_ve, double *to_ve, double *to_pe, const char *from_re_type, const char *to_re_type) noexcept->void
		{
			// 补充默认参数 //
			inv_relative_pm = inv_relative_pm ? inv_relative_pm : default_pm();
			inv_relative_vs = inv_relative_vs ? inv_relative_vs : default_vs();
			from_pe = from_pe ? from_pe : default_pe();
			from_ve = from_ve ? from_ve : default_ve();
			double default_to_pe[6];
			double default_to_ve[6];
			to_ve = to_ve ? to_ve : default_to_ve;
			to_pe = to_pe ? to_pe : default_to_pe;

			// 正式开始计算 //
			s_inv_pe2pe(inv_relative_pm, from_pe, to_pe, from_re_type, to_re_type);

			double from_vs[6], to_vs[6];
			s_ve2vs(from_pe, from_ve, from_vs, from_re_type);
			s_inv_vs2vs(inv_relative_pm, inv_relative_vs, from_vs, to_vs);
			s_vs2ve(to_vs, to_pe, to_ve, to_re_type);
		}
		auto s_vq2vq(const double *relative_pm, const double *relative_vs, const double *from_pq, const double *from_vq, double *to_vq, double *to_pq) noexcept->void
		{
			// 补充默认参数 //
			relative_pm = relative_pm ? relative_pm : default_pm();
			relative_vs = relative_vs ? relative_vs : default_vs();
			from_pq = from_pq ? from_pq : default_pq();
			from_vq = from_vq ? from_vq : default_vq();
			double default_to_pq[7];
			double default_to_vq[7];
			to_vq = to_vq ? to_vq : default_to_vq;
			to_pq = to_pq ? to_pq : default_to_pq;

			// 正式开始计算 //
			s_pq2pq(relative_pm, from_pq, to_pq);

			double from_vs[6], to_vs[6];
			s_vq2vs(from_pq, from_vq, from_vs);
			s_vs2vs(relative_pm, relative_vs, from_vs, to_vs);
			s_vs2vq(to_vs, to_pq, to_vq);
		}
		auto s_inv_vq2vq(const double *inv_relative_pm, const double *inv_relative_vs, const double *from_pq, const double *from_vq, double *to_vq, double *to_pq) noexcept->void
		{
			// 补充默认参数 //
			inv_relative_pm = inv_relative_pm ? inv_relative_pm : default_pm();
			inv_relative_vs = inv_relative_vs ? inv_relative_vs : default_vs();
			from_pq = from_pq ? from_pq : default_pq();
			from_vq = from_vq ? from_vq : default_vq();
			double default_to_pq[7];
			double default_to_vq[7];
			to_vq = to_vq ? to_vq : default_to_vq;
			to_pq = to_pq ? to_pq : default_to_pq;

			// 正式开始计算 //
			s_inv_pq2pq(inv_relative_pm, from_pq, to_pq);

			double from_vs[6], to_vs[6];
			s_vq2vs(from_pq, from_vq, from_vs);
			s_inv_vs2vs(inv_relative_pm, inv_relative_vs, from_vs, to_vs);
			s_vs2vq(to_vs, to_pq, to_vq);
		}
		auto s_vm2vm(const double *relative_pm, const double *relative_vs, const double *from_pm, const double *from_vm, double *to_vm, double *to_pm) noexcept->void
		{
			// 补充默认参数 //
			relative_pm = relative_pm ? relative_pm : default_pm();
			relative_vs = relative_vs ? relative_vs : default_vs();
			from_pm = from_pm ? from_pm : default_pm();
			from_vm = from_vm ? from_vm : default_vm();
			double default_to_pm[16];
			double default_to_vm[16];
			to_vm = to_vm ? to_vm : default_to_vm;
			to_pm = to_pm ? to_pm : default_to_pm;

			// 正式开始计算 //
			s_pm2pm(relative_pm, from_pm, to_pm);

			double from_vs[6], to_vs[6];
			s_vm2vs(from_pm, from_vm, from_vs);
			s_vs2vs(relative_pm, relative_vs, from_vs, to_vs);
			s_vs2vm(to_vs, to_pm, to_vm);
		}
		auto s_inv_vm2vm(const double *inv_relative_pm, const double *inv_relative_vs, const double *from_pm, const double *from_vm, double *to_vm, double *to_pm) noexcept->void
		{
			// 补充默认参数 //
			inv_relative_pm = inv_relative_pm ? inv_relative_pm : default_pm();
			inv_relative_vs = inv_relative_vs ? inv_relative_vs : default_vs();
			from_pm = from_pm ? from_pm : default_pm();
			from_vm = from_vm ? from_vm : default_vm();
			double default_to_pm[16];
			double default_to_vm[16];
			to_vm = to_vm ? to_vm : default_to_vm;
			to_pm = to_pm ? to_pm : default_to_pm;

			// 正式开始计算 //
			s_inv_pm2pm(inv_relative_pm, from_pm, to_pm);

			double from_vs[6], to_vs[6];
			s_vm2vs(from_pm, from_vm, from_vs);
			s_inv_vs2vs(inv_relative_pm, inv_relative_vs, from_vs, to_vs);
			s_vs2vm(to_vs, to_pm, to_vm);
		}
		auto s_vs2vs(const double *relative_pm, const double *relative_vs, const double *from_vs, double *to_vs) noexcept->void
		{
			// 补充默认参数 //
			relative_pm = relative_pm ? relative_pm : default_pm();
			relative_vs = relative_vs ? relative_vs : default_vs();
			from_vs = from_vs ? from_vs : default_vs();
			double default_to_vs[6]{ 0,0,0,0,0,0 };
			to_vs = to_vs ? to_vs : default_to_vs;

			// 正式开始计算 //
			s_tv(relative_pm, from_vs, to_vs);
			s_va(6, relative_vs, to_vs);
		}
		auto s_inv_vs2vs(const double *inv_relative_pm, const double *inv_relative_vs, const double *from_vs, double *to_vs) noexcept->void
		{
			// 补充默认参数 //
			inv_relative_pm = inv_relative_pm ? inv_relative_pm : default_pm();
			inv_relative_vs = inv_relative_vs ? inv_relative_vs : default_vs();
			from_vs = from_vs ? from_vs : default_vs();
			double default_to_vs[6];
			to_vs = to_vs ? to_vs : default_to_vs;
			
			
			// 正式开始计算 //
			double tem[6];
			std::copy_n(from_vs, 6, tem);
			s_va(6, -1, inv_relative_vs, 1, tem);
			s_inv_tv(inv_relative_pm, tem, to_vs);
		}

		auto s_ap2ap(const double *relative_pm, const double *relative_vs, const double *relative_as, 
			const double *from_pp, const double *from_vp, const double *from_ap, double *to_ap, double *to_vp, double *to_pp) noexcept->void
		{
			// 补充默认参数 //
			relative_pm = relative_pm ? relative_pm : default_pm();
			relative_vs = relative_vs ? relative_vs : default_vs();
			relative_as = relative_as ? relative_as : default_as();
			from_pp = from_pp ? from_pp : default_pp();
			from_vp = from_vp ? from_vp : default_vp();
			from_ap = from_ap ? from_ap : default_ap();
			double default_to_ap[3];
			double default_to_vp[3];
			double default_to_pp[3];
			to_ap = to_ap ? to_ap : default_to_ap;
			to_vp = to_vp ? to_vp : default_to_vp;
			to_pp = to_pp ? to_pp : default_to_pp;

			// 正式开始计算 //
			double tem_vp[3];
			s_vp2vp(relative_pm, relative_vs, from_pp, from_vp, to_vp, to_pp);

			s_c3(relative_as + 3, to_pp, to_ap);
			std::copy_n(to_vp, 3, tem_vp);
			s_mdm(3, 1, 3, 1, relative_pm, 4, from_vp, 1, 1, tem_vp, 1);
			s_c3(1, relative_vs + 3, tem_vp, 1, to_ap);
			s_mdm(3, 1, 3, 1, relative_pm, 4, from_ap, 1, 1, to_ap, 1);
			s_va(3, relative_as, to_ap);
		}
		auto s_inv_ap2ap(const double *inv_relative_pm, const double *inv_relative_vs, const double *inv_relative_as,
			const double *from_pp, const double *from_vp, const double *from_ap, double *to_ap, double *to_vp, double *to_pp) noexcept->void
		{
			// 补充默认参数 //
			inv_relative_pm = inv_relative_pm ? inv_relative_pm : default_pm();
			inv_relative_vs = inv_relative_vs ? inv_relative_vs : default_vs();
			inv_relative_as = inv_relative_as ? inv_relative_as : default_as();
			from_pp = from_pp ? from_pp : default_pp();
			from_vp = from_vp ? from_vp : default_vp();
			from_ap = from_ap ? from_ap : default_ap();
			double default_to_ap[3];
			double default_to_vp[3];
			double default_to_pp[3];
			to_ap = to_ap ? to_ap : default_to_ap;
			to_vp = to_vp ? to_vp : default_to_vp;
			to_pp = to_pp ? to_pp : default_to_pp;

			// 正式开始计算 //
			s_inv_vp2vp(inv_relative_pm, inv_relative_vs, from_pp, from_vp, to_vp, to_pp);

			std::fill_n(to_ap, 3, 0);

			double tem[3], tem2[3];

			std::copy_n(from_ap, 3, tem);
			s_c3(-1, inv_relative_as + 3, from_pp, 1, tem);

			std::copy_n(from_vp, 3, tem2);
			s_mdm(3, 1, 3, 1, inv_relative_pm, 4, to_vp, 1, 1, tem2, 1);
			s_c3(-1, inv_relative_vs + 3, tem2, 1, tem);

			s_va(3, -1, inv_relative_as, 1, tem);

			s_mdmTN(3, 1, 3, 1, inv_relative_pm, 4, tem, 1, 0, to_ap, 1);
		}
		auto s_xe2xe(const double *relative_pm, const double *relative_vs, const double *relative_as,
			const double *from_re, const double *from_we, const double *from_xe, double *to_xe, double *to_we, double *to_re, const char *from_re_type, const char *to_re_type) noexcept->void
		{
			// 补充默认参数 //
			relative_pm = relative_pm ? relative_pm : default_pm();
			relative_vs = relative_vs ? relative_vs : default_vs();
			relative_as = relative_as ? relative_as : default_as();
			from_re = from_re ? from_re : default_re();
			from_we = from_we ? from_we : default_we();
			from_xe = from_xe ? from_xe : default_xe();
			double default_to_re[3];
			double default_to_we[3];
			double default_to_xe[3];
			to_re = to_re ? to_re : default_to_re;
			to_we = to_we ? to_we : default_to_we;
			to_xe = to_xe ? to_xe : default_to_xe;

			// 正式开始计算 //
			s_re2re(relative_pm, from_re, to_re, from_re_type, to_re_type);

			double from_xa[3], to_xa[3], from_wa[3], to_wa[3];
			s_xe2xa(from_re, from_we, from_xe, from_xa, from_wa, from_re_type);
			s_xa2xa(relative_pm, relative_vs, relative_as, from_wa, from_xa, to_xa, to_wa);
			s_xa2xe(to_wa, to_xa, to_re, to_xe, to_we, to_re_type);
		}
		auto s_inv_xe2xe(const double *inv_relative_pm, const double *inv_relative_vs, const double *inv_relative_as,
			const double *from_re, const double *from_we, const double *from_xe, double *to_xe, double *to_we, double *to_re, const char *from_re_type, const char *to_re_type) noexcept->void
		{
			// 补充默认参数 //
			inv_relative_pm = inv_relative_pm ? inv_relative_pm : default_pm();
			inv_relative_vs = inv_relative_vs ? inv_relative_vs : default_vs();
			inv_relative_as = inv_relative_as ? inv_relative_as : default_as();
			from_re = from_re ? from_re : default_re();
			from_we = from_we ? from_we : default_we();
			from_xe = from_xe ? from_xe : default_xe();
			double default_to_re[3];
			double default_to_we[3];
			double default_to_xe[3];
			to_re = to_re ? to_re : default_to_re;
			to_we = to_we ? to_we : default_to_we;
			to_xe = to_xe ? to_xe : default_to_xe;

			// 正式开始计算 //
			s_inv_re2re(inv_relative_pm, from_re, to_re, from_re_type, to_re_type);

			double from_xa[3], to_xa[3], from_wa[3], to_wa[3];
			s_xe2xa(from_re, from_we, from_xe, from_xa, from_wa, from_re_type);
			s_inv_xa2xa(inv_relative_pm, inv_relative_vs, inv_relative_as, from_wa, from_xa, to_xa, to_wa);
			s_xa2xe(to_wa, to_xa, to_re, to_xe, to_we, to_re_type);
		}
		auto s_xq2xq(const double *relative_pm, const double *relative_vs, const double *relative_as,
			const double *from_rq, const double *from_wq, const double *from_xq, double *to_xq, double *to_wq, double *to_rq) noexcept->void
		{
			// 补充默认参数 //
			relative_pm = relative_pm ? relative_pm : default_pm();
			relative_vs = relative_vs ? relative_vs : default_vs();
			relative_as = relative_as ? relative_as : default_as();
			from_rq = from_rq ? from_rq : default_rq();
			from_wq = from_wq ? from_wq : default_wq();
			from_xq = from_xq ? from_xq : default_xq();
			double default_to_xq[4];
			double default_to_wq[4];
			double default_to_rq[4];
			to_xq = to_xq ? to_xq : default_to_xq;
			to_wq = to_wq ? to_wq : default_to_wq;
			to_rq = to_rq ? to_rq : default_to_rq;

			// 正式开始计算 //
			s_rq2rq(relative_pm, from_rq, to_rq);

			double from_xa[3], to_xa[3], from_wa[3], to_wa[3];
			s_xq2xa(from_rq, from_wq, from_xq, from_xa, from_wa);
			//s_wa2wa(relative_pm, relative_vs, from_wa, to_wa);
			s_xa2xa(relative_pm, relative_vs, relative_as, from_wa, from_xa, to_xa, to_wa);
			s_xa2xq(to_wa, to_xa, to_rq, to_xq, to_wq);
		}
		auto s_inv_xq2xq(const double *inv_relative_pm, const double *inv_relative_vs, const double *inv_relative_as,
			const double *from_rq, const double *from_wq, const double *from_xq, double *to_xq, double *to_wq, double *to_rq) noexcept->void
		{
			// 补充默认参数 //
			inv_relative_pm = inv_relative_pm ? inv_relative_pm : default_pm();
			inv_relative_vs = inv_relative_vs ? inv_relative_vs : default_vs();
			inv_relative_as = inv_relative_as ? inv_relative_as : default_as();
			from_rq = from_rq ? from_rq : default_rq();
			from_wq = from_wq ? from_wq : default_wq();
			from_xq = from_xq ? from_xq : default_xq();
			double default_to_xq[4];
			double default_to_wq[4];
			double default_to_rq[4];
			to_xq = to_xq ? to_xq : default_to_xq;
			to_wq = to_wq ? to_wq : default_to_wq;
			to_rq = to_rq ? to_rq : default_to_rq;

			// 正式开始计算 //
			s_inv_rq2rq(inv_relative_pm, from_rq, to_rq);

			double from_xa[3], to_xa[3], from_wa[3], to_wa[3];
			s_xq2xa(from_rq, from_wq, from_xq, from_xa, from_wa);
			s_inv_xa2xa(inv_relative_pm, inv_relative_vs, inv_relative_as, from_wa, from_xa, to_xa, to_wa);
			s_xa2xq(to_wa, to_xa, to_rq, to_xq, to_wq);
		}
		auto s_xm2xm(const double *relative_pm, const double *relative_vs, const double *relative_as,
			const double *from_rm, const double *from_wm, const double *from_xm, double *to_xm, double *to_wm, double *to_rm) noexcept->void
		{
			// 补充默认参数 //
			relative_pm = relative_pm ? relative_pm : default_pm();
			relative_vs = relative_vs ? relative_vs : default_vs();
			relative_as = relative_as ? relative_as : default_as();
			from_rm = from_rm ? from_rm : default_rm();
			from_wm = from_wm ? from_wm : default_wm();
			from_xm = from_xm ? from_xm : default_xm();
			double default_to_rm[9];
			double default_to_wm[9];
			double default_to_xm[9];
			to_rm = to_rm ? to_rm : default_to_rm;
			to_wm = to_wm ? to_wm : default_to_wm;
			to_xm = to_xm ? to_xm : default_to_xm;

			// 正式开始计算 //
			s_rm2rm(relative_pm, from_rm, to_rm);

			double from_xa[3], to_xa[3], from_wa[3], to_wa[3];
			s_xm2xa(from_rm, from_wm, from_xm, from_xa, from_wa);
			s_xa2xa(relative_pm, relative_vs, relative_as, from_wa, from_xa, to_xa, to_wa);
			s_xa2xm(to_wa, to_xa, to_rm, to_xm, to_wm);
		}
		auto s_inv_xm2xm(const double *inv_relative_pm, const double *inv_relative_vs, const double *inv_relative_as,
			const double *from_rm, const double *from_wm, const double *from_xm, double *to_xm, double *to_wm, double *to_rm) noexcept->void
		{
			// 补充默认参数 //
			inv_relative_pm = inv_relative_pm ? inv_relative_pm : default_pm();
			inv_relative_vs = inv_relative_vs ? inv_relative_vs : default_vs();
			inv_relative_as = inv_relative_as ? inv_relative_as : default_as();
			from_rm = from_rm ? from_rm : default_rm();
			from_wm = from_wm ? from_wm : default_wm();
			from_xm = from_xm ? from_xm : default_xm();
			double default_to_rm[9];
			double default_to_wm[9];
			double default_to_xm[9];
			to_rm = to_rm ? to_rm : default_to_rm;
			to_wm = to_wm ? to_wm : default_to_wm;
			to_xm = to_xm ? to_xm : default_to_xm;

			// 正式开始计算 //
			s_inv_rm2rm(inv_relative_pm, from_rm, to_rm);

			double from_xa[3], to_xa[3], from_wa[3], to_wa[3];
			s_xm2xa(from_rm, from_wm, from_xm, from_xa, from_wa);
			s_inv_xa2xa(inv_relative_pm, inv_relative_vs, inv_relative_as, from_wa, from_xa, to_xa, to_wa);
			s_xa2xm(to_wa, to_xa, to_rm, to_xm, to_wm);
		}
		auto s_xa2xa(const double *relative_pm, const double *relative_vs, const double *relative_as,
			const double *from_wa, const double *from_xa, double *to_xa, double *to_wa) noexcept->void
		{
			// 补充默认参数 //
			relative_pm = relative_pm ? relative_pm : default_pm();
			relative_vs = relative_vs ? relative_vs : default_vs();
			relative_as = relative_as ? relative_as : default_as();
			from_wa = from_wa ? from_wa : default_wa();
			from_xa = from_xa ? from_xa : default_xa();
			double default_to_xa[3], default_to_wa[3];
			to_xa = to_xa ? to_xa : default_to_xa;
			to_wa = to_wa ? to_wa : default_to_wa;

			// 正式开始计算 //
			s_wa2wa(relative_pm, relative_vs, from_wa, to_wa);
			
			s_mdm(3, 1, 3, relative_pm, 4, from_xa, 1, to_xa, 1);
			s_c3(1, relative_vs + 3, to_wa, 1, to_xa);
			s_va(3, relative_as + 3, to_xa);
		}
		auto s_inv_xa2xa(const double *inv_relative_pm, const double *inv_relative_vs, const double *inv_relative_as,
			const double *from_wa, const double *from_xa, double *to_xa, double *to_wa) noexcept->void
		{
			// 补充默认参数 //
			inv_relative_pm = inv_relative_pm ? inv_relative_pm : default_pm();
			inv_relative_vs = inv_relative_vs ? inv_relative_vs : default_vs();
			inv_relative_as = inv_relative_as ? inv_relative_as : default_as();
			from_wa = from_wa ? from_wa : default_wa();
			from_xa = from_xa ? from_xa : default_xa();
			double default_to_xa[3], default_to_wa[3];
			to_xa = to_xa ? to_xa : default_to_xa;
			to_wa = to_wa ? to_wa : default_to_wa;

			// 正式开始计算 //
			s_inv_wa2wa(inv_relative_pm, inv_relative_vs, from_wa, to_wa);

			double tem[3]{ -inv_relative_as[3],-inv_relative_as[4],-inv_relative_as[5] };
			s_va(3, from_xa, tem);
			s_c3(-1, inv_relative_vs + 3, from_wa, 1, tem);
			s_mdmTN(3, 1, 3, inv_relative_pm, 4, tem, 1, to_xa, 1);
		}
		auto s_ae2ae(const double *relative_pm, const double *relative_vs, const double *relative_as,
			const double *from_pe, const double *from_ve, const double *from_ae, double *to_ae, double *to_ve, double *to_pe, const char *from_re_type, const char *to_re_type) noexcept->void
		{
			// 补充默认参数 //
			relative_pm = relative_pm ? relative_pm : default_pm();
			relative_vs = relative_vs ? relative_vs : default_vs();
			relative_as = relative_as ? relative_as : default_as();
			from_pe = from_pe ? from_pe : default_pe();
			from_ve = from_ve ? from_ve : default_ve();
			from_ae = from_ae ? from_ae : default_ae();
			double default_to_ae[6];
			double default_to_ve[6];
			double default_to_pe[6];
			to_ae = to_ae ? to_ae : default_to_ae;
			to_ve = to_ve ? to_ve : default_to_ve;
			to_pe = to_pe ? to_pe : default_to_pe;

			// 正式开始计算 //
			s_pe2pe(relative_pm, from_pe, to_pe, from_re_type, to_re_type);

			double from_vs[6], from_as[6], to_vs[6], to_as[6];
			s_ae2as(from_pe, from_ve, from_ae, from_as, from_vs, from_re_type);
			s_as2as(relative_pm, relative_vs, relative_as, from_vs, from_as, to_as, to_vs);
			s_as2ae(to_vs, to_as, to_pe, to_ae, to_ve, to_re_type);
		}
		auto s_inv_ae2ae(const double *inv_relative_pm, const double *inv_relative_vs, const double *inv_relative_as,
			const double *from_pe, const double *from_ve, const double *from_ae, double *to_ae, double *to_ve, double *to_pe, const char *from_re_type, const char *to_re_type) noexcept->void
		{
			// 补充默认参数 //
			inv_relative_pm = inv_relative_pm ? inv_relative_pm : default_pm();
			inv_relative_vs = inv_relative_vs ? inv_relative_vs : default_vs();
			inv_relative_as = inv_relative_as ? inv_relative_as : default_as();
			from_pe = from_pe ? from_pe : default_pe();
			from_ve = from_ve ? from_ve : default_ve();
			from_ae = from_ae ? from_ae : default_ae();
			double default_to_ae[6];
			double default_to_ve[6];
			double default_to_pe[6];
			to_ae = to_ae ? to_ae : default_to_ae;
			to_ve = to_ve ? to_ve : default_to_ve;
			to_pe = to_pe ? to_pe : default_to_pe;

			// 正式开始计算 //
			s_inv_pe2pe(inv_relative_pm, from_pe, to_pe, from_re_type, to_re_type);

			double from_vs[6], from_as[6], to_vs[6], to_as[6];
			s_ae2as(from_pe, from_ve, from_ae, from_as, from_vs, from_re_type);
			s_inv_as2as(inv_relative_pm, inv_relative_vs, inv_relative_as, from_vs, from_as, to_as, to_vs);
			s_as2ae(to_vs, to_as, to_pe, to_ae, to_ve, to_re_type);
		}
		auto s_aq2aq(const double *relative_pm, const double *relative_vs, const double *relative_as,
			const double *from_pq, const double *from_vq, const double *from_aq, double *to_aq, double *to_vq, double *to_pq) noexcept->void
		{
			// 补充默认参数 //
			relative_pm = relative_pm ? relative_pm : default_pm();
			relative_vs = relative_vs ? relative_vs : default_vs();
			relative_as = relative_as ? relative_as : default_as();
			from_pq = from_pq ? from_pq : default_pq();
			from_vq = from_vq ? from_vq : default_vq();
			from_aq = from_aq ? from_aq : default_aq();
			double default_to_aq[7];
			double default_to_vq[7];
			double default_to_pq[7];
			to_aq = to_aq ? to_aq : default_to_aq;
			to_vq = to_vq ? to_vq : default_to_vq;
			to_pq = to_pq ? to_pq : default_to_pq;

			// 正式开始计算 //
			s_pq2pq(relative_pm, from_pq, to_pq);

			double from_vs[6], from_as[6], to_vs[6], to_as[6];
			s_aq2as(from_pq, from_vq, from_aq, from_as, from_vs);
			s_as2as(relative_pm, relative_vs, relative_as, from_vs, from_as, to_as, to_vs);
			s_as2aq(to_vs, to_as, to_pq, to_aq, to_vq);
		}
		auto s_inv_aq2aq(const double *inv_relative_pm, const double *inv_relative_vs, const double *inv_relative_as,
			const double *from_pq, const double *from_vq, const double *from_aq, double *to_aq, double *to_vq, double *to_pq) noexcept->void
		{
			// 补充默认参数 //
			inv_relative_pm = inv_relative_pm ? inv_relative_pm : default_pm();
			inv_relative_vs = inv_relative_vs ? inv_relative_vs : default_vs();
			inv_relative_as = inv_relative_as ? inv_relative_as : default_as();
			from_pq = from_pq ? from_pq : default_pq();
			from_vq = from_vq ? from_vq : default_vq();
			from_aq = from_aq ? from_aq : default_aq();
			double default_to_aq[7];
			double default_to_vq[7];
			double default_to_pq[7];
			to_aq = to_aq ? to_aq : default_to_aq;
			to_vq = to_vq ? to_vq : default_to_vq;
			to_pq = to_pq ? to_pq : default_to_pq;

			// 正式开始计算 //
			s_inv_pq2pq(inv_relative_pm, from_pq, to_pq);

			double from_vs[6], from_as[6], to_vs[6], to_as[6];
			s_aq2as(from_pq, from_vq, from_aq, from_as, from_vs);
			s_inv_as2as(inv_relative_pm, inv_relative_vs, inv_relative_as, from_vs, from_as, to_as, to_vs);
			s_as2aq(to_vs, to_as, to_pq, to_aq, to_vq);
		}
		auto s_am2am(const double *relative_pm, const double *relative_vs, const double *relative_as,
			const double *from_pm, const double *from_vm, const double *from_am, double *to_am, double *to_vm, double *to_pm) noexcept->void
		{
			// 补充默认参数 //
			relative_pm = relative_pm ? relative_pm : default_pm();
			relative_vs = relative_vs ? relative_vs : default_vs();
			relative_as = relative_as ? relative_as : default_as();
			from_pm = from_pm ? from_pm : default_pm();
			from_vm = from_vm ? from_vm : default_vm();
			from_am = from_am ? from_am : default_am();
			double default_to_pm[16];
			double default_to_vm[16];
			double default_to_am[16];
			to_am = to_am ? to_am : default_to_am;
			to_vm = to_vm ? to_vm : default_to_vm;
			to_pm = to_pm ? to_pm : default_to_pm;

			// 正式开始计算 //
			s_pm2pm(relative_pm, from_pm, to_pm);

			double from_vs[6], from_as[6], to_vs[6], to_as[6];
			s_am2as(from_pm, from_vm, from_am, from_as, from_vs);
			s_as2as(relative_pm, relative_vs, relative_as, from_vs, from_as, to_as, to_vs);
			s_as2am(to_vs, to_as, to_pm, to_am, to_vm);
		}
		auto s_inv_am2am(const double *inv_relative_pm, const double *inv_relative_vs, const double *inv_relative_as,
			const double *from_pm, const double *from_vm, const double *from_am, double *to_am, double *to_vm, double *to_pm) noexcept->void
		{
			// 补充默认参数 //
			inv_relative_pm = inv_relative_pm ? inv_relative_pm : default_pm();
			inv_relative_vs = inv_relative_vs ? inv_relative_vs : default_vs();
			inv_relative_as = inv_relative_as ? inv_relative_as : default_as();
			from_pm = from_pm ? from_pm : default_pm();
			from_vm = from_vm ? from_vm : default_vm();
			from_am = from_am ? from_am : default_am();
			double default_to_pm[16];
			double default_to_vm[16];
			double default_to_am[16];
			to_am = to_am ? to_am : default_to_am;
			to_vm = to_vm ? to_vm : default_to_vm;
			to_pm = to_pm ? to_pm : default_to_pm;

			// 正式开始计算 //
			s_inv_pm2pm(inv_relative_pm, from_pm, to_pm);

			double from_vs[6], from_as[6], to_vs[6], to_as[6];
			s_am2as(from_pm, from_vm, from_am, from_as, from_vs);
			s_inv_as2as(inv_relative_pm, inv_relative_vs, inv_relative_as, from_vs, from_as, to_as, to_vs);
			s_as2am(to_vs, to_as, to_pm, to_am, to_vm);
		}
		auto s_aa2aa(const double *relative_pm, const double *relative_vs, const double *relative_as,
			const double *from_pp, const double *from_va, const double *from_aa, double *to_aa, double *to_va, double *to_pp) noexcept->void
		{
			// 补充默认参数 //
			relative_pm = relative_pm ? relative_pm : default_pm();
			relative_vs = relative_vs ? relative_vs : default_vs();
			relative_as = relative_as ? relative_as : default_as();
			from_pp = from_pp ? from_pp : default_pp();
			from_va = from_va ? from_va : default_va();
			from_aa = from_aa ? from_aa : default_aa();
			double default_to_pp[3];
			double default_to_ve[6];
			double default_to_ae[6];
			to_aa = to_aa ? to_aa : default_to_ae;
			to_va = to_va ? to_va : default_to_ve;
			to_pp = to_pp ? to_pp : default_to_pp;

			// 正式开始计算 //
			s_pp2pp(relative_pm, from_pp, to_pp);

			double from_vs[6], from_as[6], to_vs[6], to_as[6];
			s_aa2as(from_pp, from_va, from_aa, from_as, from_vs);
			s_as2as(relative_pm, relative_vs, relative_as, from_vs, from_as, to_as, to_vs);
			s_as2aa(to_vs, to_as, to_pp, to_aa, to_va);
		}
		auto s_inv_aa2aa(const double *inv_relative_pm, const double *inv_relative_vs, const double *inv_relative_as,
			const double *from_pp, const double *from_va, const double *from_aa, double *to_aa, double *to_va, double *to_pp) noexcept->void
		{
			// 补充默认参数 //
			inv_relative_pm = inv_relative_pm ? inv_relative_pm : default_pm();
			inv_relative_vs = inv_relative_vs ? inv_relative_vs : default_vs();
			inv_relative_as = inv_relative_as ? inv_relative_as : default_as();
			from_pp = from_pp ? from_pp : default_pp();
			from_va = from_va ? from_va : default_va();
			from_aa = from_aa ? from_aa : default_aa();
			double default_to_pp[3];
			double default_to_ve[6];
			double default_to_ae[6];
			to_aa = to_aa ? to_aa : default_to_ae;
			to_va = to_va ? to_va : default_to_ve;
			to_pp = to_pp ? to_pp : default_to_pp;

			// 正式开始计算 //
			s_inv_pp2pp(inv_relative_pm, from_pp, to_pp);

			double from_vs[6], from_as[6], to_vs[6], to_as[6];
			s_aa2as(from_pp, from_va, from_aa, from_as, from_vs);
			s_inv_as2as(inv_relative_pm, inv_relative_vs, inv_relative_as, from_vs, from_as, to_as, to_vs);
			s_as2aa(to_vs, to_as, to_pp, to_aa, to_va);
		}
		auto s_as2as(const double *relative_pm, const double *relative_vs, const double *relative_as,
			const double *from_vs, const double *from_as, double *to_as, double *to_vs) noexcept->void
		{
			// 补充默认参数 //
			relative_pm = relative_pm ? relative_pm : default_pm();
			relative_vs = relative_vs ? relative_vs : default_vs();
			relative_as = relative_as ? relative_as : default_as();
			from_vs = from_vs ? from_vs : default_vs();
			from_as = from_as ? from_as : default_as();
			double default_to_vs[6]{ 0,0,0,0,0,0 };
			double default_to_as[6]{ 0,0,0,0,0,0 };
			to_vs = to_vs ? to_vs : default_to_vs;
			to_as = to_as ? to_as : default_to_as;

			// 正式开始计算 //
			s_vs2vs(relative_pm, relative_vs, from_vs, to_vs);
			s_cv(relative_vs, to_vs, to_as);
			s_tv(1, relative_pm, from_as, 1, to_as);
			s_va(6, 1, relative_as, 1, to_as);
		}
		auto s_inv_as2as(const double *inv_relative_pm, const double *inv_relative_vs, const double *inv_relative_as,
			const double *from_vs, const double *from_as, double *to_as, double *to_vs) noexcept->void
		{
			// 补充默认参数 //
			inv_relative_pm = inv_relative_pm ? inv_relative_pm : default_pm();
			inv_relative_vs = inv_relative_vs ? inv_relative_vs : default_vs();
			inv_relative_as = inv_relative_as ? inv_relative_as : default_as();
			from_vs = from_vs ? from_vs : default_vs();
			from_as = from_as ? from_as : default_as();
			double default_to_vs[6]{ 0,0,0,0,0,0 };
			double default_to_as[6]{ 0,0,0,0,0,0 };
			to_vs = to_vs ? to_vs : default_to_vs;
			to_as = to_as ? to_as : default_to_as;

			// 正式开始计算 //
			s_inv_vs2vs(inv_relative_pm, inv_relative_vs, from_vs, to_vs);
			double tem[6];
			std::fill_n(to_as, 6, 0);
			std::copy_n(from_as, 6, tem);
			s_va(6, -1, inv_relative_as, 1, tem);
			s_cv(-1, inv_relative_vs, from_vs, 1, tem);
			s_inv_tv(inv_relative_pm, tem, to_as);
		}

		auto s_fs2fs(const double *relative_pm, const double *from_fs, double *to_fs) noexcept->void
		{
			// 补充默认参数 //
			relative_pm = relative_pm ? relative_pm : default_pm();
			from_fs = from_fs ? from_fs : default_fs();
			double default_to_fs[6];
			to_fs = to_fs ? to_fs : default_to_fs;

			// 正式开始计算 //
			s_tf(relative_pm, from_fs, to_fs);
		}
		auto s_inv_fs2fs(const double *inv_relative_pm, const double *from_fs, double *to_fs) noexcept->void
		{
			// 补充默认参数 //
			inv_relative_pm = inv_relative_pm ? inv_relative_pm : default_pm();
			from_fs = from_fs ? from_fs : default_fs();
			double default_to_fs[6];
			to_fs = to_fs ? to_fs : default_to_fs;

			// 正式开始计算 //
			s_inv_tf(inv_relative_pm, from_fs, to_fs);
		}
		auto s_is2is(const double *relative_pm, const double *from_is, double *to_is) noexcept->void
		{
			// 补充默认参数 //
			relative_pm = relative_pm ? relative_pm : default_pm();
			from_is = from_is ? from_is : default_is();
			double default_to_is[36];
			to_is = to_is ? to_is : default_to_is;
			
			/*以下为慢速但准确的算法*/
			std::fill_n(to_is, 36, 0);
			double tem[6][6], tmf[6][6];
			s_tmf(relative_pm, *tmf);
			s_mdm(6, 6, 6, *tmf, 6, from_is, 6, *tem, 6);
			s_mdmNT(6, 6, 6, 1, *tem, 6, *tmf, 6, 0, to_is, 6);
		}
		auto s_inv_is2is(const double *inv_relative_pm, const double *from_is, double *to_is) noexcept->void
		{
			// 补充默认参数 //
			inv_relative_pm = inv_relative_pm ? inv_relative_pm : default_pm();
			from_is = from_is ? from_is : default_is();
			double default_to_is[36];
			to_is = to_is ? to_is : default_to_is;

			/*以下为慢速但准确的算法*/
			std::fill_n(to_is, 36, 0);
			double tem[6][6], tmf[6][6], pm[4][4];
			s_inv_pm(inv_relative_pm, *pm);
			s_tmf(*pm, *tmf);
			s_mdm(6, 6, 6, *tmf, 6, from_is, 6, *tem, 6);
			s_mdmNT(6, 6, 6, 1, *tem, 6, *tmf, 6, 0, to_is, 6);
		}

		auto s_tmf(const double *pm_in, double *tmf_out) noexcept->void
		{
			std::fill_n(tmf_out + 3, 3, 0);
			std::fill_n(tmf_out + 9, 3, 0);
			std::fill_n(tmf_out + 15, 3, 0);

			std::copy_n(&pm_in[0], 3, &tmf_out[0]);
			std::copy_n(&pm_in[4], 3, &tmf_out[6]);
			std::copy_n(&pm_in[8], 3, &tmf_out[12]);
			std::copy_n(&pm_in[0], 3, &tmf_out[21]);
			std::copy_n(&pm_in[4], 3, &tmf_out[27]);
			std::copy_n(&pm_in[8], 3, &tmf_out[33]);

			tmf_out[18] = -pm_in[11] * pm_in[4] + pm_in[7] * pm_in[8];
			tmf_out[24] = pm_in[11] * pm_in[0] - pm_in[3] * pm_in[8];
			tmf_out[30] = -pm_in[7] * pm_in[0] + pm_in[3] * pm_in[4];
			tmf_out[19] = -pm_in[11] * pm_in[5] + pm_in[7] * pm_in[9];
			tmf_out[25] = pm_in[11] * pm_in[1] - pm_in[3] * pm_in[9];
			tmf_out[31] = -pm_in[7] * pm_in[1] + pm_in[3] * pm_in[5];
			tmf_out[20] = -pm_in[11] * pm_in[6] + pm_in[7] * pm_in[10];
			tmf_out[26] = pm_in[11] * pm_in[2] - pm_in[3] * pm_in[10];
			tmf_out[32] = -pm_in[7] * pm_in[2] + pm_in[3] * pm_in[6];
		}
		auto s_tmv(const double *pm_in, double *tmv_out) noexcept->void
		{
			std::fill_n(tmv_out + 18, 3, 0);
			std::fill_n(tmv_out + 24, 3, 0);
			std::fill_n(tmv_out + 30, 3, 0);

			std::copy_n(&pm_in[0], 3, &tmv_out[0]);
			std::copy_n(&pm_in[4], 3, &tmv_out[6]);
			std::copy_n(&pm_in[8], 3, &tmv_out[12]);
			std::copy_n(&pm_in[0], 3, &tmv_out[21]);
			std::copy_n(&pm_in[4], 3, &tmv_out[27]);
			std::copy_n(&pm_in[8], 3, &tmv_out[33]);

			tmv_out[3] = -pm_in[11] * pm_in[4] + pm_in[7] * pm_in[8];
			tmv_out[9] = pm_in[11] * pm_in[0] - pm_in[3] * pm_in[8];
			tmv_out[15] = -pm_in[7] * pm_in[0] + pm_in[3] * pm_in[4];
			tmv_out[4] = -pm_in[11] * pm_in[5] + pm_in[7] * pm_in[9];
			tmv_out[10] = pm_in[11] * pm_in[1] - pm_in[3] * pm_in[9];
			tmv_out[16] = -pm_in[7] * pm_in[1] + pm_in[3] * pm_in[5];
			tmv_out[5] = -pm_in[11] * pm_in[6] + pm_in[7] * pm_in[10];
			tmv_out[11] = pm_in[11] * pm_in[2] - pm_in[3] * pm_in[10];
			tmv_out[17] = -pm_in[7] * pm_in[2] + pm_in[3] * pm_in[6];
		}
		auto s_tf(const double *pm_in, const double *fce_in, double *vec_out) noexcept->void
		{
			s_pm_dot_v3(pm_in, fce_in, vec_out);
			s_pm_dot_v3(pm_in, fce_in + 3, vec_out + 3);

			vec_out[3] += -pm_in[11] * vec_out[1] + pm_in[7] * vec_out[2];
			vec_out[4] += pm_in[11] * vec_out[0] - pm_in[3] * vec_out[2];
			vec_out[5] += -pm_in[7] * vec_out[0] + pm_in[3] * vec_out[1];
		}
		auto s_tf(double alpha, const double *pm_in, const double *fce_in, double beta, double *vec_out) noexcept->void
		{
			double tem[6];

			s_tf(pm_in, fce_in, tem);

			for (int i = 0; i < 6; ++i)
			{
				vec_out[i] = alpha * tem[i] + beta * vec_out[i];
			}
		}
		auto s_tf_n(int n, const double *pm_in, const double *fces_in, double *m_out) noexcept->void
		{
			std::fill_n(m_out, 6 * n, 0);
			
			s_mdm(3, n, 3, 1, pm_in, 4, fces_in, n, 0, m_out, n);
			s_mdm(3, n, 3, 1, pm_in, 4, fces_in + 3 * n, n, 0, m_out + 3 * n, n);

			for (int i = 0; i < n; ++i)
			{
				m_out[n * 3 + i] += -pm_in[11] * m_out[n + i] + pm_in[7] * m_out[n * 2 + i];
				m_out[n * 4 + i] += pm_in[11] * m_out[i] - pm_in[3] * m_out[n * 2 + i];
				m_out[n * 5 + i] += -pm_in[7] * m_out[i] + pm_in[3] * m_out[n + i];
			}
		}
		auto s_tf_n(int n, double alpha, const double *pm_in, const double *fces_in, double beta, double *m_out) noexcept->void
		{
			double vRm[3][3];

			for (int i = 0; i < 3; ++i)
			{
				vRm[0][i] = -pm_in[11] * pm_in[4 + i] + pm_in[7] * pm_in[8 + i];
				vRm[1][i] = pm_in[11] * pm_in[i] - pm_in[3] * pm_in[8 + i];
				vRm[2][i] = -pm_in[7] * pm_in[i] + pm_in[3] * pm_in[4 + i];
			}

			s_mdm(3, n, 3, alpha, pm_in, 4, fces_in, n, beta, m_out, n);
			s_mdm(3, n, 3, alpha, pm_in, 4, fces_in + 3 * n, n, beta, m_out + 3 * n, n);
			s_mdm(3, n, 3, alpha, *vRm, 3, fces_in, n, 1, m_out + 3 * n, n);
		}
		auto s_inv_tf(const double *inv_pm_in, const double *fce_in, double *vec_out) noexcept->void
		{
			double pm_in[16];
			s_inv_pm(inv_pm_in, pm_in);
			s_tf(pm_in, fce_in, vec_out);
		}
		auto s_inv_tf(double alpha, const double *inv_pm_in, const double *vs_in, double beta, double *vec_out) noexcept->void
		{
			double pm_in[16];
			s_inv_pm(inv_pm_in, pm_in);
			s_tf(alpha, pm_in, vs_in, beta, vec_out);
		}
		auto s_tv(const double *pm_in, const double *vs_in, double *vec_out) noexcept->void
		{
			s_pm_dot_v3(pm_in, vs_in, vec_out);
			s_pm_dot_v3(pm_in, vs_in + 3, vec_out + 3);

			vec_out[0] += -pm_in[11] * vec_out[4] + pm_in[7] * vec_out[5];
			vec_out[1] += pm_in[11] * vec_out[3] - pm_in[3] * vec_out[5];
			vec_out[2] += -pm_in[7] * vec_out[3] + pm_in[3] * vec_out[4];
		}
		auto s_tv(double alpha, const double *pm_in, const double *vs_in, double beta, double *vec_out) noexcept->void
		{
			double tem[6];
			
			s_tv(pm_in, vs_in, tem);

			for (int i = 0; i < 6; ++i)
			{
				vec_out[i] = alpha * tem[i] + beta * vec_out[i];
			}
		}
		auto s_tv_n(int n, const double *pm_in, const double *vels_in, double *m_out) noexcept->void
		{
			std::fill_n(m_out, 6 * n, 0);
			
			s_mdm(3, n, 3, 1, pm_in, 4, vels_in, n, 0, m_out, n);
			s_mdm(3, n, 3, 1, pm_in, 4, vels_in + 3 * n, n, 0, m_out + 3 * n, n);

			for (int i = 0; i < n; ++i)
			{
				m_out[n * 0 + i] += -pm_in[11] * m_out[4 * n + i] + pm_in[7] * m_out[5 * n + i];
				m_out[n * 1 + i] += pm_in[11] * m_out[3 * n + i] - pm_in[3] * m_out[5 * n + i];
				m_out[n * 2 + i] += -pm_in[7] * m_out[3 * n + i] + pm_in[3] * m_out[4 * n + i];
			}
		}
		auto s_tv_n(int n, double alpha, const double *pm_in, const double *vels_in, double beta, double *m_out) noexcept->void
		{
			double vRm[3][3];

			for (int i = 0; i < 3; ++i)
			{
				vRm[0][i] = -pm_in[11] * pm_in[4 + i] + pm_in[7] * pm_in[8 + i];
				vRm[1][i] = pm_in[11] * pm_in[i] - pm_in[3] * pm_in[8 + i];
				vRm[2][i] = -pm_in[7] * pm_in[i] + pm_in[3] * pm_in[4 + i];
			}

			s_mdm(3, n, 3, alpha, pm_in, 4, vels_in, n, beta, m_out, n);
			s_mdm(3, n, 3, alpha, pm_in, 4, vels_in + 3 * n, n, beta, m_out + 3 * n, n);
			s_mdm(3, n, 3, alpha, *vRm, 3, vels_in + 3 * n, n, 1, m_out, n);
		}
		auto s_inv_tv(const double *inv_pm_in, const double *vs_in, double *vec_out) noexcept->void
		{
			double pm_in[16];
			s_inv_pm(inv_pm_in, pm_in);
			s_tv(pm_in, vs_in, vec_out);
		}
		auto s_inv_tv(double alpha, const double *inv_pm_in, const double *vs_in, double beta, double *vec_out) noexcept->void
		{
			double pm_in[16];
			s_inv_pm(inv_pm_in, pm_in);
			s_tv(alpha, pm_in, vs_in, beta, vec_out);
		}
		auto s_inv_tv_n(int n, const double *inv_pm_in, const double *vs_in, double *vec_out) noexcept->void
		{
			double pm_in[16];
			s_inv_pm(inv_pm_in, pm_in);
			s_tv_n(n, pm_in, vs_in, vec_out);
		}
		auto s_inv_tv_n(int n, double alpha, const double *inv_pm_in, const double *vs_in, double beta, double *vec_out) noexcept->void
		{
			double pm_in[16];
			s_inv_pm(inv_pm_in, pm_in);
			s_tv_n(n, alpha, pm_in, vs_in, beta, vec_out);
		}
		
		auto s_cm3(const double *cro_vec_in, double *cm_out) noexcept->void
		{
			cm_out[0] = 0;
			cm_out[1] = -cro_vec_in[2];
			cm_out[2] = cro_vec_in[1];
			cm_out[3] = cro_vec_in[2];
			cm_out[4] = 0;
			cm_out[5] = -cro_vec_in[0];
			cm_out[6] = -cro_vec_in[1];
			cm_out[7] = cro_vec_in[0];
			cm_out[8] = 0;
		}
		auto s_c3(const double *cro_vec_in, const double *vec_in, double *vec_out) noexcept->void
		{
			vec_out[0] = -cro_vec_in[2] * vec_in[1] + cro_vec_in[1] * vec_in[2];
			vec_out[1] = cro_vec_in[2] * vec_in[0] - cro_vec_in[0] * vec_in[2];
			vec_out[2] = -cro_vec_in[1] * vec_in[0] + cro_vec_in[0] * vec_in[1];
		}
		auto s_c3(const double *cro_vec_in, const double *vec_in, std::size_t vec_in_ld, double *vec_out, std::size_t vec_out_ld) noexcept->void
		{
			vec_out[0] = -cro_vec_in[2] * vec_in[vec_in_ld] + cro_vec_in[1] * vec_in[2 * vec_in_ld];
			vec_out[vec_out_ld] = cro_vec_in[2] * vec_in[0] - cro_vec_in[0] * vec_in[2 * vec_in_ld];
			vec_out[2 * vec_out_ld] = -cro_vec_in[1] * vec_in[0] + cro_vec_in[0] * vec_in[vec_in_ld];
		}
		auto s_c3(double alpha, const double *cro_vec_in, const double *vec_in, double beta, double *vec_out) noexcept->void
		{
			vec_out[0] *= beta;
			vec_out[1] *= beta;
			vec_out[2] *= beta;

			vec_out[0] += alpha*(-cro_vec_in[2] * vec_in[1] + cro_vec_in[1] * vec_in[2]);
			vec_out[1] += alpha*(cro_vec_in[2] * vec_in[0] - cro_vec_in[0] * vec_in[2]);
			vec_out[2] += alpha*(-cro_vec_in[1] * vec_in[0] + cro_vec_in[0] * vec_in[1]);
		}
		auto s_c3(double alpha, const double *cro_vec_in, const double *vec_in, std::size_t vec_in_ld, double beta, double *vec_out, std::size_t vec_out_ld) noexcept->void
		{
			vec_out[0] *= beta;
			vec_out[vec_out_ld] *= beta;
			vec_out[2 * vec_out_ld] *= beta;

			vec_out[0] += alpha*(-cro_vec_in[2] * vec_in[vec_in_ld] + cro_vec_in[1] * vec_in[2 * vec_in_ld]);
			vec_out[vec_out_ld] += alpha*(cro_vec_in[2] * vec_in[0] - cro_vec_in[0] * vec_in[2 * vec_in_ld]);
			vec_out[2 * vec_out_ld] += alpha*(-cro_vec_in[1] * vec_in[0] + cro_vec_in[0] * vec_in[vec_in_ld]);
		}
		auto s_c3_n(std::size_t n, const double *cro_vec_in, const double *mat_in, std::size_t mat_in_ld, double *mat_out, std::size_t mat_out_ld) noexcept->void
		{
			for (std::size_t i = 0; i < n; ++i)s_c3(cro_vec_in, mat_in + i, mat_in_ld, mat_out + i, mat_out_ld);
		}
		auto s_c3_n(std::size_t n, double alpha, const double *cro_vec_in, const double *mat_in, std::size_t mat_in_ld, double beta, double *mat_out, std::size_t mat_out_ld) noexcept->void
		{
			for (std::size_t i = 0; i < n; ++i)s_c3(alpha, cro_vec_in, mat_in + i, mat_in_ld, beta, mat_out + i, mat_out_ld);
		}
		auto s_cmf(const double *vs_in, double *cmf_out) noexcept->void
		{
			std::fill_n(cmf_out, 36, 0);

			cmf_out[6] = vs_in[5];
			cmf_out[12] = -vs_in[4];
			cmf_out[1] = -vs_in[5];
			cmf_out[13] = vs_in[3];
			cmf_out[2] = vs_in[4];
			cmf_out[8] = -vs_in[3];

			cmf_out[27] = vs_in[5];
			cmf_out[33] = -vs_in[4];
			cmf_out[22] = -vs_in[5];
			cmf_out[34] = vs_in[3];
			cmf_out[23] = vs_in[4];
			cmf_out[29] = -vs_in[3];

			cmf_out[24] = vs_in[2];
			cmf_out[30] = -vs_in[1];
			cmf_out[19] = -vs_in[2];
			cmf_out[31] = vs_in[0];
			cmf_out[20] = vs_in[1];
			cmf_out[26] = -vs_in[0];
		}
		auto s_cf(const double *cro_vel_in, const double *vec_in, double* vec_out) noexcept->void
		{
			s_c3(cro_vel_in + 3, vec_in, vec_out);
			s_c3(cro_vel_in + 3, vec_in + 3, vec_out + 3);

			vec_out[3] += -cro_vel_in[2] * vec_in[1] + cro_vel_in[1] * vec_in[2];
			vec_out[4] += cro_vel_in[2] * vec_in[0] - cro_vel_in[0] * vec_in[2];
			vec_out[5] += -cro_vel_in[1] * vec_in[0] + cro_vel_in[0] * vec_in[1];
		}
		auto s_cf(double alpha, const double *cro_vel_in, const double *vec_in, double beta, double* vec_out) noexcept->void
		{
			s_c3(alpha, cro_vel_in + 3, vec_in, beta, vec_out);
			s_c3(alpha, cro_vel_in + 3, vec_in + 3, beta, vec_out + 3);

			vec_out[3] += alpha*(-cro_vel_in[2] * vec_in[1] + cro_vel_in[1] * vec_in[2]);
			vec_out[4] += alpha*(cro_vel_in[2] * vec_in[0] - cro_vel_in[0] * vec_in[2]);
			vec_out[5] += alpha*(-cro_vel_in[1] * vec_in[0] + cro_vel_in[0] * vec_in[1]);
		}
		auto s_cmv(const double *vs_in, double *cmv_out) noexcept->void
		{
			std::fill_n(cmv_out, 36, 0);

			cmv_out[6] = vs_in[5];
			cmv_out[12] = -vs_in[4];
			cmv_out[1] = -vs_in[5];
			cmv_out[13] = vs_in[3];
			cmv_out[2] = vs_in[4];
			cmv_out[8] = -vs_in[3];

			cmv_out[27] = vs_in[5];
			cmv_out[33] = -vs_in[4];
			cmv_out[22] = -vs_in[5];
			cmv_out[34] = vs_in[3];
			cmv_out[23] = vs_in[4];
			cmv_out[29] = -vs_in[3];

			cmv_out[9] = vs_in[2];
			cmv_out[15] = -vs_in[1];
			cmv_out[4] = -vs_in[2];
			cmv_out[16] = vs_in[0];
			cmv_out[5] = vs_in[1];
			cmv_out[11] = -vs_in[0];
		}
		auto s_cv(const double *cro_vel_in, const double *vec_in, double* vec_out) noexcept->void
		{
			s_c3(cro_vel_in + 3, vec_in, vec_out);
			s_c3(cro_vel_in + 3, vec_in+3, vec_out+3);
			
			vec_out[0] += -cro_vel_in[2] * vec_in[4] + cro_vel_in[1] * vec_in[5];
			vec_out[1] += cro_vel_in[2] * vec_in[3] - cro_vel_in[0] * vec_in[5];
			vec_out[2] += -cro_vel_in[1] * vec_in[3] + cro_vel_in[0] * vec_in[4];
		}
		auto s_cv(double alpha, const double *cro_vel_in, const double *vec_in, double beta, double* vec_out) noexcept->void
		{
			s_c3(alpha, cro_vel_in + 3, vec_in, beta, vec_out);
			s_c3(alpha, cro_vel_in + 3, vec_in + 3, beta, vec_out + 3);

			vec_out[0] += alpha*(-cro_vel_in[2] * vec_in[4] + cro_vel_in[1] * vec_in[5]);
			vec_out[1] += alpha*(cro_vel_in[2] * vec_in[3] - cro_vel_in[0] * vec_in[5]);
			vec_out[2] += alpha*(-cro_vel_in[1] * vec_in[3] + cro_vel_in[0] * vec_in[4]);
		}

		auto s_block_cpy(const int &block_size_m, const int &block_size_n,
			const double *from_mtrx, const int &fm_begin_row, const int &fm_begin_col, const int &fm_ld,
			double *to_mtrx, const int &tm_begin_row, const int &tm_begin_col, const int &tm_ld) noexcept->void
		{
			int fm_place ;
			int tm_place ;
		
			fm_place = fm_begin_row*fm_ld + fm_begin_col;
			tm_place = tm_begin_row*tm_ld + tm_begin_col;

			for (int i = 0; i < block_size_m; i++)
			{
				std::copy_n(&from_mtrx[fm_place], block_size_n, &to_mtrx[tm_place]);

				fm_place += fm_ld;
				tm_place += tm_ld;
			}

		}
		auto s_block_cpy(const int &block_size_m, const int &block_size_n,
			double alpha, const double *from_mtrx, const int &fm_begin_row, const int &fm_begin_col, const int &fm_ld,
			double beta, double *to_mtrx, const int &tm_begin_row, const int &tm_begin_col, const int &tm_ld) noexcept->void
		{
			int fm_place;
			int tm_place;

			fm_place = fm_begin_row*fm_ld + fm_begin_col;
			tm_place = tm_begin_row*tm_ld + tm_begin_col;

			for (int i = 0; i < block_size_m; i++)
			{
				for (int j = 0; j < block_size_n; j++)
				{
					to_mtrx[tm_place + j] = alpha*from_mtrx[fm_place + j] + beta*to_mtrx[tm_place + j];
				}

				fm_place += fm_ld;
				tm_place += tm_ld;
			}

		}
		auto s_block_cpyT(const int &block_size_m, const int &block_size_n,
			const double *from_mtrx, const int &fm_begin_row, const int &fm_begin_col, const int &fm_ld,
			double *to_mtrx, const int &tm_begin_row, const int &tm_begin_col, const int &tm_ld) noexcept->void
		{
			int fm_place;
			int tm_place;

			fm_place = fm_begin_row*fm_ld + fm_begin_col;
			tm_place = tm_begin_row*tm_ld + tm_begin_col;

			for (int i = 0; i < block_size_m; i++)
			{
				for (int j = 0; j < block_size_n; j++)
				{
					to_mtrx[tm_place + tm_ld * j] = from_mtrx[fm_place + j];
				}

				fm_place += fm_ld;
				tm_place += 1;
			}

		}
		auto s_block_cpyT(const int &block_size_m, const int &block_size_n,
			double alpha, const double *from_mtrx, const int &fm_begin_row, const int &fm_begin_col, const int &fm_ld,
			double beta, double *to_mtrx, const int &tm_begin_row, const int &tm_begin_col, const int &tm_ld) noexcept->void
		{
			int fm_place;
			int tm_place;

			fm_place = fm_begin_row*fm_ld + fm_begin_col;
			tm_place = tm_begin_row*tm_ld + tm_begin_col;

			for (int i = 0; i < block_size_m; i++)
			{
				for (int j = 0; j < block_size_n; j++)
				{
					to_mtrx[tm_place + tm_ld * j] = alpha*from_mtrx[fm_place + j] + beta*to_mtrx[tm_place + tm_ld * j];
				}

				fm_place += fm_ld;
				tm_place += 1;
			}

		}

		auto s_nd(int n, double a, double *x, int incx) noexcept->void
		{
			int final_idx = n*incx;
			for (int i = 0; i < final_idx; i += incx)x[i] *= a;
		}
		auto s_ndv(int n, double a, const double *x, int incx, double *y, int incy) noexcept->void
		{
			int x_idx{ 0 }, y_idx{ 0 };
			for (int i = 0; i < n; ++i) 
			{
				y[y_idx] = a*x[x_idx];
				x_idx += incx;
				y_idx += incy;
			}
		}
		auto s_vnm(int n, const double *x, int incx) noexcept->double
		{
			double norm = 0;
			int final_idx = n*incx;
			for (int i = 0; i < final_idx; i += incx)norm += x[i] * x[i];
			return std::sqrt(norm);
		}
		auto s_vsw(int n, double *x, int incx, double *y, int incy) noexcept->void
		{
			int x_idx{ 0 }, y_idx{ 0 };
			for (int i = 0; i < n; ++i)
			{
				std::swap(x[x_idx], y[y_idx]);
				x_idx += incx;
				y_idx += incy;
			}
		}
		auto s_vdv(int n, const double *x, const double *y) noexcept->double
		{
			double ret{ 0 };

			for (int i = 0; i < n; ++i)ret += x[i] * y[i];

			return ret;
		}
		auto s_vdv(int n, const double *x, int incx, const double *y, int incy) noexcept->double
		{
			double ret{ 0 };
			int x_idx{ 0 }, y_idx{ 0 };

			for (int i = 0; i < n; ++i) 
			{
				x_idx += incx;
				y_idx += incy;
				
				ret += x[x_idx] * y[y_idx];
			}
			
			return ret;
		}
		auto s_va(int n, const double* x, double* y) noexcept->void
		{
			for (auto i = 0; i < n; ++i)y[i] += x[i];
		}
		auto s_va(int n, double alpha, const double* x, double beta, double* y) noexcept->void
		{
			for (auto i = 0; i < n; ++i) 
			{
				y[i] *= beta;
				y[i] += alpha * x[i];
			}
		}
		auto s_va(int n, const double* x, int incx, double* y, int incy) noexcept->void
		{
			int x_idx{ 0 }, y_idx{ 0 };

			for (int i = 0; i < n; ++i)
			{
				y[y_idx] += x[x_idx];
				x_idx += incx;
				y_idx += incy;
			}
		}
		auto s_va(int n, double alpha, const double* x, int incx, double beta, double* y, int incy) noexcept->void
		{
			int x_idx{ 0 }, y_idx{ 0 };

			for (int i = 0; i < n; ++i)
			{
				y[y_idx] *= beta;
				y[y_idx] += alpha * x[x_idx];
				x_idx += incx;
				y_idx += incy;
			}
		}
		auto s_vav(int n, const double* x, const double* y, double* z) noexcept->void
		{
			for (auto i = 0; i < n; ++i)z[i] = x[i] + y[i];
		}
		auto s_vav(int n, double alpha, const double* x, double beta, const double* y, double gamma, double* z) noexcept->void
		{
			for (auto i = 0; i < n; ++i)
			{
				z[i] *= gamma;
				z[i] += alpha * x[i] + beta * y[i];
			}
		}
		auto s_vav(int n, const double* x, int incx, const double* y, int incy, double* z, int incz) noexcept->void
		{
			int x_idx{ 0 }, y_idx{ 0 }, z_idx{ 0 };

			for (int i = 0; i < n; ++i)
			{
				z[z_idx] = x[x_idx] + y[y_idx];
				x_idx += incx;
				y_idx += incy;
				z_idx += incz;
			}
		}
		auto s_vav(int n, double alpha, const double* x, int incx, double beta, const double* y, int incy, double gamma, double* z, int incz) noexcept->void
		{
			int x_idx{ 0 }, y_idx{ 0 }, z_idx{ 0 };

			for (int i = 0; i < n; ++i)
			{
				z[z_idx] *= gamma;
				z[z_idx] = alpha * x[x_idx] + beta * y[y_idx];
				x_idx += incx;
				y_idx += incy;
				z_idx += incz;
			}
		}
		auto s_mtm(int m, int n, const double *A, int lda, double *B, int ldb) noexcept->void
		{
			for (int i = 0; i < m; ++i)
			{
				int row_idx = i*lda;
				int B_idx{ i };

				for (int j = 0; j < n; ++j)
				{
					B[B_idx] = A[row_idx + j];
					B_idx += ldb;
				}
			}
		}
		auto s_ma(int m, int n, const double* A, int lda, double* B, int ldb) noexcept->void
		{
			for (int i = 0; i < m; ++i)
			{
				int row_idx_a = i*lda;
				int row_idx_b = i*ldb;

				for (int j = 0; j < n; ++j)B[row_idx_b + j] += A[row_idx_a + j];
			}
		}
		auto s_ma(int m, int n, double alpha, const double* A, int lda, double beta, double* B, int ldb) noexcept->void
		{
			for (int i = 0; i < m; ++i)
			{
				int row_idx_a = i*lda;
				int row_idx_b = i*ldb;

				for (int j = 0; j < n; ++j) 
				{
					B[row_idx_b + j] *= beta;
					B[row_idx_b + j] += alpha * A[row_idx_a + j];
				}
			}
		}
		auto s_mam(int m, int n, const double* A, int lda, const double* B, int ldb, double *C, int ldc) noexcept->void
		{
			for (int i = 0; i < m; ++i)
			{
				int row_idx_a = i*lda;
				int row_idx_b = i*ldb;
				int row_idx_c = i*ldc;

				for (int j = 0; j < n; ++j)C[row_idx_c + j] = A[row_idx_a + j] + B[row_idx_b + j];
			}
		}
		auto s_mam(int m, int n, double alpha, const double* A, int lda, double beta, const double* B, int ldb, double gamma, double *C, int ldc) noexcept->void
		{
			for (int i = 0; i < m; ++i)
			{
				int row_idx_a = i*lda;
				int row_idx_b = i*ldb;
				int row_idx_c = i*ldc;

				for (int j = 0; j < n; ++j) 
				{
					C[row_idx_c + j] = alpha*A[row_idx_a + j] + beta*B[row_idx_b + j];
				}
			}
		}
		auto s_mdm(int m, int n, int k, const double* A, int lda, const double* B, int ldb, double *C, int ldc) noexcept->void
		{
			for (int i = 0; i < m; ++i)
			{
				int row_idx = i*lda;
				for (int j = 0; j < n; ++j)
				{
					int idx = i*ldc + j;

					C[idx] = 0;
					for (int u = 0; u < k; ++u)
					{
						C[idx] += A[row_idx + u] * B[j + u*ldb];
					}
				}
			}
		}
		auto s_mdm(int m, int n, int k, double alpha, const double* A, int lda, const double* B, int ldb, double beta, double *C, int ldc) noexcept->void
		{
			for (int i = 0; i < m; ++i)
			{
				int row_idx = i*lda;
				for (int j = 0; j < n; ++j)
				{
					int idx = i*ldc + j;
					
					double add_factor = 0;
					for (int u = 0; u < k; ++u)
					{
						add_factor +=  A[row_idx + u] * B[j + u*ldb];
					}
					
					C[idx] *= beta;
					C[idx] += alpha *add_factor;
				}
			}
		}
		auto s_mdmTN(int m, int n, int k, const double* A, int lda, const double* B, int ldb, double *C, int ldc) noexcept->void
		{
			for (int i = 0; i < m; ++i)
			{
				for (int j = 0; j < n; ++j)
				{
					int idx = i*ldc + j;
					
					C[idx] = 0;
					for (int u = 0; u < k; ++u)
					{
						C[idx] += A[i + u*lda] * B[j + u*ldb];
					}
				}
			}
		}
		auto s_mdmTN(int m, int n, int k, double alpha, const double* A, int lda, const double* B, int ldb, double beta, double *C, int ldc) noexcept->void
		{
			for (int i = 0; i < m; ++i)
			{
				for (int j = 0; j < n; ++j)
				{
					int idx = i*ldc + j;
	
					double add_factor = 0;
					for (int u = 0; u < k; ++u)
					{
						add_factor += A[i + u*lda] * B[j + u*ldb];
					}

					C[idx] *= beta;
					C[idx] += alpha *add_factor;
				}
			}
		}
		auto s_mdmNT(int m, int n, int k, const double* A, int lda, const double* B, int ldb, double *C, int ldc) noexcept->void
		{
			for (int i = 0; i < m; ++i)
			{
				int row_idx = i*lda;
				for (int j = 0; j < n; ++j)
				{
					int col_idx = j*ldb;

					int idx = i*ldc + j;
					
					C[idx] = 0;
					for (int u = 0; u < k; ++u)
					{
						C[idx] += A[row_idx + u] * B[col_idx + u];
					}
				}
			}
		}
		auto s_mdmNT(int m, int n, int k, double alpha, const double* A, int lda, const double* B, int ldb, double beta, double *C, int ldc) noexcept->void
		{
			for (int i = 0; i < m; ++i)
			{
				int row_idx = i*lda;
				for (int j = 0; j < n; ++j)
				{
					int col_idx = j*ldb;
					int idx = i*ldc + j;
					
					double add_factor{ 0 };
					for (int u = 0; u < k; ++u)
					{
						add_factor += A[row_idx + u] * B[col_idx + u];
					}

					C[idx] *= beta;
					C[idx] += alpha * add_factor;
				}
			}
		}
		auto s_mdmTT(int m, int n, int k, const double* A, int lda, const double* B, int ldb, double *C, int ldc) noexcept->void
		{
			for (int i = 0; i < m; ++i)
			{
				for (int j = 0; j < n; ++j)
				{
					int col_idx = j*ldb;
					int idx = i*ldc + j;

					C[idx] = 0;
					for (int u = 0; u < k; ++u)
					{
						C[idx] += A[i + u*lda] * B[col_idx + u];
					}
				}
			}
		}
		auto s_mdmTT(int m, int n, int k, double alpha, const double* A, int lda, const double* B, int ldb, double beta, double *C, int ldc) noexcept->void
		{
			for (int i = 0; i < m; ++i)
			{
				for (int j = 0; j < n; ++j)
				{
					int col_idx = j*ldb;
					int idx = i*ldc + j;

					double add_factor{ 0 };
					for (int u = 0; u < k; ++u)
					{
						add_factor += A[i + u*lda] * B[col_idx + u];
					}

					C[idx] *= beta;
					C[idx] += alpha * add_factor;
				}
			}
		}

		auto s_dlt_col(const int &dlt_col_num, const int *col_index, const int &m, const int &n, double *A, const int &ldA) noexcept->void
		{
			for (int i = 0; i < dlt_col_num; ++i)
			{
				for (int k = col_index[i]; k < ((i == dlt_col_num - 1) ? (n) : (col_index[i + 1])); ++k)
				{
					for (int j = 0; j < m; ++j)
					{
						A[j*ldA + k - i] = A[j*ldA + k + 1];
					}
				}
			}
		}

		auto s_inv_pm(const double *pm_in, double *pm_out) noexcept->void
		{
			//转置
			pm_out[0] = pm_in[0];
			pm_out[1] = pm_in[4];
			pm_out[2] = pm_in[8];
			pm_out[4] = pm_in[1];
			pm_out[5] = pm_in[5];
			pm_out[6] = pm_in[9];
			pm_out[8] = pm_in[2];
			pm_out[9] = pm_in[6];
			pm_out[10] = pm_in[10];

			//位置
			pm_out[3] = -pm_out[0] * pm_in[3] - pm_out[1] * pm_in[7] - pm_out[2] * pm_in[11];
			pm_out[7] = -pm_out[4] * pm_in[3] - pm_out[5] * pm_in[7] - pm_out[6] * pm_in[11];
			pm_out[11] = -pm_out[8] * pm_in[3] - pm_out[9] * pm_in[7] - pm_out[10] * pm_in[11];

			//其他
			pm_out[12] = 0;
			pm_out[13] = 0;
			pm_out[14] = 0;
			pm_out[15] = 1;
		}
		auto s_pm_dot_pm(const double *pm1_in, const double *pm2_in, double *pm_out) noexcept->void
		{
			/*seemed that loop is faster than cblas*/
			for (int i = 0; i < 3; ++i)
			{
				for (int j = 0; j < 4; ++j)
				{
					pm_out[i * 4 + j] = pm1_in[i * 4] * pm2_in[j] + pm1_in[i * 4 + 1] * pm2_in[j + 4] + pm1_in[i * 4 + 2] * pm2_in[j + 8];
				}
			}

			pm_out[3] += pm1_in[3];
			pm_out[7] += pm1_in[7];
			pm_out[11] += pm1_in[11];

			pm_out[12] = 0;
			pm_out[13] = 0;
			pm_out[14] = 0;
			pm_out[15] = 1;
		}
		auto s_inv_pm_dot_pm(const double *inv_pm1_in, const double *pm2_in, double *pm_out) noexcept->void
		{
			/*seemed that loop is faster than cblas*/
			for (int i = 0; i < 3; ++i)
			{
				for (int j = 0; j < 4; ++j)
				{
					pm_out[i * 4 + j] = inv_pm1_in[i] * pm2_in[j] + inv_pm1_in[i + 4] * pm2_in[j + 4] + inv_pm1_in[i + 8] * pm2_in[j + 8];
				}
			}

			pm_out[3] += -inv_pm1_in[0] * inv_pm1_in[3] - inv_pm1_in[4] * inv_pm1_in[7] - inv_pm1_in[8] * inv_pm1_in[11];
			pm_out[7] += -inv_pm1_in[1] * inv_pm1_in[3] - inv_pm1_in[5] * inv_pm1_in[7] - inv_pm1_in[9] * inv_pm1_in[11];
			pm_out[11] += -inv_pm1_in[2] * inv_pm1_in[3] - inv_pm1_in[6] * inv_pm1_in[7] - inv_pm1_in[10] * inv_pm1_in[11];

			pm_out[12] = 0;
			pm_out[13] = 0;
			pm_out[14] = 0;
			pm_out[15] = 1;
		}
		auto s_pm_dot_inv_pm(const double *pm1_in, const double *inv_pm2_in, double *pm_out) noexcept->void
		{
			double tem[16];
			s_inv_pm(inv_pm2_in, tem);
			s_pm_dot_pm(pm1_in, tem, pm_out);
		}
		auto s_pm_dot_v3(const double *pm_in, const double *v3_in, double *v3_out) noexcept->void
		{
			// seemed that loop is faster than cblas //
			for (int i = 0; i < 3; ++i)
			{
				v3_out[i] = pm_in[i * 4] * v3_in[0] + pm_in[i * 4 + 1] * v3_in[1] + pm_in[i * 4 + 2] * v3_in[2];
			}
		}
		auto s_inv_pm_dot_v3(const double *inv_pm_in, const double *v3_in, double *v3_out) noexcept->void
		{
			for (int i = 0; i < 3; ++i)
			{
				v3_out[i] = inv_pm_in[i] * v3_in[0] + inv_pm_in[i + 4] * v3_in[1] + inv_pm_in[i + 8] * v3_in[2];
			}
		}
		auto s_m6_dot_v6(const double *m6_in, const double *v6_in, double *v6_out) noexcept->void
		{
			// seemed that loop is faster than cblas //
			for (int i = 0; i < 6; ++i)
			{
				v6_out[i] = m6_in[i * 6] * v6_in[0] + m6_in[i * 6 + 1] * v6_in[1] + m6_in[i * 6 + 2] * v6_in[2] +
					m6_in[i * 6 + 3] * v6_in[3] + m6_in[i * 6 + 4] * v6_in[4] + m6_in[i * 6 + 5] * v6_in[5];
			}
		}

		auto s_axes2pm(const double *origin, const double *firstAxisPnt, const double *secondAxisPnt, double *pm_out, const char *axesOrder) noexcept->void
		{
			int Order[3];
			double Axis1[3], Axis2[3], Axis3[3];
			double nrm;

			Order[0] = axesOrder[0] - 'w';//asc玛顺序 uvw  xyz……
			Order[1] = axesOrder[1] - 'w';//asc玛顺序 uvw  xyz……
			Order[2] = 6 - Order[1] - Order[0];

			std::copy_n(firstAxisPnt, 3, Axis1);
			std::copy_n(secondAxisPnt, 3, Axis2);

			s_va(3, -1, origin, 1, Axis1);
			s_va(3, -1, origin, 1, Axis2);

			nrm = s_vnm(3, Axis1, 1);
			if (nrm != 0)
			{
				for (int i = 0; i < 3; ++i)
				{
					Axis1[i] = Axis1[i] / nrm;
				}
			}
			else
			{
				Axis1[Order[0]] = 1;
			}

			nrm = s_vnm(3, Axis2, 1);
			if (nrm != 0)
			{
				for (int i = 0; i < 3; ++i)
				{
					Axis2[i] = Axis2[i] / nrm;
				}
			}
			else
			{
				Axis2[Order[1]] = 1;
			}

			//下求差乘计算。首先要判断差乘之后的正负号。
			s_c3(Axis1, Axis2, Axis3);

			nrm = s_vnm(3, Axis3, 1);
			if (nrm == 0)
			{
				//有待补充
			}
			else
			{
				for (int i = 0; i < 3; ++i)
				{
					Axis3[i] = Axis3[i] / nrm;
				}
			}

			//判断第三根轴的正负号
			if (Order[1]>Order[0])
			{
				if ((Order[1] - Order[0]) == 1)
				{
				}
				else
				{
					s_nd(3, -1, Axis3, 1);
				}
			}
			else
			{
				if ((Order[1] - Order[0]) == -1)
				{
					s_nd(3, -1, Axis3, 1);
				}
				else
				{
				}
			}

			s_c3(Axis3, Axis1, Axis2);

			if (Order[0]>Order[2])
			{
				if ((Order[0] - Order[2]) == 1)
				{
				}
				else
				{
					s_nd(3, -1, Axis2, 1);
				}
			}
			else
			{
				if ((Order[0] - Order[2]) == -1)
				{
					s_nd(3, -1, Axis2, 1);
				}
				else
				{
				}
			}

			nrm = s_vnm(3, Axis2, 1);
			for (int i = 0; i < 3; ++i)
			{
				Axis2[i] = Axis2[i] / nrm;
			}

			std::fill_n(pm_out, 16, 0);

			pm_out[3] = origin[0];
			pm_out[7] = origin[1];
			pm_out[11] = origin[2];

			pm_out[0 + Order[0] - 1] = Axis1[0];
			pm_out[4 + Order[0] - 1] = Axis1[1];
			pm_out[8 + Order[0] - 1] = Axis1[2];

			pm_out[0 + Order[1] - 1] = Axis2[0];
			pm_out[4 + Order[1] - 1] = Axis2[1];
			pm_out[8 + Order[1] - 1] = Axis2[2];

			pm_out[0 + Order[2] - 1] = Axis3[0];
			pm_out[4 + Order[2] - 1] = Axis3[1];
			pm_out[8 + Order[2] - 1] = Axis3[2];

			pm_out[15] = 1;
		}
		auto s_sov_theta(double k1, double k2, double b, double *theta_out)noexcept->void
		{
			double K = std::sqrt(k1*k1 + k2*k2);
			double rhs = b / K;

			if (std::abs(rhs) < 0.7)
			{
				double alpha_plus_theta = std::asin(rhs);
				double alpha = std::atan2(k2, k1);
				theta_out[0] = alpha_plus_theta - alpha;
				theta_out[1] = PI - alpha_plus_theta - alpha;
			}
			else
			{
				double alpha_plus_theta = std::acos(rhs);
				double alpha = std::atan2(-k1, k2);
				theta_out[0] = alpha_plus_theta - alpha;
				theta_out[1] = -alpha_plus_theta - alpha;
			}

			if (theta_out[0] > 2 * PI)theta_out[0] -= 2 * PI;
			if (theta_out[1] > 2 * PI)theta_out[1] -= 2 * PI;
			if (theta_out[0] < -2 * PI)theta_out[0] += 2 * PI;
			if (theta_out[1] < -2 * PI)theta_out[1] += 2 * PI;
		};

		auto dlmwrite(const char *FileName, const double *pMatrix, const int m, const int n)->void
		{
			std::ofstream file;

			file.open(FileName);

			file << std::setprecision(15);

			for (int i = 0; i < m; i++)
			{
				for (int j = 0; j < n; j++)
				{
					file << pMatrix[n*i + j] << "   ";
				}
				file << std::endl;
			}
		}
		auto dlmread(const char *FileName, double *pMatrix)->void
		{
			std::ifstream file;

			file.open(FileName);

			if (!file) throw std::logic_error("file not exist");


			int i = 0;
			while (!file.eof())
			{
				file >> *(pMatrix + i);
				++i;
			}
		}
		auto dsp(const double *p, const int m, const int n, const int begin_row, const int begin_col, int ld)->void
		{
			if (ld < 1)	ld = n;

			std::cout << std::setiosflags(std::ios::fixed) << std::setiosflags(std::ios::right) << std::setprecision(15);

			std::cout << std::endl;
			for (int i = 0; i < m; i++)
			{
				for (int j = 0; j < n; j++)
				{
					std::cout << p[(begin_row + i)*ld + j + begin_col] << "   ";
				}
				std::cout << std::endl;
			}
			std::cout << std::endl;
		}

		auto s_is_equal(int n, const double *v1, const double *v2, double error, int ld_v1, int ld_v2) noexcept->bool
		{
			double diff_square = 0;

			int id_v1{ 0 }, id_v2{ 0 };
			for (int i = 0; i < n; ++i) 
			{
				diff_square += (v1[id_v1] - v2[id_v2])*(v1[id_v1] - v2[id_v2]);

				id_v1 += ld_v1;
				id_v2 += ld_v2;
			}

			diff_square = std::sqrt(std::abs(diff_square));

			return diff_square > error ? false : true;
		}
		auto s_is_equal(int m, int n, const double *m1, int ld_m1, const double *m2, int ld_m2, double error) noexcept->bool
		{
			double diff_square = 0;

			int row1{ 0 }, row2{ 0 };
			for (int i = 0; i < m; ++i)
			{
				for (int j = 0; j < n; ++j)
				{
					diff_square += (m1[row1 + j] - m2[row2 + j])*(m1[row1 + j] - m2[row2 + j]);
				}

				row1 += ld_m1;
				row2 += ld_m2;
			}

			diff_square = std::sqrt(std::abs(diff_square));

			return diff_square > error ? false : true;
		}
	}
}
