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
					std::cout << p[(begin_row+i)*ld + j+begin_col] << "   ";
				}
				std::cout << std::endl;
			}
			std::cout << std::endl;
		}
		
		auto s_is_equal(int n, const double *v1, const double *v2, double error) noexcept->bool
		{
			double diff_square = 0;

			for (int i = 0; i < n; ++i)diff_square += (v1[i] - v2[i])*(v1[i] - v2[i]);

			diff_square = std::sqrt(std::abs(diff_square));

			return diff_square > error ? false : true;
		}

		auto s_cro3(const double *cro_vec_in, const double *vec_in, double *vec_out) noexcept->void
		{
			vec_out[0] = -cro_vec_in[2] * vec_in[1] + cro_vec_in[1] * vec_in[2];
			vec_out[1] = cro_vec_in[2] * vec_in[0] - cro_vec_in[0] * vec_in[2];
			vec_out[2] = -cro_vec_in[1] * vec_in[0] + cro_vec_in[0] * vec_in[1];
		}
		auto s_cro3(double alpha, const double *cro_vec_in, const double *vec_in, double beta, double *vec_out) noexcept->void
		{
			vec_out[0] *= beta;
			vec_out[1] *= beta;
			vec_out[2] *= beta;

			vec_out[0] += alpha*(-cro_vec_in[2] * vec_in[1] + cro_vec_in[1] * vec_in[2]);
			vec_out[1] += alpha*(cro_vec_in[2] * vec_in[0] - cro_vec_in[0] * vec_in[2]);
			vec_out[2] += alpha*(-cro_vec_in[1] * vec_in[0] + cro_vec_in[0] * vec_in[1]);
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

		auto s_pe2pm(const double *pe_in, double *pm_out, const char *EurType) noexcept->void
		{
			static const double P[3][3] = { { 0, -1, 1 },{ 1, 0, -1 },{ -1, 1, 0 } };
			static const double Q[3][3] = { { 1, 0, 0 },{ 0, 1, 0 },{ 0, 0, 1 } };

			double Abb, Add, Abd, Adb;
			double Bac, Bae, Bdc, Bde;
			double Cbb, Cee, Cbe, Ceb;
			double s_, c_;

			const int a = EurType[0] - '1';
			const int b = EurType[1] - '1';
			const int c = EurType[2] - '1';
			const int d = 3 - a - b;
			const int e = 3 - b - c;

			c_ = std::cos(pe_in[3]);
			s_ = std::sin(pe_in[3]);
			Abb = c_;
			Add = Abb;
			Abd = P[b][d] * s_;
			Adb = -Abd;

			s_ = std::sin(pe_in[4]);
			c_ = std::cos(pe_in[4]);
			Bac = P[a][c] * s_ + Q[a][c] * c_;
			Bae = P[a][e] * s_ + Q[a][e] * c_;
			Bdc = P[d][c] * s_ + Q[d][c] * c_;
			Bde = P[d][e] * s_ + Q[d][e] * c_;

			c_ = std::cos(pe_in[5]);
			s_ = std::sin(pe_in[5]);
			Cbb = c_;
			Cee = Cbb;
			Cbe = P[b][e] * s_;
			Ceb = -Cbe;

			pm_out[a * 4 + c] = Bac;
			pm_out[a * 4 + b] = Bae * Ceb;
			pm_out[a * 4 + e] = Bae * Cee;
			pm_out[b * 4 + c] = Abd * Bdc;
			pm_out[b * 4 + b] = Abb * Cbb + Abd * Bde * Ceb;
			pm_out[b * 4 + e] = Abb * Cbe + Abd * Bde * Cee;
			pm_out[d * 4 + c] = Add * Bdc;
			pm_out[d * 4 + b] = Adb * Cbb + Add * Bde * Ceb;
			pm_out[d * 4 + e] = Adb * Cbe + Add * Bde * Cee;

			pm_out[3] = pe_in[0];
			pm_out[7] = pe_in[1];
			pm_out[11] = pe_in[2];

			pm_out[12] = 0;
			pm_out[13] = 0;
			pm_out[14] = 0;
			pm_out[15] = 1;
		}
		auto s_pm2pe(const double *pm_in, double *pe_out, const char *EurType) noexcept->void
		{
			static const double P[3][3] = { { 0, -1, 1 }, { 1, 0, -1 }, { -1, 1, 0 } };
			static const double Q[3][3] = { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } };

			double phi13, phi31;
			double phi[3];
			double s_, c_;

			const int a = EurType[0] - '1';
			const int b = EurType[1] - '1';
			const int c = EurType[2] - '1';
			const int d = 3 - a - b;
			const int e = 3 - b - c;

			// 计算phi2 //
			s_ = std::sqrt((pm_in[4 * a + b] * pm_in[4 * a + b] + pm_in[4 * a + e] * pm_in[4 * a + e]
				+ pm_in[4 * b + c] * pm_in[4 * b + c] + pm_in[4 * d + c] * pm_in[4 * d + c])/2);
				
			c_ = pm_in[4 * a + c];
			phi[1] = (a == c ? std::atan2(s_,c_) : std::atan2(P[a][c]*c_,s_));

			// 计算phi1和phi3 //
			phi13 = std::atan2(pm_in[4 * b + e] - pm_in[4 * d + b], pm_in[4 * b + b] + pm_in[4 * d + e]);
			phi31 = std::atan2(pm_in[4 * b + e] + pm_in[4 * d + b], pm_in[4 * b + b] - pm_in[4 * d + e]);

			phi[0] = P[b][d] * (phi13 - phi31) / 2;
			phi[2] = P[b][e] * (phi13 + phi31) / 2;

			// 检查 //
			double sig[4];
			sig[0] = (P[a][e] + Q[a][e])*P[e][b] * pm_in[a * 4 + b] * std::sin(phi[2]);
			sig[1] = (P[a][e] + Q[a][e])*pm_in[4 * a + e] * std::cos(phi[2]);
			sig[2] = (P[d][c] + Q[d][c])*P[b][d] * pm_in[b * 4 + c] * std::sin(phi[0]);
			sig[3] = (P[d][c] + Q[d][c])*pm_in[4 * d + c] * std::cos(phi[0]);
			
			if (*std::max_element(sig, sig + 4, [](double d1, double d2) {return (std::abs(d1) < std::abs(d2)); })<0)
			{
				phi[0] += PI;
				phi[2] += PI;
			}

			phi[0] = (phi[0] < 0 ? phi[0] + 2 * PI : phi[0]);
			phi[2] = (phi[2] < 0 ? phi[2] + 2 * PI : phi[2]);

			// 对位置赋值 //
			std::copy_n(phi, 3, pe_out + 3);
			
			pe_out[0] = pm_in[3];
			pe_out[1] = pm_in[7];
			pe_out[2] = pm_in[11];

		}
		auto s_pq2pm(const double *pq_in, double *pm_out) noexcept->void
		{
			const double &x = pq_in[0];
			const double &y = pq_in[1];
			const double &z = pq_in[2];
			const double &q1 = pq_in[3];
			const double &q2 = pq_in[4];
			const double &q3 = pq_in[5];
			const double &q4 = pq_in[6];

			pm_out[0] = 1 - 2 * q2 * q2 - 2 * q3 * q3;
			pm_out[1] = 2 * q1 * q2 - 2 * q4 * q3;
			pm_out[2] = 2 * q1 * q3 + 2 * q4 * q2;;
			pm_out[3] = x;

			pm_out[4] = 2 * q1 * q2 + 2 * q4 * q3;
			pm_out[5] = 1 - 2 * q1 * q1 - 2 * q3 * q3;
			pm_out[6] = 2 * q2 * q3 - 2 * q4 * q1;
			pm_out[7] = y;

			pm_out[8] = 2 * q1 * q3 - 2 * q4 * q2;
			pm_out[9] = 2 * q2 * q3 + 2 * q4 * q1;
			pm_out[10] = 1 - 2 * q1 * q1 - 2 * q2 * q2;
			pm_out[11] = z;

			pm_out[12] = 0;
			pm_out[13] = 0;
			pm_out[14] = 0;
			pm_out[15] = 1;
		}
		auto s_pm2pq(const double *pm_in, double *pq_out) noexcept->void
		{
			double &x = pq_out[0];
			double &y = pq_out[1];
			double &z = pq_out[2];
			double &q1 = pq_out[3];
			double &q2 = pq_out[4];
			double &q3 = pq_out[5];
			double &q4 = pq_out[6];

			double *q = &pq_out[3];

			double tr = pm_in[0] + pm_in[5] + pm_in[10];

			// 因为无法确保在截断误差的情况下，tr+1一定为正，因此这里需要判断 //
			q4 = tr>-1 ? std::sqrt((tr + 1) / 4) : 0;

			if (q4 > 0.1)
			{
				q1 = (pm_in[9] - pm_in[6]) / q4 / 4;
				q2 = (pm_in[2] - pm_in[8]) / q4 / 4;
				q3 = (pm_in[4] - pm_in[1]) / q4 / 4;
			}
			else
			{
				q1 = std::sqrt((1 + pm_in[0] - pm_in[5] - pm_in[10])) / 2;
				q2 = std::sqrt((1 + pm_in[5] - pm_in[0] - pm_in[10])) / 2;
				q3 = std::sqrt((1 + pm_in[10] - pm_in[0] - pm_in[5])) / 2;

				// qp_out 是通过开方计算而得，因此一定为正 //
				int id = std::max_element(q, q + 3) - q;

				q[id] *= s_sgn2(pm_in[(id + 2) % 3 * 4 + (id + 1) % 3] - pm_in[(id + 1) % 3 * 4 + (id + 2) % 3]);
				q[(id + 1) % 3] *= s_sgn2(q[id])*s_sgn2(pm_in[id * 4 + (id + 1) % 3] + pm_in[((id + 1) % 3) * 4 + id]);
				q[(id + 2) % 3] *= s_sgn2(q[id])*s_sgn2(pm_in[id * 4 + (id + 2) % 3] + pm_in[((id + 2) % 3) * 4 + id]);
			}

			x = pm_in[3];
			y = pm_in[7];
			z = pm_in[11];
		}
		auto s_pq2pe(const double *pq_in, double *pe_out, const char *EurType) noexcept->void
		{
			double pm[16];
			s_pq2pm(pq_in, pm);
			s_pm2pe(pm, pe_out, EurType);
		}
		auto s_pe2pq(const double *pe_in, double *pq_out, const char *EurType) noexcept->void
		{
			double pm[16];
			s_pe2pm(pe_in, pm, EurType);
			s_pm2pq(pm, pq_out);
		}
		auto s_pe2pe(const char* eur1_type_in, const double *pe_in, const char* eur2_type_in, double *pe_out) noexcept->void
		{
			double pm[16];
			s_pe2pm(pe_in, pm, eur1_type_in);
			s_pm2pe(pm, pe_out, eur2_type_in);
		}
		auto s_vq2v(const double *pq_in, const double *vq_in, double *vel_out) noexcept->void
		{
			const double *q = &pq_in[3];
			const double *vq = &vq_in[3];
			
			double p12 = q[0] * vq[1] + q[1] * vq[0];
			double p13 = q[0] * vq[2] + q[2] * vq[0];
			double p23 = q[1] * vq[2] + q[2] * vq[1];
			double p41 = q[3] * vq[0] + q[0] * vq[3];
			double p42 = q[3] * vq[1] + q[1] * vq[3];
			double p43 = q[3] * vq[2] + q[2] * vq[3];

			double pm[4][4];
			s_pq2pm(pq_in, *pm);

			vel_out[3] = 2 * (p13 - p42)*pm[1][0]                    + 2 * (p23 + p41)*pm[1][1]                   - 4 * (q[0] * vq[0] + q[1] * vq[1])*pm[1][2];
			vel_out[4] = -4 * (q[1] * vq[1] + q[2] * vq[2])*pm[2][0] + 2 * (p12 - p43)*pm[2][1]                   + 2 * (p13 + p42)*pm[2][2];
			vel_out[5] = 2 * (p12 + p43)*pm[0][0]                    - 4 * (q[0] * vq[0] + q[2] * vq[2])*pm[0][1] + 2 * (p23 - p41)*pm[0][2];
		
			//note only first 3 elements
			std::copy_n(vq_in, 3, vel_out);
			
		}
		auto s_v2vq(const double *pm_in, const double *vel_in, double *vq_out) noexcept->void
		{
			double pq[7];
			s_pm2pq(pm_in, pq);

			double *q = &pq[3];
			double *vq = &vq_out[3];

			double vpm[4][4];
			s_v_cro_pm(vel_in, pm_in, *vpm);

			int i = std::max_element(q, q + 4, [](double a, double b) {return std::abs(a) < std::abs(b); }) - q;

			if (i == 3)
			{
				vq[3] = (vpm[0][0] + vpm[1][1] + vpm[2][2]) / 8 / q[3];

				vq[0] = (vpm[2][1] - vpm[1][2] - 4 * q[0] * vq[3]) / 4 / q[3];
				vq[1] = (vpm[0][2] - vpm[2][0] - 4 * q[1] * vq[3]) / 4 / q[3];
				vq[2] = (vpm[1][0] - vpm[0][1] - 4 * q[2] * vq[3]) / 4 / q[3];
			}
			else
			{
				int j = (i + 1) % 3;
				int k = (i + 2) % 3;
				
				vq[i] = (vpm[i][i] - vpm[j][j] - vpm[k][k]) / 8 * q[i];
				vq[j] = (vpm[j][i] + vpm[i][j] - 4 * q[j] * vq[i]) / 4 * q[i];
				vq[k] = (vpm[k][i] + vpm[i][k] - 4 * q[k] * vq[i]) / 4 * q[i];
				vq[3] = (vpm[k][j] - vpm[j][k] - 4 * q[4] * vq[i]) / 4 * q[i];
			}

			//note that only first 3 elements
			std::copy_n(vel_in, 3, vq_out);
		}
		
		auto s_vp(const double *pnt_in, const double *vel_in, double *vp_out) noexcept->void
		{
			s_cro3(vel_in + 3, pnt_in, vp_out);
			s_daxpy(3, 1, vel_in, 1, vp_out, 1);
		}
		auto s_ap(const double *pnt_in, const double *vel_in, const double *acc_in, double *ap_out) noexcept->void
		{
			double tem1[3], tem2[3], tem3[3];
			//omega cross omega cross r
			s_cro3(vel_in + 3, pnt_in, tem2);
			s_cro3(vel_in + 3, tem2, tem1);

			s_cro3(vel_in + 3, vel_in, tem2);

			s_cro3(acc_in + 3, pnt_in, tem3);

			ap_out[0] = acc_in[0] + tem1[0] + tem2[0] + tem3[0];
			ap_out[1] = acc_in[1] + tem1[1] + tem2[1] + tem3[1];
			ap_out[2] = acc_in[2] + tem1[2] + tem2[2] + tem3[2];
		}

		auto s_f2f(const double *relative_pm_in, const double *from_fce_in, double *to_fce_out) noexcept->void
		{
			static const double default_pm_in[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
			static const double default_from_fce_in[6]{ 0,0,0,0,0,0 };
			double to_fce[6]{ 0,0,0,0,0,0 };

			relative_pm_in = relative_pm_in ? relative_pm_in : default_pm_in;
			from_fce_in = from_fce_in ? from_fce_in : default_from_fce_in;
			to_fce_out = to_fce_out ? to_fce_out : to_fce;

			s_tf(relative_pm_in, from_fce_in, to_fce_out);
		}
		auto s_v2v(const double *relative_pm_in, const double *relative_vel_in, const double *from_vel_in, double *to_vel_out) noexcept->void
		{
			static const double default_pm_in[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
			static const double default_vel_in[6]{ 0,0,0,0,0,0 };
			static const double default_from_vel_in[6]{ 0,0,0,0,0,0 };
			double to_vel[6]{ 0,0,0,0,0,0 };

			relative_pm_in = relative_pm_in ? relative_pm_in : default_pm_in;
			relative_vel_in = relative_vel_in ? relative_vel_in : default_vel_in;
			from_vel_in = from_vel_in ? from_vel_in : default_from_vel_in;
			to_vel_out = to_vel_out ? to_vel_out : to_vel;

			s_tv(relative_pm_in, from_vel_in, to_vel_out);
			s_vn_add_vn(6, relative_vel_in, to_vel_out, to_vel_out);
		}
		auto s_inv_v2v(const double *inv_relative_pm_in, const double *inv_relative_vel_in, const double *from_vel_in, double *to_vel_out) noexcept->void
		{
			static const double default_pm_in[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
			static const double default_vel_in[6]{ 0,0,0,0,0,0 };
			static const double default_from_vel_in[6]{ 0,0,0,0,0,0 };
			double to_vel[6]{ 0,0,0,0,0,0 };

			inv_relative_pm_in = inv_relative_pm_in ? inv_relative_pm_in : default_pm_in;
			inv_relative_vel_in = inv_relative_vel_in ? inv_relative_vel_in : default_vel_in;
			from_vel_in = from_vel_in ? from_vel_in : default_from_vel_in;
			to_vel_out = to_vel_out ? to_vel_out : to_vel;

			/*double relative_pm_in[16], relative_vel_in[6]{ 0 };
			s_inv_pm(inv_relative_pm_in, relative_pm_in);
			s_tv(-1, relative_pm_in, inv_relative_vel_in, 0, relative_vel_in);
			s_v2v(relative_pm_in, relative_vel_in, from_vel_in, to_vel_out);*/

			double tem[6];
			std::fill_n(to_vel_out, 6, 0);
			std::copy_n(from_vel_in, 6, tem);
			s_daxpy(6, -1, inv_relative_vel_in, 1, tem, 1);
			s_inv_tv(inv_relative_pm_in, tem, to_vel_out);
		}
		auto s_a2a(const double *relative_pm_in, const double *relative_vel_in, const double *relative_acc_in,
			const double *from_vel_in, const double *from_acc_in, double *to_acc_out, double *to_vel_out) noexcept->void
		{
			static const double default_pm_in[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
			static const double default_vel_in[6]{ 0,0,0,0,0,0 };
			static const double default_acc_in[6]{ 0,0,0,0,0,0 };
			static const double default_from_vel_in[6]{ 0,0,0,0,0,0 };
			static const double default_from_acc_in[6]{ 0,0,0,0,0,0 };
			double to_vel[6]{ 0,0,0,0,0,0 };
			double to_acc[6]{ 0,0,0,0,0,0 };

			relative_pm_in = relative_pm_in ? relative_pm_in : default_pm_in;
			relative_vel_in = relative_vel_in ? relative_vel_in : default_vel_in;
			relative_acc_in = relative_acc_in ? relative_acc_in : default_acc_in;
			from_vel_in = from_vel_in ? from_vel_in : default_from_vel_in;
			from_acc_in = from_acc_in ? from_acc_in : default_from_acc_in;
			to_vel_out = to_vel_out ? to_vel_out : to_vel;
			to_acc_out = to_acc_out ? to_acc_out : to_acc;

			s_tv(relative_pm_in, from_vel_in, to_vel_out);
			s_cv(relative_vel_in, to_vel_out, to_acc_out);
			s_tv(1, relative_pm_in, from_acc_in, 1, to_acc_out);
			s_daxpy(6, 1, relative_acc_in, 1, to_acc_out, 1);

		}
		auto s_inv_a2a(const double *inv_relative_pm_in, const double *inv_relative_vel_in, const double *inv_relative_acc_in,
			const double *from_vel_in, const double *from_acc_in, double *to_acc_out, double *to_vel_out) noexcept->void
		{
			static const double default_pm_in[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
			static const double default_vel_in[6]{ 0,0,0,0,0,0 };
			static const double default_acc_in[6]{ 0,0,0,0,0,0 };
			static const double default_from_vel_in[6]{ 0,0,0,0,0,0 };
			static const double default_from_acc_in[6]{ 0,0,0,0,0,0 };
			double to_vel[6]{ 0,0,0,0,0,0 };
			double to_acc[6]{ 0,0,0,0,0,0 };

			inv_relative_pm_in = inv_relative_pm_in ? inv_relative_pm_in : default_pm_in;
			inv_relative_vel_in = inv_relative_vel_in ? inv_relative_vel_in : default_vel_in;
			inv_relative_acc_in = inv_relative_acc_in ? inv_relative_acc_in : default_acc_in;
			from_vel_in = from_vel_in ? from_vel_in : default_from_vel_in;
			from_acc_in = from_acc_in ? from_acc_in : default_from_acc_in;
			to_vel_out = to_vel_out ? to_vel_out : to_vel;
			to_acc_out = to_acc_out ? to_acc_out : to_acc;

			
			s_inv_v2v(inv_relative_pm_in, inv_relative_vel_in, from_vel_in, to_vel_out);
			double tem[6];
			std::fill_n(to_acc_out, 6, 0);
			std::copy_n(from_acc_in, 6, tem);
			s_daxpy(6, -1, inv_relative_acc_in, 1, tem, 1);
			s_cv(-1, inv_relative_vel_in, from_vel_in, 1, tem);
			s_inv_tv(inv_relative_pm_in, tem, to_acc_out);
		}
		auto s_pp2pp(const double *relative_pm_in, const double *from_pnt, double *to_pnt_out) noexcept->void
		{
			static const double default_pm_in[16] = { 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
			static const double default_from_pnt[3] = { 0,0,0 };
			double default_to_pnt[3];

			relative_pm_in = relative_pm_in ? relative_pm_in : default_pm_in;
			from_pnt = from_pnt ? from_pnt : default_from_pnt;
			to_pnt_out = to_pnt_out ? to_pnt_out : default_to_pnt;

			s_pm_dot_pnt(relative_pm_in, from_pnt, to_pnt_out);
		}
		auto s_inv_pp2pp(const double *inv_relative_pm_in, const double *from_pnt, double *to_pnt_out) noexcept->void
		{
			static const double default_pm_in[16] = { 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
			static const double default_from_pnt[3] = { 0,0,0 };
			double default_to_pnt[3];

			inv_relative_pm_in = inv_relative_pm_in ? inv_relative_pm_in : default_pm_in;
			from_pnt = from_pnt ? from_pnt : default_from_pnt;
			to_pnt_out = to_pnt_out ? to_pnt_out : default_to_pnt;

			s_inv_pm_dot_pnt(inv_relative_pm_in, from_pnt, to_pnt_out);
		}
		auto s_vp2vp(const double *relative_pm_in, const double *relative_vel_in, const double *from_pnt, const double *from_vp,
			double *to_vp_out, double *to_pnt_out) noexcept->void
		{
			static const double default_pm_in[16] = { 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
			static const double default_vel_in[6] = { 0,0,0,0,0,0 };
			static const double default_from_pnt[3] = { 0,0,0 };
			static const double default_from_pv[3] = { 0,0,0 };
			double default_to_vp[3];
			double default_to_pnt[3];

			relative_pm_in = relative_pm_in ? relative_pm_in : default_pm_in;
			relative_vel_in = relative_vel_in ? relative_vel_in : default_vel_in;
			from_pnt = from_pnt ? from_pnt : default_from_pnt;
			from_vp = from_vp ? from_vp : default_from_pv;
			to_vp_out = to_vp_out ? to_vp_out : default_to_vp;
			to_pnt_out = to_pnt_out ? to_pnt_out : default_to_pnt;

			s_pp2pp(relative_pm_in, from_pnt, to_pnt_out);
			s_cro3(relative_vel_in + 3, to_pnt_out, to_vp_out);
			s_dgemm(3, 1, 3, 1, relative_pm_in, 4, from_vp, 1, 1, to_vp_out, 1);
			s_daxpy(3, 1, relative_vel_in, 1, to_vp_out, 1);
		}
		auto s_inv_vp2vp(const double *inv_relative_pm_in, const double *inv_relative_vel_in,
			const double *from_pnt, const double *from_vp, double *to_vp_out, double *to_pnt_out) noexcept->void
		{
			static const double default_pm_in[16] = { 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
			static const double default_vel_in[6] = { 0,0,0,0,0,0 };
			static const double default_from_pnt[3] = { 0,0,0 };
			static const double default_from_pv[3] = { 0,0,0 };
			double default_to_vp[3];
			double default_to_pnt[3];

			inv_relative_pm_in = inv_relative_pm_in ? inv_relative_pm_in : default_pm_in;
			inv_relative_vel_in = inv_relative_vel_in ? inv_relative_vel_in : default_vel_in;
			from_pnt = from_pnt ? from_pnt : default_from_pnt;
			from_vp = from_vp ? from_vp : default_from_pv;
			to_vp_out = to_vp_out ? to_vp_out : default_to_vp;
			to_pnt_out = to_pnt_out ? to_pnt_out : default_to_pnt;

			std::fill_n(to_vp_out, 3, 0);

			double tem[3];
			std::copy_n(from_vp, 3, tem);
			s_cro3(-1, inv_relative_vel_in + 3, from_pnt, 1, tem);
			s_daxpy(3, -1, inv_relative_vel_in, 1, tem, 1);
			s_dgemmTN(3, 1, 3, 1, inv_relative_pm_in, 4, tem, 1, 0, to_vp_out, 1);

			s_inv_pm_dot_pnt(inv_relative_pm_in, from_pnt, to_pnt_out);

		}
		auto s_ap2ap(const double *relative_pm_in, const double *relative_vel_in, const double *relative_acc_in,
			const double *from_pnt, const double *from_vp, const double *from_pa,
			double *to_ap_out, double *to_vp_out, double *to_pnt_out) noexcept->void
		{
			static const double default_pm_in[16] = { 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
			static const double default_vel_in[6] = { 0,0,0,0,0,0 };
			static const double default_acc_in[6] = { 0,0,0,0,0,0 };
			static const double default_from_pnt[3] = { 0,0,0 };
			static const double default_from_pv[3] = { 0,0,0 };
			static const double default_from_pa[3] = { 0,0,0 };
			double default_to_ap[3]{ 0 };
			double default_to_vp[3]{ 0 };
			double default_to_pnt[3]{ 0 };

			relative_pm_in = relative_pm_in ? relative_pm_in : default_pm_in;
			relative_vel_in = relative_vel_in ? relative_vel_in : default_vel_in;
			relative_acc_in = relative_acc_in ? relative_acc_in : default_acc_in;
			from_pnt = from_pnt ? from_pnt : default_from_pnt;
			from_vp = from_vp ? from_vp : default_from_pv;
			from_pa = from_pa ? from_pa : default_from_pa;
			to_ap_out = to_ap_out ? to_ap_out : default_to_ap;
			to_vp_out = to_vp_out ? to_vp_out : default_to_vp;
			to_pnt_out = to_pnt_out ? to_pnt_out : default_to_pnt;

			double tem_vp[3];
			s_vp2vp(relative_pm_in, relative_vel_in, from_pnt, from_vp, to_vp_out, to_pnt_out);

			s_cro3(relative_acc_in + 3, to_pnt_out, to_ap_out);
			std::copy_n(to_vp_out, 3, tem_vp);
			s_dgemm(3, 1, 3, 1, relative_pm_in, 4, from_vp, 1, 1, tem_vp, 1);
			s_cro3(1, relative_vel_in + 3, tem_vp, 1, to_ap_out);
			s_dgemm(3, 1, 3, 1, relative_pm_in, 4, from_pa, 1, 1, to_ap_out, 1);
			s_daxpy(3, 1, relative_acc_in, 1, to_ap_out, 1);
		}
		auto s_inv_ap2ap(const double *inv_relative_pm_in, const double *inv_relative_vel_in, const double *inv_relative_acc_in,
			const double *from_pnt, const double *from_vp, const double *from_pa,
			double *to_ap_out, double *to_vp_out, double *to_pnt_out) noexcept->void
		{
			static const double default_pm_in[16] = { 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
			static const double default_vel_in[6] = { 0,0,0,0,0,0 };
			static const double default_acc_in[6] = { 0,0,0,0,0,0 };
			static const double default_from_pnt[3] = { 0,0,0 };
			static const double default_from_pv[3] = { 0,0,0 };
			static const double default_from_pa[3] = { 0,0,0 };
			double default_to_ap[3]{ 0 };
			double default_to_vp[3]{ 0 };
			double default_to_pnt[3]{ 0 };

			inv_relative_pm_in = inv_relative_pm_in ? inv_relative_pm_in : default_pm_in;
			inv_relative_vel_in = inv_relative_vel_in ? inv_relative_vel_in : default_vel_in;
			inv_relative_acc_in = inv_relative_acc_in ? inv_relative_acc_in : default_acc_in;
			from_pnt = from_pnt ? from_pnt : default_from_pnt;
			from_vp = from_vp ? from_vp : default_from_pv;
			from_pa = from_pa ? from_pa : default_from_pa;
			to_ap_out = to_ap_out ? to_ap_out : default_to_ap;
			to_vp_out = to_vp_out ? to_vp_out : default_to_vp;
			to_pnt_out = to_pnt_out ? to_pnt_out : default_to_pnt;

			std::fill_n(to_ap_out, 3, 0);

			double tem[3], tem2[3];
			s_inv_vp2vp(inv_relative_pm_in, inv_relative_vel_in, from_pnt, from_vp, to_vp_out, to_pnt_out);

			std::copy_n(from_pa, 3, tem);
			s_cro3(-1, inv_relative_acc_in + 3, from_pnt, 1, tem);

			std::copy_n(from_vp, 3, tem2);
			s_dgemm(3, 1, 3, 1, inv_relative_pm_in, 4, to_vp_out, 1, 1, tem2, 1);
			s_cro3(-1, inv_relative_vel_in + 3, tem2, 1, tem);

			s_daxpy(3, -1, inv_relative_acc_in, 1, tem, 1);

			s_dgemmTN(3, 1, 3, 1, inv_relative_pm_in, 4, tem, 1, 0, to_ap_out, 1);
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
			
			s_dgemm(3, n, 3, 1, pm_in, 4, fces_in, n, 0, m_out, n);
			s_dgemm(3, n, 3, 1, pm_in, 4, fces_in + 3 * n, n, 0, m_out + 3 * n, n);

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

			s_dgemm(3, n, 3, alpha, pm_in, 4, fces_in, n, beta, m_out, n);
			s_dgemm(3, n, 3, alpha, pm_in, 4, fces_in + 3 * n, n, beta, m_out + 3 * n, n);
			s_dgemm(3, n, 3, alpha, *vRm, 3, fces_in, n, 1, m_out + 3 * n, n);
		}
		auto s_inv_tf(const double *inv_pm_in, const double *fce_in, double *vec_out) noexcept->void
		{
			double pm_in[16];
			s_inv_pm(inv_pm_in, pm_in);
			s_tf(pm_in, fce_in, vec_out);
		}
		auto s_inv_tf(double alpha, const double *inv_pm_in, const double *vel_in, double beta, double *vec_out) noexcept->void
		{
			double pm_in[16];
			s_inv_pm(inv_pm_in, pm_in);
			s_tf(alpha, pm_in, vel_in, beta, vec_out);
		}
		auto s_tv(const double *pm_in, const double *vel_in, double *vec_out) noexcept->void
		{
			s_pm_dot_v3(pm_in, vel_in, vec_out);
			s_pm_dot_v3(pm_in, vel_in + 3, vec_out + 3);

			vec_out[0] += -pm_in[11] * vec_out[4] + pm_in[7] * vec_out[5];
			vec_out[1] += pm_in[11] * vec_out[3] - pm_in[3] * vec_out[5];
			vec_out[2] += -pm_in[7] * vec_out[3] + pm_in[3] * vec_out[4];
		}
		auto s_tv(double alpha, const double *pm_in, const double *vel_in, double beta, double *vec_out) noexcept->void
		{
			double tem[6];
			
			s_tv(pm_in, vel_in, tem);

			for (int i = 0; i < 6; ++i)
			{
				vec_out[i] = alpha * tem[i] + beta * vec_out[i];
			}
		}
		auto s_tv_n(int n, const double *pm_in, const double *vels_in, double *m_out) noexcept->void
		{
			std::fill_n(m_out, 6 * n, 0);
			
			s_dgemm(3, n, 3, 1, pm_in, 4, vels_in, n, 0, m_out, n);
			s_dgemm(3, n, 3, 1, pm_in, 4, vels_in + 3 * n, n, 0, m_out + 3 * n, n);

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

			s_dgemm(3, n, 3, alpha, pm_in, 4, vels_in, n, beta, m_out, n);
			s_dgemm(3, n, 3, alpha, pm_in, 4, vels_in + 3 * n, n, beta, m_out + 3 * n, n);
			s_dgemm(3, n, 3, alpha, *vRm, 3, vels_in + 3 * n, n, 1, m_out, n);
		}
		auto s_inv_tv(const double *inv_pm_in, const double *vel_in, double *vec_out) noexcept->void
		{
			double pm_in[16];
			s_inv_pm(inv_pm_in, pm_in);
			s_tv(pm_in, vel_in, vec_out);
		}
		auto s_inv_tv(double alpha, const double *inv_pm_in, const double *vel_in, double beta, double *vec_out) noexcept->void
		{
			double pm_in[16];
			s_inv_pm(inv_pm_in, pm_in);
			s_tv(alpha, pm_in, vel_in, beta, vec_out);
		}
		auto s_inv_tv_n(int n, const double *inv_pm_in, const double *vel_in, double *vec_out) noexcept->void
		{
			double pm_in[16];
			s_inv_pm(inv_pm_in, pm_in);
			s_tv_n(n, pm_in, vel_in, vec_out);
		}
		auto s_inv_tv_n(int n, double alpha, const double *inv_pm_in, const double *vel_in, double beta, double *vec_out) noexcept->void
		{
			double pm_in[16];
			s_inv_pm(inv_pm_in, pm_in);
			s_tv_n(n, alpha, pm_in, vel_in, beta, vec_out);
		}
		auto s_cmf(const double *vel_in, double *cmf_out) noexcept->void
		{
			std::fill_n(cmf_out, 36, 0);

			cmf_out[6] = vel_in[5];
			cmf_out[12] = -vel_in[4];
			cmf_out[1] = -vel_in[5];
			cmf_out[13] = vel_in[3];
			cmf_out[2] = vel_in[4];
			cmf_out[8] = -vel_in[3];

			cmf_out[27] = vel_in[5];
			cmf_out[33] = -vel_in[4];
			cmf_out[22] = -vel_in[5];
			cmf_out[34] = vel_in[3];
			cmf_out[23] = vel_in[4];
			cmf_out[29] = -vel_in[3];

			cmf_out[24] = vel_in[2];
			cmf_out[30] = -vel_in[1];
			cmf_out[19] = -vel_in[2];
			cmf_out[31] = vel_in[0];
			cmf_out[20] = vel_in[1];
			cmf_out[26] = -vel_in[0];
		}
		auto s_cmv(const double *vel_in, double *cmv_out) noexcept->void
		{
			std::fill_n(cmv_out, 36, 0);

			cmv_out[6] = vel_in[5];
			cmv_out[12] = -vel_in[4];
			cmv_out[1] = -vel_in[5];
			cmv_out[13] = vel_in[3];
			cmv_out[2] = vel_in[4];
			cmv_out[8] = -vel_in[3];

			cmv_out[27] = vel_in[5];
			cmv_out[33] = -vel_in[4];
			cmv_out[22] = -vel_in[5];
			cmv_out[34] = vel_in[3];
			cmv_out[23] = vel_in[4];
			cmv_out[29] = -vel_in[3];

			cmv_out[9] = vel_in[2];
			cmv_out[15] = -vel_in[1];
			cmv_out[4] = -vel_in[2];
			cmv_out[16] = vel_in[0];
			cmv_out[5] = vel_in[1];
			cmv_out[11] = -vel_in[0];
		}
		auto s_cf(const double *cro_vel_in, const double *vec_in, double* vec_out) noexcept->void
		{
			s_cro3(cro_vel_in + 3, vec_in, vec_out);
			s_cro3(cro_vel_in + 3, vec_in + 3, vec_out + 3);

			vec_out[3] += -cro_vel_in[2] * vec_in[1] + cro_vel_in[1] * vec_in[2];
			vec_out[4] += cro_vel_in[2] * vec_in[0] - cro_vel_in[0] * vec_in[2];
			vec_out[5] += -cro_vel_in[1] * vec_in[0] + cro_vel_in[0] * vec_in[1];
		}
		auto s_cf(double alpha, const double *cro_vel_in, const double *vec_in, double beta, double* vec_out) noexcept->void
		{
			s_cro3(alpha, cro_vel_in + 3, vec_in, beta, vec_out);
			s_cro3(alpha, cro_vel_in + 3, vec_in + 3, beta, vec_out + 3);

			vec_out[3] += alpha*(-cro_vel_in[2] * vec_in[1] + cro_vel_in[1] * vec_in[2]);
			vec_out[4] += alpha*(cro_vel_in[2] * vec_in[0] - cro_vel_in[0] * vec_in[2]);
			vec_out[5] += alpha*(-cro_vel_in[1] * vec_in[0] + cro_vel_in[0] * vec_in[1]);
		}
		auto s_cv(const double *cro_vel_in, const double *vec_in, double* vec_out) noexcept->void
		{
			s_cro3(cro_vel_in + 3, vec_in, vec_out);
			s_cro3(cro_vel_in + 3, vec_in+3, vec_out+3);
			
			vec_out[0] += -cro_vel_in[2] * vec_in[4] + cro_vel_in[1] * vec_in[5];
			vec_out[1] += cro_vel_in[2] * vec_in[3] - cro_vel_in[0] * vec_in[5];
			vec_out[2] += -cro_vel_in[1] * vec_in[3] + cro_vel_in[0] * vec_in[4];
		}
		auto s_cv(double alpha, const double *cro_vel_in, const double *vec_in, double beta, double* vec_out) noexcept->void
		{
			s_cro3(alpha, cro_vel_in + 3, vec_in, beta, vec_out);
			s_cro3(alpha, cro_vel_in + 3, vec_in + 3, beta, vec_out + 3);

			vec_out[0] += alpha*(-cro_vel_in[2] * vec_in[4] + cro_vel_in[1] * vec_in[5]);
			vec_out[1] += alpha*(cro_vel_in[2] * vec_in[3] - cro_vel_in[0] * vec_in[5]);
			vec_out[2] += alpha*(-cro_vel_in[1] * vec_in[3] + cro_vel_in[0] * vec_in[4]);
		}
		auto s_i2i(const double *from_pm_in, const double *from_im_in, double *to_im_out) noexcept->void
		{
			//double x, y, z, old_x, old_y, old_z, new_x, new_y, new_z, m;

			//m = from_im_in[0];

			//x = from_pm_in[3];
			//y = from_pm_in[7];
			//z = from_pm_in[11];

			//old_x = from_im_in[11];
			//old_y = from_im_in[15];
			//old_z = from_im_in[4];

			//new_x = from_pm_in[0] * old_x + from_pm_in[1] * old_y + from_pm_in[2] * old_z;
			//new_y = from_pm_in[4] * old_x + from_pm_in[5] * old_y + from_pm_in[6] * old_z;
			//new_z = from_pm_in[8] * old_x + from_pm_in[9] * old_y + from_pm_in[10] * old_z;

			////memset(to_im_out, 0, sizeof(double)* 36);
			//std::fill_n(to_im_out, 36, 0);
			///* 设置左上角的3*3的矩阵 */
			//to_im_out[0] = from_im_in[0];
			//to_im_out[7] = from_im_in[7];
			//to_im_out[14] = from_im_in[14];
			///* 设置右上角的3*3的矩阵 */
			//to_im_out[4] = m*z + new_z;
			//to_im_out[5] = -m*y - new_y;
			//to_im_out[11] = m*x + new_x;
			//to_im_out[9] = -to_im_out[4];
			//to_im_out[15] = -to_im_out[5];
			//to_im_out[16] = -to_im_out[11];
			///* 设置左下角的3*3的矩阵 */
			//to_im_out[24] = to_im_out[4];
			//to_im_out[30] = to_im_out[5];
			//to_im_out[19] = to_im_out[9];
			//to_im_out[31] = to_im_out[11];
			//to_im_out[20] = to_im_out[15];
			//to_im_out[26] = to_im_out[16];
			///* 设置右下角的3*3的矩阵 */
			//for (int i = 0; i < 3; ++i)
			//{
			//	for (int j = 0; j < 3; ++j)
			//	{
			//		to_im_out[21] += from_pm_in[0 + i] * from_im_in[i * 6 + j + 21] * from_pm_in[j + 0];
			//		to_im_out[22] += from_pm_in[0 + i] * from_im_in[i * 6 + j + 21] * from_pm_in[j + 4];
			//		to_im_out[23] += from_pm_in[0 + i] * from_im_in[i * 6 + j + 21] * from_pm_in[j + 8];
			//		to_im_out[28] += from_pm_in[4 + i] * from_im_in[i * 6 + j + 21] * from_pm_in[j + 4];
			//		to_im_out[29] += from_pm_in[4 + i] * from_im_in[i * 6 + j + 21] * from_pm_in[j + 8];
			//		to_im_out[35] += from_pm_in[8 + i] * from_im_in[i * 6 + j + 21] * from_pm_in[j + 8];
			//	}
			//}

			//to_im_out[21] += m*(y*y + z*z) + 2 * (new_y*y + new_z*z);
			//to_im_out[22] += -m*x*y - new_x*y - x*new_y;
			//to_im_out[23] += -m*x*z - new_x*z - x*new_z;
			//to_im_out[28] += m*(x*x + z*z) + 2 * (new_x*x + new_z*z);
			//to_im_out[29] += -m*y*z - new_y*z - y*new_z;
			//to_im_out[35] += m*(x*x + y*y) + 2 * (new_x*x + new_y*y);

			//to_im_out[27] = to_im_out[22];
			//to_im_out[33] = to_im_out[23];
			//to_im_out[34] = to_im_out[29];





			/*以下为慢速但准确的算法*/
			std::fill_n(to_im_out, 36, 0);
			double tem[6][6]{ {0} }, tmf[6][6]{ {0} };
			s_tmf(from_pm_in, *tmf);
			s_dgemm(6, 6, 6, 1, *tmf, 6, from_im_in, 6, 0, *tem, 6);
			s_dgemmNT(6, 6, 6, 1, *tem, 6, *tmf, 6, 0, to_im_out, 6);

		}
		
		auto s_mass2im(const double mass_in, const double * inertia_in, const double *pm_in, double *im_out) noexcept->void
		{
			double loc_im[6][6]{ {0} }, loc_tm[6][6];
		
			//memset(im_out, 0, sizeof(double)* 36);
			std::fill_n(im_out, 36, 0);

			im_out[0] = mass_in;
			im_out[7] = mass_in;
			im_out[14] = mass_in;

			if (inertia_in != nullptr)
			{
				im_out[21] = inertia_in[0];
				im_out[22] = inertia_in[1];
				im_out[23] = inertia_in[2];
				im_out[27] = inertia_in[3];
				im_out[28] = inertia_in[4];
				im_out[29] = inertia_in[5];
				im_out[33] = inertia_in[6];
				im_out[34] = inertia_in[7];
				im_out[35] = inertia_in[8];
			}

			if (pm_in != nullptr)
			{
				s_tmf(pm_in, *loc_tm);
				s_dgemm(6, 6, 6, 1, *loc_tm, 6, im_out, 6, 0, *loc_im, 6);
				s_dgemm(6, 6, 6, 1, *loc_im, 6, *loc_tm, 6, 0, im_out, 6);
			}
		
		}
		auto s_gamma2im(const double * gamma_in, double *im_out) noexcept->void
		{
			//memset(im_out, 0, sizeof(double)* 36);
			std::fill_n(im_out, 36, 0);

			im_out[0] = gamma_in[0];
			im_out[7] = gamma_in[0];
			im_out[14] = gamma_in[0];

			im_out[4] = gamma_in[3];
			im_out[5] = -gamma_in[2];
			im_out[9] = -gamma_in[3];
			im_out[11] = gamma_in[1];
			im_out[15] = gamma_in[2];
			im_out[16] = -gamma_in[1];

			im_out[19] = -gamma_in[3];
			im_out[20] = gamma_in[2];
			im_out[24] = gamma_in[3];
			im_out[26] = -gamma_in[1];
			im_out[30] = -gamma_in[2];
			im_out[31] = gamma_in[1];

			im_out[21] = gamma_in[4];
			im_out[22] = gamma_in[7];
			im_out[23] = gamma_in[8];
			im_out[27] = gamma_in[7];
			im_out[28] = gamma_in[5];
			im_out[29] = gamma_in[9];
			im_out[33] = gamma_in[8];
			im_out[34] = gamma_in[9];
			im_out[35] = gamma_in[6];
		}
		auto s_im2gamma(const double * im_in, double *gamma_out) noexcept->void
		{
			gamma_out[0] = im_in[0];
			gamma_out[1] = im_in[11];
			gamma_out[2] = im_in[15];
			gamma_out[3] = im_in[4];
			gamma_out[4] = im_in[21];
			gamma_out[5] = im_in[28];
			gamma_out[6] = im_in[35];
			gamma_out[7] = im_in[22];
			gamma_out[8] = im_in[23];
			gamma_out[9] = im_in[29];
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
		auto s_dlt_col(const int &dlt_col_num,const int *col_index, const int &m, const int &n, double *A, const int &ldA) noexcept->void
		{
			for (int i = 0; i < dlt_col_num; ++i)
			{
				for (int k = col_index[i]; k < ((i == dlt_col_num - 1) ? (n) : (col_index[i+1])); ++k)
				{
					for (int j = 0; j < m; ++j)
					{
						A[j*ldA + k-i] = A[j*ldA + k + 1];
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
					pm_out[i * 4 + j] = inv_pm1_in[i] * pm2_in[j] + inv_pm1_in[i + 4] * pm2_in[j + 4] + inv_pm1_in[i +8] * pm2_in[j + 8];
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
		auto s_pm_dot_pnt(const double *pm_in, const double *pos_in, double *pos_out) noexcept->void
		{
			s_pm_dot_v3(pm_in, pos_in, pos_out);
			
			pos_out[0] += pm_in[3];
			pos_out[1] += pm_in[7];
			pos_out[2] += pm_in[11];
		}
		auto s_inv_pm_dot_pnt(const double *pm_in, const double *pos_in, double *pos_out) noexcept->void
		{
			std::fill_n(pos_out, 3, 0);


			double tem[3];
			std::copy_n(pos_in, 3, tem);

			s_daxpy(3, -1, &pm_in[3], 4, tem, 1);
			s_dgemmTN(3, 1, 3, 1, pm_in, 4, tem, 1, 0, pos_out, 1);
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
		auto s_vn_add_vn(int N, const double *v1_in, const double *v2_in, double *v_out) noexcept->void
		{
			for (int i = 0; i < N; ++i)
			{
				v_out[i] = v1_in[i] + v2_in[i];
			}
		}
		auto s_vn_dot_vn(int N, const double *v1_in, const double *v2_in) noexcept->double
		{
			double ret{0};

			for (int i = 0; i < N; ++i)
			{
				ret += v1_in[i] * v2_in[i];
			}

			return ret;
		}
		auto s_v_cro_pm(const double *v_in, const double *pm_in, double *vpm_out) noexcept->void
		{
			vpm_out[0] = -v_in[5] * pm_in[4] + v_in[4] * pm_in[8];
			vpm_out[4] = v_in[5] * pm_in[0] - v_in[3] * pm_in[8];
			vpm_out[8] = -v_in[4] * pm_in[0] + v_in[3] * pm_in[4];

			vpm_out[1] = -v_in[5] * pm_in[5] + v_in[4] * pm_in[9];
			vpm_out[5] = v_in[5] * pm_in[1] - v_in[3] * pm_in[9];
			vpm_out[9] = -v_in[4] * pm_in[1] + v_in[3] * pm_in[5];

			vpm_out[2] = -v_in[5] * pm_in[6] + v_in[4] * pm_in[10];
			vpm_out[6] = v_in[5] * pm_in[2] - v_in[3] * pm_in[10];
			vpm_out[10] = -v_in[4] * pm_in[2] + v_in[3] * pm_in[6];

			vpm_out[3] = -v_in[5] * pm_in[7] + v_in[4] * pm_in[11] + v_in[0];
			vpm_out[7] = v_in[5] * pm_in[3] - v_in[3] * pm_in[11] + v_in[1];
			vpm_out[11] = -v_in[4] * pm_in[3] + v_in[3] * pm_in[7] + v_in[2];

			vpm_out[12] = 0;
			vpm_out[13] = 0;
			vpm_out[14] = 0;
			vpm_out[15] = 0;
		}

		auto s_dscal(const int n, const double a, double *x, const int incx) noexcept->void
		{
			for (int i = 0; i < n*incx; i += incx)x[i] *= a;
		}
		auto s_dnrm2(const int n, const double *x, const int incx) noexcept->double
		{
			double nrm=0;

			int finalIdx = n*incx;

			for (int i = 0; i < finalIdx; i+=incx)
			{
				nrm += x[i] * x[i];
			}
			return std::sqrt(nrm);
		}
		auto s_daxpy(const int N, const double alpha, const double *X, const int incX, double *Y, const int incY) noexcept->void
		{
			int xIdx{ 0 }, yIdx{ 0 };
			
			for (int i = 0; i < N; ++i)
			{
				Y[yIdx] += alpha*X[xIdx];
				xIdx += incX;
				yIdx += incY;
			}
		}
		auto s_swap(const int N, double *X, const int incX, double *Y, const int incY) noexcept->void
		{
			for (int i = 0; i < N; ++i)std::swap(X[incX*i], Y[incY*i]);
		}
		auto s_transpose(const int m, const int n, const double *A, const int ldA, double *B_out, const int ldB) noexcept->void
		{
			for (int i = 0; i < m; ++i)
			{
				for (int j = 0; j < n; ++j)
				{
					B_out[j*ldB + i] = A[i*ldA + j];
				}
			}
		}

		auto s_dgemm(int m, int n, int k, double alpha, const double* A, int lda, const double* B, int ldb, double beta, double *C, int ldc) noexcept->void
		{
			for (int i = 0; i < m; ++i)
			{
				int rowIndex = i*lda;
				for (int j = 0; j < n; ++j)
				{
					int idx = i*ldc + j;
					C[idx] *= beta;

					double addFactor = 0;
					for (int u = 0; u < k; ++u)
					{
						addFactor +=  A[rowIndex + u] * B[j + u*ldb];
					}
					
					C[idx] += alpha *addFactor;
				}
			}
		}
		auto s_dgemmTN(int m, int n, int k, double alpha, const double* A, int lda, const double* B, int ldb, double beta, double *C, int ldc) noexcept->void
		{
			for (int i = 0; i < m; ++i)
			{
				for (int j = 0; j < n; ++j)
				{
					int idx = i*ldc + j;
					C[idx] *= beta;

					double addFactor = 0;
					for (int u = 0; u < k; ++u)
					{
						addFactor += A[i + u*lda] * B[j + u*ldb];
					}

					C[idx] += alpha *addFactor;
				}
			}
		}
		auto s_dgemmNT(int m, int n, int k, double alpha, const double* A, int lda, const double* B, int ldb, double beta, double *C, int ldc) noexcept->void
		{
			for (int i = 0; i < m; ++i)
			{
				int rowIndex = i*lda;
				for (int j = 0; j < n; ++j)
				{
					int colIndex = j*ldb;
					
					int idx = i*ldc + j;
					C[idx] *= beta;

					double addFactor = 0;
					for (int u = 0; u < k; ++u)
					{
						addFactor += A[rowIndex + u] * B[colIndex + u];
					}

					C[idx] += alpha *addFactor;
				}
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

			s_daxpy(3, -1, origin, 1, Axis1, 1);
			s_daxpy(3, -1, origin, 1, Axis2, 1);

			nrm = s_dnrm2(3, Axis1, 1);
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

			nrm = s_dnrm2(3, Axis2, 1);
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
			s_cro3(Axis1, Axis2, Axis3);

			nrm = s_dnrm2(3, Axis3, 1);
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
					s_dscal(3, -1, Axis3, 1);
				}
			}
			else
			{
				if ((Order[1] - Order[0]) == -1)
				{
					s_dscal(3, -1, Axis3, 1);
				}
				else
				{
				}
			}

			s_cro3(Axis3, Axis1, Axis2);

			if (Order[0]>Order[2])
			{
				if ((Order[0] - Order[2]) == 1)
				{
				}
				else
				{
					s_dscal(3, -1, Axis2, 1);
				}
			}
			else
			{
				if ((Order[0] - Order[2]) == -1)
				{
					s_dscal(3, -1, Axis2, 1);
				}
				else
				{
				}
			}

			nrm = s_dnrm2(3, Axis2, 1);
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
	}
}
