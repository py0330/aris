#ifdef PLATFORM_IS_WINDOWS
	#define HAVE_LAPACK_CONFIG_H
	#define LAPACK_COMPLEX_STRUCTURE
#endif

#include <cmath>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <algorithm>
#include <limits>
#include <sstream>
using namespace std;

#include <Platform.h>
#include "Aris_DynKer.h"
#include "Aris_ExpCal.h"

extern "C"
{
#include <cblas.h>
}
#include <lapacke.h>

namespace Aris
{
	namespace DynKer
	{

		void s_dgeev(int n,	double* a, int lda, double* wr, double* wi)
		{
			LAPACKE_dgeev(CblasRowMajor, 'N', 'N', n, a, lda, wr, wi, nullptr, lda, nullptr, lda);
		}
		
		class S_MALLOC
		{
		private:
			void *pMem;
			unsigned int currentSize;
		public:
			S_MALLOC()
			{
				pMem = malloc(10000000);
				currentSize = 10000000;
			}
			~S_MALLOC()
			{
				free(pMem);
			};

			void* operator()(unsigned int size)
			{
				if (size > currentSize)
				{
					free(pMem);
					pMem = malloc(size);
					currentSize = size;
				}

				return pMem;
			}


		};
		S_MALLOC s_malloc;
		
		void dlmwrite(const char *FileName, const double *pMatrix, const unsigned int m, const unsigned int n)
		{
			ofstream file;

			file.open(FileName);

			file << setprecision(15);
			
			for (unsigned int i = 0; i < m; i++)
			{
				for (unsigned int j = 0; j < n; j++)
				{
					file << pMatrix[n*i + j] << "   ";
				}
				file << endl;
			}
		}
		void dlmread(const char *FileName, double *pMatrix)
		{
			ifstream file;

			file.open(FileName);

			if (!file)
				throw std::logic_error("file not exist");


			int i = 0;
			while (!file.eof())
			{        
				file >> *(pMatrix + i);
				++i;
			}
		}
		void dsp(const double *p, const int m, const int n, const int begin_row, const int begin_col, int ld)
		{
			if (ld < 1)
				ld = n;
		
			cout << std::setiosflags(ios::fixed) << setiosflags(ios::right) << setprecision(5);

			cout << endl;
			for (int i = 0; i < m; i++)
			{
				for (int j = 0; j < n; j++)
				{
					cout << p[(begin_row+i)*ld + j+begin_col] << "   ";
				}
				cout << endl;
			}
			cout << endl;
		}
	
		int s_sgn(double x)
		{
			if (x > 0)
			{
				return 1;
			}
			else if (x == 0)
			{
				return 0;
			}
			else
			{
				return -1;
			}
		}
		void s_cm3(const double *cro_vec_in, double *cm_out)
		{
			memset(cm_out, 0, sizeof(double)* 9);
			
			cm_out[1] = -cro_vec_in[2];
			cm_out[2] = cro_vec_in[1];
			cm_out[3] = cro_vec_in[2];
			cm_out[5] = -cro_vec_in[0];
			cm_out[6] = -cro_vec_in[1];
			cm_out[7] = cro_vec_in[0];
		}
		void s_cro3(const double *cro_vec_in, const double *vec_in, double *vec_out)
		{
			vec_out[0] = -cro_vec_in[2] * vec_in[1] + cro_vec_in[1] * vec_in[2];
			vec_out[1] = cro_vec_in[2] * vec_in[0] - cro_vec_in[0] * vec_in[2];
			vec_out[2] = -cro_vec_in[1] * vec_in[0] + cro_vec_in[0] * vec_in[1];
		}
		void s_inv_pm(const double *pm_in, double *pm_out)
		{
			//转置
			cblas_dcopy(16, pm_in, 1, pm_out, 1);
			pm_out[1] = pm_in[4];
			pm_out[2] = pm_in[8];
			pm_out[6] = pm_in[9];
			pm_out[4] = pm_in[1];
			pm_out[8] = pm_in[2];
			pm_out[9] = pm_in[6];
			//位置
			pm_out[3] = -pm_out[0] * pm_in[3] - pm_out[1] * pm_in[7] - pm_out[2] * pm_in[11];
			pm_out[7] = -pm_out[4] * pm_in[3] - pm_out[5] * pm_in[7] - pm_out[6] * pm_in[11];
			pm_out[11] = -pm_out[8] * pm_in[3] - pm_out[9] * pm_in[7] - pm_out[10] * pm_in[11];
		}
		void s_inv_im(const double *im_in, double *im_out)
		{
			int ipiv[6];
			memcpy(im_out, im_in, 36 * sizeof(double));

			double query,*work;
			lapack_int lwork;
			LAPACKE_dsytrf_work(CblasRowMajor, 'L', 6, im_out, 6, ipiv, &query, -1);
			lwork = (lapack_int)query;
			work = (double *)s_malloc(lwork*sizeof(double));
			LAPACKE_dsytrf_work(CblasRowMajor, 'L', 6, im_out, 6, ipiv, work, lwork);

			/*不确定为啥这里不用查询work的size*/
			LAPACKE_dsytri_work(CblasRowMajor, 'L', 6, im_out, 6, ipiv, work);

			im_out[1] = im_out[6];
			im_out[2] = im_out[12];
			im_out[3] = im_out[18];
			im_out[4] = im_out[24];
			im_out[5] = im_out[30];

			im_out[8] = im_out[13];
			im_out[9] = im_out[19];
			im_out[10] = im_out[25];
			im_out[11] = im_out[31];

			im_out[15] = im_out[20];
			im_out[16] = im_out[26];
			im_out[17] = im_out[32];

			im_out[22] = im_out[27];
			im_out[23] = im_out[33];

			im_out[29] = im_out[34];

		}
		void s_axes2pm(const double *origin, const double *firstAxisPnt, const double *secondAxisPnt, double *pm_out, const char *axesOrder)
		{
			int Order[3];
			double Axis1[3], Axis2[3], Axis3[3];
			double nrm;

			Order[0] = axesOrder[0] - 'w';//asc玛顺序 uvw  xyz……
			Order[1] = axesOrder[1] - 'w';//asc玛顺序 uvw  xyz……
			Order[2] = 6 - Order[1] - Order[0];

			memcpy(Axis1, firstAxisPnt, sizeof(Axis1));
			memcpy(Axis2, secondAxisPnt, sizeof(Axis2));
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

			memset(pm_out, 0, sizeof(double)* 16);

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
		void s_pm2ep(const double *pm_in, double *ep_out, const char *EurType)
		{
			static const double P[3][3] = { { 0, -1, 1 }, { 1, 0, -1 }, { -1, 1, 0 } };
			static const double Q[3][3] = { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } };

			int a, b, c, d, e;
			double phi13, phi31, phi1, phi3;
			double phi[3];
			double s_, c_;

			double loc_pm[4][4];

			memcpy(loc_pm, pm_in, sizeof(loc_pm));

			a = EurType[0] - '1';
			b = EurType[1] - '1';
			c = EurType[2] - '1';

			d = 3 - a - b;
			e = 3 - b - c;

			/*计算phi2*/
			s_ = sqrt((loc_pm[a][b] * loc_pm[a][b] + loc_pm[a][e] * loc_pm[a][e]
				+ loc_pm[b][c] * loc_pm[b][c] + loc_pm[d][c] * loc_pm[d][c]) / 2);
			
			c_ = loc_pm[a][c];
			phi[1] = (a == c ? atan2(s_,c_) : atan2(P[a][c]*c_,s_));

			/*计算phi1和phi3*/
			phi13 = atan2(loc_pm[b][e] - loc_pm[d][b], loc_pm[b][b] + loc_pm[d][e]);
			phi31 = atan2(loc_pm[b][e] + loc_pm[d][b], loc_pm[b][b] - loc_pm[d][e]);

			phi1 = P[b][d] * (phi13 - phi31) / 2;
			phi3 = P[b][e] * (phi13 + phi31) / 2;

			/*检查*/
			bool if_add_pi = (loc_pm[a][e] * (P[a][e] + Q[a][e])*cos(phi3)<0) && (loc_pm[d][c] * (P[d][c] + Q[d][c])*cos(phi1)<0);
			phi[0] = (if_add_pi ? phi1 + PI : phi1);
			phi[2] = (if_add_pi ? phi3 + PI : phi3);

			phi[0] = (phi[0] < 0 ? phi[0] + 2 * PI : phi[0]);
			phi[2] = (phi[2] < 0 ? phi[2] + 2 * PI : phi[2]);

			/*对位置赋值*/
			memcpy(ep_out, phi, sizeof(phi));
			ep_out[3] = pm_in[3];
			ep_out[4] = pm_in[7];
			ep_out[5] = pm_in[11];

		}
		void s_ep2pm(const double *ep_in, double *pm_out, const char *EurType)
		{
			static const double P[3][3] = { { 0, -1, 1 }, { 1, 0, -1 }, { -1, 1, 0 } };
			static const double Q[3][3] = { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } };

			double Abb,Add,Abd,Adb;
			double Bac,Bae,Bdc,Bde;
			double Cbb,Cee,Cbe,Ceb;
			double s_, c_;

			int a, b, c, d, e;

			a = EurType[0] - '1';
			b = EurType[1] - '1';
			c = EurType[2] - '1';
			d = 3 - a - b;
			e = 3 - b - c;
			
			c_=std::cos(ep_in[0]);
			s_=std::sin(ep_in[0]);
			Abb = c_;
			Add = Abb;
			Abd = P[b][d] * s_;
			Adb = -Abd;

			s_ = std::sin(ep_in[1]);
			c_ = std::cos(ep_in[1]);
			Bac = P[a][c] * s_ + Q[a][c] * c_;
			Bae = P[a][e] * s_ + Q[a][e] * c_;
			Bdc = P[d][c] * s_ + Q[d][c] * c_;
			Bde = P[d][e] * s_ + Q[d][e] * c_;

			c_=std::cos(ep_in[2]);
			s_=std::sin(ep_in[2]);
			Cbb = c_;
			Cee = Cbb;
			Cbe = P[b][e] * s_;
			Ceb = -Cbe;

			memset(pm_out, 0, sizeof(double)* 16);

			pm_out[a * 4 + c] = Bac;
			pm_out[a * 4 + b] = Bae * Ceb;
			pm_out[a * 4 + e] = Bae * Cee;
			pm_out[b * 4 + c] = Abd * Bdc;
			pm_out[b * 4 + b] = Abb * Cbb + Abd * Bde * Ceb;
			pm_out[b * 4 + e] = Abb * Cbe + Abd * Bde * Cee;
			pm_out[d * 4 + c] = Add * Bdc;
			pm_out[d * 4 + b] = Adb * Cbb + Add * Bde * Ceb;
			pm_out[d * 4 + e] = Adb * Cbe + Add * Bde * Cee;


			pm_out[3] = ep_in[3];
			pm_out[7] = ep_in[4];
			pm_out[11] = ep_in[5];
			pm_out[15] = 1;
		}
		void s_tmf(const double *pm_in, double *tmf_out)
		{
			memset(tmf_out, 0, sizeof(double)* 36);

			memcpy(&tmf_out[0], &pm_in[0], sizeof(double)* 3);
			memcpy(&tmf_out[6], &pm_in[4], sizeof(double)* 3);
			memcpy(&tmf_out[12], &pm_in[8], sizeof(double)* 3);

			memcpy(&tmf_out[21], &pm_in[0], sizeof(double)* 3);
			memcpy(&tmf_out[27], &pm_in[4], sizeof(double)* 3);
			memcpy(&tmf_out[33], &pm_in[8], sizeof(double)* 3);

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
		void s_tmv(const double *pm_in, double *tmd_out)
		{
			memset(tmd_out, 0, sizeof(double)* 36);

			memcpy(&tmd_out[0], &pm_in[0], sizeof(double)* 3);
			memcpy(&tmd_out[6], &pm_in[4], sizeof(double)* 3);
			memcpy(&tmd_out[12], &pm_in[8], sizeof(double)* 3);

			memcpy(&tmd_out[21], &pm_in[0], sizeof(double)* 3);
			memcpy(&tmd_out[27], &pm_in[4], sizeof(double)* 3);
			memcpy(&tmd_out[33], &pm_in[8], sizeof(double)* 3);

			tmd_out[3] = -pm_in[11] * pm_in[4] + pm_in[7] * pm_in[8];
			tmd_out[9] = pm_in[11] * pm_in[0] - pm_in[3] * pm_in[8];
			tmd_out[15] = -pm_in[7] * pm_in[0] + pm_in[3] * pm_in[4];
			tmd_out[4] = -pm_in[11] * pm_in[5] + pm_in[7] * pm_in[9];
			tmd_out[10] = pm_in[11] * pm_in[1] - pm_in[3] * pm_in[9];
			tmd_out[16] = -pm_in[7] * pm_in[1] + pm_in[3] * pm_in[5];
			tmd_out[5] = -pm_in[11] * pm_in[6] + pm_in[7] * pm_in[10];
			tmd_out[11] = pm_in[11] * pm_in[2] - pm_in[3] * pm_in[10];
			tmd_out[17] = -pm_in[7] * pm_in[2] + pm_in[3] * pm_in[6];
		}
		void s_cmf(const double *vel_in, double *cm_out)
		{
			memset(cm_out, 0, sizeof(double)* 36);

			cm_out[6] = vel_in[5];
			cm_out[12] = -vel_in[4];
			cm_out[1] = -vel_in[5];
			cm_out[13] = vel_in[3];
			cm_out[2] = vel_in[4];
			cm_out[8] = -vel_in[3];

			cm_out[27] = vel_in[5];
			cm_out[33] = -vel_in[4];
			cm_out[22] = -vel_in[5];
			cm_out[34] = vel_in[3];
			cm_out[23] = vel_in[4];
			cm_out[29] = -vel_in[3];


			cm_out[24] = vel_in[2];
			cm_out[30] = -vel_in[1];
			cm_out[19] = -vel_in[2];
			cm_out[31] = vel_in[0];
			cm_out[20] = vel_in[1];
			cm_out[26] = -vel_in[0];
		}
		void s_cmv(const double *vel_in, double *cmd_out)
		{
			memset(cmd_out, 0, sizeof(double)* 36);

			cmd_out[6] = vel_in[5];
			cmd_out[12] = -vel_in[4];
			cmd_out[1] = -vel_in[5];
			cmd_out[13] = vel_in[3];
			cmd_out[2] = vel_in[4];
			cmd_out[8] = -vel_in[3];

			cmd_out[27] = vel_in[5];
			cmd_out[33] = -vel_in[4];
			cmd_out[22] = -vel_in[5];
			cmd_out[34] = vel_in[3];
			cmd_out[23] = vel_in[4];
			cmd_out[29] = -vel_in[3];

			cmd_out[9] = vel_in[2];
			cmd_out[15] = -vel_in[1];
			cmd_out[4] = -vel_in[2];
			cmd_out[16] = vel_in[0];
			cmd_out[5] = vel_in[1];
			cmd_out[11] = -vel_in[0];
		}
		void s_i2i(const double *from_pm_in, const double *from_im_in, double *to_im_out)
		{
			double x, y, z, old_x, old_y, old_z, new_x, new_y, new_z, m;

			m = from_im_in[0];

			x = from_pm_in[3];
			y = from_pm_in[7];
			z = from_pm_in[11];

			old_x = from_im_in[11];
			old_y = from_im_in[15];
			old_z = from_im_in[4];

			new_x = from_pm_in[0] * old_x + from_pm_in[1] * old_y + from_pm_in[2] * old_z;
			new_y = from_pm_in[4] * old_x + from_pm_in[5] * old_y + from_pm_in[6] * old_z;
			new_z = from_pm_in[8] * old_x + from_pm_in[9] * old_y + from_pm_in[10] * old_z;

			memset(to_im_out, 0, sizeof(double)* 36);
			/* 设置左上角的3*3的矩阵 */
			to_im_out[0] = from_im_in[0];
			to_im_out[7] = from_im_in[7];
			to_im_out[14] = from_im_in[14];
			/* 设置右上角的3*3的矩阵 */
			to_im_out[4] = m*z + new_z;
			to_im_out[5] = -m*y - new_y;
			to_im_out[11] = m*x + new_x;
			to_im_out[9] = -to_im_out[4];
			to_im_out[15] = -to_im_out[5];
			to_im_out[16] = -to_im_out[11];
			/* 设置左下角的3*3的矩阵 */
			to_im_out[24] = to_im_out[4];
			to_im_out[30] = to_im_out[5];
			to_im_out[19] = to_im_out[9];
			to_im_out[31] = to_im_out[11];
			to_im_out[20] = to_im_out[15];
			to_im_out[26] = to_im_out[16];
			/* 设置右下角的3*3的矩阵 */
			for (int i = 0; i < 3; ++i)
			{
				for (int j = 0; j < 3; ++j)
				{
					to_im_out[21] += from_pm_in[0 + i] * from_im_in[i * 6 + j + 21] * from_pm_in[j + 0];
					to_im_out[22] += from_pm_in[0 + i] * from_im_in[i * 6 + j + 21] * from_pm_in[j + 4];
					to_im_out[23] += from_pm_in[0 + i] * from_im_in[i * 6 + j + 21] * from_pm_in[j + 8];
					to_im_out[28] += from_pm_in[4 + i] * from_im_in[i * 6 + j + 21] * from_pm_in[j + 4];
					to_im_out[29] += from_pm_in[4 + i] * from_im_in[i * 6 + j + 21] * from_pm_in[j + 8];
					to_im_out[35] += from_pm_in[8 + i] * from_im_in[i * 6 + j + 21] * from_pm_in[j + 8];
				}
			}

			to_im_out[21] += m*(y*y + z*z) + 2 * (new_y*y + new_z*z);
			to_im_out[22] += -m*x*y - new_x*y - x*new_y;
			to_im_out[23] += -m*x*z - new_x*z - x*new_z;
			to_im_out[28] += m*(x*x + z*z) + 2 * (new_x*x + new_z*z);
			to_im_out[29] += -m*y*z - new_y*z - y*new_z;
			to_im_out[35] += m*(x*x + y*y) + 2 * (new_x*x + new_y*y);

			to_im_out[27] = to_im_out[22];
			to_im_out[33] = to_im_out[23];
			to_im_out[34] = to_im_out[29];

		}
		void s_v2v(const double *relative_pm_in, const double *relative_vel_in, const double *from_vel_in, double *to_vel_out)
		{
			if (from_vel_in==nullptr)
			{
				memcpy(to_vel_out, relative_vel_in, 6 * sizeof(double));
			}
			else
			{
				if (relative_pm_in == nullptr)
				{
					memcpy(to_vel_out, from_vel_in, 6 * sizeof(double));
				}
				else
				{
					memset(to_vel_out, 0, 6 * sizeof(double));

					cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, 3, 1, 3, 1, relative_pm_in, 4, from_vel_in, 1, 0, to_vel_out, 1);
					cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, 3, 1, 3, 1, relative_pm_in, 4, from_vel_in + 3, 1, 0, to_vel_out + 3, 1);

					to_vel_out[0] += -relative_pm_in[11] * to_vel_out[4] + relative_pm_in[7] * to_vel_out[5];
					to_vel_out[1] += relative_pm_in[11] * to_vel_out[3] - relative_pm_in[3] * to_vel_out[5];
					to_vel_out[2] += -relative_pm_in[7] * to_vel_out[3] + relative_pm_in[3] * to_vel_out[4];
				}
			}
		
			if (relative_vel_in!=0)
				s_daxpy(6, 1, relative_vel_in, 1, to_vel_out, 1);
		}
		void s_a2a(const double *relative_pm_in, const double *relative_vel_in, const double *relative_acc_in,
			const double *from_vel_in, const double *from_acc_in, double *to_acc_out, double *to_vel_out)
		{
			static double cmd[6][6], tmd[6][6], to_vel[6];
		
			/* calculate to_vel */
			s_v2v(relative_pm_in, relative_vel_in, from_vel_in, to_vel);
			/* calculate to_acc_out */
			if (from_acc_in == 0)
			{
				memset(to_acc_out, 0, 6 * sizeof(double));
			}
			else
			{
				if (relative_pm_in == 0)
				{
					memcpy(to_acc_out, from_acc_in, 6 * sizeof(double));
				}
				else
				{
					if (from_acc_in == 0)
					{
						memset(to_acc_out, 0, 6 * sizeof(double));
					}
					else
					{
						memset(to_acc_out, 0, 6 * sizeof(double));
						cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, 3, 1, 3, 1, relative_pm_in, 4, from_acc_in, 1, 0, to_acc_out, 1);
						cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, 3, 1, 3, 1, relative_pm_in, 4, from_acc_in + 3, 1, 0, to_acc_out + 3, 1);
					}

					to_acc_out[0] += -relative_pm_in[11] * to_acc_out[4] + relative_pm_in[7] * to_acc_out[5];
					to_acc_out[1] += relative_pm_in[11] * to_acc_out[3] - relative_pm_in[3] * to_acc_out[5];
					to_acc_out[2] += -relative_pm_in[7] * to_acc_out[3] + relative_pm_in[3] * to_acc_out[4];

				}
			}
			
			if (relative_acc_in != 0)
				s_daxpy(6, 1, relative_acc_in, 1, to_acc_out, 1);

			/* calculate to_acc_out */
			if (relative_vel_in != 0)
			{
				s_cmv(relative_vel_in, *cmd);
				cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, 6, 1, 6, 1, *cmd, 6, to_vel, 1, 1, to_acc_out, 1);
			}

			if (to_vel_out != 0)
			{
				memcpy(to_vel_out, to_vel, 6 * sizeof(double));
			}
		
		}
		void s_pnt2pnt(const double *relative_pm_in, const double *from_pnt, double *to_pnt_out)
		{
			if (relative_pm_in == 0)
			{
				if (from_pnt == 0)
				{
					memset(to_pnt_out, 0, sizeof(double)* 3);
				}
				else
				{
					memcpy(to_pnt_out, from_pnt, sizeof(double)* 3);
				}
			}
			else
			{
				if (from_pnt == 0)
				{
					to_pnt_out[0] = relative_pm_in[3];
					to_pnt_out[1] = relative_pm_in[7];
					to_pnt_out[2] = relative_pm_in[11];
				}
				else
				{
					s_pm_dot_pnt(relative_pm_in, from_pnt, to_pnt_out);
				}
			}
		}
		void s_pv2pv(const double *relative_pm_in, const double *relative_vel_in, const double *from_pnt, const double *from_pv,
			double *to_pv_out, double *to_pnt_out)
		{
			static double to_pnt[3];
			s_pnt2pnt(relative_pm_in, from_pnt, to_pnt);

			if (relative_vel_in == 0)
			{
				memset(to_pv_out, 0, sizeof(double)* 3);
			}
			else
			{
				s_cro3(relative_vel_in + 3, to_pnt, to_pv_out);
				s_daxpy(3, 1, relative_vel_in, 1, to_pv_out, 1);
			}

			if (from_pv != 0)
			{
				if (relative_pm_in == 0)
				{
					s_daxpy(3, 1, from_pv, 1, to_pv_out, 1);
				}
				else
				{
					s_dgemm(3, 1, 3, 1, relative_pm_in, 4, from_pv, 1, 1, to_pv_out, 1);
				}
			}

			if (to_pnt_out != 0)
				memcpy(to_pnt_out, to_pnt, sizeof(double)* 3);
		}
		void s_pa2pa(const double *relative_pm_in, const double *relative_vel_in, const double *relative_acc_in,
			const double *from_pnt, const double *from_pv, const double *from_pa, 
			double *to_pa_out, double *to_pv_out, double *to_pnt_out)
		{
			static double to_pnt[3],to_pv[3],tem_pv[3],tem_pa[3];
			/* 计算to_pnt和to_pv*/
			s_pv2pv(relative_pm_in, relative_vel_in, from_pnt, from_pv, to_pv, to_pnt);

			/* 计算to_pa */
			if (relative_acc_in == 0)
			{
				memset(to_pa_out, 0, sizeof(double)* 3);
			}
			else
			{
				s_cro3(relative_acc_in + 3, to_pnt, to_pa_out);
			}
				
			if (relative_vel_in == 0)
			{
				
			}
			else
			{
				memcpy(tem_pv, to_pv, sizeof(double)* 3);
				
				if (from_pv == 0)
				{
				}
				else
				{
					if (relative_pm_in == 0)
					{
						s_daxpy(3, 1, from_pv, 1, tem_pv, 1);
					}
					else
					{
						s_dgemm(3, 1, 3, 1, relative_pm_in, 4, from_pv, 1, 1, tem_pv, 1);
					}
				}
				
				s_cro3(relative_vel_in + 3, tem_pv, tem_pa);
				s_daxpy(3, 1, tem_pa, 1, to_pa_out, 1);
			}


			if (from_pa == 0)
			{
			}
			else
			{
				if (relative_pm_in == 0)
				{
					s_daxpy(3, 1, from_pa, 1, to_pa_out, 1);
				}
				else
				{
					s_dgemm(3, 1, 3, 1, relative_pm_in, 4, from_pa, 1, 1, to_pa_out, 1);
				}
			}
			

			if (relative_acc_in!=0)
				s_daxpy(3, 1, relative_acc_in, 1, to_pa_out, 1);


			if (to_pv_out != 0)
				memcpy(to_pv_out, to_pv, sizeof(double)* 3);

			if (to_pnt_out != 0)
				memcpy(to_pnt_out, to_pnt, sizeof(double)* 3);

		}
		void s_inv_pv2pv(const double *inv_relative_pm_in, const double *inv_relative_vel_in,
			const double *from_pnt, const double *from_pv, double *to_pv_out, double *to_pnt_out)
		{
			double tem[3],tem2[3];
			if (from_pv == nullptr)
			{
				memset(tem, 0, sizeof(tem));
			}
			else
			{
				memcpy(tem, from_pv, sizeof(tem));
			}

			if (inv_relative_vel_in == nullptr)
			{
			}
			else
			{
				s_daxpy(3, -1, inv_relative_vel_in, 1, tem, 1);
				if (from_pnt == nullptr)
				{
				}
				else
				{
					s_cro3(inv_relative_vel_in + 3, from_pnt, tem2);
					s_daxpy(3, -1, tem2, 1, tem, 1);
				}
			}

			if (inv_relative_pm_in == nullptr)
			{
				memcpy(to_pv_out, tem, sizeof(tem));
			}
			else
			{
				s_dgemmTN(3, 1, 3, 1, inv_relative_pm_in, 4, tem, 1, 0, to_pv_out, 1);
			}

			if (to_pnt_out != nullptr)
			{
				if (inv_relative_pm_in == 0)
				{
					memcpy(to_pnt_out, from_pnt, sizeof(double)* 3);
				}
				else
				{
					s_inv_pm_dot_pnt(inv_relative_pm_in, from_pnt, to_pnt_out);
				}
			}
		}
		void s_inv_pa2pa(const double *inv_relative_pm_in, const double *inv_relative_vel_in, const double *inv_relative_acc_in,
			const double *from_pnt, const double *from_pv, const double *from_pa,
			double *to_pa_out, double *to_pv_out, double *to_pnt_out)
		{
			double to_pnt[3], to_pv[3], tem[3],tem2[3],tem3[3];
			s_inv_pv2pv(inv_relative_pm_in, inv_relative_vel_in, from_pnt, from_pv, to_pv, to_pnt);

			if (inv_relative_acc_in == nullptr)
			{
				memset(tem, 0, sizeof(tem));
			}
			else
			{
				if (from_pnt == nullptr)
				{
					memcpy(tem, inv_relative_acc_in, sizeof(tem));
				}
				else
				{
					s_cro3(inv_relative_acc_in + 3, from_pnt, tem);
					s_daxpy(3, 1, inv_relative_acc_in, 1, tem, 1);
				}
			}

			if (inv_relative_vel_in == nullptr)
			{
				memset(tem3, 0, sizeof(tem3));
			}
			else
			{
				if (inv_relative_pm_in == nullptr)
				{
					memcpy(tem2, to_pv, sizeof(tem2));
				}
				else
				{
					s_dgemm(3, 1, 3, 1, inv_relative_pm_in, 4, to_pv, 1, 0, tem2, 1);
				}

				if (from_pv == nullptr)
				{

				}
				else
				{
					s_daxpy(3, 1, from_pv, 1, tem2, 1);
				}

				s_cro3(inv_relative_vel_in + 3, tem2, tem3);
			}

			s_daxpy(3, 1, tem, 1, tem3, 1);

			if (from_pv == nullptr)
			{
				if (inv_relative_pm_in == nullptr)
				{
					memset(to_pa_out, 0, sizeof(double)* 3);
					if (to_pa_out!=0)
						s_daxpy(3, -1, tem3, 1, to_pa_out, 1);
				}
				else
				{
					if (to_pa_out != nullptr)
						s_dgemmTN(3, 1, 3, -1, inv_relative_pm_in, 4, tem3, 1, 0, to_pa_out, 1);
				}
			}
			else
			{
				if (from_pa == nullptr)
				{
					memset(tem, 0, sizeof(tem));
				}
				else
				{
					memcpy(tem, from_pa, sizeof(tem));
				}
				
				s_daxpy(3, -1, tem3, 1, tem, 1);
				if (to_pa_out != nullptr)
					s_dgemmTN(3, 1, 3, 1, inv_relative_pm_in, 4, tem, 1, 0, to_pa_out, 1);
			}

			if (to_pv_out != nullptr)
			{
				memcpy(to_pv_out, to_pv, sizeof(to_pv));
			}

			if (to_pnt_out != nullptr)
			{
				memcpy(to_pnt_out, to_pnt, sizeof(to_pnt));
			}


			
		}
		void s_mass2im(const double mass_in, const double * inertia_in, const double *pm_in, double *im_out)
		{
			static double loc_im[6][6], loc_tm[6][6];
		
			memset(im_out, 0, sizeof(double)* 36);

			im_out[0] = mass_in;
			im_out[7] = mass_in;
			im_out[14] = mass_in;

			if (inertia_in != 0)
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

			if (pm_in != 0)
			{
				s_tmf(pm_in, *loc_tm);
				cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, 6, 6, 6, 1, *loc_tm, 6, im_out, 6, 0, *loc_im, 6);
				cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasTrans, 6, 6, 6, 1, *loc_im, 6, *loc_tm, 6, 0, im_out, 6);
			}
		
		}
		void s_gamma2im(const double * gamma_in, double *im_out)
		{
			memset(im_out, 0, sizeof(double)* 36);

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
		void s_im2gamma(const double * im_in, double *gamma_out)
		{
			memset(gamma_out, 0, sizeof(double)* 10);

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
	
		void s_pv(const double *pnt_in, const double *vel_in, double *pv_out)
		{
			s_cro3(vel_in + 3, pnt_in, pv_out);
			s_daxpy(3, 1, vel_in, 1, pv_out, 1);
		}
		void s_pa(const double *pnt_in, const double *vel_in, const double *acc_in, double *pnt_acc_out)
		{
			double tem1[3], tem2[3], tem3[3];
			//omega cross omega cross r
			s_cro3(vel_in + 3, pnt_in, tem2);
			s_cro3(vel_in + 3, tem2, tem1);

			s_cro3(vel_in + 3, vel_in, tem2);

			s_cro3(acc_in + 3, pnt_in, tem3);

			pnt_acc_out[0] = acc_in[0] + tem1[0] + tem2[0] + tem3[0];
			pnt_acc_out[1] = acc_in[1] + tem1[1] + tem2[1] + tem3[1];
			pnt_acc_out[2] = acc_in[2] + tem1[2] + tem2[2] + tem3[2];
		}

		void s_block_cpy(const int &block_size_m, const int &block_size_n,
			const double *from_mtrx, const int &fm_begin_row, const int &fm_begin_col, const int &fm_ld,
			double *to_mtrx, const int &tm_begin_row, const int &tm_begin_col, const int &tm_ld)
		{
			static int fm_place ;
			static int tm_place ;
		
			fm_place = fm_begin_row*fm_ld + fm_begin_col;
			tm_place = tm_begin_row*tm_ld + tm_begin_col;

			for (int i = 0; i < block_size_m; i++)
			{
				memcpy(&to_mtrx[tm_place]
					, &from_mtrx[fm_place], sizeof(double)*block_size_n);
				fm_place += fm_ld;
				tm_place += tm_ld;
			}

		}
		void s_dlt_col(const int &dlt_col_num,const int *col_index, const int &m, const int &n, double *A, const int &ldA)
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

		void s_pm_dot_pm(const double *pm1_in, const double *pm2_in, double *pm_out)
		{
			memset(pm_out, 0, sizeof(double)* 16);

			pm_out[3] = pm1_in[3];
			pm_out[7] = pm1_in[7];
			pm_out[11] = pm1_in[11];
			pm_out[15] = 1;

			cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, 3, 4, 3, 1, pm1_in, 4, pm2_in, 4, 1, pm_out, 4);
		}
		void s_inv_pm_dot_pm(const double *inv_pm1_in, const double *pm2_in, double *pm_out)
		{
			memset(pm_out, 0, sizeof(double)* 16);
			
			pm_out[3] = -inv_pm1_in[0] * inv_pm1_in[3] - inv_pm1_in[4] * inv_pm1_in[7] - inv_pm1_in[8] * inv_pm1_in[11];
			pm_out[7] = -inv_pm1_in[1] * inv_pm1_in[3] - inv_pm1_in[5] * inv_pm1_in[7] - inv_pm1_in[9] * inv_pm1_in[11];
			pm_out[11] = -inv_pm1_in[2] * inv_pm1_in[3] - inv_pm1_in[6] * inv_pm1_in[7] - inv_pm1_in[10] * inv_pm1_in[11];

			cblas_dgemm(CblasRowMajor, CblasTrans, CblasNoTrans, 3, 4, 3, 1, inv_pm1_in, 4, pm2_in, 4, 1, pm_out, 4);
		}
		void s_pm_dot_pnt(const double *pm_in, const double *pos_in, double *pos_out)
		{
			pos_out[0] = pm_in[3];
			pos_out[1] = pm_in[7];
			pos_out[2] = pm_in[11];

			cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, 3, 1, 3, 1, pm_in, 4, pos_in, 1, 1, pos_out, 1);
		}
		void s_inv_pm_dot_pnt(const double *pm_in, const double *pos_in, double *pos_out)
		{
			double tem[3];
			memcpy(tem, pos_in, sizeof(tem));

			cblas_daxpy(3, -1, &pm_in[3], 4, tem, 1);
			cblas_dgemm(CblasRowMajor, CblasTrans, CblasNoTrans, 3, 1, 3, 1, pm_in, 4, tem, 1, 0, pos_out, 1);
		}
		void s_tmf_dot_fce(const double *tm_in, const double *fce_in, double *fce_out)
		{
			memset(fce_out, 0, 6 * sizeof(double));
			cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, 6, 1, 6, 1, tm_in, 6, fce_in, 1, 0, fce_out, 1);
		}
		void s_tmv_dot_vel(const double *tmd_in, const double *vel_in, double *vel_out)
		{
			memset(vel_out, 0, 6 * sizeof(double));
			cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, 6, 1, 6, 1, tmd_in, 6, vel_in, 1, 0, vel_out, 1);
		}
		void s_im_dot_gravity(const double *im_in, const double *gravity, double *gravity_fce_out)
		{
			cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, 6, 1, 6, 1, im_in, 6, gravity, 1, 0, gravity_fce_out, 1);
		}
	

		void s_dscal(const int n, const double a, double *x, const int incx)
		{
			cblas_dscal(n, a, x, incx);
		}
		double s_dnrm2(const int n, const double *x, const int incx)
		{
			double nrm=0;
			for (int i = 0; i < n; ++i)
			{
				nrm += x[incx*i] * x[incx*i];
			}
			return sqrt(nrm);
		}
		void s_daxpy(const int N, const double alpha, const double *X, const int incX, double *Y, const int incY)
		{
			cblas_daxpy(N, alpha, X, incX, Y, incY);
		}
		void s_swap(const int N, double *X, const int incX, double *Y, const int incY)
		{
			cblas_dswap(N, X, incX, Y, incY);
		}
		void s_transpose(const int m, const int n, const double *A, const int ldA, double *B_out, const int ldB)
		{
			if ((n > ldA) || (m>ldB))
			{
				return;
			}


			for (int i = 0; i < m; ++i)
			{
				for (int j = 0; j < n; ++j)
				{
					B_out[j*ldB + i] = A[i*ldA + j];
				}
			}
		}

		void s_dgeinv(const int n, double* A, const int lda, int *ipiv)
		{
			LAPACKE_dgetrf_work(CblasRowMajor, n, n, A, lda, ipiv);
			double query;
			lapack_int lwork;
			LAPACKE_dgetri_work(CblasRowMajor, n, A, lda, ipiv,&query,-1);
			lwork = (lapack_int)query;

			double* work=(double *)s_malloc(lwork*sizeof(double));
			LAPACKE_dgetri_work(CblasRowMajor, n, A, lda, ipiv, work, lwork);

		}
		void s_dgemm(int m, int n, int k, double alpha, const double* A, int lda, const double* B, int ldb, double beta, double *C, int ldc)
		{
			cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, m, n, k, alpha,  A, lda,  B, ldb, beta, C, ldc);
		}
		void s_dgemmTN(int m, int n, int k, double alpha, const double* A, int lda, const double* B, int ldb, double beta, double *C, int ldc)
		{
			cblas_dgemm(CblasRowMajor, CblasTrans, CblasNoTrans, m, n, k, alpha, A, lda, B, ldb, beta, C, ldc);
		}
		void s_dgemmNT(int m, int n, int k, double alpha, const double* A, int lda, const double* B, int ldb, double beta, double *C, int ldc)
		{
			cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasTrans, m, n, k, alpha, A, lda, B, ldb, beta, C, ldc);
		}

		void s_dgesv(int n, int nrhs, double* a, int lda, int* ipiv, double* b, int ldb)
		{
			LAPACKE_dgesv_work(CblasRowMajor, n, nrhs, a, lda, ipiv, b, ldb);
		}
		void s_dgesvT(int n, int nrhs, double* a, int lda, int* ipiv, double* b, int ldb)
		{
			LAPACKE_dgesv_work(CblasColMajor, n, nrhs, a, lda, ipiv, b, ldb);
		}
		void s_dgelsd(int m, int n, int nrhs, double* a, int lda, double* b, int ldb, double* s, double rcond, int* rank)
		{
			double queryDouble;
			lapack_int queryInt;
			double *work_double;
			lapack_int* work_int;
			lapack_int lwork;
			LAPACKE_dgelsd_work(CblasRowMajor, m, n, nrhs, a, lda, b, ldb, s, rcond, rank, &queryDouble, -1, &queryInt);

			lwork = (lapack_int)queryDouble;
			work_double = (double *)s_malloc(sizeof(double)*(lapack_int)queryDouble + sizeof(lapack_int)*queryInt);
			work_int = (int *)(work_double + (lapack_int)queryDouble);
			LAPACKE_dgelsd_work(CblasRowMajor, m, n, nrhs, a, lda, b, ldb, s, rcond, rank, work_double, lwork, work_int);

		}
		void s_dgelsdT(int m, int n, int nrhs, double* a, int lda, double* b, int ldb, double* s, double rcond, int* rank)
		{
			double queryDouble;
			lapack_int queryInt;
			double *work_double;
			lapack_int* work_int;
			lapack_int lwork;
			LAPACKE_dgelsd_work(CblasColMajor, m, n, nrhs, a, lda, b, ldb, s, rcond, rank, &queryDouble, -1, &queryInt);

			lwork = (lapack_int)queryDouble;
			work_double = (double *)s_malloc(sizeof(double)*(lapack_int)queryDouble + sizeof(lapack_int)*queryInt);
			work_int = (int *)(work_double + (lapack_int)queryDouble);
			LAPACKE_dgelsd_work(CblasColMajor, m, n, nrhs, a, lda, b, ldb, s, rcond, rank, work_double, lwork, work_int);
		}

		void s_akima(unsigned inNum, unsigned outNum, const double *x_in, const double *y_in, const double *x_out, double *y_out, unsigned order)
		{
			std::list<std::pair<double,double> > v;
			std::vector<double> x,y;
			std::vector<double> dy(inNum);
			std::vector<double> dx(inNum);
			std::vector<double> d(inNum + 3);//slope
			std::vector<double> dd(inNum + 2);
			std::vector<double> t(inNum);
			std::vector<double> p0(inNum-1), p1(inNum-1), p2(inNum-1), p3(inNum-1);

			for (unsigned i = 0; i < inNum; ++i)
			{
				v.push_back(std::make_pair(x_in[i], y_in[i]));
			}
			/*对数据进行排序*/
			v.sort([](std::pair<double, double> x, std::pair<double, double> y)
			{
				if (x.first < y.first)
					return true;
				else
					return false;
			});

			for (auto &p : v)
			{
				x.push_back(p.first);
				y.push_back(p.second);
			}

			/*求差分*/
			for (unsigned i = 0; i <inNum-1; ++i)
			{
				dx.at(i) = x.at(i + 1) - x.at(i);
				dy.at(i) = y.at(i + 1) - y.at(i);
				d.at(i + 2) = dy.at(i) / dx.at(i);
			}

			d.at(1) = 2 * d.at(2) - d.at(3);
			d.at(0) = 2 * d.at(1) - d.at(2);
			d.at(d.size() - 2) = 2 * d.at(d.size() - 3) - d.at(d.size() - 4);
			d.at(d.size() - 1) = 2 * d.at(d.size() - 2) - d.at(d.size() - 3);

			/*求差分的差分的绝对值*/
			for (unsigned i = 0; i < inNum + 2; ++i)
			{
				dd.at(i) = std::abs(d.at(i + 1) - d.at(i));
			}
			
			for (unsigned i = 0; i < inNum; ++i)
			{
				if (abs(dd.at(i + 2) + dd.at(i)) < 0.000000001)
				{
					t.at(i) = (d.at(i + 1) + d.at(i + 2)) / 2;
				}
				else
				{
					t.at(i) = (dd.at(i + 2)*d.at(i + 1) + dd.at(i)*d.at(i + 2))
						/ (dd.at(i + 2) + dd.at(i));
				}
			}

			/*计算系数们*/
			memcpy(p0.data(), y_in, sizeof(double)*(inNum - 1));
			memcpy(p1.data(), t.data(), sizeof(double)*(inNum - 1));
			for (unsigned i = 0; i < inNum - 1; ++i)
			{
				p2.at(i) = (3 * d.at(i + 2) - 2 * t.at(i) - t.at(i + 1)) / dx.at(i);
				p3.at(i) = (t.at(i) + t.at(i + 1) - 2 * d.at(i + 2)) / (dx.at(i) * dx.at(i));
			}

			/*计算待计算的数*/

			for (unsigned i = 0; i < outNum; ++i)
			{
				unsigned j;
				for (j = 0; j < inNum-2; ++j)
				{
					if (x_out[i] < x.at(j+1))
					{
						break;
					}
				}

				double w = x_out[i] - x.at(j);

				switch (order)
				{
				case 0:
					y_out[i] = ((w*p3.at(j) + p2.at(j))*w + p1.at(j))*w + p0.at(j);
					break;
				case 1: 
					y_out[i] = (3 * w*p3.at(j) + 2 * p2.at(j))*w + p1.at(j);
					break;
				case 2:
					y_out[i] = 6 * w*p3.at(j) + 2 * p2.at(j);
					break;
				default:
					y_out[i] = ((w*p3.at(j) + p2.at(j))*w + p1.at(j))*w + p0.at(j);
					break;
				}
					






				
			}







			/*for (auto & a: v)
			{
				cout << a.first << "    " << a.second << std::endl;
			}

			cout << endl;

			for (auto & a : dv)
			{
				cout << a.first << "    " << a.second << std::endl;
			}

			cout << endl;

			for (auto & a : d)
			{
				cout << a << std::endl;
			}

			cout << endl;

			for (auto & a : dd)
			{
				cout << a << std::endl;
			}

			cout << endl;

			for (auto & a : t)
			{
				cout << a << std::endl;
			}*/

			for (auto & a : p0)
			{
				cout << a << std::endl;
			}
			cout << endl;

			for (auto & a : p1)
			{
				cout << a << std::endl;
			}
			cout << endl;

			for (auto & a : p2)
			{
				cout << a << std::endl;
			}
			cout << endl;

			for (auto & a : p3)
			{
				cout << a << std::endl;
			}
			cout << endl;
		}

		template<class T>
		unsigned GetID(const T &container, const typename T::value_type::element_type *pData)
		{
			auto p = std::find_if(container.begin(), container.end(), [pData](typename T::const_reference p)
			{
				if (p.get() == pData)
					return true;
				else
					return false;
			});

			if (p == container.end())
			{
				return std::numeric_limits<unsigned>::max();
			}
			else
			{
				return p - container.begin();
			}
		}


		OBJECT::OBJECT(MODEL *pModel, const string &Name)
			:_IsActive(true)
			, _Name(Name)
			, _pModel(pModel)
		{
		}
		OBJECT::~OBJECT()
		{
		}
		string OBJECT::GetName() const
		{
			return this->_Name;
		}
		bool OBJECT::GetActive() const
		{
			return this->_IsActive;
		}
		int OBJECT::Activate()
		{
			_IsActive = true;
			return 0;
		}
		int OBJECT::Deactivate()
		{
			_IsActive = false;
			return 0;
		}

		PART::PART(MODEL *pModel, const string &Name, const double *Im, const double *pm, const double *Vel, const double *Acc)
			:OBJECT(pModel, Name)
		{
			if (Im == 0)
			{
				memset(*_PrtIm, 0, sizeof(double)* 36);
				_PrtIm[0][0] = 1;
				_PrtIm[1][1] = 1;
				_PrtIm[2][2] = 1;
				_PrtIm[3][3] = 1;
				_PrtIm[4][4] = 1;
				_PrtIm[5][5] = 1;
			}
			else
			{
				cblas_dcopy(36, Im, 1, *_PrtIm, 1);
			}

			if (pm == 0)
			{
				double temp_pm[4][4];
				memset(temp_pm, 0, sizeof(double)* 16);
				temp_pm[0][0] = 1;
				temp_pm[1][1] = 1;
				temp_pm[2][2] = 1;
				temp_pm[3][3] = 1;

				SetPm(*temp_pm);
				s_inv_pm(*temp_pm, *_PrtPm);
			}
			else
			{
				//cblas_dcopy(16, pm, 1, *_Pm, 1);
				SetPm(pm);
				s_inv_pm(pm, *_PrtPm);
			}

			if (Vel == 0)
			{
				double temp_vel[6];
				memset(temp_vel, 0, sizeof(double)* 6);
				SetVel(temp_vel);
				//s_v2v(*_PrtPm, 0, temp_vel, _PrtVel);
			}
			else
			{
				//cblas_dcopy(6, Vel, 1, _Vel, 1);
				SetVel(Vel);
				//s_v2v(*_PrtPm, 0, Vel, _PrtVel);
			}

			if (Acc == 0)
			{
				memset(_Acc, 0, sizeof(double)* 6);
				memset(_PrtAcc, 0, sizeof(double)* 6);
			}
			else
			{
				cblas_dcopy(6, Acc, 1, _Acc, 1);
			}
		}
		void PART::UpdateInPrt()
		{
			static double tem[6];
		
			s_inv_pm(*_Pm, *_PrtPm);
			s_tmf(*_PrtPm, *_PrtTmf);
			s_tmv(*_PrtPm, *_PrtTmv);

			s_tmv_dot_vel(*_PrtTmv, _Vel, _PrtVel);
			s_cmf(_PrtVel, *_PrtCmf);
			s_cmv(_PrtVel, *_PrtCmv);

			s_tmv_dot_vel(*_PrtTmv, _Acc, _PrtAcc);

			s_dgemm(6, 1, 6, 1, *_PrtTmv, 6, _pModel->_Environment.Gravity, 1, 0, _PrtGravity, 1);
			s_dgemm(6, 1, 6, 1, *_PrtIm, 6, _PrtGravity, 1, 0, _PrtFg, 1);
		
			s_dgemm(6, 1, 6, 1, *_PrtIm, 6, _PrtVel, 1, 0, tem, 1);
			s_dgemm(6, 1, 6, 1, *_PrtCmf, 6, tem, 1, 0, _PrtFv, 1);
		}
		MARKER* PART::GetMarker(const std::string &Name)
		{
			auto pMak = _markerNames.find(Name);
			if (pMak != _markerNames.end())
			{
				return _pModel->_markers.at(pMak->second).get();
			}
			else
			{
				return nullptr;
			}
		}
		const MARKER* PART::GetMarker(const std::string &Name)const
		{
			auto pMak = _markerNames.find(Name);
			if (pMak != _markerNames.end())
			{
				return _pModel->_markers.at(pMak->second).get();
			}
			else
			{
				return nullptr;
			}
		}
		MARKER* PART::AddMarker(const std::string &Name, const double *pm, MARKER *pRelativeTo)
		{
			if (_markerNames.find(Name) != _markerNames.end())
			{
				return nullptr;
			}
			
			_pModel->_markers.push_back(std::shared_ptr<MARKER>(new MARKER(_pModel, Name, this, pm, pRelativeTo)));
			_markerNames[Name] = _pModel->_markers.size() - 1;
			return _pModel->_markers.back().get();
		}
		void PART::ToXMLElement(Aris::Core::ELEMENT *pEle) const
		{
			double value[10];
			
			pEle->DeleteChildren();
			pEle->SetName(this->_Name.data());

			Aris::Core::ELEMENT *pActive = pEle->GetDocument()->NewElement("Active");
			if (this->GetActive())
				pActive->SetText("True");
			else
				pActive->SetText("False");
			pEle->InsertEndChild(pActive);
			
			Aris::Core::ELEMENT *pInertia = pEle->GetDocument()->NewElement("Inertia");
			s_im2gamma(this->GetPrtImPtr(),value);
			pInertia->SetText(MATRIX(1,10,value).ToString().c_str());
			pEle->InsertEndChild(pInertia);

			Aris::Core::ELEMENT *pEP = pEle->GetDocument()->NewElement("Pos");
			s_pm2ep(*_Pm, value);
			pEP->SetText(MATRIX(1, 6, value).ToString().c_str());
			pEle->InsertEndChild(pEP);

			Aris::Core::ELEMENT *pVel = pEle->GetDocument()->NewElement("Vel");
			pVel->SetText(MATRIX(1, 6, _Vel).ToString().c_str());
			pEle->InsertEndChild(pVel);

			Aris::Core::ELEMENT *pAcc = pEle->GetDocument()->NewElement("Acc");
			pAcc->SetText(MATRIX(1, 6, _Acc).ToString().c_str());
			pEle->InsertEndChild(pAcc);

			Aris::Core::ELEMENT *pChildMak = pEle->GetDocument()->NewElement("ChildMarker");
			pEle->InsertEndChild(pChildMak);

			for (auto &m:_markerNames)
			{
				Aris::Core::ELEMENT *ele = pEle->GetDocument()->NewElement("");

				_pModel->_markers.at(m.second)->ToXMLElement(ele);
				pChildMak->InsertEndChild(ele);
			}

			Aris::Core::ELEMENT *pGraphicFilePath = pEle->GetDocument()->NewElement("Graphic_File_Path");
			pGraphicFilePath->SetText(this->graphicFilePath.c_str());
			pEle->InsertEndChild(pGraphicFilePath);
		}
		void PART::FromXMLElement(const Aris::Core::ELEMENT *pEle)
		{
			this->_Name = pEle->Name();

			if (strcmp("True", pEle->FirstChildElement("Active")->GetText()) == 0)
			{
				this->_IsActive = true;
			}
			else if (strcmp("False", pEle->FirstChildElement("Active")->GetText()) == 0)
			{
				this->_IsActive = false;
			}
			else
			{
				return ;
			}

			MATRIX m;
			
			m = _pModel->calculator.CalculateExpression(pEle->FirstChildElement("Inertia")->GetText());
			s_gamma2im(m.Data(), *_PrtIm);

			m = _pModel->calculator.CalculateExpression(pEle->FirstChildElement("Pos")->GetText());
			s_ep2pm(m.Data(), *_Pm);

			m = _pModel->calculator.CalculateExpression(pEle->FirstChildElement("Vel")->GetText());
			memcpy(_Vel, m.Data(), sizeof(_Vel));

			m = _pModel->calculator.CalculateExpression(pEle->FirstChildElement("Acc")->GetText());
			memcpy(_Acc, m.Data(), sizeof(_Acc));

			_markerNames.clear();

			for (const Aris::Core::ELEMENT *ele = pEle->FirstChildElement("ChildMarker")->FirstChildElement(); ele != 0; ele=ele->NextSiblingElement())
			{
				AddMarker(ele->Name())->FromXMLElement(ele);
			}

			if (pEle->FirstChildElement("Graphic_File_Path")->GetText()!=nullptr)
				graphicFilePath = pEle->FirstChildElement("Graphic_File_Path")->GetText();
		}
		unsigned PART::GetID() const
		{
			return DynKer::GetID<decltype(_pModel->_parts)>(_pModel->_parts, this);
		}

		MARKER::MARKER(MODEL *pModel, const string &Name, PART* pPart, const double *pLocPm, MARKER *pRelativeTo)
			: OBJECT(pModel, Name)
			, _pPrt(pPart)
		{
			if (pRelativeTo == nullptr)
			{
				if (pLocPm == nullptr)
				{
					memset(_PrtPm, 0, sizeof(_PrtPm));
					_PrtPm[0][0] = 1;
					_PrtPm[1][1] = 1;
					_PrtPm[2][2] = 1;
					_PrtPm[3][3] = 1;
				}
				else
				{
					memcpy(_PrtPm, pLocPm, sizeof(_PrtPm));
				}
			}
			else
			{
				if (pLocPm == nullptr)
				{
					memcpy(_PrtPm, pRelativeTo->GetPrtPmPtr(), sizeof(_PrtPm));
				}
				else
				{
					s_pm_dot_pm(pRelativeTo->GetPrtPmPtr(), pLocPm, *_PrtPm);
				}
			}
			
			
		}
		void MARKER::Update()
		{
			s_dgemm(4, 4, 4, 1, _pPrt->GetPmPtr(), 4, *_PrtPm, 4, 0, *_Pm, 4);
		}
		const double* MARKER::GetPrtPmPtr() const
		{
			return *_PrtPm;
		}
		int MARKER::ToXMLElement(Aris::Core::ELEMENT *pEle) const
		{
			double value[10];

			pEle->DeleteChildren();
			pEle->SetName(this->_Name.data());

			Aris::Core::ELEMENT *pEP = pEle->GetDocument()->NewElement("Pos");
			s_pm2ep(*_PrtPm, value);
			pEP->SetText(MATRIX(1,6,value).ToString().c_str());
			pEle->InsertEndChild(pEP);

			Aris::Core::ELEMENT *pRelativeMakEle = pEle->GetDocument()->NewElement("RelativeTo");
			pRelativeMakEle->SetText("");
			pEle->InsertEndChild(pRelativeMakEle);

			return 0;
		}
		int MARKER::FromXMLElement(const Aris::Core::ELEMENT *pEle)
		{
			double pm[4][4];

			this->_Name = pEle->Name();

			MATRIX m = _pModel->calculator.CalculateExpression(pEle->FirstChildElement("Pos")->GetText());
			s_ep2pm(m.Data(), *pm);

			if (pEle->FirstChildElement("RelativeTo")->GetText() != nullptr)
			{
				MARKER *pRelativeMak = _pPrt->GetMarker(pEle->FirstChildElement("RelativeTo")->GetText());
				s_pm_dot_pm(pRelativeMak->GetPrtPmPtr(), *pm, *_PrtPm);
			}
			else
			{
				memcpy(_PrtPm, *pm, sizeof(_PrtPm));
			}

			return 0;
		}
		unsigned MARKER::GetID() const
		{
			return DynKer::GetID<decltype(_pModel->_markers)>(_pModel->_markers, this);
		}

		JOINT::JOINT(MODEL *pModel, const std::string &Name, JOINT_TYPE Type, MARKER *pMakI, MARKER *pMakJ)
			: OBJECT(pModel, Name)
			, _Type(Type)
			, _pMakI(pMakI)
			, _pMakJ(pMakJ)
		{
		}
		void JOINT::_Initiate()
		{
			double loc_cst[6][6];

			memset(*_PrtCstMtxI, 0, sizeof(_PrtCstMtxI));
			memset(*_PrtCstMtxJ, 0, sizeof(_PrtCstMtxJ));

			memset(*loc_cst, 0, sizeof(loc_cst));

			/* Get tm I2M */
			s_tmf(_pMakI->GetPrtPmPtr(), *_tm_I2M);

			switch (_Type)
			{
			case ROTATIONAL:
				loc_cst[0][0] = 1;
				loc_cst[1][1] = 1;
				loc_cst[2][2] = 1;
				loc_cst[3][3] = 1;
				loc_cst[4][4] = 1;

				cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, 6, 5, 6, 1, *_tm_I2M, 6, *loc_cst, 6, 0, *_PrtCstMtxI, 6);
				break;
			case PRISMATIC:
				loc_cst[0][0] = 1;
				loc_cst[1][1] = 1;
				loc_cst[3][2] = 1;
				loc_cst[4][3] = 1;
				loc_cst[5][4] = 1;

				cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, 6, 5, 6, 1, *_tm_I2M, 6, *loc_cst, 6, 0, *_PrtCstMtxI, 6);
				break;
			case UNIVERSAL:
				loc_cst[0][0] = 1;
				loc_cst[1][1] = 1;
				loc_cst[2][2] = 1;

				cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, 6, 3, 6, 1, *_tm_I2M, 6, *loc_cst, 6, 0, *_PrtCstMtxI, 6);
				break;
			case SPHERICAL:
				loc_cst[0][0] = 1;
				loc_cst[1][1] = 1;
				loc_cst[2][2] = 1;

				cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, 6, 3, 6, 1, *_tm_I2M, 6, *loc_cst, 6, 0, *_PrtCstMtxI, 6);
				break;
			default:
				;
			}
		}
		void JOINT::UpdateInPrt()
		{
			double _pm_I2J[4][4];
			double _tm_M2N[6][6];
			double _tem_v1[6], _tem_v2[6];
			
			double _pm_M2N[4][4];
			double inverse_of_pmI[4][4];
			double v[3];
		
			memset(*_PrtCstMtxJ, 0, sizeof(double)* 36);
			memset(_a_c, 0, sizeof(double)* 6);

			double a, a_dot;
		
			/* Get pm M2N */
			s_pm_dot_pm(GetMakJ()->GetFatherPrt()->GetPrtPmPtr(), GetMakI()->GetFatherPrt()->GetPmPtr(), *_pm_M2N);
			s_tmf(*_pm_M2N, *_tm_M2N);

			/* Get Prt velocity and cro*/

			switch (_Type)
			{
			case ROTATIONAL:
				/*update PrtCstMtx*/
				cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, 6, 5, 6, -1, *_tm_M2N, 6, *_PrtCstMtxI, 6, 0, *_PrtCstMtxJ, 6);
				/*update A_c*/
				s_dgemmTN(6, 1, 6, 1, *_tm_M2N, 6, _pMakJ->GetFatherPrt()->GetPrtVelPtr(), 1, 0, _tem_v1, 1);
				s_dgemmTN(6, 1, 6, 1, _pMakI->GetFatherPrt()->GetPrtCmfPtr(), 6, _tem_v1, 1, 0, _tem_v2, 1);
				s_dgemmTN(5, 1, 6, 1, *_PrtCstMtxI, 6, _tem_v2, 1, 0, &_a_c[0], 1);
				break;
			case PRISMATIC:
				/*update PrtCstMtx*/
				cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, 6, 5, 6, -1, *_tm_M2N, 6, *_PrtCstMtxI, 6, 0, *_PrtCstMtxJ, 6);
				/*update A_c*/
				s_dgemmTN(6, 1, 6, 1, *_tm_M2N, 6, _pMakJ->GetFatherPrt()->GetPrtVelPtr(), 1, 0, _tem_v1, 1);
				s_dgemmTN(6, 1, 6, 1, _pMakI->GetFatherPrt()->GetPrtCmfPtr(), 6, _tem_v1, 1, 0, _tem_v2, 1);
				s_dgemmTN(5, 1, 6, 1, *_PrtCstMtxI, 6, _tem_v2, 1, 0, &_a_c[0], 1);
				break;
			case UNIVERSAL:
				/*update PrtCstMtx*/
				memset(*_pm_I2J, 0, sizeof(double)* 16);
				  //get pm from I to J
				_pMakI->Update();
				_pMakJ->Update();
				s_inv_pm(_pMakI->GetPmPtr(), *inverse_of_pmI);
				cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, 4, 4, 4, 1, *inverse_of_pmI, 4, _pMakJ->GetPmPtr(), 4, 0, *_pm_I2J, 4);

				a = std::atan2(_pm_I2J[2][1], _pm_I2J[1][1]);
				v[0] = -std::sin(a);
				v[1] = std::cos(a);

				cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, 3, 1, 2, 1, *_tm_I2M + 22, 6, v, 1, 0, *_PrtCstMtxI + 21, 6);
				cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, 6, 4, 6, -1, *_tm_M2N, 6, *_PrtCstMtxI, 6, 0, *_PrtCstMtxJ, 6);
				/*update A_c*/
				  /*calculate a_dot*/
				v[0] = _pMakJ->GetVelPtr()[3] - _pMakI->GetVelPtr()[3];
				v[1] = _pMakJ->GetVelPtr()[4] - _pMakI->GetVelPtr()[4];
				v[2] = _pMakJ->GetVelPtr()[5] - _pMakI->GetVelPtr()[5];

				a_dot = _pMakI->GetPmPtr()[0] * v[0] + _pMakI->GetPmPtr()[4] * v[1] + _pMakI->GetPmPtr()[8] * v[2];
				  /*calculate part m*/
				v[0] = -std::cos(a)*a_dot;
				v[1] = -std::sin(a)*a_dot;

				s_dgemmTN(6, 1, 6, 1, *_tm_I2M, 6, _pMakI->GetFatherPrt()->GetPrtVelPtr(), 1, 0, _tem_v1, 1);
				_a_c[3] -= v[0] * _tem_v1[4] + v[1] * _tem_v1[5];
				  /*calculate part n*/
				s_dgemmTN(6, 1, 6, 1, *_tm_M2N, 6, _pMakJ->GetFatherPrt()->GetPrtVelPtr(), 1, 0, _tem_v1, 1);
				s_dgemmTN(6, 1, 6, 1, _pMakI->GetFatherPrt()->GetPrtCmfPtr(), 6, _tem_v1, 1, 0, _tem_v2, 1);
				s_dgemmTN(4, 1, 6, 1, *_PrtCstMtxI, 6, _tem_v2, 1, 1, &_a_c[0], 1);

				s_dgemmTN(6, 1, 6, 1, *_tm_I2M, 6, _tem_v1, 1, 0, _tem_v2, 1);
				_a_c[3] += v[0] * _tem_v2[4] + v[1] * _tem_v2[5];
				break;
			case SPHERICAL:
				/*update PrtCstMtx*/
				cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, 6, 3, 6, -1, *_tm_M2N, 6, *_PrtCstMtxI, 6, 0, *_PrtCstMtxJ, 6);
				/*update A_c*/
				s_dgemmTN(6, 1, 6, 1, *_tm_M2N, 6, _pMakJ->GetFatherPrt()->GetPrtVelPtr(), 1, 0, _tem_v1, 1);
				s_dgemmTN(6, 1, 6, 1, _pMakI->GetFatherPrt()->GetPrtCmfPtr(), 6, _tem_v1, 1, 0, _tem_v2, 1);
				s_dgemmTN(3, 1, 6, 1, *_PrtCstMtxI, 6, _tem_v2, 1, 0, &_a_c[0], 1);
				break;
			default:
				;
			}
		}
		int JOINT::GetCstDim() const
		{
			switch (_Type)
			{
			case ROTATIONAL:
				return 5;
				break;
			case PRISMATIC:
				return 5;
				break;
			case UNIVERSAL:
				return 4;
				break;
			case SPHERICAL:
				return 3;
				break;
			default:
				return 5;
			}
		}
		int JOINT::ToXMLElement(Aris::Core::ELEMENT *pEle) const
		{
			pEle->DeleteChildren();
			pEle->SetName(this->_Name.data());

			Aris::Core::ELEMENT *pActive = pEle->GetDocument()->NewElement("Active");
			if (this->GetActive())
				pActive->SetText("True");
			else
				pActive->SetText("False");
			pEle->InsertEndChild(pActive);

			Aris::Core::ELEMENT *pType = pEle->GetDocument()->NewElement("Type");
			switch (this->_Type)
			{
			case JOINT::ROTATIONAL:
				pType->SetText("Rotational");
				break;
			case JOINT::PRISMATIC:
				pType->SetText("Prismatic");
				break;
			case JOINT::UNIVERSAL:
				pType->SetText("Universal");
				break;
			case JOINT::SPHERICAL:
				pType->SetText("Spherical");
				break;
			}
			pEle->InsertEndChild(pType);

			Aris::Core::ELEMENT *pPrtI = pEle->GetDocument()->NewElement("iPart");
			pPrtI->SetText(_pMakI->GetFatherPrt()->GetName().data());
			pEle->InsertEndChild(pPrtI);

			Aris::Core::ELEMENT *pPrtJ = pEle->GetDocument()->NewElement("jPart");
			pPrtJ->SetText(_pMakJ->GetFatherPrt()->GetName().data());
			pEle->InsertEndChild(pPrtJ);

			Aris::Core::ELEMENT *pMakI = pEle->GetDocument()->NewElement("iMarker");
			pMakI->SetText(_pMakI->GetName().data());
			pEle->InsertEndChild(pMakI);

			Aris::Core::ELEMENT *pMakJ = pEle->GetDocument()->NewElement("jMarker");
			pMakJ->SetText(_pMakJ->GetName().data());
			pEle->InsertEndChild(pMakJ);

			return 0;
		}
		int JOINT::FromXMLElement(const Aris::Core::ELEMENT *pEle)
		{
			this->_Name = pEle->Name();

			if (strcmp("True", pEle->FirstChildElement("Active")->GetText()) == 0){
				this->_IsActive = true;
			}
			else if (strcmp("False", pEle->FirstChildElement("Active")->GetText()) == 0){
				this->_IsActive = false;
			}
			else{
				return -1;
			}

			if (strcmp(pEle->FirstChildElement("Type")->GetText(), "Rotational") == 0)
			{
				_Type = ROTATIONAL;
			}
			else if (strcmp(pEle->FirstChildElement("Type")->GetText(), "Prismatic") == 0)
			{
				_Type = PRISMATIC;
			}
			else if (strcmp(pEle->FirstChildElement("Type")->GetText(), "Universal") == 0)
			{
				_Type = UNIVERSAL;
			}
			else if (strcmp(pEle->FirstChildElement("Type")->GetText(), "Spherical") == 0)
			{
				_Type = SPHERICAL;
			}
			else
			{
				return -1;
			}

			_pMakI = _pModel->GetPart(pEle->FirstChildElement("iPart")->GetText())->GetMarker(pEle->FirstChildElement("iMarker")->GetText());
			_pMakJ = _pModel->GetPart(pEle->FirstChildElement("jPart")->GetText())->GetMarker(pEle->FirstChildElement("jMarker")->GetText());

			return 0;
		}
		unsigned JOINT::GetID() const
		{
			return DynKer::GetID<decltype(_pModel->_joints)>(_pModel->_joints, this);
		}

		MOTION::MOTION(MODEL *pModel, const std::string &Name, MOTION_TYPE type, MOTION_MODE mode, MARKER *pMakI, MARKER *pMakJ)
			: OBJECT(pModel, Name)
			, _Type(type)
			, _Mode(mode)
			, _pMakI(pMakI)
			, _pMakJ(pMakJ)
		{
			memset(_frc_coe, 0, sizeof(double)* 3);
		}
		void MOTION::_Initiate()
		{
			double _tmf_I2M[6][6];
			double loc_cst[6][6];
			
			memset(*_PrtCstMtxI, 0, sizeof(_PrtCstMtxI));
			memset(*_PrtCstMtxJ, 0, sizeof(_PrtCstMtxJ));

			memset(*loc_cst, 0, sizeof(loc_cst));

			/* Get tm I2M */
			s_tmf(_pMakI->GetPrtPmPtr(), *_tmf_I2M);
			switch (_Type)
			{
			case LINEAR:
				loc_cst[2][0] = 1;

				cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, 6, 1, 6, 1, *_tmf_I2M, 6, *loc_cst, 6, 0, *_PrtCstMtxI, 6);
				break;
			}
		}
		void MOTION::UpdateInPrt()
		{
			double _pm_M2N[4][4];
			double _tmf_M2N[6][6];

			double tem_v1[6],tem_v2[6];

			memset(*_PrtCstMtxJ, 0, sizeof(double)* 36);
			memset(_a_c, 0, sizeof(double)* 6);

			/* Get tmf M2N */
			s_pm_dot_pm(_pMakJ->GetFatherPrt()->GetPrtPmPtr(), _pMakI->GetFatherPrt()->GetPmPtr(), *_pm_M2N);
			s_tmf(*_pm_M2N, *_tmf_M2N);

			switch (_Type)
			{
			case LINEAR:
				/*计算约束矩阵*/
				cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, 6, 1, 6, -1, *_tmf_M2N, 6, *_PrtCstMtxI, 6, 0, *_PrtCstMtxJ, 6);
				/* 计算a_c */
				switch (_Mode)
				{
				case POS_CONTROL:
					s_dgemmTN(6, 1, 6, 1, *_tmf_M2N, 6, _pMakJ->GetFatherPrt()->GetPrtVelPtr(), 1, 0, tem_v1, 1);
					
					s_dgemmTN(6, 1, 6, 1, _pMakI->GetFatherPrt()->GetPrtCmfPtr(), 6, tem_v1, 1, 0, tem_v2, 1);
					s_dgemmTN(1, 1, 6, 1, *_PrtCstMtxI, 6, tem_v2, 1, 0, &_a_c[0], 1);
					_a_c[0] += _a_m[0];
					break;
				case FCE_CONTROL:
					break;
				}
				break;
			}
		}
		int MOTION::GetCstDim() const
		{
			return 1;
		}
		int MOTION::ToXMLElement(Aris::Core::ELEMENT *pEle) const
		{
			pEle->DeleteChildren();
			pEle->SetName(this->_Name.data());

			Aris::Core::ELEMENT *pActive = pEle->GetDocument()->NewElement("Active");
			if (this->GetActive())
				pActive->SetText("True");
			else
				pActive->SetText("False");
			pEle->InsertEndChild(pActive);

			Aris::Core::ELEMENT *pType = pEle->GetDocument()->NewElement("Type");
			switch (this->_Type)
			{
			case MOTION::LINEAR:
				pType->SetText("Linear");
				break;
			}
			pEle->InsertEndChild(pType);

			Aris::Core::ELEMENT *pPrtI = pEle->GetDocument()->NewElement("iPart");
			pPrtI->SetText(_pMakI->GetFatherPrt()->GetName().data());
			pEle->InsertEndChild(pPrtI);

			Aris::Core::ELEMENT *pPrtJ = pEle->GetDocument()->NewElement("jPart");
			pPrtJ->SetText(_pMakJ->GetFatherPrt()->GetName().data());
			pEle->InsertEndChild(pPrtJ);

			Aris::Core::ELEMENT *pMakI = pEle->GetDocument()->NewElement("iMarker");
			pMakI->SetText(_pMakI->GetName().data());
			pEle->InsertEndChild(pMakI);

			Aris::Core::ELEMENT *pMakJ = pEle->GetDocument()->NewElement("jMarker");
			pMakJ->SetText(_pMakJ->GetName().data());
			pEle->InsertEndChild(pMakJ);

			Aris::Core::ELEMENT *pMode = pEle->GetDocument()->NewElement("Mode");
			switch (this->_Mode)
			{
			case MOTION::POS_CONTROL:
				pMode->SetText("Pos_Control");
				break;
			case MOTION::FCE_CONTROL:
				pMode->SetText("Fce_Control");
				break;
			}
			pEle->InsertEndChild(pMode);

			Aris::Core::ELEMENT *pFrictionCoefficients = pEle->GetDocument()->NewElement("Friction_Coefficients");
			
			pFrictionCoefficients->SetText(MATRIX(1,3,_frc_coe).ToString().c_str());
			pEle->InsertEndChild(pFrictionCoefficients);

			return 0;
		}
		int MOTION::FromXMLElement(const Aris::Core::ELEMENT *pEle)
		{
			this->_Name = pEle->Name();

			if (strcmp("True", pEle->FirstChildElement("Active")->GetText()) == 0)
			{
				this->_IsActive = true;
			}
			else if (strcmp("False", pEle->FirstChildElement("Active")->GetText()) == 0)
			{
				this->_IsActive = false;
			}
			else
			{
				return -1;
			}

			if (strcmp(pEle->FirstChildElement("Type")->GetText(), "Linear") == 0)
			{
				_Type = LINEAR;
			}

			if (strcmp(pEle->FirstChildElement("Mode")->GetText(), "Pos_Control") == 0)
			{
				_Mode = POS_CONTROL;
			}
			else if (strcmp(pEle->FirstChildElement("Mode")->GetText(), "Fce_Control") == 0)
			{
				_Mode = FCE_CONTROL;
			}
			
			MATRIX m = _pModel->calculator.CalculateExpression(pEle->FirstChildElement("Friction_Coefficients")->GetText());
			memcpy(_frc_coe, m.Data(), sizeof(_frc_coe));

			_pMakI = _pModel->GetPart(pEle->FirstChildElement("iPart")->GetText())->GetMarker(pEle->FirstChildElement("iMarker")->GetText());
			_pMakJ = _pModel->GetPart(pEle->FirstChildElement("jPart")->GetText())->GetMarker(pEle->FirstChildElement("jMarker")->GetText());


			return 0;
		}
		unsigned MOTION::GetID() const
		{
			return DynKer::GetID<decltype(_pModel->_motions)>(_pModel->_motions, this);
		}

		FORCE::FORCE(MODEL *pModel, const std::string &Name, FORCE::FORCE_TYPE type, PART *pPrtI, PART *pPrtJ, MARKER *pMakA, MARKER *pMakP, const double *force)
			: OBJECT(pModel, Name)
			, _Type(type)
			, _pPrtI(pPrtI)
			, _pPrtJ(pPrtJ)
			, _pMakA(pMakA)
			, _pMakP(pMakP)
		{
			if (force == 0)
			{
				memset(_LocFce, 0, sizeof(double)* 6);
			}
			else
			{
				cblas_dcopy(6, force, 1, _LocFce, 1);
			}
		}
		void FORCE::Update()
		{
			static double pm[4][4];
			static double tm[6][6];

			cblas_dcopy(16, _pMakA->GetPmPtr(), 1, *pm, 1);
			pm[0][3] = _pMakP->GetPmPtr()[3];
			pm[1][3] = _pMakP->GetPmPtr()[7];
			pm[2][3] = _pMakP->GetPmPtr()[11];

			s_tmf(*pm, *tm);

			memset(_Fce, 0, sizeof(double)* 6);

			cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, 6, 1, 6, 1, *tm, 6, _LocFce, 1, 0, _Fce, 1);
		}
		double* FORCE::GetFceMtxPtr() const
		{
			return (double*)_Fce;
		}
		void FORCE::SetFce(const double* pFce)
		{
			cblas_dcopy(6, pFce, 1, _LocFce, 1);
		}
		unsigned FORCE::GetID() const
		{
			return DynKer::GetID<decltype(_pModel->_forces)>(_pModel->_forces, this);
		}

		ENVIRONMENT::ENVIRONMENT(MODEL *pModel)
			:OBJECT(pModel,"Environment")
		{
			double data[] = { 0, -9.8, 0, 0, 0, 0 };
			memcpy(Gravity, data, sizeof(Gravity));
		}
		ENVIRONMENT::~ENVIRONMENT()
		{
		}

		int ENVIRONMENT::ToXMLElement(Aris::Core::ELEMENT *pEle) const
		{
			pEle->DeleteChildren();
			pEle->SetName("Enviroment");

			Aris::Core::ELEMENT *pGravity = pEle->GetDocument()->NewElement("Gravity");
			pGravity->SetText(MATRIX(1, 6, Gravity).ToString().c_str());
			pEle->InsertEndChild(pGravity);

			return 0;
		}
		int ENVIRONMENT::FromXMLElement(const Aris::Core::ELEMENT *pEle)
		{
			MATRIX m = _pModel->calculator.CalculateExpression(pEle->FirstChildElement("Gravity")->GetText());
			memcpy(Gravity, m.Data(), sizeof(Gravity));
			return 0;
		}

		MODEL::MODEL(const std::string & Name)
			: OBJECT(this , Name)
			, _Environment(this)
			, pGround(nullptr)
		{
			AddPart("Ground");
			pGround = GetPart("Ground");
		}
		MODEL::~MODEL()
		{
		}

		PART* MODEL::AddPart(const std::string & Name, const double *Im, const double *pm, const double *Vel, const double *Acc)
		{
			if (GetPart(Name)!=nullptr)
			{
				return nullptr;
			}
			
			_parts.push_back(std::shared_ptr<PART>(new PART(this, Name, Im, pm, Vel, Acc)));
			return _parts.back().get();
		}
		JOINT* MODEL::AddJoint(const std::string & Name, Aris::DynKer::JOINT::JOINT_TYPE type, MARKER* pMakI, MARKER* pMakJ)
		{
			if (GetJoint(Name) != nullptr)
			{
				return nullptr;
			}
			
			_joints.push_back(std::shared_ptr<JOINT>(new JOINT(this, Name, type, pMakI, pMakJ)));
			return _joints.back().get();
		}
		MOTION* MODEL::AddMotion(const std::string & Name, Aris::DynKer::MOTION::MOTION_TYPE type, MOTION::MOTION_MODE mode, MARKER *pMakI, MARKER *pMakJ)
		{
			if (GetMotion(Name) != nullptr)
			{
				return nullptr;
			}

			_motions.push_back(std::shared_ptr<MOTION>(new MOTION(this, Name, type, mode, pMakI, pMakJ)));
			return _motions.back().get();
		}
		FORCE* MODEL::AddForce(const std::string & Name, FORCE::FORCE_TYPE type, PART *pPrtI, PART *pPrtJ, MARKER *pMakI, MARKER *pMakJ, const double *fce)
		{
			if (GetForce(Name) != nullptr)
			{
				return nullptr;
			}

			_forces.push_back(std::shared_ptr<FORCE>(new FORCE(this, Name, type, pPrtI, pPrtJ, pMakI, pMakJ, fce)));
			return _forces.back().get();
		}
		
		template<class T>
		typename T::value_type::element_type * GetContent(const T &container, const string &Name)
		{
			auto p = std::find_if(container.begin(), container.end(), [Name](typename T::const_reference p)
			{
				if (p->GetName() == Name)
					return true;
				else
					return false;
			});

			if (p == container.end())
			{
				return nullptr;
			}
			else
			{
				return p->get();
			}
		}

		const PART *MODEL::GetPart(unsigned id) const
		{
			return _parts.at(id).get();
		}
		const JOINT *MODEL::GetJoint(unsigned id)const
		{
			return _joints.at(id).get();
		}
		const MOTION *MODEL::GetMotion(unsigned id)const
		{
			return _motions.at(id).get();
		}
		const FORCE *MODEL::GetForce(unsigned id)const
		{
			return _forces.at(id).get();
		}
		const MARKER *MODEL::GetMarker(unsigned id)const
		{
			return _markers.at(id).get();
		}
		PART *MODEL::GetPart(unsigned id)
		{
			return _parts.at(id).get();
		}
		JOINT *MODEL::GetJoint(unsigned id)
		{
			return _joints.at(id).get();
		}
		MOTION *MODEL::GetMotion(unsigned id)
		{
			return _motions.at(id).get();
		}
		FORCE *MODEL::GetForce(unsigned id)
		{
			return _forces.at(id).get();
		}
		MARKER *MODEL::GetMarker(unsigned id)
		{
			return _markers.at(id).get();
		}
		const PART *MODEL::GetPart(const std::string &Name)const
		{
			return GetContent<decltype(_parts)>(_parts, Name);
		}
		const JOINT *MODEL::GetJoint(const std::string &Name)const
		{
			return GetContent<decltype(_joints)>(_joints, Name);
		}
		const MOTION *MODEL::GetMotion(const std::string &Name)const
		{
			return GetContent<decltype(_motions)>(_motions, Name);
		}
		const FORCE *MODEL::GetForce(const std::string &Name)const
		{
			return GetContent<decltype(_forces)>(_forces, Name);
		}
		PART *MODEL::GetPart(const std::string &Name)
		{
			return GetContent<decltype(_parts)>(_parts, Name);
		}
		JOINT *MODEL::GetJoint(const std::string &Name)
		{
			return GetContent<decltype(_joints)>(_joints, Name);
		}
		MOTION *MODEL::GetMotion(const std::string &Name)
		{
			return GetContent<decltype(_motions)>(_motions, Name);
		}
		FORCE *MODEL::GetForce(const std::string &Name)
		{
			return GetContent<decltype(_forces)>(_forces, Name);
		}

		void MODEL::DynPre()
		{
			int pid = 0;//part id
			int cid = 6;//Constraint id

			for (auto &part:_parts)
			{
				if (part->GetActive())
				{
					part->_RowId = pid;
					pid += 6;
				}
				else
				{
					part->_RowId = 0;
				}
			}
			for (auto &joint:_joints)
			{
				if (joint->GetActive())
				{
					joint->_Initiate();
					joint->_ColId = cid;
					cid += joint->GetCstDim();
				}
				else
				{
					joint->_ColId = 0;
				}
			}
			for (auto &motion:_motions)
			{
				if ((motion->GetActive()) && (motion->_Mode == MOTION::POS_CONTROL))
				{
					motion->_ColId = cid;
					cid += motion->GetCstDim();
					motion->_Initiate();
				}
				else
				{
					motion->_ColId = 0;
					motion->_Initiate();
				}
			}

			I_dim = pid;
			C_dim = cid;

			_C.resize(C_dim*I_dim);
			C = _C.data();
			memset(C, 0, sizeof(double)*I_dim*C_dim);

			_I.resize(I_dim*I_dim);
			pI = _I.data();
			memset(pI, 0, sizeof(double)*I_dim*I_dim);
				
			_f.resize(I_dim);
			f = _f.data();
			memset(f, 0, sizeof(double)*I_dim);

			_a_c.resize(C_dim);
			a_c = _a_c.data();
			memset(a_c, 0, sizeof(double)*C_dim);
				
			_D.resize((I_dim + C_dim)*(I_dim + C_dim));
			D = _D.data();
			memset(D, 0, sizeof(double)*(I_dim + C_dim)*(I_dim + C_dim));

			_b.resize(I_dim + C_dim);
			b = _b.data();
			memset(b, 0, sizeof(double)*(I_dim + C_dim));

			_s.resize(I_dim + C_dim);
			s = _s.data();
			memset(s, 0, sizeof(double)*(I_dim + C_dim));

			_x.resize(I_dim + C_dim);
			x = _x.data();
			memset(x, 0, sizeof(double)*(I_dim + C_dim));

			for (int i = 0; i < 6; ++i)
			{
				pI[I_dim*pGround->_RowId + pGround->_RowId] = 1;
				C[C_dim*(pGround->_RowId + i) + i] = 1;
			}

		}
		void MODEL::DynPrtMtx()
		{
			memset(f, 0, I_dim*sizeof(double));
			memset(D, 0, (C_dim + I_dim)*(C_dim + I_dim)*sizeof(double));
			/*Update pI,and fces*/
			for (auto &p:_parts)
			{
				if (p->GetActive())
				{
					p->UpdateInPrt();
					s_block_cpy(6, 6, *(p->_PrtIm), 0, 0, 6, pI, p->_RowId, p->_RowId, I_dim);

					s_daxpy(6, -1, p->_PrtFg, 1, &f[p->_RowId], 1);
					s_daxpy(6, 1, p->_PrtFv, 1, &f[p->_RowId], 1);
				}
			}
			/*Update C , a_c and force-controlled-motion force*/
			for (auto &j:_joints)
			{
				if (j->GetActive())
				{
					j->UpdateInPrt();

					s_block_cpy(6, j->GetCstDim(), j->GetPrtCstMtxIPtr(), 0, 0, 6, C, j->_pMakI->GetFatherPrt()->_RowId, j->_ColId, C_dim);
					s_block_cpy(6, j->GetCstDim(), j->GetPrtCstMtxJPtr(), 0, 0, 6, C, j->_pMakJ->GetFatherPrt()->_RowId, j->_ColId, C_dim);

					memcpy(&a_c[j->_ColId], j->GetPrtA_cPtr(), j->GetCstDim() * sizeof(double));
				}
			}
			for (auto &m : _motions)
			{
				if (m->GetActive())
				{
					double tem_f[6];
					m->UpdateInPrt();
					switch (m->_Mode)
					{
					case MOTION::POS_CONTROL:
						s_block_cpy(6, m->GetCstDim(), m->GetPrtCstMtxIPtr(), 0, 0, 6, C, m->_pMakI->GetFatherPrt()->_RowId, m->_ColId, C_dim);
						s_block_cpy(6, m->GetCstDim(), m->GetPrtCstMtxJPtr(), 0, 0, 6, C, m->_pMakJ->GetFatherPrt()->_RowId, m->_ColId, C_dim);

						memcpy(&a_c[m->_ColId], m->GetPrtA_cPtr(), m->GetCstDim() * sizeof(double));
						break;
					case MOTION::FCE_CONTROL:
						/*补偿摩擦力*/
						for (int j = 0; j < m->GetCstDim(); j++)
						{
							tem_f[j] = m->_f_m[j]
								- s_sgn(m->GetV_mPtr()[j]) * m->_frc_coe[0]
								- m->GetV_mPtr()[j] * m->_frc_coe[1]
								- m->GetA_mPtr()[j] * m->_frc_coe[2];
						}
						/*补偿摩擦力完毕*/

						s_dgemm(6, 1, m->GetCstDim(), -1, m->GetPrtCstMtxIPtr(), 6, tem_f, 1, 1, &f[m->_pMakI->GetFatherPrt()->_RowId], 1);
						s_dgemm(6, 1, m->GetCstDim(), -1, m->GetPrtCstMtxJPtr(), 6, tem_f, 1, 1, &f[m->_pMakJ->GetFatherPrt()->_RowId], 1);
						break;
					}
				}
			}

			/*calculate D and b*/
			/* for D*/
			for (int i = 0; i < 6; ++i)
			{
				D[(C_dim + I_dim)*(pGround->_RowId + i) + (i + I_dim)] = 1;
				D[(C_dim + I_dim)*(i + I_dim) + (pGround->_RowId + i)] = 1;
			}

			for (auto &p : _parts)
			{
				if (p->GetActive())
				{
					for (int i = 0; i < 6; ++i)
					{
						for (int j = 0; j < 6; ++j)
						{
							D[(I_dim + C_dim)*(p->_RowId + i) + (p->_RowId + j)]
								= -pI[I_dim*(p->_RowId + i) + (p->_RowId + j)];
						}
					}
				}
			}

			for (auto &jnt : _joints)
			{
				if (jnt->GetActive())
				{
					for (int i = 0; i < 6; i++)
					{
						for (int j = 0; j < jnt->GetCstDim(); j++)
						{
							D[(C_dim + I_dim)*(jnt->_pMakI->GetFatherPrt()->_RowId + i) + (I_dim + jnt->_ColId + j)]
								= C[C_dim*(jnt->_pMakI->GetFatherPrt()->_RowId + i) + jnt->_ColId + j];
							D[(C_dim + I_dim)*(jnt->_pMakJ->GetFatherPrt()->_RowId + i) + (I_dim + jnt->_ColId + j)]
								= C[C_dim*(jnt->_pMakJ->GetFatherPrt()->_RowId + i) + jnt->_ColId + j];
							D[(jnt->_pMakI->GetFatherPrt()->_RowId + i) + (C_dim + I_dim)*(I_dim + jnt->_ColId + j)]
								= C[C_dim*(jnt->_pMakI->GetFatherPrt()->_RowId + i) + jnt->_ColId + j];
							D[(jnt->_pMakJ->GetFatherPrt()->_RowId + i) + (C_dim + I_dim)*(I_dim + jnt->_ColId + j)]
								= C[C_dim*(jnt->_pMakJ->GetFatherPrt()->_RowId + i) + jnt->_ColId + j];
						}
					}

				}
			}

			for (auto &m : _motions)
			{
				if (m->GetActive())
				{
					switch (m->_Mode)
					{
					case MOTION::POS_CONTROL:
						for (int i = 0; i < 6; i++)
						{
							for (int j = 0; j < m->GetCstDim(); j++)
							{
								D[(C_dim + I_dim)*(m->_pMakI->GetFatherPrt()->_RowId + i) + (I_dim + m->_ColId + j)]
									= C[C_dim*(m->_pMakI->GetFatherPrt()->_RowId + i) + m->_ColId + j];
								D[(C_dim + I_dim)*(m->_pMakJ->GetFatherPrt()->_RowId + i) + (I_dim + m->_ColId + j)]
									= C[C_dim*(m->_pMakJ->GetFatherPrt()->_RowId + i) + m->_ColId + j];
								D[(m->_pMakI->GetFatherPrt()->_RowId + i) + (C_dim + I_dim)*(I_dim + m->_ColId + j)]
									= C[C_dim*(m->_pMakI->GetFatherPrt()->_RowId + i) + m->_ColId + j];
								D[(m->_pMakJ->GetFatherPrt()->_RowId + i) + (C_dim + I_dim)*(I_dim + m->_ColId + j)]
									= C[C_dim*(m->_pMakJ->GetFatherPrt()->_RowId + i) + m->_ColId + j];
							}
						}
						break;
					case MOTION::FCE_CONTROL:

						break;
					}
				}
			}

			s_block_cpy(I_dim, 1, f, 0, 0, 1, b, 0, 0, 1);
			s_block_cpy(C_dim, 1, a_c, 0, 0, 1, b, I_dim, 0, 1);


			/*以下求解*/
			memcpy(x, b, (C_dim + I_dim)*sizeof(double));
		}
		void MODEL::Dyn()
		{
			double rcond = 0.000000000001;
			int rank;
			s_dgelsd(C_dim + I_dim,
				C_dim + I_dim,
				1,
				D,
				C_dim + I_dim,
				x,
				1,
				s,
				rcond,
				&rank);

			for (auto &p:_parts)
			{
				if (p->GetActive())
				{
					memcpy(p->GetPrtAccPtr(), &x[p->_RowId], sizeof(double) * 6);
				}
			}
			for (auto &j : _joints)
			{
				if (j->GetActive())
				{
					memcpy(j->_CstFce, &x[j->_ColId + I_dim], j->GetCstDim() * sizeof(double));
				}
			}
			for (auto &m:_motions)
			{
				if (m->GetActive())
				{
					switch (m->_Mode)
					{
					case MOTION::POS_CONTROL:
						/*补偿摩擦力*/
						/*x[m->_ColId + I_dim] += m->_frc_coe[0] * s_sgn(*m->GetV_mPtr())
							+ m->_frc_coe[1] * (*m->GetV_mPtr())
							+ m->_frc_coe[2] * (*m->GetA_mPtr());*/

						memcpy(m->_f_m, &x[m->_ColId + I_dim], m->GetCstDim() * sizeof(double));
						/*补偿摩擦力*/
						*m->_f_m += m->_frc_coe[0] * s_sgn(*m->GetV_mPtr())
							+ m->_frc_coe[1] * (*m->GetV_mPtr())
							+ m->_frc_coe[2] * (*m->GetA_mPtr());
						break;
					case MOTION::FCE_CONTROL:
						break;
					}
				}
			}

		}

		void MODEL::ClbEqnTo(double *&clb_d_ptr, double *&clb_b_ptr, unsigned int &clb_dim_m, unsigned int &clb_dim_n)
		{
			this->DynPre();
			if (C_dim != I_dim)
			{
				throw std::logic_error("must calibrate square matrix");
			}
			
			unsigned dim = I_dim;

			static MATRIX _clb_d;
			static MATRIX _clb_b;

			/*初始化*/
			clb_dim_m = 0;
			clb_dim_n = 0;
			unsigned clb_prt_dim_n = 0;
			for (auto &i : _motions)
			{
				if (i->GetActive() && (i->GetMode() == MOTION::POS_CONTROL))
				{
					clb_dim_m += i->GetCstDim();
				}

				clb_dim_n += 3 * i->GetCstDim();
			}
			
			for (auto &i : _parts)//不算地面
			{
				if (i->GetActive())
				{
					clb_dim_n += 10;
					clb_prt_dim_n += 10;
				}
			}

			_clb_d.Resize(clb_dim_m, clb_dim_n);
			_clb_b.Resize(clb_dim_m, 1);

			memset(_clb_d.Data(), 0, _clb_d.Length() * sizeof(double));
			memset(_clb_b.Data(), 0, _clb_b.Length() * sizeof(double));

			/*开始计算*/
			memset(C, 0, dim*dim * sizeof(double));

			/*Update all*/
			for (auto &i:_parts)
			{
				if (i->GetActive())
				{
					i->UpdateInPrt();
				}
			}
			for (auto &i : _joints)
			{
				if (i->GetActive())
				{
					i->UpdateInPrt();
				}
			}

			/*计算C以及f*/
			for (auto &j : _joints)
			{
				if (j->GetActive())
				{
					j->UpdateInPrt();

					s_block_cpy(6, j->GetCstDim(), j->GetPrtCstMtxIPtr(), 0, 0, 6, C, j->_pMakI->GetFatherPrt()->_RowId, j->_ColId, dim);
					s_block_cpy(6, j->GetCstDim(), j->GetPrtCstMtxJPtr(), 0, 0, 6, C, j->_pMakJ->GetFatherPrt()->_RowId, j->_ColId, dim);

					memcpy(&a_c[j->_ColId], j->GetPrtA_cPtr(), j->GetCstDim() * sizeof(double));
				}
			}
			for (auto &m : _motions)
			{
				if (m->GetActive())
				{
					m->UpdateInPrt();
					switch (m->_Mode)
					{
					case MOTION::POS_CONTROL:
						s_block_cpy(6, m->GetCstDim(), m->GetPrtCstMtxIPtr(), 0, 0, 6, C, m->_pMakI->GetFatherPrt()->_RowId, m->_ColId, dim);
						s_block_cpy(6, m->GetCstDim(), m->GetPrtCstMtxJPtr(), 0, 0, 6, C, m->_pMakJ->GetFatherPrt()->_RowId, m->_ColId, dim);
						break;
					case MOTION::FCE_CONTROL:
						s_dgemm(6, 1, m->GetCstDim(), 1, m->GetPrtCstMtxIPtr(), 6, m->_f_m, 1, 1, &f[m->_pMakI->GetFatherPrt()->_RowId], 1);
						s_dgemm(6, 1, m->GetCstDim(), 1, m->GetPrtCstMtxJPtr(), 6, m->_f_m, 1, 1, &f[m->_pMakJ->GetFatherPrt()->_RowId], 1);
						break;
					}
				}
			}
			for (int i = 0; i < 6; ++i)
			{
				C[dim*(pGround->_RowId + i) + i] = 1;
			}

			/*求解C的逆，即A*/
			MATRIX A(dim, dim), B(dim, dim);
			std::vector<int> ipiv(dim);

			memcpy(A.Data(), C, sizeof(double)*A.Length());
			s_dgeinv(dim, A.Data(), dim, ipiv.data());

			

			/*求解B*/
			int beginRow = dim - clb_dim_m;

			for (auto &i:_parts)
			{
				if (i->GetActive())
				{
					double cm[6][6];
					
					s_cmf(i->GetPrtVelPtr(), *cm);
					s_dgemm(clb_dim_m, 6, 6, 1, &A(beginRow,i->_RowId), dim, *cm, 6, 0, &B(beginRow, i->_RowId), dim);
				}
			}

			/*求解clb_d*/
			int col1 = 0, col2 = 0;

			for (auto &i:_parts)
			{
				if (i->GetActive())
				{
					double q[6], v[6];

					memset(q, 0, sizeof(double) * 6);

					memcpy(q, i->GetPrtAccPtr(), sizeof(double) * 6);
					s_daxpy(6, -1, i->GetPrtGravityPtr(), 1, q, 1);

					memcpy(v, i->GetPrtVelPtr(), sizeof(double) * 6);

					for (unsigned int j = 0; j < clb_dim_m; ++j)
					{
						/*_clb_d[j][col1] = A[beginRow + j][col2 + 0] * q[0] + A[beginRow + j][col2 + 1] * q[1] + A[beginRow + j][col2 + 2] * q[2];
						_clb_d[j][col1 + 1] = A[beginRow + j][col2 + 1] * q[5] + A[beginRow + j][col2 + 5] * q[1] - A[beginRow + j][col2 + 2] * q[4] - A[beginRow + j][col2 + 4] * q[2];
						_clb_d[j][col1 + 2] = A[beginRow + j][col2 + 2] * q[3] + A[beginRow + j][col2 + 3] * q[2] - A[beginRow + j][col2 + 0] * q[5] - A[beginRow + j][col2 + 5] * q[0];
						_clb_d[j][col1 + 3] = A[beginRow + j][col2 + 0] * q[4] + A[beginRow + j][col2 + 4] * q[0] - A[beginRow + j][col2 + 1] * q[3] - A[beginRow + j][col2 + 3] * q[1];
						_clb_d[j][col1 + 4] = A[beginRow + j][col2 + 3] * q[3];
						_clb_d[j][col1 + 5] = A[beginRow + j][col2 + 4] * q[4];
						_clb_d[j][col1 + 6] = A[beginRow + j][col2 + 5] * q[5];
						_clb_d[j][col1 + 7] = A[beginRow + j][col2 + 3] * q[4] + A[beginRow + j][col2 + 4] * q[3];
						_clb_d[j][col1 + 8] = A[beginRow + j][col2 + 3] * q[5] + A[beginRow + j][col2 + 5] * q[3];
						_clb_d[j][col1 + 9] = A[beginRow + j][col2 + 4] * q[5] + A[beginRow + j][col2 + 5] * q[4];

						_clb_d[j][col1] += B[beginRow + j][col2 + 0] * v[0] + B[beginRow + j][col2 + 1] * v[1] + B[beginRow + j][col2 + 2] * v[2];
						_clb_d[j][col1 + 1] += B[beginRow + j][col2 + 1] * v[5] + B[beginRow + j][col2 + 5] * v[1] - B[beginRow + j][col2 + 2] * v[4] - B[beginRow + j][col2 + 4] * v[2];
						_clb_d[j][col1 + 2] += B[beginRow + j][col2 + 2] * v[3] + B[beginRow + j][col2 + 3] * v[2] - B[beginRow + j][col2 + 0] * v[5] - B[beginRow + j][col2 + 5] * v[0];
						_clb_d[j][col1 + 3] += B[beginRow + j][col2 + 0] * v[4] + B[beginRow + j][col2 + 4] * v[0] - B[beginRow + j][col2 + 1] * v[3] - B[beginRow + j][col2 + 3] * v[1];
						_clb_d[j][col1 + 4] += B[beginRow + j][col2 + 3] * v[3];
						_clb_d[j][col1 + 5] += B[beginRow + j][col2 + 4] * v[4];
						_clb_d[j][col1 + 6] += B[beginRow + j][col2 + 5] * v[5];
						_clb_d[j][col1 + 7] += B[beginRow + j][col2 + 3] * v[4] + B[beginRow + j][col2 + 4] * v[3];
						_clb_d[j][col1 + 8] += B[beginRow + j][col2 + 3] * v[5] + B[beginRow + j][col2 + 5] * v[3];
						_clb_d[j][col1 + 9] += B[beginRow + j][col2 + 4] * v[5] + B[beginRow + j][col2 + 5] * v[4];*/

						_clb_d(j, col1) = A(beginRow + j, col2 + 0) * q[0] + A(beginRow + j, col2 + 1) * q[1] + A(beginRow + j, col2 + 2) * q[2];
						_clb_d(j, col1 + 1) = A(beginRow + j, col2 + 1) * q[5] + A(beginRow + j, col2 + 5) * q[1] - A(beginRow + j, col2 + 2) * q[4] - A(beginRow + j, col2 + 4) * q[2];
						_clb_d(j, col1 + 2) = A(beginRow + j, col2 + 2) * q[3] + A(beginRow + j, col2 + 3) * q[2] - A(beginRow + j, col2 + 0) * q[5] - A(beginRow + j, col2 + 5) * q[0];
						_clb_d(j, col1 + 3) = A(beginRow + j, col2 + 0) * q[4] + A(beginRow + j, col2 + 4) * q[0] - A(beginRow + j, col2 + 1) * q[3] - A(beginRow + j, col2 + 3) * q[1];
						_clb_d(j, col1 + 4) = A(beginRow + j, col2 + 3) * q[3];
						_clb_d(j, col1 + 5) = A(beginRow + j, col2 + 4) * q[4];
						_clb_d(j, col1 + 6) = A(beginRow + j, col2 + 5) * q[5];
						_clb_d(j, col1 + 7) = A(beginRow + j, col2 + 3) * q[4] + A(beginRow + j, col2 + 4) * q[3];
						_clb_d(j, col1 + 8) = A(beginRow + j, col2 + 3) * q[5] + A(beginRow + j, col2 + 5) * q[3];
						_clb_d(j, col1 + 9) = A(beginRow + j, col2 + 4) * q[5] + A(beginRow + j, col2 + 5) * q[4];

						_clb_d(j, col1) += B(beginRow + j, col2 + 0) * v[0] + B(beginRow + j, col2 + 1) * v[1] + B(beginRow + j, col2 + 2) * v[2];
						_clb_d(j, col1 + 1) += B(beginRow + j, col2 + 1) * v[5] + B(beginRow + j, col2 + 5) * v[1] - B(beginRow + j, col2 + 2) * v[4] - B(beginRow + j, col2 + 4) * v[2];
						_clb_d(j, col1 + 2) += B(beginRow + j, col2 + 2) * v[3] + B(beginRow + j, col2 + 3) * v[2] - B(beginRow + j, col2 + 0) * v[5] - B(beginRow + j, col2 + 5) * v[0];
						_clb_d(j, col1 + 3) += B(beginRow + j, col2 + 0) * v[4] + B(beginRow + j, col2 + 4) * v[0] - B(beginRow + j, col2 + 1) * v[3] - B(beginRow + j, col2 + 3) * v[1];
						_clb_d(j, col1 + 4) += B(beginRow + j, col2 + 3) * v[3];
						_clb_d(j, col1 + 5) += B(beginRow + j, col2 + 4) * v[4];
						_clb_d(j, col1 + 6) += B(beginRow + j, col2 + 5) * v[5];
						_clb_d(j, col1 + 7) += B(beginRow + j, col2 + 3) * v[4] + B(beginRow + j, col2 + 4) * v[3];
						_clb_d(j, col1 + 8) += B(beginRow + j, col2 + 3) * v[5] + B(beginRow + j, col2 + 5) * v[3];
						_clb_d(j, col1 + 9) += B(beginRow + j, col2 + 4) * v[5] + B(beginRow + j, col2 + 5) * v[4];

					}
					col1 += 10;
					col2 += 6;
				}
			}

			/*求解clb_b*/
			int row = 0;
			for (auto &i : _motions)
			{
				if ((i->GetActive()) && (i->_Mode == MOTION::POS_CONTROL))
				{
					memcpy(&_clb_b(row, 0), i->_f_m, sizeof(double)*i->GetCstDim());
					row += i->GetCstDim();
				}
			}

			s_dgemm(clb_dim_m, 1, dim, 1, &A(beginRow,0), dim, f, 1, 1, _clb_b.Data(), 1);

			//dsp(f, dim, 1);


			/*以下添加驱动摩擦系数*/
			row = 0;
			unsigned num = 0;
			for (auto &i:_motions)
			{
				if (i->GetActive())
				{
					if (i->_Mode == MOTION::POS_CONTROL)
					{

						_clb_d(row, clb_prt_dim_n + num * 3) += s_sgn(*i->GetV_mPtr());
						_clb_d(row, clb_prt_dim_n + num * 3 + 1) += *i->GetV_mPtr();
						_clb_d(row, clb_prt_dim_n + num * 3 + 2) += *i->GetA_mPtr();
						++row;
					}
					else
					{
						s_dgemm(clb_dim_m, 1, 6, s_sgn(*i->GetV_mPtr()), &A(beginRow, i->_pMakI->_pPrt->_RowId), dim, i->_PrtCstMtxI[0], 6, 1, &_clb_d(0, clb_prt_dim_n + num * 3), clb_dim_n);
						s_dgemm(clb_dim_m, 1, 6, s_sgn(*i->GetV_mPtr()), &A(beginRow, i->_pMakJ->_pPrt->_RowId), dim, i->_PrtCstMtxJ[0], 6, 1, &_clb_d(0, clb_prt_dim_n + num * 3), clb_dim_n);
						s_dgemm(clb_dim_m, 1, 6, *i->GetV_mPtr(), &A(beginRow, i->_pMakI->_pPrt->_RowId), dim, i->_PrtCstMtxI[0], 6, 1, &_clb_d(0, clb_prt_dim_n + num * 3 + 1), clb_dim_n);
						s_dgemm(clb_dim_m, 1, 6, *i->GetV_mPtr(), &A(beginRow, i->_pMakJ->_pPrt->_RowId), dim, i->_PrtCstMtxJ[0], 6, 1, &_clb_d(0, clb_prt_dim_n + num * 3 + 1), clb_dim_n);
						s_dgemm(clb_dim_m, 1, 6, *i->GetA_mPtr(), &A(beginRow, i->_pMakI->_pPrt->_RowId), dim, i->_PrtCstMtxI[0], 6, 1, &_clb_d(0, clb_prt_dim_n + num * 3 + 2), clb_dim_n);
						s_dgemm(clb_dim_m, 1, 6, *i->GetA_mPtr(), &A(beginRow, i->_pMakJ->_pPrt->_RowId), dim, i->_PrtCstMtxJ[0], 6, 1, &_clb_d(0, clb_prt_dim_n + num * 3 + 2), clb_dim_n);
					}
				}
				num++;
			}


			clb_d_ptr = _clb_d.Data();
			clb_b_ptr = _clb_b.Data();
		}

		void MODEL::LoadXML(const char *filename)
		{
			if (XML_Doc.LoadFile(filename) != 0)
			{
				throw std::logic_error((string("could not open file:") + string(filename)));
			}

			const Aris::Core::ELEMENT *pModel = XML_Doc.FirstChildElement("Model");
			if (pModel == nullptr)throw(std::logic_error("XML file must have model element"));

			const Aris::Core::ELEMENT *pVar = pModel->FirstChildElement("Variable");
			if (pModel == nullptr)throw(std::logic_error("Model must have variable element"));
			const Aris::Core::ELEMENT *pEnv = pModel->FirstChildElement("Environment");
			if (pEnv == nullptr)throw(std::logic_error("Model must have environment element"));
			const Aris::Core::ELEMENT *pPrt = pModel->FirstChildElement("Part");
			if (pPrt == nullptr)throw(std::logic_error("Model must have part element"));
			const Aris::Core::ELEMENT *pJnt = pModel->FirstChildElement("Joint");
			if (pJnt == nullptr)throw(std::logic_error("Model must have joint element"));
			const Aris::Core::ELEMENT *pMot = pModel->FirstChildElement("Motion");
			if (pMot == nullptr)throw(std::logic_error("Model must have motion element"));
			const Aris::Core::ELEMENT *pFce = pModel->FirstChildElement("Force");
			if (pFce == nullptr)throw(std::logic_error("Model must have force element"));

			calculator.ClearVariables();
			for (const Aris::Core::ELEMENT *ele = pVar->FirstChildElement();
				ele != nullptr;
				ele = ele->NextSiblingElement())
			{
				calculator.AddVariable(ele->Name(), calculator.CalculateExpression(ele->GetText()));
			}

			_Environment.FromXMLElement(pEnv);

			_parts.clear();
			_joints.clear();
			_motions.clear();
			_forces.clear();

			/*读入地面*/
			for (const Aris::Core::ELEMENT *ele = pPrt->FirstChildElement();
				ele != nullptr;
				ele = ele->NextSiblingElement())
			{
				if (std::string(ele->Name()) == "Ground")
				{
					AddPart(ele->Name());
					GetPart(ele->Name())->FromXMLElement(ele);
					pGround = GetPart("Ground");
				}
			}

			if (this->GetPart("Ground") == nullptr)
			{
				throw std::logic_error("Model must contain a Ground");
			}

			/*读入其他部件*/
			for (const Aris::Core::ELEMENT *ele = pPrt->FirstChildElement();
				ele != nullptr; 
				ele = ele->NextSiblingElement())
			{
				if (std::string(ele->Name()) != "Ground")
				{
					AddPart(ele->Name());
					GetPart(ele->Name())->FromXMLElement(ele);
				}
			}

			for (const Aris::Core::ELEMENT *ele = pJnt->FirstChildElement();
				ele != nullptr;
				ele = ele->NextSiblingElement())
			{
				AddJoint(ele->Name());
				GetJoint(ele->Name())->FromXMLElement(ele);
				GetJoint(ele->Name())->_Initiate();
			}

			for (const Aris::Core::ELEMENT *ele = pMot->FirstChildElement();
				ele != nullptr;
				ele = ele->NextSiblingElement())
			{
				AddMotion(ele->Name());
				GetMotion(ele->Name())->FromXMLElement(ele);
				GetMotion(ele->Name())->_Initiate();
			}
		}
		void MODEL::SaveSnapshotXML(const char *filename) const
		{
			Aris::Core::DOCUMENT XML_Doc;
			XML_Doc.DeleteChildren();

			Aris::Core::DECLARATION *pHeader = XML_Doc.NewDeclaration("xml version=\"1.0\" encoding=\"UTF-8\" ");
			XML_Doc.InsertFirstChild(pHeader);

			Aris::Core::ELEMENT *pModel = XML_Doc.NewElement("Model");
			XML_Doc.InsertEndChild(pModel);

			Aris::Core::ELEMENT *pEnvironment = XML_Doc.NewElement("");
			_Environment.ToXMLElement(pEnvironment);
			pModel->InsertEndChild(pEnvironment);

			Aris::Core::ELEMENT *pVar = XML_Doc.NewElement("Variable");
			pModel->InsertEndChild(pVar);

			Aris::Core::ELEMENT *pPrt = XML_Doc.NewElement("Part");
			pModel->InsertEndChild(pPrt);

			Aris::Core::ELEMENT *pJnt = XML_Doc.NewElement("Joint");
			pModel->InsertEndChild(pJnt);

			Aris::Core::ELEMENT *pMot = XML_Doc.NewElement("Motion");
			pModel->InsertEndChild(pMot);

			Aris::Core::ELEMENT *pFce = XML_Doc.NewElement("Force");
			pModel->InsertEndChild(pFce);

			for (auto &p:_parts)
			{
				Aris::Core::ELEMENT *ele = XML_Doc.NewElement("");
				p->ToXMLElement(ele);
				pPrt->InsertEndChild(ele);
			}

			for (auto &j : _joints)
			{
				Aris::Core::ELEMENT *ele = XML_Doc.NewElement("");
				j->ToXMLElement(ele);
				pJnt->InsertEndChild(ele);
			}

			for (auto &m : _motions)
			{
				Aris::Core::ELEMENT *ele = XML_Doc.NewElement("");
				m->ToXMLElement(ele);
				pMot->InsertEndChild(ele);
			}


			XML_Doc.SaveFile(filename);
		}
		void MODEL::SaveAdams(const char *filename) const
		{
			ofstream file;

			file.open(filename);

			file << setprecision(15);
			/*  Basic  */
			file << "!-------------------------- Default Units for Model ---------------------------!\r\n"
				<< "!\r\n"
				<< "!\r\n"
				<< "defaults units  &\r\n"
				<< "    length = meter  &\r\n"
				<< "    angle = rad  &\r\n"
				<< "    force = newton  &\r\n"
				<< "    mass = kg  &\r\n"
				<< "    time = sec\r\n"
				<< "!\n"
				<< "defaults units  &\r\n"
				<< "    coordinate_system_type = cartesian  &\r\n"
				<< "    orientation_type = body313\r\n"
				<< "!\r\n"
				<< "!------------------------ Default Attributes for Model ------------------------!\r\n"
				<< "!\r\n"
				<< "!\r\n"
				<< "defaults attributes  &\r\n"
				<< "    inheritance = bottom_up  &\r\n"
				<< "    icon_visibility = off  &\r\n"
				<< "    grid_visibility = off  &\r\n"
				<< "    size_of_icons = 5.0E-002  &\r\n"
				<< "    spacing_for_grid = 1.0\r\n"
				<< "!\r\n"
				<< "!------------------------------ Adams/View Model ------------------------------!\r\n"
				<< "!\r\n"
				<< "!\r\n"
				<< "model create  &\r\n"
				<< "   model_name = " << this->GetName() << "\r\n"
				<< "!\r\n"
				<< "view erase\r\n"
				<< "!\r\n"
				<< "!---------------------------------- Accgrav -----------------------------------!\r\n"
				<< "!\r\n"
				<< "!\r\n"
				<< "force create body gravitational  &\r\n"
				<< "    gravity_field_name = gravity  &\r\n"
				<< "    x_component_gravity = 0.0  &\r\n"
				<< "    y_component_gravity = -9.8  &\r\n"
				<< "    z_component_gravity = 0.0\r\n"
				<< "!\r\n";

			/*  Create Parts  */
			file << "!-------------------------------- Rigid Parts ---------------------------------!\r\n"
				<< "!\r\n"
				<< "! Create parts and their dependent markers and graphics\r\n"
				<< "!\r\n"
				<< "!----------------------------------- ground -----------------------------------!\r\n"
				<< "!\r\n"
				<< "!\r\n"
				<< "! ****** Ground Part ******\r\n"
				<< "!\r\n"
				<< "defaults model  &\r\n"
				<< "    part_name = ground\r\n"
				<< "!\r\n"
				<< "defaults coordinate_system  &\r\n"
				<< "    default_coordinate_system = ." << this->GetName() << ".ground\r\n"
				<< "!\r\n"
				<< "! ****** Markers for current part ******\r\n"
				<< "!\r\n";

			for (auto &i : pGround->_markerNames)
			{
				double ep[6];

				s_pm2ep(_markers.at(i.second)->GetPrtPmPtr(), ep, "313");
				MATRIX ori(1,3,&ep[0]),loc(1, 3, &ep[3]);
				
				file << "marker create  &\r\n"
					<< "    marker_name = ." << this->GetName() << ".ground." << _markers.at(i.second)->GetName() << "  &\r\n"
					<< "    adams_id = " << i.second << "  &\r\n"
					<< "    location = (" << loc.ToString() << ")  &\r\n"
					<< "    orientation = (" << ori.ToString() << ") \r\n"
					<< "!\r\n";
			}
			
			for (auto &i : _parts)
			{
				if (i.get() == pGround)
					continue;

				double ep[6];

				s_pm2ep(i->GetPmPtr(), ep, "313");
				MATRIX ori(1, 3, &ep[0]), loc(1, 3, &ep[3]);

				file << "!----------------------------------- " << i->GetName() << " -----------------------------------!\r\n"
					<< "!\r\n"
					<< "!\r\n"
					<< "defaults coordinate_system  &\r\n"
					<< "    default_coordinate_system = ." << GetName() << ".ground\r\n"
					<< "!\r\n"
					<< "part create rigid_body name_and_position  &\r\n"
					<< "    part_name = ." << GetName() << "." << i->GetName() << "  &\r\n"
					<< "    adams_id = " << i->GetID()+1 << "  &\r\n"
					<< "    location = (" << loc.ToString() << ")  &\r\n"
					<< "    orientation = (" << ori.ToString() << ")\r\n"
					<< "!\r\n"
					<< "defaults coordinate_system  &\r\n"
					<< "    default_coordinate_system = ." << GetName() << "." << i->GetName() << " \r\n"
					<< "!\r\n";

				file << "! ****** Markers for current part ******\r\n"
					<< "marker create  &\r\n"
					<< "    marker_name = ." << GetName() << "." << i->GetName() << ".cm  &\r\n"
					<< "    adams_id = " << i->GetID() + _markers.size() << "  &\r\n"
					<< "    location = ({" << i->GetPrtImPtr()[11] << "," << -i->GetPrtImPtr()[5] << "," << -i->GetPrtImPtr()[4]<< "})  &\r\n"
					<< "    orientation = (" << "{0,0,0}" << ")\r\n"
					<< "!\r\n";

				for (auto &j : i->_markerNames)
				{
					double ep[6];

					s_pm2ep(_markers.at(j.second)->GetPrtPmPtr(), ep, "313");
					MATRIX ori(1, 3, &ep[0]), loc(1, 3, &ep[3]);
					
					file << "marker create  &\r\n"
						<< "marker_name = ." << GetName() << "." << i->GetName() << "." << _markers.at(j.second)->GetName() <<"  &\r\n"
						<< "adams_id = " << j.second << "  &\r\n"
						<< "location = (" << loc.ToString() << ")  &\r\n"
						<< "orientation = (" << ori.ToString() << ")\r\n"
						<< "!\r\n";
				}


				file << "part create rigid_body mass_properties  &\r\n"
					<< "    part_name = ." << GetName() << "." << i->GetName() << "  &\r\n"
					<< "    mass = " << i->GetPrtImPtr()[0] << "  &\r\n"
					<< "    center_of_mass_marker = ." << GetName() << "." << i->GetName() << ".cm  &\r\n"
					<< "    inertia_marker = ." << GetName() << "." << i->GetName() << ".cm  &\r\n"
					<< "    ixx = " << i->GetPrtImPtr()[21] << "  &\r\n"
					<< "    iyy = " << i->GetPrtImPtr()[28] << "  &\r\n"
					<< "    izz = " << i->GetPrtImPtr()[35] << "  &\r\n"
					<< "    ixy = " << i->GetPrtImPtr()[27] << "  &\r\n"
					<< "    izx = " << i->GetPrtImPtr()[33] << "  &\r\n"
					<< "    iyz = " << i->GetPrtImPtr()[34] << "\r\n"
					<< "!\r\n";

				std::stringstream stream(i->graphicFilePath);

				string path;
				while (stream >> path)
				{
					file << "file parasolid read &\r\n"
						<< "file_name = \"" << path << "\" &\r\n"
						<< "type = ASCII" << " &\r\n"
						<< "part_name = " << i->GetName() << " \r\n"
						<< "\r\n";
				}


			}

			file << "!----------------------------------- Joints -----------------------------------!\r\n"
				<< "!\r\n"
				<< "!\r\n";

			for (auto &i : _joints)
			{
				std::string s;
				double ep[6] = { PI/2, 0, 0, 0, 0, 0 };
				double ep2[6] = { -PI/2, 0, 0, 0, 0, 0 };
				double pm[4][4], pm2[4][4];
				switch (i->GetType())
				{
				case JOINT::ROTATIONAL:
					s = "revolutional";
					break;
				case JOINT::PRISMATIC:
					s = "translational";
					break;
				case JOINT::UNIVERSAL:
					s = "universal";

					s_ep2pm(ep, *pm, "213");
					s_pm_dot_pm(i->_pMakI->GetPrtPmPtr(), *pm, *pm2);
					s_pm2ep(*pm2, ep, "313");

					file << "marker modify &\r\n"
						<< "    marker_name = ." << GetName() << "." << i->GetMakI()->GetFatherPrt()->GetName() << "." << i->GetMakI()->GetName() << " &\r\n"
						<< "    orientation = (" << MATRIX(1, 3, ep).ToString() << ") \r\n"
						<< "!\r\n";

					s_ep2pm(ep2, *pm, "123");
					s_pm_dot_pm(i->_pMakJ->GetPrtPmPtr(), *pm, *pm2);
					s_pm2ep(*pm2, ep, "313");

					file << "marker modify &\r\n"
						<< "    marker_name = ." << GetName() << "." << i->GetMakJ()->GetFatherPrt()->GetName() << "." << i->GetMakJ()->GetName() << " &\r\n"
						<< "    orientation = (" << MATRIX(1, 3, ep).ToString() << ") \r\n"
						<< "!\r\n";

					break;
				case JOINT::SPHERICAL:
					s = "spherical";
					break;
				}

				file << "constraint create joint " << s << "  &\r\n"
					<< "    joint_name = ." << GetName() << "." << i->GetName() << "  &\r\n"
					<< "    adams_id = " << i->GetID() << "  &\r\n"
					<< "    i_marker_name = ." << GetName() << "." << i->_pMakI->_pPrt->GetName() << "." << i->_pMakI->GetName() << "  &\r\n"
					<< "    j_marker_name = ." << GetName() << "." << i->_pMakJ->_pPrt->GetName() << "." << i->_pMakJ->GetName() << "  \r\n"
					<< "!\r\n";

				if (i->GetActive() == false)
				{
					file << "constraint attributes  &\r\n"
						<< "constraint_name = ." << GetName() << "." << i->GetName() << "  &\r\n"
						<< "active = off \r\n"
						<< "!\r\n";
				}
			}

			for (auto &i : _motions)
			{
				std::string s;
				switch (i->GetType())
				{
				case MOTION::LINEAR:
					s = "z";
					break;
				}

				file << "constraint create motion_generator &\r\n"
					<< "    motion_name = ." << GetName() << "." << i->GetName() << "  &\r\n"
					<< "    adams_id = " << i->GetID() << "  &\r\n"
					<< "    i_marker_name = ." << GetName() << "." << i->_pMakI->_pPrt->GetName() << "." << i->_pMakI->GetName() << "  &\r\n"
					<< "    j_marker_name = ." << GetName() << "." << i->_pMakJ->_pPrt->GetName() << "." << i->_pMakJ->GetName() << "  &\r\n"
					<< "    axis = "<< s <<"  &\r\n"
					<< "    function = \"" << *i->GetP_mPtr() <<"\"  \r\n"
					<< "!\r\n";
				
				std::string type= "translational";

				file << "force create direct single_component_force  &\r\n"
					<< "    single_component_force_name = ." << GetName() << "." << i->GetName() << "_fce  &\r\n"
					<< "    adams_id = " << i->GetID() << "  &\r\n"
					<< "    type_of_freedom = " << type << "  &\r\n"
					<< "    i_marker_name = ." << GetName() << "." << i->_pMakI->_pPrt->GetName() << "." << i->_pMakI->GetName() << "  &\r\n"
					<< "    j_marker_name = ." << GetName() << "." << i->_pMakJ->_pPrt->GetName() << "." << i->_pMakJ->GetName() << "  &\r\n"
					<< "    action_only = off  &\r\n"
					<< "    function = \"" << *i->GetF_mPtr() << "\"  \r\n"
					<< "!\r\n";

				if (i->GetMode()==MOTION::FCE_CONTROL)
				{
					file << "constraint attributes  &\r\n"
						<< "constraint_name = ." << GetName() << "." << i->GetName() << "  &\r\n"
						<< "active = off \r\n"
						<< "!\r\n";
				}
				else
				{
					file << "force attributes  &\r\n"
						<< "force_name = ." << GetName() << "." << i->GetName() << "_fce  &\r\n"
						<< "active = off \r\n"
						<< "!\r\n";
				}


					
			}
		}
	}
}
