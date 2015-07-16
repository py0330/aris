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

			/*不确定为啥这里不用查询work的size,大概因为这个函数不需要work的buffer吧*/
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
		void s_pm2pe(const double *pm_in, double *pe_out, const char *EurType)
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
			memcpy(&pe_out[3], phi, sizeof(phi));
			pe_out[0] = pm_in[3];
			pe_out[1] = pm_in[7];
			pe_out[2] = pm_in[11];

		}
		void s_pe2pm(const double *pe_in, double *pm_out, const char *EurType)
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


			pm_out[3] = pe_in[0];
			pm_out[7] = pe_in[1];
			pm_out[11] = pe_in[2];
			pm_out[15] = 1;
		}
		void s_pm2pr(const double *pm_in, double *pr_out)
		{
			double c_theta = (pm_in[0] + pm_in[5] + pm_in[10]-1)/2;
			double theta;

			const double &r11 = pm_in[0];
			const double &r12 = pm_in[1];
			const double &r13 = pm_in[2];
			const double &r21 = pm_in[4];
			const double &r22 = pm_in[5];
			const double &r23 = pm_in[6];
			const double &r31 = pm_in[8];
			const double &r32 = pm_in[9];
			const double &r33 = pm_in[10];

			double &x = pr_out[0];
			double &y = pr_out[1];
			double &z = pr_out[2];
			double &a = pr_out[3];
			double &b = pr_out[4];
			double &c = pr_out[5];
			


			if (c_theta > 0.7071)
			{
				theta = asin(sqrt(((r32 - r23)*(r32 - r23) + (r13 - r31)*(r13 - r31) + (r21 - r12)*(r21 - r12))/4));
			}
			else if (c_theta > -0.7071)
			{
				theta = acos(c_theta);
			}
			else
			{
				theta = PI - asin(sqrt(((r32 - r23)*(r32 - r23) + (r13 - r31)*(r13 - r31) + (r21 - r12)*(r21 - r12)) / 4));
			}

			if (theta < 1e-8)
			{
				a = 0.5*(r32 - r23);
				b = 0.5*(r13 - r31);
				c = 0.5*(r21 - r12);
			}
			else if (theta < 0.785398163397448)
			{
				double  ratio = theta / sin(theta);
				
				a = 0.5*(r32 - r23)*ratio;
				b = 0.5*(r13 - r31)*ratio;
				c = 0.5*(r21 - r12)*ratio;
			}
			else
			{
				int a_sig,b_sig,c_sig;
				
				/*以下判断符号*/
				if (abs(r32 - r23) > abs(r13 - r31))
				{
					if (abs(r32 - r23) > abs(r21 - r12))
					{
						//a'的绝对值最大
						a_sig = s_sgn(r32 - r23);
						b_sig = s_sgn(r21 + r12)*a_sig;
						c_sig = s_sgn(r31 + r13)*a_sig;
					}
					else
					{
						//c'的绝对值最大
						c_sig = s_sgn(r21 - r12);
						a_sig = s_sgn(r31 + r13)*c_sig;
						b_sig = s_sgn(r32 + r23)*c_sig;
					}
				}
				else
				{
					if (abs(r13 - r31) > abs(r21 - r12))
					{
						//b'的绝对值最大
						b_sig = s_sgn(r13 - r31);
						a_sig = s_sgn(r21 + r12)*b_sig;
						c_sig = s_sgn(r32 + r23)*b_sig;
					}
					else
					{
						//c'的绝对值最大
						c_sig = s_sgn(r21 - r12);
						a_sig = s_sgn(r31 + r13)*c_sig;
						b_sig = s_sgn(r32 + r23)*c_sig;
					}
				}

				a = a_sig * theta*sqrt(abs(1 + r11 - r22 - r33) / 2.0 / (1 - cos(theta)));
				b = b_sig * theta*sqrt(abs(1 + r22 - r11 - r33) / 2.0 / (1 - cos(theta)));
				c = c_sig * theta*sqrt(abs(1 + r33 - r11 - r22) / 2.0 / (1 - cos(theta)));
			}
			                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  
			x = pm_in[3];
			y = pm_in[7];
			z = pm_in[11];
		}
		void s_pr2pm(const double *pr_in, double *pm_out)
		{
			double theta = sqrt(pr_in[3] * pr_in[3] + pr_in[4] * pr_in[4] + pr_in[5] * pr_in[5]);

			double A, B;

			const double &x = pr_in[0];
			const double &y = pr_in[1];
			const double &z = pr_in[2];
			const double &a = pr_in[3];
			const double &b = pr_in[4];
			const double &c = pr_in[5];
			

			if (theta < 1e-8)
			{
				A = 1;
				B = 0.5;
			}
			else
			{
				A = sin(theta)/theta;
				B = (1-cos(theta))/theta/theta;
			}

			memset(pm_out, 0, sizeof(double) * 16);
			
			pm_out[0] = 1 - (b*b + c*c)*B;
			pm_out[1] = -c*A + a*b*B;
			pm_out[2] = b*A + a*c*B;
			pm_out[3] = x;

			pm_out[4] = c*A + a*b*B;
			pm_out[5] = 1 - (a*a + c*c)*B;
			pm_out[6] = -a*A + b*c*B;
			pm_out[7] = y;

			pm_out[8] = -b*A + a*c*B;
			pm_out[9] = a*A + b*c*B;
			pm_out[10] = 1 - (a*a + b*b)*B;
			pm_out[11] = z;

			pm_out[15] = 1;
		}
		void s_pm2pq(const double *pm_in, double *pq_out)
		{
			const double &r11 = pm_in[0];
			const double &r12 = pm_in[1];
			const double &r13 = pm_in[2];
			const double &r21 = pm_in[4];
			const double &r22 = pm_in[5];
			const double &r23 = pm_in[6];
			const double &r31 = pm_in[8];
			const double &r32 = pm_in[9];
			const double &r33 = pm_in[10];
			
			double &x = pq_out[0];
			double &y = pq_out[1];
			double &z = pq_out[2];
			double &q1 = pq_out[3];
			double &q2 = pq_out[4];
			double &q3 = pq_out[5];
			double &q4 = pq_out[6];

			double *q = &pq_out[3];

			double tr = r11 + r22 + r33;

			/*因为无法确保在截断误差的情况下，tr+1一定为正，因此这里需要判断*/
			q4 = tr>-1 ? sqrt((tr + 1) / 4) : 0;

			if (q4 > 0.1)
			{
				q1 = (pm_in[9] - pm_in[6]) / q4 / 4;
				q2 = (pm_in[2] - pm_in[8]) / q4 / 4;
				q3 = (pm_in[4] - pm_in[1]) / q4 / 4;
			}
			else
			{
				unsigned id;
				
				q1 = sqrt((1 + pm_in[0] - pm_in[5] - pm_in[10])) / 2;
				q2 = sqrt((1 + pm_in[5] - pm_in[0] - pm_in[10])) / 2;
				q3 = sqrt((1 + pm_in[10] - pm_in[0] - pm_in[5])) / 2;

				/*qp_out 是通过开方计算而得，因此一定为正*/
				DynKer::s_max(q, 3, &id);

				q[id] *= s_sgn2(pm_in[(id + 2) % 3 * 4 + (id + 1) % 3] - pm_in[(id + 1) % 3 * 4 + (id + 2) % 3]);
				q[(id + 1) % 3] *= s_sgn2(q[id])*s_sgn2(pm_in[id * 4 + (id + 1) % 3] + pm_in[((id + 1) % 3) * 4 + id]);
				q[(id + 2) % 3] *= s_sgn2(q[id])*s_sgn2(pm_in[id * 4 + (id + 2) % 3] + pm_in[((id + 2) % 3) * 4 + id]);
			}

			x = pm_in[3];
			y = pm_in[7];
			z = pm_in[11];
		}
		void s_pq2pm(const double *pq_in, double *pm_out)
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
		void s_vq2v(const double *pq_in, const double *vq_in, double *v_out)
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

			v_out[3] = 2 * (p13 - p42)*pm[1][0]                    + 2 * (p23 + p41)*pm[1][1]                   - 4 * (q[0] * vq[0] + q[1] * vq[1])*pm[1][2];
			v_out[4] = -4 * (q[1] * vq[1] + q[2] * vq[2])*pm[2][0] + 2 * (p12 - p43)*pm[2][1]                   + 2 * (p13 + p42)*pm[2][2];
			v_out[5] = 2 * (p12 + p43)*pm[0][0]                    - 4 * (q[0] * vq[0] + q[2] * vq[2])*pm[0][1] + 2 * (p23 - p41)*pm[0][2];
		
			memcpy(&v_out[0], &vq_in[0], sizeof(double) * 3);
		}
		void s_v2vq(const double *pm_in, const double *v_in, double *vq_out)
		{
			double pq[7];
			s_pm2pq(pm_in, pq);

			double *q = &pq[3];
			double *vq = &vq_out[3];

			double vpm[4][4];
			s_v_cro_pm(v_in, pm_in, *vpm);

			unsigned i;
			s_max_abs(q, 4, &i);

			if (i == 3)
			{
				vq[3] = (vpm[0][0] + vpm[1][1] + vpm[2][2]) / 8 / q[3];

				vq[0] = (vpm[2][1] - vpm[1][2] - 4 * q[0] * vq[3]) / 4 / q[3];
				vq[1] = (vpm[0][2] - vpm[2][0] - 4 * q[1] * vq[3]) / 4 / q[3];
				vq[2] = (vpm[1][0] - vpm[0][1] - 4 * q[2] * vq[3]) / 4 / q[3];
			}
			else
			{
				unsigned j = (i + 1) % 3;
				unsigned k = (i + 1) % 3;
				
				vq[i] = (vpm[i][i] - vpm[j][j] - vpm[k][k]) / 8 * q[i];
				vq[j] = (vpm[j][i] + vpm[i][j] - 4 * q[j] * vq[i]) / 4 * q[i];
				vq[k] = (vpm[k][i] + vpm[i][k] - 4 * q[k] * vq[i]) / 4 * q[i];
				vq[3] = (vpm[k][j] - vpm[j][k] - 4 * q[4] * vq[i]) / 4 * q[i];
			}

			memcpy(&vq_out[0], &v_in[0], sizeof(double) * 3);
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
		void s_cmf(const double *vel_in, double *cmf_out)
		{
			memset(cmf_out, 0, sizeof(double) * 36);

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
		void s_cmv(const double *vel_in, double *cmv_out)
		{
			memset(cmv_out, 0, sizeof(double)* 36);

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
	
		void s_v_cro_pm(const double *v_in, const double *pm_in, double *vpm_out)
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
	}
}
