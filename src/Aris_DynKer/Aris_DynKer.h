/*!
* \file Aris_DynKer.h
* \brief 概述
* \author 潘阳
* \version 1.0.0.0
* \date 2014/4/5
*/

#ifndef Aris_DynKer_H
#define Aris_DynKer_H

#ifndef PI
#define PI 3.141592653589793
#endif

#include <vector>
#include <map>
#include <string>
#include <memory>
#include <functional>
#include <algorithm>

namespace Aris
{
	namespace DynKer
	{
		void dsp(const double *p, const int m, const int n, const int begin_row = 0, const int begin_col = 0, int ld = 0);
		void dlmwrite(const char *FileName, const double *pMatrix, const int m, const int n);
		void dlmread(const char *FileName, double *pMatrix);
		
		template <typename T> inline int s_sgn(T val) {
			return (T(0) < val) - (val < T(0));
		}
		template <typename T> inline int s_sgn2(T val) {
			return val < T(0) ? -1 : 1;
		}

		/** \brief 计算三维向量叉乘
		*
		* 用来计算：vec_out = cro_vec_in x vec_in
		*
		*/
		void s_cro3(const double *cro_vec_in, const double *vec_in, double *vec_out) noexcept;
		/** \brief 计算三维向量叉乘
		*
		* 用来计算：vec_out = alpha * cro_vec_in x vec_in + beta * vec_out
		*
		*/
		void s_cro3(double alpha, const double *cro_vec_in, const double *vec_in, double beta, double *vec_out) noexcept;
		/** \brief 计算三维向量叉乘矩阵
		*
		* 用来计算：cm_out =
		* [ 0  -z   y
		*   z   0  -x
		*  -y   x   0 ]
		* 
		*/
		void s_cm3(const double *cro_vec_in, double *cm_out) noexcept;

		/** \brief 计算位姿矩阵的逆矩阵
		*
		* 用来计算：pm_out = pm_in^-1
		*
		*/
		void s_inv_pm(const double *pm_in, double *pm_out) noexcept;
		/** \brief 根据原点和两个坐标轴上的点来求位姿矩阵
		*
		* 这里原点origin为位姿矩阵pm_out的点，firstAxisPnt位于第一根坐标轴，secondAxisPnt位于第一根坐标轴和第二根坐标轴所构成的平面内
		*
		*/
		void s_axes2pm(const double *origin, const double *firstAxisPnt, const double *secondAxisPnt, double *pm_out, const char *axesOrder = "xy") noexcept;
		/** \brief 将一种形式的欧拉角转换到另一种形式下
		*
		* 例如可以将313的欧拉角转换到321的欧拉角
		*
		*/
		void s_pe2pe(const char* type1_in, const double *pe_in, const char* type2_in, double *pe_out) noexcept;
		/** \brief 将位姿矩阵转化成欧拉角
		*
		* 
		*
		*/
		void s_pm2pe(const double *pm_in, double *pe_out, const char *EurType="313") noexcept;
		/** \brief 将欧拉角转化成位姿矩阵
		*
		*
		*
		*/
		void s_pe2pm(const double *pe_in, double *pm_out, const char *EurType="313") noexcept;
		/** \brief 将位姿矩阵转化成欧拉角
		*
		*
		*
		*/
		void s_pq2pe(const double *pq_in, double *pe_out, const char *EurType = "313") noexcept;
		/** \brief 将位置和欧拉角转化成位置和四元数
		*
		*
		*
		*/
		void s_pe2pq(const double *pe_in, double *pq_out, const char *EurType = "313") noexcept;
		void s_pm2pr(const double *pm_in, double *pr_out) noexcept;
		void s_pr2pm(const double *pr_in, double *pm_out) noexcept;
		void s_pm2pq(const double *pm_in, double *qp_out) noexcept;
		void s_pq2pm(const double *pq_in, double *pm_out) noexcept;
		void s_vq2v(const double *pq_in, const double *vq_in, double *v_out) noexcept;
		void s_v2vq(const double *pm_in, const double *v_in, double *vq_out) noexcept;
		void s_tmf(const double *pm_in, double *tmf_out) noexcept;
		void s_tmv(const double *pm_in, double *tmv_out) noexcept;
		/** \brief 根据位姿矩阵转换六维力向量
		*
		* 等同于： vec_out = tmf(pm_in) * fce_in
		*
		*/
		void s_tf(const double *pm_in, const double *fce_in, double *vec_out) noexcept;
		/** \brief 根据位姿矩阵转换六维力向量
		*
		* 等同于： vec_out = alpha * tmf(pm_in) * fce_in + beta * vec_out
		*
		*/
		void s_tf(double alpha, const double *pm_in, const double *fce_in, double beta, double *vec_out) noexcept;
		/** \brief 根据位姿矩阵转换六维力矩阵
		*
		* 等同于： m_out = tmf(pm_in) * fces_in
		*
		*/
		void s_tf_n(int n, const double *pm_in, const double *fces_in, double *m_out) noexcept;
		/** \brief 根据位姿矩阵转换六维力矩阵
		*
		* 等同于： m_out = alpha * tmf(pm_in) * fces_in + beta * m_out
		*
		*/
		void s_tf_n(int n, double alpha, const double *pm_in, const double *fces_in, double beta, double *m_out) noexcept;
		/** \brief 根据位姿矩阵的逆矩阵转换六维力向量
		*
		* 等同于： vec_out = tmv(pm_in^-1) * fce_in
		*
		*/
		void s_inv_tf(const double *inv_pm_in, const double *fce_in, double *vec_out) noexcept;
		/** \brief 根据位姿矩阵的逆矩阵转换六维速度向量
		*
		* 等同于： vec_out = alpha * tmf(pm_in^-1) * vel_in + beta * vec_out
		*
		*/
		void s_inv_tf(double alpha, const double *inv_pm_in, const double *vel_in, double beta, double *vec_out) noexcept;
		/** \brief 根据位姿矩阵转换六维速度向量
		*
		* 等同于： vec_out = tmv(pm_in) * vel_in
		*
		*/
		void s_tv(const double *pm_in, const double *vel_in, double *vec_out) noexcept;
		/** \brief 根据位姿矩阵转换六维速度向量
		*
		* 等同于： vec_out = alpha * tmv(pm_in) * vel_in + beta * vec_out
		*
		*/
		void s_tv(double alpha, const double *pm_in, const double *vel_in, double beta, double *vec_out) noexcept;
		/** \brief 根据位姿矩阵转换六维力矩阵
		*
		* 等同于： m_out = tmv(pm_in) * fces_in
		*
		*/
		void s_tv_n(int n, const double *pm_in, const double *vels_in, double *m_out) noexcept;
		/** \brief 根据位姿矩阵转换六维力矩阵
		*
		* 等同于： m_out = alpha * tmv(pm_in) * fces_in + beta * m_out
		*
		*/
		void s_tv_n(int n, double alpha, const double *pm_in, const double *vels_in, double beta, double *m_out) noexcept;
		/** \brief 根据位姿矩阵的逆矩阵转换六维速度向量
		*
		* 等同于： vec_out = tmv(pm_in^-1) * vel_in
		*
		*/
		void s_inv_tv(const double *inv_pm_in, const double *vel_in, double *vec_out) noexcept;
		/** \brief 根据位姿矩阵的逆矩阵转换六维速度向量
		*
		* 等同于： vec_out = alpha * tmv(pm_in^-1) * vel_in + beta * vec_out
		*
		*/
		void s_inv_tv(double alpha, const double *inv_pm_in, const double *vel_in, double beta, double *vec_out) noexcept;
		/** \brief 根据位姿矩阵的逆矩阵转换六维速度向量
		*
		* 等同于： vec_out = tmv(pm_in^-1) * vel_in
		*
		*/
		void s_inv_tv_n(int n, const double *inv_pm_in, const double *vel_in, double *vec_out) noexcept;
		/** \brief 根据位姿矩阵的逆矩阵转换六维速度向量
		*
		* 等同于： vec_out = alpha * tmv(pm_in^-1) * vel_in + beta * vec_out
		*
		*/
		void s_inv_tv_n(int n, double alpha, const double *inv_pm_in, const double *vel_in, double beta, double *vec_out) noexcept;
		void s_cmf(const double *vel_in, double *cm_out) noexcept;
		void s_cmv(const double *vel_in, double *cmd_out) noexcept;
		/** \brief 计算六维向量叉乘
		*
		* 用来计算：vec_out = cro_vec_in xf vec_in
		*
		*/
		void s_cf(const double *cro_vel_in, const double *vec_in, double* vec_out) noexcept;
		/** \brief 计算六维向量叉乘
		*
		* 用来计算：vec_out = alpha * cro_vec_in xf vec_in + beta * vec_out
		*
		*/
		void s_cf(double alpha, const double *cro_vel_in, const double *vec_in, double beta, double* vec_out) noexcept;
		/** \brief 计算六维向量叉乘
		*
		* 用来计算：vec_out = cro_vec_in xv vec_in
		*
		*/
		void s_cv(const double *cro_vel_in, const double *vec_in, double* vec_out) noexcept;
		/** \brief 计算六维向量叉乘
		*
		* 用来计算：vec_out = alpha * cro_vec_in xv vec_in + beta * vec_out
		*
		*/
		void s_cv(double alpha, const double *cro_vel_in, const double *vec_in, double beta, double* vec_out) noexcept;
		void s_i2i(const double *from_pm_in, const double *from_im_in, double *to_im_out) noexcept;
		/** \brief 计算点加速度
		*
		* 用来将6维空间速度从一个坐标系中转化到另一个坐标系中。例如将B坐标系中的速度转换到A坐标系中。
		*
		* \param relative_pm_in 表示两个坐标系之间的位姿矩阵。即B相对于A的位姿矩阵。
		* \param relative_vel_in 表示两个坐标系之间的相对速度。即B相对于A的空间速度向量，这个向量在坐标系A中表达。
		* \param from_vel_in 表示坐标系B中待转换的速度向量。
		* \param to_vel_out 表示转换完成的速度向量，即A坐标系中的速度向量。
		*/
		void s_v2v(const double *relative_pm_in, const double *relative_vel_in, const double *from_vel_in, double *to_vel_out) noexcept;
		void s_inv_v2v(const double *inv_relative_pm_in, const double *inv_relative_vel_in, const double *from_vel_in, double *to_vel_out) noexcept;
		void s_a2a(const double *relative_pm_in, const double *relative_vel_in, const double *relative_acc_in,
			const double *from_vel_in, const double *from_acc_in, double *to_acc_out, double *to_vel_out = 0) noexcept;
		void s_pnt2pnt(const double *relative_pm_in, const double *from_pnt, double *to_pnt_out) noexcept;
		/** \brief 计算点加速度
		*
		* 转化点速度的坐标系。
		*
		*/
		void s_pv2pv(const double *relative_pm_in, const double *relative_vel_in, 
			const double *from_pnt, const double *from_pv, double *to_pv_out, double *to_pnt_out = 0) noexcept;
		void s_inv_pv2pv(const double *inv_relative_pm_in, const double *inv_relative_vel_in,
			const double *from_pnt, const double *from_pv, double *to_pv_out, double *to_pnt_out = 0) noexcept;
		void s_pa2pa(const double *relative_pm_in, const double *relative_vel_in, const double *inv_relative_acc_in,
			const double *from_pnt, const double *from_pv, const double *from_pa,
			double *to_pa_out, double *to_pv_out = 0, double *to_pnt_out = 0) noexcept;
		void s_inv_pa2pa(const double *inv_relative_pm_in, const double *inv_relative_vel_in, const double *inv_relative_acc_in,
			const double *from_pnt, const double *from_pv, const double *from_pa, 
			double *to_pa_out, double *to_pv_out = 0, double *to_pnt_out = 0) noexcept;
		void s_mass2im(const double mass_in, const double * inertia_in, const double *pm_in, double *im_out) noexcept;
		void s_gamma2im(const double * gamma_in, double *im_out) noexcept;
		void s_im2gamma(const double * im_in, double *gamma_out) noexcept;

		void s_pv(const double *pnt_in, const double *vel_in, double *pv_out) noexcept;
		/** \brief 计算点加速度
		*
		* 根据数据点位置、空间速度和空间加速度来计算该点的加速度。
		*
		*/
		void s_pa(const double *pnt_in, const double *vel_in, const double *acc_in, double *pnt_acc_out) noexcept;

		void s_block_cpy(const int &block_size_m, const int &block_size_n,
			const double *from_mtrx, const int &fm_begin_row, const int &fm_begin_col, const int &fm_ld,
			double *to_mtrx, const int &tm_begin_row, const int &tm_begin_col, const int &tm_ld) noexcept;
		void s_block_cpy(const int &block_size_m, const int &block_size_n,
			double alpha, const double *from_mtrx, const int &fm_begin_row, const int &fm_begin_col, const int &fm_ld,
			double beta, double *to_mtrx, const int &tm_begin_row, const int &tm_begin_col, const int &tm_ld) noexcept;
		void s_block_cpyT(const int &block_size_m, const int &block_size_n,
			const double *from_mtrx, const int &fm_begin_row, const int &fm_begin_col, const int &fm_ld,
			double *to_mtrx, const int &tm_begin_row, const int &tm_begin_col, const int &tm_ld) noexcept;
		void s_block_cpyT(const int &block_size_m, const int &block_size_n,
			double alpha, const double *from_mtrx, const int &fm_begin_row, const int &fm_begin_col, const int &fm_ld,
			double beta, double *to_mtrx, const int &tm_begin_row, const int &tm_begin_col, const int &tm_ld) noexcept;

		void s_dlt_col(const int &dlt_col_num, const int *col_index, const int &m, const int &n, double *A, const int &ldA) noexcept;

		void s_pm_dot_pm(const double *pm1_in, const double *pm2_in, double *pm_out) noexcept;
		template <typename ...Args>
		void s_pm_dot_pm(const double *pm1, const double *pm2, Args ...args) noexcept
		{
			double pm[16];
			s_pm_dot_pm(pm1, pm2, pm);
			s_pm_dot_pm(pm, args...);
		}
		void s_inv_pm_dot_pm(const double *inv_pm1_in, const double *pm2_in, double *pm_out) noexcept;
		void s_pm_dot_inv_pm(const double *inv_pm1_in, const double *pm2_in, double *pm_out) noexcept;
		void s_pm_dot_pnt(const double *pm_in, const double *pos_in, double *pos_out) noexcept;
		void s_inv_pm_dot_pnt(const double *pm_in, const double *pos_in, double *pos_out) noexcept;
		void s_pm_dot_v3(const double *pm_in, const double *v3_in, double *v3_out) noexcept;
		void s_inv_pm_dot_v3(const double *inv_pm_in, const double *v3_in, double *v3_out) noexcept;
		void s_m6_dot_v6(const double *m6_in, const double *v6_in, double *v6_out) noexcept;

		void s_vn_add_vn(int N, const double *v1_in, const double *v2_in, double *v_out) noexcept;
		double s_vn_dot_vn(int N, const double *v1_in, const double *v2_in) noexcept;

		void s_v_cro_pm(const double *v_in, const double *pm_in, double *vpm_out) noexcept;

		void s_dscal(const int n, const double a, double *x, const int incx) noexcept;
		double s_dnrm2(const int n, const double *x, const int incx) noexcept;
		void s_daxpy(const int N, const double alpha, const double *X, const int incX, double *Y, const int incY) noexcept;
		void s_swap(const int N, double *X, const int incX, double *Y, const int incY) noexcept;
		void s_transpose(const int m, const int n, const double *A, const int ldA, double *B_out, const int ldB) noexcept;

		
		void s_dgemm(int m, int n, int k, double alpha, const double* A, int lda, const double* B, int ldb, double beta, double *C, int ldc) noexcept;
		void s_dgemmTN(int m, int n, int k, double alpha, const double* A, int lda, const double* B, int ldb, double beta, double *C, int ldc) noexcept;
		void s_dgemmNT(int m, int n, int k, double alpha, const double* A, int lda, const double* B, int ldb, double beta, double *C, int ldc) noexcept;
		
		//void s_dgeinv(const int n, double* A, const int lda, int *ipiv) noexcept;
		//void s_dgesv(int n, int nrhs, double* a, int lda, int* ipiv, double* b, int ldb) noexcept;
		//void s_dgesvT(int n, int nrhs, double* a, int lda, int* ipiv, double* b, int ldb) noexcept;
		//void s_dgelsd(int m, int n, int nrhs, double* a, int lda, double* b, int ldb, double* s, double rcond, int* rank) noexcept;
		//void s_dgelsdT(int m, int n, int nrhs, double* a, int lda, double* b, int ldb, double* s, double rcond, int* rank) noexcept;


		class AKIMA
		{
		public:
			AKIMA(int inNum, const double *x_in, const double *y_in);
			AKIMA(const AKIMA &) = default;
			~AKIMA() = default;
			AKIMA &operator =(const AKIMA &) = default;

			double operator()(double x, char derivativeOrder = '0') const;
			void operator()(int length, const double *x_in, double *y_out, char derivativeOrder = '0') const;

			const std::vector<double> &x() const { return _x; };
			const std::vector<double> &y() const { return _y; };
		private:
			std::vector<double> _x, _y;
			std::vector<double> _p0;
			std::vector<double> _p1;
			std::vector<double> _p2;
			std::vector<double> _p3;
		};
	}
}




























#endif
