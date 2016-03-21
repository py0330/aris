#ifndef ARIS_DYNAMIC_KERNEL_
#define ARIS_DYNAMIC_KERNEL_

#ifndef PI
#define PI 3.141592653589793
#endif

#include <vector>
#include <map>
#include <string>
#include <memory>
#include <functional>
#include <algorithm>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <list>


namespace Aris
{
	///
	/// 符号定义: \n
	/// pm  :  4x4的位姿矩阵（pose matrix）\n
	/// pe  :  6x1的位姿向量，前3个元素为位置，后三个为欧拉角（position and eula angle）\n
	/// v   :  6x1的速度螺旋（velocity of a screw）\n
	/// a   :  6x1的加速度螺旋（acceleration of a screw）\n
	/// pq  :  7x1的位姿向量，前3个元素为位置，后4个位四元数（position and quaternion）\n
	/// vq  :  7x1的速度向量，前3个元素为速度螺旋的线速度部分，后4个为四元数的微分（velocity of quaternion)\n
	/// aq  :  7x1的加速度向量，前3个元素为加速度螺旋的线速度部分，后4个为四元数的二阶微分（acceleration of quaternion）\n
	/// pp  :  3x1的点位置（position of a point）\n
	/// vp  :  3x1的点速度（velocity of a point）\n
	/// ap  :  3x1的点加速度（acceleration of a point）\n
	/// eu  :  3x1的欧拉角（eula angle）\n
	/// va  :  3x1的角速度（velocity of angle）\n
	/// aa  :  3x1的角加速度（accleration of angle)\n
	/// apa :  6x1的加速度向量，前3个元素为点加速度，后三个元素为角加速度（acceleration of a point and angle）\n
	///
	
	
	
	
	/// \brief 动力学命名空间
	/// \ingroup Aris
	/// 
	///
	///
	namespace Dynamic
	{
		auto dsp(const double *p, const int m, const int n, const int begin_row = 0, const int begin_col = 0, int ld = 0)->void;
		template<class Container>
		auto dlmwrite(const char *filename, const Container &container)->void
		{
			std::ofstream file;

			file.open(filename);

			file << std::setprecision(15);

			for (auto i : container)
			{
				for (auto j : i)
				{
					file << j << "   ";
				}
				file << std::endl;
			}
		}
		auto dlmwrite(const char *filename, const double *mtx, const int m, const int n)->void;
		auto dlmread(const char *filename, double *mtx)->void;

		auto s_is_equal(int n, const double *v1, const double *v2, double error) noexcept->bool;

		template <typename T>  
		inline auto s_sgn(T val)->int  { return (T(0) < val) - (val < T(0)); }
		template <typename T> 
		inline auto s_sgn2(T val)->int { return val < T(0) ? -1 : 1; }

		

		/// \brief 计算三维向量叉乘
		///
		/// 用来计算：vec_out = cro_vec_in x vec_in
		///
		///
		auto s_cro3(const double *cro_vec_in, const double *vec_in, double *vec_out) noexcept->void;
		/** \brief 计算三维向量叉乘
		*
		* 用来计算：vec_out = alpha * cro_vec_in x vec_in + beta * vec_out
		*
		*/
		auto s_cro3(double alpha, const double *cro_vec_in, const double *vec_in, double beta, double *vec_out) noexcept->void;
		/** \brief 计算三维向量叉乘矩阵
		*
		* 用来计算：cm_out = \n
		* [ 0  -z   y \n
		*   z   0  -x \n
		*  -y   x   0 ]
		* 
		*/
		auto s_cm3(const double *cro_vec_in, double *cm_out) noexcept->void;

		/** \brief 将欧拉角转化成位姿矩阵
		*
		*
		*
		*/
		auto s_pe2pm(const double *pe_in, double *pm_out, const char *EurType = "313") noexcept->void;
		/** \brief 将位姿矩阵转化成欧拉角
		*
		* 
		*
		*/
		auto s_pm2pe(const double *pm_in, double *pe_out, const char *EurType = "313") noexcept->void;
		/** \brief 将位置和四元数转换为位姿矩阵
		*
		*
		*
		*/
		auto s_pq2pm(const double *pq_in, double *pm_out) noexcept->void;
		/** \brief 将位姿矩阵转换为位置和四元数
		*
		*
		*
		*/
		auto s_pm2pq(const double *pm_in, double *qp_out) noexcept->void;
		/** \brief 将位姿矩阵转化成欧拉角
		*
		*
		*
		*/
		auto s_pq2pe(const double *pq_in, double *pe_out, const char *EurType = "313") noexcept->void;
		/** \brief 将位置和欧拉角转化成位置和四元数
		*
		*
		*
		*/
		auto s_pe2pq(const double *pe_in, double *pq_out, const char *EurType = "313") noexcept->void;
		/** \brief 将一种形式的欧拉角转换到另一种形式下
		*
		* 例如可以将313的欧拉角转换到321的欧拉角
		*
		*/
		auto s_pe2pe(const char* type1_in, const double *pe_in, const char* type2_in, double *pe_out) noexcept->void;
		/** \brief 将螺旋线速度和四元数导数转换为螺旋线速度和角速度
		*
		*
		*
		*/
		auto s_vq2v(const double *pq_in, const double *vq_in, double *v_out) noexcept->void;
		/** \brief 将螺旋线速度和角速度转换为螺旋线速度和四元数导数
		*
		*
		*
		*/
		auto s_v2vq(const double *pm_in, const double *v_in, double *vq_out) noexcept->void;

		/** \brief 计算点速度
		*
		* 根据数据点位置、空间速度来计算该点的速度。
		*
		*/
		auto s_vp(const double *pnt_in, const double *vel_in, double *pv_out) noexcept->void;
		/** \brief 计算点加速度
		*
		* 根据数据点位置、空间速度和空间加速度来计算该点的加速度。
		*
		*/
		auto s_ap(const double *pnt_in, const double *vel_in, const double *acc_in, double *pnt_acc_out) noexcept->void;

		auto s_f2f(const double *relative_pm_in, const double *from_fce_in, double *to_fce_out) noexcept->void;
		/** \brief 将6维空间速度移动坐标系
		*
		* 用来将6维空间速度从一个坐标系中转化到另一个坐标系中。例如将B坐标系中的速度转换到A坐标系中。
		*
		* \param relative_pm_in 表示两个坐标系之间的位姿矩阵。即B相对于A的位姿矩阵。
		* \param relative_vel_in 表示两个坐标系之间的相对速度。即B相对于A的空间速度向量，这个向量在坐标系A中表达。
		* \param from_vel_in 表示坐标系B中待转换的速度向量。
		* \param to_vel_out 表示转换完成的速度向量，即A坐标系中的速度向量。
		*/
		auto s_v2v(const double *relative_pm_in, const double *relative_vel_in, const double *from_vel_in, double *to_vel_out) noexcept->void;
		/** \brief 将6维空间速度移动坐标系
		*
		* 用来将6维空间速度从一个坐标系中转化到另一个坐标系中。例如将B坐标系中的速度转换到A坐标系中。
		*
		* \param inv_relative_pm_in 表示两个坐标系之间的相反的位姿矩阵。即A相对于B的位姿矩阵。
		* \param inv_relative_vel_in 表示两个坐标系之间的相对速度。即A相对于B的空间速度向量，这个向量在坐标系B中表达。
		* \param from_vel_in 表示坐标系B中待转换的速度向量。
		* \param to_vel_out 表示转换完成的速度向量，即A坐标系中的速度向量。
		*/
		auto s_inv_v2v(const double *inv_relative_pm_in, const double *inv_relative_vel_in, const double *from_vel_in, double *to_vel_out) noexcept->void;
		/** \brief 将6维空间速度移动坐标系
		*
		* 用来将6维空间速度从一个坐标系中转化到另一个坐标系中。例如将B坐标系中的速度转换到A坐标系中。
		*
		* \param relative_pm_in 表示两个坐标系之间的位姿矩阵。即B相对于A的位姿矩阵。
		* \param relative_vel_in 表示两个坐标系之间的相对速度。即B相对于A的空间速度向量，这个向量在坐标系A中表达。
		* \param relative_acc_in 表示两个坐标系之间的相对加速度。即B相对于A的空间加速度向量，这个向量在坐标系A中表达。
		* \param from_vel_in 表示坐标系B中的速度向量。
		* \param from_acc_in 表示坐标系B中待转换的加速度向量。
		* \param to_vel_out 表示转换完成的加速度向量，即A坐标系中的加速度向量。
		* \param to_vel_out 表示转换完成的速度向量，即A坐标系中的速度向量。
		*/
		auto s_a2a(const double *relative_pm_in, const double *relative_vel_in, const double *relative_acc_in,
			const double *from_vel_in, const double *from_acc_in, double *to_acc_out, double *to_vel_out = nullptr) noexcept->void;
		/** \brief 将6维空间速度移动坐标系
		*
		* 用来将6维空间速度从一个坐标系中转化到另一个坐标系中。例如将B坐标系中的速度转换到A坐标系中。
		*
		* \param relative_pm_in 表示两个坐标系之间的位姿矩阵。即A相对于B的位姿矩阵。
		* \param relative_vel_in 表示两个坐标系之间的相对速度。即A相对于B的空间速度向量，这个向量在坐标系B中表达。
		* \param relative_acc_in 表示两个坐标系之间的相对加速度。即A相对于B的空间加速度向量，这个向量在坐标系B中表达。
		* \param from_vel_in 表示坐标系B中的速度向量。
		* \param from_acc_in 表示坐标系B中待转换的加速度向量。
		* \param to_vel_out 表示转换完成的加速度向量，即A坐标系中的加速度向量。
		* \param to_vel_out 表示转换完成的速度向量，即A坐标系中的速度向量。
		*/
		auto s_inv_a2a(const double *inv_relative_pm_in, const double *inv_relative_vel_in, const double *inv_relative_acc_in,
			const double *from_vel_in, const double *from_acc_in, double *to_acc_out, double *to_vel_out = nullptr) noexcept->void;
		/** \brief 转化点位置的坐标系
		*
		* 转化点位置的坐标系。
		*
		*/
		auto s_pp2pp(const double *relative_pm_in, const double *from_pnt, double *to_pnt_out) noexcept->void;
		/** \brief 转化点位置的坐标系
		*
		* 转化点位置的坐标系。
		*
		*/
		auto s_inv_pp2pp(const double *inv_relative_pm_in, const double *from_pnt, double *to_pnt_out) noexcept->void;
		/** \brief 转化点速度的坐标系
		*
		* 转化点速度的坐标系。
		*
		*/
		auto s_vp2vp(const double *relative_pm_in, const double *relative_vel_in,
			const double *from_pnt, const double *from_pv, double *to_pv_out, double *to_pnt_out = nullptr) noexcept->void;
		auto s_inv_vp2vp(const double *inv_relative_pm_in, const double *inv_relative_vel_in,
			const double *from_pnt, const double *from_pv, double *to_pv_out, double *to_pnt_out = nullptr) noexcept->void;
		auto s_ap2ap(const double *relative_pm_in, const double *relative_vel_in, const double *inv_relative_acc_in,
			const double *from_pnt, const double *from_pv, const double *from_ap,
			double *to_ap_out, double *to_pv_out = nullptr, double *to_pnt_out = nullptr) noexcept->void;
		auto s_inv_ap2ap(const double *inv_relative_pm_in, const double *inv_relative_vel_in, const double *inv_relative_acc_in,
			const double *from_pnt, const double *from_pv, const double *from_ap,
			double *to_ap_out, double *to_pv_out = nullptr, double *to_pnt_out = nullptr) noexcept->void;
		auto s_i2i(const double *from_pm_in, const double *from_im_in, double *to_im_out) noexcept->void;
		auto s_mass2im(const double mass_in, const double * inertia_in, const double *pm_in, double *im_out) noexcept->void;
		auto s_gamma2im(const double * gamma_in, double *im_out) noexcept->void;
		auto s_im2gamma(const double * im_in, double *gamma_out) noexcept->void;

		/** \brief 构造6x6的力转换矩阵
		*
		*  其中，tmf = [rm (3x3),  pp x rm (3x3); O (3x3), rm (3x3)]
		*
		*/
		auto s_tmf(const double *pm_in, double *tmf_out) noexcept->void;
		/** \brief 构造6x6的速度转换矩阵
		*
		*  其中，tmv = [rm (3x3),  O (3x3); pp x rm (3x3), rm (3x3)]
		*
		*/
		auto s_tmv(const double *pm_in, double *tmv_out) noexcept->void;
		/** \brief 根据位姿矩阵转换六维力向量
		*
		* 等同于： vec_out = tmf(pm_in) * fce_in
		*
		*/
		auto s_tf(const double *pm_in, const double *fce_in, double *vec_out) noexcept->void;
		/** \brief 根据位姿矩阵转换六维力向量
		*
		* 等同于： vec_out = alpha * tmf(pm_in) * fce_in + beta * vec_out
		*
		*/
		auto s_tf(double alpha, const double *pm_in, const double *fce_in, double beta, double *vec_out) noexcept->void;
		/** \brief 根据位姿矩阵转换六维力矩阵
		*
		* 等同于： m_out = tmf(pm_in) * fces_in
		*
		*/
		auto s_tf_n(int n, const double *pm_in, const double *fces_in, double *m_out) noexcept->void;
		/** \brief 根据位姿矩阵转换六维力矩阵
		*
		* 等同于： m_out = alpha * tmf(pm_in) * fces_in + beta * m_out
		*
		*/
		auto s_tf_n(int n, double alpha, const double *pm_in, const double *fces_in, double beta, double *m_out) noexcept->void;
		/** \brief 根据位姿矩阵的逆矩阵转换六维力向量
		*
		* 等同于： vec_out = tmv(pm_in^-1) * fce_in
		*
		*/
		auto s_inv_tf(const double *inv_pm_in, const double *fce_in, double *vec_out) noexcept->void;
		/** \brief 根据位姿矩阵的逆矩阵转换六维速度向量
		*
		* 等同于： vec_out = alpha * tmf(pm_in^-1) * vel_in + beta * vec_out
		*
		*/
		auto s_inv_tf(double alpha, const double *inv_pm_in, const double *vel_in, double beta, double *vec_out) noexcept->void;
		/** \brief 根据位姿矩阵转换六维速度向量
		*
		* 等同于： vec_out = tmv(pm_in) * vel_in
		*
		*/
		auto s_tv(const double *pm_in, const double *vel_in, double *vec_out) noexcept->void;
		/** \brief 根据位姿矩阵转换六维速度向量
		*
		* 等同于： vec_out = alpha * tmv(pm_in) * vel_in + beta * vec_out
		*
		*/
		auto s_tv(double alpha, const double *pm_in, const double *vel_in, double beta, double *vec_out) noexcept->void;
		/** \brief 根据位姿矩阵转换六维力矩阵
		*
		* 等同于： m_out = tmv(pm_in) * fces_in
		*
		*/
		auto s_tv_n(int n, const double *pm_in, const double *vels_in, double *m_out) noexcept->void;
		/** \brief 根据位姿矩阵转换六维力矩阵
		*
		* 等同于： m_out = alpha * tmv(pm_in) * fces_in + beta * m_out
		*
		*/
		auto s_tv_n(int n, double alpha, const double *pm_in, const double *vels_in, double beta, double *m_out) noexcept->void;
		/** \brief 根据位姿矩阵的逆矩阵转换六维速度向量
		*
		* 等同于： vec_out = tmv(pm_in^-1) * vel_in
		*
		*/
		auto s_inv_tv(const double *inv_pm_in, const double *vel_in, double *vec_out) noexcept->void;
		/** \brief 根据位姿矩阵的逆矩阵转换六维速度向量
		*
		* 等同于： vec_out = alpha * tmv(pm_in^-1) * vel_in + beta * vec_out
		*
		*/
		auto s_inv_tv(double alpha, const double *inv_pm_in, const double *vel_in, double beta, double *vec_out) noexcept->void;
		/** \brief 根据位姿矩阵的逆矩阵转换六维速度向量
		*
		* 等同于： vec_out = tmv(pm_in^-1) * vel_in
		*
		*/
		auto s_inv_tv_n(int n, const double *inv_pm_in, const double *vel_in, double *vec_out) noexcept->void;
		/** \brief 根据位姿矩阵的逆矩阵转换六维速度向量
		*
		* 等同于： vec_out = alpha * tmv(pm_in^-1) * vel_in + beta * vec_out
		*
		*/
		auto s_inv_tv_n(int n, double alpha, const double *inv_pm_in, const double *vel_in, double beta, double *vec_out) noexcept->void;
		/** \brief 构造6x6的力叉乘矩阵
		*
		*  其中，cmf = [w x,  O; v x, w x]
		*
		*/
		auto s_cmf(const double *vel_in, double *cmf_out) noexcept->void;
		/** \brief 构造6x6的速度叉乘矩阵
		*
		*  其中，cmv = \n
		*  [w x,  v x \n
		*   O  ,  w x] \n
		*
		*/
		auto s_cmv(const double *vel_in, double *cmv_out) noexcept->void;
		/** \brief 计算六维向量叉乘
		*
		* 用来计算：vec_out = cro_vec_in xf vec_in
		*
		*/
		auto s_cf(const double *cro_vel_in, const double *vec_in, double* vec_out) noexcept->void;
		/** \brief 计算六维向量叉乘
		*
		* 用来计算：vec_out = alpha * cro_vec_in xf vec_in + beta * vec_out
		*
		*/
		auto s_cf(double alpha, const double *cro_vel_in, const double *vec_in, double beta, double* vec_out) noexcept->void;
		/** \brief 计算六维向量叉乘
		*
		* 用来计算：vec_out = cro_vec_in xv vec_in
		*
		*/
		auto s_cv(const double *cro_vel_in, const double *vec_in, double* vec_out) noexcept->void;
		/** \brief 计算六维向量叉乘
		*
		* 用来计算：vec_out = alpha * cro_vec_in xv vec_in + beta * vec_out
		*
		*/
		auto s_cv(double alpha, const double *cro_vel_in, const double *vec_in, double beta, double* vec_out) noexcept->void;

		auto s_block_cpy(const int &block_size_m, const int &block_size_n,
			const double *from_mtx, const int &fm_begin_row, const int &fm_begin_col, const int &fm_ld,
			double *to_mtx, const int &tm_begin_row, const int &tm_begin_col, const int &tm_ld) noexcept->void;
		auto s_block_cpy(const int &block_size_m, const int &block_size_n,
			double alpha, const double *from_mtx, const int &fm_begin_row, const int &fm_begin_col, const int &fm_ld,
			double beta, double *to_mtx, const int &tm_begin_row, const int &tm_begin_col, const int &tm_ld) noexcept->void;
		auto s_block_cpyT(const int &block_size_m, const int &block_size_n,
			const double *from_mtx, const int &fm_begin_row, const int &fm_begin_col, const int &fm_ld,
			double *to_mtx, const int &tm_begin_row, const int &tm_begin_col, const int &tm_ld) noexcept->void;
		auto s_block_cpyT(const int &block_size_m, const int &block_size_n,
			double alpha, const double *from_mtx, const int &fm_begin_row, const int &fm_begin_col, const int &fm_ld,
			double beta, double *to_mtx, const int &tm_begin_row, const int &tm_begin_col, const int &tm_ld) noexcept->void;

		auto s_dlt_col(const int &dlt_col_num, const int *col_index, const int &m, const int &n, double *A, const int &ldA) noexcept->void;

		/** \brief 计算位姿矩阵的逆矩阵
		*
		* 用来计算：pm_out = pm_in^-1
		*
		*/
		auto s_inv_pm(const double *pm_in, double *pm_out) noexcept->void;
		auto s_pm_dot_pm(const double *pm1_in, const double *pm2_in, double *pm_out) noexcept->void;
		template <typename ...Args>
		auto s_pm_dot_pm(const double *pm1, const double *pm2, Args ...args) noexcept->void
		{
			double pm[16];
			s_pm_dot_pm(pm1, pm2, pm);
			s_pm_dot_pm(pm, args...);
		}
		auto s_inv_pm_dot_pm(const double *inv_pm1_in, const double *pm2_in, double *pm_out) noexcept->void;
		auto s_pm_dot_inv_pm(const double *pm1_in, const double *inv_pm2_in, double *pm_out) noexcept->void;
		auto s_pm_dot_pnt(const double *pm_in, const double *pos_in, double *pos_out) noexcept->void;
		auto s_inv_pm_dot_pnt(const double *pm_in, const double *pos_in, double *pos_out) noexcept->void;
		auto s_pm_dot_v3(const double *pm_in, const double *v3_in, double *v3_out) noexcept->void;
		auto s_inv_pm_dot_v3(const double *inv_pm_in, const double *v3_in, double *v3_out) noexcept->void;
		auto s_m6_dot_v6(const double *m6_in, const double *v6_in, double *v6_out) noexcept->void;

		auto s_vn_add_vn(int N, const double *v1_in, const double *v2_in, double *v_out) noexcept->void;
		auto s_vn_dot_vn(int N, const double *v1_in, const double *v2_in) noexcept->double;

		auto s_v_cro_pm(const double *v_in, const double *pm_in, double *vpm_out) noexcept->void;

		auto s_dscal(const int n, const double a, double *x, const int incx) noexcept->void;
		auto s_dnrm2(const int n, const double *x, const int incx) noexcept->double;
		auto s_daxpy(const int N, const double alpha, const double *X, const int incX, double *Y, const int incY) noexcept->void;
		auto s_swap(const int N, double *X, const int incX, double *Y, const int incY) noexcept->void;
		auto s_transpose(const int m, const int n, const double *A, const int ldA, double *B_out, const int ldB) noexcept->void;

		
		auto s_dgemm(int m, int n, int k, double alpha, const double* A, int lda, const double* B, int ldb, double beta, double *C, int ldc) noexcept->void;
		auto s_dgemmTN(int m, int n, int k, double alpha, const double* A, int lda, const double* B, int ldb, double beta, double *C, int ldc) noexcept->void;
		auto s_dgemmNT(int m, int n, int k, double alpha, const double* A, int lda, const double* B, int ldb, double beta, double *C, int ldc) noexcept->void;

		/** \brief 根据原点和两个坐标轴上的点来求位姿矩阵
		*
		* 这里原点origin为位姿矩阵pm_out的点，firstAxisPnt位于第一根坐标轴，secondAxisPnt位于第一根坐标轴和第二根坐标轴所构成的平面内
		*
		*/
		auto s_axes2pm(const double *origin, const double *firstAxisPnt, const double *secondAxisPnt, double *pm_out, const char *axesOrder = "xy") noexcept->void;
		
		/// \brief 求解形如 k1 * sin(theta) + k2 * cos(theta) = b 的方程，该方程有2个根
		///
		///
		auto s_sov_theta(double k1, double k2, double b, double *theta_out)noexcept->void;

		
	}
}




























#endif
