#ifndef ARIS_DYNAMIC_SCREW_
#define ARIS_DYNAMIC_SCREW_

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

#include "aris_dynamic_matrix.h"

namespace aris
{
	///
	/// 符号定义: \n
	/// pp  :  3x1 点位置(position of point)  \n
	/// re  :  3x1 欧拉角(eula angle)         \n
	/// rq  :  4x1 四元数(quaternions)        \n
	/// rm  :  3x3 旋转矩阵(rotation matrix)  \n
	/// pe  :  6x1 点位置与欧拉角(position and eula angle)\n
	/// pq  :  7x1 点位置与四元数(position and quaternions)\n
	/// pm  :  4x4 位姿矩阵(pose matrix)\n
	/// ra  :  3x1 绕固定轴的旋转的指数积（rotation around axis, exponential product）
	/// ps  :  6x1 位移螺旋，[v;w]的速度螺旋转过theta角，也就是ps = [v;w]*theta
	///
	/// vp  :  3x1 线速度(velocity of point)\n
	/// we  :  3x1 欧拉角导数(omega in term of eula angle)\n
	/// wq  :  4x1 四元数导数(omega in term of quternions)\n
	/// wm  :  3x3 旋转矩阵导数(omega in term of rotation matrix)\n
	/// ve  :  6x1 线速度与欧拉角导数（velocity and omega in term of eula angle）\n
	/// vq  :  7x1 线速度与四元数导数(velocity and omega in term of quternions)\n
	/// vm  :  4x4 位姿矩阵导数(velocity in term of pose matrix)\n
	/// wa  :  3x1 角速度(omega)\n
	/// va  :  6x1 线速度与角速度(velocity and omega)\n
	/// vs  :  6x1 螺旋速度(velocity screw)\n
	///
	/// ap  :  3x1 线加速度(acceleration of point)\n
	/// xe  :  3x1 欧拉角导导数(alpha in term of eula angle)\n
	/// xq  :  4x1 四元数导导数(alpha in term of quternions)\n
	/// xm  :  3x3 旋转矩阵导导数(alpha in term of rotation matrix)\n
	/// ae  :  6x1 线加速度与欧拉角导导数(acceleration and alpha in term of eula angle)\n
	/// aq  :  7x1 线加速度与四元数导导数(acceleration and alpha in term of quternions)\n
	/// am  :  4x4 位姿矩阵导导数(acceleration in term of pose matrix)\n
	/// xa  :  3x1 角加速度(alpha, acceleration of angle)\n
	/// aa  :  6x1 线加速度与角加速度(acceleration and alpha)\n
	/// as  :  6x1 螺旋加速度(acceleration screw)\n

	/// i3  :  3x3 惯量矩阵
	/// im  :  6x6 空间惯量矩阵
	/// iv  :  10x1 惯量矩阵向量[m, cx, cy, cz, Ixx, Iyy, Izz, Ixy, Ixz, Iyz]
	namespace dynamic
	{
		auto s_inv_pm(const double *pm_in, double *pm_out) noexcept->void;
		auto s_pm_dot_pm(const double *pm1_in, const double *pm2_in, double *pm_out = nullptr) noexcept->double *;
		template <typename ...Args>
		auto s_pm_dot_pm(const double *pm1, const double *pm2, Args ...args) noexcept->double *
		{
			double pm[16];
			s_pm_dot_pm(pm1, pm2, pm);
			return s_pm_dot_pm(pm, args...);
		}
		auto s_inv_pm_dot_pm(const double *inv_pm1_in, const double *pm2_in, double *pm_out = nullptr) noexcept->double *;
		auto s_pm_dot_inv_pm(const double *pm1_in, const double *inv_pm2_in, double *pm_out = nullptr) noexcept->double *;
		auto s_pm_dot_v3(const double *pm, const double *v3_in, double *v3_out = nullptr) noexcept->double *;
		template <typename V3Type1, typename V3Type2>
		auto s_pm_dot_v3(const double *pm, const double *v3_in, V3Type1 v31_t, double *v3_out, V3Type2 v32_t) noexcept->void
		{
			const Size a0{ 0 }, a1{ next_rid(a0, v31_t) }, a2{ next_rid(a1, v31_t) };
			const Size b0{ 0 }, b1{ next_rid(b0, v32_t) }, b2{ next_rid(b1, v32_t) };

			v3_out[b0] = pm[0] * v3_in[a0] + pm[1] * v3_in[a1] + pm[2] * v3_in[a2];
			v3_out[b1] = pm[4] * v3_in[a0] + pm[5] * v3_in[a1] + pm[6] * v3_in[a2];
			v3_out[b2] = pm[8] * v3_in[a0] + pm[9] * v3_in[a1] + pm[10] * v3_in[a2];
		}
		auto s_inv_pm_dot_v3(const double *inv_pm, const double *v3, double *v3_out = nullptr) noexcept->double *;
		template <typename V3Type1, typename V3Type2>
		auto s_inv_pm_dot_v3(const double *inv_pm, const double *v3_in, V3Type1 v31_t, double *v3_out, V3Type2 v32_t) noexcept->void
		{
			const Size a0{ 0 }, a1{ next_rid(a0, v31_t) }, a2{ next_rid(a1, v31_t) };
			const Size b0{ 0 }, b1{ next_rid(b0, v32_t) }, b2{ next_rid(b1, v32_t) };

			v3_out[b0] = inv_pm[0] * v3_in[a0] + inv_pm[4] * v3_in[a1] + inv_pm[8] * v3_in[a2];
			v3_out[b1] = inv_pm[1] * v3_in[a0] + inv_pm[5] * v3_in[a1] + inv_pm[9] * v3_in[a2];
			v3_out[b2] = inv_pm[2] * v3_in[a0] + inv_pm[6] * v3_in[a1] + inv_pm[10] * v3_in[a2];
		}

		auto s_im_dot_as(const double *im, const double *as, double * fs = nullptr) noexcept->double *;
		auto s_iv_dot_as(const double *iv, const double *as, double * fs = nullptr) noexcept->double *;


		auto inline s_sinx_over_x(double x)->double { return std::abs(x)<1e-8 ? 1.0 : std::sin(x) / x; };
		auto inline s_one_minus_x_over_square_x(double x)->double { return 0.5*s_sinx_over_x(0.5*x)*s_sinx_over_x(0.5*x); };


		/// \brief 计算三维向量叉乘矩阵
		///
		/// 用来计算：cm_out = \n
		/// [ 0  -z   y \n
		///   z   0  -x \n
		///  -y   x   0 ]
		/// 
		///
		auto s_cm3(const double *a, double *cm_out) noexcept->void;
		/// \brief 计算三维向量叉乘
		///
		/// 用来计算：c = a x b
		///
		///
		auto s_c3(const double *a, const double *b, double *c_out) noexcept->void;
		/// \brief 计算三维向量叉乘
		///
		/// 用来计算：c = a x b
		///
		///
		template<typename AType, typename BType, typename CType>
		auto s_c3(const double *a, AType a_t, const double *b , BType b_t, double *c_out, CType c_t) noexcept->void
		{
			const Size a0{ 0 }, a1{ next_rid(a0, a_t) }, a2{ next_rid(a1, a_t) };
			const Size b0{ 0 }, b1{ next_rid(b0, b_t) }, b2{ next_rid(b1, b_t) };
			const Size c0{ 0 }, c1{ next_rid(c0, c_t) }, c2{ next_rid(c1, c_t) };

			c_out[c0] = -a[a2] * b[b1] + a[a1] * b[b2];
			c_out[c1] = a[a2] * b[b0] - a[a0] * b[b2];
			c_out[c2] = -a[a1] * b[b0] + a[a0] * b[b1];
		}
		/// \brief 计算三维向量叉乘
		///
		/// 用来计算：c = alpha * a x b
		///
		///
		auto s_c3(double alpha, const double *a, const double *b, double *c_out) noexcept->void;
		/// \brief 计算三维向量叉乘
		///
		/// 用来计算：c = alpha * a x b
		///
		///
		template<typename AType, typename BType, typename CType>
		auto s_c3(double alpha, const double *a, AType a_t, const double *b, BType b_t, double *c_out, CType c_t) noexcept->void
		{
			const Size a0{ 0 }, a1{ next_rid(a0, a_t) }, a2{ next_rid(a1, a_t) };
			const Size b0{ 0 }, b1{ next_rid(b0, b_t) }, b2{ next_rid(b1, b_t) };
			const Size c0{ 0 }, c1{ next_rid(c0, c_t) }, c2{ next_rid(c1, c_t) };

			c_out[c0] = alpha*(-a[a2] * b[b1] + a[a1] * b[b2]);
			c_out[c1] = alpha*(a[a2] * b[b0] - a[a0] * b[b2]);
			c_out[c2] = alpha*(-a[a1] * b[b0] + a[a0] * b[b1]);
		}
		/// \brief 计算三维向量叉乘
		///
		/// 用来计算：c = -a x b
		///
		///
		auto s_c3i(const double *a, const double *b, double *c_out) noexcept->void;
		/// \brief 计算三维向量叉乘
		///
		/// 用来计算：c = a x b
		///
		///
		template<typename AType, typename BType, typename CType>
		auto s_c3i(const double *a, AType a_t, const double *b, BType b_t, double *c_out, CType c_t) noexcept->void
		{
			const Size a0{ 0 }, a1{ next_rid(a0, a_t) }, a2{ next_rid(a1, a_t) };
			const Size b0{ 0 }, b1{ next_rid(b0, b_t) }, b2{ next_rid(b1, b_t) };
			const Size c0{ 0 }, c1{ next_rid(c0, c_t) }, c2{ next_rid(c1, c_t) };

			c_out[c0] = a[a2] * b[b1] - a[a1] * b[b2];
			c_out[c1] = -a[a2] * b[b0] + a[a0] * b[b2];
			c_out[c2] = a[a1] * b[b0] - a[a0] * b[b1];
		}
		/// \brief 计算三维向量叉乘
		///
		/// 用来计算：c = a x b + c
		///
		///
		auto s_c3a(const double *a, const double *b, double *c_out) noexcept->void;
		/// \brief 计算三维向量叉乘
		///
		/// 用来计算：c = a x b + c
		///
		///
		template<typename AType, typename BType, typename CType>
		auto s_c3a(const double *a, AType a_t, const double *b, BType b_t, double *c_out, CType c_t) noexcept->void
		{
			const Size a0{ 0 }, a1{ next_rid(a0, a_t) }, a2{ next_rid(a1, a_t) };
			const Size b0{ 0 }, b1{ next_rid(b0, b_t) }, b2{ next_rid(b1, b_t) };
			const Size c0{ 0 }, c1{ next_rid(c0, c_t) }, c2{ next_rid(c1, c_t) };

			c_out[c0] += -a[a2] * b[b1] + a[a1] * b[b2];
			c_out[c1] += a[a2] * b[b0] - a[a0] * b[b2];
			c_out[c2] += -a[a1] * b[b0] + a[a0] * b[b1];
		}
		/// \brief 计算三维向量叉乘
		///
		/// 用来计算：c = alpha * a x b + beta * c
		///
		///
		auto s_c3a(double alpha, const double *a, const double *b, double *c_out) noexcept->void;
		/// \brief 计算三维向量叉乘
		///
		/// 用来计算：c = alpha * a x b + beta * c
		///
		///
		template<typename AType, typename BType, typename CType>
		auto s_c3a(double alpha, const double *a, AType a_t, const double *b, BType b_t, double *c_out, CType c_t) noexcept->void
		{
			const Size a0{ 0 }, a1{ next_rid(a0, a_t) }, a2{ next_rid(a1, a_t) };
			const Size b0{ 0 }, b1{ next_rid(b0, b_t) }, b2{ next_rid(b1, b_t) };
			const Size c0{ 0 }, c1{ next_rid(c0, c_t) }, c2{ next_rid(c1, c_t) };
			
			c_out[c0] += alpha*(-a[a2] * b[b1] + a[a1] * b[b2]);
			c_out[c1] += alpha*(a[a2] * b[b0] - a[a0] * b[b2]);
			c_out[c2] += alpha*(-a[a1] * b[b0] + a[a0] * b[b1]);
		}
		/// \brief 计算三维向量叉乘
		///
		/// 用来计算：c = a x b + c
		///
		///
		auto s_c3s(const double *a, const double *b, double *c_out) noexcept->void;
		/// \brief 计算三维向量叉乘
		///
		/// 用来计算：c = a x b + c
		///
		///
		template<typename AType, typename BType, typename CType>
		auto s_c3s(const double *a, AType a_t, const double *b, BType b_t, double *c_out, CType c_t) noexcept->void
		{
			const Size a0{ 0 }, a1{ next_rid(a0, a_t) }, a2{ next_rid(a1, a_t) };
			const Size b0{ 0 }, b1{ next_rid(b0, b_t) }, b2{ next_rid(b1, b_t) };
			const Size c0{ 0 }, c1{ next_rid(c0, c_t) }, c2{ next_rid(c1, c_t) };

			c_out[c0] -= -a[a2] * b[b1] + a[a1] * b[b2];
			c_out[c1] -= a[a2] * b[b0] - a[a0] * b[b2];
			c_out[c2] -= -a[a1] * b[b0] + a[a0] * b[b1];
		}
		/// \brief 构造6x6的力叉乘矩阵
		///
		///  其中,cmf = [w x,  O; v x, w x]
		///
		///
		auto s_cmf(const double *vs, double *cmf_out) noexcept->void;
		/// \brief 计算六维向量叉乘
		///
		/// 用来计算：vfs = vs xf fs
		///
		///
		auto s_cf(const double *vs, const double *fs, double* vfs_out) noexcept->void;
		/// \brief 计算六维向量叉乘
		///
		/// 用来计算：vfs = vs xf fs
		///
		///
		template<typename VType, typename FType, typename VFType>
		auto s_cf(const double *vs, VType v_t, const double *fs, FType f_t, double* vfs_out, VFType vf_t) noexcept->void
		{
			Size v3_pos{ id(3, v_t) }, f3_pos{ id(3, f_t) }, vf3_pos{ id(3, vf_t) };
			
			s_c3(vs + v3_pos, v_t, fs, f_t, vfs_out, vf_t);
			s_c3(vs + v3_pos, v_t, fs + f3_pos, f_t, vfs_out + vf3_pos, vf_t);
			s_c3a(vs, v_t, fs, f_t, vfs_out + vf3_pos, vf_t);
		}
		/// \brief 计算六维向量叉乘
		///
		/// 用来计算：vfs = alpha * vs xf fs
		///
		///
		auto s_cf(double alpha, const double *vs, const double *fs, double* vfs_out) noexcept->void;
		/// \brief 计算六维向量叉乘
		///
		/// 用来计算：vfs = alpha * vs xf fs
		///
		///
		template<typename VType, typename FType, typename VFType>
		auto s_cf(double alpha, const double *vs, VType v_t, const double *fs, FType f_t, double* vfs_out, VFType vf_t) noexcept->void
		{
			s_cf(vs, v_t, fs, f_t, vfs_out, vf_t);
			s_nv(6, alpha, vfs_out, vf_t);
		}
		/// \brief 计算六维向量叉乘
		///
		/// 用来计算：vfs = vs xf fs
		///
		///
		auto s_cfi(const double *vs, const double *fs, double* vfs_out) noexcept->void;
		/// \brief 计算六维向量叉乘
		///
		/// 用来计算：vfs = vs xf fs
		///
		///
		template<typename VType, typename FType, typename VFType>
		auto s_cfi(const double *vs, VType v_t, const double *fs, FType f_t, double* vfs_out, VFType vf_t) noexcept->void
		{
			Size v3_pos{ id(3, v_t) }, f3_pos{ id(3, f_t) }, vf3_pos{ id(3, vf_t) };

			s_c3i(vs + v3_pos, v_t, fs, f_t, vfs_out, vf_t);
			s_c3i(vs + v3_pos, v_t, fs + f3_pos, f_t, vfs_out + vf3_pos, vf_t);
			s_c3s(vs, v_t, fs, f_t, vfs_out + vf3_pos, vf_t);
		}
		/// \brief 计算六维向量叉乘
		///
		/// 用来计算：vfs = vs xf fs
		///
		///
		auto s_cfa(const double *vs, const double *fs, double* vfs_out) noexcept->void;
		/// \brief 计算六维向量叉乘
		///
		/// 用来计算：vfs = vs xf fs
		///
		///
		template<typename VType, typename FType, typename VFType>
		auto s_cfa(const double *vs, VType v_t, const double *fs, FType f_t, double* vfs_out, VFType vf_t) noexcept->void
		{
			Size v3_pos{ id(3, v_t) }, f3_pos{ id(3, f_t) }, vf3_pos{ id(3, vf_t) };

			s_c3a(vs + v3_pos, v_t, fs, f_t, vfs_out, vf_t);
			s_c3a(vs + v3_pos, v_t, fs + f3_pos, f_t, vfs_out + vf3_pos, vf_t);
			s_c3a(vs, v_t, fs, f_t, vfs_out + vf3_pos, vf_t);
		}
		/// \brief 计算六维向量叉乘
		///
		/// 用来计算：vfs = alpha * vs xf fs + beta * vfs
		///
		///
		auto s_cfa(double alpha, const double *vs, const double *fs, double* vfs_out) noexcept->void;
		/// \brief 计算六维向量叉乘
		///
		/// 用来计算：vfs = alpha * vs xf fs + beta * vfs
		///
		///
		template<typename VType, typename FType, typename VFType>
		auto s_cfa(double alpha, const double *vs, VType v_t, const double *fs, FType f_t, double* vfs_out, VFType vf_t) noexcept->void
		{
			Size v3_pos{ id(3, v_t) }, f3_pos{ id(3, f_t) }, vf3_pos{ id(3, vf_t) };

			s_c3a(alpha, vs + v3_pos, v_t, fs, f_t, vfs_out, vf_t);
			s_c3a(alpha, vs + v3_pos, v_t, fs + f3_pos, f_t, vfs_out + vf3_pos, vf_t);
			s_c3a(alpha, vs, v_t, fs, f_t, vfs_out + vf3_pos, vf_t);
		}
		/// \brief 计算六维向量叉乘
		///
		/// 用来计算：vfs = vs xf fs
		///
		///
		auto s_cfs(const double *vs, const double *fs, double* vfs_out) noexcept->void;
		/// \brief 计算六维向量叉乘
		///
		/// 用来计算：vfs = vs xf fs
		///
		///
		template<typename VType, typename FType, typename VFType>
		auto s_cfs(const double *vs, VType v_t, const double *fs, FType f_t, double* vfs_out, VFType vf_t) noexcept->void
		{
			Size v3_pos{ id(3, v_t) }, f3_pos{ id(3, f_t) }, vf3_pos{ id(3, vf_t) };

			s_c3s(vs + v3_pos, v_t, fs, f_t, vfs_out, vf_t);
			s_c3s(vs + v3_pos, v_t, fs + f3_pos, f_t, vfs_out + vf3_pos, vf_t);
			s_c3s(vs, v_t, fs, f_t, vfs_out + vf3_pos, vf_t);
		}
		/// \brief 构造6x6的速度叉乘矩阵
		///
		///  其中,cmv = \n
		///  [w x,  v x \n
		///   O  ,  w x] \n
		///
		///
		auto s_cmv(const double *vs, double *cmv_out) noexcept->void;
		/// \brief 计算六维向量叉乘
		///
		/// 用来计算：vvs = vs xv vs2
		///
		///
		auto s_cv(const double *vs, const double *vs2, double* vvs_out) noexcept->void;
		/// \brief 计算六维向量叉乘
		///
		/// 用来计算：vvs = vs xv vs2
		///
		///
		template<typename VType, typename V2Type, typename VVType>
		auto s_cv(const double *vs, VType v_t, const double *vs2, V2Type v2_t, double* vvs_out, VVType vv_t) noexcept->void
		{
			Size v3_pos{ id(3, v_t) }, v23_pos{ id(3, v2_t) }, vv3_pos{ id(3, vv_t) };
			
			s_c3(vs + v3_pos, v_t, vs2, v2_t, vvs_out, vv_t);
			s_c3(vs + v3_pos, v_t, vs2 + v23_pos, v2_t, vvs_out + vv3_pos, vv_t);
			s_c3a(vs, v_t, vs2 + v23_pos, v2_t, vvs_out, vv_t);
		}
		/// \brief 计算六维向量叉乘
		///
		/// 用来计算：vvs = alpha * vs xv vs2
		///
		///
		auto s_cv(double alpha, const double *vs, const double *vs2, double* vvs_out) noexcept->void;
		/// \brief 计算六维向量叉乘
		///
		/// 用来计算：vvs = alpha * vs xv vs2
		///
		///
		template<typename VType, typename V2Type, typename VVType>
		auto s_cv(double alpha, const double *vs, VType v_t, const double *vs2, V2Type v2_t, double* vvs_out, VVType vv_t) noexcept->void
		{
			s_cv(vs, v_t, vs2, v2_t, vvs_out, vv_t);
			s_nv(6, alpha, vvs_out, vv_t);
		}
		/// \brief 计算六维向量叉乘
		///
		/// 用来计算：vvs = vs xv vs2
		///
		///
		auto s_cvi(const double *vs, const double *vs2, double* vvs_out) noexcept->void;
		/// \brief 计算六维向量叉乘
		///
		/// 用来计算：vvs = vs xv vs2
		///
		///
		template<typename VType, typename V2Type, typename VVType>
		auto s_cvi(const double *vs, VType v_t, const double *vs2, V2Type v2_t, double* vvs_out, VVType vv_t) noexcept->void
		{
			Size v3_pos{ id(3, v_t) }, v23_pos{ id(3, v2_t) }, vv3_pos{ id(3, vv_t) };

			s_c3i(vs + v3_pos, v_t, vs2, v2_t, vvs_out, vv_t);
			s_c3i(vs + v3_pos, v_t, vs2 + v23_pos, v2_t, vvs_out + vv3_pos, vv_t);
			s_c3s(vs, v_t, vs2 + v23_pos, v2_t, vvs_out, vv_t);
		}
		/// \brief 计算六维向量叉乘
		///
		/// 用来计算：vvs = vs xv vs2 + vvs
		///
		///
		auto s_cva(const double *vs, const double *vs2, double* vvs_out) noexcept->void;
		/// \brief 计算六维向量叉乘
		///
		/// 用来计算：vvs = vs xv vs2 + vvs
		///
		///
		template<typename VType, typename V2Type, typename VVType>
		auto s_cva(const double *vs, VType v_t, const double *vs2, V2Type v2_t, double* vvs_out, VVType vv_t) noexcept->void
		{
			Size v3_pos{ id(3, v_t) }, v23_pos{ id(3, v2_t) }, vv3_pos{ id(3, vv_t) };

			s_c3a(vs + v3_pos, v_t, vs2, v2_t, vvs_out, vv_t);
			s_c3a(vs + v3_pos, v_t, vs2 + v23_pos, v2_t, vvs_out + vv3_pos, vv_t);
			s_c3a(vs, v_t, vs2 + v23_pos, v2_t, vvs_out, vv_t);
		}
		/// \brief 计算六维向量叉乘
		///
		/// 用来计算：vvs = alpha * vs xv vs2 + beta * vvs
		///
		///
		auto s_cva(double alpha, const double *vs, const double *vs2, double* vvs_out) noexcept->void;
		/// \brief 计算六维向量叉乘
		///
		/// 用来计算：vvs = alpha * vs xv vs2 + beta * vvs
		///
		///
		template<typename VType, typename V2Type, typename VVType>
		auto s_cva(double alpha, const double *vs, VType v_t, const double *vs2, V2Type v2_t, double* vvs_out, VVType vv_t) noexcept->void
		{
			Size v3_pos{ id(3, v_t) }, v23_pos{ id(3, v2_t) }, vv3_pos{ id(3, vv_t) };

			s_c3a(alpha, vs + v3_pos, v_t, vs2, v2_t, vvs_out, vv_t);
			s_c3a(alpha, vs + v3_pos, v_t, vs2 + v23_pos, v2_t, vvs_out + vv3_pos, vv_t);
			s_c3a(alpha, vs, v_t, vs2 + v23_pos, v2_t, vvs_out, vv_t);
		}
		/// \brief 计算六维向量叉乘
		///
		/// 用来计算：vvs = vs xv vs2 + vvs
		///
		///
		auto s_cvs(const double *vs, const double *vs2, double* vvs_out) noexcept->void;
		/// \brief 计算六维向量叉乘
		///
		/// 用来计算：vvs = vs xv vs2 + vvs
		///
		///
		template<typename VType, typename V2Type, typename VVType>
		auto s_cvs(const double *vs, VType v_t, const double *vs2, V2Type v2_t, double* vvs_out, VVType vv_t) noexcept->void
		{
			Size v3_pos{ id(3, v_t) }, v23_pos{ id(3, v2_t) }, vv3_pos{ id(3, vv_t) };

			s_c3s(vs + v3_pos, v_t, vs2, v2_t, vvs_out, vv_t);
			s_c3s(vs + v3_pos, v_t, vs2 + v23_pos, v2_t, vvs_out + vv3_pos, vv_t);
			s_c3s(vs, v_t, vs2 + v23_pos, v2_t, vvs_out, vv_t);
		}
		/// \brief 计算三维向量叉乘
		///
		/// 用来计算：c_mtx_out = a x b_mtx
		///
		///
		auto inline s_c3_n(Size n, const double *a, const double *b_mtx, double *c_mtx_out) noexcept->void { for (Size i = 0; i < n; ++i)s_c3(a, 1, b_mtx + i, n, c_mtx_out + i, n); }
		/// \brief 计算三维向量叉乘
		///
		/// 用来计算：c_mtx_out = a x b_mtx
		///
		///
		template<typename AType, typename BType, typename CType>
		auto inline s_c3_n(Size n, const double *a, AType a_t, const double *b_mtx, BType b_t, double *c_mtx_out, CType c_t) noexcept->void
		{ 
			for (Size i(-1), bid(0), cid(0); ++i < n; bid = next_cid(bid, b_t), cid = next_cid(cid, c_t))
				s_c3(a, a_t, b_mtx + bid, b_t, c_mtx_out + cid, c_t); 
		}
		/// \brief 计算三维向量叉乘
		///
		/// 用来计算：c_mtx_out = alpha * a x b_mtx
		///
		///
		auto inline s_c3_n(Size n, double alpha, const double *a, const double *b_mtx, double *c_mtx_out) noexcept->void { for (Size i = 0; i < n; ++i)s_c3(alpha, a, 1, b_mtx + i, n, c_mtx_out + i, n); }
		/// \brief 计算三维向量叉乘
		///
		/// 用来计算：c_mtx_out = alpha * a x b_mtx
		///
		///
		template<typename AType, typename BType, typename CType>
		auto inline s_c3_n(Size n, double alpha, const double *a, AType a_t, const double *b_mtx, BType b_t, double *c_mtx_out, CType c_t) noexcept->void
		{ 
			for (Size i(-1), bid(0), cid(0); ++i < n; bid = next_cid(bid, b_t), cid = next_cid(cid, c_t))
				s_c3(alpha, a, a_t, b_mtx + bid, b_t, c_mtx_out + cid, c_t);
		}
		/// \brief 计算三维向量叉乘
		///
		/// 用来计算：c_mtx_out = a x b_mtx
		///
		///
		auto inline s_c3i_n(Size n, const double *a, const double *b_mtx, double *c_mtx_out) noexcept->void { for (Size i = 0; i < n; ++i)s_c3i(a, 1, b_mtx + i, n, c_mtx_out + i, n); }
		/// \brief 计算三维向量叉乘
		///
		/// 用来计算：c_mtx_out = a x b_mtx
		///
		///
		template<typename AType, typename BType, typename CType>
		auto inline s_c3i_n(Size n, const double *a, AType a_t, const double *b_mtx, BType b_t, double *c_mtx_out, CType c_t) noexcept->void
		{
			for (Size i(-1), bid(0), cid(0); ++i < n; bid = next_cid(bid, b_t), cid = next_cid(cid, c_t))
				s_c3i(a, a_t, b_mtx + bid, b_t, c_mtx_out + cid, c_t);
		}
		/// \brief 计算三维向量叉乘
		///
		/// 用来计算：c_mtx_out = alpha * a x b_mtx + beta * c_mtx_out
		///
		///
		auto inline s_c3a_n(Size n, const double *a, const double *b_mtx, double *c_mtx_out) noexcept->void { for (Size i = 0; i < n; ++i)s_c3a(a, 1, b_mtx + i, n, c_mtx_out + i, n); }
		/// \brief 计算三维向量叉乘
		///
		/// 用来计算：c_mtx_out = alpha * a x b_mtx + beta * c_mtx_out
		///
		///
		template<typename AType, typename BType, typename CType>
		auto inline s_c3a_n(Size n, const double *a, AType a_t, const double *b_mtx, BType b_t, double *c_mtx_out, CType c_t) noexcept->void
		{ 
			for (Size i(-1), bid(0), cid(0); ++i < n; bid = next_cid(bid, b_t), cid = next_cid(cid, c_t))
				s_c3a(a, a_t, b_mtx + bid, b_t, c_mtx_out + cid, c_t);
		}
		/// \brief 计算三维向量叉乘
		///
		/// 用来计算：c_mtx_out = alpha * a x b_mtx + beta * c_mtx_out
		///
		///
		auto inline s_c3a_n(Size n, double alpha, const double *a, const double *b_mtx, double *c_mtx_out) noexcept->void { for (Size i = 0; i < n; ++i)s_c3a(alpha, a, 1, b_mtx + i, n, c_mtx_out + i, n); }
		/// \brief 计算三维向量叉乘
		///
		/// 用来计算：c_mtx_out = alpha * a x b_mtx + beta * c_mtx_out
		///
		///
		template<typename AType, typename BType, typename CType>
		auto inline s_c3a_n(Size n, double alpha, const double *a, AType a_t, const double *b_mtx, BType b_t, double *c_mtx_out, CType c_t) noexcept->void
		{ 
			for (Size i(-1), bid(0), cid(0); ++i < n; bid = next_cid(bid, b_t), cid = next_cid(cid, c_t))
				s_c3a(alpha, a, a_t, b_mtx + bid, b_t, c_mtx_out + cid, c_t);
		}
		/// \brief 计算三维向量叉乘
		///
		/// 用来计算：c_mtx_out = alpha * a x b_mtx + beta * c_mtx_out
		///
		///
		auto inline s_c3s_n(Size n, const double *a, const double *b_mtx, double *c_mtx_out) noexcept->void { for (Size i = 0; i < n; ++i)s_c3s(a, 1, b_mtx + i, n, c_mtx_out + i, n); }
		/// \brief 计算三维向量叉乘
		///
		/// 用来计算：c_mtx_out = alpha * a x b_mtx + beta * c_mtx_out
		///
		///
		template<typename AType, typename BType, typename CType>
		auto inline s_c3s_n(Size n, const double *a, AType a_t, const double *b_mtx, BType b_t, double *c_mtx_out, CType c_t) noexcept->void
		{
			for (Size i(-1), bid(0), cid(0); ++i < n; bid = next_cid(bid, b_t), cid = next_cid(cid, c_t))
				s_c3s(a, a_t, b_mtx + bid, b_t, c_mtx_out + cid, c_t);
		}
		/// \brief 计算六维向量叉乘
		///
		/// 用来计算：vfs = vs xf fs
		///
		///
		auto inline s_cf_n(Size n, const double *vs, const double *fs_mtx, double* vfs_mtx_out) noexcept->void { for (Size i = 0; i < n; ++i)s_cf(vs, 1, fs_mtx + i, n, vfs_mtx_out + i, n); }
		/// \brief 计算六维向量叉乘
		///
		/// 用来计算：vfs = vs xf fs
		///
		///
		template<typename VType, typename FType, typename VFType>
		auto inline s_cf_n(Size n, const double *vs, VType v_t, const double *fs_mtx, FType f_t, double* vfs_mtx_out, VFType vf_t) noexcept->void 
		{ 
			for (Size i(-1), fid(0), vfid(0); ++i < n; fid = next_cid(fid, f_t), vfid = next_cid(vfid, vf_t))
				s_cf(vs, v_t, fs_mtx + fid, f_t, vfs_mtx_out + vfid, vf_t);
		}
		/// \brief 计算六维向量叉乘
		///
		/// 用来计算：vfs = alpha * vs xf fs
		///
		///
		auto inline s_cf_n(Size n, double alpha, const double *vs, const double *fs_mtx, double* vfs_mtx_out) noexcept->void { for (Size i = 0; i < n; ++i)s_cf(alpha, vs, 1, fs_mtx + i, n, vfs_mtx_out + i, n); }
		/// \brief 计算六维向量叉乘
		///
		/// 用来计算：vfs = alpha * vs xf fs
		///
		///
		template<typename VType, typename FType, typename VFType>
		auto inline s_cf_n(Size n, double alpha, const double *vs, VType v_t, const double *fs_mtx, FType f_t, double* vfs_mtx_out, VFType vf_t) noexcept->void
		{ 
			for (Size i(-1), fid(0), vfid(0); ++i < n; fid = next_cid(fid, f_t), vfid = next_cid(vfid, vf_t))
				s_cf(alpha, vs, v_t, fs_mtx + fid, f_t, vfs_mtx_out + vfid, vf_t);
		}
		/// \brief 计算六维向量叉乘
		///
		/// 用来计算：vfs = vs xf fs
		///
		///
		auto inline s_cfi_n(Size n, const double *vs, const double *fs_mtx, double* vfs_mtx_out) noexcept->void { for (Size i = 0; i < n; ++i)s_cfi(vs, 1, fs_mtx + i, n, vfs_mtx_out + i, n); }
		/// \brief 计算六维向量叉乘
		///
		/// 用来计算：vfs = vs xf fs
		///
		///
		template<typename VType, typename FType, typename VFType>
		auto inline s_cfi_n(Size n, const double *vs, VType v_t, const double *fs_mtx, FType f_t, double* vfs_mtx_out, VFType vf_t) noexcept->void
		{
			for (Size i(-1), fid(0), vfid(0); ++i < n; fid = next_cid(fid, f_t), vfid = next_cid(vfid, vf_t))
				s_cfi(vs, v_t, fs_mtx + fid, f_t, vfs_mtx_out + vfid, vf_t);
		}
		/// \brief 计算六维向量叉乘
		///
		/// 用来计算：vfs = vs xf fs
		///
		///
		auto inline s_cfa_n(Size n, const double *vs, const double *fs_mtx, double* vfs_mtx_out) noexcept->void { for (Size i = 0; i < n; ++i)s_cfa(vs, 1, fs_mtx + i, n, vfs_mtx_out + i, n); }
		/// \brief 计算六维向量叉乘
		///
		/// 用来计算：vfs = vs xf fs
		///
		///
		template<typename VType, typename FType, typename VFType>
		auto inline s_cfa_n(Size n, const double *vs, VType v_t, const double *fs_mtx, FType f_t, double* vfs_mtx_out, VFType vf_t) noexcept->void 
		{ 
			for (Size i(-1), fid(0), vfid(0); ++i < n; fid = next_cid(fid, f_t), vfid = next_cid(vfid, vf_t))
				s_cfa(vs, v_t, fs_mtx + fid, f_t, vfs_mtx_out + vfid, vf_t);
		}
		/// \brief 计算六维向量叉乘
		///
		/// 用来计算：vfs = alpha * vs xf fs + beta * vfs
		///
		///
		auto inline s_cfa_n(Size n, double alpha, const double *vs, const double *fs_mtx, double* vfs_mtx_out) noexcept->void { for (Size i = 0; i < n; ++i)s_cfa(alpha, vs, 1, fs_mtx + i, n, vfs_mtx_out + i, n); }
		/// \brief 计算六维向量叉乘
		///
		/// 用来计算：vfs = alpha * vs xf fs + beta * vfs
		///
		///
		template<typename VType, typename FType, typename VFType>
		auto inline s_cfa_n(Size n, double alpha, const double *vs, VType v_t, const double *fs_mtx, FType f_t, double* vfs_mtx_out, VFType vf_t) noexcept->void 
		{
			for (Size i(-1), fid(0), vfid(0); ++i < n; fid = next_cid(fid, f_t), vfid = next_cid(vfid, vf_t))
				s_cfa(alpha, vs, v_t, fs_mtx + fid, f_t, vfs_mtx_out + vfid, vf_t);
		}
		/// \brief 计算六维向量叉乘
		///
		/// 用来计算：vfs = vs xf fs
		///
		///
		auto inline s_cfs_n(Size n, const double *vs, const double *fs_mtx, double* vfs_mtx_out) noexcept->void { for (Size i = 0; i < n; ++i)s_cfs(vs, 1, fs_mtx + i, n, vfs_mtx_out + i, n); }
		/// \brief 计算六维向量叉乘
		///
		/// 用来计算：vfs = vs xf fs
		///
		///
		template<typename VType, typename FType, typename VFType>
		auto inline s_cfs_n(Size n, const double *vs, VType v_t, const double *fs_mtx, FType f_t, double* vfs_mtx_out, VFType vf_t) noexcept->void
		{
			for (Size i(-1), fid(0), vfid(0); ++i < n; fid = next_cid(fid, f_t), vfid = next_cid(vfid, vf_t))
				s_cfs(vs, v_t, fs_mtx + fid, f_t, vfs_mtx_out + vfid, vf_t);
		}
		/// \brief 计算六维向量叉乘
		///
		/// 用来计算：vvs = vs xv vs2
		///
		///
		auto inline s_cv_n(Size n, const double *vs, const double *vs_mtx, double* vvs_mtx_out) noexcept->void { for (Size i = 0; i < n; ++i)s_cv(vs, 1, vs_mtx + i, n, vvs_mtx_out + i, n); }
		/// \brief 计算六维向量叉乘
		///
		/// 用来计算：vvs = vs xv vs2
		///
		///
		template<typename VType, typename V2Type, typename VVType>
		auto inline s_cv_n(Size n, const double *vs, VType v_t, const double *vs_mtx, V2Type v2_t, double* vvs_mtx_out, VVType vv_t) noexcept->void 
		{ 
			for (Size i(-1), vid(0), vvid(0); ++i < n; vid = next_cid(vid, v2_t), vvid = next_cid(vvid, vv_t))
				s_cv(vs, v_t, vs_mtx + vid, v2_t, vvs_mtx_out + vvid, vv_t);
		}
		/// \brief 计算六维向量叉乘
		///
		/// 用来计算：vvs = alpha * vs xv vs2
		///
		///
		auto inline s_cv_n(Size n, double alpha, const double *vs, const double *vs_mtx, double* vvs_mtx_out) noexcept->void { for (Size i = 0; i < n; ++i)s_cv(alpha, vs, 1, vs_mtx + i, n, vvs_mtx_out + i, n); }
		/// \brief 计算六维向量叉乘
		///
		/// 用来计算：vvs = alpha * vs xv vs2
		///
		///
		template<typename VType, typename V2Type, typename VVType>
		auto inline s_cv_n(Size n, double alpha, const double *vs, VType v_t, const double *vs_mtx, V2Type v2_t, double* vvs_mtx_out, VVType vv_t) noexcept->void
		{
			for (Size i(-1), vid(0), vvid(0); ++i < n; vid = next_cid(vid, v2_t), vvid = next_cid(vvid, vv_t))
				s_cv(alpha, vs, v_t, vs_mtx + vid, v2_t, vvs_mtx_out + vvid, vv_t);
		}
		/// \brief 计算六维向量叉乘
		///
		/// 用来计算：vvs = vs xv vs2
		///
		///
		auto inline s_cvi_n(Size n, const double *vs, const double *vs_mtx, double* vvs_mtx_out) noexcept->void { for (Size i = 0; i < n; ++i)s_cvi(vs, 1, vs_mtx + i, n, vvs_mtx_out + i, n); }
		/// \brief 计算六维向量叉乘
		///
		/// 用来计算：vvs = vs xv vs2
		///
		///
		template<typename VType, typename V2Type, typename VVType>
		auto inline s_cvi_n(Size n, const double *vs, VType v_t, const double *vs_mtx, V2Type v2_t, double* vvs_mtx_out, VVType vv_t) noexcept->void
		{
			for (Size i(-1), vid(0), vvid(0); ++i < n; vid = next_cid(vid, v2_t), vvid = next_cid(vvid, vv_t))
				s_cvi(vs, v_t, vs_mtx + vid, v2_t, vvs_mtx_out + vvid, vv_t);
		}
		/// \brief 计算六维向量叉乘
		///
		/// 用来计算：vvs = vs xv vs2 + vvs
		///
		///
		auto inline s_cva_n(Size n, const double *vs, const double *vs_mtx, double* vvs_mtx_out) noexcept->void { for (Size i = 0; i < n; ++i)s_cva(vs, 1, vs_mtx + i, n, vvs_mtx_out + i, n); }
		/// \brief 计算六维向量叉乘
		///
		/// 用来计算：vvs = vs xv vs2 + vvs
		///
		///
		template<typename VType, typename V2Type, typename VVType>
		auto inline s_cva_n(Size n, const double *vs, VType v_t, const double *vs_mtx, V2Type v2_t, double* vvs_mtx_out, VVType vv_t) noexcept->void
		{
			for (Size i(-1), vid(0), vvid(0); ++i < n; vid = next_cid(vid, v2_t), vvid = next_cid(vvid, vv_t))
				s_cva(vs, v_t, vs_mtx + vid, v2_t, vvs_mtx_out + vvid, vv_t);
		}
		/// \brief 计算六维向量叉乘
		///
		/// 用来计算：vvs = alpha * vs xv vs2 + beta * vvs
		///
		///
		auto inline s_cva_n(Size n, double alpha, const double *vs, const double *vs_mtx, double* vvs_mtx_out) noexcept->void { for (Size i = 0; i < n; ++i)s_cva(alpha, vs, 1, vs_mtx + i, n, vvs_mtx_out + i, n); }
		/// \brief 计算六维向量叉乘
		///
		/// 用来计算：vvs = alpha * vs xv vs2 + beta * vvs
		///
		///
		template<typename VType, typename V2Type, typename VVType>
		auto inline s_cva_n(Size n, double alpha, const double *vs, VType v_t, const double *vs_mtx, V2Type v2_t, double* vvs_mtx_out, VVType vv_t) noexcept->void
		{
			for (Size i(-1), vid(0), vvid(0); ++i < n; vid = next_cid(vid, v2_t), vvid = next_cid(vvid, vv_t))
				s_cva(alpha, vs, v_t, vs_mtx + vid, v2_t, vvs_mtx_out + vvid, vv_t);
		}
		/// \brief 计算六维向量叉乘
		///
		/// 用来计算：vvs = vs xv vs2 + vvs
		///
		///
		auto inline s_cvs_n(Size n, const double *vs, const double *vs_mtx, double* vvs_mtx_out) noexcept->void { for (Size i = 0; i < n; ++i)s_cvs(vs, 1, vs_mtx + i, n, vvs_mtx_out + i, n); }
		/// \brief 计算六维向量叉乘
		///
		/// 用来计算：vvs = vs xv vs2 + vvs
		///
		///
		template<typename VType, typename V2Type, typename VVType>
		auto inline s_cvs_n(Size n, const double *vs, VType v_t, const double *vs_mtx, V2Type v2_t, double* vvs_mtx_out, VVType vv_t) noexcept->void
		{
			for (Size i(-1), vid(0), vvid(0); ++i < n; vid = next_cid(vid, v2_t), vvid = next_cid(vvid, vv_t))
				s_cvs(vs, v_t, vs_mtx + vid, v2_t, vvs_mtx_out + vvid, vv_t);
		}
		/// \brief 构造6x6的力转换矩阵
		///
		///  其中,tmf = [rm (3x3),  pp x rm (3x3); O (3x3), rm (3x3)]
		///
		///
		auto s_tmf(const double *pm, double *tmf_out) noexcept->void;
		/// \brief 根据位姿矩阵转换六维力向量
		///
		/// 等同于： fs_out = tmf(pm) * fs
		///
		///
		auto s_tf(const double *pm, const double *fs, double *fs_out) noexcept->void;
		/// \brief 根据位姿矩阵转换六维力向量
		///
		/// 等同于： fs_out = tmf(pm) * fs
		///
		///
		template<typename F1Type, typename F2Type>
		auto s_tf(const double *pm, const double *fs, F1Type f1_t, double *fs_out, F2Type f2_t) noexcept->void
		{
			Size f13_pos{ id(3, f1_t) }, f23_pos{ id(3,f2_t) };
			
			s_pm_dot_v3(pm, fs, f1_t, fs_out, f2_t);
			s_pm_dot_v3(pm, fs + f13_pos, f1_t, fs_out + f23_pos, f2_t);
			s_c3a(pm + 3, 4, fs_out, f2_t, fs_out + f23_pos, f2_t);
		}
		/// \brief 根据位姿矩阵转换六维力向量
		///
		/// 等同于： fs_out = alpha * tmf(pm) * fs
		///
		///
		auto s_tf(double alpha, const double *pm, const double *fs, double *fs_out) noexcept->void;
		/// \brief 根据位姿矩阵转换六维力向量
		///
		/// 等同于： fs_out = alpha * tmf(pm) * fs
		///
		///
		template<typename F1Type, typename F2Type>
		auto s_tf(double alpha, const double *pm, const double *fs, F1Type f1_t, double *fs_out, F2Type f2_t) noexcept->void
		{
			s_tf(pm, fs, f1_t, fs_out, f2_t);
			s_nv(6, alpha, fs_out, f2_t);
		}
		/// \brief 根据位姿矩阵转换六维力向量
		///
		/// 等同于： fs_out = tmf(pm) * fs + fs_out
		///
		///
		auto s_tfa(const double *pm, const double *fs, double *fs_out) noexcept->void;
		/// \brief 根据位姿矩阵转换六维力向量
		///
		/// 等同于： fs_out = tmf(pm) * fs + fs_out
		///
		///
		template<typename F1Type, typename F2Type>
		auto s_tfa(const double *pm, const double *fs, F1Type f1_t, double *fs_out, F2Type f2_t) noexcept->void
		{
			double tem[6];
			s_tf(pm, fs, f1_t, tem, 1);
			s_va(6, tem, 1, fs_out, f2_t);
		}
		/// \brief 根据位姿矩阵转换六维力向量
		///
		/// 等同于： fs_out = alpha * tmf(pm) * fs + beta * fs_out
		///
		///
		auto s_tfa(double alpha, const double *pm, const double *fs, double *fs_out) noexcept->void;
		/// \brief 根据位姿矩阵转换六维力向量
		///
		/// 等同于： fs_out = alpha * tmf(pm) * fs + beta * fs_out
		///
		///
		template<typename F1Type, typename F2Type>
		auto s_tfa(double alpha, const double *pm, const double *fs, F1Type f1_t, double *fs_out, F2Type f2_t) noexcept->void
		{
			double tem[6];
			s_tf(pm, fs, f1_t, tem, 1);
			s_va(6, alpha, tem, 1, fs_out, f2_t);
		}
		/// \brief 根据位姿矩阵的逆矩阵转换六维力向量
		///
		/// 等同于： fs_out = tmv(pm^-1) * fs
		///
		///
		auto s_inv_tf(const double *inv_pm, const double *fs, double *fs_out) noexcept->void;
		/// \brief 根据位姿矩阵的逆矩阵转换六维力向量
		///
		/// 等同于： fs_out = tmv(pm^-1) * fs
		///
		///
		template<typename F1Type, typename F2Type>
		auto s_inv_tf(const double *inv_pm, const double *fs, F1Type f1_t, double *fs_out, F2Type f2_t) noexcept->void
		{
			s_c3i(inv_pm + 3, 4, fs, f1_t, fs_out, f2_t);
			s_va(3, fs + id(3, f1_t), f1_t, fs_out, f2_t);

			s_inv_pm_dot_v3(inv_pm, fs_out, f2_t, fs_out + id(3, f2_t), f2_t);
			s_inv_pm_dot_v3(inv_pm, fs, f1_t, fs_out, f2_t);

		}
		/// \brief 根据位姿矩阵的逆矩阵转换六维力向量
		///
		/// 等同于： fs_out = alpha * tmv(pm^-1) * fs
		///
		///
		auto s_inv_tf(double alpha, const double *inv_pm, const double *fs, double *fs_out) noexcept->void;
		/// \brief 根据位姿矩阵的逆矩阵转换六维力向量
		///
		/// 等同于： fs_out = alpha * tmv(pm^-1) * fs
		///
		///
		template<typename F1Type, typename F2Type>
		auto s_inv_tf(double alpha, const double *inv_pm, const double *fs, F1Type f1_t, double *fs_out, F2Type f2_t) noexcept->void
		{
			s_inv_tf(inv_pm, fs, f1_t, fs_out, f2_t);
			s_nv(6, alpha, fs_out, f2_t);
		}
		/// \brief 根据位姿矩阵转换六维力向量
		///
		/// 等同于： fs_out = tmf(pm^-1) * fs + fs_out
		///
		///
		auto s_inv_tfa(const double *inv_pm, const double *fs, double *fs_out) noexcept->void;
		/// \brief 根据位姿矩阵转换六维力向量
		///
		/// 等同于： fs_out = tmf(pm^-1) * fs + fs_out
		///
		///
		template<typename F1Type, typename F2Type>
		auto s_inv_tfa(const double *inv_pm, const double *fs, F1Type f1_t, double *fs_out, F2Type f2_t) noexcept->void
		{
			double tem[6];
			s_inv_tf(inv_pm, fs, f1_t, tem, 1);
			s_va(6, tem, 1, fs_out, f2_t);
		}
		/// \brief 根据位姿矩阵转换六维力向量
		///
		/// 等同于： fs_out = alpha * tmf(pm^-1) * fs + beta * fs_out
		///
		///
		auto s_inv_tfa(double alpha, const double *inv_pm, const double *fs, double *fs_out) noexcept->void;
		/// \brief 根据位姿矩阵转换六维力向量
		///
		/// 等同于： fs_out = alpha * tmf(pm^-1) * fs + beta * fs_out
		///
		///
		template<typename F1Type, typename F2Type>
		auto s_inv_tfa(double alpha, const double *inv_pm, const double *fs, F1Type f1_t, double *fs_out, F2Type f2_t) noexcept->void
		{
			double tem[6];
			s_inv_tf(inv_pm, fs, f1_t, tem, 1);
			s_va(6, alpha, tem, 1, fs_out, f2_t);
		}
		/// \brief 构造6x6的速度转换矩阵
		///
		///  其中,tmv = [rm (3x3),  O (3x3); pp x rm (3x3), rm (3x3)]
		///
		///
		auto s_tmv(const double *pm, double *tmv_out) noexcept->void;
		/// \brief 根据位姿矩阵转换六维速度向量
		///
		/// 等同于： vec_out = tmv(pm_in) * vs_in
		///
		///
		auto s_tv(const double *pm, const double *vs, double *vs_out) noexcept->void;
		/// \brief 根据位姿矩阵转换六维速度向量
		///
		/// 等同于： vs_out = tmv(pm) * vs
		///
		///
		template<typename V1Type, typename V2Type>
		auto s_tv(const double *pm, const double *vs, V1Type v1_t, double *vs_out, V2Type v2_t) noexcept->void
		{
			s_pm_dot_v3(pm, vs, v1_t, vs_out, v2_t);
			s_pm_dot_v3(pm, vs + id(3, v1_t), v1_t, vs_out + id(3, v2_t), v2_t);
			s_c3a(pm + 3, 4, vs_out + id(3, v2_t), v2_t, vs_out, v2_t);

		}
		/// \brief 根据位姿矩阵转换六维速度向量
		///
		/// 等同于： vs_out = alpha * tmv(pm) * vs
		///
		///
		auto s_tv(double alpha, const double *pm, const double *vs, double *vs_out) noexcept->void;
		/// \brief 根据位姿矩阵转换六维速度向量
		///
		/// 等同于： vs_out = alpha * tmv(pm) * vs
		///
		///
		template<typename V1Type, typename V2Type>
		auto s_tv(double alpha, const double *pm, const double *vs, V1Type v1_t, double *vs_out, V2Type v2_t) noexcept->void
		{
			s_tv(pm, vs, v1_t, vs_out, v2_t);
			s_nv(6, alpha, vs_out, v2_t);
		}
		/// \brief 根据位姿矩阵转换六维速度向量
		///
		/// 等同于： vs_out = tmv(pm) * vs + vs_out
		///
		///
		auto s_tva(const double *pm, const double *vs, double *vs_out) noexcept->void;
		/// \brief 根据位姿矩阵转换六维速度向量
		///
		/// 等同于： vs_out = tmv(pm) * vs + vs_out
		///
		///
		template<typename V1Type, typename V2Type>
		auto s_tva(const double *pm, const double *vs, V1Type v1_t, double *vs_out, V2Type v2_t) noexcept->void
		{
			double tem[6];
			s_tv(pm, vs, v1_t, tem, 1);
			s_va(6, tem, 1, vs_out, v2_t);
		}
		/// \brief 根据位姿矩阵转换六维速度向量
		///
		/// 等同于： vs_out = alpha * tmv(pm) * vs + beta * vs_out
		///
		///
		auto s_tva(double alpha, const double *pm, const double *vs, double *vs_out) noexcept->void;
		/// \brief 根据位姿矩阵转换六维速度向量
		///
		/// 等同于： vs_out = alpha * tmv(pm) * vs + beta * vs_out
		///
		///
		template<typename V1Type, typename V2Type>
		auto s_tva(double alpha, const double *pm, const double *vs, V1Type v1_t, double *vs_out, V2Type v2_t) noexcept->void
		{
			double tem[6];
			s_tv(pm, vs, v1_t, tem, 1);
			s_va(6, alpha, tem, 1, vs_out, v2_t);
		}
		/// \brief 根据位姿矩阵的逆矩阵转换六维速度向量
		///
		/// 等同于： vs_out = tmv(pm^-1) * vs
		///
		///
		auto s_inv_tv(const double *inv_pm, const double *vs, double *vs_out) noexcept->void;
		/// \brief 根据位姿矩阵的逆矩阵转换六维速度向量
		///
		/// 等同于： vs_out = tmv(pm^-1) * vs
		///
		///
		template<typename V1Type, typename V2Type>
		auto s_inv_tv(const double *inv_pm, const double *vs, V1Type v1_t, double *vs_out, V2Type v2_t) noexcept->void
		{
			const double* vs3{ vs + id(3, v1_t) };
			double* vs_out3{ vs_out + id(3, v2_t) };

			s_c3i(inv_pm + 3, 4, vs3, v1_t, vs_out3, v2_t);
			s_va(3, vs, v1_t, vs_out3, v2_t);

			s_inv_pm_dot_v3(inv_pm, vs_out3, v2_t, vs_out, v2_t);
			s_inv_pm_dot_v3(inv_pm, vs3, v1_t, vs_out3, v2_t);
		}
		/// \brief 根据位姿矩阵的逆矩阵转换六维速度向量
		///
		/// 等同于： vs_out = alpha * tmv(pm^-1) * vs
		///
		///
		auto s_inv_tv(double alpha, const double *inv_pm, const double *vs, double *vs_out) noexcept->void;
		/// \brief 根据位姿矩阵的逆矩阵转换六维速度向量
		///
		/// 等同于： vs_out = alpha * tmv(pm^-1) * vs
		///
		///
		template<typename V1Type, typename V2Type>
		auto s_inv_tv(double alpha, const double *inv_pm, const double *vs, V1Type v1_t, double *vs_out, V2Type v2_t) noexcept->void
		{
			s_inv_tv(inv_pm, vs, v1_t, vs_out, v2_t);
			s_nv(6, alpha, vs_out, v2_t);
		}
		/// \brief 根据位姿矩阵转换六维速度向量
		///
		/// 等同于： vs_out = tmv(pm^-1) * vs + vs_out
		///
		///
		auto s_inv_tva(const double *inv_pm, const double *vs, double *vs_out) noexcept->void;
		/// \brief 根据位姿矩阵转换六维速度向量
		///
		/// 等同于： vs_out = tmv(pm^-1) * vs + vs_out
		///
		///
		template<typename V1Type, typename V2Type>
		auto s_inv_tva(const double *inv_pm, const double *vs, V1Type v1_t, double *vs_out, V2Type v2_t) noexcept->void
		{
			double tem[6];
			s_inv_tv(inv_pm, vs, v1_t, tem, 1);
			s_va(6, tem, 1, vs_out, v2_t);
		}
		/// \brief 根据位姿矩阵转换六维速度向量
		///
		/// 等同于： vs_out = alpha * tmv(pm^-1) * vs + beta * vs_out
		///
		///
		auto s_inv_tva(double alpha, const double *inv_pm, const double *vs, double *vs_out) noexcept->void;
		/// \brief 根据位姿矩阵转换六维速度向量
		///
		/// 等同于： vs_out = alpha * tmv(pm^-1) * vs + beta * vs_out
		///
		///
		template<typename V1Type, typename V2Type>
		auto s_inv_tva(double alpha, const double *inv_pm, const double *vs, V1Type v1_t, double *vs_out, V2Type v2_t) noexcept->void
		{
			double tem[6];
			s_inv_tv(inv_pm, vs, v1_t, tem, 1);
			s_va(6, alpha, tem, 1, vs_out, v2_t);
		}
		/// \brief 根据位姿矩阵转换六维力矩阵
		///
		/// 等同于： fs_mtx_out = tmf(pm) * fs_mtx
		///
		///
		auto inline s_tf_n(Size n, const double *pm, const double *fs_mtx, double *fs_mtx_out) noexcept->void { for (Size i = 0; i < n; ++i)s_tf(pm, fs_mtx + i, n, fs_mtx_out + i, n); }
		/// \brief 根据位姿矩阵转换六维力矩阵
		///
		/// 等同于： fs_mtx_out = tmf(pm) * fs_mtx
		///
		///
		template<typename F1Type, typename F2Type>
		auto inline s_tf_n(Size n, const double *pm, const double *fs_mtx, F1Type f1_t, double *fs_mtx_out, F2Type f2_t) noexcept->void 
		{ 
			for (Size i(-1), f1id(0), f2id(0); ++i < n; f1id = next_cid(f1id, f1_t), f2id = next_cid(f2id, f2_t))
				s_tf(pm, fs_mtx + f1id, f1_t, fs_mtx_out + f2id, f2_t);
		}
		/// \brief 根据位姿矩阵转换六维力矩阵
		///
		/// 等同于： fs_mtx_out = alpha * tmf(pm) * fs_mtx
		///
		///
		auto inline s_tf_n(Size n, double alpha, const double *pm, const double *fs_mtx, double *fs_mtx_out) noexcept->void { for (Size i = 0; i < n; ++i)s_tf(alpha, pm, fs_mtx + i, n, fs_mtx_out + i, n); }
		/// \brief 根据位姿矩阵转换六维力矩阵
		///
		/// 等同于： fs_mtx_out = alpha * tmf(pm) * fs_mtx
		///
		///
		template<typename F1Type, typename F2Type>
		auto inline s_tf_n(Size n, double alpha, const double *pm, const double *fs_mtx, F1Type f1_t, double *fs_mtx_out, F2Type f2_t) noexcept->void
		{
			for (Size i(-1), f1id(0), f2id(0); ++i < n; f1id = next_cid(f1id, f1_t), f2id = next_cid(f2id, f2_t))
				s_tf(alpha, pm, fs_mtx + f1id, f1_t, fs_mtx_out + f2id, f2_t);
		}
		/// \brief 根据位姿矩阵转换六维力矩阵
		///
		/// 等同于： fs_mtx_out = tmf(pm) * fs_mtx + fs_mtx_out
		///
		///
		auto inline s_tfa_n(Size n, const double *pm, const double *fs_mtx, double *fs_mtx_out) noexcept->void { for (Size i = 0; i < n; ++i)s_tfa(pm, fs_mtx + i, n, fs_mtx_out + i, n); }
		/// \brief 根据位姿矩阵转换六维力矩阵
		///
		/// 等同于： fs_mtx_out = tmf(pm) * fs_mtx + fs_mtx_out
		///
		///
		template<typename F1Type, typename F2Type>
		auto inline s_tfa_n(Size n, const double *pm, const double *fs_mtx, F1Type f1_t, double *fs_mtx_out, F2Type f2_t) noexcept->void
		{
			for (Size i(-1), f1id(0), f2id(0); ++i < n; f1id = next_cid(f1id, f1_t), f2id = next_cid(f2id, f2_t))
				s_tfa(pm, fs_mtx + f1id, f1_t, fs_mtx_out + f2id, f2_t);
		}
		/// \brief 根据位姿矩阵转换六维力矩阵
		///
		/// 等同于： fs_mtx_out = alpha * tmf(pm) * fs_mtx + beta * fs_mtx_out
		///
		///
		auto inline s_tfa_n(Size n, double alpha, const double *pm, const double *fs_mtx, double *fs_mtx_out) noexcept->void { for (Size i = 0; i < n; ++i)s_tfa(alpha, pm, fs_mtx + i, n, fs_mtx_out + i, n); }
		/// \brief 根据位姿矩阵转换六维力矩阵
		///
		/// 等同于： fs_mtx_out = alpha * tmf(pm) * fs_mtx + beta * fs_mtx_out
		///
		///
		template<typename F1Type, typename F2Type>
		auto inline s_tfa_n(Size n, double alpha, const double *pm, const double *fs_mtx, F1Type f1_t, double *fs_mtx_out, F2Type f2_t) noexcept->void
		{
			for (Size i(-1), f1id(0), f2id(0); ++i < n; f1id = next_cid(f1id, f1_t), f2id = next_cid(f2id, f2_t))
				s_tfa(alpha, pm, fs_mtx + f1id, f1_t, fs_mtx_out + f2id, f2_t);
		}
		/// \brief 根据位姿矩阵转换六维力矩阵
		///
		/// 等同于： fs_mtx_out = tmf(inv_pm^-1) * fs_mtx
		///
		///
		auto inline s_inv_tf_n(Size n, const double *inv_pm, const double *fs_mtx, double *fs_mtx_out) noexcept->void { for (Size i = 0; i < n; ++i)s_inv_tf(inv_pm, fs_mtx + i, n, fs_mtx_out + i, n); }
		/// \brief 根据位姿矩阵转换六维力矩阵
		///
		/// 等同于： fs_mtx_out = tmf(inv_pm^-1) * fs_mtx
		///
		///
		template<typename F1Type, typename F2Type>
		auto inline s_inv_tf_n(Size n, const double *inv_pm, const double *fs_mtx, F1Type f1_t, double *fs_mtx_out, F2Type f2_t) noexcept->void 
		{
			for (Size i(-1), f1id(0), f2id(0); ++i < n; f1id = next_cid(f1id, f1_t), f2id = next_cid(f2id, f2_t))
				s_inv_tf(inv_pm, fs_mtx + f1id, f1_t, fs_mtx_out + f2id, f2_t);
		}
		/// \brief 根据位姿矩阵转换六维力矩阵
		///
		/// 等同于： fs_mtx_out = alpha * tmf(inv_pm^-1) * fs_mtx
		///
		///
		auto inline s_inv_tf_n(Size n, double alpha, const double *inv_pm, const double *fs_mtx, double *fs_mtx_out) noexcept->void { for (Size i = 0; i < n; ++i)s_inv_tf(alpha, inv_pm, fs_mtx + i, n, fs_mtx_out + i, n); }
		/// \brief 根据位姿矩阵转换六维力矩阵
		///
		/// 等同于： fs_mtx_out = alpha * tmf(inv_pm^-1) * fs_mtx
		///
		///
		template<typename F1Type, typename F2Type>
		auto inline s_inv_tf_n(Size n, double alpha, const double *inv_pm, const double *fs_mtx, F1Type f1_t, double *fs_mtx_out, F2Type f2_t) noexcept->void
		{
			for (Size i(-1), f1id(0), f2id(0); ++i < n; f1id = next_cid(f1id, f1_t), f2id = next_cid(f2id, f2_t))
				s_inv_tf(alpha, inv_pm, fs_mtx + f1id, f1_t, fs_mtx_out + f2id, f2_t);
		}
		/// \brief 根据位姿矩阵转换六维力矩阵
		///
		/// 等同于： fs_mtx_out = tmf(pm^-1) * fs_mtx + fs_mtx_out
		///
		///
		auto inline s_inv_tfa_n(Size n, const double *inv_pm, const double *fs_mtx, double *fs_mtx_out) noexcept->void { for (Size i = 0; i < n; ++i)s_inv_tfa(inv_pm, fs_mtx + i, n, fs_mtx_out + i, n); }
		/// \brief 根据位姿矩阵转换六维力矩阵
		///
		/// 等同于： fs_mtx_out = tmf(pm^-1) * fs_mtx + fs_mtx_out
		///
		///
		template<typename F1Type, typename F2Type>
		auto inline s_inv_tfa_n(Size n, const double *inv_pm, const double *fs_mtx, F1Type f1_t, double *fs_mtx_out, F2Type f2_t) noexcept->void
		{
			for (Size i(-1), f1id(0), f2id(0); ++i < n; f1id = next_cid(f1id, f1_t), f2id = next_cid(f2id, f2_t))
				s_inv_tfa(inv_pm, fs_mtx + f1id, f1_t, fs_mtx_out + f2id, f2_t);
		}
		/// \brief 根据位姿矩阵转换六维力矩阵
		///
		/// 等同于： fs_mtx_out = alpha * tmf(pm^-1) * fs_mtx + beta * fs_mtx_out
		///
		///
		auto inline s_inv_tfa_n(Size n, double alpha, const double *inv_pm, const double *fs_mtx, double *fs_mtx_out) noexcept->void { for (Size i = 0; i < n; ++i)s_inv_tfa(alpha, inv_pm, fs_mtx + i, n, fs_mtx_out + i, n); }
		/// \brief 根据位姿矩阵转换六维力矩阵
		///
		/// 等同于： fs_mtx_out = alpha * tmf(pm^-1) * fs_mtx + beta * fs_mtx_out
		///
		///
		template<typename F1Type, typename F2Type>
		auto inline s_inv_tfa_n(Size n, double alpha, const double *inv_pm, const double *fs_mtx, F1Type f1_t, double *fs_mtx_out, F2Type f2_t) noexcept->void
		{
			for (Size i(-1), f1id(0), f2id(0); ++i < n; f1id = next_cid(f1id, f1_t), f2id = next_cid(f2id, f2_t))
				s_inv_tfa(alpha, inv_pm, fs_mtx + f1id, f1_t, fs_mtx_out + f2id, f2_t);
		}
		/// \brief 根据位姿矩阵转换六维力矩阵
		///
		/// 等同于： m_out = tmv(pm) * fces_in
		///
		///
		auto inline s_tv_n(Size n, const double *pm, const double *vs_mtx, double *vs_mtx_out) noexcept->void { for (Size i = 0; i < n; ++i)s_tv(pm, vs_mtx + i, n, vs_mtx_out + i, n); }
		/// \brief 根据位姿矩阵转换六维力矩阵
		///
		/// 等同于： m_out = tmv(pm) * fces_in
		///
		///
		template<typename V1Type, typename V2Type>
		auto inline s_tv_n(Size n, const double *pm, const double *vs_mtx, V1Type v1_t, double *vs_mtx_out, V2Type v2_t) noexcept->void
		{
			for (Size i(-1), v1id(0), v2id(0); ++i < n; v1id = next_cid(v1id, v1_t), v2id = next_cid(v2id, v2_t))
				s_tv(pm, vs_mtx + v1id, v1_t, vs_mtx_out + v2id, v2_t);
		}
		/// \brief 根据位姿矩阵转换六维速度向量
		///
		/// 等同于： vs_out = alpha * tmv(pm) * vs
		///
		///
		auto inline s_tv_n(Size n, double alpha, const double *pm, const double *vs_mtx, double *vs_mtx_out) noexcept->void { for (Size i = 0; i < n; ++i)s_tv(alpha, pm, vs_mtx + i, n, vs_mtx_out + i, n); }
		/// \brief 根据位姿矩阵转换六维速度向量
		///
		/// 等同于： vs_out = alpha * tmv(pm) * vs
		///
		///
		template<typename V1Type, typename V2Type>
		auto inline s_tv_n(Size n, double alpha, const double *pm, const double *vs_mtx, V1Type v1_t, double *vs_mtx_out, V2Type v2_t) noexcept->void 
		{
			for (Size i(-1), v1id(0), v2id(0); ++i < n; v1id = next_cid(v1id, v1_t), v2id = next_cid(v2id, v2_t))
				s_tv(alpha, pm, vs_mtx + v1id, v1_t, vs_mtx_out + v2id, v2_t);
		}
		/// \brief 根据位姿矩阵转换六维力矩阵
		///
		/// 等同于： m_out = alpha * tmv(pm) * fces_in + beta * m_out
		///
		///
		auto inline s_tva_n(Size n, const double *pm, const double *vs_mtx, double *vs_mtx_out) noexcept->void { for (Size i = 0; i < n; ++i)s_tva(pm, vs_mtx + i, n, vs_mtx_out + i, n); }
		/// \brief 根据位姿矩阵转换六维力矩阵
		///
		/// 等同于： m_out = alpha * tmv(pm) * fces_in + beta * m_out
		///
		///
		template<typename V1Type, typename V2Type>
		auto inline s_tva_n(Size n, const double *pm, const double *vs_mtx, V1Type v1_t, double *vs_mtx_out, V2Type v2_t) noexcept->void
		{
			for (Size i(-1), v1id(0), v2id(0); ++i < n; v1id = next_cid(v1id, v1_t), v2id = next_cid(v2id, v2_t))
				s_tva(pm, vs_mtx + v1id, v1_t, vs_mtx_out + v2id, v2_t);
		}
		/// \brief 根据位姿矩阵转换六维力矩阵
		///
		/// 等同于： m_out = alpha * tmv(pm) * fces_in + beta * m_out
		///
		///
		auto inline s_tva_n(Size n, double alpha, const double *pm, const double *vs_mtx, double *vs_mtx_out) noexcept->void { for (Size i = 0; i < n; ++i)s_tva(alpha, pm, vs_mtx + i, n, vs_mtx_out + i, n); }
		/// \brief 根据位姿矩阵转换六维力矩阵
		///
		/// 等同于： m_out = alpha * tmv(pm) * fces_in + beta * m_out
		///
		///
		template<typename V1Type, typename V2Type>
		auto inline s_tva_n(Size n, double alpha, const double *pm, const double *vs_mtx, V1Type v1_t, double *vs_mtx_out, V2Type v2_t) noexcept->void
		{
			for (Size i(-1), v1id(0), v2id(0); ++i < n; v1id = next_cid(v1id, v1_t), v2id = next_cid(v2id, v2_t))
				s_tva(alpha, pm, vs_mtx + v1id, v1_t, vs_mtx_out + v2id, v2_t);
		}
		/// \brief 根据位姿矩阵转换六维速度矩阵
		///
		/// 等同于： vs_mtx_out = tmv(inv_pm^-1) * vs_mtx
		///
		///
		auto inline s_inv_tv_n(Size n, const double *inv_pm, const double *vs_mtx, double *vs_mtx_out) noexcept->void { for (Size i = 0; i < n; ++i)s_inv_tv(inv_pm, vs_mtx + i, n, vs_mtx_out + i, n); }
		/// \brief 根据位姿矩阵转换六维速度矩阵
		///
		/// 等同于： vs_mtx_out = tmv(inv_pm^-1) * vs_mtx
		///
		///
		template<typename V1Type, typename V2Type>
		auto inline s_inv_tv_n(Size n, const double *inv_pm, const double *vs_mtx, V1Type v1_t, double *vs_mtx_out, V2Type v2_t) noexcept->void
		{
			for (Size i(-1), v1id(0), v2id(0); ++i < n; v1id = next_cid(v1id, v1_t), v2id = next_cid(v2id, v2_t))
				s_inv_tv(inv_pm, vs_mtx + v1id, v1_t, vs_mtx_out + v2id, v2_t);
		}
		/// \brief 根据位姿矩阵转换六维速度矩阵
		///
		/// 等同于： vs_mtx_out = alpha * tmv(inv_pm^-1) * vs_mtx
		///
		///
		auto inline s_inv_tv_n(Size n, double alpha, const double *inv_pm, const double *vs_mtx, double *vs_mtx_out) noexcept->void { for (Size i = 0; i < n; ++i)s_inv_tv(alpha, inv_pm, vs_mtx + i, n, vs_mtx_out + i, n); }
		/// \brief 根据位姿矩阵转换六维速度矩阵
		///
		/// 等同于： vs_mtx_out = alpha * tmv(inv_pm^-1) * vs_mtx
		///
		///
		template<typename V1Type, typename V2Type>
		auto inline s_inv_tv_n(Size n, double alpha, const double *inv_pm, const double *vs_mtx, V1Type v1_t, double *vs_mtx_out, V2Type v2_t) noexcept->void
		{
			for (Size i(-1), v1id(0), v2id(0); ++i < n; v1id = next_cid(v1id, v1_t), v2id = next_cid(v2id, v2_t))
				s_inv_tv(alpha, inv_pm, vs_mtx + v1id, v1_t, vs_mtx_out + v2id, v2_t);
		}
		/// \brief 根据位姿矩阵转换六维速度矩阵
		///
		/// 等同于： vs_mtx_out = tmv(pm^-1) * vs_mtx + vs_mtx_out
		///
		///
		auto inline s_inv_tva_n(Size n, const double *inv_pm, const double *vs_mtx, double *vs_mtx_out) noexcept->void { for (Size i = 0; i < n; ++i)s_inv_tva(inv_pm, vs_mtx + i, n, vs_mtx_out + i, n); }
		/// \brief 根据位姿矩阵转换六维速度矩阵
		///
		/// 等同于： vs_mtx_out = tmv(pm^-1) * vs_mtx + vs_mtx_out
		///
		///
		template<typename V1Type, typename V2Type>
		auto inline s_inv_tva_n(Size n, const double *inv_pm, const double *vs_mtx, V1Type v1_t, double *vs_mtx_out, V2Type v2_t) noexcept->void
		{
			for (Size i(-1), v1id(0), v2id(0); ++i < n; v1id = next_cid(v1id, v1_t), v2id = next_cid(v2id, v2_t))
				s_inv_tva(inv_pm, vs_mtx + v1id, v1_t, vs_mtx_out + v2id, v2_t);
		}
		/// \brief 根据位姿矩阵转换六维速度矩阵
		///
		/// 等同于： vs_mtx_out = alpha * tmv(pm^-1) * vs_mtx + beta * vs_mtx_out
		///
		///
		auto inline s_inv_tva_n(Size n, double alpha, const double *inv_pm, const double *vs_mtx, double *vs_mtx_out) noexcept->void { for (Size i = 0; i < n; ++i)s_inv_tva(alpha, inv_pm, vs_mtx + i, n, vs_mtx_out + i, n); }
		/// \brief 根据位姿矩阵转换六维速度矩阵
		///
		/// 等同于： vs_mtx_out = alpha * tmv(pm^-1) * vs_mtx + beta * vs_mtx_out
		///
		///
		template<typename V1Type, typename V2Type>
		auto inline s_inv_tva_n(Size n, double alpha, const double *inv_pm, const double *vs_mtx, V1Type v1_t, double *vs_mtx_out, V2Type v2_t) noexcept->void
		{
			for (Size i(-1), v1id(0), v2id(0); ++i < n; v1id = next_cid(v1id, v1_t), v2id = next_cid(v2id, v2_t))
				s_inv_tva(alpha, inv_pm, vs_mtx + v1id, v1_t, vs_mtx_out + v2id, v2_t);
		}

		// 以下函数为物理量之间的转换函数 //
		auto s_ra2rm(const double *ra_in, double *rm_out = nullptr, Size rm_ld = 3) noexcept->double *;
		auto s_rm2ra(const double *rm_in, double *ra_out = nullptr, Size rm_ld = 3) noexcept->double *;
		auto s_re2rm(const double *re_in, double *rm_out = nullptr, const char *eu_type_in = "313", Size rm_ld = 3) noexcept->double *;
		auto s_rm2re(const double *rm_in, double *re_out = nullptr, const char *eu_type_in = "313", Size rm_ld = 3) noexcept->double *;
		auto s_rq2rm(const double *rq_in, double *rm_out = nullptr, Size rm_ld = 3) noexcept->double *;
		auto s_rm2rq(const double *rm_in, double *rq_out = nullptr, Size rm_ld = 3) noexcept->double *;
		auto s_pp2pm(const double *pp_in, double *pm_out = nullptr) noexcept->double *;
		auto s_pm2pp(const double *pm_in, double *pp_out = nullptr) noexcept->double *;
		auto s_ra2pm(const double *ra_in, double *rm_out = nullptr) noexcept->double *;
		auto s_pm2ra(const double *rm_in, double *ra_out = nullptr) noexcept->double *;
		auto s_re2pm(const double *re_in, double *pm_out = nullptr, const char *eu_type_in = "313") noexcept->double *;
		auto s_pm2re(const double *pm_in, double *re_out = nullptr, const char *eu_type_in = "313") noexcept->double *;
		auto s_pa2pm(const double *pa_in, double *pm_out = nullptr) noexcept->double *;
		auto s_pm2pa(const double *pm_in, double *pa_out = nullptr) noexcept->double *;
		auto s_rq2pm(const double *rq_in, double *pm_out = nullptr) noexcept->double *;
		auto s_pm2rq(const double *pm_in, double *rq_out = nullptr) noexcept->double *;
		auto s_rm2pm(const double *rm_in, double *pm_out = nullptr, Size rm_ld = 3) noexcept->double *;
		auto s_pm2rm(const double *pm_in, double *rm_out = nullptr, Size rm_ld = 3) noexcept->double *;
		auto s_pe2pm(const double *pe_in, double *pm_out = nullptr, const char *eu_type_in = "313") noexcept->double *;
		auto s_pm2pe(const double *pm_in, double *pe_out = nullptr, const char *eu_type_in = "313") noexcept->double *;
		auto s_pq2pm(const double *pq_in, double *pm_out = nullptr) noexcept->double *;
		auto s_pm2pq(const double *pm_in, double *pq_out = nullptr) noexcept->double *;
		auto s_ps2pm(const double *ps_in, double *pm_out = nullptr) noexcept->double *;
		auto s_pm2ps(const double *pm_in, double *ps_out = nullptr) noexcept->double *;
		
		auto s_we2wa(const double *re_in, const double *we_in, double *wa_out = nullptr, const char *eu_type_in = "313") noexcept->double *;
		auto s_wa2we(const double *wa_in, const double *re_in, double *we_out = nullptr, const char *eu_type_in = "313") noexcept->double *;
		auto s_wq2wa(const double *rq_in, const double *wq_in, double *wa_out = nullptr) noexcept->double *;
		auto s_wa2wq(const double *wa_in, const double *rq_in, double *wq_out = nullptr) noexcept->double *;
		auto s_wm2wa(const double *rm_in, const double *wm_in, double *wa_out = nullptr, Size rm_ld = 3, Size wm_ld = 3) noexcept->double *;
		auto s_wa2wm(const double *wa_in, const double *rm_in, double *wm_out = nullptr, Size rm_ld = 3, Size wm_ld = 3) noexcept->double *;
		auto s_vp2vs(const double *pp_in, const double *vp_in, double *vs_out = nullptr) noexcept->double *;
		auto s_vs2vp(const double *vs_in, const double *pp_in, double *vp_out = nullptr) noexcept->double *;
		auto s_we2vs(const double *re_in, const double *we_in, double *vs_out = nullptr, const char *eu_type_in = "313") noexcept->double *;
		auto s_vs2we(const double *vs_in, const double *re_in, double *we_out = nullptr, const char *eu_type_in = "313") noexcept->double *;
		auto s_wq2vs(const double *rq_in, const double *wq_in, double *vs_out = nullptr) noexcept->double *;
		auto s_vs2wq(const double *vs_in, const double *rq_in, double *wq_out = nullptr) noexcept->double *;
		auto s_wm2vs(const double *rm_in, const double *wm_in, double *vs_out = nullptr, Size rm_ld = 3, Size wm_ld = 3) noexcept->double *;
		auto s_vs2wm(const double *vs_in, const double *rm_in, double *wm_out = nullptr, Size rm_ld = 3, Size wm_ld = 3) noexcept->double *;
		auto s_wa2vs(const double *wa_in, double *vs_out = nullptr) noexcept->double *;
		auto s_vs2wa(const double *vs_in, double *wa_out = nullptr) noexcept->double *;
		auto s_ve2vs(const double *pe_in, const double *ve_in, double *vs_out = nullptr, const char *eu_type_in = "313") noexcept->double *;
		auto s_vs2ve(const double *vs_in, const double *pe_in, double *ve_out = nullptr, const char *eu_type_in = "313") noexcept->double *;
		auto s_vq2vs(const double *pq_in, const double *vq_in, double *vs_out = nullptr) noexcept->double *;
		auto s_vs2vq(const double *vs_in, const double *pq_in, double *vq_out = nullptr) noexcept->double *;
		auto s_vm2vs(const double *pm_in, const double *vm_in, double *vs_out = nullptr) noexcept->double *;
		auto s_vs2vm(const double *vs_in, const double *pm_in, double *vm_out = nullptr) noexcept->double *;
		auto s_va2vs(const double *pp_in, const double *va_in, double *vs_out = nullptr) noexcept->double *;
		auto s_vs2va(const double *vs_in, const double *pp_in, double *va_out = nullptr) noexcept->double *;
		
		auto s_xe2xa(const double *re_in, const double *we_in, const double *xe_in, double *xa_out = nullptr, double *wa_out = nullptr, const char *eu_type_in = "313") noexcept->double *;
		auto s_xa2xe(const double *wa_in, const double *xa_in, const double *re_in, double *xe_out = nullptr, double *we_out = nullptr, const char *eu_type_in = "313") noexcept->double *;
		auto s_xq2xa(const double *rq_in, const double *wq_in, const double *xq_in, double *xa_out = nullptr, double *wa_out = nullptr) noexcept->double *;
		auto s_xa2xq(const double *wa_in, const double *xa_in, const double *rq_in, double *xq_out = nullptr, double *wq_out = nullptr) noexcept->double *;
		auto s_xm2xa(const double *rm_in, const double *wm_in, const double *xm_in, double *xa_out = nullptr, double *wa_out = nullptr, Size rm_ld = 3, Size wm_ld = 3, Size xm_ld = 3) noexcept->double *;
		auto s_xa2xm(const double *wa_in, const double *xa_in, const double *rm_in, double *xm_out = nullptr, double *wm_out = nullptr, Size rm_ld = 3, Size wm_ld = 3, Size xm_ld = 3) noexcept->double *;
		auto s_ap2as(const double *pp_in, const double *vp_in, const double *ap_in, double *as_out = nullptr, double *vs_out = nullptr) noexcept->double *;
		auto s_as2ap(const double *vs_in, const double *as_in, const double *pp_in, double *ap_out = nullptr, double *vp_out = nullptr) noexcept->double *;
		auto s_xe2as(const double *re_in, const double *we_in, const double *xe_in, double *as_out = nullptr, double *vs_out = nullptr, const char *eu_type_in = "313") noexcept->double *;
		auto s_as2xe(const double *vs_in, const double *as_in, const double *re_in, double *xe_out = nullptr, double *we_out = nullptr, const char *eu_type_in = "313") noexcept->double *;
		auto s_xq2as(const double *rq_in, const double *wq_in, const double *xq_in, double *as_out = nullptr, double *vs_out = nullptr) noexcept->double *;
		auto s_as2xq(const double *vs_in, const double *as_in, const double *rq_in, double *xq_out = nullptr, double *wq_out = nullptr) noexcept->double *;
		auto s_xm2as(const double *rm_in, const double *wm_in, const double *xm_in, double *as_out = nullptr, double *vs_out = nullptr, Size rm_ld = 3, Size wm_ld = 3, Size xm_ld = 3) noexcept->double *;
		auto s_as2xm(const double *vs_in, const double *as_in, const double *rm_in, double *xm_out = nullptr, double *wm_out = nullptr, Size rm_ld = 3, Size wm_ld = 3, Size xm_ld = 3) noexcept->double *;
		auto s_xa2as(const double *xa_in, double *as_out = nullptr) noexcept->double *;
		auto s_as2xa(const double *as_in, double *xa_out = nullptr) noexcept->double *;
		auto s_ae2as(const double *pe_in, const double *ve_in, const double *ae_in, double *as_out = nullptr, double *vs_out = nullptr, const char *eu_type_in = "313") noexcept->double *;
		auto s_as2ae(const double *vs_in, const double *as_in, const double *pe_in, double *ae_out = nullptr, double *ve_out = nullptr, const char *eu_type_in = "313") noexcept->double *;
		auto s_aq2as(const double *pq_in, const double *vq_in, const double *aq_in, double *as_out = nullptr, double *vs_out = nullptr) noexcept->double *;
		auto s_as2aq(const double *vs_in, const double *as_in, const double *pq_in, double *aq_out = nullptr, double *vq_out = nullptr) noexcept->double *;
		auto s_am2as(const double *pm_in, const double *vm_in, const double *am_in, double *as_out = nullptr, double *vs_out = nullptr) noexcept->double *;
		auto s_as2am(const double *vs_in, const double *as_in, const double *pm_in, double *am_out = nullptr, double *vm_out = nullptr) noexcept->double *;
		auto s_aa2as(const double *pp_in, const double *va_in, const double *aa_in, double *as_out = nullptr, double *vs_out = nullptr) noexcept->double *;
		auto s_as2aa(const double *vs_in, const double *as_in, const double *pp_in, double *aa_out = nullptr, double *va_out = nullptr) noexcept->double *;

		auto s_pq2pe(const double *pq_in, double *pe_out = nullptr, const char *eu_type_in = "313") noexcept->double *;
		auto s_pe2pq(const double *pe_in, double *pq_out = nullptr, const char *eu_type_in = "313") noexcept->double *;
		auto s_iv2im(const double *iv_in, double *im_out = nullptr) noexcept->double *;
		auto s_im2iv(const double *im_in, double *iv_out = nullptr) noexcept->double *;
		auto s_i32im(const double mass_in, const double * in_in, const double *pm_in, double *is_out = nullptr) noexcept->double *;

		// 以下函数为同一物理量在不同坐标系之间的转换函数 //
		auto s_pp2pp(const double *relative_pm, const double *from_pp, double *to_pp) noexcept->double *;
		auto s_inv_pp2pp(const double *inv_relative_pm, const double *from_pp, double *to_pp) noexcept->double *;
		auto s_re2re(const double *relative_pm, const double *from_re, double *to_re, const char *from_eu_type = "313", const char *to_eu_type = "313") noexcept->double *;
		auto s_inv_re2re(const double *inv_relative_pm, const double *from_re, double *to_re, const char *from_eu_type = "313", const char *to_eu_type = "313") noexcept->double *;
		auto s_rq2rq(const double *relative_pm, const double *from_rq, double *to_rq) noexcept->double *;
		auto s_inv_rq2rq(const double *inv_relative_pm, const double *from_rq, double *to_rq) noexcept->double *;
		auto s_rm2rm(const double *relative_pm, const double *from_rm, double *to_rm, Size from_rm_ld = 3, Size to_rm_ld = 3) noexcept->double *;
		auto s_inv_rm2rm(const double *inv_relative_pm, const double *from_rm, double *to_rm, Size from_rm_ld = 3, Size to_rm_ld = 3) noexcept->double *;
		auto s_pe2pe(const double *relative_pm, const double *from_pe, double *to_pe, const char *from_pe_type = "313", const char *to_pe_type = "313") noexcept->double *;
		auto s_inv_pe2pe(const double *inv_relative_pm, const double *from_pe, double *to_pe, const char *from_pe_type = "313", const char *to_pe_type = "313") noexcept->double *;
		auto s_pq2pq(const double *relative_pm, const double *from_pq, double *to_pq) noexcept->double *;
		auto s_inv_pq2pq(const double *inv_relative_pm, const double *from_pq, double *to_pq) noexcept->double *;
		auto s_pm2pm(const double *relative_pm, const double *from_pm, double *to_pm) noexcept->double *;
		auto s_inv_pm2pm(const double *inv_relative_pm, const double *from_pm, double *to_pm) noexcept->double *;

		auto s_vp2vp(const double *relative_pm, const double *relative_vs, const double *from_pp, const double *from_vp, double *to_vp, double *to_pp = nullptr) noexcept->double *;
		auto s_inv_vp2vp(const double *inv_relative_pm, const double *inv_relative_vs, const double *from_pp, const double *from_vp, double *to_vp, double *to_pp = nullptr) noexcept->double *;
		auto s_we2we(const double *relative_pm, const double *relative_vs, const double *from_re, const double *from_we, double *to_we, double *to_re = nullptr, const char *from_eu_type = "313", const char *to_eu_type = "313") noexcept->double *;
		auto s_inv_we2we(const double *inv_relative_pm, const double *inv_relative_vs, const double *from_re, const double *from_we, double *to_we, double *to_re = nullptr, const char *from_eu_type = "313", const char *to_eu_type = "313") noexcept->double *;
		auto s_wq2wq(const double *relative_pm, const double *relative_vs, const double *from_rq, const double *from_wq, double *to_wq, double *to_rq = nullptr) noexcept->double *;
		auto s_inv_wq2wq(const double *inv_relative_pm, const double *inv_relative_vs, const double *from_rq, const double *from_wq, double *to_wq, double *to_rq = nullptr) noexcept->double *;
		auto s_wm2wm(const double *relative_pm, const double *relative_vs, const double *from_rm, const double *from_wm, double *to_wm, double *to_rm = nullptr) noexcept->double *;
		auto s_inv_wm2wm(const double *inv_relative_pm, const double *inv_relative_vs, const double *from_rm, const double *from_wm, double *to_wm, double *to_rm = nullptr) noexcept->double *;
		auto s_wa2wa(const double *relative_pm, const double *relative_vs, const double *from_wa, double *to_wa) noexcept->double *;
		auto s_inv_wa2wa(const double *inv_relative_pm, const double *inv_relative_vs, const double *from_wa, double *to_wa) noexcept->double *;
		auto s_ve2ve(const double *relative_pm, const double *relative_vs, const double *from_pe, const double *from_ve, double *to_ve, double *to_pe = nullptr, const char *from_eu_type = "313", const char *to_eu_type = "313") noexcept->double *;
		auto s_inv_ve2ve(const double *inv_relative_pm, const double *inv_relative_vs, const double *from_pe, const double *from_ve, double *to_ve, double *to_pe = nullptr, const char *from_eu_type = "313", const char *to_eu_type = "313") noexcept->double *;
		auto s_vq2vq(const double *relative_pm, const double *relative_vs, const double *from_pq, const double *from_vq, double *to_vq, double *to_pq = nullptr) noexcept->double *;
		auto s_inv_vq2vq(const double *inv_relative_pm, const double *inv_relative_vs, const double *from_pq, const double *from_vq, double *to_vq, double *to_pq = nullptr) noexcept->double *;
		auto s_vm2vm(const double *relative_pm, const double *relative_vs, const double *from_pm, const double *from_vm, double *to_vm, double *to_pm = nullptr) noexcept->double *;
		auto s_inv_vm2vm(const double *inv_relative_pm, const double *inv_relative_vs, const double *from_pm, const double *from_vm, double *to_vm, double *to_pm = nullptr) noexcept->double *;
		auto s_va2va(const double *relative_pm, const double *relative_vs, const double *from_pp, const double *from_va, double *to_va, double *to_pp = nullptr) noexcept->double *;
		auto s_inv_va2va(const double *inv_relative_pm, const double *inv_relative_vs, const double *from_pp, const double *from_va, double *to_va, double *to_pp = nullptr) noexcept->double *;
		auto s_vs2vs(const double *relative_pm, const double *relative_vs, const double *from_vs, double *to_vs) noexcept->double *;
		auto s_inv_vs2vs(const double *inv_relative_pm, const double *inv_relative_vs, const double *from_vs, double *to_vs) noexcept->double *;

		auto s_ap2ap(const double *relative_pm, const double *relative_vs, const double *relative_as, 
			const double *from_pp, const double *from_vp, const double *from_ap, double *to_ap, double *to_vp = nullptr, double *to_pp = nullptr) noexcept->double *;
		auto s_inv_ap2ap(const double *inv_relative_pm, const double *inv_relative_vs, const double *inv_relative_as, 
			const double *from_pp, const double *from_vp, const double *from_ap, double *to_ap, double *to_vp = nullptr, double *to_pp = nullptr) noexcept->double *;
		auto s_xe2xe(const double *relative_pm, const double *relative_vs, const double *relative_as,
			const double *from_re, const double *from_we, const double *from_xe, double *to_xe, double *to_we = nullptr, double *to_re = nullptr, const char *from_re_type = "313", const char *to_eu_type = "313") noexcept->double *;
		auto s_inv_xe2xe(const double *inv_relative_pm, const double *inv_relative_vs, const double *inv_relative_as,
			const double *from_re, const double *from_we, const double *from_xe, double *to_xe, double *to_we = nullptr, double *to_re = nullptr, const char *from_re_type = "313", const char *to_eu_type = "313") noexcept->double *;
		auto s_xq2xq(const double *relative_pm, const double *relative_vs, const double *relative_as, 
			const double *from_rq, const double *from_wq, const double *from_xq, double *to_xq, double *to_wq = nullptr, double *to_rq = nullptr) noexcept->double *;
		auto s_inv_xq2xq(const double *inv_relative_pm, const double *inv_relative_vs, const double *inv_relative_as, 
			const double *from_rq, const double *from_wq, const double *from_xq, double *to_xq, double *to_wq = nullptr, double *to_rq = nullptr) noexcept->double *;
		auto s_xm2xm(const double *relative_pm, const double *relative_vs, const double *relative_as, 
			const double *from_rm, const double *from_wm, const double *from_xm, double *to_xm, double *to_wm = nullptr, double *to_rm = nullptr) noexcept->double *;
		auto s_inv_xm2xm(const double *inv_relative_pm, const double *inv_relative_vs, const double *inv_relative_as, 
			const double *from_rm, const double *from_wm, const double *from_xm, double *to_xm, double *to_wm = nullptr, double *to_rm = nullptr) noexcept->double *;
		auto s_xa2xa(const double *relative_pm, const double *relative_vs, const double *relative_as,
			const double *from_wa, const double *from_xa, double *to_xa, double *to_wa = nullptr) noexcept->double *;
		auto s_inv_xa2xa(const double *inv_relative_pm, const double *inv_relative_vs, const double *inv_relative_as,
			const double *from_wa, const double *from_xa, double *to_xa, double *to_wa = nullptr) noexcept->double *;
		auto s_ae2ae(const double *relative_pm, const double *relative_vs, const double *relative_as,
			const double *from_pe, const double *from_ve, const double *from_ae, double *to_ae, double *to_ve = nullptr, double *to_pe = nullptr, const char *from_re_type = "313", const char *to_eu_type = "313") noexcept->double *;
		auto s_inv_ae2ae(const double *inv_relative_pm, const double *inv_relative_vs, const double *inv_relative_as,
			const double *from_pe, const double *from_ve, const double *from_ae, double *to_ae, double *to_ve = nullptr, double *to_pe = nullptr, const char *from_re_type = "313", const char *to_eu_type = "313") noexcept->double *;
		auto s_aq2aq(const double *relative_pm, const double *relative_vs, const double *relative_as, 
			const double *from_pq, const double *from_vq, const double *from_aq, double *to_aq, double *to_vq = nullptr, double *to_pq = nullptr) noexcept->double *;
		auto s_inv_aq2aq(const double *inv_relative_pm, const double *inv_relative_vs, const double *inv_relative_as, 
			const double *from_pq, const double *from_vq, const double *from_aq, double *to_aq, double *to_vq = nullptr, double *to_pq = nullptr) noexcept->double *;
		auto s_am2am(const double *relative_pm, const double *relative_vs, const double *relative_as, 
			const double *from_pm, const double *from_vm, const double *from_am, double *to_am, double *to_vm = nullptr, double *to_pm = nullptr) noexcept->double *;
		auto s_inv_am2am(const double *inv_relative_pm, const double *inv_relative_vs, const double *inv_relative_as, 
			const double *from_pm, const double *from_vm, const double *from_am, double *to_am, double *to_vm = nullptr, double *to_pm = nullptr) noexcept->double *;
		auto s_aa2aa(const double *relative_pm, const double *relative_vs, const double *relative_as,
			const double *from_pp, const double *from_va, const double *from_aa, double *to_aa, double *to_va = nullptr, double *to_pp = nullptr) noexcept->double *;
		auto s_inv_aa2aa(const double *inv_relative_pm, const double *inv_relative_vs, const double *inv_relative_as,
			const double *from_pp, const double *from_va, const double *from_aa, double *to_aa, double *to_va = nullptr, double *to_pp = nullptr) noexcept->double *;
		auto s_as2as(const double *relative_pm, const double *relative_vs, const double *relative_as,
			const double *from_vs, const double *from_as, double *to_as, double *to_vs = nullptr) noexcept->double *;
		auto s_inv_as2as(const double *inv_relative_pm, const double *inv_relative_vs, const double *inv_relative_as,
			const double *from_vs, const double *from_as, double *to_as, double *to_vs = nullptr) noexcept->double *;

		auto s_fs2fs(const double *relative_pm, const double *from_fs, double *to_fs) noexcept->double *;
		auto s_inv_fs2fs(const double *inv_relative_pm, const double *from_fs, double *to_fs) noexcept->double *;
		auto s_im2im(const double *relative_pm, const double *from_im, double *to_im) noexcept->double *;
		auto s_inv_im2im(const double *inv_relative_pm, const double *from_im, double *to_im) noexcept->double *;
		auto s_iv2iv(const double *relative_pm, const double *from_im, double *to_im) noexcept->double *;
		auto s_inv_iv2iv(const double *inv_relative_pm, const double *from_im, double *to_im) noexcept->double *;
		
		/// \brief 根据原点和两个坐标轴上的点来求位姿矩阵
		///
		/// 这里原点origin为位姿矩阵pm_out的点,first_pnt位于第一根坐标轴,second_pnt位于第一根坐标轴和第二根坐标轴所构成的平面内
		///
		///
		auto s_sov_pnts2pm(const double *origin, Size origin_ld, const double *first_pnt, Size first_ld, const double *second_pnt, Size second_ld, double *pm_out, const char *axis_order = "xy") noexcept->void;
		/// \brief 根据原点和两个坐标轴上的点来求位姿矩阵
		///
		/// 这里原点origin为位姿矩阵pm_out的点,firstAxisPnt位于第一根坐标轴,secondAxisPnt位于第一根坐标轴和第二根坐标轴所构成的平面内
		///
		///
		auto inline s_sov_pnts2pm(const double *origin, const double *first_pnt, const double *second_pnt, double *pm_out, const char *axis_order = "xy") noexcept->void { s_sov_pnts2pm(origin, 1, first_pnt, 1, second_pnt, 1, pm_out, axis_order); };
		/// \brief 根据原点和两个坐标轴上的点来求位姿矩阵
		///
		/// 这里原点origin为位姿矩阵pm_out的点,first_pnt位于第一根坐标轴,second_pnt位于第一根坐标轴和第二根坐标轴所构成的平面内
		///
		///
		auto s_sov_axes2pm(const double *origin, Size origin_ld, const double *first_axis, Size first_ld, const double *second_axis, Size second_ld, double *pm_out, const char *axis_order = "xy") noexcept->void;
		/// \brief 根据原点和两个坐标轴上的点来求位姿矩阵
		///
		/// 这里原点origin为位姿矩阵pm_out的点,firstAxisPnt位于第一根坐标轴,secondAxisPnt位于第一根坐标轴和第二根坐标轴所构成的平面内
		///
		///
		auto inline s_sov_axes2pm(const double *origin, const double *first_axis, const double *second_axis, double *pm_out, const char *axis_order = "xy") noexcept->void { s_sov_axes2pm(origin, 1, first_axis, 1, second_axis, 1, pm_out, axis_order); };
		/// \brief 求解形如 k1 * sin(theta) + k2 * cos(theta) = b 的方程,该方程有2个根
		///
		///
		auto s_sov_theta(double k1, double k2, double b, double *theta_out)noexcept->void;
		/// \brief 求解alpha 和 beta, 使得轴pp0转到pp的位置，alpha和beta的转轴由order定义，pp0为alpha和beta转轴的叉乘方向
		///
		///
		auto s_sov_ab(const double*pp, double *ab, const char*order = "321")noexcept->void;
		/// \brief 求解v_alpha 和 v_beta, 使得轴pp0转到pp的位置，alpha和beta的转轴由order定义，pp0为alpha和beta转轴的叉乘方向
		///
		///
		auto s_sov_vab(const double*pp, const double*vp, double *vab, double *ab, const char*order = "321")noexcept->void;
		/// \brief 求解a_alpha 和 a_beta, 使得轴pp0转到pp的位置，alpha和beta的转轴由order定义，pp0为alpha和beta转轴的叉乘方向
		///
		///
		auto s_sov_aab(const double*pp, const double*vp, const double*ap, double *aab, double *vab, double *ab, const char*order = "321")noexcept->void;
		/// \brief 求解某根轴下的相对位移，axis为0，1，2时对应x、y、z轴的位移，为4、5、6时对应延x、y、z轴的转角
		///
		///
		auto s_sov_axis_distance(const double*from_pm, const double*to_pm, Size axis)noexcept->double;






		auto s_dlt_col(const Size &dlt_col_num, const Size *col_index, const Size &m, const Size &n, double *A, const Size &ldA) noexcept->void;
	}
}




























#endif
