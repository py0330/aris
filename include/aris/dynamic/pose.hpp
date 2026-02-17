#ifndef ARIS_DYNAMIC_POSE_H_
#define ARIS_DYNAMIC_POSE_H_

#include <cmath>

#include "aris/dynamic/math_matrix.hpp"
#include "aris/dynamic/pose.hpp"
#include "aris/dynamic/kinematics.hpp"

namespace aris::dynamic{
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
	/// ps  :  6x1 位移螺旋，单位速度螺旋[vu;wu]转theta角，也就是ps =[v,w]=[vu;wu]*theta, theta = |w|
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
	///
	/// i3  :  3x3 惯量矩阵
	/// im  :  6x6 空间惯量矩阵
	/// iv  :  10x1 惯量矩阵向量[m, cx, cy, cz, Ixx, Iyy, Izz, Ixy, Ixz, Iyz]

	using double3x3 = double[3][3];

	//
	// pose type 可选：
	//
	//     pm : 4x4
	//     pq : 7x1
	//  pe123 : 6x1
	//  pe313 : 6x1
	//  pe321 : 6x1
	//    ... : 任意形式的pe
	//    ipm : 4x4 pm 的逆
	//    ipq : 7x1 pq 的逆
	// ipe123 : 6x1 pe 的逆
	// ipe313 : 6x1  
	// ipe321 : 6x1
	//    ... : 任意形式的ipe
	//
	// 其中 ipm 是指 pm 的逆
	//
	// 例1 : s_pose2pose(pm1, "pm", pm2, "ipm"); 可以求得 pm2^-1 = pm1    =>    pm2 = pm1^-1
	// 例2 : s_pose2pose(pe1, "pe321", pq, "pq"); 可以将321的欧拉角转成位置和四元数
	// 
	auto ARIS_API s_pose2pose(const double* pose1, const char *pose_type, double* pose2, const char *pose2_type)noexcept->double*;
	auto ARIS_API s_pose_dot_pose(const double* pose1, const char* pose1_type, const double* pose2, const char* pose2_type, double* pose3 = nullptr, const char* pose3_type = "pm")noexcept->double*;
	auto ARIS_API s_pose_dot_v3(const double* pose, const char* pose_type, const double* v3_in, double* v3_out = nullptr) noexcept->double*;

	template <typename ...Args>
	auto s_pose_dot_pose(const double* pose1, const char* pose1_type, const double* pose2, const char* pose2_type, Args ...args) noexcept->double* {
		double pm[16];
		s_pose_dot_pose(pose1, pose1_type, pose2, pose2_type, pm, "pm");
		return s_pose_dot_pose(pm, "pm", args...);
	}

	auto ARIS_API s_inv_rm(const double* rm, double* inv_rm = nullptr)noexcept->double*;
	auto ARIS_API s_rm_dot_rm(const double* rm1, const double* rm2, double* rm3 = nullptr)noexcept->double*;
	auto ARIS_API s_inv_rm_dot_rm(const double* rm1, const double* rm2, double* rm3 = nullptr)noexcept->double*;
	auto ARIS_API s_rm_dot_inv_rm(const double* rm1, const double* rm2, double* rm3 = nullptr)noexcept->double*;
	auto ARIS_API s_rm_dot_v3(const double* rm, const double* v1, double* v2 = nullptr)noexcept->double*;
	auto ARIS_API s_inv_rm_dot_v3(const double* rm, const double* v1, double* v2 = nullptr)noexcept->double*;

	auto ARIS_API s_inv_pm(const double *pm_in, double *pm_out = nullptr) noexcept->double*;
	auto ARIS_API s_pm_dot_pm(const double *pm1_in, const double *pm2_in, double *pm_out =nullptr) noexcept->double *;
	auto ARIS_API s_inv_pm_dot_pm(const double *inv_pm1_in, const double *pm2_in, double *pm_out = nullptr) noexcept->double *;
	auto ARIS_API s_pm_dot_inv_pm(const double *pm1_in, const double *inv_pm2_in, double *pm_out = nullptr) noexcept->double *;
	auto ARIS_API s_pm_dot_v3(const double *pm, const double *v3_in, double *v3_out = nullptr) noexcept->double *;
	auto ARIS_API s_inv_pm_dot_v3(const double* inv_pm, const double* v3, double* v3_out = nullptr) noexcept->double*;
	auto ARIS_API s_pm_dot_plane(const double* pm, const double* plane1, double* plane2 = nullptr)noexcept->double*;

	template <typename ...Args>
	auto s_pm_dot_pm(const double* pm1, const double* pm2, Args ...args) noexcept->double* {
		double pm[16];
		s_pm_dot_pm(pm1, pm2, pm);
		return s_pm_dot_pm(pm, args...);
	}
	template <typename V3Type1, typename V3Type2>
	auto s_pm_dot_v3(const double *pm, const double *v3_in, V3Type1 v31_t, double *v3_out, V3Type2 v32_t) noexcept->void{
		const Size a0{ 0 }, a1{ next_r(a0, v31_t) }, a2{ next_r(a1, v31_t) };
		const Size b0{ 0 }, b1{ next_r(b0, v32_t) }, b2{ next_r(b1, v32_t) };

		v3_out[b0] = pm[0] * v3_in[a0] + pm[1] * v3_in[a1] + pm[2] * v3_in[a2];
		v3_out[b1] = pm[4] * v3_in[a0] + pm[5] * v3_in[a1] + pm[6] * v3_in[a2];
		v3_out[b2] = pm[8] * v3_in[a0] + pm[9] * v3_in[a1] + pm[10] * v3_in[a2];
	}
	template <typename V3Type1, typename V3Type2>
	auto s_inv_pm_dot_v3(const double *inv_pm, const double *v3_in, V3Type1 v31_t, double *v3_out, V3Type2 v32_t) noexcept->void{
		const Size a0{ 0 }, a1{ next_r(a0, v31_t) }, a2{ next_r(a1, v31_t) };
		const Size b0{ 0 }, b1{ next_r(b0, v32_t) }, b2{ next_r(b1, v32_t) };

		v3_out[b0] = inv_pm[0] * v3_in[a0] + inv_pm[4] * v3_in[a1] + inv_pm[8] * v3_in[a2];
		v3_out[b1] = inv_pm[1] * v3_in[a0] + inv_pm[5] * v3_in[a1] + inv_pm[9] * v3_in[a2];
		v3_out[b2] = inv_pm[2] * v3_in[a0] + inv_pm[6] * v3_in[a1] + inv_pm[10] * v3_in[a2];
	}
	
	auto ARIS_API s_inv_rq(const double* q, double* inv_q)noexcept->double*;
	auto ARIS_API s_rq_dot_rq(const double* q1, const double* q2, double* q3)noexcept->double*;
	auto ARIS_API s_inv_rq_dot_rq(const double* q1, const double* q2, double* q3)noexcept->double*;
	auto ARIS_API s_rq_dot_inv_rq(const double* q1, const double* q2, double* q3)noexcept->double*;
	auto ARIS_API s_rq_dot_v3(const double* rq, const double* v3_in, double* v3_out) noexcept->double*;
	auto ARIS_API s_inv_rq_dot_v3(const double* rq, const double* v1, double* v2)noexcept->double*;

	auto ARIS_API s_inv_pq(const double* pq_in, double* pq_out) noexcept->double*;
	auto ARIS_API s_pq_dot_pq(const double* pq1_in, const double* pq2_in, double* pq_out) noexcept->double*;
	auto ARIS_API s_inv_pq_dot_pq(const double* inv_pq1_in, const double* pq2_in, double* pq_out) noexcept->double*;
	auto ARIS_API s_pq_dot_inv_pq(const double* pq1_in, const double* inv_pq2_in, double* pq_out) noexcept->double*;
	auto ARIS_API s_pq_dot_v3(const double* pq, const double* v3_in, double* v3_out) noexcept->double*;
	auto ARIS_API s_inv_pq_dot_v3(const double* pq, const double* v1, double* v2)noexcept->double*;

	template <typename ...Args>
	auto s_pq_dot_pq(const double* pq1, const double* pq2, Args ...args) noexcept->double* {
		double pq[7];
		s_pq_dot_pq(pq1, pq2, pq);
		return s_pq_dot_pq(pq, args...);
	}
	//template <typename V3Type1, typename V3Type2>
	//auto s_pq_dot_v3(const double* pm, const double* v3_in, V3Type1 v31_t, double* v3_out, V3Type2 v32_t) noexcept->void {
	//	const Size a0{ 0 }, a1{ next_r(a0, v31_t) }, a2{ next_r(a1, v31_t) };
	//	const Size b0{ 0 }, b1{ next_r(b0, v32_t) }, b2{ next_r(b1, v32_t) };

	//	v3_out[b0] = pm[0] * v3_in[a0] + pm[1] * v3_in[a1] + pm[2] * v3_in[a2];
	//	v3_out[b1] = pm[4] * v3_in[a0] + pm[5] * v3_in[a1] + pm[6] * v3_in[a2];
	//	v3_out[b2] = pm[8] * v3_in[a0] + pm[9] * v3_in[a1] + pm[10] * v3_in[a2];
	//}
	//template <typename V3Type1, typename V3Type2>
	//auto s_inv_pq_dot_v3(const double* inv_pm, const double* v3_in, V3Type1 v31_t, double* v3_out, V3Type2 v32_t) noexcept->void {
	//	const Size a0{ 0 }, a1{ next_r(a0, v31_t) }, a2{ next_r(a1, v31_t) };
	//	const Size b0{ 0 }, b1{ next_r(b0, v32_t) }, b2{ next_r(b1, v32_t) };

	//	v3_out[b0] = inv_pm[0] * v3_in[a0] + inv_pm[4] * v3_in[a1] + inv_pm[8] * v3_in[a2];
	//	v3_out[b1] = inv_pm[1] * v3_in[a0] + inv_pm[5] * v3_in[a1] + inv_pm[9] * v3_in[a2];
	//	v3_out[b2] = inv_pm[2] * v3_in[a0] + inv_pm[6] * v3_in[a1] + inv_pm[10] * v3_in[a2];
	//}

	//auto inline s_theta_v_to

	auto ARIS_API s_im_dot_as(const double *im, const double *as, double * fs = nullptr) noexcept->double *;
	auto ARIS_API s_iv_dot_as(const double *iv, const double *as, double * fs = nullptr) noexcept->double *;

	// 以下函数为物理量之间的转换函数 //
	auto ARIS_API s_ra2rm(const double* ra_in, double* rm_out = nullptr, Size rm_ld = 3) noexcept->double*;
	auto ARIS_API s_rm2ra(const double* rm_in, double* ra_out = nullptr, Size rm_ld = 3) noexcept->double*;
	auto ARIS_API s_re2rm(const double* re_in, double* rm_out = nullptr, const char* eu_type_in = "313", Size rm_ld = 3) noexcept->double*;
	auto ARIS_API s_rm2re(const double* rm_in, double* re_out = nullptr, const char* eu_type_in = "313", Size rm_ld = 3) noexcept->double*;
	auto ARIS_API s_rq2rm(const double* rq_in, double* rm_out = nullptr, Size rm_ld = 3) noexcept->double*;
	auto ARIS_API s_rm2rq(const double* rm_in, double* rq_out = nullptr, Size rm_ld = 3) noexcept->double*;
	auto ARIS_API s_pp2pm(const double* pp_in, double* pm_out = nullptr) noexcept->double*;
	auto ARIS_API s_pm2pp(const double* pm_in, double* pp_out = nullptr) noexcept->double*;
	auto ARIS_API s_ra2pm(const double* ra_in, double* rm_out = nullptr) noexcept->double*;
	auto ARIS_API s_pm2ra(const double* rm_in, double* ra_out = nullptr) noexcept->double*;
	auto ARIS_API s_re2pm(const double* re_in, double* pm_out = nullptr, const char* eu_type_in = "313") noexcept->double*;
	auto ARIS_API s_pm2re(const double* pm_in, double* re_out = nullptr, const char* eu_type_in = "313") noexcept->double*;
	auto ARIS_API s_pa2pm(const double* pa_in, double* pm_out = nullptr) noexcept->double*;
	auto ARIS_API s_pm2pa(const double* pm_in, double* pa_out = nullptr) noexcept->double*;
	auto ARIS_API s_rq2pm(const double* rq_in, double* pm_out = nullptr) noexcept->double*;
	auto ARIS_API s_pm2rq(const double* pm_in, double* rq_out = nullptr) noexcept->double*;
	auto ARIS_API s_rm2pm(const double* rm_in, double* pm_out = nullptr, Size rm_ld = 3) noexcept->double*;
	auto ARIS_API s_pm2rm(const double* pm_in, double* rm_out = nullptr, Size rm_ld = 3) noexcept->double*;
	auto ARIS_API s_pe2pm(const double* pe_in, double* pm_out = nullptr, const char* eu_type_in = "313") noexcept->double*;
	auto ARIS_API s_pm2pe(const double* pm_in, double* pe_out = nullptr, const char* eu_type_in = "313") noexcept->double*;
	auto ARIS_API s_pq2pm(const double* pq_in, double* pm_out = nullptr) noexcept->double*;
	auto ARIS_API s_pm2pq(const double* pm_in, double* pq_out = nullptr) noexcept->double*;
	auto ARIS_API s_ps2pm(const double* ps_in, double* pm_out = nullptr) noexcept->double*;
	auto ARIS_API s_pm2ps(const double* pm_in, double* ps_out = nullptr) noexcept->double*;

	auto ARIS_API s_we2wa(const double* re_in, const double* we_in, double* wa_out = nullptr, const char* eu_type_in = "313") noexcept->double*;
	auto ARIS_API s_wa2we(const double* wa_in, const double* re_in, double* we_out = nullptr, const char* eu_type_in = "313") noexcept->double*;
	auto ARIS_API s_wq2wa(const double* rq_in, const double* wq_in, double* wa_out = nullptr) noexcept->double*;
	auto ARIS_API s_wa2wq(const double* wa_in, const double* rq_in, double* wq_out = nullptr) noexcept->double*;
	auto ARIS_API s_wm2wa(const double* rm_in, const double* wm_in, double* wa_out = nullptr, Size rm_ld = 3, Size wm_ld = 3) noexcept->double*;
	auto ARIS_API s_wa2wm(const double* wa_in, const double* rm_in, double* wm_out = nullptr, Size rm_ld = 3, Size wm_ld = 3) noexcept->double*;
	auto ARIS_API s_vp2vs(const double* pp_in, const double* vp_in, double* vs_out = nullptr) noexcept->double*;
	auto ARIS_API s_vs2vp(const double* vs_in, const double* pp_in, double* vp_out = nullptr) noexcept->double*;
	auto ARIS_API s_we2vs(const double* re_in, const double* we_in, double* vs_out = nullptr, const char* eu_type_in = "313") noexcept->double*;
	auto ARIS_API s_vs2we(const double* vs_in, const double* re_in, double* we_out = nullptr, const char* eu_type_in = "313") noexcept->double*;
	auto ARIS_API s_wq2vs(const double* rq_in, const double* wq_in, double* vs_out = nullptr) noexcept->double*;
	auto ARIS_API s_vs2wq(const double* vs_in, const double* rq_in, double* wq_out = nullptr) noexcept->double*;
	auto ARIS_API s_wm2vs(const double* rm_in, const double* wm_in, double* vs_out = nullptr, Size rm_ld = 3, Size wm_ld = 3) noexcept->double*;
	auto ARIS_API s_vs2wm(const double* vs_in, const double* rm_in, double* wm_out = nullptr, Size rm_ld = 3, Size wm_ld = 3) noexcept->double*;
	auto ARIS_API s_wa2vs(const double* wa_in, double* vs_out = nullptr) noexcept->double*;
	auto ARIS_API s_vs2wa(const double* vs_in, double* wa_out = nullptr) noexcept->double*;
	auto ARIS_API s_ve2vs(const double* pe_in, const double* ve_in, double* vs_out = nullptr, const char* eu_type_in = "313") noexcept->double*;
	auto ARIS_API s_vs2ve(const double* vs_in, const double* pe_in, double* ve_out = nullptr, const char* eu_type_in = "313") noexcept->double*;
	auto ARIS_API s_vq2vs(const double* pq_in, const double* vq_in, double* vs_out = nullptr) noexcept->double*;
	auto ARIS_API s_vs2vq(const double* vs_in, const double* pq_in, double* vq_out = nullptr) noexcept->double*;
	auto ARIS_API s_vm2vs(const double* pm_in, const double* vm_in, double* vs_out = nullptr) noexcept->double*;
	auto ARIS_API s_vs2vm(const double* vs_in, const double* pm_in, double* vm_out = nullptr) noexcept->double*;
	auto ARIS_API s_va2vs(const double* pp_in, const double* va_in, double* vs_out = nullptr) noexcept->double*;
	auto ARIS_API s_vs2va(const double* vs_in, const double* pp_in, double* va_out = nullptr) noexcept->double*;

	auto ARIS_API s_xe2xa(const double* re_in, const double* we_in, const double* xe_in, double* xa_out = nullptr, double* wa_out = nullptr, const char* eu_type_in = "313") noexcept->double*;
	auto ARIS_API s_xa2xe(const double* wa_in, const double* xa_in, const double* re_in, double* xe_out = nullptr, double* we_out = nullptr, const char* eu_type_in = "313") noexcept->double*;
	auto ARIS_API s_xq2xa(const double* rq_in, const double* wq_in, const double* xq_in, double* xa_out = nullptr, double* wa_out = nullptr) noexcept->double*;
	auto ARIS_API s_xa2xq(const double* wa_in, const double* xa_in, const double* rq_in, double* xq_out = nullptr, double* wq_out = nullptr) noexcept->double*;
	auto ARIS_API s_xm2xa(const double* rm_in, const double* wm_in, const double* xm_in, double* xa_out = nullptr, double* wa_out = nullptr, Size rm_ld = 3, Size wm_ld = 3, Size xm_ld = 3) noexcept->double*;
	auto ARIS_API s_xa2xm(const double* wa_in, const double* xa_in, const double* rm_in, double* xm_out = nullptr, double* wm_out = nullptr, Size rm_ld = 3, Size wm_ld = 3, Size xm_ld = 3) noexcept->double*;
	auto ARIS_API s_ap2as(const double* pp_in, const double* vp_in, const double* ap_in, double* as_out = nullptr, double* vs_out = nullptr) noexcept->double*;
	auto ARIS_API s_as2ap(const double* vs_in, const double* as_in, const double* pp_in, double* ap_out = nullptr, double* vp_out = nullptr) noexcept->double*;
	auto ARIS_API s_xe2as(const double* re_in, const double* we_in, const double* xe_in, double* as_out = nullptr, double* vs_out = nullptr, const char* eu_type_in = "313") noexcept->double*;
	auto ARIS_API s_as2xe(const double* vs_in, const double* as_in, const double* re_in, double* xe_out = nullptr, double* we_out = nullptr, const char* eu_type_in = "313") noexcept->double*;
	auto ARIS_API s_xq2as(const double* rq_in, const double* wq_in, const double* xq_in, double* as_out = nullptr, double* vs_out = nullptr) noexcept->double*;
	auto ARIS_API s_as2xq(const double* vs_in, const double* as_in, const double* rq_in, double* xq_out = nullptr, double* wq_out = nullptr) noexcept->double*;
	auto ARIS_API s_xm2as(const double* rm_in, const double* wm_in, const double* xm_in, double* as_out = nullptr, double* vs_out = nullptr, Size rm_ld = 3, Size wm_ld = 3, Size xm_ld = 3) noexcept->double*;
	auto ARIS_API s_as2xm(const double* vs_in, const double* as_in, const double* rm_in, double* xm_out = nullptr, double* wm_out = nullptr, Size rm_ld = 3, Size wm_ld = 3, Size xm_ld = 3) noexcept->double*;
	auto ARIS_API s_xa2as(const double* xa_in, double* as_out = nullptr) noexcept->double*;
	auto ARIS_API s_as2xa(const double* as_in, double* xa_out = nullptr) noexcept->double*;
	auto ARIS_API s_ae2as(const double* pe_in, const double* ve_in, const double* ae_in, double* as_out = nullptr, double* vs_out = nullptr, const char* eu_type_in = "313") noexcept->double*;
	auto ARIS_API s_as2ae(const double* vs_in, const double* as_in, const double* pe_in, double* ae_out = nullptr, double* ve_out = nullptr, const char* eu_type_in = "313") noexcept->double*;
	auto ARIS_API s_aq2as(const double* pq_in, const double* vq_in, const double* aq_in, double* as_out = nullptr, double* vs_out = nullptr) noexcept->double*;
	auto ARIS_API s_as2aq(const double* vs_in, const double* as_in, const double* pq_in, double* aq_out = nullptr, double* vq_out = nullptr) noexcept->double*;
	auto ARIS_API s_am2as(const double* pm_in, const double* vm_in, const double* am_in, double* as_out = nullptr, double* vs_out = nullptr) noexcept->double*;
	auto ARIS_API s_as2am(const double* vs_in, const double* as_in, const double* pm_in, double* am_out = nullptr, double* vm_out = nullptr) noexcept->double*;
	auto ARIS_API s_aa2as(const double* pp_in, const double* va_in, const double* aa_in, double* as_out = nullptr, double* vs_out = nullptr) noexcept->double*;
	auto ARIS_API s_as2aa(const double* vs_in, const double* as_in, const double* pp_in, double* aa_out = nullptr, double* va_out = nullptr) noexcept->double*;

	auto ARIS_API s_re2rq(const double* re_in, double* rq_out = nullptr, const char* eu_type_in = "313") noexcept->double*;
	auto ARIS_API s_rq2re(const double* rq_in, double* re_out = nullptr, const char* eu_type_in = "313") noexcept->double*;
	auto ARIS_API s_pq2pe(const double* pq_in, double* pe_out = nullptr, const char* eu_type_in = "313") noexcept->double*;
	auto ARIS_API s_pe2pq(const double* pe_in, double* pq_out = nullptr, const char* eu_type_in = "313") noexcept->double*;
	auto ARIS_API s_iv2im(const double* iv_in, double* im_out = nullptr) noexcept->double*;
	auto ARIS_API s_im2iv(const double* im_in, double* iv_out = nullptr) noexcept->double*;
	auto ARIS_API s_i32im(const double mass_in, const double * in_in, const double *pm_in, double *im_out = nullptr) noexcept->double *;

	// 以下函数为同一物理量在不同坐标系之间的转换函数 //
	auto ARIS_API s_pp2pp(const double *relative_pm, const double *from_pp, double *to_pp) noexcept->double *;
	auto ARIS_API s_inv_pp2pp(const double *inv_relative_pm, const double *from_pp, double *to_pp) noexcept->double *;
	auto ARIS_API s_re2re(const double *relative_pm, const double *from_re, double *to_re, const char *from_eu_type = "313", const char *to_eu_type = "313") noexcept->double *;
	auto ARIS_API s_inv_re2re(const double *inv_relative_pm, const double *from_re, double *to_re, const char *from_eu_type = "313", const char *to_eu_type = "313") noexcept->double *;
	auto ARIS_API s_rq2rq(const double *relative_pm, const double *from_rq, double *to_rq) noexcept->double *;
	auto ARIS_API s_inv_rq2rq(const double *inv_relative_pm, const double *from_rq, double *to_rq) noexcept->double *;
	auto ARIS_API s_rm2rm(const double *relative_pm, const double *from_rm, double *to_rm, Size from_rm_ld = 3, Size to_rm_ld = 3) noexcept->double *;
	auto ARIS_API s_inv_rm2rm(const double *inv_relative_pm, const double *from_rm, double *to_rm, Size from_rm_ld = 3, Size to_rm_ld = 3) noexcept->double *;
	auto ARIS_API s_pe2pe(const double *relative_pm, const double *from_pe, double *to_pe, const char *from_pe_type = "313", const char *to_pe_type = "313") noexcept->double *;
	auto ARIS_API s_inv_pe2pe(const double *inv_relative_pm, const double *from_pe, double *to_pe, const char *from_pe_type = "313", const char *to_pe_type = "313") noexcept->double *;
	auto ARIS_API s_pq2pq(const double *relative_pm, const double *from_pq, double *to_pq) noexcept->double *;
	auto ARIS_API s_inv_pq2pq(const double *inv_relative_pm, const double *from_pq, double *to_pq) noexcept->double *;
	auto ARIS_API s_pm2pm(const double *relative_pm, const double *from_pm, double *to_pm) noexcept->double *;
	auto ARIS_API s_inv_pm2pm(const double *inv_relative_pm, const double *from_pm, double *to_pm) noexcept->double *;

	auto ARIS_API s_vp2vp(const double *relative_pm, const double *relative_vs, const double *from_pp, const double *from_vp, double *to_vp, double *to_pp = nullptr) noexcept->double *;
	auto ARIS_API s_inv_vp2vp(const double *inv_relative_pm, const double *inv_relative_vs, const double *from_pp, const double *from_vp, double *to_vp, double *to_pp = nullptr) noexcept->double *;
	auto ARIS_API s_we2we(const double *relative_pm, const double *relative_vs, const double *from_re, const double *from_we, double *to_we, double *to_re = nullptr, const char *from_eu_type = "313", const char *to_eu_type = "313") noexcept->double *;
	auto ARIS_API s_inv_we2we(const double *inv_relative_pm, const double *inv_relative_vs, const double *from_re, const double *from_we, double *to_we, double *to_re = nullptr, const char *from_eu_type = "313", const char *to_eu_type = "313") noexcept->double *;
	auto ARIS_API s_wq2wq(const double *relative_pm, const double *relative_vs, const double *from_rq, const double *from_wq, double *to_wq, double *to_rq = nullptr) noexcept->double *;
	auto ARIS_API s_inv_wq2wq(const double *inv_relative_pm, const double *inv_relative_vs, const double *from_rq, const double *from_wq, double *to_wq, double *to_rq = nullptr) noexcept->double *;
	auto ARIS_API s_wm2wm(const double *relative_pm, const double *relative_vs, const double *from_rm, const double *from_wm, double *to_wm, double *to_rm = nullptr) noexcept->double *;
	auto ARIS_API s_inv_wm2wm(const double *inv_relative_pm, const double *inv_relative_vs, const double *from_rm, const double *from_wm, double *to_wm, double *to_rm = nullptr) noexcept->double *;
	auto ARIS_API s_wa2wa(const double *relative_pm, const double *relative_vs, const double *from_wa, double *to_wa) noexcept->double *;
	auto ARIS_API s_inv_wa2wa(const double *inv_relative_pm, const double *inv_relative_vs, const double *from_wa, double *to_wa) noexcept->double *;
	auto ARIS_API s_ve2ve(const double *relative_pm, const double *relative_vs, const double *from_pe, const double *from_ve, double *to_ve, double *to_pe = nullptr, const char *from_eu_type = "313", const char *to_eu_type = "313") noexcept->double *;
	auto ARIS_API s_inv_ve2ve(const double *inv_relative_pm, const double *inv_relative_vs, const double *from_pe, const double *from_ve, double *to_ve, double *to_pe = nullptr, const char *from_eu_type = "313", const char *to_eu_type = "313") noexcept->double *;
	auto ARIS_API s_vq2vq(const double *relative_pm, const double *relative_vs, const double *from_pq, const double *from_vq, double *to_vq, double *to_pq = nullptr) noexcept->double *;
	auto ARIS_API s_inv_vq2vq(const double *inv_relative_pm, const double *inv_relative_vs, const double *from_pq, const double *from_vq, double *to_vq, double *to_pq = nullptr) noexcept->double *;
	auto ARIS_API s_vm2vm(const double *relative_pm, const double *relative_vs, const double *from_pm, const double *from_vm, double *to_vm, double *to_pm = nullptr) noexcept->double *;
	auto ARIS_API s_inv_vm2vm(const double *inv_relative_pm, const double *inv_relative_vs, const double *from_pm, const double *from_vm, double *to_vm, double *to_pm = nullptr) noexcept->double *;
	auto ARIS_API s_va2va(const double *relative_pm, const double *relative_vs, const double *from_pp, const double *from_va, double *to_va, double *to_pp = nullptr) noexcept->double *;
	auto ARIS_API s_inv_va2va(const double *inv_relative_pm, const double *inv_relative_vs, const double *from_pp, const double *from_va, double *to_va, double *to_pp = nullptr) noexcept->double *;
	auto ARIS_API s_vs2vs(const double *relative_pm, const double *relative_vs, const double *from_vs, double *to_vs) noexcept->double *;
	auto ARIS_API s_inv_vs2vs(const double *inv_relative_pm, const double *inv_relative_vs, const double *from_vs, double *to_vs) noexcept->double *;

	auto ARIS_API s_ap2ap(const double *relative_pm, const double *relative_vs, const double *relative_as,
		const double *from_pp, const double *from_vp, const double *from_ap, double *to_ap, double *to_vp = nullptr, double *to_pp = nullptr) noexcept->double *;
	auto ARIS_API s_inv_ap2ap(const double *inv_relative_pm, const double *inv_relative_vs, const double *inv_relative_as,
		const double *from_pp, const double *from_vp, const double *from_ap, double *to_ap, double *to_vp = nullptr, double *to_pp = nullptr) noexcept->double *;
	auto ARIS_API s_xe2xe(const double *relative_pm, const double *relative_vs, const double *relative_as,
		const double *from_re, const double *from_we, const double *from_xe, double *to_xe, double *to_we = nullptr, double *to_re = nullptr, const char *from_re_type = "313", const char *to_eu_type = "313") noexcept->double *;
	auto ARIS_API s_inv_xe2xe(const double *inv_relative_pm, const double *inv_relative_vs, const double *inv_relative_as,
		const double *from_re, const double *from_we, const double *from_xe, double *to_xe, double *to_we = nullptr, double *to_re = nullptr, const char *from_re_type = "313", const char *to_eu_type = "313") noexcept->double *;
	auto ARIS_API s_xq2xq(const double *relative_pm, const double *relative_vs, const double *relative_as,
		const double *from_rq, const double *from_wq, const double *from_xq, double *to_xq, double *to_wq = nullptr, double *to_rq = nullptr) noexcept->double *;
	auto ARIS_API s_inv_xq2xq(const double *inv_relative_pm, const double *inv_relative_vs, const double *inv_relative_as,
		const double *from_rq, const double *from_wq, const double *from_xq, double *to_xq, double *to_wq = nullptr, double *to_rq = nullptr) noexcept->double *;
	auto ARIS_API s_xm2xm(const double *relative_pm, const double *relative_vs, const double *relative_as,
		const double *from_rm, const double *from_wm, const double *from_xm, double *to_xm, double *to_wm = nullptr, double *to_rm = nullptr) noexcept->double *;
	auto ARIS_API s_inv_xm2xm(const double *inv_relative_pm, const double *inv_relative_vs, const double *inv_relative_as,
		const double *from_rm, const double *from_wm, const double *from_xm, double *to_xm, double *to_wm = nullptr, double *to_rm = nullptr) noexcept->double *;
	auto ARIS_API s_xa2xa(const double *relative_pm, const double *relative_vs, const double *relative_as,
		const double *from_wa, const double *from_xa, double *to_xa, double *to_wa = nullptr) noexcept->double *;
	auto ARIS_API s_inv_xa2xa(const double *inv_relative_pm, const double *inv_relative_vs, const double *inv_relative_as,
		const double *from_wa, const double *from_xa, double *to_xa, double *to_wa = nullptr) noexcept->double *;
	auto ARIS_API s_ae2ae(const double *relative_pm, const double *relative_vs, const double *relative_as,
		const double *from_pe, const double *from_ve, const double *from_ae, double *to_ae, double *to_ve = nullptr, double *to_pe = nullptr, const char *from_re_type = "313", const char *to_eu_type = "313") noexcept->double *;
	auto ARIS_API s_inv_ae2ae(const double *inv_relative_pm, const double *inv_relative_vs, const double *inv_relative_as,
		const double *from_pe, const double *from_ve, const double *from_ae, double *to_ae, double *to_ve = nullptr, double *to_pe = nullptr, const char *from_re_type = "313", const char *to_eu_type = "313") noexcept->double *;
	auto ARIS_API s_aq2aq(const double *relative_pm, const double *relative_vs, const double *relative_as,
		const double *from_pq, const double *from_vq, const double *from_aq, double *to_aq, double *to_vq = nullptr, double *to_pq = nullptr) noexcept->double *;
	auto ARIS_API s_inv_aq2aq(const double *inv_relative_pm, const double *inv_relative_vs, const double *inv_relative_as,
		const double *from_pq, const double *from_vq, const double *from_aq, double *to_aq, double *to_vq = nullptr, double *to_pq = nullptr) noexcept->double *;
	auto ARIS_API s_am2am(const double *relative_pm, const double *relative_vs, const double *relative_as,
		const double *from_pm, const double *from_vm, const double *from_am, double *to_am, double *to_vm = nullptr, double *to_pm = nullptr) noexcept->double *;
	auto ARIS_API s_inv_am2am(const double *inv_relative_pm, const double *inv_relative_vs, const double *inv_relative_as,
		const double *from_pm, const double *from_vm, const double *from_am, double *to_am, double *to_vm = nullptr, double *to_pm = nullptr) noexcept->double *;
	auto ARIS_API s_aa2aa(const double *relative_pm, const double *relative_vs, const double *relative_as,
		const double *from_pp, const double *from_va, const double *from_aa, double *to_aa, double *to_va = nullptr, double *to_pp = nullptr) noexcept->double *;
	auto ARIS_API s_inv_aa2aa(const double *inv_relative_pm, const double *inv_relative_vs, const double *inv_relative_as,
		const double *from_pp, const double *from_va, const double *from_aa, double *to_aa, double *to_va = nullptr, double *to_pp = nullptr) noexcept->double *;
	auto ARIS_API s_as2as(const double *relative_pm, const double *relative_vs, const double *relative_as,
		const double *from_vs, const double *from_as, double *to_as, double *to_vs = nullptr) noexcept->double *;
	auto ARIS_API s_inv_as2as(const double *inv_relative_pm, const double *inv_relative_vs, const double *inv_relative_as,
		const double *from_vs, const double *from_as, double *to_as, double *to_vs = nullptr) noexcept->double *;

	auto ARIS_API s_fs2fs(const double *relative_pm, const double *from_fs, double *to_fs) noexcept->double *;
	auto ARIS_API s_inv_fs2fs(const double *inv_relative_pm, const double *from_fs, double *to_fs) noexcept->double *;
	auto ARIS_API s_im2im(const double *relative_pm, const double *from_im, double *to_im) noexcept->double *;
	auto ARIS_API s_inv_im2im(const double *inv_relative_pm, const double *from_im, double *to_im) noexcept->double *;
	auto ARIS_API s_iv2iv(const double *relative_pm, const double *from_im, double *to_im) noexcept->double *;
	auto ARIS_API s_inv_iv2iv(const double *inv_relative_pm, const double *from_im, double *to_im) noexcept->double *;

	auto inline default_pp()noexcept->const double* { static const double value[3]{ 0,0,0 }; return value; }
	auto inline default_ra()noexcept->const double* { static const double value[3]{ 0,0,0 }; return value; }
	auto inline default_re()noexcept->const double* { static const double value[3]{ 0,0,0 }; return value; }
	auto inline default_rq()noexcept->const double* { static const double value[4]{ 0,0,0,1 }; return value; }
	auto inline default_rm()noexcept->const double* { static const double value[9]{ 1,0,0,0,1,0,0,0,1 }; return value; }
	auto inline default_pe()noexcept->const double* { static const double value[6]{ 0,0,0,0,0,0 }; return value; }
	auto inline default_pq()noexcept->const double* { static const double value[7]{ 0,0,0,0,0,0,1 }; return value; }
	auto inline default_pa()noexcept->const double* { static const double value[6]{ 0,0,0,0,0,0 }; return value; }
	auto inline default_ps()noexcept->const double* { static const double value[6]{ 0,0,0,0,0,0 }; return value; }
	auto inline default_pm()noexcept->const double* { static const double value[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 }; return value; }

	auto inline default_vp()noexcept->const double* { static const double value[3]{ 0,0,0 }; return value; }
	auto inline default_we()noexcept->const double* { static const double value[3]{ 0,0,0 }; return value; }
	auto inline default_wq()noexcept->const double* { static const double value[4]{ 0,0,0,0 }; return value; }
	auto inline default_wm()noexcept->const double* { static const double value[9]{ 0,0,0,0,0,0,0,0,0 }; return value; }
	auto inline default_ve()noexcept->const double* { static const double value[6]{ 0,0,0,0,0,0 }; return value; }
	auto inline default_vq()noexcept->const double* { static const double value[7]{ 0,0,0,0,0,0,0 }; return value; }
	auto inline default_vm()noexcept->const double* { static const double value[16]{ 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 }; return value; }
	auto inline default_wa()noexcept->const double* { static const double value[3]{ 0,0,0 }; return value; }
	auto inline default_va()noexcept->const double* { static const double value[6]{ 0,0,0,0,0,0 }; return value; }
	auto inline default_vs()noexcept->const double* { static const double value[6]{ 0,0,0,0,0,0 }; return value; }

	auto inline default_ap()noexcept->const double* { static const double value[3]{ 0,0,0 }; return value; }
	auto inline default_xe()noexcept->const double* { static const double value[3]{ 0,0,0 }; return value; }
	auto inline default_xq()noexcept->const double* { static const double value[4]{ 0,0,0,0 }; return value; }
	auto inline default_xm()noexcept->const double* { static const double value[9]{ 0,0,0,0,0,0,0,0,0 }; return value; }
	auto inline default_ae()noexcept->const double* { static const double value[6]{ 0,0,0,0,0,0 }; return value; }
	auto inline default_aq()noexcept->const double* { static const double value[7]{ 0,0,0,0,0,0,0 }; return value; }
	auto inline default_am()noexcept->const double* { static const double value[16]{ 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 }; return value; }
	auto inline default_xa()noexcept->const double* { static const double value[3]{ 0,0,0 }; return value; }
	auto inline default_aa()noexcept->const double* { static const double value[6]{ 0,0,0,0,0,0 }; return value; }
	auto inline default_as()noexcept->const double* { static const double value[6]{ 0,0,0,0,0,0 }; return value; }

	auto inline default_fs()noexcept->const double* { static const double value[6]{ 0,0,0,0,0,0 }; return value; }
	auto inline default_im()noexcept->const double* { static const double value[36]{ 0 }; return value; }
	auto inline default_iv()noexcept->const double* { static const double value[10]{ 1,0,0,0,1,1,1,0,0,0 }; return value; }
	auto inline default_i3()noexcept->const double* { static const double value[3]{ 0,0,0 }; return value; }

	auto inline P()noexcept->const double3x3& { static const double p[3][3]{ { 0, -1, 1 },{ 1, 0, -1 },{ -1, 1, 0 } }; return p; }
	auto inline Q()noexcept->const double3x3& { static const double q[3][3]{ { 1, 0, 0 },{ 0, 1, 0 },{ 0, 0, 1 } };	return q; }

}

#endif