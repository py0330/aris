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


namespace aris
{
	///
	/// to do lists:
	/// 下一版本的符号定义: \n
	/// pp  :  3x1 点位置(position of point)  \n
	/// re  :  3x1 欧拉角(eula angle)         \n
	/// rq  :  4x1 四元数(quaternions)        \n
	/// rm  :  3x3 旋转矩阵(rotation matrix)  \n
	/// pe  :  6x1 点位置与欧拉角(position and eula angle)\n
	/// pq  :  7x1 点位置与四元数(position and quaternions)\n
	/// pm  :  4x4 位姿矩阵(pose matrix)\n
	///
	/// vp  :  3x1 线速度(velocity of point)\n
	/// we  :  3x1 角速度(omega)\n
	/// wq  :  4x1 四元数导数(omega in term of quternions)\n
	/// wm  :  3x3 旋转矩阵导数(omega in term of rotation matrix)\n
	/// vw  :  6x1 线速度与角速度(velocity and omega)\n
	/// vq  :  7x1 线速度与四元数导数(velocity and omega in term of quternions)\n
	/// vm  :  4x4 位姿矩阵导数(velocity in term of pose matrix)\n
	/// vs  :  6x1 螺旋速度(velocity of screw)\n
	///
	/// ap  :  3x1 线加速度(acceleration of point)\n
	/// xe  :  3x1 角加速度(alpha, acceleration of angle)\n
	/// xq  :  4x1 四元数导导数(alpha in term of quternions)\n
	/// xm  :  3x3 旋转矩阵导数(alpha in term of rotation matrix)\n
	/// ax  :  6x1 线加速度与角加速度(acceleration and alpha)\n
	/// aq  :  7x1 线加速度与四元数导导数(acceleration and alpha in term of quternions)\n
	/// am  :  4x4 位姿矩阵导导数(acceleration in term of pose matrix)\n
	/// as  :  6x1 螺旋加速度(acceleration of screw)\n


	/// \brief 动力学命名空间
	/// \ingroup aris
	/// 
	///
	///
	namespace dynamic
	{
		// 以下函数为物理量之间的转换函数 //
		auto s_re2rm(const double *re_in, double *rm_out, const char *eu_type_in = "313", std::size_t rm_ld = 3) noexcept->void;
		auto s_rm2re(const double *rm_in, double *re_out, const char *eu_type_in = "313", std::size_t rm_ld = 3) noexcept->void;
		auto s_rq2rm(const double *rq_in, double *rm_out, std::size_t rm_ld = 3) noexcept->void;
		auto s_rm2rq(const double *rm_in, double *rq_out, std::size_t rm_ld = 3) noexcept->void;
		auto s_pp2pm(const double *pp_in, double *pm_out) noexcept->void;
		auto s_pm2pp(const double *pm_in, double *pp_out) noexcept->void;
		auto s_re2pm(const double *re_in, double *pm_out, const char *eu_type_in = "313") noexcept->void;
		auto s_pm2re(const double *pm_in, double *re_out, const char *eu_type_in = "313") noexcept->void;
		auto s_rq2pm(const double *rq_in, double *pm_out) noexcept->void;
		auto s_pm2rq(const double *pm_in, double *rq_out) noexcept->void; 
		auto s_rm2pm(const double *rm_in, double *pm_out) noexcept->void;
		auto s_pm2rm(const double *pm_in, double *rm_out) noexcept->void;
		auto s_pe2pm(const double *pe_in, double *pm_out, const char *eu_type_in = "313") noexcept->void;
		auto s_pm2pe(const double *pm_in, double *pe_out, const char *eu_type_in = "313") noexcept->void;
		auto s_pq2pm(const double *pq_in, double *pm_out) noexcept->void;
		auto s_pm2pq(const double *pm_in, double *pq_out) noexcept->void;
		
		auto s_wm2we(const double *rm_in, const double *wm_in, double *we_out, std::size_t rm_ld = 3, std::size_t wm_ld = 3) noexcept->void;
		auto s_we2wm(const double *we_in, const double *rm_in, double *wm_out, std::size_t rm_ld = 3, std::size_t wm_ld = 3) noexcept->void;
		auto s_wq2we(const double *rq_in, const double *wq_in, double *we_out) noexcept->void;
		auto s_we2wq(const double *we_in, const double *rq_in, double *wq_out) noexcept->void;
		auto s_vp2vs(const double *pp_in, const double *vp_in, double *vs_out) noexcept->void;
		auto s_vs2vp(const double *vs_in, const double *pp_in, double *vp_out) noexcept->void;
		auto s_we2vs(const double *we_in, double *vs_out) noexcept->void;
		auto s_vs2we(const double *vs_in, double *we_out) noexcept->void;
		auto s_wq2vs(const double *rq_in, const double *wq_in, double *vs_out) noexcept->void;
		auto s_vs2wq(const double *vs_in, const double *rq_in, double *wq_out) noexcept->void;
		auto s_wm2vs(const double *rm_in, const double *wm_in, double *vs_out, std::size_t rm_ld = 3, std::size_t wm_ld = 3) noexcept->void;
		auto s_vs2wm(const double *vs_in, const double *rm_in, double *wm_out, std::size_t rm_ld = 3, std::size_t wm_ld = 3) noexcept->void;
		auto s_ve2vs(const double *pp_in, const double *ve_in, double *vs_out) noexcept->void;
		auto s_vs2ve(const double *vs_in, const double *pp_in, double *ve_out) noexcept->void;
		auto s_vm2vs(const double *pm_in, const double *vm_in, double *vs_out) noexcept->void;
		auto s_vs2vm(const double *vs_in, const double *pm_in, double *vm_out) noexcept->void;
		auto s_vq2vs(const double *pq_in, const double *vq_in, double *vs_out) noexcept->void;
		auto s_vs2vq(const double *vs_in, const double *pq_in, double *vq_out) noexcept->void;

		auto s_xm2xe(const double *rm_in, const double *wm_in, const double *xm_in, double *xe_out, double *we_out = nullptr, std::size_t rm_ld = 3, std::size_t wm_ld = 3, std::size_t xm_ld = 3) noexcept->void;
		auto s_xe2xm(const double *we_in, const double *xe_in, const double *rm_in, double *xm_out, double *wm_out = nullptr, std::size_t rm_ld = 3, std::size_t wm_ld = 3, std::size_t xm_ld = 3) noexcept->void;
		auto s_xq2xe(const double *rq_in, const double *wq_in, const double *xq_in, double *xe_out, double *we_out = nullptr) noexcept->void;
		auto s_xe2xq(const double *we_in, const double *xe_in, const double *rq_in, double *xq_out, double *wq_out = nullptr) noexcept->void;
		auto s_ap2as(const double *pp_in, const double *vp_in, const double *ap_in, double *as_out, double *vs_out = nullptr) noexcept->void;
		auto s_as2ap(const double *vs_in, const double *as_in, const double *pp_in, double *ap_out, double *vp_out = nullptr) noexcept->void;
		auto s_xe2as(const double *xe_in, double *as_out) noexcept->void;
		auto s_as2xe(const double *as_in, double *xe_out) noexcept->void;
		auto s_xq2as(const double *rq_in, const double *wq_in, const double *xq_in, double *as_out, double *vs_out = nullptr) noexcept->void;
		auto s_as2xq(const double *vs_in, const double *as_in, const double *rq_in, double *xq_out, double *wq_out = nullptr) noexcept->void;
		auto s_xm2as(const double *rm_in, const double *wm_in, const double *xm_in, double *as_out, double *vs_out = nullptr, std::size_t rm_ld = 3, std::size_t wm_ld = 3, std::size_t xm_ld = 3) noexcept->void;
		auto s_as2xm(const double *vs_in, const double *as_in, const double *rm_in, double *xm_out, double *wm_out = nullptr, std::size_t rm_ld = 3, std::size_t wm_ld = 3, std::size_t xm_ld = 3) noexcept->void;
		auto s_ae2as(const double *pp_in, const double *ve_in, const double *ae_in, double *as_out, double *vs_out = nullptr) noexcept->void;
		auto s_as2ae(const double *vs_in, const double *as_in, const double *pp_in, double *ae_out, double *ve_out = nullptr) noexcept->void;
		auto s_am2as(const double *pm_in, const double *vm_in, const double *am_in, double *as_out, double *vs_out = nullptr) noexcept->void;
		auto s_as2am(const double *vs_in, const double *as_in, const double *pm_in, double *am_out, double *vm_out = nullptr) noexcept->void;
		auto s_aq2as(const double *pq_in, const double *vq_in, const double *aq_in, double *as_out, double *vs_out = nullptr) noexcept->void;
		auto s_as2aq(const double *vs_in, const double *as_in, const double *pq_in, double *aq_out, double *vq_out = nullptr) noexcept->void;

		auto s_pq2pe(const double *pq_in, double *pe_out, const char *eu_type_in = "313") noexcept->void;
		auto s_pe2pq(const double *pe_in, double *pq_out, const char *eu_type_in = "313") noexcept->void;
		auto s_iv2is(const double * iv_in, double *is_out) noexcept->void;
		auto s_is2iv(const double * is_in, double *iv_out) noexcept->void;
		auto s_im2is(const double mass_in, const double * in_in, const double *pm_in, double *is_out) noexcept->void;

		// 以下函数为同一物理量在不同坐标系之间的转换函数 //
		auto s_pp2pp(const double *relative_pm, const double *from_pp, double *to_pp) noexcept->void;
		auto s_inv_pp2pp(const double *inv_relative_pm, const double *from_pp, double *to_pp) noexcept->void;
		auto s_re2re(const double *relative_pm, const double *from_re, double *to_re, const char *from_eu_type = "313", const char *to_eu_type = "313") noexcept->void;
		auto s_inv_re2re(const double *inv_relative_pm, const double *from_re, double *to_re, const char *from_eu_type = "313", const char *to_eu_type = "313") noexcept->void;
		auto s_rq2rq(const double *relative_pm, const double *from_rq, double *to_rq) noexcept->void;
		auto s_inv_rq2rq(const double *inv_relative_pm, const double *from_rq, double *to_rq) noexcept->void;
		auto s_rm2rm(const double *relative_pm, const double *from_rm, double *to_rm, std::size_t from_rm_ld = 3, std::size_t to_rm_ld = 3) noexcept->void;
		auto s_inv_rm2rm(const double *inv_relative_pm, const double *from_rm, double *to_rm, std::size_t from_rm_ld = 3, std::size_t to_rm_ld = 3) noexcept->void;
		auto s_pe2pe(const double *relative_pm, const double *from_pe, double *to_pe, const char *from_pe_type = "313", const char *to_pe_type = "313") noexcept->void;
		auto s_inv_pe2pe(const double *inv_relative_pm, const double *from_pe, double *to_pe, const char *from_pe_type = "313", const char *to_pe_type = "313") noexcept->void;
		auto s_pq2pq(const double *relative_pm, const double *from_pq, double *to_pq) noexcept->void;
		auto s_inv_pq2pq(const double *inv_relative_pm, const double *from_pq, double *to_pq) noexcept->void;
		auto s_pm2pm(const double *relative_pm, const double *from_pm, double *to_pm) noexcept->void;
		auto s_inv_pm2pm(const double *inv_relative_pm, const double *from_pm, double *to_pm) noexcept->void;

		auto s_vp2vp(const double *relative_pm, const double *relative_vs, const double *from_pp, const double *from_vp, double *to_vp, double *to_pp = nullptr) noexcept->void;
		auto s_inv_vp2vp(const double *inv_relative_pm, const double *inv_relative_vs, const double *from_pp, const double *from_vp, double *to_vp, double *to_pp = nullptr) noexcept->void;
		auto s_we2we(const double *relative_pm, const double *relative_vs, const double *from_we, double *to_we) noexcept->void;
		auto s_inv_we2we(const double *inv_relative_pm, const double *inv_relative_vs, const double *from_we, double *to_we) noexcept->void;
		auto s_wq2wq(const double *relative_pm, const double *relative_vs, const double *from_rq, const double *from_wq, double *to_wq, double *to_rq = nullptr) noexcept->void;
		auto s_inv_wq2wq(const double *inv_relative_pm, const double *inv_relative_vs, const double *from_rq, const double *from_wq, double *to_wq, double *to_rq = nullptr) noexcept->void;
		auto s_wm2wm(const double *relative_pm, const double *relative_vs, const double *from_rm, const double *from_wm, double *to_wm, double *to_rm = nullptr) noexcept->void;
		auto s_inv_wm2wm(const double *inv_relative_pm, const double *inv_relative_vs, const double *from_rm, const double *from_wm, double *to_wm, double *to_rm = nullptr) noexcept->void;
		auto s_ve2ve(const double *relative_pm, const double *relative_vs, const double *from_pp, const double *from_ve, double *to_ve, double *to_pp = nullptr) noexcept->void;
		auto s_inv_ve2ve(const double *inv_relative_pm, const double *inv_relative_vs, const double *from_pp, const double *from_ve, double *to_ve, double *to_pp = nullptr) noexcept->void;
		auto s_vq2vq(const double *relative_pm, const double *relative_vs, const double *from_pq, const double *from_vq, double *to_vq, double *to_pq = nullptr) noexcept->void;
		auto s_inv_vq2vq(const double *inv_relative_pm, const double *inv_relative_vs, const double *from_pq, const double *from_vq, double *to_vq, double *to_pq = nullptr) noexcept->void;
		auto s_vm2vm(const double *relative_pm, const double *relative_vs, const double *from_pm, const double *from_vm, double *to_vm, double *to_pm = nullptr) noexcept->void;
		auto s_inv_vm2vm(const double *inv_relative_pm, const double *inv_relative_vs, const double *from_pm, const double *from_vm, double *to_vm, double *to_pm = nullptr) noexcept->void;
		auto s_vs2vs(const double *relative_pm, const double *relative_vs, const double *from_vs, double *to_vs) noexcept->void;
		auto s_inv_vs2vs(const double *inv_relative_pm, const double *inv_relative_vs, const double *from_vs, double *to_vs) noexcept->void;

		auto s_ap2ap(const double *relative_pm, const double *relative_vs, const double *relative_as, 
			const double *from_pp, const double *from_vp, const double *from_ap, double *to_ap, double *to_vp = nullptr, double *to_pp = nullptr) noexcept->void;
		auto s_inv_ap2ap(const double *inv_relative_pm, const double *inv_relative_vs, const double *inv_relative_as, 
			const double *from_pp, const double *from_vp, const double *from_ap, double *to_ap, double *to_vp = nullptr, double *to_pp = nullptr) noexcept->void;
		auto s_xe2xe(const double *relative_pm, const double *relative_as, const double *from_xe, double *to_xe) noexcept->void;
		auto s_inv_xe2xe(const double *inv_relative_pm, const double *inv_relative_as, const double *from_xe, double *to_xe) noexcept->void;
		auto s_xq2xq(const double *relative_pm, const double *relative_vs, const double *relative_as, 
			const double *from_rq, const double *from_wq, const double *from_xq, double *to_xq, double *to_wq = nullptr, double *to_rq = nullptr) noexcept->void;
		auto s_inv_xq2xq(const double *inv_relative_pm, const double *inv_relative_vs, const double *inv_relative_as, 
			const double *from_rq, const double *from_wq, const double *from_xq, double *to_xq, double *to_wq = nullptr, double *to_rq = nullptr) noexcept->void;
		auto s_xm2xm(const double *relative_pm, const double *relative_vs, const double *relative_as, 
			const double *from_rm, const double *from_wm, const double *from_xm, double *to_xm, double *to_wm = nullptr, double *to_rm = nullptr) noexcept->void;
		auto s_inv_xm2xm(const double *inv_relative_pm, const double *inv_relative_vs, const double *inv_relative_as, 
			const double *from_rm, const double *from_wm, const double *from_xm, double *to_xm, double *to_wm = nullptr, double *to_rm = nullptr) noexcept->void;
		auto s_ae2ae(const double *relative_pm, const double *relative_vs, const double *relative_as, 
			const double *from_pp, const double *from_ve, const double *from_ae, double *to_ae, double *to_ve = nullptr, double *to_pp = nullptr) noexcept->void;
		auto s_inv_ae2ae(const double *inv_relative_pm, const double *inv_relative_vs, const double *inv_relative_as, 
			const double *from_pp, const double *from_ve, const double *from_ae, double *to_ae, double *to_ve = nullptr, double *to_pp = nullptr) noexcept->void;
		auto s_aq2aq(const double *relative_pm, const double *relative_vs, const double *relative_as, 
			const double *from_pq, const double *from_vq, const double *from_aq, double *to_aq, double *to_vq = nullptr, double *to_pq = nullptr) noexcept->void;
		auto s_inv_aq2aq(const double *inv_relative_pm, const double *inv_relative_vs, const double *inv_relative_as, 
			const double *from_pq, const double *from_vq, const double *from_aq, double *to_aq, double *to_vq = nullptr, double *to_pq = nullptr) noexcept->void;
		auto s_am2am(const double *relative_pm, const double *relative_vs, const double *relative_as, 
			const double *from_pm, const double *from_vm, const double *from_am, double *to_am, double *to_vm = nullptr, double *to_pm = nullptr) noexcept->void;
		auto s_inv_am2am(const double *inv_relative_pm, const double *inv_relative_vs, const double *inv_relative_as, 
			const double *from_pm, const double *from_vm, const double *from_am, double *to_am, double *to_vm = nullptr, double *to_pm = nullptr) noexcept->void;
		auto s_as2as(const double *relative_pm, const double *relative_vs, const double *relative_as,
			const double *from_vs, const double *from_as, double *to_as, double *to_vs = nullptr) noexcept->void;
		auto s_inv_as2as(const double *inv_relative_pm, const double *inv_relative_vs, const double *inv_relative_as,
			const double *from_vs, const double *from_as, double *to_as, double *to_vs = nullptr) noexcept->void;

		auto s_fs2fs(const double *relative_pm, const double *from_fs, double *to_fs) noexcept->void;
		auto s_inv_fs2fs(const double *inv_relative_pm, const double *from_fs, double *to_fs) noexcept->void;
		auto s_is2is(const double *relative_pm, const double *from_is, double *to_is) noexcept->void;
		auto s_inv_is2is(const double *inv_relative_pm, const double *from_is, double *to_is) noexcept->void;

		/// \brief 构造6x6的力转换矩阵
		///
		///  其中，tmf = [rm (3x3),  pp x rm (3x3); O (3x3), rm (3x3)]
		///
		///
		auto s_tmf(const double *pm_in, double *tmf_out) noexcept->void;
		/// \brief 构造6x6的速度转换矩阵
		///
		///  其中，tmv = [rm (3x3),  O (3x3); pp x rm (3x3), rm (3x3)]
		///
		///
		auto s_tmv(const double *pm_in, double *tmv_out) noexcept->void;
		/// \brief 根据位姿矩阵转换六维力向量
		///
		/// 等同于： vec_out = tmf(pm_in) * fce_in
		///
		///
		auto s_tf(const double *pm_in, const double *fce_in, double *vec_out) noexcept->void;
		/// \brief 根据位姿矩阵转换六维力向量
		///
		/// 等同于： vec_out = alpha * tmf(pm_in) * fce_in + beta * vec_out
		///
		///
		auto s_tf(double alpha, const double *pm_in, const double *fce_in, double beta, double *vec_out) noexcept->void;
		/// \brief 根据位姿矩阵转换六维力矩阵
		///
		/// 等同于： m_out = tmf(pm_in) * fces_in
		///
		///
		auto s_tf_n(int n, const double *pm_in, const double *fces_in, double *m_out) noexcept->void;
		/// \brief 根据位姿矩阵转换六维力矩阵
		///
		/// 等同于： m_out = alpha * tmf(pm_in) * fces_in + beta * m_out
		///
		///
		auto s_tf_n(int n, double alpha, const double *pm_in, const double *fces_in, double beta, double *m_out) noexcept->void;
		/// \brief 根据位姿矩阵的逆矩阵转换六维力向量
		///
		/// 等同于： vec_out = tmv(pm_in^-1) * fce_in
		///
		///
		auto s_inv_tf(const double *inv_pm_in, const double *fce_in, double *vec_out) noexcept->void;
		/// \brief 根据位姿矩阵的逆矩阵转换六维速度向量
		///
		/// 等同于： vec_out = alpha * tmf(pm_in^-1) * vs_in + beta * vec_out
		///
		///
		auto s_inv_tf(double alpha, const double *inv_pm_in, const double *vs_in, double beta, double *vec_out) noexcept->void;
		/// \brief 根据位姿矩阵转换六维速度向量
		///
		/// 等同于： vec_out = tmv(pm_in) * vs_in
		///
		///
		auto s_tv(const double *pm_in, const double *vs_in, double *vec_out) noexcept->void;
		/// \brief 根据位姿矩阵转换六维速度向量
		///
		/// 等同于： vec_out = alpha * tmv(pm_in) * vs_in + beta * vec_out
		///
		///
		auto s_tv(double alpha, const double *pm_in, const double *vs_in, double beta, double *vec_out) noexcept->void;
		/// \brief 根据位姿矩阵转换六维力矩阵
		///
		/// 等同于： m_out = tmv(pm_in) * fces_in
		///
		///
		auto s_tv_n(int n, const double *pm_in, const double *vels_in, double *m_out) noexcept->void;
		/// \brief 根据位姿矩阵转换六维力矩阵
		///
		/// 等同于： m_out = alpha * tmv(pm_in) * fces_in + beta * m_out
		///
		///
		auto s_tv_n(int n, double alpha, const double *pm_in, const double *vels_in, double beta, double *m_out) noexcept->void;
		/// \brief 根据位姿矩阵的逆矩阵转换六维速度向量
		///
		/// 等同于： vec_out = tmv(pm_in^-1) * vs_in
		///
		///
		auto s_inv_tv(const double *inv_pm_in, const double *vs_in, double *vec_out) noexcept->void;
		/// \brief 根据位姿矩阵的逆矩阵转换六维速度向量
		///
		/// 等同于： vec_out = alpha * tmv(pm_in^-1) * vs_in + beta * vec_out
		///
		///
		auto s_inv_tv(double alpha, const double *inv_pm_in, const double *vs_in, double beta, double *vec_out) noexcept->void;
		/// \brief 根据位姿矩阵的逆矩阵转换六维速度向量
		///
		/// 等同于： vec_out = tmv(pm_in^-1) * vs_in
		///
		///
		auto s_inv_tv_n(int n, const double *inv_pm_in, const double *vs_in, double *vec_out) noexcept->void;
		/// \brief 根据位姿矩阵的逆矩阵转换六维速度向量
		///
		/// 等同于： vec_out = alpha * tmv(pm_in^-1) * vs_in + beta * vec_out
		///
		///
		auto s_inv_tv_n(int n, double alpha, const double *inv_pm_in, const double *vs_in, double beta, double *vec_out) noexcept->void;

		/// \brief 计算三维向量叉乘矩阵
		///
		/// 用来计算：cm_out = \n
		/// [ 0  -z   y \n
		///   z   0  -x \n
		///  -y   x   0 ]
		/// 
		///
		auto s_cm3(const double *cro_vec_in, double *cm_out) noexcept->void;
		/// \brief 计算三维向量叉乘
		///
		/// 用来计算：vec_out = cro_vec_in x vec_in
		///
		///
		auto s_c3(const double *cro_vec_in, const double *vec_in, double *vec_out) noexcept->void;
		/// \brief 计算三维向量叉乘
		///
		/// 用来计算：vec_out = cro_vec_in x vec_in
		///
		///
		auto s_c3(const double *cro_vec_in, const double *vec_in, std::size_t vec_in_ld, double *vec_out, std::size_t vec_out_ld) noexcept->void;
		/// \brief 计算三维向量叉乘
		///
		/// 用来计算：vec_out = alpha * cro_vec_in x vec_in + beta * vec_out
		///
		///
		auto s_c3(double alpha, const double *cro_vec_in, const double *vec_in, double beta, double *vec_out) noexcept->void;
		/// \brief 计算三维向量叉乘
		///
		/// 用来计算：vec_out = alpha * cro_vec_in x vec_in + beta * vec_out
		///
		///
		auto s_c3(double alpha, const double *cro_vec_in, const double *vec_in, std::size_t vec_in_ld, double beta, double *vec_out, std::size_t vec_out_ld) noexcept->void;
		/// \brief 计算三维向量叉乘
		///
		/// 用来计算：vec_out = alpha * cro_vec_in x mat_in + beta * mat_out
		///
		///
		auto s_c3_n(std::size_t n, const double *cro_vec_in, const double *mat_in, std::size_t mat_in_ld, double *mat_out, std::size_t mat_out_ld) noexcept->void;
		/// \brief 计算三维向量叉乘
		///
		/// 用来计算：vec_out = alpha * cro_vec_in x mat_in + beta * mat_out
		///
		///
		auto s_c3_n(std::size_t n, double alpha, const double *cro_vec_in, const double *mat_in, std::size_t mat_in_ld, double beta, double *mat_out, std::size_t mat_out_ld) noexcept->void;
		/// \brief 构造6x6的力叉乘矩阵
		///
		///  其中，cmf = [w x,  O; v x, w x]
		///
		///
		auto s_cmf(const double *vs_in, double *cmf_out) noexcept->void;
		/// \brief 计算六维向量叉乘
		///
		/// 用来计算：vec_out = cro_vec_in xf vec_in
		///
		///
		auto s_cf(const double *cro_vel_in, const double *vec_in, double* vec_out) noexcept->void;
		/// \brief 计算六维向量叉乘
		///
		/// 用来计算：vec_out = alpha * cro_vec_in xf vec_in + beta * vec_out
		///
		///
		auto s_cf(double alpha, const double *cro_vel_in, const double *vec_in, double beta, double* vec_out) noexcept->void;
		/// \brief 构造6x6的速度叉乘矩阵
		///
		///  其中，cmv = \n
		///  [w x,  v x \n
		///   O  ,  w x] \n
		///
		///
		auto s_cmv(const double *vs_in, double *cmv_out) noexcept->void;
		/// \brief 计算六维向量叉乘
		///
		/// 用来计算：vec_out = cro_vec_in xv vec_in
		///
		///
		auto s_cv(const double *cro_vel_in, const double *vec_in, double* vec_out) noexcept->void;
		/// \brief 计算六维向量叉乘
		///
		/// 用来计算：vec_out = alpha * cro_vec_in xv vec_in + beta * vec_out
		///
		///
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

		auto s_nd(int n, double a, double *x, int incx) noexcept->void;// number dot
		auto s_ndv(int n, double a, double *x, int incx, double *y, int incy) noexcept->void;
		auto s_vnm(int n, const double *x, int incx) noexcept->double;// vector norm
		auto s_vsw(int n, double *x, const int incx, double *y, const int incy) noexcept->void;// vector swap
		auto s_vdv(int n, const double *x, const double *y) noexcept->double;
		auto s_vdv(int n, const double *x, int incx, const double *y, int incy) noexcept->double;
		auto s_va(int n, const double* x, double* y) noexcept->void;
		auto s_va(int n, double alpha, const double* x, double beta, double* y) noexcept->void;
		auto s_va(int n, const double* x, int incx, double* y, int incy) noexcept->void;
		auto s_va(int n, double alpha, const double* x, int incx, double beta, double* y, int incy) noexcept->void;
		auto s_vav(int n, const double* x, const double* y, double* z) noexcept->void;
		auto s_vav(int n, double alpha, const double* x, double beta, const double* y, double gamma, double* z) noexcept->void;
		auto s_vav(int n, const double* x, int incx, const double* y, int incy, double* z, int incz) noexcept->void;
		auto s_vav(int n, double alpha, const double* x, int incx, double beta, const double* y, int incy, double gamma, double* z, int incz) noexcept->void;
		auto s_mt(int m, int n, double *A) noexcept->void;// matrix transpose
		auto s_mtm(int m, int n, const double *A, int lda, double *B, int ldb) noexcept->void;// matrix transpose to matrix
		auto s_ma(int m, int n, const double* A, int lda, double* B, int ldb) noexcept->void;
		auto s_ma(int m, int n, double alpha, const double* A, int lda, double beta, double* B, int ldb) noexcept->void;
		auto s_mam(int m, int n, const double* A, int lda, const double* B, int ldb, double *C, int ldc) noexcept->void;
		auto s_mam(int m, int n, double alpha, const double* A, int lda, double beta, const double* B, int ldb, double gamma, double *C, int ldc) noexcept->void;
		auto s_mdm(int m, int n, int k, const double* A, int lda, const double* B, int ldb, double *C, int ldc) noexcept->void;
		auto s_mdm(int m, int n, int k, double alpha, const double* A, int lda, const double* B, int ldb, double beta, double *C, int ldc) noexcept->void;
		auto s_mdmTN(int m, int n, int k, const double* A, int lda, const double* B, int ldb, double *C, int ldc) noexcept->void;
		auto s_mdmTN(int m, int n, int k, double alpha, const double* A, int lda, const double* B, int ldb, double beta, double *C, int ldc) noexcept->void;
		auto s_mdmNT(int m, int n, int k, const double* A, int lda, const double* B, int ldb, double *C, int ldc) noexcept->void;
		auto s_mdmNT(int m, int n, int k, double alpha, const double* A, int lda, const double* B, int ldb, double beta, double *C, int ldc) noexcept->void;

		auto s_dlt_col(const int &dlt_col_num, const int *col_index, const int &m, const int &n, double *A, const int &ldA) noexcept->void;

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
		auto s_pm_dot_v3(const double *pm_in, const double *v3_in, double *v3_out) noexcept->void;
		auto s_inv_pm_dot_v3(const double *inv_pm_in, const double *v3_in, double *v3_out) noexcept->void;
		auto s_m6_dot_v6(const double *m6_in, const double *v6_in, double *v6_out) noexcept->void;

		/// \brief 根据原点和两个坐标轴上的点来求位姿矩阵
		///
		/// 这里原点origin为位姿矩阵pm_out的点，firstAxisPnt位于第一根坐标轴，secondAxisPnt位于第一根坐标轴和第二根坐标轴所构成的平面内
		///
		///
		auto s_axes2pm(const double *origin, const double *firstAxisPnt, const double *secondAxisPnt, double *pm_out, const char *axesOrder = "xy") noexcept->void;
		/// \brief 求解形如 k1 * sin(theta) + k2 * cos(theta) = b 的方程，该方程有2个根
		///
		///
		auto s_sov_theta(double k1, double k2, double b, double *theta_out)noexcept->void;
		auto s_is_equal(int n, const double *v1, const double *v2, double error) noexcept->bool;

		template <typename T>
		inline auto s_sgn(T val)->int { return (T(0) < val) - (val < T(0)); }
		template <typename T>
		inline auto s_sgn2(T val)->int { return val < T(0) ? -1 : 1; }

		auto dsp(const double *p, const int m, const int n, const int begin_row = 0, const int begin_col = 0, int ld = 0)->void;
		template<class Container>
		auto dlmwrite(const char *filename, const Container &container)->void
		{
			std::ofstream file;

			file.open(filename);

			file << std::setprecision(15);

			for (auto i : container)
			{
				for (auto j : i)file << j << "   ";
				file << std::endl;
			}
		}
		auto dlmwrite(const char *filename, const double *mtx, const int m, const int n)->void;
		auto dlmread(const char *filename, double *mtx)->void;
		
	}
}




























#endif
