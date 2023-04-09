#include <cmath>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <algorithm>
#include <limits>
#include <sstream>
#include <regex>
#include <limits>
#include <type_traits>

#include "aris/core/reflection.hpp"

#include "aris/dynamic/kinematics.hpp"
#include "aris/dynamic/model_motion.hpp"

namespace aris::dynamic{
	auto MotionBase::cptCpFromPm(double* cp, const double* makI_pm, const double* makJ_pm, const double* mp)const noexcept->void {
		double pm_j2i[16], ps_j2i[6];
		s_inv_pm_dot_pm(makI_pm, makJ_pm, pm_j2i);
		s_pm2ps(pm_j2i, ps_j2i);
		s_mm(dim(), 1, 6, locCmI(), ColMajor{ dim() }, ps_j2i, 1, cp, 1);
	}
	auto MotionBase::compareP(const double* p1, const double* p2)->double {
		double ret_value{ 0.0 };
		for (int i = 0; i < pSize(); ++i) {
			ret_value = std::max(ret_value, std::abs(p1[i] - p2[i]));
		}
		return ret_value;
	}
	auto MotionBase::setPByMak(const Marker* mak_tool, const Marker* mak_base, const double* p) noexcept->void {
		
		
		double pm_i[16], pm_j[16], pm_relative[16];

		s_inv_pm_dot_pm(*makI()->prtPm(), *mak_tool->prtPm(), pm_relative);

		// 已知 tool wrt base，求 maki wrt makj
		//
		// P_b_t 代表 tool 在 base 下的位姿
		// 
		// P_b_t = (P_G_b)^-1 * P_G_t
		//       = (P_G_n * P_n_b)^-1 * P_G_m * P_m_t
		//       = P_n_b^-1 * P_G_n^-1 * P_G_m * P_m_t
		//
		// P_j_i = (P_G_j)^-1 * P_G_i
		//       = P_n_j^-1 * P_G_n^-1 * P_G_m * P_m_i
		//       = (P_n_j^-1 * P_n_b) * (P_n_b^-1 * P_G_n^-1 * P_G_m * P_m_t) * (P_m_t^-1 * P_m_i)
		//       = (P_n_j^-1 * P_n_b) * P_b_t * (P_m_t^-1 * P_m_i)
		//
		// 其中



	}
	auto MotionBase::updP() noexcept->void {
		double pm_i2j[16];
		s_inv_pm_dot_pm(*makJ()->pm(), *makI()->pm(), pm_i2j);
		cptPFromPm(pm_i2j, const_cast<double*>(this->p()));
	}

	struct Motion::Imp {
		Size clb_frc_id_{ 0 }, clb_id_{ 0 };
		Size component_axis_{ 2 };
		double pitch_{ 0.0 };
		double rotate_range_{ 0.0 };
		double frc_coe_[3]{ 0,0,0 };
		double mp_offset_{ 0 }, mp_factor_{ 1.0 };
		double mp_{ 0 }, mv_{ 0 }, ma_{ 0 }, mf_{ 0 };
		double loc_cm_I[6]{ 0,0,0,0,0,1 };
	};
	auto Motion::locCmI() const noexcept->const double* { return imp_->loc_cm_I; }
	auto Motion::cptCpFromPm(double *cp, const double *makI_pm, const double *makJ_pm, const double *mp)const noexcept->void {
		// 先计算 mak_i 相对于 mak_j 所应该处的位姿 pm_i2j_should_be
		// 再计算 mak_i 相对于 mak_j 真实的位姿     pm_i2j
		// 两者的差值就是应该的补偿量
		double pm_i2j_should_be[16];
		cptPmFromP(mp, pm_i2j_should_be);

		double pm_i2j[16];
		s_inv_pm_dot_pm(makJ_pm, makI_pm, pm_i2j);

		// in real i frame
		double pm_i2j_diff[16], ps_i2j_diff[6];
		s_inv_pm_dot_pm(pm_i2j, pm_i2j_should_be, pm_i2j_diff);

		s_pm2ps(pm_i2j_diff, ps_i2j_diff);

		if (pitch()) {
			// 设若延轴向每转 a 弧度，转动所需做功为 a ，移动所需做功为 (pitch / 2/ PI)^2 * a
			double k = pitch() / 2 / PI;
			cp[0] = ps_i2j_diff[axis() - 3] / (1 + k * k) / k;  // 考虑到可能会移动很长，因此按照移动计算功率
			//cp[0] = ps_i2j_diff[axis()] * (1 + k * k);
		}
		else {
			cp[0] = ps_i2j_diff[axis()];
		}
	}
	auto Motion::cptCv(double *cv)const noexcept->void { cv[0] = mv() * (1 + pitch() * pitch()); }
	auto Motion::cptCa(double *ca)const noexcept->void { 
		Constraint::cptCa(ca); 
		ca[0] += ma() * (1 + pitch() * pitch()); 
	}
	auto Motion::cptPFromPm(const double* pm_i2j, double* p)const noexcept->void {
		double pm_j[16]{1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};
		
		// mp_internal
		double mp_internal;
		if (2 < axis() && axis() < 6 && pitch()) {
			// screw joint //
			// 根据伸缩量，计算所需转动角度 //
			mp_internal = s_sov_axis_distance(pm_j, pm_i2j, axis() - 3) / pitch() * 2 * aris::PI;
		}
		else if (2 < axis() && axis() < 6) {
			auto period = 2 * aris::PI;

			// 计算实际的内置角度 //
			mp_internal = s_sov_axis_distance(pm_j, pm_i2j, axis());

			// 计算角度所对应的中点，这里取mpInternal
			auto mid = std::isfinite(rotateRange()) ? rotateRange() + mpFactor() * mpOffset() / period : mpInternal() / period;

			// 对mid取整、取余 //
			auto t = std::trunc(mid);
			auto mod = mid - t;

			// 将 mp 置于【-周期，+周期】 内
			mp_internal = std::fmod(mp_internal, period);

			// 将 mp 置于【mod-半个周期，mod+半个周期】 内
			while (mp_internal > (mod + 0.5) * period) mp_internal -= period;
			while (mp_internal < (mod - 0.5) * period) mp_internal += period;

			// 叠加需偏移的整数个周期
			mp_internal += t * period;
		}
		else {
			mp_internal = s_sov_axis_distance(pm_j, pm_i2j, axis());
		}

		// mp
		*p = mp_internal / imp_->mp_factor_ - imp_->mp_offset_;
		
	}
	auto Motion::cptPmFromP(const double* p, double* pm_i2j)const noexcept->void {
		double mp_internal = (*p + imp_->mp_offset_)* imp_->mp_factor_;

		double pe[6]{ 0,0,0,0,0,0 };
		pe[axis()] = mp_internal;

		s_pe2pm(pe, pm_i2j, "123");

		// for pitch //
		if (axis() > 2 && pitch()) {
			pm_i2j[3 + axis() * 4] += pitch() * mp_internal / 2 / PI;
		}
	}
	auto Motion::compareP(const double* p1, const double* p2)->double {
		return axis() > 2 ? s_put_into_period(*p1, (*p2) / aris::PI, aris::PI) - (*p2) : (*p1) - (*p2);
	}
	auto Motion::p() const noexcept->const double* { return &imp_->mp_;/*imp_->mp_ / imp_->mp_factor_ - imp_->mp_offset_;*/ }
	auto Motion::updP() noexcept->void{
		double mp;
		double pm_i2j[16];
		s_inv_pm_dot_pm(*makJ()->pm(), *makI()->pm(), pm_i2j);
		cptPFromPm(pm_i2j, &mp);
		setMp(mp);
	}
	auto Motion::setP(const double *mp) noexcept->void { imp_->mp_ = *mp;/*imp_->mp_ = (mp + imp_->mp_offset_) * imp_->mp_factor_;*/ }
	auto Motion::v() const noexcept->const double* { return &imp_->mv_; }
	auto Motion::updV() noexcept->void {
		double vs_i2j[6];
		makI()->getVs(*makJ(), vs_i2j);
		setMv(vs_i2j[axis()]);
	}
	auto Motion::setV(const double *mv) noexcept->void { imp_->mv_ = *mv; }
	auto Motion::a() const noexcept->const double* { return &imp_->ma_; }
	auto Motion::updA() noexcept->void {
		double as_i2j[6];
		makI()->getAs(*makJ(), as_i2j);
		setMa(as_i2j[axis()]);
	}
	auto Motion::setA(const double *ma) noexcept->void { imp_->ma_ = *ma; }
	auto Motion::f() const noexcept->const double* { 
		const_cast<Motion*>(this)->imp_->mf_ = mfDyn() + mfFrc();
		return &imp_->mf_; 
	}
	auto Motion::setF(const double *mf) noexcept->void { 
		double f = *mf - mfFrc();
		setCf(&f);
	}
	auto Motion::setAxis(Size axis)noexcept->void {
		imp_->component_axis_ = axis;
		s_fill(1, 6, 0.0, const_cast<double*>(locCmI()));
		const_cast<double*>(locCmI())[axis] = 1.0;
		
		if(axis > 2)
			const_cast<double*>(locCmI())[axis - 3] = imp_->pitch_ / 2 / PI;
	}
	auto Motion::axis()const noexcept->Size { return imp_->component_axis_; }
	auto Motion::pitch()const noexcept->double {
		return imp_->pitch_;
	}
	auto Motion::setPitch(double pitch)noexcept->void {
		imp_->pitch_ = pitch;
		if (axis() > 2)
			const_cast<double*>(locCmI())[axis() - 3] = imp_->pitch_/ 2 / PI;
	}
	auto Motion::setRotateRange(double range)noexcept->void { imp_->rotate_range_ = range; }
	auto Motion::rotateRange()const noexcept->double { return imp_->rotate_range_; }
	auto Motion::frcCoe()const noexcept->const double3& { return imp_->frc_coe_; }
	auto Motion::setFrcCoe(const double *frc_coe) noexcept->void { std::copy_n(frc_coe, 3, imp_->frc_coe_); }
	auto Motion::mfDyn() const noexcept->double { return *cf(); }
	auto Motion::setMfDyn(double mf_dyn) noexcept->void { setCf(&mf_dyn); }
	auto Motion::mfFrc() const noexcept->double { return s_sgn(imp_->mv_, frcZeroCheck())*frcCoe()[0] + imp_->mv_*frcCoe()[1] + imp_->ma_*frcCoe()[2]; }
	auto Motion::mpOffset()const noexcept->double { return imp_->mp_offset_; }
	auto Motion::setMpOffset(double mp_offset)noexcept->void { imp_->mp_offset_ = mp_offset; }
	auto Motion::mpFactor()const noexcept->double { return imp_->mp_factor_; }
	auto Motion::setMpFactor(double mp_factor)noexcept->void { imp_->mp_factor_ = mp_factor; }
	auto Motion::mpInternal()const noexcept->double { return (imp_->mp_ + imp_->mp_offset_) * imp_->mp_factor_; }
	auto Motion::setMpInternal(double mp_internal)noexcept->void { imp_->mp_ = mp_internal / imp_->mp_factor_ - imp_->mp_offset_; }
	Motion::~Motion() = default;
	Motion::Motion(const std::string &name, Marker* makI, Marker* makJ, Size component_axis, const double *frc_coe, double mp_offset
		, double mp_factor, bool active) : MotionBase(name, makI, makJ, active)
	{
		imp_->mp_offset_ = mp_offset;
		imp_->mp_factor_ = mp_factor;

		static const double default_frc_coe[3]{ 0,0,0 };
		setFrcCoe(frc_coe ? frc_coe : default_frc_coe);
		setAxis(component_axis);
	}
	ARIS_DEFINE_BIG_FOUR_CPP(Motion);

	struct GeneralMotion::Imp {
		double mpm_[4][4]{ { 0 } }, mvs_[6]{ 0 }, mas_[6]{ 0 };
		mutable double p_[16]{ 0.0 }, v_[6]{ 0.0 }, a_[6]{ 0.0 };
		PoseType pose_type_{PoseType::EULER321};
		VelType vel_type_{ VelType::VEL };
		AccType acc_type_{ AccType::ACC };
		FceType fce_type_{ FceType::FCE };
	};
	auto GeneralMotion::setPoseType(PoseType type)->void {
		imp_->pose_type_ = type;
	}
	auto GeneralMotion::poseType()const->PoseType {
		return imp_->pose_type_;
	}
	auto GeneralMotion::setVelType(VelType type)->void {
		imp_->vel_type_ = type;
	}
	auto GeneralMotion::velType()const->VelType {
		return imp_->vel_type_;
	}
	auto GeneralMotion::setAccType(AccType type)->void {
		imp_->acc_type_ = type;
	}
	auto GeneralMotion::accType()const->AccType {
		return imp_->acc_type_;
	}
	auto GeneralMotion::setFceType(FceType type)->void {
		imp_->fce_type_ = type;
	}
	auto GeneralMotion::fceType()const->FceType {
		return imp_->fce_type_;
	}
	auto GeneralMotion::eeType()const->EEType {
		switch (poseType()) {
		case GeneralMotion::PoseType::EULER123:return EEType::PE123;
		case GeneralMotion::PoseType::EULER321:return EEType::PE321;
		case GeneralMotion::PoseType::EULER313:return EEType::PE313;
		case GeneralMotion::PoseType::QUATERNION:return EEType::PQ;
		case GeneralMotion::PoseType::POSE_MATRIX:return EEType::PM;
		default:return EEType::UNKNOWN;
		}
	}
	auto GeneralMotion::locCmI() const noexcept->const double*{
		static const double loc_cm_I[36]{ 1,0,0,0,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0,1 };
		return loc_cm_I;
	}
	auto GeneralMotion::cptCpFromPm(double *cp, const double *makI_pm, const double *makJ_pm, const double* mp)const noexcept->void{
		// 先计算 mak_i 相对于 mak_j 所应该处的位姿 pm_i2j_should_be
		// 再计算 mak_i 相对于 mak_j 真实的位姿     pm_i2j
		// 两者的差值就是应该的补偿量
		double pm_i2j_should_be[16];
		cptPmFromP(mp, pm_i2j_should_be);

		double pm_i2j[16];
		s_inv_pm_dot_pm(makJ_pm, makI_pm, pm_i2j);

		// in real i frame
		double pm_i2j_diff[16], ps_i2j_diff[6];
		s_inv_pm_dot_pm(pm_i2j, pm_i2j_should_be, pm_i2j_diff);

		s_pm2ps(pm_i2j_diff, ps_i2j_diff);
		s_vc(6, ps_i2j_diff, cp);
		
		
		//// Pi : mak I 的实际位置
		//// Pj : mak J 的实际位置
		//// Pit: mak I 应该达到的位置
		//// Pc : 需补偿的位姿
		////
		//// 补偿位姿位于 mak I 内，因此有：
		//// Pi * Pc = Pit
		////
		//// 理论上应该有：
		//// Pit = Pj * mpm
		////
		//// 于是：
		//// Pc = Pi^-1 * Pj * mpm

		//double mpm[16];

		//switch (poseType()) {
		//case GeneralMotion::PoseType::EULER123:s_pe2pm(mp, mpm, "123"); break;
		//case GeneralMotion::PoseType::EULER321:s_pe2pm(mp, mpm, "321"); break;
		//case GeneralMotion::PoseType::EULER313:s_pe2pm(mp, mpm, "313"); break;
		//case GeneralMotion::PoseType::QUATERNION:s_pq2pm(mp, mpm); break;
		//case GeneralMotion::PoseType::POSE_MATRIX:s_vc(16, mp, mpm); break;
		//}


		//double pm_it[16];
		//s_pm_dot_pm(makJ_pm, mpm, pm_it);

		//double pm_c[16], ps_c[6];
		//s_inv_pm_dot_pm(makI_pm, pm_it, pm_c);
		//s_pm2ps(pm_c, ps_c);

		//// locCmI为单位矩阵，此时无需相乘
		//s_vc(6, ps_c, cp);
	}
	auto GeneralMotion::cptGlbDmFromPm(double *dm, const double *makI_pm, const double *makJ_pm)const noexcept->void {
		double pm[16];
		s_inv_pm(makI_pm, pm);
		s_tmf(pm, dm);
	}
	auto GeneralMotion::cptCv(double *cv)const noexcept->void { s_inv_tv(*mpm(), mvs(), cv); }
	auto GeneralMotion::cptCa(double *ca)const noexcept->void { Constraint::cptCa(ca); s_inv_tva(*mpm(), mas(), ca); }
	auto GeneralMotion::cptPFromPm(const double* mak_i2j, double* p)const noexcept->void {
		switch (poseType()) {
		case GeneralMotion::PoseType::EULER123:s_pm2pe(mak_i2j, p, "123"); break;
		case GeneralMotion::PoseType::EULER321:s_pm2pe(mak_i2j, p, "321"); break;
		case GeneralMotion::PoseType::EULER313:s_pm2pe(mak_i2j, p, "313"); break;
		case GeneralMotion::PoseType::QUATERNION:s_pm2pq(mak_i2j, p); break;
		case GeneralMotion::PoseType::POSE_MATRIX:s_vc(16, mak_i2j, p); break;
		}
	}
	auto GeneralMotion::cptPmFromP(const double* p, double* pm_i2j)const noexcept->void {
		switch (poseType()) {
		case GeneralMotion::PoseType::EULER123:s_pe2pm(p, pm_i2j, "123"); break;
		case GeneralMotion::PoseType::EULER321:s_pe2pm(p, pm_i2j, "321"); break;
		case GeneralMotion::PoseType::EULER313:s_pe2pm(p, pm_i2j, "313"); break;
		case GeneralMotion::PoseType::QUATERNION:s_pq2pm(p, pm_i2j); break;
		case GeneralMotion::PoseType::POSE_MATRIX:s_vc(16, p, pm_i2j); break;
		}
	}
	auto GeneralMotion::pSize()const noexcept->Size { 
		switch (poseType()) {
		case GeneralMotion::PoseType::EULER123:return 6;
		case GeneralMotion::PoseType::EULER321:return 6;
		case GeneralMotion::PoseType::EULER313:return 6;
		case GeneralMotion::PoseType::QUATERNION:return 7;
		case GeneralMotion::PoseType::POSE_MATRIX:return 16;
		default:return 6;
		}
	}
	auto GeneralMotion::p()const noexcept->const double* { 
		switch (poseType()) {
		case GeneralMotion::PoseType::EULER123:s_pm2pe(*imp_->mpm_, imp_->p_, "123"); break;
		case GeneralMotion::PoseType::EULER321:s_pm2pe(*imp_->mpm_, imp_->p_, "321"); break;
		case GeneralMotion::PoseType::EULER313:s_pm2pe(*imp_->mpm_, imp_->p_, "313"); break;
		case GeneralMotion::PoseType::QUATERNION:s_pm2pq(*imp_->mpm_, imp_->p_); break;
		case GeneralMotion::PoseType::POSE_MATRIX:s_vc(16, *imp_->mpm_, imp_->p_); break;
		}
		
		return imp_->p_; 
	}
	auto GeneralMotion::updP() noexcept->void { s_inv_pm_dot_pm(*makJ()->pm(), *makI()->pm(), *imp_->mpm_); }
	auto GeneralMotion::setP(const double* mp) noexcept->void { 
		switch (poseType()) {
		case GeneralMotion::PoseType::EULER123:s_pe2pm(mp, *imp_->mpm_, "123"); break;
		case GeneralMotion::PoseType::EULER321:s_pe2pm(mp, *imp_->mpm_, "321"); break;
		case GeneralMotion::PoseType::EULER313:s_pe2pm(mp, *imp_->mpm_, "313"); break;
		case GeneralMotion::PoseType::QUATERNION:s_pq2pm(mp, *imp_->mpm_); break;
		case GeneralMotion::PoseType::POSE_MATRIX:s_vc(16, mp, *imp_->mpm_); break;
		}
	}
	auto GeneralMotion::getP(double* mp)const noexcept->void { s_vc(pSize(), p(), mp); }
	auto GeneralMotion::vSize()const noexcept->Size {
		switch (velType()) {
		case GeneralMotion::VelType::VEL:return 6;
		case GeneralMotion::VelType::VEL_SCREW:return 6;
		default:return 6;
		}
	}
	auto GeneralMotion::v()const noexcept->const double* { 
		switch (velType()) {
		case GeneralMotion::VelType::VEL: getMva(imp_->v_);	break;
		case GeneralMotion::VelType::VEL_SCREW:s_vc(6, imp_->mvs_, imp_->v_); break;
		}
		return imp_->v_;
	}
	auto GeneralMotion::updV() noexcept->void { s_inv_vs2vs(*makJ()->pm(), makJ()->vs(), makI()->vs(), imp_->mvs_); }
	auto GeneralMotion::setV(const double* mv) noexcept->void { 
		switch (velType()) {
		case GeneralMotion::VelType::VEL: setMva(mv);break;
		case GeneralMotion::VelType::VEL_SCREW:setMvs(mv); break;
		}
	}
	auto GeneralMotion::getV(double* mv)const noexcept->void {
		switch (velType()) {
		case GeneralMotion::VelType::VEL: getMva(mv); break;
		case GeneralMotion::VelType::VEL_SCREW:getMvs(mv); break;
		}
	}
	auto GeneralMotion::aSize()const noexcept->Size {
		switch (accType()) {
		case GeneralMotion::AccType::ACC:return 6;
		case GeneralMotion::AccType::ACC_SCREW:return 6;
		default:return 6;
		}
	}
	auto GeneralMotion::a()const noexcept->const double* { 
		switch (accType()) {
		case GeneralMotion::AccType::ACC: getMaa(imp_->a_);	break;
		case GeneralMotion::AccType::ACC_SCREW:s_vc(6, imp_->mas_, imp_->a_); break;
		}
		return imp_->a_;
	}
	auto GeneralMotion::updA() noexcept->void { s_inv_as2as(*makJ()->pm(), makJ()->vs(), makJ()->as(), makI()->vs(), makI()->as(), imp_->mas_); }
	auto GeneralMotion::setA(const double* ma) noexcept->void {
		switch (accType()) {
		case GeneralMotion::AccType::ACC: setMaa(ma); break;
		case GeneralMotion::AccType::ACC_SCREW:setMas(ma); break;
		}
	}
	auto GeneralMotion::getA(double* ma)const noexcept->void {
		switch (accType()) {
		case GeneralMotion::AccType::ACC: getMaa(ma); break;
		case GeneralMotion::AccType::ACC_SCREW:getMas(ma); break;
		}
	}
	auto GeneralMotion::mpm()const noexcept->const double4x4& { return imp_->mpm_; }
	auto GeneralMotion::setMpe(const double* pe, const char *type) noexcept->void { s_pe2pm(pe, *imp_->mpm_, type); }
	auto GeneralMotion::setMpq(const double* pq) noexcept->void { s_pq2pm(pq, *imp_->mpm_); }
	auto GeneralMotion::setMpm(const double* pm) noexcept->void { s_vc(16, pm, *imp_->mpm_); }
	auto GeneralMotion::getMpe(double* pe, const char *type)const noexcept->void { s_pm2pe(*imp_->mpm_, pe, type); }
	auto GeneralMotion::getMpq(double* pq)const noexcept->void { s_pm2pq(*imp_->mpm_, pq); }
	auto GeneralMotion::getMpm(double* pm)const noexcept->void { s_vc(16, *imp_->mpm_, pm); }
	auto GeneralMotion::mvs()const noexcept->const double6& { return imp_->mvs_; }
	auto GeneralMotion::setMve(const double* ve, const char *type) noexcept->void {
		double pe[6];
		s_pm2pe(*mpm(), pe, type);
		s_ve2vs(pe, ve, imp_->mvs_, type);
	}
	auto GeneralMotion::setMvq(const double* vq) noexcept->void {
		double pq[7];
		s_pm2pq(*mpm(), pq);
		s_vq2vs(pq, vq, imp_->mvs_);
	}
	auto GeneralMotion::setMvm(const double* vm) noexcept->void { s_vm2vs(*mpm(), vm, imp_->mvs_); }
	auto GeneralMotion::setMva(const double* va) noexcept->void {
		double pp[3];
		s_pm2pp(*mpm(), pp);
		s_va2vs(pp, va, imp_->mvs_);
	}
	auto GeneralMotion::setMvs(const double* vs) noexcept->void { s_vc(6, vs, imp_->mvs_); }
	auto GeneralMotion::getMve(double* ve, const char *type)const noexcept->void {
		double pe[6];
		s_pm2pe(*mpm(), pe, type);
		s_vs2ve(imp_->mvs_, pe, ve, type);
	}
	auto GeneralMotion::getMvq(double* vq)const noexcept->void {
		double pq[7];
		s_pm2pq(*mpm(), pq);
		s_vs2vq(imp_->mvs_, pq, vq);
	}
	auto GeneralMotion::getMvm(double* vm)const noexcept->void { s_vs2vm(imp_->mvs_, *mpm(), vm); }
	auto GeneralMotion::getMva(double* va)const noexcept->void {
		double pp[3];
		s_pm2pp(*mpm(), pp);
		s_vs2va(imp_->mvs_, pp, va);
	}
	auto GeneralMotion::getMvs(double* vs)const noexcept->void { s_vc(6, imp_->mvs_, vs); }
	auto GeneralMotion::mas()const noexcept->const double6& { return imp_->mas_; }
	auto GeneralMotion::setMae(const double* ae, const char *type) noexcept->void {
		double pe[6], ve[6];
		s_pm2pe(*mpm(), pe, type);
		s_vs2ve(mvs(), pe, ve, type);
		s_ae2as(pe, ve, ae, imp_->mas_, nullptr, type);
	}
	auto GeneralMotion::setMaq(const double* aq) noexcept->void {
		double pq[7], vq[7];
		s_pm2pq(*mpm(), pq);
		s_vs2vq(mvs(), pq, vq);
		s_aq2as(pq, vq, aq, imp_->mas_);
	}
	auto GeneralMotion::setMam(const double* am) noexcept->void	{
		double vm[16];
		getMvm(vm);
		s_am2as(*mpm(), vm, am, imp_->mas_);
	}
	auto GeneralMotion::setMaa(const double* aa) noexcept->void	{
		double pp[3], va[6];
		s_pm2pp(*mpm(), pp);
		s_vs2va(mvs(), pp, va);
		s_aa2as(pp, va, aa, imp_->mas_);
	}
	auto GeneralMotion::setMas(const double* as) noexcept->void { s_vc(6, as, imp_->mas_); }
	auto GeneralMotion::getMae(double* ae, const char *type)const noexcept->void {
		double pe[6];
		s_pm2pe(*mpm(), pe, type);
		s_as2ae(mvs(), mas(), pe, ae, nullptr, type);
	}
	auto GeneralMotion::getMaq(double* aq)const noexcept->void {
		double pq[7];
		s_pm2pq(*mpm(), pq);
		s_as2aq(mvs(), mas(), pq, aq);
	}
	auto GeneralMotion::getMam(double* am)const noexcept->void { s_as2am(mvs(), mas(), *mpm(), am); }
	auto GeneralMotion::getMaa(double* aa)const noexcept->void {
		double pp[3];
		s_pm2pp(*mpm(), pp);
		s_as2aa(mvs(), mas(), pp, aa);
	}
	auto GeneralMotion::getMas(double* as)const noexcept->void { s_vc(6, imp_->mas_, as); }
	GeneralMotion::~GeneralMotion() = default;
	GeneralMotion::GeneralMotion(const std::string &name, Marker* makI, Marker* makJ, bool active) : MotionBase(name, makI, makJ, active) {}
	ARIS_DEFINE_BIG_FOUR_CPP(GeneralMotion);

	auto PointMotion::locCmI() const noexcept->const double* {
		static const double loc_cm_I[18]{
			1,0,0,
			0,1,0,
			0,0,1,
			0,0,0,
			0,0,0,
			0,0,0,
		};
		return loc_cm_I;
	}
	auto PointMotion::cptCpFromPm(double *cp, const double *makI_pm, const double *makJ_pm, const double *mp)const noexcept->void {
		// 先计算 mak_i 相对于 mak_j 所应该处的位姿 pm_i2j_should_be
		// 再计算 mak_i 相对于 mak_j 真实的位姿     pm_i2j
		// 两者的差值就是应该的补偿量
		double pm_i2j_should_be[16];
		cptPmFromP(mp, pm_i2j_should_be);

		double pm_i2j[16];
		s_inv_pm_dot_pm(makJ_pm, makI_pm, pm_i2j);

		// in real i frame
		double pm_i2j_diff[16];
		s_inv_pm_dot_pm(pm_i2j, pm_i2j_should_be, pm_i2j_diff);

		// 【注意】：这里不应考虑角度所造成的移动距离，不同于上文
		s_vc(3, pm_i2j_diff + 3, 4, cp, 1);
	}
	auto PointMotion::cptGlbDmFromPm(double *dm, const double *makI_pm, const double *makJ_pm)const noexcept->void {
		double pm[16];
		s_inv_pm(makI_pm, pm);
		s_tmf(pm, dm);
	}
	auto PointMotion::cptCv(double *cv)const noexcept->void { 
		double vp_in_makI[3], vp_in_ground[3];
		s_pm_dot_v3(*makJ()->pm(), v(), vp_in_ground);
		s_inv_pm_dot_v3(*makI()->pm(), vp_in_ground, vp_in_makI);

		s_vc(3, vp_in_makI, cv);
	}
	auto PointMotion::cptCa(double *ca)const noexcept->void {
		Constraint::cptCa(ca);

		// w x R * dr //
		double vp_in_makI[3], vp_in_ground[3];
		s_pm_dot_v3(*makJ()->pm(), v(), vp_in_ground);
		s_inv_pm_dot_v3(*makI()->pm(), vp_in_ground, vp_in_makI);

		double vs_J_in_I[6];
		makJ()->getVs(*makI(), vs_J_in_I);

		s_c3a(vs_J_in_I + 3, vp_in_makI, ca);

		// R * ddr //
		double ap_in_makI[3], ap_in_ground[3];
		s_pm_dot_v3(*makJ()->pm(), a(), ap_in_ground);
		s_inv_pm_dot_v3(*makI()->pm(), ap_in_ground, ap_in_makI);

		s_va(3, ap_in_makI, ca);
	}
	auto PointMotion::cptPFromPm(const double *mak_i2j, double* p)const noexcept->void {
		s_pm2pp(mak_i2j, p);
	}
	auto PointMotion::cptPmFromP(const double* p, double* pm_i2j)const noexcept->void {
		s_eye(4, pm_i2j);
		pm_i2j[3] = p[0];
		pm_i2j[7] = p[1];
		pm_i2j[11] = p[2];
	}
	auto PointMotion::updV() noexcept->void { 
		double vs[6], pp[3];
		s_pm2pp(*makI()->pm(), pp);
		s_inv_vs2vs(*makJ()->pm(), makJ()->vs(), makI()->vs(), vs);
		s_vs2vp(vs, pp, v_);
	}
	auto PointMotion::updA() noexcept->void { 
		double as[6], vs[6], pp[3];
		s_pm2pp(*makI()->pm(), pp);
		s_inv_as2as(*makJ()->pm(), makJ()->vs(), makJ()->as(), makI()->vs(), makI()->as(), as, vs);
		s_as2ap(vs, as, pp, a_);
	}
	PointMotion::~PointMotion() = default;
	PointMotion::PointMotion(const std::string &name, Marker* makI, Marker* makJ, bool active) : MotionTemplate(name, makI, makJ, active){}
	ARIS_DEFINE_BIG_FOUR_CPP(PointMotion);

	struct SphericalMotion::Imp {
		double mrm_[3][3]{ { 0 } };
		mutable double p_[16]{ 0.0 };
		SphericalMotion::PoseType pose_type_{ SphericalMotion::PoseType::EULER123 };
	};
	auto SphericalMotion::setPoseType(PoseType type)->void{
		imp_->pose_type_ = type;
	}
	auto SphericalMotion::poseType()const->PoseType {
		return imp_->pose_type_;
	}
	auto SphericalMotion::locCmI() const noexcept->const double* {
		static const double loc_cm_I[18]{
			0,0,0,
			0,0,0,
			0,0,0,
			1,0,0,
			0,1,0,
			0,0,1,
		};
		return loc_cm_I;
	}
	auto SphericalMotion::cptCpFromPm(double* cp, const double* makI_pm, const double* makJ_pm, const double* mp)const noexcept->void {
		// 先计算 mak_i 相对于 mak_j 所应该处的位姿 pm_i2j_should_be
		// 再计算 mak_i 相对于 mak_j 真实的位姿     pm_i2j
		// 两者的差值就是应该的补偿量
		double pm_i2j_should_be[16];
		cptPmFromP(mp, pm_i2j_should_be);

		double pm_i2j[16];
		s_inv_pm_dot_pm(makJ_pm, makI_pm, pm_i2j);

		// in real i frame
		double pm_i2j_diff[16], ps_i2j_diff[6];
		s_inv_pm_dot_pm(pm_i2j, pm_i2j_should_be, pm_i2j_diff);

		s_pm2ps(pm_i2j_diff, ps_i2j_diff);
		s_vc(3, ps_i2j_diff + 3, cp);
	}
	auto SphericalMotion::cptGlbDmFromPm(double* dm, const double* makI_pm, const double* makJ_pm)const noexcept->void {
		double pm[16], dm1[36];
		s_inv_pm(makI_pm, pm);
		s_tmf(pm, dm1);

		s_vc(18, dm1, dm + 18);
		s_vc(18, dm1 + 18, dm);
	}
	auto SphericalMotion::cptCv(double* cv)const noexcept->void {
		// tbd
	}
	auto SphericalMotion::cptCa(double* ca)const noexcept->void {
		// tbd
	}
	auto SphericalMotion::cptPFromPm(const double* mak_i2j, double* p)const noexcept->void {
		s_pm2re(mak_i2j, p, "123");
	}
	auto SphericalMotion::cptPmFromP(const double* p, double* pm_i2j)const noexcept->void {
		s_eye(4, pm_i2j);
		s_re2pm(p, pm_i2j, "123");
	}
	auto SphericalMotion::pSize()const noexcept->Size {
		switch (poseType()) {
		case SphericalMotion::PoseType::EULER123:return 3;
		case SphericalMotion::PoseType::EULER321:return 3;
		case SphericalMotion::PoseType::EULER313:return 3;
		case SphericalMotion::PoseType::QUATERNION:return 4;
		case SphericalMotion::PoseType::POSE_MATRIX:return 9;
		default:return 6;
		}
	}
	auto SphericalMotion::p()const noexcept->const double* {
		switch (poseType()) {
		case SphericalMotion::PoseType::EULER123:s_rm2re(*imp_->mrm_, imp_->p_, "123"); break;
		case SphericalMotion::PoseType::EULER321:s_rm2re(*imp_->mrm_, imp_->p_, "321"); break;
		case SphericalMotion::PoseType::EULER313:s_rm2re(*imp_->mrm_, imp_->p_, "313"); break;
		case SphericalMotion::PoseType::QUATERNION:s_rm2rq(*imp_->mrm_, imp_->p_); break;
		case SphericalMotion::PoseType::POSE_MATRIX:s_vc(9, *imp_->mrm_, imp_->p_); break;
		}

		return imp_->p_;
	}
	auto SphericalMotion::updP() noexcept->void { s_mm(3, 3, 3, *makJ()->pm(), T(4), *makI()->pm(), 4, *imp_->mrm_, 3); }
	auto SphericalMotion::setP(const double* mp) noexcept->void {
		switch (poseType()) {
		case SphericalMotion::PoseType::EULER123:s_re2rm(mp, *imp_->mrm_, "123"); break;
		case SphericalMotion::PoseType::EULER321:s_re2rm(mp, *imp_->mrm_, "321"); break;
		case SphericalMotion::PoseType::EULER313:s_re2rm(mp, *imp_->mrm_, "313"); break;
		case SphericalMotion::PoseType::QUATERNION:s_rq2rm(mp, *imp_->mrm_); break;
		case SphericalMotion::PoseType::POSE_MATRIX:s_vc(9, mp, *imp_->mrm_); break;
		}
	}
	auto SphericalMotion::getP(double* mp)const noexcept->void { s_vc(pSize(), p(), mp); }
	auto SphericalMotion::updV() noexcept->void {
		// tbd
	}
	auto SphericalMotion::updA() noexcept->void {
		// tbd
	}
	SphericalMotion::~SphericalMotion() = default;
	SphericalMotion::SphericalMotion(const std::string & name, Marker * makI, Marker * makJ, bool active) : MotionTemplate(name, makI, makJ, active) {}
	ARIS_DEFINE_BIG_FOUR_CPP(SphericalMotion);

	struct XyztMotion::Imp { double rotate_range_{ std::numeric_limits<double>::quiet_NaN() }; };
	auto XyztMotion::locCmI() const noexcept->const double* {
		static const double loc_cm_I[24]{
			1,0,0,0,
			0,1,0,0,
			0,0,1,0,
			0,0,0,0,
			0,0,0,0,
			0,0,0,1
		};
		return loc_cm_I;
	}
	auto XyztMotion::cptCpFromPm(double *cp, const double *makI_pm, const double *makJ_pm, const double* mp)const noexcept->void {
		//// 类似general motion，但仅取其中4维
		//// 先生成mpm
		//double mpm[16];
		//s_rmz(mp[3], mpm, 4);
		//s_pp2pm(mp, mpm);
		//s_fill(1, 3, 0.0, mpm + 12);
		//mpm[15] = 1.0;

		//// 类似general motion 进行计算
		//double pm_it[16];
		//s_pm_dot_pm(makJ_pm, mpm, pm_it);

		//double pm_c[16], ps_c[6];
		//s_inv_pm_dot_pm(makI_pm, pm_it, pm_c);
		//s_pm2ps(pm_c, ps_c);

		//// locCmI为单位矩阵，此时无需相乘
		//s_vc(3, ps_c, cp);
		//cp[3] = ps_c[5];

		// 先计算 mak_i 相对于 mak_j 所应该处的位姿 pm_i2j_should_be
		// 再计算 mak_i 相对于 mak_j 真实的位姿     pm_i2j
		// 两者的差值就是应该的补偿量
		double pm_i2j_should_be[16];
		cptPmFromP(mp, pm_i2j_should_be);

		double pm_i2j[16];
		s_inv_pm_dot_pm(makJ_pm, makI_pm, pm_i2j);

		// in real i frame
		double pm_i2j_diff[16], ps_i2j_diff[6];
		s_inv_pm_dot_pm(pm_i2j, pm_i2j_should_be, pm_i2j_diff);
		s_pm2ps(pm_i2j_diff, ps_i2j_diff);

		// 【注意】：这里不应考虑角度所造成的移动距离，不同于上文
		s_vc(3, ps_i2j_diff, cp);
		cp[3] = ps_i2j_diff[5];
	}
	auto XyztMotion::cptGlbDmFromPm(double *dm, const double *makI_pm, const double *makJ_pm)const noexcept->void {
		double pm[16];
		s_inv_pm(makI_pm, pm);
		s_tmf(pm, dm);
	}
	auto XyztMotion::cptCv(double *cv)const noexcept->void {
		//// 这里不能用 point motion 的计算方法，因为point motion 的转动不对移动产生作用（是被动转动）
		//double mpe[6]{ imp_->mp_[0],imp_->mp_[1],imp_->mp_[2], imp_->mp_[3],0,0 };
		//double mpm[16];
		//s_pe2pm(mpe, mpm, "321");
		//
		//double mvs[6]{ 0,0,0,0,0, imp_->vp_[3] };


		//s_vp2vs(imp_->mp_, imp_->vp_, mvs);
		//
		//double cv_local[6];



		//s_inv_tv(mpm, mvs, cv_local);
		////cv_local[0] = -cv_local[0];
		//s_vc(3, cv_local, cv);
		//cv[3] = cv_local[5];
		////
		////aris::dynamic::dsp(1, 4, cv);

		// 点运动所添加的 cv //
		double vp_in_makI[3], vp_in_ground[3];
		s_pm_dot_v3(*makJ()->pm(), v(), vp_in_ground);
		s_inv_pm_dot_v3(*makI()->pm(), vp_in_ground, vp_in_makI);

		s_vc(3, vp_in_makI, cv);

		// 转动所添加的 cv //
		cv[3] = v()[3];
	}
	auto XyztMotion::cptCa(double *ca)const noexcept->void {
		
		///////  以下可能不对 ///////////
		///////  tbd /////
		
		Constraint::cptCa(ca);

		// w x R * dr //
		double vp_in_makI[3], vp_in_ground[3];
		s_pm_dot_v3(*makJ()->pm(), v(), vp_in_ground);
		s_inv_pm_dot_v3(*makI()->pm(), vp_in_ground, vp_in_makI);

		double vs_J_in_I[6];
		makJ()->getVs(*makI(), vs_J_in_I);

		s_c3a(vs_J_in_I + 3, vp_in_makI, ca);

		// R * ddr //
		double ap_in_makI[3], ap_in_ground[3];
		s_pm_dot_v3(*makJ()->pm(), a(), ap_in_ground);
		s_inv_pm_dot_v3(*makI()->pm(), ap_in_ground, ap_in_makI);

		s_va(3, ap_in_makI, ca);


		// 角度 //
		ca[3] += a()[3];
	}
	auto XyztMotion::cptPFromPm(const double* pm_i2j, double* p)const noexcept->void {
		auto period = 2 * aris::PI;

		double mp_internal = std::atan2(pm_i2j[4] - pm_i2j[1], pm_i2j[0] + pm_i2j[5]);

		auto mid = std::isfinite(rotateRange()) ? rotateRange() : this->p()[3] / period;
		// 对mid取整、取余 //
		auto t = std::trunc(mid);
		auto mod = mid - t;

		// 将 mp 置于【-周期，+周期】 内
		mp_internal = std::fmod(mp_internal, period);

		// 将 mp 置于【mod-半个周期，mod+半个周期】 内
		while (mp_internal > (mod + 0.5) * period) mp_internal -= period;
		while (mp_internal < (mod - 0.5) * period) mp_internal += period;

		// 叠加需偏移的整数个周期
		mp_internal += t * period;

		p[0] = pm_i2j[3];
		p[1] = pm_i2j[7];
		p[2] = pm_i2j[11];
		p[3] = mp_internal;
	}
	auto XyztMotion::cptPmFromP(const double* p, double* pm_i2j)const noexcept->void {
		s_eye(4, pm_i2j);
		pm_i2j[3] = p[0];
		pm_i2j[7] = p[1];
		pm_i2j[11] = p[2];
		s_rmz(p[3], pm_i2j, 4);
	}
	auto XyztMotion::updV() noexcept->void {
		double mvs[6];
		s_inv_vs2vs(*makJ()->pm(), makJ()->vs(), makI()->vs(), mvs);
		s_vs2vp(mvs, p_, v_);
		v_[3] = mvs[5];
	}
	auto XyztMotion::updA() noexcept->void {
		double mvs[6], mas[6];
		s_inv_as2as(*makJ()->pm(), makJ()->vs(), makJ()->as(), makI()->vs(), makI()->as(), mas, mvs);
		s_as2ap(mvs, mas, p_, a_);
		a_[3] = mas[5];
	}
	auto XyztMotion::setRotateRange(double range)noexcept->void { imp_->rotate_range_ = range; }
	auto XyztMotion::rotateRange()const noexcept->double { return imp_->rotate_range_; }
	XyztMotion::~XyztMotion() = default;
	XyztMotion::XyztMotion(const std::string &name, Marker* makI, Marker* makJ, bool active) : MotionTemplate(name, makI, makJ, active) {}
	ARIS_DEFINE_BIG_FOUR_CPP(XyztMotion);

	auto PlanarMotion::locCmI() const noexcept->const double* {
		static const double loc_cm_I[18]{
			1,0,0,
			0,1,0,
			0,0,0,
			0,0,0,
			0,0,0,
			0,0,1,
		};
		return loc_cm_I;
	}
	auto PlanarMotion::cptCpFromPm(double* cp, const double* makI_pm, const double* makJ_pm, const double* mp)const noexcept->void {
		//// 类似general motion，但仅取其中3维
		//// 先生成mpm
		//double mpm[16];
		//s_rmz(mp[2], mpm, 4);
		//s_pp2pm(mp, mpm);
		//s_fill(1, 3, 0.0, mpm + 12);
		//mpm[11] = 0.0; // z 为0 
		//mpm[15] = 1.0;

		//// 类似general motion 进行计算
		//double pm_it[16];
		//s_pm_dot_pm(makJ_pm, mpm, pm_it);

		//double pm_c[16], ps_c[6];
		//s_inv_pm_dot_pm(makI_pm, pm_it, pm_c);
		//s_pm2ps(pm_c, ps_c);

		//// locCmI为单位矩阵，此时无需相乘
		//s_vc(2, ps_c, cp);
		//cp[2] = ps_c[5];

		// 先计算 mak_i 相对于 mak_j 所应该处的位姿 pm_i2j_should_be
		// 再计算 mak_i 相对于 mak_j 真实的位姿     pm_i2j
		// 两者的差值就是应该的补偿量
		double pm_i2j_should_be[16];
		cptPmFromP(mp, pm_i2j_should_be);

		double pm_i2j[16];
		s_inv_pm_dot_pm(makJ_pm, makI_pm, pm_i2j);

		// in real i frame
		double pm_i2j_diff[16], ps_i2j_diff[6];
		s_inv_pm_dot_pm(pm_i2j, pm_i2j_should_be, pm_i2j_diff);
		s_pm2ps(pm_i2j_diff, ps_i2j_diff);

		// 【注意】：这里不应考虑角度所造成的移动距离，不同于上文
		s_vc(2, ps_i2j_diff, cp);
		cp[2] = ps_i2j_diff[5];
	}
	auto PlanarMotion::cptGlbDmFromPm(double* dm, const double* makI_pm, const double* makJ_pm)const noexcept->void {
		double pm[16];
		s_inv_pm(makI_pm, pm);
		s_tmf(pm, dm);
	}
	auto PlanarMotion::cptCv(double* cv)const noexcept->void {
		// 点运动所添加的 cv //
		double vp_in_makI[3], vp_in_ground[3];
		s_pm_dot_v3(*makJ()->pm(), v(), vp_in_ground);
		s_inv_pm_dot_v3(*makI()->pm(), vp_in_ground, vp_in_makI);

		s_vc(2, vp_in_makI, cv);

		// 转动所添加的 cv //
		cv[2] = v()[2];
	}
	auto PlanarMotion::cptCa(double* ca)const noexcept->void {

		///////  以下可能不对 ///////////
		///////  tbd /////

		Constraint::cptCa(ca);

		// w x R * dr //
		double vp_in_makI[3], vp_in_ground[3];
		s_pm_dot_v3(*makJ()->pm(), v(), vp_in_ground);
		s_inv_pm_dot_v3(*makI()->pm(), vp_in_ground, vp_in_makI);

		double vs_J_in_I[6];
		makJ()->getVs(*makI(), vs_J_in_I);

		s_c3a(vs_J_in_I + 3, vp_in_makI, ca);

		// R * ddr //
		double ap_in_makI[3], ap_in_ground[3];
		s_pm_dot_v3(*makJ()->pm(), a(), ap_in_ground);
		s_inv_pm_dot_v3(*makI()->pm(), ap_in_ground, ap_in_makI);

		s_va(2, ap_in_makI, ca);


		// 角度 //
		ca[2] += a()[2];
	}
	auto PlanarMotion::cptPFromPm(const double* pm_i2j, double* p)const noexcept->void {
		p[0] = pm_i2j[3];
		p[1] = pm_i2j[7];
		p[2] = std::atan2(pm_i2j[4] - pm_i2j[1], pm_i2j[0] + pm_i2j[5]);
	}
	auto PlanarMotion::cptPmFromP(const double* p, double* pm_i2j)const noexcept->void {
		s_eye(4, pm_i2j);
		pm_i2j[3] = p[0];
		pm_i2j[7] = p[1];
		s_rmz(p[2], pm_i2j, 4);
	}
	auto PlanarMotion::updV() noexcept->void {
		double mvs[6], vp[3];
		s_inv_vs2vs(*makJ()->pm(), makJ()->vs(), makI()->vs(), mvs);
		s_vs2vp(mvs, p(), vp);
		v_[0] = vp[0];
		v_[1] = vp[1];
		v_[2] = mvs[5];
	}
	auto PlanarMotion::updA() noexcept->void {
		double mvs[6], mas[6], ap[3];
		s_inv_as2as(*makJ()->pm(), makJ()->vs(), makJ()->as(), makI()->vs(), makI()->as(), mas, mvs);
		s_as2ap(mvs, mas, p(), ap);
		a_[0] = ap[0];
		a_[1] = ap[1];
		a_[2] = mas[5];
	}
	PlanarMotion::~PlanarMotion() = default;
	PlanarMotion::PlanarMotion(const std::string & name, Marker * makI, Marker * makJ, bool active) : MotionTemplate(name, makI, makJ, active) {}
	ARIS_DEFINE_BIG_FOUR_CPP(PlanarMotion);

	auto XyMotion::locCmI() const noexcept->const double* {
		static const double loc_cm_I[12]{
			1,0,
			0,1,
			0,0,
			0,0,
			0,0,
			0,0,
		};
		return loc_cm_I;
	}
	auto XyMotion::cptCpFromPm(double* cp, const double* makI_pm, const double* makJ_pm, const double* mp)const noexcept->void {
		//// 类似general motion，但仅取其中3维
		//// 先生成mpm
		//double mpm[16]{
		//	1,0,0,mp[0],
		//	0,1,0,mp[1],
		//	0,0,1,0,
		//	0,0,0,1
		//};

		//// 类似general motion 进行计算
		//double pm_it[16];
		//s_pm_dot_pm(makJ_pm, mpm, pm_it);

		//double pm_c[16];
		//s_inv_pm_dot_pm(makI_pm, pm_it, pm_c);

		//cp[0] = pm_c[3];
		//cp[1] = pm_c[7];

		// 先计算 mak_i 相对于 mak_j 所应该处的位姿 pm_i2j_should_be
		// 再计算 mak_i 相对于 mak_j 真实的位姿     pm_i2j
		// 两者的差值就是应该的补偿量
		double pm_i2j_should_be[16];
		cptPmFromP(mp, pm_i2j_should_be);

		double pm_i2j[16];
		s_inv_pm_dot_pm(makJ_pm, makI_pm, pm_i2j);

		// in real i frame
		double pm_i2j_diff[16], ps_i2j_diff[6];
		s_inv_pm_dot_pm(pm_i2j, pm_i2j_should_be, pm_i2j_diff);
		s_pm2ps(pm_i2j_diff, ps_i2j_diff);

		// 【注意】：这里不应考虑角度所造成的移动距离，不同于上文
		s_vc(2, ps_i2j_diff, cp);
	}
	auto XyMotion::cptGlbDmFromPm(double* dm, const double* makI_pm, const double* makJ_pm)const noexcept->void {
		double pm[16];
		s_inv_pm(makI_pm, pm);
		s_tmf(pm, dm);
	}
	auto XyMotion::cptCv(double* cv)const noexcept->void {
		// 点运动所添加的 cv //
		double vp_in_makI[3], vp_in_ground[3];
		s_pm_dot_v3(*makJ()->pm(), v(), vp_in_ground);
		s_inv_pm_dot_v3(*makI()->pm(), vp_in_ground, vp_in_makI);

		s_vc(2, vp_in_makI, cv);
	}
	auto XyMotion::cptCa(double* ca)const noexcept->void {

		///////  以下可能不对 ///////////
		///////  tbd /////

		Constraint::cptCa(ca);

		// w x R * dr //
		double vp_in_makI[3], vp_in_ground[3];
		s_pm_dot_v3(*makJ()->pm(), v(), vp_in_ground);
		s_inv_pm_dot_v3(*makI()->pm(), vp_in_ground, vp_in_makI);

		double vs_J_in_I[6];
		makJ()->getVs(*makI(), vs_J_in_I);

		s_c3a(vs_J_in_I + 3, vp_in_makI, ca);

		// R * ddr //
		double ap_in_makI[3], ap_in_ground[3];
		s_pm_dot_v3(*makJ()->pm(), a(), ap_in_ground);
		s_inv_pm_dot_v3(*makI()->pm(), ap_in_ground, ap_in_makI);

		s_va(2, ap_in_makI, ca);
	}
	auto XyMotion::cptPFromPm(const double* pm_i2j, double* p)const noexcept->void {
		p[0] = pm_i2j[3];
		p[1] = pm_i2j[7];
	}
	auto XyMotion::cptPmFromP(const double* p, double* pm_i2j)const noexcept->void {
		s_eye(4, pm_i2j);
		pm_i2j[3] = p[0];
		pm_i2j[7] = p[1];
	}
	auto XyMotion::updV() noexcept->void {
		double mvs[6], vp[3];
		s_inv_vs2vs(*makJ()->pm(), makJ()->vs(), makI()->vs(), mvs);
		s_vs2vp(mvs, p(), vp);
		p_[0] = vp[0];
		p_[1] = vp[1];
	}
	auto XyMotion::updA() noexcept->void {
		double mvs[6], mas[6], ap[3];
		s_inv_as2as(*makJ()->pm(), makJ()->vs(), makJ()->as(), makI()->vs(), makI()->as(), mas, mvs);
		s_as2ap(mvs, mas, p(), ap);
		a_[0] = ap[0];
		a_[1] = ap[1];
	}
	XyMotion::~XyMotion() = default;
	XyMotion::XyMotion(const std::string & name, Marker * makI, Marker * makJ, bool active) : MotionTemplate(name, makI, makJ, active) {}
	ARIS_DEFINE_BIG_FOUR_CPP(XyMotion);

	ARIS_REGISTRATION{
		auto setMp = [](MotionBase* c, aris::core::Matrix mat)->void {c->setP(mat.data()); };
		auto getMp = [](MotionBase* c)->aris::core::Matrix {	
			return c->p() ? aris::core::Matrix(1, c->pSize(), c->p()) : aris::core::Matrix(1, c->pSize(), 0.0);
		};
		auto setMv = [](MotionBase* c, aris::core::Matrix mat)->void {c->setP(mat.data()); };
		auto getMv = [](MotionBase* c)->aris::core::Matrix {
			return c->v() ? aris::core::Matrix(1, c->vSize(), c->v()) : aris::core::Matrix(1, c->vSize(), 0.0);
		};
		auto setMa = [](MotionBase* c, aris::core::Matrix mat)->void {c->setP(mat.data()); };
		auto getMa = [](MotionBase* c)->aris::core::Matrix {
			return c->a() ? aris::core::Matrix(1, c->aSize(), c->a()) : aris::core::Matrix(1, c->aSize(), 0.0);
		};
		aris::core::class_<MotionBase>("MotionBase")
			.inherit<aris::dynamic::Constraint>()
			.prop("mp", &setMp, &getMp)
			.prop("mv", &setMv, &getMv)
			.prop("ma", &setMa, &getMa)
			;

		auto setMotionFrc = [](Motion* c, aris::core::Matrix mat)->void {c->setFrcCoe(mat.data()); };
		auto getMotionFrc = [](Motion* c)->aris::core::Matrix {	return aris::core::Matrix(1, 3, c->frcCoe()); };

		aris::core::class_<Motion>("Motion")
			.inherit<aris::dynamic::MotionBase>()
			.prop("component", &Motion::setAxis, &Motion::axis)
			.prop("pitch", &Motion::setPitch, &Motion::pitch)
			.prop("rotate_range", &Motion::setRotateRange, &Motion::rotateRange)
			.prop("mp_offset", &Motion::setMpOffset, &Motion::mpOffset)
			.prop("mp_factor", &Motion::setMpFactor, &Motion::mpFactor)
			.prop("frc_coe", &setMotionFrc, &getMotionFrc)
			;

		aris::core::class_<GeneralMotion::PoseType>("GENERAL_MOTION_POS_TYPE")
			.textMethod([](GeneralMotion::PoseType*type)->std::string {
					switch (*type) {
					case GeneralMotion::PoseType::EULER123:return "EULER123";
					case GeneralMotion::PoseType::EULER321:return "EULER321";
					case GeneralMotion::PoseType::EULER313:return "EULER313";
					case GeneralMotion::PoseType::QUATERNION:return "QUATERNION";
					case GeneralMotion::PoseType::POSE_MATRIX:return "POSE_MATRIX";
					default:return "EULER123";
					}
				}, [](GeneralMotion::PoseType* type, std::string_view name)->void {
					if (name == "EULER123")*type = GeneralMotion::PoseType::EULER123;
					if (name == "EULER321")*type = GeneralMotion::PoseType::EULER321;
					if (name == "EULER313")*type = GeneralMotion::PoseType::EULER313;
					if (name == "QUATERNION")*type = GeneralMotion::PoseType::QUATERNION;
					if (name == "POSE_MATRIX")*type = GeneralMotion::PoseType::POSE_MATRIX;
				
				})
			;

		aris::core::class_<GeneralMotion::VelType>("GENERAL_MOTION_VEL_TYPE")
			.textMethod([](GeneralMotion::VelType*type)->std::string {
					switch (*type) {
					case GeneralMotion::VelType::VEL:return "VEL";
					case GeneralMotion::VelType::VEL_SCREW:return "VEL_SCREW";
					default:return "VEL";
					}
				}, [](GeneralMotion::VelType* type, std::string_view name)->void {
					if (name == "VEL")*type = GeneralMotion::VelType::VEL;
					if (name == "VEL_SCREW")*type = GeneralMotion::VelType::VEL_SCREW;
				})
			;

		aris::core::class_<GeneralMotion::AccType>("GENERAL_MOTION_ACC_TYPE")
			.textMethod([](GeneralMotion::AccType*type)->std::string {
					switch (*type) {
					case GeneralMotion::AccType::ACC:return "ACC";
					case GeneralMotion::AccType::ACC_SCREW:return "ACC_SCREW";
					default:return "ACC";
					}
				}, [](GeneralMotion::AccType* type, std::string_view name)->void {
					if (name == "ACC")*type = GeneralMotion::AccType::ACC;
					if (name == "ACC_SCREW")*type = GeneralMotion::AccType::ACC_SCREW;
				})
			;

		aris::core::class_<GeneralMotion>("GeneralMotion")
			.inherit<aris::dynamic::MotionBase>()
			.prop("pose_type", &GeneralMotion::setPoseType, &GeneralMotion::poseType)
			.prop("vel_type", &GeneralMotion::setVelType, &GeneralMotion::velType)
			.prop("acc_type", &GeneralMotion::setAccType, &GeneralMotion::accType)
			;

		aris::core::class_<SphericalMotion::PoseType>("SPHERICAL_MOTION_POS_TYPE")
			.textMethod([](SphericalMotion::PoseType* type)->std::string {
			switch (*type) {
			case SphericalMotion::PoseType::EULER123:return "EULER123";
			case SphericalMotion::PoseType::EULER321:return "EULER321";
			case SphericalMotion::PoseType::EULER313:return "EULER313";
			case SphericalMotion::PoseType::QUATERNION:return "QUATERNION";
			case SphericalMotion::PoseType::POSE_MATRIX:return "POSE_MATRIX";
			default:return "EULER123";
			}
				}, [](SphericalMotion::PoseType* type, std::string_view name)->void {
					if (name == "EULER123")*type = SphericalMotion::PoseType::EULER123;
					if (name == "EULER321")*type = SphericalMotion::PoseType::EULER321;
					if (name == "EULER313")*type = SphericalMotion::PoseType::EULER313;
					if (name == "QUATERNION")*type = SphericalMotion::PoseType::QUATERNION;
					if (name == "POSE_MATRIX")*type = SphericalMotion::PoseType::POSE_MATRIX;

				})
			;

		aris::core::class_<SphericalMotion>("SphericalMotion")
			.inherit<aris::dynamic::MotionBase>()
			.prop("pose_type", &SphericalMotion::setPoseType, &SphericalMotion::poseType)
			;

		aris::core::class_<PointMotion>("PointMotion")
			.inherit<aris::dynamic::MotionBase>()
			;

		aris::core::class_<XyztMotion>("XyztMotion")
			.inherit<aris::dynamic::MotionBase>()
			.prop("rotate_range", &XyztMotion::setRotateRange, &XyztMotion::rotateRange)
			;

		aris::core::class_<PlanarMotion>("PlanarMotion")
			.inherit<aris::dynamic::MotionBase>()
			;
	}
}
