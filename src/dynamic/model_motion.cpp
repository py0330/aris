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
	struct MotionBase::Imp {
		PosType pos_type_{ PosType::UNKNOWN };
		VelType vel_type_{ VelType::UNKNOWN };
		AccType acc_type_{ AccType::UNKNOWN };
		FceType fce_type_{ FceType::UNKNOWN };
		double mem_[48]{ 0.0 }; // 3*16 = 48
	};
	
	auto MotionBase::setPosType(PosType type) -> void { imp_->pos_type_ = type; }
	auto MotionBase::posType()const->PosType { return imp_->pos_type_; }
	auto MotionBase::setVelType(VelType type) -> void { imp_->vel_type_ = type; }
	auto MotionBase::velType()const->VelType { return imp_->vel_type_; }
	auto MotionBase::setAccType(AccType type) -> void { imp_->acc_type_ = type; }
	auto MotionBase::accType()const->AccType { return imp_->acc_type_; }
	auto MotionBase::setFceType(FceType type) -> void { imp_->fce_type_ = type; }
	auto MotionBase::fceType()const->FceType { return imp_->fce_type_; }

	auto MotionBase::p()const noexcept->const double* { return imp_->mem_;}
	auto MotionBase::v()const noexcept->const double* { return imp_->mem_ + 16; }
	auto MotionBase::a()const noexcept->const double* { return imp_->mem_ + 32; }
	auto MotionBase::f()const noexcept->const double* { return cf(); }

	auto MotionBase::cptPFromPm(const double* pm_i2j, double* p)const noexcept->void { 
		s_pm2pos(pm_i2j, posType(), p); 
	}
	auto MotionBase::cptPmFromP(const double* p, double* pm_i2j)const noexcept->void { 
		s_pos2pm(posType(), p, pm_i2j); 
	}
	auto MotionBase::cptVFromVs(const double* vs_i2j, double* v)const noexcept->void { 
		s_vs2vel(posType(), p(), vs_i2j, velType(), v); 
	}
	auto MotionBase::cptVsFromV(const double* v, double* vs_i2j)const noexcept->void { 
		s_vel2vs(posType(), p(), velType(), v, vs_i2j); 
	}
	auto MotionBase::cptAFromAs(const double* as_i2j, double* a)const noexcept->void { 
		// 速度必须用vs，因为可能某些维度是被动的 //
		double vs[6];
		s_inv_vs2vs(*makJ()->pm(), makJ()->vs(), makI()->vs(), vs);
		s_as2acc(posType(), p(), VelType::VS, vs, as_i2j, accType(), a); 
	}
	auto MotionBase::cptAsFromA(const double* a, double* as_i2j)const noexcept->void { 
		// 速度必须用vs，因为可能某些维度是被动的 //
		double vs[6];
		s_inv_vs2vs(*makJ()->pm(), makJ()->vs(), makI()->vs(), vs);
		s_acc2as(posType(), p(), VelType::VS, vs, accType(), a, as_i2j); 
	}

	auto MotionBase::cptCpFromPm(double* cp, const double* makI_pm, const double* makJ_pm, const double* mp)const noexcept->void {
		double pm_j2i[16], ps_j2i[6];
		s_inv_pm_dot_pm(makI_pm, makJ_pm, pm_j2i);
		s_pm2ps(pm_j2i, ps_j2i);
		s_mm(dim(), 1, 6, locCmI(), ColMajor{ dim() }, ps_j2i, 1, cp, 1);
	}
	auto MotionBase::cptCvFromV(double *cv, const double* v)const noexcept->void{


	}
	auto MotionBase::cptCvDiffFromV(double* cv, const double* v)const noexcept->void { 
		cptCvFromV(cv, v);

		double dv[6], dv_in_I[6];
		s_vc(6, makJ()->vs(), dv);
		s_vs(6, makI()->vs(), dv);
		s_inv_tv(*makI()->pm(), dv, dv_in_I);
		s_mma(dim(), 1, 6, locCmI(), ColMajor{ dim() }, dv_in_I, 1, cv, 1);
	}
	auto MotionBase::cptCaFromA(double *ca, const double* a)const noexcept->void{


	}
	auto MotionBase::cptPError(const double* p1, const double* p2)->double {
		double ret_value{ 0.0 };
		for (int i = 0; i < pSize(); ++i) {
			ret_value = std::max(ret_value, std::abs(p1[i] - p2[i]));
		}
		return ret_value;
	}
	auto MotionBase::setPByMak(const Marker* mak_tool, const Marker* mak_base, const double* p) noexcept->void {
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

		double result1[16], result2[16], p_i2j[16];

		double pm_b_t[16];
		cptPmFromP(p, pm_b_t);

		double pm_left[16];
		s_inv_pm_dot_pm(*makJ()->prtPm(), *mak_base->prtPm(), pm_left);

		double pm_right[16];
		s_inv_pm_dot_pm(*mak_tool->prtPm(), *makI()->prtPm(), pm_right);

		s_pm_dot_pm(pm_left, pm_b_t, result1);
		s_pm_dot_pm(result1, pm_right, result2);
		
		cptPFromPm(result2, p_i2j);
		setP(p_i2j);
	}
	auto MotionBase::updP() noexcept->void {
		double pm_i2j[16];
		s_inv_pm_dot_pm(*makJ()->pm(), *makI()->pm(), pm_i2j);
		cptPFromPm(pm_i2j, const_cast<double*>(this->p()));
	}
	auto MotionBase::updV() noexcept->void {
		double vs[6];
		s_inv_vs2vs(*makJ()->pm(), makJ()->vs(), makI()->vs(), vs);
		cptVFromVs(vs, const_cast<double*>(this->v()));

		// double v[16];
		// s_vs2vel(posType(), p(), vs, velType(), v);
		// setV(v);
	}
	auto MotionBase::updA() noexcept->void {
		double vs[6], as[6];
		s_inv_as2as(*makJ()->pm(), makJ()->vs(), makJ()->as(), makI()->vs(), makI()->as(), as, vs);
		cptAFromAs(as, const_cast<double*>(this->a()));
	}
	auto MotionBase::updMakIPm() noexcept->void {
		double pm_j_i[16];
		cptPmFromP(p(), pm_j_i);
		makI()->setPm(*makJ(), pm_j_i);
	}
	auto MotionBase::updMakJPm() noexcept->void {
		double pm_j_i[16], inv_pm[16];
		cptPmFromP(p(), pm_j_i);
		s_inv_pm(pm_j_i, inv_pm);
		makJ()->setPm(*makI(), inv_pm);
	}
	MotionBase::~MotionBase() = default;
	MotionBase::MotionBase(const std::string& name, Marker* makI, Marker* makJ, bool active):Constraint(name, makI, makJ, active), imp_(new Imp)
	{

	}
	ARIS_DEFINE_BIG_FOUR_CPP(MotionBase);
	
	struct Motion::Imp {
		Size clb_frc_id_{ 0 }, clb_id_{ 0 };
		Size component_axis_{ 2 };
		double pitch_{ 0.0 };
		double rotate_range_{ 0.0 };
		double max_mp_{ std::numeric_limits<double>::max() }, min_mp_{ std::numeric_limits<double>::lowest() };
		double max_mv_{ std::numeric_limits<double>::max() }, min_mv_{ std::numeric_limits<double>::lowest() };
		double max_ma_{ std::numeric_limits<double>::max() }, min_ma_{ std::numeric_limits<double>::lowest() };
		double frc_coe_[3]{ 0,0,0 };
		double mp_offset_{ 0 }, mp_factor_{ 1.0 };

		double mf_{ 0 };
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
	auto Motion::cptCvFromV(double *cv, const double* v)const noexcept->void { cv[0] = (*v) * mpFactor() * (1 + pitch() * pitch()); }
	auto Motion::cptCaFromA(double *ca, const double* a)const noexcept->void { 
		Constraint::cptCa(ca);
		ca[0] += (*a) * mpFactor() * (1 + pitch() * pitch()); 
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
		*p = mpInternal2mp(mp_internal);
		
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
	auto Motion::cptPError(const double* p1, const double* p2)->double {
		return std::abs(axis() > 2 ? s_put_into_period(*p1, (*p2)/2.0 / aris::PI, 2*aris::PI) - (*p2) : (*p1) - (*p2));
	}
	auto Motion::cptVFromVs(const double* vs_i2j, double* v)const noexcept->void {
		s_vs2vel(posType(), p(), vs_i2j, velType(), v);
		v[0] /= mpFactor();
	}
	auto Motion::cptVsFromV(const double* v, double* vs_i2j)const noexcept->void {
		double v_internal = v[0] * mpFactor();
		s_vel2vs(posType(), p(), velType(), &v_internal, vs_i2j);
	}
	auto Motion::cptAFromAs(const double* as_i2j, double* a)const noexcept->void {
		s_as2acc(posType(), p(), velType(), v(), as_i2j, accType(), a);
		a[0] /= mpFactor();
	}
	auto Motion::cptAsFromA(const double* a, double* as_i2j)const noexcept->void {
		double a_internal = a[0] * mpFactor();
		s_acc2as(posType(), p(), velType(), v(), accType(), &a_internal, as_i2j);
	}
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

		PosType axis2postype[6]{ PosType::X, PosType::Y, PosType::Z,PosType::A, PosType::B, PosType::C };
		VelType axis2veltype[6]{ VelType::DX, VelType::DY, VelType::DZ,VelType::DA, VelType::DB, VelType::DC };
		AccType axis2acctype[6]{ AccType::D2X, AccType::D2Y, AccType::D2Z,AccType::D2A, AccType::D2B, AccType::D2C };
		FceType axis2fcetype[6]{ FceType::FX, FceType::FY, FceType::FZ,FceType::TX, FceType::TY, FceType::TZ };

		setPosType(axis2postype[axis]);
		setVelType(axis2veltype[axis]);
		setAccType(axis2acctype[axis]);
		setFceType(axis2fcetype[axis]);
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
	auto Motion::setMaxMp(double max_mp)noexcept -> void { imp_->max_mp_ = max_mp; }
	auto Motion::maxMp()const noexcept -> double { return imp_->max_mp_; }
	auto Motion::setMinMp(double min_mp)noexcept -> void { imp_->min_mp_ = min_mp; }
	auto Motion::minMp()const noexcept -> double { return imp_->min_mp_; }
	auto Motion::setMaxMv(double max_mv)noexcept -> void { imp_->max_mv_ = max_mv; }
	auto Motion::maxMv()const noexcept -> double { return imp_->max_mv_; }
	auto Motion::setMinMv(double min_mv)noexcept -> void { imp_->min_mv_ = min_mv; }
	auto Motion::minMv()const noexcept -> double { return imp_->min_mv_; }
	auto Motion::setMaxMa(double max_ma)noexcept -> void { imp_->max_ma_ = max_ma; }
	auto Motion::maxMa()const noexcept -> double { return imp_->max_ma_; }
	auto Motion::setMinMa(double min_ma)noexcept -> void { imp_->min_ma_ = min_ma; }
	auto Motion::minMa()const noexcept -> double { return imp_->min_ma_; }
	auto Motion::frcCoe()const noexcept->const double3& { return imp_->frc_coe_; }
	auto Motion::setFrcCoe(const double *frc_coe) noexcept->void { std::copy_n(frc_coe, 3, imp_->frc_coe_); }
	auto Motion::mfDyn() const noexcept->double { return *cf(); }
	auto Motion::setMfDyn(double mf_dyn) noexcept->void { setCf(&mf_dyn); }
	auto Motion::mfFrc() const noexcept->double { 
		return s_sgn(*v(), frcZeroCheck()) * frcCoe()[0] + (*v()) * frcCoe()[1] + (*a()) * frcCoe()[2];
	}
	auto Motion::mpOffset()const noexcept->double { return imp_->mp_offset_; }
	auto Motion::setMpOffset(double mp_offset)noexcept->void { imp_->mp_offset_ = mp_offset; }
	auto Motion::mpFactor()const noexcept->double { return imp_->mp_factor_; }
	auto Motion::setMpFactor(double mp_factor)noexcept->void { imp_->mp_factor_ = mp_factor; }
	auto Motion::mpInternal()const noexcept->double { return mp2mpInternal(*p()); }
	auto Motion::setMpInternal(double mp_internal)noexcept->void { 
		double p = mpInternal2mp(mp_internal);
		setP(&p);
	}
	auto Motion::mp2mpInternal(double mp)const noexcept->double {
		return (mp + imp_->mp_offset_) * imp_->mp_factor_;
	}
	auto Motion::mpInternal2mp(double mp_internal)const noexcept->double {
		return mp_internal / imp_->mp_factor_ - imp_->mp_offset_;
	}
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
	}
	auto GeneralMotion::cptGlbDmFromPm(double *dm, const double *makI_pm, const double *makJ_pm)const noexcept->void {
		double pm[16];
		s_inv_pm(makI_pm, pm);
		s_tmf(pm, dm);
	}
	auto GeneralMotion::cptCvFromV(double *cv, const double* v)const noexcept->void { 
		double vs[6];
		s_vel2vs(posType(), p(), velType(), v, vs);

		double mpm[16];
		getMpm(mpm);
		s_inv_tv(mpm, vs, cv);
	}
	auto GeneralMotion::cptCaFromA(double *ca, const double* a)const noexcept->void { 
		Constraint::cptCa(ca);

		double as[6];
		s_acc2as(posType(), p(), velType(), v(), accType(), a, as);

		double mpm[16];
		getMpm(mpm);

		s_inv_tva(mpm, as, ca);
	}
	auto GeneralMotion::cptPError(const double* p1, const double* p2)->double {
		double pm1[16], pm2[16], pq1[7], pq2[7];

		s_pos2pm(posType(), p1, pm1);
		s_pm2pq(pm1, pq1);

		s_pos2pm(posType(), p2, pm2);
		s_pm2pq(pm2, pq2);

		if (s_vv(4, pq1 + 3, pq2 + 3) < 0)
			s_iv(4, pq2 + 3);

		double max_error = 0;
		for (int i = 0; i < 7; ++i) {
			max_error = std::max(max_error, std::abs(pq1[i] - pq2[i]));
		}

		return max_error;
	}

	auto GeneralMotion::setMpe(const double* pe, const char *type) noexcept->void { 
		double pm[16];
		s_pe2pm(pe, pm, type);
		s_pos2pos(PosType::PM, pm, posType(), const_cast<double*>(p()));
	}
	auto GeneralMotion::setMpq(const double* pq) noexcept->void { 
		s_pos2pos(PosType::PQ, pq, posType(), const_cast<double*>(p()));
	}
	auto GeneralMotion::setMpm(const double* pm) noexcept->void { 
		s_pos2pos(PosType::PM, pm, posType(), const_cast<double*>(p()));
	}
	auto GeneralMotion::getMpe(double* pe, const char *type)const noexcept->void { 
		// tbd, 目前因为 PosType 没法直接转为字符串表达的欧拉角... //
		double pm[16];
		s_pos2pos(posType(), p(), PosType::PM, pm);
		s_pm2pe(pm, pe, type);
	}
	auto GeneralMotion::getMpq(double* pq)const noexcept->void { 
		s_pos2pos(posType(), p(), PosType::PQ, pq);
	}
	auto GeneralMotion::getMpm(double* pm)const noexcept->void { 
		s_pos2pos(posType(), p(), PosType::PM, pm);
	}
	
	auto GeneralMotion::setMve(const double* ve, const char *type) noexcept->void {
		// tbd, 目前因为 PosType 没法直接转为字符串表达的欧拉角... //
		double pm[16], pe[6], vs[6];
		s_pos2pos(posType(), p(), PosType::PM, pm);

		s_pm2pe(pm, pe, type);
		s_ve2vs(pe, ve, vs, type);

		s_vel2vel(posType(), p(), VelType::VS, vs, velType(), const_cast<double*>(v()));
	}
	auto GeneralMotion::setMvq(const double* vq) noexcept->void {
		s_vel2vel(posType(), p(), VelType::VQ, vq, velType(), const_cast<double*>(v()));
	}
	auto GeneralMotion::setMvm(const double* vm) noexcept->void { 
		s_vel2vel(posType(), p(), VelType::VM, vm, velType(), const_cast<double*>(v()));
	}
	auto GeneralMotion::setMva(const double* va) noexcept->void {
		s_vel2vel(posType(), p(), VelType::VA, va, velType(), const_cast<double*>(v()));
	}
	auto GeneralMotion::setMvs(const double* vs) noexcept->void { 
		s_vel2vel(posType(), p(), VelType::VS, vs, velType(), const_cast<double*>(v()));
	}
	auto GeneralMotion::getMve(double* ve, const char *type)const noexcept->void {
		// tbd, 目前因为 PosType 没法直接转为字符串表达的欧拉角... //
		double pm[16], pe[6], vs[6];
		s_pos2pos(posType(), p(), PosType::PM, pm);
		s_vel2vel(posType(), p(), velType(), v(), VelType::VS, vs);

		s_pm2pe(pm, pe, type);
		s_vs2ve(vs, pe, ve, type);
	}
	auto GeneralMotion::getMvq(double* vq)const noexcept->void {
		s_vel2vel(posType(), p(), velType(), v(), VelType::VQ, vq);
	}
	auto GeneralMotion::getMvm(double* vm)const noexcept->void { 
		s_vel2vel(posType(), p(), velType(), v(), VelType::VM, vm);
	}
	auto GeneralMotion::getMva(double* va)const noexcept->void {
		s_vel2vel(posType(), p(), velType(), v(), VelType::VA, va);
	}
	auto GeneralMotion::getMvs(double* vs)const noexcept->void { 
		s_vel2vel(posType(), p(), velType(), v(), VelType::VS, vs);
	}
	
	auto GeneralMotion::setMae(const double* ae, const char *type) noexcept->void {
		// tbd, 目前因为 PosType 没法直接转为字符串表达的欧拉角... //
		double pm[16], pe[6], ve[6], as[6], vs[6];
		s_pos2pos(posType(), p(), PosType::PM, pm);
		s_vel2vel(posType(), p(), velType(), v(), VelType::VS, vs);

		s_pm2pe(pm, pe, type);
		s_vs2ve(vs, pe, ve, type);
		s_ae2as(pe, ve, ae, as, nullptr, type);
		
		s_acc2acc(posType(), p(), velType(), v(), AccType::AS, as, accType(), const_cast<double*>(a()));
	}
	auto GeneralMotion::setMaq(const double* aq) noexcept->void {
		s_acc2acc(posType(), p(), velType(), v(), AccType::AQ, aq, accType(), const_cast<double*>(a()));
	}
	auto GeneralMotion::setMam(const double* am) noexcept->void	{
		s_acc2acc(posType(), p(), velType(), v(), AccType::AM, am, accType(), const_cast<double*>(a()));
	}
	auto GeneralMotion::setMaa(const double* aa) noexcept->void	{
		s_acc2acc(posType(), p(), velType(), v(), AccType::AA, aa, accType(), const_cast<double*>(a()));
	}
	auto GeneralMotion::setMas(const double* as) noexcept->void { 
		s_acc2acc(posType(), p(), velType(), v(), AccType::AS, as, accType(), const_cast<double*>(a()));
	}
	auto GeneralMotion::getMae(double* ae, const char *type)const noexcept->void {
		// tbd, 目前因为 PosType 没法直接转为字符串表达的欧拉角... //
		double pm[16], pe[6], vs[6], as[6];
		s_pos2pos(posType(), p(), PosType::PM, pm);
		s_vel2vel(posType(), p(), velType(), v(), VelType::VS, vs);
		s_acc2acc(posType(), p(), velType(), v(), accType(), a(), AccType::AS, as);

		s_pm2pe(pm, pe, type);
		s_as2ae(vs, as, pe, ae, nullptr, type);
	}
	auto GeneralMotion::getMaq(double* aq)const noexcept->void {
		s_acc2acc(posType(), p(), velType(), v(), accType(), a(), AccType::AQ, aq);
	}
	auto GeneralMotion::getMam(double* am)const noexcept->void { 
		s_acc2acc(posType(), p(), velType(), v(), accType(), a(), AccType::AM, am);
	}
	auto GeneralMotion::getMaa(double* aa)const noexcept->void {
		s_acc2acc(posType(), p(), velType(), v(), accType(), a(), AccType::AA, aa);
	}
	auto GeneralMotion::getMas(double* as)const noexcept->void { 
		s_acc2acc(posType(), p(), velType(), v(), accType(), a(), AccType::AS, as);
	}
	GeneralMotion::~GeneralMotion() = default;
	GeneralMotion::GeneralMotion(const std::string &name, Marker* makI, Marker* makJ, bool active) : MotionBase(name, makI, makJ, active) {
		setPosType(PosType::PE321);
		setVelType(VelType::VA);
		setAccType(AccType::AA);
		setFceType(FceType::FT);
	}
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
	auto PointMotion::cptCvFromV(double *cv, const double* v)const noexcept->void { 
		double vp_in_makI[3], vp_in_ground[3];
		s_pm_dot_v3(*makJ()->pm(), v, vp_in_ground);
		s_inv_pm_dot_v3(*makI()->pm(), vp_in_ground, vp_in_makI);

		s_vc(3, vp_in_makI, cv);
	}
	auto PointMotion::cptCaFromA(double *ca, const double* a)const noexcept->void {
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
		s_pm_dot_v3(*makJ()->pm(), a, ap_in_ground);
		s_inv_pm_dot_v3(*makI()->pm(), ap_in_ground, ap_in_makI);

		s_va(3, ap_in_makI, ca);
	}

	PointMotion::~PointMotion() = default;
	PointMotion::PointMotion(const std::string &name, Marker* makI, Marker* makJ, bool active) : MotionBase(name, makI, makJ, active){
		setPosType(PosType::XYZ);
		setVelType(VelType::DXYZ);
		setAccType(AccType::D2XYZ);
		setFceType(FceType::FXYZ);
	}
	ARIS_DEFINE_BIG_FOUR_CPP(PointMotion);

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
	auto SphericalMotion::cptCvFromV(double* cv, const double* v)const noexcept->void {
		// tbd
	}
	auto SphericalMotion::cptCaFromA(double* ca, const double* a)const noexcept->void {
		// tbd
	}
	auto SphericalMotion::cptPError(const double* p1, const double* p2)->double {
		double q1[4], q2[4];

		switch (posType()) {
		case PosType::RE123:
			s_re2rq(p1, q1, "123");
			s_re2rq(p2, q2, "123");
			break;
		case PosType::RE321:
			s_re2rq(p1, q1, "321");
			s_re2rq(p2, q2, "321");
			break;
		case PosType::RE313:
			s_re2rq(p1, q1, "313");
			s_re2rq(p2, q2, "313");
			break;
		case PosType::RQ:
			s_vc(4, p1, q1);
			s_vc(4, p2, q2);
			break;
		case PosType::RM:
			s_rm2rq(p1, q1);
			s_rm2rq(p2, q2);
			break;
		default:
			THROW_FILE_LINE("wrong pos type");
			break;
		}

		if (s_vv(4, q1, q2) < 0)
			s_iv(4, q2);

		double max_error = 0;
		for (int i = 0; i < 4; ++i) {
			max_error = std::max(max_error, std::abs(q1[i] - q2[i]));
		}
		return max_error;
	}

	SphericalMotion::~SphericalMotion() = default;
	SphericalMotion::SphericalMotion(const std::string & name, Marker * makI, Marker * makJ, bool active) : MotionBase(name, makI, makJ, active) {
		setPosType(PosType::RE123);
		setVelType(VelType::WA);
		setAccType(AccType::XA);
		setFceType(FceType::TXYZ);
	}
	ARIS_DEFINE_BIG_FOUR_CPP(SphericalMotion);

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
	auto XyztMotion::cptCvFromV(double *cv, const double* v)const noexcept->void {
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
		s_pm_dot_v3(*makJ()->pm(), v, vp_in_ground);
		s_inv_pm_dot_v3(*makI()->pm(), vp_in_ground, vp_in_makI);

		s_vc(3, vp_in_makI, cv);

		// 转动所添加的 cv //
		cv[3] = v[3];
	}
	auto XyztMotion::cptCaFromA(double *ca, const double* a)const noexcept->void {
		
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
		s_pm_dot_v3(*makJ()->pm(), a, ap_in_ground);
		s_inv_pm_dot_v3(*makI()->pm(), ap_in_ground, ap_in_makI);

		s_va(3, ap_in_makI, ca);


		// 角度 //
		ca[3] += a[3];
	}
	auto XyztMotion::cptPError(const double* p1, const double* p2)->double {
		double max_error = 0;
		for (int i = 0; i < 3; ++i) {
			max_error = std::max(max_error, std::abs(p1[i] - p2[i]));
		}
		max_error = std::max(max_error, std::abs(s_put_into_period(p1[3] - p2[3], 0, 2*aris::PI)));

		return max_error;
	}
	XyztMotion::~XyztMotion() = default;
	XyztMotion::XyztMotion(const std::string &name, Marker* makI, Marker* makJ, bool active) : MotionBase(name, makI, makJ, active) {
		setPosType(PosType::XYZT);
		setVelType(VelType::DXYZT);
		setAccType(AccType::D2XYZT);
		setFceType(FceType::FXYZ_TZ);
	}
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
	auto PlanarMotion::cptCvFromV(double* cv, const double* v)const noexcept->void {
		// 点运动所添加的 cv //
		double vp_in_makI[3], vp_in_ground[3];
		s_pm_dot_v3(*makJ()->pm(), v, vp_in_ground);
		s_inv_pm_dot_v3(*makI()->pm(), vp_in_ground, vp_in_makI);

		s_vc(2, vp_in_makI, cv);

		// 转动所添加的 cv //
		cv[2] = v[2];
	}
	auto PlanarMotion::cptCaFromA(double* ca, const double* a)const noexcept->void {

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
		s_pm_dot_v3(*makJ()->pm(), a, ap_in_ground);
		s_inv_pm_dot_v3(*makI()->pm(), ap_in_ground, ap_in_makI);

		s_va(2, ap_in_makI, ca);


		// 角度 //
		ca[2] += a[2];
	}
	auto PlanarMotion::cptPError(const double* p1, const double* p2)->double {
		double max_error = 0;
		for (int i = 0; i < 2; ++i) {
			max_error = std::max(max_error, std::abs(p1[i] - p2[i]));
		}
		max_error = std::max(max_error, std::abs(s_put_into_period(p1[2] - p2[2], 0, 2 * aris::PI)));

		return max_error;
	}
	PlanarMotion::~PlanarMotion() = default;
	PlanarMotion::PlanarMotion(const std::string & name, Marker * makI, Marker * makJ, bool active) : MotionBase(name, makI, makJ, active) {
		setPosType(PosType::XYT);
		setVelType(VelType::DXYT);
		setAccType(AccType::D2XYT);
		setFceType(FceType::FXY_TZ);
	}
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
	auto XyMotion::cptCvFromV(double* cv, const double* v)const noexcept->void {
		// 点运动所添加的 cv //
		double vp_in_makI[3], vp_in_ground[3];
		s_pm_dot_v3(*makJ()->pm(), v, vp_in_ground);
		s_inv_pm_dot_v3(*makI()->pm(), vp_in_ground, vp_in_makI);

		s_vc(2, vp_in_makI, cv);
	}
	auto XyMotion::cptCaFromA(double* ca, const double* a)const noexcept->void {

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
		s_pm_dot_v3(*makJ()->pm(), a, ap_in_ground);
		s_inv_pm_dot_v3(*makI()->pm(), ap_in_ground, ap_in_makI);

		s_va(2, ap_in_makI, ca);
	}

	XyMotion::~XyMotion() = default;
	XyMotion::XyMotion(const std::string & name, Marker * makI, Marker * makJ, bool active) : MotionBase(name, makI, makJ, active) {
		setPosType(PosType::XY);
		setVelType(VelType::DXY);
		setAccType(AccType::D2XY);
		setFceType(FceType::FXY);
	}
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
			.prop("pos_type", &MotionBase::setPosType, &MotionBase::posType)
			.prop("vel_type", &MotionBase::setVelType, &MotionBase::velType)
			.prop("acc_type", &MotionBase::setAccType, &MotionBase::accType)
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
			.prop("max_mp", &Motion::setMaxMp, &Motion::maxMp)
			.prop("min_mp", &Motion::setMinMp, &Motion::minMp)
			.prop("max_mv", &Motion::setMaxMv, &Motion::maxMv)
			.prop("min_mv", &Motion::setMinMv, &Motion::minMv)
			.prop("max_ma", &Motion::setMaxMa, &Motion::maxMa)
			.prop("min_ma", &Motion::setMinMa, &Motion::minMa)
			.prop("frc_coe", &setMotionFrc, &getMotionFrc)
			;

		aris::core::class_<GeneralMotion>("GeneralMotion")
			.inherit<aris::dynamic::MotionBase>()
			;

		aris::core::class_<SphericalMotion>("SphericalMotion")
			.inherit<aris::dynamic::MotionBase>()
			;

		aris::core::class_<PointMotion>("PointMotion")
			.inherit<aris::dynamic::MotionBase>()
			;

		aris::core::class_<XyztMotion>("XyztMotion")
			.inherit<aris::dynamic::MotionBase>()
			//.prop("rotate_range", &XyztMotion::setRotateRange, &XyztMotion::rotateRange)
			;

		aris::core::class_<PlanarMotion>("PlanarMotion")
			.inherit<aris::dynamic::MotionBase>()
			;
	}
}
