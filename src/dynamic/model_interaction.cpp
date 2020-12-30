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

#include "aris/dynamic/model.hpp"

namespace aris::dynamic{

	auto Interaction::prtNameM()const->std::string { return prt_name_M_; }
	auto Interaction::setPrtNameM(std::string_view name)->void { prt_name_M_ = name; }
	auto Interaction::prtNameN()const->std::string { return prt_name_N_; }
	auto Interaction::setPrtNameN(std::string_view name)->void { prt_name_N_ = name; }
	auto Interaction::makNameI()const->std::string { return mak_name_I_; }
	auto Interaction::setMakNameI(std::string_view name)->void { mak_name_I_ = name; }
	auto Interaction::makNameJ()const->std::string { return mak_name_J_; }
	auto Interaction::setMakNameJ(std::string_view name)->void { mak_name_J_ = name; }
	Interaction::Interaction(const std::string &name, Marker *makI, Marker *makJ, bool is_active)
		: DynEle(name, is_active), makI_(makI), makJ_(makJ) 
		, prt_name_M_(makI ? makI->fatherPart().name() : std::string())
		, prt_name_N_(makJ ? makJ->fatherPart().name() : std::string())
		, mak_name_I_(makI ? makI->name() : std::string())
		, mak_name_J_(makJ ? makJ->name() : std::string()){
	}

	struct Constraint::Imp { Size col_id_, blk_col_id_; double cf_[6]{ 0 }; };
	auto Constraint::cf() const noexcept->const double* { return imp_->cf_; }
	auto Constraint::setCf(const double *cf) noexcept->void { return s_vc(dim(), cf, imp_->cf_); }
	auto Constraint::cptCpFromPm(double *cp, const double *makI_pm, const double *makJ_pm)const noexcept->void{
		double pm_j2i[16], ps_j2i[6];
		s_inv_pm_dot_pm(makI_pm, makJ_pm, pm_j2i);
		s_pm2ps(pm_j2i, ps_j2i);
		s_mm(dim(), 1, 6, locCmI(), ColMajor{ dim() }, ps_j2i, 1, cp, 1);
	}
	auto Constraint::cptCv(double *cv)const noexcept->void{
		double dv[6], dv_in_I[6];
		s_vc(6, makJ()->vs(), dv);
		s_vs(6, makI()->vs(), dv);
		s_inv_tv(*makI()->pm(), dv, dv_in_I);
		s_mm(dim(), 1, 6, locCmI(), ColMajor{ dim() }, dv_in_I, 1, cv, 1);
	};
	auto Constraint::cptCa(double *ca)const noexcept->void{
		double vi_cross_vj[6], tem[6];
		s_cv(makI()->vs(), makJ()->vs(), vi_cross_vj);
		s_inv_tv(*makI()->pm(), vi_cross_vj, tem);
		s_mmi(dim(), 1, 6, locCmI(), ColMajor{ dim() }, tem, 1, ca, 1);
	}
	Constraint::~Constraint() = default;
	Constraint::Constraint(const std::string &name, Marker* makI, Marker* makJ, bool is_active) : Interaction(name, makI, makJ, is_active) {}
	ARIS_DEFINE_BIG_FOUR_CPP(Constraint);

	struct Motion::Imp {
		Size clb_frc_id_{ 0 }, clb_id_{ 0 };
		Size component_axis_;
		double frc_coe_[3]{ 0,0,0 };
		double mp_offset_{ 0 }, mp_factor_{ 1.0 };
		double mp_{ 0 }, mv_{ 0 }, ma_{ 0 };
		double loc_cm_I[6];
		int motor_id_{ -1 };
	};
	auto Motion::locCmI() const noexcept->const double* { return imp_->loc_cm_I; }
	auto Motion::cptCpFromPm(double *cp, const double *makI_pm, const double *makJ_pm)const noexcept->void {
		// // old method
		//Constraint::cptCpFromPm(cp, makI_pm, makJ_pm);
		//cp[0] += mp();  

		if (axis() > 2)//角度
		{
			double re[3]{ 0.0 }, rm[9], pm_j_should_be[16];
			re[axis() - 3] = imp_->mp_;
			s_re2rm(re, rm, "123");

			s_vc(16, makJ_pm, pm_j_should_be);
			s_mm(3, 3, 3, makJ_pm, 4, rm, 3, pm_j_should_be, 4);

			double pm_j2i[16], ps_j2i[6];
			s_inv_pm_dot_pm(makI_pm, pm_j_should_be, pm_j2i);
			s_pm2ps(pm_j2i, ps_j2i);

			cp[0] = ps_j2i[axis()];
		}
		else
		{
			double pm_j_should_be[16];
			s_vc(16, makJ_pm, pm_j_should_be);
			s_va(3, imp_->mp_, pm_j_should_be + axis(), 4, pm_j_should_be + 3, 4);

			double pm_j2i[16], ps_j2i[6];
			s_inv_pm_dot_pm(makI_pm, pm_j_should_be, pm_j2i);
			s_pm2ps(pm_j2i, ps_j2i);
			cp[0] = ps_j2i[axis()];
		}
	}
	auto Motion::cptCv(double *cv)const noexcept->void { Constraint::cptCv(cv); cv[0] += mv(); }
	auto Motion::cptCa(double *ca)const noexcept->void { Constraint::cptCa(ca); ca[0] += ma(); }
	auto Motion::updMp() noexcept->void { imp_->mp_ = s_sov_axis_distance(*makJ()->pm(), *makI()->pm(), axis()); }
	auto Motion::updMv() noexcept->void	{
		double vs_i2j[6];
		makI()->getVs(*makJ(), vs_i2j);
		setMv(vs_i2j[axis()]);
	}
	auto Motion::updMa() noexcept->void	{
		double as_i2j[6];
		makI()->getAs(*makJ(), as_i2j);
		setMa(as_i2j[axis()]);
	}
	auto Motion::setAxis(Size axis)->void {
		imp_->component_axis_ = axis;
		s_fill(1, 6, 0.0, const_cast<double*>(locCmI()));
		const_cast<double*>(locCmI())[axis] = 1.0;
	}
	auto Motion::axis()const noexcept->Size { return imp_->component_axis_; }
	auto Motion::frcCoe()const noexcept->const double3& { return imp_->frc_coe_; }
	auto Motion::setFrcCoe(const double *frc_coe) noexcept->void { std::copy_n(frc_coe, 3, imp_->frc_coe_); }
	auto Motion::mp() const noexcept->double { return imp_->mp_ / imp_->mp_factor_ - imp_->mp_offset_; }
	auto Motion::setMp(double mp) noexcept->void { imp_->mp_ = (mp + imp_->mp_offset_) * imp_->mp_factor_; }
	auto Motion::mv() const noexcept->double { return imp_->mv_; }
	auto Motion::setMv(double mv) noexcept->void { imp_->mv_ = mv; }
	auto Motion::ma() const noexcept->double { return imp_->ma_; }
	auto Motion::setMa(double ma) noexcept->void { imp_->ma_ = ma; }
	auto Motion::mf() const noexcept->double { return mfDyn() + mfFrc(); }
	auto Motion::setMf(double mf) noexcept->void { Constraint::imp_->cf_[0] = mf - mfFrc(); }
	auto Motion::mfDyn() const noexcept->double { return Constraint::imp_->cf_[0]; }
	auto Motion::setMfDyn(double mf_dyn) noexcept->void { Constraint::imp_->cf_[0] = mf_dyn; }
	auto Motion::mfFrc() const noexcept->double { return s_sgn(imp_->mv_, frcZeroCheck())*frcCoe()[0] + imp_->mv_*frcCoe()[1] + imp_->ma_*frcCoe()[2]; }
	auto Motion::mpOffset()const noexcept->double { return imp_->mp_offset_; }
	auto Motion::setMpOffset(double mp_offset)noexcept->void { imp_->mp_offset_ = mp_offset; }
	auto Motion::mpFactor()const noexcept->double { return imp_->mp_factor_; }
	auto Motion::setMpFactor(double mp_factor)noexcept->void { imp_->mp_factor_ = mp_factor; }
	auto Motion::mpInternal()const noexcept->double { return imp_->mp_; }
	auto Motion::setMpInternal(double mp_internal)noexcept->void { imp_->mp_ = mp_internal; }
	Motion::~Motion() = default;
	Motion::Motion(const std::string &name, Marker* makI, Marker* makJ, Size component_axis, const double *frc_coe, double mp_offset, double mp_factor, bool active) : Constraint(name, makI, makJ, active)
	{
		imp_->mp_offset_ = mp_offset;
		imp_->mp_factor_ = mp_factor;

		static const double default_frc_coe[3]{ 0,0,0 };
		setFrcCoe(frc_coe ? frc_coe : default_frc_coe);
		setAxis(component_axis);
	}
	ARIS_DEFINE_BIG_FOUR_CPP(Motion);

	GeneralMotionBase::GeneralMotionBase(const std::string &name, Marker* makI, Marker* makJ, bool active) :Constraint(name, makI, makJ, active) {}
	ARIS_DEFINE_BIG_FOUR_CPP(GeneralMotionBase);

	struct GeneralMotion::Imp {
		double mpm_[4][4]{ { 0 } }, mvs_[6]{ 0 }, mas_[6]{ 0 };
	};
	auto GeneralMotion::locCmI() const noexcept->const double*{
		static const double loc_cm_I[36]{ 1,0,0,0,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0,1 };
		return loc_cm_I;
	}
	auto GeneralMotion::cptCpFromPm(double *cp, const double *makI_pm, const double *makJ_pm)const noexcept->void{
		// Pi : mak I 的实际位置
		// Pj : mak J 的实际位置
		// Pit: mak I 应该达到的位置
		// Pc : 需补偿的位姿
		// 理论上应该有：
		// Pi = Pj * mpm
		// 那么就有：
		// Pit = Pj * mpm
		// 于是：
		// Pc = Pi^-1 * Pit

		double pm_it[16];
		s_pm_dot_pm(makJ_pm, *mpm(), pm_it);

		double pm_c[16], ps_c[6];
		s_inv_pm_dot_pm(makI_pm, pm_it, pm_c);
		s_pm2ps(pm_c, ps_c);

		// locCmI为单位矩阵，此时无需相乘
		s_vc(6, ps_c, cp);
	}
	auto GeneralMotion::cptGlbDmFromPm(double *dm, const double *makI_pm, const double *makJ_pm)const noexcept->void {
		double pm[16];
		s_inv_pm(makI_pm, pm);
		s_tmf(pm, dm);
	}
	auto GeneralMotion::cptCv(double *cv)const noexcept->void { Constraint::cptCv(cv); s_inv_tva(*mpm(), mvs(), cv); }
	auto GeneralMotion::cptCa(double *ca)const noexcept->void { Constraint::cptCa(ca); s_inv_tva(*mpm(), mas(), ca); }
	auto GeneralMotion::mp()const noexcept->const double* { return *imp_->mpm_; }
	auto GeneralMotion::updMp() noexcept->void { s_inv_pm_dot_pm(*makJ()->pm(), *makI()->pm(), *imp_->mpm_); }
	auto GeneralMotion::mv()const noexcept->const double* { return imp_->mvs_; }
	auto GeneralMotion::updMv() noexcept->void { s_inv_vs2vs(*makJ()->pm(), makJ()->vs(), makI()->vs(), imp_->mvs_); }
	auto GeneralMotion::ma()const noexcept->const double* { return imp_->mas_; }
	auto GeneralMotion::updMa() noexcept->void { s_inv_as2as(*makJ()->pm(), makJ()->vs(), makJ()->as(), makI()->vs(), makI()->as(), imp_->mas_); }
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
	auto GeneralMotion::mfs() const noexcept->const double6& { return Constraint::imp_->cf_; }
	auto GeneralMotion::setMfs(const double * mfs) noexcept->void { s_vc(6, mfs, Constraint::imp_->cf_); }
	GeneralMotion::~GeneralMotion() = default;
	GeneralMotion::GeneralMotion(const std::string &name, Marker* makI, Marker* makJ, bool active) :GeneralMotionBase(name, makI, makJ, active) {}
	ARIS_DEFINE_BIG_FOUR_CPP(GeneralMotion);

	struct PointMotion::Imp { double mp_[3], vp_[3], ap_[3]; };
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
	auto PointMotion::cptCpFromPm(double *cp, const double *makI_pm, const double *makJ_pm)const noexcept->void {
		double pp_j[3]{ makJ_pm[3], makJ_pm[7], makJ_pm[11], };
		s_inv_pp2pp(makI_pm, pp_j, cp);
		
		// 把 mp 转到 marker I 坐标系下
		double mp_in_ground[3], mp_in_I[3];
		s_pm_dot_v3(makJ_pm, imp_->mp_, mp_in_ground);
		s_inv_pm_dot_v3(makI_pm, mp_in_ground, mp_in_I);
		
		// 因为是 I 想到对于 J，实际上要减去J相对于I的，所以是加法
		s_va(3, mp_in_I, cp);
	}
	auto PointMotion::cptGlbDmFromPm(double *dm, const double *makI_pm, const double *makJ_pm)const noexcept->void {
		double pm[16];
		s_inv_pm(makI_pm, pm);
		s_tmf(pm, dm);
	}
	auto PointMotion::cptCv(double *cv)const noexcept->void { 
		Constraint::cptCv(cv);

		double vp_in_makI[3], vp_in_ground[3];
		s_pm_dot_v3(*makJ()->pm(), imp_->vp_, vp_in_ground);
		s_inv_pm_dot_v3(*makI()->pm(), vp_in_ground, vp_in_makI);

		s_va(3, vp_in_makI, cv);
	}
	auto PointMotion::cptCa(double *ca)const noexcept->void {
		Constraint::cptCa(ca);

		// w x R * dr //
		double vp_in_makI[3], vp_in_ground[3];
		s_pm_dot_v3(*makJ()->pm(), imp_->vp_, vp_in_ground);
		s_inv_pm_dot_v3(*makI()->pm(), vp_in_ground, vp_in_makI);

		double vs_J_in_I[6];
		makJ()->getVs(*makI(), vs_J_in_I);

		s_c3a(vs_J_in_I + 3, vp_in_makI, ca);

		// R * ddr //
		double ap_in_makI[3], ap_in_ground[3];
		s_pm_dot_v3(*makJ()->pm(), imp_->ap_, ap_in_ground);
		s_inv_pm_dot_v3(*makI()->pm(), ap_in_ground, ap_in_makI);

		s_va(3, ap_in_makI, ca);
	}
	auto PointMotion::mp()const noexcept->const double* { return imp_->mp_; }
	auto PointMotion::updMp() noexcept->void { 
		double pp[3];
		s_pm2pp(*makI()->pm(), pp);
		s_inv_pp2pp(*makJ()->pm(), pp, imp_->mp_);
	}
	auto PointMotion::setMp(const double *mp) noexcept->void { s_vc(3, mp, imp_->mp_); }
	auto PointMotion::getMp(double *mp) noexcept->void { s_vc(3, imp_->mp_, mp); }
	auto PointMotion::mv()const noexcept->const double* { return imp_->vp_; }
	auto PointMotion::updMv() noexcept->void { 
		double vs[6], pp[3];
		s_pm2pp(*makI()->pm(), pp);
		s_inv_vs2vs(*makJ()->pm(), makJ()->vs(), makI()->vs(), vs);
		s_vs2vp(vs, pp, imp_->vp_);
	}
	auto PointMotion::setMv(const double *mv) noexcept->void { s_vc(3, mv, imp_->vp_); }
	auto PointMotion::getMv(double *mv) noexcept->void { s_vc(3, imp_->vp_, mv); }
	auto PointMotion::ma()const noexcept->const double* { return imp_->ap_; }
	auto PointMotion::updMa() noexcept->void { 
		double as[6], vs[6], pp[3];
		s_pm2pp(*makI()->pm(), pp);
		s_inv_as2as(*makJ()->pm(), makJ()->vs(), makJ()->as(), makI()->vs(), makI()->as(), as, vs);
		s_as2ap(vs, as, pp, imp_->ap_);
	}
	auto PointMotion::setMa(const double *ma) noexcept->void { s_vc(3, ma, imp_->ap_); }
	auto PointMotion::getMa(double *ma) noexcept->void { s_vc(3, imp_->ap_, ma); }
	PointMotion::~PointMotion() = default;
	PointMotion::PointMotion(const std::string &name, Marker* makI, Marker* makJ, bool active) :GeneralMotionBase(name, makI, makJ, active) {}
	ARIS_DEFINE_BIG_FOUR_CPP(PointMotion);

	auto RevoluteJoint::locCmI() const noexcept->const double* {
		static const double loc_cm_I[30] {
			1,0,0,0,0,
			0,1,0,0,0,
			0,0,1,0,0,
			0,0,0,1,0,
			0,0,0,0,1,
			0,0,0,0,0
		};
		return loc_cm_I;
	}
	auto RevoluteJoint::cptCpFromPm(double *cp, const double *makI_pm, const double *makJ_pm)const noexcept->void {
		double pm_j_in_i[16];
		s_inv_pm_dot_pm(makI_pm, makJ_pm, pm_j_in_i);

		cp[0] = pm_j_in_i[3];
		cp[1] = pm_j_in_i[7];
		cp[2] = pm_j_in_i[11];

		// 这里用i的z轴叉乘j的z轴，在i坐标系下，因此叉乘出来有如下结果:
		cp[3] = -pm_j_in_i[6];
		cp[4] = pm_j_in_i[2];
	}
	auto RevoluteJoint::cptGlbDmFromPm(double *dm, const double *makI_pm, const double *makJ_pm)const noexcept->void {
		double pm[16];
		s_inv_pm(makI_pm, pm);
		s_tmf(pm, dm);
	}
	RevoluteJoint::RevoluteJoint(const std::string &name, Marker* makI, Marker* makJ) : Joint(name, makI, makJ) {}

	auto PrismaticJoint::locCmI() const noexcept->const double* {
		static const double loc_cm_I[30] {
			1,0,0,0,0,
			0,1,0,0,0,
			0,0,0,0,0,
			0,0,1,0,0,
			0,0,0,1,0,
			0,0,0,0,1
		};
		return loc_cm_I;
	}
	auto PrismaticJoint::cptCpFromPm(double *cp, const double *makI_pm, const double *makJ_pm)const noexcept->void {
		double pm_j2i[16], ps_j2i[6];
		s_inv_pm_dot_pm(makI_pm, makJ_pm, pm_j2i);
		s_pm2ps(pm_j2i, ps_j2i);

		// 此时位移差值在makI()坐标系中
		s_vc(2, ps_j2i, cp);
		s_vc(3, ps_j2i + 3, cp + 2);
	}
	auto PrismaticJoint::cptGlbDmFromPm(double *dm, const double *makI_pm, const double *makJ_pm)const noexcept->void {
		double pm[16];
		s_inv_pm(makI_pm, pm);
		s_tmf(pm, dm);

		s_swap_m(1, 6, dm + 12, dm + 18);
		s_swap_m(1, 6, dm + 18, dm + 24);
		s_swap_m(1, 6, dm + 24, dm + 30);
	}
	PrismaticJoint::PrismaticJoint(const std::string &name, Marker* makI, Marker* makJ) : Joint(name, makI, makJ) {}

	struct UniversalJoint::Imp { double loc_cm_I[24]; };
	auto UniversalJoint::locCmI() const noexcept->const double* {
		const double axis_iz_i[3]{ 0,0,1 };
		double axis_jz_g[3], axis_jz_m[3];

		s_pm_dot_v3(*makJ()->fatherPart().pm(), &makJ()->prtPm()[0][2], 4, axis_jz_g, 1);
		s_inv_pm_dot_v3(*makI()->fatherPart().pm(), axis_jz_g, axis_jz_m);

		// 应该求 axis_iz_i(x1 y1 z1) x axis_jz_i(x2 y2 z2), 但是因为axis_iz_i为单位向量(0,0,1)
		// 那么，这里应该为：
		// [ -y2 x2 0 ]
		double x2 = makI()->prtPm()[0][0] * axis_jz_m[0] + makI()->prtPm()[1][0] * axis_jz_m[1] + makI()->prtPm()[2][0] * axis_jz_m[2];
		double y2 = makI()->prtPm()[0][1] * axis_jz_m[0] + makI()->prtPm()[1][1] * axis_jz_m[1] + makI()->prtPm()[2][1] * axis_jz_m[2];

		double norm = std::sqrt(x2*x2 + y2 * y2);

		const_cast<double*>(imp_->loc_cm_I)[15] = -y2 / norm;
		const_cast<double*>(imp_->loc_cm_I)[19] = x2 / norm;

		return imp_->loc_cm_I;
	}
	auto UniversalJoint::cptGlbDmFromPm(double *dm, const double *makI_pm, const double *makJ_pm)const noexcept->void {
		double x2 = makI_pm[0] * makJ_pm[2] + makI_pm[4] * makJ_pm[6] + makI_pm[8] * makJ_pm[10];
		double y2 = makI_pm[1] * makJ_pm[2] + makI_pm[5] * makJ_pm[6] + makI_pm[9] * makJ_pm[10];

		double norm = std::sqrt(x2*x2 + y2 * y2);

		double pm[16];
		s_inv_pm(makI_pm, pm);
		s_tmf(pm, dm);

		double r[4]{ -y2 / norm, x2 / norm, -x2 / norm, -y2 / norm };
		s_mm(2, 6, 2, r, dm + 18, pm);
		s_mc(2, 6, pm, dm + 18);
	}
	auto UniversalJoint::cptGlbCmFromPm(double *cmI, double *cmJ, const double *makI_pm, const double *makJ_pm)const noexcept->void	{
		static double loc_cst[6][4]{
			1,0,0,0,
			0,1,0,0,
			0,0,1,0,
			0,0,0,0,
			0,0,0,0,
			0,0,0,0,
		};

		double x2 = makI_pm[0] * makJ_pm[2] + makI_pm[4] * makJ_pm[6] + makI_pm[8] * makJ_pm[10];
		double y2 = makI_pm[1] * makJ_pm[2] + makI_pm[5] * makJ_pm[6] + makI_pm[9] * makJ_pm[10];

		double norm = std::sqrt(x2 * x2 + y2 * y2);

		loc_cst[3][3] = -y2 / norm;
		loc_cst[4][3] = x2 / norm;

		s_tf_n(dim(), makI_pm, *loc_cst, cmI);
		s_mi(6, dim(), cmI, cmJ);
	}
	auto UniversalJoint::cptCa(double *ca)const noexcept->void {
		Constraint::cptCa(ca);

		double tem[6];

		// update makI的z轴 和 makJ的z轴
		const double axis_i_m[3]{ makI()->prtPm()[0][2] ,makI()->prtPm()[1][2] ,makI()->prtPm()[2][2] };
		double axis_j_m[3];
		s_pm_dot_v3(*makJ()->fatherPart().pm(), &makJ()->prtPm()[0][2], 4, tem, 1);
		s_inv_pm_dot_v3(*makI()->fatherPart().pm(), tem, axis_j_m);

		// compute c_dot //
		double wm_in_m[3], wn_in_m[3];
		s_inv_pm_dot_v3(*makI()->fatherPart().pm(), makI()->fatherPart().vs() + 3, wm_in_m);
		s_inv_pm_dot_v3(*makI()->fatherPart().pm(), makJ()->fatherPart().vs() + 3, wn_in_m);

		double iwm = s_vv(3, axis_i_m, wm_in_m);
		double jwm = s_vv(3, axis_j_m, wm_in_m);
		double iwn = s_vv(3, axis_i_m, wn_in_m);
		double jwn = s_vv(3, axis_j_m, wn_in_m);

		ca[3] += 2 * jwm*iwn - jwm * iwm - jwn * iwn;
	}
	auto UniversalJoint::cptCpFromPm(double *cp, const double *makI_pm, const double *makJ_pm)const noexcept->void {
		double pm_j_in_i[16];
		s_inv_pm_dot_pm(makI_pm, makJ_pm, pm_j_in_i);

		cp[0] = pm_j_in_i[3];
		cp[1] = pm_j_in_i[7];
		cp[2] = pm_j_in_i[11];

		// 两个坐标系的z轴的角度差应该为90度
		cp[3] = -PI / 2.0 + std::acos(pm_j_in_i[10]);
	}
	UniversalJoint::~UniversalJoint() = default;
	UniversalJoint::UniversalJoint(const std::string &name, Marker* makI, Marker* makJ) : Joint(name, makI, makJ), imp_(new Imp){
		const static double loc_cst[6][4]{
			1,0,0,0,
			0,1,0,0,
			0,0,1,0,
			0,0,0,0,
			0,0,0,0,
			0,0,0,0,
		};
		s_mc(6, dim(), *loc_cst, imp_->loc_cm_I);
	}
	ARIS_DEFINE_BIG_FOUR_CPP(UniversalJoint);

	auto SphericalJoint::locCmI() const noexcept->const double*
	{
		static const double loc_cm_I[18]
		{
			1,0,0,
			0,1,0,
			0,0,1,
			0,0,0,
			0,0,0,
			0,0,0,
		};
		return loc_cm_I;
	}
	auto SphericalJoint::cptCpFromPm(double *cp, const double *makI_pm, const double *makJ_pm)const noexcept->void
	{
		/////////////////////////以下是pa的计算方法///////////////////////////
		double pp_j[3]{ makJ_pm[3], makJ_pm[7], makJ_pm[11], };
		s_inv_pp2pp(makI_pm, pp_j, cp);
		/////////////////////////以上是pa的计算方法///////////////////////////
	}
	auto SphericalJoint::cptGlbDmFromPm(double *dm, const double *makI_pm, const double *makJ_pm)const noexcept->void
	{
		double pm[16];
		s_inv_pm(makI_pm, pm);
		s_tmf(pm, dm);
	}
	SphericalJoint::SphericalJoint(const std::string &name, Marker* makI, Marker* makJ) : Joint(name, makI, makJ) {}

	auto SingleComponentForce::cptGlbFs(double *fsI, double *fsJ)const noexcept->void
	{
		s_tf(*makI()->prtPm(), fce_value_, fsJ);
		s_tf(*makI()->fatherPart().pm(), fsJ, fsI);
		s_vi(6, fsI, fsJ);
	}
	SingleComponentForce::SingleComponentForce(const std::string &name, Marker* makI, Marker* makJ, Size componentID) : Force(name, makI, makJ), component_axis_(componentID) {}

	ARIS_REGISTRATION{
		aris::core::class_<Interaction>("Interaction")
			.inherit<aris::dynamic::DynEle>()
			.prop("prt_m", &Interaction::setPrtNameM, &Interaction::prtNameM)
			.prop("prt_n", &Interaction::setPrtNameN, &Interaction::prtNameN)
			.prop("mak_i", &Interaction::setMakNameI, &Interaction::makNameI)
			.prop("mak_j", &Interaction::setMakNameJ, &Interaction::makNameJ)
			;

		auto setCf = [](Constraint* c, aris::core::Matrix cf_mat)->void {c->setCf(cf_mat.data()); };
		auto getCf = [](Constraint* c)->aris::core::Matrix {	return aris::core::Matrix(1, c->dim(), c->cf()); };
		aris::core::class_<Constraint>("Constraint")
			.inherit<aris::dynamic::Interaction>()
			.prop("cf", &setCf, &getCf)
			;

		auto setMotionFrc = [](Motion* c, aris::core::Matrix mat)->void {c->setFrcCoe(mat.data()); };
		auto getMotionFrc = [](Motion* c)->aris::core::Matrix {	return aris::core::Matrix(1, 3, c->frcCoe()); };
		aris::core::class_<Motion>("Motion")
			.inherit<aris::dynamic::Constraint>()
			.prop("component", &Motion::setAxis, &Motion::axis)
			.prop("mp_offset", &Motion::setMpOffset, &Motion::mpOffset)
			.prop("mp_factor", &Motion::setMpFactor, &Motion::mpFactor)
			.prop("mp", &Motion::setMp, &Motion::mp)
			.prop("mv", &Motion::setMv, &Motion::mv)
			.prop("ma", &Motion::setMa, &Motion::ma)
			.prop("frc_coe", &setMotionFrc, &getMotionFrc)
			;

		aris::core::class_<GeneralMotionBase>("GeneralMotionBase")
			.inherit<aris::dynamic::Constraint>()
			;

		aris::core::class_<GeneralMotion>("GeneralMotion")
			.inherit<aris::dynamic::GeneralMotionBase>()
			;

		aris::core::class_<Joint>("Joint")
			.inherit<aris::dynamic::Constraint>()
			;

		aris::core::class_<RevoluteJoint>("RevoluteJoint")
			.inherit<aris::dynamic::Joint>()
			;

		aris::core::class_<PrismaticJoint>("PrismaticJoint")
			.inherit<aris::dynamic::Joint>()
			;

		aris::core::class_<UniversalJoint>("UniversalJoint")
			.inherit<aris::dynamic::Joint>()
			;

		aris::core::class_<SphericalJoint>("SphericalJoint")
			.inherit<aris::dynamic::Joint>()
			;

		aris::core::class_<Force>("Force")
			.inherit<aris::dynamic::Interaction>()
			;

		aris::core::class_<GeneralForce>("GeneralForce")
			.inherit<aris::dynamic::Force>()
			;

		aris::core::class_<SingleComponentForce>("SingleComponentForce")
			.inherit<aris::dynamic::Force>()
			.prop("component", &SingleComponentForce::setComponentAxis, &SingleComponentForce::componentAxis)
			;

	}
}
