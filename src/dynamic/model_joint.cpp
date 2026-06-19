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

#include "aris/dynamic/model_joint.hpp"

namespace aris::dynamic{
	auto Joint::cptCpFromPm(double* cp, const double* makI_pm, const double* makJ_pm)const noexcept->void {
		double pm_j2i[16], ps_j2i[6];
		s_inv_pm_dot_pm(makI_pm, makJ_pm, pm_j2i);
		s_pm2ps(pm_j2i, ps_j2i);
		s_mm(dim(), 1, 6, locCmI(), ColMajor{ dim() }, ps_j2i, 1, cp, 1);
	}
	auto Joint::cptCvFromV(double* cv, const double *pmI, const double *pmJ, const double* vsI, const double* vsJ)const noexcept->void{
		std::fill_n(cv, dim(), 0.0);
	}; // 实际全置零
	auto Joint::cptCvDiffFromV(double* cv, const double *pmI, const double *pmJ, const double* vsI, const double* vsJ)const noexcept->void{
		// 获取当前状态下所产生的cv //
		double dv[6], dv_in_I[6];
		s_vc(6, vsJ, dv);
		s_vs(6, vsI, dv);
		s_inv_tv(pmI, dv, dv_in_I);
		s_mm(dim(), 1, 6, locCmI(), ColMajor{ dim() }, dv_in_I, 1, cv, 1);
	}; 
	auto Joint::cptCaFromA(double* ca, const double *pmI, const double *pmJ, const double* vsI, const double* vsJ)const noexcept->void{
		double vi_cross_vj[6], tem[6];
		s_cv(vsI, vsJ, vi_cross_vj);
		s_inv_tv(pmI, vi_cross_vj, tem);
		s_mmi(dim(), 1, 6, locCmI(), ColMajor{ dim() }, tem, 1, ca, 1);
	}; // 实际全置零


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

	auto ScrewJoint::locCmI() const noexcept->const double* {
		return loc_cm_i_;
	}
	auto ScrewJoint::cptCpFromPm(double* cp, const double* makI_pm, const double* makJ_pm)const noexcept->void {
		double pm_j_in_i[16];
		s_inv_pm_dot_pm(makI_pm, makJ_pm, pm_j_in_i);
		
		auto pitch_compensation = pm_j_in_i[11];
		if (pitch()) {
			pitch_compensation -= pitch_ / 2 / PI * std::atan2(pm_j_in_i[4] - pm_j_in_i[1], pm_j_in_i[0] + pm_j_in_i[5]);
			pitch_compensation = std::fmod(pitch_compensation, pitch_);
			if (pitch_compensation > pitch_ / 2)
				pitch_compensation -= pitch_;
			else if(pitch_compensation < -pitch_ / 2)
				pitch_compensation += pitch_;
		}



		cp[0] = pm_j_in_i[3];
		cp[1] = pm_j_in_i[7];
		cp[2] = pitch_compensation;

		// 这里用i的z轴叉乘j的z轴，在i坐标系下，因此叉乘出来有如下结果:
		cp[3] = -pm_j_in_i[6];
		cp[4] = pm_j_in_i[2];
	}
	auto ScrewJoint::cptGlbDmFromPm(double* dm, const double* makI_pm, const double* makJ_pm)const noexcept->void {
		double pm[16], tm[36];
		s_inv_pm(makI_pm, pm);
		s_tmf(pm, tm);
		double sm[36]{
			1,0,0,0,0,0,
			0,1,0,0,0,0,
			0,0,1,0,0,0,
			0,0,0,1,0,0,
			0,0,0,0,1,0,
			0,0,pitch_,0,0,1
		};
		s_mm(6, 6, 6, sm, tm, dm);
	}
	auto ScrewJoint::pitch()const noexcept->double {
		return pitch_;
	}
	auto ScrewJoint::setPitch(double pitch)noexcept->void {
		pitch_ = pitch;
	}
	ScrewJoint::ScrewJoint(const std::string& name, Marker* makI, Marker* makJ, double pitch) : Joint(name, makI, makJ) {
		double loc_cm_I[30]{
			1,0,0,0,0,
			0,1,0,0,0,
			0,0,1,0,0,
			0,0,0,1,0,
			0,0,0,0,1,
			0,0,-pitch/2/PI,0,0
		};

		s_vc(30, loc_cm_I, loc_cm_i_);
		pitch_ = pitch;
	}


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
	auto UniversalJoint::cptCvDiffFromV(double* cv, const double *pmI, const double *pmJ, const double* vsI, const double* vsJ)const noexcept->void {
		
		/// local cm 计算如下 //
		/// double axis_jz_i[3];
		/// s_inv_pm_dot_v3(pmI, pmJ + 2, 4, axis_jz_i, 1);
		/// double x2 = axis_jz_i[0];
		/// double y2 = axis_jz_i[1];
		/// double norm = std::sqrt(x2*x2 + y2 * y2);
		///
		/// double loc_cm_I[24]{
		/// 	1,0,0,0,
		/// 	0,1,0,0,
		/// 	0,0,1,0,
		/// 	0,0,0,-y2 / norm,
		/// 	0,0,0,x2 / norm,
		/// 	0,0,0,0
		/// };

		// 获取当前状态下所产生的cv //
		double dv[6], dv_in_I[6];
		s_vc(6, vsJ, dv);
		s_vs(6, vsI, dv);
		s_inv_tv(pmI, dv, dv_in_I);

		//s_mm(dim(), 1, 6, loc_cm_I, ColMajor{ dim() }, dv_in_I, 1, cv, 1);
		
		// 上述乘法优化后如下 //
		double x2 = pmI[0] * pmJ[2] + pmI[4] * pmJ[6] + pmI[8] * pmJ[10];
		double y2 = pmI[1] * pmJ[2] + pmI[5] * pmJ[6] + pmI[9] * pmJ[10];
		double norm = std::sqrt(x2 * x2 + y2 * y2);
		x2 /= norm;
		y2 /= -norm;

		cv[0] = dv_in_I[0];
		cv[1] = dv_in_I[1];
		cv[2] = dv_in_I[2];
		cv[3] = y2 * dv_in_I[3] + x2 * dv_in_I[4];
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
	auto UniversalJoint::cptCaFromA(double* ca, const double *pmI, const double *pmJ, const double* vsI, const double* vsJ)const noexcept->void {
		//Joint::cptCaFromA(ca, pmI, pmJ, vsI, vsJ);
		// method in Joint //
		double vi_cross_vj[6], tem[6];
		s_cv(vsI, vsJ, vi_cross_vj);
		s_inv_tv(pmI, vi_cross_vj, tem);
		//s_mmi(dim(), 1, 6, locCmI(), ColMajor{ dim() }, tem, 1, ca, 1);
		
		// 上述乘法优化后如下 //
		double x2 = pmI[0] * pmJ[2] + pmI[4] * pmJ[6] + pmI[8] * pmJ[10];
		double y2 = pmI[1] * pmJ[2] + pmI[5] * pmJ[6] + pmI[9] * pmJ[10];
		double norm = std::sqrt(x2 * x2 + y2 * y2);
		x2 /= norm;
		y2 /= -norm;

		ca[0] = -tem[0];
		ca[1] = -tem[1];
		ca[2] = -tem[2];
		ca[3] = -y2 * tem[3] - x2 * tem[4];


		// update makI的z轴 和 makJ的z轴
		// const double axis_i_i[3]{ 0,0,1 };
		double axis_j_i[3];
		double wm_in_i[3], wn_in_i[3];

		double axis_j_g[3]{pmJ[2], pmJ[6], pmJ[10]};
		s_inv_pm_dot_v3(pmI, axis_j_g, axis_j_i);

		// compute c_dot //
		s_inv_pm_dot_v3(pmI, vsI + 3, wm_in_i);
		s_inv_pm_dot_v3(pmI, vsJ + 3, wn_in_i);

		// double iwm = s_vv(3, axis_i_i, wm_in_i);
		// double jwm = s_vv(3, axis_j_i, wm_in_i);
		// double iwn = s_vv(3, axis_i_i, wn_in_i);
		// double jwn = s_vv(3, axis_j_i, wn_in_i);

		double iwm = wm_in_i[2]; // Assuming axis_i_i is [0,0,1]
		double jwm = s_vv(3, axis_j_i, wm_in_i);
		double iwn = wn_in_i[2]; // Assuming axis_i_i is [0,0,1]
		double jwn = s_vv(3, axis_j_i, wn_in_i);

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

	auto SphericalJoint::locCmI() const noexcept->const double*{
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
	auto SphericalJoint::cptCpFromPm(double *cp, const double *makI_pm, const double *makJ_pm)const noexcept->void{
		/////////////////////////以下是pa的计算方法///////////////////////////
		double pp_j[3]{ makJ_pm[3], makJ_pm[7], makJ_pm[11], };
		s_inv_pp2pp(makI_pm, pp_j, cp);
		/////////////////////////以上是pa的计算方法///////////////////////////
	}
	auto SphericalJoint::cptGlbDmFromPm(double *dm, const double *makI_pm, const double *makJ_pm)const noexcept->void{
		double pm[16];
		s_inv_pm(makI_pm, pm);
		s_tmf(pm, dm);
	}
	SphericalJoint::SphericalJoint(const std::string &name, Marker* makI, Marker* makJ) : Joint(name, makI, makJ) {}

	ARIS_REGISTRATION{
		aris::core::class_<Joint>("Joint")
			.inherit<aris::dynamic::Constraint>()
			;

		aris::core::class_<RevoluteJoint>("RevoluteJoint")
			.inherit<aris::dynamic::Joint>()
			;

		aris::core::class_<PrismaticJoint>("PrismaticJoint")
			.inherit<aris::dynamic::Joint>()
			;

		aris::core::class_<ScrewJoint>("ScrewJoint")
			.inherit<aris::dynamic::Joint>()
			.prop("pitch", &ScrewJoint::setPitch, &ScrewJoint::pitch)
			;

		aris::core::class_<UniversalJoint>("UniversalJoint")
			.inherit<aris::dynamic::Joint>()
			;

		aris::core::class_<SphericalJoint>("SphericalJoint")
			.inherit<aris::dynamic::Joint>()
			;
	}
}
