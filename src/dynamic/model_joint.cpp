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
