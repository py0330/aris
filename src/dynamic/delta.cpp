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
#include <array>

#include "aris/core/reflection.hpp"

#include "aris/dynamic/model.hpp"
#include "aris/dynamic/model_solver.hpp"
#include "aris/dynamic/delta.hpp"

namespace aris::dynamic{

	struct DeltaModel::Imp {
		aris::core::Matrix dh_;
	};
	auto deltaInverse(const double *param, const double *ee_xyza, int which_root, double *input)->int {
		const double &a = param[0];
		const double &b = param[1];
		const double &c = param[2];
		const double &d = param[3];
		const double &e = param[4];

		// 此处将点转回到x z平面，因此与构造delta的地方不太一样 //
		const double theta[] = { 0.0, -PI * 2 / 3, PI * 2 / 3 };

		for (int i = 0; i < 3; ++i) {
			auto x = ee_xyza[0] * std::cos(theta[i]) - ee_xyza[1] * std::sin(theta[i]);
			auto y = ee_xyza[0] * std::sin(theta[i]) + ee_xyza[1] * std::cos(theta[i]);
			auto z = ee_xyza[2];

			if (std::abs(d) < std::abs(y)) return -1;

			auto d1 = std::sqrt(d * d - y * y);
			auto k = x + e - a;
			auto l = sqrt(k * k + z * z);

			if (b + l < d1 || b + d1 < l || d1 + l < b)
				return -2;

			input[i] =
				((0x01 << i) & which_root) ?
				-std::atan2(z, k) + std::acos((b * b + l * l - d1 * d1) / b / l / 2.0) :
				-std::atan2(z, k) - std::acos((b * b + l * l - d1 * d1) / b / l / 2.0);
		}

		input[3] = ee_xyza[3];

		return 0;
	}

	class DeltaInverseKinematicSolver :public aris::dynamic::InverseKinematicSolver {
	public:
		auto virtual kinPos()->int override {
			double input[4], output[6];
			
			model()->getOutputPos(output);
			if (auto ret = deltaInverse(dynamic_cast<DeltaModel*>(model())->imp_->dh_.data(), output, 0, input))
				return ret;

			// ee //
			double pe[6]{ output[0],output[1],output[2],output[3], 0.0, 0.0 }, pp[3];
			model()->generalMotionPool()[0].makI()->setPe(*model()->generalMotionPool()[0].makJ(), pe, "321");

			// up //
			s_fill(1, 3, 0.0, pe);
			pe[3] = -pe[3];
			model()->jointPool()[15].makJ()->setPe(*model()->jointPool()[15].makI(), pe, "321");

			// link1 //
			s_fill(1, 6, 0.0, pe);
			pe[5] = input[0];
			model()->jointPool()[0].makI()->setPe(*model()->jointPool()[0].makJ(), pe, "123");

			s_fill(1, 6, 0.0, pe);
			pe[5] = input[1];
			model()->jointPool()[5].makI()->setPe(*model()->jointPool()[5].makJ(), pe, "123");

			s_fill(1, 6, 0.0, pe);
			pe[5] = input[2];
			model()->jointPool()[10].makI()->setPe(*model()->jointPool()[10].makJ(), pe, "123");

			// link2&3 //
			s_fill(1, 6, 0.0, pe);
			pe[5] = PI / 2;
			model()->jointPool()[3].makI()->getPp(*model()->jointPool()[1].makJ(), pp);
			s_sov_ab(pp, pe + 3, "312");
			model()->jointPool()[1].makI()->setPe(*model()->jointPool()[1].makJ(), pe, "312");

			model()->jointPool()[4].makI()->getPp(*model()->jointPool()[2].makJ(), pp);
			s_sov_ab(pp, pe + 3, "312");
			model()->jointPool()[2].makI()->setPe(*model()->jointPool()[2].makJ(), pe, "312");

			model()->jointPool()[8].makI()->getPp(*model()->jointPool()[6].makJ(), pp);
			s_sov_ab(pp, pe + 3, "312");
			model()->jointPool()[6].makI()->setPe(*model()->jointPool()[6].makJ(), pe, "312");

			model()->jointPool()[9].makI()->getPp(*model()->jointPool()[7].makJ(), pp);
			s_sov_ab(pp, pe + 3, "312");
			model()->jointPool()[7].makI()->setPe(*model()->jointPool()[7].makJ(), pe, "312");

			model()->jointPool()[13].makI()->getPp(*model()->jointPool()[11].makJ(), pp);
			s_sov_ab(pp, pe + 3, "312");
			model()->jointPool()[11].makI()->setPe(*model()->jointPool()[11].makJ(), pe, "312");

			model()->jointPool()[14].makI()->getPp(*model()->jointPool()[12].makJ(), pp);
			s_sov_ab(pp, pe + 3, "312");
			model()->jointPool()[12].makI()->setPe(*model()->jointPool()[12].makJ(), pe, "312");

			for (auto &m : model()->motionPool()) m.updP();
			return 0;
		}

		DeltaInverseKinematicSolver() = default;
	};

	auto DeltaModel::dh()->aris::core::Matrix& { return imp_->dh_; }
	DeltaModel::~DeltaModel() = default;
	DeltaModel::DeltaModel(const DeltaParam &param):imp_(new Imp) {
		imp_->dh_ = aris::core::Matrix({param.a, param.b, param.c, param.d, param.e});

		// 设置重力 //
		const double gravity[6]{ 0.0,0.0,-9.8,0.0,0.0,0.0 };
		this->environment().setGravity(gravity);

		// add part //
		const double default_iv[10]{ 1,0,0,0,0,0,0,0,0,0 };
		auto &p11 = this->partPool().add<Part>("L11", param.iv_vec.size() == 4 ? param.iv_vec[0].data() : default_iv);
		auto &p12 = this->partPool().add<Part>("L12", param.iv_vec.size() == 4 ? param.iv_vec[1].data() : default_iv);
		auto &p13 = this->partPool().add<Part>("L13", param.iv_vec.size() == 4 ? param.iv_vec[2].data() : default_iv);
		auto &p21 = this->partPool().add<Part>("L21", param.iv_vec.size() == 4 ? param.iv_vec[0].data() : default_iv);
		auto &p22 = this->partPool().add<Part>("L22", param.iv_vec.size() == 4 ? param.iv_vec[1].data() : default_iv);
		auto &p23 = this->partPool().add<Part>("L23", param.iv_vec.size() == 4 ? param.iv_vec[2].data() : default_iv);
		auto &p31 = this->partPool().add<Part>("L31", param.iv_vec.size() == 4 ? param.iv_vec[0].data() : default_iv);
		auto &p32 = this->partPool().add<Part>("L32", param.iv_vec.size() == 4 ? param.iv_vec[1].data() : default_iv);
		auto &p33 = this->partPool().add<Part>("L33", param.iv_vec.size() == 4 ? param.iv_vec[2].data() : default_iv);
		auto &pup = this->partPool().add<Part>("UP", param.iv_vec.size() == 4 ? param.iv_vec[3].data() : default_iv);
		auto &pee = this->partPool().add<Part>("EE", param.iv_vec.size() == 4 ? param.iv_vec[3].data() : default_iv);

		// add geometry //
		ground().geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\aris\\aris-1.5.0\\resource\\test_dynamic\\delta\\base.xmt_txt");
		pup.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\aris\\aris-1.5.0\\resource\\test_dynamic\\delta\\up.xmt_txt");
		pee.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\aris\\aris-1.5.0\\resource\\test_dynamic\\delta\\ee.xmt_txt");
		double geo_p13_prt_pm[16]{ 1,0,0,0,0,1,0,-param.c,0,0,1,0,0,0,0,1 };
		double geo_rot_pm[16], geo_local_pm[16];
		p11.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\aris\\aris-1.5.0\\resource\\test_dynamic\\delta\\p1.xmt_txt");
		p12.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\aris\\aris-1.5.0\\resource\\test_dynamic\\delta\\p2.xmt_txt");
		p13.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\aris\\aris-1.5.0\\resource\\test_dynamic\\delta\\p2.xmt_txt", geo_p13_prt_pm);
		s_eye(4, geo_rot_pm);
		s_rmz(PI * 2 / 3, geo_rot_pm, 4);
		s_pm_dot_pm(geo_rot_pm, geo_p13_prt_pm, geo_local_pm);
		p21.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\aris\\aris-1.5.0\\resource\\test_dynamic\\delta\\p1.xmt_txt", geo_rot_pm);
		p22.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\aris\\aris-1.5.0\\resource\\test_dynamic\\delta\\p2.xmt_txt", geo_rot_pm);
		p23.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\aris\\aris-1.5.0\\resource\\test_dynamic\\delta\\p2.xmt_txt", geo_local_pm);
		s_eye(4, geo_rot_pm);
		s_rmz(-PI * 2 / 3, geo_rot_pm, 4);
		s_pm_dot_pm(geo_rot_pm, geo_p13_prt_pm, geo_local_pm);
		p31.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\aris\\aris-1.5.0\\resource\\test_dynamic\\delta\\p1.xmt_txt", geo_rot_pm);
		p32.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\aris\\aris-1.5.0\\resource\\test_dynamic\\delta\\p2.xmt_txt", geo_rot_pm);
		p33.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\aris\\aris-1.5.0\\resource\\test_dynamic\\delta\\p2.xmt_txt", geo_local_pm);

		// add joint //
		double r11_pos[3]{ param.a          ,          0.0, 0.0 };
		double s12_pos[3]{ param.a + param.b,  param.c / 2, 0.0 };
		double s13_pos[3]{ param.a + param.b, -param.c / 2, 0.0 };
		double s14_pos[3]{ param.e          ,  param.c / 2, -std::sqrt(param.d*param.d - (param.a + param.b - param.e)*(param.a + param.b - param.e)) };
		double s15_pos[3]{ param.e          , -param.c / 2, -std::sqrt(param.d*param.d - (param.a + param.b - param.e)*(param.a + param.b - param.e)) };
		double r11_axis[6]{ 0.0, 1.0, 0.0 };
		double u12_first_axis[3]{ 0.0, 1.0, 0.0 };
		double u12_second_axis[3]{ s14_pos[2] - s12_pos[2], 0.0, -s14_pos[0] + s12_pos[0] };
		double r21_pos[3], s22_pos[3], s23_pos[3], s24_pos[3], s25_pos[3], r21_axis[3], u22_first_axis[3], u22_second_axis[3];
		double r31_pos[3], s32_pos[3], s33_pos[3], s34_pos[3], s35_pos[3], r31_axis[3], u32_first_axis[3], u32_second_axis[3];

		double rm2[9], rm3[9];
		s_rmz(PI * 2 / 3, rm2);
		s_rmz(-PI * 2 / 3, rm3);

		s_mm(3, 1, 3, rm2, r11_pos, r21_pos);
		s_mm(3, 1, 3, rm2, s12_pos, s22_pos);
		s_mm(3, 1, 3, rm2, s13_pos, s23_pos);
		s_mm(3, 1, 3, rm2, s14_pos, s24_pos);
		s_mm(3, 1, 3, rm2, s15_pos, s25_pos);
		s_mm(3, 1, 3, rm2, r11_axis, r21_axis);
		s_mm(3, 1, 3, rm2, u12_first_axis, u22_first_axis);
		s_mm(3, 1, 3, rm2, u12_second_axis, u22_second_axis);

		s_mm(3, 1, 3, rm3, r11_pos, r31_pos);
		s_mm(3, 1, 3, rm3, s12_pos, s32_pos);
		s_mm(3, 1, 3, rm3, s13_pos, s33_pos);
		s_mm(3, 1, 3, rm3, s14_pos, s34_pos);
		s_mm(3, 1, 3, rm3, s15_pos, s35_pos);
		s_mm(3, 1, 3, rm3, r11_axis, r31_axis);
		s_mm(3, 1, 3, rm3, u12_first_axis, u32_first_axis);
		s_mm(3, 1, 3, rm3, u12_second_axis, u32_second_axis);

		auto &r11 = this->addRevoluteJoint(p11, this->ground(), r11_pos, r11_axis);
		auto &u12 = this->addUniversalJoint(p12, p11, s12_pos, u12_second_axis, u12_first_axis);
		auto &u13 = this->addUniversalJoint(p13, p11, s13_pos, u12_second_axis, u12_first_axis);
		auto &s14 = this->addSphericalJoint(pup, p12, s14_pos);
		auto &s15 = this->addSphericalJoint(pup, p13, s15_pos);

		auto &r21 = this->addRevoluteJoint(p21, this->ground(), r21_pos, r21_axis);
		auto &u22 = this->addUniversalJoint(p22, p21, s22_pos, u22_second_axis, u22_first_axis);
		auto &u23 = this->addUniversalJoint(p23, p21, s23_pos, u22_second_axis, u22_first_axis);
		auto &s24 = this->addSphericalJoint(pup, p22, s24_pos);
		auto &s25 = this->addSphericalJoint(pup, p23, s25_pos);

		auto &r31 = this->addRevoluteJoint(p31, this->ground(), r31_pos, r31_axis);
		auto &u32 = this->addUniversalJoint(p32, p31, s32_pos, u32_second_axis, u32_first_axis);
		auto &u33 = this->addUniversalJoint(p33, p31, s33_pos, u32_second_axis, u32_first_axis);
		auto &s34 = this->addSphericalJoint(pup, p32, s34_pos);
		auto &s35 = this->addSphericalJoint(pup, p33, s35_pos);

		double re_pos[3]{ 0,0,-std::sqrt(param.d*param.d - (param.a + param.b - param.e)*(param.a + param.b - param.e)) };
		double re_axis[3]{ 0,0,1 };
		auto &re = this->addRevoluteJoint(pee, pup, re_pos, re_axis);
		// add actuation //
		auto &m1 = this->addMotion(r11);
		auto &m2 = this->addMotion(r21);
		auto &m3 = this->addMotion(r31);
		auto &m4 = this->addMotion(re);

		const double default_mot_frc[3]{ 0.0, 0.0, 0.0 };
		m1.setFrcCoe(param.mot_frc_vec.size() == 6 ? param.mot_frc_vec[0].data() : default_mot_frc);
		m2.setFrcCoe(param.mot_frc_vec.size() == 6 ? param.mot_frc_vec[1].data() : default_mot_frc);
		m3.setFrcCoe(param.mot_frc_vec.size() == 6 ? param.mot_frc_vec[2].data() : default_mot_frc);
		m4.setFrcCoe(param.mot_frc_vec.size() == 6 ? param.mot_frc_vec[3].data() : default_mot_frc);

		// add ee general motion //
		double ee_i_pm[16]{ 1,0,0,0,0,1,0,0,0,0,1,-std::sqrt(param.d*param.d - (param.a + param.b - param.e)*(param.a + param.b - param.e)),0,0,0,1 };
		double ee_j_pm[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };

		auto &makI = pee.addMarker("tool0", ee_i_pm);
		auto &makJ = this->ground().addMarker("wobj0", ee_j_pm);
		this->variablePool().add<aris::dynamic::MatrixVariable>("tool0_axis_home", aris::core::Matrix(1, 6, 0.0));
		auto &ee = this->generalMotionPool().add<aris::dynamic::XyztMotion>("tool", &makI, &makJ, false);

		// change robot pose wrt ground //
		double robot_pm[16];
		s_pe2pm(param.base2ref_pe, robot_pm, param.base2ref_pe_type.empty() ? "321" : param.base2ref_pe_type.c_str());

		p11.setPm(s_pm_dot_pm(robot_pm, *p11.pm()));
		p12.setPm(s_pm_dot_pm(robot_pm, *p12.pm()));
		p13.setPm(s_pm_dot_pm(robot_pm, *p13.pm()));
		p21.setPm(s_pm_dot_pm(robot_pm, *p21.pm()));
		p22.setPm(s_pm_dot_pm(robot_pm, *p22.pm()));
		p23.setPm(s_pm_dot_pm(robot_pm, *p23.pm()));
		p31.setPm(s_pm_dot_pm(robot_pm, *p31.pm()));
		p32.setPm(s_pm_dot_pm(robot_pm, *p32.pm()));
		p33.setPm(s_pm_dot_pm(robot_pm, *p33.pm()));
		pup.setPm(s_pm_dot_pm(robot_pm, *pup.pm()));
		pee.setPm(s_pm_dot_pm(robot_pm, *pee.pm()));
		r11.makJ()->setPrtPm(s_pm_dot_pm(robot_pm, *r11.makJ()->prtPm()));
		r21.makJ()->setPrtPm(s_pm_dot_pm(robot_pm, *r21.makJ()->prtPm()));
		r31.makJ()->setPrtPm(s_pm_dot_pm(robot_pm, *r31.makJ()->prtPm()));
		ee.makJ()->setPrtPm(s_pm_dot_pm(robot_pm, *ee.makJ()->prtPm()));

		// add tools and wobj //
		for (int i = 1; i < 17; ++i) {
			pee.addMarker("tool" + std::to_string(i), ee_i_pm);
		}
		for (int i = 1; i < 33; ++i) this->ground().markerPool().add<aris::dynamic::Marker>("wobj" + std::to_string(i), ee_j_pm);

		// add solver
		auto &inverse_kinematic = this->solverPool().add<aris::dynamic::DeltaInverseKinematicSolver>();
		auto &forward_kinematic = this->solverPool().add<ForwardKinematicSolver>();
		auto &inverse_dynamic = this->solverPool().add<aris::dynamic::InverseDynamicSolver>();
		auto &forward_dynamic = this->solverPool().add<aris::dynamic::ForwardDynamicSolver>();

		// make topology correct // 
		for (auto &m : this->motionPool())m.activate(true);
		for (auto &gm : this->generalMotionPool())gm.activate(false);
		for (auto &f : this->forcePool())f.activate(false);

		this->init();
	}
	DeltaModel::DeltaModel(DeltaModel&&)=default;
	DeltaModel& DeltaModel::operator=(DeltaModel&&)=default;
	
	ARIS_REGISTRATION{
		aris::core::class_<DeltaInverseKinematicSolver>("DeltaInverseKinematicSolver")
			.inherit<InverseKinematicSolver>()
			;
		
		aris::core::class_<DeltaModel>("DeltaModel")
			.inherit<aris::dynamic::Model>()
			.prop("dh", &DeltaModel::dh)
			;
	}
}
