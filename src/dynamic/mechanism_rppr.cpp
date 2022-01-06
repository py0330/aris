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
#include "aris/dynamic/mechanism_rppr.hpp"

namespace aris::dynamic {

	auto rpprInverse(const double *param, const double *ee_xyza, int which_root, double *input)->int {
		const double &a = param[0];
		const double &b = param[1];

		input[0] = std::atan2(ee_xyza[0], ee_xyza[1]);
		input[1] = ee_xyza[2] - a;
		input[2] = std::sqrt(ee_xyza[1] * ee_xyza[1] + ee_xyza[0] * ee_xyza[0]) - b;
		input[3] = ee_xyza[3] - input[0];

		while (input[3] > aris::PI) input[3] -= 2 * aris::PI;
		while (input[3] < -aris::PI) input[3] += 2 * aris::PI;

		return 0;
	}
	class RpprInverseKinematicSolver :public aris::dynamic::InverseKinematicSolver {
	public:
		auto virtual kinPos()->int override {
			double input[4], output[4];

			auto dh = dynamic_cast<aris::dynamic::MatrixVariable*>(model()->findVariable("dh"))->data().data();

			model()->getOutputPos(output);
			if (auto ret = rpprInverse(dh, output, 0, input))
				return ret;

			// link1~4 //
			double pe[6]{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
			pe[5] = input[0];
			model()->jointPool()[0].makI()->setPe(*model()->jointPool()[0].makJ(), pe, "123");

			pe[5] = 0.0;
			pe[2] = input[1];
			model()->jointPool()[1].makI()->setPe(*model()->jointPool()[1].makJ(), pe, "123");

			pe[2] = input[2];
			model()->jointPool()[2].makI()->setPe(*model()->jointPool()[2].makJ(), pe, "123");

			pe[2] = 0.0;
			pe[5] = input[3];
			model()->jointPool()[3].makI()->setPe(*model()->jointPool()[3].makJ(), pe, "123");

			for (auto &m : model()->motionPool()) m.updP();
			return 0;
		}

		RpprInverseKinematicSolver() = default;
	};
	auto createModelRppr(const RpprParam &param)->std::unique_ptr<aris::dynamic::Model> {
		std::unique_ptr<aris::dynamic::Model> model(new aris::dynamic::Model);

		model->variablePool().add<aris::dynamic::MatrixVariable>("dh", aris::core::Matrix({ param.a, param.b }));

		// 设置重力 //
		const double gravity[6]{ 0.0,0.0,-9.8,0.0,0.0,0.0 };
		model->environment().setGravity(gravity);

		// add part //
		const double default_iv[10]{ 1,0,0,0,0,0,0,0,0,0 };
		auto &p1 = model->partPool().add<Part>("L1", param.iv_vec.size() == 4 ? param.iv_vec[0].data() : default_iv);
		auto &p2 = model->partPool().add<Part>("L2", param.iv_vec.size() == 4 ? param.iv_vec[1].data() : default_iv);
		auto &p3 = model->partPool().add<Part>("L3", param.iv_vec.size() == 4 ? param.iv_vec[2].data() : default_iv);
		auto &p4 = model->partPool().add<Part>("L4", param.iv_vec.size() == 4 ? param.iv_vec[0].data() : default_iv);

		// add geometry //

		// add joint //
		const double joint1_position[3]{ 0 , 0 , 0 };
		const double joint1_axis[3]{ 0 , 0 , 1 };
		const double joint2_position[3]{ 0 , 0 , 0 };
		const double joint2_axis[3]{ 0 , 0 , 1 };
		const double joint3_position[3]{ 0 , 0 , param.a };
		const double joint3_axis[3]{ 1 , 0 , 0 };
		const double joint4_position[3]{ param.b , 0 , param.a };
		const double joint4_axis[3]{ 0 , 0 , 1 };

		auto &r1 = model->addRevoluteJoint(p1, model->ground(), joint1_position, joint1_axis);
		auto &r2 = model->addPrismaticJoint(p2, p1, joint2_position, joint2_axis);
		auto &j3 = model->addPrismaticJoint(p3, p2, joint3_position, joint3_axis);
		auto &r4 = model->addRevoluteJoint(p4, p3, joint4_position, joint4_axis);

		// add actuation //
		auto &m1 = model->addMotion(r1);
		auto &m2 = model->addMotion(r2);
		auto &m3 = model->addMotion(j3);
		auto &m4 = model->addMotion(r4);

		const double default_mot_frc[3]{ 0.0, 0.0, 0.0 };
		m1.setFrcCoe(param.mot_frc_vec.size() == 6 ? param.mot_frc_vec[0].data() : default_mot_frc);
		m2.setFrcCoe(param.mot_frc_vec.size() == 6 ? param.mot_frc_vec[1].data() : default_mot_frc);
		m3.setFrcCoe(param.mot_frc_vec.size() == 6 ? param.mot_frc_vec[2].data() : default_mot_frc);
		m4.setFrcCoe(param.mot_frc_vec.size() == 6 ? param.mot_frc_vec[3].data() : default_mot_frc);

		// add ee general motion //
		double ee_i_pm[16]{ 1,0,0,param.b,0,1,0,0,0,0,1,param.a,0,0,0,1 };
		double ee_j_pm[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };

		auto &makI = p4.addMarker("tool0", ee_i_pm);
		auto &makJ = model->ground().addMarker("wobj0", ee_j_pm);
		model->variablePool().add<aris::dynamic::MatrixVariable>("tool0_axis_home", aris::core::Matrix(1, 6, 0.0));
		auto &ee = model->generalMotionPool().add<aris::dynamic::XyztMotion>("tool", &makI, &makJ, false);

		// change robot pose wrt ground //
		double robot_pm[16];
		s_pe2pm(param.base2ref_pe, robot_pm, param.base2ref_pe_type.empty() ? "321" : param.base2ref_pe_type.c_str());

		p1.setPm(s_pm_dot_pm(robot_pm, *p1.pm()));
		p2.setPm(s_pm_dot_pm(robot_pm, *p2.pm()));
		p3.setPm(s_pm_dot_pm(robot_pm, *p3.pm()));
		p4.setPm(s_pm_dot_pm(robot_pm, *p4.pm()));
		r1.makJ()->setPrtPm(s_pm_dot_pm(robot_pm, *r1.makJ()->prtPm()));
		ee.makJ()->setPrtPm(s_pm_dot_pm(robot_pm, *ee.makJ()->prtPm()));

		// add tools and wobj //
		for (int i = 1; i < 17; ++i) {
			p4.addMarker("tool" + std::to_string(i), ee_i_pm);
		}
		for (int i = 1; i < 33; ++i) model->ground().markerPool().add<aris::dynamic::Marker>("wobj" + std::to_string(i), ee_j_pm);

		// add solver
		auto &inverse_kinematic = model->solverPool().add<aris::dynamic::RpprInverseKinematicSolver>();
		auto &forward_kinematic = model->solverPool().add<aris::dynamic::ForwardKinematicSolver>();
		auto &inverse_dynamic = model->solverPool().add<aris::dynamic::InverseDynamicSolver>();
		auto &forward_dynamic = model->solverPool().add<aris::dynamic::ForwardDynamicSolver>();

		// make topology correct // 
		for (auto &m : model->motionPool())m.activate(true);
		for (auto &gm : model->generalMotionPool())gm.activate(false);
		for (auto &f : model->forcePool())f.activate(false);

		model->init();
		return model;
	}


	ARIS_REGISTRATION{
		aris::core::class_<RpprInverseKinematicSolver>("RpprInverseKinematicSolver")
			.inherit<InverseKinematicSolver>()
			;
	}
}
