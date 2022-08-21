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
#include "aris/dynamic/scara.hpp"

namespace aris::dynamic{
	// 不考虑 pitch //
	auto scaraInverse(const double *param, const double *ee_xyza, int which_root, double *input)->int {
		const double &a = param[0];
		const double &b = param[1];

		auto c = std::sqrt(ee_xyza[0] * ee_xyza[0] + ee_xyza[1] * ee_xyza[1]);
		if (c<std::abs(a - b) || c>std::abs(a + b)) return -1;

		input[0] = which_root == 0 ? std::atan2(ee_xyza[1], ee_xyza[0]) - std::acos((a*a + c * c - b * b) / (2 * a*c)) : std::atan2(ee_xyza[1], ee_xyza[0]) + std::acos((a*a + c * c - b * b) / (2 * a*c));
		input[1] = which_root == 0 ? -std::acos((a*a + b*b - c*c) / (2*a*b)) + aris::PI / 2 : std::acos((a*a + b*b - c*c) / (2*a*b)) - aris::PI * 3 / 2;
		input[2] = ee_xyza[2];
		input[3] = ee_xyza[3] - input[1] - input[0];

		input[0] = std::fmod(input[0], 2 * PI);
		if (input[0] > PI) input[0] -= 2 * PI;
		if (input[0] < -PI) input[0] += 2 * PI;

		input[1] = std::fmod(input[1], 2 * PI);
		if (input[1] > PI) input[1] -= 2 * PI;
		if (input[1] < -PI) input[1] += 2 * PI;

		input[3] = std::fmod(input[3], 2 * PI);
		if (input[3] > PI) input[3] -= 2 * PI;
		if (input[3] < -PI) input[3] += 2 * PI;

		return 0;
	}
	class ScaraForwardKinematicSolver :public aris::dynamic::ForwardKinematicSolver {
	public:
		auto virtual kinPos()->int override {
			auto dh = dynamic_cast<aris::dynamic::MatrixVariable*>(model()->findVariable("dh"))->data().data();
			
			// link1~4 //
			double pe[6]{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
			pe[5] = model()->motionPool()[0].mpInternal();
			model()->jointPool()[0].makI()->setPe(*model()->jointPool()[0].makJ(), pe, "123");

			pe[5] = model()->motionPool()[1].mpInternal();
			model()->jointPool()[1].makI()->setPe(*model()->jointPool()[1].makJ(), pe, "123");

			pe[5] = 0.0;
			pe[2] = model()->motionPool()[2].mpInternal();
			model()->jointPool()[2].makI()->setPe(*model()->jointPool()[2].makJ(), pe, "123");

			pe[2] = model()->motionPool()[3].mpInternal() / 2 / PI
				* dynamic_cast<ScrewJoint&>(model()->jointPool()[3]).pitch();
			pe[5] = model()->motionPool()[3].mpInternal();
			model()->jointPool()[3].makI()->setPe(*model()->jointPool()[3].makJ(), pe, "123");

			for (auto& m : model()->generalMotionPool()) m.updP();
			return 0;
		}
		ScaraForwardKinematicSolver() = default;
	};
	class ScaraInverseKinematicSolver :public aris::dynamic::InverseKinematicSolver {
	public:
		auto virtual kinPos()->int override {
			auto dh = dynamic_cast<aris::dynamic::MatrixVariable*>(model()->findVariable("dh"))->data().data();
			const int ROOT_NUM = 2;
			const int ROOT_SIZE = 4;
			if (which_root_ == ROOT_NUM) {
				int solution_num = 0;
				double diff_q[ROOT_NUM][ROOT_SIZE];
				double diff_norm[ROOT_NUM];

				for (int i = 0; i < ROOT_NUM; ++i) {
					double output[ROOT_SIZE];
					model()->getOutputPos(output);

					if (scaraInverse(dh, output, i, diff_q[solution_num]) == 0) {
						diff_norm[solution_num] = 0;

						for (auto j: { 0,1,3 }) {
							diff_q[solution_num][j] -= model()->motionPool()[j].mpInternal();

							// 如果是2 的话，忽略轴的范围，选择最近的可达解 //
							while (diff_q[solution_num][j] > PI) diff_q[solution_num][j] -= 2 * PI;
							while (diff_q[solution_num][j] < -PI)diff_q[solution_num][j] += 2 * PI;

							diff_norm[solution_num] += std::abs(diff_q[solution_num][j]);
						}

						// 对3轴抵消4轴的pitch影响 //
						diff_q[solution_num][2] -= model()->motionPool()[2].mpInternal() 
							+ (model()->motionPool()[3].mpInternal() + diff_q[solution_num][3]) / 2 / PI
							* dynamic_cast<ScrewJoint&>(model()->jointPool()[3]).pitch();

						++solution_num;
					}
				}

				if (solution_num == 0) return -1;

				auto real_solution = std::min_element(diff_norm, diff_norm + solution_num) - diff_norm;

				// link1~3 //
				double pe[6]{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
				pe[5] = model()->motionPool()[0].mpInternal() + diff_q[real_solution][0];
				model()->jointPool()[0].makI()->setPe(*model()->jointPool()[0].makJ(), pe, "123");

				pe[5] = model()->motionPool()[1].mpInternal() + diff_q[real_solution][1];
				model()->jointPool()[1].makI()->setPe(*model()->jointPool()[1].makJ(), pe, "123");

				pe[5] = 0.0;
				pe[2] = model()->motionPool()[2].mpInternal() + diff_q[real_solution][2];
				model()->jointPool()[2].makI()->setPe(*model()->jointPool()[2].makJ(), pe, "123");

				pe[2] = (model()->motionPool()[3].mpInternal() + diff_q[real_solution][3]) / 2 / PI
					* dynamic_cast<ScrewJoint&>(model()->jointPool()[3]).pitch();
				pe[5] = model()->motionPool()[3].mpInternal() + diff_q[real_solution][3];
				model()->jointPool()[3].makI()->setPe(*model()->jointPool()[3].makJ(), pe, "123");

				for (auto& m : model()->motionPool()) m.updP();
				return 0;
			}
			else {
				double input[4], output[4];
				model()->getOutputPos(output);
				if (auto ret = scaraInverse(dh, output, which_root_, input))
					return ret;

				// link1~4 //
				double pe[6]{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
				pe[5] = input[0];
				model()->jointPool()[0].makI()->setPe(*model()->jointPool()[0].makJ(), pe, "123");

				pe[5] = input[1];
				model()->jointPool()[1].makI()->setPe(*model()->jointPool()[1].makJ(), pe, "123");

				pe[5] = 0.0;
				pe[2] = input[2];
				model()->jointPool()[2].makI()->setPe(*model()->jointPool()[2].makJ(), pe, "123");

				pe[2] = input[3] / 2 / PI
					* dynamic_cast<ScrewJoint&>(model()->jointPool()[3]).pitch();
				pe[5] = input[3];
				model()->jointPool()[3].makI()->setPe(*model()->jointPool()[3].makJ(), pe, "123");

				for (auto& m : model()->motionPool()) m.updP();
				return 0;
			}
		}
		auto setWhichRoot(int root_of_0_to_7)->void { which_root_ = root_of_0_to_7; };
		auto whichRoot()const->int { return which_root_; }

		ScaraInverseKinematicSolver() = default;

	private:
		int which_root_{ 2 };
	};
	auto createModelScara(const ScaraParam &param)->std::unique_ptr<aris::dynamic::Model> {
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
		const double joint2_position[3]{ param.a , 0 , 0 };
		const double joint2_axis[3]{ 0 , 0 , 1 };
		const double joint3_position[3]{ param.a , param.b , 0 };
		const double joint3_axis[3]{ 0 , 0 , 1 };
		const double joint4_position[3]{ param.a , param.b , 0 };
		const double joint4_axis[3]{ 0 , 0 , 1 };

		auto &r1 = model->addRevoluteJoint(p1, model->ground(), joint1_position, joint1_axis);
		auto &r2 = model->addRevoluteJoint(p2, p1, joint2_position, joint2_axis);
		auto &j3 = model->addPrismaticJoint(p3, p2, joint3_position, joint3_axis);
		auto &r4 = model->addScrewJoint(p4, p3, joint4_position, joint4_axis, param.pitch);

		// add actuation //
		auto &m1 = model->addMotion(r1);
		auto &m2 = model->addMotion(r2);
		auto &m3 = model->addMotion(j3);
		auto &m4 = model->addMotion(r4);

		// 设置0位为打直的位置
		m2.setMpOffset(-aris::PI / 2);

		// 考虑到可能存在多解，4轴改为选用当前的位置作为周期求解
		m4.setRotateRange(std::numeric_limits<double>::infinity());

		const double default_mot_frc[3]{ 0.0, 0.0, 0.0 };
		m1.setFrcCoe(param.mot_frc_vec.size() == 6 ? param.mot_frc_vec[0].data() : default_mot_frc);
		m2.setFrcCoe(param.mot_frc_vec.size() == 6 ? param.mot_frc_vec[1].data() : default_mot_frc);
		m3.setFrcCoe(param.mot_frc_vec.size() == 6 ? param.mot_frc_vec[2].data() : default_mot_frc);
		m4.setFrcCoe(param.mot_frc_vec.size() == 6 ? param.mot_frc_vec[3].data() : default_mot_frc);

		// add ee general motion //
		double ee_i_pm[16]{ 1,0,0,param.a,0,1,0,param.b,0,0,1,0,0,0,0,1 };
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
		auto &inverse_kinematic = model->solverPool().add<aris::dynamic::ScaraInverseKinematicSolver>();
		auto &forward_kinematic = model->solverPool().add<aris::dynamic::ScaraForwardKinematicSolver>();
		auto &inverse_dynamic = model->solverPool().add<aris::dynamic::InverseDynamicSolver>();
		auto &forward_dynamic = model->solverPool().add<aris::dynamic::ForwardDynamicSolver>();

		// make topology correct // 
		for (auto &m : model->motionPool())m.activate(true);
		for (auto &gm : model->generalMotionPool())gm.activate(false);
		for (auto &f : model->forcePool())f.activate(false);

		model->init();
		return model;
	}

	auto planarScaraInverse(const double *param, const double *ee_xya, int which_root, double *input)->int {
		const double &a = param[0];
		const double &b = param[1];

		auto c = std::sqrt(ee_xya[0] * ee_xya[0] + ee_xya[1] * ee_xya[1]);
		if (c<std::abs(a - b) || c>std::abs(a + b)) return -1;

		input[0] = which_root == 0 ? std::atan2(ee_xya[1], ee_xya[0]) - std::acos((a*a + c * c - b * b) / (2 * a*c)) : std::atan2(ee_xya[1], ee_xya[0]) + std::acos((a*a + c * c - b * b) / (2 * a*c));
		input[1] = which_root == 0 ? -std::acos((a*a + b * b - c * c) / (2 * a*b)) + aris::PI / 2 : std::acos((a*a + b * b - c * c) / (2 * a*b)) - aris::PI * 3 / 2;
		input[2] = ee_xya[2] - input[1] - input[0];

		input[0] = std::fmod(input[0], 2 * PI);
		if (input[0] > PI) input[0] -= 2 * PI;
		if (input[0] < -PI) input[0] += 2 * PI;

		input[1] = std::fmod(input[1], 2 * PI);
		if (input[1] > PI) input[1] -= 2 * PI;
		if (input[1] < -PI) input[1] += 2 * PI;

		input[2] = std::fmod(input[2], 2 * PI);
		if (input[2] > PI) input[2] -= 2 * PI;
		if (input[2] < -PI) input[2] += 2 * PI;

		return 0;
	}
	class PlanarScaraInverseKinematicSolver :public aris::dynamic::InverseKinematicSolver {
	public:
		auto virtual kinPos()->int override {
			auto dh = dynamic_cast<aris::dynamic::MatrixVariable*>(model()->findVariable("dh"))->data().data();
			const int ROOT_NUM = 2;
			const int ROOT_SIZE = 3;
			if (which_root_ == ROOT_NUM) {
				int solution_num = 0;
				double diff_q[ROOT_NUM][ROOT_SIZE];
				double diff_norm[ROOT_NUM];

				for (int i = 0; i < ROOT_NUM; ++i) {
					double output[4];
					if (planarScaraInverse(dh, output, i, diff_q[solution_num])) {
						diff_norm[solution_num] = 0;
						for (int j = 0; j < ROOT_SIZE; ++j) {
							diff_q[solution_num][j] -= model()->motionPool()[j].mpInternal();

							// 如果是2 的话，忽略轴的范围，选择最近的可达解 //
							while (diff_q[solution_num][j] > PI) diff_q[solution_num][j] -= 2 * PI;
							while (diff_q[solution_num][j] < -PI)diff_q[solution_num][j] += 2 * PI;

							diff_norm[solution_num] += std::abs(diff_q[solution_num][j]);
						}

						++solution_num;
					}
				}

				if (solution_num == 0) return -1;

				auto real_solution = std::min_element(diff_norm, diff_norm + solution_num) - diff_norm;

				// link1~3 //
				double pe[6]{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
				pe[5] = model()->motionPool()[0].mpInternal() + diff_q[real_solution][0];
				model()->jointPool()[0].makI()->setPe(*model()->jointPool()[0].makJ(), pe, "123");

				pe[5] = model()->motionPool()[1].mpInternal() + diff_q[real_solution][1];
				model()->jointPool()[1].makI()->setPe(*model()->jointPool()[1].makJ(), pe, "123");

				pe[5] = model()->motionPool()[2].mpInternal() + diff_q[real_solution][2];
				model()->jointPool()[2].makI()->setPe(*model()->jointPool()[2].makJ(), pe, "123");

				for (auto& m : model()->motionPool()) m.updP();
				return 0;
			}
			else {
				double input[4], output[4];
				model()->getOutputPos(output);
				if (auto ret = planarScaraInverse(dh, output, which_root_, input))
					return ret;

				// link1~4 //
				double pe[6]{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
				pe[5] = input[0];
				model()->jointPool()[0].makI()->setPe(*model()->jointPool()[0].makJ(), pe, "123");

				pe[5] = input[1];
				model()->jointPool()[1].makI()->setPe(*model()->jointPool()[1].makJ(), pe, "123");

				pe[5] = input[2];
				model()->jointPool()[2].makI()->setPe(*model()->jointPool()[2].makJ(), pe, "123");

				for (auto& m : model()->motionPool()) m.updP();
				return 0;
			}
		}
		auto setWhichRoot(int root_of_0_to_7)->void { which_root_ = root_of_0_to_7; };
		auto whichRoot()const->int { return which_root_; }

		PlanarScaraInverseKinematicSolver() = default;

	private:
		int which_root_{ 2 };
	};
	auto createModelPlanarScara(const ScaraParam &param)->std::unique_ptr<aris::dynamic::Model> {
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

		// add geometry //

		// add joint //
		const double joint1_position[3]{ 0 , 0 , 0 };
		const double joint1_axis[3]{ 0 , 0 , 1 };
		const double joint2_position[3]{ param.a , 0 , 0 };
		const double joint2_axis[3]{ 0 , 0 , 1 };
		const double joint3_position[3]{ param.a , param.b , 0 };
		const double joint3_axis[3]{ 0 , 0 , 1 };

		auto &r1 = model->addRevoluteJoint(p1, model->ground(), joint1_position, joint1_axis);
		auto &r2 = model->addRevoluteJoint(p2, p1, joint2_position, joint2_axis);
		auto &r3 = model->addRevoluteJoint(p3, p2, joint3_position, joint3_axis);

		// add actuation //
		auto &m1 = model->addMotion(r1);
		auto &m2 = model->addMotion(r2);
		auto &m3 = model->addMotion(r3);

		const double default_mot_frc[3]{ 0.0, 0.0, 0.0 };
		m1.setFrcCoe(param.mot_frc_vec.size() == 6 ? param.mot_frc_vec[0].data() : default_mot_frc);
		m2.setFrcCoe(param.mot_frc_vec.size() == 6 ? param.mot_frc_vec[1].data() : default_mot_frc);
		m3.setFrcCoe(param.mot_frc_vec.size() == 6 ? param.mot_frc_vec[2].data() : default_mot_frc);

		// add ee general motion //
		double ee_i_pm[16]{ 1,0,0,param.a,0,1,0,param.b,0,0,1,0,0,0,0,1 };
		double ee_j_pm[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };

		auto &makI = p3.addMarker("tool0", ee_i_pm);
		auto &makJ = model->ground().addMarker("wobj0", ee_j_pm);
		model->variablePool().add<aris::dynamic::MatrixVariable>("tool0_axis_home", aris::core::Matrix(1, 6, 0.0));
		auto &ee = model->generalMotionPool().add<aris::dynamic::PlanarMotion>("tool", &makI, &makJ, false);

		// change robot pose wrt ground //
		double robot_pm[16];
		s_pe2pm(param.base2ref_pe, robot_pm, param.base2ref_pe_type.empty() ? "321" : param.base2ref_pe_type.c_str());

		p1.setPm(s_pm_dot_pm(robot_pm, *p1.pm()));
		p2.setPm(s_pm_dot_pm(robot_pm, *p2.pm()));
		p3.setPm(s_pm_dot_pm(robot_pm, *p3.pm()));
		r1.makJ()->setPrtPm(s_pm_dot_pm(robot_pm, *r1.makJ()->prtPm()));
		ee.makJ()->setPrtPm(s_pm_dot_pm(robot_pm, *ee.makJ()->prtPm()));

		// add tools and wobj //
		for (int i = 1; i < 17; ++i) {
			p3.addMarker("tool" + std::to_string(i), ee_i_pm);
		}
		for (int i = 1; i < 33; ++i) model->ground().markerPool().add<aris::dynamic::Marker>("wobj" + std::to_string(i), ee_j_pm);

		// add solver
		auto &inverse_kinematic = model->solverPool().add<aris::dynamic::PlanarScaraInverseKinematicSolver>();
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

	
	auto calibModelByTwoPoints(aris::dynamic::Model& scara, const double* points, std::string_view tool_name)->int {
		double result[2];
		if (s_calib_tool_two_pnts(points, result))return -1;

		auto tool = scara.generalMotionPool()[0].makI()->fatherPart().findMarker(tool_name);
		auto tool0 = scara.generalMotionPool()[0].makI()->fatherPart().findMarker("tool0");
		if (!tool || !tool0)return -2;

		// 只改变相对于tool0的x和y //
		double pm[16];
		tool->getPm(*tool0, pm);
		s_vc(2, result, 1, pm + 3, 4);
		tool->setPrtPm(s_pm_dot_pm(*tool0->prtPm(), pm));
		return 0;
	}

	ARIS_REGISTRATION{
		aris::core::class_<ScaraForwardKinematicSolver>("ScaraForwardKinematicSolver")
			.inherit<ForwardKinematicSolver>()
			;

		aris::core::class_<ScaraInverseKinematicSolver>("ScaraInverseKinematicSolver")
			.inherit<InverseKinematicSolver>()
			.prop("which_root", &ScaraInverseKinematicSolver::setWhichRoot, &ScaraInverseKinematicSolver::whichRoot)
			;

		aris::core::class_<PlanarScaraInverseKinematicSolver>("PlanarScaraInverseKinematicSolver")
			.inherit<InverseKinematicSolver>()
			.prop("which_root", &PlanarScaraInverseKinematicSolver::setWhichRoot, &PlanarScaraInverseKinematicSolver::whichRoot)
			;
	}
}
