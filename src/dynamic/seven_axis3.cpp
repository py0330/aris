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

#include "aris/dynamic/model.hpp"
#include "aris/dynamic/model_solver.hpp"
#include "aris/dynamic/seven_axis3.hpp"
#include "aris/core/reflection.hpp"

namespace aris::dynamic
{
	auto createModelSevenAxis3(const SevenAxisParam3 &param)->std::unique_ptr<aris::dynamic::Model>
	{
		std::unique_ptr<aris::dynamic::Model> model = std::make_unique<aris::dynamic::Model>();

		// 设置重力 //
		const double gravity[6]{ 0.0,0.0,-9.8,0.0,0.0,0.0 };
		model->environment().setGravity(gravity);

		// add part //
		const double default_iv[10]{ 1,0,0,0,0,0,0,0,0,0 };
		auto &p1 = model->partPool().add<Part>("L1", param.iv_vec.size() == 7 ? param.iv_vec[0].data() : default_iv);
		auto &p2 = model->partPool().add<Part>("L2", param.iv_vec.size() == 7 ? param.iv_vec[1].data() : default_iv);
		auto &p3 = model->partPool().add<Part>("L3", param.iv_vec.size() == 7 ? param.iv_vec[2].data() : default_iv);
		auto &p4 = model->partPool().add<Part>("L4", param.iv_vec.size() == 7 ? param.iv_vec[3].data() : default_iv);
		auto &p5 = model->partPool().add<Part>("L5", param.iv_vec.size() == 7 ? param.iv_vec[4].data() : default_iv);
		auto &p6 = model->partPool().add<Part>("L6", param.iv_vec.size() == 7 ? param.iv_vec[5].data() : default_iv);
		auto &p7 = model->partPool().add<Part>("L7", param.iv_vec.size() == 7 ? param.iv_vec[6].data() : default_iv);

		// add joint //
		const double j1_pos[3]{ 0.0, 0.0, param.d1 };
		const double j2_pos[3]{ 0.0, 0.0, param.d1 };
		const double j3_pos[3]{ 0.0, 0.0, param.d1 + param.d2 };
		const double j4_pos[3]{ 0.0, 0.0, param.d1 + param.d2 + param.d3 };
		const double j5_pos[3]{ 0.0, 0.0, param.d1 + param.d2 + param.d3 + param.d5 };
		const double j6_pos[3]{ 0.0, 0.0, param.d1 + param.d2 + param.d3 + param.d5 };
		const double j7_pos[3]{ 0.0, 0.0, param.d1 + param.d2 + param.d3 + param.d5 };

		const double j1_axis[3]{ 0.0, 0.0, 1.0 };
		const double j2_axis[3]{ 0.0, 1.0, 0.0 };
		const double j3_axis[3]{ 0.0, 1.0, 0.0 };
		const double j4_axis[3]{ 1.0, 0.0, 0.0 };
		const double j5_axis[3]{ 0.0, 0.0, 1.0 };
		const double j6_axis[3]{ 0.0, 1.0, 0.0 };
		const double j7_axis[3]{ 1.0, 0.0, 0.0 };

		auto &j1 = model->addRevoluteJoint(p1, model->ground(), j1_pos, j1_axis);
		auto &j2 = model->addRevoluteJoint(p2, p1, j2_pos, j2_axis);
		auto &j3 = model->addRevoluteJoint(p3, p2, j3_pos, j3_axis);
		auto &j4 = model->addRevoluteJoint(p4, p3, j4_pos, j4_axis);
		auto &j5 = model->addRevoluteJoint(p5, p4, j5_pos, j5_axis);
		auto &j6 = model->addRevoluteJoint(p6, p5, j6_pos, j6_axis);
		auto &j7 = model->addRevoluteJoint(p7, p6, j7_pos, j7_axis);

		// add actuation //
		auto &m1 = model->addMotion(j1);
		auto &m2 = model->addMotion(j2);
		auto &m3 = model->addMotion(j3);
		auto &m4 = model->addMotion(j4);
		auto &m5 = model->addMotion(j5);
		auto &m6 = model->addMotion(j6);
		auto &m7 = model->addMotion(j7);

		const double default_mot_frc[3]{0.0, 0.0, 0.0};

		m1.setFrcCoe(param.mot_frc_vec.size() == 7 ? param.mot_frc_vec[0].data() : default_mot_frc);
		m2.setFrcCoe(param.mot_frc_vec.size() == 7 ? param.mot_frc_vec[1].data() : default_mot_frc);
		m3.setFrcCoe(param.mot_frc_vec.size() == 7 ? param.mot_frc_vec[2].data() : default_mot_frc);
		m4.setFrcCoe(param.mot_frc_vec.size() == 7 ? param.mot_frc_vec[3].data() : default_mot_frc);
		m5.setFrcCoe(param.mot_frc_vec.size() == 7 ? param.mot_frc_vec[4].data() : default_mot_frc);
		m6.setFrcCoe(param.mot_frc_vec.size() == 7 ? param.mot_frc_vec[5].data() : default_mot_frc);
		m7.setFrcCoe(param.mot_frc_vec.size() == 7 ? param.mot_frc_vec[6].data() : default_mot_frc);

		// add ee general motion //
		const double axis_7_pe[]{ 0.0, 0.0, param.d1 + param.d2 + param.d3 + param.d5, 0.0, 0.0 ,0.0 };
		double axis_7_pm[16];
		double ee_i_pm[16], ee_i_wrt_axis_7_pm[16];
		double ee_j_pm[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };

		s_pe2pm(axis_7_pe, axis_7_pm, "321");
		s_pe2pm(param.tool0_pe, ee_i_wrt_axis_7_pm, param.tool0_pe_type.empty() ? "321" : param.tool0_pe_type.c_str());
		s_pm2pm(axis_7_pm, ee_i_wrt_axis_7_pm, ee_i_pm);

		auto &makI = p7.addMarker("tool0", ee_i_pm);
		auto &makJ = model->ground().addMarker("wobj0", ee_j_pm);
		auto &ee = model->generalMotionPool().add<aris::dynamic::GeneralMotion>("ee", &makI, &makJ, false);

		// change robot pose wrt ground //
		double robot_pm[16];
		s_pe2pm(param.base2ref_pe, robot_pm, param.base2ref_pe_type.empty() ? "321" : param.base2ref_pe_type.c_str());

		p1.setPm(s_pm_dot_pm(robot_pm, *p1.pm()));
		p2.setPm(s_pm_dot_pm(robot_pm, *p2.pm()));
		p3.setPm(s_pm_dot_pm(robot_pm, *p3.pm()));
		p4.setPm(s_pm_dot_pm(robot_pm, *p4.pm()));
		p5.setPm(s_pm_dot_pm(robot_pm, *p5.pm()));
		p6.setPm(s_pm_dot_pm(robot_pm, *p6.pm()));
		p7.setPm(s_pm_dot_pm(robot_pm, *p7.pm()));
		j1.makJ()->setPrtPm(s_pm_dot_pm(robot_pm, *j1.makJ()->prtPm()));
		ee.makJ()->setPrtPm(s_pm_dot_pm(robot_pm, *ee.makJ()->prtPm()));

		// add tools and wobj //
		for (int i = 1; i < 17; ++i)
		{
			p7.addMarker("tool" + std::to_string(i), ee_i_pm);
		}
		for (int i = 1; i < 33; ++i) model->ground().markerPool().add<aris::dynamic::Marker>("wobj" + std::to_string(i), ee_j_pm);


		// add solver
		auto &inverse_kinematic = model->solverPool().add<aris::dynamic::SevenAxisInverseKinematicSolver3>();
		auto &forward_kinematic = model->solverPool().add<ForwardKinematicSolver>();
		auto &inverse_dynamic = model->solverPool().add<aris::dynamic::InverseDynamicSolver>();
		auto &forward_dynamic = model->solverPool().add<aris::dynamic::ForwardDynamicSolver>();

		inverse_kinematic.setWhichRoot(8);
		inverse_kinematic.setAxisAngle(0.0);
		
		model->init();
		// make topology correct // 
		for (auto &m : model->motionPool())m.activate(true);
		for (auto &gm : model->generalMotionPool())gm.activate(false);
		for (auto &f : model->forcePool())f.activate(false);

		return model;
	}
	
	auto sevenAxisInverse(const SevenAxisParam3 &param, const double *ee_pm, double axis_angle, int which_root, double *input)->bool
	{
		// 七轴机器人构型：
		//
		//      EE
		//      |        x
		//     ---       y      ---
		//      |        z       |
		//                       d5
		//                       |
		//     ---       x      ---                                                            
		//                       |
		//                       d3
		//                       |   
		//     ---       y      ---
		//                       |
		//                       d2
		//                       |  
		//     ---       y      ---
		//      |        z    
		//     BASE
		//
		//  A 坐标系为前3轴的交点， z 轴和 1 轴平行， y 轴和 2 轴平行
		//  D 坐标系为 5 6 7 三根轴的交点，零位下与 A 坐标系方向一致
		//  
		//  
		// 
		
		auto d2 = param.d2;
		auto d3 = param.d3;
		auto d5 = param.d5;

		double pm_A_in_Ground[16]{ 1,0,0,0,0,1,0,0,0,0,1,param.d1,0,0,0,1 };
		double pm_EE_in_D[16];
		s_pe2pm(param.tool0_pe, pm_EE_in_D, param.base2ref_pe_type.c_str());

		double E_in_A[16];
		s_inv_pm_dot_pm(pm_A_in_Ground, ee_pm, E_in_A);
		double D_in_A[16];
		s_pm_dot_inv_pm(E_in_A, pm_EE_in_D, D_in_A);

		double q[7]{ 0 };

		// 轴角就是q4 //
		q[3] = axis_angle;

		// 求q3
		double distance_D = std::sqrt(D_in_A[3] * D_in_A[3] + D_in_A[7] * D_in_A[7] + D_in_A[11] * D_in_A[11]);
		auto D_modified = std::sqrt(distance_D * distance_D - d5 * d5* std::sin(axis_angle)* std::sin(axis_angle));
		auto d5_modified = std::cos(axis_angle)*d5 + d3;

		if (auto cq4 = (d5_modified*d5_modified + d2 * d2 - D_modified * D_modified) / (2 * d5_modified*d2); (cq4 - 1.0) > 1.0e-15 || (cq4 + 1.0) < -1.0e-15) {
			return false;
		}
		else {
			if (which_root & 0x01) {
				q[2] = -aris::PI + std::acos(std::max(std::min(cq4, 1.0), -1.0));
			}
			else {
				q[2] = aris::PI - std::acos(std::max(std::min(cq4, 1.0), -1.0));
			}
		}


		
		// 求q1与q2
		auto s2 = std::sin(q[2]);
		auto c2 = std::cos(q[2]);
		auto s3 = std::sin(q[3]);
		auto c3 = std::cos(q[3]);
		double pos_when_q1q2_equal_zero[3] = { d3*s2 + c3 * d5*s2, -d5 * s3, c2*d3 + c2 * c3*d5 + d2 };
		

		double q1[2], q2[2], D_pos_in_A[3];
		s_vc(3, D_in_A + 3, 4, D_pos_in_A, 1);
		if (s_sov_ab_arbitrary(pos_when_q1q2_equal_zero, D_pos_in_A, q1, q2, "32")) {
			return false;
		}
		else {
			q[0] = which_root & 0x02 ? q1[0] : q1[1];
			q[1] = which_root & 0x02 ? q2[0] : q2[1];
		}

		// 求 q5 q6 q7 //
		double rm_E_wrt_4[9], rm4[9], re_tem[3]{ q[0], q[1] + q[2], q[3] };
		s_re2rm(re_tem, rm4, "321");
		s_mm(3, 3, 3, rm4, ColMajor(3), D_in_A, 4, rm_E_wrt_4, 3);
		s_rm2re(rm_E_wrt_4, q + 4, "321");
		if (which_root & 0x04) {
			q[4] = q[4] > PI ? q[4] - PI : q[4] + PI;
			q[5] = PI - q[5];
			q[6] = q[6] > PI ? q[6] - PI : q[6] + PI;
		}

		// 添加所有的偏移 //
		for (int i = 0; i < 7; ++i)
		{
			while (q[i] > PI) q[i] -= 2 * PI;
			while (q[i] < -PI) q[i] += 2 * PI;
		}
		
		s_vc(7, q, input);
		
		return true;
	}
	struct SevenAxisInverseKinematicSolver3::Imp{
		int which_root_{ 0 };
		double axis_angle{ 0.0 };
		SevenAxisParam3 seven_axis_param;
		union
		{
			struct { Part* GR, *L1, *L2, *L3, *L4, *L5, *L6, *L7; };
			Part* parts[8];
		};
		union
		{
			struct { RevoluteJoint *R1, *R2, *R3, *R4, *R5, *R6, *R7; };
			RevoluteJoint* joints[7];
		};
		union
		{
			struct { Motion *M1, *M2, *M3, *M4, *M5, *M6, *M7; };
			Motion* motions[7];
		};
		GeneralMotion *ee;
	};
	auto SevenAxisInverseKinematicSolver3::allocateMemory()->void{
		InverseKinematicSolver::allocateMemory();

		this->imp_->GR;
		imp_->GR = &model()->partPool().at(0);
		imp_->L1 = &model()->partPool().at(1);
		imp_->L2 = &model()->partPool().at(2);
		imp_->L3 = &model()->partPool().at(3);
		imp_->L4 = &model()->partPool().at(4);
		imp_->L5 = &model()->partPool().at(5);
		imp_->L6 = &model()->partPool().at(6);
		imp_->L7 = &model()->partPool().at(7);

		imp_->R1 = dynamic_cast<RevoluteJoint*>(&model()->jointPool().at(0));
		imp_->R2 = dynamic_cast<RevoluteJoint*>(&model()->jointPool().at(1));
		imp_->R3 = dynamic_cast<RevoluteJoint*>(&model()->jointPool().at(2));
		imp_->R4 = dynamic_cast<RevoluteJoint*>(&model()->jointPool().at(3));
		imp_->R5 = dynamic_cast<RevoluteJoint*>(&model()->jointPool().at(4));
		imp_->R6 = dynamic_cast<RevoluteJoint*>(&model()->jointPool().at(5));
		imp_->R7 = dynamic_cast<RevoluteJoint*>(&model()->jointPool().at(6));

		imp_->M1 = &model()->motionPool().at(0);
		imp_->M2 = &model()->motionPool().at(1);
		imp_->M3 = &model()->motionPool().at(2);
		imp_->M4 = &model()->motionPool().at(3);
		imp_->M5 = &model()->motionPool().at(4);
		imp_->M6 = &model()->motionPool().at(5);
		imp_->M7 = &model()->motionPool().at(6);

		imp_->ee = dynamic_cast<GeneralMotion*>(&model()->generalMotionPool().at(0));

		auto &p = imp_->seven_axis_param;

		//  config seven axis param, tbd.....//
		imp_->seven_axis_param.d1 = imp_->R1->makJ()->prtPm()[2][3];

		double diff_p[3];
		s_vc(3, &imp_->R3->makJ()->prtPm()[0][3], 4, diff_p, 1);
		s_vs(3, &imp_->R2->makI()->prtPm()[0][3], 4, diff_p, 1);
		imp_->seven_axis_param.d2 = diff_p[2];

		s_vc(3, &imp_->R4->makJ()->prtPm()[0][3], 4, diff_p, 1);
		s_vs(3, &imp_->R3->makI()->prtPm()[0][3], 4, diff_p, 1);
		imp_->seven_axis_param.d3 = s_norm(3, diff_p);

		s_vc(3, &imp_->R6->makJ()->prtPm()[0][3], 4, diff_p, 1);
		s_vs(3, &imp_->R4->makI()->prtPm()[0][3], 4, diff_p, 1);
		imp_->seven_axis_param.d5 = s_norm(3, diff_p);

		// config tool0 //
		const double axis_7_pe[]{ 0.0, 0.0, imp_->seven_axis_param.d1 + imp_->seven_axis_param.d2 + imp_->seven_axis_param.d3 + imp_->seven_axis_param.d5, 0.0, 0.0 ,0.0 };
		double axis_7_pm[16];
		double ee_i_pm[16], ee_i_wrt_axis_7_pm[16];
		double ee_j_pm[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };

		s_vc(16, static_cast<const double*>(*imp_->ee->makI()->prtPm()), ee_i_pm);
		s_pe2pm(axis_7_pe, axis_7_pm, "321");
		s_inv_pm2pm(axis_7_pm, ee_i_pm, ee_i_wrt_axis_7_pm);
		imp_->seven_axis_param.tool0_pe_type = "321";
		s_pm2pe(ee_i_wrt_axis_7_pm, imp_->seven_axis_param.tool0_pe, "321");
	}
	auto SevenAxisInverseKinematicSolver3::kinPos()->int
	{
		// 求解轴角 //
		{
			this->setAxisAngle(*this->model()->motionPool()[3].p());
		}
		
		// 求解 //
		if (imp_->which_root_ == 8)
		{
			int solution_num = 0;
			double diff_q[8][7];
			double diff_norm[8];

			for (int i = 0; i < 8; ++i)
			{
				if (sevenAxisInverse(imp_->seven_axis_param, *imp_->ee->mpm(), imp_->axis_angle, i, diff_q[solution_num]))
				{
					diff_norm[solution_num] = 0;
					for (int j = 0; j < 7; ++j)
					{
						diff_q[solution_num][j] -= imp_->motions[j]->mpInternal();

						while (diff_q[solution_num][j] > PI) diff_q[solution_num][j] -= 2 * PI;
						while (diff_q[solution_num][j] < -PI)diff_q[solution_num][j] += 2 * PI;

						diff_norm[solution_num] += std::abs(diff_q[solution_num][j]);
					}

					++solution_num;
				}
			}

			if (solution_num == 0) return -1;

			auto real_solution = std::min_element(diff_norm, diff_norm + solution_num) - diff_norm;

			for (aris::Size i = 0; i < 7; ++i)
			{
				if (&imp_->joints[i]->makI()->fatherPart() == imp_->parts[i + 1])
				{
					double pm_prt_i[16], pm_mak_i[16], pm_rot[16];
					s_pe2pm(std::array<double, 6>{0, 0, 0, 0, 0, imp_->motions[i]->mpInternal() + diff_q[real_solution][i]}.data(), pm_rot);
					s_pm_dot_pm(*imp_->joints[i]->makJ()->pm(), pm_rot, pm_mak_i);
					s_pm_dot_inv_pm(pm_mak_i, *imp_->joints[i]->makI()->prtPm(), pm_prt_i);
					imp_->parts[i + 1]->setPm(pm_prt_i);
				}
				else
				{
					double pm_prt_j[16], pm_mak_j[16], pm_rot[16];
					s_pe2pm(std::array<double, 6>{0, 0, 0, 0, 0, -imp_->motions[i]->mpInternal() - diff_q[real_solution][i]}.data(), pm_rot);
					s_pm_dot_pm(*imp_->joints[i]->makI()->pm(), pm_rot, pm_mak_j);
					s_pm_dot_inv_pm(pm_mak_j, *imp_->joints[i]->makJ()->prtPm(), pm_prt_j);
					imp_->parts[i + 1]->setPm(pm_prt_j);
				}

				imp_->motions[i]->setMpInternal(imp_->motions[i]->mpInternal() + diff_q[real_solution][i]);
			}

			return 0;
		}
		else
		{
			if (double q[7]; sevenAxisInverse(imp_->seven_axis_param, *imp_->ee->mpm(), imp_->axis_angle, imp_->which_root_, q))
			{
				for (aris::Size i = 0; i < 7; ++i)
				{
					if (&imp_->joints[i]->makI()->fatherPart() == imp_->parts[i + 1])
					{
						double pm_prt_i[16], pm_mak_i[16], pm_rot[16];
						s_pe2pm(std::array<double, 6>{0, 0, 0, 0, 0, q[i]}.data(), pm_rot);
						s_pm_dot_pm(*imp_->joints[i]->makJ()->pm(), pm_rot, pm_mak_i);
						s_pm_dot_inv_pm(pm_mak_i, *imp_->joints[i]->makI()->prtPm(), pm_prt_i);
						imp_->parts[i + 1]->setPm(pm_prt_i);
					}
					else
					{
						double pm_prt_j[16], pm_mak_j[16], pm_rot[16];
						s_pe2pm(std::array<double, 6>{0, 0, 0, 0, 0, -q[i]}.data(), pm_rot);
						s_pm_dot_pm(*imp_->joints[i]->makI()->pm(), pm_rot, pm_mak_j);
						s_pm_dot_inv_pm(pm_mak_j, *imp_->joints[i]->makJ()->prtPm(), pm_prt_j);
						imp_->parts[i + 1]->setPm(pm_prt_j);
					}

					double last_mp = imp_->motions[i]->mpInternal();
					imp_->motions[i]->updP();
					while (imp_->motions[i]->mpInternal() - last_mp > PI)imp_->motions[i]->setMpInternal(imp_->motions[i]->mpInternal() - 2 * PI);
					while (imp_->motions[i]->mpInternal() - last_mp < -PI)imp_->motions[i]->setMpInternal(imp_->motions[i]->mpInternal() + 2 * PI);
				}

				return 0;
			}
			else return -2;
		}
	}
	auto SevenAxisInverseKinematicSolver3::setWhichRoot(int root_of_0_to_7)->void { imp_->which_root_ = root_of_0_to_7; }
	auto SevenAxisInverseKinematicSolver3::whichRoot()->int { return imp_->which_root_; }
	auto SevenAxisInverseKinematicSolver3::setAxisAngle(double axis_angle)->void { imp_->axis_angle = axis_angle; }
	SevenAxisInverseKinematicSolver3::~SevenAxisInverseKinematicSolver3() = default;
	SevenAxisInverseKinematicSolver3::SevenAxisInverseKinematicSolver3() :InverseKinematicSolver(1, 0.0), imp_(new Imp) {}
	ARIS_DEFINE_BIG_FOUR_CPP(SevenAxisInverseKinematicSolver3);

	ARIS_REGISTRATION{
		aris::core::class_<SevenAxisInverseKinematicSolver3>("SevenAxisInverseKinematicSolver3")
			.inherit<InverseKinematicSolver>()
			.prop("which_root", &SevenAxisInverseKinematicSolver3::setWhichRoot, &SevenAxisInverseKinematicSolver3::whichRoot)
			;
	}
}
