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
#include "aris/dynamic/mechanism_seven_axis2.hpp"
#include "aris/dynamic/kinematics.hpp"
#include "aris/core/reflection.hpp"

namespace aris::dynamic
{
	auto createModelSevenAxis2(const SevenAxisParam2 &param)->std::unique_ptr<aris::dynamic::Model>
	{
		std::unique_ptr<aris::dynamic::Model> model = std::make_unique<aris::dynamic::Model>();

		model->setName("OffsetSevenAxis");

		////////////////////////////  DH  /////////////////////////////
		model->variablePool().add<aris::dynamic::MatrixVariable>("dh", aris::core::Matrix({ param.d1, param.a2, param.d3, param.d5 }));
		model->variablePool().add<aris::dynamic::MatrixVariable>("tool0_pe", aris::core::Matrix(1, 6, param.tool0_pe));
		model->variablePool().add<aris::dynamic::StringVariable>("tool0_pe_type", param.tool0_pe_type.empty() ? std::string("321") : param.tool0_pe_type);
		model->variablePool().add<aris::dynamic::MatrixVariable>("base_pe", aris::core::Matrix(1, 6, param.base2ref_pe));
		model->variablePool().add<aris::dynamic::StringVariable>("base_pe_type", param.base2ref_pe_type.empty() ? std::string("321") : param.base2ref_pe_type);
		model->variablePool().add<aris::dynamic::MatrixVariable>("axis_range", aris::core::Matrix(1, 7, param.axis_range));
		model->variablePool().add<aris::dynamic::MatrixVariable>("install_method", aris::core::Matrix(1, 1, param.install_method));

		////////////////////////////  ENVIRONMENTS  /////////////////////////////
		const double gravity[6]{ 0.0,0.0,-9.8,0.0,0.0,0.0 };
		model->environment().setGravity(gravity);

		////////////////////////////  EE  /////////////////////////////
		const double axis_7_pe[]{ 0.0, 0.0, param.d1 + param.d3 + param.d5, 0.0, 0.0 ,0.0 };
		double axis_7_pm[16];
		double ee_i_pm[16], ee_i_wrt_axis_7_pm[16];
		double ee_j_pm[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };

		s_pe2pm(axis_7_pe, axis_7_pm, "321");
		s_pe2pm(param.tool0_pe, ee_i_wrt_axis_7_pm, param.tool0_pe_type.empty() ? "321" : param.tool0_pe_type.c_str());
		s_pm2pm(axis_7_pm, ee_i_wrt_axis_7_pm, ee_i_pm);

		////////////////////////////  PARTS  /////////////////////////////
		const double default_iv[10]{ 1,0,0,0,0,0,0,0,0,0 };
		auto &p1 = model->partPool().add<Part>("L1", param.iv_vec.size() == 7 ? param.iv_vec[0].data() : default_iv);
		auto &p2 = model->partPool().add<Part>("L2", param.iv_vec.size() == 7 ? param.iv_vec[1].data() : default_iv);
		auto &p3 = model->partPool().add<Part>("L3", param.iv_vec.size() == 7 ? param.iv_vec[2].data() : default_iv);
		auto &p4 = model->partPool().add<Part>("L4", param.iv_vec.size() == 7 ? param.iv_vec[3].data() : default_iv);
		auto &p5 = model->partPool().add<Part>("L5", param.iv_vec.size() == 7 ? param.iv_vec[4].data() : default_iv);
		auto &p6 = model->partPool().add<Part>("L6", param.iv_vec.size() == 7 ? param.iv_vec[5].data() : default_iv);
		auto &p7 = model->partPool().add<Part>("L7", param.iv_vec.size() == 7 ? param.iv_vec[6].data() : default_iv, ee_i_pm);

		////////////////////////////  JOINTS  /////////////////////////////
		const double j1_pos[3]{ 0.0, 0.0, param.d1 };
		const double j2_pos[3]{ 0.0, 0.0, param.d1 };
		const double j3_pos[3]{ 0.0, param.a2, param.d1 };
		const double j4_pos[3]{ 0.0, 0.0, param.d1 + param.d3 };
		const double j5_pos[3]{ 0.0, 0.0, param.d1 + param.d3 + param.d5 };
		const double j6_pos[3]{ 0.0, 0.0, param.d1 + param.d3 + param.d5 };
		const double j7_pos[3]{ 0.0, 0.0, param.d1 + param.d3 + param.d5 };

		const double j1_axis[3]{ 0.0, 0.0, 1.0 };
		const double j2_axis[3]{ 0.0, 1.0, 0.0 };
		const double j3_axis[3]{ 0.0, 0.0, 1.0 };
		const double j4_axis[3]{ 0.0, 1.0, 0.0 };
		const double j5_axis[3]{ 0.0, 0.0, 1.0 };
		const double j6_axis[3]{ 0.0, 1.0, 0.0 };
		const double j7_axis[3]{ 0.0, 0.0, 1.0 };

		auto &j1 = model->addRevoluteJoint(p1, model->ground(), j1_pos, j1_axis);
		auto &j2 = model->addRevoluteJoint(p2, p1, j2_pos, j2_axis);
		auto &j3 = model->addRevoluteJoint(p3, p2, j3_pos, j3_axis);
		auto &j4 = model->addRevoluteJoint(p4, p3, j4_pos, j4_axis);
		auto &j5 = model->addRevoluteJoint(p5, p4, j5_pos, j5_axis);
		auto &j6 = model->addRevoluteJoint(p6, p5, j6_pos, j6_axis);
		auto &j7 = model->addRevoluteJoint(p7, p6, j7_pos, j7_axis);

		////////////////////////////  MOTIONS  /////////////////////////////
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

		////////////////////////////  EES  /////////////////////////////
		auto &makI = p7.addMarker("tool0");
		auto &makJ = model->ground().addMarker("wobj0", ee_j_pm);
		auto &ee = model->generalMotionPool().add<aris::dynamic::GeneralMotion>("ee", &makI, &makJ, false);
		auto &arm_mot = model->generalMotionPool().add<aris::dynamic::Motion>("arm_mot", m3.makI(), m3.makJ(), 5);

		////////////////////////////  INSTALL METHODS  /////////////////////////////
		double install_pm_relative[16];
		switch (param.install_method) {
		case 0:
			s_eye(4, install_pm_relative);
			break;
		case 1:
			s_eye(4, install_pm_relative);
			s_rmx(aris::PI, install_pm_relative, 4);
			break;
		case 2: {
			double pe[6]{ 0,0,0,aris::PI , aris::PI / 2 , 0, };
			s_pe2pm(pe, install_pm_relative, "123");
			break;
		}
		case 3: {
			s_eye(4, install_pm_relative);
			s_rmy(aris::PI / 2, install_pm_relative, 4);
			break;
		}
		default:
			THROW_FILE_LINE("INVALID value for install method");
		}
		double install_pm[16];
		s_pm_dot_pm(install_pm_relative, *j1.makJ()->prtPm(), install_pm);
		j1.makJ()->setPrtPm(install_pm);

		////////////////////////////  ROBOT POSITION  /////////////////////////////
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

		////////////////////////////  TOOLS WOBJS  /////////////////////////////
		for (int i = 1; i < 17; ++i)
			p7.addMarker("tool" + std::to_string(i), *ee.makI()->prtPm());
		for (int i = 1; i < 33; ++i) 
			model->ground().markerPool().add<aris::dynamic::Marker>("wobj" + std::to_string(i), *ee.makJ()->prtPm());


		////////////////////////////  SOLVERS  /////////////////////////////
		auto &inverse_kinematic = model->solverPool().add<aris::dynamic::SevenAxisInverseKinematicSolver2>();
		auto &forward_kinematic = model->solverPool().add<ForwardKinematicSolver>();
		auto &inverse_dynamic = model->solverPool().add<aris::dynamic::InverseDynamicSolver>();
		auto &forward_dynamic = model->solverPool().add<aris::dynamic::ForwardDynamicSolver>();

		inverse_kinematic.setWhichRoot(8);

		////////////////////////////  TOPOLOGY  /////////////////////////////
		for (auto &m : model->motionPool())m.activate(true);
		for (auto &gm : model->generalMotionPool())gm.activate(false);
		for (auto &f : model->forcePool())f.activate(false);

		model->init();
		return model;
	}
	
	auto sevenAxisInverse(const void* para, const double *ee_pos, const double *current_input, int which_root, double *input)->bool
	{
		// 七轴机器人构型：
		//
		//      EE
		//      |        z
		//     ---       y      ---
		//      |        z       |
		//                       d5
		//                       |
		//     ---       y      ---                                                            
		//                       |
		//                       d3
		//      |        z       |   
		//     ---       y      ---
		//      |        z    
		//     BASE
		//
		//  A 坐标系为前3轴的交点， z 轴和 1 轴平行， y 轴和 2 轴平行
		//  D 坐标系为 5 6 7 三根轴的交点，零位下与 A 坐标系方向一致
		//  
		//  
		// 
		auto& param = *reinterpret_cast<const SevenAxisParam2*>(para);
		const double* ee_pm = ee_pos;
		const double axis_angle = ee_pos[16];

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

		// 轴角就是q3 //
		q[2] = axis_angle;

		// 求q4
		double distance_D = std::sqrt(D_in_A[3] * D_in_A[3] + D_in_A[7] * D_in_A[7] + D_in_A[11] * D_in_A[11]);
		if (distance_D < std::abs(param.a2))return false;

		auto d3_modified = std::sqrt(d3 * d3 + std::sin(q[2])*std::sin(q[2]) * param.a2 * param.a2);
		auto D_modified = std::sqrt(distance_D * distance_D - (1 - std::cos(q[2])) * (1 - std::cos(q[2])) * param.a2 * param.a2);
		if (auto cq4 = (d3_modified*d3_modified + d5 * d5 - D_modified * D_modified) / (2 * d3_modified*d5); cq4 > 1.0 || cq4 < -1.0){
			return false;
		}
		else{
			if (which_root & 0x01){
				q[3] = -aris::PI + std::acos(cq4) + std::atan2(std::sin(q[2]) * param.a2 , d3);
			}
			else{
				q[3] = aris::PI - std::acos(cq4) + std::atan2(std::sin(q[2]) * param.a2 , d3);
			}
		}

		// 求q1与q2
		auto s2 = std::sin(q[2]);
		auto c2 = std::cos(q[2]);
		auto s3 = std::sin(q[3]);
		auto c3 = std::cos(q[3]);
		double pos_when_q1q2_equal_zero[3] = { s2*param.a2 + c2*s3*d5, (1 - c2)*param.a2+s2*s3*d5, d3 + c3*d5 };

		if (pos_when_q1q2_equal_zero[0] * pos_when_q1q2_equal_zero[0] + pos_when_q1q2_equal_zero[2] * pos_when_q1q2_equal_zero[2] < D_in_A[11] * D_in_A[11]){
			return false;
		}
		else {
			double q2_tem[2];
			s_sov_theta(-pos_when_q1q2_equal_zero[0], pos_when_q1q2_equal_zero[2], D_in_A[11], q2_tem);

			if (which_root & 0x02) {
				q[1] = q2_tem[0];
				
			}
			else{
				q[1] = q2_tem[1];
			}

			double a1 = -pos_when_q1q2_equal_zero[1];
			double a2 = pos_when_q1q2_equal_zero[2] * std::sin(q[1]) + pos_when_q1q2_equal_zero[0] * std::cos(q[1]);
			double b1 = pos_when_q1q2_equal_zero[0] * std::cos(q[1]) + pos_when_q1q2_equal_zero[2] * std::sin(q[1]);
			double b2 = pos_when_q1q2_equal_zero[1];

			double c1 = D_in_A[3];
			double c2 = D_in_A[7];

			q[0] = std::atan2(b1*c2-b2*c1, c1*a2-c2*a1);
		}

		// 求 q5 q6 q7 //
		double rm_E_wrt_4[9], rm3[9], rm4[9], tem[9];
		s_re2rm(std::array<double, 3>{q[0], q[1], q[2]}.data(), rm3, "323");
		s_rmy(q[3], tem);
		s_mm(3, 3, 3, rm3, tem, rm4);
		s_mm(3, 3, 3, rm4, ColMajor(3), D_in_A, 4, rm_E_wrt_4, 3);
		s_rm2re(rm_E_wrt_4, q + 4, "323");
		if (which_root & 0x04) {
			q[4] = q[4] > PI ? q[4] - PI : q[4] + PI;
			q[5] = 2 * PI - q[5];
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
	struct SevenAxisInverseKinematicSolver2::Imp{
		int which_root_{ 0 };
		SevenAxisParam2 seven_axis_param;
		union
		{
			struct { Part* GR, *L1, *L2, *L3, *L4, *L5, *L6, *L7; };
			Part* parts[8]{ nullptr };
		};
		union
		{
			struct { RevoluteJoint *R1, *R2, *R3, *R4, *R5, *R6, *R7; };
			RevoluteJoint* joints[7]{ nullptr };
		};
		union
		{
			struct { Motion *M1, *M2, *M3, *M4, *M5, *M6, *M7; };
			Motion* motions[7]{ nullptr };
		};
		GeneralMotion *EE{ nullptr };
	};
	auto SevenAxisInverseKinematicSolver2::allocateMemory()->void
	{
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

		imp_->EE = dynamic_cast<GeneralMotion*>(&model()->generalMotionPool().at(0));


		auto &p = imp_->seven_axis_param;
		
		//  config seven axis param, tbd.....//
		double r1_pos_wrt_base[3];
		s_vc(3, &imp_->R1->makJ()->prtPm()[0][3], 4, r1_pos_wrt_base, 1);
		s_vs(3, &imp_->EE->makJ()->prtPm()[0][3], 4, r1_pos_wrt_base, 1);
		imp_->seven_axis_param.d1 = s_vv(3, r1_pos_wrt_base, 1, &imp_->EE->makJ()->prtPm()[0][2], 4);

		double diff_p[3];
		s_vc(3, &imp_->R4->makJ()->prtPm()[0][3], 4, diff_p, 1);
		s_vs(3, &imp_->R3->makI()->prtPm()[0][3], 4, diff_p, 1);
		imp_->seven_axis_param.d3 = diff_p[2];
		imp_->seven_axis_param.a2 = -diff_p[1];

		s_vc(3, &imp_->R5->makJ()->prtPm()[0][3], 4, diff_p, 1);
		s_vs(3, &imp_->R4->makI()->prtPm()[0][3], 4, diff_p, 1);
		imp_->seven_axis_param.d5 = s_norm(3, diff_p);

		// config tool0 //
		// 
		// solve:
		// P_tool0_wrt_eei
		// 
		// eei~P_tool0 = eei~P_L7 * L7~P_tool0
		//             = eei~P_R7i * R7i~P_L7 * L7~P_tool0
		//             = eei~P_R7j * R7j~P_R7i * R7i~P_L7 * L7~P_tool0
		//             = eei~P_L6 * L6~P_R7j * R7j~P_R7i * R7i~P_L7 * L7~P_tool0
		// 
		// at init point:
		// eei~P_L6    = [0,0,-d1-d3-d5,0,0,0]
		// R7j~P_R7i   = eye(4)
		// L7~P_tool0  = eye(4)
		//
		double pm_temp1[16], pm_temp2[16];
		const double pm_eei_wrt_L6[16]{
			1,0,0,0,
			0,1,0,0,
			0,0,1,-imp_->seven_axis_param.d1 - imp_->seven_axis_param.d3 - imp_->seven_axis_param.d5,
			0,0,0,1,
		};// only z changes

		s_pm_dot_pm(pm_eei_wrt_L6, *imp_->R7->makJ()->prtPm(), pm_temp1);
		s_pm_dot_inv_pm(pm_temp1, *imp_->R7->makI()->prtPm(), pm_temp2);

		imp_->seven_axis_param.tool0_pe_type = "321";
		s_pm2pe(pm_temp2, imp_->seven_axis_param.tool0_pe, "321");
	}
	
	auto SevenAxisInverseKinematicSolver2::kinPos()->int{
		double output_pos[17], input_pos[7], current_input_pos[7];
		model()->getOutputPos(output_pos);
		model()->getInputPos(current_input_pos);

		if (auto ret = kinPosPure(output_pos, input_pos, whichRoot()))
			return ret;

		// 设置所有杆件位置 //
		for (aris::Size i = 0; i < 7; ++i) {
			if (&imp_->joints[i]->makI()->fatherPart() == imp_->parts[i + 1]) {
				double pm_prt_i[16], pm_mak_i[16], pm_rot[16];
				double pe[6]{ 0, 0, 0, 0, 0, input_pos[i] };
				s_pe2pm(pe, pm_rot);
				s_pm_dot_pm(*imp_->joints[i]->makJ()->pm(), pm_rot, pm_mak_i);
				s_pm_dot_inv_pm(pm_mak_i, *imp_->joints[i]->makI()->prtPm(), pm_prt_i);
				imp_->parts[i + 1]->setPm(pm_prt_i);
			}
			else {
				double pm_prt_j[16], pm_mak_j[16], pm_rot[16];
				double pe[6]{ 0, 0, 0, 0, 0, -input_pos[i] };
				s_pe2pm(pe, pm_rot);
				s_pm_dot_pm(*imp_->joints[i]->makI()->pm(), pm_rot, pm_mak_j);
				s_pm_dot_inv_pm(pm_mak_j, *imp_->joints[i]->makJ()->prtPm(), pm_prt_j);
				imp_->parts[i + 1]->setPm(pm_prt_j);
			}
		}

		// 设置电机位置 //
		for (aris::Size i = 0; i < 7; ++i) {
			imp_->motions[i]->setMpInternal(input_pos[i]);
		}

		return 0;
	}
	auto SevenAxisInverseKinematicSolver2::kinPosPure(const double* output, double* input, int which_root, const double* current_input)->int {
		double ee_pos[17]{}, root_mem[7]{};

		switch (imp_->EE->poseType()) {
		case GeneralMotion::PoseType::EULER123:
			s_pe2pm(output, ee_pos, "123");
			ee_pos[16] = output[6];
			break;
		case GeneralMotion::PoseType::EULER321:
			s_pe2pm(output, ee_pos, "321");
			ee_pos[16] = output[6];
			break;
		case GeneralMotion::PoseType::EULER313:
			s_pe2pm(output, ee_pos, "313");
			ee_pos[16] = output[6];
			break;
		case GeneralMotion::PoseType::QUATERNION:
			s_pq2pm(output, ee_pos);
			ee_pos[16] = output[7];
			break;
		case GeneralMotion::PoseType::POSE_MATRIX:
			s_vc(16, output, ee_pos);
			ee_pos[16] = output[16];
			break;
		}
		

		constexpr double input_period[7]{
			aris::PI * 2, aris::PI * 2,aris::PI * 2,aris::PI * 2,aris::PI * 2,aris::PI * 2,aris::PI * 2,
		};

		if (current_input == nullptr) {
			double current_input_pos[7];
			for (int i = 0; i < 7; ++i)
				current_input_pos[i] = model()->motionPool()[i].mpInternal();
			return s_ik(7, rootNumber(), &imp_->seven_axis_param, sevenAxisInverse, which_root, ee_pos, input, root_mem, input_period, current_input_pos);
		}
		else {
			return s_ik(7, rootNumber(), &imp_->seven_axis_param, sevenAxisInverse, which_root, ee_pos, input, root_mem, input_period, current_input);
		}
	}
	SevenAxisInverseKinematicSolver2::~SevenAxisInverseKinematicSolver2() = default;
	SevenAxisInverseKinematicSolver2::SevenAxisInverseKinematicSolver2() :InverseKinematicSolver(1, 0.0), imp_(new Imp) {
		setWhichRoot(8);
		setRootNumber(8);
	}
	ARIS_DEFINE_BIG_FOUR_CPP(SevenAxisInverseKinematicSolver2);

	ARIS_REGISTRATION{
		aris::core::class_<SevenAxisInverseKinematicSolver2>("SevenAxisInverseKinematicSolver2")
			.inherit<InverseKinematicSolver>()
			;
	}
}
