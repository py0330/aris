#include <cmath>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <algorithm>
#include <limits>
#include <sstream>
#include <regex>
#include <type_traits>
#include <array>

#include "aris/dynamic/model.hpp"
#include "aris/dynamic/model_solver.hpp"
#include "aris/dynamic/mechanism_seven_axis5.hpp"
#include "aris/dynamic/kinematics.hpp"
#include "aris/core/reflection.hpp"

namespace aris::dynamic{
	struct SevenAxisInverseKinematicSolver5::Imp {
		int which_root_{ 0 };
		SevenAxisParam5 seven_axis_param;
		union
		{
			struct { Part* GR, * L1, * L2, * L3, * L4, * L5, * L6, * L7; };
			Part* parts[8]{ nullptr };
		};
		union
		{
			struct { RevoluteJoint* R1, * R2, * R3, * R4, * R5, * R6, * R7; };
			RevoluteJoint* joints[7]{ nullptr };
		};
		union
		{
			struct { Motion* M1, * M2, * M3, * M4, * M5, * M6, * M7; };
			Motion* motions[7]{ nullptr };
		};
		GeneralMotion* EE{ nullptr };
	};

	auto createModelSevenAxis5(const SevenAxisParam5 &param)->std::unique_ptr<aris::dynamic::Model>{
		std::unique_ptr<aris::dynamic::Model> model = std::make_unique<aris::dynamic::Model>();

		model->setName("SevenAxis5");
		model->setMechanismType("SevenAxis5");

		////////////////////////////  DH  /////////////////////////////
		model->variablePool().add<aris::dynamic::MatrixVariable>("dh", aris::core::Matrix({ param.d1, param.d2, param.d3 }));
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
		// 末端法兰位于末端链（z 方向叠加 d1+d2+d3）处 //
		const double axis_7_pe[]{ 0.0, 0.0, param.d1 + param.d2 + param.d3, 0.0, 0.0 ,0.0 };
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
		// 转轴序列 z-x-z-x-z-x-y，偏移均为相对上一关节的 z 方向 //
		const double j1_pos[3]{ 0.0, 0.0, 0.0 };
		const double j2_pos[3]{ 0.0, 0.0, 0.0 };
		const double j3_pos[3]{ 0.0, 0.0, 0.0 };
		const double j4_pos[3]{ 0.0, 0.0, param.d1 };
		const double j5_pos[3]{ 0.0, 0.0, param.d1 };
		const double j6_pos[3]{ 0.0, 0.0, param.d1 + param.d2 };
		const double j7_pos[3]{ 0.0, 0.0, param.d1 + param.d2 + param.d3 };

		const double j1_axis[3]{ 0.0, 0.0, 1.0 };
		const double j2_axis[3]{ 1.0, 0.0, 0.0 };
		const double j3_axis[3]{ 0.0, 0.0, 1.0 };
		const double j4_axis[3]{ 1.0, 0.0, 0.0 };
		const double j5_axis[3]{ 0.0, 0.0, 1.0 };
		const double j6_axis[3]{ 1.0, 0.0, 0.0 };
		const double j7_axis[3]{ 0.0, 1.0, 0.0 };

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

		m1.setRotateRange(param.axis_range[0]);
		m2.setRotateRange(param.axis_range[1]);
		m3.setRotateRange(param.axis_range[2]);
		m4.setRotateRange(param.axis_range[3]);
		m5.setRotateRange(param.axis_range[4]);
		m6.setRotateRange(param.axis_range[5]);
		m7.setRotateRange(param.axis_range[6]);

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
		auto &arm_mot = model->generalMotionPool().add<aris::dynamic::Motion>("arm_mot", m6.makI(), m6.makJ(), 5);

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
		auto &inverse_kinematic = model->solverPool().add<aris::dynamic::SevenAxisInverseKinematicSolver5>();
		auto &forward_kinematic = model->solverPool().add<aris::dynamic::SerialForwardKinematicSolver>();
		auto &inverse_dynamic = model->solverPool().add<aris::dynamic::InverseDynamicSolver>();
		auto &forward_dynamic = model->solverPool().add<aris::dynamic::ForwardDynamicSolver>();
		inverse_kinematic.imp_->seven_axis_param = param;
		inverse_kinematic.setWhichRoot(8);

		////////////////////////////  TOPOLOGY  /////////////////////////////
		for (auto &m : model->motionPool())m.activate(true);
		for (auto &gm : model->generalMotionPool())gm.activate(false);
		for (auto &f : model->forcePool())f.activate(false);

		model->init();
		return model;
	}

	// 反解函数 //
	auto sevenAxisInverse5(const void* para, const double* ee_pos, const double* current_input, std::int64_t which_root, double* input)->int{
        // 七轴机器人构型：
        //
        //      EE
        //      o        y      ---
        //                       |
        //                       d3
        //                       |
        //     ---       x      ---
        //      |        z       |
        //                       d2
        //                       |
        //     ---       x      ---                                                            
        //                       |
        //                       d1
        //      |        z       |   
        //     ---       x      ---
        //      |        z    
        //     BASE
        //
        //  A 坐标系为前3轴的交点， z 轴和 1 轴平行， y 轴和 2 轴平行
        //  D 坐标系为 5 7 两根轴零位时的交点，零位下与 A 坐标系方向一致
        //
        
        
        // 七根转轴产生的位姿运动：
        // pm1 = [c1,-s1, 0, 0,
        // 	   s1, c1, 0, 0,
        // 	    0,  0, 1, 0,
        // 	    0,  0, 0, 1,];
        //
        // pm2 = [  1,  0,  0, 0,
        // 	     0, c2,-s2, 0,
        // 	     0, s2, c2, 0,
        // 	     0,  0,  0, 1,];
        //
        // pm3 = [c3,-s3, 0, 0,
        // 	   s3, c3, 0, 0,
        // 	    0,  0, 1, 0,
        // 	    0,  0, 0, 1,];
        //
        // pm4 = [  1,  0,  0, 0,
        // 	     0, c4,-s4, d1*s4,
        // 	     0, s4, c4, d1*(1-c4),
        // 	     0,  0,  0, 1,];
        //
        // pm5 = [c5,-s5, 0, 0,
        // 	   s5, c5, 0, 0,
        // 	    0,  0, 1, 0,
        // 	    0,  0, 0, 1,];
        //
        // pm6 = [  1,  0,  0, 0,
        // 	     0, c6,-s6, (d1+d2)*s6,
        // 	     0, s6, c6, (d1+d2)*(1-c6),
        // 	     0,  0,  0, 1,];
        //
        // pm7 = [c7, 0, s7, -(d1+d2+d3)*s7,
        // 	    0, 1,  0, 0,
        //       -s7, 0, c7, (d1+d2+d3)*(1-c7),
        // 	    0, 0,  0, 1,];

        // 后四根转轴产生的位姿运动：
        // inv_pm7 = [c7, 0,-s7, 0,
        // 	    0, 1,  0, 0,
        //        s7, 0, c7, 0,
        // 	    0, 0,  0, 1,];
        //
        // inv_pm6 = [  1,  0,  0, 0,
        // 	     0, c6, s6, d*s6,
        // 	     0,-s6, c6, -d*(1-c6),
        // 	     0,  0,  0, 1,];
        //
        // inv_pm5 = [c5,s5, 0, 0,
        // 	  -s5,c5, 0, 0,
        // 	    0, 0, 1, 0,
        // 	    0, 0, 0, 1,];
        //
        // inv_pm4 = [  1,  0,  0, 0,
        // 	     0, c4, s4, e*s4,
        // 	     0,-s4, c4, -e*(1-c4),
        // 	     0,  0,  0, 1,];

        // 假设 x，y，z 为 A 在 D 下的坐标，有：
        // [x,y,z,1]' = inv_pm7*inv_pm6*inv_pm5*inv_pm4*[0;0;-(d1+d2+d3);1]
        //
        // 上式展开，并做如下变化可得（t6 = tan（q6））： 
        //
        //                              -d1*s4*s5    == x*c7+z*s7
        //       d3 + d2*c6 + d1*c4*c6 - d1*s4*c5*s6 == x*s7-z*c7
        //   -d3*t6 - d2*s6 - d1*c4*s6 - d1*s4*c5*c6 == y - d3*t6
        // 
        // 1-3式两侧平方和，可以消去 q7 与 q5 影响，可得：
        // A = (2*d1*d2*c6 + 2*d1*d3);
        // B = (x*x+z*z+y*y-d1^2-d2^2-d3^2)*c6 - 2*y*d3*s6 - 2*d2*d3;
        // q4 = acos(B/A) or -acos(B/A)
        // 
        // 进一步：
        // C = -d1*s4*c6;
        // D = y + d2*s6 + d1*c4*s6;
        // q5 = acos(D/C) or -acos(D/C)
        //
        // 进一步：
        // E = -d1*s4*s5;
        // F = d3 + d2*c6 + d1*c4*c6 - d1*s4*c5*s6;
        // q7 = atan2((E*z + F*x),(E*x - F*z));
        //
        // 求出 q4, q5, q6, q7 后，可以求得：
        // rm4567 = rm4*rm5*rm6*rm7
        // rm123 = rm4567'*rm_ee
        // 
        // [q1,q2,q3] = s_rm2re(313)

        
		auto& param = *reinterpret_cast<const SevenAxisParam5*>(para);
		const double* ee_pm = ee_pos;
		const double axis_angle = ee_pos[16];

		const double d1 = param.d1, d2 = param.d2, d3 = param.d3;

		// A 在 ground 中的位姿（base2ref），以及 tool0 相对 D 的位姿 //
		double pm_A_in_Ground[16];
		s_pe2pm(param.base2ref_pe, pm_A_in_Ground, param.base2ref_pe_type.empty() ? "321" : param.base2ref_pe_type.c_str());
		double pm_EE_in_D[16];
		s_pe2pm(param.tool0_pe, pm_EE_in_D, param.tool0_pe_type.empty() ? "321" : param.tool0_pe_type.c_str());

		// EE 在 A 中，再去除 tool0 得 D 在 A 中 //
		double E_in_A[16];
		s_inv_pm_dot_pm(pm_A_in_Ground, ee_pm, E_in_A);
		double D_in_A[16];
		s_pm_dot_inv_pm(E_in_A, pm_EE_in_D, D_in_A);

		// x,y,z = A 在 D 中的坐标 //
		double A_in_D[16];
		s_inv_pm(D_in_A, A_in_D);
		const double x = A_in_D[3], y = A_in_D[7], z = A_in_D[11];

		double q[7]{ 0 };

		// 冗余自由度：q6 = 轴角 //
		q[5] = axis_angle;
		const double c6 = std::cos(q[5]), s6 = std::sin(q[5]);

		// 求 q4 //
		const double A = 2 * d1*d2*c6 + 2 * d1*d3;
		const double B = (x*x + z*z + y*y - d1*d1 - d2*d2 - d3*d3)*c6 - 2 * y*d3*s6 - 2 * d2*d3;
		if (std::abs(A) < 1e-12) return -1;
		const double cq4 = B / A;
		if (cq4 > 1.0 || cq4 < -1.0) return -1;
		q[3] = (which_root & 0x01) ? -std::acos(cq4) : std::acos(cq4);
		const double s4 = std::sin(q[3]), c4 = std::cos(q[3]);

		// 求 q5 //
		const double C = -d1*s4*c6;
		const double D = y + d2*s6 + d1*c4*s6;
		if (std::abs(C) < 1e-12) return -1;
		const double cq5 = D / C;
		if (cq5 > 1.0 || cq5 < -1.0) return -1;
		q[4] = (which_root & 0x02) ? -std::acos(cq5) : std::acos(cq5);
		const double s5 = std::sin(q[4]), c5 = std::cos(q[4]);

		// 求 q7 //
		const double E = -d1*s4*s5;
		const double F = d3 + d2*c6 + d1*c4*c6 - d1*s4*c5*s6;
		if (std::abs(E*x - F*z) < 1e-12 && std::abs(E*z + F*x) < 1e-12)
			q[6] = current_input[6];
		else
			q[6] = std::atan2(E*z + F*x, E*x - F*z);

		// 求 q1,q2,q3：rm4567 = Rx(q4)*Rz(q5)*Rx(q6)*Ry(q7) //
		double rm4[9], rm5[9], rm6[9], rm7[9], rm45[9], rm456[9], rm4567[9];
		s_rmx(q[3], rm4);
		s_rmz(q[4], rm5);
		s_rmx(q[5], rm6);
		s_rmy(q[6], rm7);
		s_mm(3, 3, 3, rm4, rm5, rm45);
		s_mm(3, 3, 3, rm45, rm6, rm456);
		s_mm(3, 3, 3, rm456, rm7, rm4567);

		// rm123 = R_D * rm4567'（R_D = R123 * R4567）//
		const double R_D[9]{ D_in_A[0], D_in_A[1], D_in_A[2], D_in_A[4], D_in_A[5], D_in_A[6], D_in_A[8], D_in_A[9], D_in_A[10] };
		double rm123[9];
		for (int r = 0; r < 3; ++r)
			for (int c = 0; c < 3; ++c) {
				double sum = 0.0;
				for (int k = 0; k < 3; ++k) sum += R_D[r * 3 + k] * rm4567[c * 3 + k];
				rm123[r * 3 + c] = sum;
			}

		s_rm2re(rm123, q, "313");

		// 313 欧拉奇异点（q2≈0 或 π）用当前值锚定 q3 //
		if (q[1] < 1e-6) {
			q[0] = q[0] + q[2] - current_input[2];
			q[2] = current_input[2];
		}
		else if (q[1] > PI - 1e-6) {
			q[0] = -(q[2] - q[0] - current_input[2]);
			q[2] = current_input[2];
		}
		else if (which_root & 0x04) {
			// 313 欧拉的另一个解：a+π, 2π-b, c+π //
			q[0] = q[0] > PI ? q[0] - PI : q[0] + PI;
			q[1] = 2 * PI - q[1];
			q[2] = q[2] > PI ? q[2] - PI : q[2] + PI;
		}

		// 归一化到 [-π, π] //
		for (int i = 0; i < 7; ++i) {
			while (q[i] > PI) q[i] -= 2 * PI;
			while (q[i] < -PI) q[i] += 2 * PI;
		}

		s_vc(7, q, input);
		return 0;
	}

	auto SevenAxisInverseKinematicSolver5::allocateMemory()->void{
		InverseKinematicSolver::allocateMemory();

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
	}

	auto SevenAxisInverseKinematicSolver5::kinPos()->int{
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
	auto SevenAxisInverseKinematicSolver5::kinPosPure(const double* output, double* input, std::int64_t which_root, const double* current_input)const->int {
		double ee_pos[17]{}, root_mem[7]{};

		s_pos2pm(imp_->EE->posType(), output, ee_pos);
		ee_pos[16] = output[s_pos_type_size(imp_->EE->posType())];

		constexpr double input_period[7]{
			aris::PI * 2, aris::PI * 2,aris::PI * 2,aris::PI * 2,aris::PI * 2,aris::PI * 2,aris::PI * 2,
		};

		if (current_input == nullptr) {
			double current_input_pos[7];
			for (int i = 0; i < 7; ++i)
				current_input_pos[i] = model()->motionPool()[i].mpInternal();
			return s_ik(7, rootNumber(), &imp_->seven_axis_param, sevenAxisInverse5, which_root, ee_pos, input, root_mem, input_period, current_input_pos);
		}
		else {
			return s_ik(7, rootNumber(), &imp_->seven_axis_param, sevenAxisInverse5, which_root, ee_pos, input, root_mem, input_period, current_input);
		}
	}
	SevenAxisInverseKinematicSolver5::~SevenAxisInverseKinematicSolver5() = default;
	SevenAxisInverseKinematicSolver5::SevenAxisInverseKinematicSolver5() :InverseKinematicSolver(1, 0.0), imp_(new Imp) {
		setWhichRoot(8);
		setRootNumber(8);
	}
	ARIS_DEFINE_BIG_FOUR_CPP(SevenAxisInverseKinematicSolver5);

	ARIS_REGISTRATION{
		aris::core::class_<SevenAxisInverseKinematicSolver5>("SevenAxisInverseKinematicSolver5")
			.inherit<InverseKinematicSolver>()
			;
	}
}
