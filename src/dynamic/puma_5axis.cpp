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
#include "aris/dynamic/puma_5axis.hpp"

namespace aris::dynamic
{
	auto createModelPuma5(const PumaParam &param)->std::unique_ptr<aris::dynamic::Model>
	{
		std::unique_ptr<aris::dynamic::Model> model = std::make_unique<aris::dynamic::Model>();

		// 设置重力 //
		const double gravity[6]{ 0.0,0.0,-9.8,0.0,0.0,0.0 };
		model->environment().setGravity(gravity);

		// add part //
		const double default_iv[10]{ 1,0,0,0,0,0,0,0,0,0 };
		auto &p1 = model->partPool().add<Part>("L1", param.iv_vec.size() == 6 ? param.iv_vec[0].data() : default_iv);
		auto &p2 = model->partPool().add<Part>("L2", param.iv_vec.size() == 6 ? param.iv_vec[1].data() : default_iv);
		auto &p3 = model->partPool().add<Part>("L3", param.iv_vec.size() == 6 ? param.iv_vec[2].data() : default_iv);
		auto &p4 = model->partPool().add<Part>("L4", param.iv_vec.size() == 6 ? param.iv_vec[3].data() : default_iv);
		auto &p5 = model->partPool().add<Part>("L5", param.iv_vec.size() == 6 ? param.iv_vec[4].data() : default_iv);
		auto &p6 = model->partPool().add<Part>("L6", param.iv_vec.size() == 6 ? param.iv_vec[5].data() : default_iv);

		// add joint //
		const double j1_pos[3]{ 0.0,                 0.0, param.d1 };
		const double j2_pos[3]{ param.a1,                 0.0, param.d1 };
		const double j3_pos[3]{ param.a1,                 0.0, param.d1 + param.a2 };
		const double j4_pos[3]{ param.a1 + param.d4, param.d3, param.d1 + param.a2 + param.a3 };
		const double j5_pos[3]{ param.a1 + param.d4, param.d3, param.d1 + param.a2 + param.a3 };
		const double j6_pos[3]{ param.a1 + param.d4, param.d3, param.d1 + param.a2 + param.a3 };

		const double j1_axis[6]{ 0.0, 0.0, 1.0 };
		const double j2_axis[6]{ 0.0, 1.0, 0.0 };
		const double j3_axis[6]{ 0.0, 1.0, 0.0 };
		const double j4_axis[6]{ 1.0, 0.0, 0.0 };
		const double j5_axis[6]{ 0.0, 1.0, 0.0 };
		const double j6_axis[6]{ 1.0, 0.0, 0.0 };

		auto &j1 = model->addRevoluteJoint(p1, model->ground(), j1_pos, j1_axis);
		auto &j2 = model->addRevoluteJoint(p2, p1, j2_pos, j2_axis);
		auto &j3 = model->addRevoluteJoint(p3, p2, j3_pos, j3_axis);
		auto &j4 = model->addRevoluteJoint(p4, p3, j4_pos, j4_axis);
		auto &j5 = model->addRevoluteJoint(p5, p4, j5_pos, j5_axis);
		auto &j6 = model->addRevoluteJoint(p6, p5, j6_pos, j6_axis);

		// add actuation //
		auto &m1 = model->addMotion(j1);
		auto &m2 = model->addMotion(j2);
		auto &m3 = model->addMotion(j3);
		auto &m4 = model->addMotion(j4);
		auto &m5 = model->addMotion(j5);
		auto &m6 = model->addMotion(j6);

		const double default_mot_frc[3]{ 0.0, 0.0, 0.0 };

		m1.setFrcCoe(param.mot_frc_vec.size() == 6 ? param.mot_frc_vec[0].data() : default_mot_frc);
		m2.setFrcCoe(param.mot_frc_vec.size() == 6 ? param.mot_frc_vec[1].data() : default_mot_frc);
		m3.setFrcCoe(param.mot_frc_vec.size() == 6 ? param.mot_frc_vec[2].data() : default_mot_frc);
		m4.setFrcCoe(param.mot_frc_vec.size() == 6 ? param.mot_frc_vec[3].data() : default_mot_frc);
		m5.setFrcCoe(param.mot_frc_vec.size() == 6 ? param.mot_frc_vec[4].data() : default_mot_frc);
		m6.setFrcCoe(param.mot_frc_vec.size() == 6 ? param.mot_frc_vec[5].data() : default_mot_frc);

		// add ee general motion //
		const double axis_6_pe[]{ param.a1 + param.d4, param.d3, param.d1 + param.a2 + param.a3, 0.0, aris::PI / 2.0 ,0.0 };
		double axis_6_pm[16];
		double ee_i_pm[16], ee_i_wrt_axis_6_pm[16];
		double ee_j_pm[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };

		s_pe2pm(axis_6_pe, axis_6_pm, "321");
		s_pe2pm(param.tool0_pe, ee_i_wrt_axis_6_pm, param.tool0_pe_type.empty() ? "321" : param.tool0_pe_type.c_str());
		s_pm2pm(axis_6_pm, ee_i_wrt_axis_6_pm, ee_i_pm);

		auto &makI = p5.addMarker("tool0", ee_i_pm);
		auto &makJ = model->ground().addMarker("wobj0", ee_j_pm);
		model->variablePool().add<aris::dynamic::MatrixVariable>("tool0_axis_home", aris::core::Matrix(1, 6, 0.0));
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
		j1.makJ()->setPrtPm(s_pm_dot_pm(robot_pm, *j1.makJ()->prtPm()));
		ee.makJ()->setPrtPm(s_pm_dot_pm(robot_pm, *ee.makJ()->prtPm()));

		// add tools and wobj //
		for (int i = 1; i < 17; ++i)
		{
			p5.addMarker("tool" + std::to_string(i), ee_i_pm);
		}
		for (int i = 1; i < 33; ++i) model->ground().markerPool().add<aris::dynamic::Marker>("wobj" + std::to_string(i), ee_j_pm);

		// add solver
		auto &inverse_kinematic = model->solverPool().add<aris::dynamic::Puma5InverseKinematicSolver>();
		auto &forward_kinematic = model->solverPool().add<ForwardKinematicSolver>();
		auto &inverse_dynamic = model->solverPool().add<aris::dynamic::InverseDynamicSolver>();
		auto &forward_dynamic = model->solverPool().add<aris::dynamic::ForwardDynamicSolver>();
		inverse_kinematic.setWhichRoot(8);

		// make topology correct // 
		for (auto &m : model->motionPool())m.activate(true);
		for (auto &gm : model->generalMotionPool())gm.activate(false);
		for (auto &f : model->forcePool())f.activate(false);

		model->init();

		return model;
	}

	struct Puma5ParamLocal
	{
		// puma机器人构型：
		//                                x            y     x
		//  ---                     ---   - [***d5***] o     -                                                    
		//   *                       *
		//   d4(y方向)        (z方向)d3
		//  --- | [***d1***] o [***d2***] o   
		//      z            y            y           
		// 
		// 其中：
		// |为z方向转动
		// o为y方向转动
		// -为x方向转动
		//
		//
		//  A 坐标系为 1 轴和 2 轴的交点， z 轴和 1 轴平行， y 轴和 2 轴平行
		//  D 坐标系为 4 5 6 三根轴的交点，零位下与 A 坐标系方向一致
		//
		//
		// puma的各个参数定义如下
		// A坐标系：
		//      z轴和R1的z轴一致
		//      R1角度为0时，它的y轴和R2转动的z轴一致
		//
		// d1为A的z轴到R2 转动轴的距离，可能为负
		//
		// d2为R2 R3转轴的距离，必定为正
		//
		// d3、d4、d5可能为负
		//
		// D坐标系起始位于后三轴交点，方向与A一样
		// 
		// 
		// 计算的零位有如下特征：
		//  2 轴位于A坐标系的x轴上，并且其转动方向朝向 A 的 y 轴正方向
		//  3 轴位于A坐标系的x轴上，并且相对2轴来说，它在x轴正方向。亦即R2 R3 是 A 的 x 轴正方向
		//  4 轴的转动方向为 A 的 x 轴正方向 
		//  5 轴的转动方向为 A 的 y 轴正方向 
		//  6 轴的转动方向为 A 的 x 轴正方向 
		//
		// 上述零位与真实机器人的零位不一样，还需调用generate函数来计算
		// 
		double d1;
		double d2;
		double d3;
		double d4;
		double d5;

		double pm_A_in_Ground[16];
		double pm_EE_in_D[16];

		// 驱动在零位处的偏移，以及系数
		double mp_offset[6];// mp_real = (mp_theoretical - mp_offset) * mp_factor
		double mp_factor[6];
	};
	auto puma5Inverse(const Puma5ParamLocal &param, const double *ee_pm, int which_root, double *input)->bool
	{
		const double &d1 = param.d1;
		const double &d2 = param.d2;
		const double &d3 = param.d3;
		const double &d4 = param.d4;
		const double &d5 = param.d5;

		const double *A_pm = param.pm_A_in_Ground;
		const double *E_pm_in_D = param.pm_EE_in_D;

		const double *offset = param.mp_offset;
		const double *factor = param.mp_factor;

		double q[6]{ 0 };

		// 将末端设置成D坐标系，同时得到它在A中的表达 //
		double E_in_A[16];
		s_inv_pm_dot_pm(A_pm, ee_pm, E_in_A);
		double D_in_A[16];
		s_pm_dot_inv_pm(E_in_A, E_pm_in_D, D_in_A);

		// 开始求1轴 //
		// 求第一根轴的位置，这里末端可能工作空间以外，此时末端离原点过近，判断方法为查看以下if //
		// 事实上这里可以有2个解
		if (std::abs(d4) > std::sqrt(D_in_A[3] * D_in_A[3] + D_in_A[7] * D_in_A[7])) return false;//工作空间以外
		if (which_root & 0x04)
		{
			q[0] = std::atan2(D_in_A[7], D_in_A[3]) - std::asin(d4 / std::sqrt(D_in_A[3] * D_in_A[3] + D_in_A[7] * D_in_A[7]));
		}
		else
		{
			q[0] = PI + std::atan2(D_in_A[7], D_in_A[3]) + std::asin(d4 / std::sqrt(D_in_A[3] * D_in_A[3] + D_in_A[7] * D_in_A[7]));
		}

		// 开始求2，3轴 //
		// 事实上这里也有2个解
		double R1_pm[16];
		double R23456_pm[16];

		s_pe2pm(std::array<double, 6>{0, 0, 0, q[0], 0, 0}.data(), R1_pm, "321");
		s_inv_pm_dot_pm(R1_pm, D_in_A, R23456_pm);

		double x = R23456_pm[3];
		double y = R23456_pm[7];
		double z = R23456_pm[11];

		double a1 = std::sqrt((x - d1)*(x - d1) + z * z);
		double a2 = std::sqrt(d3*d3 + d5 * d5);

		if (a1 > (a2 + std::abs(d2)))return false;//工作空间以外
		if (which_root & 0x02)
		{
			q[1] = -std::atan2(z, x - d1) + std::acos((a1*a1 + d2 * d2 - a2 * a2) / (2 * a1*d2));
			q[2] = -(PI - std::acos((a2 * a2 + d2 * d2 - a1 * a1) / (2 * a2*d2))) + std::atan2(d3, d5);
		}
		else
		{
			q[1] = -std::atan2(z, x - d1) - std::acos((a1*a1 + d2 * d2 - a2 * a2) / (2 * a1*d2));
			q[2] = (PI - std::acos((a2 * a2 + d2 * d2 - a1 * a1) / (2 * a2*d2))) + std::atan2(d3, d5);
		}

		// 开始求4,5,6轴 //
		// 事实上这里也有2个解
		double R123_pm[16];
		s_pe2pm(std::array<double, 6>{0, 0, 0, q[0], q[1] + q[2], 0}.data(), R123_pm, "321");

		double R456_pm[16], R456_pe[6];
		s_inv_pm_dot_pm(R123_pm, D_in_A, R456_pm);

		s_pm2pe(R456_pm, R456_pe, "121");

		if (which_root & 0x01)
		{
			q[3] = R456_pe[3]>PI ? R456_pe[3] - PI : R456_pe[3] + PI;
			q[4] = 2 * PI - R456_pe[4];
			q[5] = R456_pe[5]>PI ? R456_pe[5] - PI : R456_pe[5] + PI;
		}
		else
		{
			q[3] = R456_pe[3];
			q[4] = R456_pe[4];
			q[5] = R456_pe[5];
		}

		// 添加所有的偏移 //
		for (int i = 0; i < 6; ++i)
		{
			q[i] -= offset[i];
			q[i] *= factor[i];

			while (q[i] > PI) q[i] -= 2 * PI;
			while (q[i] < -PI) q[i] += 2 * PI;
		}

		/////////////////////////////////////////////////////////////
		q[5] = 0.0;
		/////////////////////////////////////////////////////////////

		// 将q copy到input中
		s_vc(6, q, input);
		return true;
	}
	struct Puma5InverseKinematicSolver::Imp{
		int which_root_{ 0 };
		Puma5ParamLocal puma_param;
		union
		{
			struct { Part* GR, *L1, *L2, *L3, *L4, *L5, *L6; };
			Part* parts[7];
		};
		union
		{
			struct { RevoluteJoint *R1, *R2, *R3, *R4, *R5, *R6; };
			RevoluteJoint* joints[6];
		};
		union
		{
			struct { Motion *M1, *M2, *M3, *M4, *M5, *M6; };
			Motion* motions[6];
		};
		GeneralMotion *ee;
	};

	auto Puma5InverseKinematicSolver::allocateMemory()->void
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

		imp_->R1 = dynamic_cast<RevoluteJoint*>(&model()->jointPool().at(0));
		imp_->R2 = dynamic_cast<RevoluteJoint*>(&model()->jointPool().at(1));
		imp_->R3 = dynamic_cast<RevoluteJoint*>(&model()->jointPool().at(2));
		imp_->R4 = dynamic_cast<RevoluteJoint*>(&model()->jointPool().at(3));
		imp_->R5 = dynamic_cast<RevoluteJoint*>(&model()->jointPool().at(4));
		imp_->R6 = dynamic_cast<RevoluteJoint*>(&model()->jointPool().at(5));

		imp_->M1 = &model()->motionPool().at(0);
		imp_->M2 = &model()->motionPool().at(1);
		imp_->M3 = &model()->motionPool().at(2);
		imp_->M4 = &model()->motionPool().at(3);
		imp_->M5 = &model()->motionPool().at(4);
		imp_->M6 = &model()->motionPool().at(5);

		imp_->ee = dynamic_cast<GeneralMotion*>(&model()->generalMotionPool().at(0));

		auto R1_mak_on_GR = &imp_->R1->makI()->fatherPart() == imp_->GR ? imp_->R1->makI() : imp_->R1->makJ();
		auto R1_mak_on_L1 = &imp_->R1->makI()->fatherPart() == imp_->L1 ? imp_->R1->makI() : imp_->R1->makJ();
		auto R2_mak_on_L1 = &imp_->R2->makI()->fatherPart() == imp_->L1 ? imp_->R2->makI() : imp_->R2->makJ();
		auto R2_mak_on_L2 = &imp_->R2->makI()->fatherPart() == imp_->L2 ? imp_->R2->makI() : imp_->R2->makJ();
		auto R3_mak_on_L2 = &imp_->R3->makI()->fatherPart() == imp_->L2 ? imp_->R3->makI() : imp_->R3->makJ();
		auto R3_mak_on_L3 = &imp_->R3->makI()->fatherPart() == imp_->L3 ? imp_->R3->makI() : imp_->R3->makJ();
		auto R4_mak_on_L3 = &imp_->R4->makI()->fatherPart() == imp_->L3 ? imp_->R4->makI() : imp_->R4->makJ();
		auto R4_mak_on_L4 = &imp_->R4->makI()->fatherPart() == imp_->L4 ? imp_->R4->makI() : imp_->R4->makJ();
		auto R5_mak_on_L4 = &imp_->R5->makI()->fatherPart() == imp_->L4 ? imp_->R5->makI() : imp_->R5->makJ();
		auto R5_mak_on_L5 = &imp_->R5->makI()->fatherPart() == imp_->L5 ? imp_->R5->makI() : imp_->R5->makJ();
		auto R6_mak_on_L5 = &imp_->R6->makI()->fatherPart() == imp_->L5 ? imp_->R6->makI() : imp_->R6->makJ();
		auto R6_mak_on_L6 = &imp_->R6->makI()->fatherPart() == imp_->L6 ? imp_->R6->makI() : imp_->R6->makJ();
		auto ee_mak_on_GR = &imp_->ee->makI()->fatherPart() == imp_->GR ? imp_->ee->makI() : imp_->ee->makJ();
		auto ee_mak_on_L6 = &imp_->ee->makI()->fatherPart() == imp_->L5 ? imp_->ee->makI() : imp_->ee->makJ();

		// get A pm //
		{
			// 构建A坐标系相对于R1 mak的坐标系，它的y轴是R2在R1下的 z 轴，z轴是R1的z轴[0,0,1],x轴为他俩叉乘
			// 它的xy坐标为0，z坐标为R2在R1 mak下坐标系的z
			//
			// 获得R2相对于R1的位姿矩阵
			double pm[16], pm_A_in_R1[16];
			s_eye(4, pm_A_in_R1);

			R2_mak_on_L1->getPm(*R1_mak_on_L1, pm);
			s_vc(3, pm + 2, 4, pm_A_in_R1 + 1, 4);
			s_c3(pm_A_in_R1 + 1, 4, pm_A_in_R1 + 2, 4, pm_A_in_R1, 4);
			pm_A_in_R1[11] = pm[11];

			// 把 A_in_R1 换算到 A in ground，这里的ground是指ee 的 makJ
			R1_mak_on_GR->getPm(*ee_mak_on_GR, pm);
			s_pm_dot_pm(pm, pm_A_in_R1, imp_->puma_param.pm_A_in_Ground);
		}

		// get d1 //
		{
			// 得到 R2_mak_on_L1 相对于 R1_mak_on_L1 的位置
			double pm[16];
			R2_mak_on_L1->getPm(*R1_mak_on_L1, pm);

			// 求取这个位置分量在 A 坐标系下的 x 分量,首先去掉 z 分量，然后用y轴叉乘它，y 叉 x 得到的 z 分量是它的负值
			pm[11] = 0;//去掉 z 分量
			double pp[3];
			s_c3(pm + 2, 4, pm + 3, 4, pp, 1);

			imp_->puma_param.d1 = -pp[2];
		}

		// get d2 //
		{
			double pm[16];
			R3_mak_on_L2->getPm(*R2_mak_on_L2, pm);
			imp_->puma_param.d2 = std::sqrt(pm[3] * pm[3] + pm[7] * pm[7]);
		}

		// get d3\d4\d5 //
		{
			// 取得4轴和5轴的交点
			double pm[16];
			R5_mak_on_L4->getPm(*R4_mak_on_L4, pm);
			double pp_in_R4_mak[3]{ 0.0,0.0,pm[11] };

			double pp_in_R3_mak[3];
			R4_mak_on_L3->getPm(*R3_mak_on_L3, pm);
			s_pp2pp(pm, pp_in_R4_mak, pp_in_R3_mak);

			// 将原点的偏移叠加到该变量上
			R1_mak_on_L1->getPm(*R2_mak_on_L1, pm);
			double R1_pp_in_R2_mak[3]{ pm[3],pm[7],pm[11] };
			R2_mak_on_L2->getPm(*R3_mak_on_L2, pm);
			double R1_pp_in_R3_mak[3];
			s_pp2pp(pm, R1_pp_in_R2_mak, R1_pp_in_R3_mak);

			pp_in_R3_mak[2] -= R1_pp_in_R3_mak[2];

			// 将方向矫正一下,需要将pp_in_R3_mak 变到如下坐标系: x轴为4轴的转轴z，y轴为2轴的转轴z(因为2轴3轴可能反向，而二轴又定义了A)
			double rm[9];// y轴是0，0，1
			R4_mak_on_L3->getPm(*R3_mak_on_L3, pm);
			s_vc(3, pm + 2, 4, rm, 3);
			R2_mak_on_L2->getPm(*R3_mak_on_L2, pm);
			s_vc(3, pm + 2, 4, rm + 1, 3);
			s_c3(rm, 3, rm + 1, 3, rm + 2, 3);

			double final_pp[3];
			s_mm(3, 1, 3, rm, T(3), pp_in_R3_mak, 1, final_pp, 1);

			imp_->puma_param.d3 = final_pp[2];
			imp_->puma_param.d4 = final_pp[1];
			imp_->puma_param.d5 = final_pp[0];
		}

		// get D pm //
		{
			// 构建D坐标系相对于R6 mak的坐标系，它的 x 轴是R6的 z 轴，y轴是R5的转轴（z轴）
			// 它的xyz坐标和两轴的交点重合
			//
			// 获得R5相对于R6的位姿矩阵
			double pm[16];
			R5_mak_on_L5->getPm(*R6_mak_on_L5, pm);
			double pm_D_in_R6[16]{ 0,0,0,0, 0,0,0,0, 1,0,0,pm[11], 0,0,0,1 };
			s_vc(3, pm + 2, 4, pm_D_in_R6 + 1, 4);
			s_c3(pm_D_in_R6, 4, pm_D_in_R6 + 1, 4, pm_D_in_R6 + 2, 4);

			// 把 D_in_R6 换算出 pm_EE_in_D
			ee_mak_on_L6->getPm(*R6_mak_on_L6, pm);
			s_inv_pm_dot_pm(pm_D_in_R6, pm, imp_->puma_param.pm_EE_in_D);
		}

		// get mp_offset and mp_factor //
		{
			// mp_offset[0] 始终为0，因为A是在关节角度为0时定义出来的
			imp_->puma_param.mp_offset[0] = 0.0;
			imp_->puma_param.mp_factor[0] = R1_mak_on_L1 == imp_->R1->makI() ? 1.0 : -1.0;

			// mp_offset[1] 应该能把R2转到延x轴正向的位置
			// 先把A坐标系的x轴转到R2坐标系下
			double Ax_axis_in_EE[3], Ax_axis_in_R1[3], Ax_axis_in_R2[3];
			double pm[16];
			s_vc(3, imp_->puma_param.pm_A_in_Ground, 4, Ax_axis_in_EE, 1);
			ee_mak_on_GR->getPm(*R1_mak_on_GR, pm);
			s_pm_dot_v3(pm, Ax_axis_in_EE, Ax_axis_in_R1);
			R1_mak_on_L1->getPm(*R2_mak_on_L1, pm);
			s_pm_dot_v3(pm, Ax_axis_in_R1, Ax_axis_in_R2);

			// 再看R2R3连线和以上x轴的夹角
			R3_mak_on_L2->getPm(*R2_mak_on_L2, pm);

			double s = Ax_axis_in_R2[0] * pm[7] - Ax_axis_in_R2[1] * pm[3];// x cross R2R3
			double c = Ax_axis_in_R2[0] * pm[3] + Ax_axis_in_R2[1] * pm[7];

			imp_->puma_param.mp_offset[1] = std::atan2(s, c);
			imp_->puma_param.mp_factor[1] = R2_mak_on_L2 == imp_->R2->makI() ? 1.0 : -1.0;

			// mp_offset[2] 应该能让R3把R4轴线转到R2R3连线方向（x轴）
			double R4_axis_in_R3[3], R4_axis_in_R2[3];
			R4_mak_on_L3->getPm(*R3_mak_on_L3, pm);
			s_vc(3, pm + 2, 4, R4_axis_in_R3, 1);
			R3_mak_on_L2->getPm(*R2_mak_on_L2, pm);
			s_pm_dot_v3(pm, R4_axis_in_R3, R4_axis_in_R2);

			// 获取 R2R3 连线，在R2下
			R3_mak_on_L2->getPm(*R2_mak_on_L2, pm);

			// R4的z轴和 x_axis_in_R3 的夹角就是offset
			s = pm[3] * R4_axis_in_R2[1] - pm[7] * R4_axis_in_R2[0];// x cross R2R3
			c = pm[3] * R4_axis_in_R2[0] + pm[7] * R4_axis_in_R2[1];

			imp_->puma_param.mp_offset[2] = std::atan2(s, c);
			imp_->puma_param.mp_factor[2] = R3_mak_on_L3 == imp_->R3->makI() ? 1.0 : -1.0;

			// 看看2轴和3轴是否反向 //
			bool is_23axes_the_same;
			R2_mak_on_L2->getPm(*R3_mak_on_L2, pm);
			is_23axes_the_same = pm[10] > 0.0 ? true : false;
			imp_->puma_param.mp_factor[2] *= is_23axes_the_same ? 1.0 : -1.0;

			// mp_offset[3] 应该能让R4把R5轴线转到和R2轴一致
			double R2_z_axis_in_R3[3], R2_z_axis_in_R4[3];
			R2_mak_on_L2->getPm(*R3_mak_on_L2, pm);
			s_vc(3, pm + 2, 4, R2_z_axis_in_R3, 1);
			R3_mak_on_L3->getPm(*R4_mak_on_L3, pm);
			s_pm_dot_v3(pm, R2_z_axis_in_R3, R2_z_axis_in_R4);
			R5_mak_on_L4->getPm(*R4_mak_on_L4, pm);

			s = R2_z_axis_in_R4[0] * pm[6] - R2_z_axis_in_R4[1] * pm[2];// x cross R2R3
			c = R2_z_axis_in_R4[0] * pm[2] + R2_z_axis_in_R4[1] * pm[6];

			imp_->puma_param.mp_offset[3] = std::atan2(s, c);
			imp_->puma_param.mp_factor[3] = R4_mak_on_L4 == imp_->R4->makI() ? 1.0 : -1.0;

			// mp_offset[4] 应该能让R5把R6轴线转到和R4轴一致
			double R6_z_axis_in_R5[3], R4_z_axis_in_R5[3];
			R6_mak_on_L5->getPm(*R5_mak_on_L5, pm);
			s_vc(3, pm + 2, 4, R6_z_axis_in_R5, 1);
			R4_mak_on_L4->getPm(*R5_mak_on_L4, pm);
			s_vc(3, pm + 2, 4, R4_z_axis_in_R5, 1);
			s = R4_z_axis_in_R5[0] * R6_z_axis_in_R5[1] - R4_z_axis_in_R5[1] * R6_z_axis_in_R5[0];// x cross R2R3
			c = R4_z_axis_in_R5[0] * R6_z_axis_in_R5[0] + R4_z_axis_in_R5[1] * R6_z_axis_in_R5[1];

			imp_->puma_param.mp_offset[4] = std::atan2(s, c);
			imp_->puma_param.mp_factor[4] = R5_mak_on_L5 == imp_->R5->makI() ? 1.0 : -1.0;

			// mp_offset[5] 可以随便设
			imp_->puma_param.mp_offset[5] = 0.0;
			imp_->puma_param.mp_factor[5] = R6_mak_on_L6 == imp_->R6->makI() ? 1.0 : -1.0;
		}
	}
	auto Puma5InverseKinematicSolver::kinPos()->int
	{
		if (imp_->which_root_ == 8)
		{
			int solution_num = 0;
			double diff_q[8][6];
			double diff_norm[8];

			for (int i = 0; i < 8; ++i)
			{
				if (puma5Inverse(imp_->puma_param, *imp_->ee->mpm(), i, diff_q[solution_num]))
				{
					diff_norm[solution_num] = 0;
					for (int j = 0; j < 6; ++j)
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

			///////////////////////////////////////////////////////////////////////////////////////////////////
			diff_q[real_solution][5] = 0.0;
			///////////////////////////////////////////////////////////////////////////////////////////////////


			for (aris::Size i = 0; i < 6; ++i)
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
			if (double q[6]; puma5Inverse(imp_->puma_param, *imp_->ee->mpm(), imp_->which_root_, q))
			{
				for (aris::Size i = 0; i < 6; ++i)
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
	auto Puma5InverseKinematicSolver::setWhichRoot(int root_of_0_to_7)->void { imp_->which_root_ = root_of_0_to_7; }
	auto Puma5InverseKinematicSolver::whichRoot()const->int { return imp_->which_root_; }
	Puma5InverseKinematicSolver::~Puma5InverseKinematicSolver() = default;
	Puma5InverseKinematicSolver::Puma5InverseKinematicSolver() :InverseKinematicSolver(1, 0.0), imp_(new Imp) {}
	ARIS_DEFINE_BIG_FOUR_CPP(Puma5InverseKinematicSolver);

	ARIS_REGISTRATION{
		aris::core::class_<Puma5InverseKinematicSolver>("Puma5InverseKinematicSolver")
			.inherit<InverseKinematicSolver>()
			.prop("which_root", &Puma5InverseKinematicSolver::setWhichRoot, &Puma5InverseKinematicSolver::whichRoot)
			;
	}
}
