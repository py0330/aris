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
#include "aris/dynamic/mechanism_ur.hpp"
#include "aris/core/reflection.hpp"

namespace aris::dynamic
{
	struct UrParamLocal {
		// UR的机构有如下特点：
		// 1轴和2轴垂直且交于一点： A点
		// 2轴、3轴、4轴平行
		// 4轴、5轴垂直且交于一点： B点
		// 5轴、6轴垂直且交于一点： C点
		//
		// 定义：
		// 0位     ：5轴与1轴平行，6轴与2轴平行
		// A坐标系 ：位于地面，位置在1轴和2轴的交点，z方向和1轴平行，y轴为0位处2轴方向
		// B点     ：位于L4，在0位处，为4轴和A坐标系xoz平面的交点
		// C点     ：4轴和5轴的交点
		// D坐标系 ：位于L6，位置在5轴和6轴的交点，姿态在0位时和A重合
		// E坐标系 ：末端坐标系，位于L6。
		// 
		// 以下为尺寸变量：
		// d1：2轴和3轴的距离
		// d2：3轴和4轴的距离
		// d3：C点（4轴5轴的交点）和D点（5轴6轴的交点），到A坐标系xz平面的距离，即 0 位处，C和D在A坐标系内的 y 分量，C和D的连线平行于xz平面
		// d4：D点到B点的距离，注意这里是z轴为正

		double L1;
		double L2;
		double W1;
		double H2;

		double pm_A_in_Ground[16];
		double pm_EE_in_D[16];

		// 驱动在零位处的偏移，以及系数
		double mp_offset[6];// mp_real = (mp_theoretical - mp_offset) * mp_factor
		double mp_factor[6];
	};

	// UR的机构有如下特点：
	// 1轴和2轴垂直且交于一点： A点
	// 2轴、3轴、4轴平行
	// 4轴、5轴垂直且交于一点： B点
	// 5轴、6轴垂直且交于一点： C点
	//                                           x    EE
	//                                           ^ z
	//                                         \ |/
	//                                         . *----> y
	//                                       W2 /
	//                                       . /
	//                                      \ /
	//                                   --- o y6
	//                                     . *
	//                                    H2 *
	//                                     . *
	//                                   --\ | z5
	//                                    . * 
	//                                  W1 * 
	//                                  . * 
	//        y2                       \ * 
	//   ---  o *** L1 *** o *** L2 *** o  
	//    .   |
	//    .   z1           y3           y4 
	//    H1
	//    .   z
	//    .   ^ y
	//    .   |/
	//   ---  *----> x
	//       O
	auto inverseUr(const UrParamLocal &param, const double*ee_pm, int which_root, double*input)->int {
		const double &L1 = param.L1;
		const double &L2 = param.L2;
		const double &W1 = param.W1;
		const double &H2 = param.H2;

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
		if (W1 > std::sqrt(D_in_A[3] * D_in_A[3] + D_in_A[7] * D_in_A[7])) return false;
		if (which_root & 0x04) {
			q[0] = PI + std::atan2(D_in_A[7], D_in_A[3]) + std::asin(W1 / std::sqrt(D_in_A[3] * D_in_A[3] + D_in_A[7] * D_in_A[7]));
		}
		else {
			q[0] = std::atan2(D_in_A[7], D_in_A[3]) - std::asin(W1 / std::sqrt(D_in_A[3] * D_in_A[3] + D_in_A[7] * D_in_A[7]));
		}


		// 开始求5，6轴 //
		// 事实上这里也可以有2组解
		double R1_pm[16];
		double R23456_pm[16], R23456_pe[6];

		s_pe2pm(std::array<double, 6>{0, 0, 0, q[0], 0, 0}.data(), R1_pm, "321");
		s_inv_pm_dot_pm(R1_pm, D_in_A, R23456_pm);
		s_pm2pe(R23456_pm, R23456_pe, "232");
		if (std::abs(R23456_pe[4] - 0) < 1e-10) // 为了去除奇异点
		{
			R23456_pe[5] = (R23456_pe[3] + R23456_pe[5]) > PI ? R23456_pe[3] + R23456_pe[5] - 2 * PI : R23456_pe[3] + R23456_pe[5];
			R23456_pe[3] = 0.0;
		}
		// 选根
		if (which_root & 0x02)
		{
			R23456_pe[3] = R23456_pe[3] > PI ? R23456_pe[3] - PI : R23456_pe[3] + PI;
			R23456_pe[4] = 2 * PI - R23456_pe[4];
			R23456_pe[5] = R23456_pe[5] > PI ? R23456_pe[5] - PI : R23456_pe[5] + PI;
		}
		q[4] = R23456_pe[4];
		q[5] = R23456_pe[5];


		// 开始求2，3，4轴 // 
		double R1234_pm[16], B_in_A[3];
		s_pe2pm(std::array<double, 6>{0, 0, 0, q[0], R23456_pe[3], 0.0}.data(), R1234_pm, "321");
		s_vc(3, D_in_A + 3, 4, B_in_A, 1);
		s_va(3, -H2, R1234_pm + 2, 4, B_in_A, 1);
		s_va(3, -W1, R1234_pm + 1, 4, B_in_A, 1);

		double B_pos[3];
		s_inv_pm_dot_v3(R1_pm, B_in_A, B_pos);

		double l_square = B_pos[0] * B_pos[0] + B_pos[2] * B_pos[2];
		double l = std::sqrt(l_square);
		if (l > (L1 + L2) || l < (std::max(std::abs(L1), std::abs(L2)) - std::min(std::abs(L1), std::abs(L2))))return false;


		if (which_root & 0x01)
		{
			q[2] = PI + std::acos((L1 * L1 + L2 * L2 - l_square) / (2 * L1 * L2));
			q[1] = std::acos((l_square + L1 * L1 - L2 * L2) / (2 * l * L1)) - std::atan2(B_pos[2], B_pos[0]);
			q[3] = R23456_pe[3] - q[1] - q[2];
		}
		else
		{
			q[2] = PI - std::acos((L1 * L1 + L2 * L2 - l_square) / (2 * L1 * L2));
			q[1] = -std::acos((l_square + L1 * L1 - L2 * L2) / (2 * l * L1)) - std::atan2(B_pos[2], B_pos[0]);
			q[3] = R23456_pe[3] - q[1] - q[2];
		}

		// 这里让每个电机角度都落在[0，2pi]
		for (Size i = 0; i < 6; ++i) {
			q[i] = std::fmod(q[i], 2.0 * PI);
		}

		// 添加所有的偏移 //
		for (int i = 0; i < 6; ++i) {
			q[i] -= offset[i];
			q[i] *= factor[i];

			q[i] = std::fmod(q[i], 2.0 * PI);
			if (q[i] > PI) q[i] -= 2 * PI;
			if (q[i] < -PI) q[i] += 2 * PI;
		}

		// 将q copy到input中
		s_vc(6, q, input);
		return true;
	}

	struct UrInverseKinematicSolver::Imp{
		UrParamLocal puma_param;
		union{
			struct { Part* GR, *L1, *L2, *L3, *L4, *L5, *L6; };
			Part* parts[7]{ nullptr };
		};
		union{
			struct { RevoluteJoint *R1, *R2, *R3, *R4, *R5, *R6; };
			RevoluteJoint* joints[6]{ nullptr };
		};
		union{
			struct { Motion *M1, *M2, *M3, *M4, *M5, *M6; };
			Motion* motions[6]{ nullptr };
		};
		GeneralMotion *EE{ nullptr };
	};
	auto UrInverseKinematicSolver::allocateMemory()->void {
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

		imp_->EE = dynamic_cast<GeneralMotion*>(&model()->generalMotionPool().at(0));

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
		auto ee_mak_on_GR = &imp_->EE->makI()->fatherPart() == imp_->GR ? imp_->EE->makI() : imp_->EE->makJ();
		auto ee_mak_on_L6 = &imp_->EE->makI()->fatherPart() == imp_->L6 ? imp_->EE->makI() : imp_->EE->makJ();

		auto &param = imp_->puma_param;

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

		// get D pm //
		{
			// 构建D坐标系相对于R6 mak的坐标系，零位下方向与A重合：z轴与R5的z轴重合，y轴与R6的z轴重合
			// 它的xyz坐标和R5、R6轴的交点重合
			//
			// 获得R5相对于R6的位姿矩阵
			double pm[16];
			R5_mak_on_L5->getPm(*R6_mak_on_L5, pm);
			double pm_D_in_R6[16]{ 0,0,0,0, 0,0,0,0, 0,1,0,pm[11], 0,0,0,1 };
			s_vc(3, pm + 2, 4, pm_D_in_R6 + 2, 4);
			s_c3(pm_D_in_R6 + 1, 4, pm_D_in_R6 + 2, 4, pm_D_in_R6, 4);

			// 把 D_in_R6 换算出 pm_EE_in_D
			ee_mak_on_L6->getPm(*R6_mak_on_L6, pm);
			s_inv_pm_dot_pm(pm_D_in_R6, pm, imp_->puma_param.pm_EE_in_D);
		}

		// get L1 //
		{
			double pm[16];
			R3_mak_on_L2->getPm(*R2_mak_on_L2, pm);
			imp_->puma_param.L1 = std::sqrt(pm[3] * pm[3] + pm[7] * pm[7]);
		}

		// get L2 //
		{
			double pm[16];
			R4_mak_on_L3->getPm(*R3_mak_on_L3, pm);
			imp_->puma_param.L2 = std::sqrt(pm[3] * pm[3] + pm[7] * pm[7]);
		}

		// get W1 //
		{
			imp_->puma_param.W1 = 0.0;

			// R3相对于R2的偏移 //
			double pm[16];
			R3_mak_on_L2->getPm(*R2_mak_on_L2, pm);
			imp_->puma_param.W1 += pm[11];

			// R4相对于R3的偏移，ratio在考虑R2、R3方向有相反的可能 //
			auto ratio = pm[10] > 0 ? 1.0 : -1.0;
			R4_mak_on_L3->getPm(*R3_mak_on_L3, pm);
			imp_->puma_param.W1 += pm[11] * ratio;

			// R5相对于R4的偏移，ratio在考虑R2、R3方向有相反的可能 //
			ratio *= pm[10] > 0 ? 1.0 : -1.0;
			R5_mak_on_L4->getPm(*R4_mak_on_L4, pm);
			imp_->puma_param.W1 += pm[11] * ratio;
		}

		// get H2 //
		{
			imp_->puma_param.H2 = 0.0;

			// R6相对于R5z的偏移 //
			double pm[16];
			R6_mak_on_L5->getPm(*R5_mak_on_L5, pm);
			imp_->puma_param.H2 += pm[11];

			// R4相对于R5z的偏移 //
			R4_mak_on_L4->getPm(*R5_mak_on_L4, pm);
			imp_->puma_param.H2 -= pm[11];
		}

		// get mp_offset and mp_factor //
		{
			auto &param = imp_->puma_param;

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

			// mp_offset[2] 应该能让R3把R2和R4转到同一条直线上
			double R4_pp_in_R3[3], R2_pp_in_R3[3];
			R2_mak_on_L2->getPm(*R3_mak_on_L2, pm);
			s_vc(3, pm + 3, 4, R2_pp_in_R3, 1);
			R4_mak_on_L3->getPm(*R3_mak_on_L3, pm);
			s_vc(3, pm + 3, 4, R4_pp_in_R3, 1);
			imp_->puma_param.mp_offset[2] = -(std::atan2(R2_pp_in_R3[1], R2_pp_in_R3[0]) + aris::PI - std::atan2(R4_pp_in_R3[1], R4_pp_in_R3[0]));
			imp_->puma_param.mp_factor[2] = R3_mak_on_L3 == imp_->R3->makI() ? 1.0 : -1.0;

			// 看R2和R3是否反向
			double is_23axes_the_same;
			R2_mak_on_L2->getPm(*R3_mak_on_L2, pm);
			is_23axes_the_same = pm[10] > 0.0 ? 1.0 : -1.0;
			imp_->puma_param.mp_factor[2] *= is_23axes_the_same;

			// mp_offset[3] 应该能让R5的z轴在零位处和R1的z轴平行
			R1_mak_on_L1->getPm(*R2_mak_on_L1, pm);
			double R1_z_in_R2[3]{ pm[2], pm[6], pm[10] };

			R2_mak_on_L2->getPm(*R3_mak_on_L2, pm);
			double R1_z_in_R3_raw[3];
			s_pm_dot_v3(pm, R1_z_in_R2, R1_z_in_R3_raw);

			// 还需考虑 R2 R3 需要自转到零位，所带来的额外的转动 //
			double rm_R12[9];
			s_rmz(param.mp_factor[1] * param.mp_offset[1] + param.mp_factor[2] * param.mp_offset[2], rm_R12);
			double R1_z_in_R3[3];
			s_mm(3, 1, 3, rm_R12, R1_z_in_R3_raw, R1_z_in_R3);

			double R1_z_in_R4[3];
			R3_mak_on_L3->getPm(*R4_mak_on_L3, pm);
			s_pm_dot_v3(pm, R1_z_in_R3, R1_z_in_R4);

			R5_mak_on_L4->getPm(*R4_mak_on_L4, pm);
			double R5_z_in_R4[3]{ pm[2], pm[6], pm[10] };

			imp_->puma_param.mp_offset[3] = -std::atan2(R1_z_in_R4[1], R1_z_in_R4[0]) + std::atan2(R5_z_in_R4[1], R5_z_in_R4[0]);
			imp_->puma_param.mp_factor[3] = R4_mak_on_L4 == imp_->R4->makI() ? 1.0 : -1.0;

			// 看R3和R4是否反向
			double is_34axes_the_same;
			R3_mak_on_L3->getPm(*R4_mak_on_L3, pm);
			is_34axes_the_same = pm[10] > 0.0 ? 1.0 : -1.0;
			imp_->puma_param.mp_factor[3] *= is_23axes_the_same * is_34axes_the_same;

			// mp_offset[4] 应该能让R6的z轴转到R4的z轴上，但方向应考虑R4有可能和R2不同向的情况
			double R6_z_axis_in_R5[3], R4_z_axis_in_R5[3];
			R6_mak_on_L5->getPm(*R5_mak_on_L5, pm);
			s_vc(3, pm + 2, 4, R6_z_axis_in_R5, 1);
			R4_mak_on_L4->getPm(*R5_mak_on_L4, pm);
			s_vc(3, is_23axes_the_same * is_34axes_the_same, pm + 2, 4, R4_z_axis_in_R5, 1);
			s = R4_z_axis_in_R5[0] * R6_z_axis_in_R5[1] - R4_z_axis_in_R5[1] * R6_z_axis_in_R5[0];// x cross R2R3
			c = R4_z_axis_in_R5[0] * R6_z_axis_in_R5[0] + R4_z_axis_in_R5[1] * R6_z_axis_in_R5[1];

			imp_->puma_param.mp_offset[4] = atan2(s, c);
			imp_->puma_param.mp_factor[4] = R5_mak_on_L5 == imp_->R5->makI() ? 1.0 : -1.0;

			// mp_offset[5] 应该能让R5的z轴转到垂直于R3R4连线上
			imp_->puma_param.mp_offset[5] = 0.0;
			imp_->puma_param.mp_factor[5] = R6_mak_on_L6 == imp_->R6->makI() ? 1.0 : -1.0;
		}
	}
	auto UrInverseKinematicSolver::kinPos()->int {
		double output_pos[16], input_pos[6], current_input_pos[6];
		model()->getOutputPos(output_pos);
		model()->getInputPos(current_input_pos);

		if (auto ret = kinPosPure(output_pos, input_pos, whichRoot()))
			return ret;
		
		// 设置所有杆件位置 //
		for (aris::Size i = 0; i < 6; ++i) {
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
		for (aris::Size i = 0; i < 6; ++i) {
			imp_->motions[i]->setMpInternal(input_pos[i]);
		}
	};
	auto UrInverseKinematicSolver::kinPosPure(const double* output, double* input, int which_root)->int {
		double current_input_pos[6]{}, ee_pos[16]{}, root_mem[6]{};

		switch (imp_->EE->poseType()) {
		case GeneralMotion::PoseType::EULER123:s_pe2pm(output, ee_pos, "123"); break;
		case GeneralMotion::PoseType::EULER321:s_pe2pm(output, ee_pos, "321"); break;
		case GeneralMotion::PoseType::EULER313:s_pe2pm(output, ee_pos, "313"); break;
		case GeneralMotion::PoseType::QUATERNION:s_pq2pm(output, ee_pos); break;
		case GeneralMotion::PoseType::POSE_MATRIX:s_vc(16, output, ee_pos); break;
		}

		const double input_period[6]{
			aris::PI * 2, aris::PI * 2,aris::PI * 2,aris::PI * 2,aris::PI * 2,aris::PI * 2,
		};

		for (int i = 0; i < 6; ++i)
			current_input_pos[i] = model()->motionPool()[i].mpInternal();

		auto ik = [this](const double* ee_pos, int which_root, double* input)->int {
			return inverseUr(this->imp_->puma_param, ee_pos, which_root, input);
		};

		return s_ik(6, rootNumber(), ik, which_root, ee_pos, input, root_mem, input_period, current_input_pos);

	}
	UrInverseKinematicSolver::~UrInverseKinematicSolver() = default;
	UrInverseKinematicSolver::UrInverseKinematicSolver() :InverseKinematicSolver(1, 0.0), imp_(new Imp) {
		setWhichRoot(8);
		setRootNumber(8);
	}
	ARIS_DEFINE_BIG_FOUR_CPP(UrInverseKinematicSolver);

	auto createModelUr(const UrParam &param)->std::unique_ptr<aris::dynamic::Model> {
		std::unique_ptr<aris::dynamic::Model> model = std::make_unique<aris::dynamic::Model>();

		// 设置重力 //
		const double gravity[6]{ 0.0,0.0,-9.8,0.0,0.0,0.0 };
		model->environment().setGravity(gravity);

		// compute ee info //
		double ee_j_pm[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
		double ee_i_pm[16]{
			0,1,0,0,
			0,0,1,param.W1 + param.W2,
			1,0,0,param.L1 + param.L2 + param.H1 + param.H2,
			0,0,0,1 };

		// add parts //
		const double default_iv[10]{ 1,0,0,0,0,0,0,0,0,0 };
		auto &p1 = model->partPool().add<Part>("L1", param.iv_vec.size() == 6 ? param.iv_vec[0].data() : default_iv);
		auto &p2 = model->partPool().add<Part>("L2", param.iv_vec.size() == 6 ? param.iv_vec[1].data() : default_iv);
		auto &p3 = model->partPool().add<Part>("L3", param.iv_vec.size() == 6 ? param.iv_vec[2].data() : default_iv);
		auto &p4 = model->partPool().add<Part>("L4", param.iv_vec.size() == 6 ? param.iv_vec[3].data() : default_iv);
		auto &p5 = model->partPool().add<Part>("L5", param.iv_vec.size() == 6 ? param.iv_vec[4].data() : default_iv);
		auto &p6 = model->partPool().add<Part>("L6", param.iv_vec.size() == 6 ? param.iv_vec[5].data() : default_iv,
			ee_i_pm);

		// add joints //
		//const double j1_pos[3]{      0.0,                 0.0, param.H1 };
		//const double j2_pos[3]{      0.0,                 0.0, param.H1 };
		//const double j3_pos[3]{ param.L1,                 0.0, param.H1 + 0.1 };
		//const double j4_pos[3]{ param.L1 + param.L2,      0.0, param.H1 };
		//const double j5_pos[3]{ param.L1 + param.L2, param.W1, param.H1 };
		//const double j6_pos[3]{ param.L1 + param.L2, param.W1, param.H1 + param.H2 };

		const double j1_pos[3]{ 0.0,      0.0, param.H1 };
		const double j2_pos[3]{ 0.0,      0.0, param.H1 };
		const double j3_pos[3]{ 0.0,      0.0, param.H1 + param.L1 };
		const double j4_pos[3]{ 0.0,      0.0, param.H1 + param.L1 + param.L2 };
		const double j5_pos[3]{ 0.0, param.W1, param.H1 };
		const double j6_pos[3]{ 0.0, param.W1, param.H1 + param.L1 + param.L2 + param.H2 };

		const double j1_axis[6]{ 0.0, 0.0, 1.0 };
		const double j2_axis[6]{ 0.0, 1.0, 0.0 };
		const double j3_axis[6]{ 0.0, 1.0, 0.0 };
		const double j4_axis[6]{ 0.0, 1.0, 0.0 };
		const double j5_axis[6]{ 0.0, 0.0, 1.0 };
		const double j6_axis[6]{ 0.0, 1.0, 0.0 };

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
		auto &makI = p6.addMarker("tool0");
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
		for (int i = 1; i < 17; ++i) {
			p6.addMarker("tool" + std::to_string(i));
		}
		for (int i = 1; i < 33; ++i) model->ground().markerPool().add<aris::dynamic::Marker>("wobj" + std::to_string(i), ee_j_pm);

		// add solver
		auto &inverse_kinematic = model->solverPool().add<aris::dynamic::UrInverseKinematicSolver>();
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

	ARIS_REGISTRATION{
		aris::core::class_<UrInverseKinematicSolver>("UrInverseKinematicSolver")
			.inherit<InverseKinematicSolver>()
			;
	}
}
