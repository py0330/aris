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
	struct ScaraParamLocal {
		double a;
		double b;

		double pm_A_in_Ground[16];
		double pm_EE_in_D[16];

		// 驱动在零位处的偏移，以及系数，这里主要考虑可能建模时，其零位和系统零位不一致
		double mp_offset[4];// mp_real = (mp_theoretical - mp_offset) * mp_factor
		double mp_factor[4];
	};
	// 不考虑 pitch //
	auto scaraInverse(const ScaraParamLocal &param, const double* ee_xyza, int which_root, double* input)->int {
		const double& a = param.a;
		const double& b = param.b;

		const double* A_pm = param.pm_A_in_Ground;
		const double* E_pm_in_D = param.pm_EE_in_D;
		
		const double* offset = param.mp_offset;
		const double* factor = param.mp_factor;

		// 将末端设置成D坐标系，同时得到它在A中的表达 //
		double ee_pe321[6]{ ee_xyza[0], ee_xyza[1], ee_xyza[2], ee_xyza[3], 0.0, 0.0};
		double ee_pm[16];
		s_pe2pm(ee_pe321, ee_pm, "321");
		
		double E_in_A[16];
		s_inv_pm_dot_pm(A_pm, ee_pm, E_in_A);
		double D_in_A[16];
		s_pm_dot_inv_pm(E_in_A, E_pm_in_D, D_in_A);

		double pe_D_in_A[6];
		s_pm2pe(D_in_A, pe_D_in_A, "321");
		ee_xyza = pe_D_in_A;

		// 开始求解 //
		auto c = std::sqrt(ee_xyza[0] * ee_xyza[0] + ee_xyza[1] * ee_xyza[1]);
		if (c<std::abs(a - b) || c>std::abs(a + b)) return -1;

		input[0] = which_root == 0
			? std::atan2(ee_xyza[1], ee_xyza[0]) - std::acos((a * a + c * c - b * b) / (2 * a * c))
			: std::atan2(ee_xyza[1], ee_xyza[0]) + std::acos((a * a + c * c - b * b) / (2 * a * c));
		input[1] = which_root == 0
			? -std::acos((a * a + b * b - c * c) / (2 * a * b)) + aris::PI
			: std::acos((a * a + b * b - c * c) / (2 * a * b)) - aris::PI;
		input[2] = ee_xyza[2];
		input[3] = ee_xyza[3] - input[1] - input[0];

		// mp_real = (mp_theoretical - mp_offset) * mp_factor
		// 添加所有的偏移 //
		for (int i = 0; i < 4; ++i) {
			input[i] -= offset[i];
			input[i] *= factor[i];
		}

		return 0;
	}

	struct ScaraInverseKinematicSolver::Imp {
		int which_root_{ 2 };
		ScaraParamLocal scara_param;
		union {
			struct { Part* GR, * L1, * L2, * L3, * L4; };
			Part* parts[4]{ nullptr };
		};
		union {
			struct { RevoluteJoint* J1, * J2;  PrismaticJoint* J3; ScrewJoint* J4; };
			Joint* joints[4]{ nullptr };
		};
		union {
			struct { Motion* M1, * M2, * M3, * M4; };
			Motion* motions[4]{ nullptr };
		};
		XyztMotion* EE{ nullptr };
	};
	auto ScaraInverseKinematicSolver::allocateMemory()->void {
		InverseKinematicSolver::allocateMemory();

		auto& param = imp_->scara_param;
		auto GR = imp_->GR = &model()->partPool().at(0);
		auto L1 = imp_->L1 = &model()->partPool().at(1);
		auto L2 = imp_->L2 = &model()->partPool().at(2);
		auto L3 = imp_->L3 = &model()->partPool().at(3);
		auto L4 = imp_->L4 = &model()->partPool().at(4);

		auto J1 = imp_->J1 = dynamic_cast<RevoluteJoint*>(&model()->jointPool().at(0));
		auto J2 = imp_->J2 = dynamic_cast<RevoluteJoint*>(&model()->jointPool().at(1));
		auto J3 = imp_->J3 = dynamic_cast<PrismaticJoint*>(&model()->jointPool().at(2));
		auto J4 = imp_->J4 = dynamic_cast<ScrewJoint*>(&model()->jointPool().at(3));

		auto M1 = imp_->M1 = &model()->motionPool().at(0);
		auto M2 = imp_->M2 = &model()->motionPool().at(1);
		auto M3 = imp_->M3 = &model()->motionPool().at(2);
		auto M4 = imp_->M4 = &model()->motionPool().at(3);

		auto EE = imp_->EE = dynamic_cast<XyztMotion*>(&model()->generalMotionPool().at(0));

		auto J1_mak_on_GR = &J1->makI()->fatherPart() == GR ? J1->makI() : J1->makJ();
		auto J1_mak_on_L1 = &J1->makI()->fatherPart() == L1 ? J1->makI() : J1->makJ();
		auto J2_mak_on_L1 = &J2->makI()->fatherPart() == L1 ? J2->makI() : J2->makJ();
		auto J2_mak_on_L2 = &J2->makI()->fatherPart() == L2 ? J2->makI() : J2->makJ();
		auto J3_mak_on_L2 = &J3->makI()->fatherPart() == L2 ? J3->makI() : J3->makJ();
		auto J3_mak_on_L3 = &J3->makI()->fatherPart() == L3 ? J3->makI() : J3->makJ();
		auto J4_mak_on_L3 = &J4->makI()->fatherPart() == L3 ? J4->makI() : J4->makJ();
		auto J4_mak_on_L4 = &J4->makI()->fatherPart() == L4 ? J4->makI() : J4->makJ();
		auto EE_mak_on_GR = &EE->makI()->fatherPart() == GR ? EE->makI() : EE->makJ();
		auto EE_mak_on_L4 = &EE->makI()->fatherPart() == L4 ? EE->makI() : EE->makJ();

		// get A pm //
		{
			// 构建A坐标系相对于R1 mak的坐标系：
			//     z 轴：是R1的z轴[0,0,1]
			//     x 轴：为垂直于z轴的R1 R2连线
			//     y 轴：z cross x
			// 原点为R1 mak的原点
			//
			// 获得R2相对于R1的位姿矩阵
			double pm[16];
			J2_mak_on_L1->getPm(*J1_mak_on_L1, pm);
			double n = std::sqrt(pm[3] * pm[3] + pm[7] * pm[7]);

			double pm_A_in_R1[16]{
				pm[3]/n, -pm[7]/n, 0, 0,
				pm[7]/n,  pm[3]/n, 0, 0,
				0      ,  0      , 1, 0,
				0      ,  0      , 0, 1,
			};

			// 把 A_in_R1 换算到 A in ground，这里的ground是指ee 的 makJ
			J1_mak_on_GR->getPm(*EE_mak_on_GR, pm);
			s_pm_dot_pm(pm, pm_A_in_R1, imp_->scara_param.pm_A_in_Ground);
		}

		// get D pm //
		{
			// 构建D坐标系相对于R4 mak的坐标系
			//    z 轴是R1的 z 轴
			//    x 轴是R2-R4的连线垂直于z轴的分量
			//    y 轴是 z cross x
			// xyz 坐标：
			//    x & y ：与 R4 坐标系重合
			//    z     ：机器人零位处，沿着z轴，位于 A 坐标系的 xy 平面内
			//

			// 获得J4_mak相对于J2_mak的位姿矩阵
			double pm[16], pm1[16], pm2[16];
			double pm_J4_in_J2[16];
			s_eye(4, pm);
			J3_mak_on_L2->getPm(*J2_mak_on_L2, pm1);
			s_pm_dot_pm(pm, pm1, pm2);
			s_vc(16, pm2, pm);
			J4_mak_on_L3->getPm(*J3_mak_on_L3, pm1);
			s_pm_dot_pm(pm, pm1, pm2);
			s_vc(16, pm2, pm_J4_in_J2);

			// 获得J4_mak相对于J1_mak的位姿矩阵
			double pm_J4_in_J1[16];
			J2_mak_on_L1->getPm(*J2_mak_on_L1, pm1);
			s_pm_dot_pm(pm1, pm_J4_in_J2, pm_J4_in_J1);

			// D in R4
			double n = std::sqrt(pm_J4_in_J2[3] * pm_J4_in_J2[3] + pm_J4_in_J2[7] * pm_J4_in_J2[7]);
			double pm_D_in_R4[16]{
				pm_J4_in_J2[3] / n, -pm_J4_in_J2[7] / n, 0               ,0,
				pm_J4_in_J2[7] / n, pm_J4_in_J2[3] / n , 0               ,0,
				0                 , 0                  , pm_J4_in_J1[10] ,-pm_J4_in_J1[11] * pm_J4_in_J1[10],
				0                 , 0                  , 0               ,1 
			};

			// pm_EE_in_D 
			EE_mak_on_L4->getPm(*J4_mak_on_L4, pm);
			s_inv_pm_dot_pm(pm_D_in_R4, pm, imp_->scara_param.pm_EE_in_D);
		}

		// get dh //
		{
			// 构建A坐标系相对于R1 mak的坐标系：
			//     z 轴：是R1的z轴[0,0,1]
			//     x 轴：为垂直于z轴的R1 R2连线
			//     y 轴：z cross x
			// 原点为R1 mak的原点
			//
			// 获得R2相对于R1的位姿矩阵
			double pm[16], pm1[16], pm2[16];
			J2_mak_on_L1->getPm(*J1_mak_on_L1, pm);
			param.a = std::sqrt(pm[3] * pm[3] + pm[7] * pm[7]);
			
			double pm_J4_in_J2[16];
			s_eye(4, pm);
			J3_mak_on_L2->getPm(*J2_mak_on_L2, pm1);
			s_pm_dot_pm(pm, pm1, pm2);
			s_vc(16, pm2, pm);
			J4_mak_on_L3->getPm(*J3_mak_on_L3, pm1);
			s_pm_dot_pm(pm, pm1, pm2);
			s_vc(16, pm2, pm_J4_in_J2);
			param.b = std::sqrt(pm_J4_in_J2[3] * pm_J4_in_J2[3] + pm_J4_in_J2[7] * pm_J4_in_J2[7]);
		}

		// get mp_offset and mp_factor //
		{
			// mp_factor[0] 跟 makI 位于的杆件相关
			// mp_offset[0] 因为已经设置过 A_in_Ground, 所以这里无需再计算
			double pm[16], pm1[16], pm2[16];
			J2_mak_on_L1->getPm(*J1_mak_on_L1, pm);
			param.mp_factor[0] = J1_mak_on_L1 == J1->makI() ? 1.0 : -1.0;
			param.mp_offset[0] = 0.0;

			// mp_factor[1] 跟 makI 位于的杆件相关，也与R1 与 R2 的轴线是否相同相关
			// mp_offset[1] 可将R1\R2\R4 的位置转到一条直线上
			J2_mak_on_L1->getPm(*J1_mak_on_L1, pm);
			param.mp_factor[1] = J2_mak_on_L2 == J2->makI() ? s_sgn(pm[10]) : -s_sgn(pm[10]);
			double J4_x_in_J2, J4_y_in_J2, J1_x_in_J2, J1_y_in_J2;
			J3_mak_on_L2->getPm(*J2_mak_on_L2, pm1);
			J4_mak_on_L3->getPm(*J3_mak_on_L3, pm2);
			s_pm_dot_pm(pm1, pm2, pm);
			J4_x_in_J2 = pm[3];
			J4_y_in_J2 = pm[7];
			J1_mak_on_L1->getPm(*J2_mak_on_L1, pm);
			J1_x_in_J2 = pm[3];
			J1_y_in_J2 = pm[7];
			param.mp_offset[1] = 
				//(J2_mak_on_L2 == J2->makI() ? 1.0 : -1.0) *
				(std::atan2(J4_y_in_J2, J4_x_in_J2) - std::atan2(-J1_y_in_J2, -J1_x_in_J2));

			// mp_factor[2] 跟 makI 位于的杆件相关，也与R1 与 P3 的轴线是否相同相关
			// mp_offset[2] 为0，因为计算 EE_in_D 的时候，已经是在 J3 转角为零的时候 
			J2_mak_on_L1->getPm(*J1_mak_on_L1, pm1);
			J3_mak_on_L2->getPm(*J2_mak_on_L2, pm2);
			s_pm_dot_pm(pm1, pm2, pm);
			param.mp_factor[2] = J3_mak_on_L3 == J3->makI() ? s_sgn(pm[10]) : -s_sgn(pm[10]);
			param.mp_offset[2] = 0.0;

			// mp_factor[3] 应该和 R1 的z轴保持一致
			// mp_offset[3] 应该为 0，因为计算 EE_in_D的时候已经考虑
			param.mp_factor[3] = 1.0;
			J2_mak_on_L1->getPm(*J1_mak_on_L1, pm);
			param.mp_factor[3] *= s_sgn(pm[10]);
			J3_mak_on_L2->getPm(*J2_mak_on_L2, pm);
			param.mp_factor[3] *= s_sgn(pm[10]);
			J4_mak_on_L3->getPm(*J3_mak_on_L3, pm);
			param.mp_factor[3] *= s_sgn(pm[10]);
			param.mp_factor[3] *= J4_mak_on_L4 == J4->makI() ? 1.0 : -1.0;
			param.mp_offset[3] = 0.0;
		}
	}
	auto ScaraInverseKinematicSolver::kinPos()->int {
		const int ROOT_NUM = 2;
		const int ROOT_SIZE = 4;
		if (imp_->which_root_ == ROOT_NUM) {
			int solution_num = 0;
			double diff_q[ROOT_NUM][ROOT_SIZE];
			double diff_norm[ROOT_NUM];

			for (int i = 0; i < ROOT_NUM; ++i) {
				double output[ROOT_SIZE];
				model()->getOutputPos(output);

				if (scaraInverse(imp_->scara_param, output, i, diff_q[solution_num]) == 0) {
					diff_norm[solution_num] = 0;

					for (auto j : { 0,1,3 }) {
						if (!std::isfinite(model()->motionPool()[j].mpInternal())) {
							model()->motionPool()[j].setMpInternal(0.0);
						}
						diff_q[solution_num][j] = std::fmod(diff_q[solution_num][j] - model()->motionPool()[j].mpInternal(), 2 * PI);

						// 如果是 2 的话，忽略轴的范围，选择最近的可达解 //
						if (diff_q[solution_num][j] > PI) diff_q[solution_num][j] -= 2 * PI;
						if (diff_q[solution_num][j] < -PI)diff_q[solution_num][j] += 2 * PI;

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

			// link1~4 //
			double pe[6]{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
			if (&imp_->J1->makI()->fatherPart() == imp_->L1) {
				pe[5] = model()->motionPool()[0].mpInternal() + diff_q[real_solution][0];
				imp_->J1->makI()->setPe(*imp_->J1->makJ(), pe, "123");
			}
			else {
				pe[5] = -model()->motionPool()[0].mpInternal() - diff_q[real_solution][0];
				imp_->J1->makJ()->setPe(*imp_->J1->makI(), pe, "123");
			}

			if (&imp_->J2->makI()->fatherPart() == imp_->L2) {
				pe[5] = model()->motionPool()[1].mpInternal() + diff_q[real_solution][1];
				imp_->J2->makI()->setPe(*imp_->J2->makJ(), pe, "123");
			}
			else {
				pe[5] = -model()->motionPool()[1].mpInternal() - diff_q[real_solution][1];
				imp_->J2->makJ()->setPe(*imp_->J2->makI(), pe, "123");
			}

			pe[5] = 0.0;
			if (&imp_->J3->makI()->fatherPart() == imp_->L3) {
				pe[2] = model()->motionPool()[2].mpInternal() + diff_q[real_solution][2];
				imp_->J3->makI()->setPe(*imp_->J3->makJ(), pe, "123");
			}
			else {
				pe[2] = -model()->motionPool()[2].mpInternal() - diff_q[real_solution][2];
				imp_->J3->makJ()->setPe(*imp_->J3->makI(), pe, "123");
			}
			
			if (&imp_->J4->makI()->fatherPart() == imp_->L4) {
				pe[2] = (model()->motionPool()[3].mpInternal() + diff_q[real_solution][3]) / 2 / PI
					* dynamic_cast<ScrewJoint&>(model()->jointPool()[3]).pitch();
				pe[5] = model()->motionPool()[3].mpInternal() + diff_q[real_solution][3];
				imp_->J4->makI()->setPe(*imp_->J4->makJ(), pe, "123");
			}
			else {
				pe[2] = -(model()->motionPool()[3].mpInternal() + diff_q[real_solution][3]) / 2 / PI
					* dynamic_cast<ScrewJoint&>(model()->jointPool()[3]).pitch();
				pe[5] = -model()->motionPool()[3].mpInternal() - diff_q[real_solution][3];
				imp_->J4->makJ()->setPe(*imp_->J4->makI(), pe, "123");
			}

			for (auto& m : model()->motionPool()) m.updP();
			return 0;
		}
		else {
			double input[4], output[4];
			model()->getOutputPos(output);
			if (auto ret = scaraInverse(imp_->scara_param, output, imp_->which_root_, input))
				return ret;

			// link1~4 //
			double pe[6]{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
			if (&imp_->J1->makI()->fatherPart() == imp_->L1) {
				pe[5] = input[0];
				imp_->J1->makI()->setPe(*imp_->J1->makJ(), pe, "123");
			}
			else {
				pe[5] = -input[0];
				imp_->J1->makJ()->setPe(*imp_->J1->makI(), pe, "123");
			}

			if (&imp_->J2->makI()->fatherPart() == imp_->L2) {
				pe[5] = input[1];
				imp_->J2->makI()->setPe(*imp_->J2->makJ(), pe, "123");
			}
			else {
				pe[5] = -input[1];
				imp_->J2->makJ()->setPe(*imp_->J2->makI(), pe, "123");
			}

			pe[5] = 0.0;
			if (&imp_->J3->makI()->fatherPart() == imp_->L3) {
				pe[2] = input[2];
				imp_->J3->makI()->setPe(*imp_->J3->makJ(), pe, "123");
			}
			else {
				pe[2] = -input[2];
				imp_->J3->makJ()->setPe(*imp_->J3->makI(), pe, "123");
			}

			if (&imp_->J4->makI()->fatherPart() == imp_->L4) {
				pe[2] = input[3] / 2 / PI
					* dynamic_cast<ScrewJoint&>(model()->jointPool()[3]).pitch();
				pe[5] = input[3];
				imp_->J4->makI()->setPe(*imp_->J4->makJ(), pe, "123");
			}
			else {
				pe[2] = -input[3] / 2 / PI
					* dynamic_cast<ScrewJoint&>(model()->jointPool()[3]).pitch();
				pe[5] = -input[3];
				imp_->J4->makJ()->setPe(*imp_->J4->makI(), pe, "123");
			}

			for (auto& m : model()->motionPool()) m.updP();
			return 0;
		}
	}
	auto ScaraInverseKinematicSolver::setWhichRoot(int root_of_0_to_1)->void { imp_->which_root_ = root_of_0_to_1; };
	auto ScaraInverseKinematicSolver::whichRoot()const->int { return imp_->which_root_; }
	ScaraInverseKinematicSolver::~ScaraInverseKinematicSolver() = default;
	ScaraInverseKinematicSolver::ScaraInverseKinematicSolver() :InverseKinematicSolver(1, 0.0), imp_(new Imp) {}
	ARIS_DEFINE_BIG_FOUR_CPP(ScaraInverseKinematicSolver);


	auto createModelScara(const ScaraParam &param)->std::unique_ptr<aris::dynamic::Model> {
		std::unique_ptr<aris::dynamic::Model> model(new aris::dynamic::Model);

		model->setName("ScaraModel");
		
		////////////////////////////  DH  /////////////////////////////
		model->variablePool().add<aris::dynamic::MatrixVariable>("dh", aris::core::Matrix({ param.a, param.b }));
		model->variablePool().add<aris::dynamic::MatrixVariable>("tool0_pe", aris::core::Matrix(1, 6, param.tool0_pe));
		model->variablePool().add<aris::dynamic::StringVariable>("tool0_pe_type", param.tool0_pe_type.empty() ? std::string("321") : param.tool0_pe_type);
		model->variablePool().add<aris::dynamic::MatrixVariable>("base_pe", aris::core::Matrix(1, 6, param.base2ref_pe));
		model->variablePool().add<aris::dynamic::StringVariable>("base_pe_type", param.base2ref_pe_type.empty() ? std::string("321") : param.base2ref_pe_type);
		model->variablePool().add<aris::dynamic::MatrixVariable>("axis_range", aris::core::Matrix(1, 4, param.axis_range));
		model->variablePool().add<aris::dynamic::MatrixVariable>("install_method", aris::core::Matrix(1, 1, param.install_method));

		////////////////////////////  ENVIRONMENTS  /////////////////////////////
		const double gravity[6]{ 0.0,0.0,-9.8,0.0,0.0,0.0 };
		model->environment().setGravity(gravity);

		////////////////////////////  EE  /////////////////////////////
		// 这里的末端角度不对，和注释对不上 tbd//
		// compute ee info //
		const double axis_6_pe[6]{ param.a + param.b, 0.0, 0.0, 0.0, 0.0,0.0 };
		double axis_6_pm[16];
		double ee_i_pm[16], ee_i_wrt_axis_6_pm[16];
		double ee_j_pm[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };

		s_pe2pm(axis_6_pe, axis_6_pm, "321");
		s_pe2pm(param.tool0_pe, ee_i_wrt_axis_6_pm, param.tool0_pe_type.empty() ? "321" : param.tool0_pe_type.c_str());
		s_pm2pm(axis_6_pm, ee_i_wrt_axis_6_pm, ee_i_pm);

		////////////////////////////  PARTS  /////////////////////////////
		const double default_iv[10]{ 1,0,0,0,0,0,0,0,0,0 };
		auto &p1 = model->partPool().add<Part>("L1", param.iv_vec.size() == 4 ? param.iv_vec[0].data() : default_iv);
		auto &p2 = model->partPool().add<Part>("L2", param.iv_vec.size() == 4 ? param.iv_vec[1].data() : default_iv);
		auto &p3 = model->partPool().add<Part>("L3", param.iv_vec.size() == 4 ? param.iv_vec[2].data() : default_iv);
		auto &p4 = model->partPool().add<Part>("L4", param.iv_vec.size() == 4 ? param.iv_vec[0].data() : default_iv, ee_i_pm);

		// add geometry //

		////////////////////////////  JOINTS  /////////////////////////////
		const double joint1_position[3]{ 0 ,                  0 , 0 };
		const double joint1_axis[3]{ 0 , 0 , 1 };
		const double joint2_position[3]{ param.a ,            0 , 0 };
		const double joint2_axis[3]{ 0 , 0 , 1 };
		const double joint3_position[3]{ param.a + param.b  , 0 , 0 };
		const double joint3_axis[3]{ 0 , 0 , 1 };
		const double joint4_position[3]{ param.a + param.b  , 0 , 0 };
		const double joint4_axis[3]{ 0 , 0 , 1 };

		auto &j1 = model->addRevoluteJoint(p1, model->ground(), joint1_position, joint1_axis);
		auto &j2 = model->addRevoluteJoint(p2, p1, joint2_position, joint2_axis);
		auto &j3 = model->addPrismaticJoint(p3, p2, joint3_position, joint3_axis);
		auto &j4 = model->addScrewJoint(p4, p3, joint4_position, joint4_axis, param.pitch);

		////////////////////////////  MOTIONS  /////////////////////////////
		auto &m1 = model->addMotion(j1);
		auto &m2 = model->addMotion(j2);
		auto &m3 = model->addMotion(j3);
		auto &m4 = model->addMotion(j4);

		m1.setRotateRange(param.axis_range[0]);
		m2.setRotateRange(param.axis_range[1]);
		m3.setRotateRange(param.axis_range[2]);
		m4.setRotateRange(param.axis_range[3]);

		const double default_mot_frc[3]{ 0.0, 0.0, 0.0 };
		m1.setFrcCoe(param.mot_frc_vec.size() == 6 ? param.mot_frc_vec[0].data() : default_mot_frc);
		m2.setFrcCoe(param.mot_frc_vec.size() == 6 ? param.mot_frc_vec[1].data() : default_mot_frc);
		m3.setFrcCoe(param.mot_frc_vec.size() == 6 ? param.mot_frc_vec[2].data() : default_mot_frc);
		m4.setFrcCoe(param.mot_frc_vec.size() == 6 ? param.mot_frc_vec[3].data() : default_mot_frc);

		////////////////////////////  EES  /////////////////////////////
		auto &makI = p4.addMarker("tool0");
		auto &makJ = model->ground().addMarker("wobj0", ee_j_pm);
		auto &ee = model->generalMotionPool().add<aris::dynamic::XyztMotion>("ee", &makI, &makJ, false);

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
		// 这里还需要把末端的坐标系的角度做一个相反的处理
		s_vc(16, *ee.makI()->prtPm(), install_pm);
		s_mm(3, 3, 3, install_pm_relative, 4, *ee.makI()->prtPm(), 4, install_pm, 4);
		ee.makI()->setPrtPm(install_pm);

		

		////////////////////////////  ROBOT POSITION  /////////////////////////////
		double robot_pm[16];
		s_pe2pm(param.base2ref_pe, robot_pm, param.base2ref_pe_type.empty() ? "321" : param.base2ref_pe_type.c_str());

		p1.setPm(s_pm_dot_pm(robot_pm, *p1.pm()));
		p2.setPm(s_pm_dot_pm(robot_pm, *p2.pm()));
		p3.setPm(s_pm_dot_pm(robot_pm, *p3.pm()));
		p4.setPm(s_pm_dot_pm(robot_pm, *p4.pm()));
		j1.makJ()->setPrtPm(s_pm_dot_pm(robot_pm, *j1.makJ()->prtPm()));
		ee.makJ()->setPrtPm(s_pm_dot_pm(robot_pm, *ee.makJ()->prtPm()));

		////////////////////////////  TOOLS WOBJS  /////////////////////////////
		for (int i = 1; i < 17; ++i) 
			p4.addMarker("tool" + std::to_string(i), *ee.makI()->prtPm());
		for (int i = 1; i < 33; ++i) 
			model->ground().markerPool().add<aris::dynamic::Marker>("wobj" + std::to_string(i), *ee.makJ()->prtPm());

		////////////////////////////  SOLVERS  /////////////////////////////
		auto &inverse_kinematic = model->solverPool().add<aris::dynamic::ScaraInverseKinematicSolver>();
		auto &forward_kinematic = model->solverPool().add<aris::dynamic::ScaraForwardKinematicSolver>();
		auto &inverse_dynamic = model->solverPool().add<aris::dynamic::InverseDynamicSolver>();
		auto &forward_dynamic = model->solverPool().add<aris::dynamic::ForwardDynamicSolver>();
		inverse_kinematic.setWhichRoot(2);

		////////////////////////////  TOPOLOGY  /////////////////////////////
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
