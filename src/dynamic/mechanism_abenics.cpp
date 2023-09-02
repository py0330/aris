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
#include "aris/dynamic/mechanism_abenics.hpp"

namespace aris::dynamic {

	auto abenicsInverse(const double *param, const double *ee_xyza, int which_root, double *input)->int {
		return 0;
	}
	class AbenicsInverseKinematicSolver :public aris::dynamic::InverseKinematicSolver {
	public:
		auto virtual kinPos()->int override {
			//InverseKinematicSolver::kinPos();
			////
			//double input[6];
			//model()->getInputPos(input);
			//dsp(1, 3, input);

			auto& ee = model()->generalMotionPool()[0];

			double rm_ee[9];
			s_re2rm(ee.p(), rm_ee, "123");


			double s2 = std::sqrt(2.0) / 2;
			double rm_ee0_wrt_A[9]{
				 s2,  0,  s2,
				  0,  1,  0,
				-s2,  0,  s2
			};
			double rm_ee0_wrt_B[9]{
				-s2,  0,  s2,
				 0,   1,  0,
				-s2,  0, -s2
			};

			// ee_in_A = rot_in_A * ee0_in_A
			//
			// also:
			// ee_in_A = ee0_in_A * ee_in_ee0
			//
			// ee_in_ee0 is ee pos
			//
			// gives : rot_in_A = ee0_in_A * ee_in_ee0 * ee0_in_A^-1

			double rm1[9], rm2[9], re_A[3], re_B[3];

			s_mm(3, 3, 3, rm_ee0_wrt_A, rm_ee, rm1);
			s_mm(3, 3, 3, rm1, 3, rm_ee0_wrt_A, T(3), rm2, 3);
			s_rm2re(rm2, re_A, "131");

			s_mm(3, 3, 3, rm_ee0_wrt_B, rm_ee, rm1);
			s_mm(3, 3, 3, rm1, 3, rm_ee0_wrt_B, T(3), rm2, 3);
			s_rm2re(rm2, re_B, "131");

			//if (this->whichRoot() & 0x01) 
			{
				re_A[0] = re_A[0] > PI ? re_A[0] - PI : re_A[0] + PI;
				re_A[1] = 2 * PI - re_A[1];
				re_A[2] = re_A[2] > PI ? re_A[2] - PI : re_A[2] + PI;
			}
			
			//if (this->whichRoot() & 0x02) 
			{
				re_B[0] = re_B[0] > PI ? re_B[0] - PI : re_B[0] + PI;
				re_B[1] = 2 * PI - re_B[1];
				re_B[2] = re_B[2] > PI ? re_B[2] - PI : re_B[2] + PI;
			}

			for (int i = 0; i < 3; ++i) {
				re_A[i] -= re_A[i] > PI ? 2 * PI : 0.0;
				re_A[i] += re_A[i] < -PI ? 2 * PI : 0.0;
				re_B[i] -= re_B[i] > PI ? 2 * PI : 0.0;
				re_B[i] += re_B[i] < -PI ? 2 * PI : 0.0;
			}
			
			//if (re_A[0] > PI / 2) {
			//	re_A[0] -= PI;
			//	re_A[1] = -re_A[1];
			//	re_A[2] -= re_A[2] > 0 ? PI : -PI;
			//}

			//if (re_A[0] < -PI / 2) {
			//	re_A[0] += PI;
			//	re_A[1] = -re_A[1];
			//	re_A[2] -= re_A[2] > 0 ? PI : -PI;
			//}

			//if (re_B[0] > PI / 2) {
			//	re_B[0] -= PI;
			//	re_B[1] = -re_B[1];
			//	re_B[2] -= re_B[2] > 0 ? PI : -PI;
			//}

			//if (re_B[0] < -PI / 2) {
			//	re_B[0] += PI;
			//	re_B[1] = -re_B[1];
			//	re_B[2] -= re_B[2] > 0 ? PI : -PI;
			//}

			//dsp(1, 3, re_A);
			//dsp(1, 3, re_B);
			
			
			// 生成4组解 
			double roots[4][4]{ 
				{ re_A[0], re_A[1], re_B[0], re_B[1] },
				{ re_A[0] + PI, -re_A[1], re_B[0], re_B[1] },
				{ re_A[0], re_A[1], re_B[0] + aris::PI, -re_B[1] },
				{ re_A[0] + PI, -re_A[1], re_B[0] + aris::PI, -re_B[1] } 
			};

			// 如果当前没有解，将其设为0
			for (int i = 0; i < 4; ++i) {
				if (!std::isfinite(model()->inputPosAt(i))) {
					model()->setInputPosAt(0.0, i);
				}
			}


			// 将4组解，置入当前电机的位置里
			for (int i = 0; i < 4; ++i) {
				for (int j = 0; j < 4; ++j) {
					roots[i][j] = s_put_into_period(roots[i][j], model()->inputPosAt(j) / 2 / aris::PI, aris::PI * 2);
				}
			}

			// 找到最接近的解
			double diff_q[4]{ 0 }, q[4], diff_nrm[4];
			model()->getInputPos(q);
			for (int i = 0; i < 4; ++i) {
				s_vc(4, roots[i], diff_q);
				s_vs(4, q, diff_q);
				diff_nrm[i] = s_norm(4, diff_q);
			}

			auto real_solution = std::min_element(diff_nrm, diff_nrm + 4) - diff_nrm;



			model()->setInputPos(roots[real_solution]);

			//model()->setInputPosAt(re_A[0], 0);
			//model()->setInputPosAt(re_A[1], 1);
			//model()->setInputPosAt(re_B[0], 2);
			//model()->setInputPosAt(re_B[1], 3);

			// upd each part //
			model()->motionPool()[0].updMakIPm();
			model()->motionPool()[1].updMakIPm();
			model()->motionPool()[2].updMakIPm();
			model()->motionPool()[3].updMakIPm();
			ee.updMakIPm();



			return 0;
		}

		AbenicsInverseKinematicSolver() = default;
	};
	class AbenicsForwardKinematicSolver :public aris::dynamic::ForwardKinematicSolver {
	public:
		auto virtual kinPos()->int override {
			auto& ee = model()->generalMotionPool()[0];

			double sq2 = std::sqrt(2.0) / 2;
			double rm_ee0_wrt_A[9]{
			 sq2,  0,  sq2,
			   0,  1,  0,
			-sq2,  0,  sq2
			};



			double q1 = model()->inputPosAt(0);
			double q2 = model()->inputPosAt(1);
			double q3 = model()->inputPosAt(2);
			double q4 = model()->inputPosAt(3);

			auto s1 = std::sin(q1);
			auto c1 = std::cos(q1);
			auto s2 = std::sin(q2);
			auto c2 = std::cos(q2);
			auto s3 = std::sin(q3);
			auto c3 = std::cos(q3);
			auto s4 = std::sin(q4);
			auto c4 = std::cos(q4);

			double rm[9]{
				c2,    0, -s3*s4,
				c1*s2, 0, c3*s4,
				s1*s2, 0, c4
			};

			s_c3(rm + 2, 3, rm, 3, rm + 1, 3);
			s_c3(rm, 3, rm + 1, 3, rm + 2, 3);

			s_nv(3, 1.0 / std::sqrt(s_vv(3, rm, 3, rm, 3)), rm, 3);
			s_nv(3, 1.0 / std::sqrt(s_vv(3, rm + 1, 3, rm + 1, 3)), rm + 1, 3);
			s_nv(3, 1.0 / std::sqrt(s_vv(3, rm + 2, 3, rm + 2, 3)), rm + 2, 3);

			double rm_ee[9], rm_ee2[9];
			s_mm(3, 3, 3, rm_ee0_wrt_A, T(3), rm, 3, rm_ee, 3);
			s_mm(3, 3, 3, rm_ee, 3, rm_ee0_wrt_A, 3, rm_ee2, 3);

			double re[3];
			s_rm2re(rm_ee2, re, "123");

			model()->generalMotionPool()[0].setP(re);

			// upd each part //
			model()->motionPool()[0].updMakIPm();
			model()->motionPool()[1].updMakIPm();
			model()->motionPool()[2].updMakIPm();
			model()->motionPool()[3].updMakIPm();
			ee.updMakIPm();



			return 0;
		}

		AbenicsForwardKinematicSolver() = default;
	};

	auto createModelAbenics(const AbenicsParam& param)->std::unique_ptr<aris::dynamic::Model> {
		std::unique_ptr<aris::dynamic::Model> model(new aris::dynamic::Model);

		// 设置重力 //
		const double gravity[6]{ 0.0,0.0,-9.8,0.0,0.0,0.0 };
		model->environment().setGravity(gravity);

		// add part //
		auto& A1 = model->partPool().add<Part>("A1");
		auto& A2 = model->partPool().add<Part>("A2");
		auto& B1 = model->partPool().add<Part>("B1");
		auto& B2 = model->partPool().add<Part>("B2");
		auto& EE = model->partPool().add<Part>("EE");

		// add geometry //

		// add joint //
		auto s2 = std::sqrt(2.0) / 2.0;
		const double center[3]{ 0 , 0 , 0 };
		const double a1_axis[3]{  s2 , 0 , s2 };
		const double a2_axis[3]{ -s2 , 0 , s2 };
		const double a3_axis[3]{  s2 , 0 , s2 };
		const double b1_axis[3]{ -s2 , 0 , s2 };
		const double b2_axis[3]{ -s2 , 0 ,-s2 };
		const double b3_axis[3]{ -s2 , 0 , s2 };

		auto& a1 = model->addRevoluteJoint(A1, model->ground(), center, a1_axis);
		auto& a2 = model->addRevoluteJoint(A2, A1, center, a2_axis);
		auto& a3 = model->addRevoluteJoint(EE, A2, center, a3_axis);
		auto& b1 = model->addRevoluteJoint(B1, model->ground(), center, b1_axis);
		auto& b2 = model->addRevoluteJoint(B2, B1, center, b2_axis);
		auto& b3 = model->addRevoluteJoint(EE, B2, center, b3_axis);


		// add actuation //
		auto& m1 = model->addMotion(a1);
		auto& m2 = model->addMotion(a2);
		//auto& m5 = model->addMotion(a3);
		auto& m3 = model->addMotion(b1);
		auto& m4 = model->addMotion(b2);
		//auto& m6 = model->addMotion(b3);

		// add ee general motion //
		auto& makI = EE.addMarker("tool0");
		auto& makJ = model->ground().addMarker("wobj0");
		auto& ee = model->generalMotionPool().add<aris::dynamic::SphericalMotion>("tool", &makI, &makJ, false);

		// add solver
		auto& inverse_kinematic = model->solverPool().add<aris::dynamic::AbenicsInverseKinematicSolver>();
		auto& forward_kinematic = model->solverPool().add<aris::dynamic::AbenicsForwardKinematicSolver>();
		auto& inverse_dynamic = model->solverPool().add<aris::dynamic::InverseDynamicSolver>();
		auto& forward_dynamic = model->solverPool().add<aris::dynamic::ForwardDynamicSolver>();

		// make topology correct // 
		for (auto& m : model->motionPool())m.activate(true);
		for (auto& gm : model->generalMotionPool())gm.activate(false);
		for (auto& f : model->forcePool())f.activate(false);

		model->init();
		return model;
	}

	ARIS_REGISTRATION{
		aris::core::class_<AbenicsInverseKinematicSolver>("AbenicsInverseKinematicSolver")
			.inherit<InverseKinematicSolver>()
			;

		aris::core::class_<AbenicsForwardKinematicSolver>("AbenicsForwardKinematicSolver")
			.inherit<ForwardKinematicSolver>()
			;
	}
}
