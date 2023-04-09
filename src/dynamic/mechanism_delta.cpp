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
#include "aris/dynamic/kinematics.hpp"
#include "aris/dynamic/mechanism_delta.hpp"

namespace aris::dynamic {

//#define ARIS_DEBUG_DELTA_SOLVER

	auto deltaInverse(const double *param, const double *ee_xyza, int which_root, double *input)->int {
		for (int i = 0; i < 3; ++i) {
			// 尺寸 //
			const double& ax = param[0 + i * 11];
			const double& ay = param[1 + i * 11];
			const double& az = param[2 + i * 11];
			const double& b = param[3 + i * 11];
			const double& c = param[4 + i * 11];
			const double& d = param[5 + i * 11];
			const double& ex = param[6 + i * 11];
			const double& ey = param[7 + i * 11];
			const double& ez = param[8 + i * 11];
			const double& theta = param[9 + i * 11];

			// 此处将点转回到原x z平面，因此与构造delta的地方不太一样 //
			auto x =  ee_xyza[0] * std::cos(theta) + ee_xyza[1] * std::sin(theta);
			auto y = -ee_xyza[0] * std::sin(theta) + ee_xyza[1] * std::cos(theta) + ey - ay;
			auto z =  ee_xyza[2] + ez - az;

			if (std::abs(d) < std::abs(y)) return -1;

			auto d1 = std::sqrt(d * d - y * y);
			auto k = x + ex - ax;
			auto l = std::sqrt(k * k + z * z);

			auto bc = std::sqrt(b * b + c * c);

			if (bc + l < d1 || bc + d1 < l || d1 + l < bc)
				return -2;

			input[i] =
				0x01 << which_root ?
				-std::atan2(z, k) + std::atan2(c, b) - std::acos((bc * bc + l * l - d1 * d1) / bc / l / 2.0) :
				-std::atan2(z, k) + std::atan2(c, b) + std::acos((bc * bc + l * l - d1 * d1) / bc / l / 2.0);
		}

		input[3] = ee_xyza[3];
		return 0;
	}
	auto deltaForward(const double *param, const double *input, int which_root, double *ee_xyza)->int {
		// 记p1 为 S1 S2 的中点位置，p2为末端到 S3 S4 中点的向量
		//
		// 于是对其中某一根支联，应有以下方程：
		//  || ee + p2 - p1 || = d
		// 
		// 若记：k = p2 - p1
		//
		// 继而：
		//   (x + k0)^2 + (y + k1)^2 + (z + k2)^2 = d^2
		//
		// 展开：
		//   x^2 + 2k0 x + k0^2 + y^2 + 2k1 x + k1^2 + z^2 + 2k2 z + k2^2 = d^2
		// 
		// 若记: t = x^2 + y^2 + z^2
		//       s = d^2 - k0^2 - k1^2 - k2^2
		//       k = 2k
		//
		// 于是有：
		//   [k0 k1 k2] * [x; y; z] = s - t
		//
		// 对3根支联列方程：
		//   [ k00 k01 k02 ]   [ x ]   [ s1 - t ]
		//   | k10 k11 k12 | * | y | = | s2 - t |
		//   [ k20 k21 k22 ]   [ z ]   [ s3 - t ]
		//
		// 【METHOD 1】：K矩阵不奇异
		// 记做：
		//   K * x = b - t
		// 其中 x^T * x = t
		//
		// 于是：
		//    x = K^-1 * (b - t) = [ g1 + h1 * t ]
		//                         | g2 + h2 * t |
		//                         [ g3 + h3 * t ]
		//
		// 继而有：
		//    t = (h1^2 + h2^2 + h3^2)t^2 + (2 h1 g1 + 2 h1 g1 + 2 h1 g1)t + (g1^2 + g2^2 + g3^2)
		//
		// 于是有：
		//    A t^2 + B t + C = 0
		// 其中：
		//    A：(h1^2 + h2^2 + h3^2)
		//    B: (2 h1 g1 + 2 h1 g1 + 2 h1 g1) - 1
		//    C: (g1^2 + g2^2 + g3^2)
		//
		// 用以上1元2次方程求解出t
		//
		// 进一步的，有：
		// [x; y; z] = g + h * t
		// 
		// 
		// 
		// 
		// 【METHOD 2】：
		// 原方程化为：
		//   [ k00 k01 k02 1.0 ]   [ x ]   [ s1 ]
		//   | k10 k11 k12 1.0 | * | y | = | s2 |
		//   [ k20 k21 k22 1.0 ]   | z |   [ s3 ]
		//                         [ t ]
		// 
		//       x^2 + y^2 + z^2         =   t              
		// 
		//  
		// 进一步的：
		//    Q * R * P^T * x  =  b
		// 
		// 考虑用未知数 y 替换 x：
		//    y = P^T * x
		// 
		// 
		// 进一步的：
		//    [ R1 R2 ] * y    =  Q^T * b
		// 
		// 因为R1 维 3 x 3的矩阵，因此它应该满秩
		// 
		// 于是有：
		//  [ y1 ]  =  R1^-1 * [ Q^T * b , R2 * y4]  = [ g1 + h1 * y4 ]
		//  | y2 |                                     | g2 + h2 * y4 |
		//  [ y3 ]                                     [ g3 + h3 * y4 ]
		//  
		// 若 p[4] == 4 则 y4 为 t，为 method 1 中的情况：
		// 此时可以将 x^2 + y^2 + z^2         =   t    化为：
		// 
		//  A * y4^2 + B * y4 + C =0
		// 
		// 其中 ：
		// 
		// A = h1^2 + h2^2 + h3^2
		// B = 2(h1 g1 + h2 g2 + h3 g3) - 1
		// C = g1^2 + g2^2 + g3^2
		// 
		// 可求出 y4
		// 
		// 下面列出通用情况：
		// 
		// 令 g = [g1 g2 g3 0]^T
		//    h = [h1 h2 h3 1]^T
		//
		// 令 t 经 p 变化后的索引为 idx3，则：
		// 
		//    f = g, f[idx3] = -1
		//    C = f^T * g;
		//
		//    f = 2 * g, f[idx3] = -1
		//    B = f^T * h;
		// 
		//    f = h, f[idx3] = 0
		//    A = f^T * h;
		// 
		// 求解该二次方程 A y4^2 + B y4 + C = 0, 可得y4
		// 
		// 之后可得y1 ~ y3
		// 
		// 
		// 



		// 根据 p1 & p2 计算k
		double p1[9], p2[9], k[9], s[3];
		for (int i = 0; i < 3; ++i) {
			const double& ax = param[0 + i * 11];
			const double& ay = param[1 + i * 11];
			const double& az = param[2 + i * 11];
			const double& b = param[3 + i * 11];
			const double& c = param[4 + i * 11];
			const double& d = param[5 + i * 11];
			const double& ex = param[6 + i * 11];
			const double& ey = param[7 + i * 11];
			const double& ez = param[8 + i * 11];
			const double& theta = param[9 + i * 11];

			p1[0 + i * 3] = (ax + std::cos(input[i]) * b + std::sin(input[i]) * c) * std::cos(theta) - ay * std::sin(theta);
			p1[1 + i * 3] = (ax + std::cos(input[i]) * b + std::sin(input[i]) * c) * std::sin(theta) + ay * std::cos(theta);
			p1[2 + i * 3] = -std::sin(input[i]) * b + std::cos(input[i]) * c + az;

			p2[0 + i * 3] = ex * std::cos(theta) - ey * std::sin(theta);
			p2[1 + i * 3] = ex * std::sin(theta) + ey * std::cos(theta);
			p2[2 + i * 3] = 0.0;

			k[0 + i * 3] = p2[0 + i * 3] - p1[0 + i * 3];
			k[1 + i * 3] = p2[1 + i * 3] - p1[1 + i * 3];
			k[2 + i * 3] = p2[2 + i * 3] - p1[2 + i * 3];

			s[i] = d * d - k[0 + i * 3] * k[0 + i * 3] - k[1 + i * 3] * k[1 + i * 3] - k[2 + i * 3] * k[2 + i * 3];
		}

		// 随后k乘以2
		s_nv(9, 2.0, k);

		// 求解上述方程 //
		{
			double A_mat[12]{
				k[0], k[1], k[2], 1.0,
				k[3], k[4], k[5], 1.0,
				k[6], k[7], k[8], 1.0
			};

			double U[12], tau[4];
			aris::Size p[4], rank;

			s_householder_utp(3, 4, A_mat, U, tau, p, rank);

			double b[6]{
				s[0], -U[3],
				s[1], -U[7],
				s[2], -U[11],
			};

			// 注意，这里仅仅乘以 s
			s_householder_ut_qt_dot(3, 4, 1, U, 4, tau, 1, b, 2, b, 2);
			s_sov_um(3, 2, U, 4, b, 2, b, 2);

			double g[4]{ b[0], b[2], b[4], 0.0 }, h[4]{ b[1], b[3], b[5], 1.0 }, f[4];

			auto idx_3 = std::find(p, p + 4, 3) - p;

			s_vc(4, g, f);
			f[idx_3] = -1;
			double C = s_vv(3, f, g);
			
			s_vc(4, 2.0, g, f);
			f[idx_3] = -1;
			double B = s_vv(4, f, h);

			s_vc(4, h, f);
			f[idx_3] = 0.0;
			double A = s_vv(4, f, h);

			double v1[4], v2[4];

			v1[p[3]] = (-B - std::sqrt(B * B - 4 * A * C)) / 2.0 / A;
			v1[p[0]] = b[0] + b[1] * v1[p[3]];
			v1[p[1]] = b[2] + b[3] * v1[p[3]];
			v1[p[2]] = b[4] + b[5] * v1[p[3]];

			v2[p[3]] = (-B + std::sqrt(B * B - 4 * A * C)) / 2.0 / A;
			v2[p[0]] = b[0] + b[1] * v2[p[3]];
			v2[p[1]] = b[2] + b[3] * v2[p[3]];
			v2[p[2]] = b[4] + b[5] * v2[p[3]];



#ifdef ARIS_DEBUG_DELTA_SOLVER
			std::cout << "A * v:" << std::endl;
			{
				double result[4], Q[9], R[12];
				s_mm(3, 1, 4, A_mat, v, result);
				dsp(1, 3, result);
			}
			std::cout << "x^2 + y^2 + z^2 - norm:" << v[0]* v[0] + v[1] * v[1] + v[2] * v[2] - v[3] << std::endl;
#endif

			// 选择 z 轴较小的根，在 which_root 为 0 时
			if ((v2[2] < v1[2]) == (which_root == 0)) {
				ee_xyza[0] = v2[0];
				ee_xyza[1] = v2[1];
				ee_xyza[2] = v2[2];
			}
			else {
				ee_xyza[0] = v1[0];
				ee_xyza[1] = v1[1];
				ee_xyza[2] = v1[2];
			}
			ee_xyza[3] = input[3];

			return 0;
		}
	}

	class DeltaInverseKinematicSolver :public aris::dynamic::InverseKinematicSolver {
	public:
		auto virtual kinPos()->int override {
			double input[4], output[4];

			auto dh = dynamic_cast<aris::dynamic::MatrixVariable*>(model()->findVariable("dh"))->data().data();

			model()->getOutputPos(output);
			if (auto ret = deltaInverse(dh, output, 0, input))
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
	class DeltaForwardKinematicSolver :public aris::dynamic::ForwardKinematicSolver {
	public:
		auto virtual kinPos()->int override {
			double input[4], output[4];

			auto dh = dynamic_cast<aris::dynamic::MatrixVariable*>(model()->findVariable("dh"))->data().data();

			model()->getInputPos(input);
			if (auto ret = deltaForward(dh, input, 0, output))
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

			for (auto &m : model()->generalMotionPool()) m.updP();
			return 0;
		}

		DeltaForwardKinematicSolver() = default;
	};

	auto ARIS_API createModelDelta(const DeltaParam &param)->std::unique_ptr<aris::dynamic::Model> {
		DeltaFullParam full_param;
		full_param.ax1 = full_param.ax2 = full_param.ax3 = param.a;
		full_param.b1 = full_param.b2 = full_param.b3 = param.b;
		full_param.c1 = full_param.c2 = full_param.c3 = param.c;
		full_param.d1 = full_param.d2 = full_param.d3 = param.d;
		full_param.ex1 = full_param.ex2 = full_param.ex3 = param.e;

		full_param.theta1 = 0.0;
		full_param.theta2 = aris::PI * 2 / 3;
		full_param.theta3 = -aris::PI * 2 / 3;

		s_vc(6, param.tool0_pe, full_param.tool0_pe);
		full_param.tool0_pe_type = param.tool0_pe_type;

		s_vc(6, param.base2ref_pe, full_param.base2ref_pe);
		full_param.base2ref_pe_type = param.base2ref_pe_type;

		full_param.iv_vec = param.iv_vec;

		full_param.mot_frc_vec = param.mot_frc_vec;

		return createModelDelta(full_param);
	}
	auto ARIS_API createModelDelta(const DeltaFullParam &param)->std::unique_ptr<aris::dynamic::Model> {
		std::unique_ptr<aris::dynamic::Model> model(new aris::dynamic::Model);

		model->variablePool().add<aris::dynamic::MatrixVariable>("dh", aris::core::Matrix({ 
			param.ax1, param.ay1, param.az1, param.b1, param.c1, param.d1, param.ex1, param.ey1, param.ez1, param.theta1, param.f1,
			param.ax2, param.ay2, param.az2, param.b2, param.c2, param.d2, param.ex2, param.ey2, param.ez2, param.theta2, param.f2,
			param.ax3, param.ay3, param.az3, param.b3, param.c3, param.d3, param.ex3, param.ey3, param.ez3, param.theta3, param.f3, 
		}));

		// 设置重力 //
		const double gravity[6]{ 0.0,0.0,-9.8,0.0,0.0,0.0 };
		model->environment().setGravity(gravity);

		// add part //
		const double default_iv[10]{ 1,0,0,0,0,0,0,0,0,0 };
		auto &p11 = model->partPool().add<Part>("L11", param.iv_vec.size() == 4 ? param.iv_vec[0].data() : default_iv);
		auto &p12 = model->partPool().add<Part>("L12", param.iv_vec.size() == 4 ? param.iv_vec[1].data() : default_iv);
		auto &p13 = model->partPool().add<Part>("L13", param.iv_vec.size() == 4 ? param.iv_vec[2].data() : default_iv);
		auto &p21 = model->partPool().add<Part>("L21", param.iv_vec.size() == 4 ? param.iv_vec[0].data() : default_iv);
		auto &p22 = model->partPool().add<Part>("L22", param.iv_vec.size() == 4 ? param.iv_vec[1].data() : default_iv);
		auto &p23 = model->partPool().add<Part>("L23", param.iv_vec.size() == 4 ? param.iv_vec[2].data() : default_iv);
		auto &p31 = model->partPool().add<Part>("L31", param.iv_vec.size() == 4 ? param.iv_vec[0].data() : default_iv);
		auto &p32 = model->partPool().add<Part>("L32", param.iv_vec.size() == 4 ? param.iv_vec[1].data() : default_iv);
		auto &p33 = model->partPool().add<Part>("L33", param.iv_vec.size() == 4 ? param.iv_vec[2].data() : default_iv);
		auto &pup = model->partPool().add<Part>("UP", param.iv_vec.size() == 4 ? param.iv_vec[3].data() : default_iv);
		auto &pee = model->partPool().add<Part>("EE", param.iv_vec.size() == 4 ? param.iv_vec[3].data() : default_iv);

		// add geometry //
		//model->ground().geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\aris\\aris-1.5.0\\resource\\test_dynamic\\delta\\base.xmt_txt");
		//pup.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\aris\\aris-1.5.0\\resource\\test_dynamic\\delta\\up.xmt_txt");
		//pee.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\aris\\aris-1.5.0\\resource\\test_dynamic\\delta\\ee.xmt_txt");
		//double geo_p13_prt_pm[16]{ 1,0,0,0,0,1,0,-param.c,0,0,1,0,0,0,0,1 };
		//double geo_rot_pm[16], geo_local_pm[16];
		//p11.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\aris\\aris-1.5.0\\resource\\test_dynamic\\delta\\p1.xmt_txt");
		//p12.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\aris\\aris-1.5.0\\resource\\test_dynamic\\delta\\p2.xmt_txt");
		//p13.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\aris\\aris-1.5.0\\resource\\test_dynamic\\delta\\p2.xmt_txt", geo_p13_prt_pm);
		//s_eye(4, geo_rot_pm);
		//s_rmz(PI * 2 / 3, geo_rot_pm, 4);
		//s_pm_dot_pm(geo_rot_pm, geo_p13_prt_pm, geo_local_pm);
		//p21.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\aris\\aris-1.5.0\\resource\\test_dynamic\\delta\\p1.xmt_txt", geo_rot_pm);
		//p22.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\aris\\aris-1.5.0\\resource\\test_dynamic\\delta\\p2.xmt_txt", geo_rot_pm);
		//p23.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\aris\\aris-1.5.0\\resource\\test_dynamic\\delta\\p2.xmt_txt", geo_local_pm);
		//s_eye(4, geo_rot_pm);
		//s_rmz(-PI * 2 / 3, geo_rot_pm, 4);
		//s_pm_dot_pm(geo_rot_pm, geo_p13_prt_pm, geo_local_pm);
		//p31.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\aris\\aris-1.5.0\\resource\\test_dynamic\\delta\\p1.xmt_txt", geo_rot_pm);
		//p32.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\aris\\aris-1.5.0\\resource\\test_dynamic\\delta\\p2.xmt_txt", geo_rot_pm);
		//p33.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\aris\\aris-1.5.0\\resource\\test_dynamic\\delta\\p2.xmt_txt", geo_local_pm);

		// 正解计算起始时刻的末端位置 //
		double dh_param[33]{ 
			param.ax1, param.ay1, param.az1, param.b1, param.c1, param.d1, param.ex1, param.ey1, param.ez1, param.theta1, param.f1,
			param.ax2, param.ay2, param.az2, param.b2, param.c2, param.d2, param.ex2, param.ey2, param.ez2, param.theta2, param.f2,
			param.ax3, param.ay3, param.az3, param.b3, param.c3, param.d3, param.ex3, param.ey3, param.ez3, param.theta3, param.f3,
		};
		double init_zero[4]{ 0,0,0,0 };
		double xyza[4], xyz1[3], xyz2[3], xyz3[3];
		deltaForward(dh_param, init_zero, 0, xyza);

		double rm1[9], rm2[9], rm3[9];
		s_rmz(param.theta1, rm1);
		s_rmz(param.theta2, rm2);
		s_rmz(param.theta3, rm3);

		s_mm(3, 1, 3, rm1, T(3), xyza, 1, xyz1, 1);
		s_mm(3, 1, 3, rm2, T(3), xyza, 1, xyz2, 1);
		s_mm(3, 1, 3, rm3, T(3), xyza, 1, xyz3, 1);

		// add joint //
		double y_axis[3]{ 0,1,0 };

		double r11_pos_1[3]{ param.ax1           , param.ay1               , param.az1 };
		double s12_pos_1[3]{ param.ax1 + param.b1, param.ay1 + param.f1 / 2, param.az1 + param.c1 };
		double s13_pos_1[3]{ param.ax1 + param.b1, param.ay1 - param.f1 / 2, param.az1 + param.c1 };
		double s14_pos_1[3]{ xyz1[0] + param.ex1 , xyz1[1] + param.ey1 + param.f1 / 2, xyz1[2] + param.ez1 };
		double s15_pos_1[3]{ xyz1[0] + param.ex1 , xyz1[1] + param.ey1 - param.f1 / 2, xyz1[2] + param.ez1 };
		
		double r11_axis_1[6]{ 0.0, 1.0, 0.0 };
		
		double s12_s14[3]{ s14_pos_1[0] - s12_pos_1[0], s14_pos_1[1] - s12_pos_1[1], s14_pos_1[2] - s12_pos_1[2]};
		double u12_first_axis_1[3];
		double u12_second_axis_1[3];
		s_c3(s12_s14, y_axis, u12_second_axis_1);
		s_nv(3, 1.0 / s_norm(3, u12_second_axis_1), u12_second_axis_1);
		s_c3(s12_s14, u12_second_axis_1, u12_first_axis_1);

		double r21_pos_1[3]{ param.ax2           , param.ay2               , param.az2 };
		double s22_pos_1[3]{ param.ax2 + param.b2, param.ay2 + param.f2 / 2, param.az2 + param.c2 };
		double s23_pos_1[3]{ param.ax2 + param.b2, param.ay2 - param.f2 / 2, param.az2 + param.c2 };
		double s24_pos_1[3]{ xyz2[0] + param.ex2 , xyz2[1] + param.ey2 + param.f2 / 2, xyz2[2] + param.ez2 };
		double s25_pos_1[3]{ xyz2[0] + param.ex2 , xyz2[1] + param.ey2 - param.f2 / 2, xyz2[2] + param.ez2 };

		double r21_axis_1[6]{ 0.0, 1.0, 0.0 };

		double s22_s24[3]{ s24_pos_1[0] - s22_pos_1[0], s24_pos_1[1] - s22_pos_1[1], s24_pos_1[2] - s22_pos_1[2] };
		double u22_first_axis_1[3];
		double u22_second_axis_1[3];
		s_c3(s22_s24, y_axis, u22_second_axis_1);
		s_nv(3, 1.0 / s_norm(3, u22_second_axis_1), u22_second_axis_1);
		s_c3(s22_s24, u22_second_axis_1, u22_first_axis_1);

		double r31_pos_1[3]{ param.ax3           , param.ay3               , param.az3 };
		double s32_pos_1[3]{ param.ax3 + param.b3, param.ay3 + param.f3 / 2, param.az3 + param.c3 };
		double s33_pos_1[3]{ param.ax3 + param.b3, param.ay3 - param.f3 / 2, param.az3 + param.c3 };
		double s34_pos_1[3]{ xyz3[0] + param.ex3 , xyz3[1] + param.ey3 + param.f3 / 2, xyz3[2] + param.ez3 };
		double s35_pos_1[3]{ xyz3[0] + param.ex3 , xyz3[1] + param.ey3 - param.f3 / 2, xyz3[2] + param.ez3 };

		double r31_axis_1[6]{ 0.0, 1.0, 0.0 };

		double s32_s34[3]{ s34_pos_1[0] - s32_pos_1[0], s34_pos_1[1] - s32_pos_1[1], s34_pos_1[2] - s32_pos_1[2] };
		double u32_first_axis_1[3];
		double u32_second_axis_1[3];
		s_c3(s32_s34, y_axis, u32_second_axis_1);
		s_nv(3, 1.0 / s_norm(3, u32_second_axis_1), u32_second_axis_1);
		s_c3(s32_s34, u32_second_axis_1, u32_first_axis_1);
		
		double r11_pos[3], s12_pos[3], s13_pos[3], s14_pos[3], s15_pos[3], r11_axis[3], u12_first_axis[3], u12_second_axis[3];
		double r21_pos[3], s22_pos[3], s23_pos[3], s24_pos[3], s25_pos[3], r21_axis[3], u22_first_axis[3], u22_second_axis[3];
		double r31_pos[3], s32_pos[3], s33_pos[3], s34_pos[3], s35_pos[3], r31_axis[3], u32_first_axis[3], u32_second_axis[3];

		

		s_mm(3, 1, 3, rm1, r11_pos_1,         r11_pos);
		s_mm(3, 1, 3, rm1, s12_pos_1,         s12_pos);
		s_mm(3, 1, 3, rm1, s13_pos_1,         s13_pos);
		s_mm(3, 1, 3, rm1, s14_pos_1,         s14_pos);
		s_mm(3, 1, 3, rm1, s15_pos_1,         s15_pos);
		s_mm(3, 1, 3, rm1, r11_axis_1,        r11_axis);
		s_mm(3, 1, 3, rm1, u12_first_axis_1,  u12_first_axis);
		s_mm(3, 1, 3, rm1, u12_second_axis_1, u12_second_axis);

		s_mm(3, 1, 3, rm2, r21_pos_1,         r21_pos);
		s_mm(3, 1, 3, rm2, s22_pos_1,         s22_pos);
		s_mm(3, 1, 3, rm2, s23_pos_1,         s23_pos);
		s_mm(3, 1, 3, rm2, s24_pos_1,         s24_pos);
		s_mm(3, 1, 3, rm2, s25_pos_1,         s25_pos);
		s_mm(3, 1, 3, rm2, r21_axis_1,        r21_axis);
		s_mm(3, 1, 3, rm2, u22_first_axis_1,  u22_first_axis);
		s_mm(3, 1, 3, rm2, u22_second_axis_1, u22_second_axis);

		s_mm(3, 1, 3, rm3, r31_pos_1,         r31_pos);
		s_mm(3, 1, 3, rm3, s32_pos_1,         s32_pos);
		s_mm(3, 1, 3, rm3, s33_pos_1,         s33_pos);
		s_mm(3, 1, 3, rm3, s34_pos_1,         s34_pos);
		s_mm(3, 1, 3, rm3, s35_pos_1,         s35_pos);
		s_mm(3, 1, 3, rm3, r31_axis_1,        r31_axis);
		s_mm(3, 1, 3, rm3, u32_first_axis_1,  u32_first_axis);
		s_mm(3, 1, 3, rm3, u32_second_axis_1, u32_second_axis);

		auto &r11 = model->addRevoluteJoint(p11, model->ground(), r11_pos, r11_axis);
		auto &u12 = model->addUniversalJoint(p12, p11, s12_pos, u12_second_axis, u12_first_axis);
		auto &u13 = model->addUniversalJoint(p13, p11, s13_pos, u12_second_axis, u12_first_axis);
		auto &s14 = model->addSphericalJoint(pup, p12, s14_pos);
		auto &s15 = model->addSphericalJoint(pup, p13, s15_pos);

		auto &r21 = model->addRevoluteJoint(p21, model->ground(), r21_pos, r21_axis);
		auto &u22 = model->addUniversalJoint(p22, p21, s22_pos, u22_second_axis, u22_first_axis);
		auto &u23 = model->addUniversalJoint(p23, p21, s23_pos, u22_second_axis, u22_first_axis);
		auto &s24 = model->addSphericalJoint(pup, p22, s24_pos);
		auto &s25 = model->addSphericalJoint(pup, p23, s25_pos);

		auto &r31 = model->addRevoluteJoint(p31, model->ground(), r31_pos, r31_axis);
		auto &u32 = model->addUniversalJoint(p32, p31, s32_pos, u32_second_axis, u32_first_axis);
		auto &u33 = model->addUniversalJoint(p33, p31, s33_pos, u32_second_axis, u32_first_axis);
		auto &s34 = model->addSphericalJoint(pup, p32, s34_pos);
		auto &s35 = model->addSphericalJoint(pup, p33, s35_pos);

		// 得到末端
		double re_pos[3]{ xyza[0], xyza[1], xyza[2]};
		double re_axis[3]{ 0,0,1 };
		auto &re = model->addRevoluteJoint(pee, pup, re_pos, re_axis);
		// add actuation //
		auto &m1 = model->addMotion(r11);
		auto &m2 = model->addMotion(r21);
		auto &m3 = model->addMotion(r31);
		auto &m4 = model->addMotion(re);

		const double default_mot_frc[3]{ 0.0, 0.0, 0.0 };
		m1.setFrcCoe(param.mot_frc_vec.size() == 6 ? param.mot_frc_vec[0].data() : default_mot_frc);
		m2.setFrcCoe(param.mot_frc_vec.size() == 6 ? param.mot_frc_vec[1].data() : default_mot_frc);
		m3.setFrcCoe(param.mot_frc_vec.size() == 6 ? param.mot_frc_vec[2].data() : default_mot_frc);
		m4.setFrcCoe(param.mot_frc_vec.size() == 6 ? param.mot_frc_vec[3].data() : default_mot_frc);
		m4.setRotateRange(std::numeric_limits<double>::quiet_NaN());

		// add ee general motion //
		double ee_i_pm[16]{ 
			1,0,0,xyza[0],
			0,1,0,xyza[1],
			0,0,1,xyza[2],
			0,0,0,1 };
		double ee_j_pm[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };

		auto &makI = pee.addMarker("tool0", ee_i_pm);
		auto &makJ = model->ground().addMarker("wobj0", ee_j_pm);
		model->variablePool().add<aris::dynamic::MatrixVariable>("tool0_axis_home", aris::core::Matrix(1, 6, 0.0));
		auto &ee = model->generalMotionPool().add<aris::dynamic::XyztMotion>("tool", &makI, &makJ, false);

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
		for (int i = 1; i < 33; ++i) model->ground().markerPool().add<aris::dynamic::Marker>("wobj" + std::to_string(i), ee_j_pm);

		// add solver
		auto &inverse_kinematic = model->solverPool().add<aris::dynamic::DeltaInverseKinematicSolver>();
		auto &forward_kinematic = model->solverPool().add<aris::dynamic::DeltaForwardKinematicSolver>();
		auto &inverse_dynamic = model->solverPool().add<aris::dynamic::InverseDynamicSolver>();
		auto &forward_dynamic = model->solverPool().add<aris::dynamic::ForwardDynamicSolver>();

		// make topology correct // 
		for (auto &m : model->motionPool())m.activate(true);
		for (auto &gm : model->generalMotionPool())gm.activate(false);
		for (auto &f : model->forcePool())f.activate(false);

		model->init();
		return model;
	}

	auto planarDeltaInverse(const double *param, const double *ee_xya, int which_root, double *input)->int {
		// Planar Delta
		//---------------------------------------------------------------------------------------------
		// 包含4个自由度。共4个杆件  link1~4
		// 尺寸示意：
		//                   上平台 
		//                     
		//                     o------o      
		//                    /         \    
		//                   /            \   
		//                   o              o   
		//                     \           /  
		//                       \        /  
		//                         \     /     
		//                           \  /        
		//                             o
		//                       ee(长度e)
		//---------------------------------------------------------------------------------------------
		// 零位示意：
		//                         y 
		//                         ^
		//                         | a |<- b ->|
		//             o-------o---*---o-------o   ->  x             
		//            / \                     / 
		//                \                 /   
		//                  \             /        
		//                 c  \         /       
		//                      \     /        
		//                        \ /     
		//                         o         
		//                        /|
		//                            
		//                       ee(长度e)
		//---------------------------------------------------------------------------------------------
		
		// 输入输出关系：
		// 
		// [ x ] = [ a ] + [ c1 -s1 ] * [ b ] + [ c2 -s2 ] * [ c ] 
		// [ y ]   [ 0 ]   [ s1  c1 ]   [ 0 ]   [ s2  c2 ]   [ 0 ]
		//
		// 其中 theta1 为电机的转角，theta2 为转动副2的转角减去电机的转角
		//  
		// 进一步的：
		// [ x ] = [ a + b * c1 + c * c2 ] 
		// [ y ]   [     b * s1 + c * s2 ]
		//
		// 消除未知数 theta2：
		//
		// x - a - b * c1 = c * c2  
		// y     - b * s1 = c * s2 
		//
		// -> : c^2 = (x - a - b * c1)^2 + (y - b * s1)^2
		//
		// -> : c^2 - b^2 = (x - a)^2 - 2 (x - a) b c1 + y^2 - 2 y b s1  
		// -> : c^2 - b^2 - (x - a)^2 - y^2 = - 2 y b s1 - 2 (x - a) b c1
		
		for (int i = 0; i < 2; ++i) {
			//尺寸
			auto a = param[0 + i * 3];
			auto b = param[1 + i * 3];
			auto c = param[2 + i * 3];
			auto d = param[6];

			auto x = ee_xya[0];
			auto y = ee_xya[1] + d;

			auto k1 = -2 * y * b;
			auto k2 = -2 * (x - a) * b;
			auto right_side = c * c - b * b - (x - a)*(x - a) - y * y;

			double result[2];
			auto ret = s_sov_theta(k1, k2, right_side, result);

			input[i] = result[(0x01 << i) & which_root];
		}

		input[2] = ee_xya[2];
		return 0;
	}
	auto planarDeltaForward(const double *param, const double *input, int which_root, double *ee_xya)->int {
		// 
		//            ----* p2  
		//    p1 *----    |
		//         \     /
		//           \  /
		//             *  ee
		//
		// p1:[x1 y1]
		// p2:[x2 y2]
		// ee:[x  y ]
		//
		// 根据余弦公式，求得   p2 - p1 - ee 之间的夹角theta，再求出ee 与 p1的夹角
		//
		//
		auto d = param[6];

		double x[2], y[2];
		for (int i = 0; i < 2; ++i) {
			//尺寸
			auto a = param[0 + i * 3];
			auto b = param[1 + i * 3];
			auto c = param[2 + i * 3];
			

			auto c1 = std::cos(input[i]);
			auto s1 = std::sin(input[i]);
			
			x[i] = a + b * c1;
			y[i] = b * s1;
		}
		
		auto c1 = param[2];
		auto c2 = param[5];
		
		auto d_p1_p2 = std::sqrt((x[1] - x[0]) * (x[1] - x[0]) + (y[1] - y[0])* (y[1] - y[0]));

		auto theta = std::acos((c1 * c1 + d_p1_p2 * d_p1_p2 - c2 * c2) / (2.0* c1 * d_p1_p2));
		if (which_root != 0)theta = -theta;


		double p1_p2[2]{ (x[1] - x[0])/ d_p1_p2, (y[1] - y[0])/ d_p1_p2 };
		double c1_axis[2]{p1_p2[0] * std::cos(theta) - p1_p2[1] * std::sin(theta), p1_p2[1] * std::cos(theta) + p1_p2[0] * std::sin(theta) };

		ee_xya[0] = x[0] + c1_axis[0] * c1;
		ee_xya[1] = y[0] + c1_axis[1] * c1 - d;
		ee_xya[2] = input[2];

		return 0;
	}

	class PlanarDeltaInverseKinematicSolver :public aris::dynamic::InverseKinematicSolver {
	public:
		auto virtual kinPos()->int override {
			double input[4], output[4];

			auto dh = dynamic_cast<aris::dynamic::MatrixVariable*>(model()->findVariable("dh"))->data().data();

			model()->getOutputPos(output);
			if (auto ret = planarDeltaInverse(dh, output, 0, input))
				return ret;

			//// ee //
			//double pe[6]{ output[0],output[1],output[2],output[3], 0.0, 0.0 }, pp[3];
			//model()->generalMotionPool()[0].makI()->setPe(*model()->generalMotionPool()[0].makJ(), pe, "321");

			//// up //
			//s_fill(1, 3, 0.0, pe);
			//pe[3] = -pe[3];
			//model()->jointPool()[15].makJ()->setPe(*model()->jointPool()[15].makI(), pe, "321");

			//// link1 //
			//s_fill(1, 6, 0.0, pe);
			//pe[5] = input[0];
			//model()->jointPool()[0].makI()->setPe(*model()->jointPool()[0].makJ(), pe, "123");

			//s_fill(1, 6, 0.0, pe);
			//pe[5] = input[1];
			//model()->jointPool()[5].makI()->setPe(*model()->jointPool()[5].makJ(), pe, "123");

			//s_fill(1, 6, 0.0, pe);
			//pe[5] = input[2];
			//model()->jointPool()[10].makI()->setPe(*model()->jointPool()[10].makJ(), pe, "123");

			//// link2&3 //
			//s_fill(1, 6, 0.0, pe);
			//pe[5] = PI / 2;
			//model()->jointPool()[3].makI()->getPp(*model()->jointPool()[1].makJ(), pp);
			//s_sov_ab(pp, pe + 3, "312");
			//model()->jointPool()[1].makI()->setPe(*model()->jointPool()[1].makJ(), pe, "312");

			//model()->jointPool()[4].makI()->getPp(*model()->jointPool()[2].makJ(), pp);
			//s_sov_ab(pp, pe + 3, "312");
			//model()->jointPool()[2].makI()->setPe(*model()->jointPool()[2].makJ(), pe, "312");

			//model()->jointPool()[8].makI()->getPp(*model()->jointPool()[6].makJ(), pp);
			//s_sov_ab(pp, pe + 3, "312");
			//model()->jointPool()[6].makI()->setPe(*model()->jointPool()[6].makJ(), pe, "312");

			//model()->jointPool()[9].makI()->getPp(*model()->jointPool()[7].makJ(), pp);
			//s_sov_ab(pp, pe + 3, "312");
			//model()->jointPool()[7].makI()->setPe(*model()->jointPool()[7].makJ(), pe, "312");

			//model()->jointPool()[13].makI()->getPp(*model()->jointPool()[11].makJ(), pp);
			//s_sov_ab(pp, pe + 3, "312");
			//model()->jointPool()[11].makI()->setPe(*model()->jointPool()[11].makJ(), pe, "312");

			//model()->jointPool()[14].makI()->getPp(*model()->jointPool()[12].makJ(), pp);
			//s_sov_ab(pp, pe + 3, "312");
			//model()->jointPool()[12].makI()->setPe(*model()->jointPool()[12].makJ(), pe, "312");

			model()->motionPool()[0].setMp(input[0]);
			model()->motionPool()[1].setMp(input[1]);
			model()->motionPool()[2].setMp(input[2]);
			return 0;
		}

		PlanarDeltaInverseKinematicSolver() = default;
	};
	class PlanarDeltaForwardKinematicSolver :public aris::dynamic::ForwardKinematicSolver {
	public:
		auto virtual kinPos()->int override {
			double input[4], output[4];

			auto dh = dynamic_cast<aris::dynamic::MatrixVariable*>(model()->findVariable("dh"))->data().data();

			model()->getInputPos(input);
			if (auto ret = planarDeltaForward(dh, input, 0, output))
				return ret;

			// ee //
			double pe[6]{ output[0], output[1], 0.0, output[2], 0.0, 0.0 };
			model()->generalMotionPool()[0].makI()->setPe(*model()->generalMotionPool()[0].makJ(), pe, "321");

			//// up //
			//s_fill(1, 3, 0.0, pe);
			//pe[3] = -pe[3];
			//model()->jointPool()[15].makJ()->setPe(*model()->jointPool()[15].makI(), pe, "321");

			//// link1 //
			//s_fill(1, 6, 0.0, pe);
			//pe[5] = input[0];
			//model()->jointPool()[0].makI()->setPe(*model()->jointPool()[0].makJ(), pe, "123");

			//s_fill(1, 6, 0.0, pe);
			//pe[5] = input[1];
			//model()->jointPool()[5].makI()->setPe(*model()->jointPool()[5].makJ(), pe, "123");

			//s_fill(1, 6, 0.0, pe);
			//pe[5] = input[2];
			//model()->jointPool()[10].makI()->setPe(*model()->jointPool()[10].makJ(), pe, "123");

			//// link2&3 //
			//s_fill(1, 6, 0.0, pe);
			//pe[5] = PI / 2;
			//model()->jointPool()[3].makI()->getPp(*model()->jointPool()[1].makJ(), pp);
			//s_sov_ab(pp, pe + 3, "312");
			//model()->jointPool()[1].makI()->setPe(*model()->jointPool()[1].makJ(), pe, "312");

			//model()->jointPool()[4].makI()->getPp(*model()->jointPool()[2].makJ(), pp);
			//s_sov_ab(pp, pe + 3, "312");
			//model()->jointPool()[2].makI()->setPe(*model()->jointPool()[2].makJ(), pe, "312");

			//model()->jointPool()[8].makI()->getPp(*model()->jointPool()[6].makJ(), pp);
			//s_sov_ab(pp, pe + 3, "312");
			//model()->jointPool()[6].makI()->setPe(*model()->jointPool()[6].makJ(), pe, "312");

			//model()->jointPool()[9].makI()->getPp(*model()->jointPool()[7].makJ(), pp);
			//s_sov_ab(pp, pe + 3, "312");
			//model()->jointPool()[7].makI()->setPe(*model()->jointPool()[7].makJ(), pe, "312");

			//model()->jointPool()[13].makI()->getPp(*model()->jointPool()[11].makJ(), pp);
			//s_sov_ab(pp, pe + 3, "312");
			//model()->jointPool()[11].makI()->setPe(*model()->jointPool()[11].makJ(), pe, "312");

			//model()->jointPool()[14].makI()->getPp(*model()->jointPool()[12].makJ(), pp);
			//s_sov_ab(pp, pe + 3, "312");
			//model()->jointPool()[12].makI()->setPe(*model()->jointPool()[12].makJ(), pe, "312");

			for (auto &m : model()->generalMotionPool()) m.updP();
			return 0;
		}

		PlanarDeltaForwardKinematicSolver() = default;
	};

	/*auto ARIS_API createModelPlanarDelta(const PlanarDeltaParam &param)->std::unique_ptr<aris::dynamic::Model> {
		DeltaFullParam full_param;
		full_param.a1 = full_param.a2 = full_param.a3 = param.a;
		full_param.b1 = full_param.b2 = full_param.b3 = param.b;
		full_param.c1 = full_param.c2 = full_param.c3 = param.c;
		full_param.d1 = full_param.d2 = full_param.d3 = param.d;
		full_param.e1 = full_param.e2 = full_param.e3 = param.e;

		full_param.theta1 = 0.0;
		full_param.theta2 = aris::PI * 2 / 3;
		full_param.theta3 = -aris::PI * 2 / 3;

		s_vc(6, param.tool0_pe, full_param.tool0_pe);
		full_param.tool0_pe_type = param.tool0_pe_type;

		s_vc(6, param.base2ref_pe, full_param.base2ref_pe);
		full_param.base2ref_pe_type = param.base2ref_pe_type;

		full_param.iv_vec = param.iv_vec;

		full_param.mot_frc_vec = param.mot_frc_vec;

		return createModelDelta(full_param);
	}*/
	auto ARIS_API createModelPlanarDelta(const PlanarDeltaFullParam &param)->std::unique_ptr<aris::dynamic::Model> {
		std::unique_ptr<aris::dynamic::Model> model(new aris::dynamic::Model);

		model->variablePool().add<aris::dynamic::MatrixVariable>("dh", aris::core::Matrix({
			param.a1, param.b1, param.c1,
			param.a2, param.b2, param.c2, 
			param.d
			}));

		// 设置重力 //
		const double gravity[6]{ 0.0,0.0,-9.8,0.0,0.0,0.0 };
		model->environment().setGravity(gravity);

		// add part //
		const double default_iv[10]{ 1,0,0,0,0,0,0,0,0,0 };
		auto &p11 = model->partPool().add<Part>("L11", param.iv_vec.size() == 4 ? param.iv_vec[0].data() : default_iv);
		auto &p12 = model->partPool().add<Part>("L12", param.iv_vec.size() == 4 ? param.iv_vec[1].data() : default_iv);
		auto &p21 = model->partPool().add<Part>("L31", param.iv_vec.size() == 4 ? param.iv_vec[0].data() : default_iv);
		auto &p22 = model->partPool().add<Part>("L32", param.iv_vec.size() == 4 ? param.iv_vec[1].data() : default_iv);
		auto &pup = model->partPool().add<Part>("UP", param.iv_vec.size() == 4 ? param.iv_vec[3].data() : default_iv);
		auto &pee = model->partPool().add<Part>("EE", param.iv_vec.size() == 4 ? param.iv_vec[3].data() : default_iv);

		// add geometry //

		// add joint //
		double axis[3]{ 0,0,1 };
		double r11_pos[3]{ param.a1           ,          0.0, 0.0 };
		double r12_pos[3]{ param.a1 + param.b1,  param.c1 / 2, 0.0 };
		double r13_pos[3]{ param.a1 + param.b1, -param.c1 / 2, 0.0 };
		
		double r21_pos[3]{ param.a2           ,           0.0, 0.0 };
		double r22_pos[3]{ param.a2 + param.b2,  param.c2 / 2, 0.0 };
		double r23_pos[3]{ param.a2 + param.b2, -param.c2 / 2, 0.0 };

		auto &r11 = model->addRevoluteJoint(p11, model->ground(), r11_pos, axis);
		auto &r12 = model->addRevoluteJoint(p12, p11, r12_pos, axis);
		auto &r13 = model->addRevoluteJoint(pup, p11, r13_pos, axis);

		auto &r21 = model->addRevoluteJoint(p21, model->ground(), r21_pos, axis);
		auto &r22 = model->addRevoluteJoint(p22, p21, r22_pos, axis);
		auto &r23 = model->addRevoluteJoint(pup, p21, r23_pos, axis);


		// 求正解计算末端位置 //
		double input_0[4]{ 0,0,0,0 };
		double output_0[4]{ 0,0,0,0 };
		planarDeltaForward(dynamic_cast<aris::dynamic::MatrixVariable&>(model->variablePool()[0]).data().data(), input_0, 0, output_0);

		// 得到末端
		double re_pos[3]{ output_0[0], output_0[1], output_0[2] };
		double re_axis[3]{ 0,0,1 };
		auto &re = model->addRevoluteJoint(pee, pup, re_pos, re_axis);
		// add actuation //
		auto &m1 = model->addMotion(r11);
		auto &m2 = model->addMotion(r21);
		auto &m3 = model->addMotion(re);

		const double default_mot_frc[3]{ 0.0, 0.0, 0.0 };
		m1.setFrcCoe(param.mot_frc_vec.size() == 6 ? param.mot_frc_vec[0].data() : default_mot_frc);
		m2.setFrcCoe(param.mot_frc_vec.size() == 6 ? param.mot_frc_vec[1].data() : default_mot_frc);
		m3.setFrcCoe(param.mot_frc_vec.size() == 6 ? param.mot_frc_vec[2].data() : default_mot_frc);

		// add ee general motion //
		double ee_i_pm[16]{
			1,0,0,output_0[0],
			0,1,0,output_0[1],
			0,0,1,output_0[2],
			0,0,0,1 };
		double ee_j_pm[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };

		auto &makI = pee.addMarker("tool0", ee_i_pm);
		auto &makJ = model->ground().addMarker("wobj0", ee_j_pm);
		model->variablePool().add<aris::dynamic::MatrixVariable>("tool0_axis_home", aris::core::Matrix(1, 6, 0.0));
		auto &ee = model->generalMotionPool().add<aris::dynamic::PlanarMotion>("tool", &makI, &makJ, false);

		// change robot pose wrt ground //

		// add tools and wobj //
		for (int i = 1; i < 17; ++i) {
			pee.addMarker("tool" + std::to_string(i), ee_i_pm);
		}
		for (int i = 1; i < 33; ++i) model->ground().markerPool().add<aris::dynamic::Marker>("wobj" + std::to_string(i), ee_j_pm);

		// add solver
		auto &inverse_kinematic = model->solverPool().add<aris::dynamic::PlanarDeltaInverseKinematicSolver>();
		auto &forward_kinematic = model->solverPool().add<aris::dynamic::PlanarDeltaForwardKinematicSolver>();
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
		aris::core::class_<DeltaInverseKinematicSolver>("DeltaInverseKinematicSolver")
			.inherit<InverseKinematicSolver>()
			;

		aris::core::class_<DeltaForwardKinematicSolver>("DeltaForwardKinematicSolver")
			.inherit<ForwardKinematicSolver>()
			;

		aris::core::class_<PlanarDeltaInverseKinematicSolver>("PlanarDeltaInverseKinematicSolver")
			.inherit<InverseKinematicSolver>()
			;

		aris::core::class_<PlanarDeltaForwardKinematicSolver>("PlanarDeltaForwardKinematicSolver")
			.inherit<ForwardKinematicSolver>()
			;
	}
}
