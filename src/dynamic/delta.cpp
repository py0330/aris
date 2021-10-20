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
#include "aris/dynamic/delta.hpp"

namespace aris::dynamic{

	auto deltaInverse(const double *param, const double *ee_xyza, int which_root, double *input)->int {
		const double &a = param[0];
		const double &b = param[1];
		const double &c = param[2];
		const double &d = param[3];
		const double &e = param[4];

		// 此处将点转回到x z平面，因此与构造delta的地方不太一样 //
		const double theta[] = { 0.0, -PI * 2 / 3, PI * 2 / 3 };

		for (int i = 0; i < 3; ++i) {
			auto x = ee_xyza[0] * std::cos(theta[i]) - ee_xyza[1] * std::sin(theta[i]);
			auto y = ee_xyza[0] * std::sin(theta[i]) + ee_xyza[1] * std::cos(theta[i]);
			auto z = ee_xyza[2];

			if (std::abs(d) < std::abs(y)) return -1;

			auto d1 = std::sqrt(d * d - y * y);
			auto k = x + e - a;
			auto l = sqrt(k * k + z * z);

			if (b + l < d1 || b + d1 < l || d1 + l < b)
				return -2;

			input[i] =
				((0x01 << i) & which_root) ?
				-std::atan2(z, k) + std::acos((b * b + l * l - d1 * d1) / b / l / 2.0) :
				-std::atan2(z, k) - std::acos((b * b + l * l - d1 * d1) / b / l / 2.0);
		}

		input[3] = ee_xyza[3];
		return 0;
	}
	auto deltaForward(const double *param, const double *input, int which_root, double *ee_xyza)->int {
		const double &a = param[0];
		const double &b = param[1];
		const double &c = param[2];
		const double &d = param[3];
		const double &e = param[4];

		// 在所有的输入都为0的时候，以下方法的k矩阵奇异，因此需要特殊处理：
		if (std::abs(input[0] - 0.0) < 1e-5 && std::abs(input[1] - 0.0) < 1e-5 && std::abs(input[2] - 0.0) < 1e-5) {
			ee_xyza[0] = 0.0;
			ee_xyza[1] = 0.0;
			ee_xyza[2] = -std::sqrt(d*d - (a + b - e)*(a + b - e));
			ee_xyza[3] = input[3];
			return 0;
		}

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

		// 每根支联的转角
		const double theta[] = { 0.0, PI * 2 / 3, -PI * 2 / 3 };

		// 计算 p1
		double p1[9];
		for (int i = 0; i < 3; ++i) {
			p1[0 + i * 3] = (a + std::cos(input[i]) * b) * std::cos(theta[i]);
			p1[1 + i * 3] = (a + std::cos(input[i]) * b) * std::sin(theta[i]);
			p1[2 + i * 3] = -std::sin(input[i]) * b;
		}

		// 计算 p2
		double p2[9];
		for (int i = 0; i < 3; ++i) {
			p2[0 + i * 3] = e * std::cos(theta[i]);
			p2[1 + i * 3] = e * std::sin(theta[i]);
			p2[2 + i * 3] = 0.0;
		}

		// 计算 k
		double k[9];
		s_vc(9, p2, k);
		s_vs(9, p1, k);
		
		// 计算 s，随后k乘以2
		double s[3]{ d*d - k[0] * k[0] - k[1] * k[1]- k[2] * k[2], d*d - k[3] * k[3] - k[4] * k[4] - k[5] * k[5] ,d*d - k[6] * k[6] - k[7] * k[7] - k[8] * k[8] };
		s_nv(9, 2.0, k);
		
		// 计算k的逆
		double inv_k[9], u[9], tau[3], tau2[3];
		aris::Size p[3], rank;
		s_householder_utp(3, 3, k, u, tau, p, rank);
		s_householder_utp2pinv(3, 3, rank, u, tau, p, inv_k, tau2);
		
		// 计算 g&h
		double g[3], h[3]{-inv_k[0]- inv_k[1]- inv_k[2], -inv_k[3] - inv_k[4] - inv_k[5], -inv_k[6] - inv_k[7] - inv_k[8], };
		s_mm(3, 1, 3, inv_k, s, g);

		// 计算 A,B,C
		double A = h[0] * h[0] + h[1] * h[1] + h[2] * h[2];
		double B = 2.0*(h[0] * g[0] + h[1] * g[1] + h[2] * g[2]) - 1.0;
		double C = g[0] * g[0] + g[1] * g[1] + g[2] * g[2];

		// 此时有2个根 //
		double t;
		if (auto lambda = B * B - 4 * A*C; lambda < 0) return -1;
		else {
			// 这里用异或，找到z < 0的根
			t = ((which_root == 0) ^ (h[2] < 0)) ? (-B - std::sqrt(lambda)) / (2 * A) : (-B + std::sqrt(lambda)) / (2 * A);
			if (t < 0)return -2;
		}

		ee_xyza[0] = g[0] + h[0] * t;
		ee_xyza[1] = g[1] + h[1] * t;
		ee_xyza[2] = g[2] + h[2] * t;
		ee_xyza[3] = input[3];

		return 0;
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

	DeltaModel::~DeltaModel() = default;
	DeltaModel::DeltaModel(const DeltaParam &param) {
		//imp_->dh_ = aris::core::Matrix({param.a, param.b, param.c, param.d, param.e});
		this->variablePool().add<aris::dynamic::MatrixVariable>("dh", aris::core::Matrix({ param.a, param.b, param.c, param.d, param.e }));

		// 设置重力 //
		const double gravity[6]{ 0.0,0.0,-9.8,0.0,0.0,0.0 };
		this->environment().setGravity(gravity);

		// add part //
		const double default_iv[10]{ 1,0,0,0,0,0,0,0,0,0 };
		auto &p11 = this->partPool().add<Part>("L11", param.iv_vec.size() == 4 ? param.iv_vec[0].data() : default_iv);
		auto &p12 = this->partPool().add<Part>("L12", param.iv_vec.size() == 4 ? param.iv_vec[1].data() : default_iv);
		auto &p13 = this->partPool().add<Part>("L13", param.iv_vec.size() == 4 ? param.iv_vec[2].data() : default_iv);
		auto &p21 = this->partPool().add<Part>("L21", param.iv_vec.size() == 4 ? param.iv_vec[0].data() : default_iv);
		auto &p22 = this->partPool().add<Part>("L22", param.iv_vec.size() == 4 ? param.iv_vec[1].data() : default_iv);
		auto &p23 = this->partPool().add<Part>("L23", param.iv_vec.size() == 4 ? param.iv_vec[2].data() : default_iv);
		auto &p31 = this->partPool().add<Part>("L31", param.iv_vec.size() == 4 ? param.iv_vec[0].data() : default_iv);
		auto &p32 = this->partPool().add<Part>("L32", param.iv_vec.size() == 4 ? param.iv_vec[1].data() : default_iv);
		auto &p33 = this->partPool().add<Part>("L33", param.iv_vec.size() == 4 ? param.iv_vec[2].data() : default_iv);
		auto &pup = this->partPool().add<Part>("UP", param.iv_vec.size() == 4 ? param.iv_vec[3].data() : default_iv);
		auto &pee = this->partPool().add<Part>("EE", param.iv_vec.size() == 4 ? param.iv_vec[3].data() : default_iv);

		// add geometry //
		ground().geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\aris\\aris-1.5.0\\resource\\test_dynamic\\delta\\base.xmt_txt");
		pup.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\aris\\aris-1.5.0\\resource\\test_dynamic\\delta\\up.xmt_txt");
		pee.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\aris\\aris-1.5.0\\resource\\test_dynamic\\delta\\ee.xmt_txt");
		double geo_p13_prt_pm[16]{ 1,0,0,0,0,1,0,-param.c,0,0,1,0,0,0,0,1 };
		double geo_rot_pm[16], geo_local_pm[16];
		p11.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\aris\\aris-1.5.0\\resource\\test_dynamic\\delta\\p1.xmt_txt");
		p12.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\aris\\aris-1.5.0\\resource\\test_dynamic\\delta\\p2.xmt_txt");
		p13.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\aris\\aris-1.5.0\\resource\\test_dynamic\\delta\\p2.xmt_txt", geo_p13_prt_pm);
		s_eye(4, geo_rot_pm);
		s_rmz(PI * 2 / 3, geo_rot_pm, 4);
		s_pm_dot_pm(geo_rot_pm, geo_p13_prt_pm, geo_local_pm);
		p21.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\aris\\aris-1.5.0\\resource\\test_dynamic\\delta\\p1.xmt_txt", geo_rot_pm);
		p22.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\aris\\aris-1.5.0\\resource\\test_dynamic\\delta\\p2.xmt_txt", geo_rot_pm);
		p23.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\aris\\aris-1.5.0\\resource\\test_dynamic\\delta\\p2.xmt_txt", geo_local_pm);
		s_eye(4, geo_rot_pm);
		s_rmz(-PI * 2 / 3, geo_rot_pm, 4);
		s_pm_dot_pm(geo_rot_pm, geo_p13_prt_pm, geo_local_pm);
		p31.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\aris\\aris-1.5.0\\resource\\test_dynamic\\delta\\p1.xmt_txt", geo_rot_pm);
		p32.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\aris\\aris-1.5.0\\resource\\test_dynamic\\delta\\p2.xmt_txt", geo_rot_pm);
		p33.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\aris\\aris-1.5.0\\resource\\test_dynamic\\delta\\p2.xmt_txt", geo_local_pm);

		// add joint //
		double r11_pos[3]{ param.a          ,          0.0, 0.0 };
		double s12_pos[3]{ param.a + param.b,  param.c / 2, 0.0 };
		double s13_pos[3]{ param.a + param.b, -param.c / 2, 0.0 };
		double s14_pos[3]{ param.e          ,  param.c / 2, -std::sqrt(param.d*param.d - (param.a + param.b - param.e)*(param.a + param.b - param.e)) };
		double s15_pos[3]{ param.e          , -param.c / 2, -std::sqrt(param.d*param.d - (param.a + param.b - param.e)*(param.a + param.b - param.e)) };
		double r11_axis[6]{ 0.0, 1.0, 0.0 };
		double u12_first_axis[3]{ 0.0, 1.0, 0.0 };
		double u12_second_axis[3]{ s14_pos[2] - s12_pos[2], 0.0, -s14_pos[0] + s12_pos[0] };
		double r21_pos[3], s22_pos[3], s23_pos[3], s24_pos[3], s25_pos[3], r21_axis[3], u22_first_axis[3], u22_second_axis[3];
		double r31_pos[3], s32_pos[3], s33_pos[3], s34_pos[3], s35_pos[3], r31_axis[3], u32_first_axis[3], u32_second_axis[3];

		double rm2[9], rm3[9];
		s_rmz(PI * 2 / 3, rm2);
		s_rmz(-PI * 2 / 3, rm3);

		s_mm(3, 1, 3, rm2, r11_pos, r21_pos);
		s_mm(3, 1, 3, rm2, s12_pos, s22_pos);
		s_mm(3, 1, 3, rm2, s13_pos, s23_pos);
		s_mm(3, 1, 3, rm2, s14_pos, s24_pos);
		s_mm(3, 1, 3, rm2, s15_pos, s25_pos);
		s_mm(3, 1, 3, rm2, r11_axis, r21_axis);
		s_mm(3, 1, 3, rm2, u12_first_axis, u22_first_axis);
		s_mm(3, 1, 3, rm2, u12_second_axis, u22_second_axis);

		s_mm(3, 1, 3, rm3, r11_pos, r31_pos);
		s_mm(3, 1, 3, rm3, s12_pos, s32_pos);
		s_mm(3, 1, 3, rm3, s13_pos, s33_pos);
		s_mm(3, 1, 3, rm3, s14_pos, s34_pos);
		s_mm(3, 1, 3, rm3, s15_pos, s35_pos);
		s_mm(3, 1, 3, rm3, r11_axis, r31_axis);
		s_mm(3, 1, 3, rm3, u12_first_axis, u32_first_axis);
		s_mm(3, 1, 3, rm3, u12_second_axis, u32_second_axis);

		auto &r11 = this->addRevoluteJoint(p11, this->ground(), r11_pos, r11_axis);
		auto &u12 = this->addUniversalJoint(p12, p11, s12_pos, u12_second_axis, u12_first_axis);
		auto &u13 = this->addUniversalJoint(p13, p11, s13_pos, u12_second_axis, u12_first_axis);
		auto &s14 = this->addSphericalJoint(pup, p12, s14_pos);
		auto &s15 = this->addSphericalJoint(pup, p13, s15_pos);

		auto &r21 = this->addRevoluteJoint(p21, this->ground(), r21_pos, r21_axis);
		auto &u22 = this->addUniversalJoint(p22, p21, s22_pos, u22_second_axis, u22_first_axis);
		auto &u23 = this->addUniversalJoint(p23, p21, s23_pos, u22_second_axis, u22_first_axis);
		auto &s24 = this->addSphericalJoint(pup, p22, s24_pos);
		auto &s25 = this->addSphericalJoint(pup, p23, s25_pos);

		auto &r31 = this->addRevoluteJoint(p31, this->ground(), r31_pos, r31_axis);
		auto &u32 = this->addUniversalJoint(p32, p31, s32_pos, u32_second_axis, u32_first_axis);
		auto &u33 = this->addUniversalJoint(p33, p31, s33_pos, u32_second_axis, u32_first_axis);
		auto &s34 = this->addSphericalJoint(pup, p32, s34_pos);
		auto &s35 = this->addSphericalJoint(pup, p33, s35_pos);

		double re_pos[3]{ 0,0,-std::sqrt(param.d*param.d - (param.a + param.b - param.e)*(param.a + param.b - param.e)) };
		double re_axis[3]{ 0,0,1 };
		auto &re = this->addRevoluteJoint(pee, pup, re_pos, re_axis);
		// add actuation //
		auto &m1 = this->addMotion(r11);
		auto &m2 = this->addMotion(r21);
		auto &m3 = this->addMotion(r31);
		auto &m4 = this->addMotion(re);

		const double default_mot_frc[3]{ 0.0, 0.0, 0.0 };
		m1.setFrcCoe(param.mot_frc_vec.size() == 6 ? param.mot_frc_vec[0].data() : default_mot_frc);
		m2.setFrcCoe(param.mot_frc_vec.size() == 6 ? param.mot_frc_vec[1].data() : default_mot_frc);
		m3.setFrcCoe(param.mot_frc_vec.size() == 6 ? param.mot_frc_vec[2].data() : default_mot_frc);
		m4.setFrcCoe(param.mot_frc_vec.size() == 6 ? param.mot_frc_vec[3].data() : default_mot_frc);

		// add ee general motion //
		double ee_i_pm[16]{ 1,0,0,0,0,1,0,0,0,0,1,-std::sqrt(param.d*param.d - (param.a + param.b - param.e)*(param.a + param.b - param.e)),0,0,0,1 };
		double ee_j_pm[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };

		auto &makI = pee.addMarker("tool0", ee_i_pm);
		auto &makJ = this->ground().addMarker("wobj0", ee_j_pm);
		this->variablePool().add<aris::dynamic::MatrixVariable>("tool0_axis_home", aris::core::Matrix(1, 6, 0.0));
		auto &ee = this->generalMotionPool().add<aris::dynamic::XyztMotion>("tool", &makI, &makJ, false);

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
		for (int i = 1; i < 33; ++i) this->ground().markerPool().add<aris::dynamic::Marker>("wobj" + std::to_string(i), ee_j_pm);

		// add solver
		auto &inverse_kinematic = this->solverPool().add<aris::dynamic::DeltaInverseKinematicSolver>();
		auto &forward_kinematic = this->solverPool().add<aris::dynamic::DeltaForwardKinematicSolver>();
		auto &inverse_dynamic = this->solverPool().add<aris::dynamic::InverseDynamicSolver>();
		auto &forward_dynamic = this->solverPool().add<aris::dynamic::ForwardDynamicSolver>();

		// make topology correct // 
		for (auto &m : this->motionPool())m.activate(true);
		for (auto &gm : this->generalMotionPool())gm.activate(false);
		for (auto &f : this->forcePool())f.activate(false);

		this->init();
	}
	DeltaModel::DeltaModel(DeltaModel&&)=default;
	DeltaModel& DeltaModel::operator=(DeltaModel&&)=default;
	
	auto ARIS_API createModelDelta(const DeltaParam &param)->std::unique_ptr<aris::dynamic::Model> {
		std::unique_ptr<aris::dynamic::Model> model(new aris::dynamic::Model);

		model->variablePool().add<aris::dynamic::MatrixVariable>("dh", aris::core::Matrix({ param.a, param.b, param.c, param.d, param.e }));

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
		model->ground().geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\aris\\aris-1.5.0\\resource\\test_dynamic\\delta\\base.xmt_txt");
		pup.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\aris\\aris-1.5.0\\resource\\test_dynamic\\delta\\up.xmt_txt");
		pee.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\aris\\aris-1.5.0\\resource\\test_dynamic\\delta\\ee.xmt_txt");
		double geo_p13_prt_pm[16]{ 1,0,0,0,0,1,0,-param.c,0,0,1,0,0,0,0,1 };
		double geo_rot_pm[16], geo_local_pm[16];
		p11.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\aris\\aris-1.5.0\\resource\\test_dynamic\\delta\\p1.xmt_txt");
		p12.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\aris\\aris-1.5.0\\resource\\test_dynamic\\delta\\p2.xmt_txt");
		p13.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\aris\\aris-1.5.0\\resource\\test_dynamic\\delta\\p2.xmt_txt", geo_p13_prt_pm);
		s_eye(4, geo_rot_pm);
		s_rmz(PI * 2 / 3, geo_rot_pm, 4);
		s_pm_dot_pm(geo_rot_pm, geo_p13_prt_pm, geo_local_pm);
		p21.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\aris\\aris-1.5.0\\resource\\test_dynamic\\delta\\p1.xmt_txt", geo_rot_pm);
		p22.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\aris\\aris-1.5.0\\resource\\test_dynamic\\delta\\p2.xmt_txt", geo_rot_pm);
		p23.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\aris\\aris-1.5.0\\resource\\test_dynamic\\delta\\p2.xmt_txt", geo_local_pm);
		s_eye(4, geo_rot_pm);
		s_rmz(-PI * 2 / 3, geo_rot_pm, 4);
		s_pm_dot_pm(geo_rot_pm, geo_p13_prt_pm, geo_local_pm);
		p31.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\aris\\aris-1.5.0\\resource\\test_dynamic\\delta\\p1.xmt_txt", geo_rot_pm);
		p32.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\aris\\aris-1.5.0\\resource\\test_dynamic\\delta\\p2.xmt_txt", geo_rot_pm);
		p33.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\aris\\aris-1.5.0\\resource\\test_dynamic\\delta\\p2.xmt_txt", geo_local_pm);

		// add joint //
		double r11_pos[3]{ param.a          ,          0.0, 0.0 };
		double s12_pos[3]{ param.a + param.b,  param.c / 2, 0.0 };
		double s13_pos[3]{ param.a + param.b, -param.c / 2, 0.0 };
		double s14_pos[3]{ param.e          ,  param.c / 2, -std::sqrt(param.d*param.d - (param.a + param.b - param.e)*(param.a + param.b - param.e)) };
		double s15_pos[3]{ param.e          , -param.c / 2, -std::sqrt(param.d*param.d - (param.a + param.b - param.e)*(param.a + param.b - param.e)) };
		double r11_axis[6]{ 0.0, 1.0, 0.0 };
		double u12_first_axis[3]{ 0.0, 1.0, 0.0 };
		double u12_second_axis[3]{ s14_pos[2] - s12_pos[2], 0.0, -s14_pos[0] + s12_pos[0] };
		double r21_pos[3], s22_pos[3], s23_pos[3], s24_pos[3], s25_pos[3], r21_axis[3], u22_first_axis[3], u22_second_axis[3];
		double r31_pos[3], s32_pos[3], s33_pos[3], s34_pos[3], s35_pos[3], r31_axis[3], u32_first_axis[3], u32_second_axis[3];

		double rm2[9], rm3[9];
		s_rmz(PI * 2 / 3, rm2);
		s_rmz(-PI * 2 / 3, rm3);

		s_mm(3, 1, 3, rm2, r11_pos, r21_pos);
		s_mm(3, 1, 3, rm2, s12_pos, s22_pos);
		s_mm(3, 1, 3, rm2, s13_pos, s23_pos);
		s_mm(3, 1, 3, rm2, s14_pos, s24_pos);
		s_mm(3, 1, 3, rm2, s15_pos, s25_pos);
		s_mm(3, 1, 3, rm2, r11_axis, r21_axis);
		s_mm(3, 1, 3, rm2, u12_first_axis, u22_first_axis);
		s_mm(3, 1, 3, rm2, u12_second_axis, u22_second_axis);

		s_mm(3, 1, 3, rm3, r11_pos, r31_pos);
		s_mm(3, 1, 3, rm3, s12_pos, s32_pos);
		s_mm(3, 1, 3, rm3, s13_pos, s33_pos);
		s_mm(3, 1, 3, rm3, s14_pos, s34_pos);
		s_mm(3, 1, 3, rm3, s15_pos, s35_pos);
		s_mm(3, 1, 3, rm3, r11_axis, r31_axis);
		s_mm(3, 1, 3, rm3, u12_first_axis, u32_first_axis);
		s_mm(3, 1, 3, rm3, u12_second_axis, u32_second_axis);

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

		double re_pos[3]{ 0,0,-std::sqrt(param.d*param.d - (param.a + param.b - param.e)*(param.a + param.b - param.e)) };
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

		// add ee general motion //
		double ee_i_pm[16]{ 1,0,0,0,0,1,0,0,0,0,1,-std::sqrt(param.d*param.d - (param.a + param.b - param.e)*(param.a + param.b - param.e)),0,0,0,1 };
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

	ARIS_REGISTRATION{
		aris::core::class_<DeltaInverseKinematicSolver>("DeltaInverseKinematicSolver")
			.inherit<InverseKinematicSolver>()
			;

		aris::core::class_<DeltaForwardKinematicSolver>("DeltaForwardKinematicSolver")
			.inherit<ForwardKinematicSolver>()
			;
		
		aris::core::class_<DeltaModel>("DeltaModel")
			.inherit<aris::dynamic::Model>()
			;
	}
}
