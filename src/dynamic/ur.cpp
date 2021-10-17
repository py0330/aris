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
#include "aris/dynamic/ur.hpp"
#include "aris/core/reflection.hpp"

namespace aris::dynamic
{
	struct Relation
	{
		struct Block 
		{ 
			const Constraint* constraint; 
			bool is_I;
			double pmI[16], pmJ[16];
		};

		const Part *prtI; // 对角块对应的Part
		const Part *prtJ; // rd对应的Part
		Size dim_, size;
		std::vector<Block> cst_pool_;
	};
	struct Diag
	{
		// D * C * P =[I  C]
		//            [0  0]
		// 对于存在多个约束的relation来说，P有意义
		std::vector<Size> p_vec;
		Size *p;
		double dm[36], iv[10];
		double pm1[16], pm2[16], *pm, *last_pm;
		double xp[6], bp[6], *bc, *xc;
		std::vector<double> bc_vec, xc_vec;

		double *cmI, *cmJ, *cmU, *cmT; // 仅仅在计算多个关节构成的relation时有用

		Size rows;// in F
		const Part *part;
		Diag *rd;//related diag, for row addition
		Relation rel_;

		std::function<void(Diag*)> upd_d;
		std::function<void(Diag*)> cpt_cp_from_pm;
	};
	struct Remainder
	{
		struct Block { Diag* diag; bool is_I; };

		Diag *i_diag, *j_diag;
		double xp[6], bp[6];
		double *cmI, *cmJ, *bc, *xc;
		std::vector<double> cmI_vec, cmJ_vec, bc_vec, xc_vec;

		std::vector<Block> cm_blk_series;
		Relation rel_;
	};
	struct SubSystem
	{
		std::vector<Diag> diag_pool_;
		std::vector<Remainder> remainder_pool_;

		Size fm, fn, fr, gm, gn;

		double *F, *FU, *FT;
		Size* FP;
		double *G, *GU, *GT;
		Size *GP;

		double *S;
		double *QT_DOT_G;

		double *xpf, *xcf;
		double *bpf, *bcf;
		double *beta;

		bool has_ground_;
		double error_, max_error_;
		Size iter_count_, max_iter_count_;

		auto hasGround()const noexcept->bool { return has_ground_; }
		// 从模型中跟新数据 //
		auto updMakPm()noexcept->void;
		auto updDmCm()noexcept->void;
		auto updDiagIv()noexcept->void;
		auto updCpToBc()noexcept->void;
		auto updCvToBc()noexcept->void;
		auto updCaToBc()noexcept->void;
		// 求解 //
		auto updError()noexcept->void;
		auto updF()noexcept->void;
		auto sovXp()noexcept->void;
		auto updG()noexcept->void;
		auto sovXc()noexcept->void;
		// 接口 //
		auto kinPos()noexcept->void;
		auto kinVel()noexcept->void;
		auto dynAccAndFce()noexcept->void;
	};

	struct UniversalSolver::Imp
	{
		std::vector<SubSystem> subsys_pool_;

		static auto one_constraint_upd_d(Diag *d)noexcept->void
		{
			d->rel_.cst_pool_[0].constraint->cptGlbDmFromPm(d->dm, d->rel_.cst_pool_[0].pmI, d->rel_.cst_pool_[0].pmJ);
			if (!d->rel_.cst_pool_.begin()->is_I)s_iv(36, d->dm);
		}
		static auto revolute_upd_d(Diag *d)noexcept->void
		{
			d->rel_.cst_pool_[0].constraint->cptGlbDmFromPm(d->dm, d->rel_.cst_pool_[0].pmI, d->rel_.cst_pool_[0].pmJ);
			if (!d->rel_.cst_pool_.begin()->is_I)s_iv(36, d->dm);
		}
		static auto prismatic_upd_d(Diag *d)noexcept->void
		{
			d->rel_.cst_pool_[0].constraint->cptGlbDmFromPm(d->dm, d->rel_.cst_pool_[0].pmI, d->rel_.cst_pool_[0].pmJ);
			if (!d->rel_.cst_pool_.begin()->is_I)s_iv(36, d->dm);
		}
		static auto normal_upd_d(Diag *d)noexcept->void
		{
			Size pos{ 0 };
			for (auto &c : d->rel_.cst_pool_)
			{
				double cmI_tem[36], cmJ_tem[36];
				c.constraint->cptGlbCmFromPm(cmI_tem, cmJ_tem, c.pmI, c.pmJ);

				s_mc(6, c.constraint->dim(), cmI_tem, c.constraint->dim(), (c.is_I ? d->cmI : d->cmJ) + pos, d->rel_.size);
				s_mc(6, c.constraint->dim(), cmJ_tem, c.constraint->dim(), (c.is_I ? d->cmJ : d->cmI) + pos, d->rel_.size);
				pos += c.constraint->dim();
			}

			double Q[36];
			s_householder_utp(6, d->rel_.size, d->cmI, d->cmU, d->cmT, d->p, d->rel_.dim_);
			s_householder_ut2qr(6, d->rel_.size, d->cmU, d->cmT, Q, d->cmU);

			double tem[36]{ 1,0,0,0,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0,1 };
			s_inv_um(6, d->cmU, d->rel_.size, tem, 6);
			s_mm(6, 6, 6, tem, 6, Q, dynamic::ColMajor{ 6 }, d->dm, 6);
		}

		static auto one_constraint_cpt_cp_from_pm(Diag *d)noexcept->void
		{
			d->rel_.cst_pool_[0].constraint->cptCpFromPm(d->bc, d->rel_.cst_pool_[0].pmI, d->rel_.cst_pool_[0].pmJ);
		}
		static auto revolute_cpt_cp_from_pm(Diag *d)noexcept->void
		{
			auto &c = d->rel_.cst_pool_[1];
			auto m = static_cast<const Motion*>(c.constraint);

			double rm[9], pm_j_should_be[16];
			s_rmz(m->mpInternal(), rm);

			s_vc(16, c.pmJ, pm_j_should_be);
			s_mm(3, 3, 3, c.pmJ, 4, rm, 3, pm_j_should_be, 4);

			double pm_j2i[16], ps_j2i[6];
			s_inv_pm_dot_pm(c.pmI, pm_j_should_be, pm_j2i);
			s_pm2ps(pm_j2i, ps_j2i);

			// motion所对应的cp在最后 //
			s_vc(m->axis(), ps_j2i, d->bc);
			s_vc(5 - m->axis(), ps_j2i + m->axis() + 1, d->bc + m->axis());
			d->bc[5] = ps_j2i[m->axis()];
		}
		static auto prismatic_cpt_cp_from_pm(Diag *d)noexcept->void
		{
			auto &c = d->rel_.cst_pool_[1];
			auto m = static_cast<const Motion*>(c.constraint);

			double pm_j_should_be[16];
			s_vc(16, c.pmJ, pm_j_should_be);
			s_va(3, m->mpInternal(), pm_j_should_be + m->axis(), 4, pm_j_should_be + 3, 4);

			double pm_j2i[16], ps_j2i[6];
			s_inv_pm_dot_pm(c.pmI, pm_j_should_be, pm_j2i);
			s_pm2ps(pm_j2i, ps_j2i);

			// motion所对应的cp在最后 //
			s_vc(m->axis(), ps_j2i, d->bc);
			s_vc(5 - m->axis(), ps_j2i + m->axis() + 1, d->bc + m->axis());
			d->bc[5] = ps_j2i[m->axis()];
		}
		static auto normal_cpt_cp_from_pm(Diag *d)noexcept->void
		{
			Size pos{ 0 };
			for (auto &c : d->rel_.cst_pool_)
			{
				c.constraint->cptCpFromPm(d->bc + pos, c.pmI, c.pmJ);
				pos += c.constraint->dim();
			}
		}
	};

	const double ZERO_THRESH = 0.00000001;
	int SIGN(double x) {
		return (x > 0) - (x < 0);
	}
	const double d1 = 0.089159;
	const double a2 = -0.42500;
	const double a3 = -0.39225;
	const double d4 = 0.10915;
	const double d5 = 0.09465;
	const double d6 = 0.0823;
	int inverse(const double* T, double* q_sols, double q6_des) {
		int num_sols = 0;
		//double T02 = -*T; T++; double T00 = *T; T++; double T01 = *T; T++; double T03 = -*T; T++;
		//double T12 = -*T; T++; double T10 = *T; T++; double T11 = *T; T++; double T13 = -*T; T++;
		//double T22 = *T; T++; double T20 = -*T; T++; double T21 = -*T; T++; double T23 = *T;
		double T01 = *T; T++; double T00 = *T; T++; double T02 = *T; T++; double T03 = *T; T++;
		double T11 = *T; T++; double T10 = *T; T++; double T12 = *T; T++; double T13 = *T; T++;
		double T21 = *T; T++; double T20 = *T; T++; double T22 = *T; T++; double T23 = *T;

		////////////////////////////// shoulder rotate joint (q1) //////////////////////////////
		double q1[2];
		{
			double A = d6 * T12 - T13;
			double B = d6 * T02 - T03;
			double R = A * A + B * B;
			if (fabs(A) < ZERO_THRESH) {
				double div;
				if (fabs(fabs(d4) - fabs(B)) < ZERO_THRESH)
					div = -SIGN(d4)*SIGN(B);
				else
					div = -d4 / B;
				double arcsin = asin(div);
				if (fabs(arcsin) < ZERO_THRESH)
					arcsin = 0.0;
				if (arcsin < 0.0)
					q1[0] = arcsin + 2.0*PI;
				else
					q1[0] = arcsin;
				q1[1] = PI - arcsin;
			}
			else if (fabs(B) < ZERO_THRESH) {
				double div;
				if (fabs(fabs(d4) - fabs(A)) < ZERO_THRESH)
					div = SIGN(d4)*SIGN(A);
				else
					div = d4 / A;
				double arccos = acos(div);
				q1[0] = arccos;
				q1[1] = 2.0*PI - arccos;
			}
			else if (d4*d4 > R) {
				return num_sols;
			}
			else {
				double arccos = acos(d4 / sqrt(R));
				double arctan = atan2(-B, A);
				double pos = arccos + arctan;
				double neg = -arccos + arctan;
				if (fabs(pos) < ZERO_THRESH)
					pos = 0.0;
				if (fabs(neg) < ZERO_THRESH)
					neg = 0.0;
				if (pos >= 0.0)
					q1[0] = pos;
				else
					q1[0] = 2.0*PI + pos;
				if (neg >= 0.0)
					q1[1] = neg;
				else
					q1[1] = 2.0*PI + neg;
			}
		}
		////////////////////////////////////////////////////////////////////////////////

		////////////////////////////// wrist 2 joint (q5) //////////////////////////////
		double q5[2][2];
		{
			for (int i = 0; i<2; i++) {
				double numer = (T03*sin(q1[i]) - T13 * cos(q1[i]) - d4);
				double div;
				if (fabs(fabs(numer) - fabs(d6)) < ZERO_THRESH)
					div = SIGN(numer) * SIGN(d6);
				else
					div = numer / d6;
				double arccos = acos(div);
				q5[i][0] = arccos;
				q5[i][1] = 2.0*PI - arccos;
			}
		}
		////////////////////////////////////////////////////////////////////////////////

		{
			for (int i = 0; i<2; i++) {
				for (int j = 0; j<2; j++) {
					double c1 = cos(q1[i]), s1 = sin(q1[i]);
					double c5 = cos(q5[i][j]), s5 = sin(q5[i][j]);
					double q6;
					////////////////////////////// wrist 3 joint (q6) //////////////////////////////
					if (fabs(s5) < ZERO_THRESH)
						q6 = q6_des;
					else {
						q6 = atan2(SIGN(s5)*-(T01*s1 - T11 * c1),
							SIGN(s5)*(T00*s1 - T10 * c1));
						if (fabs(q6) < ZERO_THRESH)
							q6 = 0.0;
						if (q6 < 0.0)
							q6 += 2.0*PI;
					}
					////////////////////////////////////////////////////////////////////////////////

					double q2[2], q3[2], q4[2];
					///////////////////////////// RRR joints (q2,q3,q4) ////////////////////////////
					double c6 = cos(q6), s6 = sin(q6);
					double x04x = -s5 * (T02*c1 + T12 * s1) - c5 * (s6*(T01*c1 + T11 * s1) - c6 * (T00*c1 + T10 * s1));
					double x04y = c5 * (T20*c6 - T21 * s6) - T22 * s5;
					double p13x = d5 * (s6*(T00*c1 + T10 * s1) + c6 * (T01*c1 + T11 * s1)) - d6 * (T02*c1 + T12 * s1) +
						T03 * c1 + T13 * s1;
					double p13y = T23 - d1 - d6 * T22 + d5 * (T21*c6 + T20 * s6);

					double c3 = (p13x*p13x + p13y * p13y - a2 * a2 - a3 * a3) / (2.0*a2*a3);
					if (fabs(fabs(c3) - 1.0) < ZERO_THRESH)
						c3 = SIGN(c3);
					else if (fabs(c3) > 1.0) {
						// TODO NO SOLUTION
						continue;
					}
					double arccos = acos(c3);
					q3[0] = arccos;
					q3[1] = 2.0*PI - arccos;
					double denom = a2 * a2 + a3 * a3 + 2 * a2*a3*c3;
					double s3 = sin(arccos);
					double A = (a2 + a3 * c3), B = a3 * s3;
					q2[0] = atan2((A*p13y - B * p13x) / denom, (A*p13x + B * p13y) / denom);
					q2[1] = atan2((A*p13y + B * p13x) / denom, (A*p13x - B * p13y) / denom);
					double c23_0 = cos(q2[0] + q3[0]);
					double s23_0 = sin(q2[0] + q3[0]);
					double c23_1 = cos(q2[1] + q3[1]);
					double s23_1 = sin(q2[1] + q3[1]);
					q4[0] = atan2(c23_0*x04y - s23_0 * x04x, x04x*c23_0 + x04y * s23_0);
					q4[1] = atan2(c23_1*x04y - s23_1 * x04x, x04x*c23_1 + x04y * s23_1);
					////////////////////////////////////////////////////////////////////////////////
					for (int k = 0; k<2; k++) {
						if (fabs(q2[k]) < ZERO_THRESH)
							q2[k] = 0.0;
						else if (q2[k] < 0.0) q2[k] += 2.0*PI;
						if (fabs(q4[k]) < ZERO_THRESH)
							q4[k] = 0.0;
						else if (q4[k] < 0.0) q4[k] += 2.0*PI;
						q_sols[num_sols * 6 + 0] = q1[i];    q_sols[num_sols * 6 + 1] = q2[k];
						q_sols[num_sols * 6 + 2] = q3[k];    q_sols[num_sols * 6 + 3] = q4[k];
						q_sols[num_sols * 6 + 4] = q5[i][j]; q_sols[num_sols * 6 + 5] = q6;
						num_sols++;
					}

				}
			}
		}
		return num_sols;
	}



	struct UrParamLocal{
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
		for (Size i = 0; i < 6; ++i){
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



	auto UrInverseKinematic(Model &m, SubSystem &sys, int which_root)->int{
		Part* GR = &m.partPool().at(0);
		Part* L1 = &m.partPool().at(1);
		Part* L2 = &m.partPool().at(2);
		Part* L3 = &m.partPool().at(3);
		Part* L4 = &m.partPool().at(4);
		Part* L5 = &m.partPool().at(5);
		Part* L6 = &m.partPool().at(6);

		Joint *R1 = &m.jointPool().at(0);
		Joint *R2 = &m.jointPool().at(1);
		Joint *R3 = &m.jointPool().at(2);
		Joint *R4 = &m.jointPool().at(3);
		Joint *R5 = &m.jointPool().at(4);
		Joint *R6 = &m.jointPool().at(5);

		GeneralMotion *ee = &dynamic_cast<GeneralMotion&>(m.generalMotionPool().at(0));

		// UR的机构有如下特点：
		// 1轴和2轴垂直且交于一点： A点
		// 2轴、3轴、4轴平行
		// 4轴、5轴垂直且交于一点： B点
		// 5轴、6轴垂直且交于一点： C点
		//
		//                    y6         o   ---
		//                                    *
		//                                    *
		//                    z5    ---  |    d4      (两个y轴在z方向)
		//                           *        *  
		//                           d3       *       (两个z轴在x方向)
		// y2  o [***d1***] o [***d2***] o   --- 
		//     | 
		//     z1           y3           y4       
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
		const double d1 = 0.425;
		const double d2 = 0.39225;
		const double d3 = 0.10915;
		const double d4 = -0.09465;


		// 以下也是尺寸变量，分别为 A 在地面参考系（ee的makI）和 E 在 D 中的坐标系
		const double A_pm[16]{
			1,0,0,0,
			0,1,0,0,
			0,0,1,0.089159,
			0,0,0,1
		};
		const double E_pm_in_D[16]{
			-1,0,0,0,
			0,0,1,0.0823,
			0,1,0,0,
			0,0,0,1
		};

		double q[6]{ 0 };


		double E_in_A[16];
		s_inv_pm_dot_pm(A_pm, *ee->mpm(), E_in_A);

		double D_in_A[16];
		s_pm_dot_inv_pm(E_in_A, E_pm_in_D, D_in_A);


		// 开始求1轴 //
		// 求第一根轴的位置，这里末端可能工作空间以外，此时末端离原点过近，判断方法为查看以下if //
		// 事实上这里可以有2个解
		if (d3 > std::sqrt(D_in_A[3] * D_in_A[3] + D_in_A[7] * D_in_A[7])) return -1;
		if (which_root & 0x04){
			q[0] = PI + std::atan2(D_in_A[7], D_in_A[3]) + std::asin(d3 / std::sqrt(D_in_A[3] * D_in_A[3] + D_in_A[7] * D_in_A[7]));
		}
		else{
			q[0] = std::atan2(D_in_A[7], D_in_A[3]) - std::asin(d3 / std::sqrt(D_in_A[3] * D_in_A[3] + D_in_A[7] * D_in_A[7]));
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
			R23456_pe[3] = R23456_pe[3]>PI ? R23456_pe[3] - PI : R23456_pe[3] + PI;
			R23456_pe[4] = 2 * PI - R23456_pe[4];
			R23456_pe[5] = R23456_pe[3]>PI ? R23456_pe[3] - PI : R23456_pe[3] + PI;
		}
		q[4] = R23456_pe[4];
		q[5] = R23456_pe[5];


		// 开始求2，3，4轴 // 
		double R1234_pm[16], B_in_A[3];
		s_pe2pm(std::array<double, 6>{0, 0, 0, q[0], R23456_pe[3], 0.0}.data(), R1234_pm, "321");
		s_vc(3, D_in_A + 3, 4, B_in_A, 1);
		s_va(3, -d4, R1234_pm + 2, 4, B_in_A, 1);
		s_va(3, -d3, R1234_pm + 1, 4, B_in_A, 1);

		double B_pos[3];
		s_inv_pm_dot_v3(R1_pm, B_in_A, B_pos);

		double l_square = B_pos[0] * B_pos[0] + B_pos[2] * B_pos[2];
		double l = std::sqrt(l_square);
		if (l > (d1 + d2) || l<(std::max(std::abs(d1), std::abs(d2)) - std::min(std::abs(d1), std::abs(d2))))return -2;


		if (which_root & 0x01)
		{
			q[2] = PI + std::acos((d1 * d1 + d2 * d2 - l_square) / (2 * d1 * d2));
			q[1] = std::acos((l_square + d1 * d1 - d2 * d2) / (2 * l * d1)) - std::atan2(B_pos[2], B_pos[0]);
			q[3] = R23456_pe[3] - q[1] - q[2];
		}
		else
		{
			q[2] = PI - std::acos((d1 * d1 + d2 * d2 - l_square) / (2 * d1 * d2));
			q[1] = -std::acos((l_square + d1 * d1 - d2 * d2) / (2 * l * d1)) - std::atan2(B_pos[2], B_pos[0]);
			q[3] = R23456_pe[3] - q[1] - q[2];
		}


		// 这里对每根轴做正负处理 //
		//q[0] = q[0] + PI; 这样可以和ur官方吻合
		//q[3] = q[3] + PI; 这样可以和ur官方吻合
		q[4] = -q[4];

		// 这里让每个电机角度都落在[0，2pi]
		for (Size i = 0; i < 6; ++i)
		{
			q[i] = q[i] < 0 ? q[i] + 2 * PI : q[i];
			q[i] = q[i] > 2 * PI ? q[i] - 2 * PI : q[i];
		}

		// 这里更新每个杆件
		for (aris::Size i = 0; i < 6; ++i)
		{
			double pe3[6]{ 0.0,0.0,0.0,0.0,0.0,0.0 }, pm[16], pm1[16];

			pe3[5] = q[i];
			s_pm_dot_pm(*m.jointPool().at(i).makJ()->pm(), s_pe2pm(pe3, pm, "123"), pm1);
			s_pm_dot_inv_pm(pm1, *m.jointPool().at(i).makI()->prtPm(), const_cast<double*>(*m.jointPool().at(i).makI()->fatherPart().pm()));

			auto last_mp = m.motionPool().at(i).mpInternal();
			m.motionPool().at(i).updP();
			while (m.motionPool().at(i).mpInternal() - last_mp > PI)m.motionPool().at(i).setMpInternal(m.motionPool().at(i).mpInternal() - 2 * PI);
			while (m.motionPool().at(i).mpInternal() - last_mp < -PI)m.motionPool().at(i).setMpInternal(m.motionPool().at(i).mpInternal() + 2 * PI);
		}


		return 0;
	}

	struct UrInverseKinematicSolver::Imp
	{
		int which_root_{ 0 };
		UrParamLocal puma_param;
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
		auto ee_mak_on_L6 = &imp_->ee->makI()->fatherPart() == imp_->L6 ? imp_->ee->makI() : imp_->ee->makJ();

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
			double R5_z_in_R4[3]{pm[2], pm[6], pm[10]};

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
	auto UrInverseKinematicSolver::setWhichRoot(int root_of_0_to_7)->void{	imp_->which_root_ = root_of_0_to_7; }
	auto UrInverseKinematicSolver::whichRoot()->int { return imp_->which_root_; }
	auto UrInverseKinematicSolver::kinPos()->int {
		if (imp_->which_root_ == 8) {
			int solution_num = 0;
			double diff_q[8][6];
			double diff_norm[8];

			for (int i = 0; i < 8; ++i) {
				if (inverseUr(imp_->puma_param, *imp_->ee->mpm(), i, diff_q[solution_num])) {
					diff_norm[solution_num] = 0;
					for (int j = 0; j < 6; ++j) {
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

			for (aris::Size i = 0; i < 6; ++i) {
				if (&imp_->joints[i]->makI()->fatherPart() == imp_->parts[i + 1]) {
					double pm_prt_i[16], pm_mak_i[16], pm_rot[16];
					s_pe2pm(std::array<double, 6>{0, 0, 0, 0, 0, imp_->motions[i]->mpInternal() + diff_q[real_solution][i]}.data(), pm_rot);
					s_pm_dot_pm(*imp_->joints[i]->makJ()->pm(), pm_rot, pm_mak_i);
					s_pm_dot_inv_pm(pm_mak_i, *imp_->joints[i]->makI()->prtPm(), pm_prt_i);
					imp_->parts[i + 1]->setPm(pm_prt_i);
				}
				else {
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
		else {
			if (double q[6]; inverseUr(imp_->puma_param, *imp_->ee->mpm(), imp_->which_root_, q)) {
				for (aris::Size i = 0; i < 6; ++i) {
					if (&imp_->joints[i]->makI()->fatherPart() == imp_->parts[i + 1]) {
						double pm_prt_i[16], pm_mak_i[16], pm_rot[16];
						s_pe2pm(std::array<double, 6>{0, 0, 0, 0, 0, q[i]}.data(), pm_rot);
						s_pm_dot_pm(*imp_->joints[i]->makJ()->pm(), pm_rot, pm_mak_i);
						s_pm_dot_inv_pm(pm_mak_i, *imp_->joints[i]->makI()->prtPm(), pm_prt_i);
						imp_->parts[i + 1]->setPm(pm_prt_i);
					}
					else {
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
	};
	UrInverseKinematicSolver::~UrInverseKinematicSolver() = default;
	UrInverseKinematicSolver::UrInverseKinematicSolver() :InverseKinematicSolver(1, 0.0), imp_(new Imp) {}
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

		const double j1_pos[3]{ 0.0,      0.0, param.H1                                  };
		const double j2_pos[3]{ 0.0,      0.0, param.H1                                  };
		const double j3_pos[3]{ 0.0,      0.0, param.H1 + param.L1                       };
		const double j4_pos[3]{ 0.0,      0.0, param.H1 + param.L1 + param.L2            };
		const double j5_pos[3]{ 0.0, param.W1, param.H1                                  };
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
			.prop("which_root", &UrInverseKinematicSolver::setWhichRoot, &UrInverseKinematicSolver::whichRoot)
			;
	}
}
