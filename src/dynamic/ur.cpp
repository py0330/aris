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

	auto isUrMechanism(SubSystem &sys)->bool
	{
		std::vector<const Part*> part_vec(7, nullptr);
		std::vector<const RevoluteJoint*> joint_vec(6, nullptr);

		// 必然有地，且与地相连的为杆件1, 那么找出所有杆件 //
		//part_vec[0] = sys.diag_pool_.at(0).part;
		//part_vec[6] = sys.diag_pool_.at(1).part;
		for (auto i = 2; i < 7; ++i)
		{
			auto diag = &sys.diag_pool_.at(i);

			// 向前迭代，看看几个循环能迭代到地面或者末端，那么该diag里的关节就是第几个
			for (int count = 1; true; ++count)
			{
				// 连到了地面上 //
				if (diag->rd == &sys.diag_pool_.at(0))
				{
					part_vec[count] = sys.diag_pool_.at(i).part;
					break;
				}
				// 连到了末端杆件 //
				if (diag->rd == &sys.diag_pool_.at(1))
				{
					part_vec[6 - count] = sys.diag_pool_.at(i).part;
					break;
				}

				diag = diag->rd;
			}
		}

		for (auto p : part_vec)
		{
			std::cout << p->name() << std::endl;
		}


		// 找出所有关节 //




		return true;
	}
	auto UrInverseKinematic(Model &m, SubSystem &sys, int which_root)->int
	{
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
		const double A_pm[16]
		{
			1,0,0,0,
			0,1,0,0,
			0,0,1,0.089159,
			0,0,0,1
		};
		const double E_pm_in_D[16]
		{
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
		if (which_root & 0x04)
		{
			q[0] = PI + std::atan2(D_in_A[7], D_in_A[3]) + std::asin(d3 / std::sqrt(D_in_A[3] * D_in_A[3] + D_in_A[7] * D_in_A[7]));
		}
		else
		{
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

	auto Ur5InverseKinematicSolver::setWhichRoot(int root_of_0_to_7)->void
	{
		if (root_of_0_to_7 < 0 || root_of_0_to_7 > 7) THROW_FILE_LINE("root must be 0 to 7");
		which_root_ = root_of_0_to_7;
	}
	auto Ur5InverseKinematicSolver::whichRoot()->int { return which_root_; }
	auto Ur5InverseKinematicSolver::kinPos()->int { return UrInverseKinematic(*model(), UniversalSolver::imp_->subsys_pool_.at(0), which_root_); };



	ARIS_REGISTRATION
	{
		aris::core::class_<Ur5InverseKinematicSolver>("Ur5InverseKinematicSolver")
			.inherit<InverseKinematicSolver>()
			.prop("which_root", &Ur5InverseKinematicSolver::setWhichRoot, &Ur5InverseKinematicSolver::whichRoot)
			;
	}
}
