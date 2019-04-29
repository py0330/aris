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

namespace aris::dynamic
{
	struct Solver::Imp
	{
		Size max_iter_count_, iter_count_;
		double max_error_, error_;

		Imp(Size max_iter_count, double max_error) :max_iter_count_(max_iter_count), max_error_(max_error) {};
	};
	auto Solver::saveXml(aris::core::XmlElement &xml_ele) const->void
	{
		Element::saveXml(xml_ele);
		xml_ele.SetAttribute("max_iter_count", static_cast<std::int64_t>(imp_->max_iter_count_));
		xml_ele.SetAttribute("max_error", imp_->max_error_);
	}
	auto Solver::loadXml(const aris::core::XmlElement &xml_ele)->void
	{
		imp_->max_iter_count_ = attributeInt32(xml_ele, "max_iter_count", 100);
		imp_->max_error_ = attributeDouble(xml_ele, "max_error", 1e-10);
		Element::loadXml(xml_ele);
	}
	auto Solver::error()const->double { return imp_->error_; }
	auto Solver::setError(double error)->void { imp_->error_ = error; }
	auto Solver::maxError()const->double { return imp_->max_error_; }
	auto Solver::setMaxError(double max_error)->void { imp_->max_error_ = max_error; }
	auto Solver::iterCount()const->Size { return imp_->iter_count_; }
	auto Solver::setIterCount(Size iter_count)->void { imp_->iter_count_ = iter_count; }
	auto Solver::maxIterCount()const->Size { return imp_->max_iter_count_; }
	auto Solver::setMaxIterCount(Size max_count)->void { imp_->max_iter_count_ = max_count; }
	Solver::~Solver() = default;
	Solver::Solver(const std::string &name, Size max_iter_count, double max_error) : Element(name), imp_(new Imp(max_iter_count, max_error)) {}
	ARIS_DEFINE_BIG_FOUR_CPP(Solver);

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
	auto SubSystem::updMakPm()noexcept->void
	{
		for (auto d = diag_pool_.begin() + 1; d < diag_pool_.end(); ++d)
		{
			for (auto &c : d->rel_.cst_pool_)
			{
				s_pm_dot_pm(c.is_I ? d->pm : d->rd->pm, *c.constraint->makI().prtPm(), c.pmI);
				s_pm_dot_pm(c.is_I ? d->rd->pm : d->pm, *c.constraint->makJ().prtPm(), c.pmJ);
			}
		}

		for (auto &r : remainder_pool_)
		{
			for (auto &c : r.rel_.cst_pool_)
			{
				s_pm_dot_pm(c.is_I ? r.i_diag->pm : r.j_diag->pm, *c.constraint->makI().prtPm(), c.pmI);
				s_pm_dot_pm(c.is_I ? r.j_diag->pm : r.i_diag->pm, *c.constraint->makJ().prtPm(), c.pmJ);
			}
		}
	}
	auto SubSystem::updDmCm()noexcept->void
	{
		// upd dm and rel dim
		fm = 0;
		for (auto d = diag_pool_.begin() + 1; d < diag_pool_.end(); ++d)
		{
			d->upd_d(&*d);
			d->rows = fm;
			fm += 6 - d->rel_.dim_;
		}

		// upd remainder data //
		for (auto &r : remainder_pool_)
		{
			Size pos{ 0 };
			for (auto &c : r.rel_.cst_pool_)
			{
				double cmI[36], cmJ[36];
				c.constraint->cptGlbCmFromPm(cmI, cmJ, c.pmI, c.pmJ);
				s_mc(6, c.constraint->dim(), cmI, c.constraint->dim(), r.cmI + pos, r.rel_.size);
				s_mc(6, c.constraint->dim(), cmJ, c.constraint->dim(), r.cmJ + pos, r.rel_.size);
				pos += c.constraint->dim();
			}
		}
	}
	auto SubSystem::updDiagIv()noexcept->void { for (auto &d : diag_pool_)s_iv2iv(*d.part->pm(), d.part->prtIv(), d.iv); }
	auto SubSystem::updCpToBc()noexcept->void
	{
		// bc in diag //
		for (auto d = diag_pool_.begin() + 1; d < diag_pool_.end(); ++d)d->cpt_cp_from_pm(&*d);

		// bc in remainder //
		for (auto &r : remainder_pool_)
		{
			Size pos{ 0 };
			for (auto &c : r.rel_.cst_pool_)
			{
				c.constraint->cptCpFromPm(r.bc + pos, c.pmI, c.pmJ);
				pos += c.constraint->dim();
			}
		}
	}
	auto SubSystem::updCvToBc()noexcept->void
	{
		// bc in diag //
		for (auto d = diag_pool_.begin() + 1; d < diag_pool_.end(); ++d)
		{
			Size pos{ 0 };
			for (auto &c : d->rel_.cst_pool_)
			{
				c.constraint->cptCv(d->bc + pos);
				pos += c.constraint->dim();
			}
		}
		// bc in remainder //
		for (auto &r : remainder_pool_)
		{
			Size pos{ 0 };
			for (auto &c : r.rel_.cst_pool_)
			{
				c.constraint->cptCv(r.bc + pos);
				pos += c.constraint->dim();
			}
		}
	}
	auto SubSystem::updCaToBc()noexcept->void
	{
		// bc in diag //
		for (auto d = diag_pool_.begin() + 1; d < diag_pool_.end(); ++d)
		{
			Size pos{ 0 };
			for (auto &c : d->rel_.cst_pool_)
			{
				c.constraint->cptCa(d->bc + pos);
				pos += c.constraint->dim();
			}
		}
		// bc in remainder //
		for (auto &r : remainder_pool_)
		{
			Size pos{ 0 };
			for (auto &c : r.rel_.cst_pool_)
			{
				c.constraint->cptCa(r.bc + pos);
				pos += c.constraint->dim();
			}
		}
	}
	auto SubSystem::updError()noexcept->void
	{
		// 求解当前误差
		error_ = 0.0;
		for (auto d = diag_pool_.begin() + 1; d < diag_pool_.end(); ++d)for (Size i{ 0 }; i < d->rel_.size; ++i)error_ = std::max(error_, std::abs(d->bc[i]));
		for (auto &r : remainder_pool_)for (Size i{ 0 }; i < r.rel_.size; ++i)error_ = std::max(error_, std::abs(r.bc[i]));
	}
	auto SubSystem::updF()noexcept->void
	{
		// check F size //
		s_fill(fm, fn, 0.0, F);

		Size cols{ 0 };
		for (auto &r : remainder_pool_)
		{
			for (auto &b : r.cm_blk_series)
			{
				s_mm(6 - b.diag->rel_.dim_, r.rel_.size, 6, b.diag->dm + at(b.diag->rel_.dim_, 0, 6), 6, b.is_I ? r.cmI : r.cmJ, r.rel_.size, F + at(b.diag->rows, cols, fn), fn);
			}
			cols += r.rel_.size;
		}

		s_householder_utp(fm, fn, F, FU, FT, FP, fr, max_error_);
	}
	auto SubSystem::sovXp()noexcept->void
	{
		// 请参考step 4，这里先把xp做个预更新,以及初始化 //
		std::fill_n(diag_pool_[0].xp, 6, 0.0);
		for (auto d = diag_pool_.begin() + 1; d < diag_pool_.end(); ++d)
		{
			// 如果是多个杆件，那么需要重新排序 //
			s_permutate(d->rel_.size, 1, d->p, d->bc);

			// 预更新 //
			s_mm(6, 1, d->rel_.dim_, d->dm, T(6), d->bc, 1, d->xp, 1);
		}

		// 构造bcf //
		Size cols{ 0 };
		for (auto &r : remainder_pool_)
		{
			s_vc(r.rel_.size, r.bc, bcf + cols);
			for (auto &b : r.cm_blk_series)
			{
				auto cm = b.is_I ? r.cmJ : r.cmI;//这里是颠倒的，因为加到右侧需要乘-1.0
				s_mma(r.rel_.size, 1, 6, cm, ColMajor{ r.rel_.size }, b.diag->xp, 1, bcf + cols, 1);//请参考step 4，将预更新的东西取出
			}
			cols += r.rel_.size;
		}

		// 求解 F' * xpf = bcf //
		s_vc(fn, bcf, xpf);
		s_permutate(fn, 1, FP, xpf);
		s_sov_lm(fr, 1, FU, ColMajor(fn), xpf, 1, xpf, 1, max_error_);
		s_householder_ut_q_dot(fm, fn, 1, FU, FT, xpf, xpf);

		// 更新xp //  相当于 D' * yp
		for (auto d = diag_pool_.begin() + 1; d<diag_pool_.end(); ++d)
		{
			s_mma(6, 1, 6 - d->rel_.dim_, d->dm + at(0, d->rel_.dim_, T(6)), T(6), xpf + d->rows, 1, d->xp, 1);
		}

		// 做行变换 //  相当于 P' * (D' * yp)
		for (auto d = diag_pool_.begin() + 1; d<diag_pool_.end(); ++d)s_va(6, d->rd->xp, d->xp);
	}
	auto SubSystem::updG()noexcept->void
	{
		gm = hasGround() ? fm : fm + 6;
		gn = hasGround() ? fm - fr : fm - fr + 6;

		//////////////////////////////////////////// 求CT * xp = bc 的通解S step 5 //////////////////////////////
		std::fill_n(xpf, fm, 0.0);
		for (Size j(-1); ++j < fm - fr;)
		{
			xpf[fr + j] = 1.0;
			s_householder_ut_q_dot(fm, fn, 1, FU, fn, FT, 1, xpf, 1, S + j, fm - fr);
			xpf[fr + j] = 0.0;
		}

		//////////////////////////////////////////// 求G，参考 step 6 //////////////////////////////
		s_fill(gm, gn, 0.0, G);
		// 先求S产生的G
		for (Size j(-1); ++j < fm - fr;)
		{
			// 初始化xp并乘以DT
			std::fill(diag_pool_[0].xp, diag_pool_[0].xp + 6, 0.0);
			for (auto d = diag_pool_.begin() + 1; d<diag_pool_.end(); ++d)
			{
				if (d->rel_.dim_ == 6)std::fill(d->xp, d->xp + 6, 0.0);
				else s_mm(6, 1, 6 - d->rel_.dim_, d->dm + at(0, d->rel_.dim_, ColMajor{ 6 }), ColMajor{ 6 }, S + at(d->rows, j, fm - fr), fm - fr, d->xp, 1);
			}

			// 乘以PT，行加
			for (auto d = diag_pool_.begin() + 1; d < diag_pool_.end(); ++d)s_va(6, d->rd->xp, d->xp);

			// 乘以I
			for (auto d = diag_pool_.begin() + 1; d < diag_pool_.end(); ++d)
			{
				double tem[6];
				s_iv_dot_as(d->iv, d->xp, tem);
				s_vc(6, tem, d->xp);
			}

			// 乘以P，行加
			for (auto d = diag_pool_.rbegin(); d < diag_pool_.rend() - 1; ++d) s_va(6, d->xp, d->rd->xp);

			// 乘以D，并取出来
			for (auto d = diag_pool_.rbegin(); d<diag_pool_.rend() - 1; ++d)
			{
				s_mm(6 - d->rel_.dim_, 1, 6, d->dm + at(d->rel_.dim_, 0, 6), 6, d->xp, 1, G + at(d->rows, j, gn), gn);
			}

			// 如果无地，需要考虑G中第一个杆件处的xp
			if (!hasGround())s_vc(6, diag_pool_.begin()->xp, 1, G + at(fm, j, fm - fr + 6), fm - fr + 6);
		}
		// 再求无地处第一个杆件处产生的G
		if (!hasGround())
		{
			for (Size j(-1); ++j < 6;)
			{
				// 初始化，此时无需乘以DT,因为第一个杆件的DT为单位阵，其他地方的xp为0
				for (auto &d : diag_pool_)std::fill(d.xp, d.xp + 6, 0.0);
				diag_pool_[0].xp[j] = 1.0;

				// 乘以PT
				for (auto d = diag_pool_.begin() + 1; d<diag_pool_.end(); ++d)s_va(6, d->rd->xp, d->xp);

				// 乘以I, 因为bp里面储存了外力，因此不能用bp
				for (auto &d : diag_pool_)
				{
					double tem[6];
					s_iv_dot_as(d.iv, d.xp, tem);
					s_vc(6, tem, d.xp);
				}

				// 乘以P, 类似rowAddBp();
				for (auto d = diag_pool_.rbegin(); d < diag_pool_.rend() - 1; ++d) s_va(6, d->xp, d->rd->xp);

				// 乘以D，并取出来
				for (auto d = diag_pool_.rbegin(); d<diag_pool_.rend() - 1; ++d)
				{
					s_mm(6 - d->rel_.dim_, 1, 6, d->dm + at(d->rel_.dim_, 0, 6), 6, d->xp, 1, G + at(d->rows, fm - fr + j, gn), gn);
				}
				s_vc(6, diag_pool_[0].xp, 1, G + at(fm, fm - fr + j, gn), gn);
			}
		}

		//// 求QT_DOT_G ////
		if (!hasGround())s_mc(gm - fm, gn, G + at(fm, 0, gn), QT_DOT_G + at(fm, 0, gn)); // 这一项实际是把无地面产生的G拷贝到QT_DOT_G中
		s_householder_ut_qt_dot(fm, fn, gn, FU, FT, G, QT_DOT_G);
	}
	auto SubSystem::sovXc()noexcept->void
	{
		//// 更新每个杆件的力 pf ////
		for (auto &d : diag_pool_)
		{
			// 外力已经储存在了bp中,因此这里不需要增加外力 //

			// I*(a-g) //
			double as_minus_g[6], iv_dot_as[6];
			s_vc(6, d.xp, as_minus_g);// xp储存加速度
			s_vs(6, d.part->ancestor<Model>()->environment().gravity(), as_minus_g);
			s_iv_dot_as(d.iv, as_minus_g, iv_dot_as);
			s_va(6, iv_dot_as, d.bp);

			// v x I * v //
			double I_dot_v[6];
			s_iv_dot_as(d.iv, d.part->vs(), I_dot_v);
			s_cfa(d.part->vs(), I_dot_v, d.bp);
		}

		//// P*bp  对bp做行加变换 ////
		for (auto d = diag_pool_.rbegin(); d < diag_pool_.rend() - 1; ++d) s_va(6, d->bp, d->rd->bp);

		//// 取出bpf ////
		for (auto d = diag_pool_.begin() + 1; d < diag_pool_.end(); ++d)
		{
			double tem[6];

			s_mm(6, 1, 6, d->dm, 6, d->bp, 1, tem, 1);

			s_vc(6, tem, d->bp);
			s_vc(6 - d->rel_.dim_, d->bp + d->rel_.dim_, bpf + d->rows);
		}

		//// 根据G求解S解空间的beta, 先把beta都弄成它的右侧未知量 ////
		if (!hasGround())s_vc(6, diag_pool_[0].bp, beta + fm);//这一项实际是把无地面的xp拷贝到beta中
		s_householder_ut_qt_dot(fm, fn, 1, FU, FT, bpf, beta);

		///////////////////////////////
		// 求解之前通解的解的系数
		// 可以通过rank == m-r来判断质点等是否影响计算
		///////////////////////////////
		Size rank;
		s_householder_utp(gn, gn, QT_DOT_G + at(fr, 0, gn), GU, GT, GP, rank, max_error_);
		s_householder_utp_sov(gn, gn, 1, rank, GU, GT, GP, beta + fr, beta);

		// 这里就求出了beta
		// 接着求真正的bpf, 并求解xc
		s_mma(fm, 1, gn, G, beta, bpf);
		s_householder_utp_sov(fm, fn, 1, fr, FU, FT, FP, bpf, xcf, max_error_);

		///////////////////////////////////根据xcf求解xc/////////////////////////////////////////////
		// 将已经求出的x更新到remainder中，此后将已知数移到右侧
		Size cols{ 0 };
		for (auto &r : remainder_pool_)
		{
			s_vc(r.rel_.size, xcf + cols, r.xc);

			// 更新待求
			for (auto &b : r.cm_blk_series)
			{
				double tem[6];
				s_mm(6, 1, r.rel_.size, b.is_I ? r.cmJ : r.cmI, xcf + cols, tem);
				s_mma(6, 1, 6, b.diag->dm, 6, tem, 1, b.diag->bp, 1);
			}

			cols += r.rel_.size;
		}
		for (auto d = diag_pool_.begin() + 1; d < diag_pool_.end(); ++d)
		{
			s_vc(d->rel_.dim_, d->bp, d->xc);
			std::fill(d->xc + d->rel_.dim_, d->xc + d->rel_.size, 0.0);
			s_permutate_inv(d->rel_.size, 1, d->p, d->xc);
		}

		//// 重新求解 xp ，这次考虑惯量 ////
		// 根据特解更新 xpf 以及无地面处的特解（杆件1速度之前可以随便设）
		s_mms(fm, 1, fm - fr, S, beta, xpf);
		if (!hasGround())s_vi(6, beta + fm - fr, diag_pool_[0].xp);
		// 将xpf更新到xp，先乘以D' 再乘以 P'
		for (auto d = diag_pool_.begin() + 1; d<diag_pool_.end(); ++d)
		{
			// 结合bc 并乘以D'
			s_mm(6, 1, d->rel_.dim_, d->dm, ColMajor{ 6 }, d->bc, 1, d->xp, 1);
			s_mma(6, 1, 6 - d->rel_.dim_, d->dm + at(0, d->rel_.dim_, T(6)), T(6), xpf + d->rows, 1, d->xp, 1);

			// 乘以P'
			s_va(6, d->rd->xp, d->xp);
		}
	}
	auto SubSystem::kinPos()noexcept->void
	{
		updMakPm();
		updCpToBc();
		updError();
		for (iter_count_ = 0; iter_count_ < max_iter_count_; ++iter_count_)
		{
			if (error_ < max_error_) return;

			// make A
			updDmCm();

			// solve
			updF();
			sovXp();

			// 将xp更新成pm
			for (auto &d : diag_pool_)
			{
				std::swap(d.pm, d.last_pm);
				double tem[16];
				s_ps2pm(d.xp, tem);
				s_pm2pm(tem, d.last_pm, d.pm);
			}

			double last_error = error_;
			updMakPm();
			updCpToBc();
			updError();

			// 对于非串联臂，当迭代误差反而再增大时，会主动缩小步长
			if (!remainder_pool_.empty())// 只有不是串联臂才会用以下迭代
			{
				if (error_ > last_error)// 这里如果用while可以确保每个循环误差都会减小，但是有可能出现卡死
				{
					double coe = last_error / (error_ + last_error);

					for (auto &d : diag_pool_)
					{
						s_nv(6, coe, d.xp);
						double tem[16];
						s_ps2pm(d.xp, tem);
						s_pm2pm(tem, d.last_pm, d.pm);
					}

					updMakPm();
					updCpToBc();
					updError();
				}
			}
		}
	}
	auto SubSystem::kinVel()noexcept->void
	{
		updMakPm();
		
		// make b
		updCvToBc();

		// make A
		updDmCm();

		// solve
		updF();
		sovXp();
	}
	auto SubSystem::dynAccAndFce()noexcept->void
	{
		updMakPm();
		
		// upd Iv dm cm and ca  //
		updDiagIv();
		updDmCm();

		// upd F and G //
		updF();
		updG();

		//// 求解 xp 的某个特解（不考虑惯量），求出beta 以及 xc
		updCaToBc();
		sovXp();
		sovXc();
	}
	struct UniversalSolver::Imp
	{
		// 动力学计算以下变量的关系
		// I  ： 惯量矩阵,m x m
		// C  ： 约束矩阵,m x n
		// pa ： 杆件的螺旋加速度 m x 1
		// pf ： 杆件的螺旋外力（不包括惯性力）m x 1
		// ca ： 约束的加速度（不是螺旋）n x 1
		// cf ： 约束力n x 1
		// 动力学主要求解以下方程：
		// [ -I  C  ]  *  [ pa ]  = [ pf ]
		// [  C' O  ]     [ cf ]    [ ca ]
		//
		// A = [-I  C ]
		//     [ C' O ]
		//
		// x = [ pa ] = [ xp ]
		//     [ cf ]   [ xc ]
		//
		// b = [ pf ] = [ bp ]
		//     [ ca ]   [ bc ]
		// 
		// 
		// 计算方法：
		// --------------------------------------------------------------------
		// step 1:分出子系统，调整杆件和约束顺序，使得约束矩阵变成上三角块矩阵
		//        经过这一步，约束矩阵变为：
		//
		//        有地面(n个约束，m个杆件)：
		//        [ Cg  -C1  ...              ... -Cn ]
		//        |      C1  ...  -Cm-1       ...     |
		//        |          ...         -Cm  ...     |
		//        [                Cm-1   Cm  ...  Cn ]                
		//    
		//        无地面(n个约束，m个杆件)
		//        [ -C1  ...             -Cn ]
		//        |  C1  ...  -Cm-1          |
		//        |      ...         -Cm     |
		//        [            Cm-1   Cm  Cn ]  
		// --------------------------------------------------------------------
		// step 2:寻找矩阵P，使得约束矩阵对角化
		//        
		//        有地面：
		//        P * C = [ Cg                 -Cm ... -Cn ]
		//                |    C1              -Cm ...     |
		//                |       C2               ...  Cn | 
		//                |          ...        Cm ...  Cn |
		//                [              Cm-1      ...  Cn ]
		//        无地面，此时第一行为空：
		//        P * C = [                                ]
		//                |    C1              -Cm ...     |
		//                |       C2               ...  Cn | 
		//                |          ...        Cm ...  Cn |
		//                [              Cm-1      ...  Cn ]
		// --------------------------------------------------------------------
		// step 3:两边乘以矩阵D，并且可以取得子块F
		//        矩阵D的定义为：  D1_6x6 * C1_6xn = I_6xn   其中n为约束的维数
		// 
		//        有地面：
		//                                                              c1            cn
		//        D * P * C = [ I                                      -Cm  ...      -Cn ]
		//                    |    [ I1 ]                           -D1*Cm  ...          |
		//                    |    [  0 ]                                                |  r2
		//                    |            [ I2 ]                           ...    D2*Cn |  
		//                    |            [  0 ]                                        |  r3
		//                    |                    ...               Di*Cm  ...    Di*Cn |
		//                    |                         [ Im-1 ]                         |
		//                    [                         [   0  ]            ...  Dm-1*Cn ]  rm
		//
		//        无地面，此时第一行为空：
		//                                                           cm            cn
		//        D * P * C = [                                                       ]
		//                    | [ I1 ]                           -D1*Cm  ...          |
		//                    | [  0 ]                                                |  r2
		//                    |         [ I2 ]                           ...    D2*Cn |
		//                    |         [  0 ]                                        |  r3
		//                    |                 ...               Di*Cm  ...    Di*Cn |
		//                    |                      [ Im-1 ]                         |
		//                    [                      [   0  ]            ...  Dm-1*Cn ]  rm
		//
		//        抽出 cm ... cn 列，r2 ... rm 行，组成F
		//        F = D * P * C  (r2 ... rm , cm ... cn)
		// --------------------------------------------------------------------
		// step 4:求解方程C' * xp = bc
		//        该方程可化作：
		//           C' * P' * D' * D'^-1 * P'^-1 * xp = bc
		//           DPC' * D'^-1 * P'^-1 * xp = bc
		//           DPC' * yp = bc
		//        其中：yp = D'^-1 * P'^-1 * xp
		//        
		//        DPC' 和 yp 为
		//        有地面：
		//                            r2        r3                  rm 
		//        DPC' = [  I                                            ]
		//               |       [ I1 0 ]                                |
		//               |                 [ I2 0 ]                      |  
		//               |                            ...                |
		//               |                                   [ Im-1 0 ]  |  
		//               | -Cm'  -Cm'*D1'           Cm'*Di'   Cm'*Dm-1'  |  c1
		//               | ...      ...       ...   Cj'*Di'     ...      | 
		//               [ -Cn'             Cn'*D2' Cn'*Di'   Cn'*Dm-1'  ]  cn
		// 
		//        无地面：
		//                            r2        r3                  rm 
		//        DPC' = [  0                                            ]
		//               |  0    [ I1 0 ]                                |
		//               |  0              [ I2 0 ]                      |  
		//               |  0                         ...                |
		//               |  0                                [ Im-1 0 ]  |  
		//               |  0    -Cm'*D1'           Cm'*Di'   Cm'*Dm-1'  |  c1
		//               |  0       ...       ...   Cj'*Di'     ...      | 
		//               [  0               Cn'*D2' Cn'*Di'   Cn'*Dm-1'  ]  cn
		// 
		//        yp都一样：
		//        yp   = [    yp1   ]
		//               | -------- |
		//               | [ ypa2 ] |
		//               | [ ypf2 ] |
		//               | -------- |
		//               |    ...   |
		//               |    ...   |
		//               | -------- |
		//               | [ ypam ] |
		//               [ [ ypfm ] ]
		//        而bc为：
		//        bc   = [  bc1  ]
		//               |  bc2  |
		//               |  ...  |
		//               [  bcn  ]
		//
		//        从而可以求得yp中的一部分：
		//        [ ypa2 ]  =  [  bc1  ]
		//        | ypa3 |     |  bc2  |
		//        |  ... |     |  ...  |
		//        [ ypam ]     [ bcm-1 ]
		//        yp中的另外一部分需要用上文中的 F 来求，下文中的k是对应约束的维数：
		//        
		//        F * [ ypf2 ]  =  [  bcm  ]   -   [ -Cm'*D1(1:k,1:6)'*bc1   + ... + Cm'*Dm-1(1:k,1:6)'*bcm-1   ]
		//            | ypf3 |     | bcm+1 |       |  Cm+1'*D1(1:k,1:6)'*bc1 + ... + Cm+1'*Dm-1(1:k,1:6)'*bcm-1 |
		//            |  ... |     |  ...  |       |                           ...                              |
		//            [ ypfm ]     [  bcn  ]       [ -Cn'*D1(1:k,1:6)'*bc1   + ... + Cn'*Dm-1(1:k,1:6)'*bcm-1   ]
		// 
		//        yp的最后一部分是yp1，它如下：
		//        有地面：  yp1 = [0,0,0,0,0,0]'
		//        无地面：  yp1 可以取任何值，本节中无法求得它的数值
		//        求得yp后，可以求得xp
		//        xp = P' * D' * yp = P' * diag([yp1,  D1(1:k,1:6)'*bc1 + D1(k+1:6,1:6)'*ypf2], ... ,  Dm-1(1:k,1:6)'*bcm-1 + Dm-1(k+1:6,1:6)'*ypfm])
		//        
		//        在实际的计算中，可以先将xp设为   D1(1:k,1:6)'*bc1，从而减少计算
		// --------------------------------------------------------------------
		// step 5:求解方程F' * ypf = bcf 的特解 xpf 和通解 S
		//
		//        F * P = Q * R
		//        这里R为：[R1 R2 | 0 0]
		//            1   r   n
		//        1 [ * * * * * ]
		//          |   * * * * |
		//        r |     * * * |
		//          |           |
		//          |           |
		//        m [           ]
		//
		//        这里求解：
		//        F' * ypf = bcf
		//        即：
		//        P^-T * P' * F' * xpf = bcf
		//        P^-T * R' * Q' * xpf = bcf
		//        这里的R'为：
		//            1   r   m
		//        1 [ *         ]
		//          | * *       |
		//        r | * * *     |
		//          | * * *     |
		//          | * * *     |
		//        n [ * * *     ]
		//        于是Q' * x 的通解为：
		//            1  m-r 
		//        1 [       ]
		//          |       |
		//        r |       |
		//          | 1     |
		//          |   1   |
		//        m [     1 ]
		//        于是x的通解S为以上左乘Q
		//        S = Q * [    0_rxr      ]
		//                [ I_(m-r)x(m-r) ]
		//
		//        P^-T * R' * Q' * xpf = bcf
		//        其特解为:
		//        Q * [ R1^-1   ]  *  P' * bcf
		//            [       1 ]
		// --------------------------------------------------------------------
		// step 6:S是通解，现在需要求出G，从而利用惯性矩阵确定真实的约束力
		//        根据S可以求得yp的通解：
		//        有地面：
		//        K1 = [ [   0   ] ]
		//             | ......... |
		//             | [   0   ] |
		//             | [  Sf1  ] |
		//             | ......... |  
		//             | [   0   ] |
		//             | [  Sf2  ] |
		//             |    ...    |
		//             | [   0   ] |
		//             [ [ Sfm-1 ] ]
		//        无地面：
		//        K1 = [ I .         ]
		//             | ........... |
		//             |   . [  0  ] |
		//             |   . [ Sf1 ] |
		//             | ........... |
		//             |   . [  0  ] |
		//             |   . [ Sf2 ] |
		//             |   .   ...   |
		//             |   . [  0  ] |
		//             [   . [ Sfn ] ]
		//        
		//        进一步可以求得xp的通解K：
		//        K = P' * D' * K1        
		//        
		//        求得xp的通解K，那么最终方程为：
		//        [ I C ] * [ xpt + K*beta ] = [ bp ]
		//                  [     xc       ]
		//        可以化为：
		//        [ C I*K ] * [  xc  ] = bp - I * xpt
		//                    [ beta ]
		//        
		//        这个两边乘以 P*D ，可得：
		//        [ PDC PDIK ] * [  xcf ] = PD(bp - I * xpt)
		//                       [ beta ]
		//        
		//        PDC和PDIK在 cm ... cn : end 列，r2 ... rm 行，组成[F G]
		//        有地面时，F和G的行数相同，G的列数为fm - fr:
		//        [ F  G ] * [  xcf ] = bpf
		//                   [ beta ]
		//        无地面时，G的行数为fm + 6, 列数为fm-fr+6：
		//        [ F  G1 ] * [  xcf ] = [ bpf ]
		//        [    G2 ]   [ beta ]   [ bp1 ]
		// --------------------------------------------------------------------
		// step 7:求出G后，可以进行下一步，求取H
		//        既然已经求出过 F 的 QR 分解，那么可以先用F的Q乘以两侧: 注意，这里的G2 并非上文中无地面的G2
		//        有地面时：
		//                     fn    fm-fr
		//        fr       [ R*P^-1  [Q'*G](   1:fr,:)  ] * [  xcf ] = [Q'*bpf](   1:fr)
		//        fm-fr    [         [Q'*G](fr+1:fm,:)  ]   [ beta ]   [Q'*bpf](fr+1:fm)
		//        无地面时：
		//                     fn    fm-fr+6
		//        fr       [ R*P^-1  [Q'*G1](   1:fr,:)  ] * [  xcf  ] = [ [Q'*bpf](   1:fr) ]
		//        fm-fr    [         [Q'*G1](fr+1:fm,:)  ]   [ beta  ]   | [Q'*bpf](fr+1:fm) |
		//        6        [                G2           ]               [       bp1         ]
		//
		//        求解右下角，即可得beta
		//        最终可得xcf
		//

		std::vector<Diag *> get_diag_from_part_id_;

		std::vector<SubSystem> subsys_pool_;

		std::vector<double> F_, FU_, FT_, G_, GU_, GT_, S_, QT_DOT_G_, xpf_, xcf_, bpf_, bcf_, beta_, cmI, cmJ, cmU, cmT;
		std::vector<Size> FP_, GP_;

		std::vector<double> Jg_, cg_;
		std::vector<double> M_, h_;

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
	auto UniversalSolver::allocateMemory()->void
	{
		// make active part pool //
		std::vector<const Part*> active_part_pool;
		active_part_pool.clear();
		active_part_pool.push_back(&ancestor<Model>()->ground());
		for (auto &p : ancestor<Model>()->partPool())if (p.active() && &p != &ancestor<Model>()->ground())active_part_pool.push_back(&p);

		// make active constraint pool //
		std::vector<const Constraint*> cp;
		for (auto &jnt : ancestor<Model>()->jointPool())if (jnt.active())cp.push_back(&jnt);
		for (auto &mot : ancestor<Model>()->motionPool())if (mot.active()) cp.push_back(&mot);
		for (auto &gmt : ancestor<Model>()->generalMotionPool())if (gmt.active())cp.push_back(&gmt);

		// make relation pool //
		std::vector<Relation> relation_pool;
		for (auto c : cp)
		{
			auto ret = std::find_if(relation_pool.begin(), relation_pool.end(), [&c](Relation &relation)
			{
				auto ri{ relation.prtI }, rj{ relation.prtJ }, ci{ &c->makI().fatherPart() }, cj{ &c->makJ().fatherPart() };
				return ((ri == ci) && (rj == cj)) || ((ri == cj) && (rj == ci));
			});

			if (ret == relation_pool.end())
			{
				relation_pool.push_back(Relation{ &c->makI().fatherPart(), &c->makJ().fatherPart(), c->dim(), c->dim(),{ { c, true } } });
			}
			else
			{
				ret->cst_pool_.push_back({ c, &c->makI().fatherPart() == ret->prtI });
				std::sort(ret->cst_pool_.begin(), ret->cst_pool_.end(), [](const Relation::Block& a, const Relation::Block& b) { return a.constraint->dim() >= b.constraint->dim(); });//这里把大的约束往前放
				ret->size += c->dim();
			}
		}

		// 划分出相关的Part和Relation //
		std::vector<std::tuple<std::vector<const Part*>, std::vector<Relation> > > part_and_relation_vec;
		while (active_part_pool.size() > 1)
		{
			std::function<void(std::vector<const Part *> &part_pool_, std::vector<const Part *> &left_part_pool, std::vector<Relation> &relation_pool, const Part *part)> addPart;
			addPart = [&](std::vector<const Part *> &part_pool_, std::vector<const Part *> &left_part_pool, std::vector<Relation> &relation_pool, const Part *part)->void
			{
				if (std::find(part_pool_.begin(), part_pool_.end(), part) != part_pool_.end())return;

				part_pool_.push_back(part);

				// 如果不是地面，那么抹掉该杆件，同时添加该杆件的相关杆件 //
				if (part != left_part_pool.front())
				{
					left_part_pool.erase(std::find(left_part_pool.begin(), left_part_pool.end(), part));
					for (auto &rel : relation_pool)
					{
						if (rel.prtI == part || rel.prtJ == part)
						{
							addPart(part_pool_, left_part_pool, relation_pool, rel.prtI == part ? rel.prtJ : rel.prtI);
						}
					}
				}
			};

			// insert part_vec and rel_vec //
			part_and_relation_vec.push_back(std::make_tuple(std::vector<const Part*>(), std::vector<Relation>()));
			auto &part_vec = std::get<0>(part_and_relation_vec.back());
			auto &rel_vec = std::get<1>(part_and_relation_vec.back());

			// add related part //
			addPart(part_vec, active_part_pool, relation_pool, active_part_pool.at(1));

			// add related relation //
			for (auto &rel : relation_pool)
			{
				if (std::find_if(part_vec.begin(), part_vec.end(), [&rel, this](const Part *prt) { return prt != &(this->ancestor<Model>()->ground()) && (prt == rel.prtI || prt == rel.prtJ); }) != part_vec.end())
				{
					rel_vec.push_back(rel);
				}
			}

			// 对sys的part和relation排序 //
			for (Size i = 0; i < std::min(part_vec.size(), rel_vec.size()); ++i)
			{
				// 先对part排序，找出下一个跟上一个part联系的part
				std::sort(part_vec.begin() + i, part_vec.end(), [i, this, &rel_vec](const Part* a, const Part* b)
				{
					if (a == &this->ancestor<Model>()->ground()) return true; // 地面最优先
					if (b == &this->ancestor<Model>()->ground()) return false; // 地面最优先
					if (i == 0)return a->id() < b->id();// 第一轮先找地面或其他地面，防止下面的索引i-1出错
					if (b == rel_vec[i - 1].prtI) return false;
					if (b == rel_vec[i - 1].prtJ) return false;
					if (a == rel_vec[i - 1].prtI) return true;
					if (a == rel_vec[i - 1].prtJ) return true;
					return a->id() < b->id();
				});
				// 再插入连接新part的relation
				std::sort(rel_vec.begin() + i, rel_vec.end(), [i, this, &part_vec](Relation a, Relation b)
				{
					auto pend = part_vec.begin() + i + 1;
					auto a_part_i = std::find_if(part_vec.begin(), pend, [a](const Part* p)->bool { return p == a.prtI; });
					auto a_part_j = std::find_if(part_vec.begin(), pend, [a](const Part* p)->bool { return p == a.prtJ; });
					auto b_part_i = std::find_if(part_vec.begin(), pend, [b](const Part* p)->bool { return p == b.prtI; });
					auto b_part_j = std::find_if(part_vec.begin(), pend, [b](const Part* p)->bool { return p == b.prtJ; });

					bool a_is_ok = (a_part_i == pend) != (a_part_j == pend);
					bool b_is_ok = (b_part_i == pend) != (b_part_j == pend);

					if (a_is_ok && !b_is_ok) return true;
					else if (!a_is_ok && b_is_ok) return false;
					else if (a.size != b.size)return a.size > b.size;
					else if (a.dim_ != b.dim_)return a.dim_ > b.dim_;
					else return false;
				});
			}
		}

		// 构建子系统 //
		imp_->subsys_pool_.clear();
		Size max_F_size{ 0 }, max_fm{ 0 }, max_fn{ 0 }, max_G_size{ 0 }, max_gm{ 0 }, max_gn{ 0 }, max_cm_size{ 0 };
		for (auto &part_and_relation : part_and_relation_vec)
		{
			auto part_vec = std::get<0>(part_and_relation);
			auto rel_vec = std::get<1>(part_and_relation);

			// 插入SubSystem //
			imp_->subsys_pool_.push_back(SubSystem());
			auto &sys = imp_->subsys_pool_.back();
			sys.max_error_ = maxError();

			// 判断是否有地面 //
			sys.has_ground_ = (part_vec.front() == &ancestor<Model>()->ground());

			// 制造diag pool //
			sys.diag_pool_.clear();
			sys.diag_pool_.resize(part_vec.size());
			sys.diag_pool_.at(0).part = part_vec.at(0);
			sys.diag_pool_.at(0).pm = sys.diag_pool_.at(0).pm1;
			sys.diag_pool_.at(0).last_pm = sys.diag_pool_.at(0).pm2;
			for (Size i = 1; i < sys.diag_pool_.size(); ++i)
			{
				auto &diag = sys.diag_pool_.at(i);

				diag.rel_ = rel_vec.at(i - 1);
				// 如果relation的prtI不是对角线的杆件，那么进行反转
				if (diag.rel_.prtI != part_vec.at(i))
				{
					std::swap(diag.rel_.prtI, diag.rel_.prtJ);
					for (auto &c : diag.rel_.cst_pool_)c.is_I = !c.is_I;
				}

				diag.part = part_vec.at(i);
				diag.rd = &*std::find_if(sys.diag_pool_.begin(), sys.diag_pool_.end(), [&](Diag &d) {return d.part == diag.rel_.prtJ; });
				diag.pm = diag.pm1;
				diag.last_pm = diag.pm2;
				diag.bc_vec.resize(diag.rel_.size);
				diag.bc = diag.bc_vec.data();
				diag.xc_vec.resize(diag.rel_.size);
				diag.xc = diag.xc_vec.data();
				diag.p_vec.resize(diag.rel_.size);
				diag.p = diag.p_vec.data();
				std::iota(diag.p_vec.begin(), diag.p_vec.end(), 0);

				max_cm_size = std::max(max_cm_size, diag.rel_.size);

				// 以下优化dm矩阵的计算
				// 针对约束仅仅有一个时的优化 //
				if (diag.rel_.cst_pool_.size() == 1)
				{
					diag.upd_d = Imp::one_constraint_upd_d;
					diag.cpt_cp_from_pm = Imp::one_constraint_cpt_cp_from_pm;
				}
				// 针对转动副加转动电机 //
				else if (diag.rel_.cst_pool_.size() == 2
					&& dynamic_cast<const RevoluteJoint*>(diag.rel_.cst_pool_.at(0).constraint)
					&& dynamic_cast<const Motion*>(diag.rel_.cst_pool_.at(1).constraint)
					&& dynamic_cast<const Motion*>(diag.rel_.cst_pool_.at(1).constraint)->axis() == 5
					&& &diag.rel_.cst_pool_.at(0).constraint->makI() == &diag.rel_.cst_pool_.at(1).constraint->makI())
				{
					diag.upd_d = Imp::revolute_upd_d;
					diag.cpt_cp_from_pm = Imp::revolute_cpt_cp_from_pm;
					diag.rel_.dim_ = 6;
				}
				// 针对移动副加移动电机 //
				else if (diag.rel_.cst_pool_.size() == 2
					&& dynamic_cast<const PrismaticJoint*>(diag.rel_.cst_pool_.at(0).constraint)
					&& dynamic_cast<const Motion*>(diag.rel_.cst_pool_.at(1).constraint)
					&& dynamic_cast<const Motion*>(diag.rel_.cst_pool_.at(1).constraint)->axis() == 2
					&& &diag.rel_.cst_pool_.at(0).constraint->makI() == &diag.rel_.cst_pool_.at(1).constraint->makI())
				{
					diag.upd_d = Imp::prismatic_upd_d;
					diag.cpt_cp_from_pm = Imp::prismatic_cpt_cp_from_pm;
					diag.rel_.dim_ = 6;
				}
				// 不优化 //
				else
				{
					diag.upd_d = Imp::normal_upd_d;
					diag.cpt_cp_from_pm = Imp::normal_cpt_cp_from_pm;
				}
			}

			// 制造remainder pool //
			sys.remainder_pool_.clear();
			sys.remainder_pool_.resize(rel_vec.size() - part_vec.size() + 1);
			for (Size i = 0; i < sys.remainder_pool_.size(); ++i)
			{
				auto &r = sys.remainder_pool_.at(i);

				r.rel_ = rel_vec.at(i + sys.diag_pool_.size() - 1);
				r.i_diag = &*std::find_if(sys.diag_pool_.begin(), sys.diag_pool_.end(), [&r](Diag& d) {return r.rel_.prtI == d.part; });
				r.j_diag = &*std::find_if(sys.diag_pool_.begin(), sys.diag_pool_.end(), [&r](Diag& d) {return r.rel_.prtJ == d.part; });
				r.bc_vec.resize(r.rel_.size);
				r.bc = r.bc_vec.data();
				r.xc_vec.resize(r.rel_.size);
				r.xc = r.xc_vec.data();
				r.cmI_vec.resize(6 * r.rel_.size);
				r.cmI = r.cmI_vec.data();
				r.cmJ_vec.resize(6 * r.rel_.size);
				r.cmJ = r.cmJ_vec.data();
				r.cm_blk_series.clear();
				r.cm_blk_series.push_back(Remainder::Block());
				r.cm_blk_series.back().diag = &*std::find_if(sys.diag_pool_.begin(), sys.diag_pool_.end(), [&r](Diag& d) {return r.rel_.prtI == d.part; });
				r.cm_blk_series.back().is_I = true;
				r.cm_blk_series.push_back(Remainder::Block());
				r.cm_blk_series.back().diag = &*std::find_if(sys.diag_pool_.begin(), sys.diag_pool_.end(), [&r](Diag& d) {return r.rel_.prtJ == d.part; });
				r.cm_blk_series.back().is_I = false;

				for (auto rd = sys.diag_pool_.rbegin(); rd < sys.diag_pool_.rend() - 1; ++rd)
				{
					auto &d = *rd;

					auto diag_part = d.rel_.prtI;
					auto add_part = d.rel_.prtJ;

					// 判断当前remainder加法元素是否存在（不为0）
					auto diag_blk = std::find_if(r.cm_blk_series.begin(), r.cm_blk_series.end(), [&](Remainder::Block &blk) {return blk.diag->part == diag_part; });
					auto add_blk = std::find_if(r.cm_blk_series.begin(), r.cm_blk_series.end(), [&](Remainder::Block &blk) {return blk.diag->part == add_part; });
					if (diag_blk != r.cm_blk_series.end())
					{
						if (add_blk != r.cm_blk_series.end())
						{
							r.cm_blk_series.erase(add_blk);
						}
						else
						{
							Remainder::Block blk;
							blk.is_I = diag_blk->is_I;

							blk.diag = &*std::find_if(sys.diag_pool_.begin(), sys.diag_pool_.end(), [&](Diag &d) {return d.part == add_part; });

							r.cm_blk_series.push_back(blk);
						}
					}
				}
			}

			// 更新子系统尺寸 //
			sys.fm = 0;
			sys.fn = 0;
			for (auto d = sys.diag_pool_.begin() + 1; d < sys.diag_pool_.end(); ++d) sys.fm += 6 - d->rel_.dim_;
			for (auto &r : sys.remainder_pool_) sys.fn += r.rel_.dim_;

			sys.gm = sys.hasGround() ? sys.fm : sys.fm + 6;
			sys.gn = sys.hasGround() ? sys.fm : sys.fm + 6;

			max_F_size = std::max(max_F_size, sys.fm * sys.fn);
			max_fm = std::max(max_fm, sys.fm);
			max_fn = std::max(max_fn, sys.fn);
			max_G_size = std::max(max_G_size, sys.gm * sys.gn);
			max_gm = std::max(max_gm, sys.gm);
			max_gn = std::max(max_gn, sys.gn);
		}

		// 分配根据part id寻找diag的vector //
		imp_->get_diag_from_part_id_.clear();
		imp_->get_diag_from_part_id_.resize(ancestor<Model>()->partPool().size(), nullptr);
		for (auto &sys : imp_->subsys_pool_)for (auto &diag : sys.diag_pool_)imp_->get_diag_from_part_id_.at(diag.part->id()) = &diag;

		// 分配计算所需内存 //
		imp_->F_.resize(max_F_size);
		imp_->FU_.resize(max_F_size);
		imp_->FT_.resize(std::max(max_fm, max_fn));
		imp_->FP_.resize(std::max(max_fm, max_fn));
		imp_->G_.resize(max_G_size);
		imp_->GU_.resize(max_G_size);
		imp_->GT_.resize(std::max(max_gm, max_gn));
		imp_->GP_.resize(std::max(max_gm, max_gn));
		imp_->S_.resize(max_fm*max_fm, 0.0);
		imp_->beta_.resize(max_gn, 0.0); // beta可能会存储无地面处的未知量
		imp_->QT_DOT_G_.resize(max_G_size, 0.0);
		imp_->xcf_.resize(std::max(max_fn, max_fm)); //s_house_holder_ut_q_dot要求x > b
		imp_->xpf_.resize(std::max(max_fn, max_fm));
		imp_->bcf_.resize(max_fn);
		imp_->bpf_.resize(max_fm);
		imp_->cmI.resize(max_cm_size * 6);
		imp_->cmJ.resize(max_cm_size * 6);
		imp_->cmU.resize(max_cm_size * 6);
		imp_->cmT.resize(std::max(Size(6), max_cm_size));

		// 将内存付给子系统 //
		for (auto &sys : imp_->subsys_pool_)
		{
			sys.has_ground_ = sys.diag_pool_.begin()->part == &ancestor<Model>()->ground();

			sys.F = imp_->F_.data();
			sys.FU = imp_->FU_.data();
			sys.FT = imp_->FT_.data();
			sys.FP = imp_->FP_.data();
			sys.G = imp_->G_.data();
			sys.GU = imp_->GU_.data();
			sys.GT = imp_->GT_.data();
			sys.GP = imp_->GP_.data();
			sys.S = imp_->S_.data();
			sys.beta = imp_->beta_.data();
			sys.QT_DOT_G = imp_->QT_DOT_G_.data();
			sys.xcf = imp_->xcf_.data();
			sys.xpf = imp_->xpf_.data();
			sys.bcf = imp_->bcf_.data();
			sys.bpf = imp_->bpf_.data();

			for (auto &diag : sys.diag_pool_)
			{
				diag.cmI = imp_->cmI.data();
				diag.cmJ = imp_->cmJ.data();
				diag.cmU = imp_->cmU.data();
				diag.cmT = imp_->cmT.data();
			}
		}

		// 分配内存给雅可比 //
		imp_->Jg_.resize(ancestor<Model>()->partPool().size() * 6 * (ancestor<Model>()->motionPool().size() + ancestor<Model>()->generalMotionPool().size() * 6));
		imp_->cg_.resize(ancestor<Model>()->partPool().size() * 6);

		// 分配内存给动力学通用形式 //
		imp_->M_.resize((ancestor<Model>()->motionPool().size() + ancestor<Model>()->generalMotionPool().size() * 6) * (ancestor<Model>()->motionPool().size() + ancestor<Model>()->generalMotionPool().size() * 6));
		imp_->h_.resize((ancestor<Model>()->motionPool().size() + ancestor<Model>()->generalMotionPool().size() * 6));
	}
	auto UniversalSolver::kinPos()->bool
	{
		double pm[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
		s_mc(4, 4, pm, const_cast<double *>(*ancestor<Model>()->ground().pm()));

		// 将杆件位姿拷贝到局部变量中 //
		for (auto &sys : imp_->subsys_pool_)for (auto &d : sys.diag_pool_)d.part->getPm(d.pm);

		setError(0.0);
		setIterCount(0);
		for (auto &sys : imp_->subsys_pool_)
		{
			sys.max_error_ = maxError();
			sys.max_iter_count_ = maxIterCount();
			sys.kinPos();

			setIterCount(std::max(iterCount(), sys.iter_count_));
			setError(std::max(error(), sys.error_));
		}

		// 迭代成功，设置各杆件 //
		if (error() < maxError())for (auto &sys : imp_->subsys_pool_)for (auto &d : sys.diag_pool_)const_cast<Part*>(d.part)->setPm(d.pm);
		return error() < maxError();
	}
	auto UniversalSolver::kinVel()->void
	{
		for (auto &sys : imp_->subsys_pool_)for (auto &d : sys.diag_pool_)d.part->getPm(d.pm);

		s_fill(6, 1, 0.0, const_cast<double *>(ancestor<Model>()->ground().vs()));
		for (auto &sys : imp_->subsys_pool_)sys.kinVel();

		// 计算成功，设置各杆件 //
		for (auto &sys : imp_->subsys_pool_) for (auto &d : sys.diag_pool_)s_va(6, d.xp, const_cast<double*>(d.part->vs()));
	}
	auto UniversalSolver::dynAccAndFce()->void
	{
		// 更新杆件位姿，每个杆件外力 //
		for (auto &sys : imp_->subsys_pool_) 
		{
			for (auto &d : sys.diag_pool_) 
			{
				d.part->getPm(d.pm);
				std::fill(d.bp, d.bp + 6, 0.0);
			}
		}

		// 更新外力 //
		for (auto &fce : ancestor<Model>()->forcePool())
		{
			if (fce.active())
			{
				double fsI[6], fsJ[6];
				fce.cptGlbFs(fsI, fsJ);

				s_vs(6, fsI, imp_->get_diag_from_part_id_.at(fce.makI().fatherPart().id())->bp);
				s_vs(6, fsJ, imp_->get_diag_from_part_id_.at(fce.makJ().fatherPart().id())->bp);
			}
		}

		// 更新地面的as //
		s_fill(6, 1, 0.0, const_cast<double *>(ancestor<Model>()->ground().as()));
		for (auto &sys : imp_->subsys_pool_) sys.dynAccAndFce();

		// 计算成功，设置各关节和杆件
		for (auto &sys : imp_->subsys_pool_)
		{
			for (auto &r : sys.remainder_pool_)
			{
				Size pos{ 0 };
				// 将Xcf更新 //
				for (auto &c : r.rel_.cst_pool_)
				{
					const_cast<Constraint*>(c.constraint)->setCf(r.xc + pos);
					pos += c.constraint->dim();
				}
			}
			for (auto d = sys.diag_pool_.begin() + 1; d < sys.diag_pool_.end(); ++d)
			{
				Size pos{ 0 };
				for (auto &c : d->rel_.cst_pool_)
				{
					const_cast<Constraint*>(c.constraint)->setCf(d->xc + pos);
					pos += c.constraint->dim();
				}
			}

			for (auto &d : sys.diag_pool_)s_vc(6, d.xp, const_cast<double*>(d.part->as()));
		}
	}
	auto UniversalSolver::cptGeneralJacobi()noexcept->void
	{
		std::fill(imp_->Jg_.begin(), imp_->Jg_.end(), 0.0);
		std::fill(imp_->cg_.begin(), imp_->cg_.end(), 0.0);

		for (auto &sys : imp_->subsys_pool_)
		{
			for (auto &d : sys.diag_pool_)d.part->getPm(d.pm);

			sys.updMakPm();

			// make A
			sys.updDmCm();

			// solve
			sys.updF();
			for (auto &d : sys.diag_pool_) std::fill(d.bc, d.bc + d.rel_.size, 0.0);
			for (auto &r : sys.remainder_pool_) std::fill(r.bc, r.bc + r.rel_.size, 0.0);

			// upd Jg //
			auto getJacobiColumn = [&](Relation &rel, double *bc)
			{
				Size pos = 0;
				for (auto &c : rel.cst_pool_)
				{
					if (auto mot = dynamic_cast<const Motion*>(c.constraint))
					{
						// 更新bc，将当前电机的未知量更新为当前c的1.0 //
						bc[pos] = 1.0;
						sys.sovXp();
						for (auto &d : sys.diag_pool_)s_vc(6, d.xp, 1, &imp_->Jg_.at(at(d.part->id() * 6, mot->id(), nJg())), nJg());
						bc[pos] = 0.0;
					}
					else if (auto gmt = dynamic_cast<const GeneralMotion*>(c.constraint))
					{
						double tmf[6][6];
						s_tmf(*gmt->mpm(), *tmf);

						for (Size k(-1); ++k < 6;)
						{
							// 更新bc，将当前电机的未知量更新为当前c的1.0 //
							// Tmf^(T) * v //
							s_vc(6, tmf[k], bc);
							sys.sovXp();
							for (auto &d : sys.diag_pool_)s_vc(6, d.xp, 1, &imp_->Jg_.at(at(d.part->id() * 6, this->ancestor<Model>()->motionPool().size() + gmt->id() * 6 + k, nJg())), nJg());
						}

						std::fill(bc, bc + 6, 0.0);
					}

					pos += c.constraint->dim();
				}
			};
			for (auto &d : sys.diag_pool_) getJacobiColumn(d.rel_, d.bc);
			for (auto &r : sys.remainder_pool_) getJacobiColumn(r.rel_, r.bc);

			// upd cg //
			sys.updCaToBc();
			auto clearMotionMa = [&](Relation &rel, double *bc)
			{
				Size pos = 0;
				for (auto &c : rel.cst_pool_)
				{
					if (auto mot = dynamic_cast<const Motion*>(c.constraint))
					{
						bc[pos] -= mot->ma();
					}
					else if (auto gm = dynamic_cast<const GeneralMotion*>(c.constraint))
					{
						s_inv_tva(-1.0, *gm->mpm(), gm->mas(), bc);
					}

					pos += c.constraint->dim();
				}
			};
			for (auto &d : sys.diag_pool_)clearMotionMa(d.rel_, d.bc);
			for (auto &r : sys.remainder_pool_)clearMotionMa(r.rel_, r.bc);
			sys.sovXp();
			for (auto &d : sys.diag_pool_)s_vc(6, d.xp, imp_->cg_.data() + d.part->id() * 6);
		}
	}
	auto UniversalSolver::mJg()const noexcept->Size { return ancestor<Model>()->partPool().size() * 6; }
	auto UniversalSolver::nJg()const noexcept->Size { return ancestor<Model>()->motionPool().size() + ancestor<Model>()->generalMotionPool().size() * 6; }
	auto UniversalSolver::Jg()const noexcept->const double * { return imp_->Jg_.data(); }
	auto UniversalSolver::cg()const noexcept->const double * { return imp_->cg_.data(); }
	auto UniversalSolver::cptGeneralInverseDynamicMatrix()noexcept->void
	{
		// 更新地面的as //
		std::fill(imp_->M_.begin(), imp_->M_.end(), 0.0);
		std::fill(imp_->h_.begin(), imp_->h_.end(), 0.0);

		for (auto &sys : imp_->subsys_pool_)
		{
			for (auto &d : sys.diag_pool_)d.part->getPm(d.pm);
			
			sys.updMakPm();
			// 动力学计算，和dynAccAndFce() 一模一样 //
			sys.updDiagIv();
			sys.updDmCm();

			sys.updF();
			sys.updG();
			sys.updCaToBc();

			auto dynamic = [&]()
			{
				for (auto &sys : imp_->subsys_pool_)for (auto &d : sys.diag_pool_)std::fill(d.bp, d.bp + 6, 0.0);
				for (auto &fce : this->ancestor<Model>()->forcePool())
				{
					if (fce.active())
					{
						double fsI[6], fsJ[6];
						fce.cptGlbFs(fsI, fsJ);
						s_vs(6, fsI, imp_->get_diag_from_part_id_.at(fce.makI().fatherPart().id())->bp);
						s_vs(6, fsJ, imp_->get_diag_from_part_id_.at(fce.makJ().fatherPart().id())->bp);
					}
				}
				
				sys.sovXp();
				sys.sovXc();
			};

			// 开始计算h //
			// 先去掉驱动的加速度, 并计算h
			auto clearMotionMa = [&](Relation &rel, double *bc)
			{
				Size pos = 0;
				for (auto &c : rel.cst_pool_)
				{
					if (auto mot = dynamic_cast<const Motion*>(c.constraint))
					{
						bc[pos] -= mot->ma();
					}
					else if (auto gm = dynamic_cast<const GeneralMotion*>(c.constraint))
					{
						s_inv_tva(-1.0, *gm->mpm(), gm->mas(), bc);
					}

					pos += c.constraint->dim();
				}
			};
			for (auto &d : sys.diag_pool_)clearMotionMa(d.rel_, d.bc);
			for (auto &r : sys.remainder_pool_)clearMotionMa(r.rel_, r.bc);

			// 动力学计算并取出h
			dynamic();
			auto getH = [&](Relation &rel, double *xc)
			{
				Size pos{ 0 };
				// 将Xcf更新 //
				for (auto &c : rel.cst_pool_)
				{
					if (auto mot = dynamic_cast<const Motion*>(c.constraint))
					{
						imp_->h_[mot->id()] = xc[pos];
					}
					if (dynamic_cast<const GeneralMotion*>(c.constraint))
					{
						s_vc(6, xc + pos, imp_->h_.data() + this->ancestor<Model>()->motionPool().size() + c.constraint->id() * 6);
					}
					pos += c.constraint->dim();
				}
			};
			for (auto &r : sys.remainder_pool_)getH(r.rel_, r.xc);
			for (auto &d : sys.diag_pool_) getH(d.rel_, d.xc);

			// 开始计算M //
			auto getMColumn = [&](const Constraint *c, Size cid)
			{
				auto Mn = this->ancestor<Model>()->motionPool().size() + this->ancestor<Model>()->generalMotionPool().size() * 6;
				auto getMRow = [&](Relation &rel, double *xc)
				{
					Size pos2{ 0 };
					for (auto &cc : rel.cst_pool_)
					{
						if (dynamic_cast<const Motion*>(cc.constraint))
						{
							Size ccid = cc.constraint->id();
							imp_->M_[at(ccid, cid, Mn)] = xc[pos2] - h()[ccid];
						}
						if (dynamic_cast<const GeneralMotion*>(cc.constraint))
						{
							Size ccid = cc.constraint->id() * 6 + this->ancestor<Model>()->motionPool().size();
							for (Size i = 0; i < 6; ++i)
							{
								s_vc(6, xc, 1, imp_->M_.data() + at(ccid, cid, Mn), Mn);
								s_vs(6, h() + ccid, 1, imp_->M_.data() + at(ccid, cid, Mn), Mn);
							}

						}
						pos2 += cc.constraint->dim();
					}
				};
				for (auto &r : sys.remainder_pool_)getMRow(r.rel_, r.xc);
				for (auto &d : sys.diag_pool_)getMRow(d.rel_, d.xc);
			};
			auto getM = [&](Relation &rel, double *bc)
			{
				Size pos{ 0 };
				for (auto &c : rel.cst_pool_)
				{
					if (dynamic_cast<const Motion*>(c.constraint))
					{
						bc[pos] += 1.0;

						dynamic();

						getMColumn(c.constraint, c.constraint->id());

						bc[pos] -= 1.0;
					}
					else if (dynamic_cast<const GeneralMotion*>(c.constraint))
					{
						double tmf[6][6];
						s_tmf(*dynamic_cast<const GeneralMotion*>(c.constraint)->mpm(), *tmf);
						for (Size i = 0; i < 6; ++i)
						{
							s_va(6, tmf[i], bc);

							dynamic();

							getMColumn(c.constraint, c.constraint->id() * 6 + this->ancestor<Model>()->motionPool().size() + i);

							s_vs(6, tmf[i], bc);
						}
					}
					pos += c.constraint->dim();
				}
			};
			for (auto &d : sys.diag_pool_) getM(d.rel_, d.bc);
			for (auto &r : sys.remainder_pool_) getM(r.rel_, r.bc);
		}
	}
	auto UniversalSolver::nM()const noexcept->Size { return ancestor<Model>()->motionPool().size() + ancestor<Model>()->generalMotionPool().size() * 6; }
	auto UniversalSolver::M()const noexcept->const double * { return imp_->M_.data(); }
	auto UniversalSolver::h()const noexcept->const double * { return imp_->h_.data(); }
	auto UniversalSolver::plotRelation()->void
	{
		for (Size i = 0; i < imp_->subsys_pool_.size(); ++i)
		{
			std::cout << "sub sys " << std::to_string(i) << ":" << std::endl;
			std::cout << "fm:" << imp_->subsys_pool_.at(i).fm << " fn:" << imp_->subsys_pool_.at(i).fn << std::endl;
			auto &sys = imp_->subsys_pool_.at(i);

			std::size_t name_size{ 0 };
			for (auto &d1 : sys.diag_pool_)name_size = std::max(d1.part->name().size(), name_size);

			/////  plot  origin /////
			for (auto &d1 : sys.diag_pool_)
			{
				std::string s(name_size, ' ');
				s.replace(0, d1.part->name().size(), d1.part->name().data());

				std::cout << s << ":";
				for (auto &d : sys.diag_pool_)
				{
					if (d.rel_.cst_pool_.size() == 0)
					{
						if (d1.part == &ancestor<Model>()->ground())
						{
							std::cout << "  6x6 ";
						}
						else
							std::cout << "      ";

						continue;
					}

					auto &rel = d.rel_;
					std::cout << " ";
					if (rel.cst_pool_.begin()->is_I && rel.prtI == d1.part)
					{
						std::cout << " 6x" << rel.size;
					}
					else if (rel.cst_pool_.begin()->is_I && rel.prtJ == d1.part)
					{
						std::cout << "-6x" << rel.size;
					}
					else if (rel.prtI == d1.part)
					{
						std::cout << "-6x" << rel.size;
					}
					else if (rel.prtJ == d1.part)
					{
						std::cout << " 6x" << rel.size;
					}
					else
						std::cout << "    ";
					std::cout << " ";

				}
				for (auto &r : sys.remainder_pool_)
				{
					std::cout << " ";


					if (r.rel_.prtI == d1.part)
						std::cout << " 6x" << r.rel_.size;
					else if (r.rel_.prtJ == d1.part)
						std::cout << "-6x" << r.rel_.size;
					else
						std::cout << "    ";

					std::cout << " ";
				}
				std::cout << std::endl;
			}
			std::cout << std::endl;
			/////  plot  diag /////
			for (auto &d1 : sys.diag_pool_)
			{
				std::string s(name_size, ' ');
				s.replace(0, d1.part->name().size(), d1.part->name().data());

				std::cout << s << ":";
				for (auto &d : sys.diag_pool_)
				{
					if (d.rel_.cst_pool_.size() == 0)
					{
						if (d1.part == &ancestor<Model>()->ground())
						{
							std::cout << "  6x6 ";
						}
						else
							std::cout << "      ";

						continue;
					}

					auto &rel = d.rel_;
					std::cout << " ";
					if (rel.cst_pool_.begin()->is_I && rel.prtI == d1.part)
					{
						std::cout << " 6x" << rel.size;
					}
					else if (!rel.cst_pool_.begin()->is_I && rel.prtI == d1.part)
					{
						std::cout << "-6x" << rel.size;
					}
					else
						std::cout << "    ";
					std::cout << " ";

				}
				for (auto &r : sys.remainder_pool_)
				{
					std::cout << " ";

					bool found{ false };
					for (auto blk : r.cm_blk_series)
					{
						if (d1.part == blk.diag->part)
						{
							found = true;
							if (blk.is_I)
								std::cout << " 6x" << r.rel_.size;
							else
								std::cout << "-6x" << r.rel_.size;
						}
					}
					if (!found)std::cout << "    ";

					std::cout << " ";
				}
				std::cout << std::endl;
			}
			std::cout << "------------------------------------------------" << std::endl << std::endl;
		}
	}
	UniversalSolver::~UniversalSolver() = default;
	UniversalSolver::UniversalSolver(const std::string &name, Size max_iter_count, double max_error) :Solver(name, max_iter_count, max_error) {}
	ARIS_DEFINE_BIG_FOUR_CPP(UniversalSolver);

	class HelpResetRAII
	{
	public:
		std::vector<bool> prt_active_, jnt_active_, mot_active_, gm_active_, fce_active_;
		Model *model_;

		HelpResetRAII(Model *model) : model_(model)
		{
			for (auto &prt : model_->partPool())prt_active_.push_back(prt.active());
			for (auto &jnt : model_->jointPool())jnt_active_.push_back(jnt.active());
			for (auto &mot : model_->motionPool())mot_active_.push_back(mot.active());
			for (auto &gm : model_->generalMotionPool())gm_active_.push_back(gm.active());
			for (auto &fce : model_->forcePool())fce_active_.push_back(fce.active());
		}
		~HelpResetRAII() 
		{
			for (auto &prt : model_->partPool())prt.activate(prt_active_[prt.id()]);
			for (auto &jnt : model_->jointPool())jnt.activate(jnt_active_[jnt.id()]);
			for (auto &mot : model_->motionPool())mot.activate(mot_active_[mot.id()]);
			for (auto &gm : model_->generalMotionPool())gm.activate(gm_active_[gm.id()]);
			for (auto &fce : model_->forcePool())fce.activate(fce_active_[fce.id()]);
		}

	};

	struct ForwardKinematicSolver::Imp { std::vector<double> J_vec_, cf_vec_; };
	auto ForwardKinematicSolver::allocateMemory()->void
	{
		HelpResetRAII help_reset(this->ancestor<Model>());
		
		for (auto &m : ancestor<Model>()->motionPool())m.activate(true);
		for (auto &gm : ancestor<Model>()->generalMotionPool())gm.activate(false);

		imp_->J_vec_.resize(6 * ancestor<Model>()->generalMotionPool().size() * ancestor<Model>()->motionPool().size());
		imp_->cf_vec_.resize(6 * ancestor<Model>()->generalMotionPool().size());

		UniversalSolver::allocateMemory();
	}
	auto ForwardKinematicSolver::kinPos()->bool
	{
		UniversalSolver::kinPos();
		if (error() < maxError())for (auto &m : ancestor<Model>()->generalMotionPool())m.updMpm();
		return error() < maxError();
	}
	auto ForwardKinematicSolver::kinVel()->void
	{
		UniversalSolver::kinVel();
		for (auto &m : ancestor<Model>()->generalMotionPool())m.updMvs();
	}
	auto ForwardKinematicSolver::dynAccAndFce()->void
	{
		UniversalSolver::dynAccAndFce();
		for (auto &m : ancestor<Model>()->generalMotionPool())m.updMas();
	}
	auto ForwardKinematicSolver::cptJacobi() noexcept->void
	{
		cptGeneralJacobi();

		// 需要根据求出末端对每个杆件造成的速度，然后针对驱动，寻找它的速度差，就求出了速度雅可比，找出加速度差，就是cfi
		for (auto &gm : ancestor<Model>()->generalMotionPool())
		{
			for (auto &mot : ancestor<Model>()->motionPool())
			{
				double tem[6];
				s_vc(6, Jg() + at(gm.makI().fatherPart().id() * 6, mot.id(), nJg()), nJg(), tem, 1);
				s_vs(6, Jg() + at(gm.makJ().fatherPart().id() * 6, mot.id(), nJg()), nJg(), tem, 1);

				s_inv_tv(*gm.makJ().pm(), tem, 1, imp_->J_vec_.data() + at(gm.id() * 6, mot.id(), nJf()), nJf());

				// 以下求cf //
				s_vc(6, cg() + gm.makI().fatherPart().id() * 6, tem);
				s_vs(6, cg() + gm.makJ().fatherPart().id() * 6, tem);
				s_inv_as2as(*gm.makJ().pm(), gm.makJ().vs(), cg() + gm.makJ().fatherPart().id() * 6, gm.makI().vs(), cg() + gm.makI().fatherPart().id() * 6, imp_->cf_vec_.data() + gm.id() * 6);
			}
		}
	}
	auto ForwardKinematicSolver::mJf()const noexcept->Size { return ancestor<Model>()->motionPool().size(); }
	auto ForwardKinematicSolver::nJf()const noexcept->Size { return ancestor<Model>()->generalMotionPool().size() * 6; }
	auto ForwardKinematicSolver::Jf()const noexcept->const double * { return imp_->J_vec_.data(); }
	auto ForwardKinematicSolver::cf()const noexcept->const double * { return imp_->cf_vec_.data(); }
	ForwardKinematicSolver::~ForwardKinematicSolver() = default;
	ForwardKinematicSolver::ForwardKinematicSolver(const std::string &name, Size max_iter_count, double max_error) :UniversalSolver(name, max_iter_count, max_error), imp_(new Imp) {}
	ARIS_DEFINE_BIG_FOUR_CPP(ForwardKinematicSolver);

	struct InverseKinematicSolver::Imp{std::vector<double> J_vec_, ci_vec_;};
	auto InverseKinematicSolver::allocateMemory()->void
	{
		HelpResetRAII help_reset(this->ancestor<Model>());
		
		for (auto &m : ancestor<Model>()->motionPool())m.activate(false);
		for (auto &gm : ancestor<Model>()->generalMotionPool())gm.activate(true);

		imp_->J_vec_.resize(6 * ancestor<Model>()->generalMotionPool().size() * ancestor<Model>()->motionPool().size());
		imp_->ci_vec_.resize(6 * ancestor<Model>()->motionPool().size());

		UniversalSolver::allocateMemory();
	}
	auto InverseKinematicSolver::kinPos()->bool
	{
		UniversalSolver::kinPos();
		if (error() < maxError())for (auto &m : ancestor<Model>()->motionPool())m.updMp();
		return error() < maxError();
	}
	auto InverseKinematicSolver::kinVel()->void
	{
		UniversalSolver::kinVel();
		for (auto &m : ancestor<Model>()->motionPool())m.updMv();
	}
	auto InverseKinematicSolver::dynAccAndFce()->void
	{
		UniversalSolver::dynAccAndFce();
		for (auto &m : ancestor<Model>()->motionPool())m.updMa();
	}
	auto InverseKinematicSolver::cptJacobi()noexcept->void
	{
		cptGeneralJacobi();

		// 需要根据求出末端对每个杆件造成的速度，然后针对驱动，寻找它的速度差，就求出了速度雅可比
		for (auto &gm : ancestor<Model>()->generalMotionPool())
		{
			for (auto &mot : ancestor<Model>()->motionPool())
			{
				for (Size i = 0; i < 6; ++i)
				{
					double tem[6], tem2[6];
					s_vc(6, Jg() + at(mot.makI().fatherPart().id() * 6, mJi() + gm.id() * 6 + i, nJg()), nJg(), tem, 1);
					s_vs(6, Jg() + at(mot.makJ().fatherPart().id() * 6, mJi() + gm.id() * 6 + i, nJg()), nJg(), tem, 1);

					s_inv_tv(*mot.makI().pm(), tem, tem2);
					imp_->J_vec_[at(mot.id(), gm.id() * 6 + i, nJi())] = tem2[mot.axis()];

					// 以下求ci //
					// 这一段相当于updMv //
					mot.makI().getVs(mot.makJ(), tem);
					double dq = tem[mot.axis()];
					s_cv(mot.makJ().vs(), mot.makI().vs(), tem2);

					// ai - aj - vi x (vi - vj) * dq = ai - aj - v1 x v2 * dq
					s_vc(6, cg() + mot.makI().fatherPart().id() * 6, tem);
					s_vs(6, cg() + mot.makJ().fatherPart().id() * 6, tem);
					s_va(6, -dq, tem2, tem);
					s_inv_tv(*mot.makI().pm(), tem, tem2);
					imp_->ci_vec_[mot.id()] = tem2[mot.axis()];
				}
			}
		}
	}
	auto InverseKinematicSolver::mJi()const noexcept->Size { return ancestor<Model>()->generalMotionPool().size() * 6; }
	auto InverseKinematicSolver::nJi()const noexcept->Size { return ancestor<Model>()->motionPool().size(); }
	auto InverseKinematicSolver::Ji()const noexcept->const double * { return imp_->J_vec_.data(); }
	auto InverseKinematicSolver::ci()const noexcept->const double * { return imp_->ci_vec_.data(); }
	InverseKinematicSolver::~InverseKinematicSolver() = default;
	InverseKinematicSolver::InverseKinematicSolver(const std::string &name, Size max_iter_count, double max_error) :UniversalSolver(name, max_iter_count, max_error), imp_(new Imp) {}
	ARIS_DEFINE_BIG_FOUR_CPP(InverseKinematicSolver);

	auto ForwardDynamicSolver::allocateMemory()->void
	{
		HelpResetRAII help_reset(this->ancestor<Model>());
		
		for (auto &m : ancestor<Model>()->motionPool())m.activate(false);
		for (auto &gm : ancestor<Model>()->generalMotionPool())gm.activate(false);
		for (auto &f : ancestor<Model>()->forcePool())f.activate(true);
		UniversalSolver::allocateMemory();
	}
	auto ForwardDynamicSolver::kinPos()->bool
	{
		UniversalSolver::kinPos();
		if (error() < maxError())for (auto &m : ancestor<Model>()->generalMotionPool())m.updMpm();
		return error() < maxError();
	}
	auto ForwardDynamicSolver::kinVel()->void
	{
		UniversalSolver::kinVel();
		for (auto &m : ancestor<Model>()->generalMotionPool())m.updMvs();
	}
	auto ForwardDynamicSolver::dynAccAndFce()->void
	{
		UniversalSolver::dynAccAndFce();
		for (auto &m : ancestor<Model>()->generalMotionPool())m.updMas();
	}
	ForwardDynamicSolver::~ForwardDynamicSolver() = default;
	ForwardDynamicSolver::ForwardDynamicSolver(const std::string &name, Size max_iter_count, double max_error) :UniversalSolver(name, max_iter_count, max_error) {}
	ARIS_DEFINE_BIG_FOUR_CPP(ForwardDynamicSolver);

	auto InverseDynamicSolver::allocateMemory()->void
	{
		HelpResetRAII help_reset(this->ancestor<Model>());
		
		for (auto &m : ancestor<Model>()->motionPool())m.activate(true);
		for (auto &gm : ancestor<Model>()->generalMotionPool())gm.activate(false);
		for (auto &f : ancestor<Model>()->forcePool())f.activate(false);
		UniversalSolver::allocateMemory();
	}
	auto InverseDynamicSolver::kinPos()->bool
	{
		UniversalSolver::kinPos();
		if (error() < maxError())for (auto &m : ancestor<Model>()->motionPool())m.updMp();
		return error() < maxError();
	}
	auto InverseDynamicSolver::kinVel()->void
	{
		UniversalSolver::kinVel();
		for (auto &m : ancestor<Model>()->motionPool())m.updMv();
	}
	auto InverseDynamicSolver::dynAccAndFce()->void
	{
		UniversalSolver::dynAccAndFce();
		for (auto &m : ancestor<Model>()->generalMotionPool())m.updMas();
	}
	InverseDynamicSolver::~InverseDynamicSolver() = default;
	InverseDynamicSolver::InverseDynamicSolver(const std::string &name, Size max_iter_count, double max_error) :UniversalSolver(name, max_iter_count, max_error) {}
	ARIS_DEFINE_BIG_FOUR_CPP(InverseDynamicSolver);
}
