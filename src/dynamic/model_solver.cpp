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

#define ARIS_LOOP_BLOCK(RELATION) for (auto b = RELATION blk_data_; b < RELATION blk_data_ + RELATION blk_size_; ++b)
#define ARIS_LOOP_D for (auto d = d_data_; d < d_data_ + d_size_; ++d)
#define ARIS_LOOP_D_2_TO_END for (auto d = d_data_ + 1; d < d_data_ + d_size_; ++d)
#define ARIS_LOOP_DIAG_INVERSE_2_TO_END for (auto d = d_data_ + d_size_ - 1; d > d_data_; --d)
#define ARIS_LOOP_R for (auto r = r_data_; r < r_data_ + r_size_; ++r)
	struct Relation
	{
		struct Block 
		{ 
			const Constraint* cst_;
			bool is_I_;
		};

		const Part *prtI_, *prtJ_; // prtI为对角块的part
		Size dim_, size_;
		
		Block* blk_data_;
		Size blk_size_;
	};
	struct LocalRelation :public Relation { std::vector<Block> cst_pool_; }; //仅仅为了实现
	struct Diag
	{
		// D * C * P =[I  C]
		//            [0  0]
		// 对于存在多个约束的relation来说，P有意义
		Size *p_;
		double dm_[36], iv_[10];
		double pm1_[16], pm2_[16], *pm_, *last_pm_;
		double xp_[6], bp_[6], *bc_, *xc_;
		double *cmI_, *cmJ_, *cmU_, *cmT_; // 因为可能有多个约束，总约束的个数可能超过6，cm维数也不确定

		Size rows_;// in F
		const Part *part_;
		Diag *rd_;//related diag, for row addition
		Relation rel_;

		typedef void(*UpdFunc2)(Diag*, bool cpt_cp);
		UpdFunc2 upd_d_and_cp_;
	};
	struct Remainder
	{
		struct Block { Diag* diag_; bool is_I_; };

		Diag *i_diag_, *j_diag_;
		double *cmI_, *cmJ_, *bc_, *xc_;

		Block* blk_data_;
		Size blk_size_;

		Relation rel_;
	};
	struct LocalRemainder : public Remainder { std::vector<Block> cm_blk_series; };
	struct PublicData;
	struct SubSystem
	{
		PublicData *pd_;
		
		Diag* d_data_;
		Size d_size_;
		
		Remainder* r_data_;
		Size r_size_;

		Size fm_, fn_, fr_, gm_, gn_;

		bool has_ground_;
		double error_, max_error_;
		Size iter_count_, max_iter_count_;

		auto hasGround()const noexcept->bool { return has_ground_; }
		// 从模型中跟新数据 //
		auto updDmCm(bool cpt_cp)noexcept->void;
		auto updDiagIv()noexcept->void;
		auto updCv()noexcept->void;
		auto updCa()noexcept->void;
		// 求解 //
		auto updF()noexcept->void;
		auto sovXp()noexcept->void;
		auto updG()noexcept->void;
		auto sovXc()noexcept->void;
		// 接口 //
		auto kinPos()noexcept->void;
		auto kinVel()noexcept->void;
		auto dynAccAndFce()noexcept->void;
	};
	struct PublicData
	{
		SubSystem *subsys_data_;
		Size subsys_size_;
		Diag** get_diag_from_part_id_;

		double *F_, *FU_, *FT_, *G_, *GU_, *GT_, *S_, *QT_DOT_G_, *xpf_, *xcf_, *bpf_, *bcf_, *beta_, *cmI_, *cmJ_, *cmU_, *cmT_;
		Size *FP_, *GP_;
		double *Jg_, *cg_, *M_, *h_;
	};
	auto SubSystem::updDmCm(bool cpt_cp)noexcept->void
	{
		if (cpt_cp)error_ = 0.0;// error //
		
		// upd dm and rel dim
		fm_ = 0;
		ARIS_LOOP_D_2_TO_END
		{
			d->upd_d_and_cp_(d, cpt_cp);// cp //
			d->rows_ = fm_;
			fm_ += 6 - d->rel_.dim_;
			if (cpt_cp)for (Size i{ 0 }; i < d->rel_.size_; ++i) error_ = std::max(error_, std::abs(d->bc_[i]));// error //
		}

		// upd remainder data //
		ARIS_LOOP_R
		{
			Size pos{ 0 };
			ARIS_LOOP_BLOCK(r->rel_.)
			{
				double pmI[16], pmJ[16];
				s_pm_dot_pm(b->is_I_ ? r->i_diag_->pm_ : r->j_diag_->pm_, *b->cst_->makI().prtPm(), pmI);
				s_pm_dot_pm(b->is_I_ ? r->j_diag_->pm_ : r->i_diag_->pm_, *b->cst_->makJ().prtPm(), pmJ);

				if (cpt_cp)b->cst_->cptCpFromPm(r->bc_ + pos, pmI, pmJ);// cp //

				double cmI[36], cmJ[36];
				b->cst_->cptGlbCmFromPm(cmI, cmJ, pmI, pmJ);
				s_mc(6, b->cst_->dim(), cmI, b->cst_->dim(), r->cmI_ + pos, r->rel_.size_);
				s_mc(6, b->cst_->dim(), cmJ, b->cst_->dim(), r->cmJ_ + pos, r->rel_.size_);
				pos += b->cst_->dim();
			}

			if (cpt_cp)for (Size i{ 0 }; i < r->rel_.size_; ++i)error_ = std::max(error_, std::abs(r->bc_[i])); // error //
		}
	}
	auto SubSystem::updDiagIv()noexcept->void { ARIS_LOOP_D s_iv2iv(*d->part_->pm(), d->part_->prtIv(), d->iv_); }
	auto SubSystem::updCv()noexcept->void
	{
		// bc in diag //
		ARIS_LOOP_D_2_TO_END
		{
			Size pos{ 0 };
			ARIS_LOOP_BLOCK(d->rel_.)
			{
				b->cst_->cptCv(d->bc_ + pos);
				pos += b->cst_->dim();
			}
		}
		// bc in remainder //
		ARIS_LOOP_R
		{
			Size pos{ 0 };
			ARIS_LOOP_BLOCK(r->rel_.)
			{
				b->cst_->cptCv(r->bc_ + pos);
				pos += b->cst_->dim();
			}
		}
	}
	auto SubSystem::updCa()noexcept->void
	{
		// bc in diag //
		ARIS_LOOP_D_2_TO_END
		{
			Size pos{ 0 };
			ARIS_LOOP_BLOCK(d->rel_.)
			{
				b->cst_->cptCa(d->bc_ + pos);
				pos += b->cst_->dim();
			}
		}
		// bc in remainder //
		ARIS_LOOP_R
		{
			Size pos{ 0 };
			ARIS_LOOP_BLOCK(r->rel_.)
			{
				b->cst_->cptCa(r->bc_ + pos);
				pos += b->cst_->dim();
			}
		}
	}
	auto SubSystem::updF()noexcept->void
	{
		// check F size //
		s_fill(fm_, fn_, 0.0, pd_->F_);

		Size cols{ 0 };
		ARIS_LOOP_R
		{
			ARIS_LOOP_BLOCK(r->)
			{
				s_mm(6 - b->diag_->rel_.dim_, r->rel_.size_, 6, b->diag_->dm_ + at(b->diag_->rel_.dim_, 0, 6), 6, b->is_I_ ? r->cmI_ : r->cmJ_, r->rel_.size_, pd_->F_ + at(b->diag_->rows_, cols, ColMajor(fm_)), ColMajor(fm_));
			}
			cols += r->rel_.size_;
		}

		s_householder_utp(fm_, fn_, pd_->F_, ColMajor(fm_), pd_->FU_, ColMajor(fm_), pd_->FT_, 1, pd_->FP_, fr_, max_error_);
	}
	auto SubSystem::sovXp()noexcept->void
	{
		// 请参考step 4，这里先把xp做个预更新,以及初始化 //
		std::fill_n(d_data_[0].xp_, 6, 0.0);
		ARIS_LOOP_D_2_TO_END
		{
			// 如果是多个杆件，那么需要重新排序 //
			s_permutate(d->rel_.size_, 1, d->p_, d->bc_);

			// 预更新 //
			s_mm(6, 1, d->rel_.dim_, d->dm_, T(6), d->bc_, 1, d->xp_, 1);
		}

		// 构造bcf //
		Size cols{ 0 };
		ARIS_LOOP_R
		{
			s_vc(r->rel_.size_, r->bc_, pd_->bcf_ + cols);
			ARIS_LOOP_BLOCK(r->)
			{
				auto cm = b->is_I_ ? r->cmJ_ : r->cmI_;//这里是颠倒的，因为加到右侧需要乘-1.0
				s_mma(r->rel_.size_, 1, 6, cm, ColMajor{ r->rel_.size_ }, b->diag_->xp_, 1, pd_->bcf_ + cols, 1);//请参考step 4，将预更新的东西取出
			}
			cols += r->rel_.size_;
		}

		// 求解 F' * xpf = bcf //
		s_vc(fn_, pd_->bcf_, pd_->xpf_);
		s_permutate(fn_, 1, pd_->FP_, pd_->xpf_);
		s_sov_lm(fr_, 1, pd_->FU_, T(ColMajor(fm_)), pd_->xpf_, 1, pd_->xpf_, 1, max_error_);
		s_householder_ut_q_dot(fm_, fn_, 1, pd_->FU_, ColMajor(fm_), pd_->FT_, 1, pd_->xpf_, 1, pd_->xpf_, 1);

		// 更新xp //  相当于 D' * yp
		ARIS_LOOP_D_2_TO_END s_mma(6, 1, 6 - d->rel_.dim_, d->dm_ + at(0, d->rel_.dim_, T(6)), T(6), pd_->xpf_ + d->rows_, 1, d->xp_, 1);

		// 做行变换 //  相当于 P' * (D' * yp)
		ARIS_LOOP_D_2_TO_END s_va(6, d->rd_->xp_, d->xp_);
	}
	auto SubSystem::updG()noexcept->void
	{
		gm_ = hasGround() ? fm_ : fm_ + 6;
		gn_ = hasGround() ? fm_ - fr_ : fm_ - fr_ + 6;

		//////////////////////////////////////////// 求CT * xp = bc 的通解S step 5 //////////////////////////////
		std::fill_n(pd_->xpf_, fm_, 0.0);
		for (Size j(-1); ++j < fm_ - fr_;)
		{
			pd_->xpf_[fr_ + j] = 1.0;
			s_householder_ut_q_dot(fm_, fn_, 1, pd_->FU_, ColMajor(fm_), pd_->FT_, 1, pd_->xpf_, 1, pd_->S_ + j, fm_ - fr_);
			pd_->xpf_[fr_ + j] = 0.0;
		}

		//////////////////////////////////////////// 求G，参考 step 6 //////////////////////////////
		s_fill(gm_, gn_, 0.0, pd_->G_);
		// 先求S产生的G
		for (Size j(-1); ++j < fm_ - fr_;)
		{
			// 初始化xp并乘以DT
			std::fill(d_data_[0].xp_, d_data_[0].xp_ + 6, 0.0);
			ARIS_LOOP_D_2_TO_END
			{
				if (d->rel_.dim_ == 6)std::fill(d->xp_, d->xp_ + 6, 0.0);
				else s_mm(6, 1, 6 - d->rel_.dim_, d->dm_ + at(0, d->rel_.dim_, ColMajor{ 6 }), ColMajor{ 6 }, pd_->S_ + at(d->rows_, j, fm_ - fr_), fm_ - fr_, d->xp_, 1);
			}

			// 乘以PT，行加
			ARIS_LOOP_D_2_TO_END s_va(6, d->rd_->xp_, d->xp_);

			// 乘以I
			ARIS_LOOP_D_2_TO_END
			{
				double tem[6];
				s_iv_dot_as(d->iv_, d->xp_, tem);
				s_vc(6, tem, d->xp_);
			}

			// 乘以P，行加
			ARIS_LOOP_DIAG_INVERSE_2_TO_END s_va(6, d->xp_, d->rd_->xp_);

			// 乘以D，并取出来
			ARIS_LOOP_DIAG_INVERSE_2_TO_END s_mm(6 - d->rel_.dim_, 1, 6, d->dm_ + at(d->rel_.dim_, 0, 6), 6, d->xp_, 1, pd_->G_ + at(d->rows_, j, gn_), gn_);

			// 如果无地，需要考虑G中第一个杆件处的xp
			if (!hasGround())s_vc(6, d_data_->xp_, 1, pd_->G_ + at(fm_, j, fm_ - fr_ + 6), fm_ - fr_ + 6);
		}
		// 再求无地处第一个杆件处产生的G
		if (!hasGround())
		{
			for (Size j(-1); ++j < 6;)
			{
				// 初始化，此时无需乘以DT,因为第一个杆件的DT为单位阵，其他地方的xp为0
				ARIS_LOOP_D std::fill(d->xp_, d->xp_ + 6, 0.0);
				d_data_[0].xp_[j] = 1.0;

				// 乘以PT
				ARIS_LOOP_D_2_TO_END s_va(6, d->rd_->xp_, d->xp_);

				// 乘以I, 因为bp里面储存了外力，因此不能用bp
				ARIS_LOOP_D
				{
					double tem[6];
					s_iv_dot_as(d->iv_, d->xp_, tem);
					s_vc(6, tem, d->xp_);
				}

				// 乘以P, 类似rowAddBp();
				ARIS_LOOP_DIAG_INVERSE_2_TO_END s_va(6, d->xp_, d->rd_->xp_);

				// 乘以D，并取出来
				ARIS_LOOP_DIAG_INVERSE_2_TO_END s_mm(6 - d->rel_.dim_, 1, 6, d->dm_ + at(d->rel_.dim_, 0, 6), 6, d->xp_, 1, pd_->G_ + at(d->rows_, fm_ - fr_ + j, gn_), gn_);
				s_vc(6, d_data_[0].xp_, 1, pd_->G_ + at(fm_, fm_ - fr_ + j, gn_), gn_);
			}
		}
	}
	auto SubSystem::sovXc()noexcept->void
	{
		/////////////////////////////////// 求解beta /////////////////////////////////////////////////////////////
		//// 更新每个杆件的力 pf ////
		ARIS_LOOP_D
		{
			// 外力（不包括惯性力）已经储存在了bp中，但为了后续可能存在的修正，这里将其与惯性力之和暂存到 last_pm_ 中 //
			
			// v x I * v //
			double I_dot_v[6];
			s_iv_dot_as(d->iv_, d->part_->vs(), I_dot_v);
			s_cfa(d->part_->vs(), I_dot_v, d->bp_);
			s_vc(6, d->bp_, d->last_pm_); // 暂存处理

			// I*(a-g) //
			double as_minus_g[6], iv_dot_as[6];
			s_vc(6, d->xp_, as_minus_g);// xp储存加速度
			s_vs(6, d->part_->ancestor<Model>()->environment().gravity(), as_minus_g);
			s_iv_dot_as(d->iv_, as_minus_g, iv_dot_as);
			s_va(6, iv_dot_as, d->bp_);
		}

		//// P*bp  对bp做行加变换，并取出bcf ////
		ARIS_LOOP_DIAG_INVERSE_2_TO_END
		{
			// 行变换
			s_va(6, d->bp_, d->rd_->bp_);

			// 取出bcf
			double tem[6];
			s_mm(6, 1, 6, d->dm_, 6, d->bp_, 1, tem, 1);
			s_vc(6, tem, d->bp_);
			s_vc(6 - d->rel_.dim_, d->bp_ + d->rel_.dim_, pd_->bpf_ + d->rows_);
		}

		//// 根据G求解S解空间的beta, 先把beta都弄成它的右侧未知量 ////
		if (!hasGround())s_vc(6, d_data_[0].bp_, pd_->beta_ + fm_);//这一项实际是把无地面的xp拷贝到beta中
		s_householder_ut_qt_dot(fm_, fr_, 1, pd_->FU_, ColMajor(fm_), pd_->FT_, 1, pd_->bpf_, 1, pd_->beta_, 1);

		//// 求QT_DOT_G ////
		// G 和 QT_DOT_G位于同一片内存，因此不需要以下第一句
		// if (!hasGround())s_mc(gm - fm_, gn_, G + at(fm_, 0, gn_), QT_DOT_G + at(fm_, 0, gn_)); // 这一项实际是把无地面产生的G拷贝到QT_DOT_G中
		s_householder_ut_qt_dot(fm_, fr_, gn_, pd_->FU_, ColMajor(fm_), pd_->FT_, 1, pd_->G_, gn_, pd_->QT_DOT_G_, gn_);

		///////////////////////////////
		// 求解之前通解的解的系数 beta
		// 可以通过rank == m-r来判断质点等是否影响计算
		///////////////////////////////
		Size rank;
		s_householder_utp(gn_, gn_, pd_->QT_DOT_G_ + at(fr_, 0, gn_), pd_->GU_ + at(fr_, 0, gn_), pd_->GT_, pd_->GP_, rank, max_error_);
		s_householder_utp_sov(gn_, gn_, 1, rank, pd_->GU_ + at(fr_, 0, gn_), pd_->GT_, pd_->GP_, pd_->beta_ + fr_, pd_->beta_);

		/////////////////////////////////// 求解xp /////////////////////////////////////////////////////////////
		//// 重新求解 xp ，这次考虑惯量 ////
		// 根据特解更新 xpf 以及无地面处的特解（杆件1速度之前可以随便设）
		s_mms(fm_, 1, fm_ - fr_, pd_->S_, pd_->beta_, pd_->xpf_);
		if (!hasGround())s_vi(6, pd_->beta_ + fm_ - fr_, d_data_[0].xp_);
		// 将xpf更新到xp，先乘以D' 再乘以 P'
		ARIS_LOOP_D_2_TO_END
		{
			// 结合bc 并乘以D'
			s_mm(6, 1, d->rel_.dim_, d->dm_, ColMajor{ 6 }, d->bc_, 1, d->xp_, 1);
			s_mma(6, 1, 6 - d->rel_.dim_, d->dm_ + at(0, d->rel_.dim_, T(6)), T(6), pd_->xpf_ + d->rows_, 1, d->xp_, 1);

			// 乘以P'
			s_va(6, d->rd_->xp_, d->xp_);
		}

		/////////////////////////////////// 求解xc /////////////////////////////////////////////////////////////
		// 因为上文中 xp 可能不是真实解，这里重新做循环计算
		//// 更新每个杆件的力 pf ////
		ARIS_LOOP_D
		{
			// 取出之前暂存的外力 //
			s_vc(6, d->last_pm_, d->bp_);

			// I*(a-g) //
			double as_minus_g[6], iv_dot_as[6];
			s_vc(6, d->xp_, as_minus_g);// xp储存加速度
			s_vs(6, d->part_->ancestor<Model>()->environment().gravity(), as_minus_g);
			s_iv_dot_as(d->iv_, as_minus_g, iv_dot_as);
			s_va(6, iv_dot_as, d->bp_);
		}

		//// P*bp  对bp做行加变换 ////
		ARIS_LOOP_DIAG_INVERSE_2_TO_END 
		{
			s_va(6, d->bp_, d->rd_->bp_);
			
			double tem[6];
			s_mm(6, 1, 6, d->dm_, 6, d->bp_, 1, tem, 1);
			s_vc(6, tem, d->bp_);
			s_vc(6 - d->rel_.dim_, d->bp_ + d->rel_.dim_, pd_->bpf_ + d->rows_);
		}
		
		// 求解xcf //
		s_householder_utp_sov(fm_, fn_, 1, fr_, pd_->FU_, ColMajor(fm_), pd_->FT_, 1, pd_->FP_, pd_->bpf_, 1, pd_->xcf_, 1, max_error_);

		// 将已经求出的x更新到remainder中，此后将已知数移到右侧
		Size cols{ 0 };
		ARIS_LOOP_R
		{
			s_vc(r->rel_.size_, pd_->xcf_ + cols, r->xc_);

			// 更新待求
			ARIS_LOOP_BLOCK(r->)
			{
				double tem[6];
				s_mm(6, 1, r->rel_.size_, b->is_I_ ? r->cmJ_ : r->cmI_, pd_->xcf_ + cols, tem);
				s_mma(6, 1, 6, b->diag_->dm_, 6, tem, 1, b->diag_->bp_, 1);
			}

			cols += r->rel_.size_;
		}
		ARIS_LOOP_D_2_TO_END
		{
			s_vc(d->rel_.dim_, d->bp_, d->xc_);
			std::fill(d->xc_ + d->rel_.dim_, d->xc_ + d->rel_.size_, 0.0);
			s_permutate_inv(d->rel_.size_, 1, d->p_, d->xc_);
		}
	}
	auto SubSystem::kinPos()noexcept->void
	{
		updDmCm(true);
		for (iter_count_ = 0; iter_count_ < max_iter_count_; ++iter_count_)
		{
			if (error_ < max_error_) return;

			// solve
			updF();
			sovXp();

			// 将xp更新成pm
			ARIS_LOOP_D
			{
				std::swap(d->pm_, d->last_pm_);
				double tem[16];
				s_ps2pm(d->xp_, tem);
				s_pm2pm(tem, d->last_pm_, d->pm_);
			}

			double last_error = error_;
			updDmCm(true);

			// 对于非串联臂，当迭代误差反而再增大时，会主动缩小步长
			if (r_size_)// 只有不是串联臂才会用以下迭代
			{
				if (error_ > last_error)// 这里如果用while可以确保每个循环误差都会减小，但是有可能出现卡死
				{
					double coe = last_error / (error_ + last_error);

					ARIS_LOOP_D
					{
						s_nv(6, coe, d->xp_);
						double tem[16];
						s_ps2pm(d->xp_, tem);
						s_pm2pm(tem, d->last_pm_, d->pm_);
					}

					updDmCm(true);
				}
			}
		}
	}
	auto SubSystem::kinVel()noexcept->void
	{
		// make b
		updCv();

		// make A
		updDmCm(false);

		// solve
		updF();
		sovXp();
	}
	auto SubSystem::dynAccAndFce()noexcept->void
	{
		// upd Iv dm cm and ca  //
		updDiagIv();
		updDmCm(false);

		// upd F and G //
		updF();
		updG();

		//// 求解 xp 的某个特解（不考虑惯量），求出beta 以及 xc
		updCa();
		sovXp();
		sovXc();
	}
#undef ARIS_LOOP_D
#undef ARIS_LOOP_D_2_TO_END
#undef ARIS_LOOP_DIAG_INVERSE_2_TO_END
#undef ARIS_LOOP_R
#define ARIS_LOOP_SYS for (auto sys = imp_->pd_->subsys_data_; sys < imp_->pd_->subsys_data_ + imp_->pd_->subsys_size_; ++sys)
#define ARIS_LOOP_SYS_D for (auto d = sys->d_data_; d < sys->d_data_ + sys->d_size_; ++d)
#define ARIS_LOOP_SYS_R for (auto r = sys->r_data_; r < sys->r_data_ + sys->r_size_; ++r)
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
		//        F * [ ypf2 ]  =  [  bcm  ]   -   [ -Cm'  * D1(1:k,1:6)' * bc1 + ... + Cm'  * Dm-1(1:k,1:6)' * bcm-1 ]
		//            | ypf3 |     | bcm+1 |       |  Cm+1'* D1(1:k,1:6)' * bc1 + ... + Cm+1'* Dm-1(1:k,1:6)' * bcm-1 |
		//            |  ... |     |  ...  |       |                              ...                                 |
		//            [ ypfm ]     [  bcn  ]       [ -Cn'  * D1(1:k,1:6)' * bc1 + ... + Cn'  * Dm-1(1:k,1:6)' * bcm-1 ]
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
		// step 7:求出G后，可以进行下一步，求取 beta
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
		
		PublicData *pd_;
		std::vector<char> mem_pool_;

		static auto one_constraint_upd_d_and_cp(Diag *d, bool cpt_cp)noexcept->void
		{
			// 更新 pm //
			double pmI[16], pmJ[16];
			auto b = &d->rel_.blk_data_[0];
			s_pm_dot_pm(b->is_I_ ? d->pm_ : d->rd_->pm_, *b->cst_->makI().prtPm(), pmI);
			s_pm_dot_pm(b->is_I_ ? d->rd_->pm_ : d->pm_, *b->cst_->makJ().prtPm(), pmJ);
			
			// 计算 dm //
			d->rel_.blk_data_[0].cst_->cptGlbDmFromPm(d->dm_, pmI, pmJ);
			if (!d->rel_.blk_data_[0].is_I_)s_iv(36, d->dm_);
			
			// 计算 cp //
			if (cpt_cp)d->rel_.blk_data_[0].cst_->cptCpFromPm(d->bc_, pmI, pmJ);
		}
		static auto revolute_upd_d_and_cp(Diag *d, bool cpt_cp)noexcept->void
		{
			// 更新 pm //
			double pmI[16], pmJ[16];
			auto b = &d->rel_.blk_data_[0];
			s_pm_dot_pm(b->is_I_ ? d->pm_ : d->rd_->pm_, *b->cst_->makI().prtPm(), pmI);
			s_pm_dot_pm(b->is_I_ ? d->rd_->pm_ : d->pm_, *b->cst_->makJ().prtPm(), pmJ);
			
			// 计算 dm //
			d->rel_.blk_data_[0].cst_->cptGlbDmFromPm(d->dm_, pmI, pmJ);
			if (!d->rel_.blk_data_[0].is_I_)s_iv(36, d->dm_);
			
			// 计算 cp //
			if (cpt_cp)
			{
				auto m = static_cast<const Motion*>(d->rel_.blk_data_[1].cst_);

				double rm[9], pm_j_should_be[16];
				s_rmz(m->mpInternal(), rm);

				s_vc(16, pmJ, pm_j_should_be);
				s_mm(3, 3, 3, pmJ, 4, rm, 3, pm_j_should_be, 4);

				double pm_j2i[16], ps_j2i[6];
				s_inv_pm_dot_pm(pmI, pm_j_should_be, pm_j2i);
				s_pm2ps(pm_j2i, ps_j2i);

				// motion所对应的cp在最后 //
				s_vc(m->axis(), ps_j2i, d->bc_);
				s_vc(5 - m->axis(), ps_j2i + m->axis() + 1, d->bc_ + m->axis());
				d->bc_[5] = ps_j2i[m->axis()];
			}
		}
		static auto prismatic_upd_d_and_cp(Diag *d, bool cpt_cp)noexcept->void
		{
			// 更新 pm //
			double pmI[16], pmJ[16];
			auto b = &d->rel_.blk_data_[0];
			s_pm_dot_pm(b->is_I_ ? d->pm_ : d->rd_->pm_, *b->cst_->makI().prtPm(), pmI);
			s_pm_dot_pm(b->is_I_ ? d->rd_->pm_ : d->pm_, *b->cst_->makJ().prtPm(), pmJ);

			// 计算 dm //
			d->rel_.blk_data_[0].cst_->cptGlbDmFromPm(d->dm_, pmI, pmJ);
			if (!d->rel_.blk_data_[0].is_I_)s_iv(36, d->dm_);
			
			// 计算 cp //
			if (cpt_cp)
			{
				auto m = static_cast<const Motion*>(d->rel_.blk_data_[1].cst_);

				double pm_j_should_be[16];
				s_vc(16, pmJ, pm_j_should_be);
				s_va(3, m->mpInternal(), pm_j_should_be + m->axis(), 4, pm_j_should_be + 3, 4);

				double pm_j2i[16], ps_j2i[6];
				s_inv_pm_dot_pm(pmI, pm_j_should_be, pm_j2i);
				s_pm2ps(pm_j2i, ps_j2i);

				// motion所对应的cp在最后 //
				s_vc(m->axis(), ps_j2i, d->bc_);
				s_vc(5 - m->axis(), ps_j2i + m->axis() + 1, d->bc_ + m->axis());
				d->bc_[5] = ps_j2i[m->axis()];
			}
		}
		static auto normal_upd_d_and_cp(Diag *d, bool cpt_cp)noexcept->void
		{
			Size pos{ 0 };
			ARIS_LOOP_BLOCK(d->rel_.)
			{
				// 更新 pm //
				double pmI[16], pmJ[16];
				s_pm_dot_pm(b->is_I_ ? d->pm_ : d->rd_->pm_, *b->cst_->makI().prtPm(), pmI);
				s_pm_dot_pm(b->is_I_ ? d->rd_->pm_ : d->pm_, *b->cst_->makJ().prtPm(), pmJ);

				// 计算 cp //
				if (cpt_cp)b->cst_->cptCpFromPm(d->bc_ + pos, pmI, pmJ);

				// 计算 dm //
				double cmI_tem[36], cmJ_tem[36];
				b->cst_->cptGlbCmFromPm(cmI_tem, cmJ_tem, pmI, pmJ);

				s_mc(6, b->cst_->dim(), cmI_tem, b->cst_->dim(), (b->is_I_ ? d->cmI_ : d->cmJ_) + pos, d->rel_.size_);
				s_mc(6, b->cst_->dim(), cmJ_tem, b->cst_->dim(), (b->is_I_ ? d->cmJ_ : d->cmI_) + pos, d->rel_.size_);
				pos += b->cst_->dim();
			}

			double Q[36];
			s_householder_utp(6, d->rel_.size_, d->cmI_, d->cmU_, d->cmT_, d->p_, d->rel_.dim_);
			s_householder_ut2qr(6, d->rel_.size_, d->cmU_, d->cmT_, Q, d->cmU_);

			double tem[36]{ 1,0,0,0,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0,1 };
			s_inv_um(d->rel_.dim_, d->cmU_, d->rel_.size_, tem, 6);
			s_mm(6, 6, 6, tem, 6, Q, dynamic::ColMajor{ 6 }, d->dm_, 6);
		}

		template<typename T>
		static auto allocMem(Size &mem_pool_size, T* &pointer, Size size)->void
		{
			*reinterpret_cast<Size*>(&pointer) = mem_pool_size;
			mem_pool_size += sizeof(T) * size;
		}
		template<typename T>
		static auto getMem(char *mem_pool, T* &pointer)->T*
		{
			return reinterpret_cast<T*>(mem_pool + *reinterpret_cast<Size*>(&pointer));
		}
	};
	auto UniversalSolver::allocateMemory()->void
	{
		// for mem_pool
		Size mem_pool_size = 0;

		// 根据关联拓扑，计算出 part 和 rel 的分组，用于构建subsys
		std::vector<std::vector<const Part*>> prt_vec_vec;
		std::vector<std::vector<LocalRelation>> rel_vec_vec;
		{
			// make active part pool //
			std::vector<const Part*> active_part_pool;
			active_part_pool.push_back(&ancestor<Model>()->ground());
			for (auto &p : ancestor<Model>()->partPool())if (p.active() && &p != &ancestor<Model>()->ground())active_part_pool.push_back(&p);

			// make active constraint pool //
			std::vector<const Constraint*> cp;
			for (auto &jnt : ancestor<Model>()->jointPool())if (jnt.active())cp.push_back(&jnt);
			for (auto &mot : ancestor<Model>()->motionPool())if (mot.active()) cp.push_back(&mot);
			for (auto &gmt : ancestor<Model>()->generalMotionPool())if (gmt.active())cp.push_back(&gmt);

			// make relation pool //
			std::vector<LocalRelation> relation_pool;
			for (auto c : cp)
			{
				auto ret = std::find_if(relation_pool.begin(), relation_pool.end(), [&c](auto &relation)
				{
					auto ri{ relation.prtI_ }, rj{ relation.prtJ_ }, ci{ &c->makI().fatherPart() }, cj{ &c->makJ().fatherPart() };
					return ((ri == ci) && (rj == cj)) || ((ri == cj) && (rj == ci));
				});

				if (ret == relation_pool.end())
				{
					relation_pool.push_back(LocalRelation{ &c->makI().fatherPart(), &c->makJ().fatherPart(), c->dim(), c->dim() });
					relation_pool.back().cst_pool_.push_back({ c, true });
				}
				else
				{
					ret->cst_pool_.push_back({ c, &c->makI().fatherPart() == ret->prtI_ });
					std::sort(ret->cst_pool_.begin(), ret->cst_pool_.end(), [](auto& a, auto& b){return a.cst_->dim() > b.cst_->dim();});//这里把大的约束往前放
					ret->size_ += c->dim();
					ret->dim_ = ret->cst_pool_[0].cst_->dim();// relation 的 dim 以大的为准，最大的在第一个
				}
			}

			// 划分出相关的Part和Relation //
			while (active_part_pool.size() > 1)
			{
				std::function<void(std::vector<const Part *> &part_pool_, std::vector<const Part *> &left_part_pool, std::vector<LocalRelation> &relation_pool, const Part *part)> addPart;
				addPart = [&](std::vector<const Part *> &part_pool_, std::vector<const Part *> &left_part_pool, std::vector<LocalRelation> &relation_pool, const Part *part)->void
				{
					if (std::find(part_pool_.begin(), part_pool_.end(), part) != part_pool_.end())return;

					part_pool_.push_back(part);

					// 如果不是地面，那么抹掉该杆件，同时添加该杆件的相关杆件 //
					if (part != left_part_pool.front())
					{
						left_part_pool.erase(std::find(left_part_pool.begin(), left_part_pool.end(), part));
						for (auto &rel : relation_pool)
						{
							if (rel.prtI_ == part || rel.prtJ_ == part)
							{
								addPart(part_pool_, left_part_pool, relation_pool, rel.prtI_ == part ? rel.prtJ_ : rel.prtI_);
							}
						}
					}
				};

				// insert prt_vec and rel_vec //
				prt_vec_vec.push_back(std::vector<const Part*>());
				rel_vec_vec.push_back(std::vector<LocalRelation>());
				auto &prt_vec = prt_vec_vec.back();
				auto &rel_vec = rel_vec_vec.back();

				// add related part //
				addPart(prt_vec, active_part_pool, relation_pool, active_part_pool.at(1));

				// add related relation //
				for (auto &rel : relation_pool)
				{
					if (std::find_if(prt_vec.begin(), prt_vec.end(), [&rel, this](const Part *prt) { return prt != &(this->ancestor<Model>()->ground()) && (prt == rel.prtI_ || prt == rel.prtJ_); }) != prt_vec.end())
					{
						rel_vec.push_back(rel);
					}
				}

				// 对sys的part和relation排序 //
				for (Size i = 0; i < std::min(prt_vec.size(), rel_vec.size()); ++i)
				{
					// 先对part排序，找出下一个跟上一个part联系的part
					std::sort(prt_vec.begin() + i, prt_vec.end(), [i, this, &rel_vec](const Part* a, const Part* b)
					{
						if (a == &this->ancestor<Model>()->ground()) return true; // 地面最优先
						if (b == &this->ancestor<Model>()->ground()) return false; // 地面最优先
						if (i == 0)return a->id() < b->id();// 第一轮先找地面或其他地面，防止下面的索引i-1出错
						if (b == rel_vec[i - 1].prtI_) return false;
						if (b == rel_vec[i - 1].prtJ_) return false;
						if (a == rel_vec[i - 1].prtI_) return true;
						if (a == rel_vec[i - 1].prtJ_) return true;
						return a->id() < b->id();
					});
					// 再插入连接新part的relation
					std::sort(rel_vec.begin() + i, rel_vec.end(), [i, this, &prt_vec](Relation a, Relation b)
					{
						auto pend = prt_vec.begin() + i + 1;
						auto a_part_i = std::find_if(prt_vec.begin(), pend, [a](const Part* p)->bool { return p == a.prtI_; });
						auto a_part_j = std::find_if(prt_vec.begin(), pend, [a](const Part* p)->bool { return p == a.prtJ_; });
						auto b_part_i = std::find_if(prt_vec.begin(), pend, [b](const Part* p)->bool { return p == b.prtI_; });
						auto b_part_j = std::find_if(prt_vec.begin(), pend, [b](const Part* p)->bool { return p == b.prtJ_; });

						bool a_is_ok = (a_part_i == pend) != (a_part_j == pend);
						bool b_is_ok = (b_part_i == pend) != (b_part_j == pend);

						if (a_is_ok && !b_is_ok) return true;
						else if (!a_is_ok && b_is_ok) return false;
						else if (a.size_ != b.size_)return a.size_ > b.size_;
						else if (a.dim_ != b.dim_)return a.dim_ > b.dim_;
						else return false;
					});
				}
			}
		}

		// 构建公共变量区 //
		PublicData pub_data;
		Imp::allocMem(mem_pool_size, imp_->pd_, 1);

		// 构建子系统 //
		std::vector<SubSystem> sys_vec;
		std::vector<std::vector<Diag>> d_vec_vec;
		std::vector<std::vector<LocalRemainder>> r_vec_vec;
		Size max_F_size{ 0 }, max_fm{ 0 }, max_fn{ 0 }, max_G_size{ 0 }, max_gm{ 0 }, max_gn{ 0 }, max_cm_size{ 0 };
		for (int i = 0; i < prt_vec_vec.size(); ++i)
		{
			auto &prt_vec = prt_vec_vec[i];
			auto &rel_vec = rel_vec_vec[i];

			// 插入SubSystem //
			sys_vec.push_back(SubSystem());
			auto &sys = sys_vec.back();
			sys.max_error_ = maxError();
			sys.has_ground_ = (prt_vec.front() == &ancestor<Model>()->ground());

			// 制造 d_vec (diag_vec) //
			Imp::allocMem(mem_pool_size, sys.d_data_, prt_vec.size());
			d_vec_vec.push_back(std::vector<Diag>());
			auto &d_vec = d_vec_vec.back();
			d_vec.resize(prt_vec.size());
			d_vec[0].part_ = prt_vec[0];
			for (Size i = 1; i < d_vec.size(); ++i)
			{
				auto &diag = d_vec[i];
				auto &rel = rel_vec[i - 1];

				// 根据diag更改是否为I part
				if (rel.prtI_ != prt_vec.at(i))
				{
					std::swap(rel.prtI_, rel.prtJ_);
					for (auto &c : rel.cst_pool_)c.is_I_ = !c.is_I_;
				}
				diag.part_ = prt_vec[i];

				// 分配 Relation::Block 内存
				Imp::allocMem(mem_pool_size, rel.blk_data_, rel.cst_pool_.size());

				// 分配 Diag中 p bc xc 的尺寸
				Imp::allocMem(mem_pool_size, d_vec[i].p_, rel.size_);
				Imp::allocMem(mem_pool_size, d_vec[i].bc_, rel.size_);
				Imp::allocMem(mem_pool_size, d_vec[i].xc_, rel.size_);

				// 计算 max_cm 的尺寸
				max_cm_size = std::max(max_cm_size, rel.size_);

				// 以下优化dm矩阵的计算，因为优化会改变系统所需内存的计算，因此必须放到这里 //
				{
					// 针对约束仅仅有一个时的优化 //
					if (rel.cst_pool_.size() == 1)
					{
						diag.upd_d_and_cp_ = Imp::one_constraint_upd_d_and_cp;
					}
					// 针对转动副加转动电机 //
					else if (rel.cst_pool_.size() == 2
						&& dynamic_cast<const RevoluteJoint*>(rel.cst_pool_.at(0).cst_)
						&& dynamic_cast<const Motion*>(rel.cst_pool_.at(1).cst_)
						&& dynamic_cast<const Motion*>(rel.cst_pool_.at(1).cst_)->axis() == 5
						&& &rel.cst_pool_.at(0).cst_->makI() == &rel.cst_pool_.at(1).cst_->makI())
					{
						diag.upd_d_and_cp_ = Imp::revolute_upd_d_and_cp;
						rel.dim_ = 6;
					}
					// 针对移动副加移动电机 //
					else if (rel.cst_pool_.size() == 2
						&& dynamic_cast<const PrismaticJoint*>(rel.cst_pool_.at(0).cst_)
						&& dynamic_cast<const Motion*>(rel.cst_pool_.at(1).cst_)
						&& dynamic_cast<const Motion*>(rel.cst_pool_.at(1).cst_)->axis() == 2
						&& &rel.cst_pool_.at(0).cst_->makI() == &rel.cst_pool_.at(1).cst_->makI())
					{
						diag.upd_d_and_cp_ = Imp::prismatic_upd_d_and_cp;
						rel.dim_ = 6;
					}
					// 不优化 //
					else
					{
						diag.upd_d_and_cp_ = Imp::normal_upd_d_and_cp;
					}
				}
			}

			// 制造 r_vec (remainder_vec) //
			Imp::allocMem(mem_pool_size, sys.r_data_, rel_vec.size() - prt_vec.size() + 1);
			r_vec_vec.push_back(std::vector<LocalRemainder>());
			auto &r_vec = r_vec_vec.back();
			r_vec.clear();
			r_vec.resize(rel_vec.size() - prt_vec.size() + 1);
			for (Size i = 0; i < r_vec.size(); ++i)
			{
				auto &r = r_vec.at(i);
				auto &rel = rel_vec.at(i + d_vec.size() - 1);

				r.cm_blk_series.clear();
				r.cm_blk_series.push_back(Remainder::Block());
				r.cm_blk_series.back().diag_ = &*std::find_if(d_vec.begin(), d_vec.end(), [&rel](Diag& d) {return rel.prtI_ == d.part_; });
				r.cm_blk_series.back().is_I_ = true;
				r.cm_blk_series.push_back(Remainder::Block());
				r.cm_blk_series.back().diag_ = &*std::find_if(d_vec.begin(), d_vec.end(), [&rel](Diag& d) {return rel.prtJ_ == d.part_; });
				r.cm_blk_series.back().is_I_ = false;

				for (auto rd = d_vec.rbegin(); rd < d_vec.rend() - 1; ++rd)
				{
					auto &d = *rd;
					auto &d_rel = rel_vec.at(d_vec.rend() - rd - 2);

					auto diag_part = d_rel.prtI_;
					auto add_part = d_rel.prtJ_;

					// 判断当前remainder加法元素是否存在（不为0）
					auto diag_blk = std::find_if(r.cm_blk_series.begin(), r.cm_blk_series.end(), [&](Remainder::Block &blk) {return blk.diag_->part_ == diag_part; });
					auto add_blk = std::find_if(r.cm_blk_series.begin(), r.cm_blk_series.end(), [&](Remainder::Block &blk) {return blk.diag_->part_ == add_part; });
					if (diag_blk != r.cm_blk_series.end())
					{
						if (add_blk != r.cm_blk_series.end())
						{
							r.cm_blk_series.erase(add_blk);
						}
						else
						{
							Remainder::Block blk;
							blk.is_I_ = diag_blk->is_I_;
							blk.diag_ = &*std::find_if(d_vec.begin(), d_vec.end(), [&](Diag &d) 
							{
								return d.part_ == add_part; 
							});
							r.cm_blk_series.push_back(blk);
						}
					}
				}

				// 分配 Relation::Block 内存
				Imp::allocMem(mem_pool_size, rel.blk_data_, rel.cst_pool_.size());

				// 分配 Remainder 中 cmI_vec, cmJ_vec, bc_vec, xc_vec 的尺寸
				Imp::allocMem(mem_pool_size, r_vec[i].cmI_, 6 * rel.size_);
				Imp::allocMem(mem_pool_size, r_vec[i].cmJ_, 6 * rel.size_);
				Imp::allocMem(mem_pool_size, r_vec[i].bc_, rel.size_);
				Imp::allocMem(mem_pool_size, r_vec[i].xc_, rel.size_);

				// 分配 Remainder::Block 内存
				r_vec[i].blk_size_ = r.cm_blk_series.size();
				Imp::allocMem(mem_pool_size, r_vec[i].blk_data_, r.cm_blk_series.size());
			}
			
			// 更新子系统尺寸 //
			sys.fm_ = 0;
			sys.fn_ = 0;
			for (Size i = 1; i < d_vec.size(); ++i)sys.fm_ += rel_vec[i - 1].dim_;
			for (Size i = 0; i < r_vec.size(); ++i)sys.fn_ += rel_vec[i + d_vec.size() - 1].size_;

			sys.gm_ = sys.hasGround() ? sys.fm_ : sys.fm_ + 6;
			sys.gn_ = sys.hasGround() ? sys.fm_ : sys.fm_ + 6;

			max_F_size = std::max(max_F_size, sys.fm_ * sys.fn_);
			max_fm = std::max(max_fm, sys.fm_);
			max_fn = std::max(max_fn, sys.fn_);
			max_G_size = std::max(max_G_size, sys.gm_ * sys.gn_);
			max_gm = std::max(max_gm, sys.gm_);
			max_gn = std::max(max_gn, sys.gn_);
		}
		Imp::allocMem(mem_pool_size, pub_data.subsys_data_, sys_vec.size());

		//std::cout << "mem size 0:" << mem_pool_size << std::endl;

		// 计算公共的内存及偏移
		Imp::allocMem(mem_pool_size, pub_data.F_, max_F_size);
		Imp::allocMem(mem_pool_size, pub_data.FT_, std::max(max_fm, max_fn));
		Imp::allocMem(mem_pool_size, pub_data.FP_, std::max(max_fm, max_fn));
		Imp::allocMem(mem_pool_size, pub_data.G_, max_G_size);
		Imp::allocMem(mem_pool_size, pub_data.GT_, std::max(max_gm, max_gn));
		Imp::allocMem(mem_pool_size, pub_data.GP_, std::max(max_gm, max_gn));
		Imp::allocMem(mem_pool_size, pub_data.S_, max_fm * max_fm);
		Imp::allocMem(mem_pool_size, pub_data.beta_, max_gn);
		Imp::allocMem(mem_pool_size, pub_data.xcf_, std::max(max_fn, max_fm));
		Imp::allocMem(mem_pool_size, pub_data.xpf_, std::max(max_fn, max_fm));
		Imp::allocMem(mem_pool_size, pub_data.bcf_, max_fn);
		Imp::allocMem(mem_pool_size, pub_data.bpf_, max_fm);
		Imp::allocMem(mem_pool_size, pub_data.cmI_, max_cm_size * 6);
		Imp::allocMem(mem_pool_size, pub_data.cmJ_, max_cm_size * 6);
		Imp::allocMem(mem_pool_size, pub_data.cmU_, max_cm_size * 6);
		Imp::allocMem(mem_pool_size, pub_data.cmT_, std::max(Size(6), max_cm_size));
		Imp::allocMem(mem_pool_size, pub_data.Jg_, ancestor<Model>()->partPool().size() * 6 * (ancestor<Model>()->motionPool().size() + ancestor<Model>()->generalMotionPool().size() * 6));
		Imp::allocMem(mem_pool_size, pub_data.cg_, ancestor<Model>()->partPool().size() * 6);
		Imp::allocMem(mem_pool_size, pub_data.M_, (ancestor<Model>()->motionPool().size() + ancestor<Model>()->generalMotionPool().size() * 6) * (ancestor<Model>()->motionPool().size() + ancestor<Model>()->generalMotionPool().size() * 6));
		Imp::allocMem(mem_pool_size, pub_data.h_, (ancestor<Model>()->motionPool().size() + ancestor<Model>()->generalMotionPool().size() * 6));
		Imp::allocMem(mem_pool_size, pub_data.get_diag_from_part_id_, ancestor<Model>()->partPool().size());

		// 分配内存
		imp_->mem_pool_.resize(mem_pool_size);

		//std::cout <<"mem size e:"<< mem_pool_size << std::endl;

		// 更新公共变量区 //
		{
			imp_->pd_ = Imp::getMem(imp_->mem_pool_.data(), imp_->pd_);
			*imp_->pd_ = pub_data;

			auto pd = imp_->pd_;

			// 获得雅可比部分的内存 //
			imp_->pd_->Jg_ = Imp::getMem(imp_->mem_pool_.data(), imp_->pd_->Jg_);
			imp_->pd_->cg_ = Imp::getMem(imp_->mem_pool_.data(), imp_->pd_->cg_);
			imp_->pd_->M_ = Imp::getMem(imp_->mem_pool_.data(), imp_->pd_->M_);
			imp_->pd_->h_ = Imp::getMem(imp_->mem_pool_.data(), imp_->pd_->h_);

			imp_->pd_->F_ = Imp::getMem(imp_->mem_pool_.data(), imp_->pd_->F_);
			imp_->pd_->FU_ = imp_->pd_->F_;
			imp_->pd_->FT_ = Imp::getMem(imp_->mem_pool_.data(), imp_->pd_->FT_);
			imp_->pd_->FP_ = Imp::getMem(imp_->mem_pool_.data(), imp_->pd_->FP_);
			imp_->pd_->G_ = Imp::getMem(imp_->mem_pool_.data(), imp_->pd_->G_);
			imp_->pd_->GU_ = imp_->pd_->G_;
			imp_->pd_->GT_ = Imp::getMem(imp_->mem_pool_.data(), imp_->pd_->GT_);
			imp_->pd_->GP_ = Imp::getMem(imp_->mem_pool_.data(), imp_->pd_->GP_);
			imp_->pd_->QT_DOT_G_ = imp_->pd_->G_;
			imp_->pd_->S_ = Imp::getMem(imp_->mem_pool_.data(), imp_->pd_->S_);
			imp_->pd_->beta_ = Imp::getMem(imp_->mem_pool_.data(), imp_->pd_->beta_);
			imp_->pd_->xcf_ = Imp::getMem(imp_->mem_pool_.data(), imp_->pd_->xcf_);
			imp_->pd_->xpf_ = Imp::getMem(imp_->mem_pool_.data(), imp_->pd_->xpf_);
			imp_->pd_->bcf_ = Imp::getMem(imp_->mem_pool_.data(), imp_->pd_->bcf_);
			imp_->pd_->bpf_ = Imp::getMem(imp_->mem_pool_.data(), imp_->pd_->bpf_);
			imp_->pd_->cmI_ = Imp::getMem(imp_->mem_pool_.data(), imp_->pd_->cmI_);
			imp_->pd_->cmJ_ = Imp::getMem(imp_->mem_pool_.data(), imp_->pd_->cmJ_);
			imp_->pd_->cmU_ = Imp::getMem(imp_->mem_pool_.data(), imp_->pd_->cmU_);
			imp_->pd_->cmT_ = Imp::getMem(imp_->mem_pool_.data(), imp_->pd_->cmT_);
		}

		// 将内存付给子系统，并初始化 //
		for (int i = 0; i < sys_vec.size(); ++i)
		{
			auto &sys = sys_vec[i];
			auto &prt_vec = prt_vec_vec[i];
			auto &rel_vec = rel_vec_vec[i];
			auto &d_vec = d_vec_vec[i];
			auto &r_vec = r_vec_vec[i];

			sys.pd_ = imp_->pd_;
			sys.d_data_ = Imp::getMem(imp_->mem_pool_.data(), sys.d_data_);
			sys.d_size_ = d_vec.size();
			for (int i = 0; i < sys.d_size_; ++i)sys.d_data_[i] = d_vec[i];
			sys.d_data_[0].part_ = prt_vec.at(0);
			sys.d_data_[0].pm_ = sys.d_data_[0].pm1_;
			sys.d_data_[0].last_pm_ = sys.d_data_[0].pm2_;
			for (Size i = 1; i < sys.d_size_; ++i)
			{
				auto &diag = sys.d_data_[i];
				auto &rel = rel_vec.at(i - 1);

				// 获取 Block::Relation 内存 //
				{
					rel.blk_data_ = Imp::getMem(imp_->mem_pool_.data(), rel.blk_data_);
					rel.blk_size_ = rel.cst_pool_.size();
					std::copy(rel.cst_pool_.data(), rel.cst_pool_.data() + rel.cst_pool_.size(), rel.blk_data_);
					diag.rel_ = rel;
				}
				
				// 获取 p_vec 内存 //
				diag.p_ = Imp::getMem(imp_->mem_pool_.data(), diag.p_);
				diag.bc_ = Imp::getMem(imp_->mem_pool_.data(), diag.bc_);
				diag.xc_ = Imp::getMem(imp_->mem_pool_.data(), diag.xc_);
				std::iota(diag.p_, diag.p_ + rel.size_, 0);

				// 初始化 diag //
				diag.rd_ = std::find_if(sys.d_data_, sys.d_data_ + sys.d_size_, [&](Diag &d) {return d.part_ == rel.prtJ_; });
				diag.pm_ = diag.pm1_;
				diag.last_pm_ = diag.pm2_;
			}

			sys.r_data_ = reinterpret_cast<Remainder*>(imp_->mem_pool_.data() + *reinterpret_cast<Size*>(&sys.r_data_));
			sys.r_size_ = r_vec.size();
			for(int i = 0; i < sys.r_size_; ++i)sys.r_data_[i] = r_vec[i];
			for (Size i = 0; i < sys.r_size_; ++i)
			{
				auto &r = sys.r_data_[i];
				auto &rel = rel_vec[i + sys.d_size_ - 1];

				// 获取 Relation::Block 内存
				{
					rel.blk_data_ = Imp::getMem(imp_->mem_pool_.data(), rel.blk_data_);
					rel.blk_size_ = rel.cst_pool_.size();
					std::copy(rel.cst_pool_.data(), rel.cst_pool_.data() + rel.cst_pool_.size(), rel.blk_data_);
					r.rel_ = rel;
				}

				// 分配 Remainder 中 cmI_, cmJ_, bc_, xc_ 的尺寸
				r.cmI_ = Imp::getMem(imp_->mem_pool_.data(), r.cmI_);
				r.cmJ_ = Imp::getMem(imp_->mem_pool_.data(), r.cmJ_);
				r.bc_ = Imp::getMem(imp_->mem_pool_.data(), r.bc_);
				r.xc_ = Imp::getMem(imp_->mem_pool_.data(), r.xc_);

				// 获取 Remainder::Block 内存
				r.blk_data_ = Imp::getMem(imp_->mem_pool_.data(), r.blk_data_);
				for (int j = 0; j < r.blk_size_; ++j)
				{
					r.blk_data_[j] = r_vec[i].cm_blk_series[j];
					r.blk_data_[j].diag_ = sys.d_data_ + (r.blk_data_[j].diag_ - d_vec.data());
				}

				// 构建 r
				r.i_diag_ = std::find_if(sys.d_data_, sys.d_data_ + sys.d_size_, [&rel](Diag& d) {return rel.prtI_ == d.part_; });
				r.j_diag_ = std::find_if(sys.d_data_, sys.d_data_ + sys.d_size_, [&rel](Diag& d) {return rel.prtJ_ == d.part_; });
			}

			sys.has_ground_ = sys.d_data_->part_ == &ancestor<Model>()->ground();

			for (auto d = sys.d_data_; d < sys.d_data_ + sys.d_size_; ++d)
			{
				d->cmI_ = imp_->pd_->cmI_;
				d->cmJ_ = imp_->pd_->cmJ_;
				d->cmU_ = imp_->pd_->cmU_;
				d->cmT_ = imp_->pd_->cmT_;
			}
		}

		// 分配根据part id寻找diag的vector //
		imp_->pd_->get_diag_from_part_id_ = Imp::getMem(imp_->mem_pool_.data(), imp_->pd_->get_diag_from_part_id_);
		for (auto &sys : sys_vec)
			for (auto diag = sys.d_data_; diag < sys.d_data_ + sys.d_size_; ++diag)
				imp_->pd_->get_diag_from_part_id_[diag->part_->id()] = diag;

		imp_->pd_->subsys_size_ = sys_vec.size();
		imp_->pd_->subsys_data_ = Imp::getMem(imp_->mem_pool_.data(), imp_->pd_->subsys_data_);
		std::copy_n(sys_vec.data(), sys_vec.size(), imp_->pd_->subsys_data_);
	}
	auto UniversalSolver::kinPos()->int
	{
		double pm[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
		s_mc(4, 4, pm, const_cast<double *>(*ancestor<Model>()->ground().pm()));

		// 将杆件位姿拷贝到局部变量中 //
		ARIS_LOOP_SYS ARIS_LOOP_SYS_D d->part_->getPm(d->pm_);

		setError(0.0);
		setIterCount(0);

		ARIS_LOOP_SYS
		{
			sys->max_error_ = maxError();
			sys->max_iter_count_ = maxIterCount();
			sys->kinPos();

			setIterCount(std::max(iterCount(), sys->iter_count_));
			setError(std::max(error(), sys->error_));
		}

		// 迭代成功，设置各杆件 //
		if (error() < maxError())ARIS_LOOP_SYS ARIS_LOOP_SYS_D const_cast<Part*>(d->part_)->setPm(d->pm_);
		return error() < maxError() ? 0 : -1;
	}
	auto UniversalSolver::kinVel()->void
	{
		ARIS_LOOP_SYS ARIS_LOOP_SYS_D d->part_->getPm(d->pm_);

		s_fill(6, 1, 0.0, const_cast<double *>(ancestor<Model>()->ground().vs()));
		ARIS_LOOP_SYS sys->kinVel();

		// 计算成功，设置各杆件 //
		ARIS_LOOP_SYS ARIS_LOOP_SYS_D s_va(6, d->xp_, const_cast<double*>(d->part_->vs()));
	}
	auto UniversalSolver::dynAccAndFce()->void
	{
		// 更新杆件位姿，每个杆件外力 //
		ARIS_LOOP_SYS ARIS_LOOP_SYS_D
		{
			d->part_->getPm(d->pm_);
			std::fill(d->bp_, d->bp_ + 6, 0.0);
		}

		// 更新外力 //
		for (auto &fce : ancestor<Model>()->forcePool())
		{
			if (fce.active())
			{
				double fsI[6], fsJ[6];
				fce.cptGlbFs(fsI, fsJ);

				s_vs(6, fsI, imp_->pd_->get_diag_from_part_id_[fce.makI().fatherPart().id()]->bp_);
				s_vs(6, fsJ, imp_->pd_->get_diag_from_part_id_[fce.makJ().fatherPart().id()]->bp_);
			}
		}

		// 更新地面的as //
		s_fill(6, 1, 0.0, const_cast<double *>(ancestor<Model>()->ground().as()));
		ARIS_LOOP_SYS sys->dynAccAndFce();

		// 计算成功，设置各关节和杆件
		ARIS_LOOP_SYS
		{
			ARIS_LOOP_SYS_R
			{
				Size pos{ 0 };
				ARIS_LOOP_BLOCK(r->rel_.)
				{
					const_cast<Constraint*>(b->cst_)->setCf(r->xc_ + pos);
					pos += b->cst_->dim();
				}
			}
			for (auto d = sys->d_data_ + 1; d<sys->d_data_ + sys->d_size_; ++d)
			{
				Size pos{ 0 };
				ARIS_LOOP_BLOCK(d->rel_.)
				{
					const_cast<Constraint*>(b->cst_)->setCf(d->xc_ + pos);
					pos += b->cst_->dim();
				}
			}

			ARIS_LOOP_SYS_D s_vc(6, d->xp_, const_cast<double*>(d->part_->as()));
		}
	}
	auto UniversalSolver::cptGeneralJacobi()noexcept->void
	{
		auto Jg = imp_->pd_->Jg_;
		auto cg = imp_->pd_->cg_;
		
		std::fill(Jg, Jg + mJg() * nJg(), 0.0);
		std::fill(cg, cg + mJg(), 0.0);

		ARIS_LOOP_SYS
		{
			ARIS_LOOP_SYS_D d->part_->getPm(d->pm_);

			// make A
			sys->updDmCm(false);

			// solve
			sys->updF();
			ARIS_LOOP_SYS_D std::fill(d->bc_, d->bc_ + d->rel_.size_, 0.0);
			ARIS_LOOP_SYS_R std::fill(r->bc_, r->bc_ + r->rel_.size_, 0.0);

			// upd Jg //
			auto getJacobiColumn = [&](Relation &rel, double *bc)
			{
				Size pos = 0;
				ARIS_LOOP_BLOCK(rel.)
				{
					if (auto mot = dynamic_cast<const Motion*>(b->cst_))
					{
						// 更新bc，将当前电机的未知量更新为当前c的1.0 //
						bc[pos] = 1.0;
						sys->sovXp();
						ARIS_LOOP_SYS_D s_vc(6, d->xp_, 1, Jg + at(d->part_->id() * 6, mot->id(), nJg()), nJg());
						bc[pos] = 0.0;
					}
					else if (auto gmt = dynamic_cast<const GeneralMotion*>(b->cst_))
					{
						double tmf[6][6];
						s_tmf(*gmt->mpm(), *tmf);

						for (Size k(-1); ++k < 6;)
						{
							// 更新bc，将当前电机的未知量更新为当前c的1.0 //
							// Tmf^(T) * v //
							s_vc(6, tmf[k], bc);
							sys->sovXp();
							ARIS_LOOP_SYS_D s_vc(6, d->xp_, 1, Jg + at(d->part_->id() * 6, this->ancestor<Model>()->motionPool().size() + gmt->id() * 6 + k, nJg()), nJg());
						}

						std::fill(bc, bc + 6, 0.0);
					}

					pos += b->cst_->dim();
				}
			};
			ARIS_LOOP_SYS_D getJacobiColumn(d->rel_, d->bc_);
			ARIS_LOOP_SYS_R getJacobiColumn(r->rel_, r->bc_);

			// upd cg //
			sys->updCa();
			auto clearMotionMa = [&](Relation &rel, double *bc)
			{
				Size pos = 0;
				ARIS_LOOP_BLOCK(rel.)
				{
					if (auto mot = dynamic_cast<const Motion*>(b->cst_))
					{
						bc[pos] -= mot->ma();
					}
					else if (auto gm = dynamic_cast<const GeneralMotion*>(b->cst_))
					{
						s_inv_tva(-1.0, *gm->mpm(), gm->mas(), bc);
					}

					pos += b->cst_->dim();
				}
			};
			ARIS_LOOP_SYS_D clearMotionMa(d->rel_, d->bc_);
			ARIS_LOOP_SYS_R clearMotionMa(r->rel_, r->bc_);
			sys->sovXp();
			ARIS_LOOP_SYS_D s_vc(6, d->xp_, cg + d->part_->id() * 6);
		}
	}
	auto UniversalSolver::mJg()const noexcept->Size { return ancestor<Model>()->partPool().size() * 6; }
	auto UniversalSolver::nJg()const noexcept->Size { return ancestor<Model>()->motionPool().size() + ancestor<Model>()->generalMotionPool().size() * 6; }
	auto UniversalSolver::Jg()const noexcept->const double * { return imp_->pd_->Jg_; }
	auto UniversalSolver::cg()const noexcept->const double * { return imp_->pd_->cg_; }
	auto UniversalSolver::cptGeneralInverseDynamicMatrix()noexcept->void
	{
		auto M = imp_->pd_->M_;
		auto h = imp_->pd_->h_;
		
		// init //
		std::fill(M, M + nM()* nM(), 0.0);
		std::fill(h, h + nM(), 0.0);

		ARIS_LOOP_SYS
		{
			ARIS_LOOP_SYS_D d->part_->getPm(d->pm_);
			
			// 动力学计算，和dynAccAndFce() 一模一样 //
			sys->updDiagIv();
			sys->updDmCm(false);

			sys->updF();
			sys->updG();
			sys->updCa();

			auto dynamic = [&]()
			{
				ARIS_LOOP_SYS ARIS_LOOP_SYS_D std::fill(d->bp_, d->bp_ + 6, 0.0);
				for (auto &fce : this->ancestor<Model>()->forcePool())
				{
					if (fce.active())
					{
						double fsI[6], fsJ[6];
						fce.cptGlbFs(fsI, fsJ);
						s_vs(6, fsI, imp_->pd_->get_diag_from_part_id_[fce.makI().fatherPart().id()]->bp_);
						s_vs(6, fsJ, imp_->pd_->get_diag_from_part_id_[fce.makJ().fatherPart().id()]->bp_);
					}
				}
				
				sys->sovXp();
				sys->sovXc();
			};

			// 开始计算h //
			// 先去掉驱动的加速度, 并计算h
			auto clearMotionMa = [&](Relation &rel, double *bc)
			{
				Size pos = 0;
				ARIS_LOOP_BLOCK(rel.)
				{
					if (auto mot = dynamic_cast<const Motion*>(b->cst_))
					{
						bc[pos] -= mot->ma();
					}
					else if (auto gm = dynamic_cast<const GeneralMotion*>(b->cst_))
					{
						s_inv_tva(-1.0, *gm->mpm(), gm->mas(), bc);
					}

					pos += b->cst_->dim();
				}
			};
			ARIS_LOOP_SYS_D clearMotionMa(d->rel_, d->bc_);
			ARIS_LOOP_SYS_R clearMotionMa(r->rel_, r->bc_);

			// 动力学计算并取出h
			dynamic();
			auto getH = [&](Relation &rel, double *xc)
			{
				Size pos{ 0 };
				// 将Xcf更新 //
				ARIS_LOOP_BLOCK(rel.)
				{
					if (auto mot = dynamic_cast<const Motion*>(b->cst_))
					{
						h[mot->id()] = xc[pos];
					}
					if (dynamic_cast<const GeneralMotion*>(b->cst_))
					{
						s_vc(6, xc + pos, h + this->ancestor<Model>()->motionPool().size() + b->cst_->id() * 6);
					}
					pos += b->cst_->dim();
				}
			};
			ARIS_LOOP_SYS_D getH(d->rel_, d->xc_);
			ARIS_LOOP_SYS_R getH(r->rel_, r->xc_);
			
			// 开始计算M //
			auto getMColumn = [&](const Constraint *c, Size cid)
			{
				auto Mn = this->ancestor<Model>()->motionPool().size() + this->ancestor<Model>()->generalMotionPool().size() * 6;
				auto getMRow = [&](Relation &rel, double *xc)
				{
					Size pos2{ 0 };
					ARIS_LOOP_BLOCK(rel.)
					{
						if (dynamic_cast<const Motion*>(b->cst_))
						{
							Size ccid = b->cst_->id();
							M[at(ccid, cid, Mn)] = xc[pos2] - h[ccid];
						}
						if (dynamic_cast<const GeneralMotion*>(b->cst_))
						{
							Size ccid = b->cst_->id() * 6 + this->ancestor<Model>()->motionPool().size();
							for (Size i = 0; i < 6; ++i)
							{
								s_vc(6, xc, 1, M + at(ccid, cid, Mn), Mn);
								s_vs(6, h + ccid, 1, M + at(ccid, cid, Mn), Mn);
							}

						}
						pos2 += b->cst_->dim();
					}
				};
				ARIS_LOOP_SYS_R getMRow(r->rel_, r->xc_);
				ARIS_LOOP_SYS_D getMRow(d->rel_, d->xc_);
			};
			auto getM = [&](Relation &rel, double *bc)
			{
				Size pos{ 0 };
				ARIS_LOOP_BLOCK(rel.)
				{
					if (dynamic_cast<const Motion*>(b->cst_))
					{
						bc[pos] += 1.0;

						dynamic();

						getMColumn(b->cst_, b->cst_->id());

						bc[pos] -= 1.0;
					}
					else if (dynamic_cast<const GeneralMotion*>(b->cst_))
					{
						double tmf[6][6];
						s_tmf(*dynamic_cast<const GeneralMotion*>(b->cst_)->mpm(), *tmf);
						for (Size i = 0; i < 6; ++i)
						{
							s_va(6, tmf[i], bc);

							dynamic();

							getMColumn(b->cst_, b->cst_->id() * 6 + this->ancestor<Model>()->motionPool().size() + i);

							s_vs(6, tmf[i], bc);
						}
					}
					pos += b->cst_->dim();
				}
			};
			ARIS_LOOP_SYS_D getM(d->rel_, d->bc_);
			ARIS_LOOP_SYS_R getM(r->rel_, r->bc_);
		}
	}
	auto UniversalSolver::nM()const noexcept->Size { return ancestor<Model>()->motionPool().size() + ancestor<Model>()->generalMotionPool().size() * 6; }
	auto UniversalSolver::M()const noexcept->const double * { return imp_->pd_->M_; }
	auto UniversalSolver::h()const noexcept->const double * { return imp_->pd_->h_; }
	UniversalSolver::~UniversalSolver() = default;
	UniversalSolver::UniversalSolver(const std::string &name, Size max_iter_count, double max_error) :Solver(name, max_iter_count, max_error) {}
	ARIS_DEFINE_BIG_FOUR_CPP(UniversalSolver);
#undef ARIS_LOOP_SYS
#undef ARIS_LOOP_SYS_D
#undef ARIS_LOOP_SYS_R
#undef ARIS_LOOP_BLOCK
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
	auto ForwardKinematicSolver::kinPos()->int
	{
		UniversalSolver::kinPos();
		if (error() < maxError())for (auto &m : ancestor<Model>()->generalMotionPool())m.updMpm();
		return error() < maxError() ? 0 : -1;
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
	auto ForwardKinematicSolver::cptJacobiWrtEE()noexcept->void
	{
		cptGeneralJacobi();

		for (auto &gm : ancestor<Model>()->generalMotionPool())
		{
			for (auto &mot : ancestor<Model>()->motionPool())
			{
				double tem[6];
				s_vc(6, Jg() + at(gm.makI().fatherPart().id() * 6, mot.id(), nJg()), nJg(), tem, 1);
				s_vs(6, Jg() + at(gm.makJ().fatherPart().id() * 6, mot.id(), nJg()), nJg(), tem, 1);

				s_inv_tv(*gm.makJ().pm(), tem, 1, imp_->J_vec_.data() + at(gm.id() * 6, mot.id(), nJf()), nJf());

				double pp[3];
				gm.makI().getPp(gm.makJ(), pp);
				s_c3a(imp_->J_vec_.data() + at(gm.id() * 6 + 3, mot.id(), nJf()), nJf(), pp, 1, imp_->J_vec_.data() + at(gm.id() * 6, mot.id(), nJf()), nJf());
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
	auto InverseKinematicSolver::kinPos()->int
	{
		UniversalSolver::kinPos();
		if (error() < maxError())for (auto &m : ancestor<Model>()->motionPool())m.updMp();
		return error() < maxError() ? 0 : -1;
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
	auto ForwardDynamicSolver::kinPos()->int
	{
		UniversalSolver::kinPos();
		if (error() < maxError())for (auto &m : ancestor<Model>()->generalMotionPool())m.updMpm();
		return error() < maxError() ? 0 : -1;
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
	auto InverseDynamicSolver::kinPos()->int
	{
		UniversalSolver::kinPos();
		if (error() < maxError())for (auto &m : ancestor<Model>()->motionPool())m.updMp();
		return error() < maxError() ? 0 : -1;
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
