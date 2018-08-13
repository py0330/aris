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

#include "aris_dynamic_model.h"


namespace aris
{
	namespace dynamic
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
		Solver::Solver(const Solver&) = default;
		Solver::Solver(Solver&&) = default;
		Solver& Solver::operator=(const Solver&) = default;
		Solver& Solver::operator=(Solver&&) = default;

		struct Relation
		{
			struct Block { Constraint* constraint; bool is_I; };

			Part *prtI; // 对角块对应的Part
			Part *prtJ; // rd对应的Part
			Size dim;
			std::vector<Block> cst_pool_;
		};
		struct Diag
		{
			double dm[36], iv[10];
			double pm1[16], pm2[16];
			double *pm, *last_pm;
			double xp[6], bc[6], xc[6], bp[6];
			Size rows;// in F
			Part *part;
			Diag *rd;//related diag, for row addition
			Relation rel_;

			std::function<void(Diag*)> upd_d;
			std::function<void(Diag*, const double *left, double* right)> d_dot;
			std::function<void(Diag*, const double *left, double* right)> dt_dot;
		};
		struct Remainder
		{
			struct Block { Diag* diag; bool is_I; };

			Diag *i_diag, *j_diag;
			double cmI[36], cmJ[36];
			double xp[6], bc[6], xc[6], bp[6];
			std::vector<Block> cm_blk_series;
			Relation rel_;
		};
		struct SubSystem
		{
			std::vector<Diag> diag_pool_;
			std::vector<Remainder> remainder_pool_;

			Size fm, fn, fr;

			double *F, *FU, *FT;
			Size* FP;
			double *G, *GU, *GT;
			Size *GP;

			double *S;
			double *QT_DOT_G;

			double *xpf, *xcf;
			double *bpf, *bcf;
			double *alpha;

			bool has_ground_;

			double max_error_;
			double error_;
			Size max_iter_count_;
			Size iter_count_;

			static auto addPart(std::vector<Part *> &part_pool_, std::vector<Part *> &left_part_pool, std::vector<Relation> &relation_pool, Part *part)->void
			{
				if (std::find(part_pool_.begin(), part_pool_.end(), part) != part_pool_.end())return;

				part_pool_.push_back(part);

				// 如果不是地面 //
				if (part != left_part_pool.front())
				{
					left_part_pool.erase(std::find(left_part_pool.begin(), left_part_pool.end(), part));
					for (auto &rel : relation_pool)
					{
						if (rel.prtI == part)
						{
							addPart(part_pool_, left_part_pool, relation_pool, rel.prtJ);
						}
						else if (rel.prtJ == part)
						{
							addPart(part_pool_, left_part_pool, relation_pool, rel.prtI);
						}
					}
				}
			}

			auto hasGround()->bool { return has_ground_; }
			auto rowAddInverseXp()->void;
			auto rowAddBp()->void;
			auto updDiagDm()->void;
			auto updDiagIv()->void;
			auto updCpToBc()->void;
			auto updCvToBc()->void;
			auto updCaToBc()->void;
			auto updPfToBp()->void;
			auto updRemainderCm()->void;
			auto updF()->void;
			auto updXpf()->void;
			auto updBcf()->void;
			auto updXcf()->void;
			auto updBpf()->void;
			auto updXp()->void;
			auto updXc()->void;
			auto updCf()->void;
			auto updPp()->void;
			auto updPv()->void;
			auto updPa()->void;

			auto kinPos()->void;
			auto kinVel()->void;
			auto dynAccAndFce()->void;
		};
		auto SubSystem::rowAddInverseXp()->void { for (auto d = diag_pool_.begin() + 1; d<diag_pool_.end(); ++d)s_va(6, d->rd->xp, d->xp); }
		auto SubSystem::rowAddBp()->void { for (auto d = diag_pool_.rbegin(); d < diag_pool_.rend() - 1; ++d) s_va(6, d->bp, d->rd->bp); }
		auto SubSystem::updDiagDm()->void { for (auto d = diag_pool_.begin() + 1; d < diag_pool_.end(); ++d)d->upd_d(&*d); }
		auto SubSystem::updDiagIv()->void
		{
			if (!hasGround())s_iv2iv(*diag_pool_.begin()->part->pm(), diag_pool_.begin()->part->prtIv(), diag_pool_.begin()->iv);
			for (auto d = diag_pool_.begin() + 1; d < diag_pool_.end(); ++d)s_iv2iv(*d->part->pm(), d->part->prtIv(), d->iv);
		}
		auto SubSystem::updF()->void
		{
			s_fill(fm, fn, 0.0, F);

			Size cols{ 0 };
			for (auto &r : remainder_pool_)
			{
				for (auto &b : r.cm_blk_series)
				{
					s_mm(6 - b.diag->rel_.dim, r.rel_.dim, 6, b.diag->dm + at(b.diag->rel_.dim, 0, 6), 6, b.is_I ? r.cmI : r.cmJ, r.rel_.dim, F + at(b.diag->rows, cols, fn), fn);
				}
				cols += r.rel_.dim;
			}
		}
		auto SubSystem::updXpf()->void
		{
			s_householder_utp(fn, fm, F, ColMajor{ fn }, FU, ColMajor{ fn }, FT, 1, FP, fr, max_error_);
			s_householder_utp_sov(fn, fm, 1, fr, FU, ColMajor{ fn }, FT, 1, FP, bcf, 1, xpf, 1, max_error_);
		}
		auto SubSystem::updXcf()->void
		{
			s_householder_utp(fm, fn, F, FU, FT, FP, fr, max_error_);
			s_householder_utp_sov(fm, fn, 1, fr, FU, FT, FP, bpf, xcf, max_error_);
		}
		auto SubSystem::updBpf()->void
		{
			if (!hasGround())s_vc(6, diag_pool_.begin()->bp, bpf + diag_pool_.begin()->rows);
			for (auto d = diag_pool_.begin() + 1; d<diag_pool_.end(); ++d)
			{
				double tem[6];

				s_mm(6, 1, 6, d->dm, 6, d->bp, 1, tem, 1);

				s_vc(6, tem, d->bp);
				s_vc(6 - d->rel_.dim, d->bp + d->rel_.dim, bpf + d->rows);
			}
		}
		auto SubSystem::updBcf()->void
		{
			// 使用已求出的未知数，用以构建b
			Size cols{ 0 };
			for (auto &r : remainder_pool_)
			{
				double bb[6]{ 0,0,0,0,0,0 };

				s_vc(r.rel_.dim, r.bc, bcf + cols);
				for (auto &b : r.cm_blk_series)
				{
					double tem[6];
					auto cm = b.is_I ? r.cmJ : r.cmI;//这里是颠倒的，因为加到右侧需要乘-1.0

					s_mm(6, 1, b.diag->rel_.dim, b.diag->dm, ColMajor{ 6 }, b.diag->bc, 1, tem, 1);
					s_mma(r.rel_.dim, 1, 6, cm, ColMajor{ r.rel_.dim }, tem, 1, bcf + cols, 1);
				}
				cols += r.rel_.dim;
			}
		}
		auto SubSystem::updXp()->void
		{
			if (!hasGround())s_vc(6, xpf + diag_pool_.begin()->rows, diag_pool_.begin()->xp);
			for (auto d = diag_pool_.begin() + 1; d<diag_pool_.end(); ++d)
			{
				double tem[6];
				s_vc(d->rel_.dim, d->bc, tem);
				s_vc(6 - d->rel_.dim, xpf + d->rows, tem + d->rel_.dim);

				s_mm(6, 1, 6, d->dm, ColMajor{ 6 }, tem, 1, d->xp, 1);
			}
		}
		auto SubSystem::updXc()->void
		{
			// 将已经求出的x更新到remainder中，此后将已知数移到右侧
			Size cols{ 0 };
			for (auto &r : remainder_pool_)
			{
				s_vc(r.rel_.dim, xcf + cols, r.xc);

				// 更新待求
				for (auto &b : r.cm_blk_series)
				{
					double tem[6];
					s_mm(6, 1, r.rel_.dim, b.is_I ? r.cmJ : r.cmI, xcf + cols, tem);
					s_mma(6, 1, 6, b.diag->dm, 6, tem, 1, b.diag->bp, 1);
				}

				cols += r.rel_.dim;
			}
			for (auto d = diag_pool_.begin() + 1; d < diag_pool_.end(); ++d)
			{
				s_vc(d->rel_.dim, d->bp, d->xc);
			}
		}
		auto SubSystem::updCf()->void
		{
			for (auto &r : remainder_pool_)
			{
				Size pos{ 0 };
				// 将Xcf更新 //
				for (auto &c : r.rel_.cst_pool_)
				{
					c.constraint->setCf(r.xc + pos);
					pos += c.constraint->dim();
				}
			}
			for (auto d = diag_pool_.begin() + 1; d < diag_pool_.end(); ++d)
			{
				Size pos{ 0 };
				for (auto &c : d->rel_.cst_pool_)
				{
					c.constraint->setCf(d->xc + pos);
					pos += c.constraint->dim();
				}
			}
		}
		auto SubSystem::updPp()->void
		{
			auto beg = hasGround() ? diag_pool_.begin() + 1 : diag_pool_.begin();
			for (auto d = beg; d<diag_pool_.end(); ++d)
			{
				std::swap(d->pm, d->last_pm);

				double tem[16];
				s_ps2pm(d->xp, tem);
				s_pm2pm(tem, d->last_pm, d->pm);
			}
		}
		auto SubSystem::updPv()->void
		{
			auto beg = hasGround() ? diag_pool_.begin() + 1 : diag_pool_.begin();
			for (auto d = beg; d<diag_pool_.end(); ++d)s_va(6, d->xp, const_cast<double*>(d->part->vs()));
		}
		auto SubSystem::updPa()->void
		{
			auto beg = hasGround() ? diag_pool_.begin() + 1 : diag_pool_.begin();
			for (auto d = beg; d<diag_pool_.end(); ++d)s_vc(6, d->xp, const_cast<double*>(d->part->as()));
		}
		auto SubSystem::updCpToBc()->void
		{
			// bc in diag //
			for (auto d = diag_pool_.begin() + 1; d < diag_pool_.end(); ++d)
			{
				Size pos{ 0 };
				for (auto &c : d->rel_.cst_pool_)
				{
					double makI_pm[16], makJ_pm[16];
					s_pm_dot_pm(c.is_I ? d->pm : d->rd->pm, *c.constraint->makI().prtPm(), makI_pm);
					s_pm_dot_pm(c.is_I ? d->rd->pm : d->pm, *c.constraint->makJ().prtPm(), makJ_pm);
					c.constraint->cptCpFromPm(d->bc + pos, makI_pm, makJ_pm);
					pos += c.constraint->dim();
				}
			}
			// bc in remainder //
			for (auto &r : remainder_pool_)
			{
				Size pos{ 0 };
				for (auto &c : r.rel_.cst_pool_)
				{
					double makI_pm[16], makJ_pm[16];
					s_pm_dot_pm(c.is_I ? r.i_diag->pm : r.j_diag->pm, *c.constraint->makI().prtPm(), makI_pm);
					s_pm_dot_pm(c.is_I ? r.j_diag->pm : r.i_diag->pm, *c.constraint->makJ().prtPm(), makJ_pm);
					c.constraint->cptCpFromPm(r.bc + pos, makI_pm, makJ_pm);
					pos += c.constraint->dim();
				}
			}
		}
		auto SubSystem::updCvToBc()->void
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
		auto SubSystem::updCaToBc()->void
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
		auto SubSystem::updPfToBp()->void
		{
			auto beg = hasGround() ? diag_pool_.begin() + 1 : diag_pool_.begin();
			for (auto d = beg; d < diag_pool_.end(); ++d)
			{
				// 外力已经储存在了bp中

				// I*(a-g) //
				double as_minus_g[6], iv_dot_as[6];
				s_vc(6, d->xp, as_minus_g);
				s_vs(6, d->part->model().environment().gravity(), d->xp);
				s_iv_dot_as(d->iv, d->xp, iv_dot_as);
				s_va(6, iv_dot_as, d->bp);

				// v x I * v //
				double I_dot_v[6];
				s_iv_dot_as(d->iv, d->part->vs(), I_dot_v);
				s_cfa(d->part->vs(), I_dot_v, d->bp);
			}
		}
		auto SubSystem::updRemainderCm()->void
		{
			// upd remainder data //
			for (auto &r : remainder_pool_)
			{
				Size pos{ 0 };
				for (auto &c : r.rel_.cst_pool_)
				{
					double makI_pm[16], makJ_pm[16];
					s_pm_dot_pm(c.is_I ? r.i_diag->pm : r.j_diag->pm, *c.constraint->makI().prtPm(), makI_pm);
					s_pm_dot_pm(c.is_I ? r.j_diag->pm : r.i_diag->pm, *c.constraint->makJ().prtPm(), makJ_pm);

					double cmI[36], cmJ[36];
					c.constraint->cptGlbCmFromPm(cmI, cmJ, makI_pm, makJ_pm);
					s_mc(6, c.constraint->dim(), cmI, c.constraint->dim(), r.cmI + pos, r.rel_.dim);
					s_mc(6, c.constraint->dim(), cmJ, c.constraint->dim(), r.cmJ + pos, r.rel_.dim);
					pos += c.constraint->dim();
				}
			}
		}
		auto SubSystem::kinPos()->void
		{
			for (iter_count_ = 0; iter_count_ < max_iter_count_; ++iter_count_)
			{
				// make b
				updCpToBc();

				// 求解当前的误差
				error_ = 0.0;
				for (auto d = diag_pool_.begin() + 1; d < diag_pool_.end(); ++d)for (Size i{ 0 }; i < d->rel_.dim; ++i)error_ = std::max(error_, std::abs(d->bc[i]));
				for (auto &r : remainder_pool_)for (Size i{ 0 }; i < r.rel_.dim; ++i)error_ = std::max(error_, std::abs(r.bc[i]));
				if (error_ < max_error_) return;

				// make A
				updDiagDm();
				updRemainderCm();

				// solve
				updF();
				updBcf();
				updXpf();
				updXp();

				// 反向做行变换
				rowAddInverseXp();

				// 将x更新到杆件
				updPp();

				// 以下为了防止奇异造成解过大的问题 //
				// make b
				updCpToBc();

				// 求解当前的误差
				double last_error = error_;
				error_ = 0.0;
				for (auto d = diag_pool_.begin() + 1; d < diag_pool_.end(); ++d)for (Size i{ 0 }; i < d->rel_.dim; ++i)error_ = std::max(error_, std::abs(d->bc[i]));
				for (auto &r : remainder_pool_)for (Size i{ 0 }; i < r.rel_.dim; ++i)error_ = std::max(error_, std::abs(r.bc[i]));

				double coe = 1.0;
				if (error_ > last_error)// 这里如果用while可以确保每个循环误差都会减小，但是有可能出现卡死
				{
					coe *= last_error / (error_ + last_error);

					auto beg = hasGround() ? diag_pool_.begin() + 1 : diag_pool_.begin();
					for (auto d = beg; d<diag_pool_.end(); ++d)
					{
						s_nv(6, coe, d->xp);
						double pm[4][4];
						s_ps2pm(d->xp, *pm);
						double final_pm[4][4];
						s_pm2pm(*pm, d->last_pm, *final_pm);
						s_vc(16, *final_pm, d->pm);
					}

					updCpToBc();
					error_ = 0.0;
					for (auto d = diag_pool_.begin() + 1; d < diag_pool_.end(); ++d)for (Size i{ 0 }; i < d->rel_.dim; ++i)error_ = std::max(error_, std::abs(d->bc[i]));
					for (auto &r : remainder_pool_)for (Size i{ 0 }; i < r.rel_.dim; ++i)error_ = std::max(error_, std::abs(r.bc[i]));
				}
				/////////////////////////////////结束///////////////////////////////
			}
		}
		auto SubSystem::kinVel()->void
		{
			// make b
			updCvToBc();

			// make A
			updDiagDm();
			updRemainderCm();

			// solve
			updF();
			updBcf();
			updXpf();
			updXp();

			// 做行加变换
			rowAddInverseXp();

			// 将x更新到杆件
			updPv();
		}
		auto SubSystem::dynAccAndFce()->void
		{
			// [I   C] * [ pa ] = [ pf ]
			// [C^T  ]   [ cf ]   [ ca ]

			// upd Im //
			updDiagIv();

			// make b
			updCaToBc();

			// make A
			updDiagDm();
			updRemainderCm();

			// make and solve remainder equation
			updF();
			updBcf();


			//////////////////////////////////////////// 求CT * xp = bc 的通解和特解 //////////////////////////////
			s_householder_utp(fm, fn, F, FU, FT, FP, fr, max_error_);

			Size m_mimus_r = fm - fr;

			s_fill(fm, 1, 0.0, xpf);

			//// 这里更新G ////
			for (Size j(-1); ++j < m_mimus_r;)
			{
				//// 求通解 ////
				// F * P = Q * R
				// 这里R为：
				//     1   r   n
				// 1 [ * * * * * ]
				//   |   * * * * |
				// r |     * * * |
				//   |           |
				//   |           |
				// m [           ]
				//
				// 这里求解：
				// F^T * x = b
				// 即：
				// P^-T * P^T * F^T * x = b
				// P^-T * R^T * Q*T * x = b
				// 这里的R^T为：
				//     1   r   m
				// 1 [ *         ]
				//   | * *       |
				// r | * * *     |
				//   | * * *     |
				//   | * * *     |
				// n [ * * *     ]
				// 于是Q*T * x 的通解为：
				//     1  m-r 
				// 1 [       ]
				//   |       |
				// r |       |
				//   | 1     |
				//   |   1   |
				// m [     1 ]
				// 于是x的通解为以上左乘Q
				// 
				// 求通解
				xpf[fr + j] = 1.0;
				s_householder_ut_q_dot(fm, fn, 1, FU, fn, FT, 1, xpf, 1, S + j, m_mimus_r);
				xpf[fr + j] = 0.0;

				// 以下求G
				// 乘以DT
				if (!hasGround())s_vc(6, S + at(diag_pool_.begin()->rows, j, m_mimus_r), m_mimus_r, diag_pool_.begin()->xp, 1);
				for (auto d = diag_pool_.begin() + 1; d<diag_pool_.end(); ++d)
				{
					s_mm(6, 1, 6 - d->rel_.dim, d->dm + at(0, d->rel_.dim, ColMajor{ 6 }), ColMajor{ 6 }, S + at(d->rows, j, m_mimus_r), m_mimus_r, d->xp, 1);
				}

				// 乘以PT
				rowAddInverseXp();

				// 乘以I, 因为bp里面储存了外力，因此不能用bp
				if (!hasGround())
				{
					double tem[6];
					s_iv_dot_as(diag_pool_.begin()->iv, diag_pool_.begin()->xp, tem);
					s_vc(6, tem, diag_pool_.begin()->xp);
				}
				for (auto d = diag_pool_.begin() + 1; d < diag_pool_.end(); ++d)
				{
					double tem[6];
					s_iv_dot_as(d->iv, d->xp, tem);
					s_vc(6, tem, d->xp);
				}

				// 乘以P, 类似rowAddBp();
				for (auto d = diag_pool_.rbegin(); d < diag_pool_.rend() - 1; ++d) s_va(6, d->xp, d->rd->xp);

				// 乘以D，并取出来
				for (auto d = diag_pool_.rbegin(); d<diag_pool_.rend() - 1; ++d)
				{
					s_mm(6 - d->rel_.dim, 1, 6, d->dm + at(d->rel_.dim, 0, 6), 6, d->xp, 1, G + at(d->rows, j, m_mimus_r), m_mimus_r);
				}
				if (!hasGround())s_vc(6, diag_pool_.begin()->xp, 1, G + at(diag_pool_.begin()->rows, j, m_mimus_r), m_mimus_r);

				// 后面（循环内外）会用到xp为0
				std::fill(diag_pool_.begin()->xp, diag_pool_.begin()->xp + 6, 0.0);
			}



			//// 求特解 ////
			s_vc(fn, bcf, xpf);
			s_permutate(fn, 1, FP, xpf);
			s_sov_lm(fr, 1, FU, ColMajor(fn), xpf, 1, xpf, 1, max_error_);
			s_householder_ut_q_dot(fm, fn, 1, FU, FT, xpf, xpf);



			//// 根据特解求解[F, G] 的右侧 ////
			updXp();
			rowAddInverseXp();
			updPfToBp();
			rowAddBp();
			updBpf();


			//// 根据G求解 ////
			// 首先用G左乘F产生的QT
			s_householder_ut_qt_dot(fm, fn, m_mimus_r, FU, FT, G, QT_DOT_G);
			s_householder_ut_qt_dot(fm, fn, 1, FU, FT, bpf, alpha);

			// 其次求解
			Size rank;
			s_householder_utp(m_mimus_r, m_mimus_r, QT_DOT_G + at(fr, 0, m_mimus_r), GU, GT, GP, rank, max_error_);
			///////////////////////////////
			// 可以通过rank == m-r来判断质点等是否影响计算
			///////////////////////////////
			s_householder_utp_sov(m_mimus_r, m_mimus_r, 1, rank, GU, GT, GP, alpha + fr, alpha);

			// 这里就求出了alpha
			// 接着求真正的bpf
			s_mma(fm, 1, m_mimus_r, G, alpha, bpf);

			// 求解力的信息
			s_householder_utp_sov(fm, fn, 1, fr, FU, FT, FP, bpf, xcf, max_error_);
			updXc();
			updCf();

			//// 根据alpha更新 xpf等 ////
			s_mms(fm, 1, m_mimus_r, S, alpha, xpf);

			updXp();
			rowAddInverseXp();
			updPa();
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
			// x = [ pa ]
			//     [ cf ]
			//
			// b = [ pf ]
			//     [ ca ]
			// 
			std::vector<Diag *> get_diag_from_part_id_;

			std::vector<SubSystem> subsys_pool_;

			std::vector<double> F_, FU_, FT_;
			std::vector<Size> FP_;
			std::vector<double> S_;
			std::vector<double> G_, GU_, GT_;
			std::vector<Size> GP_;

			std::vector<double> QT_DOT_G_;
			std::vector<double> xpf_, xcf_;
			std::vector<double> bpf_, bcf_;
			std::vector<double> alpha_;

			std::vector<double> Jg_, cg_;

			std::vector<double> M_, h_;

			static auto one_constraint_upd_d(Diag *d)->void
			{
				double makI_pm[16], makJ_pm[16];
				s_pm_dot_pm(d->rel_.cst_pool_.begin()->is_I ? d->pm : d->rd->pm, *d->rel_.cst_pool_.begin()->constraint->makI().prtPm(), makI_pm);
				s_pm_dot_pm(d->rel_.cst_pool_.begin()->is_I ? d->rd->pm : d->pm, *d->rel_.cst_pool_.begin()->constraint->makJ().prtPm(), makJ_pm);
				d->rel_.cst_pool_.begin()->constraint->cptGlbDmFromPm(d->dm, makI_pm, makJ_pm);
				if (!d->rel_.cst_pool_.begin()->is_I)s_iv(36, d->dm);
			}
			static auto revolute_upd_d(Diag *d)->void
			{
				double makI_pm[16], makJ_pm[16];
				s_pm_dot_pm(d->rel_.cst_pool_.begin()->is_I ? d->pm : d->rd->pm, *d->rel_.cst_pool_.begin()->constraint->makI().prtPm(), makI_pm);
				s_pm_dot_pm(d->rel_.cst_pool_.begin()->is_I ? d->rd->pm : d->pm, *d->rel_.cst_pool_.begin()->constraint->makJ().prtPm(), makJ_pm);
				d->rel_.cst_pool_.begin()->constraint->cptGlbDmFromPm(d->dm, makI_pm, makJ_pm);
				if (!d->rel_.cst_pool_.begin()->is_I)s_iv(36, d->dm);
			}
			static auto prismatic_upd_d(Diag *d)->void
			{
				double makI_pm[16], makJ_pm[16];
				s_pm_dot_pm(d->rel_.cst_pool_.begin()->is_I ? d->pm : d->rd->pm, *d->rel_.cst_pool_.begin()->constraint->makI().prtPm(), makI_pm);
				s_pm_dot_pm(d->rel_.cst_pool_.begin()->is_I ? d->rd->pm : d->pm, *d->rel_.cst_pool_.begin()->constraint->makJ().prtPm(), makJ_pm);
				d->rel_.cst_pool_.begin()->constraint->cptGlbDmFromPm(d->dm, makI_pm, makJ_pm);
				if (!d->rel_.cst_pool_.begin()->is_I)s_iv(36, d->dm);
			}
			static auto normal_upd_d(Diag *d)->void
			{
				double makI_pm[16], makJ_pm[16];
				s_pm_dot_pm(d->rel_.cst_pool_.begin()->is_I ? d->pm : d->rd->pm, *d->rel_.cst_pool_.begin()->constraint->makI().prtPm(), makI_pm);
				s_pm_dot_pm(d->rel_.cst_pool_.begin()->is_I ? d->rd->pm : d->pm, *d->rel_.cst_pool_.begin()->constraint->makJ().prtPm(), makJ_pm);

				double cm1[36], cm2[36];
				double U[36], tau[6];
				double Q[36], R[36];

				Size pos{ 0 };
				for (auto &c : d->rel_.cst_pool_)
				{
					double cmI_tem[36], cmJ_tem[36];

					double *cmI = c.is_I ? cm1 : cm2;
					double *cmJ = c.is_I ? cm2 : cm1;

					c.constraint->cptGlbCmFromPm(cmI_tem, cmJ_tem, makI_pm, makJ_pm);
					s_mc(6, c.constraint->dim(), cmI_tem, c.constraint->dim(), cmI + pos, d->rel_.dim);
					s_mc(6, c.constraint->dim(), cmJ_tem, c.constraint->dim(), cmJ + pos, d->rel_.dim);
					pos += c.constraint->dim();
				}

				s_householder_ut(6, d->rel_.dim, cm1, U, tau);
				s_householder_ut2qr(6, d->rel_.dim, U, tau, Q, R);

				double tem[36];
				s_fill(6, 6, 0.0, tem);
				s_fill(6, 1, 1.0, tem, 7);
				s_inv_um(d->rel_.dim, R, d->rel_.dim, tem, 6);
				s_mm(6, 6, 6, tem, 6, Q, dynamic::ColMajor{ 6 }, d->dm, 6);
			}
		};
		auto UniversalSolver::allocateMemory()->void
		{
			// make active part pool //
			std::vector<Part*> active_part_pool;
			active_part_pool.clear();
			active_part_pool.push_back(&model().ground());
			for (auto &p : model().partPool())if (p.active() && &p != &model().ground())active_part_pool.push_back(&p);

			// make active constraint pool //
			std::vector<Constraint*> cp;
			for (auto &jnt : model().jointPool())if (jnt.active())cp.push_back(&jnt);
			for (auto &mot : model().motionPool())if (mot.active()) cp.push_back(&mot);
			for (auto &gmt : model().generalMotionPool())if (gmt.active())cp.push_back(&gmt);

			// 制作包含所有relation的pool //
			std::vector<Relation> relation_pool;
			for (auto c : cp)
			{
				auto ret = std::find_if(relation_pool.begin(), relation_pool.end(), [&c](Relation &relation)
				{
					const auto ri{ relation.prtI }, rj{ relation.prtJ }, ci{ &c->makI().fatherPart() }, cj{ &c->makJ().fatherPart() };
					return ((ri == ci) && (rj == cj)) || ((ri == cj) && (rj == ci));
				});

				if (ret == relation_pool.end())
				{
					relation_pool.push_back(Relation{ &c->makI().fatherPart(), &c->makJ().fatherPart(), c->dim(),{ { c, true } } });
				}
				else
				{
					ret->dim += c->dim();
					ret->cst_pool_.push_back({ c, &c->makI().fatherPart() == ret->prtI });
				}
			}

			// 划分子系统,将相关的Part放到一起 //
			std::vector<std::vector<Part*> > part_pool_pool_;
			while (active_part_pool.size() > 1)
			{
				part_pool_pool_.push_back(std::vector<Part*>());
				SubSystem::addPart(part_pool_pool_.back(), active_part_pool, relation_pool, active_part_pool.at(1));
			}

			imp_->subsys_pool_.clear();
			// 分配子系统的内存 //
			Size max_fm{ 0 }, max_fn{ 0 };
			for (auto &part_pool : part_pool_pool_)
			{
				imp_->subsys_pool_.push_back(SubSystem());
				auto &sys = imp_->subsys_pool_.back();

				// 寻找本系统的relation_pool
				std::vector<Relation> sys_relation_pool;
				for (auto &rel : relation_pool)
				{
					if (std::find_if(part_pool.begin(), part_pool.end(), [&rel, this](const Part *prt) { return prt != &model().ground() && (prt == rel.prtI || prt == rel.prtJ); }) != part_pool.end())
					{
						sys_relation_pool.push_back(rel);
					}
				}

				// 对sys的part和relation排序 //
				for (Size i = 0; i < std::min(part_pool.size(), sys_relation_pool.size()); ++i)
				{
					// 先对part排序，找出下一个跟上一个part联系的part
					std::sort(part_pool.begin() + i, part_pool.end(), [i, this, &sys_relation_pool](Part* a, Part* b)
					{
						if (a == &model().ground()) return true; // 地面最优先
						if (b == &model().ground()) return false; // 地面最优先
						if (i == 0)return a->id() < b->id();// 第一轮先找地面或其他地面，防止下面的索引i-1出错
						if (b == sys_relation_pool[i - 1].prtI) return false;
						if (b == sys_relation_pool[i - 1].prtJ) return false;
						if (a == sys_relation_pool[i - 1].prtI) return true;
						if (a == sys_relation_pool[i - 1].prtJ) return true;
						return a->id() < b->id();
					});
					// 再插入连接新part的relation
					std::sort(sys_relation_pool.begin() + i, sys_relation_pool.end(), [i, this, &sys, &part_pool](Relation a, Relation b)
					{
						auto pend = part_pool.begin() + i + 1;
						auto a_part_i = std::find_if(part_pool.begin(), pend, [a](Part* p)->bool { return p == a.prtI; });
						auto a_part_j = std::find_if(part_pool.begin(), pend, [a](Part* p)->bool { return p == a.prtJ; });
						auto b_part_i = std::find_if(part_pool.begin(), pend, [b](Part* p)->bool { return p == b.prtI; });
						auto b_part_j = std::find_if(part_pool.begin(), pend, [b](Part* p)->bool { return p == b.prtJ; });

						bool a_is_ok = (a_part_i == pend) != (a_part_j == pend);
						bool b_is_ok = (b_part_i == pend) != (b_part_j == pend);

						if (a_is_ok && !b_is_ok) return true;
						else if (!a_is_ok && b_is_ok) return false;
						else if (a.dim != b.dim)return a.dim > b.dim;
						else return false;
					});
				}

				// 判断是否有地面 //
				sys.has_ground_ = (part_pool.front() == &model().ground());

				// 制造diag pool //
				sys.diag_pool_.clear();
				sys.diag_pool_.resize(part_pool.size());
				sys.diag_pool_.at(0).part = part_pool.at(0);
				sys.diag_pool_.at(0).rd = &sys.diag_pool_.at(0);
				sys.diag_pool_.at(0).pm = sys.diag_pool_.at(0).pm1;
				sys.diag_pool_.at(0).last_pm = sys.diag_pool_.at(0).pm2;

				for (Size i = 1; i < sys.diag_pool_.size(); ++i)
				{
					auto &diag = sys.diag_pool_.at(i);

					diag.rel_ = sys_relation_pool.at(i - 1);
					// 如果relation的prtI不是对角线的杆件，那么进行反转
					if (diag.rel_.prtI != part_pool.at(i))
					{
						std::swap(diag.rel_.prtI, diag.rel_.prtJ);
						for (auto &c : diag.rel_.cst_pool_)c.is_I = !c.is_I;
					}

					diag.part = diag.rel_.prtI;
					diag.rd = &*std::find_if(sys.diag_pool_.begin(), sys.diag_pool_.end(), [&](Diag &d) {return d.part == diag.rel_.prtJ; });
					diag.pm = diag.pm1;
					diag.last_pm = diag.pm2;

					// 以下优化dm矩阵的计算
					// 针对约束仅仅有一个时的优化 //
					if (diag.rel_.cst_pool_.size() == 1)
					{
						diag.upd_d = Imp::one_constraint_upd_d;
					}
					// 针对转动副加转动电机 //
					else if (diag.rel_.cst_pool_.size() == 2
						&& dynamic_cast<RevoluteJoint*>(diag.rel_.cst_pool_.at(0).constraint)
						&& dynamic_cast<Motion*>(diag.rel_.cst_pool_.at(1).constraint)
						&& dynamic_cast<Motion*>(diag.rel_.cst_pool_.at(1).constraint)->axis() == 5
						&& &diag.rel_.cst_pool_.at(0).constraint->makI() == &diag.rel_.cst_pool_.at(1).constraint->makI())
					{
						diag.upd_d = Imp::revolute_upd_d;
					}
					// 针对移动副加移动电机 //
					else if (diag.rel_.cst_pool_.size() == 2
						&& dynamic_cast<PrismaticJoint*>(diag.rel_.cst_pool_.at(0).constraint)
						&& dynamic_cast<Motion*>(diag.rel_.cst_pool_.at(1).constraint)
						&& dynamic_cast<Motion*>(diag.rel_.cst_pool_.at(1).constraint)->axis() == 2
						&& &diag.rel_.cst_pool_.at(0).constraint->makI() == &diag.rel_.cst_pool_.at(1).constraint->makI())
					{
						diag.upd_d = Imp::prismatic_upd_d;
					}
					// 不优化 //
					else
					{
						diag.upd_d = Imp::normal_upd_d;
					}
				}

				// 制造remainder pool //
				sys.remainder_pool_.clear();
				sys.remainder_pool_.resize(sys_relation_pool.size() - part_pool.size() + 1);
				for (Size i = 0; i < sys.remainder_pool_.size(); ++i)
				{
					auto &r = sys.remainder_pool_.at(i);

					r.rel_ = sys_relation_pool.at(i + sys.diag_pool_.size() - 1);
					r.i_diag = &*std::find_if(sys.diag_pool_.begin(), sys.diag_pool_.end(), [&r](Diag& d) {return r.rel_.prtI == d.part; });
					r.j_diag = &*std::find_if(sys.diag_pool_.begin(), sys.diag_pool_.end(), [&r](Diag& d) {return r.rel_.prtJ == d.part; });
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

				// 计算所需要内存 //
				sys.fm = 0;
				sys.fn = 0;
				sys.fr = 0;

				for (auto d = sys.diag_pool_.begin() + 1; d < sys.diag_pool_.end(); ++d)
				{
					d->rows = sys.fm;
					sys.fm += 6 - d->rel_.dim;
				}
				if (!sys.hasGround())
				{
					sys.diag_pool_.begin()->rows = sys.fm;
					sys.fm += 6;
				}
				for (auto &r : sys.remainder_pool_) sys.fn += r.rel_.dim;

				max_fm = std::max(sys.fm, max_fm);
				max_fn = std::max(sys.fn, max_fn);
			}

			// 分配根据part id寻找diag的vector //
			imp_->get_diag_from_part_id_.clear();
			imp_->get_diag_from_part_id_.resize(model().partPool().size(), nullptr);
			for (auto &sys : imp_->subsys_pool_)
			{
				for (auto &diag : sys.diag_pool_)
				{
					imp_->get_diag_from_part_id_.at(diag.part->id()) = &diag;
				}
			}


			// 分配计算所需内存 //
			imp_->F_.clear();
			imp_->F_.resize(max_fm*max_fn, 0.0);
			imp_->FU_.clear();
			imp_->FU_.resize(max_fm*max_fn, 0.0);
			imp_->FT_.clear();
			imp_->FT_.resize(std::max(max_fm, max_fn), 0.0);
			imp_->FP_.clear();
			imp_->FP_.resize(std::max(max_fm, max_fn), 0);
			imp_->G_.clear();
			imp_->G_.resize(max_fm*max_fm, 0.0);
			imp_->GU_.clear();
			imp_->GU_.resize(max_fm*max_fm, 0.0);
			imp_->GT_.clear();
			imp_->GT_.resize(max_fm, 0.0);
			imp_->GP_.clear();
			imp_->GP_.resize(max_fm, 0);

			imp_->S_.clear();
			imp_->S_.resize(max_fm*max_fm, 0.0);
			imp_->alpha_.clear();
			imp_->alpha_.resize(max_fm, 0.0);
			imp_->QT_DOT_G_.clear();
			imp_->QT_DOT_G_.resize(max_fm*max_fm, 0.0);


			// 这里必须给xcf等分配尽量大的内存，因为s_house_holder_ut_q_dot要求x > b
			imp_->xcf_.clear();
			imp_->xcf_.resize(std::max(max_fn, max_fm), 0.0);
			imp_->xpf_.clear();
			imp_->xpf_.resize(std::max(max_fn, max_fm), 0.0);
			imp_->bcf_.clear();
			imp_->bcf_.resize(max_fn, 0.0);
			imp_->bpf_.clear();
			imp_->bpf_.resize(max_fm, 0.0);

			// 将内存付给子系统 //
			for (auto &sys : imp_->subsys_pool_)
			{
				sys.has_ground_ = sys.diag_pool_.begin()->part == &model().ground();

				sys.F = imp_->F_.data();
				sys.FU = imp_->FU_.data();
				sys.FT = imp_->FT_.data();
				sys.FP = imp_->FP_.data();

				sys.G = imp_->G_.data();
				sys.GU = imp_->GU_.data();
				sys.GT = imp_->GT_.data();
				sys.GP = imp_->GP_.data();

				sys.S = imp_->S_.data();
				sys.alpha = imp_->alpha_.data();
				sys.QT_DOT_G = imp_->QT_DOT_G_.data();

				sys.xcf = imp_->xcf_.data();
				sys.xpf = imp_->xpf_.data();
				sys.bcf = imp_->bcf_.data();
				sys.bpf = imp_->bpf_.data();
			}

			// 分配内存给雅可比 //
			imp_->Jg_.clear();
			imp_->Jg_.resize(model().partPool().size() * 6 * (model().motionPool().size() + model().generalMotionPool().size() * 6), 0.0);
			imp_->cg_.clear();
			imp_->cg_.resize(model().partPool().size() * 6, 0.0);

			// 分配内存给动力学通用形式 //
			imp_->M_.clear();
			imp_->M_.resize((model().motionPool().size() + model().generalMotionPool().size() * 6) * (model().motionPool().size() + model().generalMotionPool().size() * 6), 0.0);
			imp_->h_.clear();
			imp_->h_.resize((model().motionPool().size() + model().generalMotionPool().size() * 6), 0.0);
		}
		auto UniversalSolver::kinPos()->bool
		{
			// 将杆件位姿拷贝到局部变量中 //
			for (auto &sys : imp_->subsys_pool_)for (auto &d : sys.diag_pool_)d.part->getPm(d.pm);

			double pm[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
			s_mc(4, 4, pm, const_cast<double *>(*model().ground().pm()));

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
			if (error() < maxError())for (auto &sys : imp_->subsys_pool_)for (auto &d : sys.diag_pool_)d.part->setPm(d.pm);
			return error() < maxError();
		}
		auto UniversalSolver::kinVel()->void
		{
			for (auto &sys : imp_->subsys_pool_)for (auto &d : sys.diag_pool_)d.part->getPm(d.pm);

			s_fill(6, 1, 0.0, const_cast<double *>(model().ground().vs()));
			for (auto &sys : imp_->subsys_pool_)sys.kinVel();
		}
		auto UniversalSolver::dynAccAndFce()->void
		{
			for (auto &sys : imp_->subsys_pool_)for (auto &d : sys.diag_pool_)d.part->getPm(d.pm);
			for (auto &sys : imp_->subsys_pool_)for (auto &d : sys.diag_pool_)std::fill(d.bp, d.bp + 6, 0.0);

			// 更新外力 //
			for (auto &fce : model().forcePool())
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
			s_fill(6, 1, 0.0, const_cast<double *>(model().ground().as()));
			for (auto &sys : imp_->subsys_pool_) sys.dynAccAndFce();
		}
		auto UniversalSolver::cptGeneralJacobi()->void
		{
			for (auto &sys : imp_->subsys_pool_)for (auto &d : sys.diag_pool_)d.part->getPm(d.pm);

			std::fill(imp_->Jg_.begin(), imp_->Jg_.end(), 0.0);
			std::fill(imp_->cg_.begin(), imp_->cg_.end(), 0.0);

			for (auto &sys : imp_->subsys_pool_)
			{
				// make A
				sys.updDiagDm();
				sys.updRemainderCm();

				// solve
				sys.updF();

				for (auto &d : sys.diag_pool_) std::fill(d.bc, d.bc + 6, 0.0);
				for (auto &r : sys.remainder_pool_) std::fill(r.bc, r.bc + 6, 0.0);

				auto func = [&](Size c_pos) 
				{
					sys.updBcf();
					sys.updXpf();
					sys.updXp();
					sys.rowAddInverseXp();

					auto beg = sys.hasGround() ? sys.diag_pool_.begin() + 1 : sys.diag_pool_.begin();
					for (auto d = beg; d < sys.diag_pool_.end(); ++d)
					{
						s_vc(6, d->xp, 1, &imp_->Jg_.at(at(d->part->id() * 6, c_pos, nJ())), nJ());
					}
				};


				// upd Jg //
				for (auto &d : sys.diag_pool_)
				{
					Size pos = 0;
					for (auto &c : d.rel_.cst_pool_)
					{
						if (dynamic_cast<Motion*>(c.constraint))
						{
							// 更新bc，将当前电机的未知量更新为当前c的1.0 //
							d.bc[pos] = 1.0;

							func(c.constraint->id());

							std::fill(d.bc, d.bc + 6, 0.0);
						}
						else if (dynamic_cast<GeneralMotion*>(c.constraint))
						{
							double tmf[6][6];
							s_tmf(*dynamic_cast<GeneralMotion*>(c.constraint)->mpm(), *tmf);

							for (Size k(-1); ++k < 6;)
							{
								// 更新bc，将当前电机的未知量更新为当前c的1.0 //
								// Tmf^(T) * v //
								s_vc(6, tmf[k], d.bc);

								func(model().motionPool().size() + c.constraint->id() * 6 + k);
							}

							std::fill(d.bc, d.bc + 6, 0.0);
						}

						pos += c.constraint->dim();
					}
				}
				for (auto &r : sys.remainder_pool_)
				{
					Size pos = 0;
					for (auto &c : r.rel_.cst_pool_)
					{
						if (dynamic_cast<Motion*>(c.constraint))
						{
							// 更新bc，将当前电机的未知量更新为当前c的1.0 //
							r.bc[0] = 1.0;

							func(c.constraint->id());

							std::fill(r.bc, r.bc + 6, 0.0);
						}
						else if (dynamic_cast<GeneralMotion*>(c.constraint))
						{
							double tmf[6][6];
							s_tmf(*dynamic_cast<GeneralMotion*>(c.constraint)->mpm(), *tmf);

							for (Size k(-1); ++k < 6;)
							{
								// 更新bc，将当前电机的未知量更新为当前c的1.0 //
								// Tmf^(T) * v //
								s_vc(6, tmf[k], r.bc);

								func(model().motionPool().size() + c.constraint->id() + k);
							}

							std::fill(r.bc, r.bc + 6, 0.0);
						}

						pos += c.constraint->dim();
					}
				}

				sys.updCaToBc();
				// upd cg //
				for (auto &d : sys.diag_pool_)
				{
					Size pos = 0;
					for (auto &c : d.rel_.cst_pool_)
					{
						if (dynamic_cast<Motion*>(c.constraint))
						{
							d.bc[pos] -= dynamic_cast<Motion*>(c.constraint)->ma();
						}
						else if (dynamic_cast<GeneralMotion*>(c.constraint))
						{
							auto gm = dynamic_cast<GeneralMotion*>(c.constraint);
							s_inv_tva(-1.0, *gm->mpm(), gm->mas(), d.bc);
						}

						pos += c.constraint->dim();
					}
				}
				for (auto &r : sys.remainder_pool_)
				{
					Size pos = 0;
					for (auto &c : r.rel_.cst_pool_)
					{
						if (dynamic_cast<Motion*>(c.constraint))
						{
							r.bc[pos] -= dynamic_cast<Motion*>(c.constraint)->ma();
						}
						else if (dynamic_cast<GeneralMotion*>(c.constraint))
						{
							auto gm = dynamic_cast<GeneralMotion*>(c.constraint);
							s_inv_tva(-1.0, *gm->mpm(), gm->mas(), r.bc);
						}

						pos += c.constraint->dim();
					}
				}

				sys.updBcf();
				sys.updXpf();
				sys.updXp();
				sys.rowAddInverseXp();

				auto beg = sys.hasGround() ? sys.diag_pool_.begin() + 1 : sys.diag_pool_.begin();
				for (auto d = beg; d < sys.diag_pool_.end(); ++d)
				{
					s_vc(6, d->xp, imp_->cg_.data() + d->part->id() * 6);
				}
			}
		}
		auto UniversalSolver::mJ()->Size { return model().partPool().size() * 6; }
		auto UniversalSolver::nJ()->Size { return model().motionPool().size() + model().generalMotionPool().size() * 6; }
		auto UniversalSolver::Jg()->double * { return imp_->Jg_.data(); }
		auto UniversalSolver::cg()->double * { return imp_->cg_.data(); }
		auto UniversalSolver::cptGeneralInverseDynamicMatrix()->void
		{
			for (auto &sys : imp_->subsys_pool_)for (auto &d : sys.diag_pool_)d.part->getPm(d.pm);

			// 更新地面的as //
			s_fill(6, 1, 0.0, const_cast<double *>(model().ground().as()));


			std::fill(imp_->M_.begin(), imp_->M_.end(), 0.0);
			std::fill(imp_->h_.begin(), imp_->h_.end(), 0.0);

			for (auto &sys : imp_->subsys_pool_)
			{
				// 更新矩阵 //
				sys.updDiagIv();
				sys.updDiagDm();
				sys.updRemainderCm();
				sys.updF();
				s_householder_utp(sys.fm, sys.fn, sys.F, sys.FU, sys.FT, sys.FP, sys.fr, sys.max_error_);
				Size m_mimus_r = sys.fm - sys.fr;
				s_fill(sys.fm, 1, 0.0, sys.xpf);
				for (Size j(-1); ++j < m_mimus_r;)
				{
					sys.xpf[sys.fr + j] = 1.0;
					s_householder_ut_q_dot(sys.fm, sys.fn, 1, sys.FU, sys.fn, sys.FT, 1, sys.xpf, 1, sys.S + j, m_mimus_r);
					sys.xpf[sys.fr + j] = 0.0;
					if (!sys.hasGround())s_vc(6, sys.S + at(sys.diag_pool_.begin()->rows, j, m_mimus_r), m_mimus_r, sys.diag_pool_.begin()->xp, 1);
					for (auto d = sys.diag_pool_.begin() + 1; d<sys.diag_pool_.end(); ++d)
					{
						s_mm(6, 1, 6 - d->rel_.dim, d->dm + at(0, d->rel_.dim, ColMajor{ 6 }), ColMajor{ 6 }, sys.S + at(d->rows, j, m_mimus_r), m_mimus_r, d->xp, 1);
					}
					sys.rowAddInverseXp();
					if (!sys.hasGround())
					{
						double tem[6];
						s_iv_dot_as(sys.diag_pool_.begin()->iv, sys.diag_pool_.begin()->xp, tem);
						s_vc(6, tem, sys.diag_pool_.begin()->xp);
					}
					for (auto d = sys.diag_pool_.begin() + 1; d < sys.diag_pool_.end(); ++d)
					{
						double tem[6];
						s_iv_dot_as(d->iv, d->xp, tem);
						s_vc(6, tem, d->xp);
					}
					for (auto d = sys.diag_pool_.rbegin(); d < sys.diag_pool_.rend() - 1; ++d) s_va(6, d->xp, d->rd->xp);
					for (auto d = sys.diag_pool_.rbegin(); d < sys.diag_pool_.rend() - 1; ++d)
					{
						s_mm(6 - d->rel_.dim, 1, 6, d->dm + at(d->rel_.dim, 0, 6), 6, d->xp, 1, sys.G + at(d->rows, j, m_mimus_r), m_mimus_r);
					}
					if (!sys.hasGround())s_vc(6, sys.diag_pool_.begin()->xp, 1, sys.G + at(sys.diag_pool_.begin()->rows, j, m_mimus_r), m_mimus_r);
					std::fill(sys.diag_pool_.begin()->xp, sys.diag_pool_.begin()->xp + 6, 0.0);
				}
				sys.updCaToBc();

				auto func = [&]()
				{
					for (auto &sys : imp_->subsys_pool_)for (auto &d : sys.diag_pool_)std::fill(d.bp, d.bp + 6, 0.0);
					for (auto &fce : model().forcePool())
					{
						if (fce.active())
						{
							double fsI[6], fsJ[6];
							fce.cptGlbFs(fsI, fsJ);
							s_vs(6, fsI, imp_->get_diag_from_part_id_.at(fce.makI().fatherPart().id())->bp);
							s_vs(6, fsJ, imp_->get_diag_from_part_id_.at(fce.makJ().fatherPart().id())->bp);
						}
					}
					sys.updBcf();
					s_vc(sys.fn, sys.bcf, sys.xpf);
					s_permutate(sys.fn, 1, sys.FP, sys.xpf);
					s_sov_lm(sys.fr, 1, sys.FU, ColMajor(sys.fn), sys.xpf, 1, sys.xpf, 1, sys.max_error_);
					s_householder_ut_q_dot(sys.fm, sys.fn, 1, sys.FU, sys.FT, sys.xpf, sys.xpf);
					sys.updXp();
					sys.rowAddInverseXp();
					sys.updPfToBp();
					sys.rowAddBp();
					sys.updBpf();
					s_householder_ut_qt_dot(sys.fm, sys.fn, m_mimus_r, sys.FU, sys.FT, sys.G, sys.QT_DOT_G);
					s_householder_ut_qt_dot(sys.fm, sys.fn, 1, sys.FU, sys.FT, sys.bpf, sys.alpha);
					Size rank;
					s_householder_utp(m_mimus_r, m_mimus_r, sys.QT_DOT_G + at(sys.fr, 0, m_mimus_r), sys.GU, sys.GT, sys.GP, rank, sys.max_error_);
					///////////////////////////////
					// 可以通过rank == m-r来判断质点等是否影响计算
					///////////////////////////////
					s_householder_utp_sov(m_mimus_r, m_mimus_r, 1, rank, sys.GU, sys.GT, sys.GP, sys.alpha + sys.fr, sys.alpha);
					s_mma(sys.fm, 1, m_mimus_r, sys.G, sys.alpha, sys.bpf);
					s_householder_utp_sov(sys.fm, sys.fn, 1, sys.fr, sys.FU, sys.FT, sys.FP, sys.bpf, sys.xcf, sys.max_error_);
					sys.updXc();
				};

				// 开始计算h //
				// 先去掉驱动的加速度, 并计算h
				for (auto &d : sys.diag_pool_)
				{
					Size pos{ 0 };
					for (auto &c : d.rel_.cst_pool_)
					{
						if (dynamic_cast<Motion*>(c.constraint))
						{
							d.bc[pos] -= dynamic_cast<Motion*>(c.constraint)->ma();
						}
						if (dynamic_cast<GeneralMotion*>(c.constraint))
						{
							auto gm = dynamic_cast<GeneralMotion*>(c.constraint);
							s_inv_tva(-1.0, *gm->mpm(), gm->mas(), d.bc);
						}
						pos += c.constraint->dim();
					}
				}
				for (auto &r : sys.remainder_pool_)
				{
					Size pos{ 0 };
					// 将Xcf更新 //
					for (auto &c : r.rel_.cst_pool_)
					{
						if (dynamic_cast<Motion*>(c.constraint))
						{
							r.bc[pos] -= dynamic_cast<Motion*>(c.constraint)->ma();
						}
						if (dynamic_cast<GeneralMotion*>(c.constraint))
						{
							auto gm = dynamic_cast<GeneralMotion*>(c.constraint);
							s_inv_tva(-1.0, *gm->mpm(), gm->mas(), r.bc);
						}
						pos += c.constraint->dim();
					}
				}
				func();
				for (auto &r : sys.remainder_pool_)
				{
					Size pos{ 0 };
					// 将Xcf更新 //
					for (auto &c : r.rel_.cst_pool_)
					{
						if (dynamic_cast<Motion*>(c.constraint))
						{
							imp_->h_[dynamic_cast<Motion*>(c.constraint)->id()] = r.xc[pos];
						}
						if (dynamic_cast<GeneralMotion*>(c.constraint))
						{
							s_vc(6, r.xc + pos, imp_->h_.data() + model().motionPool().size() + c.constraint->id() * 6);
						}
						pos += c.constraint->dim();
					}
				}
				for (auto d = sys.diag_pool_.begin() + 1; d < sys.diag_pool_.end(); ++d)
				{
					Size pos{ 0 };
					for (auto &c : d->rel_.cst_pool_)
					{
						if (dynamic_cast<Motion*>(c.constraint))
						{
							imp_->h_[dynamic_cast<Motion*>(c.constraint)->id()] = d->xc[pos];
						}
						if (dynamic_cast<GeneralMotion*>(c.constraint))
						{
							s_vc(6, d->xc, imp_->h_.data() + model().motionPool().size() + c.constraint->id() * 6);
						}
						pos += c.constraint->dim();
					}
				}

				// 开始计算M //
				auto func2 = [&](Constraint *c, Size cid)
				{
					auto m = model().motionPool().size() + model().generalMotionPool().size() * 6;

					for (auto &rr : sys.remainder_pool_)
					{
						Size pos2{ 0 };
						for (auto &cc : rr.rel_.cst_pool_)
						{
							if (dynamic_cast<Motion*>(cc.constraint))
							{
								Size ccid = cc.constraint->id();
								imp_->M_[at(ccid, cid, m)] = rr.xc[pos2] - h()[ccid];
							}
							if (dynamic_cast<GeneralMotion*>(cc.constraint))
							{
								Size ccid = cc.constraint->id() * 6 + model().motionPool().size();
								for (Size i = 0; i < 6; ++i)
								{
									s_vc(6, rr.xc, 1, imp_->M_.data() + at(ccid, cid, m), m);
									s_vs(6, h() + ccid, 1, imp_->M_.data() + at(ccid, cid, m), m);
								}

							}
							pos2 += cc.constraint->dim();
						}
					}
					for (auto dd = sys.diag_pool_.begin() + 1; dd < sys.diag_pool_.end(); ++dd)
					{
						Size pos2{ 0 };
						for (auto &cc : dd->rel_.cst_pool_)
						{
							if (dynamic_cast<Motion*>(cc.constraint))
							{
								Size ccid = cc.constraint->id();
								imp_->M_[at(cc.constraint->id(), cid, m)] = dd->xc[pos2] - h()[cc.constraint->id()];
							}
							if (dynamic_cast<GeneralMotion*>(cc.constraint))
							{
								Size ccid = cc.constraint->id() * 6 + model().motionPool().size();
								s_vc(6, dd->xc, 1, imp_->M_.data() + at(ccid, cid, m), m);
								s_vs(6, h() + ccid, 1, imp_->M_.data() + at(ccid, cid, m), m);
							}
							pos2 += cc.constraint->dim();
						}
					}
				};
				for (auto &d : sys.diag_pool_)
				{
					Size pos{ 0 };
					for (auto &c : d.rel_.cst_pool_)
					{
						if (dynamic_cast<Motion*>(c.constraint))
						{
							d.bc[pos] += 1.0;

							func();

							func2(c.constraint, c.constraint->id());

							d.bc[pos] -= 1.0;
						}
						else if (dynamic_cast<GeneralMotion*>(c.constraint))
						{
							double tmf[6][6];
							s_tmf(*dynamic_cast<GeneralMotion*>(c.constraint)->mpm(), *tmf);
							for (Size i = 0; i < 6; ++i)
							{
								s_va(6, tmf[i], d.bc);

								func();

								func2(c.constraint, c.constraint->id()*6+model().motionPool().size()+ i);

								s_vs(6, tmf[i], d.bc);
							}
						}
						pos += c.constraint->dim();
					}
				}
				for (auto &r : sys.remainder_pool_)
				{
					Size pos{ 0 };
					for (auto &c : r.rel_.cst_pool_)
					{
						if (dynamic_cast<Motion*>(c.constraint))
						{
							r.bc[pos] += 1.0;

							func();

							func2(c.constraint, c.constraint->id());

							r.bc[pos] -= 1.0;
						}
						else if (dynamic_cast<GeneralMotion*>(c.constraint))
						{
							double tmf[6][6];
							s_tmf(*dynamic_cast<GeneralMotion*>(c.constraint)->mpm(), *tmf);

							for (Size i = 0; i < 6; ++i)
							{
								s_va(6, tmf[i], r.bc);

								func();

								func2(c.constraint, c.constraint->id() * 6 + model().motionPool().size() + i);

								s_vs(6, tmf[i], r.bc);
							}
						}
						pos += c.constraint->dim();
					}
				}
			}
		}
		auto UniversalSolver::nM()->Size { return model().motionPool().size() + model().generalMotionPool().size() * 6; }
		auto UniversalSolver::M()->double * { return imp_->M_.data(); }
		auto UniversalSolver::h()->double * { return imp_->h_.data(); }
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
							if (d1.part == &model().ground())
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
							std::cout << " 6x" << rel.dim;
						}
						else if (rel.cst_pool_.begin()->is_I && rel.prtJ == d1.part)
						{
							std::cout << "-6x" << rel.dim;
						}
						else if (rel.prtI == d1.part)
						{
							std::cout << "-6x" << rel.dim;
						}
						else if (rel.prtJ == d1.part)
						{
							std::cout << " 6x" << rel.dim;
						}
						else
							std::cout << "    ";
						std::cout << " ";

					}
					for (auto &r : sys.remainder_pool_)
					{
						std::cout << " ";


						if (r.rel_.prtI == d1.part)
							std::cout << " 6x" << r.rel_.dim;
						else if (r.rel_.prtJ == d1.part)
							std::cout << "-6x" << r.rel_.dim;
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
							if (d1.part == &model().ground())
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
							std::cout << " 6x" << rel.dim;
						}
						else if (!rel.cst_pool_.begin()->is_I && rel.prtI == d1.part)
						{
							std::cout << "-6x" << rel.dim;
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
									std::cout << " 6x" << r.rel_.dim;
								else
									std::cout << "-6x" << r.rel_.dim;
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
		UniversalSolver::UniversalSolver(const UniversalSolver &other) = default;
		UniversalSolver::UniversalSolver(UniversalSolver &&other) = default;
		UniversalSolver& UniversalSolver::operator=(const UniversalSolver &other) = default;
		UniversalSolver& UniversalSolver::operator=(UniversalSolver &&other) = default;

		auto ForwardKinematicSolver::allocateMemory()->void
		{
			for (auto &m : model().motionPool())m.activate(true);
			for (auto &gm : model().generalMotionPool())gm.activate(false);

			UniversalSolver::allocateMemory();
		}
		auto ForwardKinematicSolver::kinPos()->bool
		{
			UniversalSolver::kinPos();
			if (error() < maxError())for (auto &m : model().generalMotionPool())m.updMpm();
			return error() < maxError();
		}
		auto ForwardKinematicSolver::kinVel()->void
		{
			UniversalSolver::kinVel();
			for (auto &m : model().generalMotionPool())m.updMvs();
		}
		auto ForwardKinematicSolver::dynAccAndFce()->void
		{
			UniversalSolver::dynAccAndFce();
			for (auto &m : model().motionPool())m.updMa();
		}
		ForwardKinematicSolver::~ForwardKinematicSolver() = default;
		ForwardKinematicSolver::ForwardKinematicSolver(const std::string &name, Size max_iter_count, double max_error) :UniversalSolver(name, max_iter_count, max_error) {}
		ForwardKinematicSolver::ForwardKinematicSolver(const ForwardKinematicSolver &other) = default;
		ForwardKinematicSolver::ForwardKinematicSolver(ForwardKinematicSolver &&other) = default;
		ForwardKinematicSolver& ForwardKinematicSolver::operator=(const ForwardKinematicSolver &other) = default;
		ForwardKinematicSolver& ForwardKinematicSolver::operator=(ForwardKinematicSolver &&other) = default;

		auto InverseKinematicSolver::allocateMemory()->void
		{
			for (auto &m : model().motionPool())m.activate(false);
			for (auto &gm : model().generalMotionPool())gm.activate(true);

			UniversalSolver::allocateMemory();
		}
		auto InverseKinematicSolver::kinPos()->bool
		{
			UniversalSolver::kinPos();
			if (error() < maxError())for (auto &m : model().motionPool())m.updMp();
			return error() < maxError();
		}
		auto InverseKinematicSolver::kinVel()->void
		{
			UniversalSolver::kinVel();
			for (auto &m : model().motionPool())m.updMv();
		}
		auto InverseKinematicSolver::dynAccAndFce()->void
		{
			UniversalSolver::dynAccAndFce();
			for (auto &m : model().generalMotionPool())m.updMas();
		}
		InverseKinematicSolver::~InverseKinematicSolver() = default;
		InverseKinematicSolver::InverseKinematicSolver(const std::string &name, Size max_iter_count, double max_error) :UniversalSolver(name, max_iter_count, max_error) {}
		InverseKinematicSolver::InverseKinematicSolver(const InverseKinematicSolver &other) = default;
		InverseKinematicSolver::InverseKinematicSolver(InverseKinematicSolver &&other) = default;
		InverseKinematicSolver& InverseKinematicSolver::operator=(const InverseKinematicSolver &other) = default;
		InverseKinematicSolver& InverseKinematicSolver::operator=(InverseKinematicSolver &&other) = default;

		auto ForwardDynamicSolver::allocateMemory()->void
		{
			for (auto &m : model().motionPool())m.activate(false);
			for (auto &gm : model().generalMotionPool())gm.activate(false);
			for (auto &f : model().forcePool())f.activate(true);
			UniversalSolver::allocateMemory();
		}
		auto ForwardDynamicSolver::kinPos()->bool
		{
			UniversalSolver::kinPos();
			if (error() < maxError())for (auto &m : model().generalMotionPool())m.updMpm();
			return error() < maxError();
		}
		auto ForwardDynamicSolver::kinVel()->void
		{
			UniversalSolver::kinVel();
			for (auto &m : model().generalMotionPool())m.updMvs();
		}
		auto ForwardDynamicSolver::dynAccAndFce()->void
		{
			UniversalSolver::dynAccAndFce();
			for (auto &m : model().motionPool())m.updMa();
			for (auto &m : model().generalMotionPool())m.updMas();
		}
		ForwardDynamicSolver::~ForwardDynamicSolver() = default;
		ForwardDynamicSolver::ForwardDynamicSolver(const std::string &name, Size max_iter_count, double max_error) :UniversalSolver(name, max_iter_count, max_error) {}
		ForwardDynamicSolver::ForwardDynamicSolver(const ForwardDynamicSolver &other) = default;
		ForwardDynamicSolver::ForwardDynamicSolver(ForwardDynamicSolver &&other) = default;
		ForwardDynamicSolver& ForwardDynamicSolver::operator=(const ForwardDynamicSolver &other) = default;
		ForwardDynamicSolver& ForwardDynamicSolver::operator=(ForwardDynamicSolver &&other) = default;

		auto InverseDynamicSolver::allocateMemory()->void
		{
			for (auto &m : model().motionPool())m.activate(true);
			for (auto &gm : model().generalMotionPool())gm.activate(false);
			for (auto &f : model().forcePool())f.activate(false);
			UniversalSolver::allocateMemory();
		}
		auto InverseDynamicSolver::kinPos()->bool
		{
			UniversalSolver::kinPos();
			if (error() < maxError())for (auto &m : model().motionPool())m.updMp();
			return error() < maxError();
		}
		auto InverseDynamicSolver::kinVel()->void
		{
			UniversalSolver::kinVel();
			for (auto &m : model().motionPool())m.updMv();
		}
		auto InverseDynamicSolver::dynAccAndFce()->void
		{
			UniversalSolver::dynAccAndFce();
			for (auto &m : model().generalMotionPool())m.updMas();
		}
		InverseDynamicSolver::~InverseDynamicSolver() = default;
		InverseDynamicSolver::InverseDynamicSolver(const std::string &name, Size max_iter_count, double max_error) :UniversalSolver(name, max_iter_count, max_error) {}
		InverseDynamicSolver::InverseDynamicSolver(const InverseDynamicSolver &other) = default;
		InverseDynamicSolver::InverseDynamicSolver(InverseDynamicSolver &&other) = default;
		InverseDynamicSolver& InverseDynamicSolver::operator=(const InverseDynamicSolver &other) = default;
		InverseDynamicSolver& InverseDynamicSolver::operator=(InverseDynamicSolver &&other) = default;

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
			std::vector<Part*> part_vec(7, nullptr);
			std::vector<RevoluteJoint*> joint_vec(6, nullptr);
			GeneralMotion *ee;

			// 必然有地，且与地相连的为杆件1, 那么找出所有杆件 //
			part_vec[0] = sys.diag_pool_.at(0).part;
			part_vec[6] = sys.diag_pool_.at(1).part;
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
		auto UrInverseKinematic(Model &m, SubSystem &sys, int which_root)->bool
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

			GeneralMotion *ee = &m.generalMotionPool().at(0);

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

			double q[6]{0};


			double E_in_A[16];
			s_inv_pm_dot_pm(A_pm, *ee->mpm(), E_in_A);

			double D_in_A[16];
			s_pm_dot_inv_pm(E_in_A, E_pm_in_D, D_in_A);


			// 开始求1轴 //
			// 求第一根轴的位置，这里末端可能工作空间以外，此时末端离原点过近，判断方法为查看以下if //
			// 事实上这里可以有2个解
			if (d3 > std::sqrt(D_in_A[3] * D_in_A[3] + D_in_A[7] * D_in_A[7])) return false;
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
			if (l > (d1 + d2) || l<(std::max(std::abs(d1), std::abs(d2)) - std::min(std::abs(d1), std::abs(d2))))return false;


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
				s_pm_dot_pm(*m.jointPool().at(i).makJ().pm(), s_pe2pm(pe3, pm, "123"), pm1);
				s_pm_dot_inv_pm(pm1, *m.jointPool().at(i).makI().prtPm(), const_cast<double*>(*m.jointPool().at(i).makI().fatherPart().pm()));
				
				
				auto last_mp = m.motionPool().at(i).mp();
				m.motionPool().at(i).updMp();
				while (m.motionPool().at(i).mp() - last_mp > PI)m.motionPool().at(i).setMp(m.motionPool().at(i).mp() - 2 * PI);
				while (m.motionPool().at(i).mp() - last_mp < -PI)m.motionPool().at(i).setMp(m.motionPool().at(i).mp() + 2 * PI);
			}


			return true;
		}

		auto Ur5InverseKinematicSolver::setWhichRoot(int root_of_0_to_7)
		{
			if (root_of_0_to_7 < 0 || root_of_0_to_7 > 7) throw std::runtime_error("root must be 0 to 7");
			which_root_ = root_of_0_to_7;
		}
		auto Ur5InverseKinematicSolver::kinPos()->bool
		{
			return UrInverseKinematic(model(), imp_->subsys_pool_.at(0), which_root_);
		};
	}
}
