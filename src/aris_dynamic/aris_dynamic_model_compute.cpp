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
		auto Solver::init()->void
		{
			allocateMemory();
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

		struct Calibrator::Imp 
		{
			struct ConstraintBlock 
			{
				Constraint *c;
				Size ri, rj, col;
			};
			struct ForceBlock 
			{
				Force *f;
				Size ri, rj;
			};
			std::vector<ConstraintBlock> cst_blk_vec_;
			std::vector<ForceBlock> fce_blk_vec_;

			Size dyn_m_, dyn_n_;
			std::vector<double> C_, C_inv_, U_, tau_, Q_, R_, B_, D_, f_;
			std::vector<Size> p_;

			Size m_, g_, k_;
			std::vector<double> A_, x_, b_;
		};
		auto Calibrator::m()->Size { return imp_->m_; }
		auto Calibrator::g()->Size { return imp_->g_; }
		auto Calibrator::k()->Size { return imp_->k_; }
		auto Calibrator::A()->double* { return imp_->A_.data(); }
		auto Calibrator::x()->double* { return imp_->x_.data(); }
		auto Calibrator::b()->double* { return imp_->b_.data(); }
		auto Calibrator::allocateMemory()->void
		{
			imp_->m_ = 0;
			imp_->g_ = 0;
			imp_->k_ = 0;

			for (auto &m : model().motionPool())
			{
				if (m.active())
				{
					imp_->m_ ++;
					imp_->k_ += 3;
				}
			}
			for (auto &p : model().partPool())
			{
				if (p.active())
				{
					imp_->g_ += 10;
				}
			}

			imp_->A_.resize(m()*n());
			imp_->x_.resize(n());
			imp_->b_.resize(m());




			imp_->dyn_m_ = 0;
			imp_->dyn_n_ = 6;
			std::vector<Part*> active_parts;
			for (auto &prt : model().partPool())
			{
				if (prt.active()) 
				{
					active_parts.push_back(&prt);
					imp_->dyn_m_ += 6;
				}
			}
			
			imp_->cst_blk_vec_.clear();
			for (auto &jnt : model().jointPool()) 
			{
				if (jnt.active()) 
				{
					imp_->cst_blk_vec_.push_back(Imp::ConstraintBlock
					{
						&jnt,
						static_cast<Size>((std::find(active_parts.begin(), active_parts.end(), &jnt.makI().fatherPart()) - active_parts.begin()) * 6),
						static_cast<Size>((std::find(active_parts.begin(), active_parts.end(), &jnt.makJ().fatherPart()) - active_parts.begin()) * 6),
						imp_->dyn_n_
					});
					imp_->dyn_n_ += jnt.dim();
				}
			}
			for (auto &mot : model().motionPool()) 
			{
				if (mot.active()) 
				{
					imp_->cst_blk_vec_.push_back(Imp::ConstraintBlock
					{
						&mot,
						static_cast<Size>((std::find(active_parts.begin(), active_parts.end(), &mot.makI().fatherPart()) - active_parts.begin()) * 6),
						static_cast<Size>((std::find(active_parts.begin(), active_parts.end(), &mot.makJ().fatherPart()) - active_parts.begin()) * 6),
						imp_->dyn_n_
					});
					imp_->dyn_n_ += mot.dim();
				}
			}
			for (auto &gmt : model().generalMotionPool()) 
			{
				if (gmt.active())
				{
					imp_->cst_blk_vec_.push_back(Imp::ConstraintBlock
					{
						&gmt,
						static_cast<Size>((std::find(active_parts.begin(), active_parts.end(), &gmt.makI().fatherPart()) - active_parts.begin()) * 6),
						static_cast<Size>((std::find(active_parts.begin(), active_parts.end(), &gmt.makJ().fatherPart()) - active_parts.begin()) * 6),
						imp_->dyn_n_
					});
					imp_->dyn_n_ += 6;
				}
			}

			imp_->fce_blk_vec_.clear();
			for (auto &fce : model().forcePool())
			{
				if (fce.active())
				{
					imp_->fce_blk_vec_.push_back(Imp::ForceBlock
					{
						&fce,
						static_cast<Size>((std::find(active_parts.begin(), active_parts.end(), &fce.makI().fatherPart()) - active_parts.begin()) * 6),
						static_cast<Size>((std::find(active_parts.begin(), active_parts.end(), &fce.makJ().fatherPart()) - active_parts.begin()) * 6)
					});
				}
			}

			imp_->f_.resize(imp_->dyn_m_, 0.0);

			imp_->C_.resize(imp_->dyn_m_*imp_->dyn_n_, 0.0);
			imp_->U_.resize(imp_->dyn_m_*imp_->dyn_n_, 0.0);
			imp_->tau_.resize(std::max(imp_->dyn_m_, imp_->dyn_n_), 0.0);
			imp_->p_.resize(std::max(imp_->dyn_m_, imp_->dyn_n_), 0);

			imp_->C_inv_.resize(imp_->dyn_n_*imp_->dyn_m_, 0.0);
			imp_->R_.resize(imp_->dyn_m_*imp_->dyn_n_, 0.0);
			imp_->Q_.resize(imp_->dyn_m_*imp_->dyn_m_, 0.0);

			imp_->B_.resize(imp_->m_*imp_->dyn_m_, 0.0);
			imp_->D_.resize(imp_->m_*imp_->dyn_m_, 0.0);
		}
		auto Calibrator::clb()->void
		{
			auto f = imp_->f_.data();
			
			auto C = imp_->C_.data();
			auto U = imp_->U_.data();
			auto t = imp_->tau_.data();
			auto p = imp_->p_.data();
			auto Q = imp_->Q_.data();
			auto R = imp_->R_.data();
			auto C_inv = imp_->C_inv_.data();
			
			auto B = imp_->B_.data();
			auto D = imp_->D_.data();

			auto A = imp_->A_.data();
			auto b = imp_->b_.data();
			auto x = imp_->x_.data();

			/////////////////////////////////////////// make A ///////////////////////////////////////
			// make C //
			std::fill_n(C, imp_->dyn_m_*imp_->dyn_n_, 0.0);
			for (int i = 0; i < 6; ++i)
			{
				C[dynamic::id(i, i, imp_->dyn_n_)] = 1;
			}
			for (auto &b : imp_->cst_blk_vec_)
			{
				b.c->cptPrtCm(C + dynamic::id(b.ri, b.col, imp_->dyn_n_), imp_->dyn_n_, C + dynamic::id(b.rj, b.col, imp_->dyn_n_), imp_->dyn_n_);
			}

			// make C inv //
			Size rank;
			s_householder_utp(imp_->dyn_m_, imp_->dyn_n_, C, U, t, p, rank);
			s_householder_ut2qr(imp_->dyn_m_, imp_->dyn_n_, U, t, Q, R);
			s_sov_um(rank, imp_->dyn_m_, R, imp_->dyn_n_, Q, T(imp_->dyn_m_), C_inv, imp_->dyn_m_);
			s_permutate_inv(imp_->dyn_n_, imp_->dyn_m_, p, C_inv);

			// make D //
			Size di = 0;
			for (auto &b : imp_->cst_blk_vec_)
			{
				if (dynamic_cast<Motion*>(b.c))
				{
					s_mc(1,imp_->dyn_m_, C_inv + dynamic::id(b.col, 0, imp_->dyn_m_), D + dynamic::id(di, 0, imp_->dyn_m_));
					di++;
				}
			}

			// make B //
			for (auto &b : imp_->cst_blk_vec_)
			{
				Size row{ 0 };
				for (auto &p : model().partPool())
				{
					double cm[6][6], vs[6];

					s_inv_tv(*p.pm(), p.vs(), vs);
					s_cmf(vs, *cm);
					s_mm(imp_->m_, 6, 6, D + dynamic::id(0, row, imp_->dyn_m_), imp_->dyn_m_, *cm, 6, B + dynamic::id(0, row, imp_->dyn_m_), imp_->dyn_m_);

					row += 6;
				}
			}
			
			// make A //
			int col1 = 0, col2 = 0;
			for (auto &prt : model().partPool())
			{
				if (prt.active())
				{
					double q[6]{ 0 };

					s_inv_tv(*prt.pm(), prt.as(), q);
					s_inv_tva(-1.0, *prt.pm(), model().environment().gravity(), q);

					double v[6];
					s_inv_tv(*prt.pm(), prt.vs(), v);

					for (std::size_t j = 0; j < imp_->m_; ++j)
					{
						A[dynamic::id(j, col1, n())] = D[dynamic::id(j, col2 + 0, imp_->dyn_m_)] * q[0] + D[dynamic::id(j, col2 + 1, imp_->dyn_m_)] * q[1] + D[dynamic::id(j, col2 + 2, imp_->dyn_m_)] * q[2];
						A[dynamic::id(j, col1 + 1, n())] = D[dynamic::id(j, col2 + 1, imp_->dyn_m_)] * q[5] + D[dynamic::id(j, col2 + 5, imp_->dyn_m_)] * q[1] - D[dynamic::id(j, col2 + 2, imp_->dyn_m_)] * q[4] - D[dynamic::id(j, col2 + 4, imp_->dyn_m_)] * q[2];
						A[dynamic::id(j, col1 + 2, n())] = D[dynamic::id(j, col2 + 2, imp_->dyn_m_)] * q[3] + D[dynamic::id(j, col2 + 3, imp_->dyn_m_)] * q[2] - D[dynamic::id(j, col2 + 0, imp_->dyn_m_)] * q[5] - D[dynamic::id(j, col2 + 5, imp_->dyn_m_)] * q[0];
						A[dynamic::id(j, col1 + 3, n())] = D[dynamic::id(j, col2 + 0, imp_->dyn_m_)] * q[4] + D[dynamic::id(j, col2 + 4, imp_->dyn_m_)] * q[0] - D[dynamic::id(j, col2 + 1, imp_->dyn_m_)] * q[3] - D[dynamic::id(j, col2 + 3, imp_->dyn_m_)] * q[1];
						A[dynamic::id(j, col1 + 4, n())] = D[dynamic::id(j, col2 + 3, imp_->dyn_m_)] * q[3];
						A[dynamic::id(j, col1 + 5, n())] = D[dynamic::id(j, col2 + 4, imp_->dyn_m_)] * q[4];
						A[dynamic::id(j, col1 + 6, n())] = D[dynamic::id(j, col2 + 5, imp_->dyn_m_)] * q[5];
						A[dynamic::id(j, col1 + 7, n())] = D[dynamic::id(j, col2 + 3, imp_->dyn_m_)] * q[4] + D[dynamic::id(j, col2 + 4, imp_->dyn_m_)] * q[3];
						A[dynamic::id(j, col1 + 8, n())] = D[dynamic::id(j, col2 + 3, imp_->dyn_m_)] * q[5] + D[dynamic::id(j, col2 + 5, imp_->dyn_m_)] * q[3];
						A[dynamic::id(j, col1 + 9, n())] = D[dynamic::id(j, col2 + 4, imp_->dyn_m_)] * q[5] + D[dynamic::id(j, col2 + 5, imp_->dyn_m_)] * q[4];

						A[dynamic::id(j, col1, n())] += B[dynamic::id(j, col2 + 0, imp_->dyn_m_)] * v[0] + B[dynamic::id(j, col2 + 1, imp_->dyn_m_)] * v[1] + B[dynamic::id(j, col2 + 2, imp_->dyn_m_)] * v[2];
						A[dynamic::id(j, col1 + 1, n())] += B[dynamic::id(j, col2 + 1, imp_->dyn_m_)] * v[5] + B[dynamic::id(j, col2 + 5, imp_->dyn_m_)] * v[1] - B[dynamic::id(j, col2 + 2, imp_->dyn_m_)] * v[4] - B[dynamic::id(j, col2 + 4, imp_->dyn_m_)] * v[2];
						A[dynamic::id(j, col1 + 2, n())] += B[dynamic::id(j, col2 + 2, imp_->dyn_m_)] * v[3] + B[dynamic::id(j, col2 + 3, imp_->dyn_m_)] * v[2] - B[dynamic::id(j, col2 + 0, imp_->dyn_m_)] * v[5] - B[dynamic::id(j, col2 + 5, imp_->dyn_m_)] * v[0];
						A[dynamic::id(j, col1 + 3, n())] += B[dynamic::id(j, col2 + 0, imp_->dyn_m_)] * v[4] + B[dynamic::id(j, col2 + 4, imp_->dyn_m_)] * v[0] - B[dynamic::id(j, col2 + 1, imp_->dyn_m_)] * v[3] - B[dynamic::id(j, col2 + 3, imp_->dyn_m_)] * v[1];
						A[dynamic::id(j, col1 + 4, n())] += B[dynamic::id(j, col2 + 3, imp_->dyn_m_)] * v[3];
						A[dynamic::id(j, col1 + 5, n())] += B[dynamic::id(j, col2 + 4, imp_->dyn_m_)] * v[4];
						A[dynamic::id(j, col1 + 6, n())] += B[dynamic::id(j, col2 + 5, imp_->dyn_m_)] * v[5];
						A[dynamic::id(j, col1 + 7, n())] += B[dynamic::id(j, col2 + 3, imp_->dyn_m_)] * v[4] + B[dynamic::id(j, col2 + 4, imp_->dyn_m_)] * v[3];
						A[dynamic::id(j, col1 + 8, n())] += B[dynamic::id(j, col2 + 3, imp_->dyn_m_)] * v[5] + B[dynamic::id(j, col2 + 5, imp_->dyn_m_)] * v[3];
						A[dynamic::id(j, col1 + 9, n())] += B[dynamic::id(j, col2 + 4, imp_->dyn_m_)] * v[5] + B[dynamic::id(j, col2 + 5, imp_->dyn_m_)] * v[4];
					}
					col1 += 10;
					col2 += 6;
				}
			}
			// make A from frictions //
			Size ai = 0;
			for (auto &b : imp_->cst_blk_vec_)
			{
				if (dynamic_cast<Motion*>(b.c))
				{
					A[dynamic::id(ai, g() + ai * 3, n())] = s_sgn(dynamic_cast<Motion*>(b.c)->mv());
					A[dynamic::id(ai, g() + ai * 3 + 1, n())] = dynamic_cast<Motion*>(b.c)->mv();
					A[dynamic::id(ai, g() + ai * 3 + 2, n())] = dynamic_cast<Motion*>(b.c)->ma();
					ai++;
				}

			}


			/////////////////////////////////////////// make b ///////////////////////////////////////
			Size bi{ 0 };
			for (auto &blk : imp_->cst_blk_vec_)
			{
				if (dynamic_cast<Motion*>(blk.c))
				{
					b[bi] = dynamic_cast<Motion*>(blk.c)->mf();
					bi++;
				}
			}
			std::fill(f, f + imp_->dyn_m_, 0.0);
			for (auto &fce : imp_->fce_blk_vec_)
			{
				double glb_fsI[6], glb_fsJ[6];
				fce.f->cptGlbFs(glb_fsI, glb_fsJ);

				double prt_fsI[6], prt_fsJ[6];
				s_inv_tf(*fce.f->makI().fatherPart().pm(), glb_fsI, prt_fsI);
				s_inv_tf(*fce.f->makJ().fatherPart().pm(), glb_fsJ, prt_fsJ);

				s_va(6, prt_fsI, f + fce.ri);
				s_va(6, prt_fsJ, f + fce.rj);
			}

			s_mma(m(), 1, imp_->dyn_m_, D, f, b);

			/////////////////////////////////////////// make x ///////////////////////////////////////
			Size xi = 0;
			for (auto &prt : model().partPool())
			{
				if (prt.active())
				{
					s_vc(10, prt.prtIv(), x + xi);
					xi += 10;
				}
			}
			// make x from frictions //
			bi = 0;
			for (auto &blk : imp_->cst_blk_vec_)
			{
				if (dynamic_cast<Motion*>(blk.c))
				{
					s_vc(3, dynamic_cast<Motion*>(blk.c)->frcCoe(), x + g() + bi * 3);
					bi++;
				}
			}
		}

		
		
		
		Calibrator::~Calibrator() = default;
		Calibrator::Calibrator(const std::string &name) : Element(name), imp_(new Imp) {}
		Calibrator::Calibrator(const Calibrator&) = default;
		Calibrator::Calibrator(Calibrator&&) = default;
		Calibrator& Calibrator::operator=(const Calibrator&) = default;
		Calibrator& Calibrator::operator=(Calibrator&&) = default;

		struct SimResult::TimeResult::Imp { std::deque<double> time_; };
		auto SimResult::TimeResult::saveXml(aris::core::XmlElement &xml_ele)const->void
		{
			Element::saveXml(xml_ele);

			std::stringstream ss;
			ss << std::setprecision(15);
			ss.str().reserve((25 * 1 + 1)*imp_->time_.size());

			for (auto &t : imp_->time_)ss << t << std::endl;

			xml_ele.SetText(ss.str().c_str());
		}
		auto SimResult::TimeResult::loadXml(const aris::core::XmlElement &xml_ele)->void
		{
			// 以下导入数据 //
			std::stringstream ss(std::string(xml_ele.GetText()));
			for (double t; ss >> t, !ss.eof(); imp_->time_.push_back(t));

			Element::loadXml(xml_ele);
		}
		auto SimResult::TimeResult::record()->void { imp_->time_.push_back(model().time()); }
		auto SimResult::TimeResult::restore(Size pos)->void { model().setTime(imp_->time_.at(pos)); }
		SimResult::TimeResult::~TimeResult() = default;
		SimResult::TimeResult::TimeResult(const std::string &name) : Element(name), imp_(new Imp) {}
		SimResult::TimeResult::TimeResult(const SimResult::TimeResult&) = default;
		SimResult::TimeResult::TimeResult(SimResult::TimeResult&&) = default;
		SimResult::TimeResult& SimResult::TimeResult::operator=(const TimeResult&) = default;
		SimResult::TimeResult& SimResult::TimeResult::operator=(TimeResult&&) = default;

		struct SimResult::PartResult::Imp
		{
			Part *part_;
			std::deque<std::array<double, 6> > pe_;
			std::deque<std::array<double, 6> > vs_;
			std::deque<std::array<double, 6> > as_;

			Imp(Part* part) :part_(part) {};
		};
		auto SimResult::PartResult::saveXml(aris::core::XmlElement &xml_ele)const->void
		{
			Element::saveXml(xml_ele);

			xml_ele.SetAttribute("part", part().name().c_str());
			std::stringstream ss;
			ss << std::setprecision(15);
			ss.str().reserve((25 * 18 + 1)*imp_->pe_.size());

			for (auto pe = imp_->pe_.begin(), vs = imp_->vs_.begin(), as = imp_->as_.begin(); pe < imp_->pe_.end(); ++pe, ++vs, ++as)
			{
				for (auto e : *pe) ss << e << " ";
				for (auto e : *vs)ss << e << " ";
				for (auto e : *as)ss << e << " ";
				ss << std::endl;
			}

			xml_ele.SetText(ss.str().c_str());
		}
		auto SimResult::PartResult::loadXml(const aris::core::XmlElement &xml_ele)->void
		{
			// 以下寻找对应的part //
			if (model().findByName("part_pool") == model().children().end())
				throw std::runtime_error("you must insert \"part_pool\" node before insert " + type() + " \"" + name() + "\"");

			auto &part_pool = static_cast<aris::core::ObjectPool<Part, Element>&>(*model().findByName("part_pool"));

			if (!xml_ele.Attribute("part"))throw std::runtime_error(std::string("xml element \"") + name() + "\" must have Attribute \"part\"");
			auto p = part_pool.findByName(xml_ele.Attribute("part"));
			if (p == part_pool.end())	throw std::runtime_error(std::string("can't find part for PartResult \"") + this->name() + "\"");

			imp_->part_ = &*p;

			// 以下导入数据 //
			std::stringstream ss(std::string(xml_ele.GetText()));
			std::array<double, 6> pe, vs, as;
			for (Size i{ 0 }; !ss.eof(); ++i)
			{
				if (i < 6) ss >> pe[i];
				else if (i < 12) ss >> vs[i - 6];
				else if (i < 18) ss >> as[i - 12];

				if (i == 6)imp_->pe_.push_back(pe);
				if (i == 12)imp_->vs_.push_back(vs);
				if (i == 18) { imp_->as_.push_back(as); i = -1; }
			}

			Element::loadXml(xml_ele);
		}
		auto SimResult::PartResult::part()->Part& { return *imp_->part_; }
		auto SimResult::PartResult::record()->void
		{
			std::array<double, 6> result;
			s_pm2pe(*part().pm(), result.data());
			imp_->pe_.push_back(result);
			std::copy(static_cast<const double*>(part().vs()), static_cast<const double*>(part().vs()) + 6, result.data());
			imp_->vs_.push_back(result);
			std::copy(static_cast<const double*>(part().as()), static_cast<const double*>(part().as()) + 6, result.data());
			imp_->as_.push_back(result);
		}
		auto SimResult::PartResult::restore(Size pos)->void
		{
			part().setPe(imp_->pe_.at(pos).data());
			part().setVs(imp_->vs_.at(pos).data());
			part().setAs(imp_->as_.at(pos).data());
		}
		SimResult::PartResult::~PartResult() = default;
		SimResult::PartResult::PartResult(const std::string &name, Part *part) : Element(name), imp_(new Imp(part)) {}
		SimResult::PartResult::PartResult(const SimResult::PartResult&) = default;
		SimResult::PartResult::PartResult(SimResult::PartResult&&) = default;
		SimResult::PartResult& SimResult::PartResult::operator=(const PartResult&) = default;
		SimResult::PartResult& SimResult::PartResult::operator=(PartResult&&) = default;

		struct SimResult::ConstraintResult::Imp
		{
			Constraint *constraint_;
			std::deque<std::array<double, 6> > cf_;

			Imp(Constraint* constraint) :constraint_(constraint) {};
		};
		auto SimResult::ConstraintResult::saveXml(aris::core::XmlElement &xml_ele)const->void
		{
			Element::saveXml(xml_ele);

			xml_ele.SetAttribute("constraint", constraint().name().c_str());

			std::stringstream ss;
			ss << std::setprecision(15);
			ss.str().reserve((25 * 6 + 1)*imp_->cf_.size());
			for (auto &cf : imp_->cf_)
			{
				for (Size i(-1); ++i < constraint().dim();) ss << cf[i] << " ";
				ss << std::endl;
			}

			xml_ele.SetText(ss.str().c_str());
		}
		auto SimResult::ConstraintResult::loadXml(const aris::core::XmlElement &xml_ele)->void
		{
			// 以下寻找对应的constraint //
			if (!xml_ele.Attribute("constraint"))throw std::runtime_error(std::string("xml element \"") + name() + "\" must have Attribute \"constraint\"");
			if (!imp_->constraint_ && model().findByName("joint_pool") != model().children().end())
			{
				auto &pool = static_cast<aris::core::ObjectPool<Joint, Element>&>(*model().findByName("joint_pool"));
				auto c = pool.findByName(xml_ele.Attribute("constraint"));
				if (c != pool.end())imp_->constraint_ = &*c;
			}
			if (!imp_->constraint_ && model().findByName("motion_pool") != model().children().end())
			{
				auto &pool = static_cast<aris::core::ObjectPool<Motion, Element>&>(*model().findByName("motion_pool"));
				auto c = pool.findByName(xml_ele.Attribute("constraint"));
				if (c != pool.end())imp_->constraint_ = &*c;
			}
			if (!imp_->constraint_ && model().findByName("general_motion_pool") != model().children().end())
			{
				auto &pool = static_cast<aris::core::ObjectPool<GeneralMotion, Element>&>(*model().findByName("general_motion_pool"));
				auto c = pool.findByName(xml_ele.Attribute("constraint"));
				if (c != pool.end())imp_->constraint_ = &*c;
			}
			if (!imp_->constraint_)throw std::runtime_error(std::string("can't find constraint for ConstraintResult \"") + this->name() + "\"");

			// 以下读取数据 //
			std::stringstream ss(std::string(xml_ele.GetText()));
			std::array<double, 6> cf{ 0,0,0,0,0,0 };
			for (Size i{ 0 }; !ss.eof(); ss >> cf[i++])
			{
				if (i == constraint().dim())
				{
					i = 0;
					imp_->cf_.push_back(cf);
				}
			}

			Element::loadXml(xml_ele);
		}
		auto SimResult::ConstraintResult::constraint()->Constraint& { return *imp_->constraint_; }
		auto SimResult::ConstraintResult::record()->void
		{
			std::array<double, 6> result{ 0,0,0,0,0,0 };
			std::copy(constraint().cf(), constraint().cf() + constraint().dim(), result.data());
			imp_->cf_.push_back(result);
		}
		auto SimResult::ConstraintResult::restore(Size pos)->void 
		{ 
			constraint().setCf(imp_->cf_.at(pos).data());
			if (dynamic_cast<Motion*>(&constraint()))
			{
				dynamic_cast<Motion*>(&constraint())->updMp();
				dynamic_cast<Motion*>(&constraint())->updMv();
				dynamic_cast<Motion*>(&constraint())->updMa();
			}
			if (dynamic_cast<GeneralMotion*>(&constraint()))
			{
				dynamic_cast<GeneralMotion*>(&constraint())->updMpm();
				dynamic_cast<GeneralMotion*>(&constraint())->updMvs();
				dynamic_cast<GeneralMotion*>(&constraint())->updMas();
			}
		}
		SimResult::ConstraintResult::~ConstraintResult() = default;
		SimResult::ConstraintResult::ConstraintResult(const std::string &name, Constraint *constraint) : Element(name), imp_(new Imp(constraint)) {}
		SimResult::ConstraintResult::ConstraintResult(const SimResult::ConstraintResult&) = default;
		SimResult::ConstraintResult::ConstraintResult(SimResult::ConstraintResult&&) = default;
		SimResult::ConstraintResult& SimResult::ConstraintResult::operator=(const ConstraintResult&) = default;
		SimResult::ConstraintResult& SimResult::ConstraintResult::operator=(ConstraintResult&&) = default;

		struct SimResult::Imp
		{
			TimeResult *time_result_;
			aris::core::ObjectPool<PartResult, Element> *part_result_pool_;
			aris::core::ObjectPool<ConstraintResult, Element> *constraint_result_pool_;
		};
		auto SimResult::loadXml(const aris::core::XmlElement &xml_ele)->void
		{
			Element::loadXml(xml_ele);

			imp_->time_result_ = findOrInsert<TimeResult>("time_result");
			imp_->constraint_result_pool_ = findOrInsert<aris::core::ObjectPool<SimResult::ConstraintResult, Element> >("constraint_result_pool");
			imp_->part_result_pool_ = findOrInsert<aris::core::ObjectPool<SimResult::PartResult, Element> >("part_result_pool");

		}
		auto SimResult::timeResult()->TimeResult& { return *imp_->time_result_; }
		auto SimResult::partResultPool()->aris::core::ObjectPool<SimResult::PartResult, Element>& { return *imp_->part_result_pool_; }
		auto SimResult::constraintResultPool()->aris::core::ObjectPool<SimResult::ConstraintResult, Element>& { return *imp_->constraint_result_pool_; }
		auto SimResult::allocateMemory()->void
		{
			partResultPool().clear();
			for (auto &p : model().partPool())partResultPool().add<PartResult>(p.name() + "_result", &p);
			constraintResultPool().clear();
			for (auto &c : model().jointPool())constraintResultPool().add<ConstraintResult>(c.name() + "_result", &c);
			for (auto &c : model().motionPool())constraintResultPool().add<ConstraintResult>(c.name() + "_result", &c);
			for (auto &c : model().generalMotionPool())constraintResultPool().add<ConstraintResult>(c.name() + "_result", &c);
		}
		auto SimResult::record()->void
		{
			timeResult().record();
			for (auto &p : partResultPool())p.record();
			for (auto &p : constraintResultPool())p.record();
		}
		auto SimResult::restore(Size pos)->void
		{
			timeResult().restore(pos);
			for (auto &r : partResultPool())r.restore(pos);
			for (auto &r : constraintResultPool())r.restore(pos);
		}
		auto SimResult::size()const->Size { return timeResult().imp_->time_.size() == 0 ? 0 : timeResult().imp_->time_.size() - 1; }
		auto SimResult::clear()->void
		{
			timeResult().imp_->time_.clear();
			for (auto &r : partResultPool())
			{
				r.imp_->pe_.clear();
				r.imp_->vs_.clear();
				r.imp_->as_.clear();
			}
			for (auto &r : constraintResultPool())r.imp_->cf_.clear();
		}
		SimResult::~SimResult() = default;
		SimResult::SimResult(const std::string &name) : Element(name), imp_(new Imp())
		{
			registerType<aris::core::ObjectPool<SimResult::PartResult, Element>>();
			registerType<aris::core::ObjectPool<SimResult::ConstraintResult, Element>>();
			registerType<SimResult::PartResult>();
			registerType<SimResult::ConstraintResult>();
			registerType<SimResult::TimeResult>();
			
			imp_->time_result_ = &add<TimeResult>("time_result");
			imp_->part_result_pool_ = &add<aris::core::ObjectPool<SimResult::PartResult, Element> >("part_result_pool");
			imp_->constraint_result_pool_ = &add<aris::core::ObjectPool<SimResult::ConstraintResult, Element> >("constraint_result_pool");
		}
		SimResult::SimResult(const SimResult&other) : Element(other), imp_(other.imp_)
		{
			imp_->time_result_ = findType<TimeResult >("time_result");
			imp_->constraint_result_pool_ = findType<aris::core::ObjectPool<SimResult::ConstraintResult, Element> >("constraint_result_pool");
			imp_->part_result_pool_ = findType<aris::core::ObjectPool<SimResult::PartResult, Element> >("part_result_pool");
		}
		SimResult::SimResult(SimResult&&other) : Element(std::move(other)), imp_(std::move(other.imp_))
		{
			imp_->time_result_ = findType<TimeResult >("time_result");
			imp_->constraint_result_pool_ = findType<aris::core::ObjectPool<SimResult::ConstraintResult, Element> >("constraint_result_pool");
			imp_->part_result_pool_ = findType<aris::core::ObjectPool<SimResult::PartResult, Element> >("part_result_pool");
		}
		SimResult& SimResult::operator=(const SimResult&other)
		{
			Element::operator=(other);
			imp_ = other.imp_;
			imp_->time_result_ = findType<TimeResult >("time_result");
			imp_->constraint_result_pool_ = findType<aris::core::ObjectPool<SimResult::ConstraintResult, Element> >("constraint_result_pool");
			imp_->part_result_pool_ = findType<aris::core::ObjectPool<SimResult::PartResult, Element> >("part_result_pool");
			return *this;
		}
		SimResult& SimResult::operator=(SimResult&&other)
		{
			Element::operator=(std::move(other));
			imp_ = other.imp_;
			imp_->time_result_ = findType<TimeResult >("time_result");
			imp_->constraint_result_pool_ = findType<aris::core::ObjectPool<SimResult::ConstraintResult, Element> >("constraint_result_pool");
			imp_->part_result_pool_ = findType<aris::core::ObjectPool<SimResult::PartResult, Element> >("part_result_pool");
			return *this;
		}

		struct Simulator::Imp {};
		auto Simulator::simulate(const PlanFunction &plan, void *param, std::uint32_t param_size, SimResult &result)->void
		{
			result.allocateMemory();
			// 记录初始状态 //
			result.record();
			// 记录轨迹中的状态 //
			for (PlanParam plan_param{ &model(), std::uint32_t(1), param, param_size }; plan(plan_param) != 0; ++plan_param.count_)result.record();
			// 记录结束状态 //
			result.record();
		}
		Simulator::~Simulator() = default;
		Simulator::Simulator(const std::string &name) : Element(name), imp_(new Imp) {}
		Simulator::Simulator(const Simulator&) = default;
		Simulator::Simulator(Simulator&&) = default;
		Simulator& Simulator::operator=(const Simulator&) = default;
		Simulator& Simulator::operator=(Simulator&&) = default;

		struct UniversalSolver::Imp
		{
			// 动力学计算以下变量的关系
			// I  ： 惯量矩阵,m*m
			// C  ： 约束矩阵,m*n
			// pa ： 杆件的螺旋加速度 m*1
			// pf ： 杆件的螺旋外力（不包括惯性力）m*1
			// ca ： 约束的加速度（不是螺旋）n*1
			// cf ： 约束力n*1
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
			// 约束矩阵C为m x n维的矩阵,惯量矩阵为m x m维的矩阵
			// 约束力为n维的向量,约束加速度为n维向量
			// 部件力为m维的向量,部件加速度为m维向量
			// 动力学为所求的未知量为部件加速度和约束力,其他均为已知
			// 动力学主要求解以下方程：
			// [  I  C  ]  *  [ as ]  = [ fs ]
			// [  C' O  ]     [ cf ]    [ ca ]
			//
			// A = [ I  C ]
			//     [ C' O ]
			//
			// x = [ as ]
			//     [ cf ]
			//
			// b = [ fs ]
			//     [ ca ]
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
				auto updDiagIm()->void;
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

			std::vector<double> general_jacobi_;

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
		auto UniversalSolver::Imp::SubSystem::rowAddInverseXp()->void { for (auto d = diag_pool_.begin() + 1; d<diag_pool_.end(); ++d)s_va(6, d->rd->xp, d->xp); }
		auto UniversalSolver::Imp::SubSystem::rowAddBp()->void { for (auto d = diag_pool_.rbegin(); d < diag_pool_.rend() - 1; ++d) s_va(6, d->bp, d->rd->bp); }
		auto UniversalSolver::Imp::SubSystem::updDiagDm()->void { for (auto d = diag_pool_.begin() + 1; d < diag_pool_.end(); ++d)d->upd_d(&*d); }
		auto UniversalSolver::Imp::SubSystem::updDiagIm()->void 
		{ 
			if (!hasGround())s_iv2iv(*diag_pool_.begin()->part->pm(), diag_pool_.begin()->part->prtIv(), diag_pool_.begin()->iv);
			for (auto d = diag_pool_.begin() + 1; d < diag_pool_.end(); ++d)s_iv2iv(*d->part->pm(), d->part->prtIv(), d->iv);
		}
		auto UniversalSolver::Imp::SubSystem::updF()->void
		{
			s_fill(fm, fn, 0.0, F);
			
			Size cols{ 0 };
			for (auto &r : remainder_pool_)
			{
				for (auto &b : r.cm_blk_series)
				{
					s_mm(6 - b.diag->rel_.dim, r.rel_.dim, 6, b.diag->dm + dynamic::id(b.diag->rel_.dim, 0, 6), 6, b.is_I ? r.cmI : r.cmJ, r.rel_.dim, F + dynamic::id(b.diag->rows, cols, fn), fn);
				}
				cols += r.rel_.dim;
			}
		}
		auto UniversalSolver::Imp::SubSystem::updXpf()->void
		{
			s_householder_utp(fn, fm, F, ColMajor{ fn }, FU, ColMajor{ fn }, FT, 1, FP, fr, max_error_);
			s_householder_utp_sov(fn, fm, 1, fr, FU, ColMajor{ fn }, FT, 1, FP, bcf, 1, xpf, 1, max_error_);
		}
		auto UniversalSolver::Imp::SubSystem::updXcf()->void
		{
			s_householder_utp(fm, fn, F, FU, FT, FP, fr, max_error_);
			s_householder_utp_sov(fm, fn, 1, fr, FU, FT, FP, bpf, xcf, max_error_);
		}
		auto UniversalSolver::Imp::SubSystem::updBpf()->void
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
		auto UniversalSolver::Imp::SubSystem::updBcf()->void
		{
			// 使用已求出的未知数，用以构建b
			Size cols{ 0 };
			for (auto &r : remainder_pool_)
			{
				double bb[6]{ 0,0,0,0,0,0 };

				for (auto &b : r.cm_blk_series)
				{
					double tem[6];
					auto cm = b.is_I ? r.cmJ : r.cmI;//这里是颠倒的，因为加到右侧需要乘-1.0

					s_mm(6, 1, b.diag->rel_.dim, b.diag->dm, ColMajor{ 6 }, b.diag->bc, 1, tem, 1);
					s_mma(r.rel_.dim, 1, 6, cm, ColMajor{ r.rel_.dim }, tem, 1, r.bc, 1);
				}
				s_vc(r.rel_.dim, r.bc, bcf + cols);
				cols += r.rel_.dim;
			}
		}
		auto UniversalSolver::Imp::SubSystem::updXp()->void
		{
			if(!hasGround())s_vc(6, xpf + diag_pool_.begin()->rows, diag_pool_.begin()->xp);
			for (auto d = diag_pool_.begin() + 1; d<diag_pool_.end(); ++d)
			{
				double tem[6];
				s_vc(d->rel_.dim, d->bc, tem);
				s_vc(6 - d->rel_.dim, xpf + d->rows, tem + d->rel_.dim);

				s_mm(6, 1, 6, d->dm, ColMajor{ 6 }, tem, 1, d->xp, 1);
			}
		}
		auto UniversalSolver::Imp::SubSystem::updXc()->void
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
		auto UniversalSolver::Imp::SubSystem::updCf()->void
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
		auto UniversalSolver::Imp::SubSystem::updPp()->void
		{
			auto beg = hasGround() ? diag_pool_.begin() + 1 : diag_pool_.begin();
			for (auto d = beg; d<diag_pool_.end(); ++d)
			{
				std::swap(d->pm, d->last_pm);
				
				double pm[4][4];
				s_ps2pm(d->xp, *pm);
				double final_pm[4][4];
				s_pm2pm(*pm, d->last_pm, *final_pm);

				s_vc(16, *final_pm, d->pm);
			}
		}
		auto UniversalSolver::Imp::SubSystem::updPv()->void
		{
			auto beg = hasGround() ? diag_pool_.begin() + 1 : diag_pool_.begin();
			for (auto d = beg; d<diag_pool_.end(); ++d)s_va(6, d->xp, const_cast<double*>(d->part->vs()));
		}
		auto UniversalSolver::Imp::SubSystem::updPa()->void
		{
			auto beg = hasGround() ? diag_pool_.begin() + 1 : diag_pool_.begin();
			for (auto d = beg; d<diag_pool_.end(); ++d)s_vc(6, d->xp, const_cast<double*>(d->part->as()));
		}
		auto UniversalSolver::Imp::SubSystem::updCpToBc()->void
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
		auto UniversalSolver::Imp::SubSystem::updCvToBc()->void
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
		auto UniversalSolver::Imp::SubSystem::updCaToBc()->void
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
		auto UniversalSolver::Imp::SubSystem::updPfToBp()->void
		{
			auto beg = hasGround() ? diag_pool_.begin() + 1 : diag_pool_.begin();
			for (auto d = beg; d < diag_pool_.end(); ++d)
			{
				// I*(a-g) //
				double as_minus_g[6];
				s_vc(6, d->xp, as_minus_g);
				s_vs(6, d->part->model().environment().gravity(), d->xp);
				s_iv_dot_as(d->iv, d->xp, d->bp);

				// v x I * v //
				double I_dot_v[6];
				s_iv_dot_as(d->iv, d->part->vs(), I_dot_v);
				s_cfa(d->part->vs(), I_dot_v, d->bp);

				// 外力，储存在as中//
				s_vs(6, d->part->as(), d->bp);
			}
		}
		auto UniversalSolver::Imp::SubSystem::updRemainderCm()->void
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
					//c.constraint->cptGlbCm(r.cmI + pos, r.rel_.dim, r.cmJ + pos, r.rel_.dim);
					pos += c.constraint->dim();
				}
			}
		}
		auto UniversalSolver::Imp::SubSystem::kinPos()->void
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
		auto UniversalSolver::Imp::SubSystem::kinVel()->void
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
		auto UniversalSolver::Imp::SubSystem::dynAccAndFce()->void
		{
			// [I   C] * [ pa ] = [ pf ]
			// [C^T  ]   [ cf ]   [ ca ]

			// upd Im //
			updDiagIm();

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
				// 这里的R为：
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
				xpf[fr + j] = 1.0;
				s_householder_ut_q_dot(fm, fn, 1, FU, fn, FT, 1, xpf, 1, S + j, m_mimus_r);
				xpf[fr + j] = 0.0;

				// 乘以DT
				if (!hasGround())s_vc(6, S + dynamic::id(diag_pool_.begin()->rows, j, m_mimus_r), m_mimus_r, diag_pool_.begin()->xp, 1);
				for (auto d = diag_pool_.begin() + 1; d<diag_pool_.end(); ++d)
				{
					//////////////////////// 可以优化 //////////////////
					double tem[6]{ 0,0,0,0,0,0 };
					s_vc(6 - d->rel_.dim, S + dynamic::id(d->rows, j, m_mimus_r), m_mimus_r, tem + d->rel_.dim, 1);
					s_mm(6, 1, 6, d->dm, ColMajor{ 6 }, tem, 1, d->xp, 1);
				}

				// 乘以PT
				rowAddInverseXp();

				// 乘以I
				if (!hasGround())s_iv_dot_as(diag_pool_.begin()->iv, diag_pool_.begin()->xp, diag_pool_.begin()->bp);
				for (auto d = diag_pool_.begin() + 1; d < diag_pool_.end(); ++d)s_iv_dot_as(d->iv, d->xp, d->bp);

				// 乘以P
				rowAddBp();

				// 乘以D，并取出来
				for (auto d = diag_pool_.rbegin(); d<diag_pool_.rend() - 1; ++d)
				{
					/////////// 可以优化
					double tem[6];
					s_mm(6, 1, 6, d->dm, 6, d->bp, 1, tem, 1);
					s_mc(6 - d->rel_.dim, 1, tem + d->rel_.dim, 1, G + dynamic::id(d->rows, j, m_mimus_r), m_mimus_r);
				}
				if (!hasGround())s_vc(6, diag_pool_.begin()->bp, 1, G + dynamic::id(diag_pool_.begin()->rows, j, m_mimus_r), m_mimus_r);
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
			s_householder_utp(m_mimus_r, m_mimus_r, QT_DOT_G + dynamic::id(fr, 0, m_mimus_r), GU, GT, GP, rank, max_error_);
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
			for (auto &gmt : model().generalMotionPool())if (gmt.active())cp.push_back( &gmt);
			
			// 制作包含所有relation的pool //
			std::vector<Imp::Relation> relation_pool;
			relation_pool.clear();
			for (auto c : cp)
			{
				auto ret = std::find_if(relation_pool.begin(), relation_pool.end(), [&c](Imp::Relation &relation)
				{
					const auto ri{ relation.prtI }, rj{ relation.prtJ }, ci{ &c->makI().fatherPart() }, cj{ &c->makJ().fatherPart() };
					return ((ri == ci) && (rj == cj)) || ((ri == cj) && (rj == ci));
				});

				if (ret == relation_pool.end())
				{
					relation_pool.push_back(Imp::Relation{ &c->makI().fatherPart(), &c->makJ().fatherPart(), c->dim(),{ { c, true } } });
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
				Imp::SubSystem::addPart(part_pool_pool_.back(), active_part_pool, relation_pool, active_part_pool.at(1));
			}
			
			imp_->subsys_pool_.clear();
			// 分配子系统的内存 //
			Size max_fm{ 0 }, max_fn{ 0 };
			for (auto &part_pool : part_pool_pool_)
			{
				imp_->subsys_pool_.push_back(Imp::SubSystem());
				auto &sys = imp_->subsys_pool_.back();

				// 寻找本系统的relation_pool
				std::vector<Imp::Relation> sys_relation_pool;
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
					std::sort(sys_relation_pool.begin() + i, sys_relation_pool.end(), [i, this, &sys, &part_pool](Imp::Relation a, Imp::Relation b)
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
					diag.rd = &*std::find_if(sys.diag_pool_.begin(), sys.diag_pool_.end(), [&](Imp::Diag &d) {return d.part == diag.rel_.prtJ; });
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
					r.i_diag = &*std::find_if(sys.diag_pool_.begin(), sys.diag_pool_.end(), [&r](Imp::Diag& d) {return r.rel_.prtI == d.part; });
					r.j_diag = &*std::find_if(sys.diag_pool_.begin(), sys.diag_pool_.end(), [&r](Imp::Diag& d) {return r.rel_.prtJ == d.part; });
					r.cm_blk_series.clear();
					r.cm_blk_series.push_back(Imp::Remainder::Block());
					r.cm_blk_series.back().diag = &*std::find_if(sys.diag_pool_.begin(), sys.diag_pool_.end(), [&r](Imp::Diag& d) {return r.rel_.prtI == d.part; });
					r.cm_blk_series.back().is_I = true;
					r.cm_blk_series.push_back(Imp::Remainder::Block());
					r.cm_blk_series.back().diag = &*std::find_if(sys.diag_pool_.begin(), sys.diag_pool_.end(), [&r](Imp::Diag& d) {return r.rel_.prtJ == d.part; });
					r.cm_blk_series.back().is_I = false;

					for (auto rd = sys.diag_pool_.rbegin(); rd < sys.diag_pool_.rend() - 1; ++rd)
					{
						auto &d = *rd;

						auto diag_part = d.rel_.prtI;
						auto add_part = d.rel_.prtJ;

						// 判断当前remainder加法元素是否存在（不为0）
						auto diag_blk = std::find_if(r.cm_blk_series.begin(), r.cm_blk_series.end(), [&](Imp::Remainder::Block &blk) {return blk.diag->part == diag_part; });
						auto add_blk = std::find_if(r.cm_blk_series.begin(), r.cm_blk_series.end(), [&](Imp::Remainder::Block &blk) {return blk.diag->part == add_part; });
						if (diag_blk != r.cm_blk_series.end())
						{
							if (add_blk != r.cm_blk_series.end())
							{
								r.cm_blk_series.erase(add_blk);
							}
							else
							{
								Imp::Remainder::Block blk;
								blk.is_I = diag_blk->is_I;

								blk.diag = &*std::find_if(sys.diag_pool_.begin(), sys.diag_pool_.end(), [&](Imp::Diag &d) {return d.part == add_part; });

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
			imp_->general_jacobi_.resize(model().partPool().size() * 6 * (model().motionPool().size() + model().generalMotionPool().size() * 6));
		}
		auto UniversalSolver::kinPos()->bool
		{
			// 储存位置信息，以便迭代失败后恢复 //
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
			
			for (auto &prt : model().partPool())std::fill_n(const_cast<double*>(prt.as()), 6, 0.0);

			// 更新外力，这里先将他们保存到as中 //
			for (auto &fce : model().forcePool())
			{
				if (fce.active())
				{
					double fsI[6], fsJ[6];
					fce.cptGlbFs(fsI, fsJ);

					s_va(6, fsI, const_cast<double*>(fce.makI().fatherPart().as()));
					s_va(6, fsJ, const_cast<double*>(fce.makJ().fatherPart().as()));
				}
			}

			// 更新地面的as //
			s_fill(6, 1, 0.0, const_cast<double *>(model().ground().as()));
			for (auto &sys : imp_->subsys_pool_) sys.dynAccAndFce();
		}
		auto UniversalSolver::cptGeneralJacobi()->void
		{
			std::fill(imp_->general_jacobi_.begin(), imp_->general_jacobi_.end(), 0.0);
			
			for (auto &sys : imp_->subsys_pool_)
			{
				// make A
				sys.updDiagDm();
				sys.updRemainderCm();

				// solve
				sys.updF();

				for (auto &d : sys.diag_pool_) std::fill(d.bc, d.bc + 6, 0.0);
				for (auto &r : sys.remainder_pool_) std::fill(r.bc, r.bc + 6, 0.0);
				Size n = model().motionPool().size() + model().generalMotionPool().size() * 6;
				
				for (auto &d : sys.diag_pool_)
				{
					Size pos = 0;
					for (auto &c : d.rel_.cst_pool_)
					{
						if (dynamic_cast<Motion*>(c.constraint))
						{
							// 更新bc，将当前电机的未知量更新为当前c的1.0 //
							d.bc[pos] = 1.0;

							sys.updBcf();
							sys.updXpf();
							sys.updXp();
							sys.rowAddInverseXp();
							
							auto beg = sys.hasGround() ? sys.diag_pool_.begin() + 1 : sys.diag_pool_.begin();
							for (auto d = beg; d < sys.diag_pool_.end(); ++d) 
							{
								s_vc(6, d->xp, 1, &imp_->general_jacobi_.at(aris::dynamic::id(d->part->id() * 6, c.constraint->id(), n)), n);
							}

							std::fill(d.bc, d.bc + 6, 0.0);
						}
						else if (dynamic_cast<GeneralMotion*>(c.constraint))
						{
							double tmf[6][6];
							s_tmf(*dynamic_cast<GeneralMotion*>(c.constraint)->makI().pm(), *tmf);
							
							for (Size k(-1); ++k < 6;)
							{
								// 更新bc，将当前电机的未知量更新为当前c的1.0 //
								// Tmf^(T) * v //
								s_vc(6, tmf[k], d.bc);

								sys.updBcf();
								sys.updXpf();
								sys.updXp();
								sys.rowAddInverseXp();

								auto beg = sys.hasGround() ? sys.diag_pool_.begin() + 1 : sys.diag_pool_.begin();
								for (auto d = beg; d < sys.diag_pool_.end(); ++d)
								{
									s_vc(6, d->xp, 1, &imp_->general_jacobi_.at(aris::dynamic::id(d->part->id() * 6, model().motionPool().size() + c.constraint->id() + k, n)), n);
								}
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

							sys.updBcf();
							sys.updXpf();
							sys.updXp();
							sys.rowAddInverseXp();

							auto beg = sys.hasGround() ? sys.diag_pool_.begin() + 1 : sys.diag_pool_.begin();
							for (auto d = beg; d < sys.diag_pool_.end(); ++d)
							{
								s_vc(6, d->xp, 1, &imp_->general_jacobi_.at(aris::dynamic::id(d->part->id() * 6, c.constraint->id(), n)), n);
							}

							std::fill(r.bc, r.bc + 6, 0.0);
						}
						else if (dynamic_cast<GeneralMotion*>(c.constraint))
						{
							double tmf[6][6];
							s_tmf(*dynamic_cast<GeneralMotion*>(c.constraint)->makI().pm(), *tmf);

							for (Size k(-1); ++k < 6;)
							{
								// 更新bc，将当前电机的未知量更新为当前c的1.0 //
								// Tmf^(T) * v //
								s_vc(6, tmf[k], r.bc);

								sys.updBcf();
								sys.updXpf();
								sys.updXp();
								sys.rowAddInverseXp();

								auto beg = sys.hasGround() ? sys.diag_pool_.begin() + 1 : sys.diag_pool_.begin();
								for (auto d = beg; d < sys.diag_pool_.end(); ++d)
								{
									s_vc(6, d->xp, 1, &imp_->general_jacobi_.at(aris::dynamic::id(d->part->id() * 6, model().motionPool().size() + c.constraint->id() + k, n)), n);
								}
							}

							std::fill(r.bc, r.bc + 6, 0.0);
						}

						pos += c.constraint->dim();
					}
				}

				//dsp(model().partPool().size() * 6, n, imp_->general_jacobi_.data());

				//Size m = model().partPool().size() * 6;

				//double input[6]{0.0,0.0,0.0,0.0,0.0,0.0};
				//input[0] = model().motionPool()[0].mv();
				//input[1] = model().motionPool()[1].mv();
				//input[2] = model().motionPool()[2].mv();
				//input[3] = model().motionPool()[3].mv();
				//input[4] = model().motionPool()[4].mv();
				//input[5] = model().motionPool()[5].mv();

				//double result[42];
				//s_mm(m, 1, 6, imp_->general_jacobi_.data() + 6, 12, model().generalMotionPool().at(0).mvs(), 1, result, 1);
				//dsp(42, 1, result);
				


				// 做行加变换
				//sys.rowAddInverseXp();

				// 将x更新到杆件
				// sys.updPv();
			}
			
			
			

		}
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


						if(r.rel_.prtI == d1.part)
							std::cout << " 6x" << r.rel_.dim;
						else if(r.rel_.prtJ == d1.part)
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
		UniversalSolver::UniversalSolver(const std::string &name, Size max_iter_count, double max_error) :Solver(name, max_iter_count, max_error){}
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

		struct SolverSimulator::Imp
		{
			Solver *solver_;

			Imp(Solver *solver) :solver_(solver) { };
		};
		auto SolverSimulator::saveXml(aris::core::XmlElement &xml_ele) const->void
		{
			Simulator::saveXml(xml_ele);
			xml_ele.SetAttribute("solver", solver().name().c_str());
		}
		auto SolverSimulator::loadXml(const aris::core::XmlElement &xml_ele)->void
		{
			Simulator::loadXml(xml_ele);
			
			if (model().findByName("solver_pool") == model().children().end())
				throw std::runtime_error("you must insert \"solver_pool\" node before insert " + type() + " \"" + name() + "\"");

			auto &solver_pool = static_cast<aris::core::ObjectPool<Solver, Element>&>(*model().findByName("solver_pool"));

			if (!xml_ele.Attribute("solver"))throw std::runtime_error(std::string("xml element \"") + name() + "\" must have Attribute \"solver\"");
			auto s = solver_pool.findByName(xml_ele.Attribute("solver"));
			if (s == solver_pool.end())	throw std::runtime_error(std::string("can't find solver for element \"") + this->name() + "\"");

			imp_->solver_ = &*s;
		}
		auto SolverSimulator::solver()->Solver& { return *imp_->solver_; }
		auto SolverSimulator::simulate(const PlanFunction &plan, void *param, std::uint32_t param_size, SimResult &result)->void
		{
			solver().allocateMemory();
			result.allocateMemory();
			// 记录初始位置 //
			result.record();
			// 记录轨迹中的位置 //
			for (PlanParam plan_param{ &model(), std::uint32_t(1), param, param_size }; plan(plan_param) != 0; ++plan_param.count_)
			{
				solver().kinPos();
				if (solver().iterCount() == solver().maxIterCount())throw std::runtime_error("simulate failed because kinPos() failed at " + std::to_string(plan_param.count_) + " count");
				solver().kinVel();
				solver().dynAccAndFce();
				result.record();
			}
			// 记录结束位置 //
			result.record();
			result.restore(0);
		}
		SolverSimulator::~SolverSimulator() = default;
		SolverSimulator::SolverSimulator(const std::string &name, Solver *solver) : Simulator(name), imp_(new Imp(solver)) {}
		SolverSimulator::SolverSimulator(const SolverSimulator&) = default;
		SolverSimulator::SolverSimulator(SolverSimulator&&) = default;
		SolverSimulator& SolverSimulator::operator=(const SolverSimulator&) = default;
		SolverSimulator& SolverSimulator::operator=(SolverSimulator&&) = default;

		struct AdamsSimulator::Imp {};
		auto AdamsSimulator::saveAdams(const std::string &filename, SimResult &result, Size pos)->void
		{
			std::string filename_ = filename;
			if (filename_.size() < 4 || filename_.substr(filename.size() - 4, 4) != ".cmd")
			{
				filename_ += ".cmd";
			}

			std::ofstream file;
			file.open(filename_, std::ios::out | std::ios::trunc);

			saveAdams(file, result, pos);

			file.close();
		}
		auto AdamsSimulator::saveAdams(std::ofstream &file, SimResult &result, Size pos)->void
		{
			// 生成akima曲线 //
			std::vector<double> time(result.size() + 1);
			std::vector<std::vector<double>> mot_akima(model().motionPool().size(), std::vector<double>(result.size() + 1));
			std::vector<std::vector<std::array<double, 6>>> gm_akima(model().generalMotionPool().size(), std::vector<std::array<double, 6>>(result.size() + 1));
			if (pos == -1)
			{
				if (result.size() < 4)throw std::runtime_error("failed to AdamsSimulator::saveAdams: because result size is smaller than 4\n");

				for (Size i(-1); ++i < result.size() + 1;)
				{
					result.restore(i);
					time.at(i) = model().time();
					for (Size j(-1); ++j < model().motionPool().size();)
					{
						model().motionPool().at(j).updMp();
						mot_akima.at(j).at(i) = model().motionPool().at(j).mp();
					}
					for (Size j(-1); ++j < model().generalMotionPool().size();)
					{
						model().generalMotionPool().at(j).updMpm();
						model().generalMotionPool().at(j).getMpe(gm_akima.at(j).at(i).data(), "123");
					}
				}
			}

			// 生成ADAMS模型
			result.restore(pos == -1 ? 0 : pos);
			file << std::setprecision(15);
			file << "!----------------------------------- Environment -------------------------------!\r\n!\r\n!\r\n";
			file << "!-------------------------- Default Units for Model ---------------------------!\r\n"
				<< "!\r\n"
				<< "!\r\n"
				<< "defaults units  &\r\n"
				<< "    length = meter  &\r\n"
				<< "    angle = rad  &\r\n"
				<< "    force = newton  &\r\n"
				<< "    mass = kg  &\r\n"
				<< "    time = sec\r\n"
				<< "!\n"
				<< "defaults units  &\r\n"
				<< "    coordinate_system_type = cartesian  &\r\n"
				<< "    orientation_type = body313\r\n"
				<< "!\r\n"
				<< "!------------------------ Default Attributes for Model ------------------------!\r\n"
				<< "!\r\n"
				<< "!\r\n"
				<< "defaults attributes  &\r\n"
				<< "    inheritance = bottom_up  &\r\n"
				<< "    icon_visibility = off  &\r\n"
				<< "    grid_visibility = off  &\r\n"
				<< "    size_of_icons = 5.0E-002  &\r\n"
				<< "    spacing_for_grid = 1.0\r\n"
				<< "!\r\n"
				<< "!------------------------------ Adams/View Model ------------------------------!\r\n"
				<< "!\r\n"
				<< "!\r\n"
				<< "model create  &\r\n"
				<< "   model_name = " << this->model().name() << "\r\n"
				<< "!\r\n"
				<< "view erase\r\n"
				<< "!\r\n"
				<< "!---------------------------------- Accgrav -----------------------------------!\r\n"
				<< "!\r\n"
				<< "!\r\n"
				<< "force create body gravitational  &\r\n"
				<< "    gravity_field_name = gravity  &\r\n"
				<< "    x_component_gravity = " << model().environment().gravity()[0] << "  &\r\n"
				<< "    y_component_gravity = " << model().environment().gravity()[1] << "  &\r\n"
				<< "    z_component_gravity = " << model().environment().gravity()[2] << "\r\n"
				<< "!\r\n";
			for (auto &part : model().partPool())
			{
				if (&part == &model().ground())
				{
					file << "!----------------------------------- ground -----------------------------------!\r\n"
						<< "!\r\n"
						<< "!\r\n"
						<< "! ****** Ground Part ******\r\n"
						<< "!\r\n"
						<< "defaults model  &\r\n"
						<< "    part_name = ground\r\n"
						<< "!\r\n"
						<< "defaults coordinate_system  &\r\n"
						<< "    default_coordinate_system = ." << model().name() << ".ground\r\n"
						<< "!\r\n"
						<< "! ****** Markers for current part ******\r\n"
						<< "!\r\n";
				}
				else
				{
					double pe[6];
					s_pm2pe(*part.pm(), pe, "313");
					core::Matrix ori(1, 3, &pe[3]), loc(1, 3, &pe[0]);

					file << "!----------------------------------- " << part.name() << " -----------------------------------!\r\n"
						<< "!\r\n"
						<< "!\r\n"
						<< "defaults coordinate_system  &\r\n"
						<< "    default_coordinate_system = ." << model().name() << ".ground\r\n"
						<< "!\r\n"
						<< "part create rigid_body name_and_position  &\r\n"
						<< "    part_name = ." << model().name() << "." << part.name() << "  &\r\n"
						<< "    adams_id = " << adamsID(part) << "  &\r\n"
						<< "    location = (" << loc.toString() << ")  &\r\n"
						<< "    orientation = (" << ori.toString() << ")\r\n"
						<< "!\r\n"
						<< "defaults coordinate_system  &\r\n"
						<< "    default_coordinate_system = ." << model().name() << "." << part.name() << " \r\n"
						<< "!\r\n";


					double mass = part.prtIv()[0] == 0 ? 1 : part.prtIv()[0];
					std::fill_n(pe, 6, 0);
					pe[0] = part.prtIv()[1] / mass;
					pe[1] = part.prtIv()[2] / mass;
					pe[2] = part.prtIv()[3] / mass;

					file << "! ****** cm and mass for current part ******\r\n"
						<< "marker create  &\r\n"
						<< "    marker_name = ." << model().name() << "." << part.name() << ".cm  &\r\n"
						<< "    adams_id = " << adamsID(part) + std::accumulate(model().partPool().begin(), model().partPool().end(), Size(0), [](Size a, Part &b) {return a + b.markerPool().size(); }) << "  &\r\n"
						<< "    location = ({" << pe[0] << "," << pe[1] << "," << pe[2] << "})  &\r\n"
						<< "    orientation = (" << "{0,0,0}" << ")\r\n"
						<< "!\r\n";

					double pm[16];
					double iv[10];

					s_pe2pm(pe, pm);
					s_inv_iv2iv(pm, part.prtIv(), iv);

					//！注意！//
					//Adams里对惯量矩阵的定义貌似和我自己的定义在Ixy,Ixz,Iyz上互为相反数。别问我为什么,我也不知道。
					file << "part create rigid_body mass_properties  &\r\n"
						<< "    part_name = ." << model().name() << "." << part.name() << "  &\r\n"
						<< "    mass = " << part.prtIv()[0] << "  &\r\n"
						<< "    center_of_mass_marker = ." << model().name() << "." << part.name() << ".cm  &\r\n"
						<< "    inertia_marker = ." << model().name() << "." << part.name() << ".cm  &\r\n"
						<< "    ixx = " << iv[4] << "  &\r\n"
						<< "    iyy = " << iv[5] << "  &\r\n"
						<< "    izz = " << iv[6] << "  &\r\n"
						<< "    ixy = " << -iv[7] << "  &\r\n"
						<< "    izx = " << -iv[8] << "  &\r\n"
						<< "    iyz = " << -iv[9] << "\r\n"
						<< "!\r\n";

					
					double cm_pm_in_g[16];
					s_pm2pm(*part.pm(), pm, cm_pm_in_g);
					double cm_vs[6];
					s_inv_tv(cm_pm_in_g, part.vs(), cm_vs);

					file << "part create rigid_body initial_velocity  &\r\n"
						<< "    part_name = ." << model().name() << "." << part.name() << "  &\r\n"
						<< "    vx = " << cm_vs[0] << "  &\r\n"
						<< "    vy = " << cm_vs[1] << "  &\r\n"
						<< "    vz = " << cm_vs[2] << "  &\r\n"
						<< "    wx = " << cm_vs[3] << "  &\r\n"
						<< "    wy = " << cm_vs[4] << "  &\r\n"
						<< "    wz = " << cm_vs[5] << "  \r\n"
						<< "!\r\n";

					file << "part modify rigid_body initial_velocity  &\r\n"
						<< "    part_name = ." << model().name() << "." << part.name() << "  &\r\n"
						<< "    vm = ." << model().name() << "." << part.name() << ".cm  &\r\n"
						<< "    wm = ." << model().name() << "." << part.name() << ".cm \r\n"
						<< "!\r\n";


				}

				//导入marker
				for (auto &marker : part.markerPool())
				{
					double pe[6];

					s_pm2pe(*marker.prtPm(), pe, "313");
					core::Matrix ori(1, 3, &pe[3]), loc(1, 3, &pe[0]);

					file << "marker create  &\r\n"
						<< "marker_name = ." << model().name() << "." << part.name() << "." << marker.name() << "  &\r\n"
						<< "adams_id = " << adamsID(marker) << "  &\r\n"
						<< "location = (" << loc.toString() << ")  &\r\n"
						<< "orientation = (" << ori.toString() << ")\r\n"
						<< "!\r\n";
				}
			}
			for (auto &joint : model().jointPool())
			{
				std::string type;
				if (dynamic_cast<RevoluteJoint*>(&joint))type = "revolute";
				else if (dynamic_cast<PrismaticJoint*>(&joint))type = "translational";
				else if (dynamic_cast<UniversalJoint*>(&joint))type = "universal";
				else if (dynamic_cast<SphericalJoint*>(&joint))type = "spherical";
				else throw std::runtime_error("unrecognized joint type:" + joint.type());

				file << "constraint create joint " << type << "  &\r\n"
					<< "    joint_name = ." << model().name() << "." << joint.name() << "  &\r\n"
					<< "    adams_id = " << adamsID(joint) << "  &\r\n"
					<< "    i_marker_name = ." << model().name() << "." << joint.makI().fatherPart().name() << "." << joint.makI().name() << "  &\r\n"
					<< "    j_marker_name = ." << model().name() << "." << joint.makJ().fatherPart().name() << "." << joint.makJ().name() << "  \r\n"
					<< "!\r\n";
			}
			for (auto &motion : model().motionPool())
			{
				std::string axis_names[6]{ "x","y","z","B1","B2","B3" };
				std::string axis_name = axis_names[motion.axis()];

				std::string akima = motion.name() + "_akima";
				std::string akima_func = "AKISPL(time,0," + akima + ")";
				std::string polynomial_func = static_cast<const std::stringstream &>(std::stringstream() << std::setprecision(16) << motion.mp() << " + " << motion.mv() << " * time + " << motion.ma()*0.5 << " * time * time").str();

				// 构建akima曲线 //
				if (pos == -1)
				{
					file << "data_element create spline &\r\n"
						<< "    spline_name = ." << model().name() + "." + motion.name() + "_akima &\r\n"
						<< "    adams_id = " << adamsID(motion) << "  &\r\n"
						<< "    units = m &\r\n"
						<< "    x = " << time.at(0);
					for (auto p = time.begin() + 1; p < time.end(); ++p)
					{
						file << "," << *p;
					}
					file << "    y = " << mot_akima.at(motion.id()).at(0);
					for (auto p = mot_akima.at(motion.id()).begin() + 1; p < mot_akima.at(motion.id()).end(); ++p)
					{
						file << "," << *p;
					}
					file << " \r\n!\r\n";
				}

				file << "constraint create motion_generator &\r\n"
					<< "    motion_name = ." << model().name() << "." << motion.name() << "  &\r\n"
					<< "    adams_id = " << adamsID(motion) << "  &\r\n"
					<< "    i_marker_name = ." << model().name() << "." << motion.makI().fatherPart().name() << "." << motion.makI().name() << "  &\r\n"
					<< "    j_marker_name = ." << model().name() << "." << motion.makJ().fatherPart().name() << "." << motion.makJ().name() << "  &\r\n"
					<< "    axis = " << axis_name << "  &\r\n"
					<< "    function = \"" << (pos == -1 ? akima_func : polynomial_func) << "\"  \r\n"
					<< "!\r\n";
			}
			for (auto &gm : model().generalMotionPool())
			{
				file << "ude create instance  &\r\n"
					<< "    instance_name = ." << model().name() << "." << gm.name() << "  &\r\n"
					<< "    definition_name = .MDI.Constraints.general_motion  &\r\n"
					<< "    location = 0.0, 0.0, 0.0  &\r\n"
					<< "    orientation = 0.0, 0.0, 0.0  \r\n"
					<< "!\r\n";

				file << "variable modify  &\r\n"
					<< "	variable_name = ." << model().name() << "." << gm.name() << ".i_marker  &\r\n"
					<< "	object_value = ." << model().name() << "." << gm.makI().fatherPart().name() << "." << gm.makI().name() << " \r\n"
					<< "!\r\n";

				file << "variable modify  &\r\n"
					<< "	variable_name = ." << model().name() << "." << gm.name() << ".j_marker  &\r\n"
					<< "	object_value = ." << model().name() << "." << gm.makJ().fatherPart().name() << "." << gm.makJ().name() << " \r\n"
					<< "!\r\n";

				std::string axis_names[6]{ "t1", "t2", "t3", "r1", "r2", "r3" };

				double pe123[6], ve123[6], ae123[6];
				gm.getMpe(pe123, "123");
				gm.getMve(ve123, "123");
				gm.getMae(ae123, "123");
				for (Size i = 0; i < 6; ++i)
				{
					std::string akima = gm.name() + "_" + axis_names[i] + "_akima";
					std::string akima_func = "AKISPL(time,0," + akima + ")";
					std::string polynomial_func = static_cast<const std::stringstream &>(std::stringstream() << std::setprecision(16) << pe123[i] << " + " << ve123[i] << " * time + " << ae123[i] * 0.5 << " * time * time").str();
					std::string func = pos == -1 ? akima_func : polynomial_func;

					// 构建akima曲线 //
					if (pos == -1)
					{
						file << "data_element create spline &\r\n"
							<< "    spline_name = ." << model().name() + "." + akima + " &\r\n"
							<< "    adams_id = " << model().motionPool().size() + adamsID(gm) * 6 + i << "  &\r\n"
							<< "    units = m &\r\n"
							<< "    x = " << time.at(0);
						for (auto p = time.begin() + 1; p < time.end(); ++p)
						{
							file << "," << *p;
						}
						file << "    y = " << gm_akima.at(gm.id()).at(0).at(i);
						for (auto p = gm_akima.at(gm.id()).begin() + 1; p < gm_akima.at(gm.id()).end(); ++p)
						{
							file << "," << p->at(i);
						}
						file << " \r\n!\r\n";
					}

					file << "variable modify  &\r\n"
						<< "	variable_name = ." << model().name() << "." << gm.name() << "." << axis_names[i] << "_type  &\r\n"
						<< "	integer_value = 1 \r\n"
						<< "!\r\n";

					file << "variable modify  &\r\n"
						<< "	variable_name = ." << model().name() << "." << gm.name() << "." << axis_names[i] << "_func  &\r\n"
						<< "	string_value = \"" + func + "\" \r\n"
						<< "!\r\n";

					file << "variable modify  &\r\n"
						<< "	variable_name = ." << model().name() << "." << gm.name() << "." << axis_names[i] << "_ic_disp  &\r\n"
						<< "	real_value = 0.0 \r\n"
						<< "!\r\n";

					file << "variable modify  &\r\n"
						<< "	variable_name = ." << model().name() << "." << gm.name() << "." << axis_names[i] << "_ic_velo  &\r\n"
						<< "	real_value = 0.0 \r\n"
						<< "!\r\n";
				}

				file << "ude modify instance  &\r\n"
					<< "	instance_name = ." << model().name() << "." << gm.name() << "\r\n"
					<< "!\r\n";
			}
			for (auto &force : model().forcePool())
			{
				double fsI[6], fsJ[6], fsI_loc[6];
				force.cptGlbFs(fsI, fsJ);
				s_inv_fs2fs(*force.makI().pm(), fsI, fsI_loc);

				file << "floating_marker create  &\r\n"
					<< "    floating_marker_name = ." << model().name() << "." << force.makJ().fatherPart().name() << "." << force.name() << "_FMAK  &\r\n"
					<< "    adams_id = " << adamsID(force) + model().partPool().size() + std::accumulate(model().partPool().begin(), model().partPool().end(), Size(0), [](Size a, Part &b) {return a + b.markerPool().size(); }) << "\r\n"
					<< "!\r\n";

				file << "force create direct general_force  &\r\n"
					<< "    general_force_name = ." << model().name() << "." << force.name() << "  &\r\n"
					<< "    adams_id = " << adamsID(force) << "  &\r\n"
					<< "    i_marker_name = ." << model().name() << "." << force.makI().fatherPart().name() << "." << force.makI().name() << "  &\r\n"
					<< "    j_floating_marker_name = ." << model().name() << "." << force.makJ().fatherPart().name() << "." << force.name() << "_FMAK  &\r\n"
					<< "    ref_marker_name = ." << model().name() << "." << force.makI().fatherPart().name() << "." << force.makI().name() << "  &\r\n"
					<< "    x_force_function = \"" << fsI_loc[0] << "\"  &\r\n"
					<< "    y_force_function = \"" << fsI_loc[1] << "\"  &\r\n"
					<< "    z_force_function = \"" << fsI_loc[2] << "\"  &\r\n"
					<< "    x_torque_function = \"" << fsI_loc[3] << "\"  &\r\n"
					<< "    y_torque_function = \"" << fsI_loc[4] << "\"  &\r\n"
					<< "    z_torque_function = \"" << fsI_loc[5] << "\"\r\n"
					<< "!\r\n";

			}

			// geometry, 防止geometry 添加marker，导致marker id冲突
			for (auto &part : model().partPool())
			{
				for (auto &geometry : part.geometryPool())
				{
					if (ParasolidGeometry* geo = dynamic_cast<ParasolidGeometry*>(&geometry))
					{
						double pe[6];
						s_pm2pe(*geo->prtPm(), pe, "313");
						core::Matrix ori(1, 3, &pe[3]), loc(1, 3, &pe[0]);

						file << "file parasolid read &\r\n"
							<< "	file_name = \"" << geo->filePath() << "\" &\r\n"
							<< "	type = ASCII" << " &\r\n"
							<< "	part_name = " << part.name() << " &\r\n"
							<< "	location = (" << loc.toString() << ") &\r\n"
							<< "	orientation = (" << ori.toString() << ") &\r\n"
							<< "	relative_to = ." << model().name() << "." << part.name() << " \r\n"
							<< "!\r\n";
					}
					else if (FileGeometry* geo = dynamic_cast<FileGeometry*>(&geometry))
					{
						double pe[6];
						s_pm2pe(*geo->prtPm(), pe, "313");
						core::Matrix ori(1, 3, &pe[3]), loc(1, 3, &pe[0]);

						file << "file geometry read &\r\n"
							<< "	type_of_geometry = \"" << "dwg" << "\" &\r\n"
							<< "	file_name = \"" << geo->filePath() << "\" &\r\n"
							<< "	part_name = " << part.name() << " &\r\n"
							<< "	location = (" << loc.toString() << ") &\r\n"
							<< "	orientation = (" << ori.toString() << ") &\r\n"
							<< "	relative_to = ." << model().name() << "." << part.name() << " &\r\n"
							<< "	scale = " << "0.001" << " \r\n"
							<< "!\r\n";
					}
					else
					{
						throw std::runtime_error("unrecognized geometry type:" + geometry.type());
					}

				}
			}

			file << "!----------------------------------- Motify Active -------------------------------------!\r\n!\r\n!\r\n";
			for (auto &prt : model().partPool())
			{
				if ((&prt != &model().ground()) && (!prt.active()))
				{
					file << "part attributes  &\r\n"
						<< "    part_name = ." << model().name() << "." << prt.name() << "  &\r\n"
						<< "    active = off \r\n!\r\n";
				}
			}
			for (auto &jnt : model().jointPool())
			{
				if (!jnt.active())
				{
					file << "constraint attributes  &\r\n"
						<< "    constraint_name = ." << model().name() << "." << jnt.name() << "  &\r\n"
						<< "    active = off \r\n!\r\n";
				}
			}
			for (auto &mot : model().motionPool())
			{
				if (!mot.active())
				{
					file << "constraint attributes  &\r\n"
						<< "    constraint_name = ." << model().name() << "." << mot.name() << "  &\r\n"
						<< "    active = off \r\n!\r\n";
				}
			}
			for (auto &gm : model().generalMotionPool())
			{
				if (!gm.active())
				{
					file << "ude attributes  &\r\n"
						<< "    instance_name = ." << model().name() << "." << gm.name() << "  &\r\n"
						<< "    active = off \r\n!\r\n";
				}
			}
			for (auto &fce : model().forcePool())
			{
				if (!fce.active())
				{
					file << "force attributes  &\r\n"
						<< "    force_name = ." << model().name() << "." << fce.name() << "  &\r\n"
						<< "    active = off \r\n!\r\n";
				}
			}
		}
		auto AdamsSimulator::saveAdams(const std::string &filename)->void
		{
			std::string filename_ = filename;
			if (filename_.size() < 4 || filename_.substr(filename.size() - 4, 4) != ".cmd")
			{
				filename_ += ".cmd";
			}

			std::ofstream file;
			file.open(filename_, std::ios::out | std::ios::trunc);

			saveAdams(file);

			file.close();
		}
		auto AdamsSimulator::saveAdams(std::ofstream &file)->void
		{
			model().simResultPool().add<SimResult>();
			model().simResultPool().back().record();

			saveAdams(file, model().simResultPool().back(), 0);

			model().simResultPool().erase(model().simResultPool().end() - 1);
		}
		auto AdamsSimulator::adamsID(const Marker &mak)const->Size
		{
			Size size{ 0 };

			for (auto &prt : model().partPool())
			{
				if (&prt == &mak.fatherPart()) break;
				size += prt.markerPool().size();
			}

			size += mak.id() + 1;

			return size;
		}
		auto AdamsSimulator::adamsID(const Part &prt)const->Size { return (&prt == &model().ground()) ? 1 : prt.id() + (model().ground().id() < prt.id() ? 1 : 2); }
		AdamsSimulator::~AdamsSimulator() = default;
		AdamsSimulator::AdamsSimulator(const std::string &name) : Simulator(name) {}
		AdamsSimulator::AdamsSimulator(const AdamsSimulator&) = default;
		AdamsSimulator::AdamsSimulator(AdamsSimulator&&) = default;
		AdamsSimulator& AdamsSimulator::operator=(const AdamsSimulator&) = default;
		AdamsSimulator& AdamsSimulator::operator=(AdamsSimulator&&) = default;
	}
}
