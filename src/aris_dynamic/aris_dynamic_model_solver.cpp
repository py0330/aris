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

#include "aris_dynamic_model_solver.h"


namespace aris
{
	namespace dynamic
	{
		struct CombineSolver::Imp
		{
			Size p_size_, c_size_;
			std::vector<double> A_, x_, b_;
			std::vector<double> U_, t_;
			std::vector<Size> p_;

			std::vector<PartBlock> part_block_pool_;
			std::vector<ConstraintBlock> constraint_block_pool_;
		};
		auto CombineSolver::allocateMemory()->void
		{
			// make active pool //
			imp_->part_block_pool_.clear();
			imp_->constraint_block_pool_.clear();

			imp_->part_block_pool_.push_back(PartBlock{ &model().ground(), 0 });
			for (auto &prt : model().partPool())if (prt.active() && (&prt != &model().ground()))imp_->part_block_pool_.push_back(PartBlock{ &prt, 0 });
			for (auto &jnt : model().jointPool())if (jnt.active())imp_->constraint_block_pool_.push_back(ConstraintBlock{ &jnt,0, nullptr, nullptr });
			for (auto &mot : model().motionPool())if (mot.active()) imp_->constraint_block_pool_.push_back(ConstraintBlock{ &mot,0, nullptr, nullptr });
			for (auto &gmt : model().generalMotionPool())if (gmt.active())imp_->constraint_block_pool_.push_back(ConstraintBlock{ &gmt,0, nullptr, nullptr });

			// compute memory size //
			imp_->p_size_ = 0;
			imp_->c_size_ = 6;

			for (auto &pb : activePartBlockPool())
			{
				pb.row_id_ = imp_->p_size_;
				imp_->p_size_ += 6;
			}
			for (auto &cb : activeConstraintBlockPool())
			{
				cb.col_id_ = imp_->c_size_;
				imp_->c_size_ += cb.constraint_->dim();

				auto i_ = std::find_if(activePartBlockPool().begin(), activePartBlockPool().end(), [&cb](PartBlock &pb) {return pb.part_ == &cb.constraint_->makI().fatherPart(); });
				auto j_ = std::find_if(activePartBlockPool().begin(), activePartBlockPool().end(), [&cb](PartBlock &pb) {return pb.part_ == &cb.constraint_->makJ().fatherPart(); });
				if (i_ == activePartBlockPool().end()) throw std::runtime_error("i part not found");
				if (j_ == activePartBlockPool().end()) throw std::runtime_error("j part not found");
				cb.pb_i_ = &*i_;
				cb.pb_j_ = &*j_;
			}

			// allocate memory //
			imp_->A_.clear();
			imp_->A_.resize(aSize() * aSize(), 0.0);
			imp_->b_.clear();
			imp_->b_.resize(aSize() * 1, 0.0);
			imp_->x_.clear();
			imp_->x_.resize(aSize() * 1, 0.0);
			for (Size i(-1); ++i < 6;) 
			{
				imp_->A_.data()[aris::dynamic::id(i, i, aSize())] = 1.0;
				imp_->A_.data()[aris::dynamic::id(i + pSize(), i, aSize())] = 1.0;
				imp_->A_.data()[aris::dynamic::id(i, i + pSize(), aSize())] = 1.0;
			}

			imp_->U_.clear();
			imp_->U_.resize(aSize() * aSize(), 0.0);
			imp_->t_.clear();
			imp_->t_.resize(aSize() * 1, 0.0);
			imp_->p_.clear();
			imp_->p_.resize(aSize() * 1, 0);
		}
		auto CombineSolver::activePartBlockPool()->std::vector<PartBlock>& { return imp_->part_block_pool_; }
		auto CombineSolver::activeConstraintBlockPool()->std::vector<ConstraintBlock>& { return imp_->constraint_block_pool_; }
		auto CombineSolver::cSize()->Size { return imp_->c_size_; }
		auto CombineSolver::pSize()->Size { return imp_->p_size_; }
		auto CombineSolver::A()->double * { return imp_->A_.data(); }
		auto CombineSolver::x()->double * { return imp_->x_.data(); }
		auto CombineSolver::b()->double * { return imp_->b_.data(); }
		auto CombineSolver::kinPos()->void
		{
			setIterCount(0);

			double pm[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
			s_mc(4, 4, pm, const_cast<double *>(*model().ground().pm()));
			for (; iterCount() < maxIterCount(); setIterCount(iterCount() + 1))
			{
				// make cp //
				updCp();

				// check error //
				setError(s_norm(cSize(), cp()));
				if (error() < maxError()) return;

				auto e = error();

				// make C //
				updCm();

				// solve dp //
				s_householder_utp(cSize(), pSize(), cm(), ColMajor{ aSize() }, imp_->U_.data(), pSize(), imp_->t_.data(), 1, imp_->p_.data(), maxError());
				s_householder_utp_sov(cSize(), pSize(), 1, imp_->U_.data(), imp_->t_.data(), imp_->p_.data(), cp(), pp(), maxError());

				// upd part pos //
				updPartPos();
			}
		}
		auto CombineSolver::kinVel()->void
		{
			s_fill(6, 1, 0.0, const_cast<double *>(model().ground().vs()));
			
			// make cv //
			updCv();

			// make C //
			updCm();

			// solve compensate pv //
			s_householder_utp(cSize(), pSize(), cm(), ColMajor{ aSize() }, imp_->U_.data(), pSize(), imp_->t_.data(), 1, imp_->p_.data(), maxError());
			s_householder_utp_sov(cSize(), pSize(), 1, imp_->U_.data(), imp_->t_.data(), imp_->p_.data(), cv(), pv(), maxError());

			// upd part vel //
			updPartVel();
		}
		auto CombineSolver::dynAccAndFce()->void
		{
			s_fill(6, 1, 0.0, const_cast<double *>(model().ground().as()));
			
			updIm();
			updCm();
			updCmT();

			updCa();
			updPf();

			// solve x //
			s_householder_utp(aSize(), aSize(), A(), imp_->U_.data(), imp_->t_.data(), imp_->p_.data(), maxError());
			s_householder_utp_sov(aSize(), aSize(), 1, imp_->U_.data(), imp_->t_.data(), imp_->p_.data(), b(), x(), maxError());

			updConstraintFce();
			updPartAcc();

		}
		CombineSolver::~CombineSolver() = default;
		CombineSolver::CombineSolver(const std::string &name, Size max_iter_count, double max_error) :Solver(name, max_iter_count, max_error) {}
		CombineSolver::CombineSolver(Object &father, const aris::core::XmlElement &xml_ele) : Solver(father, xml_ele) {}
		CombineSolver::CombineSolver(const CombineSolver &other) = default;
		CombineSolver::CombineSolver(CombineSolver &&other) = default;
		CombineSolver& CombineSolver::operator=(const CombineSolver &other) = default;
		CombineSolver& CombineSolver::operator=(CombineSolver &&other) = default;

		auto GroundCombineSolver::updCm()->void 
		{
			for (auto &cb : activeConstraintBlockPool())
			{
				auto row_i = cb.pb_i_->row_id_;
				auto row_j = cb.pb_j_->row_id_;
				auto col = cb.col_id_ + pSize();

				cb.constraint_->cptGlbCm(A() + dynamic::id(row_i, col, aSize()), aSize(), A() + dynamic::id(row_j, col, aSize()), aSize());
			}
		}
		auto GroundCombineSolver::updCmT()->void
		{
			for (auto &cb : activeConstraintBlockPool())
			{
				auto row_i = cb.pb_i_->row_id_;
				auto row_j = cb.pb_j_->row_id_;
				auto col = cb.col_id_ + pSize();

				cb.constraint_->cptGlbCm(A() + dynamic::id(row_i, col, ColMajor{ aSize() }), ColMajor{ aSize() }, A() + dynamic::id(row_j, col, ColMajor{ aSize() }), ColMajor{ aSize() });
			}
		}
		auto GroundCombineSolver::updIm()->void { for (auto &pb : activePartBlockPool())pb.part_->cptGlbIm(A() + dynamic::id(pb.row_id_, pb.row_id_, aSize()), aSize()); }
		auto GroundCombineSolver::updCp()->void { for (auto &cb : activeConstraintBlockPool())cb.constraint_->cptCp(cp() + cb.col_id_); }
		auto GroundCombineSolver::updCv()->void { for (auto &cb : activeConstraintBlockPool())cb.constraint_->cptCv(cv() + cb.col_id_); }
		auto GroundCombineSolver::updCa()->void { for (auto &cb : activeConstraintBlockPool())cb.constraint_->cptCa(ca() + cb.col_id_); }
		auto GroundCombineSolver::updPv()->void { for (auto &pb : activePartBlockPool())pb.part_->getVs(pv() + pb.row_id_); }
		auto GroundCombineSolver::updPa()->void { for (auto &pb : activePartBlockPool())pb.part_->getAs(pa() + pb.row_id_); }
		auto GroundCombineSolver::updPf()->void { for (auto &pb : activePartBlockPool())pb.part_->cptGlbPf(pf() + pb.row_id_); }
		auto GroundCombineSolver::updConstraintFce()->void { for (auto &cb : activeConstraintBlockPool())cb.constraint_->setCf(cf() + dynamic::id(cb.col_id_, 0, 1));}
		auto GroundCombineSolver::updPartPos()->void 
		{
			for (auto pb : activePartBlockPool())
			{
				if (pb.part_ != &model().ground())
				{
					double pm[4][4];
					double pq[7];

					s_vc(6, x() + dynamic::id(pb.row_id_, 0, 1), pq);

					double theta = s_norm(3, pq + 3);
					pq[6] = std::cos(theta / 2);

					double factor = theta < 1e-4 ? 0.5 : std::sin(theta / 2) / theta;
					s_nv(3, factor, pq + 3);

					s_pq2pm(pq, *pm);

					double final_pm[4][4];
					s_pm2pm(*pm, *pb.part_->pm(), *final_pm);

					pb.part_->setPm(*final_pm);
				}
			}
		}
		auto GroundCombineSolver::updPartVel()->void { for (auto &pb : activePartBlockPool()) s_va(6, pv() + pb.row_id_, const_cast<double*>(pb.part_->vs())); }
		auto GroundCombineSolver::updPartAcc()->void { for (auto &pb : activePartBlockPool()) pb.part_->setAs(pa() + dynamic::id(pb.row_id_, 0, 1)); }
		GroundCombineSolver::~GroundCombineSolver() = default;
		GroundCombineSolver::GroundCombineSolver(const std::string &name, Size max_iter_count, double max_error) :CombineSolver(name, max_iter_count, max_error) {}
		GroundCombineSolver::GroundCombineSolver(Object &father, const aris::core::XmlElement &xml_ele) : CombineSolver(father, xml_ele) {}
		GroundCombineSolver::GroundCombineSolver(const GroundCombineSolver &other) = default;
		GroundCombineSolver::GroundCombineSolver(GroundCombineSolver &&other) = default;
		GroundCombineSolver& GroundCombineSolver::operator=(const GroundCombineSolver &other) = default;
		GroundCombineSolver& GroundCombineSolver::operator=(GroundCombineSolver &&other) = default;

		struct DividedSolver::Imp
		{
			Size p_size_, c_size_;
			std::vector<double> im_, cm_, pp_, pv_, pa_, pf_, cp_, cv_, ca_, cf_;

			std::vector<PartBlock> part_block_pool_;
			std::vector<ConstraintBlock> constraint_block_pool_;

			BlockSize p_blk_size_, c_blk_size_;
			BlockData im_blk_, cm_blk_, pp_blk_, pv_blk_, pa_blk_, pf_blk_, cp_blk_, cv_blk_, ca_blk_, cf_blk_;
		};
		auto DividedSolver::allocateMemory()->void
		{
			// make active pool //
			imp_->part_block_pool_.clear();
			imp_->constraint_block_pool_.clear();

			for (auto &prt : model().partPool())if (prt.active())imp_->part_block_pool_.push_back(PartBlock{ &prt, 0, 0 });
			for (auto &jnt : model().jointPool())if (jnt.active())imp_->constraint_block_pool_.push_back(ConstraintBlock{ &jnt,0,0, nullptr, nullptr });
			for (auto &mot : model().motionPool())if (mot.active()) imp_->constraint_block_pool_.push_back(ConstraintBlock{ &mot,0,0, nullptr, nullptr });
			for (auto &gmt : model().generalMotionPool())if (gmt.active())imp_->constraint_block_pool_.push_back(ConstraintBlock{ &gmt,0,0, nullptr, nullptr });

			// compute memory size //
			imp_->p_size_ = 0;
			imp_->c_size_ = 6;
			imp_->p_blk_size_.clear();
			imp_->c_blk_size_.clear();
			imp_->c_blk_size_.resize(1, 6);

			for (auto &pb : activePartBlockPool())
			{
				pb.row_id_ = imp_->p_size_;
				pb.blk_row_id_ = imp_->p_blk_size_.size();
				imp_->p_size_ += 6;
				imp_->p_blk_size_.push_back(6);
			}
			for (auto &cb : activeConstraintBlockPool())
			{
				cb.col_id_ = imp_->c_size_;
				cb.blk_col_id_ = imp_->c_blk_size_.size();
				imp_->c_size_ += cb.constraint_->dim();
				imp_->c_blk_size_.push_back(cb.constraint_->dim());

				auto i_ = std::find_if(activePartBlockPool().begin(), activePartBlockPool().end(), [&cb](PartBlock &pb) {return pb.part_ == &cb.constraint_->makI().fatherPart(); });
				auto j_ = std::find_if(activePartBlockPool().begin(), activePartBlockPool().end(), [&cb](PartBlock &pb) {return pb.part_ == &cb.constraint_->makJ().fatherPart(); });
				if (i_ == activePartBlockPool().end()) throw std::runtime_error("i part not found");
				if (j_ == activePartBlockPool().end()) throw std::runtime_error("j part not found");
				cb.pb_i_ = &*i_;
				cb.pb_j_ = &*j_;
			}

			// allocate memory //
			imp_->im_.clear();
			imp_->im_.resize(imp_->p_size_ * imp_->p_size_, 0.0);
			imp_->cm_.clear();
			imp_->cm_.resize(imp_->p_size_ * imp_->c_size_, 0.0);
			imp_->cp_.clear();
			imp_->cp_.resize(imp_->c_size_ * 1, 0.0);
			imp_->cv_.clear();
			imp_->cv_.resize(imp_->c_size_ * 1, 0.0);
			imp_->ca_.clear();
			imp_->ca_.resize(imp_->c_size_ * 1, 0.0);
			imp_->cf_.clear();
			imp_->cf_.resize(imp_->c_size_ * 1, 0.0);
			imp_->pp_.clear();
			imp_->pp_.resize(imp_->p_size_ * 1, 0.0);
			imp_->pv_.clear();
			imp_->pv_.resize(imp_->p_size_ * 1, 0.0);
			imp_->pa_.clear();
			imp_->pa_.resize(imp_->p_size_ * 1, 0.0);
			imp_->pf_.clear();
			imp_->pf_.resize(imp_->p_size_ * 1, 0.0);

			imp_->im_blk_.clear();
			imp_->im_blk_.resize(imp_->p_blk_size_.size() * imp_->p_blk_size_.size());
			imp_->cm_blk_.clear();
			imp_->cm_blk_.resize(imp_->p_blk_size_.size() * imp_->c_blk_size_.size());
			imp_->cp_blk_.clear();
			imp_->cp_blk_.resize(imp_->c_blk_size_.size() * 1);
			imp_->cv_blk_.clear();
			imp_->cv_blk_.resize(imp_->c_blk_size_.size() * 1);
			imp_->ca_blk_.clear();
			imp_->ca_blk_.resize(imp_->c_blk_size_.size() * 1);
			imp_->cf_blk_.clear();
			imp_->cf_blk_.resize(imp_->c_blk_size_.size() * 1);
			imp_->pp_blk_.clear();
			imp_->pp_blk_.resize(imp_->p_blk_size_.size() * 1);
			imp_->pv_blk_.clear();
			imp_->pv_blk_.resize(imp_->p_blk_size_.size() * 1);
			imp_->pa_blk_.clear();
			imp_->pa_blk_.resize(imp_->p_blk_size_.size() * 1);
			imp_->pf_blk_.clear();
			imp_->pf_blk_.resize(imp_->p_blk_size_.size() * 1);
			
			s_blk_map(cBlkSize(), { 1 }, imp_->cp_.data(), imp_->cp_blk_);
			s_blk_map(cBlkSize(), { 1 }, imp_->cv_.data(), imp_->cv_blk_);
			s_blk_map(cBlkSize(), { 1 }, imp_->ca_.data(), imp_->ca_blk_);
			s_blk_map(cBlkSize(), { 1 }, imp_->cf_.data(), imp_->cf_blk_);
			s_blk_map(pBlkSize(), { 1 }, imp_->pp_.data(), imp_->pp_blk_);
			s_blk_map(pBlkSize(), { 1 }, imp_->pv_.data(), imp_->pv_blk_);
			s_blk_map(pBlkSize(), { 1 }, imp_->pa_.data(), imp_->pa_blk_);
			s_blk_map(pBlkSize(), { 1 }, imp_->pf_.data(), imp_->pf_blk_);
			s_blk_map(pBlkSize(), pBlkSize(), imp_->im_.data(), imp_->im_blk_);
			s_blk_map(pBlkSize(), cBlkSize(), imp_->cm_.data(), imp_->cm_blk_);

			for (auto &ele : imp_->im_blk_)ele.is_zero = true;
			for (auto &ele : imp_->cm_blk_)ele.is_zero = true;
			for (auto &pb : activePartBlockPool())imp_->im_blk_[dynamic::id(pb.blk_row_id_, pb.blk_row_id_, pBlkSize().size())].is_zero = false;
			for (auto &cb : activeConstraintBlockPool())
			{
				imp_->cm_blk_[dynamic::id(cb.pb_i_->blk_row_id_, cb.blk_col_id_, cBlkSize().size())].is_zero = false;
				imp_->cm_blk_[dynamic::id(cb.pb_j_->blk_row_id_, cb.blk_col_id_, cBlkSize().size())].is_zero = false;
			}

			auto ground_iter = std::find_if(imp_->part_block_pool_.begin(), imp_->part_block_pool_.end(), [&](PartBlock &pb) {return pb.part_ == &model().ground(); });

			imp_->cm_blk_[dynamic::id(ground_iter->blk_row_id_, 0, cBlkSize().size())].is_zero = false;
			for (Size i = 0; i < 6; ++i)imp_->cm_[aris::dynamic::id(ground_iter->row_id_ + i, i, cSize())] = 1.0;

		}
		auto DividedSolver::activePartBlockPool()->std::vector<PartBlock>& { return imp_->part_block_pool_; }
		auto DividedSolver::activeConstraintBlockPool()->std::vector<ConstraintBlock>& { return imp_->constraint_block_pool_; }
		auto DividedSolver::cSize()->Size { return imp_->c_size_; }
		auto DividedSolver::pSize()->Size { return imp_->p_size_; }
		auto DividedSolver::im()->double * { return imp_->im_.data(); }
		auto DividedSolver::cm()->double * { return imp_->cm_.data(); }
		auto DividedSolver::pp()->double * { return imp_->pp_.data(); }
		auto DividedSolver::pv()->double * { return imp_->pv_.data(); }
		auto DividedSolver::pa()->double * { return imp_->pa_.data(); }
		auto DividedSolver::pf()->double * { return imp_->pf_.data(); }
		auto DividedSolver::cp()->double * { return imp_->cp_.data(); }
		auto DividedSolver::cv()->double * { return imp_->cv_.data(); }
		auto DividedSolver::ca()->double * { return imp_->ca_.data(); }
		auto DividedSolver::cf()->double * { return imp_->cf_.data(); }
		auto DividedSolver::cBlkSize()->BlockSize& { return imp_->c_blk_size_; }
		auto DividedSolver::pBlkSize()->BlockSize& { return imp_->p_blk_size_; }
		auto DividedSolver::imBlk()->BlockData& { return imp_->im_blk_; }
		auto DividedSolver::cmBlk()->BlockData& { return imp_->cm_blk_; }
		auto DividedSolver::ppBlk()->BlockData& { return imp_->pp_blk_; }
		auto DividedSolver::pvBlk()->BlockData& { return imp_->pv_blk_; }
		auto DividedSolver::paBlk()->BlockData& { return imp_->pa_blk_; }
		auto DividedSolver::pfBlk()->BlockData& { return imp_->pf_blk_; }
		auto DividedSolver::cpBlk()->BlockData& { return imp_->cp_blk_; }
		auto DividedSolver::cvBlk()->BlockData& { return imp_->cv_blk_; }
		auto DividedSolver::caBlk()->BlockData& { return imp_->ca_blk_; }
		auto DividedSolver::cfBlk()->BlockData& { return imp_->cf_blk_; }
		DividedSolver::~DividedSolver() = default;
		DividedSolver::DividedSolver(const std::string &name, Size max_iter_count, double max_error) :Solver(name, max_iter_count, max_error) {}
		DividedSolver::DividedSolver(Object &father, const aris::core::XmlElement &xml_ele) :Solver(father, xml_ele){}
		DividedSolver::DividedSolver(const DividedSolver &other) = default;
		DividedSolver::DividedSolver(DividedSolver &&other) = default;
		DividedSolver& DividedSolver::operator=(const DividedSolver &other) = default;
		DividedSolver& DividedSolver::operator=(DividedSolver &&other) = default;
		
		struct GroundDividedSolver::Imp {};
		auto GroundDividedSolver::updCp()->void { for (auto &cb : activeConstraintBlockPool())cb.constraint_->cptCp(cp() + dynamic::id(cb.col_id_, 0, 1)); }
		auto GroundDividedSolver::updCv()->void { for (auto &cb : activeConstraintBlockPool())cb.constraint_->cptCv(cv() + dynamic::id(cb.col_id_, 0, 1)); }
		auto GroundDividedSolver::updCa()->void { for (auto &cb : activeConstraintBlockPool())cb.constraint_->cptCa(ca() + dynamic::id(cb.col_id_, 0, 1)); }
		auto GroundDividedSolver::updPv()->void { for (auto &pb : activePartBlockPool())pb.part_->getVs(pv() + dynamic::id(pb.row_id_, 0, 1)); }
		auto GroundDividedSolver::updPa()->void { for (auto &pb : activePartBlockPool())pb.part_->getAs(pa() + dynamic::id(pb.row_id_, 0, 1)); }
		auto GroundDividedSolver::updPf()->void { for (auto &pb : activePartBlockPool())pb.part_->cptGlbPf(pf() + dynamic::id(pb.row_id_, 0, 1)); }
		auto GroundDividedSolver::updIm()->void { for (auto &pb : activePartBlockPool())pb.part_->cptGlbIm(im() + dynamic::id(pb.row_id_, pb.row_id_, pSize()), pSize()); }
		auto GroundDividedSolver::updCm()->void { for (auto &cb : activeConstraintBlockPool())cb.constraint_->cptGlbCm(cm() + dynamic::id(cb.pb_i_->row_id_, cb.col_id_, cSize()), cSize(), cm() + dynamic::id(cb.pb_j_->row_id_, cb.col_id_, cSize()), cSize()); }
		auto GroundDividedSolver::updConstraintFce()->void { for (auto &cb : activeConstraintBlockPool())cb.constraint_->setCf(cf() + dynamic::id(cb.col_id_, 0, 1)); }
		auto GroundDividedSolver::updPartPos()->void
		{
			double pm[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
			s_mc(4, 4, pm, const_cast<double *>(*model().ground().pm()));
			
			for (auto pb : activePartBlockPool())
			{
				if (pb.part_ != &model().ground())
				{
					double pm[4][4];
					double pq[7];

					s_vc(6, pp() + dynamic::id(pb.row_id_, 0, 1), pq);

					double theta = s_norm(3, pq + 3);
					pq[6] = std::cos(theta / 2);

					double factor = theta < 1e-4 ? 0.5 : std::sin(theta / 2) / theta;
					s_nv(3, factor, pq + 3);

					s_pq2pm(pq, *pm);

					double final_pm[4][4];
					s_pm2pm(*pm, *pb.part_->pm(), *final_pm);

					pb.part_->setPm(*final_pm);
				}
			}
		}
		auto GroundDividedSolver::updPartVel()->void 
		{ 
			for (auto &pb : activePartBlockPool()) 
				if (pb.part_ != &model().ground()) 
					s_va(6, pv() + dynamic::id(pb.row_id_, 0, 1), const_cast<double*>(pb.part_->vs())); 
		}
		auto GroundDividedSolver::updPartAcc()->void 
		{ 
			for (auto &pb : activePartBlockPool())
				if (pb.part_ != &model().ground()) 
					pb.part_->setAs(pa() + dynamic::id(pb.row_id_, 0, 1)); 
		}
		GroundDividedSolver::~GroundDividedSolver() = default;
		GroundDividedSolver::GroundDividedSolver(const std::string &name, Size max_iter_count, double max_error) :DividedSolver(name, max_iter_count, max_error) {}
		GroundDividedSolver::GroundDividedSolver(Object &father, const aris::core::XmlElement &xml_ele) : DividedSolver(father, xml_ele) {}
		GroundDividedSolver::GroundDividedSolver(const GroundDividedSolver &other) = default;
		GroundDividedSolver::GroundDividedSolver(GroundDividedSolver &&other) = default;
		GroundDividedSolver& GroundDividedSolver::operator=(const GroundDividedSolver &other) = default;
		GroundDividedSolver& GroundDividedSolver::operator=(GroundDividedSolver &&other) = default;

		struct PartDividedSolver::Imp {};
		auto PartDividedSolver::updCp()->void { for (auto &cb : activeConstraintBlockPool())cb.constraint_->cptCp(cp() + dynamic::id(cb.col_id_, 0, 1)); }
		auto PartDividedSolver::updCv()->void { for (auto &cb : activeConstraintBlockPool())cb.constraint_->cptCv(cv() + dynamic::id(cb.col_id_, 0, 1)); }
		auto PartDividedSolver::updCa()->void { for (auto &cb : activeConstraintBlockPool())cb.constraint_->cptCa(ca() + dynamic::id(cb.col_id_, 0, 1)); }
		auto PartDividedSolver::updPv()->void { for (auto &pb : activePartBlockPool())pb.part_->cptPrtVs(pv() + dynamic::id(pb.row_id_, 0, 1)); }
		auto PartDividedSolver::updPa()->void { for (auto &pb : activePartBlockPool())pb.part_->cptPrtAs(pa() + dynamic::id(pb.row_id_, 0, 1)); }
		auto PartDividedSolver::updPf()->void { for (auto &pb : activePartBlockPool())pb.part_->cptPrtPf(pf() + dynamic::id(pb.row_id_, 0, 1)); }
		auto PartDividedSolver::updIm()->void { for (auto &pb : activePartBlockPool())s_mc(6, 6, *pb.part_->prtIm(), 6, im() + dynamic::id(pb.row_id_, pb.row_id_, pSize()), pSize()); }
		auto PartDividedSolver::updCm()->void { for (auto &cb : activeConstraintBlockPool())cb.constraint_->cptPrtCm(cm() + dynamic::id(cb.pb_i_->row_id_, cb.col_id_, cSize()), cSize(), cm() + dynamic::id(cb.pb_j_->row_id_, cb.col_id_, cSize()), cSize()); }
		auto PartDividedSolver::updConstraintFce()->void { for (auto &cb : activeConstraintBlockPool())cb.constraint_->setCf(cf() + dynamic::id(cb.col_id_, 0, 1)); }
		auto PartDividedSolver::updPartPos()->void
		{
			double pm[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
			s_mc(4, 4, pm, const_cast<double *>(*model().ground().pm()));

			for (auto pb : activePartBlockPool())
			{
				if (pb.part_ != &model().ground())
				{
					double pm[4][4];
					double pq[7];

					s_vc(6, pp() + dynamic::id(pb.row_id_, 0, 1), pq);
					s_tv(*pb.part_->pm(), pp() + dynamic::id(pb.row_id_, 0, 1), pq);

					double theta = s_norm(3, pq + 3);
					pq[6] = std::cos(theta / 2);

					double factor = theta < 1e-4 ? 0.5 : std::sin(theta / 2) / theta;
					s_nv(3, factor, pq + 3);

					s_pq2pm(pq, *pm);

					double final_pm[4][4];
					s_pm2pm(*pm, *pb.part_->pm(), *final_pm);

					pb.part_->setPm(*final_pm);
				}
			}
		}
		auto PartDividedSolver::updPartVel()->void { for (auto &pb : activePartBlockPool())s_tva(*pb.part_->pm(), pv() + dynamic::id(pb.row_id_, 0, 1), const_cast<double6&>(pb.part_->vs())); }
		auto PartDividedSolver::updPartAcc()->void { for (auto &pb : activePartBlockPool())s_tv(*pb.part_->pm(), pa() + dynamic::id(pb.row_id_, 0, 1), const_cast<double6&>(pb.part_->as())); }
		PartDividedSolver::~PartDividedSolver() = default;
		PartDividedSolver::PartDividedSolver(const std::string &name, Size max_iter_count, double max_error) :DividedSolver(name, max_iter_count, max_error) {}
		PartDividedSolver::PartDividedSolver(Object &father, const aris::core::XmlElement &xml_ele) : DividedSolver(father, xml_ele) {}
		PartDividedSolver::PartDividedSolver(const PartDividedSolver &other) = default;
		PartDividedSolver::PartDividedSolver(PartDividedSolver &&other) = default;
		PartDividedSolver& PartDividedSolver::operator=(const PartDividedSolver &other) = default;
		PartDividedSolver& PartDividedSolver::operator=(PartDividedSolver &&other) = default;

		struct LltGroundDividedSolver::Imp
		{
			std::vector<double> cct_, ctc_, cct_llt_, cct_x_, cct_b_, ctc_llt_, ctc_x_, ctc_b_;
			BlockData cct_blk_, ctc_blk_;
			BlockData cct_llt_blk_, cct_x_blk_, cct_b_blk_, ctc_llt_blk_, ctc_x_blk_, ctc_b_blk_;
		};
		auto LltGroundDividedSolver::allocateMemory()->void
		{
			DividedSolver::allocateMemory();

			imp_->cct_.resize(pSize() * pSize());
			imp_->cct_llt_.resize(pSize() * pSize());
			imp_->cct_b_.resize(pSize() * 1);
			imp_->cct_x_.resize(pSize() * 1);

			imp_->cct_blk_.resize(pBlkSize().size() * pBlkSize().size());
			imp_->cct_llt_blk_.resize(pBlkSize().size() * pBlkSize().size());
			imp_->cct_b_blk_.resize(pBlkSize().size() * 1);
			imp_->cct_x_blk_.resize(pBlkSize().size() * 1);

			s_blk_map(pBlkSize(), pBlkSize(), imp_->cct_.data(), imp_->cct_blk_);
			s_blk_map(pBlkSize(), pBlkSize(), imp_->cct_llt_.data(), imp_->cct_llt_blk_);
			s_blk_map(pBlkSize(), { 1 }, imp_->cct_b_.data(), imp_->cct_b_blk_);
			s_blk_map(pBlkSize(), { 1 }, imp_->cct_x_.data(), imp_->cct_x_blk_);
		}
		auto LltGroundDividedSolver::kinPos()->void
		{
			setIterCount(0);

			double pm[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
			s_mc(4, 4, pm, const_cast<double *>(*model().ground().pm()));
			for (; iterCount() < maxIterCount(); setIterCount(iterCount() + 1))
			{
				updCm();
				updCp();
				s_blk_mm(pBlkSize(), pBlkSize(), cBlkSize(), cmBlk(), BlockStride{ cBlkSize().size(),1,cSize(),1 }, cmBlk(), T(BlockStride{ cBlkSize().size(),1,cSize(),1 }), imp_->cct_blk_, BlockStride{ pBlkSize().size(),1,pSize(),1 });
				s_blk_mm(pBlkSize(), { 1 }, cBlkSize(), cmBlk(), cpBlk(), imp_->cct_b_blk_);

				setError(s_blk_norm_fro(cBlkSize(), { 1 }, cpBlk(), BlockStride{ 1,1,1,1 }));

				if (error() < maxError()) return;

				s_blk_llt(pBlkSize(), imp_->cct_blk_, BlockStride{ pBlkSize().size(),1,pSize(),1 }, imp_->cct_llt_blk_, BlockStride{ pBlkSize().size(),1,pSize(),1 });
				s_blk_sov_lm(pBlkSize(), { 1 }, imp_->cct_llt_blk_, BlockStride{ pBlkSize().size(),1,pSize(),1 }, imp_->cct_b_blk_, BlockStride{ 1,1,1,1 }, imp_->cct_x_blk_, BlockStride{ 1,1,1,1 });
				s_blk_sov_um(pBlkSize(), { 1 }, imp_->cct_llt_blk_, BlockStride{ pBlkSize().size(),1,pSize(),1 }, imp_->cct_x_blk_, BlockStride{ 1,1,1,1 }, ppBlk(), BlockStride{ 1,1,1,1 });

				updPartPos();
			}
		}
		auto LltGroundDividedSolver::kinVel()->void
		{
			s_fill(6, 1, 0.0, const_cast<double*>(model().ground().vs()));
			
			updCm();
			updCv();

			s_blk_mm(pBlkSize(), pBlkSize(), cBlkSize(), cmBlk(), BlockStride{ cBlkSize().size(),1,cSize(),1 }, cmBlk(), T(BlockStride{ cBlkSize().size(),1,cSize(),1 }), imp_->cct_blk_, BlockStride{ pBlkSize().size(),1,pSize(),1 });
			s_blk_mm(pBlkSize(), { 1 }, cBlkSize(), cmBlk(), cvBlk(), imp_->cct_b_blk_);

			s_blk_llt(pBlkSize(), imp_->cct_blk_, BlockStride{ pBlkSize().size(),1,pSize(),1 }, imp_->cct_llt_blk_, BlockStride{ pBlkSize().size(),1,pSize(),1 });
			s_blk_sov_lm(pBlkSize(), { 1 }, imp_->cct_llt_blk_, BlockStride{ pBlkSize().size(),1,pSize(),1 }, imp_->cct_b_blk_, BlockStride{ 1,1,1,1 }, imp_->cct_x_blk_, BlockStride{ 1,1,1,1 });
			s_blk_sov_um(pBlkSize(), { 1 }, imp_->cct_llt_blk_, BlockStride{ pBlkSize().size(),1,pSize(),1 }, imp_->cct_x_blk_, BlockStride{ 1,1,1,1 }, pvBlk(), BlockStride{ 1,1,1,1 });

			updPartVel();
		}
		auto LltGroundDividedSolver::kinAcc()->void
		{
			s_fill(6, 1, 0.0, const_cast<double *>(model().ground().as()));
			
			updCm();
			updCa();
			s_blk_mm(pBlkSize(), pBlkSize(), cBlkSize(), cmBlk(), BlockStride{ cBlkSize().size(),1,cSize(),1 }, cmBlk(), T(BlockStride{ cBlkSize().size(),1,cSize(),1 }), imp_->cct_blk_, BlockStride{ pBlkSize().size(),1,pSize(),1 });
			s_blk_mm(pBlkSize(), { 1 }, cBlkSize(), cmBlk(), caBlk(), imp_->cct_b_blk_);

			s_blk_llt(pBlkSize(), imp_->cct_blk_, BlockStride{ pBlkSize().size(),1,pSize(),1 }, imp_->cct_llt_blk_, BlockStride{ pBlkSize().size(),1,pSize(),1 });
			s_blk_sov_lm(pBlkSize(), { 1 }, imp_->cct_llt_blk_, BlockStride{ pBlkSize().size(),1,pSize(),1 }, imp_->cct_b_blk_, BlockStride{ 1,1,1,1 }, imp_->cct_x_blk_, BlockStride{ 1,1,1,1 });
			s_blk_sov_um(pBlkSize(), { 1 }, imp_->cct_llt_blk_, BlockStride{ pBlkSize().size(),1,pSize(),1 }, imp_->cct_x_blk_, BlockStride{ 1,1,1,1 }, paBlk(), BlockStride{ 1,1,1,1 });

			updPartAcc();
		}
		auto LltGroundDividedSolver::dynFce()->void
		{
			updIm();
			updCm();
			updPf();
			updPa();

			s_blk_mm(pBlkSize(), pBlkSize(), cBlkSize(), cmBlk(), BlockStride{ cBlkSize().size(),1,cSize(),1 }, cmBlk(), T(BlockStride{ cBlkSize().size(),1,cSize(),1 }), imp_->cct_blk_, BlockStride{ pBlkSize().size(),1,pSize(),1 });
			s_vc(pSize(), pf(), imp_->cct_b_.data());
			s_blk_mma(pBlkSize(), { 1 }, pBlkSize(), -1.0, imBlk(), BlockStride{ pBlkSize().size(),1,pSize(),1 }, paBlk(), BlockStride{ 1,1,1,1 }, imp_->cct_b_blk_, BlockStride{ 1,1,1,1 });

			s_blk_llt(pBlkSize(), imp_->cct_blk_, BlockStride{ pBlkSize().size(),1,pSize(),1 }, imp_->cct_llt_blk_, BlockStride{ pBlkSize().size(),1,pSize(),1 });
			s_blk_sov_lm(pBlkSize(), { 1 }, imp_->cct_llt_blk_, BlockStride{ pBlkSize().size(),1,pSize(),1 }, imp_->cct_b_blk_, BlockStride{ 1,1,1,1 }, imp_->cct_x_blk_, BlockStride{ 1,1,1,1 });
			s_blk_sov_um(pBlkSize(), { 1 }, imp_->cct_llt_blk_, BlockStride{ pBlkSize().size(),1,pSize(),1 }, imp_->cct_x_blk_, BlockStride{ 1,1,1,1 }, imp_->cct_b_blk_, BlockStride{ 1,1,1,1 });
			s_vc(pSize(), imp_->cct_b_.data(), imp_->cct_x_.data());

			s_blk_mm(cBlkSize(), { 1 }, pBlkSize(), cmBlk(), T(BlockStride{ cBlkSize().size(),1,cSize(),1 }), imp_->cct_x_blk_, BlockStride{ 1,1,1,1 }, cfBlk(), BlockStride{ 1,1,1,1 });

			updConstraintFce();
		}
		LltGroundDividedSolver::~LltGroundDividedSolver() = default;
		LltGroundDividedSolver::LltGroundDividedSolver(const std::string &name) :GroundDividedSolver(name) {}
		LltGroundDividedSolver::LltGroundDividedSolver(Object &father, const aris::core::XmlElement &xml_ele) : GroundDividedSolver(father, xml_ele) {}
		LltGroundDividedSolver::LltGroundDividedSolver(const LltGroundDividedSolver &other) = default;
		LltGroundDividedSolver::LltGroundDividedSolver(LltGroundDividedSolver &&other) = default;
		LltGroundDividedSolver& LltGroundDividedSolver::operator=(const LltGroundDividedSolver &other) = default;
		LltGroundDividedSolver& LltGroundDividedSolver::operator=(LltGroundDividedSolver &&other) = default;

		struct LltPartDividedSolver::Imp
		{
			std::vector<double> cct_, ctc_, cct_llt_, cct_x_, cct_b_, ctc_llt_, ctc_x_, ctc_b_;
			BlockData cct_blk_, ctc_blk_;
			BlockData cct_llt_blk_, cct_x_blk_, cct_b_blk_, ctc_llt_blk_, ctc_x_blk_, ctc_b_blk_;
		};
		auto LltPartDividedSolver::allocateMemory()->void
		{
			DividedSolver::allocateMemory();

			imp_->cct_.resize(pSize() * pSize());
			imp_->cct_llt_.resize(pSize() * pSize());
			imp_->cct_b_.resize(pSize() * 1);
			imp_->cct_x_.resize(pSize() * 1);

			imp_->cct_blk_.resize(pBlkSize().size() * pBlkSize().size());
			imp_->cct_llt_blk_.resize(pBlkSize().size() * pBlkSize().size());
			imp_->cct_b_blk_.resize(pBlkSize().size() * 1);
			imp_->cct_x_blk_.resize(pBlkSize().size() * 1);

			s_blk_map(pBlkSize(), pBlkSize(), imp_->cct_.data(), imp_->cct_blk_);
			s_blk_map(pBlkSize(), pBlkSize(), imp_->cct_llt_.data(), imp_->cct_llt_blk_);
			s_blk_map(pBlkSize(), { 1 }, imp_->cct_b_.data(), imp_->cct_b_blk_);
			s_blk_map(pBlkSize(), { 1 }, imp_->cct_x_.data(), imp_->cct_x_blk_);
		}
		auto LltPartDividedSolver::kinPos()->void
		{
			setIterCount(0);

			double pm[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
			s_mc(4, 4, pm, const_cast<double *>(*model().ground().pm()));
			for (; iterCount() < maxIterCount(); setIterCount(iterCount() + 1))
			{
				updCm();
				updCp();
				s_blk_mm(pBlkSize(), pBlkSize(), cBlkSize(), cmBlk(), BlockStride{ cBlkSize().size(),1,cSize(),1 }, cmBlk(), T(BlockStride{ cBlkSize().size(),1,cSize(),1 }), imp_->cct_blk_, BlockStride{ pBlkSize().size(),1,pSize(),1 });
				s_blk_mm(pBlkSize(), { 1 }, cBlkSize(), cmBlk(), cpBlk(), imp_->cct_b_blk_);

				setError(s_blk_norm_fro(cBlkSize(), { 1 }, cpBlk(), BlockStride{ 1,1,1,1 }));

				if (error() < maxError()) return;

				s_blk_llt(pBlkSize(), imp_->cct_blk_, BlockStride{ pBlkSize().size(),1,pSize(),1 }, imp_->cct_llt_blk_, BlockStride{ pBlkSize().size(),1,pSize(),1 });
				s_blk_sov_lm(pBlkSize(), { 1 }, imp_->cct_llt_blk_, BlockStride{ pBlkSize().size(),1,pSize(),1 }, imp_->cct_b_blk_, BlockStride{ 1,1,1,1 }, imp_->cct_x_blk_, BlockStride{ 1,1,1,1 });
				s_blk_sov_um(pBlkSize(), { 1 }, imp_->cct_llt_blk_, BlockStride{ pBlkSize().size(),1,pSize(),1 }, imp_->cct_x_blk_, BlockStride{ 1,1,1,1 }, ppBlk(), BlockStride{ 1,1,1,1 });

				updPartPos();
			}
		}
		auto LltPartDividedSolver::kinVel()->void
		{
			s_fill(6, 1, 0.0, const_cast<double*>(model().ground().vs()));
			
			updCm();
			updCv();

			s_blk_mm(pBlkSize(), pBlkSize(), cBlkSize(), cmBlk(), BlockStride{ cBlkSize().size(),1,cSize(),1 }, cmBlk(), T(BlockStride{ cBlkSize().size(),1,cSize(),1 }), imp_->cct_blk_, BlockStride{ pBlkSize().size(),1,pSize(),1 });
			s_blk_mm(pBlkSize(), { 1 }, cBlkSize(), cmBlk(), cvBlk(), imp_->cct_b_blk_);

			s_blk_llt(pBlkSize(), imp_->cct_blk_, BlockStride{ pBlkSize().size(),1,pSize(),1 }, imp_->cct_llt_blk_, BlockStride{ pBlkSize().size(),1,pSize(),1 });
			s_blk_sov_lm(pBlkSize(), { 1 }, imp_->cct_llt_blk_, BlockStride{ pBlkSize().size(),1,pSize(),1 }, imp_->cct_b_blk_, BlockStride{ 1,1,1,1 }, imp_->cct_x_blk_, BlockStride{ 1,1,1,1 });
			s_blk_sov_um(pBlkSize(), { 1 }, imp_->cct_llt_blk_, BlockStride{ pBlkSize().size(),1,pSize(),1 }, imp_->cct_x_blk_, BlockStride{ 1,1,1,1 }, pvBlk(), BlockStride{ 1,1,1,1 });

			updPartVel();
		}
		auto LltPartDividedSolver::kinAcc()->void
		{
			s_fill(6, 1, 0.0, const_cast<double *>(model().ground().as()));
			
			updCm();
			updCa();
			s_blk_mm(pBlkSize(), pBlkSize(), cBlkSize(), cmBlk(), BlockStride{ cBlkSize().size(),1,cSize(),1 }, cmBlk(), T(BlockStride{ cBlkSize().size(),1,cSize(),1 }), imp_->cct_blk_, BlockStride{ pBlkSize().size(),1,pSize(),1 });
			s_blk_mm(pBlkSize(), { 1 }, cBlkSize(), cmBlk(), caBlk(), imp_->cct_b_blk_);

			s_blk_llt(pBlkSize(), imp_->cct_blk_, BlockStride{ pBlkSize().size(),1,pSize(),1 }, imp_->cct_llt_blk_, BlockStride{ pBlkSize().size(),1,pSize(),1 });
			s_blk_sov_lm(pBlkSize(), { 1 }, imp_->cct_llt_blk_, BlockStride{ pBlkSize().size(),1,pSize(),1 }, imp_->cct_b_blk_, BlockStride{ 1,1,1,1 }, imp_->cct_x_blk_, BlockStride{ 1,1,1,1 });
			s_blk_sov_um(pBlkSize(), { 1 }, imp_->cct_llt_blk_, BlockStride{ pBlkSize().size(),1,pSize(),1 }, imp_->cct_x_blk_, BlockStride{ 1,1,1,1 }, paBlk(), BlockStride{ 1,1,1,1 });

			updPartAcc();
		}
		auto LltPartDividedSolver::dynFce()->void
		{
			updIm();
			updCm();
			updPf();
			updPa();

			s_blk_mm(pBlkSize(), pBlkSize(), cBlkSize(), cmBlk(), BlockStride{ cBlkSize().size(),1,cSize(),1 }, cmBlk(), T(BlockStride{ cBlkSize().size(),1,cSize(),1 }), imp_->cct_blk_, BlockStride{ pBlkSize().size(),1,pSize(),1 });
			s_vc(pSize(), pf(), imp_->cct_b_.data());
			s_blk_mma(pBlkSize(), { 1 }, pBlkSize(), -1.0, imBlk(), BlockStride{ pBlkSize().size(),1,pSize(),1 }, paBlk(), BlockStride{ 1,1,1,1 }, imp_->cct_b_blk_, BlockStride{ 1,1,1,1 });

			s_blk_llt(pBlkSize(), imp_->cct_blk_, BlockStride{ pBlkSize().size(),1,pSize(),1 }, imp_->cct_llt_blk_, BlockStride{ pBlkSize().size(),1,pSize(),1 });
			s_blk_sov_lm(pBlkSize(), { 1 }, imp_->cct_llt_blk_, BlockStride{ pBlkSize().size(),1,pSize(),1 }, imp_->cct_b_blk_, BlockStride{ 1,1,1,1 }, imp_->cct_x_blk_, BlockStride{ 1,1,1,1 });
			s_blk_sov_um(pBlkSize(), { 1 }, imp_->cct_llt_blk_, BlockStride{ pBlkSize().size(),1,pSize(),1 }, imp_->cct_x_blk_, BlockStride{ 1,1,1,1 }, imp_->cct_b_blk_, BlockStride{ 1,1,1,1 });
			s_vc(pSize(), imp_->cct_b_.data(), imp_->cct_x_.data());

			s_blk_mm(cBlkSize(), { 1 }, pBlkSize(), cmBlk(), T(BlockStride{ cBlkSize().size(),1,cSize(),1 }), imp_->cct_x_blk_, BlockStride{ 1,1,1,1 }, cfBlk(), BlockStride{ 1,1,1,1 });

			updConstraintFce();
		}
		LltPartDividedSolver::~LltPartDividedSolver() = default;
		LltPartDividedSolver::LltPartDividedSolver(const std::string &name, Size max_iter_count, double max_error) :PartDividedSolver(name, max_iter_count, max_error) {}
		LltPartDividedSolver::LltPartDividedSolver(Object &father, const aris::core::XmlElement &xml_ele) : PartDividedSolver(father, xml_ele) {}
		LltPartDividedSolver::LltPartDividedSolver(const LltPartDividedSolver &other) = default;
		LltPartDividedSolver::LltPartDividedSolver(LltPartDividedSolver &&other) = default;
		LltPartDividedSolver& LltPartDividedSolver::operator=(const LltPartDividedSolver &other) = default;
		LltPartDividedSolver& LltPartDividedSolver::operator=(LltPartDividedSolver &&other) = default;

		struct DiagSolver::Imp
		{
			std::vector<Relation> relation_pool_;
			std::vector<Part *> part_pool_;
			std::vector<Diag> diag_pool_;
			std::vector<Remainder> remainder_pool_;

			std::vector<double> A_;
			std::vector<double> x_;
			std::vector<double> b_;
			std::vector<double> U_, tau_;
			std::vector<Size> p_;

			Size rows, cols;
		};
		auto DiagSolver::allocateMemory()->void 
		{
			// make active part pool //
			activePartPool().clear();
			for (auto &p : model().partPool())if (p.active())activePartPool().push_back(&p);
			
			// make active constraint pool //
			std::vector<Constraint*> cp;
			for (auto &jnt : model().jointPool())if (jnt.active())cp.push_back(&jnt);
			for (auto &mot : model().motionPool())if (mot.active()) cp.push_back(&mot);
			for (auto &gmt : model().generalMotionPool())if (gmt.active())cp.push_back( &gmt);
			
			// make relation pool //
			relationPool().clear();
			for (auto c : cp)
			{
				auto ret = std::find_if(relationPool().begin(), relationPool().end(), [&c](Relation &relation)
				{
					const auto ri{ relation.prtI }, rj{ relation.prtJ }, ci{ &c->makI().fatherPart() }, cj{ &c->makJ().fatherPart() };
					return ((ri == ci) && (rj == cj)) || ((ri == cj) && (rj == ci));
				});

				if (ret == relationPool().end()) relationPool().push_back(Relation{ &c->makI().fatherPart(), &c->makJ().fatherPart(), c->dim(),{ { c, true } } });
				else
				{
					ret->dim += c->dim();
					ret->cst_pool_.push_back({ c, &c->makI().fatherPart() == ret->prtI });
				}
			}
			
			// adjust order //
			for (Size i = 0; i < std::min(activePartPool().size(), relationPool().size()); ++i)
			{
				// 先对part排序，找出下一个跟上一个part联系的part
				std::sort(activePartPool().begin() + i, activePartPool().end(), [i, this](Part* a, Part* b)
				{
					if (a == &model().ground()) return true;
					if (b == &model().ground()) return false;
					if (i == 0)return a->id() < b->id();
					if (b == relationPool()[i - 1].prtI) return false;
					if (b == relationPool()[i - 1].prtJ) return false;
					if (a == relationPool()[i - 1].prtI) return true;
					if (a == relationPool()[i - 1].prtJ) return true;
					return a->id() < b->id();
				});
				// 再插入连接新part的relation
				std::sort(relationPool().begin() + i, relationPool().end(), [i, this](Relation a, Relation b)
				{
					auto pend = activePartPool().begin() + i + 1;
					auto a_part_i = std::find_if(activePartPool().begin(), pend, [a](Part* p)->bool { return p == a.prtI; });
					auto a_part_j = std::find_if(activePartPool().begin(), pend, [a](Part* p)->bool { return p == a.prtJ; });
					auto b_part_i = std::find_if(activePartPool().begin(), pend, [b](Part* p)->bool { return p == b.prtI; });
					auto b_part_j = std::find_if(activePartPool().begin(), pend, [b](Part* p)->bool { return p == b.prtJ; });

					bool a_is_ok = (a_part_i == pend) != (a_part_j == pend);
					bool b_is_ok = (b_part_i == pend) != (b_part_j == pend);

					if (a_is_ok && !b_is_ok) return true;
					else if (!a_is_ok && b_is_ok) return false;
					else if (a.dim != b.dim)return a.dim > b.dim;
					else return false;
				});
			}
			
			// make diag pool //
			diagPool().clear();
			diagPool().resize(activePartPool().size());
			diagPool().at(0).is_I = true;
			diagPool().at(0).rel = nullptr;
			diagPool().at(0).part = &model().ground();
			diagPool().at(0).rd = &diagPool().at(0);
			std::fill_n(diagPool().at(0).b, 6, 0.0);
			std::fill_n(diagPool().at(0).x, 6, 0.0);
			std::fill_n(diagPool().at(0).cm, 36, 0.0);
			for (Size i{ 0 }; i < 6; ++i)diagPool().at(0).cm[i * 6 + i] = 1.0;
			for (Size i = 1; i < diagPool().size(); ++i)
			{
				diagPool().at(i).rel = &relationPool().at(i - 1);
				diagPool().at(i).is_I = relationPool().at(i - 1).prtI == activePartPool().at(i);
				diagPool().at(i).part = diagPool().at(i).is_I ? relationPool().at(i - 1).prtI : relationPool().at(i - 1).prtJ;
				auto add_part = diagPool().at(i).is_I ? diagPool().at(i).rel->prtJ : diagPool().at(i).rel->prtI;
				diagPool().at(i).rd = &*std::find_if(diagPool().begin(), diagPool().end(), [&](Diag &d) {return d.part == add_part; });
				
			}
			
			// make remainder pool //
			remainderPool().clear();
			remainderPool().resize(relationPool().size() - activePartPool().size() + 1);
			for (Size i = 0; i < remainderPool().size(); ++i) 
			{
				auto &r = remainderPool().at(i);

				r.rel = &relationPool().at(i + diagPool().size() - 1);
				r.cm_blk_series.clear();
				r.cm_blk_series.push_back(Remainder::Block());
				r.cm_blk_series.back().diag = &*std::find_if(diagPool().begin(), diagPool().end(), [&r](Diag&d) {return r.rel->prtI == d.part; });
				r.cm_blk_series.back().is_I = true;
				r.cm_blk_series.push_back(Remainder::Block());
				r.cm_blk_series.back().diag = &*std::find_if(diagPool().begin(), diagPool().end(), [&r](Diag&d) {return r.rel->prtJ == d.part; });
				r.cm_blk_series.back().is_I = false;

				for (auto rd = diagPool().rbegin(); rd < diagPool().rend(); ++rd)
				{
					auto &d = *rd;
					
					// 判断是不是地 //
					if (d.rel)
					{
						auto diag_part = d.is_I ? d.rel->prtI : d.rel->prtJ;
						auto add_part = d.is_I ? d.rel->prtJ : d.rel->prtI;

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
								
								blk.diag = &*std::find_if(diagPool().begin(), diagPool().end(), [&](Diag&d) {return d.part == add_part; });
								
								r.cm_blk_series.push_back(blk);
							}
						}
					}
				}
			}
			
			// allocate memory //
			imp_->rows = 0;
			imp_->cols = 0;
			for (auto &d : diagPool()) 
			{
				d.rows = 0;
				if (d.rel && d.rel->dim < 6)
				{
					d.rows = imp_->rows;
					imp_->rows += 6 - d.rel->dim;
				}
			}
			for (auto &r : remainderPool()) imp_->cols += r.rel->dim ;

			imp_->A_.clear();
			imp_->A_.resize(imp_->rows*imp_->cols, 0.0);
			imp_->x_.clear();
			imp_->x_.resize(std::max(imp_->rows, imp_->cols) * 1, 0.0);
			imp_->b_.clear();
			imp_->b_.resize(std::max(imp_->rows, imp_->cols) * 1, 0.0);
			imp_->U_.clear();
			imp_->U_.resize(imp_->rows*imp_->cols, 0.0);
			imp_->tau_.clear();
			imp_->tau_.resize(std::max(imp_->rows, imp_->cols), 0.0);
			imp_->p_.clear();
			imp_->p_.resize(std::max(imp_->rows, imp_->cols), 0);
		}
		auto DiagSolver::kinPos()->void 
		{
			double pm[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
			s_mc(4, 4, pm, const_cast<double *>(*model().ground().pm()));

			setIterCount(0);
			
			for (; iterCount() < maxIterCount(); setIterCount(iterCount() + 1))
			{
				updDiagCp();
				updRemainderCp();
				
				double error{ 0.0 };
				for (auto d = diagPool().begin() + 1; d < diagPool().end(); ++d)for (Size i{ 0 }; i < d->rel->dim; ++i)error = std::max(error, std::abs(d->b[i]));
				for (auto &r : remainderPool())for (Size i{ 0 }; i < r.rel->dim; ++i)error = std::max(error, std::abs(r.b[i]));
				if (error < maxError())return;

				updDiagCm();
				updRemainderCm();
				updA();
				updB();
				updX();

				for (auto &x : imp_->x_)
				{
					x = std::min(x, std::max(error, 1.0));
					x = std::max(x, std::min(-error, -1.0));
				}

				// 将x写入diag, 重新乘Q, 并反向做行变换
				for (auto d = diagPool().begin() + 1; d<diagPool().end(); ++d)
				{
					double tem[6];
					s_vc(6 - d->rel->dim, imp_->x_.data() + d->rows, d->x + d->rel->dim);
					///////////////////////////////////////
					s_mm(6, 1, 6, d->Q, d->x, tem);
					////
					//s_householder_ut_q_dot(6, d->rel->dim, 1, d->U, d->tau, d->x, tem);
					///////////////////////////////////////
					s_vc(6, tem, d->x);
					s_va(6, d->rd->x, d->x);
				}

				// 将速度更新为矩阵
				for (auto d = diagPool().begin() + 1; d<diagPool().end(); ++d)
				{
					double pm[4][4];
					double pq[7];

					s_vc(6, d->x, pq);

					double theta = s_norm(3, pq + 3);
					pq[6] = std::cos(theta / 2);

					double factor = theta < 1e-4 ? 0.5 : std::sin(theta / 2) / theta;
					s_nv(3, factor, pq + 3);

					s_pq2pm(pq, *pm);

					double final_pm[4][4];
					s_pm2pm(*pm, *d->part->pm(), *final_pm);

					d->part->setPm(*final_pm);
				}
			}
		}
		auto DiagSolver::kinVel()->void 
		{
			s_fill(6, 1, 0.0, const_cast<double *>(model().ground().vs()));
			// make A
			updDiagCm();
			updRemainderCm();
			updA();
			// make b
			updDiagCv();
			updRemainderCv();
			updB();
			// using qr to solve x
			updX();

			// 将x写入diag, 重新乘Q, 并反向做行变换
			for (auto d = diagPool().begin() + 1; d<diagPool().end(); ++d)
			{
				double tem[6];
				s_vc(6 - d->rel->dim, imp_->x_.data() + d->rows, d->x + d->rel->dim);
				///////////////////////////////////////
				s_mm(6, 1, 6, d->Q, d->x, tem);
				////
				//s_householder_ut_q_dot(6, d->rel->dim, 1, d->U, d->tau, d->x, tem);
				///////////////////////////////////////
				s_vc(6, tem, d->x);
				s_va(6, d->rd->x, d->x);
			}

			// 将x更新到杆件速度
			for (auto d = diagPool().begin() + 1; d<diagPool().end(); ++d)
			{
				s_va(6, d->x, const_cast<double*>(d->part->vs()));
			}
		}
		auto DiagSolver::kinAcc()->void 
		{
			s_fill(6, 1, 0.0, const_cast<double *>(model().ground().as()));
			
			// make A
			updDiagCm();
			updRemainderCm();
			updA();
			// make b
			updDiagCa();
			updRemainderCa();
			updB();
			// using qr to solve x
			updX();

			// 将x写入diag, 重新乘Q, 并反向做行变换
			for (auto d = diagPool().begin() + 1; d<diagPool().end(); ++d)
			{
				s_mc(6 - d->rel->dim, 1, imp_->x_.data() + d->rows, d->x + d->rel->dim);
				//////////////////////////////////
				s_mm(6, 1, 6, d->Q, d->x, const_cast<double *>(d->part->as()));
				//////
				//s_householder_ut_q_dot(6, d->rel->dim, 1, d->U, d->tau, d->x, const_cast<double *>(d->part->as()));
				//////////////////////////////////
				s_ma(6, 1, d->rd->part->as(), const_cast<double *>(d->part->as()));
			}
		}
		auto DiagSolver::dynFce()->void 
		{
			// make A
			updDiagCm();
			updRemainderCm();
			updA();
			// make b
			updDiagPf();
			updBf();
			// using qr to solve x
			updXf();
		}
		auto DiagSolver::updDiagCm()->void
		{
			// upd diag cm data //
			for (auto d = diagPool().begin() + 1; d < diagPool().end(); ++d)
			{
				Size pos{ 0 };
				for (auto &c : d->rel->cst_pool_)
				{
					double cm[36];
					double *cmI = d->is_I ? d->cm : cm;
					double *cmJ = d->is_I ? cm : d->cm;

					c.constraint->cptGlbCm(cmI + pos, d->rel->dim, cmJ + pos, d->rel->dim);
					pos += c.constraint->dim();
					
					// make ut and qr
					s_householder_ut(6, d->rel->dim, d->cm, d->U, d->tau);
					s_householder_ut2qr(6, d->rel->dim, d->U, d->tau, d->Q, d->R);
				}
			}
		}
		auto DiagSolver::updDiagCp()->void 
		{
			for (auto d = diagPool().begin() + 1; d < diagPool().end(); ++d)
			{
				Size pos{ 0 };
				for (auto &c : d->rel->cst_pool_)
				{
					c.constraint->cptCp(d->b + pos);
					pos += c.constraint->dim();
				}
			}
		}
		auto DiagSolver::updDiagCv()->void 
		{
			for (auto d = diagPool().begin() + 1; d < diagPool().end(); ++d)
			{
				Size pos{ 0 };
				for (auto &c : d->rel->cst_pool_)
				{
					c.constraint->cptCv(d->b + pos);
					pos += c.constraint->dim();
				}
			}
		}
		auto DiagSolver::updDiagCa()->void 
		{
			for (auto d = diagPool().begin() + 1; d < diagPool().end(); ++d)
			{
				Size pos{ 0 };
				for (auto &c : d->rel->cst_pool_)
				{
					c.constraint->cptCa(d->b + pos);
					pos += c.constraint->dim();
				}
			}
		}
		auto DiagSolver::updDiagPf()->void 
		{
			for (auto d = diagPool().begin() + 1; d < diagPool().end(); ++d)
			{
				double prt_as[6], prt_f[6];
				
				d->part->cptPrtPf(prt_f);
				d->part->cptPrtAs(prt_as);
				s_mms(6, 1, 6, *d->part->prtIm(), prt_as, prt_f);
				s_tf(*d->part->pm(), prt_f, d->b);
			}
		}
		auto DiagSolver::updRemainderCm()->void
		{
			// upd remainder data //
			for (auto &r : remainderPool())
			{
				Size pos{ 0 };
				for (auto &c : r.rel->cst_pool_)
				{
					c.constraint->cptGlbCm(r.cmI + pos, r.rel->dim, r.cmJ + pos, r.rel->dim);
					pos += c.constraint->dim();
				}
			}
		}
		auto DiagSolver::updRemainderCp()->void
		{
			// upd remainder data //
			for (auto &r : remainderPool())
			{
				Size pos{ 0 };
				for (auto &c : r.rel->cst_pool_)
				{
					c.constraint->cptCp(r.b + pos);
					pos += c.constraint->dim();
				}
			}
		}
		auto DiagSolver::updRemainderCv()->void
		{
			// upd remainder data //
			for (auto &r : remainderPool())
			{
				Size pos{ 0 };
				for (auto &c : r.rel->cst_pool_)
				{
					c.constraint->cptCv(r.b + pos);
					pos += c.constraint->dim();
				}
			}
		}
		auto DiagSolver::updRemainderCa()->void
		{
			// upd remainder data //
			for (auto &r : remainderPool())
			{
				Size pos{ 0 };
				for (auto &c : r.rel->cst_pool_)
				{
					c.constraint->cptCa(r.b + pos);
					pos += c.constraint->dim();
				}
			}
		}
		auto DiagSolver::updA()->void
		{
			Size cols{ 0 };
			for (auto &r : remainderPool())
			{
				for (auto &b : r.cm_blk_series)
				{
					/////////////////////////////
					s_mm(6 - b.diag->rel->dim, r.rel->dim, 6, b.diag->Q + dynamic::id(0, b.diag->rel->dim, 6), ColMajor{ 6 }, b.is_I ? r.cmI : r.cmJ, r.rel->dim, imp_->A_.data() + dynamic::id(b.diag->rows, cols, imp_->cols), imp_->cols);
					/////////////////////////////
					//double tem[36];
					//s_householder_ut_qt_dot(6, b.diag->rel->dim, r.rel->dim, b.diag->U, b.diag->tau, b.is_I ? r.cmI : r.cmJ, tem);
					//s_mc(6 - b.diag->rel->dim, r.rel->dim, tem + dynamic::id(b.diag->rel->dim, 0, r.rel->dim), r.rel->dim, imp_->A_.data() + dynamic::id(b.diag->rows, cols, imp_->cols), imp_->cols);
					/////////////////////////////
				}
				cols += r.rel->dim;
			}
		}
		auto DiagSolver::updB()->void
		{
			// 求解对角线上的未知数
			for (auto d = diagPool().begin() + 1; d<diagPool().end(); ++d)
			{
				s_fill(6, 1, 0.0, d->x);
				s_sov_lm(d->rel->dim, 1, d->U, ColMajor{ d->rel->dim }, d->b, 1, d->x, 1);
			}
			// 使用已求出的未知数，用以构建b
			Size cols{ 0 };
			for (auto &r : remainderPool())
			{
				for (auto &b : r.cm_blk_series)
				{
					double tem[6];
					auto cm = b.is_I ? r.cmJ : r.cmI;//这里是颠倒的，因为加到右侧需要乘-1.0
					/////////////////////////////////////////////
					s_mm(6, 1, b.diag->rel->dim, b.diag->Q, 6, b.diag->x, 1, tem, 1);
					//s_householder_ut_q_dot(6, b.diag->rel->dim, 1, b.diag->U, b.diag->tau, b.diag->x, tem);
					/////////////////////////////////////////////
					s_mma(r.rel->dim, 1, 6, cm, ColMajor{ r.rel->dim }, tem, 1, r.b, 1);
				}
				s_mc(r.rel->dim, 1, r.b, imp_->b_.data() + cols);
				cols += r.rel->dim;
			}
		}
		auto DiagSolver::updX()->void
		{
			auto &a = imp_;
			
			// 求解x
			//s_householder_ut(imp_->cols, imp_->rows, imp_->A_.data(), ColMajor{ imp_->cols }, imp_->U_.data(), ColMajor{ imp_->cols }, imp_->tau_.data(), 1);
			//s_householder_ut_sov(imp_->cols, imp_->rows, 1, imp_->U_.data(), ColMajor{ imp_->cols }, imp_->tau_.data(), 1, imp_->b_.data(), 1, imp_->x_.data(), 1);

			s_householder_utp(imp_->cols, imp_->rows, imp_->A_.data(), ColMajor{ imp_->cols }, imp_->U_.data(), ColMajor{ imp_->cols }, imp_->tau_.data(), 1, imp_->p_.data());
			s_householder_utp_sov(imp_->cols, imp_->rows, 1, imp_->U_.data(), ColMajor{ imp_->cols }, imp_->tau_.data(), 1, imp_->p_.data(), imp_->b_.data(), 1, imp_->x_.data(), 1);
		}
		auto DiagSolver::updBf()->void
		{
			for (auto d = diagPool().rbegin(); d<diagPool().rend() -1; ++d)
			{
				// 做行变换
				s_va(6, d->b, d->rd->b);

				// dot Q //
				double tem[6];
				//////////////////////////////////////
				s_mm(6, 1, 6, d->Q, ColMajor{ 6 }, d->b, 1, tem, 1);
				//s_householder_ut_qt_dot(6, d->rel->dim, 1, d->U, d->tau, d->b, tem);
				//////////////////////////////////////
				s_vc(6, tem, d->b);
				s_vc(6 - d->rel->dim, d->b + d->rel->dim, imp_->b_.data() + d->rows);
			}
		}
		auto DiagSolver::updXf()->void
		{
			//s_householder_ut(imp_->rows, imp_->cols, imp_->A_.data(), imp_->U_.data(), imp_->tau_.data(), maxError());
			//s_householder_ut_sov(imp_->rows, imp_->cols, 1, imp_->U_.data(), imp_->tau_.data(), imp_->b_.data(), imp_->x_.data(), maxError());

			s_householder_utp(imp_->rows, imp_->cols, imp_->A_.data(), imp_->U_.data(), imp_->tau_.data(), imp_->p_.data(), maxError());
			s_householder_utp_sov(imp_->rows, imp_->cols, 1, imp_->U_.data(), imp_->tau_.data(), imp_->p_.data(), imp_->b_.data(), imp_->x_.data(), maxError());

			// 将已经求出的x更新到remainder中，此后将已知数移到右侧
			Size cols{ 0 };
			for (auto &r : remainderPool())
			{
				Size pos{ 0 };
				for (auto &c : r.rel->cst_pool_)
				{
					c.constraint->setCf(imp_->x_.data() + cols + pos);
					pos += c.constraint->dim();
				}
				for (auto &b : r.cm_blk_series)
				{
					double tem[6];
					s_mm(6, 1, r.rel->dim, b.is_I ? r.cmJ : r.cmI, imp_->x_.data() + cols, tem);
					/////////////////////////////////////////
					s_mma(6, 1, 6, b.diag->Q, ColMajor{ 6 }, tem, 1, b.diag->b, 1);
					/////////////////////////////////////////
					//double tem2[6];
					//s_householder_ut_qt_dot(6, b.diag->rel->dim, 1, b.diag->U, b.diag->tau, tem, tem2);
					//s_ma(6, 1, tem2, b.diag->b);
					////////////////////////////////////////
				}
				
				cols += r.rel->dim;
			}

			for (auto d = diagPool().begin() + 1; d < diagPool().end(); ++d)
			{
				s_sov_um(d->rel->dim, 1, d->U, d->b, d->x);
				Size pos{ 0 };
				for (auto &c : d->rel->cst_pool_)
				{
					c.constraint->setCf(d->x + pos);
					pos += c.constraint->dim();
				}
			}

		}
		auto DiagSolver::relationPool()->std::vector<Relation>& { return imp_->relation_pool_; }
		auto DiagSolver::activePartPool()->std::vector<Part*>& { return imp_->part_pool_; }
		auto DiagSolver::diagPool()->std::vector<Diag>& { return imp_->diag_pool_; }
		auto DiagSolver::remainderPool()->std::vector<Remainder>& { return imp_->remainder_pool_; }
		auto DiagSolver::plotRelation()->void
		{
			std::size_t name_size{ 0 };
			for (auto prt : activePartPool())
			{
				name_size = std::max(prt->name().size(), name_size);
			}
			
			for (auto prt : activePartPool())
			{
				std::string s(name_size, ' ');
				s.replace(0, prt->name().size(), prt->name().data());

				std::cout << s << ":";
				
				if (prt == &model().ground())
				{
					std::cout << "  6x6 ";
				}
				else
					std::cout << "      ";



				for (auto &rel : relationPool())
				{
					std::cout << " ";
					if (rel.prtI == prt)
						std::cout << " 6x" << rel.dim;
					else if (rel.prtJ == prt)
						std::cout << "-6x" << rel.dim;
					else
						std::cout << "    ";

					std::cout << " ";
				}
				std::cout << std::endl;
			}
		}
		auto DiagSolver::plotDiag()->void
		{
			std::size_t name_size{ 0 };
			for (auto prt : activePartPool())
			{
				name_size = std::max(prt->name().size(), name_size);
			}
			
			for (auto prt : activePartPool())
			{
				std::string s(name_size, ' ');
				s.replace(0, prt->name().size(), prt->name().data());

				std::cout << s << ":";
				
				
				
				
				for (auto &d : diagPool())
				{
					if (d.rel == nullptr)
					{
						if (prt == &model().ground())
						{
							std::cout << "  6x6 ";
						}
						else
							std::cout << "      ";

						continue;
					}

					auto &rel = *d.rel;
					std::cout << " ";
					if (d.is_I && rel.prtI == prt)
					{
						std::cout << " 6x" << rel.dim;
					}
					else if (!d.is_I && rel.prtJ == prt)
					{
						std::cout << "-6x" << rel.dim;
					}
					else
						std::cout << "    ";
					std::cout << " ";

				}
				std::cout << std::endl;
			}
		}
		auto DiagSolver::plotRemainder()->void
		{
			std::size_t name_size{ 0 };
			for (auto prt : activePartPool())
			{
				name_size = std::max(prt->name().size(), name_size);
			}
			
			for (auto prt : activePartPool())
			{
				std::string s(name_size, ' ');
				s.replace(0, prt->name().size(), prt->name().data());

				std::cout << s << ":";

				for (auto &r : remainderPool())
				{
					std::cout << " ";

					bool found{ false };
					for (auto blk : r.cm_blk_series)
					{
						if (prt == blk.diag->part)
						{
							found = true;
							if (blk.is_I)
								std::cout << " 6x" << r.rel->dim;
							else
								std::cout << "-6x" << r.rel->dim;
						}
					}
					if (!found)std::cout << "    ";

					std::cout << " ";
				}
				std::cout << std::endl;
			}
		}
		DiagSolver::~DiagSolver() = default;
		DiagSolver::DiagSolver(const std::string &name, Size max_iter_count, double max_error) :Solver(name, max_iter_count, max_error){}
		DiagSolver::DiagSolver(Object &father, const aris::core::XmlElement &xml_ele) : Solver(father, xml_ele){}
		DiagSolver::DiagSolver(const DiagSolver &other) = default;
		DiagSolver::DiagSolver(DiagSolver &&other) = default;
		DiagSolver& DiagSolver::operator=(const DiagSolver &other) = default;
		DiagSolver& DiagSolver::operator=(DiagSolver &&other) = default;
	}
}
