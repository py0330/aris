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
#include <numeric>
#include <deque>
#include <array>

#include "aris/dynamic/model.hpp"
#include "aris/plan/root.hpp"

namespace aris::dynamic{
	struct Calibrator::Imp{
		struct ConstraintBlock	{
			Constraint *c;
			Size ri, rj, col;
		};
		struct ForceBlock{
			Force *f;
			Size ri, rj;
		};
		std::vector<ConstraintBlock> cst_blk_vec_;
		std::vector<ForceBlock> fce_blk_vec_;

		Size dyn_m_{ 0 }, dyn_n_{ 0 };
		std::vector<double> C_, C_inv_, U_, tau_, Q_, R_, B_, D_, f_;
		std::vector<Size> p_;

		Size m_{ 0 }, g_{ 0 }, k_{ 0 };
		std::vector<double> A_, x_, b_;

		std::vector<double> torque_constant_, velocity_ratio_, torque_weight_, velocity_dead_zone_;
		int pos_idx_{ 0 }, vel_idx_{ 1 }, fce_idx_{ 2 }, data_num_per_motor_{ 3 };
		int filter_window_size_{ 20 };
		double tolerable_variance_{ 0.05 };

		std::string verify_result_path;

		friend auto makeDataset(const Calibrator *clb, const std::vector<double> &mtx, std::vector< std::vector<std::vector<double> >*> &dataset);
	};
	auto Calibrator::m()->Size { return imp_->m_; }
	auto Calibrator::g()->Size { return imp_->g_; }
	auto Calibrator::k()->Size { return imp_->k_; }
	auto Calibrator::A()->double* { return imp_->A_.data(); }
	auto Calibrator::x()->double* { return imp_->x_.data(); }
	auto Calibrator::b()->double* { return imp_->b_.data(); }
	auto Calibrator::allocateMemory()->void{
		imp_->m_ = 0;
		imp_->g_ = 0;
		imp_->k_ = 0;

		for (auto &m : model()->motionPool()){
			if (m.active())	{
				imp_->m_++;
				imp_->k_ += 3;
			}
		}
		for (auto &p : model()->partPool())	{
			if (p.active() && &p != &model()->ground())	{
				imp_->g_ += 10;
			}
		}
		imp_->A_.clear();
		imp_->A_.resize(m()*n(), 0.0);
		imp_->x_.clear();
		imp_->x_.resize(n(), 0.0);
		imp_->b_.clear();
		imp_->b_.resize(m(), 0.0);

		imp_->dyn_m_ = 0;
		imp_->dyn_n_ = 6;
		std::vector<Part*> active_parts;
		for (auto &prt : model()->partPool()){
			if (prt.active()){
				active_parts.push_back(&prt);
				imp_->dyn_m_ += 6;
			}
		}

		imp_->cst_blk_vec_.clear();
		for (auto &jnt : model()->jointPool()){
			if (jnt.active()){
				imp_->cst_blk_vec_.push_back(Imp::ConstraintBlock
					{
						&jnt,
						static_cast<Size>((std::find(active_parts.begin(), active_parts.end(), &jnt.makI()->fatherPart()) - active_parts.begin()) * 6),
						static_cast<Size>((std::find(active_parts.begin(), active_parts.end(), &jnt.makJ()->fatherPart()) - active_parts.begin()) * 6),
						imp_->dyn_n_
					});
				imp_->dyn_n_ += jnt.dim();
			}
		}
		for (auto &mot : model()->motionPool())	{
			if (mot.active()){
				imp_->cst_blk_vec_.push_back(Imp::ConstraintBlock
					{
						&mot,
						static_cast<Size>((std::find(active_parts.begin(), active_parts.end(), &mot.makI()->fatherPart()) - active_parts.begin()) * 6),
						static_cast<Size>((std::find(active_parts.begin(), active_parts.end(), &mot.makJ()->fatherPart()) - active_parts.begin()) * 6),
						imp_->dyn_n_
					});
				imp_->dyn_n_ += mot.dim();
			}
		}
		for (auto &gmt : model()->generalMotionPool()){
			if (gmt.active()){
				imp_->cst_blk_vec_.push_back(Imp::ConstraintBlock
					{
						&gmt,
						static_cast<Size>((std::find(active_parts.begin(), active_parts.end(), &gmt.makI()->fatherPart()) - active_parts.begin()) * 6),
						static_cast<Size>((std::find(active_parts.begin(), active_parts.end(), &gmt.makJ()->fatherPart()) - active_parts.begin()) * 6),
						imp_->dyn_n_
					});
				imp_->dyn_n_ += 6;
			}
		}

		imp_->fce_blk_vec_.clear();
		for (auto &fce : model()->forcePool()){
			if (fce.active()){
				imp_->fce_blk_vec_.push_back(Imp::ForceBlock
					{
						&fce,
						static_cast<Size>((std::find(active_parts.begin(), active_parts.end(), &fce.makI()->fatherPart()) - active_parts.begin()) * 6),
						static_cast<Size>((std::find(active_parts.begin(), active_parts.end(), &fce.makJ()->fatherPart()) - active_parts.begin()) * 6)
					});
			}
		}

		imp_->f_.clear();
		imp_->f_.resize(imp_->dyn_m_, 0.0);

		imp_->C_.clear();
		imp_->C_.resize(imp_->dyn_m_*imp_->dyn_n_, 0.0);
		imp_->U_.clear();
		imp_->U_.resize(imp_->dyn_m_*imp_->dyn_n_, 0.0);
		imp_->tau_.clear();
		imp_->tau_.resize(std::max(imp_->dyn_m_, imp_->dyn_n_), 0.0);
		imp_->p_.clear();
		imp_->p_.resize(std::max(imp_->dyn_m_, imp_->dyn_n_), 0);

		imp_->C_inv_.clear();
		imp_->C_inv_.resize(imp_->dyn_n_*imp_->dyn_m_, 0.0);
		imp_->R_.clear();
		imp_->R_.resize(imp_->dyn_m_*imp_->dyn_n_, 0.0);
		imp_->Q_.clear();
		imp_->Q_.resize(imp_->dyn_m_*imp_->dyn_m_, 0.0);

		imp_->B_.clear();
		imp_->B_.resize(imp_->m_*imp_->dyn_m_, 0.0);
		imp_->D_.clear();
		imp_->D_.resize(imp_->m_*imp_->dyn_m_, 0.0);
	}
	auto Calibrator::clb()->void{
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
		for (int i = 0; i < 6; ++i)	{
			C[at(i, i, imp_->dyn_n_)] = 1;
		}
		for (auto &b : imp_->cst_blk_vec_)	{
			b.c->cptPrtCm(C + at(b.ri, b.col, imp_->dyn_n_), imp_->dyn_n_, C + at(b.rj, b.col, imp_->dyn_n_), imp_->dyn_n_);
		}

		// make C inv //
		Size rank;
		s_householder_utp(imp_->dyn_m_, imp_->dyn_n_, C, U, t, p, rank);
		s_householder_ut2qr(imp_->dyn_m_, imp_->dyn_n_, U, t, Q, R);
		s_sov_um(rank, imp_->dyn_m_, R, imp_->dyn_n_, Q, T(imp_->dyn_m_), C_inv, imp_->dyn_m_);
		s_permutate_inv(imp_->dyn_n_, imp_->dyn_m_, p, C_inv);

		// make D //
		Size di = 0;
		for (auto &b : imp_->cst_blk_vec_){
			if (dynamic_cast<Motion*>(b.c)){
				s_mc(1, imp_->dyn_m_, C_inv + at(b.col, 0, imp_->dyn_m_), D + at(di, 0, imp_->dyn_m_));
				di++;
			}
		}

		// make B //
		for (auto &b : imp_->cst_blk_vec_){
			Size row{ 0 };
			for (auto &p : model()->partPool()){
				double cm[6][6], vs[6];

				s_inv_tv(*p.pm(), p.vs(), vs);
				s_cmf(vs, *cm);
				s_mm(imp_->m_, 6, 6, D + at(0, row, imp_->dyn_m_), imp_->dyn_m_, *cm, 6, B + at(0, row, imp_->dyn_m_), imp_->dyn_m_);

				row += 6;
			}
		}

		// make A finally //
		Size col1 = 0, col2 = 6;
		for (auto &prt : model()->partPool()){
			if (prt.active() && &prt != &model()->ground()){
				double q[6]{ 0 };

				s_inv_tv(*prt.pm(), prt.as(), q);
				s_inv_tva(-1.0, *prt.pm(), model()->environment().gravity(), q);

				double v[6];
				s_inv_tv(*prt.pm(), prt.vs(), v);

				for (std::size_t j = 0; j < imp_->m_; ++j){
					A[at(j, col1 + 0, n())] = D[at(j, col2 + 0, imp_->dyn_m_)] * q[0] + D[at(j, col2 + 1, imp_->dyn_m_)] * q[1] + D[at(j, col2 + 2, imp_->dyn_m_)] * q[2];
					A[at(j, col1 + 1, n())] = D[at(j, col2 + 1, imp_->dyn_m_)] * q[5] + D[at(j, col2 + 5, imp_->dyn_m_)] * q[1] - D[at(j, col2 + 2, imp_->dyn_m_)] * q[4] - D[at(j, col2 + 4, imp_->dyn_m_)] * q[2];
					A[at(j, col1 + 2, n())] = D[at(j, col2 + 2, imp_->dyn_m_)] * q[3] + D[at(j, col2 + 3, imp_->dyn_m_)] * q[2] - D[at(j, col2 + 0, imp_->dyn_m_)] * q[5] - D[at(j, col2 + 5, imp_->dyn_m_)] * q[0];
					A[at(j, col1 + 3, n())] = D[at(j, col2 + 0, imp_->dyn_m_)] * q[4] + D[at(j, col2 + 4, imp_->dyn_m_)] * q[0] - D[at(j, col2 + 1, imp_->dyn_m_)] * q[3] - D[at(j, col2 + 3, imp_->dyn_m_)] * q[1];
					A[at(j, col1 + 4, n())] = D[at(j, col2 + 3, imp_->dyn_m_)] * q[3];
					A[at(j, col1 + 5, n())] = D[at(j, col2 + 4, imp_->dyn_m_)] * q[4];
					A[at(j, col1 + 6, n())] = D[at(j, col2 + 5, imp_->dyn_m_)] * q[5];
					A[at(j, col1 + 7, n())] = D[at(j, col2 + 3, imp_->dyn_m_)] * q[4] + D[at(j, col2 + 4, imp_->dyn_m_)] * q[3];
					A[at(j, col1 + 8, n())] = D[at(j, col2 + 3, imp_->dyn_m_)] * q[5] + D[at(j, col2 + 5, imp_->dyn_m_)] * q[3];
					A[at(j, col1 + 9, n())] = D[at(j, col2 + 4, imp_->dyn_m_)] * q[5] + D[at(j, col2 + 5, imp_->dyn_m_)] * q[4];

					A[at(j, col1 + 0, n())] += B[at(j, col2 + 0, imp_->dyn_m_)] * v[0] + B[at(j, col2 + 1, imp_->dyn_m_)] * v[1] + B[at(j, col2 + 2, imp_->dyn_m_)] * v[2];
					A[at(j, col1 + 1, n())] += B[at(j, col2 + 1, imp_->dyn_m_)] * v[5] + B[at(j, col2 + 5, imp_->dyn_m_)] * v[1] - B[at(j, col2 + 2, imp_->dyn_m_)] * v[4] - B[at(j, col2 + 4, imp_->dyn_m_)] * v[2];
					A[at(j, col1 + 2, n())] += B[at(j, col2 + 2, imp_->dyn_m_)] * v[3] + B[at(j, col2 + 3, imp_->dyn_m_)] * v[2] - B[at(j, col2 + 0, imp_->dyn_m_)] * v[5] - B[at(j, col2 + 5, imp_->dyn_m_)] * v[0];
					A[at(j, col1 + 3, n())] += B[at(j, col2 + 0, imp_->dyn_m_)] * v[4] + B[at(j, col2 + 4, imp_->dyn_m_)] * v[0] - B[at(j, col2 + 1, imp_->dyn_m_)] * v[3] - B[at(j, col2 + 3, imp_->dyn_m_)] * v[1];
					A[at(j, col1 + 4, n())] += B[at(j, col2 + 3, imp_->dyn_m_)] * v[3];
					A[at(j, col1 + 5, n())] += B[at(j, col2 + 4, imp_->dyn_m_)] * v[4];
					A[at(j, col1 + 6, n())] += B[at(j, col2 + 5, imp_->dyn_m_)] * v[5];
					A[at(j, col1 + 7, n())] += B[at(j, col2 + 3, imp_->dyn_m_)] * v[4] + B[at(j, col2 + 4, imp_->dyn_m_)] * v[3];
					A[at(j, col1 + 8, n())] += B[at(j, col2 + 3, imp_->dyn_m_)] * v[5] + B[at(j, col2 + 5, imp_->dyn_m_)] * v[3];
					A[at(j, col1 + 9, n())] += B[at(j, col2 + 4, imp_->dyn_m_)] * v[5] + B[at(j, col2 + 5, imp_->dyn_m_)] * v[4];
				}
				col1 += 10;
				col2 += 6;
			}
		}
		// make A from frictions //
		Size ai = 0;
		for (auto &b : imp_->cst_blk_vec_){
			if (dynamic_cast<Motion*>(b.c))	{
				A[at(ai, g() + ai * 3 + 0, n())] = s_sgn(dynamic_cast<Motion*>(b.c)->mv(), dynamic_cast<Motion*>(b.c)->frcZeroCheck());
				A[at(ai, g() + ai * 3 + 1, n())] = dynamic_cast<Motion*>(b.c)->mv();
				A[at(ai, g() + ai * 3 + 2, n())] = dynamic_cast<Motion*>(b.c)->ma();
				ai++;
			}
		}


		/////////////////////////////////////////// make b ///////////////////////////////////////
		Size bi{ 0 };
		for (auto &blk : imp_->cst_blk_vec_){
			if (dynamic_cast<Motion*>(blk.c)){
				b[bi] = dynamic_cast<Motion*>(blk.c)->mf();
				bi++;
			}
		}
		std::fill(f, f + imp_->dyn_m_, 0.0);
		for (auto &fce : imp_->fce_blk_vec_){
			double glb_fsI[6], glb_fsJ[6];
			fce.f->cptGlbFs(glb_fsI, glb_fsJ);

			double prt_fsI[6], prt_fsJ[6];
			s_inv_tf(*fce.f->makI()->fatherPart().pm(), glb_fsI, prt_fsI);
			s_inv_tf(*fce.f->makJ()->fatherPart().pm(), glb_fsJ, prt_fsJ);

			s_va(6, prt_fsI, f + fce.ri);
			s_va(6, prt_fsJ, f + fce.rj);
		}

		s_mma(m(), 1, imp_->dyn_m_, D, f, b);

		/////////////////////////////////////////// make x ///////////////////////////////////////
		Size xi = 0;
		for (auto &prt : model()->partPool()){
			if (prt.active() && &prt != &model()->ground())	{
				s_vc(10, prt.prtIv(), x + xi);
				xi += 10;
			}
		}
		// make x from frictions //
		bi = 0;
		for (auto &blk : imp_->cst_blk_vec_){
			if (dynamic_cast<Motion*>(blk.c)){
				s_vc(3, dynamic_cast<Motion*>(blk.c)->frcCoe(), x + g() + bi * 3);
				bi++;
			}
		}
	}
	auto Calibrator::setDataIndex(int pos_idx, int vel_idx, int fce_idx, int data_num_per_motor)->void {
		imp_->pos_idx_ = pos_idx;
		imp_->vel_idx_ = vel_idx;
		imp_->fce_idx_ = fce_idx;
		imp_->data_num_per_motor_ = data_num_per_motor;
	}
	auto Calibrator::setFilterWindowSize(int window_size)->void {
		imp_->filter_window_size_ = window_size;
	}
	auto Calibrator::dataIndex()const->std::tuple<int, int, int, int> { return std::make_tuple(imp_->pos_idx_, imp_->vel_idx_, imp_->fce_idx_, imp_->data_num_per_motor_); }
	auto Calibrator::filterWindowSize()const->int { return imp_->filter_window_size_; }
	auto Calibrator::torqueConstant()const->std::vector<double> { return imp_->torque_constant_; }
	auto Calibrator::setTorqueConstant(std::vector<double> constant)->void { imp_->torque_constant_ = constant; }
	auto Calibrator::velocityRatio()const->std::vector<double> { return imp_->velocity_ratio_; }
	auto Calibrator::setVelocityRatio(std::vector<double> constant)->void { imp_->velocity_ratio_ = constant; }
	auto Calibrator::torqueWeight()const->std::vector<double> { return imp_->torque_weight_; }
	auto Calibrator::setTorqueWeight(std::vector<double> constant)->void { imp_->torque_weight_ = constant; }
	auto Calibrator::velocityDeadZone()const->std::vector<double> { return imp_->velocity_dead_zone_; }
	auto Calibrator::setVelocityDeadZone(std::vector<double> constant)->void { imp_->velocity_dead_zone_ = constant; }
	auto Calibrator::tolerableVariance()const->double { return imp_->tolerable_variance_; }
	auto Calibrator::setTolerableVariance(double constant)->void { imp_->tolerable_variance_ = constant; }

	auto makeDataset(const Calibrator *clb, const std::vector<double> &mtx, std::vector< std::vector<std::vector<double> >*> &dataset){
		const auto[pos_at, vel_at, fce_at, mot_data_num] = clb->dataIndex();
		const Size filter_size = clb->filterWindowSize();
		const Size mot_num = clb->model()->motionPool().size();
		const Size line_num = mot_num * mot_data_num;
		const auto torque_constant = clb->torqueConstant();
		const double dt = 0.001;

		auto &pos = *dataset[0];
		auto &vel = *dataset[1];
		auto &acc = *dataset[2];
		auto &fce = *dataset[3];

		// 预分配内存，为当前数据叠加新的数据 //
		auto num = mtx.size() / line_num / filter_size - 2;
		for (Size i = 0; i < mot_num; ++i){
			pos[i].reserve(num + pos[i].size());
			vel[i].reserve(num + vel[i].size());
			acc[i].reserve(num + acc[i].size());
			fce[i].reserve(num + fce[i].size());
		}

		for (Size i = 1; i < num + 1; ++i){
			for (Size j = 0; j < mot_num; ++j){
				// make actual pos //
				pos[j].push_back(0.0);
				for (Size k = 0; k < filter_size; ++k){
					pos[j].back() += mtx[i * line_num * filter_size + k * line_num + j * mot_data_num + pos_at] / filter_size;
				}

				// make actual vel //
				vel[j].push_back(0.0);
				for (Size k = 0; k < filter_size; ++k){
					vel[j].back() += mtx[i * line_num * filter_size + k * line_num + j * mot_data_num + vel_at] / filter_size * clb->velocityRatio()[j];
				}

				// make actual acc //
				const Size avg_size = filter_size / 2;

				double r = mtx[i * line_num * filter_size + j * mot_data_num + vel_at + line_num * filter_size] / (avg_size * 2 - 1);
				double l = mtx[i * line_num * filter_size + j * mot_data_num + vel_at] / (avg_size * 2 - 1);
				for (Size k = 1; k < avg_size; ++k){
					r += mtx[i * line_num * filter_size + j * mot_data_num + vel_at + line_num * k + line_num * filter_size ] / (avg_size * 2 - 1);
					r += mtx[i * line_num * filter_size + j * mot_data_num + vel_at - line_num * k + line_num * filter_size ] / (avg_size * 2 - 1);
					l += mtx[i * line_num * filter_size + j * mot_data_num + vel_at + line_num * k] / (avg_size * 2 - 1);
					l += mtx[i * line_num * filter_size + j * mot_data_num + vel_at - line_num * k] / (avg_size * 2 - 1);
				}
				acc[j].push_back((r - l) * 1.0 / dt / filter_size * clb->velocityRatio()[j]);

				// make actual fce //
				fce[j].push_back(0.0);
				for (int k = 0; k < filter_size; ++k){
					fce[j].back() += mtx[i * line_num * filter_size + k * line_num + j * mot_data_num + fce_at] / filter_size * torque_constant[j];
				}
			}
		}
	}
	auto Calibrator::clbFiles(const std::vector<std::string> &file_paths)->int{
		// init some values //
		if (imp_->torque_constant_.empty())imp_->torque_constant_.resize(model()->motionPool().size(), 1.0);
		if (imp_->velocity_ratio_.empty()) imp_->velocity_ratio_.resize(model()->motionPool().size(), 1.0);
		if (imp_->torque_weight_.empty())  imp_->torque_weight_.resize(model()->motionPool().size(), 1.0);
		if (imp_->velocity_dead_zone_.empty())imp_->velocity_dead_zone_.resize(model()->motionPool().size(), 0.1);
		
		// make datasets //
		//std::cout << "making datasets" << std::endl;
		std::vector<std::vector<double> > pos(model()->motionPool().size());
		std::vector<std::vector<double> > vel(model()->motionPool().size());
		std::vector<std::vector<double> > acc(model()->motionPool().size());
		std::vector<std::vector<double> > fce(model()->motionPool().size());

		for (auto &file : file_paths){
			std::cout << "clb----loading file:" << file << std::endl;
			auto mtx = aris::dynamic::dlmread(file.c_str());
			std::vector<std::vector<std::vector<double> > *> dataset{ &pos, &vel, &acc, &fce };
			makeDataset(this, mtx, dataset);
		}
		
		// make calibration matrix //
		std::cout << "clb----computing data" << std::endl;
		this->allocateMemory();

		auto num = pos[0].size();
		std::vector<double> A(num * m() * n()), b(num * m(), 0.0), tau(num * m(), 0.0), x(num * m(), 0.0);
		std::vector<aris::Size> p(num * m());

		Size rows{ 0 }, cols{ n() };
		for (int i = 0; i < num; ++i){
			for (int j = 0; j < model()->motionPool().size(); ++j){
				this->model()->motionPool()[j].setP(&pos[j][i]);
				this->model()->motionPool()[j].setV(&vel[j][i]);
				this->model()->motionPool()[j].setA(&acc[j][i]);
			}

			this->model()->solverPool().at(1).kinPos();
			this->model()->solverPool().at(1).kinVel();
			this->model()->solverPool().at(2).dynAccAndFce();

			for (int j = 0; j < model()->motionPool().size(); ++j){
				this->model()->motionPool()[j].setF(&fce[j][i]);
			}
			this->clb();

			for (int j = 0; j < model()->motionPool().size(); ++j){
				// 考虑速度死区 //
				if (std::abs(this->model()->motionPool()[j].mv()) < velocityDeadZone()[j])continue;

				// 考虑电机扭矩权重 //
				s_vc(n(), 1.0/torqueWeight()[j], this->A() + j * n(), A.data() + rows * n());
				b[rows] = this->b()[j] * 1.0/torqueWeight()[j];
				rows++;
			}
		}
		auto max_value = *std::max_element(A.begin(), A.begin() + rows * n());
		auto min_value = *std::min_element(A.begin(), A.begin() + rows * n());
		auto real_max = std::max(std::abs(max_value), std::abs(min_value));
		std::cout << "clb----A size:" << rows << "x" << cols << std::endl;
		std::cout << "clb----max value of A:" << real_max << std::endl;

		// solve calibration matrix //
		std::vector<double> U(rows * n());

		//std::cout << "solve calibration matrix" << std::endl;
		aris::Size rank;
		double zero_check = 1e-6;

		s_nv(rows * n(), 1.0 / real_max, A.data());
		s_nv(rows, 1.0 / real_max, b.data());
		s_householder_utp(rows, n(), A.data(), U.data(), tau.data(), p.data(), rank, zero_check);
		s_householder_utp_sov(rows, n(), 1, rank, U.data(), tau.data(), p.data(), b.data(), x.data(), zero_check);
		std::cout << "clb----rank:" << rank << std::endl;

		std::cout << "clb----inertia result:" << std::endl;
		dsp(model()->partPool().size() - 1, 10, x.data());
		std::cout << "clb----friction result:" << std::endl;
		dsp(model()->motionPool().size(), 3, x.data() + (model()->partPool().size() - 1) * 10);


		// check variance //
		s_mms(rows, 1, n(), A.data(), x.data(), b.data());
		auto variance = std::sqrt(s_vv(n(), b.data(), b.data()) / n());
		std::cout << "clb----variance:" << variance << std::endl;

		if (variance > imp_->tolerable_variance_) return -1;

		// update inertias //
		updateInertiaParam(x.data());
		return 0;
	}
	
	auto Calibrator::setVerifyOutputFileDir(std::string file_path)->void {
		imp_->verify_result_path = file_path;
	}
	auto Calibrator::verifyFiles(const std::vector<std::string> &file_paths)->void{
		// make datasets //
		std::cout << "making datasets" << std::endl;
		std::vector<std::vector<double> > pos(model()->motionPool().size());
		std::vector<std::vector<double> > vel(model()->motionPool().size());
		std::vector<std::vector<double> > acc(model()->motionPool().size());
		std::vector<std::vector<double> > fce(model()->motionPool().size());

		for (auto &file : file_paths) {
			std::cout << "----loading file:" << file << std::endl;
			auto mtx = aris::dynamic::dlmread(file.c_str());
			std::cout << "----making data" << std::endl;
			std::vector<std::vector<std::vector<double> > *> dataset{ &pos, &vel, &acc, &fce };
			makeDataset(this, mtx, dataset);
		}
		
		// now test datasets //
		auto num = pos[0].size();
		std::cout << "clb finished now compute dynamics of this dataset" << std::endl;
		std::vector<std::vector<double> > f(model()->motionPool().size(), std::vector<double>(num));
		std::vector<std::vector<double> > ff(model()->motionPool().size(), std::vector<double>(num));
		std::vector<std::vector<double> > fd(model()->motionPool().size(), std::vector<double>(num));
		for (int i = 0; i < num; ++i){
			for (int j = 0; j < model()->motionPool().size(); ++j){
				this->model()->motionPool()[j].setMp(pos[j][i]);
				this->model()->motionPool()[j].setMv(vel[j][i]);
				this->model()->motionPool()[j].setMa(acc[j][i]);
			}

			this->model()->solverPool().at(1).kinPos();
			this->model()->solverPool().at(1).kinVel();
			this->model()->solverPool().at(2).dynAccAndFce();

			for (int j = 0; j < model()->motionPool().size(); ++j)	{
				f[j][i] = this->model()->motionPool()[j].mf();
			}
		}

		



		std::cout << "dynamic finished, now output results" << std::endl;

		std::filesystem::create_directories(imp_->verify_result_path);

		for (int i = 0; i<model()->motionPool().size(); ++i){
			char posn[1024], veln[1024], accn[1024], fcen[1024], fn[1024], ffn[1024], fdn[1024];

			sprintf(posn, (imp_->verify_result_path + "/pos%d.txt").c_str(), i);
			sprintf(veln, (imp_->verify_result_path + "/vel%d.txt").c_str(), i);
			sprintf(accn, (imp_->verify_result_path + "/acc%d.txt").c_str(), i);
			sprintf(fcen, (imp_->verify_result_path + "/fce%d.txt").c_str(), i);
			sprintf(fn, (imp_->verify_result_path + "/f%d.txt").c_str(), i);
			sprintf(ffn, (imp_->verify_result_path + "/ff%d.txt").c_str(), i);
			sprintf(fdn, (imp_->verify_result_path + "/fd%d.txt").c_str(), i);

			dlmwrite(num, 1, pos[i].data(), posn);
			dlmwrite(num, 1, vel[i].data(), veln);
			dlmwrite(num, 1, acc[i].data(), accn);
			dlmwrite(num, 1, fce[i].data(), fcen);
			dlmwrite(num, 1, f[i].data(), fn);
			dlmwrite(num, 1, ff[i].data(), ffn);
			dlmwrite(num, 1, fd[i].data(), fdn);
		}



		std::cout << "end" << std::endl;
	}

	auto Calibrator::clbFile(const std::string &file_paths)->void{
		auto mtx = aris::dynamic::dlmread(file_paths.c_str());

		std::cout << "mtx size:" << mtx.size() << std::endl;

		auto num = mtx.size() / 24 / 10 - 1;
		double torque_constant[6]{ 0.283*4808,0.283*4808,0.276*2546,0.226*1556,0.219*849,0.219*849 };

		///////////////////////////////////////////////////////////////////////////////////
		// make data correct
		std::cout << "extract data from files" << std::endl;
		std::vector<std::vector<double> > pos(6, std::vector<double>(num));
		std::vector<std::vector<double> > vel(6, std::vector<double>(num));
		std::vector<std::vector<double> > acc(6, std::vector<double>(num));
		std::vector<std::vector<double> > fce(6, std::vector<double>(num));
		
		for (Size i = 0; i < num; ++i){
			for (Size j = 0; j < 6; ++j)	{
				// make actual pos //
				pos[j][i] = 0.0;
				for (Size k = 0; k < 10; ++k){
					pos[j][i] += mtx[i * 240 + k * 24 + j * 4 + 1] / 10.0;
				}

				// make actual vel //
				vel[j][i] = 0.0;
				for (Size k = 0; k < 10; ++k){
					vel[j][i] += mtx[i * 240 + k * 24 + j * 4 + 2] / 10.0;
				}

				// make actual acc //
				acc[j][i] = (mtx[i * 240 + j * 4 + 242] - mtx[i * 240 + j * 4 + 2])*100.0;

				// make actual fce //
				fce[j][i] = 0.0;
				for (Size k = 0; k < 10; ++k){
					fce[j][i] += mtx[i * 240 + k * 24 + j * 4 + 3] / 10.0 * torque_constant[j]/1e6;
				}
			}
		}
		///////////////////////////////////////////////////////////////////////////////////

		std::cout << "make calibration matrix" << std::endl;
		this->allocateMemory();

		std::vector<double> A(num * m() * n()), b(num * m(), 0.0), tau(num * m(), 0.0), x(num * m(), 0.0);
		std::vector<aris::Size> p(num * m());

		std::cout << "A size:" << num * m() << "x" << n() << std::endl;
		for (Size i = 0; i < num; ++i){
			for (Size j = 0; j < 6; ++j)	{
				this->model()->motionPool()[j].setMp(pos[j][i]);
				this->model()->motionPool()[j].setMv(vel[j][i]);
				this->model()->motionPool()[j].setMa(acc[j][i]);
				this->model()->motionPool()[j].setMf(fce[j][i]);
			}

			this->model()->solverPool().at(1).kinPos();
			this->model()->solverPool().at(1).kinVel();
			this->model()->solverPool().at(2).dynAccAndFce();

			for (int j = 0; j < 6; ++j){
				this->model()->motionPool()[j].setMf(fce[j][i]);
			}
			this->clb();
			
			std::copy_n(this->A(), m()*n(), A.data() + i * m() * n());
			std::copy_n(this->b(), m(), b.data() + i * m());
		}

		//dlmwrite(num * m(), n(), A.data(), "C:\\Users\\py033\\Desktop\\data_after\\A.txt");
		//dlmwrite(num * m(), 1, b.data(), "C:\\Users\\py033\\Desktop\\data_after\\b.txt");
		



		//double ratio = 1.0;
		//for (int i = 60; i < 78; ++i)
		//{
		//	s_nv(m()*num, ratio, A.data() + i, n());
		//}
		auto max_value = *std::max_element(A.begin(), A.end());
		auto min_value = *std::max_element(A.begin(), A.end());
		auto real_max = std::max(std::abs(max_value), std::abs(min_value));
		std::cout << "max value of A:" << real_max << std::endl;

		/////////////////////////////////////////////////////////////////////////////////////////
		std::cout << "solve calibration matrix" << std::endl;
		aris::Size rank;
		double zero_check = 1e-4;
		s_householder_utp(m()*num, n(), A.data(), A.data(), tau.data(), p.data(), rank, zero_check);
		//dlmwrite(num * m(), n(), A.data(), "C:\\Users\\py033\\Desktop\\data_after\\U.txt");
		//std::vector<double> Q(num * m() * n(), 0.0);
		//s_householder_ut2qmn(m()*num, rank, A.data(), n(), tau.data(),1, Q.data(),rank);
		//dlmwrite(num * m(), rank, Q.data(), "C:\\Users\\py033\\Desktop\\data_after\\Q.txt");
		//std::vector<double> R(num * m() * n(), 0.0);
		//s_householder_ut2r(m()*num, n(), A.data(), tau.data(), R.data());
		//dlmwrite(num * m(), n(), R.data(), "C:\\Users\\py033\\Desktop\\data_after\\R.txt");

		s_householder_utp_sov(m()*num, n(), 1, rank, A.data(), tau.data(), p.data(), b.data(), x.data(), zero_check);
		std::cout << "rank:" << rank << std::endl;
		
		//for (int i = 0; i < 78; ++i)std::cout << "  " << p[i];
		//std::cout << std::endl;

		//s_nv(18, ratio, x.data() + 60);
		
		std::cout << "inertia result:" << std::endl;
		dsp(6, 10, x.data());
		std::cout << "friction result:" << std::endl;
		dsp(6, 3, x.data() + 60);

		//dlmwrite(n(), 1, x.data(), "C:\\Users\\py033\\Desktop\\data_after\\x.txt");

		updateInertiaParam(x.data());


		std::cout << "clb finished now compute dynamics of this dataset" << std::endl;
		std::vector<std::vector<double> > f(6, std::vector<double>(num));
		std::vector<std::vector<double> > ff(6, std::vector<double>(num));
		std::vector<std::vector<double> > fd(6, std::vector<double>(num));
		for (int i = 0; i < num; ++i){
			for (int j = 0; j < 6; ++j){
				this->model()->motionPool()[j].setMp(pos[j][i]);
				this->model()->motionPool()[j].setMv(vel[j][i]);
				this->model()->motionPool()[j].setMa(acc[j][i]);
				this->model()->motionPool()[j].setMf(fce[j][i]);
			}

			this->model()->solverPool().at(1).kinPos();
			this->model()->solverPool().at(1).kinVel();
			this->model()->solverPool().at(2).dynAccAndFce();

			for (int j = 0; j < 6; ++j)
			{
				f[j][i] = this->model()->motionPool()[j].mf();
				ff[j][i] = this->model()->motionPool()[j].mfFrc();
				fd[j][i] = this->model()->motionPool()[j].mfDyn();
			}
		}





		std::cout << "dynamic finished, now output results" << std::endl;


		//dsp(1, 3, this->model()->motionPool()[0].frcCoe());



		
		for (int i = 0; i<6;++i){
			char posn[1024], veln[1024], accn[1024], fcen[1024], fn[1024], ffn[1024], fdn[1024];

			sprintf(posn, "C:\\Users\\py033\\Desktop\\data_after\\pos%d.txt", i);
			sprintf(veln, "C:\\Users\\py033\\Desktop\\data_after\\vel%d.txt", i);
			sprintf(accn, "C:\\Users\\py033\\Desktop\\data_after\\acc%d.txt", i);
			sprintf(fcen, "C:\\Users\\py033\\Desktop\\data_after\\fce%d.txt", i);
			sprintf(fn, "C:\\Users\\py033\\Desktop\\data_after\\f%d.txt", i);
			sprintf(ffn, "C:\\Users\\py033\\Desktop\\data_after\\ff%d.txt", i);
			sprintf(fdn, "C:\\Users\\py033\\Desktop\\data_after\\fd%d.txt", i);

			//dlmwrite(num, 1, pos[i].data(), posn);
			//dlmwrite(num, 1, vel[i].data(), veln);
			//dlmwrite(num, 1, acc[i].data(), accn);
			//dlmwrite(num, 1, fce[i].data(), fcen);
			dlmwrite(num, 1, f[i].data(), fn);
			dlmwrite(num, 1, ff[i].data(), ffn);
			dlmwrite(num, 1, fd[i].data(), fdn);
		}



		std::cout << "end" << std::endl;
	}
	auto Calibrator::updateInertiaParam(const double *x)->void{
		Size xi = 0;
		for (auto &prt : model()->partPool()){
			if (prt.active() && &prt != &model()->ground()){
				prt.setPrtIv(x + xi);
				xi += 10;
			}
		}
		// make x from frictions //
		Size bi = 0;
		for (auto &blk : imp_->cst_blk_vec_){
			if (dynamic_cast<Motion*>(blk.c)){
				dynamic_cast<Motion*>(blk.c)->setFrcCoe(x + g() + bi * 3);
				bi++;
			}
		}
	}
	Calibrator::~Calibrator() = default;
	Calibrator::Calibrator(const std::string &name) : imp_(new Imp) {}
	ARIS_DEFINE_BIG_FOUR_CPP_NOEXCEPT(Calibrator);

	struct SimResult::TimeResult::Imp { std::deque<double> time_; };
	/*
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
	*/
	auto SimResult::TimeResult::record()->void { 
		imp_->time_.push_back(model()->time()); 
	}
	auto SimResult::TimeResult::restore(Size pos)->void { model()->setTime(imp_->time_.at(pos)); }
	SimResult::TimeResult::~TimeResult() = default;
	SimResult::TimeResult::TimeResult(const std::string &name) :imp_(new Imp) {}
	SimResult::TimeResult::TimeResult(const SimResult::TimeResult&) = default;
	SimResult::TimeResult::TimeResult(SimResult::TimeResult&&)noexcept = default;
	SimResult::TimeResult& SimResult::TimeResult::operator=(const TimeResult&) = default;
	SimResult::TimeResult& SimResult::TimeResult::operator=(TimeResult&&)noexcept = default;

	struct SimResult::PartResult::Imp
	{
		Part *part_;
		std::deque<std::array<double, 6> > pe_;
		std::deque<std::array<double, 6> > vs_;
		std::deque<std::array<double, 6> > as_;

		Imp(Part* part) :part_(part) {};
	};
	/*
	auto SimResult::PartResult::saveXml(aris::core::XmlElement &xml_ele)const->void
	{
		Element::saveXml(xml_ele);

		xml_ele.SetAttribute("part", part().name().c_str());
		std::stringstream ss;
		ss << std::setprecision(15);
		ss.str().reserve((25 * 18 + 1)*imp_->pe_.size());

		for (auto pe = imp_->pe_.begin(), vs = imp_->vs_.begin(), as = imp_->as_.begin(); pe < imp_->pe_.end(); ++pe, ++vs, ++as)
		{
			for (auto e : *pe)ss << e << " ";
			for (auto e : *vs)ss << e << " ";
			for (auto e : *as)ss << e << " ";
			ss << std::endl;
		}

		xml_ele.SetText(ss.str().c_str());
	}
	auto SimResult::PartResult::loadXml(const aris::core::XmlElement &xml_ele)->void
	{
		
		// 以下寻找对应的part //
		if (model()->findByName("part_pool") == model()->children().end())
			THROW_FILE_LINE("you must insert \"part_pool\" node before insert " + type() + " \"" + name() + "\"");

		auto &part_pool = static_cast<aris::core::PointerArray<Part, Element>&>(*model()->findByName("part_pool"));

		if (!xml_ele.Attribute("part"))THROW_FILE_LINE(std::string("xml element \"") + name() + "\" must have Attribute \"part\"");
		auto p = part_pool.findByName(xml_ele.Attribute("part"));
		if (p == part_pool.end())	THROW_FILE_LINE(std::string("can't find part for PartResult \"") + this->name() + "\"");

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
	*/
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
	SimResult::PartResult::PartResult(const std::string &name, Part *part) : imp_(new Imp(part)) {}
	SimResult::PartResult::PartResult(const SimResult::PartResult&) = default;
	SimResult::PartResult::PartResult(SimResult::PartResult&&)noexcept = default;
	SimResult::PartResult& SimResult::PartResult::operator=(const PartResult&) = default;
	SimResult::PartResult& SimResult::PartResult::operator=(PartResult&&)noexcept = default;

	struct SimResult::ConstraintResult::Imp
	{
		Constraint *constraint_;
		std::deque<std::array<double, 6> > cf_;

		Imp(Constraint* constraint) :constraint_(constraint) {};
	};
	/*
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
		if (!xml_ele.Attribute("constraint"))THROW_FILE_LINE(std::string("xml element \"") + name() + "\" must have Attribute \"constraint\"");
		if (!imp_->constraint_ && model()->findByName("joint_pool") != model()->children().end())
		{
			auto &pool = static_cast<aris::core::PointerArray<Joint, Element>&>(*model()->findByName("joint_pool"));
			auto c = pool.findByName(xml_ele.Attribute("constraint"));
			if (c != pool.end())imp_->constraint_ = &*c;
		}
		if (!imp_->constraint_ && model()->findByName("motion_pool") != model()->children().end())
		{
			auto &pool = static_cast<aris::core::PointerArray<Motion, Element>&>(*model()->findByName("motion_pool"));
			auto c = pool.findByName(xml_ele.Attribute("constraint"));
			if (c != pool.end())imp_->constraint_ = &*c;
		}
		if (!imp_->constraint_ && model()->findByName("general_motion_pool") != model()->children().end())
		{
			auto &pool = static_cast<aris::core::PointerArray<GeneralMotion, Element>&>(*model()->findByName("general_motion_pool"));
			auto c = pool.findByName(xml_ele.Attribute("constraint"));
			if (c != pool.end())imp_->constraint_ = &*c;
		}
		if (!imp_->constraint_)THROW_FILE_LINE(std::string("can't find constraint for ConstraintResult \"") + this->name() + "\"");

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
	*/
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
			dynamic_cast<Motion*>(&constraint())->updP();
			dynamic_cast<Motion*>(&constraint())->updV();
			dynamic_cast<Motion*>(&constraint())->updA();
		}
		if (dynamic_cast<GeneralMotion*>(&constraint()))
		{
			dynamic_cast<GeneralMotion*>(&constraint())->updP();
			dynamic_cast<GeneralMotion*>(&constraint())->updV();
			dynamic_cast<GeneralMotion*>(&constraint())->updA();
		}
	}
	SimResult::ConstraintResult::~ConstraintResult() = default;
	SimResult::ConstraintResult::ConstraintResult(const std::string &name, Constraint *constraint) : imp_(new Imp(constraint)) {}
	SimResult::ConstraintResult::ConstraintResult(const SimResult::ConstraintResult&) = default;
	SimResult::ConstraintResult::ConstraintResult(SimResult::ConstraintResult&&)noexcept = default;
	SimResult::ConstraintResult& SimResult::ConstraintResult::operator=(const ConstraintResult&) = default;
	SimResult::ConstraintResult& SimResult::ConstraintResult::operator=(ConstraintResult&&)noexcept = default;

	struct SimResult::Imp
	{
		TimeResult time_result_;
		std::vector<PartResult> part_result_pool_;
		std::vector<ConstraintResult> constraint_result_pool_;
	};
	/*
	auto SimResult::loadXml(const aris::core::XmlElement &xml_ele)->void
	{
		
		Element::loadXml(xml_ele);

		imp_->time_result_ = findOrInsertType<TimeResult>();
		imp_->constraint_result_pool_ = findOrInsertType<aris::core::PointerArray<SimResult::ConstraintResult, Element> >();
		imp_->part_result_pool_ = findOrInsertType<aris::core::PointerArray<SimResult::PartResult, Element> >();
		
	}
	*/
	auto SimResult::timeResult()->TimeResult& { return imp_->time_result_; }
	auto SimResult::partResultPool()->std::vector<SimResult::PartResult>& { return imp_->part_result_pool_; }
	auto SimResult::constraintResultPool()->std::vector<SimResult::ConstraintResult>& { return imp_->constraint_result_pool_; }
	auto SimResult::allocateMemory()->void
	{
		partResultPool().clear();
		for (auto &p : model()->partPool())partResultPool().emplace_back(p.name() + "_result", &p);
		constraintResultPool().clear();
		for (auto &c : model()->jointPool())constraintResultPool().emplace_back(c.name() + "_result", &c);
		for (auto &c : model()->motionPool())constraintResultPool().emplace_back(c.name() + "_result", &c);
		for (auto &c : model()->generalMotionPool())constraintResultPool().emplace_back(c.name() + "_result", &c);

		timeResult().resetModel(model());
		for (auto &c : partResultPool())c.resetModel(model());
		for (auto &c : constraintResultPool())c.resetModel(model());
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
	SimResult::SimResult(const std::string &name) : imp_(new Imp())
	{
		/*
		imp_->time_result_ = &add<TimeResult>("time_result");
		imp_->part_result_pool_ = &add<aris::core::PointerArray<SimResult::PartResult, Element> >("part_result_pool");
		imp_->constraint_result_pool_ = &add<aris::core::PointerArray<SimResult::ConstraintResult, Element> >("constraint_result_pool");
		*/
	}
	SimResult::SimResult(const SimResult&other) : Element(other), imp_(other.imp_)
	{
		/*
		imp_->time_result_ = findType<TimeResult >("time_result");
		imp_->constraint_result_pool_ = findType<aris::core::PointerArray<SimResult::ConstraintResult, Element> >("constraint_result_pool");
		imp_->part_result_pool_ = findType<aris::core::PointerArray<SimResult::PartResult, Element> >("part_result_pool");
		*/
	}
	SimResult::SimResult(SimResult&&other)noexcept : Element(std::move(other)), imp_(std::move(other.imp_)){
		/*
		imp_->time_result_ = findType<TimeResult >("time_result");
		imp_->constraint_result_pool_ = findType<aris::core::PointerArray<SimResult::ConstraintResult, Element> >("constraint_result_pool");
		imp_->part_result_pool_ = findType<aris::core::PointerArray<SimResult::PartResult, Element> >("part_result_pool");
		*/
	}
	SimResult& SimResult::operator=(const SimResult&other)
	{
		/*
		Element::operator=(other);
		imp_ = other.imp_;
		imp_->time_result_ = findType<TimeResult >("time_result");
		imp_->constraint_result_pool_ = findType<aris::core::PointerArray<SimResult::ConstraintResult, Element> >("constraint_result_pool");
		imp_->part_result_pool_ = findType<aris::core::PointerArray<SimResult::PartResult, Element> >("part_result_pool");
		*/
		return *this;
	}
	SimResult& SimResult::operator=(SimResult&&other)noexcept{
		/*
		Element::operator=(std::move(other));
		imp_ = other.imp_;
		imp_->time_result_ = findType<TimeResult >("time_result");
		imp_->constraint_result_pool_ = findType<aris::core::PointerArray<SimResult::ConstraintResult, Element> >("constraint_result_pool");
		imp_->part_result_pool_ = findType<aris::core::PointerArray<SimResult::PartResult, Element> >("part_result_pool");
		*/
		return *this;
	}

	struct Simulator::Imp {};
	auto Simulator::simulate(aris::plan::Plan &plan, SimResult &result)->void{
		result.allocateMemory();
		// 记录初始状态 //
		result.record();
		
		auto p = std::unique_ptr<aris::plan::Plan>(plan.clone());

		p->setCount(1);
		p->setModelBase(model());
		p->setMaster(nullptr);
		p->setController(nullptr);
		p->setControlServer(nullptr);

		// 记录轨迹中的状态 //
		for (; p->executeRT() != 0; p->setCount(p->count() + 1)) result.record();
		
		// 记录结束状态 //
		result.record();
		result.restore(0);
	}
	Simulator::~Simulator() = default;
	Simulator::Simulator(const std::string &name) :imp_(new Imp) {}
	ARIS_DEFINE_BIG_FOUR_CPP_NOEXCEPT(Simulator);

	struct SolverSimulator::Imp
	{
		Solver *solver_;
		Imp(Solver *solver) :solver_(solver) { };
	};
	/*
	auto SolverSimulator::saveXml(aris::core::XmlElement &xml_ele) const->void
	{

		Simulator::saveXml(xml_ele);
		xml_ele.SetAttribute("solver", solver().name().c_str());

	}
	auto SolverSimulator::loadXml(const aris::core::XmlElement &xml_ele)->void
	{

		Simulator::loadXml(xml_ele);

		if (model()->findByName("solver_pool") == model()->children().end())
			THROW_FILE_LINE("you must insert \"solver_pool\" node before insert " + type() + " \"" + name() + "\"");

		auto &solver_pool = static_cast<aris::core::PointerArray<Solver, Element>&>(*model()->findByName("solver_pool"));

		if (!xml_ele.Attribute("solver"))THROW_FILE_LINE(std::string("xml element \"") + name() + "\" must have Attribute \"solver\"");
		auto s = solver_pool.findByName(xml_ele.Attribute("solver"));
		if (s == solver_pool.end())	THROW_FILE_LINE(std::string("can't find solver for element \"") + this->name() + "\"");

		imp_->solver_ = &*s;
		
	}
	*/
	auto SolverSimulator::solver()->Solver& { return *imp_->solver_; }
	auto SolverSimulator::simulate(aris::plan::Plan &plan, SimResult &result)->void{ Simulator::simulate(plan, result); }
	SolverSimulator::~SolverSimulator() = default;
	SolverSimulator::SolverSimulator(const std::string &name, Solver *solver) : Simulator(name), imp_(new Imp(solver)) {}
	ARIS_DEFINE_BIG_FOUR_CPP_NOEXCEPT(SolverSimulator);

	struct AdamsSimulator::Imp {};
	auto AdamsSimulator::saveAdams(const std::string &filename, SimResult &result, Size pos)->void
	{
		std::string filename_ = filename;
		if (filename_.size() < 4 || filename_.substr(filename.size() - 4, 4) != ".cmd"){
			filename_ += ".cmd";
		}

		std::ofstream file;
		file.open(filename_, std::ios::out | std::ios::trunc);

		saveAdams(file, result, pos);

		file.close();
	}
	auto AdamsSimulator::saveAdams(std::ofstream &file, SimResult &result, Size pos)->void{
		// 名字 //
		auto model_name = std::string("model");
		auto geoid = 0;
		
		// 生成akima曲线 //
		std::vector<double> time(result.size() + 1);
		std::vector<std::vector<double>> mot_akima(model()->motionPool().size(), std::vector<double>(result.size() + 1));
		std::vector<std::vector<std::array<double, 6>>> gm_akima(model()->generalMotionPool().size(), std::vector<std::array<double, 6>>(result.size() + 1));
		if (pos == -1){
			if (result.size() < 4)THROW_FILE_LINE("failed to AdamsSimulator::saveAdams: because result size is smaller than 4\n");

			for (Size i(-1); ++i < result.size() + 1;){
				result.restore(i);
				time.at(i) = model()->time();
				for (Size j(-1); ++j < model()->motionPool().size();){
					model()->motionPool().at(j).updP();
					mot_akima.at(j).at(i) = model()->motionPool().at(j).mpInternal();
				}
				for (Size j(-1); ++j < model()->generalMotionPool().size();){
					if (auto gm = dynamic_cast<GeneralMotion*>(&model()->generalMotionPool().at(j))){
						gm->updP();
						gm->getMpe(gm_akima.at(j).at(i).data(), "123");
					}
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
			<< "   model_name = " << model_name << "\r\n"
			<< "!\r\n"
			<< "view erase\r\n"
			<< "!\r\n"
			<< "!---------------------------------- Accgrav -----------------------------------!\r\n"
			<< "!\r\n"
			<< "!\r\n"
			<< "force create body gravitational  &\r\n"
			<< "    gravity_field_name = gravity  &\r\n"
			<< "    x_component_gravity = " << model()->environment().gravity()[0] << "  &\r\n"
			<< "    y_component_gravity = " << model()->environment().gravity()[1] << "  &\r\n"
			<< "    z_component_gravity = " << model()->environment().gravity()[2] << "\r\n"
			<< "!\r\n";
		for (auto &part : model()->partPool()){
			if (&part == &model()->ground()){
				file << "!----------------------------------- ground -----------------------------------!\r\n"
					<< "!\r\n"
					<< "!\r\n"
					<< "! ****** Ground Part ******\r\n"
					<< "!\r\n"
					<< "defaults model  &\r\n"
					<< "    part_name = ground\r\n"
					<< "!\r\n"
					<< "defaults coordinate_system  &\r\n"
					<< "    default_coordinate_system = ." << model_name << ".ground\r\n"
					<< "!\r\n"
					<< "! ****** Markers for current part ******\r\n"
					<< "!\r\n";
			}
			else{
				double pe[6];
				s_pm2pe(*part.pm(), pe, "313");
				core::Matrix ori(1, 3, &pe[3]), loc(1, 3, &pe[0]);

				file << "!----------------------------------- " << part.name() << " -----------------------------------!\r\n"
					<< "!\r\n"
					<< "!\r\n"
					<< "defaults coordinate_system  &\r\n"
					<< "    default_coordinate_system = ." << model_name << ".ground\r\n"
					<< "!\r\n"
					<< "part create rigid_body name_and_position  &\r\n"
					<< "    part_name = ." << model_name << "." << part.name() << "  &\r\n"
					<< "    adams_id = " << adamsID(part) << "  &\r\n"
					<< "    location = (" << loc.toString() << ")  &\r\n"
					<< "    orientation = (" << ori.toString() << ")\r\n"
					<< "!\r\n"
					<< "defaults coordinate_system  &\r\n"
					<< "    default_coordinate_system = ." << model_name << "." << part.name() << " \r\n"
					<< "!\r\n";


				double mass = part.prtIv()[0] == 0 ? 1 : part.prtIv()[0];
				std::fill_n(pe, 6, 0);
				pe[0] = part.prtIv()[1] / mass;
				pe[1] = part.prtIv()[2] / mass;
				pe[2] = part.prtIv()[3] / mass;

				file << "! ****** cm and mass for current part ******\r\n"
					<< "marker create  &\r\n"
					<< "    marker_name = ." << model_name << "." << part.name() << ".cm  &\r\n"
					<< "    adams_id = " << adamsID(part) + std::accumulate(model()->partPool().begin(), model()->partPool().end(), Size(0), [](Size a, Part &b) {return a + b.markerPool().size(); }) << "  &\r\n"
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
					<< "    part_name = ." << model_name << "." << part.name() << "  &\r\n"
					<< "    mass = " << part.prtIv()[0] << "  &\r\n"
					<< "    center_of_mass_marker = ." << model_name << "." << part.name() << ".cm  &\r\n"
					<< "    inertia_marker = ." << model_name << "." << part.name() << ".cm  &\r\n"
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
					<< "    part_name = ." << model_name << "." << part.name() << "  &\r\n"
					<< "    vx = " << cm_vs[0] << "  &\r\n"
					<< "    vy = " << cm_vs[1] << "  &\r\n"
					<< "    vz = " << cm_vs[2] << "  &\r\n"
					<< "    wx = " << cm_vs[3] << "  &\r\n"
					<< "    wy = " << cm_vs[4] << "  &\r\n"
					<< "    wz = " << cm_vs[5] << "  \r\n"
					<< "!\r\n";

				file << "part modify rigid_body initial_velocity  &\r\n"
					<< "    part_name = ." << model_name << "." << part.name() << "  &\r\n"
					<< "    vm = ." << model_name << "." << part.name() << ".cm  &\r\n"
					<< "    wm = ." << model_name << "." << part.name() << ".cm \r\n"
					<< "!\r\n";


			}

			//导入marker
			for (auto &marker : part.markerPool()){
				double pe[6];

				s_pm2pe(*marker.prtPm(), pe, "313");
				core::Matrix ori(1, 3, &pe[3]), loc(1, 3, &pe[0]);

				file << "marker create  &\r\n"
					<< "marker_name = ." << model_name << "." << part.name() << "." << marker.name() << "  &\r\n"
					<< "adams_id = " << adamsID(marker) << "  &\r\n"
					<< "location = (" << loc.toString() << ")  &\r\n"
					<< "orientation = (" << ori.toString() << ")\r\n"
					<< "!\r\n";
			}
		}
		for (auto &joint : model()->jointPool()){
#define ARIS_ADAMS_EXPORT_JOINT(TYPE)                                                                                                             \
			file << "constraint create joint " << TYPE << "  &\r\n"                                                                               \
				<< "    joint_name = ." << model_name << "." << joint.name() << "  &\r\n"                                                         \
				<< "    adams_id = " << adamsID(joint) << "  &\r\n"                                                                               \
				<< "    i_marker_name = ." << model_name << "." << joint.makI()->fatherPart().name() << "." << joint.makI()->name() << "  &\r\n"  \
				<< "    j_marker_name = ." << model_name << "." << joint.makJ()->fatherPart().name() << "." << joint.makJ()->name() << "  \r\n"   \
				<< "!\r\n"
			std::string type;
			if (dynamic_cast<RevoluteJoint*>(&joint))ARIS_ADAMS_EXPORT_JOINT("revolute");
			else if (dynamic_cast<PrismaticJoint*>(&joint))ARIS_ADAMS_EXPORT_JOINT("translational");
			else if (dynamic_cast<UniversalJoint*>(&joint))ARIS_ADAMS_EXPORT_JOINT("universal");
			else if (dynamic_cast<SphericalJoint*>(&joint))ARIS_ADAMS_EXPORT_JOINT("spherical");
			else if (dynamic_cast<ScrewJoint*>(&joint)) {
				file << "constraint create joint " << "screw" << "  &\r\n"                                                                            \
					<< "    joint_name = ." << model_name << "." << joint.name() << "  &\r\n"                                                         \
					<< "    adams_id = " << adamsID(joint) << "  &\r\n"                                                                               \
					<< "    i_marker_name = ." << model_name << "." << joint.makI()->fatherPart().name() << "." << joint.makI()->name() << "  &\r\n"  \
					<< "    j_marker_name = ." << model_name << "." << joint.makJ()->fatherPart().name() << "." << joint.makJ()->name() << "  &\r\n"   \
					<< "    pitch = " << dynamic_cast<ScrewJoint*>(&joint)->pitch() << "\r\n"
					<< "!\r\n";

				file << "constraint create joint " << "cylindrical" << "  &\r\n"                                                                      \
					<< "    joint_name = ." << model_name << ".__" << joint.name() << "  &\r\n"                                                         \
					<< "    adams_id = " << adamsID(joint) + model()->jointPool().size() << "  &\r\n"                                                 \
					<< "    i_marker_name = ." << model_name << "." << joint.makI()->fatherPart().name() << "." << joint.makI()->name() << "  &\r\n"  \
					<< "    j_marker_name = ." << model_name << "." << joint.makJ()->fatherPart().name() << "." << joint.makJ()->name() << "  \r\n"   \
					<< "!\r\n";
			}
			else THROW_FILE_LINE("unrecognized joint type");
#undef ARIS_ADAMS_EXPORT_JOINT
		}
		for (auto &motion : model()->motionPool()){
			std::string axis_names[6]{ "x","y","z","B1","B2","B3" };
			std::string axis_name = axis_names[motion.axis()];

			std::string akima = motion.name() + "_akima";
			std::string akima_func = "AKISPL(time,0," + akima + ")";
			std::string polynomial_func = static_cast<const std::stringstream &>(std::stringstream() << std::setprecision(16) << motion.mpInternal() << " + " << motion.mv() << " * time + " << motion.ma()*0.5 << " * time * time").str();

			// 构建akima曲线 //
			if (pos == -1){
				file << "data_element create spline &\r\n"
					<< "    spline_name = ." << model_name + "." + motion.name() + "_akima &\r\n"
					<< "    adams_id = " << adamsID(motion) << "  &\r\n"
					<< "    units = m &\r\n"
					<< "    x = " << time.at(0);
				for (auto p = time.begin() + 1; p < time.end(); ++p){
					file << "," << *p;
				}
				file << "    y = " << mot_akima.at(motion.id()).at(0);
				for (auto p = mot_akima.at(motion.id()).begin() + 1; p < mot_akima.at(motion.id()).end(); ++p){
					file << "," << *p;
				}
				file << " \r\n!\r\n";
			}

			file << "constraint create motion_generator &\r\n"
				<< "    motion_name = ." << model_name << "." << motion.name() << "  &\r\n"
				<< "    adams_id = " << adamsID(motion) << "  &\r\n"
				<< "    i_marker_name = ." << model_name << "." << motion.makI()->fatherPart().name() << "." << motion.makI()->name() << "  &\r\n"
				<< "    j_marker_name = ." << model_name << "." << motion.makJ()->fatherPart().name() << "." << motion.makJ()->name() << "  &\r\n"
				<< "    axis = " << axis_name << "  &\r\n"
				<< "    function = \"" << (pos == -1 ? akima_func : polynomial_func) << "\"  \r\n"
				<< "!\r\n";
		}
		for (auto &gmb : model()->generalMotionPool()){
			if (auto gm = dynamic_cast<GeneralMotion*>(&gmb)) {
				file << "ude create instance  &\r\n"
					<< "    instance_name = ." << model_name << "." << gm->name() << "  &\r\n"
					<< "    definition_name = .MDI.Constraints.general_motion  &\r\n"
					<< "    location = 0.0, 0.0, 0.0  &\r\n"
					<< "    orientation = 0.0, 0.0, 0.0  \r\n"
					<< "!\r\n";

				file << "variable modify  &\r\n"
					<< "	variable_name = ." << model_name << "." << gm->name() << ".i_marker  &\r\n"
					<< "	object_value = ." << model_name << "." << gm->makI()->fatherPart().name() << "." << gm->makI()->name() << " \r\n"
					<< "!\r\n";

				file << "variable modify  &\r\n"
					<< "	variable_name = ." << model_name << "." << gm->name() << ".j_marker  &\r\n"
					<< "	object_value = ." << model_name << "." << gm->makJ()->fatherPart().name() << "." << gm->makJ()->name() << " \r\n"
					<< "!\r\n";

				std::string axis_names[6]{ "t1", "t2", "t3", "r1", "r2", "r3" };

				double pe123[6], ve123[6], ae123[6];
				gm->getMpe(pe123, "123");
				gm->getMve(ve123, "123");
				gm->getMae(ae123, "123");
				for (Size i = 0; i < 6; ++i)
				{
					std::string akima = gm->name() + "_" + axis_names[i] + "_akima";
					std::string akima_func = "AKISPL(time,0," + akima + ")";
					std::string polynomial_func = static_cast<const std::stringstream &>(std::stringstream() << std::setprecision(16) << pe123[i] << " + " << ve123[i] << " * time + " << ae123[i] * 0.5 << " * time * time").str();
					std::string func = pos == -1 ? akima_func : polynomial_func;

					// 构建akima曲线 //
					if (pos == -1)
					{
						file << "data_element create spline &\r\n"
							<< "    spline_name = ." << model_name + "." + akima + " &\r\n"
							<< "    adams_id = " << model()->motionPool().size() + adamsID(*gm) * 6 + i << "  &\r\n"
							<< "    units = m &\r\n"
							<< "    x = " << time.at(0);
						for (auto p = time.begin() + 1; p < time.end(); ++p)
						{
							file << "," << *p;
						}
						file << "    y = " << gm_akima.at(gm->id()).at(0).at(i);
						for (auto p = gm_akima.at(gm->id()).begin() + 1; p < gm_akima.at(gm->id()).end(); ++p)
						{
							file << "," << p->at(i);
						}
						file << " \r\n!\r\n";
					}
					file << "variable modify  &\r\n"
						<< "	variable_name = ." << model_name << "." << gm->name() << "." << axis_names[i] << "_type  &\r\n"
						<< "	integer_value = 1 \r\n"
						<< "!\r\n";

					file << "variable modify  &\r\n"
						<< "	variable_name = ." << model_name << "." << gm->name() << "." << axis_names[i] << "_func  &\r\n"
						<< "	string_value = \"" + func + "\" \r\n"
						<< "!\r\n";

					file << "variable modify  &\r\n"
						<< "	variable_name = ." << model_name << "." << gm->name() << "." << axis_names[i] << "_ic_disp  &\r\n"
						<< "	real_value = 0.0 \r\n"
						<< "!\r\n";

					file << "variable modify  &\r\n"
						<< "	variable_name = ." << model_name << "." << gm->name() << "." << axis_names[i] << "_ic_velo  &\r\n"
						<< "	real_value = 0.0 \r\n"
						<< "!\r\n";
				}

				file << "ude modify instance  &\r\n"
					<< "	instance_name = ." << model_name << "." << gm->name() << "\r\n"
					<< "!\r\n";
			
			}
			else if (auto gmp = dynamic_cast<PointMotion*>(&gmb)) {
				file << "ude create instance  &\r\n"
					<< "    instance_name = ." << model_name << "." << gmp->name() << "  &\r\n"
					<< "    definition_name = .MDI.Constraints.general_motion  &\r\n"
					<< "    location = 0.0, 0.0, 0.0  &\r\n"
					<< "    orientation = 0.0, 0.0, 0.0  \r\n"
					<< "!\r\n";

				file << "variable modify  &\r\n"
					<< "	variable_name = ." << model_name << "." << gmp->name() << ".i_marker  &\r\n"
					<< "	object_value = ." << model_name << "." << gmp->makI()->fatherPart().name() << "." << gmp->makI()->name() << " \r\n"
					<< "!\r\n";

				file << "variable modify  &\r\n"
					<< "	variable_name = ." << model_name << "." << gmp->name() << ".j_marker  &\r\n"
					<< "	object_value = ." << model_name << "." << gmp->makJ()->fatherPart().name() << "." << gmp->makJ()->name() << " \r\n"
					<< "!\r\n";

				std::string axis_names[6]{ "t1", "t2", "t3", "r1", "r2", "r3" };

				double mp[6]{ gmp->p()[0],gmp->p()[1],gmp->p()[2],0,0,0 };
				double mv[6]{ gmp->v()[0],gmp->v()[1],gmp->v()[2],0,0,0 };
				double ma[6]{ gmp->a()[0],gmp->a()[1],gmp->a()[2],0,0,0 };

				for (Size i = 0; i < 6; ++i)
				{
					std::string akima = gmp->name() + "_" + axis_names[i] + "_akima";
					std::string akima_func = "AKISPL(time,0," + akima + ")";
					std::string polynomial_func = static_cast<const std::stringstream &>(std::stringstream() << std::setprecision(16) << mp[i] << " + " << mv[i] << " * time + " << ma[i] * 0.5 << " * time * time").str();
					std::string func = pos == -1 ? akima_func : polynomial_func;

					// 构建akima曲线 //
					if (pos == -1){
						file << "data_element create spline &\r\n"
							<< "    spline_name = ." << model_name + "." + akima + " &\r\n"
							<< "    adams_id = " << model()->motionPool().size() + adamsID(*gmp) * 6 + i << "  &\r\n"
							<< "    units = m &\r\n"
							<< "    x = " << time.at(0);
						for (auto p = time.begin() + 1; p < time.end(); ++p){
							file << "," << *p;
						}
						file << "    y = " << gm_akima.at(gmp->id()).at(0).at(i);
						for (auto p = gm_akima.at(gmp->id()).begin() + 1; p < gm_akima.at(gmp->id()).end(); ++p){
							file << "," << p->at(i);
						}
						file << " \r\n!\r\n";
					}

					file << "variable modify  &\r\n"
						<< "	variable_name = ." << model_name << "." << gmp->name() << "." << axis_names[i] << "_type  &\r\n"
						<< "	integer_value = " << (i < 3 ? 1 : 0) <<  " \r\n"
						<< "!\r\n";

					file << "variable modify  &\r\n"
						<< "	variable_name = ." << model_name << "." << gmp->name() << "." << axis_names[i] << "_func  &\r\n"
						<< "	string_value = \"" + func + "\" \r\n"
						<< "!\r\n";

					file << "variable modify  &\r\n"
						<< "	variable_name = ." << model_name << "." << gmp->name() << "." << axis_names[i] << "_ic_disp  &\r\n"
						<< "	real_value = 0.0 \r\n"
						<< "!\r\n";

					file << "variable modify  &\r\n"
						<< "	variable_name = ." << model_name << "." << gmp->name() << "." << axis_names[i] << "_ic_velo  &\r\n"
						<< "	real_value = 0.0 \r\n"
						<< "!\r\n";
				}

				file << "ude modify instance  &\r\n"
					<< "	instance_name = ." << model_name << "." << gmp->name() << "\r\n"
					<< "!\r\n";
			}
			else if (auto gmp = dynamic_cast<XyztMotion*>(&gmb)) {
				file << "ude create instance  &\r\n"
					<< "    instance_name = ." << model_name << "." << gmp->name() << "  &\r\n"
					<< "    definition_name = .MDI.Constraints.general_motion  &\r\n"
					<< "    location = 0.0, 0.0, 0.0  &\r\n"
					<< "    orientation = 0.0, 0.0, 0.0  \r\n"
					<< "!\r\n";

				file << "variable modify  &\r\n"
					<< "	variable_name = ." << model_name << "." << gmp->name() << ".i_marker  &\r\n"
					<< "	object_value = ." << model_name << "." << gmp->makI()->fatherPart().name() << "." << gmp->makI()->name() << " \r\n"
					<< "!\r\n";

				file << "variable modify  &\r\n"
					<< "	variable_name = ." << model_name << "." << gmp->name() << ".j_marker  &\r\n"
					<< "	object_value = ." << model_name << "." << gmp->makJ()->fatherPart().name() << "." << gmp->makJ()->name() << " \r\n"
					<< "!\r\n";

				std::string axis_names[6]{ "t1", "t2", "t3", "r1", "r2", "r3" };

				double mp[6]{ gmp->p()[0],gmp->p()[1],gmp->p()[2],0,0,gmp->p()[3] };
				double mv[6]{ gmp->v()[0],gmp->v()[1],gmp->v()[2],0,0,gmp->p()[3] };
				double ma[6]{ gmp->a()[0],gmp->a()[1],gmp->a()[2],0,0,gmp->p()[3] };

				for (Size i = 0; i < 6; ++i){
					std::string akima = gmp->name() + "_" + axis_names[i] + "_akima";
					std::string akima_func = "AKISPL(time,0," + akima + ")";
					std::string polynomial_func = static_cast<const std::stringstream &>(std::stringstream() << std::setprecision(16) << mp[i] << " + " << mv[i] << " * time + " << ma[i] * 0.5 << " * time * time").str();
					std::string func = pos == -1 ? akima_func : polynomial_func;

					// 构建akima曲线 //
					if (pos == -1) {
						file << "data_element create spline &\r\n"
							<< "    spline_name = ." << model_name + "." + akima + " &\r\n"
							<< "    adams_id = " << model()->motionPool().size() + adamsID(*gmp) * 6 + i << "  &\r\n"
							<< "    units = m &\r\n"
							<< "    x = " << time.at(0);
						for (auto p = time.begin() + 1; p < time.end(); ++p) {
							file << "," << *p;
						}
						file << "    y = " << gm_akima.at(gmp->id()).at(0).at(i);
						for (auto p = gm_akima.at(gmp->id()).begin() + 1; p < gm_akima.at(gmp->id()).end(); ++p) {
							file << "," << p->at(i);
						}
						file << " \r\n!\r\n";
					}

					file << "variable modify  &\r\n"
						<< "	variable_name = ." << model_name << "." << gmp->name() << "." << axis_names[i] << "_type  &\r\n"
						<< "	integer_value = " << (i < 3 || i==5 ? 1 : 0) << " \r\n"
						<< "!\r\n";

					file << "variable modify  &\r\n"
						<< "	variable_name = ." << model_name << "." << gmp->name() << "." << axis_names[i] << "_func  &\r\n"
						<< "	string_value = \"" + func + "\" \r\n"
						<< "!\r\n";

					file << "variable modify  &\r\n"
						<< "	variable_name = ." << model_name << "." << gmp->name() << "." << axis_names[i] << "_ic_disp  &\r\n"
						<< "	real_value = 0.0 \r\n"
						<< "!\r\n";

					file << "variable modify  &\r\n"
						<< "	variable_name = ." << model_name << "." << gmp->name() << "." << axis_names[i] << "_ic_velo  &\r\n"
						<< "	real_value = 0.0 \r\n"
						<< "!\r\n";
				}

				file << "ude modify instance  &\r\n"
					<< "	instance_name = ." << model_name << "." << gmp->name() << "\r\n"
					<< "!\r\n";
			}
		}
		for (auto &force : model()->forcePool())
		{
			double fsI[6], fsJ[6], fsI_loc[6];
			force.cptGlbFs(fsI, fsJ);
			s_inv_fs2fs(*force.makI()->pm(), fsI, fsI_loc);

			file << "floating_marker create  &\r\n"
				<< "    floating_marker_name = ." << model_name << "." << force.makJ()->fatherPart().name() << "." << force.name() << "_FMAK  &\r\n"
				<< "    adams_id = " << adamsID(force) + model()->partPool().size() + std::accumulate(model()->partPool().begin(), model()->partPool().end(), Size(0), [](Size a, Part &b) {return a + b.markerPool().size(); }) << "\r\n"
				<< "!\r\n";

			file << "force create direct general_force  &\r\n"
				<< "    general_force_name = ." << model_name << "." << force.name() << "  &\r\n"
				<< "    adams_id = " << adamsID(force) << "  &\r\n"
				<< "    i_marker_name = ." << model_name << "." << force.makI()->fatherPart().name() << "." << force.makI()->name() << "  &\r\n"
				<< "    j_floating_marker_name = ." << model_name << "." << force.makJ()->fatherPart().name() << "." << force.name() << "_FMAK  &\r\n"
				<< "    ref_marker_name = ." << model_name << "." << force.makI()->fatherPart().name() << "." << force.makI()->name() << "  &\r\n"
				<< "    x_force_function = \"" << fsI_loc[0] << "\"  &\r\n"
				<< "    y_force_function = \"" << fsI_loc[1] << "\"  &\r\n"
				<< "    z_force_function = \"" << fsI_loc[2] << "\"  &\r\n"
				<< "    x_torque_function = \"" << fsI_loc[3] << "\"  &\r\n"
				<< "    y_torque_function = \"" << fsI_loc[4] << "\"  &\r\n"
				<< "    z_torque_function = \"" << fsI_loc[5] << "\"\r\n"
				<< "!\r\n";

		}

		// geometry, 防止geometry 添加marker，导致marker id冲突
		for (auto &part : model()->partPool())
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
						<< "	relative_to = ." << model_name << "." << part.name() << " \r\n"
						<< "!\r\n";
				}
				else if (FileGeometry* geo = dynamic_cast<FileGeometry*>(&geometry))
				{
					double pe[6];
					s_pm2pe(*geo->prtPm(), pe, "313");
					core::Matrix ori(1, 3, &pe[3]), loc(1, 3, &pe[0]);

					auto file_type = geo->filePath().substr(geo->filePath().rfind('.') + 1);

					file << "file geometry read &\r\n"
						<< "	type_of_geometry = \"" << file_type << "\" &\r\n"
						<< "	file_name = \"" << geo->filePath() << "\" &\r\n"
						<< "	part_name = " << part.name() << " &\r\n"
						<< "	location = (" << loc.toString() << ") &\r\n"
						<< "	orientation = (" << ori.toString() << ") &\r\n"
						<< "	relative_to = ." << model_name << "." << part.name() << " &\r\n"
						<< "	scale = " << "0.001" << "\r\n"
						<< "!\r\n";
				}
				else if (ShellGeometry* geo = dynamic_cast<ShellGeometry*>(&geometry))
				{
					file << "geometry create shape shell  &\r\n"
						<< "	shell_name = ." << model_name << "." << part.name() << ".geometry" << ++geoid << " &\r\n"
						<< "	reference_marker = ." << model_name << "." << part.name() << "." << geo->relativeToMarker().name() << " &\r\n"
						<< "	file_name = \"" << geo->filePath() << "\" &\r\n"
						<< "	wireframe_only = " << "no" << "\r\n"
						<< "!\r\n";
				}
				else
				{
					//THROW_FILE_LINE("unrecognized geometry type:" + geometry.type());
					THROW_FILE_LINE("unrecognized geometry type");
				}

			}
		}

		file << "!----------------------------------- Motify Active -------------------------------------!\r\n!\r\n!\r\n";
		for (auto &prt : model()->partPool())
		{
			if ((&prt != &model()->ground()) && (!prt.active()))
			{
				file << "part attributes  &\r\n"
					<< "    part_name = ." << model_name << "." << prt.name() << "  &\r\n"
					<< "    active = off \r\n!\r\n";
			}
		}
		for (auto &jnt : model()->jointPool())
		{
			if (!jnt.active())
			{
				file << "constraint attributes  &\r\n"
					<< "    constraint_name = ." << model_name << "." << jnt.name() << "  &\r\n"
					<< "    active = off \r\n!\r\n";
			}
		}
		for (auto &mot : model()->motionPool())
		{
			if (!mot.active())
			{
				file << "constraint attributes  &\r\n"
					<< "    constraint_name = ." << model_name << "." << mot.name() << "  &\r\n"
					<< "    active = off \r\n!\r\n";
			}
		}
		for (auto &gmb : model()->generalMotionPool())
		{
			if (!gmb.active()){
				if (auto gm = dynamic_cast<GeneralMotion*>(&gmb)) {

					file << "ude attributes  &\r\n"
						<< "    instance_name = ." << model_name << "." << gm->name() << "  &\r\n"
						<< "    active = off \r\n!\r\n";
				}
				else if (auto gm2 = dynamic_cast<PointMotion*>(&gmb)) {
					file << "ude attributes  &\r\n"
						<< "    instance_name = ." << model_name << "." << gm2->name() << "  &\r\n"
						<< "    active = off \r\n!\r\n";
				}
				else if (auto gm3 = dynamic_cast<XyztMotion*>(&gmb)) {
					file << "ude attributes  &\r\n"
						<< "    instance_name = ." << model_name << "." << gm3->name() << "  &\r\n"
						<< "    active = off \r\n!\r\n";
				}
			}
		}
		for (auto &fce : model()->forcePool())
		{
			if (!fce.active())
			{
				file << "force attributes  &\r\n"
					<< "    force_name = ." << model_name << "." << fce.name() << "  &\r\n"
					<< "    active = off \r\n!\r\n";
			}
		}
		
	}
	auto AdamsSimulator::saveAdams(const std::string &filename)->void{
		std::string filename_ = filename;
		if (filename_.size() < 4 || filename_.substr(filename.size() - 4, 4) != ".cmd")	{
			filename_ += ".cmd";
		}

		std::ofstream file;
		file.open(filename_, std::ios::out | std::ios::trunc);

		saveAdams(file);

		file.close();
	}
	auto AdamsSimulator::saveAdams(std::ofstream &file)->void
	{
		model()->simResultPool().push_back(new SimResult);
		model()->simResultPool().back().resetModel(this->model());
		model()->simResultPool().back().allocateMemory();

		model()->simResultPool().back().record();

		saveAdams(file, model()->simResultPool().back(), 0);

		model()->simResultPool().erase(model()->simResultPool().end() - 1);
	}
	auto AdamsSimulator::adamsID(const Marker &mak)const->Size
	{
		Size size{ 0 };

		for (auto &prt : model()->partPool())
		{
			if (&prt == &mak.fatherPart()) break;
			size += prt.markerPool().size();
		}

		size += mak.id() + 1;

		return size;
	}
	auto AdamsSimulator::adamsID(const Part &prt)const->Size { return (&prt == &model()->ground()) ? 1 : prt.id() + (model()->ground().id() < prt.id() ? 1 : 2); }
	AdamsSimulator::~AdamsSimulator() = default;
	AdamsSimulator::AdamsSimulator(const std::string &name) : Simulator(name) {}
	ARIS_DEFINE_BIG_FOUR_CPP_NOEXCEPT(AdamsSimulator);


	ARIS_REGISTRATION
	{
		//auto timeresult_to_text = [](SimResult::TimeResult *result)->std::string
		//{
		//	//std::stringstream ss;
		//	//ss << std::setprecision(15);
		//	//ss.str().reserve((25 * 1 + 1)*result->imp_->time_.size());

		//	//for (auto &t : imp_->time_)ss << t << std::endl;
		//};
		
		aris::core::class_<SimResult::TimeResult>("TimeResult")
			;

		auto to_data_idx = [](Calibrator *obj, aris::core::Matrix data_idx)->void {
			obj->setDataIndex((int)data_idx.data()[0], (int)data_idx.data()[1], (int)data_idx.data()[2], (int)data_idx.data()[3]);
		};
		auto from_data_idx = [](Calibrator *obj)->aris::core::Matrix {
			return aris::core::Matrix{
				(double)std::get<0>(obj->dataIndex()), 
				(double)std::get<1>(obj->dataIndex()), 
				(double)std::get<2>(obj->dataIndex()), 
				(double)std::get<3>(obj->dataIndex())
			};
		};

#define VECTOR_TO_MATRIX(set_name, get_name) auto FUNC_SET_##set_name = [](Calibrator *obj, aris::core::Matrix data)->void {	\
	 		obj->set_name(std::vector<double>(data.begin(), data.end()));								\
		};																																	\
		auto FUNC_GET_##get_name = [](Calibrator *obj)->aris::core::Matrix {																\
	 		return aris::core::Matrix(1, obj->get_name().size(), obj->get_name().data());													\
		};																														

		VECTOR_TO_MATRIX(setVelocityRatio,		velocityRatio);
		VECTOR_TO_MATRIX(setTorqueConstant,		torqueConstant);
		VECTOR_TO_MATRIX(setTorqueWeight,		torqueWeight);
		VECTOR_TO_MATRIX(setVelocityDeadZone,	velocityDeadZone);

		aris::core::class_<Calibrator>("Calibrator")
			.prop("data_index", &to_data_idx, &from_data_idx)
			.prop("filter_window_size", &Calibrator::setFilterWindowSize,  &Calibrator::filterWindowSize)
			.prop("tolerable_variance", &Calibrator::setTolerableVariance, &Calibrator::tolerableVariance)
			.prop("velocity_ratio", &FUNC_SET_setVelocityRatio, &FUNC_GET_velocityRatio)
			.prop("torque_constant", &FUNC_SET_setTorqueConstant, &FUNC_GET_torqueConstant)
			.prop("torque_weight", &FUNC_SET_setTorqueWeight, &FUNC_GET_torqueWeight)
			.prop("velocity_dead_zone", &FUNC_SET_setVelocityDeadZone, &FUNC_GET_velocityDeadZone)
			;
	}
}
