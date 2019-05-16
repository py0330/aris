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

namespace aris::dynamic
{
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

		for (auto &m : ancestor<Model>()->motionPool())
		{
			if (m.active())
			{
				imp_->m_++;
				imp_->k_ += 3;
			}
		}
		for (auto &p : ancestor<Model>()->partPool())
		{
			if (p.active() && &p != &ancestor<Model>()->ground())
			{
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
		for (auto &prt : ancestor<Model>()->partPool())
		{
			if (prt.active())
			{
				active_parts.push_back(&prt);
				imp_->dyn_m_ += 6;
			}
		}

		imp_->cst_blk_vec_.clear();
		for (auto &jnt : ancestor<Model>()->jointPool())
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
		for (auto &mot : ancestor<Model>()->motionPool())
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
		for (auto &gmt : ancestor<Model>()->generalMotionPool())
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
		for (auto &fce : ancestor<Model>()->forcePool())
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
			C[at(i, i, imp_->dyn_n_)] = 1;
		}
		for (auto &b : imp_->cst_blk_vec_)
		{
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
		for (auto &b : imp_->cst_blk_vec_)
		{
			if (dynamic_cast<Motion*>(b.c))
			{
				s_mc(1, imp_->dyn_m_, C_inv + at(b.col, 0, imp_->dyn_m_), D + at(di, 0, imp_->dyn_m_));
				di++;
			}
		}

		// make B //
		for (auto &b : imp_->cst_blk_vec_)
		{
			Size row{ 0 };
			for (auto &p : ancestor<Model>()->partPool())
			{
				double cm[6][6], vs[6];

				s_inv_tv(*p.pm(), p.vs(), vs);
				s_cmf(vs, *cm);
				s_mm(imp_->m_, 6, 6, D + at(0, row, imp_->dyn_m_), imp_->dyn_m_, *cm, 6, B + at(0, row, imp_->dyn_m_), imp_->dyn_m_);

				row += 6;
			}
		}

		// make A finally //
		int col1 = 0, col2 = 6;
		for (auto &prt : ancestor<Model>()->partPool())
		{
			if (prt.active() && &prt != &ancestor<Model>()->ground())
			{
				double q[6]{ 0 };

				s_inv_tv(*prt.pm(), prt.as(), q);
				s_inv_tva(-1.0, *prt.pm(), ancestor<Model>()->environment().gravity(), q);

				double v[6];
				s_inv_tv(*prt.pm(), prt.vs(), v);

				for (std::size_t j = 0; j < imp_->m_; ++j)
				{
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
		for (auto &b : imp_->cst_blk_vec_)
		{
			if (dynamic_cast<Motion*>(b.c))
			{
				A[at(ai, g() + ai * 3 + 0, n())] = s_sgn(dynamic_cast<Motion*>(b.c)->mv(), dynamic_cast<Motion*>(b.c)->frcZeroCheck());
				A[at(ai, g() + ai * 3 + 1, n())] = dynamic_cast<Motion*>(b.c)->mv();
				A[at(ai, g() + ai * 3 + 2, n())] = dynamic_cast<Motion*>(b.c)->ma();
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
		for (auto &prt : ancestor<Model>()->partPool())
		{
			if (prt.active() && &prt != &ancestor<Model>()->ground())
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
	auto makeDataset(const Calibrator *clb, const std::vector<double> &mtx, std::vector< std::vector<std::vector<double> >*> &dataset)
	{
		const int filter_size = 10;
		const int mot_num = 6;
		const int pos_at = 1;
		const int vel_at = 2;
		const int cur_at = 3;
		const int mot_data_num = 4;

		const double torque_constant[6]{ 0.283 * 81 * 4808, 0.283 * 81 * 4808, 0.276 * 81 * 2546, 0.226 * 72.857 * 1556, 0.219 * 81 * 849, 0.219 * 50 * 849 };

		auto &pos = *dataset[0];
		auto &vel = *dataset[1];
		auto &acc = *dataset[2];
		auto &fce = *dataset[3];

		auto num = mtx.size() / 24 / 10 - 2;
		for (int i = 0; i < mot_num; ++i)
		{
			pos[i].reserve(num + pos[i].size());
			vel[i].reserve(num + vel[i].size());
			acc[i].reserve(num + acc[i].size());
			fce[i].reserve(num + fce[i].size());
		}

		for (int i = 1; i < num + 1; ++i)
		{
			for (int j = 0; j < mot_num; ++j)
			{
				// make actual pos //
				pos[j].push_back(0.0);
				for (int k = 0; k < filter_size; ++k)
				{
					pos[j].back() += mtx[i * 24 * filter_size + k * 24 + j * 4 + 1] / filter_size;
				}

				// make actual vel //
				vel[j].push_back(0.0);
				for (int k = 0; k < filter_size; ++k)
				{
					vel[j].back() += mtx[i * 24 * filter_size + k * 24 + j * 4 + 2] / filter_size;
				}

				// make actual acc //
				const int avg_size = 4;

				double r = mtx[i * 24 * filter_size + j * 4 + 24 * filter_size + 2] / (1 + avg_size * 2);
				double l = mtx[i * 24 * filter_size + j * 4 + 2] / (1 + avg_size * 2);
				for (int k = 0; k < avg_size; ++k)
				{
					r += mtx[i * 24 * filter_size + j * 4 + 24 * filter_size + 2 + 24 * k] / (1 + avg_size * 2);
					r += mtx[i * 24 * filter_size + j * 4 + 24 * filter_size + 2 - 24 * k] / (1 + avg_size * 2);
					l += mtx[i * 24 * filter_size + j * 4 + 2 + 24 * k] / (1 + avg_size * 2);
					l += mtx[i * 24 * filter_size + j * 4 + 2 - 24 * k] / (1 + avg_size * 2);
				}
				acc[j].push_back((r - l) * 1000 / filter_size);

				// make actual fce //
				fce[j].push_back(0.0);
				for (int k = 0; k < filter_size; ++k)
				{
					fce[j].back() += mtx[i * 24 * filter_size + k * 24 + j * 4 + 3] / filter_size * torque_constant[j] / 1e6;
				}
			}
		}
	}
	auto Calibrator::clbFiles(const std::vector<std::string> &file_paths)->void
	{
		// make datasets //
		std::cout << "making datasets" << std::endl;
		std::vector<std::vector<double> > pos(6);
		std::vector<std::vector<double> > vel(6);
		std::vector<std::vector<double> > acc(6);
		std::vector<std::vector<double> > fce(6);

		for (auto &file : file_paths)
		{
			std::cout << "----loading file:" << file << std::endl;
			auto mtx = aris::dynamic::dlmread(file.c_str());
			std::cout << "----making data" << std::endl;
			std::vector<std::vector<std::vector<double> > *> dataset{ &pos, &vel, &acc, &fce };
			makeDataset(this, mtx, dataset);
		}
		
		// make calibration matrix //
		std::cout << "make calibration matrix" << std::endl;
		this->allocateMemory();

		auto num = pos[0].size();
		std::vector<double> A(num * m() * n()), b(num * m(), 0.0), tau(num * m(), 0.0), x(num * m(), 0.0);
		std::vector<aris::Size> p(num * m());

		Size rows{ 0 }, cols{ n() };
		for (int i = 0; i < num; ++i)
		{
			for (int j = 0; j < 6; ++j)
			{
				this->model().motionPool()[j].setMp(pos[j][i]);
				this->model().motionPool()[j].setMv(vel[j][i]);
				this->model().motionPool()[j].setMa(acc[j][i]);
				this->model().motionPool()[j].setMf(fce[j][i]);
			}

			this->model().solverPool().at(1).kinPos();
			this->model().solverPool().at(1).kinVel();
			this->model().solverPool().at(2).dynAccAndFce();

			for (int j = 0; j < 6; ++j)
			{
				this->model().motionPool()[j].setMf(fce[j][i]);
			}
			this->clb();

			for (int j = 0; j < 6; ++j)
			{
				if (std::abs(this->model().motionPool()[j].mv()) < 0.01)continue;

				std::copy_n(this->A() + j * n(), n(), A.data() + rows * n());
				b[rows] = this->b()[j];
				rows++;
			}
		}
		auto max_value = *std::max_element(A.begin(), A.begin() + rows * n());
		auto min_value = *std::max_element(A.begin(), A.begin() + rows * n());
		auto real_max = std::max(std::abs(max_value), std::abs(min_value));
		std::cout << "A size:" << rows << "x" << cols << std::endl;
		std::cout << "max value of A:" << real_max << std::endl;

		// solve calibration matrix //
		std::cout << "solve calibration matrix" << std::endl;
		aris::Size rank;
		double zero_check = 1e-6;
		s_householder_utp(rows, n(), A.data(), A.data(), tau.data(), p.data(), rank, zero_check);
		s_householder_utp_sov(rows, n(), 1, rank, A.data(), tau.data(), p.data(), b.data(), x.data(), zero_check);
		std::cout << "rank:" << rank << std::endl;

		std::cout << "inertia result:" << std::endl;
		dsp(6, 10, x.data());
		std::cout << "friction result:" << std::endl;
		dsp(6, 3, x.data() + 60);

		// update inertias //
		updateInertiaParam(x.data());
	}
	auto Calibrator::verifyFiles(const std::vector<std::string> &file_paths)->void
	{
		// make datasets //
		std::cout << "making datasets" << std::endl;
		std::vector<std::vector<double> > pos(6);
		std::vector<std::vector<double> > vel(6);
		std::vector<std::vector<double> > acc(6);
		std::vector<std::vector<double> > fce(6);

		for (auto &file : file_paths)
		{
			std::cout << "----loading file:" << file << std::endl;
			auto mtx = aris::dynamic::dlmread(file.c_str());
			std::cout << "----making data" << std::endl;
			std::vector<std::vector<std::vector<double> > *> dataset{ &pos, &vel, &acc, &fce };
			makeDataset(this, mtx, dataset);
		}

		// now test datasets //
		auto num = pos[0].size();
		std::cout << "clb finished now compute dynamics of this dataset" << std::endl;
		std::vector<std::vector<double> > f(6, std::vector<double>(num));
		std::vector<std::vector<double> > ff(6, std::vector<double>(num));
		std::vector<std::vector<double> > fd(6, std::vector<double>(num));
		for (int i = 0; i < num; ++i)
		{
			for (int j = 0; j < 6; ++j)
			{
				this->model().motionPool()[j].setMp(pos[j][i]);
				this->model().motionPool()[j].setMv(vel[j][i]);
				this->model().motionPool()[j].setMa(acc[j][i]);
			}

			this->model().solverPool().at(1).kinPos();
			this->model().solverPool().at(1).kinVel();
			this->model().solverPool().at(2).dynAccAndFce();

			for (int j = 0; j < 6; ++j)
			{
				f[j][i] = this->model().motionPool()[j].mf();
			}
		}

		



		std::cout << "dynamic finished, now output results" << std::endl;

		for (int i = 0; i<6; ++i)
		{
			char posn[1024], veln[1024], accn[1024], fcen[1024], fn[1024], ffn[1024], fdn[1024];

			sprintf(posn, "C:\\Users\\py033\\Desktop\\data_after\\pos%d.txt", i);
			sprintf(veln, "C:\\Users\\py033\\Desktop\\data_after\\vel%d.txt", i);
			sprintf(accn, "C:\\Users\\py033\\Desktop\\data_after\\acc%d.txt", i);
			sprintf(fcen, "C:\\Users\\py033\\Desktop\\data_after\\fce%d.txt", i);
			sprintf(fn, "C:\\Users\\py033\\Desktop\\data_after\\f%d.txt", i);
			sprintf(ffn, "C:\\Users\\py033\\Desktop\\data_after\\ff%d.txt", i);
			sprintf(fdn, "C:\\Users\\py033\\Desktop\\data_after\\fd%d.txt", i);

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
	auto Calibrator::clbFile(const std::string &file_paths)->void
	{
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
		
		for (int i = 0; i < num; ++i)
		{
			for (int j = 0; j < 6; ++j)
			{
				// make actual pos //
				pos[j][i] = 0.0;
				for (int k = 0; k < 10; ++k)
				{
					pos[j][i] += mtx[i * 240 + k * 24 + j * 4 + 1] / 10.0;
				}

				// make actual vel //
				vel[j][i] = 0.0;
				for (int k = 0; k < 10; ++k)
				{
					vel[j][i] += mtx[i * 240 + k * 24 + j * 4 + 2] / 10.0;
				}

				// make actual acc //
				acc[j][i] = (mtx[i * 240 + j * 4 + 242] - mtx[i * 240 + j * 4 + 2])*100.0;

				// make actual fce //
				fce[j][i] = 0.0;
				for (int k = 0; k < 10; ++k)
				{
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
		for (int i = 0; i < num; ++i)
		{
			for (int j = 0; j < 6; ++j)
			{
				this->model().motionPool()[j].setMp(pos[j][i]);
				this->model().motionPool()[j].setMv(vel[j][i]);
				this->model().motionPool()[j].setMa(acc[j][i]);
				this->model().motionPool()[j].setMf(fce[j][i]);
			}

			this->model().solverPool().at(1).kinPos();
			this->model().solverPool().at(1).kinVel();
			this->model().solverPool().at(2).dynAccAndFce();

			for (int j = 0; j < 6; ++j)
			{
				this->model().motionPool()[j].setMf(fce[j][i]);
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
		for (int i = 0; i < num; ++i)
		{
			for (int j = 0; j < 6; ++j)
			{
				this->model().motionPool()[j].setMp(pos[j][i]);
				this->model().motionPool()[j].setMv(vel[j][i]);
				this->model().motionPool()[j].setMa(acc[j][i]);
				this->model().motionPool()[j].setMf(fce[j][i]);
			}

			this->model().solverPool().at(1).kinPos();
			this->model().solverPool().at(1).kinVel();
			this->model().solverPool().at(2).dynAccAndFce();

			for (int j = 0; j < 6; ++j)
			{
				f[j][i] = this->model().motionPool()[j].mf();
				ff[j][i] = this->model().motionPool()[j].mfFrc();
				fd[j][i] = this->model().motionPool()[j].mfDyn();
			}
		}





		std::cout << "dynamic finished, now output results" << std::endl;


		//dsp(1, 3, this->model().motionPool()[0].frcCoe());



		
		for (int i = 0; i<6;++i)
		{
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
	auto Calibrator::updateInertiaParam(const double *x)->void
	{
		Size xi = 0;
		for (auto &prt : ancestor<Model>()->partPool())
		{
			if (prt.active() && &prt != &ancestor<Model>()->ground())
			{
				prt.setPrtIv(x + xi);
				xi += 10;
			}
		}
		// make x from frictions //
		Size bi = 0;
		for (auto &blk : imp_->cst_blk_vec_)
		{
			if (dynamic_cast<Motion*>(blk.c))
			{
				dynamic_cast<Motion*>(blk.c)->setFrcCoe(x + g() + bi * 3);
				bi++;
			}
		}
	}
	Calibrator::~Calibrator() = default;
	Calibrator::Calibrator(const std::string &name) : Element(name), imp_(new Imp) {}
	ARIS_DEFINE_BIG_FOUR_CPP(Calibrator);

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
	auto SimResult::TimeResult::record()->void { imp_->time_.push_back(ancestor<Model>()->time()); }
	auto SimResult::TimeResult::restore(Size pos)->void { ancestor<Model>()->setTime(imp_->time_.at(pos)); }
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
		if (ancestor<Model>()->findByName("part_pool") == ancestor<Model>()->children().end())
			throw std::runtime_error("you must insert \"part_pool\" node before insert " + type() + " \"" + name() + "\"");

		auto &part_pool = static_cast<aris::core::ObjectPool<Part, Element>&>(*ancestor<Model>()->findByName("part_pool"));

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
		if (!imp_->constraint_ && ancestor<Model>()->findByName("joint_pool") != ancestor<Model>()->children().end())
		{
			auto &pool = static_cast<aris::core::ObjectPool<Joint, Element>&>(*ancestor<Model>()->findByName("joint_pool"));
			auto c = pool.findByName(xml_ele.Attribute("constraint"));
			if (c != pool.end())imp_->constraint_ = &*c;
		}
		if (!imp_->constraint_ && ancestor<Model>()->findByName("motion_pool") != ancestor<Model>()->children().end())
		{
			auto &pool = static_cast<aris::core::ObjectPool<Motion, Element>&>(*ancestor<Model>()->findByName("motion_pool"));
			auto c = pool.findByName(xml_ele.Attribute("constraint"));
			if (c != pool.end())imp_->constraint_ = &*c;
		}
		if (!imp_->constraint_ && ancestor<Model>()->findByName("general_motion_pool") != ancestor<Model>()->children().end())
		{
			auto &pool = static_cast<aris::core::ObjectPool<GeneralMotion, Element>&>(*ancestor<Model>()->findByName("general_motion_pool"));
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

		imp_->time_result_ = findOrInsertType<TimeResult>();
		imp_->constraint_result_pool_ = findOrInsertType<aris::core::ObjectPool<SimResult::ConstraintResult, Element> >();
		imp_->part_result_pool_ = findOrInsertType<aris::core::ObjectPool<SimResult::PartResult, Element> >();
	}
	auto SimResult::timeResult()->TimeResult& { return *imp_->time_result_; }
	auto SimResult::partResultPool()->aris::core::ObjectPool<SimResult::PartResult, Element>& { return *imp_->part_result_pool_; }
	auto SimResult::constraintResultPool()->aris::core::ObjectPool<SimResult::ConstraintResult, Element>& { return *imp_->constraint_result_pool_; }
	auto SimResult::allocateMemory()->void
	{
		partResultPool().clear();
		for (auto &p : ancestor<Model>()->partPool())partResultPool().add<PartResult>(p.name() + "_result", &p);
		constraintResultPool().clear();
		for (auto &c : ancestor<Model>()->jointPool())constraintResultPool().add<ConstraintResult>(c.name() + "_result", &c);
		for (auto &c : ancestor<Model>()->motionPool())constraintResultPool().add<ConstraintResult>(c.name() + "_result", &c);
		for (auto &c : ancestor<Model>()->generalMotionPool())constraintResultPool().add<ConstraintResult>(c.name() + "_result", &c);
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
	auto Simulator::simulate(aris::plan::Plan &plan, SimResult &result)->void
	{
		result.allocateMemory();
		// 记录初始状态 //
		result.record();

		aris::plan::PlanTarget target
		{
			&plan,
			nullptr,
			&model(),
			nullptr,
			0,
			0,
			std::vector<std::uint64_t>(model().motionPool().size(), 0),
			std::any(),
			0,
			0,
			aris::control::Master::RtStasticsData{ 0,0,0,0x8fffffff,0,0,0 },
			std::any(),
			aris::plan::PlanTarget::CANCELLED,
			std::future<void>()
		};

		// 记录轨迹中的状态 //
		for (;plan.executeRT(target) != 0;++target.count)result.record();
			
		// 记录结束状态 //
		result.record();
		result.restore(0);
	}
	Simulator::~Simulator() = default;
	Simulator::Simulator(const std::string &name) : Element(name), imp_(new Imp) {}
	ARIS_DEFINE_BIG_FOUR_CPP(Simulator);

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

		if (ancestor<Model>()->findByName("solver_pool") == ancestor<Model>()->children().end())
			throw std::runtime_error("you must insert \"solver_pool\" node before insert " + type() + " \"" + name() + "\"");

		auto &solver_pool = static_cast<aris::core::ObjectPool<Solver, Element>&>(*ancestor<Model>()->findByName("solver_pool"));

		if (!xml_ele.Attribute("solver"))throw std::runtime_error(std::string("xml element \"") + name() + "\" must have Attribute \"solver\"");
		auto s = solver_pool.findByName(xml_ele.Attribute("solver"));
		if (s == solver_pool.end())	throw std::runtime_error(std::string("can't find solver for element \"") + this->name() + "\"");

		imp_->solver_ = &*s;
	}
	auto SolverSimulator::solver()->Solver& { return *imp_->solver_; }
	auto SolverSimulator::simulate(aris::plan::Plan &plan, SimResult &result)->void
	{
		solver().allocateMemory();
		Simulator::simulate(plan, result);
	}
	SolverSimulator::~SolverSimulator() = default;
	SolverSimulator::SolverSimulator(const std::string &name, Solver *solver) : Simulator(name), imp_(new Imp(solver)) {}
	ARIS_DEFINE_BIG_FOUR_CPP(SolverSimulator);

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
		std::vector<std::vector<double>> mot_akima(ancestor<Model>()->motionPool().size(), std::vector<double>(result.size() + 1));
		std::vector<std::vector<std::array<double, 6>>> gm_akima(ancestor<Model>()->generalMotionPool().size(), std::vector<std::array<double, 6>>(result.size() + 1));
		if (pos == -1)
		{
			if (result.size() < 4)throw std::runtime_error("failed to AdamsSimulator::saveAdams: because result size is smaller than 4\n");

			for (Size i(-1); ++i < result.size() + 1;)
			{
				result.restore(i);
				time.at(i) = ancestor<Model>()->time();
				for (Size j(-1); ++j < ancestor<Model>()->motionPool().size();)
				{
					ancestor<Model>()->motionPool().at(j).updMp();
					mot_akima.at(j).at(i) = ancestor<Model>()->motionPool().at(j).mpInternal();
				}
				for (Size j(-1); ++j < ancestor<Model>()->generalMotionPool().size();)
				{
					ancestor<Model>()->generalMotionPool().at(j).updMpm();
					ancestor<Model>()->generalMotionPool().at(j).getMpe(gm_akima.at(j).at(i).data(), "123");
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
			<< "   model_name = " << this->ancestor<Model>()->name() << "\r\n"
			<< "!\r\n"
			<< "view erase\r\n"
			<< "!\r\n"
			<< "!---------------------------------- Accgrav -----------------------------------!\r\n"
			<< "!\r\n"
			<< "!\r\n"
			<< "force create body gravitational  &\r\n"
			<< "    gravity_field_name = gravity  &\r\n"
			<< "    x_component_gravity = " << ancestor<Model>()->environment().gravity()[0] << "  &\r\n"
			<< "    y_component_gravity = " << ancestor<Model>()->environment().gravity()[1] << "  &\r\n"
			<< "    z_component_gravity = " << ancestor<Model>()->environment().gravity()[2] << "\r\n"
			<< "!\r\n";
		for (auto &part : ancestor<Model>()->partPool())
		{
			if (&part == &ancestor<Model>()->ground())
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
					<< "    default_coordinate_system = ." << ancestor<Model>()->name() << ".ground\r\n"
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
					<< "    default_coordinate_system = ." << ancestor<Model>()->name() << ".ground\r\n"
					<< "!\r\n"
					<< "part create rigid_body name_and_position  &\r\n"
					<< "    part_name = ." << ancestor<Model>()->name() << "." << part.name() << "  &\r\n"
					<< "    adams_id = " << adamsID(part) << "  &\r\n"
					<< "    location = (" << loc.toString() << ")  &\r\n"
					<< "    orientation = (" << ori.toString() << ")\r\n"
					<< "!\r\n"
					<< "defaults coordinate_system  &\r\n"
					<< "    default_coordinate_system = ." << ancestor<Model>()->name() << "." << part.name() << " \r\n"
					<< "!\r\n";


				double mass = part.prtIv()[0] == 0 ? 1 : part.prtIv()[0];
				std::fill_n(pe, 6, 0);
				pe[0] = part.prtIv()[1] / mass;
				pe[1] = part.prtIv()[2] / mass;
				pe[2] = part.prtIv()[3] / mass;

				file << "! ****** cm and mass for current part ******\r\n"
					<< "marker create  &\r\n"
					<< "    marker_name = ." << ancestor<Model>()->name() << "." << part.name() << ".cm  &\r\n"
					<< "    adams_id = " << adamsID(part) + std::accumulate(ancestor<Model>()->partPool().begin(), ancestor<Model>()->partPool().end(), Size(0), [](Size a, Part &b) {return a + b.markerPool().size(); }) << "  &\r\n"
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
					<< "    part_name = ." << ancestor<Model>()->name() << "." << part.name() << "  &\r\n"
					<< "    mass = " << part.prtIv()[0] << "  &\r\n"
					<< "    center_of_mass_marker = ." << ancestor<Model>()->name() << "." << part.name() << ".cm  &\r\n"
					<< "    inertia_marker = ." << ancestor<Model>()->name() << "." << part.name() << ".cm  &\r\n"
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
					<< "    part_name = ." << ancestor<Model>()->name() << "." << part.name() << "  &\r\n"
					<< "    vx = " << cm_vs[0] << "  &\r\n"
					<< "    vy = " << cm_vs[1] << "  &\r\n"
					<< "    vz = " << cm_vs[2] << "  &\r\n"
					<< "    wx = " << cm_vs[3] << "  &\r\n"
					<< "    wy = " << cm_vs[4] << "  &\r\n"
					<< "    wz = " << cm_vs[5] << "  \r\n"
					<< "!\r\n";

				file << "part modify rigid_body initial_velocity  &\r\n"
					<< "    part_name = ." << ancestor<Model>()->name() << "." << part.name() << "  &\r\n"
					<< "    vm = ." << ancestor<Model>()->name() << "." << part.name() << ".cm  &\r\n"
					<< "    wm = ." << ancestor<Model>()->name() << "." << part.name() << ".cm \r\n"
					<< "!\r\n";


			}

			//导入marker
			for (auto &marker : part.markerPool())
			{
				double pe[6];

				s_pm2pe(*marker.prtPm(), pe, "313");
				core::Matrix ori(1, 3, &pe[3]), loc(1, 3, &pe[0]);

				file << "marker create  &\r\n"
					<< "marker_name = ." << ancestor<Model>()->name() << "." << part.name() << "." << marker.name() << "  &\r\n"
					<< "adams_id = " << adamsID(marker) << "  &\r\n"
					<< "location = (" << loc.toString() << ")  &\r\n"
					<< "orientation = (" << ori.toString() << ")\r\n"
					<< "!\r\n";
			}
		}
		for (auto &joint : ancestor<Model>()->jointPool())
		{
			std::string type;
			if (dynamic_cast<RevoluteJoint*>(&joint))type = "revolute";
			else if (dynamic_cast<PrismaticJoint*>(&joint))type = "translational";
			else if (dynamic_cast<UniversalJoint*>(&joint))type = "universal";
			else if (dynamic_cast<SphericalJoint*>(&joint))type = "spherical";
			else throw std::runtime_error("unrecognized joint type:" + joint.type());

			file << "constraint create joint " << type << "  &\r\n"
				<< "    joint_name = ." << ancestor<Model>()->name() << "." << joint.name() << "  &\r\n"
				<< "    adams_id = " << adamsID(joint) << "  &\r\n"
				<< "    i_marker_name = ." << ancestor<Model>()->name() << "." << joint.makI().fatherPart().name() << "." << joint.makI().name() << "  &\r\n"
				<< "    j_marker_name = ." << ancestor<Model>()->name() << "." << joint.makJ().fatherPart().name() << "." << joint.makJ().name() << "  \r\n"
				<< "!\r\n";
		}
		for (auto &motion : ancestor<Model>()->motionPool())
		{
			std::string axis_names[6]{ "x","y","z","B1","B2","B3" };
			std::string axis_name = axis_names[motion.axis()];

			std::string akima = motion.name() + "_akima";
			std::string akima_func = "AKISPL(time,0," + akima + ")";
			std::string polynomial_func = static_cast<const std::stringstream &>(std::stringstream() << std::setprecision(16) << motion.mpInternal() << " + " << motion.mv() << " * time + " << motion.ma()*0.5 << " * time * time").str();

			// 构建akima曲线 //
			if (pos == -1)
			{
				file << "data_element create spline &\r\n"
					<< "    spline_name = ." << ancestor<Model>()->name() + "." + motion.name() + "_akima &\r\n"
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
				<< "    motion_name = ." << ancestor<Model>()->name() << "." << motion.name() << "  &\r\n"
				<< "    adams_id = " << adamsID(motion) << "  &\r\n"
				<< "    i_marker_name = ." << ancestor<Model>()->name() << "." << motion.makI().fatherPart().name() << "." << motion.makI().name() << "  &\r\n"
				<< "    j_marker_name = ." << ancestor<Model>()->name() << "." << motion.makJ().fatherPart().name() << "." << motion.makJ().name() << "  &\r\n"
				<< "    axis = " << axis_name << "  &\r\n"
				<< "    function = \"" << (pos == -1 ? akima_func : polynomial_func) << "\"  \r\n"
				<< "!\r\n";
		}
		for (auto &gm : ancestor<Model>()->generalMotionPool())
		{
			file << "ude create instance  &\r\n"
				<< "    instance_name = ." << ancestor<Model>()->name() << "." << gm.name() << "  &\r\n"
				<< "    definition_name = .MDI.Constraints.general_motion  &\r\n"
				<< "    location = 0.0, 0.0, 0.0  &\r\n"
				<< "    orientation = 0.0, 0.0, 0.0  \r\n"
				<< "!\r\n";

			file << "variable modify  &\r\n"
				<< "	variable_name = ." << ancestor<Model>()->name() << "." << gm.name() << ".i_marker  &\r\n"
				<< "	object_value = ." << ancestor<Model>()->name() << "." << gm.makI().fatherPart().name() << "." << gm.makI().name() << " \r\n"
				<< "!\r\n";

			file << "variable modify  &\r\n"
				<< "	variable_name = ." << ancestor<Model>()->name() << "." << gm.name() << ".j_marker  &\r\n"
				<< "	object_value = ." << ancestor<Model>()->name() << "." << gm.makJ().fatherPart().name() << "." << gm.makJ().name() << " \r\n"
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
						<< "    spline_name = ." << ancestor<Model>()->name() + "." + akima + " &\r\n"
						<< "    adams_id = " << ancestor<Model>()->motionPool().size() + adamsID(gm) * 6 + i << "  &\r\n"
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
					<< "	variable_name = ." << ancestor<Model>()->name() << "." << gm.name() << "." << axis_names[i] << "_type  &\r\n"
					<< "	integer_value = 1 \r\n"
					<< "!\r\n";

				file << "variable modify  &\r\n"
					<< "	variable_name = ." << ancestor<Model>()->name() << "." << gm.name() << "." << axis_names[i] << "_func  &\r\n"
					<< "	string_value = \"" + func + "\" \r\n"
					<< "!\r\n";

				file << "variable modify  &\r\n"
					<< "	variable_name = ." << ancestor<Model>()->name() << "." << gm.name() << "." << axis_names[i] << "_ic_disp  &\r\n"
					<< "	real_value = 0.0 \r\n"
					<< "!\r\n";

				file << "variable modify  &\r\n"
					<< "	variable_name = ." << ancestor<Model>()->name() << "." << gm.name() << "." << axis_names[i] << "_ic_velo  &\r\n"
					<< "	real_value = 0.0 \r\n"
					<< "!\r\n";
			}

			file << "ude modify instance  &\r\n"
				<< "	instance_name = ." << ancestor<Model>()->name() << "." << gm.name() << "\r\n"
				<< "!\r\n";
		}
		for (auto &force : ancestor<Model>()->forcePool())
		{
			double fsI[6], fsJ[6], fsI_loc[6];
			force.cptGlbFs(fsI, fsJ);
			s_inv_fs2fs(*force.makI().pm(), fsI, fsI_loc);

			file << "floating_marker create  &\r\n"
				<< "    floating_marker_name = ." << ancestor<Model>()->name() << "." << force.makJ().fatherPart().name() << "." << force.name() << "_FMAK  &\r\n"
				<< "    adams_id = " << adamsID(force) + ancestor<Model>()->partPool().size() + std::accumulate(ancestor<Model>()->partPool().begin(), ancestor<Model>()->partPool().end(), Size(0), [](Size a, Part &b) {return a + b.markerPool().size(); }) << "\r\n"
				<< "!\r\n";

			file << "force create direct general_force  &\r\n"
				<< "    general_force_name = ." << ancestor<Model>()->name() << "." << force.name() << "  &\r\n"
				<< "    adams_id = " << adamsID(force) << "  &\r\n"
				<< "    i_marker_name = ." << ancestor<Model>()->name() << "." << force.makI().fatherPart().name() << "." << force.makI().name() << "  &\r\n"
				<< "    j_floating_marker_name = ." << ancestor<Model>()->name() << "." << force.makJ().fatherPart().name() << "." << force.name() << "_FMAK  &\r\n"
				<< "    ref_marker_name = ." << ancestor<Model>()->name() << "." << force.makI().fatherPart().name() << "." << force.makI().name() << "  &\r\n"
				<< "    x_force_function = \"" << fsI_loc[0] << "\"  &\r\n"
				<< "    y_force_function = \"" << fsI_loc[1] << "\"  &\r\n"
				<< "    z_force_function = \"" << fsI_loc[2] << "\"  &\r\n"
				<< "    x_torque_function = \"" << fsI_loc[3] << "\"  &\r\n"
				<< "    y_torque_function = \"" << fsI_loc[4] << "\"  &\r\n"
				<< "    z_torque_function = \"" << fsI_loc[5] << "\"\r\n"
				<< "!\r\n";

		}

		// geometry, 防止geometry 添加marker，导致marker id冲突
		for (auto &part : ancestor<Model>()->partPool())
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
						<< "	relative_to = ." << ancestor<Model>()->name() << "." << part.name() << " \r\n"
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
						<< "	relative_to = ." << ancestor<Model>()->name() << "." << part.name() << " &\r\n"
						<< "	scale = " << "0.001" << "\r\n"
						<< "!\r\n";
				}
				else if (ShellGeometry* geo = dynamic_cast<ShellGeometry*>(&geometry))
				{
					file << "geometry create shape shell  &\r\n"
						<< "	shell_name = ." << ancestor<Model>()->name() << "." << part.name() << "." << geo->name() << " &\r\n"
						<< "	reference_marker = ." << ancestor<Model>()->name() << "." << part.name() << "." << geo->relativeToMarker().name() << " &\r\n"
						<< "	file_name = \"" << geo->filePath() << "\" &\r\n"
						<< "	wireframe_only = " << "no" << "\r\n"
						<< "!\r\n";
				}
				else
				{
					throw std::runtime_error("unrecognized geometry type:" + geometry.type());
				}

			}
		}

		file << "!----------------------------------- Motify Active -------------------------------------!\r\n!\r\n!\r\n";
		for (auto &prt : ancestor<Model>()->partPool())
		{
			if ((&prt != &ancestor<Model>()->ground()) && (!prt.active()))
			{
				file << "part attributes  &\r\n"
					<< "    part_name = ." << ancestor<Model>()->name() << "." << prt.name() << "  &\r\n"
					<< "    active = off \r\n!\r\n";
			}
		}
		for (auto &jnt : ancestor<Model>()->jointPool())
		{
			if (!jnt.active())
			{
				file << "constraint attributes  &\r\n"
					<< "    constraint_name = ." << ancestor<Model>()->name() << "." << jnt.name() << "  &\r\n"
					<< "    active = off \r\n!\r\n";
			}
		}
		for (auto &mot : ancestor<Model>()->motionPool())
		{
			if (!mot.active())
			{
				file << "constraint attributes  &\r\n"
					<< "    constraint_name = ." << ancestor<Model>()->name() << "." << mot.name() << "  &\r\n"
					<< "    active = off \r\n!\r\n";
			}
		}
		for (auto &gm : ancestor<Model>()->generalMotionPool())
		{
			if (!gm.active())
			{
				file << "ude attributes  &\r\n"
					<< "    instance_name = ." << ancestor<Model>()->name() << "." << gm.name() << "  &\r\n"
					<< "    active = off \r\n!\r\n";
			}
		}
		for (auto &fce : ancestor<Model>()->forcePool())
		{
			if (!fce.active())
			{
				file << "force attributes  &\r\n"
					<< "    force_name = ." << ancestor<Model>()->name() << "." << fce.name() << "  &\r\n"
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
		ancestor<Model>()->simResultPool().add<SimResult>();
		ancestor<Model>()->simResultPool().back().record();

		saveAdams(file, ancestor<Model>()->simResultPool().back(), 0);

		ancestor<Model>()->simResultPool().erase(ancestor<Model>()->simResultPool().end() - 1);
	}
	auto AdamsSimulator::adamsID(const Marker &mak)const->Size
	{
		Size size{ 0 };

		for (auto &prt : ancestor<Model>()->partPool())
		{
			if (&prt == &mak.fatherPart()) break;
			size += prt.markerPool().size();
		}

		size += mak.id() + 1;

		return size;
	}
	auto AdamsSimulator::adamsID(const Part &prt)const->Size { return (&prt == &ancestor<Model>()->ground()) ? 1 : prt.id() + (ancestor<Model>()->ground().id() < prt.id() ? 1 : 2); }
	AdamsSimulator::~AdamsSimulator() = default;
	AdamsSimulator::AdamsSimulator(const std::string &name) : Simulator(name) {}
	ARIS_DEFINE_BIG_FOUR_CPP(AdamsSimulator);
}
