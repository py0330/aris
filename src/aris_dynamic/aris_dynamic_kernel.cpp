#include <cmath>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <algorithm>
#include <limits>
#include <sstream>
#include <cstddef>
#include <array>
#include <list>

#include "aris_dynamic_kernel.h"

namespace aris
{
	namespace dynamic
	{
		auto s_blk_make(const double *mtx, const BlockSize &blk_size_m, const BlockSize &blk_size_n, BlockData &blk_mtx) noexcept->void
		{
			blk_mtx.clear();

			int ld{ 0 };
			for (auto size : blk_size_n)ld += size;


			int begin_row{ 0 };
			for (int i = 0; i < static_cast<int>(blk_size_m.size()); ++i)
			{
				blk_mtx.push_back(std::vector<std::vector<double>>());

				int begin_col{ 0 };
				for (int j = 0; j < static_cast<int>(blk_size_n.size()); ++j)
				{
					blk_mtx.back().push_back(std::vector<double>());
					blk_mtx.back().back().resize(blk_size_m[i] * blk_size_n[j]);
					s_mc(blk_size_m[i], blk_size_n[j], mtx + begin_row*ld + begin_col, ld, blk_mtx.back().back().data(), blk_size_n[j]);

					begin_col += blk_size_n[j];
				}

				begin_row += blk_size_m[i];
			}
		}
		auto s_blk_resolve(const BlockSize &blk_size_m, const BlockSize &blk_size_n, const BlockData &blk_mtx, double *mtx) noexcept->void
		{
			int ld{ 0 };
			for (auto size : blk_size_n)ld += size;

			int begin_row{ 0 };
			for (int i = 0; i < static_cast<int>(blk_size_m.size()); ++i)
			{
				int begin_col{ 0 };
				for (int j = 0; j < static_cast<int>(blk_size_n.size()); ++j)
				{
					if (blk_mtx[i][j].empty())
					{
						s_fill(blk_size_m[i], blk_size_n[j], 0, mtx + begin_row*ld + begin_col, ld);
					}
					else
					{
						s_mc(blk_size_m[i], blk_size_n[j], blk_mtx[i][j].data(), blk_size_n[j], mtx + begin_row*ld + begin_col, ld);
					}



					begin_col += blk_size_n[j];
				}

				begin_row += blk_size_m[i];
			}
		}
		auto s_blk_allocate(const BlockSize &blk_size_m, const BlockSize &blk_size_n, BlockData &blk_mtx) noexcept->void
		{
			blk_mtx.clear();
			for (int i = 0; i < static_cast<int>(blk_size_m.size()); ++i)
			{
				blk_mtx.push_back(std::vector<std::vector<double>>());
				for (int j = 0; j < static_cast<int>(blk_size_n.size()); ++j)
				{
					blk_mtx.back().push_back(std::vector<double>());
					blk_mtx.back().back().reserve(blk_size_m[i] * blk_size_n[j]);
				}
			}
		}
		auto s_blk_check_empty_num(const BlockData &blk_mtx)noexcept->int
		{
			int num{ 0 };
			for (auto &row : blk_mtx)for (auto &ele : row)if (ele.empty())++num;
			return num;
		}

		auto s_blk_norm(const BlockSize &blk_size_m, const BlockData &blk_mtx)noexcept->double
		{
			double norm{ 0 };
			for (std::size_t i = 0; i < blk_size_m.size(); ++i)
			{
				for (auto j = 0; j < blk_size_m[i]; ++j)
				{
					norm += blk_mtx[i][0][j] * blk_mtx[i][0][j];
				}
			}

			return std::sqrt(norm);
		}
		auto s_blk_norm_col(const BlockSize &blk_size_m, const BlockSize &blk_size_n, const BlockData &blk_mtx, int blk_i, int blk_j, const double *data)noexcept->double
		{
			auto begin_n = blk_size_m[blk_i] - (data - blk_mtx[blk_i][blk_j].data()) / blk_size_n[blk_j];
			auto col_id = (data - blk_mtx[blk_i][blk_j].data()) % blk_size_n[blk_j];

			double norm = s_vv(begin_n, data, blk_size_n[blk_j], data, blk_size_n[blk_j]);
			for (std::size_t i = blk_i + 1; i < blk_size_m.size(); ++i)
			{
				if(!blk_mtx[i][blk_j].empty())
				norm += s_vv(blk_size_m[i], blk_mtx[i][blk_j].data() + id(0, col_id, blk_size_n[blk_j]), blk_size_n[blk_j], blk_mtx[i][blk_j].data() + id(0, col_id, blk_size_n[blk_j]), blk_size_n[blk_j]);
			}

			return std::sqrt(norm);
		}
		auto s_blk_norm_row(const BlockSize &blk_size_m, const BlockSize &blk_size_n, const BlockData &blk_mtx, int blk_i, int blk_j, const double *data)noexcept->double
		{
			auto begin_n = blk_size_n[blk_j] - (data - blk_mtx[blk_i][blk_j].data()) % blk_size_n[blk_j];
			auto row_id = (data - blk_mtx[blk_i][blk_j].data()) / blk_size_n[blk_j];

			double norm = s_vv(begin_n, data, data);
			for (std::size_t j = blk_j + 1; j < blk_size_n.size(); ++j)
			{
				if (!blk_mtx[blk_i][j].empty())
					norm += s_vv(blk_size_n[j], blk_mtx[blk_i][j].data() + id(row_id, 0, blk_size_n[j]), blk_mtx[blk_i][j].data() + id(row_id, 0, blk_size_n[j]));
			}

			return std::sqrt(norm);
		}



		auto s_blk_mm(const BlockSize &blk_size_m, const BlockSize &blk_size_n, const BlockSize &blk_size_k, const BlockData &A, const BlockData &B, BlockData &C)noexcept->void
		{
			for (std::size_t i{ 0 }; i < blk_size_m.size(); ++i)
			{
				for (std::size_t j{ 0 }; j < blk_size_n.size(); ++j)
				{
					C[i][j].clear();
					for (std::size_t k{ 0 }; k < blk_size_k.size(); ++k)
					{
						if (A[i][k].empty() || B[k][j].empty())continue;
						if (C[i][j].empty()) 
						{
							C[i][j].resize(blk_size_m[i] * blk_size_n[j]);
							s_mm(blk_size_m[i], blk_size_n[j], blk_size_k[k], A[i][k].data(), B[k][j].data(), C[i][j].data());
						}
						else
						{
							s_mma(blk_size_m[i], blk_size_n[j], blk_size_k[k], A[i][k].data(), B[k][j].data(), C[i][j].data());
						}
					}
				}
			}
		}
		auto s_blk_mmNT(const BlockSize &blk_size_m, const BlockSize &blk_size_n, const BlockSize &blk_size_k, const BlockData &A, const BlockData &B, BlockData &C)noexcept->void
		{
			for (std::size_t i{ 0 }; i < blk_size_m.size(); ++i)
			{
				for (std::size_t j{ 0 }; j < blk_size_n.size(); ++j)
				{
					C[i][j].clear();
					for (std::size_t k{ 0 }; k < blk_size_k.size(); ++k)
					{
						if (A[i][k].empty() || B[j][k].empty())continue;
						if (C[i][j].empty())
						{
							C[i][j].resize(blk_size_m[i] * blk_size_n[j]);
							s_mmNT(blk_size_m[i], blk_size_n[j], blk_size_k[k], A[i][k].data(), B[j][k].data(), C[i][j].data());
						}
						else
						{
							s_mmaNT(blk_size_m[i], blk_size_n[j], blk_size_k[k], A[i][k].data(), B[j][k].data(), C[i][j].data());
						}
					}
				}
			}
		}
		auto s_blk_mmTN(const BlockSize &blk_size_m, const BlockSize &blk_size_n, const BlockSize &blk_size_k, const BlockData &A, const BlockData &B, BlockData &C)noexcept->void
		{
			for (std::size_t i{ 0 }; i < blk_size_m.size(); ++i)
			{
				for (std::size_t j{ 0 }; j < blk_size_n.size(); ++j)
				{
					C[i][j].clear();
					for (std::size_t k{ 0 }; k < blk_size_k.size(); ++k)
					{
						if (A[k][i].empty() || B[k][j].empty())continue;
						if (C[i][j].empty())
						{
							C[i][j].resize(blk_size_m[i] * blk_size_n[j]);
							s_mmTN(blk_size_m[i], blk_size_n[j], blk_size_k[k], A[k][i].data(), B[k][j].data(), C[i][j].data());
						}
						else
						{
							s_mmaTN(blk_size_m[i], blk_size_n[j], blk_size_k[k], A[k][i].data(), B[k][j].data(), C[i][j].data());
						}
					}
				}
			}
		}
		auto s_blk_mmTT(const BlockSize &blk_size_m, const BlockSize &blk_size_n, const BlockSize &blk_size_k, const BlockData &A, const BlockData &B, BlockData &C)noexcept->void
		{
			for (std::size_t i{ 0 }; i < blk_size_m.size(); ++i)
			{
				for (std::size_t j{ 0 }; j < blk_size_n.size(); ++j)
				{
					C[i][j].clear();
					for (std::size_t k{ 0 }; k < blk_size_k.size(); ++k)
					{
						if (A[k][i].empty() || B[j][k].empty())continue;
						if (C[i][j].empty())
						{
							C[i][j].resize(blk_size_m[i] * blk_size_n[j]);
							s_mmTT(blk_size_m[i], blk_size_n[j], blk_size_k[k], A[k][i].data(), B[j][k].data(), C[i][j].data());
						}
						else
						{
							s_mmaTT(blk_size_m[i], blk_size_n[j], blk_size_k[k], A[k][i].data(), B[j][k].data(), C[i][j].data());
						}
					}
				}
			}
		}

		auto s_blk_llt(const BlockSize &blk_size, const BlockData &A, BlockData &L) noexcept->void
		{
			int m = blk_size.size();

			for (int j = 0; j < m; ++j)
			{
				L[j][j].resize(blk_size[j] * blk_size[j]);

				std::copy(A[j][j].data(), A[j][j].data() + blk_size[j] * blk_size[j], L[j][j].data());
				for (int k = 0; k < j; ++k)
				{
					if (!L[j][k].empty())s_mmaNT(blk_size[j], blk_size[j], blk_size[k], -1.0, L[j][k].data(), L[j][k].data(), L[j][j].data());
				}
				s_llt(blk_size[j], L[j][j].data(), L[j][j].data());

				for (int i = j + 1; i < m; ++i)
				{
					L[i][j].clear();
					L[j][i].clear();

					if (!A[i][j].empty())
					{
						L[i][j].resize(blk_size[i] * blk_size[j]);
						std::copy(A[i][j].data(), A[i][j].data() + blk_size[i] * blk_size[j], L[i][j].data());
					}
					for (int k = 0; k < j; ++k)
					{
						if (L[i][k].empty() || L[j][k].empty())continue;

						if (L[i][j].empty())
						{
							L[i][j].resize(blk_size[i] * blk_size[j]);
							s_mmNT(blk_size[i], blk_size[j], blk_size[k], -1.0, L[i][k].data(), L[j][k].data(), L[i][j].data());
						}
						else
						{
							s_mmaNT(blk_size[i], blk_size[j], blk_size[k], -1.0, L[i][k].data(), L[j][k].data(), L[i][j].data());
						}
					}

					if (!L[i][j].empty())
					{
						L[j][i].resize(blk_size[i] * blk_size[j]);
						s_sov_lmNT(blk_size[j], blk_size[i], L[j][j].data(), L[i][j].data(), L[j][i].data());
						s_transpose(blk_size[j], blk_size[i], L[j][i].data(), L[i][j].data());
					}
				}
			}
		}
		auto s_blk_sov_lm(const BlockSize &blk_size, const BlockSize &rhs_size, const BlockData &L, const BlockData &b, BlockData &x)noexcept->void
		{
			int m = static_cast<int>(blk_size.size());
			int rhs = static_cast<int>(rhs_size.size());

			for (int j = 0; j < rhs; ++j)
			{
				for (int i = 0; i < m; ++i)
				{
					if (!b[i][j].empty())
						x[i][j].assign(b[i][j].begin(), b[i][j].begin() + rhs_size[j] * blk_size[i]);

					for (int k = 0; k < i; ++k)
					{
						if (L[i][k].empty() || x[k][j].empty()) { continue; std::cout << "continue" << std::endl; };
						if (x[i][j].empty())x[i][j].resize(blk_size[i] * rhs_size[j], 0);
						s_mma(blk_size[i], rhs_size[j], blk_size[k], -1.0, L[i][k].data(), x[k][j].data(), x[i][j].data());
					}

					if (x[i][j].empty()) x[i][j].clear();
					else s_sov_lm(blk_size[i], rhs_size[j], L[i][i].data(), x[i][j].data(), x[i][j].data());
				}
			}
		}
		auto s_blk_sov_um(const BlockSize &blk_size, const BlockSize &rhs_size, const BlockData &L, const BlockData &b, BlockData &x)noexcept->void
		{
			int m = static_cast<int>(blk_size.size());
			int rhs = static_cast<int>(rhs_size.size());

			for (int j = 0; j < rhs; ++j)
			{
				for (int i = m - 1; i > -1; --i)
				{
					if (!b[i][j].empty())x[i][j].assign(b[i][j].begin(), b[i][j].begin() + rhs_size[j] * blk_size[i]);

					for (int k = i + 1; k < m; ++k)
					{
						if (L[i][k].empty() || x[k][j].empty()) { continue; std::cout << "continue" << std::endl; };
						if (x[i][j].empty())x[i][j].resize(blk_size[i] * rhs_size[j], 0);
						s_mma(blk_size[i], rhs_size[j], blk_size[k], -1.0, L[i][k].data(), x[k][j].data(), x[i][j].data());
					}

					if (x[i][j].empty()) x[i][j].clear();
					else s_sov_um(blk_size[i], rhs_size[j], L[i][i].data(), x[i][j].data(), x[i][j].data());
				}
			}
		}

		auto s_blk_householder_ut(const BlockSize &blk_size, const BlockData &A, BlockData &U, BlockData &tau) noexcept->void
		{
			int m = blk_size.size();

			for (int i{ 0 }; i < m; ++i)
			{
				for (int k{ 0 }; k < blk_size[i]; ++k)
				{
					double rho = s_blk_norm_col(blk_size, blk_size, A, i, i, A[i][i].data() + id(k, k, blk_size[i]));
					//tau[id(i, 0, tau_t)] = U[id(i, i, u_t)] / rho - 1.0;
					std::cout << "rho:" << rho << std::endl;
				}
			}
		}



		// R is U //
		auto inline local_U2QR(int m, int n, double *Q, double *R, double *tau)->void 
		{
			// init Q
			double t = tau[0];
			s_mm(m - 1, m - 1, 1, t, R + n, n, R + n, ColMajor{ n }, Q + m + 1, m);
			s_vc(m - 1, t, R + n, n, Q + 1, 1);
			s_vc(m - 1, Q + 1, 1, Q + m, m);
			*Q = t;
			for (int i = 0; i < m; ++i)Q[i*m + i] += 1.0;

			// store tau because we need memory near n
			s_vc(std::min(m, n) - 1, tau + 1, 1, R + n, n);
			// make Q
			for (int i = 1; i < std::min(m - 1, n); ++i)
			{
				s_vc(m, Q + i, m, tau, 1);

				s_mma(m, 1, m - i - 1, Q + i + 1, m, R + (i + 1)*n + i, n, tau, 1);
				s_nv(m, R[i*n], tau);

				s_va(m, tau, 1, Q + i, m);
				s_mma(m, m - i - 1, 1, tau, RowMajor{ 1 }, R + (i + 1)*n + i, ColMajor{ n }, Q + i + 1, RowMajor{ m });
				s_fill(m - i - 1, 1, 0.0, R + (i + 1)*n + i, n);
			}
			s_fill(m - 1, 1, 0.0, R + n, n);
		};
		auto s_householder_qr(int m, int n, const double *A, double *Q, double *R, double *tau)noexcept->void
		{
			//s_householder(m, n, A, R, tau);
			local_U2QR(m, n, Q, R, tau);
		}
		auto s_householder_sov(int m, int n, int rhs, const double *U, const double *tau, double *b, double *x)noexcept->void
		{
			for (int i = 0; i < std::min(m - 1, n); ++i)
			{
				double k = tau[i] * (b[i] + s_vv(m - i - 1, U + (i + 1)*n + i, n, b + i + 1, 1));
				b[i] += k;
				s_va(m - i - 1, k, U + (i + 1)*n + i, n, b + i + 1, 1);
			}

			s_sov_um(std::min(m, n), rhs, U, n, b, rhs, x, rhs);
		}
		auto s_householder_colpiv(int m, int n, const double *A, double *U, double *tau, int *P)noexcept->void
		{
			std::copy(A, A + m*n, U);

			for (int i = 0; i < n; ++i)P[i] = i;

			for (int i = 0; i < std::min(m - 1, n); ++i)
			{
				//////////////////////////////// following is different ///////////////////////
				for (int j = i; j < n; ++j)tau[j] = s_vv(m - i, U + i*n + j, n, U + i*n + j, n);
				int max_col = std::max_element(tau + i, tau + n) - tau;
				s_swap_v(m, U + max_col, n, U + i, n);
				std::swap(P[i], P[max_col]);
				//////////////////////////////// different part finished ///////////////////////
				
				// compute householder vector //
				double rho = -s_norm(m - i, U + i*n + i, n) * s_sgn2(U[i*n + i]);
				tau[i] = U[i*n + i] / rho - 1.0;
				s_nv(m - 1 - i, 1.0 / (U[i*n + i] - rho), U + (i + 1)*n + i, n);
				U[i*n + i] = rho;

				// update matrix //
				s_mc(1, n - i - 1, U + i*n + i + 1, 1, tau + i + 1, 1);
				//s_mmaTN(1, n - i - 1, m - i - 1, U + (i + 1)*n + i, n, U + (i + 1)*n + i + 1, n, tau + i + 1, 1);
				s_mma(1, n - i - 1, m - i - 1, U + (i + 1)*n + i, ColMajor{ n }, U + (i + 1)*n + i + 1, RowMajor{ n }, tau + i + 1, RowMajor{ 1 });
				s_nv(n - i - 1, tau[i], tau + i + 1);

				s_va(n - i - 1, tau + i + 1, U + i*n + i + 1);
				s_mma(m - i - 1, n - i - 1, 1, U + (i + 1)*n + i, n, tau + i + 1, n, U + (i + 1)*n + i + 1, n);
			}
			
			// switch last max element
			if (m < n)
			{
				auto begin = U + (m - 1)*n + m - 1;
				auto end = begin + n - m + 1;
				int max_col = std::max_element(begin, end, [](double a, double b) {return std::abs(a) < std::abs(b); }) - begin;

				s_swap_v(m, U + max_col + m - 1, n, U + m - 1, n);
				std::swap(P[m - 1], P[max_col]);
			}
		}
		auto s_householder_colpiv_qr(int m, int n, const double *A, double *Q, double *R, double *tau, int *P)noexcept->void
		{
			s_householder_colpiv(m, n, A, R, tau, P);
			local_U2QR(m, n, Q, R, tau);
		}
		
		auto dlmwrite(const char *FileName, const double *pMatrix, const int m, const int n)->void
		{
			std::ofstream file;

			file.open(FileName);

			file << std::setprecision(15);

			for (int i = 0; i < m; i++)
			{
				for (int j = 0; j < n; j++)
				{
					file << pMatrix[n*i + j] << "   ";
				}
				file << std::endl;
			}
		}
		auto dlmread(const char *FileName, double *pMatrix)->void
		{
			std::ifstream file;

			file.open(FileName);

			if (!file) throw std::logic_error("file not exist");


			int i = 0;
			while (!file.eof())
			{
				file >> *(pMatrix + i);
				++i;
			}
		}
	}
}
