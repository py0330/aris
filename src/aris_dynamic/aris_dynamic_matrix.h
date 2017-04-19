#ifndef ARIS_DYNAMIC_MATRIX_
#define ARIS_DYNAMIC_MATRIX_

#ifndef PI
#define PI 3.141592653589793
#endif

#include <vector>
#include <map>
#include <string>
#include <memory>
#include <functional>
#include <algorithm>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <list>

#include <aris_core_basic_type.h>

namespace aris
{
	namespace dynamic
	{
		struct RowMajor { Size r_ld; };
		struct ColMajor { Size c_ld; };
		struct Stride { Size r_ld, c_ld; };

		auto inline vid(Size i, Size ld)->Size { return i*ld; }
		auto inline next_vid(Size id, Size ld)->Size { return id + ld; }

		auto inline T(Size ld)->ColMajor { return ColMajor{ ld }; }
		auto inline id(Size i, Size j, Size ld)->Size { return i*ld + j; }
		auto inline next_rid(Size id, Size ld)->Size { return id + ld; }
		auto inline next_cid(Size id, Size ld)->Size { return id + 1; }

		auto inline T(RowMajor r)->ColMajor { return ColMajor{ r.r_ld }; }
		auto inline id(Size i, Size j, RowMajor row_major)->Size { return i*row_major.r_ld + j; }
		auto inline next_rid(Size id, RowMajor row_major)->Size { return id + row_major.r_ld; }
		auto inline next_cid(Size id, RowMajor row_major)->Size { return id + 1; }

		auto inline T(ColMajor c)->RowMajor { return RowMajor{ c.c_ld }; }
		auto inline id(Size i, Size j, ColMajor col_major)->Size { return i + j*col_major.c_ld; }
		auto inline next_rid(Size id, ColMajor col_major)->Size { return id + 1; }
		auto inline next_cid(Size id, ColMajor col_major)->Size { return id + col_major.c_ld; }

		auto inline T(Stride s)->Stride { return Stride{ s.c_ld, s.r_ld }; }
		auto inline id(Size i, Size j, Stride stride)->Size { return i*stride.r_ld + j*stride.c_ld; }
		auto inline next_rid(Size id, Stride stride)->Size { return id + stride.r_ld; }
		auto inline next_cid(Size id, Stride stride)->Size { return id + stride.c_ld; }

		// make vector type from matrix type, just select one row
		auto inline RV(Size ld)->Size { return 1; }
		auto inline RV(RowMajor row_major)->Size { return 1; }
		auto inline RV(ColMajor col_major)->Size { return col_major.c_ld; }
		auto inline RV(Stride stride)->Size { return stride.c_ld; }
		// make vector type from matrix type, just select one col
		auto inline CV(Size ld)->Size { return ld; }
		auto inline CV(RowMajor row_major)->Size { return row_major.r_ld; }
		auto inline CV(ColMajor col_major)->Size { return 1; }
		auto inline CV(Stride stride)->Size { return stride.r_ld; }
		// make matrix type from a row vector
		auto inline RM(Size ld)->ColMajor { return ColMajor{ ld }; }
		// make matrix type from a col vector
		auto inline CM(Size ld)->RowMajor { return RowMajor{ ld }; }


		template <typename T, typename TType>
		auto inline dsp(Size m, Size n, const T *data, TType d_t)->void
		{
			std::cout << std::setiosflags(std::ios::fixed) << std::setiosflags(std::ios::right) << std::setprecision(15);

			std::cout << std::endl;
			for (Size i = 0; i < m; i++)
			{
				for (Size j = 0; j < n; j++)
				{
					std::cout << data[id(i, j, d_t)] << "   ";
				}
				std::cout << std::endl;
			}
			std::cout << std::endl;
		};
		template <typename T>
		auto inline dsp(Size m, Size n, const T *data)->void { dsp(m, n, data, n); }
		template<class Container>
		auto dlmwrite(const char *filename, const Container &container)->void
		{
			std::ofstream file;

			file.open(filename);

			file << std::setprecision(15);

			for (auto i : container)
			{
				for (auto j : i)file << j << "   ";
				file << std::endl;
			}
		}
		auto dlmwrite(const char *filename, const double *mtx, const Size m, const Size n)->void;
		auto dlmread(const char *filename, double *mtx)->void;

		template <typename T>
		auto inline s_sgn(T val)->T { return T(T(0) < val) - (val < T(0)); }
		template <typename T>
		auto inline s_sgn2(T val)->T { return val < T(0) ? T(-1) : T(1); }

		auto inline s_is_equal(double a, double b, double error) { return std::abs(a - b) < error; }
		template <typename V1Type, typename V2Type>
		auto inline s_is_equal(Size n, const double *v1, V1Type v1_t, const double *v2, V2Type v2_t, double error) noexcept->bool
		{
			double diff_square{ 0 };

			for (Size i = 0; i < n; ++i)
			{
				diff_square += (v1[vid(i, v1_t)] - v2[vid(i, v2_t)])*(v1[vid(i, v1_t)] - v2[vid(i, v2_t)]);
			}

			return std::sqrt(std::abs(diff_square)) < error ? true : false;
		}
		auto inline s_is_equal(Size n, const double *v1, const double *v2, double error) noexcept->bool { return s_is_equal(n, v1, 1, v2, 1, error); };
		template <typename M1Type, typename M2Type>
		auto inline s_is_equal(Size m, Size n, const double *m1, M1Type m1_t, const double *m2, M2Type m2_t, double error) noexcept->bool
		{
			double diff_square{ 0 };

			for (Size i = 0; i < m; ++i)for (Size j = 0; j < n; ++j)
			{
				diff_square += (m1[id(i, j, m1_t)] - m2[id(i, j, m2_t)])*(m1[id(i, j, m1_t)] - m2[id(i, j, m2_t)]);
			}

			return std::sqrt(std::abs(diff_square)) > error ? false : true;
		}
		auto inline s_is_equal(Size m, Size n, const double *m1, const double *m2, double error) noexcept->bool { return s_is_equal(m, n, m1, n, m2, n, error); };

		template<typename XType>
		auto inline s_norm(Size n, const double *x, XType x_t) noexcept->double
		{
			double norm = 0;
			for (Size i(-1), x_id{ 0 }; ++i < n; x_id = next_vid(x_id, x_t))norm += x[x_id] * x[x_id];
			return std::sqrt(norm);
		}
		auto inline s_norm(Size n, const double *x) noexcept->double { return s_norm(n, x, 1); }
		template<typename XType>
		auto inline s_norm_col(Size m, const double *x, XType x_t) noexcept->double
		{
			double norm = 0;
			for (Size i(-1), x_id{ 0 }; ++i < m; x_id = next_rid(x_id, x_t))norm += x[x_id] * x[x_id];
			return std::sqrt(norm);
		}
		template<typename XType>
		auto inline s_norm_row(Size n, const double *x, XType x_t) noexcept->double
		{
			double norm = 0;
			for (Size i(-1), x_id{ 0 }; ++i < n; x_id = next_cid(x_id, x_t))norm += x[x_id] * x[x_id];
			return std::sqrt(norm);
		}
		template<typename XType, typename YType>
		auto inline s_swap_v(Size n, double *x, XType x_t, double *y, YType y_t) noexcept->void
		{
			for (Size i(-1), x_id{ 0 }, y_id{ 0 }; ++i < n; x_id = next_vid(x_id, x_t), y_id = next_vid(y_id, y_t))
				std::swap(x[x_id], y[y_id]);
		}
		auto inline s_swap_v(Size n, double *x, double *y) noexcept->void { s_swap_v(n, x, 1, y, 1); }
		template<typename AType>
		auto inline s_fill(Size m, Size n, double value, double *A, AType a_t) noexcept->void
		{
			for (Size i(-1), a_row{ 0 }; ++i < m; a_row = next_rid(a_row, a_t)) for (Size j(-1), a_id{ a_row }; ++j < n; a_id = next_cid(a_id, a_t)) A[a_id] = value;
		}// fill matrix with value
		auto inline s_fill(Size m, Size n, double value, double *A) noexcept->void { std::fill(A, A + m*n, value); }
		template<typename XType>
		auto inline s_nv(Size n, double alpha, double *x, XType x_t) noexcept->void { for (Size i(-1), vid{ 0 }; ++i < n; vid = next_vid(vid, x_t))x[vid] *= alpha; }
		auto inline s_nv(Size n, double alpha, double *x) noexcept->void { for (Size i(-1); ++i < n;)x[i] *= alpha; }
		template<typename XType, typename YType>
		auto inline s_vc(Size n, const double *x, XType x_t, double *y, YType y_t) noexcept->void { for (Size i(-1), x_id{ 0 }, y_id{ 0 }; ++i < n; x_id = next_vid(x_id, x_t), y_id = next_vid(y_id, y_t))y[y_id] = x[x_id]; }
		template<typename XType, typename YType>
		auto inline s_vc(Size n, double alpha, const double *x, XType x_t, double *y, YType y_t) noexcept->void { for (Size i(-1), x_id{ 0 }, y_id{ 0 }; ++i < n; x_id = next_vid(x_id, x_t), y_id = next_vid(y_id, y_t))y[y_id] = alpha*x[x_id]; }
		auto inline s_vc(Size n, const double *x, double *y) noexcept->void { std::copy(x, x + n, y); }
		auto inline s_vc(Size n, double alpha, const double *x, double *y) noexcept->void { for (Size i(-1); ++i < n;)y[i] = alpha*x[i]; }
		template<typename XType, typename YType>
		auto inline s_va(Size n, const double* x, XType x_t, double* y, YType y_t) noexcept->void { for (Size i(-1), x_id{ 0 }, y_id{ 0 }; ++i < n; x_id = next_vid(x_id, x_t), y_id = next_vid(y_id, y_t))y[y_id] += x[x_id]; }
		template<typename XType, typename YType>
		auto inline s_va(Size n, double alpha, const double* x, XType x_t, double* y, YType y_t) noexcept->void { for (Size i(-1), x_id{ 0 }, y_id{ 0 }; ++i < n; x_id = next_vid(x_id, x_t), y_id = next_vid(y_id, y_t))y[y_id] += alpha*x[x_id]; }
		auto inline s_va(Size n, const double* x, double* y) noexcept->void { for (Size i = 0; i < n; ++i)y[i] += x[i]; }
		auto inline s_va(Size n, double alpha, const double* x, double* y) noexcept->void { for (Size i = 0; i < n; ++i)y[i] += alpha * x[i]; }
		template<typename XType, typename YType>
		auto inline s_vv(Size n, const double *x, XType x_t, const double *y, YType y_t) noexcept->double { double ret{ 0 }; for (Size i(-1), x_id{ 0 }, y_id{ 0 }; ++i < n; x_id = next_vid(x_id, x_t), y_id = next_vid(y_id, y_t))ret += x[x_id] * y[y_id]; return ret; }
		auto inline s_vv(Size n, const double *x, const double *y) noexcept->double { double ret{ 0 }; for (Size i = 0; i < n; ++i)ret += x[i] * y[i];	return ret; }
		template<typename AType>
		auto inline s_nm(Size m, Size n, double alpha, double* A, AType a_t) noexcept->void
		{
			for (Size i(-1), a_row{ 0 }; ++i < m; a_row = next_rid(a_row, a_t))for (Size j(-1), a_id{ a_row }; ++j < n; a_id = next_cid(a_id, a_t))	A[a_id] *= alpha;
		}
		auto inline s_nm(Size m, Size n, double alpha, double* A) noexcept->void { s_nv(m*n, alpha, A); }
		template<typename AType, typename BType>
		auto inline s_mc(Size m, Size n, const double *A, AType a_t, double *B, BType b_t) noexcept->void
		{
			for (Size i(-1), row_a{ 0 }, row_b{ 0 }; ++i < m; row_a = next_rid(row_a, a_t), row_b = next_rid(row_b, b_t))
				for (Size j(-1), a_id{ row_a }, b_id{ row_b }; ++j < n; a_id = next_cid(a_id, a_t), b_id = next_cid(b_id, b_t))
					B[b_id] = A[a_id];
		}
		template<typename AType, typename BType>
		auto inline s_mc(Size m, Size n, double alpha, const double *A, AType a_t, double *B, BType b_t) noexcept->void
		{
			for (Size i(-1), row_a{ 0 }, row_b{ 0 }; ++i < m; row_a = next_rid(row_a, a_t), row_b = next_rid(row_b, b_t))
				for (Size j(-1), a_id{ row_a }, b_id{ row_b }; ++j < n; a_id = next_cid(a_id, a_t), b_id = next_cid(b_id, b_t))
					B[b_id] = alpha * A[a_id];
		}
		auto inline s_mc(Size m, Size n, const double *A, double *B) noexcept->void { s_vc(m*n, A, B); }
		auto inline s_mc(Size m, Size n, double alpha, const double *A, double *B) noexcept->void { s_vc(m*n, alpha, A, B); }
		template<typename AType, typename BType>
		auto inline s_ma(Size m, Size n, const double* A, AType a_t, double* B, BType b_t) noexcept->void
		{
			for (Size i(-1), row_a{ 0 }, row_b{ 0 }; ++i < m; row_a = next_rid(row_a, a_t), row_b = next_rid(row_b, b_t))
				for (Size j(-1), a_id{ row_a }, b_id{ row_b }; ++j < n; a_id = next_cid(a_id, a_t), b_id = next_cid(b_id, b_t))
					B[b_id] += A[a_id];
		}
		template<typename AType, typename BType>
		auto inline s_ma(Size m, Size n, double alpha, const double* A, AType a_t, double* B, BType b_t) noexcept->void
		{
			for (Size i(-1), row_a{ 0 }, row_b{ 0 }; ++i < m; row_a = next_rid(row_a, a_t), row_b = next_rid(row_b, b_t))
				for (Size j(-1), a_id{ row_a }, b_id{ row_b }; ++j < n; a_id = next_cid(a_id, a_t), b_id = next_cid(b_id, b_t))
					B[b_id] += alpha*A[a_id];
		}
		auto inline s_ma(Size m, Size n, const double* A, double* B) noexcept->void { s_va(m*n, A, B); }
		auto inline s_ma(Size m, Size n, double alpha, const double* A, double* B) noexcept->void { s_va(m*n, alpha, A, B); }
		template<typename AType, typename BType, typename CType>
		auto inline s_mma(Size m, Size n, Size k, const double* A, AType a_t, const double* B, BType b_t, double *C, CType c_t)->void
		{
			for (Size i(-1), a_row{ 0 }, c_row{ 0 }; ++i < m; a_row = next_rid(a_row, a_t), c_row = next_rid(c_row, c_t))
			{
				for (Size j(-1), b_col{ 0 }, c_id{ c_row }; ++j < n; b_col = next_cid(b_col, b_t), c_id = next_cid(c_id, c_t))
				{
					for (Size u(-1), a_id{ a_row }, b_id{ b_col }; ++u < k; a_id = next_cid(a_id, a_t), b_id = next_rid(b_id, b_t))
						C[c_id] += A[a_id] * B[b_id];
				}
			}
		}
		template<typename AType, typename BType, typename CType>
		auto inline s_mma(Size m, Size n, Size k, double alpha, const double* A, AType a_t, const double* B, BType b_t, double *C, CType c_t)->void
		{
			for (Size i(-1), a_row{ 0 }, c_row{ 0 }; ++i < m; a_row = next_rid(a_row, a_t), c_row = next_rid(c_row, c_t))
			{
				for (Size j(-1), b_col{ 0 }, c_id{ c_row }; ++j < n; b_col = next_cid(b_col, b_t), c_id = next_cid(c_id, c_t))
				{
					double value{ 0 };
					for (Size u(-1), a_id{ a_row }, b_id{ b_col }; ++u < k; a_id = next_cid(a_id, a_t), b_id = next_rid(b_id, b_t))
						value += A[a_id] * B[b_id];
					C[c_id] += alpha*value;
				}
			}
		}
		auto inline s_mma(Size m, Size n, Size k, const double* A, const double* B, double *C) noexcept->void { s_mma(m, n, k, A, k, B, n, C, n); }
		auto inline s_mma(Size m, Size n, Size k, double alpha, const double* A, const double* B, double *C) noexcept->void { s_mma(m, n, k, alpha, A, k, B, n, C, n); }
		template<typename AType, typename BType, typename CType>
		auto inline s_mm(Size m, Size n, Size k, const double* A, AType a_t, const double* B, BType b_t, double *C, CType c_t) noexcept->void
		{
			for (Size i(-1), a_row{ 0 }, c_row{ 0 }; ++i < m; a_row = next_rid(a_row, a_t), c_row = next_rid(c_row, c_t))
			{
				for (Size j(-1), b_col{ 0 }, c_id{ c_row }; ++j < n; b_col = next_cid(b_col, b_t), c_id = next_cid(c_id, c_t))
				{
					C[c_id] = 0.0;
					for (Size u(-1), a_id{ a_row }, b_id{ b_col }; ++u < k; a_id = next_cid(a_id, a_t), b_id = next_rid(b_id, b_t))
						C[c_id] += A[a_id] * B[b_id];
				}
			}
		}
		template<typename AType, typename BType, typename CType>
		auto inline s_mm(Size m, Size n, Size k, double alpha, const double* A, AType a_t, const double* B, BType b_t, double *C, CType c_t) noexcept->void
		{
			for (Size i(-1), a_row{ 0 }, c_row{ 0 }; ++i < m; a_row = next_rid(a_row, a_t), c_row = next_rid(c_row, c_t))
			{
				for (Size j(-1), b_col{ 0 }, c_id{ c_row }; ++j < n; b_col = next_cid(b_col, b_t), c_id = next_cid(c_id, c_t))
				{
					C[c_id] = 0.0;
					for (Size u(-1), a_id{ a_row }, b_id{ b_col }; ++u < k; a_id = next_cid(a_id, a_t), b_id = next_rid(b_id, b_t))
						C[c_id] += A[a_id] * B[b_id];
					C[c_id] *= alpha;
				}
			}
		}
		auto inline s_mm(Size m, Size n, Size k, const double* A, const double* B, double *C) noexcept->void { s_fill(m, n, 0, C); s_mma(m, n, k, A, B, C); }
		auto inline s_mm(Size m, Size n, Size k, double alpha, const double* A, const double* B, double *C) noexcept->void { s_mm(m, n, k, A, B, C); s_nm(m, n, alpha, C); }

		// A can be the same as L, only when they have same type
		template<typename AType, typename LType>
		auto inline s_llt(Size m, const double *A, AType a_t, double *L, LType l_t) noexcept->void
		{
			for (Size j(-1), a_jj{ 0 }, l_jj{ 0 }, l_j0{ 0 }; ++j < m; a_jj = next_rid(next_cid(a_jj, a_t), a_t), l_jj = next_rid(next_cid(l_jj, l_t), l_t), l_j0 = next_rid(l_j0, l_t))
			{
				L[l_jj] = A[a_jj];
				for (Size k(-1), l_jk{ l_j0 }; ++k < j; l_jk = next_cid(l_jk, l_t))
				{
					L[l_jj] -= L[l_jk] * L[l_jk];
				}
				L[l_jj] = std::sqrt(L[l_jj]);


				for (Size i(j), l_i0{ next_rid(l_j0,l_t) }, l_ji{ next_cid(l_jj, l_t) }, l_ij{ next_rid(l_jj, l_t) }, a_ij{ next_rid(a_jj,a_t) }; ++i < m; l_i0 = next_rid(l_i0, l_t), l_ji = next_cid(l_ji, l_t), l_ij = next_rid(l_ij, l_t), a_ij = next_rid(a_ij, a_t))
				{
					L[l_ij] = A[a_ij];
					for (Size k(-1), l_ik{ l_i0 }, l_jk{ l_j0 }; ++k < j; l_ik = next_cid(l_ik, l_t), l_jk = next_cid(l_jk, l_t))
					{
						L[l_ij] -= L[l_ik] * L[l_jk];
					}
					L[l_ij] /= L[l_jj];
					L[l_ji] = L[l_ij];
				}
			}
		}
		auto inline s_llt(Size m, const double *A, double *L) noexcept->void { s_llt(m, A, m, L, m); };
		// L can be the same as inv_L, only when they have same type
		template<typename LType, typename InvLType>
		auto inline s_inv_lm(Size m, const double *L, LType l_t, double *inv_L, InvLType inv_l_t) noexcept->void
		{
			for (Size j(-1), inv_Ljj{ 0 }, Ljj{ 0 }; ++j < m; inv_Ljj = next_rid(next_cid(inv_Ljj, inv_l_t), inv_l_t), Ljj = next_rid(next_cid(Ljj, l_t), l_t))
			{
				inv_L[inv_Ljj] = 1.0/ L[Ljj];
				
				for (Size i(j), inv_Lij{ next_rid(inv_Ljj, inv_l_t) }, inv_Lji{ next_cid(inv_Ljj, inv_l_t) }; ++i < m; inv_Lij = next_rid(inv_Lij, inv_l_t), inv_Lji = next_cid(inv_Lji, inv_l_t))
				{
					double alpha{ 0 };
					for (Size k(j-1), Lik{ id(i, j, l_t) }, inv_Lkj{ id(j, j, inv_l_t) }; ++k < i; Lik = next_cid(Lik, l_t), inv_Lkj = next_rid(inv_Lkj, inv_l_t))
					{
						alpha -= L[Lik] * inv_L[inv_Lkj];
					}
					inv_L[inv_Lij] = alpha/L[id(i, i, l_t)];
					inv_L[inv_Lji] = 0.0;
				}
			}
		}
		auto inline s_inv_lm(Size m, const double *L, double *inv_L) noexcept->void { s_inv_lm(m, L, m, inv_L, m); }
		// U can be the same as inv_U, only when they have same type
		template<typename LType, typename InvLType>
		auto inline s_inv_um(Size m, const double *U, LType l_t, double *inv_U, InvLType inv_l_t) noexcept->void{ s_inv_lm(m, U, T(l_t), inv_U, T(inv_l_t));}
		auto inline s_inv_um(Size m, const double *U, double *inv_U) noexcept->void { s_inv_um(m, U, m, inv_U, m); }
		// b can be the same as x, only when they have same type
		template<typename LType, typename bType, typename xType>
		auto inline s_sov_lm(Size m, Size rhs, const double *L, LType l_t, const double *b, bType b_t, double *x, xType x_t) noexcept->void
		{
			for (Size j(-1); ++j < rhs;)
			{
				for (Size i(-1); ++i < m;)
				{
					x[id(i, j, x_t)] = b[id(i, j, b_t)];

					for (Size k(-1); ++k < i;)
					{
						x[id(i, j, x_t)] -= L[id(i, k, l_t)] * x[id(k, j, x_t)];
					}
					x[id(i, j, x_t)] /= L[id(i, i, l_t)];
				}
			}
		}
		auto inline s_sov_lm(Size m, Size rhs, const double *L, const double *b, double *x) noexcept->void { s_sov_lm(m, rhs, L, m, b, rhs, x, rhs); }
		// b can be the same as x, only when they have same type
		template<typename LType, typename bType, typename xType>
		auto inline s_sov_um(Size m, Size rhs, const double *L, LType l_t, const double *b, bType b_t, double *x, xType x_t) noexcept->void
		{
			for (Size j(-1); ++j < rhs;)
			{
				for (Size i(m); --i < m;)
				{
					x[id(i, j, x_t)] = b[id(i, j, b_t)];

					for (Size k(i); ++k < m;)
					{
						x[id(i, j, x_t)] -= L[id(i, k, l_t)] * x[id(k, j, x_t)];
					}
					x[id(i, j, x_t)] /= L[id(i, i, l_t)];
				}
			}
		}
		auto inline s_sov_um(Size m, Size rhs, const double *L, const double *b, double *x) noexcept->void { s_sov_um(m, rhs, L, m, b, rhs, x, rhs); }

		// tau must have same size with max(m,n), A can be the same as U
		template<typename AType, typename UType, typename TauType>
		auto inline s_householder_ut(Size m, Size n, const double *A, AType a_t, double *U, UType u_t, double *tau, TauType tau_t)noexcept->void
		{
			s_mc(m, n, A, a_t, U, u_t);
			for (Size i(-1); ++i < std::min(m - 1, n);)
			{
				// compute householder vector //
				double rho = -s_norm_col(m - i, U + id(i, i, u_t), u_t) * s_sgn2(U[id(i, i, u_t)]);
				tau[id(i, 0, tau_t)] = U[id(i, i, u_t)] / rho - 1.0;
				s_nv(m - 1 - i, 1.0 / (U[id(i, i, u_t)] - rho), U + id(i + 1, i, u_t), u_t);
				U[id(i, i, u_t)] = rho;

				// update matrix //
				s_mc(1, n - i - 1, U + id(i, i + 1, u_t), u_t, tau + id(i + 1, 0, tau_t), T(tau_t));
				s_mma(1, n - i - 1, m - i - 1, U + id(i + 1, i, u_t), T(u_t), U + id(i + 1, i + 1, u_t), u_t, tau + id(i + 1, 0, tau_t), T(tau_t));
				s_nm(n - i - 1, 1, tau[id(i, 0, tau_t)], tau + id(i + 1, 0, tau_t), tau_t);

				s_ma(n - i - 1, 1, tau + id(i + 1, 0, tau_t), tau_t, U + id(i, i + 1, u_t), T(u_t));
				s_mma(m - i - 1, n - i - 1, 1, U + id(i + 1, i, u_t), u_t, tau + id(i + 1, 0, tau_t), T(tau_t), U + id(i + 1, i + 1, u_t), u_t);
			}
		}
		auto inline s_householder_ut(Size m, Size n, const double *A, double *U, double *tau)noexcept->void { s_householder_ut(m, n, A, n, U, n, tau, 1); }
		// U can be the same as R
		template<typename UType, typename TauType, typename QType, typename RType>
		auto inline s_householder_ut2qr(Size m, Size n, const double *U, UType u_t, const double *tau, TauType tau_t, double *Q, QType q_t, double *R, RType r_t)noexcept->void
		{
			// init R
			s_mc(m, n, U, u_t, R, r_t);

			// init Q
			Q[0] = tau[0];
			s_mm(m - 1, m - 1, 1, tau[0], R + id(1, 0, r_t), r_t, R + id(1, 0, r_t), T(r_t), Q + id(1, 1, q_t), q_t);
			s_mc(m - 1, 1, tau[0], R + id(1, 0, r_t), r_t, Q + id(1, 0, q_t), q_t);
			s_mc(1, m - 1, Q + id(1, 0, q_t), T(q_t), Q + id(0, 1, q_t), q_t);
			for (Size i = 0; i < m; ++i) Q[id(i, i, q_t)] += 1.0;

			// make Q
			double r = R[0];
			for (Size i = 1; i < std::min(m - 1, n); ++i)
			{
				s_mc(m, 1, Q + id(0, i, q_t), q_t, R, r_t);

				s_mma(m, 1, m - i - 1, Q + id(0, i + 1, q_t), q_t, R + id(i + 1, i, r_t), r_t, R, r_t);
				s_nv(m, tau[id(i, 0, tau_t)], R, r_t);

				s_va(m, R, r_t, Q + id(0, i, q_t), q_t);
				s_mma(m, m - i - 1, 1, R, r_t, R + id(i + 1, i, r_t), T(r_t), Q + id(0, i + 1, q_t), q_t);
				s_fill(m - i - 1, 1, 0.0, R + id(i + 1, i, r_t), r_t);
			}
			s_fill(m - 1, 1, 0.0, R + id(1, 0, r_t), r_t);
			R[0] = r;
		}
		auto inline s_householder_ut2qr(Size m, Size n, const double *U, const double *tau, double *Q, double *R) { s_householder_ut2qr(m, n, U, n, tau, 1, Q, m, R, n); }
		// x must have the same or bigger size with b
		template<typename UType, typename TauType, typename BType, typename XType>
		auto inline s_householder_ut_sov(Size m, Size n, Size rhs, const double *U, UType u_t, const double *tau, TauType tau_t, const double *b, BType b_t, double *x, XType x_t)noexcept->void
		{
			s_mc(m, rhs, b, b_t, x, x_t);

			for (Size i = 0; i < std::min(m - 1, n); ++i)
			{
				for (Size j = 0; j < rhs; ++j)
				{
					double k = tau[id(i, 0, tau_t)] * (x[id(i, j, x_t)] + s_vv(m - i - 1, U + id(i + 1, i, u_t), CV(u_t), x + id(i + 1, j, x_t), CV(x_t)));
					x[id(i, j, x_t)] += k;
					s_va(m - i - 1, k, U + id(i + 1, i, u_t), u_t, x + id(i + 1, j, x_t), x_t);
				}
			}

			s_sov_um(std::min(m, n), rhs, U, u_t, x, x_t, x, x_t);

			if (n > m)s_fill(n - m, rhs, 0.0, x + id(m, 0, x_t), x_t);
		}
		auto inline s_householder_ut_sov(Size m, Size n, Size rhs, const double *U, const double *tau, const double *b, double *x) { s_householder_ut_sov(m, n, rhs, U, n, tau, 1, b, rhs, x, rhs); }
	}
}




























#endif
