#ifndef ARIS_DYNAMIC_MATRIX_
#define ARIS_DYNAMIC_MATRIX_

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
#include <cmath>

#include <aris_core_basic_type.h>

namespace aris
{
	namespace dynamic
	{
		struct RowMajor { Size r_ld; constexpr RowMajor(Size r_ld_) :r_ld(r_ld_) {}; };
		struct ColMajor { Size c_ld; constexpr ColMajor(Size c_ld_) :c_ld(c_ld_) {}; };
		struct Stride { Size r_ld, c_ld; Stride() = default; constexpr Stride(Size r_ld_, Size c_ld_) :r_ld(r_ld_), c_ld(c_ld_) {}; };

		auto inline constexpr T(Size ld)->ColMajor { return ColMajor(ld); }
		auto inline constexpr id(Size i, Size ld)->Size { return i*ld; }
		auto inline constexpr id(Size i, Size j, Size ld)->Size { return i*ld + j; }
		auto inline constexpr next_rid(Size id, Size ld)->Size { return id + ld; }
		auto inline constexpr last_rid(Size id, Size ld)->Size { return id - ld; }
		auto inline constexpr next_cid(Size id, Size ld)->Size { return id + 1; }
		auto inline constexpr last_cid(Size id, Size ld)->Size { return id - 1; }
		auto inline constexpr next_did(Size id, Size ld)->Size { return id + 1 + ld; }
		auto inline constexpr last_did(Size id, Size ld)->Size { return id - 1 - ld; }

		auto inline constexpr T(RowMajor r)->ColMajor { return ColMajor(r.r_ld); }
		auto inline constexpr id(Size i, RowMajor row_major)->Size { return i*row_major.r_ld; }
		auto inline constexpr id(Size i, Size j, RowMajor row_major)->Size { return i*row_major.r_ld + j; }
		auto inline constexpr next_rid(Size id, RowMajor row_major)->Size { return id + row_major.r_ld; }
		auto inline constexpr last_rid(Size id, RowMajor row_major)->Size { return id - row_major.r_ld; }
		auto inline constexpr next_cid(Size id, RowMajor row_major)->Size { return id + 1; }
		auto inline constexpr last_cid(Size id, RowMajor row_major)->Size { return id - 1; }
		auto inline constexpr next_did(Size id, RowMajor row_major)->Size { return id + 1 + row_major.r_ld; }
		auto inline constexpr last_did(Size id, RowMajor row_major)->Size { return id - 1 - row_major.r_ld; }

		auto inline constexpr T(ColMajor c)->RowMajor { return RowMajor(c.c_ld); }
		auto inline constexpr id(Size i, ColMajor col_major)->Size { return i; }
		auto inline constexpr id(Size i, Size j, ColMajor col_major)->Size { return i + j*col_major.c_ld; }
		auto inline constexpr next_rid(Size id, ColMajor col_major)->Size { return id + 1; }
		auto inline constexpr last_rid(Size id, ColMajor col_major)->Size { return id - 1; }
		auto inline constexpr next_cid(Size id, ColMajor col_major)->Size { return id + col_major.c_ld; }
		auto inline constexpr last_cid(Size id, ColMajor col_major)->Size { return id - col_major.c_ld; }
		auto inline constexpr next_did(Size id, ColMajor col_major)->Size { return id + 1 + col_major.c_ld; }
		auto inline constexpr last_did(Size id, ColMajor col_major)->Size { return id - 1 - col_major.c_ld; }

		auto inline constexpr T(Stride s)->Stride { return Stride(s.c_ld, s.r_ld); }
		auto inline constexpr id(Size i, Stride stride)->Size { return i*stride.r_ld; }
		auto inline constexpr id(Size i, Size j, Stride stride)->Size { return i*stride.r_ld + j*stride.c_ld; }
		auto inline constexpr next_rid(Size id, Stride stride)->Size { return id + stride.r_ld; }
		auto inline constexpr last_rid(Size id, Stride stride)->Size { return id - stride.r_ld; }
		auto inline constexpr next_cid(Size id, Stride stride)->Size { return id + stride.c_ld; }
		auto inline constexpr last_cid(Size id, Stride stride)->Size { return id - stride.c_ld; }
		auto inline constexpr next_did(Size id, Stride stride)->Size { return id + stride.c_ld + stride.r_ld; }
		auto inline constexpr last_did(Size id, Stride stride)->Size { return id - stride.c_ld - stride.r_ld; }

		template <typename T, typename TType>
		auto inline dsp(Size m, Size n, const T *data, TType d_t)->void
		{
			std::cout << std::setiosflags(std::ios::fixed) << std::setiosflags(std::ios::right) << std::setprecision(16);

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
		template<typename AType>
		auto dlmwrite(const Size m, const Size n, const double *A, AType a_t, const char *filename)->void
		{
			std::ofstream file;

			file.open(filename);

			file << std::setprecision(15);

			for (Size i(-1); ++i < m;)
			{
				for (Size j(-1); ++j < n;)
				{
					file << A[id(i, j, a_t)] << "   ";
				}
				file << std::endl;
			}
		}
		auto inline dlmwrite(const Size m, const Size n, const double *A, const char *filename)->void { dlmwrite(m, n, A, n, filename); }
		auto dlmread(const char *filename, double *mtx)->void;

		template <typename T>
		auto inline s_sgn(T val)->T { return T(T(0) < val) - (val < T(0)); }
		template <typename T>
		auto inline s_sgn2(T val)->T { return val < T(0) ? T(-1) : T(1); }

		auto inline s_is_equal(double a, double b, double error) { return std::abs(a - b) < error; }
		template <typename V1Type, typename V2Type>
		auto inline s_is_equal(Size n, const double *v1, V1Type v1_t, const double *v2, V2Type v2_t, double error) noexcept->bool
		{
			for (Size i = 0; i < n; ++i)if (!s_is_equal(v1[id(i, v1_t)], v2[id(i, v2_t)], error))return false;
			return true;
		}
		auto inline s_is_equal(Size n, const double *v1, const double *v2, double error) noexcept->bool { return s_is_equal(n, v1, 1, v2, 1, error); };
		template <typename M1Type, typename M2Type>
		auto inline s_is_equal(Size m, Size n, const double *m1, M1Type m1_t, const double *m2, M2Type m2_t, double error) noexcept->bool
		{
			for (Size i = 0; i < m; ++i)for (Size j = 0; j < n; ++j)if (!s_is_equal(m1[id(i, j, m1_t)], m2[id(i, j, m2_t)], error)) return false;
			return true;
		}
		auto inline s_is_equal(Size m, Size n, const double *m1, const double *m2, double error) noexcept->bool { return s_is_equal(m, n, m1, n, m2, n, error); };

		template<typename XType>
		auto inline s_norm(Size n, const double *x, XType x_t) noexcept->double
		{
			double norm = 0;
			for (Size i(-1), x_id{ 0 }; ++i < n; x_id = next_rid(x_id, x_t))norm += x[x_id] * x[x_id];
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
			for (Size i(-1), x_id{ 0 }, y_id{ 0 }; ++i < n; x_id = next_rid(x_id, x_t), y_id = next_rid(y_id, y_t))
				std::swap(x[x_id], y[y_id]);
		}
		auto inline s_swap_v(Size n, double *x, double *y) noexcept->void { s_swap_v(n, x, 1, y, 1); }
		template<typename AType, typename BType>
		auto inline s_swap_m(Size m, Size n, double *a, AType a_t, double *b, BType b_t) noexcept->void
		{
			for (Size i(-1), ai0{ 0 }, bi0{ 0 }; ++i < m; ai0 = next_rid(ai0, a_t), bi0 = next_rid(bi0, b_t))
			{
				for (Size j(-1), aij{ ai0 }, bij{ bi0 }; ++j < n; aij = next_cid(aij, a_t), bij = next_cid(bij, b_t))
				{
					std::swap(a[aij], b[bij]);
				}
			}
		}
		auto inline s_swap_m(Size m, Size n, double *a, double *b) noexcept->void { s_swap_m(m, n, a, n, b, n); }
		template<typename AType>
		auto inline s_fill(Size m, Size n, double value, double *A, AType a_t) noexcept->void
		{
			for (Size i(-1), a_row{ 0 }; ++i < m; a_row = next_rid(a_row, a_t)) for (Size j(-1), a_id{ a_row }; ++j < n; a_id = next_cid(a_id, a_t)) A[a_id] = value;
		}// fill matrix with value
		auto inline s_fill(Size m, Size n, double value, double *A) noexcept->void { std::fill(A, A + m*n, value); }
		template<typename XType>
		auto inline s_iv(Size n, double *x, XType x_t) noexcept->void { for (Size i(-1), vid{ 0 }; ++i < n; vid = next_rid(vid, x_t))x[vid] = -x[vid]; }
		auto inline s_iv(Size n, double *x) noexcept->void { for (Size i(-1); ++i < n;)x[i] = -x[i]; }
		template<typename XType>
		auto inline s_nv(Size n, double alpha, double *x, XType x_t) noexcept->void { for (Size i(-1), vid{ 0 }; ++i < n; vid = next_rid(vid, x_t))x[vid] *= alpha; }
		auto inline s_nv(Size n, double alpha, double *x) noexcept->void { for (Size i(-1); ++i < n;)x[i] *= alpha; }
		template<typename XType, typename YType>
		auto inline s_vc(Size n, const double *x, XType x_t, double *y, YType y_t) noexcept->void { for (Size i(-1), x_id{ 0 }, y_id{ 0 }; ++i < n; x_id = next_rid(x_id, x_t), y_id = next_rid(y_id, y_t))y[y_id] = x[x_id]; }
		template<typename XType, typename YType>
		auto inline s_vc(Size n, double alpha, const double *x, XType x_t, double *y, YType y_t) noexcept->void { for (Size i(-1), x_id{ 0 }, y_id{ 0 }; ++i < n; x_id = next_rid(x_id, x_t), y_id = next_rid(y_id, y_t))y[y_id] = alpha*x[x_id]; }
		auto inline s_vc(Size n, const double *x, double *y) noexcept->void { std::copy(x, x + n, y); }
		auto inline s_vc(Size n, double alpha, const double *x, double *y) noexcept->void { for (Size i(-1); ++i < n;)y[i] = alpha*x[i]; }
		template<typename XType, typename YType>
		auto inline s_vi(Size n, const double *x, XType x_t, double *y, YType y_t) noexcept->void { for (Size i(-1), x_id{ 0 }, y_id{ 0 }; ++i < n; x_id = next_rid(x_id, x_t), y_id = next_rid(y_id, y_t))y[y_id] = -x[x_id]; }
		auto inline s_vi(Size n, const double *x, double *y) noexcept->void { for (Size i(-1); ++i < n;)y[i] = -x[i]; }
		template<typename XType, typename YType>
		auto inline s_va(Size n, const double* x, XType x_t, double* y, YType y_t) noexcept->void { for (Size i(-1), x_id{ 0 }, y_id{ 0 }; ++i < n; x_id = next_rid(x_id, x_t), y_id = next_rid(y_id, y_t))y[y_id] += x[x_id]; }
		template<typename XType, typename YType>
		auto inline s_va(Size n, double alpha, const double* x, XType x_t, double* y, YType y_t) noexcept->void { for (Size i(-1), x_id{ 0 }, y_id{ 0 }; ++i < n; x_id = next_rid(x_id, x_t), y_id = next_rid(y_id, y_t))y[y_id] += alpha*x[x_id]; }
		auto inline s_va(Size n, const double* x, double* y) noexcept->void { for (Size i = 0; i < n; ++i)y[i] += x[i]; }
		auto inline s_va(Size n, double alpha, const double* x, double* y) noexcept->void { for (Size i = 0; i < n; ++i)y[i] += alpha * x[i]; }
		template<typename XType, typename YType>
		auto inline s_vs(Size n, const double* x, XType x_t, double* y, YType y_t) noexcept->void { for (Size i(-1), x_id{ 0 }, y_id{ 0 }; ++i < n; x_id = next_rid(x_id, x_t), y_id = next_rid(y_id, y_t))y[y_id] -= x[x_id]; }
		auto inline s_vs(Size n, const double* x, double* y) noexcept->void { for (Size i = 0; i < n; ++i)y[i] -= x[i]; }
		template<typename XType, typename YType>
		auto inline s_vv(Size n, const double *x, XType x_t, const double *y, YType y_t) noexcept->double { double ret{ 0 }; for (Size i(-1), x_id{ 0 }, y_id{ 0 }; ++i < n; x_id = next_rid(x_id, x_t), y_id = next_rid(y_id, y_t))ret += x[x_id] * y[y_id]; return ret; }
		auto inline s_vv(Size n, const double *x, const double *y) noexcept->double { double ret{ 0 }; for (Size i = 0; i < n; ++i)ret += x[i] * y[i];	return ret; }
		template<typename AType>
		auto inline s_nm(Size m, Size n, double alpha, double* A, AType a_t) noexcept->void
		{
			for (Size i(-1), ai0{ 0 }; ++i < m; ai0 = next_rid(ai0, a_t))
				for (Size j(-1), aij{ ai0 }; ++j < n; aij = next_cid(aij, a_t))
					A[aij] *= alpha;
		}
		auto inline s_nm(Size m, Size n, double alpha, double* A) noexcept->void { s_nv(m*n, alpha, A); }
		template<typename AType>
		auto inline s_im(Size m, Size n, double* A, AType a_t) noexcept->void
		{
			for (Size i(-1), ai0{ 0 }; ++i < m; ai0 = next_rid(ai0, a_t))
				for (Size j(-1), aij{ ai0 }; ++j < n; aij = next_cid(aij, a_t))
					A[aij] = -A[aij];
		}
		auto inline s_im(Size m, Size n, double* A) noexcept->void { s_iv(m*n, A); }
		template<typename AType, typename BType>
		auto inline s_mc(Size m, Size n, const double *A, AType a_t, double *B, BType b_t) noexcept->void
		{
			for (Size i(-1), ai0{ 0 }, bi0{ 0 }; ++i < m; ai0 = next_rid(ai0, a_t), bi0 = next_rid(bi0, b_t))
				for (Size j(-1), aij{ ai0 }, bij{ bi0 }; ++j < n; aij = next_cid(aij, a_t), bij = next_cid(bij, b_t))
					B[bij] = A[aij];
		}
		template<typename AType, typename BType>
		auto inline s_mc(Size m, Size n, double alpha, const double *A, AType a_t, double *B, BType b_t) noexcept->void
		{
			for (Size i(-1), ai0{ 0 }, bi0{ 0 }; ++i < m; ai0 = next_rid(ai0, a_t), bi0 = next_rid(bi0, b_t))
				for (Size j(-1), aij{ ai0 }, bij{ bi0 }; ++j < n; aij = next_cid(aij, a_t), bij = next_cid(bij, b_t))
					B[bij] = alpha * A[aij];
		}
		auto inline s_mc(Size m, Size n, const double *A, double *B) noexcept->void { s_vc(m*n, A, B); }
		auto inline s_mc(Size m, Size n, double alpha, const double *A, double *B) noexcept->void { s_vc(m*n, alpha, A, B); }
		template<typename AType, typename BType>
		auto inline s_ma(Size m, Size n, const double* A, AType a_t, double* B, BType b_t) noexcept->void
		{
			for (Size i(-1), ai0{ 0 }, bi0{ 0 }; ++i < m; ai0 = next_rid(ai0, a_t), bi0 = next_rid(bi0, b_t))
				for (Size j(-1), aij{ ai0 }, bij{ bi0 }; ++j < n; aij = next_cid(aij, a_t), bij = next_cid(bij, b_t))
					B[bij] += A[aij];
		}
		template<typename AType, typename BType>
		auto inline s_ma(Size m, Size n, double alpha, const double* A, AType a_t, double* B, BType b_t) noexcept->void
		{
			for (Size i(-1), ai0{ 0 }, bi0{ 0 }; ++i < m; ai0 = next_rid(ai0, a_t), bi0 = next_rid(bi0, b_t))
				for (Size j(-1), aij{ ai0 }, bij{ bi0 }; ++j < n; aij = next_cid(aij, a_t), bij = next_cid(bij, b_t))
					B[bij] += alpha*A[aij];
		}
		auto inline s_ma(Size m, Size n, const double* A, double* B) noexcept->void { s_va(m*n, A, B); }
		auto inline s_ma(Size m, Size n, double alpha, const double* A, double* B) noexcept->void { s_va(m*n, alpha, A, B); }
		template<typename AType, typename BType>
		auto inline s_mi(Size m, Size n, const double* A, AType a_t, double* B, BType b_t) noexcept->void
		{
			for (Size i(-1), ai0{ 0 }, bi0{ 0 }; ++i < m; ai0 = next_rid(ai0, a_t), bi0 = next_rid(bi0, b_t))
				for (Size j(-1), aij{ ai0 }, bij{ bi0 }; ++j < n; aij = next_cid(aij, a_t), bij = next_cid(bij, b_t))
					B[bij] = -A[aij];
		}
		auto inline s_mi(Size m, Size n, const double* A, double* B) noexcept->void { s_vi(m*n, A, B); }
		template<typename AType, typename BType>
		auto inline s_ms(Size m, Size n, const double* A, AType a_t, double* B, BType b_t) noexcept->void
		{
			for (Size i(-1), ai0{ 0 }, bi0{ 0 }; ++i < m; ai0 = next_rid(ai0, a_t), bi0 = next_rid(bi0, b_t))
				for (Size j(-1), aij{ ai0 }, bij{ bi0 }; ++j < n; aij = next_cid(aij, a_t), bij = next_cid(bij, b_t))
					B[bij] -= A[aij];
		}
		auto inline s_ms(Size m, Size n, const double* A, double* B) noexcept->void { s_vs(m*n, A, B); }
		template<typename AType, typename BType, typename CType>
		auto inline s_mma(Size m, Size n, Size k, const double* A, AType a_t, const double* B, BType b_t, double *C, CType c_t)->void
		{
			for (Size i(-1), ai0{ 0 }, ci0{ 0 }; ++i < m; ai0 = next_rid(ai0, a_t), ci0 = next_rid(ci0, c_t))
			{
				for (Size j(-1), b0j{ 0 }, cij{ ci0 }; ++j < n; b0j = next_cid(b0j, b_t), cij = next_cid(cij, c_t))
				{
					for (Size u(-1), aiu{ ai0 }, buj{ b0j }; ++u < k; aiu = next_cid(aiu, a_t), buj = next_rid(buj, b_t))
						C[cij] += A[aiu] * B[buj];
				}
			}
		}
		template<typename AType, typename BType, typename CType>
		auto inline s_mma(Size m, Size n, Size k, double alpha, const double* A, AType a_t, const double* B, BType b_t, double *C, CType c_t)->void
		{
			for (Size i(-1), ai0{ 0 }, ci0{ 0 }; ++i < m; ai0 = next_rid(ai0, a_t), ci0 = next_rid(ci0, c_t))
			{
				for (Size j(-1), b0j{ 0 }, cij{ ci0 }; ++j < n; b0j = next_cid(b0j, b_t), cij = next_cid(cij, c_t))
				{
					double value{ 0 };
					for (Size u(-1), aiu{ ai0 }, buj{ b0j }; ++u < k; aiu = next_cid(aiu, a_t), buj = next_rid(buj, b_t))
						value += A[aiu] * B[buj];
					C[cij] += alpha*value;
				}
			}
		}
		auto inline s_mma(Size m, Size n, Size k, const double* A, const double* B, double *C) noexcept->void { s_mma(m, n, k, A, k, B, n, C, n); }
		auto inline s_mma(Size m, Size n, Size k, double alpha, const double* A, const double* B, double *C) noexcept->void { s_mma(m, n, k, alpha, A, k, B, n, C, n); }
		template<typename AType, typename BType, typename CType>
		auto inline s_mms(Size m, Size n, Size k, const double* A, AType a_t, const double* B, BType b_t, double *C, CType c_t)->void
		{
			for (Size i(-1), ai0{ 0 }, ci0{ 0 }; ++i < m; ai0 = next_rid(ai0, a_t), ci0 = next_rid(ci0, c_t))
			{
				for (Size j(-1), b0j{ 0 }, cij{ ci0 }; ++j < n; b0j = next_cid(b0j, b_t), cij = next_cid(cij, c_t))
				{
					for (Size u(-1), aiu{ ai0 }, buj{ b0j }; ++u < k; aiu = next_cid(aiu, a_t), buj = next_rid(buj, b_t))
						C[cij] -= A[aiu] * B[buj];
				}
			}
		}
		auto inline s_mms(Size m, Size n, Size k, const double* A, const double* B, double *C) noexcept->void { s_mms(m, n, k, A, k, B, n, C, n); }
		template<typename AType, typename BType, typename CType>
		auto inline s_mm(Size m, Size n, Size k, const double* A, AType a_t, const double* B, BType b_t, double *C, CType c_t) noexcept->void
		{
			for (Size i(-1), ai0{ 0 }, ci0{ 0 }; ++i < m; ai0 = next_rid(ai0, a_t), ci0 = next_rid(ci0, c_t))
			{
				for (Size j(-1), b0j{ 0 }, cij{ ci0 }; ++j < n; b0j = next_cid(b0j, b_t), cij = next_cid(cij, c_t))
				{
					C[cij] = 0.0;
					for (Size u(-1), aiu{ ai0 }, buj{ b0j }; ++u < k; aiu = next_cid(aiu, a_t), buj = next_rid(buj, b_t))
						C[cij] += A[aiu] * B[buj];
				}
			}
		}
		template<typename AType, typename BType, typename CType>
		auto inline s_mm(Size m, Size n, Size k, double alpha, const double* A, AType a_t, const double* B, BType b_t, double *C, CType c_t) noexcept->void
		{
			for (Size i(-1), ai0{ 0 }, ci0{ 0 }; ++i < m; ai0 = next_rid(ai0, a_t), ci0 = next_rid(ci0, c_t))
			{
				for (Size j(-1), b0j{ 0 }, cij{ ci0 }; ++j < n; b0j = next_cid(b0j, b_t), cij = next_cid(cij, c_t))
				{
					C[cij] = 0.0;
					for (Size u(-1), aiu{ ai0 }, buj{ b0j }; ++u < k; aiu = next_cid(aiu, a_t), buj = next_rid(buj, b_t))
						C[cij] += A[aiu] * B[buj];
					C[cij] *= alpha;
				}
			}
		}
		auto inline s_mm(Size m, Size n, Size k, const double* A, const double* B, double *C) noexcept->void { s_fill(m, n, 0, C); s_mma(m, n, k, A, B, C); }
		auto inline s_mm(Size m, Size n, Size k, double alpha, const double* A, const double* B, double *C) noexcept->void { s_mm(m, n, k, A, B, C); s_nm(m, n, alpha, C); }
		template<typename AType, typename BType, typename CType>
		auto inline s_mmi(Size m, Size n, Size k, const double* A, AType a_t, const double* B, BType b_t, double *C, CType c_t) noexcept->void
		{
			for (Size i(-1), ai0{ 0 }, ci0{ 0 }; ++i < m; ai0 = next_rid(ai0, a_t), ci0 = next_rid(ci0, c_t))
			{
				for (Size j(-1), b0j{ 0 }, cij{ ci0 }; ++j < n; b0j = next_cid(b0j, b_t), cij = next_cid(cij, c_t))
				{
					C[cij] = 0.0;
					for (Size u(-1), aiu{ ai0 }, buj{ b0j }; ++u < k; aiu = next_cid(aiu, a_t), buj = next_rid(buj, b_t))
						C[cij] -= A[aiu] * B[buj];
				}
			}
		}
		auto inline s_mmi(Size m, Size n, Size k, const double* A, const double* B, double *C) noexcept->void { s_fill(m, n, 0, C); s_mms(m, n, k, A, B, C); }

		// x_new[i] = x[p[i]]
		template<typename XType>
		auto inline s_permutate(Size m, Size rhs, const Size *p, double *x, XType x_t)->void
		{
			for (Size i(-1), xi0{ 0 }; ++i < m; xi0 = next_rid(xi0, x_t))
			{
				Size k{p[i]};
				for (; k < i; k = p[k]);
				
				for (Size j(-1), xij{ xi0 }, xkj{ id(k,0,x_t) }; ++j < rhs; xij = next_cid(xij, x_t), xkj = next_cid(xkj, x_t))std::swap(x[xij], x[xkj]);
			}
		}
		auto inline s_permutate(Size m, Size rhs, const Size *p, double *x)->void{s_permutate(m, rhs, p, x, 1);}
		// x_new[p[i]] = x[i]
		template<typename XType>
		auto inline s_permutate_inv(Size m, Size rhs, const Size *p, double *x, XType x_t)->void
		{
			for (Size i(-1), xi0{ 0 }; ++i < m; xi0 = next_rid(xi0, x_t))
			{
				Size check_k{ p[i] };
				for (; check_k > i; check_k = p[check_k]);

				if (check_k == i)
				{
					for (Size k{ p[i] }; k > i; k = p[k])
					{
						for (Size j(-1), xij{ xi0 }, xkj{ id(k,0,x_t) }; ++j < rhs; xij = next_cid(xij, x_t), xkj = next_cid(xkj, x_t))
							std::swap(x[xij], x[xkj]);
					}
				}
			}
		}
		auto inline s_permutate_inv(Size m, Size rhs, const Size *p, double *x)->void { s_permutate_inv(m, rhs, p, x, 1); }

		// A can be the same as L, only when they have same type
		template<typename AType, typename LType>
		auto inline s_llt(Size m, const double *A, AType a_t, double *L, LType l_t) noexcept->void
		{
			for (Size j(-1), ajj{ 0 }, ljj{ 0 }, lj0{ 0 }; ++j < m; ajj = next_did(ajj, a_t), ljj = next_did(ljj, l_t), lj0 = next_rid(lj0, l_t))
			{
				L[ljj] = A[ajj];
				for (Size k(-1), ljk{ lj0 }; ++k < j; ljk = next_cid(ljk, l_t))
				{
					L[ljj] -= L[ljk] * L[ljk];
				}
				L[ljj] = std::sqrt(L[ljj]);


				for (Size i(j), li0{ next_rid(lj0,l_t) }, lji{ next_cid(ljj, l_t) }, lij{ next_rid(ljj, l_t) }, a_ij{ next_rid(ajj,a_t) }; ++i < m; li0 = next_rid(li0, l_t), lji = next_cid(lji, l_t), lij = next_rid(lij, l_t), a_ij = next_rid(a_ij, a_t))
				{
					L[lij] = A[a_ij];
					for (Size k(-1), l_ik{ li0 }, ljk{ lj0 }; ++k < j; l_ik = next_cid(l_ik, l_t), ljk = next_cid(ljk, l_t))
					{
						L[lij] -= L[l_ik] * L[ljk];
					}
					L[lij] /= L[ljj];
					L[lji] = L[lij];
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
		auto inline s_sov_lm(Size m, Size rhs, const double *L, LType l_t, const double *b, bType b_t, double *x, xType x_t, double zero_check = 1e-10) noexcept->void
		{
			for (Size j(-1), x0j{ 0 }, b0j{ 0 }; ++j < rhs; x0j = next_cid(x0j, x_t), b0j = next_cid(b0j, b_t))
			{
				for (Size i(-1), xij{ x0j }, bij{ b0j }, li0{ 0 }, lii{ 0 }; ++i < m; xij = next_rid(xij, x_t), bij = next_rid(bij, b_t), li0 = next_rid(li0, l_t), lii = next_did(lii, l_t))
				{
					x[xij] = b[bij];

					for (Size k(-1), lik{ li0 }, xkj{ x0j }; ++k < i; lik = next_cid(lik, l_t), xkj = next_rid(xkj, x_t))
					{
						x[xij] -= L[lik] * x[xkj];
					}
					x[xij] = std::abs(L[lii]) > zero_check ? x[xij] / L[lii] : 0.0;
				}
			}
		}
		auto inline s_sov_lm(Size m, Size rhs, const double *L, const double *b, double *x, double zero_check = 1e-10) noexcept->void { s_sov_lm(m, rhs, L, m, b, rhs, x, rhs, zero_check); }
		// b can be the same as x, only when they have same type
		template<typename LType, typename bType, typename xType>
		auto inline s_sov_um(Size m, Size rhs, const double *L, LType l_t, const double *b, bType b_t, double *x, xType x_t, double zero_check = 1e-10) noexcept->void
		{
			for (Size j(-1), xmj{ id(m - 1, 0, x_t) }, bmj{ id(m - 1, 0, b_t) }; ++j < rhs; xmj = next_cid(xmj, x_t), bmj = next_cid(bmj, b_t))
			{
				for (Size i(m), xij{ xmj }, bij{ bmj }, lii{ id(m - 1, m - 1, l_t) }; --i < m; xij = last_rid(xij, x_t), bij = last_rid(bij, b_t), lii = last_did(lii, l_t))
				{
					x[xij] = b[bij];

					for (Size k(i), lik{ next_cid(lii, l_t) }, xkj{ next_rid(xij, x_t) }; ++k < m; lik = next_cid(lik, l_t), xkj = next_rid(xkj, x_t))
					{
						x[xij] -= L[lik] * x[xkj];
					}
					x[xij] = std::abs(L[lii]) > zero_check ? x[xij] / L[lii] : 0.0;
				}
			}
		}
		auto inline s_sov_um(Size m, Size rhs, const double *L, const double *b, double *x, double zero_check = 1e-10) noexcept->void { s_sov_um(m, rhs, L, m, b, rhs, x, rhs, zero_check); }

		// tau must have same size with max(m,n), A can be the same as U
		template<typename AType, typename UType, typename TauType>
		auto inline s_householder_ut(Size m, Size n, const double *A, AType a_t, double *U, UType u_t, double *tau, TauType tau_t, double zero_check = 1e-10)noexcept->void
		{
			s_mc(m, n, A, a_t, U, u_t);
			// 这里防止 m - 1 变成 -1（既最大）
			for (Size i(-1), uii{ 0 }, ti0{ 0 }; ++i < std::min(std::min(m - 1, n), m); uii = next_did(uii, u_t), ti0 = next_rid(ti0, tau_t))
			{
				// compute householder vector //
				double rho = -s_norm_col(m - i, U + uii, u_t) * s_sgn2(U[uii]);
				if (std::abs(rho) > zero_check)
				{
					auto U_i_i1 = U + next_cid(uii, u_t);
					auto U_i1_i = U + next_rid(uii, u_t);
					auto U_i1_i1 = U + next_did(uii, u_t);
					auto ti1 = tau + next_rid(ti0, tau_t);
					
					tau[ti0] = U[uii] / rho - 1.0;
					s_nm(m - 1 - i, 1, 1.0 / (U[uii] - rho), U_i1_i, u_t);
					U[uii] = rho;

					// update matrix //
					s_mc(1, n - i - 1, U_i_i1, u_t, ti1, T(tau_t));
					s_mma(1, n - i - 1, m - i - 1, U_i1_i, T(u_t), U_i1_i1, u_t, ti1, T(tau_t));
					s_nm(n - i - 1, 1, tau[ti0], ti1, tau_t);

					s_ma(n - i - 1, 1, ti1, tau_t, U_i_i1, T(u_t));
					s_mma(m - i - 1, n - i - 1, 1, U_i1_i, u_t, ti1, T(tau_t), U_i1_i1, u_t);
				}
				else
				{
					tau[ti0] = 0.0;
				}
				

			}
		}
		auto inline s_householder_ut(Size m, Size n, const double *A, double *U, double *tau, double zero_check = 1e-10)noexcept->void { s_householder_ut(m, n, A, n, U, n, tau, 1, zero_check); }
		// U can be the same as R
		template<typename UType, typename TauType, typename QType, typename RType>
		auto inline s_householder_ut2qr(Size m, Size n, const double *U, UType u_t, const double *tau, TauType tau_t, double *Q, QType q_t, double *R, RType r_t)noexcept->void
		{
			// init R
			s_mc(m, n, U, u_t, R, r_t);

			// init Q
			auto R10 = R + next_rid(0, r_t);
			auto Q11 = Q + next_did(0, q_t);
			auto Q10 = Q + next_rid(0, q_t);
			auto Q01 = Q + next_cid(0, q_t);


			Size m_minus_one = std::max(Size(1), m) - 1;
			Q[0] = tau[0];
			s_mm(m_minus_one, m_minus_one, 1, tau[0], R10, r_t, R10, T(r_t), Q11, q_t);
			s_mc(m_minus_one, 1, tau[0], R10, r_t, Q10, q_t);
			s_mc(1, m_minus_one, Q10, T(q_t), Q01, q_t);
			for (Size i = 0; i < m; ++i) Q[id(i, i, q_t)] += 1.0;

			// make Q
			double r = R[0];
			for (Size i(0), q0i{ next_cid(0, q_t) }, rii{ next_did(0, r_t) }, ti0{ next_rid(0, tau_t) }; ++i < std::min(m_minus_one, n); q0i = next_cid(q0i, q_t), rii = next_did(rii, r_t), ti0 = next_rid(ti0, tau_t))
			{
				auto Ri1i = R + next_rid(rii, r_t);
				auto Qii1 = Q + next_cid(q0i, q_t);
				
				s_mc(m, 1, Q + q0i, q_t, R, r_t);

				s_mma(m, 1, m - i - 1, Qii1, q_t, Ri1i, r_t, R, r_t);
				s_nv(m, tau[ti0], R, r_t);

				s_va(m, R, r_t, Q + q0i, q_t);
				s_mma(m, m - i - 1, 1, R, r_t, Ri1i, T(r_t), Qii1, q_t);
				s_fill(m - i - 1, 1, 0.0, Ri1i, r_t);
			}
			s_fill(m_minus_one, 1, 0.0, R10, r_t);
			R[0] = r;
		}
		auto inline s_householder_ut2qr(Size m, Size n, const double *U, const double *tau, double *Q, double *R) { s_householder_ut2qr(m, n, U, n, tau, 1, Q, m, R, n); }
		// x must have the same or bigger size with b
		template<typename UType, typename TauType, typename BType, typename XType>
		auto inline s_householder_ut_q_dot(Size m, Size n, Size rhs, const double *U, UType u_t, const double *tau, TauType tau_t, const double *b, BType b_t, double *x, XType x_t)noexcept->void
		{
			s_mc(m, rhs, b, b_t, x, x_t);

			Size i_begin{ std::min(std::min(m - 1, n), m) };
			for (Size i(i_begin); --i < i_begin;)
			{
				for (Size j(-1); ++j < rhs;)
				{
					double k = tau[id(i, 0, tau_t)] * (x[id(i, j, x_t)] + s_vv(m - i - 1, U + id(i + 1, i, u_t), u_t, x + id(i + 1, j, x_t), x_t));
					x[id(i, j, x_t)] += k;
					s_ma(m - i - 1, 1, k, U + id(i + 1, i, u_t), u_t, x + id(i + 1, j, x_t), x_t);
				}
			}
		}
		auto inline s_householder_ut_q_dot(Size m, Size n, Size rhs, const double *U, const double *tau, const double *b, double *x) { s_householder_ut_q_dot(m, n, rhs, U, n, tau, 1, b, rhs, x, rhs); }
		// x must have the same or bigger size with b
		template<typename UType, typename TauType, typename BType, typename XType>
		auto inline s_householder_ut_qt_dot(Size m, Size n, Size rhs, const double *U, UType u_t, const double *tau, TauType tau_t, const double *b, BType b_t, double *x, XType x_t)noexcept->void
		{
			s_mc(m, rhs, b, b_t, x, x_t);

			for (Size i(-1), ti0{ 0 }, xi0{ 0 }, uii{0}; ++i < std::min(std::min(m - 1, n), m); ti0 = next_rid(ti0, tau_t), xi0 = next_rid(xi0, x_t), uii = next_did(uii, u_t))
			{
				for (Size j(-1), xij{xi0}; ++j < rhs; xij = next_cid(xij, x_t))
				{
					auto Xi1j = x + next_rid(xij, x_t);
					auto Ui1j = U + next_rid(uii, u_t);
					
					double k = tau[ti0] * (x[xij] + s_vv(m - i - 1, Ui1j, u_t, Xi1j, x_t));
					x[xij] += k;
					s_ma(m - i - 1, 1, k, Ui1j, u_t, Xi1j, x_t);
				}
			}
		}
		auto inline s_householder_ut_qt_dot(Size m, Size n, Size rhs, const double *U, const double *tau, const double *b, double *x) { s_householder_ut_qt_dot(m, n, rhs, U, n, tau, 1, b, rhs, x, rhs); }
		// x must have the same or bigger size with b
		template<typename UType, typename TauType, typename BType, typename XType>
		auto inline s_householder_ut_sov(Size m, Size n, Size rhs, const double *U, UType u_t, const double *tau, TauType tau_t, const double *b, BType b_t, double *x, XType x_t, double zero_check = 1e-10)noexcept->void
		{
			s_householder_ut_qt_dot(m, n, rhs, U, u_t, tau, tau_t, b, b_t, x, x_t);
			s_sov_um(std::min(m, n), rhs, U, u_t, x, x_t, x, x_t, zero_check);

			if (n > m)s_fill(n - m, rhs, 0.0, x + id(m, 0, x_t), x_t);
		}
		auto inline s_householder_ut_sov(Size m, Size n, Size rhs, const double *U, const double *tau, const double *b, double *x, double zero_check = 1e-10) { s_householder_ut_sov(m, n, rhs, U, n, tau, 1, b, rhs, x, rhs, zero_check); }
		// tau must have same size with max(m,n), A can be the same as U
		template<typename AType, typename UType, typename TauType>
		auto inline s_householder_utp(Size m, Size n, const double *A, AType a_t, double *U, UType u_t, double *tau, TauType tau_t, Size *p, Size &rank, double zero_check = 1e-10)noexcept->void
		{
			rank = 0;

			// init u //
			s_mc(m, n, A, a_t, U, u_t);

			// init p and tau //
			for (Size i(-1), ti0{ 0 }, u0i{ 0 }; ++i < n; p[i] = i, ti0 = next_rid(ti0, tau_t), u0i = next_cid(u0i, u_t))
			{
				tau[ti0] = s_vv(m, U + u0i, u_t, U + u0i, u_t);
			};

			// compute //
			for (Size i(-1), uii{ 0 }, ti0{ 0 }; ++i < std::min(m, n); uii = next_did(uii, u_t), ti0 = next_rid(ti0, tau_t))
			{
				double max_value{ 0 };
				Size max_pos{ i };

				for (Size j(i - 1), tj0{ ti0 }; ++j < n; tj0 = next_rid(tj0, tau_t))
				{
					max_pos = tau[tj0] > max_value ? j : max_pos;
					max_value = tau[tj0] > max_value ? tau[tj0] : max_value;
				}

				s_swap_v(m, U + id(0, max_pos, u_t), u_t, U + id(0, i, u_t), u_t);
				std::swap(tau[id(max_pos, 0, tau_t)], tau[ti0]);
				std::swap(p[max_pos], p[i]);

				// compute rank //
				// 这里由于截断误差，max_value可能小于0
				double rho = -std::sqrt(std::max(max_value, 0.0)) * s_sgn2(U[uii]);
				if (std::abs(rho) < zero_check) { s_fill(m - i, 1, 0.0, tau + ti0); return; }

				++rank;

				// 若已经到达最后一行，那么就返回，因为最后一行不需要householder化 //
				if (i == m - 1) return;
				
				// compute householder vector //
				auto U_i1_i = U + next_rid(uii, u_t);

				tau[ti0] = U[uii] / rho - 1.0;
				s_nm(m - 1 - i, 1, 1.0 / (U[uii] - rho), U_i1_i, u_t);
				U[uii] = rho;

				// update matrix //
				for (Size j(i), uij(next_cid(uii, u_t)), tj0{ next_rid(ti0, tau_t) }; ++j < n; uij = next_cid(uij, u_t), tj0 = next_rid(tj0, tau_t))
				{
					double t = tau[ti0] * (s_vv(m - i - 1, U_i1_i, u_t, U + next_rid(uij, u_t), u_t) + U[uij]);
					U[uij] += t;
					s_va(m - i - 1, t, U_i1_i, u_t, U + next_rid(uij, u_t), u_t);

					// 更新余下的矩阵列向量的模，用于下一轮寻找最大的模 //
					tau[tj0] -= U[uij] * U[uij];
				}
			}
		}
		auto inline s_householder_utp(Size m, Size n, const double *A, double *U, double *tau, Size *p, Size &rank, double zero_check = 1e-10)noexcept->void { s_householder_utp(m, n, A, n, U, n, tau, 1, p, rank, zero_check); }
		// x must have the same or bigger size with b
		template<typename UType, typename TauType, typename BType, typename XType>
		auto inline s_householder_utp_sov(Size m, Size n, Size rhs, Size rank, const double *U, UType u_t, const double *tau, TauType tau_t, const Size *p, const double *b, BType b_t, double *x, XType x_t, double zero_check = 1e-10)noexcept->void
		{
			s_householder_ut_qt_dot(m, rank, rhs, U, u_t, tau, tau_t, b, b_t, x, x_t);
			s_sov_um(rank, rhs, U, u_t, x, x_t, x, x_t, zero_check);
			
			if (n > m)s_fill(n - m, rhs, 0.0, x + id(m, 0, x_t), x_t);
			s_permutate_inv(n, rhs, p, x, x_t);
		}
		auto inline s_householder_utp_sov(Size m, Size n, Size rhs, Size rank, const double *U, const double *tau, const Size *p, const double *b, double *x, double zero_check = 1e-10) { s_householder_utp_sov(m, n, rhs, rank, U, n, tau, 1, p, b, rhs, x, rhs, zero_check); }
		template<typename UType, typename TauType, typename XType>
		auto inline s_householder_utp_sov_solution_space(Size m, Size n, Size rank, const double *U, UType u_t, const double *tau, TauType tau_t, const Size *p, double *x, XType x_t, double zero_check = 1e-10)noexcept->void
		{
			// [R1, R2]
			// solution is:
			// [ R1\R2 ]
			// [  -I   ]

			s_sov_um(rank, n - rank, U, u_t, U + id(0, rank, u_t), u_t, x, x_t, zero_check);
			s_fill(n - rank, n - rank, 0.0, x + id(rank, 0, x_t), x_t);
			for (Size i(-1); ++i < n - rank; )x[id(i + rank, i, x_t)] = -1.0;
			s_permutate_inv(n, n - rank, p, x, x_t);
		}
		auto inline s_householder_utp_sov_solution_space(Size m, Size n, Size rank, const double *U, const double *tau, const Size *p, double *x, double zero_check = 1e-10) { s_householder_utp_sov_solution_space(m, n, rank, U, n, tau, 1, p, x, n-rank, zero_check); }


	}
}




























#endif
