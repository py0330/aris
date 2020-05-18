#ifndef ARIS_DYNAMIC_MATRIX_H_
#define ARIS_DYNAMIC_MATRIX_H_

#include <vector>
#include <numeric>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <algorithm>
#include <cmath>

#include <aris_lib_export.h>
#include <aris/core/basic_type.hpp>

namespace aris::dynamic
{
	struct ARIS_API RowMajor { Size r_ld; constexpr RowMajor(Size r_ld_)noexcept :r_ld(r_ld_) {}; };
	struct ARIS_API ColMajor { Size c_ld; constexpr ColMajor(Size c_ld_)noexcept :c_ld(c_ld_) {}; };
	struct ARIS_API Stride { Size r_ld, c_ld; Stride()noexcept = default; constexpr Stride(Size r_ld_, Size c_ld_)noexcept :r_ld(r_ld_), c_ld(c_ld_) {}; };

	auto inline constexpr T(Size ld)noexcept->ColMajor { return ColMajor(ld); }
	auto inline constexpr at(Size i, Size ld)noexcept->Size { return i * ld; }
	auto inline constexpr at(Size i, Size j, Size ld)noexcept->Size { return i * ld + j; }
	auto inline constexpr next_r(Size at, Size ld)noexcept->Size { return at + ld; }
	auto inline constexpr last_r(Size at, Size ld)noexcept->Size { return at - ld; }
	auto inline constexpr next_c(Size at, Size ld)noexcept->Size { return at + 1; }
	auto inline constexpr last_c(Size at, Size ld)noexcept->Size { return at - 1; }
	auto inline constexpr next_d(Size at, Size ld)noexcept->Size { return at + 1 + ld; }
	auto inline constexpr last_d(Size at, Size ld)noexcept->Size { return at - 1 - ld; }

	auto inline constexpr T(RowMajor r)noexcept->ColMajor { return ColMajor(r.r_ld); }
	auto inline constexpr at(Size i, RowMajor row_major)noexcept->Size { return i * row_major.r_ld; }
	auto inline constexpr at(Size i, Size j, RowMajor row_major)noexcept->Size { return i * row_major.r_ld + j; }
	auto inline constexpr next_r(Size at, RowMajor row_major)noexcept->Size { return at + row_major.r_ld; }
	auto inline constexpr last_r(Size at, RowMajor row_major)noexcept->Size { return at - row_major.r_ld; }
	auto inline constexpr next_c(Size at, RowMajor row_major)noexcept->Size { return at + 1; }
	auto inline constexpr last_c(Size at, RowMajor row_major)noexcept->Size { return at - 1; }
	auto inline constexpr next_d(Size at, RowMajor row_major)noexcept->Size { return at + 1 + row_major.r_ld; }
	auto inline constexpr last_d(Size at, RowMajor row_major)noexcept->Size { return at - 1 - row_major.r_ld; }

	auto inline constexpr T(ColMajor c)noexcept->RowMajor { return RowMajor(c.c_ld); }
	auto inline constexpr at(Size i, ColMajor col_major)noexcept->Size { return i; }
	auto inline constexpr at(Size i, Size j, ColMajor col_major)noexcept->Size { return i + j * col_major.c_ld; }
	auto inline constexpr next_r(Size at, ColMajor col_major)noexcept->Size { return at + 1; }
	auto inline constexpr last_r(Size at, ColMajor col_major)noexcept->Size { return at - 1; }
	auto inline constexpr next_c(Size at, ColMajor col_major)noexcept->Size { return at + col_major.c_ld; }
	auto inline constexpr last_c(Size at, ColMajor col_major)noexcept->Size { return at - col_major.c_ld; }
	auto inline constexpr next_d(Size at, ColMajor col_major)noexcept->Size { return at + 1 + col_major.c_ld; }
	auto inline constexpr last_d(Size at, ColMajor col_major)noexcept->Size { return at - 1 - col_major.c_ld; }

	auto inline constexpr T(Stride s)noexcept->Stride { return Stride(s.c_ld, s.r_ld); }
	auto inline constexpr at(Size i, Stride stride)noexcept->Size { return i * stride.r_ld; }
	auto inline constexpr at(Size i, Size j, Stride stride)noexcept->Size { return i * stride.r_ld + j * stride.c_ld; }
	auto inline constexpr next_r(Size at, Stride stride)noexcept->Size { return at + stride.r_ld; }
	auto inline constexpr last_r(Size at, Stride stride)noexcept->Size { return at - stride.r_ld; }
	auto inline constexpr next_c(Size at, Stride stride)noexcept->Size { return at + stride.c_ld; }
	auto inline constexpr last_c(Size at, Stride stride)noexcept->Size { return at - stride.c_ld; }
	auto inline constexpr next_d(Size at, Stride stride)noexcept->Size { return at + stride.c_ld + stride.r_ld; }
	auto inline constexpr last_d(Size at, Stride stride)noexcept->Size { return at - stride.c_ld - stride.r_ld; }

	template <typename T, typename TType>
	auto inline dsp(Size m, Size n, const T *data, TType d_t)noexcept->void
	{
		std::cout << std::setiosflags(std::ios::fixed) << std::setiosflags(std::ios::right) << std::setprecision(14);

		std::cout << std::endl;
		for (Size i = 0; i < m; i++)
		{
			for (Size j = 0; j < n; j++)
			{
				std::cout << data[at(i, j, d_t)] << "   ";
			}
			std::cout << std::endl;
		}
		std::cout << std::endl;
	};
	template <typename T>
	auto inline dsp(Size m, Size n, const T *data)noexcept->void { dsp(m, n, data, n); }
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
				file << A[at(i, j, a_t)] << "   ";
			}
			file << std::endl;
		}
	}
	auto inline dlmwrite(const Size m, const Size n, const double *A, const char *filename)->void { dlmwrite(m, n, A, n, filename); }
	auto ARIS_API dlmread(const char *filename, double *mtx)->void;
	auto ARIS_API dlmread(const char *filename)->std::vector<double>;

	template <typename T>
	auto inline s_sgn(T val)noexcept->T { return T(T(0) < val) - (val < T(0)); }
	template <typename T>
	auto inline s_sgn(T val, T zero_check)noexcept->T { return std::abs(val)<zero_check ? T(0) : s_sgn(val); }
	template <typename T>
	auto inline s_sgn2(T val)noexcept->T { return val < T(0) ? T(-1) : T(1); }

	auto inline s_is_equal(double a, double b, double error)noexcept { return std::abs(a - b) < error; }
	template <typename V1Type, typename V2Type>
	auto inline s_is_equal(Size n, const double *v1, V1Type v1_t, const double *v2, V2Type v2_t, double error) noexcept->bool
	{
		for (Size i = 0; i < n; ++i)if (!s_is_equal(v1[at(i, v1_t)], v2[at(i, v2_t)], error))return false;
		return true;
	}
	auto inline s_is_equal(Size n, const double *v1, const double *v2, double error) noexcept->bool { return s_is_equal(n, v1, 1, v2, 1, error); };
	template <typename M1Type, typename M2Type>
	auto inline s_is_equal(Size m, Size n, const double *m1, M1Type m1_t, const double *m2, M2Type m2_t, double error) noexcept->bool
	{
		for (Size i = 0; i < m; ++i)for (Size j = 0; j < n; ++j)if (!s_is_equal(m1[at(i, j, m1_t)], m2[at(i, j, m2_t)], error)) return false;
		return true;
	}
	auto inline s_is_equal(Size m, Size n, const double *m1, const double *m2, double error) noexcept->bool { return s_is_equal(m, n, m1, n, m2, n, error); };

	template <typename AType>
	auto inline s_eye(Size m, double *A, AType a_t) noexcept->void
	{
		for (Size i(-1), ai0{ 0 }, aii{ 0 }; ++i < m; ai0 = next_r(ai0, a_t), aii = next_d(aii, a_t))
		{
			for (Size j(-1), aij{ ai0 }; ++j < m; aij = next_c(aij, a_t))
				A[aij] = 0.0;
			A[aii] = 1.0;
		}
	}
	auto inline s_eye(Size m, double *A) noexcept->void { return s_eye(m, A, m); }
	template <typename AType>
	auto inline s_rmx(double angle, double *A, AType a_t) noexcept->void
	{
		A[at(0, 0, a_t)] = 1.0;
		A[at(0, 1, a_t)] = 0.0;
		A[at(0, 2, a_t)] = 0.0;
		A[at(1, 0, a_t)] = 0.0;
		A[at(1, 1, a_t)] = std::cos(angle);
		A[at(1, 2, a_t)] = -std::sin(angle);
		A[at(2, 0, a_t)] = 0.0;
		A[at(2, 1, a_t)] = std::sin(angle);
		A[at(2, 2, a_t)] = std::cos(angle);
	}
	auto inline s_rmx(double angle, double *A) noexcept->void { return s_rmx(angle, A, 3); }
	template <typename AType>
	auto inline s_rmy(double angle, double *A, AType a_t) noexcept->void
	{
		A[at(0, 0, a_t)] = std::cos(angle);
		A[at(0, 1, a_t)] = 0.0;
		A[at(0, 2, a_t)] = std::sin(angle);
		A[at(1, 0, a_t)] = 0.0;
		A[at(1, 1, a_t)] = 1.0;
		A[at(1, 2, a_t)] = 0.0;
		A[at(2, 0, a_t)] = -std::sin(angle);
		A[at(2, 1, a_t)] = 0.0;
		A[at(2, 2, a_t)] = std::cos(angle);
	}
	auto inline s_rmy(double angle, double *A) noexcept->void { return s_rmy(angle, A, 3); }
	template <typename AType>
	auto inline s_rmz(double angle, double *A, AType a_t) noexcept->void
	{
		A[at(0, 0, a_t)] = std::cos(angle);
		A[at(0, 1, a_t)] = -std::sin(angle);
		A[at(0, 2, a_t)] = 0.0;
		A[at(1, 0, a_t)] = std::sin(angle);
		A[at(1, 1, a_t)] = std::cos(angle);
		A[at(1, 2, a_t)] = 0.0;
		A[at(2, 0, a_t)] = 0.0;
		A[at(2, 1, a_t)] = 0.0;
		A[at(2, 2, a_t)] = 1.0;
	}
	auto inline s_rmz(double angle, double *A) noexcept->void { return s_rmz(angle, A, 3); }

	template<typename XType>
	auto inline s_norm(Size n, const double *x, XType x_t) noexcept->double
	{
		double norm = 0;
		for (Size i(-1), x_id{ 0 }; ++i < n; x_id = next_r(x_id, x_t))norm += x[x_id] * x[x_id];
		return std::sqrt(norm);
	}
	auto inline s_norm(Size n, const double *x) noexcept->double { return s_norm(n, x, 1); }
	template<typename XType, typename YType>
	auto inline s_swap_v(Size n, double *x, XType x_t, double *y, YType y_t) noexcept->void
	{
		for (Size i(-1), x_id{ 0 }, y_id{ 0 }; ++i < n; x_id = next_r(x_id, x_t), y_id = next_r(y_id, y_t))
			std::swap(x[x_id], y[y_id]);
	}
	auto inline s_swap_v(Size n, double *x, double *y) noexcept->void { s_swap_v(n, x, 1, y, 1); }
	template<typename AType, typename BType>
	auto inline s_swap_m(Size m, Size n, double *a, AType a_t, double *b, BType b_t) noexcept->void
	{
		for (Size i(-1), ai0{ 0 }, bi0{ 0 }; ++i < m; ai0 = next_r(ai0, a_t), bi0 = next_r(bi0, b_t))
		{
			for (Size j(-1), aij{ ai0 }, bij{ bi0 }; ++j < n; aij = next_c(aij, a_t), bij = next_c(bij, b_t))
			{
				std::swap(a[aij], b[bij]);
			}
		}
	}
	auto inline s_swap_m(Size m, Size n, double *a, double *b) noexcept->void { s_swap_m(m, n, a, n, b, n); }
	template<typename AType>
	auto inline s_fill(Size m, Size n, double value, double *A, AType a_t) noexcept->void
	{
		for (Size i(-1), ai0{ 0 }; ++i < m; ai0 = next_r(ai0, a_t))
			for (Size j(-1), a_ij{ ai0 }; ++j < n; a_ij = next_c(a_ij, a_t))
				A[a_ij] = value;
	}// fill matrix with value
	auto inline s_fill(Size m, Size n, double value, double *A) noexcept->void { std::fill(A, A + m * n, value); }
	template<typename XType>
	auto inline s_iv(Size n, double *x, XType x_t) noexcept->void { for (Size i(-1), vid{ 0 }; ++i < n; vid = next_r(vid, x_t))x[vid] = -x[vid]; }
	auto inline s_iv(Size n, double *x) noexcept->void { for (Size i(-1); ++i < n;)x[i] = -x[i]; }
	template<typename XType>
	auto inline s_nv(Size n, double alpha, double *x, XType x_t) noexcept->void { for (Size i(-1), vid{ 0 }; ++i < n; vid = next_r(vid, x_t))x[vid] *= alpha; }
	auto inline s_nv(Size n, double alpha, double *x) noexcept->void { for (Size i(-1); ++i < n;)x[i] *= alpha; }
	template<typename XType, typename YType>
	auto inline s_vc(Size n, const double *x, XType x_t, double *y, YType y_t) noexcept->void { for (Size i(-1), x_id{ 0 }, y_id{ 0 }; ++i < n; x_id = next_r(x_id, x_t), y_id = next_r(y_id, y_t))y[y_id] = x[x_id]; }
	template<typename XType, typename YType>
	auto inline s_vc(Size n, double alpha, const double *x, XType x_t, double *y, YType y_t) noexcept->void { for (Size i(-1), x_id{ 0 }, y_id{ 0 }; ++i < n; x_id = next_r(x_id, x_t), y_id = next_r(y_id, y_t))y[y_id] = alpha * x[x_id]; }
	auto inline s_vc(Size n, const double *x, double *y) noexcept->void { std::copy_n(x, n, y); }
	auto inline s_vc(Size n, double alpha, const double *x, double *y) noexcept->void { for (Size i(-1); ++i < n;)y[i] = alpha * x[i]; }
	template<typename XType, typename YType>
	auto inline s_vi(Size n, const double *x, XType x_t, double *y, YType y_t) noexcept->void { for (Size i(-1), x_id{ 0 }, y_id{ 0 }; ++i < n; x_id = next_r(x_id, x_t), y_id = next_r(y_id, y_t))y[y_id] = -x[x_id]; }
	auto inline s_vi(Size n, const double *x, double *y) noexcept->void { for (Size i(-1); ++i < n;)y[i] = -x[i]; }
	template<typename XType, typename YType>
	auto inline s_va(Size n, const double* x, XType x_t, double* y, YType y_t) noexcept->void { for (Size i(-1), x_id{ 0 }, y_id{ 0 }; ++i < n; x_id = next_r(x_id, x_t), y_id = next_r(y_id, y_t))y[y_id] += x[x_id]; }
	template<typename XType, typename YType>
	auto inline s_va(Size n, double alpha, const double* x, XType x_t, double* y, YType y_t) noexcept->void { for (Size i(-1), x_id{ 0 }, y_id{ 0 }; ++i < n; x_id = next_r(x_id, x_t), y_id = next_r(y_id, y_t))y[y_id] += alpha * x[x_id]; }
	auto inline s_va(Size n, const double* x, double* y) noexcept->void { for (Size i = 0; i < n; ++i)y[i] += x[i]; }
	auto inline s_va(Size n, double alpha, const double* x, double* y) noexcept->void { for (Size i = 0; i < n; ++i)y[i] += alpha * x[i]; }
	template<typename XType, typename YType>
	auto inline s_vs(Size n, const double* x, XType x_t, double* y, YType y_t) noexcept->void { for (Size i(-1), x_id{ 0 }, y_id{ 0 }; ++i < n; x_id = next_r(x_id, x_t), y_id = next_r(y_id, y_t))y[y_id] -= x[x_id]; }
	auto inline s_vs(Size n, const double* x, double* y) noexcept->void { for (Size i = 0; i < n; ++i)y[i] -= x[i]; }
	template<typename XType, typename YType>
	auto inline s_vv(Size n, const double *x, XType x_t, const double *y, YType y_t) noexcept->double { double ret{ 0 }; for (Size i(-1), x_id{ 0 }, y_id{ 0 }; ++i < n; x_id = next_r(x_id, x_t), y_id = next_r(y_id, y_t))ret += x[x_id] * y[y_id]; return ret; }
	auto inline s_vv(Size n, const double *x, const double *y) noexcept->double { double ret{ 0 }; for (Size i = 0; i < n; ++i)ret += x[i] * y[i];	return ret; }
	template<typename AType>
	auto inline s_nm(Size m, Size n, double alpha, double* A, AType a_t) noexcept->void
	{
		for (Size i(-1), ai0{ 0 }; ++i < m; ai0 = next_r(ai0, a_t))
			for (Size j(-1), aij{ ai0 }; ++j < n; aij = next_c(aij, a_t))
				A[aij] *= alpha;
	}
	auto inline s_nm(Size m, Size n, double alpha, double* A) noexcept->void { s_nv(m*n, alpha, A); }
	template<typename AType>
	auto inline s_im(Size m, Size n, double* A, AType a_t) noexcept->void
	{
		for (Size i(-1), ai0{ 0 }; ++i < m; ai0 = next_r(ai0, a_t))
			for (Size j(-1), aij{ ai0 }; ++j < n; aij = next_c(aij, a_t))
				A[aij] = -A[aij];
	}
	auto inline s_im(Size m, Size n, double* A) noexcept->void { s_iv(m*n, A); }
	template<typename AType, typename BType>
	auto inline s_mc(Size m, Size n, const double *A, AType a_t, double *B, BType b_t) noexcept->void
	{
		for (Size i(-1), ai0{ 0 }, bi0{ 0 }; ++i < m; ai0 = next_r(ai0, a_t), bi0 = next_r(bi0, b_t))
			for (Size j(-1), aij{ ai0 }, bij{ bi0 }; ++j < n; aij = next_c(aij, a_t), bij = next_c(bij, b_t))
				B[bij] = A[aij];
	}
	template<typename AType, typename BType>
	auto inline s_mc(Size m, Size n, double alpha, const double *A, AType a_t, double *B, BType b_t) noexcept->void
	{
		for (Size i(-1), ai0{ 0 }, bi0{ 0 }; ++i < m; ai0 = next_r(ai0, a_t), bi0 = next_r(bi0, b_t))
			for (Size j(-1), aij{ ai0 }, bij{ bi0 }; ++j < n; aij = next_c(aij, a_t), bij = next_c(bij, b_t))
				B[bij] = alpha * A[aij];
	}
	auto inline s_mc(Size m, Size n, const double *A, double *B) noexcept->void { s_vc(m*n, A, B); }
	auto inline s_mc(Size m, Size n, double alpha, const double *A, double *B) noexcept->void { s_vc(m*n, alpha, A, B); }
	template<typename AType, typename BType>
	auto inline s_ma(Size m, Size n, const double* A, AType a_t, double* B, BType b_t) noexcept->void
	{
		for (Size i(-1), ai0{ 0 }, bi0{ 0 }; ++i < m; ai0 = next_r(ai0, a_t), bi0 = next_r(bi0, b_t))
			for (Size j(-1), aij{ ai0 }, bij{ bi0 }; ++j < n; aij = next_c(aij, a_t), bij = next_c(bij, b_t))
				B[bij] += A[aij];
	}
	template<typename AType, typename BType>
	auto inline s_ma(Size m, Size n, double alpha, const double* A, AType a_t, double* B, BType b_t) noexcept->void
	{
		for (Size i(-1), ai0{ 0 }, bi0{ 0 }; ++i < m; ai0 = next_r(ai0, a_t), bi0 = next_r(bi0, b_t))
			for (Size j(-1), aij{ ai0 }, bij{ bi0 }; ++j < n; aij = next_c(aij, a_t), bij = next_c(bij, b_t))
				B[bij] += alpha * A[aij];
	}
	auto inline s_ma(Size m, Size n, const double* A, double* B) noexcept->void { s_va(m*n, A, B); }
	auto inline s_ma(Size m, Size n, double alpha, const double* A, double* B) noexcept->void { s_va(m*n, alpha, A, B); }
	template<typename AType, typename BType>
	auto inline s_mi(Size m, Size n, const double* A, AType a_t, double* B, BType b_t) noexcept->void
	{
		for (Size i(-1), ai0{ 0 }, bi0{ 0 }; ++i < m; ai0 = next_r(ai0, a_t), bi0 = next_r(bi0, b_t))
			for (Size j(-1), aij{ ai0 }, bij{ bi0 }; ++j < n; aij = next_c(aij, a_t), bij = next_c(bij, b_t))
				B[bij] = -A[aij];
	}
	auto inline s_mi(Size m, Size n, const double* A, double* B) noexcept->void { s_vi(m*n, A, B); }
	template<typename AType, typename BType>
	auto inline s_ms(Size m, Size n, const double* A, AType a_t, double* B, BType b_t) noexcept->void
	{
		for (Size i(-1), ai0{ 0 }, bi0{ 0 }; ++i < m; ai0 = next_r(ai0, a_t), bi0 = next_r(bi0, b_t))
			for (Size j(-1), aij{ ai0 }, bij{ bi0 }; ++j < n; aij = next_c(aij, a_t), bij = next_c(bij, b_t))
				B[bij] -= A[aij];
	}
	auto inline s_ms(Size m, Size n, const double* A, double* B) noexcept->void { s_vs(m*n, A, B); }
	template<typename AType, typename BType, typename CType>
	auto inline s_mma(Size m, Size n, Size k, const double* A, AType a_t, const double* B, BType b_t, double *C, CType c_t)noexcept->void
	{
		for (Size i(-1), ai0{ 0 }, ci0{ 0 }; ++i < m; ai0 = next_r(ai0, a_t), ci0 = next_r(ci0, c_t))
		{
			for (Size j(-1), b0j{ 0 }, cij{ ci0 }; ++j < n; b0j = next_c(b0j, b_t), cij = next_c(cij, c_t))
			{
				for (Size u(-1), aiu{ ai0 }, buj{ b0j }; ++u < k; aiu = next_c(aiu, a_t), buj = next_r(buj, b_t))
					C[cij] += A[aiu] * B[buj];
			}
		}
	}
	template<typename AType, typename BType, typename CType>
	auto inline s_mma(Size m, Size n, Size k, double alpha, const double* A, AType a_t, const double* B, BType b_t, double *C, CType c_t)noexcept->void
	{
		for (Size i(-1), ai0{ 0 }, ci0{ 0 }; ++i < m; ai0 = next_r(ai0, a_t), ci0 = next_r(ci0, c_t))
		{
			for (Size j(-1), b0j{ 0 }, cij{ ci0 }; ++j < n; b0j = next_c(b0j, b_t), cij = next_c(cij, c_t))
			{
				double value{ 0 };
				for (Size u(-1), aiu{ ai0 }, buj{ b0j }; ++u < k; aiu = next_c(aiu, a_t), buj = next_r(buj, b_t))
					value += A[aiu] * B[buj];
				C[cij] += alpha * value;
			}
		}
	}
	auto inline s_mma(Size m, Size n, Size k, const double* A, const double* B, double *C) noexcept->void { s_mma(m, n, k, A, k, B, n, C, n); }
	auto inline s_mma(Size m, Size n, Size k, double alpha, const double* A, const double* B, double *C) noexcept->void { s_mma(m, n, k, alpha, A, k, B, n, C, n); }
	template<typename AType, typename BType, typename CType>
	auto inline s_mms(Size m, Size n, Size k, const double* A, AType a_t, const double* B, BType b_t, double *C, CType c_t)noexcept->void
	{
		for (Size i(-1), ai0{ 0 }, ci0{ 0 }; ++i < m; ai0 = next_r(ai0, a_t), ci0 = next_r(ci0, c_t))
		{
			for (Size j(-1), b0j{ 0 }, cij{ ci0 }; ++j < n; b0j = next_c(b0j, b_t), cij = next_c(cij, c_t))
			{
				for (Size u(-1), aiu{ ai0 }, buj{ b0j }; ++u < k; aiu = next_c(aiu, a_t), buj = next_r(buj, b_t))
					C[cij] -= A[aiu] * B[buj];
			}
		}
	}
	auto inline s_mms(Size m, Size n, Size k, const double* A, const double* B, double *C) noexcept->void { s_mms(m, n, k, A, k, B, n, C, n); }
	template<typename AType, typename BType, typename CType>
	auto inline s_mm(Size m, Size n, Size k, const double* A, AType a_t, const double* B, BType b_t, double *C, CType c_t) noexcept->void
	{
		for (Size i(-1), ai0{ 0 }, ci0{ 0 }; ++i < m; ai0 = next_r(ai0, a_t), ci0 = next_r(ci0, c_t))
		{
			for (Size j(-1), b0j{ 0 }, cij{ ci0 }; ++j < n; b0j = next_c(b0j, b_t), cij = next_c(cij, c_t))
			{
				C[cij] = 0.0;
				for (Size u(-1), aiu{ ai0 }, buj{ b0j }; ++u < k; aiu = next_c(aiu, a_t), buj = next_r(buj, b_t))
					C[cij] += A[aiu] * B[buj];
			}
		}
	}
	template<typename AType, typename BType, typename CType>
	auto inline s_mm(Size m, Size n, Size k, double alpha, const double* A, AType a_t, const double* B, BType b_t, double *C, CType c_t) noexcept->void
	{
		for (Size i(-1), ai0{ 0 }, ci0{ 0 }; ++i < m; ai0 = next_r(ai0, a_t), ci0 = next_r(ci0, c_t))
		{
			for (Size j(-1), b0j{ 0 }, cij{ ci0 }; ++j < n; b0j = next_c(b0j, b_t), cij = next_c(cij, c_t))
			{
				C[cij] = 0.0;
				for (Size u(-1), aiu{ ai0 }, buj{ b0j }; ++u < k; aiu = next_c(aiu, a_t), buj = next_r(buj, b_t))
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
		for (Size i(-1), ai0{ 0 }, ci0{ 0 }; ++i < m; ai0 = next_r(ai0, a_t), ci0 = next_r(ci0, c_t))
		{
			for (Size j(-1), b0j{ 0 }, cij{ ci0 }; ++j < n; b0j = next_c(b0j, b_t), cij = next_c(cij, c_t))
			{
				C[cij] = 0.0;
				for (Size u(-1), aiu{ ai0 }, buj{ b0j }; ++u < k; aiu = next_c(aiu, a_t), buj = next_r(buj, b_t))
					C[cij] -= A[aiu] * B[buj];
			}
		}
	}
	auto inline s_mmi(Size m, Size n, Size k, const double* A, const double* B, double *C) noexcept->void { s_fill(m, n, 0, C); s_mms(m, n, k, A, B, C); }

	// x_new[i] = x[p[i]]
	template<typename XType>
	auto inline s_permutate(Size m, Size rhs, const Size *p, double *x, XType x_t)noexcept->void
	{
		for (Size i(-1), xi0{ 0 }; ++i < m; xi0 = next_r(xi0, x_t))
		{
			Size k{ p[i] };
			for (; k < i; k = p[k]);

			for (Size j(-1), xij{ xi0 }, xkj{ at(k,0,x_t) }; ++j < rhs; xij = next_c(xij, x_t), xkj = next_c(xkj, x_t))std::swap(x[xij], x[xkj]);
		}
	}
	auto inline s_permutate(Size m, Size rhs, const Size *p, double *x)noexcept->void { s_permutate(m, rhs, p, x, rhs); }
	// x_new[p[i]] = x[i]
	template<typename XType>
	auto inline s_permutate_inv(Size m, Size rhs, const Size *p, double *x, XType x_t)noexcept->void
	{
		for (Size i(-1), xi0{ 0 }; ++i < m; xi0 = next_r(xi0, x_t))
		{
			Size check_k{ p[i] };
			for (; check_k > i; check_k = p[check_k]);

			if (check_k == i)
			{
				for (Size k{ p[i] }; k > i; k = p[k])
				{
					for (Size j(-1), xij{ xi0 }, xkj{ at(k,0,x_t) }; ++j < rhs; xij = next_c(xij, x_t), xkj = next_c(xkj, x_t))
						std::swap(x[xij], x[xkj]);
				}
			}
		}
	}
	auto inline s_permutate_inv(Size m, Size rhs, const Size *p, double *x)noexcept->void { s_permutate_inv(m, rhs, p, x, rhs); }

	// A can be the same as L, only when they have same type
	template<typename AType, typename LType>
	auto inline s_llt(Size m, const double *A, AType a_t, double *L, LType l_t) noexcept->void
	{
		for (Size j(-1), ajj{ 0 }, ljj{ 0 }, lj0{ 0 }; ++j < m; ajj = next_d(ajj, a_t), ljj = next_d(ljj, l_t), lj0 = next_r(lj0, l_t))
		{
			L[ljj] = A[ajj];
			for (Size k(-1), ljk{ lj0 }; ++k < j; ljk = next_c(ljk, l_t))
			{
				L[ljj] -= L[ljk] * L[ljk];
			}
			L[ljj] = std::sqrt(L[ljj]);


			for (Size i(j), li0{ next_r(lj0,l_t) }, lji{ next_c(ljj, l_t) }, lij{ next_r(ljj, l_t) }, a_ij{ next_r(ajj,a_t) }; ++i < m; li0 = next_r(li0, l_t), lji = next_c(lji, l_t), lij = next_r(lij, l_t), a_ij = next_r(a_ij, a_t))
			{
				L[lij] = A[a_ij];
				for (Size k(-1), l_ik{ li0 }, ljk{ lj0 }; ++k < j; l_ik = next_c(l_ik, l_t), ljk = next_c(ljk, l_t))
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
		for (Size j(-1), inv_Ljj{ 0 }, Ljj{ 0 }; ++j < m; inv_Ljj = next_r(next_c(inv_Ljj, inv_l_t), inv_l_t), Ljj = next_r(next_c(Ljj, l_t), l_t))
		{
			inv_L[inv_Ljj] = 1.0 / L[Ljj];

			for (Size i(j), inv_Lij{ next_r(inv_Ljj, inv_l_t) }, inv_Lji{ next_c(inv_Ljj, inv_l_t) }; ++i < m; inv_Lij = next_r(inv_Lij, inv_l_t), inv_Lji = next_c(inv_Lji, inv_l_t))
			{
				double alpha{ 0 };
				for (Size k(j - 1), Lik{ at(i, j, l_t) }, inv_Lkj{ at(j, j, inv_l_t) }; ++k < i; Lik = next_c(Lik, l_t), inv_Lkj = next_r(inv_Lkj, inv_l_t))
				{
					alpha -= L[Lik] * inv_L[inv_Lkj];
				}
				inv_L[inv_Lij] = alpha / L[at(i, i, l_t)];
				inv_L[inv_Lji] = 0.0;
			}
		}
	}
	auto inline s_inv_lm(Size m, const double *L, double *inv_L) noexcept->void { s_inv_lm(m, L, m, inv_L, m); }
	// U can be the same as inv_U, only when they have same type
	template<typename LType, typename InvLType>
	auto inline s_inv_um(Size m, const double *U, LType l_t, double *inv_U, InvLType inv_l_t) noexcept->void { s_inv_lm(m, U, T(l_t), inv_U, T(inv_l_t)); }
	auto inline s_inv_um(Size m, const double *U, double *inv_U) noexcept->void { s_inv_um(m, U, m, inv_U, m); }
	// b can be the same as x, only when they have same type
	template<typename LType, typename bType, typename xType>
	auto inline s_sov_lm(Size m, Size rhs, const double *L, LType l_t, const double *b, bType b_t, double *x, xType x_t, double zero_check = 1e-10) noexcept->void
	{
		for (Size j(-1), x0j{ 0 }, b0j{ 0 }; ++j < rhs; x0j = next_c(x0j, x_t), b0j = next_c(b0j, b_t))
		{
			for (Size i(-1), xij{ x0j }, bij{ b0j }, li0{ 0 }, lii{ 0 }; ++i < m; xij = next_r(xij, x_t), bij = next_r(bij, b_t), li0 = next_r(li0, l_t), lii = next_d(lii, l_t))
			{
				x[xij] = b[bij];

				for (Size k(-1), lik{ li0 }, xkj{ x0j }; ++k < i; lik = next_c(lik, l_t), xkj = next_r(xkj, x_t))
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
		for (Size j(-1), xmj{ at(m - 1, 0, x_t) }, bmj{ at(m - 1, 0, b_t) }; ++j < rhs; xmj = next_c(xmj, x_t), bmj = next_c(bmj, b_t))
		{
			for (Size i(m), xij{ xmj }, bij{ bmj }, lii{ at(m - 1, m - 1, l_t) }; --i < m; xij = last_r(xij, x_t), bij = last_r(bij, b_t), lii = last_d(lii, l_t))
			{
				x[xij] = b[bij];

				for (Size k(i), lik{ next_c(lii, l_t) }, xkj{ next_r(xij, x_t) }; ++k < m; lik = next_c(lik, l_t), xkj = next_r(xkj, x_t))
				{
					x[xij] -= L[lik] * x[xkj];
				}
				x[xij] = std::abs(L[lii]) > zero_check ? x[xij] / L[lii] : 0.0;
			}
		}
	}
	auto inline s_sov_um(Size m, Size rhs, const double *L, const double *b, double *x, double zero_check = 1e-10) noexcept->void { s_sov_um(m, rhs, L, m, b, rhs, x, rhs, zero_check); }

	// solve decomposition of A
	//
	//    A :        m x n
	//    U :        m x n
	//  tau : max(m,n) x 1
	//
	//    U can be the same address with A
	template<typename AType, typename UType, typename TauType>
	auto inline s_householder_ut(Size m, Size n, const double *A, AType a_t, double *U, UType u_t, double *tau, TauType tau_t, double zero_check = 1e-10)noexcept->void
	{
		s_mc(m, n, A, a_t, U, u_t);
		// 这里防止 m - 1 变成 -1（既最大）
		for (Size i(-1), uii{ 0 }, ti{ 0 }; ++i < std::min({ m - 1, m, n }); uii = next_d(uii, u_t), ti = next_r(ti, tau_t))
		{
			// compute householder vector //
			double rho = -s_norm(m - i, U + uii, u_t) * s_sgn2(U[uii]);
			if (std::abs(rho) > zero_check)
			{
				auto U_i1_i = U + next_r(uii, u_t);

				tau[ti] = U[uii] / rho - 1.0;
				s_nv(m - 1 - i, 1.0 / (U[uii] - rho), U_i1_i, u_t);
				U[uii] = rho;

				for (Size j(i), uij{ next_c(uii,u_t) }; ++j < n; uij = next_c(uij, u_t))
				{
					auto U_i1_j = U + next_r(uij, u_t);

					double k = tau[ti] * (s_vv(m - i - 1, U_i1_i, u_t, U_i1_j, u_t) + U[uij]);
					U[uij] += k;
					s_va(m - i - 1, k, U_i1_i, u_t, U_i1_j, u_t);
				}
			}
			else
			{
				tau[ti] = 0.0;
			}
		}
	}
	auto inline s_householder_ut(Size m, Size n, const double *A, double *U, double *tau, double zero_check = 1e-10)noexcept->void { s_householder_ut(m, n, A, n, U, n, tau, 1, zero_check); }
	
	//    Q = f(U,tau)  where Q * R = A 
	//
	//    U :        m x n
	//  tau : max(m,n) x 1
	//    Q :        m x m
	//
	//    Q can be the same address with U, if m < n
	template<typename UType, typename TauType, typename QType>
	auto inline s_householder_ut2q(Size m, Size n, const double *U, UType u_t, const double *tau, TauType tau_t, double *Q, QType q_t)noexcept->void
	{
		auto size = std::min(m, n);
		s_fill(m - size, m - size, 0.0, Q + at(n, n, q_t), q_t);
		for (Size i(-1), qii(at(n, n, q_t)); ++i < m - size; qii = next_d(qii, q_t)) Q[qii] = 1.0;
		if (m > 0) Q[at(m - 1, m - 1, q_t)] = 1.0;

		// make Q
		for (Size j(std::min({ m - 1, m, n })), qjj(at(j - 1, j - 1, q_t)), uj1j(at(j, j - 1, u_t)), tj(at(j - 1, tau_t)); --j < std::min({ m - 1, m, n }); qjj = last_d(qjj, q_t), uj1j = last_d(uj1j, u_t), tj = last_r(tj, tau_t))
		{
			Q[qjj] = 1 + tau[tj];
			s_vc(m - j - 1, tau[tj], U + uj1j, u_t, Q + next_r(qjj, q_t), q_t);
			s_mm(1, m - j - 1, m - j - 1, Q + next_r(qjj, q_t), T(q_t), Q + next_d(qjj, q_t), q_t, Q + next_c(qjj, q_t), q_t);
			s_mma(m - j - 1, m - j - 1, 1, U + uj1j, u_t, Q + next_c(qjj, q_t), q_t, Q + next_d(qjj, q_t), q_t);
		}
	}
	auto inline s_householder_ut2q(Size m, Size n, const double *U, const double *tau, double *Q)noexcept->void { s_householder_ut2q(m, n, U, n, tau, 1, Q, m); }
	
	//    Q = f(U,tau)  where Q * R = A 
	//
	//    U :        m x n
	//  tau : max(m,n) x 1
	//    Q :        m x n
	//
	//    Q can be the same address with U
	template<typename UType, typename TauType, typename QType>
	auto inline s_householder_ut2qmn(Size m, Size n, const double *U, UType u_t, const double *tau, TauType tau_t, double *Q, QType q_t)noexcept->void
	{
		if (m <= n && m > 0) Q[at(m - 1, m - 1, q_t)] = 1.0;
		// make Q
		for (Size j(std::min({ m - 1, m, n })), qjj(at(j - 1, j - 1, q_t)), uj1j(at(j, j - 1, u_t)), tj(at(j - 1, tau_t)); --j < std::min({ m - 1, m, n }); qjj = last_d(qjj, q_t), uj1j = last_d(uj1j, u_t), tj = last_r(tj, tau_t))
		{
			Q[qjj] = 1 + tau[tj];
			s_vc(m - j - 1, tau[tj], U + uj1j, u_t, Q + next_r(qjj, q_t), q_t);
			s_mm(1, std::min(m, n) - j - 1, m - j - 1, Q + next_r(qjj, q_t), T(q_t), Q + next_d(qjj, q_t), q_t, Q + next_c(qjj, q_t), q_t);
			s_mma(m - j - 1, std::min(m, n) - j - 1, 1, U + uj1j, u_t, Q + next_c(qjj, q_t), q_t, Q + next_d(qjj, q_t), q_t);
		}
	}
	auto inline s_householder_ut2qmn(Size m, Size n, const double *U, const double *tau, double *Q)noexcept->void { s_householder_ut2qmn(m, n, U, n, tau, 1, Q, n); }
	
	//    R = f(U,tau)  where Q * R = A 
	//
	//    U :        m x n
	//  tau : max(m,n) x 1
	//    R :        m x n
	//
	//    R can be the same address with U
	template<typename UType, typename TauType, typename RType>
	auto inline s_householder_ut2r(Size m, Size n, const double *U, UType u_t, const double *tau, TauType tau_t, double *R, RType r_t)noexcept->void
	{
		s_mc(m, n, U, u_t, R, r_t);
		for (Size i(-1), rj1j{ next_r(0,r_t) }; ++i < std::min({ m - 1, m, n }); rj1j = next_d(rj1j, r_t))
		{
			s_fill(m - i - 1, 1, 0.0, R + rj1j, r_t);
		}
	}
	auto inline s_householder_ut2r(Size m, Size n, const double *U, const double *tau, double *R)noexcept->void { s_householder_ut2r(m, n, U, n, tau, 1, R, n); }
	
	// [Q,R]= f(U,tau)  where Q * R = A 
	//
	//    U :        m x n
	//  tau : max(m,n) x 1
	//    Q :        m x m
	//    R :        m x n
	//
	//    R can be the same address with U
	template<typename UType, typename TauType, typename QType, typename RType>
	auto inline s_householder_ut2qr(Size m, Size n, const double *U, UType u_t, const double *tau, TauType tau_t, double *Q, QType q_t, double *R, RType r_t)noexcept->void
	{
		s_householder_ut2q(m, n, U, u_t, tau, tau_t, Q, q_t);
		s_householder_ut2r(m, n, U, u_t, tau, tau_t, R, r_t);
	}
	auto inline s_householder_ut2qr(Size m, Size n, const double *U, const double *tau, double *Q, double *R)noexcept->void { s_householder_ut2qr(m, n, U, n, tau, 1, Q, m, R, n); }
	
	//    x = Q * b 
	//
	//    U :        m x n
	//  tau : max(m,n) x 1
	//    b :        m x rhs
	//    x : max(n,m) x rhs
	//
	//    x can be the same address with b
	template<typename UType, typename TauType, typename BType, typename XType>
	auto inline s_householder_ut_q_dot(Size m, Size n, Size rhs, const double *U, UType u_t, const double *tau, TauType tau_t, const double *b, BType b_t, double *x, XType x_t)noexcept->void
	{
		s_mc(m, rhs, b, b_t, x, x_t);

		Size i_begin{ std::min({ m - 1, m, n }) };
		for (Size i(i_begin), ti{ at(i - 1, tau_t) }, xi0{ at(i - 1, 0, x_t) }, uii{ at(i - 1, i - 1, u_t) }; --i < i_begin; ti = last_r(ti, tau_t), xi0 = last_r(xi0, x_t), uii = last_d(uii, u_t))
		{
			for (Size j(-1), xij{ xi0 }; ++j < rhs; xij = next_c(xij, x_t))
			{
				auto Xi1j = x + next_r(xij, x_t);
				auto Ui1j = U + next_r(uii, u_t);

				double k = tau[ti] * (x[xij] + s_vv(m - i - 1, Ui1j, u_t, Xi1j, x_t));
				x[xij] += k;
				s_ma(m - i - 1, 1, k, Ui1j, u_t, Xi1j, x_t);
			}
		}
	}
	auto inline s_householder_ut_q_dot(Size m, Size n, Size rhs, const double *U, const double *tau, const double *b, double *x)noexcept->void { s_householder_ut_q_dot(m, n, rhs, U, n, tau, 1, b, rhs, x, rhs); }
	
	//    x = Q^T * b
	//
	//    U :        m x n
	//  tau : max(m,n) x 1
	//    b :        m x rhs
	//    x :        m x rhs
	//
	//    x can be the same address with b
	template<typename UType, typename TauType, typename BType, typename XType>
	auto inline s_householder_ut_qt_dot(Size m, Size n, Size rhs, const double *U, UType u_t, const double *tau, TauType tau_t, const double *b, BType b_t, double *x, XType x_t)noexcept->void
	{
		s_mc(m, rhs, b, b_t, x, x_t);

		for (Size i(-1), ti{ 0 }, xi0{ 0 }, uii{ 0 }; ++i < std::min({ m - 1, m, n }); ti = next_r(ti, tau_t), xi0 = next_r(xi0, x_t), uii = next_d(uii, u_t))
		{
			for (Size j(-1), xij{ xi0 }; ++j < rhs; xij = next_c(xij, x_t))
			{
				auto Xi1j = x + next_r(xij, x_t);
				auto Ui1j = U + next_r(uii, u_t);

				double k = tau[ti] * (x[xij] + s_vv(m - i - 1, Ui1j, u_t, Xi1j, x_t));
				x[xij] += k;
				s_ma(m - i - 1, 1, k, Ui1j, u_t, Xi1j, x_t);
			}
		}
	}
	auto inline s_householder_ut_qt_dot(Size m, Size n, Size rhs, const double *U, const double *tau, const double *b, double *x)noexcept->void { s_householder_ut_qt_dot(m, n, rhs, U, n, tau, 1, b, rhs, x, rhs); }
	
	//    U :        m x n
	//  tau : max(m,n) x 1
	//    p : max(m,n) x 1
	//    b :        m x rhs
	//    x : max(n,m) x rhs
	//
	//    x can be the same address with b
	template<typename UType, typename TauType, typename BType, typename XType>
	auto inline s_householder_ut_sov(Size m, Size n, Size rhs, const double *U, UType u_t, const double *tau, TauType tau_t, const double *b, BType b_t, double *x, XType x_t, double zero_check = 1e-10)noexcept->void
	{
		s_householder_ut_qt_dot(m, n, rhs, U, u_t, tau, tau_t, b, b_t, x, x_t);
		s_sov_um(std::min(m, n), rhs, U, u_t, x, x_t, x, x_t, zero_check);

		if (n > m)s_fill(n - m, rhs, 0.0, x + at(m, 0, x_t), x_t);
	}
	auto inline s_householder_ut_sov(Size m, Size n, Size rhs, const double *U, const double *tau, const double *b, double *x, double zero_check = 1e-10)noexcept->void { s_householder_ut_sov(m, n, rhs, U, n, tau, 1, b, rhs, x, rhs, zero_check); }
	
	// solve decomposition of A * P
	//    A :        m x n
	//    U :        m x n
	//  tau : max(m,n) x 1
	//    p : max(m,n) x 1
	//
	//    U can be the same address with A
	template<typename AType, typename UType, typename TauType>
	auto inline s_householder_utp(Size m, Size n, const double *A, AType a_t, double *U, UType u_t, double *tau, TauType tau_t, Size *p, Size &rank, double zero_check = 1e-10)noexcept->void
	{
		rank = 0;

		// init u //
		s_mc(m, n, A, a_t, U, u_t);

		// init p //
		std::iota(p, p + n, 0);

		// compute //
		for (Size i(-1), uii{ 0 }, ti{ 0 }; ++i < std::min(m, n); uii = next_d(uii, u_t), ti = next_r(ti, tau_t))
		{
			// 找到模最大的一列 //
			double max_value{ 0 };
			Size max_pos{ i };

			for (Size j(i - 1), uij{ uii }; ++j < n; uij = next_c(uij, u_t))
			{
				double value = s_vv(m - i, U + uij, u_t, U + uij, u_t);
				max_pos = value > max_value ? j : max_pos;
				max_value = value > max_value ? value : max_value;
			}

			// 判断是否返回 //
			max_value = std::sqrt(max_value);
			if (max_value < zero_check) { s_fill(m - i, 1, 0.0, tau + ti, tau_t); return; }
			
			++rank;
			s_swap_v(m, U + at(0, max_pos, u_t), u_t, U + at(0, i, u_t), u_t);
			std::swap(p[max_pos], p[i]);

			// 若已经到达最后一行，那么就返回，因为最后一行不需要householder化 //
			if (i == m - 1) return;

			// compute householder vector //
			auto U_i1_i = U + next_r(uii, u_t);

			double rho = -max_value * s_sgn2(U[uii]);
			tau[ti] = U[uii] / rho - 1.0;
			s_nm(m - 1 - i, 1, 1.0 / (U[uii] - rho), U_i1_i, u_t);
			U[uii] = rho;

			// update matrix //
			for (Size j(i), uij(next_c(uii, u_t)), tj{ next_r(ti, tau_t) }; ++j < n; uij = next_c(uij, u_t), tj = next_r(tj, tau_t))
			{
				auto U_i1_j = U + next_r(uij, u_t);

				double k = tau[ti] * (s_vv(m - i - 1, U_i1_i, u_t, U_i1_j, u_t) + U[uij]);
				U[uij] += k;
				s_va(m - i - 1, k, U_i1_i, u_t, U_i1_j, u_t);
			}
		}
	}
	auto inline s_householder_utp(Size m, Size n, const double *A, double *U, double *tau, Size *p, Size &rank, double zero_check = 1e-10)noexcept->void { s_householder_utp(m, n, A, n, U, n, tau, 1, p, rank, zero_check); }
	
	//    U :        m x n
	//  tau : max(m,n) x 1
	//    p : max(m,n) x 1
	//    b :        m x rhs
	//    x : max(n,m) x rhs
	//
	//    x can be the same address with b
	template<typename UType, typename TauType, typename BType, typename XType>
	auto inline s_householder_utp_sov(Size m, Size n, Size rhs, Size rank, const double *U, UType u_t, const double *tau, TauType tau_t, const Size *p, const double *b, BType b_t, double *x, XType x_t, double zero_check = 1e-10)noexcept->void
	{
		s_householder_ut_qt_dot(m, rank, rhs, U, u_t, tau, tau_t, b, b_t, x, x_t);
		s_sov_um(rank, rhs, U, u_t, x, x_t, x, x_t, zero_check);
		s_fill(n - rank, rhs, 0.0, x + at(rank, 0, x_t), x_t);
		s_permutate_inv(n, rhs, p, x, x_t);
	}
	auto inline s_householder_utp_sov(Size m, Size n, Size rhs, Size rank, const double *U, const double *tau, const Size *p, const double *b, double *x, double zero_check = 1e-10)noexcept->void { s_householder_utp_sov(m, n, rhs, rank, U, n, tau, 1, p, b, rhs, x, rhs, zero_check); }
	template<typename UType, typename TauType, typename XType>
	auto inline s_householder_utp_sov_solution_space(Size m, Size n, Size rank, const double *U, UType u_t, const double *tau, TauType tau_t, const Size *p, double *x, XType x_t, double zero_check = 1e-10)noexcept->void
	{
		// R is:
		// [R1, R2]
		// [0 , 0 ]
		//
		// its solution space is:
		// [ R1\R2 ]
		// [  -I   ]

		s_sov_um(rank, n - rank, U, u_t, U + at(0, rank, u_t), u_t, x, x_t, zero_check);
		s_fill(n - rank, n - rank, 0.0, x + at(rank, 0, x_t), x_t);
		for (Size i(-1); ++i < n - rank; )x[at(i + rank, i, x_t)] = -1.0;
		s_permutate_inv(n, n - rank, p, x, x_t);
	}
	auto inline s_householder_utp_sov_solution_space(Size m, Size n, Size rank, const double *U, const double *tau, const Size *p, double *x, double zero_check = 1e-10)noexcept->void { s_householder_utp_sov_solution_space(m, n, rank, U, n, tau, 1, p, x, n - rank, zero_check); }
	
	//    U :        m x n
	//  tau : max(m,n) x 1
	//    p : max(m,n) x 1
	//    x :        n x m
	// tau2 : max(n,m) x 1
	template<typename UType, typename TauType, typename XType, typename TauType2>
	auto inline s_householder_utp2pinv(Size m, Size n, Size rank, const double *U, UType u_t, const double *tau, TauType tau_t, const Size *p, double *x, XType x_t, double *tau2, TauType2 t_t, double zero_check = 1e-10)noexcept->void
	{
		// X 是 A 的 moore penrose 逆，为 n x m 维
		//
		// A 的qr分解:
		// A_mn * P_nn = Q_mxm * R_mxn
		// 若A的秩为r
		// 此时R_mn 为：
		//   1           r         n
		// 1 [           .         ]
		//   |    R1     .    R2   |
		// r |.....................|
		//   |           .         |
		// m [           .         ] 
		// 
		// A*P*x = b的最小范数的最小二乘解（linear least square:lls）为：
		// 
		// c = QT * b
		// y = P * x
		// 
		// y的某一个特解为y_t：
		// 1 [         ]
		//   |  R1\c1  |
		// r |.........|
		//   |         |
		// n [    0    ]
		// 
		// y的通解为y_s：
		//     1         n-r
		// 1 [             ]
		//   |    R1\R2    |
		// r | ............|
		//   | -1          |           
		//   |    -1       |
		//   |       ..    |
		// n [          -1 ]
		// 
		// 于是y的lls解中需求通解y_s的系数，它以下方程的最小二乘解：
		// 
		// y_s * k = y_t
		// k =  y_s\(R1\c1) = y_s \ y_t
		//
		// 于是y的lls解为：
		// y = y_t - y_s*(y_s \ y_t）=(I - y_s * y_s^-1) y_t
		// x = P * y
		//
		// 
		// 
		// 若求广义逆，那么令b为mxm的单位阵，此时所求出x即为广义逆A+：
		// 同时令QR分解矩阵为：
		// A_mn * P_nn = Q_mxr * R_rxn
		// 此时y_t：
		//     1            m
		// 1 [              ]
		//   |  R1\Q_mxr^T  |
		// r |..............|
		//   |              |
		// n [    0         ]
		//
		// 而此时y_s可以进行qr分解：
		// y_s = S_nxn * T_nxn-r
		// 其中 T_nxn-r：
		//     1     n-r               
		//  1  [      |           
		//     |  T1  |
		// n-r |......|
		//     |      |
		//  n  [  T2  ]
		// 这里T_nxn-r ^ -1为：
		// 
		//     1       n-r    n               
		//  1  [        .     ]           
		//     | T1^-1  .     |
		// n-r [        .     ]
		// 
		// 于是 y_s * y_s ^ -1 为：
		//   S * T * T^-1 *S^T
		// = S(:,1:n-r) * S(:,1:n-r) ^ T
		// 
		// 而 I - y_s * y_s^-1 = S*S^T - S(:,1:n-r) * S(:,1:n-r) ^ T
		//  = S(:,n-r+1:end) * S(:,n-r+1:end) ^ T
		// 带入上式：
		// y = S(:,n-r+1:end)_nxr * S(:,n-r+1:end)^T_rxn * y_t
		//   = S(:,n-r+1:end)_nxr * S(1:r,n-r+1:end)^T_rxr * (R1\Q^T)_rxm
		// 
		// x = P^-1 * y;
		// 
		//


		// step 1:
		// change x to:
		//   1           r         m
		// 1 [                     ]
		//   |         R1\QT       |
		// r |.....................|
		//   | (R1\R2)^T .         |
		// n [           .         ] 
		//
		//

		// QT
		s_householder_ut2qmn(m, rank, U, u_t, tau, tau_t, x, T(x_t));
		// R1\QT
		s_sov_um(rank, m, U, u_t, x + at(0, 0, x_t), x_t, x, x_t, zero_check);
		// R1\R2
		s_sov_um(rank, n - rank, U, u_t, U + at(0, rank, u_t), u_t, x + at(rank, 0, x_t), T(x_t), zero_check);

		// step 2:
		// 将通解矩阵做个行变换如下：
		//
		//     1           n-r         
		//  1  [ -1          |           
		//     |    -1       |
		//     |       ..    |
		//     |          -1 |
		// n-r | ............|
		//     |    R1\R2    |
		//  n  [             ]
		//
		// 其household变换产生的U（记作S）和tau有如下特征：
		//     1           n-r         
		//  1  [ *  *  *  * |           
		//     |    *  *  * |
		//     |       *  * |
		//     |          * |
		// n-r |............|
		//     | *  *  *  * |
		//  n  [ *  *  *  * ]
		//
		//   1       n-r         
		// [ *  *  *  * ]
		// 
		// 这里将上述S的下三角部分储存在x中
		//
		// x变成:
		//   1           r         m
		// 1 [                     ]
		//   |         R1\QT       |
		// r |.....................|
		//   |     S     .         |
		// n [           .         ] 
		//
		//
		// make S and tau
		for (Size i(-1), k0i{ at(0, rank, T(x_t)) }, ti{ 0 }; ++i < std::min({ n - rank, n, n - 1 }); k0i = next_c(k0i, T(x_t)), ti = next_r(ti, t_t))
		{
			double rho = std::sqrt(s_vv(rank, x + k0i, T(x_t), x + k0i, T(x_t)) + 1.0);

			// Aii 为 -1.0
			s_nv(rank, 1.0 / (-1.0 - rho), x + k0i, T(x_t));
			tau2[ti] = -1.0 / rho - 1.0;

			for (Size j(i), kij{ next_c(k0i,T(x_t)) }; ++j < n - rank; kij = next_c(kij, T(x_t)))
			{
				double k = tau2[ti] * (s_vv(rank, x + k0i, T(x_t), x + kij, T(x_t)));
				s_va(rank, k, x + k0i, T(x_t), x + kij, T(x_t));
			}
		}

		// step 3:
		// 利用S产生的QT来乘以R1\QT，S事实上为一个很大的矩阵，但其左下角很多为0，因此每一列只用做rank维的乘法
		//
		// x变成:
		//   1           r         m
		// 1 [                     ]
		//   |     ST * (R1\QT)    |
		// r |.....................|
		//   |     S     .         |
		// n [           .         ]  
		//
		//
		for (Size i(-1), k0i{ at(0, rank, T(x_t)) }, ti{ 0 }; ++i < n - rank; k0i = next_c(k0i, T(x_t)), ti = next_r(ti, t_t))
		{
			for (Size j(-1), x0j{ at(0, 0, x_t) }; ++j < m; x0j = next_c(x0j, x_t))
			{
				double alpha = tau2[ti] * (s_vv(rank, x + k0i, T(x_t), x + x0j, x_t));
				s_ma(rank, 1, alpha, x + k0i, T(x_t), x + x0j, x_t);
			}
		}

		// step 4:
		// 利用S产生的Q来乘以ST（R1\QT），这里防止S被覆盖，因此局部会用tau(n-r+1:n)的内存来局部存储
		//
		// x变成:
		//   1                     m
		// 1 [                     ]
		//   |                     |
		//   |   S * ST * (R1\QT)  |
		//   |                     |
		// n [                     ]  
		//
		for (Size i(n - rank), k0i{ at(0, n - 1, T(x_t)) }, ti{ at(n - rank - 1, t_t) }; --i < n - rank; k0i = last_c(k0i, T(x_t)), ti = last_r(ti, t_t))
		{
			// 因为需要少占内存，因此先将
			for (Size j(-1), x0j{ at(0, 0, x_t) }, tj{ at(n - rank,t_t) }; ++j < rank; x0j = next_c(x0j, x_t), tj = next_r(tj, t_t))
			{
				double alpha = tau2[ti] * (s_vv(rank, x + k0i, T(x_t), x + x0j, x_t));
				s_ma(rank, 1, alpha, x + k0i, T(x_t), x + x0j, x_t);
				tau2[tj] = alpha;
			}
			for (Size j(rank - 1), x0j{ at(0, rank, x_t) }; ++j < m; x0j = next_c(x0j, x_t))
			{
				double alpha = tau2[ti] * (s_vv(rank, x + k0i, T(x_t), x + x0j, x_t));
				s_ma(rank, 1, alpha, x + k0i, T(x_t), x + x0j, x_t);
				x[at(i + rank, j, x_t)] = alpha;
			}
			s_vc(rank, tau2 + at(n - rank, t_t), t_t, x + at(i + rank, 0, x_t), T(x_t));
		}

		// step 5:
		// permutate
		s_permutate_inv(n, m, p, x, x_t);
	}
	auto inline s_householder_utp2pinv(Size m, Size n, Size rank, const double *U, const double *tau, const Size *p, double *x, double *tau2, double zero_check = 1e-10)noexcept->void { s_householder_utp2pinv(m, n, rank, U, n, tau, 1, p, x, m, tau2, 1, zero_check); }

	//   
	template<typename AType, typename UType, typename TauType, typename TauType2>
	auto inline s_svd(Size m, Size n, const double *A, AType a_t, double *U, UType u_t, double *tau, TauType tau_t, double *tau2, TauType2 tau2_t, Size *p, double zero_check = 1e-10)noexcept->void
	{
		// init u //
		s_mc(m, n, A, a_t, U, u_t);

		// make u as bidiagnal //
		for (Size i(-1), uii{ 0 }, ti{ 0 }, t2i{ 0 }; ++i < std::min(m,n); uii = next_d(uii, u_t), ti = next_r(ti, tau_t), t2i = next_r(t2i, tau2_t))
		{
			///////////////////////////////////////////第一次变换////////////////////////////////////////////////////
			{
				Size m1 = n;
				Size n1 = m;
				auto u_t1 = T(u_t);

				// 若已经到达最后一行，因为最后一行不需要householder化 //
				if (i + 1 < m1)
				{
					// compute householder vector //
					auto U_i1_i = U + next_r(uii, u_t1);

					double rho = -std::sqrt(s_vv(m1 - i, U + uii, u_t1, U + uii, u_t1)) * s_sgn2(U[uii]);
					tau[ti] = U[uii] / rho - 1.0;
					s_nm(m1 - 1 - i, 1, 1.0 / (U[uii] - rho), U_i1_i, u_t1);
					U[uii] = rho;

					// update matrix //
					for (Size j(i), uij(next_c(uii, u_t1)), tj{ next_r(ti, tau_t) }; ++j < n1; uij = next_c(uij, u_t1), tj = next_r(tj, tau_t))
					{
						auto U_i1_j = U + next_r(uij, u_t1);

						double k = tau[ti] * (s_vv(m1 - i - 1, U_i1_i, u_t1, U_i1_j, u_t1) + U[uij]);
						U[uij] += k;
						s_va(m1 - i - 1, k, U_i1_i, u_t1, U_i1_j, u_t1);
					}
				}
			}
			
			///////////////////////////////////////////第二次变换////////////////////////////////////////////////////
			{
				if (i + 2 < m)
				{
					auto ui1i = next_r(uii, u_t);

					// compute householder vector //
					auto U_i2_i = U + next_r(ui1i, u_t);
					double rho = -std::sqrt(s_vv(m - i - 1, U + ui1i, u_t, U + ui1i, u_t)) * s_sgn2(U[ui1i]);
					tau2[t2i] = U[ui1i] / rho - 1.0;
					s_nm(m - 2 - i, 1, 1.0 / (U[ui1i] - rho), U_i2_i, u_t);
					U[ui1i] = rho;

					// update matrix //
					for (Size j(i), ui1j(next_c(ui1i, u_t)), t2j{ next_r(t2i, tau2_t) }; ++j < n; ui1j = next_c(ui1j, u_t), t2j = next_r(t2j, tau2_t))
					{
						auto U_i2_j = U + next_r(ui1j, u_t);

						double k = tau2[t2i] * (s_vv(m - i - 2, U_i2_i, u_t, U_i2_j, u_t) + U[ui1j]);
						U[ui1j] += k;
						s_va(m - i - 2, k, U_i2_i, u_t, U_i2_j, u_t);
					}
				}
			}
		}

		// solve singular values //
		//std::function<void()> dvc;

		auto dvc = [](Size m, Size n, double f)
		{





		};




	}
	//auto inline s_svd(Size m, Size n, Size rank, const double *U, const double *tau, const Size *p, double *x, double *tau2, double zero_check = 1e-10)noexcept->void { s_svd(m, n, rank, U, n, tau, 1, p, x, m, tau2, 1, zero_check); }


}

#endif
