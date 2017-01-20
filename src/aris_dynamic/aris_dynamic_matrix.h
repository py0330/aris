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
		
		using BlockSize = std::vector<Size>;
		using BlockMatrix = std::vector<std::vector<std::vector<double> > >;

		auto inline vid(Size i, Size ld)->Size { return i*ld; }
		auto inline next_vid(Size id, Size ld)->Size { return id + ld; }

		auto inline id(Size i, Size j, Size ld)->Size { return i*ld + j; }
		auto inline id(Size i, Size j, RowMajor row_major)->Size { return i*row_major.r_ld + j; }
		auto inline id(Size i, Size j, ColMajor col_major)->Size { return i + j*col_major.c_ld; }
		auto inline id(Size i, Size j, Stride stride)->Size { return i*stride.r_ld + j*stride.c_ld; }
		auto inline next_row(Size id, Size ld)->Size { return id + ld; }
		auto inline next_row(Size id, RowMajor row_major)->Size { return id + row_major.r_ld; }
		auto inline next_row(Size id, ColMajor col_major)->Size { return id + 1; }
		auto inline next_row(Size id, Stride stride)->Size { return id + stride.r_ld; }
		auto inline next_col(Size id, Size ld)->Size { return id + 1; }
		auto inline next_col(Size id, RowMajor row_major)->Size { return id + 1; }
		auto inline next_col(Size id, ColMajor col_major)->Size { return id + col_major.c_ld; }
		auto inline next_col(Size id, Stride stride)->Size { return id + stride.c_ld; }
		// transpose matrix type
		auto inline T(Size ld)->ColMajor { return ColMajor{ ld }; }
		auto inline T(RowMajor r)->ColMajor { return ColMajor{ r.r_ld }; }
		auto inline T(ColMajor c)->RowMajor { return RowMajor{ c.c_ld }; }
		auto inline T(Stride s)->Stride { return Stride{ s.c_ld, s.r_ld }; }

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
		auto inline dsp(Size m, Size n, const T *data)->void{	dsp(m, n, data, n);}
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
		auto inline dlmwrite(const char *filename, const BlockMatrix &mtx, const BlockSize &m, const BlockSize &n)->void
		{
			std::ofstream file;

			file.open(filename);

			file << std::setprecision(15);

			for (std::size_t blk_i = 0; blk_i < m.size(); ++blk_i)
			{
				for (Size i = 0; i < m[blk_i]; ++i)
				{
					for (std::size_t blk_j = 0; blk_j < n.size(); ++blk_j)
					{
						for (Size j = 0; j < n[blk_j]; ++j)
						{
							//std::cout << "blk:" << blk_i << "," << blk_j << "  inner:" << i << "," << j << std::endl;

							double a = mtx[blk_i][blk_j].empty() ? 0.0 : mtx[blk_i][blk_j].at(i*n[blk_j] + j);
							file << a << "   ";
						}
					}
					file << std::endl;
				}
			}
		}
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
			for (Size i{ 0 }, x_id{ 0 }; i < n; ++i, x_id = next_vid(x_id, x_t))norm += x[x_id] * x[x_id];
			return std::sqrt(norm);
		}
		auto inline s_norm(Size n, const double *x) noexcept->double { return s_norm(n, x, 1); }
		template<typename XType>
		auto inline s_norm_col(Size m, const double *x, XType x_t) noexcept->double
		{
			double norm = 0;
			for (Size i{ 0 }, x_id{ 0 }; i < m; ++i, x_id = next_row(x_id, x_t))norm += x[x_id] * x[x_id];
			return std::sqrt(norm);
		}
		template<typename XType>
		auto inline s_norm_row(Size n, const double *x, XType x_t) noexcept->double
		{
			double norm = 0;
			for (Size i{ 0 }, x_id{ 0 }; i < n; ++i, x_id = next_col(x_id, x_t))norm += x[x_id] * x[x_id];
			return std::sqrt(norm);
		}
		template<typename XType, typename YType>
		auto inline s_swap_v(Size n, double *x, XType x_t, double *y, YType y_t) noexcept->void
		{
			for (Size i{ 0 }, x_id{ 0 }, y_id{0}; i < n; ++i, x_id = next_vid(x_id, x_t), y_id = next_vid(y_id, y_t))
				std::swap(x[x_id], y[y_id]);
		}
		auto inline s_swap_v(Size n, double *x, double *y) noexcept->void { s_swap_v(n, x, 1, y, 1); }
		// matrix transpose to matrix
		template<typename AType, typename BType>
		auto inline s_transpose(Size m, Size n, const double *A, AType a_t, double *B, BType b_t) noexcept->void
		{
			for (Size i{ 0 }, a_row{ 0 }, b_col{0}; i < m; ++i, a_row = next_row(a_row, a_t), b_col = next_col(b_col, b_t))
				for (Size j{ 0 }, a_id{ a_row }, b_id{ b_col }; j < n; ++j, a_id = next_col(a_id, a_t), b_id = next_row(b_id, b_t))
					B[b_id] = A[a_id];
		}
		auto inline s_transpose(Size m, Size n, const double *A, double *B) noexcept->void { s_transpose(m, n, A, n, B, m); };
		template<typename AType>
		auto inline s_fill(Size m, Size n, double value, double *A, AType a_t) noexcept->void 
		{ 
			for (Size i{ 0 }, a_row{ 0 }; i < m; ++i, a_row = next_row(a_row, a_t)) for (Size j{ 0 }, a_id{ a_row }; j < n; ++j, a_id = next_col(a_id, a_t)) A[a_id] = value;
		}// fill matrix with value
		auto inline s_fill(Size m, Size n, double value, double *A) noexcept->void { std::fill(A, A + m*n, value); }

		template<typename XType>
		auto inline s_nv(Size n, double alpha, double *x, XType x_t) noexcept->void { for (Size i{ 0 }, vid{ 0 }; i < n; ++i, vid = next_vid(vid, x_t))x[vid] *= alpha; }
		auto inline s_nv(Size n, double alpha, double *x) noexcept->void  { for (Size i{ 0 }; i < n; ++i)x[i] *= alpha; }
		template<typename XType, typename YType>
		auto inline s_vc(Size n, const double *x, XType x_t, double *y, YType y_t) noexcept->void { for (Size i{ 0 }, x_id{ 0 }, y_id{ 0 }; i < n; ++i, x_id = next_vid(x_id, x_t), y_id = next_vid(y_id, y_t))y[y_id] = x[x_id]; }
		template<typename XType, typename YType>
		auto inline s_vc(Size n, double alpha, const double *x, XType x_t, double *y, YType y_t) noexcept->void { for (Size i{ 0 }, x_id{ 0 }, y_id{ 0 }; i < n; ++i, x_id = next_vid(x_id, x_t), y_id = next_vid(y_id, y_t))y[y_id] = alpha*x[x_id]; }
		auto inline s_vc(Size n, const double *x, double *y) noexcept->void { std::copy(x, x + n, y); }
		auto inline s_vc(Size n, double alpha, const double *x, double *y) noexcept->void { for (Size i{ 0 }; i < n; ++i)y[i] = alpha*x[i]; }
		template<typename XType, typename YType>
		auto inline s_va(Size n, const double* x, XType x_t, double* y, YType y_t) noexcept->void { for (Size i{ 0 }, x_id{ 0 }, y_id{ 0 }; i < n; ++i, x_id = next_vid(x_id, x_t), y_id = next_vid(y_id, y_t))y[y_id] += x[x_id]; }
		template<typename XType, typename YType>
		auto inline s_va(Size n, double alpha, const double* x, XType x_t, double* y, YType y_t) noexcept->void { for (Size i{ 0 }, x_id{ 0 }, y_id{ 0 }; i < n; ++i, x_id = next_vid(x_id, x_t), y_id = next_vid(y_id, y_t))y[y_id] += alpha*x[x_id]; }
		auto inline s_va(Size n, const double* x, double* y) noexcept->void { for (Size i = 0; i < n; ++i)y[i] += x[i]; }
		auto inline s_va(Size n, double alpha, const double* x, double* y) noexcept->void { for (Size i = 0; i < n; ++i)y[i] += alpha * x[i]; }
		template<typename XType, typename YType>
		auto inline s_vv(Size n, const double *x, XType x_t, const double *y, YType y_t) noexcept->double { double ret{ 0 }; for (Size i{ 0 }, x_id{ 0 }, y_id{ 0 }; i < n; ++i, x_id = next_vid(x_id, x_t), y_id = next_vid(y_id, y_t))ret += x[x_id] * y[y_id]; return ret; }
		auto inline s_vv(Size n, const double *x, const double *y) noexcept->double { double ret{ 0 }; for (Size i = 0; i < n; ++i)ret += x[i] * y[i];	return ret; }
		template<typename AType>
		auto inline s_nm(Size m, Size n, double alpha, double* A, AType a_t) noexcept->void
		{ 
			for (Size i{ 0 }, a_row{0}; i < m; ++i, a_row = next_row(a_row, a_t))for (Size j{ 0 }, a_id{ a_row }; j < n; ++j, a_id = next_col(a_id, a_t))	A[id(i, j, a_t)] *= alpha; 
		}
		auto inline s_nm(Size m, Size n, double alpha, double* A) noexcept->void { s_nv(m*n, alpha, A); }
		template<typename AType, typename BType>
		auto inline s_mc(Size m, Size n, const double *A, AType a_t, double *B, BType b_t) noexcept->void
		{ 
			for (Size i{ 0 }, row_a{ 0 }, row_b{ 0 }; i < m; ++i, row_a = next_row(row_a, a_t), row_b = next_row(row_b, b_t))
				for (Size j{ 0 }, a_id{ row_a }, b_id{ row_b }; j < n; ++j, a_id = next_col(a_id, a_t), b_id = next_col(b_id, b_t))
					B[b_id] = A[a_id];
		}
		template<typename AType, typename BType>
		auto inline s_mc(Size m, Size n, double alpha, const double *A, AType a_t, double *B, BType b_t) noexcept->void
		{
			for (Size i{ 0 }, row_a{ 0 }, row_b{ 0 }; i < m; ++i, row_a = next_row(row_a, a_t), row_b = next_row(row_b, b_t))
				for (Size j{ 0 }, a_id{ row_a }, b_id{ row_b }; j < n; ++j, a_id = next_col(a_id, a_t), b_id = next_col(b_id, b_t))
					B[b_id] = alpha * A[a_id];
		}
		auto inline s_mc(Size m, Size n, const double *A, double *B) noexcept->void { s_vc(m*n, A, B); }
		auto inline s_mc(Size m, Size n, double alpha, const double *A, double *B) noexcept->void { s_vc(m*n, alpha, A, B); }
		auto inline s_mcT(Size m, Size n, const double *A, double *B) noexcept->void { s_mc(m, n, A, ColMajor{ m }, B, n); }
		auto inline s_mcT(Size m, Size n, double alpha, const double *A, double *B) noexcept->void { s_mc(m, n, alpha, A, ColMajor{ m }, B, n); }
		template<typename AType, typename BType>
		auto inline s_ma(Size m, Size n, const double* A, AType a_t, double* B, BType b_t) noexcept->void 
		{ 
			for (Size i{ 0 }, row_a{ 0 }, row_b{ 0 }; i < m; ++i, row_a = next_row(row_a, a_t), row_b = next_row(row_b, b_t))
				for (Size j{ 0 }, a_id{ row_a }, b_id{ row_b }; j < n; ++j, a_id = next_col(a_id, a_t), b_id = next_col(b_id, b_t))
					B[b_id] += A[a_id];
		}
		template<typename AType, typename BType>
		auto inline s_ma(Size m, Size n, double alpha, const double* A, AType a_t, double* B, BType b_t) noexcept->void
		{ 
			for (Size i{ 0 }, row_a{ 0 }, row_b{ 0 }; i < m; ++i, row_a = next_row(row_a, a_t), row_b = next_row(row_b, b_t))
				for (Size j{ 0 }, a_id{ row_a }, b_id{ row_b }; j < n; ++j, a_id = next_col(a_id, a_t), b_id = next_col(b_id, b_t))
					B[b_id] += alpha*A[a_id];
		}
		auto inline s_ma(Size m, Size n, const double* A, double* B) noexcept->void { s_va(m*n, A, B); }
		auto inline s_ma(Size m, Size n, double alpha, const double* A, double* B) noexcept->void { s_va(m*n, alpha, A, B); }
		auto inline s_maT(Size m, Size n, const double* A, double* B) noexcept->void { s_ma(m, n, A, ColMajor{ m }, B, n); }
		auto inline s_maT(Size m, Size n, double alpha, const double* A, double* B) noexcept->void { s_ma(m, n, alpha, A, ColMajor{ m }, B, n); }
		
		template<typename AType, typename BType, typename CType>
		auto inline s_mma(Size m, Size n, Size k, const double* A, AType a_t, const double* B, BType b_t, double *C, CType c_t)->void
		{
			for (Size i{ 0 }, a_row{ 0 }, c_row{ 0 }; i < m; ++i, a_row = next_row(a_row, a_t), c_row = next_row(c_row, c_t))
			{
				for (Size j{ 0 }, b_col{ 0 }, c_id{ c_row }; j < n; ++j, b_col = next_col(b_col, b_t), c_id = next_col(c_id, c_t))
				{
					for (Size u{ 0 }, a_id{ a_row }, b_id{ b_col }; u < k; ++u, a_id = next_col(a_id, a_t), b_id = next_row(b_id, b_t))
						C[c_id] += A[a_id] * B[b_id];
				}
			}
		}
		template<typename AType, typename BType, typename CType>
		auto inline s_mma(Size m, Size n, Size k, double alpha, const double* A, AType a_t, const double* B, BType b_t, double *C, CType c_t)->void
		{
			for (Size i{ 0 }, a_row{ 0 }, c_row{ 0 }; i < m; ++i, a_row = next_row(a_row, a_t), c_row = next_row(c_row, c_t))
			{
				for (Size j{ 0 }, b_col{ 0 }, c_id{ c_row }; j < n; ++j, b_col = next_col(b_col, b_t), c_id = next_col(c_id, c_t))
				{
					double value{ 0 };
					for (Size u{ 0 }, a_id{ a_row }, b_id{ b_col }; u < k; ++u, a_id = next_col(a_id, a_t), b_id = next_row(b_id, b_t))
						value += A[a_id] * B[b_id];
					C[c_id] += alpha*value;
				}
			}
		}
		auto inline s_mma(Size m, Size n, Size k, const double* A, const double* B, double *C) noexcept->void { s_mma(m, n, k, A, k, B, n, C, n); }
		auto inline s_mma(Size m, Size n, Size k, double alpha, const double* A, const double* B, double *C) noexcept->void { s_mma(m, n, k, alpha, A, k, B, n, C, n); }
		auto inline s_mmaTN(Size m, Size n, Size k, const double* A, const double* B, double *C) noexcept->void { s_mma(m, n, k, A, ColMajor{ m }, B, RowMajor{ n }, C, RowMajor{ n }); }
		auto inline s_mmaTN(Size m, Size n, Size k, double alpha, const double* A, const double* B, double *C) noexcept->void { s_mma(m, n, k, alpha, A, ColMajor{ m }, B, RowMajor{ n }, C, RowMajor{ n }); }
		auto inline s_mmaNT(Size m, Size n, Size k, const double* A, const double* B, double *C) noexcept->void { s_mma(m, n, k, A, RowMajor{ k }, B, ColMajor{ k }, C, RowMajor{ n }); }
		auto inline s_mmaNT(Size m, Size n, Size k, double alpha, const double* A, const double* B, double *C) noexcept->void { s_mma(m, n, k, alpha, A, RowMajor{ k }, B, ColMajor{ k }, C, RowMajor{ n }); }
		auto inline s_mmaTT(Size m, Size n, Size k, const double* A, const double* B, double *C) noexcept->void { s_mma(m, n, k, A, ColMajor{ m }, B, ColMajor{ k }, C, RowMajor{ n }); }
		auto inline s_mmaTT(Size m, Size n, Size k, double alpha, const double* A, const double* B, double *C) noexcept->void { s_mma(m, n, k, alpha, A, ColMajor{ m }, B, ColMajor{ k }, C, RowMajor{ n }); }
		template<typename AType, typename BType, typename CType>
		auto inline s_mm(Size m, Size n, Size k, const double* A, AType a_t, const double* B, BType b_t, double *C, CType c_t) noexcept->void { s_fill(m, n, 0.0, C, c_t); s_mma(m, n, k, A, a_t, B, b_t, C, c_t); }
		template<typename AType, typename BType, typename CType>
		auto inline s_mm(Size m, Size n, Size k, double alpha, const double* A, AType a_t, const double* B, BType b_t, double *C, CType c_t) noexcept->void { s_mm(m, n, k, A, a_t, B, b_t, C, c_t);	s_nm(m, n, alpha, C, c_t); }
		auto inline s_mm(Size m, Size n, Size k, const double* A, const double* B, double *C) noexcept->void { s_fill(m, n, 0, C); s_mma(m, n, k, A, B, C); }
		auto inline s_mm(Size m, Size n, Size k, double alpha, const double* A, const double* B, double *C) noexcept->void { s_mm(m, n, k, A, B, C); s_nm(m, n, alpha, C);}
		auto inline s_mmTN(Size m, Size n, Size k, const double* A, const double* B, double *C) noexcept->void { s_fill(m, n, 0, C); s_mmaTN(m, n, k, A, B, C); }
		auto inline s_mmTN(Size m, Size n, Size k, double alpha, const double* A, const double* B, double *C) noexcept->void { s_mmTN(m, n, k, A, B, C); s_nm(m, n, alpha, C); }
		auto inline s_mmNT(Size m, Size n, Size k, const double* A, const double* B, double *C) noexcept->void { s_fill(m, n, 0, C); s_mmaNT(m, n, k, A, B, C); }
		auto inline s_mmNT(Size m, Size n, Size k, double alpha, const double* A, const double* B, double *C) noexcept->void { s_mmNT(m, n, k, A, B, C); s_nm(m, n, alpha, C); }
		auto inline s_mmTT(Size m, Size n, Size k, const double* A, const double* B, double *C) noexcept->void { s_fill(m, n, 0, C); s_mmaTT(m, n, k, A, B, C); }
		auto inline s_mmTT(Size m, Size n, Size k, double alpha, const double* A, const double* B, double *C) noexcept->void { s_mmTT(m, n, k, A, B, C); s_nm(m, n, alpha, C); }


		auto s_blk_make(const double *mtx, const BlockSize &blk_size_m, const BlockSize &blk_size_n, BlockMatrix &blk_mtx) noexcept->void;
		auto s_blk_resolve(const BlockSize &blk_size_m, const BlockSize &blk_size_n, const BlockMatrix &blk_mtx, double *mtx) noexcept->void;
		auto s_blk_allocate(const BlockSize &blk_size_m, const BlockSize &blk_size_n, BlockMatrix &blk_mtx) noexcept->void;
		auto s_blk_check_empty_num(const BlockMatrix &blk_mtx)noexcept->Size;

		auto s_blk_norm(const BlockSize &blk_size_m, const BlockMatrix &blk_mtx)noexcept->double;
		auto s_blk_norm_col(const BlockSize &blk_size_m, const BlockSize &blk_size_n, const BlockMatrix &blk_mtx, Size blk_i, Size blk_j, const double *data)noexcept->double;
		auto s_blk_norm_row(const BlockSize &blk_size_m, const BlockSize &blk_size_n, const BlockMatrix &blk_mtx, Size blk_i, Size blk_j, const double *data)noexcept->double;
		
		auto inline s_blk_nm(const BlockSize &blk_size_m, const BlockSize &blk_size_n, double alpha, BlockMatrix &A) 
		{ 
			for (std::size_t i = 0; i < blk_size_m.size(); ++i)
			{
				for (std::size_t j = 0; j < blk_size_n.size(); ++j)
				{
					if (!A[i][j].empty())s_nm(blk_size_m[i], blk_size_n[j], alpha, A[i][j].data());
				}
			}
		}
		auto s_blk_mm(const BlockSize &blk_size_m, const BlockSize &blk_size_n, const BlockSize &blk_size_k, const BlockMatrix &A, const BlockMatrix &B, BlockMatrix &C)noexcept->void;
		auto s_blk_mmNT(const BlockSize &blk_size_m, const BlockSize &blk_size_n, const BlockSize &blk_size_k, const BlockMatrix &A, const BlockMatrix &B, BlockMatrix &C)noexcept->void;
		auto s_blk_mmTN(const BlockSize &blk_size_m, const BlockSize &blk_size_n, const BlockSize &blk_size_k, const BlockMatrix &A, const BlockMatrix &B, BlockMatrix &C)noexcept->void;
		auto s_blk_mmTT(const BlockSize &blk_size_m, const BlockSize &blk_size_n, const BlockSize &blk_size_k, const BlockMatrix &A, const BlockMatrix &B, BlockMatrix &C)noexcept->void;

		auto s_blk_llt(const BlockSize &blk_size, const BlockMatrix &A, BlockMatrix &L) noexcept->void;
		auto s_blk_sov_lm(const BlockSize &blk_size, const BlockSize &rhs_size, const BlockMatrix &L, const BlockMatrix &b, BlockMatrix &x)noexcept->void;
		auto s_blk_sov_um(const BlockSize &blk_size, const BlockSize &rhs_size, const BlockMatrix &L, const BlockMatrix &b, BlockMatrix &x)noexcept->void;


		// A can be the same as L
		template<typename AType, typename LType>
		auto inline s_llt(Size m, const double *A, AType a_t, double *L, LType l_t) noexcept->void
		{
			for (Size j = 0; j < m; ++j)
			{
				L[id(j, j, l_t)] = A[id(j, j, a_t)];
				for (Size k = 0; k < j; ++k)
				{
					L[id(j, j, l_t)] -= L[id(j, k, l_t)] * L[id(j, k, l_t)];
				}
				L[id(j, j, l_t)] = std::sqrt(L[id(j, j, l_t)]);


				for (Size i = j + 1; i < m; ++i)
				{
					L[id(i, j, l_t)] = A[id(i, j, a_t)];
					for (Size k = 0; k < j; ++k)
					{
						L[id(i, j, l_t)] -= L[id(i, k, l_t)] * L[id(j, k, l_t)];
					}
					L[id(i, j, l_t)] /= L[id(j, j, l_t)];
					L[id(j, i, l_t)] = L[id(i, j, l_t)];
				}
			}
		}
		auto inline s_llt(Size m, const double *A, double *L) noexcept->void { s_llt(m, A, m, L, m); };
		template<typename LType, typename InvLType>
		auto inline s_inv_lm(Size m, const double *L, LType l_t, double *inv_L, InvLType inv_l_t) noexcept->void
		{
			s_fill(m, m, 0.0, inv_L, inv_l_t);

			for (Size j = 0; j < m; ++j)
			{
				inv_L[id(j, j, inv_l_t)] = 1.0;

				for (Size i = j; i < m; ++i)
				{
					for (Size k = 0; k < i; ++k)
					{
						inv_L[id(i, j, inv_l_t)] -= L[id(i, k, l_t)] * inv_L[id(k, j, inv_l_t)];
					}
					inv_L[id(i, j, inv_l_t)] /= L[id(i, i, l_t)];
				}
			}
		}
		auto inline s_inv_lm(Size m, const double *L, double *inv_L) noexcept->void { s_inv_lm(m, L, m, inv_L, m); }
		// b can be the same as x
		template<typename LType, typename bType, typename xType>
		auto inline s_sov_lm(Size m, Size rhs, const double *L, LType l_t, const double *b, bType b_t, double *x, xType x_t) noexcept->void
		{
			for (Size j = 0; j < rhs; ++j)
			{
				for (Size i = 0; i < m; ++i)
				{
					x[id(i, j, x_t)] = b[id(i, j, b_t)];

					for (Size k = 0; k < i; ++k)
					{
						x[id(i, j, x_t)] -= L[id(i, k, l_t)] * x[id(k, j, x_t)];
					}
					x[id(i, j, x_t)] /= L[id(i, i, l_t)];
				}
			}
		}
		auto inline s_sov_lm(Size m, Size rhs, const double *L, const double *b, double *x) noexcept->void { s_sov_lm(m, rhs, L, m, b, rhs, x, rhs); }
		auto inline s_sov_lmNT(Size m, Size rhs, const double *L, const double *b, double *x) noexcept->void { s_sov_lm(m, rhs, L, m, b, ColMajor{ m }, x, rhs); }
		// b can be the same as x
		template<typename LType, typename bType, typename xType>
		auto inline s_sov_um(Size m, Size rhs, const double *L, LType l_t, const double *b, bType b_t, double *x, xType x_t) noexcept->void
		{
			for (Size j = 0; j < rhs; ++j)
			{
				for (Size i = m - 1; i < m; --i)
				{
					x[id(i, j, x_t)] = b[id(i, j, b_t)];

					for (Size k = i + 1; k < m; ++k)
					{
						x[id(i, j, x_t)] -= L[id(i, k, l_t)] * x[id(k, j, x_t)];
					}
					x[id(i, j, x_t)] /= L[id(i, i, l_t)];
				}
			}
		}
		auto inline s_sov_um(Size m, Size rhs, const double *L, const double *b, double *x) noexcept->void { s_sov_um(m, rhs, L, m, b, rhs, x, rhs); }
		auto inline s_sov_umNT(Size m, Size rhs, const double *L, const double *b, double *x) noexcept->void { s_sov_um(m, rhs, L, m, b, ColMajor{ m }, x, rhs); }

		// tau must have same size with max(m,n), A can be the same as U
		template<typename AType, typename UType, typename TauType>
		auto inline s_householder_ut(Size m, Size n, const double *A, AType a_t, double *U, UType u_t, double *tau, TauType tau_t)noexcept->void
		{
			s_mc(m, n, A, a_t, U, u_t);
			for (Size i = 0; i < std::min(m - 1, n); ++i)
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

		auto s_blk_householder_ut(const BlockSize &blk_size, const BlockMatrix &A, BlockMatrix &U, BlockMatrix &tau) noexcept->void;

		auto s_householder_qr(Size m, Size n, const double *A, double *Q, double *R, double *tau)noexcept->void;
		auto s_householder_sov(Size m, Size n, Size rhs, const double *U, const double *tau, double *b, double *x)noexcept->void;
		auto s_householder_colpiv(Size m, Size n, const double *A, double *U, double *tau, Size *P)noexcept->void;
		auto s_householder_colpiv_qr(Size m, Size n, const double *A, double *Q, double *R, double *tau, Size *P)noexcept->void;
	}
}




























#endif
