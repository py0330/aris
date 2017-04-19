#ifndef ARIS_DYNAMIC_BLOCK_MATRIX_
#define ARIS_DYNAMIC_BLOCK_MATRIX_

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
#include <numeric>
#include <type_traits>

#include <aris_dynamic_matrix.h>

namespace aris
{
	namespace dynamic
	{
		template <typename Iter, typename Align>
		struct Block 
		{ 
			Size m, n; 
			Iter data;
			Align align;
			bool is_zero;

			using DataType = typename std::decay<decltype(*data)>::type;
		};
		template <typename Iter, typename Align>
		auto inline Tv(const Block<Iter, Align> &node)->Block<Iter, decltype(T(node.align))>
		{ 
			return Block<Iter, decltype(T(node.align))>{ node.n,node.m, node.data, T(node.align), node.is_zero };
		}

		struct StrideN { Size r_ld, c_ld; };
		struct StrideT { Size r_ld, c_ld; };
		
		auto inline T(StrideN s)->StrideT { return StrideT{ s.c_ld, s.r_ld}; }
		auto inline id(Size i, Size j, StrideN s)->Size { return i*s.r_ld + j*s.c_ld; }
		auto inline next_rid(Size id, StrideN s)->Size { return id + s.r_ld; }
		auto inline next_cid(Size id, StrideN s)->Size { return id + s.c_ld; }
		template<typename AIter>
		auto inline ele(AIter A, StrideN a_t)->decltype(*A) { return *A; }
		template<typename AIter>
		auto inline next(AIter &A, StrideN a_t, Size rows, Size cols)->void { std::advance(A, rows*a_t.r_ld + cols*a_t.c_ld); }
		template<typename AIter>
		auto inline next_row(AIter &A, StrideN a_t)->void { std::advance(A, a_t.r_ld); }
		template<typename AIter>
		auto inline next_col(AIter &A, StrideN a_t)->void { std::advance(A, a_t.c_ld); }
		template<typename AIter>
		auto inline next_diag(AIter &A, StrideN a_t)->void { std::advance(A, a_t.r_ld + a_t.c_ld); }
		template<typename AIter>
		auto inline prev_row(AIter &A, StrideN a_t)->void { std::advance(A, -static_cast<int>(a_t.r_ld)); }
		template<typename AIter>
		auto inline prev_col(AIter &A, StrideN a_t)->void { std::advance(A, -static_cast<int>(a_t.c_ld)); }
		template<typename AIter>
		auto inline prev_diag(AIter &A, StrideN a_t)->void { std::advance(A, -static_cast<int>(a_t.c_ld + a_t.r_ld)); }

		auto inline T(StrideT s)->StrideN { return StrideN{ s.c_ld, s.r_ld }; }
		auto inline id(Size i, Size j, StrideT s)->Size { return i*s.r_ld + j*s.c_ld; }
		auto inline next_rid(Size id, StrideT s)->Size { return id + s.r_ld; }
		auto inline next_cid(Size id, StrideT s)->Size { return id + s.c_ld; }
		template<typename AIter>
		auto inline ele(AIter A, StrideT a_t)->decltype(*A) { return Tv(*A); }
		template<typename AIter>
		auto inline next(AIter &A, StrideT a_t, Size rows, Size cols)->void { std::advance(A, rows*a_t.r_ld + cols*a_t.c_ld); }
		template<typename AIter>
		auto inline next_row(AIter &A, StrideT a_t)->void { std::advance(A, a_t.r_ld); }
		template<typename AIter>
		auto inline next_col(AIter &A, StrideT a_t)->void { std::advance(A, a_t.c_ld); }
		template<typename AIter>
		auto inline next_diag(AIter &A, StrideT a_t)->void { std::advance(A, a_t.r_ld + a_t.c_ld); }
		template<typename AIter>
		auto inline prev_row(AIter &A, StrideT a_t)->void { std::advance(A, -static_cast<int>(a_t.r_ld)); }
		template<typename AIter>
		auto inline prev_col(AIter &A, StrideT a_t)->void { std::advance(A, -static_cast<int>(a_t.c_ld)); }
		template<typename AIter>
		auto inline prev_diag(AIter &A, StrideT a_t)->void { std::advance(A, -static_cast<int>(a_t.c_ld + a_t.r_ld)); }

		auto inline Tv(double &A)->double& { return A; }

		template <typename AType, typename UnaryFunc>
		auto inline for_each_ele(AType &A, UnaryFunc func)
		{
			auto Ai0{ A.data }, Aij{ A.data };
			for (Size i(-1); ++i < A.m; next_row(Ai0, A.align), Aij = Ai0)
			{
				for (Size j(-1); ++j < A.n; next_col(Aij, A.align))
				{
					func(ele(Aij, A.align));
				}
			}
		}
		template <typename AType, typename BType, typename BinaryFunc>
		auto inline for_each_ele(AType &A, BType &B, BinaryFunc func)
		{
			auto Ai0{ A.data }, Aij{ A.data };
			auto Bi0{ B.data }, Bij{ B.data };
			for (Size i(-1); ++i < A.m; next_row(Ai0, A.align), Aij = Ai0, next_row(Bi0, B.align), Bij = Bi0)
			{
				for (Size j(-1); ++j < A.n; next_col(Aij, A.align), next_col(Bij, B.align))
				{
					func(ele(Aij, A.align), ele(Bij, B.align));
				}
			}
		}


		auto inline s_fill(double &A, double value) { A = value; }
		auto inline s_fill_zero(double &A) { A = 0.0; }
		auto inline s_nm(double A, double &B) { B *= A; }
		auto inline s_im(double &A) { A = -A; }
		auto inline s_mc(double A, double &B) { B = A; }
		auto inline s_ma(double A, double &B) { B += A; }
		auto inline s_mi(double A, double &B) { B = -A; }
		auto inline s_ms(double A, double &B) { B -= A; }
		auto inline s_mm(double a, double b, double &c)->void { c = a*b; }
		auto inline s_mma(double a, double b, double &c)->void { c += a*b; }
		auto inline s_mms(double a, double b, double &c)->void { c -= a*b; }
		auto inline s_mmi(double a, double b, double &c)->void { c = -a*b; }
		auto inline s_sov_lm(double L, double b, double &x) { x = b / L; }
		auto inline s_sov_um(double U, double b, double &x) { x = b / U; }
		auto inline s_llt(double A, double &L) { L = std::sqrt(A); }


		template<typename AType, typename ValueType>
		auto s_is_equal(const AType &A, const ValueType &value, const ValueType &error)
		{
			if (A.is_zero && (std::abs(value) <= error)) return true;
			if (A.is_zero) return false;

			auto Ai0{ A.data }, Aij{ A.data };
			for (Size i(-1); ++i < A.m; next_row(Ai0, A.align), Aij = Ai0)
			{
				for (Size j(-1); ++j < A.n; next_col(Aij, A.align))
				{
					if (!s_is_equal(ele(Aij, A.align), value, error))return false;
				}
			}
			return true;
		}
		template<typename AType, typename BType, typename ErrorType>
		auto s_is_equal(const AType &A, const BType &B, const ErrorType &error) 
		{ 
			if (A.is_zero) return s_is_equal(B, ErrorType(0), error);
			else if (B.is_zero) return s_is_equal(A, ErrorType(0), error);
			else
			{
				auto Ai0{ A.data }, Aij{ A.data };
				auto Bi0{ B.data }, Bij{ B.data };
				for (Size i(-1); ++i < A.m; next_row(Ai0, A.align), Aij = Ai0, next_row(Bi0, B.align), Bij = Bi0)
				{
					for (Size j(-1); ++j < A.n; next_col(Aij, A.align), next_col(Bij, B.align))
					{
						if (!s_is_equal(ele(Aij, A.align), ele(Bij, B.align), error))return false;
					}
				}
			}
			return true; 
		}
		template<typename AType, typename ValueType>
		auto s_fill(AType &A, const ValueType &value) noexcept->void
		{
			if (A.is_zero = (value == 0)) return;
			for_each_ele(A, [&value](typename AType::DataType &ele) {s_fill(ele, value); });
		}
		template<typename AType>
		auto s_fill_zero(AType &A){ A.is_zero = true; }
		template<typename ValueType, typename AType>
		auto s_nm(const ValueType &number, AType &A) noexcept->void
		{
			if (A.is_zero = (number == 0 || A.is_zero))return;
			for_each_ele(A, [&number](typename AType::DataType &ele) {	s_nm(number, ele); });
		}
		template<typename AType>
		auto s_im(AType &A) noexcept->void
		{
			if (A.is_zero)return;
			for_each_ele(A, [](typename AType::DataType & ele) { s_im(ele); });
		}
		template<typename AType, typename BType>
		auto s_mc(const AType &A, BType &B) noexcept->void
		{
			if(B.is_zero = A.is_zero)return;
			for_each_ele(A, B, [](typename AType::DataType &A_ele, typename BType::DataType &B_ele) {s_mc(A_ele, B_ele); });
		}
		template<typename AType, typename BType>
		auto s_ma(const AType &A, BType &B) noexcept->void
		{
			if (A.is_zero)return;
			if (B.is_zero) { s_mc(A, B);	return; }
			for_each_ele(A, B, [](typename AType::DataType &A_ele, typename BType::DataType &B_ele) {s_ma(A_ele, B_ele); });
		}
		template<typename AType, typename BType>
		auto s_mi(const AType &A, BType &B) noexcept->void
		{
			// set and check if A.is_zero is true, note here operator '=' is correct
			if (B.is_zero = A.is_zero)return;
			for_each_ele(A, B, [](typename AType::DataType &A_ele, typename BType::DataType &B_ele) {s_mi(A_ele, B_ele); });
		}
		template<typename AType, typename BType>
		auto s_ms(const AType &A, BType &B) noexcept->void
		{
			if (A.is_zero)return;
			if (B.is_zero) { s_mi(A, B);	return; }
			for_each_ele(A, B, [](typename AType::DataType &A_ele, typename BType::DataType &B_ele) {s_ms(A_ele, B_ele); });
		}
		template<typename AType, typename BType, typename CType>
		auto s_mma(const AType &A, const BType &B, CType &C)->void
		{
			if (A.is_zero || B.is_zero)return;
			
			auto Ai0{ A.data }, Aik{ A.data };
			auto B0j{ B.data }, Bkj{ B.data };
			auto Ci0{ C.data }, Cij{ C.data };
			
			if (C.is_zero)
			{
				for (Size i(-1); ++i < A.m; next_row(Ai0, A.align), Aik = Ai0, B0j = B.data, Bkj = B0j, next_row(Ci0, C.align), Cij = Ci0)
				{
					for (Size j(-1); ++j < B.n; Aik = Ai0, next_col(B0j, B.align), Bkj = B0j, next_col(Cij, C.align))
					{
						s_fill_zero(ele(Cij, C.align));
						for (Size u(-1); ++u < A.n; next_col(Aik, A.align), next_row(Bkj, B.align))
							s_mma(ele(Aik, A.align), ele(Bkj, B.align), ele(Cij, C.align));
					}
				}
			}
			else
			{
				for (Size i(-1); ++i < A.m; next_row(Ai0, A.align), Aik = Ai0, B0j = B.data, Bkj = B0j, next_row(Ci0, C.align), Cij = Ci0)
				{
					for (Size j(-1); ++j < B.n; Aik = Ai0, next_col(B0j, B.align), Bkj = B0j, next_col(Cij, C.align))
					{
						for (Size u(-1); ++u < A.n; next_col(Aik, A.align), next_row(Bkj, B.align))
							s_mma(ele(Aik, A.align), ele(Bkj, B.align), ele(Cij, C.align));
					}
				}
			}

			C.is_zero = false;
		}
		template<typename ValueType, typename AType, typename BType, typename CType, typename std::enable_if<std::is_same<ValueType, typename AType::DataType>::value>::type* = nullptr>
		auto s_mma(const ValueType& alpha, const AType &A, const BType &B, CType &C)->void
		{
			if (A.is_zero || B.is_zero)return;

			auto Ai0{ A.data }, Aik{ A.data };
			auto B0j{ B.data }, Bkj{ B.data };
			auto Ci0{ C.data }, Cij{ C.data };

			if (C.is_zero)
			{
				for (Size i(-1); ++i < A.m; next_row(Ai0, A.align), Aik = Ai0, B0j = B.data, Bkj = B0j, next_row(Ci0, C.align), Cij = Ci0)
				{
					for (Size j(-1); ++j < B.n; Aik = Ai0, next_col(B0j, B.align), Bkj = B0j, next_col(Cij, C.align))
					{
						s_fill_zero(ele(Cij, C.align));
						ValueType value(0);
						for (Size u(-1); ++u < A.n; next_col(Aik, A.align), next_row(Bkj, B.align))
							s_mma(ele(Aik, A.align), ele(Bkj, B.align), value);
						ele(Cij, C.align) += alpha * value;
					}
				}
			}
			else
			{
				for (Size i(-1); ++i < A.m; next_row(Ai0, A.align), Aik = Ai0, B0j = B.data, Bkj = B0j, next_row(Ci0, C.align), Cij = Ci0)
				{
					for (Size j(-1); ++j < B.n; Aik = Ai0, next_col(B0j, B.align), Bkj = B0j, next_col(Cij, C.align))
					{
						ValueType value(0);
						for (Size u(-1); ++u < A.n; next_col(Aik, A.align), next_row(Bkj, B.align))
							s_mma(ele(Aik, A.align), ele(Bkj, B.align), value);
						ele(Cij, C.align) += alpha * value;
					}
				}
			}

			C.is_zero = false;
		}
		template<typename ValueType, typename AType, typename BType, typename CType, typename std::enable_if<!std::is_same<ValueType, typename AType::DataType>::value>::type* = nullptr>
		auto s_mma(const ValueType& alpha, const AType &A, const BType &B, CType &C)->void
		{
			if (A.is_zero || B.is_zero)return;

			auto Ai0{ A.data }, Aik{ A.data };
			auto B0j{ B.data }, Bkj{ B.data };
			auto Ci0{ C.data }, Cij{ C.data };

			if (C.is_zero)
			{
				for (Size i(-1); ++i < A.m; next_row(Ai0, A.align), Aik = Ai0, B0j = B.data, Bkj = B0j, next_row(Ci0, C.align), Cij = Ci0)
				{
					for (Size j(-1); ++j < B.n; Aik = Ai0, next_col(B0j, B.align), Bkj = B0j, next_col(Cij, C.align))
					{
						s_fill_zero(ele(Cij, C.align));
						for (Size u(-1); ++u < A.n; next_col(Aik, A.align), next_row(Bkj, B.align))
							s_mma(alpha, ele(Aik, A.align), ele(Bkj, B.align), ele(Cij, C.align));
					}
				}
			}
			else
			{
				for (Size i(-1); ++i < A.m; next_row(Ai0, A.align), Aik = Ai0, B0j = B.data, Bkj = B0j, next_row(Ci0, C.align), Cij = Ci0)
				{
					for (Size j(-1); ++j < B.n; Aik = Ai0, next_col(B0j, B.align), Bkj = B0j, next_col(Cij, C.align))
					{
						for (Size u(-1); ++u < A.n; next_col(Aik, A.align), next_row(Bkj, B.align))
							s_mma(alpha, ele(Aik, A.align), ele(Bkj, B.align), ele(Cij, C.align));
					}
				}
			}

			C.is_zero = false;
		}
		template<typename AType, typename BType, typename CType>
		auto s_mms(const AType &A, const BType &B, CType &C)->void
		{
			if (A.is_zero || B.is_zero)return;

			auto Ai0{ A.data }, Aik{ A.data };
			auto B0j{ B.data }, Bkj{ B.data };
			auto Ci0{ C.data }, Cij{ C.data };

			if (C.is_zero)
			{
				for (Size i(-1); ++i < A.m; next_row(Ai0, A.align), Aik = Ai0, B0j = B.data, Bkj = B0j, next_row(Ci0, C.align), Cij = Ci0)
				{
					for (Size j(-1); ++j < B.n; Aik = Ai0, next_col(B0j, B.align), Bkj = B0j, next_col(Cij, C.align))
					{
						s_fill_zero(ele(Cij, C.align));
						for (Size u(-1); ++u < A.n; next_col(Aik, A.align), next_row(Bkj, B.align))
							s_mms(ele(Aik, A.align), ele(Bkj, B.align), ele(Cij, C.align));
					}
				}
			}
			else
			{
				for (Size i(-1); ++i < A.m; next_row(Ai0, A.align), Aik = Ai0, B0j = B.data, Bkj = B0j, next_row(Ci0, C.align), Cij = Ci0)
				{
					for (Size j(-1); ++j < B.n; Aik = Ai0, next_col(B0j, B.align), Bkj = B0j, next_col(Cij, C.align))
					{
						for (Size u(-1); ++u < A.n; next_col(Aik, A.align), next_row(Bkj, B.align))
							s_mms(ele(Aik, A.align), ele(Bkj, B.align), ele(Cij, C.align));
					}
				}
			}

			C.is_zero = false;
		}
		template<typename ValueType, typename AType, typename BType, typename CType>
		auto inline s_mms(const ValueType& alpha, const AType &A, const BType &B, CType &C)->void{ s_mma(-alpha, A, B, C);}
		template<typename AType, typename BType, typename CType>
		auto inline s_mmi(const AType &A, const BType &B, CType &C)->void
		{
			C.is_zero = true;
			s_mms(A, B, C);
		}
		template<typename ValueType, typename AType, typename BType, typename CType>
		auto inline s_mmi(const ValueType& alpha, const AType &A, const BType &B, CType &C)->void
		{
			C.is_zero = true;
			s_mms(alpha, A, B, C);
		}
		template<typename AType, typename BType, typename CType>
		auto inline s_mm(const AType &A, const BType &B, CType &C)->void
		{
			C.is_zero = true;
			s_mma(A, B, C);
		}
		template<typename ValueType, typename AType, typename BType, typename CType>
		auto inline s_mm(const ValueType& alpha, const AType &A, const BType &B, CType &C)->void
		{
			C.is_zero = true;
			s_mma(alpha, A, B, C);
		}
		
		template <typename LType, typename BType, typename XType>
		auto s_sov_lm(LType &L, BType& B, XType& X)noexcept->void
		{
			auto X0j{ X.data };
			auto B0j{ B.data };
			for (Size j(-1); ++j < X.n; next_col(X0j, X.align), next_col(B0j, B.align))
			{
				auto Li0{ L.data };
				auto Xij{ X0j };
				auto Bij{ B0j };
				for (Size i(-1); ++i < L.m; next_row(Xij, X.align), next_row(Bij, B.align), next_row(Li0, L.align))
				{
					s_mc(ele(Bij, B.align), ele(Xij, X.align));

					auto Lik{ Li0 };
					auto Xkj{ X0j };
					for (Size k(-1); ++k < i; next_col(Lik, L.align), next_row(Xkj, X.align))
					{
						s_mms(ele(Lik, L.align), ele(Xkj, X.align), ele(Xij, X.align));
					}

					auto Lii{ Lik };
					s_sov_lm(ele(Lii,L.align), ele(Xij, X.align), ele(Xij, X.align));
				}
			}
		}
		template <typename UType, typename BType, typename XType>
		auto s_sov_um(UType &U, BType& B, XType& X)noexcept->void
		{
			auto Xmj{ X.data };
			auto Bmj{ B.data };
			next(Xmj, X.align, X.m - 1, 0);
			next(Bmj, B.align, B.m - 1, 0);
			for (Size j(-1); ++j < X.n; next_col(Xmj, X.align), next_col(Bmj, B.align))
			{
				auto Uim{ U.data };
				auto Xij{ Xmj };
				auto Bij{ Bmj };
				next(Uim, U.align, U.m - 1, U.m - 1);
				for (Size i(U.m); --i < U.m; prev_row(Xij, X.align), prev_row(Bij, B.align), prev_row(Uim, U.align))
				{
					s_mc(ele(Bij, B.align), ele(Xij, X.align));

					auto Uik{ Uim };
					auto Xkj{ Xmj };
					for (Size k(i); ++k < U.m; prev_col(Uik, U.align), prev_row(Xkj, X.align))
					{
						s_mms(ele(Uik, U.align), ele(Xkj, X.align), ele(Xij, X.align));
					}

					auto Uii{ Uik };
					s_sov_um(ele(Uii, U.align), ele(Xij, X.align), ele(Xij, X.align));
				}
			}
		}
		template<typename AType, typename LltType>
		auto s_llt(const AType &A, LltType &L)->void
		{
			auto Ajj{ A.data };
			auto Ljj{ L.data }, Lj0{ L.data };

			for (Size j(-1); ++j < A.m; next_row(Ajj, A.align), next_col(Ajj, A.align), next_row(Ljj, L.align), next_col(Ljj, L.align), next_row(Lj0, L.align))
			{
				s_mc(ele(Ajj, L.align), ele(Ljj, L.align));
				auto Ljk{ Lj0 };
				for (Size k(-1); ++k < j; next_col(Ljk, L.align))
				{
					s_mms(ele(Ljk, L.align), Tv(ele(Ljk, L.align)), ele(Ljj, L.align));
				}
				s_llt(ele(Ljj, L.align), ele(Ljj, L.align));

				auto Aij{ Ajj };
				next_row(Aij, A.align);
				auto Li0{ Lj0 }, Lji{ Ljj }, Lij{ Ljj };
				next_row(Li0, L.align);
				next_col(Lji, L.align);
				next_row(Lij, L.align);
				for (Size i(j); ++i < A.m; next_row(Li0, L.align), next_col(Lji, L.align), next_row(Lij, L.align), next_row(Aij, A.align))
				{
					s_mc(ele(Aij, A.align), ele(Lij, L.align));

					auto Lik{ Li0 }, Ljk{ Lj0 };
					for (Size k(-1); ++k < j; next_col(Lik, L.align), next_col(Ljk, L.align))
					{
						s_mms(ele(Lik, L.align), Tv(ele(Ljk, L.align)), ele(Lij, L.align));
					}
					s_sov_lm(ele(Ljj, L.align), ele(Lij, L.align), ele(Lij, L.align));
					s_mc(Tv(ele(Lij, L.align)), ele(Lji, L.align));
				}
			}
		}
		
		template<typename AType, typename UType, typename TType>
		auto s_householder_ut(const AType &A, UType &U, TType &T)
		{
			s_mc(A, U);

			auto Aii{ A.data };
			for (Size i(-1); ++i < U.m; next_diag(Aii, A.align))
			{
				//for(Size bi(-1); ++i<U.)
				
				
				//// compute householder vector //
				//double rho = -s_norm_col(m - i, U + id(i, i, u_t), u_t) * s_sgn2(U[id(i, i, u_t)]);
				//tau[id(i, 0, tau_t)] = U[id(i, i, u_t)] / rho - 1.0;
				//s_nv(m - 1 - i, 1.0 / (U[id(i, i, u_t)] - rho), U + id(i + 1, i, u_t), u_t);
				//U[id(i, i, u_t)] = rho;

				//// update matrix //
				//s_mc(1, n - i - 1, U + id(i, i + 1, u_t), u_t, tau + id(i + 1, 0, tau_t), T(tau_t));
				//s_mma(1, n - i - 1, m - i - 1, U + id(i + 1, i, u_t), T(u_t), U + id(i + 1, i + 1, u_t), u_t, tau + id(i + 1, 0, tau_t), T(tau_t));
				//s_nm(n - i - 1, 1, tau[id(i, 0, tau_t)], tau + id(i + 1, 0, tau_t), tau_t);

				//s_ma(n - i - 1, 1, tau + id(i + 1, 0, tau_t), tau_t, U + id(i, i + 1, u_t), T(u_t));
				//s_mma(m - i - 1, n - i - 1, 1, U + id(i + 1, i, u_t), u_t, tau + id(i + 1, 0, tau_t), T(tau_t), U + id(i + 1, i + 1, u_t), u_t);
			}
		}


		template <typename Container>
		auto inline count_size(Container &c)->Size
		{
			Size size{0};
			for (auto &ele:c)size += count_size(ele);
			return size;
		}
		auto inline count_size(Size s)->Size { return s; }
		
		//template <typename Iter, typename Align, typename AType, typename MContainer, typename NContainer>
		//auto s_blk_map(Iter data, Align align, AType &A, MContainer m, NContainer n)
		//{
		//	Size rows{ 0 };
		//	auto Ai0{ A.data };
		//	for (auto &ms : m)
		//	{
		//		Size cols{ 0 };
		//		auto Aij{ Ai0 };
		//		for (auto &ns : n)
		//		{
		//			s_blk_map


		//			next_col(Aij, A.align);
		//			cols += count_size(ns);
		//		}

		//		next_row(Ai0, A.align);
		//		rows += count_size(ms);
		//	}
		//}




		/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		struct BlockStride 
		{ 
			union
			{
				struct { Size r_ld, c_ld; };
				Stride self_stride;
			};
			union
			{
				struct { Size blk_r_ld, blk_c_ld; };
				Stride block_stride;
			};
		};
		struct BlockNode { double * data; bool is_zero; };
		using BlockSize = std::vector<Size>;
		using BlockData = std::vector<BlockNode>;






		auto inline T(BlockStride blk_s)->BlockStride { return BlockStride{ blk_s.c_ld, blk_s.r_ld, blk_s.blk_c_ld, blk_s.blk_r_ld }; }
		auto inline id(Size i, Size j, BlockStride blk_s)->Size { return id(i,j,blk_s.self_stride); }
		auto inline next_rid(Size id, BlockStride blk_s)->Size { return next_rid(id, blk_s.self_stride); }
		auto inline next_cid(Size id, BlockStride blk_s)->Size { return next_cid(id, blk_s.self_stride); }

		template <typename AType, typename BType>
		auto inline s_blk_map(const BlockSize &blk_m, const BlockSize &blk_n, double *A, AType a_t, BlockData &blk_A, BType blk_a_t)noexcept->void
		{
			for (Size blk_i(-1), i{ 0 }; ++blk_i < blk_m.size(); i += blk_m[blk_i])
			{
				for (Size blk_j(-1), j{ 0 }; ++blk_j < blk_n.size(); j += blk_n[blk_j])
				{
					blk_A[id(blk_i, blk_j, blk_a_t)] = BlockNode{ A + id(i,j,a_t), false };
				}
			}
		}
		auto inline s_blk_map(const BlockSize &blk_m, const BlockSize &blk_n, double *A, BlockData &blk_A)noexcept->void
		{
			s_blk_map(blk_m, blk_n, A, std::accumulate(blk_n.begin(), blk_n.end(), Size(0)), blk_A, blk_n.size());
		}
		
		template<typename AType, typename BType>
		auto inline s_blk_mc(BlockSize m, BlockSize n, const BlockData &A, AType a_t, BlockData &B, BType b_t) noexcept->void
		{
			for (Size i(-1), row_a{ 0 }, row_b{ 0 }; ++i < m.size(); row_a = next_rid(row_a, a_t), row_b = next_rid(row_b, b_t))
				for (Size j(-1), a_id{ row_a }, b_id{ row_b }; ++j < n.size(); a_id = next_cid(a_id, a_t), b_id = next_cid(b_id, b_t))
				{
					B[b_id].is_zero = A[a_id].is_zero;
					if (!B[b_id].is_zero)s_mc(m[i], n[j], A[id(i, j, a_t)].data, a_t.block_stride, B[id(i, j, b_t)].data, b_t.block_stride);
				}
					
		}
		//template<typename AType, typename BType>
		//auto inline s_blk_mc(Size m, Size n, double alpha, const double *A, AType a_t, double *B, BType b_t) noexcept->void
		//{
		//	for (Size i(-1), row_a{ 0 }, row_b{ 0 }; ++i < m; row_a = next_rid(row_a, a_t), row_b = next_rid(row_b, b_t))
		//		for (Size j(-1), a_id{ row_a }, b_id{ row_b }; ++j < n; a_id = next_cid(a_id, a_t), b_id = next_cid(b_id, b_t))
		//			B[b_id] = alpha * A[a_id];
		//}
		//auto inline s_blk_mc(Size m, Size n, const double *A, double *B) noexcept->void { s_vc(m*n, A, B); }
		//auto inline s_blk_mc(Size m, Size n, double alpha, const double *A, double *B) noexcept->void { s_vc(m*n, alpha, A, B); }


		template <typename AType>
		auto s_blk_norm_fro(const BlockSize &blk_m, const BlockSize &blk_n, const BlockData &blk_A, AType a_t)noexcept->double
		{
			double norm{ 0 };

			for (Size i{ 0 }, blk_row{ 0 }; i < blk_m.size(); ++i, blk_row = next_rid(blk_row, a_t))
			{
				for (Size j{ 0 }, blk_id{ blk_row }; j < blk_n.size(); ++j, blk_id = next_cid(blk_id, a_t))
				{
					for (Size inner_i{ 0 }, row_id{ 0 }; inner_i < blk_m[i]; ++inner_i, row_id = next_rid(row_id, a_t.block_stride))
					{
						for (Size inner_j{ 0 }, ele_id{ row_id }; inner_j < blk_n[j]; ++inner_j, ele_id = next_cid(ele_id, a_t.block_stride))
						{
							norm += blk_A[blk_id].data[ele_id] * blk_A[blk_id].data[ele_id];
						}
					}
				}
			}

			return std::sqrt(norm);
		}
		template <typename AType>
		auto s_blk_norm_1(const BlockSize &blk_m, const BlockSize &blk_n, const BlockData &blk_A, AType a_t)noexcept->double
		{
			double max_norm{ 0 };

			for (Size j{ 0 }, blk_col{ 0 }; j < blk_n.size(); ++j, blk_col = next_cid(blk_col, a_t))
			{
				for (Size inner_j{ 0 }, col_id{ 0 }; inner_j < blk_n[j]; ++inner_j, col_id = next_cid(col_id, a_t.block_stride))
				{
					double norm{ 0 };

					for (Size i{ 0 }, blk_id{ blk_col }; i < blk_m.size(); ++i, blk_id = next_rid(blk_id, a_t))
					{
						for (Size inner_i{ 0 }, ele_id{ col_id }; inner_i < blk_m[i]; ++inner_i, ele_id = next_rid(ele_id, a_t.block_stride))
						{
							norm += std::abs(blk_A[blk_id].data[ele_id]);
						}
					}

					max_norm = std::max(norm, max_norm);
				}
			}

			return max_norm;
		}
		template <typename AType>
		auto s_blk_norm_inf(const BlockSize &blk_m, const BlockSize &blk_n, const BlockData &blk_A, AType a_t)noexcept->double
		{
			double max_norm{ 0 };

			for (Size i{ 0 }, blk_row{ 0 }; i < blk_m.size(); ++i, blk_row = next_rid(blk_row, a_t))
			{
				for (Size inner_i{ 0 }, row_id{ 0 }; inner_i < blk_m[i]; ++inner_i, row_id = next_rid(row_id, a_t.block_stride))
				{
					double norm{ 0 };

					for (Size j{ 0 }, blk_id{ blk_row }; j < blk_n.size(); ++j, blk_id = next_cid(blk_id, a_t))
					{
						for (Size inner_j{ 0 }, ele_id{ row_id }; inner_j < blk_n[j]; ++inner_j, ele_id = next_cid(ele_id, a_t.block_stride))
						{
							norm += std::abs(blk_A[blk_id].data[ele_id]);
						}
					}

					max_norm = std::max(norm, max_norm);
				}
			}

			return max_norm;
		}

		template <typename AType, typename BType, typename CType>
		auto inline s_blk_mma(const BlockSize &blk_m, const BlockSize &blk_n, const BlockSize &blk_k, const BlockData &A, AType a_t, const BlockData &B, BType b_t, BlockData &C, CType c_t)noexcept->void
		{
			for (Size i(-1), a_row{ 0 }, c_row{ 0 }; ++i < blk_m.size(); a_row = next_rid(a_row, a_t), c_row = next_rid(c_row, c_t))
			{
				for (Size j(-1), b_col{ 0 }, c_id{ c_row }; ++j < blk_n.size(); b_col = next_cid(b_col, b_t), c_id = next_cid(c_id, c_t))
				{
					for (Size u(-1), a_id{ a_row }, b_id{ b_col }; ++u < blk_k.size(); a_id = next_cid(a_id, a_t), b_id = next_rid(b_id, b_t))
					{
						if (!(A[a_id].is_zero || B[b_id].is_zero))
						{
							if (C[c_id].is_zero)
							{
								C[c_id].is_zero = false;
								s_fill(blk_m[i], blk_n[j], 0.0, C[c_id].data, c_t.block_stride);
							}

							s_mma(blk_m[i], blk_n[j], blk_k[u], A[a_id].data, a_t.block_stride, B[b_id].data, b_t.block_stride, C[c_id].data, c_t.block_stride);
						}
					}
				}
			}
		}
		template <typename AType, typename BType, typename CType>
		auto inline s_blk_mma(const BlockSize &blk_m, const BlockSize &blk_n, const BlockSize &blk_k, double alpha, const BlockData &A, AType a_t, const BlockData &B, BType b_t, BlockData &C, CType c_t)noexcept->void
		{
			for (Size i(-1), a_row{ 0 }, c_row{ 0 }; ++i < blk_m.size(); a_row = next_rid(a_row, a_t), c_row = next_rid(c_row, c_t))
			{
				for (Size j(-1), b_col{ 0 }, c_id{ c_row }; ++j < blk_n.size(); b_col = next_cid(b_col, b_t), c_id = next_cid(c_id, c_t))
				{
					for (Size u(-1), a_id{ a_row }, b_id{ b_col }; ++u < blk_k.size(); a_id = next_cid(a_id, a_t), b_id = next_rid(b_id, b_t))
					{
						if (!(A[a_id].is_zero || B[b_id].is_zero))
						{
							if (C[c_id].is_zero)
							{
								C[c_id].is_zero = false;
								s_fill(blk_m[i], blk_n[j], 0.0, C[c_id].data, c_t.block_stride);
							}

							s_mma(blk_m[i], blk_n[j], blk_k[u], alpha, A[a_id].data, a_t.block_stride, B[b_id].data, b_t.block_stride, C[c_id].data, c_t.block_stride);
						}
					}
				}
			}
		}
		auto inline s_blk_mma(const BlockSize &blk_m, const BlockSize &blk_n, const BlockSize &blk_k, const BlockData &A, const BlockData &B, BlockData &C)noexcept->void
		{
			s_blk_mma(blk_m, blk_n, blk_k, A, BlockStride{ blk_k.size(),1,std::accumulate(blk_k.begin(),blk_k.end(),Size(0)),1 },
				B, BlockStride{ blk_n.size(),1,std::accumulate(blk_n.begin(),blk_n.end(),Size(0)),1 },
				C, BlockStride{ blk_n.size(),1,std::accumulate(blk_n.begin(),blk_n.end(),Size(0)),1 });
		}
		auto inline s_blk_mma(const BlockSize &blk_m, const BlockSize &blk_n, const BlockSize &blk_k, double alpha, const BlockData &A, const BlockData &B, BlockData &C)noexcept->void
		{
			s_blk_mma(blk_m, blk_n, blk_k, alpha, A, BlockStride{ blk_k.size(),1,std::accumulate(blk_k.begin(),blk_k.end(),Size(0)),1 },
				B, BlockStride{ blk_n.size(),1,std::accumulate(blk_n.begin(),blk_n.end(),Size(0)),1 },
				C, BlockStride{ blk_n.size(),1,std::accumulate(blk_n.begin(),blk_n.end(),Size(0)),1 });
		}
		template <typename AType, typename BType, typename CType>
		auto inline s_blk_mm(const BlockSize &blk_m, const BlockSize &blk_n, const BlockSize &blk_k, const BlockData &A, AType a_t, const BlockData &B, BType b_t, BlockData &C, CType c_t)noexcept->void
		{
			for (Size i(-1), a_row{ 0 }, c_row{ 0 }; ++i < blk_m.size(); a_row = next_rid(a_row, a_t), c_row = next_rid(c_row, c_t))
			{
				for (Size j(-1), b_col{ 0 }, c_id{ c_row }; ++j < blk_n.size(); b_col = next_cid(b_col, b_t), c_id = next_cid(c_id, c_t))
				{
					C[c_id].is_zero = true;
					for (Size u(-1), a_id{ a_row }, b_id{ b_col }; ++u < blk_k.size(); a_id = next_cid(a_id, a_t), b_id = next_rid(b_id, b_t))
					{
						if (!(A[a_id].is_zero || B[b_id].is_zero))
						{
							if (C[c_id].is_zero)
							{
								C[c_id].is_zero = false;
								s_fill(blk_m[i], blk_n[j], 0.0, C[c_id].data, c_t.block_stride);
							}
							
							s_mma(blk_m[i], blk_n[j], blk_k[u], A[a_id].data, a_t.block_stride, B[b_id].data, b_t.block_stride, C[c_id].data, c_t.block_stride);
						}
					}
				}
			}
		}
		template <typename AType, typename BType, typename CType>
		auto inline s_blk_mm(const BlockSize &blk_m, const BlockSize &blk_n, const BlockSize &blk_k, double alpha, const BlockData &A, AType a_t, const BlockData &B, BType b_t, BlockData &C, CType c_t)noexcept->void
		{
			for (Size i(-1), a_row{ 0 }, c_row{ 0 }; ++i < blk_m.size(); a_row = next_rid(a_row, a_t), c_row = next_rid(c_row, c_t))
			{
				for (Size j(-1), b_col{ 0 }, c_id{ c_row }; ++j < blk_n.size(); b_col = next_cid(b_col, b_t), c_id = next_cid(c_id, c_t))
				{
					C[c_id].is_zero = true;
					for (Size u(-1), a_id{ a_row }, b_id{ b_col }; ++u < blk_k.size(); a_id = next_cid(a_id, a_t), b_id = next_rid(b_id, b_t))
					{
						if (!(A[a_id].is_zero || B[b_id].is_zero))
						{
							if (C[c_id].is_zero)
							{
								C[c_id].is_zero = false;
								s_fill(blk_m[i], blk_n[j], 0.0, C[c_id].data, c_t.block_stride);
							}

							s_mma(blk_m[i], blk_n[j], blk_k[u], alpha, A[a_id].data, a_t.block_stride, B[b_id].data, b_t.block_stride, C[c_id].data, c_t.block_stride);
						}
					}
				}
			}
		}
		auto inline s_blk_mm(const BlockSize &blk_m, const BlockSize &blk_n, const BlockSize &blk_k, const BlockData &A, const BlockData &B, BlockData &C)noexcept->void
		{
			s_blk_mm(blk_m, blk_n, blk_k, A, BlockStride{ blk_k.size(),1,std::accumulate(blk_k.begin(),blk_k.end(),Size(0)),1 },
				B, BlockStride{ blk_n.size(),1,std::accumulate(blk_n.begin(),blk_n.end(),Size(0)),1 },
				C, BlockStride{ blk_n.size(),1,std::accumulate(blk_n.begin(),blk_n.end(),Size(0)),1 });
		}
		auto inline s_blk_mm(const BlockSize &blk_m, const BlockSize &blk_n, const BlockSize &blk_k, double alpha, const BlockData &A, const BlockData &B, BlockData &C)noexcept->void
		{
			s_blk_mm(blk_m, blk_n, blk_k, alpha, A, BlockStride{ blk_k.size(),1,std::accumulate(blk_k.begin(),blk_k.end(),Size(0)),1 },
				B, BlockStride{ blk_n.size(),1,std::accumulate(blk_n.begin(),blk_n.end(),Size(0)),1 },
				C, BlockStride{ blk_n.size(),1,std::accumulate(blk_n.begin(),blk_n.end(),Size(0)),1 });
		}

		template <typename AType, typename LltType>
		auto s_blk_llt(const BlockSize &blk_m, const BlockData &blk_A, AType a_t, BlockData &blk_llt, LltType l_t) noexcept->void
		{
			for (Size j(-1), a_jj{ 0 }, l_jj{ 0 }, l_j0{ 0 }; ++j < blk_m.size(); a_jj = next_rid(next_cid(a_jj, a_t), a_t), l_jj = next_rid(next_cid(l_jj, l_t), l_t), l_j0 = next_rid(l_j0, l_t))
			{
				s_mc(blk_m[j], blk_m[j], blk_A[a_jj].data, a_t.block_stride, blk_llt[l_jj].data, l_t.block_stride);
				blk_llt[l_jj].is_zero = false;

				for (Size k(-1), l_jk{ l_j0 }; ++k < j; l_jk = next_cid(l_jk, l_t))
				{
					if (!blk_llt[l_jk].is_zero)s_mma(blk_m[j], blk_m[j], blk_m[k], -1.0, blk_llt[l_jk].data, l_t.block_stride, blk_llt[l_jk].data, T(l_t.block_stride), blk_llt[l_jj].data, l_t.block_stride);
				}

				s_llt(blk_m[j], blk_llt[l_jj].data, l_t.block_stride, blk_llt[l_jj].data, l_t.block_stride);

				for (Size i(j), l_i0{ next_rid(l_j0,l_t) }, l_ji{ next_cid(l_jj, l_t) }, l_ij{ next_rid(l_jj, l_t) }, a_ij{ next_rid(a_jj,a_t) }; ++i < blk_m.size(); l_i0 = next_rid(l_i0, l_t), l_ji = next_cid(l_ji, l_t), l_ij = next_rid(l_ij, l_t), a_ij = next_rid(a_ij, a_t))
				{
					blk_llt[l_ij].is_zero = blk_A[a_ij].is_zero;
					s_mc(blk_m[i], blk_m[j], blk_A[a_ij].data, a_t.block_stride, blk_llt[l_ij].data, l_t.block_stride);

					for (Size k(-1), l_ik{ l_i0 }, l_jk{ l_j0 }; ++k < j; l_ik = next_cid(l_ik, l_t), l_jk = next_cid(l_jk, l_t))
					{
						if (blk_llt[l_ik].is_zero || blk_llt[l_jk].is_zero)continue;

						if (blk_llt[l_ij].is_zero)
						{
							blk_llt[l_ij].is_zero = false;
							s_mm(blk_m[i], blk_m[j], blk_m[k], -1.0, blk_llt[l_ik].data, l_t.block_stride, blk_llt[l_jk].data, T(l_t.block_stride), blk_llt[l_ij].data, l_t.block_stride);
						}
						else
						{
							s_mma(blk_m[i], blk_m[j], blk_m[k], -1.0, blk_llt[l_ik].data, l_t.block_stride, blk_llt[l_jk].data, T(l_t.block_stride), blk_llt[l_ij].data, l_t.block_stride);
						}
					}
				
					blk_llt[l_ji].is_zero = blk_llt[l_ij].is_zero;
					if (!blk_llt[l_ij].is_zero)
					{
						s_sov_lm(blk_m[j], blk_m[i], blk_llt[l_jj].data, l_t.block_stride, blk_llt[l_ij].data, T(l_t.block_stride), blk_llt[l_ji].data, l_t.block_stride);
						s_mc(blk_m[j], blk_m[i], blk_llt[l_ji].data, l_t.block_stride, blk_llt[l_ij].data, T(l_t.block_stride));
					}
				}
			}
		}
		template <typename LltType, typename BType, typename XType>
		auto s_blk_sov_lm(const BlockSize &blk_m, const BlockSize &blk_rhs, const BlockData &blk_llt, LltType llt_t, const BlockData &blk_b, BType b_t, BlockData &blk_x, XType x_t)noexcept->void
		{
			for (Size j(-1); ++j < blk_rhs.size();)
			{
				for (Size i(-1); ++i < blk_m.size();)
				{
					blk_x[id(i, j, x_t)].is_zero = blk_b[id(i, j, b_t)].is_zero;
					if (!blk_b[id(i, j, b_t)].is_zero)s_mc(blk_m[i], blk_rhs[j], blk_b[id(i, j, b_t)].data, b_t.block_stride, blk_x[id(i, j, x_t)].data, x_t.block_stride);
						
					for (Size k(-1); ++k < i;)
					{
						if (blk_llt[id(i, k, llt_t)].is_zero || blk_x[id(k, j, x_t)].is_zero) continue; 
						if (blk_x[id(i, j, x_t)].is_zero)
						{
							s_mm(blk_m[i], blk_rhs[j], blk_m[k], -1.0, blk_llt[id(i, k, llt_t)].data, llt_t.block_stride, blk_x[id(k, j, x_t)].data, x_t.block_stride, blk_x[id(i, j, x_t)].data, x_t.block_stride);
							blk_x[id(i, j, x_t)].is_zero = false;
						}
						else
						{
							s_mma(blk_m[i], blk_rhs[j], blk_m[k], -1.0, blk_llt[id(i, k, llt_t)].data, llt_t.block_stride, blk_x[id(k, j, x_t)].data, x_t.block_stride, blk_x[id(i, j, x_t)].data, x_t.block_stride);
						}
						
					}

					if (!blk_x[id(i, j, x_t)].is_zero) s_sov_lm(blk_m[i], blk_rhs[j], blk_llt[id(i, i, llt_t)].data, llt_t.block_stride, blk_x[id(i, j, x_t)].data, x_t.block_stride, blk_x[id(i, j, x_t)].data, x_t.block_stride);
				}
			}
		}
		template <typename LltType, typename BType, typename XType>
		auto s_blk_sov_um(const BlockSize &blk_m, const BlockSize &blk_rhs, const BlockData &blk_llt, LltType llt_t, const BlockData &blk_b, BType b_t, BlockData &blk_x, XType x_t)noexcept->void
		{
			for (Size j(-1); ++j < blk_rhs.size();)
			{
				for (Size i(blk_m.size()); --i < blk_m.size();)
				{
					blk_x[id(i, j, x_t)].is_zero = blk_b[id(i, j, b_t)].is_zero;
					if (!blk_b[id(i, j, b_t)].is_zero)s_mc(blk_m[i], blk_rhs[j], blk_b[id(i, j, b_t)].data, b_t.block_stride, blk_x[id(i, j, x_t)].data, x_t.block_stride);

					for (Size k(i); ++k < blk_m.size();)
					{
						if (blk_llt[id(i, k, llt_t)].is_zero || blk_x[id(k, j, x_t)].is_zero) continue;
						if (blk_x[id(i, j, x_t)].is_zero)
						{
							s_mm(blk_m[i], blk_rhs[j], blk_m[k], -1.0, blk_llt[id(i, k, llt_t)].data, llt_t.block_stride, blk_x[id(k, j, x_t)].data, x_t.block_stride, blk_x[id(i, j, x_t)].data, x_t.block_stride);
							blk_x[id(i, j, x_t)].is_zero = false;
						}
						else
						{
							s_mma(blk_m[i], blk_rhs[j], blk_m[k], -1.0, blk_llt[id(i, k, llt_t)].data, llt_t.block_stride, blk_x[id(k, j, x_t)].data, x_t.block_stride, blk_x[id(i, j, x_t)].data, x_t.block_stride);
						}

					}

					if (!blk_x[id(i, j, x_t)].is_zero) s_sov_um(blk_m[i], blk_rhs[j], blk_llt[id(i, i, llt_t)].data, llt_t.block_stride, blk_x[id(i, j, x_t)].data, x_t.block_stride, blk_x[id(i, j, x_t)].data, x_t.block_stride);
				}
			}
		}
		
		template <typename AType, typename UType, typename TauType>
		auto s_blk_householder_ut(const BlockSize &blk_m, const BlockData &blk_A, AType a_t, BlockData &blk_U, UType u_t, BlockData &blk_tau, TauType tau_t) noexcept->void
		{
			s_blk_mc(blk_m, blk_m, blk_A, a_t, blk_U, u_t);
			
			for (Size j(-1); ++j < blk_m.size();)
			{
				for (Size blk_j(-1); ++blk_j < blk_m[j];)
				{
					auto Ujj_blkjj = blk_U[id(j, j, u_t)].data + id(blk_j, blk_j, u_t.block_stride);
					
					// compute rho //
					double rho{ 0 };
					
					if (!blk_U[id(j, j, u_t)].is_zero)
					{
						s_mma(1, 1, blk_m[j] - blk_j, Ujj_blkjj, T(u_t.block_stride), Ujj_blkjj, u_t.block_stride, &rho, 1);
					}
					for (Size i(j); ++i < blk_m.size();)
					{
						if (!blk_U[id(i, j, u_t)].is_zero)
						{
							auto data = blk_U[id(i, j, u_t)].data + id(0, blk_j, u_t.block_stride);
							s_mma(1, 1, blk_m[i], data, T(u_t.block_stride), data, u_t.block_stride, &rho, 1);
						}
					}
					rho = -std::sqrt(rho);

					// compute tau[i] //
					auto t = blk_U[id(j, j, u_t)].data[id(blk_j, blk_j, u_t.block_stride)] / rho - 1.0;
					// blk_tau[id(j, 0, tau_t)].data[id(blk_j, 0, tau_t.block_stride)] = 1.0;

					// compute U left downward //
					auto alpha = 1.0 / (*Ujj_blkjj - rho);
					*Ujj_blkjj = 1.0;
					if (!blk_U[id(j, j, u_t)].is_zero)
					{
						s_nm(blk_m[j] - blk_j - 1, 1, alpha, blk_U[id(j, j, u_t)].data + id(blk_j + 1, blk_j, u_t.block_stride), u_t.block_stride);
					}
					for (Size i(j); ++i < blk_m.size();)
					{
						if (!blk_U[id(i, j, u_t)].is_zero)
						{
							auto data = blk_U[id(i, j, u_t)].data + id(0, blk_j, u_t.block_stride);
							s_nm(blk_m[i], 1, alpha, data, u_t.block_stride);
						}
					}

					// compute tau[i+1:m,i]
					if (!blk_U[id(j, j, u_t)].is_zero)
					{
						//blk_tau[id(j, 0, tau_t)].is_zero = blk_U[id(j, j, u_t)].is_zero;
						//if(!blk_tau[id(j, 0, tau_t)].is_zero)s_mm(1,blk_m[j]-j+1,blk_m[j]-j+1, Ujj_blkjj, T(u_t.block_stride), U)
						
						//for (Size i(j); ++i < blk_m.size();)
						//{
						//	blk_tau[id(j, 0, tau_t)].is_zero = true;
						//	if (!blk_U[id(i, j, u_t)].is_zero)
						//	{
						//		blk_tau[id(j, 0, tau_t)].is_zero = false;
						//	}
						//}

						//s_nm(blk_m[j] - blk_j - 1, 1, alpha, blk_U[id(j, j, u_t)].data + id(blk_j + 1, blk_j, u_t.block_stride), u_t.block_stride);
					}
					for (Size i(j); ++i < blk_m.size();)
					{
						if (!blk_U[id(i, j, u_t)].is_zero)
						{
							auto data = blk_U[id(i, j, u_t)].data + id(0, blk_j, u_t.block_stride);
							s_nm(blk_m[i], 1, alpha, data, u_t.block_stride);
						}
					}



					//std::cout << "rho:" << rho << std::endl;
					//std::cout << "tau:" << a << std::endl;
				}
			}
		}
		auto inline s_blk_householder_ut(const BlockSize &blk_m, const BlockData &blk_A, BlockData &blk_U, BlockData &blk_tau)
		{
			s_blk_householder_ut(blk_m, blk_A, BlockStride{ blk_m.size(),1,std::accumulate(blk_m.begin(),blk_m.end(),Size(0)),1 },
				blk_U, BlockStride{ blk_m.size(),1,std::accumulate(blk_m.begin(),blk_m.end(),Size(0)),1 },
				blk_tau, BlockStride{ 1,1,1,1 });
		}

		//template <typename AType, typename UType, typename TauType>
		//auto s_blk_householder_ut(Size m, const Size *blk_m, const BlockData &blk_A, AType a_t, BlockData &blk_U, UType u_t, BlockData &blk_tau, TauType tau_t) noexcept->void
		//{
		//	s_blk_mc(blk_m, blk_m, blk_A, a_t, blk_U, u_t);

		//	for (Size j(-1); ++j < blk_m.size();)
		//	{
		//		for (Size blk_j(-1); ++blk_j < blk_m[j];)
		//		{
		//			auto Ujj_blkjj = blk_U[id(j, j, u_t)].data + id(blk_j, blk_j, u_t.block_stride);

		//			// compute rho //
		//			double rho{ 0 };

		//			if (!blk_U[id(j, j, u_t)].is_zero)
		//			{
		//				s_mma(1, 1, blk_m[j] - blk_j, Ujj_blkjj, T(u_t.block_stride), Ujj_blkjj, u_t.block_stride, &rho, 1);
		//			}
		//			for (Size i(j); ++i < blk_m.size();)
		//			{
		//				if (!blk_U[id(i, j, u_t)].is_zero)
		//				{
		//					auto data = blk_U[id(i, j, u_t)].data + id(0, blk_j, u_t.block_stride);
		//					s_mma(1, 1, blk_m[i], data, T(u_t.block_stride), data, u_t.block_stride, &rho, 1);
		//				}
		//			}
		//			rho = -std::sqrt(rho);

		//			// compute tau[i] //
		//			auto t = blk_U[id(j, j, u_t)].data[id(blk_j, blk_j, u_t.block_stride)] / rho - 1.0;
		//			// blk_tau[id(j, 0, tau_t)].data[id(blk_j, 0, tau_t.block_stride)] = 1.0;

		//			// compute U left downward //
		//			auto alpha = 1.0 / (*Ujj_blkjj - rho);
		//			*Ujj_blkjj = 1.0;
		//			if (!blk_U[id(j, j, u_t)].is_zero)
		//			{
		//				s_nm(blk_m[j] - blk_j - 1, 1, alpha, blk_U[id(j, j, u_t)].data + id(blk_j + 1, blk_j, u_t.block_stride), u_t.block_stride);
		//			}
		//			for (Size i(j); ++i < blk_m.size();)
		//			{
		//				if (!blk_U[id(i, j, u_t)].is_zero)
		//				{
		//					auto data = blk_U[id(i, j, u_t)].data + id(0, blk_j, u_t.block_stride);
		//					s_nm(blk_m[i], 1, alpha, data, u_t.block_stride);
		//				}
		//			}

		//			// compute tau[i+1:m,i]
		//			if (!blk_U[id(j, j, u_t)].is_zero)
		//			{
		//				//blk_tau[id(j, 0, tau_t)].is_zero = blk_U[id(j, j, u_t)].is_zero;
		//				//if(!blk_tau[id(j, 0, tau_t)].is_zero)s_mm(1,blk_m[j]-j+1,blk_m[j]-j+1, Ujj_blkjj, T(u_t.block_stride), U)

		//				//for (Size i(j); ++i < blk_m.size();)
		//				//{
		//				//	blk_tau[id(j, 0, tau_t)].is_zero = true;
		//				//	if (!blk_U[id(i, j, u_t)].is_zero)
		//				//	{
		//				//		blk_tau[id(j, 0, tau_t)].is_zero = false;
		//				//	}
		//				//}

		//				//s_nm(blk_m[j] - blk_j - 1, 1, alpha, blk_U[id(j, j, u_t)].data + id(blk_j + 1, blk_j, u_t.block_stride), u_t.block_stride);
		//			}
		//			for (Size i(j); ++i < blk_m.size();)
		//			{
		//				if (!blk_U[id(i, j, u_t)].is_zero)
		//				{
		//					auto data = blk_U[id(i, j, u_t)].data + id(0, blk_j, u_t.block_stride);
		//					s_nm(blk_m[i], 1, alpha, data, u_t.block_stride);
		//				}
		//			}



		//			//std::cout << "rho:" << rho << std::endl;
		//			//std::cout << "tau:" << a << std::endl;
		//		}
		//	}
		//}
		//auto inline s_blk_householder_ut(const BlockSize &blk_m, const BlockData &blk_A, BlockData &blk_U, BlockData &blk_tau)
		//{
		//	s_blk_householder_ut(blk_m, blk_A, BlockStride{ blk_m.size(),1,std::accumulate(blk_m.begin(),blk_m.end(),Size(0)),1 },
		//		blk_U, BlockStride{ blk_m.size(),1,std::accumulate(blk_m.begin(),blk_m.end(),Size(0)),1 },
		//		blk_tau, BlockStride{ 1,1,1,1 });
		//}
	}
}




























#endif
