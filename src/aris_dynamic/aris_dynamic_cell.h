#ifndef ARIS_DYNAMIC_CELL_
#define ARIS_DYNAMIC_CELL_

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
		auto inline next_r(Size id, StrideN s)->Size { return id + s.r_ld; }
		auto inline next_c(Size id, StrideN s)->Size { return id + s.c_ld; }
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
		auto inline next_r(Size id, StrideT s)->Size { return id + s.r_ld; }
		auto inline next_c(Size id, StrideT s)->Size { return id + s.c_ld; }
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
			if ((A.is_zero = (value == 0))) return;
			for_each_ele(A, [&value](typename AType::DataType &ele) {s_fill(ele, value); });
		}
		template<typename AType>
		auto s_fill_zero(AType &A){ A.is_zero = true; }
		template<typename ValueType, typename AType>
		auto s_nm(const ValueType &number, AType &A) noexcept->void
		{
			if ((A.is_zero = (number == 0 || A.is_zero)))return;
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
			if((B.is_zero = A.is_zero))return;
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
			if ((B.is_zero = A.is_zero))return;
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
	}
}




























#endif
