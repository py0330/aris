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

#include <aris_dynamic_matrix.h>

namespace aris
{
	namespace dynamic
	{
		struct BlockStride 
		{ 
			Size r_ld, c_ld, blk_r_ld, blk_c_ld;
			auto selfStride()->Stride { return Stride{ r_ld, c_ld }; }
			auto blockStride()->Stride { return Stride{ blk_r_ld, blk_c_ld }; }
		};
		struct BlockNode { double * data; bool is_zero; };
		using BlockSize = std::vector<Size>;
		using BlockData = std::vector<BlockNode>;

		auto inline id(Size i, Size j, BlockStride blk_s)->Size { return i*blk_s.r_ld + j*blk_s.c_ld; }
		auto inline next_row(Size id, BlockStride blk_s)->Size { return id + blk_s.r_ld; }
		auto inline next_col(Size id, BlockStride blk_s)->Size { return id + blk_s.c_ld; }
		auto inline T(BlockStride blk_s)->BlockStride { return BlockStride{ blk_s.c_ld, blk_s.r_ld, blk_s.blk_c_ld, blk_s.blk_r_ld }; }
		
		template <typename AType, typename BType>
		auto inline s_blk_map(const BlockSize &blk_m, const BlockSize &blk_n, double *A, AType a_t, BlockData &blk_A, BType blk_a_t)->void
		{
			for (Size i{ 0 }, blk_i{ 0 }; blk_i < blk_m.size(); i += blk_m[blk_i], ++blk_i)
			{
				for (Size j{ 0 }, blk_j{ 0 }; blk_j < blk_n.size(); j += blk_n[blk_j], ++blk_j)
				{
					blk_A[id(blk_i, blk_j, blk_a_t)] = BlockNode{ A + id(i,j,a_t), false };
				}
			}
		}
		auto inline s_blk_map(const BlockSize &blk_m, const BlockSize &blk_n, double *A, BlockData &blk_A)->void
		{
			s_blk_map(blk_m, blk_n, A, std::accumulate(blk_n.begin(), blk_n.end(), Size(0)), blk_A, blk_n.size());
		}
		template <typename AType, typename BType, typename CType>
		auto inline s_blk_mm(const BlockSize &blk_m, const BlockSize &blk_n, const BlockSize &blk_k, const BlockData &A, AType a_t, const BlockData &B, BType b_t, BlockData &C, CType c_t)noexcept->void
		{
			for (Size i{ 0 }, a_row{ 0 }, c_row{ 0 }; i < blk_m.size(); ++i, a_row = next_row(a_row, a_t), c_row = next_row(c_row, c_t))
			{
				for (Size j{ 0 }, b_col{ 0 }, c_id{ c_row }; j < blk_n.size(); ++j, b_col = next_col(b_col, b_t), c_id = next_col(c_id, c_t))
				{
					C[id(i, j, c_t)].is_zero = true;
					for (Size u{ 0 }, a_id{ a_row }, b_id{ b_col }; u < blk_k.size(); ++u, a_id = next_col(a_id, a_t), b_id = next_row(b_id, b_t))
					{
						if (A[id(i, u, a_t)].is_zero || B[id(u, j, b_t)].is_zero)continue;
						if (C[id(i, j, c_t)].is_zero)
						{
							C[id(i, j, c_t)].is_zero = false;
							s_mm(blk_m[i], blk_n[j], blk_k[u], A[a_id].data, Stride{ a_t.blk_r_ld, a_t.blk_c_ld }, B[b_id].data, Stride{ b_t.blk_r_ld, b_t.blk_c_ld }, C[c_id].data, Stride{ c_t.blk_r_ld, c_t.blk_c_ld });
						}
						else
						{
							s_mma(blk_m[i], blk_n[j], blk_k[u], A[a_id].data, Stride{ a_t.blk_r_ld, a_t.blk_c_ld }, B[b_id].data, Stride{ b_t.blk_r_ld, b_t.blk_c_ld }, C[c_id].data, Stride{ c_t.blk_r_ld, c_t.blk_c_ld });
						}
					}
				}
			}
		}
		auto inline s_blk_mm(const BlockSize &blk_m, const BlockSize &blk_n, const BlockSize &blk_k, const BlockData &A, const BlockData &B, BlockData &C)->void
		{
			s_blk_mm(blk_m, blk_n, blk_k, A, BlockStride{ blk_k.size(),1,std::accumulate(blk_k.begin(),blk_k.end(),Size(0)),1 },
				B, BlockStride{ blk_n.size(),1,std::accumulate(blk_n.begin(),blk_n.end(),Size(0)),1 },
				C, BlockStride{ blk_n.size(),1,std::accumulate(blk_n.begin(),blk_n.end(),Size(0)),1 });
		}

		template <typename AType, typename LltType>
		auto s_blk_llt(const BlockSize &blk_m, const BlockData &blk_A, AType a_t, BlockData &blk_llt, LltType llt_t) noexcept->void
		{
			for (Size j = 0; j < blk_m.size(); ++j)
			{
				s_mc(blk_m[j], blk_m[j], blk_A[id(j, j, a_t)].data, a_t.blockStride(), blk_llt[id(j, j, llt_t)].data, llt_t.blockStride());
				blk_llt[id(j, j, llt_t)].is_zero = false;

				for (Size k = 0; k < j; ++k)
				{
					if (!blk_llt[id(j, k, llt_t)].is_zero)s_mma(blk_m[j], blk_m[j], blk_m[k], -1.0, blk_llt[id(j, k, llt_t)].data, llt_t.blockStride(), blk_llt[id(j, k, llt_t)].data, T(llt_t.blockStride()), blk_llt[id(j, j, llt_t)].data, llt_t.blockStride());
				}

				s_llt(blk_m[j], blk_llt[id(j, j, llt_t)].data, llt_t.blockStride(), blk_llt[id(j, j, llt_t)].data, llt_t.blockStride());

				for (Size i = j + 1; i < blk_m.size(); ++i)
				{
					blk_llt[id(i, j, llt_t)].is_zero = true;
					blk_llt[id(j, i, llt_t)].is_zero = true;

					if (!blk_A[id(i, j, a_t)].is_zero)
					{
						s_mc(blk_m[i], blk_m[j], blk_A[id(i, j, a_t)].data, a_t.blockStride(), blk_llt[id(i, j, llt_t)].data, llt_t.blockStride());
						blk_llt[id(i, j, llt_t)].is_zero = false;
					}
					for (Size k = 0; k < j; ++k)
					{
						if (blk_llt[id(i, k, llt_t)].is_zero || blk_llt[id(j, k, llt_t)].is_zero)continue;

						if (blk_llt[id(i, j, llt_t)].is_zero)
						{
							blk_llt[id(i, j, llt_t)].is_zero = false;
							s_mm(blk_m[i], blk_m[j], blk_m[k], -1.0, blk_llt[id(i, k, llt_t)].data, llt_t.blockStride(), blk_llt[id(j, k, llt_t)].data, T(llt_t.blockStride()), blk_llt[id(i, j, llt_t)].data, llt_t.blockStride());
						}
						else
						{
							s_mma(blk_m[i], blk_m[j], blk_m[k], -1.0, blk_llt[id(i, k, llt_t)].data, llt_t.blockStride(), blk_llt[id(j, k, llt_t)].data, T(llt_t.blockStride()), blk_llt[id(i, j, llt_t)].data, llt_t.blockStride());
						}
					}
				
					if (!blk_llt[id(i, j, llt_t)].is_zero)
					{
						blk_llt[id(j, i, llt_t)].is_zero = false;
						s_sov_lm(blk_m[j], blk_m[i], blk_llt[id(j, j, llt_t)].data, llt_t.blockStride(), blk_llt[id(i, j, llt_t)].data, T(llt_t.blockStride()), blk_llt[id(j, i, llt_t)].data, llt_t.blockStride());
						s_mc(blk_m[j], blk_m[i], blk_llt[id(j, i, llt_t)].data, llt_t.blockStride(), blk_llt[id(i, j, llt_t)].data, T(llt_t.blockStride()));
					}
				}
			}
			
			
			
			//Size m = blk_size.size();

			//for (Size j = 0; j < m; ++j)
			//{
			//	L[j][j].resize(blk_size[j] * blk_size[j]);

			//	std::copy(A[j][j].data(), A[j][j].data() + blk_size[j] * blk_size[j], L[j][j].data());
			//	for (Size k = 0; k < j; ++k)
			//	{
			//		if (!L[j][k].empty())s_mmaNT(blk_size[j], blk_size[j], blk_size[k], -1.0, L[j][k].data(), L[j][k].data(), L[j][j].data());
			//	}
			//	s_llt(blk_size[j], L[j][j].data(), L[j][j].data());

			//	for (Size i = j + 1; i < m; ++i)
			//	{
			//		L[i][j].clear();
			//		L[j][i].clear();

			//		if (!A[i][j].empty())
			//		{
			//			L[i][j].resize(blk_size[i] * blk_size[j]);
			//			std::copy(A[i][j].data(), A[i][j].data() + blk_size[i] * blk_size[j], L[i][j].data());
			//		}
			//		for (Size k = 0; k < j; ++k)
			//		{
			//			if (L[i][k].empty() || L[j][k].empty())continue;

			//			if (L[i][j].empty())
			//			{
			//				L[i][j].resize(blk_size[i] * blk_size[j]);
			//				s_mmNT(blk_size[i], blk_size[j], blk_size[k], -1.0, L[i][k].data(), L[j][k].data(), L[i][j].data());
			//			}
			//			else
			//			{
			//				s_mmaNT(blk_size[i], blk_size[j], blk_size[k], -1.0, L[i][k].data(), L[j][k].data(), L[i][j].data());
			//			}
			//		}

			//		if (!L[i][j].empty())
			//		{
			//			L[j][i].resize(blk_size[i] * blk_size[j]);
			//			s_sov_lmNT(blk_size[j], blk_size[i], L[j][j].data(), L[i][j].data(), L[j][i].data());
			//			s_transpose(blk_size[j], blk_size[i], L[j][i].data(), L[i][j].data());
			//		}
			//	}
			//}
		}
		template <typename LltType, typename BType, typename XType>
		auto s_blk_sov_lm(const BlockSize &blk_m, const BlockSize &blk_rhs, const BlockData &blk_llt, LltType llt_t, const BlockData &blk_b, BType b_t, BlockData &blk_x, XType x_t)noexcept->void
		{
			for (Size j = 0; j < blk_rhs.size(); ++j)
			{
				for (Size i = 0; i < blk_m.size(); ++i)
				{
					blk_x[id(i, j, x_t)].is_zero = true;
					
					if (!blk_b[id(i, j, b_t)].is_zero)
					{
						s_mc(blk_m[i], blk_rhs[j], blk_b[id(i, j, b_t)].data, b_t.blockStride(), blk_x[id(i, j, x_t)].data, x_t.blockStride());
						blk_x[id(i, j, x_t)].is_zero = false;
					}
						
					for (Size k = 0; k < i; ++k)
					{
						if (blk_llt[id(i, k, llt_t)].is_zero || blk_x[id(k, j, x_t)].is_zero) continue; 
						if (blk_x[id(i, j, x_t)].is_zero)
						{
							s_mm(blk_m[i], blk_rhs[j], blk_m[k], -1.0, blk_llt[id(i, k, llt_t)].data, llt_t.blockStride(), blk_x[id(k, j, x_t)].data, x_t.blockStride(), blk_x[id(i, j, x_t)].data, x_t.blockStride());
							blk_x[id(i, j, x_t)].is_zero = false;
						}
						else
						{
							s_mma(blk_m[i], blk_rhs[j], blk_m[k], -1.0, blk_llt[id(i, k, llt_t)].data, llt_t.blockStride(), blk_x[id(k, j, x_t)].data, x_t.blockStride(), blk_x[id(i, j, x_t)].data, x_t.blockStride());
						}
						
					}

					if (!blk_x[id(i, j, x_t)].is_zero) s_sov_lm(blk_m[i], blk_rhs[j], blk_llt[id(i, i, llt_t)].data, llt_t.blockStride(), blk_x[id(i, j, x_t)].data, x_t.blockStride(), blk_x[id(i, j, x_t)].data, x_t.blockStride());
				}
			}
		}
		template <typename LltType, typename BType, typename XType>
		auto s_blk_sov_um(const BlockSize &blk_m, const BlockSize &blk_rhs, const BlockData &blk_llt, LltType llt_t, const BlockData &blk_b, BType b_t, BlockData &blk_x, XType x_t)noexcept->void
		{
			for (Size j = 0; j < blk_rhs.size(); ++j)
			{
				for (Size i = blk_m.size() - 1; i < blk_m.size(); --i)
				{
					blk_x[id(i, j, x_t)].is_zero = true;

					if (!blk_b[id(i, j, b_t)].is_zero)
					{
						s_mc(blk_m[i], blk_rhs[j], blk_b[id(i, j, b_t)].data, b_t.blockStride(), blk_x[id(i, j, x_t)].data, x_t.blockStride());
						blk_x[id(i, j, x_t)].is_zero = false;
					}

					for (Size k = i + 1; k < blk_m.size(); ++k)
					{
						if (blk_llt[id(i, k, llt_t)].is_zero || blk_x[id(k, j, x_t)].is_zero) continue;
						if (blk_x[id(i, j, x_t)].is_zero)
						{
							s_mm(blk_m[i], blk_rhs[j], blk_m[k], -1.0, blk_llt[id(i, k, llt_t)].data, llt_t.blockStride(), blk_x[id(k, j, x_t)].data, x_t.blockStride(), blk_x[id(i, j, x_t)].data, x_t.blockStride());
							blk_x[id(i, j, x_t)].is_zero = false;
						}
						else
						{
							s_mma(blk_m[i], blk_rhs[j], blk_m[k], -1.0, blk_llt[id(i, k, llt_t)].data, llt_t.blockStride(), blk_x[id(k, j, x_t)].data, x_t.blockStride(), blk_x[id(i, j, x_t)].data, x_t.blockStride());
						}

					}

					if (!blk_x[id(i, j, x_t)].is_zero) s_sov_um(blk_m[i], blk_rhs[j], blk_llt[id(i, i, llt_t)].data, llt_t.blockStride(), blk_x[id(i, j, x_t)].data, x_t.blockStride(), blk_x[id(i, j, x_t)].data, x_t.blockStride());
				}
			}
		}


		
	}
}




























#endif
