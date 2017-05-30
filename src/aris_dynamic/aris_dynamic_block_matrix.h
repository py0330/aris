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
	}
}




























#endif
