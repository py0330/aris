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

		auto inline id(Size i, Size j, BlockStride blk_s)->Size { return id(i,j,blk_s.self_stride); }
		auto inline next_row(Size id, BlockStride blk_s)->Size { return next_row(id, blk_s.self_stride); }
		auto inline next_col(Size id, BlockStride blk_s)->Size { return next_col(id, blk_s.self_stride); }
		auto inline T(BlockStride blk_s)->BlockStride { return BlockStride{ blk_s.c_ld, blk_s.r_ld, blk_s.blk_c_ld, blk_s.blk_r_ld }; }
		
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
		
		template <typename AType>
		auto s_blk_norm_fro(const BlockSize &blk_m, const BlockSize &blk_n, const BlockData &blk_A, AType a_t)noexcept->double
		{
			double norm{ 0 };

			for (Size i{ 0 }, blk_row{ 0 }; i < blk_m.size(); ++i, blk_row = next_row(blk_row, a_t))
			{
				for (Size j{ 0 }, blk_id{ blk_row }; j < blk_n.size(); ++j, blk_id = next_col(blk_id, a_t))
				{
					for (Size inner_i{ 0 }, row_id{ 0 }; inner_i < blk_m[i]; ++inner_i, row_id = next_row(row_id, a_t.block_stride))
					{
						for (Size inner_j{ 0 }, ele_id{ row_id }; inner_j < blk_n[j]; ++inner_j, ele_id = next_col(ele_id, a_t.block_stride))
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

			for (Size j{ 0 }, blk_col{ 0 }; j < blk_n.size(); ++j, blk_col = next_col(blk_col, a_t))
			{
				for (Size inner_j{ 0 }, col_id{ 0 }; inner_j < blk_n[j]; ++inner_j, col_id = next_col(col_id, a_t.block_stride))
				{
					double norm{ 0 };

					for (Size i{ 0 }, blk_id{ blk_col }; i < blk_m.size(); ++i, blk_id = next_row(blk_id, a_t))
					{
						for (Size inner_i{ 0 }, ele_id{ col_id }; inner_i < blk_m[i]; ++inner_i, ele_id = next_row(ele_id, a_t.block_stride))
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

			for (Size i{ 0 }, blk_row{ 0 }; i < blk_m.size(); ++i, blk_row = next_row(blk_row, a_t))
			{
				for (Size inner_i{ 0 }, row_id{ 0 }; inner_i < blk_m[i]; ++inner_i, row_id = next_row(row_id, a_t.block_stride))
				{
					double norm{ 0 };

					for (Size j{ 0 }, blk_id{ blk_row }; j < blk_n.size(); ++j, blk_id = next_col(blk_id, a_t))
					{
						for (Size inner_j{ 0 }, ele_id{ row_id }; inner_j < blk_n[j]; ++inner_j, ele_id = next_col(ele_id, a_t.block_stride))
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
			for (Size i(-1), a_row{ 0 }, c_row{ 0 }; ++i < blk_m.size(); a_row = next_row(a_row, a_t), c_row = next_row(c_row, c_t))
			{
				for (Size j(-1), b_col{ 0 }, c_id{ c_row }; ++j < blk_n.size(); b_col = next_col(b_col, b_t), c_id = next_col(c_id, c_t))
				{
					for (Size u(-1), a_id{ a_row }, b_id{ b_col }; ++u < blk_k.size(); a_id = next_col(a_id, a_t), b_id = next_row(b_id, b_t))
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
			for (Size i(-1), a_row{ 0 }, c_row{ 0 }; ++i < blk_m.size(); a_row = next_row(a_row, a_t), c_row = next_row(c_row, c_t))
			{
				for (Size j(-1), b_col{ 0 }, c_id{ c_row }; ++j < blk_n.size(); b_col = next_col(b_col, b_t), c_id = next_col(c_id, c_t))
				{
					for (Size u(-1), a_id{ a_row }, b_id{ b_col }; ++u < blk_k.size(); a_id = next_col(a_id, a_t), b_id = next_row(b_id, b_t))
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
			for (Size i(-1), a_row{ 0 }, c_row{ 0 }; ++i < blk_m.size(); a_row = next_row(a_row, a_t), c_row = next_row(c_row, c_t))
			{
				for (Size j(-1), b_col{ 0 }, c_id{ c_row }; ++j < blk_n.size(); b_col = next_col(b_col, b_t), c_id = next_col(c_id, c_t))
				{
					C[c_id].is_zero = true;
					for (Size u(-1), a_id{ a_row }, b_id{ b_col }; ++u < blk_k.size(); a_id = next_col(a_id, a_t), b_id = next_row(b_id, b_t))
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
			for (Size i(-1), a_row{ 0 }, c_row{ 0 }; ++i < blk_m.size(); a_row = next_row(a_row, a_t), c_row = next_row(c_row, c_t))
			{
				for (Size j(-1), b_col{ 0 }, c_id{ c_row }; ++j < blk_n.size(); b_col = next_col(b_col, b_t), c_id = next_col(c_id, c_t))
				{
					C[c_id].is_zero = true;
					for (Size u(-1), a_id{ a_row }, b_id{ b_col }; ++u < blk_k.size(); a_id = next_col(a_id, a_t), b_id = next_row(b_id, b_t))
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
		auto s_blk_llt(const BlockSize &blk_m, const BlockData &blk_A, AType a_t, BlockData &blk_llt, LltType llt_t) noexcept->void
		{
			for (Size j(-1); ++j < blk_m.size();)
			{
				s_mc(blk_m[j], blk_m[j], blk_A[id(j, j, a_t)].data, a_t.block_stride, blk_llt[id(j, j, llt_t)].data, llt_t.block_stride);
				blk_llt[id(j, j, llt_t)].is_zero = false;

				for (Size k(-1); ++k < j;)
				{
					if (!blk_llt[id(j, k, llt_t)].is_zero)s_mma(blk_m[j], blk_m[j], blk_m[k], -1.0, blk_llt[id(j, k, llt_t)].data, llt_t.block_stride, blk_llt[id(j, k, llt_t)].data, T(llt_t.block_stride), blk_llt[id(j, j, llt_t)].data, llt_t.block_stride);
				}

				s_llt(blk_m[j], blk_llt[id(j, j, llt_t)].data, llt_t.block_stride, blk_llt[id(j, j, llt_t)].data, llt_t.block_stride);

				for (Size i(j); ++i < blk_m.size();)
				{
					blk_llt[id(i, j, llt_t)].is_zero = true;
					blk_llt[id(j, i, llt_t)].is_zero = true;

					if (!blk_A[id(i, j, a_t)].is_zero)
					{
						s_mc(blk_m[i], blk_m[j], blk_A[id(i, j, a_t)].data, a_t.block_stride, blk_llt[id(i, j, llt_t)].data, llt_t.block_stride);
						blk_llt[id(i, j, llt_t)].is_zero = false;
					}
					for (Size k(-1); ++k < j;)
					{
						if (blk_llt[id(i, k, llt_t)].is_zero || blk_llt[id(j, k, llt_t)].is_zero)continue;

						if (blk_llt[id(i, j, llt_t)].is_zero)
						{
							blk_llt[id(i, j, llt_t)].is_zero = false;
							s_mm(blk_m[i], blk_m[j], blk_m[k], -1.0, blk_llt[id(i, k, llt_t)].data, llt_t.block_stride, blk_llt[id(j, k, llt_t)].data, T(llt_t.block_stride), blk_llt[id(i, j, llt_t)].data, llt_t.block_stride);
						}
						else
						{
							s_mma(blk_m[i], blk_m[j], blk_m[k], -1.0, blk_llt[id(i, k, llt_t)].data, llt_t.block_stride, blk_llt[id(j, k, llt_t)].data, T(llt_t.block_stride), blk_llt[id(i, j, llt_t)].data, llt_t.block_stride);
						}
					}
				
					if (!blk_llt[id(i, j, llt_t)].is_zero)
					{
						blk_llt[id(j, i, llt_t)].is_zero = false;
						s_sov_lm(blk_m[j], blk_m[i], blk_llt[id(j, j, llt_t)].data, llt_t.block_stride, blk_llt[id(i, j, llt_t)].data, T(llt_t.block_stride), blk_llt[id(j, i, llt_t)].data, llt_t.block_stride);
						s_mc(blk_m[j], blk_m[i], blk_llt[id(j, i, llt_t)].data, llt_t.block_stride, blk_llt[id(i, j, llt_t)].data, T(llt_t.block_stride));
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
					blk_x[id(i, j, x_t)].is_zero = true;
					
					if (!blk_b[id(i, j, b_t)].is_zero)
					{
						s_mc(blk_m[i], blk_rhs[j], blk_b[id(i, j, b_t)].data, b_t.block_stride, blk_x[id(i, j, x_t)].data, x_t.block_stride);
						blk_x[id(i, j, x_t)].is_zero = false;
					}
						
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
					blk_x[id(i, j, x_t)].is_zero = true;

					if (!blk_b[id(i, j, b_t)].is_zero)
					{
						s_mc(blk_m[i], blk_rhs[j], blk_b[id(i, j, b_t)].data, b_t.block_stride, blk_x[id(i, j, x_t)].data, x_t.block_stride);
						blk_x[id(i, j, x_t)].is_zero = false;
					}

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
		auto s_blk_householder_ut(const BlockSize &blk_m, const BlockSize &blk_n, const BlockData &blk_A, AType a_t, BlockData &blk_U, UType u_t, BlockData &blk_tau, TauType tau_t) noexcept->void
		{

		}
	}
}




























#endif
