#ifndef ARIS_DYNAMIC_MATH_INTERVAL_H_
#define ARIS_DYNAMIC_MATH_INTERVAL_H_

#include <aris_lib_export.h>
#include <aris/core/basic_type.hpp>

namespace aris::dynamic {
	//
	// 对区间集合 a 和集合 b 求交集
	// 
	// set_a : 2*m 维
	//         [ left_1, right_1, left_2, right_2, ... , left_m, right_m ]
	//       
	// set_b : 2*n 维
	//         [ left_1, right_1, left_2, right_2, ... , left_n, right_n ]
	// 
	// set_c : 2*k 维, k <= (m+n)-1
	//         [ left_1, right_1, left_2, right_2, ... , left_k, right_k ]
	// 
	// 需要满足: 
	// left_1  < left_2  ... < left_m  ( or n, k)
	// right_1 < right_2 ... < right_m ( or n, k)
	//
	// 返回值 k 小于等于 m+n-1
	auto ARIS_API s_interval_intersect(Size m, Size n, const double *set_a, const double* set_b, Size &k, double* set_c) -> void;

	//
	// 对区间集合 a 和集合 b 求并集
	//
	// set_a : 2*m 维
	//         [ left_1, right_1, left_2, right_2, ... , left_m, right_m ]
	//       
	// set_b : 2*n 维
	//         [ left_1, right_1, left_2, right_2, ... , left_n, right_n ]
	// 
	// set_c : 2*k 维, k <= m+n
	//         [ left_1, right_1, left_2, right_2, ... , left_k, right_k ]
	// 
	// 需要满足: 
	// left_1  < left_2  ... < left_m  ( or n, k)
	// right_1 < right_2 ... < right_m ( or n, k)
	//
	// 返回值 k 小于等于 m+n
	auto ARIS_API s_interval_union(Size m, Size n, const double* set_a, const double* set_b, Size &k, double* set_c)->void;

	// 在全数域上求集合 a 的补集
	// 
	// set_a : 2*m 维
	//         [ left_1, right_1, left_2, right_2, ... , left_m, right_m ]
	//       
	// set_c : 2*k 维, k <= m+1
	//         [ left_1, right_1, left_2, right_2, ... , left_k, right_k ]
	// 
	// 需要满足: 
	// left_1  < left_2  ... < left_m  ( or n, k)
	// right_1 < right_2 ... < right_m ( or n, k)
	//
	// 返回值 k 小于等于 m+1
	auto ARIS_API s_interval_inverse(Size m, const double* set_a, Size &k, double* set_c)->void;
}

#endif
