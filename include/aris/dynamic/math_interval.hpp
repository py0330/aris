#ifndef ARIS_DYNAMIC_MATH_INTERVAL_H_
#define ARIS_DYNAMIC_MATH_INTERVAL_H_

#include <aris_lib_export.h>
#include <aris/core/basic_type.hpp>

namespace aris::dynamic {
	//
	// set_a : 2*m 维
	//         [ left_1, right_1, left_2, right_2, ... , left_m, right_m ]
	//       
	// set_b : 2*n 维
	//         [ left_1, right_1, left_2, right_2, ... , left_n, right_n ]
	// 
	// set_c : 2*k 维, k < m+n
	//         [ left_1, right_1, left_2, right_2, ... , left_k, right_k ]
	// 
	// 需要满足: 
	// left_1  < left_2  ... < left_m  ( or n, k)
	// right_1 < right_2 ... < right_m ( or n, k)
	auto ARIS_API s_interval_intersect(Size m, const double *set_a, Size n, const double* set_b, double* set_c, double zero_check = 1e-10) -> Size;
}

#endif
