#include <iostream>

#include "aris/core/log.hpp"

#include "aris/dynamic/math_interval.hpp"

namespace aris::dynamic {

	auto ARIS_API s_interval_intersect(Size m, const double* set_a, Size n, const double* set_b, double* set_c, double zero_check)->Size {

		Size k = 0;
		for (Size i = 0, j = 0; i < m && j < n;) {
			auto left_i = set_a[i*2];
			auto right_i = set_a[i*2 + 1];
			auto left_j = set_b[j*2];
			auto right_j = set_b[j*2 + 1];

			if (std::max(left_i, left_j) < std::min(right_i, right_j)) {
				set_c[k * 2] = std::max(left_i, left_j);
				set_c[k * 2 + 1] = std::min(right_i, right_j);
				++k;
			}

			++(right_i < right_j ? i : j);
		}
		
		return k;

	}

}





