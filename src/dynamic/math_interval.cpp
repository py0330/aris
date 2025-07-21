#include <iostream>

#include "aris/core/log.hpp"

#include "aris/dynamic/math_interval.hpp"
#include "aris/dynamic/math_matrix.hpp"

namespace aris::dynamic {

	auto ARIS_API s_interval_intersect(Size m, Size n, const double* set_a, const double* set_b, Size &k, double* set_c)->void {

		k = 0;
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

	}

	auto ARIS_API s_interval_union(Size m, Size n, const double* set_a, const double* set_b, Size &k, double* set_c)->void {

		k = 0;

		Size i = 0, j = 0;
		for (Size idx = 0; idx < m + n; idx++) {
			if (i >= m || j >= n)
				break;

			auto left_i = set_a[at(i, 0, 2)];
			auto right_i = set_a[at(i, 1, 2)];
			auto left_j = set_b[at(j, 0, 2)];
			auto right_j = set_b[at(j, 1, 2)];

			auto l = left_i < left_j ? left_i : left_j;
			auto r = left_i < left_j ? right_i : right_j;
			(left_i < left_j ? i : j)++;

			if (idx == 0) {
				set_c[at(k, 0, 2)] = l;
				set_c[at(k, 1, 2)] = r;
				++k;
			}
			else {
				if (l <= set_c[at(k-1, 1, 2)]) {
					set_c[at(k-1, 1, 2)] = std::max(set_c[at(k-1, 1, 2)], r);
				}
				else {
					set_c[at(k, 0, 2)] = l;
					set_c[at(k, 1, 2)] = r;
					++k;
				}
			}
		}

		for (Size p = i; p < m; ++p) {
			if (set_a[at(p, 0, 2)] <= set_c[at(k-1, 1, 2)]) {
				set_c[at(k-1, 1, 2)] = std::max(set_c[at(k-1, 1, 2)], set_a[at(p, 1, 2)]);
			}
			else {
				set_c[at(k, 0, 2)] = set_a[at(p, 0, 2)];
				set_c[at(k, 1, 2)] = set_a[at(p, 1, 2)];
				++k;
			}
		}

		for (Size p = j; p < n; ++p) {
			if (set_b[at(p, 0, 2)] <= set_c[at(k-1, 1, 2)]) {
				set_c[at(k-1, 1, 2)] = std::max(set_c[at(k-1, 1, 2)], set_b[at(p, 1, 2)]);
			}
			else {
				set_c[at(k, 0, 2)] = set_b[at(p, 0, 2)];
				set_c[at(k, 1, 2)] = set_b[at(p, 1, 2)];
				++k;
			}
		}
	}

	auto ARIS_API s_interval_inverse(Size m, const double* set_a, Size &k, double* set_c)->void {
		if (m == 0) {
			set_c[0] = -std::numeric_limits<double>::infinity();
			set_c[1] = std::numeric_limits<double>::infinity();
			k = 1;
			return;
		}

		Size size_c = 0;

		auto l = set_a[0];
		auto r = set_a[2 * (m - 1) + 1];
		constexpr auto inf = std::numeric_limits<double>::infinity();

		if (l == -inf && r == inf) {
			std::copy_n(set_a + 1, 2*m - 2, set_c);
			k = m - 1;
			return;
		}
		else if (l == -inf) {
			std::copy_n(set_a + 1, 2*m - 1, set_c);
			set_c[2 * (m - 1) + 1] = inf;
			k = m;
			return;
		}
		else if (r == inf) {
			set_c[0] = -inf;
			std::copy_n(set_a, 2*m - 1, set_c + 1);
			k = m;
			return;
		}
		else {
			set_c[0] = -inf;
			std::copy_n(set_a, 2*m, set_c + 1);
			set_c[2*m + 1] = inf;
			k = m + 1;
			return;
		}

	}
}





