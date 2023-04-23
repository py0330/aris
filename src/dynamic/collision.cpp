#include <cmath>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <algorithm>
#include <limits>
#include <sstream>
#include <cstddef>
#include <array>
#include <list>

#include "aris/dynamic/math_matrix.hpp"
#include "aris/dynamic/screw.hpp"
#include "aris/dynamic/collision.hpp"

namespace aris::dynamic{
	auto s_generate_box(const double* reference_marker_pm, const double* eul_321, const double* point1_xyz, const double* point2_xyz,
		double* box_center, double* box_eul, double* box_length)noexcept->void {

		// box center //
		double box_center_in_reference[3]{
			(point1_xyz[0] + point2_xyz[0]) * 0.5,
			(point1_xyz[1] + point2_xyz[1]) * 0.5,
			(point1_xyz[2] + point2_xyz[2]) * 0.5,
		};
		s_pp2pp(reference_marker_pm, box_center_in_reference, box_center);

		// box length //
		double rm[9], length[3]{ point2_xyz[0] - point1_xyz[0],point2_xyz[1] - point1_xyz[1],point2_xyz[2] - point1_xyz[2] };
		s_re2rm(eul_321, rm, "321");
		s_mm(3, 1, 3, rm, T(3), length, 1, box_length, 1);
		for (int i = 0; i < 3; ++i) box_length[i] = std::abs(box_length[i]);

		// box eul //
		s_re2re(reference_marker_pm, eul_321, box_eul);
	}
	auto s_collide_check_box2box(const double* box1_center, const double* box1_321_eul, const double* box1_length_xyz,
		const double* box2_center, const double* box2_321_eul, const double* box2_length_xyz)noexcept->int
	{
		double box1_rm[9], box2_rm[9], half_length1[3], half_length2[3], box1_to_box2_distance[3];
		s_vc(3, 0.5, box1_length_xyz, half_length1);
		s_vc(3, 0.5, box2_length_xyz, half_length2);
		for (int i = 0; i < 3; ++i) {
			half_length1[0] = std::abs(half_length1[0]);
			half_length1[1] = std::abs(half_length1[1]);
			half_length1[2] = std::abs(half_length1[2]);
		}

		s_re2rm(box1_321_eul, box1_rm, "321");
		s_re2rm(box2_321_eul, box2_rm, "321");

		s_vc(3, box2_center, box1_to_box2_distance);
		s_vs(3, box1_center, box1_to_box2_distance);

#ifdef DEBUG_COLLIDE_CHECK_BOX2BOX
		/////////////////////////
		double box1_vertexes[8][3], box2_vertexes[8][3];
		for (int i = 0; i < 8; ++i) {
			s_vc(3, box1_center, box1_vertexes[i]);
			s_va(3, (i & 0x01 ? 0.5 : -0.5) * box1_length_xyz[0], box1_rm + 0, 3, box1_vertexes[i], 1);
			s_va(3, (i & 0x02 ? 0.5 : -0.5) * box1_length_xyz[1], box1_rm + 1, 3, box1_vertexes[i], 1);
			s_va(3, (i & 0x04 ? 0.5 : -0.5) * box1_length_xyz[2], box1_rm + 2, 3, box1_vertexes[i], 1);

			s_vc(3, box2_center, box2_vertexes[i]);
			s_va(3, (i & 0x01 ? 0.5 : -0.5) * box2_length_xyz[0], box2_rm + 0, 3, box2_vertexes[i], 1);
			s_va(3, (i & 0x02 ? 0.5 : -0.5) * box2_length_xyz[1], box2_rm + 1, 3, box2_vertexes[i], 1);
			s_va(3, (i & 0x04 ? 0.5 : -0.5) * box2_length_xyz[2], box2_rm + 2, 3, box2_vertexes[i], 1);
		}

		dsp(8, 3, *box1_vertexes);
		dsp(8, 3, *box2_vertexes);
		/////////////////////////
#endif
		// 验证是否 box1 中的点被 box2 所包含
		auto check_if_contain = [](const double* box1_center, const double* half_length1, const double* box1_rm,
			const double* box2_center, const double* half_length2, const double* box2_rm)noexcept->int
		{
			for (int i = 0; i < 8; ++i) {
				double vertex_to_other_center[3];
				// compute vertex //
				s_vc(3, box1_center, vertex_to_other_center);
				s_va(3, i & 0x01 ? half_length1[0] : -half_length1[0], box1_rm + 0, 3, vertex_to_other_center, 1);
				s_va(3, i & 0x02 ? half_length1[1] : -half_length1[1], box1_rm + 1, 3, vertex_to_other_center, 1);
				s_va(3, i & 0x04 ? half_length1[2] : -half_length1[2], box1_rm + 2, 3, vertex_to_other_center, 1);

				// compute vertex to other center //
				s_vs(3, box2_center, vertex_to_other_center);

				double distance[3];
				s_mm(3, 1, 3, box2_rm, T(3), vertex_to_other_center, 1, distance, 1);

				// 判断是否在区域外 //
				if (std::abs(distance[0]) > std::abs(half_length2[0])
					|| std::abs(distance[1]) > std::abs(half_length2[1])
					|| std::abs(distance[2]) > std::abs(half_length2[2])) {
					return i + 1;
				}
			}
			return 0;
		};

		auto contain2 = check_if_contain(box2_center, half_length2, box2_rm,
			box1_center, half_length1, box1_rm);
		if (!contain2) {
#ifdef DEBUG_COLLIDE_CHECK_BOX2BOX
			std::cout << "A contains B" << std::endl;
#endif
			return 2;
		}

		auto contain1 = check_if_contain(box1_center, half_length1, box1_rm,
			box2_center, half_length2, box2_rm);
		if (!contain1) {
#ifdef DEBUG_COLLIDE_CHECK_BOX2BOX
			std::cout << "B contains A" << std::endl;
#endif
			return 3;
		}

		// check if collide, see OBB collide OBB in reference //
		// check OBB1's direction
		for (int i = 0; i < 3; ++i) {
			// direction of box1
			double da = std::abs(half_length1[i]);
			double db =
				std::abs(half_length2[0] * s_vv(3, box1_rm + i, 3, box2_rm + 0, 3)) +
				std::abs(half_length2[1] * s_vv(3, box1_rm + i, 3, box2_rm + 1, 3)) +
				std::abs(half_length2[2] * s_vv(3, box1_rm + i, 3, box2_rm + 2, 3));

			double d = std::abs(s_vv(3, box1_to_box2_distance, 1, box1_rm + i, 3));

#ifdef DEBUG_COLLIDE_BOX2BOX
			std::cout << "OBB1 " << "d:" << d << "  da:" << da << "  db:" << db << std::endl;
#endif
			// 不干涉
			if (d > da + db) {
				return 0;
			}
		}
		// check OBB2's direction
		for (int i = 0; i < 3; ++i) {
			// direction of box1
			double da = std::abs(half_length2[i]);
			double db =
				std::abs(half_length1[0] * s_vv(3, box2_rm + i, 3, box1_rm + 0, 3)) +
				std::abs(half_length1[1] * s_vv(3, box2_rm + i, 3, box1_rm + 1, 3)) +
				std::abs(half_length1[2] * s_vv(3, box2_rm + i, 3, box1_rm + 2, 3));

			double d = std::abs(s_vv(3, box1_to_box2_distance, 1, box2_rm + i, 3));

#ifdef DEBUG_COLLIDE_BOX2BOX
			std::cout << "OBB2 " << "d:" << d << "  da:" << da << "  db:" << db << std::endl;
#endif

			// 不干涉
			if (d > da + db) {
				return 0;
			}
		}
		// check (OBB1 x OBB2) 's direction
		for (int i = 0; i < 3; ++i) {
			for (int j = 0; j < 3; ++j) {
				double direction[3];
				s_c3(box1_rm + i, 3, box2_rm + j, 3, direction, 1);

				if (auto norm = s_norm(3, direction); norm > 1e-10) {
					// direction of box1
					double da =
						std::abs(half_length1[0] * s_vv(3, direction, 1, box1_rm + 0, 3)) +
						std::abs(half_length1[1] * s_vv(3, direction, 1, box1_rm + 1, 3)) +
						std::abs(half_length1[2] * s_vv(3, direction, 1, box1_rm + 2, 3));
					double db =
						std::abs(half_length2[0] * s_vv(3, direction, 1, box2_rm + 0, 3)) +
						std::abs(half_length2[1] * s_vv(3, direction, 1, box2_rm + 1, 3)) +
						std::abs(half_length2[2] * s_vv(3, direction, 1, box2_rm + 2, 3));

					double d = std::abs(s_vv(3, box1_to_box2_distance, direction));

#ifdef DEBUG_COLLIDE_BOX2BOX
					std::cout << "OBB1 X OBB2 " << "d:" << d << "  da:" << da << "  db:" << db << std::endl;
#endif

					// 不干涉
					if (d > da + db) {
						return 0;
					}
				}
			}
		}


#ifdef DEBUG_COLLIDE_BOX2BOX
		std::cout << "collide" << std::endl;
#endif

		// 干涉但不包含 //
		return 1;
	};

	auto s_collide_check_sphere2sphere(const double* sphere1_center_xyz, double sphere1_radius,
		const double* sphere2_center_xyz, double sphere2_radius)noexcept->int
	{
		sphere1_radius = std::abs(sphere1_radius);
		sphere2_radius = std::abs(sphere2_radius);

		double xyz_diff[3];
		s_vc(3, sphere2_center_xyz, xyz_diff);
		s_vs(3, sphere1_center_xyz, xyz_diff);

		double distance = s_norm(3, xyz_diff);

		if (distance > sphere1_radius + sphere2_radius)
			return 0;
		else if (distance < sphere1_radius - sphere2_radius)
			return 2;
		else if (distance < sphere2_radius - sphere1_radius)
			return 3;
		else
			return 1;
	}

	auto s_collide_check_sphere2box(const double* sphere1_center_xyz, double sphere1_radius,
		const double* box2_center, const double* box2_321_eul, const double* box2_length_xyz)noexcept->int
	{
		sphere1_radius = std::abs(sphere1_radius);

		double box2_rm[9];
		s_re2rm(box2_321_eul, box2_rm, "321");

#ifdef DEBUG_COLLIDE_CHECK_SPHERE2BOX
		/////////////////////////
		double box2_vertexes[8][3];
		for (int i = 0; i < 8; ++i) {
			s_vc(3, box2_center, box2_vertexes[i]);
			s_va(3, (i & 0x01 ? 0.5 : -0.5) * box2_length_xyz[0], box2_rm + 0, 3, box2_vertexes[i], 1);
			s_va(3, (i & 0x02 ? 0.5 : -0.5) * box2_length_xyz[1], box2_rm + 1, 3, box2_vertexes[i], 1);
			s_va(3, (i & 0x04 ? 0.5 : -0.5) * box2_length_xyz[2], box2_rm + 2, 3, box2_vertexes[i], 1);
		}
		dsp(8, 3, *box2_vertexes);
		/////////////////////////
#endif

		double diff[3], diff_local[3];
		s_vc(3, sphere1_center_xyz, diff);
		s_vs(3, box2_center, diff);

		s_mm(3, 1, 3, box2_rm, T(3), diff, 1, diff_local, 1);

		double half_length[3]{ std::abs(0.5 * box2_length_xyz[0]),std::abs(0.5 * box2_length_xyz[1]), std::abs(0.5 * box2_length_xyz[2]) };

		// 距离小球最近的一个点 //
		double close_point[3]{
			std::max(0.0, std::abs(diff_local[0]) - std::abs(half_length[0])),
			std::max(0.0, std::abs(diff_local[1]) - std::abs(half_length[1])),
			std::max(0.0, std::abs(diff_local[2]) - std::abs(half_length[2])),
		};

		// 距离小球最远的一个点 //
		double far_point[3]{
			std::abs(diff_local[0]) + std::abs(half_length[0]),
			std::abs(diff_local[1]) + std::abs(half_length[1]),
			std::abs(diff_local[2]) + std::abs(half_length[2]),
		};

		auto close_distance_to_sphere = s_norm(3, close_point);
		auto far_distance_to_sphere = s_norm(3, far_point);

		// 距离小球最近的点也大于半径，两者分离 //
		if (close_distance_to_sphere > sphere1_radius)
			return 0;
		// 距离小球最远的点也小于半径，小球包含长方体 //
		else if (far_distance_to_sphere < sphere1_radius)
			return 2;
		else if (std::abs(half_length[0]) - std::abs(diff_local[0]) > sphere1_radius &&
			std::abs(half_length[1]) - std::abs(diff_local[1]) > sphere1_radius &&
			std::abs(half_length[2]) - std::abs(diff_local[2]) > sphere1_radius)
			return 3;
		else
			return 1;
	}

	auto s_collide_check_point2box(const double* point_xyz,
		const double* box2_center, const double* box2_321_eul, const double* box2_length_xyz)noexcept->int
	{
		double diff[3]{
			point_xyz[0] - box2_center[0],
			point_xyz[1] - box2_center[1],
			point_xyz[2] - box2_center[2]
		};

		double box2_rm[9];
		s_re2rm(box2_321_eul, box2_rm, "321");

		double diff_in_box[3];
		s_mm(3, 1, 3, box2_rm, T(3), diff, 1, diff_in_box, 1);

		return diff_in_box[0] < box2_length_xyz[0]
			&& diff_in_box[1] < box2_length_xyz[1]
			&& diff_in_box[2] < box2_length_xyz[2]
			? 1 : 0;
	}

	auto s_collide_check_point2sphere(const double* point_xyz,
		const double* sphere2_center_xyz, double sphere2_radius)noexcept->int
	{
		double diff[3]{
			point_xyz[0] - sphere2_center_xyz[0],
			point_xyz[1] - sphere2_center_xyz[1],
			point_xyz[2] - sphere2_center_xyz[2]
		};

		return s_norm(3, diff) < sphere2_radius ? 1 : 0;
	}
}
