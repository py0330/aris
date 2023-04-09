#ifndef ARIS_DYNAMIC_COLLISION_H_
#define ARIS_DYNAMIC_COLLISION_H_

#include <vector>
#include <numeric>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <algorithm>
#include <cmath>
#include <iterator>

#include <aris_lib_export.h>
#include <aris/core/basic_type.hpp>

namespace aris::dynamic{
	//    生成长方体区域
	//    input:  reference_marker_pm 4x4
	//            eul_321
	//            point1_xyz
	//            point2_xyz
	//    
	//    output: box_center    : wrt World Frame
	//            box_eul       : wrt World Frame
	//            box_length    : length along xyz
	auto ARIS_API s_generate_box(const double* reference_marker_pm, const double* eul_321, const double* point1_xyz, const double* point2_xyz,
		double* box_center, double* box_eul, double* box_length)noexcept->void;

	//    检查两个长方体是否碰撞
	//    ret 0: 两者无干涉
	//        1: 两者有干涉
	//        2: box1 包含 box2
	//        3: box2 包含 box1 
	auto ARIS_API s_collide_check_box2box(const double* box1_center, const double* box1_321_eul, const double* box1_length_xyz,
		const double* box2_center, const double* box2_321_eul, const double* box2_length_xyz)noexcept->int;

	//    检查两个长方体是否碰撞
	//    ret 0: 两者无干涉
	//        1: 两者有干涉
	//        2: sphere1 包含 sphere2
	//        3: sphere2 包含 sphere1 
	auto ARIS_API s_collide_check_sphere2sphere(const double* sphere1_center_xyz, double sphere1_radius,
		const double* sphere2_center_xyz, double sphere2_radius)noexcept->int;

	//    检查球体与长方体是否碰撞
	//    ret 0: 两者无干涉
	//        1: 两者有干涉
	//        2: sphere1 包含 box2 
	//        3: box2    包含 sphere1
	auto ARIS_API s_collide_check_sphere2box(const double* sphere1_center_xyz, double sphere1_radius,
		const double* box2_center, const double* box2_321_eul, const double* box2_length_xyz)noexcept->int;

	//    检查长方体与球体是否碰撞
	//    ret 0: 两者无干涉
	//        1: 两者有干涉
	//        2: box1    包含 sphere2 
	//        3: sphere2 包含 box1
	auto inline ARIS_API s_collide_check_box2sphere(const double* box1_center, const double* box1_321_eul, const double* box1_length_xyz,
		const double* sphere2_center_xyz, double sphere2_radius)noexcept->int
	{
		auto ret = s_collide_check_sphere2box(sphere2_center_xyz, sphere2_radius, box1_center, box1_321_eul, box1_length_xyz);
		return ret < 2 ? ret : (ret == 2 ? 3 : 2);
	}

	//    检查点与长方体是否碰撞
	//    ret 0: 两者无干涉
	//        1: 两者有干涉，同时点也被box包含
	auto ARIS_API s_collide_check_point2box(const double* point_xyz,
		const double* box2_center, const double* box2_321_eul, const double* box2_length_xyz)noexcept->int;

	//    检查点与球体是否碰撞
	//    ret 0: 两者无干涉
	//        1: 两者有干涉，同时点也被sphere包含
	auto ARIS_API s_collide_check_point2sphere(const double* point_xyz,
		const double* sphere2_center_xyz, double sphere2_radius)noexcept->int;
}

#endif
