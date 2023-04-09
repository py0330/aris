#ifndef ARIS_DYNAMIC_KINEMATICS_H_
#define ARIS_DYNAMIC_KINEMATICS_H_

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
	auto inline s_sinx_over_x(double x)->double { return std::abs(x) < 1e-8 ? 1.0 : std::sin(x) / x; };
	// 1-cos(x) = 2 sin(x/2)^2
	//    1-cos(x) / x^2
	// =  2 sin(x/2)^2 / x^2 
	// =  0.5 sin(x/2)^2 / (x/2)^2 
	// =  0.5 [sin(x/2) / (x/2) ]^2 
	// 
	auto inline s_one_minus_cosx_over_square_x(double x)->double { return 0.5 * s_sinx_over_x(0.5 * x) * s_sinx_over_x(0.5 * x); };

	// value         : 输入值
	// which_period  : 返回值应该在的周期，
	//                 例如 which_period =  0.0，代表返回值应该在 [ -0.5 * period ,  0.5 * period ] 中
	//                      which_period =  0.5，代表返回值应该在 [             0 ,        period ] 中
	//                      which_period = -2.3，代表返回值应该在 [ -2.8 * period , -1.8 * period ] 中
	// period        : 周期值，常见的是 pi
	auto ARIS_API s_put_into_period(double value, double which_period, double period)->double;

	/// \brief 根据原点和两个坐标轴上的点来求位姿矩阵
	///
	/// 这里原点origin为位姿矩阵pm_out的点,first_pnt位于第一根坐标轴,second_pnt位于第一根坐标轴和第二根坐标轴所构成的平面内
	///
	///
	auto ARIS_API s_sov_pnts2pm(const double* origin, Size origin_ld, const double* first_pnt, Size first_ld, const double* second_pnt, Size second_ld, double* pm_out, const char* axis_order = "xy") noexcept->void;
	auto inline s_sov_pnts2pm(const double* origin, const double* first_pnt, const double* second_pnt, double* pm_out, const char* axis_order = "xy") noexcept->void { s_sov_pnts2pm(origin, 1, first_pnt, 1, second_pnt, 1, pm_out, axis_order); };

	/// \brief 根据原点和两个坐标轴上的点来求位姿矩阵
	///
	/// 这里原点origin为位姿矩阵pm_out的点,first_axis为第一根坐标轴的方向,second_pnt位于第一根坐标轴和第二根坐标轴所构成的平面内
	///
	///
	auto ARIS_API s_sov_axes2pm(const double* origin, Size origin_ld, const double* first_axis, Size first_ld, const double* second_axis, Size second_ld, double* pm_out, const char* axis_order = "xy") noexcept->void;
	/// \brief 根据原点和两个坐标轴上的点来求位姿矩阵
	///
	/// 这里原点origin为位姿矩阵pm_out的点,first_axis为第一坐标轴的方向,second_axis位于第一根坐标轴与第二根坐标轴的平面内，也是方向
	///
	///
	auto inline s_sov_axes2pm(const double* origin, const double* first_axis, const double* second_axis, double* pm_out, const char* axis_order = "xy") noexcept->void { s_sov_axes2pm(origin, 1, first_axis, 1, second_axis, 1, pm_out, axis_order); };

	/// \brief 求解形如 k1 * sin(theta) + k2 * cos(theta) = b 的方程,该方程有2个根(可能相等)，成功返回0，失败返回-1
	///
	///
	auto ARIS_API s_sov_theta(double k1, double k2, double b, double* theta_out)noexcept->int;

	/// \brief 求解alpha 和 beta, 使得轴pp0转到pp的位置，alpha和beta的转轴由order定义，pp0为alpha和beta转轴的叉乘方向
	///
	///
	auto ARIS_API s_sov_ab(const double* pp, double* ab, const char* order = "321")noexcept->void;
	/// \brief 求解v_alpha 和 v_beta, 使得轴pp0转到pp的位置，alpha和beta的转轴由order定义，pp0为alpha和beta转轴的叉乘方向
	///
	///
	auto ARIS_API s_sov_vab(const double* pp, const double* vp, double* vab, double* ab, const char* order = "321")noexcept->void;
	/// \brief 求解a_alpha 和 a_beta, 使得轴pp0转到pp的位置，alpha和beta的转轴由order定义，pp0为alpha和beta转轴的叉乘方向
	///
	///
	auto ARIS_API s_sov_aab(const double* pp, const double* vp, const double* ap, double* aab, double* vab, double* ab, const char* order = "321")noexcept->void;

	/// \brief 求解alpha 和 beta, 使得轴pp0转到pp的位置，alpha和beta的转轴由order定义，pp0为任意位置，包含两个解
	///
	///
	auto ARIS_API s_sov_ab_arbitrary(const double* pp0, const double* pp, double* alpha, double* beta, const char* order = "321")noexcept->int;


	/// \brief 求解某根轴下的相对位移，axis为0，1，2时对应x、y、z轴的位移，为4、5、6时对应延x、y、z轴的转角
	///
	///
	auto ARIS_API s_sov_axis_distance(const double* from_pm, const double* to_pm, Size axis)noexcept->double;


	/// \brief 求解平面内2点法标定，适用于scara 和 delta等4轴机器人
	/// input:      2组数据6个数：[x1 y1 c1 x2 y2 c2]
	/// result:     [tool_x tool_y]
	/// mini_angle: 最小允许的输入角度差值
	auto ARIS_API s_calib_tool_two_pnts(const double* input, double* result, double mini_angle = 0.1)noexcept->int;

	enum class EEType {
		PE313,
		PE321,
		PE123,
		PQ,
		PM,
		XYZT,
		XYZ,
		XYT,
		XY,
		X,
		ABC,
		A,
		UNKNOWN,
	};

	auto inline getEETypePosSize(EEType type)noexcept->aris::Size {
		switch (type)
		{
		case EEType::PE313:
			return 6;
		case EEType::PE321:
			return 6;
		case EEType::PE123:
			return 6;
		case EEType::PQ:
			return 7;
		case EEType::PM:
			return 16;
		case EEType::XYZT:
			return 4;
		case EEType::XYZ:
			return 3;
		case EEType::XYT:
			return 3;
		case EEType::XY:
			return 2;
		case EEType::X:
			return 1;
		case EEType::ABC:
			return 1;
		case EEType::A:
			return 1;
		case EEType::UNKNOWN:
			return -1;
		default:
			return -1;
		}
	}
	auto inline getEETypePosSize(aris::Size n, const EEType* ee_types)noexcept->aris::Size {
		aris::Size size = 0;
		for (Size i = 0; i < n; ++i) {
			size += getEETypePosSize(ee_types[i]);
		}
		return size;
	}
	auto inline getScurveSize(EEType type)noexcept->aris::Size {
		switch (type)
		{
		case EEType::PE313:
			return 2;
		case EEType::PE321:
			return 2;
		case EEType::PE123:
			return 2;
		case EEType::PQ:
			return 2;
		case EEType::PM:
			return 2;
		case EEType::XYZT:
			return 2;
		case EEType::XYZ:
			return 1;
		case EEType::XYT:
			return 2;
		case EEType::XY:
			return 1;
		case EEType::X:
			return 1;
		case EEType::ABC:
			return 3;
		case EEType::A:
			return 1;
		case EEType::UNKNOWN:
			return -1;
		default:
			return -1;
		}
	}
	auto inline getEeTypeScurveSize(aris::Size n, const EEType *ee_types)noexcept->aris::Size {
		aris::Size size = 0;
		for (Size i = 0; i < n;++i) {
			size += getScurveSize(ee_types[i]);
		}
		return size;
	}
}

#endif
