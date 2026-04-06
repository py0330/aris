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
#include <functional>

#include <aris_lib_export.h>
#include <aris/core/basic_type.hpp>

namespace aris::dynamic{
	
	// 补偿手眼精度
	// 
	// 1. 坐标系：
	//    E：眼睛坐标系 
	//    R：机器人坐标系（法兰盘位置）
	//    B：基座坐标系
	//    O：工件坐标系（obj）
	// 
	// 2. 时刻
	//    a：视觉标定时刻（理想状态）
	//    b：一次标定时刻（小车刚到）
	//    c：二次标定时刻（机械臂基于视觉补偿运动到理想状态零位）
	//
	// 3. 假设
	//    
	// 
	// 4. 已知
	//    P_Ea2Ra : 理想状态下的手眼关系
	//    P_Oa2Ea : 理想状态下的工件位置
	//    P_Ra2Ba : 理想状态下的初始法兰位置
	//    
	//    P_Ob2Eb : 小车刚到时，工件在视觉中的位置
	// 
	// 
	// 5. 未知
	//    手眼的误差：Pd（P_Eb2Rb = P_Ec2Rc = Pd * P_Ea2Ra）
	//   
	// 
	//    
	// 6. 推导
	//    3个时刻的 O 为世界坐标系远点，因此有：
	//       P_Eb2Ea = P_Oa2Ea * P_Ob2Eb^T
	//	  
	//    理论上在b时刻，经过视觉后的机器人位姿补偿量为 Pc
	//    => P_Rb2Ra = P_Ea2Ra
	// 
	// 
	// 
	//

	auto inline s_eye_compensate() {
	
	}

	//
	// obj_in_eye
	// tool_in_base
	// mem_need: n x n x 16
	auto ARIS_API s_eye_in_hand_calib(int n, const double* pq_obj_in_eye, const double* pq_tool_in_base, double* eye_in_tool, double *mem_need)->void;

	//
	// obj_in_eye
	// tool_in_base
	// mem_need: n x n x 16
	auto ARIS_API s_eye_to_hand_calib(int n, const double* pq_obj_in_eye, const double* pq_tool_in_base, double* eye_in_base, double* mem_need)->void;

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
	// period        : 周期值，常见的是 2*pi
	auto ARIS_API s_put_into_period(double value, double which_period, double period)->double;

	// value         : 输入值
	// current_value : 返回值应该在该值的附近，与该值的差应小于半个周期
	// period        : 周期值，常见的是 2*pi
	auto ARIS_API s_put_near_value(double value, double current_value, double period)->double;

	// value         : 输入值
	// period        : 周期值，常见的是 2*pi
	// range_left    : range 左侧
	// range_right   : range 右侧
	//
	// 返回          : 0 success, -1 failed
	auto ARIS_API s_put_into_range(double value, double period, double range_left, double range_right, double& result)->int;

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

	enum class PosType {
		PE121,   // 位置与121欧拉角，6维末端， 6维向量
		PE123,   // 位置与123欧拉角，6维末端， 6维向量
		PE131,   // 位置与131欧拉角，6维末端， 6维向量
		PE132,   // 位置与132欧拉角，6维末端， 6维向量
		PE212,   // 位置与212欧拉角，6维末端， 6维向量
		PE213,   // 位置与213欧拉角，6维末端， 6维向量
		PE231,   // 位置与231欧拉角，6维末端， 6维向量
		PE232,   // 位置与232欧拉角，6维末端， 6维向量
		PE312,   // 位置与312欧拉角，6维末端， 6维向量
		PE313,   // 位置与313欧拉角，6维末端， 6维向量
		PE321,   // 位置与321欧拉角，6维末端， 6维向量
		PE323,   // 位置与323欧拉角，6维末端， 6维向量
		PQ,      // 位置与四元数，   6维末端， 7维向量
		PM,      // 位置与位姿矩阵， 6维末端，16维向量
		RE121,   // 121欧拉角，      3维末端， 3维向量
		RE123,   // 123欧拉角，      3维末端， 3维向量
		RE131,   // 131欧拉角，      3维末端， 3维向量
		RE132,   // 132欧拉角，      3维末端， 3维向量
		RE212,   // 212欧拉角，      3维末端， 3维向量
		RE213,   // 213欧拉角，      3维末端， 3维向量
		RE231,   // 231欧拉角，      3维末端， 3维向量
		RE232,   // 232欧拉角，      3维末端， 3维向量
		RE312,   // 312欧拉角，      3维末端， 3维向量
		RE313,   // 313欧拉角，      3维末端， 3维向量
		RE321,   // 321欧拉角，      3维末端， 3维向量
		RE323,   // 323欧拉角，      3维末端， 3维向量
		RQ,      // 四元数，         3维末端， 4维向量
		RM,      // 位姿矩阵，       3维末端， 9维向量
		XYZT,    // x,y,z,theta，    4维末端， 4维向量
		XYZ,     // x,y,z，          3维末端， 3维向量
		RTZ,     // 极坐标r,theta,z，3维末端， 3维向量
		XYT,     // x,y,theta，      3维末端， 3维向量
		XY,      // x,y，            2维末端， 2维向量
		RT,      // 极坐标r,theta，  2维末端， 2维向量
		X,       // 位置x，          1维末端， 1维向量
		Y,       // 位置y，          1维末端， 1维向量
		Z,       // 位置z，          1维末端， 1维向量
		A,       // 位置a，          1维末端， 1维向量
		B,       // 位置b，          1维末端， 1维向量
		C,       // 角度c，          1维末端， 1维向量
		UNKNOWN,
	};
	constexpr auto inline s_pos_type_size(PosType type)noexcept->aris::Size {
		switch (type){
		case PosType::PE121:return 6;
		case PosType::PE123:return 6;
		case PosType::PE131:return 6;
		case PosType::PE132:return 6;
		case PosType::PE212:return 6;
		case PosType::PE213:return 6;
		case PosType::PE231:return 6;
		case PosType::PE232:return 6;
		case PosType::PE312:return 6;
		case PosType::PE313:return 6;
		case PosType::PE321:return 6;
		case PosType::PE323:return 6;
		case PosType::PQ:   return 7;
		case PosType::PM:   return 16;
		case PosType::RE121:return 3;
		case PosType::RE123:return 3;
		case PosType::RE131:return 3;
		case PosType::RE132:return 3;
		case PosType::RE212:return 3;
		case PosType::RE213:return 3;
		case PosType::RE231:return 3;
		case PosType::RE232:return 3;
		case PosType::RE312:return 3;
		case PosType::RE313:return 3;
		case PosType::RE321:return 3;
		case PosType::RE323:return 3;
		case PosType::RQ:   return 4;
		case PosType::RM:   return 9;
		case PosType::XYZT: return 4;
		case PosType::XYZ:  return 3;
		case PosType::RTZ:  return 3;
		case PosType::XYT:  return 3;
		case PosType::XY:   return 2;
		case PosType::RT:   return 2;
		case PosType::X:    return 1;
		case PosType::Y:    return 1;
		case PosType::Z:    return 1;
		case PosType::A:    return 1;
		case PosType::B:    return 1;
		case PosType::C:    return 1;
		case PosType::UNKNOWN:return -1;
		default:return -1;
		}
	}
	auto inline s_pos_type_size(aris::Size n, const PosType* ee_types)noexcept->aris::Size {
		aris::Size size = 0;
		for (Size i = 0; i < n; ++i) {
			size += s_pos_type_size(ee_types[i]);
		}
		return size;
	}
	auto ARIS_API s_pos2pm(PosType type, const double* pos, double* pm)noexcept->void;
	auto ARIS_API s_pm2pos(const double* pm, PosType type, double* pos)noexcept->void;
	auto ARIS_API s_pos2pos(PosType p1_t, const double* pos1, PosType p2_t, double* pos2)->void;
	auto ARIS_API s_pos2pos(Size n, const PosType* p1_t, const double* p1, const PosType* p2_t, double* p2) -> void;


	// 返回位置大小的 size，例如xyz abc距离和角度，它的mag size 就是2
	constexpr auto inline s_pos_type_mag_size(PosType type)noexcept->aris::Size {
		switch (type) {
		case PosType::PE121:return 2;
		case PosType::PE123:return 2;
		case PosType::PE131:return 2;
		case PosType::PE132:return 2;
		case PosType::PE212:return 2;
		case PosType::PE213:return 2;
		case PosType::PE231:return 2;
		case PosType::PE232:return 2;
		case PosType::PE312:return 2;
		case PosType::PE313:return 2;
		case PosType::PE321:return 2;
		case PosType::PE323:return 2;
		case PosType::PQ:   return 2;
		case PosType::PM:   return 2;
		case PosType::RE121:return 1;
		case PosType::RE123:return 1;
		case PosType::RE131:return 1;
		case PosType::RE132:return 1;
		case PosType::RE212:return 1;
		case PosType::RE213:return 1;
		case PosType::RE231:return 1;
		case PosType::RE232:return 1;
		case PosType::RE312:return 1;
		case PosType::RE313:return 1;
		case PosType::RE321:return 1;
		case PosType::RE323:return 1;
		case PosType::RQ:   return 1;
		case PosType::RM:   return 1;
		case PosType::XYZT: return 2;
		case PosType::XYZ:  return 1;
		case PosType::RTZ:  return 1;
		case PosType::XYT:  return 2;
		case PosType::XY:   return 1;
		case PosType::RT:   return 1;
		case PosType::X:    return 1;
		case PosType::Y:    return 1;
		case PosType::Z:    return 1;
		case PosType::A:    return 1;
		case PosType::B:    return 1;
		case PosType::C:    return 1;
		case PosType::UNKNOWN:return -1;
		default:return -1;
		}
	}
	auto inline s_pos_type_mag_size(aris::Size n, const PosType *ee_types)noexcept->aris::Size {
		aris::Size size = 0;
		for (Size i = 0; i < n;++i) {
			size += s_pos_type_mag_size(ee_types[i]);
		}
		return size;
	}

	auto inline s_pos_type_rot_dim(PosType pos_type)->int {
		switch (pos_type) {
		case PosType::PE121:return 3;
		case PosType::PE123:return 3;
		case PosType::PE131:return 3;
		case PosType::PE132:return 3;
		case PosType::PE212:return 3;
		case PosType::PE213:return 3;
		case PosType::PE231:return 3;
		case PosType::PE232:return 3;
		case PosType::PE312:return 3;
		case PosType::PE313:return 3;
		case PosType::PE321:return 3;
		case PosType::PE323:return 3;
		case PosType::PQ:   return 3;
		case PosType::PM:   return 3;
		case PosType::RE121:return 3;
		case PosType::RE123:return 3;
		case PosType::RE131:return 3;
		case PosType::RE132:return 3;
		case PosType::RE212:return 3;
		case PosType::RE213:return 3;
		case PosType::RE231:return 3;
		case PosType::RE232:return 3;
		case PosType::RE312:return 3;
		case PosType::RE313:return 3;
		case PosType::RE321:return 3;
		case PosType::RE323:return 3;
		case PosType::RQ:   return 3;
		case PosType::RM:   return 3;
		case PosType::XYZT: return 1;
		case PosType::XYZ:  return 0;
		case PosType::RTZ:  return 0;
		case PosType::XYT:  return 1;
		case PosType::XY:   return 0;
		case PosType::RT:   return 0;
		case PosType::X:    return 0;
		case PosType::Y:    return 0;
		case PosType::Z:    return 0;
		case PosType::A:    return 1;
		case PosType::B:    return 1;
		case PosType::C:    return 1;
		case PosType::UNKNOWN:return 0;
		default:return 0;
		}
	}
	auto inline s_pos_type_mov_dim(PosType pos_type)->int {
		switch (pos_type) {
		case PosType::PE121:return 3;
		case PosType::PE123:return 3;
		case PosType::PE131:return 3;
		case PosType::PE132:return 3;
		case PosType::PE212:return 3;
		case PosType::PE213:return 3;
		case PosType::PE231:return 3;
		case PosType::PE232:return 3;
		case PosType::PE312:return 3;
		case PosType::PE313:return 3;
		case PosType::PE321:return 3;
		case PosType::PE323:return 3;
		case PosType::PQ:   return 3;
		case PosType::PM:   return 3;
		case PosType::RE121:return 0;
		case PosType::RE123:return 0;
		case PosType::RE131:return 0;
		case PosType::RE132:return 0;
		case PosType::RE212:return 0;
		case PosType::RE213:return 0;
		case PosType::RE231:return 0;
		case PosType::RE232:return 0;
		case PosType::RE312:return 0;
		case PosType::RE313:return 0;
		case PosType::RE321:return 0;
		case PosType::RE323:return 0;
		case PosType::RQ:   return 0;
		case PosType::RM:   return 0;
		case PosType::XYZT: return 3;
		case PosType::XYZ:  return 3;
		case PosType::RTZ:  return 3;
		case PosType::XYT:  return 2;
		case PosType::XY:   return 2;
		case PosType::RT:   return 2;
		case PosType::X:    return 1;
		case PosType::Y:    return 1;
		case PosType::Z:    return 1;
		case PosType::A:    return 0;
		case PosType::B:    return 0;
		case PosType::C:    return 0;
		case PosType::UNKNOWN:return 0;
		default:return 0;
		}
	}

	enum class VelType {
		VA,      // 速度与角速度，   6维末端， 6维向量
		VS,      // 速度旋量，       6维末端， 6维向量
		VE121,   // 位置与121欧拉角，6维末端， 6维向量
		VE123,   // 位置与123欧拉角，6维末端， 6维向量
		VE131,   // 位置与131欧拉角，6维末端， 6维向量
		VE132,   // 位置与132欧拉角，6维末端， 6维向量
		VE212,   // 位置与212欧拉角，6维末端， 6维向量
		VE213,   // 位置与213欧拉角，6维末端， 6维向量
		VE231,   // 位置与231欧拉角，6维末端， 6维向量
		VE232,   // 位置与232欧拉角，6维末端， 6维向量
		VE312,   // 位置与312欧拉角，6维末端， 6维向量
		VE313,   // 位置与313欧拉角，6维末端， 6维向量
		VE321,   // 位置与321欧拉角，6维末端， 6维向量
		VE323,   // 位置与323欧拉角，6维末端， 6维向量
		VQ,      // 位置与四元数，   6维末端， 7维向量
		VM,      // 位置与位姿矩阵， 6维末端，16维向量
		WA,      // 角速度，         3维末端， 3维向量
		WE121,   // 121欧拉角，      3维末端， 3维向量
		WE123,   // 123欧拉角，      3维末端， 3维向量
		WE131,   // 131欧拉角，      3维末端， 3维向量
		WE132,   // 132欧拉角，      3维末端， 3维向量
		WE212,   // 212欧拉角，      3维末端， 3维向量
		WE213,   // 213欧拉角，      3维末端， 3维向量
		WE231,   // 231欧拉角，      3维末端， 3维向量
		WE232,   // 232欧拉角，      3维末端， 3维向量
		WE312,   // 312欧拉角，      3维末端， 3维向量
		WE313,   // 313欧拉角，      3维末端， 3维向量
		WE321,   // 321欧拉角，      3维末端， 3维向量
		WE323,   // 323欧拉角，      3维末端， 3维向量
		WQ,      // 四元数，         3维末端， 4维向量
		WM,      // 位姿矩阵，       3维末端， 9维向量
		DXYZT,   // x,y,z,theta，    4维末端， 4维向量
		DXYZ,    // x,y,z，          3维末端， 3维向量
		DRTZ,    // 极坐标r,theta,z，3维末端， 3维向量
		DXYT,    // x,y,theta，      3维末端， 3维向量
		DXY,     // x,y，            2维末端， 2维向量
		DRT,     // 极坐标r,theta，  2维末端， 2维向量
		DX,      // 位置x，          1维末端， 1维向量
		DY,      // 位置y，          1维末端， 1维向量
		DZ,      // 位置z，          1维末端， 1维向量
		DA,      // 位置a，          1维末端， 1维向量
		DB,      // 位置b，          1维末端， 1维向量
		DC,      // 角度c，          1维末端， 1维向量
		UNKNOWN,
	};
	constexpr auto inline s_vel_type_size(VelType type)noexcept->aris::Size {
		switch (type) {
		case VelType::VA:return 6;
		case VelType::VS:return 6;
		case VelType::VE121:return 6;
		case VelType::VE123:return 6;
		case VelType::VE131:return 6;
		case VelType::VE132:return 6;
		case VelType::VE212:return 6;
		case VelType::VE213:return 6;
		case VelType::VE231:return 6;
		case VelType::VE232:return 6;
		case VelType::VE312:return 6;
		case VelType::VE313:return 6;
		case VelType::VE321:return 6;
		case VelType::VE323:return 6;
		case VelType::VQ:   return 7;
		case VelType::VM:   return 16;
		case VelType::WE121:return 3;
		case VelType::WE123:return 3;
		case VelType::WE131:return 3;
		case VelType::WE132:return 3;
		case VelType::WE212:return 3;
		case VelType::WE213:return 3;
		case VelType::WE231:return 3;
		case VelType::WE232:return 3;
		case VelType::WE312:return 3;
		case VelType::WE313:return 3;
		case VelType::WE321:return 3;
		case VelType::WE323:return 3;
		case VelType::WQ:   return 4;
		case VelType::WM:   return 9;
		case VelType::DXYZT:return 4;
		case VelType::DXYZ: return 3;
		case VelType::DRTZ: return 3;
		case VelType::DXYT: return 3;
		case VelType::DXY:  return 2;
		case VelType::DRT:  return 2;
		case VelType::DX:   return 1;
		case VelType::DY:   return 1;
		case VelType::DZ:   return 1;
		case VelType::DA:   return 1;
		case VelType::DB:   return 1;
		case VelType::DC:   return 1;
		case VelType::UNKNOWN:return -1;
		default:return -1;
		}
	}
	auto inline s_vel_type_size(aris::Size n, const VelType* ee_types)noexcept->aris::Size {
		aris::Size size = 0;
		for (Size i = 0; i < n; ++i) {
			size += s_vel_type_size(ee_types[i]);
		}
		return size;
	}
	auto ARIS_API s_vel2vs(PosType p_t, const double* pos, VelType v_t, const double* vel, double* vs)noexcept->void;
	auto ARIS_API s_vs2vel(PosType p_t, const double* pos, const double* vs, VelType v_t, double* vel)noexcept->void;
	auto ARIS_API s_vel2vel(PosType p1_t, const double* pos1, VelType v1_t, const double* vel1, VelType v2_t, double* vel2)->void;
	auto ARIS_API s_vel2vel(Size n, const PosType *p1_t, const double* p1, const VelType *v1_t, const double* v1, const VelType *v2_t, double* v2)->void;

	enum class AccType {
		AA,      // 速度与角速度，   6维末端， 6维向量
		AS,      // 速度旋量，       6维末端， 6维向量
		AE121,   // 位置与121欧拉角，6维末端， 6维向量
		AE123,   // 位置与123欧拉角，6维末端， 6维向量
		AE131,   // 位置与131欧拉角，6维末端， 6维向量
		AE132,   // 位置与132欧拉角，6维末端， 6维向量
		AE212,   // 位置与212欧拉角，6维末端， 6维向量
		AE213,   // 位置与213欧拉角，6维末端， 6维向量
		AE231,   // 位置与231欧拉角，6维末端， 6维向量
		AE232,   // 位置与232欧拉角，6维末端， 6维向量
		AE312,   // 位置与312欧拉角，6维末端， 6维向量
		AE313,   // 位置与313欧拉角，6维末端， 6维向量
		AE321,   // 位置与321欧拉角，6维末端， 6维向量
		AE323,   // 位置与323欧拉角，6维末端， 6维向量
		AQ,      // 位置与四元数，   6维末端， 7维向量
		AM,      // 位置与位姿矩阵， 6维末端，16维向量
		XA,      // 角加速度，       3维末端， 3维向量
		XE121,   // 121欧拉角，      3维末端， 3维向量
		XE123,   // 123欧拉角，      3维末端， 3维向量
		XE131,   // 131欧拉角，      3维末端， 3维向量
		XE132,   // 132欧拉角，      3维末端， 3维向量
		XE212,   // 212欧拉角，      3维末端， 3维向量
		XE213,   // 213欧拉角，      3维末端， 3维向量
		XE231,   // 231欧拉角，      3维末端， 3维向量
		XE232,   // 232欧拉角，      3维末端， 3维向量
		XE312,   // 312欧拉角，      3维末端， 3维向量
		XE313,   // 313欧拉角，      3维末端， 3维向量
		XE321,   // 321欧拉角，      3维末端， 3维向量
		XE323,   // 323欧拉角，      3维末端， 3维向量
		XQ,      // 四元数，         3维末端， 4维向量
		XM,      // 位姿矩阵，       3维末端， 9维向量
		D2XYZT,   // x,y,z,theta，    4维末端， 4维向量
		D2XYZ,    // x,y,z，          3维末端， 3维向量
		D2RTZ,    // 极坐标r,theta,z，3维末端， 3维向量
		D2XYT,    // x,y,theta，      3维末端， 3维向量
		D2XY,     // x,y，            2维末端， 2维向量
		D2RT,     // 极坐标r,theta，  2维末端， 2维向量
		D2X,      // 位置x，          1维末端， 1维向量
		D2Y,      // 位置y，          1维末端， 1维向量
		D2Z,      // 位置z，          1维末端， 1维向量
		D2A,      // 位置a，          1维末端， 1维向量
		D2B,      // 位置b，          1维末端， 1维向量
		D2C,      // 角度c，          1维末端， 1维向量
		UNKNOWN,
	};
	constexpr auto inline s_acc_type_size(AccType type)noexcept->aris::Size {
		switch (type) {
		case AccType::AA:return 6;
		case AccType::AS:return 6;
		case AccType::AE121:return 6;
		case AccType::AE123:return 6;
		case AccType::AE131:return 6;
		case AccType::AE132:return 6;
		case AccType::AE212:return 6;
		case AccType::AE213:return 6;
		case AccType::AE231:return 6;
		case AccType::AE232:return 6;
		case AccType::AE312:return 6;
		case AccType::AE313:return 6;
		case AccType::AE321:return 6;
		case AccType::AE323:return 6;
		case AccType::AQ:   return 7;
		case AccType::AM:   return 16;
		case AccType::XE121:return 3;
		case AccType::XE123:return 3;
		case AccType::XE131:return 3;
		case AccType::XE132:return 3;
		case AccType::XE212:return 3;
		case AccType::XE213:return 3;
		case AccType::XE231:return 3;
		case AccType::XE232:return 3;
		case AccType::XE312:return 3;
		case AccType::XE313:return 3;
		case AccType::XE321:return 3;
		case AccType::XE323:return 3;
		case AccType::XQ:   return 4;
		case AccType::XM:   return 9;
		case AccType::D2XYZT:return 4;
		case AccType::D2XYZ: return 3;
		case AccType::D2RTZ: return 3;
		case AccType::D2XYT: return 3;
		case AccType::D2XY:  return 2;
		case AccType::D2RT:  return 2;
		case AccType::D2X:   return 1;
		case AccType::D2Y:   return 1;
		case AccType::D2Z:   return 1;
		case AccType::D2A:   return 1;
		case AccType::D2B:   return 1;
		case AccType::D2C:   return 1;
		case AccType::UNKNOWN:return -1;
		default:return -1;
		}
	}
	auto inline s_acc_type_size(aris::Size n, const AccType* ee_types)noexcept->aris::Size {
		aris::Size size = 0;
		for (Size i = 0; i < n; ++i) {
			size += s_acc_type_size(ee_types[i]);
		}
		return size;
	}
	
	auto ARIS_API s_acc2as(PosType p_t, const double* pos, VelType v_t, const double* vel, AccType a_t, const double* acc, double* as)noexcept->void;
	auto ARIS_API s_as2acc(PosType p_t, const double* pos, VelType v_t, const double* vel, const double* as, AccType a_t, double* acc)noexcept->void;
	auto ARIS_API s_acc2acc(PosType p1_t, const double* pos1, VelType v1_t, const double* vel1, AccType a1_t, const double* acc1, AccType a2_t, double* acc2)->void;
	auto ARIS_API s_acc2acc(Size n, const PosType *p1_t, const double* p1, const VelType *v1_t, const double* v1, const AccType *a1_t, const double* a1, const AccType *a2_t, double* a2)->void;

	enum class FceType {
		FT,        // 力与转矩，     6维末端， 6维向量
		FS,        // 力旋量，       6维末端， 6维向量
		FXYZ_TZ,   // 3维转矩，      4维末端， 4维向量
		TXYZ,      // 转矩，         3维末端， 3维向量
		FXYZ,      // 3维力，        3维末端， 3维向量
		FXY_TZ,    // Fxy和Tz，      3维末端， 3维向量
		FXY,       // Fxy，          2维末端， 2维向量
		FX,        // Fx，           1维末端， 1维向量
		FY,        // Fy，           1维末端， 1维向量
		FZ,        // Fz，           1维末端， 1维向量
		TX,        // Tx，           1维末端， 1维向量
		TY,        // Ty，           1维末端， 1维向量
		TZ,        // Tz，           1维末端， 1维向量
		UNKNOWN,
	};
	constexpr auto inline s_fce_type_size(FceType type)noexcept->aris::Size {
		switch (type) {
		case FceType::FT:      return 6;
		case FceType::FS:      return 6;
		case FceType::FXYZ_TZ: return 4;
		case FceType::TXYZ:    return 3;
		case FceType::FXYZ:    return 3;
		case FceType::FXY_TZ:  return 3;
		case FceType::FXY:     return 2;
		case FceType::FX:      return 1;
		case FceType::FY:      return 1;
		case FceType::FZ:      return 1;
		case FceType::TX:      return 1;
		case FceType::TY:      return 1;
		case FceType::TZ:      return 1;
		case FceType::UNKNOWN: return -1;
		default:return -1;
		}
	}
	auto inline s_fce_type_size(aris::Size n, const FceType* ee_types)noexcept->aris::Size {
		aris::Size size = 0;
		for (Size i = 0; i < n; ++i) {
			size += s_fce_type_size(ee_types[i]);
		}
		return size;
	}

	using IkFunc = std::function<int(const void* dh, const double* ee_pos, const double*current_input, std::int64_t which_root, double* input)>;
	//    root_size : 解的大小，例如 puma 的解是 6 维
	//     root_num : 解的个数，例如 puma 的反解的个数为 8 
	//           dh : 反解需要用到的参数
	//         func : 反解函数
	//   which_root : 哪一组解，位于区间 [0 root_num) 时，为特定的解，否则选择最接近 current_root 的这一组解
	//       ee_pos : 末端位置
	//    input_pos : 反解最终的结果保存在此
	//    roots_mem : 计算所需的内存，大小应为 root_size
	// root_periods : 解的周期，例如转动轴周期为 2 PI，如果为 inf 或 nan，则说明没有周期
	// current_root : 当前解，用于在 which_root < 0 或 >= root_num 时，选择解
	auto ARIS_API s_ik(int root_size, std::int64_t root_num, const void* dh, IkFunc func, std::int64_t which_root, const double* ee_pos, double* input_pos, double* roots_mem
		, const double* root_periods = nullptr, const double* current_root = nullptr, const double* input_min = nullptr, const double* input_max = nullptr)->int;

}

#endif
