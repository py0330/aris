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
		PE313,   // 位置与313欧拉角，6维末端， 6维向量
		PE321,   // 位置与321欧拉角，6维末端， 6维向量
		PE123,   // 位置与123欧拉角，6维末端， 6维向量
		PQ,      // 位置与四元数，   6维末端， 7维向量
		PM,      // 位置与位姿矩阵， 6维末端，16维向量
		RE313,   // 313欧拉角，      3维末端， 3维向量
		RE321,   // 321欧拉角，      3维末端， 3维向量
		RE123,   // 123欧拉角，      3维末端， 3维向量
		RQ,      // 四元数，         3维末端， 4维向量
		RM,      // 位姿矩阵，       3维末端， 9维向量
		XYZT,    // x,y,z,theta，    4维末端， 4维向量
		XYZ,     // x,y,z，          3维末端， 3维向量
		RTZ,     // 极坐标r,theta,z，3维末端， 3维向量
		XYT,     // x,y,theta，      3维末端， 3维向量
		XY,      // x,y，            2维末端， 2维向量
		RT,      // 极坐标r,theta，  2维末端， 2维向量
		X,       // 位置x，          1维末端， 1维向量
		A,       // 角度a，          1维末端， 1维向量
		UNKNOWN,
	};
	auto inline s_ee_type_pos_size(EEType type)noexcept->aris::Size {
		switch (type){
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
		case EEType::RE313:
			return 3;
		case EEType::RE321:
			return 3;
		case EEType::RE123:
			return 3;
		case EEType::RQ:
			return 4;
		case EEType::RM:
			return 9;
		case EEType::XYZT:
			return 4;
		case EEType::XYZ:
			return 3;
		case EEType::RTZ:
			return 3;
		case EEType::XYT:
			return 3;
		case EEType::XY:
			return 2;
		case EEType::RT:
			return 2;
		case EEType::X:
			return 1;
		case EEType::A:
			return 1;
		case EEType::UNKNOWN:
			return -1;
		default:
			return -1;
		}
	}
	auto inline s_ee_type_pos_size(aris::Size n, const EEType* ee_types)noexcept->aris::Size {
		aris::Size size = 0;
		for (Size i = 0; i < n; ++i) {
			size += s_ee_type_pos_size(ee_types[i]);
		}
		return size;
	}
	auto ARIS_API s_ee_pos2pm(EEType type, const double* pos, double* pm)noexcept->void;
	auto ARIS_API s_ee_pm2pos(EEType type, const double* pm, double* pos)noexcept->void;

	auto inline s_ee_type_vel_dim(EEType type)noexcept->aris::Size {
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
		case EEType::RE313:
			return 1;
		case EEType::RE321:
			return 1;
		case EEType::RE123:
			return 1;
		case EEType::RQ:
			return 1;
		case EEType::RM:
			return 1;
		case EEType::XYZT:
			return 2;
		case EEType::XYZ:
			return 1;
		case EEType::RTZ:
			return 2;
		case EEType::XYT:
			return 2;
		case EEType::XY:
			return 1;
		case EEType::RT:
			return 2;
		case EEType::X:
			return 1;
		case EEType::A:
			return 1;
		case EEType::UNKNOWN:
			return -1;
		default:
			return -1;
		}
	}
	auto inline s_ee_type_vel_dim(aris::Size n, const EEType *ee_types)noexcept->aris::Size {
		aris::Size size = 0;
		for (Size i = 0; i < n;++i) {
			size += s_ee_type_vel_dim(ee_types[i]);
		}
		return size;
	}

	enum class EEVelType {
		VA,      // 速度与角速度，   6维末端， 6维向量
		VS,      // 速度旋量，       6维末端， 6维向量
		VE313,   // 位置与313欧拉角，6维末端， 6维向量
		VE321,   // 位置与321欧拉角，6维末端， 6维向量
		VE123,   // 位置与123欧拉角，6维末端， 6维向量
		VQ,      // 位置与四元数，   6维末端， 7维向量
		VM,      // 位置与位姿矩阵， 6维末端，16维向量
		WE313,   // 313欧拉角，      3维末端， 3维向量
		WE321,   // 321欧拉角，      3维末端， 3维向量
		WE123,   // 123欧拉角，      3维末端， 3维向量
		WQ,      // 四元数，         3维末端， 4维向量
		WM,      // 位姿矩阵，       3维末端， 9维向量
		DXYZT,   // x,y,z,theta，    4维末端， 4维向量
		DXYZ,    // x,y,z，          3维末端， 3维向量
		DRTZ,    // 极坐标r,theta,z，3维末端， 3维向量
		DXYT,    // x,y,theta，      3维末端， 3维向量
		DXY,     // x,y，            2维末端， 2维向量
		DRT,     // 极坐标r,theta，  2维末端， 2维向量
		DX,      // 位置x，          1维末端， 1维向量
		DA,      // 角度a，          1维末端， 1维向量
		UNKNOWN,
	};
	auto inline s_ee_type_vel_size(EEType type)noexcept->aris::Size {
		switch (type) {
		case EEType::PE313:
			return 6;
		case EEType::PE321:
			return 6;
		case EEType::PE123:
			return 6;
		case EEType::PQ:
			return 6;
		case EEType::PM:
			return 6;
		case EEType::RE313:
			return 3;
		case EEType::RE321:
			return 3;
		case EEType::RE123:
			return 3;
		case EEType::RQ:
			return 3;
		case EEType::RM:
			return 3;
		case EEType::XYZT:
			return 4;
		case EEType::XYZ:
			return 3;
		case EEType::RTZ:
			return 3;
		case EEType::XYT:
			return 3;
		case EEType::XY:
			return 2;
		case EEType::RT:
			return 2;
		case EEType::X:
			return 1;
		case EEType::A:
			return 1;
		case EEType::UNKNOWN:
			return -1;
		default:
			return -1;
		}
	}
	auto inline s_ee_type_vel_size(aris::Size n, const EEType* ee_types)noexcept->aris::Size {
		aris::Size size = 0;
		for (Size i = 0; i < n; ++i) {
			size += s_ee_type_vel_size(ee_types[i]);
		}
		return size;
	}

	enum class EEAccType {
		AA,      // 速度与角速度，   6维末端， 6维向量
		AS,      // 速度旋量，       6维末端， 6维向量
		AE313,   // 位置与313欧拉角，6维末端， 6维向量
		AE321,   // 位置与321欧拉角，6维末端， 6维向量
		AE123,   // 位置与123欧拉角，6维末端， 6维向量
		AQ,      // 位置与四元数，   6维末端， 7维向量
		AM,      // 位置与位姿矩阵， 6维末端，16维向量
		XE313,   // 313欧拉角，      3维末端， 3维向量
		XE321,   // 321欧拉角，      3维末端， 3维向量
		XE123,   // 123欧拉角，      3维末端， 3维向量
		XQ,      // 四元数，         3维末端， 4维向量
		XM,      // 位姿矩阵，       3维末端， 9维向量
		D2XYZT,   // x,y,z,theta，    4维末端， 4维向量
		D2XYZ,    // x,y,z，          3维末端， 3维向量
		D2RTZ,    // 极坐标r,theta,z，3维末端， 3维向量
		D2XYT,    // x,y,theta，      3维末端， 3维向量
		D2XY,     // x,y，            2维末端， 2维向量
		D2RT,     // 极坐标r,theta，  2维末端， 2维向量
		D2X,      // 位置x，          1维末端， 1维向量
		D2A,      // 角度a，          1维末端， 1维向量
		UNKNOWN,
	};
	auto inline s_ee_type_acc_size(EEType type)noexcept->aris::Size {
		return s_ee_type_vel_size(type);
	}
	auto inline s_ee_type_acc_size(aris::Size n, const EEType* ee_types)noexcept->aris::Size {
		aris::Size size = 0;
		for (Size i = 0; i < n; ++i) {
			size += s_ee_type_acc_size(ee_types[i]);
		}
		return size;
	}
	
	enum class EEFceType {
		FT,        // 力与转矩，     6维末端， 6维向量
		FS,        // 力旋量，       6维末端， 6维向量
		TORQUE,    // 转矩，         3维末端， 3维向量
		FORCE,     // 3维力，        3维末端， 3维向量
		FXYZ_TZ,   // 3维转矩，      4维末端， 4维向量
		FXY_TZ,    // Fxy和Tz，      3维末端， 3维向量
		FX,        // Fx，           1维末端， 1维向量
		TZ,        // Tz，           1维末端， 1维向量
		UNKNOWN,
	};
	auto inline s_ee_type_fce_size(EEType type)noexcept->aris::Size {
		return s_ee_type_vel_size(type);
	}
	auto inline s_ee_type_fce_size(aris::Size n, const EEType* ee_types)noexcept->aris::Size {
		aris::Size size = 0;
		for (Size i = 0; i < n; ++i) {
			size += s_ee_type_acc_size(ee_types[i]);
		}
		return size;
	}

	using IkFunc = std::function<int(const void* dh, const double* ee_pos, const double*current_input, int which_root, double* input)>;
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
	auto ARIS_API s_ik(int root_size, int root_num, const void* dh, IkFunc func, int which_root, const double* ee_pos, double* input_pos, double* roots_mem, const double* root_periods = nullptr, const double* current_root = nullptr)->int;

}

#endif
