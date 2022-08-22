#ifndef ARIS_DYNAMIC_PUMA_H_
#define ARIS_DYNAMIC_PUMA_H_

#include <array>
#include <aris/dynamic/model_solver.hpp>

namespace aris::dynamic{
	/// @defgroup dynamic_model_group 动力学建模模块
	/// @{
	///
	// puma机器人构型：
	//                                                        EE(tool0)
	//                                                      y
	//                         x4           y5    x6       /
	//                       \ - ********** o *** - ***  *----> z                                
	//                    d3 * | ... d4 ... | ... d5 ... |             
	//                  \  *                             x
	//               --- *                              
	//               a3  *                                
	//                .  *                              
	//               --- o  y3
	//                .  *                               
	//               a2  *                                   
	//                .  *                               
	//  --- | *** a1 *** o                               
	//   .  z1           y2	
	//  d1
	//   .  z
	//   .  ^  y
	//   .  | /
	//  --- *----> x  
	//     O(wobj0)
	//
	struct ARIS_API PumaParam{
		// DH PARAM //
		double d1{ 0.0 };
		double a1{ 0.0 };
		double a2{ 0.0 };
		double d3{ 0.0 };
		double a3{ 0.0 };
		double d4{ 0.0 };

		// 安装方式
		// 0, 正常安装，零位时末端法兰盘朝向：地面 x 轴，零位时末端1轴朝向：地面 z 轴
		// 1，顶部吊装，零位时末端法兰盘朝向：地面 x 轴，零位时末端1轴朝向：地面-z 轴
		// 2，左侧吊装，零位时末端法兰盘朝向：地面-z 轴，零位时末端1轴朝向：地面 x 轴
		// 3，右侧吊装，零位时末端法兰盘朝向：地面-z 轴，零位时末端1轴朝向：地面-x 轴
		int install_method{0};

		// TOOL 0, by default is 321 type
		double tool0_pe[6]{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
		std::string tool0_pe_type;

		// BASE wrt REF, by default is 321 type 
		double base2ref_pe[6]{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
		std::string base2ref_pe_type;

		// axis rotate range
		// 代表关节角度所处在的区间偏移，关节角做更新时，会处于 [-pi + range()*2pi, pi + range()*2pi] 中
		// 例如，当 axis_range = 0 时，关节角处于 [ -pi, pi ] 中，当 axis_range = 0.5 时，处于[ 0, 2*pi ]中
		// 如果axis_range = nan，此时关节角根据当前位置 x 来计算，结果处于: [ -pi + x, pi + x ] 中
		double axis_range[6]{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

		// inertia vector, size must be 6
		std::vector<std::array<double, 10> > iv_vec;

		// mot friction vector, size must be 6
		std::vector<std::array<double, 3> > mot_frc_vec;
	};
	auto ARIS_API createModelPuma(const PumaParam &param)->std::unique_ptr<aris::dynamic::Model>;


	///
	/// @}
}

#endif
