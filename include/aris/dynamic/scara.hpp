#ifndef ARIS_DYNAMIC_SCARA_H_
#define ARIS_DYNAMIC_SCARA_H_

#include <array>
#include <aris/dynamic/model_solver.hpp>

namespace aris::dynamic{
	//---------------------------------------------------------------------------------------------
	// 包含4个自由度。共4个杆件  link1~4
	// 尺寸示意：
	//
	//                           o
	//                           |        y
	//                           |  b     ^
	//                           |        |
	//           o---------------o        *---> x
	//        origin         a
	//---------------------------------------------------------------------------------------------
	
	/// @defgroup dynamic_model_group 动力学建模模块
	/// @{
	///
	struct ARIS_API ScaraParam{
		// DH PARAM //
		double a{ 0.0 };
		double b{ 0.0 };

		// TOOL 0, by default is 321 type
		double tool0_pe[6]{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
		std::string tool0_pe_type;

		// BASE wrt REF, by default is 321 type 
		double base2ref_pe[6]{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
		std::string base2ref_pe_type;

		// inertia vector, size must be 6
		std::vector<std::array<double, 10> > iv_vec;

		// mot friction vector, size must be 6
		std::vector<std::array<double, 3> > mot_frc_vec;
	};


	auto ARIS_API createModelScara(const ScaraParam &param)->std::unique_ptr<aris::dynamic::Model>;

	// \brief 求解平面内2点法标定，适用于scara 和 delta等4轴机器人
	// scara:      4维末端的机器人，例如scara或delta
	// points:      2组数据6个数：[x1 y1 c1 x2 y2 c2]
	// tool_name:   需要标定的工具坐标系名称
	auto ARIS_API calibModelByTwoPoints(aris::dynamic::Model& scara, const double* points, std::string_view tool_name)->int;


	///
	/// @}
}

#endif
