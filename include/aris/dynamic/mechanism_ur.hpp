#ifndef ARIS_DYNAMIC_MECHANISM_UR_H_
#define ARIS_DYNAMIC_MECHANISM_UR_H_

#include <aris/dynamic/model_solver.hpp>

namespace aris::dynamic
{
	/// @defgroup dynamic_model_group 动力学建模模块
	/// @{
	///
	// UR的机构有如下特点：
	// 1轴和2轴垂直且交于一点： A点
	// 2轴、3轴、4轴平行
	// 4轴、5轴垂直且交于一点： B点
	// 5轴、6轴垂直且交于一点： C点
	//
	//
	//        | ....W1....|....W2...|
	//                              x
	//                              ^    tool_0 坐标系
	//                    y6 轴     |  
	//   ---              - * * * * *----> z
	//    .               *        /
	//    H2              *       y
	//    .               *
	//    .  y4 轴        *
	//   ---  - * * * * * |  z5 轴                       
	//    .   *                    
	//    L2  *                             
	//    .   *                          
	//    .   *                             
	//   ---  -  y3 轴                     
	//    .   *                          
	//	  L1  *                         
	//    .   *                       
	//    .   *                     
	//   ---  -  y2 轴
	//    .   *    
	//    .   *         
	//    H1  |  z1 轴
	//    .   z
	//    .   ^ 
	//    .   |
	//   ---  *----> y
	//       /
	//      x
	//
	//      wobj_0
	struct ARIS_API UrParam{
		// DH PARAM, default is ur5 //
		double H1{ 0.089159 };
		double W1{ 0.13585 - 0.1197 + 0.093 };
		double L1{ 0.425 };
		double L2{ 0.39225 };
		double H2{ -0.09465 };
		double W2{ 0.0823 };

		// 安装方式 //
		// 0, 正常安装，零位时末端法兰盘朝向：地面 x 轴，零位时末端1轴朝向：地面 z 轴
		// 1，顶部吊装，零位时末端法兰盘朝向：地面 x 轴，零位时末端1轴朝向：地面-z 轴
		// 2，侧装向上，零位时末端法兰盘朝向：地面 z 轴，零位时末端1轴朝向：地面 x 轴
		// 3，侧装向下，零位时末端法兰盘朝向：地面-z 轴，零位时末端1轴朝向：地面 x 轴
		int install_method{ 0 };

		// TOOL 0, by default is 321 type
		double tool0_pe[6]{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
		std::string tool0_pe_type;

		// BASE wrt REF, by default is 321 type 
		double base2ref_pe[6]{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
		std::string base2ref_pe_type;

		// axis rotate range
		// 代表关节角度所处在的区间偏移，关节角做更新时，会处于 [-pi + range()*2pi, pi + range()*2pi] 中
		// 例如 axis_range = 0   时，关节角处于 [ -pi, pi ] 中，当 axis_range = 0.5 时，处于[ 0, 2*pi ]中
		// 此外 axis_range = nan 时，此时关节角根据当前位置 x 来计算，结果处于: [ -pi + x, pi + x ] 中
		double axis_range[6]{
			std::numeric_limits<double>::quiet_NaN(),
			std::numeric_limits<double>::quiet_NaN(),
			std::numeric_limits<double>::quiet_NaN(),
			std::numeric_limits<double>::quiet_NaN(),
			std::numeric_limits<double>::quiet_NaN(),
			std::numeric_limits<double>::quiet_NaN()
		};

		// inertia vector, size must be 6
		std::vector<std::array<double, 10> > iv_vec;

		// mot friction vector, size must be 6
		std::vector<std::array<double, 3> > mot_frc_vec;
	};
	auto ARIS_API createModelUr(const UrParam &param)->std::unique_ptr<aris::dynamic::Model>;
	
	struct ARIS_API CalibUrParam {
		double dh_init[6]; // H1 W1 L1 L2 H2 W2
		double mp_offset_init[6]; // motion pos init offset
		
		Size n; // pose num
		const double* pq_obj_in_eye; // n x 7 : pose in 3d eye
		const double* joint_pos; // n x 6 : joint pos

		double dh_result[6];
		double mp_offset_result[6];
		double eye_pq[7];
		double avg_obj_pos_err, avg_obj_quad_err;
	};
	auto ARIS_API calibUrParamBy3DEye(CalibUrParam & param)->int;


	class ARIS_API UrInverseKinematicSolver :public aris::dynamic::InverseKinematicSolver{
	public:
		auto virtual allocateMemory()->void override;
		auto virtual kinPos()->int override;
		auto virtual kinPosPure(const double* output, double* input, int which_root, const double *current_input = nullptr)->int override;

		virtual ~UrInverseKinematicSolver();
		explicit UrInverseKinematicSolver();
		ARIS_DECLARE_BIG_FOUR(UrInverseKinematicSolver);

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};
	///
	/// @}
}

#endif
