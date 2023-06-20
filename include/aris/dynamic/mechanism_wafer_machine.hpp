#ifndef ARIS_DYNAMIC_MECHANISM_WAFER_MACHINE_H_
#define ARIS_DYNAMIC_MECHANISM_WAFER_MACHINE_H_

#include <array>
#include <aris/dynamic/model_solver.hpp>

namespace aris::dynamic{
	/// @defgroup dynamic_model_group 动力学建模模块
	/// @{
	///
	// 晶圆机构型：
	//         EE(tool0)
	//        y
	//       /
	//      *----> x   
	//      | ... c ... |  L3
	//            L2    |         
	//  --- | ... b ... |  
	//   L1    
	//   .  |
	//   . |||    p1
	//   a ---
	//   .
	//   .  z
	//   .  ^  y
	//   .  | /
	//  --- *----> x  
	//     O(wobj0)
	//
	struct ARIS_API WaferMachineParam{
		// DH PARAM //
		double a{ 0.1 };
		double b{ 0.5 };
		double c{ 0.5 };

		// 安装方式 //
		// 0, 正常安装，零位时末端法兰盘朝向：地面 x 轴，零位时末端1轴朝向：地面 z 轴
		// 1，顶部吊装，零位时末端法兰盘朝向：地面 x 轴，零位时末端1轴朝向：地面-z 轴
		// 2，侧装向上，零位时末端法兰盘朝向：地面 z 轴，零位时末端1轴朝向：地面 x 轴
		// 3，侧装向下，零位时末端法兰盘朝向：地面-z 轴，零位时末端1轴朝向：地面 x 轴
		int install_method{0};

		// TOOL 相对于 456 轴交点处的位置与欧拉角，默认为 321 形式
		double tool0_pe[6]{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
		std::string tool0_pe_type{ "321" };

		// BASE 在地面的位置与欧拉角，默认为 321 形式
		double base2ref_pe[6]{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
		std::string base2ref_pe_type{ "321" };

		// axis rotate range
		// 代表关节角度所处在的区间偏移，关节角做更新时，会处于 [-pi + range()*2pi, pi + range()*2pi] 中
		// 例如 axis_range = 0   时，关节角处于 [ -pi, pi ] 中，当 axis_range = 0.5 时，处于[ 0, 2*pi ]中
		// 此外 axis_range = nan 时，此时关节角根据当前位置 x 来计算，结果处于: [ -pi + x, pi + x ] 中
		double axis_range[3]{ 
			std::numeric_limits<double>::quiet_NaN(),
			std::numeric_limits<double>::quiet_NaN(),
			std::numeric_limits<double>::quiet_NaN(),
		};

		// inertia vector, size must be 6
		std::vector<std::array<double, 10> > iv_vec;

		// mot friction vector, size must be 6
		std::vector<std::array<double, 3> > mot_frc_vec;
	};
	class ARIS_API WaferMachineInverseKinematicSolver :public aris::dynamic::InverseKinematicSolver {
	public:
		auto virtual allocateMemory()->void override;
		auto virtual kinPos()->int override;
		auto virtual kinPosPure(const double *output, double *input, int which_root)->int override;

		virtual ~WaferMachineInverseKinematicSolver();
		explicit WaferMachineInverseKinematicSolver();
		ARIS_DECLARE_BIG_FOUR(WaferMachineInverseKinematicSolver);

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};
	class ARIS_API WaferMachineForwardKinematicSolver :public aris::dynamic::ForwardKinematicSolver {
	public:
		auto virtual allocateMemory()->void override;
		auto virtual kinPos()->int override;
		auto virtual kinPosPure(const double* output, double* input, int which_root)->int override;

		virtual ~WaferMachineForwardKinematicSolver();
		explicit WaferMachineForwardKinematicSolver();
		ARIS_DECLARE_BIG_FOUR(WaferMachineForwardKinematicSolver);

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};
	auto ARIS_API createModelWaferMachine(const WaferMachineParam&param)->std::unique_ptr<aris::dynamic::Model>;


	///
	/// @}
}

#endif
