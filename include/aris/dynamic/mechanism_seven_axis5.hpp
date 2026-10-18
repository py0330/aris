#ifndef ARIS_DYNAMIC_MECHANISM_SEVEN_AXIS5_H_
#define ARIS_DYNAMIC_MECHANISM_SEVEN_AXIS5_H_

#include <array>
#include <aris/dynamic/model_solver.hpp>

namespace aris::dynamic{
	/// @defgroup dynamic_model_group 动力学建模模块
	/// @{
	///
	/// 

	// 带有偏移的七轴机器人（转轴序列 z-x-z-x-z-x-y，偏移均为相对上一关节的 z 方向）
	// 
	// 七轴机器人构型：
	//
	//      EE
	//      o        y      ---
	//                       |
	//                       d3
	//                       |
	//     ---       x      ---
	//      |        z       |
	//                       d2
	//                       |
	//     ---       x      ---                                                            
	//                       |
	//                       d1
	//      |        z       |   
	//     ---       x      ---
	//      |        z    
	//     BASE
	//
	//  A 坐标系为前3轴的交点， z 轴和 1 轴平行， y 轴和 2 轴平行
	//  D 坐标系为 5 6 7 三根轴的交点，零位下与 A 坐标系方向一致
	//  
	//  
	// 
	// 各关节转轴（零位时，相对 base）依次为：
	//   z(0) x(0) z(0) x(d1) z(0) x(d2) y(d3)
	// 其中括号内为相对上一关节的 z 方向偏移。
	//
	// 反解时，6轴（从1开始计数，倒数第2轴）的角度已知
	struct ARIS_API SevenAxisParam5{
		// 连杆偏移（均为相对上一关节的 z 方向偏移）//
		double d1{ 0.0 };
		double d2{ 0.0 };
		double d3{ 0.0 };

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
		std::string base2ref_pe_type = "321";

		// axis rotate range
		// 代表关节角度所处在的区间偏移，关节角做更新时，会处于 [-pi + range()*2pi, pi + range()*2pi] 中
		// 例如 axis_range = 0   时，关节角处于 [ -pi, pi ] 中，当 axis_range = 0.5 时，处于[ 0, 2*pi ]中
		// 此外 axis_range = nan 时，此时关节角根据当前位置 x 来计算，结果处于: [ -pi + x, pi + x ] 中
		double axis_range[7]{
			std::numeric_limits<double>::quiet_NaN(),
			std::numeric_limits<double>::quiet_NaN(),
			std::numeric_limits<double>::quiet_NaN(),
			std::numeric_limits<double>::quiet_NaN(),
			std::numeric_limits<double>::quiet_NaN(),
			std::numeric_limits<double>::quiet_NaN(),
			std::numeric_limits<double>::quiet_NaN()
		};

		// inertia vector, size must be 7
		std::vector<std::array<double, 10> > iv_vec;

		// mot friction vector, size must be 7
		std::vector<std::array<double, 3> > mot_frc_vec;
	};
	auto ARIS_API createModelSevenAxis5(const SevenAxisParam5 &param)->std::unique_ptr<aris::dynamic::Model>;

	class ARIS_API SevenAxisInverseKinematicSolver5 :public aris::dynamic::InverseKinematicSolver{
	public:
		auto virtual allocateMemory()->void override;
		auto virtual kinPos()->int override;
		auto virtual kinPosPure(const double* output, double* input, std::int64_t which_root, const double* current_input = nullptr)const->int override;

		virtual ~SevenAxisInverseKinematicSolver5();
		explicit SevenAxisInverseKinematicSolver5();
		ARIS_DECLARE_BIG_FOUR(SevenAxisInverseKinematicSolver5);

	private:
		friend auto ARIS_API createModelSevenAxis5(const SevenAxisParam5& param)->std::unique_ptr<aris::dynamic::Model>;
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};
	///
	/// @}
}

#endif
