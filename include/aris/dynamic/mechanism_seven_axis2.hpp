#ifndef ARIS_DYNAMIC_MECHANISM_SEVEN_AXIS2_H_
#define ARIS_DYNAMIC_MECHANISM_SEVEN_AXIS2_H_

#include <array>
#include <aris/dynamic/model_solver.hpp>

namespace aris::dynamic{
	/// @defgroup dynamic_model_group 动力学建模模块
	/// @{
	///
	struct ARIS_API SevenAxisParam2{
		// DH PARAM //
		double d1{ 0.0 };
		double a2{ 0.0 };
		double d3{ 0.0 };
		double d5{ 0.0 };

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
			std::numeric_limits<double>::quiet_NaN()
		};

		// inertia vector, size must be 7
		std::vector<std::array<double, 10> > iv_vec;

		// mot friction vector, size must be 7
		std::vector<std::array<double, 3> > mot_frc_vec;
	};
	auto ARIS_API createModelSevenAxis2(const SevenAxisParam2 &param)->std::unique_ptr<aris::dynamic::Model>;

	class ARIS_API SevenAxisInverseKinematicSolver2 :public aris::dynamic::InverseKinematicSolver
	{
	public:
		auto virtual allocateMemory()->void override;
		auto virtual kinPos()->int override;
		auto virtual kinPosPure(const double* output, double* input, int which_root, const double* current_input = nullptr)->int override;
		auto setWhichRoot(int root_of_0_to_7)->void;
		auto whichRoot()->int;

		virtual ~SevenAxisInverseKinematicSolver2();
		explicit SevenAxisInverseKinematicSolver2();
		ARIS_DECLARE_BIG_FOUR(SevenAxisInverseKinematicSolver2);

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};
	///
	/// @}
}

#endif
