#ifndef ARIS_DYNAMIC_SCARA_H_
#define ARIS_DYNAMIC_SCARA_H_

#include <array>
#include <aris/dynamic/model_solver.hpp>

namespace aris::dynamic{
	/// @defgroup dynamic_model_group 动力学建模模块
	/// @{
	///
	
	//---------------------------------------------------------------------------------------------
	// 包含4个自由度。共4个杆件  link1~4
	// 尺寸示意：
	//
	//                           
	//                                  y
	//                                ^
	//                                |
	//           o--------o--------o  *---> x
	//        origin    a       b
	//---------------------------------------------------------------------------------------------
	struct ARIS_API ScaraParam{
		// DH PARAM //
		double a{ 0.0 };
		double b{ 0.0 };

		// 安装方式
		// 0, 正常安装，零位时小臂朝向：地面 x 轴，零位时末端1轴朝向：地面 z 轴
		// 1，顶部吊装，零位时小臂朝向：地面 x 轴，零位时末端1轴朝向：地面-z 轴
		// 2，侧装向上，零位时小臂朝向：地面 z 轴，零位时末端1轴朝向：地面 x 轴
		// 3，侧装向下，零位时小臂朝向：地面-z 轴，零位时末端1轴朝向：地面 x 轴
		int install_method{ 0 };

		// TOOL 0 位置与欧拉角，默认为 321 形式
		double tool0_pe[6]{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
		std::string tool0_pe_type{ "321" };

		// BASE   位置与欧拉角，默认为 321 形式
		double base2ref_pe[6]{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
		std::string base2ref_pe_type{ "321" };

		// axis rotate range
		// 代表关节角度所处在的区间偏移，关节角做更新时，会处于 [-pi + range()*2pi, pi + range()*2pi] 中
		// 例如 axis_range = 0   时，关节角处于 [ -pi, pi ] 中，当 axis_range = 0.5 时，处于[ 0, 2*pi ]中
		// 此外 axis_range = nan 时，此时关节角根据当前位置 x 来计算，结果处于: [ -pi + x, pi + x ] 中
		double axis_range[4]{
			std::numeric_limits<double>::quiet_NaN(),
			std::numeric_limits<double>::quiet_NaN(),
			std::numeric_limits<double>::quiet_NaN(),
			std::numeric_limits<double>::quiet_NaN(),
		};

		// inertia vector, size must be 6
		std::vector<std::array<double, 10> > iv_vec;

		// mot friction vector, size must be 6
		std::vector<std::array<double, 3> > mot_frc_vec;

		// 暂时未启用 //
		double pitch{ 0.0 };
	};
	class ARIS_API ScaraForwardKinematicSolver :public aris::dynamic::ForwardKinematicSolver {
	public:
		auto virtual kinPos()->int override {
			// link1~4 //
			double pe[6]{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
			
			if (&model()->jointPool()[0].makI()->fatherPart() == &model()->partPool()[1]) {
				pe[5] = model()->motionPool()[0].mpInternal();
				model()->jointPool()[0].makI()->setPe(*model()->jointPool()[0].makJ(), pe, "123");
			}
			else {
				pe[5] = -model()->motionPool()[0].mpInternal();
				model()->jointPool()[0].makJ()->setPe(*model()->jointPool()[0].makI(), pe, "123");
			}
				
			if (&model()->jointPool()[1].makI()->fatherPart() == &model()->partPool()[2]) {
				pe[5] = model()->motionPool()[1].mpInternal();
				model()->jointPool()[1].makI()->setPe(*model()->jointPool()[1].makJ(), pe, "123");
			}
			else {
				pe[5] = -model()->motionPool()[1].mpInternal();
				model()->jointPool()[1].makJ()->setPe(*model()->jointPool()[1].makI(), pe, "123");
			}
				
			pe[5] = 0.0;
			if (&model()->jointPool()[2].makI()->fatherPart() == &model()->partPool()[3]) {
				pe[2] = model()->motionPool()[2].mpInternal();
				model()->jointPool()[2].makI()->setPe(*model()->jointPool()[2].makJ(), pe, "123");
			}
			else {
				pe[2] = -model()->motionPool()[2].mpInternal();
				model()->jointPool()[2].makJ()->setPe(*model()->jointPool()[2].makI(), pe, "123");
			}
				
			if (&model()->jointPool()[3].makI()->fatherPart() == &model()->partPool()[4]) {
				pe[2] = model()->motionPool()[3].mpInternal() / 2 / PI
					* dynamic_cast<ScrewJoint&>(model()->jointPool()[3]).pitch();
				pe[5] = model()->motionPool()[3].mpInternal();
				model()->jointPool()[3].makI()->setPe(*model()->jointPool()[3].makJ(), pe, "123");
			}
			else {
				pe[2] = -model()->motionPool()[3].mpInternal() / 2 / PI
					* dynamic_cast<ScrewJoint&>(model()->jointPool()[3]).pitch();
				pe[5] = -model()->motionPool()[3].mpInternal();
				model()->jointPool()[3].makJ()->setPe(*model()->jointPool()[3].makI(), pe, "123");
			}
			
			model()->generalMotionPool()[0].updP();

			return 0;
		}
		ScaraForwardKinematicSolver() = default;
	};
	class ARIS_API ScaraInverseKinematicSolver :public aris::dynamic::InverseKinematicSolver {
	public:
		auto virtual allocateMemory()->void override;
		auto virtual kinPos()->int override;
		auto virtual kinPosPure(const double* output, double* input, int which_root)->int override;

		virtual ~ScaraInverseKinematicSolver();
		explicit ScaraInverseKinematicSolver();
		ARIS_DECLARE_BIG_FOUR(ScaraInverseKinematicSolver);
	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};
	auto ARIS_API createModelScara(const ScaraParam &param)->std::unique_ptr<aris::dynamic::Model>;
	auto ARIS_API createModelPlanarScara(const ScaraParam &param)->std::unique_ptr<aris::dynamic::Model>;

	// \brief 求解平面内2点法标定，适用于scara 和 delta等4轴机器人
	// scara:      4维末端的机器人，例如scara或delta
	// points:      2组数据6个数：[x1 y1 c1 x2 y2 c2]
	// tool_name:   需要标定的工具坐标系名称
	auto ARIS_API calibModelByTwoPoints(aris::dynamic::Model& scara, const double* points, std::string_view tool_name)->int;


	///
	/// @}
}

#endif
