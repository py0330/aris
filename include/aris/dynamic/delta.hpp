#ifndef ARIS_DYNAMIC_DELTA_H_
#define ARIS_DYNAMIC_DELTA_H_

#include <array>
#include <aris/dynamic/model_solver.hpp>

namespace aris::dynamic
{
	//---------------------------------------------------------------------------------------------
	// 包含4个自由度。每个支联4个杆件  link1~4
	// 尺寸示意：
	//                   上平台 
	//                      base(长度a，平台中心到关节的距离)
	//                    *-------o      -> R1
	//                             \     -> link1 (长度b) 
	//                              \o   -> S2
	//                              /|   -> link1 (长度c)
	//                             / o   -> S3
	//                            / /
	//               S4 <-       o /     -> link2&3 (长度d)
	//           转轴Re <-  |----|/         Re 连接并联末端 up 与最终的末端 ee，从而变成 4 自由度              
	//               S5 <-       o
	//                       ee(长度e)
	//--------------------------------------------------------------------------------------------- 
	// 零位：
	//                 (0,0,0)           o  
	//                    |-------o------|       ->  x方向
	//                                 / o
	//                                / /
	//                               / /
	//                              o /
	//                    |---------|/
	//                              o
	//----------------------------------------------------------------------------------------------
	// 俯视图：
	//                       y
	//                       ^
	//                       |
	//
	//                    o
	//                     \         
	//                      \       
	//                       *------o      -> x
	//                      /
	//                     /
	//                    o
	//
	//
	
	
	/// @defgroup dynamic_model_group 动力学建模模块
	/// @{
	///
	struct ARIS_API DeltaParam{
		// DH PARAM //
		double a{ 0.0 };
		double b{ 0.0 };
		double c{ 0.0 };
		double d{ 0.0 };
		double e{ 0.0 };

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
	auto ARIS_API createModelDelta(const DeltaParam &param)->std::unique_ptr<aris::dynamic::Model>;

	class ARIS_API DeltaInverseKinematicSolver :public aris::dynamic::InverseKinematicSolver{
	public:
		auto virtual allocateMemory()->void override;
		auto virtual kinPos()->int override;

		virtual ~DeltaInverseKinematicSolver();
		explicit DeltaInverseKinematicSolver();
		ARIS_DECLARE_BIG_FOUR(DeltaInverseKinematicSolver);

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};
	///
	/// @}
}

#endif
