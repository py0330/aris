#ifndef ARIS_DYNAMIC_UR_H_
#define ARIS_DYNAMIC_UR_H_

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
	//                                           x    EE
	//                                           ^ z
	//                                         \ |/
	//                                         . *----> y
	//                                       W2 /
	//                                       . /
	//                                      \ /
	//                                   --- o y6
	//                                     . *
	//                                    H2 *
	//                                     . *
	//                                   --\ | z5
	//                                    . * 
	//                                  W1 * 
	//                                  . * 
	//        y2                       \ * 
	//   ---  o *** L1 *** o *** L2 *** o                  
	//    .   |                                                          
	//    .   z1           y3           y4                              
	//    H1
	//    .   z
	//    .   ^ y
	//    .   |/
	//   ---  *----> x
	//       O
	struct ARIS_API UrParam{
		// DH PARAM, default is ur5 //
		double H1{ 0.089159 };
		double W1{ 0.13585 - 0.1197 + 0.093 };
		double L1{ 0.425 };
		double L2{ 0.39225 };
		double H2{ -0.09465 };
		double W2{ 0.0823 };

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
	auto ARIS_API createModelUr(const UrParam &param)->std::unique_ptr<aris::dynamic::Model>;




	class ARIS_API UrInverseKinematicSolver :public aris::dynamic::InverseKinematicSolver
	{
	public:
		auto virtual allocateMemory()->void override;
		auto virtual kinPos()->int override;
		auto setWhichRoot(int root_of_0_to_7)->void;
		auto whichRoot()->int;

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
