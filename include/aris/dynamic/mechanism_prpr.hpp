#ifndef ARIS_DYNAMIC_MECHANISM_PRPR_H_
#define ARIS_DYNAMIC_MECHANISM_PRPR_H_

#include <array>
#include <aris/dynamic/model_solver.hpp>

namespace aris::dynamic{
	//---------------------------------------------------------------------------------------------
	// 包含4个自由度。共4个杆件  link1~4
	// 尺寸示意：
	//
	//           |<-      b     ->|     
	//
	//                   P        R           
	//           ****** |-- ***** | *** EE    ---
	//           *                             ^
	//           *                             |
	//      R    |
	//           *         
	//           *                             a
	//           *        
	//      P    |
	//           *
	//           ^
	//           |                             |
	//           *---> x                      ---
	//        origin         
	//---------------------------------------------------------------------------------------------
	
	/// @defgroup dynamic_model_group 动力学建模模块
	/// @{
	///
	struct ARIS_API PrprParam{
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

	auto ARIS_API createModelPrpr(const PrprParam &param)->std::unique_ptr<aris::dynamic::Model>;

	///
	/// @}
}

#endif
