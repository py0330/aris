#ifndef ARIS_DYNAMIC_MECHANISM_STEWART_H_
#define ARIS_DYNAMIC_MECHANISM_STEWART_H_

#include <aris/dynamic/model_solver.hpp>

namespace aris::dynamic
{
	/// @defgroup dynamic_model_group 动力学建模模块
	/// @{
	///

	auto ARIS_API createModelStewart()->std::unique_ptr<aris::dynamic::Model>;

	class ARIS_API StewartInverseKinematicSolver :public aris::dynamic::InverseKinematicSolver
	{
	public:
		auto virtual allocateMemory()->void override;
		auto virtual kinPos()->int override;

		virtual ~StewartInverseKinematicSolver();
		explicit StewartInverseKinematicSolver();
		ARIS_DECLARE_BIG_FOUR(StewartInverseKinematicSolver);

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};
	///
	/// @}
}

#endif
