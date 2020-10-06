#ifndef ARIS_DYNAMIC_PUMA_5_AXIS_H_
#define ARIS_DYNAMIC_PUMA_5_AXIS_H_

#include <array>
#include <aris/dynamic/model_solver.hpp>
#include <aris/dynamic/puma.hpp>

namespace aris::dynamic
{
	auto ARIS_API createModelPuma5(const PumaParam &param)->std::unique_ptr<aris::dynamic::Model>;

	class ARIS_API Puma5InverseKinematicSolver :public aris::dynamic::InverseKinematicSolver
	{
	public:
		auto virtual allocateMemory()->void override;
		auto virtual kinPos()->int override;
		auto virtual setPmEE(const double *ee_pm, const double *ext_axes)->void
		{
			model()->generalMotionPool()[0].setMpm(ee_pm);
			if (ext_axes)
			{
				for (int i = 6; i < model()->motionPool().size(); ++i)
				{
					model()->motionPool()[i].setMp(ext_axes[i - 6]);
				}
			}
		}
		auto setWhichRoot(int root_of_0_to_7)->void;
		auto whichRoot()const->int;

		virtual ~Puma5InverseKinematicSolver();
		explicit Puma5InverseKinematicSolver();
		ARIS_DECLARE_BIG_FOUR(Puma5InverseKinematicSolver);

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};
}

#endif
