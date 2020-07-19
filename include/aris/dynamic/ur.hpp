#ifndef ARIS_DYNAMIC_UR_H_
#define ARIS_DYNAMIC_UR_H_

#include <aris/dynamic/model_solver.hpp>

namespace aris::dynamic
{
	/// @defgroup dynamic_model_group 动力学建模模块
	/// @{
	///
	class ARIS_API Ur5InverseKinematicSolver :public aris::dynamic::InverseKinematicSolver
	{
	public:
		auto virtual kinPos()->int override;
		auto virtual setPmEE(const double *ee_pm, const double *ext_axes)->void override
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
		auto whichRoot()->int;

		virtual ~Ur5InverseKinematicSolver() = default;
		explicit Ur5InverseKinematicSolver(const std::string &name = "ur5_inverse_solver", aris::Size max_iter_count = 100, double max_error = 1e-10) :InverseKinematicSolver(name, max_iter_count, max_error) {}
		ARIS_DEFINE_BIG_FOUR(Ur5InverseKinematicSolver);

	private:
		int which_root_{ 0 };
	};
	///
	/// @}
}

#endif
