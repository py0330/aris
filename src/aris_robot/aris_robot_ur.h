#ifndef ARIS_ROBOT_UR_H_
#define ARIS_ROBOT_UR_H_

#include <memory>

#include <aris_dynamic.h>
#include <aris_control.h>

namespace aris
{
	/// \brief 机器人命名空间
	/// \ingroup aris
	/// 
	///
	///
	namespace robot
	{
		class Ur5InverseSolver :public aris::dynamic::InverseKinematicSolver
		{
		public:
			static const std::string& Type() { static const std::string type("Ur5InverseSolver"); return type; }
			auto virtual type() const->const std::string& override{ return Type(); }
			auto virtual kinPos()->bool override;

			virtual ~Ur5InverseSolver() = default;
			explicit Ur5InverseSolver(const std::string &name = "ur5_inverse_solver", aris::Size max_iter_count = 100, double max_error = 1e-10) :InverseKinematicSolver(name, max_iter_count, max_error) {}
		};
		auto createUr5Model()->std::unique_ptr<aris::dynamic::Model>;
		auto createUr5Controller()->std::unique_ptr<aris::control::Controller>;
	}
}


#endif