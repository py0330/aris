#ifndef ARIS_DYNAMIC_MODEL_SOLVER_
#define ARIS_DYNAMIC_MODEL_SOLVER_

#include <vector>
#include <array>
#include <map>
#include <string>
#include <memory>
#include <functional>
#include <algorithm>

#include <aris_dynamic_model_basic.h>
#include <aris_dynamic_model_coordinate.h>
#include <aris_dynamic_model_interaction.h>

namespace aris::dynamic
{
	/// @defgroup dynamic_model_group 动力学建模模块
	/// @{
	///
	class Solver :public Element
	{
	public:
		static auto Type()->const std::string & { static const std::string type{ "Solver" }; return type; }
		auto virtual type() const->const std::string& override { return Type(); }
		auto virtual saveXml(aris::core::XmlElement &xml_ele) const->void override;
		auto virtual loadXml(const aris::core::XmlElement &xml_ele)->void override;
		auto virtual allocateMemory()->void = 0;
		auto virtual kinPos()->bool = 0;
		auto virtual kinVel()->void = 0;
		auto virtual dynAccAndFce()->void = 0;
		auto error()const->double;
		auto setError(double error)->void;
		auto maxError()const->double;
		auto setMaxError(double max_error)->void;
		auto iterCount()const->Size;
		auto setIterCount(Size iter_count)->void;
		auto maxIterCount()const->Size;
		auto setMaxIterCount(Size max_count)->void;

		virtual ~Solver();
		explicit Solver(const std::string &name = "solver", Size max_iter_count = 100, double max_error = 1e-10);
		Solver(const Solver&);
		Solver(Solver&&);
		Solver& operator=(const Solver&);
		Solver& operator=(Solver&&);

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};
	class UniversalSolver : public Solver
	{
	public:
		static const std::string& Type() { static const std::string type("UniversalSolver"); return type; }
		auto virtual type() const->const std::string& override { return Type(); }
		auto virtual allocateMemory()->void override;
		auto virtual kinPos()->bool override;
		auto virtual kinVel()->void override;
		auto virtual dynAccAndFce()->void override;
		auto cptGeneralJacobi()->void;
		auto mJ()->Size;// = part_number x 6
		auto nJ()->Size;// = motion_number + general_motion_number x 6
		auto Jg()->double *;// dimension : mJ x nJ 
		auto cg()->double *;// dimension : mJ x 1
		auto cptGeneralInverseDynamicMatrix()->void;// torque = M * theta_dot_dot + h
		auto nM()->Size;// = motion_number + general_motion_number x 6
		auto M()->double *;// dimension : nM x nM 
		auto h()->double *;// dimension : nM x 1
		auto plotRelation()->void;

		virtual ~UniversalSolver();
		explicit UniversalSolver(const std::string &name = "diag_solver", Size max_iter_count = 100, double max_error = 1e-10);
		UniversalSolver(const UniversalSolver &other);
		UniversalSolver(UniversalSolver &&other);
		UniversalSolver& operator=(const UniversalSolver &other);
		UniversalSolver& operator=(UniversalSolver &&other);

	public:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};
	class ForwardKinematicSolver :public UniversalSolver
	{
	public:
		static const std::string& Type() { static const std::string type("ForwardKinematicSolver"); return type; }
		auto virtual type() const->const std::string& override { return Type(); }

		auto virtual allocateMemory()->void override;
		auto virtual kinPos()->bool override;
		auto virtual kinVel()->void override;
		auto virtual dynAccAndFce()->void override;

		virtual ~ForwardKinematicSolver();
		explicit ForwardKinematicSolver(const std::string &name = "forward_kinematic_solver", Size max_iter_count = 100, double max_error = 1e-10);
		ForwardKinematicSolver(const ForwardKinematicSolver &other);
		ForwardKinematicSolver(ForwardKinematicSolver &&other);
		ForwardKinematicSolver& operator=(const ForwardKinematicSolver &other);
		ForwardKinematicSolver& operator=(ForwardKinematicSolver &&other);
	};
	class InverseKinematicSolver :public UniversalSolver
	{
	public:
		static const std::string& Type() { static const std::string type("InverseKinematicSolver"); return type; }
		auto virtual type() const->const std::string& override { return Type(); }

		auto virtual allocateMemory()->void override;
		auto virtual kinPos()->bool override;
		auto virtual kinVel()->void override;
		auto virtual dynAccAndFce()->void override;

		virtual ~InverseKinematicSolver();
		explicit InverseKinematicSolver(const std::string &name = "inverse_kinematic_solver", Size max_iter_count = 100, double max_error = 1e-10);
		InverseKinematicSolver(const InverseKinematicSolver &other);
		InverseKinematicSolver(InverseKinematicSolver &&other);
		InverseKinematicSolver& operator=(const InverseKinematicSolver &other);
		InverseKinematicSolver& operator=(InverseKinematicSolver &&other);
	};
	class ForwardDynamicSolver :public UniversalSolver
	{
	public:
		static const std::string& Type() { static const std::string type("ForwardDynamicSolver"); return type; }
		auto virtual type() const->const std::string& override { return Type(); }

		auto virtual allocateMemory()->void override;
		auto virtual kinPos()->bool override;
		auto virtual kinVel()->void override;
		auto virtual dynAccAndFce()->void override;

		virtual ~ForwardDynamicSolver();
		explicit ForwardDynamicSolver(const std::string &name = "forward_dynamic_solver", Size max_iter_count = 100, double max_error = 1e-10);
		ForwardDynamicSolver(const ForwardDynamicSolver &other);
		ForwardDynamicSolver(ForwardDynamicSolver &&other);
		ForwardDynamicSolver& operator=(const ForwardDynamicSolver &other);
		ForwardDynamicSolver& operator=(ForwardDynamicSolver &&other);
	};
	class InverseDynamicSolver :public UniversalSolver
	{
	public:
		static const std::string& Type() { static const std::string type("InverseDynamicSolver"); return type; }
		auto virtual type() const->const std::string& override { return Type(); }

		auto virtual allocateMemory()->void override;
		auto virtual kinPos()->bool override;
		auto virtual kinVel()->void override;
		auto virtual dynAccAndFce()->void override;

		virtual ~InverseDynamicSolver();
		explicit InverseDynamicSolver(const std::string &name = "inverse_dynamic_solver", Size max_iter_count = 100, double max_error = 1e-10);
		InverseDynamicSolver(const InverseDynamicSolver &other);
		InverseDynamicSolver(InverseDynamicSolver &&other);
		InverseDynamicSolver& operator=(const InverseDynamicSolver &other);
		InverseDynamicSolver& operator=(InverseDynamicSolver &&other);
	};

	class Ur5InverseKinematicSolver :public aris::dynamic::InverseKinematicSolver
	{
	public:
		static const std::string& Type() { static const std::string type("Ur5InverseSolver"); return type; }
		auto virtual type() const->const std::string& override { return Type(); }
		auto virtual kinPos()->bool override;
		auto setWhichRoot(int root_of_0_to_7);

		virtual ~Ur5InverseKinematicSolver() = default;
		explicit Ur5InverseKinematicSolver(const std::string &name = "ur5_inverse_solver", aris::Size max_iter_count = 100, double max_error = 1e-10) :InverseKinematicSolver(name, max_iter_count, max_error) {}

	private:
		int which_root_{ 0 };
	};
	///
	/// @}
}

#endif
