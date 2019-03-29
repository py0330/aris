#ifndef ARIS_DYNAMIC_MODEL_SOLVER_
#define ARIS_DYNAMIC_MODEL_SOLVER_

#include <vector>
#include <array>
#include <map>
#include <string>
#include <memory>
#include <functional>
#include <algorithm>

#include <aris/dynamic/model_basic.hpp>
#include <aris/dynamic/model_coordinate.hpp>
#include <aris/dynamic/model_interaction.hpp>

namespace aris::dynamic
{
	/// @defgroup dynamic_model_group 动力学建模模块
	/// @{
	///
	class Solver :public Element
	{
	public:
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
		ARIS_REGISTER_TYPE("Solver");
		ARIS_DECLARE_BIG_FOUR(Solver);

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};
	class UniversalSolver : public Solver
	{
	public:
		auto virtual allocateMemory()->void override;
		auto virtual kinPos()->bool override;
		auto virtual kinVel()->void override;
		auto virtual dynAccAndFce()->void override;
		auto cptGeneralJacobi() noexcept->void;// all_part_vs = Jg * theta_dot, all_part_as = Jg * theta_dot_dot + cg
		auto mJg()const noexcept->Size;// = part_number x 6
		auto nJg()const noexcept->Size;// = motion_number + general_motion_number x 6
		auto Jg()const noexcept->const double *;// dimension : mJ x nJ 
		auto cg()const noexcept->const double *;// dimension : mJ x 1
		auto cptGeneralInverseDynamicMatrix() noexcept->void;// torque = M * theta_dot_dot + h
		auto nM()const noexcept->Size;// = motion_number + general_motion_number x 6
		auto M()const noexcept->const double *;// dimension : nM x nM 
		auto h()const noexcept->const double *;// dimension : nM x 1
		auto plotRelation()->void;

		virtual ~UniversalSolver();
		explicit UniversalSolver(const std::string &name = "diag_solver", Size max_iter_count = 100, double max_error = 1e-10);
		ARIS_REGISTER_TYPE("UniversalSolver");
		ARIS_DECLARE_BIG_FOUR(UniversalSolver);

	public:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};
	class ForwardKinematicSolver :public UniversalSolver
	{
	public:
		auto virtual allocateMemory()->void override;
		auto virtual kinPos()->bool override;
		auto virtual kinVel()->void override;
		auto virtual dynAccAndFce()->void override;
		auto cptJacobi()noexcept->void;
		auto mJf()const noexcept->Size;// equal mot num
		auto nJf()const noexcept->Size;// equal ee num * 6
		auto Jf()const noexcept->const double *;// inverse jacobi   mot_vs = Ji * ee_vs
		auto cf()const noexcept->const double *;// dimension : mJ x 1

		virtual ~ForwardKinematicSolver();
		explicit ForwardKinematicSolver(const std::string &name = "forward_kinematic_solver", Size max_iter_count = 100, double max_error = 1e-10);
		ARIS_REGISTER_TYPE("ForwardKinematicSolver");
		ARIS_DECLARE_BIG_FOUR(ForwardKinematicSolver);

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};
	class InverseKinematicSolver :public UniversalSolver
	{
	public:
		auto virtual allocateMemory()->void override;
		auto virtual kinPos()->bool override;
		auto virtual kinVel()->void override;
		auto virtual dynAccAndFce()->void override;
		auto cptJacobi()noexcept->void;
		auto mJi()const noexcept->Size;// equal mot num
		auto nJi()const noexcept->Size;// equal ee num * 6
		auto Ji()const noexcept->const double *;// inverse jacobi   mot_vs = Ji * ee_vs
		auto ci()const noexcept->const double *;// dimension : mJ x 1

		virtual ~InverseKinematicSolver();
		explicit InverseKinematicSolver(const std::string &name = "inverse_kinematic_solver", Size max_iter_count = 100, double max_error = 1e-10);
		ARIS_REGISTER_TYPE("InverseKinematicSolver");
		ARIS_DECLARE_BIG_FOUR(InverseKinematicSolver);

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};
	class ForwardDynamicSolver :public UniversalSolver
	{
	public:
		auto virtual allocateMemory()->void override;
		auto virtual kinPos()->bool override;
		auto virtual kinVel()->void override;
		auto virtual dynAccAndFce()->void override;

		virtual ~ForwardDynamicSolver();
		explicit ForwardDynamicSolver(const std::string &name = "forward_dynamic_solver", Size max_iter_count = 100, double max_error = 1e-10);
		ARIS_REGISTER_TYPE("ForwardDynamicSolver");
		ARIS_DECLARE_BIG_FOUR(ForwardDynamicSolver);
	};
	class InverseDynamicSolver :public UniversalSolver
	{
	public:
		auto virtual allocateMemory()->void override;
		auto virtual kinPos()->bool override;
		auto virtual kinVel()->void override;
		auto virtual dynAccAndFce()->void override;

		virtual ~InverseDynamicSolver();
		explicit InverseDynamicSolver(const std::string &name = "inverse_dynamic_solver", Size max_iter_count = 100, double max_error = 1e-10);
		ARIS_REGISTER_TYPE("InverseDynamicSolver");
		ARIS_DECLARE_BIG_FOUR(InverseDynamicSolver);
	};

	class Ur5InverseKinematicSolver :public aris::dynamic::InverseKinematicSolver
	{
	public:
		auto virtual kinPos()->bool override;
		auto setWhichRoot(int root_of_0_to_7);

		virtual ~Ur5InverseKinematicSolver() = default;
		explicit Ur5InverseKinematicSolver(const std::string &name = "ur5_inverse_solver", aris::Size max_iter_count = 100, double max_error = 1e-10) :InverseKinematicSolver(name, max_iter_count, max_error) {}
		ARIS_REGISTER_TYPE("Ur5InverseKinematicSolver");
		ARIS_DEFINE_BIG_FOUR(Ur5InverseKinematicSolver);

	private:
		int which_root_{ 0 };
	};

	class PumaInverseKinematicSolver :public aris::dynamic::InverseKinematicSolver
	{
	public:
		auto virtual allocateMemory()->void override;
		auto virtual kinPos()->bool override;
		auto setWhichRoot(int root_of_0_to_7)->void;

		virtual ~PumaInverseKinematicSolver() = default;
		explicit PumaInverseKinematicSolver(const std::string &name = "puma_inverse_solver");
		ARIS_REGISTER_TYPE("PumaInverseKinematicSolver");
		ARIS_DECLARE_BIG_FOUR(PumaInverseKinematicSolver);

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};

	class StewartInverseKinematicSolver :public aris::dynamic::InverseKinematicSolver
	{
	public:
		auto virtual allocateMemory()->void override;
		auto virtual kinPos()->bool override;

		virtual ~StewartInverseKinematicSolver() = default;
		explicit StewartInverseKinematicSolver(const std::string &name = "stewart_inverse_solver");
		ARIS_REGISTER_TYPE("StewartInverseKinematicSolver");
		ARIS_DECLARE_BIG_FOUR(StewartInverseKinematicSolver);

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};
	///
	/// @}
}

#endif
