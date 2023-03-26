#ifndef ARIS_DYNAMIC_MODEL_SOLVER_H_
#define ARIS_DYNAMIC_MODEL_SOLVER_H_

#include <aris/dynamic/model_basic.hpp>

namespace aris::dynamic
{
	/// @defgroup dynamic_model_group 动力学建模模块
	/// @{
	///
	class ARIS_API Solver :public Element{
	public:
		auto virtual allocateMemory()->void = 0;
		auto virtual kinPos()->int = 0;
		auto virtual kinVel()->int = 0;
		auto virtual dynAccAndFce()->int = 0;

		auto virtual kinPosPure(const double* motion_pos, double* answer, int which_root)->int { return -1; };

		// 运动学的解个数 //
		auto setRootNumber(int root_of_solver)->void;
		auto rootNumber()const->int;
		// 选择哪个解 //
		auto setWhichRoot(int root_of_solver)->void;
		auto whichRoot()const->int;
		auto error()const->double;
		auto setError(double error)->void;
		auto maxError()const->double;
		auto setMaxError(double max_error)->void;
		auto iterCount()const->Size;
		auto setIterCount(Size iter_count)->void;
		auto maxIterCount()const->Size;
		auto setMaxIterCount(Size max_count)->void;

		virtual ~Solver();
		explicit Solver(Size max_iter_count = 100, double max_error = 1e-10);
		ARIS_DECLARE_BIG_FOUR(Solver);

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};
	class ARIS_API UniversalSolver : public Solver{
	public:
		auto virtual allocateMemory()->void override;
		auto virtual kinPos()->int override;
		auto virtual kinVel()->int override;
		auto virtual dynAccAndFce()->int override;
		
		auto virtual kinPosPure(const double* motion_pos, double* answer, int which_root)->int override;

		auto kinPosCompute()->int;                               // 进行计算
		auto kinPosSetActiveMotionPos(const double* mp)->void;   // 将用户的作为 mp
		auto kinPosGetUnactiveMotionPos(double* mp)->void; // 获取未激活的motion 的 mp
		auto kinPosSetMotionPosFromModel()->void;                // 将Model的状态作为 mp
		auto kinPosUpdateModel()->void;                          // 计算之后，设置 model 的状态
		auto cptGeneralJacobi() noexcept->void;// all_part_vs = Jg * theta_dot, all_part_as = Jg * theta_dot_dot + cg
		auto mJg()const noexcept->Size;// = part_number x 6
		auto nJg()const noexcept->Size;// = motion_number + general_motion_number x 6
		auto Jg()const noexcept->const double *;// dimension : mJ x nJ 
		auto cg()const noexcept->const double *;// dimension : mJ x 1
		auto cptGeneralInverseDynamicMatrix() noexcept->void;// torque = M * theta_dot_dot + h
		auto nM()const noexcept->Size;// = sum of all active motion dimension
		auto M()const noexcept->const double *;// dimension : nM x nM 
		auto h()const noexcept->const double *;// dimension : nM x 1

		virtual ~UniversalSolver();
		explicit UniversalSolver(Size max_iter_count = 100, double max_error = 1e-10);
		ARIS_DECLARE_BIG_FOUR(UniversalSolver);

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
		friend class ForwardKinematicSolver;
		friend class InverseKinematicSolver;
	};
	class ARIS_API ForwardKinematicSolver :public UniversalSolver{
	public:
		auto virtual allocateMemory()->void override;
		auto virtual kinPos()->int override;
		auto virtual kinVel()->int override;
		auto virtual dynAccAndFce()->int override;

		auto cptJacobi()noexcept->void;
		auto mJf()const noexcept->Size;// equal mot num
		auto nJf()const noexcept->Size;// equal ee num * 6
		auto Jf()const noexcept->const double *;// inverse jacobi   ee_vs = Jf * ee_vs + mot_vs
		auto cf()const noexcept->const double *;// dimension : mJ x 1

		virtual ~ForwardKinematicSolver();
		explicit ForwardKinematicSolver(Size max_iter_count = 100, double max_error = 1e-10);
		ARIS_DECLARE_BIG_FOUR(ForwardKinematicSolver);

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};
	class ARIS_API InverseKinematicSolver :public UniversalSolver{
	public:
		auto virtual allocateMemory()->void override;
		auto virtual kinPos()->int override;
		auto virtual kinVel()->int override;
		auto virtual dynAccAndFce()->int override;

		auto cptJacobi()noexcept->void;
		auto mJi()const noexcept->Size;// equal mot num
		auto nJi()const noexcept->Size;// equal ee num * 6
		auto Ji()const noexcept->const double *;// inverse jacobi   mot_vs = Ji * ee_vs
		auto ci()const noexcept->const double *;// dimension : mJ x 1

		virtual ~InverseKinematicSolver();
		explicit InverseKinematicSolver(Size max_iter_count = 100, double max_error = 1e-10);
		ARIS_DECLARE_BIG_FOUR(InverseKinematicSolver);

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};
	class ARIS_API ForwardDynamicSolver :public UniversalSolver{
	public:
		auto virtual allocateMemory()->void override;
		auto virtual kinPos()->int override;
		auto virtual kinVel()->int override;
		auto virtual dynAccAndFce()->int override;

		virtual ~ForwardDynamicSolver();
		explicit ForwardDynamicSolver(Size max_iter_count = 100, double max_error = 1e-10);
		ARIS_DECLARE_BIG_FOUR(ForwardDynamicSolver);
	};
	class ARIS_API InverseDynamicSolver :public UniversalSolver{
	public:
		auto virtual allocateMemory()->void override;
		auto virtual kinPos()->int override;
		auto virtual kinVel()->int override;
		auto virtual dynAccAndFce()->int override;

		virtual ~InverseDynamicSolver();
		explicit InverseDynamicSolver(Size max_iter_count = 100, double max_error = 1e-10);
		ARIS_DECLARE_BIG_FOUR(InverseDynamicSolver);
	};
	///
	/// @}
}

#endif
