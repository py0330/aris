#ifndef ARIS_DYNAMIC_MODEL_SIMULATION_H_
#define ARIS_DYNAMIC_MODEL_SIMULATION_H_

#include <aris/dynamic/model_interaction.hpp>
#include <aris/dynamic/model_solver.hpp>


namespace aris::plan
{
	struct PlanTarget;
	class Plan;
}

namespace aris::dynamic
{
	/// @defgroup dynamic_model_group 动力学建模模块
	/// @{
	///

	class ARIS_API SimResult : public Element
	{
	public:
		class TimeResult : public Element
		{
		public:
			auto record()->void;
			auto restore(Size pos)->void;

			virtual ~TimeResult();
			explicit TimeResult(const std::string &name = "time_result");
			ARIS_DECLARE_BIG_FOUR(TimeResult);

		private:
			struct Imp;
			aris::core::ImpPtr<Imp> imp_;

			friend class SimResult;
		};
		class PartResult : public Element
		{
		public:
			auto part()->Part&;
			auto part()const->const Part& { return const_cast<PartResult*>(this)->part(); }
			auto record()->void;
			auto restore(Size pos)->void;

			virtual ~PartResult();
			explicit PartResult(const std::string &name = "part_result", Part *part = nullptr);
			ARIS_DECLARE_BIG_FOUR(PartResult);

		private:
			struct Imp;
			aris::core::ImpPtr<Imp> imp_;

			friend class SimResult;
		};
		class ConstraintResult : public Element
		{
		public:
			auto constraint()->Constraint&;
			auto constraint()const->const Constraint& { return const_cast<ConstraintResult*>(this)->constraint(); }
			auto record()->void;
			auto restore(Size pos)->void;

			virtual ~ConstraintResult();
			explicit ConstraintResult(const std::string &name = "constraint_result", Constraint *constraint = nullptr);
			ARIS_DECLARE_BIG_FOUR(ConstraintResult);

		private:
			struct Imp;
			aris::core::ImpPtr<Imp> imp_;

			friend class SimResult;
		};

		auto timeResult()->TimeResult&;
		auto timeResult()const->const TimeResult& { return const_cast<SimResult*>(this)->timeResult(); }
		auto partResultPool()->std::vector<PartResult>&;
		auto partResultPool()const->const std::vector<PartResult>& { return const_cast<SimResult*>(this)->partResultPool(); };
		auto constraintResultPool()->std::vector<ConstraintResult>&;
		auto constraintResultPool()const->const std::vector<ConstraintResult>& { return const_cast<SimResult*>(this)->constraintResultPool(); };

		auto allocateMemory()->void;
		auto record()->void;
		auto restore(Size pos)->void;
		auto size()const->Size;
		auto clear()->void;

		virtual ~SimResult();
		explicit SimResult(const std::string &name = "sim_result");
		ARIS_DECLARE_BIG_FOUR(SimResult);

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};
	class ARIS_API Simulator :public Element
	{
	public:
		auto virtual simulate(aris::plan::Plan &plan, SimResult &result)->void;

		virtual ~Simulator();
		explicit Simulator(const std::string &name = "simulator");
		ARIS_DECLARE_BIG_FOUR(Simulator);

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};
	class ARIS_API SolverSimulator : public Simulator
	{
	public:
		auto virtual simulate(aris::plan::Plan &plan, SimResult &result)->void override;
		using Simulator::simulate;
		auto solver()->Solver&;
		auto solver()const ->const Solver& { return const_cast<SolverSimulator*>(this)->solver(); };

		virtual ~SolverSimulator();
		explicit SolverSimulator(const std::string &name = "solver_simulator", Solver *solver = nullptr);
		ARIS_DECLARE_BIG_FOUR(SolverSimulator);

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};
	class ARIS_API AdamsSimulator :public Simulator
	{
	public:
		auto saveAdams(const std::string &filename, SimResult &result, Size pos = -1)->void;
		auto saveAdams(std::ofstream &file, SimResult &result, Size pos = -1)->void;
		auto saveAdams(const std::string &filename)->void;
		auto saveAdams(std::ofstream &file)->void;
		auto adamsID(const Marker &mak)const->Size;
		auto adamsID(const Part &prt)const->Size;
		auto adamsID(const Element &ele)const->Size { return ele.id() + 1; };

		virtual ~AdamsSimulator();
		explicit AdamsSimulator(const std::string &name = "adams_solver");
		ARIS_DECLARE_BIG_FOUR(AdamsSimulator);

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};

	// 辨识会得到 A 、 x 、 b 这样的矩阵和向量
	// 理论上 A * x = b
	// A为 m * n 维，x 为 n * 1 维，b维 m * 1维
	// 
	// m 为电机个数，也就是当前点的方程数，比如这里就是6
	// n 为待辨识的参数，它为杆件数(不含地面) * 10 + 电机 * 3，因此这里是78
	//
	// 杆件的辨识参数如下，其中xyz为质心位置：
	// m m*x m*y m*z Ixx Iyy Izz Ixy Ixz Iyz
	// 电机的辨识参数如下，也就是静摩擦力、粘性摩擦系数、电机转子惯量：
	// fs kv ki
	// 其中电机摩擦力计算为： 
	// f = sig(v)*fs + kv * v + ki * a
	//
	// x为当前的惯量值，注意它并不是辨识出来的结果，它仅仅保存了当前model中各个杆件的惯量和电机参数
	// b为当前的电机出力
	// A为观测矩阵
	class ARIS_API Calibrator :public Element
	{
	public:
		auto virtual allocateMemory()->void;
		auto m()->Size;
		auto n()->Size { return g() + k(); }
		auto g()->Size;
		auto k()->Size;
		auto A()->double*;
		auto x()->double*;
		auto b()->double*;
		auto clb()->void;
		
		// 设置位置、速度、电流/力矩的index，以及每个电机共记录多少个数据
		auto setDataIndex(int pos_idx, int vel_idx, int fce_idx, int data_num_per_motor)->void;
		auto dataIndex()const->std::tuple<int, int, int, int>;
		auto setFilterWindowSize(int window_size)->void;
		auto filterWindowSize()const->int;

		auto velocityRatio()const->std::vector<double>;
		auto setVelocityRatio(std::vector<double> constant)->void;

		auto torqueConstant()const->std::vector<double>;
		auto setTorqueConstant(std::vector<double> constant)->void;

		// tbd //
		auto clbFile(const std::string &file_path)->void;
		auto clbFiles(const std::vector<std::string> &file_paths)->void;
		auto verifyFiles(const std::vector<std::string> &file_paths)->void;
		auto updateInertiaParam(const double *inertia_param)->void;

		virtual ~Calibrator();
		explicit Calibrator(const std::string &name = "calibrator");
		ARIS_DECLARE_BIG_FOUR(Calibrator);

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};

	/// @}
}

#endif
