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

	class ARIS_API SimResult : public Element{
	public:
		class TimeResult : public Element{
		public:
			auto record()->void;
			auto restore(Size pos)->void;

			virtual ~TimeResult();
			explicit TimeResult(const std::string &name = "time_result");
			ARIS_DECLARE_BIG_FOUR_NOEXCEPT(TimeResult);

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
			ARIS_DECLARE_BIG_FOUR_NOEXCEPT(PartResult);

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
			ARIS_DECLARE_BIG_FOUR_NOEXCEPT(ConstraintResult);

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
		ARIS_DECLARE_BIG_FOUR_NOEXCEPT(SimResult);

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};
	class ARIS_API Simulator :public Element{
	public:
		auto virtual simulate(aris::plan::Plan &plan, SimResult &result)->void;

		virtual ~Simulator();
		explicit Simulator(const std::string &name = "simulator");
		ARIS_DECLARE_BIG_FOUR_NOEXCEPT(Simulator);

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};
	class ARIS_API SolverSimulator : public Simulator{
	public:
		auto virtual simulate(aris::plan::Plan &plan, SimResult &result)->void override;
		using Simulator::simulate;
		auto solver()->Solver&;
		auto solver()const ->const Solver& { return const_cast<SolverSimulator*>(this)->solver(); };

		virtual ~SolverSimulator();
		explicit SolverSimulator(const std::string &name = "solver_simulator", Solver *solver = nullptr);
		ARIS_DECLARE_BIG_FOUR_NOEXCEPT(SolverSimulator);

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};
	class ARIS_API AdamsSimulator :public Simulator{
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
		ARIS_DECLARE_BIG_FOUR_NOEXCEPT(AdamsSimulator);

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
	class ARIS_API Calibrator :public Element{
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
		
		// 设置位置、速度、力矩的index，以及每个电机共记录多少个数据
		// pos_idx            : 第一个电机位置的index
		// vel_idx            : 第一个电机速度的index
		// fce_idx            : 第一个电机力矩的index
		// data_num_per_motor : 每一个电机共记录多少个数据
		// 例如，所需标定的数据如下，共有3个电机：
		//
		// ***   1#速度 1#位置 1#电流 1#力矩   2#速度 2#位置 2#电流 2#力矩   3#速度 3#位置 3#电流 3#力矩  ***
		//
		// 那么需要设置
		// setDataIndex(1, 0, 2, 4);
		auto setDataIndex(int pos_idx, int vel_idx, int fce_idx, int data_num_per_motor)->void;
		auto dataIndex()const->std::tuple<int, int, int, int>;
		
		// 滤波窗口
		auto setFilterWindowSize(int window_size)->void;
		auto filterWindowSize()const->int;

		// 速度系数：真实速度 = 标定表格中速度 * 系数
		auto velocityRatio()const->std::vector<double>;
		auto setVelocityRatio(std::vector<double> constant)->void;

		// 力矩系数：真实力矩 = 标定表格中力矩 * 系数
		auto torqueConstant()const->std::vector<double>;
		auto setTorqueConstant(std::vector<double> constant)->void;

		// 力矩权重：每个电机对误差的承受能力
		//           力矩越大的电机，应该承受能力越强
		// !! 因此 ：力矩权重一般设置成: 电机额定扭矩 / 系统中最大的电机额定扭矩！！
		//           默认为 1
		auto torqueWeight()const->std::vector<double>;
		auto setTorqueWeight(std::vector<double> constant)->void;

		// 速度死区：电机在死区内，摩擦力模型不适用，此时忽略该电机的力矩（不建模型）
		//           默认为0.01
		auto velocityDeadZone()const->std::vector<double>;
		auto setVelocityDeadZone(std::vector<double> constant)->void;

		// 允许的最大方差：若经过计算，结果的方差小于该值，则可以接受，否则返回错误
		auto tolerableVariance()const->double;
		auto setTolerableVariance(double variance)->void;

		// tbd //
		auto clbFile(const std::string &file_path)->void;
		auto clbFiles(const std::vector<std::string> &file_paths)->int;
		
		auto updateInertiaParam(const double* inertia_param)->void;


		// verifies ... //
		auto setVerifyOutputFileDir(std::string file_path)->void;
		auto verifyFiles(const std::vector<std::string> &file_paths)->void;
		

		virtual ~Calibrator();
		explicit Calibrator(const std::string &name = "calibrator");
		ARIS_DECLARE_BIG_FOUR_NOEXCEPT(Calibrator);

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};

	/// @}
}

#endif
