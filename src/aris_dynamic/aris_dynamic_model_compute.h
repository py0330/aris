#ifndef ARIS_DYNAMIC_MODEL_COMPUTE_
#define ARIS_DYNAMIC_MODEL_COMPUTE_

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

namespace aris
{
	namespace dynamic
	{
		/// @defgroup dynamic_model_group 动力学建模模块
		/// @{
		///
		
		struct PlanParam
		{
			Model* model_;
			std::uint32_t count_;
			void *param_;
			std::uint32_t param_size_;
		};
		using PlanFunction = std::function<int(const PlanParam &)>;
		using ParseFunction = std::function<void(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::MsgBase &msg_out)>;

		/// @{
		class Solver :public Element
		{
		public:
			static auto Type()->const std::string &{ static const std::string type{ "Solver" }; return type; }
			auto virtual type() const->const std::string& override{ return Type(); }
			auto virtual saveXml(aris::core::XmlElement &xml_ele) const->void override;
			auto virtual loadXml(const aris::core::XmlElement &xml_ele)->void override;
			auto virtual allocateMemory()->void = 0;
			auto virtual kinPos()->bool = 0;
			auto virtual kinVel()->void = 0;
			auto virtual dynAccAndFce()->void = 0;
			auto init()->void;
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
			auto virtual type() const->const std::string& override{ return Type(); }
			auto virtual allocateMemory()->void override;
			auto virtual kinPos()->bool override;
			auto virtual kinVel()->void override;
			auto virtual dynAccAndFce()->void override;
			auto cptGeneralJacobi()->void; 
			auto Jg()->double *;
			auto cg()->double *;
			auto cptGeneralInverseDynamicMatrix()->void;// torque = M * theta_dot_dot + h
			auto M()->double *;
			auto h()->double *;
			auto plotRelation()->void;

			virtual ~UniversalSolver();
			explicit UniversalSolver(const std::string &name = "diag_solver", Size max_iter_count = 100, double max_error = 1e-10);
			UniversalSolver(const UniversalSolver &other);
			UniversalSolver(UniversalSolver &&other);
			UniversalSolver& operator=(const UniversalSolver &other);
			UniversalSolver& operator=(UniversalSolver &&other);

		private:
			struct Imp;
			aris::core::ImpPtr<Imp> imp_;
		};
		class ForwardKinematicSolver :public UniversalSolver
		{
		public:
			static const std::string& Type() { static const std::string type("ForwardKinematicSolver"); return type; }
			auto virtual type() const->const std::string& override{ return Type(); }
			
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
			auto virtual type() const->const std::string& override{ return Type(); }

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
			auto virtual type() const->const std::string& override{ return Type(); }

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
			auto virtual type() const->const std::string& override{ return Type(); }

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
		/// @}

		class SimResult : public Element
		{
		public:
			class TimeResult : public Element
			{
			public:
				static auto Type()->const std::string &{ static const std::string type{ "TimeResult" }; return type; }
				auto virtual type() const->const std::string& override{ return Type(); }
				auto virtual saveXml(aris::core::XmlElement &xml_ele) const->void override;
				auto virtual loadXml(const aris::core::XmlElement &xml_ele)->void override;
				auto record()->void;
				auto restore(Size pos)->void;

				virtual ~TimeResult();
				explicit TimeResult(const std::string &name = "time_result");
				TimeResult(const TimeResult&);
				TimeResult(TimeResult&&);
				TimeResult& operator=(const TimeResult&);
				TimeResult& operator=(TimeResult&&);

			private:
				struct Imp;
				aris::core::ImpPtr<Imp> imp_;

				friend class SimResult;
			};
			class PartResult : public Element
			{
			public:
				static auto Type()->const std::string &{ static const std::string type{ "PartResult" }; return type; }
				auto virtual type() const->const std::string& override{ return Type(); }
				auto virtual saveXml(aris::core::XmlElement &xml_ele) const->void override;
				auto virtual loadXml(const aris::core::XmlElement &xml_ele)->void override;
				auto part()->Part&;
				auto part()const->const Part&{ return const_cast<PartResult*>(this)->part(); }
				auto record()->void;
				auto restore(Size pos)->void;

				virtual ~PartResult();
				explicit PartResult(const std::string &name = "part_result", Part *part = nullptr);
				PartResult(const PartResult&);
				PartResult(PartResult&&);
				PartResult& operator=(const PartResult&);
				PartResult& operator=(PartResult&&);

			private:
				struct Imp;
				aris::core::ImpPtr<Imp> imp_;

				friend class SimResult;
			};
			class ConstraintResult : public Element
			{
			public:
				static auto Type()->const std::string &{ static const std::string type{ "ConstraintResult" }; return type; }
				auto virtual type() const->const std::string& override{ return Type(); }
				auto virtual saveXml(aris::core::XmlElement &xml_ele) const->void override;
				auto virtual loadXml(const aris::core::XmlElement &xml_ele)->void override;
				auto constraint()->Constraint&;
				auto constraint()const->const Constraint&{ return const_cast<ConstraintResult*>(this)->constraint(); }
				auto record()->void;
				auto restore(Size pos)->void;

				virtual ~ConstraintResult();
				explicit ConstraintResult(const std::string &name = "constraint_result", Constraint *constraint = nullptr);
				ConstraintResult(const ConstraintResult&);
				ConstraintResult(ConstraintResult&&);
				ConstraintResult& operator=(const ConstraintResult&);
				ConstraintResult& operator=(ConstraintResult&&);

			private:
				struct Imp;
				aris::core::ImpPtr<Imp> imp_;

				friend class SimResult;
			};

			static auto Type()->const std::string &{ static const std::string type{ "SimResult" }; return type; }
			auto virtual type() const->const std::string& override{ return Type(); }
			auto virtual loadXml(const aris::core::XmlElement &xml_ele)->void override;
			auto timeResult()->TimeResult&;
			auto timeResult()const->const TimeResult&{ return const_cast<SimResult*>(this)->timeResult(); }
			auto partResultPool()->aris::core::ObjectPool<PartResult, Element>&;
			auto partResultPool()const->const aris::core::ObjectPool<PartResult, Element>&{return const_cast<SimResult*>(this)->partResultPool(); };
			auto constraintResultPool()->aris::core::ObjectPool<ConstraintResult, Element>&;
			auto constraintResultPool()const->const aris::core::ObjectPool<ConstraintResult, Element>&{return const_cast<SimResult*>(this)->constraintResultPool(); };

			auto allocateMemory()->void;
			auto record()->void;
			auto restore(Size pos)->void;
			auto size()const->Size;
			auto clear()->void;

			virtual ~SimResult();
			explicit SimResult(const std::string &name = "sim_result");
			SimResult(const SimResult&);
			SimResult(SimResult&&);
			SimResult& operator=(const SimResult&);
			SimResult& operator=(SimResult&&);

		private:
			struct Imp;
			aris::core::ImpPtr<Imp> imp_;
		};
		class Simulator :public Element
		{
		public:
			static auto Type()->const std::string &{ static const std::string type{ "Simulator" }; return type; }
			auto virtual type() const->const std::string& override{ return Type(); }

			auto virtual simulate(const PlanFunction &plan, void *param, std::uint32_t param_size, SimResult &result)->void;
			template<typename ParamType>
			auto simulate(const PlanFunction &plan, ParamType type, SimResult &result)->void
			{
				static_assert(std::is_trivial<ParamType>::value, "ParamType must be trivial type");
				simulate(plan, &type, std::int32_t(sizeof(type)), result);
			}

			virtual ~Simulator();
			explicit Simulator(const std::string &name = "simulator");
			Simulator(const Simulator&);
			Simulator(Simulator&&);
			Simulator& operator=(const Simulator&);
			Simulator& operator=(Simulator&&);

		private:
			struct Imp;
			aris::core::ImpPtr<Imp> imp_;
		};
		class SolverSimulator : public Simulator
		{
		public:
			static auto Type()->const std::string &{ static const std::string type{ "SolverSimulator" }; return type; }
			auto virtual type() const->const std::string& override{ return Type(); }
			auto virtual saveXml(aris::core::XmlElement &xml_ele) const->void override;
			auto virtual loadXml(const aris::core::XmlElement &xml_ele)->void override;
			auto virtual simulate(const PlanFunction &plan, void *param, std::uint32_t param_size, SimResult &result)->void override;
			using Simulator::simulate;
			auto solver()->Solver&;
			auto solver()const ->const Solver&{ return const_cast<SolverSimulator*>(this)->solver(); };

			virtual ~SolverSimulator();
			explicit SolverSimulator(const std::string &name = "solver_simulator", Solver *solver = nullptr);
			SolverSimulator(const SolverSimulator&);
			SolverSimulator(SolverSimulator&&);
			SolverSimulator& operator=(const SolverSimulator&);
			SolverSimulator& operator=(SolverSimulator&&);

		private:
			struct Imp;
			aris::core::ImpPtr<Imp> imp_;
		};
		class AdamsSimulator :public Simulator
		{
		public:
			static auto Type()->const std::string &{ static const std::string type{ "AdamsSimulator" }; return type; }
			auto virtual type() const->const std::string& override{ return Type(); }
			auto saveAdams(const std::string &filename, SimResult &result, Size pos = -1)->void;
			auto saveAdams(std::ofstream &file, SimResult &result, Size pos = -1)->void;
			auto saveAdams(const std::string &filename)->void;
			auto saveAdams(std::ofstream &file)->void;
			auto adamsID(const Marker &mak)const->Size;
			auto adamsID(const Part &prt)const->Size;
			auto adamsID(const Element &ele)const->Size { return ele.id() + 1; };

			virtual ~AdamsSimulator();
			explicit AdamsSimulator(const std::string &name = "adams_solver");
			AdamsSimulator(const AdamsSimulator&);
			AdamsSimulator(AdamsSimulator&&);
			AdamsSimulator& operator=(const AdamsSimulator&);
			AdamsSimulator& operator=(AdamsSimulator&&);

		private:
			struct Imp;
			aris::core::ImpPtr<Imp> imp_;
		};

		class Calibrator :public Element
		{
		public:
			static auto Type()->const std::string &{ static const std::string type{ "Calibrator" }; return type; }
			auto virtual type() const->const std::string& override{ return Type(); }
			auto virtual allocateMemory()->void;
			auto m()->Size;
			auto n()->Size { return g() + k(); }
			auto g()->Size;
			auto k()->Size;
			auto A()->double*;
			auto x()->double*;
			auto b()->double*;
			auto clb()->void;

			virtual ~Calibrator();
			explicit Calibrator(const std::string &name = "calibrator");
			Calibrator(const Calibrator&);
			Calibrator(Calibrator&&);
			Calibrator& operator=(const Calibrator&);
			Calibrator& operator=(Calibrator&&);

		private:
			struct Imp;
			aris::core::ImpPtr<Imp> imp_;
		};

		/// @}
	}
}

#endif
