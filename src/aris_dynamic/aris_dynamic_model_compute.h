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
		struct PlanParam
		{
			Model* model_;
			std::uint32_t count_;
			void *param_;
			std::uint32_t param_size_;
		};
		using PlanFunction = std::function<int(const PlanParam &)>;
		using ParseFunction = std::function<void(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::MsgBase &msg_out)>;

		class Solver :public Element
		{
		public:
			static auto Type()->const std::string &{ static const std::string type{ "Solver" }; return type; }
			auto virtual type() const->const std::string& override{ return Type(); }
			auto virtual saveXml(aris::core::XmlElement &xml_ele) const->void override;
			auto virtual loadXml(const aris::core::XmlElement &xml_ele)->void override;
			auto virtual allocateMemory()->void = 0;
			auto virtual kinPos()->void = 0;
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
		class Calibrator :public Element
		{
		public:
			static auto Type()->const std::string &{ static const std::string type{ "Calibrator" }; return type; }
			auto virtual type() const->const std::string& override{ return Type(); }
			auto virtual allocateMemory()->void = 0;

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

		class FrameSolver:public Solver
		{
		public:
			enum Frame
			{
				GROUND,
				PART,
			};

			auto setFrame(Frame frame)->void { frame_ = frame; }
			template<typename CMI_TYPE, typename CMJ_TYPE>
			auto cptCm(Constraint &cst, double *cmI, CMI_TYPE cmi_type, double *cmJ, CMJ_TYPE cmj_type)->void
			{
				switch (frame_)
				{
				case GROUND:
					return cst.cptGlbCm(cmI, cmi_type, cmJ, cmj_type);
				case PART:
					return cst.cptPrtCm(cmI, cmi_type, cmJ, cmj_type);
				}
			}
			auto cptCm(Constraint &cst, double *cmI, double *cmJ)->void
			{
				switch (frame_)
				{
				case GROUND:
					return cst.cptGlbCm(cmI, cmJ);
				case PART:
					return cst.cptPrtCm(cmI, cmJ);
				}
			}




			virtual ~FrameSolver() = default;
			explicit FrameSolver(const std::string &name = "frame_solver", Size max_iter_count = 100, double max_error = 1e-10) :Solver(name, max_iter_count, max_error) {}
			FrameSolver(const FrameSolver &other) = default;
			FrameSolver(FrameSolver &&other) = default;
			FrameSolver& operator=(const FrameSolver &other) = default;
			FrameSolver& operator=(FrameSolver &&other) = default;


		private:
			Frame frame_{ GROUND };
		};
		class CombineSolver : public FrameSolver
		{
		public:
			struct PartBlock
			{
				Part* part_;
				Size row_id_;
			};
			struct ConstraintBlock
			{
				Constraint* constraint_;
				Size col_id_;
				PartBlock *pb_i_, *pb_j_;
			};

			static const std::string& Type() { static const std::string type("CombineSolver"); return type; }
			auto virtual type() const->const std::string& override{ return Type(); }
			auto virtual allocateMemory()->void override;
			auto virtual kinPos()->void override;
			auto virtual kinVel()->void override;
			auto virtual dynAccAndFce()->void override;
			auto virtual updCm()->void;
			auto virtual updCmT()->void;
			auto virtual updIm()->void;
			auto virtual updCp()->void;
			auto virtual updCv()->void;
			auto virtual updCa()->void;
			auto virtual updPv()->void;
			auto virtual updPa()->void;
			auto virtual updPf()->void;
			auto virtual updConstraintFce()->void;
			auto virtual updPartPos()->void;
			auto virtual updPartVel()->void;
			auto virtual updPartAcc()->void;


			auto activePartBlockPool()->std::vector<PartBlock>&;
			auto activeConstraintBlockPool()->std::vector<ConstraintBlock>&;
			auto cSize()->Size;
			auto pSize()->Size;
			auto aSize()->Size { return cSize() + pSize(); };
			auto A()->double *;
			auto x()->double *;
			auto b()->double *;
			auto cm()->double * { return A() + pSize(); }
			auto im()->double * { return A(); }
			auto pp()->double * { return x(); }
			auto pv()->double * { return x(); }
			auto pa()->double * { return x(); }
			auto pf()->double * { return b(); }
			auto cp()->double * { return b() + pSize(); }
			auto cv()->double * { return b() + pSize(); }
			auto ca()->double * { return b() + pSize(); }
			auto cf()->double * { return x() + pSize(); }

			virtual ~CombineSolver();
			explicit CombineSolver(const std::string &name = "combine_solver", Size max_iter_count = 100, double max_error = 1e-10);
			CombineSolver(const CombineSolver &other);
			CombineSolver(CombineSolver &&other);
			CombineSolver& operator=(const CombineSolver &other);
			CombineSolver& operator=(CombineSolver &&other);

		private:
			struct Imp;
			aris::core::ImpPtr<Imp> imp_;
		};

		class DividedSolver : public Solver
		{
		public:
			struct PartBlock
			{
				Part* part_;
				Size row_id_, blk_row_id_;
			};
			struct ConstraintBlock
			{
				Constraint* constraint_;
				Size col_id_, blk_col_id_;
				PartBlock *pb_i_, *pb_j_;
			};
			
			static const std::string& Type() { static const std::string type("DividedSolver"); return type; }
			auto virtual type() const->const std::string& override{ return Type(); }
			auto virtual allocateMemory()->void override;

			auto activePartBlockPool()->std::vector<PartBlock>&;
			auto activeConstraintBlockPool()->std::vector<ConstraintBlock>&;
			auto cSize()->Size;
			auto pSize()->Size;
			auto im()->double *;
			auto cm()->double *;
			auto cp()->double *;
			auto cv()->double *;
			auto ca()->double *;
			auto cf()->double *;
			auto pp()->double *;
			auto pv()->double *;
			auto pa()->double *;
			auto pf()->double *;

			auto cBlkSize()->BlockSize&;
			auto pBlkSize()->BlockSize&;
			auto cpBlk()->BlockData&;
			auto cvBlk()->BlockData&;
			auto caBlk()->BlockData&;
			auto cfBlk()->BlockData&;
			auto imBlk()->BlockData&;
			auto cmBlk()->BlockData&;
			auto ppBlk()->BlockData&;
			auto pvBlk()->BlockData&;
			auto paBlk()->BlockData&;
			auto pfBlk()->BlockData&;

			virtual ~DividedSolver();
			explicit DividedSolver(const std::string &name = "devided_solver", Size max_iter_count = 100, double max_error = 1e-10);
			DividedSolver(const DividedSolver &other);
			DividedSolver(DividedSolver &&other);
			DividedSolver& operator=(const DividedSolver &other);
			DividedSolver& operator=(DividedSolver &&other);

		private:
			struct Imp;
			aris::core::ImpPtr<Imp> imp_;
		};
		class GroundDividedSolver:public DividedSolver
		{
		public:
			static const std::string& Type() { static const std::string type("GroundDividedSolver"); return type; }
			auto virtual type() const->const std::string& override{ return Type(); }
			auto updIm()->void;
			auto updCm()->void;
			auto updCp()->void;
			auto updCv()->void;
			auto updCa()->void;
			auto updPv()->void;
			auto updPa()->void;
			auto updPf()->void;
			auto updConstraintFce()->void;
			auto updPartPos()->void;
			auto updPartVel()->void;
			auto updPartAcc()->void;

			virtual ~GroundDividedSolver();
			explicit GroundDividedSolver(const std::string &name = "ground_devided_solver", Size max_iter_count = 100, double max_error = 1e-10);
			GroundDividedSolver(const GroundDividedSolver &other);
			GroundDividedSolver(GroundDividedSolver &&other);
			GroundDividedSolver& operator=(const GroundDividedSolver &other);
			GroundDividedSolver& operator=(GroundDividedSolver &&other);

		private:
			struct Imp;
			aris::core::ImpPtr<Imp> imp_;
		};
		class PartDividedSolver :public DividedSolver
		{
		public:
			static const std::string& Type() { static const std::string type("PartDividedSolver"); return type; }
			auto virtual type() const->const std::string& override{ return Type(); }
			auto updIm()->void;
			auto updCm()->void;
			auto updCp()->void;
			auto updCv()->void;
			auto updCa()->void;
			auto updPv()->void;
			auto updPa()->void;
			auto updPf()->void;
			auto updConstraintFce()->void;
			auto updPartPos()->void;
			auto updPartVel()->void;
			auto updPartAcc()->void;

			virtual ~PartDividedSolver();
			explicit PartDividedSolver(const std::string &name = "part_divided_solver", Size max_iter_count = 100, double max_error = 1e-10);
			PartDividedSolver(const PartDividedSolver &other);
			PartDividedSolver(PartDividedSolver &&other);
			PartDividedSolver& operator=(const PartDividedSolver &other);
			PartDividedSolver& operator=(PartDividedSolver &&other);

		private:
			struct Imp;
			aris::core::ImpPtr<Imp> imp_;
		};
		class LltGroundDividedSolver :public GroundDividedSolver
		{
		public:
			static const std::string& Type() { static const std::string type("LltGroundDividedSolver"); return type; }
			auto virtual type() const->const std::string& override{ return Type(); }
			auto virtual allocateMemory()->void override;
			auto virtual kinPos()->void override;
			auto virtual kinVel()->void override;
			auto kinAcc()->void;
			auto dynFce()->void;
			auto virtual dynAccAndFce()->void override
			{
				kinAcc();
				dynFce();
			};

			virtual ~LltGroundDividedSolver();
			explicit LltGroundDividedSolver(const std::string &name = "llt_ground_divided_solver");
			LltGroundDividedSolver(const LltGroundDividedSolver &other);
			LltGroundDividedSolver(LltGroundDividedSolver &&other);
			LltGroundDividedSolver& operator=(const LltGroundDividedSolver &other);
			LltGroundDividedSolver& operator=(LltGroundDividedSolver &&other);

		private:
			struct Imp;
			aris::core::ImpPtr<Imp> imp_;
		};
		class LltPartDividedSolver :public PartDividedSolver
		{
		public:
			static const std::string& Type() { static const std::string type("LltPartDividedSolver"); return type; }
			auto virtual type() const->const std::string& override{ return Type(); }
			auto virtual allocateMemory()->void override;
			auto virtual kinPos()->void override;
			auto virtual kinVel()->void override;
			auto kinAcc()->void;
			auto dynFce()->void;
			auto virtual dynAccAndFce()->void override 
			{
				kinAcc();
				dynFce();
			};

			virtual ~LltPartDividedSolver();
			explicit LltPartDividedSolver(const std::string &name = "llt_part_divided_solver", Size max_iter_count = 100, double max_error = 1e-10);
			LltPartDividedSolver(const LltPartDividedSolver &other);
			LltPartDividedSolver(LltPartDividedSolver &&other);
			LltPartDividedSolver& operator=(const LltPartDividedSolver &other);
			LltPartDividedSolver& operator=(LltPartDividedSolver &&other);

		private:
			struct Imp;
			aris::core::ImpPtr<Imp> imp_;
		};
		class DiagSolver : public Solver
		{
		public:
			struct Relation
			{
				struct Block { Constraint* constraint; bool is_I; };
				
				Part *prtI;
				Part *prtJ;
				Size dim;
				std::vector<Block> cst_pool_;
			};
			struct Diag
			{
				double cm[36], x[6], b[6]; // determine I or J automatically
				double U[36], tau[6];
				double Q[36], R[36];
				double RN[36];
				//double T1[36], L1[36], R1[36], U1[36], tau1[6];
				Size rows;
				Relation *rel;
				Part *part;
				Diag *rd;//related diag
				bool is_I;

				std::function<void(double*)> fast;
			};
			struct Remainder
			{
				struct Block { Diag* diag; bool is_I; };
				
				double cmI[36], cmJ[36], x[6], b[6];
				std::vector<Block> cm_blk_series;
				Relation *rel;
			};
			static const std::string& Type() { static const std::string type("DiagSolver"); return type; }
			auto virtual type() const->const std::string& override{ return Type(); }
			auto virtual allocateMemory()->void override;
			auto virtual kinPos()->void override;
			auto virtual kinVel()->void override;
			auto kinAcc()->void;
			auto dynFce()->void;
			auto virtual dynAccAndFce()->void override
			{
				kinAcc();
				dynFce();
			};
			auto updDiagCm()->void;
			auto updDiagCp()->void;
			auto updDiagCv()->void;
			auto updDiagCa()->void;
			auto updDiagPf()->void;
			auto updRemainderCm()->void;
			auto updRemainderCp()->void;
			auto updRemainderCv()->void;
			auto updRemainderCa()->void;
			auto updA()->void;
			auto updB()->void;
			auto updX()->void;
			auto updBf()->void;
			auto updXf()->void;
			auto relationPool()->std::vector<Relation>&;
			auto activePartPool()->std::vector<Part*>&;
			auto diagPool()->std::vector<Diag>&;
			auto remainderPool()->std::vector<Remainder>&;
			auto plotRelation()->void;
			auto plotDiag()->void;
			auto plotRemainder()->void;

			virtual ~DiagSolver();
			explicit DiagSolver(const std::string &name = "diag_solver", Size max_iter_count = 100, double max_error = 1e-10);
			DiagSolver(const DiagSolver &other);
			DiagSolver(DiagSolver &&other);
			DiagSolver& operator=(const DiagSolver &other);
			DiagSolver& operator=(DiagSolver &&other);

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
		class AdamsSimulator :public SolverSimulator
		{
		public:
			static auto Type()->const std::string &{ static const std::string type{ "AdamsSimulator" }; return type; }
			auto virtual type() const->const std::string& override{ return Type(); }
			auto saveAdams(const std::string &filename, SimResult &result, Size pos = -1)->void;
			auto saveAdams(std::ofstream &file, SimResult &result, Size pos = -1)->void;
			auto adamsID(const Marker &mak)const->Size;
			auto adamsID(const Part &prt)const->Size;
			auto adamsID(const Element &ele)const->Size { return ele.id() + 1; };

			virtual ~AdamsSimulator();
			explicit AdamsSimulator(const std::string &name = "adams_solver", Solver *solver = nullptr);
			AdamsSimulator(const AdamsSimulator&);
			AdamsSimulator(AdamsSimulator&&);
			AdamsSimulator& operator=(const AdamsSimulator&);
			AdamsSimulator& operator=(AdamsSimulator&&);

		private:
			struct Imp;
			aris::core::ImpPtr<Imp> imp_;
		};
	}
}

#endif
