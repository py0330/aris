#ifndef ARIS_PLAN_FUNCTION_H_
#define ARIS_PLAN_FUNCTION_H_

#include <list>
#include <cmath>
#include <iostream>
#include <functional>
#include <map>

#include <aris_core.h>
#include <aris_plan_command.h>

namespace aris
{
	namespace dynamic { class Model; class SimResult; }
	namespace control { class Master; }
	
	
	/// \brief 轨迹规划命名空间
	/// \ingroup aris
	/// 
	///
	///
	namespace plan 
	{
		struct PlanParam
		{
			std::uint32_t count_;
			aris::dynamic::Model* model_;
			aris::control::Master* master_;
			void *param_;
			std::uint32_t param_size_;
		};
		using PlanFunction = std::function<int(const PlanParam &plan_param)>;
		using ParseFunction = std::function<void(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::MsgBase &msg_out)>;

		class Plan:public aris::core::Object
		{
		public:
			static auto Type()->const std::string &{ static const std::string type("Plan"); return std::ref(type); }
			auto virtual type() const->const std::string& override{ return Type(); }
			auto virtual prepairNrt(const std::string &cmd, const std::map<std::string,std::string> &params, aris::core::MsgBase &msg_out)->void {}
			auto virtual runRT()->void {}
			auto virtual finishNrt()->void {}
			auto command()->plan::Command &;

			virtual ~Plan();
			explicit Plan(const std::string &name = "plan");
			Plan(const Plan &);
			Plan(Plan &&);
			Plan& operator=(const Plan &);
			Plan& operator=(Plan &&);

		private:
			struct Imp;
			aris::core::ImpPtr<Imp> imp_;
		};
		class PlanRoot :public aris::core::Object
		{
		public:
			static auto Type()->const std::string &{ static const std::string type("PlanRoot"); return std::ref(type); }
			auto virtual type() const->const std::string& override{ return Type(); }
			auto planPool()->aris::core::ObjectPool<Plan> &;
			auto planPool()const->const aris::core::ObjectPool<Plan> &{ return const_cast<std::decay_t<decltype(*this)> *>(this)->planPool(); }

			virtual ~PlanRoot();
			explicit PlanRoot(const std::string &name = "plan_root");
			PlanRoot(const PlanRoot &);
			PlanRoot(PlanRoot &&);
			PlanRoot& operator=(const PlanRoot &);
			PlanRoot& operator=(PlanRoot &&);

		private:
			struct Imp;
			aris::core::ImpPtr<Imp> imp_;
		};


		auto simulatePlan(aris::dynamic::Model *model, aris::plan::Plan *plan)->void;
		auto simulateCommand(std::string cmd, aris::plan::PlanRoot *plan_root, aris::dynamic::Model *model, aris::dynamic::SimResult *result)->void;

		auto executeCommand()->void;


	}
}


#endif