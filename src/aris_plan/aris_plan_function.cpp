#include <algorithm>

#include"aris_plan_function.h"
#include"aris_plan_command.h"

namespace aris
{
	namespace plan
	{

		struct Plan::Imp
		{
			Imp(){}
		};
		auto Plan::command()->plan::Command & { return dynamic_cast<plan::Command&>(children().front()); }
		Plan::~Plan() = default;
		Plan::Plan(const std::string &name):Object(name), imp_(new Imp)
		{
			registerType<plan::Command>();
			add<plan::Command>(name);
		}
		Plan::Plan(const Plan &) = default;
		Plan::Plan(Plan &&) = default;
		Plan& Plan::operator=(const Plan &) = default;
		Plan& Plan::operator=(Plan &&) = default;
		
		struct PlanRoot::Imp { Imp() {} };
		auto PlanRoot::planPool()->aris::core::ObjectPool<Plan> & { return dynamic_cast<aris::core::ObjectPool<Plan> &>(children().front()); }
		PlanRoot::~PlanRoot() = default;
		PlanRoot::PlanRoot(const std::string &name):Object(name) 
		{
			registerType<aris::core::ObjectPool<Plan> >();
			add<aris::core::ObjectPool<Plan> >("plan_pool_object");
		}
		PlanRoot::PlanRoot(const PlanRoot &) = default;
		PlanRoot::PlanRoot(PlanRoot &&) = default;
		PlanRoot& PlanRoot::operator=(const PlanRoot &) = default;
		PlanRoot& PlanRoot::operator=(PlanRoot &&) = default;
		
		
		auto simulateCommand(std::string cmd_string, aris::plan::PlanRoot *plan_root, aris::dynamic::Model *model)->void
		{
			CommandParser parser;
			for (auto &plan : plan_root->planPool())
			{
				parser.commandPool().add<Command>(plan.command());
			}
			
			std::string cmd;
			std::map<std::string, std::string> param_map;
			parser.parse(cmd_string, cmd, param_map);

			std::cout << "cmd:" << cmd <<std::endl;
			for (auto param : param_map)
			{
				std::cout << "  " << param.first << "   " << param.second << std::endl;
			}

		}
		
		
		
		
		
	}
}
