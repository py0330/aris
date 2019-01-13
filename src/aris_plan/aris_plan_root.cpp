#include <algorithm>
#include <future>

#include"aris_plan_function.h"
#include"aris_plan_root.h"

#include <aris_control.h>
#include <aris_dynamic.h>

namespace aris::plan
{
	struct Plan::Imp{};
	auto Plan::command()->aris::core::Command & { return dynamic_cast<aris::core::Command&>(children().front()); }
	Plan::~Plan() = default;
	Plan::Plan(const std::string &name) :Object(name), imp_(new Imp)
	{
		registerType<aris::core::Command>();
		add<aris::core::Command>(name);
	}
	Plan::Plan(const Plan &) = default;
	Plan::Plan(Plan &&) = default;
	Plan& Plan::operator=(const Plan &) = default;
	Plan& Plan::operator=(Plan &&) = default;

	struct PlanRoot::Imp { Imp() {} };
	auto PlanRoot::planPool()->aris::core::ObjectPool<Plan> & { return dynamic_cast<aris::core::ObjectPool<Plan> &>(children().front()); }
	auto PlanRoot::planParser()->aris::core::CommandParser
	{
		aris::core::CommandParser parser;
		for (auto &plan : planPool()) parser.commandPool().add<aris::core::Command>(plan.command());
		return parser;
	}
	PlanRoot::~PlanRoot() = default;
	PlanRoot::PlanRoot(const std::string &name) :Object(name)
	{
		registerType<aris::core::ObjectPool<Plan> >();
		add<aris::core::ObjectPool<Plan> >("plan_pool_object");
	}
	PlanRoot::PlanRoot(const PlanRoot &) = default;
	PlanRoot::PlanRoot(PlanRoot &&) = default;
	PlanRoot& PlanRoot::operator=(const PlanRoot &) = default;
	PlanRoot& PlanRoot::operator=(PlanRoot &&) = default;

	auto default_prepair_check_option(const std::map<std::string, std::string> &cmd_params, PlanTarget &param)->void
	{
		for (auto cmd_param : cmd_params)
		{
			if (cmd_param.first == "check_all")
			{
				param.option &= ~(
					Plan::NOT_CHECK_POS_MIN |
					Plan::NOT_CHECK_POS_MAX |
					Plan::NOT_CHECK_POS_CONTINUOUS |
					Plan::NOT_CHECK_POS_CONTINUOUS_AT_START |
					Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER |
					Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER_AT_START |
					Plan::NOT_CHECK_POS_FOLLOWING_ERROR |
					Plan::NOT_CHECK_VEL_MIN |
					Plan::NOT_CHECK_VEL_MAX |
					Plan::NOT_CHECK_VEL_CONTINUOUS |
					Plan::NOT_CHECK_VEL_CONTINUOUS_AT_START |
					Plan::NOT_CHECK_VEL_FOLLOWING_ERROR);
			}
			else if (cmd_param.first == "check_none")
			{
				param.option |=
					Plan::NOT_CHECK_POS_MIN |
					Plan::NOT_CHECK_POS_MAX |
					Plan::NOT_CHECK_POS_CONTINUOUS |
					Plan::NOT_CHECK_POS_CONTINUOUS_AT_START |
					Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER |
					Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER_AT_START |
					Plan::NOT_CHECK_POS_FOLLOWING_ERROR |
					Plan::NOT_CHECK_VEL_MIN |
					Plan::NOT_CHECK_VEL_MAX |
					Plan::NOT_CHECK_VEL_CONTINUOUS |
					Plan::NOT_CHECK_VEL_CONTINUOUS_AT_START |
					Plan::NOT_CHECK_VEL_FOLLOWING_ERROR;
			}
			else if (cmd_param.first == "check_pos")
			{
				param.option &= ~(
					Plan::NOT_CHECK_POS_MIN |
					Plan::NOT_CHECK_POS_MAX |
					Plan::NOT_CHECK_POS_CONTINUOUS |
					Plan::NOT_CHECK_POS_CONTINUOUS_AT_START |
					Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER |
					Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER_AT_START |
					Plan::NOT_CHECK_POS_FOLLOWING_ERROR);
			}
			else if (cmd_param.first == "not_check_pos")
			{
				param.option |=
					Plan::NOT_CHECK_POS_MIN |
					Plan::NOT_CHECK_POS_MAX |
					Plan::NOT_CHECK_POS_CONTINUOUS |
					Plan::NOT_CHECK_POS_CONTINUOUS_AT_START |
					Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER |
					Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER_AT_START |
					Plan::NOT_CHECK_POS_FOLLOWING_ERROR;
			}
			else if (cmd_param.first == "check_vel")
			{
				param.option &= ~(
					Plan::NOT_CHECK_VEL_MIN |
					Plan::NOT_CHECK_VEL_MAX |
					Plan::NOT_CHECK_VEL_CONTINUOUS |
					Plan::NOT_CHECK_VEL_CONTINUOUS_AT_START |
					Plan::NOT_CHECK_VEL_FOLLOWING_ERROR);
			}
			else if (cmd_param.first == "not_check_vel")
			{
				param.option |=
					Plan::NOT_CHECK_VEL_MIN |
					Plan::NOT_CHECK_VEL_MAX |
					Plan::NOT_CHECK_VEL_CONTINUOUS |
					Plan::NOT_CHECK_VEL_CONTINUOUS_AT_START |
					Plan::NOT_CHECK_VEL_FOLLOWING_ERROR;
			}
			else if (cmd_param.first == "check_pos_min")
			{
				param.option &= ~Plan::NOT_CHECK_POS_MIN;
			}
			else if (cmd_param.first == "not_check_pos_min")
			{
				param.option |= Plan::NOT_CHECK_POS_MIN;
			}
			else if (cmd_param.first == "check_pos_max")
			{
				param.option &= ~Plan::NOT_CHECK_POS_MAX;
			}
			else if (cmd_param.first == "not_check_pos_max")
			{
				param.option |= Plan::NOT_CHECK_POS_MAX;
			}
			else if (cmd_param.first == "check_pos_continuous")
			{
				param.option &= ~Plan::NOT_CHECK_POS_CONTINUOUS;
			}
			else if (cmd_param.first == "not_check_pos_continuous")
			{
				param.option |= Plan::NOT_CHECK_POS_CONTINUOUS;
			}
			else if (cmd_param.first == "check_pos_continuous_at_start")
			{
				param.option &= ~Plan::NOT_CHECK_POS_CONTINUOUS_AT_START;
			}
			else if (cmd_param.first == "not_check_pos_continuous_at_start")
			{
				param.option |= Plan::NOT_CHECK_POS_CONTINUOUS_AT_START;
			}
			else if (cmd_param.first == "check_pos_continuous_second_order")
			{
				param.option &= ~Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
			}
			else if (cmd_param.first == "not_check_pos_continuous_second_order")
			{
				param.option |= Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
			}
			else if (cmd_param.first == "check_pos_continuous_second_order_at_start")
			{
				param.option &= ~Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER_AT_START;
			}
			else if (cmd_param.first == "not_check_pos_continuous_second_order_at_start")
			{
				param.option |= Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER_AT_START;
			}
			else if (cmd_param.first == "check_pos_following_error")
			{
				param.option &= ~Plan::NOT_CHECK_POS_FOLLOWING_ERROR;
			}
			else if (cmd_param.first == "not_check_pos_following_error")
			{
				param.option |= Plan::NOT_CHECK_POS_FOLLOWING_ERROR;
			}
			else if (cmd_param.first == "check_vel_min")
			{
				param.option &= ~Plan::NOT_CHECK_VEL_MIN;
			}
			else if (cmd_param.first == "not_check_vel_min")
			{
				param.option |= Plan::NOT_CHECK_VEL_MIN;
			}
			else if (cmd_param.first == "check_vel_max")
			{
				param.option &= ~Plan::NOT_CHECK_VEL_MAX;
			}
			else if (cmd_param.first == "not_check_vel_max")
			{
				param.option |= Plan::NOT_CHECK_VEL_MAX;
			}
			else if (cmd_param.first == "check_vel_continuous")
			{
				param.option &= ~Plan::NOT_CHECK_VEL_CONTINUOUS;
			}
			else if (cmd_param.first == "not_check_vel_continuous")
			{
				param.option |= Plan::NOT_CHECK_VEL_CONTINUOUS;
			}
			else if (cmd_param.first == "check_vel_continuous_at_start")
			{
				param.option &= ~Plan::NOT_CHECK_VEL_CONTINUOUS_AT_START;
			}
			else if (cmd_param.first == "not_check_vel_continuous_at_start")
			{
				param.option |= Plan::NOT_CHECK_VEL_CONTINUOUS_AT_START;
			}
			else if (cmd_param.first == "check_vel_following_error")
			{
				param.option &= ~Plan::NOT_CHECK_VEL_FOLLOWING_ERROR;
			}
			else if (cmd_param.first == "not_check_vel_following_error")
			{
				param.option |= Plan::NOT_CHECK_VEL_FOLLOWING_ERROR;
			}
		}
	}

	struct EnableParam 
	{
		std::int32_t limit_time;
		std::vector<int> active_motor;
	};
	struct EnablePlan::Imp { Imp() {} };
	auto EnablePlan::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		EnableParam param;

		for (auto cmd_param : params)
		{
			if (cmd_param.first == "limit_time")
				param.limit_time = std::stoi(cmd_param.second);
			else if (cmd_param.first == "all")
			{
				param.active_motor.clear();
				param.active_motor.resize(dynamic_cast<aris::control::Controller *>(target.master)->motionPool().size(), 1);
			}
			else if (cmd_param.first == "none")
			{
				param.active_motor.clear();
				param.active_motor.resize(dynamic_cast<aris::control::Controller *>(target.master)->motionPool().size(), 0);
			}
			else if (cmd_param.first == "motion_id")
			{
				param.active_motor.clear();
				param.active_motor.resize(dynamic_cast<aris::control::Controller *>(target.master)->motionPool().size(), 0);
				param.active_motor.at(std::stoi(cmd_param.second)) = 1;
			}
			else if (cmd_param.first == "physical_id")
			{
				param.active_motor.clear();
				param.active_motor.resize(dynamic_cast<aris::control::Controller *>(target.master)->motionPool().size(), 0);
				param.active_motor.at(dynamic_cast<aris::control::Controller*>(target.master)->motionAtPhy(std::stoi(cmd_param.second)).phyId()) = 1;
			}
			else if (cmd_param.first == "slave_id")
			{
				param.active_motor.clear();
				param.active_motor.resize(dynamic_cast<aris::control::Controller *>(target.master)->motionPool().size(), 0);
				param.active_motor.at(dynamic_cast<aris::control::Controller*>(target.master)->motionAtPhy(std::stoi(cmd_param.second)).slaId()) = 1;
			}
			else if (cmd_param.first == "check_all")
			{
				target.option &= ~(
					NOT_CHECK_POS_MIN |
					NOT_CHECK_POS_MAX |
					NOT_CHECK_POS_CONTINUOUS |
					NOT_CHECK_POS_CONTINUOUS_AT_START |
					NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER |
					NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER_AT_START |
					NOT_CHECK_POS_FOLLOWING_ERROR |
					NOT_CHECK_VEL_MIN |
					NOT_CHECK_VEL_MAX |
					NOT_CHECK_VEL_CONTINUOUS |
					NOT_CHECK_VEL_CONTINUOUS_AT_START |
					NOT_CHECK_VEL_FOLLOWING_ERROR);
			}
			else if (cmd_param.first == "check_none")
			{
				target.option |=
					NOT_CHECK_POS_MIN |
					NOT_CHECK_POS_MAX |
					NOT_CHECK_POS_CONTINUOUS |
					NOT_CHECK_POS_CONTINUOUS_AT_START |
					NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER |
					NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER_AT_START |
					NOT_CHECK_POS_FOLLOWING_ERROR |
					NOT_CHECK_VEL_MIN |
					NOT_CHECK_VEL_MAX |
					NOT_CHECK_VEL_CONTINUOUS |
					NOT_CHECK_VEL_CONTINUOUS_AT_START |
					NOT_CHECK_VEL_FOLLOWING_ERROR;
			}
			else
				throw std::runtime_error("unknown input target when prepair EnablePlan");
		}

		target.param = param;
	}
	auto EnablePlan::executeRT(PlanTarget &target)->int
	{
		auto controller = dynamic_cast<aris::control::Controller *>(target.master);
		auto &param = std::any_cast<EnableParam &>(target.param);

		bool is_all_finished = true;
		for (std::size_t i = 0; i < controller->motionPool().size(); ++i)
		{
			auto &cm = controller->motionPool().at(i);

			if (param.active_motor[i])
			{
				auto ret = cm.enable();
				if (ret)
				{
					is_all_finished = false;

					if (target.count % 1000 == 0)
					{
						controller->mout() << "Unenabled motor, slave id: " << cm.id() << ", absolute id: " << i << ", ret: " << ret << std::endl;
					}
				}
			}
		}

		return (is_all_finished || target.count >= param.limit_time) ? 0 : 1;
	}
	auto EnablePlan::collectNrt(PlanTarget &param)->void {}
	EnablePlan::~EnablePlan() = default;
	EnablePlan::EnablePlan(const std::string &name) :Plan(name), imp_(new Imp)
	{
		command().loadXmlStr(
			"<en default_child_type=\"Param\">"
			"	<group type=\"GroupParam\" default_child_type=\"Param\">"
			"		<limit_time default=\"5000\"/>"
			"		<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"all\">"
			"			<all abbreviation=\"a\"/>"
			"			<none abbreviation=\"n\"/>"
			"			<motion_id abbreviation=\"m\" default=\"0\"/>"
			"			<physical_id abbreviation=\"p\" default=\"0\"/>"
			"			<slave_id abbreviation=\"s\" default=\"0\"/>"
			"		</unique>"
			"		<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_none\">"
			"			<check_all/>"
			"			<check_none/>"
			"			<group type=\"GroupParam\" default_child_type=\"Param\">"
			"				<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_pos\">"
			"					<check_pos/>"
			"					<not_check_pos/>"
			"					<group type=\"GroupParam\" default_child_type=\"Param\">"
			"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_pos_max\">"
			"							<check_pos_max/>"
			"							<not_check_pos_max/>"
			"						</unique>"
			"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_pos_min\">"
			"							<check_pos_min/>"
			"							<not_check_pos_min/>"
			"						</unique>"
			"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_pos_continuous\">"
			"							<check_pos_continuous/>"
			"							<not_check_pos_continuous/>"
			"						</unique>"
			"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_pos_continuous_at_start\">"
			"							<check_pos_continuous_at_start/>"
			"							<not_check_pos_continuous_at_start/>"
			"						</unique>"
			"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_pos_continuous_second_order\">"
			"							<check_pos_continuous_second_order/>"
			"							<not_check_pos_continuous_second_order/>"
			"						</unique>"
			"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_pos_continuous_second_order_at_start\">"
			"							<check_pos_continuous_second_order_at_start/>"
			"							<not_check_pos_continuous_second_order_at_start/>"
			"						</unique>"
			"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_pos_following_error\">"
			"							<check_pos_following_error/>"
			"							<not_check_pos_following_error />"
			"						</unique>"
			"					</group>"
			"				</unique>"
			"				<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_vel\">"
			"					<check_vel/>"
			"					<not_check_vel/>"
			"					<group type=\"GroupParam\" default_child_type=\"Param\">"
			"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_vel_max\">"
			"							<check_vel_max/>"
			"							<not_check_vel_max/>"
			"						</unique>"
			"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_vel_min\">"
			"							<check_vel_min/>"
			"							<not_check_vel_min/>"
			"						</unique>"
			"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_vel_continuous\">"
			"							<check_vel_continuous/>"
			"							<not_check_vel_continuous/>"
			"						</unique>"
			"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_vel_continuous_at_start\">"
			"							<check_vel_continuous_at_start/>"
			"							<not_check_vel_continuous_at_start/>"
			"						</unique>"
			"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_vel_following_error\">"
			"							<check_vel_following_error/>"
			"							<not_check_vel_following_error />"
			"						</unique>"
			"					</group>"
			"				</unique>"
			"			</group>"
			"		</unique>"
			"	</group>"
			"</en>");
	}
	EnablePlan::EnablePlan(const EnablePlan &) = default;
	EnablePlan::EnablePlan(EnablePlan &&) = default;
	EnablePlan& EnablePlan::operator=(const EnablePlan &) = default;
	EnablePlan& EnablePlan::operator=(EnablePlan &&) = default;

	struct DisableParam
	{
		std::int32_t limit_time;
		std::vector<int> active_motor;
	};
	struct DisablePlan::Imp { Imp() {} };
	auto DisablePlan::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		DisableParam param;

		for (auto cmd_param : params)
		{
			if (cmd_param.first == "limit_time")
				param.limit_time = std::stoi(cmd_param.second);
			else if (cmd_param.first == "all")
			{
				param.active_motor.clear();
				param.active_motor.resize(dynamic_cast<aris::control::Controller *>(target.master)->motionPool().size(), 1);
			}
			else if (cmd_param.first == "none")
			{
				param.active_motor.clear();
				param.active_motor.resize(dynamic_cast<aris::control::Controller *>(target.master)->motionPool().size(), 0);
			}
			else if (cmd_param.first == "motion_id")
			{
				param.active_motor.clear();
				param.active_motor.resize(dynamic_cast<aris::control::Controller *>(target.master)->motionPool().size(), 0);
				param.active_motor.at(std::stoi(cmd_param.second)) = 1;
			}
			else if (cmd_param.first == "physical_id")
			{
				param.active_motor.clear();
				param.active_motor.resize(dynamic_cast<aris::control::Controller *>(target.master)->motionPool().size(), 0);
				param.active_motor.at(dynamic_cast<aris::control::Controller*>(target.master)->motionAtPhy(std::stoi(cmd_param.second)).phyId()) = 1;
			}
			else if (cmd_param.first == "slave_id")
			{
				param.active_motor.clear();
				param.active_motor.resize(dynamic_cast<aris::control::Controller *>(target.master)->motionPool().size(), 0);
				param.active_motor.at(dynamic_cast<aris::control::Controller*>(target.master)->motionAtPhy(std::stoi(cmd_param.second)).slaId()) = 1;
			}
			else if (cmd_param.first == "check_all")
			{
				target.option &= ~(
					NOT_CHECK_POS_MIN |
					NOT_CHECK_POS_MAX |
					NOT_CHECK_POS_CONTINUOUS |
					NOT_CHECK_POS_CONTINUOUS_AT_START |
					NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER |
					NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER_AT_START |
					NOT_CHECK_POS_FOLLOWING_ERROR |
					NOT_CHECK_VEL_MIN |
					NOT_CHECK_VEL_MAX |
					NOT_CHECK_VEL_CONTINUOUS |
					NOT_CHECK_VEL_CONTINUOUS_AT_START |
					NOT_CHECK_VEL_FOLLOWING_ERROR);
			}
			else if (cmd_param.first == "check_none")
			{
				target.option |=
					NOT_CHECK_POS_MIN |
					NOT_CHECK_POS_MAX |
					NOT_CHECK_POS_CONTINUOUS |
					NOT_CHECK_POS_CONTINUOUS_AT_START |
					NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER |
					NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER_AT_START |
					NOT_CHECK_POS_FOLLOWING_ERROR |
					NOT_CHECK_VEL_MIN |
					NOT_CHECK_VEL_MAX |
					NOT_CHECK_VEL_CONTINUOUS |
					NOT_CHECK_VEL_CONTINUOUS_AT_START |
					NOT_CHECK_VEL_FOLLOWING_ERROR;
			}
			else
				throw std::runtime_error("unknown input target when prepair DisablePlan");
		}

		target.param = param;
	}
	auto DisablePlan::executeRT(PlanTarget &target)->int
	{
		auto controller = dynamic_cast<aris::control::Controller *>(target.master);
		auto &param = std::any_cast<DisableParam &>(target.param);

		bool is_all_finished = true;
		for (std::size_t i = 0; i < controller->motionPool().size(); ++i)
		{
			auto &cm = controller->motionPool().at(i);

			if (param.active_motor[i])
			{
				auto ret = cm.disable();
				if (ret)
				{
					is_all_finished = false;

					if (target.count % 1000 == 0)
					{
						controller->mout() << "Undisabled motor, slave id: " << cm.id() << ", absolute id: " << i << ", ret: " << ret << std::endl;
					}
				}
			}
		}

		return (is_all_finished || target.count >= param.limit_time) ? 0 : 1;
	}
	auto DisablePlan::collectNrt(PlanTarget &param)->void {}
	DisablePlan::~DisablePlan() = default;
	DisablePlan::DisablePlan(const std::string &name) :Plan(name), imp_(new Imp)
	{
		command().loadXmlStr(
			"<ds default_child_type=\"Param\">"
			"	<group type=\"GroupParam\" default_child_type=\"Param\">"
			"		<limit_time default=\"5000\"/>"
			"		<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"all\">"
			"			<all abbreviation=\"a\"/>"
			"			<none abbreviation=\"n\"/>"
			"			<motion_id abbreviation=\"m\" default=\"0\"/>"
			"			<physical_id abbreviation=\"p\" default=\"0\"/>"
			"			<slave_id abbreviation=\"s\" default=\"0\"/>"
			"		</unique>"
			"		<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_none\">"
			"			<check_all/>"
			"			<check_none/>"
			"			<group type=\"GroupParam\" default_child_type=\"Param\">"
			"				<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_pos\">"
			"					<check_pos/>"
			"					<not_check_pos/>"
			"					<group type=\"GroupParam\" default_child_type=\"Param\">"
			"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_pos_max\">"
			"							<check_pos_max/>"
			"							<not_check_pos_max/>"
			"						</unique>"
			"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_pos_min\">"
			"							<check_pos_min/>"
			"							<not_check_pos_min/>"
			"						</unique>"
			"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_pos_continuous\">"
			"							<check_pos_continuous/>"
			"							<not_check_pos_continuous/>"
			"						</unique>"
			"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_pos_continuous_at_start\">"
			"							<check_pos_continuous_at_start/>"
			"							<not_check_pos_continuous_at_start/>"
			"						</unique>"
			"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_pos_continuous_second_order\">"
			"							<check_pos_continuous_second_order/>"
			"							<not_check_pos_continuous_second_order/>"
			"						</unique>"
			"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_pos_continuous_second_order_at_start\">"
			"							<check_pos_continuous_second_order_at_start/>"
			"							<not_check_pos_continuous_second_order_at_start/>"
			"						</unique>"
			"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_pos_following_error\">"
			"							<check_pos_following_error/>"
			"							<not_check_pos_following_error />"
			"						</unique>"
			"					</group>"
			"				</unique>"
			"				<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_vel\">"
			"					<check_vel/>"
			"					<not_check_vel/>"
			"					<group type=\"GroupParam\" default_child_type=\"Param\">"
			"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_vel_max\">"
			"							<check_vel_max/>"
			"							<not_check_vel_max/>"
			"						</unique>"
			"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_vel_min\">"
			"							<check_vel_min/>"
			"							<not_check_vel_min/>"
			"						</unique>"
			"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_vel_continuous\">"
			"							<check_vel_continuous/>"
			"							<not_check_vel_continuous/>"
			"						</unique>"
			"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_vel_continuous_at_start\">"
			"							<check_vel_continuous_at_start/>"
			"							<not_check_vel_continuous_at_start/>"
			"						</unique>"
			"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_vel_following_error\">"
			"							<check_vel_following_error/>"
			"							<not_check_vel_following_error />"
			"						</unique>"
			"					</group>"
			"				</unique>"
			"			</group>"
			"		</unique>"
			"	</group>"
			"</ds>");
	}
	DisablePlan::DisablePlan(const DisablePlan &) = default;
	DisablePlan::DisablePlan(DisablePlan &&) = default;
	DisablePlan& DisablePlan::operator=(const DisablePlan &) = default;
	DisablePlan& DisablePlan::operator=(DisablePlan &&) = default;

	struct HomeParam
	{
		std::int32_t limit_time;
		std::vector<int> active_motor;
	};
	struct HomePlan::Imp { Imp() {} };
	auto HomePlan::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		HomeParam param;

		for (auto cmd_param : params)
		{
			if (cmd_param.first == "limit_time")
				param.limit_time = std::stoi(cmd_param.second);
			else if (cmd_param.first == "all")
			{
				param.active_motor.clear();
				param.active_motor.resize(dynamic_cast<aris::control::Controller *>(target.master)->motionPool().size(), 1);
			}
			else if (cmd_param.first == "none")
			{
				param.active_motor.clear();
				param.active_motor.resize(dynamic_cast<aris::control::Controller *>(target.master)->motionPool().size(), 0);
			}
			else if (cmd_param.first == "motion_id")
			{
				param.active_motor.clear();
				param.active_motor.resize(dynamic_cast<aris::control::Controller *>(target.master)->motionPool().size(), 0);
				param.active_motor.at(std::stoi(cmd_param.second)) = 1;
			}
			else if (cmd_param.first == "physical_id")
			{
				param.active_motor.clear();
				param.active_motor.resize(dynamic_cast<aris::control::Controller *>(target.master)->motionPool().size(), 0);
				param.active_motor.at(dynamic_cast<aris::control::Controller*>(target.master)->motionAtPhy(std::stoi(cmd_param.second)).phyId()) = 1;
			}
			else if (cmd_param.first == "slave_id")
			{
				param.active_motor.clear();
				param.active_motor.resize(dynamic_cast<aris::control::Controller *>(target.master)->motionPool().size(), 0);
				param.active_motor.at(dynamic_cast<aris::control::Controller*>(target.master)->motionAtPhy(std::stoi(cmd_param.second)).slaId()) = 1;
			}
		}

		for (aris::Size i = 0; i<param.active_motor.size(); ++i)
		{
			if (param.active_motor[i])
			{
				std::int8_t method = std::stoi(params.at(std::string("method")));
				if (method < 1 || method > 35) throw std::runtime_error("invalid home method");

				std::int32_t offset = std::stoi(params.at(std::string("offset")));
				std::uint32_t high_speed = std::stoi(params.at(std::string("high_speed")));
				std::uint32_t low_speed = std::stoi(params.at(std::string("low_speed")));
				std::uint32_t acc = std::stoi(params.at(std::string("acceleration")));

				auto controller = dynamic_cast<aris::control::EthercatController *>(target.master);
				auto &cm = dynamic_cast<aris::control::EthercatMotion &>(controller->motionPool()[i]);

				cm.writeSdo(0x6098, 0x00, method);
				std::int8_t method_read;
				cm.readSdo(0x6098, 0x00, method_read);
				if (method_read != method)throw std::runtime_error("home sdo write failed method");

				cm.writeSdo(0x607C, 0x00, offset);
				std::int32_t offset_read;
				cm.readSdo(0x607C, 0x00, offset_read);
				if (offset_read != offset)throw std::runtime_error("home sdo write failed offset");

				cm.writeSdo(0x6099, 0x01, high_speed);
				std::int32_t high_speed_read;
				cm.readSdo(0x6099, 0x01, high_speed_read);
				if (high_speed_read != high_speed)throw std::runtime_error("home sdo write failed high_speed");

				cm.writeSdo(0x6099, 0x02, low_speed);
				std::int32_t low_speed_read;
				cm.readSdo(0x6099, 0x02, low_speed_read);
				if (low_speed_read != low_speed)throw std::runtime_error("home sdo write failed low_speed");

				cm.writeSdo(0x609A, 0x00, acc);
				std::int32_t acc_read;
				cm.readSdo(0x609A, 0x00, acc_read);
				if (acc_read != acc)throw std::runtime_error("home sdo write failed acc");
			}
		}

		target.param = param;
	}
	auto HomePlan::executeRT(PlanTarget &target)->int
	{
		auto controller = dynamic_cast<aris::control::Controller *>(target.master);
		auto &param = std::any_cast<HomeParam &>(target.param);

		bool is_all_finished = true;
		for (std::size_t i = 0; i < controller->motionPool().size(); ++i)
		{
			auto &cm = controller->motionPool().at(i);

			if (param.active_motor[i])
			{
				if (target.count == 1) cm.setControlWord(0x000F);
				auto ret = cm.home();
				if (ret)
				{
					is_all_finished = false;

					if (target.count % 1000 == 0)
					{
						controller->mout() << "Unhomed motor, slave id: " << cm.id() << ", absolute id: " << i << ", ret: " << ret << std::endl;
					}
				}
			}
		}

		return (is_all_finished || target.count >= param.limit_time) ? 0 : 1;
	}
	auto HomePlan::collectNrt(PlanTarget &param)->void {}
	HomePlan::~HomePlan() = default;
	HomePlan::HomePlan(const std::string &name) :Plan(name), imp_(new Imp)
	{
		command().loadXmlStr(
			"<hm default_child_type=\"Param\">"
			"	<group type=\"GroupParam\" default_child_type=\"Param\">"
			"		<method default=\"35\"/>"
			"		<offset default=\"0\"/>"
			"		<high_speed default=\"20000\"/>"
			"		<low_speed default=\"10000\"/>"
			"		<acceleration default=\"100000\"/>"
			"		<limit_time default=\"5000\"/>"
			"		<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"all\">"
			"			<all abbreviation=\"a\"/>"
			"			<none abbreviation=\"n\"/>"
			"			<motion_id abbreviation=\"m\" default=\"0\"/>"
			"			<physical_id abbreviation=\"p\" default=\"0\"/>"
			"			<slave_id abbreviation=\"s\" default=\"0\"/>"
			"		</unique>"
			"	</group>"
			"</hm>");
	}
	HomePlan::HomePlan(const HomePlan &) = default;
	HomePlan::HomePlan(HomePlan &&) = default;
	HomePlan& HomePlan::operator=(const HomePlan &) = default;
	HomePlan& HomePlan::operator=(HomePlan &&) = default;

	struct ModeParam
	{
		std::int32_t limit_time;
		std::int32_t mode;
		std::vector<int> active_motor;
	};
	struct ModePlan::Imp { Imp() {} };
	auto ModePlan::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		ModeParam param;

		for (auto cmd_param : params)
		{
			if (cmd_param.first == "limit_time")
			{
				param.limit_time = std::stoi(cmd_param.second);
			}
			else if (cmd_param.first == "mode")
			{
				param.mode = std::stoi(cmd_param.second);
			}
			else if (cmd_param.first == "all")
			{
				param.active_motor.clear();
				param.active_motor.resize(dynamic_cast<aris::control::Controller *>(target.master)->motionPool().size(), 1);
			}
			else if (cmd_param.first == "none")
			{
				param.active_motor.clear();
				param.active_motor.resize(dynamic_cast<aris::control::Controller *>(target.master)->motionPool().size(), 0);
			}
			else if (cmd_param.first == "motion_id")
			{
				param.active_motor.clear();
				param.active_motor.resize(dynamic_cast<aris::control::Controller *>(target.master)->motionPool().size(), 0);
				param.active_motor.at(std::stoi(cmd_param.second)) = 1;
			}
			else if (cmd_param.first == "physical_id")
			{
				param.active_motor.clear();
				param.active_motor.resize(dynamic_cast<aris::control::Controller *>(target.master)->motionPool().size(), 0);
				param.active_motor.at(dynamic_cast<aris::control::Controller*>(target.master)->motionAtPhy(std::stoi(cmd_param.second)).phyId()) = 1;
			}
			else if (cmd_param.first == "slave_id")
			{
				param.active_motor.clear();
				param.active_motor.resize(dynamic_cast<aris::control::Controller *>(target.master)->motionPool().size(), 0);
				param.active_motor.at(dynamic_cast<aris::control::Controller*>(target.master)->motionAtPhy(std::stoi(cmd_param.second)).slaId()) = 1;
			}
			else if (cmd_param.first == "check_all")
			{
				target.option &= ~(
					NOT_CHECK_POS_MIN |
					NOT_CHECK_POS_MAX |
					NOT_CHECK_POS_CONTINUOUS |
					NOT_CHECK_POS_CONTINUOUS_AT_START |
					NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER |
					NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER_AT_START |
					NOT_CHECK_POS_FOLLOWING_ERROR |
					NOT_CHECK_VEL_MIN |
					NOT_CHECK_VEL_MAX |
					NOT_CHECK_VEL_CONTINUOUS |
					NOT_CHECK_VEL_CONTINUOUS_AT_START |
					NOT_CHECK_VEL_FOLLOWING_ERROR);
			}
			else if (cmd_param.first == "check_none")
			{
				target.option |=
					NOT_CHECK_POS_MIN |
					NOT_CHECK_POS_MAX |
					NOT_CHECK_POS_CONTINUOUS |
					NOT_CHECK_POS_CONTINUOUS_AT_START |
					NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER |
					NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER_AT_START |
					NOT_CHECK_POS_FOLLOWING_ERROR |
					NOT_CHECK_VEL_MIN |
					NOT_CHECK_VEL_MAX |
					NOT_CHECK_VEL_CONTINUOUS |
					NOT_CHECK_VEL_CONTINUOUS_AT_START |
					NOT_CHECK_VEL_FOLLOWING_ERROR;
			}
			else
				throw std::runtime_error("unknown input target when prepair ModePlan");
		}

		target.param = param;
	}
	auto ModePlan::executeRT(PlanTarget &target)->int
	{
		auto controller = dynamic_cast<aris::control::Controller *>(target.master);
		auto &param = std::any_cast<ModeParam &>(target.param);

		bool is_all_finished = true;
		for (std::size_t i = 0; i < controller->motionPool().size(); ++i)
		{
			auto &cm = controller->motionPool().at(i);

			if (param.active_motor[i])
			{
				auto ret = cm.mode(8);
				cm.setTargetPos(cm.actualPos());
				if (ret)
				{
					is_all_finished = false;

					if (target.count % 1000 == 0)
					{
						controller->mout() << "Unmoded motor, slave id: " << cm.id() << ", absolute id: " << i << ", ret: " << ret << std::endl;
					}
				}
			}
		}

		return (is_all_finished || target.count >= param.limit_time) ? 0 : 1;
	}
	auto ModePlan::collectNrt(PlanTarget &param)->void {}
	ModePlan::~ModePlan() = default;
	ModePlan::ModePlan(const std::string &name) :Plan(name), imp_(new Imp)
	{
		command().loadXmlStr(
			"<md default_child_type=\"Param\">"
			"	<group type=\"GroupParam\" default_child_type=\"Param\">"
			"		<limit_time default=\"5000\"/>"
			"       <mode abbreviation=\"d\" default=\"8\"/>"
			"		<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"all\">"
			"			<all abbreviation=\"a\"/>"
			"			<none abbreviation=\"n\"/>"
			"			<motion_id abbreviation=\"m\" default=\"0\"/>"
			"			<physical_id abbreviation=\"p\" default=\"0\"/>"
			"			<slave_id abbreviation=\"s\" default=\"0\"/>"
			"		</unique>"
			"		<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_none\">"
			"			<check_all/>"
			"			<check_none/>"
			"			<group type=\"GroupParam\" default_child_type=\"Param\">"
			"				<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_pos\">"
			"					<check_pos/>"
			"					<not_check_pos/>"
			"					<group type=\"GroupParam\" default_child_type=\"Param\">"
			"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_pos_max\">"
			"							<check_pos_max/>"
			"							<not_check_pos_max/>"
			"						</unique>"
			"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_pos_min\">"
			"							<check_pos_min/>"
			"							<not_check_pos_min/>"
			"						</unique>"
			"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_pos_continuous\">"
			"							<check_pos_continuous/>"
			"							<not_check_pos_continuous/>"
			"						</unique>"
			"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_pos_continuous_at_start\">"
			"							<check_pos_continuous_at_start/>"
			"							<not_check_pos_continuous_at_start/>"
			"						</unique>"
			"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_pos_continuous_second_order\">"
			"							<check_pos_continuous_second_order/>"
			"							<not_check_pos_continuous_second_order/>"
			"						</unique>"
			"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_pos_continuous_second_order_at_start\">"
			"							<check_pos_continuous_second_order_at_start/>"
			"							<not_check_pos_continuous_second_order_at_start/>"
			"						</unique>"
			"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_pos_following_error\">"
			"							<check_pos_following_error/>"
			"							<not_check_pos_following_error />"
			"						</unique>"
			"					</group>"
			"				</unique>"
			"				<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_vel\">"
			"					<check_vel/>"
			"					<not_check_vel/>"
			"					<group type=\"GroupParam\" default_child_type=\"Param\">"
			"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_vel_max\">"
			"							<check_vel_max/>"
			"							<not_check_vel_max/>"
			"						</unique>"
			"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_vel_min\">"
			"							<check_vel_min/>"
			"							<not_check_vel_min/>"
			"						</unique>"
			"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_vel_continuous\">"
			"							<check_vel_continuous/>"
			"							<not_check_vel_continuous/>"
			"						</unique>"
			"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_vel_continuous_at_start\">"
			"							<check_vel_continuous_at_start/>"
			"							<not_check_vel_continuous_at_start/>"
			"						</unique>"
			"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_vel_following_error\">"
			"							<check_vel_following_error/>"
			"							<not_check_vel_following_error />"
			"						</unique>"
			"					</group>"
			"				</unique>"
			"			</group>"
			"		</unique>"
			"	</group>"
			"</md>");
	}
	ModePlan::ModePlan(const ModePlan &) = default;
	ModePlan::ModePlan(ModePlan &&) = default;
	ModePlan& ModePlan::operator=(const ModePlan &) = default;
	ModePlan& ModePlan::operator=(ModePlan &&) = default;

	struct ResetParam
	{
		std::vector<Size> total_count_vec;
		std::vector<double> axis_begin_pos_vec;
		std::vector<double> axis_pos_vec;
		std::vector<double> axis_vel_vec;
		std::vector<double> axis_acc_vec;
		std::vector<double> axis_dec_vec;
	};
	struct ResetPlan::Imp { Imp() {} };
	auto ResetPlan::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		default_prepair_check_option(params, target);

		auto c = dynamic_cast<aris::control::Controller*>(target.master);
		ResetParam param;
		param.axis_begin_pos_vec.resize(c->motionPool().size(), 0.0);
		param.total_count_vec.resize(c->motionPool().size(), 1);
		for (auto cmd_param : params)
		{
			if (cmd_param.first == "pos")
			{
				auto p = target.model->calculator().calculateExpression(cmd_param.second);
				if (p.size() == 1)
				{
					param.axis_pos_vec.resize(c->motionPool().size(), p.toDouble());
				}
				else if (p.size() == c->motionPool().size())
				{
					param.axis_pos_vec.assign(p.begin(), p.end());
				}
				else
				{
					throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + " failed");
				}

				for (Size i = 0; i < c->motionPool().size(); ++i)
				{
					if(param.axis_pos_vec[i] > 1.0 || param.axis_pos_vec[i]<0.0)
						throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + " failed");

					param.axis_pos_vec[i] = param.axis_pos_vec[i] * (c->motionPool()[i].maxPos() - c->motionPool()[i].minPos()) + c->motionPool()[i].minPos();
					param.axis_pos_vec[i] = std::min(std::max(param.axis_pos_vec[i], c->motionPool()[i].minPos()), c->motionPool()[i].maxPos());// 防止截断误差
				}

			}
			else if (cmd_param.first == "acc")
			{
				auto a = target.model->calculator().calculateExpression(cmd_param.second);

				if (a.size() == 1)
				{
					param.axis_acc_vec.resize(c->motionPool().size(), a.toDouble());
				}
				else if (a.size() == c->motionPool().size())
				{
					param.axis_acc_vec.assign(a.begin(), a.end());
				}
				else
				{
					throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + " failed");
				}

				for (Size i = 0; i < c->motionPool().size(); ++i)
				{
					if (param.axis_acc_vec[i] > 1.0 || param.axis_acc_vec[i] < 0.01)
						throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + " failed");

					param.axis_acc_vec[i] = param.axis_acc_vec[i] * c->motionPool()[i].maxAcc();
				}
			}
			else if (cmd_param.first == "vel")
			{
				auto v = target.model->calculator().calculateExpression(cmd_param.second);

				if (v.size() == 1)
				{
					param.axis_vel_vec.resize(c->motionPool().size(), v.toDouble());
				}
				else if (v.size() == c->motionPool().size())
				{
					param.axis_vel_vec.assign(v.begin(), v.end());
				}
				else
				{
					throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + " failed");
				}

				for (Size i = 0; i < c->motionPool().size(); ++i)
				{
					if (param.axis_vel_vec[i] > 1.0 || param.axis_vel_vec[i] < 0.01)
						throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + " failed");

					param.axis_vel_vec[i] = param.axis_vel_vec[i] * c->motionPool()[i].maxVel();
				}
			}
			else if (cmd_param.first == "dec")
			{
				auto d = target.model->calculator().calculateExpression(cmd_param.second);

				if (d.size() == 1)
				{
					param.axis_dec_vec.resize(c->motionPool().size(), d.toDouble());
				}
				else if (d.size() == c->motionPool().size())
				{
					param.axis_dec_vec.assign(d.begin(), d.end());
				}
				else
				{
					throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + " failed");
				}

				for (Size i = 0; i < c->motionPool().size(); ++i)
				{
					if (param.axis_dec_vec[i] > 1.0 || param.axis_dec_vec[i] < 0.01)
						throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + " failed");

					param.axis_dec_vec[i] = param.axis_dec_vec[i] * c->motionPool()[i].minAcc();
				}
			}
		}

		target.option |=
			aris::plan::Plan::USE_TARGET_POS | 
			aris::plan::Plan::EXECUTE_WHEN_ALL_PLAN_COLLECTED |
			aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_AT_START | 
			aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER_AT_START;

		target.param = param;
	}
	auto ResetPlan::executeRT(PlanTarget &target)->int
	{
		auto &param = std::any_cast<ResetParam&>(target.param);
		auto controller = dynamic_cast<aris::control::Controller *>(target.master);

		// 取得起始位置 //
		if (target.count == 1)
		{
			for (Size i = 0; i < controller->motionPool().size(); ++i)
			{
				param.axis_begin_pos_vec[i] = controller->motionPool().at(i).actualPos();
			}
		}

		// 设置驱动器的位置 //
		for (Size i = 0; i < controller->motionPool().size(); ++i)
		{
			double p, v, a;
			aris::plan::moveAbsolute(target.count, param.axis_begin_pos_vec[i], param.axis_pos_vec[i], param.axis_vel_vec[i] / 1000
				, param.axis_acc_vec[i] / 1000 / 1000, param.axis_dec_vec[i] / 1000 / 1000, p, v, a, param.total_count_vec[i]);
			controller->motionAtAbs(i).setTargetPos(p);
		}

		// 改变模型中的驱动位置 //
		for (Size i = 0; i < std::min(controller->motionPool().size(), target.model->motionPool().size()); ++i)
		{
			target.model->motionPool()[i].setMp(controller->motionPool().at(i).targetPos());
		}

		target.model->solverPool().at(1).kinPos();
		return (static_cast<int>(*std::max_element(param.total_count_vec.begin(), param.total_count_vec.end())) > target.count) ? 1 : 0;
	}
	auto ResetPlan::collectNrt(PlanTarget &target)->void {}
	ResetPlan::~ResetPlan() = default;
	ResetPlan::ResetPlan(const std::string &name) :Plan(name), imp_(new Imp)
	{
		command().loadXmlStr(
			"<rs default_child_type=\"Param\">"
			"	<group type=\"GroupParam\" default_child_type=\"Param\">"
			"		<pos default=\"0.5\" abbreviation=\"p\"/>"
			"		<acc default=\"0.3\" abbreviation=\"a\"/>"
			"		<vel default=\"0.3\" abbreviation=\"v\"/>"
			"		<dec default=\"0.3\" abbreviation=\"d\"/>"
			"		<unique_check type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_all\">"
			"			<check_all/>"
			"			<check_none/>"
			"			<group type=\"GroupParam\" default_child_type=\"Param\">"
			"				<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_pos\">"
			"					<check_pos/>"
			"					<not_check_pos/>"
			"					<group type=\"GroupParam\" default_child_type=\"Param\">"
			"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_pos_max\">"
			"							<check_pos_max/>"
			"							<not_check_pos_max/>"
			"						</unique>"
			"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_pos_min\">"
			"							<check_pos_min/>"
			"							<not_check_pos_min/>"
			"						</unique>"
			"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_pos_continuous\">"
			"							<check_pos_continuous/>"
			"							<not_check_pos_continuous/>"
			"						</unique>"
			"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_pos_continuous_at_start\">"
			"							<check_pos_continuous_at_start/>"
			"							<not_check_pos_continuous_at_start/>"
			"						</unique>"
			"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_pos_continuous_second_order\">"
			"							<check_pos_continuous_second_order/>"
			"							<not_check_pos_continuous_second_order/>"
			"						</unique>"
			"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_pos_continuous_second_order_at_start\">"
			"							<check_pos_continuous_second_order_at_start/>"
			"							<not_check_pos_continuous_second_order_at_start/>"
			"						</unique>"
			"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_pos_following_error\">"
			"							<check_pos_following_error/>"
			"							<not_check_pos_following_error />"
			"						</unique>"
			"					</group>"
			"				</unique>"
			"				<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_vel\">"
			"					<check_vel/>"
			"					<not_check_vel/>"
			"					<group type=\"GroupParam\" default_child_type=\"Param\">"
			"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_vel_max\">"
			"							<check_vel_max/>"
			"							<not_check_vel_max/>"
			"						</unique>"
			"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_vel_min\">"
			"							<check_vel_min/>"
			"							<not_check_vel_min/>"
			"						</unique>"
			"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_vel_continuous\">"
			"							<check_vel_continuous/>"
			"							<not_check_vel_continuous/>"
			"						</unique>"
			"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_vel_continuous_at_start\">"
			"							<check_vel_continuous_at_start/>"
			"							<not_check_vel_continuous_at_start/>"
			"						</unique>"
			"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_vel_following_error\">"
			"							<check_vel_following_error/>"
			"							<not_check_vel_following_error />"
			"						</unique>"
			"					</group>"
			"				</unique>"
			"			</group>"
			"		</unique_check>"
			"	</group>"
			"</rs>");
	}
	ResetPlan::ResetPlan(const ResetPlan &) = default;
	ResetPlan::ResetPlan(ResetPlan &&) = default;
	ResetPlan& ResetPlan::operator=(const ResetPlan &) = default;
	ResetPlan& ResetPlan::operator=(ResetPlan &&) = default;

	struct RecoverParam
	{
		std::atomic_bool is_kinematic_ready_;
		std::atomic_bool is_rt_waiting_ready_;
		std::future<void> fut;
		int kin_ret;
	};
	auto RecoverPlan::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		target.option |= NOT_CHECK_POS_CONTINUOUS_AT_START;
		target.option |= NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER_AT_START;

		auto p = std::make_shared<RecoverParam>();
		
		p->is_kinematic_ready_ = false;
		p->is_rt_waiting_ready_ = false;
		p->fut = std::async(std::launch::async, [](std::shared_ptr<RecoverParam> p, PlanTarget *target) 
		{
			// 等待正解求解的需求 //
			while (!p->is_rt_waiting_ready_.load())std::this_thread::sleep_for(std::chrono::milliseconds(1));

			// 求正解 //
			p->kin_ret = target->model->solverPool()[1].kinPos() ? 0 : -1;

			// 通知实时线程 //
			p->is_kinematic_ready_.store(true);
		}, p, &target);

		target.param = p;
	}
	auto RecoverPlan::executeRT(PlanTarget &target)->int
	{
		auto controller = dynamic_cast<aris::control::Controller *>(target.master);
		auto param = std::any_cast<std::shared_ptr<RecoverParam> &>(target.param);

		if (target.count == 1)
		{
			for (Size i = 0; i < std::min(controller->motionPool().size(), target.model->motionPool().size()); ++i)
			{
				controller->motionPool()[i].setTargetPos(controller->motionPool().at(i).actualPos());
				target.model->motionPool()[i].setMp(controller->motionPool().at(i).actualPos());
			}

			param->is_rt_waiting_ready_.store(true);

			return 0;
		}

		return param->is_kinematic_ready_.load() ? param->kin_ret : 1;
	}
	auto RecoverPlan::collectNrt(PlanTarget &target)->void { std::any_cast<std::shared_ptr<RecoverParam>&>(target.param)->fut.get(); }
	RecoverPlan::~RecoverPlan() = default;
	RecoverPlan::RecoverPlan(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<rc default_child_type=\"Param\">"
			"</rc>");
	}
	RecoverPlan::RecoverPlan(const RecoverPlan &) = default;
	RecoverPlan::RecoverPlan(RecoverPlan &&) = default;
	RecoverPlan& RecoverPlan::operator=(const RecoverPlan &) = default;
	RecoverPlan& RecoverPlan::operator=(RecoverPlan &&) = default;

	struct SleepParam { int count; };
	struct SleepPlan::Imp {};
	auto SleepPlan::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		target.param = SleepParam{ std::stoi(params.at("count")) };
	}
	auto SleepPlan::executeRT(PlanTarget &target)->int { return std::any_cast<SleepParam&>(target.param).count - target.count; }
	auto SleepPlan::collectNrt(PlanTarget &target)->void {}
	SleepPlan::~SleepPlan() = default;
	SleepPlan::SleepPlan(const std::string &name) :Plan(name), imp_(new Imp)
	{
		command().loadXmlStr(
			"<sl default_child_type=\"Param\">"
			"	<count default=\"1000\" abbreviation=\"c\"/>"
			"</sl>");
	}
	SleepPlan::SleepPlan(const SleepPlan &) = default;
	SleepPlan::SleepPlan(SleepPlan &&) = default;
	SleepPlan& SleepPlan::operator=(const SleepPlan &) = default;
	SleepPlan& SleepPlan::operator=(SleepPlan &&) = default;

	struct MovePlan::Imp
	{
		struct MvParam { double pq[7]; double acceleration, velocity, deceleration, angular_acceleration, angular_velocity, angular_deceleration; };
	};
	auto MovePlan::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		default_prepair_check_option(params, target);

		Imp::MvParam mv_param;

		double pe[6], pe_unit;
		std::string eul_type;
		bool use_pe{ false };

		for (auto cmd_param : params)
		{
			if (cmd_param.first == "pq")
			{
				auto pq = target.model->calculator().calculateExpression(cmd_param.second);
				if (pq.size() != 7)throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + " failed");
				aris::dynamic::s_vc(7, pq.data(), mv_param.pq);
			}
			else if (cmd_param.first == "pm")
			{
				auto pm = target.model->calculator().calculateExpression(cmd_param.second);
				if (pm.size() != 16)throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + " failed");
				aris::dynamic::s_pm2pq(pm.data(), mv_param.pq);
			}
			else if (cmd_param.first == "pe")
			{
				auto pe_mat = target.model->calculator().calculateExpression(cmd_param.second);
				if (pe_mat.size() != 6)throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + " failed");
				std::copy(pe_mat.data(), pe_mat.data() + 6, pe);
				use_pe = true;
			}
			else if (cmd_param.first == "orientation_unit")
			{
				if (cmd_param.second == "rad")pe_unit = 1.0;
				else if (cmd_param.second == "degree")pe_unit = PI / 180.0;
				else throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + " failed");
			}
			else if (cmd_param.first == "euler_type")
			{
				eul_type = cmd_param.second;
			}
			else if (cmd_param.first == "acceleration")
			{
				mv_param.acceleration = std::atof(cmd_param.second.c_str());
			}
			else if (cmd_param.first == "velocity")
			{
				mv_param.velocity = std::atof(cmd_param.second.c_str());
			}
			else if (cmd_param.first == "deceleration")
			{
				mv_param.deceleration = std::atof(cmd_param.second.c_str());
			}
			else if (cmd_param.first == "angular_acceleration")
			{
				mv_param.angular_acceleration = std::atof(cmd_param.second.c_str());
			}
			else if (cmd_param.first == "angular_velocity")
			{
				mv_param.angular_velocity = std::atof(cmd_param.second.c_str());
			}
			else if (cmd_param.first == "angular_deceleration")
			{
				mv_param.angular_deceleration = std::atof(cmd_param.second.c_str());
			}
		}

		if (use_pe)
		{
			aris::dynamic::s_nv(3, pe_unit, pe + 3);
			aris::dynamic::s_pe2pq(pe, mv_param.pq, eul_type.data());
		}

		target.option |= USE_TARGET_POS;
		target.param = mv_param;
	}
	auto MovePlan::executeRT(PlanTarget &target)->int
	{
		auto mv_param = std::any_cast<Imp::MvParam>(&target.param);
		auto controller = dynamic_cast<aris::control::Controller *>(target.master);

		// 取得起始位置 //
		static double begin_pm[16], relative_pm[16], relative_pa[6], pos_ratio, ori_ratio, norm_pos, norm_ori;
		double p, v, a;
		aris::Size pos_total_count, ori_total_count;
		if (target.count == 1)
		{
			double end_pm[16];
			aris::dynamic::s_pq2pm(mv_param->pq, end_pm);
			target.model->generalMotionPool().at(0).updMpm();
			target.model->generalMotionPool().at(0).getMpm(begin_pm);
			aris::dynamic::s_inv_pm_dot_pm(begin_pm, end_pm, relative_pm);

			// relative_pa //
			aris::dynamic::s_pm2pa(relative_pm, relative_pa);

			norm_pos = aris::dynamic::s_norm(3, relative_pa);
			norm_ori = aris::dynamic::s_norm(3, relative_pa + 3);

			aris::plan::moveAbsolute(target.count, 0.0, norm_pos, mv_param->velocity / 1000, mv_param->acceleration / 1000 / 1000, mv_param->deceleration / 1000 / 1000, p, v, a, pos_total_count);
			aris::plan::moveAbsolute(target.count, 0.0, norm_ori, mv_param->angular_velocity / 1000, mv_param->angular_acceleration / 1000 / 1000, mv_param->angular_deceleration / 1000 / 1000, p, v, a, ori_total_count);

			pos_ratio = pos_total_count < ori_total_count ? double(pos_total_count) / ori_total_count : 1.0;
			ori_ratio = ori_total_count < pos_total_count ? double(ori_total_count) / pos_total_count : 1.0;

			aris::plan::moveAbsolute(target.count, 0.0, norm_pos, mv_param->velocity / 1000 * pos_ratio, mv_param->acceleration / 1000 / 1000 * pos_ratio* pos_ratio, mv_param->deceleration / 1000 / 1000 * pos_ratio* pos_ratio, p, v, a, pos_total_count);
			aris::plan::moveAbsolute(target.count, 0.0, norm_ori, mv_param->angular_velocity / 1000 * ori_ratio, mv_param->angular_acceleration / 1000 / 1000 * ori_ratio * ori_ratio, mv_param->angular_deceleration / 1000 / 1000 * ori_ratio * ori_ratio, p, v, a, ori_total_count);
		}

		double pa[6]{ 0,0,0,0,0,0 }, pm[16], pm2[16];

		aris::plan::moveAbsolute(target.count, 0.0, norm_pos, mv_param->velocity / 1000 * pos_ratio, mv_param->acceleration / 1000 / 1000 * pos_ratio* pos_ratio, mv_param->deceleration / 1000 / 1000 * pos_ratio* pos_ratio, p, v, a, pos_total_count);
		if (norm_pos > 1e-10)aris::dynamic::s_vc(3, p / norm_pos, relative_pa, pa);

		aris::plan::moveAbsolute(target.count, 0.0, norm_ori, mv_param->angular_velocity / 1000 * ori_ratio, mv_param->angular_acceleration / 1000 / 1000 * ori_ratio * ori_ratio, mv_param->angular_deceleration / 1000 / 1000 * ori_ratio * ori_ratio, p, v, a, ori_total_count);
		if (norm_ori > 1e-10)aris::dynamic::s_vc(3, p / norm_ori, relative_pa + 3, pa + 3);

		aris::dynamic::s_pa2pm(pa, pm);
		aris::dynamic::s_pm_dot_pm(begin_pm, pm, pm2);

		// 反解计算电机位置 //
		target.model->generalMotionPool().at(0).setMpm(pm2);
		if (!target.model->solverPool().at(0).kinPos())return -1;


		double pq[7];
		aris::dynamic::s_pm2pq(*target.model->generalMotionPool().at(0).mpm(), pq);
		target.master->lout() << target.count << " " << pq[0] << " " << pq[1] << " " << pq[2] << " " << pq[3] << " " << pq[4] << " " << pq[5] << " " << pq[6]<<"  ";

		for (auto &cm : controller->motionPool())
		{
			target.master->lout() << "  " << cm.targetPos() << "  " << cm.actualPos() << "  " << cm.actualVel() << "  " << cm.actualCur() << "  ";
		}
		target.master->lout() << "\n";


		return std::max(pos_total_count, ori_total_count) > target.count ? 1 : 0;
	}
	auto MovePlan::collectNrt(PlanTarget &param)->void {}
	MovePlan::~MovePlan() = default;
	MovePlan::MovePlan(const std::string &name) :Plan(name), imp_(new Imp)
	{
		command().loadXmlStr(
			"<mv default_child_type=\"Param\">"
			"	<group type=\"GroupParam\" default_child_type=\"Param\">"
			"		<position_unit default=\"m\"/>"
			"		<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"pq\">"
			"			<pq default=\"{0,0.63,0.316,0,0,0,1}\"/>"
			"			<pm default=\"{1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1}\"/>"
			"			<group type=\"GroupParam\" default_child_type=\"Param\">"
			"				<pe default=\"{0,0.63,0.316,0,0,0}\"/>"
			"				<orientation_unit default=\"rad\"/>"
			"				<euler_type default=\"321\"/>"
			"			</group>"
			"		</unique>"
			"		<acceleration default=\"0.2\"/>"
			"		<velocity default=\"0.2\"/>"
			"		<deceleration default=\"0.2\"/>"
			"		<angular_acceleration default=\"0.2\"/>"
			"		<angular_velocity default=\"0.2\"/>"
			"		<angular_deceleration default=\"0.2\"/>"
			"		<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_all\">"
			"			<check_all/>"
			"			<check_none/>"
			"			<group type=\"GroupParam\" default_child_type=\"Param\">"
			"				<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_pos\">"
			"					<check_pos/>"
			"					<not_check_pos/>"
			"					<group type=\"GroupParam\" default_child_type=\"Param\">"
			"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_pos_max\">"
			"							<check_pos_max/>"
			"							<not_check_pos_max/>"
			"						</unique>"
			"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_pos_min\">"
			"							<check_pos_min/>"
			"							<not_check_pos_min/>"
			"						</unique>"
			"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_pos_continuous\">"
			"							<check_pos_continuous/>"
			"							<not_check_pos_continuous/>"
			"						</unique>"
			"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_pos_continuous_at_start\">"
			"							<check_pos_continuous_at_start/>"
			"							<not_check_pos_continuous_at_start/>"
			"						</unique>"
			"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_pos_continuous_second_order\">"
			"							<check_pos_continuous_second_order/>"
			"							<not_check_pos_continuous_second_order/>"
			"						</unique>"
			"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_pos_continuous_second_order_at_start\">"
			"							<check_pos_continuous_second_order_at_start/>"
			"							<not_check_pos_continuous_second_order_at_start/>"
			"						</unique>"
			"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_pos_following_error\">"
			"							<check_pos_following_error/>"
			"							<not_check_pos_following_error />"
			"						</unique>"
			"					</group>"
			"				</unique>"
			"				<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_vel\">"
			"					<check_vel/>"
			"					<not_check_vel/>"
			"					<group type=\"GroupParam\" default_child_type=\"Param\">"
			"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_vel_max\">"
			"							<check_vel_max/>"
			"							<not_check_vel_max/>"
			"						</unique>"
			"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_vel_min\">"
			"							<check_vel_min/>"
			"							<not_check_vel_min/>"
			"						</unique>"
			"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_vel_continuous\">"
			"							<check_vel_continuous/>"
			"							<not_check_vel_continuous/>"
			"						</unique>"
			"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_vel_continuous_at_start\">"
			"							<check_vel_continuous_at_start/>"
			"							<not_check_vel_continuous_at_start/>"
			"						</unique>"
			"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_vel_following_error\">"
			"							<check_vel_following_error/>"
			"							<not_check_vel_following_error />"
			"						</unique>"
			"					</group>"
			"				</unique>"
			"			</group>"
			"		</unique>"
			"	</group>"
			"</mv>");
	}
	MovePlan::MovePlan(const MovePlan &) = default;
	MovePlan::MovePlan(MovePlan &&) = default;
	MovePlan& MovePlan::operator=(const MovePlan &) = default;
	MovePlan& MovePlan::operator=(MovePlan &&) = default;

	struct UniversalPlan::Imp
	{
		PrepairFunc prepair_nrt;
		ExecuteFunc execute_rt;
		CollectFunc collect_nrt;
	};
	auto UniversalPlan::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void { if (imp_->prepair_nrt)imp_->prepair_nrt(params, target); }
	auto UniversalPlan::executeRT(PlanTarget &param)->int { return imp_->execute_rt ? imp_->execute_rt(param) : 0; }
	auto UniversalPlan::collectNrt(PlanTarget &param)->void { if (imp_->collect_nrt)imp_->collect_nrt(param); }
	auto UniversalPlan::setPrepairFunc(PrepairFunc func)->void { imp_->prepair_nrt = func; }
	auto UniversalPlan::setExecuteFunc(ExecuteFunc func)->void { imp_->execute_rt = func; }
	auto UniversalPlan::setCollectFunc(CollectFunc func)->void { imp_->collect_nrt = func; }
	UniversalPlan::~UniversalPlan() = default;
	UniversalPlan::UniversalPlan(const std::string &name, PrepairFunc prepair_func, ExecuteFunc execute_func, CollectFunc collect_func, const std::string & cmd_xml_str) :Plan(name), imp_(new Imp)
	{
		imp_->prepair_nrt = prepair_func;
		imp_->execute_rt = execute_func;
		imp_->collect_nrt = collect_func;
		command().loadXmlStr(cmd_xml_str);
	}
	UniversalPlan::UniversalPlan(const UniversalPlan &) = default;
	UniversalPlan::UniversalPlan(UniversalPlan &&) = default;
	UniversalPlan& UniversalPlan::operator=(const UniversalPlan &) = default;
	UniversalPlan& UniversalPlan::operator=(UniversalPlan &&) = default;

	auto Show::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		target.option |= 
			NOT_CHECK_POS_MIN |
			NOT_CHECK_POS_MAX |
			NOT_CHECK_POS_CONTINUOUS |
			NOT_CHECK_POS_CONTINUOUS_AT_START |
			NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER |
			NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER_AT_START |
			NOT_CHECK_POS_FOLLOWING_ERROR |
			NOT_CHECK_VEL_MIN |
			NOT_CHECK_VEL_MAX |
			NOT_CHECK_VEL_CONTINUOUS |
			NOT_CHECK_VEL_CONTINUOUS_AT_START |
			NOT_CHECK_VEL_FOLLOWING_ERROR;
	}
	auto Show::executeRT(PlanTarget &target)->int
	{
		auto controller = dynamic_cast<aris::control::Controller *>(target.master);

		controller->mout() << "pos: ";
		for (auto &m : controller->motionPool())
		{
			controller->mout() << std::setprecision(15) << m.actualPos() << "   ";
		}
		controller->mout() << std::endl;

		return 0;
	}
	auto Show::collectNrt(PlanTarget &param)->void { }
	Show::Show(const std::string &name) : Plan(name)
	{
		command().loadXmlStr(
			"<show default_child_type=\"Param\">"
			"</show>");
	}

	auto MoveJ::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void 
	{
		default_prepair_check_option(params, target);
		
		auto c = dynamic_cast<aris::control::Controller*>(target.master);
		MoveJ::Param param;

		for (auto cmd_param : params)
		{
			if (cmd_param.first == "all")
			{
				param.joint_active_vec.resize(c->motionPool().size(), true);
			}
			else if (cmd_param.first == "none")
			{
				param.joint_active_vec.resize(c->motionPool().size(), false);
			}
			else if (cmd_param.first == "motion_id")
			{
				param.joint_active_vec.resize(c->motionPool().size(), false);
				param.joint_active_vec.at(std::stoi(cmd_param.second)) = true;
			}
			else if (cmd_param.first == "physical_id")
			{
				param.joint_active_vec.resize(c->motionPool().size(), false);
				param.joint_active_vec.at(c->motionAtPhy(std::stoi(cmd_param.second)).phyId()) = true;
			}
			else if (cmd_param.first == "slave_id")
			{
				param.joint_active_vec.resize(c->motionPool().size(), false);
				param.joint_active_vec.at(c->motionAtPhy(std::stoi(cmd_param.second)).slaId()) = true;
			}
			else if (cmd_param.first == "pos")
			{
				aris::core::Calculator cal;
				auto mat = cal.calculateExpression(cmd_param.second);
				if(mat.size()==1)param.joint_pos_vec.resize(c->motionPool().size(), mat.toDouble());
				else 
				{
					param.joint_pos_vec.resize(mat.size());
					std::copy(mat.begin(), mat.end(), param.joint_pos_vec.begin());
				}
			}
			else if (cmd_param.first == "vel")
			{
				param.vel = std::stod(cmd_param.second);
			}
			else if (cmd_param.first == "acc")
			{
				param.acc = std::stod(cmd_param.second);
			}
			else if (cmd_param.first == "dec")
			{
				param.dec = std::stod(cmd_param.second);
			}
		}

		param.begin_joint_pos_vec.resize(c->motionPool().size());
		target.param = param;
	}
	auto MoveJ::executeRT(PlanTarget &target)->int 
	{ 
		auto param = std::any_cast<Param>(&target.param);
		auto controller = dynamic_cast<aris::control::Controller *>(target.master);

		if (target.count == 1)
		{
			for (Size i = 0; i < param->joint_active_vec.size(); ++i)
			{
				if (param->joint_active_vec[i])
				{
					param->begin_joint_pos_vec[i] = controller->motionPool()[i].targetPos();
				}
			}
		}

		aris::Size total_count{ 1 };
		for (Size i = 0; i < param->joint_active_vec.size(); ++i)
		{
			if (param->joint_active_vec[i])
			{
				double p, v, a;
				aris::Size t_count;
				aris::plan::moveAbsolute(target.count, param->begin_joint_pos_vec[i], param->joint_pos_vec[i], param->vel / 1000, param->acc / 1000 / 1000, param->dec / 1000 / 1000, p, v, a, t_count);
				controller->motionPool()[i].setTargetPos(p);
				total_count = std::max(total_count, t_count);
			}
		}

		return total_count - target.count;
	}
	auto MoveJ::collectNrt(PlanTarget &param)->void { }
	struct MoveJ::Imp {};
	MoveJ::~MoveJ() = default;
	MoveJ::MoveJ(const std::string &name): Plan(name), imp_(new Imp)
	{
		command().loadXmlStr(
			"<moveJ default_child_type=\"Param\">"
			"	<group type=\"GroupParam\" default_child_type=\"Param\">"
			"		<limit_time default=\"5000\"/>"
			"		<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"all\">"
			"			<all abbreviation=\"a\"/>"
			"			<motion_id abbreviation=\"m\" default=\"0\"/>"
			"			<physical_id abbreviation=\"p\" default=\"0\"/>"
			"			<slave_id abbreviation=\"s\" default=\"0\"/>"
			"		</unique>"
			"		<pos default=\"0\"/>"
			"		<vel default=\"0.5\"/>"
			"		<acc default=\"1\"/>"
			"		<dec default=\"1\"/>"
			"		<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_all\">"
			"			<check_all/>"
			"			<check_none/>"
			"			<group type=\"GroupParam\" default_child_type=\"Param\">"
			"				<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_pos\">"
			"					<check_pos/>"
			"					<not_check_pos/>"
			"					<group type=\"GroupParam\" default_child_type=\"Param\">"
			"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_pos_max\">"
			"							<check_pos_max/>"
			"							<not_check_pos_max/>"
			"						</unique>"
			"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_pos_min\">"
			"							<check_pos_min/>"
			"							<not_check_pos_min/>"
			"						</unique>"
			"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_pos_continuous\">"
			"							<check_pos_continuous/>"
			"							<not_check_pos_continuous/>"
			"						</unique>"
			"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_pos_continuous_at_start\">"
			"							<check_pos_continuous_at_start/>"
			"							<not_check_pos_continuous_at_start/>"
			"						</unique>"
			"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_pos_continuous_second_order\">"
			"							<check_pos_continuous_second_order/>"
			"							<not_check_pos_continuous_second_order/>"
			"						</unique>"
			"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_pos_continuous_second_order_at_start\">"
			"							<check_pos_continuous_second_order_at_start/>"
			"							<not_check_pos_continuous_second_order_at_start/>"
			"						</unique>"
			"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_pos_following_error\">"
			"							<check_pos_following_error/>"
			"							<not_check_pos_following_error />"
			"						</unique>"
			"					</group>"
			"				</unique>"
			"				<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_vel\">"
			"					<check_vel/>"
			"					<not_check_vel/>"
			"					<group type=\"GroupParam\" default_child_type=\"Param\">"
			"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_vel_max\">"
			"							<check_vel_max/>"
			"							<not_check_vel_max/>"
			"						</unique>"
			"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_vel_min\">"
			"							<check_vel_min/>"
			"							<not_check_vel_min/>"
			"						</unique>"
			"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_vel_continuous\">"
			"							<check_vel_continuous/>"
			"							<not_check_vel_continuous/>"
			"						</unique>"
			"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_vel_continuous_at_start\">"
			"							<check_vel_continuous_at_start/>"
			"							<not_check_vel_continuous_at_start/>"
			"						</unique>"
			"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_vel_following_error\">"
			"							<check_vel_following_error/>"
			"							<not_check_vel_following_error />"
			"						</unique>"
			"					</group>"
			"				</unique>"
			"			</group>"
			"		</unique>"
			"	</group>"
			"</moveJ>");
	}
	MoveJ::MoveJ(const MoveJ &) = default;
	MoveJ::MoveJ(MoveJ &&) = default;
	MoveJ& MoveJ::operator=(const MoveJ &) = default;
	MoveJ& MoveJ::operator=(MoveJ &&) = default;

	struct MvL::Imp
	{
		struct Param {};
	};
	auto MvL::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		default_prepair_check_option(params, target);

		MoveJ::Param param;
		for (auto cmd_param : params)
		{
			if (cmd_param.first == "all")
			{
				param.joint_active_vec.resize(target.model->motionPool().size(), true);
			}
			else if (cmd_param.first == "none")
			{
				param.joint_active_vec.resize(target.model->motionPool().size(), false);
			}
			else if (cmd_param.first == "motion_id")
			{
				param.joint_active_vec.resize(target.model->motionPool().size(), false);
				param.joint_active_vec.at(std::stoi(cmd_param.second)) = true;
			}
			else if (cmd_param.first == "physical_id")
			{
				param.joint_active_vec.resize(dynamic_cast<aris::control::Controller *>(target.master)->motionPool().size(), false);
				param.joint_active_vec.at(dynamic_cast<aris::control::Controller*>(target.master)->motionAtPhy(std::stoi(cmd_param.second)).phyId()) = true;
			}
			else if (cmd_param.first == "slave_id")
			{
				param.joint_active_vec.resize(dynamic_cast<aris::control::Controller *>(target.master)->motionPool().size(), false);
				param.joint_active_vec.at(dynamic_cast<aris::control::Controller*>(target.master)->motionAtPhy(std::stoi(cmd_param.second)).slaId()) = true;
			}
			else if (cmd_param.first == "pos")
			{
				aris::core::Matrix mat = target.model->calculator().calculateExpression(cmd_param.second);
				if (mat.size() == 1)param.joint_pos_vec.resize(dynamic_cast<aris::control::Controller *>(target.master)->motionPool().size(), mat.toDouble());
				else
				{
					param.joint_pos_vec.resize(mat.size());
					std::copy(mat.begin(), mat.end(), param.joint_pos_vec.begin());
				}
			}
			else if (cmd_param.first == "vel")
			{
				param.vel = std::stod(cmd_param.second);
			}
			else if (cmd_param.first == "acc")
			{
				param.acc = std::stod(cmd_param.second);
			}
			else if (cmd_param.first == "dec")
			{
				param.dec = std::stod(cmd_param.second);
			}
		}

		param.begin_joint_pos_vec.resize(target.model->motionPool().size());
		target.param = param;
	}
	auto MvL::executeRT(PlanTarget &target)->int
	{
		auto param = std::any_cast<Imp::Param>(&target.param);

		return 0;
	}
	auto MvL::collectNrt(PlanTarget &param)->void {}
	MvL::~MvL() = default;
	MvL::MvL(const std::string &name) : Plan(name), imp_(new Imp)
	{
		command().loadXmlStr(
			"<moveJ default_child_type=\"Param\">"
			"	<group type=\"GroupParam\" default_child_type=\"Param\">"
			"		<limit_time default=\"5000\"/>"
			"		<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"all\">"
			"			<all abbreviation=\"a\"/>"
			"			<motion_id abbreviation=\"m\" default=\"0\"/>"
			"			<physical_id abbreviation=\"p\" default=\"0\"/>"
			"			<slave_id abbreviation=\"s\" default=\"0\"/>"
			"		</unique>"
			"		<pos default=\"0\"/>"
			"		<vel default=\"0.5\"/>"
			"		<acc default=\"1\"/>"
			"		<dec default=\"1\"/>"
			"		<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_all\">"
			"			<check_all/>"
			"			<check_none/>"
			"			<group type=\"GroupParam\" default_child_type=\"Param\">"
			"				<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_pos\">"
			"					<check_pos/>"
			"					<not_check_pos/>"
			"					<group type=\"GroupParam\" default_child_type=\"Param\">"
			"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_pos_max\">"
			"							<check_pos_max/>"
			"							<not_check_pos_max/>"
			"						</unique>"
			"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_pos_min\">"
			"							<check_pos_min/>"
			"							<not_check_pos_min/>"
			"						</unique>"
			"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_pos_continuous\">"
			"							<check_pos_continuous/>"
			"							<not_check_pos_continuous/>"
			"						</unique>"
			"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_pos_continuous_at_start\">"
			"							<check_pos_continuous_at_start/>"
			"							<not_check_pos_continuous_at_start/>"
			"						</unique>"
			"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_pos_continuous_second_order\">"
			"							<check_pos_continuous_second_order/>"
			"							<not_check_pos_continuous_second_order/>"
			"						</unique>"
			"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_pos_continuous_second_order_at_start\">"
			"							<check_pos_continuous_second_order_at_start/>"
			"							<not_check_pos_continuous_second_order_at_start/>"
			"						</unique>"
			"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_pos_following_error\">"
			"							<check_pos_following_error/>"
			"							<not_check_pos_following_error />"
			"						</unique>"
			"					</group>"
			"				</unique>"
			"				<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_vel\">"
			"					<check_vel/>"
			"					<not_check_vel/>"
			"					<group type=\"GroupParam\" default_child_type=\"Param\">"
			"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_vel_max\">"
			"							<check_vel_max/>"
			"							<not_check_vel_max/>"
			"						</unique>"
			"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_vel_min\">"
			"							<check_vel_min/>"
			"							<not_check_vel_min/>"
			"						</unique>"
			"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_vel_continuous\">"
			"							<check_vel_continuous/>"
			"							<not_check_vel_continuous/>"
			"						</unique>"
			"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_vel_continuous_at_start\">"
			"							<check_vel_continuous_at_start/>"
			"							<not_check_vel_continuous_at_start/>"
			"						</unique>"
			"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_vel_following_error\">"
			"							<check_vel_following_error/>"
			"							<not_check_vel_following_error />"
			"						</unique>"
			"					</group>"
			"				</unique>"
			"			</group>"
			"		</unique>"
			"	</group>"
			"</moveJ>");
	}
	MvL::MvL(const MvL &) = default;
	MvL::MvL(MvL &&) = default;
	MvL& MvL::operator=(const MvL &) = default;
	MvL& MvL::operator=(MvL &&) = default;

	struct AutoMove::Imp
	{
		struct Param 
		{
			double pe[6];
		};

		static std::atomic_bool is_auto_move_running_;
		//std::atomic<>
		

	};
	std::atomic_bool AutoMove::Imp::is_auto_move_running_ = false;
	std::atomic<std::array<double, 6>> auto_pe_;
	auto AutoMove::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		default_prepair_check_option(params, target);

		AutoMove::Imp::Param param;
		for (auto cmd_param : params)
		{
			if (cmd_param.first == "start")
			{
				if (Imp::is_auto_move_running_.load())throw std::runtime_error("auto mode already started");
				
				Imp::is_auto_move_running_.store(true);
				target.option |= aris::plan::Plan::EXECUTE_WHEN_ALL_PLAN_COLLECTED | NOT_PRINT_EXECUTE_COUNT;
			}
			else if (cmd_param.first == "stop")
			{
				if (!Imp::is_auto_move_running_.load())throw std::runtime_error("auto mode not started, when stop");

				Imp::is_auto_move_running_.store(false);
				target.option |= aris::plan::Plan::WAIT_FOR_COLLECTION;
			}
			else if (cmd_param.first == "pe")
			{
				if (!Imp::is_auto_move_running_.load())throw std::runtime_error("auto mode not started, when pe");
				
				aris::core::Calculator c;
				aris::core::Matrix mat;

				try 
				{
					mat = c.calculateExpression(cmd_param.second);
				}
				catch (std::runtime_error &e)
				{
					std::cout << "invalid value in AutoMove" << std::endl;
					LOG_ERROR << "invalid value in AutoMove" << std::endl;
				}

				if (mat.size() != 6)
				{
					std::cout << "invalid mat size in AutoMove" << std::endl;
					LOG_ERROR << "invalid mat size in AutoMove" << std::endl;
				}

				std::copy(mat.begin(), mat.end(), param.pe);
				
				std::array<double, 6> pe;
				std::copy(param.pe, param.pe + 6, pe.begin());
				
				double max_value[6]{ 0.5,0.5,0.5,1.0,1.0,1.0 };
				for (int i = 0; i<6; ++i)pe[i] *= max_value[i];

				auto_pe_.store(pe);
				target.option |= NOT_RUN_EXECUTE_FUNCTION | NOT_RUN_COLLECT_FUNCTION | NOT_PRINT_CMD_INFO | NOT_LOG_CMD_INFO;
			}
			else if (cmd_param.first == "vel")
			{

			}
			else if (cmd_param.first == "acc")
			{

			}
			else if (cmd_param.first == "dec")
			{

			}
		}

		target.option |= NOT_CHECK_POS_CONTINUOUS 
			| NOT_CHECK_POS_CONTINUOUS_AT_START
			| NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER
			| NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER_AT_START
			| NOT_CHECK_POS_FOLLOWING_ERROR
			| NOT_CHECK_POS_MAX
			| NOT_CHECK_POS_MIN
			| NOT_CHECK_VEL_CONTINUOUS
			| NOT_CHECK_VEL_CONTINUOUS_AT_START;
		target.param = param;
	}
	auto AutoMove::executeRT(PlanTarget &target)->int
	{
		auto param = std::any_cast<Imp::Param>(&target.param);

		if (target.count == 1)
		{
			std::array<double, 6> pe{ 0,0,0,0,0,0 };
			auto_pe_.store(pe);
		}

		// get current pe //
		double pe_now[6], ve_now[6], ae_now[6];
		target.model->generalMotionPool()[0].getMpe(pe_now, "123");
		target.model->generalMotionPool()[0].getMve(ve_now, "123");
		target.model->generalMotionPool()[0].getMae(ae_now, "123");
		
		for (int i = 3; i < 6; ++i)
		{
			if (pe_now[i] > aris::PI)
				pe_now[i] -= 2 * PI;
		}

		// get target pe //
		std::array<double, 6> pe_target, ve_target, ae_target;
		pe_target = auto_pe_.load();

		// yaw 应该为0 //
		//pe_target[5] += 0.0;

		// 向上的轴加1.0，为默认位置 //
		pe_target[2] += 1.0;
		
		// 改变yz轴朝向 //
		std::swap(pe_target[4], pe_target[5]);
		std::swap(pe_target[1], pe_target[2]);
		
		std::fill(ve_target.begin(), ve_target.end(), 0.0);
		std::fill(ae_target.begin(), ae_target.end(), 0.0);

		// now plan //
		double pe_next[6], ve_next[6], ae_next[6];
		for (int i = 0; i < 6; ++i)
		{
			aris::Size t;
			aris::plan::moveAbsolute2(pe_now[i], ve_now[i], ae_now[i]
				, pe_target[i], ve_target[i], ae_target[i]
				, 0.1, 10, 10
				, 1e-3, 1e-10, pe_next[i], ve_next[i], ae_next[i], t);
		}

		static int i = 0;
		if (++i % 1000 == 0)
		{
			//target.master->mout() << "pe_now :"
			//	<< pe_now[0] << "  " << pe_now[1] << "  " << pe_now[2] << "  "
			//	<< pe_now[3] << "  " << pe_now[4] << "  " << pe_now[5] << std::endl;
			
			//target.master->mout() << "pe_target :"
			//	<< pe_target[0] << "  "	<< pe_target[1] << "  "	<< pe_target[2] << "  "
			//	<< pe_target[3] << "  " << pe_target[4] << "  " << pe_target[5] << std::endl;

			//target.master->mout() << "pe_next:" 
			//	<< pe_next[0] << "  " << pe_next[1] << "  " << pe_next[2] << "  " 
			//	<< pe_next[3] << "  " << pe_next[4] << "  " << pe_next[5] << std::endl;
		}

		target.model->generalMotionPool()[0].setMpe(pe_next, "123");
		target.model->generalMotionPool()[0].setMve(ve_next, "123");
		target.model->generalMotionPool()[0].setMae(ae_next, "123");

		target.model->solverPool()[0].kinPos();
		
		return imp_->is_auto_move_running_.load() ? 1: 0;
	}
	auto AutoMove::collectNrt(PlanTarget &param)->void {}
	AutoMove::~AutoMove() = default;
	AutoMove::AutoMove(const std::string &name) : Plan(name), imp_(new Imp)
	{
		command().loadXmlStr(
			"<am default_child_type=\"Param\">"
			"	<group type=\"GroupParam\" default_child_type=\"Param\">"
			"		<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"start\">"
			"			<start/>"
			"			<stop/>"
			"			<group type=\"GroupParam\" default_child_type=\"Param\">"
			"				<pe default=\"{0,0,0,0,0,0}\"/>"
			"				<vel default=\"0.5\"/>"
			"				<acc default=\"1\"/>"
			"				<dec default=\"1\"/>"
			"			</group>"
			"		</unique>"
			"		<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_all\">"
			"			<check_all/>"
			"			<check_none/>"
			"			<group type=\"GroupParam\" default_child_type=\"Param\">"
			"				<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_pos\">"
			"					<check_pos/>"
			"					<not_check_pos/>"
			"					<group type=\"GroupParam\" default_child_type=\"Param\">"
			"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_pos_max\">"
			"							<check_pos_max/>"
			"							<not_check_pos_max/>"
			"						</unique>"
			"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_pos_min\">"
			"							<check_pos_min/>"
			"							<not_check_pos_min/>"
			"						</unique>"
			"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_pos_continuous\">"
			"							<check_pos_continuous/>"
			"							<not_check_pos_continuous/>"
			"						</unique>"
			"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_pos_continuous_at_start\">"
			"							<check_pos_continuous_at_start/>"
			"							<not_check_pos_continuous_at_start/>"
			"						</unique>"
			"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_pos_continuous_second_order\">"
			"							<check_pos_continuous_second_order/>"
			"							<not_check_pos_continuous_second_order/>"
			"						</unique>"
			"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_pos_continuous_second_order_at_start\">"
			"							<check_pos_continuous_second_order_at_start/>"
			"							<not_check_pos_continuous_second_order_at_start/>"
			"						</unique>"
			"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_pos_following_error\">"
			"							<check_pos_following_error/>"
			"							<not_check_pos_following_error />"
			"						</unique>"
			"					</group>"
			"				</unique>"
			"				<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_vel\">"
			"					<check_vel/>"
			"					<not_check_vel/>"
			"					<group type=\"GroupParam\" default_child_type=\"Param\">"
			"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_vel_max\">"
			"							<check_vel_max/>"
			"							<not_check_vel_max/>"
			"						</unique>"
			"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_vel_min\">"
			"							<check_vel_min/>"
			"							<not_check_vel_min/>"
			"						</unique>"
			"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_vel_continuous\">"
			"							<check_vel_continuous/>"
			"							<not_check_vel_continuous/>"
			"						</unique>"
			"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_vel_continuous_at_start\">"
			"							<check_vel_continuous_at_start/>"
			"							<not_check_vel_continuous_at_start/>"
			"						</unique>"
			"						<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"check_vel_following_error\">"
			"							<check_vel_following_error/>"
			"							<not_check_vel_following_error />"
			"						</unique>"
			"					</group>"
			"				</unique>"
			"			</group>"
			"		</unique>"
			"	</group>"
			"</am>");
	}
	AutoMove::AutoMove(const AutoMove &) = default;
	AutoMove::AutoMove(AutoMove &&) = default;
	AutoMove& AutoMove::operator=(const AutoMove &) = default;
	AutoMove& AutoMove::operator=(AutoMove &&) = default;
}
