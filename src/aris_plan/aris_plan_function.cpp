#include <algorithm>

#include"aris_plan_function.h"
#include"aris_plan_command.h"

#include <aris_control.h>
#include <aris_dynamic.h>

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
		auto PlanRoot::planParser()->aris::plan::CommandParser 
		{
			CommandParser parser;
			for (auto &plan : planPool()) parser.commandPool().add<Command>(plan.command());
			return parser;
		}
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
		
		auto default_prepair_check_option(const std::map<std::string, std::string> &cmd_params, aris::core::Msg &msg_out)->void
		{
			for (auto cmd_param : cmd_params)
			{
				if (cmd_param.first == "check_all")
				{
					msg_out.header().reserved1_ &= ~(
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
					msg_out.header().reserved1_ |=
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
					msg_out.header().reserved1_ &= ~(
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
					msg_out.header().reserved1_ |=
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
					msg_out.header().reserved1_ &= ~(
						Plan::NOT_CHECK_VEL_MIN |
						Plan::NOT_CHECK_VEL_MAX |
						Plan::NOT_CHECK_VEL_CONTINUOUS |
						Plan::NOT_CHECK_VEL_CONTINUOUS_AT_START |
						Plan::NOT_CHECK_VEL_FOLLOWING_ERROR);
				}
				else if (cmd_param.first == "not_check_vel")
				{
					msg_out.header().reserved1_ |=
						Plan::NOT_CHECK_VEL_MIN |
						Plan::NOT_CHECK_VEL_MAX |
						Plan::NOT_CHECK_VEL_CONTINUOUS |
						Plan::NOT_CHECK_VEL_CONTINUOUS_AT_START |
						Plan::NOT_CHECK_VEL_FOLLOWING_ERROR;
				}
				else if (cmd_param.first == "check_pos_min")
				{
					msg_out.header().reserved1_ &= ~Plan::NOT_CHECK_POS_MIN;
				}
				else if (cmd_param.first == "not_check_pos_min")
				{
					msg_out.header().reserved1_ |= Plan::NOT_CHECK_POS_MIN;
				}
				else if (cmd_param.first == "check_pos_max")
				{
					msg_out.header().reserved1_ &= ~Plan::NOT_CHECK_POS_MAX;
				}
				else if (cmd_param.first == "not_check_pos_max")
				{
					msg_out.header().reserved1_ |= Plan::NOT_CHECK_POS_MAX;
				}
				else if (cmd_param.first == "check_pos_continuous")
				{
					msg_out.header().reserved1_ &= ~Plan::NOT_CHECK_POS_CONTINUOUS;
				}
				else if (cmd_param.first == "not_check_pos_continuous")
				{
					msg_out.header().reserved1_ |= Plan::NOT_CHECK_POS_CONTINUOUS;
				}
				else if (cmd_param.first == "check_pos_continuous_at_start")
				{
					msg_out.header().reserved1_ &= ~Plan::NOT_CHECK_POS_CONTINUOUS_AT_START;
				}
				else if (cmd_param.first == "not_check_pos_continuous_at_start")
				{
					msg_out.header().reserved1_ |= Plan::NOT_CHECK_POS_CONTINUOUS_AT_START;
				}
				else if (cmd_param.first == "check_pos_continuous_second_order")
				{
					msg_out.header().reserved1_ &= ~Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
				}
				else if (cmd_param.first == "not_check_pos_continuous_second_order")
				{
					msg_out.header().reserved1_ |= Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
				}
				else if (cmd_param.first == "check_pos_continuous_second_order_at_start")
				{
					msg_out.header().reserved1_ &= ~Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER_AT_START;
				}
				else if (cmd_param.first == "not_check_pos_continuous_second_order_at_start")
				{
					msg_out.header().reserved1_ |= Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER_AT_START;
				}
				else if (cmd_param.first == "check_pos_following_error")
				{
					msg_out.header().reserved1_ &= ~Plan::NOT_CHECK_POS_FOLLOWING_ERROR;
				}
				else if (cmd_param.first == "not_check_pos_following_error")
				{
					msg_out.header().reserved1_ |= Plan::NOT_CHECK_POS_FOLLOWING_ERROR;
				}
				else if (cmd_param.first == "check_vel_min")
				{
					msg_out.header().reserved1_ &= ~Plan::NOT_CHECK_VEL_MIN;
				}
				else if (cmd_param.first == "not_check_vel_min")
				{
					msg_out.header().reserved1_ |= Plan::NOT_CHECK_VEL_MIN;
				}
				else if (cmd_param.first == "check_vel_max")
				{
					msg_out.header().reserved1_ &= ~Plan::NOT_CHECK_VEL_MAX;
				}
				else if (cmd_param.first == "not_check_vel_max")
				{
					msg_out.header().reserved1_ |= Plan::NOT_CHECK_VEL_MAX;
				}
				else if (cmd_param.first == "check_vel_continuous")
				{
					msg_out.header().reserved1_ &= ~Plan::NOT_CHECK_VEL_CONTINUOUS;
				}
				else if (cmd_param.first == "not_check_vel_continuous")
				{
					msg_out.header().reserved1_ |= Plan::NOT_CHECK_VEL_CONTINUOUS;
				}
				else if (cmd_param.first == "check_vel_continuous_at_start")
				{
					msg_out.header().reserved1_ &= ~Plan::NOT_CHECK_VEL_CONTINUOUS_AT_START;
				}
				else if (cmd_param.first == "not_check_vel_continuous_at_start")
				{
					msg_out.header().reserved1_ |= Plan::NOT_CHECK_VEL_CONTINUOUS_AT_START;
				}
				else if (cmd_param.first == "check_vel_following_error")
				{
					msg_out.header().reserved1_ &= ~Plan::NOT_CHECK_VEL_FOLLOWING_ERROR;
				}
				else if (cmd_param.first == "not_check_vel_following_error")
				{
					msg_out.header().reserved1_ |= Plan::NOT_CHECK_VEL_FOLLOWING_ERROR;
				}
			}
		}

		struct EnablePlan::Imp
		{
			Imp() {}
		};
		auto EnablePlan::prepairNrt(const PlanParam &param, const std::map<std::string, std::string> &cmd_params, aris::core::Msg &msg_out)->void
		{
			std::int32_t limit_time;
			std::vector<int> active_motor;

			for (auto cmd_param : cmd_params)
			{
				if (cmd_param.first == "limit_time")
					limit_time = std::stoi(cmd_param.second);
				else if (cmd_param.first == "all")
				{
					active_motor.clear();
					active_motor.resize(dynamic_cast<aris::control::Controller *>(param.master_)->motionPool().size(), 1);
				}
				else if (cmd_param.first == "none")
				{
					active_motor.clear();
					active_motor.resize(dynamic_cast<aris::control::Controller *>(param.master_)->motionPool().size(), 0);
				}
				else if (cmd_param.first == "motion_id")
				{
					active_motor.clear();
					active_motor.resize(dynamic_cast<aris::control::Controller *>(param.master_)->motionPool().size(), 0);
					active_motor.at(std::stoi(cmd_param.second)) = 1;
				}
				else if (cmd_param.first == "physical_id")
				{
					active_motor.clear();
					active_motor.resize(dynamic_cast<aris::control::Controller *>(param.master_)->motionPool().size(), 0);
					active_motor.at(dynamic_cast<aris::control::Controller*>(param.master_)->motionAtPhy(std::stoi(cmd_param.second)).phyId()) = 1;
				}
				else if (cmd_param.first == "slave_id")
				{
					active_motor.clear();
					active_motor.resize(dynamic_cast<aris::control::Controller *>(param.master_)->motionPool().size(), 0);
					active_motor.at(dynamic_cast<aris::control::Controller*>(param.master_)->motionAtPhy(std::stoi(cmd_param.second)).slaId()) = 1;
				}
				else if (cmd_param.first == "check_all")
				{
					msg_out.header().reserved1_ &= ~(
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
					msg_out.header().reserved1_ |= 
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
					throw std::runtime_error("unknown input param when prepair EnablePlan");
			}

			msg_out.copyStruct(limit_time);
			msg_out.copyMore(active_motor.data(), active_motor.size() * sizeof(int));
		}
		auto EnablePlan::executeRT(const PlanParam &param)->int
		{ 
			auto controller = dynamic_cast<aris::control::Controller *>(param.master_);
			
			bool is_all_finished = true;
			for (std::size_t i = 0; i < controller->motionPool().size(); ++i)
			{
				auto &cm = controller->motionPool().at(i);
				int * active_motor = reinterpret_cast<int*>(reinterpret_cast<char *>(param.param_) + sizeof(std::int32_t));

				if (active_motor[i])
				{
					auto ret = cm.enable();
					if (ret)
					{
						is_all_finished = false;

						if (param.count_ % 1000 == 0)
						{
							controller->mout() << "Unenabled motor, slave id: " << cm.id() << ", absolute id: " << i << ", ret: " << ret << std::endl;
						}
					}
				}
			}

			auto max_count = *reinterpret_cast<std::int32_t*>(param.param_);

			return (is_all_finished || max_count <= param.count_) ? 0 : 1;
		}
		auto EnablePlan::collectNrt(const PlanParam &param)->void {}
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
				"</en>");
		}
		EnablePlan::EnablePlan(const EnablePlan &) = default;
		EnablePlan::EnablePlan(EnablePlan &&) = default;
		EnablePlan& EnablePlan::operator=(const EnablePlan &) = default;
		EnablePlan& EnablePlan::operator=(EnablePlan &&) = default;

		struct HomePlan::Imp
		{
			Imp() {}
		};
		auto HomePlan::prepairNrt(const PlanParam &param, const std::map<std::string, std::string> &cmd_params, aris::core::Msg &msg_out)->void
		{
			std::int32_t limit_time;
			std::vector<int> active_motor;

			for (auto cmd_param : cmd_params)
			{
				if (cmd_param.first == "limit_time")
					limit_time = std::stoi(cmd_param.second);
				else if (cmd_param.first == "all")
				{
					active_motor.clear();
					active_motor.resize(dynamic_cast<aris::control::Controller *>(param.master_)->motionPool().size(), 1);
				}
				else if (cmd_param.first == "none")
				{
					active_motor.clear();
					active_motor.resize(dynamic_cast<aris::control::Controller *>(param.master_)->motionPool().size(), 0);
				}
				else if (cmd_param.first == "motion_id")
				{
					active_motor.clear();
					active_motor.resize(dynamic_cast<aris::control::Controller *>(param.master_)->motionPool().size(), 0);
					active_motor.at(std::stoi(cmd_param.second)) = 1;
				}
				else if (cmd_param.first == "physical_id")
				{
					active_motor.clear();
					active_motor.resize(dynamic_cast<aris::control::Controller *>(param.master_)->motionPool().size(), 0);
					active_motor.at(dynamic_cast<aris::control::Controller*>(param.master_)->motionAtPhy(std::stoi(cmd_param.second)).phyId()) = 1;
				}
				else if (cmd_param.first == "slave_id")
				{
					active_motor.clear();
					active_motor.resize(dynamic_cast<aris::control::Controller *>(param.master_)->motionPool().size(), 0);
					active_motor.at(dynamic_cast<aris::control::Controller*>(param.master_)->motionAtPhy(std::stoi(cmd_param.second)).slaId()) = 1;
				}
				else if (cmd_param.first == "check_all")
				{
					msg_out.header().reserved1_ &= ~(
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
					msg_out.header().reserved1_ |=
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
					throw std::runtime_error("unknown input param when prepair EnablePlan");
			}

			msg_out.copyStruct(limit_time);
			msg_out.copyMore(active_motor.data(), active_motor.size() * sizeof(int));
		}
		auto HomePlan::executeRT(const PlanParam &param)->int
		{
			auto controller = dynamic_cast<aris::control::Controller *>(param.master_);

			bool is_all_finished = true;
			for (std::size_t i = 0; i < controller->motionPool().size(); ++i)
			{
				auto &cm = controller->motionPool().at(i);
				int * active_motor = reinterpret_cast<int*>(reinterpret_cast<char *>(param.param_) + sizeof(std::int32_t));

				if (active_motor[i])
				{
					auto ret = cm.home();
					if (ret)
					{
						is_all_finished = false;

						if (param.count_ % 1000 == 0)
						{
							controller->mout() << "Unenabled motor, slave id: " << cm.id() << ", absolute id: " << i << ", ret: " << ret << std::endl;
						}
					}
				}
			}

			auto max_count = *reinterpret_cast<std::int32_t*>(param.param_);

			return (is_all_finished || max_count <= param.count_) ? 0 : 1;
		}
		auto HomePlan::collectNrt(const PlanParam &param)->void {}
		HomePlan::~HomePlan() = default;
		HomePlan::HomePlan(const std::string &name) :Plan(name), imp_(new Imp)
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
				"</en>");
		}
		HomePlan::HomePlan(const HomePlan &) = default;
		HomePlan::HomePlan(HomePlan &&) = default;
		HomePlan& HomePlan::operator=(const HomePlan &) = default;
		HomePlan& HomePlan::operator=(HomePlan &&) = default;

		struct RecoverPlan::Imp
		{
			Imp() {}
		};
		auto RecoverPlan::prepairNrt(const PlanParam &param, const std::map<std::string, std::string> &cmd_params, aris::core::Msg &msg_out)->void
		{
			std::int32_t limit_time;
			std::vector<int> active_motor;

			for (auto cmd_param : cmd_params)
			{
				if (cmd_param.first == "limit_time")
					limit_time = std::stoi(cmd_param.second);
				else if (cmd_param.first == "all")
				{
					active_motor.clear();
					active_motor.resize(dynamic_cast<aris::control::Controller *>(param.master_)->motionPool().size(), 1);
				}
				else if (cmd_param.first == "none")
				{
					active_motor.clear();
					active_motor.resize(dynamic_cast<aris::control::Controller *>(param.master_)->motionPool().size(), 0);
				}
				else if (cmd_param.first == "motion_id")
				{
					active_motor.clear();
					active_motor.resize(dynamic_cast<aris::control::Controller *>(param.master_)->motionPool().size(), 0);
					active_motor.at(std::stoi(cmd_param.second)) = 1;
				}
				else if (cmd_param.first == "physical_id")
				{
					active_motor.clear();
					active_motor.resize(dynamic_cast<aris::control::Controller *>(param.master_)->motionPool().size(), 0);
					active_motor.at(dynamic_cast<aris::control::Controller*>(param.master_)->motionAtPhy(std::stoi(cmd_param.second)).phyId()) = 1;
				}
				else if (cmd_param.first == "slave_id")
				{
					active_motor.clear();
					active_motor.resize(dynamic_cast<aris::control::Controller *>(param.master_)->motionPool().size(), 0);
					active_motor.at(dynamic_cast<aris::control::Controller*>(param.master_)->motionAtPhy(std::stoi(cmd_param.second)).slaId()) = 1;
				}
				else if (cmd_param.first == "check_all")
				{
					msg_out.header().reserved1_ &= ~(
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
					msg_out.header().reserved1_ |=
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
					throw std::runtime_error("unknown input param when prepair EnablePlan");
			}

			msg_out.copyStruct(limit_time);
			msg_out.copyMore(active_motor.data(), active_motor.size() * sizeof(int));
		}
		auto RecoverPlan::executeRT(const PlanParam &param)->int
		{
			auto controller = dynamic_cast<aris::control::Controller *>(param.master_);

			// 取得起始位置 //
			static double begin_pos[6];
			if (param.count_ == 1)
			{
				for (int i = 0; i < 6; ++i)
				{
					begin_pos[i] = controller->motionPool().at(i).actualPos();
				}
			}

			//// 计算轨迹 //
			//auto p = static_cast<RcParam*>(param.param_);
			//static aris::Size total_count[6];
			//for (int i = 0; i < 6; ++i)
			//{
			//	if (p->active[i])
			//	{
			//		auto &cm = cs.controller().motionPool().at(i);
			//		double mp, mv, ma;
			//		aris::dynamic::moveAbsolute(param.count_, begin_pos[i], p->p[i], 0.5*cm.maxVel() / 1000, 0.5*cm.maxVel() / 1000 / 1000, 0.5*cm.maxVel() / 1000 / 1000, mp, mv, ma, total_count[i]);
			//		cs.model().motionPool().at(i).setMp(mp);
			//	}
			//	else
			//	{
			//		cs.model().motionPool().at(i).setMp(begin_pos[i]);
			//		total_count[i] = 0;
			//	}
			//}

			//// 正解确定末端,这里先将角度转成弧度 //
			//double mp[6];
			//for (int i = 0; i < 6; ++i)
			//{
			//	mp[i] = cs.model().motionPool().at(i).mp();
			//	cs.model().motionPool().at(i).setMp(mp[i] / 360 * 2.0 * aris::PI);
			//}
			//cs.model().solverPool().at(1).kinPos();
			//for (int i = 0; i < 6; ++i)cs.model().motionPool().at(i).setMp(mp[i]);

			//return (static_cast<int>(*std::max_element(total_count, total_count + 6)) > param.count_) ? 1 : 0;
			return 0;
		}
		auto RecoverPlan::collectNrt(const PlanParam &param)->void {}
		RecoverPlan::~RecoverPlan() = default;
		RecoverPlan::RecoverPlan(const std::string &name) :Plan(name), imp_(new Imp)
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
				"</en>");
		}
		RecoverPlan::RecoverPlan(const RecoverPlan &) = default;
		RecoverPlan::RecoverPlan(RecoverPlan &&) = default;
		RecoverPlan& RecoverPlan::operator=(const RecoverPlan &) = default;
		RecoverPlan& RecoverPlan::operator=(RecoverPlan &&) = default;

		struct MovePlan::Imp
		{
			struct MvParam { double pq[7]; double acceleration, velocity, deceleration, angular_acceleration, angular_velocity, angular_deceleration; };
			
			Imp() {}
		};
		auto MovePlan::prepairNrt(const PlanParam &param, const std::map<std::string, std::string> &cmd_params, aris::core::Msg &msg_out)->void
		{
			Imp::MvParam mv_param;

			double data[16];

			default_prepair_check_option(cmd_params, msg_out);

			for (auto cmd_param : cmd_params)
			{
				if (cmd_param.first == "target")
				{
					auto target = param.model_->calculator().calculateExpression(cmd_param.second);
					if (target.size() != 6)throw std::runtime_error("eula angle expression must have 6 elements, first 3 are positon, others are angles");
					std::copy(target.data(), target.data() + 6, data);
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

			aris::dynamic::s_nv(3, aris::PI / 180, data + 3);
			aris::dynamic::s_pe2pq(data, mv_param.pq, "123");

			msg_out.header().reserved1_ |= USING_TARGET_POS;
			msg_out.copyStruct(mv_param);
		}
		auto MovePlan::executeRT(const PlanParam &param)->int
		{
			auto mv_param = static_cast<Imp::MvParam*>(param.param_);
			auto controller = dynamic_cast<aris::control::Controller *>(param.master_);

			// 取得起始位置 //
			static double begin_pm[16], relative_pm[16], relative_pa[6], pos_ratio, ori_ratio, norm_pos, norm_ori;
			double p, v, a;
			aris::Size pos_total_count, ori_total_count;
			if (param.count_ == 1)
			{
				double end_pm[16];
				aris::dynamic::s_pq2pm(mv_param->pq, end_pm);
				param.model_->generalMotionPool().at(0).updMpm();
				param.model_->generalMotionPool().at(0).getMpm(begin_pm);
				aris::dynamic::s_inv_pm_dot_pm(begin_pm, end_pm, relative_pm);

				// relative_pa //
				aris::dynamic::s_pm2pa(relative_pm, relative_pa);

				norm_pos = aris::dynamic::s_norm(3, relative_pa);
				norm_ori = aris::dynamic::s_norm(3, relative_pa + 3);
				
				aris::dynamic::moveAbsolute(param.count_, 0.0, norm_pos, mv_param->velocity / 1000, mv_param->acceleration / 1000 / 1000, mv_param->deceleration / 1000 / 1000, p, v, a, pos_total_count);
				aris::dynamic::moveAbsolute(param.count_, 0.0, norm_ori, mv_param->angular_velocity / 1000, mv_param->angular_acceleration / 1000 / 1000, mv_param->angular_deceleration / 1000 / 1000, p, v, a, ori_total_count);

				pos_ratio = pos_total_count < ori_total_count ? double(pos_total_count) / ori_total_count : 1.0;
				ori_ratio = ori_total_count < pos_total_count ? double(ori_total_count) / pos_total_count : 1.0;

				std::cout << "pos count before:" << pos_total_count << std::endl;
				std::cout << "ori count before:" << ori_total_count << std::endl;

				aris::dynamic::moveAbsolute(param.count_, 0.0, norm_pos, mv_param->velocity / 1000 * pos_ratio, mv_param->acceleration / 1000 / 1000 * pos_ratio* pos_ratio, mv_param->deceleration / 1000 / 1000 * pos_ratio* pos_ratio, p, v, a, pos_total_count);
				aris::dynamic::moveAbsolute(param.count_, 0.0, norm_ori, mv_param->angular_velocity / 1000 * ori_ratio, mv_param->angular_acceleration / 1000 / 1000 * ori_ratio * ori_ratio, mv_param->angular_deceleration / 1000 / 1000 * ori_ratio * ori_ratio, p, v, a, ori_total_count);

				std::cout << "pos count after:" << pos_total_count << std::endl;
				std::cout << "ori count after:" << ori_total_count << std::endl;
			}

			double pa[6]{ 0,0,0,0,0,0 }, pm[16], pm2[16];
			
			aris::dynamic::moveAbsolute(param.count_, 0.0, norm_pos, mv_param->velocity / 1000 * pos_ratio, mv_param->acceleration / 1000 / 1000 * pos_ratio* pos_ratio, mv_param->deceleration / 1000 / 1000 * pos_ratio* pos_ratio, p, v, a, pos_total_count);
			if (norm_pos > 1e-10)aris::dynamic::s_vc(3, p / norm_pos, relative_pa, pa);
			
			aris::dynamic::moveAbsolute(param.count_, 0.0, norm_ori, mv_param->angular_velocity / 1000 * ori_ratio, mv_param->angular_acceleration / 1000 / 1000 * ori_ratio * ori_ratio, mv_param->angular_deceleration / 1000 / 1000 * ori_ratio * ori_ratio, p, v, a, ori_total_count);
			if (norm_ori > 1e-10)aris::dynamic::s_vc(3, p / norm_ori, relative_pa + 3, pa + 3);





			aris::dynamic::s_pa2pm(pa, pm);
			aris::dynamic::s_pm_dot_pm(begin_pm, pm, pm2);



			for (int i = 0; i < 16; ++i)
				param.master_->lout()<< std::setprecision(16) << "  " << pm[i];

			if (std::max(pos_total_count, ori_total_count) == param.count_)
			{
				aris::dynamic::dsp(1, 6, pa);
				aris::dynamic::dsp(1, 6, relative_pa);
				
				aris::dynamic::dsp(4, 4, pm);
				aris::dynamic::dsp(4, 4, pm2);
			}



			// 反解计算电机位置 //
			param.model_->generalMotionPool().at(0).setMpm(pm2);
			param.model_->solverPool().at(0).kinPos();
			for (int i = 0; i < 6; ++i)
			{
				param.model_->motionPool().at(i).updMp();
				param.model_->motionPool().at(i).setMp(param.model_->motionPool().at(i).mp() * 360 / 2.0 / aris::PI);
			}

			return std::max(pos_total_count, ori_total_count) > param.count_ ? 1 : 0;
		}
		auto MovePlan::collectNrt(const PlanParam &param)->void {}
		MovePlan::~MovePlan() = default;
		MovePlan::MovePlan(const std::string &name) :Plan(name), imp_(new Imp)
		{
			command().loadXmlStr(
				"<mv default_child_type=\"Param\">"
				"	<group type=\"GroupParam\" default_child_type=\"Param\">"
				"		<position_unit default=\"m\"/>"
				"		<unique type=\"UniqueParam\" default_child_type=\"Param\" default=\"target\">"
				"			<target default=\"{0,0.63,0.316,0,-90,-90}\" abbreviation=\"t\"/>"
				"			<pq default=\"{0,0.63,0.316,0,0,0,1}\"/>"
				"			<pm default=\"{1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1}\"/>"
				"			<group type=\"GroupParam\" default_child_type=\"Param\">"
				"				<pe default=\"{0,0.63,0.316,0,0,0}\"/>"
				"				<orientation_unit default=\"rad\"/>"
				"				<euler_type default=\"321\"/>"
				"			</group>"
				"			<group type=\"GroupParam\" default_child_type=\"Param\">"
				"				<xyz default=\"{0,0.63,0.316,0,0,0}\"/>"
				"				<rpy default=\"rad\"/>"
				"			</group>"
				"		</unique>"
				"		<acceleration default=\"0.1\"/>"
				"		<velocity default=\"0.1\"/>"
				"		<deceleration default=\"0.1\"/>"
				"		<angular_acceleration default=\"0.1\"/>"
				"		<angular_velocity default=\"0.1\"/>"
				"		<angular_deceleration default=\"0.1\"/>"
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
		auto UniversalPlan::prepairNrt(const PlanParam &param, const std::map<std::string, std::string> &cmd_params, aris::core::Msg &msg_out)->void { if (imp_->prepair_nrt)imp_->prepair_nrt(param, cmd_params, msg_out); }
		auto UniversalPlan::executeRT(const PlanParam &param)->int{	return imp_->execute_rt ? imp_->execute_rt(param) : 0;}
		auto UniversalPlan::collectNrt(const PlanParam &param)->void{	if (imp_->collect_nrt)imp_->collect_nrt(param);}
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

		auto simulateCommand(std::string cmd_string, aris::plan::PlanRoot *plan_root, aris::dynamic::Model *model, aris::dynamic::SimResult *result)->void
		{
			// parse cmd_string to cmd and param map //
			CommandParser parser;
			for (auto &plan : plan_root->planPool()) parser.commandPool().add<Command>(plan.command());
			
			std::string cmd;
			std::map<std::string, std::string> param_map;
			parser.parse(cmd_string, cmd, param_map);

			std::cout << "cmd:" << cmd <<std::endl;
			for (auto param : param_map)
			{
				std::cout << "  " << param.first << "   " << param.second << std::endl;
			}


			// find the plan //
			auto f = std::find_if(plan_root->planPool().begin(), plan_root->planPool().end(), [&](decltype(*plan_root->planPool().begin())& iter)->bool {return iter.command().name() == cmd; });
			std::cout << f->name() << std::endl;

			// run prepare function //




		}
	}
}
