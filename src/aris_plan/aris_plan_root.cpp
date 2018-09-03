#include <algorithm>

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

	struct EnablePlan::Imp { Imp() {} };
	auto EnablePlan::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		std::int32_t limit_time;
		std::vector<int> active_motor;

		for (auto cmd_param : params)
		{
			if (cmd_param.first == "limit_time")
				limit_time = std::stoi(cmd_param.second);
			else if (cmd_param.first == "all")
			{
				active_motor.clear();
				active_motor.resize(dynamic_cast<aris::control::Controller *>(target.master)->motionPool().size(), 1);
			}
			else if (cmd_param.first == "none")
			{
				active_motor.clear();
				active_motor.resize(dynamic_cast<aris::control::Controller *>(target.master)->motionPool().size(), 0);
			}
			else if (cmd_param.first == "motion_id")
			{
				active_motor.clear();
				active_motor.resize(dynamic_cast<aris::control::Controller *>(target.master)->motionPool().size(), 0);
				active_motor.at(std::stoi(cmd_param.second)) = 1;
			}
			else if (cmd_param.first == "physical_id")
			{
				active_motor.clear();
				active_motor.resize(dynamic_cast<aris::control::Controller *>(target.master)->motionPool().size(), 0);
				active_motor.at(dynamic_cast<aris::control::Controller*>(target.master)->motionAtPhy(std::stoi(cmd_param.second)).phyId()) = 1;
			}
			else if (cmd_param.first == "slave_id")
			{
				active_motor.clear();
				active_motor.resize(dynamic_cast<aris::control::Controller *>(target.master)->motionPool().size(), 0);
				active_motor.at(dynamic_cast<aris::control::Controller*>(target.master)->motionAtPhy(std::stoi(cmd_param.second)).slaId()) = 1;
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

		target.param = limit_time;
		//msg_out.copyMore(active_motor.data(), active_motor.size() * sizeof(int));
	}
	auto EnablePlan::executeRT(PlanTarget &param)->int
	{
		auto controller = dynamic_cast<aris::control::Controller *>(param.master);

		bool is_all_finished = true;
		for (std::size_t i = 0; i < controller->motionPool().size(); ++i)
		{
			auto &cm = controller->motionPool().at(i);
//			int * active_motor = reinterpret_cast<int*>(reinterpret_cast<char *>(param.param) + sizeof(std::int32_t));

//			if (active_motor[i])
			{
				auto ret = cm.enable();
				if (ret)
				{
					is_all_finished = false;

					if (param.count % 1000 == 0)
					{
						controller->mout() << "Unenabled motor, slave id: " << cm.id() << ", absolute id: " << i << ", ret: " << ret << std::endl;
					}
				}
			}
		}

//		auto max_count = *reinterpret_cast<std::uint32_t*>(param.param);
		auto max_count = 1000;

		return (is_all_finished || max_count <= param.count) ? 0 : 1;
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

	struct RecoverParam
	{
		double axis_pos[6]{ 0.0 };
		double acceleration, velocity, deceleration;
	};
	struct RecoverPlan::Imp { Imp() {} };
	auto RecoverPlan::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		default_prepair_check_option(params, target);

		RecoverParam rc_param;

		double pe[6], pe_unit;
		std::string eul_type;
		bool use_pe{ false };
		for (auto cmd_param : params)
		{
			if (cmd_param.first == "pq")
			{
				auto pq = target.model->calculator().calculateExpression(cmd_param.second);
				if (pq.size() != 7)throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + " failed");
				target.model->generalMotionPool().at(0).setMpq(pq.data());
			}
			else if (cmd_param.first == "pm")
			{
				auto pm = target.model->calculator().calculateExpression(cmd_param.second);
				if (pm.size() != 16)throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + " failed");
				target.model->generalMotionPool().at(0).setMpm(pm.data());
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
				rc_param.acceleration = std::atof(cmd_param.second.c_str());
			}
			else if (cmd_param.first == "velocity")
			{
				rc_param.velocity = std::atof(cmd_param.second.c_str());
			}
			else if (cmd_param.first == "deceleration")
			{
				rc_param.deceleration = std::atof(cmd_param.second.c_str());
			}
		}

		if (use_pe)
		{
			aris::dynamic::s_nv(3, pe_unit, pe + 3);
			target.model->generalMotionPool().at(0).setMpe(pe, eul_type.data());
		}
		//////////////////
		auto &gm = target.model->generalMotionPool().at(0);
		aris::dynamic::dsp(4, 4, *gm.mpm());
		/////////////////

		aris::dynamic::dsp(4, 4, *gm.makI().pm());
		aris::dynamic::dsp(4, 4, *gm.makJ().pm());
		if (!target.model->solverPool().at(0).kinPos())throw std::runtime_error(__FILE__ + std::to_string(__LINE__) + " failed");

		for (aris::Size i = 0; i < target.model->motionPool().size(); ++i)
		{
			target.model->motionPool().at(i).updMp();
			rc_param.axis_pos[i] = target.model->motionPool().at(i).mp();
		}

		target.option =
			aris::plan::Plan::USING_TARGET_POS | 
			aris::plan::Plan::EXECUTE_WHEN_ALL_PLAN_COLLECTED |
			aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_AT_START | 
			aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER_AT_START;

		target.param = rc_param;
	}
	auto RecoverPlan::executeRT(PlanTarget &target)->int
	{
		auto rc_param = std::any_cast<RecoverParam>(&target.param);
		auto controller = dynamic_cast<aris::control::Controller *>(target.master);

		// 取得起始位置 //
		static double begin_pos[6], acc[6], vel[6], dec[6];
		static aris::Size total_count[6];
		if (target.count == 1)
		{
			for (int i = 0; i < controller->motionPool().size(); ++i)
			{
				begin_pos[i] = controller->motionPool().at(i).actualPos();
			}
		}

		for (int i = 0; i < controller->motionPool().size(); ++i)
		{
			double p, v, a;
			aris::plan::moveAbsolute(target.count, begin_pos[i], rc_param->axis_pos[i], rc_param->velocity / 1000, rc_param->acceleration / 1000 / 1000, rc_param->deceleration / 1000 / 1000, p, v, a, total_count[i]);
			target.model->motionPool().at(i).setMp(p);
		}

		double pq[7];
		aris::dynamic::s_pm2pq(*target.model->generalMotionPool().at(0).mpm(), pq);
		target.master->lout() << target.count << " " << pq[0] << " " << pq[1] << " " << pq[2] << " " << pq[3] << " " << pq[4] << " " << pq[5] << " " << pq[6] << "\n";

		target.model->solverPool().at(1).kinPos();
		return (static_cast<int>(*std::max_element(total_count, total_count + 6)) > target.count) ? 1 : 0;
	}
	auto RecoverPlan::collectNrt(PlanTarget &target)->void {}
	RecoverPlan::~RecoverPlan() = default;
	RecoverPlan::RecoverPlan(const std::string &name) :Plan(name), imp_(new Imp)
	{
		command().loadXmlStr(
			"<rc default_child_type=\"Param\">"
			"	<group type=\"GroupParam\" default_child_type=\"Param\">"
			"		<position_unit default=\"m\"/>"
			"		<unique_pos type=\"UniqueParam\" default_child_type=\"Param\" default=\"pq\">"
			"			<pq default=\"{0,0,0,0,0,0,1}\"/>"
			"			<pm default=\"{1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1}\"/>"
			"			<group type=\"GroupParam\" default_child_type=\"Param\">"
			"				<pe default=\"{0,0,0,0,0,0}\"/>"
			"				<orientation_unit default=\"rad\"/>"
			"				<euler_type default=\"321\"/>"
			"			</group>"
			"		</unique_pos>"
			"		<acceleration default=\"0.5\"/>"
			"		<velocity default=\"0.5\"/>"
			"		<deceleration default=\"0.5\"/>"
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
			"</rc>");
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

		target.option |= USING_TARGET_POS;
		target.param = mv_param;
	}
	auto MovePlan::executeRT(PlanTarget &param)->int
	{
		auto mv_param = std::any_cast<Imp::MvParam>(&param.param);
		auto controller = dynamic_cast<aris::control::Controller *>(param.master);

		// 取得起始位置 //
		static double begin_pm[16], relative_pm[16], relative_pa[6], pos_ratio, ori_ratio, norm_pos, norm_ori;
		double p, v, a;
		aris::Size pos_total_count, ori_total_count;
		if (param.count == 1)
		{
			double end_pm[16];
			aris::dynamic::s_pq2pm(mv_param->pq, end_pm);
			param.model->generalMotionPool().at(0).updMpm();
			param.model->generalMotionPool().at(0).getMpm(begin_pm);
			aris::dynamic::s_inv_pm_dot_pm(begin_pm, end_pm, relative_pm);

			// relative_pa //
			aris::dynamic::s_pm2pa(relative_pm, relative_pa);

			norm_pos = aris::dynamic::s_norm(3, relative_pa);
			norm_ori = aris::dynamic::s_norm(3, relative_pa + 3);

			aris::plan::moveAbsolute(param.count, 0.0, norm_pos, mv_param->velocity / 1000, mv_param->acceleration / 1000 / 1000, mv_param->deceleration / 1000 / 1000, p, v, a, pos_total_count);
			aris::plan::moveAbsolute(param.count, 0.0, norm_ori, mv_param->angular_velocity / 1000, mv_param->angular_acceleration / 1000 / 1000, mv_param->angular_deceleration / 1000 / 1000, p, v, a, ori_total_count);

			pos_ratio = pos_total_count < ori_total_count ? double(pos_total_count) / ori_total_count : 1.0;
			ori_ratio = ori_total_count < pos_total_count ? double(ori_total_count) / pos_total_count : 1.0;

			std::cout << "pos count before:" << pos_total_count << std::endl;
			std::cout << "ori count before:" << ori_total_count << std::endl;

			aris::plan::moveAbsolute(param.count, 0.0, norm_pos, mv_param->velocity / 1000 * pos_ratio, mv_param->acceleration / 1000 / 1000 * pos_ratio* pos_ratio, mv_param->deceleration / 1000 / 1000 * pos_ratio* pos_ratio, p, v, a, pos_total_count);
			aris::plan::moveAbsolute(param.count, 0.0, norm_ori, mv_param->angular_velocity / 1000 * ori_ratio, mv_param->angular_acceleration / 1000 / 1000 * ori_ratio * ori_ratio, mv_param->angular_deceleration / 1000 / 1000 * ori_ratio * ori_ratio, p, v, a, ori_total_count);

			std::cout << "pos count after:" << pos_total_count << std::endl;
			std::cout << "ori count after:" << ori_total_count << std::endl;
		}

		double pa[6]{ 0,0,0,0,0,0 }, pm[16], pm2[16];

		aris::plan::moveAbsolute(param.count, 0.0, norm_pos, mv_param->velocity / 1000 * pos_ratio, mv_param->acceleration / 1000 / 1000 * pos_ratio* pos_ratio, mv_param->deceleration / 1000 / 1000 * pos_ratio* pos_ratio, p, v, a, pos_total_count);
		if (norm_pos > 1e-10)aris::dynamic::s_vc(3, p / norm_pos, relative_pa, pa);

		aris::plan::moveAbsolute(param.count, 0.0, norm_ori, mv_param->angular_velocity / 1000 * ori_ratio, mv_param->angular_acceleration / 1000 / 1000 * ori_ratio * ori_ratio, mv_param->angular_deceleration / 1000 / 1000 * ori_ratio * ori_ratio, p, v, a, ori_total_count);
		if (norm_ori > 1e-10)aris::dynamic::s_vc(3, p / norm_ori, relative_pa + 3, pa + 3);

		aris::dynamic::s_pa2pm(pa, pm);
		aris::dynamic::s_pm_dot_pm(begin_pm, pm, pm2);

		/////////////////////////////////////////////
		//////////////////
		auto &gm = param.model->generalMotionPool().at(0);
		aris::dynamic::dsp(4, 4, *gm.mpm());
		/////////////////

		aris::dynamic::dsp(4, 4, *gm.makI().pm());
		aris::dynamic::dsp(4, 4, *gm.makJ().pm());
		/////////////////////////////////////////////

		// 反解计算电机位置 //
		param.model->generalMotionPool().at(0).setMpm(pm2);
		if (!param.model->solverPool().at(0).kinPos())return -1;


		double pq[7];
		aris::dynamic::s_pm2pq(*param.model->generalMotionPool().at(0).mpm(), pq);
		param.master->lout() << param.count << " " << pq[0] << " " << pq[1] << " " << pq[2] << " " << pq[3] << " " << pq[4] << " " << pq[5] << " " << pq[6] << "\n";


		return std::max(pos_total_count, ori_total_count) > param.count ? 1 : 0;
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

	auto MoveJ::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void 
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
				if(mat.size()==1)param.joint_pos_vec.resize(dynamic_cast<aris::control::Controller *>(target.master)->motionPool().size(), mat.toDouble());
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
	auto MoveJ::executeRT(PlanTarget &target)->int 
	{ 
		auto param = std::any_cast<Param>(&target.param);
		auto controller = dynamic_cast<aris::control::Controller *>(target.master);

		if (target.count == 1)
		{
			for (auto i = 0; i < param->joint_active_vec.size(); ++i)
			{
				if (param->joint_active_vec[i])
				{
					param->begin_joint_pos_vec[i] = target.model->motionPool().at(i).mp();
				}
			}
		}

		aris::Size total_count{ 1 };
		for (auto i = 0; i < param->joint_active_vec.size(); ++i)
		{
			if (param->joint_active_vec[i])
			{
				double p, v, a;
				aris::Size t_count;
				aris::plan::moveAbsolute(target.count, param->begin_joint_pos_vec[i], param->joint_pos_vec[i], param->vel / 1000, param->acc / 1000 / 1000, param->dec / 1000 / 1000, p, v, a, t_count);
				target.model->motionPool().at(i).setMp(p);
				total_count = std::max(total_count, t_count);
			}
		}

		target.model->solverPool().at(1).kinPos();
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
}
