#include <algorithm>
#include <future>

#include"aris/plan/function.hpp"
#include"aris/plan/root.hpp"

#include <aris/control/control.hpp>
#include <aris/dynamic/dynamic.hpp>
#include <aris/server/server.hpp>

namespace aris::plan
{
	struct Plan::Imp {};
	auto Plan::command()->aris::core::Command & { return dynamic_cast<aris::core::Command&>(children().front()); }
	Plan::~Plan() = default;
	Plan::Plan(const std::string &name) :Object(name), imp_(new Imp)
	{
		add<aris::core::Command>(name);
	}
	ARIS_DEFINE_BIG_FOUR_CPP(Plan);

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
		aris::core::Object::registerTypeGlobal<aris::core::ObjectPool<Plan> >();
		add<aris::core::ObjectPool<Plan> >("plan_pool_object");
	}
	ARIS_DEFINE_BIG_FOUR_CPP(PlanRoot);

#define CHECK_PARAM_STRING \
		"		<UniqueParam default=\"check_none\">" \
		"			<Param name=\"check_all\"/>" \
		"			<Param name=\"check_none\"/>" \
		"			<GroupParam>"\
		"				<UniqueParam default=\"check_pos\">"\
		"					<Param name=\"check_pos\"/>"\
		"					<Param name=\"not_check_pos\"/>"\
		"					<GroupParam>"\
		"						<UniqueParam default=\"check_pos_max\">"\
		"							<Param name=\"check_pos_max\"/>"\
		"							<Param name=\"not_check_pos_max\"/>"\
		"						</UniqueParam>"\
		"						<UniqueParam default=\"check_pos_min\">"\
		"							<Param name=\"check_pos_min\"/>"\
		"							<Param name=\"not_check_pos_min\"/>"\
		"						</UniqueParam>"\
		"						<UniqueParam default=\"check_pos_continuous\">"\
		"							<Param name=\"check_pos_continuous\"/>"\
		"							<Param name=\"not_check_pos_continuous\"/>"\
		"						</UniqueParam>"\
		"						<UniqueParam default=\"check_pos_continuous_at_start\">"\
		"							<Param name=\"check_pos_continuous_at_start\"/>"\
		"							<Param name=\"not_check_pos_continuous_at_start\"/>"\
		"						</UniqueParam>"\
		"						<UniqueParam default=\"check_pos_continuous_second_order\">"\
		"							<Param name=\"check_pos_continuous_second_order\"/>"\
		"							<Param name=\"not_check_pos_continuous_second_order\"/>"\
		"						</UniqueParam>"\
		"						<UniqueParam default=\"check_pos_continuous_second_order_at_start\">"\
		"							<Param name=\"check_pos_continuous_second_order_at_start\"/>"\
		"							<Param name=\"not_check_pos_continuous_second_order_at_start\"/>"\
		"						</UniqueParam>"\
		"						<UniqueParam default=\"check_pos_following_error\">"\
		"							<Param name=\"check_pos_following_error\"/>"\
		"							<Param name=\"not_check_pos_following_error\"/>"\
		"						</UniqueParam>"\
		"					</GroupParam>"\
		"				</UniqueParam>"\
		"				<UniqueParam default=\"check_vel\">"\
		"					<Param name=\"check_vel\"/>"\
		"					<Param name=\"not_check_vel\"/>"\
		"					<GroupParam>"\
		"						<UniqueParam default=\"check_vel_max\">"\
		"							<Param name=\"check_vel_max\"/>"\
		"							<Param name=\"not_check_vel_max\"/>"\
		"						</UniqueParam>"\
		"						<UniqueParam default=\"check_vel_min\">"\
		"							<Param name=\"check_vel_min\"/>"\
		"							<Param name=\"not_check_vel_min\"/>"\
		"						</UniqueParam>"\
		"						<UniqueParam default=\"check_vel_continuous\">"\
		"							<Param name=\"check_vel_continuous\"/>"\
		"							<Param name=\"not_check_vel_continuous\"/>"\
		"						</UniqueParam>"\
		"						<UniqueParam default=\"check_vel_continuous_at_start\">"\
		"							<Param name=\"check_vel_continuous_at_start\"/>"\
		"							<Param name=\"not_check_vel_continuous_at_start\"/>"\
		"						</UniqueParam>"\
		"						<UniqueParam default=\"check_vel_following_error\">"\
		"							<Param name=\"check_vel_following_error\"/>"\
		"							<Param name=\"not_check_vel_following_error\"/>"\
		"						</UniqueParam>"\
		"					</GroupParam>"\
		"				</UniqueParam>"\
		"			</GroupParam>"\
		"		</UniqueParam>"
	auto set_check_option(const std::map<std::string, std::string> &cmd_params, PlanTarget &target)->void
	{
		for (auto cmd_param : cmd_params)
		{
			if (cmd_param.first == "check_all")
			{
				target.option &= ~(
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
				target.option |=
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
				target.option &= ~(
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
				target.option |=
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
				target.option &= ~(
					Plan::NOT_CHECK_VEL_MIN |
					Plan::NOT_CHECK_VEL_MAX |
					Plan::NOT_CHECK_VEL_CONTINUOUS |
					Plan::NOT_CHECK_VEL_CONTINUOUS_AT_START |
					Plan::NOT_CHECK_VEL_FOLLOWING_ERROR);
			}
			else if (cmd_param.first == "not_check_vel")
			{
				target.option |=
					Plan::NOT_CHECK_VEL_MIN |
					Plan::NOT_CHECK_VEL_MAX |
					Plan::NOT_CHECK_VEL_CONTINUOUS |
					Plan::NOT_CHECK_VEL_CONTINUOUS_AT_START |
					Plan::NOT_CHECK_VEL_FOLLOWING_ERROR;
			}
			else if (cmd_param.first == "check_pos_min")
			{
				target.option &= ~Plan::NOT_CHECK_POS_MIN;
			}
			else if (cmd_param.first == "not_check_pos_min")
			{
				target.option |= Plan::NOT_CHECK_POS_MIN;
			}
			else if (cmd_param.first == "check_pos_max")
			{
				target.option &= ~Plan::NOT_CHECK_POS_MAX;
			}
			else if (cmd_param.first == "not_check_pos_max")
			{
				target.option |= Plan::NOT_CHECK_POS_MAX;
			}
			else if (cmd_param.first == "check_pos_continuous")
			{
				target.option &= ~Plan::NOT_CHECK_POS_CONTINUOUS;
			}
			else if (cmd_param.first == "not_check_pos_continuous")
			{
				target.option |= Plan::NOT_CHECK_POS_CONTINUOUS;
			}
			else if (cmd_param.first == "check_pos_continuous_at_start")
			{
				target.option &= ~Plan::NOT_CHECK_POS_CONTINUOUS_AT_START;
			}
			else if (cmd_param.first == "not_check_pos_continuous_at_start")
			{
				target.option |= Plan::NOT_CHECK_POS_CONTINUOUS_AT_START;
			}
			else if (cmd_param.first == "check_pos_continuous_second_order")
			{
				target.option &= ~Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
			}
			else if (cmd_param.first == "not_check_pos_continuous_second_order")
			{
				target.option |= Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
			}
			else if (cmd_param.first == "check_pos_continuous_second_order_at_start")
			{
				target.option &= ~Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER_AT_START;
			}
			else if (cmd_param.first == "not_check_pos_continuous_second_order_at_start")
			{
				target.option |= Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER_AT_START;
			}
			else if (cmd_param.first == "check_pos_following_error")
			{
				target.option &= ~Plan::NOT_CHECK_POS_FOLLOWING_ERROR;
			}
			else if (cmd_param.first == "not_check_pos_following_error")
			{
				target.option |= Plan::NOT_CHECK_POS_FOLLOWING_ERROR;
			}
			else if (cmd_param.first == "check_vel_min")
			{
				target.option &= ~Plan::NOT_CHECK_VEL_MIN;
			}
			else if (cmd_param.first == "not_check_vel_min")
			{
				target.option |= Plan::NOT_CHECK_VEL_MIN;
			}
			else if (cmd_param.first == "check_vel_max")
			{
				target.option &= ~Plan::NOT_CHECK_VEL_MAX;
			}
			else if (cmd_param.first == "not_check_vel_max")
			{
				target.option |= Plan::NOT_CHECK_VEL_MAX;
			}
			else if (cmd_param.first == "check_vel_continuous")
			{
				target.option &= ~Plan::NOT_CHECK_VEL_CONTINUOUS;
			}
			else if (cmd_param.first == "not_check_vel_continuous")
			{
				target.option |= Plan::NOT_CHECK_VEL_CONTINUOUS;
			}
			else if (cmd_param.first == "check_vel_continuous_at_start")
			{
				target.option &= ~Plan::NOT_CHECK_VEL_CONTINUOUS_AT_START;
			}
			else if (cmd_param.first == "not_check_vel_continuous_at_start")
			{
				target.option |= Plan::NOT_CHECK_VEL_CONTINUOUS_AT_START;
			}
			else if (cmd_param.first == "check_vel_following_error")
			{
				target.option &= ~Plan::NOT_CHECK_VEL_FOLLOWING_ERROR;
			}
			else if (cmd_param.first == "not_check_vel_following_error")
			{
				target.option |= Plan::NOT_CHECK_VEL_FOLLOWING_ERROR;
			}
		}
	}

	struct SetActiveMotor { std::vector<int> active_motor; };
#define SELECT_MOTOR_STRING \
		"		<UniqueParam default=\"all\">"\
		"			<Param name=\"all\" abbreviation=\"a\"/>"\
		"			<Param name=\"motion_id\" abbreviation=\"m\" default=\"0\"/>"\
		"			<Param name=\"physical_id\" abbreviation=\"p\" default=\"0\"/>"\
		"			<Param name=\"slave_id\" abbreviation=\"s\" default=\"0\"/>"\
		"		</UniqueParam>"
	auto set_active_motor(const std::map<std::string, std::string> &cmd_params, PlanTarget &target, SetActiveMotor &param)->void
	{
		param.active_motor.clear();
		param.active_motor.resize(target.controller->motionPool().size(), 0);
		
		for (auto cmd_param : cmd_params)
		{
			if (cmd_param.first == "all")
			{
				std::fill(param.active_motor.begin(), param.active_motor.end(), 1);
			}
			else if (cmd_param.first == "motion_id")
			{
				param.active_motor.at(std::stoi(cmd_param.second)) = 1;
			}
			else if (cmd_param.first == "physical_id")
			{
				param.active_motor.at(target.controller->motionAtPhy(std::stoi(cmd_param.second)).motId()) = 1;
			}
			else if (cmd_param.first == "slave_id")
			{
				param.active_motor.at(target.controller->motionAtSla(std::stoi(cmd_param.second)).motId()) = 1;
			}
		}
	}

	struct SetInputMovement 
	{
		std::vector<double> axis_begin_pos_vec;
		std::vector<double> axis_pos_vec;
		std::vector<double> axis_vel_vec;
		std::vector<double> axis_acc_vec;
		std::vector<double> axis_dec_vec;
	};
#define SET_INPUT_MOVEMENT_STRING \
		"		<Param name=\"pos\" default=\"0.5\"/>"\
		"		<Param name=\"acc\" default=\"0.1\"/>"\
		"		<Param name=\"vel\" default=\"0.1\"/>"\
		"		<Param name=\"dec\" default=\"0.1\"/>"
	auto set_input_movement(const std::map<std::string, std::string> &cmd_params, PlanTarget &target, SetInputMovement &param)->void
	{
		param.axis_begin_pos_vec.resize(target.controller->motionPool().size(), 0.0);
		for (auto cmd_param : cmd_params)
		{
			if (cmd_param.first == "pos")
			{
				auto p = target.model->calculator().calculateExpression(cmd_param.second);
				if (p.size() == 1)
				{
					param.axis_pos_vec.resize(target.controller->motionPool().size(), p.toDouble());
				}
				else if (p.size() == target.controller->motionPool().size())
				{
					param.axis_pos_vec.assign(p.begin(), p.end());
				}
				else
				{
					THROW_FILE_AND_LINE("");
				}
			}
			else if (cmd_param.first == "acc")
			{
				auto a = target.model->calculator().calculateExpression(cmd_param.second);

				if (a.size() == 1)
				{
					param.axis_acc_vec.resize(target.controller->motionPool().size(), a.toDouble());
				}
				else if (a.size() == target.controller->motionPool().size())
				{
					param.axis_acc_vec.assign(a.begin(), a.end());
				}
				else
				{
					THROW_FILE_AND_LINE("");
				}

			}
			else if (cmd_param.first == "vel")
			{
				auto v = target.model->calculator().calculateExpression(cmd_param.second);

				if (v.size() == 1)
				{
					param.axis_vel_vec.resize(target.controller->motionPool().size(), v.toDouble());
				}
				else if (v.size() == target.controller->motionPool().size())
				{
					param.axis_vel_vec.assign(v.begin(), v.end());
				}
				else
				{
					THROW_FILE_AND_LINE("");
				}
			}
			else if (cmd_param.first == "dec")
			{
				auto d = target.model->calculator().calculateExpression(cmd_param.second);

				if (d.size() == 1)
				{
					param.axis_dec_vec.resize(target.controller->motionPool().size(), d.toDouble());
				}
				else if (d.size() == target.controller->motionPool().size())
				{
					param.axis_dec_vec.assign(d.begin(), d.end());
				}
				else
				{
					THROW_FILE_AND_LINE("");
				}
			}
		}
	}
	auto check_input_movement(const std::map<std::string, std::string> &cmd_params, PlanTarget &target, SetInputMovement &param)->void
	{
		auto c = target.controller;
		for (Size i = 0; i < c->motionPool().size(); ++i)
		{
			if (param.axis_pos_vec[i] > c->motionPool()[i].maxPos() || param.axis_pos_vec[i] < c->motionPool()[i].minPos())
				THROW_FILE_AND_LINE("");

			if (param.axis_vel_vec[i] > c->motionPool()[i].maxVel() || param.axis_vel_vec[i] <= 0.0)
				THROW_FILE_AND_LINE("");

			if (param.axis_acc_vec[i] > c->motionPool()[i].maxAcc() || param.axis_acc_vec[i] <= 0.0)
				THROW_FILE_AND_LINE("");

			if (param.axis_dec_vec[i] > c->motionPool()[i].maxAcc() || param.axis_dec_vec[i] <= 0.0)
				THROW_FILE_AND_LINE("");
		}
	}

	struct EnableParam :public SetActiveMotor { std::int32_t limit_time; };
	struct Enable::Imp { Imp() {} };
	auto Enable::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		EnableParam param;

		set_check_option(params, target);
		set_active_motor(params, target, param);
		param.limit_time = std::stoi(params.at("limit_time"));

		target.param = param;
		target.option |= aris::plan::Plan::NOT_CHECK_OPERATION_ENABLE;
	}
	auto Enable::executeRT(PlanTarget &target)->int
	{
		auto &param = std::any_cast<EnableParam &>(target.param);

		bool is_all_finished = true;
		for (std::size_t i = 0; i < target.controller->motionPool().size(); ++i)
		{
			if (param.active_motor[i])
			{
				auto &cm = target.controller->motionPool().at(i);
				auto ret = cm.enable();
				if (ret)
				{
					is_all_finished = false;

					if (target.count % 1000 == 0)
					{
						target.controller->mout() << "Unenabled motor, slave id: " << cm.id() 
							<< ", absolute id: " << i << ", ret: " << ret << std::endl;
					}
				}
			}
		}

		return (is_all_finished || target.count >= param.limit_time) ? 0 : 1;
	}
	Enable::~Enable() = default;
	Enable::Enable(const std::string &name) :Plan(name), imp_(new Imp)
	{
		command().loadXmlStr(
			"<Command name=\"en\">"
			"	<GroupParam>"
			"		<Param name=\"limit_time\" default=\"5000\"/>"
					SELECT_MOTOR_STRING
					CHECK_PARAM_STRING
			"	</GroupParam>"
			"</Command>");
	}
	ARIS_DEFINE_BIG_FOUR_CPP(Enable);

	struct DisableParam :public SetActiveMotor { std::int32_t limit_time; };
	struct Disable::Imp { Imp() {} };
	auto Disable::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		DisableParam param;

		set_check_option(params, target);
		set_active_motor(params, target, param);
		param.limit_time = std::stoi(params.at("limit_time"));

		target.param = param;
		target.option |= aris::plan::Plan::NOT_CHECK_OPERATION_ENABLE;
	}
	auto Disable::executeRT(PlanTarget &target)->int
	{
		auto &param = std::any_cast<DisableParam &>(target.param);

		bool is_all_finished = true;
		for (std::size_t i = 0; i < target.controller->motionPool().size(); ++i)
		{
			if (param.active_motor[i])
			{
				auto &cm = target.controller->motionPool().at(i);
				auto ret = cm.disable();
				if (ret)
				{
					is_all_finished = false;

					if (target.count % 1000 == 0)
					{
						target.controller->mout() << "Undisabled motor, slave id: " << cm.id() 
							<< ", absolute id: " << i << ", ret: " << ret << std::endl;
					}
				}
			}
		}

		return (is_all_finished || target.count >= param.limit_time) ? 0 : 1;
	}
	Disable::~Disable() = default;
	Disable::Disable(const std::string &name) :Plan(name), imp_(new Imp)
	{
		command().loadXmlStr(
			"<Command name=\"ds\">"
			"	<GroupParam>"
			"		<Param name=\"limit_time\" default=\"5000\"/>"
					SELECT_MOTOR_STRING
					CHECK_PARAM_STRING
			"	</GroupParam>"
			"</Command>");
	}
	ARIS_DEFINE_BIG_FOUR_CPP(Disable);

	struct HomeParam :public SetActiveMotor 
	{ 
		std::int32_t limit_time;
		double offset;
	};
	struct Home::Imp { Imp() {} };
	auto Home::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		HomeParam param;

		set_active_motor(params, target, param);
		param.limit_time = std::stoi(params.at("limit_time"));

		for (aris::Size i = 0; i<param.active_motor.size(); ++i)
		{
			if (param.active_motor[i])
			{
				std::int8_t method = std::stoi(params.at(std::string("method")));
				if (method < 1 || method > 35) throw std::runtime_error("invalid home method");

				param.offset = std::stod(params.at(std::string("offset")));
				std::int32_t offset = std::stoi(params.at(std::string("offset")));
				std::uint32_t high_speed = std::stoi(params.at(std::string("high_speed")));
				std::uint32_t low_speed = std::stoi(params.at(std::string("low_speed")));
				std::uint32_t acc = std::stoi(params.at(std::string("acceleration")));

				auto &cm = dynamic_cast<aris::control::EthercatMotion &>(target.controller->motionPool()[i]);
			
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
	auto Home::executeRT(PlanTarget &target)->int
	{
		auto &param = std::any_cast<HomeParam &>(target.param);
		
		bool is_all_finished = true;
		for (std::size_t i = 0; i < target.controller->motionPool().size(); ++i)
		{
			if (param.active_motor[i])
			{
				auto &cm = target.controller->motionPool().at(i);
				auto ret = cm.home();
				if (ret)
				{
					is_all_finished = false;

					if (target.count % 1000 == 0)
					{
						target.controller->mout() << "Unhomed motor, slave id: " << cm.id()
							<< ", absolute id: " << i << ", ret: " << ret << std::endl;
					}
				}
				else
				{
					param.active_motor[i] = false;
				}
			}
		}

		return is_all_finished ? 0 : 1;
	}
	Home::~Home() = default;
	Home::Home(const std::string &name) :Plan(name), imp_(new Imp)
	{
		command().loadXmlStr(
			"<Command name=\"hm\">"
			"	<GroupParam>"
			"		<Param name=\"method\" default=\"35\"/>"
			"		<Param name=\"offset\" default=\"0\"/>"
			"		<Param name=\"high_speed\" default=\"20000\"/>"
			"		<Param name=\"low_speed\" default=\"20\"/>"
			"		<Param name=\"acceleration\" default=\"100000\"/>"
			"		<Param name=\"limit_time\" default=\"5000\"/>"
					SELECT_MOTOR_STRING
					CHECK_PARAM_STRING
			"	</GroupParam>"
			"</Command>");
	}
	ARIS_DEFINE_BIG_FOUR_CPP(Home);

	struct ModeParam :public SetActiveMotor { std::int32_t limit_time, mode; };
	struct Mode::Imp { Imp() {} };
	auto Mode::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		ModeParam param;

		set_check_option(params, target);
		set_active_motor(params, target, param);
		param.limit_time = std::stoi(params.at("limit_time"));
		param.mode = std::stoi(params.at("mode"));

		if (param.mode > 10 && param.mode < 8)throw std::runtime_error("invalid mode, aris now only support mode 8,9,10");

		target.param = param;
		target.option |= aris::plan::Plan::NOT_CHECK_OPERATION_ENABLE;
	}
	auto Mode::executeRT(PlanTarget &target)->int
	{
		auto &param = std::any_cast<ModeParam &>(target.param);

		bool is_all_finished = true;
		for (std::size_t i = 0; i < target.controller->motionPool().size(); ++i)
		{
			if (param.active_motor[i])
			{
				auto &cm = target.controller->motionPool().at(i);
				auto ret = cm.mode(param.mode);
				if (target.count == 1) 
				{
					switch (param.mode)
					{
					case 8:
						cm.setTargetPos(cm.actualPos());
						break;
					case 9:
						cm.setTargetVel(0.0);
						break;
					case 10:
						cm.setTargetCur(0.0);
						break;
					default:
						break;
					}
				}

				if (ret)
				{
					is_all_finished = false;

					if (target.count % 1000 == 0)
					{
						target.controller->mout() << "Unmoded motor, slave id: " << cm.id() 
							<< ", absolute id: " << i << ", ret: " << ret << std::endl;
					}
				}
			}
		}

		return (is_all_finished || target.count >= param.limit_time) ? 0 : 1;
	}
	Mode::~Mode() = default;
	Mode::Mode(const std::string &name) :Plan(name), imp_(new Imp)
	{
		command().loadXmlStr(
			"<Command name=\"md\">"
			"	<GroupParam>"
			"		<Param name=\"limit_time\" default=\"5000\"/>"
			"       <Param name=\"mode\" abbreviation=\"d\" default=\"8\"/>"
					SELECT_MOTOR_STRING
					CHECK_PARAM_STRING
			"	</GroupParam>"
			"</Command>");
	}
	ARIS_DEFINE_BIG_FOUR_CPP(Mode);

	struct ResetParam :public SetActiveMotor, SetInputMovement { std::vector<Size> total_count_vec; };
	struct Reset::Imp { Imp() {} };
	auto Reset::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		ResetParam param;
		set_check_option(params, target);
		set_active_motor(params, target, param);
		set_input_movement(params, target, param);

		for (Size i = 0; i < target.controller->motionPool().size(); ++i)
		{
			auto &cm = target.controller->motionPool()[i];
			param.axis_pos_vec[i] = param.axis_pos_vec[i] * (cm.maxPos() - cm.minPos()) + cm.minPos();
			param.axis_acc_vec[i] = param.axis_acc_vec[i] * cm.maxAcc();
			param.axis_vel_vec[i] = param.axis_vel_vec[i] * cm.maxVel();
			param.axis_dec_vec[i] = param.axis_dec_vec[i] * cm.maxAcc();
		}
		check_input_movement(params, target, param);

		param.total_count_vec.resize(target.controller->motionPool().size(), 1);

		target.option |=
			aris::plan::Plan::EXECUTE_WHEN_ALL_PLAN_COLLECTED |
			aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_AT_START |
			aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER_AT_START;

		target.param = param;
	}
	auto Reset::executeRT(PlanTarget &target)->int
	{
		auto &param = std::any_cast<ResetParam&>(target.param);

		// 取得起始位置 //
		if (target.count == 1)
		{
			for (Size i = 0; i < target.controller->motionPool().size(); ++i)
			{
				if (param.active_motor[i])
				{
					param.axis_begin_pos_vec[i] = target.controller->motionPool().at(i).actualPos();
				}
			}
		}

		// 设置驱动器的位置 //
		for (Size i = 0; i < target.controller->motionPool().size(); ++i)
		{
			if (param.active_motor[i])
			{
				double p, v, a;
				aris::plan::moveAbsolute(target.count, param.axis_begin_pos_vec[i], param.axis_pos_vec[i], param.axis_vel_vec[i] / 1000
					, param.axis_acc_vec[i] / 1000 / 1000, param.axis_dec_vec[i] / 1000 / 1000, p, v, a, param.total_count_vec[i]);
				target.controller->motionAtAbs(i).setTargetPos(p);
			}
		}

		return (static_cast<int>(*std::max_element(param.total_count_vec.begin(), param.total_count_vec.end())) > target.count) ? 1 : 0;
	}
	Reset::~Reset() = default;
	Reset::Reset(const std::string &name) :Plan(name), imp_(new Imp)
	{
		command().loadXmlStr(
			"<Command name=\"rs\">"
			"	<GroupParam>"
					SET_INPUT_MOVEMENT_STRING
					SELECT_MOTOR_STRING
					CHECK_PARAM_STRING
			"	</GroupParam>"
			"</Command>");
	}
	ARIS_DEFINE_BIG_FOUR_CPP(Reset);

	struct RecoverParam
	{
		std::atomic_bool is_kinematic_ready_;
		std::atomic_bool is_rt_waiting_ready_;
		std::future<void> fut;
		int kin_ret;
	};
	auto Recover::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		auto p = std::make_shared<RecoverParam>();

		p->is_kinematic_ready_ = false;
		p->is_rt_waiting_ready_ = false;
		p->fut = std::async(std::launch::async, [](std::shared_ptr<RecoverParam> p, PlanTarget &target)
		{
			// 等待正解求解的需求 //
			while (!p->is_rt_waiting_ready_.load())std::this_thread::sleep_for(std::chrono::milliseconds(1));

			// 求正解 //
			p->kin_ret = target.model->solverPool()[1].kinPos() ? 0 : -1;

			// 通知实时线程 //
			p->is_kinematic_ready_.store(true);
		}, p, std::ref(target));

		target.param = p;
		target.option |= 
			NOT_CHECK_POS_CONTINUOUS_AT_START | 
			NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER_AT_START |
			NOT_CHECK_OPERATION_ENABLE;
	}
	auto Recover::executeRT(PlanTarget &target)->int
	{
		auto param = std::any_cast<std::shared_ptr<RecoverParam> &>(target.param);

		if (target.count == 1)
		{
			for (Size i = 0; i < std::min(target.controller->motionPool().size(), target.model->motionPool().size()); ++i)
			{
				target.controller->motionPool()[i].setTargetPos(target.controller->motionPool().at(i).actualPos());
				target.model->motionPool()[i].setMp(target.controller->motionPool().at(i).actualPos());
			}

			param->is_rt_waiting_ready_.store(true);

			return 1;
		}

		return param->is_kinematic_ready_.load() ? param->kin_ret : 1;
	}
	auto Recover::collectNrt(PlanTarget &target)->void
	{
		if (target.count)
		{
			std::any_cast<std::shared_ptr<RecoverParam>&>(target.param)->fut.get();
		}
		else
		{
			// 此时前面指令出错，系统清理了该命令，这时设置一下 //
			std::any_cast<std::shared_ptr<RecoverParam>&>(target.param)->is_rt_waiting_ready_.store(true);
			std::any_cast<std::shared_ptr<RecoverParam>&>(target.param)->fut.get();
		}
	}
	Recover::~Recover() = default;
	Recover::Recover(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"rc\">"
			"</Command>");
	}
	ARIS_DEFINE_BIG_FOUR_CPP(Recover);

	struct SleepParam { int count; };
	struct Sleep::Imp {};
	auto Sleep::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		target.param = SleepParam{ std::stoi(params.at("count")) };
		target.option |= NOT_CHECK_OPERATION_ENABLE;
	}
	auto Sleep::executeRT(PlanTarget &target)->int { return std::any_cast<SleepParam&>(target.param).count - target.count; }
	Sleep::~Sleep() = default;
	Sleep::Sleep(const std::string &name) :Plan(name), imp_(new Imp)
	{
		command().loadXmlStr(
			"<Command name=\"sl\">"
			"	<Param name=\"count\" default=\"1000\" abbreviation=\"c\"/>"
			"</Command>");
	}
	ARIS_DEFINE_BIG_FOUR_CPP(Sleep);

	auto Show::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		target.option |=
			NOT_CHECK_OPERATION_ENABLE |
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
		target.controller->mout() << "pos: ";
		for (auto &m : target.controller->motionPool())
		{
			target.controller->mout() << std::setprecision(15) << m.actualPos() << "   ";
		}
		target.controller->mout() << std::endl;

		return 0;
	}
	Show::Show(const std::string &name) : Plan(name)
	{
		command().loadXmlStr("<Command name=\"sh\"/>");
	}
	ARIS_DEFINE_BIG_FOUR_CPP(Show);

	struct MoveAbsJParam :public SetActiveMotor, SetInputMovement{};
	auto MoveAbsJ::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		MoveAbsJParam param;

		set_check_option(params, target);
		set_active_motor(params, target, param);
		set_input_movement(params, target, param);
		check_input_movement(params, target, param);

		param.axis_begin_pos_vec.resize(target.controller->motionPool().size());
		target.param = param;
	}
	auto MoveAbsJ::executeRT(PlanTarget &target)->int
	{
		auto param = std::any_cast<MoveAbsJParam>(&target.param);

		if (target.count == 1)
		{
			for (Size i = 0; i < param->active_motor.size(); ++i)
			{
				if (param->active_motor[i])
				{
					param->axis_begin_pos_vec[i] = target.controller->motionPool()[i].targetPos();
				}
			}
		}

		aris::Size total_count{ 1 };
		for (Size i = 0; i < param->active_motor.size(); ++i)
		{
			if (param->active_motor[i])
			{
				double p, v, a;
				aris::Size t_count;
				aris::plan::moveAbsolute(target.count,
					param->axis_begin_pos_vec[i], param->axis_pos_vec[i],
					param->axis_vel_vec[i] / 1000, param->axis_acc_vec[i] / 1000 / 1000, param->axis_dec_vec[i] / 1000 / 1000,
					p, v, a, t_count);
				target.controller->motionPool()[i].setTargetPos(p);
				total_count = std::max(total_count, t_count);
			}
		}

		return total_count - target.count;
	}
	struct MoveAbsJ::Imp {};
	MoveAbsJ::~MoveAbsJ() = default;
	MoveAbsJ::MoveAbsJ(const std::string &name) : Plan(name), imp_(new Imp)
	{
		command().loadXmlStr(
			"<Command name=\"mvaj\">"
			"	<GroupParam>"
			"		<Param name=\"pos\" default=\"0.0\"/>"
			"		<Param name=\"vel\" default=\"1.0\"/>"
			"		<Param name=\"acc\" default=\"1.0\"/>"
			"		<Param name=\"dec\" default=\"1.0\"/>"
					SELECT_MOTOR_STRING
					CHECK_PARAM_STRING
			"	</GroupParam>"
			"</Command>");
	}
	ARIS_DEFINE_BIG_FOUR_CPP(MoveAbsJ);

	auto check_eul_validity(const std::string &eul_type)->bool
	{
		if (eul_type.size()<3)return false;

		for (int i = 0; i < 3; ++i)if (eul_type[i] > '3' || eul_type[i] < '1')return false;

		if (eul_type[0] == eul_type[1] || eul_type[1] == eul_type[2]) return false;

		return true;
	}
	auto find_pq(const std::map<std::string, std::string> &params, PlanTarget &target, double *pq_out)->bool
	{
		double pos_unit;
		auto pos_unit_found = params.find("pos_unit");
		if (pos_unit_found == params.end()) pos_unit = 1.0;
		else if (pos_unit_found->second == "m")pos_unit = 1.0;
		else if (pos_unit_found->second == "mm")pos_unit = 0.001;
		else if (pos_unit_found->second == "cm")pos_unit = 0.01;
		else THROW_FILE_AND_LINE("");

		for (auto cmd_param : params)
		{
			if (cmd_param.first == "pq")
			{
				auto pq_mat = target.model->calculator().calculateExpression(cmd_param.second);
				if (pq_mat.size() != 7)THROW_FILE_AND_LINE("");
				aris::dynamic::s_vc(7, pq_mat.data(), pq_out);
				aris::dynamic::s_nv(3, pos_unit, pq_out);
				return true;
			}
			else if (cmd_param.first == "pm")
			{
				auto pm_mat = target.model->calculator().calculateExpression(cmd_param.second);
				if (pm_mat.size() != 16)THROW_FILE_AND_LINE("");
				aris::dynamic::s_pm2pq(pm_mat.data(), pq_out);
				aris::dynamic::s_nv(3, pos_unit, pq_out);
				return true;
			}
			else if (cmd_param.first == "pe")
			{
				double ori_unit;
				auto ori_unit_found = params.find("ori_unit");
				if (ori_unit_found == params.end()) ori_unit = 1.0;
				else if (ori_unit_found->second == "rad")ori_unit = 1.0;
				else if (ori_unit_found->second == "degree")ori_unit = PI / 180.0;
				else THROW_FILE_AND_LINE("");

				std::string eul_type;
				auto eul_type_found = params.find("eul_type");
				if (eul_type_found == params.end()) eul_type = "321";
				else if (check_eul_validity(eul_type_found->second.data()))	eul_type = eul_type_found->second;
				else THROW_FILE_AND_LINE("");

				auto pe_mat = target.model->calculator().calculateExpression(cmd_param.second);
				if (pe_mat.size() != 6)THROW_FILE_AND_LINE("");
				aris::dynamic::s_nv(3, ori_unit, pe_mat.data() + 3);
				aris::dynamic::s_pe2pq(pe_mat.data(), pq_out, eul_type.data());
				aris::dynamic::s_nv(3, pos_unit, pq_out);
				return true;
			}
		}

		THROW_FILE_AND_LINE("No pose input");
	}
	struct MoveJParam
	{
		std::vector<double> joint_vel, joint_acc, joint_dec, ee_pq, joint_pos_begin, joint_pos_end;
		std::vector<Size> total_count;
	};
	struct MoveJ::Imp {};
	auto MoveJ::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		set_check_option(params, target);

		MoveJParam mvj_param;

		// find ee pq //
		mvj_param.ee_pq.resize(7);
		find_pq(params, target, mvj_param.ee_pq.data());

		mvj_param.joint_pos_begin.resize(target.model->motionPool().size(), 0.0);
		mvj_param.joint_pos_end.resize(target.model->motionPool().size(), 0.0);
		mvj_param.total_count.resize(target.model->motionPool().size(), 0);

		// find joint acc/vel/dec/
		for (auto cmd_param : params)
		{
			auto c = target.controller;
			if (cmd_param.first == "joint_acc")
			{
				mvj_param.joint_acc.clear();
				mvj_param.joint_acc.resize(target.model->motionPool().size(), 0.0);

				auto acc_mat = target.model->calculator().calculateExpression(cmd_param.second);
				if (acc_mat.size() == 1)std::fill(mvj_param.joint_acc.begin(), mvj_param.joint_acc.end(), acc_mat.toDouble());
				else if (acc_mat.size() == target.model->motionPool().size()) std::copy(acc_mat.begin(), acc_mat.end(), mvj_param.joint_acc.begin());
				else THROW_FILE_AND_LINE("");

				// check value validity //
				for (Size i = 0; i< std::min(target.model->motionPool().size(), c->motionPool().size()); ++i)
					if (mvj_param.joint_acc[i] <= 0 || mvj_param.joint_acc[i] > c->motionPool()[i].maxAcc())
						THROW_FILE_AND_LINE("");
			}
			else if (cmd_param.first == "joint_vel")
			{
				mvj_param.joint_vel.clear();
				mvj_param.joint_vel.resize(target.model->motionPool().size(), 0.0);

				auto vel_mat = target.model->calculator().calculateExpression(cmd_param.second);
				if (vel_mat.size() == 1)std::fill(mvj_param.joint_vel.begin(), mvj_param.joint_vel.end(), vel_mat.toDouble());
				else if (vel_mat.size() == target.model->motionPool().size()) std::copy(vel_mat.begin(), vel_mat.end(), mvj_param.joint_vel.begin());
				else THROW_FILE_AND_LINE("");

				// check value validity //
				for (Size i = 0; i< std::min(target.model->motionPool().size(), c->motionPool().size()); ++i)
					if (mvj_param.joint_vel[i] <= 0 || mvj_param.joint_vel[i] > c->motionPool()[i].maxAcc())
						THROW_FILE_AND_LINE("");
			}
			else if (cmd_param.first == "joint_dec")
			{
				mvj_param.joint_dec.clear();
				mvj_param.joint_dec.resize(target.model->motionPool().size(), 0.0);

				auto dec_mat = target.model->calculator().calculateExpression(cmd_param.second);
				if (dec_mat.size() == 1)std::fill(mvj_param.joint_dec.begin(), mvj_param.joint_dec.end(), dec_mat.toDouble());
				else if (dec_mat.size() == target.model->motionPool().size()) std::copy(dec_mat.begin(), dec_mat.end(), mvj_param.joint_dec.begin());
				else THROW_FILE_AND_LINE("");

				// check value validity //
				for (Size i = 0; i< std::min(target.model->motionPool().size(), c->motionPool().size()); ++i)
					if (mvj_param.joint_dec[i] <= 0 || mvj_param.joint_dec[i] > c->motionPool()[i].maxAcc())
						THROW_FILE_AND_LINE("");
			}
		}

		target.param = mvj_param;
	}
	auto MoveJ::executeRT(PlanTarget &target)->int
	{
		auto mvj_param = std::any_cast<MoveJParam>(&target.param);
		auto controller = target.controller;

		// 取得起始位置 //
		double p, v, a;
		static Size max_total_count;
		if (target.count == 1)
		{
			// inverse kinematic //
			double end_pm[16];
			aris::dynamic::s_pq2pm(mvj_param->ee_pq.data(), end_pm);
			target.model->generalMotionPool().at(0).setMpm(end_pm);
			if (!target.model->solverPool().at(0).kinPos())return -1;

			// init joint_pos //
			for (Size i = 0; i < std::min(controller->motionPool().size(), target.model->motionPool().size()); ++i)
			{
				mvj_param->joint_pos_begin[i] = controller->motionPool()[i].targetPos();
				mvj_param->joint_pos_end[i] = target.model->motionPool()[i].mp();
				aris::plan::moveAbsolute(target.count, mvj_param->joint_pos_begin[i], mvj_param->joint_pos_end[i]
					, mvj_param->joint_vel[i] / 1000, mvj_param->joint_acc[i] / 1000 / 1000, mvj_param->joint_dec[i] / 1000 / 1000
					, p, v, a, mvj_param->total_count[i]);
			}

			max_total_count = *std::max_element(mvj_param->total_count.begin(), mvj_param->total_count.end());
		}

		for (Size i = 0; i < std::min(controller->motionPool().size(), target.model->motionPool().size()); ++i)
		{
			aris::plan::moveAbsolute(static_cast<double>(target.count) * mvj_param->total_count[i] / max_total_count,
				mvj_param->joint_pos_begin[i], mvj_param->joint_pos_end[i],
				mvj_param->joint_vel[i] / 1000, mvj_param->joint_acc[i] / 1000 / 1000, mvj_param->joint_dec[i] / 1000 / 1000,
				p, v, a, mvj_param->total_count[i]);

			controller->motionPool()[i].setTargetPos(p);
		}

		return max_total_count == 0 ? 0 : max_total_count - target.count;
	}
	MoveJ::~MoveJ() = default;
	MoveJ::MoveJ(const std::string &name) :Plan(name), imp_(new Imp)
	{
		command().loadXmlStr(
			"<Command name=\"mvj\">"
			"	<GroupParam>"
			"		<Param name=\"pos_unit\" default=\"m\"/>"
			"		<UniqueParam default=\"pq\">"
			"			<Param name=\"pq\" default=\"{0,0,0,0,0,0,1}\"/>"
			"			<Param name=\"pm\" default=\"{1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1}\"/>"
			"			<GroupParam>"
			"				<Param name=\"pe\" default=\"{0,0,0,0,0,0}\"/>"
			"				<Param name=\"ori_unit\" default=\"rad\"/>"
			"				<Param name=\"eul_type\" default=\"321\"/>"
			"			</GroupParam>"
			"		</UniqueParam>"
			"		<Param name=\"joint_acc\" default=\"0.1\"/>"
			"		<Param name=\"joint_vel\" default=\"0.1\"/>"
			"		<Param name=\"joint_dec\" default=\"0.1\"/>"
					CHECK_PARAM_STRING
			"	</GroupParam>"
			"</Command>");
	}
	ARIS_DEFINE_BIG_FOUR_CPP(MoveJ);

	struct MoveLParam
	{
		std::vector<double> joint_vel, joint_acc, joint_dec, ee_pq, joint_pos_begin, joint_pos_end;
		Size total_count[6];

		double acc, vel, dec;
		double angular_acc, angular_vel, angular_dec;
	};
	struct MoveL::Imp {};
	auto MoveL::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		set_check_option(params, target);

		MoveLParam mvl_param;
		mvl_param.ee_pq.resize(7);
		if (!find_pq(params, target, mvl_param.ee_pq.data()))THROW_FILE_AND_LINE("");

		for (auto cmd_param : params)
		{
			if (cmd_param.first == "acc")
			{
				mvl_param.acc = std::stod(cmd_param.second);
			}
			else if (cmd_param.first == "vel")
			{
				mvl_param.vel = std::stod(cmd_param.second);
			}
			else if (cmd_param.first == "dec")
			{
				mvl_param.dec = std::stod(cmd_param.second);
			}
			else if (cmd_param.first == "angular_acc")
			{
				mvl_param.angular_acc = std::stod(cmd_param.second);
			}
			else if (cmd_param.first == "angular_vel")
			{
				mvl_param.angular_vel = std::stod(cmd_param.second);
			}
			else if (cmd_param.first == "angular_dec")
			{
				mvl_param.angular_dec = std::stod(cmd_param.second);
			}
		}

		target.option |= USE_TARGET_POS;
		target.param = mvl_param;
	}
	auto MoveL::executeRT(PlanTarget &target)->int
	{
		auto mvl_param = std::any_cast<MoveLParam>(&target.param);
		auto controller = target.controller;

		// 取得起始位置 //
		static double begin_pm[16], relative_pm[16], relative_pa[6], pos_ratio, ori_ratio, norm_pos, norm_ori;
		double p, v, a;
		aris::Size pos_total_count, ori_total_count;
		if (target.count == 1)
		{
			double end_pm[16];
			aris::dynamic::s_pq2pm(mvl_param->ee_pq.data(), end_pm);
			target.model->generalMotionPool().at(0).updMpm();
			target.model->generalMotionPool().at(0).getMpm(begin_pm);
			aris::dynamic::s_inv_pm_dot_pm(begin_pm, end_pm, relative_pm);

			// relative_pa //
			aris::dynamic::s_pm2pa(relative_pm, relative_pa);

			norm_pos = aris::dynamic::s_norm(3, relative_pa);
			norm_ori = aris::dynamic::s_norm(3, relative_pa + 3);

			aris::plan::moveAbsolute(target.count, 0.0, norm_pos, mvl_param->vel / 1000, mvl_param->acc / 1000 / 1000, mvl_param->dec / 1000 / 1000, p, v, a, pos_total_count);
			aris::plan::moveAbsolute(target.count, 0.0, norm_ori, mvl_param->angular_vel / 1000, mvl_param->angular_acc / 1000 / 1000, mvl_param->angular_dec / 1000 / 1000, p, v, a, ori_total_count);

			pos_ratio = pos_total_count < ori_total_count ? double(pos_total_count) / ori_total_count : 1.0;
			ori_ratio = ori_total_count < pos_total_count ? double(ori_total_count) / pos_total_count : 1.0;

			aris::plan::moveAbsolute(target.count, 0.0, norm_pos, mvl_param->vel / 1000 * pos_ratio, mvl_param->acc / 1000 / 1000 * pos_ratio* pos_ratio, mvl_param->dec / 1000 / 1000 * pos_ratio* pos_ratio, p, v, a, pos_total_count);
			aris::plan::moveAbsolute(target.count, 0.0, norm_ori, mvl_param->angular_vel / 1000 * ori_ratio, mvl_param->angular_acc / 1000 / 1000 * ori_ratio * ori_ratio, mvl_param->angular_dec / 1000 / 1000 * ori_ratio * ori_ratio, p, v, a, ori_total_count);
		}

		double pa[6]{ 0,0,0,0,0,0 }, pm[16], pm2[16];

		aris::plan::moveAbsolute(target.count, 0.0, norm_pos, mvl_param->vel / 1000 * pos_ratio, mvl_param->acc / 1000 / 1000 * pos_ratio* pos_ratio, mvl_param->dec / 1000 / 1000 * pos_ratio* pos_ratio, p, v, a, pos_total_count);
		if (norm_pos > 1e-10)aris::dynamic::s_vc(3, p / norm_pos, relative_pa, pa);

		aris::plan::moveAbsolute(target.count, 0.0, norm_ori, mvl_param->angular_vel / 1000 * ori_ratio, mvl_param->angular_acc / 1000 / 1000 * ori_ratio * ori_ratio, mvl_param->angular_dec / 1000 / 1000 * ori_ratio * ori_ratio, p, v, a, ori_total_count);
		if (norm_ori > 1e-10)aris::dynamic::s_vc(3, p / norm_ori, relative_pa + 3, pa + 3);

		aris::dynamic::s_pa2pm(pa, pm);
		aris::dynamic::s_pm_dot_pm(begin_pm, pm, pm2);

		// 反解计算电机位置 //
		target.model->generalMotionPool().at(0).setMpm(pm2);
		if (!target.model->solverPool().at(0).kinPos())return -1;

		////////////////////////////////////// log ///////////////////////////////////////
		double pq[7];
		aris::dynamic::s_pm2pq(*target.model->generalMotionPool().at(0).mpm(), pq);
		target.controller->lout() << target.count << " " << pq[0] << " " << pq[1] << " " << pq[2] << " " << pq[3] << " " << pq[4] << " " << pq[5] << " " << pq[6] << "  ";

		for (auto &cm : controller->motionPool())
		{
			target.controller->lout() << "  " << cm.targetPos() << "  " << cm.actualPos() << "  " << cm.actualVel() << "  " << cm.actualCur() << "  ";
		}
		target.controller->lout() << "\n";
		//////////////////////////////////////////////////////////////////////////////////


		return std::max(pos_total_count, ori_total_count) > target.count ? 1 : 0;
	}
	MoveL::~MoveL() = default;
	MoveL::MoveL(const std::string &name) :Plan(name), imp_(new Imp)
	{
		command().loadXmlStr(
			"<Command name=\"mvl\">"
			"	<GroupParam>"
			"		<Param name=\"pos_unit\" default=\"m\"/>"
			"		<UniqueParam default=\"pq\">"
			"			<Param name=\"pq\" default=\"{0,0,0,0,0,0,1}\"/>"
			"			<Param name=\"pm\" default=\"{1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1}\"/>"
			"			<GroupParam>"
			"				<Param name=\"pe\" default=\"{0,0,0,0,0,0}\"/>"
			"				<Param name=\"ori_unit\" default=\"rad\"/>"
			"				<Param name=\"eul_type\" default=\"321\"/>"
			"			</GroupParam>"
			"		</UniqueParam>"
			"		<Param name=\"acc\" default=\"0.1\"/>"
			"		<Param name=\"vel\" default=\"0.1\"/>"
			"		<Param name=\"dec\" default=\"0.1\"/>"
			"		<Param name=\"angular_acc\" default=\"0.1\"/>"
			"		<Param name=\"angular_vel\" default=\"0.1\"/>"
			"		<Param name=\"angular_dec\" default=\"0.1\"/>"
					CHECK_PARAM_STRING
			"	</GroupParam>"
			"</Command>");
	}
	ARIS_DEFINE_BIG_FOUR_CPP(MoveL);

	struct AutoMoveParam {};
	struct AutoMove::Imp
	{
		static std::atomic_bool is_running_;
		static std::atomic<std::array<double, 24>> pvade_;

		double max_pe_[6], min_pe_[6];
		std::string eul_type;
	};
	std::atomic_bool AutoMove::Imp::is_running_ = false;
	std::atomic<std::array<double, 24>> AutoMove::Imp::pvade_ = std::array<double, 24>{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	auto AutoMove::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		set_check_option(params, target);

		target.option = 0;

		AutoMoveParam param;
		for (auto cmd_param : params)
		{
			if (cmd_param.first == "start")
			{
				if (Imp::is_running_.load())throw std::runtime_error("auto mode already started");

				imp_->eul_type = params.at("eul_type");
				if (!check_eul_validity(imp_->eul_type))THROW_FILE_AND_LINE("");

				auto mat = target.model->calculator().calculateExpression(params.at("max_pe"));
				if (mat.size() != 6)THROW_FILE_AND_LINE("");
				std::copy(mat.begin(), mat.end(), imp_->max_pe_);

				mat = target.model->calculator().calculateExpression(params.at("min_pe"));
				if (mat.size() != 6)THROW_FILE_AND_LINE("");
				std::copy(mat.begin(), mat.end(), imp_->min_pe_);

				std::array<double, 24> pvade;
				mat = target.model->calculator().calculateExpression(params.at("init_pe"));
				if (mat.size() != 6)THROW_FILE_AND_LINE("");
				std::copy(mat.begin(), mat.end(), pvade.begin() + 0);

				mat = target.model->calculator().calculateExpression(params.at("init_ve"));
				if (mat.size() != 6)THROW_FILE_AND_LINE("");
				std::copy(mat.begin(), mat.end(), pvade.begin() + 6);

				mat = target.model->calculator().calculateExpression(params.at("init_ae"));
				if (mat.size() != 6)THROW_FILE_AND_LINE("");
				std::copy(mat.begin(), mat.end(), pvade.begin() + 12);

				mat = target.model->calculator().calculateExpression(params.at("init_de"));
				if (mat.size() != 6)THROW_FILE_AND_LINE("");
				std::copy(mat.begin(), mat.end(), pvade.begin() + 18);

				Imp::pvade_.store(pvade);
				Imp::is_running_.store(true);
				target.option |= EXECUTE_WHEN_ALL_PLAN_COLLECTED | NOT_PRINT_EXECUTE_COUNT | USE_TARGET_POS;
			}
			else if (cmd_param.first == "stop")
			{
				if (!Imp::is_running_.load())throw std::runtime_error("auto mode not started, when stop");

				Imp::is_running_.store(false);
				target.option |= aris::plan::Plan::WAIT_FOR_COLLECTION;
			}
			else if (cmd_param.first == "pe")
			{
				if (!Imp::is_running_.load())throw std::runtime_error("auto mode not started, when pe");

				std::array<double, 24> pvade;
				auto mat = target.model->calculator().calculateExpression(params.at("pe"));
				if (mat.size() != 6)THROW_FILE_AND_LINE("");
				std::copy(mat.begin(), mat.end(), pvade.begin() + 0);

				mat = target.model->calculator().calculateExpression(params.at("ve"));
				if (mat.size() != 6)THROW_FILE_AND_LINE("");
				std::copy(mat.begin(), mat.end(), pvade.begin() + 6);

				mat = target.model->calculator().calculateExpression(params.at("ae"));
				if (mat.size() != 6)THROW_FILE_AND_LINE("");
				std::copy(mat.begin(), mat.end(), pvade.begin() + 12);

				mat = target.model->calculator().calculateExpression(params.at("de"));
				if (mat.size() != 6)THROW_FILE_AND_LINE("");
				std::copy(mat.begin(), mat.end(), pvade.begin() + 18);

				for (int i = 0; i < 6; ++i)
				{
					pvade[i] = std::max(imp_->min_pe_[i], pvade[i]);
					pvade[i] = std::min(imp_->max_pe_[i], pvade[i]);
				}

				AutoMove::Imp::pvade_.store(pvade);
				target.option |= NOT_RUN_EXECUTE_FUNCTION | NOT_RUN_COLLECT_FUNCTION | NOT_PRINT_CMD_INFO | NOT_LOG_CMD_INFO;
			}
		}

		target.option |= NOT_CHECK_POS_FOLLOWING_ERROR;
		target.param = param;
	}
	auto AutoMove::executeRT(PlanTarget &target)->int
	{
		auto param = std::any_cast<AutoMoveParam>(&target.param);

		if (target.count == 1)
		{
			target.model->generalMotionPool()[0].setMve(std::array<double, 6>{0, 0, 0, 0, 0, 0}.data(), "123");
		}

		// get current pe //
		double pe_now[6], ve_now[6], ae_now[6];
		target.model->generalMotionPool()[0].getMpe(pe_now, imp_->eul_type.c_str());
		target.model->generalMotionPool()[0].getMve(ve_now, imp_->eul_type.c_str());
		target.model->generalMotionPool()[0].getMae(ae_now, imp_->eul_type.c_str());
		for (int i = 3; i < 6; ++i)if (pe_now[i] > aris::PI) pe_now[i] -= 2 * PI;

		// get target pe //
		auto pvade = AutoMove::Imp::pvade_.load();
		auto pe = pvade.data() + 0;
		auto ve = pvade.data() + 6;
		auto ae = pvade.data() + 12;
		auto de = pvade.data() + 18;

		// now plan //
		double pe_next[6], ve_next[6], ae_next[6];
		for (int i = 0; i < 6; ++i)
		{
			aris::Size t;
			aris::plan::moveAbsolute2(pe_now[i], ve_now[i], ae_now[i]
				, pe[i], 0.0, 0.0
				, ve[i], ae[i], de[i]
				, 1e-3, 1e-10, pe_next[i], ve_next[i], ae_next[i], t);
		}

		target.model->generalMotionPool()[0].setMpe(pe_next, imp_->eul_type.c_str());
		target.model->generalMotionPool()[0].setMve(ve_next, imp_->eul_type.c_str());
		target.model->generalMotionPool()[0].setMae(ae_next, imp_->eul_type.c_str());

		static int i = 0;
		if (++i % 1000 == 0)
		{
			//target.controller->mout() << "pe_now :"
			//	<< pe_now[0] << "  " << pe_now[1] << "  " << pe_now[2] << "  "
			//	<< pe_now[3] << "  " << pe_now[4] << "  " << pe_now[5] << std::endl;

			//target.controller->mout() << "pe_target :"
			//	<< pvade[0] << "  " << pvade[1] << "  " << pvade[2] << "  "
			//	<< pvade[3] << "  " << pvade[4] << "  " << pvade[5] << std::endl;

			//target.controller->mout() << "pe_next:"
			//	<< pe_next[0] << "  " << pe_next[1] << "  " << pe_next[2] << "  "
			//	<< pe_next[3] << "  " << pe_next[4] << "  " << pe_next[5] << std::endl;
		}

		target.model->solverPool()[0].kinPos();
		target.model->solverPool()[0].kinVel();

		return imp_->is_running_.load() ? 1 : 0;
	}
	auto AutoMove::collectNrt(PlanTarget &target)->void { if (~(target.option | USE_TARGET_POS))Imp::is_running_.store(false); }
	AutoMove::~AutoMove() = default;
	AutoMove::AutoMove(const std::string &name) : Plan(name), imp_(new Imp)
	{
		command().loadXmlStr(
			"<Command name=\"am\">"
			"	<GroupParam>"
			"		<UniqueParam default=\"start_group\">"
			"			<GroupParam name=\"start_group\">"
			"				<Param name=\"start\"/>"
			"				<Param name=\"init_pe\" default=\"{0,0,0,0,0,0}\"/>"
			"				<Param name=\"init_ve\" default=\"{0.1,0.1,0.1,0.1,0.1,0.1}\"/>"
			"				<Param name=\"init_ae\" default=\"{0.1,0.1,0.1,0.1,0.1,0.1}\"/>"
			"				<Param name=\"init_de\" default=\"{0.1,0.1,0.1,0.1,0.1,0.1}\"/>"
			"				<Param name=\"max_pe\" default=\"{0,0,0,0,0,0}\"/>"
			"				<Param name=\"min_pe\" default=\"{0,0,0,0,0,0}\"/>"
			"				<Param name=\"eul_type\" default=\"321\"/>"
			"			</GroupParam>"
			"			<Param name=\"stop\"/>"
			"			<GroupParam>"
			"				<Param name=\"pe\" default=\"{0,0,0,0,0,0}\"/>"
			"				<Param name=\"ve\" default=\"{0.1,0.1,0.1,0.1,0.1,0.1}\"/>"
			"				<Param name=\"ae\" default=\"{0.1,0.1,0.1,0.1,0.1,0.1}\"/>"
			"				<Param name=\"de\" default=\"{0.1,0.1,0.1,0.1,0.1,0.1}\"/>"
			"			</GroupParam>"
			"		</UniqueParam>"
					CHECK_PARAM_STRING
			"	</GroupParam>"
			"</Command>");
	}
	ARIS_DEFINE_BIG_FOUR_CPP(AutoMove);

	struct ManualMoveParam {};
	struct ManualMove::Imp
	{
		static std::atomic_bool is_running_;
		static std::atomic<std::array<int, 6>> is_increase_;
		double ve_[6], ae_[6], de_[6];
		std::string eul_type;
		int increase_count;
	};
	std::atomic_bool ManualMove::Imp::is_running_ = false;
	std::atomic<std::array<int, 6>> ManualMove::Imp::is_increase_ = std::array<int, 6>{0, 0, 0, 0, 0, 0};
	auto ManualMove::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		set_check_option(params, target);

		target.option = 0;

		ManualMoveParam param;
		for (auto cmd_param : params)
		{
			if (cmd_param.first == "start")
			{
				if (Imp::is_running_.load())throw std::runtime_error("auto mode already started");

				imp_->eul_type = params.at("eul_type");
				if (!check_eul_validity(imp_->eul_type))THROW_FILE_AND_LINE("");

				imp_->increase_count = std::stoi(params.at("increase_count"));
				if (imp_->increase_count < 0 || imp_->increase_count>1e5)THROW_FILE_AND_LINE("");

				auto mat = target.model->calculator().calculateExpression(params.at("ve"));
				if (mat.size() != 6)THROW_FILE_AND_LINE("");
				std::copy(mat.begin(), mat.end(), imp_->ve_);

				mat = target.model->calculator().calculateExpression(params.at("ae"));
				if (mat.size() != 6)THROW_FILE_AND_LINE("");
				std::copy(mat.begin(), mat.end(), imp_->ae_);

				mat = target.model->calculator().calculateExpression(params.at("de"));
				if (mat.size() != 6)THROW_FILE_AND_LINE("");
				std::copy(mat.begin(), mat.end(), imp_->de_);

				Imp::is_increase_.store(std::array<int, 6>{0, 0, 0, 0, 0, 0});
				Imp::is_running_.store(true);
				target.option |= EXECUTE_WHEN_ALL_PLAN_COLLECTED | NOT_PRINT_EXECUTE_COUNT | USE_TARGET_POS;
			}
			else if (cmd_param.first == "stop")
			{
				if (!Imp::is_running_.load())throw std::runtime_error("manual mode not started, when stop");

				Imp::is_running_.store(false);
				target.option |= WAIT_FOR_COLLECTION;
			}
			else if (cmd_param.first == "x")
			{
				if (!Imp::is_running_.load())throw std::runtime_error("manual mode not started, when pe");

				std::array<int, 6> is_increase;
				is_increase[0] = std::stoi(params.at("x"));
				is_increase[1] = std::stoi(params.at("y"));
				is_increase[2] = std::stoi(params.at("z"));
				is_increase[3] = std::stoi(params.at("a"));
				is_increase[4] = std::stoi(params.at("b"));
				is_increase[5] = std::stoi(params.at("c"));

				for (auto &value : is_increase)value = std::max(std::min(1, value), -1) * imp_->increase_count;

				Imp::is_increase_.store(is_increase);
				target.option |= NOT_RUN_EXECUTE_FUNCTION | NOT_RUN_COLLECT_FUNCTION | NOT_PRINT_CMD_INFO | NOT_LOG_CMD_INFO;
			}
		}

		target.option |= NOT_CHECK_POS_FOLLOWING_ERROR;
		target.param = param;
	}
	auto ManualMove::executeRT(PlanTarget &target)->int
	{
		auto param = std::any_cast<ManualMoveParam>(&target.param);

		// get current pe //
		double pe_now[6], ve_now[6], ae_now[6];
		target.model->generalMotionPool()[0].getMpe(pe_now, imp_->eul_type.c_str());
		target.model->generalMotionPool()[0].getMve(ve_now, imp_->eul_type.c_str());
		target.model->generalMotionPool()[0].getMae(ae_now, imp_->eul_type.c_str());
		for (int i = 3; i < 6; ++i) if (pe_now[i] > aris::PI) pe_now[i] -= 2 * PI;

		// init status //
		static std::array<int, 6> increase_status;
		static std::array<double, 6> target_pe{ 0,0,0,0,0,0 };
		if (target.count == 1)
		{
			std::fill_n(increase_status.data(), 6, 0);
			std::copy_n(pe_now, 6, target_pe.data());
			std::fill_n(ve_now, 6, 0.0);
			std::fill_n(ae_now, 6, 0.0);
		}

		// get is_increase //
		auto is_increase = Imp::is_increase_.exchange(std::array<int, 6>{0, 0, 0, 0, 0, 0});
		for (int i = 0; i < 6; ++i) if (is_increase[i] != 0) increase_status[i] = is_increase[i];

		// calculate target pe //
		std::array<double, 6> target_pe_new;
		for (int i = 0; i < 6; ++i)
		{
			target_pe_new[i] = target_pe[i] + aris::dynamic::s_sgn(increase_status[i])*imp_->ve_[i] * 1e-3;
			increase_status[i] -= aris::dynamic::s_sgn(increase_status[i]);
		}

		// check if target_pe is valid //
		target.model->generalMotionPool()[0].setMpe(target_pe_new.data(), imp_->eul_type.c_str());
		auto check_motion_limit = [&]()->bool
		{
			auto c = target.controller;
			for (std::size_t i = 0; i < std::min(c->motionPool().size(), target.model->motionPool().size()); ++i)
			{
				auto &cm = c->motionPool().at(i);
				auto &mm = target.model->motionPool().at(i);

				if (mm.mp() < cm.minPos() + 0.005 || mm.mp() > cm.maxPos() - 0.005) return false;
			}
			return true;
		};
		if (target.model->solverPool()[0].kinPos() && check_motion_limit()) std::swap(target_pe, target_pe_new);

		// calculate real value //
		double pe_next[6], ve_next[6], ae_next[6];
		for (int i = 0; i < 6; ++i)
		{
			aris::Size t;
			aris::plan::moveAbsolute2(pe_now[i], ve_now[i], ae_now[i]
				, target_pe[i], 0.0, 0.0
				, imp_->ve_[i], imp_->ae_[i], imp_->de_[i]
				, 1e-3, 1e-10, pe_next[i], ve_next[i], ae_next[i], t);
		}

		// set everything //
		target.model->generalMotionPool()[0].setMpe(pe_next, imp_->eul_type.c_str());
		target.model->generalMotionPool()[0].setMve(ve_next, imp_->eul_type.c_str());
		target.model->generalMotionPool()[0].setMae(ae_next, imp_->eul_type.c_str());
		target.model->solverPool()[0].kinPos();
		target.model->solverPool()[0].kinVel();

		return imp_->is_running_.load() ? 1 : 0;
	}
	auto ManualMove::collectNrt(PlanTarget &target)-> void
	{ 
		if (~(target.option | USE_TARGET_POS))Imp::is_running_.store(false); 
	}
	ManualMove::~ManualMove() = default;
	ManualMove::ManualMove(const std::string &name) : Plan(name), imp_(new Imp)
	{
		command().loadXmlStr(
			"<Command name=\"mm\">"
			"	<GroupParam>"
			"		<UniqueParam default=\"start_group\">"
			"			<GroupParam name=\"start_group\">"
			"				<Param name=\"start\"/>"
			"				<Param name=\"ve\" default=\"{0.1,0.1,0.1,0.1,0.1,0.1}\"/>"
			"				<Param name=\"ae\" default=\"{0.1,0.1,0.1,0.1,0.1,0.1}\"/>"
			"				<Param name=\"de\" default=\"{0.1,0.1,0.1,0.1,0.1,0.1}\"/>"
			"				<Param name=\"increase_count\" default=\"50\"/>"
			"				<Param name=\"eul_type\" default=\"321\"/>"
			"			</GroupParam>"
			"			<Param name=\"stop\"/>"
			"			<GroupParam>"
			"				<Param name=\"x\" default=\"0\"/>"
			"				<Param name=\"y\" default=\"0\"/>"
			"				<Param name=\"z\" default=\"0\"/>"
			"				<Param name=\"a\" default=\"0\"/>"
			"				<Param name=\"b\" default=\"0\"/>"
			"				<Param name=\"c\" default=\"0\"/>"
			"			</GroupParam>"
			"		</UniqueParam>"
					CHECK_PARAM_STRING
			"	</GroupParam>"
			"</Command>");
	}
	ARIS_DEFINE_BIG_FOUR_CPP(ManualMove);

	auto GetPartPq::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		auto pm = std::make_any<std::vector<double> >(target.model->partPool().size() * 16);
		target.server->getRtData([](aris::server::ControlServer& cs, std::any& data)
		{
			for (aris::Size i(-1); ++i < cs.model().partPool().size();)
				cs.model().partPool().at(i).getPm(std::any_cast<std::vector<double>& >(data).data() + i * 16);
		}, pm);

		auto pq = std::vector<double>(target.model->partPool().size() * 7);

		for (aris::Size i(-1); ++i < target.server->model().partPool().size();)
			aris::dynamic::s_pm2pq(std::any_cast<std::vector<double>& >(pm).data() + i * 16, pq.data() + i * 7);
		
		//   return text
		// target.param = aris::core::Matrix(pq.size(), 1, pq.data()).toString();
		//

		//  return binary
		std::string ret(reinterpret_cast<char*>(pq.data()), pq.size() * sizeof(double));
		target.ret = ret;

		target.option |= NOT_RUN_EXECUTE_FUNCTION | NOT_PRINT_CMD_INFO | NOT_PRINT_CMD_INFO;
	}
	GetPartPq::~GetPartPq() = default;
	GetPartPq::GetPartPq(const std::string &name) : Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"get_part_pq\">"
			"</Command>");
	}

	auto GetXml::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		target.server->waitForAllCollection();
		target.ret = target.server->xmlString();
		target.option |= NOT_RUN_EXECUTE_FUNCTION | NOT_RUN_COLLECT_FUNCTION;
	}
	GetXml::~GetXml() = default;
	GetXml::GetXml(const std::string &name) : Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"get_xml\">"
			"</Command>");
	}

	auto SetXml::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{		
		// remove all symbols "{" "}"
		if (target.server->running())THROW_FILE_AND_LINE("server is not running,can't save xml");
		auto xml_str = params.at("xml").substr(1, params.at("xml").size() - 2);
		// 这一句要小心，此时 this 已被销毁，后面不能再调用this了 //
		target.server->loadXmlStr(xml_str);
		target.option |= NOT_RUN_EXECUTE_FUNCTION | NOT_RUN_COLLECT_FUNCTION;
	}
	SetXml::~SetXml() = default;
	SetXml::SetXml(const std::string &name) : Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"set_xml\">"
			"	<Param name=\"xml\"/>"
			"</Command>");
	}

	auto Start::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		target.server->start();
		target.option |= NOT_RUN_EXECUTE_FUNCTION | NOT_RUN_COLLECT_FUNCTION;
	}
	Start::~Start() = default;
	Start::Start(const std::string &name) : Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"start\">"
			"</Command>");
	}

	auto Stop::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		target.server->stop();
		target.option |= NOT_RUN_EXECUTE_FUNCTION | NOT_RUN_COLLECT_FUNCTION;
	}
	Stop::~Stop() = default;
	Stop::Stop(const std::string &name) : Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"stop\">"
			"</Command>");
	}

	auto RemoveFile::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		std::uintmax_t  memo = std::stoull(params.at("memo"));
		auto file_path = params.at("filePath");

		// 获得所有文件
		std::vector<std::filesystem::path> files;
		for (auto &p : std::filesystem::directory_iterator(file_path))
		{
			if (p.is_regular_file())files.push_back(p.path());
		}
		// 按照修改时间排序
		std::sort(files.begin(), files.end(), [](const std::filesystem::path &p1, const std::filesystem::path &p2)->bool   //lambda函数，匿名
		{
			return std::filesystem::last_write_time(p1) < std::filesystem::last_write_time(p2);
		});

		std::cout << "log file num:" << files.size() << std::endl;

		std::filesystem::space_info devi = std::filesystem::space(file_path);
		// 根据内存地址删除;
		while (devi.available < std::uintmax_t(1048576) * memo && !files.empty())
		{
			std::filesystem::remove(files.back());
			files.pop_back();
			devi = std::filesystem::space(file_path);
		}

		std::cout << "left space  :" << devi.available / std::uintmax_t(1048576) << "MB" << std::endl;

		target.option =	NOT_RUN_EXECUTE_FUNCTION;
	}
	RemoveFile::~RemoveFile() = default;
	RemoveFile::RemoveFile(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"rmFi\">"
			"	<GroupParam>"
			"	    <Param name=\"filePath\" default=\"C:/Users/qianch_kaanh_cn/Desktop/build_qianch/log/\" abbreviation=\"f\" />"
			"	    <Param name=\"memo\" default=\"40\" abbreviation=\"m\" />"
			"	</GroupParam>"
			"</Command>");
	}

	struct UniversalPlan::Imp
	{
		PrepairFunc prepair_nrt;
		ExecuteFunc execute_rt;
		CollectFunc collect_nrt;
	};
	auto UniversalPlan::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void { if (imp_->prepair_nrt)imp_->prepair_nrt(params, target); }
	auto UniversalPlan::executeRT(PlanTarget &param)->int { return imp_->execute_rt ? imp_->execute_rt(param) : 0; }
	auto UniversalPlan::collectNrt(PlanTarget &param)->void
	{ 
		if (imp_->collect_nrt)imp_->collect_nrt(param);
	}
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
	ARIS_DEFINE_BIG_FOUR_CPP(UniversalPlan);
}
