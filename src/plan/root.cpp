#include <algorithm>
#include <future>
#include <array>
#include <charconv>

#include"aris/plan/function.hpp"
#include"aris/plan/root.hpp"

#include <aris/control/control.hpp>
#include <aris/dynamic/dynamic.hpp>
#include <aris/server/server.hpp>

namespace aris::plan
{
	struct Plan::Imp 
	{
		std::int64_t count_;
		
		aris::dynamic::Model *model_;
		aris::control::Master *master_;
		aris::control::Controller *controller_;
		aris::control::EthercatMaster *ec_master_;
		aris::control::EthercatController *ec_controller_;
		aris::server::ControlServer *cs_;

		std::weak_ptr<Plan> shared_for_this_;

		std::uint64_t option_;
		std::vector<std::uint64_t> mot_options_;

		std::string cmd_str_;
		std::string_view cmd_name_;
		std::map<std::string_view, std::string_view> cmd_params_;
		
		std::int64_t begin_global_count_;
		std::uint64_t command_id_{ 0 };
		aris::control::Master::RtStasticsData rt_stastic_;

		std::any param;
		std::any ret;
		std::int32_t ret_code;
		char ret_msg[1024]{};

		template<typename Type>
		auto inline getType(std::string_view param)->Type
		{
			Type ret;
			auto value = cmd_params_.at(param);
			auto result = std::from_chars(value.data(), value.data() + value.size(), ret);
			if (result.ec == std::errc::invalid_argument) {	THROW_FILE_LINE("invalid argument for param:" + std::string(param));}
			return ret;
		}
	};
	auto Plan::command()->aris::core::Command & { return dynamic_cast<aris::core::Command&>(children().front()); }
	auto Plan::count()->std::int64_t { return imp_->count_; }
	auto Plan::controlServer()->aris::server::ControlServer * { return imp_->cs_; }
	auto Plan::model()->aris::dynamic::Model* { return imp_->model_; }
	auto Plan::master()->aris::control::Master* { return imp_->master_; }
	auto Plan::controller()->aris::control::Controller* { return imp_->controller_; }
	auto Plan::ecMaster()->aris::control::EthercatMaster* { return imp_->ec_master_; }
	auto Plan::ecController()->aris::control::EthercatController* { return imp_->ec_controller_; }
	auto Plan::sharedPtrForThis()->std::shared_ptr<Plan> { return imp_->shared_for_this_.lock(); }
	auto Plan::option()->std::uint64_t& { return imp_->option_; }
	auto Plan::motorOptions()->std::vector<std::uint64_t>& { return imp_->mot_options_; }
	auto Plan::cmdString()->std::string_view { return imp_->cmd_str_; }
	auto Plan::cmdName()->std::string_view { return imp_->cmd_name_; }
	auto Plan::cmdParams()->const std::map<std::string_view, std::string_view>& { return imp_->cmd_params_; }
	auto Plan::cmdId()->std::int64_t { return imp_->command_id_; }
	auto Plan::beginGlobalCount()->std::int64_t { return imp_->begin_global_count_; }
	auto Plan::rtStastic()->aris::control::Master::RtStasticsData & { return imp_->rt_stastic_; }
	auto Plan::doubleParam(std::string_view param_name)->double { return std::stod(std::string(cmdParams().at(param_name))); }
	auto Plan::floatParam(std::string_view param_name)->float { return std::stod(std::string(cmdParams().at(param_name))); }
	auto Plan::int32Param(std::string_view param_name)->std::int32_t { return imp_->getType<std::int32_t>(param_name); }
	auto Plan::int64Param(std::string_view param_name)->std::int64_t { return imp_->getType<std::int64_t>(param_name); }
	auto Plan::uint32Param(std::string_view param_name)->std::uint32_t { return imp_->getType<std::uint32_t>(param_name); }
	auto Plan::uint64Param(std::string_view param_name)->std::uint64_t { return imp_->getType<std::uint64_t>(param_name); }
	auto Plan::matrixParam(std::string_view param_name)->aris::core::Matrix
	{
		auto &value = cmdParams().at(param_name);
		auto mat = model()->calculator().calculateExpression(std::string(value));
		return std::any_cast<double>(&mat.second) ? aris::core::Matrix(std::any_cast<double>(mat.second)): std::any_cast<aris::core::Matrix&>(mat.second);
	}
	auto Plan::matrixParam(std::string_view param_name, int m, int n)->aris::core::Matrix
	{
		auto mat = matrixParam(param_name);
		if (mat.m() != m || mat.n() != n) THROW_FILE_LINE("invalid matrix size");
		return mat;
	}
	auto Plan::param()->std::any& { return imp_->param; }
	auto Plan::ret()->std::any& { return imp_->ret; }
	auto Plan::setErrMsgRT(const char *msg)->void { std::copy_n(msg, std::max(std::strlen(msg), static_cast<std::size_t>(1024)), const_cast<char*>(controlServer()->errorMsg())); }
	auto Plan::retCode()->std::int32_t { return imp_->ret_code; }
	auto Plan::retMsg()->const char * { return imp_->ret_msg; }
	Plan::~Plan() = default;
	Plan::Plan(const std::string &name) :Object(name), imp_(new Imp) { add<aris::core::Command>(name); }
	ARIS_DEFINE_BIG_FOUR_CPP(Plan);

	struct PlanRoot::Imp { aris::core::CommandParser parser_; };
	auto PlanRoot::planPool()->aris::core::ObjectPool<Plan> & { return dynamic_cast<aris::core::ObjectPool<Plan> &>(children().front()); }
	auto PlanRoot::planParser()->aris::core::CommandParser&{ return imp_->parser_; }
	auto PlanRoot::init()->void
	{
		imp_->parser_.commandPool().clear();
		for (auto &plan : planPool()) imp_->parser_.commandPool().add<aris::core::Command>(plan.command());
		imp_->parser_.init();
	}
	PlanRoot::~PlanRoot() = default;
	PlanRoot::PlanRoot(const std::string &name) :Object(name), imp_(new Imp)
	{	
		aris::core::Object::registerTypeGlobal<aris::core::ObjectPool<Plan> >();
		add<aris::core::ObjectPool<Plan> >("plan_pool_object");
	}
	ARIS_DEFINE_BIG_FOUR_CPP(PlanRoot);

#define CHECK_PARAM_STRING \
		"		<UniqueParam default=\"check_all\">" \
		"			<Param name=\"check_all\"/>" \
		"			<Param name=\"check_none\"/>" \
		"			<GroupParam>"\
		"				<UniqueParam default=\"check_enable\">"\
		"					<Param name=\"check_enable\"/>"\
		"					<Param name=\"not_check_enable\"/>"\
		"				</UniqueParam>"\
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
		"						<UniqueParam default=\"check_pos_continuous_second_order\">"\
		"							<Param name=\"check_pos_continuous_second_order\"/>"\
		"							<Param name=\"not_check_pos_continuous_second_order\"/>"\
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
		"						<UniqueParam default=\"check_vel_following_error\">"\
		"							<Param name=\"check_vel_following_error\"/>"\
		"							<Param name=\"not_check_vel_following_error\"/>"\
		"						</UniqueParam>"\
		"					</GroupParam>"\
		"				</UniqueParam>"\
		"			</GroupParam>"\
		"		</UniqueParam>"
	auto set_check_option(const std::map<std::string_view, std::string_view> &cmd_params, Plan &plan)->void
	{
		for (auto cmd_param : cmd_params)
		{
			if (cmd_param.first == "check_all")
			{
				for (auto &option : plan.motorOptions())	option &= ~(
					Plan::NOT_CHECK_POS_MIN |
					Plan::NOT_CHECK_POS_MAX |
					Plan::NOT_CHECK_POS_CONTINUOUS |
					Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER |
					Plan::NOT_CHECK_POS_FOLLOWING_ERROR |
					Plan::NOT_CHECK_VEL_MIN |
					Plan::NOT_CHECK_VEL_MAX |
					Plan::NOT_CHECK_VEL_CONTINUOUS |
					Plan::NOT_CHECK_VEL_FOLLOWING_ERROR);
			}
			else if (cmd_param.first == "check_none")
			{
				for (auto &option : plan.motorOptions())	option |=
					Plan::NOT_CHECK_POS_MIN |
					Plan::NOT_CHECK_POS_MAX |
					Plan::NOT_CHECK_POS_CONTINUOUS |
					Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER |
					Plan::NOT_CHECK_POS_FOLLOWING_ERROR |
					Plan::NOT_CHECK_VEL_MIN |
					Plan::NOT_CHECK_VEL_MAX |
					Plan::NOT_CHECK_VEL_CONTINUOUS |
					Plan::NOT_CHECK_VEL_FOLLOWING_ERROR;
			}
			else if (cmd_param.first == "check_enable")
			{
				for (auto &option : plan.motorOptions()) option &= ~(
					Plan::NOT_CHECK_ENABLE);
			}
			else if (cmd_param.first == "not_check_enable")
			{
				for (auto &option : plan.motorOptions()) option |=
					Plan::NOT_CHECK_ENABLE;
			}
			else if (cmd_param.first == "check_pos")
			{
				for (auto &option : plan.motorOptions()) option &= ~(
					Plan::NOT_CHECK_POS_MIN |
					Plan::NOT_CHECK_POS_MAX |
					Plan::NOT_CHECK_POS_CONTINUOUS |
					Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER |
					Plan::NOT_CHECK_POS_FOLLOWING_ERROR);
			}
			else if (cmd_param.first == "not_check_pos")
			{
				for (auto &option : plan.motorOptions()) option |=
					Plan::NOT_CHECK_POS_MIN |
					Plan::NOT_CHECK_POS_MAX |
					Plan::NOT_CHECK_POS_CONTINUOUS |
					Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER |
					Plan::NOT_CHECK_POS_FOLLOWING_ERROR;
			}
			else if (cmd_param.first == "check_vel")
			{
				for (auto &option : plan.motorOptions()) option &= ~(
					Plan::NOT_CHECK_VEL_MIN |
					Plan::NOT_CHECK_VEL_MAX |
					Plan::NOT_CHECK_VEL_CONTINUOUS |
					Plan::NOT_CHECK_VEL_FOLLOWING_ERROR);
			}
			else if (cmd_param.first == "not_check_vel")
			{
				for (auto &option : plan.motorOptions()) option |=
					Plan::NOT_CHECK_VEL_MIN |
					Plan::NOT_CHECK_VEL_MAX |
					Plan::NOT_CHECK_VEL_CONTINUOUS |
					Plan::NOT_CHECK_VEL_FOLLOWING_ERROR;
			}
			else if (cmd_param.first == "check_pos_min")
			{
				for (auto &option : plan.motorOptions()) option &= ~Plan::NOT_CHECK_POS_MIN;
			}
			else if (cmd_param.first == "not_check_pos_min")
			{
				for (auto &option : plan.motorOptions()) option |= Plan::NOT_CHECK_POS_MIN;
			}
			else if (cmd_param.first == "check_pos_max")
			{
				for (auto &option : plan.motorOptions()) option &= ~Plan::NOT_CHECK_POS_MAX;
			}
			else if (cmd_param.first == "not_check_pos_max")
			{
				for (auto &option : plan.motorOptions()) option |= Plan::NOT_CHECK_POS_MAX;
			}
			else if (cmd_param.first == "check_pos_continuous")
			{
				for (auto &option : plan.motorOptions()) option &= ~Plan::NOT_CHECK_POS_CONTINUOUS;
			}
			else if (cmd_param.first == "not_check_pos_continuous")
			{
				for (auto &option : plan.motorOptions()) option |= Plan::NOT_CHECK_POS_CONTINUOUS;
			}
			else if (cmd_param.first == "check_pos_continuous_second_order")
			{
				for (auto &option : plan.motorOptions()) option &= ~Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
			}
			else if (cmd_param.first == "not_check_pos_continuous_second_order")
			{
				for (auto &option : plan.motorOptions()) option |= Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;
			}
			else if (cmd_param.first == "check_pos_following_error")
			{
				for (auto &option : plan.motorOptions()) option &= ~Plan::NOT_CHECK_POS_FOLLOWING_ERROR;
			}
			else if (cmd_param.first == "not_check_pos_following_error")
			{
				for (auto &option : plan.motorOptions()) option |= Plan::NOT_CHECK_POS_FOLLOWING_ERROR;
			}
			else if (cmd_param.first == "check_vel_min")
			{
				for (auto &option : plan.motorOptions()) option &= ~Plan::NOT_CHECK_VEL_MIN;
			}
			else if (cmd_param.first == "not_check_vel_min")
			{
				for (auto &option : plan.motorOptions()) option |= Plan::NOT_CHECK_VEL_MIN;
			}
			else if (cmd_param.first == "check_vel_max")
			{
				for (auto &option : plan.motorOptions()) option &= ~Plan::NOT_CHECK_VEL_MAX;
			}
			else if (cmd_param.first == "not_check_vel_max")
			{
				for (auto &option : plan.motorOptions()) option |= Plan::NOT_CHECK_VEL_MAX;
			}
			else if (cmd_param.first == "check_vel_continuous")
			{
				for (auto &option : plan.motorOptions()) option &= ~Plan::NOT_CHECK_VEL_CONTINUOUS;
			}
			else if (cmd_param.first == "not_check_vel_continuous")
			{
				for (auto &option : plan.motorOptions()) option |= Plan::NOT_CHECK_VEL_CONTINUOUS;
			}
			else if (cmd_param.first == "check_vel_following_error")
			{
				for (auto &option : plan.motorOptions()) option &= ~Plan::NOT_CHECK_VEL_FOLLOWING_ERROR;
			}
			else if (cmd_param.first == "not_check_vel_following_error")
			{
				for (auto &option : plan.motorOptions()) option |= Plan::NOT_CHECK_VEL_FOLLOWING_ERROR;
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
	auto set_active_motor(const std::map<std::string_view, std::string_view> &cmd_params, Plan &plan, SetActiveMotor &param)->void
	{
		param.active_motor.clear();
		param.active_motor.resize(plan.controller()->motionPool().size(), 0);
		
		for (auto cmd_param : cmd_params)
		{
			if (cmd_param.first == "all")
			{
				std::fill(param.active_motor.begin(), param.active_motor.end(), 1);
			}
			else if (cmd_param.first == "motion_id")
			{
				param.active_motor.at(plan.int32Param(cmd_param.first)) = 1;
			}
			else if (cmd_param.first == "physical_id")
			{
				param.active_motor.at(plan.controller()->motionAtPhy(plan.int32Param(cmd_param.first)).motId()) = 1;
			}
			else if (cmd_param.first == "slave_id")
			{
				param.active_motor.at(plan.controller()->motionAtSla(plan.int32Param(cmd_param.first)).motId()) = 1;
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
	auto set_input_movement(const std::map<std::string_view, std::string_view> &cmd_params, Plan &plan, SetInputMovement &param)->void
	{
		param.axis_begin_pos_vec.resize(plan.controller()->motionPool().size(), 0.0);
		for (auto cmd_param : cmd_params)
		{
			if (cmd_param.first == "pos")
			{
				auto p = plan.matrixParam(cmd_param.first);
				if (p.size() == 1)
				{
					param.axis_pos_vec.resize(plan.controller()->motionPool().size(), p.toDouble());
				}
				else if (p.size() == plan.controller()->motionPool().size())
				{
					param.axis_pos_vec.assign(p.begin(), p.end());
				}
				else
				{
					THROW_FILE_LINE("");
				}
			}
			else if (cmd_param.first == "acc")
			{
				auto a = plan.matrixParam(cmd_param.first);

				if (a.size() == 1)
				{
					param.axis_acc_vec.resize(plan.controller()->motionPool().size(), a.toDouble());
				}
				else if (a.size() == plan.controller()->motionPool().size())
				{
					param.axis_acc_vec.assign(a.begin(), a.end());
				}
				else
				{
					THROW_FILE_LINE("");
				}

			}
			else if (cmd_param.first == "vel")
			{
				auto v = plan.matrixParam(cmd_param.first);

				if (v.size() == 1)
				{
					param.axis_vel_vec.resize(plan.controller()->motionPool().size(), v.toDouble());
				}
				else if (v.size() == plan.controller()->motionPool().size())
				{
					param.axis_vel_vec.assign(v.begin(), v.end());
				}
				else
				{
					THROW_FILE_LINE("");
				}
			}
			else if (cmd_param.first == "dec")
			{
				auto d = plan.matrixParam(cmd_param.first);

				if (d.size() == 1)
				{
					param.axis_dec_vec.resize(plan.controller()->motionPool().size(), d.toDouble());
				}
				else if (d.size() == plan.controller()->motionPool().size())
				{
					param.axis_dec_vec.assign(d.begin(), d.end());
				}
				else
				{
					THROW_FILE_LINE("");
				}
			}
		}
	}
	auto check_input_movement(const std::map<std::string_view, std::string_view> &cmd_params, Plan &plan, SetInputMovement &param, SetActiveMotor &active)->void
	{
		auto c = plan.controller();
		for (Size i = 0; i < c->motionPool().size(); ++i)
		{
			if (active.active_motor[i])
			{
				if (param.axis_pos_vec[i] > c->motionPool()[i].maxPos() || param.axis_pos_vec[i] < c->motionPool()[i].minPos())
					THROW_FILE_LINE("input pos beyond range");

				if (param.axis_vel_vec[i] > c->motionPool()[i].maxVel() || param.axis_vel_vec[i] <= 0.0)
					THROW_FILE_LINE("input vel beyond range");

				if (param.axis_acc_vec[i] > c->motionPool()[i].maxAcc() || param.axis_acc_vec[i] <= 0.0)
					THROW_FILE_LINE("input acc beyond range");

				if (param.axis_dec_vec[i] > c->motionPool()[i].maxAcc() || param.axis_dec_vec[i] <= 0.0)
					THROW_FILE_LINE("input dec beyond range");
			}
		}
	}

	struct Enable::Imp :public SetActiveMotor { std::int32_t limit_time; };
	auto Enable::prepareNrt()->void
	{
		set_check_option(cmdParams(), *this);
		set_active_motor(cmdParams(), *this, *imp_);
		imp_->limit_time = int32Param("limit_time");
		
		for (auto &option : motorOptions()) option |= 
			aris::plan::Plan::NOT_CHECK_ENABLE | 
			aris::plan::Plan::NOT_CHECK_POS_MAX | 
			aris::plan::Plan::NOT_CHECK_POS_MIN |
			aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;

		std::vector<std::pair<std::string, std::any>> ret_value;
		ret() = ret_value;
	}
	auto Enable::executeRT()->int
	{
		bool is_all_finished = true;
		for (std::size_t i = 0; i < controller()->motionPool().size(); ++i)
		{
			if (imp_->active_motor[i])
			{
				auto &cm = controller()->motionPool().at(i);
				auto ret = cm.enable();
				if (ret)
				{
					is_all_finished = false;

					if (count() % 1000 == 0)
					{
						mout() << "Unenabled motor, slave id: " << cm.id() << ", absolute id: " << i << ", ret: " << ret << std::endl;
					}
				}
			}
		}

		return is_all_finished ? 0 : (count() < imp_->limit_time ? 1 : aris::plan::Plan::PLAN_OVER_TIME);
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

	struct Disable::Imp :public SetActiveMotor { std::int32_t limit_time; };
	auto Disable::prepareNrt()->void
	{
		set_check_option(cmdParams(), *this);
		set_active_motor(cmdParams(), *this, *imp_);
		imp_->limit_time = std::stoi(std::string(cmdParams().at("limit_time")));

		for (auto &option : motorOptions()) option |= aris::plan::Plan::CHECK_NONE;

		std::vector<std::pair<std::string, std::any>> ret_value;
		ret() = ret_value;
	}
	auto Disable::executeRT()->int
	{
		bool is_all_finished = true;
		for (std::size_t i = 0; i < controller()->motionPool().size(); ++i)
		{
			if (imp_->active_motor[i])
			{
				auto &cm = controller()->motionPool().at(i);
				auto ret = cm.disable();
				if (ret)
				{
					is_all_finished = false;

					if (count() % 1000 == 0)
					{
						mout() << "Undisabled motor, slave id: " << cm.id() << ", absolute id: " << i << ", ret: " << ret << std::endl;
					}
				}
			}
		}

		return is_all_finished ? 0 : (count() < imp_->limit_time ? 1 : aris::plan::Plan::PLAN_OVER_TIME);
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

	struct Home::Imp :public SetActiveMotor { std::int32_t limit_time; double offset;};
	auto Home::prepareNrt()->void
	{
		set_active_motor(cmdParams(), *this, *imp_);
		imp_->limit_time = std::stoi(std::string(cmdParams().at("limit_time")));

		for (aris::Size i = 0; i<imp_->active_motor.size(); ++i)
		{
			if (imp_->active_motor[i])
			{
				std::int8_t method = std::stoi(std::string(cmdParams().at("method")));
				if (method < 1 || method > 35) THROW_FILE_LINE("invalid home method");

				imp_->offset = std::stod(std::string(cmdParams().at("offset")));
				std::int32_t offset = std::stoi(std::string(cmdParams().at("offset")));
				std::uint32_t high_speed = std::stoi(std::string(cmdParams().at("high_speed")));
				std::uint32_t low_speed = std::stoi(std::string(cmdParams().at("low_speed")));
				std::uint32_t acc = std::stoi(std::string(cmdParams().at("acceleration")));

				auto &cm = dynamic_cast<aris::control::EthercatMotor &>(controller()->motionPool()[i]);
			
				cm.writeSdo(0x6098, 0x00, method);
				std::int8_t method_read;
				cm.readSdo(0x6098, 0x00, method_read);
				if (method_read != method)THROW_FILE_LINE("home sdo write failed method");
				cm.writeSdo(0x607C, 0x00, offset);
				std::int32_t offset_read;
				cm.readSdo(0x607C, 0x00, offset_read);
				if (offset_read != offset)THROW_FILE_LINE("home sdo write failed offset");
				cm.writeSdo(0x6099, 0x01, high_speed);
				std::int32_t high_speed_read;
				cm.readSdo(0x6099, 0x01, high_speed_read);
				if (high_speed_read != high_speed)THROW_FILE_LINE("home sdo write failed high_speed");
				cm.writeSdo(0x6099, 0x02, low_speed);
				std::int32_t low_speed_read;
				cm.readSdo(0x6099, 0x02, low_speed_read);
				if (low_speed_read != low_speed)THROW_FILE_LINE("home sdo write failed low_speed");
				cm.writeSdo(0x609A, 0x00, acc);
				std::int32_t acc_read;
				cm.readSdo(0x609A, 0x00, acc_read);
				if (acc_read != acc)THROW_FILE_LINE("home sdo write failed acc");
				
			}
		}

		std::vector<std::pair<std::string, std::any>> ret_value;
		ret() = ret_value;
	}
	auto Home::executeRT()->int
	{
		bool is_all_finished = true;
		for (std::size_t i = 0; i < controller()->motionPool().size(); ++i)
		{
			if (imp_->active_motor[i])
			{
				auto &cm = controller()->motionPool().at(i);
				auto ret = cm.home();
				if (ret)
				{
					is_all_finished = false;

					if (count() % 1000 == 0)
					{
						mout() << "Unhomed motor, slave id: " << cm.id() << ", absolute id: " << i << ", ret: " << ret << std::endl;
					}
				}
				else
				{
					imp_->active_motor[i] = false;
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

	struct Mode::Imp :public SetActiveMotor { std::int32_t limit_time, mode; };
	auto Mode::prepareNrt()->void
	{
		set_check_option(cmdParams(), *this);
		set_active_motor(cmdParams(), *this, *imp_);
		imp_->limit_time = std::stoi(std::string(cmdParams().at("limit_time")));
		imp_->mode = std::stoi(std::string(cmdParams().at("mode")));

		if (imp_->mode > 10 && imp_->mode < 8)THROW_FILE_LINE("invalid mode, aris now only support mode 8,9,10");

		for (auto &option : motorOptions()) option |= aris::plan::Plan::NOT_CHECK_ENABLE | aris::plan::Plan::NOT_CHECK_POS_MAX | aris::plan::Plan::NOT_CHECK_POS_MIN;

		std::vector<std::pair<std::string, std::any>> ret_value;
		ret() = ret_value;
	}
	auto Mode::executeRT()->int
	{
		bool is_all_finished = true;
		for (std::size_t i = 0; i < controller()->motionPool().size(); ++i)
		{
			if (imp_->active_motor[i])
			{
				auto &cm = controller()->motionPool().at(i);
				auto ret = cm.mode(imp_->mode);
				if (count() == 1) 
				{
					switch (imp_->mode)
					{
					case 8:
						cm.setTargetPos(cm.actualPos());
						break;
					case 9:
						cm.setTargetVel(0.0);
						break;
					case 10:
						cm.setTargetToq(0.0);
						break;
					default:
						break;
					}
				}

				if (ret)
				{
					is_all_finished = false;

					if (count() % 1000 == 0)
					{
						mout() << "Unmoded motor, slave id: " << cm.id() << ", absolute id: " << i << ", ret: " << ret << std::endl;
					}
				}
			}
		}

		return is_all_finished ? 0 : (count() < imp_->limit_time ? 1 : aris::plan::Plan::PLAN_OVER_TIME);
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

	auto Clear::prepareNrt()->void
	{
		aris::server::ControlServer::instance().waitForAllCollection();
		aris::server::ControlServer::instance().clearError();
		option() = NOT_RUN_EXECUTE_FUNCTION | NOT_RUN_COLLECT_FUNCTION;
	}
	Clear::Clear(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"cl\">"
			"</Command>");
	}

	struct Reset::Imp :public SetActiveMotor, SetInputMovement { std::vector<Size> total_count_vec; };
	auto Reset::prepareNrt()->void
	{
		set_check_option(cmdParams(), *this);
		set_active_motor(cmdParams(), *this, *imp_);
		set_input_movement(cmdParams(), *this, *imp_);

		for (Size i = 0; i < controller()->motionPool().size(); ++i)
		{
			auto &cm = controller()->motionPool()[i];
			imp_->axis_pos_vec[i] = imp_->axis_pos_vec[i] * (cm.maxPos() - cm.minPos()) + cm.minPos();
			imp_->axis_acc_vec[i] = imp_->axis_acc_vec[i] * cm.maxAcc();
			imp_->axis_vel_vec[i] = imp_->axis_vel_vec[i] * cm.maxVel();
			imp_->axis_dec_vec[i] = imp_->axis_dec_vec[i] * cm.maxAcc();
		}
		check_input_movement(cmdParams(), *this, *imp_, *imp_);

		imp_->total_count_vec.resize(controller()->motionPool().size(), 1);

		std::vector<std::pair<std::string, std::any>> ret_value;
		ret() = ret_value;
	}
	auto Reset::executeRT()->int
	{
		// 取得起始位置 //
		if (count() == 1)
		{
			for (Size i = 0; i < controller()->motionPool().size(); ++i)
			{
				if (imp_->active_motor[i])
				{
					imp_->axis_begin_pos_vec[i] = controller()->motionPool().at(i).actualPos();
				}
			}
		}

		// 设置驱动器的位置 //
		for (Size i = 0; i < controller()->motionPool().size(); ++i)
		{
			if (imp_->active_motor[i])
			{
				double p, v, a;
				aris::plan::moveAbsolute(static_cast<double>(count()), imp_->axis_begin_pos_vec[i], imp_->axis_pos_vec[i], imp_->axis_vel_vec[i] / 1000
					, imp_->axis_acc_vec[i] / 1000 / 1000, imp_->axis_dec_vec[i] / 1000 / 1000, p, v, a, imp_->total_count_vec[i]);
				controller()->motionAtAbs(i).setTargetPos(p);
			}
		}

		return (static_cast<int>(*std::max_element(imp_->total_count_vec.begin(), imp_->total_count_vec.end())) > count()) ? 1 : 0;
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
	auto Recover::prepareNrt()->void
	{
		auto p = std::make_shared<RecoverParam>();

		p->is_kinematic_ready_ = false;
		p->is_rt_waiting_ready_ = false;
		p->fut = std::async(std::launch::async, [this](std::shared_ptr<RecoverParam> p)
		{
			// 等待正解求解的需求 //
			while (!p->is_rt_waiting_ready_.load())std::this_thread::sleep_for(std::chrono::milliseconds(1));

			// 求正解 //
			p->kin_ret = model()->solverPool()[1].kinPos();

			// 通知实时线程 //
			p->is_kinematic_ready_.store(true);
		}, p);

		this->param() = p;
		for (auto &option : motorOptions()) option |= NOT_CHECK_ENABLE | NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER;

		std::vector<std::pair<std::string, std::any>> ret_value;
		ret() = ret_value;
	}
	auto Recover::executeRT()->int
	{
		auto param = std::any_cast<std::shared_ptr<RecoverParam> &>(this->param());

		if (count() == 1)
		{
			for (Size i = 0; i < std::min(controller()->motionPool().size(), model()->motionPool().size()); ++i)
			{
				controller()->motionPool()[i].setTargetPos(controller()->motionPool().at(i).actualPos());
				model()->motionPool()[i].setMp(controller()->motionPool().at(i).actualPos());
			}

			param->is_rt_waiting_ready_.store(true);
		}

		if (count() < 3) return 1;

		return param->is_kinematic_ready_.load() ? param->kin_ret : 1;
	}
	auto Recover::collectNrt()->void
	{
		if (count())
		{
			std::any_cast<std::shared_ptr<RecoverParam>&>(this->param())->fut.get();
		}
		else
		{
			// 此时前面指令出错，系统清理了该命令，这时设置一下 //
			std::any_cast<std::shared_ptr<RecoverParam>&>(this->param())->is_rt_waiting_ready_.store(true);
			std::any_cast<std::shared_ptr<RecoverParam>&>(this->param())->fut.get();
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

	struct Sleep::Imp { int count; };
	auto Sleep::prepareNrt()->void
	{
		imp_->count = std::stoi(std::string(cmdParams().at("count")));
		for (auto &option : motorOptions()) option |= NOT_CHECK_ENABLE;

		std::vector<std::pair<std::string, std::any>> ret_value;
		ret() = ret_value;
	}
	auto Sleep::executeRT()->int { return imp_->count - count(); }
	Sleep::~Sleep() = default;
	Sleep::Sleep(const std::string &name) :Plan(name), imp_(new Imp)
	{
		command().loadXmlStr(
			"<Command name=\"sl\">"
			"	<Param name=\"count\" default=\"1000\" abbreviation=\"c\"/>"
			"</Command>");
	}
	ARIS_DEFINE_BIG_FOUR_CPP(Sleep);

	auto Show::prepareNrt()->void{	for (auto &option : motorOptions()) option |= NOT_CHECK_ENABLE; }
	auto Show::executeRT()->int
	{
		controller()->mout() << "pos: ";
		for (auto &m : controller()->motionPool())
		{
			controller()->mout() << std::setprecision(15) << m.actualPos() << "   ";
		}
		controller()->mout() << std::endl;
		return 0;
	}
	Show::Show(const std::string &name) : Plan(name) { command().loadXmlStr("<Command name=\"sh\"/>"); }
	ARIS_DEFINE_BIG_FOUR_CPP(Show);

	struct MoveAbsJ::Imp :public SetActiveMotor, SetInputMovement {};
	auto MoveAbsJ::prepareNrt()->void
	{
		set_check_option(cmdParams(), *this);
		set_active_motor(cmdParams(), *this, *imp_);
		set_input_movement(cmdParams(), *this, *imp_);
		check_input_movement(cmdParams(), *this, *imp_, *imp_);

		imp_->axis_begin_pos_vec.resize(controller()->motionPool().size());
		//for (auto &option : motorOptions()) option |= aris::plan::Plan::NOT_CHECK_ENABLE;

		std::vector<std::pair<std::string, std::any>> ret_value;
		ret() = ret_value;
	}
	auto MoveAbsJ::executeRT()->int
	{
		if (count() == 1)
		{
			for (Size i = 0; i < imp_->active_motor.size(); ++i)
			{
				if (imp_->active_motor[i])
				{
					imp_->axis_begin_pos_vec[i] = controller()->motionPool()[i].targetPos();
				}
			}
		}

		aris::Size total_count{ 1 };
		for (Size i = 0; i < imp_->active_motor.size(); ++i)
		{
			if (imp_->active_motor[i])
			{
				double p, v, a;
				aris::Size t_count;
				aris::plan::moveAbsolute(static_cast<double>(count()),
					imp_->axis_begin_pos_vec[i], imp_->axis_pos_vec[i],
					imp_->axis_vel_vec[i] / 1000, imp_->axis_acc_vec[i] / 1000 / 1000, imp_->axis_dec_vec[i] / 1000 / 1000,
					p, v, a, t_count);
				controller()->motionPool()[i].setTargetPos(p);
				total_count = std::max(total_count, t_count);
			}
		}

		return total_count - count();
	}
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

	auto check_eul_validity(std::string_view eul_type)->bool
	{
		if (eul_type.size()<3)return false;

		for (int i = 0; i < 3; ++i)if (eul_type[i] > '3' || eul_type[i] < '1')return false;

		if (eul_type[0] == eul_type[1] || eul_type[1] == eul_type[2]) return false;

		return true;
	}
	auto find_pq(const std::map<std::string_view, std::string_view> &params, aris::plan::Plan &plan, double *pq_out)->bool
	{
		double pos_unit;
		auto pos_unit_found = params.find("pos_unit");
		if (pos_unit_found == params.end()) pos_unit = 1.0;
		else if (pos_unit_found->second == "m")pos_unit = 1.0;
		else if (pos_unit_found->second == "mm")pos_unit = 0.001;
		else if (pos_unit_found->second == "cm")pos_unit = 0.01;
		else THROW_FILE_LINE("");

		for (auto cmd_param : params)
		{
			if (cmd_param.first == "pq")
			{
				auto pq_mat = plan.matrixParam(cmd_param.first);
				if (pq_mat.size() != 7)THROW_FILE_LINE("");
				aris::dynamic::s_vc(7, pq_mat.data(), pq_out);
				aris::dynamic::s_nv(3, pos_unit, pq_out);
				return true;
			}
			else if (cmd_param.first == "pm")
			{
				auto pm_mat = plan.matrixParam(cmd_param.first);
				if (pm_mat.size() != 16)THROW_FILE_LINE("");
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
				else THROW_FILE_LINE("");

				std::string eul_type;
				auto eul_type_found = params.find("eul_type");
				if (eul_type_found == params.end()) eul_type = "321";
				else if (check_eul_validity(eul_type_found->second.data()))	eul_type = eul_type_found->second;
				else THROW_FILE_LINE("");

				auto pe_mat = plan.matrixParam(cmd_param.first);
				if (pe_mat.size() != 6)THROW_FILE_LINE("");
				aris::dynamic::s_nv(3, ori_unit, pe_mat.data() + 3);
				aris::dynamic::s_pe2pq(pe_mat.data(), pq_out, eul_type.data());
				aris::dynamic::s_nv(3, pos_unit, pq_out);
				return true;
			}
		}

		THROW_FILE_LINE("No pose input");
	}
	struct MoveJParam
	{
		std::vector<double> joint_vel, joint_acc, joint_dec, ee_pq, joint_pos_begin, joint_pos_end;
		std::vector<Size> total_count;
	};
	struct MoveJ::Imp {};
	auto MoveJ::prepareNrt()->void
	{
		set_check_option(cmdParams(), *this);

		MoveJParam mvj_param;

		// find ee pq //
		mvj_param.ee_pq.resize(7);
		find_pq(cmdParams(), *this, mvj_param.ee_pq.data());

		mvj_param.joint_pos_begin.resize(model()->motionPool().size(), 0.0);
		mvj_param.joint_pos_end.resize(model()->motionPool().size(), 0.0);
		mvj_param.total_count.resize(model()->motionPool().size(), 0);

		// find joint acc/vel/dec/
		for (auto cmd_param : cmdParams())
		{
			auto c = controller();
			if (cmd_param.first == "joint_acc")
			{
				mvj_param.joint_acc.clear();
				mvj_param.joint_acc.resize(model()->motionPool().size(), 0.0);

				auto acc_mat = matrixParam(cmd_param.first);
				if (acc_mat.size() == 1)std::fill(mvj_param.joint_acc.begin(), mvj_param.joint_acc.end(), acc_mat.toDouble());
				else if (acc_mat.size() == model()->motionPool().size()) std::copy(acc_mat.begin(), acc_mat.end(), mvj_param.joint_acc.begin());
				else THROW_FILE_LINE("");

				for (int i = 0; i < 6; ++i)mvj_param.joint_acc[i] *= controller()->motionPool()[i].maxAcc();

				// check value validity //
				for (Size i = 0; i< std::min(model()->motionPool().size(), c->motionPool().size()); ++i)
					if (mvj_param.joint_acc[i] <= 0 || mvj_param.joint_acc[i] > c->motionPool()[i].maxAcc())
						THROW_FILE_LINE("");
			}
			else if (cmd_param.first == "joint_vel")
			{
				mvj_param.joint_vel.clear();
				mvj_param.joint_vel.resize(model()->motionPool().size(), 0.0);

				auto vel_mat = matrixParam(cmd_param.first);
				if (vel_mat.size() == 1)std::fill(mvj_param.joint_vel.begin(), mvj_param.joint_vel.end(), vel_mat.toDouble());
				else if (vel_mat.size() == model()->motionPool().size()) std::copy(vel_mat.begin(), vel_mat.end(), mvj_param.joint_vel.begin());
				else THROW_FILE_LINE("");

				for (int i = 0; i < 6; ++i)mvj_param.joint_vel[i] *= controller()->motionPool()[i].maxVel();

				// check value validity //
				for (Size i = 0; i< std::min(model()->motionPool().size(), c->motionPool().size()); ++i)
					if (mvj_param.joint_vel[i] <= 0 || mvj_param.joint_vel[i] > c->motionPool()[i].maxVel())
						THROW_FILE_LINE("");
			}
			else if (cmd_param.first == "joint_dec")
			{
				mvj_param.joint_dec.clear();
				mvj_param.joint_dec.resize(model()->motionPool().size(), 0.0);

				auto dec_mat = matrixParam(cmd_param.first);
				if (dec_mat.size() == 1)std::fill(mvj_param.joint_dec.begin(), mvj_param.joint_dec.end(), dec_mat.toDouble());
				else if (dec_mat.size() == model()->motionPool().size()) std::copy(dec_mat.begin(), dec_mat.end(), mvj_param.joint_dec.begin());
				else THROW_FILE_LINE("");

				for (int i = 0; i < 6; ++i) mvj_param.joint_dec[i] *= controller()->motionPool()[i].maxAcc();

				// check value validity //
				for (Size i = 0; i< std::min(model()->motionPool().size(), c->motionPool().size()); ++i)
					if (mvj_param.joint_dec[i] <= 0 || mvj_param.joint_dec[i] > c->motionPool()[i].maxAcc())
						THROW_FILE_LINE("");
			}
		}

		this->param() = mvj_param;

		std::vector<std::pair<std::string, std::any>> ret_value;
		ret() = ret_value;
	}
	auto MoveJ::executeRT()->int
	{
		auto mvj_param = std::any_cast<MoveJParam>(&this->param());

		// 取得起始位置 //
		double p, v, a;
		static Size max_total_count;
		if (count() == 1)
		{
			// inverse kinematic //
			double end_pm[16];
			aris::dynamic::s_pq2pm(mvj_param->ee_pq.data(), end_pm);
			model()->generalMotionPool().at(0).setMpm(end_pm);
			if (model()->solverPool().at(0).kinPos())return -1;

			// init joint_pos //
			for (Size i = 0; i < std::min(controller()->motionPool().size(), model()->motionPool().size()); ++i)
			{
				mvj_param->joint_pos_begin[i] = controller()->motionPool()[i].targetPos();
				mvj_param->joint_pos_end[i] = model()->motionPool()[i].mp();
				aris::plan::moveAbsolute(static_cast<double>(count()), mvj_param->joint_pos_begin[i], mvj_param->joint_pos_end[i]
					, mvj_param->joint_vel[i] / 1000, mvj_param->joint_acc[i] / 1000 / 1000, mvj_param->joint_dec[i] / 1000 / 1000
					, p, v, a, mvj_param->total_count[i]);
			}

			max_total_count = *std::max_element(mvj_param->total_count.begin(), mvj_param->total_count.end());
		}

		for (Size i = 0; i < std::min(controller()->motionPool().size(), model()->motionPool().size()); ++i)
		{
			aris::plan::moveAbsolute(static_cast<double>(count()) * mvj_param->total_count[i] / max_total_count,
				mvj_param->joint_pos_begin[i], mvj_param->joint_pos_end[i],
				mvj_param->joint_vel[i] / 1000, mvj_param->joint_acc[i] / 1000 / 1000, mvj_param->joint_dec[i] / 1000 / 1000,
				p, v, a, mvj_param->total_count[i]);

			controller()->motionPool()[i].setTargetPos(p);
		}

		return max_total_count == 0 ? 0 : max_total_count - count();
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
	auto MoveL::prepareNrt()->void
	{
		set_check_option(cmdParams(), *this);

		MoveLParam mvl_param;
		mvl_param.ee_pq.resize(7);
		if (!find_pq(cmdParams(), *this, mvl_param.ee_pq.data()))THROW_FILE_LINE("");

		for (auto cmd_param : cmdParams())
		{
			if (cmd_param.first == "acc")
			{
				mvl_param.acc = doubleParam(cmd_param.first);
			}
			else if (cmd_param.first == "vel")
			{
				mvl_param.vel = doubleParam(cmd_param.first);
			}
			else if (cmd_param.first == "dec")
			{
				mvl_param.dec = doubleParam(cmd_param.first);
			}
			else if (cmd_param.first == "angular_acc")
			{
				mvl_param.angular_acc = doubleParam(cmd_param.first);
			}
			else if (cmd_param.first == "angular_vel")
			{
				mvl_param.angular_vel = doubleParam(cmd_param.first);
			}
			else if (cmd_param.first == "angular_dec")
			{
				mvl_param.angular_dec = doubleParam(cmd_param.first);
			}
		}

		for (auto &option : motorOptions())	option |= USE_TARGET_POS;
		this->param() = mvl_param;

		std::vector<std::pair<std::string, std::any>> ret_value;
		ret() = ret_value;
	}
	auto MoveL::executeRT()->int
	{
		auto mvl_param = std::any_cast<MoveLParam>(&this->param());

		// 取得起始位置 //
		static double begin_pm[16], relative_pm[16], relative_pa[6], pos_ratio, ori_ratio, norm_pos, norm_ori;
		double p, v, a;
		aris::Size pos_total_count, ori_total_count;
		if (count() == 1)
		{
			double end_pm[16];
			aris::dynamic::s_pq2pm(mvl_param->ee_pq.data(), end_pm);
			model()->generalMotionPool().at(0).updMpm();
			model()->generalMotionPool().at(0).getMpm(begin_pm);
			aris::dynamic::s_inv_pm_dot_pm(begin_pm, end_pm, relative_pm);

			// relative_pa //
			aris::dynamic::s_pm2pa(relative_pm, relative_pa);

			norm_pos = aris::dynamic::s_norm(3, relative_pa);
			norm_ori = aris::dynamic::s_norm(3, relative_pa + 3);

			aris::plan::moveAbsolute(static_cast<double>(count()), 0.0, norm_pos, mvl_param->vel / 1000, mvl_param->acc / 1000 / 1000, mvl_param->dec / 1000 / 1000, p, v, a, pos_total_count);
			aris::plan::moveAbsolute(static_cast<double>(count()), 0.0, norm_ori, mvl_param->angular_vel / 1000, mvl_param->angular_acc / 1000 / 1000, mvl_param->angular_dec / 1000 / 1000, p, v, a, ori_total_count);

			pos_ratio = pos_total_count < ori_total_count ? double(pos_total_count) / ori_total_count : 1.0;
			ori_ratio = ori_total_count < pos_total_count ? double(ori_total_count) / pos_total_count : 1.0;

			aris::plan::moveAbsolute(static_cast<double>(count()), 0.0, norm_pos, mvl_param->vel / 1000 * pos_ratio, mvl_param->acc / 1000 / 1000 * pos_ratio* pos_ratio, mvl_param->dec / 1000 / 1000 * pos_ratio* pos_ratio, p, v, a, pos_total_count);
			aris::plan::moveAbsolute(static_cast<double>(count()), 0.0, norm_ori, mvl_param->angular_vel / 1000 * ori_ratio, mvl_param->angular_acc / 1000 / 1000 * ori_ratio * ori_ratio, mvl_param->angular_dec / 1000 / 1000 * ori_ratio * ori_ratio, p, v, a, ori_total_count);
		}

		double pa[6]{ 0,0,0,0,0,0 }, pm[16], pm2[16];

		aris::plan::moveAbsolute(static_cast<double>(count()), 0.0, norm_pos, mvl_param->vel / 1000 * pos_ratio, mvl_param->acc / 1000 / 1000 * pos_ratio* pos_ratio, mvl_param->dec / 1000 / 1000 * pos_ratio* pos_ratio, p, v, a, pos_total_count);
		if (norm_pos > 1e-10)aris::dynamic::s_vc(3, p / norm_pos, relative_pa, pa);

		aris::plan::moveAbsolute(static_cast<double>(count()), 0.0, norm_ori, mvl_param->angular_vel / 1000 * ori_ratio, mvl_param->angular_acc / 1000 / 1000 * ori_ratio * ori_ratio, mvl_param->angular_dec / 1000 / 1000 * ori_ratio * ori_ratio, p, v, a, ori_total_count);
		if (norm_ori > 1e-10)aris::dynamic::s_vc(3, p / norm_ori, relative_pa + 3, pa + 3);

		aris::dynamic::s_pa2pm(pa, pm);
		aris::dynamic::s_pm_dot_pm(begin_pm, pm, pm2);

		// 反解计算电机位置 //
		model()->generalMotionPool().at(0).setMpm(pm2);
		if (model()->solverPool().at(0).kinPos())return -1;

		////////////////////////////////////// log ///////////////////////////////////////
		//double pq[7];
		//aris::dynamic::s_pm2pq(*model()->generalMotionPool().at(0).mpm(), pq);
		//controller()->lout() << count() << " " << pq[0] << " " << pq[1] << " " << pq[2] << " " << pq[3] << " " << pq[4] << " " << pq[5] << " " << pq[6] << "  ";

		//for (auto &cm : controller->motionPool())
		//{
		//	controller()->lout() << "  " << cm.targetPos() << "  " << cm.actualPos() << "  " << cm.actualVel() << "  " << cm.actualCur() << "  ";
		//}
		//controller()->lout() << "\n";
		//////////////////////////////////////////////////////////////////////////////////


		return std::max(pos_total_count, ori_total_count) > count() ? 1 : 0;
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

	struct AutoMoveParam { bool is_start_cmd; };
	struct AutoMove::Imp
	{
		static std::atomic_bool is_running_;
		static std::atomic<std::array<double, 24>> pvade_;

		double max_pe_[6], min_pe_[6];
		std::string eul_type;
	};
	std::atomic_bool AutoMove::Imp::is_running_ = false;
	std::atomic<std::array<double, 24>> AutoMove::Imp::pvade_ = std::array<double, 24>{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	auto AutoMove::prepareNrt()->void
	{
		set_check_option(cmdParams(), *this);

		option() = 0;

		AutoMoveParam param{ false };
		for (auto cmd_param : cmdParams())
		{
			if (cmd_param.first == "start")
			{
				if (Imp::is_running_.load())THROW_FILE_LINE("auto mode already started");

				imp_->eul_type = cmdParams().at("eul_type");
				if (!check_eul_validity(imp_->eul_type))THROW_FILE_LINE("");

				auto mat = matrixParam("max_pe");
				if (mat.size() != 6)THROW_FILE_LINE("");
				std::copy(mat.begin(), mat.end(), imp_->max_pe_);

				mat = matrixParam("min_pe");
				if (mat.size() != 6)THROW_FILE_LINE("");
				std::copy(mat.begin(), mat.end(), imp_->min_pe_);

				std::array<double, 24> pvade;
				mat = matrixParam("init_pe");
				if (mat.size() != 6)THROW_FILE_LINE("");
				std::copy(mat.begin(), mat.end(), pvade.begin() + 0);

				mat = matrixParam("init_ve");
				if (mat.size() != 6)THROW_FILE_LINE("");
				std::copy(mat.begin(), mat.end(), pvade.begin() + 6);

				mat = matrixParam("init_ae");
				if (mat.size() != 6)THROW_FILE_LINE("");
				std::copy(mat.begin(), mat.end(), pvade.begin() + 12);

				mat = matrixParam("init_de");
				if (mat.size() != 6)THROW_FILE_LINE("");
				std::copy(mat.begin(), mat.end(), pvade.begin() + 18);

				Imp::pvade_.store(pvade);
				Imp::is_running_.store(true);
				param.is_start_cmd = true;
				option() |= EXECUTE_WHEN_ALL_PLAN_COLLECTED | NOT_PRINT_EXECUTE_COUNT;
				for (auto &option : motorOptions())	option |= USE_TARGET_POS;
			}
			else if (cmd_param.first == "stop")
			{
				if (!Imp::is_running_.load())THROW_FILE_LINE("auto mode not started, when stop");

				Imp::is_running_.store(false);
				option() |= aris::plan::Plan::WAIT_FOR_COLLECTION;
			}
			else if (cmd_param.first == "pe")
			{
				if (!Imp::is_running_.load())THROW_FILE_LINE("auto mode not started, when pe");

				std::array<double, 24> pvade;
				auto mat = matrixParam("pe");
				if (mat.size() != 6)THROW_FILE_LINE("");
				std::copy(mat.begin(), mat.end(), pvade.begin() + 0);

				mat = matrixParam("ve");
				if (mat.size() != 6)THROW_FILE_LINE("");
				std::copy(mat.begin(), mat.end(), pvade.begin() + 6);

				mat = matrixParam("ae");
				if (mat.size() != 6)THROW_FILE_LINE("");
				std::copy(mat.begin(), mat.end(), pvade.begin() + 12);

				mat = matrixParam("de");
				if (mat.size() != 6)THROW_FILE_LINE("");
				std::copy(mat.begin(), mat.end(), pvade.begin() + 18);

				for (int i = 0; i < 6; ++i)
				{
					pvade[i] = std::max(imp_->min_pe_[i], pvade[i]);
					pvade[i] = std::min(imp_->max_pe_[i], pvade[i]);
				}

				AutoMove::Imp::pvade_.store(pvade);
				option() |= NOT_RUN_EXECUTE_FUNCTION | NOT_RUN_COLLECT_FUNCTION | NOT_PRINT_CMD_INFO | NOT_LOG_CMD_INFO;
			}
		}

		for (auto &option : motorOptions()) option |= NOT_CHECK_POS_FOLLOWING_ERROR;
		this->param() = param;
	}
	auto AutoMove::executeRT()->int
	{
		auto param = std::any_cast<AutoMoveParam>(&this->param());

		if (count() == 1)
		{
			model()->generalMotionPool()[0].setMve(std::array<double, 6>{0, 0, 0, 0, 0, 0}.data(), "123");
		}

		// get current pe //
		double pe_now[6], ve_now[6], ae_now[6];
		model()->generalMotionPool()[0].getMpe(pe_now, imp_->eul_type.c_str());
		model()->generalMotionPool()[0].getMve(ve_now, imp_->eul_type.c_str());
		model()->generalMotionPool()[0].getMae(ae_now, imp_->eul_type.c_str());
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

		model()->generalMotionPool()[0].setMpe(pe_next, imp_->eul_type.c_str());
		model()->generalMotionPool()[0].setMve(ve_next, imp_->eul_type.c_str());
		model()->generalMotionPool()[0].setMae(ae_next, imp_->eul_type.c_str());

		static int i = 0;
		if (++i % 1000 == 0)
		{
			//controller()->mout() << "pe_now :"
			//	<< pe_now[0] << "  " << pe_now[1] << "  " << pe_now[2] << "  "
			//	<< pe_now[3] << "  " << pe_now[4] << "  " << pe_now[5] << std::endl;

			//controller()->mout() << "pe_target :"
			//	<< pvade[0] << "  " << pvade[1] << "  " << pvade[2] << "  "
			//	<< pvade[3] << "  " << pvade[4] << "  " << pvade[5] << std::endl;

			//controller()->mout() << "pe_next:"
			//	<< pe_next[0] << "  " << pe_next[1] << "  " << pe_next[2] << "  "
			//	<< pe_next[3] << "  " << pe_next[4] << "  " << pe_next[5] << std::endl;
		}

		model()->solverPool()[0].kinPos();
		model()->solverPool()[0].kinVel();

		return imp_->is_running_.load() ? 1 : 0;
	}
	auto AutoMove::collectNrt()->void { if (std::any_cast<AutoMoveParam>(&this->param())->is_start_cmd)Imp::is_running_.store(false); }
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

	struct ManualMoveParam { bool is_start_cmd; };
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
	auto ManualMove::prepareNrt()->void
	{
		set_check_option(cmdParams(), *this);

		option() = 0;

		ManualMoveParam param{ false };
		for (auto cmd_param : cmdParams())
		{
			if (cmd_param.first == "start")
			{
				if (Imp::is_running_.load())THROW_FILE_LINE("auto mode already started");

				imp_->eul_type = cmdParams().at("eul_type");
				if (!check_eul_validity(imp_->eul_type))THROW_FILE_LINE("");

				imp_->increase_count = int32Param("increase_count");
				if (imp_->increase_count < 0 || imp_->increase_count>1e5)THROW_FILE_LINE("");

				auto mat = matrixParam("ve");
				if (mat.size() != 6)THROW_FILE_LINE("");
				std::copy(mat.begin(), mat.end(), imp_->ve_);

				mat = matrixParam("ae");
				if (mat.size() != 6)THROW_FILE_LINE("");
				std::copy(mat.begin(), mat.end(), imp_->ae_);

				mat = matrixParam("de");
				if (mat.size() != 6)THROW_FILE_LINE("");
				std::copy(mat.begin(), mat.end(), imp_->de_);

				Imp::is_increase_.store(std::array<int, 6>{0, 0, 0, 0, 0, 0});
				Imp::is_running_.store(true);
				option() |= EXECUTE_WHEN_ALL_PLAN_COLLECTED | NOT_PRINT_EXECUTE_COUNT;
				for (auto &option : motorOptions())	option |= USE_TARGET_POS;
			}
			else if (cmd_param.first == "stop")
			{
				if (!Imp::is_running_.load())THROW_FILE_LINE("manual mode not started, when stop");

				Imp::is_running_.store(false);
				option() |= WAIT_FOR_COLLECTION;
			}
			else if (cmd_param.first == "x")
			{
				if (!Imp::is_running_.load())THROW_FILE_LINE("manual mode not started, when pe");

				std::array<int, 6> is_increase;
				is_increase[0] = int32Param("x");
				is_increase[1] = int32Param("y");
				is_increase[2] = int32Param("z");
				is_increase[3] = int32Param("a");
				is_increase[4] = int32Param("b");
				is_increase[5] = int32Param("c");

				for (auto &value : is_increase)value = std::max(std::min(1, value), -1) * imp_->increase_count;

				Imp::is_increase_.store(is_increase);
				option() |= NOT_RUN_EXECUTE_FUNCTION | NOT_RUN_COLLECT_FUNCTION | NOT_PRINT_CMD_INFO | NOT_LOG_CMD_INFO;
			}
		}

		for (auto &option : motorOptions()) option |= NOT_CHECK_POS_FOLLOWING_ERROR;
		this->param() = param;
	}
	auto ManualMove::executeRT()->int
	{
		auto param = std::any_cast<ManualMoveParam>(&this->param());

		// get current pe //
		double pe_now[6], ve_now[6], ae_now[6];
		model()->generalMotionPool()[0].getMpe(pe_now, imp_->eul_type.c_str());
		model()->generalMotionPool()[0].getMve(ve_now, imp_->eul_type.c_str());
		model()->generalMotionPool()[0].getMae(ae_now, imp_->eul_type.c_str());
		for (int i = 3; i < 6; ++i) if (pe_now[i] > aris::PI) pe_now[i] -= 2 * PI;

		// init status //
		static std::array<int, 6> increase_status;
		static std::array<double, 6> target_pe{ 0,0,0,0,0,0 };
		if (count() == 1)
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
		model()->generalMotionPool()[0].setMpe(target_pe_new.data(), imp_->eul_type.c_str());
		auto check_motion_limit = [&]()->bool
		{
			auto c = controller();
			for (std::size_t i = 0; i < std::min(c->motionPool().size(), model()->motionPool().size()); ++i)
			{
				auto &cm = c->motionPool().at(i);
				auto &mm = model()->motionPool().at(i);

				if (mm.mp() < cm.minPos() + 0.005 || mm.mp() > cm.maxPos() - 0.005) return false;
			}
			return true;
		};
		if (model()->solverPool()[0].kinPos() == 0 && check_motion_limit()) std::swap(target_pe, target_pe_new);

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
		model()->generalMotionPool()[0].setMpe(pe_next, imp_->eul_type.c_str());
		model()->generalMotionPool()[0].setMve(ve_next, imp_->eul_type.c_str());
		model()->generalMotionPool()[0].setMae(ae_next, imp_->eul_type.c_str());
		model()->solverPool()[0].kinPos();
		model()->solverPool()[0].kinVel();

		return imp_->is_running_.load() ? 1 : 0;
	}
	auto ManualMove::collectNrt()-> void { if (std::any_cast<ManualMoveParam>(&this->param())->is_start_cmd)Imp::is_running_.store(false); }
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

	auto GetXml::prepareNrt()->void
	{
		std::vector<std::pair<std::string, std::any>> ret_value;
		ret_value.push_back(std::make_pair(std::string("configure_xml"), controlServer()->xmlString()));
		ret() = ret_value;
		option() |= NOT_RUN_EXECUTE_FUNCTION | NOT_RUN_COLLECT_FUNCTION;
	}
	GetXml::~GetXml() = default;
	GetXml::GetXml(const std::string &name) : Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"get_xml\">"
			"</Command>");
	}

	auto SetXml::prepareNrt()->void
	{		
		// remove all symbols "{" "}"
		if (this->controlServer()->running())THROW_FILE_LINE("server is running, can't set xml");
		auto xml_str = cmdParams().at("xml").substr(1, cmdParams().at("xml").size() - 2);
		// 这一句要小心，此时 this 已被销毁，后面不能再调用this了 //
		
		//controlServer()->close();
		//controlServer()->loadXmlStr(xml_str);
		// server load 会导致interface失败 ////////////////////////////////////////
		aris::core::XmlDocument doc;
		if(doc.Parse(xml_str.data(), xml_str.size()) != tinyxml2::XML_SUCCESS) THROW_FILE_LINE("XML failed");

		if (doc.RootElement()->FirstChildElement("EthercatController"))controlServer()->controller().loadXml(*doc.RootElement()->FirstChildElement("EthercatController"));
		if (doc.RootElement()->FirstChildElement("Model"))controlServer()->model().loadXml(*doc.RootElement()->FirstChildElement("Model"));
		// 这里后面需要改 ////////////////////////////////////////


		option() |= NOT_RUN_EXECUTE_FUNCTION | NOT_RUN_COLLECT_FUNCTION;

		std::vector<std::pair<std::string, std::any>> ret_value;
		ret() = ret_value;
	}
	SetXml::~SetXml() = default;
	SetXml::SetXml(const std::string &name) : Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"set_xml\">"
			"	<Param name=\"xml\"/>"
			"</Command>");
	}

	auto Start::prepareNrt()->void
	{
		controlServer()->start();
		option() |= NOT_RUN_EXECUTE_FUNCTION | NOT_RUN_COLLECT_FUNCTION;

		std::vector<std::pair<std::string, std::any>> ret_value;
		ret() = ret_value;
	}
	Start::~Start() = default;
	Start::Start(const std::string &name) : Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"cs_start\">"
			"</Command>");
	}

	auto Stop::prepareNrt()->void
	{
		controlServer()->stop();
		option() |= NOT_RUN_EXECUTE_FUNCTION | NOT_RUN_COLLECT_FUNCTION;

		std::vector<std::pair<std::string, std::any>> ret_value;
		ret() = ret_value;
	}
	Stop::~Stop() = default;
	Stop::Stop(const std::string &name) : Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"cs_stop\">"
			"</Command>");
	}

	



	auto RemoveFile::prepareNrt()->void
	{
		std::uintmax_t  memo = std::stoull(std::string(cmdParams().at("memo")));
		auto file_path = cmdParams().at("filePath");

		// 获得所有文件
		std::vector<std::filesystem::path> files;
		for (auto &p : std::filesystem::directory_iterator(file_path))
		{
			if (p.is_regular_file())files.push_back(p.path());
		}

		// 按照修改时间排序
		std::sort(files.begin(), files.end(), [](const std::filesystem::path &p1, const std::filesystem::path &p2)->bool
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

		option() =	NOT_RUN_EXECUTE_FUNCTION;
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
		prepareFunc prepare_nrt;
		ExecuteFunc execute_rt;
		CollectFunc collect_nrt;
	};
	auto UniversalPlan::prepareNrt()->void { if (imp_->prepare_nrt)imp_->prepare_nrt(this); }
	auto UniversalPlan::executeRT()->int { return imp_->execute_rt ? imp_->execute_rt(this) : 0; }
	auto UniversalPlan::collectNrt()->void { if (imp_->collect_nrt)imp_->collect_nrt(this); }
	auto UniversalPlan::setprepareFunc(prepareFunc func)->void { imp_->prepare_nrt = func; }
	auto UniversalPlan::setExecuteFunc(ExecuteFunc func)->void { imp_->execute_rt = func; }
	auto UniversalPlan::setCollectFunc(CollectFunc func)->void { imp_->collect_nrt = func; }
	UniversalPlan::~UniversalPlan() = default;
	UniversalPlan::UniversalPlan(const std::string &name, prepareFunc prepare_func, ExecuteFunc execute_func, CollectFunc collect_func, const std::string & cmd_xml_str) :Plan(name), imp_(new Imp)
	{
		imp_->prepare_nrt = prepare_func;
		imp_->execute_rt = execute_func;
		imp_->collect_nrt = collect_func;
		command().loadXmlStr(cmd_xml_str);
	}
	ARIS_DEFINE_BIG_FOUR_CPP(UniversalPlan);

	struct MoveSeriesParam 
	{
		std::vector<double> t, x, xp1, xp2, xp3, y, yp1, yp2, yp3;
	};
	auto MoveSeries::prepareNrt()->void
	{
		MoveSeriesParam param;
		
		auto x_mat = matrixParam("x");
		auto y_mat = matrixParam("y");
		if (x_mat.size() != y_mat.size())THROW_FILE_LINE("x and y size not correct");

		param.t.resize(x_mat.size() + 6);
		param.t[0] = -3;
		param.t[1] = -2;
		param.t[2] = -1;
		param.t[3] = 0;

		param.x.resize(x_mat.size() + 6);
		std::copy(x_mat.begin(), x_mat.end(), param.x.begin() + 3);
		param.x[0] = param.x[3];
		param.x[1] = param.x[3];
		param.x[2] = param.x[3];
		*(param.x.end() - 1) = *(param.x.end() - 4);
		*(param.x.end() - 2) = *(param.x.end() - 4);
		*(param.x.end() - 3) = *(param.x.end() - 4);

		param.y.resize(y_mat.size() + 6);
		std::copy(y_mat.begin(), y_mat.end(), param.y.begin() + 3);
		param.y[0] = param.y[3];
		param.y[1] = param.y[3];
		param.y[2] = param.y[3];
		*(param.y.end() - 1) = *(param.y.end() - 4);
		*(param.y.end() - 2) = *(param.y.end() - 4);
		*(param.y.end() - 3) = *(param.y.end() - 4);


		auto scale = doubleParam("scale");

		for (int i = 1; i < x_mat.size(); ++i)
		{
			auto diff_x = x_mat.data()[i] - x_mat.data()[i - 1];
			auto diff_y = y_mat.data()[i] - y_mat.data()[i - 1];
			
			param.t.data()[i + 3] = param.t.data()[i + 2] + std::max(std::sqrt(diff_x * diff_x + diff_y * diff_y), 1e-10) * scale;
		}

		*(param.t.end() - 3) = *(param.t.end() - 4) + 1;
		*(param.t.end() - 2) = *(param.t.end() - 4) + 2;
		*(param.t.end() - 1) = *(param.t.end() - 4) + 3;
		
		param.xp1.resize(x_mat.size() + 6);
		param.xp2.resize(x_mat.size() + 6);
		param.xp3.resize(x_mat.size() + 6);
		param.yp1.resize(x_mat.size() + 6);
		param.yp2.resize(x_mat.size() + 6);
		param.yp3.resize(x_mat.size() + 6);

		aris::dynamic::s_akima(param.t.size(), param.t.data(), param.x.data(), param.xp1.data(), param.xp2.data(), param.xp3.data(), 1e-10);
		aris::dynamic::s_akima(param.t.size(), param.t.data(), param.y.data(), param.yp1.data(), param.yp2.data(), param.yp3.data(), 1e-10);

		this->param() = param;
	}
	auto MoveSeries::executeRT()->int 
	{
		auto &param = std::any_cast<MoveSeriesParam&>(this->param());
		

		auto x = aris::dynamic::s_akima_at(param.t.size(), param.t.data(), param.x.data(), param.xp1.data(), param.xp2.data(), param.xp3.data(), count() / 1000.0);
		auto y = aris::dynamic::s_akima_at(param.t.size(), param.t.data(), param.y.data(), param.yp1.data(), param.yp2.data(), param.yp3.data(), count() / 1000.0);

		std::cout << x << "  " << y << std::endl;



		return count() / 1000.0 > *(param.t.end() - 4) ? 0 : 1;

	}
	MoveSeries::~MoveSeries() = default;
	MoveSeries::MoveSeries(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"mvs\">"
			"	<GroupParam>"
			"	    <Param name=\"x\" default=\"0\" abbreviation=\"x\" />"
			"	    <Param name=\"y\" default=\"0\" abbreviation=\"y\" />"
			"	    <Param name=\"scale\" default=\"10\" />"
			"	</GroupParam>"
			"</Command>");
	}





}
