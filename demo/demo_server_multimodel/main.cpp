/// \example demo_server_multimodel/main.cpp
/// 仿照 demo_server 建立的控制服务器示例，模型采用 demo_model_multi 的双臂 MultiModel 模型
///

#include <iostream>
#include <regex>
#include <charconv>
#include <fstream>
#include <iomanip>
#include <ctime>

#include <aris.hpp>

aris::plan::PlannerDispacher pd;

// resume 用：保存排队中的运动指令（文本 + 节点 id），按插入顺序 //
static std::vector<std::pair<std::string, std::int64_t>> saved_cmds_;

// 清理 saved_cmds_ 中已经执行过的指令（节点 id < 当前节点 id）//
static void clean_saved_cmds() {
	auto curr = pd.plannerAt(0).currnetId();
	saved_cmds_.erase(
		std::remove_if(saved_cmds_.begin(), saved_cmds_.end(),
			[curr](const std::pair<std::string, std::int64_t>& p) { return p.second < curr; }),
		saved_cmds_.end());
}

class MoveL : public aris::core::CloneObject<MoveL, aris::plan::Plan>{
public:
	auto virtual prepareNrt()->void override;
	auto virtual executeRT()->int override;
	auto virtual collectNrt()->void override;

	virtual ~MoveL();
	explicit MoveL(const std::string& name = "MoveL");
	MoveL(const MoveL& other);

private:
	std::int64_t planner_node_id_{ 0 };
	std::vector<aris::Size> sub_model_;
	std::vector<aris::Size> motor_id_;

	// 电机位置日志 //
	std::int64_t log_count_{ 0 };
	std::int64_t log_motor_num_{ 0 };
	std::vector<double> log_pos_;
};
auto MoveL::prepareNrt()->void{
	sub_model_ = {1};

	// p,v,a,j,z //
	auto pos_mtx = matrixParam("pos");
	auto vel_mtx = matrixParam("vel");
	auto acc_mtx = matrixParam("acc");
	auto jerk_mtx = matrixParam("jerk");
	auto zone_mtx = matrixParam("zone");

	// tool & wobj //
	auto tools = stringParam("tool");
	auto wobjs = stringParam("wobj");

	if (tools.size() > 0 && tools.front() == '{' && tools.back() == '}') {
		tools.erase(tools.begin());
		tools.erase(tools.end() - 1);
	}
	if (wobjs.size() > 0 && wobjs.front() == '{' && wobjs.back() == '}') {
		wobjs.erase(wobjs.begin());
		wobjs.erase(wobjs.end() - 1);
	}

	// insert line //
	if (int64Param("resume") == -1) {
		planner_node_id_ = pd.plannerAt(0).insertMoveL(tools, wobjs, pos_mtx.data(), vel_mtx.data(), acc_mtx.data(), jerk_mtx.data(), zone_mtx.data());
		std::cout << "insert line id: " << planner_node_id_ << std::endl;
		if(planner_node_id_ < 0) 
			THROW_FILE_LINE("insert line failed");
		
		pd.plannerAt(0).updateInsertPos();

		// 清理已执行过的指令，并保存当前指令文本与节点 id，供 resume 重新下发 //
		clean_saved_cmds();
		saved_cmds_.push_back({ std::string(cmdString()), planner_node_id_ });
	}
	else {
		planner_node_id_ = int64Param("resume");
	}


	// controller setting //
	motor_id_.resize(pd.model().inputSize());
	pd.model().getSubInputMotionIds(sub_model_.size(), sub_model_.data(), motor_id_.data());

	// logging init //
	log_count_ = 0;
	log_motor_num_ = pd.model().subInputPosSize(sub_model_.size(), sub_model_.data());
	log_pos_.clear();
	log_pos_.reserve(200000 * log_motor_num_);
}
auto MoveL::executeRT()->int{
	double p[100];
	auto ret = pd.getNextInput(0, p);

	this->controller()->setMotorTargetPosById(motor_id_.size(), motor_id_.data(), p);

	// 记录电机目标位置 //
	for (std::int64_t i = 0; i < log_motor_num_; ++i)
		log_pos_.push_back(p[i]);
	++log_count_;

	return ret <= 0 ? ret : (ret <= planner_node_id_ ? ret : 0);
}
auto MoveL::collectNrt()->void{
	// 写出日志 //
	{
		char fname[160];
		std::snprintf(fname, sizeof(fname), "mvl_motor_log_%lld.csv", (long long)std::time(nullptr));
		std::ofstream ofs(fname);
		if (ofs.is_open()) {
			ofs << std::setprecision(15);
			ofs << "count,time_s";
			for (std::int64_t j = 0; j < log_motor_num_; ++j)
				ofs << ",pos" << j;
			ofs << "\n";
			for (std::int64_t c = 0; c < log_count_; ++c) {
				ofs << c << "," << c * 1e-3;
				for (std::int64_t j = 0; j < log_motor_num_; ++j)
					ofs << "," << log_pos_[c * log_motor_num_ + j];
				ofs << "\n";
			}
			ofs.close();
			std::cout << "[MoveL] logged " << log_count_ << " samples -> " << fname << std::endl;
		}
		else {
			std::cerr << "[MoveL] failed to open " << fname << std::endl;
		}
	}
}
MoveL::~MoveL() = default;
MoveL::MoveL(const MoveL & other) = default;
MoveL::MoveL(const std::string & name) {
	aris::core::fromXmlString(command(),
		"<Command name=\"mvl\">"
		"	<GroupParam>"
		"		<Param name=\"pos\" default=\"{0,0,0,0,0,0,0}\"/>"
		"		<Param name=\"pos_type\" default=\"pe321\"/>"
		"		<Param name=\"acc\" default=\"{3.0,3.0,3.0}\"/>"
		"		<Param name=\"vel\" default=\"{0.5,0.5,0.5}\"/>"
		"		<Param name=\"dec\" default=\"{3.0,3.0,3.0}\"/>"
		"		<Param name=\"jerk\" default=\"{5.0,5.0,5.0}\"/>"
		"		<Param name=\"zone\" default=\"{5.0,5.0,5.0}\"/>"
		"		<Param name=\"tool\" default=\"\"/>"
		"		<Param name=\"wobj\" default=\"\"/>"
		"		<Param name=\"resume\" default=\"-1\"/>"
		"	</GroupParam>"
		"</Command>");
}

class MoveJ : public aris::core::CloneObject<MoveJ, aris::plan::Plan>{
public:
	auto virtual prepareNrt()->void override;
	auto virtual executeRT()->int override;

	virtual ~MoveJ();
	explicit MoveJ(const std::string& name = "MoveJ");
	MoveJ(const MoveJ& other);

private:
	std::int64_t id_{ 0 };
	std::vector<aris::Size> sub_model_;
	std::vector<aris::Size> motor_id_;
};
auto MoveJ::prepareNrt()->void{
	sub_model_ = {1};

	// p,v,a,j,z //
	auto pos_mtx = matrixParam("pos");
	auto vel_mtx = matrixParam("vel");
	auto acc_mtx = matrixParam("acc");
	auto jerk_mtx = matrixParam("jerk");
	auto zone_mtx = matrixParam("zone");

	// tool & wobj //
	auto tools = stringParam("tool");
	auto wobjs = stringParam("wobj");

	if (tools.size() > 0 && tools.front() == '{' && tools.back() == '}') {
		tools.erase(tools.begin());
		tools.erase(tools.end() - 1);
	}
	if (wobjs.size() > 0 && wobjs.front() == '{' && wobjs.back() == '}') {
		wobjs.erase(wobjs.begin());
		wobjs.erase(wobjs.end() - 1);
	}

	// insert line //
	if (int64Param("resume") == -1) {
		id_ = pd.plannerAt(0).insertMoveJ(tools, wobjs, pos_mtx.data(), vel_mtx.data(), acc_mtx.data(), jerk_mtx.data(), zone_mtx.data(), nullptr);
		if (id_ < 0)
			THROW_FILE_LINE("insert movej failed");
		pd.plannerAt(0).updateInsertPos();

		// 清理已执行过的指令，并保存当前指令文本与节点 id，供 resume 重新下发 //
		clean_saved_cmds();
		saved_cmds_.push_back({ std::string(cmdString()), id_ });
	}
	else {
		id_ = int64Param("resume");
	}

	// controller setting //
	motor_id_.resize(pd.model().inputSize());
	pd.model().getSubInputMotionIds(sub_model_.size(), sub_model_.data(), motor_id_.data());
}
auto MoveJ::executeRT()->int{
	double p[100];
	auto ret = pd.getNextInput(0, p);
	this->controller()->setMotorTargetPosById(motor_id_.size(), motor_id_.data(), p);
	return ret <= id_ ? ret : 0;
}
MoveJ::~MoveJ() = default;
MoveJ::MoveJ(const MoveJ & other) = default;
MoveJ::MoveJ(const std::string & name) {
	aris::core::fromXmlString(command(),
		"<Command name=\"mvj\">"
		"	<GroupParam>"
		"		<Param name=\"pos\" default=\"{0,0,0,0,0,0,0}\"/>"
		"		<Param name=\"pos_type\" default=\"pe321\"/>"
		"		<Param name=\"acc\" default=\"{3.0,3.0,3.0,3.0,3.0,3.0,3.0}\"/>"
		"		<Param name=\"vel\" default=\"{0.5,0.5,0.5,0.5,0.5,0.5,0.5}\"/>"
		"		<Param name=\"dec\" default=\"{3.0,3.0,3.0,3.0,3.0,3.0,3.0}\"/>"
		"		<Param name=\"jerk\" default=\"{5.0,5.0,5.0,5.0,5.0,5.0,5.0}\"/>"
		"		<Param name=\"zone\" default=\"{5.0,5.0,5.0,5.0,5.0,5.0,5.0}\"/>"
		"		<Param name=\"tool\" default=\"0\"/>"
		"		<Param name=\"wobj\" default=\"0\"/>"
		"		<Param name=\"resume\" default=\"-1\"/>"
		"	</GroupParam>"
		"</Command>");
}


// goto 系列：独立管线（专用 tg/is/sr，zone=0），不插入主队列。
// 仅允许从 Uninitialized 或 Paused 状态启动；执行期间由 executeRT 驱动 getNextInput，
// 返回 0 表示到达目标（状态回到 Uninitialized / Paused）。
class GotoL : public aris::core::CloneObject<GotoL, aris::plan::Plan>{
public:
	auto virtual prepareNrt()->void override;
	auto virtual executeRT()->int override;

	virtual ~GotoL();
	explicit GotoL(const std::string& name = "GotoL");
	GotoL(const GotoL& other);

private:
	std::vector<aris::Size> sub_model_;
	std::vector<aris::Size> motor_id_;
};
auto GotoL::prepareNrt()->void{
	sub_model_ = {1};

	// 目标位姿 p（tool/wobj 坐标系），末端速度/加速度/加加速度 v,a,j //
	auto pos_mtx = matrixParam("pos");
	auto vel_mtx = matrixParam("vel");
	auto acc_mtx = matrixParam("acc");
	auto jerk_mtx = matrixParam("jerk");

	// tool & wobj //
	auto tools = stringParam("tool");
	auto wobjs = stringParam("wobj");

	if (tools.size() > 0 && tools.front() == '{' && tools.back() == '}') {
		tools.erase(tools.begin());
		tools.erase(tools.end() - 1);
	}
	if (wobjs.size() > 0 && wobjs.front() == '{' && wobjs.back() == '}') {
		wobjs.erase(wobjs.begin());
		wobjs.erase(wobjs.end() - 1);
	}

	// goto 直线目标（独立管线，zone=0）//
	if (pd.plannerAt(0).gotoL(tools, wobjs, pos_mtx.data(), vel_mtx.data(), acc_mtx.data(), jerk_mtx.data()) < 0)
		THROW_FILE_LINE("gotoL failed (planner must be Uninitialized or Paused)");

	// controller setting //
	motor_id_.resize(pd.model().inputSize());
	pd.model().getSubInputMotionIds(sub_model_.size(), sub_model_.data(), motor_id_.data());
}
auto GotoL::executeRT()->int{
	double p[100];
	auto ret = pd.getNextInput(0, p);
	this->controller()->setMotorTargetPosById(motor_id_.size(), motor_id_.data(), p);
	return ret; // 0 = 到达目标；>0 = 继续 goto；<0 = 错误
}
GotoL::~GotoL() = default;
GotoL::GotoL(const GotoL & other) = default;
GotoL::GotoL(const std::string & name) {
	aris::core::fromXmlString(command(),
		"<Command name=\"gotol\">"
		"	<GroupParam>"
		"		<Param name=\"pos\" default=\"{0,0,0,0,0,0,0}\"/>"
		"		<Param name=\"vel\" default=\"{0.5,0.5,0.5}\"/>"
		"		<Param name=\"acc\" default=\"{3.0,3.0,3.0}\"/>"
		"		<Param name=\"jerk\" default=\"{5.0,5.0,5.0}\"/>"
		"		<Param name=\"tool\" default=\"\"/>"
		"		<Param name=\"wobj\" default=\"\"/>"
		"	</GroupParam>"
		"</Command>");
}

class GotoC : public aris::core::CloneObject<GotoC, aris::plan::Plan>{
public:
	auto virtual prepareNrt()->void override;
	auto virtual executeRT()->int override;

	virtual ~GotoC();
	explicit GotoC(const std::string& name = "GotoC");
	GotoC(const GotoC& other);

private:
	std::vector<aris::Size> sub_model_;
	std::vector<aris::Size> motor_id_;
};
auto GotoC::prepareNrt()->void{
	sub_model_ = {1};

	// 目标位姿 p 与中间位姿 mid（tool/wobj 坐标系），末端速度/加速度/加加速度 v,a,j //
	auto pos_mtx = matrixParam("pos");
	auto mid_mtx = matrixParam("mid");
	auto vel_mtx = matrixParam("vel");
	auto acc_mtx = matrixParam("acc");
	auto jerk_mtx = matrixParam("jerk");

	// tool & wobj //
	auto tools = stringParam("tool");
	auto wobjs = stringParam("wobj");

	if (tools.size() > 0 && tools.front() == '{' && tools.back() == '}') {
		tools.erase(tools.begin());
		tools.erase(tools.end() - 1);
	}
	if (wobjs.size() > 0 && wobjs.front() == '{' && wobjs.back() == '}') {
		wobjs.erase(wobjs.begin());
		wobjs.erase(wobjs.end() - 1);
	}

	// goto 圆弧目标（独立管线，zone=0）//
	if (pd.plannerAt(0).gotoC(tools, wobjs, pos_mtx.data(), mid_mtx.data(), vel_mtx.data(), acc_mtx.data(), jerk_mtx.data()) < 0)
		THROW_FILE_LINE("gotoC failed (planner must be Uninitialized or Paused)");

	// controller setting //
	motor_id_.resize(pd.model().inputSize());
	pd.model().getSubInputMotionIds(sub_model_.size(), sub_model_.data(), motor_id_.data());
}
auto GotoC::executeRT()->int{
	double p[100];
	auto ret = pd.getNextInput(0, p);
	this->controller()->setMotorTargetPosById(motor_id_.size(), motor_id_.data(), p);
	return ret; // 0 = 到达目标；>0 = 继续 goto；<0 = 错误
}
GotoC::~GotoC() = default;
GotoC::GotoC(const GotoC & other) = default;
GotoC::GotoC(const std::string & name) {
	aris::core::fromXmlString(command(),
		"<Command name=\"gotoc\">"
		"	<GroupParam>"
		"		<Param name=\"pos\" default=\"{0,0,0,0,0,0,0}\"/>"
		"		<Param name=\"mid\" default=\"{0,0,0,0,0,0,0}\"/>"
		"		<Param name=\"vel\" default=\"{0.5,0.5,0.5}\"/>"
		"		<Param name=\"acc\" default=\"{3.0,3.0,3.0}\"/>"
		"		<Param name=\"jerk\" default=\"{5.0,5.0,5.0}\"/>"
		"		<Param name=\"tool\" default=\"\"/>"
		"		<Param name=\"wobj\" default=\"\"/>"
		"	</GroupParam>"
		"</Command>");
}

class GotoJ : public aris::core::CloneObject<GotoJ, aris::plan::Plan>{
public:
	auto virtual prepareNrt()->void override;
	auto virtual executeRT()->int override;

	virtual ~GotoJ();
	explicit GotoJ(const std::string& name = "GotoJ");
	GotoJ(const GotoJ& other);

private:
	std::vector<aris::Size> sub_model_;
	std::vector<aris::Size> motor_id_;
};
auto GotoJ::prepareNrt()->void{
	sub_model_ = {1};

	// 目标位姿 p（tool/wobj 坐标系），关节速度/加速度/加加速度 v,a,j //
	auto pos_mtx = matrixParam("pos");
	auto vel_mtx = matrixParam("vel");
	auto acc_mtx = matrixParam("acc");
	auto jerk_mtx = matrixParam("jerk");

	// tool & wobj //
	auto tools = stringParam("tool");
	auto wobjs = stringParam("wobj");

	if (tools.size() > 0 && tools.front() == '{' && tools.back() == '}') {
		tools.erase(tools.begin());
		tools.erase(tools.end() - 1);
	}
	if (wobjs.size() > 0 && wobjs.front() == '{' && wobjs.back() == '}') {
		wobjs.erase(wobjs.begin());
		wobjs.erase(wobjs.end() - 1);
	}

	// goto 关节空间目标（MoveJ：反解目标位姿后做关节插补，独立管线，zone=0）//
	if (pd.plannerAt(0).gotoJ(tools, wobjs, pos_mtx.data(), vel_mtx.data(), acc_mtx.data(), jerk_mtx.data()) < 0)
		THROW_FILE_LINE("gotoJ failed (planner must be Uninitialized or Paused)");

	// controller setting //
	motor_id_.resize(pd.model().inputSize());
	pd.model().getSubInputMotionIds(sub_model_.size(), sub_model_.data(), motor_id_.data());
}
auto GotoJ::executeRT()->int{
	double p[100];
	auto ret = pd.getNextInput(0, p);
	this->controller()->setMotorTargetPosById(motor_id_.size(), motor_id_.data(), p);
	return ret; // 0 = 到达目标；>0 = 继续 goto；<0 = 错误
}
GotoJ::~GotoJ() = default;
GotoJ::GotoJ(const GotoJ & other) = default;
GotoJ::GotoJ(const std::string & name) {
	aris::core::fromXmlString(command(),
		"<Command name=\"gotoj\">"
		"	<GroupParam>"
		"		<Param name=\"pos\" default=\"{0,0,0,0,0,0,0}\"/>"
		"		<Param name=\"vel\" default=\"{0.5,0.5,0.5,0.5,0.5,0.5,0.5}\"/>"
		"		<Param name=\"acc\" default=\"{3.0,3.0,3.0,3.0,3.0,3.0,3.0}\"/>"
		"		<Param name=\"jerk\" default=\"{5.0,5.0,5.0,5.0,5.0,5.0,5.0}\"/>"
		"		<Param name=\"tool\" default=\"\"/>"
		"		<Param name=\"wobj\" default=\"\"/>"
		"	</GroupParam>"
		"</Command>");
}

class GotoAbsJ : public aris::core::CloneObject<GotoAbsJ, aris::plan::Plan>{
public:
	auto virtual prepareNrt()->void override;
	auto virtual executeRT()->int override;

	virtual ~GotoAbsJ();
	explicit GotoAbsJ(const std::string& name = "GotoAbsJ");
	GotoAbsJ(const GotoAbsJ& other);

private:
	std::vector<aris::Size> sub_model_;
	std::vector<aris::Size> motor_id_;
};
auto GotoAbsJ::prepareNrt()->void{
	sub_model_ = {1};

	// 目标关节位置 p，关节速度/加速度/加加速度 v,a,j //
	auto pos_mtx = matrixParam("pos");
	auto vel_mtx = matrixParam("vel");
	auto acc_mtx = matrixParam("acc");
	auto jerk_mtx = matrixParam("jerk");

	// goto 绝对关节角目标（MoveAbsJ，独立管线，zone=0）//
	if (pd.plannerAt(0).gotoAbsJ(pos_mtx.data(), vel_mtx.data(), acc_mtx.data(), jerk_mtx.data()) < 0)
		THROW_FILE_LINE("gotoAbsJ failed (planner must be Uninitialized or Paused)");

	// controller setting //
	motor_id_.resize(pd.model().inputSize());
	pd.model().getSubInputMotionIds(sub_model_.size(), sub_model_.data(), motor_id_.data());
}
auto GotoAbsJ::executeRT()->int{
	double p[100];
	auto ret = pd.getNextInput(0, p);
	this->controller()->setMotorTargetPosById(motor_id_.size(), motor_id_.data(), p);
	return ret; // 0 = 到达目标；>0 = 继续 goto；<0 = 错误
}
GotoAbsJ::~GotoAbsJ() = default;
GotoAbsJ::GotoAbsJ(const GotoAbsJ & other) = default;
GotoAbsJ::GotoAbsJ(const std::string & name) {
	aris::core::fromXmlString(command(),
		"<Command name=\"gotoabj\">"
		"	<GroupParam>"
		"		<Param name=\"pos\" default=\"{0,0,0,0,0,0,0}\"/>"
		"		<Param name=\"vel\" default=\"{0.5,0.5,0.5,0.5,0.5,0.5,0.5}\"/>"
		"		<Param name=\"acc\" default=\"{3.0,3.0,3.0,3.0,3.0,3.0,3.0}\"/>"
		"		<Param name=\"jerk\" default=\"{5.0,5.0,5.0,5.0,5.0,5.0,5.0}\"/>"
		"	</GroupParam>"
		"</Command>");
}


class Stop : public aris::core::CloneObject<Stop, aris::plan::Plan>{
public:
	auto virtual prepareNrt()->void override;

	virtual ~Stop();
	explicit Stop(const std::string& name = "Stop");
	Stop(const Stop& other);
};
auto Stop::prepareNrt()->void{
	// 仅请求停止当前正在执行的指令：将 planner 状态置为 Stopping，由原指令的 executeRT 完成平滑减速 //
	option() = NOT_RUN_EXECUTE_FUNCTION | NOT_RUN_COLLECT_FUNCTION;
	pd.plannerAt(chanelId()).requestStop();
}
Stop::~Stop() = default;
Stop::Stop(const Stop & other) = default;
Stop::Stop(const std::string & name) {
	aris::core::fromXmlString(command(),
		"<Command name=\"st\">"
		"</Command>");
}


class Pause : public aris::core::CloneObject<Pause, aris::plan::Plan>{
public:
	auto virtual prepareNrt()->void override;

	virtual ~Pause();
	explicit Pause(const std::string& name = "Pause");
	Pause(const Pause& other);
};
auto Pause::prepareNrt()->void{
	// 仅请求暂停当前正在执行的指令：将 planner 状态置为 Pausing，由原指令的 executeRT 完成平滑减速 //
	option() = NOT_RUN_EXECUTE_FUNCTION | NOT_RUN_COLLECT_FUNCTION;
	if (pd.plannerAt(chanelId()).requestPause() < 0)
		std::cerr << "[Pause] requestPause failed (planner is not running)" << std::endl;

	//pd.plannerAt(chanelId());
}
Pause::~Pause() = default;
Pause::Pause(const Pause & other) = default;
Pause::Pause(const std::string & name) {
	aris::core::fromXmlString(command(),
		"<Command name=\"pause\">"
		"</Command>");
}


class Resume : public aris::core::CloneObject<Resume, aris::plan::Plan>{
public:
	auto virtual prepareNrt()->void override;

	virtual ~Resume();
	explicit Resume(const std::string& name = "Resume");
	Resume(const Resume& other);
};
auto Resume::prepareNrt()->void{
	// 恢复：根据 planner 的 currentId / finalId 确定待继续的节点区间，重新下发对应指令（带 resume 标志跳过插入），由其 executeRT 继续推进原轨迹 //
	option() = NOT_RUN_EXECUTE_FUNCTION | NOT_RUN_COLLECT_FUNCTION;

	if (saved_cmds_.empty()) {
		std::cerr << "[Resume] no saved motion command to resume" << std::endl;
		return;
	}

	// 用 planner 的 currentId / finalId 判断哪些节点还需要继续执行 //
	auto curr = pd.plannerAt(chanelId()).currnetId();
	auto final_id = pd.plannerAt(chanelId()).finalId();

	std::vector<std::pair<std::string, std::int64_t>> cmds;
	for (auto& p : saved_cmds_) {
		if (p.second >= curr && p.second < final_id)
			cmds.push_back(p);
	}

	if (cmds.empty()) {
		std::cerr << "[Resume] no pending motion command in [currnetId, finalId)" << std::endl;
		return;
	}

	// 状态 Paused → Resuming，由重新下发的指令驱动 resumeOneStep → Running //
	if (pd.plannerAt(chanelId()).requestResume() < 0)
		std::cerr << "[Resume] requestResume failed (planner is not paused)" << std::endl;

	for (auto& p : cmds) {
		auto cmd = p.first + " --resume=" + std::to_string(p.second);
		this->controlServer()->executeCmd(cmd, nullptr, chanelId());
	}
}
Resume::~Resume() = default;
Resume::Resume(const Resume & other) = default;
Resume::Resume(const std::string & name) {
	aris::core::fromXmlString(command(),
		"<Command name=\"resume\">"
		"</Command>");
}


auto createMultiModel() -> std::unique_ptr<aris::dynamic::MultiModel> {
	// 真机数据 //
	double joints1[7]{ 149.982, -43.4585, -30.0078, -74.5814, 137.039, 49.1006, 55.3223 };
	double ee1[7]{ 0.203427, -0.420417, 0.263479, 117.546, 42.114, 222.485, -30.0078 * aris::PI / 180.0 };

	double joints2[7]{ 98.0283, -59.5074, -30.0078, -53.0331, 100.313, 68.8685, 2.34352 };
	double ee2[7]{ -0.202153, -0.476769, 0.346417, 95.5752, -19.3522, 279.021,-30.0078 * aris::PI / 180.0 };

	// 仿真时，人为给定初始状态 //
	for (int i = 0; i < 7; i++) {
		joints1[i] *= aris::PI / 180.0;
		joints2[i] *= aris::PI / 180.0;
	}
	for (int i = 0; i < 3; i++) {
		ee1[i + 3] *= aris::PI / 180.0;
		ee2[i + 3] *= aris::PI / 180.0;
	}

	// 构造模型 //
	aris::Size sub_num = 1;
	aris::Size sub_id[1]{ 1 };

	std::cout << "create multi model" << std::endl;
	std::cout << ARIS_INSTALL_PATH << std::endl;

	auto multi_model = std::make_unique<aris::dynamic::MultiModel>();
	aris::core::fromXmlFile(*multi_model, ARIS_INSTALL_PATH + std::string("/resource/test_plan/dual_arm.xml"));

	auto& sub0 = dynamic_cast<aris::dynamic::Model&>(multi_model->subModels()[0]);

	for(auto i = 0; i < 7; i++) {
		auto& sub1 = dynamic_cast<aris::dynamic::Model&>(multi_model->subModels()[1]);
		sub1.motionPool()[i].setMaxMv(1.5);
		sub1.motionPool()[i].setMinMv(-1.5);
		sub1.motionPool()[i].setMaxMa(5.0);
		sub1.motionPool()[i].setMinMa(-5.0);
		// 位置限位：与 main() 中控制器的 maxPos/minPos 保持一致，
		// 否则规划器会按模型默认的“无限制”输出，而控制器的 3.14 限位检查会报超限 //
		sub1.motionPool()[i].setMaxMp(aris::PI);
		sub1.motionPool()[i].setMinMp(-aris::PI);

		sub0.motionPool()[i].setMaxMv(1.5);
		sub0.motionPool()[i].setMinMv(-1.5);
		sub0.motionPool()[i].setMaxMa(5.0);
		sub0.motionPool()[i].setMinMa(-5.0);
		sub0.motionPool()[i].setMaxMp(aris::PI);
		sub0.motionPool()[i].setMinMp(-aris::PI);
	}

	multi_model->init();

	multi_model->subModels()[1].setInputPos(joints1);
	multi_model->subForwardKinematics(sub_num, sub_id);

	return multi_model;
}

int main(int argc, char *argv[]){
	aris::core::setLanguage(1);

	auto& cs = aris::server::ControlServer::instance();

	// 创建双臂模型 //
	auto multi_model = createMultiModel();
	auto input_size = multi_model->inputSize();

	// master / controller //
	cs.resetMaster(aris::control::createDefaultEthercatMaster(input_size, 0, 0).release());
	cs.resetController(
		aris::control::createDefaultEthercatController(input_size, 0, 0, dynamic_cast<aris::control::EthercatMaster&>(cs.master())).release()
	);
	for (int i = 0; i < input_size; ++i) {
		cs.controller().motorPool()[i].setMaxPos(aris::PI);
		cs.controller().motorPool()[i].setMinPos(-aris::PI);
		cs.controller().motorPool()[i].setMaxVel(1.5);
		cs.controller().motorPool()[i].setMinVel(-1.5);
		cs.controller().motorPool()[i].setMaxAcc(5.0);
		cs.controller().motorPool()[i].setMinAcc(-5.0);
	}

	cs.resetModel(multi_model.release());

	// 仿真环境：虚拟 EtherCAT 从站的 actualPos 就是 targetPos，初始为 0。
	// 若保持 0，则 rc 会把模型拉回全零位（奇异点），后续 mvl 反解失败、input 恒为 0；
	// 同时 mvl 第一步的位置连续性检查也会因 0 -> joints1 的跳变而报错。
	// 因此把虚拟电机初始位置设为模型当前输入位置（joints1），使两者保持一致。
	{
		std::vector<double> init_pos(input_size);
		cs.model().getInputPos(init_pos.data());
		for (int i = 0; i < input_size; ++i) {
			cs.controller().motorPool()[i].setTargetPos(init_pos[i]);
		}
	}

	// 规划调度器 //
	auto& mm = dynamic_cast<aris::dynamic::MultiModel&>(cs.model());
	pd.setModel(mm);
	pd.setChanelSize(2);
	pd.setDt(1e-3);
	pd.init();
	pd.plannerAt(0).setTargetSpeedRatio(1);

	pd.tryLockChanel(0, {1});

	// plan root //
	auto plan_root = std::make_unique<aris::plan::PlanRoot>();
	plan_root->planPool().add<aris::plan::Enable>();
    plan_root->planPool().add<aris::plan::Disable>();
    plan_root->planPool().add<aris::plan::Mode>();
    plan_root->planPool().add<aris::plan::Recover>();
    plan_root->planPool().add<aris::plan::Clear>();
    plan_root->planPool().add<aris::plan::Show>();
    plan_root->planPool().add<MoveL>();
	plan_root->planPool().add<MoveJ>();
	plan_root->planPool().add<GotoL>();
	plan_root->planPool().add<GotoC>();
	plan_root->planPool().add<GotoJ>();
	plan_root->planPool().add<GotoAbsJ>();
	plan_root->planPool().add<Stop>();
	plan_root->planPool().add<Pause>();
	plan_root->planPool().add<Resume>();
	cs.resetPlanRoot(plan_root.release());

	try {
		cs.interfacePool().add<aris::server::ProgramWebInterface>();

		cs.init();
		cs.open();
		cs.start();

		cs.runCmdLine();
	}
	catch (std::exception &e) {
		std::cout << e.what() << std::endl;
	}

	return 0;
}
