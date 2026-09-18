#ifndef ARIS_PLAN_MULTIMODEL_ASYNC_PLANNER_H_
#define ARIS_PLAN_MULTIMODEL_ASYNC_PLANNER_H_

#include <list>
#include <cmath>
#include <iostream>
#include <functional>
#include <map>
#include <any>

#include <aris/core/object.hpp>
#include <aris/core/expression_calculator.hpp>
#include <aris/plan/trajectory.hpp>
#include <aris/dynamic/model_base.hpp>

/// \brief 轨迹规划命名空间
/// \ingroup aris
/// 
///
///
/// 
/// 
namespace aris::plan{

	class ARIS_API ToolWobjSelector {
	public:
		using MarkerVec = std::vector<aris::dynamic::Marker*>;
		
		// 配置模型 //
		auto setModel(aris::dynamic::MultiModel& model) -> void;
		auto model() -> aris::dynamic::MultiModel&;

		auto setSubModelId(std::vector<aris::Size> id_list) -> void;
		auto subModelId() -> const std::vector<aris::Size>&;

		auto selectTw(aris::dynamic::Marker** tools, aris::dynamic::Marker** wobjs) -> int;
		auto setTwPos(const double* twpos)->void;
		auto getTwPos(double* twpos)->void;
		auto setEePos(const double* eepos) -> void;
		auto getEePos(double* eepos) -> void;

		~ToolWobjSelector();
		ToolWobjSelector();
		ARIS_DELETE_BIG_FOUR(ToolWobjSelector);

	private:
		struct Imp;
		std::unique_ptr<Imp> imp_;
	};

	/// @brief 规划器运行状态机
	///
	/// 描述 MultimodelPlanner 当前的运行状态，以及正常运行、暂停/恢复、停止、
	/// 错误等状态之间的迁移关系。
	///
	/// 各状态含义：
	/// - Uninitialized：未初始化（构造后、或停止后）；
	/// - Idle：已初始化且停止；
	/// - Running：正常运行中；
	/// - Stopping：正在减速停止（speed ratio 趋向 0），停止完成后进入 Uninitialized；
	/// - Pausing：正在减速暂停（speed ratio 趋向 0），降到 0 后进入 Paused；
	/// - Paused：已完全暂停，机器人可能被移动到其他位置；
	/// - Resuming：正在恢复运行（speed ratio 仍为 0），机器人先平滑运动回暂停位置；
	/// - PausedMoving：正在移动到目标（独立管线），完成后回到 Paused；
	/// - UninitializedMoving：正在移动到目标（独立管线），完成后回到 Uninitialized；
	/// - Error：运行中出现错误。
	///
	/// 各操作的语义：
	/// - getNextInput()：根据当前状态调度到内部 run/pause/resume/stop 各 OneStep
	///   以及 move-to-target 的专用管线（tg/is/sr），并在其返回 0 时切换终态；
	///   返回规划器节点 id（0 表示执行完毕）；
	/// - requestInit()：Uninitialized → Idle；
	/// - requestStop()：运动状态 → Stopping（平滑减速），静止状态（Idle/Paused）→ Uninitialized；
	/// - requestPause()：Running → Pausing；
	/// - requestResume()：Paused → Resuming。
	enum class PlannerState {
		Uninitialized,	///< 未初始化（构造后、或停止后）
		Idle,		///< 已初始化且停止
		Running,	///< 正常运行中
		Stopping,	///< 正在减速停止（speed ratio 趋向 0），清除当前规划队列
		Pausing,	///< 正在减速暂停（speed ratio 趋向 0），不清除当前规划队列，后续可恢复运行
		Paused,		///< 已完全暂停，当前状态下机器人可能移动到其他位置，后续可恢复运行
		Resuming,	///< 正在恢复运行（speed ratio 仍为0，机器人运动到暂停的位置）
		PausedMoving,		///< 正在移动到目标（独立管线），完成后回到 Paused
		UninitializedMoving,	///< 正在移动到目标（独立管线），完成后回到 Uninitialized
		Error,		///< 运行中出现错误
	};

	class ARIS_API MultimodelPlanner {
	public:
		using TW = std::vector<std::pair<std::string, std::string>>;
		
		////////////////// PART 1 config ////////////////

		auto setDt(double dt) -> void;
		auto dt() -> double;
		
		auto setModel(aris::dynamic::MultiModel& model)->void;
		auto model() -> aris::dynamic::MultiModel&;

		auto setSubModelId(std::vector<aris::Size> id_list) -> void;
		auto subModelId() -> const std::vector<aris::Size> &;

		auto setMaxPos(aris::core::Matrix pos) -> void;
		auto maxPos() -> aris::core::Matrix;
		auto setMaxVel(aris::core::Matrix vel) -> void;
		auto maxVel() -> aris::core::Matrix;
		auto setMaxAcc(aris::core::Matrix acc) -> void;
		auto maxAcc() -> aris::core::Matrix;
		auto setMaxJerk(aris::core::Matrix jerk) -> void;
		auto maxJerk() -> aris::core::Matrix;
		auto setMinPos(aris::core::Matrix pos) -> void;
		auto minPos() -> aris::core::Matrix;
		auto setMinVel(aris::core::Matrix vel) -> void;
		auto minVel() -> aris::core::Matrix;
		auto setMinAcc(aris::core::Matrix acc) -> void;
		auto minAcc() -> aris::core::Matrix;
		auto setMinJerk(aris::core::Matrix jerk) -> void;
		auto minJerk() -> aris::core::Matrix;
		
		// 笛卡尔空间中重规划个数 //
		auto maxReplanNum()const -> int;
		auto setMaxReplanNum(int max_replan_num = 10) -> void;
		
		// 前瞻个数 //
		auto setLookAheadCount(int count) -> void;
		auto lookAheadCount() -> int;

		////////////////// PART 2 NRT operation ////////////////

		auto allocateMemory() -> void;
		
		// init之后才可以 insert 及 run //
		auto init() -> void;
		
		// 插入新的数据 //
		auto insertLinePos(TW& tw, const double* ee_pos, const double* vel, const double* acc, const double* jerk, const double* zone, double time_zone = 0.0) -> std::int64_t;
		auto insertLinePos(std::string_view tools, std::string_view wobjs, const double* ee_pos, const double* vel, const double* acc, const double* jerk, const double* zone, double time_zone = 0.0) -> std::int64_t;

		// 插入新的数据 //
		auto insertCirclePos(TW& tw, const double* ee_pos, const double* mid_pos, const double* vel, const double* acc, const double* jerk, const double* zone, double time_zone = 0.0) -> std::int64_t;
		auto insertCirclePos(std::string_view tools, std::string_view wobjs, const double* ee_pos, const double* mid_pos, const double* vel, const double* acc, const double* jerk, const double* zone, double time_zone = 0.0) -> std::int64_t;

		// 插入新的数据 //
		auto insertMoveJ(TW& tw, const double* ee_pos, const double* joint_v, const double* joint_a, const double* joint_j, const double* zone, const std::int64_t *which_root = nullptr, double time_zone = 0.0) -> std::int64_t;
		auto insertMoveJ(std::string_view tools, std::string_view wobjs, const double* ee_pos, const double* joint_v, const double* joint_a, const double* joint_j, const double* zone, const std::int64_t *which_root = nullptr, double time_zone = 0.0) -> std::int64_t;

		// 插入新的数据 //
		auto insertMoveAbsJ(const double* joint_p, const double* joint_v, const double* joint_a, const double* joint_j, const double* zone, double time_zone = 0.0) -> std::int64_t;

		// 重规划 //
		auto updateInsertPos()->void;

		// 当前还剩余的指令数 //
		auto unusedPosNum() -> int;

		// 返回当前所有的节点 id //
		auto unusedNodeIds()const -> std::vector<std::int64_t>;

		// 调速设置 //
		auto setTargetSpeedRatio(double ds) -> void; // 0 <= ds <= 1
		auto targetSpeedRatio() -> double;
		auto actualSpeedRatio() -> double;

		/// @brief 并发设计（线程模型）
		///
		/// 状态机按「最多两个线程、两类角色」设计，线程安全依赖
		/// `std::atomic<PlannerState> state_` 的 CAS 语义：
		///
		/// - onestep 系列（实时线程，串行）：
		///   `getNextInput`（内部调用 runOneStep / pauseOneStep / resumeOneStep /
		///   stopOneStep 各 OneStep）。由同一个实时控制线程
		///   循环调用，彼此不会并发执行；
		///
		/// - request 系列（非实时线程）：
		///   `requestInit` / `requestStop` / `requestPause` / `requestResume`。
		///   由非实时线程调用，且调用方保证同一时刻最多只有一个 request 函数
		///   在执行——即两个 request 系列函数不可能在不同线程上并发；
		///
		/// - 因此系统内最多同时只有两个线程碰撞：一个 request 线程 + 一个
		///   onestep 线程。
		///
		/// 状态切换职责划分（两类函数各管各的边，互不重叠）：
		///
		/// 1. 实时循环内的 getNextInput（onestep 调度）负责「运动推进」相关的切换：
		///    - Idle            → Running        ：首拍 runOneStep 产生运动（ret != 0）
		///    - Running         → Idle           ：runOneStep 返回 0（轨迹执行完毕）
		///    - Pausing         → Paused         ：pauseOneStep 返回 0（速度降到 0）
		///    - Resuming        → Running        ：resumeOneStep 返回 0（恢复完成）
		///    - Stopping        → Uninitialized  ：stopOneStep 返回 0（停止完成）
		///    - PausedMoving    → Paused         ：move 专用管线返回 0（到达目标）
		///    - UninitializedMoving → Uninitialized ：move 专用管线返回 0（到达目标）
		///    它的终态只可能是：Running、Idle、Paused、Uninitialized
		///    （过程中可短暂停留在 Pausing / Resuming / Stopping /
		///     PausedMoving / UninitializedMoving）。
		///
		/// 2. request 系列（非实时线程）负责「外部请求」相关的切换：
		///    - Uninitialized          → Idle           ：requestInit
		///    - Idle                   → Uninitialized  ：requestStop（静止状态直接清除）
		///    - Paused                 → Uninitialized  ：requestStop（静止状态直接清除）
		///    - Running                → Stopping       ：requestStop（运动状态平滑停止）
		///    - Pausing                → Stopping       ：requestStop（暂停中转为停止）
		///    - Resuming               → Stopping       ：requestStop（恢复中转为停止）
		///    - PausedMoving           → Stopping       ：requestStop（移动中转为停止）
		///    - UninitializedMoving    → Stopping       ：requestStop（移动中转为停止）
		///    - Stopping               → Stopping       ：requestStop（保持停止流程）
		///    - Error                  → Stopping       ：requestStop（错误状态转为停止）
		///    - Running                → Pausing        ：requestPause
		///    - Resuming               → Pausing        ：requestPause（恢复中暂停，减速后回 Paused）
		///    - Paused                 → Resuming       ：requestResume
		///    它的切换结果只可能是：Idle、Stopping、Pausing、Resuming、Uninitialized
		///    （各 request 函数直接在自己的函数体内用 state_ 的 CAS 完成；
		///      其中 requestStop 对静止状态直达 Uninitialized，对运动状态切 Stopping）。
		///
		/// 同步要点：
		/// - request 系列的状态写入在各 request 函数内直接使用 state_ 的 CAS；
		/// - `requestResume` 先构建 resume scurve（写 `resume_from_pos_`、
		///   `resume_scurve_params_`、`resume_t_`、`resume_T_`），再 CAS 到
		///   `Resuming`。借助原子操作的 happens-before 关系，保证 onestep 线程
		///   观察到 Resuming 时这些数据已就绪；
		/// - `requestStop` / `requestPause` 使用循环 CAS 应对实时线程的并发状态切换
		///   （例如 requestPause 时实时线程恰好把 Resuming 切换为 Running）；
		///   `requestPause` 在 CAS 前写 `pausing_from_resume_` 标志（Pausing 状态下由
		///   onestep 线程读取），同样借助 state_ 的 happens-before 保证可见；
		/// - 其余 request 函数只做一次 CAS，不触碰 onestep 线程正在读写的位置/
		///   速度缓冲，因此无需额外加锁。
		///

		/// @brief 请求初始化：仅将状态机从 Uninitialized 置为 Idle
		/// @return 0 成功，-1 失败
		auto requestInit() -> std::int64_t;
		/// @brief 请求停止：运动状态置为 Stopping（由后续 stopOneStep 平滑减速）；静止状态（Idle/Paused）直接置为 Uninitialized
		/// @return 0 成功，-1 失败
		auto requestStop() -> std::int64_t;
		/// @brief 请求暂停：Running / Resuming 置为 Pausing（Resuming 时按当前速度平滑减速到 Paused）
		/// @return 0 成功，-1 失败
		auto requestPause() -> std::int64_t;
		/// @brief 请求恢复：仅将状态机置为 Resuming
		/// @return 0 成功，-1 失败
		auto requestResume() -> std::int64_t;

		////////////////// PART 3 RT operation ////////////////
		
		// 暂停/恢复控制 //
		/// @brief 获取当前暂停/恢复状态
		auto state() const -> PlannerState;
		/// @brief 根据当前状态执行一步（内部调度到 run/pause/resume/stop 各 OneStep 及 move 专用管线）
		/// @param input_pos 输出电机位置（inputSize 维）
		/// @return 规划器节点 id，0 表示执行完毕
		auto getNextInput(double* input_pos) -> std::int64_t;

		// 移动到目标位置（独立管线：专用 tg/is/sr，单指令，不插入主队列）//
		/// @brief 关节空间移动到目标（MoveAbsJ，zone=0）。
		/// 仅允许从 Uninitialized（→UninitializedMoving）或 Paused（→PausedMoving）启动；
		/// 已在 moving 状态时返回 -1（不能重复插入）。
		/// @param joint_pos 目标关节位置（inputSize 维）
		/// @param joint_v/joint_a/joint_j 关节速度/加速度/加加速度（inputSize 维）
		/// @return 1 成功，<0 失败
		auto moveToTargetJoint(const double* joint_pos, const double* joint_v, const double* joint_a, const double* joint_j) -> std::int64_t;

		/// @brief 笛卡尔直线移动到目标（Line，zone=0），状态约束同 moveToTargetJoint。
		/// @param tw 工具/工件配对
		/// @param tw_pos 目标位置（tool/wobj 坐标系，outputPosSize 维）
		/// @param vel/acc/jerk 末端速度/加速度/加加速度（outputPosMagSize 维）
		/// @return 1 成功，<0 失败
		auto moveToTargetLine(TW& tw, const double* tw_pos, const double* vel, const double* acc, const double* jerk) -> std::int64_t;
		/// @brief 笛卡尔直线移动到目标（字符串工具/工件重载）
		auto moveToTargetLine(std::string_view tools, std::string_view wobjs, const double* tw_pos, const double* vel, const double* acc, const double* jerk) -> std::int64_t;

		// 清除错误 //
		auto clearError() -> void;

		/// @brief 获取规划器返回值
		/// @param chanel 通道
		/// @return 返回规划器内节点的 id，如果为 0 则规划执行完毕
		auto tgRet() -> std::int64_t;

		/// @brief 获取逆运动学返回值
		/// @param chanel 通道
		/// @return 逆运动学返回值，一般来说 ret < 0 为报错
		auto ikRet() -> std::int64_t;

		auto currnetId()->std::int64_t;
		
		// 
		auto finalId()->std::int64_t;

		/// @brief 获取当前节点剩余时间
		/// @return 剩余时间
		auto leftNodeS() -> double;

		/// @brief 获取输出位置类型
		/// @return 输出位置类型向量
		auto outputPosTypes()const -> const std::vector<aris::dynamic::PosType>&;

		/// @brief 获取输入维数
		/// @return 输入维数
		auto inputSize() -> int;

		~MultimodelPlanner();
		MultimodelPlanner();
		ARIS_DELETE_BIG_FOUR(MultimodelPlanner);

	private:
		struct Imp;
		std::unique_ptr<Imp> imp_;
	};
	
}

#endif