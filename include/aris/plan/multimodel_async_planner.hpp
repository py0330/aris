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
	/// - MovingToTarget：正在运动到目标位置，运动完成后切换回进入前的状态；
	/// - Stopping：正在减速停止（speed ratio 趋向 0），停止完成后进入 Uninitialized；
	/// - Pausing：正在减速暂停（speed ratio 趋向 0），降到 0 后进入 Paused；
	/// - Paused：已完全暂停，机器人可能被移动到其他位置；
	/// - Resuming：正在恢复运行（speed ratio 仍为 0），机器人先平滑运动回暂停位置；
	/// - Error：运行中出现错误。
	///
	/// 各操作的可调用状态与结束状态（返回 0 表示进入终态）：
	/// - runOneStep()：仅在 Idle/Running 下可调用；结束后进入 Idle（返回 0）或 Running；
	/// - stopOneStep()：任意状态均可调用；结束后进入 Stopping 或 Uninitialized（返回 0）；
	/// - pauseOneStep()：仅在 Idle/Paused/Pausing/Running 下可调用；
	///   Idle 下保持不变（返回 0）；其余进入 Paused（返回 0）或 Pausing（返回非 0）；
	/// - resumeOneStep()：仅在 Paused/Resuming 下可调用；结束后进入 Resuming
	///   或 Running（返回 0）；
	/// - moveToTargetOneStep()：仅在 Uninitialized/Idle/Paused/MovingToTarget 下可调用；
	///   结束后切换回进入前的状态（返回 0）或保持 MovingToTarget（返回非 0）。
	enum class PlannerState {
		Uninitialized,	///< 未初始化（构造后、或停止后）
		Idle,		///< 已初始化且停止
		Running,	///< 正常运行中
		MovingToTarget,	///< 正在运动到目标位置，运动完成后切换回进入前的状态
		Stopping,	///< 正在减速停止（speed ratio 趋向 0），清除当前规划队列
		Pausing,	///< 正在减速暂停（speed ratio 趋向 0），不清除当前规划队列，后续可恢复运行
		Paused,		///< 已完全暂停，当前状态下机器人可能移动到其他位置，后续可恢复运行
		Resuming,	///< 正在恢复运行（speed ratio 仍为0，机器人运动到暂停的位置）
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

		// 删除已经不用的数据 //
		auto clearUsedPos() -> void;

		// 删除全部数据 //
		auto clearAllPos() -> void;

		// 当前还剩余的指令数 //
		auto unusedPosNum() -> int;

		// 返回当前所有的节点 id //
		auto unusedNodeIds()const -> std::vector<std::int64_t>;

		// 调速设置 //
		auto setTargetSpeedRatio(double ds) -> void; // 0 <= ds <= 1
		auto targetSpeedRatio() -> double;
		auto actualSpeedRatio() -> double;

		/// @brief 请求停止：将状态机置为 Stopping，由后续 stopOneStep 完成平滑减速
		auto requestStop() -> void;
		/// @brief 请求暂停：仅将状态机置为 Pausing
		auto requestPause() -> void;
		/// @brief 请求恢复：仅将状态机置为 Resuming
		auto requestResume() -> void;

		////////////////// PART 3 RT operation ////////////////
		
		// 暂停/恢复控制 //
		/// @brief 获取当前暂停/恢复状态
		auto state() const -> PlannerState;
		/// @brief 执行一步暂停过程（平滑减速到零），每次调用推进一步
		/// @param input_pos 输出电机位置（inputSize 维）
		/// @return 1 表示仍在减速中，0 表示已完全暂停（状态切换为 Paused）
		auto pauseOneStep(double* input_pos) -> std::int64_t;
		/// @brief 执行一步恢复过程（先平滑回到暂停位置，再加速回目标速度）
		/// @param input_pos 输出电机位置（inputSize 维）
		/// @return 1 表示仍在恢复中，0 表示已完全恢复（状态切换为 Running）
		auto resumeOneStep(double* input_pos) -> std::int64_t;


		// 正常运行 //
		auto runOneStep(double* p) -> std::int64_t;

		auto stopOneStep(double* p) -> std::int64_t;



		// 移动到目标位置 //
		/// @brief 设置移动目标（关节空间，inputSize 维）
		auto setMoveTarget(const double* input_pos) -> void;
		/// @brief 获取移动目标（关节空间，inputSize 维）
		auto moveTarget() -> const double*;
		/// @brief 执行一步移动到目标的过程，每次调用推进一步
		/// @param input_pos 输出电机位置（inputSize 维）
		/// @return 1 表示仍在运动中，0 表示已到达目标并切换回进入前的状态
		auto moveToTargetOneStep(double* input_pos) -> std::int64_t;

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