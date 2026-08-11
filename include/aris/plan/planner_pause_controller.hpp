#ifndef ARIS_PLAN_PLANNER_PAUSE_CONTROLLER_H_
#define ARIS_PLAN_PLANNER_PAUSE_CONTROLLER_H_

#include <aris/plan/multimodel_async_planner.hpp>

namespace aris::plan {

/// @brief 规划器暂停/恢复状态
enum class PlannerState {
    Idle,       ///< 未初始化或已停止
    Running,    ///< 正常运行中
    Pausing,    ///< 正在减速暂停（speed ratio 趋向 0）
    Paused,     ///< 已完全暂停
    Resuming,   ///< 正在加速恢复（speed ratio 趋向目标值）
};

/// @brief 带暂停/恢复状态机的规划器控制器
///
/// 内部持有一个 MultimodelPlanner，并通过 setTargetSpeedRatio 实现平滑的
/// 暂停（减速到零）和恢复（加速回目标速度）。状态机在每次 getNextInput()
/// 调用时自动更新。
///
/// 使用示例：
/// @code
/// PlannerPauseController ctrl;
/// ctrl.planner().setModel(model);
/// ctrl.planner().setDt(0.001);
/// ctrl.planner().allocateMemory();
/// ctrl.planner().init();
///
/// // 插入运动指令...
/// ctrl.planner().insertLinePos(...);
/// ctrl.planner().updateInsertPos();
///
/// // RT 循环中
/// while (true) {
///     double input[6];
///     ctrl.getNextInput(input);  // 自动更新状态机
///
///     if (need_pause) ctrl.pause();
///     if (need_resume) ctrl.resume();
/// }
/// @endcode
class ARIS_API PlannerPauseController {
public:
    /// @name 状态查询
    /// @{

    /// @brief 获取当前状态
    auto state() const -> PlannerState;

    /// @brief 是否处于正常运行状态
    auto isRunning() const -> bool;

    /// @brief 是否处于暂停状态（含 Pausing/Paused）
    auto isPaused() const -> bool;

    /// @}

    /// @name 暂停/恢复控制
    /// @{

    /// @brief 执行暂停过程（平滑减速到零）
    ///
    /// 每次调用将状态置为 Pausing，设置 speed ratio 为 0，
    /// 调用 planner.getNextInput 推进一帧，然后检测是否已完全停止。
    /// @param input_pos 电机位置（input_psize 维），作为 getNextInput 的输出
    /// @return 1 表示仍在减速中，0 表示已完全暂停（状态切换为 Paused）
    auto pause(double* input_pos) -> int;

    /// @brief 执行恢复过程（平滑加速回目标速度）
    ///
    /// 每次调用将状态置为 Resuming，设置 speed ratio 为目标值，
    /// 调用 planner.getNextInput 推进一帧，然后检测是否已恢复。
    /// @param input_pos 电机位置（input_psize 维），作为 getNextInput 的输出
    /// @return 1 表示仍在加速中，0 表示已完全恢复（状态切换为 Running）
    auto resume(double* input_pos) -> int;

    /// @brief 设置恢复后的目标 speed ratio
    auto setTargetSpeedRatio(double ratio) -> void;

    /// @brief 获取恢复后的目标 speed ratio
    auto targetSpeedRatio() const -> double;

    /// @}

    /// @name 底层规划器访问
    /// @{

    /// @brief 获取内部 MultimodelPlanner 引用（可读写）
    ///
    /// 用于配置参数、插入运动指令等操作。
    auto planner() -> MultimodelPlanner&;

    /// @brief 获取内部 MultimodelPlanner 常量引用（只读）
    auto planner() const -> const MultimodelPlanner&;

    /// @}

    /// @name 生命周期
    /// @{

    auto allocateMemory() -> void;
    auto init() -> void;
    auto stop() -> void;

    /// @}

    /// @name 实时运行
    /// @{

    /// @brief 获取下一帧输入，同时自动更新暂停/恢复状态机
    /// @param p 输出电机位置（input_psize 维）
    /// @return 当前节点 id
    auto getNextInput(double* p) -> std::int64_t;

    /// @}

    ~PlannerPauseController();
    PlannerPauseController();
    ARIS_DELETE_BIG_FOUR(PlannerPauseController);

private:
    struct Imp;
    std::unique_ptr<Imp> imp_;
};

}  // namespace aris::plan

#endif
