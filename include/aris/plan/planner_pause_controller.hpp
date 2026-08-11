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

    /// @brief 请求暂停（平滑减速到零）
    ///
    /// 调用后状态从 Running 切换到 Pausing，planner 会将 speed ratio
    /// 逐步降到 0。当 actualSpeedRatio 到达 0 时自动切换到 Paused。
    auto pause() -> void;

    /// @brief 请求恢复（平滑加速回目标速度）
    ///
    /// 调用后状态从 Paused/Pausing 切换到 Resuming，planner 会将
    /// speed ratio 恢复到暂停前的值。当 actualSpeedRatio 到达目标
    /// 时自动切换到 Running。
    auto resume() -> void;

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
