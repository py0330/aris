#include <aris/plan/planner_pause_controller.hpp>

#include <algorithm>
#include <cmath>

namespace aris::plan {

struct PlannerPauseController::Imp {
    MultimodelPlanner planner_;
    PlannerState state_{ PlannerState::Idle };
    double target_speed_ratio_{ 1.0 };  // 恢复时的目标 speed ratio
    double speed_epsilon_{ 1e-6 };       // 判断 speed ratio 是否到位的阈值
    std::vector<double> pause_pos_;      // 暂停时的电机位置
};

// ─── 状态查询 ────────────────────────────────────────────

auto PlannerPauseController::state() const -> PlannerState {
    return imp_->state_;
}

auto PlannerPauseController::isRunning() const -> bool {
    return imp_->state_ == PlannerState::Running;
}

auto PlannerPauseController::isPaused() const -> bool {
    auto s = imp_->state_;
    return s == PlannerState::Pausing || s == PlannerState::Paused;
}

// ─── 暂停/恢复控制 ────────────────────────────────────────

auto PlannerPauseController::pause(double* input_pos) -> int {
    // 已经暂停完毕，不再操作 planner（避免副作用）
    if (imp_->state_ == PlannerState::Paused) {
        return 0;
    }

    imp_->state_ = PlannerState::Pausing;

    // 保存暂停时的电机位置
    auto input_size = imp_->planner_.inputSize();
    imp_->pause_pos_.assign(input_pos, input_pos + input_size);

    // 设置 speed ratio 为 0，减速到停止
    imp_->planner_.setTargetSpeedRatio(0.0);

    // 推进一帧
    imp_->planner_.getNextInput(input_pos);

    // 检测是否已完全暂停
    if (imp_->planner_.actualSpeedRatio() <= imp_->speed_epsilon_) {
        imp_->state_ = PlannerState::Paused;
        return 0;
    }
    return 1;
}

auto PlannerPauseController::resume(double* input_pos) -> int {
    // 已经恢复完毕，不再操作 planner（避免副作用）
    if (imp_->state_ == PlannerState::Running) {
        return 0;
    }

    imp_->state_ = PlannerState::Resuming;

    // 设置 speed ratio 为目标值
    imp_->planner_.setTargetSpeedRatio(imp_->target_speed_ratio_);

    // 推进一帧
    imp_->planner_.getNextInput(input_pos);

    // 检测是否已完全恢复
    if (std::abs(imp_->planner_.actualSpeedRatio() - imp_->target_speed_ratio_) <= imp_->speed_epsilon_) {
        imp_->state_ = PlannerState::Running;
        return 0;
    }
    return 1;
}

auto PlannerPauseController::setTargetSpeedRatio(double ratio) -> void {
    imp_->target_speed_ratio_ = ratio;
}

auto PlannerPauseController::targetSpeedRatio() const -> double {
    return imp_->target_speed_ratio_;
}

// ─── 底层规划器访问 ────────────────────────────────────────

auto PlannerPauseController::planner() -> MultimodelPlanner& {
    return imp_->planner_;
}

auto PlannerPauseController::planner() const -> const MultimodelPlanner& {
    return imp_->planner_;
}

// ─── 生命周期 ────────────────────────────────────────────

auto PlannerPauseController::allocateMemory() -> void {
    imp_->planner_.allocateMemory();
}

auto PlannerPauseController::init() -> void {
    imp_->planner_.init();
    imp_->state_ = PlannerState::Running;
}

auto PlannerPauseController::stop() -> void {
    imp_->planner_.stop();
    imp_->state_ = PlannerState::Idle;
}

// ─── 实时运行 ────────────────────────────────────────────

auto PlannerPauseController::getNextInput(double* p) -> std::int64_t {
    return imp_->planner_.getNextInput(p);
}

PlannerPauseController::~PlannerPauseController() {
    stop();
}

PlannerPauseController::PlannerPauseController()
    : imp_(new Imp) {}

}  // namespace aris::plan
