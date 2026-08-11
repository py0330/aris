#include <aris/plan/planner_pause_controller.hpp>

#include <algorithm>
#include <cmath>

namespace aris::plan {

struct PlannerPauseController::Imp {
    MultimodelPlanner planner_;
    PlannerState state_{ PlannerState::Idle };
    double target_speed_ratio_{ 1.0 };  // 暂停前保存的目标 speed ratio
    double speed_epsilon_{ 1e-6 };       // 判断 speed ratio 是否到位的阈值

    auto updateState() -> void {
        switch (state_) {
        case PlannerState::Pausing: {
            double actual = planner_.actualSpeedRatio();
            if (actual <= speed_epsilon_) {
                state_ = PlannerState::Paused;
            }
            break;
        }
        case PlannerState::Resuming: {
            double actual = planner_.actualSpeedRatio();
            if (std::abs(actual - target_speed_ratio_) <= speed_epsilon_) {
                state_ = PlannerState::Running;
            }
            break;
        }
        default:
            break;
        }
    }
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

auto PlannerPauseController::pause() -> void {
    if (imp_->state_ == PlannerState::Pausing ||
        imp_->state_ == PlannerState::Paused) {
        return;  // 已在暂停流程中
    }
    // 保存当前目标 speed ratio 以便恢复
    imp_->target_speed_ratio_ = imp_->planner_.targetSpeedRatio();
    imp_->planner_.setTargetSpeedRatio(0.0);
    imp_->state_ = PlannerState::Pausing;
}

auto PlannerPauseController::resume() -> void {
    if (imp_->state_ == PlannerState::Running ||
        imp_->state_ == PlannerState::Resuming) {
        return;  // 已在运行/恢复中
    }
    imp_->planner_.setTargetSpeedRatio(imp_->target_speed_ratio_);
    imp_->state_ = PlannerState::Resuming;
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
    imp_->target_speed_ratio_ = imp_->planner_.targetSpeedRatio();
}

auto PlannerPauseController::stop() -> void {
    imp_->planner_.stop();
    imp_->state_ = PlannerState::Idle;
}

// ─── 实时运行 ────────────────────────────────────────────

auto PlannerPauseController::getNextInput(double* p) -> std::int64_t {
    // 1. 调用底层 planner 获取下一帧
    auto ret = imp_->planner_.getNextInput(p);

    // 2. 更新暂停/恢复状态机
    imp_->updateState();

    return ret;
}

PlannerPauseController::~PlannerPauseController() {
    stop();
}

PlannerPauseController::PlannerPauseController()
    : imp_(new Imp) {}

}  // namespace aris::plan
