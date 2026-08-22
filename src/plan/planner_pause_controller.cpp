#include <aris/plan/planner_pause_controller.hpp>

#include <algorithm>
#include <cmath>

namespace aris::plan {

struct PlannerPauseController::Imp {
    MultimodelPlanner planner_;
    PlannerState state_{ PlannerState::Idle };
    double target_speed_ratio_{ 1.0 };  // 恢复时的目标 speed ratio
    double speed_epsilon_{ 1e-10 };       // 判断 speed ratio 是否到位的阈值
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
        // 给出已经暂停的位置 //
        std::copy_n(imp_->pause_pos_.data(), imp_->planner_.inputSize(), input_pos);
        return 0;
    }

    if (imp_->state_ == PlannerState::Running || imp_->state_ == PlannerState::Pausing) {
        // 设置状态为 Pausing，并推进执行 //
        imp_->state_ = PlannerState::Pausing;
        imp_->planner_.setTargetSpeedRatio(0.0);
        imp_->planner_.getNextInput(input_pos);

        // 检测是否已完全暂停 //
        if (imp_->planner_.actualSpeedRatio() <= imp_->speed_epsilon_) {
            imp_->state_ = PlannerState::Paused;
            std::copy_n(input_pos, imp_->planner_.inputSize(), imp_->pause_pos_.data());
            return 0;
        }
        return 1;
    }

    // 如果状态不是 Running 或 Pausing，则无法暂停 //
    return -1;
}

auto PlannerPauseController::resume(double* input_pos) -> int {
    static std::vector<aris::plan::SCurveParam> scurve_params;
    static std::vector<double> cur_pos;
    static std::vector<double> max_vel;
    static std::vector<double> max_acc;
    static double resume_t = 0.0;
    static double resume_T = 0.0;

    if(imp_->state_ == PlannerState::Paused){
        // 设置状态为 Resuming，构建scurve，从当前位置到暂停位置，当前位置读取model 的input //
        imp_->state_ = PlannerState::Resuming;

        const int n = imp_->planner_.inputSize();
        scurve_params.resize(n);
        cur_pos.resize(n);
        max_vel.resize(n);
        max_acc.resize(n);

        // 当前位置读取 model 的 input，同时读取电机的速度/加速度上限 //
        auto& model = imp_->planner_.model();
        const auto& sub_ids = imp_->planner_.subModelId();
        model.getSubInputPos(sub_ids.size(), sub_ids.data(), cur_pos.data());
        model.getSubMaxInputVel(sub_ids.size(), sub_ids.data(), max_vel.data());
        model.getSubMaxInputAcc(sub_ids.size(), sub_ids.data(), max_acc.data());

        for (int i = 0; i < n; ++i) {
            scurve_params[i].pa_ = cur_pos[i];              // 当前位置
            scurve_params[i].pb_ = imp_->pause_pos_[i];     // 暂停位置
            scurve_params[i].vc_max_ = max_vel[i] > 0.0 ? max_vel[i] : 1.0;
            scurve_params[i].a_ = max_acc[i] > 0.0 ? max_acc[i] : 1.0;
            scurve_params[i].j_ = scurve_params[i].a_ * 5.0;
        }

        aris::plan::s_scurve_make(n, scurve_params.data(), 0.001);
        resume_t = 0.0;
        resume_T = n > 0 ? scurve_params[0].T_ : 0.0;

        // 恢复目标速度比 //
        imp_->planner_.setTargetSpeedRatio(imp_->target_speed_ratio_);
        return 1;
    }
    else if(imp_->state_ == PlannerState::Resuming) {
        // 从 scurve 中获取具体位置来 resume //
        const int n = imp_->planner_.inputSize();
        for (int i = 0; i < n; ++i) {
            aris::plan::LargeNum p;
            aris::plan::s_scurve_at(scurve_params[i], resume_t, &p);
            input_pos[i] = static_cast<double>(p);
        }
        resume_t += imp_->planner_.dt();

        // 检测是否已完全恢复（scurve 运行结束即回到暂停位置）//
        if (resume_t >= resume_T) {
            imp_->state_ = PlannerState::Running;
            return 0;
        }
        return 1;
    }

    return -1;
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
