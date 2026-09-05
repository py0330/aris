#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

#include <aris/plan/plan.hpp>
#include <aris/dynamic/mechanism_puma.hpp>

namespace {

auto create_single_puma_model() -> aris::dynamic::MultiModel {
	aris::dynamic::PumaParam puma_param;
	puma_param.d1 = 0.3;
	puma_param.a1 = 0.1;
	puma_param.a2 = 0.4;
	puma_param.a3 = 0.05;
	puma_param.d3 = 0.0;
	puma_param.d4 = 0.35;
	puma_param.install_method = 0;

	auto puma = aris::dynamic::createModelPuma(puma_param);

	aris::dynamic::MultiModel multi_model;
	multi_model.subModels().add(puma.release());
	multi_model.init();
	return multi_model;
}

auto expect_motion_finished(
	aris::plan::MultimodelPlanner &planner,
	aris::dynamic::MultiModel &model,
	std::int64_t expected_node,
	const char *scenario) -> std::vector<double> {
	std::vector<double> input(model.inputPosSize(), 0.0);
	std::vector<double> prev_input(model.inputPosSize(), 0.0);
	std::vector<double> prev_prev_input(model.inputPosSize(), 0.0);
	bool seen_node = false;
	bool finished = false;

	for (int i = 0; i < 50000; ++i) {
		auto ret = planner.runOneStep(input.data());
		EXPECT_GE(ret, 0) << "Planner returned negative ret in " << scenario;
		if (ret < 0) break;

		if (ret == expected_node) seen_node = true;
		if (ret == 0) {
			finished = true;
			break;
		}

		prev_prev_input = prev_input;
		prev_input = input;
	}

	EXPECT_TRUE(seen_node) << "Inserted node id was not observed in " << scenario;
	EXPECT_TRUE(finished) << "Planner did not finish in expected iterations in " << scenario;
	EXPECT_GE(planner.tgRet(), 0) << "tgRet should not be negative in " << scenario;
	EXPECT_GE(planner.ikRet(), 0) << "ikRet should not be negative in " << scenario;

	return input;
}

// 插入一条标准直线运动并提交重规划，返回节点 id
auto insert_standard_line(aris::plan::MultimodelPlanner &planner) -> std::int64_t {
	std::vector<std::pair<std::string, std::string>> tw{
		{ "PumaModel.EE.tool0", "PumaModel.ground.wobj0" }
	};
	double target_ee[6]{ 0.45, 0.1, 0.75, aris::PI / 4.0, aris::PI / 2.0, aris::PI / 4.0 };
	double vel[2]{ 0.2, 0.2 };
	double acc[2]{ 1.0, 1.0 };
	double jerk[2]{ 10.0, 10.0 };
	double zone[2]{ 0.0, 0.0 };
	auto node = planner.insertLinePos(tw, target_ee, vel, acc, jerk, zone);
	planner.updateInsertPos();
	return node;
}

// 推进 runOneStep 指定步数；任何一步返回负数视为失败
auto run_steps(aris::plan::MultimodelPlanner &planner, int steps) -> bool {
	std::vector<double> p(planner.inputSize(), 0.0);
	for (int i = 0; i < steps; ++i) {
		if (planner.runOneStep(p.data()) < 0) return false;
	}
	return true;
}

// 持续暂停直到返回 0（Paused），paused_pos 为暂停位置
auto pause_until_paused(aris::plan::MultimodelPlanner &planner, std::vector<double> &paused_pos) -> bool {
	std::vector<double> p(planner.inputSize(), 0.0);
	for (int i = 0; i < 50000; ++i) {
		auto ret = planner.pauseOneStep(p.data());
		if (ret < 0) return false;
		if (ret == 0) { paused_pos = p; return true; }
	}
	return false;
}

// 持续停止直到返回 0（Uninitialized）；before_final 为停止前最后一步位置，final_pos 为最终位置
auto stop_until_stopped(aris::plan::MultimodelPlanner &planner, std::vector<double> &before_final, std::vector<double> &final_pos) -> bool {
	for (int i = 0; i < 50000; ++i) {
		auto ret = planner.stopOneStep(final_pos.data());
		if (ret < 0) return false;
		if (ret == 0) {
			if (i == 0) before_final = final_pos;
			return true;
		}
		before_final = final_pos;
	}
	return false;
}

// 持续移动到目标直到返回 0；final_pos 为最后输出
auto move_to_target_until_done(aris::plan::MultimodelPlanner &planner, std::vector<double> &final_pos) -> bool {
	for (int i = 0; i < 50000; ++i) {
		auto ret = planner.moveToTargetOneStep(final_pos.data());
		if (ret < 0) return false;
		if (ret == 0) return true;
	}
	return false;
}

} // namespace

class MultimodelPlannerTest : public ::testing::Test {
protected:
	void SetUp() override {
		multi_model = create_single_puma_model();
		multi_model.tools().clear();
		multi_model.wobjs().clear();
		multi_model.tools().push_back(multi_model.findMarker("PumaModel.EE.tool0"));
		multi_model.tools().push_back(multi_model.findMarker("PumaModel.EE.tool1"));
		multi_model.wobjs().push_back(multi_model.findMarker("PumaModel.ground.wobj0"));
		multi_model.wobjs().push_back(multi_model.findMarker("PumaModel.ground.wobj1"));

		planner.setModel(multi_model);
		planner.setDt(0.001);
		planner.setSubModelId({ 0 });

		auto input_size = multi_model.inputPosSize();
		std::vector<double> max_vel(input_size, 3.14);
		std::vector<double> min_vel(input_size, -3.14);
		std::vector<double> max_acc(input_size, 31.4);
		std::vector<double> min_acc(input_size, -31.4);

		planner.setMaxVel(aris::core::Matrix(input_size, 1, max_vel.data()));
		planner.setMinVel(aris::core::Matrix(input_size, 1, min_vel.data()));
		planner.setMaxAcc(aris::core::Matrix(input_size, 1, max_acc.data()));
		planner.setMinAcc(aris::core::Matrix(input_size, 1, min_acc.data()));

		planner.allocateMemory();
		planner.init();
	}

	aris::dynamic::MultiModel multi_model;
	aris::plan::MultimodelPlanner planner;
};

TEST_F(MultimodelPlannerTest, ToolWobjSelectorRoundTripTwToEeToTw) {
	auto *tool = multi_model.findMarker("PumaModel.EE.tool0");
	auto *wobj = multi_model.findMarker("PumaModel.ground.wobj0");
	ASSERT_NE(tool, nullptr);
	ASSERT_NE(wobj, nullptr);

	aris::plan::ToolWobjSelector selector;
	selector.setModel(multi_model);
	selector.setSubModelId({ 0 });

	aris::dynamic::Marker *tools[1]{ tool };
	aris::dynamic::Marker *wobjs[1]{ wobj };
	ASSERT_EQ(selector.selectTw(tools, wobjs), 0);

	double tw_pos[6]{ 0.45, 0.1, 0.75, aris::PI / 4.0, aris::PI / 2.0, aris::PI / 4.0 };
	double ee_pos[6]{};
	double tw_roundtrip[6]{};

	selector.setTwPos(tw_pos);
	selector.getEePos(ee_pos);
	selector.setEePos(ee_pos);
	selector.getTwPos(tw_roundtrip);

	for (int i = 0; i < 3; ++i) {
		EXPECT_NEAR(tw_roundtrip[i], tw_pos[i], 1e-9) << "Position mismatch at index " << i;
	}

	double pm_expected[16]{};
	double pm_roundtrip[16]{};
	aris::dynamic::s_pe2pm(tw_pos, pm_expected, "321");
	aris::dynamic::s_pe2pm(tw_roundtrip, pm_roundtrip, "321");
	for (int i = 0; i < 16; ++i) {
		EXPECT_NEAR(pm_roundtrip[i], pm_expected[i], 1e-10)
			<< "Pose matrix mismatch at index " << i;
	}
}

TEST_F(MultimodelPlannerTest, InsertLineMotionFinishes) {
	auto *tool = multi_model.findMarker("PumaModel.EE.tool0");
	auto *wobj = multi_model.findMarker("PumaModel.ground.wobj0");
	ASSERT_NE(tool, nullptr);
	ASSERT_NE(wobj, nullptr);

	std::vector<std::pair<std::string, std::string>> tw{
		{ "PumaModel.EE.tool0", "PumaModel.ground.wobj0" }
	};

	double target_ee[6]{ 0.45, 0.1, 0.75, aris::PI / 4.0, aris::PI / 2.0, aris::PI / 4.0 };
	double vel[2]{ 0.2, 0.2 };
	double acc[2]{ 1.0, 1.0 };
	double jerk[2]{ 10.0, 10.0 };
	double zone[2]{ 0.0, 0.0 };

	auto node = planner.insertLinePos(tw, target_ee, vel, acc, jerk, zone);
	EXPECT_GT(node, 0);

	planner.updateInsertPos();
	auto final_input = expect_motion_finished(planner, multi_model, node, "line-motion");
	EXPECT_EQ(final_input.size(), static_cast<std::size_t>(multi_model.inputPosSize()));
}

TEST_F(MultimodelPlannerTest, InsertCircleMotionFinishes) {
	auto *tool = multi_model.findMarker("PumaModel.EE.tool0");
	auto *wobj = multi_model.findMarker("PumaModel.ground.wobj0");
	ASSERT_NE(tool, nullptr);
	ASSERT_NE(wobj, nullptr);

	std::vector<std::pair<std::string, std::string>> tw{
		{ "PumaModel.EE.tool0", "PumaModel.ground.wobj0" }
	};

	double mid_ee[6]{ 0.45, 0.0, 0.70, aris::PI / 4.0, aris::PI / 2.0, aris::PI / 4.0 };
	double target_ee[6]{ 0.45, -0.1, 0.75, aris::PI / 4.0, aris::PI / 2.0, aris::PI / 4.0 };
	double vel[2]{ 0.2, 0.2 };
	double acc[2]{ 1.0, 1.0 };
	double jerk[2]{ 10.0, 10.0 };
	double zone[2]{ 0.0, 0.0 };

	auto node = planner.insertCirclePos(tw, target_ee, mid_ee, vel, acc, jerk, zone);
	EXPECT_GT(node, 0);

	planner.updateInsertPos();
	auto final_input = expect_motion_finished(planner, multi_model, node, "circle-motion");
	EXPECT_EQ(final_input.size(), static_cast<std::size_t>(multi_model.inputPosSize()));
}

TEST_F(MultimodelPlannerTest, RunOneStepTransitionsToRunningThenIdle) {
	ASSERT_GT(insert_standard_line(planner), 0);
	EXPECT_EQ(planner.state(), aris::plan::PlannerState::Idle);

	std::vector<double> p(planner.inputSize(), 0.0);
	auto ret = planner.runOneStep(p.data());
	ASSERT_GE(ret, 0);
	EXPECT_EQ(planner.state(), aris::plan::PlannerState::Running);

	bool finished = false;
	for (int i = 0; i < 50000; ++i) {
		ret = planner.runOneStep(p.data());
		ASSERT_GE(ret, 0);
		if (ret == 0) { finished = true; break; }
	}
	EXPECT_TRUE(finished);
	EXPECT_EQ(planner.state(), aris::plan::PlannerState::Idle);

	// 队列已空：runOneStep 仍返回 0 并保持 Idle
	EXPECT_EQ(planner.runOneStep(p.data()), 0);
	EXPECT_EQ(planner.state(), aris::plan::PlannerState::Idle);
}

TEST_F(MultimodelPlannerTest, PauseFreezesMotionAndHoldsPosition) {
	ASSERT_GT(insert_standard_line(planner), 0);
	ASSERT_TRUE(run_steps(planner, 200));
	EXPECT_EQ(planner.state(), aris::plan::PlannerState::Running);

	std::vector<double> pause_pos;
	ASSERT_TRUE(pause_until_paused(planner, pause_pos));
	EXPECT_EQ(planner.state(), aris::plan::PlannerState::Paused);

	// 已暂停后 pauseOneStep 返回 0 且输出保持不变
	std::vector<double> p(planner.inputSize(), 0.0);
	EXPECT_EQ(planner.pauseOneStep(p.data()), 0);
	for (std::size_t i = 0; i < p.size(); ++i) {
		EXPECT_NEAR(p[i], pause_pos[i], 1e-12);
	}
}

TEST_F(MultimodelPlannerTest, ResumeReturnsToPausePositionThenContinues) {
	ASSERT_GT(insert_standard_line(planner), 0);
	ASSERT_TRUE(run_steps(planner, 200));

	std::vector<double> pause_pos;
	ASSERT_TRUE(pause_until_paused(planner, pause_pos));
	EXPECT_EQ(planner.state(), aris::plan::PlannerState::Paused);

	// 模拟暂停期间机器人被移动
	aris::Size sub_id = 0;
	std::vector<double> moved = pause_pos;
	for (auto &v : moved) v += 0.05;
	multi_model.setSubInputPos(1, &sub_id, moved.data());

	// 首次 resume：Paused → Resuming
	std::vector<double> final_pos(planner.inputSize(), 0.0);
	ASSERT_GT(planner.resumeOneStep(final_pos.data()), 0);
	EXPECT_EQ(planner.state(), aris::plan::PlannerState::Resuming);

	// Resuming 状态：run / pause 均不可调用
	EXPECT_EQ(planner.runOneStep(final_pos.data()), -1);
	EXPECT_EQ(planner.pauseOneStep(final_pos.data()), -1);

	// 继续恢复直到 Running
	bool resumed = false;
	for (int i = 0; i < 50000; ++i) {
		auto ret = planner.resumeOneStep(final_pos.data());
		ASSERT_GE(ret, 0);
		if (ret == 0) { resumed = true; break; }
	}
	ASSERT_TRUE(resumed);
	EXPECT_EQ(planner.state(), aris::plan::PlannerState::Running);

	// 应回到暂停位置
	for (std::size_t i = 0; i < final_pos.size(); ++i) {
		EXPECT_NEAR(final_pos[i], pause_pos[i], 1e-3) << "resume should return to pause position at dim " << i;
	}

	// 继续运行直到完成
	bool finished = false;
	std::vector<double> p(planner.inputSize(), 0.0);
	for (int i = 0; i < 50000; ++i) {
		auto ret = planner.runOneStep(p.data());
		ASSERT_GE(ret, 0);
		if (ret == 0) { finished = true; break; }
	}
	EXPECT_TRUE(finished);
	EXPECT_EQ(planner.state(), aris::plan::PlannerState::Idle);
}

TEST_F(MultimodelPlannerTest, StopDeceleratesToZeroVelocity) {
	ASSERT_GT(insert_standard_line(planner), 0);
	ASSERT_TRUE(run_steps(planner, 200));
	EXPECT_EQ(planner.state(), aris::plan::PlannerState::Running);

	std::vector<double> before_final(planner.inputSize(), 0.0);
	std::vector<double> final_pos(planner.inputSize(), 0.0);
	ASSERT_TRUE(stop_until_stopped(planner, before_final, final_pos));
	EXPECT_EQ(planner.state(), aris::plan::PlannerState::Uninitialized);

	// 速度已降为 0：最后两步位置几乎相同
	for (std::size_t i = 0; i < final_pos.size(); ++i) {
		EXPECT_NEAR(final_pos[i], before_final[i], 1e-9) << "stop should zero the velocity at dim " << i;
	}

	// 已停止后 stopOneStep 直接返回 0
	std::vector<double> p(planner.inputSize(), 0.0);
	EXPECT_EQ(planner.stopOneStep(p.data()), 0);
}

TEST_F(MultimodelPlannerTest, StepFunctionsRejectInvalidStates) {
	std::vector<double> p(planner.inputSize(), 0.0);

	// init 后 → Idle：resume 不可调用，pause 无操作，stop → Uninitialized
	EXPECT_EQ(planner.state(), aris::plan::PlannerState::Idle);
	EXPECT_EQ(planner.resumeOneStep(p.data()), -1);
	EXPECT_EQ(planner.pauseOneStep(p.data()), 0);
	EXPECT_EQ(planner.stopOneStep(p.data()), 0);
	EXPECT_EQ(planner.state(), aris::plan::PlannerState::Uninitialized);

	// Uninitialized：run/pause/resume 均不可调用，stop 为无操作
	EXPECT_EQ(planner.runOneStep(p.data()), -1);
	EXPECT_EQ(planner.pauseOneStep(p.data()), -1);
	EXPECT_EQ(planner.resumeOneStep(p.data()), -1);
	EXPECT_EQ(planner.stopOneStep(p.data()), 0);
	EXPECT_EQ(planner.state(), aris::plan::PlannerState::Uninitialized);

	// 重新初始化 → Idle
	planner.init();
	EXPECT_EQ(planner.state(), aris::plan::PlannerState::Idle);

	// 启动运动 → Running
	ASSERT_GT(insert_standard_line(planner), 0);
	ASSERT_GT(planner.runOneStep(p.data()), 0);
	EXPECT_EQ(planner.state(), aris::plan::PlannerState::Running);

	// Running：resume 不可调用
	EXPECT_EQ(planner.resumeOneStep(p.data()), -1);

	// 暂停到 Paused
	std::vector<double> pause_pos;
	ASSERT_TRUE(pause_until_paused(planner, pause_pos));
	EXPECT_EQ(planner.state(), aris::plan::PlannerState::Paused);

	// Paused：run 不可调用
	EXPECT_EQ(planner.runOneStep(p.data()), -1);

	// 停止（任意状态可停）→ 最终 Uninitialized
	std::vector<double> before(planner.inputSize(), 0.0), curr(planner.inputSize(), 0.0);
	ASSERT_TRUE(stop_until_stopped(planner, before, curr));
	EXPECT_EQ(planner.state(), aris::plan::PlannerState::Uninitialized);
}

TEST_F(MultimodelPlannerTest, MoveToTargetFromIdleReturnsToIdle) {
	// 读取当前关节位置作为起点
	aris::Size sub_id = 0;
	std::vector<double> start(planner.inputSize(), 0.0);
	multi_model.getSubInputPos(1, &sub_id, start.data());

	std::vector<double> target = start;
	for (auto &v : target) v += 0.1;
	planner.setMoveTarget(target.data());

	EXPECT_EQ(planner.state(), aris::plan::PlannerState::Idle);

	std::vector<double> p(planner.inputSize(), 0.0);
	EXPECT_GT(planner.moveToTargetOneStep(p.data()), 0);
	EXPECT_EQ(planner.state(), aris::plan::PlannerState::MovingToTarget);

	ASSERT_TRUE(move_to_target_until_done(planner, p));
	EXPECT_EQ(planner.state(), aris::plan::PlannerState::Idle);

	for (std::size_t i = 0; i < target.size(); ++i) {
		EXPECT_NEAR(p[i], target[i], 1e-3) << "move target mismatch at dim " << i;
	}
}

TEST_F(MultimodelPlannerTest, MoveToTargetFromPausedReturnsToPaused) {
	ASSERT_GT(insert_standard_line(planner), 0);
	ASSERT_TRUE(run_steps(planner, 200));

	std::vector<double> pause_pos;
	ASSERT_TRUE(pause_until_paused(planner, pause_pos));
	EXPECT_EQ(planner.state(), aris::plan::PlannerState::Paused);

	std::vector<double> target = pause_pos;
	for (auto &v : target) v += 0.1;
	planner.setMoveTarget(target.data());

	std::vector<double> p(planner.inputSize(), 0.0);
	EXPECT_GT(planner.moveToTargetOneStep(p.data()), 0);
	EXPECT_EQ(planner.state(), aris::plan::PlannerState::MovingToTarget);

	// MovingToTarget 状态：run/pause/resume 均不可调用
	EXPECT_EQ(planner.runOneStep(p.data()), -1);
	EXPECT_EQ(planner.pauseOneStep(p.data()), -1);
	EXPECT_EQ(planner.resumeOneStep(p.data()), -1);

	ASSERT_TRUE(move_to_target_until_done(planner, p));
	EXPECT_EQ(planner.state(), aris::plan::PlannerState::Paused);

	for (std::size_t i = 0; i < target.size(); ++i) {
		EXPECT_NEAR(p[i], target[i], 1e-3) << "move target mismatch at dim " << i;
	}
}
