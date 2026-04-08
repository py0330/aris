#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <vector>

#include <aris/plan/plan.hpp>
#include <aris/dynamic/mechanism_puma.hpp>

namespace {

struct LimitCheckConfig {
	double forced_min_vel{ -std::numeric_limits<double>::infinity() };
	double forced_max_vel{ std::numeric_limits<double>::infinity() };
	double forced_min_acc{ -std::numeric_limits<double>::infinity() };
	double forced_max_acc{ std::numeric_limits<double>::infinity() };
};

auto check_input_limits(
	aris::dynamic::MultiModel& model,
	aris::Size sub_num,
	const aris::Size* sub_id,
	double dt,
	const std::vector<double>& input,
	const std::vector<double>* prev_input,
	const std::vector<double>* prev_prev_input,
	const char* scenario,
	int step,
	const LimitCheckConfig& cfg) -> void {
	auto input_pos_size = model.subInputPosSize(sub_num, sub_id);
	std::vector<double> min_pos(input_pos_size, 0.0);
	std::vector<double> max_pos(input_pos_size, 0.0);
	std::vector<double> min_vel(input_pos_size, 0.0);
	std::vector<double> max_vel(input_pos_size, 0.0);
	std::vector<double> min_acc(input_pos_size, 0.0);
	std::vector<double> max_acc(input_pos_size, 0.0);

	model.getSubMinInputPos(sub_num, sub_id, min_pos.data());
	model.getSubMaxInputPos(sub_num, sub_id, max_pos.data());
	model.getSubMinInputVel(sub_num, sub_id, min_vel.data());
	model.getSubMaxInputVel(sub_num, sub_id, max_vel.data());
	model.getSubMinInputAcc(sub_num, sub_id, min_acc.data());
	model.getSubMaxInputAcc(sub_num, sub_id, max_acc.data());

	constexpr double pos_tol = 1e-6;
	constexpr double dyn_ratio_tol = 0.01;

	for (aris::Size j = 0; j < input_pos_size; ++j) {
		ASSERT_GE(input[j], min_pos[j] - pos_tol)
			<< "Joint " << j << " position " << input[j] 
			<< " below lower limit " << min_pos[j] << " in " << scenario;
		ASSERT_LE(input[j], max_pos[j] + pos_tol)
			<< "Joint " << j << " position " << input[j] 
			<< " above upper limit " << max_pos[j] << " in " << scenario;

		if (prev_input) {
			double actual_vel = (input[j] - (*prev_input)[j]) / dt;
			double compare_min_vel = std::max(min_vel[j], cfg.forced_min_vel);
			double compare_max_vel = std::min(max_vel[j], cfg.forced_max_vel);
			double vel_min_tol = std::abs(compare_min_vel) * dyn_ratio_tol;
			double vel_max_tol = std::abs(compare_max_vel) * dyn_ratio_tol;
			
			ASSERT_GE(actual_vel, compare_min_vel - vel_min_tol)
				<< "Joint " << j << " velocity " << actual_vel 
				<< " below lower limit " << compare_min_vel << " in " << scenario;
			ASSERT_LE(actual_vel, compare_max_vel + vel_max_tol)
				<< "Joint " << j << " velocity " << actual_vel 
				<< " above upper limit " << compare_max_vel << " in " << scenario;
		}

		if (prev_input && prev_prev_input) {
			double actual_acc = (input[j] - 2.0 * (*prev_input)[j] + (*prev_prev_input)[j]) / (dt * dt);
			double compare_min_acc = std::max(min_acc[j], cfg.forced_min_acc);
			double compare_max_acc = std::min(max_acc[j], cfg.forced_max_acc);
			double acc_min_tol = std::abs(compare_min_acc) * dyn_ratio_tol;
			double acc_max_tol = std::abs(compare_max_acc) * dyn_ratio_tol;
			
			ASSERT_GE(actual_acc, compare_min_acc - acc_min_tol)
				<< "Joint " << j << " acceleration " << actual_acc 
				<< " below lower limit " << compare_min_acc << " in " << scenario;
			ASSERT_LE(actual_acc, compare_max_acc + acc_max_tol)
				<< "Joint " << j << " acceleration " << actual_acc 
				<< " above upper limit " << compare_max_acc << " in " << scenario;
		}
	}
}
    
auto expect_motion_finished(
	aris::plan::PlannerDispacher& dispacher,
	int chanel,
	aris::Size input_size,
	std::int64_t node_id,
	aris::Size sub_num,
	const aris::Size* sub_id,
	const char* scenario,
	const LimitCheckConfig& cfg = LimitCheckConfig{}) -> std::vector<double> {
	auto& model = dispacher.model();
	double dt = dispacher.dt();
	
	std::vector<double> input(input_size, 0.0);
	std::vector<double> prev_input(input_size, 0.0);
	std::vector<double> prev_prev_input(input_size, 0.0);
	bool seen_inserted_node = false;
	bool finished = false;

	for (int i = 0; i < 50000; ++i) {
		auto ret = dispacher.getNextInput(chanel, input.data());

		check_input_limits(
			model,
			sub_num,
			sub_id,
			dt,
			input,
			i > 0 ? &prev_input : nullptr,
			i > 1 ? &prev_prev_input : nullptr,
			scenario,
			i,
			cfg);

		prev_prev_input = prev_input;
		prev_input = input;
		
		EXPECT_GE(ret, 0) << "PlannerDispacher getNextInput returned negative ret in " << scenario;
		if (ret < 0) break;
		
		if (ret == node_id) {
			seen_inserted_node = true;
		}
		if (ret == 0) {
			finished = true;
			break;
		}
	}

	EXPECT_TRUE(seen_inserted_node) << "PlannerDispacher did not observe inserted node id in " << scenario;
	EXPECT_TRUE(finished) << "PlannerDispacher motion did not finish in expected iterations in " << scenario;
	EXPECT_GE(dispacher.tgRet(chanel), 0) << "PlannerDispacher tgRet should not be negative in " << scenario;
	
	auto ik_ret = dispacher.ikRet(chanel);
	if (ik_ret < 0) {
		ADD_FAILURE() << "PlannerDispacher ikRet < 0 in " << scenario << ", ikRet=" << ik_ret;
	}
	
	if (ik_ret >= 0) {
		auto input_pos_size = model.subInputPosSize(sub_num, sub_id);
		std::vector<double> model_input(input_pos_size, 0.0);
		model.getSubInputPos(sub_num, sub_id, model_input.data());
		for (aris::Size i = 0; i < input_pos_size; ++i) {
			EXPECT_NEAR(input[i], model_input[i], 1e-6)
				<< "PlannerDispacher final input does not match model input in " << scenario
				<< ", joint=" << i;
		}
	}

	return input;
}

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

} // namespace

class PlannerDispacherTest : public ::testing::Test {
protected:
	void SetUp() override {
		multi_model = create_single_puma_model();
		sub_num = aris::Size(1);
		sub_id = aris::Size(0);
		
		dispacher.setModel(multi_model);
		dispacher.setChanelSize(1);
		dispacher.setDt(0.001);
		dispacher.init();
		
		lock_ret = dispacher.tryLockChanel(0, { 0 });
		ASSERT_EQ(lock_ret, 1) << "Failed to lock planner dispacher chanel 0";
	}

	void TearDown() override {
		if (dispacher.tryLockChanel(0, { 0 }) > 0) {
			dispacher.releaseChanel(0);
		}
	}

	aris::dynamic::MultiModel multi_model;
	aris::plan::PlannerDispacher dispacher;
	aris::Size sub_num;
	aris::Size sub_id;
	int lock_ret = 0;
};

// =============================================================================
// Test: Single model, basic lock and release
// =============================================================================

TEST(PlannerDispacherBasicTest, SingleModelLockRelease) {
	auto multi_model = create_single_puma_model();
	auto sub_num = aris::Size(1);
	auto sub_id = aris::Size(0);

	aris::plan::PlannerDispacher dispacher;
	dispacher.setModel(multi_model);
	dispacher.setChanelSize(1);
	dispacher.setDt(0.001);
	dispacher.init();

	auto lock_ret = dispacher.tryLockChanel(0, { 0 });
	EXPECT_EQ(lock_ret, 1);

	auto release_ret = dispacher.releaseChanel(0);
	EXPECT_EQ(release_ret, 0);

	release_ret = dispacher.releaseChanel(0);
	EXPECT_LT(release_ret, 0) << "Second release should return negative value (error state)";
}

// =============================================================================
// Test: Line motion insertion and execution
// =============================================================================

TEST_F(PlannerDispacherTest, InsertLineMotion) {
	auto output_pos_size = multi_model.subOutputPosSize(sub_num, &sub_id);
	auto output_vel_size = multi_model.subOutputPosMagSize(sub_num, &sub_id);

	std::vector<double> tw_pos(output_pos_size, 0.0);
	std::vector<double> vel(output_vel_size, 5.0);
	std::vector<double> acc(output_vel_size, 20.0);
	std::vector<double> jerk(output_vel_size, 100.0);
	std::vector<double> zone(output_vel_size, 0.001);

	multi_model.getSubOutputPos(sub_num, &sub_id, tw_pos.data());
	tw_pos.at(0) += 0.02;

	auto node_id = dispacher.insertLinePos(0, "", "", tw_pos.data(), vel.data(), acc.data(), jerk.data(), zone.data());
	EXPECT_GT(node_id, 0) << "Line motion node_id should be positive";

	dispacher.updateInsertPos(0);
	auto final_input = expect_motion_finished(dispacher, 0, multi_model.inputPosSize(), node_id, sub_num, &sub_id, "line-motion");

	auto release_ret = dispacher.releaseChanel(0);
	EXPECT_EQ(release_ret, 0);
}

// =============================================================================
// Test: Circular motion insertion and execution
// =============================================================================

TEST_F(PlannerDispacherTest, InsertCircleMotion) {
	auto output_pos_size = multi_model.subOutputPosSize(sub_num, &sub_id);
	auto output_vel_size = multi_model.subOutputPosMagSize(sub_num, &sub_id);

	std::vector<double> vel(output_vel_size, 5.0);
	std::vector<double> acc(output_vel_size, 20.0);
	std::vector<double> jerk(output_vel_size, 100.0);
	std::vector<double> zone(output_vel_size, 0.001);
	std::vector<double> tw_mid_pos(output_pos_size, 0.0);
	std::vector<double> tw_target_pos(output_pos_size, 0.0);

	multi_model.getSubOutputPos(sub_num, &sub_id, tw_mid_pos.data());
	tw_target_pos = tw_mid_pos;
	tw_mid_pos.at(0) += 0.01;
	tw_mid_pos.at(1) += 0.01;
	tw_target_pos.at(1) += 0.02;

	auto node_id = dispacher.insertCirclePos(0, "", "", tw_target_pos.data(), tw_mid_pos.data(), vel.data(), acc.data(), jerk.data(), zone.data());
	EXPECT_GT(node_id, 0) << "Circle motion node_id should be positive";

	dispacher.updateInsertPos(0);
	expect_motion_finished(dispacher, 0, multi_model.inputPosSize(), node_id, sub_num, &sub_id, "circle-motion");

	auto release_ret = dispacher.releaseChanel(0);
	EXPECT_EQ(release_ret, 0);
}

// =============================================================================
// Test: Joint motion insertion and execution
// =============================================================================

TEST_F(PlannerDispacherTest, InsertMoveJMotion) {
	auto output_pos_size = multi_model.subOutputPosSize(sub_num, &sub_id);
	auto input_pos_size = multi_model.subInputPosSize(sub_num, &sub_id);

	std::vector<double> tw_pos(output_pos_size, 0.0);
	std::vector<double> joint_vel(input_pos_size, 5.0);
	std::vector<double> joint_acc(input_pos_size, 20.0);
	std::vector<double> joint_jerk(input_pos_size, 100.0);
	std::vector<double> joint_zone(input_pos_size, 0.001);
	std::int64_t which_root = 0;

	multi_model.getSubOutputPos(sub_num, &sub_id, tw_pos.data());
	tw_pos.at(2) += 0.02;

	auto node_id = dispacher.insertMoveJPos(0, "", "", tw_pos.data(), joint_vel.data(), joint_acc.data(), joint_jerk.data(), joint_zone.data(), &which_root);
	EXPECT_GT(node_id, 0) << "MoveJ motion node_id should be positive";

	dispacher.updateInsertPos(0);
	expect_motion_finished(dispacher, 0, multi_model.inputPosSize(), node_id, sub_num, &sub_id, "movej-motion");

	auto release_ret = dispacher.releaseChanel(0);
	EXPECT_EQ(release_ret, 0);
}

// =============================================================================
// Test: Absolute joint motion execution
// =============================================================================

TEST_F(PlannerDispacherTest, InsertMoveAbsJMotion) {
	auto input_pos_size = multi_model.subInputPosSize(sub_num, &sub_id);

	std::vector<double> init_joint_pos(multi_model.inputPosSize(), 0.0);
	init_joint_pos.at(0) = 0.1;
	init_joint_pos.at(1) = -0.05;
	multi_model.setSubInputPos(sub_num, &sub_id, init_joint_pos.data());
	multi_model.subForwardKinematics(sub_num, &sub_id);

	// Re-initialize dispacher after changing model state
	dispacher.setModel(multi_model);
	dispacher.setChanelSize(1);
	dispacher.setDt(0.001);
	dispacher.init();
	lock_ret = dispacher.tryLockChanel(0, { 0 });

	std::vector<double> joint_pos(input_pos_size, 0.0);
	std::vector<double> joint_vel(input_pos_size, 5.0);
	std::vector<double> joint_acc(input_pos_size, 20.0);
	std::vector<double> joint_jerk(input_pos_size, 100.0);
	std::vector<double> joint_zone(input_pos_size, 0.001);
	
	multi_model.getSubInputPos(sub_num, &sub_id, joint_pos.data());
	joint_pos.at(0) += 0.03;
	joint_pos.at(1) -= 0.02;

	auto node_id = dispacher.insertMoveAbsJPos(0, joint_pos.data(), joint_vel.data(), joint_acc.data(), joint_jerk.data(), joint_zone.data());
	EXPECT_GT(node_id, 0) << "MoveAbsJ motion node_id should be positive";

	dispacher.updateInsertPos(0);
	auto final_input = expect_motion_finished(dispacher, 0, multi_model.inputPosSize(), node_id, sub_num, &sub_id, "moveabsj-motion");
	
	EXPECT_NEAR(final_input.at(0), joint_pos.at(0), 1e-4) << "Final joint 0 position mismatch";
	EXPECT_NEAR(final_input.at(1), joint_pos.at(1), 1e-4) << "Final joint 1 position mismatch";

	auto release_ret = dispacher.releaseChanel(0);
	EXPECT_EQ(release_ret, 0);
}

// =============================================================================
// Test: transfer matrix is left-multiplied to planner output
// =============================================================================

TEST_F(PlannerDispacherTest, TransferMatrixAppliedOnOutput) {
	auto input_pos_size = static_cast<int>(multi_model.subInputPosSize(sub_num, &sub_id));
	ASSERT_GE(input_pos_size, 1);

	// Reconfigure transfer matrix and re-lock channel so the new matrix is bound.
	EXPECT_EQ(dispacher.releaseChanel(0), 0);

	std::vector<double> mat_data(input_pos_size * input_pos_size, 0.0);
	for (int i = 0; i < input_pos_size; ++i) {
		mat_data[i * input_pos_size + i] = 1.0;
	}
	mat_data[0] = 1.2; // scale joint-0 output to make transform observable

	dispacher.transferMatrice().clear();
	dispacher.transferMatrice().emplace_back(input_pos_size, input_pos_size, mat_data.data());

	lock_ret = dispacher.tryLockChanel(0, { 0 });
	ASSERT_EQ(lock_ret, 1);

	std::vector<double> joint_pos(input_pos_size, 0.0);
	std::vector<double> joint_vel(input_pos_size, 3.0);
	std::vector<double> joint_acc(input_pos_size, 20.0);
	std::vector<double> joint_jerk(input_pos_size, 100.0);
	std::vector<double> joint_zone(input_pos_size, 0.0);

	multi_model.getSubInputPos(sub_num, &sub_id, joint_pos.data());
	joint_pos[0] += 0.03;

	auto node_id = dispacher.insertMoveAbsJPos(0, joint_pos.data(), joint_vel.data(), joint_acc.data(), joint_jerk.data(), joint_zone.data());
	ASSERT_GT(node_id, 0);

	dispacher.updateInsertPos(0);

	std::vector<double> output(input_pos_size, 0.0);
	std::vector<double> raw_input(input_pos_size, 0.0);
	bool finished = false;
	bool saw_node = false;
	bool transform_effect_observed = false;

	for (int i = 0; i < 50000; ++i) {
		auto ret = dispacher.getNextInput(0, output.data());
		ASSERT_GE(ret, 0);

		multi_model.getSubInputPos(sub_num, &sub_id, raw_input.data());

		if (std::abs(output[0] - raw_input[0]) > 1e-8) {
			transform_effect_observed = true;
		}

		if (ret == node_id) {
			saw_node = true;
		}
		if (ret == 0) {
			finished = true;
			break;
		}
	}

	EXPECT_TRUE(saw_node);
	EXPECT_TRUE(finished);
	EXPECT_TRUE(transform_effect_observed);
	EXPECT_NEAR(output[0], 1.2 * joint_pos[0], 1e-6);
	for (int j = 1; j < input_pos_size; ++j) {
		EXPECT_NEAR(output[j], joint_pos[j], 1e-6)
			<< "final transformed output mismatch at joint " << j;
	}

	auto release_ret = dispacher.releaseChanel(0);
	EXPECT_EQ(release_ret, 0);
}
