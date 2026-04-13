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
		if (input[j] < min_pos[j] - pos_tol || input[j] > max_pos[j] + pos_tol) {
			throw std::runtime_error(
				std::string("PlannerDispacher input position limit exceeded in ") + scenario
				+ ", step=" + std::to_string(step)
				+ ", joint=" + std::to_string(j)
				+ ", value=" + std::to_string(input[j])
				+ ", range=[" + std::to_string(min_pos[j]) + "," + std::to_string(max_pos[j]) + "]");
		}

		if (prev_input) {
			double actual_vel = (input[j] - (*prev_input)[j]) / dt;
			double compare_min_vel = std::max(min_vel[j], cfg.forced_min_vel);
			double compare_max_vel = std::min(max_vel[j], cfg.forced_max_vel);
			double vel_min_tol = std::abs(compare_min_vel) * dyn_ratio_tol;
			double vel_max_tol = std::abs(compare_max_vel) * dyn_ratio_tol;
			if (actual_vel < compare_min_vel - vel_min_tol || actual_vel > compare_max_vel + vel_max_tol) {
				throw std::runtime_error(
					std::string("PlannerDispacher input velocity limit exceeded in ") + scenario
					+ ", step=" + std::to_string(step)
					+ ", joint=" + std::to_string(j)
					+ ", value=" + std::to_string(actual_vel)
					+ ", range=[" + std::to_string(compare_min_vel) + "," + std::to_string(compare_max_vel) + "]");
			}
		}

		if (prev_input && prev_prev_input) {
			double actual_acc = (input[j] - 2.0 * (*prev_input)[j] + (*prev_prev_input)[j]) / (dt * dt);
			double compare_min_acc = std::max(min_acc[j], cfg.forced_min_acc);
			double compare_max_acc = std::min(max_acc[j], cfg.forced_max_acc);
			double acc_min_tol = std::abs(compare_min_acc) * dyn_ratio_tol;
			double acc_max_tol = std::abs(compare_max_acc) * dyn_ratio_tol;
			if (actual_acc < compare_min_acc - acc_min_tol || actual_acc > compare_max_acc + acc_max_tol) {
				throw std::runtime_error(
					std::string("PlannerDispacher input acceleration limit exceeded in ") + scenario
					+ ", step=" + std::to_string(step)
					+ ", joint=" + std::to_string(j)
					+ ", value=" + std::to_string(actual_acc)
					+ ", range=[" + std::to_string(compare_min_acc) + "," + std::to_string(compare_max_acc) + "]");
			}
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
		
		if (ret < 0) {
			throw std::runtime_error(std::string("PlannerDispacher getNextInput returned negative ret in ") + scenario);
		}
		if (ret == node_id) {
			seen_inserted_node = true;
		}
		if (ret == 0) {
			finished = true;
			break;
		}
	}

	if (!seen_inserted_node) {
		throw std::runtime_error(std::string("PlannerDispacher did not observe inserted node id in ") + scenario);
	}
	if (!finished) {
		throw std::runtime_error(std::string("PlannerDispacher motion did not finish in expected iterations in ") + scenario);
	}
	if (dispacher.tgRet(chanel) < 0) {
		throw std::runtime_error(std::string("PlannerDispacher tgRet should not be negative in ") + scenario);
	}
	auto ik_ret = dispacher.ikRet(chanel);
	if (ik_ret < 0) {
		std::cout << "NOTE: PlannerDispacher ikRet < 0 in " << scenario << ", ikRet=" << ik_ret << std::endl;
	}
	if (ik_ret >= 0) {
		auto input_pos_size = model.subInputPosSize(sub_num, sub_id);
		std::vector<double> model_input(input_pos_size, 0.0);
		model.getSubInputPos(sub_num, sub_id, model_input.data());
		for (aris::Size i = 0; i < input_pos_size; ++i) {
			if (std::abs(input[i] - model_input[i]) > 1e-6) {
				throw std::runtime_error(
					std::string("PlannerDispacher final input does not match model input in ") + scenario
					+ ", joint=" + std::to_string(i)
					+ ", final=" + std::to_string(input[i])
					+ ", model=" + std::to_string(model_input[i]));
			}
		}
	}

	return input;
}

auto expect_motion_sequence_finished(
	aris::plan::PlannerDispacher& dispacher,
	int chanel,
	aris::Size input_size,
	const std::vector<std::int64_t>& node_ids,
	aris::Size sub_num,
	const aris::Size* sub_id,
	const char* scenario,
	const LimitCheckConfig& cfg = LimitCheckConfig{}) -> std::vector<double> 
    {
	auto& model = dispacher.model();
	double dt = dispacher.dt();

	std::vector<double> input(input_size, 0.0);
	std::vector<double> prev_input(input_size, 0.0);
	std::vector<double> prev_prev_input(input_size, 0.0);
	std::vector<bool> seen(node_ids.size(), false);
	std::vector<std::int64_t> observed_ids;
	bool finished = false;

	for (int i = 0; i < 100000; ++i) {
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
		
		if (ret < 0) {
			throw std::runtime_error(std::string("PlannerDispacher getNextInput returned negative ret in ") + scenario);
		}
		if (std::find(observed_ids.begin(), observed_ids.end(), ret) == observed_ids.end()) {
			observed_ids.push_back(ret);
		}
		for (std::size_t index = 0; index < node_ids.size(); ++index) {
			if (ret == node_ids[index]) {
				seen[index] = true;
			}
		}
		if (ret == 0) {
			finished = true;
			break;
		}
	}

	for (std::size_t index = 0; index < node_ids.size(); ++index) {
		if (!seen[index]) {
			std::string observed_text;
			for (std::size_t i = 0; i < observed_ids.size(); ++i) {
				if (i > 0) observed_text += ",";
				observed_text += std::to_string(observed_ids[i]);
			}
			throw std::runtime_error(
				std::string("PlannerDispacher did not observe queued node id ")
				+ std::to_string(node_ids[index])
				+ " in " + scenario
				+ ", observed ids: [" + observed_text + "]");
		}
	}
	if (!finished) {
		throw std::runtime_error(std::string("PlannerDispacher queued motions did not finish in expected iterations in ") + scenario);
	}
	if (dispacher.tgRet(chanel) < 0) {
		throw std::runtime_error(std::string("PlannerDispacher tgRet should not be negative in ") + scenario);
	}
	auto ik_ret = dispacher.ikRet(chanel);
	if (ik_ret < 0) {
		std::cout << "NOTE: PlannerDispacher ikRet < 0 in " << scenario << ", ikRet=" << ik_ret << std::endl;
	}
	if (ik_ret >= 0) {
		auto input_pos_size = model.subInputPosSize(sub_num, sub_id);
		std::vector<double> model_input(input_pos_size, 0.0);
		model.getSubInputPos(sub_num, sub_id, model_input.data());
		for (aris::Size i = 0; i < input_pos_size; ++i) {
			if (std::abs(input[i] - model_input[i]) > 1e-6) {
				throw std::runtime_error(
					std::string("PlannerDispacher final input does not match model input in ") + scenario
					+ ", joint=" + std::to_string(i)
					+ ", final=" + std::to_string(input[i])
					+ ", model=" + std::to_string(model_input[i]));
			}
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

auto create_two_puma_model() -> aris::dynamic::MultiModel {
	aris::dynamic::PumaParam puma_param;
	puma_param.d1 = 0.3;
	puma_param.a1 = 0.1;
	puma_param.a2 = 0.4;
	puma_param.a3 = 0.05;
	puma_param.d3 = 0.0;
	puma_param.d4 = 0.35;
	puma_param.install_method = 0;

	auto puma0 = aris::dynamic::createModelPuma(puma_param);
	auto puma1 = aris::dynamic::createModelPuma(puma_param);
	puma0->setName("Puma0");
	puma1->setName("Puma1");

	aris::dynamic::MultiModel multi_model;
	multi_model.subModels().add(puma0.release());
	multi_model.subModels().add(puma1.release());
	multi_model.init();

	return multi_model;
}

auto test_planner_dispacher_locking() -> void {
	auto multi_model = create_single_puma_model();

	aris::plan::PlannerDispacher dispacher;
	dispacher.setDt(0.002);
	dispacher.setModel(multi_model);
	dispacher.setChanelSize(2);
	dispacher.init();

	if (std::abs(dispacher.dt() - 0.002) > 1e-12) {
		throw std::runtime_error("PlannerDispacher dt mismatch");
	}
	if (dispacher.chanelSize() != 2) {
		throw std::runtime_error("PlannerDispacher channel size mismatch");
	}

	auto lock_ret = dispacher.tryLockChanel(0, { 0 });
	if (lock_ret != 1) {
		throw std::runtime_error("PlannerDispacher first lock should return 1");
	}

	lock_ret = dispacher.tryLockChanel(1, { 0 });
	if (lock_ret != -3) {
		throw std::runtime_error("PlannerDispacher should reject conflicting lock");
	}

	lock_ret = dispacher.tryLockChanel(0, { 0 });
	if (lock_ret != 2) {
		throw std::runtime_error("PlannerDispacher relock count mismatch");
	}

	dispacher.setTargetSpeedRatio(0, 0.5);
	if (std::abs(dispacher.targetSpeedRatio(0) - 0.5) > 1e-12) {
		throw std::runtime_error("PlannerDispacher target speed ratio mismatch");
	}

	auto release_ret = dispacher.releaseChanel(0);
	if (release_ret != 0) {
		throw std::runtime_error("PlannerDispacher first release should succeed");
	}

	release_ret = dispacher.releaseChanel(0);
	if (release_ret != 0) {
		throw std::runtime_error("PlannerDispacher second release should succeed");
	}

	release_ret = dispacher.releaseChanel(0);
	if (release_ret != -2) {
		throw std::runtime_error("PlannerDispacher extra release should fail");
	}

	lock_ret = dispacher.tryLockChanel(2, { 0 });
	if (lock_ret != -1) {
		throw std::runtime_error("PlannerDispacher invalid channel should return -1");
	}
}

auto test_planner_dispacher_switch_submodel() -> void {
	auto multi_model = create_two_puma_model();
	aris::plan::PlannerDispacher dispacher;
	dispacher.setModel(multi_model);
	dispacher.setChanelSize(2);
	dispacher.init();

	auto lock_ret = dispacher.tryLockChanel(0, { 0 });
	if (lock_ret != 1) {
		throw std::runtime_error("PlannerDispacher first lock in two-model case should return 1");
	}

	lock_ret = dispacher.tryLockChanel(0, { 1 });
	if (lock_ret != -2) {
		throw std::runtime_error("PlannerDispacher relock with different submodel should return -2");
	}

	lock_ret = dispacher.tryLockChanel(1, { 0 });
	if (lock_ret != -3) {
		throw std::runtime_error("PlannerDispacher should reject locking a submodel already held by another channel");
	}

	auto release_ret = dispacher.releaseChanel(0);
	if (release_ret != 0) {
		throw std::runtime_error("PlannerDispacher two-model case first release should succeed");
	}

	lock_ret = dispacher.tryLockChanel(1, { 0 });
	if (lock_ret != 1) {
		throw std::runtime_error("PlannerDispacher should allow another channel to lock submodel after release");
	}

	release_ret = dispacher.releaseChanel(0);
	if (release_ret != -2) {
		throw std::runtime_error("PlannerDispacher two-model case extra release should fail");
	}

	release_ret = dispacher.releaseChanel(1);
	if (release_ret != 0) {
		throw std::runtime_error("PlannerDispacher channel 1 release should succeed");
	}
}

auto test_planner_dispacher_insert_line() -> void {
	auto multi_model = create_single_puma_model();
	auto sub_num = aris::Size(1);
	auto sub_id = aris::Size(0);
	auto output_pos_size = multi_model.subOutputPosSize(sub_num, &sub_id);
	auto output_vel_size = multi_model.subOutputPosMagSize(sub_num, &sub_id);

	aris::plan::PlannerDispacher dispacher;
	dispacher.setModel(multi_model);
	dispacher.setChanelSize(1);
	dispacher.setDt(0.001);
	dispacher.init();

	auto lock_ret = dispacher.tryLockChanel(0, { 0 });
	if (lock_ret != 1) {
		throw std::runtime_error("PlannerDispacher line-flow lock should return 1");
	}

	std::vector<double> tw_pos(output_pos_size, 0.0);
	std::vector<double> vel(output_vel_size, 5.0);
	std::vector<double> acc(output_vel_size, 20.0);
	std::vector<double> jerk(output_vel_size, 100.0);
	std::vector<double> zone(output_vel_size, 0.001);

	multi_model.getSubOutputPos(sub_num, &sub_id, tw_pos.data());
	tw_pos.at(0) += 0.02;

	auto node_id = dispacher.insertLinePos(0, "", "", tw_pos.data(), vel.data(), acc.data(), jerk.data(), zone.data());
	if (node_id <= 0) {
		throw std::runtime_error("PlannerDispacher insertLinePos should return positive node id");
	}

	dispacher.updateInsertPos(0);
	expect_motion_finished(dispacher, 0, multi_model.inputPosSize(), node_id, sub_num, &sub_id, "line-flow test");

	auto release_ret = dispacher.releaseChanel(0);
	if (release_ret != 0) {
		throw std::runtime_error("PlannerDispacher line-flow release should succeed");
	}
}

auto test_planner_dispacher_insert_circle() -> void {
	auto multi_model = create_single_puma_model();
	auto sub_num = aris::Size(1);
	auto sub_id = aris::Size(0);
	auto output_pos_size = multi_model.subOutputPosSize(sub_num, &sub_id);
	auto output_vel_size = multi_model.subOutputPosMagSize(sub_num, &sub_id);

	aris::plan::PlannerDispacher dispacher;
	dispacher.setModel(multi_model);
	dispacher.setChanelSize(1);
	dispacher.setDt(0.001);
	dispacher.init();

	auto lock_ret = dispacher.tryLockChanel(0, { 0 });
	if (lock_ret != 1) {
		throw std::runtime_error("PlannerDispacher circle-flow lock should return 1");
	}

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
	if (node_id <= 0) {
		throw std::runtime_error("PlannerDispacher insertCirclePos should return positive node id");
	}

	dispacher.updateInsertPos(0);
	expect_motion_finished(dispacher, 0, multi_model.inputPosSize(), node_id, sub_num, &sub_id, "circle-flow test");

	auto release_ret = dispacher.releaseChanel(0);
	if (release_ret != 0) {
		throw std::runtime_error("PlannerDispacher circle-flow release should succeed");
	}
}

auto test_planner_dispacher_insert_movej() -> void {
	auto multi_model = create_single_puma_model();
	auto sub_num = aris::Size(1);
	auto sub_id = aris::Size(0);
	auto output_pos_size = multi_model.subOutputPosSize(sub_num, &sub_id);
	auto input_pos_size = multi_model.subInputPosSize(sub_num, &sub_id);

	aris::plan::PlannerDispacher dispacher;
	dispacher.setModel(multi_model);
	dispacher.setChanelSize(1);
	dispacher.setDt(0.001);
	dispacher.init();

	auto lock_ret = dispacher.tryLockChanel(0, { 0 });
	if (lock_ret != 1) {
		throw std::runtime_error("PlannerDispacher movej-flow lock should return 1");
	}

	std::vector<double> tw_pos(output_pos_size, 0.0);
	std::vector<double> joint_vel(input_pos_size, 5.0);
	std::vector<double> joint_acc(input_pos_size, 20.0);
	std::vector<double> joint_jerk(input_pos_size, 100.0);
	std::vector<double> joint_zone(input_pos_size, 0.001);

	multi_model.getSubOutputPos(sub_num, &sub_id, tw_pos.data());
	tw_pos.at(2) += 0.02;

	auto node_id = dispacher.insertMoveJPos(0, "", "", tw_pos.data(), joint_vel.data(), joint_acc.data(), joint_jerk.data(), joint_zone.data(), nullptr);
	if (node_id <= 0) {
		throw std::runtime_error("PlannerDispacher insertMoveJPos should return positive node id");
	}

	dispacher.updateInsertPos(0);
	expect_motion_finished(dispacher, 0, multi_model.inputPosSize(), node_id, sub_num, &sub_id, "movej-flow test");

	auto release_ret = dispacher.releaseChanel(0);
	if (release_ret != 0) {
		throw std::runtime_error("PlannerDispacher movej-flow release should succeed");
	}
}

auto test_planner_dispacher_insert_moveabsj() -> void {
	auto multi_model = create_single_puma_model();
	auto sub_num = aris::Size(1);
	auto sub_id = aris::Size(0);
	auto input_pos_size = multi_model.subInputPosSize(sub_num, &sub_id);

	std::vector<double> init_joint_pos(multi_model.inputPosSize(), 0.0);
	init_joint_pos.at(0) = 0.1;
	init_joint_pos.at(1) = -0.05;
	multi_model.setSubInputPos(sub_num, &sub_id, init_joint_pos.data());
	multi_model.subForwardKinematics(sub_num, &sub_id);

	aris::plan::PlannerDispacher dispacher;
	dispacher.setModel(multi_model);
	dispacher.setChanelSize(1);
	dispacher.setDt(0.001);
	dispacher.init();

	auto lock_ret = dispacher.tryLockChanel(0, { 0 });
	if (lock_ret != 1) {
		throw std::runtime_error("PlannerDispacher moveabsj-flow lock should return 1");
	}

	std::vector<double> joint_pos(input_pos_size, 0.0);
	std::vector<double> joint_vel(input_pos_size, 5.0);
	std::vector<double> joint_acc(input_pos_size, 20.0);
	std::vector<double> joint_jerk(input_pos_size, 100.0);
	std::vector<double> joint_zone(input_pos_size, 0.001);
	multi_model.getSubInputPos(sub_num, &sub_id, joint_pos.data());
	joint_pos.at(0) += 0.03;
	joint_pos.at(1) -= 0.02;

	auto node_id = dispacher.insertMoveAbsJPos(0, joint_pos.data(), joint_vel.data(), joint_acc.data(), joint_jerk.data(), joint_zone.data());
	if (node_id <= 0) {
		throw std::runtime_error("PlannerDispacher insertMoveAbsJPos should return positive node id");
	}

	dispacher.updateInsertPos(0);
	auto final_input = expect_motion_finished(dispacher, 0, multi_model.inputPosSize(), node_id, sub_num, &sub_id, "moveabsj-flow test");
	if (std::abs(final_input.at(0) - joint_pos.at(0)) > 1e-4 || std::abs(final_input.at(1) - joint_pos.at(1)) > 1e-4) {
		throw std::runtime_error("PlannerDispacher moveabsj-flow final input mismatch");
	}

	auto release_ret = dispacher.releaseChanel(0);
	if (release_ret != 0) {
		throw std::runtime_error("PlannerDispacher moveabsj-flow release should succeed");
	}
}

auto test_planner_dispacher_mixed_sequence() -> void {
	auto multi_model = create_single_puma_model();
	auto sub_num = aris::Size(1);
	auto sub_id = aris::Size(0);
	auto output_pos_size = multi_model.subOutputPosSize(sub_num, &sub_id);
	auto output_vel_size = multi_model.subOutputPosMagSize(sub_num, &sub_id);
	auto input_pos_size = multi_model.subInputPosSize(sub_num, &sub_id);

	aris::plan::PlannerDispacher dispacher;
	dispacher.setModel(multi_model);
	dispacher.setChanelSize(1);
	dispacher.setDt(0.001);
	dispacher.init();

	auto lock_ret = dispacher.tryLockChanel(0, { 0 });
	if (lock_ret != 1) {
		throw std::runtime_error("PlannerDispacher mixed-flow lock should return 1");
	}

	std::vector<double> cart_vel(output_vel_size, 5.0);
	std::vector<double> cart_acc(output_vel_size, 20.0);
	std::vector<double> cart_jerk(output_vel_size, 100.0);
	std::vector<double> cart_zone(output_vel_size, 0.001);
	std::vector<double> joint_vel(input_pos_size, 5.0);
	std::vector<double> joint_acc(input_pos_size, 20.0);
	std::vector<double> joint_jerk(input_pos_size, 100.0);
	std::vector<double> joint_zone(input_pos_size, 0.001);

	std::vector<double> line_pos(output_pos_size, 0.0);
	multi_model.getSubOutputPos(sub_num, &sub_id, line_pos.data());
	line_pos.at(0) += 0.02;

	auto line_id = dispacher.insertLinePos(0, "", "", line_pos.data(), cart_vel.data(), cart_acc.data(), cart_jerk.data(), cart_zone.data());
	if (line_id <= 0) {
		throw std::runtime_error("PlannerDispacher mixed-flow line insert should return positive node id");
	}
	dispacher.updateInsertPos(0);
	expect_motion_finished(dispacher, 0, multi_model.inputPosSize(), line_id, sub_num, &sub_id, "mixed-flow line test");

	std::vector<double> circle_mid_pos = line_pos;
	std::vector<double> circle_target_pos = line_pos;
	circle_mid_pos.at(1) += 0.01;
	circle_target_pos.at(1) += 0.02;

	auto circle_id = dispacher.insertCirclePos(0, "", "", circle_target_pos.data(), circle_mid_pos.data(), cart_vel.data(), cart_acc.data(), cart_jerk.data(), cart_zone.data());
	if (circle_id <= line_id) {
		throw std::runtime_error("PlannerDispacher mixed-flow circle insert should return increasing node id");
	}
	dispacher.updateInsertPos(0);
	expect_motion_finished(dispacher, 0, multi_model.inputPosSize(), circle_id, sub_num, &sub_id, "mixed-flow circle test");

	std::vector<double> movej_pos = circle_target_pos;
	movej_pos.at(0) += 0.03;
	movej_pos.at(1) -= 0.02;
	movej_pos.at(2) += 0.05;

	auto movej_id = dispacher.insertMoveJPos(0, "", "", movej_pos.data(), joint_vel.data(), joint_acc.data(), joint_jerk.data(), joint_zone.data(), nullptr);
	if (movej_id <= circle_id) {
		throw std::runtime_error("PlannerDispacher mixed-flow movej insert should return increasing node id");
	}
	dispacher.updateInsertPos(0);
	expect_motion_finished(dispacher, 0, multi_model.inputPosSize(), movej_id, sub_num, &sub_id, "mixed-flow movej test");

	std::vector<double> moveabsj_pos(input_pos_size, 0.0);
	multi_model.getSubInputPos(sub_num, &sub_id, moveabsj_pos.data());
	moveabsj_pos.at(0) += 0.03;
	moveabsj_pos.at(1) -= 0.02;

	auto moveabsj_id = dispacher.insertMoveAbsJPos(0, moveabsj_pos.data(), joint_vel.data(), joint_acc.data(), joint_jerk.data(), joint_zone.data());
	if (moveabsj_id <= movej_id) {
		throw std::runtime_error("PlannerDispacher mixed-flow moveabsj insert should return increasing node id");
	}
	dispacher.updateInsertPos(0);
	auto final_input = expect_motion_finished(dispacher, 0, multi_model.inputPosSize(), moveabsj_id, sub_num, &sub_id, "mixed-flow moveabsj test");
	if (std::abs(final_input.at(0) - moveabsj_pos.at(0)) > 1e-4 || std::abs(final_input.at(1) - moveabsj_pos.at(1)) > 1e-4) {
		throw std::runtime_error("PlannerDispacher mixed-flow moveabsj final input mismatch");
	}

	auto release_ret = dispacher.releaseChanel(0);
	if (release_ret != 0) {
		throw std::runtime_error("PlannerDispacher mixed-flow release should succeed");
	}
}

auto test_planner_dispacher_batch_sequence() -> void {
	auto multi_model = create_single_puma_model();
	auto sub_num = aris::Size(1);
	auto sub_id = aris::Size(0);
	auto output_pos_size = multi_model.subOutputPosSize(sub_num, &sub_id);
	auto output_vel_size = multi_model.subOutputPosMagSize(sub_num, &sub_id);
	auto input_pos_size = multi_model.subInputPosSize(sub_num, &sub_id);

	std::vector<double> init_joint_pos(multi_model.inputPosSize(), 0.0);
	init_joint_pos.at(0) = 0.1;
	init_joint_pos.at(1) = -0.05;
	multi_model.setSubInputPos(sub_num, &sub_id, init_joint_pos.data());
	multi_model.subForwardKinematics(sub_num, &sub_id);

	aris::plan::PlannerDispacher dispacher;
	dispacher.setModel(multi_model);
	dispacher.setChanelSize(1);
	dispacher.setDt(0.001);
	dispacher.init();

	auto lock_ret = dispacher.tryLockChanel(0, { 0 });
	if (lock_ret != 1) {
		throw std::runtime_error("PlannerDispacher batch-flow lock should return 1");
	}

	std::vector<double> cart_vel(output_vel_size, 5.0);
	std::vector<double> cart_acc(output_vel_size, 20.0);
	std::vector<double> cart_jerk(output_vel_size, 100.0);
	std::vector<double> cart_zone(output_vel_size, 0.001);
	std::vector<double> joint_vel(input_pos_size, 5.0);
	std::vector<double> joint_acc(input_pos_size, 20.0);
	std::vector<double> joint_jerk(input_pos_size, 100.0);
	std::vector<double> joint_zone(input_pos_size, 0.001);

	std::vector<double> line_pos(output_pos_size, 0.0);
	multi_model.getSubOutputPos(sub_num, &sub_id, line_pos.data());
	line_pos.at(0) += 0.02;

	auto line_id = dispacher.insertLinePos(0, "", "", line_pos.data(), cart_vel.data(), cart_acc.data(), cart_jerk.data(), cart_zone.data());
	if (line_id <= 0) {
		throw std::runtime_error("PlannerDispacher batch-flow line insert should return positive node id");
	}

	std::vector<double> circle_mid_pos = line_pos;
	std::vector<double> circle_target_pos = line_pos;
	circle_mid_pos.at(1) += 0.01;
	circle_target_pos.at(1) += 0.02;

	auto circle_id = dispacher.insertCirclePos(0, "", "", circle_target_pos.data(), circle_mid_pos.data(), cart_vel.data(), cart_acc.data(), cart_jerk.data(), cart_zone.data());
	if (circle_id <= line_id) {
		throw std::runtime_error("PlannerDispacher batch-flow circle insert should return increasing node id");
	}

	std::vector<double> movej_pos = circle_target_pos;
	movej_pos.at(2) += 0.02;

	auto movej_id = dispacher.insertMoveJPos(0, "", "", movej_pos.data(), joint_vel.data(), joint_acc.data(), joint_jerk.data(), joint_zone.data(), nullptr);
	if (movej_id <= circle_id) {
		throw std::runtime_error("PlannerDispacher batch-flow movej insert should return increasing node id");
	}

	dispacher.updateInsertPos(0);
	auto final_input = expect_motion_sequence_finished(
		dispacher,
		0,
		multi_model.inputPosSize(),
		{ line_id, circle_id, movej_id },
		sub_num,
		&sub_id,
		"batch-flow test");
	if (!std::isfinite(final_input.at(0)) || !std::isfinite(final_input.at(1))) {
		throw std::runtime_error("PlannerDispacher batch-flow final input should stay finite");
	}

	auto release_ret = dispacher.releaseChanel(0);
	if (release_ret != 0) {
		throw std::runtime_error("PlannerDispacher batch-flow release should succeed");
	}
}

auto test_planner_dispacher_many_points() -> void {
	auto multi_model = create_single_puma_model();
	auto sub_num = aris::Size(1);
	auto sub_id = aris::Size(0);
	auto output_pos_size = multi_model.subOutputPosSize(sub_num, &sub_id);
	auto output_vel_size = multi_model.subOutputPosMagSize(sub_num, &sub_id);
	auto input_pos_size = multi_model.subInputPosSize(sub_num, &sub_id);

	aris::plan::PlannerDispacher dispacher;
	dispacher.setModel(multi_model);
	dispacher.setChanelSize(1);
	dispacher.setDt(0.001);
	dispacher.init();

	auto lock_ret = dispacher.tryLockChanel(0, { 0 });
	if (lock_ret != 1) {
		throw std::runtime_error("PlannerDispacher many-points lock should return 1");
	}

	std::vector<double> cart_vel(output_vel_size, 5.0);
	std::vector<double> cart_acc(output_vel_size, 20.0);
	std::vector<double> cart_jerk(output_vel_size, 100.0);
	std::vector<double> cart_zone(output_vel_size, 0.001);
	std::vector<double> joint_vel(input_pos_size, 5.0);
	std::vector<double> joint_acc(input_pos_size, 20.0);
	std::vector<double> joint_jerk(input_pos_size, 100.0);
	std::vector<double> joint_zone(input_pos_size, 0.001);

	std::vector<double> line_pos(output_pos_size, 0.0);
	std::vector<double> joint_pos(input_pos_size, 0.0);
	multi_model.getSubOutputPos(sub_num, &sub_id, line_pos.data());
	multi_model.getSubInputPos(sub_num, &sub_id, joint_pos.data());

	std::vector<std::int64_t> node_ids;
	node_ids.reserve(200);

	for (int i = 0; i < 40; ++i) {
		line_pos.at(0) += 0.0005;
		line_pos.at(1) += (i % 2 == 0) ? 0.0002 : -0.0002;

		auto line_id = dispacher.insertLinePos(0, "", "", line_pos.data(), cart_vel.data(), cart_acc.data(), cart_jerk.data(), cart_zone.data());
		if (line_id <= 0) {
			throw std::runtime_error("PlannerDispacher many-points line insert should return positive node id");
		}
		node_ids.push_back(line_id);

		std::vector<double> circle_mid_pos = line_pos;
		std::vector<double> circle_target_pos = line_pos;
		circle_mid_pos.at(1) += (i % 2 == 0) ? 0.0003 : -0.0003;
		circle_target_pos.at(0) += 0.0003;
		circle_target_pos.at(1) += (i % 3 == 0) ? 0.0004 : -0.0004;

		auto circle_id = dispacher.insertCirclePos(0, "", "", circle_target_pos.data(), circle_mid_pos.data(), cart_vel.data(), cart_acc.data(), cart_jerk.data(), cart_zone.data());
		if (circle_id <= line_id) {
			throw std::runtime_error("PlannerDispacher many-points circle insert should return increasing node id");
		}
		node_ids.push_back(circle_id);

		std::vector<double> movej_pos = circle_target_pos;
		movej_pos.at(2) += (i % 2 == 0) ? 0.001 : -0.001;
		auto movej_id = dispacher.insertMoveJPos(0, "", "", movej_pos.data(), joint_vel.data(), joint_acc.data(), joint_jerk.data(), joint_zone.data(), nullptr);
		if (movej_id <= circle_id) {
			throw std::runtime_error("PlannerDispacher many-points movej insert should return increasing node id");
		}
		node_ids.push_back(movej_id);

		joint_pos.at(0) += (i % 2 == 0) ? 0.0008 : -0.0008;
		joint_pos.at(1) += (i % 3 == 0) ? 0.0006 : -0.0006;
		auto moveabsj_id = dispacher.insertMoveAbsJPos(0, joint_pos.data(), joint_vel.data(), joint_acc.data(), joint_jerk.data(), joint_zone.data());
		if (moveabsj_id <= movej_id) {
			throw std::runtime_error("PlannerDispacher many-points moveabsj insert should return increasing node id");
		}
		node_ids.push_back(moveabsj_id);
	}

	dispacher.updateInsertPos(0);
	expect_motion_sequence_finished(
		dispacher,
		0,
		multi_model.inputPosSize(),
		node_ids,
		sub_num,
		&sub_id,
		"many-points stress test");

	auto release_ret = dispacher.releaseChanel(0);
	if (release_ret != 0) {
		throw std::runtime_error("PlannerDispacher many-points release should succeed");
	}
}

auto test_planner_dispacher_intentional_failures() -> void {
	auto run_expect_fail = [](
		const LimitCheckConfig& cfg,
		const char* expected_keyword,
		const char* scenario) {
		auto multi_model = create_single_puma_model();
		auto sub_num = aris::Size(1);
		auto sub_id = aris::Size(0);
		auto output_pos_size = multi_model.subOutputPosSize(sub_num, &sub_id);
		auto output_vel_size = multi_model.subOutputPosMagSize(sub_num, &sub_id);

		aris::plan::PlannerDispacher dispacher;
		dispacher.setModel(multi_model);
		dispacher.setChanelSize(1);
		dispacher.setDt(0.001);
		dispacher.init();

		auto lock_ret = dispacher.tryLockChanel(0, { 0 });
		if (lock_ret != 1) {
			throw std::runtime_error("PlannerDispacher intentional-failure lock should return 1");
		}

		std::vector<double> tw_pos(output_pos_size, 0.0);
		std::vector<double> vel(output_vel_size, 5.0);
		std::vector<double> acc(output_vel_size, 20.0);
		std::vector<double> jerk(output_vel_size, 100.0);
		std::vector<double> zone(output_vel_size, 0.001);

		multi_model.getSubOutputPos(sub_num, &sub_id, tw_pos.data());
		tw_pos.at(0) += 0.02;

		auto node_id = dispacher.insertLinePos(0, "", "", tw_pos.data(), vel.data(), acc.data(), jerk.data(), zone.data());
		if (node_id <= 0) {
			throw std::runtime_error("PlannerDispacher intentional-failure insertLinePos should return positive node id");
		}

		dispacher.updateInsertPos(0);

		bool saw_expected_fail = false;
		try {
			expect_motion_finished(dispacher, 0, multi_model.inputPosSize(), node_id, sub_num, &sub_id, scenario, cfg);
		}
		catch (const std::runtime_error& e) {
			std::string msg(e.what());
			if (msg.find(expected_keyword) != std::string::npos) {
				saw_expected_fail = true;
			}
			else {
				throw;
			}
		}

		auto release_ret = dispacher.releaseChanel(0);
		if (release_ret != 0) {
			throw std::runtime_error("PlannerDispacher intentional-failure release should succeed");
		}

		if (!saw_expected_fail) {
			throw std::runtime_error(std::string("PlannerDispacher expected failure was not triggered in ") + scenario);
		}
	};

	run_expect_fail(
		LimitCheckConfig{
			-std::numeric_limits<double>::infinity(),
			0.1,
			-std::numeric_limits<double>::infinity(),
			std::numeric_limits<double>::infinity()
		},
		"velocity limit exceeded",
		"intentional velocity fail test");

	run_expect_fail(
		LimitCheckConfig{
			-std::numeric_limits<double>::infinity(),
			std::numeric_limits<double>::infinity(),
			-0.05,
			std::numeric_limits<double>::infinity()
		},
		"acceleration limit exceeded",
		"intentional acceleration fail test");
}
}

auto test_planner_dispacher() -> void {
	std::cout << "test planner dispacher" << std::endl;
	test_planner_dispacher_locking();
	test_planner_dispacher_switch_submodel();
	test_planner_dispacher_insert_line();
	test_planner_dispacher_insert_circle();
	test_planner_dispacher_insert_movej();
	test_planner_dispacher_insert_moveabsj();
	test_planner_dispacher_mixed_sequence();
	test_planner_dispacher_batch_sequence();
	test_planner_dispacher_many_points();
	test_planner_dispacher_intentional_failures();

	std::cout << "test planner dispacher finished" << std::endl;
}