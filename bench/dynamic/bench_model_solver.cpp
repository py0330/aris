#include <array>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <aris/core/msg.hpp>
#include <aris/core/serialization.hpp>
#include <aris/dynamic/dynamic.hpp>

using namespace aris::dynamic;

namespace {

#include "model_solver_extra_xml.inc"

template<typename Function>
auto bench_print(const std::string &label, std::size_t count, Function &&function) -> void {
	std::cout << label << aris::core::benchmark(count, std::forward<Function>(function)) << std::endl;
}

auto build_3r_model() -> std::unique_ptr<Model> {
	const double link1_position_and_euler321[6]{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	const double link2_position_and_euler321[6]{1.0, 0.0, 0.0, aris::PI / 2, 0.0, 0.0};
	const double link3_position_and_euler321[6]{1.0, 1.0, 0.0, aris::PI, 0.0, 0.0};
	const double link_inertia[10]{2.0, 0.0, 0.0, 0.0, 1.0, 1.0, 10.0, 0.0, 0.0, 0.0};
	const double joint1_position[3]{0.0, 0.0, 0.0};
	const double joint2_position[3]{1.0, 0.0, 0.0};
	const double joint3_position[3]{1.0, 1.0, 0.0};
	const double joint_axis[3]{0.0, 0.0, 1.0};
	const double ee_pe[6]{0.0, 1.0, 0.0, aris::PI, 0.0, 0.0};

	auto model = std::make_unique<Model>();
	auto &link1 = model->addPartByPe(link1_position_and_euler321, "321", link_inertia);
	auto &link2 = model->addPartByPe(link2_position_and_euler321, "321", link_inertia);
	auto &link3 = model->addPartByPe(link3_position_and_euler321, "321", link_inertia);
	auto &joint1 = model->addRevoluteJoint(link1, model->ground(), joint1_position, joint_axis);
	auto &joint2 = model->addRevoluteJoint(link2, link1, joint2_position, joint_axis);
	auto &joint3 = model->addRevoluteJoint(link3, link2, joint3_position, joint_axis);
	auto &motion1 = model->addMotion(joint1);
	auto &motion2 = model->addMotion(joint2);
	auto &motion3 = model->addMotion(joint3);
	auto &ee = model->addGeneralMotionByPe(link3, model->ground(), ee_pe, "321");
	ee.setPosType(PosType::PM);
	model->forcePool().add<SingleComponentForce>("f1", motion1.makI(), motion1.makJ(), 5);
	model->forcePool().add<SingleComponentForce>("f2", motion2.makI(), motion2.makJ(), 5);
	model->forcePool().add<SingleComponentForce>("f3", motion3.makI(), motion3.makJ(), 5);
	auto &solver = model->solverPool().add<UniversalSolver>();
	solver.setMaxError(1e-15);
	return model;
}

auto build_ur5_model_from_test_xml() -> std::unique_ptr<Model> {
	auto model = std::make_unique<Model>();
	aris::core::fromXmlString(*model, xml_file_ur5);
	return model;
}

auto build_stewart_model_from_test_xml() -> std::unique_ptr<Model> {
	auto model = std::make_unique<Model>();
	aris::core::fromXmlString(*model, xml_file_stewart);
	return model;
}

auto build_ur5_on_stewart_model_from_test_xml() -> std::unique_ptr<Model> {
	auto model = std::make_unique<Model>();
	aris::core::fromXmlString(*model, xml_file_ur5_on_stewart);
	return model;
}

auto build_multi_systems_model_from_test_xml() -> std::unique_ptr<Model> {
	auto model = std::make_unique<Model>();
	aris::core::fromXmlString(*model, xml_file_multi);
	return model;
}

struct CaseDef {
	std::string name;
	std::function<std::unique_ptr<Model>()> build_model;
	std::vector<double> origin_pos;
	std::vector<double> origin_vel;
	std::vector<double> origin_acc;
	std::vector<double> input_pos;
	std::vector<double> input_vel;
	std::vector<double> input_acc;
	std::array<double, 6> tol;
	std::size_t bench_count;
	aris::Size fwd_solver_idx{0};
	aris::Size inv_solver_idx{0};
};

auto bench_solver(Model &model, const CaseDef &cfg) -> void {
	auto &fwd_solver = model.solverPool().at(cfg.fwd_solver_idx);
	auto &inv_solver = model.solverPool().at(cfg.inv_solver_idx);
	const auto prefix = cfg.name;

	std::vector<double> out_pos_target;
	std::vector<double> out_pos_origin;
	std::vector<double> out_vel_target;
	std::vector<double> out_acc_target;
	std::vector<double> in_pos_target;
	std::vector<double> in_pos_origin;
	std::vector<double> in_vel_target;
	std::vector<double> in_acc_target;
	std::vector<double> scratch;

	for (auto &motion : model.motionPool()) motion.activate(true);
	for (auto &force : model.forcePool()) force.activate(false);
	for (auto &gm : model.generalMotionPool()) gm.activate(false);
	model.init();

	out_pos_target.assign(static_cast<std::size_t>(model.outputPosSize()), 0.0);
	out_pos_origin.assign(static_cast<std::size_t>(model.outputPosSize()), 0.0);
	out_vel_target.assign(static_cast<std::size_t>(model.outputVelSize()), 0.0);
	out_acc_target.assign(static_cast<std::size_t>(model.outputAccSize()), 0.0);
	in_pos_target.assign(static_cast<std::size_t>(model.inputPosSize()), 0.0);
	in_pos_origin.assign(static_cast<std::size_t>(model.inputPosSize()), 0.0);
	in_vel_target.assign(static_cast<std::size_t>(model.inputVelSize()), 0.0);
	in_acc_target.assign(static_cast<std::size_t>(model.inputAccSize()), 0.0);
	const auto scratch_len = static_cast<std::size_t>(std::max({
		model.inputPosSize(), model.outputPosSize(), model.outputVelSize(), model.outputAccSize(), model.inputVelSize(), model.inputAccSize()
	}) + 16);
	scratch.assign(scratch_len, 0.0);

	model.setInputPos(cfg.input_pos.data());
	model.setInputVel(cfg.input_vel.data());
	model.setInputAcc(cfg.input_acc.data());
	fwd_solver.kinPos();
	fwd_solver.kinVel();
	fwd_solver.dynAccAndFce();
	model.getOutputPos(out_pos_target.data());
	model.getOutputVel(out_vel_target.data());
	model.getOutputAcc(out_acc_target.data());

	model.setInputPos(cfg.origin_pos.data());
	model.setInputVel(cfg.origin_vel.data());
	model.setInputAcc(cfg.origin_acc.data());
	fwd_solver.kinPos();
	model.getOutputPos(out_pos_origin.data());

	std::size_t toggle{0};
	bench_print(prefix + "::forward computational pos time:", cfg.bench_count, [&]() {
		if (toggle % 2 == 0) model.setInputPos(cfg.input_pos.data());
		else model.setInputPos(cfg.origin_pos.data());
		fwd_solver.kinPos();
		model.getOutputPos(scratch.data());
		const auto *expected = (toggle % 2 == 0) ? out_pos_target.data() : out_pos_origin.data();
		if (!s_is_equal(model.outputPosSize(), scratch.data(), expected, cfg.tol[0])) {
			throw std::runtime_error(prefix + "::forward pos benchmark failed");
		}
		++toggle;
	});

	bench_print(prefix + "::forward computational vel time:", cfg.bench_count, [&]() {
		model.setInputPos(cfg.input_pos.data());
		model.setInputVel(cfg.input_vel.data());
		model.setInputAcc(cfg.input_acc.data());
		fwd_solver.kinPos();
		fwd_solver.kinVel();
		model.getOutputVel(scratch.data());
		if (!s_is_equal(model.outputVelSize(), scratch.data(), out_vel_target.data(), cfg.tol[1])) {
			throw std::runtime_error(prefix + "::forward vel benchmark failed");
		}
	});

	bench_print(prefix + "::forward computational acc time:", cfg.bench_count, [&]() {
		model.setInputPos(cfg.input_pos.data());
		model.setInputVel(cfg.input_vel.data());
		model.setInputAcc(cfg.input_acc.data());
		fwd_solver.kinPos();
		fwd_solver.kinVel();
		fwd_solver.dynAccAndFce();
		model.getOutputAcc(scratch.data());
		if (!s_is_equal(model.outputAccSize(), scratch.data(), out_acc_target.data(), cfg.tol[2])) {
			throw std::runtime_error(prefix + "::forward acc benchmark failed");
		}
	});

	bench_print(prefix + "::forward computational dyn mat time:", cfg.bench_count, [&]() {
		model.setInputPos(cfg.input_pos.data());
		model.setInputVel(cfg.input_vel.data());
		model.setInputAcc(cfg.input_acc.data());
		dynamic_cast<UniversalSolver &>(fwd_solver).cptGeneralInverseDynamicMatrix();
	});

	for (auto &motion : model.motionPool()) motion.activate(false);
	for (auto &force : model.forcePool()) force.activate(false);
	for (auto &gm : model.generalMotionPool()) gm.activate(true);
	model.init();
	model.setOutputPos(out_pos_target.data());
	inv_solver.kinPos();
	for (auto &motion : model.motionPool()) motion.updP();
	model.getInputPos(in_pos_target.data());

	model.setOutputPos(out_pos_origin.data());
	inv_solver.kinPos();
	for (auto &motion : model.motionPool()) motion.updP();
	model.getInputPos(in_pos_origin.data());

	model.setOutputPos(out_pos_target.data());
	inv_solver.kinPos();
	model.setOutputVel(out_vel_target.data());
	inv_solver.kinVel();
	for (auto &motion : model.motionPool()) motion.updV();
	model.getInputVel(in_vel_target.data());

	model.setOutputPos(out_pos_target.data());
	model.setOutputVel(out_vel_target.data());
	inv_solver.kinPos();
	inv_solver.kinVel();
	model.setOutputAcc(out_acc_target.data());
	inv_solver.dynAccAndFce();
	for (auto &motion : model.motionPool()) motion.updA();
	model.getInputAcc(in_acc_target.data());

	toggle = 0;
	bench_print(prefix + "::inverse computational pos time:", cfg.bench_count, [&]() {
		if (toggle % 2 == 0) model.setOutputPos(out_pos_target.data());
		else model.setOutputPos(out_pos_origin.data());
		inv_solver.kinPos();
		for (auto &motion : model.motionPool()) motion.updP();
		model.getInputPos(scratch.data());
		const auto *expected = (toggle % 2 == 0) ? in_pos_target.data() : in_pos_origin.data();
		if (!s_is_equal(model.inputPosSize(), scratch.data(), expected, cfg.tol[3])) {
			throw std::runtime_error(prefix + "::inverse pos benchmark failed");
		}
		++toggle;
	});

	bench_print(prefix + "::inverse computational vel time:", cfg.bench_count, [&]() {
		model.setOutputPos(out_pos_target.data());
		inv_solver.kinPos();
		model.setOutputVel(out_vel_target.data());
		inv_solver.kinVel();
		for (auto &motion : model.motionPool()) motion.updV();
		model.getInputVel(scratch.data());
		if (!s_is_equal(model.inputVelSize(), scratch.data(), in_vel_target.data(), cfg.tol[4])) {
			throw std::runtime_error(prefix + "::inverse vel benchmark failed");
		}
	});

	bench_print(prefix + "::inverse computational acc time:", cfg.bench_count, [&]() {
		model.setOutputPos(out_pos_target.data());
		model.setOutputVel(out_vel_target.data());
		inv_solver.kinPos();
		inv_solver.kinVel();
		model.setOutputAcc(out_acc_target.data());
		inv_solver.dynAccAndFce();
		for (auto &motion : model.motionPool()) motion.updA();
		model.getInputAcc(scratch.data());
		if (!s_is_equal(model.inputAccSize(), scratch.data(), in_acc_target.data(), cfg.tol[5])) {
			throw std::runtime_error(prefix + "::inverse acc benchmark failed");
		}
	});

	bench_print(prefix + "::inverse computational dyn mat time:", cfg.bench_count, [&]() {
		model.setOutputPos(out_pos_target.data());
		model.setOutputVel(out_vel_target.data());
		model.setOutputAcc(out_acc_target.data());
		dynamic_cast<UniversalSolver &>(inv_solver).cptGeneralInverseDynamicMatrix();
	});
}

auto run_case(const CaseDef &cfg) -> void {
	std::cout << "bench " << cfg.name << ":" << std::endl;
	auto model = cfg.build_model();
	bench_solver(*model, cfg);
}

} // namespace

int main() {
	std::cout << "\n-----------------model solver bench---------------------" << std::endl;

	const std::vector<CaseDef> cases{
		CaseDef{
			"3R robot",
			build_3r_model,
			{0.0, 0.0, 0.0},
			{0.0, 0.0, 0.0},
			{0.0, 0.0, 0.0},
			{-0.0648537067263432, -0.4611742608347527, 0.5260279675610960},
			{0.2647720948695498, -0.5918279267633222, 0.6270558318937725},
			{0.8080984807847047, -0.7798913328042270, 0.1717928520195222},
			{1e-9, 1e-9, 1e-8, 1e-8, 1e-9, 1e-8},
			10000,
		},
		CaseDef{
			"UR5",
			build_ur5_model_from_test_xml,
			{0.0, 0.0, 0.2, 0.3, 0.1, 0.0},
			{0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
			{0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
			{-0.2, -0.3, 0.5, 0.4, 0.1, 0.2},
			{0.93426722257942, -0.024823760537999, -0.89419018046124, 0.245922301638701, -1.23100367003297, -0.48185561218356},
			{0.70807836306709, -0.496581922752884, -0.159513727427361, -0.590163055515337, 0.131806583011732, -1.65802060177352},
			{1e-9, 1e-9, 1e-8, 1e-8, 1e-8, 1e-8},
			10000,
			0,
			0,
		},
		CaseDef{
			"Stewart",
			build_stewart_model_from_test_xml,
			{2.0, 2.0, 2.0, 2.0, 2.0, 2.0},
			{0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
			{0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
			{2.15, 2.03, 1.98, 1.68, 2.22, 2.01},
			{0.687, 1.521, -0.325, 0.665, 1.225, -0.999},
			{1.687, 0.521, -1.325, 1.665, 0.225, -1.999},
			{1e-9, 1e-8, 1e-7, 1e-8, 1e-8, 1e-7},
			10000,
		},
		CaseDef{
			"UR5 on Stewart",
			build_ur5_on_stewart_model_from_test_xml,
			{0.0, 0.0, 0.2, 0.3, 0.1, 0.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0},
			{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
			{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
			{-0.2, -0.3, 0.5, 0.4, 0.1, 0.2, 2.15, 2.03, 1.98, 1.68, 2.22, 2.01},
			{0.93426722257942, -0.024823760537999, -0.89419018046124, 0.245922301638701, -1.23100367003297, -0.48185561218356, 0.687, 1.521, -0.325, 0.665, 1.225, -0.999},
			{0.70807836306709, -0.496581922752884, -0.159513727427361, -0.590163055515337, 0.131806583011732, -1.65802060177352, 1.687, 0.521, -1.325, 1.665, 0.225, -1.999},
			{1e-8, 1e-8, 1e-8, 1e-8, 1e-8, 1e-8},
			10000,
		},
		CaseDef{
			"Multi systems",
			build_multi_systems_model_from_test_xml,
			{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0},
			{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
			{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
			{-0.0648537067263432, -0.4611742608347527, 0.5260279675610960, -0.084321840829742, 0.111235847475406, 0.163501201249858, 0.41316722587035, -0.0861578092597486, 0.229246197281016, 2.15, 2.03, 1.98, 1.68, 2.22, 2.01},
			{0.2647720948695498, -0.5918279267633222, 0.6270558318937725, 0.93426722257942, -0.024823760537999, -0.89419018046124, 0.245922301638701, -1.23100367003297, -0.48185561218356, 0.687, 1.521, -0.325, 0.665, 1.225, -0.999},
			{0.8080984807847047, -0.7798913328042270, 0.1717928520195222, 0.70807836306709, -0.496581922752884, -0.159513727427361, -0.590163055515337, 0.131806583011732, -1.65802060177352, 1.687, 0.521, -1.325, 1.665, 0.225, -1.999},
			{1e-8, 1e-8, 1e-8, 1e-8, 1e-8, 1e-8},
			10000,
		},
	};

	for (const auto &cfg : cases) {
		try {
			run_case(cfg);
		} catch (const std::exception &e) {
			std::cout << cfg.name << " failed: " << e.what() << std::endl;
		}
	}

	std::cout << "-----------------model solver bench finished------------\n" << std::endl;
	return 0;
}
