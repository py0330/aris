#include <iostream>
#include <array>
#include <vector>
#include <cstdlib>
#include <iomanip>
#include <aris.hpp>

using namespace aris::dynamic;
using namespace aris::robot;

const double PI = 3.14159265358979;

Model rbt;

namespace {
auto create_debug_ur_model() -> std::unique_ptr<aris::dynamic::Model> {
	aris::dynamic::UrParam param;
	param.H1 = 0.089159;
	param.W1 = 0.13585 - 0.1197 + 0.093;
	param.L1 = 0.425;
	param.L2 = 0.39225;
	param.H2 = -0.09465;
	param.W2 = 0.0823;
	param.install_method = 0;
	return aris::dynamic::createModelUr(param);
}

void print_vec(const std::string &name, const std::vector<double> &v) {
	std::cout << name << " = [";
	for (aris::Size i = 0; i < v.size(); ++i) {
		if (i) std::cout << ", ";
		std::cout << std::setprecision(17) << v[i];
	}
	std::cout << "]" << std::endl;
}

void print_diff(const std::string &name, const std::vector<double> &lhs, const std::vector<double> &rhs) {
	std::cout << name << " = [";
	for (aris::Size i = 0; i < lhs.size(); ++i) {
		if (i) std::cout << ", ";
		std::cout << std::setprecision(17) << (lhs[i] - rhs[i]);
	}
	std::cout << "]" << std::endl;
}

auto run_debug_pollute_replay() -> int {
	auto m = create_debug_ur_model();
	m->init();

	auto &inv = dynamic_cast<aris::dynamic::InverseKinematicSolver&>(m->solverPool().at(0));
	auto &fwd = dynamic_cast<aris::dynamic::ForwardKinematicSolver&>(m->solverPool().at(1));

	const double cmd_q[6]{0.17, -0.09, 0.14, -0.22, 0.19, -0.11};
	const double cmd_v_ref[6]{0.12, -0.10, 0.08, -0.06, 0.04, -0.02};
	const double zero_vs[6]{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	const double tail_vs[6]{1.0, 2.0, 3.0, 4.0, 5.0, 6.0};

	m->setInputPos(cmd_q);
	if (m->forwardKinematics()) {
		std::cout << "forwardKinematics failed" << std::endl;
		return 1;
	}

	std::vector<double> out_pos(m->outputPosSize(), 0.0);
	m->getOutputPos(out_pos.data());

	m->setInputVel(cmd_v_ref);
	if (m->forwardKinematicsVel()) {
		std::cout << "forwardKinematicsVel failed when building target output vel" << std::endl;
		return 1;
	}

	std::vector<double> out_vel_target(m->outputVelSize(), 0.0);
	m->getOutputVel(out_vel_target.data());

	std::vector<double> cmd_v_ref_vec(cmd_v_ref, cmd_v_ref + 6);
	std::cout << "\n=== Pollute Replay Debug ===" << std::endl;
	print_vec("cmd_v_ref", cmd_v_ref_vec);
	print_vec("out_vel_target", out_vel_target);

	std::int64_t which_root{0};
	if (m->getWhichInverseRoot(out_pos.data(), cmd_q, &which_root)) {
		std::cout << "getWhichInverseRoot failed" << std::endl;
		return 1;
	}

	auto run_stateless_velocity_consistency_case = [&]() {
		std::vector<double> out_vel_dirty_2(m->outputVelSize(), 0.0);
		for (aris::Size i = 0; i < out_vel_dirty_2.size(); ++i) {
			out_vel_dirty_2[i] = (i % 2 == 0) ? (0.31 - 0.04 * static_cast<double>(i)) : (-0.27 + 0.03 * static_cast<double>(i));
		}

		std::vector<double> cmd_v_stateful_ref_a(m->inputVelSize(), 0.0), cmd_v_stateful_ref_b(m->inputVelSize(), 0.0);
		std::vector<double> cmd_v_stateless_ref_a(m->inputVelSize(), 0.0), cmd_v_stateless_ref_b(m->inputVelSize(), 0.0);

		// Match gtest: first polluted state = clean velocity state.
		for (auto &prt : m->partPool()) prt.setVs(zero_vs);
		m->setOutputVel(out_vel_target.data());
		if (m->inverseKinematicsVel()) {
			std::cout << "inverseKinematicsVel failed in stateful_ref_a" << std::endl;
			return;
		}
		m->getInputVel(cmd_v_stateful_ref_a.data());

		if (inv.kinVelPure(out_vel_target.data(), cmd_v_stateless_ref_a.data())) {
			std::cout << "inv.kinVelPure failed in stateless_ref_a" << std::endl;
			return;
		}

		// Dirty the model with the second polluted velocity state, then switch back to the same target.
		m->setOutputVel(out_vel_dirty_2.data());
		if (m->inverseKinematicsVel()) {
			std::cout << "inverseKinematicsVel failed while dirtying with dirty_2" << std::endl;
			return;
		}

		m->setOutputVel(out_vel_target.data());
		if (m->inverseKinematicsVel()) {
			std::cout << "inverseKinematicsVel failed in stateful_ref_b" << std::endl;
			return;
		}
		m->getInputVel(cmd_v_stateful_ref_b.data());

		if (inv.kinVelPure(out_vel_target.data(), cmd_v_stateless_ref_b.data())) {
			std::cout << "inv.kinVelPure failed in stateless_ref_b" << std::endl;
			return;
		}

		std::cout << "\n--- stateless_velocity_consistency_replay ---" << std::endl;
		print_vec("dirty_state_2_output_vel", out_vel_dirty_2);
		print_vec("stateful_ref_a", cmd_v_stateful_ref_a);
		print_vec("stateful_ref_b", cmd_v_stateful_ref_b);
		print_diff("stateful_ref_b_minus_a", cmd_v_stateful_ref_b, cmd_v_stateful_ref_a);
		print_diff("stateful_ref_a_minus_ref", cmd_v_stateful_ref_a, cmd_v_ref_vec);
		print_diff("stateful_ref_b_minus_ref", cmd_v_stateful_ref_b, cmd_v_ref_vec);

		print_vec("stateless_ref_a", cmd_v_stateless_ref_a);
		print_vec("stateless_ref_b", cmd_v_stateless_ref_b);
		print_diff("stateless_ref_b_minus_a", cmd_v_stateless_ref_b, cmd_v_stateless_ref_a);
		print_diff("stateless_ref_a_minus_ref", cmd_v_stateless_ref_a, cmd_v_ref_vec);
		print_diff("stateless_ref_b_minus_ref", cmd_v_stateless_ref_b, cmd_v_ref_vec);
	};

	auto run_case = [&](const char *label, bool pollute_tail) {
		for (auto &prt : m->partPool()) prt.setVs(zero_vs);
		if (pollute_tail) m->partPool().back().setVs(tail_vs);

		std::vector<double> out_vel_stateful(m->outputVelSize(), 0.0);
		std::vector<double> out_vel_stateless(m->outputVelSize(), 0.0);
		std::vector<double> mv_stateful(m->inputVelSize(), 0.0);
		std::vector<double> mv_stateless(m->inputVelSize(), 0.0);

		m->setInputVel(cmd_v_ref);
		if (m->forwardKinematicsVel()) {
			std::cout << "forwardKinematicsVel failed in case " << label << std::endl;
			return;
		}
		m->getOutputVel(out_vel_stateful.data());

		if (fwd.kinVelPure(cmd_v_ref, out_vel_stateless.data())) {
			std::cout << "fwd.kinVelPure failed in case " << label << std::endl;
			return;
		}

		m->setWhichInverseRoot(&which_root);
		m->setInputPos(cmd_q);
		m->setOutputPos(out_pos.data());
		if (m->inverseKinematics()) {
			std::cout << "inverseKinematics failed in case " << label << std::endl;
			return;
		}

		m->setOutputVel(out_vel_target.data());
		if (m->inverseKinematicsVel()) {
			std::cout << "inverseKinematicsVel failed in case " << label << std::endl;
			return;
		}
		m->getInputVel(mv_stateful.data());

		if (inv.kinVelPure(out_vel_target.data(), mv_stateless.data())) {
			std::cout << "inv.kinVelPure failed in case " << label << std::endl;
			return;
		}

		std::cout << "\n--- " << label << " ---" << std::endl;
		print_vec("fwd_stateful_out_vel", out_vel_stateful);
		print_vec("fwd_stateless_out_vel", out_vel_stateless);
		print_diff("fwd_stateless_minus_stateful", out_vel_stateless, out_vel_stateful);
		print_diff("fwd_stateful_minus_target", out_vel_stateful, out_vel_target);
		print_diff("fwd_stateless_minus_target", out_vel_stateless, out_vel_target);

		print_vec("inv_stateful_mv", mv_stateful);
		print_vec("inv_stateless_mv", mv_stateless);
		print_diff("inv_stateful_minus_ref", mv_stateful, cmd_v_ref_vec);
		print_diff("inv_stateless_minus_ref", mv_stateless, cmd_v_ref_vec);
		print_diff("inv_stateless_minus_stateful", mv_stateless, mv_stateful);
	};

	run_case("no_pollute", false);
	run_case("tail_pollute_last_part_vs_123456", true);
	run_stateless_velocity_consistency_case();

	return 0;
}
}

void build_model() 
{
	
}

int main()
{
	std::cout <<"begin" << std::endl;
	
	//const char *debug_pollute = std::getenv("ARIS_DEMO_UR_DEBUG_POLLUTE");
	//if (debug_pollute && debug_pollute[0] != '0') {
	return run_debug_pollute_replay();
	//}

}