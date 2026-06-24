#include <aris.hpp>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <vector>

void print_arr(const char *name, const double *v, int n) {
    std::cout << name << " = [";
    for (int i = 0; i < n; ++i)
        std::cout << (i ? ", " : "") << std::setprecision(16) << v[i];
    std::cout << "]\n";
}

// ── helper: validate inverse-dynamics consistency ──────────────────────
// Returns the maximum |I_ground*a - Σconstraint_force| across all parts.
auto check_dynamics(aris::dynamic::Model &m) -> double {
	// to do:
	// 请在这里帮我计算相邻两个杆件的加速度差，并计算出来相应的电机加速度
	std::cout << "\n===== Motor accelerations from part as =====\n";
	for (auto &mot : m.motionPool()) {
		auto &part_i = mot.makI()->fatherPart();
		auto &part_j = mot.makJ()->fatherPart();

		double as[6], vs[6];  // relative spatial accel & vel (in marker frame)
		aris::dynamic::s_inv_as2as(*mot.makJ()->pm(), part_j.vs(), part_j.as(),
		            part_i.vs(), part_i.as(), as, vs);
		double motor_acc;
		mot.cptAFromAs(as, &motor_acc);

		std::cout << "  " << mot.name() << ": ma = "
		          << std::setprecision(16) << motor_acc
		          << "  (mot.ma() = " << mot.ma() << ")\n";
	}

	// to do:
	//
	// 这里帮我计算每个joint 的 cmI *(vsI-vsJ) 以及 cmI *(asI-asJ)，并确认它们的结果是否全为0，如果不是，也不要debug，仅告诉我结果既可。

	std::cout << "\n===== Joint constraint velocity / acceleration check =====\n";
	for (auto &jnt : m.jointPool()) {
		double cmI[36], cmJ[36];
		jnt.cptGlbCmFromPm(cmI, cmJ, *jnt.makI()->pm(), *jnt.makJ()->pm());
		int dim = (int)jnt.dim();

		auto &part_i = jnt.makI()->fatherPart();
		auto &part_j = jnt.makJ()->fatherPart();

		// ── velocity: cmI^T * (vs_marker_I - vs_marker_J) ──
		double vs_mI[6], vs_mJ[6];
		aris::dynamic::s_tv(*jnt.makI()->prtPm(), part_i.vs(), vs_mI);
		aris::dynamic::s_tv(*jnt.makJ()->prtPm(), part_j.vs(), vs_mJ);
		double vs_diff[6];
		for (int k = 0; k < 6; ++k) vs_diff[k] = vs_mI[k] - vs_mJ[k];

		double cv[6] = {0};
		for (int r = 0; r < dim; ++r)
			for (int c = 0; c < 6; ++c)
				cv[r] += cmI[c * 6 + r] * vs_diff[c];

		bool vel_nonzero = false;
		for (int k = 0; k < dim; ++k)
			if (std::abs(cv[k]) > 1e-9) vel_nonzero = true;

		// ── acceleration: cmI^T * (as_marker_I - as_marker_J) ──
		double as_mI[6], as_mJ[6];
		aris::dynamic::s_vc(6, part_i.as(), as_mI);
		aris::dynamic::s_vc(6, part_j.as(), as_mJ);
		double as_diff[6];
		for (int k = 0; k < 6; ++k) as_diff[k] = as_mI[k] - as_mJ[k];

		double ca[6] = {0};
		aris::dynamic::s_mm(5,1,6,cmI,aris::dynamic::T(5),as_diff,1,ca,1);

		bool acc_nonzero = false;
		for (int k = 0; k < dim; ++k)
			if (std::abs(ca[k]) > 1e-9) acc_nonzero = true;

		std::cout << "  " << jnt.name() << " (dim=" << dim
		          << "): cmI*(vsI-vsJ)";
		if (vel_nonzero) {
			std::cout << " = [";
			for (int k = 0; k < dim; ++k)
				std::cout << (k ? "," : "") << std::setprecision(8) << cv[k];
			std::cout << "]  ← 非零!";
		} else {
			std::cout << " = 0";
		}
		std::cout << "  |  cmI*(asI-asJ)";
		if (acc_nonzero) {
			std::cout << " = [";
			for (int k = 0; k < dim; ++k)
				std::cout << (k ? "," : "") << std::setprecision(8) << ca[k];
			std::cout << "]  ← 非零!";
		} else {
			std::cout << " = 0";
		}
		std::cout << "\n";
	}



	// to do:
	// 
	// 请在这里帮我遍历所有的 joint 和 motion，计算它们对每个 part 的约束力（用 cmI/cmJ 乘 cf） 
	std::cout << "\n===== Constraint forces per constraint (Newton-III check) =====\n";
	std::vector<double> part_force(m.partPool().size() * 6, 0.0);

	auto process_constraint = [&](const aris::dynamic::Constraint &cst) {
		double cmI[36], cmJ[36];
		cst.cptGlbCmFromPm(cmI, cmJ, *cst.makI()->pm(), *cst.makJ()->pm());
		int dim = (int)cst.dim();
		const double *cf = cst.cf();
		int pi = cst.makI()->fatherPart().id();
		int pj = cst.makJ()->fatherPart().id();

		double fI[6] = {0}, fJ[6] = {0};
		aris::dynamic::s_mm(6, 1, dim, cmI, dim, cf, 1, fI, 1);
		aris::dynamic::s_mm(6, 1, dim, cmJ, dim, cf, 1, fJ, 1);

		for (int k = 0; k < 6; ++k) {
			part_force[pi * 6 + k] += fI[k];
			part_force[pj * 6 + k] += fJ[k];
		}
	};

	for (auto &jnt : m.jointPool())   process_constraint(jnt);
	for (auto &mot : m.motionPool())  process_constraint(mot);
	// general motion is deactive for inverse dynamics, skip it

	// ── total force across all parts ──
	double total[6] = {0};
	for (int i = 0; i < (int)m.partPool().size(); ++i)
		for (int k = 0; k < 6; ++k)
			total[k] += part_force[i * 6 + k];

	std::cout << "\n  Sum of all constraint forces on all parts:\n";
	std::cout << "  total = [";
	for (int k = 0; k < 6; ++k)
		std::cout << (k ? ", " : "") << std::setprecision(8) << total[k];
	std::cout << "]\n";

	double total_norm = 0;
	for (int k = 0; k < 6; ++k) total_norm += total[k] * total[k];
	total_norm = std::sqrt(total_norm);
	std::cout << "  |total| = " << total_norm;
	if (total_norm > 1e-9)
		std::cout << "  ← 合力不为零!";
	std::cout << "\n";


	// to do：
	// 请用 s_iv2iv + part.pm() + partIv() 将每个杆件的惯量变换到
	// 地面坐标系，再与地面坐标系下的加速度相乘，然后与约束力比较

	std::cout << "\n===== Inertial force (I_ground * a_ground) per part =====\n";
	for (int i = 0; i < (int)m.partPool().size(); ++i) {
		auto &part = m.partPool()[i];
		double iv_global[10];  // spatial inertia in ground frame
		aris::dynamic::s_iv2iv(*part.pm(), part.prtIv(), iv_global);

		double as_global[6], I_dot_a[6];
		part.getAs(as_global);  // global spatial acceleration
		aris::dynamic::s_iv_dot_as(iv_global, as_global, I_dot_a);

		std::cout << "  part[" << i << "] (" << part.name()
		          << ") I*a = [";
		for (int k = 0; k < 6; ++k)
			std::cout << (k ? ", " : "") << std::setprecision(8) << I_dot_a[k];
		std::cout << "]\n";
	}

	std::cout << "\n===== Check: I_ground*a vs constraint force sum per part =====\n";
	double max_err = 0.0;
	for (int i = 1; i < (int)m.partPool().size(); ++i) {
		auto &part = m.partPool()[i];
		double iv_global[10];
		aris::dynamic::s_iv2iv(*part.pm(), part.prtIv(), iv_global);

		double as_global[6], I_dot_a[6];
		part.getAs(as_global);
		aris::dynamic::s_iv_dot_as(iv_global, as_global, I_dot_a);

		double diff[6], diff_norm = 0;
		for (int k = 0; k < 6; ++k) {
			diff[k] = I_dot_a[k] - part_force[i * 6 + k];
			diff_norm += diff[k] * diff[k];
		}
		diff_norm = std::sqrt(diff_norm);
		if (diff_norm > max_err) max_err = diff_norm;
		std::cout << "  part[" << i << "] |I*a - Σconstraint| = " << diff_norm;
		if (diff_norm > 1e-9)
			std::cout << "  ← 不匹配!";
		std::cout << "\n";
	}
	return max_err;
}

int main() {
    // 1. create UR model
    aris::dynamic::UrParam param;
    param.H1 = 0.089159;
    param.W1 = 0.13585 - 0.1197 + 0.093;
    param.L1 = 0.425;   param.L2 = 0.39225;
    param.H2 = -0.09465; param.W2 = 0.0823;
    param.install_method = 0;
    auto m = aris::dynamic::createModelUr(param);

	double gravity[6]{0,0,0,0,0,0};
	m->environment().setGravity(gravity);
	m->init();

    // 2. data: positions, velocities, accelerations (factor=1)
    // const double cmd_q[6]{-0.14, 0.11, -0.09, 0.16, -0.12, 0.07};
    // const double cmd_v[6]{0.025, -0.018, 0.021, -0.015, 0.013, -0.011};
    // const double cmd_a[6]{-0.032, 0.027, -0.022, 0.019, -0.016, 0.014};
    // const double tau[6]{-0.228362181000519, 1.647532593224324, 3.349343785322493, 3.478416887326793, -0.1442579360727938, 0.03858765658378447};

	// const double cmd_q[6]{0,0,0,0,0,0};
    // const double cmd_v[6]{0,0,0,0,0,0};
    // const double cmd_a[6]{0,0,0,0,0,0.1};
    // const double tau[6]{0, 0.1, 0.1, 0.1, 0, 0.1};

	// const double cmd_q[6]{0,0,0,0,0,0};
    // const double cmd_v[6]{0,0,0,0,0,0};
    // const double cmd_a[6]{0.01, -0.05, -0.15, 0.3, 0.2, 0.1};
    // const double tau[6]{0.4844174372750001, 0.08216360686594992, 0.30460511073, 0.7373381148611999, 0.4121035361150001, 0.2};

	const double cmd_q[6]{0, 0, 0, 0, 0.3, 0.0};
    const double cmd_v[6]{0, 0, 0, 0, 0, 0};
    const double cmd_a[6]{0.0, 0.0, 0.0, 0.3, 0.0, 0.0};
    const double tau[6]{0.00665747741422852, 0.928460078350723, 1.171662248350723, 1.396123545250723, -0.02612219954077147, 0.2866009467376818};

	// const double cmd_q[6]{0, 0, 0, 0, 0.3, 0.0};
    // const double cmd_v[6]{0, 0, 0, 0, 0, 0};
    // const double cmd_a[6]{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    // const double tau[6]{0, -0.07776023197879819, -0.07776023197879819, -0.07776023197879819, 0, 4.213623736612206e-17};


    // 5. forward dynamics — the failing step
    m->setInputPos(cmd_q);  
	m->setInputVel(cmd_v);
	m->forwardKinematics(); 
	m->forwardKinematicsVel();
	

    // double cmd_tau_fd[6];
	// m->setInputAcc(cmd_a);
	// m->inverseDynamics();
	// m->getInputFce(cmd_tau_fd);
	// print_arr("cmd_tau_fd:\n", cmd_tau_fd, 6);
	// print_arr("cmd_tau (expected):\n", tau, 6);

	// double max_err = check_dynamics(*m);
	// std::cout << "\nMax |I*a - Σconstraint| = " << max_err << "\n";

	// bool ok = true;
	// for (int i = 0; i < 6; ++i)
    //     if (std::abs(cmd_tau_fd[i] - tau[i]) > 1e-10)
    //         { 
	// 			ok = false; 
	// 		}
    // std::cout << (ok ? "  PASS\n" : "  FAIL\n");

    double cmd_a_fd[6];
	m->setInputFce(tau); 
	m->forwardDynamics();
	m->getInputAcc(cmd_a_fd);
	print_arr("cmd_a_fd:\n", cmd_a_fd, 6);
    print_arr("cmd_a (expected):\n", cmd_a, 6);

    bool ok = true;
    for (int i = 0; i < 6; ++i)
        if (std::abs(cmd_a_fd[i] - cmd_a[i]) > 1e-10)
            { 
				ok = false; 
			}
    std::cout << (ok ? "  PASS\n" : "  FAIL\n");
	


    return ok ? 0 : 1;
}