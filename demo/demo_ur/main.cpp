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

int main() {
    // 1. create UR model
    aris::dynamic::UrParam param;
    param.H1 = 0.089159;
    param.W1 = 0.13585 - 0.1197 + 0.093;
    param.L1 = 0.425;   param.L2 = 0.39225;
    param.H2 = -0.09465; param.W2 = 0.0823;
    param.install_method = 0;
    auto m = aris::dynamic::createModelUr(param);

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
    const double tau[6]{0.00665747741422852, 0.8506998463719249, 1.093902016371925, 1.318363313271925, -0.02612219954077147, 0.2866009467376818};

	// const double cmd_q[6]{0, 0, 0, 0, 0.3, 0.0};
    // const double cmd_v[6]{0, 0, 0, 0, 0, 0};
    // const double cmd_a[6]{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    // const double tau[6]{0, -0.07776023197879819, -0.07776023197879819, -0.07776023197879819, 0, 4.213623736612206e-17};


    // 5. forward dynamics — the failing step
    m->setInputPos(cmd_q);  
	m->setInputVel(cmd_v);
	m->forwardKinematics(); 
	m->forwardKinematicsVel();
	
    

    double cmd_a_fd[6];
	m->setInputFce(tau); 
	m->forwardDynamics();
	m->getInputAcc(cmd_a_fd);
	print_arr("cmd_a_fd:\n", cmd_a_fd, 6);
    print_arr("cmd_a (expected):\n", cmd_a, 6);


	double cmd_tau_fd[6];
	m->setInputAcc(cmd_a);
	m->inverseDynamics();
	m->getInputFce(cmd_tau_fd);
	print_arr("cmd_tau_fd:\n", cmd_tau_fd, 6);
	print_arr("cmd_tau (expected):\n", tau, 6);

	// cmd_a_fd = [-0.0810145887252834, -0.0329731988943424, -0.04159520652372094, -0.003793694012478449, -0.3433349067821795, -3.536170261369297e-18]
    // cmd_a (expected) = [-0.032, 0.027, -0.022, 0.019, -0.016, 0.014]
	

	

    bool ok = true;
    for (int i = 0; i < 6; ++i)
        if (std::abs(cmd_a_fd[i] - cmd_a[i]) > 1e-10)
            { 
				ok = false; 
			}
    std::cout << (ok ? "  PASS\n" : "  FAIL\n");
	
	ok = true;
	for (int i = 0; i < 6; ++i)
        if (std::abs(cmd_tau_fd[i] - tau[i]) > 1e-10)
            { 
				ok = false; 
			}
    std::cout << (ok ? "  PASS\n" : "  FAIL\n");

    return ok ? 0 : 1;
}