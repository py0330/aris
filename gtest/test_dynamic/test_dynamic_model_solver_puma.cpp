#include <gtest/gtest.h>

#include <array>
#include <vector>

#include <aris/dynamic/dynamic.hpp>
#include <aris/dynamic/mechanism_puma.hpp>

namespace {

struct MotorScaleCase {
    double factor;
    double offset;
};

auto createPumaModel() -> std::unique_ptr<aris::dynamic::Model> {
    aris::dynamic::PumaParam param;
    param.d1 = 0.3295;
    param.a1 = 0.04;
    param.a2 = 0.275;
    param.d3 = 0.0;
    param.a3 = 0.025;
    param.d4 = 0.28;
    param.install_method = 0;

    // Match the historical EE frames used by legacy tests.
    param.tool0_pe[0] = -0.005;
    param.tool0_pe[1] = 0.05;
    param.tool0_pe[2] = 0.0005;
    param.tool0_pe[3] = 0.1;
    param.tool0_pe[4] = 0.03;
    param.tool0_pe[5] = 0.15;

    param.base2ref_pe[0] = 0.013;
    param.base2ref_pe[1] = -0.15;
    param.base2ref_pe[2] = 0.1;
    param.base2ref_pe[3] = 0.01;
    param.base2ref_pe[4] = 0.02;
    param.base2ref_pe[5] = 0.2;

    return aris::dynamic::createModelPuma(param);
}

void applyMotorFactorOffset(aris::dynamic::Model &m, double factor, double offset) {
    for (auto &mot : m.motionPool()) {
        mot.setMpFactor(factor);
        mot.setMpOffset(offset);
    }
}

void physicalToMotorCmd(const double phy[6], double cmd[6], double factor, double offset) {
    for (int i = 0; i < 6; ++i) {
        cmd[i] = phy[i] / factor - offset;
    }
}

auto motor_scale_cases() -> std::array<MotorScaleCase, 3> {
    return {{
        MotorScaleCase{0.1, 3.0},
        MotorScaleCase{0.2, -1.5},
        MotorScaleCase{1.0, 0.0},
    }};
}

TEST(ModelSolverPuma, PositionRoundTrip) {
    for (const auto &p : motor_scale_cases()) {
        SCOPED_TRACE(::testing::Message() << "factor=" << p.factor << " offset=" << p.offset);

        auto m = createPumaModel();
        applyMotorFactorOffset(*m, p.factor, p.offset);

        const double phy_samples[][6]{
            {0.05, -0.10, 0.20, -0.30, 0.40, -0.50},
            {-0.20, 0.15, -0.10, 0.25, -0.35, 0.45},
            {0.30, 0.20, -0.15, -0.10, 0.05, 0.00},
        };

        const auto root_num = m->inverseRootNumber();
        ASSERT_GT(root_num, 0);
        std::vector<int> root_covered(static_cast<size_t>(root_num), 0);

        for (const auto &phy_q : phy_samples) {
            double cmd_q[6], out_pos[6];
            physicalToMotorCmd(phy_q, cmd_q, p.factor, p.offset);

            EXPECT_TRUE(m->forwardKinematics(cmd_q, out_pos, nullptr, cmd_q) == 0);

            for (std::int64_t root = 0; root < root_num; ++root) {
                double cmd_q_back[6], out_pos_recheck[6];
                if (m->inverseKinematics(out_pos, cmd_q_back, &root, cmd_q) != 0) {
                    continue;
                }

                root_covered[static_cast<size_t>(root)] = 1;
                EXPECT_TRUE(m->forwardKinematics(cmd_q_back, out_pos_recheck, nullptr, cmd_q_back) == 0);
                EXPECT_TRUE(aris::dynamic::s_is_equal(6, 1, out_pos, out_pos_recheck, 1e-8));
            }
        }

        int covered_count = 0;
        for (auto c : root_covered) covered_count += c;
        EXPECT_EQ(covered_count, static_cast<int>(root_num));
    }
}

TEST(ModelSolverPuma, VelAccForceRoundTrip) {
    for (const auto &p : motor_scale_cases()) {
        SCOPED_TRACE(::testing::Message() << "factor=" << p.factor << " offset=" << p.offset);

        auto m = createPumaModel();
        applyMotorFactorOffset(*m, p.factor, p.offset);

        const double phy_q[6]{-0.14, 0.11, -0.09, 0.16, -0.12, 0.07};
        const double cmd_v[6]{0.25, -0.18, 0.21, -0.15, 0.13, -0.11};
        const double cmd_a[6]{-0.32, 0.27, -0.22, 0.19, -0.16, 0.14};
        double cmd_q[6];
        physicalToMotorCmd(phy_q, cmd_q, p.factor, p.offset);

        double out_pos[6], out_vel[6], out_acc[6], tau[6];
        m->setInputPos(cmd_q);
        m->setInputVel(cmd_v);
        m->setInputAcc(cmd_a);
        EXPECT_TRUE(m->forwardKinematics() == 0);
        EXPECT_TRUE(m->forwardKinematicsVel() == 0);
        EXPECT_TRUE(m->forwardKinematicsAcc() == 0);
        EXPECT_TRUE(m->inverseDynamics() == 0);
        m->getOutputPos(out_pos);
        m->getOutputVel(out_vel);
        m->getOutputAcc(out_acc);
        m->getInputFce(tau);

        m->setInputPos(cmd_q);
        m->setOutputPos(out_pos);
        m->setOutputVel(out_vel);
        m->setOutputAcc(out_acc);
        EXPECT_TRUE(m->inverseKinematics() == 0);
        EXPECT_TRUE(m->inverseKinematicsVel() == 0);
        EXPECT_TRUE(m->inverseKinematicsAcc() == 0);
        EXPECT_TRUE(m->inverseDynamics() == 0);

        double cmd_q_back[6], cmd_v_back[6], cmd_a_back[6], tau_back[6];
        m->getInputPos(cmd_q_back);
        m->getInputVel(cmd_v_back);
        m->getInputAcc(cmd_a_back);
        m->getInputFce(tau_back);

        EXPECT_TRUE(aris::dynamic::s_is_equal(6, 1, cmd_q, cmd_q_back, 1e-7));
        EXPECT_TRUE(aris::dynamic::s_is_equal(6, 1, cmd_v, cmd_v_back, 1e-7));
        EXPECT_TRUE(aris::dynamic::s_is_equal(6, 1, cmd_a, cmd_a_back, 1e-7));
        EXPECT_TRUE(aris::dynamic::s_is_equal(6, 1, tau, tau_back, 1e-6));

        m->setInputPos(cmd_q);
        m->setInputVel(cmd_v);
        m->setInputFce(tau);
        EXPECT_TRUE(m->forwardDynamics() == 0);
        double cmd_a_from_fd[6];
        m->getInputAcc(cmd_a_from_fd);
        EXPECT_TRUE(aris::dynamic::s_is_equal(6, 1, cmd_a, cmd_a_from_fd, 1e-6));
    }
}

TEST(ModelSolverPuma, JacobianConsistency) {
    for (const auto &p : motor_scale_cases()) {
        SCOPED_TRACE(::testing::Message() << "factor=" << p.factor << " offset=" << p.offset);

        auto m = createPumaModel();
        applyMotorFactorOffset(*m, p.factor, p.offset);

        const double phy_q[6]{0.17, -0.09, 0.14, -0.22, 0.19, -0.11};
        double cmd_q[6];
        physicalToMotorCmd(phy_q, cmd_q, p.factor, p.offset);

        m->setInputPos(cmd_q);
        EXPECT_TRUE(m->forwardKinematics() == 0);

        double out_pos[6];
        m->getOutputPos(out_pos);

        auto &inv = dynamic_cast<aris::dynamic::PumaInverseKinematicSolver&>(m->solverPool().at(0));
        auto &fwd = dynamic_cast<aris::dynamic::ForwardKinematicSolver&>(m->solverPool().at(1));
        inv.cptJacobi();
        fwd.cptJacobi();

        double jf_ji[36];
        aris::dynamic::s_mm(6, 6, 6, fwd.Jf(), inv.Ji(), jf_ji);
        double I[36]{
            1,0,0,0,0,0,
            0,1,0,0,0,0,
            0,0,1,0,0,0,
            0,0,0,1,0,0,
            0,0,0,0,1,0,
            0,0,0,0,0,1,
        };
        EXPECT_TRUE(aris::dynamic::s_is_equal(36, jf_ji, I, 1e-6));

        for (int col = 0; col < 6; ++col) {
            double out_vel_target[6]{0, 0, 0, 0, 0, 0};
            out_vel_target[col] = 1.0;

            m->setInputPos(cmd_q);
            m->setOutputPos(out_pos);
            m->setOutputVel(out_vel_target);
            EXPECT_TRUE(m->inverseKinematics() == 0);
            EXPECT_TRUE(m->inverseKinematicsVel() == 0);

            double cmd_v[6];
            m->getInputVel(cmd_v);

            m->setInputPos(cmd_q);
            m->setInputVel(cmd_v);
            EXPECT_TRUE(m->forwardKinematics() == 0);
            EXPECT_TRUE(m->forwardKinematicsVel() == 0);

            double out_vel_recheck[6];
            m->getOutputVel(out_vel_recheck);
            EXPECT_TRUE(aris::dynamic::s_is_equal(6, 1, out_vel_target, out_vel_recheck, 1e-7));
        }
    }
}

TEST(ModelSolverPuma, DynamicMatrixConsistency) {
    for (const auto &p : motor_scale_cases()) {
        SCOPED_TRACE(::testing::Message() << "factor=" << p.factor << " offset=" << p.offset);

        auto m = createPumaModel();
        applyMotorFactorOffset(*m, p.factor, p.offset);

        const double phy_q[6]{0.12, -0.16, 0.21, -0.19, 0.14, -0.08};
        const double cmd_v[6]{0.18, -0.22, 0.15, -0.11, 0.09, -0.07};
        const double cmd_a_test[6]{-0.27, 0.23, -0.19, 0.17, -0.13, 0.11};
        const double eps = 1e-6;

        double cmd_q[6];
        physicalToMotorCmd(phy_q, cmd_q, p.factor, p.offset);

        double h[6], a_zero[6]{0, 0, 0, 0, 0, 0};
        m->setInputPos(cmd_q);
        m->setInputVel(cmd_v);
        m->setInputAcc(a_zero);
        EXPECT_TRUE(m->inverseDynamics() == 0);
        m->getInputFce(h);

        double M[36]{0};
        for (int col = 0; col < 6; ++col) {
            double a_eps[6]{0, 0, 0, 0, 0, 0};
            a_eps[col] = eps;

            double tau_eps[6];
            m->setInputPos(cmd_q);
            m->setInputVel(cmd_v);
            m->setInputAcc(a_eps);
            EXPECT_TRUE(m->inverseDynamics() == 0);
            m->getInputFce(tau_eps);

            for (int row = 0; row < 6; ++row) {
                M[row * 6 + col] = (tau_eps[row] - h[row]) / eps;
            }
        }

        double tau_model[6];
        m->setInputPos(cmd_q);
        m->setInputVel(cmd_v);
        m->setInputAcc(cmd_a_test);
        EXPECT_TRUE(m->inverseDynamics() == 0);
        m->getInputFce(tau_model);

        double tau_mat[6]{0, 0, 0, 0, 0, 0};
        for (int row = 0; row < 6; ++row) {
            double sum = h[row];
            for (int col = 0; col < 6; ++col) {
                sum += M[row * 6 + col] * cmd_a_test[col];
            }
            tau_mat[row] = sum;
        }
        EXPECT_TRUE(aris::dynamic::s_is_equal(6, 1, tau_model, tau_mat, 5e-5));

        m->setInputPos(cmd_q);
        m->setInputVel(cmd_v);
        m->setInputFce(tau_model);
        EXPECT_TRUE(m->forwardDynamics() == 0);
        double cmd_a_fd[6];
        m->getInputAcc(cmd_a_fd);
        EXPECT_TRUE(aris::dynamic::s_is_equal(6, 1, cmd_a_test, cmd_a_fd, 5e-5));
    }
}

} // namespace
