#include <gtest/gtest.h>

#include <cstdio>
#include <array>
#include <cmath>
#include <vector>

#include <aris/dynamic/dynamic.hpp>
#include <aris/dynamic/mechanism_ur.hpp>

namespace {

struct MotorScaleCase {
    double factor;
    double offset;
};

auto createUrModel() -> std::unique_ptr<aris::dynamic::Model> {
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

void applyMotorFactorOffset(aris::dynamic::Model &m, double factor, double offset) {
    for (auto &mot : m.motionPool()) {
        mot.setMpFactor(factor);
        mot.setMpOffset(offset);
    }
}

void physicalToMotorCmd(const double *phy, double *cmd, int n, double factor, double offset) {
    for (int i = 0; i < n; ++i) cmd[i] = phy[i] / factor - offset;
}

void motorCmdToPhysical(const double *cmd, double *phy, int n, double factor, double offset) {
    for (int i = 0; i < n; ++i) phy[i] = factor * (cmd[i] + offset);
}

template <size_t N>
void setMotorPosLimitsFromSamples(aris::dynamic::Model &m, const double (&phy_samples)[N][6], double factor, double offset, double phy_margin = 0.5) {
    for (int axis = 0; axis < 6; ++axis) {
        double min_phy = phy_samples[0][axis];
        double max_phy = phy_samples[0][axis];
        for (size_t i = 1; i < N; ++i) {
            min_phy = std::min(min_phy, phy_samples[i][axis]);
            max_phy = std::max(max_phy, phy_samples[i][axis]);
        }

        const double min_cmd = (min_phy - phy_margin) / factor - offset;
        const double max_cmd = (max_phy + phy_margin) / factor - offset;
        m.motionPool().at(static_cast<aris::Size>(axis)).setMinMp(min_cmd);
        m.motionPool().at(static_cast<aris::Size>(axis)).setMaxMp(max_cmd);
    }
}

auto motor_scale_cases() -> std::array<MotorScaleCase, 3> {
    return {{
        MotorScaleCase{0.1, 3.0},
        MotorScaleCase{0.2, -1.5},
        MotorScaleCase{1.0, 0.0},
    }};
}

TEST(ModelSolverUr, InverseKinematicsCombinedFlow) {
    for (const auto &p : motor_scale_cases()) {
        SCOPED_TRACE(::testing::Message() << "factor=" << p.factor << " offset=" << p.offset);

        auto m = createUrModel();
        applyMotorFactorOffset(*m, p.factor, p.offset);

        const double two_pi = 2.0 * aris::PI;
        const double phy_samples[][6]{
            {0.05, -0.10, 0.20, -0.30, 0.40, -0.50},
            {-0.20, 0.15, -0.10, 0.25, -0.35, 0.45},
            {0.30, 0.20, -0.15, -0.10, 0.05, 0.00},
            {two_pi + 0.30, -0.25, 0.20, -two_pi - 0.40, 0.15, -0.05},
        };
        setMotorPosLimitsFromSamples(*m, phy_samples, p.factor, p.offset, 20.0);

        const auto out_pos_size = static_cast<int>(m->outputPosSize());
        const auto root_num = m->inverseRootNumber();
        ASSERT_GT(root_num, 0);
        std::vector<int> root_covered_stateless(static_cast<size_t>(root_num), 0);
        std::vector<int> root_covered_stateful(static_cast<size_t>(root_num), 0);

        for (const auto &phy_q : phy_samples) {
            double cmd_q[6];
            physicalToMotorCmd(phy_q, cmd_q, 6, p.factor, p.offset);

            std::vector<double> out_pos(out_pos_size);
            EXPECT_TRUE(m->forwardKinematics(cmd_q, out_pos.data(), nullptr, cmd_q) == 0);

            std::int64_t root_of_cmd_q{-1};
            EXPECT_TRUE(m->getWhichInverseRoot(out_pos.data(), cmd_q, &root_of_cmd_q) == 0);

            std::vector<std::pair<std::int64_t, std::array<double, 6>>> candidates;
            candidates.reserve(static_cast<size_t>(root_num));

            // 1) Stateless API: inverseKinematics(output, input, which_root, current_input)
            for (std::int64_t root = 0; root < root_num; ++root) {
                double cmd_q_back_direct[6]{0, 0, 0, 0, 0, 0};
                std::vector<double> out_pos_recheck(out_pos_size);
                auto this_root = root;

                if (m->inverseKinematics(out_pos.data(), cmd_q_back_direct, &this_root, cmd_q) != 0) {
                    continue;
                }

                root_covered_stateless[static_cast<size_t>(root)] = 1;
                EXPECT_TRUE(m->forwardKinematics(cmd_q_back_direct, out_pos_recheck.data(), nullptr, cmd_q_back_direct) == 0);
                EXPECT_TRUE(aris::dynamic::s_is_equal(out_pos_size, 1, out_pos.data(), out_pos_recheck.data(), 1e-8));

                std::array<double, 6> c{};
                for (int i = 0; i < 6; ++i) c[static_cast<size_t>(i)] = cmd_q_back_direct[i];
                candidates.emplace_back(root, c);

                if (root == root_of_cmd_q) {
                    EXPECT_TRUE(aris::dynamic::s_is_equal(6, 1, cmd_q, cmd_q_back_direct, 1e-9));
                }
            }

            ASSERT_GT(candidates.size(), 0U);

            // Stateless auto-root should follow current_input.
            if (candidates.size() >= 2U) {
                for (size_t ti = 0; ti < candidates.size(); ++ti) {
                    const auto &target = candidates[ti];
                    const auto &distractor = candidates[(ti + 1U) % candidates.size()];

                    m->setInputPos(distractor.second.data());

                    double result_direct[6]{0, 0, 0, 0, 0, 0};
                    EXPECT_TRUE(m->inverseKinematics(out_pos.data(), result_direct, nullptr, target.second.data()) == 0);

                    std::int64_t chosen_root_direct{-1};
                    EXPECT_TRUE(m->getWhichInverseRoot(out_pos.data(), result_direct, &chosen_root_direct) == 0);
                    EXPECT_EQ(chosen_root_direct, target.first);
                }
            }

            // 2) Stateful API: inverseKinematics()
            for (std::int64_t root = 0; root < root_num; ++root) {
                double cmd_q_back_state[6]{0, 0, 0, 0, 0, 0};
                std::vector<double> out_pos_recheck(out_pos_size);
                auto this_root = root;

                m->setWhichInverseRoot(&this_root);
                m->setInputPos(cmd_q);
                m->setOutputPos(out_pos.data());
                if (m->inverseKinematics() != 0) {
                    continue;
                }
                m->getInputPos(cmd_q_back_state);

                root_covered_stateful[static_cast<size_t>(root)] = 1;
                EXPECT_TRUE(m->forwardKinematics(cmd_q_back_state, out_pos_recheck.data(), nullptr, cmd_q_back_state) == 0);
                EXPECT_TRUE(aris::dynamic::s_is_equal(out_pos_size, 1, out_pos.data(), out_pos_recheck.data(), 1e-8));

                if (root == root_of_cmd_q) {
                    EXPECT_TRUE(aris::dynamic::s_is_equal(6, 1, cmd_q, cmd_q_back_state, 1e-9));
                }
            }

            // Stateful auto-root should choose the nearest initial guess.
            for (const auto &cand : candidates) {
                double initial[6], result[6];
                std::int64_t auto_root{-1};
                for (int i = 0; i < 6; ++i) {
                    initial[i] = cand.second[static_cast<size_t>(i)] + (i % 2 == 0 ? 1e-4 : -1e-4);
                }

                m->setWhichInverseRoot(&auto_root);
                m->setInputPos(initial);
                m->setOutputPos(out_pos.data());
                EXPECT_TRUE(m->inverseKinematics() == 0);
                m->getInputPos(result);

                std::int64_t chosen_root{-1};
                EXPECT_TRUE(m->getWhichInverseRoot(out_pos.data(), result, &chosen_root) == 0);
                EXPECT_EQ(chosen_root, cand.first);
            }

            int covered_stateless = 0;
            int covered_stateful = 0;
            for (auto c : root_covered_stateless) covered_stateless += c;
            for (auto c : root_covered_stateful) covered_stateful += c;
            EXPECT_EQ(covered_stateless, static_cast<int>(root_num));
            EXPECT_EQ(covered_stateful, static_cast<int>(root_num));
        }
    }
}

TEST(ModelSolverUr, VelAccForceRoundTrip) {
    for (const auto &p : motor_scale_cases()) {
        SCOPED_TRACE(::testing::Message() << "factor=" << p.factor << " offset=" << p.offset);

        auto m = createUrModel();
        applyMotorFactorOffset(*m, p.factor, p.offset);

        const double phy_q[6]{-0.14, 0.11, -0.09, 0.16, -0.12, 0.07};
        const double phy_v[6]{0.025, -0.018, 0.021, -0.015, 0.013, -0.011};
        const double phy_a[6]{-0.032, 0.027, -0.022, 0.019, -0.016, 0.014};
        static const double out_pos_expected[6]{0.073418907191431307, 0.18239762681842445, 0.80887723303246817, 6.1092869809501833, -1.32116056256423, 4.6255283870689334};
        static const double out_vel_expected[6]{-0.010813777434918556, 0.002846120816297221, 0.00091941271231262933, -0.0021767231075832693, -0.022840620994553064, 0.038025720285703002};
        static const double out_acc_expected[6]{0.017950634052707765, -0.0046699378289260475, -0.0015104478043674549, 0.0046370614223827595, 0.037629090863995608, -0.04805458265728018};
        static const double tau_expected[6]{-0.0045940278720832013, 1.529566410922824, 3.2583776030209926, 3.392450705025293, -0.049432434546374117, -3.055099084055785e-17};

        double cmd_q[6];
        double cmd_v[6], cmd_a[6];
        physicalToMotorCmd(phy_q, cmd_q, 6, p.factor, p.offset);
        for (int i = 0; i < 6; ++i) {
            cmd_v[i] = phy_v[i] / p.factor;
            cmd_a[i] = phy_a[i] / p.factor;
        }

        std::vector<double> out_pos(m->outputPosSize()), out_vel(m->outputVelSize()), out_acc(m->outputAccSize()), tau(m->inputFceSize());

        // Forward kinematics / dynamics.
        m->setInputPos(cmd_q);
        m->setInputVel(cmd_v);
        m->setInputAcc(cmd_a);
        EXPECT_TRUE(m->forwardKinematics() == 0);
        EXPECT_TRUE(m->forwardKinematicsVel() == 0);
        EXPECT_TRUE(m->forwardKinematicsAcc() == 0);
        EXPECT_TRUE(m->inverseDynamics() == 0);
        m->getOutputPos(out_pos.data());
        m->getOutputVel(out_vel.data());
        m->getOutputAcc(out_acc.data());
        m->getInputFce(tau.data());

        EXPECT_TRUE(aris::dynamic::s_is_equal(6, 1, out_pos_expected, out_pos.data(), 1e-12));
        EXPECT_TRUE(aris::dynamic::s_is_equal(6, 1, out_vel_expected, out_vel.data(), 1e-12));
        EXPECT_TRUE(aris::dynamic::s_is_equal(6, 1, out_acc_expected, out_acc.data(), 1e-12));
        EXPECT_TRUE(aris::dynamic::s_is_equal(6, 1, tau_expected, tau.data(), 1e-12));

        // Inverse kinematics / inverse kinematics vel+acc / forward dynamics.
        std::int64_t which_root{0};
        EXPECT_TRUE(m->getWhichInverseRoot(out_pos.data(), cmd_q, &which_root) == 0);

        double cmd_q_back[6], cmd_v_back[6], cmd_a_back[6], cmd_a_fd[6];
        EXPECT_TRUE(m->inverseKinematics(out_pos.data(), cmd_q_back, &which_root, cmd_q) == 0);
        for (int i = 0; i < 6; ++i) {
            const double dq = std::remainder(p.factor * (cmd_q_back[i] - cmd_q[i]), 2.0 * aris::PI);
            EXPECT_NEAR(dq, 0.0, 1e-10);
        }

        m->setWhichInverseRoot(&which_root);
        m->setOutputPos(out_pos.data());
        EXPECT_TRUE(m->inverseKinematics() == 0);
        m->setOutputVel(out_vel.data());
        m->setOutputAcc(out_acc.data());
        EXPECT_TRUE(m->inverseKinematicsVel() == 0);
        EXPECT_TRUE(m->inverseKinematicsAcc() == 0);
        m->getInputVel(cmd_v_back);
        m->getInputAcc(cmd_a_back);

        std::vector<double> cmd_a_stateless(static_cast<size_t>(m->inputAccSize()), 123456.0);
        EXPECT_TRUE(m->inverseKinematicsAcc(out_acc.data(), cmd_a_stateless.data()) == 0);
        EXPECT_TRUE(aris::dynamic::s_is_equal(static_cast<int>(cmd_a_stateless.size()), 1, cmd_a_back, cmd_a_stateless.data(), 1e-10));

        m->setInputPos(cmd_q);
        m->setInputVel(cmd_v_back);
        m->setInputFce(tau.data());
        EXPECT_TRUE(m->forwardDynamics() == 0);
        m->getInputAcc(cmd_a_fd);

        double phy_q_back[6], phy_v_back[6], phy_a_back[6], phy_a_fd[6];
        motorCmdToPhysical(cmd_q_back, phy_q_back, 6, p.factor, p.offset);
        for (int i = 0; i < 6; ++i) {
            phy_v_back[i] = p.factor * cmd_v_back[i];
            phy_a_back[i] = p.factor * cmd_a_back[i];
            phy_a_fd[i] = p.factor * cmd_a_fd[i];

            const double d = std::remainder(phy_q_back[i] - phy_q[i], 2.0 * aris::PI);
            EXPECT_NEAR(d, 0.0, 1e-10);
            EXPECT_NEAR(phy_v_back[i], phy_v[i], 1e-10);
            EXPECT_NEAR(phy_a_back[i], phy_a[i], 1e-10);
            EXPECT_NEAR(phy_a_fd[i], phy_a[i], 1e-10);
        }
    }
}

TEST(ModelSolverUr, JacobianConsistency) {
    for (const auto &p : motor_scale_cases()) {
        SCOPED_TRACE(::testing::Message() << "factor=" << p.factor << " offset=" << p.offset);

        auto m = createUrModel();
        applyMotorFactorOffset(*m, p.factor, p.offset);

        const double phy_q[6]{0.17, -0.09, 0.14, -0.22, 0.19, -0.11};
        double cmd_q[6];
        physicalToMotorCmd(phy_q, cmd_q, 6, p.factor, p.offset);

        m->setInputPos(cmd_q);
        EXPECT_TRUE(m->forwardKinematics() == 0);

        std::vector<double> out_pos(m->outputPosSize());
        m->getOutputPos(out_pos.data());

        auto &inv = dynamic_cast<aris::dynamic::InverseKinematicSolver&>(m->solverPool().at(0));
        auto &fwd = dynamic_cast<aris::dynamic::ForwardKinematicSolver&>(m->solverPool().at(1));
        inv.cptJacobi();
        fwd.cptJacobi();

        const int m_out = static_cast<int>(m->outputVelSize());
        const int n_in = static_cast<int>(m->inputVelSize());
        std::vector<double> jf_ji(m_out * m_out, 0.0), I(m_out * m_out, 0.0);
        for (int i = 0; i < m_out; ++i) I[i * m_out + i] = 1.0;
        aris::dynamic::s_mm(m_out, m_out, n_in, fwd.Jf(), inv.Ji(), jf_ji.data());
        EXPECT_TRUE(aris::dynamic::s_is_equal(m_out * m_out, jf_ji.data(), I.data(), 1e-6));

        // Verify velocity mapping consistency between Jacobian and solver APIs.
        std::int64_t which_root{0};
        EXPECT_TRUE(m->getWhichInverseRoot(out_pos.data(), cmd_q, &which_root) == 0);

        std::vector<double> out_vel_target(m_out, 0.0);
        for (int i = 0; i < m_out; ++i) {
            out_vel_target[i] = (i % 2 == 0) ? (0.12 + 0.03 * i) : (-0.08 - 0.02 * i);
        }

        std::vector<double> cmd_v_from_jac(n_in, 0.0), cmd_v_from_ikvel(n_in, 0.0);
        aris::dynamic::s_mm(n_in, 1, m_out, inv.Ji(), out_vel_target.data(), cmd_v_from_jac.data());

        m->setWhichInverseRoot(&which_root);
        m->setInputPos(cmd_q);
        m->setOutputPos(out_pos.data());
        EXPECT_TRUE(m->inverseKinematics() == 0);
        m->setOutputVel(out_vel_target.data());
        EXPECT_TRUE(m->inverseKinematicsVel() == 0);
        m->getInputVel(cmd_v_from_ikvel.data());
        EXPECT_TRUE(aris::dynamic::s_is_equal(n_in, 1, cmd_v_from_jac.data(), cmd_v_from_ikvel.data(), 1e-5));

        std::vector<double> out_vel_from_jac(m_out, 0.0), out_vel_from_fkvel(m_out, 0.0);
        aris::dynamic::s_mm(m_out, 1, n_in, fwd.Jf(), cmd_v_from_ikvel.data(), out_vel_from_jac.data());

        m->setInputPos(cmd_q);
        m->setInputVel(cmd_v_from_ikvel.data());
        EXPECT_TRUE(m->forwardKinematics() == 0);
        EXPECT_TRUE(m->forwardKinematicsVel() == 0);
        m->getOutputVel(out_vel_from_fkvel.data());

        EXPECT_TRUE(aris::dynamic::s_is_equal(m_out, 1, out_vel_target.data(), out_vel_from_jac.data(), 1e-6));
        EXPECT_TRUE(aris::dynamic::s_is_equal(m_out, 1, out_vel_target.data(), out_vel_from_fkvel.data(), 1e-6));
        EXPECT_TRUE(aris::dynamic::s_is_equal(m_out, 1, out_vel_from_jac.data(), out_vel_from_fkvel.data(), 1e-6));

    }
}

TEST(ModelSolverUr, StatelessVelocityConsistencyClean) {
    for (const auto &p : motor_scale_cases()) {
        SCOPED_TRACE(::testing::Message() << "factor=" << p.factor << " offset=" << p.offset);

        auto m = createUrModel();
        applyMotorFactorOffset(*m, p.factor, p.offset);

        const double phy_q[6]{0.17, -0.09, 0.14, -0.22, 0.19, -0.11};
        double cmd_q[6];
        physicalToMotorCmd(phy_q, cmd_q, 6, p.factor, p.offset);

        m->setInputPos(cmd_q);
        ASSERT_EQ(m->forwardKinematics(), 0);

        std::vector<double> out_pos(m->outputPosSize());
        m->getOutputPos(out_pos.data());

        auto &inv = dynamic_cast<aris::dynamic::InverseKinematicSolver&>(m->solverPool().at(0));
        auto &fwd = dynamic_cast<aris::dynamic::ForwardKinematicSolver&>(m->solverPool().at(1));

        const int m_out = static_cast<int>(m->outputVelSize());
        const int n_in = static_cast<int>(m->inputVelSize());
        const double inv_tol = 1e-8;

        const double cmd_v_ref_arr[6]{0.12, -0.10, 0.08, -0.06, 0.04, -0.02};
        std::vector<double> cmd_v_ref(cmd_v_ref_arr, cmd_v_ref_arr + n_in);

        m->setInputVel(cmd_v_ref_arr);
        ASSERT_EQ(m->forwardKinematicsVel(), 0);
        std::vector<double> out_vel_target(m->outputVelSize(), 0.0);
        m->getOutputVel(out_vel_target.data());

        std::int64_t which_root{0};
        ASSERT_EQ(m->getWhichInverseRoot(out_pos.data(), cmd_q, &which_root), 0);

        std::vector<double> cmd_v_stateful_clean(n_in, 0.0), cmd_v_stateless_clean(n_in, 0.0);
        m->setWhichInverseRoot(&which_root);
        m->setInputPos(cmd_q);
        m->setOutputPos(out_pos.data());
        ASSERT_EQ(m->inverseKinematics(), 0);
        m->setOutputVel(out_vel_target.data());
        ASSERT_EQ(m->inverseKinematicsVel(), 0);
        m->getInputVel(cmd_v_stateful_clean.data());
        ASSERT_EQ(inv.kinVelPure(out_vel_target.data(), cmd_v_stateless_clean.data()), 0);

        EXPECT_TRUE(aris::dynamic::s_is_equal(n_in, 1, cmd_v_ref.data(), cmd_v_stateful_clean.data(), inv_tol));
        EXPECT_TRUE(aris::dynamic::s_is_equal(n_in, 1, cmd_v_ref.data(), cmd_v_stateless_clean.data(), inv_tol));
        EXPECT_TRUE(aris::dynamic::s_is_equal(n_in, 1, cmd_v_stateful_clean.data(), cmd_v_stateless_clean.data(), inv_tol));

        std::vector<double> out_from_stateful(m_out, 0.0), out_from_stateless(m_out, 0.0);
        m->setInputPos(cmd_q);
        ASSERT_EQ(m->forwardKinematics(), 0);
        m->setInputVel(cmd_v_stateful_clean.data());
        ASSERT_EQ(m->forwardKinematicsVel(), 0);
        m->getOutputVel(out_from_stateful.data());
        ASSERT_EQ(fwd.kinVelPure(cmd_v_stateless_clean.data(), out_from_stateless.data()), 0);
        EXPECT_TRUE(aris::dynamic::s_is_equal(m_out, 1, out_vel_target.data(), out_from_stateful.data(), 1e-6));
        EXPECT_TRUE(aris::dynamic::s_is_equal(m_out, 1, out_vel_target.data(), out_from_stateless.data(), 1e-6));
    }
}

TEST(ModelSolverUr, StatelessVelocityConsistencyPolluted) {
    for (const auto &p : motor_scale_cases()) {
        SCOPED_TRACE(::testing::Message() << "factor=" << p.factor << " offset=" << p.offset);

        auto m = createUrModel();
        applyMotorFactorOffset(*m, p.factor, p.offset);

        const double phy_q[6]{0.17, -0.09, 0.14, -0.22, 0.19, -0.11};
        double cmd_q[6];
        physicalToMotorCmd(phy_q, cmd_q, 6, p.factor, p.offset);

        m->setInputPos(cmd_q);
        ASSERT_EQ(m->forwardKinematics(), 0);

        std::vector<double> out_pos(m->outputPosSize());
        m->getOutputPos(out_pos.data());

        const int m_out = static_cast<int>(m->outputVelSize());
        const int n_in = static_cast<int>(m->inputVelSize());
        const double inv_tol = 1e-8;

        const double cmd_v_ref_arr[6]{0.12, -0.10, 0.08, -0.06, 0.04, -0.02};
        std::vector<double> cmd_v_ref(cmd_v_ref_arr, cmd_v_ref_arr + n_in);

        m->setInputVel(cmd_v_ref_arr);
        ASSERT_EQ(m->forwardKinematicsVel(), 0);
        std::vector<double> out_vel_target(m->outputVelSize(), 0.0);
        m->getOutputVel(out_vel_target.data());

        std::int64_t which_root{0};
        ASSERT_EQ(m->getWhichInverseRoot(out_pos.data(), cmd_q, &which_root), 0);

        m->setWhichInverseRoot(&which_root);
        m->setInputPos(cmd_q);
        m->setOutputPos(out_pos.data());
        ASSERT_EQ(m->inverseKinematics(), 0);

        std::vector<double> out_vel_dirty_2(m_out, 0.0);
        for (int i = 0; i < m_out; ++i) {
            out_vel_dirty_2[i] = (i % 2 == 0) ? (0.31 - 0.04 * i) : (-0.27 + 0.03 * i);
        }

        std::vector<double> cmd_v_stateful_polluted(n_in, 0.0), cmd_v_stateless_polluted(n_in, 0.0);
        m->setOutputVel(out_vel_dirty_2.data());
        ASSERT_EQ(m->inverseKinematicsVel(), 0);
        m->setOutputVel(out_vel_target.data());
        ASSERT_EQ(m->inverseKinematicsVel(), 0);
        m->getInputVel(cmd_v_stateful_polluted.data());
        ASSERT_EQ(m->inverseKinematicsVel(out_vel_target.data(), cmd_v_stateless_polluted.data()), 0);

        EXPECT_TRUE(aris::dynamic::s_is_equal(n_in, 1, cmd_v_ref.data(), cmd_v_stateful_polluted.data(), inv_tol));
        EXPECT_TRUE(aris::dynamic::s_is_equal(n_in, 1, cmd_v_ref.data(), cmd_v_stateless_polluted.data(), inv_tol));

        std::vector<double> out_from_stateful_polluted(m_out, 0.0), out_from_stateless_polluted(m_out, 0.0);
        m->setInputPos(cmd_q);
        ASSERT_EQ(m->forwardKinematics(), 0);
        m->setInputVel(cmd_v_stateful_polluted.data());
        ASSERT_EQ(m->forwardKinematicsVel(), 0);
        m->getOutputVel(out_from_stateful_polluted.data());
        ASSERT_EQ(m->forwardKinematicsVel(cmd_v_stateless_polluted.data(), out_from_stateless_polluted.data()), 0);
        EXPECT_TRUE(aris::dynamic::s_is_equal(m_out, 1, out_vel_target.data(), out_from_stateful_polluted.data(), 1e-6));
        EXPECT_TRUE(aris::dynamic::s_is_equal(m_out, 1, out_vel_target.data(), out_from_stateless_polluted.data(), 1e-6));
    }
}

TEST(ModelSolverUr, StatelessAccelerationConsistencyPolluted) {
    for (const auto &p : motor_scale_cases()) {
        SCOPED_TRACE(::testing::Message() << "factor=" << p.factor << " offset=" << p.offset);

        auto m = createUrModel();
        applyMotorFactorOffset(*m, p.factor, p.offset);

        const double phy_q[6]{0.17, -0.09, 0.14, -0.22, 0.19, -0.11};
        double cmd_q[6];
        physicalToMotorCmd(phy_q, cmd_q, 6, p.factor, p.offset);

        m->setInputPos(cmd_q);
        ASSERT_EQ(m->forwardKinematics(), 0);

        std::vector<double> out_pos(m->outputPosSize());
        m->getOutputPos(out_pos.data());

        const int n_in = static_cast<int>(m->inputAccSize());
        const double acc_tol = 1e-8;

        const double cmd_v_ref_arr[6]{0.12, -0.10, 0.08, -0.06, 0.04, -0.02};
        const double cmd_a_ref_arr[6]{-0.07, 0.05, -0.04, 0.03, -0.02, 0.01};

        m->setInputVel(cmd_v_ref_arr);
        ASSERT_EQ(m->forwardKinematicsVel(), 0);
        std::vector<double> out_vel_target(m->outputVelSize(), 0.0);
        m->getOutputVel(out_vel_target.data());

        m->setInputAcc(cmd_a_ref_arr);
        ASSERT_EQ(m->forwardKinematicsAcc(), 0);
        std::vector<double> out_acc_target(m->outputAccSize(), 0.0);
        m->getOutputAcc(out_acc_target.data());

        std::int64_t which_root{0};
        ASSERT_EQ(m->getWhichInverseRoot(out_pos.data(), cmd_q, &which_root), 0);

        // Align stateful kinematic branch (pos->vel) before stateless accel query.
        m->setWhichInverseRoot(&which_root);
        m->setInputPos(cmd_q);
        m->setOutputPos(out_pos.data());
        ASSERT_EQ(m->inverseKinematics(), 0);
        m->setOutputVel(out_vel_target.data());
        ASSERT_EQ(m->inverseKinematicsVel(), 0);

        // Baseline stateful result.
        m->setOutputAcc(out_acc_target.data());
        ASSERT_EQ(m->inverseKinematicsAcc(), 0);
        std::vector<double> cmd_a_stateful(n_in, 0.0);
        m->getInputAcc(cmd_a_stateful.data());

        // Pollute internal accel-related states by solving with unrelated output acceleration once.
        std::vector<double> out_acc_dirty(m->outputAccSize(), 0.0);
        for (int i = 0; i < static_cast<int>(out_acc_dirty.size()); ++i) {
            out_acc_dirty[static_cast<size_t>(i)] = (i % 2 == 0) ? (-0.19 + 0.02 * i) : (0.17 - 0.015 * i);
        }
        m->setOutputAcc(out_acc_dirty.data());
        ASSERT_EQ(m->inverseKinematicsAcc(), 0);

        // Stateless API should still recover the target input acceleration.
        std::vector<double> cmd_a_stateless_polluted(n_in, 123456.0);
        ASSERT_EQ(m->inverseKinematicsAcc(out_acc_target.data(), cmd_a_stateless_polluted.data()), 0);

        EXPECT_TRUE(aris::dynamic::s_is_equal(n_in, 1, cmd_a_stateful.data(), cmd_a_stateless_polluted.data(), acc_tol));
    }
}

TEST(ModelSolverUr, StatelessForwardAccelerationConsistencyPolluted) {
    for (const auto &p : motor_scale_cases()) {
        SCOPED_TRACE(::testing::Message() << "factor=" << p.factor << " offset=" << p.offset);

        auto m = createUrModel();
        applyMotorFactorOffset(*m, p.factor, p.offset);

        const double phy_q[6]{0.17, -0.09, 0.14, -0.22, 0.19, -0.11};
        double cmd_q[6];
        physicalToMotorCmd(phy_q, cmd_q, 6, p.factor, p.offset);

        m->setInputPos(cmd_q);
        ASSERT_EQ(m->forwardKinematics(), 0);

        const double cmd_v_ref_arr[6]{0.12, -0.10, 0.08, -0.06, 0.04, -0.02};
        const double cmd_a_ref_arr[6]{-0.07, 0.05, -0.04, 0.03, -0.02, 0.01};

        m->setInputVel(cmd_v_ref_arr);
        ASSERT_EQ(m->forwardKinematicsVel(), 0);

        const int out_acc_size = static_cast<int>(m->outputAccSize());
        const int n_in = static_cast<int>(m->inputAccSize());

        // Baseline stateful result.
        m->setInputAcc(cmd_a_ref_arr);
        ASSERT_EQ(m->forwardKinematicsAcc(), 0);
        std::vector<double> out_acc_stateful(out_acc_size, 0.0);
        m->getOutputAcc(out_acc_stateful.data());

        // Stateless result under clean state.
        std::vector<double> out_acc_stateless_clean(out_acc_size, 123456.0);
        ASSERT_EQ(m->forwardKinematicsAcc(cmd_a_ref_arr, out_acc_stateless_clean.data()), 0);
        EXPECT_TRUE(aris::dynamic::s_is_equal(out_acc_size, 1, out_acc_stateful.data(), out_acc_stateless_clean.data(), 1e-6));

        // Pollute the current state by solving with a different input acceleration.
        std::vector<double> cmd_a_dirty(n_in, 0.0);
        for (int i = 0; i < n_in; ++i) {
            cmd_a_dirty[static_cast<size_t>(i)] = (i % 2 == 0) ? (0.21 - 0.03 * i) : (-0.18 + 0.025 * i);
        }
        m->setInputAcc(cmd_a_dirty.data());
        ASSERT_EQ(m->forwardKinematicsAcc(), 0);

        std::vector<double> out_acc_stateless_polluted(out_acc_size, 123456.0);
        ASSERT_EQ(m->forwardKinematicsAcc(cmd_a_ref_arr, out_acc_stateless_polluted.data()), 0);
        EXPECT_TRUE(aris::dynamic::s_is_equal(out_acc_size, 1, out_acc_stateless_clean.data(), out_acc_stateless_polluted.data(), 1e-6));
    }
}

TEST(ModelSolverUr, StatelessInverseDynamicsConsistencyPolluted) {
    for (const auto &p : motor_scale_cases()) {
        SCOPED_TRACE(::testing::Message() << "factor=" << p.factor << " offset=" << p.offset);

        auto m = createUrModel();
        applyMotorFactorOffset(*m, p.factor, p.offset);

        const double phy_q[6]{-0.14, 0.11, -0.09, 0.16, -0.12, 0.07};
        const double phy_v[6]{0.025, -0.018, 0.021, -0.015, 0.013, -0.011};
        const double phy_a[6]{-0.032, 0.027, -0.022, 0.019, -0.016, 0.014};

        double cmd_q[6], cmd_v[6], cmd_a[6];
        physicalToMotorCmd(phy_q, cmd_q, 6, p.factor, p.offset);
        for (int i = 0; i < 6; ++i) {
            cmd_v[i] = phy_v[i] / p.factor;
            cmd_a[i] = phy_a[i] / p.factor;
        }

        m->setInputPos(cmd_q);
        m->setInputVel(cmd_v);
        m->setInputAcc(cmd_a);
        ASSERT_EQ(m->forwardKinematics(), 0);
        ASSERT_EQ(m->forwardKinematicsVel(), 0);
        ASSERT_EQ(m->forwardKinematicsAcc(), 0);

        // Baseline stateful result.
        ASSERT_EQ(m->inverseDynamics(), 0);
        std::vector<double> tau_stateful(m->inputFceSize(), 0.0);
        m->getInputFce(tau_stateful.data());

        // Pollute the model state with unrelated forces before calling the stateless API.
        std::vector<double> dirty_tau(m->inputFceSize(), 0.0);
        for (int i = 0; i < static_cast<int>(dirty_tau.size()); ++i) {
            dirty_tau[static_cast<size_t>(i)] = (i % 2 == 0) ? (0.41 - 0.05 * i) : (-0.33 + 0.04 * i);
        }
        m->setInputFce(dirty_tau.data());

        std::vector<double> tau_stateless(m->inputFceSize(), 123456.0);
        ASSERT_EQ(m->inverseDynamics(cmd_a, tau_stateless.data()), 0);
        EXPECT_TRUE(aris::dynamic::s_is_equal(static_cast<int>(tau_stateful.size()), 1, tau_stateful.data(), tau_stateless.data(), 1e-10));
    }
}

TEST(ModelSolverUr, StatelessForwardDynamicsConsistencyPolluted) {
    for (const auto &p : motor_scale_cases()) {
        SCOPED_TRACE(::testing::Message() << "factor=" << p.factor << " offset=" << p.offset);

        auto m = createUrModel();
        applyMotorFactorOffset(*m, p.factor, p.offset);

        const double phy_q[6]{-0.14, 0.11, -0.09, 0.16, -0.12, 0.07};
        const double phy_v[6]{0.025, -0.018, 0.021, -0.015, 0.013, -0.011};
        const double phy_a[6]{-0.032, 0.027, -0.022, 0.019, -0.016, 0.014};

        double cmd_q[6], cmd_v[6], cmd_a[6];
        physicalToMotorCmd(phy_q, cmd_q, 6, p.factor, p.offset);
        for (int i = 0; i < 6; ++i) {
            cmd_v[i] = phy_v[i] / p.factor;
            cmd_a[i] = phy_a[i] / p.factor;
        }

        m->setInputPos(cmd_q);
        m->setInputVel(cmd_v);
        m->setInputAcc(cmd_a);
        ASSERT_EQ(m->forwardKinematics(), 0);
        ASSERT_EQ(m->forwardKinematicsVel(), 0);
        ASSERT_EQ(m->forwardKinematicsAcc(), 0);

        // Build a dynamically consistent force input from inverse dynamics.
        ASSERT_EQ(m->inverseDynamics(), 0);
        std::vector<double> tau_ref(m->inputFceSize(), 0.0);
        m->getInputFce(tau_ref.data());

        // Baseline stateful forward dynamics result.
        m->setInputFce(tau_ref.data());
        ASSERT_EQ(m->forwardDynamics(), 0);
        std::vector<double> acc_stateful(m->inputAccSize(), 0.0);
        m->getInputAcc(acc_stateful.data());

        // Pollute the model state with unrelated force/acc before stateless call.
        std::vector<double> dirty_tau(m->inputFceSize(), 0.0), dirty_acc(m->inputAccSize(), 0.0);
        for (int i = 0; i < static_cast<int>(dirty_tau.size()); ++i) {
            dirty_tau[static_cast<size_t>(i)] = (i % 2 == 0) ? (-0.29 + 0.03 * i) : (0.22 - 0.02 * i);
        }
        for (int i = 0; i < static_cast<int>(dirty_acc.size()); ++i) {
            dirty_acc[static_cast<size_t>(i)] = (i % 2 == 0) ? (0.37 - 0.04 * i) : (-0.31 + 0.035 * i);
        }
        m->setInputFce(dirty_tau.data());
        m->setInputAcc(dirty_acc.data());

        std::vector<double> acc_stateless(m->inputAccSize(), 123456.0);
        ASSERT_EQ(m->forwardDynamics(tau_ref.data(), acc_stateless.data()), 0);

        EXPECT_TRUE(aris::dynamic::s_is_equal(static_cast<int>(acc_stateful.size()), 1, phy_a, acc_stateless.data(), 1e-10));
    }
}

TEST(ModelSolverUr, VelocityDiffModeCoverageInvFwd) {
    const std::array<bool, 2> vel_diff_modes{false, true};

    for (const auto &p : motor_scale_cases()) {
        for (bool if_compute_vel_by_diff : vel_diff_modes) {
            SCOPED_TRACE(::testing::Message()
                << "factor=" << p.factor
                << " offset=" << p.offset
                << " if_compute_vel_by_diff=" << if_compute_vel_by_diff);

            auto m = createUrModel();
            applyMotorFactorOffset(*m, p.factor, p.offset);

            auto &inv = dynamic_cast<aris::dynamic::InverseKinematicSolver&>(m->solverPool().at(0));
            auto &fwd = dynamic_cast<aris::dynamic::ForwardKinematicSolver&>(m->solverPool().at(1));
            inv.setIfComputeVelByDiff(if_compute_vel_by_diff);
            fwd.setIfComputeVelByDiff(if_compute_vel_by_diff);

            const double phy_q[6]{0.17, -0.09, 0.14, -0.22, 0.19, -0.11};
            double cmd_q[6];
            physicalToMotorCmd(phy_q, cmd_q, 6, p.factor, p.offset);

            m->setInputPos(cmd_q);
            ASSERT_EQ(m->forwardKinematics(), 0);

            std::vector<double> out_pos(m->outputPosSize());
            m->getOutputPos(out_pos.data());

            const int m_out = static_cast<int>(m->outputVelSize());
            const int n_in = static_cast<int>(m->inputVelSize());

            const double cmd_v_ref_arr[6]{0.12, -0.10, 0.08, -0.06, 0.04, -0.02};
            m->setInputVel(cmd_v_ref_arr);
            ASSERT_EQ(m->forwardKinematicsVel(), 0);

            std::vector<double> out_vel_target(m_out, 0.0);
            m->getOutputVel(out_vel_target.data());

            std::int64_t which_root{0};
            ASSERT_EQ(m->getWhichInverseRoot(out_pos.data(), cmd_q, &which_root), 0);

            std::vector<double> cmd_v_stateful(n_in, 0.0), cmd_v_stateless(n_in, 0.0);
            m->setWhichInverseRoot(&which_root);
            m->setInputPos(cmd_q);
            m->setOutputPos(out_pos.data());
            ASSERT_EQ(m->inverseKinematics(), 0);
            m->setOutputVel(out_vel_target.data());
            ASSERT_EQ(m->inverseKinematicsVel(), 0);
            m->getInputVel(cmd_v_stateful.data());
            ASSERT_EQ(inv.kinVelPure(out_vel_target.data(), cmd_v_stateless.data()), 0);

            EXPECT_TRUE(aris::dynamic::s_is_equal(n_in, 1, cmd_v_stateful.data(), cmd_v_stateless.data(), 1e-8));

            std::vector<double> out_vel_from_stateful(m_out, 0.0), out_vel_from_stateless(m_out, 0.0);
            m->setInputPos(cmd_q);
            ASSERT_EQ(m->forwardKinematics(), 0);
            m->setInputVel(cmd_v_stateful.data());
            ASSERT_EQ(m->forwardKinematicsVel(), 0);
            m->getOutputVel(out_vel_from_stateful.data());
            ASSERT_EQ(fwd.kinVelPure(cmd_v_stateless.data(), out_vel_from_stateless.data()), 0);

            EXPECT_TRUE(aris::dynamic::s_is_equal(m_out, 1, out_vel_target.data(), out_vel_from_stateful.data(), 1e-6));
            EXPECT_TRUE(aris::dynamic::s_is_equal(m_out, 1, out_vel_target.data(), out_vel_from_stateless.data(), 1e-6));
            EXPECT_TRUE(aris::dynamic::s_is_equal(m_out, 1, out_vel_from_stateful.data(), out_vel_from_stateless.data(), 1e-6));
        }
    }
}

TEST(ModelSolverUr, DynamicMatrixConsistency) {
    for (const auto &p : motor_scale_cases()) {
        SCOPED_TRACE(::testing::Message() << "factor=" << p.factor << " offset=" << p.offset);

        auto m = createUrModel();
        applyMotorFactorOffset(*m, p.factor, p.offset);

        const int n = static_cast<int>(m->inputAccSize());
        const int nf = static_cast<int>(m->inputFceSize());

        const double phy_q[6]{0.12, -0.16, 0.21, -0.19, 0.14, -0.08};
        const double cmd_v[6]{0.18, -0.22, 0.15, -0.11, 0.09, -0.07};
        const double cmd_a_test[6]{-0.27, 0.23, -0.19, 0.17, -0.13, 0.11};
        const double eps = 1e-6;

        double cmd_q[6];
        physicalToMotorCmd(phy_q, cmd_q, 6, p.factor, p.offset);

        std::vector<double> h(nf, 0.0), a_zero(n, 0.0);
        m->setInputPos(cmd_q);
        m->setInputVel(cmd_v);
        m->setInputAcc(a_zero.data());
        EXPECT_TRUE(m->inverseDynamics() == 0);
        m->getInputFce(h.data());

        std::vector<double> M(nf * n, 0.0);
        for (int col = 0; col < n; ++col) {
            std::vector<double> a_eps(n, 0.0), tau_eps(nf, 0.0);
            a_eps[col] = eps;

            m->setInputPos(cmd_q);
            m->setInputVel(cmd_v);
            m->setInputAcc(a_eps.data());
            EXPECT_TRUE(m->inverseDynamics() == 0);
            m->getInputFce(tau_eps.data());

            for (int row = 0; row < nf; ++row) {
                M[row * n + col] = (tau_eps[row] - h[row]) / eps;
            }
        }

        std::vector<double> tau_model(nf, 0.0), tau_mat(nf, 0.0), cmd_a_vec(n, 0.0);
        for (int i = 0; i < n && i < 6; ++i) cmd_a_vec[i] = cmd_a_test[i];

        m->setInputPos(cmd_q);
        m->setInputVel(cmd_v);
        m->setInputAcc(cmd_a_vec.data());
        EXPECT_TRUE(m->inverseDynamics() == 0);
        m->getInputFce(tau_model.data());

        for (int row = 0; row < nf; ++row) {
            double sum = h[row];
            for (int col = 0; col < n; ++col) sum += M[row * n + col] * cmd_a_vec[col];
            tau_mat[row] = sum;
        }
        EXPECT_TRUE(aris::dynamic::s_is_equal(nf, 1, tau_model.data(), tau_mat.data(), 5e-5));

        m->setInputPos(cmd_q);
        m->setInputVel(cmd_v);
        m->setInputFce(tau_model.data());
        EXPECT_TRUE(m->forwardDynamics() == 0);
        std::vector<double> cmd_a_fd(n, 0.0);
        m->getInputAcc(cmd_a_fd.data());
        EXPECT_TRUE(aris::dynamic::s_is_equal(n, 1, cmd_a_vec.data(), cmd_a_fd.data(), 5e-5));
    }
}

} // namespace
