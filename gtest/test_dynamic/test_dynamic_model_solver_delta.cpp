#include <gtest/gtest.h>

#include <cstdio>
#include <cmath>
#include <vector>

#include <aris/dynamic/dynamic.hpp>
#include <aris/dynamic/mechanism_delta.hpp>

namespace {

struct MotorScaleCase {
    double factor;
    double offset;
};

auto createDeltaModel() -> std::unique_ptr<aris::dynamic::Model> {
    aris::dynamic::DeltaParam param;
    param.a = 0.5;
    param.b = 0.2;
    param.c = 0.1;
    param.d = 0.7;
    param.e = 0.1;
    return aris::dynamic::createModelDelta(param);
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

auto deltaOutputPosEqual(const std::vector<double> &lhs, const std::vector<double> &rhs, double tol) -> bool {
    if (lhs.size() != rhs.size()) return false;
    if (lhs.size() < 4) return aris::dynamic::s_is_equal(static_cast<int>(lhs.size()), 1, lhs.data(), rhs.data(), tol);

    if (!aris::dynamic::s_is_equal(3, 1, lhs.data(), rhs.data(), tol)) return false;
    const double d = std::atan2(std::sin(lhs[3] - rhs[3]), std::cos(lhs[3] - rhs[3]));
    return std::abs(d) < tol;
}

class ModelSolverDeltaParam : public ::testing::TestWithParam<MotorScaleCase> {};

auto case_name(const ::testing::TestParamInfo<MotorScaleCase> &info) -> std::string {
    char buf[64];
    std::snprintf(buf, sizeof(buf), "F%.2f_O%.2f", info.param.factor, info.param.offset);
    std::string s(buf);
    for (char &c : s) {
        if (c == '-') c = 'N';
        if (c == '.') c = 'p';
    }
    return s;
}

TEST_P(ModelSolverDeltaParam, DISABLED_PositionRoundTrip) {
    auto m = createDeltaModel();
    const auto p = GetParam();
    applyMotorFactorOffset(*m, p.factor, p.offset);

    const int in_size = static_cast<int>(m->inputPosSize());
    const int out_size = static_cast<int>(m->outputPosSize());
    const auto root_num = m->inverseRootNumber();
    ASSERT_GT(root_num, 0);
    std::vector<int> root_covered(static_cast<size_t>(root_num), 0);

    const double phy_samples[][4]{
        {-0.10, 0.10, -0.45, 0.30},
        {-0.08, 0.05, -0.40, 0.20},
        {-0.12, 0.08, -0.42, 0.25},
    };

    for (const auto &phy_q : phy_samples) {
        std::vector<double> cmd_q(in_size, 0.0), out_pos(out_size, 0.0);
        physicalToMotorCmd(phy_q, cmd_q.data(), in_size, p.factor, p.offset);

        EXPECT_TRUE(m->forwardKinematics(cmd_q.data(), out_pos.data(), nullptr, cmd_q.data()) == 0);

        for (std::int64_t root = 0; root < root_num; ++root) {
            std::vector<double> cmd_q_back(in_size, 0.0), out_pos_recheck(out_size, 0.0);
            if (m->inverseKinematics(out_pos.data(), cmd_q_back.data(), &root, cmd_q.data()) != 0) {
                continue;
            }

            root_covered[static_cast<size_t>(root)] = 1;
            EXPECT_TRUE(m->forwardKinematics(cmd_q_back.data(), out_pos_recheck.data(), nullptr, cmd_q_back.data()) == 0);
            EXPECT_TRUE(deltaOutputPosEqual(out_pos, out_pos_recheck, 1e-7));
        }
    }

    int covered_count = 0;
    for (auto c : root_covered) covered_count += c;
    EXPECT_EQ(covered_count, static_cast<int>(root_num));
}

TEST_P(ModelSolverDeltaParam, VelAccForceRoundTrip) {
    auto m = createDeltaModel();
    const auto p = GetParam();
    applyMotorFactorOffset(*m, p.factor, p.offset);

    const int n = static_cast<int>(m->inputPosSize());
    std::vector<double> phy_q(n, 0.0), cmd_q(n, 0.0), cmd_v(n, 0.0), cmd_a(n, 0.0);
    if (n >= 4) {
        phy_q = {-0.10, 0.10, -0.45, 0.30};
        cmd_v = {0.05, -0.06, 0.07, -0.04};
        cmd_a = {-0.08, 0.09, -0.07, 0.06};
    }
    physicalToMotorCmd(phy_q.data(), cmd_q.data(), n, p.factor, p.offset);

    std::vector<double> out_pos(m->outputPosSize()), out_vel(m->outputVelSize()), out_acc(m->outputAccSize()), tau(m->inputFceSize());
    m->setInputPos(cmd_q.data());
    m->setInputVel(cmd_v.data());
    m->setInputAcc(cmd_a.data());
    EXPECT_TRUE(m->forwardKinematics() == 0);
    EXPECT_TRUE(m->forwardKinematicsVel() == 0);
    EXPECT_TRUE(m->forwardKinematicsAcc() == 0);
    EXPECT_TRUE(m->inverseDynamics() == 0);
    m->getOutputPos(out_pos.data());
    m->getOutputVel(out_vel.data());
    m->getOutputAcc(out_acc.data());
    m->getInputFce(tau.data());

    std::int64_t which_root{0};
    EXPECT_TRUE(m->getWhichInverseRoot(out_pos.data(), cmd_q.data(), &which_root) == 0);
    m->setWhichInverseRoot(&which_root);

    std::vector<double> cmd_zero(n, 0.0);
    m->setInputPos(cmd_zero.data());
    m->setInputVel(cmd_zero.data());
    m->setInputAcc(cmd_zero.data());
    m->setOutputPos(out_pos.data());
    m->setOutputVel(out_vel.data());
    m->setOutputAcc(out_acc.data());
    EXPECT_TRUE(m->inverseKinematics() == 0);
    EXPECT_TRUE(m->inverseKinematicsVel() == 0);
    EXPECT_TRUE(m->inverseKinematicsAcc() == 0);

    std::vector<double> cmd_q_back(n, 0.0), cmd_v_back(n, 0.0), cmd_a_back(n, 0.0);
    m->getInputPos(cmd_q_back.data());
    m->getInputVel(cmd_v_back.data());
    m->getInputAcc(cmd_a_back.data());

    for (int i = 0; i < n; ++i) {
        EXPECT_TRUE(std::isfinite(cmd_q_back[i]));
        EXPECT_TRUE(std::isfinite(cmd_v_back[i]));
        EXPECT_TRUE(std::isfinite(cmd_a_back[i]));
    }
}

TEST_P(ModelSolverDeltaParam, JacobianConsistency) {
    auto m = createDeltaModel();
    const auto p = GetParam();
    applyMotorFactorOffset(*m, p.factor, p.offset);

    const int n = static_cast<int>(m->inputPosSize());
    const int m_out = static_cast<int>(m->outputVelSize());
    std::vector<double> phy_q(n, 0.0), cmd_q(n, 0.0);
    if (n >= 4) phy_q = {-0.10, 0.10, -0.45, 0.30};
    physicalToMotorCmd(phy_q.data(), cmd_q.data(), n, p.factor, p.offset);

    m->setInputPos(cmd_q.data());
    EXPECT_TRUE(m->forwardKinematics() == 0);

    std::vector<double> out_pos(m->outputPosSize());
    m->getOutputPos(out_pos.data());

    auto &inv = dynamic_cast<aris::dynamic::InverseKinematicSolver&>(m->solverPool().at(0));
    auto &fwd = dynamic_cast<aris::dynamic::ForwardKinematicSolver&>(m->solverPool().at(1));
    inv.cptJacobi();
    fwd.cptJacobi();

    for (int i = 0; i < m_out * n; ++i) {
        EXPECT_TRUE(std::isfinite(fwd.Jf()[i]));
    }
    for (int i = 0; i < n * m_out; ++i) {
        EXPECT_TRUE(std::isfinite(inv.Ji()[i]));
    }
}

TEST_P(ModelSolverDeltaParam, DynamicMatrixConsistency) {
    auto m = createDeltaModel();
    const auto p = GetParam();
    applyMotorFactorOffset(*m, p.factor, p.offset);

    const int n = static_cast<int>(m->inputAccSize());
    const int nf = static_cast<int>(m->inputFceSize());
    const double eps = 1e-6;

    std::vector<double> phy_q(n, 0.0), cmd_q(n, 0.0), cmd_v(n, 0.0), cmd_a_test(n, 0.0);
    if (n >= 4) {
        phy_q = {-0.10, 0.10, -0.45, 0.30};
        cmd_v = {0.05, -0.06, 0.07, -0.04};
        cmd_a_test = {-0.08, 0.09, -0.07, 0.06};
    }
    physicalToMotorCmd(phy_q.data(), cmd_q.data(), n, p.factor, p.offset);

    std::vector<double> h(nf, 0.0), a_zero(n, 0.0);
    m->setInputPos(cmd_q.data());
    m->setInputVel(cmd_v.data());
    m->setInputAcc(a_zero.data());
    EXPECT_TRUE(m->inverseDynamics() == 0);
    m->getInputFce(h.data());

    std::vector<double> M(nf * n, 0.0);
    for (int col = 0; col < n; ++col) {
        std::vector<double> a_eps(n, 0.0), tau_eps(nf, 0.0);
        a_eps[col] = eps;

        m->setInputPos(cmd_q.data());
        m->setInputVel(cmd_v.data());
        m->setInputAcc(a_eps.data());
        EXPECT_TRUE(m->inverseDynamics() == 0);
        m->getInputFce(tau_eps.data());

        for (int row = 0; row < nf; ++row) M[row * n + col] = (tau_eps[row] - h[row]) / eps;
    }

    std::vector<double> tau_model(nf, 0.0), tau_mat(nf, 0.0);
    m->setInputPos(cmd_q.data());
    m->setInputVel(cmd_v.data());
    m->setInputAcc(cmd_a_test.data());
    EXPECT_TRUE(m->inverseDynamics() == 0);
    m->getInputFce(tau_model.data());

    for (int row = 0; row < nf; ++row) {
        double sum = h[row];
        for (int col = 0; col < n; ++col) sum += M[row * n + col] * cmd_a_test[col];
        tau_mat[row] = sum;
    }
    EXPECT_TRUE(aris::dynamic::s_is_equal(nf, 1, tau_model.data(), tau_mat.data(), 5e-5));

    m->setInputPos(cmd_q.data());
    m->setInputVel(cmd_v.data());
    m->setInputFce(tau_model.data());
    EXPECT_TRUE(m->forwardDynamics() == 0);
    std::vector<double> cmd_a_fd(n, 0.0);
    m->getInputAcc(cmd_a_fd.data());
    EXPECT_TRUE(aris::dynamic::s_is_equal(n, 1, cmd_a_test.data(), cmd_a_fd.data(), 5e-5));
}

INSTANTIATE_TEST_SUITE_P(
    MotorScales,
    ModelSolverDeltaParam,
    ::testing::Values(
        MotorScaleCase{0.3, 1.0},
        MotorScaleCase{0.2, -1.5},
        MotorScaleCase{1.0, 0.0}
    ),
    case_name
);

} // namespace
