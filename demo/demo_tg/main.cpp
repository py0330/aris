#include <aris/plan/plan.hpp>
#include "backup/trajectory_generator_backup.hpp"

#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>

namespace {

struct DebugMonitor {
    double prev_pe[6]{};
    double prev_v_lin[3]{};
    double prev_v_ang[3]{};
    double prev_a_lin[3]{};
    double prev_a_ang[3]{};
    bool has_prev_pos{false};
    bool has_prev_speed{false};
    bool has_prev_acc{false};
};

auto norm3(const double v[3]) -> double {
    return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

void calc_speed_from_pe321(const double *pe, double dt, DebugMonitor &m, double v_lin[3], double v_ang[3]) {
    v_lin[0] = 0.0;
    v_lin[1] = 0.0;
    v_lin[2] = 0.0;
    v_ang[0] = 0.0;
    v_ang[1] = 0.0;
    v_ang[2] = 0.0;

    if (!m.has_prev_pos) {
        return;
    }

    double dp[3]{
        pe[0] - m.prev_pe[0],
        pe[1] - m.prev_pe[1],
        pe[2] - m.prev_pe[2],
    };
    v_lin[0] = dp[0] / dt;
    v_lin[1] = dp[1] / dt;
    v_lin[2] = dp[2] / dt;

    double pq_curr[7]{};
    double pq_prev[7]{};
    aris::dynamic::s_pos2pos(aris::dynamic::PosType::PE321, pe, aris::dynamic::PosType::PQ, pq_curr);
    aris::dynamic::s_pos2pos(aris::dynamic::PosType::PE321, m.prev_pe, aris::dynamic::PosType::PQ, pq_prev);

    double dot = 0.0;
    for (int i = 3; i < 7; ++i) dot += pq_curr[i] * pq_prev[i];
    if (dot < 0.0) {
        for (int i = 3; i < 7; ++i) pq_curr[i] = -pq_curr[i];
    }

    double wq[4]{
        (pq_curr[3] - pq_prev[3]) / dt,
        (pq_curr[4] - pq_prev[4]) / dt,
        (pq_curr[5] - pq_prev[5]) / dt,
        (pq_curr[6] - pq_prev[6]) / dt,
    };
    aris::dynamic::s_wq2wa(pq_curr + 3, wq, v_ang);
}

} // namespace

int main() {
    aris::plan::TrajectoryGeneratorBackup tg;
    tg.setPosTypes({aris::dynamic::PosType::PE321});
    tg.setDt(0.001);

    double p0[6]{0.45, 0.0, 0.75, aris::PI / 2.0, 0.0, aris::PI / 2.0};
    double p1[6]{0.46, 0.02, 0.74, aris::PI / 2.0, 0.0, aris::PI / 2.0};
    double p2[6]{0.43, -0.03, 0.76, aris::PI / 2.0, 0.0, aris::PI / 2.0};
    double vel_limits[2]{0.2, 0.8};
    double acc_limits[2]{1.0, 5.0};
    double jerk_limits[2]{10.0, 20.0};
    double zone[2]{0.001, 0.001};
    constexpr double tol = 1e-3;

    tg.insertLinePos(1, p0, vel_limits, acc_limits, jerk_limits, zone);

    const char *out_file = "/Users/panyang/Documents/MATLAB/test/demo_tg_line_debug.csv";

    std::ofstream ofs(out_file);
    ofs << std::fixed << std::setprecision(12);
    ofs
        << "step,time_s,ret,s,arc,left_node_s,left_total_s,"
        << "pe_x,pe_y,pe_z,pe_a,pe_b,pe_c,"
        << "pq_x,pq_y,pq_z,pq_qw,pq_qx,pq_qy,pq_qz,"
        << "v_lin_norm,v_ang_norm,a_lin_norm,a_ang_norm,j_lin_norm,j_ang_norm,"
        << "v_lin_over,v_ang_over,a_lin_over,a_ang_over,j_lin_over,j_ang_over\n";

    DebugMonitor monitor;
    double pe[6]{};

    // First sampling follows test behavior.
    auto ret = tg.getEePosAndMoveDt(pe, nullptr, nullptr);

    tg.insertLinePos(2, p1, vel_limits, acc_limits, jerk_limits, zone);
    tg.insertLinePos(3, p2, vel_limits, acc_limits, jerk_limits, zone);

    int step = 0;
    double max_a_lin = 0.0;
    double max_a_ang = 0.0;
    double max_j_lin = 0.0;
    double max_j_ang = 0.0;
    double max_arc = 0.0;

    for (; step < 100000; ++step) {
        if (step > 0) {
            ret = tg.getEePosAndMoveDt(pe, nullptr, nullptr);
        }

        double pq[7]{};
        aris::dynamic::s_pos2pos(aris::dynamic::PosType::PE321, pe, aris::dynamic::PosType::PQ, pq);

        double v_lin[3]{};
        double v_ang[3]{};
        calc_speed_from_pe321(pe, tg.dt(), monitor, v_lin, v_ang);

        double a_lin[3]{};
        double a_ang[3]{};
        double j_lin[3]{};
        double j_ang[3]{};

        if (monitor.has_prev_speed) {
            for (int i = 0; i < 3; ++i) {
                a_lin[i] = (v_lin[i] - monitor.prev_v_lin[i]) / tg.dt();
                a_ang[i] = (v_ang[i] - monitor.prev_v_ang[i]) / tg.dt();
            }
        }

        if (monitor.has_prev_acc) {
            for (int i = 0; i < 3; ++i) {
                j_lin[i] = (a_lin[i] - monitor.prev_a_lin[i]) / tg.dt();
                j_ang[i] = (a_ang[i] - monitor.prev_a_ang[i]) / tg.dt();
            }
        }

        double v_lin_norm = norm3(v_lin);
        double v_ang_norm = norm3(v_ang);
        double a_lin_norm = norm3(a_lin);
        double a_ang_norm = norm3(a_ang);
        double j_lin_norm = norm3(j_lin);
        double j_ang_norm = norm3(j_ang);

        max_a_lin = std::max(max_a_lin, a_lin_norm);
        max_a_ang = std::max(max_a_ang, a_ang_norm);
        max_j_lin = std::max(max_j_lin, j_lin_norm);
        max_j_ang = std::max(max_j_ang, j_ang_norm);

        int v_lin_over = (v_lin_norm > vel_limits[0] + tol) ? 1 : 0;
        int v_ang_over = (v_ang_norm > vel_limits[1] + tol) ? 1 : 0;
        int a_lin_over = (a_lin_norm > acc_limits[0] + tol) ? 1 : 0;
        int a_ang_over = (a_ang_norm > acc_limits[1] + tol) ? 1 : 0;
        int j_lin_over = (j_lin_norm > jerk_limits[0] + tol) ? 1 : 0;
        int j_ang_over = (j_ang_norm > jerk_limits[1] + tol) ? 1 : 0;

        double current_s = tg.currentS();
        double current_arc = tg.currentArc();
        double left_node_s = tg.leftNodeS();
        double left_total_s = tg.leftTotalS();

        max_arc = std::max(max_arc, current_arc);

        ofs
            << step << ',' << (step * tg.dt()) << ',' << ret << ',' << current_s << ',' << current_arc << ',' << left_node_s << ',' << left_total_s << ','
            << pe[0] << ',' << pe[1] << ',' << pe[2] << ',' << pe[3] << ',' << pe[4] << ',' << pe[5] << ','
            << pq[0] << ',' << pq[1] << ',' << pq[2] << ',' << pq[3] << ',' << pq[4] << ',' << pq[5] << ',' << pq[6] << ','
            << v_lin_norm << ',' << v_ang_norm << ',' << a_lin_norm << ',' << a_ang_norm << ',' << j_lin_norm << ',' << j_ang_norm << ','
            << v_lin_over << ',' << v_ang_over << ',' << a_lin_over << ',' << a_ang_over << ',' << j_lin_over << ',' << j_ang_over << '\n';

        for (int i = 0; i < 6; ++i) monitor.prev_pe[i] = pe[i];
        for (int i = 0; i < 3; ++i) {
            monitor.prev_v_lin[i] = v_lin[i];
            monitor.prev_v_ang[i] = v_ang[i];
            monitor.prev_a_lin[i] = a_lin[i];
            monitor.prev_a_ang[i] = a_ang[i];
        }
        monitor.has_prev_pos = true;
        monitor.has_prev_speed = true;
        monitor.has_prev_acc = true;

        if (ret == 0) {
            break;
        }
    }

    std::cout << "demo_tg finished at step=" << step << "\n";
    std::cout << "max_a_lin=" << max_a_lin << ", max_a_ang=" << max_a_ang << "\n";
    std::cout << "max_j_lin=" << max_j_lin << ", max_j_ang=" << max_j_ang << "\n";
    std::cout << "max_arc=" << max_arc << "\n";
    std::cout << "csv=" << out_file << "\n";
    return 0;
}
