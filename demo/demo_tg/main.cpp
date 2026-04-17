#include <aris/plan/plan.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <vector>

namespace {

auto default_vel_type(aris::dynamic::PosType type) -> aris::dynamic::VelType {
    switch (type) {
    case aris::dynamic::PosType::PE121:
    case aris::dynamic::PosType::PE123:
    case aris::dynamic::PosType::PE131:
    case aris::dynamic::PosType::PE132:
    case aris::dynamic::PosType::PE212:
    case aris::dynamic::PosType::PE213:
    case aris::dynamic::PosType::PE231:
    case aris::dynamic::PosType::PE232:
    case aris::dynamic::PosType::PE312:
    case aris::dynamic::PosType::PE313:
    case aris::dynamic::PosType::PE321:
    case aris::dynamic::PosType::PE323:
    case aris::dynamic::PosType::PM:
    case aris::dynamic::PosType::PQ:
        return aris::dynamic::VelType::VQ;
    case aris::dynamic::PosType::RE121:
    case aris::dynamic::PosType::RE123:
    case aris::dynamic::PosType::RE131:
    case aris::dynamic::PosType::RE132:
    case aris::dynamic::PosType::RE212:
    case aris::dynamic::PosType::RE213:
    case aris::dynamic::PosType::RE231:
    case aris::dynamic::PosType::RE232:
    case aris::dynamic::PosType::RE312:
    case aris::dynamic::PosType::RE313:
    case aris::dynamic::PosType::RE321:
    case aris::dynamic::PosType::RE323:
    case aris::dynamic::PosType::RM:
    case aris::dynamic::PosType::RQ:
        return aris::dynamic::VelType::WQ;
    case aris::dynamic::PosType::XYZT:
        return aris::dynamic::VelType::DXYZT;
    case aris::dynamic::PosType::XYZ:
        return aris::dynamic::VelType::DXYZ;
    case aris::dynamic::PosType::RTZ:
        return aris::dynamic::VelType::DRTZ;
    case aris::dynamic::PosType::XYT:
        return aris::dynamic::VelType::DXYT;
    case aris::dynamic::PosType::XY:
        return aris::dynamic::VelType::DXY;
    case aris::dynamic::PosType::RT:
        return aris::dynamic::VelType::DRT;
    case aris::dynamic::PosType::X:
        return aris::dynamic::VelType::DX;
    case aris::dynamic::PosType::Y:
        return aris::dynamic::VelType::DY;
    case aris::dynamic::PosType::Z:
        return aris::dynamic::VelType::DZ;
    case aris::dynamic::PosType::A:
        return aris::dynamic::VelType::DA;
    case aris::dynamic::PosType::B:
        return aris::dynamic::VelType::DB;
    case aris::dynamic::PosType::C:
        return aris::dynamic::VelType::DC;
    default:
        return aris::dynamic::VelType::UNKNOWN;
    }
}

struct LimitMonitor {
    std::vector<aris::Size> group_dims;
    std::vector<double> prev_pos;
    std::vector<double> prev_speed_groups;
    std::vector<double> prev_acc_groups;
    bool has_prev_pos{false};
    bool has_prev_speed{false};
    bool has_prev_acc{false};
};

auto make_limit_monitor(const std::vector<aris::dynamic::PosType> &types) -> LimitMonitor {
    LimitMonitor monitor;
    aris::Size total_pos_size = 0;
    for (auto type : types) {
        total_pos_size += aris::dynamic::s_pos_type_size(type);
        switch (type) {
        case aris::dynamic::PosType::PE121:
        case aris::dynamic::PosType::PE123:
        case aris::dynamic::PosType::PE131:
        case aris::dynamic::PosType::PE132:
        case aris::dynamic::PosType::PE212:
        case aris::dynamic::PosType::PE213:
        case aris::dynamic::PosType::PE231:
        case aris::dynamic::PosType::PE232:
        case aris::dynamic::PosType::PE312:
        case aris::dynamic::PosType::PE313:
        case aris::dynamic::PosType::PE321:
        case aris::dynamic::PosType::PE323:
        case aris::dynamic::PosType::PM:
        case aris::dynamic::PosType::PQ:
            monitor.group_dims.push_back(3);
            monitor.group_dims.push_back(3);
            break;
        case aris::dynamic::PosType::RE121:
        case aris::dynamic::PosType::RE123:
        case aris::dynamic::PosType::RE131:
        case aris::dynamic::PosType::RE132:
        case aris::dynamic::PosType::RE212:
        case aris::dynamic::PosType::RE213:
        case aris::dynamic::PosType::RE231:
        case aris::dynamic::PosType::RE232:
        case aris::dynamic::PosType::RE312:
        case aris::dynamic::PosType::RE313:
        case aris::dynamic::PosType::RE321:
        case aris::dynamic::PosType::RE323:
        case aris::dynamic::PosType::RM:
        case aris::dynamic::PosType::RQ:
            monitor.group_dims.push_back(3);
            break;
        case aris::dynamic::PosType::XYZT:
            monitor.group_dims.push_back(3);
            monitor.group_dims.push_back(1);
            break;
        case aris::dynamic::PosType::XYZ:
        case aris::dynamic::PosType::RTZ:
            monitor.group_dims.push_back(3);
            break;
        case aris::dynamic::PosType::XYT:
            monitor.group_dims.push_back(2);
            monitor.group_dims.push_back(1);
            break;
        case aris::dynamic::PosType::XY:
        case aris::dynamic::PosType::RT:
            monitor.group_dims.push_back(2);
            break;
        case aris::dynamic::PosType::X:
        case aris::dynamic::PosType::Y:
        case aris::dynamic::PosType::Z:
        case aris::dynamic::PosType::A:
        case aris::dynamic::PosType::B:
        case aris::dynamic::PosType::C:
            monitor.group_dims.push_back(1);
            break;
        default:
            break;
        }
    }
    aris::Size total_dim = 0;
    for (auto dim : monitor.group_dims) total_dim += dim;
    monitor.prev_pos.assign(total_pos_size, 0.0);
    monitor.prev_speed_groups.assign(total_dim, 0.0);
    monitor.prev_acc_groups.assign(total_dim, 0.0);
    return monitor;
}

auto default_output_vel_size(const std::vector<aris::dynamic::PosType> &types) -> aris::Size {
    aris::Size size = 0;
    for (auto type : types) {
        size += aris::dynamic::s_vel_type_size(default_vel_type(type));
    }
    return size;
}

auto extract_limit_vectors(
    const std::vector<aris::dynamic::PosType> &types,
    const double *pos,
    const double *vel,
    double dt,
    const LimitMonitor &monitor,
    std::vector<double> &speed_groups,
    std::vector<double> &speed_mags) -> void {
    speed_groups.clear();
    speed_mags.clear();

    aris::Size pos_offset = 0;
    aris::Size vel_offset = 0;
    for (auto type : types) {
        auto pos_size = aris::dynamic::s_pos_type_size(type);
        auto vel_size = aris::dynamic::s_vel_type_size(default_vel_type(type));

        switch (type) {
        case aris::dynamic::PosType::PM:
        case aris::dynamic::PosType::PQ:
        case aris::dynamic::PosType::RE121:
        case aris::dynamic::PosType::RE123:
        case aris::dynamic::PosType::RE131:
        case aris::dynamic::PosType::RE132:
        case aris::dynamic::PosType::RE212:
        case aris::dynamic::PosType::RE213:
        case aris::dynamic::PosType::RE231:
        case aris::dynamic::PosType::RE232:
        case aris::dynamic::PosType::RE312:
        case aris::dynamic::PosType::RE313:
        case aris::dynamic::PosType::RE321:
        case aris::dynamic::PosType::RE323:
        case aris::dynamic::PosType::RM:
        case aris::dynamic::PosType::RQ: {
            double vs[6]{};
            aris::dynamic::s_vel2vs(type, pos + pos_offset, default_vel_type(type), vel + vel_offset, vs);
            if (aris::dynamic::s_pos_type_mov_dim(type) > 0) {
                speed_groups.insert(speed_groups.end(), vs, vs + 3);
                speed_mags.push_back(aris::dynamic::s_norm(3, vs));
            }
            if (aris::dynamic::s_pos_type_rot_dim(type) > 0) {
                speed_groups.insert(speed_groups.end(), vs + 3, vs + 6);
                speed_mags.push_back(aris::dynamic::s_norm(3, vs + 3));
            }
            break;
        }
        case aris::dynamic::PosType::XYZT: {
            double xyz[3]{vel[vel_offset + 0], vel[vel_offset + 1], vel[vel_offset + 2]};
            double t[1]{vel[vel_offset + 3]};
            speed_groups.insert(speed_groups.end(), xyz, xyz + 3);
            speed_mags.push_back(aris::dynamic::s_norm(3, xyz));
            speed_groups.insert(speed_groups.end(), t, t + 1);
            speed_mags.push_back(std::abs(t[0]));
            break;
        }
        case aris::dynamic::PosType::X:
        case aris::dynamic::PosType::Y:
        case aris::dynamic::PosType::Z:
        case aris::dynamic::PosType::A:
        case aris::dynamic::PosType::B:
        case aris::dynamic::PosType::C: {
            double v1[1]{vel[vel_offset]};
            speed_groups.insert(speed_groups.end(), v1, v1 + 1);
            speed_mags.push_back(std::abs(v1[0]));
            break;
        }
        default:
            break;
        }

        pos_offset += pos_size;
        vel_offset += vel_size;
    }
}

auto monitor_one_step(
    const std::vector<aris::dynamic::PosType> &types,
    const double *pos,
    const double *vel,
    double dt,
    LimitMonitor &monitor,
    const double *vel_limits,
    const double *acc_limits,
    const double *jerk_limits,
    std::vector<double> &speed_mags,
    std::vector<double> &acc_mags,
    std::vector<double> &jerk_mags,
    double &max_acc_exceed,
    double &max_acc_actual,
    double &max_acc_limit,
    int &max_acc_group,
    double &max_jerk_exceed,
    double &max_jerk_actual,
    double &max_jerk_limit,
    int &max_jerk_group,
    double tol = 1e-3) -> void {
    std::vector<double> speed_groups;
    extract_limit_vectors(types, pos, vel, dt, monitor, speed_groups, speed_mags);

    std::vector<double> acc_groups(speed_groups.size(), 0.0);
    acc_mags.assign(speed_mags.size(), 0.0);
    jerk_mags.assign(speed_mags.size(), 0.0);

    aris::Size group_offset = 0;
    for (aris::Size i = 0; i < speed_mags.size(); ++i) {
        auto group_dim = monitor.group_dims[i];

        (void)vel_limits;
        if (monitor.has_prev_speed) {
            double acc_sq = 0.0;
            for (aris::Size j = 0; j < group_dim; ++j) {
                auto acc_comp = (speed_groups[group_offset + j] - monitor.prev_speed_groups[group_offset + j]) / dt;
                acc_groups[group_offset + j] = acc_comp;
                acc_sq += acc_comp * acc_comp;
            }
            acc_mags[i] = std::sqrt(acc_sq);
            auto bound = acc_limits[i] + tol;
            auto exceed = acc_mags[i] - bound;
            if (exceed > max_acc_exceed) {
                max_acc_exceed = exceed;
                max_acc_actual = acc_mags[i];
                max_acc_limit = bound;
                max_acc_group = static_cast<int>(i);
            }
        }

        if (monitor.has_prev_acc) {
            double jerk_sq = 0.0;
            for (aris::Size j = 0; j < group_dim; ++j) {
                auto jerk_comp = (acc_groups[group_offset + j] - monitor.prev_acc_groups[group_offset + j]) / dt;
                jerk_sq += jerk_comp * jerk_comp;
            }
            jerk_mags[i] = std::sqrt(jerk_sq);
            auto bound = jerk_limits[i] + tol;
            auto exceed = jerk_mags[i] - bound;
            if (exceed > max_jerk_exceed) {
                max_jerk_exceed = exceed;
                max_jerk_actual = jerk_mags[i];
                max_jerk_limit = bound;
                max_jerk_group = static_cast<int>(i);
            }
        }

        group_offset += group_dim;
    }

    monitor.prev_speed_groups = speed_groups;
    monitor.prev_acc_groups = acc_groups;
    std::copy(pos, pos + monitor.prev_pos.size(), monitor.prev_pos.begin());
    monitor.has_prev_pos = true;
    monitor.has_prev_speed = true;
    monitor.has_prev_acc = true;
}

} // namespace

int main() {
    aris::plan::TrajectoryGenerator tg;
    const std::vector<aris::dynamic::PosType> types{
        aris::dynamic::PosType::X,
        aris::dynamic::PosType::XYZT,
        aris::dynamic::PosType::PQ,
    };
    tg.setPosTypes(types);
    tg.setDt(0.001);

    constexpr int total_size = 1 + 4 + 7;
    constexpr int vel_size = 1 + 2 + 2;

    double pq0[7]{};
    double pq1[7]{};
    double pq2[7]{};
    double pq_mid[7]{};
    double pe0[6]{0.45, 0.00, 0.75, aris::PI / 2.0, 0.0, aris::PI / 2.0};
    double pe1[6]{0.46, 0.02, 0.73, aris::PI / 2.0, 0.1, aris::PI / 2.0};
    double pe2[6]{0.44, -0.03, 0.74, aris::PI / 2.0, -0.1, aris::PI / 2.0};
    double pe_mid[6]{0.45, -0.01, 0.735, aris::PI / 2.0, 0.0, aris::PI / 2.0};
    aris::dynamic::s_pe2pq(pe0, pq0, "321");
    aris::dynamic::s_pe2pq(pe1, pq1, "321");
    aris::dynamic::s_pe2pq(pe2, pq2, "321");
    aris::dynamic::s_pe2pq(pe_mid, pq_mid, "321");

    double p0[total_size]{0.0, 0.10, 0.20, 0.30, 0.00};
    double p1[total_size]{0.2, 0.15, 0.25, 0.35, 0.10};
    double p2[total_size]{-0.1, 0.12, 0.18, 0.33, -0.15};
    double mid[total_size]{0.05, 0.13, 0.21, 0.34, -0.02};
    std::copy_n(pq0, 7, p0 + 5);
    std::copy_n(pq1, 7, p1 + 5);
    std::copy_n(pq2, 7, p2 + 5);
    std::copy_n(pq_mid, 7, mid + 5);

    double vel[vel_size]{0.2, 0.2, 0.6, 0.3, 0.8};
    double acc[vel_size]{1.0, 2.0, 4.0, 2.0, 5.0};
    double jerk[vel_size]{10.0, 10.0, 10.0, 10.0, 10.0};
    double zone[vel_size]{0.0, 0.001, 0.001, 0.001, 0.001};

    tg.insertLinePos(1, p0, vel, acc, jerk, zone);
    std::vector<double> out(total_size, 0.0);
    std::vector<double> vel_out(default_output_vel_size(types), 0.0);
    (void)tg.getEePosAndMoveDt(out.data(), vel_out.data(), nullptr);

    tg.insertLinePos(2, p1, vel, acc, jerk, zone);
    tg.insertCirclePos(3, p2, mid, vel, acc, jerk, zone);

    tg.updateInsertPos();

    LimitMonitor monitor = make_limit_monitor(types);
    std::vector<double> speed_mags, acc_mags, jerk_mags;

    double max_acc_exceed = -1.0;
    double max_acc_actual = 0.0;
    double max_acc_limit = 0.0;
    int max_acc_group = -1;
    double max_jerk_exceed = -1.0;
    double max_jerk_actual = 0.0;
    double max_jerk_limit = 0.0;
    int max_jerk_group = -1;

    const char *csv_file = "/Users/panyang/Documents/MATLAB/test/demo_tg_mixed_debug.csv";
    std::ofstream ofs(csv_file);
    ofs << std::fixed << std::setprecision(12);
    ofs << "step,time_s,ret,node_id";
    for (int i = 0; i < total_size; ++i) ofs << ",p" << i;
    for (aris::Size i = 0; i < vel_out.size(); ++i) ofs << ",v" << i;
    for (aris::Size i = 0; i < monitor.group_dims.size(); ++i) {
        ofs << ",g" << i << "_v,g" << i << "_a,g" << i << "_j";
    }
    ofs << "\n";

    int step = 0;
    for (; step < 200000; ++step) {
        auto ret = tg.getEePosAndMoveDt(out.data(), vel_out.data(), nullptr);
        monitor_one_step(types, out.data(), vel_out.data(), tg.dt(), monitor,
            vel, acc, jerk,
            speed_mags, acc_mags, jerk_mags,
            max_acc_exceed, max_acc_actual, max_acc_limit, max_acc_group,
            max_jerk_exceed, max_jerk_actual, max_jerk_limit, max_jerk_group);

        ofs << step << ',' << (step * tg.dt()) << ',' << ret << ',' << tg.currentNodeId();
        for (double v : out) ofs << ',' << v;
        for (double v : vel_out) ofs << ',' << v;
        for (aris::Size i = 0; i < monitor.group_dims.size(); ++i) {
            ofs << ',' << speed_mags[i] << ',' << acc_mags[i] << ',' << jerk_mags[i];
        }
        ofs << '\n';

        if (ret == 0) break;
    }

    std::cout << "mixed finished at step=" << step << "\n";
    std::cout << "max_acc_exceed=" << max_acc_exceed
              << " (actual=" << max_acc_actual << ", limit=" << max_acc_limit << ", group=" << max_acc_group << ")\n";
    std::cout << "max_jerk_exceed=" << max_jerk_exceed
              << " (actual=" << max_jerk_actual << ", limit=" << max_jerk_limit << ", group=" << max_jerk_group << ")\n";
    std::cout << "csv=" << csv_file << "\n";

    return 0;
}
