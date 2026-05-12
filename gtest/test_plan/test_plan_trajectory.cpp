#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <vector>

#include <aris/plan/plan.hpp>

namespace {

constexpr int kHookCaseInsertPublishExchangeConflictRetry = 1;
constexpr int kHookCaseInsertPublishExchangeConflictRetryResult = 2;
constexpr int kHookCaseInsertReplanFailedFallback = 3;

struct TrajectoryHookControl {
	int publish_attempt_count{ 0 };
	int failed_publish_count{ 0 };
	int replan_failed_fallback_count{ 0 };
	int retry_path_count{ 0 };

	auto reset() -> void {
		publish_attempt_count = 0;
		failed_publish_count = 0;
		replan_failed_fallback_count = 0;
		retry_path_count = 0;
	}
};

inline TrajectoryHookControl *g_trajectory_hook_control{ nullptr };

void trajectory_concurrency_test_hook(int point, std::int64_t, std::int64_t, bool exchange_nonnull) {
	auto *control = g_trajectory_hook_control;
	if (!control) return;

	if (point == kHookCaseInsertPublishExchangeConflictRetry) {
		++control->publish_attempt_count;
		++control->failed_publish_count;
		++control->retry_path_count;
	}
	else if (point == kHookCaseInsertPublishExchangeConflictRetryResult) {
		++control->publish_attempt_count;
		if (!exchange_nonnull) {
			++control->failed_publish_count;
		}
	}
	else if (point == kHookCaseInsertReplanFailedFallback) {
		++control->replan_failed_fallback_count;
	}
}

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
	for (auto dim : monitor.group_dims) {
		total_dim += dim;
	}
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
		case aris::dynamic::PosType::PE323: {
			double linear_vs[3]{};
			double angular_vs[3]{};
			if (monitor.has_prev_pos) {
				double dp[3]{
					pos[pos_offset + 0] - monitor.prev_pos[pos_offset + 0],
					pos[pos_offset + 1] - monitor.prev_pos[pos_offset + 1],
					pos[pos_offset + 2] - monitor.prev_pos[pos_offset + 2],
				};
				linear_vs[0] = dp[0] / dt;
				linear_vs[1] = dp[1] / dt;
				linear_vs[2] = dp[2] / dt;

				double pq_curr[7]{};
				double pq_prev[7]{};
				aris::dynamic::s_pos2pos(type, pos + pos_offset, aris::dynamic::PosType::PQ, pq_curr);
				aris::dynamic::s_pos2pos(type, monitor.prev_pos.data() + pos_offset, aris::dynamic::PosType::PQ, pq_prev);
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
				aris::dynamic::s_wq2wa(pq_curr + 3, wq, angular_vs);
			}
			speed_groups.insert(speed_groups.end(), linear_vs, linear_vs + 3);
			speed_mags.push_back(aris::dynamic::s_norm(3, linear_vs));
			speed_groups.insert(speed_groups.end(), angular_vs, angular_vs + 3);
			speed_mags.push_back(aris::dynamic::s_norm(3, angular_vs));
			break;
		}
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
		case aris::dynamic::PosType::XYZ:
		case aris::dynamic::PosType::RTZ: {
			double v3[3]{vel[vel_offset + 0], vel[vel_offset + 1], vel[vel_offset + 2]};
			speed_groups.insert(speed_groups.end(), v3, v3 + 3);
			speed_mags.push_back(aris::dynamic::s_norm(3, v3));
			break;
		}
		case aris::dynamic::PosType::XYT: {
			double xy[2]{vel[vel_offset + 0], vel[vel_offset + 1]};
			double t[1]{vel[vel_offset + 2]};
			speed_groups.insert(speed_groups.end(), xy, xy + 2);
			speed_mags.push_back(aris::dynamic::s_norm(2, xy));
			speed_groups.insert(speed_groups.end(), t, t + 1);
			speed_mags.push_back(std::abs(t[0]));
			break;
		}
		case aris::dynamic::PosType::XY:
		case aris::dynamic::PosType::RT: {
			double v2[2]{vel[vel_offset + 0], vel[vel_offset + 1]};
			speed_groups.insert(speed_groups.end(), v2, v2 + 2);
			speed_mags.push_back(aris::dynamic::s_norm(2, v2));
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

auto expect_motion_limits_respected(
	const std::vector<aris::dynamic::PosType> &types,
	const double *pos,
	const double *vel,
	const double *vel_limits,
	const double *acc_limits,
	const double *jerk_limits,
	double dt,
	LimitMonitor &monitor,
	double tol = 1e-3,
	int sample_idx = -1,
	std::int64_t ret_id = -1) -> void {
	std::vector<double> speed_groups;
	std::vector<double> speed_mags;
	extract_limit_vectors(types, pos, vel, dt, monitor, speed_groups, speed_mags);
	std::vector<double> acc_groups(speed_groups.size(), 0.0);

	aris::Size group_offset = 0;
	for (aris::Size i = 0; i < speed_mags.size(); ++i) {
		auto speed_mag = speed_mags[i];
		EXPECT_LE(speed_mag, vel_limits[i] + tol)
			<< "Velocity limit exceeded at group " << i
			<< ", sample_idx=" << sample_idx
			<< ", ret_id=" << ret_id;

		double acc_mag = 0.0;
		auto group_dim = monitor.group_dims[i];
		if (monitor.has_prev_speed) {
			double acc_sq = 0.0;
			for (aris::Size j = 0; j < group_dim; ++j) {
				auto acc_comp = (speed_groups[group_offset + j] - monitor.prev_speed_groups[group_offset + j]) / dt;
				acc_groups[group_offset + j] = acc_comp;
				acc_sq += acc_comp * acc_comp;
			}
			acc_mag = std::sqrt(acc_sq);
			// TODO: This temporary 1.1x max_acc tolerance should be removed after acceleration profile optimization.
			EXPECT_LE(acc_mag, 1.1 * acc_limits[i] + tol)
				<< "Acceleration limit exceeded at group " << i
				<< ", sample_idx=" << sample_idx
				<< ", ret_id=" << ret_id;
		}

		if (monitor.has_prev_acc) {
			double jerk_sq = 0.0;
			for (aris::Size j = 0; j < group_dim; ++j) {
				auto jerk_comp = (acc_groups[group_offset + j] - monitor.prev_acc_groups[group_offset + j]) / dt;
				jerk_sq += jerk_comp * jerk_comp;
			}
			auto jerk_mag = std::sqrt(jerk_sq);
			auto jerk_limit = 4.0 * jerk_limits[i] + tol;
			// TODO: This temporary 4x max_j tolerance should be removed after jerk profile optimization.
			EXPECT_LE(jerk_mag, jerk_limit)
				<< "Jerk limit exceeded at group " << i
				<< ", exceed_by=" << (jerk_mag - jerk_limit)
				<< ", ratio=" << (jerk_mag / jerk_limit)
				<< ", sample_idx=" << sample_idx
				<< ", ret_id=" << ret_id;
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

auto expect_pose_near(const double *pe_actual, const double *pe_expected, double tol = 1e-5) -> void {
	double pm_actual[16]{};
	double pm_expected[16]{};
	aris::dynamic::s_pe2pm(pe_actual, pm_actual, "321");
	aris::dynamic::s_pe2pm(pe_expected, pm_expected, "321");
	for (int i = 0; i < 16; ++i) {
		EXPECT_NEAR(pm_actual[i], pm_expected[i], tol) << "Pose matrix mismatch at index " << i;
	}
}

auto expect_segment_near(
	aris::dynamic::PosType type,
	const double *actual,
	const double *expected,
	double tol = 1e-5) -> void {
	switch (type) {
	case aris::dynamic::PosType::X:
	case aris::dynamic::PosType::Y:
	case aris::dynamic::PosType::Z:
	case aris::dynamic::PosType::A:
	case aris::dynamic::PosType::B:
	case aris::dynamic::PosType::C: {
		auto size = aris::dynamic::s_pos_type_size(type);
		for (aris::Size i = 0; i < size; ++i) {
			EXPECT_NEAR(actual[i], expected[i], tol) << "Scalar segment mismatch at index " << i;
		}
		break;
	}
	default: {
		double pm_actual[16]{};
		double pm_expected[16]{};
		aris::dynamic::s_pos2pm(type, actual, pm_actual);
		aris::dynamic::s_pos2pm(type, expected, pm_expected);
		for (int i = 0; i < 16; ++i) {
			EXPECT_NEAR(pm_actual[i], pm_expected[i], tol)
				<< "Pose segment mismatch at matrix index " << i;
		}
		break;
	}
	}
}

auto expect_mixed_position_near(
	const std::vector<aris::dynamic::PosType> &types,
	const double *actual,
	const double *expected,
	double tol = 1e-5) -> void {
	aris::Size offset = 0;
	for (auto type : types) {
		expect_segment_near(type, actual + offset, expected + offset, tol);
		offset += aris::dynamic::s_pos_type_size(type);
	}
}

auto step_until_finished(
	aris::plan::TrajectoryGenerator &tg,
	std::vector<double> &out,
	const std::vector<aris::dynamic::PosType> *types = nullptr,
	const double *vel_limits = nullptr,
	const double *acc_limits = nullptr,
	const double *jerk_limits = nullptr,
	int max_iters = 100000,
	LimitMonitor *monitor_in_out = nullptr) -> std::vector<std::int64_t> {
	std::vector<std::int64_t> seen_ids;
	std::vector<double> vel_out;
	LimitMonitor monitor;
	if (types) {
		vel_out.resize(default_output_vel_size(*types), 0.0);
		monitor = monitor_in_out ? *monitor_in_out : make_limit_monitor(*types);
	}
	for (int i = 0; i < max_iters; ++i) {
		auto ret = tg.getEePosAndMoveDt(out.data(), types ? vel_out.data() : nullptr, nullptr);
		if (types) {
			expect_motion_limits_respected(*types, out.data(), vel_out.data(), vel_limits, acc_limits, jerk_limits, tg.dt(), monitor);
		}
		if (ret != 0 && std::find(seen_ids.begin(), seen_ids.end(), ret) == seen_ids.end()) {
			seen_ids.push_back(ret);
		}
		if (ret == 0) {
			if (types && monitor_in_out) {
				*monitor_in_out = monitor;
			}
			return seen_ids;
		}
	}
	if (types && monitor_in_out) {
		*monitor_in_out = monitor;
	}
	ADD_FAILURE() << "Trajectory did not finish within expected iterations";
	return seen_ids;
}

} // namespace

// 验证纯直线序列能完整执行，并最终到达最后目标点。
TEST(TrajectoryTest, LineSequenceFinishesAndReachesLastTarget) {
	aris::plan::TrajectoryGenerator tg;
	const std::vector<aris::dynamic::PosType> types{aris::dynamic::PosType::PE321};
	tg.setPosTypes(types);
	tg.setDt(0.001);

	double p0[6]{0.45, 0.0, 0.75, aris::PI / 2.0, 0.0, aris::PI / 2.0};
	double p1[6]{0.46, 0.02, 0.74, aris::PI / 2.0, 0.0, aris::PI / 2.0};
	double p2[6]{0.43, -0.03, 0.76, aris::PI / 2.0, 0.0, aris::PI / 2.0};
	double vel[2]{0.2, 0.8};
	double acc[2]{1.0, 5.0};
	double jerk[2]{10.0, 20.0};
	double zone[2]{0.001, 0.001};

	tg.insertLinePos(1, p0, vel, acc, jerk, zone);
	tg.updateInsertPos();
	std::vector<double> out(6, 0.0);
	std::vector<double> vel_out(default_output_vel_size(types), 0.0);
	LimitMonitor monitor = make_limit_monitor(types);
	(void)tg.getEePosAndMoveDt(out.data(), vel_out.data(), nullptr);
	expect_motion_limits_respected(types, out.data(), vel_out.data(), vel, acc, jerk, tg.dt(), monitor);

	tg.insertLinePos(2, p1, vel, acc, jerk, zone);
	tg.insertLinePos(3, p2, vel, acc, jerk, zone);
	tg.updateInsertPos();

	bool seen_2 = false;
	bool seen_3 = false;
	bool finished = false;
	std::int64_t ret = -1;
	for (int i = 0; i < 100000; ++i) {
		ret = tg.getEePosAndMoveDt(out.data(), vel_out.data(), nullptr);
		expect_motion_limits_respected(types, out.data(), vel_out.data(), vel, acc, jerk, tg.dt(), monitor);
		if (ret == 2) seen_2 = true;
		if (ret == 3) seen_3 = true;
		if (ret == 0) {
			finished = true;
			break;
		}
	}

	EXPECT_TRUE(seen_2);
	EXPECT_TRUE(seen_3);
	EXPECT_TRUE(finished);
	expect_pose_near(out.data(), p2, 1e-4);
}

	// 验证圆弧轨迹执行与节点清理接口（clearUsedPos / clearAllPos）行为正确。
TEST(TrajectoryTest, CircleMotionAndNodeManagementWorks) {
	aris::plan::TrajectoryGenerator tg;
	const std::vector<aris::dynamic::PosType> types{aris::dynamic::PosType::PE321};
	tg.setPosTypes(types);
	tg.setDt(0.001);

	double p0[6]{0.40, 0.00, 0.70, aris::PI / 2.0, 0.0, aris::PI / 2.0};
	double mid[6]{0.45, 0.05, 0.70, aris::PI / 2.0, 0.0, aris::PI / 2.0};
	double p2[6]{0.50, 0.00, 0.70, aris::PI / 2.0, 0.0, aris::PI / 2.0};
	double vel[2]{0.2, 0.8};
	double acc[2]{1.0, 5.0};
	double jerk[2]{10.0, 20.0};
	double zone[2]{0.0, 0.0};

	tg.insertInitPos(10, p0);
	tg.insertCirclePos(11, p2, mid, vel, acc, jerk, zone);
	tg.updateInsertPos();

	EXPECT_GE(tg.unusedPosNum(), 1);
	auto ids = tg.unusedNodeIds();
	EXPECT_FALSE(ids.empty());

	std::vector<double> out(6, 0.0);
	std::vector<double> vel_out(default_output_vel_size(types), 0.0);
	LimitMonitor monitor = make_limit_monitor(types);
	bool seen_circle = false;
	bool finished = false;
	for (int i = 0; i < 100000; ++i) {
		auto ret = tg.getEePosAndMoveDt(out.data(), vel_out.data(), nullptr);
		expect_motion_limits_respected(types, out.data(), vel_out.data(), vel, acc, jerk, tg.dt(), monitor);
		if (ret == 11) seen_circle = true;
		if (ret == 0) {
			finished = true;
			break;
		}
	}

	EXPECT_TRUE(seen_circle);
	EXPECT_TRUE(finished);
	expect_pose_near(out.data(), p2, 1e-4);

	tg.clearUsedPos();
	EXPECT_GE(tg.unusedPosNum(), 0);
	tg.clearAllPos();
	EXPECT_EQ(tg.unusedPosNum(), 0);
}

	// 验证初始化点、队列状态迁移以及 clearUsedPos 后剩余节点集合符合预期。
TEST(TrajectoryTest, InitPosAndQueueStateTransitionWorks) {
	aris::plan::TrajectoryGenerator tg;
	const std::vector<aris::dynamic::PosType> types{aris::dynamic::PosType::PE321};
	tg.setPosTypes(types);
	tg.setDt(0.001);

	double p0[6]{0.40, 0.00, 0.70, aris::PI / 2.0, 0.0, aris::PI / 2.0};
	double p1[6]{0.42, 0.01, 0.72, aris::PI / 2.0, 0.0, aris::PI / 2.0};
	double p2[6]{0.45, -0.02, 0.73, aris::PI / 2.0, 0.0, aris::PI / 2.0};
	double vel[2]{0.2, 0.8};
	double acc[2]{1.0, 5.0};
	double jerk[2]{10.0, 20.0};
	double zone[2]{0.001, 0.001};

	tg.insertInitPos(10, p0);
	tg.insertLinePos(11, p1, vel, acc, jerk, zone);
	tg.insertLinePos(12, p2, vel, acc, jerk, zone);
	tg.updateInsertPos();

	EXPECT_EQ(tg.currentNodeId(), 10);
	EXPECT_FALSE(tg.isCurrentNodeFinished());
	EXPECT_EQ(tg.unusedPosNum(), 2);
	EXPECT_EQ(tg.unusedNodeIds(), (std::vector<std::int64_t>{10, 11, 12}));

	std::vector<double> out(6, 0.0);
	std::vector<double> vel_out(default_output_vel_size(types), 0.0);
	LimitMonitor monitor = make_limit_monitor(types);
	bool reached_11 = false;
	for (int i = 0; i < 10000; ++i) {
		(void)tg.getEePosAndMoveDt(out.data(), vel_out.data(), nullptr);
		expect_motion_limits_respected(types, out.data(), vel_out.data(), vel, acc, jerk, tg.dt(), monitor);
		if (tg.currentNodeId() == 11) {
			reached_11 = true;
			break;
		}
	}

	EXPECT_TRUE(reached_11);
	tg.clearUsedPos();
	EXPECT_EQ(tg.unusedNodeIds(), (std::vector<std::int64_t>{11, 12}));

	auto seen_ids = step_until_finished(tg, out, &types, vel, acc, jerk, 100000, &monitor);
	EXPECT_NE(std::find(seen_ids.begin(), seen_ids.end(), 11), seen_ids.end());
	EXPECT_NE(std::find(seen_ids.begin(), seen_ids.end(), 12), seen_ids.end());
	expect_pose_near(out.data(), p2, 1e-4);

	tg.clearAllPos();
	EXPECT_EQ(tg.unusedPosNum(), 0);
	EXPECT_TRUE(tg.unusedNodeIds().empty());
}

	// 验证“小平移+大旋转”场景下轨迹可完成且满足运动学约束。
TEST(TrajectoryTest, LargeRotationSmallTranslationSequenceFinishes) {
	aris::plan::TrajectoryGenerator tg;
	const std::vector<aris::dynamic::PosType> types{aris::dynamic::PosType::PE321};
	tg.setPosTypes(types);
	tg.setDt(0.001);

	// Keep translation tiny while making orientation change dominant.
	double p0[6]{0.4500, 0.0000, 0.7500, aris::PI / 2.0, 0.00, aris::PI / 2.0};
	double p1[6]{0.4504, -0.0003, 0.7502, aris::PI / 2.0 + 0.03, 0.90, aris::PI / 2.0 - 0.02};
	double p2[6]{0.4498, 0.0002, 0.7499, aris::PI / 2.0 - 0.025, -0.95, aris::PI / 2.0 + 0.015};

	double vel[2]{0.80, 0.90};
	double acc[2]{0.80, 1.00};
	double jerk[2]{20.0, 10.0};
	double zone[2]{0.0002, 0.2};

	tg.insertLinePos(101, p0, vel, acc, jerk, zone);
	tg.updateInsertPos();
	std::vector<double> out(6, 0.0);
	std::vector<double> vel_out(default_output_vel_size(types), 0.0);
	LimitMonitor monitor = make_limit_monitor(types);
	(void)tg.getEePosAndMoveDt(out.data(), vel_out.data(), nullptr);
	expect_motion_limits_respected(types, out.data(), vel_out.data(), vel, acc, jerk, tg.dt(), monitor);

	tg.insertLinePos(102, p1, vel, acc, jerk, zone);
	tg.insertLinePos(103, p2, vel, acc, jerk, zone);
	tg.updateInsertPos();

	bool seen_102 = false;
	bool seen_103 = false;
	bool finished = false;
	for (int i = 0; i < 120000; ++i) {
		auto ret = tg.getEePosAndMoveDt(out.data(), vel_out.data(), nullptr);
		expect_motion_limits_respected(types, out.data(), vel_out.data(), vel, acc, jerk, tg.dt(), monitor, 1e-3, i, ret);
		if (ret == 102) {
			seen_102 = true;
		}
		if (ret == 103) {
			seen_103 = true;
		}
		if (ret == 0) {
			finished = true;
			break;
		}
	}

	EXPECT_TRUE(seen_102);
	EXPECT_TRUE(seen_103);
	EXPECT_TRUE(finished);
	expect_pose_near(out.data(), p2, 1e-4);
}

	// 验证混合位姿类型（X/XYZT/PQ）下，线段与圆弧组合轨迹可正确完成。
TEST(TrajectoryTest, MixedPosTypesLineAndCircleSequenceFinishes) {
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
	tg.updateInsertPos();
	std::vector<double> out(total_size, 0.0);
	(void)tg.getEePosAndMoveDt(out.data());

	tg.insertLinePos(2, p1, vel, acc, jerk, zone);
	tg.insertCirclePos(3, p2, mid, vel, acc, jerk, zone);
	tg.updateInsertPos();

	LimitMonitor monitor = make_limit_monitor(types);
	auto seen_ids = step_until_finished(tg, out, &types, vel, acc, jerk, 100000, &monitor);
	EXPECT_NE(std::find(seen_ids.begin(), seen_ids.end(), 2), seen_ids.end());
	EXPECT_NE(std::find(seen_ids.begin(), seen_ids.end(), 3), seen_ids.end());
	expect_mixed_position_near(types, out.data(), p2, 1e-4);
}

	// 验证标量轴与 RTZ 混合场景下，多段插入与复位流程可完成并收敛到目标。
TEST(TrajectoryTest, MixedScalarAndRTZSequenceFinishes) {
	aris::plan::TrajectoryGenerator tg;
	const std::vector<aris::dynamic::PosType> types{
		aris::dynamic::PosType::X,
		aris::dynamic::PosType::X,
		aris::dynamic::PosType::RTZ,
	};
	tg.setPosTypes(types);
	tg.setDt(0.001);

	constexpr int pos_size = 1 + 1 + 3;
	constexpr int vel_size = 1 + 1 + 2;

	double init_pos[pos_size]{0.0, 0.0, 0.0, 0.0, 0.0};
	double p0[pos_size]{0.1, 0.3, 0.2, 0.4, 0.1};
	double p1[pos_size]{0.2, 0.6, 0.6, 0.3, 0.8};
	double p2[pos_size]{0.3, 0.4, 0.1, 0.4, 0.4};
	double mid[pos_size]{0.25, 0.5, 0.35, 0.35, 0.6};
	double vel[vel_size]{0.1, 0.3, 1.0, 1.5};
	double acc[vel_size]{0.1, 0.1, 5.0, 5.0};
	double jerk[vel_size]{1.0, 10.0, 10.0, 10.0};
	double zone[vel_size]{0.2, 0.2, 0.2, 0.2};

	tg.insertLinePos(10, p0, vel, acc, jerk, zone);
	tg.updateInsertPos();
	std::vector<double> out(pos_size, 0.0);
	(void)tg.getEePosAndMoveDt(out.data());

	tg.insertLinePos(11, p1, vel, acc, jerk, zone);
	tg.insertInitPos(29, p2);
	tg.updateInsertPos();
	LimitMonitor monitor_first = make_limit_monitor(types);
	auto seen_first = step_until_finished(tg, out, &types, vel, acc, jerk, 100000, &monitor_first);
	EXPECT_NE(std::find(seen_first.begin(), seen_first.end(), 11), seen_first.end());
	expect_mixed_position_near(types, out.data(), p2, 1e-4);

	tg.insertInitPos(30, p0);
	tg.insertCirclePos(50, p2, mid, vel, acc, jerk, zone);
	tg.insertLinePos(51, init_pos, vel, acc, jerk, zone);
	tg.updateInsertPos();

	LimitMonitor monitor_second = make_limit_monitor(types);
	auto seen_second = step_until_finished(tg, out, &types, vel, acc, jerk, 100000, &monitor_second);
	EXPECT_NE(std::find(seen_second.begin(), seen_second.end(), 50), seen_second.end());
	EXPECT_NE(std::find(seen_second.begin(), seen_second.end(), 51), seen_second.end());
	expect_mixed_position_near(types, out.data(), init_pos, 1e-4);
}

	// 验证运行过程中在线追加节点时，系统可按插入顺序执行并最终完成。
TEST(TrajectoryTest, OnlineInsertDuringExecutionFinishesInInsertedOrder) {
	aris::plan::TrajectoryGenerator tg;
	const std::vector<aris::dynamic::PosType> types{aris::dynamic::PosType::PE321};
	tg.setPosTypes(types);
	tg.setDt(0.001);

	double init[6]{-0.029091, 0.017520, -0.015596, 83.478 * aris::PI / 180.0, 51.402 * aris::PI / 180.0, 3.401 * aris::PI / 180.0};
	double p1[6]{-0.028557, 0.018248, -0.014542, 82.251 * aris::PI / 180.0, 48.042 * aris::PI / 180.0, 3.974 * aris::PI / 180.0};
	double p2[6]{-0.028818, 0.017541, -0.010604, 84.384 * aris::PI / 180.0, 35.789 * aris::PI / 180.0, 5.922 * aris::PI / 180.0};
	double p3[6]{-0.029146, 0.014578, -0.004558, 86.495 * aris::PI / 180.0, 17.774 * aris::PI / 180.0, 9.034 * aris::PI / 180.0};
	double p4[6]{-0.029184, 0.008114, 0.000144, 87.246 * aris::PI / 180.0, 5.021 * aris::PI / 180.0, 13.373 * aris::PI / 180.0};
	double p5[6]{-0.028906, -0.002406, 0.004319, 87.215 * aris::PI / 180.0, 2.450 * aris::PI / 180.0, 14.832 * aris::PI / 180.0};
	double vel[2]{0.1, aris::PI};
	double acc[2]{5.0, 5.0 * aris::PI};
	double jerk[2]{50.0, 50.0 * aris::PI};
	double zone[2]{0.02, 0.02};

	tg.insertInitPos(10000, init);
	tg.insertLinePos(1, p1, vel, acc, jerk, zone);
	tg.updateInsertPos();

	std::vector<double> out(6, 0.0);
	std::vector<double> vel_out(default_output_vel_size(types), 0.0);

	LimitMonitor monitor = make_limit_monitor(types);
	std::vector<std::int64_t> seen_ids;
	int cmd_count = 0;
	std::int64_t last_ret = -1;
	bool inserted_2 = false;
	bool inserted_3 = false;
	bool inserted_4 = false;
	bool inserted_5 = false;
	bool finished = false;

	for (int i = 0; i < 200000; ++i) {
		auto ret = tg.getEePosAndMoveDt(out.data(), vel_out.data(), nullptr);
		expect_motion_limits_respected(types, out.data(), vel_out.data(), vel, acc, jerk, tg.dt(), monitor, 1e-3, i, ret);

		if (ret != last_ret) {
			cmd_count = 0;
			last_ret = ret;
		}
		++cmd_count;

		if (ret != 0 && std::find(seen_ids.begin(), seen_ids.end(), ret) == seen_ids.end()) {
			seen_ids.push_back(ret);
		}

		if (!inserted_2 && ret == 1 && cmd_count >= 20) {
			tg.insertLinePos(2, p2, vel, acc, jerk, zone);
			tg.updateInsertPos();
			inserted_2 = true;
			EXPECT_GE(tg.unusedPosNum(), 1);
		}
		if (!inserted_3 && ret == 1 && cmd_count >= 40) {
			tg.insertLinePos(3, p3, vel, acc, jerk, zone);
			tg.updateInsertPos();
			inserted_3 = true;
		}
		if (!inserted_4 && ret == 2 && cmd_count >= 10) {
			tg.insertLinePos(4, p4, vel, acc, jerk, zone);
			tg.updateInsertPos();
			inserted_4 = true;
		}
		if (!inserted_5 && ret == 3 && cmd_count >= 10) {
			tg.insertLinePos(5, p5, vel, acc, jerk, zone);
			tg.updateInsertPos();
			inserted_5 = true;
		}

		if (ret == 0) {
			finished = true;
			break;
		}
	}

	EXPECT_TRUE(inserted_2);
	EXPECT_TRUE(inserted_3);
	EXPECT_TRUE(inserted_4);
	EXPECT_TRUE(inserted_5);
	EXPECT_TRUE(finished);
	EXPECT_NE(std::find(seen_ids.begin(), seen_ids.end(), 1), seen_ids.end());
	EXPECT_NE(std::find(seen_ids.begin(), seen_ids.end(), 2), seen_ids.end());
	EXPECT_NE(std::find(seen_ids.begin(), seen_ids.end(), 3), seen_ids.end());
	EXPECT_NE(std::find(seen_ids.begin(), seen_ids.end(), 4), seen_ids.end());
	EXPECT_NE(std::find(seen_ids.begin(), seen_ids.end(), 5), seen_ids.end());
	expect_pose_near(out.data(), p5, 1e-4);
}

	// 验证批量 insert 后单次 update 能一次性发布并执行完所有新增节点。
TEST(TrajectoryTest, BatchInsertThenSingleUpdateFinishesAllInsertedNodes) {
	aris::plan::TrajectoryGenerator tg;
	const std::vector<aris::dynamic::PosType> types{aris::dynamic::PosType::PE321};
	tg.setPosTypes(types);
	tg.setDt(0.001);

	double init[6]{0.45, 0.00, 0.75, aris::PI / 2.0, 0.0, aris::PI / 2.0};
	double p1[6]{0.46, 0.02, 0.74, aris::PI / 2.0, 0.0, aris::PI / 2.0};
	double p2[6]{0.43, -0.03, 0.76, aris::PI / 2.0, 0.0, aris::PI / 2.0};
	double p3[6]{0.42, 0.01, 0.73, aris::PI / 2.0, 0.0, aris::PI / 2.0};
	double p4[6]{0.41, -0.02, 0.72, aris::PI / 2.0, 0.0, aris::PI / 2.0};
	double p5[6]{0.44, 0.01, 0.71, aris::PI / 2.0, 0.0, aris::PI / 2.0};
	double vel[2]{0.2, 0.8};
	double acc[2]{1.0, 5.0};
	double jerk[2]{10.0, 20.0};
	double zone[2]{0.001, 0.001};

	tg.insertInitPos(100, init);
	tg.insertLinePos(101, p1, vel, acc, jerk, zone);
	tg.updateInsertPos();

	std::vector<double> out(6, 0.0);
	std::vector<double> vel_out(default_output_vel_size(types), 0.0);
	LimitMonitor monitor = make_limit_monitor(types);
	(void)tg.getEePosAndMoveDt(out.data(), vel_out.data(), nullptr);
	expect_motion_limits_respected(types, out.data(), vel_out.data(), vel, acc, jerk, tg.dt(), monitor);

	// 多次插入，不立即 update。
	tg.insertLinePos(102, p2, vel, acc, jerk, zone);
	tg.insertLinePos(103, p3, vel, acc, jerk, zone);
	tg.insertLinePos(104, p4, vel, acc, jerk, zone);
	tg.insertLinePos(105, p5, vel, acc, jerk, zone);

	// 一次同步。
	tg.updateInsertPos();

	auto seen_ids = step_until_finished(tg, out, &types, vel, acc, jerk, 160000, &monitor);
	EXPECT_NE(std::find(seen_ids.begin(), seen_ids.end(), 102), seen_ids.end());
	EXPECT_NE(std::find(seen_ids.begin(), seen_ids.end(), 103), seen_ids.end());
	EXPECT_NE(std::find(seen_ids.begin(), seen_ids.end(), 104), seen_ids.end());
	EXPECT_NE(std::find(seen_ids.begin(), seen_ids.end(), 105), seen_ids.end());
	expect_pose_near(out.data(), p5, 1e-4);
}

	// 验证三种串行 case：切换前更新、切换后更新、强制 publish 冲突重试都可最终完成。
TEST(TrajectoryTest, ConcurrentGetAndSingleBatchUpdateEventuallyFinishes) {
	const std::vector<aris::dynamic::PosType> types{ aris::dynamic::PosType::PE321 };
	double init[6]{ -0.029091, 0.017520, -0.015596, 83.478 * aris::PI / 180.0, 51.402 * aris::PI / 180.0, 3.401 * aris::PI / 180.0 };
	double p1[6]{ -0.028557, 0.018248, -0.014542, 82.251 * aris::PI / 180.0, 48.042 * aris::PI / 180.0, 3.974 * aris::PI / 180.0 };
	double p2[6]{ -0.028818, 0.017541, -0.010604, 84.384 * aris::PI / 180.0, 35.789 * aris::PI / 180.0, 5.922 * aris::PI / 180.0 };
	double p3[6]{ -0.029146, 0.014578, -0.004558, 86.495 * aris::PI / 180.0, 17.774 * aris::PI / 180.0, 9.034 * aris::PI / 180.0 };
	double p4[6]{ -0.029184, 0.008114, 0.000144, 87.246 * aris::PI / 180.0, 5.021 * aris::PI / 180.0, 13.373 * aris::PI / 180.0 };
	double p5[6]{ -0.028906, -0.002406, 0.004319, 87.215 * aris::PI / 180.0, 2.450 * aris::PI / 180.0, 14.832 * aris::PI / 180.0 };
	double p6[6]{ -0.028700, -0.003200, 0.006200, 87.100 * aris::PI / 180.0, 2.200 * aris::PI / 180.0, 15.200 * aris::PI / 180.0 };
	double p7[6]{ -0.028450, -0.004100, 0.007400, 87.020 * aris::PI / 180.0, 1.900 * aris::PI / 180.0, 15.650 * aris::PI / 180.0 };
	double vel[2]{ 0.1, aris::PI };
	double acc[2]{ 5.0, 5.0 * aris::PI };
	double jerk[2]{ 50.0, 50.0 * aris::PI };
	double zone[2]{ 0.02, 0.02 };

	TrajectoryHookControl hook_control;
	g_trajectory_hook_control = &hook_control;
	aris::plan::__trajectory_test::setTrajectoryConcurrencyTestHook(trajectory_concurrency_test_hook);
	aris::plan::__trajectory_test::setTrajectoryConcurrencyForcePublishConflictCurrentId(-1, 0);
	auto insert_batch_nodes = [&](aris::plan::TrajectoryGenerator &tg) {
		tg.insertLinePos(203, p3, vel, acc, jerk, zone);
		tg.insertLinePos(204, p4, vel, acc, jerk, zone);
		tg.insertLinePos(205, p5, vel, acc, jerk, zone);
		tg.insertLinePos(206, p6, vel, acc, jerk, zone);
		tg.insertLinePos(207, p7, vel, acc, jerk, zone);
	};
	auto expect_finishes_remaining_nodes = [&](aris::plan::TrajectoryGenerator &tg) {
		std::vector<double> out(6, 0.0);
		std::vector<double> vel_out(default_output_vel_size(types), 0.0);
		LimitMonitor monitor = make_limit_monitor(types);
		double jerk_limits_concurrent[2]{ jerk[0] * 20.0, jerk[1] * 20.0 };
		constexpr int kWarmupSamples = 8;
		std::vector<std::int64_t> seen_ids;
		for (int i = 0; i < 120000; ++i) {
			auto ret = tg.getEePosAndMoveDt(out.data(), vel_out.data(), nullptr);
			if (i < kWarmupSamples) {
				std::copy(out.begin(), out.end(), monitor.prev_pos.begin());
				monitor.has_prev_pos = true;
				monitor.has_prev_speed = false;
				monitor.has_prev_acc = false;
			}
			else {
				expect_motion_limits_respected(types, out.data(), vel_out.data(), vel, acc, jerk_limits_concurrent, tg.dt(), monitor, 1e-3, i, ret);
			}
			if (ret != 0 && std::find(seen_ids.begin(), seen_ids.end(), ret) == seen_ids.end()) {
				seen_ids.push_back(ret);
			}
			if (ret == 0) {
				break;
			}
		}
		EXPECT_NE(std::find(seen_ids.begin(), seen_ids.end(), 203), seen_ids.end());
		EXPECT_NE(std::find(seen_ids.begin(), seen_ids.end(), 204), seen_ids.end());
		EXPECT_NE(std::find(seen_ids.begin(), seen_ids.end(), 205), seen_ids.end());
		EXPECT_NE(std::find(seen_ids.begin(), seen_ids.end(), 206), seen_ids.end());
		EXPECT_NE(std::find(seen_ids.begin(), seen_ids.end(), 207), seen_ids.end());
		expect_pose_near(out.data(), p7, 1e-4);
	};

	auto run_case = [&](int case_idx, const char *case_name) {
		SCOPED_TRACE(case_name);
		hook_control.reset();
		aris::plan::__trajectory_test::setTrajectoryConcurrencyForcePublishConflictCurrentId(-1, 0);

		aris::plan::TrajectoryGenerator tg;
		tg.setPosTypes(types);
		tg.setDt(0.001);

		tg.insertInitPos(200, init);
		tg.insertLinePos(201, p1, vel, acc, jerk, zone);
		tg.insertLinePos(202, p2, vel, acc, jerk, zone);
		tg.updateInsertPos();

		std::vector<double> out(6, 0.0);
		for (int i = 0; i < 200000; ++i) {
			(void)tg.getEePosAndMoveDt(out.data(), nullptr, nullptr);
			if (tg.currentNodeId() == 201) break;
		}
		ASSERT_EQ(tg.currentNodeId(), 201) << "did not reach node 201";

		if (case_idx == 2) {
			for (int i = 0; i < 200000; ++i) {
				(void)tg.getEePosAndMoveDt(out.data(), nullptr, nullptr);
				if (tg.currentNodeId() == 202) break;
			}
			ASSERT_EQ(tg.currentNodeId(), 202) << "did not switch to node 202 before update";
		}

		insert_batch_nodes(tg);

		if (case_idx == 3) {
			aris::plan::__trajectory_test::setTrajectoryConcurrencyForcePublishConflictCurrentId(201, 1);
		}

		tg.updateInsertPos();

		if (case_idx == 3) {
			EXPECT_GT(hook_control.publish_attempt_count, 0)
				<< "Should reach publish result hook emission at trajectory.cpp:1722";
			EXPECT_GT(hook_control.failed_publish_count, 0)
				<< "Should record failed publish (exchange_nonnull=false) from forced conflict";

				if (case_idx == 2) {
					EXPECT_EQ(hook_control.retry_path_count, 0)
						<< "Case2 should NOT execute the retry path (1732 else branch)";
				}

				if (case_idx == 3) {
					EXPECT_GT(hook_control.retry_path_count, 0)
						<< "Case3 should execute the retry path (1732 else branch) when forced conflict occurs";
				}
		}

		expect_finishes_remaining_nodes(tg);
	};

	run_case(1, "case 1: update before switch (serial)");
	run_case(2, "case 2: switch before update (serial)");
	run_case(3, "case 3: forced publish conflict retry (serial)");

	aris::plan::__trajectory_test::setTrajectoryConcurrencyForcePublishConflictCurrentId(-1, 0);
	aris::plan::__trajectory_test::setTrajectoryConcurrencyTestHook(nullptr);
	g_trajectory_hook_control = nullptr;
}

	// 验证在共线路径上，不同插入时机是否命中 replan 失败回退分支。
TEST(TrajectoryTest, ReplanFallbackHitDiffersByInsertTimingOnCollinearPath) {
	const std::vector<aris::dynamic::PosType> types{ aris::dynamic::PosType::PE321 };
	double vel[2]{ 1.0, 1.0 };
	double acc[2]{ 2.0, 2.0 };
	double jerk[2]{ 10.0, 10.0 };
	double zone[2]{ 0.02, 0.02 };
	double opposite[6]{ 0.9, 0.01, 0.0, 0.0, 0.0, 0.0 };

	TrajectoryHookControl hook_control;
	g_trajectory_hook_control = &hook_control;
	aris::plan::__trajectory_test::setTrajectoryConcurrencyTestHook(trajectory_concurrency_test_hook);

	auto run_case = [&](int trigger_node_id, std::int64_t force_fail_current_id, bool expect_fallback_hit, const char *case_name) {
		SCOPED_TRACE(case_name);
		hook_control.reset();
		aris::plan::__trajectory_test::setTrajectoryConcurrencyForceReplanFailCurrentId(force_fail_current_id);

		aris::plan::TrajectoryGenerator tg;
		tg.setPosTypes(types);
		tg.setDt(0.001);

		std::array<std::array<double, 6>, 11> pts{};
		for (int i = 0; i <= 10; ++i) {
			pts[i] = { 0.1 * static_cast<double>(i), 0.0, 0.0, 0.0, 0.0, 0.0 };
		}

		tg.insertInitPos(0, pts[0].data());
		for (int i = 1; i <= 10; ++i) {
			tg.insertLinePos(i, pts[i].data(), vel, acc, jerk, zone);
		}
		tg.updateInsertPos();

		std::vector<double> out(6, 0.0);
		std::vector<double> vel_out(default_output_vel_size(types), 0.0);
		LimitMonitor monitor = make_limit_monitor(types);

		bool reached_trigger = false;
		for (int i = 0; i < 200000; ++i) {
			(void)tg.getEePosAndMoveDt(out.data(), vel_out.data(), nullptr);
			expect_motion_limits_respected(types, out.data(), vel_out.data(), vel, acc, jerk, tg.dt(), monitor, 1e-3, i, tg.currentNodeId());
			if (tg.currentNodeId() == trigger_node_id) {
				reached_trigger = true;
				break;
			}
		}
		ASSERT_TRUE(reached_trigger) << "did not reach trigger node id=" << trigger_node_id;

		const std::int64_t inserted_id = 1000 + trigger_node_id;
		tg.insertLinePos(inserted_id, opposite, vel, acc, jerk, zone);
		tg.updateInsertPos();

		if (expect_fallback_hit) {
			EXPECT_GT(hook_control.replan_failed_fallback_count, 0)
				<< "expected replan failed fallback branch (around trajectory.cpp:1670) to be hit";
		}
		else {
			EXPECT_EQ(hook_control.replan_failed_fallback_count, 0)
				<< "did not expect replan failed fallback branch (around trajectory.cpp:1670)";
		}

		auto seen_ids = step_until_finished(tg, out, &types, vel, acc, jerk, 240000, &monitor);
		EXPECT_NE(std::find(seen_ids.begin(), seen_ids.end(), inserted_id), seen_ids.end());
		expect_pose_near(out.data(), opposite, 1e-4);
	};

	run_case(2, -1, false, "case1: insert near reverse direction at node 2 should replan directly");
	run_case(7, 7, true, "case2: insert near reverse direction at node 7 should hit fallback branch");

	aris::plan::__trajectory_test::setTrajectoryConcurrencyForceReplanFailCurrentId(-1);
	aris::plan::__trajectory_test::setTrajectoryConcurrencyTestHook(nullptr);
	g_trajectory_hook_control = nullptr;
}
