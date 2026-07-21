#include"aris/plan/trajectory.hpp"
#include"aris/plan/function.hpp"

namespace aris::plan {
#ifdef ARIS_BUILD_TESTS
	namespace {
		// Case 名字与触发点对应关系：
		// 1) CaseInsertPublishExchangeConflictRetry:
		//    在 publish 前触发，用于卡住 update 与运行线程切换窗口。
		// 2) CaseInsertPublishExchangeConflictRetryResult:
		//    在 publish 后触发，exchange_nonnull=false 表示本轮发布冲突，需要回滚并重试。
		// 3) CaseInsertReplanFailedFallback:
		//    replan_nodes 失败，进入仅追加新节点的回退路径（对应 update_insert 的失败分支）。
		constexpr int kTrajectoryHookCaseInsertPublishExchangeConflictRetry = 1;
		constexpr int kTrajectoryHookCaseInsertPublishExchangeConflictRetryResult = 2;
		constexpr int kTrajectoryHookCaseInsertReplanFailedFallback = 3;

		std::atomic<TrajectoryConcurrencyTestHook> g_trajectory_concurrency_test_hook{ nullptr };
		std::atomic<std::int64_t> g_trajectory_force_replan_fail_current_id{ -1 };
		std::atomic<std::int64_t> g_trajectory_force_publish_conflict_current_id{ -1 };
		std::atomic<int> g_trajectory_force_publish_conflict_times{ 0 };

		auto emit_trajectory_concurrency_test_hook(int point, std::int64_t current_id, std::int64_t next_id, bool exchange_nonnull)->void {
			auto hook = g_trajectory_concurrency_test_hook.load(std::memory_order_relaxed);
			if (hook) hook(point, current_id, next_id, exchange_nonnull);
		}
	}

	namespace __trajectory_test {
		auto setTrajectoryConcurrencyTestHook(TrajectoryConcurrencyTestHook hook)->void {
			g_trajectory_concurrency_test_hook.store(hook, std::memory_order_relaxed);
		}

		auto setTrajectoryConcurrencyForceReplanFailCurrentId(std::int64_t current_id)->void {
			g_trajectory_force_replan_fail_current_id.store(current_id, std::memory_order_relaxed);
		}

		auto setTrajectoryConcurrencyForcePublishConflictCurrentId(std::int64_t current_id, int conflict_times)->void {
			g_trajectory_force_publish_conflict_current_id.store(current_id, std::memory_order_relaxed);
			g_trajectory_force_publish_conflict_times.store(std::max(conflict_times, 0), std::memory_order_relaxed);
		}
	} // namespace __trajectory_test
#endif

	struct Node {
		enum class NodeType {
			ResetInitPos,
			Line,
			Circle,
		};
		enum class UnitType {
			Line3,
			Line2,
			Line1,
			Circle3,
			Circle2,
			Rotate3,
		};

		struct OriginData{
			aris::dynamic::PosType pos_type_{};
			double ee_pos_[7], mid_pos_[7], v_[2], a_[2], j_[2], zone_[2];
		};
		struct Zone {
			enum class ZoneType {
				LL, // Line Line
				LC, // Line Circle
				CL, // Circle Line
				CC, // Circle Circle
				QQ, // Quaternion Quaternion
				OO, // One dof One dof
			};
			struct Lines {
				double p0_[3], p1_[3], p2_[3];
			};
			struct LineCircle {
				double p0_[3], p1_[3], center_[3], axis_[3], theta_;
			};
			struct CircleLine {
				double p1_[3], p2_[3], center_[3], axis_[3], theta_;
			};
			struct Circles {
				double pcenter_[3], c1_[3], a1_[3], theta1_, c2_[3], a2_[3], theta2_;   // circle circle
			};
			struct Quaternions {
				double q0_[4], q1_[4], q2_[4];
			};
			struct OneDof {
				double p0_, p1_, p2_;
			};
			ZoneType type_{ ZoneType::LL };
			// length 是融合后的弧长，zone_value是用户的zone的输入。
			// 对于四元数，length是角度差，不是四元数弧长。两者相差2倍
			double length_{ 0.0 }, zone_value_{ 0.0 }; 
			EstimateBezierArcParam bezier_param;
			union {
				Lines lines_{};
				LineCircle line_circle_;
				CircleLine circle_line_;
				Circles circles_;
				Quaternions quaternions_;
				OneDof one_dof_;
			};
		};
		struct Move {
			struct LineData {
				double p0_[3], dir_[3];
			};
			struct CircleData {
				double p0_[3], center_[3], axis_[3], radius_;
			};
			struct QuternionData {
				double q0_[4], q1_[4];
			};
			struct OneDof {
				double p0_, p1_;
			};

			double length_{ 0.0 };  // 对于 quaternion 来说，是指角度
			double origin_length_{ 0.0 }; // 添加转弯区会缩短move的长度，因此需要保存原始长度，以便添加多段转弯区
			union {
				LineData line_{};
				CircleData circle_;
				QuternionData quaternion_;
				OneDof one_dof_;
			};
		};
		struct Unit {
			UnitType    type_{ UnitType::Line3 };
			Move        move_;
			Zone        zone1_, 
				        zone2_;
			SCurveParam scurve_;
		};
		struct EePlanData {
			// Origin Data
			OriginData data_;
			
			// Move Unit
			Unit x_, a_;
		};

		NodeType                type_;
		std::int64_t            id_;
		LargeNum                s_beg_;
		LargeNum                s_end_;
		std::vector<EePlanData> ee_plans_;
		std::atomic<Node*>      next_node_;
		bool                    finished_{ false }; // 这个标志位判断是否结束过 //

		~Node() = default;
		Node(aris::Size ee_size) {
			type_ = NodeType::Line;
			id_ = 1;
			s_beg_ = 0;
			s_end_ = 0;
			ee_plans_.resize(ee_size);
			next_node_.store(this);
		}
		Node(const Node& other) {
			type_ = other.type_;
			id_ = other.id_;
			s_beg_ = other.s_beg_;
			s_end_ = other.s_end_;
			ee_plans_ = other.ee_plans_;
			next_node_.store(other.next_node_.load());
		}
		Node& operator=(const Node& other) {
			id_ = other.id_;
			s_beg_ = other.s_beg_;
			s_end_ = other.s_end_;
			ee_plans_ = other.ee_plans_;
			next_node_.store(other.next_node_.load());
			return *this;
		}
	};

	// make & compute raw data // 
	auto s_make_line3(const double* p0, const double* p1, double* dir, double& length) -> void {
		length = std::sqrt(
			(p1[0] - p0[0]) * (p1[0] - p0[0]) + (p1[1] - p0[1]) * (p1[1] - p0[1]) + (p1[2] - p0[2]) * (p1[2] - p0[2])
		);

		// 计算 direction //
		aris::dynamic::s_vc(3, p1, dir);
		aris::dynamic::s_vs(3, p0, dir);
		aris::dynamic::s_nv(3, 1.0 / std::max(length, 1e-10), dir);
	}
	auto s_compute_line3_at(const double* p0, const double* dir, double l, double* xyz, double dl = 0.0, double* dxyz = nullptr, double d2l = 0.0, double* d2xyz = nullptr) -> void {
		// pos //
		xyz[0] = p0[0] + dir[0] * l;
		xyz[1] = p0[1] + dir[1] * l;
		xyz[2] = p0[2] + dir[2] * l;

		// vel //
		if (dxyz) {
			dxyz[0] = dir[0] * dl;
			dxyz[1] = dir[1] * dl;
			dxyz[2] = dir[2] * dl;
		}

		// acc //
		if (d2xyz) {
			d2xyz[0] = dir[0] * d2l;
			d2xyz[1] = dir[1] * d2l;
			d2xyz[2] = dir[2] * d2l;
		}
	}

	auto s_make_line2(const double* p0, const double* p1, double* dir, double& length) -> void {
		length = std::sqrt(
			(p1[0] - p0[0]) * (p1[0] - p0[0]) + (p1[1] - p0[1]) * (p1[1] - p0[1])
		);

		// 计算 direction //
		aris::dynamic::s_vc(2, p1, dir);
		aris::dynamic::s_vs(2, p0, dir);
		aris::dynamic::s_nv(2, 1.0 / std::max(length, 1e-10), dir);
	}
	auto s_compute_line2_at(const double* p0, const double* dir, double l, double* xyz, double dl = 0.0, double* dxyz = nullptr, double d2l = 0.0, double* d2xyz = nullptr) -> void {
		// pos //
		xyz[0] = p0[0] + dir[0] * l;
		xyz[1] = p0[1] + dir[1] * l;

		// vel //
		if (dxyz) {
			dxyz[0] = dir[0] * dl;
			dxyz[1] = dir[1] * dl;
		}

		// acc //
		if (d2xyz) {
			d2xyz[0] = dir[0] * d2l;
			d2xyz[1] = dir[1] * d2l;
		}
	}

	auto s_make_circle3(const double* p0, const double* p1, const double* p2, double* center, double* axis, double& radius, double& length) -> void
	{
		// see https://www.jianshu.com/p/f99246170561
		double
			x0 = p0[0],
			y0 = p0[1],
			z0 = p0[2],
			x1 = p1[0],
			y1 = p1[1],
			z1 = p1[2],
			x2 = p2[0],
			y2 = p2[1],
			z2 = p2[2];

		double a1 = (y0 * z1 - y1 * z0 - y0 * z2 + y2 * z0 + y1 * z2 - y2 * z1),
			b1 = -(x0 * z1 - x1 * z0 - x0 * z2 + x2 * z0 + x1 * z2 - x2 * z1),
			c1 = (x0 * y1 - x1 * y0 - x0 * y2 + x2 * y0 + x1 * y2 - x2 * y1),
			d1 = -(x0 * y1 * z2 - x0 * y2 * z1 - x1 * y0 * z2 + x1 * y2 * z0 + x2 * y0 * z1 - x2 * y1 * z0);

		double a2 = 2 * (x1 - x0),
			b2 = 2 * (y1 - y0),
			c2 = 2 * (z1 - z0),
			d2 = x0 * x0 + y0 * y0 + z0 * z0 - x1 * x1 - y1 * y1 - z1 * z1;

		double a3 = 2 * (x2 - x0),
			b3 = 2 * (y2 - y0),
			c3 = 2 * (z2 - z0),
			d3 = x0 * x0 + y0 * y0 + z0 * z0 - x2 * x2 - y2 * y2 - z2 * z2;

		double div = (a1 * b2 * c3 - a1 * b3 * c2 - a2 * b1 * c3 + a2 * b3 * c1 + a3 * b1 * c2 - a3 * b2 * c1);

		if (std::abs(div) < 1e-14) {
			double p2_minus_p0[3]{
				p2[0] - p0[0],
				p2[1] - p0[1],
				p2[2] - p0[2],
			};

			radius = std::numeric_limits<double>::infinity();
			length = aris::dynamic::s_norm(3, p2_minus_p0);
		}
		else {
			center[0] = -(b1 * c2 * d3 - b1 * c3 * d2 - b2 * c1 * d3 + b2 * c3 * d1 + b3 * c1 * d2 - b3 * c2 * d1)
				/ div;
			center[1] = (a1 * c2 * d3 - a1 * c3 * d2 - a2 * c1 * d3 + a2 * c3 * d1 + a3 * c1 * d2 - a3 * c2 * d1)
				/ div;
			center[2] = -(a1 * b2 * d3 - a1 * b3 * d2 - a2 * b1 * d3 + a2 * b3 * d1 + a3 * b1 * d2 - a3 * b2 * d1)
				/ div;

			double p1_minus_p0[3]{
				p1[0] - p0[0],
				p1[1] - p0[1],
				p1[2] - p0[2],
			};
			double p2_minus_p1[3]{
				p2[0] - p1[0],
				p2[1] - p1[1],
				p2[2] - p1[2],
			};
			aris::dynamic::s_c3(p1_minus_p0, p2_minus_p1, axis);
			aris::dynamic::s_nv(3, 1.0 / aris::dynamic::s_norm(3, axis), axis);

			radius = std::sqrt((center[0] - p0[0]) * (center[0] - p0[0])
				+ (center[1] - p0[1]) * (center[1] - p0[1])
				+ (center[2] - p0[2]) * (center[2] - p0[2]));

			double p0_minus_center[3]{
				(p0[0] - center[0]) / radius,
				(p0[1] - center[1]) / radius,
				(p0[2] - center[2]) / radius,
			};
			double p2_minus_center[3]{
				(p2[0] - center[0]) / radius,
				(p2[1] - center[1]) / radius,
				(p2[2] - center[2]) / radius,
			};

			double cross_result[3];
			aris::dynamic::s_c3(p0_minus_center, p2_minus_center, cross_result);

			double s = aris::dynamic::s_norm(3, cross_result);
			double c = aris::dynamic::s_vv(3, p0_minus_center, p2_minus_center);

			double dir = aris::dynamic::s_vv(3, cross_result, axis);

			length = dir < 0.0 ? radius * (2.0 * aris::PI - std::atan2(s, c)) : radius * std::atan2(s, c);
		}
	}
	auto s_compute_circle3_at(const double* p0, const double* center, const double* axis, double radius, double total_length, double l, double* xyz, double dl =0.0, double *dxyz = nullptr, double d2l = 0.0, double *d2xyz = nullptr) -> void
	{
		l = std::min(l, total_length);
		l = std::max(l, 0.0);

		double arc_at = l / radius;

		double rx[3]{
			p0[0] - center[0],
			p0[1] - center[1],
			p0[2] - center[2],
		};

		double ry[3];
		aris::dynamic::s_c3(axis, rx, ry);

		double s = std::sin(arc_at);
		double c = std::cos(arc_at);

		xyz[0] = center[0] + s * ry[0] + c * rx[0];
		xyz[1] = center[1] + s * ry[1] + c * rx[1];
		xyz[2] = center[2] + s * ry[2] + c * rx[2];

		// 计算速度、加速度 //
		double darc = dl / radius;
		double d2arc = d2l / radius;
		double k1 = c * darc;
		double k2 = -s * darc;
		double k3 = (k2 * darc + c * d2arc);
		double k4 = (-k1 * darc - s * d2arc);

		if (dxyz) {
			dxyz[0] = k1 * ry[0] + k2 * rx[0];
			dxyz[1] = k1 * ry[1] + k2 * rx[1];
			dxyz[2] = k1 * ry[2] + k2 * rx[2];
		}

		if (d2xyz) {
			d2xyz[0] = k3 * ry[0] + k4 * rx[0];
			d2xyz[1] = k3 * ry[1] + k4 * rx[1];
			d2xyz[2] = k3 * ry[2] + k4 * rx[2];
		}
	}

	auto s_make_circle2(const double* p0, const double* p1, const double* p2, double* center, double* axis, double& radius, double& length)->void{
		double p0_[3]{ p0[0],p0[1],0.0 }, p1_[3]{ p1[0],p1[1],0.0 }, p2_[3]{ p2[0],p2[1],0.0 };
		s_make_circle3(p0_, p1_, p2_, center, axis, radius, length);
	}
	auto s_compute_circle2_at(const double* p0, const double* center, const double* axis, double radius, double total_length, double l, double* xyz, double dl = 0.0, double* dxyz = nullptr, double d2l = 0.0, double* d2xyz = nullptr) -> void {
		const double p0_[3]{ p0[0],p0[1],0.0 };
		double xyz_[3], dxyz_[3], d2xyz_[3];
		s_compute_circle3_at(p0_, center, axis, radius, total_length, l, xyz_, dl, dxyz_, d2l, d2xyz_);
		if(xyz)
			aris::dynamic::s_vc(2, xyz_, xyz);
		if(dxyz)
			aris::dynamic::s_vc(2, dxyz_, dxyz);
		if(d2xyz)
			aris::dynamic::s_vc(2, d2xyz_, d2xyz);
	}

	auto s_make_quaternion_data(const double* q0, const double* q1, double& length) -> void {
		double c = aris::dynamic::s_vv(4, q0, q1);
		double q_diff[4];
		aris::dynamic::s_vc(4, -c, q0, q_diff);
		aris::dynamic::s_va(4, q1, q_diff);
			
		double s = aris::dynamic::s_norm(4, q_diff);
		
		length = 2.0 * std::atan2(s, std::abs(c));
	}
	auto s_compute_quaternion_at(const double* q0, const double* q1, double total_length, double l, double* q, double dl = 0.0, double* dq = nullptr, double d2l = 0.0, double* d2q = nullptr) -> void {
		l = std::min(l, total_length);
		l = std::max(l, 0.0);

		double ratio = total_length < std::numeric_limits<double>::epsilon() ? 0.5 : l / total_length;

		double dir = aris::dynamic::s_vv(4, q0, q1) < 0.0 ? -1.0 : 1.0;

		if (total_length < 1e-10) {
			double s1 = (1 - ratio);
			double s2 = dir * ratio;

			q[0] = s1 * q0[0] + s2 * q1[0];
			q[1] = s1 * q0[1] + s2 * q1[1];
			q[2] = s1 * q0[2] + s2 * q1[2];
			q[3] = s1 * q0[3] + s2 * q1[3];

			if (dq) {
				double ds1 = -dl / std::max(total_length, 1e-10);
				double ds2 = dir * dl / std::max(total_length, 1e-10);
				dq[0] = (ds1 * q0[0] + ds2 * q1[0]);
				dq[1] = (ds1 * q0[1] + ds2 * q1[1]);
				dq[2] = (ds1 * q0[2] + ds2 * q1[2]);
				dq[3] = (ds1 * q0[3] + ds2 * q1[3]);
			}

			if (d2q) {
				double d2s1 = -d2l / std::max(total_length, 1e-10);
				double d2s2 = dir * d2l / std::max(total_length, 1e-10);
				d2q[0] = (d2s1 * q0[0] + d2s2 * q1[0]);
				d2q[1] = (d2s1 * q0[1] + d2s2 * q1[1]);
				d2q[2] = (d2s1 * q0[2] + d2s2 * q1[2]);
				d2q[3] = (d2s1 * q0[3] + d2s2 * q1[3]);
			}
		}
		else {
			double s1 = std::sin((total_length - l) / 2.0);
			double s2 = dir * std::sin(l / 2.0);
			double s3 = std::sin(total_length / 2.0);

			q[0] = (s1 * q0[0] + s2 * q1[0]) / s3;
			q[1] = (s1 * q0[1] + s2 * q1[1]) / s3;
			q[2] = (s1 * q0[2] + s2 * q1[2]) / s3;
			q[3] = (s1 * q0[3] + s2 * q1[3]) / s3;

			if (dq) {
				double ds1 = -0.5 * std::cos((total_length - l) / 2.0) * dl;
				double ds2 = dir * 0.5 * std::cos(l / 2.0) * dl;
				dq[0] = (ds1 * q0[0] + ds2 * q1[0]) / s3;
				dq[1] = (ds1 * q0[1] + ds2 * q1[1]) / s3;
				dq[2] = (ds1 * q0[2] + ds2 * q1[2]) / s3;
				dq[3] = (ds1 * q0[3] + ds2 * q1[3]) / s3;
			}

			if (d2q) {
				double d2s1 = -0.5 * std::cos((total_length - l) / 2.0) * d2l - 0.5 * std::sin((total_length - l) / 2.0) * dl * dl;
				double d2s2 = dir * 0.5 * std::cos(l / 2.0) * d2l - dir * 0.5 * 0.5 * std::sin(l / 2.0) * dl * dl;
				d2q[0] = (d2s1 * q0[0] + d2s2 * q1[0]) / s3;
				d2q[1] = (d2s1 * q0[1] + d2s2 * q1[1]) / s3;
				d2q[2] = (d2s1 * q0[2] + d2s2 * q1[2]) / s3;
				d2q[3] = (d2s1 * q0[3] + d2s2 * q1[3]) / s3;
			}

		}
	}

	auto s_compute_data_at_end(const Node::Unit& unit, double *p_end)->void {
		switch (unit.type_) {
		case aris::plan::Node::UnitType::Line3: {
			s_compute_line3_at(unit.move_.line_.p0_, unit.move_.line_.dir_, unit.move_.length_, p_end);
			break;
		}
		case aris::plan::Node::UnitType::Circle3: {
			s_compute_circle3_at(unit.move_.circle_.p0_, unit.move_.circle_.center_, unit.move_.circle_.axis_, unit.move_.circle_.radius_, unit.move_.length_, unit.move_.length_, p_end);
			break;
		}
		case aris::plan::Node::UnitType::Line2: {
			s_compute_line2_at(unit.move_.line_.p0_, unit.move_.line_.dir_, unit.move_.length_, p_end);
			break;
		}
		case aris::plan::Node::UnitType::Circle2: {
			s_compute_circle2_at(unit.move_.circle_.p0_, unit.move_.circle_.center_, unit.move_.circle_.axis_, unit.move_.circle_.radius_, unit.move_.length_, unit.move_.length_, p_end);
			break;
		}
		case aris::plan::Node::UnitType::Rotate3: {
			aris::dynamic::s_vc(4, unit.move_.quaternion_.q1_, p_end);
			break;
		}
		case aris::plan::Node::UnitType::Line1: {
			*p_end = *unit.move_.line_.p0_ + (*unit.move_.line_.dir_) * unit.move_.length_;
			break;
		}
		default:
			break;
		}
	}

	// init unit //
	auto init_unit_l3(const double* p0, const double* p1, double vel, double acc, double jerk, double zone, Node::Unit& unit) -> void {
		// move type //
		unit.type_ = Node::UnitType::Line3;
		
		// moves //
		std::fill_n(unit.move_.line_.p0_, 3, 0.0);
		std::fill_n(unit.move_.line_.dir_, 3, 0.0);
		aris::dynamic::s_vc(3, p0, unit.move_.line_.p0_);
		s_make_line3(p0, p1, unit.move_.line_.dir_, unit.move_.length_);
		unit.move_.origin_length_ = unit.move_.length_;

		// zones //
		unit.zone1_.type_ = Node::Zone::ZoneType::LL;
		unit.zone1_.zone_value_ = 0.0;
		unit.zone1_.length_ = 0.0;
		std::fill_n(unit.zone1_.lines_.p0_, 3, 0.0);
		aris::dynamic::s_vc(3, p0, unit.zone1_.lines_.p0_);
		std::fill_n(unit.zone1_.lines_.p1_, 3, 0.0);
		aris::dynamic::s_vc(3, p0, unit.zone1_.lines_.p1_);
		std::fill_n(unit.zone1_.lines_.p2_, 3, 0.0);
		aris::dynamic::s_vc(3, p0, unit.zone1_.lines_.p2_);
		unit.zone2_.type_ = Node::Zone::ZoneType::LL;
		unit.zone2_.zone_value_ = zone;
		unit.zone2_.length_ = 0.0;
		std::fill_n(unit.zone2_.lines_.p0_, 3, 0.0);
		aris::dynamic::s_vc(3, p1, unit.zone2_.lines_.p0_);
		std::fill_n(unit.zone2_.lines_.p1_, 3, 0.0);
		aris::dynamic::s_vc(3, p1, unit.zone2_.lines_.p1_);
		std::fill_n(unit.zone2_.lines_.p2_, 3, 0.0);
		aris::dynamic::s_vc(3, p1, unit.zone2_.lines_.p2_);

		// scurves //
		double p = unit.zone1_.length_ / 2.0 + unit.zone2_.length_ / 2.0 + unit.move_.length_;
		unit.scurve_.pa_ = 0.0;
		unit.scurve_.pb_ = p;
		unit.scurve_.va_ = 0.0;
		unit.scurve_.vc_max_ = vel;
		unit.scurve_.vb_max_ = 0.0;
		unit.scurve_.a_ = acc;
		unit.scurve_.j_ = jerk;
		unit.scurve_.t0_ = 0.0;
	}
	auto init_unit_l2(const double* p0, const double* p1, double vel, double acc, double jerk, double zone, Node::Unit& unit) -> void {
		const double p0_[3]{ p0[0], p0[1], 0.0}, p1_[3]{ p1[0], p1[1], 0.0 };
		init_unit_l3(p0_, p1_, vel, acc, jerk, zone, unit);
		unit.type_ = Node::UnitType::Line2;
	}
	auto init_unit_l1(const double* p0, const double* p1, double vel, double acc, double jerk, double zone, Node::Unit& unit) -> void {
		const double p0_[3]{ p0[0], 0.0, 0.0}, p1_[3]{ p1[0], 0.0, 0.0};
		init_unit_l3(p0_, p1_, vel, acc, jerk, zone, unit);
		unit.type_ = Node::UnitType::Line1;
	}
	auto init_unit_c3(const double* p0, const double* p1, const double* p2, double vel, double acc, double jerk, double zone, Node::Unit& unit) -> void {
		// move type //
		unit.type_ = Node::UnitType::Circle3;
		
		// moves //
		std::fill_n(unit.move_.circle_.p0_, 3, 0.0);
		aris::dynamic::s_vc(3, p0, unit.move_.circle_.p0_);
		s_make_circle3(p0, p1, p2, unit.move_.circle_.center_, unit.move_.circle_.axis_, unit.move_.circle_.radius_, unit.move_.length_);
		unit.move_.origin_length_ = unit.move_.length_;

		// 考虑退化 //
		if (!std::isfinite(unit.move_.circle_.radius_)) {
			unit.type_ = Node::UnitType::Line3;
			init_unit_l3(p0, p2, vel, acc, jerk, zone, unit);
			return;
		}

		// zones //
		unit.zone1_.type_ = Node::Zone::ZoneType::LL;
		unit.zone1_.zone_value_ = 0.0;
		unit.zone1_.length_ = 0.0;
		std::fill_n(unit.zone1_.lines_.p0_, 3, 0.0);
		aris::dynamic::s_vc(3, p0, unit.zone1_.lines_.p0_);
		std::fill_n(unit.zone1_.lines_.p1_, 3, 0.0);
		aris::dynamic::s_vc(3, p0, unit.zone1_.lines_.p1_);
		std::fill_n(unit.zone1_.lines_.p2_, 3, 0.0);
		aris::dynamic::s_vc(3, p0, unit.zone1_.lines_.p2_);
		unit.zone2_.type_ = Node::Zone::ZoneType::LL;
		unit.zone2_.zone_value_ = zone;
		unit.zone2_.length_ = 0.0;
		std::fill_n(unit.zone2_.lines_.p0_, 3, 0.0);
		aris::dynamic::s_vc(3, p2, unit.zone2_.lines_.p0_);
		std::fill_n(unit.zone2_.lines_.p1_, 3, 0.0);
		aris::dynamic::s_vc(3, p2, unit.zone2_.lines_.p1_);
		std::fill_n(unit.zone2_.lines_.p2_, 3, 0.0);
		aris::dynamic::s_vc(3, p2, unit.zone2_.lines_.p2_);

		// scurves //
		double p = unit.zone1_.length_ / 2.0 + unit.zone2_.length_ / 2.0 + unit.move_.length_;
		unit.scurve_.pa_ = 0.0;
		unit.scurve_.pb_ = p;
		unit.scurve_.va_ = 0.0;
		unit.scurve_.vc_max_ = std::min(vel, std::sqrt(acc * unit.move_.circle_.radius_));
		unit.scurve_.vb_max_ = 0.0;
		unit.scurve_.a_ = acc;
		unit.scurve_.j_ = jerk;
		unit.scurve_.t0_ = 0.0;
	}
	auto init_unit_c2(const double* p0, const double* p1, const double* p2, double vel, double acc, double jerk, double zone, Node::Unit& unit) -> void {
		const double p0_[3]{ p0[0], p0[1], 0.0 }, p1_[3]{ p1[0], p1[1], 0.0 }, p2_[3]{ p2[0], p2[1], 0.0 };
		init_unit_c3(p0_, p1_, p2_, vel, acc, jerk, zone, unit);
		unit.type_ = Node::UnitType::Circle2;
	}
	auto init_unit_q3(const double* q0, const double* q1, double vel, double acc, double jerk, double zone, Node::Unit& unit) -> void {
		// move type //
		unit.type_ = Node::UnitType::Rotate3;
		
		// 将两个四元数方向变得统一 //
		double q1_[4]{q1[0],q1[1],q1[2],q1[3]};
		if (aris::dynamic::s_vv(4, q0, q1_) < 0.0) {
			aris::dynamic::s_nv(4, -1.0, q1_);
		}
		
		// moves //
		aris::dynamic::s_vc(4, q0, unit.move_.quaternion_.q0_);
		aris::dynamic::s_vc(4, q1_, unit.move_.quaternion_.q1_);
		s_make_quaternion_data(unit.move_.quaternion_.q0_, unit.move_.quaternion_.q1_, unit.move_.length_);
		unit.move_.origin_length_ = unit.move_.length_;

		// zones //
		unit.zone1_.type_ = Node::Zone::ZoneType::QQ;
		unit.zone1_.zone_value_ = 0.0;
		unit.zone1_.length_ = 0.0;
		aris::dynamic::s_vc(4, unit.move_.quaternion_.q0_, unit.zone1_.quaternions_.q0_);
		aris::dynamic::s_vc(4, unit.move_.quaternion_.q0_, unit.zone1_.quaternions_.q1_);
		aris::dynamic::s_vc(4, unit.move_.quaternion_.q0_, unit.zone1_.quaternions_.q2_);
		unit.zone2_.type_ = Node::Zone::ZoneType::QQ;
		unit.zone2_.zone_value_ = zone;
		unit.zone2_.length_ = 0.0;
		aris::dynamic::s_vc(4, unit.move_.quaternion_.q1_, unit.zone2_.quaternions_.q0_);
		aris::dynamic::s_vc(4, unit.move_.quaternion_.q1_, unit.zone2_.quaternions_.q1_);
		aris::dynamic::s_vc(4, unit.move_.quaternion_.q1_, unit.zone2_.quaternions_.q2_);

		// scurves //
		double p = unit.zone1_.length_ / 2.0 + unit.zone2_.length_ / 2.0 + unit.move_.length_;
		unit.scurve_.pa_ = 0.0;
		unit.scurve_.pb_ = p;
		unit.scurve_.va_ = 0.0;
		unit.scurve_.vc_max_ = vel;
		unit.scurve_.vb_max_ = 0.0;
		unit.scurve_.a_ = acc;
		unit.scurve_.j_ = jerk;
		unit.scurve_.t0_ = 0.0;
	}

	auto init_unit(Node::UnitType unit_type, const double* p0, const double* p1, const double* p2, double vel, double acc, double jerk, double zone, Node::Unit& unit)->void {
		switch (unit_type) {
		case aris::plan::Node::UnitType::Line3: {
			init_unit_l3(p0, p2, vel, acc, jerk, zone, unit);
			break;
		}
		case aris::plan::Node::UnitType::Circle3: {
			init_unit_c3(p0, p1, p2, vel, acc, jerk, zone, unit);
			break;
		}
		case aris::plan::Node::UnitType::Rotate3: {
			init_unit_q3(p0, p2, vel, acc, jerk, zone, unit);
			break;
		}
		case aris::plan::Node::UnitType::Line2: {
			init_unit_l2(p0, p2, vel, acc, jerk, zone, unit);
			break;
		}
		case aris::plan::Node::UnitType::Circle2: {
			init_unit_c2(p0, p1, p2, vel, acc, jerk, zone, unit);
			break;
		}
		case aris::plan::Node::UnitType::Line1: {
			init_unit_l1(p0, p2, vel, acc, jerk, zone, unit);
			break;
		}
		default:
			break;
		}
	}

	// connect unit to last unit //
	auto make_zone_and_scurve_ll(Node::Unit& last_u, Node::Unit& this_u) ->void {
		// STEP 1. 计算真实的交融半径
		double real_zone = std::min({ last_u.move_.origin_length_ * 0.5, last_u.zone2_.zone_value_, this_u.move_.origin_length_ * 0.5 });

		// STEP 2. 计算 last_p 和 this_p 的交融点
		double p1[3], p01[3], p12[3];
		s_compute_line3_at(last_u.move_.line_.p0_, last_u.move_.line_.dir_, last_u.move_.length_, p1);
		s_compute_line3_at(last_u.move_.line_.p0_, last_u.move_.line_.dir_, last_u.move_.length_ - real_zone, p01);
		s_compute_line3_at(this_u.move_.line_.p0_, this_u.move_.line_.dir_, real_zone, p12);

		// STEP 3. 更新 last_p 和 this_p 的 move 部分
		last_u.move_.length_ -= real_zone;
		aris::dynamic::s_vc(3, p12, this_u.move_.line_.p0_);
		this_u.move_.length_ -= real_zone;

		// STEP 4. 更新 last_p 和 this_p 的 zone 部分
		auto& zone_x2 = last_u.zone2_;
		zone_x2.type_ = Node::Zone::ZoneType::LL;
		aris::dynamic::s_vc(3, p01, zone_x2.lines_.p0_);
		aris::dynamic::s_vc(3, p1, zone_x2.lines_.p1_);
		aris::dynamic::s_vc(3, p12, zone_x2.lines_.p2_);

		double arc, darc, d2arc;
		double darc_ds_0, d2arc_ds2_0, ds_darc_0, d2s_darc2_0;
		double darc_ds_1, d2arc_ds2_1, ds_darc_1, d2s_darc2_1;
		double darc_ds_50, d2arc_ds2_50, ds_darc_50, d2s_darc2_50;

		double p[3], dp[3], d2p[3];
		s_bezier3_blend_line_line(0.0, p01, p1, p12, p, dp, d2p);
		s_bezier3_darc_ds(3, dp, d2p, darc_ds_0, d2arc_ds2_0, ds_darc_0, d2s_darc2_0);

		s_bezier3_blend_line_line(1.0, p01, p1, p12, p, dp, d2p);
		s_bezier3_darc_ds(3, dp, d2p, darc_ds_1, d2arc_ds2_1, ds_darc_1, d2s_darc2_1);

		s_bezier3_blend_line_line(0.5, p01, p1, p12, p, dp, d2p);
		s_bezier3_darc_ds(3, dp, d2p, darc_ds_50, d2arc_ds2_50, ds_darc_50, d2s_darc2_50);

		s_bezier3_estimate_arc_param(darc_ds_0, d2arc_ds2_0, darc_ds_1, d2arc_ds2_1, darc_ds_50
			, zone_x2.bezier_param);

		s_bezier3_s2arc(1.0, zone_x2.bezier_param, arc, darc, d2arc);
		zone_x2.length_ = arc;

		this_u.zone1_ = zone_x2;

		// STEP 5. 更新 scurve //
		last_u.scurve_.pb_ = last_u.scurve_.pa_ + last_u.zone1_.length_ / 2.0 + last_u.zone2_.length_ / 2.0 + last_u.move_.length_;
		this_u.scurve_.pa_ = last_u.scurve_.pb_;
		this_u.scurve_.pb_ = this_u.scurve_.pa_ + this_u.zone1_.length_ / 2.0 + this_u.zone2_.length_ / 2.0 + this_u.move_.length_;

		// STEP 6. 考虑曲线的真实曲率（一部分加速度必须用来克服曲率），修正交融中点处的最大速度 //
		double p50[4], dp50[4], d2p50[4], d3p50[4];
		double vb;
		s_bezier3_blend_line_line(0.5, last_u.zone2_.lines_.p0_, last_u.zone2_.lines_.p1_, last_u.zone2_.lines_.p2_,
			p50, dp50, d2p50, d3p50);
		s_bezier3_max_v_at(3, arc, dp50, d2p50, d3p50, std::min(last_u.scurve_.a_, this_u.scurve_.a_), std::min(last_u.scurve_.j_, this_u.scurve_.j_), vb);
		last_u.scurve_.vb_max_ = std::min({ vb, last_u.scurve_.vc_max_, this_u.scurve_.vc_max_ });
	}
	auto make_zone_and_scurve_lc(Node::Unit& last_u, Node::Unit& this_u) ->void {
		// STEP 0. 检查是否需要退化成直线
		if (this_u.type_ == Node::UnitType::Line3 && last_u.type_ == Node::UnitType::Line3) {
			make_zone_and_scurve_ll(last_u, this_u);
			return;
		}
		
		// STEP 1. 计算真实的交融半径
		double real_zone = std::min({ last_u.move_.origin_length_ * 0.5, last_u.zone2_.zone_value_, this_u.move_.origin_length_ * 0.5 });

		// STEP 2. 计算 last_p 和 this_p 的交融点
		double p1[3], p01[3], p12[3];
		//aris::dynamic::s_vc(3, last_u.move_.line_.p1_, p1);
		s_compute_line3_at(last_u.move_.line_.p0_, last_u.move_.line_.dir_, last_u.move_.length_, p1);

		s_compute_line3_at(
			last_u.move_.line_.p0_,
			last_u.move_.line_.dir_,
			last_u.move_.length_ - real_zone,
			p01);

		s_compute_circle3_at(
			this_u.move_.circle_.p0_,
			this_u.move_.circle_.center_,
			this_u.move_.circle_.axis_,
			this_u.move_.circle_.radius_,
			this_u.move_.length_,
			real_zone, p12);

		// STEP 3. 更新 last_p 和 this_p 的 move 部分
		last_u.move_.length_ -= real_zone;
		aris::dynamic::s_vc(3, p12, this_u.move_.circle_.p0_);
		this_u.move_.length_ -= real_zone;

		// STEP 4. 更新 last_p 和 this_p 的 zone 部分
		auto& zone_x2 = last_u.zone2_;
		zone_x2.type_ = Node::Zone::ZoneType::LC;
		aris::dynamic::s_vc(3, p01, zone_x2.line_circle_.p0_);
		aris::dynamic::s_vc(3, p1, zone_x2.line_circle_.p1_);
		aris::dynamic::s_vc(3, this_u.move_.circle_.center_, zone_x2.line_circle_.center_);
		aris::dynamic::s_vc(3, this_u.move_.circle_.axis_, zone_x2.line_circle_.axis_);
		zone_x2.line_circle_.theta_ = real_zone / this_u.move_.circle_.radius_;

		double arc, darc, d2arc;
		double darc_ds_0, d2arc_ds2_0, ds_darc_0, d2s_darc2_0;
		double darc_ds_1, d2arc_ds2_1, ds_darc_1, d2s_darc2_1;
		double darc_ds_50, d2arc_ds2_50, ds_darc_50, d2s_darc2_50;

		double p[3], dp[3], d2p[3];
		s_bezier3_blend_line_circle(0.0, p01, p1, zone_x2.line_circle_.center_, zone_x2.line_circle_.axis_, zone_x2.line_circle_.theta_, p, dp, d2p);
		s_bezier3_darc_ds(3, dp, d2p, darc_ds_0, d2arc_ds2_0, ds_darc_0, d2s_darc2_0);

		s_bezier3_blend_line_circle(1.0, p01, p1, zone_x2.line_circle_.center_, zone_x2.line_circle_.axis_, zone_x2.line_circle_.theta_, p, dp, d2p);
		s_bezier3_darc_ds(3, dp, d2p, darc_ds_1, d2arc_ds2_1, ds_darc_1, d2s_darc2_1);

		s_bezier3_blend_line_circle(0.5, p01, p1, zone_x2.line_circle_.center_, zone_x2.line_circle_.axis_, zone_x2.line_circle_.theta_, p, dp, d2p);
		s_bezier3_darc_ds(3, dp, d2p, darc_ds_50, d2arc_ds2_50, ds_darc_50, d2s_darc2_50);

		s_bezier3_estimate_arc_param(darc_ds_0, d2arc_ds2_0, darc_ds_1, d2arc_ds2_1, darc_ds_50
			, zone_x2.bezier_param);

		s_bezier3_s2arc(1.0, zone_x2.bezier_param, arc, darc, d2arc);
		zone_x2.length_ = arc;

		this_u.zone1_ = zone_x2;

		// STEP 5. 更新 scurve //
		last_u.scurve_.pb_ = last_u.scurve_.pa_ + last_u.zone1_.length_ / 2.0 + last_u.zone2_.length_ / 2.0 + last_u.move_.length_;
		this_u.scurve_.pa_ = last_u.scurve_.pb_;
		this_u.scurve_.pb_ = this_u.scurve_.pa_ + this_u.zone1_.length_ / 2.0 + this_u.zone2_.length_ / 2.0 + this_u.move_.length_;

		// STEP 6. 考虑曲线的真实曲率（一部分加速度必须用来克服曲率），修正交融中点处的最大速度 //
		double p50[4], dp50[4], d2p50[4], d3p50[4];
		double vb;
		s_bezier3_blend_line_circle(0.5, last_u.zone2_.line_circle_.p0_, last_u.zone2_.line_circle_.p1_
			, last_u.zone2_.line_circle_.center_, last_u.zone2_.line_circle_.axis_, last_u.zone2_.line_circle_.theta_,
			p50, dp50, d2p50, d3p50);
		s_bezier3_max_v_at(3, arc, dp50, d2p50, d3p50, std::min(last_u.scurve_.a_, this_u.scurve_.a_), std::min(last_u.scurve_.j_, this_u.scurve_.j_), vb);
		last_u.scurve_.vb_max_ = std::min({ vb, last_u.scurve_.vc_max_, this_u.scurve_.vc_max_ });
	}
	auto make_zone_and_scurve_cl(Node::Unit& last_u, Node::Unit& this_u) ->void {
		// STEP 0. 检查是否需要退化成直线
		if (this_u.type_ == Node::UnitType::Line3 && last_u.type_ == Node::UnitType::Line3) {
			make_zone_and_scurve_ll(last_u, this_u);
			return;
		}
		
		// STEP 1. 计算真实的交融半径
		double real_zone = std::min({ last_u.move_.origin_length_ * 0.5, last_u.zone2_.zone_value_, this_u.move_.origin_length_ * 0.5 });

		// STEP 2. 计算 last_p 和 this_p 的交融点，只需要计算直线部分的接触点，因为圆的起点就是两者交点（p1）
		double p1[3], p12[3];
		aris::dynamic::s_vc(3, this_u.move_.line_.p0_, p1);
		s_compute_line3_at(this_u.move_.line_.p0_, this_u.move_.line_.dir_, real_zone, p12);

		// STEP 3. 更新 last_p 和 this_p 的 move 部分
		last_u.move_.length_ -= real_zone;
		aris::dynamic::s_vc(3, p12, this_u.move_.line_.p0_);
		this_u.move_.length_ -= real_zone;

		// STEP 4. 更新 last_p 和 this_p 的 zone 部分
		auto& zone_x2 = last_u.zone2_;
		zone_x2.type_ = Node::Zone::ZoneType::CL;
		aris::dynamic::s_vc(3, last_u.move_.circle_.center_, zone_x2.circle_line_.center_);
		aris::dynamic::s_vc(3, -1.0, last_u.move_.circle_.axis_, zone_x2.circle_line_.axis_);
		aris::dynamic::s_vc(3, p1, zone_x2.circle_line_.p1_);   // p1 实际是circle的终点，但是把它视为 zone 的反向起点
		aris::dynamic::s_vc(3, p12, zone_x2.circle_line_.p2_);
		zone_x2.circle_line_.theta_ = real_zone / last_u.move_.circle_.radius_;

		double arc, darc, d2arc;
		double darc_ds_0, d2arc_ds2_0, ds_darc_0, d2s_darc2_0;
		double darc_ds_1, d2arc_ds2_1, ds_darc_1, d2s_darc2_1;
		double darc_ds_50, d2arc_ds2_50, ds_darc_50, d2s_darc2_50;

		double p[3], dp[3], d2p[3];
		s_bezier3_blend_line_circle(0.0, p12, p1, zone_x2.circle_line_.center_, zone_x2.circle_line_.axis_, zone_x2.circle_line_.theta_, p, dp, d2p);
		s_bezier3_darc_ds(3, dp, d2p, darc_ds_0, d2arc_ds2_0, ds_darc_0, d2s_darc2_0);

		s_bezier3_blend_line_circle(1.0, p12, p1, zone_x2.circle_line_.center_, zone_x2.circle_line_.axis_, zone_x2.circle_line_.theta_, p, dp, d2p);
		s_bezier3_darc_ds(3, dp, d2p, darc_ds_1, d2arc_ds2_1, ds_darc_1, d2s_darc2_1);

		s_bezier3_blend_line_circle(0.5, p12, p1, zone_x2.circle_line_.center_, zone_x2.circle_line_.axis_, zone_x2.circle_line_.theta_, p, dp, d2p);
		s_bezier3_darc_ds(3, dp, d2p, darc_ds_50, d2arc_ds2_50, ds_darc_50, d2s_darc2_50);

		s_bezier3_estimate_arc_param(darc_ds_0, d2arc_ds2_0, darc_ds_1, d2arc_ds2_1, darc_ds_50, zone_x2.bezier_param);

		s_bezier3_s2arc(1.0, zone_x2.bezier_param, arc, darc, d2arc);
		zone_x2.length_ = arc;

		this_u.zone1_ = zone_x2;

		// STEP 5. 更新 scurve //
		last_u.scurve_.pb_ = last_u.scurve_.pa_ + last_u.zone1_.length_ / 2.0 + last_u.zone2_.length_ / 2.0 + last_u.move_.length_;
		this_u.scurve_.pa_ = last_u.scurve_.pb_;
		this_u.scurve_.pb_ = this_u.scurve_.pa_ + this_u.zone1_.length_ / 2.0 + this_u.zone2_.length_ / 2.0 + this_u.move_.length_;

		// STEP 6. 考虑曲线的真实曲率（一部分加速度必须用来克服曲率），修正交融中点处的最大速度 //
		double p50[4], dp50[4], d2p50[4], d3p50[4];
		double vb;
		s_bezier3_blend_line_circle(0.5, last_u.zone2_.circle_line_.p2_, last_u.zone2_.circle_line_.p1_
			, last_u.zone2_.circle_line_.center_, last_u.zone2_.circle_line_.axis_, last_u.zone2_.circle_line_.theta_,
			p50, dp50, d2p50, d3p50);
		s_bezier3_max_v_at(3, arc, dp50, d2p50, d3p50, std::min(last_u.scurve_.a_, this_u.scurve_.a_), std::min(last_u.scurve_.j_, this_u.scurve_.j_), vb);
		last_u.scurve_.vb_max_ = std::min({ vb, last_u.scurve_.vc_max_, this_u.scurve_.vc_max_ });
	}
	auto make_zone_and_scurve_cc(Node::Unit& last_u, Node::Unit& this_u) ->void {
		// STEP 0. 检查是否需要退化成直线
		if (this_u.type_ == Node::UnitType::Line3 && last_u.type_ == Node::UnitType::Circle3) {
			make_zone_and_scurve_cl(last_u, this_u);
			return;
		}
		else if (this_u.type_ == Node::UnitType::Circle3 && last_u.type_ == Node::UnitType::Line3) {
			make_zone_and_scurve_lc(last_u, this_u);
			return;
		}
		else if (this_u.type_ == Node::UnitType::Line3 && last_u.type_ == Node::UnitType::Line3) {
			make_zone_and_scurve_ll(last_u, this_u);
			return;
		}
		
		// STEP 1. 计算真实的交融半径
		double real_zone = std::min({ last_u.move_.origin_length_*0.5, last_u.zone2_.zone_value_, this_u.move_.origin_length_*0.5 });

		// STEP 2. 计算 last_p 和 this_p 的交融点
		double p1[3];
		aris::dynamic::s_vc(3, this_u.move_.circle_.p0_, p1);

		// STEP 3. 更新 last_p 和 this_p 的 move 部分
		last_u.move_.length_ -= real_zone;
		s_compute_circle3_at(
			this_u.move_.circle_.p0_,
			this_u.move_.circle_.center_,
			this_u.move_.circle_.axis_,
			this_u.move_.circle_.radius_,
			this_u.move_.length_,
			real_zone, this_u.move_.circle_.p0_);
		this_u.move_.length_ -= real_zone;

		// STEP 4. 更新 last_p 和 this_p 的 zone 部分
		auto& zone_x2 = last_u.zone2_;
		auto& circles = zone_x2.circles_;
		
		zone_x2.type_ = Node::Zone::ZoneType::CC;
		aris::dynamic::s_vc(3, p1, circles.pcenter_);
		aris::dynamic::s_vc(3, last_u.move_.circle_.axis_, circles.a1_);
		aris::dynamic::s_vc(3, last_u.move_.circle_.center_, circles.c1_);
		aris::dynamic::s_vc(3, this_u.move_.circle_.axis_, circles.a2_);
		aris::dynamic::s_vc(3, this_u.move_.circle_.center_, circles.c2_);
		circles.theta1_ = real_zone / last_u.move_.circle_.radius_;
		circles.theta2_ = real_zone / this_u.move_.circle_.radius_;

		double arc, darc, d2arc;
		double darc_ds_0, d2arc_ds2_0, ds_darc_0, d2s_darc2_0;
		double darc_ds_1, d2arc_ds2_1, ds_darc_1, d2s_darc2_1;
		double darc_ds_50, d2arc_ds2_50, ds_darc_50, d2s_darc2_50;

		double p[3], dp[3], d2p[3];
		s_bezier3_blend_circle_circle(0.0, circles.pcenter_, circles.c1_, circles.a1_, circles.theta1_
			, circles.c2_, circles.a2_, circles.theta2_
			, p, dp, d2p);
		s_bezier3_darc_ds(3, dp, d2p, darc_ds_0, d2arc_ds2_0, ds_darc_0, d2s_darc2_0);

		s_bezier3_blend_circle_circle(1.0, circles.pcenter_, circles.c1_, circles.a1_, circles.theta1_
			, circles.c2_, circles.a2_, circles.theta2_
			, p, dp, d2p);
		s_bezier3_darc_ds(3, dp, d2p, darc_ds_1, d2arc_ds2_1, ds_darc_1, d2s_darc2_1);

		s_bezier3_blend_circle_circle(0.5, circles.pcenter_, circles.c1_, circles.a1_, circles.theta1_
			, circles.c2_, circles.a2_, circles.theta2_
			, p, dp, d2p);
		s_bezier3_darc_ds(3, dp, d2p, darc_ds_50, d2arc_ds2_50, ds_darc_50, d2s_darc2_50);

		s_bezier3_estimate_arc_param(darc_ds_0, d2arc_ds2_0, darc_ds_1, d2arc_ds2_1, darc_ds_50
			, zone_x2.bezier_param);

		s_bezier3_s2arc(1.0, zone_x2.bezier_param, arc, darc, d2arc);
		zone_x2.length_ = arc;

		this_u.zone1_ = zone_x2;

		// STEP 5. 更新 scurve //
		last_u.scurve_.pb_ = last_u.scurve_.pa_ + last_u.zone1_.length_ / 2.0 + last_u.zone2_.length_ / 2.0 + last_u.move_.length_;
		this_u.scurve_.pa_ = last_u.scurve_.pb_;
		this_u.scurve_.pb_ = this_u.scurve_.pa_ + this_u.zone1_.length_ / 2.0 + this_u.zone2_.length_ / 2.0 + this_u.move_.length_;

		// STEP 6. 考虑曲线的真实曲率（一部分加速度必须用来克服曲率），修正交融中点处的最大速度 //
		double p50[4], dp50[4], d2p50[4], d3p50[4];
		double vb;
		s_bezier3_blend_circle_circle(0.5, circles.pcenter_, circles.c1_, circles.a1_, circles.theta1_
			, circles.c2_, circles.a2_, circles.theta2_
			, p50, dp50, d2p50, d3p50);
		s_bezier3_max_v_at(3, arc, dp50, d2p50, d3p50, std::min(last_u.scurve_.a_, this_u.scurve_.a_), std::min(last_u.scurve_.j_, this_u.scurve_.j_), vb);
		last_u.scurve_.vb_max_ = std::min({ vb, last_u.scurve_.vc_max_, this_u.scurve_.vc_max_ });
	}
	auto make_zone_and_scurve_qq(Node::Unit& last_u, Node::Unit& this_u)->void {
		// STEP 1. 计算真实的交融半径
		double real_zone = std::min({ last_u.move_.origin_length_ * 0.5, last_u.zone2_.zone_value_, this_u.move_.origin_length_*0.5 });

		// STEP 2. 计算 last_p 和 this_p 的交融点
		double q1[4], q01[4], q12[4];
		aris::dynamic::s_vc(4, last_u.move_.quaternion_.q1_, q1);

		s_compute_quaternion_at(
			last_u.move_.quaternion_.q0_,
			last_u.move_.quaternion_.q1_,
			last_u.move_.length_,
			last_u.move_.length_ - real_zone,
			q01);

		s_compute_quaternion_at(
			this_u.move_.quaternion_.q0_,
			this_u.move_.quaternion_.q1_,
			this_u.move_.length_,
			real_zone,
			q12);

		// STEP 3. 更新 last_p 和 this_p 的 move 部分
		aris::dynamic::s_vc(4, q01, last_u.move_.quaternion_.q1_);
		last_u.move_.length_ -= real_zone;
		aris::dynamic::s_vc(4, q12, this_u.move_.quaternion_.q0_);
		this_u.move_.length_ -= real_zone;

		// STEP 4. 更新 last_p 和 this_p 的 zone 部分
		auto& zone_a2 = last_u.zone2_;
		zone_a2.type_ = Node::Zone::ZoneType::QQ;
		aris::dynamic::s_vc(4, q01, zone_a2.quaternions_.q0_);
		aris::dynamic::s_vc(4, q1, zone_a2.quaternions_.q1_);
		aris::dynamic::s_vc(4, q12, zone_a2.quaternions_.q2_);

		double arc, darc, d2arc;
		double darc_ds_0, d2arc_ds2_0, ds_darc_0, d2s_darc2_0;
		double darc_ds_1, d2arc_ds2_1, ds_darc_1, d2s_darc2_1;
		double darc_ds_50, d2arc_ds2_50, ds_darc_50, d2s_darc2_50;

		double q[4], dq[4], d2q[4];
		s_bezier3_blend_quaternion(0.0, q01, q1, q12, q, dq, d2q);
		s_bezier3_darc_ds(4, dq, d2q, darc_ds_0, d2arc_ds2_0, ds_darc_0, d2s_darc2_0);

		s_bezier3_blend_quaternion(1.0, q01, q1, q12, q, dq, d2q);
		s_bezier3_darc_ds(4, dq, d2q, darc_ds_1, d2arc_ds2_1, ds_darc_1, d2s_darc2_1);

		s_bezier3_blend_quaternion(0.5, q01, q1, q12, q, dq, d2q);
		s_bezier3_darc_ds(4, dq, d2q, darc_ds_50, d2arc_ds2_50, ds_darc_50, d2s_darc2_50);

		// 弧度是四元数插值的模乘以2
		s_bezier3_estimate_arc_param(2.0 * darc_ds_0, 2.0 * d2arc_ds2_0, 2.0 * darc_ds_1, 2.0 * d2arc_ds2_1, 2.0 * darc_ds_50
			, zone_a2.bezier_param);

		s_bezier3_s2arc(1.0, zone_a2.bezier_param, arc, darc, d2arc);
		zone_a2.length_ = arc;

		this_u.zone1_ = zone_a2;

		// STEP 5. 更新 scurve //
		last_u.scurve_.pb_ = last_u.scurve_.pa_ + last_u.zone1_.length_ / 2.0 + last_u.zone2_.length_ / 2.0 + last_u.move_.length_;
		this_u.scurve_.pa_ = last_u.scurve_.pb_;
		this_u.scurve_.pb_ = this_u.scurve_.pa_ + this_u.zone1_.length_ / 2.0 + this_u.zone2_.length_ / 2.0 + this_u.move_.length_;

		// STEP 6. 考虑曲线的真实曲率（一部分加速度必须用来克服曲率），修正交融中点处的最大速度 //
		double p50[4], dp50[4], d2p50[4], d3p50[4];
		double vb;
		s_bezier3_blend_quaternion(0.5, last_u.zone2_.quaternions_.q0_, last_u.zone2_.quaternions_.q1_, last_u.zone2_.quaternions_.q2_,
			p50, dp50, d2p50, d3p50);

		// 需要将四元数转化为角速度与角加速度
		double xa50[3], wa50[3], ja50[3];
		// 这里先用 s_xq2xa 得到中点处的角速度/角加速度（wa50/xa50）。
		// 当前 ja50 仍通过下方数值差分从 xa(s) 近似获得，便于快速接通 jerk 约束。
		// TODO: 后续改为对四元数链路做解析求导，直接得到角跃度，替代该数值差分实现。
		aris::dynamic::s_xq2xa(p50, dp50, d2p50, xa50, wa50);
		{
			// 数值差分估计 ja50 = d(xa)/ds：
			// 1) 用 p50/dp50/d2p50/d3p50 构造 s±h 处状态；
			// 2) 通过 s_xq2xa 得到 xa(s±h)；
			// 3) 中心差分 + Richardson 外推提升稳定性。
			auto compute_ja = [&](double h, double *ja)->void {
				double p_plus[4], dp_plus[4], d2p_plus[4], xa_plus[3], wa_plus[3];
				double p_minus[4], dp_minus[4], d2p_minus[4], xa_minus[3], wa_minus[3];

				for (int i = 0; i < 4; ++i) {
					p_plus[i] = p50[i] + h * dp50[i] + 0.5 * h * h * d2p50[i] + h * h * h * d3p50[i] / 6.0;
					dp_plus[i] = dp50[i] + h * d2p50[i] + 0.5 * h * h * d3p50[i];
					d2p_plus[i] = d2p50[i] + h * d3p50[i];

					p_minus[i] = p50[i] - h * dp50[i] + 0.5 * h * h * d2p50[i] - h * h * h * d3p50[i] / 6.0;
					dp_minus[i] = dp50[i] - h * d2p50[i] + 0.5 * h * h * d3p50[i];
					d2p_minus[i] = d2p50[i] - h * d3p50[i];
				}

				aris::dynamic::s_xq2xa(p_plus, dp_plus, d2p_plus, xa_plus, wa_plus);
				aris::dynamic::s_xq2xa(p_minus, dp_minus, d2p_minus, xa_minus, wa_minus);
				for (int i = 0; i < 3; ++i) {
					ja[i] = (xa_plus[i] - xa_minus[i]) / (2.0 * h);
				}
			};

			constexpr double h = 1e-7;
			double ja_h[3], ja_h2[3];
			compute_ja(h, ja_h);
			compute_ja(h * 0.5, ja_h2);
			for (int i = 0; i < 3; ++i) {
				ja50[i] = (4.0 * ja_h2[i] - ja_h[i]) / 3.0;
			}
		}

		auto j_max = std::min(last_u.scurve_.j_, this_u.scurve_.j_);
		auto a_max = std::min(last_u.scurve_.a_, this_u.scurve_.a_);

		s_bezier3_max_v_at(3, arc, wa50, xa50, ja50, a_max, j_max, vb);
		last_u.scurve_.vb_max_ = std::min({ vb, last_u.scurve_.vc_max_, this_u.scurve_.vc_max_ });
	}

	auto make_zone_and_scurve(Node::Unit& last_u, Node::Unit& this_u) ->void {
		switch (last_u.type_) {
		case Node::UnitType::Line3:
			switch (this_u.type_) {
			case Node::UnitType::Line3:
				make_zone_and_scurve_ll(last_u, this_u);
				break;
			case Node::UnitType::Circle3:
				make_zone_and_scurve_lc(last_u, this_u);
				break;
			default:
				THROW_FILE_LINE("unknown unit type");
				break;
			}
			break;
		case Node::UnitType::Circle3:
			switch (this_u.type_) {
			case Node::UnitType::Line3:
				make_zone_and_scurve_cl(last_u, this_u);
				break;
			case Node::UnitType::Circle3:
				make_zone_and_scurve_cc(last_u, this_u);
				break;
			default:
				THROW_FILE_LINE("unknown unit type");
				break;
			}
			break;
		case Node::UnitType::Line2:
			switch (this_u.type_) {
			case Node::UnitType::Line2:
				make_zone_and_scurve_ll(last_u, this_u);
				break;
			case Node::UnitType::Circle2:
				make_zone_and_scurve_lc(last_u, this_u);
				break;
			default:
				THROW_FILE_LINE("unknown unit type");
				break;
			}
			break;
		case Node::UnitType::Circle2:
			switch (this_u.type_) {
			case Node::UnitType::Line2:
				make_zone_and_scurve_cl(last_u, this_u);
				break;
			case Node::UnitType::Circle2:
				make_zone_and_scurve_cc(last_u, this_u);
				break;
			default:
				THROW_FILE_LINE("unknown unit type");
				break;
			}
			break;
		case Node::UnitType::Rotate3:
			switch (this_u.type_) {
			case Node::UnitType::Rotate3:
				make_zone_and_scurve_qq(last_u, this_u);
				break;
			default:
				THROW_FILE_LINE("unknown unit type");
				break;
			}
			break;
		case Node::UnitType::Line1:
			switch (this_u.type_) {
			case Node::UnitType::Line1:
				make_zone_and_scurve_ll(last_u, this_u);
				break;
			default:
				THROW_FILE_LINE("unknown unit type");
				break;
			}
			break;
		}
	}

	// get unit data //
	auto get_zone_data(double arc, double darc, double d2arc, Node::UnitType type, const Node::Zone& z, double* p, double* dp, double* d2p)->void {
		double s, ds, d2s;

		double p_[4], dp_[4], d2p_[4];

		switch (z.type_) {
		case Node::Zone::ZoneType::LL: {
			s_bezier3_arc2s(arc, darc, d2arc, z.bezier_param, s, ds, d2s);
			s_bezier3_blend_line_line(s, z.lines_.p0_, z.lines_.p1_, z.lines_.p2_, p_, dp_, d2p_);

			// d2p_dt2 = (dp_dt)' = (dp_ds * ds)' = d2p_ds2 * ds^2 + dp_ds * d2s
			aris::dynamic::s_nv(3, ds * ds, d2p_);
			aris::dynamic::s_va(3, d2s, dp_, d2p_);

			// dp_dt = dp_ds * ds
			aris::dynamic::s_nv(3, ds, dp_);
			break;
		}
		case Node::Zone::ZoneType::LC: {
			s_bezier3_arc2s(arc, darc, d2arc, z.bezier_param, s, ds, d2s);
			s_bezier3_blend_line_circle(s, z.line_circle_.p0_, z.line_circle_.p1_, z.line_circle_.center_, z.line_circle_.axis_, z.line_circle_.theta_
				, p_, dp_, d2p_);
			// d2p_dt2 = (dp_dt)' = (dp_ds * ds)' = d2p_ds2 * ds^2 + dp_ds * d2s
			aris::dynamic::s_nv(3, ds * ds, d2p_);
			aris::dynamic::s_va(3, d2s, dp_, d2p_);

			// dp_dt = dp_ds * ds
			aris::dynamic::s_nv(3, ds, dp_);
			break;
		}
		case Node::Zone::ZoneType::CL: {
			s_bezier3_arc2s(z.length_ - arc, -darc, -d2arc, z.bezier_param, s, ds, d2s);
			s_bezier3_blend_line_circle(s, z.circle_line_.p2_, z.circle_line_.p1_, z.circle_line_.center_, z.circle_line_.axis_, z.circle_line_.theta_
				, p_, dp_, d2p_);
			// d2p_dt2 = (dp_dt)' = (dp_ds * ds)' = d2p_ds2 * ds^2 + dp_ds * d2s
			aris::dynamic::s_nv(3, ds * ds, d2p_);
			aris::dynamic::s_va(3, d2s, dp_, d2p_);

			// dp_dt = dp_ds * ds
			aris::dynamic::s_nv(3, ds, dp_);
			break;
		}
		case Node::Zone::ZoneType::CC: {
			s_bezier3_arc2s(arc, darc, d2arc, z.bezier_param, s, ds, d2s);
			s_bezier3_blend_circle_circle(s, z.circles_.pcenter_, z.circles_.c1_, z.circles_.a1_, z.circles_.theta1_
				, z.circles_.c2_, z.circles_.a2_, z.circles_.theta2_
				, p_, dp_, d2p_);
			// d2p_dt2 = (dp_dt)' = (dp_ds * ds)' = d2p_ds2 * ds^2 + dp_ds * d2s
			aris::dynamic::s_nv(3, ds * ds, d2p_);
			aris::dynamic::s_va(3, d2s, dp_, d2p_);

			// dp_dt = dp_ds * ds
			aris::dynamic::s_nv(3, ds, dp_);
			break;
		}
		case Node::Zone::ZoneType::QQ: {
			s_bezier3_arc2s(arc, darc, d2arc, z.bezier_param, s, ds, d2s);
			s_bezier3_blend_quaternion(s, z.quaternions_.q0_, z.quaternions_.q1_, z.quaternions_.q2_, p_, dp_, d2p_);
			// d2p_dt2 = (dp_dt)' = (dp_ds * ds)' = d2p_ds2 * ds^2 + dp_ds * d2s
			aris::dynamic::s_nv(4, ds * ds, d2p_);
			aris::dynamic::s_va(4, d2s, dp_, d2p_);

			// dp_dt = dp_ds * ds
			aris::dynamic::s_nv(4, ds, dp_);
			break;
		}
		default:
			break;
		}

		aris::Size data_size = 0;
		switch (type)
		{
		case aris::plan::Node::UnitType::Line3:
			data_size = 3;
			break;
		case aris::plan::Node::UnitType::Line2:
			data_size = 2;
			break;
		case aris::plan::Node::UnitType::Line1:
			data_size = 1;
			break;
		case aris::plan::Node::UnitType::Circle3:
			data_size = 3;
			break;
		case aris::plan::Node::UnitType::Circle2:
			data_size = 2;
			break;
		case aris::plan::Node::UnitType::Rotate3:
			data_size = 4;
			break;
		default:
			break;
		}

		aris::dynamic::s_vc(data_size, p_, p);
		aris::dynamic::s_vc(data_size, dp_, dp);
		aris::dynamic::s_vc(data_size, d2p_, d2p);
	}
	auto get_move_data(double arc, double darc, double d2arc, Node::UnitType type, const Node::Move& m, double* p, double* dp, double* d2p) -> void {
		switch (type) {
		case Node::UnitType::Line3: {
			s_compute_line3_at(m.line_.p0_, m.line_.dir_, arc, p, darc, dp, d2arc, d2p);
			break;
		}
		case Node::UnitType::Line2: {
			s_compute_line2_at(m.line_.p0_, m.line_.dir_, arc, p, darc, dp, d2arc, d2p);
			break;
		}
		case Node::UnitType::Circle3: {
			s_compute_circle3_at(m.circle_.p0_, m.circle_.center_, m.circle_.axis_, m.circle_.radius_, m.length_,
				arc, p, darc, dp, d2arc, d2p);
			break;
		}
		case Node::UnitType::Circle2: {
			s_compute_circle2_at(m.circle_.p0_, m.circle_.center_, m.circle_.axis_, m.circle_.radius_, m.length_,
				arc, p, darc, dp, d2arc, d2p);
			break;
		}
		case Node::UnitType::Line1: {
			double p_[3], v_[3], a_[3];
			s_compute_line3_at(m.line_.p0_, m.line_.dir_, arc, p_, darc, v_, d2arc, a_);
			p[0] = p_[0];
			dp[0] = v_[0];
			d2p[0] = a_[0];
			break;
		}
		case Node::UnitType::Rotate3: {
			s_compute_quaternion_at(m.quaternion_.q0_, m.quaternion_.q1_, m.length_, arc, p, darc, dp, d2arc, d2p);
			break;
		}
		default:
			break;
		}
	}
	auto get_unit_data(double arc, const Node::Unit& u, double* p, double* dp_darc, double* d2p_darc2)->void {
		LargeNum sp;
		double sv, sa, sj;
		s_scurve_at(u.scurve_, arc, &sp, &sv, &sa, &sj);
		double l = sp - u.scurve_.pa_;
		if (l < u.zone1_.length_ / 2.0) {
			get_zone_data(l + u.zone1_.length_ / 2.0, sv, sa, u.type_, u.zone1_, p, dp_darc, d2p_darc2);
		}
		else if (l > u.zone1_.length_ / 2.0 + u.move_.length_) {
			get_zone_data(l - u.zone1_.length_ / 2.0 - u.move_.length_, sv, sa, u.type_, u.zone2_, p, dp_darc, d2p_darc2);
		}
		else {
			get_move_data(l - u.zone1_.length_ / 2.0, sv, sa, u.type_, u.move_, p, dp_darc, d2p_darc2);
		}
	}

	// make nodes //
	// 创建节点
	auto create_node(Node* node, aris::Size ee_num, aris::dynamic::PosType* ee_types,
		Node::NodeType node_type, const double* ee_pos, const double* mid_pos, const double* vel, const double* acc, const double* jerk, const double* zone)
	{
		node->type_ = node_type;
		
		// 更新本段轨迹的 move //
		for (Size i{ 0 }, pos_idx{ 0 }, vel_idx{ 0 }; i < ee_num; ++i) {
			auto this_p = &node->ee_plans_[i];

			auto pos_len = aris::dynamic::s_pos_type_size(ee_types[i]);
			auto vel_len = aris::dynamic::s_pos_type_mag_size(ee_types[i]);

			this_p->data_.pos_type_ = ee_types[i];
			std::copy_n(ee_pos + pos_idx, pos_len, this_p->data_.ee_pos_);
			std::copy_n(mid_pos + pos_idx, pos_len, this_p->data_.mid_pos_);
			std::copy_n(vel + vel_idx, vel_len, this_p->data_.v_);
			std::copy_n(acc + vel_idx, vel_len, this_p->data_.a_);
			std::copy_n(jerk + vel_idx, vel_len, this_p->data_.j_);
			std::copy_n(zone + vel_idx, vel_len, this_p->data_.zone_);

			pos_idx += aris::dynamic::s_pos_type_size(ee_types[i]);
			vel_idx += aris::dynamic::s_pos_type_mag_size(ee_types[i]);
		}
	}
	// 根据前一个节点的数据，初始化当前节点起始位置 //
	auto init_node(Node* this_node, const Node* last_node)->void {
		// 更新本段轨迹的 move //
		for (Size i{ 0 }, pos_idx{ 0 }, vel_idx{ 0 }; i < this_node->ee_plans_.size(); ++i) {
			auto this_p = &this_node->ee_plans_[i];

			auto ee_type = this_p->data_.pos_type_;

			auto ee_xyz = this_p->data_.ee_pos_;
			auto mid_xyz = this_p->data_.mid_pos_;
			auto v_xyz = this_p->data_.v_;
			auto a_xyz = this_p->data_.a_;
			auto j_xyz = this_p->data_.j_;
			auto z_xyz = this_p->data_.zone_;

			auto ee_abc = this_p->data_.ee_pos_ + aris::dynamic::s_pos_type_mov_dim(ee_type);
			auto mid_abc = this_p->data_.mid_pos_ + aris::dynamic::s_pos_type_mov_dim(ee_type);
			auto v_abc = this_p->data_.v_ + (aris::dynamic::s_pos_type_mov_dim(ee_type) > 0 ? 1 : 0);
			auto a_abc = this_p->data_.a_ + (aris::dynamic::s_pos_type_mov_dim(ee_type) > 0 ? 1 : 0);
			auto j_abc = this_p->data_.j_ + (aris::dynamic::s_pos_type_mov_dim(ee_type) > 0 ? 1 : 0);
			auto z_abc = this_p->data_.zone_ + (aris::dynamic::s_pos_type_mov_dim(ee_type) > 0 ? 1 : 0);

			switch (this_node->type_) {
			case aris::plan::Node::NodeType::ResetInitPos: {
				// init //
				if (aris::dynamic::s_pos_type_mov_dim(ee_type) == 3) {
					init_unit(Node::UnitType::Line3, ee_xyz, mid_xyz, ee_xyz, *v_xyz, *a_xyz, *j_xyz, *z_xyz, this_p->x_);
				}
				else if (aris::dynamic::s_pos_type_mov_dim(ee_type) == 2) {
					init_unit(Node::UnitType::Line2, ee_xyz, mid_xyz, ee_xyz, *v_xyz, *a_xyz, *j_xyz, *z_xyz, this_p->x_);
				}
				else if (aris::dynamic::s_pos_type_mov_dim(ee_type) == 1) {
					init_unit(Node::UnitType::Line1, ee_xyz, mid_xyz, ee_xyz, *v_xyz, *a_xyz, *j_xyz, *z_xyz, this_p->x_);
				}

				if (aris::dynamic::s_pos_type_rot_dim(ee_type) == 3) {
					init_unit(Node::UnitType::Rotate3, ee_abc, mid_abc, ee_abc, *v_abc, *a_abc, *j_abc, *z_abc, this_p->a_);
				}
				else if (aris::dynamic::s_pos_type_rot_dim(ee_type) == 1) {
					init_unit(Node::UnitType::Line1, ee_abc, mid_abc, ee_abc, *v_abc, *a_abc, *j_abc, *z_abc, this_p->a_);
				}

				
				break;
			}
			case aris::plan::Node::NodeType::Line: {
				auto last_p = &last_node->ee_plans_[i];
				double p_end[4];
				
				// line //
				if (aris::dynamic::s_pos_type_mov_dim(ee_type) == 3) {
					// xyz //
					s_compute_data_at_end(last_p->x_, p_end);
					init_unit(Node::UnitType::Line3, p_end, mid_xyz, ee_xyz, *v_xyz, *a_xyz, *j_xyz, *z_xyz, this_p->x_);
				}
				else if (aris::dynamic::s_pos_type_mov_dim(ee_type) == 2) {
					// xyz //
					s_compute_data_at_end(last_p->x_, p_end);
					init_unit(Node::UnitType::Line2, p_end, mid_xyz, ee_xyz, *v_xyz, *a_xyz, *j_xyz, *z_xyz, this_p->x_);
				}
				else if (aris::dynamic::s_pos_type_mov_dim(ee_type) == 1) {
					// xyz //
					s_compute_data_at_end(last_p->x_, p_end);
					init_unit(Node::UnitType::Line1, p_end, mid_xyz, ee_xyz, *v_xyz, *a_xyz, *j_xyz, *z_xyz, this_p->x_);
				}

				if (aris::dynamic::s_pos_type_rot_dim(ee_type) == 3) {
					// abc //
					s_compute_data_at_end(last_p->a_, p_end);
					init_unit(Node::UnitType::Rotate3, p_end, mid_abc, ee_abc, *v_abc, *a_abc, *j_abc, *z_abc, this_p->a_);
				}
				else if (aris::dynamic::s_pos_type_rot_dim(ee_type) == 1) {
					// abc //
					s_compute_data_at_end(last_p->a_, p_end);
					init_unit(Node::UnitType::Line1, p_end, mid_abc, ee_abc, *v_abc, *a_abc, *j_abc, *z_abc, this_p->a_);
				}

				break;
			}
			case aris::plan::Node::NodeType::Circle: {
				auto last_p = &last_node->ee_plans_[i];
				double p_end[4];
				
				// circle //
				if (aris::dynamic::s_pos_type_mov_dim(ee_type) == 3) {
					// xyz //
					s_compute_data_at_end(last_p->x_, p_end);
					init_unit(Node::UnitType::Circle3, p_end, mid_xyz, ee_xyz, *v_xyz, *a_xyz, *j_xyz, *z_xyz, this_p->x_);
				}
				else if (aris::dynamic::s_pos_type_mov_dim(ee_type) == 2) {
					// xyz //
					s_compute_data_at_end(last_p->x_, p_end);
					init_unit(Node::UnitType::Circle2, p_end, mid_xyz, ee_xyz, *v_xyz, *a_xyz, *j_xyz, *z_xyz, this_p->x_);
				}
				else if (aris::dynamic::s_pos_type_mov_dim(ee_type) == 1) {
					// xyz //
					s_compute_data_at_end(last_p->x_, p_end);
					init_unit(Node::UnitType::Line1, p_end, mid_xyz, ee_xyz, *v_xyz, *a_xyz, *j_xyz, *z_xyz, this_p->x_);
				}

				if (aris::dynamic::s_pos_type_rot_dim(ee_type) == 3) {
					// abc //
					s_compute_data_at_end(last_p->a_, p_end);
					init_unit(Node::UnitType::Rotate3, p_end, mid_abc, ee_abc, *v_abc, *a_abc, *j_abc, *z_abc, this_p->a_);
				}
				else if (aris::dynamic::s_pos_type_rot_dim(ee_type) == 1) {
					// abc //
					s_compute_data_at_end(last_p->a_, p_end);
					init_unit(Node::UnitType::Line1, p_end, mid_abc, ee_abc, *v_abc, *a_abc, *j_abc, *z_abc, this_p->a_);
				}

				break;
			}
			}

			pos_idx += aris::dynamic::s_pos_type_size(ee_type);
			vel_idx += aris::dynamic::s_pos_type_mag_size(ee_type);
		}
	}
	// 连接节点，创造转弯区等 //
	auto connect_nodes(Node* this_node, Node* last_node, aris::Size ee_num, aris::dynamic::PosType* ee_types, bool if_make_zone)->void {
		if(this_node->type_ == Node::NodeType::ResetInitPos || last_node->type_ == Node::NodeType::ResetInitPos) {
			if_make_zone = false;
		}
		
		// 更新本段轨迹的 move //
		for (Size i{ 0 }; i < ee_num; ++i) {
			auto this_p = &this_node->ee_plans_[i];
			auto last_p = &last_node->ee_plans_[i];

			if(if_make_zone){
				if (aris::dynamic::s_pos_type_mov_dim(ee_types[i]) > 0) {
					make_zone_and_scurve(last_p->x_, this_p->x_);
				}
				if (aris::dynamic::s_pos_type_rot_dim(ee_types[i]) > 0) {
					make_zone_and_scurve(last_p->a_, this_p->a_);
				}
			}
			else{
				// 无需做转弯区，直接拿上次的 pb 做数据
				if (aris::dynamic::s_pos_type_mov_dim(ee_types[i]) > 0) {
					// 仅仅 init 之后，节点的 pa = 0
					this_p->x_.scurve_.pa_ = last_p->x_.scurve_.pb_;
					this_p->x_.scurve_.pb_ = this_p->x_.scurve_.pa_ + this_p->x_.move_.length_ + this_p->x_.zone1_.length_ / 2.0 + this_p->x_.zone2_.length_ / 2.0;
				}
				if (aris::dynamic::s_pos_type_rot_dim(ee_types[i]) > 0) {
					// 仅仅 init 之后，节点的 pa = 0
					this_p->a_.scurve_.pa_ = last_p->a_.scurve_.pb_;
					this_p->a_.scurve_.pb_ = this_p->a_.scurve_.pa_ + this_p->a_.move_.length_ + this_p->a_.zone1_.length_ / 2.0 + this_p->a_.zone2_.length_ / 2.0;
				}
			}
			
		}

	}
	// 重规划节点 //
	auto replan_nodes(const std::vector<aris::dynamic::PosType> &ee_types, std::list<Node>::iterator last, std::list<Node>::iterator begin, std::list<Node>::iterator end)->int {
		
		// 对区间的节点进行规划，规划的结果会直接更新到节点中 //
		auto replan_local = [](const std::vector<aris::dynamic::PosType> &ee_types, std::list<Node>::iterator last, std::list<Node>::iterator begin, std::list<Node>::iterator end)->int {

			// 更新起始节点的时间 //
			for (int i = 0; i < ee_types.size();++i) {
				if(begin->type_ == Node::NodeType::ResetInitPos){
					begin->ee_plans_[i].x_.scurve_.t0_ = 0;
					begin->ee_plans_[i].a_.scurve_.t0_ = 0;
					begin->ee_plans_[i].x_.scurve_.T_ = 0;
					begin->ee_plans_[i].a_.scurve_.T_ = 0;
				}
				else{
					begin->ee_plans_[i].x_.scurve_.t0_ = last->ee_plans_[i].x_.scurve_.t0_ + last->ee_plans_[i].x_.scurve_.T_;
					begin->ee_plans_[i].a_.scurve_.t0_ = last->ee_plans_[i].a_.scurve_.t0_ + last->ee_plans_[i].a_.scurve_.T_;
				}
			}
			
			// 构造 scurve list //
			std::list<SCurveNode> ins_scurve_list;//ins_scurve_origin_list
			for (auto iter = begin; iter != end; ++iter) {
				if(iter->type_ == Node::NodeType::ResetInitPos)
					continue;

				auto scurve_size = static_cast<int>(aris::dynamic::s_pos_type_mag_size(ee_types.size(), ee_types.data()));
				ins_scurve_list.push_back(SCurveNode{});
				auto& scurve_node = ins_scurve_list.back();
				scurve_node.params_.reserve(scurve_size);

				for (int i = 0; i < ee_types.size();++i) {
					// x //
					if (aris::dynamic::s_pos_type_mov_dim(ee_types[i]) > 0) {
						scurve_node.params_.push_back(iter->ee_plans_[i].x_.scurve_);
					}
					// a //
					if (aris::dynamic::s_pos_type_rot_dim(ee_types[i]) > 0) {
						scurve_node.params_.push_back(iter->ee_plans_[i].a_.scurve_);
					}
				}
			}

			// 对 scurve 进行规划 //
			if (s_scurve_make_nodes(ins_scurve_list.begin(), ins_scurve_list.end()) != 0) {
				//std::cout << "[debug failed] : make scurve error" << std::endl;
				return -1;
			}

			// 将规划好的 scurve 返回到 nodes 中的优化后的位置 //
			for (auto iter = begin; iter != end; ++iter) {
				if(iter->type_ == Node::NodeType::ResetInitPos)
					continue;

				auto& scurve_node = ins_scurve_list.front();
				iter->s_beg_ = scurve_node.params_[0].t0_;
				iter->s_end_ = scurve_node.params_[0].t0_ + scurve_node.params_[0].T_;
				for (int i = 0, s_idx = 0; i < iter->ee_plans_.size(); ++i) {
					auto& ee_p = iter->ee_plans_[i];
					
					// x //
					if (aris::dynamic::s_pos_type_mov_dim(ee_types[i]) > 0) {
						ee_p.x_.scurve_ = scurve_node.params_[s_idx];
						s_idx++;
					}
					// a //
					if (aris::dynamic::s_pos_type_rot_dim(ee_types[i]) > 0) {
						ee_p.a_.scurve_ = scurve_node.params_[s_idx];
						s_idx++;
					}
				}
				ins_scurve_list.pop_front();
			}


			return 0;
		};
		
		// 设置所有节点的 next_node_ 指针 //
		for (auto iter = begin; iter != end; ++iter) {
			iter->next_node_.store(std::next(iter) == end ? &*iter : &*std::next(iter));
		}

		// 根据 ResetInitPos 分段 //
		for(auto local_beg = begin, local_last = last; local_beg != end;) {
			auto local_end = std::find_if(std::next(local_beg), end, [](auto& node)->bool {
				return node.type_ == Node::NodeType::ResetInitPos;
			});

			auto ret = replan_local(ee_types, local_last, local_beg, local_end);
			if(ret != 0)
				return ret;

			local_beg = local_end;
			local_last = std::prev(local_end);
		}

		return 0;
	}
	// 获取节点数据 //
	auto get_node_data(aris::Size ee_num, const aris::dynamic::PosType* ee_types, const Node* current_node, LargeNum s, double ds, double dds, double ddds,
		double* internal_pos, double* internal_vel, double* internal_acc) -> void
	{
		int idx = 0;
		for (int i = 0; i < ee_num; ++i) {
			auto& ee_p = current_node->ee_plans_[i];

			// Line3 Line2 Line1 //
			if (aris::dynamic::s_pos_type_mov_dim(ee_types[i]) > 0) {
				// x //
				get_unit_data(s, ee_p.x_, internal_pos + idx, internal_vel + idx, internal_acc + idx);
				idx += aris::dynamic::s_pos_type_mov_dim(ee_types[i]);
			}

			// Rot3 Rot1 //
			if (aris::dynamic::s_pos_type_rot_dim(ee_types[i]) == 3) {
				// a //
				get_unit_data(s, ee_p.a_, internal_pos + idx, internal_vel + idx, internal_acc + idx);
				idx += 4;
			}
			else if (aris::dynamic::s_pos_type_rot_dim(ee_types[i]) == 1) {
				// a //
				get_unit_data(s, ee_p.a_, internal_pos + idx, internal_vel + idx, internal_acc + idx);
				idx += 1;
			}
		}
	}

	// 关于 tg 的并发：
	//
	// tg 中包含一系列 node ：
	// 
	//             next node              
	// 
	//   0      o      NULL    【begin】
	//                  
	//   1      o      NULL    
	// 
	//   ...    ...    ...
	//                   
	// 	 m-1    o      NULL   
	// 
	//   m      o      m+1     【current】   
	//          |
	//   m+1    o      m+2
	//          |
	//   m+2    o      m+3
	// 
	//   ...    ...    ...
	//
	//   n-1    o      n        
	//          |
	//   n      o      n       【end】
	struct TrajectoryGenerator::Imp {
		// 时间参数 //
		double dt_{ 0.001 };
		LargeNum s_{ 0.0 };
		double ds_{ 1.0 }, dds_{ 0.0 }, ddds_{ 0.0 };
		double max_ds_{ 1.0 }, max_dds_{ 10.0 }, max_ddds_{ 100.0 };
		std::atomic<double> target_ds_{ 1.0 };

		// 末端类型 //
		std::vector<aris::dynamic::PosType> ee_pos_types_;
		std::vector<aris::dynamic::VelType> ee_vel_types_;
		std::vector<aris::dynamic::AccType> ee_acc_types_;

		aris::Size ee_size_{0}, internal_pos_size{ 0 };
		double* internal_pos_{ nullptr }, * internal_vel_{ nullptr }, * internal_acc_{ nullptr };
		aris::dynamic::PosType* internal_pos_type_{ nullptr }, *out_pos_type_{ nullptr };
		aris::dynamic::VelType* internal_vel_type_{ nullptr }, *out_vel_type_{ nullptr };
		aris::dynamic::AccType* internal_acc_type_{ nullptr }, *out_acc_type_{ nullptr };
		std::vector<char> mem_pool_;

		// 规划节点 //
		int max_replan_num_{ 10 };
		std::list<Node> nodes_;
		std::atomic<Node*> current_node_;

		// 互斥区，保护访问
		std::recursive_mutex mu_;

		auto update_insert()->void {
			// init current_iter //
			auto current_node = current_node_.load();
			auto current_iter = std::find_if(nodes_.begin(), nodes_.end(), [current_node](auto& node)->bool {
					return &node == current_node;
				});

			// find inserted node //
			auto ins_iter = nodes_.end();
			for (auto iter = std::next(current_iter); iter != nodes_.end(); ++iter) {
				if (std::prev(iter)->next_node_.load() == &*std::prev(iter)) {
					ins_iter = iter;
					break;
				}
			}
			// CaseInsertNoPendingNode: 没有待发布的新插入节点，直接返回。
			if(ins_iter == nodes_.end())
				return;

			// 制作插入节点序列的转弯区域，后面只连接，不更新 //
			init_node(&*ins_iter, &*std::prev(ins_iter));
			for(auto iter = std::next(ins_iter); iter != nodes_.end(); ++iter) {
				init_node(&*iter, &*std::prev(iter));
				connect_nodes(&*iter, &*std::prev(iter), ee_size_, ee_pos_types_.data(), true);
			}

			Node ins_node_copy = *ins_iter;

			// loop insert
			bool insert_success = false;
			do {
				// 获得需要重新规划的区间：从 current 的下一个节点到新插入节点 
				current_node = current_node_.load();
				current_iter = std::find_if(nodes_.begin(), nodes_.end(), [current_node](auto& node)->bool {
					return &node == current_node;
				});

				// 找到需要重规划的节点，并插到新插入节点前，但数量过多时，限制重规划的起始节点以控制重规划的计算量 //
				auto replan_iter_begin = std::next(current_iter);
				auto replan_iter_end = replan_iter_begin;
				aris::Size replan_num = 0;
				for (; replan_iter_end != ins_iter; ++replan_iter_end) {
					if (replan_iter_end->type_ == Node::NodeType::ResetInitPos) {
						replan_iter_begin = std::next(replan_iter_end);
						replan_num = 0;
					}
					else if (replan_num > max_replan_num_)
						replan_iter_begin++;
					else
						replan_num++;
				}
				replan_iter_end = nodes_.insert(ins_iter, replan_iter_begin, replan_iter_end);

				// connect nodes, 配置转弯区 //
				connect_nodes(&*ins_iter, &*std::prev(ins_iter), ee_size_, ee_pos_types_.data(), replan_num > 0);
				for(auto iter = std::next(ins_iter); iter != nodes_.end(); ++iter) {
					connect_nodes(&*iter, &*std::prev(iter), ee_size_, ee_pos_types_.data(), false);
				}

				// 重规划 scurve
				auto replan_ret = replan_nodes(ee_pos_types_, std::prev(replan_iter_begin), replan_iter_end, nodes_.end());

#ifdef ARIS_BUILD_TESTS
				auto forced_fail_current_id = g_trajectory_force_replan_fail_current_id.load(std::memory_order_relaxed);
				auto current_iter_id = current_iter == nodes_.end() ? static_cast<std::int64_t>(-1) : current_iter->id_;
				if (forced_fail_current_id >= 0 && forced_fail_current_id == current_iter_id) {
					replan_ret = -1;
				}
#endif

				// CaseInsertReplanFailedFallback:
				// replan 失败，本轮不再尝试替换旧链路，回退为“仅追加新节点”的保守路径。
				if (replan_ret != 0) {
#ifdef ARIS_BUILD_TESTS
					emit_trajectory_concurrency_test_hook(kTrajectoryHookCaseInsertReplanFailedFallback, current_iter->id_, ins_iter->id_, false);
#endif
					nodes_.erase(replan_iter_end, ins_iter);
#ifdef ARIS_BUILD_TESTS
					emit_trajectory_concurrency_test_hook(kTrajectoryHookCaseInsertPublishExchangeConflictRetry, current_iter->id_, ins_iter->id_, false);
#endif
					*ins_iter = ins_node_copy;
					replan_nodes(ee_pos_types_, std::prev(ins_iter), ins_iter, nodes_.end());
					std::prev(ins_iter)->next_node_.exchange(&*ins_iter);
					insert_success = true;
				}
				else {
					// 重规划成功后尝试 publish 新链路；若 exchange 失败则进入并发冲突回退分支。
#ifdef ARIS_BUILD_TESTS
					emit_trajectory_concurrency_test_hook(kTrajectoryHookCaseInsertPublishExchangeConflictRetry, current_iter->id_, ins_iter->id_, false);
#endif
					Node *exchanged_node{ std::prev(replan_iter_begin)->next_node_.exchange(&*replan_iter_end) };
#ifdef ARIS_BUILD_TESTS
					auto forced_conflict_current_id = g_trajectory_force_publish_conflict_current_id.load(std::memory_order_relaxed);
					auto forced_conflict_times = g_trajectory_force_publish_conflict_times.load(std::memory_order_relaxed);
					if (forced_conflict_times > 0 && forced_conflict_current_id == current_iter->id_) {
						g_trajectory_force_publish_conflict_times.fetch_sub(1, std::memory_order_relaxed);
						exchanged_node = nullptr;
					}
					emit_trajectory_concurrency_test_hook(kTrajectoryHookCaseInsertPublishExchangeConflictRetryResult, current_iter->id_, ins_iter->id_, exchanged_node != nullptr);
#endif
					insert_success = exchanged_node != nullptr || replan_num == 0;
					if (insert_success) {
						nodes_.erase(replan_iter_begin, replan_iter_end);
					}
					else {
						// CaseInsertPublishExchangeConflictRetry:
						// exchange 返回 nullptr，说明运行线程刚切换过，当前发布失效。
						// 回滚临时重规划内容，下一轮基于最新 current_node 重试。
						nodes_.erase(replan_iter_end, ins_iter);
#ifdef ARIS_BUILD_TESTS
						emit_trajectory_concurrency_test_hook(kTrajectoryHookCaseInsertPublishExchangeConflictRetry, current_iter->id_, ins_iter->id_, false);
#endif
						*ins_iter = ins_node_copy;
					}
				}
			} while (!insert_success);
		}

		auto insert_node(Node::NodeType move_type, std::int64_t id, const double* ee_pos, const double* mid_pos, const double* vel, const double* acc, const double* jerk, const double* zone)->void {
			std::lock_guard<std::recursive_mutex> lck(mu_);

			// insert_node 仅在队尾插入并初始化最新节点
			std::vector<double> ee_pos_internal(internal_pos_size), mid_pos_internal(internal_pos_size);

			aris::dynamic::s_pos2pos(ee_pos_types_.size(), ee_pos_types_.data(), ee_pos, internal_pos_type_, ee_pos_internal.data());
			aris::dynamic::s_pos2pos(ee_pos_types_.size(), ee_pos_types_.data(), mid_pos, internal_pos_type_, mid_pos_internal.data());

			auto& ins_node = nodes_.emplace_back(ee_pos_types_.size());
			ins_node.id_ = id;

			create_node(&ins_node, ee_size_, internal_pos_type_, move_type,
				ee_pos_internal.data(), mid_pos_internal.data(), vel, acc, jerk, zone);
		}
	};
	auto TrajectoryGenerator::posTypes()const-> const std::vector<aris::dynamic::PosType>& {
		return imp_->ee_pos_types_;
	}
	auto TrajectoryGenerator::setPosTypes(const std::vector<aris::dynamic::PosType>& ee_types)->void {
		imp_->ee_pos_types_ = ee_types;
		this->allocateMemory();
	}
	auto TrajectoryGenerator::velTypes()const -> const std::vector<aris::dynamic::VelType>& {
		return imp_->ee_vel_types_;
	}
	auto TrajectoryGenerator::setVelTypes(const std::vector<aris::dynamic::VelType>& ee_types) -> void {
		imp_->ee_vel_types_ = ee_types;
		this->allocateMemory();
	}
	auto TrajectoryGenerator::accTypes()const -> const std::vector<aris::dynamic::AccType>& {
		return imp_->ee_acc_types_;
	}
	auto TrajectoryGenerator::setAccTypes(const std::vector<aris::dynamic::AccType>& ee_types) -> void {
		imp_->ee_acc_types_ = ee_types;
		this->allocateMemory();
	}
	auto TrajectoryGenerator::maxReplanNum()const->int {
		return imp_->max_replan_num_;
	}
	auto TrajectoryGenerator::setMaxReplanNum(int max_replan_num) -> void {
		imp_->max_replan_num_ = max_replan_num;
	}
	auto TrajectoryGenerator::dt()const->double {
		return imp_->dt_;
	}
	auto TrajectoryGenerator::setDt(double dt)->void {
		imp_->dt_ = dt;
	}
	auto TrajectoryGenerator::currentDs()const->double {
		return imp_->ds_;
	}
	auto TrajectoryGenerator::setCurrentDs(double ds)->void {
		imp_->ds_ = ds;
	}
	auto TrajectoryGenerator::targetDs()const->double {
		return imp_->target_ds_;
	}
	auto TrajectoryGenerator::setTargetDs(double ds)->void {
		imp_->target_ds_ = ds;
	}
	auto TrajectoryGenerator::currentDds()const->double {
		return imp_->dds_;
	}
	auto TrajectoryGenerator::setCurrentDds(double dds)->void {
		imp_->dds_ = dds;
	}
	auto TrajectoryGenerator::maxDds()const->double {
		return imp_->max_dds_;
	}
	auto TrajectoryGenerator::setMaxDds(double max_dds)->void {
		imp_->max_dds_ = max_dds;
	}
	auto TrajectoryGenerator::maxDdds()const->double {
		return imp_->max_ddds_;
	}
	auto TrajectoryGenerator::setMaxDdds(double max_ddds)->void {
		imp_->max_ddds_ = max_ddds;
	}
	auto TrajectoryGenerator::leftNodeS()const->double {
		auto current_node = imp_->current_node_.load();
		return current_node->s_end_ - imp_->s_;
	}
	auto TrajectoryGenerator::currentNodeDuration()const->double {
		auto current_node = imp_->current_node_.load();
		return current_node->s_end_ - current_node->s_beg_;
	}
	auto TrajectoryGenerator::isCurrentNodeMove()const->bool {
		auto current_node = imp_->current_node_.load();
		return current_node->type_ != Node::NodeType::ResetInitPos;
	}
	auto TrajectoryGenerator::nextMoveNodeDuration()const->double {
		auto cur = imp_->current_node_.load();
		auto node = cur->next_node_.load();
		while (node && node != cur) {
			if (node->type_ != Node::NodeType::ResetInitPos)
				return node->s_end_ - node->s_beg_;
			cur = node;
			node = cur->next_node_.load();
		}
		return 0.0;
	}

	auto nodeMaxTa(const Node* node, const std::vector<aris::dynamic::PosType>& ee_types)->double {
		double max_ta = 0.0;
		for (aris::Size i = 0; i < node->ee_plans_.size(); ++i) {
			double v;
			if (aris::dynamic::s_pos_type_mov_dim(ee_types[i]) > 0) {
				v = node->ee_plans_[i].x_.scurve_.Ta_;
				if (v > 0.0) max_ta = std::max(max_ta, v);
			}
			if (aris::dynamic::s_pos_type_rot_dim(ee_types[i]) > 0) {
				v = node->ee_plans_[i].a_.scurve_.Ta_;
				if (v > 0.0) max_ta = std::max(max_ta, v);
			}
		}
		return max_ta;
	}
	auto nodeMaxTb(const Node* node, const std::vector<aris::dynamic::PosType>& ee_types)->double {
		double max_tb = 0.0;
		for (aris::Size i = 0; i < node->ee_plans_.size(); ++i) {
			double v;
			if (aris::dynamic::s_pos_type_mov_dim(ee_types[i]) > 0) {
				v = node->ee_plans_[i].x_.scurve_.Tb_;
				if (v > 0.0) max_tb = std::max(max_tb, v);
			}
			if (aris::dynamic::s_pos_type_rot_dim(ee_types[i]) > 0) {
				v = node->ee_plans_[i].a_.scurve_.Tb_;
				if (v > 0.0) max_tb = std::max(max_tb, v);
			}
		}
		return max_tb;
	}

	auto TrajectoryGenerator::currentNodeMaxTa()const->double {
		return nodeMaxTa(imp_->current_node_.load(), imp_->ee_pos_types_);
	}
	auto TrajectoryGenerator::currentNodeMaxTb()const->double {
		return nodeMaxTb(imp_->current_node_.load(), imp_->ee_pos_types_);
	}
	auto TrajectoryGenerator::nextMoveNodeMaxTa()const->double {
		auto cur = imp_->current_node_.load();
		auto node = cur->next_node_.load();
		while (node && node != cur) {
			if (node->type_ != Node::NodeType::ResetInitPos)
				return nodeMaxTa(node, imp_->ee_pos_types_);
			cur = node;
			node = cur->next_node_.load();
		}
		return 0.0;
	}
	auto TrajectoryGenerator::nextMoveNodeMaxTb()const->double {
		auto cur = imp_->current_node_.load();
		auto node = cur->next_node_.load();
		while (node && node != cur) {
			if (node->type_ != Node::NodeType::ResetInitPos)
				return nodeMaxTb(node, imp_->ee_pos_types_);
			cur = node;
			node = cur->next_node_.load();
		}
		return 0.0;
	}
	auto TrajectoryGenerator::leftTotalS()const->double {
		return imp_->nodes_.back().s_end_ - imp_->s_;
	}

	auto TrajectoryGenerator::allocateMemory() -> void {
		auto outpos_size = aris::dynamic::s_pos_type_size(imp_->ee_pos_types_.size(), imp_->ee_pos_types_.data());

		// 计算内部的类型 //
		std::vector<aris::dynamic::PosType> internal_pos_type;
		std::vector<aris::dynamic::VelType> internal_vel_type;
		std::vector<aris::dynamic::AccType> internal_acc_type;
		for (auto type : imp_->ee_pos_types_) {
			switch (type) {
			case aris::dynamic::PosType::PE121: [[fallthrough]];
			case aris::dynamic::PosType::PE123: [[fallthrough]];
			case aris::dynamic::PosType::PE131: [[fallthrough]];
			case aris::dynamic::PosType::PE132: [[fallthrough]];
			case aris::dynamic::PosType::PE212: [[fallthrough]];
			case aris::dynamic::PosType::PE213: [[fallthrough]];
			case aris::dynamic::PosType::PE231: [[fallthrough]];
			case aris::dynamic::PosType::PE232: [[fallthrough]];
			case aris::dynamic::PosType::PE312: [[fallthrough]];
			case aris::dynamic::PosType::PE313: [[fallthrough]];
			case aris::dynamic::PosType::PE321: [[fallthrough]];
			case aris::dynamic::PosType::PE323: [[fallthrough]];
			case aris::dynamic::PosType::PM: [[fallthrough]];
			case aris::dynamic::PosType::PQ:
				internal_pos_type.push_back(aris::dynamic::PosType::PQ);
				internal_vel_type.push_back(aris::dynamic::VelType::VQ);
				internal_acc_type.push_back(aris::dynamic::AccType::AQ);
				break;
			case aris::dynamic::PosType::RE121: [[fallthrough]];
			case aris::dynamic::PosType::RE123: [[fallthrough]];
			case aris::dynamic::PosType::RE131: [[fallthrough]];
			case aris::dynamic::PosType::RE132: [[fallthrough]];
			case aris::dynamic::PosType::RE212: [[fallthrough]];
			case aris::dynamic::PosType::RE213: [[fallthrough]];
			case aris::dynamic::PosType::RE231: [[fallthrough]];
			case aris::dynamic::PosType::RE232: [[fallthrough]];
			case aris::dynamic::PosType::RE312: [[fallthrough]];
			case aris::dynamic::PosType::RE313: [[fallthrough]];
			case aris::dynamic::PosType::RE321: [[fallthrough]];
			case aris::dynamic::PosType::RE323: [[fallthrough]];
			case aris::dynamic::PosType::RM: [[fallthrough]];
			case aris::dynamic::PosType::RQ:
				internal_pos_type.push_back(aris::dynamic::PosType::RQ);
				internal_vel_type.push_back(aris::dynamic::VelType::WQ);
				internal_acc_type.push_back(aris::dynamic::AccType::XQ);
				break;
			case aris::dynamic::PosType::XYZT:
				internal_pos_type.push_back(type);
				internal_vel_type.push_back(aris::dynamic::VelType::DXYZT);
				internal_acc_type.push_back(aris::dynamic::AccType::D2XYZT);
				break;
			case aris::dynamic::PosType::XYZ:
				internal_pos_type.push_back(type);
				internal_vel_type.push_back(aris::dynamic::VelType::DXYZ);
				internal_acc_type.push_back(aris::dynamic::AccType::D2XYZ);
				break;
			case aris::dynamic::PosType::XYT:
				internal_pos_type.push_back(type);
				internal_vel_type.push_back(aris::dynamic::VelType::DXYT);
				internal_acc_type.push_back(aris::dynamic::AccType::D2XYT);
				break;
			case aris::dynamic::PosType::RTZ:
				internal_pos_type.push_back(type);
				internal_vel_type.push_back(aris::dynamic::VelType::DRTZ);
				internal_acc_type.push_back(aris::dynamic::AccType::D2RTZ);
				break;
			case aris::dynamic::PosType::XY:
				internal_pos_type.push_back(type);
				internal_vel_type.push_back(aris::dynamic::VelType::DXY);
				internal_acc_type.push_back(aris::dynamic::AccType::D2XY);
				break;
			case aris::dynamic::PosType::X:
				internal_pos_type.push_back(type);
				internal_vel_type.push_back(aris::dynamic::VelType::DX);
				internal_acc_type.push_back(aris::dynamic::AccType::D2X);
				break;
			case aris::dynamic::PosType::Y:
				internal_pos_type.push_back(type);
				internal_vel_type.push_back(aris::dynamic::VelType::DY);
				internal_acc_type.push_back(aris::dynamic::AccType::D2Y);
				break;
			case aris::dynamic::PosType::Z:
				internal_pos_type.push_back(type);
				internal_vel_type.push_back(aris::dynamic::VelType::DZ);
				internal_acc_type.push_back(aris::dynamic::AccType::D2Z);
				break;
			case aris::dynamic::PosType::A:
				internal_pos_type.push_back(type);
				internal_vel_type.push_back(aris::dynamic::VelType::DA);
				internal_acc_type.push_back(aris::dynamic::AccType::D2A);
				break;
			case aris::dynamic::PosType::B:
				internal_pos_type.push_back(type);
				internal_vel_type.push_back(aris::dynamic::VelType::DB);
				internal_acc_type.push_back(aris::dynamic::AccType::D2B);
				break;
			case aris::dynamic::PosType::C:
				internal_pos_type.push_back(type); 
				internal_vel_type.push_back(aris::dynamic::VelType::DC);
				internal_acc_type.push_back(aris::dynamic::AccType::D2C);
				break;
			case aris::dynamic::PosType::UNKNOWN:
				break;
			default:
				break;
			}
		}

		imp_->internal_pos_size = aris::dynamic::s_pos_type_size(internal_pos_type.size(), internal_pos_type.data());
		imp_->ee_size_ = imp_->ee_pos_types_.size();

		aris::Size mem_pool_size{ 0 };
		core::allocMem(mem_pool_size, imp_->internal_pos_, imp_->internal_pos_size);
		core::allocMem(mem_pool_size, imp_->internal_vel_, imp_->internal_pos_size);
		core::allocMem(mem_pool_size, imp_->internal_acc_, imp_->internal_pos_size);
		core::allocMem(mem_pool_size, imp_->internal_pos_type_, imp_->ee_size_);
		core::allocMem(mem_pool_size, imp_->internal_vel_type_, imp_->ee_size_);
		core::allocMem(mem_pool_size, imp_->internal_acc_type_, imp_->ee_size_);
		core::allocMem(mem_pool_size, imp_->out_pos_type_, imp_->ee_size_);
		core::allocMem(mem_pool_size, imp_->out_vel_type_, imp_->ee_size_);
		core::allocMem(mem_pool_size, imp_->out_acc_type_, imp_->ee_size_);

		imp_->mem_pool_.resize(mem_pool_size);

		imp_->internal_pos_ = core::getMem(imp_->mem_pool_.data(), imp_->internal_pos_);
		imp_->internal_vel_ = core::getMem(imp_->mem_pool_.data(), imp_->internal_vel_);
		imp_->internal_acc_ = core::getMem(imp_->mem_pool_.data(), imp_->internal_acc_);
		imp_->internal_pos_type_ = core::getMem(imp_->mem_pool_.data(), imp_->internal_pos_type_);
		imp_->internal_vel_type_ = core::getMem(imp_->mem_pool_.data(), imp_->internal_vel_type_);
		imp_->internal_acc_type_ = core::getMem(imp_->mem_pool_.data(), imp_->internal_acc_type_);
		imp_->out_pos_type_ = core::getMem(imp_->mem_pool_.data(), imp_->out_pos_type_);
		imp_->out_vel_type_ = core::getMem(imp_->mem_pool_.data(), imp_->out_vel_type_);
		imp_->out_acc_type_ = core::getMem(imp_->mem_pool_.data(), imp_->out_acc_type_);

		// 赋值类型系统，外部的速度类型还需要确认长度与位置类型一致 //
		std::copy_n(internal_pos_type.data(), internal_pos_type.size(), imp_->internal_pos_type_);
		std::copy_n(internal_vel_type.data(), internal_vel_type.size(), imp_->internal_vel_type_);
		std::copy_n(internal_acc_type.data(), internal_acc_type.size(), imp_->internal_acc_type_);

		std::copy_n(imp_->ee_pos_types_.data(), imp_->ee_size_, imp_->out_pos_type_);
		std::copy_n(internal_pos_type.size() == imp_->ee_vel_types_.size() ? imp_->ee_vel_types_.data() : imp_->internal_vel_type_, imp_->ee_size_, imp_->out_vel_type_);
		std::copy_n(internal_pos_type.size() == imp_->ee_acc_types_.size() ? imp_->ee_acc_types_.data() : imp_->internal_acc_type_, imp_->ee_size_, imp_->out_acc_type_);
	}
	TrajectoryGenerator::~TrajectoryGenerator() = default;
	TrajectoryGenerator::TrajectoryGenerator() :imp_(new Imp) {
		imp_->current_node_.store(nullptr);
	}
	auto TrajectoryGenerator::getEePosAndMoveDt(double* ee_pos, double* ee_vel, double* ee_acc)->std::int64_t {
		auto current_node = imp_->current_node_.load();
		auto next_node = current_node->next_node_.load();

		auto target_ds = imp_->target_ds_.load();

		// 正常运行 //
		auto& s_ = imp_->s_;
		s_ = s_ + currentDs() * dt();
		aris::Size total_count;
		moveAbsolute2(imp_->ds_, imp_->dds_, imp_->ddds_, target_ds, 0.0, 0.0,
			imp_->max_dds_, imp_->max_ddds_, imp_->max_ddds_, imp_->dt_, 1e-10,
			imp_->ds_, imp_->dds_, imp_->ddds_, total_count);
		
		
		
		// 需要切换或结束
		while (current_node->s_end_ - s_ < 0.0) {
			// check 是否全局结束，即所有指令都已执行完
			if (current_node == next_node) {
				s_ = current_node->s_end_;
				get_node_data(posTypes().size(), imp_->internal_pos_type_, current_node, s_, imp_->ds_, imp_->dds_, imp_->ddds_, imp_->internal_pos_, imp_->internal_vel_, imp_->internal_acc_);
				aris::dynamic::s_pos2pos(posTypes().size(), imp_->internal_pos_type_, imp_->internal_pos_, posTypes().data(), ee_pos);
				
				if (ee_acc) {
					aris::dynamic::s_nv(imp_->internal_pos_size, imp_->ds_ * imp_->ds_, imp_->internal_acc_);
					aris::dynamic::s_va(imp_->internal_pos_size, imp_->dds_, imp_->internal_vel_, imp_->internal_acc_);
					s_acc2acc(imp_->ee_size_, imp_->internal_pos_type_, imp_->internal_pos_, imp_->internal_vel_type_, imp_->internal_vel_, imp_->internal_acc_type_, imp_->internal_acc_, imp_->out_acc_type_, ee_acc);
				}
				if (ee_vel) {
					aris::dynamic::s_nv(imp_->internal_pos_size, imp_->ds_, imp_->internal_vel_);
					s_vel2vel(imp_->ee_size_, imp_->internal_pos_type_, imp_->internal_pos_, imp_->internal_vel_type_, imp_->internal_vel_, imp_->out_vel_type_, ee_vel);
				}
				// to be removed //
				imp_->ds_ = target_ds;
				imp_->dds_ = 0.0;
				imp_->ddds_ = 0.0;
				// to be removed //

				current_node->finished_ = true;
				return 0;
			}
			// check 是否局部结束，即下一条指令是 init
			else if (current_node->type_ != Node::NodeType::ResetInitPos && next_node->type_ == Node::NodeType::ResetInitPos) {
				// 如果此前没有结束过（例如后面的ResetInitPos是新插进来的），则返回当前指令的末尾状态，否则直接切换到下一条指令 //
				if(!current_node->finished_){
					s_ = current_node->s_end_;
					get_node_data(posTypes().size(), imp_->internal_pos_type_, current_node, s_, imp_->ds_, imp_->dds_, imp_->ddds_, imp_->internal_pos_, imp_->internal_vel_, imp_->internal_acc_);
					aris::dynamic::s_pos2pos(posTypes().size(), imp_->internal_pos_type_, imp_->internal_pos_, posTypes().data(), ee_pos);
					if (ee_acc) {
						aris::dynamic::s_nv(imp_->internal_pos_size, imp_->ds_ * imp_->ds_, imp_->internal_acc_);
						aris::dynamic::s_va(imp_->internal_pos_size, imp_->dds_, imp_->internal_vel_, imp_->internal_acc_);
						s_acc2acc(imp_->ee_size_, imp_->internal_pos_type_, imp_->internal_pos_, imp_->internal_vel_type_, imp_->internal_vel_, imp_->internal_acc_type_, imp_->internal_acc_, imp_->out_acc_type_, ee_acc);
						//aris::dynamic::s_vc(imp_->internal_pos_size, imp_->internal_acc_, ee_acc);
					}
					if (ee_vel) {
						aris::dynamic::s_nv(imp_->internal_pos_size, imp_->ds_, imp_->internal_vel_);
						s_vel2vel(imp_->ee_size_, imp_->internal_pos_type_, imp_->internal_pos_, imp_->internal_vel_type_, imp_->internal_vel_, imp_->out_vel_type_, ee_vel);
						//aris::dynamic::s_vc(imp_->internal_pos_size, imp_->internal_vel_, ee_vel);
					}

					// to be removed //
					imp_->ds_ = target_ds;
					imp_->dds_ = 0.0;
					imp_->ddds_ = 0.0;
					// to be removed //

					//current_node = current_node->next_node_.exchange(nullptr);
					//imp_->current_node_.store(current_node);
					current_node->finished_ = true;
					return current_node->id_;
				}
				else{
					current_node = current_node->next_node_.exchange(nullptr);
					next_node = current_node->next_node_.load();
					imp_->current_node_.store(current_node);
				}
			}
			// check 是否仅存一条 init 指令
			else if (current_node->type_ == Node::NodeType::ResetInitPos) {
				imp_->s_ = target_ds * dt();
				imp_->ds_ = target_ds;
				imp_->dds_ = 0.0;
				imp_->ddds_ = 0.0;
				
				current_node = current_node->next_node_.exchange(nullptr);
				next_node = current_node->next_node_.load();
				imp_->current_node_.store(current_node);
			}
			// 下一条指令是运动指令，正常切换
			else {
				current_node = current_node->next_node_.exchange(nullptr);
				next_node = current_node->next_node_.load();
				imp_->current_node_.store(current_node);
			}
		}


		get_node_data(posTypes().size(), imp_->internal_pos_type_, current_node, s_, imp_->ds_, imp_->dds_, imp_->ddds_, imp_->internal_pos_, imp_->internal_vel_, imp_->internal_acc_);
		aris::dynamic::s_pos2pos(posTypes().size(), imp_->internal_pos_type_, imp_->internal_pos_, posTypes().data(), ee_pos);
		if (ee_acc) {
			aris::dynamic::s_nv(imp_->internal_pos_size, imp_->ds_ * imp_->ds_, imp_->internal_acc_);
			aris::dynamic::s_va(imp_->internal_pos_size, imp_->dds_, imp_->internal_vel_, imp_->internal_acc_);
			s_acc2acc(imp_->ee_size_, imp_->internal_pos_type_, imp_->internal_pos_, imp_->internal_vel_type_, imp_->internal_vel_, imp_->internal_acc_type_, imp_->internal_acc_, imp_->out_acc_type_, ee_acc);
			//aris::dynamic::s_vc(imp_->internal_pos_size, imp_->internal_acc_, ee_acc);
		}
		if (ee_vel) {
			aris::dynamic::s_nv(imp_->internal_pos_size, imp_->ds_, imp_->internal_vel_);
			s_vel2vel(imp_->ee_size_, imp_->internal_pos_type_, imp_->internal_pos_, imp_->internal_vel_type_, imp_->internal_vel_, imp_->out_vel_type_, ee_vel);
			//aris::dynamic::s_vc(imp_->internal_pos_size, imp_->internal_vel_, ee_vel);
		}
		
		

		return current_node->id_;
	}
	auto TrajectoryGenerator::currentNodeId()const->std::int64_t{
		return imp_->current_node_.load()->id_;
	}
    auto TrajectoryGenerator::isCurrentNodeFinished() const -> bool{
        return imp_->current_node_.load()->finished_;
    }
	auto TrajectoryGenerator::insertInitPos(std::int64_t id, const double* ee_pos)->void {
		std::lock_guard<std::recursive_mutex> lck(imp_->mu_);
		
		// auto current_node = imp_->current_node_.load();

		// // 转化 pos 表达 //
		// std::vector<double> ee_pos_internal(imp_->internal_pos_size), mid_pos_internal(imp_->internal_pos_size);
		// aris::dynamic::s_pos2pos(posTypes().size(), posTypes().data(), ee_pos, imp_->internal_pos_type_, ee_pos_internal.data());

		// // 插入初始化指令 //
		// auto& nodes_ = imp_->nodes_;
		// auto& ins_node = nodes_.emplace_back(posTypes().size());
		// ins_node.id_ = id;
		// ins_node.next_node_.store(&ins_node);

		// // 初始化节点 //
		// auto scurve_size = aris::dynamic::s_pos_type_mag_size(posTypes().size(), posTypes().data());
		// std::vector<double> vel_vec(scurve_size, 1.0), acc_vec(scurve_size, 1.0), jerk_vec(scurve_size, 1.0), zone_vec(scurve_size, 0.0);
		// make_node(&ins_node, imp_->ee_size_, imp_->internal_pos_type_, Node::NodeType::ResetInitPos, ee_pos_internal.data(), mid_pos_internal.data()
		// 	, vel_vec.data(), acc_vec.data(), jerk_vec.data(), zone_vec.data());

		auto scurve_size = aris::dynamic::s_pos_type_mag_size(posTypes().size(), posTypes().data());
		std::vector<double> vel_vec(scurve_size, 0.0), acc_vec(scurve_size, 0.0), jerk_vec(scurve_size, 0.0), zone_vec(scurve_size, 0.0);

		imp_->insert_node(Node::NodeType::ResetInitPos, id, ee_pos, ee_pos, vel_vec.data(), acc_vec.data(), jerk_vec.data(), zone_vec.data());

		// 设置当前 node 为 current_node_ 或 将此node设置为之前node的下一个值 //
		if (imp_->nodes_.size() < 2){
			// 只有当前node 或者 上次node已经运行结束
			init_node(&imp_->nodes_.back(), nullptr);
			imp_->current_node_.store(&imp_->nodes_.back());
		}
		// else
		// 	std::prev(nodes_.end(), 2)->next_node_.store(&ins_node);
	}
	auto TrajectoryGenerator::insertLinePos(std::int64_t id, const double* ee_pos, const double* vel, const double* acc, const double* jerk, const double* zone)->void{
		std::lock_guard<std::recursive_mutex> lck(imp_->mu_);

		// 如果当前指令队列为空，那么会插入ResetInitPos指令 //
		auto& nodes_ = imp_->nodes_;
		if (nodes_.empty())
			insertInitPos(id, ee_pos);

		imp_->insert_node(Node::NodeType::Line, id, ee_pos, ee_pos, vel, acc, jerk, zone);
	}
	auto TrajectoryGenerator::insertCirclePos(std::int64_t id, const double* ee_pos, const double* mid_pos, const double* vel, const double* acc, const double* jerk, const double* zone)->void {
		std::lock_guard<std::recursive_mutex> lck(imp_->mu_);

		// 如果当前指令队列为空，那么会插入ResetInitPos指令 //
		auto& nodes_ = imp_->nodes_;
		if (nodes_.empty())
			insertInitPos(id, ee_pos);

		imp_->insert_node(Node::NodeType::Circle, id, ee_pos, mid_pos, vel, acc, jerk, zone);
	}
	auto TrajectoryGenerator::updateInsertPos()->void {
		std::lock_guard<std::recursive_mutex> lck(imp_->mu_);
		if (imp_->nodes_.size() < 2) return;
		imp_->update_insert();
	}
	auto TrajectoryGenerator::clearUsedPos()->void {
		std::lock_guard<std::recursive_mutex> lck(imp_->mu_);

		auto& nodes_ = imp_->nodes_;

		auto current_node = imp_->current_node_.load();
		auto current_iter = std::find_if(nodes_.begin(), nodes_.end(), [current_node](auto& node)->bool {
			return &node == current_node;
			});

		nodes_.erase(nodes_.begin(), current_iter);
	}
	auto TrajectoryGenerator::clearAllPos()->void {
		std::lock_guard<std::recursive_mutex> lck(imp_->mu_);
		imp_->current_node_.store(nullptr);
		imp_->nodes_.clear();
	}
	auto TrajectoryGenerator::unusedPosNum()->int {
		std::lock_guard<std::recursive_mutex> lck(imp_->mu_);
		auto current_node = imp_->current_node_.load();
		auto current_iter = std::find_if(imp_->nodes_.begin(), imp_->nodes_.end(), [current_node](auto& node)->bool {
			return &node == current_node;
			});

		return std::max((int)std::distance(current_iter, imp_->nodes_.end()) - 1, 0);
	}
	auto TrajectoryGenerator::unusedNodeIds()const->std::vector<std::int64_t> {
		std::lock_guard<std::recursive_mutex> lck(imp_->mu_);

		auto current_node = imp_->current_node_.load();
		auto current_iter = std::find_if(imp_->nodes_.begin(), imp_->nodes_.end(), [current_node](auto& node)->bool {
			return &node == current_node;
			});

		std::vector<std::int64_t> id_list_;

		for (auto iter = current_iter; iter != imp_->nodes_.end(); iter++) {
			id_list_.push_back(iter->id_);
		}

		return id_list_;
	}
}
