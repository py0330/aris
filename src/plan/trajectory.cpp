#include"aris/plan/trajectory.hpp"
#include"aris/plan/function.hpp"

namespace aris::plan {
	struct Node {
		enum class MoveType {
			Line,
			Circle,
			ResetInitPos
		};
		struct Zone {
			enum class ZoneType {
				LL,
				LC,
				CL,
				CC,
				QQ,
				OO,
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
			double length_{ 0.0 }, zone_value_{ 0.0 }; // length 是融合后的弧长，zone_value是用户的zone的输入
			double a_, b_, c_, d_, e_;
			union {
				Lines lines_;
				LineCircle line_circle_;
				CircleLine circle_line_;
				Circles circles_;
				Quaternions quaternions_;
				OneDof one_dof_;
			};
		};
		struct Move {
			struct LineData {
				double p0_[3], p1_[3];
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

			double length_;  // 对于 quaternion 来说，是指角度
			union {
				LineData line_;
				CircleData circle_;
				QuternionData quaternion_;
				OneDof one_dof_;
			};
		};
		struct EePlanData {
			// 末端种类
			//aris::dynamic::EEType ee_type_{ aris::dynamic::EEType::PE123 };
			// Move 种类
			MoveType move_type_{ MoveType::Line };
			// zones
			Zone zone_x1_, zone_a1_, zone_x2_, zone_a2_; // 转弯区参数，移动、转动、开始、结束，因此有4个
			// 主运动部分
			Move move_x_, move_a_;
			// scurves
			SCurveParam scurve_x_, scurve_a_;        // s 曲线
		};

		std::int64_t            id_;
		LargeNum                s_end_;
		std::vector<EePlanData> ee_plans_;
		std::atomic<Node*>      next_node_;

		~Node() = default;
		Node(aris::Size ee_size) {
			id_ = 1;
			s_end_ = 0;
			ee_plans_.resize(ee_size);
			next_node_.store(this);
		}
		Node(const Node& other) {
			id_ = other.id_;
			s_end_ = other.s_end_;
			ee_plans_ = other.ee_plans_;
			next_node_.store(other.next_node_.load());
		}
		Node& operator=(const Node& other) {
			id_ = other.id_;
			s_end_ = other.s_end_;
			ee_plans_ = other.ee_plans_;
			next_node_.store(other.next_node_.load());
			return *this;
		}
	};

	auto internal_pos_to_outpos(const std::vector<aris::dynamic::EEType>& ee_types, const double* internal_pos, double* out_pos) {
		aris::Size internal_idx{ 0 }, out_idx{ 0 };
		for (auto ee_type : ee_types) {
			switch (ee_type) {
			case aris::dynamic::EEType::PE313: {
				aris::dynamic::s_pq2pe(internal_pos + internal_idx, out_pos + out_idx, "313");
				internal_idx += 7;
				out_idx += 6;
				break;
			}
			case aris::dynamic::EEType::PE321: {
				aris::dynamic::s_pq2pe(internal_pos + internal_idx, out_pos + out_idx, "321");
				internal_idx += 7;
				out_idx += 6;
				break;
			}
			case aris::dynamic::EEType::PE123: {
				aris::dynamic::s_pq2pe(internal_pos + internal_idx, out_pos + out_idx, "123");
				internal_idx += 7;
				out_idx += 6;
				break;
			}
			case aris::dynamic::EEType::PQ: {
				aris::dynamic::s_vc(7, internal_pos + internal_idx, out_pos + out_idx);
				internal_idx += 7;
				out_idx += 7;
				break;
			}
			case aris::dynamic::EEType::PM: {
				aris::dynamic::s_pq2pm(internal_pos + internal_idx, out_pos + out_idx);
				internal_idx += 7;
				out_idx += 16;
				break;
			}
			case aris::dynamic::EEType::XYZT: {
				aris::dynamic::s_vc(4, internal_pos + internal_idx, out_pos + out_idx);
				internal_idx += 4;
				out_idx += 4;
				break;
			}
			case aris::dynamic::EEType::XYZ: {
				aris::dynamic::s_vc(3, internal_pos + internal_idx, out_pos + out_idx);
				internal_idx += 3;
				out_idx += 3;
				break;
			}
			case aris::dynamic::EEType::XYT: {
				aris::dynamic::s_vc(3, internal_pos + internal_idx, out_pos + out_idx);
				internal_idx += 3;
				out_idx += 3;
				break;
			}
			case aris::dynamic::EEType::XY: {
				aris::dynamic::s_vc(2, internal_pos + internal_idx, out_pos + out_idx);
				internal_idx += 2;
				out_idx += 2;
				break;
			}
			case aris::dynamic::EEType::X: {
				aris::dynamic::s_vc(1, internal_pos + internal_idx, out_pos + out_idx);
				internal_idx += 1;
				out_idx += 1;
				break;
			}
			case aris::dynamic::EEType::A: {
				aris::dynamic::s_vc(1, internal_pos + internal_idx, out_pos + out_idx);
				internal_idx += 1;
				out_idx += 1;
				break;
			}
			case aris::dynamic::EEType::UNKNOWN:
				break;
			default:
				break;
			}
		}


	}
	auto outpos_to_internal_pos(const std::vector<aris::dynamic::EEType>& ee_types, const double* out_pos, double* internal_pos) {
		aris::Size internal_idx{ 0 }, out_idx{ 0 };
		for (auto ee_type : ee_types) {
			switch (ee_type) {
			case aris::dynamic::EEType::PE313: {
				aris::dynamic::s_pe2pq(out_pos + out_idx, internal_pos + internal_idx, "313");
				internal_idx += 7;
				out_idx += 6;
				break;
			}
			case aris::dynamic::EEType::PE321: {
				aris::dynamic::s_pe2pq(out_pos + out_idx, internal_pos + internal_idx, "321");
				internal_idx += 7;
				out_idx += 6;
				break;
			}
			case aris::dynamic::EEType::PE123: {
				aris::dynamic::s_pe2pq(out_pos + out_idx, internal_pos + internal_idx, "123");
				internal_idx += 7;
				out_idx += 6;
				break;
			}
			case aris::dynamic::EEType::PQ: {
				aris::dynamic::s_vc(7, out_pos + out_idx, internal_pos + internal_idx);
				internal_idx += 7;
				out_idx += 7;
				break;
			}
			case aris::dynamic::EEType::PM: {
				aris::dynamic::s_pm2pq(out_pos + out_idx, internal_pos + internal_idx);
				internal_idx += 7;
				out_idx += 16;
				break;
			}
			case aris::dynamic::EEType::XYZT: {
				aris::dynamic::s_vc(4, out_pos + out_idx, internal_pos + internal_idx);
				internal_idx += 4;
				out_idx += 4;
				break;
			}
			case aris::dynamic::EEType::XYZ: {
				aris::dynamic::s_vc(3, out_pos + out_idx, internal_pos + internal_idx);
				internal_idx += 3;
				out_idx += 3;
				break;
			}
			case aris::dynamic::EEType::XYT: {
				aris::dynamic::s_vc(3, out_pos + out_idx, internal_pos + internal_idx);
				internal_idx += 3;
				out_idx += 3;
				break;
			}
			case aris::dynamic::EEType::XY: {
				aris::dynamic::s_vc(2, out_pos + out_idx, internal_pos + internal_idx);
				internal_idx += 2;
				out_idx += 2;
				break;
			}
			case aris::dynamic::EEType::X: {
				aris::dynamic::s_vc(1, out_pos + out_idx, internal_pos + internal_idx);
				internal_idx += 1;
				out_idx += 1;
				break;
			}
			case aris::dynamic::EEType::A: {
				aris::dynamic::s_vc(1, out_pos + out_idx, internal_pos + internal_idx);
				internal_idx += 1;
				out_idx += 1;
				break;
			}
			case aris::dynamic::EEType::UNKNOWN:
			default:
				;
			}
		}


	}

	// make & get data // 
	auto s_make_circle_data(const double* p0, const double* p1, const double* p2, double* center, double* axis, double& radius, double& length)
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

		if (std::abs(div) < 1e-10) {
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
	auto s_compute_circle_pos_at(double length_at, const double* p0, const double* center, const double* axis, double radius, double total_length, double* pos_xyz)
	{
		length_at = std::min(length_at, total_length);
		length_at = std::max(length_at, 0.0);

		double arc_at = length_at / radius;

		double rx[3]{
			p0[0] - center[0],
			p0[1] - center[1],
			p0[2] - center[2],
		};

		double ry[3];
		aris::dynamic::s_c3(axis, rx, ry);

		double s = std::sin(arc_at);
		double c = std::cos(arc_at);

		pos_xyz[0] = center[0] + s * ry[0] + c * rx[0];
		pos_xyz[1] = center[1] + s * ry[1] + c * rx[1];
		pos_xyz[2] = center[2] + s * ry[2] + c * rx[2];
	}

	auto s_make_line_data(const double* p0, const double* p1, double& length) {
		length = std::sqrt(
			(p1[0] - p0[0]) * (p1[0] - p0[0]) + (p1[1] - p0[1]) * (p1[1] - p0[1]) + (p1[2] - p0[2]) * (p1[2] - p0[2])
		);
	}
	auto s_compute_line_pos_at(double length_at, const double* p0, const double* p1, double total_length, double* pos_xyz) {
		length_at = std::min(length_at, total_length);
		length_at = std::max(length_at, 0.0);

		double ratio = total_length < std::numeric_limits<double>::epsilon() ? 0.5 : length_at / total_length;

		pos_xyz[0] = p0[0] * (1 - ratio) + p1[0] * ratio;
		pos_xyz[1] = p0[1] * (1 - ratio) + p1[1] * ratio;
		pos_xyz[2] = p0[2] * (1 - ratio) + p1[2] * ratio;
	}

	auto s_make_quaternion_data(const double* q0, const double* q1, double& length) {
		double c = aris::dynamic::s_vv(4, q0, q1);
		double q_diff[4];
		aris::dynamic::s_vc(4, -c, q0, q_diff);
		aris::dynamic::s_va(4, q1, q_diff);
			
		double s = aris::dynamic::s_norm(4, q_diff);
		
		length = 2.0 * std::atan2(s, std::abs(c));
	}
	auto s_compute_quaternion_at(double length_at, const double* q0, const double* q1, double total_length, double* q) {
		length_at = std::min(length_at, total_length);
		length_at = std::max(length_at, 0.0);

		double ratio = total_length < std::numeric_limits<double>::epsilon() ? 0.5 : length_at / total_length;

		double dir = aris::dynamic::s_vv(4, q0, q1) < 0.0 ? -1.0 : 1.0;

		if (total_length < 1e-10) {
			double s1 = (1 - ratio);
			double s2 = dir * ratio;

			q[0] = s1 * q0[0] + s2 * q1[0];
			q[1] = s1 * q0[1] + s2 * q1[1];
			q[2] = s1 * q0[2] + s2 * q1[2];
			q[3] = s1 * q0[3] + s2 * q1[3];
		}
		else {
			double s1 = std::sin((1 - ratio) * total_length / 2.0);
			double s2 = dir * std::sin(ratio * total_length / 2.0);
			double s3 = std::sin(total_length / 2.0);

			q[0] = (s1 * q0[0] + s2 * q1[0]) / s3;
			q[1] = (s1 * q0[1] + s2 * q1[1]) / s3;
			q[2] = (s1 * q0[2] + s2 * q1[2]) / s3;
			q[3] = (s1 * q0[3] + s2 * q1[3]) / s3;
		}
	}

	auto s_make_onedof_data(double p0, double p1, double& length) {
		length = p1 - p0;
	}
	auto s_compute_onedof_at(double length_at, double p0, double p1, double total_length, double* p) {
		length_at = std::min(length_at, total_length);
		length_at = std::max(length_at, 0.0);

		double ratio = total_length < std::numeric_limits<double>::epsilon() ? 0.5 : length_at / total_length;

		*p = ratio * total_length + p0;
	}

	// init ee_p //
	auto init_ee_plan_x(aris::Size dim, const double* p0, const double* p1, double vel, double acc, double jerk, double zone, Node::EePlanData& ee_p) {
		// moves //
		std::fill_n(ee_p.move_x_.line_.p0_, 3, 0.0);
		aris::dynamic::s_vc(dim, p0, ee_p.move_x_.line_.p0_);
		std::fill_n(ee_p.move_x_.line_.p1_, 3, 0.0);
		aris::dynamic::s_vc(dim, p1, ee_p.move_x_.line_.p1_);
		s_make_line_data(ee_p.move_x_.line_.p0_, ee_p.move_x_.line_.p1_, ee_p.move_x_.length_);

		// zones //
		ee_p.zone_x1_.type_ = Node::Zone::ZoneType::LL;
		ee_p.zone_x1_.zone_value_ = 0.0;
		ee_p.zone_x1_.length_ = 0.0;
		std::fill_n(ee_p.zone_x1_.lines_.p0_, 3, 0.0);
		aris::dynamic::s_vc(dim, p0, ee_p.zone_x1_.lines_.p0_);
		std::fill_n(ee_p.zone_x1_.lines_.p1_, 3, 0.0);
		aris::dynamic::s_vc(dim, p0, ee_p.zone_x1_.lines_.p1_);
		std::fill_n(ee_p.zone_x1_.lines_.p2_, 3, 0.0);
		aris::dynamic::s_vc(dim, p0, ee_p.zone_x1_.lines_.p2_);
		ee_p.zone_x2_.type_ = Node::Zone::ZoneType::LL;
		ee_p.zone_x2_.zone_value_ = zone;
		ee_p.zone_x2_.length_ = 0.0;
		std::fill_n(ee_p.zone_x2_.lines_.p0_, 3, 0.0);
		aris::dynamic::s_vc(dim, p1, ee_p.zone_x2_.lines_.p0_);
		std::fill_n(ee_p.zone_x2_.lines_.p1_, 3, 0.0);
		aris::dynamic::s_vc(dim, p1, ee_p.zone_x2_.lines_.p1_);
		std::fill_n(ee_p.zone_x2_.lines_.p2_, 3, 0.0);
		aris::dynamic::s_vc(dim, p1, ee_p.zone_x2_.lines_.p2_);

		// scurves //
		double p = ee_p.zone_x1_.length_ / 2.0 + ee_p.zone_x2_.length_ / 2.0 + ee_p.move_x_.length_;
		ee_p.scurve_x_.pa_ = 0.0;
		ee_p.scurve_x_.pb_ = p;
		ee_p.scurve_x_.va_ = 0.0;
		ee_p.scurve_x_.vc_max_ = vel;
		ee_p.scurve_x_.vb_max_ = 0.0;
		ee_p.scurve_x_.a_ = acc;
		ee_p.scurve_x_.j_ = jerk;
		ee_p.scurve_x_.t0_ = 0.0;
	}
	auto init_ee_plan_c(aris::Size dim, const double* p0, const double* p1, const double* p2, double vel, double acc, double jerk, double zone, Node::EePlanData& ee_p) {
		// moves //
		std::fill_n(ee_p.move_x_.circle_.p0_, 3, 0.0);
		aris::dynamic::s_vc(dim, p0, ee_p.move_x_.circle_.p0_);
		s_make_circle_data(p0, p1, p2, ee_p.move_x_.circle_.center_, ee_p.move_x_.circle_.axis_, ee_p.move_x_.circle_.radius_, ee_p.move_x_.length_);

		// 考虑退化 //
		if (!std::isfinite(ee_p.move_x_.circle_.radius_)) {
			ee_p.move_type_ = Node::MoveType::Line;
			init_ee_plan_x(dim, p0, p2, vel, acc, jerk, zone, ee_p);
			return;
		}

		// zones //
		ee_p.zone_x1_.type_ = Node::Zone::ZoneType::LL;
		ee_p.zone_x1_.zone_value_ = 0.0;
		ee_p.zone_x1_.length_ = 0.0;
		std::fill_n(ee_p.zone_x1_.lines_.p0_, 3, 0.0);
		aris::dynamic::s_vc(3, p0, ee_p.zone_x1_.lines_.p0_);
		std::fill_n(ee_p.zone_x1_.lines_.p1_, 3, 0.0);
		aris::dynamic::s_vc(3, p0, ee_p.zone_x1_.lines_.p1_);
		std::fill_n(ee_p.zone_x1_.lines_.p2_, 3, 0.0);
		aris::dynamic::s_vc(3, p0, ee_p.zone_x1_.lines_.p2_);
		ee_p.zone_x2_.type_ = Node::Zone::ZoneType::LL;
		ee_p.zone_x2_.zone_value_ = zone;
		ee_p.zone_x2_.length_ = 0.0;
		std::fill_n(ee_p.zone_x2_.lines_.p0_, 3, 0.0);
		aris::dynamic::s_vc(dim, p1, ee_p.zone_x2_.lines_.p0_);
		std::fill_n(ee_p.zone_x2_.lines_.p1_, 3, 0.0);
		aris::dynamic::s_vc(dim, p1, ee_p.zone_x2_.lines_.p1_);
		std::fill_n(ee_p.zone_x2_.lines_.p2_, 3, 0.0);
		aris::dynamic::s_vc(dim, p1, ee_p.zone_x2_.lines_.p2_);

		// scurves //
		double p = ee_p.zone_x1_.length_ / 2.0 + ee_p.zone_x2_.length_ / 2.0 + ee_p.move_x_.length_;
		ee_p.scurve_x_.pa_ = 0.0;
		ee_p.scurve_x_.pb_ = p;
		ee_p.scurve_x_.va_ = 0.0;
		ee_p.scurve_x_.vc_max_ = vel;
		ee_p.scurve_x_.vb_max_ = 0.0;
		ee_p.scurve_x_.a_ = acc;
		ee_p.scurve_x_.j_ = jerk;
		ee_p.scurve_x_.t0_ = 0.0;
	}
	auto init_ee_plan_a(const double* q0, const double* q1, double vel, double acc, double jerk, double zone, Node::EePlanData& ee_p) {
		// moves //
		aris::dynamic::s_vc(4, q0, ee_p.move_a_.quaternion_.q0_);
		aris::dynamic::s_vc(4, q1, ee_p.move_a_.quaternion_.q1_);
		s_make_quaternion_data(ee_p.move_a_.quaternion_.q0_, ee_p.move_a_.quaternion_.q1_, ee_p.move_a_.length_);
		
		// zones //
		ee_p.zone_a1_.type_ = Node::Zone::ZoneType::QQ;
		ee_p.zone_a1_.zone_value_ = 0.0;
		ee_p.zone_a1_.length_ = 0.0;
		aris::dynamic::s_vc(4, ee_p.move_a_.quaternion_.q0_, ee_p.zone_a1_.quaternions_.q0_);
		aris::dynamic::s_vc(4, ee_p.move_a_.quaternion_.q0_, ee_p.zone_a1_.quaternions_.q1_);
		aris::dynamic::s_vc(4, ee_p.move_a_.quaternion_.q0_, ee_p.zone_a1_.quaternions_.q2_);
		ee_p.zone_a2_.type_ = Node::Zone::ZoneType::QQ;
		ee_p.zone_a2_.zone_value_ = zone;
		ee_p.zone_a2_.length_ = 0.0;
		aris::dynamic::s_vc(4, ee_p.move_a_.quaternion_.q1_, ee_p.zone_a2_.quaternions_.q0_);
		aris::dynamic::s_vc(4, ee_p.move_a_.quaternion_.q1_, ee_p.zone_a2_.quaternions_.q1_);
		aris::dynamic::s_vc(4, ee_p.move_a_.quaternion_.q1_, ee_p.zone_a2_.quaternions_.q2_);

		// scurves //
		double p = ee_p.zone_a1_.length_ / 2.0 + ee_p.zone_a2_.length_ / 2.0 + ee_p.move_a_.length_;
		ee_p.scurve_a_.pa_ = 0.0;
		ee_p.scurve_a_.pb_ = p;
		ee_p.scurve_a_.va_ = 0.0;
		ee_p.scurve_a_.vc_max_ = vel;
		ee_p.scurve_a_.vb_max_ = 0.0;
		ee_p.scurve_a_.a_ = acc;
		ee_p.scurve_a_.j_ = jerk;
		ee_p.scurve_a_.t0_ = 0.0;
	}

	// make zones //
	auto make_zone_and_scurve_line_line_x(Node::EePlanData* last_p, Node::EePlanData* this_p) ->void {
		// STEP 1. 计算真实的交融半径
		double real_zone = std::min({ last_p->move_x_.length_, last_p->zone_x2_.zone_value_, this_p->move_x_.length_ * 0.5 });

		// STEP 2. 计算 last_p 和 this_p 的交融点
		double p1[3], p01[3], p12[3];
		aris::dynamic::s_vc(3, last_p->move_x_.line_.p1_, p1);

		s_compute_line_pos_at(
			last_p->move_x_.length_ - real_zone,
			last_p->move_x_.line_.p0_,
			last_p->move_x_.line_.p1_,
			last_p->move_x_.length_,
			p01);

		s_compute_line_pos_at(
			real_zone,
			this_p->move_x_.line_.p0_,
			this_p->move_x_.line_.p1_,
			this_p->move_x_.length_,
			p12);

		// STEP 3. 更新 last_p 和 this_p 的 move 部分
		aris::dynamic::s_vc(3, p01, last_p->move_x_.line_.p1_);
		s_make_line_data(last_p->move_x_.line_.p0_, last_p->move_x_.line_.p1_, last_p->move_x_.length_);
		aris::dynamic::s_vc(3, p12, this_p->move_x_.line_.p0_);
		s_make_line_data(this_p->move_x_.line_.p0_, this_p->move_x_.line_.p1_, this_p->move_x_.length_);

		// STEP 4. 更新 last_p 和 this_p 的 zone 部分
		auto& zone_x2 = last_p->zone_x2_;
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
			, zone_x2.a_, zone_x2.b_, zone_x2.c_, zone_x2.d_, zone_x2.e_);

		s_bezier3_s2arc(1.0, zone_x2.a_, zone_x2.b_, zone_x2.c_, zone_x2.d_, zone_x2.e_, arc, darc, d2arc);
		zone_x2.length_ = arc;

		this_p->zone_x1_ = zone_x2;

		// STEP 5. 更新 scurve //
		last_p->scurve_x_.pb_ = last_p->scurve_x_.pa_ + last_p->zone_x1_.length_ / 2.0 + last_p->zone_x2_.length_ / 2.0 + last_p->move_x_.length_;
		this_p->scurve_x_.pa_ = last_p->scurve_x_.pb_;
		this_p->scurve_x_.pb_ = this_p->scurve_x_.pa_ + this_p->zone_x1_.length_ / 2.0 + this_p->zone_x2_.length_ / 2.0 + this_p->move_x_.length_;

		double p50[4], dp50[4], d2p50[4];
		double vb;
		s_bezier3_blend_line_line(0.5, last_p->zone_x2_.lines_.p0_, last_p->zone_x2_.lines_.p1_, last_p->zone_x2_.lines_.p2_,
			p50, dp50, d2p50);
		s_bezier3_max_v_at(3, dp50, d2p50, std::min(last_p->scurve_x_.a_, this_p->scurve_x_.a_), vb);
		last_p->scurve_x_.vb_max_ = std::min({ vb, last_p->scurve_x_.vc_max_, this_p->scurve_x_.vc_max_ });
	}
	auto make_zone_and_scurve_line_circle_x(Node::EePlanData* last_p, Node::EePlanData* this_p) ->void {
		// STEP 0. 检查是否需要退化成直线
		if (this_p->move_type_ == Node::MoveType::Line && last_p->move_type_ == Node::MoveType::Line) {
			make_zone_and_scurve_line_line_x(last_p, this_p);
			return;
		}
		
		// STEP 1. 计算真实的交融半径
		double real_zone = std::min({ last_p->move_x_.length_, last_p->zone_x2_.zone_value_, this_p->move_x_.length_ * 0.5 });

		// STEP 2. 计算 last_p 和 this_p 的交融点
		double p1[3], p01[3], p12[3];
		aris::dynamic::s_vc(3, last_p->move_x_.line_.p1_, p1);

		s_compute_line_pos_at(
			last_p->move_x_.length_ - real_zone,
			last_p->move_x_.line_.p0_,
			last_p->move_x_.line_.p1_,
			last_p->move_x_.length_,
			p01);

		s_compute_circle_pos_at(
			real_zone,
			this_p->move_x_.circle_.p0_,
			this_p->move_x_.circle_.center_,
			this_p->move_x_.circle_.axis_,
			this_p->move_x_.circle_.radius_,
			this_p->move_x_.length_,
			p12);

		// STEP 3. 更新 last_p 和 this_p 的 move 部分
		aris::dynamic::s_vc(3, p01, last_p->move_x_.line_.p1_);
		s_make_line_data(last_p->move_x_.line_.p0_, last_p->move_x_.line_.p1_, last_p->move_x_.length_);
		aris::dynamic::s_vc(3, p12, this_p->move_x_.circle_.p0_);
		this_p->move_x_.length_ -= real_zone;

		// STEP 4. 更新 last_p 和 this_p 的 zone 部分
		auto& zone_x2 = last_p->zone_x2_;
		zone_x2.type_ = Node::Zone::ZoneType::LC;
		aris::dynamic::s_vc(3, p01, zone_x2.line_circle_.p0_);
		aris::dynamic::s_vc(3, p1, zone_x2.line_circle_.p1_);
		aris::dynamic::s_vc(3, this_p->move_x_.circle_.center_, zone_x2.line_circle_.center_);
		aris::dynamic::s_vc(3, this_p->move_x_.circle_.axis_, zone_x2.line_circle_.axis_);
		zone_x2.line_circle_.theta_ = real_zone / this_p->move_x_.circle_.radius_;

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
			, zone_x2.a_, zone_x2.b_, zone_x2.c_, zone_x2.d_, zone_x2.e_);

		s_bezier3_s2arc(1.0, zone_x2.a_, zone_x2.b_, zone_x2.c_, zone_x2.d_, zone_x2.e_, arc, darc, d2arc);
		zone_x2.length_ = arc;

		this_p->zone_x1_ = zone_x2;

		// STEP 5. 更新 scurve //
		last_p->scurve_x_.pb_ = last_p->scurve_x_.pa_ + last_p->zone_x1_.length_ / 2.0 + last_p->zone_x2_.length_ / 2.0 + last_p->move_x_.length_;
		this_p->scurve_x_.pa_ = last_p->scurve_x_.pb_;
		this_p->scurve_x_.pb_ = this_p->scurve_x_.pa_ + this_p->zone_x1_.length_ / 2.0 + this_p->zone_x2_.length_ / 2.0 + this_p->move_x_.length_;

		// 更新 scurve 中点处的最大速度作为 vb_max //
		double p50[4], dp50[4], d2p50[4];
		double vb;
		s_bezier3_blend_line_circle(0.5, last_p->zone_x2_.line_circle_.p0_, last_p->zone_x2_.line_circle_.p1_
			, last_p->zone_x2_.line_circle_.center_, last_p->zone_x2_.line_circle_.axis_, last_p->zone_x2_.line_circle_.theta_,
			p50, dp50, d2p50);
		s_bezier3_max_v_at(3, dp50, d2p50, std::min(last_p->scurve_x_.a_, this_p->scurve_x_.a_), vb);
		last_p->scurve_x_.vb_max_ = std::min({ vb, last_p->scurve_x_.vc_max_, this_p->scurve_x_.vc_max_ });
	}
	auto make_zone_and_scurve_circle_line_x(Node::EePlanData* last_p, Node::EePlanData* this_p) ->void {
		// STEP 0. 检查是否需要退化成直线
		if (this_p->move_type_ == Node::MoveType::Line && last_p->move_type_ == Node::MoveType::Line) {
			make_zone_and_scurve_line_line_x(last_p, this_p);
			return;
		}
		
		// STEP 1. 计算真实的交融半径
		double real_zone = std::min({ last_p->move_x_.length_, last_p->zone_x2_.zone_value_, this_p->move_x_.length_ * 0.5 });

		// STEP 2. 计算 last_p 和 this_p 的交融点，只需要计算直线部分的接触点，因为圆的起点就是两者交点（p1）
		double p1[3], p12[3];
		aris::dynamic::s_vc(3, this_p->move_x_.line_.p0_, p1);

		s_compute_line_pos_at(
			real_zone,
			this_p->move_x_.line_.p0_,
			this_p->move_x_.line_.p1_,
			this_p->move_x_.length_,
			p12);

		// STEP 3. 更新 last_p 和 this_p 的 move 部分
		last_p->move_x_.length_ -= real_zone;
		aris::dynamic::s_vc(3, p12, this_p->move_x_.line_.p0_);
		s_make_line_data(this_p->move_x_.line_.p0_, this_p->move_x_.line_.p1_, this_p->move_x_.length_);

		// STEP 4. 更新 last_p 和 this_p 的 zone 部分
		auto& zone_x2 = last_p->zone_x2_;
		zone_x2.type_ = Node::Zone::ZoneType::CL;
		aris::dynamic::s_vc(3, last_p->move_x_.circle_.center_, zone_x2.circle_line_.center_);
		aris::dynamic::s_vc(3, -1.0, last_p->move_x_.circle_.axis_, zone_x2.circle_line_.axis_);
		aris::dynamic::s_vc(3, p1, zone_x2.circle_line_.p1_);   // p1 实际是circle的终点，但是把它视为 zone 的反向起点
		aris::dynamic::s_vc(3, p12, zone_x2.circle_line_.p2_);
		zone_x2.circle_line_.theta_ = real_zone / last_p->move_x_.circle_.radius_;

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

		s_bezier3_estimate_arc_param(darc_ds_0, d2arc_ds2_0, darc_ds_1, d2arc_ds2_1, darc_ds_50
			, zone_x2.a_, zone_x2.b_, zone_x2.c_, zone_x2.d_, zone_x2.e_);

		s_bezier3_s2arc(1.0, zone_x2.a_, zone_x2.b_, zone_x2.c_, zone_x2.d_, zone_x2.e_, arc, darc, d2arc);
		zone_x2.length_ = arc;

		this_p->zone_x1_ = zone_x2;

		// STEP 5. 更新 scurve //
		last_p->scurve_x_.pb_ = last_p->scurve_x_.pa_ + last_p->zone_x1_.length_ / 2.0 + last_p->zone_x2_.length_ / 2.0 + last_p->move_x_.length_;
		this_p->scurve_x_.pa_ = last_p->scurve_x_.pb_;
		this_p->scurve_x_.pb_ = this_p->scurve_x_.pa_ + this_p->zone_x1_.length_ / 2.0 + this_p->zone_x2_.length_ / 2.0 + this_p->move_x_.length_;

		// 更新 scurve 中点处的最大速度作为 vb_max //
		double p50[4], dp50[4], d2p50[4];
		double vb;
		s_bezier3_blend_line_circle(0.5, last_p->zone_x2_.circle_line_.p2_, last_p->zone_x2_.circle_line_.p1_
			, last_p->zone_x2_.circle_line_.center_, last_p->zone_x2_.circle_line_.axis_, last_p->zone_x2_.circle_line_.theta_,
			p50, dp50, d2p50);
		s_bezier3_max_v_at(3, dp50, d2p50, std::min(last_p->scurve_x_.a_, this_p->scurve_x_.a_), vb);
		last_p->scurve_x_.vb_max_ = std::min({ vb, last_p->scurve_x_.vc_max_, this_p->scurve_x_.vc_max_ });
	}
	auto make_zone_and_scurve_circle_circle_x(Node::EePlanData* last_p, Node::EePlanData* this_p) ->void {
		// STEP 0. 检查是否需要退化成直线
		if (this_p->move_type_ == Node::MoveType::Line && last_p->move_type_ == Node::MoveType::Circle) {
			make_zone_and_scurve_circle_line_x(last_p, this_p);
			return;
		}
		else if (this_p->move_type_ == Node::MoveType::Circle && last_p->move_type_ == Node::MoveType::Line) {
			make_zone_and_scurve_line_circle_x(last_p, this_p);
			return;
		}
		else if (this_p->move_type_ == Node::MoveType::Line && last_p->move_type_ == Node::MoveType::Line) {
			make_zone_and_scurve_line_line_x(last_p, this_p);
			return;
		}
		
		// STEP 1. 计算真实的交融半径
		double real_zone = std::min({ last_p->move_x_.length_, last_p->zone_x2_.zone_value_, this_p->move_x_.length_*0.5 });

		// STEP 2. 计算 last_p 和 this_p 的交融点
		double p1[3];
		aris::dynamic::s_vc(3, this_p->move_x_.circle_.p0_, p1);

		// STEP 3. 更新 last_p 和 this_p 的 move 部分
		last_p->move_x_.length_ -= real_zone;
		s_compute_circle_pos_at(
			real_zone,
			this_p->move_x_.circle_.p0_,
			this_p->move_x_.circle_.center_,
			this_p->move_x_.circle_.axis_,
			this_p->move_x_.circle_.radius_,
			this_p->move_x_.length_,
			this_p->move_x_.circle_.p0_);
		this_p->move_x_.length_ -= real_zone;

		// STEP 4. 更新 last_p 和 this_p 的 zone 部分
		auto& zone_x2 = last_p->zone_x2_;
		auto& circles = zone_x2.circles_;
		
		zone_x2.type_ = Node::Zone::ZoneType::CC;
		aris::dynamic::s_vc(3, p1, circles.pcenter_);
		aris::dynamic::s_vc(3, last_p->move_x_.circle_.axis_, circles.a1_);
		aris::dynamic::s_vc(3, last_p->move_x_.circle_.center_, circles.c1_);
		aris::dynamic::s_vc(3, this_p->move_x_.circle_.axis_, circles.a2_);
		aris::dynamic::s_vc(3, this_p->move_x_.circle_.center_, circles.c2_);
		circles.theta1_ = real_zone / last_p->move_x_.circle_.radius_;
		circles.theta2_ = real_zone / this_p->move_x_.circle_.radius_;

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
			, zone_x2.a_, zone_x2.b_, zone_x2.c_, zone_x2.d_, zone_x2.e_);

		s_bezier3_s2arc(1.0, zone_x2.a_, zone_x2.b_, zone_x2.c_, zone_x2.d_, zone_x2.e_, arc, darc, d2arc);
		zone_x2.length_ = arc;

		this_p->zone_x1_ = zone_x2;

		// STEP 5. 更新 scurve //
		last_p->scurve_x_.pb_ = last_p->scurve_x_.pa_ + last_p->zone_x1_.length_ / 2.0 + last_p->zone_x2_.length_ / 2.0 + last_p->move_x_.length_;
		this_p->scurve_x_.pa_ = last_p->scurve_x_.pb_;
		this_p->scurve_x_.pb_ = this_p->scurve_x_.pa_ + this_p->zone_x1_.length_ / 2.0 + this_p->zone_x2_.length_ / 2.0 + this_p->move_x_.length_;

		// 更新 scurve 中点处的最大速度作为 vb_max //
		double p50[4], dp50[4], d2p50[4];
		double vb;
		s_bezier3_blend_circle_circle(0.5, circles.pcenter_, circles.c1_, circles.a1_, circles.theta1_
			, circles.c2_, circles.a2_, circles.theta2_
			, p50, dp50, d2p50);
		s_bezier3_max_v_at(3, dp50, d2p50, std::min(last_p->scurve_x_.a_, this_p->scurve_x_.a_), vb);
		last_p->scurve_x_.vb_max_ = std::min({ vb, last_p->scurve_x_.vc_max_, this_p->scurve_x_.vc_max_ });
	}
	auto make_zone_and_scurve_quternion_a(Node::EePlanData* last_p, Node::EePlanData* this_p)->void {
		// STEP 1. 计算真实的交融半径
		double real_zone = std::min({ last_p->move_a_.length_, last_p->zone_a2_.zone_value_, this_p->move_a_.length_ });

		// STEP 2. 计算 last_p 和 this_p 的交融点
		double q1[4], q01[4], q12[4];
		aris::dynamic::s_vc(4, last_p->move_a_.quaternion_.q1_, q1);

		s_compute_quaternion_at(
			last_p->move_a_.length_ - real_zone,
			last_p->move_a_.quaternion_.q0_,
			last_p->move_a_.quaternion_.q1_,
			last_p->move_a_.length_,
			q01);

		s_compute_quaternion_at(
			real_zone,
			this_p->move_a_.quaternion_.q0_,
			this_p->move_a_.quaternion_.q1_,
			this_p->move_a_.length_,
			q12);

		// STEP 3. 更新 last_p 和 this_p 的 move 部分
		aris::dynamic::s_vc(4, q01, last_p->move_a_.quaternion_.q1_);
		s_make_quaternion_data(last_p->move_a_.quaternion_.q0_, last_p->move_a_.quaternion_.q1_, last_p->move_a_.length_);
		aris::dynamic::s_vc(4, q12, this_p->move_a_.quaternion_.q0_);
		s_make_quaternion_data(this_p->move_a_.quaternion_.q0_, this_p->move_a_.quaternion_.q1_, this_p->move_a_.length_);

		// STEP 4. 更新 last_p 和 this_p 的 zone 部分
		auto& zone_a2 = last_p->zone_a2_;
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
			, zone_a2.a_, zone_a2.b_, zone_a2.c_, zone_a2.d_, zone_a2.e_);

		s_bezier3_s2arc(1.0, zone_a2.a_, zone_a2.b_, zone_a2.c_, zone_a2.d_, zone_a2.e_, arc, darc, d2arc);
		zone_a2.length_ = arc;

		this_p->zone_a1_ = zone_a2;

		// STEP 5. 更新 scurve //
		last_p->scurve_a_.pb_ = last_p->scurve_a_.pa_ + last_p->zone_a1_.length_ / 2.0 + last_p->zone_a2_.length_ / 2.0 + last_p->move_a_.length_;
		this_p->scurve_a_.pa_ = last_p->scurve_a_.pb_;
		this_p->scurve_a_.pb_ = this_p->scurve_a_.pa_ + this_p->zone_a1_.length_ / 2.0 + this_p->zone_a2_.length_ / 2.0 + this_p->move_a_.length_;

		double p50[4], dp50[4], d2p50[4];
		double vb;
		s_bezier3_blend_quaternion(0.5, last_p->zone_a2_.quaternions_.q0_, last_p->zone_a2_.quaternions_.q1_, last_p->zone_a2_.quaternions_.q2_,
			p50, dp50, d2p50);
		s_bezier3_max_v_at(4, dp50, d2p50, std::min(last_p->scurve_a_.a_, this_p->scurve_a_.a_), vb);
		last_p->scurve_a_.vb_max_ = std::min({ vb * 2.0, last_p->scurve_a_.vc_max_, this_p->scurve_a_.vc_max_ });
	}

	// make nodes //
	auto make_node(aris::Size replan_num, Node* this_node, Node* last_node, const std::vector<aris::dynamic::EEType>& ee_types,
		Node::MoveType move_type, const double* ee_pos, const double* mid_pos, const double* vel, const double* acc, const double* jerk, const double* zone)->void {
		// 更新本段轨迹的 move //
		for (Size i{ 0 }, pos_idx{ 0 }, vel_idx{ 0 }; i < ee_types.size(); ++i) {
			auto this_p = &this_node->ee_plans_[i];
			this_p->move_type_ = move_type;

			switch (ee_types[i]) {
			case aris::dynamic::EEType::PE313: [[fallthrough]];
			case aris::dynamic::EEType::PE321: [[fallthrough]];
			case aris::dynamic::EEType::PE123: [[fallthrough]];
			case aris::dynamic::EEType::PM: [[fallthrough]];
			case aris::dynamic::EEType::PQ: {
				switch (this_p->move_type_) {
				case aris::plan::Node::MoveType::ResetInitPos: {
					// init //
					init_ee_plan_x(3, ee_pos + pos_idx, ee_pos + pos_idx, vel[vel_idx], acc[vel_idx], jerk[vel_idx], zone[vel_idx], *this_p);
					init_ee_plan_a(ee_pos + pos_idx + 3, ee_pos + pos_idx + 3, vel[vel_idx + 1], acc[vel_idx + 1], jerk[vel_idx + 1], zone[vel_idx + 1], *this_p);

					// 将四元数转向与上一次设置一致
					if (last_node && aris::dynamic::s_vv(4, last_node->ee_plans_[i].move_a_.quaternion_.q1_, this_p->move_a_.quaternion_.q1_) < 0.0) {
						aris::dynamic::s_nv(4, -1.0, this_p->move_a_.quaternion_.q1_);
						aris::dynamic::s_nv(4, -1.0, this_p->zone_a2_.quaternions_.q0_);
						aris::dynamic::s_nv(4, -1.0, this_p->zone_a2_.quaternions_.q1_);
					}

					break;
				}
				case aris::plan::Node::MoveType::Line: {
					auto last_p = &last_node->ee_plans_[i];

					switch (last_p->move_type_) {
					case aris::plan::Node::MoveType::ResetInitPos: {
						// init //
						init_ee_plan_x(3, last_p->move_x_.line_.p1_, ee_pos + pos_idx, vel[vel_idx], acc[vel_idx], jerk[vel_idx], zone[vel_idx], *this_p);
						init_ee_plan_a(last_p->move_a_.quaternion_.q1_, ee_pos + pos_idx + 3, vel[vel_idx + 1], acc[vel_idx + 1], jerk[vel_idx + 1], zone[vel_idx + 1], *this_p);
						break;
					}
					case aris::plan::Node::MoveType::Line: {
						// init //
						init_ee_plan_x(3, last_p->move_x_.line_.p1_, ee_pos + pos_idx, vel[vel_idx], acc[vel_idx], jerk[vel_idx], zone[vel_idx], *this_p);
						init_ee_plan_a(last_p->move_a_.quaternion_.q1_, ee_pos + pos_idx + 3, vel[vel_idx + 1], acc[vel_idx + 1], jerk[vel_idx + 1], zone[vel_idx + 1], *this_p);
						
						// 将四元数转向与上一次设置一致
						if (aris::dynamic::s_vv(4, last_p->move_a_.quaternion_.q1_, this_p->move_a_.quaternion_.q1_) < 0.0) {
							aris::dynamic::s_nv(4, -1.0, this_p->move_a_.quaternion_.q1_);
							aris::dynamic::s_nv(4, -1.0, this_p->zone_a2_.quaternions_.q0_);
							aris::dynamic::s_nv(4, -1.0, this_p->zone_a2_.quaternions_.q1_);
						}

						// 和上一段进行路径拼接
						if (replan_num) {
							make_zone_and_scurve_line_line_x(last_p, this_p);
							make_zone_and_scurve_quternion_a(last_p, this_p);
						}
						break;
					}
					case aris::plan::Node::MoveType::Circle: {
						// get last circle end //
						double p1[3];
						s_compute_circle_pos_at(last_p->move_x_.length_, last_p->move_x_.circle_.p0_, last_p->move_x_.circle_.center_, last_p->move_x_.circle_.axis_
							, last_p->move_x_.circle_.radius_, last_p->move_x_.length_, p1);

						// init //
						init_ee_plan_x(3, p1, ee_pos + pos_idx, vel[vel_idx], acc[vel_idx], jerk[vel_idx], zone[vel_idx], *this_p);
						init_ee_plan_a(last_p->move_a_.quaternion_.q1_, ee_pos + pos_idx + 3, vel[vel_idx + 1], acc[vel_idx + 1], jerk[vel_idx + 1], zone[vel_idx + 1], *this_p);

						// 将四元数转向与上一次设置一致
						if (aris::dynamic::s_vv(4, last_p->move_a_.quaternion_.q1_, this_p->move_a_.quaternion_.q1_) < 0.0) {
							aris::dynamic::s_nv(4, -1.0, this_p->move_a_.quaternion_.q1_);
							aris::dynamic::s_nv(4, -1.0, this_p->zone_a2_.quaternions_.q0_);
							aris::dynamic::s_nv(4, -1.0, this_p->zone_a2_.quaternions_.q1_);
						}

						// 和上一段进行路径拼接
						if (replan_num) {
							make_zone_and_scurve_circle_line_x(last_p, this_p);
							make_zone_and_scurve_quternion_a(last_p, this_p);
						}
						break;
					}
					default:
						break;
					}
					break;
				}
				case aris::plan::Node::MoveType::Circle: {
					auto last_p = &last_node->ee_plans_[i];
					switch (last_p->move_type_) {
					case aris::plan::Node::MoveType::ResetInitPos: {
						// init //
						init_ee_plan_c(3, last_p->move_x_.line_.p1_, mid_pos + pos_idx, ee_pos + pos_idx, vel[vel_idx], acc[vel_idx], jerk[vel_idx], zone[vel_idx], *this_p);
						init_ee_plan_a(last_p->move_a_.quaternion_.q1_, ee_pos + pos_idx + 3, vel[vel_idx + 1], acc[vel_idx + 1], jerk[vel_idx + 1], zone[vel_idx + 1], *this_p);
						break;
					}
					case aris::plan::Node::MoveType::Line: {
						// init //
						init_ee_plan_c(3, last_p->move_x_.line_.p1_, mid_pos + pos_idx, ee_pos + pos_idx, vel[vel_idx], acc[vel_idx], jerk[vel_idx], zone[vel_idx], *this_p);
						init_ee_plan_a(last_p->move_a_.quaternion_.q1_, ee_pos + pos_idx + 3, vel[vel_idx + 1], acc[vel_idx + 1], jerk[vel_idx + 1], zone[vel_idx + 1], *this_p);

						// 将四元数转向与上一次设置一致
						if (aris::dynamic::s_vv(4, last_p->move_a_.quaternion_.q1_, this_p->move_a_.quaternion_.q1_) < 0.0) {
							aris::dynamic::s_nv(4, -1.0, this_p->move_a_.quaternion_.q1_);
							aris::dynamic::s_nv(4, -1.0, this_p->zone_a2_.quaternions_.q0_);
							aris::dynamic::s_nv(4, -1.0, this_p->zone_a2_.quaternions_.q1_);
						}

						// 和上一段进行路径拼接
						if (replan_num) {
							make_zone_and_scurve_line_circle_x(last_p, this_p);
							make_zone_and_scurve_quternion_a(last_p, this_p);
						}
						break;
					}
					case aris::plan::Node::MoveType::Circle: {
						// get last circle end //
						double p1[3];
						s_compute_circle_pos_at(last_p->move_x_.length_, last_p->move_x_.circle_.p0_, last_p->move_x_.circle_.center_, last_p->move_x_.circle_.axis_
							, last_p->move_x_.circle_.radius_, last_p->move_x_.length_, p1);

						// init //
						init_ee_plan_c(3, p1, mid_pos + pos_idx, ee_pos + pos_idx, vel[vel_idx], acc[vel_idx], jerk[vel_idx], zone[vel_idx], *this_p);
						init_ee_plan_a(last_p->move_a_.quaternion_.q1_, ee_pos + pos_idx + 3, vel[vel_idx + 1], acc[vel_idx + 1], jerk[vel_idx + 1], zone[vel_idx + 1], *this_p);

						// 将四元数转向与上一次设置一致
						if (aris::dynamic::s_vv(4, last_p->move_a_.quaternion_.q1_, this_p->move_a_.quaternion_.q1_) < 0.0) {
							aris::dynamic::s_nv(4, -1.0, this_p->move_a_.quaternion_.q1_);
							aris::dynamic::s_nv(4, -1.0, this_p->zone_a2_.quaternions_.q0_);
							aris::dynamic::s_nv(4, -1.0, this_p->zone_a2_.quaternions_.q1_);
						}

						// 和上一段进行路径拼接，并只做 s 曲线
						if (replan_num) {
							make_zone_and_scurve_circle_circle_x(last_p, this_p);
							make_zone_and_scurve_quternion_a(last_p, this_p);
						}
						break;
					}
					default:
						break;
					}
					break;
				
				}
				default:
					break;
				}
				
				pos_idx += 7;
				vel_idx += 2;
				break;
			}
			case aris::dynamic::EEType::XYZT: {
				pos_idx += 4;
				vel_idx += 2;
				break;
			}
			case aris::dynamic::EEType::XYZ: {
				pos_idx += 3;
				vel_idx += 1;
				break;
			}
			case aris::dynamic::EEType::XYT: {
				pos_idx += 3;
				vel_idx += 2;
				break;
			}
			case aris::dynamic::EEType::XY: {
				pos_idx += 2;
				vel_idx += 1;
				break;
			}
			case aris::dynamic::EEType::X: [[fallthrough]];
			case aris::dynamic::EEType::A: {
				switch (this_p->move_type_) {
				case aris::plan::Node::MoveType::ResetInitPos:
					init_ee_plan_x(1, ee_pos + pos_idx, ee_pos + pos_idx, vel[vel_idx], acc[vel_idx], jerk[vel_idx], zone[vel_idx], *this_p);
					break;
				case aris::plan::Node::MoveType::Line: [[fallthrough]];
				case aris::plan::Node::MoveType::Circle: {
					this_p->move_type_ = aris::plan::Node::MoveType::Line;
					auto last_p = &last_node->ee_plans_[i];
					switch (last_p->move_type_) {
					case aris::plan::Node::MoveType::ResetInitPos: [[fallthrough]];
					case aris::plan::Node::MoveType::Line: [[fallthrough]];
					case aris::plan::Node::MoveType::Circle: {
						// init //
						init_ee_plan_x(1, last_p->move_x_.line_.p1_, ee_pos + pos_idx, vel[vel_idx], acc[vel_idx], jerk[vel_idx], zone[vel_idx], *this_p);

						// 和上一段进行路径拼接
						if (replan_num) {
							make_zone_and_scurve_line_line_x(last_p, this_p);
						}
						break;
					}
					default:
						break;
					}
					break;
				}
				default:
					break;
				}

				pos_idx += 1;
				vel_idx += 1;
				break;
			}
			case aris::dynamic::EEType::UNKNOWN: break;
			default:
				break;
			}
		}
	}
	auto replan_scurve(int scurve_size, const std::vector<aris::dynamic::EEType> &ee_types, std::list<Node>::iterator last, std::list<Node>::iterator begin, std::list<Node>::iterator end) {
		std::list<SCurveNode> ins_scurve_list;
		LargeNum t0;
		for (auto iter = begin; iter != end; ++iter) {
			ins_scurve_list.push_back(SCurveNode{});
			auto& scurve_node = ins_scurve_list.back();
			scurve_node.params_.reserve(scurve_size);
			//for (auto& ee_p : iter->ee_plans_) {
			for (int i = 0; i < ee_types.size();++i) {
				auto& ee_p = iter->ee_plans_[i];
				switch (ee_types[i]) {
				case aris::dynamic::EEType::PE313: [[fallthrough]];
				case aris::dynamic::EEType::PE321: [[fallthrough]];
				case aris::dynamic::EEType::PE123: [[fallthrough]];
				case aris::dynamic::EEType::PM: [[fallthrough]];
				case aris::dynamic::EEType::PQ: {
					begin->ee_plans_[i].scurve_x_.t0_ = last->ee_plans_[i].scurve_x_.t0_ + last->ee_plans_[i].scurve_x_.T_;
					begin->ee_plans_[i].scurve_a_.t0_ = last->ee_plans_[i].scurve_a_.t0_ + last->ee_plans_[i].scurve_a_.T_;
					scurve_node.params_.push_back(ee_p.scurve_x_);
					scurve_node.params_.push_back(ee_p.scurve_a_);
					break;
				}
				case aris::dynamic::EEType::XYZT: {
					begin->ee_plans_[i].scurve_x_.t0_ = last->ee_plans_[i].scurve_x_.t0_ + last->ee_plans_[i].scurve_x_.T_;
					begin->ee_plans_[i].scurve_a_.t0_ = last->ee_plans_[i].scurve_a_.t0_ + last->ee_plans_[i].scurve_a_.T_;
					scurve_node.params_.push_back(ee_p.scurve_x_);
					scurve_node.params_.push_back(ee_p.scurve_a_);
					break;
				}
				case aris::dynamic::EEType::XYZ: {
					begin->ee_plans_[i].scurve_x_.t0_ = last->ee_plans_[i].scurve_x_.t0_ + last->ee_plans_[i].scurve_x_.T_;
					scurve_node.params_.push_back(ee_p.scurve_x_);
					break;
				}
				case aris::dynamic::EEType::XYT: {
					begin->ee_plans_[i].scurve_x_.t0_ = last->ee_plans_[i].scurve_x_.t0_ + last->ee_plans_[i].scurve_x_.T_;
					scurve_node.params_.push_back(ee_p.scurve_x_);
					scurve_node.params_.push_back(ee_p.scurve_a_);
					break;
				}
				case aris::dynamic::EEType::XY: {
					begin->ee_plans_[i].scurve_x_.t0_ = last->ee_plans_[i].scurve_x_.t0_ + last->ee_plans_[i].scurve_x_.T_;
					scurve_node.params_.push_back(ee_p.scurve_x_);
					break;
				}
				case aris::dynamic::EEType::X: [[fallthrough]];
				case aris::dynamic::EEType::A: {
					begin->ee_plans_[i].scurve_x_.t0_ = last->ee_plans_[i].scurve_x_.t0_ + last->ee_plans_[i].scurve_x_.T_;
					scurve_node.params_.push_back(ee_p.scurve_x_);
					break;
				}
				case aris::dynamic::EEType::UNKNOWN:
				default:
					;
				}
			}
		}

		s_compute_scurve(ins_scurve_list.begin(), ins_scurve_list.end());

		for (auto iter = begin; iter != end; ++iter) {
			auto& scurve_node = ins_scurve_list.front();
			iter->s_end_ = scurve_node.params_[0].t0_ + scurve_node.params_[0].T_;
			for (int i = 0, s_idx = 0; i < iter->ee_plans_.size(); ++i) {
				auto& ee_p = iter->ee_plans_[i];
				switch (ee_types[i]) {
				case aris::dynamic::EEType::PE313: [[fallthrough]];
				case aris::dynamic::EEType::PE321: [[fallthrough]];
				case aris::dynamic::EEType::PE123: [[fallthrough]];
				case aris::dynamic::EEType::PM: [[fallthrough]];
				case aris::dynamic::EEType::PQ: {
					ee_p.scurve_x_ = scurve_node.params_[s_idx];
					s_idx++;
					ee_p.scurve_a_ = scurve_node.params_[s_idx];
					s_idx++;
					break;
				}
				case aris::dynamic::EEType::XYZT: {
					ee_p.scurve_x_ = scurve_node.params_[s_idx];
					s_idx++;
					ee_p.scurve_a_ = scurve_node.params_[s_idx];
					s_idx++;
					break;
				}
				case aris::dynamic::EEType::XYZ: {
					ee_p.scurve_x_ = scurve_node.params_[s_idx];
					s_idx++;
					break;
				}
				case aris::dynamic::EEType::XYT: {
					ee_p.scurve_x_ = scurve_node.params_[s_idx];
					s_idx++;
					ee_p.scurve_a_ = scurve_node.params_[s_idx];
					s_idx++;
					break;
				}
				case aris::dynamic::EEType::XY: {
					ee_p.scurve_x_ = scurve_node.params_[s_idx];
					s_idx++;
					break;
				}
				case aris::dynamic::EEType::X: [[fallthrough]];
				case aris::dynamic::EEType::A: {
					ee_p.scurve_x_ = scurve_node.params_[s_idx];
					s_idx++;
					break;
				}
				case aris::dynamic::EEType::UNKNOWN:
				default:
					;
				}
			}
			ins_scurve_list.pop_front();
			iter->next_node_.store(std::next(iter) == end ? &*iter : &*std::next(iter));
		}
	}

	// get data //
	auto get_zone_data(double arc, const Node::Zone &z, double *data)->void {
		switch (z.type_){
		case Node::Zone::ZoneType::LL: {
			double s;
			s_bezier3_arc2s(arc, z.a_, z.b_, z.c_, z.d_, z.e_, s);
			double p[3], dp[3], d2p[3];
			s_bezier3_blend_line_line(s, z.lines_.p0_, z.lines_.p1_, z.lines_.p2_, p, dp, d2p);
			aris::dynamic::s_vc(3, p, data);
			break;
		}
		case Node::Zone::ZoneType::LC: {
			double s;
			s_bezier3_arc2s(arc, z.a_, z.b_, z.c_, z.d_, z.e_, s);
			double p[3], dp[3], d2p[3];
			s_bezier3_blend_line_circle(s, z.line_circle_.p0_, z.line_circle_.p1_, z.line_circle_.center_, z.line_circle_.axis_, z.line_circle_.theta_
				, p, dp, d2p);
			aris::dynamic::s_vc(3, p, data);
			break;
		}
		case Node::Zone::ZoneType::CL: {
			double s;
			s_bezier3_arc2s(z.length_ - arc, z.a_, z.b_, z.c_, z.d_, z.e_, s);
			double p[3], dp[3], d2p[3];
			s_bezier3_blend_line_circle(s, z.circle_line_.p2_, z.circle_line_.p1_, z.circle_line_.center_, z.circle_line_.axis_, z.circle_line_.theta_
				, p, dp, d2p);
			aris::dynamic::s_vc(3, p, data);
			break;
		}
		case Node::Zone::ZoneType::CC: {
			double s;
			s_bezier3_arc2s(arc, z.a_, z.b_, z.c_, z.d_, z.e_, s);
			double p[3], dp[3], d2p[3];
			s_bezier3_blend_circle_circle(s, z.circles_.pcenter_, z.circles_.c1_, z.circles_.a1_, z.circles_.theta1_
				, z.circles_.c2_, z.circles_.a2_, z.circles_.theta2_
				, p, dp, d2p);
			aris::dynamic::s_vc(3, p, data);
			break;
		}
		case Node::Zone::ZoneType::QQ: {
			double s;
			s_bezier3_arc2s(arc, z.a_, z.b_, z.c_, z.d_, z.e_, s);
			double p[4], dp[4], d2p[4];
			s_bezier3_blend_quaternion(s, z.quaternions_.q0_, z.quaternions_.q1_, z.quaternions_.q2_, p, dp, d2p);
			aris::dynamic::s_vc(4, p, data);
			break;
		}
		default:
			break;
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
		double target_ds_{ 1.0 };

		// 末端类型 //
		std::vector<aris::dynamic::EEType> ee_types_;
		aris::Size outpos_size_{ 0 }, internal_pos_size{ 0 };
		std::vector<double> internal_pos_;

		// 规划节点 //
		std::list<Node> nodes_;
		std::atomic<Node*> current_node_;

		// 互斥区，保护访问
		std::recursive_mutex mu_;
	};
	auto TrajectoryGenerator::eeTypes()const-> const std::vector<aris::dynamic::EEType>& {
		return imp_->ee_types_;
	}
	auto TrajectoryGenerator::setEeTypes(const std::vector<aris::dynamic::EEType>& ee_types)->void {
		imp_->ee_types_ = ee_types;
		imp_->outpos_size_ = aris::dynamic::getEETypePosSize(ee_types);

		auto &internal_pos_size = imp_->internal_pos_size;
		for (auto type : ee_types) {
			switch (type) {
			case aris::dynamic::EEType::PE313: [[fallthrough]];
			case aris::dynamic::EEType::PE321: [[fallthrough]];
			case aris::dynamic::EEType::PE123: [[fallthrough]];
			case aris::dynamic::EEType::PM: [[fallthrough]];
			case aris::dynamic::EEType::PQ:
				internal_pos_size += 7;
				break;
			case aris::dynamic::EEType::XYZT:
				internal_pos_size += 4;
				break;
			case aris::dynamic::EEType::XYZ:
				internal_pos_size += 3;
				break;
			case aris::dynamic::EEType::XYT:
				internal_pos_size += 3;
				break;
			case aris::dynamic::EEType::XY:
				internal_pos_size += 2;
				break;
			case aris::dynamic::EEType::X:
				internal_pos_size += 1;
				break;
			case aris::dynamic::EEType::A:
				internal_pos_size += 1;
				break;
			case aris::dynamic::EEType::UNKNOWN:
				break;
			default:
				break;
			}
		}

		imp_->internal_pos_.resize(internal_pos_size);
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
	auto TrajectoryGenerator::targetDs()const->double {
		return imp_->target_ds_;
	}
	auto TrajectoryGenerator::setTargetDs(double ds)->void {
		imp_->target_ds_ = ds;
	}
	auto TrajectoryGenerator::currentDds()const->double {
		return imp_->dds_;
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
	TrajectoryGenerator::~TrajectoryGenerator() = default;
	TrajectoryGenerator::TrajectoryGenerator() :imp_(new Imp) {
		imp_->current_node_.store(nullptr);
	}
	auto TrajectoryGenerator::getEePosAndMoveDt(double* ee_pos)->std::int64_t {
		auto current_node = imp_->current_node_.load();
		auto next_node = current_node->next_node_.load();

		// 既然能进入本次规划，说明 s 一定合法 //
		auto &s_ = imp_->s_;

		int idx = 0;
		for (int i = 0; i < imp_->ee_types_.size();++i) {
			auto& ee_p = current_node->ee_plans_[i];
			switch (ee_p.move_type_){
			case Node::MoveType::ResetInitPos: {
				s_ = 0.0;
				imp_->ds_ = imp_->target_ds_;

				switch (imp_->ee_types_[i]) {
				case aris::dynamic::EEType::PE313: [[fallthrough]];
				case aris::dynamic::EEType::PE321: [[fallthrough]];
				case aris::dynamic::EEType::PE123: [[fallthrough]];
				case aris::dynamic::EEType::PM: [[fallthrough]];
				case aris::dynamic::EEType::PQ: {
					aris::dynamic::s_vc(3, ee_p.move_x_.line_.p1_, imp_->internal_pos_.data() + idx);
					idx += 3;
					aris::dynamic::s_vc(4, ee_p.move_a_.quaternion_.q1_, imp_->internal_pos_.data() + idx);
					idx += 4;
					break;
				}
				case aris::dynamic::EEType::XYZT: {
					break;
				}
				case aris::dynamic::EEType::XYZ: {
					break;
				}
				case aris::dynamic::EEType::XYT: {
					break;
				}
				case aris::dynamic::EEType::XY: {
					break;
				}
				case aris::dynamic::EEType::X: [[fallthrough]];
				case aris::dynamic::EEType::A: {
					aris::dynamic::s_vc(1, ee_p.move_x_.line_.p1_, imp_->internal_pos_.data() + idx);
					idx += 1;
					break;
				}
				case aris::dynamic::EEType::UNKNOWN:
				default:
					;
				}
				break;
			}
			case Node::MoveType::Line: {
				switch (imp_->ee_types_[i]) {
				case aris::dynamic::EEType::PE313: [[fallthrough]];
				case aris::dynamic::EEType::PE321: [[fallthrough]];
				case aris::dynamic::EEType::PE123: [[fallthrough]];
				case aris::dynamic::EEType::PM: [[fallthrough]];
				case aris::dynamic::EEType::PQ: {
					// x //
					{
						LargeNum sp;
						double sv, sa, sj;
						s_scurve_at(ee_p.scurve_x_, s_, &sp, &sv, &sa, &sj);
						double l = sp - ee_p.scurve_x_.pa_;
						if (l < ee_p.zone_x1_.length_ / 2.0) {
							get_zone_data(l + ee_p.zone_x1_.length_ / 2.0, ee_p.zone_x1_, imp_->internal_pos_.data() + idx);
						}
						else if (l > ee_p.zone_x1_.length_ / 2.0 + ee_p.move_x_.length_) {
							get_zone_data(l - ee_p.zone_x1_.length_ / 2.0 - ee_p.move_x_.length_, ee_p.zone_x2_, imp_->internal_pos_.data() + idx);
						} 
						else {
							auto& line = ee_p.move_x_.line_;
							s_compute_line_pos_at(l - ee_p.zone_x1_.length_ / 2.0, line.p0_, line.p1_, ee_p.move_x_.length_, imp_->internal_pos_.data() + idx);
						}
						idx += 3;
					}

					// a //
					{
						LargeNum sp;
						double sv, sa, sj;
						s_scurve_at(ee_p.scurve_a_, s_, &sp, &sv, &sa, &sj);
						double l = sp - ee_p.scurve_a_.pa_;
						if (l < ee_p.zone_a1_.length_ / 2.0) {
							get_zone_data(l + ee_p.zone_a1_.length_ / 2.0, ee_p.zone_a1_, imp_->internal_pos_.data() + idx);
						}
						else if (l > ee_p.zone_a1_.length_ / 2.0 + ee_p.move_a_.length_) {
							get_zone_data(l - ee_p.zone_a1_.length_ / 2.0 - ee_p.move_a_.length_, ee_p.zone_a2_, imp_->internal_pos_.data() + idx);
						}
						else {
							// in move //
							auto& quternion = ee_p.move_a_.quaternion_;
							s_compute_quaternion_at(l - ee_p.zone_a1_.length_ / 2.0, quternion.q0_, quternion.q1_, ee_p.move_a_.length_, imp_->internal_pos_.data() + idx);
						}

						idx += 4;
					}
					
					break;
				}
				case aris::dynamic::EEType::XYZT: {
					break;
				}
				case aris::dynamic::EEType::XYZ: {
					break;
				}
				case aris::dynamic::EEType::XYT: {
					break;
				}
				case aris::dynamic::EEType::XY: {
					break;
				}
				case aris::dynamic::EEType::X: [[fallthrough]];
				case aris::dynamic::EEType::A: {
					// x //
					{
						LargeNum sp;
						double sv, sa, sj;
						s_scurve_at(ee_p.scurve_x_, s_, &sp, &sv, &sa, &sj);
						double l = sp - ee_p.scurve_x_.pa_;
						if (l < ee_p.zone_x1_.length_ / 2.0) {
							double p[3];
							get_zone_data(l + ee_p.zone_x1_.length_ / 2.0, ee_p.zone_x1_, p);
							imp_->internal_pos_.data()[idx] = p[0];
						}
						else if (l > ee_p.zone_x1_.length_ / 2.0 + ee_p.move_x_.length_) {
							double p[3];
							get_zone_data(l - ee_p.zone_x1_.length_ / 2.0 - ee_p.move_x_.length_, ee_p.zone_x2_, p);
							imp_->internal_pos_.data()[idx] = p[0];
						}
						else {
							double p[3];
							auto& line = ee_p.move_x_.line_;
							s_compute_line_pos_at(l - ee_p.zone_x1_.length_ / 2.0, line.p0_, line.p1_, ee_p.move_x_.length_, p);
							imp_->internal_pos_.data()[idx] = p[0];
						}
						idx += 1;
					}
					break;
				}
				case aris::dynamic::EEType::UNKNOWN:
				default:
					;
				}
				break;
			}
			case Node::MoveType::Circle: {
				switch (imp_->ee_types_[i]) {
				case aris::dynamic::EEType::PE313: [[fallthrough]];
				case aris::dynamic::EEType::PE321: [[fallthrough]];
				case aris::dynamic::EEType::PE123: [[fallthrough]];
				case aris::dynamic::EEType::PM: [[fallthrough]];
				case aris::dynamic::EEType::PQ: {
					// x //
					{
						LargeNum sp;
						double sv, sa, sj;
						s_scurve_at(ee_p.scurve_x_, s_, &sp, &sv, &sa, &sj);
						double l = sp - ee_p.scurve_x_.pa_;
						if (l < ee_p.zone_x1_.length_ / 2.0) {
							get_zone_data(l + ee_p.zone_x1_.length_ / 2.0, ee_p.zone_x1_, imp_->internal_pos_.data() + idx);
						}
						else if (l > ee_p.zone_x1_.length_ / 2.0 + ee_p.move_x_.length_) {
							get_zone_data(l - ee_p.zone_x1_.length_ / 2.0 - ee_p.move_x_.length_, ee_p.zone_x2_, imp_->internal_pos_.data() + idx);
						}
						else {
							auto& c = ee_p.move_x_.circle_;
							s_compute_circle_pos_at(l - ee_p.zone_x1_.length_ / 2.0, c.p0_, c.center_, c.axis_, c.radius_, ee_p.move_x_.length_, imp_->internal_pos_.data() + idx);
						}
						idx += 3;
					}

					// a //
					{
						LargeNum sp;
						double sv, sa, sj;
						s_scurve_at(ee_p.scurve_a_, s_, &sp, &sv, &sa, &sj);
						double l = sp - ee_p.scurve_a_.pa_;
						if (l < ee_p.zone_a1_.length_ / 2.0) {
							get_zone_data(l + ee_p.zone_a1_.length_ / 2.0, ee_p.zone_a1_, imp_->internal_pos_.data() + idx);
						}
						else if (l > ee_p.zone_a1_.length_ / 2.0 + ee_p.move_a_.length_) {
							get_zone_data(l - ee_p.zone_a1_.length_ / 2.0 - ee_p.move_a_.length_, ee_p.zone_a2_, imp_->internal_pos_.data() + idx);
						}
						else {
							// in move //
							auto& quternion = ee_p.move_a_.quaternion_;
							s_compute_quaternion_at(l - ee_p.zone_a1_.length_ / 2.0, quternion.q0_, quternion.q1_, ee_p.move_a_.length_, imp_->internal_pos_.data() + idx);
						}

						idx += 4;
					}

					break;
				}
				case aris::dynamic::EEType::XYZT: {
					break;
				}
				case aris::dynamic::EEType::XYZ: {
					break;
				}
				case aris::dynamic::EEType::XYT: {
					break;
				}
				case aris::dynamic::EEType::XY: {
					break;
				}
				default:
					THROW_FILE_LINE("INVALID node when get data: only 6dof node support circle plan");
				}
			}

			}
		}

		internal_pos_to_outpos(eeTypes(), imp_->internal_pos_.data(), ee_pos);

		// 已经结束 //
		if (current_node == next_node && s_ - current_node->s_end_ >= 0.0) {
			s_ = current_node->s_end_;
			imp_->ds_ = imp_->target_ds_;
			imp_->dds_ = 0.0;
			imp_->ddds_ = 0.0;
			return 0;
		}

		// 下次会结束 //
		if (current_node == next_node && s_ + currentDs() * dt() - current_node->s_end_ >= 0.0) {
			s_ = current_node->s_end_;
			imp_->ds_ = imp_->target_ds_;
			imp_->dds_ = 0.0;
			imp_->ddds_ = 0.0;
			return current_node->id_;
		}

		// 正常运行 //
		s_ = s_ + currentDs() * dt();
		aris::Size total_count;
		moveAbsolute2(imp_->ds_, imp_->dds_, imp_->ddds_, imp_->target_ds_, 0.0, 0.0,
			imp_->max_dds_, imp_->max_ddds_, imp_->max_ddds_, imp_->dt_, 1e-10,
			imp_->ds_, imp_->dds_, imp_->ddds_, total_count);

		// 需要切换
		if (current_node->s_end_ - s_ < 0.0) {
			auto next = current_node->next_node_.exchange(nullptr);
			imp_->current_node_.store(next_node);
		}
		return current_node->id_;
	}
	auto TrajectoryGenerator::insertInitPos(std::int64_t id, const double* ee_pos)->void {
		std::lock_guard<std::recursive_mutex> lck(imp_->mu_);

		auto current_node = imp_->current_node_.load();

		// 转化 pos 表达 //
		std::vector<double> ee_pos_internal(imp_->internal_pos_size), mid_pos_internal(imp_->internal_pos_size);
		outpos_to_internal_pos(eeTypes(), ee_pos, ee_pos_internal.data());

		// 插入初始化指令 //
		auto& nodes_ = imp_->nodes_;
		auto& ins_node = nodes_.emplace_back(eeTypes().size());
		ins_node.id_ = id;

		// 初始化节点 //
		auto scurve_size = aris::dynamic::getScurveSize(eeTypes());
		std::vector<double> vel_vec(scurve_size, 1.0), acc_vec(scurve_size, 1.0), jerk_vec(scurve_size, 1.0), zone_vec(scurve_size, 0.0);
		make_node(0, &ins_node, current_node ? &*std::prev(nodes_.end(), 2) : nullptr, eeTypes(), Node::MoveType::ResetInitPos, ee_pos_internal.data(), mid_pos_internal.data()
			, vel_vec.data(), acc_vec.data(), jerk_vec.data(), zone_vec.data());

		// 设置当前 current_node_ 或 之前node的下一个值 //
		if(nodes_.size() == 1)
			imp_->current_node_.store(&ins_node);
		else
			std::prev(nodes_.end(), 2)->next_node_.store(&ins_node);
	}
	auto TrajectoryGenerator::insertLinePos(std::int64_t id, const double* ee_pos, const double* vel, const double* acc, const double* jerk, const double* zone)->void{
		std::lock_guard<std::recursive_mutex> lck(imp_->mu_);

		// 转化 pos 表达 //
		std::vector<double> ee_pos_internal(imp_->internal_pos_size), mid_pos_internal(imp_->internal_pos_size);
		outpos_to_internal_pos(eeTypes(), ee_pos, ee_pos_internal.data());

		// 如果当前指令队列为空，那么会Z=插入ResetInitPos指令 //
		auto& nodes_ = imp_->nodes_;
		if (nodes_.empty())
			insertInitPos(id, ee_pos);

		// 插入节点，循环确保成功 //
		bool insert_success = false;
		do {
			// 插入最新节点 //
			auto& last_node = *std::prev(nodes_.end());
			auto& ins_node = nodes_.emplace_back(eeTypes().size());

			// 获得需要重新规划的起点，默认为5段轨迹
			auto current_node = imp_->current_node_.load();
			auto current_iter = std::find_if(nodes_.begin(), nodes_.end(), [current_node](auto& node)->bool {
				return &node == current_node;
				});
			auto replan_iter_begin = std::next(current_iter);
			auto replan_iter_end = std::next(current_iter);
			aris::Size replan_num = 0;
			for (; std::next(replan_iter_end) != nodes_.end(); replan_iter_end++) {
				if (replan_iter_end->ee_plans_[0].move_type_ == Node::MoveType::ResetInitPos) {
					replan_iter_begin = std::next(replan_iter_end);
					replan_num = 0;
				}
				else if (replan_num > 4)
					replan_iter_begin++;
				else
					replan_num++;
			}
			replan_iter_end = nodes_.insert(std::prev(nodes_.end()), replan_iter_begin, replan_iter_end);

			// 初始化最新的节点 //
			ins_node.id_ = id;
			make_node(replan_num, &ins_node, &*std::prev(nodes_.end(), 2), eeTypes(), Node::MoveType::Line, ee_pos_internal.data(), ee_pos_internal.data(), vel, acc, jerk, zone);

			// 重规划 scurve
			auto scurve_size = aris::dynamic::getScurveSize(eeTypes());
			replan_scurve((int)scurve_size, eeTypes(), std::prev(replan_iter_begin), replan_iter_end, nodes_.end());

			// 并发设置
			insert_success = std::prev(replan_iter_begin)->next_node_.exchange(&*replan_iter_end) != nullptr || replan_num == 0;

			// 如果成功，则删除需要重新规划的节点，否则删除新插入的节点
			if (insert_success) {
				nodes_.erase(replan_iter_begin, replan_iter_end);
			}
			else {
				nodes_.erase(replan_iter_end, nodes_.end());
			}

		} while (!insert_success);
	}
	auto TrajectoryGenerator::insertCirclePos(std::int64_t id, const double* ee_pos, const double* mid_pos, const double* vel, const double* acc, const double* jerk, const double* zone)->void {
		std::lock_guard<std::recursive_mutex> lck(imp_->mu_);

		// 转化 pos 表达 //
		std::vector<double> ee_pos_internal(imp_->internal_pos_size), mid_pos_internal(imp_->internal_pos_size);
		outpos_to_internal_pos(eeTypes(), ee_pos, ee_pos_internal.data());
		outpos_to_internal_pos(eeTypes(), mid_pos, mid_pos_internal.data());

		// 如果当前指令队列为空，那么会Z=插入ResetInitPos指令 //
		auto& nodes_ = imp_->nodes_;
		if (nodes_.empty())
			insertInitPos(id, ee_pos);

		// 插入节点，循环确保成功 //
		bool insert_success = false;
		do {
			// 插入最新节点 //
			auto& last_node = *std::prev(nodes_.end());
			auto& ins_node = nodes_.emplace_back(eeTypes().size());
			
			// 获得需要重新规划的起点，默认为5段轨迹
			auto current_node = imp_->current_node_.load();
			auto current_iter = std::find_if(nodes_.begin(), nodes_.end(), [current_node](auto& node)->bool {
				return &node == current_node;
				});
			auto replan_iter_begin = std::next(current_iter);
			auto replan_iter_end = std::next(current_iter);
			aris::Size replan_num = 0;
			for (; std::next(replan_iter_end) != nodes_.end(); replan_iter_end++) {
				if (replan_iter_end->ee_plans_[0].move_type_ == Node::MoveType::ResetInitPos) {
					replan_iter_begin = std::next(replan_iter_end);
					replan_num = 0;
				}
				else if (replan_num > 4)
					replan_iter_begin++;
				else
					replan_num++;
			}
			replan_iter_end = nodes_.insert(std::prev(nodes_.end()), replan_iter_begin, replan_iter_end);

			// 初始化最新的节点 //
			ins_node.id_ = id;
			make_node(replan_num, &ins_node, &*std::prev(nodes_.end(), 2), eeTypes(), Node::MoveType::Circle, ee_pos_internal.data(), mid_pos_internal.data(), vel, acc, jerk, zone);


			// 重规划 scurve
			auto scurve_size = aris::dynamic::getScurveSize(eeTypes());
			replan_scurve((int)scurve_size, eeTypes(), std::prev(replan_iter_begin), replan_iter_end, nodes_.end());

			// 并发设置
			insert_success = std::prev(replan_iter_begin)->next_node_.exchange(&*replan_iter_end) != nullptr || replan_num == 0;

			// 如果成功，则删除需要重新规划的节点，否则删除新插入的节点
			if (insert_success) {
				nodes_.erase(replan_iter_begin, replan_iter_end);
			}
			else {
				nodes_.erase(replan_iter_end, nodes_.end());
			}

		} while (!insert_success);
	
	
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
}
