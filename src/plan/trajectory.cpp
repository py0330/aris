#include"aris/plan/trajectory.hpp"
#include"aris/plan/function.hpp"

namespace aris::plan {
	struct Node {
		enum class NodeType {
			ResetInitPos,
			Line,
			Circle,
		};
		enum class UnitType {
			Line3,
			Line1,
			Circle3,
			Rotate3,
			Onedof
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
			SCurveParam scurve_, scurve_origin_;
		};
		struct EePlanData {
			// Move Unit
			Unit x_, a_;
		};

		NodeType                type_;
		std::int64_t            id_;
		LargeNum                s_end_;
		std::vector<EePlanData> ee_plans_;
		std::atomic<Node*>      next_node_;

		~Node() = default;
		Node(aris::Size ee_size) {
			type_ = NodeType::Line;
			id_ = 1;
			s_end_ = 0;
			ee_plans_.resize(ee_size);
			next_node_.store(this);
		}
		Node(const Node& other) {
			type_ = other.type_;
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
			case aris::dynamic::EEType::RE313: {
				aris::dynamic::s_rq2re(internal_pos + internal_idx, out_pos + out_idx, "313");
				internal_idx += 4;
				out_idx += 3;
				break;
			}
			case aris::dynamic::EEType::RE321: {
				aris::dynamic::s_rq2re(internal_pos + internal_idx, out_pos + out_idx, "321");
				internal_idx += 4;
				out_idx += 3;
				break;
			}
			case aris::dynamic::EEType::RE123: {
				aris::dynamic::s_rq2re(internal_pos + internal_idx, out_pos + out_idx, "123");
				internal_idx += 4;
				out_idx += 3;
				break;
			}
			case aris::dynamic::EEType::RQ: {
				aris::dynamic::s_vc(4, internal_pos + internal_idx, out_pos + out_idx);
				internal_idx += 4;
				out_idx += 4;
				break;
			}
			case aris::dynamic::EEType::RM: {
				aris::dynamic::s_rq2rm(internal_pos + internal_idx, out_pos + out_idx);
				internal_idx += 4;
				out_idx += 9;
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
			case aris::dynamic::EEType::RE313: {
				aris::dynamic::s_re2rq(out_pos + out_idx, internal_pos + internal_idx, "313");
				internal_idx += 4;
				out_idx += 3;
				break;
			}
			case aris::dynamic::EEType::RE321: {
				aris::dynamic::s_re2rq(out_pos + out_idx, internal_pos + internal_idx, "321");
				internal_idx += 4;
				out_idx += 3;
				break;
			}
			case aris::dynamic::EEType::RE123: {
				aris::dynamic::s_re2rq(out_pos + out_idx, internal_pos + internal_idx, "123");
				internal_idx += 4;
				out_idx += 3;
				break;
			}
			case aris::dynamic::EEType::RQ: {
				aris::dynamic::s_vc(4, out_pos + out_idx, internal_pos + internal_idx);
				internal_idx += 4;
				out_idx += 4;
				break;
			}
			case aris::dynamic::EEType::RM: {
				aris::dynamic::s_rm2rq(out_pos + out_idx, internal_pos + internal_idx);
				internal_idx += 9;
				out_idx += 9;
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
				break;
			default:
				break;
			}
		}


	}

	// make & compute raw data // 
	auto s_make_line3(const double* p0, const double* p1, double* dir, double& length) {
		length = std::sqrt(
			(p1[0] - p0[0]) * (p1[0] - p0[0]) + (p1[1] - p0[1]) * (p1[1] - p0[1]) + (p1[2] - p0[2]) * (p1[2] - p0[2])
		);

		// 计算 direction //
		aris::dynamic::s_vc(3, p1, dir);
		aris::dynamic::s_vs(3, p0, dir);
		aris::dynamic::s_nv(3, 1.0 / std::max(length, 1e-10), dir);
	}
	auto s_compute_line3_at(const double* p0, const double* dir, double l, double* xyz, double dl = 0.0, double* dxyz = nullptr, double d2l = 0.0, double* d2xyz = nullptr) {
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

	auto s_make_circle3(const double* p0, const double* p1, const double* p2, double* center, double* axis, double& radius, double& length)
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
	auto s_compute_circle3_at(const double* p0, const double* center, const double* axis, double radius, double total_length, double l, double* xyz, double dl =0.0, double *dxyz = nullptr, double d2l = 0.0, double *d2xyz = nullptr)
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

	auto s_make_quaternion_data(const double* q0, const double* q1, double& length) {
		double c = aris::dynamic::s_vv(4, q0, q1);
		double q_diff[4];
		aris::dynamic::s_vc(4, -c, q0, q_diff);
		aris::dynamic::s_va(4, q1, q_diff);
			
		double s = aris::dynamic::s_norm(4, q_diff);
		
		length = 2.0 * std::atan2(s, std::abs(c));
	}
	auto s_compute_quaternion_at(const double* q0, const double* q1, double total_length, double l, double* q, double dl = 0.0, double* dq = nullptr, double d2l = 0.0, double* d2q = nullptr) {
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
	auto init_unit_l3(const double* p0, const double* p1, double vel, double acc, double jerk, double zone, Node::Unit& unit) {
		// move type //
		unit.type_ = Node::UnitType::Line3;
		
		// moves //
		std::fill_n(unit.move_.line_.p0_, 3, 0.0);
		std::fill_n(unit.move_.line_.dir_, 3, 0.0);
		aris::dynamic::s_vc(3, p0, unit.move_.line_.p0_);
		s_make_line3(p0, p1, unit.move_.line_.dir_, unit.move_.length_);

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
	auto init_unit_c3(const double* p0, const double* p1, const double* p2, double vel, double acc, double jerk, double zone, Node::Unit& unit) {
		// move type //
		unit.type_ = Node::UnitType::Circle3;
		
		// moves //
		std::fill_n(unit.move_.circle_.p0_, 3, 0.0);
		aris::dynamic::s_vc(3, p0, unit.move_.circle_.p0_);
		s_make_circle3(p0, p1, p2, unit.move_.circle_.center_, unit.move_.circle_.axis_, unit.move_.circle_.radius_, unit.move_.length_);

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
	auto init_unit_q3(const double* q0, const double* q1, double vel, double acc, double jerk, double zone, Node::Unit& unit) {
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
	auto init_unit_l1(const double* p0, const double* p1, double vel, double acc, double jerk, double zone, Node::Unit& unit) {
		double p0_[3]{ *p0, 0.0, 0.0 };
		double p1_[3]{ *p1, 0.0, 0.0 };
		
		init_unit_l3(p0_, p1_, vel, acc, jerk, zone, unit);

		unit.type_ = Node::UnitType::Line1;
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
		double real_zone = std::min({ last_u.move_.length_, last_u.zone2_.zone_value_, this_u.move_.length_ * 0.5 });

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
		double p50[4], dp50[4], d2p50[4];
		double vb;
		s_bezier3_blend_line_line(0.5, last_u.zone2_.lines_.p0_, last_u.zone2_.lines_.p1_, last_u.zone2_.lines_.p2_,
			p50, dp50, d2p50);
		s_bezier3_max_v_at(3, dp50, d2p50, std::min(last_u.scurve_.a_, this_u.scurve_.a_), vb);
		last_u.scurve_.vb_max_ = std::min({ vb, last_u.scurve_.vc_max_, this_u.scurve_.vc_max_ });
	}
	auto make_zone_and_scurve_lc(Node::Unit& last_u, Node::Unit& this_u) ->void {
		// STEP 0. 检查是否需要退化成直线
		if (this_u.type_ == Node::UnitType::Line3 && last_u.type_ == Node::UnitType::Line3) {
			make_zone_and_scurve_ll(last_u, this_u);
			return;
		}
		
		// STEP 1. 计算真实的交融半径
		double real_zone = std::min({ last_u.move_.length_, last_u.zone2_.zone_value_, this_u.move_.length_ * 0.5 });

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
		double p50[4], dp50[4], d2p50[4];
		double vb;
		s_bezier3_blend_line_circle(0.5, last_u.zone2_.line_circle_.p0_, last_u.zone2_.line_circle_.p1_
			, last_u.zone2_.line_circle_.center_, last_u.zone2_.line_circle_.axis_, last_u.zone2_.line_circle_.theta_,
			p50, dp50, d2p50);
		s_bezier3_max_v_at(3, dp50, d2p50, std::min(last_u.scurve_.a_, this_u.scurve_.a_), vb);
		last_u.scurve_.vb_max_ = std::min({ vb, last_u.scurve_.vc_max_, this_u.scurve_.vc_max_ });
	}
	auto make_zone_and_scurve_cl(Node::Unit& last_u, Node::Unit& this_u) ->void {
		// STEP 0. 检查是否需要退化成直线
		if (this_u.type_ == Node::UnitType::Line3 && last_u.type_ == Node::UnitType::Line3) {
			make_zone_and_scurve_ll(last_u, this_u);
			return;
		}
		
		// STEP 1. 计算真实的交融半径
		double real_zone = std::min({ last_u.move_.length_, last_u.zone2_.zone_value_, this_u.move_.length_ * 0.5 });

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
		double p50[4], dp50[4], d2p50[4];
		double vb;
		s_bezier3_blend_line_circle(0.5, last_u.zone2_.circle_line_.p2_, last_u.zone2_.circle_line_.p1_
			, last_u.zone2_.circle_line_.center_, last_u.zone2_.circle_line_.axis_, last_u.zone2_.circle_line_.theta_,
			p50, dp50, d2p50);
		s_bezier3_max_v_at(3, dp50, d2p50, std::min(last_u.scurve_.a_, this_u.scurve_.a_), vb);
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
		double real_zone = std::min({ last_u.move_.length_, last_u.zone2_.zone_value_, this_u.move_.length_*0.5 });

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
		double p50[4], dp50[4], d2p50[4];
		double vb;
		s_bezier3_blend_circle_circle(0.5, circles.pcenter_, circles.c1_, circles.a1_, circles.theta1_
			, circles.c2_, circles.a2_, circles.theta2_
			, p50, dp50, d2p50);
		s_bezier3_max_v_at(3, dp50, d2p50, std::min(last_u.scurve_.a_, this_u.scurve_.a_), vb);
		last_u.scurve_.vb_max_ = std::min({ vb, last_u.scurve_.vc_max_, this_u.scurve_.vc_max_ });
	}
	auto make_zone_and_scurve_qq(Node::Unit& last_u, Node::Unit& this_u)->void {
		// STEP 1. 计算真实的交融半径
		double real_zone = std::min({ last_u.move_.length_, last_u.zone2_.zone_value_, this_u.move_.length_*0.5 });

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
		double p50[4], dp50[4], d2p50[4];
		double vb;
		s_bezier3_blend_quaternion(0.5, last_u.zone2_.quaternions_.q0_, last_u.zone2_.quaternions_.q1_, last_u.zone2_.quaternions_.q2_,
			p50, dp50, d2p50);

		// 需要将四元数转化为角速度与角加速度
		double xa50[3], wa50[3];
		aris::dynamic::s_xq2xa(p50, dp50, d2p50, xa50, wa50);
		s_bezier3_max_v_at(3, wa50, xa50, std::min(last_u.scurve_.a_, this_u.scurve_.a_), vb);
		last_u.scurve_.vb_max_ = std::min({ vb, last_u.scurve_.vc_max_, this_u.scurve_.vc_max_ });
	}

	auto make_zone_and_scurve(Node::Unit& last_u, Node::Unit& this_u, bool if_need_connect) ->void {
		// 不需要融合，此时仅更改 scurve 的位置初值
		if (!if_need_connect) {
			this_u.scurve_.pb_ = last_u.scurve_.pb_ + this_u.scurve_.pb_;
			this_u.scurve_.pa_ = last_u.scurve_.pb_;
			return;
		}

		switch (last_u.type_) {
		case Node::UnitType::Line3:
			switch (this_u.type_) {
			case Node::UnitType::Line3: 
				make_zone_and_scurve_ll(last_u, this_u);
				break;
			case Node::UnitType::Circle3:
				make_zone_and_scurve_lc(last_u, this_u);
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
			}
			break;
		case Node::UnitType::Rotate3:
			switch (this_u.type_) {
			case Node::UnitType::Rotate3:
				make_zone_and_scurve_qq(last_u, this_u);
				break;
			}
			break;
		case Node::UnitType::Line1:
			switch (this_u.type_) {
			case Node::UnitType::Line1:
				make_zone_and_scurve_ll(last_u, this_u);
				break;
			}
			break;
		}

		// 更新起始的scurve数值 //
		last_u.scurve_origin_ = last_u.scurve_;
		this_u.scurve_origin_ = this_u.scurve_;
	}

	// get unit data //
	auto get_zone_data(double arc, double darc, double d2arc, const Node::Zone& z, double* p, double* dp, double* d2p)->void {
		double s, ds, d2s;

		switch (z.type_) {
		case Node::Zone::ZoneType::LL: {
			s_bezier3_arc2s(arc, darc, d2arc, z.bezier_param, s, ds, d2s);
			s_bezier3_blend_line_line(s, z.lines_.p0_, z.lines_.p1_, z.lines_.p2_, p, dp, d2p);

			// d2p_dt2 = (dp_dt)' = (dp_ds * ds)' = d2p_ds2 * ds^2 + dp_ds * d2s
			aris::dynamic::s_nv(3, ds * ds, d2p);
			aris::dynamic::s_va(3, d2s, dp, d2p);

			// dp_dt = dp_ds * ds
			aris::dynamic::s_nv(3, ds, dp);
			break;
		}
		case Node::Zone::ZoneType::LC: {
			s_bezier3_arc2s(arc, darc, d2arc, z.bezier_param, s, ds, d2s);
			s_bezier3_blend_line_circle(s, z.line_circle_.p0_, z.line_circle_.p1_, z.line_circle_.center_, z.line_circle_.axis_, z.line_circle_.theta_
				, p, dp, d2p);
			// d2p_dt2 = (dp_dt)' = (dp_ds * ds)' = d2p_ds2 * ds^2 + dp_ds * d2s
			aris::dynamic::s_nv(3, ds * ds, d2p);
			aris::dynamic::s_va(3, d2s, dp, d2p);

			// dp_dt = dp_ds * ds
			aris::dynamic::s_nv(3, ds, dp);
			break;
		}
		case Node::Zone::ZoneType::CL: {
			s_bezier3_arc2s(z.length_ - arc, -darc, -d2arc, z.bezier_param, s, ds, d2s);
			s_bezier3_blend_line_circle(s, z.circle_line_.p2_, z.circle_line_.p1_, z.circle_line_.center_, z.circle_line_.axis_, z.circle_line_.theta_
				, p, dp, d2p);
			// d2p_dt2 = (dp_dt)' = (dp_ds * ds)' = d2p_ds2 * ds^2 + dp_ds * d2s
			aris::dynamic::s_nv(3, ds * ds, d2p);
			aris::dynamic::s_va(3, d2s, dp, d2p);

			// dp_dt = dp_ds * ds
			aris::dynamic::s_nv(3, ds, dp);
			break;
		}
		case Node::Zone::ZoneType::CC: {
			s_bezier3_arc2s(arc, darc, d2arc, z.bezier_param, s, ds, d2s);
			s_bezier3_blend_circle_circle(s, z.circles_.pcenter_, z.circles_.c1_, z.circles_.a1_, z.circles_.theta1_
				, z.circles_.c2_, z.circles_.a2_, z.circles_.theta2_
				, p, dp, d2p);
			// d2p_dt2 = (dp_dt)' = (dp_ds * ds)' = d2p_ds2 * ds^2 + dp_ds * d2s
			aris::dynamic::s_nv(3, ds * ds, d2p);
			aris::dynamic::s_va(3, d2s, dp, d2p);

			// dp_dt = dp_ds * ds
			aris::dynamic::s_nv(3, ds, dp);
			break;
		}
		case Node::Zone::ZoneType::QQ: {
			s_bezier3_arc2s(arc, darc, d2arc, z.bezier_param, s, ds, d2s);
			s_bezier3_blend_quaternion(s, z.quaternions_.q0_, z.quaternions_.q1_, z.quaternions_.q2_, p, dp, d2p);
			// d2p_dt2 = (dp_dt)' = (dp_ds * ds)' = d2p_ds2 * ds^2 + dp_ds * d2s
			aris::dynamic::s_nv(4, ds * ds, d2p);
			aris::dynamic::s_va(4, d2s, dp, d2p);

			// dp_dt = dp_ds * ds
			aris::dynamic::s_nv(4, ds, dp);
			break;
		}
		default:
			break;
		}
	}
	auto get_unit_data(double arc, const Node::Unit& u, double* p, double* dp_darc, double* d2p_darc2)->void {
		switch (u.type_) {
		case aris::plan::Node::UnitType::Line3: {
			LargeNum sp;
			double sv, sa, sj;
			s_scurve_at(u.scurve_, arc, &sp, &sv, &sa, &sj);
			double l = sp - u.scurve_.pa_;
			if (l < u.zone1_.length_ / 2.0) {
				get_zone_data(l + u.zone1_.length_ / 2.0, sv, sa, u.zone1_, p, dp_darc, d2p_darc2);
			}
			else if (l > u.zone1_.length_ / 2.0 + u.move_.length_) {
				get_zone_data(l - u.zone1_.length_ / 2.0 - u.move_.length_, sv, sa, u.zone2_, p, dp_darc, d2p_darc2);
			}
			else {
				auto& line = u.move_.line_;
				s_compute_line3_at(line.p0_, line.dir_, l - u.zone1_.length_ / 2.0, p, sv, dp_darc, sa, d2p_darc2);
			}
			break;
		}
		case aris::plan::Node::UnitType::Circle3: {
			LargeNum sp;
			double sv, sa, sj;
			s_scurve_at(u.scurve_, arc, &sp, &sv, &sa, &sj);
			double l = sp - u.scurve_.pa_;
			if (l < u.zone1_.length_ / 2.0) {
				get_zone_data(l + u.zone1_.length_ / 2.0, sv, sa, u.zone1_, p, dp_darc, d2p_darc2);
			}
			else if (l > u.zone1_.length_ / 2.0 + u.move_.length_) {
				get_zone_data(l - u.zone1_.length_ / 2.0 - u.move_.length_, sv, sa, u.zone2_, p, dp_darc, d2p_darc2);
			}
			else {
				auto& c = u.move_.circle_;
				s_compute_circle3_at(c.p0_, c.center_, c.axis_, c.radius_, u.move_.length_,
					l - u.zone1_.length_ / 2.0, p, sv, dp_darc, sa, d2p_darc2);
			}
			break;
		}
		case aris::plan::Node::UnitType::Rotate3: {
			LargeNum sp;
			double sv, sa, sj;
			s_scurve_at(u.scurve_, arc, &sp, &sv, &sa, &sj);
			double l = sp - u.scurve_.pa_;
			if (l < u.zone1_.length_ / 2.0) {
				get_zone_data(l + u.zone1_.length_ / 2.0, sv, sa, u.zone1_, p, dp_darc, d2p_darc2);
			}
			else if (l > u.zone1_.length_ / 2.0 + u.move_.length_) {
				get_zone_data(l - u.zone1_.length_ / 2.0 - u.move_.length_, sv, sa, u.zone2_, p, dp_darc, d2p_darc2);
			}
			else {
				// in move //
				auto& quternion = u.move_.quaternion_;
				s_compute_quaternion_at(quternion.q0_, quternion.q1_, u.move_.length_, l - u.zone1_.length_ / 2.0, p, sv, dp_darc, sa, d2p_darc2);
			}
			break;
		}
		case aris::plan::Node::UnitType::Line1: {
			double p_[3], v_[3], a_[3];
			LargeNum sp;
			double sv, sa, sj;
			s_scurve_at(u.scurve_, arc, &sp, &sv, &sa, &sj);
			double l = sp - u.scurve_.pa_;
			if (l < u.zone1_.length_ / 2.0) {
				get_zone_data(l + u.zone1_.length_ / 2.0, sv, sa, u.zone1_, p_, v_, a_);
			}
			else if (l > u.zone1_.length_ / 2.0 + u.move_.length_) {
				get_zone_data(l - u.zone1_.length_ / 2.0 - u.move_.length_, sv, sa, u.zone2_, p_, v_, a_);
			}
			else {
				auto& line = u.move_.line_;
				s_compute_line3_at(line.p0_, line.dir_, l - u.zone1_.length_ / 2.0, p_, sv, v_, sa, a_);
			}
			p[0] = p_[0];
			dp_darc[0] = v_[0];
			d2p_darc2[0] = a_[0];
			break;
		}
		default:
			break;
		}



	}


	// make nodes //
	auto make_node(aris::Size replan_num, Node* this_node, Node* last_node, const std::vector<aris::dynamic::EEType>& ee_types,
		Node::NodeType node_type, const double* ee_pos, const double* mid_pos, const double* vel, const double* acc, const double* jerk, const double* zone
	)->void {
		
		this_node->type_ = node_type;
		
		// 更新本段轨迹的 move //
		for (Size i{ 0 }, pos_idx{ 0 }, vel_idx{ 0 }; i < ee_types.size(); ++i) {
			auto this_p = &this_node->ee_plans_[i];

			switch (ee_types[i]) {
			case aris::dynamic::EEType::PE313: [[fallthrough]];
			case aris::dynamic::EEType::PE321: [[fallthrough]];
			case aris::dynamic::EEType::PE123: [[fallthrough]];
			case aris::dynamic::EEType::PM: [[fallthrough]];
			case aris::dynamic::EEType::PQ: {
				auto ee_xyz = ee_pos + pos_idx;
				auto mid_xyz = mid_pos + pos_idx;
				auto v_xyz = vel[vel_idx];
				auto a_xyz = acc[vel_idx];
				auto j_xyz = jerk[vel_idx];
				auto z_xyz = zone[vel_idx];

				auto ee_abc = ee_pos + pos_idx + 3;
				auto mid_abc = mid_pos + pos_idx + 3;
				auto v_abc = vel[vel_idx + 1];
				auto a_abc = acc[vel_idx + 1];
				auto j_abc = jerk[vel_idx + 1];
				auto z_abc = zone[vel_idx + 1];
				
				switch (node_type) {
				case aris::plan::Node::NodeType::ResetInitPos: {
					// init //
					init_unit(Node::UnitType::Line3, ee_xyz, mid_xyz, ee_xyz, v_xyz, a_xyz, j_xyz, z_xyz, this_p->x_);
					init_unit(Node::UnitType::Rotate3, ee_abc, mid_abc, ee_abc, v_abc, a_abc, j_abc, z_abc, this_p->a_);
					break;
				}
				case aris::plan::Node::NodeType::Line: {
					auto last_p = &last_node->ee_plans_[i];
					double p_end[4];
					// xyz //
					s_compute_data_at_end(last_p->x_, p_end);
					init_unit(Node::UnitType::Line3, p_end, mid_xyz, ee_xyz, v_xyz, a_xyz, j_xyz, z_xyz, this_p->x_);
					make_zone_and_scurve(last_p->x_, this_p->x_, replan_num > 0);

					// abc //
					s_compute_data_at_end(last_p->a_, p_end);
					init_unit(Node::UnitType::Rotate3, p_end, mid_abc, ee_abc, v_abc, a_abc, j_abc, z_abc, this_p->a_);
					make_zone_and_scurve(last_p->a_, this_p->a_, replan_num > 0);
					break;
				}
				case aris::plan::Node::NodeType::Circle: {
					auto last_p = &last_node->ee_plans_[i];
					double p_end[4];
					// xyz //
					s_compute_data_at_end(last_p->x_, p_end);
					init_unit(Node::UnitType::Circle3, p_end, mid_xyz, ee_xyz, v_xyz, a_xyz, j_xyz, z_xyz, this_p->x_);
					make_zone_and_scurve(last_p->x_, this_p->x_, replan_num > 0);

					// abc //
					s_compute_data_at_end(last_p->a_, p_end);
					init_unit(Node::UnitType::Rotate3, p_end, mid_abc, ee_abc, v_abc, a_abc, j_abc, z_abc, this_p->a_);
					make_zone_and_scurve(last_p->a_, this_p->a_, replan_num > 0);
					break;
				}
				}
				
				pos_idx += 7;
				vel_idx += 2;
				break;
			}
			case aris::dynamic::EEType::RE313: [[fallthrough]];
			case aris::dynamic::EEType::RE321: [[fallthrough]];
			case aris::dynamic::EEType::RE123: [[fallthrough]];
			case aris::dynamic::EEType::RM: [[fallthrough]];
			case aris::dynamic::EEType::RQ: {
				auto ee_abc = ee_pos + pos_idx;
				auto mid_abc = mid_pos + pos_idx;
				auto v_abc = vel[vel_idx];
				auto a_abc = acc[vel_idx];
				auto j_abc = jerk[vel_idx];
				auto z_abc = zone[vel_idx];

				switch (node_type) {
				case aris::plan::Node::NodeType::ResetInitPos: {
					// init //
					init_unit(Node::UnitType::Rotate3, ee_abc, mid_abc, ee_abc, v_abc, a_abc, j_abc, z_abc, this_p->a_);
					break;
				}
				case aris::plan::Node::NodeType::Line: {
					auto last_p = &last_node->ee_plans_[i];
					double p_end[4];
					// abc //
					s_compute_data_at_end(last_p->a_, p_end);
					init_unit(Node::UnitType::Rotate3, p_end, mid_abc, ee_abc, v_abc, a_abc, j_abc, z_abc, this_p->a_);
					make_zone_and_scurve(last_p->a_, this_p->a_, replan_num > 0);
					break;
				}
				case aris::plan::Node::NodeType::Circle: {
					auto last_p = &last_node->ee_plans_[i];
					double p_end[4];
					// abc //
					s_compute_data_at_end(last_p->a_, p_end);
					init_unit(Node::UnitType::Rotate3, p_end, mid_abc, ee_abc, v_abc, a_abc, j_abc, z_abc, this_p->a_);
					make_zone_and_scurve(last_p->a_, this_p->a_, replan_num > 0);
					break;
				}
				}

				pos_idx += 4;
				vel_idx += 1;
				break;
			}
			case aris::dynamic::EEType::XYZT: {
				auto ee_xyz = ee_pos + pos_idx;
				auto mid_xyz = mid_pos + pos_idx;
				auto v_xyz = vel[vel_idx];
				auto a_xyz = acc[vel_idx];
				auto j_xyz = jerk[vel_idx];
				auto z_xyz = zone[vel_idx];

				auto ee_abc = ee_pos + pos_idx + 3;
				auto mid_abc = mid_pos + pos_idx + 3;
				auto v_abc = vel[vel_idx + 1];
				auto a_abc = acc[vel_idx + 1];
				auto j_abc = jerk[vel_idx + 1];
				auto z_abc = zone[vel_idx + 1];

				switch (node_type) {
				case aris::plan::Node::NodeType::ResetInitPos: {
					// init //
					init_unit(Node::UnitType::Line3, ee_xyz, mid_xyz, ee_xyz, v_xyz, a_xyz, j_xyz, z_xyz, this_p->x_);
					init_unit(Node::UnitType::Line1, ee_abc, mid_abc, ee_abc, v_abc, a_abc, j_abc, z_abc, this_p->a_);
					break;
				}
				case aris::plan::Node::NodeType::Line: {
					auto last_p = &last_node->ee_plans_[i];
					double p_end[4];
					// xyz //
					s_compute_data_at_end(last_p->x_, p_end);
					init_unit(Node::UnitType::Line3, p_end, mid_xyz, ee_xyz, v_xyz, a_xyz, j_xyz, z_xyz, this_p->x_);
					make_zone_and_scurve(last_p->x_, this_p->x_, replan_num > 0);

					// abc //
					s_compute_data_at_end(last_p->a_, p_end);
					init_unit(Node::UnitType::Line1, p_end, mid_abc, ee_abc, v_abc, a_abc, j_abc, z_abc, this_p->a_);
					make_zone_and_scurve(last_p->a_, this_p->a_, replan_num > 0);
					break;
				}
				case aris::plan::Node::NodeType::Circle: {
					auto last_p = &last_node->ee_plans_[i];
					double p_end[4];
					// xyz //
					s_compute_data_at_end(last_p->x_, p_end);
					init_unit(Node::UnitType::Circle3, p_end, mid_xyz, ee_xyz, v_xyz, a_xyz, j_xyz, z_xyz, this_p->x_);
					make_zone_and_scurve(last_p->x_, this_p->x_, replan_num > 0);

					// abc //
					s_compute_data_at_end(last_p->a_, p_end);
					init_unit(Node::UnitType::Line1, p_end, mid_abc, ee_abc, v_abc, a_abc, j_abc, z_abc, this_p->a_);
					make_zone_and_scurve(last_p->a_, this_p->a_, replan_num > 0);
					break;
				}
				}
				
				pos_idx += 4;
				vel_idx += 2;
				break;
			}
			case aris::dynamic::EEType::XYZ: {
				auto ee_xyz = ee_pos + pos_idx;
				auto mid_xyz = mid_pos + pos_idx;
				auto v_xyz = vel[vel_idx];
				auto a_xyz = acc[vel_idx];
				auto j_xyz = jerk[vel_idx];
				auto z_xyz = zone[vel_idx];

				switch (node_type) {
				case aris::plan::Node::NodeType::ResetInitPos: {
					// init //
					init_unit(Node::UnitType::Line3, ee_xyz, mid_xyz, ee_xyz, v_xyz, a_xyz, j_xyz, z_xyz, this_p->x_);
					break;
				}
				case aris::plan::Node::NodeType::Line: {
					auto last_p = &last_node->ee_plans_[i];
					double p_end[4];
					// xyz //
					s_compute_data_at_end(last_p->x_, p_end);
					init_unit(Node::UnitType::Line3, p_end, mid_xyz, ee_xyz, v_xyz, a_xyz, j_xyz, z_xyz, this_p->x_);
					make_zone_and_scurve(last_p->x_, this_p->x_, replan_num > 0);
					break;
				}
				case aris::plan::Node::NodeType::Circle: {
					auto last_p = &last_node->ee_plans_[i];
					double p_end[4];
					// xyz //
					s_compute_data_at_end(last_p->x_, p_end);
					init_unit(Node::UnitType::Circle3, p_end, mid_xyz, ee_xyz, v_xyz, a_xyz, j_xyz, z_xyz, this_p->x_);
					make_zone_and_scurve(last_p->x_, this_p->x_, replan_num > 0);
					break;
				}
				}

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
				auto ee = ee_pos + pos_idx;
				auto mid = mid_pos + pos_idx;
				auto v = vel[vel_idx];
				auto a = acc[vel_idx];
				auto j = jerk[vel_idx];
				auto z = zone[vel_idx];

				switch (node_type) {
				case aris::plan::Node::NodeType::ResetInitPos:
					// init //
					init_unit(Node::UnitType::Line1, ee, mid, ee, v, a, j, z, this_p->x_);
					break;
				case aris::plan::Node::NodeType::Line: [[fallthrough]];
				case aris::plan::Node::NodeType::Circle: {
					auto last_p = &last_node->ee_plans_[i];
					double p_end[4];

					// xyz //
					s_compute_data_at_end(last_p->x_, p_end);
					init_unit(Node::UnitType::Line1, p_end, mid, ee, v, a, j, z, this_p->x_);
					make_zone_and_scurve(last_p->x_, this_p->x_, replan_num > 0);

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
	auto replan_nodes(int scurve_size, const std::vector<aris::dynamic::EEType> &ee_types, std::list<Node>::iterator last, std::list<Node>::iterator begin, std::list<Node>::iterator end) {
		// 构造 scurve list //
		std::list<SCurveNode> ins_scurve_list, ins_scurve_origin_list;
		LargeNum t0;
		for (auto iter = begin; iter != end; ++iter) {
			ins_scurve_list.push_back(SCurveNode{});
			auto& scurve_node = ins_scurve_list.back();
			scurve_node.params_.reserve(scurve_size);

			ins_scurve_origin_list.push_back(SCurveNode{});
			auto& scurve_node_origin = ins_scurve_origin_list.back();
			scurve_node_origin.params_.reserve(scurve_size);

			for (int i = 0; i < ee_types.size();++i) {
				auto& ee_p = iter->ee_plans_[i];
				switch (ee_types[i]) {
				case aris::dynamic::EEType::PE313: [[fallthrough]];
				case aris::dynamic::EEType::PE321: [[fallthrough]];
				case aris::dynamic::EEType::PE123: [[fallthrough]];
				case aris::dynamic::EEType::PM: [[fallthrough]];
				case aris::dynamic::EEType::PQ: [[fallthrough]];
				case aris::dynamic::EEType::XYZT: [[fallthrough]];
				case aris::dynamic::EEType::XYT: {
					begin->ee_plans_[i].x_.scurve_.t0_ = last->ee_plans_[i].x_.scurve_.t0_ + last->ee_plans_[i].x_.scurve_.T_;
					begin->ee_plans_[i].a_.scurve_.t0_ = last->ee_plans_[i].a_.scurve_.t0_ + last->ee_plans_[i].a_.scurve_.T_;
					scurve_node.params_.push_back(ee_p.x_.scurve_);
					scurve_node.params_.push_back(ee_p.a_.scurve_);
					scurve_node_origin.params_.push_back(ee_p.x_.scurve_origin_);
					scurve_node_origin.params_.push_back(ee_p.a_.scurve_origin_);
					break;
				}
				case aris::dynamic::EEType::RE313: [[fallthrough]];
				case aris::dynamic::EEType::RE321: [[fallthrough]];
				case aris::dynamic::EEType::RE123: [[fallthrough]];
				case aris::dynamic::EEType::RM: [[fallthrough]];
				case aris::dynamic::EEType::RQ: {
					begin->ee_plans_[i].a_.scurve_.t0_ = last->ee_plans_[i].a_.scurve_.t0_ + last->ee_plans_[i].a_.scurve_.T_;
					scurve_node.params_.push_back(ee_p.a_.scurve_);
					scurve_node_origin.params_.push_back(ee_p.a_.scurve_origin_);
					break;
				}
				case aris::dynamic::EEType::XYZ: [[fallthrough]];
				case aris::dynamic::EEType::XY: [[fallthrough]];
				case aris::dynamic::EEType::X: [[fallthrough]];
				case aris::dynamic::EEType::A: {
					begin->ee_plans_[i].x_.scurve_.t0_ = last->ee_plans_[i].x_.scurve_.t0_ + last->ee_plans_[i].x_.scurve_.T_;
					scurve_node.params_.push_back(ee_p.x_.scurve_);
					scurve_node_origin.params_.push_back(ee_p.x_.scurve_origin_);
					break;
				}
				case aris::dynamic::EEType::UNKNOWN:
					break;
				default:
					break;
				}
			}
		}

		// 进行规划 //
		if (s_scurve_make_nodes(ins_scurve_list.begin(), ins_scurve_list.end()) != 0) {
			// 如果失败，则改为原始数据，因为原始数据一定会成功
			// 但是原始数据的首个节点的起始速度不对，应改为上次优化后的数据
			for (int i = 0; i < ins_scurve_list.front().params_.size(); ++i) {
				// param1 //
				auto& origin_param = ins_scurve_origin_list.front().params_[i];
				auto& param = ins_scurve_list.front().params_[i];

				double vb = origin_param.vb_;

				s_scurve_plan_eliminate_optimization(vb, param);

				std::swap(origin_param, param);
			}
			// 交换优化 //
			std::swap(ins_scurve_list, ins_scurve_origin_list);


			if (s_scurve_make_nodes(std::next(ins_scurve_list.begin()), ins_scurve_list.end()) != 0) {
				return -1;
			}
		}


			

		// 将规划好的 scurve 返回到 nodes 中的 origin 位置 //
		auto scurve_iter = ins_scurve_list.begin();
		for (auto iter = begin; iter != end; ++iter) {
			auto& scurve_node = *scurve_iter;
			scurve_iter++;
			iter->s_end_ = scurve_node.params_[0].t0_ + scurve_node.params_[0].T_;
			for (int i = 0, s_idx = 0; i < iter->ee_plans_.size(); ++i) {
				auto& ee_p = iter->ee_plans_[i];
				switch (ee_types[i]) {
				case aris::dynamic::EEType::PE313: [[fallthrough]];
				case aris::dynamic::EEType::PE321: [[fallthrough]];
				case aris::dynamic::EEType::PE123: [[fallthrough]];
				case aris::dynamic::EEType::PM: [[fallthrough]];
				case aris::dynamic::EEType::PQ: [[fallthrough]];
				case aris::dynamic::EEType::XYZT: [[fallthrough]];
				case aris::dynamic::EEType::XYT: {
					ee_p.x_.scurve_origin_ = scurve_node.params_[s_idx];
					s_idx++;
					ee_p.a_.scurve_origin_ = scurve_node.params_[s_idx];
					s_idx++;
					break;
				}
				case aris::dynamic::EEType::RE313: [[fallthrough]];
				case aris::dynamic::EEType::RE321: [[fallthrough]];
				case aris::dynamic::EEType::RE123: [[fallthrough]];
				case aris::dynamic::EEType::RM: [[fallthrough]];
				case aris::dynamic::EEType::RQ: {
					ee_p.a_.scurve_origin_ = scurve_node.params_[s_idx];
					s_idx++;
					break;
				}
				case aris::dynamic::EEType::XYZ: [[fallthrough]];
				case aris::dynamic::EEType::XY: [[fallthrough]];
				case aris::dynamic::EEType::X: [[fallthrough]];
				case aris::dynamic::EEType::A: {
					ee_p.x_.scurve_origin_ = scurve_node.params_[s_idx];
					s_idx++;
					break;
				}
				case aris::dynamic::EEType::UNKNOWN:
					break;
				default:
					break;
				}
			}
		}

		// 优化节点 //
		s_optimize_scurve_adjacent_nodes(ins_scurve_list.begin(), ins_scurve_list.end());

		// 将规划好的 scurve 返回到 nodes 中的优化后的位置 //
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
				case aris::dynamic::EEType::PQ: [[fallthrough]];
				case aris::dynamic::EEType::XYZT: [[fallthrough]];
				case aris::dynamic::EEType::XYT: {
					ee_p.x_.scurve_ = scurve_node.params_[s_idx];
					s_idx++;
					ee_p.a_.scurve_ = scurve_node.params_[s_idx];
					s_idx++;
					break;
				}
				case aris::dynamic::EEType::RE313: [[fallthrough]];
				case aris::dynamic::EEType::RE321: [[fallthrough]];
				case aris::dynamic::EEType::RE123: [[fallthrough]];
				case aris::dynamic::EEType::RM: [[fallthrough]];
				case aris::dynamic::EEType::RQ: {
					ee_p.a_.scurve_ = scurve_node.params_[s_idx];
					s_idx++;
					break;
				}
				case aris::dynamic::EEType::XYZ: [[fallthrough]];
				case aris::dynamic::EEType::XY: [[fallthrough]];
				case aris::dynamic::EEType::X: [[fallthrough]];
				case aris::dynamic::EEType::A: {
					ee_p.x_.scurve_ = scurve_node.params_[s_idx];
					s_idx++;
					break;
				}
				case aris::dynamic::EEType::UNKNOWN:
					break;
				default:
					break;
				}
			}
			ins_scurve_list.pop_front();
			iter->next_node_.store(std::next(iter) == end ? &*iter : &*std::next(iter));
		}

		return 0;
	}
	auto get_node_data(const std::vector<aris::dynamic::EEType> &ee_types, const Node* current_node, LargeNum s, double ds, double dds, double ddds,
		double* internal_pos, double* internal_vel, double* internal_acc) 
	{
		int idx = 0;
		for (int i = 0; i < ee_types.size(); ++i) {
			auto& ee_p = current_node->ee_plans_[i];
			switch (ee_types[i]) {
			case aris::dynamic::EEType::PE313: [[fallthrough]];
			case aris::dynamic::EEType::PE321: [[fallthrough]];
			case aris::dynamic::EEType::PE123: [[fallthrough]];
			case aris::dynamic::EEType::PM: [[fallthrough]];
			case aris::dynamic::EEType::PQ: {
				// x //
				get_unit_data(s, ee_p.x_, internal_pos + idx, internal_vel + idx, internal_acc + idx);
				idx += 3;

				// a //
				get_unit_data(s, ee_p.a_, internal_pos + idx, internal_vel + idx, internal_acc + idx);
				idx += 4;
				break;
			}
			case aris::dynamic::EEType::RE313: [[fallthrough]];
			case aris::dynamic::EEType::RE321: [[fallthrough]];
			case aris::dynamic::EEType::RE123: [[fallthrough]];
			case aris::dynamic::EEType::RM: [[fallthrough]];
			case aris::dynamic::EEType::RQ: {
				// a //
				get_unit_data(s, ee_p.a_, internal_pos + idx, internal_vel + idx, internal_acc + idx);
				idx += 4;
				break;
			}
			case aris::dynamic::EEType::XYZT: {
				// x //
				get_unit_data(s, ee_p.x_, internal_pos + idx, internal_vel + idx, internal_acc + idx);
				idx += 3;

				// t //
				get_unit_data(s, ee_p.a_, internal_pos + idx, internal_vel + idx, internal_acc + idx);
				idx += 1;
				break;
			}
			case aris::dynamic::EEType::XYZ: {
				// x //
				get_unit_data(s, ee_p.x_, internal_pos + idx, internal_vel + idx, internal_acc + idx);
				idx += 3;
				break;
			}
			case aris::dynamic::EEType::XYT: {
				break;
			}
			case aris::dynamic::EEType::XY: {
				break;
			}
			case aris::dynamic::EEType::A: [[fallthrough]];
			case aris::dynamic::EEType::X: {
				// x //
				get_unit_data(s, ee_p.x_, internal_pos + idx, internal_vel + idx, internal_acc + idx);
				idx += 1;

				break;
			}
			default:
				THROW_FILE_LINE("INVALID node when get data: only 6dof node support circle plan");
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
		std::vector<aris::dynamic::EEType> ee_types_;
		aris::Size outpos_size_{ 0 }, outvel_size_{ 0 }, internal_pos_size{ 0 };
		double* internal_pos_{ nullptr }, * internal_vel_{ nullptr }, * internal_acc_{ nullptr };
		std::vector<double> internal_vec_;

		// 规划节点 //
		int max_replan_num_{ 10 };
		std::list<Node> nodes_;
		std::atomic<Node*> current_node_;

		// 互斥区，保护访问
		std::recursive_mutex mu_;

		auto insert_node(Node::NodeType move_type, std::int64_t id, const double* ee_pos, const double* mid_pos, const double* vel, const double* acc, const double* jerk, const double* zone)->void {
			std::lock_guard<std::recursive_mutex> lck(mu_);

			// 转化 pos 表达 //
			std::vector<double> ee_pos_internal(internal_pos_size), mid_pos_internal(internal_pos_size);
			outpos_to_internal_pos(ee_types_, ee_pos, ee_pos_internal.data());
			outpos_to_internal_pos(ee_types_, mid_pos, mid_pos_internal.data());

			// 插入节点，循环确保成功 //
			bool insert_success = false;
			do {
				// 插入最新节点 //
				auto& last_node = *std::prev(nodes_.end());
				auto& ins_node = nodes_.emplace_back(ee_types_.size());

				// 获得需要重新规划的起点，默认为5段轨迹
				auto current_node = current_node_.load();
				auto current_iter = std::find_if(nodes_.begin(), nodes_.end(), [current_node](auto& node)->bool {
					return &node == current_node;
					});
				auto replan_iter_begin = std::next(current_iter);
				auto replan_iter_end = std::next(current_iter);
				aris::Size replan_num = 0;
				for (; std::next(replan_iter_end) != nodes_.end(); replan_iter_end++) {
					if (replan_iter_end->type_ == Node::NodeType::ResetInitPos) {
						replan_iter_begin = std::next(replan_iter_end);
						replan_num = 0;
					}
					else if (replan_num > max_replan_num_)
						replan_iter_begin++;
					else
						replan_num++;
				}
				replan_iter_end = nodes_.insert(std::prev(nodes_.end()), replan_iter_begin, replan_iter_end);

				// 初始化最新的节点 //
				ins_node.id_ = id;
				make_node(replan_num, &ins_node, &*std::prev(nodes_.end(), 2), ee_types_, move_type, ee_pos_internal.data(), mid_pos_internal.data(), vel, acc, jerk, zone);

				// 重规划 scurve
				auto scurve_size = aris::dynamic::getEeTypeScurveSize(ee_types_.size(), ee_types_.data());
				auto replan_ret = replan_nodes((int)scurve_size, ee_types_, std::prev(replan_iter_begin), replan_iter_end, nodes_.end());

				// 查看是否冲规划成功，如果规划失败，说明当前的速度过大，融合转弯区后无法减速达到要求。
				if (replan_ret != 0) {
					nodes_.erase(replan_iter_end, std::prev(nodes_.end()));
					make_node(0, &ins_node, &*std::prev(nodes_.end(), 2), ee_types_, move_type, ee_pos_internal.data(), mid_pos_internal.data(), vel, acc, jerk, zone);
					replan_nodes((int)scurve_size, ee_types_, std::prev(nodes_.end(), 2), std::prev(nodes_.end(), 1), nodes_.end());
					std::prev(nodes_.end(), 2)->next_node_.exchange(&ins_node);
					insert_success = true;
				}
				else {
					// 并发设置
					insert_success = std::prev(replan_iter_begin)->next_node_.exchange(&*replan_iter_end) != nullptr || replan_num == 0;

					// 如果成功，则删除需要重新规划的节点，否则删除新插入的节点
					if (insert_success) {
						nodes_.erase(replan_iter_begin, replan_iter_end);
					}
					else {
						nodes_.erase(replan_iter_end, nodes_.end());
					}
				}
			} while (!insert_success);
		}
	};
	auto TrajectoryGenerator::eeTypes()const-> const std::vector<aris::dynamic::EEType>& {
		return imp_->ee_types_;
	}
	auto TrajectoryGenerator::setEeTypes(const std::vector<aris::dynamic::EEType>& ee_types)->void {
		imp_->ee_types_ = ee_types;
		imp_->outpos_size_ = aris::dynamic::getEETypePosSize(ee_types.size(), ee_types.data());

		auto &internal_pos_size = imp_->internal_pos_size;
		auto& out_vel_size = imp_->outvel_size_;
		for (auto type : ee_types) {
			switch (type) {
			case aris::dynamic::EEType::PE313: [[fallthrough]];
			case aris::dynamic::EEType::PE321: [[fallthrough]];
			case aris::dynamic::EEType::PE123: [[fallthrough]];
			case aris::dynamic::EEType::PM: [[fallthrough]];
			case aris::dynamic::EEType::PQ:
				internal_pos_size += 7;
				out_vel_size += 6;
				break;
			case aris::dynamic::EEType::RE313: [[fallthrough]];
			case aris::dynamic::EEType::RE321: [[fallthrough]];
			case aris::dynamic::EEType::RE123: [[fallthrough]];
			case aris::dynamic::EEType::RM: [[fallthrough]];
			case aris::dynamic::EEType::RQ:
				internal_pos_size += 4;
				out_vel_size += 3;
				break;
			case aris::dynamic::EEType::XYZT:
				internal_pos_size += 4;
				out_vel_size += 4;
				break;
			case aris::dynamic::EEType::XYZ:
				internal_pos_size += 3;
				out_vel_size += 3;
				break;
			case aris::dynamic::EEType::XYT:
				internal_pos_size += 3;
				out_vel_size += 3;
				break;
			case aris::dynamic::EEType::XY:
				internal_pos_size += 2;
				out_vel_size += 2;
				break;
			case aris::dynamic::EEType::X:
				internal_pos_size += 1;
				out_vel_size += 1;
				break;
			case aris::dynamic::EEType::A:
				internal_pos_size += 1;
				out_vel_size += 1;
				break;
			case aris::dynamic::EEType::UNKNOWN:
				break;
			default:
				break;
			}
		}

		imp_->internal_vec_.resize(3 * internal_pos_size);
		imp_->internal_pos_ = imp_->internal_vec_.data() + 0 * internal_pos_size;
		imp_->internal_vel_ = imp_->internal_vec_.data() + 1 * internal_pos_size;
		imp_->internal_acc_ = imp_->internal_vec_.data() + 2 * internal_pos_size;
	}
	auto TrajectoryGenerator::maxReplanNum()const->int {
		return imp_->max_replan_num_;
	}
	auto TrajectoryGenerator::setMaxReplanNum(int max_replan_num = 10) -> void {
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
	auto TrajectoryGenerator::leftTotalS()const->double {
		return imp_->nodes_.back().s_end_ - imp_->s_;
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
				imp_->ds_ = target_ds;
				imp_->dds_ = 0.0;
				imp_->ddds_ = 0.0;
				get_node_data(eeTypes(), current_node, s_, imp_->ds_, imp_->dds_, imp_->ddds_, imp_->internal_pos_, imp_->internal_vel_, imp_->internal_acc_);
				internal_pos_to_outpos(eeTypes(), imp_->internal_pos_, ee_pos);
				if (ee_acc) {
					aris::dynamic::s_nv(imp_->internal_pos_size, imp_->ds_ * imp_->ds_, imp_->internal_acc_);
					aris::dynamic::s_va(imp_->internal_pos_size, imp_->dds_, imp_->internal_vel_, imp_->internal_acc_);
					aris::dynamic::s_vc(imp_->internal_pos_size, imp_->internal_acc_, ee_acc);
				}
				if (ee_vel) {
					aris::dynamic::s_nv(imp_->internal_pos_size, imp_->ds_, imp_->internal_vel_);
					aris::dynamic::s_vc(imp_->internal_pos_size, imp_->internal_vel_, ee_vel);
				}
				
				return 0;
			}
			// check 是否局部结束，即下一条指令是 init
			else if (current_node->type_ != Node::NodeType::ResetInitPos && next_node->type_ == Node::NodeType::ResetInitPos) {
				s_ = current_node->s_end_;
				imp_->ds_ = target_ds;
				imp_->dds_ = 0.0;
				imp_->ddds_ = 0.0;
				get_node_data(eeTypes(), current_node, s_, imp_->ds_, imp_->dds_, imp_->ddds_, imp_->internal_pos_, imp_->internal_vel_, imp_->internal_acc_);
				internal_pos_to_outpos(eeTypes(), imp_->internal_pos_, ee_pos);
				if (ee_acc) {
					aris::dynamic::s_nv(imp_->internal_pos_size, imp_->ds_ * imp_->ds_, imp_->internal_acc_);
					aris::dynamic::s_va(imp_->internal_pos_size, imp_->dds_, imp_->internal_vel_, imp_->internal_acc_);
					aris::dynamic::s_vc(imp_->internal_pos_size, imp_->internal_acc_, ee_acc);
				}
				if (ee_vel) {
					aris::dynamic::s_nv(imp_->internal_pos_size, imp_->ds_, imp_->internal_vel_);
					aris::dynamic::s_vc(imp_->internal_pos_size, imp_->internal_vel_, ee_vel);
				}

				current_node = current_node->next_node_.exchange(nullptr);
				next_node = current_node->next_node_.load();
				imp_->current_node_.store(current_node);
				imp_->ds_ = 0.0;
				return current_node->id_;
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

		get_node_data(eeTypes(), current_node, s_, imp_->ds_, imp_->dds_, imp_->ddds_, imp_->internal_pos_, imp_->internal_vel_, imp_->internal_acc_);
		internal_pos_to_outpos(eeTypes(), imp_->internal_pos_, ee_pos);
		if (ee_acc) {
			aris::dynamic::s_nv(imp_->internal_pos_size, imp_->ds_ * imp_->ds_, imp_->internal_acc_);
			aris::dynamic::s_va(imp_->internal_pos_size, imp_->dds_, imp_->internal_vel_, imp_->internal_acc_);
			aris::dynamic::s_vc(imp_->internal_pos_size, imp_->internal_acc_, ee_acc);
		}
		if (ee_vel) {
			aris::dynamic::s_nv(imp_->internal_pos_size, imp_->ds_, imp_->internal_vel_);
			aris::dynamic::s_vc(imp_->internal_pos_size, imp_->internal_vel_, ee_vel);
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
		auto scurve_size = aris::dynamic::getEeTypeScurveSize(eeTypes().size(), eeTypes().data());
		std::vector<double> vel_vec(scurve_size, 1.0), acc_vec(scurve_size, 1.0), jerk_vec(scurve_size, 1.0), zone_vec(scurve_size, 0.0);
		make_node(0, &ins_node, current_node ? &*std::prev(nodes_.end(), 2) : nullptr, eeTypes(), Node::NodeType::ResetInitPos, ee_pos_internal.data(), mid_pos_internal.data()
			, vel_vec.data(), acc_vec.data(), jerk_vec.data(), zone_vec.data());

		// 设置当前 node 为 current_node_ 或 将此node设置为之前node的下一个值 //
		if (nodes_.size() < 2)
			// 只有当前node 或者 上次node已经运行结束
			imp_->current_node_.store(&ins_node);
		else
			std::prev(nodes_.end(), 2)->next_node_.store(&ins_node);
	}
	auto TrajectoryGenerator::insertLinePos(std::int64_t id, const double* ee_pos, const double* vel, const double* acc, const double* jerk, const double* zone)->void{
		std::lock_guard<std::recursive_mutex> lck(imp_->mu_);

		// 如果当前指令队列为空，那么会Z=插入ResetInitPos指令 //
		auto& nodes_ = imp_->nodes_;
		if (nodes_.empty())
			insertInitPos(id, ee_pos);

		imp_->insert_node(Node::NodeType::Line, id, ee_pos, ee_pos, vel, acc, jerk, zone);
	}
	auto TrajectoryGenerator::insertCirclePos(std::int64_t id, const double* ee_pos, const double* mid_pos, const double* vel, const double* acc, const double* jerk, const double* zone)->void {
		std::lock_guard<std::recursive_mutex> lck(imp_->mu_);

		// 如果当前指令队列为空，那么会Z=插入ResetInitPos指令 //
		auto& nodes_ = imp_->nodes_;
		if (nodes_.empty())
			insertInitPos(id, ee_pos);

		imp_->insert_node(Node::NodeType::Circle, id, ee_pos, mid_pos, vel, acc, jerk, zone);
	
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
