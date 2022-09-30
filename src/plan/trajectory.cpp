#include"aris/plan/trajectory.hpp"

namespace aris::plan {
	// 输入：
	// pq0  :上次规划zone1 结束点
	// pq1  :应当为上一个规划的终止点（也是本次规划的起点）
	// pq2  :为本次规划的终点
	// vel  :速度，是2维向量，表示线速度和角速度
	// acc  :加速度，同上
	// jerk :跃度，同上
	// zone :上一个规划的 zone2
	//
	// 输出：
	// real_zone : 上次规划经修正后的 zone2
	// pq01      : 上次规划zone2的开始点，
	// pq12      : 本次规划zone1的结束点
	auto make_zone_data_line_line(
		const double* zone, const double* pq0, const double* pq1, const double* pq2,
		double* real_zone, double* pq01, double* pq12)->void
	{
		// STEP 1：和上一段曲线拼接，计算 real zone //
		double pm1[16];
		aris::dynamic::s_pq2pm(pq1, pm1);
		double pq0_wrt_pq1[7], pq2_wrt_pq1[7];
		aris::dynamic::s_inv_pq2pq(pm1, pq0, pq0_wrt_pq1);
		aris::dynamic::s_inv_pq2pq(pm1, pq2, pq2_wrt_pq1);

		double real_zone_x = std::min(aris::dynamic::s_norm(3, pq0_wrt_pq1), zone[0]);
		real_zone_x = std::min(real_zone_x, aris::dynamic::s_norm(3, pq2_wrt_pq1));

		double real_zone_a = std::min(std::atan2(aris::dynamic::s_norm(3, pq0_wrt_pq1 + 3), pq0_wrt_pq1[6]) * 2.0, zone[1]);
		real_zone_a = std::min(real_zone_a, std::atan2(aris::dynamic::s_norm(3, pq2_wrt_pq1 + 3), pq2_wrt_pq1[6]) * 2.0);

		// STEP 2：计算 pq01, pq12/
		// 因为0和2两个坐标系都是在1下的表达，所以可以直接计算
		double pq01_wrt_pq1[7], pq12_wrt_pq1[7];
		aris::dynamic::s_vc(3, real_zone_x < 1e-10 ? 0.0 : real_zone_x / aris::dynamic::s_norm(3, pq0_wrt_pq1), pq0_wrt_pq1, pq01_wrt_pq1);
		aris::dynamic::s_vc(3, real_zone_a < 1e-9 ? 0.0 : std::sin(real_zone_a / 2) / aris::dynamic::s_norm(3, pq0_wrt_pq1 + 3), pq0_wrt_pq1 + 3, pq01_wrt_pq1 + 3);
		pq01_wrt_pq1[6] = std::cos(real_zone_a / 2);

		aris::dynamic::s_vc(3, real_zone_x < 1e-10 ? 0.0 : real_zone_x / aris::dynamic::s_norm(3, pq2_wrt_pq1), pq2_wrt_pq1, pq12_wrt_pq1);
		aris::dynamic::s_vc(3, real_zone_a < 1e-9 ? 0.0 : std::sin(real_zone_a / 2) / aris::dynamic::s_norm(3, pq2_wrt_pq1 + 3), pq2_wrt_pq1 + 3, pq12_wrt_pq1 + 3);
		pq12_wrt_pq1[6] = std::cos(real_zone_a / 2);

		aris::dynamic::s_pq2pq(pm1, pq01_wrt_pq1, pq01);
		aris::dynamic::s_pq2pq(pm1, pq12_wrt_pq1, pq12);
	}

	auto s_make_circle_data(const double* p0, const double* p1, const double* p2,
		double* center, double* axis, double& radius, double& length)
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
	auto s_compute_circle_pos_at(double length_at, const double* p0, const double* center, const double* axis, double radius, double total_length
		, double* pos_xyz)
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


	auto TrajectoryGenerator::Node::init_ee_plans(Node::MoveType move_type,	const double* ee_pos, const double* mid_pos,
		const double* vel, const double* acc, const double* jerk, const double* zone,
		const double* begin_pos,   // 为 nullptr 时，就用last_node
		const Node* last_node,
		const std::vector<aris::dynamic::EEType>& ee_types,
		std::vector<EePlanData>& ee_plans)->void
	{
		for (Size i{ 0 }, pos_idx{ 0 }, vel_idx{ 0 }; i < ee_types.size(); ++i) {
			auto& ee_p = ee_plans[i];
			ee_p.move_type_ = move_type;
			switch (ee_types[i]) {
			case aris::dynamic::EEType::PE313: [[fallthrough]];
			case aris::dynamic::EEType::PE321: [[fallthrough]];
			case aris::dynamic::EEType::PE123: [[fallthrough]];
			case aris::dynamic::EEType::PM: [[fallthrough]];
			case aris::dynamic::EEType::PQ: {
				
				double begin_pq[7];
				switch (move_type) {
				case aris::plan::TrajectoryGenerator::Node::MoveType::ResetInitPos: {
					aris::dynamic::s_vc(7, ee_pos + pos_idx, begin_pq);
					break;
				}
				case aris::plan::TrajectoryGenerator::Node::MoveType::Line: {
					auto& last_ee_p = last_node->ee_plans_[i];
					aris::dynamic::s_vc(3, last_ee_p.move_x_.line_.p1_, begin_pq);
					aris::dynamic::s_vc(4, last_ee_p.move_a_.quaternion_.q1_, begin_pq + 3);
					break;
				}
				case aris::plan::TrajectoryGenerator::Node::MoveType::Circle: {
					auto& last_ee_p = last_node->ee_plans_[i];
					s_compute_circle_pos_at(last_ee_p.move_x_.length_, last_ee_p.move_x_.circle_.p0_, last_ee_p.move_x_.circle_.center_,
						last_ee_p.move_x_.circle_.axis_, last_ee_p.move_x_.circle_.radius_, last_ee_p.move_x_.length_,
						begin_pq);
					aris::dynamic::s_vc(4, last_ee_p.move_a_.quaternion_.q1_, begin_pq + 3);
					break;
				}
				}

				// moves //
				switch (move_type) {
				case aris::plan::TrajectoryGenerator::Node::MoveType::ResetInitPos: {
					aris::dynamic::s_vc(3, begin_pq, ee_p.move_x_.line_.p0_);
					aris::dynamic::s_vc(3, ee_pos + pos_idx, ee_p.move_x_.line_.p1_);
					s_make_line_data(ee_p.move_x_.line_.p0_, ee_p.move_x_.line_.p1_, ee_p.move_x_.length_);
					break;
				}
				case aris::plan::TrajectoryGenerator::Node::MoveType::Line: {
					aris::dynamic::s_vc(3, begin_pq, ee_p.move_x_.line_.p0_);
					aris::dynamic::s_vc(3, ee_pos + pos_idx, ee_p.move_x_.line_.p1_);
					s_make_line_data(ee_p.move_x_.line_.p0_, ee_p.move_x_.line_.p1_, ee_p.move_x_.length_);
					break;
				}
				case aris::plan::TrajectoryGenerator::Node::MoveType::Circle:
					aris::dynamic::s_vc(3, begin_pq, ee_p.move_x_.circle_.p0_);
					s_make_circle_data(ee_p.move_x_.circle_.p0_, mid_pos, ee_pos + pos_idx,
						ee_p.move_x_.circle_.center_, ee_p.move_x_.circle_.axis_, ee_p.move_x_.circle_.radius_, ee_p.move_x_.length_);
					break;
				default:
					break;
				}
				aris::dynamic::s_vc(4, begin_pq + 3, ee_p.move_a_.quaternion_.q0_);
				aris::dynamic::s_vc(4, ee_pos + pos_idx + 3, ee_p.move_a_.quaternion_.q1_);
				s_make_quaternion_data(ee_p.move_a_.quaternion_.q0_, ee_p.move_a_.quaternion_.q1_, ee_p.move_a_.length_);

				// zones //
				ee_p.zone_x1_.zone_value_ = 0.0;
				ee_p.zone_x1_.length_ = 0.0;
				ee_p.zone_a1_.zone_value_ = 0.0;
				ee_p.zone_a1_.length_ = 0.0;
				ee_p.zone_x2_.zone_value_ = zone[0];
				ee_p.zone_x2_.length_ = 0.0;
				ee_p.zone_a2_.zone_value_ = zone[1];
				ee_p.zone_a2_.length_ = 0.0;

				aris::dynamic::s_vc(4, begin_pq, ee_p.zone_x1_.lines_.p0_);
				aris::dynamic::s_vc(4, begin_pq, ee_p.zone_x1_.lines_.p1_);
				aris::dynamic::s_vc(4, ee_pos + pos_idx, ee_p.zone_x2_.lines_.p0_);
				aris::dynamic::s_vc(4, ee_pos + pos_idx, ee_p.zone_x2_.lines_.p1_);
				aris::dynamic::s_vc(4, begin_pq + 3, ee_p.zone_a1_.quaternions_.q0_);
				aris::dynamic::s_vc(4, begin_pq + 3, ee_p.zone_a1_.quaternions_.q1_);
				aris::dynamic::s_vc(4, ee_pos + pos_idx + 3, ee_p.zone_a2_.quaternions_.q0_);
				aris::dynamic::s_vc(4, ee_pos + pos_idx + 3, ee_p.zone_a2_.quaternions_.q1_);

				// scurves //
				double p = ee_p.zone_x1_.length_ / 2.0 + ee_p.zone_x2_.length_ / 2.0 + ee_p.move_x_.length_;
				ee_p.scurve_x_.pa_ = 0.0;
				ee_p.scurve_x_.pa_count_ = 0;
				ee_p.scurve_x_.pb_ = std::fmod(p, 1000.0);
				ee_p.scurve_x_.pb_count_ = std::lround((p - std::fmod(p, 1000.0)) / 1000.0);
				ee_p.scurve_x_.va_ = 0.0;
				ee_p.scurve_x_.vc_max_ = vel[vel_idx];
				ee_p.scurve_x_.vb_max_ = 0.0;
				ee_p.scurve_x_.a_ = acc[vel_idx];
				ee_p.scurve_x_.j_ = jerk[vel_idx];
				ee_p.scurve_x_.t0_ = 0.0;
				ee_p.scurve_x_.t0_count_ = 0;

				p = ee_p.zone_a1_.length_ / 2.0 + ee_p.zone_a2_.length_ / 2.0 + ee_p.move_a_.length_;
				ee_p.scurve_a_.pa_ = 0.0;
				ee_p.scurve_a_.pa_count_ = 0;
				ee_p.scurve_a_.pb_ = std::fmod(p, 1000.0);
				ee_p.scurve_a_.pb_count_ = std::lround((p - std::fmod(p, 1000.0)) / 1000.0);
				ee_p.scurve_a_.va_ = 0.0;
				ee_p.scurve_a_.vc_max_ = vel[vel_idx + 1];
				ee_p.scurve_a_.vb_max_ = 0.0;
				ee_p.scurve_a_.a_ = acc[vel_idx + 1];
				ee_p.scurve_a_.j_ = jerk[vel_idx + 1];
				ee_p.scurve_a_.t0_ = 0.0;
				ee_p.scurve_a_.t0_count_ = 0;

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
			case aris::dynamic::EEType::X: {
				pos_idx += 1;
				vel_idx += 1;
				break;
			}
			case aris::dynamic::EEType::A: {
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

	auto TrajectoryGenerator::Node::make_ee_plan_path(Node* last_node, Node* node)->void {
		// 更新本段轨迹的 move //
		for (int i = 0; i < node->ee_plans_.size(); ++i) {
			auto this_p = &node->ee_plans_[i];
			auto last_p = &last_node->ee_plans_[i];

			switch (this_p->ee_type_) {
			case aris::dynamic::EEType::PE313: [[fallthrough]];
			case aris::dynamic::EEType::PE321: [[fallthrough]];
			case aris::dynamic::EEType::PE123: [[fallthrough]];
			case aris::dynamic::EEType::PM: [[fallthrough]];
			case aris::dynamic::EEType::PQ: {
				switch (this_p->move_type_) {
				case aris::plan::TrajectoryGenerator::Node::MoveType::Line: {
					switch (last_p->move_type_) {
					case aris::plan::TrajectoryGenerator::Node::MoveType::Line: {
						// 将四元数转向与上一次设置一致
						if (aris::dynamic::s_vv(4, last_p->move_a_.quaternion_.q1_, this_p->move_a_.quaternion_.q1_) < 0.0) {
							aris::dynamic::s_nv(4, -1.0, this_p->move_a_.quaternion_.q1_);
							aris::dynamic::s_nv(4, -1.0, this_p->zone_a2_.quaternions_.q0_);
							aris::dynamic::s_nv(4, -1.0, this_p->zone_a2_.quaternions_.q1_);
						}
							

						// 和上一段进行路径拼接
						make_zone_x_line_line(last_p, this_p);
						make_zone_a_line_line(last_p, this_p);

						// 更新 scurve //
						double p = last_p->scurve_x_.pa_ + last_p->zone_x1_.length_ / 2.0 + last_p->zone_x2_.length_ / 2.0 + last_p->move_x_.length_;
						last_p->scurve_x_.pb_ = std::fmod(p, 1000.0);
						last_p->scurve_x_.pb_count_ = last_p->scurve_x_.pa_count_ + std::lround((p - std::fmod(p, 1000.0)) / 1000.0);

						p = last_p->scurve_a_.pa_ + last_p->zone_a1_.length_ / 2.0 + last_p->zone_a2_.length_ / 2.0 + last_p->move_a_.length_;
						last_p->scurve_a_.pb_ = std::fmod(p, 1000.0);
						last_p->scurve_a_.pb_count_ = last_p->scurve_a_.pa_count_ + std::lround((p - std::fmod(p, 1000.0)) / 1000.0);

						this_p->scurve_x_.pa_ = last_p->scurve_x_.pb_;
						this_p->scurve_x_.pa_count_ = last_p->scurve_x_.pb_count_;

						this_p->scurve_a_.pa_ = last_p->scurve_a_.pb_;
						this_p->scurve_a_.pa_count_ = last_p->scurve_a_.pb_count_;

						p = this_p->scurve_x_.pa_ + this_p->zone_x1_.length_ / 2.0 + this_p->zone_x2_.length_ / 2.0 + this_p->move_x_.length_;
						this_p->scurve_x_.pb_ = std::fmod(p, 1000.0);
						this_p->scurve_x_.pb_count_ = last_p->scurve_x_.pb_count_ + std::lround((p - std::fmod(p, 1000.0)) / 1000.0);

						p = this_p->scurve_a_.pa_ + this_p->zone_a1_.length_ / 2.0 + this_p->zone_a2_.length_ / 2.0 + this_p->move_a_.length_;
						this_p->scurve_a_.pb_ = std::fmod(p, 1000.0);
						this_p->scurve_a_.pb_count_ = last_p->scurve_a_.pb_count_ + std::lround((p - std::fmod(p, 1000.0)) / 1000.0);

						// 更新上次的截止速度
						double p50[4], dp50[4], d2p50[4];
						double vb;
						s_bezier3_blend_line_line(0.5, last_p->zone_x2_.lines_.p0_, last_p->zone_x2_.lines_.p1_, last_p->zone_x2_.lines_.p2_, 
							p50, dp50, d2p50);
						s_bezier3_max_v_at(3, dp50, d2p50, std::min(last_p->scurve_x_.a_, this_p->scurve_x_.a_), vb);
						last_p->scurve_x_.vb_max_ = std::min({vb, last_p->scurve_x_.vc_max_, this_p->scurve_x_.vc_max_ });

						
						s_bezier3_blend_quaternion(0.5, last_p->zone_a2_.quaternions_.q0_, last_p->zone_a2_.quaternions_.q1_, last_p->zone_a2_.quaternions_.q2_,
							p50, dp50, d2p50);
						s_bezier3_max_v_at(4, dp50, d2p50, std::min(last_p->scurve_a_.a_, this_p->scurve_a_.a_), vb);
						last_p->scurve_a_.vb_max_ = std::min({ vb * 2.0, last_p->scurve_a_.vc_max_, this_p->scurve_a_.vc_max_ });
						break;
					}
					case aris::plan::TrajectoryGenerator::Node::MoveType::Circle:
						break;
					default:
						break;
					}
					break;
				}
				case aris::plan::TrajectoryGenerator::Node::MoveType::Circle:
					break;
				default:
					break;
				}


				break;
			}
			default:
				break;
			}
		}






	}

	auto TrajectoryGenerator::Node::make_zone_x_line_line(EePlanData* last_p, EePlanData* this_p)->void {
		// STEP 0. 设置 this_p 的 move_x 的起始位置
		aris::dynamic::s_vc(3, last_p->move_x_.line_.p1_, this_p->move_x_.line_.p0_);
		s_make_line_data(this_p->move_x_.line_.p0_, this_p->move_x_.line_.p1_, this_p->move_x_.length_);

		// STEP 1. 计算真实的交融半径
		double real_zone = std::min({ last_p->move_x_.length_, last_p->zone_x2_.zone_value_, this_p->move_x_.length_ });

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
	}
	auto TrajectoryGenerator::Node::make_zone_a_line_line(EePlanData* last_p, EePlanData* this_p)->void {
		// STEP 0. 设置 this_p 的 move_x 的起始位置
		aris::dynamic::s_vc(4, last_p->move_a_.quaternion_.q1_, this_p->move_a_.quaternion_.q0_);
		s_make_quaternion_data(this_p->move_a_.quaternion_.q0_, this_p->move_a_.quaternion_.q1_, this_p->move_a_.length_);

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
		s_bezier3_estimate_arc_param(2.0*darc_ds_0, 2.0*d2arc_ds2_0, 2.0*darc_ds_1, 2.0*d2arc_ds2_1, 2.0*darc_ds_50
			, zone_a2.a_, zone_a2.b_, zone_a2.c_, zone_a2.d_, zone_a2.e_);

		s_bezier3_s2arc(1.0, zone_a2.a_, zone_a2.b_, zone_a2.c_, zone_a2.d_, zone_a2.e_, arc, darc, d2arc);
		zone_a2.length_ = arc; 

		this_p->zone_a1_ = zone_a2;
	}

	auto TrajectoryGenerator::Node::EePlanData::getInternalEndPos(Node::MoveType move_type, aris::dynamic::EEType ee_type, double* end_pos)const noexcept->void {
		switch (ee_type) {
		case aris::dynamic::EEType::PE313: [[fallthrough]];
		case aris::dynamic::EEType::PE321: [[fallthrough]];
		case aris::dynamic::EEType::PE123: [[fallthrough]];
		case aris::dynamic::EEType::PM: [[fallthrough]];
		case aris::dynamic::EEType::PQ: {
			switch (move_type) {
			case aris::plan::TrajectoryGenerator::Node::MoveType::Line:
				aris::dynamic::s_vc(3, move_x_.line_.p1_, end_pos);
				break;
			case aris::plan::TrajectoryGenerator::Node::MoveType::Circle:
				s_compute_circle_pos_at(move_x_.length_, move_x_.circle_.p0_, move_x_.circle_.center_,
					move_x_.circle_.axis_, move_x_.circle_.radius_, move_x_.length_,
					end_pos);
				break;
			}
			aris::dynamic::s_vc(4, move_a_.quaternion_.q1_, end_pos + 3);
			return;

		}

		case aris::dynamic::EEType::XYZT: {
			return;
		}
		case aris::dynamic::EEType::XYZ: {
			return;
		}
		case aris::dynamic::EEType::XYT: {
			return;
		}
		case aris::dynamic::EEType::XY: {
			return;
		}
		case aris::dynamic::EEType::X: {
			return;
		}
		case aris::dynamic::EEType::A: {
			return;
		}
		case aris::dynamic::EEType::UNKNOWN:
			return;
		default:
			return;
		}

	}

	auto TrajectoryGenerator::getEePosAndMoveDt(double* ee_pos)->std::int64_t {
		auto current_node = current_node_.load();

		if (current_node == nullptr) {
			return 0;
		}

		s_ += ds_ * dt_;
		if (s_ > 1000.0) {
			s_ -= 1000.0;
			s_count_++;
		}

		bool need_to_switch = false;

		int idx = 0;
		for (auto& ee_p : current_node->ee_plans_) {
			switch (ee_p.move_type_){
			case Node::MoveType::ResetInitPos: {
				switch (ee_p.ee_type_) {
				case aris::dynamic::EEType::PE313: [[fallthrough]];
				case aris::dynamic::EEType::PE321: [[fallthrough]];
				case aris::dynamic::EEType::PE123: [[fallthrough]];
				case aris::dynamic::EEType::PM: [[fallthrough]];
				case aris::dynamic::EEType::PQ: {
					need_to_switch = true;
					aris::dynamic::s_vc(3, ee_p.move_x_.line_.p1_, internal_pos.data() + idx);
					idx += 3;
					aris::dynamic::s_vc(4, ee_p.move_a_.quaternion_.q1_, internal_pos.data() + idx);
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
				case aris::dynamic::EEType::X: {
					break;
				}
				case aris::dynamic::EEType::A: {
					break;
				}
				case aris::dynamic::EEType::UNKNOWN:
				default:
					;
				}
				break;
			}
			case Node::MoveType::Line: {
				switch (ee_p.ee_type_) {
				case aris::dynamic::EEType::PE313: [[fallthrough]];
				case aris::dynamic::EEType::PE321: [[fallthrough]];
				case aris::dynamic::EEType::PE123: [[fallthrough]];
				case aris::dynamic::EEType::PM: [[fallthrough]];
				case aris::dynamic::EEType::PQ: {
					if ((ee_p.scurve_x_.t0_count_ - s_count_) * 1000.0  + ee_p.scurve_x_.t0_ + ee_p.scurve_x_.T_ - s_ < 0.0) {
						need_to_switch = true;
					}
					
					// x //
					{
						int sp_count;
						double sp, sv, sa, sj;
						s_scurve_at(ee_p.scurve_x_, s_count_, s_, &sp_count, &sp, &sv, &sa, &sj);

						double l = (sp_count - ee_p.scurve_x_.pa_count_) * 1000.0 + sp - ee_p.scurve_x_.pa_;

						if (l < ee_p.zone_x1_.length_ / 2.0) {
							double s;
							auto& z = ee_p.zone_x1_;
							s_bezier3_arc2s(l + ee_p.zone_x1_.length_ / 2.0, z.a_, z.b_, z.c_, z.d_, z.e_, s);
							double p[3], dp[3], d2p[3];
							s_bezier3_blend_line_line(s, z.lines_.p0_, z.lines_.p1_, z.lines_.p2_, p, dp, d2p);
							aris::dynamic::s_vc(3, p, internal_pos.data() + idx);
						}
						else if (l < ee_p.zone_x1_.length_ / 2.0 + ee_p.move_x_.length_) {
							auto& line = ee_p.move_x_.line_;
							double p[3];
							s_compute_line_pos_at(l - ee_p.zone_x1_.length_ / 2.0, line.p0_, line.p1_, ee_p.move_x_.length_, p);
							aris::dynamic::s_vc(3, p, internal_pos.data() + idx);
						}
						else  {
							double s;
							auto& z = ee_p.zone_x2_;
							s_bezier3_arc2s(l - ee_p.zone_x1_.length_ / 2.0 - ee_p.move_x_.length_, z.a_, z.b_, z.c_, z.d_, z.e_, s);
							double p[3], dp[3], d2p[3];
							s_bezier3_blend_line_line(s, z.lines_.p0_, z.lines_.p1_, z.lines_.p2_, p, dp, d2p);
							aris::dynamic::s_vc(3, p, internal_pos.data() + idx);
						}

						idx += 3;
					}

					// a //
					{
						int sp_count;
						double sp, sv, sa, sj;
						s_scurve_at(ee_p.scurve_a_, s_count_, s_, &sp_count, &sp, &sv, &sa, &sj);

						double l = (sp_count - ee_p.scurve_a_.pa_count_) * 1000.0 + sp - ee_p.scurve_a_.pa_;

						if (l < ee_p.zone_a1_.length_ / 2.0) {
							double s;
							auto& z = ee_p.zone_a1_;
							s_bezier3_arc2s(l + ee_p.zone_a1_.length_ / 2.0, z.a_, z.b_, z.c_, z.d_, z.e_, s);
							double p[4], dp[4], d2p[4];
							s_bezier3_blend_quaternion(s, z.quaternions_.q0_, z.quaternions_.q1_, z.quaternions_.q2_, p, dp, d2p);
							aris::dynamic::s_vc(4, p, internal_pos.data() + idx);

							////////////////

							//if (s_ > 4.383) {
							//	s_bezier3_arc2s(ee_p.zone_a1_.length_, z.a_, z.b_, z.c_, z.d_, z.e_, s);
							//	s_bezier3_blend_quaternion(s, z.quaternions_.q0_, z.quaternions_.q1_, z.quaternions_.q2_, p, dp, d2p);
							//	//aris::dynamic::dsp(1, 4, z.quaternions_.q0_);
							//	//aris::dynamic::dsp(1, 4, z.quaternions_.q1_);
							//	//aris::dynamic::dsp(1, 4, z.quaternions_.q2_);
							//	aris::dynamic::dsp(1, 4, p);
							//}

							/////////////////
						}
						else if (l < ee_p.zone_a1_.length_ / 2.0 + ee_p.move_a_.length_) {
							auto& quternion = ee_p.move_a_.quaternion_;
							double p[4];
							s_compute_quaternion_at(l - ee_p.zone_a1_.length_ / 2.0, quternion.q0_, quternion.q1_, ee_p.move_a_.length_, p);
							aris::dynamic::s_vc(4, p, internal_pos.data() + idx);
						}
						else {
							double s;
							auto& z = ee_p.zone_a2_;
							s_bezier3_arc2s(l - ee_p.zone_a1_.length_ / 2.0 - ee_p.move_a_.length_, z.a_, z.b_, z.c_, z.d_, z.e_, s);
							double p[4], dp[4], d2p[4];
							s_bezier3_blend_quaternion(s, z.quaternions_.q0_, z.quaternions_.q1_, z.quaternions_.q2_, p, dp, d2p);
							aris::dynamic::s_vc(4, p, internal_pos.data() + idx);
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
				case aris::dynamic::EEType::X: {
					break;
				}
				case aris::dynamic::EEType::A: {
					break;
				}
				case aris::dynamic::EEType::UNKNOWN:
				default:
					;
				}
			}

			}


			

			
		}

		if (need_to_switch) {
			auto next = current_node->next_node_.exchange(nullptr);
			current_node_.store(next);
		}

		this->internal_pos_to_outpos(ee_types_, internal_pos.data(), ee_pos);

		return current_node->id_;
	}

	auto TrajectoryGenerator::insertPos(std::int64_t id, Node::MoveType move_type, const double* ee_pos, const double* mid_pos,
		const double* vel, const double* acc, const double* jerk, const double* zone)->void
	{
		std::lock_guard<std::recursive_mutex> lck(mu_);

		// 转化 pos 表达 //
		std::vector<double> ee_pos_internal(internal_pos_size), mid_pos_internal(internal_pos_size);
		outpos_to_internal_pos(ee_types_, ee_pos, ee_pos_internal.data());
		outpos_to_internal_pos(ee_types_, mid_pos, mid_pos_internal.data());

		// 如果当前指令队列为空，那么会插入ResetInitPos指令 //
		if (nodes_.empty()) {
			auto& ins_node = nodes_.emplace_back(ee_types_.size());

			ins_node.id_ = id;

			// 计算scurve的尺寸
			auto scurve_size = aris::dynamic::getScurveSize(ee_types_);

			// 生成起始的数据
			std::vector<double> vel_vec(scurve_size, 1.0), acc_vec(scurve_size, 1.0), jerk_vec(scurve_size, 1.0), zone_vec(scurve_size, 0.0);

			Node::init_ee_plans(Node::MoveType::ResetInitPos, ee_pos_internal.data(), mid_pos_internal.data(), 
				vel_vec.data(), acc_vec.data(), jerk_vec.data(), zone_vec.data()
				, nullptr, nullptr, ee_types_, ins_node.ee_plans_);

			// 插入scurve的数据 //
			std::list<SCurveNode> ins_list;
			ins_list.push_back(SCurveNode{});
			auto& scurve_node = ins_list.back();

			// 规划scurve //
			scurve_node.params_.reserve(scurve_size);
			for (auto& ee_p : ins_node.ee_plans_) {
				switch (ee_p.ee_type_) {
				case aris::dynamic::EEType::PE313: [[fallthrough]];
				case aris::dynamic::EEType::PE321: [[fallthrough]];
				case aris::dynamic::EEType::PE123: [[fallthrough]];
				case aris::dynamic::EEType::PM: [[fallthrough]];
				case aris::dynamic::EEType::PQ: {
					scurve_node.params_.push_back(ee_p.scurve_x_);
					scurve_node.params_.push_back(ee_p.scurve_a_);
					break;
				}
				case aris::dynamic::EEType::XYZT: {
					scurve_node.params_.push_back(ee_p.scurve_x_);
					scurve_node.params_.push_back(ee_p.scurve_a_);
					break;
				}
				case aris::dynamic::EEType::XYZ: {
					scurve_node.params_.push_back(ee_p.scurve_x_);
					break;
				}
				case aris::dynamic::EEType::XYT: {
					scurve_node.params_.push_back(ee_p.scurve_x_);
					scurve_node.params_.push_back(ee_p.scurve_a_);
					break;
				}
				case aris::dynamic::EEType::XY: {
					scurve_node.params_.push_back(ee_p.scurve_x_);
					break;
				}
				case aris::dynamic::EEType::X: {
					scurve_node.params_.push_back(ee_p.scurve_x_);
					break;
				}
				case aris::dynamic::EEType::A: {
					scurve_node.params_.push_back(ee_p.scurve_a_);
					break;
				}
				case aris::dynamic::EEType::UNKNOWN:
				default:
					;
				}
			}
			s_compute_scurve(ins_list.begin(), ins_list.end());

			// 重新插入规划后的 scurve //
			for (int i = 0, s_idx = 0; i < ins_node.ee_plans_.size(); ++i) {
				auto& ee_p = ins_node.ee_plans_[i];
				switch (ee_p.ee_type_) {
				case aris::dynamic::EEType::PE313: [[fallthrough]];
				case aris::dynamic::EEType::PE321: [[fallthrough]];
				case aris::dynamic::EEType::PE123: [[fallthrough]];
				case aris::dynamic::EEType::PM: [[fallthrough]];
				case aris::dynamic::EEType::PQ: {
					ee_p.scurve_x_ = ins_list.back().params_[s_idx];
					s_idx++;
					ee_p.scurve_a_ = ins_list.back().params_[s_idx];
					s_idx++;
					break;
				}
				case aris::dynamic::EEType::XYZT: {
					ee_p.scurve_x_ = ins_list.back().params_[s_idx];
					s_idx++;
					ee_p.scurve_a_ = ins_list.back().params_[s_idx];
					s_idx++;
					break;
				}
				case aris::dynamic::EEType::XYZ: {
					ee_p.scurve_x_ = ins_list.back().params_[s_idx];
					s_idx++;
					break;
				}
				case aris::dynamic::EEType::XYT: {
					ee_p.scurve_x_ = ins_list.back().params_[s_idx];
					s_idx++;
					ee_p.scurve_a_ = ins_list.back().params_[s_idx];
					s_idx++;
					break;
				}
				case aris::dynamic::EEType::XY: {
					ee_p.scurve_x_ = ins_list.back().params_[s_idx];
					s_idx++;
					break;
				}
				case aris::dynamic::EEType::X: {
					ee_p.scurve_x_ = ins_list.back().params_[s_idx];
					s_idx++;
					break;
				}
				case aris::dynamic::EEType::A: {
					ee_p.scurve_a_ = ins_list.back().params_[s_idx];
					s_idx++;
					break;
				}
				case aris::dynamic::EEType::UNKNOWN:
				default:
					;
				}
			}

			// 设置当前 current_node_ //
			current_node_.store(&*nodes_.begin());

			if (move_type == Node::MoveType::ResetInitPos)
				return;
		}

		// 插入最新节点 //
		auto& last_node = *std::prev(nodes_.end());
		auto& ins_node = nodes_.emplace_back(ee_types_.size());
		ins_node.id_ = id;
		Node::init_ee_plans(move_type, ee_pos_internal.data(), mid_pos_internal.data(), vel, acc, jerk, zone
			, nullptr, &last_node, ee_types_, ins_node.ee_plans_);
		Node::make_ee_plan_path(&last_node, &ins_node);
		auto ins_iter = std::prev(nodes_.end());

		// 获得需要重新规划的起点，默认为5段轨迹
		auto current_node = current_node_.load();
		auto current_iter = std::find_if(nodes_.begin(), nodes_.end(), [current_node](auto& node)->bool {
			return &node == current_node;
			});
		
		auto replan_iter_start = std::next(current_iter);
		auto replan_iter_end = std::next(current_iter);
		aris::Size replan_num = 0;

		for (; std::next(replan_iter_end) != nodes_.end(); replan_iter_end++) {
			if (replan_num > 4) {
				replan_iter_start++;
			}
			else {
				replan_num++;
			}
		}
		
		replan_iter_end = nodes_.insert(std::prev(nodes_.end()), replan_iter_start, replan_iter_end);

		// 重规划，计算scurve
		auto scurve_size = aris::dynamic::getScurveSize(ee_types_);

		// 插入最新插入的节点
		std::list<SCurveNode> ins_scurve_list;
		ins_scurve_list.push_front(SCurveNode{});
		ins_scurve_list.front().params_.reserve(scurve_size);
		for (auto& ee_p : ins_node.ee_plans_) {
			auto& scurve_node = ins_scurve_list.front();

			switch (ee_p.ee_type_) {
			case aris::dynamic::EEType::PE313: [[fallthrough]];
			case aris::dynamic::EEType::PE321: [[fallthrough]];
			case aris::dynamic::EEType::PE123: [[fallthrough]];
			case aris::dynamic::EEType::PM: [[fallthrough]];
			case aris::dynamic::EEType::PQ: {
				scurve_node.params_.push_back(ee_p.scurve_x_);
				scurve_node.params_.push_back(ee_p.scurve_a_);
				break;
			}
			case aris::dynamic::EEType::XYZT: {
				scurve_node.params_.push_back(ee_p.scurve_x_);
				scurve_node.params_.push_back(ee_p.scurve_a_);
				break;
			}
			case aris::dynamic::EEType::XYZ: {
				scurve_node.params_.push_back(ee_p.scurve_x_);
				break;
			}
			case aris::dynamic::EEType::XYT: {
				scurve_node.params_.push_back(ee_p.scurve_x_);
				scurve_node.params_.push_back(ee_p.scurve_a_);
				break;
			}
			case aris::dynamic::EEType::XY: {
				scurve_node.params_.push_back(ee_p.scurve_x_);
				break;
			}
			case aris::dynamic::EEType::X: {
				scurve_node.params_.push_back(ee_p.scurve_x_);
				break;
			}
			case aris::dynamic::EEType::A: {
				scurve_node.params_.push_back(ee_p.scurve_a_);
				break;
			}
			case aris::dynamic::EEType::UNKNOWN:
			default:
				;
			}
		}

		// 插入其他节点 //
		auto ins_iter_start = std::prev(nodes_.end());
		for (int i = 0; i < replan_num; ++i) {
			ins_scurve_list.push_front(SCurveNode{});
			auto& scurve_node = ins_scurve_list.front();
			scurve_node.params_.reserve(scurve_size);
			for (auto& ee_p : std::prev(ins_iter_start)->ee_plans_) {
				switch (ee_p.ee_type_) {
				case aris::dynamic::EEType::PE313: [[fallthrough]];
				case aris::dynamic::EEType::PE321: [[fallthrough]];
				case aris::dynamic::EEType::PE123: [[fallthrough]];
				case aris::dynamic::EEType::PM: [[fallthrough]];
				case aris::dynamic::EEType::PQ: {
					scurve_node.params_.push_back(ee_p.scurve_x_);
					scurve_node.params_.push_back(ee_p.scurve_a_);
					break;
				}
				case aris::dynamic::EEType::XYZT: {
					scurve_node.params_.push_back(ee_p.scurve_x_);
					scurve_node.params_.push_back(ee_p.scurve_a_);
					break;
				}
				case aris::dynamic::EEType::XYZ: {
					scurve_node.params_.push_back(ee_p.scurve_x_);
					break;
				}
				case aris::dynamic::EEType::XYT: {
					scurve_node.params_.push_back(ee_p.scurve_x_);
					scurve_node.params_.push_back(ee_p.scurve_a_);
					break;
				}
				case aris::dynamic::EEType::XY: {
					scurve_node.params_.push_back(ee_p.scurve_x_);
					break;
				}
				case aris::dynamic::EEType::X: {
					scurve_node.params_.push_back(ee_p.scurve_x_);
					break;
				}
				case aris::dynamic::EEType::A: {
					scurve_node.params_.push_back(ee_p.scurve_a_);
					break;
				}
				case aris::dynamic::EEType::UNKNOWN:
				default:
					;
				}
			}
			ins_iter_start = std::prev(ins_iter_start);
			ins_iter_start->next_node_.store(&*std::next(ins_iter_start));
		}

		// 规划scurve //
		s_compute_scurve(ins_scurve_list.begin(), ins_scurve_list.end());

		// 重新插入规划后的 scurve //
		for (auto iter = ins_iter_start; iter != nodes_.end(); ++iter) {
			auto& scurve_node = ins_scurve_list.front();
			for (int i = 0, s_idx = 0; i < iter->ee_plans_.size(); ++i) {
				auto& ee_p = iter->ee_plans_[i];
				switch (ee_p.ee_type_) {
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
				case aris::dynamic::EEType::X: {
					ee_p.scurve_x_ = scurve_node.params_[s_idx];
					s_idx++;
					break;
				}
				case aris::dynamic::EEType::A: {
					ee_p.scurve_a_ = scurve_node.params_[s_idx];
					s_idx++;
					break;
				}
				case aris::dynamic::EEType::UNKNOWN:
				default:
					;
				}
			}
			ins_scurve_list.pop_front();
		}

		// return value
		bool ret = std::prev(replan_iter_start)->next_node_.exchange(&*ins_iter_start) != nullptr || replan_num == 0;

		// 如果成功，则删除需要重新规划的节点，否则删除新插入的节点
		if (ret) {
			nodes_.erase(replan_iter_start, replan_iter_end);
		}
		else {
			nodes_.erase(ins_iter_start, nodes_.end());
		}
	}
}
