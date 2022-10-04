#include"aris/plan/trajectory.hpp"

namespace aris::plan {
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
				continue;
			}
			case aris::dynamic::EEType::PE321: {
				aris::dynamic::s_pe2pq(out_pos + out_idx, internal_pos + internal_idx, "321");
				internal_idx += 7;
				out_idx += 6;
				continue;
			}
			case aris::dynamic::EEType::PE123: {
				aris::dynamic::s_pe2pq(out_pos + out_idx, internal_pos + internal_idx, "123");
				internal_idx += 7;
				out_idx += 6;
				continue;
			}
			case aris::dynamic::EEType::PQ: {
				aris::dynamic::s_vc(7, out_pos + out_idx, internal_pos + internal_idx);
				internal_idx += 7;
				out_idx += 7;
				continue;
			}
			case aris::dynamic::EEType::PM: {
				aris::dynamic::s_pm2pq(out_pos + out_idx, internal_pos + internal_idx);
				internal_idx += 7;
				out_idx += 16;
				continue;
			}
			case aris::dynamic::EEType::XYZT: {

			}
			case aris::dynamic::EEType::XYZ: {

			}
			case aris::dynamic::EEType::XYT: {

			}
			case aris::dynamic::EEType::XY: {

			}
			case aris::dynamic::EEType::X: {

			}
			case aris::dynamic::EEType::A: {

			}
			case aris::dynamic::EEType::UNKNOWN:
			default:
				;
			}
		}


	}

	// make & get data // 
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


	auto init_node(TrajectoryGenerator::Node* this_node, const TrajectoryGenerator::Node* last_node, const std::vector<aris::dynamic::EEType>& ee_types,
		TrajectoryGenerator::Node::MoveType move_type, const double* ee_pos, const double* mid_pos, const double* vel, const double* acc, const double* jerk, const double* zone)->void
	{
		auto& ee_plans = this_node->ee_plans_;
		this_node->next_node_.store(this_node);
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
					this_node->s_end_ = 0.0;
					this_node->s_end_count_ = 0;
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
				ee_p.scurve_x_.pb_ = p;
				//ee_p.scurve_x_.pa_ = 0.0;
				//ee_p.scurve_x_.pa_count_ = 0;
				//ee_p.scurve_x_.pb_ = std::fmod(p, 1000.0);
				//ee_p.scurve_x_.pb_count_ = std::lround((p - std::fmod(p, 1000.0)) / 1000.0);
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

	auto replan_scurve(int scurve_size, std::list<TrajectoryGenerator::Node>::iterator begin, std::list<TrajectoryGenerator::Node>::iterator end) {
		std::list<SCurveNode> ins_scurve_list;
		for (auto iter = begin; iter != end; ++iter) {
			ins_scurve_list.push_back(SCurveNode{});
			auto& scurve_node = ins_scurve_list.back();
			scurve_node.params_.reserve(scurve_size);
			for (auto& ee_p : iter->ee_plans_) {
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
		}

		s_compute_scurve(ins_scurve_list.begin(), ins_scurve_list.end());

		for (auto iter = begin; iter != end; ++iter) {
			auto& scurve_node = ins_scurve_list.front();
			iter->s_end_ = scurve_node.params_[0].t0_ + scurve_node.params_[0].T_;
			iter->s_end_count_ = scurve_node.params_[0].t0_count_;
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
			iter->next_node_.store(std::next(iter) == end ? &*iter : &*std::next(iter));


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


	auto make_zone_line_line_x(TrajectoryGenerator::Node::EePlanData* last_p, TrajectoryGenerator::Node::EePlanData* this_p) ->void {
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
	auto make_zone_line_line_a(TrajectoryGenerator::Node::EePlanData* last_p, TrajectoryGenerator::Node::EePlanData* this_p)->void {
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
		s_bezier3_estimate_arc_param(2.0 * darc_ds_0, 2.0 * d2arc_ds2_0, 2.0 * darc_ds_1, 2.0 * d2arc_ds2_1, 2.0 * darc_ds_50
			, zone_a2.a_, zone_a2.b_, zone_a2.c_, zone_a2.d_, zone_a2.e_);

		s_bezier3_s2arc(1.0, zone_a2.a_, zone_a2.b_, zone_a2.c_, zone_a2.d_, zone_a2.e_, arc, darc, d2arc);
		zone_a2.length_ = arc;

		this_p->zone_a1_ = zone_a2;
	}


	struct TrajectoryGenerator::Imp {


	};
	TrajectoryGenerator::~TrajectoryGenerator() = default;
	TrajectoryGenerator::TrajectoryGenerator() = default;
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
						make_zone_line_line_x(last_p, this_p);
						make_zone_line_line_a(last_p, this_p);

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
	auto TrajectoryGenerator::getEePosAndMoveDt(double* ee_pos)->std::int64_t {
		auto current_node = current_node_.load();
		auto next_node = current_node->next_node_.load();

		s_ += ds_ * dt_;
		if (s_ > 1000.0) {
			s_ -= 1000.0;
			s_count_++;
		}
		
		bool is_end = current_node == next_node && (current_node->s_end_count_ - s_count_) * 1000.0 + current_node->s_end_ - s_ < 0.0;
		if (is_end) {
			s_count_ = current_node->s_end_count_;
			s_ = current_node->s_end_;
		}

		int idx = 0;
		for (auto& ee_p : current_node->ee_plans_) {
			switch (ee_p.move_type_){
			case Node::MoveType::ResetInitPos: {
				s_ = 0.0;
				s_count_ = 0.0;
				ds_ = target_ds_;

				switch (ee_p.ee_type_) {
				case aris::dynamic::EEType::PE313: [[fallthrough]];
				case aris::dynamic::EEType::PE321: [[fallthrough]];
				case aris::dynamic::EEType::PE123: [[fallthrough]];
				case aris::dynamic::EEType::PM: [[fallthrough]];
				case aris::dynamic::EEType::PQ: {
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

		internal_pos_to_outpos(ee_types_, internal_pos.data(), ee_pos);

		// check if is end //
		if (is_end) {
			return 0;
		}
		
		// check if need to switch //
		if (current_node != next_node && (current_node->s_end_count_ - s_count_) * 1000.0 + current_node->s_end_ - s_ - ds_*dt_ < 0.0) {
			auto next = current_node->next_node_.exchange(nullptr);
			current_node_.store(next_node);
		}

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

		// 如果当前指令队列为空，那么会Z=插入ResetInitPos指令 //
		if (nodes_.empty()) {
			auto& ins_node = nodes_.emplace_back(ee_types_.size());
			ins_node.id_ = id;

			// 初始化节点 //
			auto scurve_size = aris::dynamic::getScurveSize(ee_types_);
			std::vector<double> vel_vec(scurve_size, 1.0), acc_vec(scurve_size, 1.0), jerk_vec(scurve_size, 1.0), zone_vec(scurve_size, 0.0);
			init_node(&ins_node, nullptr, ee_types_, Node::MoveType::ResetInitPos, ee_pos_internal.data(), mid_pos_internal.data(), vel, acc, jerk, zone);

			// 设置当前 current_node_ //
			current_node_.store(&*nodes_.begin());

			// 如果本身插入的就是 resetInitPos, 那么返回，否则会继续插入指令
			if (move_type == Node::MoveType::ResetInitPos)
				return;
		}

		// 插入最新节点 //
		auto& last_node = *std::prev(nodes_.end());
		auto& ins_node = nodes_.emplace_back(ee_types_.size());
		ins_node.id_ = id;
		init_node(&ins_node, &last_node, ee_types_, move_type, ee_pos_internal.data(), mid_pos_internal.data(), vel, acc, jerk, zone);
		if (move_type == Node::MoveType::ResetInitPos) {
			last_node.next_node_.store(&ins_node);
			return;
		}
		Node::make_ee_plan_path(&last_node, &ins_node);

		// 获得需要重新规划的起点，默认为5段轨迹
		auto current_node = current_node_.load();
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

		// 重规划 scurve
		auto scurve_size = aris::dynamic::getScurveSize(ee_types_);
		replan_scurve(scurve_size, replan_iter_end, nodes_.end());

		// 并发设置
		bool ret = std::prev(replan_iter_begin)->next_node_.exchange(&*replan_iter_end) != nullptr || replan_num == 0;

		// 如果成功，则删除需要重新规划的节点，否则删除新插入的节点
		if (ret) {
			nodes_.erase(replan_iter_begin, replan_iter_end);
		}
		else {
			nodes_.erase(replan_iter_end, nodes_.end());
		}
	}
}
