#ifndef ARIS_PLAN_TRAJECTORY_H_
#define ARIS_PLAN_TRAJECTORY_H_

#include <list>
#include <cmath>
#include <iostream>
#include <functional>
#include <map>
#include <any>

//#include <aris/core/core.hpp>
//#include <aris/control/control.hpp>
#include <aris/plan/scurve.hpp>
#include <aris/plan/path.hpp>

/// \brief 轨迹规划命名空间
/// \ingroup aris
/// 
///
///
/// 
/// 
namespace aris::plan{
	class ARIS_API TrajectoryGenerator {
	public:
		static auto internal_pos_to_outpos(const std::vector<aris::dynamic::EEType> &ee_types, const double *internal_pos, double *out_pos) {
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
		static auto outpos_to_internal_pos(const std::vector<aris::dynamic::EEType>& ee_types, const double* out_pos, double* internal_pos) {
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


		struct Node {
			enum class MoveType {
				Line,
				Circle,
				ResetInitPos
			};
			struct Zone {
				struct Lines {
					double p0_[3], p1_[3], p2_[3];
				};
				struct LineCircle {
					double p0_[3], p1_[3], center_[3], axis_[3], theta_;
				};
				struct CircleLine {
					double p0_[3], p1_[3], center_[3], axis_[3], theta_;
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
				aris::dynamic::EEType ee_type_{ aris::dynamic::EEType::PE123 };
				// Move 种类
				MoveType move_type_{ MoveType::Line };
				// scurves
				SCurveParam scurve_x_, scurve_a_;        // s 曲线
				// zones
				Zone zone_x1_, zone_a1_, zone_x2_, zone_a2_; // 转弯区参数，移动、转动、开始、结束，因此有4个
				// 主运动部分
				Move move_x_, move_a_;

				auto getInternalEndPos(Node::MoveType move_type, aris::dynamic::EEType ee_type, double* end_pos)const noexcept->void;
			};
			
			static auto inline make_zone_x_line_line(EePlanData* last_p, EePlanData* this_p)->void;
			static auto inline make_zone_a_line_line(EePlanData* last_p, EePlanData* this_p)->void;

			static auto init_ee_plans(
				Node::MoveType move_type,
				const double* ee_pos,
				const double* mid_pos,
				const double* vel,
				const double* acc,
				const double* jerk,
				const double* zone,
				const double* begin_pos,   // 为 nullptr 时，就用last_node
				const Node* last_node,
				const std::vector<aris::dynamic::EEType>& ee_types,
				std::vector<EePlanData>& ee_plans)->void;
			
			static auto make_ee_plan_path(Node* last_node, Node* node)->void;

			std::int64_t id_;
			std::vector<EePlanData> ee_plans_;
			std::atomic<Node*>      next_node_;

			~Node() = default;
			Node(aris::Size ee_size) {
				id_ = 1;
				ee_plans_.resize(ee_size);
			}
			Node(const Node& other):
				ee_plans_(other.ee_plans_),
				id_(other.id_)
			{
			}
			Node& operator=(const Node& other) {
				id_ = other.id_;
				ee_plans_ = other.ee_plans_;
				return *this;
			}
		};
		// 配置末端类型 //
		auto setEeTypes(const std::vector<aris::dynamic::EEType>& ee_types)->void {
			ee_types_ = ee_types;
			outpos_size_ = aris::dynamic::getEETypePosSize(ee_types_);
			//begin_pos.resize(outpos_size_, 0.0);

			this->internal_pos_size = 0;
			for (auto type : ee_types) {
				switch (type){
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

			this->internal_pos.resize(internal_pos_size);
		}

		// 设置时间间隔 //
		auto setDt(double dt)->void {
			dt_ = dt;
		}

		// 设置时间间隔 //
		auto setTargetDs(double ds)->void {
			target_ds_ = ds;
		}

		// 设置时间流逝的加速度
		auto setMaxDds(double max_dds)->void {
			max_dds_ = max_dds;
		}

		// 设置时间流逝的加加速度
		auto setMaxDdds(double max_ddds)->void {
			max_ddds_ = max_ddds;
		}

		// 获取末端数据，并移动dt //
		auto getEePosAndMoveDt(double *ee_pos)->std::int64_t;

		// 初始化数据 //
		auto init(const double* ee_pos)->void {
			std::lock_guard<std::recursive_mutex> lck(mu_);
			nodes_.clear();
			nodes_.emplace_back(ee_types_.size());
		}

		// 插入新的数据，并重规划 //
		auto insertPos(
			std::int64_t id,
			Node::MoveType move_type,
			const double* ee_pos,
			const double* mid_pos,
			const double* vel,
			const double* acc,
			const double* jerk,
			const double* zone)->void;

		//auto setInitPos(const double* ee_pos)noexcept->void {
		//	aris::dynamic::s_vc(begin_pos.size(), ee_pos, begin_pos.data());
		//	outpos_to_internal_pos(ee_types_, begin_pos.data(), internal_begin_pos.data());
		//}

	private:
		// 时间参数 //
		double max_ds_{ 1.0 }, max_dds_{ 10.0 }, max_ddds_{ 100.0 };
		double dt_{ 0.001 };
		double s_{ 0.0 }, ds_{ 1.0 }, dds_{ 0.0 }, ddds_{ 0.0 };
		int s_count_{0};
		double target_ds_{ 1.0 };

		// 规划节点
		std::vector<aris::dynamic::EEType> ee_types_;
		//std::vector<double> begin_pos, internal_begin_pos; // 前者可能是各种形式，后者只有 pq，不包括 pe313 等
		aris::Size outpos_size_{ 0 }, internal_pos_size{ 0 };
		
		std::list<Node> nodes_;

		// 互斥区，保护访问
		std::recursive_mutex mu_;
		// 当前正在使用的节点 //
		std::atomic<Node*> current_node_;

		std::vector<double> internal_pos;
	};
}

#endif