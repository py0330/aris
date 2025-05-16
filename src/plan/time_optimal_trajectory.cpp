#include"aris/plan/time_optimal_trajectory.hpp"
#include"aris/plan/function.hpp"

#include "aris/core/core.hpp"

namespace aris::plan {
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

	auto internal_pos_to_outpos(const std::vector<aris::dynamic::EEType>& ee_types, const double* internal_pos, double* out_pos) -> void;
	auto outpos_to_internal_pos(const std::vector<aris::dynamic::EEType>& ee_types, const double* out_pos, double* internal_pos) -> void;

	// make & compute raw data // 
	auto s_make_line3(const double* p0, const double* p1, double* dir, double& length) -> void;
	auto s_compute_line3_at(const double* p0, const double* dir, double l, double* xyz, double dl = 0.0, double* dxyz = nullptr, double d2l = 0.0, double* d2xyz = nullptr) -> void;

	auto s_make_line2(const double* p0, const double* p1, double* dir, double& length);
	auto s_compute_line2_at(const double* p0, const double* dir, double l, double* xyz, double dl = 0.0, double* dxyz = nullptr, double d2l = 0.0, double* d2xyz = nullptr) -> void;

	auto s_make_circle3(const double* p0, const double* p1, const double* p2, double* center, double* axis, double& radius, double& length) -> void;
	auto s_compute_circle3_at(const double* p0, const double* center, const double* axis, double radius, double total_length, double l, double* xyz, double dl = 0.0, double* dxyz = nullptr, double d2l = 0.0, double* d2xyz = nullptr) -> void;

	auto s_make_circle2(const double* p0, const double* p1, const double* p2, double* center, double* axis, double& radius, double& length) -> void;
	auto s_compute_circle2_at(const double* p0, const double* center, const double* axis, double radius, double total_length, double l, double* xyz, double dl = 0.0, double* dxyz = nullptr, double d2l = 0.0, double* d2xyz = nullptr) -> void;

	auto s_make_quaternion_data(const double* q0, const double* q1, double& length) -> void;
	auto s_compute_quaternion_at(const double* q0, const double* q1, double total_length, double l, double* q, double dl = 0.0, double* dq = nullptr, double d2l = 0.0, double* d2q = nullptr) -> void;

	auto s_compute_data_at_end(const Node::Unit& unit, double* p_end) -> void;


	// init unit //
	auto init_unit_l3(const double* p0, const double* p1, double vel, double acc, double jerk, double zone, Node::Unit& unit) -> void;
	auto init_unit_l2(const double* p0, const double* p1, double vel, double acc, double jerk, double zone, Node::Unit& unit) -> void;
	auto init_unit_c3(const double* p0, const double* p1, const double* p2, double vel, double acc, double jerk, double zone, Node::Unit& unit) -> void;
	auto init_unit_c2(const double* p0, const double* p1, const double* p2, double vel, double acc, double jerk, double zone, Node::Unit& unit) -> void;
	auto init_unit_q3(const double* q0, const double* q1, double vel, double acc, double jerk, double zone, Node::Unit& unit) -> void;
	auto init_unit_l1(const double* p0, const double* p1, double vel, double acc, double jerk, double zone, Node::Unit& unit) -> void;

	auto init_unit(Node::UnitType unit_type, const double* p0, const double* p1, const double* p2, double vel, double acc, double jerk, double zone, Node::Unit& unit) -> void;

	// connect unit to last unit //
	auto make_zone_and_scurve_ll(Node::Unit& last_u, Node::Unit& this_u) -> void;
	auto make_zone_and_scurve_lc(Node::Unit& last_u, Node::Unit& this_u) -> void;
	auto make_zone_and_scurve_cl(Node::Unit& last_u, Node::Unit& this_u) -> void;
	auto make_zone_and_scurve_cc(Node::Unit& last_u, Node::Unit& this_u) -> void;
	auto make_zone_and_scurve_qq(Node::Unit& last_u, Node::Unit& this_u) -> void;

	auto make_zone_and_scurve(Node::Unit& last_u, Node::Unit& this_u, bool if_need_connect) -> void;

	// get unit data //
	auto get_zone_data(double arc, double darc, double d2arc, Node::UnitType type, const Node::Zone& z, double* p, double* dp, double* d2p) -> void;
	auto get_move_data(double arc, double darc, double d2arc, Node::UnitType type, const Node::Move& m, double* p, double* dp, double* d2p) -> void;
	auto get_unit_data(double arc, const Node::Unit& u, double* p, double* dp_darc, double* d2p_darc2) -> void;

	// make nodes //
	auto make_node(aris::Size replan_num, Node* this_node, Node* last_node, const std::vector<aris::dynamic::EEType>& ee_types,
		Node::NodeType node_type, const double* ee_pos, const double* mid_pos, const double* vel, const double* acc, const double* jerk, const double* zone
	) -> void;
	auto replan_nodes(int scurve_size, const std::vector<aris::dynamic::EEType> &ee_types, std::list<Node>::iterator last, std::list<Node>::iterator begin, std::list<Node>::iterator end) -> int;
	auto get_node_data(const std::vector<aris::dynamic::EEType> &ee_types, const Node* current_node, LargeNum s, double ds, double dds, double ddds,
		double* internal_pos, double* internal_vel, double* internal_acc) -> void;


	struct SmoothParam2 {
		double dt;
		int dim;
		const double* min_p, * max_p, * min_dp, * max_dp, * min_d2p, * max_d2p, * min_d3p, * max_d3p;
		double min_ds, max_ds, min_d2s, max_d2s, min_d3s, max_d3s;
		double ds1, ds2, ds3;
		double* p0, * p1, * p2, * p3;

		double target_ds;
	};
	struct SmoothRet2 {
		double d3s_lhs0, d3s_rhs0, d3s_lhs1, d3s_rhs1, d3s_lhs2, d3s_rhs2, d3s_lhs3, d3s_rhs3;
		double d3s_lhs_all, d3s_rhs_all;
	};

	auto s_smooth_curve4(const SmoothParam2& param, SmoothRet2& ret) -> int {

		///////////////////////////// PART 1 计算 d3s 与 d3p 的关系 ///////////////////////// 
		//
		// dp  = dp_ds * ds
		// d2p = d2p_ds2 * ds^2 + dp_ds * d2s
		// d3p = d3p_ds3 * ds^3 + 2 * d2p_ds2 * ds * d2s + d2p_ds2 * ds * d2s + dp_ds * d3s
		//     = d3p_ds3 * ds^3 + 3 * d2p_ds2 * ds * d2s + dp_ds * d3s
		// 
		///////////////////////////// PART 2 计算d3s4 ///////////////////////////////// 
		// t0        t1   t15   t2   t25   t3
		//
		// p0        p1         p2         p3
		//     dp1        dp2        dp3
		//          d2p2       d2p3
		//               d3p3
		//  
		// s0        s1         s2         s3
		//     ds1        ds2        ds3
		//          d2s2       d2s3
		//               d3s3
		//
		// at point t=1.5:
		//
		// dp_ds_t15   = dp_t15/ds_t15
		// d2p_ds2_t15 = (d2p_t15 - dp_ds_t15 * d2s_t15)/ds_t15^2
		// d3p_ds3_t15 = (d3p_t15 - 3 * d2p_ds2_t15 * ds_t15 * d2s_t15 - dp_ds_t15 * d3s_t15)/ds_t15^3
		// 
		// at point t=2.5:
		// 
		// ds_t25 = ds3
		// d2s_t25 = d2s_t15 + (d3s_t15+d3s_25)/2*dt
		// 
		// d3p_ds3_t25 = d3p_ds3_t15
		// d2p_ds2_t25 = d2p_ds2_t15 + d3p_ds3_t15 * (s_25 - s_15)
		// dp_ds_t25   = dp_ds_t15 + d2p_ds2_t15 * (s_25 - s_15) + 0.5 * d3p_ds3_t15 * (s_25 - s_15)^2
		// 
		// 于是：
		// d3p4 = d3p_ds3_25 * ds_25^3 + 3 * d2p_ds2_25 * ds_25 * d2s_25 + dp_ds_25 * d3s4
		//      = k * d3s4 + g
		// 其中:
		//    k = (dp_ds_t25 + (3 * d2p_ds2_t25 * ds_t25 * dt) / 2)
		//    g = d3p_ds3_t25 * ds_t25 * ds_t25 * ds_t25 + 3 * d2p_ds2_t25 * ds3 * d2s3
		//    
		///////////////////////////// PART 3 列不等式 ///////////////////////////////// 
		// 
		// p4   = p3   + dp4  * dt
		//      = p3   + dp3  * dt + d2p3 * dt^2 + d3p4 * dt^3
		// dp4  = dp3  + d2p4 * dt
		//      = dp3  + d2p3 * dt + d3p4 * dt^2
		// d2p4 = d2p3 + d3p4 * dt
		// 
		// 考虑约束：
		// p_min   <   p4 < p_max
		// dp_min  <  dp4 < dp_max
		// d2p_min < d2p4 < d2p_max
		// d3p_min < d3p4 < d3p_max
		// 
		// =>
		// (p_min   - e1)/f1 < d3s4 < (p_max   - e1)/f1
		// (dp_min  - e2)/f2 < d3s4 < (dp_max  - e2)/f2
		// (d2p_min - e3)/f3 < d3s4 < (d2p_max - e3)/f3
		// (d3p_min - e4)/f4 < d3s4 < (d3p_max - e4)/f4
		// 
		// 其中：
		// f1 = k * dt * dt * dt;
		// f2 = k * dt * dt;
		// f3 = k * dt;
		// f4 = k;
		// e1 = p3 + dp3 * dt + d2p3 * dt * dt + g * dt * dt * dt;
		// e2 = dp3 + d2p3 * dt + g * dt * dt;
		// e3 = d2p3 + g * dt;
		// e4 = g;
		// 
		///////////////////////////// PART 4 ds 的限制 /////////////////////////////////
		// 
		// 应有：
		//       0 <  ds  < 1
		// MIN_D2S <  d2s < MAX_D2S
		// MIN_D3S <  d3s < MAX_D3S
		// 
		// ds4  = ds3  + d2s3 * dt + d3s4 * dt^2
		// d2s4 = d2s3 + d3s4 * dt
		// 
		// =>
		// 
		// (MIN_DS - ds3 - d2s3 * dt)/dt^2 < d3s4 < (MAX_DS - ds3 - d2s3 * dt)/dt^2
		//             (MIN_D2S - d2s3)/dt < d3s4 < (MAX_D2S - d2s3)/dt
		//                         MIN_D3S < d3s4 < MAX_D3S
		//

		double zero_check = 1e-10;

		const double MAX_DS = param.max_ds;
		const double MIN_DS = param.min_ds;
		const double MAX_D2S = param.max_d2s;
		const double MIN_D2S = param.min_d2s;
		const double MAX_D3S = param.max_d3s;
		const double MIN_D3S = param.min_d3s;

		double dt = param.dt;
		auto dim = param.dim;

		auto ds1 = param.ds1;
		auto ds2 = param.ds2;
		auto ds3 = param.ds3;

		auto p3 = param.p3;
		auto p2 = param.p2;
		auto p1 = param.p1;
		auto p0 = param.p0;

		auto p_max = param.max_p;
		auto p_min = param.min_p;
		auto dp_max = param.max_dp;
		auto dp_min = param.min_dp;
		auto d2p_max = param.max_d2p;
		auto d2p_min = param.min_d2p;
		auto d3p_max = param.max_d3p;
		auto d3p_min = param.min_d3p;

		auto d2s2 = (ds2 - ds1) / dt;
		auto d2s3 = (ds3 - ds2) / dt;
		auto d3s3 = (d2s3 - d2s2) / dt;

		double ds_t15 = ds2;
		double d2s_t15 = (d2s2 + d2s3) / 2;
		double d3s_t15 = d3s3;
		double ds_t25 = ds3;

		/////// ds 起始值 //////

		double MIN_d3s4 = std::max({
				(MIN_DS - ds3 - d2s3 * dt) / dt / dt, (MIN_D2S - d2s3) / dt , MIN_D3S
			});

		double MAX_d3s4 = std::min({
				(MAX_DS - ds3 - d2s3 * dt) / dt / dt, (MAX_D2S - d2s3) / dt , MAX_D3S
			});

		////////////



		double d3s_lhs0{ MIN_d3s4 }, d3s_rhs0{ MAX_d3s4 },
			d3s_lhs1{ MIN_d3s4 }, d3s_rhs1{ MAX_d3s4 },
			d3s_lhs2{ MIN_d3s4 }, d3s_rhs2{ MAX_d3s4 },
			d3s_lhs3{ MIN_d3s4 }, d3s_rhs3{ MAX_d3s4 };

		for (int i = 0; i < dim; ++i) {
			auto dp3 = (p3[i] - p2[i]) / dt;
			auto dp2 = (p2[i] - p1[i]) / dt;
			auto dp1 = (p1[i] - p0[i]) / dt;

			auto d2p3 = (dp3 - dp2) / dt;
			auto d2p2 = (dp2 - dp1) / dt;

			auto d3p3 = (d2p3 - d2p2) / dt;

			auto dp_t15 = dp2;
			auto d2p_t15 = (d2p2 + d2p3) / 2;
			auto d3p_t15 = d3p3;

			auto dp_ds_t15 = dp_t15 / ds_t15;
			auto d2p_ds2_t15 = (d2p_t15 - dp_ds_t15 * d2s_t15) / ds_t15 / ds_t15;
			auto d3p_ds3_t15 = (d3p_t15 - 3 * d2p_ds2_t15 * ds_t15 * d2s_t15 - dp_ds_t15 * d3s_t15) / ds_t15 / ds_t15 / ds_t15;

			auto s25_s15 = (ds2 + ds3) / 2 * dt;
			auto d3p_ds3_t25 = d3p_ds3_t15;
			auto d2p_ds2_t25 = d2p_ds2_t15 + d3p_ds3_t15 * s25_s15;
			auto dp_ds_t25 = dp_ds_t15 + d2p_ds2_t15 * s25_s15 + 0.5 * d3p_ds3_t15 * s25_s15 * s25_s15;

			auto k = (dp_ds_t25 + (3 * d2p_ds2_t25 * ds_t25 * dt) / 2);
			auto g = d3p_ds3_t25 * ds_t25 * ds_t25 * ds_t25 + 3 * d2p_ds2_t25 * ds3 * d2s3;

			auto f1 = k * dt * dt * dt;
			auto f2 = k * dt * dt;
			auto f3 = k * dt;
			auto f4 = k;
			auto e1 = p3[i] + dp3 * dt + d2p3 * dt * dt + g * dt * dt * dt;
			auto e2 = dp3 + d2p3 * dt + g * dt * dt;
			auto e3 = d2p3 + g * dt;
			auto e4 = g;

			if (std::abs(k) > zero_check) {
				auto lhs0_local = (p_min[i] - e1) / f1;
				auto rhs0_local = (p_max[i] - e1) / f1;

				auto lhs1_local = (dp_min[i] - e2) / f2;
				auto rhs1_local = (dp_max[i] - e2) / f2;

				auto lhs2_local = (d2p_min[i] - e3) / f3;
				auto rhs2_local = (d2p_max[i] - e3) / f3;

				auto lhs3_local = (d3p_min[i] - e4) / f4;
				auto rhs3_local = (d3p_max[i] - e4) / f4;

				if (k < 0) {
					std::swap(lhs0_local, rhs0_local);
					std::swap(lhs1_local, rhs1_local);
					std::swap(lhs2_local, rhs2_local);
					std::swap(lhs3_local, rhs3_local);
				}

				d3s_lhs0 = std::max(d3s_lhs0, lhs0_local);
				d3s_lhs1 = std::max(d3s_lhs1, lhs1_local);
				d3s_lhs2 = std::max(d3s_lhs2, lhs2_local);
				d3s_lhs3 = std::max(d3s_lhs3, lhs3_local);
				d3s_rhs0 = std::min(d3s_rhs0, rhs0_local);
				d3s_rhs1 = std::min(d3s_rhs1, rhs1_local);
				d3s_rhs2 = std::min(d3s_rhs2, rhs2_local);
				d3s_rhs3 = std::min(d3s_rhs3, rhs3_local);
			}
		}

		ret.d3s_lhs0 = d3s_lhs0;
		ret.d3s_lhs1 = d3s_lhs1;
		ret.d3s_lhs2 = d3s_lhs2;
		ret.d3s_lhs3 = d3s_lhs3;
		ret.d3s_rhs0 = d3s_rhs0;
		ret.d3s_rhs1 = d3s_rhs1;
		ret.d3s_rhs2 = d3s_rhs2;
		ret.d3s_rhs3 = d3s_rhs3;

		ret.d3s_lhs_all = std::max({ d3s_lhs0 ,d3s_lhs1 ,d3s_lhs2 ,d3s_lhs3 });
		ret.d3s_rhs_all = std::min({ d3s_rhs0 ,d3s_rhs1 ,d3s_rhs2 ,d3s_rhs3 });

		return 0;
	};

	struct LookAheadParam {
		enum NodeType {
			DEC,
			ACC
		};
		struct Node {
			double s_, ds_, d2s_;
			NodeType type_;
		};

		std::list<Node> nodes_;


	};

	struct LookAheadProcessor::Imp {
		LookAheadParam param_;

		InverseKinematicMethod inv_func_;

		aris::Size input_size_{ 0 };

		std::vector<char> mem_;
		double
			* max_poss_,
			* max_vels_,
			* max_accs_,
			* max_jerks_,
			* min_poss_,
			* min_vels_,
			* min_accs_,
			* min_jerks_,
			* smooth_max_poss_,
			* smooth_max_vels_,
			* smooth_max_accs_,
			* smooth_max_jerks_,
			* smooth_min_poss_,
			* smooth_min_vels_,
			* smooth_min_accs_,
			* smooth_min_jerks_,
			* input_pos_begin_,// 奇异状态的起始值
			* input_vel_begin_,
			* input_acc_begin_,
			* input_pos_end_,  // 奇异状态的终止值
			* input_vel_end_,
			* input_acc_end_,
			* input_pos_last_, // 真实状态值
			* input_vel_last_,
			* input_pos_this_,
			* input_vel_this_,
			* input_acc_this_,
			* input_acc_ratio_,
			* input_acc_max_consider_ratio_,
			* input_acc_min_consider_ratio_,
			* output_pos_,
			* p_, 
			* u_,
			* s_;

		std::int64_t* cmd_ids_;

		std::int32_t* Ts_count_;

		std::int64_t tg_ret_{ 0 }, tg_ret_begin_{ 0 };
		aris::Size total_singular_count_{ 0 }, current_singular_count_{ 0 };
		//CurveParam* curve_params_;

		double target_ds_{ 1.0 };
		double ds1_{ 1.0 }, ds2_{ 1.0 }, ds3_{ 1.0 };
		
		int p_mem_size_{ 6 }, p_pop_idx_{ 0 }, p_push_idx_{ 0 };
		
		std::int64_t current_cmd_id_{ 0 };

		aris::dynamic::ModelBase* model_{ nullptr };
		aris::plan::TimeOptimalTrajectoryGenerator* tg_{ nullptr };

		// 是否已经处于奇异状态，或者是否还在继续准备处理奇异情况 //
		int singular_idx;

		enum class SingularState {
			SINGULAR,
			SINGULAR_PREPARE,
			NORMAL
		};

		SingularState state_{ SingularState::NORMAL };

		std::vector<std::pair<double, double>> su_series_;

		auto test_future_plan_success() -> int {
			double u_diff = tg_->dt();
			int idx = 4;

			std::vector<double> p(input_size_ * 5), output_pos(model_->outputPosSize());
			double u[5], s[5];

			for (int i = 0; i < 5; ++i) {
				std::copy_n(this->p_ + ((p_push_idx_ + p_mem_size_ + i - 4) % p_mem_size_) * input_size_, input_size_, p.data() + input_size_ * i);
				u[i] = u_[(p_push_idx_ + p_mem_size_ + i - 4) % p_mem_size_];
				s[i] = s_[(p_push_idx_ + p_mem_size_ + i - 4) % p_mem_size_];
			}


			for (int i = 0;i<1000;++i) {
				idx = (idx + 1)%5;

				auto& s0 = s[(idx + 5 - 4) % 5];
				auto& s1 = s[(idx + 5 - 3) % 5];
				auto& s2 = s[(idx + 5 - 2) % 5];
				auto& s3 = s[(idx + 5 - 1) % 5];
				auto& s4 = s[(idx + 5 - 0) % 5];
				auto& u0 = u[(idx + 5 - 4) % 5];
				auto& u1 = u[(idx + 5 - 3) % 5];
				auto& u2 = u[(idx + 5 - 2) % 5];
				auto& u3 = u[(idx + 5 - 1) % 5];
				auto& u4 = u[(idx + 5 - 0) % 5];

				auto p0 = p.data() + ((idx + 5 - 4) % 5) * input_size_;
				auto p1 = p.data() + ((idx + 5 - 3) % 5) * input_size_;
				auto p2 = p.data() + ((idx + 5 - 2) % 5) * input_size_;
				auto p3 = p.data() + ((idx + 5 - 1) % 5) * input_size_;
				auto p4 = p.data() + ((idx + 5 - 0) % 5) * input_size_;

				//aris::dynamic::dsp(5, 6, p.data());

				
				u4 = u3 + u_diff;
				
				SmoothParam2 param{
					u_diff,
					input_size_,
					smooth_min_poss_, smooth_max_poss_,
					smooth_min_vels_, smooth_max_vels_,
					smooth_min_accs_, smooth_max_accs_,
					smooth_min_jerks_, smooth_max_jerks_,
					0.005,1.0,-100.0,100.0,-10000.0,10000.0,
					(s1 - s0)/u_diff, (s2 - s1)/u_diff, (s3 - s2)/u_diff,
					p0, p1, p2, p3,
					target_ds_
				};
				SmoothRet2 smooth_ret;
				s_smooth_curve4(param, smooth_ret);


				auto ds2 = (s2 - s1) / u_diff;
				auto ds3 = (s3 - s2) / u_diff;
				auto d2s3 = (ds3 - ds2) / u_diff;

				if ((s3 - s2) / u_diff + d2s3 * u_diff < 0.01)
					return 0;

				if (smooth_ret.d3s_lhs_all > smooth_ret.d3s_rhs_all)
					return -1;

				auto d3s4 = smooth_ret.d3s_lhs_all;

				auto d2s4 = d2s3 + d3s4 * u_diff;
				auto ds4 = ds3 + d2s4 * u_diff;
				s4 = s3 + ds4 * u_diff;


				auto ret = tg_->getEePosByS(s4, output_pos.data());

				if (ret == 0)
					return 0;

				model_->inverseKinematics(output_pos.data(), p4, model_->inverseRootNumber(), p3);
			}

			return 0;
		}
	};
	



	
	auto s_cpt_d3u_lr(int p_size, const double* p0, const double* p1, const double* p2, const double* p3,
		const double* p_min, const double* p_max, const double* dp_min, const double* dp_max,
		const double* d2p_min, const double* d2p_max, const double* d3p_min, const double* d3p_max,
		double s_diff, double u0, double u1, double u2, double& d3u_ds3_3_L, double & d3u_ds3_3_R, double zero_check)-> void
	{
		const double MAX_DS = 1;
		const double MIN_DS = 0;
		const double MAX_D2S = 1;
		const double MIN_D2S = 1;
		const double MAX_D3S = 1;
		const double MIN_D3S = 1;

		double
			lhs_s{ -1e10 }, rhs_s{ 1e10 },
			lhs_ds{ MIN_DS }, rhs_ds{ MAX_DS },
			lhs_d2s{ MIN_D2S }, rhs_d2s{ MAX_D2S },
			lhs_d3s{ MIN_D3S }, rhs_d3s{ MAX_D3S }
		;

		auto u_diff_1 = u1 - u0;
		auto u_diff_2 = u2 - u1;
		//auto u_diff_diff = u_diff_2 - u_diff_1;
		//auto u_diff_3 = u2 + (u_diff_2 + u_diff_diff);
		auto u_diff_3 = u_diff_2;

		auto du_ds_1 = u_diff_1 / s_diff;
		auto du_ds_2 = u_diff_2 / s_diff;
		auto du_ds_15 = (du_ds_1 + du_ds_2) / 2.0;

		auto d2u_ds2_2 = (du_ds_2 - du_ds_1) / s_diff;

		auto ds_du_1 = 1.0 / du_ds_1;
		auto ds_du_2 = 1.0 / du_ds_2;
		auto ds_du_15 = (ds_du_1 + ds_du_2) / 2;

		auto d2s_du2_2 = -d2u_ds2_2 / (du_ds_15* du_ds_15* du_ds_15);

		auto d3s_du3_3_L = std::numeric_limits<double>::lowest();
		auto d3s_du3_3_R = std::numeric_limits<double>::max();

		for (int i = 0; i < p_size; ++i) {
			auto dp_ds_3 = (p3[i] - p2[i]) / s_diff;
			auto dp_ds_2 = (p2[i] - p1[i]) / s_diff;
			auto dp_ds_1 = (p1[i] - p0[i]) / s_diff;
			auto dp_ds_15 = (dp_ds_1 + dp_ds_2) / 2;

			auto d2p_ds2_2 = (dp_ds_2 - dp_ds_1) / s_diff;
			auto d2p_ds2_3 = (dp_ds_3 - dp_ds_2) / s_diff;
			auto d2p_ds2_25 = (d2p_ds2_2 + d2p_ds2_3) / 2;

			auto d3p_ds3_3 = (d2p_ds2_3 - d2p_ds2_2) / s_diff;

			auto dp_du_2 = dp_ds_2 * ds_du_2;
			auto d2p_du2_2 = d2p_ds2_2 * ds_du_15* ds_du_15 + dp_ds_15 * d2s_du2_2;
			
			auto k = (dp_ds_2 + (3 * d2p_ds2_25 * ds_du_2 * u_diff_2) / 2);
			auto g = d3p_ds3_3 * (ds_du_2*ds_du_2*ds_du_2) + 3 * d2p_ds2_25 * ds_du_2 * d2s_du2_2;

			auto f1 = k * u_diff_3 * u_diff_3 * u_diff_3;
			auto f2 = k * u_diff_3 * u_diff_3;
			auto f3 = k * u_diff_3;
			auto f4 = k;

			auto e1 = p2[i] + dp_du_2 * u_diff_3 + d2p_du2_2 * u_diff_3 * u_diff_3 + g * u_diff_3 * u_diff_3 * u_diff_3;
			auto e2 = dp_du_2 + d2p_du2_2 * u_diff_3 + g * u_diff_3 * u_diff_3;
			auto e3 = d2p_du2_2 + g * u_diff_3;
			auto e4 = g;

			if (std::abs(k) > zero_check &&
				// 本条件也在限制 dp_ds 不能太小，如果 k 递增，或者 d2p_ds2_t25 产生的效应超过其他项
				(std::abs(k) > std::abs(dp_ds_2) || std::abs(dp_ds_2) > std::abs(3 * d2p_ds2_25 * ds_du_2 * u_diff_2))
				) 
			{
				auto lhs1_local = (p_min[i] - e1) / f1;
				auto rhs1_local = (p_max[i] - e1) / f1;

				auto lhs2_local = (dp_min[i] - e2) / f2;
				auto rhs2_local = (dp_max[i] - e2) / f2;

				auto lhs3_local = (d2p_min[i] - e3) / f3;
				auto rhs3_local = (d2p_max[i] - e3) / f3;

				auto lhs4_local = (d3p_min[i] - e4) / f4;
				auto rhs4_local = (d3p_max[i] - e4) / f4;

				if (k < 0) {
					std::swap(lhs1_local, rhs1_local);
					std::swap(lhs2_local, rhs2_local);
					std::swap(lhs3_local, rhs3_local);
					std::swap(lhs4_local, rhs4_local);
				}

				d3s_du3_3_L = std::max({ d3s_du3_3_L, lhs1_local, lhs2_local, lhs3_local, lhs4_local });
				d3s_du3_3_R = std::min({ d3s_du3_3_R, rhs1_local, rhs2_local, rhs3_local, rhs4_local });
			}
		}

		// 限制 ds_du 在 0~1内
		// d2s_du2_3 = d2s_du2_2 + d3s_du3_3*u_diff
		// ds_du_3 = ds_du_2 + d2s_du2_3*u_diff
		//         = ds_du_2 + d2s_du2_2*u_diff + d3s_du3_3*u_diff*u_diff
		// 
		//d3s_du3_3_L = std::max({ d3s_du3_3_L, (MIN_DS - ds_du_2 - d2s_du2_2 * u_diff_3) / u_diff_3 / u_diff_3 });
		//d3s_du3_3_R = std::min({ d3s_du3_3_R, (MAX_DS - ds_du_2 - d2s_du2_2 * u_diff_3) / u_diff_3 / u_diff_3 });


		//% dx_dt   = 1/dt_dx
		//% d2x_dt2 = -1/(dt_dx)^2 * d2t_dx2 * dx_dt
		//%         = -d2t_dx2 / (dt_dx)^3
		//% d3x_dt3 = (3*(d2t_dx2)^2 - dt_dx * d3t_dx3) / (dt_dx)^5

		auto d2s_du2_25_L = d2s_du2_2 + d3s_du3_3_L * u_diff_2 / 2;
		auto d2s_du2_25_R = d2s_du2_2 + d3s_du3_3_R * u_diff_2 / 2;

		d3u_ds3_3_L = (3 * (d2s_du2_2 * d2s_du2_2) - ds_du_2 * d3s_du3_3_R) / std::pow(ds_du_2, 5);
		d3u_ds3_3_R = (3 * (d2s_du2_2 * d2s_du2_2) - ds_du_2 * d3s_du3_3_L) / std::pow(ds_du_2, 5);
	
	}
	

	auto s_interp_5th(int n, double T, const double* y0, const double* y1, const double* y2, const double* y3, const double* y4, double* dy_at_x2, double* d2y_at_x2, double* d3y_at_x2) -> void {
		for (int i = 0; i < n; ++i) {
			dy_at_x2[i] = (y0[i] - 8 * y1[i] + 8 * y3[i] - y4[i]) / (12 * T);
			d2y_at_x2[i] = (-y0[i] + 16 * y1[i] - 30 * y2[i] + 16 * y3[i] - y4[i]) / (12 * T * T);
			d3y_at_x2[i] = (-y0[i] + 2 * y1[i] - 2 * y3[i] + y4[i]) / (2 * T*T*T);
		}
	};


	auto s_cpt_d3u_lr2(int p_size, const double* p0, const double* p1, const double* p2, const double* p3, const double* p4,
		const double* p_min, const double* p_max, const double* dp_min, const double* dp_max,
		const double* d2p_min, const double* d2p_max, const double* d3p_min, const double* d3p_max,
		double s_diff, double u0, double u1, double u2, double u3, double& u4_L, double& u4_R, double zero_check = 1e-10) -> void
	{
		double du_ds_2 = (u3 - u1) / 2 / s_diff;
		double d2u_d2s_2 = (((u3 - u2) / s_diff) - ((u2 - u1) / s_diff))/s_diff;

		//% dx_dt   = 1/dt_dx
		//% d2x_dt2 = -1/(dt_dx)^2 * d2t_dx2 * dx_dt
		//%         = -d2t_dx2 / (dt_dx)^3
		double ds_du_2 = 1.0 / du_ds_2;
		double d2s_du2_2 = -d2u_d2s_2 / (du_ds_2* du_ds_2* du_ds_2);

		double dp_ds[12], d2p_ds2[12], d3p_ds3[12];
		s_interp_5th(p_size, s_diff, p0, p1, p2, p3, p4, dp_ds, d2p_ds2, d3p_ds3);

		double ds_l{ -1e10 }, ds_r{ 1e10 }, d2s_l{ -1e10 }, d2s_r{ 1e10 }, d3s_l{ -1e10 }, d3s_r{ 1e10 };
		for (int i = 0; i < p_size; ++i) {
			if (std::abs(dp_ds[i]) > zero_check) {
				//% dp  = dp_ds * ds
				//% d2p = d2p_ds2 * ds^2 + dp_ds * d2s
				//% d3p = d3p_ds3 * ds^3 + 2*d2p_ds2*ds*d2s + d2p_ds2*ds*d2s + dp_ds*d3s
				//%     = d3p_ds3 * ds^3 + 3*d2p_ds2*ds*d2s + dp_ds*d3s
				auto lhs_ds = dp_min[i] / dp_ds[i];
				auto rhs_ds = dp_max[i] / dp_ds[i];

				auto lhs_d2s = (d2p_min[i] - d2p_ds2[i] * ds_du_2 * ds_du_2) / dp_ds[i];
				auto rhs_d2s = (d2p_max[i] - d2p_ds2[i] * ds_du_2 * ds_du_2) / dp_ds[i];

				auto lhs_d3s = (d3p_min[i] - d3p_ds3[i] * ds_du_2 * ds_du_2 * ds_du_2 - 3 * d2p_ds2[i] * ds_du_2 * d2s_du2_2) / dp_ds[i];
				auto rhs_d3s = (d3p_max[i] - d3p_ds3[i] * ds_du_2 * ds_du_2 * ds_du_2 - 3 * d2p_ds2[i] * ds_du_2 * d2s_du2_2) / dp_ds[i];

				if (dp_ds[i] < 0) {
					std::swap(lhs_ds, rhs_ds);
					std::swap(lhs_d2s, rhs_d2s);
					std::swap(lhs_d3s, rhs_d3s);
				}

				ds_l = std::max({ ds_l, lhs_ds });
				ds_r = std::min({ ds_r, rhs_ds });
				d2s_l = std::max({ d2s_l, lhs_d2s });
				d2s_r = std::min({ d2s_r, rhs_d2s });
				d3s_l = std::max({ d3s_l, lhs_d3s });
				d3s_r = std::min({ d3s_r, rhs_d3s });
			}
		}

		u4_L = std::max({
			-12 * ds_l * s_diff + u0 - 8 * u1 + 8 * u3,
			-12 * d2s_l * (s_diff * s_diff) - u0 + 16 * u1 - 30 * u2 + 16 * u3, //- 12*d2y*T^2 - y0 + 16*y1 - 30*y2 + 16*y3
			2 * d3s_l * (s_diff * s_diff * s_diff) + u0 - 2 * u1 + 2 * u3,
			});

		u4_R = std::min({
			-12 * ds_r * s_diff  + u0 - 8 * u1 + 8 * u3,
			-12 * d2s_r * (s_diff * s_diff) - u0 + 16 * u1 - 30 * u2 + 16 * u3, //- 12*d2y*T^2 - y0 + 16*y1 - 30*y2 + 16*y3
			2 * d3s_r * (s_diff * s_diff * s_diff) + u0 - 2 * u1 + 2 * u3,
			});
	}


	auto LookAheadProcessor::lookAheadOneStep() -> int {
		double u_diff = imp_->tg_->dt();
		
		auto& s0 = imp_->s_[(imp_->p_push_idx_ + imp_->p_mem_size_ - 4) % imp_->p_mem_size_];
		auto& s1 = imp_->s_[(imp_->p_push_idx_ + imp_->p_mem_size_ - 3) % imp_->p_mem_size_];
		auto& s2 = imp_->s_[(imp_->p_push_idx_ + imp_->p_mem_size_ - 2) % imp_->p_mem_size_];
		auto& s3 = imp_->s_[(imp_->p_push_idx_ + imp_->p_mem_size_ - 1) % imp_->p_mem_size_];
		auto& s4 = imp_->s_[(imp_->p_push_idx_ + imp_->p_mem_size_ - 0) % imp_->p_mem_size_];
		auto& u0 = imp_->u_[(imp_->p_push_idx_ + imp_->p_mem_size_ - 4) % imp_->p_mem_size_];
		auto& u1 = imp_->u_[(imp_->p_push_idx_ + imp_->p_mem_size_ - 3) % imp_->p_mem_size_];
		auto& u2 = imp_->u_[(imp_->p_push_idx_ + imp_->p_mem_size_ - 2) % imp_->p_mem_size_];
		auto& u3 = imp_->u_[(imp_->p_push_idx_ + imp_->p_mem_size_ - 1) % imp_->p_mem_size_];
		auto& u4 = imp_->u_[(imp_->p_push_idx_ + imp_->p_mem_size_ - 0) % imp_->p_mem_size_];

		auto p0 = imp_->p_ + ((imp_->p_push_idx_ + imp_->p_mem_size_ - 4) % imp_->p_mem_size_) * imp_->input_size_;
		auto p1 = imp_->p_ + ((imp_->p_push_idx_ + imp_->p_mem_size_ - 3) % imp_->p_mem_size_) * imp_->input_size_;
		auto p2 = imp_->p_ + ((imp_->p_push_idx_ + imp_->p_mem_size_ - 2) % imp_->p_mem_size_) * imp_->input_size_;
		auto p3 = imp_->p_ + ((imp_->p_push_idx_ + imp_->p_mem_size_ - 1) % imp_->p_mem_size_) * imp_->input_size_;
		auto p4 = imp_->p_ + ((imp_->p_push_idx_ + imp_->p_mem_size_ - 0) % imp_->p_mem_size_) * imp_->input_size_;

		auto& cmd_id4 = imp_->cmd_ids_[((imp_->p_push_idx_ + imp_->p_mem_size_ - 0) % imp_->p_mem_size_)];

		u4 = u3 + u_diff;

		SmoothParam2 param{
			imp_->tg_->dt(),
			imp_->input_size_,
			imp_->smooth_min_poss_, imp_->smooth_max_poss_,
			imp_->smooth_min_vels_, imp_->smooth_max_vels_,
			imp_->smooth_min_accs_, imp_->smooth_max_accs_,
			imp_->smooth_min_jerks_, imp_->smooth_max_jerks_,
			0.005,1.0,-100.0,100.0,-10000.0,10000.0,
			(s1 - s0)/u_diff, (s2 - s1) / u_diff, (s3 - s2)/u_diff,
			p0, p1, p2, p3,
			imp_->target_ds_
		};
		SmoothRet2 smooth_ret;
		s_smooth_curve4(param, smooth_ret);

		
		auto ds2 = (s2 - s1) / u_diff;
		auto ds3 = (s3 - s2) / u_diff;
		auto d2s3 = (ds3 - ds2) / u_diff;

		auto d3s4 = smooth_ret.d3s_rhs_all;
		
		auto d2s4 = d2s3 + d3s4 * u_diff;
		auto ds4 = std::max(ds3 + d2s4 * u_diff, 0.005);
		s4 = s3 + ds4 * u_diff;

		cmd_id4 = imp_->tg_->getEePosByS(s4, imp_->output_pos_);
		imp_->model_->inverseKinematics(imp_->output_pos_, p4, imp_->model_->inverseRootNumber());

		if (imp_->test_future_plan_success() == 0) {
			//std::cout << "success:" << s4 << std::endl;

		}
		else {
			d3s4 = smooth_ret.d3s_lhs_all;

			d2s4 = d2s3 + d3s4 * u_diff;
			ds4 = std::max(ds3 + d2s4 * u_diff, 0.005);
			s4 = s3 + ds4 * u_diff;

			cmd_id4 = imp_->tg_->getEePosByS(s4, imp_->output_pos_);
			imp_->model_->inverseKinematics(imp_->output_pos_, p4, imp_->model_->inverseRootNumber());

			std::cout << "failed:" << s4 <<"  ds:" << ds4  << "  ret:" << imp_->current_cmd_id_ << std::endl;
		}

		if(cmd_id4 == 0)
			std::cout << "  ret2:" << imp_->current_cmd_id_ << std::endl;

		//imp_->u3_ = std::min(imp_->u3_, imp_->u2_ + 100 * s_diff);
		//imp_->u3_ = std::max(imp_->u3_, imp_->u2_ + s_diff);

		//imp_->su_series_.push_back({ imp_->s3_, imp_->u3_ });
		imp_->p_push_idx_ += 1;
		return imp_->current_cmd_id_;
	}
	auto LookAheadProcessor::lookAhead(double s_begin) -> int {
		
		while ((imp_->p_push_idx_ - imp_->p_pop_idx_ + imp_->p_mem_size_) % imp_->p_mem_size_ > 0)
		{
			lookAheadOneStep();
			//if (imp_->s3_ > 1.5)
			//	std::cout << "debug" << std::endl;
			//std::cout << "su:" <<imp_->s3_ <<"   " << imp_->u3_ << std::endl;
		}











		return 0;
	}
	auto LookAheadProcessor::setMaxPoss(const double* max_poss, const double* min_poss)->void {
		std::copy(max_poss, max_poss + imp_->input_size_, imp_->max_poss_);
		if (min_poss) {
			std::copy(min_poss, min_poss + imp_->input_size_, imp_->min_poss_);
		}
		else {
			for (int i = 0; i < imp_->input_size_; ++i)
				imp_->min_poss_[i] = -imp_->max_poss_[i];
		}

		// for smooth //
		aris::dynamic::s_vc(imp_->input_size_, imp_->max_poss_, imp_->smooth_max_poss_);
		aris::dynamic::s_vc(imp_->input_size_, imp_->min_poss_, imp_->smooth_min_poss_);
	}
	auto LookAheadProcessor::setMaxVels(const double* max_vels, const double* min_vels)->void {
		std::copy(max_vels, max_vels + imp_->input_size_, imp_->max_vels_);
		if (min_vels) {
			std::copy(min_vels, min_vels + imp_->input_size_, imp_->min_vels_);
		}
		else {
			for (int i = 0; i < imp_->input_size_; ++i)
				imp_->min_vels_[i] = -imp_->max_vels_[i];
		}

		// for smooth //
		aris::dynamic::s_vc(imp_->input_size_, 0.99, imp_->max_vels_, imp_->smooth_max_vels_);
		aris::dynamic::s_vc(imp_->input_size_, 0.99, imp_->min_vels_, imp_->smooth_min_vels_);
	}
	auto LookAheadProcessor::setMaxAccs(const double* max_accs, const double* min_accs)->void {
		std::copy(max_accs, max_accs + imp_->input_size_, imp_->max_accs_);
		if (min_accs) {
			std::copy(min_accs, min_accs + imp_->input_size_, imp_->min_accs_);
		}
		else {
			for (int i = 0; i < imp_->input_size_; ++i)
				imp_->min_accs_[i] = -imp_->max_accs_[i];
		}

		// for smooth //
		aris::dynamic::s_vc(imp_->input_size_, 0.99, imp_->max_accs_, imp_->smooth_max_accs_);
		aris::dynamic::s_vc(imp_->input_size_, 0.99, imp_->min_accs_, imp_->smooth_min_accs_);
	}
	auto LookAheadProcessor::setMaxJerks(const double* max_jerks, const double* min_jerks) -> void {
		std::copy(max_jerks, max_jerks + imp_->input_size_, imp_->max_jerks_);
		if (min_jerks) {
			std::copy(min_jerks, min_jerks + imp_->input_size_, imp_->min_jerks_);
		}
		else {
			for (int i = 0; i < imp_->input_size_; ++i)
				imp_->min_jerks_[i] = -imp_->max_jerks_[i];
		}

		// for smooth //
		aris::dynamic::s_vc(imp_->input_size_, 0.99, imp_->max_jerks_, imp_->smooth_max_jerks_);
		aris::dynamic::s_vc(imp_->input_size_, 0.99, imp_->min_jerks_, imp_->smooth_min_jerks_);
	}
	auto LookAheadProcessor::setMaxVelRatio(double vel_ratio)->void {
		//imp_->max_vel_ratio_ = vel_ratio;
	}
	auto LookAheadProcessor::setMaxAccRatio(double acc_ratio)->void {
		//imp_->max_acc_ratio_ = acc_ratio;
	}
	auto LookAheadProcessor::setModel(aris::dynamic::ModelBase& model)->void {
		imp_->model_ = &model;
		imp_->input_size_ = model.inputPosSize();

		Size mem_size = 0;
		core::allocMem(mem_size, imp_->max_poss_, imp_->input_size_);
		core::allocMem(mem_size, imp_->max_vels_, imp_->input_size_);
		core::allocMem(mem_size, imp_->max_accs_, imp_->input_size_);
		core::allocMem(mem_size, imp_->max_jerks_, imp_->input_size_);
		core::allocMem(mem_size, imp_->min_poss_, imp_->input_size_);
		core::allocMem(mem_size, imp_->min_vels_, imp_->input_size_);
		core::allocMem(mem_size, imp_->min_accs_, imp_->input_size_);
		core::allocMem(mem_size, imp_->min_jerks_, imp_->input_size_);
		core::allocMem(mem_size, imp_->smooth_max_poss_, imp_->input_size_);
		core::allocMem(mem_size, imp_->smooth_max_vels_, imp_->input_size_);
		core::allocMem(mem_size, imp_->smooth_max_accs_, imp_->input_size_);
		core::allocMem(mem_size, imp_->smooth_max_jerks_, imp_->input_size_);
		core::allocMem(mem_size, imp_->smooth_min_poss_, imp_->input_size_);
		core::allocMem(mem_size, imp_->smooth_min_vels_, imp_->input_size_);
		core::allocMem(mem_size, imp_->smooth_min_accs_, imp_->input_size_);
		core::allocMem(mem_size, imp_->smooth_min_jerks_, imp_->input_size_);
		core::allocMem(mem_size, imp_->input_pos_begin_, imp_->input_size_);
		core::allocMem(mem_size, imp_->input_vel_begin_, imp_->input_size_);
		core::allocMem(mem_size, imp_->input_pos_end_, imp_->input_size_);
		core::allocMem(mem_size, imp_->input_vel_end_, imp_->input_size_);
		core::allocMem(mem_size, imp_->input_pos_last_, imp_->input_size_);
		core::allocMem(mem_size, imp_->input_vel_last_, imp_->input_size_);
		core::allocMem(mem_size, imp_->input_pos_this_, imp_->input_size_);
		core::allocMem(mem_size, imp_->input_vel_this_, imp_->input_size_);
		core::allocMem(mem_size, imp_->input_acc_this_, imp_->input_size_);
		core::allocMem(mem_size, imp_->input_acc_ratio_, imp_->input_size_);
		core::allocMem(mem_size, imp_->input_acc_max_consider_ratio_, imp_->input_size_);
		core::allocMem(mem_size, imp_->input_acc_min_consider_ratio_, imp_->input_size_);
		core::allocMem(mem_size, imp_->output_pos_, imp_->input_size_);
		//core::allocMem(mem_size, imp_->curve_params_, imp_->input_size_);
		core::allocMem(mem_size, imp_->p_, imp_->input_size_ * imp_->p_mem_size_);
		core::allocMem(mem_size, imp_->u_, imp_->p_mem_size_);
		core::allocMem(mem_size, imp_->s_, imp_->p_mem_size_);
		core::allocMem(mem_size, imp_->cmd_ids_, imp_->p_mem_size_);
		core::allocMem(mem_size, imp_->Ts_count_, imp_->input_size_ * 3);

		imp_->mem_.resize(mem_size, char(0));

		imp_->max_poss_ = core::getMem(imp_->mem_.data(), imp_->max_poss_);
		imp_->max_vels_ = core::getMem(imp_->mem_.data(), imp_->max_vels_);
		imp_->max_accs_ = core::getMem(imp_->mem_.data(), imp_->max_accs_);
		imp_->max_jerks_ = core::getMem(imp_->mem_.data(), imp_->max_jerks_);
		imp_->min_poss_ = core::getMem(imp_->mem_.data(), imp_->min_poss_);
		imp_->min_vels_ = core::getMem(imp_->mem_.data(), imp_->min_vels_);
		imp_->min_accs_ = core::getMem(imp_->mem_.data(), imp_->min_accs_);
		imp_->min_jerks_ = core::getMem(imp_->mem_.data(), imp_->min_jerks_);
		imp_->smooth_max_poss_ = core::getMem(imp_->mem_.data(), imp_->smooth_max_poss_);
		imp_->smooth_max_vels_ = core::getMem(imp_->mem_.data(), imp_->smooth_max_vels_);
		imp_->smooth_max_accs_ = core::getMem(imp_->mem_.data(), imp_->smooth_max_accs_);
		imp_->smooth_max_jerks_ = core::getMem(imp_->mem_.data(), imp_->smooth_max_jerks_);
		imp_->smooth_min_poss_ = core::getMem(imp_->mem_.data(), imp_->smooth_min_poss_);
		imp_->smooth_min_vels_ = core::getMem(imp_->mem_.data(), imp_->smooth_min_vels_);
		imp_->smooth_min_accs_ = core::getMem(imp_->mem_.data(), imp_->smooth_min_accs_);
		imp_->smooth_min_jerks_ = core::getMem(imp_->mem_.data(), imp_->smooth_min_jerks_);
		imp_->input_pos_begin_ = core::getMem(imp_->mem_.data(), imp_->input_pos_begin_);
		imp_->input_vel_begin_ = core::getMem(imp_->mem_.data(), imp_->input_vel_begin_);
		imp_->input_acc_begin_ = core::getMem(imp_->mem_.data(), imp_->input_acc_begin_);
		imp_->input_pos_end_ = core::getMem(imp_->mem_.data(), imp_->input_pos_end_);
		imp_->input_vel_end_ = core::getMem(imp_->mem_.data(), imp_->input_vel_end_);
		imp_->input_acc_end_ = core::getMem(imp_->mem_.data(), imp_->input_acc_end_);
		imp_->input_pos_last_ = core::getMem(imp_->mem_.data(), imp_->input_pos_last_);
		imp_->input_vel_last_ = core::getMem(imp_->mem_.data(), imp_->input_vel_last_);
		imp_->input_pos_this_ = core::getMem(imp_->mem_.data(), imp_->input_pos_this_);
		imp_->input_vel_this_ = core::getMem(imp_->mem_.data(), imp_->input_vel_this_);
		imp_->input_acc_this_ = core::getMem(imp_->mem_.data(), imp_->input_acc_this_);
		imp_->input_acc_ratio_ = core::getMem(imp_->mem_.data(), imp_->input_acc_ratio_);
		imp_->input_acc_max_consider_ratio_ = core::getMem(imp_->mem_.data(), imp_->input_acc_max_consider_ratio_);
		imp_->input_acc_min_consider_ratio_ = core::getMem(imp_->mem_.data(), imp_->input_acc_min_consider_ratio_);
		imp_->output_pos_ = core::getMem(imp_->mem_.data(), imp_->output_pos_);
		//imp_->curve_params_ = core::getMem(imp_->mem_.data(), imp_->curve_params_);
		imp_->p_ = core::getMem(imp_->mem_.data(), imp_->p_);
		imp_->u_ = core::getMem(imp_->mem_.data(), imp_->u_);
		imp_->s_ = core::getMem(imp_->mem_.data(), imp_->s_);
		imp_->cmd_ids_ = core::getMem(imp_->mem_.data(), imp_->cmd_ids_);
		imp_->Ts_count_ = core::getMem(imp_->mem_.data(), imp_->Ts_count_);

		std::fill_n(imp_->max_poss_, imp_->input_size_, 1e10);
		std::fill_n(imp_->min_poss_, imp_->input_size_, -1e10);
		std::fill_n(imp_->max_vels_, imp_->input_size_, 1.0);
		std::fill_n(imp_->min_vels_, imp_->input_size_, -1.0);
		std::fill_n(imp_->max_accs_, imp_->input_size_, 10.0);
		std::fill_n(imp_->min_accs_, imp_->input_size_, -10.0);
		std::fill_n(imp_->max_jerks_, imp_->input_size_, 1000.0);
		std::fill_n(imp_->min_jerks_, imp_->input_size_, -1000.0);

		aris::dynamic::s_vc(imp_->input_size_, imp_->max_poss_, imp_->smooth_max_poss_);
		aris::dynamic::s_vc(imp_->input_size_, imp_->min_poss_, imp_->smooth_min_poss_);
		aris::dynamic::s_vc(imp_->input_size_, 0.99, imp_->max_vels_, imp_->smooth_max_vels_);
		aris::dynamic::s_vc(imp_->input_size_, 0.99, imp_->min_vels_, imp_->smooth_min_vels_);
		aris::dynamic::s_vc(imp_->input_size_, 0.99, imp_->max_accs_, imp_->smooth_max_accs_);
		aris::dynamic::s_vc(imp_->input_size_, 0.99, imp_->min_accs_, imp_->smooth_min_accs_);
		aris::dynamic::s_vc(imp_->input_size_, 0.99, imp_->max_jerks_, imp_->smooth_max_jerks_);
		aris::dynamic::s_vc(imp_->input_size_, 0.99, imp_->min_jerks_, imp_->smooth_min_jerks_);
	}
	auto LookAheadProcessor::setTrajectoryGenerator(TimeOptimalTrajectoryGenerator& tg)->void {
		imp_->tg_ = &tg;
	}
	auto LookAheadProcessor::init()->void {
		double s_diff = imp_->tg_->dt();

		imp_->p_pop_idx_ = 0;
		for (int i = 0; i < 5; ++i) {
			imp_->s_[i] = i * s_diff * 0.5;
			imp_->u_[i] = i * s_diff;
			imp_->cmd_ids_[i] = imp_->tg_->getEePosByS(imp_->s_[i], imp_->output_pos_);

			imp_->model_->inverseKinematics(imp_->output_pos_, imp_->p_ + i*imp_->input_size_, imp_->model_->inverseRootNumber());
		}
		imp_->p_push_idx_ = 5;

		//imp_->su_series_.clear();
		//imp_->su_series_.push_back({ imp_->s0_, imp_->u0_ });
		//imp_->su_series_.push_back({ imp_->s1_, imp_->u1_ });
		//imp_->su_series_.push_back({ imp_->s2_, imp_->u2_ });
		//imp_->su_series_.push_back({ imp_->s3_, imp_->u3_ });
		//imp_->su_series_.push_back({ imp_->s4_, imp_->u4_ });
		//imp_->su_series_.push_back({ imp_->s5_, imp_->u5_ });
	}
	auto LookAheadProcessor::getNextInput(double* p) -> int {
		auto cmd_id = imp_->cmd_ids_[imp_->p_pop_idx_];
		
		std::copy_n(imp_->p_ + imp_->p_pop_idx_ * imp_->input_size_, imp_->input_size_, p);
		imp_->p_pop_idx_ = (imp_->p_pop_idx_ + 1)%imp_->p_mem_size_;

		
		static int count_ = 0;
		if (count_++ < 3000) {
			std::cout << "count:" << count_ << " s:" << imp_->s_[imp_->p_pop_idx_] << std::endl;
		
		}

		// left planed num: 
		// (imp_->p_push_idx_ - imp_->p_pop_idx_)%imp_->p_mem_size_

		return cmd_id;
	}
	auto LookAheadProcessor::setDs(double ds)->void {
		imp_->ds3_ = imp_->ds2_ = imp_->ds1_ = ds;
	}
	auto LookAheadProcessor::currentDs() -> double {
		return imp_->ds3_;
	}
	auto LookAheadProcessor::setTargetDs(double ds)->void {
		imp_->target_ds_ = ds;
	}
	auto LookAheadProcessor::setModelPosAndMoveDt()->std::int64_t {



		return 0;
	}
	auto LookAheadProcessor::setInverseKinematicMethod(InverseKinematicMethod func)->void {
		imp_->inv_func_ = func;
	}
	LookAheadProcessor::~LookAheadProcessor() = default;
	LookAheadProcessor::LookAheadProcessor() :imp_(new Imp) {
		imp_->param_.nodes_.push_back(LookAheadParam::Node{ 0.0,1.0,0.0,LookAheadParam::NodeType::DEC });
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


	struct TimeOptimalTrajectoryGenerator::Imp {
		// 时间参数 //
		double dt_{ 0.001 };

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

				// 获得需要重新规划的起点
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
				auto scurve_size = aris::dynamic::s_ee_type_vel_dim(ee_types_.size(), ee_types_.data());
				auto replan_ret = replan_nodes((int)scurve_size, ee_types_, std::prev(replan_iter_begin), replan_iter_end, nodes_.end());

				// 查看是否重规划成功，如果规划失败，说明当前的速度过大，融合转弯区后无法减速达到要求。
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
	auto TimeOptimalTrajectoryGenerator::eeTypes()const-> const std::vector<aris::dynamic::EEType>& {
		return imp_->ee_types_;
	}
	auto TimeOptimalTrajectoryGenerator::setEeTypes(const std::vector<aris::dynamic::EEType>& ee_types)->void {
		imp_->ee_types_ = ee_types;
		imp_->outpos_size_ = aris::dynamic::s_ee_type_pos_size(ee_types.size(), ee_types.data());

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
			case aris::dynamic::EEType::RTZ:
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
	auto TimeOptimalTrajectoryGenerator::maxReplanNum()const->int {
		return imp_->max_replan_num_;
	}
	auto TimeOptimalTrajectoryGenerator::setMaxReplanNum(int max_replan_num) -> void {
		imp_->max_replan_num_ = max_replan_num;
	}
	auto TimeOptimalTrajectoryGenerator::dt()const->double {
		return imp_->dt_;
	}
	auto TimeOptimalTrajectoryGenerator::setDt(double dt)->void {
		imp_->dt_ = dt;
	}
	TimeOptimalTrajectoryGenerator::~TimeOptimalTrajectoryGenerator() = default;
	TimeOptimalTrajectoryGenerator::TimeOptimalTrajectoryGenerator() :imp_(new Imp) {
		imp_->current_node_.store(nullptr);
	}
	auto TimeOptimalTrajectoryGenerator::getEePosByS(double s, double* ee_pos, double* ee_vel, double* ee_acc, std::int64_t id)->std::int64_t {
		auto current_node = imp_->current_node_.load();
		auto next_node = current_node->next_node_.load();

		// 需要切换或结束
		while (current_node->s_end_ - s < 0.0 && current_node != next_node && next_node->type_ != Node::NodeType::ResetInitPos) {
			current_node = next_node;
			next_node = current_node->next_node_.load();
		}

		auto real_s = std::min(s, (double)current_node->s_end_);

		get_node_data(eeTypes(), current_node, real_s, 1.0, 0.0, 0.0, imp_->internal_pos_, imp_->internal_vel_, imp_->internal_acc_);
		internal_pos_to_outpos(eeTypes(), imp_->internal_pos_, ee_pos);
		if (ee_acc) {
			aris::dynamic::s_nv(imp_->internal_pos_size, 1.0, imp_->internal_acc_);
			aris::dynamic::s_va(imp_->internal_pos_size, 0.0, imp_->internal_vel_, imp_->internal_acc_);
			aris::dynamic::s_vc(imp_->internal_pos_size, imp_->internal_acc_, ee_acc);
		}
		if (ee_vel) {
			aris::dynamic::s_nv(imp_->internal_pos_size, 1.0, imp_->internal_vel_);
			aris::dynamic::s_vc(imp_->internal_pos_size, imp_->internal_vel_, ee_vel);
		}


		return current_node->s_end_ - s < 0.0 ? 0 : current_node->id_;
	}
	auto clearNodesBefore(std::int64_t id) -> int {
		return 0;
	}

	auto TimeOptimalTrajectoryGenerator::insertInitPos(std::int64_t id, const double* ee_pos)->void {
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
		auto scurve_size = aris::dynamic::s_ee_type_vel_dim(eeTypes().size(), eeTypes().data());
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
	auto TimeOptimalTrajectoryGenerator::insertLinePos(std::int64_t id, const double* ee_pos, const double* vel, const double* acc, const double* jerk, const double* zone)->void{
		std::lock_guard<std::recursive_mutex> lck(imp_->mu_);

		// 如果当前指令队列为空，那么会Z=插入ResetInitPos指令 //
		auto& nodes_ = imp_->nodes_;
		if (nodes_.empty())
			insertInitPos(id, ee_pos);

		imp_->insert_node(Node::NodeType::Line, id, ee_pos, ee_pos, vel, acc, jerk, zone);
	}
	auto TimeOptimalTrajectoryGenerator::insertCirclePos(std::int64_t id, const double* ee_pos, const double* mid_pos, const double* vel, const double* acc, const double* jerk, const double* zone)->void {
		std::lock_guard<std::recursive_mutex> lck(imp_->mu_);

		// 如果当前指令队列为空，那么会Z=插入ResetInitPos指令 //
		auto& nodes_ = imp_->nodes_;
		if (nodes_.empty())
			insertInitPos(id, ee_pos);

		imp_->insert_node(Node::NodeType::Circle, id, ee_pos, mid_pos, vel, acc, jerk, zone);
	
	}
	auto TimeOptimalTrajectoryGenerator::clearUsedPos()->void {
		std::lock_guard<std::recursive_mutex> lck(imp_->mu_);

		auto& nodes_ = imp_->nodes_;

		auto current_node = imp_->current_node_.load();
		auto current_iter = std::find_if(nodes_.begin(), nodes_.end(), [current_node](auto& node)->bool {
			return &node == current_node;
			});

		nodes_.erase(nodes_.begin(), current_iter);
	}
	auto TimeOptimalTrajectoryGenerator::clearAllPos()->void {
		std::lock_guard<std::recursive_mutex> lck(imp_->mu_);
		imp_->current_node_.store(nullptr);
		imp_->nodes_.clear();
	}
	auto TimeOptimalTrajectoryGenerator::unusedPosNum()->int {
		std::lock_guard<std::recursive_mutex> lck(imp_->mu_);
		auto current_node = imp_->current_node_.load();
		auto current_iter = std::find_if(imp_->nodes_.begin(), imp_->nodes_.end(), [current_node](auto& node)->bool {
			return &node == current_node;
			});

		return std::max((int)std::distance(current_iter, imp_->nodes_.end()) - 1, 0);
	}
	auto TimeOptimalTrajectoryGenerator::unusedNodeIds()const->std::vector<std::int64_t> {
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
