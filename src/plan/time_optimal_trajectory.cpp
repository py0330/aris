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
			* p0_,             // 理想的位置值，仅仅在 move_in_tg 中改变
			* p1_,
			* p2_,
			* p3_;

		std::int32_t* Ts_count_;

		std::int64_t tg_ret_{ 0 }, tg_ret_begin_{ 0 };
		aris::Size total_singular_count_{ 0 }, current_singular_count_{ 0 };
		//CurveParam* curve_params_;

		double target_ds_{ 1.0 };
		double ds1_{ 1.0 }, ds2_{ 1.0 }, ds3_{ 1.0 };
		double s0_{ 0.0 }, s1_{ 0.0 }, s2_{ 0.0 }, s3_{ 0.0 };
		double u0_{ 0.0 }, u1_{ 0.0 }, u2_{ 0.0 }, u3_{ 0.0 };

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

		auto test_future_plan_success() -> int {
			double s_diff = tg_->dt();
			
			std::vector<double> p0(input_size_), p1(input_size_), p2(input_size_), p3(input_size_), output_pos(model_->outputPosSize());
			
			std::copy_n(this->p0_, input_size_, p0.data());
			std::copy_n(this->p1_, input_size_, p1.data());
			std::copy_n(this->p2_, input_size_, p2.data());
			std::copy_n(this->p3_, input_size_, p3.data());

			double u0 = u0_;
			double u1 = u1_;
			double u2 = u2_;
			double u3 = u3_;
			double s0 = s0_;
			double s1 = s1_;
			double s2 = s2_;
			double s3 = s3_;

			for (int i = 0;i<1000;++i) {
				std::swap(s0, s1);
				std::swap(s1, s2);
				std::swap(s2, s3);
				s3 = s2 + s_diff;

				std::swap(p0, p1);
				std::swap(p1, p2);
				std::swap(p2, p3);
				tg_->getEePosByS(s3, p3.data());
				model_->inverseKinematics(output_pos_, p3.data(), model_->inverseRootNumber());

				std::swap(u0, u1);
				std::swap(u1, u2);
				std::swap(u2, u3);

				// 准备计算 d3u_l 和 d3u_r
				double d3u_r, d3u_l;
				s_cpt_d3u_lr(input_size_, p0_, p1_, p2_, p3_,
					min_poss_, max_poss_, min_vels_, max_vels_, min_accs_, max_accs_, min_accs_, max_accs_,
					s_diff, u0_, u1_, u2_, d3u_l, d3u_r);
				

				if (d3u_l >= d3u_r)
					return -1;



				auto d3u = d3u_r;

				auto u_diff_1 = u1 - u0;
				auto u_diff_2 = u2 - u1;
				auto du_ds_1 = u_diff_1 / s_diff;
				auto du_ds_2 = u_diff_2 / s_diff;

				auto d2u_ds2_2 = (du_ds_2 - du_ds_1) / s_diff;

				auto d2u_ds2_3 = d2u_ds2_2 + d3u * s_diff;
				auto du_ds_3 = du_ds_2 + d2u_ds2_3 * s_diff;
				u3 = u2 + du_ds_3 * s_diff;


				if ((u3 - u2) / s_diff > 100)
					return 1;

				//std::cout << "u3:" << u3 <<"  u2:" << u2 << "  :" << (u3 - u2) / s_diff << std::endl;
			}

			return 1;
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
		d3s_du3_3_L = std::max({ d3s_du3_3_L, (MIN_DS - ds_du_2 - d2s_du2_2 * u_diff_3) / u_diff_3 / u_diff_3 });
		d3s_du3_3_R = std::min({ d3s_du3_3_R, (MAX_DS - ds_du_2 - d2s_du2_2 * u_diff_3) / u_diff_3 / u_diff_3 });


		//% dx_dt   = 1/dt_dx
		//% d2x_dt2 = -1/(dt_dx)^2 * d2t_dx2 * dx_dt
		//%         = -d2t_dx2 / (dt_dx)^3
		//% d3x_dt3 = (3*(d2t_dx2)^2 - dt_dx * d3t_dx3) / (dt_dx)^5

		auto d2s_du2_25_L = d2s_du2_2 + d3s_du3_3_L * u_diff_2 / 2;
		auto d2s_du2_25_R = d2s_du2_2 + d3s_du3_3_R * u_diff_2 / 2;

		d3u_ds3_3_L = (3 * (d2s_du2_25_R * d2s_du2_25_R) - ds_du_2 * d3s_du3_3_R) / std::pow(ds_du_2, 5);
		d3u_ds3_3_R = (3 * (d2s_du2_25_L * d2s_du2_25_L) - ds_du_2 * d3s_du3_3_L) / std::pow(ds_du_2, 5);
	
	}
	

	auto LookAheadProcessor::lookAheadOneStep() -> int {
		double s_diff = imp_->tg_->dt();
		
		auto& s0 = imp_->s0_;
		auto& s1 = imp_->s1_;
		auto& s2 = imp_->s2_;
		auto& s3 = imp_->s3_;
		auto& u0 = imp_->u0_;
		auto& u1 = imp_->u1_;
		auto& u2 = imp_->u2_;
		auto& u3 = imp_->u3_;
		auto& p0 = imp_->p0_;
		auto& p1 = imp_->p1_;
		auto& p2 = imp_->p2_;
		auto& p3 = imp_->p3_;

		// 更新数据 //
		std::swap(s0, s1);
		std::swap(s1, s2);
		std::swap(s2, s3);
		s3 = s2 + s_diff;

		std::swap(p0, p1);
		std::swap(p1, p2);
		std::swap(p2, p3);
		imp_->tg_->getEePosByS(s3, imp_->output_pos_);
		imp_->model_->inverseKinematics(imp_->output_pos_, imp_->p3_, imp_->model_->inverseRootNumber());

		std::swap(u0, u1);
		std::swap(u1, u2);
		std::swap(u2, u3);

		// 准备计算 d3u_l 和 d3u_r
		double d3u_r, d3u_l;
		s_cpt_d3u_lr(imp_->input_size_, imp_->p0_, imp_->p1_, imp_->p2_, imp_->p3_,
			imp_->min_poss_, imp_->max_poss_, imp_->min_vels_, imp_->max_vels_, imp_->min_accs_, imp_->max_accs_, imp_->min_accs_, imp_->max_accs_,
			s_diff, imp_->u0_, imp_->u1_, imp_->u2_, d3u_l, d3u_r);

		// 加速对应 d3u_l, 减速对应 d3u_r
		auto d3u = d3u_l;

		auto u_diff_1 = imp_->u1_ - imp_->u0_;
		auto u_diff_2 = imp_->u2_ - imp_->u1_;
		auto du_ds_1 = u_diff_1 / s_diff;
		auto du_ds_2 = u_diff_2 / s_diff;

		auto d2u_ds2_2 = (du_ds_2 - du_ds_1) / s_diff;

		auto d2u_ds2_3 = d2u_ds2_2 + d3u * s_diff;
		auto du_ds_3 = du_ds_2 + d2u_ds2_3 * s_diff;
		imp_->u3_ = imp_->u2_ + du_ds_3 * s_diff;

		if (d3u_r >= d3u_l && imp_->test_future_plan_success()) {


		}
		else {
			auto d3u = d3u_r;

			auto u_diff_1 = imp_->u1_ - imp_->u0_;
			auto u_diff_2 = imp_->u2_ - imp_->u1_;
			auto du_ds_1 = u_diff_1 / s_diff;
			auto du_ds_2 = u_diff_2 / s_diff;

			auto d2u_ds2_2 = (du_ds_2 - du_ds_1) / s_diff;

			auto d2u_ds2_3 = d2u_ds2_2 + d3u * s_diff;
			auto du_ds_3 = du_ds_2 + d2u_ds2_3 * s_diff;
			imp_->u3_ = imp_->u2_ + du_ds_3 * s_diff;


		}

		return 0;
	}

	auto LookAheadProcessor::lookAhead(double s_begin) -> int {
		//auto nodes = imp_->param_.nodes_;

		//auto node_beg = std::prev(std::find_if(nodes.begin(), nodes.end(), [s_begin](LookAheadParam::Node &node)->bool {
		//	return node.s_ < s_begin;
		//	}));

		//nodes.erase(std::next(node_beg), nodes.end());

		//auto interval = imp_->tg_->dt();

		//double s_init = s_begin;
		//double ds_du_init = 1.0;
		//double d2s_du2_init = 0.0;












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
		core::allocMem(mem_size, imp_->p0_, imp_->input_size_);
		core::allocMem(mem_size, imp_->p1_, imp_->input_size_);
		core::allocMem(mem_size, imp_->p2_, imp_->input_size_);
		core::allocMem(mem_size, imp_->p3_, imp_->input_size_);
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
		imp_->p0_ = core::getMem(imp_->mem_.data(), imp_->p0_);
		imp_->p1_ = core::getMem(imp_->mem_.data(), imp_->p1_);
		imp_->p2_ = core::getMem(imp_->mem_.data(), imp_->p2_);
		imp_->p3_ = core::getMem(imp_->mem_.data(), imp_->p3_);
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
		//imp_->model_->getInputPos(imp_->input_pos_this_);
		//std::fill_n(imp_->input_vel_this_, imp_->input_size_, 0.0);
		//std::fill_n(imp_->input_acc_this_, imp_->input_size_, 0.0);
		//std::fill_n(imp_->input_vel_last_, imp_->input_size_, 0.0);

		//imp_->model_->getInputPos(imp_->p0_);
		//aris::dynamic::s_vc(imp_->input_size_, imp_->p0_, imp_->p1_);
		//aris::dynamic::s_vc(imp_->input_size_, imp_->p0_, imp_->p2_);
		//aris::dynamic::s_vc(imp_->input_size_, imp_->p0_, imp_->p3_);

		//imp_->ds1_ = imp_->ds2_ = imp_->ds3_ = 1.0;

		//imp_->state_ = Imp::SingularState::NORMAL;




		double s_diff = imp_->tg_->dt();

		imp_->s0_ = 0 * s_diff;
		imp_->s1_ = 1 * s_diff;
		imp_->s2_ = 2 * s_diff;
		imp_->s3_ = 3 * s_diff;

		imp_->tg_->getEePosByS(imp_->s0_, imp_->output_pos_);
		imp_->model_->inverseKinematics(imp_->output_pos_, imp_->p0_, imp_->model_->inverseRootNumber());
		imp_->tg_->getEePosByS(imp_->s1_, imp_->output_pos_);
		imp_->model_->inverseKinematics(imp_->output_pos_, imp_->p1_, imp_->model_->inverseRootNumber());
		imp_->tg_->getEePosByS(imp_->s2_, imp_->output_pos_);
		imp_->model_->inverseKinematics(imp_->output_pos_, imp_->p2_, imp_->model_->inverseRootNumber());
		imp_->tg_->getEePosByS(imp_->s3_, imp_->output_pos_);
		imp_->model_->inverseKinematics(imp_->output_pos_, imp_->p3_, imp_->model_->inverseRootNumber());

		imp_->u0_ = 0 * s_diff;
		imp_->u1_ = 1 * s_diff;
		imp_->u2_ = 2 * s_diff;
		imp_->u3_ = 3 * s_diff;

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







	//auto try_look_ahead()->void {
		
	//}



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
	auto TimeOptimalTrajectoryGenerator::currentDs()const->double {
		return imp_->ds_;
	}
	auto TimeOptimalTrajectoryGenerator::setCurrentDs(double ds)->void {
		imp_->ds_ = ds;
	}
	auto TimeOptimalTrajectoryGenerator::targetDs()const->double {
		return imp_->target_ds_;
	}
	auto TimeOptimalTrajectoryGenerator::setTargetDs(double ds)->void {
		imp_->target_ds_ = ds;
	}
	auto TimeOptimalTrajectoryGenerator::currentDds()const->double {
		return imp_->dds_;
	}
	auto TimeOptimalTrajectoryGenerator::setCurrentDds(double dds)->void {
		imp_->dds_ = dds;
	}
	auto TimeOptimalTrajectoryGenerator::maxDds()const->double {
		return imp_->max_dds_;
	}
	auto TimeOptimalTrajectoryGenerator::setMaxDds(double max_dds)->void {
		imp_->max_dds_ = max_dds;
	}
	auto TimeOptimalTrajectoryGenerator::maxDdds()const->double {
		return imp_->max_ddds_;
	}
	auto TimeOptimalTrajectoryGenerator::setMaxDdds(double max_ddds)->void {
		imp_->max_ddds_ = max_ddds;
	}
	auto TimeOptimalTrajectoryGenerator::leftNodeS()const->double {
		auto current_node = imp_->current_node_.load();
		return current_node->s_end_ - imp_->s_;
	}
	auto TimeOptimalTrajectoryGenerator::leftTotalS()const->double {
		return imp_->nodes_.back().s_end_ - imp_->s_;
	}
	TimeOptimalTrajectoryGenerator::~TimeOptimalTrajectoryGenerator() = default;
	TimeOptimalTrajectoryGenerator::TimeOptimalTrajectoryGenerator() :imp_(new Imp) {
		imp_->current_node_.store(nullptr);
	}
	auto TimeOptimalTrajectoryGenerator::getEePosAndMoveDt(double* ee_pos, double* ee_vel, double* ee_acc)->std::int64_t {
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
	auto TimeOptimalTrajectoryGenerator::getEePosByS(double s, double* ee_pos, double* ee_vel, double* ee_acc, std::int64_t id)->std::int64_t {
		auto current_node = imp_->current_node_.load();
		auto next_node = current_node->next_node_.load();

		// 需要切换或结束
		while (current_node->s_end_ - s < 0.0 && current_node != next_node && next_node->type_ != Node::NodeType::ResetInitPos) {
			current_node = current_node->next_node_.load();
		}

		s = std::min(s, (double)current_node->s_end_);

		get_node_data(eeTypes(), current_node, s, imp_->ds_, imp_->dds_, imp_->ddds_, imp_->internal_pos_, imp_->internal_vel_, imp_->internal_acc_);
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
