#include"aris/plan/time_optimal_trajectory.hpp"
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
