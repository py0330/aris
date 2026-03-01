#include"aris/plan/input_smoother.hpp"
#include"aris/plan/function.hpp"

//#define ARIS_DEBUG_INPUT_SMOOTHER

namespace aris::plan {
#ifdef ARIS_DEBUG_INPUT_SMOOTHER
	int begin_log = 0;
#endif
	struct InputInterpolator::Imp {
		struct InterpNode {
			double p_;
			double k0_, k1_, k2_, k3_;
		};
		// 线性插值 //
		auto interp_at(const InterpNode* intp, double s_div_dt) -> double {
			return s_div_dt * intp->k1_ + intp->k0_;
		}
		auto make_interp(InterpNode& node0, InterpNode& node1) -> void {
			node1.k1_ = node1.p_ - node0.p_;
			node1.k0_ = node0.p_;
		}

		// 三次插值 //
		auto interp_at4(const InterpNode* intp, double s_div_dt) -> double {
			return s_div_dt * (s_div_dt * (s_div_dt * intp->k3_ + intp->k2_) + intp->k1_) + intp->k0_;
		}
		auto make_interp4(InterpNode& node0, InterpNode& node1, InterpNode& node2, InterpNode& node3) -> void {
			//(2*p1)/dt_^3 - (2*p2)/dt_^3 + v1/dt_^2 + v2/dt_^2
			//(3*p2)/dt_^2 - (3*p1)/dt_^2 - (2*v1)/dt_ - v2/dt_
			//                                               v1
			//                                               p1
			{
				double v1 = (node2.p_ - node0.p_) / 2;
				double v2 = (node3.p_ - node1.p_) / 2;
				double v3 = 0.0;
				double p0 = node0.p_;
				double p1 = node1.p_;
				double p2 = node2.p_;
				double p3 = node3.p_;

				node2.k3_ = 2 * (p1 - p2) + (v1 + v2); // 2*p1 - 2*p2 + p2 - p0 + p3 - p1
				node2.k2_ = 3 * (p2 - p1) - 2 * v1 - v2;//3*p2 - 3*p1 - 2*p1 + 2*p0 - 2*p2 + 2*p1
				node2.k1_ = v1; // p1 - p0
				node2.k0_ = p1;

				node3.k3_ = 2 * (p2 - p3) + (v2 + v3);
				node3.k2_ = 3 * (p3 - p2) - 2 * v2 - v3;
				node3.k1_ = v2;
				node3.k0_ = p2;
			}
		}

		// 三次带权重插值，可解决大部分过穿的问题 //
		auto make_interp_with_ratio(InterpNode& node0, InterpNode& node1, InterpNode& node2, InterpNode& node3) -> void {
			//(2*p1)/dt_^3 - (2*p2)/dt_^3 + v1/dt_^2 + v2/dt_^2
			//(3*p2)/dt_^2 - (3*p1)/dt_^2 - (2*v1)/dt_ - v2/dt_
			//                                               v1
			//                                               p1
			{
				// 根据权重（w），速度绝对值越小的权重越高，因此有：
				// wa = 1/abs(v1a)
				// wb = 1/abs(v1b)
				//
				// v1 = (wa*v1a + wb*v1b)/(wa + wb)	
				//    = (v1a/abs(v1a) + v1b/abs(v1b)) / (1/abs(v1a) + 1/abs(v1b))
				//    = (v1a*abs(v1b) + v1b*abs(v1a)) / (abs(v1a) + abs(v1b))
				//    
				// v1a = p1 - p0
				// v1b = p2 - p1
				// 


				double v1a = (node1.p_ - node0.p_);
				double v1b = (node2.p_ - node1.p_);
				double v1 = (std::abs(v1a) + std::abs(v1b)) > 1e-10
					? (v1a * std::abs(v1b) + v1b * std::abs(v1a)) / (std::abs(v1a) + std::abs(v1b))
					: 0.0;

				double v2a = (node2.p_ - node1.p_);
				double v2b = (node3.p_ - node2.p_);
				double v2 = (std::abs(v2a) + std::abs(v2b)) > 1e-10
					? (v2a * std::abs(v2b) + v2b * std::abs(v2a)) / (std::abs(v2a) + std::abs(v2b))
					: 0.0;

				double v3 = 0.0;
				double p0 = node0.p_;
				double p1 = node1.p_;
				double p2 = node2.p_;
				double p3 = node3.p_;

				node2.k3_ = 2 * (p1 - p2) + (v1 + v2); // 2*p1 - 2*p2 + p2 - p0 + p3 - p1
				node2.k2_ = 3 * (p2 - p1) - 2 * v1 - v2;//3*p2 - 3*p1 - 2*p1 + 2*p0 - 2*p2 + 2*p1
				node2.k1_ = v1; // p1 - p0
				node2.k0_ = p1;

				node3.k3_ = 2 * (p2 - p3) + (v2 + v3);
				node3.k2_ = 3 * (p3 - p2) - 2 * v2 - v3;
				node3.k1_ = v2;
				node3.k0_ = p2;
			}
		}

		// 三次插值用 6 个数据 //
		auto interp_at6(const InterpNode* intp, double s_div_dt) -> double {
			return s_div_dt * (s_div_dt * (s_div_dt * intp->k3_ + intp->k2_) + intp->k1_) + intp->k0_;
		}
		auto make_interp6(InterpNode& node0, InterpNode& node1, InterpNode& node2, InterpNode& node3, InterpNode& node4, InterpNode& node5) -> void {
			//(2*p1)/dt_^3 - (2*p2)/dt_^3 + v1/dt_^2 + v2/dt_^2
			//(3*p2)/dt_^2 - (3*p1)/dt_^2 - (2*v1)/dt_ - v2/dt_
			//                                               v1
			//                                               p1
			{
				
				// dy_at_x2  = (y0 - 8*y1 + 8*y3 - y4)/(12*T);
				// d2y_at_x2 = (-y0 + 16 * y1 - 30 * y2 + 16 * y3 - y4) / (12 * T ^ 2);
				
				double v2 = (node0.p_ - 8 * node1.p_ + 8 * node3.p_ - node4.p_)/12;
				double v3 = (node1.p_ - 8 * node2.p_ + 8 * node4.p_ - node5.p_)/12;
				double v4 = (node2.p_ - 8 * node3.p_ + 8 * node5.p_ - node5.p_)/12;
				double v5 = (node3.p_ - 8 * node4.p_ + 8 * node5.p_ - node5.p_)/12;

				//double a2 = (-node0.p_ + 16 * node1.p_ - 30 * node2.p_ + 16 * node3.p_ - node4.p_) / 12;
				//double a3 = (-node1.p_ + 16 * node2.p_ - 30 * node3.p_ + 16 * node4.p_ - node5.p_) / 12;
				//double a4 = (-node2.p_ + 16 * node3.p_ - 30 * node4.p_ + 16 * node5.p_ - node5.p_) / 12;
				//double a5 = (-node3.p_ + 16 * node4.p_ - 30 * node5.p_ + 16 * node5.p_ - node5.p_) / 12;
				
				
				
				
				
				//double v1 = (node2.p_ - node0.p_) / 2;
				//double v2 = (node3.p_ - node1.p_) / 2;
				//double v3 = 0.0;
				double p0 = node0.p_;
				double p1 = node1.p_;
				double p2 = node2.p_;
				double p3 = node3.p_;
				double p4 = node3.p_;
				double p5 = node3.p_;


				//node2.k3_ = 2 * (p1 - p2) + (v1 + v2); // 2*p1 - 2*p2 + p2 - p0 + p3 - p1
				//node2.k2_ = 3 * (p2 - p1) - 2 * v1 - v2;//3*p2 - 3*p1 - 2*p1 + 2*p0 - 2*p2 + 2*p1
				//node2.k1_ = v1; // p1 - p0
				//node2.k0_ = p1;

				node3.k3_ = 2 * (p2 - p3) + (v2 + v3);
				node3.k2_ = 3 * (p3 - p2) - 2 * v2 - v3;
				node3.k1_ = v2;
				node3.k0_ = p2;

				node4.k3_ = 2 * (p3 - p4) + (v3 + v4);
				node4.k2_ = 3 * (p4 - p3) - 2 * v3 - v4;
				node4.k1_ = v3;
				node4.k0_ = p3;

				node5.k3_ = 2 * (p4 - p5) + (v4 + v5);
				node5.k2_ = 3 * (p5 - p4) - 2 * v4 - v5;
				node5.k1_ = v4;
				node5.k0_ = p4;
			}
		}

		// 三次插值用 6 个数据 //
		auto make_interp6_weno(InterpNode& node0, InterpNode& node1, InterpNode& node2, InterpNode& node3, InterpNode& node4, InterpNode& node5) -> void {
			//(2*p1)/dt_^3 - (2*p2)/dt_^3 + v1/dt_^2 + v2/dt_^2
			//(3*p2)/dt_^2 - (3*p1)/dt_^2 - (2*v1)/dt_ - v2/dt_
			//                                               v1
			//                                               p1
			{
				auto cpt_v = [](double p0, double p1, double p2, double p3, double p4)->double {
					
					// 左侧二次插值、中间二次插值、右边二次插值 //
					// 在权重为 1/6，2/3，1/6 时，三者之和为高次插值 (y0 - 8*y1 + 8*y3 - y4)/(12*T)
					//
					auto v_left = (p0 - 4 * p1 + 3 * p2) / 2;
					auto v_mid = (p3 - p1) / 2;
					auto v_right = (-3 * p2 + 4 * p3 - p4) / 2;

					// 实际权重根据二次曲率确定 //
					auto left_c = (p0 - 2 * p1 + p2);
					auto mid_c = (p1 - 2 * p2 + p3);
					auto right_c = (p2 - 2 * p3 + p4);

					// 计算权重
					auto alpha1 = 1.0 / 6.0 / (std::numeric_limits<double>::epsilon() + std::abs(left_c*left_c));
					auto alpha2 = 2.0 / 3.0 / (std::numeric_limits<double>::epsilon() + std::abs(mid_c*mid_c));
					auto alpha3 = 1.0 / 6.0 / (std::numeric_limits<double>::epsilon() + std::abs(right_c*right_c));

					// 归一化 //
					auto sum_alpha = alpha1 + alpha2 + alpha3;
					auto w1 = alpha1 / sum_alpha;
					auto w3 = alpha3 / sum_alpha;
					auto w2 = 1.0 - w1 - w3;

					// 计算速度
					return w1 * v_left + w2 * v_mid + w3 * v_right;
				};
				
				auto v2 = cpt_v(node0.p_, node1.p_, node2.p_, node3.p_, node4.p_);
				auto v3 = cpt_v(node1.p_, node2.p_, node3.p_, node4.p_, node5.p_);
				auto v4 = cpt_v(node2.p_, node3.p_, node4.p_, node5.p_, node5.p_);
				auto v5 = cpt_v(node3.p_, node4.p_, node5.p_, node5.p_, node5.p_);

				const double p2 = node2.p_;
				const double p3 = node3.p_;
				const double p4 = node4.p_;
				const double p5 = node5.p_;


				//node2.k3_ = 2 * (p1 - p2) + (v1 + v2); // 2*p1 - 2*p2 + p2 - p0 + p3 - p1
				//node2.k2_ = 3 * (p2 - p1) - 2 * v1 - v2;//3*p2 - 3*p1 - 2*p1 + 2*p0 - 2*p2 + 2*p1
				//node2.k1_ = v1; // p1 - p0
				//node2.k0_ = p1;

				node3.k3_ = 2 * (p2 - p3) + (v2 + v3);
				node3.k2_ = 3 * (p3 - p2) - 2 * v2 - v3;
				node3.k1_ = v2;
				node3.k0_ = p2;

				node4.k3_ = 2 * (p3 - p4) + (v3 + v4);
				node4.k2_ = 3 * (p4 - p3) - 2 * v3 - v4;
				node4.k1_ = v3;
				node4.k0_ = p3;

				node5.k3_ = 2 * (p4 - p5) + (v4 + v5);
				node5.k2_ = 3 * (p5 - p4) - 2 * v4 - v5;
				node5.k1_ = v4;
				node5.k0_ = p4;
			}
		}


		/////////////////////////////////////////////////////////////
		InputGenerator input_generator_{ nullptr };
		int input_size_{ 0 };
		double dt_{ 1e-3 };
		int look_head_size_{ 150 }; // 前瞻数据
		int interpolation_size_{ 6 }; // 插值的大小

		/////////////////////////////////////////////////////////////
		std::vector<char> mem_;
		int pool_size_{ 0 }; // = interpolation_size_ + look_head_size_ + 1

		InterpNode* input_poss_{ nullptr };
		std::int64_t* node_ids_{ nullptr };
		double* p3_{ nullptr };

		std::int64_t tg_idx_{ 0 };// 当前tg运行到的位置

		auto allocate_mem() -> void {
			//look_head_size_ = std::ceil(T_ / dt_);
			pool_size_ = look_head_size_ + interpolation_size_ + 1;

			Size mem_size = 0;
			core::allocMem(mem_size, node_ids_, pool_size_);
			core::allocMem(mem_size, input_poss_, input_size_ * pool_size_);
			core::allocMem(mem_size, p3_, input_size_);

			mem_.resize(mem_size, char(0));

			node_ids_ = core::getMem(mem_.data(), node_ids_);
			input_poss_ = core::getMem(mem_.data(), input_poss_);
			p3_ = core::getMem(mem_.data(), p3_);
		};
		auto getInputByS(double s, double* p) -> std::int64_t {
			std::int64_t current_idx = std::min(std::int64_t(s / dt_) + 1, tg_idx_);
			auto s_local_div_dt = std::min(s / dt_ - (current_idx - 1), 1.0);
			auto p3 = input_poss_ + (current_idx % pool_size_) * input_size_;

			//std::int64_t current_idx = std::int64_t(s / dt_) + 1;
			//auto s_local_div_dt = std::fmod(s, dt_)/dt_;
			//auto p3 = input_poss_ + (std::min(current_idx, tg_idx_) % pool_size_) * input_size_;

			for (auto i = decltype(input_size_)(0); i < input_size_; ++i) {
				// 线性插值 //
				//p[i] = interp_at(p3 + i, s_local_div_dt);

				// 三次插值 //
				p[i] = interp_at4(p3 + i, s_local_div_dt);
			}

			return node_ids_[std::min(std::int64_t(s / dt_), tg_idx_) % pool_size_];
		}
		auto insert_nodes() -> std::int64_t {
			tg_idx_ = tg_idx_ + 1;
			node_ids_[(tg_idx_ % pool_size_)] = input_generator_(p3_);

			if (tg_idx_ == 1) {
				node_ids_[0] = node_ids_[1];
			}

			// 三次插值 6 个数据 //
			auto nodes0 = input_poss_ + (std::max(tg_idx_ - 5, std::int64_t(0)) % pool_size_) * input_size_;
			auto nodes1 = input_poss_ + (std::max(tg_idx_ - 4, std::int64_t(0)) % pool_size_) * input_size_;
			auto nodes2 = input_poss_ + (std::max(tg_idx_ - 3, std::int64_t(0)) % pool_size_) * input_size_;
			auto nodes3 = input_poss_ + (std::max(tg_idx_ - 2, std::int64_t(0)) % pool_size_) * input_size_;
			auto nodes4 = input_poss_ + (std::max(tg_idx_ - 1, std::int64_t(0)) % pool_size_) * input_size_;
			auto ins_nodes = input_poss_ + (tg_idx_ % pool_size_) * input_size_;
			
			// 如果报错，此时用上一次的数据 //
			if (node_ids_[(tg_idx_ % pool_size_)] < 0) {
				for (int i = 0; i < input_size_; ++i)
					p3_[i] = nodes4[i].p_;
			}
			
			// 插值 //
			for (int j = 0; j < input_size_; ++j) {
				ins_nodes[j].p_ = p3_[j];
				make_interp6_weno(nodes0[j], nodes1[j], nodes2[j], nodes3[j], nodes4[j], ins_nodes[j]);
				//make_interp6(nodes0[j], nodes1[j], nodes2[j], nodes3[j], nodes4[j], ins_nodes[j]);
				//make_interp4(nodes0[j], nodes1[j], nodes2[j], ins_nodes[j]);
			}

			return node_ids_[(tg_idx_ % pool_size_)];
		}
		auto init_pos(const double* init_input_pos) -> void {
			// 更新起始 Node //
			for (int j = 0; j < input_size_; ++j) {
				input_poss_[j].p_ = init_input_pos[j];
				input_poss_[j].k0_ = input_poss_[j].p_;
				input_poss_[j].k1_ = 0.0;
				input_poss_[j].k2_ = 0.0;
				input_poss_[j].k3_ = 0.0;
			}

			node_ids_[0] = 0;

			// 更新 tg_idx //
			tg_idx_ = 0;
		}
		auto final_ret_code() -> std::int64_t {
			return node_ids_[(tg_idx_ % pool_size_)];
		}
	};
	auto InputInterpolator::interpolationSize() -> aris::Size { return imp_->interpolation_size_; }
	auto InputInterpolator::setInputGenerator(InputGenerator generator) -> void {
		imp_->input_generator_ = generator;
	}
	auto InputInterpolator::setInputSize(int input_size) -> void {
		imp_->input_size_ = input_size;
	}
	auto InputInterpolator::inputSize() -> int {
		return imp_->input_size_;
	}
	auto InputInterpolator::setPoolSize(aris::Size size) -> void {
		imp_->look_head_size_ = static_cast<int>(size);
	}
	auto InputInterpolator::poolSize() -> aris::Size {
		return imp_->look_head_size_;
	}
	auto InputInterpolator::setDt(double dt) -> void {
		imp_->dt_ = dt;
	}
	auto InputInterpolator::dt() -> double {
		return imp_->dt_;
	}
	auto InputInterpolator::allocateMemory() -> void {
		imp_->allocate_mem();
	}
	auto InputInterpolator::init(const double* init_input_pos) -> void {
		imp_->init_pos(init_input_pos);
	}
	auto InputInterpolator::generateInput() -> std::int64_t {
		return imp_->insert_nodes();
	}
	auto InputInterpolator::retCodeAt(double s) -> std::int64_t {
		std::int64_t current_idx = std::int64_t(s / imp_->dt_) + 1;
		return imp_->node_ids_[std::min(current_idx - 1, imp_->tg_idx_) % imp_->pool_size_];
	}
	auto InputInterpolator::getInputAt(double s, double* p) -> std::int64_t {
		return imp_->getInputByS(s, p);
	}
	auto InputInterpolator::finalRetCode() -> std::int64_t {
		return imp_->final_ret_code();
	}
	auto InputInterpolator::finalS() -> double {
		return imp_->tg_idx_ * imp_->dt_;
	}
	InputInterpolator::~InputInterpolator() = default;
	InputInterpolator::InputInterpolator() :imp_(new Imp) {

	}

	struct InputSmoother::Imp {
		InputInterpolator ii_;

		/////////////////////////////////////////////////////////////
		InputGenerator input_generator_{ nullptr };
		int input_size_{ 0 };
		aris::core::Matrix min_pos_mat_, max_pos_mat_, min_vel_mat_, max_vel_mat_, min_acc_mat_, max_acc_mat_;
		double dt_{ 1e-3 };
		int look_head_size_{ 150 }; // 前瞻数据

		/////////////////////////////////////////////////////////////
		std::vector<char> mem_;

		double * max_poss_{ nullptr },* max_vels_{ nullptr },* max_accs_{ nullptr }, 
			* min_poss_{ nullptr }, * min_vels_{ nullptr }, * min_accs_{ nullptr },
			* p1_back_{ nullptr }, * p2_back_{ nullptr },* p1_{ nullptr }, * p2_{ nullptr }, *p3_{ nullptr };

		double ds0_{ 1.0 }, s1_{ 0.0 }, s2_{ dt_ };

		double max_d3s_{ 100.0 };// 只在上升时有效，下降取决于二分法
		double max_d2s_{ 10.0 };// 同上

		//% dy = y*k + a;
		//% 求常微分可得：
		//%  y = -a/k + exp(k*t)*C
		//%
		//% 其中 C 为常数，若已知 y0 则 C = y0 + a/k
		//% y 从 y0 降为0 的时间为：
		//%
		//% T = log(a/(a + k*y0))/k
		//% 上式若已知 k y0 T 求a，则：
		//% a = (k*y0)/(exp(-T*k) - 1)
		//
		//
		// T, k 决定了曲线的走向，a是根据T 和 k计算出来的
		// T是根据 lookaheadCount 计算得出, 但在起始时T可能变化
		double k_{ -9 }, a_{ -1 }, T_{ 0.1 };
		double last_a_{ -1.0 }; // 在递减失败时，会沿着 last_a_ 减速，确保一定成功


		auto allocate_mem()->void{
			ii_.allocateMemory();
			
			Size mem_size = 0;
			core::allocMem(mem_size, max_poss_, input_size_);
			core::allocMem(mem_size, min_poss_, input_size_);
			core::allocMem(mem_size, max_vels_, input_size_);
			core::allocMem(mem_size, min_vels_, input_size_);
			core::allocMem(mem_size, max_accs_, input_size_);
			core::allocMem(mem_size, min_accs_, input_size_);
			core::allocMem(mem_size, p1_, input_size_);
			core::allocMem(mem_size, p2_, input_size_);
			core::allocMem(mem_size, p3_, input_size_);
			core::allocMem(mem_size, p1_back_, input_size_);
			core::allocMem(mem_size, p2_back_, input_size_);

			mem_.resize(mem_size, char(0));

			max_poss_ = core::getMem(mem_.data(), max_poss_);
			min_poss_ = core::getMem(mem_.data(), min_poss_);
			max_vels_ = core::getMem(mem_.data(), max_vels_);
			min_vels_ = core::getMem(mem_.data(), min_vels_);
			max_accs_ = core::getMem(mem_.data(), max_accs_);
			min_accs_ = core::getMem(mem_.data(), min_accs_);
			p1_ = core::getMem(mem_.data(), p1_);
			p2_ = core::getMem(mem_.data(), p2_);
			p3_ = core::getMem(mem_.data(), p3_);
			p1_back_ = core::getMem(mem_.data(), p1_back_);
			p2_back_ = core::getMem(mem_.data(), p2_back_);

			// 设置相关的值 //
			if (!max_pos_mat_.empty())
				aris::dynamic::s_vc(input_size_, max_pos_mat_.data(), max_poss_);
			if (!min_pos_mat_.empty())
				aris::dynamic::s_vc(input_size_, min_pos_mat_.data(), min_poss_);
			if (!max_vel_mat_.empty())
				aris::dynamic::s_vc(input_size_, dt_, max_vel_mat_.data(), max_vels_);
			if (!min_vel_mat_.empty())
				aris::dynamic::s_vc(input_size_, dt_, min_vel_mat_.data(), min_vels_);
			if (!max_acc_mat_.empty())
				aris::dynamic::s_vc(input_size_, dt_ * dt_, max_acc_mat_.data(), max_accs_);
			if (!min_acc_mat_.empty())
				aris::dynamic::s_vc(input_size_, dt_ * dt_, min_acc_mat_.data(), min_accs_);
			
			T_ = look_head_size_ * dt_;
		};
		auto test_next_input(double d2s) -> bool {
			double s2 = s2_;
			double s3 = s2 + (s2_ - s1_) + d2s * dt_ * dt_;
			
			// 下面要用，为避免每次计算，这里预计算
			auto A = a_ * dt_ * dt_;
			auto B = 1.0 + k_ * dt_;

			// p1 存储 p2 - p1 = v1
			// p2 存储 p2
			for (int i = 0; i < input_size_; ++i)
				p1_[i] = p2_[i] - p1_[i];

			for (int i = 0; i < look_head_size_ + 1; ++i) {
				// 判断是否成功 //
				if ((s3 - s2) <= 0) {
					return true;
				}
				
				// 本循环必须多做，不能根据下面返回值判断是否返回，因为要确保结束的时候速度是0 //
				ii_.getInputAt(s3, p3_);
				
				// 判断是否失败，同时更新循环及数据
				// 
				// p1 存储 v1
				// p2 存储 v2
				// p3 不变
				for (int j = 0; j < input_size_; ++j) {
					p2_[j] = p3_[j] - p2_[j];

					// 检查速度及加速度是否超限 //
					double a = (p2_[j] - p1_[j]);
					if (p2_[j] > max_vels_[j] || p2_[j] < min_vels_[j] || a > max_accs_[j] || a < min_accs_[j]) {
						return false;
					}
				}

				// cpt s4（next s3）
				//
				// 
				// ds3 = (s3 - s2) / dt_;
				// d2s = a_ + k_ * ds3;
				//     = a_ + k_ *(s3 - s2) / dt_
				// 
				// ds4 = ds3 + d2s * dt_;
				//     = (s3 - s2) / dt_ + a_ * dt + k_*(s3 - s2);
				// 
				// s4 = s3 + ds4 * dt_;
				//    = s3 + (s3 - s2) + (a_*dt + k*(s3 - s2))*dt
				//    = s3 + a*dt*dt + (1+k*dt)*s_diff
				//
				// 
				double s_diff = (s3 - s2);
				s2 = s3;
				s3 += A + B * s_diff;

				std::swap(p1_, p2_);
				std::swap(p2_, p3_);
			}
			
			THROW_FILE_LINE("should not execute here");
		}
		auto init_pos(const double* init_input_pos) -> void {
			// 设置 s2 //
			s2_ = 0.0;

			// 计算 s1 //
			aris::dynamic::s_vc(input_size_, init_input_pos, p1_back_);
			aris::dynamic::s_vc(input_size_, init_input_pos, p2_back_);

			T_ = 0.0;
			if (ii_.finalRetCode() > 0) {
				T_ = std::min(ii_.finalS() - ii_.interpolationSize() * ii_.dt() - s2_, look_head_size_ * dt_);
			}
			else {
				T_ = std::max({ ii_.finalS() - s2_, T_, dt_ }); // T_ 不能是 0 
			}


			double ds_l{ 0.0 }, ds_r{ 1.0 };
			for (; std::abs(ds_r - ds_l) > 1e-10;) {
				double ds = (ds_l + ds_r) / 2;

				s1_ = s2_ - dt_ * ds;
				
				a_ = (k_ * ds) / (std::exp(-T_ * k_) - 1);
				a_ = std::min(a_, -1e-6);

				double d2s = a_ + k_ * ds;
				aris::dynamic::s_vc(input_size_, p1_back_, p1_);
				aris::dynamic::s_vc(input_size_, p2_back_, p2_);

				if (test_next_input(d2s)) {
					s1_ = s2_ - ds * dt_;
					last_a_ = a_;
					ds_l = ds;
				}
				else
					ds_r = ds;
			}

			ds0_ = (s2_ - s1_) /dt_;
		}
		auto getNextInput(double* p) -> std::int64_t {
			// ----------------- PART 0 Check 是否需要init -------------- //
			{
				// 当前位置已经超过结束位置，或当前位置处返回0，则重新init
				// 有可能当前位置返回 0，但是s2_ 不超 finalS()
				// 例如上次正常结束，重新init时没有新的数据进来，但此时 ii 的 finalS() 为 dt
				if (s2_ >= ii_.finalS() || ii_.retCodeAt(s2_) <= 0) {
					// 初始化到指定位置 //
					ii_.getInputAt(s2_, p3_);
					ii_.init(p3_); // 设置 ret_code 为0//
					
					// 前瞻 4 + interpolate_size 个数据 //
					for (int i = 0; i < 4 + static_cast<int>(ii_.interpolationSize()); ++i) {
						if (ii_.generateInput() == 0)
							break;
					}

					init_pos(p3_);
				}
			}
			
			// ----------------- PART 1 拿数据--------------------------- //
			{
				ii_.getInputAt(s1_, p1_back_);
				ii_.getInputAt(s2_, p2_back_);

				// 二分法求解最优的 d2s
				{
					bool if_test_success = false;
					
					auto ds = (s2_ - s1_) / dt_;
					double l = std::max(last_a_ + k_ * ds, -ds/dt_); // ds 不能小于 0 
					double r = std::min({ (1.0 - ds) / dt_, (ds-ds0_)/dt_ + max_d3s_ * dt_, max_d2s_ }); // ds 最大不能超过 1

					// 考虑到 init 时，ii中的数据是缓慢加进来的，因此必须在到达插值区前就可以降速到0
					// ii 插值区间内的插值函数随着数据的插入可能会改变，因此 T_ 应该小一点
					if (ii_.finalRetCode() > 0) {
						T_ = std::min(ii_.finalS() - ii_.interpolationSize() * ii_.dt() - s2_, look_head_size_ * dt_);
					}
					else {
						T_ = std::max(ii_.finalS() - s2_, T_); // 至少维持上次的值
						T_ = std::min(T_, look_head_size_ * dt_); // 最大不能超过前瞻周期
					}


					//for (; mid != last_mid;) {
					for (; (r - l)>1e-7;) {
						double mid = (l + r) / 2;

						double ds3 = ds + mid * dt_;
						double s3 = s2_ + ds3 * dt_;

						a_ = (k_ * ds3) / (std::exp(-T_ * k_) - 1);
						// 重要！ds = 0 时此选项为 0
						a_ = std::min(a_, -1e-6);

						aris::dynamic::s_vc(input_size_, p1_back_, p1_);
						aris::dynamic::s_vc(input_size_, p2_back_, p2_);

						//if (test_next_input(s2_, s3, p1_, p2_, p3_)) {
						if (test_next_input(mid)) {
							l = mid;
							last_a_ = a_;
							if_test_success = true;
						}
						else
							r = mid;
					}

					double d2s3 = if_test_success ? l : last_a_ + k_ * ds;
					double ds3 = ds + d2s3 * dt_;
					double s3 = std::max(s2_ + ds3 * dt_, s2_);

					s1_ = s2_;
					s2_ = s3;
					ds0_ = (s2_ - s1_)/dt_;
				}
			}
			
			// ----------------- PART 2 增数据--------------------------- //
			{
				// 最多增加4个数据 //
				for (int i = 0; i < 4; ++i) {
					// 如果轨迹已经尚未结束，或已经满
					if (ii_.finalRetCode() && (ii_.finalS() - s2_) < look_head_size_ * dt_) {
						ii_.generateInput();
					}
					else {
						break;
					}
				}
			}


#ifdef ARIS_DEBUG_INPUT_SMOOTHER
			static int count_{ 0 };
			count_++;
			if (count_ > 15000 && count_ < 15002)
				begin_log = 1;
			else
				begin_log = 0;

			if (begin_log) {
			
			}

#endif

			return ii_.getInputAt(s2_, p);
		}
	};
	auto InputSmoother::setInputGenerator(InputGenerator generator) -> void {
		imp_->input_generator_ = generator;
		imp_->ii_.setInputGenerator(generator);
	}
	auto InputSmoother::setInputSize(int input_size) -> void {
		imp_->input_size_ = input_size;
		imp_->ii_.setInputSize(input_size);
	}
	auto InputSmoother::inputSize() -> int {
		return imp_->input_size_;
	}
	auto InputSmoother::setLookAheadCount(int count) -> void {
		imp_->look_head_size_ = count;
		imp_->ii_.setPoolSize(count);
	}
	auto InputSmoother::lookAheadCount() -> int {
		return imp_->look_head_size_;
	}
	auto InputSmoother::setDt(double dt) -> void {
		imp_->dt_ = dt;
		imp_->ii_.setDt(dt);
	}
	auto InputSmoother::dt() -> double {
		return imp_->dt_;
	}
	auto InputSmoother::setMaxPos(aris::core::Matrix pos) -> void {
		imp_->max_pos_mat_ = pos;
	}
	auto InputSmoother::maxPos() -> aris::core::Matrix {
		return imp_->max_pos_mat_;
	}
	auto InputSmoother::setMaxVel(aris::core::Matrix vel) -> void {
		imp_->max_vel_mat_ = vel;
	}
	auto InputSmoother::maxVel() -> aris::core::Matrix {
		return imp_->max_vel_mat_;
	}
	auto InputSmoother::setMaxAcc(aris::core::Matrix acc) -> void {
		imp_->max_acc_mat_ = acc;
	}
	auto InputSmoother::maxAcc() -> aris::core::Matrix {
		return imp_->max_acc_mat_;
	}
	auto InputSmoother::setMinPos(aris::core::Matrix pos) -> void {
		imp_->min_pos_mat_ = pos;
	}
	auto InputSmoother::minPos() -> aris::core::Matrix {
		return imp_->min_pos_mat_;
	}
	auto InputSmoother::setMinVel(aris::core::Matrix vel) -> void {
		imp_->min_vel_mat_ = vel;
	}
	auto InputSmoother::minVel() -> aris::core::Matrix {
		return imp_->min_vel_mat_;
	}
	auto InputSmoother::setMinAcc(aris::core::Matrix acc) -> void {
		imp_->min_acc_mat_ = acc;
	}
	auto InputSmoother::minAcc() -> aris::core::Matrix {
		return imp_->min_acc_mat_;
	}
	auto InputSmoother::allocateMemory() -> void {
		imp_->allocate_mem();
	}
	auto InputSmoother::init(const double *init_input_pos) -> void {
		// 设置 ii //
		imp_->ii_.init(init_input_pos);
	}
	auto InputSmoother::getNextInput(double* p) -> std::int64_t {
		return imp_->getNextInput(p);
	}
	InputSmoother::~InputSmoother() = default;
	InputSmoother::InputSmoother() :imp_(new Imp) {

	}
}
