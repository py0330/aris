#include"aris/plan/input_smoother.hpp"
#include"aris/plan/function.hpp"

//#define ARIS_DEBUG_INPUT_SMOOTHER

namespace aris::plan {
#ifdef ARIS_DEBUG_INPUT_SMOOTHER
	int begin_log = 0;
#endif
	
	struct InputSmoother::Imp {
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
			return s_div_dt * (s_div_dt * (s_div_dt * intp->k3_ + intp->k2_)+ intp->k1_) + intp->k0_;
		}
		auto make_interp4(InterpNode& node0, InterpNode& node1, InterpNode& node2, InterpNode& node3) -> void {
			//(2*p1)/dt_^3 - (2*p2)/dt_^3 + v1/dt_^2 + v2/dt_^2
			//(3*p2)/dt_^2 - (3*p1)/dt_^2 - (2*v1)/dt_ - v2/dt_
			//                                               v1
			//                                               p1
			{
				double v1 = (node2.p_ - node0.p_)/2;
				double v2 = (node3.p_ - node1.p_)/2;
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
		
		/////////////////////////////////////////////////////////////
		InputGenerator input_generator_{ nullptr };
		int input_size_{ 0 };
		aris::core::Matrix min_pos_mat_, max_pos_mat_, min_vel_mat_, max_vel_mat_, min_acc_mat_, max_acc_mat_;
		double dt_{ 1e-3 };
		int look_head_size_{ 150 }; // 前瞻数据
		int interpolation_size_{ 4 }; // 插值的大小

		/////////////////////////////////////////////////////////////
		std::vector<char> mem_;
		int pool_size_{ 0 }; // = interpolation_size_ + look_head_size_ + 1

		InterpNode* input_poss_;
		double * max_poss_,* max_vels_,* max_accs_, * min_poss_, * min_vels_, * min_accs_, 
			* p1_back_, * p2_back_,* p1_, * p2_, *p3_;
		std::int64_t* node_ids_;

		double s1_{ 0.0 }, s2_{ dt_ };
		std::int64_t tg_idx_{ 0 };// 当前tg运行到的位置

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
		// T, k 决定了曲线的走向，a是根据T 和 k计算出来的, T是根据 lookaheadcount 计算得出
		double k_{ -9 }, a_{ -1 }, last_a_{ -1 }, T_{ 0.1 };


		auto allocate_mem()->void{
			//look_head_size_ = std::ceil(T_ / dt_);
			pool_size_ = look_head_size_ + interpolation_size_ + 1; 

			Size mem_size = 0;
			core::allocMem(mem_size, max_poss_, input_size_);
			core::allocMem(mem_size, max_vels_, input_size_);
			core::allocMem(mem_size, max_accs_, input_size_);
			core::allocMem(mem_size, min_poss_, input_size_);
			core::allocMem(mem_size, min_vels_, input_size_);
			core::allocMem(mem_size, min_accs_, input_size_);
			core::allocMem(mem_size, node_ids_, pool_size_);
			core::allocMem(mem_size, input_poss_, input_size_ * pool_size_);
			core::allocMem(mem_size, p1_back_, input_size_);
			core::allocMem(mem_size, p2_back_, input_size_);
			core::allocMem(mem_size, p1_, input_size_);
			core::allocMem(mem_size, p2_, input_size_);
			core::allocMem(mem_size, p3_, input_size_);

			mem_.resize(mem_size, char(0));

			max_poss_ = core::getMem(mem_.data(), max_poss_);
			max_vels_ = core::getMem(mem_.data(), max_vels_);
			max_accs_ = core::getMem(mem_.data(), max_accs_);
			min_poss_ = core::getMem(mem_.data(), min_poss_);
			min_vels_ = core::getMem(mem_.data(), min_vels_);
			min_accs_ = core::getMem(mem_.data(), min_accs_);
			node_ids_ = core::getMem(mem_.data(), node_ids_);
			input_poss_ = core::getMem(mem_.data(), input_poss_);
			p1_back_ = core::getMem(mem_.data(), p1_back_);
			p2_back_ = core::getMem(mem_.data(), p2_back_);
			p1_ = core::getMem(mem_.data(), p1_);
			p2_ = core::getMem(mem_.data(), p2_);
			p3_ = core::getMem(mem_.data(), p3_);

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
		auto getInputByS(double s, double* p) -> int {
			std::int64_t current_idx = std::int64_t(s / dt_) + 1;
			auto s_local_div_dt = std::fmod(s, dt_)/dt_;

			auto p3 = input_poss_ + (std::min(current_idx, tg_idx_) % pool_size_) * input_size_;

			for (Size i = 0; i < input_size_; ++i) {
				// 线性插值 //
				//p[i] = interp_at(p3 + i, s_local_div_dt);

				// 三次插值 //
				p[i] = interp_at4(p3 + i, s_local_div_dt);
			}
			
			return 0;
		}
		
		auto insert_nodes() -> void {
			tg_idx_ = tg_idx_ + 1;
			node_ids_[(tg_idx_ % pool_size_)] = input_generator_(p3_);

			// 线性插值 //
			//auto nodes0 = input_poss_ + ((tg_idx_-1) % pool_size_) * input_size_;
			//auto ins_nodes = input_poss_ + (tg_idx_ % pool_size_) * input_size_;
			//for (int j = 0; j < input_size_; ++j) {
			//	ins_nodes[j].p_ = p3_[j];
			//	make_interp(nodes0[j], ins_nodes[j]);
			//}

			// 三次插值 //
			auto nodes0 = input_poss_ + ((tg_idx_ - 3) % pool_size_) * input_size_;
			auto nodes1 = input_poss_ + ((tg_idx_ - 2) % pool_size_) * input_size_;
			auto nodes2 = input_poss_ + ((tg_idx_ - 1) % pool_size_) * input_size_;
			auto ins_nodes = input_poss_ + (tg_idx_ % pool_size_) * input_size_;
			for (int j = 0; j < input_size_; ++j) {
				ins_nodes[j].p_ = p3_[j];
				make_interp4(nodes0[j], nodes1[j], nodes2[j], ins_nodes[j]);
			}
		}
		auto check_if_ok(const double* p1, const double* p2, const double* p3) -> int {
			// here is condition //
			for (int idx = 0; idx < input_size_; ++idx) {
				double v2 = (p3[idx] - p2[idx]);
				double v1 = (p2[idx] - p1[idx]);
				double a = (v2 - v1);

				if (v2 > max_vels_[idx] || v2 < min_vels_[idx] || a > max_accs_[idx] || a < min_accs_[idx]) {
					return idx;
				}
			}
			return input_size_;
		};
		auto test_next_input(double d2s) -> bool {
			double s2 = s2_;
			double s3 = s2 + (s2_ - s1_) + d2s * dt_ * dt_;
			
			for (int i = 0; i < look_head_size_ + 1; ++i) {
				getInputByS(s3, p3_);
				
				// 判断是否成功 //
				if ((s3 - s2) <= 0) {
					return true;
				}

				// 检查加速度等 //
				if (check_if_ok(p1_, p2_, p3_) != input_size_) {
					return false;
				}

				double ds = (s3 - s2) / dt_;
				double d2s = a_ + k_ * ds;

				double ds4 = ds + d2s * dt_;
				double s4 = s3 + ds4 * dt_;

				s2 = s3;
				s3 = s4;

				std::swap(p1_, p2_);
				std::swap(p2_, p3_);
			}
			
			return false;
		}
		auto init_pos(const double* init_input_pos) -> void {
			// 确保有足够的起始值 //
			for (int i = 0; i < interpolation_size_; ++i) {
				for (int j = 0; j < input_size_; ++j) {
					input_poss_[i * input_size_ + j].p_ = init_input_pos[j];
				}
			}

			tg_idx_ = interpolation_size_ - 1;

			// 前瞻足够的数据 //
			for (int i = 0; i < look_head_size_; ++i) {
				insert_nodes();
				if (node_ids_[(tg_idx_ % pool_size_)] == 0)
					break;
			}

			// 确定正确的ds，并设置到s1_
			s2_ = (interpolation_size_ - 1) * dt_;
			aris::dynamic::s_vc(input_size_, init_input_pos, p1_back_);
			aris::dynamic::s_vc(input_size_, init_input_pos, p2_back_);

			double ds_l = 0.0;
			double ds_r = 1.0;
			for (; std::abs(ds_l-ds_r)>1e-10;) {
				double ds = (ds_l + ds_r) / 2;

				s1_ = s2_ - dt_ * ds;
				
				a_ = (k_ * ds) / (std::exp(-T_ * k_) - 1);
				a_ = std::min(a_, -1e-6);

				double d2s = a_ + k_ * ds;
				aris::dynamic::s_vc(input_size_, p1_back_, p1_);
				aris::dynamic::s_vc(input_size_, p2_back_, p2_);

				if (test_next_input(d2s)) {
					s1_ = s2_ - ds * dt_;
					ds_l = ds;
				}
				else
					ds_r = ds;
			}
		}
		auto getNextInput(double* p) -> std::int64_t {
			// ----------------- PART 1 拿数据--------------------------- //
			{
				getInputByS(s1_, p1_back_);
				getInputByS(s2_, p2_back_);

				// 二分法求解最优的 d3s
				{
					bool if_test_success = false;
					
					auto ds = (s2_ - s1_) / dt_;
					double l = std::max(last_a_ + k_ * ds, -ds/dt_); // ds 不能小于 0 
					double r = std::min((1.0 - ds) / dt_, 10.0); // ds 最大不能超过 1

					//for (; mid != last_mid;) {
					for (; std::abs(r - l)>1e-7;) {
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

					getInputByS(s3, p);
					s1_ = s2_;
					s2_ = s3;
				}
			}

			std::int64_t current_idx = std::int64_t(s2_ / dt_) + 1;
			// ----------------- PART 2 增数据--------------------------- //
			{
				// 如果轨迹已经结束，或已经满
				if (node_ids_[(tg_idx_ % pool_size_)] && tg_idx_ - current_idx < look_head_size_) {
					insert_nodes();

					//node_ids_[(tg_idx_ % pool_size_)] = input_generator_(input_poss_ + (tg_idx_ % pool_size_) * input_size_);
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


			return current_idx == tg_idx_ ? 0 : node_ids_[current_idx % pool_size_];
		}
	};
	auto InputSmoother::setInputGenerator(InputGenerator generator) -> void {
		imp_->input_generator_ = generator;
	}
	auto InputSmoother::setInputSize(int input_size) -> void {
		imp_->input_size_ = input_size;
	}
	auto InputSmoother::inputSize() -> int {
		return imp_->input_size_;
	}
	auto InputSmoother::setLookAheadCount(int count) -> void {
		imp_->look_head_size_ = count;
	}
	auto InputSmoother::lookAheadCount() -> int {
		return imp_->look_head_size_;
	}
	auto InputSmoother::setDt(double dt) -> void {
		imp_->dt_ = dt;
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
		imp_->init_pos(init_input_pos);
	}
	auto InputSmoother::getNextInput(double* p) -> std::int64_t {
		return imp_->getNextInput(p);
	}
	InputSmoother::~InputSmoother() = default;
	InputSmoother::InputSmoother() :imp_(new Imp) {

	}

	struct SpeedRegulator::Imp {

		/////////////////////////////////////////////////////////////
		InputGenerator input_generator_{ nullptr };
		int input_size_{ 0 };
		aris::core::Matrix min_pos_mat_, max_pos_mat_, min_vel_mat_, max_vel_mat_, min_acc_mat_, max_acc_mat_;
		double dt_{ 1e-3 };

		/////////////////////////////////////////////////////////////
		std::vector<char> mem_;
		double* max_poss_, * max_vels_, * max_accs_, * min_poss_, * min_vels_, * min_accs_,
			* p1_, * p2_, * p3_, *pu0_, *pu1_;

		std::int64_t ret1_{ 0 }, ret2_{ 0 }, ret3_{0};

		double u0_{ 0.0 }, u1_{ 1.0 };
		std::atomic<double> target_du_{ 1.0 };

		double max_d2u_{ 0.01 }, min_d2u_{-0.01};

		auto allocate_mem() -> void {
			Size mem_size = 0;
			core::allocMem(mem_size, max_poss_, input_size_);
			core::allocMem(mem_size, max_vels_, input_size_);
			core::allocMem(mem_size, max_accs_, input_size_);
			core::allocMem(mem_size, min_poss_, input_size_);
			core::allocMem(mem_size, min_vels_, input_size_);
			core::allocMem(mem_size, min_accs_, input_size_);
			core::allocMem(mem_size, p1_, input_size_);
			core::allocMem(mem_size, p2_, input_size_);
			core::allocMem(mem_size, p3_, input_size_);
			core::allocMem(mem_size, pu0_, input_size_);
			core::allocMem(mem_size, pu1_, input_size_);

			mem_.resize(mem_size, char(0));

			max_poss_ = core::getMem(mem_.data(), max_poss_);
			max_vels_ = core::getMem(mem_.data(), max_vels_);
			max_accs_ = core::getMem(mem_.data(), max_accs_);
			min_poss_ = core::getMem(mem_.data(), min_poss_);
			min_vels_ = core::getMem(mem_.data(), min_vels_);
			min_accs_ = core::getMem(mem_.data(), min_accs_);
			p1_ = core::getMem(mem_.data(), p1_);
			p2_ = core::getMem(mem_.data(), p2_);
			p3_ = core::getMem(mem_.data(), p3_);
			pu0_ = core::getMem(mem_.data(), pu0_);
			pu1_ = core::getMem(mem_.data(), pu1_);

			// 设置相关值 //
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

		};
		auto check_if_ok(const double* p1, const double* p2, const double* p3) -> int {
			// here is condition //
			for (int idx = 0; idx < input_size_; ++idx) {
				double v2 = (p3[idx] - p2[idx]);
				double v1 = (p2[idx] - p1[idx]);
				double a = (v2 - v1);

				if (v2 > max_vels_[idx] || v2 < min_vels_[idx] || a > max_accs_[idx] || a < min_accs_[idx]) {
					return idx;
				}
			}
			return input_size_;
		};

		auto get_input_by_u(double u, double *p) {
			if (u < 2.0) {
				double l = 2 - u;
				double r = u - 1;
				for (int i = 0; i < input_size_; ++i) {
					p[i] = p1_[i] * l + p2_[i] * r;
				}
			}
			else {
				double l = 3 - u;
				double r = u - 2;
				for (int i = 0; i < input_size_; ++i) {
					p[i] = p2_[i] * l + p3_[i] * r;
				}
			}

		}

		auto init(double init_target_ds) -> void {
			ret1_ = input_generator_(p1_);
			ret2_ = input_generator_(p2_);
			ret3_ = input_generator_(p3_);

			u0_ = 1.0;
			u1_ = 1.0 + init_target_ds;

			get_input_by_u(u0_, pu0_);
			get_input_by_u(u1_, pu1_);

			target_du_.store(init_target_ds);
		}
		auto get_next_input2(double* p) -> std::int64_t {

			// STEP 1 计算 u2 的范围 //
			double ur{ u1_ + (u1_ - u0_) + max_d2u_ }, ul{ u1_ + (u1_ - u0_) + min_d2u_ }; // du1 = u1 - u0
			{
				//% dp_du   = dp_ds * ds_du = dp_ds / du_ds
				//% 当前 p0 p1 p2
				//% 
				//% pu2 = pu1 + dp_du * du
				//%
				//% 减速时，有以下两种可能：
				//%
				//% CASE1:
				//% p0          p1           p2          p3
				//%                u1    u2
				//%
				//% pu2 = pu1 + dp_du2 *(u2 - u1)
				//%     = k*u2 + b
				//%
				//% where k = dp_du2
				//%       b = pu1 - dp_du2*u1 
				//%
				//% CASE2:
				//%             |---dp_du2---|---dp_du3---|
				//% p0          p1           p2          p3
				//%                   u1           u2
				//%             
				//%
				//%
				//% pu2 = pu1 + dp_du2 *(u_at_p2 - u1) + dp_du3*(u2 - u_at_p2)
				//%     = k*u2 + b
				//%
				//% where k = dp_du3
				//%       b = pu1 + dp_du2 *(u_at_p2 - u1) - dp_du3*u_at_p2
				//% 
				//% ---------------------------------------------
				//%
				//% au2 = ((pu2 - pu1)/dt-(pu1-pu0)/dt)/dt 
				//%     = (pu2 - 2*pu1 + pu0)/dt^2
				//%     = f*u2 + c
				//%
				//% where f = k/dt^2
				//%       c = (b - 2*pu1 + pu0)/dt^2
				//%
				//% a_min - c < f*u2 < a_max - c
				double ul_a = 1.0;
				double ur_a = 3.0;
				double ul_b = 1.0;
				double ur_b = 3.0;

				for (int i = 0; i < input_size_; ++i) {
					double dp_du3 = (p3_[i] - p2_[i]);
					double dp_du2 = (p2_[i] - p1_[i]);

					double u_at_p2 = 2.0;

					// CASE 1 //
					{
						double k = dp_du2;
						double b = pu1_[i] - dp_du2 * u1_;

						double f = k;
						double c = (b - 2 * pu1_[i] + pu0_[i]);

						double left = (min_accs_[i] - c);
						double right = (max_accs_[i] - c);
						if (f < 0) {
							std::swap(left, right);
						}

						if (std::abs(f) > 1e-10) {
							ul_a = std::max(ul_a, left / f);
							ur_a = std::min(ur_a, right / f);
						}
					}

					// CASE 2 //
					{
						double k = dp_du3;
						double b = pu1_[i] + dp_du2 * (u_at_p2 - u1_) - dp_du3 * u_at_p2;

						double f = k;
						double c = (b - 2 * pu1_[i] + pu0_[i]);

						double left = (min_accs_[i] - c);
						double right = (max_accs_[i] - c);
						if (f < 0) {
							std::swap(left, right);
						}

						if (std::abs(f) > 1e-10) {
							ul_b = std::max(ul_b, left / f);
							ur_b = std::min(ur_b, right / f);
						}
					}
				}

				double du1 = u1_ - u0_;
				double d2ul = std::max(min_d2u_, -du1);
				double d2ur = std::min(max_d2u_, 1.0 - du1);

				if (ur_a >= ul_a && ur_a > 2.0) { // 2.0 可行 
					ur = std::min(ur_b, ur);
					ul = std::max(ul_a, ul);
				}
				else if (ur_a > ul_a) {
					ur = std::min(ur_a, ur);
					ul = std::max(ul_a, ul);
				}
				else {
					ur = std::min(ur_b, ur);
					ul = std::max(ul_b, ul);
				}
			}

			// STEP 2 计算 u2 //
			auto target_du = target_du_.load();
			double u2;
			if (target_du > (ur - u1_)) {
				u2 = ur;
			}
			else if (target_du < (ul - u1_)) {
				u2 = ul;
			}
			else {
				u2 = u1_ + target_du;
			}
			
			// STEP 4 更新数据
			if (u2 > 2.0) {
				std::swap(p1_, p2_);
				std::swap(p2_, p3_);
				ret1_ = ret2_;
				ret2_ = ret3_;

				if(ret2_)
					ret3_ = input_generator_(p3_);
				else {
					aris::dynamic::s_vc(input_size_, p2_, p3_);
				}
					

				u1_ -= 1.0;
				u2 -= 1.0;
			}

			u0_ = u1_;
			u1_ = u2;
			std::swap(pu0_, pu1_);
			get_input_by_u(u1_, pu1_);
			aris::dynamic::s_vc(input_size_, pu1_, p);

			// 返回值应该为上一个值
			return ret1_;
		}
	};
	auto SpeedRegulator::setInputGenerator(InputGenerator generator) -> void {
		imp_->input_generator_ = generator;
	}
	auto SpeedRegulator::setInputSize(int input_size) -> void {
		imp_->input_size_ = input_size;
	}
	auto SpeedRegulator::inputSize() -> int {
		return imp_->input_size_;
	}
	auto SpeedRegulator::setDt(double dt) -> void {
		imp_->dt_ = dt;
	}
	auto SpeedRegulator::dt() -> double {
		return imp_->dt_;
	}
	auto SpeedRegulator::setMaxPos(aris::core::Matrix pos) -> void {
		imp_->max_pos_mat_ = pos;
	}
	auto SpeedRegulator::maxPos() -> aris::core::Matrix {
		return imp_->max_pos_mat_;
	}
	auto SpeedRegulator::setMaxVel(aris::core::Matrix vel) -> void {
		imp_->max_vel_mat_ = vel;
	}
	auto SpeedRegulator::maxVel() -> aris::core::Matrix {
		return imp_->max_vel_mat_;
	}
	auto SpeedRegulator::setMaxAcc(aris::core::Matrix acc) -> void {
		imp_->max_acc_mat_ = acc;
	}
	auto SpeedRegulator::maxAcc() -> aris::core::Matrix {
		return imp_->max_acc_mat_;
	}
	auto SpeedRegulator::setMinPos(aris::core::Matrix pos) -> void {
		imp_->min_pos_mat_ = pos;
	}
	auto SpeedRegulator::minPos() -> aris::core::Matrix {
		return imp_->min_pos_mat_;
	}
	auto SpeedRegulator::setMinVel(aris::core::Matrix vel) -> void {
		imp_->min_vel_mat_ = vel;
	}
	auto SpeedRegulator::minVel() -> aris::core::Matrix {
		return imp_->min_vel_mat_;
	}
	auto SpeedRegulator::setMinAcc(aris::core::Matrix acc) -> void {
		imp_->min_acc_mat_ = acc;
	}
	auto SpeedRegulator::minAcc() -> aris::core::Matrix {
		return imp_->min_acc_mat_;
	}

	auto SpeedRegulator::allocateMemory() -> void {
		imp_->allocate_mem();
	}
	auto SpeedRegulator::init(double init_target_ds) -> void {
		imp_->init(init_target_ds);
	}

	auto SpeedRegulator::setTargetSpeedRatio(double du) -> void {
		imp_->target_du_.store(du);
	}
	auto SpeedRegulator::targetSpeedRatio() -> double {
		return imp_->target_du_.load();
	}
	auto SpeedRegulator::actualSpeedRatio() -> double {
		return (imp_->u1_ - imp_->u0_) / imp_->dt_;
	}
	auto SpeedRegulator::getNextInput(double* p) -> std::int64_t {
		return imp_->get_next_input2(p);
	}

	SpeedRegulator::~SpeedRegulator() = default;
	SpeedRegulator::SpeedRegulator() :imp_(new Imp) {

	}
}
