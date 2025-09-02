#include"aris/plan/input_smoother.hpp"
#include"aris/plan/function.hpp"

#define ARIS_DEBUG_INPUT_SMOOTHER

namespace aris::plan {
#ifdef ARIS_DEBUG_INPUT_SMOOTHER
	int begin_log = 0;
#endif
	
	struct InputSmoother::Imp {
		InputGenerator input_generator_{ nullptr };
		aris::Size input_size_{ 0 };
		aris::core::Matrix min_pos_mat_, max_pos_mat_, min_vel_mat_, max_vel_mat_, min_acc_mat_, max_acc_mat_;

		std::vector<char> mem_;
		double * max_poss_,* max_vels_,* max_accs_, * min_poss_, * min_vels_, * min_accs_, 
			* input_poss_,* p1_back_, * p2_back_,* p1_, * p2_, *p3_;

		std::int64_t* node_ids_;

		double dt_{ 1e-3 };
		double s1_{ 0.0 }, s2_{ dt_ };

		std::int64_t tg_idx_{ 0 };// 当前tg运行到的位置
		int look_head_size_{ 0 }; // 前瞻数据
		int interpolation_size_{ 4 };
		int pool_size_{ 0 };

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
		double k_{ -9 }, a_{ -1 }, last_a_{ -1 }, T_{ 0.128 };


		auto allocate_mem()->void{
			look_head_size_ = std::ceil(T_ / dt_);
			pool_size_ = look_head_size_ + interpolation_size_;

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
		};
		auto getInputByS(double s, double* p) -> int {
			std::int64_t current_idx = std::int64_t(s / dt_) + 1;
			auto s_local_div_dt = std::fmod(s, dt_)/dt_;

			double* p2 = input_poss_ + ((current_idx - 1) % pool_size_) * input_size_;
			double* p3 = input_poss_ + (std::min(current_idx, tg_idx_) % pool_size_) * input_size_;

			for (Size i = 0; i < input_size_; ++i) {
				p[i] = p2[i] + (p3[i] - p2[i]) * s_local_div_dt;
			}
			
			return 0;
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
		auto test_next_input(double s2, double s3, double *p1, double *p2, double *p3) -> bool {
			getInputByS(s3, p3);

			// 判断是否成功 //
			if ((s3 - s2) <= 0) {
				return true;
			}

			// 检查加速度等 //
			if (check_if_ok(p1, p2, p3) != input_size_) {
				return false;
			}

			double ds = (s3 - s2)/dt_;
			double d2s = a_ + k_ * ds;

			double ds4 = ds + d2s * dt_;
			double s4 = s3 + ds4 * dt_;

			return test_next_input(s3, s4, p2, p3, p1);
		}

		auto test_next_input2(double d2s) -> bool {
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

			//if (!max_vel_mat_.empty())
			//	aris::dynamic::s_vc(input_size_,  max_vel_mat_.data(), max_vels_);
			//if (!min_vel_mat_.empty())
			//	aris::dynamic::s_vc(input_size_,  min_vel_mat_.data(), min_vels_);
			//if (!max_acc_mat_.empty())
			//	aris::dynamic::s_vc(input_size_,  max_acc_mat_.data(), max_accs_);
			//if (!min_acc_mat_.empty())
			//	aris::dynamic::s_vc(input_size_, min_acc_mat_.data(), min_accs_);

			// 第一第二个数据应该为起始数据 //
			tg_idx_ = 0;
			aris::dynamic::s_vc(input_size_, init_input_pos, input_poss_);

			tg_idx_ = 1;
			aris::dynamic::s_vc(input_size_, init_input_pos, input_poss_ + (tg_idx_ % pool_size_) * input_size_);

			// 前瞻足够的数据 //
			for (int i = 0; i < look_head_size_; ++i) {
				tg_idx_ = tg_idx_ + 1;
				node_ids_[(tg_idx_ % pool_size_)] = input_generator_(input_poss_ + (tg_idx_ % pool_size_) * input_size_);

				if (node_ids_[(tg_idx_ % pool_size_)] == 0)
					break;
			}

			// 确保前两个数字有正确的值 //
			node_ids_[0] = node_ids_[2];
			node_ids_[1] = node_ids_[2];

			// 确定正确的ds，并设置到s1_
			s1_ = 1 * dt_;
			s2_ = 1 * dt_;
			aris::dynamic::s_vc(input_size_, init_input_pos, p1_back_);
			aris::dynamic::s_vc(input_size_, init_input_pos, p2_back_);

			double ds_l = 0.0;
			double ds_r = 1.0;
			for (; std::abs(ds_l-ds_r)>1e-10;) {
				double ds = (ds_l + ds_r) / 2;

				a_ = (k_ * ds) / (std::exp(-T_ * k_) - 1);
				a_ = std::min(a_, -1e-6);

				double d2s = a_ + k_ * ds;
				aris::dynamic::s_vc(input_size_, p1_back_, p1_);
				aris::dynamic::s_vc(input_size_, p2_back_, p2_);

				double s3 = s2_ + ds * dt_ + d2s * dt_ * dt_;

				if (test_next_input(s2_, s3, p1_, p2_, p3_)) {
					s1_ = s2_ - ds * dt_;
					ds_l = ds;
				}
				else
					ds_r = ds;
			}
		}
		auto getNextInput(double* p) -> int {
			// ----------------- PART 1 拿数据--------------------------- //
			{
				getInputByS(s1_, p1_back_);
				getInputByS(s2_, p2_back_);

				// 二分法求解最优的 d3s
				{
					bool if_test_success = false;
					
					auto ds = (s2_ - s1_) / dt_;
					double l = last_a_ + k_ * ds;
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
						if (test_next_input2(mid)) {
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
					tg_idx_ = tg_idx_ + 1;
					node_ids_[(tg_idx_ % pool_size_)] = input_generator_(input_poss_ + (tg_idx_ % pool_size_) * input_size_);
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

	auto InputSmoother::setInitInputPos(const double *init_input_pos) -> void {
		imp_->init_pos(init_input_pos);
	}
	auto InputSmoother::getNextInput(double* p) -> int {
		return imp_->getNextInput(p);
	}
	InputSmoother::~InputSmoother() = default;
	InputSmoother::InputSmoother() :imp_(new Imp) {

	}


}
