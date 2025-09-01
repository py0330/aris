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

		std::vector<char> mem_;
		double * max_poss_,* max_vels_,* max_accs_, * min_poss_, * min_vels_, * min_accs_, 
			* input_poss_,* p1_back_, * p2_back_,* p1_, * p2_, *p3_;

		std::int64_t* node_ids_;

		double dt_{ 1e-3 };
		double s1_{ 0.0 }, s2_{ dt_ };

		std::int64_t tg_idx_{ 0 };// 当前tg运行到的位置
		int look_head_size_{ 2000 }; // 前瞻数据

		double k_{ -9 }, a_{ -1 }, last_a_{ -1 }, T_{ 0.128 };

		auto getInputByS(double s, double* p) -> int {
			auto pool_size = look_head_size_ + 7;// 数据池因为有一个起始数据,以及过去用来插值的3个数据，以及未来3个数据，因此应该比前瞻的数据多 7

			std::int64_t current_idx = std::int64_t(s / dt_) + 1;
			auto s_local_div_dt = std::fmod(s, dt_)/dt_;

			double* p2 = input_poss_ + (std::max(current_idx - 1, std::int64_t(0)) % pool_size) * input_size_;
			double* p3 = input_poss_ + (std::min(current_idx, tg_idx_) % pool_size) * input_size_;

			for (Size i = 0; i < input_size_; ++i) {
				p[i] = p2[i] + (p3[i] - p2[i]) * s_local_div_dt;
			}
			
			return 0;
		}
		auto check_if_ok(const double* p1, const double* p2, const double* p3) -> int {
			// here is condition //
			for (int idx = 0; idx < input_size_; ++idx) {
				double v2 = (p3[idx] - p2[idx]) / dt_;
				double v1 = (p2[idx] - p1[idx]) / dt_;
				double a = (v2 - v1) / dt_;

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
		auto init_pos(const double* init_input_pos) -> void {
			auto pool_size = look_head_size_ + 7;

			// 第一第二个数据应该为起始数据 //
			tg_idx_ = 0;
			aris::dynamic::s_vc(input_size_, init_input_pos, input_poss_);

			tg_idx_ = 1;
			aris::dynamic::s_vc(input_size_, init_input_pos, input_poss_ + (tg_idx_ % pool_size) * input_size_);

			// 前瞻足够的数据 //
			for (int i = 0; i < look_head_size_; ++i) {
				tg_idx_ = tg_idx_ + 1;
				node_ids_[(tg_idx_ % pool_size)] = input_generator_(input_poss_ + (tg_idx_ % pool_size) * input_size_);

				if (node_ids_[(tg_idx_ % pool_size)] == 0)
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
			auto pool_size = look_head_size_ + 7;// 数据池因为有一个起始数据,以及过去用来插值的3个数据，以及未来3个数据，因此应该比前瞻的数据多 7

			// ----------------- PART 1 拿数据--------------------------- //
			{
				getInputByS(s1_, p1_back_);
				getInputByS(s2_, p2_back_);

				// 二分法求解最优的 d3s
				{
					bool if_test_success = false;
					
					auto ds = (s2_ - s1_) / dt_;
					a_ = (k_ * ds) / (std::exp(-T_ * k_) - 1);

					// 重要！ds = 0 时此选项为 0
					a_ = std::min(a_, -1e-6);

					double l = a_ + k_ * ds;
					double r = std::min((1.0 - ds) / dt_, 10.0); // ds 最大不能超过 1

					//for (; mid != last_mid;) {
					for (; std::abs(r - l)>1e-7;) {
						double mid = (l + r) / 2;

						double ds3 = ds + mid * dt_;
						double s3 = s2_ + ds3 * dt_;

						aris::dynamic::s_vc(input_size_, p1_back_, p1_);
						aris::dynamic::s_vc(input_size_, p2_back_, p2_);

						if (test_next_input(s2_, s3, p1_, p2_, p3_)) {
							l = mid;
							last_a_ = a_;
							if_test_success = true;
						}
						else
							r = mid;
					}

					double d2s4 = if_test_success ? l : last_a_ + k_ * ds;
					double ds4 = ds + d2s4 * dt_;
					double s4 = std::max(s2_ + ds4 * dt_, s2_);

					getInputByS(s4, p);
					s1_ = s2_;
					s2_ = s4;
				}
			}

			std::int64_t current_idx = std::int64_t(s2_ / dt_) + 1;
			// ----------------- PART 2 增数据--------------------------- //
			{
				// 如果轨迹已经结束，或已经满
				if (node_ids_[(tg_idx_ % pool_size)] && tg_idx_ - current_idx < look_head_size_) {
					tg_idx_ = tg_idx_ + 1;
					node_ids_[(tg_idx_ % pool_size)] = input_generator_(input_poss_ + (tg_idx_ % pool_size) * input_size_);
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


			return current_idx == tg_idx_ ? 0 : node_ids_[current_idx % pool_size];
		}
	};
	auto InputSmoother::setInputGenerator(InputGenerator generator) -> void {
		imp_->input_generator_ = generator;
	}
	
	auto InputSmoother::allocateMemory(int input_size) -> void {
		imp_->input_size_ = input_size;

		Size mem_size = 0;
		core::allocMem(mem_size, imp_->max_poss_, imp_->input_size_);
		core::allocMem(mem_size, imp_->max_vels_, imp_->input_size_);
		core::allocMem(mem_size, imp_->max_accs_, imp_->input_size_);
		core::allocMem(mem_size, imp_->min_poss_, imp_->input_size_);
		core::allocMem(mem_size, imp_->min_vels_, imp_->input_size_);
		core::allocMem(mem_size, imp_->min_accs_, imp_->input_size_);
		core::allocMem(mem_size, imp_->node_ids_, imp_->look_head_size_ + 7);
		core::allocMem(mem_size, imp_->input_poss_, imp_->input_size_ * (imp_->look_head_size_ + 7));
		core::allocMem(mem_size, imp_->p1_back_, imp_->input_size_);
		core::allocMem(mem_size, imp_->p2_back_, imp_->input_size_);
		core::allocMem(mem_size, imp_->p1_, imp_->input_size_);
		core::allocMem(mem_size, imp_->p2_, imp_->input_size_);
		core::allocMem(mem_size, imp_->p3_, imp_->input_size_);

		imp_->mem_.resize(mem_size, char(0));

		imp_->max_poss_ = core::getMem(imp_->mem_.data(), imp_->max_poss_);
		imp_->max_vels_ = core::getMem(imp_->mem_.data(), imp_->max_vels_);
		imp_->max_accs_ = core::getMem(imp_->mem_.data(), imp_->max_accs_);
		imp_->min_poss_ = core::getMem(imp_->mem_.data(), imp_->min_poss_);
		imp_->min_vels_ = core::getMem(imp_->mem_.data(), imp_->min_vels_);
		imp_->min_accs_ = core::getMem(imp_->mem_.data(), imp_->min_accs_);
		imp_->node_ids_ = core::getMem(imp_->mem_.data(), imp_->node_ids_);
		imp_->input_poss_ = core::getMem(imp_->mem_.data(), imp_->input_poss_);
		imp_->p1_back_ = core::getMem(imp_->mem_.data(), imp_->p1_back_);
		imp_->p2_back_ = core::getMem(imp_->mem_.data(), imp_->p2_back_);
		imp_->p1_ = core::getMem(imp_->mem_.data(), imp_->p1_);
		imp_->p2_ = core::getMem(imp_->mem_.data(), imp_->p2_);
		imp_->p3_ = core::getMem(imp_->mem_.data(), imp_->p3_);

		std::fill_n(imp_->max_poss_, imp_->input_size_, 1e10);
		std::fill_n(imp_->min_poss_, imp_->input_size_, -1e10);
		std::fill_n(imp_->max_vels_, imp_->input_size_, 1.0);
		std::fill_n(imp_->min_vels_, imp_->input_size_, -1.0);
		std::fill_n(imp_->max_accs_, imp_->input_size_, 10.0);
		std::fill_n(imp_->min_accs_, imp_->input_size_, -10.0);
	}
	auto InputSmoother::setPosLimits(const double* max_poss, const double* min_poss) -> void {
		std::copy(max_poss, max_poss + imp_->input_size_, imp_->max_poss_);
		if (min_poss) {
			std::copy(min_poss, min_poss + imp_->input_size_, imp_->min_poss_);
		}
		else {
			for (int i = 0; i < imp_->input_size_; ++i)
				imp_->min_poss_[i] = -imp_->max_poss_[i];
		}
	}
	auto InputSmoother::setVelLimits(const double* max_vels, const double* min_vels) -> void {
		std::copy(max_vels, max_vels + imp_->input_size_, imp_->max_vels_);
		if (min_vels) {
			std::copy(min_vels, min_vels + imp_->input_size_, imp_->min_vels_);
		}
		else {
			for (int i = 0; i < imp_->input_size_; ++i)
				imp_->min_vels_[i] = -imp_->max_vels_[i];
		}
	}
	auto InputSmoother::setAccLimits(const double* max_accs, const double* min_accs) -> void {
		std::copy(max_accs, max_accs + imp_->input_size_, imp_->max_accs_);
		if (min_accs) {
			std::copy(min_accs, min_accs + imp_->input_size_, imp_->min_accs_);
		}
		else {
			for (int i = 0; i < imp_->input_size_; ++i)
				imp_->min_accs_[i] = -imp_->max_accs_[i];
		}
	}
	auto InputSmoother::setBeginInputPos(const double *init_input_pos) -> void {
		imp_->init_pos(init_input_pos);
	}
	auto InputSmoother::getNextInput(double* p) -> int {
		return imp_->getNextInput(p);
	}
	InputSmoother::~InputSmoother() = default;
	InputSmoother::InputSmoother() :imp_(new Imp) {

	}
}
