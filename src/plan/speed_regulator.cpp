#include"aris/plan/speed_regulator.hpp"
#include"aris/plan/function.hpp"


namespace aris::plan {

	struct SpeedRegulator::Imp {

		/////////////////////////////////////////////////////////////
		InputGenerator input_generator_{ nullptr };
		int input_size_{ 0 };
		aris::core::Matrix min_pos_mat_, max_pos_mat_, min_vel_mat_, max_vel_mat_, min_acc_mat_, max_acc_mat_;
		double dt_{ 1e-3 };

		/////////////////////////////////////////////////////////////
		std::vector<char> mem_;
		double* max_poss_, * max_vels_, * max_accs_, * min_poss_, * min_vels_, * min_accs_,
			* p1_, * p2_, * p3_, * pu0_, * pu1_;

		std::int64_t ret1_{ 0 }, ret2_{ 0 }, ret3_{ 0 };

		double u0_{ 0.0 }, u1_{ 1.0 };
		std::atomic<double> target_du_{ 1.0 };

		double max_d2u_{ 0.01 }, min_d2u_{ -0.01 };

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

		auto get_input_by_u(double u, double* p) {
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
			
			if (ret1_ > 0) {
				ret2_ = input_generator_(p2_);
			}
			else {
				ret2_ = 0;
				aris::dynamic::s_vc(input_size_, p1_, p2_);
			}
			if (ret2_ > 0) {
				ret3_ = input_generator_(p3_);
			}
			else {
				ret3_ = 0;
				aris::dynamic::s_vc(input_size_, p2_, p3_);
			}

			u0_ = 1.0;
			u1_ = 1.0 + init_target_ds;

			get_input_by_u(u0_, pu0_);
			get_input_by_u(u1_, pu1_);

			target_du_.store(init_target_ds);
		}
		auto get_next_input(double* p) -> std::int64_t {
			auto target_du = target_du_.load();
			
			// STEP 0 check 是否结束 //
			if (ret1_ == 0) {
				init(target_du);
			}

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

				// 还没结束或还没出错 //
				if (ret2_ > 0) {
					ret3_ = input_generator_(p3_);
				}
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
		return imp_->get_next_input(p);
	}
	SpeedRegulator::~SpeedRegulator() = default;
	SpeedRegulator::SpeedRegulator() :imp_(new Imp) {

	}
}
