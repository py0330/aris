#include"aris/plan/singular_processor.hpp"

namespace aris::plan {
	struct ThirdPolynomialParam {
		double a, b, c, d;
	};
	auto s_third_polynomial_compute_Tmin(double pb, double pe, double vb, double ve, double vmax, double amax)->double {
		// 根据速度算 T 的最小值
		double A1 = (- vb*vb - vb*ve - 3*vmax*vb - ve*ve - 3*vmax*ve);
		double B1 = (6*pe*vb - 6*pb*ve - 6*pb*vb + 6*pe*ve - 6*pb*vmax + 6*pe*vmax);
		double C1 = (-9*pb*pb + 18*pb*pe - 9*pe*pe);

		double A2 = (- vb*vb - vb*ve + 3*vmax*vb - ve*ve + 3*vmax*ve);
		double B2 = (6*pe*vb - 6*pb*ve - 6*pb*vb + 6*pe*ve + 6*pb*vmax - 6*pe*vmax);
		double C2 = (-9*pb*pb + 18*pb*pe - 9*pe*pe);

		double T_min_candidate[4]{ -1,-1,-1,-1 };

		if (B1 * B1 - 4 * A1 * C1 > 0) {
			T_min_candidate[0] = (-B1 - std::sqrt(B1 * B1 - 4 * A1 * C1)) / (2 * A1);
			T_min_candidate[1] = (-B1 + std::sqrt(B1 * B1 - 4 * A1 * C1)) / (2 * A1);
		}
			
		if (B2 * B2 - 4 * A2 * C2 > 0) {
			T_min_candidate[2] = (-B2 - std::sqrt(B2 * B2 - 4 * A2 * C2)) / (2 * A2);
			T_min_candidate[3] = (-B2 + std::sqrt(B2 * B2 - 4 * A2 * C2)) / (2 * A2);
		}
		
		std::sort(T_min_candidate, T_min_candidate + 4);

		double Tmin = 0.0;
		for (int i = 0; i < 4; ++i) {
			double T = T_min_candidate[3 - i];
			double m = (T * (3 * pb - 3 * pe + 2 * T * vb + T * ve)) / (3 * (2 * pb - 2 * pe + T * vb + T * ve));
			if ((m < T && m > 0) || T < 0.0){
				Tmin = T;
				break;
			}
		}

		// 根据加速度算 T 最小值
		double coes[4][3]{
			{-amax, (2 * vb + 4 * ve), 6 * pb - 6 * pe},
			{amax, (2 * vb + 4 * ve), 6 * pb - 6 * pe},
			{-amax, -(4 * vb + 2 * ve), -6 * pb + 6 * pe},
			{amax, -(4 * vb + 2 * ve), -6 * pb + 6 * pe}
		};

		for (int i = 0; i < 4; ++i) {
			double A = coes[i][0];
			double B = coes[i][1];
			double C = coes[i][2];

			if (B * B - 4 * A * C > 0) {
				Tmin = std::max(Tmin, (-B + std::sqrt(B * B - 4 * A * C)) / (2 * A));
				Tmin = std::max(Tmin, (-B - std::sqrt(B * B - 4 * A * C)) / (2 * A));
			}
		}
		return Tmin;
	}
	auto s_third_polynomial_compute_param(double pb, double pe, double vb, double ve, double T)->ThirdPolynomialParam {
		ThirdPolynomialParam param;
		
		param.a = (2 * pb - 2 * pe + T * vb + T * ve) / T / T / T;
		param.b = -(3 * pb - 3 * pe + 2 * T * vb + T * ve) / T / T;
		param.c = vb;
		param.d = pb;

		return param;
	}
	auto s_third_polynomial_compute_value(const ThirdPolynomialParam& param, double t)->double {
		return param.a * t * t * t + param.b * t * t + param.c * t + param.d;
	}

	struct SingularProcessor::Imp {
		aris::Size input_size_{ 0 };
		double dt{ 0.001 };



		std::vector<char> mem_;
		double *max_vels_, 
			   *max_accs_,
			   *input_pos_begin_,
			   *input_vel_begin_,
			   *input_acc_begin_,
			   *input_pos_end_,
			   *input_vel_end_,
			   *input_acc_end_,
			   *input_pos_last_,
			   *input_vel_last_,
			   *input_pos_this_,
			   *input_vel_this_,
			   *input_acc_this_,
			   *output_pos_begin_,
			   *output_pos_end_;

		std::int64_t singular_ret_{ 0 };
		aris::Size total_singular_count_{ 0 }, current_singular_count_{ 0 };
		ThirdPolynomialParam* curve_params_;


		
		double check_rate_{ 0.99 };
		double max_vel_ratio_{ 0.95 };
		double max_acc_ratio_{ 0.9 };
		double current_ds_{ 1.0 };
		aris::dynamic::ModelBase* model_{nullptr};
		aris::plan::TrajectoryGenerator* tg_{nullptr};

		bool is_in_singular{ false }, is_beyond_vel_or_acc_{ false };
	};
	auto SingularProcessor::setMaxVels(const double* max_vels)->void {
		std::copy(max_vels, max_vels + imp_->input_size_, imp_->max_vels_);
	}
	auto SingularProcessor::setMaxAccs(const double* max_accs)->void {
		std::copy(max_accs, max_accs + imp_->input_size_, imp_->max_accs_);
	}
	auto SingularProcessor::setModel(aris::dynamic::ModelBase& model)->void {
		imp_->model_ = &model;
		imp_->input_size_ = model.inputPosSize();

		Size mem_size = 0;
		core::allocMem(mem_size, imp_->max_vels_, imp_->input_size_);
		core::allocMem(mem_size, imp_->max_accs_, imp_->input_size_);
		core::allocMem(mem_size, imp_->input_pos_begin_, imp_->input_size_);
		core::allocMem(mem_size, imp_->input_vel_begin_, imp_->input_size_);
		core::allocMem(mem_size, imp_->input_acc_begin_, imp_->input_size_);
		core::allocMem(mem_size, imp_->input_pos_end_, imp_->input_size_);
		core::allocMem(mem_size, imp_->input_vel_end_, imp_->input_size_);
		core::allocMem(mem_size, imp_->input_acc_end_, imp_->input_size_);
		core::allocMem(mem_size, imp_->input_pos_last_, imp_->input_size_);
		core::allocMem(mem_size, imp_->input_vel_last_, imp_->input_size_);
		core::allocMem(mem_size, imp_->input_pos_this_, imp_->input_size_);
		core::allocMem(mem_size, imp_->input_vel_this_, imp_->input_size_);
		core::allocMem(mem_size, imp_->input_acc_this_, imp_->input_size_);
		core::allocMem(mem_size, imp_->output_pos_begin_, imp_->input_size_);
		core::allocMem(mem_size, imp_->output_pos_end_, imp_->input_size_);
		core::allocMem(mem_size, imp_->curve_params_, imp_->input_size_);
		
		imp_->mem_.resize(mem_size, char(0));

		imp_->max_vels_ = core::getMem(imp_->mem_.data(), imp_->max_vels_);
		imp_->max_accs_ = core::getMem(imp_->mem_.data(), imp_->max_accs_);
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
		imp_->output_pos_begin_ = core::getMem(imp_->mem_.data(), imp_->output_pos_begin_);
		imp_->output_pos_end_ = core::getMem(imp_->mem_.data(), imp_->output_pos_end_);
		imp_->curve_params_ = core::getMem(imp_->mem_.data(), imp_->curve_params_);
	}
	auto SingularProcessor::setTrajectoryGenerator(TrajectoryGenerator& tg)->void {
		imp_->tg_ = &tg;
	}
	auto SingularProcessor::init()->void {
		imp_->model_->getInputPos(imp_->input_pos_this_);
		std::fill_n(imp_->input_vel_this_, imp_->input_size_, 0.0);
		std::fill_n(imp_->input_acc_this_, imp_->input_size_, 0.0);
		std::fill_n(imp_->input_vel_last_, imp_->input_size_, 0.0);
	}
	auto SingularProcessor::setModelPosAndMoveDt()->int {
		static int count{ 0 };
		std::cout <<"count: " << count++ << std::endl;
		if (count > 145)
			std::cout << "debug" << std::endl;
		
		// move tg step //
		auto move_tg_step = [this]()->std::int64_t {
			// 当前处于非奇异状态，正常求反解 //
			auto ret = imp_->tg_->getEePosAndMoveDt(imp_->output_pos_end_);
			imp_->model_->setOutputPos(imp_->output_pos_end_);
			if(imp_->model_->inverseKinematics())
				std::cout << "failed to kinematics" << std::endl;

			// 取出位置
			std::swap(imp_->input_pos_this_, imp_->input_pos_last_);
			std::swap(imp_->input_vel_this_, imp_->input_vel_last_);
			imp_->model_->getInputPos(imp_->input_pos_this_);

			//std::cout << "----------------" << std::endl;
			//aris::dynamic::dsp(1, 6, imp_->output_pos_end_);
			//aris::dynamic::dsp(1, 6, imp_->input_pos_this_);
			

			// 计算速度
			aris::dynamic::s_vc(imp_->input_size_, imp_->input_pos_this_, imp_->input_vel_this_);
			aris::dynamic::s_vs(imp_->input_size_, imp_->input_pos_last_, imp_->input_vel_this_);
			aris::dynamic::s_nv(imp_->input_size_, 1.0 / imp_->dt, imp_->input_vel_this_);

			//aris::dynamic::dsp(1, 6, imp_->input_vel_this_);

			// 计算加速度
			aris::dynamic::s_vc(imp_->input_size_, imp_->input_vel_this_, imp_->input_acc_this_);
			aris::dynamic::s_vs(imp_->input_size_, imp_->input_vel_last_, imp_->input_acc_this_);
			aris::dynamic::s_nv(imp_->input_size_, 1.0 / imp_->dt, imp_->input_acc_this_);

			//aris::dynamic::dsp(1, 6, imp_->input_acc_this_);

			return ret;
		};
		
		// move in singular state
		auto move_in_singular = [this]()->std::int64_t {
			// 取出位置
			std::swap(imp_->input_pos_this_, imp_->input_pos_last_);
			std::swap(imp_->input_vel_this_, imp_->input_vel_last_);
			for (int i = 0; i < imp_->input_size_; ++i) {
				imp_->input_pos_this_[i] = s_third_polynomial_compute_value(imp_->curve_params_[i], imp_->current_singular_count_ * imp_->dt);
			}

			imp_->model_->setInputPos(imp_->input_pos_this_);
			imp_->model_->forwardKinematics();

			//std::cout << "move in singular----------------" << std::endl;
			//aris::dynamic::dsp(1, 6, imp_->output_pos_end_);
			//aris::dynamic::dsp(1, 6, imp_->input_pos_this_);


			// 计算速度
			aris::dynamic::s_vc(imp_->input_size_, imp_->input_pos_this_, imp_->input_vel_this_);
			aris::dynamic::s_vs(imp_->input_size_, imp_->input_pos_last_, imp_->input_vel_this_);
			aris::dynamic::s_nv(imp_->input_size_, 1.0 / imp_->dt, imp_->input_vel_this_);

			//aris::dynamic::dsp(1, 6, imp_->input_vel_this_);

			// 计算加速度
			aris::dynamic::s_vc(imp_->input_size_, imp_->input_vel_this_, imp_->input_acc_this_);
			aris::dynamic::s_vs(imp_->input_size_, imp_->input_vel_last_, imp_->input_acc_this_);
			aris::dynamic::s_nv(imp_->input_size_, 1.0 / imp_->dt, imp_->input_acc_this_);

			//aris::dynamic::dsp(1, 6, imp_->input_acc_this_);

			imp_->current_singular_count_++;
			if (imp_->current_singular_count_ > imp_->total_singular_count_)
				imp_->is_in_singular = false;

			return imp_->singular_ret_;
		};

		// check if singular //
		auto check_if_singular = [](aris::Size input_size, const double *max_vel, const double*max_acc, const double*vel, const double *acc)->int {
			// here is condition //
			int idx = 0;
			for (idx = 0; idx < input_size; ++idx) {
				if (vel[idx] > max_vel[idx] || vel[idx] < -max_vel[idx]) {
					return idx;
				}
			}
			return input_size;
		};

		// 获得最大的速度比或加速度比
		auto get_max_ratio = [](aris::Size input_size, const double* max_value, const double* value)->double {
			double max_ratio = 0.0;
			for (auto idx = 0; idx < input_size; ++idx)
				max_ratio = std::max(max_ratio, std::abs(value[idx]) / max_value[idx]);
			return max_ratio;
		};
		
		
		if (imp_->is_in_singular) {
			// 当前已经处于奇异点 //
			return move_in_singular();
		}
		else {
			auto ret = move_tg_step();
			if (check_if_singular(imp_->input_size_, imp_->max_vels_, imp_->max_accs_, imp_->input_vel_this_, imp_->input_acc_this_) == imp_->input_size_) {
				// 正常情况下，需改变规划器中ds //
				auto vel_ratio = get_max_ratio(imp_->input_size_, imp_->max_vels_, imp_->input_vel_this_);
				auto acc_ratio = get_max_ratio(imp_->input_size_, imp_->max_accs_, imp_->input_acc_this_);
				
				auto ds_ratio = std::max({ 
					vel_ratio / imp_->max_vel_ratio_,
					acc_ratio / imp_->max_acc_ratio_ * acc_ratio / imp_->max_acc_ratio_,
					1.0});

				auto ds_target = std::max(imp_->tg_->currentDs() / ds_ratio, 0.01);

				if (ds_target < 0.9)
					std::cout << "less: ds_target:" << ds_target << "  ds_ratio" << ds_ratio << "  vel_ratio" << vel_ratio << std::endl;

				imp_->tg_->setTargetDs(std::min(ds_target, imp_->current_ds_));

				return ret;
			}
			else {
				std::cout << "singular" << std::endl;
				imp_->is_in_singular = true;
				imp_->singular_ret_ = ret;
				
				// 储存起始时刻 //
				aris::dynamic::s_vc(imp_->input_size_, imp_->input_pos_last_, imp_->input_pos_begin_);
				aris::dynamic::s_vc(imp_->input_size_, imp_->input_vel_last_, imp_->input_vel_begin_);

				// 迭代到下一个非奇异的时刻 //
				int idx;
				do {
					move_tg_step();
					idx = check_if_singular(imp_->input_size_, imp_->max_vels_, imp_->max_accs_, imp_->input_vel_this_, imp_->input_acc_this_);
				} while (idx != imp_->input_size_);

				// 获取终止时刻 //
				aris::dynamic::s_vc(imp_->input_size_, imp_->input_pos_this_, imp_->input_pos_end_);
				aris::dynamic::s_vc(imp_->input_size_, imp_->input_vel_this_, imp_->input_vel_end_);

				// 插补，目前采用三次样条插补 //
				double Tmin = 0.0;
				for (int i = 0; i < imp_->input_size_; ++i) {
					Tmin = std::max(Tmin, s_third_polynomial_compute_Tmin(
						imp_->input_pos_begin_[i],
						imp_->input_pos_end_[i],
						imp_->input_vel_begin_[i],
						imp_->input_vel_end_[i],
						imp_->max_vels_[i],
						imp_->max_accs_[i]
					));
				}

				imp_->total_singular_count_ = (aris::Size)(Tmin/imp_->dt) + 1;
				imp_->current_singular_count_ = 1;

				Tmin = imp_->total_singular_count_ * imp_->dt;
				for (int i = 0; i < imp_->input_size_; ++i) {
					imp_->curve_params_[i] = s_third_polynomial_compute_param(
						imp_->input_pos_begin_[i],
						imp_->input_pos_end_[i],
						imp_->input_vel_begin_[i],
						imp_->input_vel_end_[i],
						Tmin
					);
				}

				// 复原上一时刻
				aris::dynamic::s_vc(imp_->input_size_, imp_->input_pos_begin_, imp_->input_pos_this_);
				aris::dynamic::s_vc(imp_->input_size_, imp_->input_vel_begin_, imp_->input_vel_this_);

				// 移动一步
				return move_in_singular();
			}
		}

		return 0;
	}


	SingularProcessor::~SingularProcessor() = default;
	SingularProcessor::SingularProcessor() :imp_(new Imp) {

	}

	

	struct TrajectoryModelAdapter::Imp {
		std::vector<double> max_vels_, 
			                max_accs_, 
			                input_pos_begin_,
			                input_pos_end_,
							input_pos_last_,
							input_pos_this_,
							input_pos_diff_,
			                output_pos_begin_,
			                output_pos_end_;

		aris::Size input_size_;

		double T;
		double check_rate_{ 0.99 };
		aris::dynamic::ModelBase* model_{nullptr};
		aris::plan::TrajectoryGenerator* tg_{nullptr};

		bool is_in_singular{ false };
	};
	auto TrajectoryModelAdapter::setMaxVels(const std::vector<double> max_vels)->void {
		imp_->max_vels_ = max_vels;
	}
	auto TrajectoryModelAdapter::setMaxAccs(const std::vector<double> max_accs)->void {
		imp_->max_accs_ = max_accs;
	}
	auto TrajectoryModelAdapter::setModel(aris::dynamic::ModelBase& model)->void {
		imp_->model_ = &model;
		imp_->input_pos_begin_.resize(model.inputPosSize());
		imp_->input_pos_end_.resize(model.inputPosSize());
		imp_->output_pos_begin_.resize(model.outputPosSize());
		imp_->output_pos_end_.resize(model.outputPosSize());
		imp_->input_size_ = model.inputPosSize();
	}
	auto TrajectoryModelAdapter::setTrajectoryGenerator(TrajectoryGenerator& tg)->void {
		imp_->tg_ = &tg;
	}
	auto TrajectoryModelAdapter::setModelPosAndMoveDt()->int {
		// check if singular //
		
		



		return 0;
	}
	TrajectoryModelAdapter::~TrajectoryModelAdapter() = default;
	TrajectoryModelAdapter::TrajectoryModelAdapter() :imp_(new Imp) {

	}





}
