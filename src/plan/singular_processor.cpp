#include"aris/plan/singular_processor.hpp"

//#define ARIS_DEBUG_SINGULAR_PROCESSOR

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
			if ((0 < m && m < T) || T < 0.0){
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

#ifdef ARIS_DEBUG_SINGULAR_PROCESSOR
				std::cout << "Tmin1:" << (-B + std::sqrt(B * B - 4 * A * C)) / (2 * A) << std::endl;
				std::cout << "Tmin1:" << (-B - std::sqrt(B * B - 4 * A * C)) / (2 * A) << std::endl;
#endif
				Tmin = std::max(Tmin, (-B + std::sqrt(B * B - 4 * A * C)) / (2 * A));
				Tmin = std::max(Tmin, (-B - std::sqrt(B * B - 4 * A * C)) / (2 * A));
			}
		}
#ifdef ARIS_DEBUG_SINGULAR_PROCESSOR
		std::cout << "Tmin:" << Tmin << std::endl;
#endif
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

	struct TcurveParam{
		double pb, pe, vb, ve, vmax, amax;
		double T, Ta, Tb, v, a;
		int mode;
	};
	// 计算可行的时间域
	// (T1 T2), (T3,inf)
	auto s_tcurve_T_range(const TcurveParam &param, double &T1, double &T2, double &T3)->void {
		// 
		const double pb = param.pb;
		const double pe = param.pe;
		const double vb = param.vb;
		const double ve = param.ve;
		const double vmax = param.vmax;
		const double amax = param.amax;
		
		// 
		const double pt = pe - pb;
		const double Tb2e = std::abs(vb - ve) / amax;
		const double pb2e = (vb + ve) / 2 * Tb2e;

		double v;
		if (pb2e < pt) {
			if ((vmax + vb) / 2 * (vmax - vb) / amax + (vmax + ve) / 2 * (vmax - ve) / amax < pt){
				v = vmax;
				double Ta = (v - vb) / amax;
				double Tb = (v - ve) / amax;
				T1 = (pt - Ta * (vb + v) / 2 - Tb * (ve + v) / 2) / v + Ta + Tb;
			}
			else {
				v = std::sqrt(std::abs(pt * amax + (vb*vb + ve*ve)/2));
				T1 = (v - vb) / amax + (v - ve) / amax;
			}
		}
		else {
			if ((-vmax + vb) / 2 * (vb + vmax) / amax + (-vmax + ve) / 2 * (ve + vmax) / amax > pt){
				v = -vmax;
				double Ta = (vb - v) / amax;
				double Tb = (ve - v) / amax;
				T1 = (pt - Ta * (vb + v) / 2 - Tb * (ve + v) / 2) / v + Ta + Tb;
			}
			else {
				v = -std::sqrt(std::abs(-pt*amax + (vb*vb + ve*ve)/2));
				T1 = (vb - v) / amax + (ve - v) / amax;
			}
		}

		T2 = T1;
		T3 = T1;
		if (v > 0 && vb > 0 && ve > 0) {
			if (vb * vb / 2 / amax + ve * ve / 2 / amax > pt) {
				v = std::sqrt(std::abs(-pt * amax + (vb * vb + ve * ve) / 2));
				T2 = std::abs(vb - v) / amax + std::abs(ve - v) / amax;
				T3 = std::abs(vb + v) / amax + std::abs(ve + v) / amax;
			}
		}
		else if(v < 0 && vb < 0 && ve < 0) {
			if (-vb * vb / 2 / amax - ve * ve / 2 / amax < pt) {
				v = -std::sqrt(std::abs(pt * amax + (vb * vb + ve * ve) / 2));
				T2 = std::abs(vb - v) / amax + std::abs(ve - v) / amax;
				T3 = std::abs(vb + v) / amax + std::abs(ve + v) / amax;
			}
		}
	}
	// 计算param
	auto s_tcurve_param(TcurveParam &param)->void {
		const auto pb = param.pb;
		const auto pe = param.pe;
		const auto vb = param.vb;
		const auto ve = param.ve;
		const auto vmax = param.vmax;
		const auto amax = param.amax;
		const auto T = param.T;

		auto& a = param.a;
		auto& v = param.v;
		auto& Ta = param.Ta;
		auto& Tb = param.Tb;
		auto& mode = param.mode;

		double pt = pe - pb;
		double Tb2e = std::abs(vb - ve) / amax;
		double pb2e = (vb + ve) / 2 * Tb2e;


		if ((pt - pb2e) > (T - Tb2e) * std::max(vb, ve)) {
			param.mode = 0;
			param.a = amax;

			double A= 1;
			double B = -T * a - vb - ve;
			double C = (vb * vb + ve * ve) / 2 + pt * a;

			param.v = (-B - std::sqrt(std::abs(B * B - 4 * A * C))) / (2 * A);
			param.Ta = (v - vb) / a;
			param.Tb = (v - ve) / a;
		}
			
		else if ((pt - pb2e) > (T - Tb2e) * std::min(vb, ve)) {
			param.mode = 1;
			param.v = (pt - pb2e) / (T - Tb2e);
			Ta = (pt - pb2e) / (vb - ve) - ve / (vb - ve) * (T - Tb2e);
			Ta = std::max(0.0, Ta);
			param.Ta = std::min(Ta, T - Tb2e);
			param.Tb = T - Tb2e - Ta;
			param.a = aris::dynamic::s_sgn2(ve - vb) * amax;
		}

		else {
			param.mode = 0;
			param.a = -amax;
			double A = 1;
			double B = -T * a - vb - ve;
			double C = (vb * vb + ve * ve) / 2 + pt * a;

			param.v = (-B + sqrt(std::abs(B * B - 4 * A * C))) / (2 * A);
			param.Ta = (v - vb) / a;
			param.Tb = (v - ve) / a;
		}
	}
	// 计算值
	auto s_tcurve_value(const TcurveParam& param, double t)->double {
		const auto pb = param.pb;
		const auto pe = param.pe;
		const auto vb = param.vb;
		const auto ve = param.ve;
		const auto vmax = param.vmax;
		const auto amax = param.amax;
		const auto T = param.T;

		const auto a = param.a;
		const auto v = param.v;
		const auto Ta = param.Ta;
		const auto Tb = param.Tb;
		const auto mode = param.mode;


		if (mode == 0)
			if (t < Ta)
				return pb + vb * t + a*t*t / 2;
			else if(t < T - Tb)
				return pb + vb * Ta + a * Ta * Ta / 2 + v * (t - Ta);
			else
				return pe - ve * (T - t) - a * (T - t)*(T - t) / 2;
		else
			if (t < Ta)
				return pb + vb * t;
			else if(t < T - Tb)
				return pb + vb * t + a * (t - Ta)*(t - Ta) / 2;
			else
				return  pe - ve * (T - t);
	}

	struct SingularProcessor::Imp {
		using CurveParam = TcurveParam;

		aris::Size input_size_{ 0 };
		double dt{ 0.001 };



		std::vector<char> mem_;
		double *max_vels_, 
			   *max_accs_,
			   *input_pos_begin_,
			   *input_vel_begin_,
			   *input_pos_end_,
			   *input_vel_end_,
			   *input_pos_last_,
			   *input_vel_last_,
			   *input_pos_this_,
			   *input_vel_this_,
			   *input_acc_this_,
			   *input_acc_ratio_,
			   *output_pos_;

		std::int64_t singular_ret_{ 0 };
		aris::Size total_singular_count_{ 0 }, current_singular_count_{ 0 };
		CurveParam* curve_params_;


		
		double check_rate_{ 0.99 };
		double max_vel_ratio_{ 0.95 };
		double max_acc_ratio_{ 0.9 };
		double current_ds_{ 1.0 }, last_dds{0.0};
		aris::dynamic::ModelBase* model_{nullptr};
		aris::plan::TrajectoryGenerator* tg_{nullptr};

		// 是否已经处于奇异状态，或者是否还在继续准备处理奇异情况 //
		int singular_idx;

		enum class SingularState {
			SINGULAR,
			SINGULAR_PREPARE,
			NORMAL
		};

		SingularState state_{ SingularState::NORMAL };

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
		core::allocMem(mem_size, imp_->input_pos_end_, imp_->input_size_);
		core::allocMem(mem_size, imp_->input_vel_end_, imp_->input_size_);
		core::allocMem(mem_size, imp_->input_pos_last_, imp_->input_size_);
		core::allocMem(mem_size, imp_->input_vel_last_, imp_->input_size_);
		core::allocMem(mem_size, imp_->input_pos_this_, imp_->input_size_);
		core::allocMem(mem_size, imp_->input_vel_this_, imp_->input_size_);
		core::allocMem(mem_size, imp_->input_acc_this_, imp_->input_size_);
		core::allocMem(mem_size, imp_->input_acc_ratio_, imp_->input_size_);
		core::allocMem(mem_size, imp_->output_pos_, imp_->input_size_);
		core::allocMem(mem_size, imp_->curve_params_, imp_->input_size_);
		
		imp_->mem_.resize(mem_size, char(0));

		imp_->max_vels_ = core::getMem(imp_->mem_.data(), imp_->max_vels_);
		imp_->max_accs_ = core::getMem(imp_->mem_.data(), imp_->max_accs_);
		imp_->input_pos_begin_ = core::getMem(imp_->mem_.data(), imp_->input_pos_begin_);
		imp_->input_vel_begin_ = core::getMem(imp_->mem_.data(), imp_->input_vel_begin_);
		imp_->input_pos_end_ = core::getMem(imp_->mem_.data(), imp_->input_pos_end_);
		imp_->input_vel_end_ = core::getMem(imp_->mem_.data(), imp_->input_vel_end_);
		imp_->input_pos_last_ = core::getMem(imp_->mem_.data(), imp_->input_pos_last_);
		imp_->input_vel_last_ = core::getMem(imp_->mem_.data(), imp_->input_vel_last_);
		imp_->input_pos_this_ = core::getMem(imp_->mem_.data(), imp_->input_pos_this_);
		imp_->input_vel_this_ = core::getMem(imp_->mem_.data(), imp_->input_vel_this_);
		imp_->input_acc_this_ = core::getMem(imp_->mem_.data(), imp_->input_acc_this_);
		imp_->input_acc_ratio_ = core::getMem(imp_->mem_.data(), imp_->input_acc_ratio_);
		imp_->output_pos_ = core::getMem(imp_->mem_.data(), imp_->output_pos_);
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
	auto SingularProcessor::setDs(double ds)->void {
		imp_->current_ds_ = ds;
	}
	auto SingularProcessor::setModelPosAndMoveDt()->std::int64_t {
		
#ifdef ARIS_DEBUG_SINGULAR_PROCESSOR
		static int count{ 0 };
		count++;
		if(count % 1000 == 0)
			std::cout <<"count: " << count++ << std::endl;
		if (count== 125)
			std::cout << "debug" << std::endl;
#endif

		// 获得最大的速度比或加速度比
		auto get_max_ratio = [](aris::Size input_size, const double* max_value, const double* value)->double {
			double max_ratio = 0.0;
			for (auto idx = 0; idx < input_size; ++idx)
				max_ratio = std::max(max_ratio, std::abs(value[idx]) / max_value[idx]);
			return max_ratio;
		};
		
		// move tg step //
		auto move_tg_step = [this,get_max_ratio]()->std::int64_t {
			// 当前处于非奇异状态，正常求反解 //
			auto ret = imp_->tg_->getEePosAndMoveDt(imp_->output_pos_);
			imp_->model_->setOutputPos(imp_->output_pos_);
			if(imp_->model_->inverseKinematics())
				std::cout << "failed to kinematics" << std::endl;

			auto tg_ds = imp_->tg_->currentDs();

			// 取出位置
			std::swap(imp_->input_pos_this_, imp_->input_pos_last_);
			std::swap(imp_->input_vel_this_, imp_->input_vel_last_);
			imp_->model_->getInputPos(imp_->input_pos_this_);

			// 计算速度
			aris::dynamic::s_vc(imp_->input_size_, imp_->input_pos_this_, imp_->input_vel_this_);
			aris::dynamic::s_vs(imp_->input_size_, imp_->input_pos_last_, imp_->input_vel_this_);
			aris::dynamic::s_nv(imp_->input_size_, 1.0 / imp_->dt, imp_->input_vel_this_);

			// 计算加速度
			aris::dynamic::s_vc(imp_->input_size_, imp_->input_vel_this_, imp_->input_acc_this_);
			aris::dynamic::s_vs(imp_->input_size_, imp_->input_vel_last_, imp_->input_acc_this_);
			aris::dynamic::s_nv(imp_->input_size_, 1.0 / imp_->dt, imp_->input_acc_this_);

			// 计算去除 dds 项所影响的加速度
			// d2p = d2p_ds2 * ds^2 + dp_ds * d2s
			aris::dynamic::s_vc(imp_->input_size_, imp_->input_acc_this_, imp_->input_acc_ratio_);
			aris::dynamic::s_va(imp_->input_size_, -imp_->last_dds/tg_ds, imp_->input_vel_this_, imp_->input_acc_ratio_);

			// 正常情况下，可能需改变规划器中ds //
			auto vel_ratio = get_max_ratio(imp_->input_size_, imp_->max_vels_, imp_->input_vel_this_);
			auto acc_ratio = get_max_ratio(imp_->input_size_, imp_->max_accs_, imp_->input_acc_ratio_);

			// 判断是否要改 ds_ratio //
			auto ds_ratio = std::max({
				vel_ratio / imp_->max_vel_ratio_,
				std::sqrt(acc_ratio / imp_->max_acc_ratio_)
				});

			if (ds_ratio > 1.0) {
				auto ds = tg_ds / ds_ratio;
				ds = std::max(0.01, ds);
				imp_->tg_->setCurrentDs(ds);
				imp_->tg_->setCurrentDds(0.0);
				imp_->tg_->setTargetDs(ds);
				imp_->last_dds = (ds - tg_ds)/imp_->dt;
			}
			else {
				//auto ds_target = tg_ds / std::max(ds_ratio, 0.01);
				//ds_target = std::min(ds_target, imp_->current_ds_);
				//imp_->tg_->setTargetDs(ds_target);
				//imp_->last_dds = imp_->tg_->currentDds();

				auto ds = tg_ds / ds_ratio;
				ds = std::max(0.01, ds);
				ds = std::min(imp_->current_ds_, ds);
				imp_->tg_->setCurrentDs(ds);
				imp_->tg_->setCurrentDds(0.0);
				imp_->tg_->setTargetDs(ds);
				imp_->last_dds = (ds - tg_ds) / imp_->dt;
			}

#ifdef ARIS_DEBUG_SINGULAR_PROCESSOR
			static int print_count = 0;

			if(print_count++ < 2000)

			//if (ds_ratio > 1.0)
				std::cout << "less: ds:" << tg_ds << "  ds_ratio" << ds_ratio
				<< "  vel_ratio" << vel_ratio << "  acc_ratio" << acc_ratio << std::endl;
#endif

			return ret;
		};
		
		// move in singular state
		auto move_in_singular = [this]()->std::int64_t {
			// 取出位置
			std::swap(imp_->input_pos_this_, imp_->input_pos_last_);
			std::swap(imp_->input_vel_this_, imp_->input_vel_last_);
			for (int i = 0; i < imp_->input_size_; ++i) {
				//imp_->input_pos_this_[i] = s_third_polynomial_compute_value(imp_->curve_params_[i], imp_->current_singular_count_ * imp_->dt);
				imp_->input_pos_this_[i] = s_tcurve_value(imp_->curve_params_[i], imp_->current_singular_count_ * imp_->dt);
			}

			imp_->model_->setInputPos(imp_->input_pos_this_);
			imp_->model_->forwardKinematics();

			// 计算速度
			aris::dynamic::s_vc(imp_->input_size_, imp_->input_pos_this_, imp_->input_vel_this_);
			aris::dynamic::s_vs(imp_->input_size_, imp_->input_pos_last_, imp_->input_vel_this_);
			aris::dynamic::s_nv(imp_->input_size_, 1.0 / imp_->dt, imp_->input_vel_this_);

			// 计算加速度
			aris::dynamic::s_vc(imp_->input_size_, imp_->input_vel_this_, imp_->input_acc_this_);
			aris::dynamic::s_vs(imp_->input_size_, imp_->input_vel_last_, imp_->input_acc_this_);
			aris::dynamic::s_nv(imp_->input_size_, 1.0 / imp_->dt, imp_->input_acc_this_);

			imp_->current_singular_count_++;
			if (imp_->current_singular_count_ > imp_->total_singular_count_)
				imp_->state_ = Imp::SingularState::NORMAL;

			return imp_->singular_ret_;
		};

		// check if singular //
		auto check_if_singular = [](aris::Size input_size, const double *max_vel, const double*max_acc, const double*vel, const double *acc)->int {
			// here is condition //
			int idx = 0;
			for (idx = 0; idx < input_size; ++idx) {
				if (vel[idx] > max_vel[idx] || vel[idx] < -max_vel[idx] || acc[idx] > max_acc[idx] || acc[idx] < -max_acc[idx]) {
#ifdef ARIS_DEBUG_SINGULAR_PROCESSOR
					std::cout << "singular idx" << idx << " vel:" << vel[idx] << "  max_vel:" << max_vel[idx] << "  acc:" << acc[idx] << "  max_acc:" << max_acc[idx] << std::endl;
#endif
					return idx;
				}
			}
			return input_size;
		};

		auto prepare_singular = [&]()->std::int64_t {
#ifdef ARIS_DEBUG_SINGULAR_PROCESSOR
			std::cout << "singular parpare" << std::endl;
#endif
			imp_->state_ = Imp::SingularState::SINGULAR_PREPARE;

			// 尝试迭代到下一个非奇异的时刻，最多迭代 10 次 //
			const int MAX_ITER_COUNT = 12;

			int idx;
			int while_count{ 0 };
			do {
				while_count++;
				move_tg_step();
				idx = check_if_singular(imp_->input_size_, imp_->max_vels_, imp_->max_accs_, imp_->input_vel_this_, imp_->input_acc_this_);

				// 尝试处理奇异情况 //
				if (idx == imp_->input_size_ || while_count >= MAX_ITER_COUNT) {
					// 获取终止时刻 //
					aris::dynamic::s_vc(imp_->input_size_, imp_->input_pos_this_, imp_->input_pos_end_);
					aris::dynamic::s_vc(imp_->input_size_, imp_->input_vel_this_, imp_->input_vel_end_);

					// 考虑结束条件可能是循环次数到了
					for (int i = 0; i < imp_->input_size_; ++i) {
						imp_->input_vel_end_[i] = std::max(-imp_->max_vels_[i], imp_->input_vel_end_[i]);
						imp_->input_vel_end_[i] = std::min(imp_->max_vels_[i], imp_->input_vel_end_[i]);
					}

					// 计算每根轴所需要的时间 //
					std::int64_t Ts_count[100];
					for (int i = 0; i < imp_->input_size_; ++i) {
						auto& tcurve_param = imp_->curve_params_[i];

						tcurve_param.pb = imp_->input_pos_begin_[i];
						tcurve_param.pe = imp_->input_pos_this_[i];
						tcurve_param.vb = imp_->input_vel_begin_[i];
						tcurve_param.ve = imp_->input_vel_this_[i];
						tcurve_param.vmax = imp_->max_vels_[i];
						tcurve_param.amax = imp_->max_accs_[i];

						double T1, T2, T3;
						s_tcurve_T_range(tcurve_param, T1, T2, T3);
						if (T1 == T3) {
							Ts_count[i * 3] = (std::int64_t)std::ceil(T1 / imp_->dt);
							Ts_count[i * 3 + 1] = (std::int64_t)std::ceil(T2 / imp_->dt);
							Ts_count[i * 3 + 2] = (std::int64_t)std::ceil(T3 / imp_->dt);
						}
						else {
							Ts_count[i * 3] = (std::int64_t)std::ceil(T1 / imp_->dt);
							Ts_count[i * 3 + 1] = (std::int64_t)std::floor(T2 / imp_->dt);
							Ts_count[i * 3 + 2] = (std::int64_t)std::ceil(T3 / imp_->dt);
						}
					}

					// 寻找最小可行时间
					auto Tmin_count = *std::max_element(Ts_count, Ts_count + imp_->input_size_ * 3);
					for (int i = 0; i < imp_->input_size_; ++i) {
						if (Ts_count[i * 3 + 2] < Tmin_count) {
							auto candidate_count = Ts_count[i * 3 + 2];
							bool candidate_failed = false;
							for (int j = 0; j < imp_->input_size_; ++j) {
								if ((candidate_count <= Ts_count[j * 3]
									|| (candidate_count >= Ts_count[j * 3 + 1]) && candidate_count <= Ts_count[j * 3 + 2])) {
									candidate_failed = true;
									break;
								}
							}
							if (!candidate_failed)
								Tmin_count = candidate_count;
						}

						if (Ts_count[i * 3] < Tmin_count) {
							auto candidate_count = Ts_count[i * 3];
							bool candidate_failed = false;
							for (int j = 0; j < imp_->input_size_; ++j) {
								if ((candidate_count <= Ts_count[j * 3]
									|| (candidate_count >= Ts_count[j * 3 + 1]) && candidate_count <= Ts_count[j * 3 + 2])) {
									candidate_failed = true;
									break;
								}
							}
							if (!candidate_failed)
								Tmin_count = candidate_count;
						}
					}

					// 根据 Tmin 生成曲线
					for (int i = 0; i < imp_->input_size_; ++i) {
						imp_->curve_params_[i].T = Tmin_count * imp_->dt;
						s_tcurve_param(imp_->curve_params_[i]);
					}

					imp_->total_singular_count_ = Tmin_count;
					imp_->current_singular_count_ = 1;

					// 判断是否满足完全修复条件
					auto& singular_param = imp_->curve_params_[imp_->singular_idx];
					if ((singular_param.vb * singular_param.ve < 0.0)
						|| (singular_param.v * singular_param.vb >= 0))
					{
						imp_->state_ = Imp::SingularState::SINGULAR;
					}
				}
			} while (imp_->state_ != Imp::SingularState::SINGULAR && while_count < MAX_ITER_COUNT);

			// 复原上一时刻
			aris::dynamic::s_vc(imp_->input_size_, imp_->input_pos_begin_, imp_->input_pos_this_);
			aris::dynamic::s_vc(imp_->input_size_, imp_->input_vel_begin_, imp_->input_vel_this_);

			// 移动一步
			return move_in_singular();
		};


		if (imp_->state_ == Imp::SingularState::SINGULAR) {
			// 当前已经处于奇异状态 //
			return move_in_singular();
		}
		else if (imp_->state_ == Imp::SingularState::SINGULAR_PREPARE) {
#ifdef ARIS_DEBUG_SINGULAR_PROCESSOR
			std::cout << "singular prepare" << std::endl;
#endif

			// 这一次的开始是真正开始的值 //
			aris::dynamic::s_vc(imp_->input_size_, imp_->input_pos_this_, imp_->input_pos_begin_);
			aris::dynamic::s_vc(imp_->input_size_, imp_->input_vel_this_, imp_->input_vel_begin_);
			aris::dynamic::s_vc(imp_->input_size_, imp_->input_pos_end_, imp_->input_pos_this_);
			aris::dynamic::s_vc(imp_->input_size_, imp_->input_vel_end_, imp_->input_vel_this_);
			return prepare_singular();
		}
		else {
			imp_->singular_ret_ = move_tg_step();
			if ((imp_->singular_idx = check_if_singular(imp_->input_size_, imp_->max_vels_, imp_->max_accs_, imp_->input_vel_this_, imp_->input_acc_this_)) == imp_->input_size_) {
				return imp_->singular_ret_;
			}
			else {
				// 上一次的数据，才是真正开始的值 //
				aris::dynamic::s_vc(imp_->input_size_, imp_->input_pos_last_, imp_->input_pos_begin_);
				aris::dynamic::s_vc(imp_->input_size_, imp_->input_vel_last_, imp_->input_vel_begin_);
#ifdef ARIS_DEBUG_SINGULAR_PROCESSOR
				std::cout << "singular" << std::endl;
#endif
				return prepare_singular();
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
