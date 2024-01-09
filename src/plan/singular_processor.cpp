#include"aris/plan/singular_processor.hpp"
#include"aris/plan/function.hpp"

//#define ARIS_DEBUG_SINGULAR_PROCESSOR

namespace aris::plan {
	struct ThirdPolynomialParam {
		double a, b, c, d;
	};
	auto s_third_polynomial_compute_Tmin(double pb, double pe, double vb, double ve, double vmax, double amax)->double {
		// 根据速度算 T 的最小值
		double A1 = (-vb * vb - vb * ve - 3 * vmax * vb - ve * ve - 3 * vmax * ve);
		double B1 = (6 * pe * vb - 6 * pb * ve - 6 * pb * vb + 6 * pe * ve - 6 * pb * vmax + 6 * pe * vmax);
		double C1 = (-9 * pb * pb + 18 * pb * pe - 9 * pe * pe);

		double A2 = (-vb * vb - vb * ve + 3 * vmax * vb - ve * ve + 3 * vmax * ve);
		double B2 = (6 * pe * vb - 6 * pb * ve - 6 * pb * vb + 6 * pe * ve + 6 * pb * vmax - 6 * pe * vmax);
		double C2 = (-9 * pb * pb + 18 * pb * pe - 9 * pe * pe);

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
			if ((0 < m && m < T) || T < 0.0) {
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

	struct TcurveParam {
		double pb, pe, vb, ve, vmax, amax;
		double T, Ta, Tb, v, a;
		int mode;
	};
	// 计算可行的时间域
	// (T1 T2), (T3,inf)
	auto s_tcurve_T_range(const TcurveParam& param, double& T1, double& T2, double& T3)->void {
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
			if ((vmax + vb) / 2 * (vmax - vb) / amax + (vmax + ve) / 2 * (vmax - ve) / amax < pt) {
				v = vmax;
				double Ta = (v - vb) / amax;
				double Tb = (v - ve) / amax;
				T1 = (pt - Ta * (vb + v) / 2 - Tb * (ve + v) / 2) / v + Ta + Tb;
			}
			else {
				v = std::sqrt(std::abs(pt * amax + (vb * vb + ve * ve) / 2));
				T1 = (v - vb) / amax + (v - ve) / amax;
			}
		}
		else {
			if ((-vmax + vb) / 2 * (vb + vmax) / amax + (-vmax + ve) / 2 * (ve + vmax) / amax > pt) {
				v = -vmax;
				double Ta = (vb - v) / amax;
				double Tb = (ve - v) / amax;
				T1 = (pt - Ta * (vb + v) / 2 - Tb * (ve + v) / 2) / v + Ta + Tb;
			}
			else {
				v = -std::sqrt(std::abs(-pt * amax + (vb * vb + ve * ve) / 2));
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
		else if (v < 0 && vb < 0 && ve < 0) {
			if (-vb * vb / 2 / amax - ve * ve / 2 / amax < pt) {
				v = -std::sqrt(std::abs(pt * amax + (vb * vb + ve * ve) / 2));
				T2 = std::abs(vb - v) / amax + std::abs(ve - v) / amax;
				T3 = std::abs(vb + v) / amax + std::abs(ve + v) / amax;
			}
		}
	}
	// 计算param
	auto s_tcurve_param(TcurveParam& param)->void {
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

			double A = 1;
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

			param.v = (-B + std::sqrt(std::abs(B * B - 4 * A * C))) / (2 * A);
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
				return pb + vb * t + a * t * t / 2;
			else if (t < T - Tb)
				return pb + vb * Ta + a * Ta * Ta / 2 + v * (t - Ta);
			else
				return pe - ve * (T - t) - a * (T - t) * (T - t) / 2;
		else
			if (t < Ta)
				return pb + vb * t;
			else if (t < T - Tb)
				return pb + vb * t + a * (t - Ta) * (t - Ta) / 2;
			else
				return  pe - ve * (T - t);
	}

	struct SingularProcessor::Imp {
		using CurveParam = TcurveParam;

		InverseKinematicMethod inv_func_;

		aris::Size input_size_{ 0 };
		//double dt_{ 0.001 };



		std::vector<char> mem_;
		double* max_vels_,
			* max_accs_,
			* input_pos_begin_,
			* input_vel_begin_,
			* input_acc_begin_,
			* input_pos_end_,
			* input_vel_end_,
			* input_acc_end_,
			* input_pos_last_,
			* input_vel_last_,
			* input_pos_this_,
			* input_vel_this_,
			* input_acc_this_,
			* input_acc_ratio_,
			* input_acc_max_consider_ratio_,
			* input_acc_min_consider_ratio_,
			* output_pos_;

		std::int64_t singular_ret_{ 0 };
		aris::Size total_singular_count_{ 0 }, current_singular_count_{ 0 };
		CurveParam* curve_params_;



		double T;
		double check_rate_{ 0.99 };
		double max_vel_ratio_{ 0.95 };
		double max_acc_ratio_{ 0.9 };
		double current_ds_{ 1.0 }, // 类似 tg 中的 ds，但是在降速的时候，它其实是系统的上界，真实的 tg 里的ds 并不一定是这个值
			target_ds_{ 1.0 },  // 类似 tg 中的 target_ds，它是ds 追赶的目标
			last_dds{ 0.0 };
		
		aris::dynamic::ModelBase* model_{ nullptr };
		aris::plan::TrajectoryGenerator* tg_{ nullptr };

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
	auto SingularProcessor::setMaxVelRatio(double vel_ratio)->void {
		imp_->max_vel_ratio_ = vel_ratio;
	}
	auto SingularProcessor::setMaxAccRatio(double acc_ratio)->void {
		imp_->max_acc_ratio_ = acc_ratio;
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
		core::allocMem(mem_size, imp_->input_acc_max_consider_ratio_, imp_->input_size_);
		core::allocMem(mem_size, imp_->input_acc_min_consider_ratio_, imp_->input_size_);
		core::allocMem(mem_size, imp_->output_pos_, imp_->input_size_);
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
		imp_->input_acc_ratio_ = core::getMem(imp_->mem_.data(), imp_->input_acc_ratio_);
		imp_->input_acc_max_consider_ratio_ = core::getMem(imp_->mem_.data(), imp_->input_acc_max_consider_ratio_);
		imp_->input_acc_min_consider_ratio_ = core::getMem(imp_->mem_.data(), imp_->input_acc_min_consider_ratio_);
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

		imp_->current_ds_ = imp_->target_ds_;
		imp_->last_dds = 0.0;
		imp_->state_ = Imp::SingularState::NORMAL;
	}
	auto SingularProcessor::setDs(double ds)->void {
		imp_->current_ds_ = ds;
	}
	auto SingularProcessor::currentDs() -> double {
		return imp_->current_ds_;
	}
	auto SingularProcessor::setTargetDs(double ds)->void {
		imp_->target_ds_ = ds;
	}
	auto SingularProcessor::setModelPosAndMoveDt()->std::int64_t {

#ifdef ARIS_DEBUG_SINGULAR_PROCESSOR
		static int count{ 0 };
		count++;
		if (count % 1000 == 0)
			std::cout << "count: " << count++ << std::endl;
		if (count == 125)
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
		// max_vel_ratio 和 max_acc_ratio 会触发正常降速
		auto move_tg_step = [this, get_max_ratio]()->std::int64_t {
			double zero_check = 1e-10;
			auto dt = imp_->tg_->dt();

			// 当前处于非奇异状态，正常求反解 //
			auto ret = imp_->tg_->getEePosAndMoveDt(imp_->output_pos_);
			if (imp_->inv_func_) {
				if (auto ret = imp_->inv_func_(*imp_->model_, imp_->output_pos_); ret)
					return ret;
			}
			else {
				imp_->model_->setOutputPos(imp_->output_pos_);
				if (auto ret = imp_->model_->inverseKinematics(); ret)
					return ret;
			}

			auto last_ds = imp_->current_ds_;

			// 取出位置
			std::swap(imp_->input_pos_this_, imp_->input_pos_last_);
			std::swap(imp_->input_vel_this_, imp_->input_vel_last_);
			imp_->model_->getInputPos(imp_->input_pos_this_);

			// 计算速度
			aris::dynamic::s_vc(imp_->input_size_, imp_->input_pos_this_, imp_->input_vel_this_);
			aris::dynamic::s_vs(imp_->input_size_, imp_->input_pos_last_, imp_->input_vel_this_);
			aris::dynamic::s_nv(imp_->input_size_, 1.0 / dt, imp_->input_vel_this_);

			// 计算加速度
			aris::dynamic::s_vc(imp_->input_size_, imp_->input_vel_this_, imp_->input_acc_this_);
			aris::dynamic::s_vs(imp_->input_size_, imp_->input_vel_last_, imp_->input_acc_this_);
			aris::dynamic::s_nv(imp_->input_size_, 1.0 / dt, imp_->input_acc_this_);

			//t0       t1         t2
			//
			//p0       p1         p2
			//   dp1        dp2
			//        d2p2
			//
			//         s1         s2
			//   ds1        ds2
			//        d2s2
			//
			//         ds(t1)    ds(t2)
			//        d2s(t1)    d2s(t2)
			//
			//
			//at point 1:
			//
			//dp(t1)  = dp_ds(t1) * ds(t1)
			//d2p(t1) = d2p_ds2(t1) * ds(t1)^2 + dp_ds(t1) * d2s(t1)
			//        = d2p_ds2(t1) * ds(t1)^2 + (dp1+dp2)/2 / ((ds1+ds2)/2)

			// 计算去除 dds 项所影响的加速度，因此以下为 ds 不变时，速度和加速度的比例
			// dp  = dp_ds * ds
			// d2p = d2p_ds2 * ds^2 + dp_ds * d2s
			//     = d2p_ds2 * ds^2 + dp * d2s / ds
			// =>
			// d2p_ds2 * ds^2 = d2p - dp * d2s / ds
			double ds_real = ((last_ds - imp_->last_dds * dt) + last_ds) / 2;
			aris::dynamic::s_vc(imp_->input_size_, imp_->input_acc_this_, imp_->input_acc_ratio_);
			aris::dynamic::s_va(imp_->input_size_, -0.5 * imp_->last_dds / ds_real, imp_->input_vel_this_, imp_->input_acc_ratio_);
			aris::dynamic::s_va(imp_->input_size_, -0.5 * imp_->last_dds / ds_real, imp_->input_vel_last_, imp_->input_acc_ratio_);

			// 正常情况下，可能需改变规划器中ds //
			auto vel_ratio = get_max_ratio(imp_->input_size_, imp_->max_vels_, imp_->input_vel_this_) / imp_->max_vel_ratio_;
			auto acc_ratio = std::sqrt(get_max_ratio(imp_->input_size_, imp_->max_accs_, imp_->input_acc_ratio_) / imp_->max_acc_ratio_);

			// 判断是否要改 ds_ratio，用以判断是否需要减速 //
			auto ds_ratio = std::max({
				vel_ratio ,
				acc_ratio
				});

			// 需要根据当前的速度情况，计算max_dds min_dds  //
			//
			// d2p_min < d2p < d2p_max
			// =>
			// d2p_min < d2p_ds2 * ds ^ 2 + dp / ds * d2s < d2p_max
			// =>
			// d2p_min - d2p_ds2 * ds ^ 2 < dp / ds * d2s < d2p_max - d2p_ds2 * ds ^ 2 
			// 
			// 其中 d2p_ds2 * ds ^ 2 在上文中计算过
			//
			// 因此若 dp < 0
			// [d2p_max - (d2p_ds2 * ds^2)]*ds/dp  < d2s < [d2p_min - (d2p_ds2 * ds^2)]*ds/dp
			// 否则
			// [d2p_min - (d2p_ds2 * ds^2)]*ds/dp  < d2s < [d2p_max - (d2p_ds2 * ds^2)]*ds/dp
			// 
			for (int i = 0; i < imp_->input_size_; ++i) {
				if (std::abs(imp_->input_vel_this_[i]) < zero_check) {
					imp_->input_acc_max_consider_ratio_[i] = 1e10;
					imp_->input_acc_min_consider_ratio_[i] = -1e10;
				}
				else {
					imp_->input_acc_max_consider_ratio_[i] = (0.99*imp_->max_accs_[i] - imp_->input_acc_ratio_[i]) * last_ds / imp_->input_vel_this_[i];
					imp_->input_acc_min_consider_ratio_[i] = (-0.99*imp_->max_accs_[i] - imp_->input_acc_ratio_[i]) * last_ds / imp_->input_vel_this_[i];
					if (imp_->input_vel_this_[i] < 0)
						std::swap(imp_->input_acc_max_consider_ratio_[i], imp_->input_acc_min_consider_ratio_[i]);
				}
			}
			double max_dds = *std::min_element(imp_->input_acc_max_consider_ratio_, imp_->input_acc_max_consider_ratio_ + imp_->input_size_);
			double min_dds = *std::max_element(imp_->input_acc_min_consider_ratio_, imp_->input_acc_min_consider_ratio_ + imp_->input_size_);

			// 确保 max_dds > min_dds
			double mid = (max_dds + min_dds) / 2;
			max_dds = std::max(mid, max_dds);
			min_dds = std::min(mid, min_dds);

			// 进行调速 //
			double target_ds = std::min(imp_->target_ds_, (vel_ratio > acc_ratio ? last_ds : ds_real) / ds_ratio); // 注意，这里的速度/加速度降速的比较基准不一样
			target_ds = std::max(0.01, target_ds);
			target_ds = std::min(1.0, target_ds);
			if (target_ds - last_ds > dt * max_dds) {
				imp_->current_ds_ = last_ds + dt * max_dds;
			}
			else if (target_ds - last_ds < dt * min_dds) {
				imp_->current_ds_ = last_ds + dt * min_dds;
			}
			else {
				imp_->current_ds_ = target_ds;
			}

			//if (target_ds < 1.0) {
			//	std::cout << "dec : " << target_ds << std::endl;
			//}


			imp_->current_ds_ = std::max(0.01, imp_->current_ds_);
			imp_->current_ds_ = std::min(1.0, imp_->current_ds_);

			imp_->tg_->setCurrentDs(imp_->current_ds_);
			imp_->tg_->setCurrentDds(0.0);
			imp_->tg_->setTargetDs(imp_->current_ds_);
			imp_->last_dds = (imp_->current_ds_ - last_ds) / dt;

#ifdef ARIS_DEBUG_SINGULAR_PROCESSOR
			static int print_count = 0;

			if (print_count++ > 200 && print_count < 220)
			{
				std::cout << print_count << ": ds:" << last_ds << "  ds_ratio" << ds_ratio
					<< "  vel_ratio" << vel_ratio << "  acc_ratio" << acc_ratio << "  dds:" << imp_->last_dds << std::endl;
				
				

				for (int i = 0; i < imp_->input_size_; ++i) {
					std::cout << std::abs(imp_->input_acc_ratio_[i]/ imp_->max_accs_[i]) << "  ";
				}
				std::cout << std::endl;
				for (int i = 0; i < imp_->input_size_; ++i) {
					std::cout << std::sqrt(std::abs(imp_->input_acc_ratio_[i] / imp_->max_accs_[i])) << "  ";
				}
				std::cout << std::endl;
				for (int i = 0; i < imp_->input_size_; ++i) {
					std::cout << std::sqrt(std::abs(imp_->input_acc_ratio_[i] / imp_->max_accs_[i]) / imp_->max_acc_ratio_) << "  ";
				}
				std::cout << std::endl;
				for (int i = 0; i < imp_->input_size_; ++i) {
					std::cout << ds_real / std::sqrt(std::abs(imp_->input_acc_ratio_[i] / imp_->max_accs_[i]) / imp_->max_acc_ratio_) << "  ";
				}
				std::cout << std::endl;

				aris::dynamic::dsp(1, imp_->input_size_, imp_->input_acc_ratio_);
				aris::dynamic::s_nv(imp_->input_size_, 1.0 / ds_real / ds_real, imp_->input_acc_ratio_);
				aris::dynamic::dsp(1, imp_->input_size_, imp_->input_acc_ratio_);

				for (int i = 0; i < imp_->input_size_; ++i) {
					std::cout << 1.0/std::sqrt(std::abs(imp_->input_acc_ratio_[i] / imp_->max_accs_[i]) / imp_->max_acc_ratio_) << "  ";
				}
				std::cout << std::endl;

				std::cout << "------------------" << std::endl;
			}
			//if (ds_ratio > 1.0)

#endif

			return ret;
		};

		// move in singular state
		// 不考虑 max_vel_ratio 与 max_acc_ratio，奇异降速时，只考虑真实电机限制
		auto move_in_singular = [this]()->std::int64_t {
			auto dt = imp_->tg_->dt();
			
			// 取出位置
			std::swap(imp_->input_pos_this_, imp_->input_pos_last_);
			std::swap(imp_->input_vel_this_, imp_->input_vel_last_);
			for (int i = 0; i < imp_->input_size_; ++i) {
				imp_->input_pos_this_[i] = s_tcurve_value(imp_->curve_params_[i], imp_->current_singular_count_ * dt);
			}

			imp_->model_->setInputPos(imp_->input_pos_this_);
			imp_->model_->forwardKinematics();

			// 计算速度
			aris::dynamic::s_vc(imp_->input_size_, imp_->input_pos_this_, imp_->input_vel_this_);
			aris::dynamic::s_vs(imp_->input_size_, imp_->input_pos_last_, imp_->input_vel_this_);
			aris::dynamic::s_nv(imp_->input_size_, 1.0 / dt, imp_->input_vel_this_);

			// 计算加速度
			aris::dynamic::s_vc(imp_->input_size_, imp_->input_vel_this_, imp_->input_acc_this_);
			aris::dynamic::s_vs(imp_->input_size_, imp_->input_vel_last_, imp_->input_acc_this_);
			aris::dynamic::s_nv(imp_->input_size_, 1.0 / dt, imp_->input_acc_this_);

			imp_->current_singular_count_++;
			if (imp_->current_singular_count_ > imp_->total_singular_count_)
				imp_->state_ = Imp::SingularState::NORMAL;

			return imp_->singular_ret_;
		};

		// check if singular //
		auto check_if_singular = [](aris::Size input_size, const double* max_vel, const double* max_acc, const double* vel, const double* acc)->int {
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
			auto dt = imp_->tg_->dt();
			do {
				while_count++;
				imp_->singular_ret_ = move_tg_step();
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
							Ts_count[i * 3] = (std::int64_t)std::ceil(T1 / dt);
							Ts_count[i * 3 + 1] = (std::int64_t)std::ceil(T2 / dt);
							Ts_count[i * 3 + 2] = (std::int64_t)std::ceil(T3 / dt);
						}
						else {
							Ts_count[i * 3] = (std::int64_t)std::ceil(T1 / dt);
							Ts_count[i * 3 + 1] = (std::int64_t)std::floor(T2 / dt);
							Ts_count[i * 3 + 2] = (std::int64_t)std::ceil(T3 / dt);
						}
					}

					// 寻找最小可行时间
					auto Tmin_count = *std::max_element(Ts_count, Ts_count + imp_->input_size_ * 3);
					for (int i = 0; i < imp_->input_size_; ++i) {
						if (Ts_count[i * 3 + 2] < Tmin_count) {
							auto candidate_count = Ts_count[i * 3 + 2];
							bool candidate_failed = false;
							for (int j = 0; j < imp_->input_size_; ++j) {
								if ((candidate_count < Ts_count[j * 3]
									|| ((candidate_count > Ts_count[j * 3 + 1]) && candidate_count < Ts_count[j * 3 + 2]))) {
									candidate_failed = true;
									break;
								}
							}
							if (!candidate_failed)
								Tmin_count = candidate_count;
						}

						if (Ts_count[i * 3 + 1] < Tmin_count && Ts_count[i * 3] <= Ts_count[i * 3 + 1]) {
							auto candidate_count = Ts_count[i * 3 + 1];
							bool candidate_failed = false;
							for (int j = 0; j < imp_->input_size_; ++j) {
								if ((candidate_count < Ts_count[j * 3]
									|| ((candidate_count > Ts_count[j * 3 + 1]) && candidate_count < Ts_count[j * 3 + 2]))) {
									candidate_failed = true;
									break;
								}
							}
							if (!candidate_failed)
								Tmin_count = candidate_count;
						}

						if (Ts_count[i * 3] < Tmin_count && Ts_count[i * 3] <= Ts_count[i * 3 + 1]) {
							auto candidate_count = Ts_count[i * 3];
							bool candidate_failed = false;
							for (int j = 0; j < imp_->input_size_; ++j) {
								if ((candidate_count < Ts_count[j * 3]
									|| ((candidate_count > Ts_count[j * 3 + 1]) && candidate_count < Ts_count[j * 3 + 2]))) {
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
						imp_->curve_params_[i].T = Tmin_count * dt;
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
	auto SingularProcessor::setInverseKinematicMethod(InverseKinematicMethod func)->void {
		imp_->inv_func_ = func;
	}
	SingularProcessor::~SingularProcessor() = default;
	SingularProcessor::SingularProcessor() :imp_(new Imp) {

	}






}
