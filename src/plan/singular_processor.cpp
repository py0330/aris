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
		double pb_, pe_, vb_, ve_, vmax_, amax_;
		double T_, Ta_, Tb_, v_, a_;
		int mode_;
	};
	// 计算可行的时间域
	// (T1 T2), (T3,inf)
	auto s_tcurve_T_range(const TcurveParam& param, double& T1, double& T2, double& T3)->void {
		const double pb = param.pb_;
		const double pe = param.pe_;
		const double vb = param.vb_;
		const double ve = param.ve_;
		const double vmax = param.vmax_;
		const double amax = param.amax_;

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
		const auto pb = param.pb_;
		const auto pe = param.pe_;
		const auto vb = param.vb_;
		const auto ve = param.ve_;
		const auto vmax = param.vmax_;
		const auto amax = param.amax_;
		const auto T = param.T_;

		auto& a = param.a_;
		auto& v = param.v_;
		auto& Ta = param.Ta_;
		auto& Tb = param.Tb_;
		auto& mode = param.mode_;

		double pt = pe - pb;
		double Tb2e = std::abs(vb - ve) / amax;
		double pb2e = (vb + ve) / 2 * Tb2e;


		if ((pt - pb2e) > (T - Tb2e) * std::max(vb, ve)) {
			param.mode_ = 0;
			param.a_ = amax;

			double A = 1;
			double B = -T * a - vb - ve;
			double C = (vb * vb + ve * ve) / 2 + pt * a;

			param.v_ = (-B - std::sqrt(std::abs(B * B - 4 * A * C))) / (2 * A);
			param.Ta_ = (v - vb) / a;
			param.Tb_ = (v - ve) / a;
		}

		else if ((pt - pb2e) > (T - Tb2e) * std::min(vb, ve)) {
			param.mode_ = 1;
			param.v_ = (pt - pb2e) / (T - Tb2e);
			Ta = (pt - pb2e) / (vb - ve) - ve / (vb - ve) * (T - Tb2e);
			Ta = std::max(0.0, Ta);
			param.Ta_ = std::min(Ta, T - Tb2e);
			param.Tb_ = T - Tb2e - Ta;
			param.a_ = aris::dynamic::s_sgn2(ve - vb) * amax;
		}

		else {
			param.mode_ = 0;
			param.a_ = -amax;
			double A = 1;
			double B = -T * a - vb - ve;
			double C = (vb * vb + ve * ve) / 2 + pt * a;

			param.v_ = (-B + std::sqrt(std::abs(B * B - 4 * A * C))) / (2 * A);
			param.Ta_ = (v - vb) / a;
			param.Tb_ = (v - ve) / a;
		}
	}
	// 计算值
	auto s_tcurve_value(const TcurveParam& param, double t)->double {
		const auto pb = param.pb_;
		const auto pe = param.pe_;
		const auto vb = param.vb_;
		const auto ve = param.ve_;
		const auto vmax = param.vmax_;
		const auto amax = param.amax_;
		const auto T = param.T_;

		const auto a = param.a_;
		const auto v = param.v_;
		const auto Ta = param.Ta_;
		const auto Tb = param.Tb_;
		const auto mode = param.mode_;


		if (mode == 0)
			if (t < Ta)
				return pb + vb * t + a * t * t / 2;
			else if (t < T - Tb)
				return pb + vb * Ta + a * Ta * Ta / 2 + v * (t - Ta);
			else
				return pe - ve * (T - t) - a * (T - t) * (T - t) / 2;
		else {

			const double Ta = param.Ta_ - std::min(param.Ta_, param.Tb_);
			const double Tb = param.Tb_ - std::min(param.Ta_, param.Tb_);
			const double lower_ratio = (param.T_ - Ta - Tb) < 1e-9 ? 1.0 : (param.T_ - param.Ta_ - param.Tb_) / (param.T_ - Ta - Tb);
			const double a = param.a_ * lower_ratio;

			if (t < Ta)
				return pb + vb * t;
			else if (t < T - Tb)
				return pb + vb * t + a * (t - Ta) * (t - Ta) / 2;
			else
				return  pe - ve * (T - t);
		}
			
	}

	// t1   t15   t2   t25   t3            t4
	//
	// p1         p2         p             p_next
	//      v2         v          v_next
	//            a3
	// 已知 p v，为了满足后续所有时刻都不超p,v,a的限制，求得下一时刻许可的 v_next 的范围
	auto s_smooth2_v_range(double p_max, double p_min, double v_max, double v_min, double a_max, double a_min, double dt
		, double p, double v, double &v_upper, double &v_below)->int
	{
		//
		// 
		// 
		// t0        t1   t15   t2   t25   t3
		//
		// p0        p1         p2         p3
		//      v1         v2         v3
		//           a2         a3
		//                 j3
		// 
		// 
		// 
		//%  当前状态为 p,v > 0, 当前最大加速度为a_max，全力减速时
		//%  p(t) = p + v*t + 0.5*a_min*t*t
		//%  其极值为 
		//%  t          = -v/a_min
		//%  p(v/a_min) = p - v^2/(2*a_min) < p_max
		//%
		//%  => v <  sqrt(-2*a_min*(p_max-p))
		//%  同理：
		//%     v > -sqrt( 2*a_max*(p-p_min))
		//%  
		//%  v4    = v_t35 
		//%  p_t35 = p3 + (v4 + v3)/2*dt/2
		//%
		//%  于是：
		//%  v4 < sqrt(-2*a_min*(p_max-p_t35))
		//%  => -v4^2 /2 / a_min < p_max - p_t35
		//%  => -v4^2 /2 / a_min < p_max - p3 - (v4 + v3)/2*dt/2
		//%  
		//%  => v4 < sqrt(value2) + a_min*dt/4
		//%  其中：
		//%  value2 = ((a_min^2*dt^2)/16 + 2*a_min*p3 - 2*a_min*p_max + (v3*a_min*dt)/2)
		//
		//%
		//%  同理可求：
		//%  a4 > -sqrt(value1) + a_max*dt/4
		//%  其中：
		//%  value1 = ((a_max^2*dt^2)/16 + 2*a_max*p3 - 2*a_max*p_min + (v3*a_max*dt)/2)
		v_upper = v_max; 
		v_below = v_min;

		if (v > 0) {
			double value2 = ((a_min * a_min * dt * dt) / 16 + 2 * a_min * p - 2 * a_min * p_max + (v * a_min * dt) / 2);
			v_upper = a_min > 0.0 ? 0.0 : std::min(std::sqrt(value2) + a_min * dt / 4, v_max);

			//double v35_upper = std::sqrt(-2 * a_min * (p_max - p));
			//v_upper = a_min > 0.0 ? 0.0 : v35_upper + a_min * dt / 2;
			//v_upper = a_min > 0.0 ? 0.0 : v35_upper + a_min * dt;
		}
		else {
			double value1 = ((a_max * a_max * dt * dt) / 16 + 2 * a_max * p - 2 * a_max * p_min + (v * a_max * dt) / 2);
			v_below = a_max < 0.0 ? 0.0 : std::max(-std::sqrt(value1) + a_max * dt / 4, v_min);

			//double v35_upper = -std::sqrt(2 * a_max * (p - p_min));
			//v_below = a_max < 0.0 ? 0.0 : v35_upper + a_max * dt / 2;
		}

		return 0;
	};
	
	// t0        t1   t15   t2   t25   t3
	//
	// p0        p1         p2         p
	//      v1         v2         v
	//           a2         a          a_next
	//                 j
	//
	// 已知 p v a，为了满足后续所有时刻都不超p,v,a,j的限制，求得下一时刻许可的 a_next 的范围
	auto s_smooth3_a_range(double p_max, double p_min, double v_max, double v_min, double a_max, double a_min, double j_max, double j_min
		, double dt, double p, double v, double a, double &a_upper, double &a_below) -> int 
	{
		// t0        t1   t15   t2   t25   t3
		//
		// p0        p1         p2         p3
		//      v1         v2         v3
		//           a2         a3
		//                 j3

		double a_max_real = a_max, a_min_real = a_min;

		if (v < 0) {
			auto ret = s_smooth3_a_range(-p_min, -p_max, -v_min, -v_max, -a_min, -a_max, -j_min, -j_max
				, dt, -p, -v, -a, a_below, a_upper);
		
			a_upper = -a_upper;
			a_below = -a_below;

			return ret;
		}

		auto pb = p_max;

		auto A = -1 / (2 * j_min);
		auto B = dt / 2;
		auto C = a_min*a_min / (2 * j_min) + v;

		double a_ans1, a_ans2, p_case0, p_case1, p_case2, p_case3;

		{
			auto L1 = (v + (a_min * dt) / 2);
			p_case0 = p - (L1 * L1 / 2) / a_min;
			auto L4 = (a_min - a_max);
			auto L5 = (v + (a_max * dt) / 2 + (a_max * L4) / j_min + (L4 * L4 / 2) / j_min);
			p_case3 = p - L5*L5 / (2 * a_min) + L4 / j_min / j_min * ((a_max * L4) / 2 + ((v + (a_max * dt) / 2)) * j_min + (L4*L4 / 6));
		}
		if (B * B - 4 * A * C >= 0) {
			a_ans1 = (-B - std::sqrt(B * B - 4 * A * C)) / (2 * A);
			a_ans2 = (-B + std::sqrt(B * B - 4 * A * C)) / (2 * A);
			auto L2 = (a_ans1 - a_min);
			p_case1 = p + L2 / j_min / j_min * ((a_ans1 * L2) / 2 - (v + (a_ans1 * dt) / 2) * j_min - (L2 * L2 / 6));
			auto L3 = (a_ans2 - a_min);
			p_case2 = p + L3 / j_min / j_min * ((a_ans2 * L3) / 2 - (v + (a_ans2 * dt) / 2) * j_min - (L3 * L3 / 6));
		}
		else {
			a_ans1 = a_min;
			a_ans2 = a_min;
			p_case1 = p_case0;
			p_case2 = p_case0;
		}
		
		if (pb < p_case0) {
		    // branch 1.1
			auto k2 = v/2;
			auto k1 = p - pb;
			auto k0 = (dt*(p - pb))/2;
        
			auto t = (-k1 + std::sqrt(k1*k1-4*k0*k2))/(2*k2);
			a_upper = -(2*v)/(dt + 2*t);
			a_below = a_upper;
		}
		else if (pb < p_case1) {
			auto lhs = a_min;
			auto rhs = a_ans1;
			// branch 1.2
			auto k4 = -1 / (8 * a_min * j_min * j_min);
			auto k3 = 1 / (3 * j_min * j_min) + dt / (4 * a_min * j_min);
			auto k2 = -a_min / (2 * j_min * j_min) - (dt * dt / 4 - (v + a_min * a_min / (2 * j_min)) / j_min) / (2 * a_min) - dt / (2 * j_min);
			auto k1 = (a_min * dt) / (2 * j_min) - (dt * (a_min * a_min / (2 * j_min) + v)) / (2 * a_min) - v / j_min;
			auto k0 = p - (a_min * a_min / (2 * j_min) + v) * (a_min * a_min / (2 * j_min) + v) / (2 * a_min) + a_min * a_min * a_min / (6 * j_min * j_min) + (a_min * v) / j_min - pb;

			a_upper = aris::dynamic::s_newton_raphson_binary_search([k0,k1,k2,k3,k4](double x){
				return k4 * x * x * x * x + k3 * x * x * x + k2 * x * x + k1 * x + k0;
			}, lhs, rhs);
			a_below = a_min;
		}

		else if (pb < p_case2) {
		    // branch 1.3
			auto k4 = -j_min/12;
			auto k3 = -j_min*dt/6;
			auto k2 = v/2;
			auto k1 = p-pb;
			auto k0 = (dt*(p - pb))/2;
			auto t = a_upper = aris::dynamic::s_newton_raphson_binary_search([k0, k1, k2, k3, k4](double x) {
				return k4 * x * x * x * x + k3 * x * x * x + k2 * x * x + k1 * x + k0;
				}, (a_min - a_ans1) / j_min, (a_min - a_ans2) / j_min);

			a_upper = -((j_min*t*t)/2 + v)/(t+dt/2);
			a_below = a_min;
		}
		else if (pb < p_case3) {
			auto lhs = a_ans2;
			auto rhs = a_max;
			// branch 1.2
			auto k4 = -1 / (8 * a_min * j_min * j_min);
			auto k3 = 1 / (3 * j_min * j_min) + dt / (4 * a_min * j_min);
			auto k2 = -a_min / (2 * j_min * j_min) - (dt * dt / 4 - (v + a_min * a_min / (2 * j_min)) / j_min) / (2 * a_min) - dt / (2 * j_min);
			auto k1 = (a_min * dt) / (2 * j_min) - (dt * (a_min * a_min / (2 * j_min) + v)) / (2 * a_min) - v / j_min;
			auto k0 = p - (a_min * a_min / (2 * j_min) + v) * (a_min * a_min / (2 * j_min) + v) / (2 * a_min) + a_min * a_min * a_min / (6 * j_min * j_min) + (a_min * v) / j_min - pb;

			a_upper = aris::dynamic::s_newton_raphson_binary_search([k0, k1, k2, k3, k4](double x) {
				return k4 * x * x * x * x + k3 * x * x * x + k2 * x * x + k1 * x + k0;
				}, lhs, rhs);
			a_below = a_min;
		}
		else {
			a_upper = a_max;
			a_below = a_min;
		}
		return 0;
	}





	auto s_smooth_curve(const SmoothParam &param, SmoothRet &ret) -> int {
		
		///////////////////////////// PART 1 计算 d3s 的可选范围 ///////////////////////////////// 
		// 
		// 
		// 
		// 
		//% 以下考虑约束条件
		//% 【COND 1】经过 dt 时间后，速度不超过上下限
		//%  
		//%  dp_min < dp3 + (d2p3 + d3p25*dt)*dt < dp_max
		//%  =>
		//%  (dp_min - dp3 - d2p3*dt - g*dt*dt)/k/dt/dt 
		//%  <
		//%    d3s
		//%  <
		//%  (dp_max - dp3 - d2p3*dt - g*dt*dt)/k/dt/dt 
		//% 【COND 2】经过 dt 时间后，加速度不超过上下限
		//%
		//%  d2p_min < d2p3 + d3p_25 * dt < d2p_max
		//%
		//%  (d2p_min - d2p3 - g*dt)/k/dt
		//%
		//% 【COND 3】jerk 不超过上下限
		//%
		//%  d3p_min < d3p_25 < d3p_max
		//% 
		//% 【COND 4】jerk 不超过考虑速度与加速度的上下限
		//%
		//%  d2p_min2 < d2p3 + d3p_25 * dt < d2p_max2
		//%
		//%
		//% 【COND 4】推导如下：
		//%
		//%  将加速度减为0，所需时间：T = abs(d2p)/d3p_max
		//%  减为0时的速度：dp + T*d2p/2
		//%  dp_min < dp + T*d2p/2 < dp_max
		//%  =>
		//%  dp_min-dp < T*d2p/2 < dp_max-dp
		//%  2*(dp_min-dp)*d3p_max < sig(d2p)*d2p*d2p < 2*(dp_max-dp)*d3p_max
		//%  => 
		//%  d2p < 0时
		//%    d2p > -sqrt(2*(dp-dp_min)*d3p_max)
		//%  d2p > 0时
		//%    d2p < sqrt(2*(dp_max-dp)*d3p_max)
		//% 
		//%  考虑经过dt，则应有：
		//%  
		//%  d2p < 0时
		//%    d2p > -sqrt(2*(dp-dp_min-d2p_max*dt)*d3p_max)
		//%  d2p > 0时
		//%    d2p < sqrt(2*(dp_max-dp-d2p_max*dt)*d3p_max)
		// 
		// 应有以下等式：
		// dp_min  < dp  < dp_max
		// d2p_min < d2p < d2p_max
		// d3p_min < d3p < d3p_max
		//
		// dp  = dp_ds * ds
		// d2p = d2p_ds2 * ds^2 + dp_ds * d2s
		// d3p = d3p_ds3 * ds^3 + 2 * d2p_ds2 * ds * d2s + d2p_ds2 * ds * d2s + dp_ds * d3s
		//     = d3p_ds3 * ds^3 + 3 * d2p_ds2 * ds * d2s + dp_ds * d3s
		//
		// 带入上述不等式，应有：
		//                    dp_min/dp_ds < ds  < dp_max/dp_ds
		//  (d2p_max-d2p_ds2 * ds^2)/dp_ds < d2s < (d2p_max-d2p_ds2 * ds^2)/dp_ds
		//               (d3p_min-f)/dp_ds < d3s < (d3p_max-f)/dp_ds
		//
		// 其中 f = d3p_ds2 * ds^3 + 3 * d2p_ds * ds * d2s
		// 
		// 若 dp_ds < 0，则上式左右相反。
		//
		// 上式仅考虑了电机端的输入，没有考虑加速度将为 0 也需要时间。此时：
		//
		//       dp_min  <  dp + d2p * T + 0.5 * d3p * T^2  <  dp_max
		//                                  
		// 中间式子在 T = -d2p/d3p 时取到极值，大小为：
		//    
		//       dp - 0.5*d2p*d2p/d3p
		//
		// 若 d2p < 0 && dp_ds < 0，则应有 d3p > 0：
		//       dp_min < dp - 0.5*d2p*d2p/d3p
		// =>    d2p*d2p/d3p < 2*(dp-dp_min)
		// =>    d3p > d2p*d2p/(2(dp-dp_min))
		// =>    d3s < (d2p*d2p/(2(dp-dp_min)) - f)/dp_ds
		// 		
		// 若 d2p > 0 && dp_ds > 0，则应有 d3p > 0：
		//
		//       d3p < d2p*d2p/(2(dp_max-dp))
		// =>    d3s < (d2p*d2p/(2(dp-dp_max)) - f)/dp_ds
		//
		// 考虑加速度不超限，应有：
		// 
		//                           d2p_min  < d2p + d3p*dt < d2p_max
		// =>             (d2p_min - d2p)/dt  <      d3p     <  (d2p_max - d2p)/dt
		// =>  ((d2p_min - d2p)/dt - f)/dp_ds <      d3s     <  ((d2p_max - d2p)/dt - f)/dp_ds
		// 
		///////////////////////////// PART 2 计算其中的数据 ///////////////////////////////// 
		// t0        t1   t15   t2   t25   t3
		//
		// p0        p1         p2         p3
		//     dp1        dp2        dp3
		//          d2p2       d2p3
		//               d3p3
		//  
		// s0        s1         s2         s3
		//     ds1        ds2        ds3
		//          d2s2       d2s3
		//               d3s3
		//
		//
		//
		// at point t=1.5:
		//
		// dp_ds_t15   = dp_t15/ds_t15
		// d2p_ds2_t15 = (d2p_t15 - dp_ds_t15 * d2s_t15)/ds_t15^2
		// d3p_ds3_t15 = (d3p_t15 - 3 * d2p_ds2_t15 * ds_t15 * d2s_t15 - dp_ds_t15 * d3s_t15)/ds_t15^3
		// 
		// at point t=2.5:
		// 
		// ds_t25 = ds3
		// d2s_t25 = d2s_t15 + (d3s_t15+d3s_25)/2*dt
		// 
		// d3p_ds3_t25 = d3p_ds3_t15
		// d2p_ds2_t25 = d2p_ds2_t15 + d3p_ds3_t15 * (s_25 - s_15)
		// dp_ds_t25   = dp_ds_t15 + d2p_ds2_t15 * (s_25 - s_15) + 0.5 * d3p_ds3_t15 * (s_25 - s_15)^2
		// 
		// f_25 = d3p_ds3_t25 * ds_t25^3 + 3 * d2p_ds2_t25 * ds_t25 * d2s_t25
		//      = d3p_ds3_t25 * ds_t25^3 + 3 * d2p_ds2_t25 * ds_t25 * (d2s_t15 + (d3s_t15+d3s_25)/2*dt)
		//      = d3p_ds3_t25 * ds_t25^3 + 3 * d2p_ds2_t25 * ds_t25 * (d2s15 + d3s_t15*dt/2) + g*dp_ds_t25* d3s_25
		//      = k + g*dp_ds_t25* d3s_25
		// 
		// 其中:
		//    k = d3p_ds3_t25 * ds_t25^3 + 3 * d2p_ds2_t25 * ds_t25 * (d2s15 + d3s_t15*dt/2)
		//    g = 3 * d2p_ds2_t25 * ds_t25 /2 * dt / dp_ds_t25
		// 
		// 根据以下3个不等式，可求出 d3s 的许可范围：
		//                 (d3p_min-f_25)/dp_ds_t25 < d3s_25 < (d3p_max-f_25)/dp_ds_t25
		// ((d2p_min - d2p_25)/dt - f_25)/dp_ds_t25 < d3s_25 < ((d2p_max - d2p_25)/dt - f_25)/dp_ds_t25
		//                                           d3s_25 < (d2p_25*d2p_25/(2(dp_25-dp_max)) - f_25)/dp_ds_t25
		// 
		// => 
		// 
		// 			                (d3p_min-k)/r < d3s_25 < (d3p_max-k)/r
		//          ((d2p_min - d2p_25)/dt - k)/r < d3s_25 < ((d2p_max - d2p_25)/dt - k)/r
		//                                          d3s_25 < (d2p_25*d2p_25/(2(dp_25-dp_max)) - k)/r
		// 
		// 其中 r = (1+g) * dp_ds_t25 = dp_ds_t25 + 3 * d2p_ds2_t25 * ds_t25 /2 * dt
		// 
		// 其中 不等式1 和 2 左右需要根据dp_ds_t25 的符号进行切换
		
		double zero_check = 1e-10;
		
		const double MAX_DS = 1.0;
		const double MIN_DS = std::min(0.005, param.target_ds);
		const double MAX_D2S = 10.0;
		const double MIN_D2S = -10.0;
		const double MAX_D3S = 10000.0;
		const double MIN_D3S = -10000.0;

		const double VEL_BOUND_RATIO = 0.99;
		const double ACC_BOUND_RATIO = 0.95;
		const double JERK_BOUND_RATIO = 0.95;

		double dt = param.dt;
		auto dim = param.dim;

		auto target_ds = param.target_ds;
		auto ds1 = param.ds1;
		auto ds2 = param.ds2;
		auto ds3 = param.ds3;

		auto p3 = param.p3;
		auto p2 = param.p2;
		auto p1 = param.p1;
		auto p0 = param.p0;
		
		auto dp_max = param.max_dp;
		auto dp_min = param.min_dp;
		auto d2p_max = param.max_d2p;
		auto d2p_min = param.min_d2p;
		auto d3p_max = param.max_d3p;
		auto d3p_min = param.min_d3p;

		auto d2s2 = (ds2 - ds1) / dt;
		auto d2s3 = (ds3 - ds2) / dt;
		auto d3s3 = (d2s3 - d2s2) / dt;

		double ds_t15 = ds2;
		double d2s_t15 = (d2s2 + d2s3) / 2;
		double d3s_t15 = d3s3;
		double ds_t25 = ds3;

		double d3s_min_1 = -1e10;
		double d3s_max_1 = 1e10;
		double d3s_min_2 = -1e10;
		double d3s_max_2 = 1e10;
		double d3s_min_3 = -1e10;
		double d3s_max_3 = 1e10;
		double d3s_min_4 = -1e10;
		double d3s_max_4 = 1e10;

		for (int i = 0; i < dim; ++i) {
			auto dp3 = (p3[i] - p2[i]) / dt;
			auto dp2 = (p2[i] - p1[i]) / dt;
			auto dp1 = (p1[i] - p0[i]) / dt;

			auto d2p3 = (dp3 - dp2) / dt;
			auto d2p2 = (dp2 - dp1) / dt;
			
			auto d3p3 = (d2p3 - d2p2) / dt;

			auto dp_t15 = dp2;
			auto d2p_t15 = (d2p2 + d2p3) / 2;
			auto d3p_t15 = d3p3;

			auto dp_ds_t15 = dp_t15 / ds_t15;
			auto d2p_ds2_t15 = (d2p_t15 - dp_ds_t15 * d2s_t15) / ds_t15 / ds_t15;
			auto d3p_ds3_t15 = (d3p_t15 - 3 * d2p_ds2_t15 * ds_t15 * d2s_t15 - dp_ds_t15 * d3s_t15) / ds_t15 / ds_t15 / ds_t15;

			auto s25_s15 = (ds2 + ds3) / 2 * dt;
			auto d3p_ds3_t25 = d3p_ds3_t15;
			auto d2p_ds2_t25 = d2p_ds2_t15 + d3p_ds3_t15 * s25_s15;
			auto dp_ds_t25 = dp_ds_t15 + d2p_ds2_t15 * s25_s15 + 0.5 * d3p_ds3_t15 * s25_s15 * s25_s15;

			double k = (dp_ds_t25 + (3 * d2p_ds2_t25 * ds_t25 * dt) / 2);
			double g = d3p_ds3_t25 * ds_t25 * ds_t25 * ds_t25 + 3 * d2p_ds2_t25 * ds3 * d2s3;

			if (std::abs(k) > zero_check) {
				auto d3s_l1 = (-dp_max[i] * VEL_BOUND_RATIO - dp3 - d2p3 * dt - g * dt * dt) / k / dt / dt;
				auto d3s_r1 = (dp_max[i] * VEL_BOUND_RATIO - dp3 - d2p3 * dt - g * dt * dt) / k / dt / dt;
				
				auto d3s_l2 = (-d2p_max[i] * ACC_BOUND_RATIO - d2p3 - g * dt) / k / dt;
				auto d3s_r2 = (d2p_max[i] * ACC_BOUND_RATIO - d2p3 - g * dt) / k / dt;

				auto d3s_l3 = (-d3p_max[i] * JERK_BOUND_RATIO - g) / k;
				auto d3s_r3 = (d3p_max[i] * JERK_BOUND_RATIO - g) / k;

				auto d2p_min2 = -std::sqrt(std::max(2 * (dp3 + dp_max[i] * VEL_BOUND_RATIO + d2p3 * dt) * d3p_max[i] * JERK_BOUND_RATIO, 0.0));
				auto d2p_max2 = std::sqrt(std::max(2 * (dp_max[i] * VEL_BOUND_RATIO - dp3 - d2p3 * dt) * d3p_max[i] * JERK_BOUND_RATIO, 0.0));
				auto d3s_l4 = (d2p_min2 - d2p3 - g * dt) / k / dt;
				auto d3s_r4 = (d2p_max2 - d2p3 - g * dt) / k / dt;

				if (k < 0) {
					std::swap(d3s_l1, d3s_r1);
					std::swap(d3s_l2, d3s_r2);
					std::swap(d3s_l3, d3s_r3);
					std::swap(d3s_l4, d3s_r4);
				}

				d3s_min_1 = std::max(d3s_l1, d3s_min_1);
				d3s_max_1 = std::min(d3s_r1, d3s_max_1);
				d3s_min_2 = std::max(d3s_l2, d3s_min_2);
				d3s_max_2 = std::min(d3s_r2, d3s_max_2);
				d3s_min_3 = std::max(d3s_l3, d3s_min_3);
				d3s_max_3 = std::min(d3s_r3, d3s_max_3);
				d3s_min_4 = std::max(d3s_l4, d3s_min_4);
				d3s_max_4 = std::min(d3s_r4, d3s_max_4);

			}
		}

		double d3s_min_12 = std::max({
			d3s_min_1,
			d3s_min_2,
			});
		double d3s_max_12 = std::min({
			d3s_max_1,
			d3s_max_2,
			});
		double d3s_min_124 = std::max({
			d3s_min_1,
			d3s_min_2,
			d3s_min_4,
			});
		double d3s_max_124 = std::min({
			d3s_max_1,
			d3s_max_2,
			d3s_max_4,
			});
		double d3s_min_all = std::max({
			d3s_min_1,
			d3s_min_2,
			d3s_min_3,
			d3s_min_4,
			});
		double d3s_max_all = std::min({
			d3s_max_1,
			d3s_max_2,
			d3s_max_3,
			d3s_max_4,
			});

		double next_ds, next_d2s, next_d3s{ 0.0 };

		double d3s_max, d3s_min;

		// 0. ds 过小、无法通过ds 判断出原始轨迹的曲率等
		if (ds3 <= zero_check || ds2 < zero_check || ds1 < zero_check) {
			aris::Size total_count;
			s_follow_x(ds3, d2s3, target_ds, MAX_D2S, MIN_D2S, MAX_D3S, MIN_D3S, dt, zero_check, next_ds, next_d2s, next_d3s, total_count);
			goto next_step;
		}

		// 1. 原始轨迹在起始点，可以直接加速到想要的 ds
		if (d3s_max_all >= 1e9 && d3s_min_all < -1e9 && ds3 > zero_check) {
			next_ds = ds2 = ds3 = target_ds;
			goto next_step;
		}

		// 2. 可以在COND 1234不超限的情况下，完成规划
		d3s_max = d3s_max_all;
		d3s_min = d3s_min_all;
		if (d3s_max > d3s_min) {
			bool is_avalable = false;
			if (ds3 > 0.5) {
				if (d2s3 > 0.0) {
					// diff_ds = 0.5* d3s_min * T^2
					// 
					// T = d2s / d3s_min
					// 
					// => diff_ds = 0.5* d2s^2 / d3s_min
					auto d3s_desired = ds3 < MAX_DS ? -d2s3 * d2s3 / ((MAX_DS - (ds3)) * 2.0) : 0.0;
					is_avalable = d3s_desired > d3s_min;

					//auto d2s_desired = std::sqrt(std::max(-d3s_min * (MAX_DS - (ds3 + d2s3 * dt + 0.5 * d3s_max * dt*dt))*2.0, 0.0));
					//d3s_max = std::min(d3s_max, (d2s_desired - d2s3) / dt);
				}
				else {
					is_avalable = true;
				}
			}
			else {
				if (d2s3 > 0.0) {
					is_avalable = true;
				}
				else {
					auto d3s_desired = ds3 > MIN_DS ? -d2s3 * d2s3 / ((MIN_DS - ds3) * 2.0) : 0.0;
					is_avalable = d3s_desired < d3s_max;

					//auto d2s_desired = -std::sqrt(std::max(-d3s_max * (MIN_DS - (ds3 + d2s3 * dt + 0.5 * d3s_min * dt * dt)) * 2.0, 0.0));
					//d3s_min = std::max(d3s_min, (d2s_desired - d2s3) / dt);
				}
			}
			if (is_avalable) {
				aris::Size total_count;
				s_follow_x(ds3, d2s3, target_ds, MAX_D2S, MIN_D2S, d3s_max, d3s_min, dt, zero_check, next_ds, next_d2s, next_d3s, total_count);
				goto next_step;
			}
		}

		// 3. 可以在COND 124不超限的情况下，完成规划
		d3s_max = d3s_max_124;
		d3s_min = d3s_min_124;
		if (d3s_max > d3s_min) {
			bool is_avalable = false;
			if (ds3 > 0.5) {
				if (d2s3 > 0.0) {
					auto d3s_desired = ds3 < MAX_DS ? -d2s3 * d2s3 / ((MAX_DS - ds3) * 2.0) : 0.0;
					is_avalable = d3s_desired > d3s_min;
					d3s_max = std::min(d3s_desired, d3s_max);
					//d3s_min = d3s_desired;
				}
				else {
					is_avalable = true;
				}
			}
			else {
				if (d2s3 > 0.0) {
					is_avalable = true;
				}
				else {
					auto d3s_desired = ds3 > MIN_DS ? -d2s3 * d2s3 / ((MIN_DS - ds3) * 2.0) : 0.0;
					is_avalable = d3s_desired < d3s_max;
					//d3s_max = d3s_desired;
					d3s_min = std::max(d3s_desired, d3s_min);
				}
			}
			if (is_avalable) {
				aris::Size total_count;
				s_follow_x(ds3, d2s3, target_ds, MAX_D2S, MIN_D2S, d3s_max, d3s_min, dt, zero_check, next_ds, next_d2s, next_d3s, total_count);
				goto next_step;
			}
		}

		// 4. 可以在COND 12不超限的情况下，完成规划
		d3s_max = d3s_max_12;
		d3s_min = d3s_min_12;
		if (d3s_max > d3s_min) {
			bool is_avalable = false;
			if (ds3 > 0.5) {
				if (d2s3 > 0.0) {
					auto d3s_desired = ds3 < MAX_DS ? -d2s3 * d2s3 / ((MAX_DS - ds3) * 2.0) : 0.0;
					is_avalable = d3s_desired > d3s_min;
					d3s_max = std::min(d3s_desired, d3s_max);
				}
				else {
					is_avalable = true;
				}
			}
			else {
				if (d2s3 > 0.0) {
					is_avalable = true;
				}
				else {
					auto d3s_desired = ds3 > MIN_DS ? -d2s3 * d2s3 / ((MIN_DS - ds3) * 2.0) : 0.0;
					is_avalable = d3s_desired < d3s_max;
					d3s_min = std::max(d3s_desired, d3s_min);
				}
			}
			if (is_avalable) {
				aris::Size total_count;
				s_follow_x(ds3, d2s3, target_ds, 1e9, -1e9, d3s_max, d3s_min, dt, zero_check, next_ds, next_d2s, next_d3s, total_count);
				goto next_step;
			}
		}

		// 5. 只考虑减速环节
		{
			aris::Size total_count;
			s_follow_x(ds3, d2s3, MIN_DS, MAX_D2S, MIN_D2S, MAX_D3S, MIN_D3S, dt, zero_check, next_ds, next_d2s, next_d3s, total_count);
		}






	next_step:

#ifdef ARIS_DEBUG_SINGULAR_PROCESSOR
		static int count_{ 0 };
		//std::cout << "count: " << count_++ << std::endl;

		if (count_ > 1995 && count_ < 2005) {
			std::cout << "bound:" << d3s_max_all << "  " << d3s_min_all << std::endl;
			std::cout << "next_d3s:" << next_d3s << std::endl;
			//aris::dynamic::dsp(1, 6, d3s_l1.data());
			//aris::dynamic::dsp(1, 6, d3s_r1.data());
			//aris::dynamic::dsp(1, 6, d3s_l2.data());
			//aris::dynamic::dsp(1, 6, d3s_r2.data());
			//aris::dynamic::dsp(1, 6, d3s_l3.data());
			//aris::dynamic::dsp(1, 6, d3s_r3.data());
			//aris::dynamic::dsp(1, 6, d3s_l4.data());
			//aris::dynamic::dsp(1, 6, d3s_r4.data());

			if (count_ == 180) {
				std::cout << "debug" << std::endl;
			}
			//return 0;
		}

#endif // ARIS_DEBUG_SINGULAR_PROCESSOR

		//next_d2s = d2s3 + next_d3s * dt;
		//next_ds = ds3 + next_d2s * dt;

		next_ds = std::min(next_ds, MAX_DS);
		next_ds = std::max(next_ds, std::min(MIN_DS, target_ds));

		ret.next_ds = next_ds;
		return 0;
	};

	auto s_smooth_curve2(const SmoothParam& param, SmoothRet& ret) -> int {

		///////////////////////////// PART 1 计算 d3s 的可选范围 ///////////////////////////////// 
		// 
		// 
		// 
		// 
		//% 以下考虑约束条件
		//% 【COND 1】经过 dt 时间后，速度不超过上下限
		//%  
		//%  dp_min < dp3 + (d2p3 + d3p25*dt)*dt < dp_max
		//%  =>
		//%  (dp_min - dp3 - d2p3*dt - g*dt*dt)/k/dt/dt 
		//%  <
		//%    d3s
		//%  <
		//%  (dp_max - dp3 - d2p3*dt - g*dt*dt)/k/dt/dt 
		//% 【COND 2】经过 dt 时间后，加速度不超过上下限
		//%
		//%  d2p_min < d2p3 + d3p_25 * dt < d2p_max
		//%
		//%  (d2p_min - d2p3 - g*dt)/k/dt
		//%
		//% 【COND 3】jerk 不超过上下限
		//%
		//%  d3p_min < d3p_25 < d3p_max
		//% 
		//% 【COND 4】jerk 不超过考虑速度与加速度的上下限
		//%
		//%  d2p_min2 < d2p3 + d3p_25 * dt < d2p_max2
		//%
		//%
		//% 【COND 4】推导如下：
		//%
		//%  将加速度减为0，所需时间：T = abs(d2p)/d3p_max
		//%  减为0时的速度：dp + T*d2p/2
		//%  dp_min < dp + T*d2p/2 < dp_max
		//%  =>
		//%  dp_min-dp < T*d2p/2 < dp_max-dp
		//%  2*(dp_min-dp)*d3p_max < sig(d2p)*d2p*d2p < 2*(dp_max-dp)*d3p_max
		//%  => 
		//%  d2p < 0时
		//%    d2p > -sqrt(2*(dp-dp_min)*d3p_max)
		//%  d2p > 0时
		//%    d2p < sqrt(2*(dp_max-dp)*d3p_max)
		//% 
		//%  考虑经过dt，则应有：
		//%  
		//%  d2p < 0时
		//%    d2p > -sqrt(2*(dp-dp_min-d2p_max*dt)*d3p_max)
		//%  d2p > 0时
		//%    d2p < sqrt(2*(dp_max-dp-d2p_max*dt)*d3p_max)
		// 
		// 应有以下等式：
		// dp_min  < dp  < dp_max
		// d2p_min < d2p < d2p_max
		// d3p_min < d3p < d3p_max
		//
		// dp  = dp_ds * ds
		// d2p = d2p_ds2 * ds^2 + dp_ds * d2s
		// d3p = d3p_ds3 * ds^3 + 2 * d2p_ds2 * ds * d2s + d2p_ds2 * ds * d2s + dp_ds * d3s
		//     = d3p_ds3 * ds^3 + 3 * d2p_ds2 * ds * d2s + dp_ds * d3s
		//
		// 带入上述不等式，应有：
		//                    dp_min/dp_ds < ds  < dp_max/dp_ds
		//  (d2p_max-d2p_ds2 * ds^2)/dp_ds < d2s < (d2p_max-d2p_ds2 * ds^2)/dp_ds
		//               (d3p_min-f)/dp_ds < d3s < (d3p_max-f)/dp_ds
		//
		// 其中 f = d3p_ds2 * ds^3 + 3 * d2p_ds * ds * d2s
		// 
		// 若 dp_ds < 0，则上式左右相反。
		//
		// 上式仅考虑了电机端的输入，没有考虑加速度将为 0 也需要时间。此时：
		//
		//       dp_min  <  dp + d2p * T + 0.5 * d3p * T^2  <  dp_max
		//                                  
		// 中间式子在 T = -d2p/d3p 时取到极值，大小为：
		//    
		//       dp - 0.5*d2p*d2p/d3p
		//
		// 若 d2p < 0 && dp_ds < 0，则应有 d3p > 0：
		//       dp_min < dp - 0.5*d2p*d2p/d3p
		// =>    d2p*d2p/d3p < 2*(dp-dp_min)
		// =>    d3p > d2p*d2p/(2(dp-dp_min))
		// =>    d3s < (d2p*d2p/(2(dp-dp_min)) - f)/dp_ds
		// 		
		// 若 d2p > 0 && dp_ds > 0，则应有 d3p > 0：
		//
		//       d3p < d2p*d2p/(2(dp_max-dp))
		// =>    d3s < (d2p*d2p/(2(dp-dp_max)) - f)/dp_ds
		//
		// 考虑加速度不超限，应有：
		// 
		//                           d2p_min  < d2p + d3p*dt < d2p_max
		// =>             (d2p_min - d2p)/dt  <      d3p     <  (d2p_max - d2p)/dt
		// =>  ((d2p_min - d2p)/dt - f)/dp_ds <      d3s     <  ((d2p_max - d2p)/dt - f)/dp_ds
		// 
		///////////////////////////// PART 2 计算其中的数据 ///////////////////////////////// 
		// t0        t1   t15   t2   t25   t3
		//
		// p0        p1         p2         p3
		//     dp1        dp2        dp3
		//          d2p2       d2p3
		//               d3p3
		//  
		// s0        s1         s2         s3
		//     ds1        ds2        ds3
		//          d2s2       d2s3
		//               d3s3
		//
		// 
		//
		// at point t=1.5:
		//
		// dp_ds_t15   = dp_t15/ds_t15
		// d2p_ds2_t15 = (d2p_t15 - dp_ds_t15 * d2s_t15)/ds_t15^2
		// d3p_ds3_t15 = (d3p_t15 - 3 * d2p_ds2_t15 * ds_t15 * d2s_t15 - dp_ds_t15 * d3s_t15)/ds_t15^3
		// 
		// at point t=2.5:
		// 
		// ds_t25 = ds3
		// d2s_t25 = d2s_t15 + (d3s_t15+d3s_25)/2*dt
		// 
		// d3p_ds3_t25 = d3p_ds3_t15
		// d2p_ds2_t25 = d2p_ds2_t15 + d3p_ds3_t15 * (s_25 - s_15)
		// dp_ds_t25   = dp_ds_t15 + d2p_ds2_t15 * (s_25 - s_15) + 0.5 * d3p_ds3_t15 * (s_25 - s_15)^2
		// 
		// f_25 = d3p_ds3_t25 * ds_t25^3 + 3 * d2p_ds2_t25 * ds_t25 * d2s_t25
		//      = d3p_ds3_t25 * ds_t25^3 + 3 * d2p_ds2_t25 * ds_t25 * (d2s_t15 + (d3s_t15+d3s_25)/2*dt)
		//      = d3p_ds3_t25 * ds_t25^3 + 3 * d2p_ds2_t25 * ds_t25 * (d2s15 + d3s_t15*dt/2) + g*dp_ds_t25* d3s_25
		//      = k + g*dp_ds_t25* d3s_25
		// 
		// 其中:
		//    k = d3p_ds3_t25 * ds_t25^3 + 3 * d2p_ds2_t25 * ds_t25 * (d2s15 + d3s_t15*dt/2)
		//    g = 3 * d2p_ds2_t25 * ds_t25 /2 * dt / dp_ds_t25
		// 
		// 根据以下3个不等式，可求出 d3s 的许可范围：
		//                 (d3p_min-f_25)/dp_ds_t25 < d3s_25 < (d3p_max-f_25)/dp_ds_t25
		// ((d2p_min - d2p_25)/dt - f_25)/dp_ds_t25 < d3s_25 < ((d2p_max - d2p_25)/dt - f_25)/dp_ds_t25
		//                                            d3s_25 < (d2p_25*d2p_25/(2(dp_25-dp_max)) - f_25)/dp_ds_t25
		// 
		// => 
		// 
		// 			                (d3p_min-k)/r < d3s_25 < (d3p_max-k)/r
		//          ((d2p_min - d2p_25)/dt - k)/r < d3s_25 < ((d2p_max - d2p_25)/dt - k)/r
		//                                          d3s_25 < (d2p_25*d2p_25/(2(dp_25-dp_max)) - k)/r
		// 
		// 其中 r = (1+g) * dp_ds_t25 = dp_ds_t25 + 3 * d2p_ds2_t25 * ds_t25 /2 * dt
		// 
		// 其中 不等式1 和 2 左右需要根据dp_ds_t25 的符号进行切换

		double zero_check = 1e-10;

		const double MAX_DS = std::max(param.max_ds, param.target_ds);
		const double MIN_DS = std::min(param.min_ds, param.target_ds);
		const double MAX_D2S = param.max_d2s;
		const double MIN_D2S = param.min_d2s;
		const double MAX_D3S = param.max_d3s;
		const double MIN_D3S = param.min_d3s;

		const double VEL_BOUND_RATIO = 1.0;
		const double ACC_BOUND_RATIO = 1.0;
		const double JERK_BOUND_RATIO = 1.0;

		double dt = param.dt;
		auto dim = param.dim;

		auto target_ds = param.target_ds;
		auto ds1 = param.ds1;
		auto ds2 = param.ds2;
		auto ds3 = param.ds3;

		auto p3 = param.p3;
		auto p2 = param.p2;
		auto p1 = param.p1;
		auto p0 = param.p0;

		auto p_max = param.max_p;
		auto p_min = param.min_p;
		auto dp_max = param.max_dp;
		auto dp_min = param.min_dp;
		auto d2p_max = param.max_d2p;
		auto d2p_min = param.min_d2p;
		auto d3p_max = param.max_d3p;
		auto d3p_min = param.min_d3p;

		auto d2s2 = (ds2 - ds1) / dt;
		auto d2s3 = (ds3 - ds2) / dt;
		auto d3s3 = (d2s3 - d2s2) / dt;

		double ds_t15 = ds2;
		double d2s_t15 = (d2s2 + d2s3) / 2;
		double d3s_t15 = d3s3;
		double ds_t25 = ds3;

		auto d3s_max_level_11 = 1.0;

		//% 【LEVEL 0】p不超边界，v,a,j超边界
		//%  -- COND 0.1 下一时刻的位置满足条件
		//%     lhs_01 = (p_min - e1)/f1
		//%     rhs_01 = (p_max - e1)/f1
		//%
		//% 【LEVEL 1】p,v不超边界，a,j超边界
		//%  -- COND 1.0 下一时刻的ds满足条件
		//%     lhs_10 = (ds_min - ds3 - d2s3*dt)/dt^2
		//%     rhs_10 = (ds_max - ds3 - d2s3*dt)/dt^2
		//%
		//%  -- COND 1.1 下一时刻的速度满足条件
		//%     lhs_11 = (dp_min - e2)/f2
		//%     rhs_11 = (dp_max - e2)/f2
		//% 
		//% 【LEVEL 2】p,v,a不超边界，j超边界
		//%  -- COND 2.0 下一时刻的d2s满足条件
		//%     lhs_20 = (d2s_min - d2s3)/dt
		//%     rhs_20 = (d2s_max - d2s3)/dt
		//%
		//%  -- COND 2.1 下一时刻的加速度满足条件 
		//%     lhs_21 = (d2p_min - e3)/f3
		//%     rhs_21 = (d2p_max - e3)/f3
		//%
		//%  -- COND 2.2 加速度不超的前提下，当前速度不能太快，否则未来位置可能超出边界
		//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%tbd
		//%
		//% 【LEVEL 3】p,v,a,j不超边界
		//%  -- COND 3.0 下一时刻的d3s满足条件
		//%     lhs_30 = d3s_min
		//%     rhs_30 = d3s_max
		//%
		//%  -- COND 3.1 d3s不超的前提下，当前d2s不能太快，否则未来d3s可能超出边界
		//%     value1 = ((d3s_min^2*dt^2)/16 + 2*d3s_min*ds3 - 2*d3s_min*ds_max + (d2s3*d3s_min*dt)/2)
		//%     value2 = ((d3s_max^2*dt^2)/16 + 2*d3s_max*ds3 - 2*d3s_max*ds_min + (d2s3*d3s_max*dt)/2)
		//%     lhs_31 = -sqrt(value1) + d3s_min*dt/4
		//%     rhs_31 =  sqrt(value2) + d3s_max*dt/4
		//%
		//%  -- COND 3.2 下一时刻的跃度满足条件
		//%     lhs_32 = (d3p_min - e4)/f4
		//%     rhs_32 = (d3p_max - e4)/f4
		//%
		//%  -- COND 3.3 跃度不超的前提下，当前加速度不能太快，否则未来速度可能超出边界
		//%     value1 = ((d3p_min^2*dt^2)/16 + 2*d3p_min*dp3 - 2*d3p_min*dp_max + (d2p3*d3p_min*dt)/2)
		//%     value2 = ((d3p_max^2*dt^2)/16 + 2*d3p_max*dp3 - 2*d3p_max*dp_min + (d2p3*d3p_max*dt)/2)
		//%     
		//%     lhs_33 = -sqrt(value1) + d3p_max*dt/4
		//%     rhs_33 =  sqrt(value2) + d3p_min*dt/4
		//%
		//%  -- COND 3.4 跃度不超的前提下，当前速度不能太快，否则未来位置可能超出边界
		//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%tbd
		
		double level31_value1 = ((MIN_D3S * MIN_D3S * dt * dt) / 16 + 2 * MIN_D3S * ds3 - 2 * MIN_D3S * MAX_DS + (d2s3 * MIN_D3S * dt) / 2);
		double level31_value2 = ((MAX_D3S * MAX_D3S * dt * dt) / 16 + 2 * MAX_D3S * ds3 - 2 * MAX_D3S * MIN_DS + (d2s3 * MAX_D3S * dt) / 2);
		double 
			lhs01{ -1e10 }, rhs01{ 1e10 },
			lhs10{ (MIN_DS - ds3 - d2s3 * dt)/dt/dt }, rhs10{ (MAX_DS - ds3 - d2s3 * dt)/dt/dt },
			lhs11{ -1e10 }, rhs11{ 1e10 },
			lhs20{ (MIN_D2S - d2s3) / dt }, rhs20{ (MAX_D2S - d2s3) / dt },
			lhs205{ lhs10 }, rhs205{ rhs10 },
			lhs21{ -1e10 }, rhs21{ 1e10 },
			lhs22{ -1e10 }, rhs22{ 1e10 },
			lhs23{ -1e10 }, rhs23{ 1e10 },
			lhs24{ -1e10 }, rhs24{ 1e10 },
			lhs30{ MIN_D3S }, rhs30{ MAX_D3S },
			lhs31{ -std::sqrt(std::max(0.0, level31_value1)) + MIN_D3S*dt/4 }, rhs31{ std::sqrt(std::max(0.0, level31_value2)) + MAX_D3S * dt / 4 },
			lhs32{ -1e10 }, rhs32{ 1e10 },
			lhs33{ -1e10 }, rhs33{ 1e10 },
			lhs34{ -1e10 }, rhs34{ 1e10 }
		;
		double max_k = 0.0;
		for (int i = 0; i < dim; ++i) {
			auto dp3 = (p3[i] - p2[i]) / dt;
			auto dp2 = (p2[i] - p1[i]) / dt;
			auto dp1 = (p1[i] - p0[i]) / dt;

			auto d2p3 = (dp3 - dp2) / dt;
			auto d2p2 = (dp2 - dp1) / dt;

			auto d3p3 = (d2p3 - d2p2) / dt;

			auto dp_t15 = dp2;
			auto d2p_t15 = (d2p2 + d2p3) / 2;
			auto d3p_t15 = d3p3;

			auto dp_ds_t15 = dp_t15 / ds_t15;
			auto d2p_ds2_t15 = (d2p_t15 - dp_ds_t15 * d2s_t15) / ds_t15 / ds_t15;
			auto d3p_ds3_t15 = (d3p_t15 - 3 * d2p_ds2_t15 * ds_t15 * d2s_t15 - dp_ds_t15 * d3s_t15) / ds_t15 / ds_t15 / ds_t15;

			auto s25_s15 = (ds2 + ds3) / 2 * dt;
			auto d3p_ds3_t25 = d3p_ds3_t15;
			auto d2p_ds2_t25 = d2p_ds2_t15 + d3p_ds3_t15 * s25_s15;
			auto dp_ds_t25 = dp_ds_t15 + d2p_ds2_t15 * s25_s15 + 0.5 * d3p_ds3_t15 * s25_s15 * s25_s15;

			auto k = (dp_ds_t25 + (3 * d2p_ds2_t25 * ds_t25 * dt) / 2);
			auto g = d3p_ds3_t25 * ds_t25 * ds_t25 * ds_t25 + 3 * d2p_ds2_t25 * ds3 * d2s3;

			auto f1 = k * dt * dt * dt;
			auto f2 = k * dt * dt;
			auto f3 = k * dt;
			auto f4 = k;
			auto e1 = p3[i] + dp3 * dt + d2p3 * dt * dt + g * dt * dt * dt;
			auto e2 = dp3 + d2p3 * dt + g * dt * dt;
			auto e3 = d2p3 + g * dt;
			auto e4 = g;


			//std::cout << "d3p_ds3_t25:" << d3p_ds3_t25 << "  ds_t25:" << ds_t25 << "  d2p_ds2_t25:" << d2p_ds2_t25 << "  d2s3:" << d2s3 << std::endl;
			//std::cout << "d3p_ds3_t15:" << d3p_ds3_t15 << "  d3p_t15:" << d3p_t15 << "  d2p_ds2_t15:" << d2p_ds2_t15 << "  ds_t15:" << ds_t15 << "  d2s_t15:" << d2s_t15 << "  dp_ds_t15:" << dp_ds_t15 << std::endl;
			//std::cout << "d2p_ds2_t15:" << d2p_ds2_t15 << "  d2p_t15:" << d2p_t15 << "  dp_ds_t15:" << dp_ds_t15 << "  d2s_t15:" << d2s_t15 << "  ds_t15:" << ds_t15 << std::endl;

			if (std::abs(k) > zero_check) {
				auto lhs01_local = (p_min[i] - e1) / f2;
				auto rhs01_local = (p_max[i] - e2) / f2;

				auto lhs11_local = (dp_min[i] * VEL_BOUND_RATIO - e2) / f2;
				auto rhs11_local = (dp_max[i] * VEL_BOUND_RATIO - e2) / f2;

				auto lhs21_local = (d2p_min[i] * ACC_BOUND_RATIO - e3) / f3;
				auto rhs21_local = (d2p_max[i] * ACC_BOUND_RATIO - e3) / f3;

				auto value2 = ((d2p_min[i] * d2p_min[i] * dt * dt) / 16 + 2 * d2p_min[i] * p3[i] - 2 * d2p_min[i] * p_max[i] + (dp3 * d2p_min[i] * dt) / 2);
				auto dp_max3 = std::sqrt(std::max(value2, 0.0)) + d2p_min[i] * dt / 4;
				auto value1 = ((d2p_max[i] * d2p_max[i] * dt * dt) / 16 + 2 * d2p_max[i] * p3[i] - 2 * d2p_max[i] * p_min[i] + (dp3 * d2p_max[i] * dt) / 2);
				auto dp_min3 = -std::sqrt(std::max(value1, 0.0)) + d2p_max[i] * dt / 4;
				auto lhs22_local = (dp_min3 - e2) / f2;
				auto rhs22_local = (dp_max3 - e2) / f2;

				auto lhs32_local = (d3p_min[i] * JERK_BOUND_RATIO - e4) / f4;
				auto rhs32_local = (d3p_max[i] * JERK_BOUND_RATIO - e4) / f4;

				value2 = ((d3p_min[i] * d3p_min[i] * dt * dt) / 16 + 2 * d3p_min[i] * dp3 - 2 * d3p_min[i] * dp_max[i] + (d2p3 * d3p_min[i] * dt) / 2);
				auto d2p_max3 = std::sqrt(std::max(value2, 0.0)) + d3p_min[i] * dt / 4;
				value1 = ((d3p_max[i] * d3p_max[i] * dt * dt) / 16 + 2 * d3p_max[i] * dp3 - 2 * d3p_max[i] * dp_min[i] + (d2p3 * d3p_max[i] * dt) / 2);
				auto d2p_min3 = -std::sqrt(std::max(value1, 0.0)) + d3p_max[i] * dt / 4;
				auto lhs33_local = (d2p_min3 - e3) / f3;
				auto rhs33_local = (d2p_max3 - e3) / f3;

				if (k < 0) {
					std::swap(lhs11_local, rhs11_local);
					std::swap(lhs21_local, rhs21_local);
					std::swap(lhs22_local, rhs22_local);
					std::swap(lhs32_local, rhs32_local);
					std::swap(lhs33_local, rhs33_local);
				}

				lhs11 = std::max(lhs11_local, lhs11);
				rhs11 = std::min(rhs11_local, rhs11);
				lhs21 = std::max(lhs21_local, lhs21);
				rhs21 = std::min(rhs21_local, rhs21);
				lhs22 = std::max(lhs22_local, lhs22);
				rhs22 = std::min(rhs22_local, rhs22);
				lhs32 = std::max(lhs32_local, lhs32);
				rhs32 = std::min(rhs32_local, rhs32);
				lhs33 = std::max(lhs33_local, lhs33);
				rhs33 = std::min(rhs33_local, rhs33);


				////////////////////////////

				
				auto g2 = d3p_ds3_t25 * ds_t25 * ds_t25 * ds_t25 + 2 * d2p_ds2_t25 * ds3 * d2s3;
				auto k2 = d2p_ds2_t25 * ds_t25 * dt;

				auto F3 = d2p3 - (dp2 + dp3) / (ds2 + ds3) * d2s3;
				auto f5 = k2 * dt;
				auto e5 = F3 + g2 * dt;

				double f6 = k2;
				double e6 = g2;

				
				//std::cout << "F3:" << F3 <<"  f5:" << f5 <<"  e5:" << e5 <<"  g2:" << g2 << std::endl;

				auto lhs205_local = (d2p_min[i] - e5) / f5;
				auto rhs205_local = (d2p_max[i] - e5) / f5;
				
				double d3s_min_guess = std::max({ lhs10, lhs11_local, lhs20, lhs21_local,lhs22_local,lhs30,lhs31,lhs32_local,lhs33_local });
				double d3s_max_guess = std::min({ rhs11_local, rhs20, rhs21_local,rhs22_local,rhs30,rhs31,rhs32_local,rhs33_local });
				double d2F_max = 5*d3p_ds3_t25*ds_t25*ds_t25*d2s3 + 2*d2p_ds2_t25*d2s3*d2s3 + 2*d2p_ds2_t25*ds_t25*d3s_max_guess;
				double d2F_min = 5*d3p_ds3_t25*ds_t25*ds_t25*d2s3 + 2*d2p_ds2_t25*d2s3*d2s3 + 2*d2p_ds2_t25*ds_t25*d3s_min_guess;

				if (d2F_max < d2F_min)
					std::swap(d2F_max, d2F_min);

				double dF3 = d3p_ds3_t25*ds_t25*ds_t25*ds_t25 + 2*d2p_ds2_t25*ds_t25*d2s3;

				value2 = ((d2F_min * d2F_min * dt * dt) / 16 + 2 * d2F_min * F3 - 2 * d2F_min * d2p_max[i] + (dF3 * d2F_min * dt) / 2);
				double dF_max = std::sqrt(std::max(value2, 0.0)) + d2F_min * dt / 4;
				value1 = ((d2F_max * d2F_max * dt * dt) / 16 + 2 * d2F_max * F3 - 2 * d2F_max * d2p_min[i] + (dF3 * d2F_max * dt) / 2);
				double dF_min = -std::sqrt(std::max(value1, 0.0)) + d2F_max * dt / 4;

				auto lhs23_local = (d3p_min[i] - e6) / f6;
				auto rhs23_local = (d3p_max[i] - e6) / f6;

				auto lhs24_local = (dF_min - e6) / f6;
				auto rhs24_local = (dF_max - e6) / f6;

				if (k2 < 0) {
					std::swap(lhs23_local, rhs23_local);
					std::swap(lhs24_local, rhs24_local);
				}

				double d2F3 = 5 * d3p_ds3_t25 * ds_t25 * ds_t25 * d2s3 + 2 * d2p_ds2_t25 * d2s3 * d2s3 + 2 * d2p_ds2_t25 * ds_t25 * d3s3;
				//std::cout << "F3:" << F3 << "  dF3:" << dF3 << "  d2F3:" << d2F3 << std::endl << std::endl;
				
				//////////////////////////////////////////
				auto ds_max_by_F = std::sqrt(std::abs(std::max(std::abs(d2p_max[i]), std::abs(d2p_min[i])) / d2p_ds2_t25));

				lhs205_local = (MIN_DS - ds3 - d2s3 * dt) / dt / dt;
				rhs205_local = (ds_max_by_F - ds3 - d2s3 * dt) / dt / dt;
				if (rhs205_local < lhs205_local)
					std::swap(lhs205_local, rhs205_local);

				//double level31_value1 = ((MIN_D3S * MIN_D3S * dt * dt) / 16 + 2 * MIN_D3S * ds3 - 2 * MIN_D3S * MAX_DS + (d2s3 * MIN_D3S * dt) / 2);
				//double level31_value2 = ((MAX_D3S * MAX_D3S * dt * dt) / 16 + 2 * MAX_D3S * ds3 - 2 * MAX_D3S * MIN_DS + (d2s3 * MAX_D3S * dt) / 2);
				value2 = ((d3s_min_guess * d3s_min_guess * dt * dt) / 16 + 2 * d3s_min_guess * ds3 - 2 * d3s_min_guess * rhs205_local + (d2s3 * d3s_min_guess * dt) / 2);
				lhs23_local = -std::sqrt(std::max(value2, 0.0)) + d3s_min_guess * dt / 4;
				value1 = ((d3s_max_guess * d3s_max_guess * dt * dt) / 16 + 2 * d3s_max_guess * ds3 - 2 * d3s_max_guess * lhs205_local + (d2s3 * d3s_max_guess * dt) / 2);
				rhs23_local = std::sqrt(std::max(value1, 0.0)) + d3s_max_guess * dt / 4;




				rhs24_local = rhs23_local;
				lhs24_local = lhs23_local;
				std::cout << lhs205_local << "  " << rhs205_local << std::endl;
				//////////////////////////////////////////

				rhs205 = std::min(rhs205_local, rhs205);
				lhs205 = std::max(lhs205_local, lhs205);
				rhs23 = std::min(rhs23_local, rhs23);
				lhs23 = std::max(lhs23_local, lhs23);
				rhs24 = std::min(rhs24_local, rhs24);
				lhs24 = std::max(lhs24_local, lhs24);
			}

			max_k = std::max(std::abs(k), max_k);
		}

		double lhs_level_0 = std::max({ lhs01 });
		double rhs_level_0 = std::min({ rhs01 });
		double lhs_level_1 = std::max({ lhs_level_0, lhs10, lhs11 });
		double rhs_level_1 = std::min({ rhs_level_0, rhs10, rhs11 });
		double lhs_level_2 = std::max({ lhs_level_1, lhs20, lhs21, lhs22 });
		double rhs_level_2 = std::min({ rhs_level_1, rhs20, rhs21, rhs22 });
		double lhs_level_3 = std::max({ lhs_level_2, lhs30, lhs31, lhs32, lhs33 });
		double rhs_level_3 = std::min({ rhs_level_2, rhs30, rhs31, rhs32, rhs33 });

		const int CONS_NUM = 14;

		//                        1       2      3      4       5      6      7      8      9     10     11     12     13     14
		double lhs1[CONS_NUM]{ lhs01, lhs10, lhs11, lhs20, lhs205, lhs21, lhs22, lhs23, lhs24, lhs30, lhs31, lhs32, lhs33, lhs34 };
		double rhs1[CONS_NUM]{ rhs01, rhs10, rhs11, rhs20, rhs205, rhs21, rhs22, rhs23, rhs24, rhs30, rhs31, rhs32, rhs33, rhs34 };
		double lhs[CONS_NUM], rhs[CONS_NUM];
		std::copy_n(lhs1, CONS_NUM, lhs);
		std::copy_n(rhs1, CONS_NUM, rhs);

		for (int i = 1; i < CONS_NUM; ++i) {
			lhs[i] = std::max(lhs[i], lhs[i - 1]);
			rhs[i] = std::min(rhs[i], rhs[i - 1]);
		}


		double next_ds, next_d2s, next_d3s{ 0.0 };
		// 0. ds 过小、无法通过 ds 判断出原始轨迹的曲率等
		if (ds3 <= zero_check || ds2 < zero_check || ds1 < zero_check) {
			aris::Size total_count;
			s_follow_x(ds3, d2s3, target_ds, MAX_D2S, MIN_D2S, MAX_D3S, MIN_D3S, dt, zero_check, next_ds, next_d2s, next_d3s, total_count);
			next_ds = std::min(next_ds, std::max(MAX_DS, target_ds));
			next_ds = std::max(next_ds, std::min(MIN_DS, target_ds));

			ret.next_ds = next_ds;
			ret.state = 0;
			return 0;
		}

		// 1. 原始轨迹在起始点，可以直接加速到想要的 ds
		if (max_k < 1e-9 && ds3 > zero_check) {
			next_ds = ds2 = ds3 = target_ds;
			next_ds = std::min(next_ds, std::max(MAX_DS, target_ds));
			next_ds = std::max(next_ds, std::min(MIN_DS, target_ds));

			ret.next_ds = next_ds;
			ret.state = 1;
			return 0;
		}


		if (rhs[CONS_NUM-1] > lhs[CONS_NUM-1]) {
			// 在 ds 的边界处，因为边界限制了 d3s 减速（或加速），导致ds无法加速（或减速）
			// 去除 lhs10 和 205
			double d3s_min_modify = std::max({ lhs01, lhs11, lhs20, lhs21, lhs22, lhs30, lhs31, lhs32, lhs33, lhs34 });
			double d3s_max_modify = std::min({ rhs01, rhs11, rhs20, rhs21, rhs22, rhs30, rhs31, rhs32, rhs33, rhs34 });

			if (ds3 < MIN_DS + 0.5 * rhs[CONS_NUM - 1] * dt * dt && d2s3 < rhs[CONS_NUM - 1] * dt && target_ds > ds3) {
				aris::Size total_count;
				s_follow_x(ds3, d2s3, target_ds, MAX_D2S, MIN_D2S, rhs[CONS_NUM - 1], d3s_min_modify, dt, zero_check, next_ds, next_d2s, next_d3s, total_count);
				next_ds = std::min(next_ds, std::max(MAX_DS, target_ds));
				next_ds = std::max(next_ds, std::min(MIN_DS, target_ds));
				ret.next_ds = next_ds;
				ret.state = CONS_NUM;
				return 0;
			}
			
			aris::Size total_count;
			s_follow_x(ds3, d2s3, target_ds, MAX_D2S, MIN_D2S, rhs[CONS_NUM-1], lhs[CONS_NUM-1], dt, zero_check, next_ds, next_d2s, next_d3s, total_count);
			next_ds = std::min(next_ds, std::max(MAX_DS, target_ds));
			next_ds = std::max(next_ds, std::min(MIN_DS, target_ds));

			ret.next_ds = next_ds;
			ret.state = CONS_NUM;
			return 0;
		}

		for (int i = CONS_NUM-2; i >= 0; --i) {
			
			if (rhs[i] > lhs[i]) {
				/*aris::Size total_count;
				s_follow_x(ds3, d2s3, target_ds, MAX_D2S, MIN_D2S, std::max(rhs[i+1], lhs[i+1]), std::max(lhs[i+1], rhs[i+1]), dt, zero_check, next_ds, next_d2s, next_d3s, total_count);
				next_ds = std::min(next_ds, std::max(MAX_DS, target_ds));
				next_ds = std::max(next_ds, std::min(MIN_DS, target_ds));*/
				next_d3s = std::max(lhs[i], std::min(lhs[i + 1], rhs[i + 1]));
				next_d2s = d2s3 + next_d3s * dt;

				std::cout << lhs[i] << "  " << rhs[i] << "   " << lhs[i + 1] << "  " << rhs[i + 1] << std::endl;
				ret.next_ds = ds3 + next_d2s * dt;
				ret.state = i + 1;
				return 0;
			}
			
		}
		return -1;

		// 2. 可以在 LEVEL-3 条件下完成规划
		if (rhs_level_3 > lhs_level_3) {
			aris::Size total_count;
			s_follow_x(ds3, d2s3, target_ds, MAX_D2S, MIN_D2S, rhs_level_3, lhs_level_3, dt, zero_check, next_ds, next_d2s, next_d3s, total_count);
			next_ds = std::min(next_ds, std::max(MAX_DS, target_ds));
			next_ds = std::max(next_ds, std::min(MIN_DS, target_ds));

			ret.next_ds = next_ds;
			ret.state = 2;
			return 0;
		}

		// 3. 可以在 LEVEL-2 条件下完成规划
		if (rhs_level_2 > lhs_level_2) {
			aris::Size total_count;
			s_follow_x(ds3, d2s3, target_ds, MAX_D2S, MIN_D2S, rhs_level_2, lhs_level_2, dt, zero_check, next_ds, next_d2s, next_d3s, total_count);
			next_ds = std::min(next_ds, std::max(MAX_DS, target_ds));
			next_ds = std::max(next_ds, std::min(MIN_DS, target_ds));

			ret.next_ds = next_ds;
			ret.state = 3;
			return 0;
		}

		// 4. 可以在 LEVEL-1 条件下完成规划
		if (rhs_level_1 > lhs_level_1) {
			aris::Size total_count;
			s_follow_x(ds3, d2s3, target_ds, MAX_D2S, MIN_D2S, rhs_level_1, lhs_level_1, dt, zero_check, next_ds, next_d2s, next_d3s, total_count);
			next_ds = std::min(next_ds, std::max(MAX_DS, target_ds));
			next_ds = std::max(next_ds, std::min(MIN_DS, target_ds));

			ret.next_ds = next_ds;
			ret.state = 4;
			return 0;
		}

		// 5. 可以在 LEVEL-0 条件下完成规划
		{
			aris::Size total_count;
			s_follow_x(ds3, d2s3, MIN_DS, MAX_D2S, MIN_D2S, MAX_D3S, MIN_D3S, dt, zero_check, next_ds, next_d2s, next_d3s, total_count);
			next_ds = std::min(next_ds, std::max(MAX_DS, target_ds));
			next_ds = std::max(next_ds, std::min(MIN_DS, target_ds));

			ret.next_ds = next_ds;
			ret.state = 5;
			return 0;

		}
	
		return 0;
	};

	auto s_smooth_curve3(const SmoothParam& param, SmoothRet& ret) -> int {

		///////////////////////////// PART 1 计算 d3s 的可选范围 ///////////////////////////////// 
		// 
		// 
		// 
		// 
		//% 以下考虑约束条件
		//% 【COND 1】经过 dt 时间后，速度不超过上下限
		//%  
		//%  dp_min < dp3 + (d2p3 + d3p25*dt)*dt < dp_max
		//%  =>
		//%  (dp_min - dp3 - d2p3*dt - g*dt*dt)/k/dt/dt 
		//%  <
		//%    d3s
		//%  <
		//%  (dp_max - dp3 - d2p3*dt - g*dt*dt)/k/dt/dt 
		//% 【COND 2】经过 dt 时间后，加速度不超过上下限
		//%
		//%  d2p_min < d2p3 + d3p_25 * dt < d2p_max
		//%
		//%  (d2p_min - d2p3 - g*dt)/k/dt
		//%
		//% 【COND 3】jerk 不超过上下限
		//%
		//%  d3p_min < d3p_25 < d3p_max
		//% 
		//% 【COND 4】jerk 不超过考虑速度与加速度的上下限
		//%
		//%  d2p_min2 < d2p3 + d3p_25 * dt < d2p_max2
		//%
		//%
		//% 【COND 4】推导如下：
		//%
		//%  将加速度减为0，所需时间：T = abs(d2p)/d3p_max
		//%  减为0时的速度：dp + T*d2p/2
		//%  dp_min < dp + T*d2p/2 < dp_max
		//%  =>
		//%  dp_min-dp < T*d2p/2 < dp_max-dp
		//%  2*(dp_min-dp)*d3p_max < sig(d2p)*d2p*d2p < 2*(dp_max-dp)*d3p_max
		//%  => 
		//%  d2p < 0时
		//%    d2p > -sqrt(2*(dp-dp_min)*d3p_max)
		//%  d2p > 0时
		//%    d2p < sqrt(2*(dp_max-dp)*d3p_max)
		//% 
		//%  考虑经过dt，则应有：
		//%  
		//%  d2p < 0时
		//%    d2p > -sqrt(2*(dp-dp_min-d2p_max*dt)*d3p_max)
		//%  d2p > 0时
		//%    d2p < sqrt(2*(dp_max-dp-d2p_max*dt)*d3p_max)
		// 
		// 应有以下等式：
		// dp_min  < dp  < dp_max
		// d2p_min < d2p < d2p_max
		// d3p_min < d3p < d3p_max
		//
		// dp  = dp_ds * ds
		// d2p = d2p_ds2 * ds^2 + dp_ds * d2s
		// d3p = d3p_ds3 * ds^3 + 2 * d2p_ds2 * ds * d2s + d2p_ds2 * ds * d2s + dp_ds * d3s
		//     = d3p_ds3 * ds^3 + 3 * d2p_ds2 * ds * d2s + dp_ds * d3s
		//
		// 带入上述不等式，应有：
		//                    dp_min/dp_ds < ds  < dp_max/dp_ds
		//  (d2p_max-d2p_ds2 * ds^2)/dp_ds < d2s < (d2p_max-d2p_ds2 * ds^2)/dp_ds
		//               (d3p_min-f)/dp_ds < d3s < (d3p_max-f)/dp_ds
		//
		// 其中 f = d3p_ds2 * ds^3 + 3 * d2p_ds * ds * d2s
		// 
		// 若 dp_ds < 0，则上式左右相反。
		//
		// 上式仅考虑了电机端的输入，没有考虑加速度将为 0 也需要时间。此时：
		//
		//       dp_min  <  dp + d2p * T + 0.5 * d3p * T^2  <  dp_max
		//                                  
		// 中间式子在 T = -d2p/d3p 时取到极值，大小为：
		//    
		//       dp - 0.5*d2p*d2p/d3p
		//
		// 若 d2p < 0 && dp_ds < 0，则应有 d3p > 0：
		//       dp_min < dp - 0.5*d2p*d2p/d3p
		// =>    d2p*d2p/d3p < 2*(dp-dp_min)
		// =>    d3p > d2p*d2p/(2(dp-dp_min))
		// =>    d3s < (d2p*d2p/(2(dp-dp_min)) - f)/dp_ds
		// 		
		// 若 d2p > 0 && dp_ds > 0，则应有 d3p > 0：
		//
		//       d3p < d2p*d2p/(2(dp_max-dp))
		// =>    d3s < (d2p*d2p/(2(dp-dp_max)) - f)/dp_ds
		//
		// 考虑加速度不超限，应有：
		// 
		//                           d2p_min  < d2p + d3p*dt < d2p_max
		// =>             (d2p_min - d2p)/dt  <      d3p     <  (d2p_max - d2p)/dt
		// =>  ((d2p_min - d2p)/dt - f)/dp_ds <      d3s     <  ((d2p_max - d2p)/dt - f)/dp_ds
		// 
		///////////////////////////// PART 2 计算其中的数据 ///////////////////////////////// 
		// t0        t1   t15   t2   t25   t3
		//
		// p0        p1         p2         p3
		//     dp1        dp2        dp3
		//          d2p2       d2p3
		//               d3p3
		//  
		// s0        s1         s2         s3
		//     ds1        ds2        ds3
		//          d2s2       d2s3
		//               d3s3
		//
		// 
		//
		// at point t=1.5:
		//
		// dp_ds_t15   = dp_t15/ds_t15
		// d2p_ds2_t15 = (d2p_t15 - dp_ds_t15 * d2s_t15)/ds_t15^2
		// d3p_ds3_t15 = (d3p_t15 - 3 * d2p_ds2_t15 * ds_t15 * d2s_t15 - dp_ds_t15 * d3s_t15)/ds_t15^3
		// 
		// at point t=2.5:
		// 
		// ds_t25 = ds3
		// d2s_t25 = d2s_t15 + (d3s_t15+d3s_25)/2*dt
		// 
		// d3p_ds3_t25 = d3p_ds3_t15
		// d2p_ds2_t25 = d2p_ds2_t15 + d3p_ds3_t15 * (s_25 - s_15)
		// dp_ds_t25   = dp_ds_t15 + d2p_ds2_t15 * (s_25 - s_15) + 0.5 * d3p_ds3_t15 * (s_25 - s_15)^2
		// 
		// f_25 = d3p_ds3_t25 * ds_t25^3 + 3 * d2p_ds2_t25 * ds_t25 * d2s_t25
		//      = d3p_ds3_t25 * ds_t25^3 + 3 * d2p_ds2_t25 * ds_t25 * (d2s_t15 + (d3s_t15+d3s_25)/2*dt)
		//      = d3p_ds3_t25 * ds_t25^3 + 3 * d2p_ds2_t25 * ds_t25 * (d2s15 + d3s_t15*dt/2) + g*dp_ds_t25* d3s_25
		//      = k + g*dp_ds_t25* d3s_25
		// 
		// 其中:
		//    k = d3p_ds3_t25 * ds_t25^3 + 3 * d2p_ds2_t25 * ds_t25 * (d2s15 + d3s_t15*dt/2)
		//    g = 3 * d2p_ds2_t25 * ds_t25 /2 * dt / dp_ds_t25
		// 
		// 根据以下3个不等式，可求出 d3s 的许可范围：
		//                 (d3p_min-f_25)/dp_ds_t25 < d3s_25 < (d3p_max-f_25)/dp_ds_t25
		// ((d2p_min - d2p_25)/dt - f_25)/dp_ds_t25 < d3s_25 < ((d2p_max - d2p_25)/dt - f_25)/dp_ds_t25
		//                                            d3s_25 < (d2p_25*d2p_25/(2(dp_25-dp_max)) - f_25)/dp_ds_t25
		// 
		// => 
		// 
		// 			                (d3p_min-k)/r < d3s_25 < (d3p_max-k)/r
		//          ((d2p_min - d2p_25)/dt - k)/r < d3s_25 < ((d2p_max - d2p_25)/dt - k)/r
		//                                          d3s_25 < (d2p_25*d2p_25/(2(dp_25-dp_max)) - k)/r
		// 
		// 其中 r = (1+g) * dp_ds_t25 = dp_ds_t25 + 3 * d2p_ds2_t25 * ds_t25 /2 * dt
		// 
		// 其中 不等式1 和 2 左右需要根据dp_ds_t25 的符号进行切换
		
		double zero_check = 1e-10;

		const double MAX_DS = std::max(param.max_ds, param.target_ds);
		const double MIN_DS = std::min(param.min_ds, param.target_ds);
		const double MAX_D2S = param.max_d2s;
		const double MIN_D2S = param.min_d2s;
		const double MAX_D3S = param.max_d3s;
		const double MIN_D3S = param.min_d3s;

		double dt = param.dt;
		auto dim = param.dim;

		auto target_ds = param.target_ds;
		auto ds1 = param.ds1;
		auto ds2 = param.ds2;
		auto ds3 = param.ds3;

		auto p3 = param.p3;
		auto p2 = param.p2;
		auto p1 = param.p1;
		auto p0 = param.p0;

		auto p_max = param.max_p;
		auto p_min = param.min_p;
		auto dp_max = param.max_dp;
		auto dp_min = param.min_dp;
		auto d2p_max = param.max_d2p;
		auto d2p_min = param.min_d2p;
		auto d3p_max = param.max_d3p;
		auto d3p_min = param.min_d3p;

		auto d2s2 = (ds2 - ds1) / dt;
		auto d2s3 = (ds3 - ds2) / dt;
		auto d3s3 = (d2s3 - d2s2) / dt;

		double ds_t15 = ds2;
		double d2s_t15 = (d2s2 + d2s3) / 2;
		double d3s_t15 = d3s3;
		double ds_t25 = ds3;

		auto d3s_max_level_11 = 1.0;



		//% 【LEVEL 0】p不超边界，v,a,j超边界
		//%  -- COND 0.1 下一时刻的位置满足条件
		//%     lhs_01 = (p_min - e1)/f1
		//%     rhs_01 = (p_max - e1)/f1
		//%
		//% 【LEVEL 1】p,v不超边界，a,j超边界
		//%  -- COND 1.0 下一时刻的ds满足条件
		//%     lhs_10 = (ds_min - ds3 - d2s3*dt)/dt^2
		//%     rhs_10 = (ds_max - ds3 - d2s3*dt)/dt^2
		//%
		//%  -- COND 1.1 下一时刻的速度满足条件
		//%     lhs_11 = (dp_min - e2)/f2
		//%     rhs_11 = (dp_max - e2)/f2
		//% 
		//% 【LEVEL 2】p,v,a不超边界，j超边界
		//%  -- COND 2.0 下一时刻的d2s满足条件
		//%     lhs_20 = (d2s_min - d2s3)/dt
		//%     rhs_20 = (d2s_max - d2s3)/dt
		//%
		//%  -- COND 2.1 下一时刻的加速度满足条件 
		//%     lhs_21 = (d2p_min - e3)/f3
		//%     rhs_21 = (d2p_max - e3)/f3
		//%
		//%  -- COND 2.2 加速度不超的前提下，当前速度不能太快，否则未来位置可能超出边界
		//%     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%tbd
		//%
		//% 【LEVEL 3】p,v,a,j不超边界
		//%  -- COND 3.0 下一时刻的d3s满足条件
		//%     lhs_30 = d3s_min
		//%     rhs_30 = d3s_max
		//%
		//%  -- COND 3.1 d3s不超的前提下，当前d2s不能太快，否则未来d3s可能超出边界
		//%     value1 = ((d3s_min^2*dt^2)/16 + 2*d3s_min*ds3 - 2*d3s_min*ds_max + (d2s3*d3s_min*dt)/2)
		//%     value2 = ((d3s_max^2*dt^2)/16 + 2*d3s_max*ds3 - 2*d3s_max*ds_min + (d2s3*d3s_max*dt)/2)
		//%     lhs_31 = -sqrt(value1) + d3s_min*dt/4
		//%     rhs_31 =  sqrt(value2) + d3s_max*dt/4
		//%
		//%  -- COND 3.2 下一时刻的跃度满足条件
		//%     lhs_32 = (d3p_min - e4)/f4
		//%     rhs_32 = (d3p_max - e4)/f4
		//%
		//%  -- COND 3.3 跃度不超的前提下，当前加速度不能太快，否则未来速度可能超出边界
		//%     value1 = ((d3p_min^2*dt^2)/16 + 2*d3p_min*dp3 - 2*d3p_min*dp_max + (d2p3*d3p_min*dt)/2)
		//%     value2 = ((d3p_max^2*dt^2)/16 + 2*d3p_max*dp3 - 2*d3p_max*dp_min + (d2p3*d3p_max*dt)/2)
		//%     
		//%     lhs_33 = -sqrt(value1) + d3p_max*dt/4
		//%     rhs_33 =  sqrt(value2) + d3p_min*dt/4
		//%
		//%  -- COND 3.4 跃度不超的前提下，当前速度不能太快，否则未来位置可能超出边界
		//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%tbd

		double 
			lhs_s{ -1e10 }, rhs_s{1e10},
			lhs_ds{ MIN_DS }, rhs_ds{ MAX_DS },
			lhs_d2s{ MIN_D2S }, rhs_d2s{MAX_D2S},
			lhs_d3s{ MIN_D3S }, rhs_d3s{ MAX_D3S }
		;

		double s3 = 0.0;

		double max_k = 0.0;
		for (int i = 0; i < dim; ++i) {
			auto dp3 = (p3[i] - p2[i]) / dt;
			auto dp2 = (p2[i] - p1[i]) / dt;
			auto dp1 = (p1[i] - p0[i]) / dt;

			auto d2p3 = (dp3 - dp2) / dt;
			auto d2p2 = (dp2 - dp1) / dt;

			auto d3p3 = (d2p3 - d2p2) / dt;

			auto dp_t15 = dp2;
			auto d2p_t15 = (d2p2 + d2p3) / 2;
			auto d3p_t15 = d3p3;

			auto dp_ds_t15 = dp_t15 / ds_t15;
			auto d2p_ds2_t15 = (d2p_t15 - dp_ds_t15 * d2s_t15) / ds_t15 / ds_t15;
			auto d3p_ds3_t15 = (d3p_t15 - 3 * d2p_ds2_t15 * ds_t15 * d2s_t15 - dp_ds_t15 * d3s_t15) / ds_t15 / ds_t15 / ds_t15;

			auto s25_s15 = (ds2 + ds3) / 2 * dt;
			auto d3p_ds3_t25 = d3p_ds3_t15;
			auto d2p_ds2_t25 = d2p_ds2_t15 + d3p_ds3_t15 * s25_s15;
			auto dp_ds_t25 = dp_ds_t15 + d2p_ds2_t15 * s25_s15 + 0.5 * d3p_ds3_t15 * s25_s15 * s25_s15;

			auto k = (dp_ds_t25 + (3 * d2p_ds2_t25 * ds_t25 * dt) / 2);
			auto g = d3p_ds3_t25 * ds_t25 * ds_t25 * ds_t25 + 3 * d2p_ds2_t25 * ds3 * d2s3;

			auto f1 = k * dt * dt * dt;
			auto f2 = k * dt * dt;
			auto f3 = k * dt;
			auto f4 = k;
			auto e1 = p3[i] + dp3 * dt + d2p3 * dt * dt + g * dt * dt * dt;
			auto e2 = dp3 + d2p3 * dt + g * dt * dt;
			auto e3 = d2p3 + g * dt;
			auto e4 = g;

			if (std::abs(k) > zero_check) {
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

				auto lhs_d3s_local = lhs4_local;
				auto rhs_d3s_local = rhs4_local;

				auto lhs_d2s_local = d2s3 + lhs3_local * dt;
				auto rhs_d2s_local = d2s3 + rhs3_local * dt;
				
				auto lhs_ds_local = ds3 + d2s3 * dt + lhs2_local * dt * dt;
				auto rhs_ds_local = ds3 + d2s3 * dt + rhs2_local * dt * dt;

				auto lhs_s_local = s3 + ds3 * dt + d2s3 * dt * dt + lhs1_local * dt * dt * dt;
				auto rhs_s_local = s3 + ds3 * dt + d2s3 * dt * dt + rhs1_local * dt * dt * dt;	

				// 修正1：某个电机减速度过大的时候，如果减小 ds，反而会增加减速度，因此需要提前预判 //
				double rhs_d2s_local2 = rhs_d2s_local, rhs_ds_local2 = rhs_ds_local;
				auto solve_sdis_dsb = [](double ds_a, double d2s_a, double d3s_min, double s_b, double ds_b, double d2p_ext, double zero_check)->double {
					double B3 = d3s_min;
					double B2 = d2s_a;
					double B1 = ds_a;
					double B0 = 0;

					double k3 = -B3 / 12;
					double k1 = B1 / 2 + ds_b / 2;
					double k0 = B0 - s_b;

					double t[3], t_sol;
					if (auto ret = aris::dynamic::s_poly3_solve(k3, 0.0, k1, k0, t)) {
						t_sol = t[ret - 1];
						if (t_sol > 0) {
							auto d2s_max2 = (ds_b - B3 / 2 * t_sol * t_sol - B1) / t_sol;
							return d2s_max2;
						}
					}

					return std::numeric_limits<double>::quiet_NaN();
				};

				double d2p_ds2_a = d2p_ds2_t15;
				double d3p_ds3_a = d3p_ds3_t15;
				double dp_ds_a = dp_ds_t15;

				auto cond1 = (d3p_ds3_t15 > zero_check && d2p_ds2_t15 > zero_check && dp_ds_t15 < -zero_check);
				auto cond2 = (d3p_ds3_t15 < -zero_check && d2p_ds2_t25 < -zero_check && dp_ds_t25 > zero_check);
				if (cond1 || cond2){
					auto dp_exp = cond1 ? std::max(d2p_max[i], 0.0) : std::min(d2p_min[i], 0.0);
					auto d2p_exp = cond1 ? d2p_max[i]: d2p_min[i];
					auto d2p_exp_inv = cond1 ? d2p_min[i]: d2p_max[i];
					auto s_b = cond1 ? -(d2p_ds2_a - std::sqrt(d2p_ds2_a * d2p_ds2_a - 2 * d3p_ds3_a * dp_ds_a)) / d3p_ds3_a
						: -(d2p_ds2_a + std::sqrt(d2p_ds2_a * d2p_ds2_a - 2 * d3p_ds3_a * dp_ds_a)) / d3p_ds3_a;;
					
					double ds_b = std::sqrt(d2p_exp / (d2p_ds2_a + d3p_ds3_a * s_b));
					if (auto d2s_max2 = solve_sdis_dsb(ds_t15, d2s_t15, lhs_d3s_local, s_b, ds_b, d2p_exp, zero_check); std::isfinite(d2s_max2)) {
						rhs_d2s_local2 = std::min(rhs_d2s_local, std::max(d2s_max2, d2s3 + lhs_d3s_local * dt));
					}
					if (ds_b < ds_t15) {
						double s_b_2 = (d2p_exp / ds_t15 / ds_t15 - d2p_ds2_a) / d3p_ds3_a;
						double ds_b_2;
						if (s_b_2 < 0) {
							ds_b_2 = ds_b + (ds_t15 - ds_b) * (-s_b_2 / s_b) - std::max(rhs_d2s_local2, 0.0) * dt; // 以线性达到比例处

							//if (ds_b_2 < 0)
								//std::cout << "ds_b2: " << ds_b_2 << std::endl;
						}
						else {
							ds_b_2 = ds_b + (ds_t15 - ds_b) * std::sqrt(s_b_2 / s_b) - std::max(rhs_d2s_local2, 0.0) * dt; // 以根号达到比例处

							//if (ds_b_2 < 0)
								//std::cout << "ds_b2: " << ds_b_2 << std::endl;
						}

						if (auto d2s_max2 = solve_sdis_dsb(ds_t15, d2s_t15, lhs_d3s_local, s_b_2, ds_b_2, d2p_exp_inv, zero_check); std::isfinite(d2s_max2)) {
							rhs_d2s_local2 = std::min(rhs_d2s_local, std::max(d2s_max2, d2s3 + lhs_d3s_local * dt));

							
							//std::cout << "ds_t15:" << ds_t15 << "  d2s_t15:" << d2s_t15 << 
							//	"  lhs_d3s_local:" << lhs_d3s_local << "  s_b_2:" << s_b_2 << "  ds_b_2:" << ds_b_2
							//	<<"  d2s_max2:" << d2s_max2 << std::endl;
							//std::cout << "rhs_d2s_local2: " << rhs_d2s_local2 << std::endl;
						}
					}

					// 对ds做限制 //
					rhs_ds_local2 = std::min(rhs_ds_local, std::sqrt((dp_exp - d2p_exp * dt) / d2p_ds2_t25));
				}
				///////////////////////////////////////////////////////////////

				// 修正2：在到速度极限前，提前降低加速度 //
				double rhs_d2s_local3 = rhs_d2s_local, lhs_d2s_local3 = lhs_d2s_local;
				{
					double d2p_below, d2p_upper;
					s_smooth2_v_range(dp_max[i], dp_min[i], d2p_max[i], d2p_min[i], d3p_max[i], d3p_min[i], dt, dp3, d2p3, d2p_upper, d2p_below);
					auto lhs3_local = (d2p_below - e3) / f3;
					auto rhs3_local = (d2p_upper - e3) / f3;

					if (k < 0) {
						std::swap(lhs3_local, rhs3_local);
					}

					lhs_d2s_local3 = d2s3 + lhs3_local * dt;
					rhs_d2s_local3 = d2s3 + rhs3_local * dt;
				}

				// 修正3：在到位置极限前，提前降低速度
				double rhs_d2s_local4 = rhs_d2s_local, lhs_d2s_local4 = lhs_d2s_local;
				{
					double d2p_min2, d2p_max2;
					s_smooth3_a_range(p_max[i], p_min[i], dp_max[i], dp_min[i], d2p_max[i], d2p_min[i], d3p_max[i], d3p_min[i], dt, p3[i], dp3, d2p3, d2p_max2, d2p_min2);
					auto lhs4_local = (d2p_min2 - e3) / f3;
					auto rhs4_local = (d2p_max2 - e3) / f3;

					if (k < 0) {
						std::swap(lhs4_local, rhs4_local);
					}

					lhs_d2s_local4 = d2s3 + lhs4_local * dt;
					rhs_d2s_local4 = d2s3 + rhs4_local * dt;
				}

				// 最终更新数据 //
				lhs_d3s = std::max(lhs_d3s_local, lhs_d3s);
				rhs_d3s = std::min(rhs_d3s_local, rhs_d3s);
				lhs_d2s = std::max({ lhs_d2s_local, lhs_d2s_local3, lhs_d2s_local4, lhs_d2s });
				rhs_d2s = std::min({ rhs_d2s_local, rhs_d2s_local2, rhs_d2s_local3, rhs_d2s_local4, rhs_d2s });
				lhs_ds = std::max(lhs_ds_local, lhs_ds);
				rhs_ds = std::min({ rhs_ds_local, rhs_ds_local2, rhs_ds });
				lhs_s = std::max(lhs_s_local, lhs_s);
				rhs_s = std::min(rhs_s_local, rhs_s);
			}

			max_k = std::max(std::abs(k), max_k);
		}

		// 违反位置约束 //
		if (lhs_s > rhs_s || rhs_s < 0.0) {
			auto next_ds = std::min(MIN_DS, target_ds);
			ret.next_ds = next_ds;
			ret.state = -1;
			return ret.state;
		}
		// max min 是考虑所有因素的，rhs lhs 是仅仅考虑自身的
		double ds_min = std::max(lhs_ds, (lhs_s - s3) / dt);
		double ds_max = std::min(rhs_ds, (rhs_s - s3) / dt);

		// 违反速度约束 //
		if (ds_min > ds_max || ds_max < 0.0) {
			ret.next_ds = std::min(ds_min, ds_max);
			ret.next_ds = std::min(ret.next_ds, std::max(MAX_DS, target_ds));
			ret.next_ds = std::max(ret.next_ds, std::min(MIN_DS, target_ds));
			ret.state = -2;
			return ret.state;
		}

		double d2s_min = std::max(lhs_d2s, (ds_min - ds3) / dt);
		double d2s_max = std::min(rhs_d2s, (ds_max - ds3) / dt);

		{
			double d2s_min2, d2s_max2;
			s_smooth2_v_range(1.0, 0.005, rhs_d2s, lhs_d2s, rhs_d3s, lhs_d3s, dt, ds3, d2s3, d2s_max2, d2s_min2);

			d2s_min = std::max(d2s_min, d2s_min2);
			d2s_max = std::min(d2s_max, d2s_max2);
		}

		// 违反加速度约束 //
		if (d2s_min > d2s_max) {
			double next_d2s;
			if (ds3 > ds_max) {
				next_d2s = std::min(d2s_min, d2s_max);
			}
			else if (ds3 < ds_min) {
				next_d2s = std::max(d2s_min, d2s_max);
			}
			else if(std::abs(ds_max - ds_min) > zero_check) {
				auto l = std::min(d2s_min, d2s_max);
				auto r = std::max(d2s_min, d2s_max);
				next_d2s = l + (r-l)*(ds3 - ds_min)/(ds_max - ds_min);
			}
			else {
				next_d2s = (d2s_min + d2s_max)/2;
			}

			//std::cout << "  d2s_min: " << d2s_min << "   d2s_max:" << d2s_max << std::endl;

			ret.next_ds = ds3 + next_d2s * dt;
			ret.next_ds = std::min(ret.next_ds, std::max(MAX_DS, target_ds));
			ret.next_ds = std::max(ret.next_ds, std::min(MIN_DS, target_ds));

			ret.state = -3;
			return ret.state;
		}

		double d3s_min = std::max(lhs_d3s, (d2s_min - d2s3) / dt);
		double d3s_max = std::min(rhs_d3s, (d2s_max - d2s3) / dt);

		// 违反跃度约束 //
		if (d3s_min > d3s_max) {
			double next_d3s;
			if (d2s3 > d2s_max) {
				next_d3s = std::min(d3s_min, d3s_max);
			}
			else if (d2s3 < d2s_min) {
				next_d3s = std::max(d3s_min, d3s_max);
			}
			else {
				next_d3s = (d3s_max + d3s_min) / 2;
			}

			auto next_d2s = d2s3 + next_d3s * dt;
			ret.next_ds = ds3 + next_d2s * dt;
			ret.next_ds = std::min(ret.next_ds, std::max(MAX_DS, target_ds));
			ret.next_ds = std::max(ret.next_ds, std::min(MIN_DS, target_ds));
			ret.state = -4;
			return ret.state;
		}

		double next_ds, next_d2s, next_d3s{ 0.0 };
		aris::Size total_count;
		s_follow_x(ds3, d2s3, target_ds, rhs_d2s, lhs_d2s, rhs_d3s, lhs_d3s, dt, zero_check, next_ds, next_d2s, next_d3s, total_count);
		next_ds = std::min(next_ds, std::max(MAX_DS, target_ds));
		next_ds = std::max(next_ds, std::min(MIN_DS, target_ds));

		ret.next_ds = next_ds;
		ret.state = 0;
		return 0;
	};

	struct SingularProcessor::Imp {
		using CurveParam = TcurveParam;

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

		std::int32_t *Ts_count_;

		std::int64_t tg_ret_{ 0 }, tg_ret_begin_{0};
		aris::Size total_singular_count_{ 0 }, current_singular_count_{ 0 };
		CurveParam* curve_params_;

		double target_ds_{ 1.0 };
		double ds1_{ 1.0 }, ds2_{ 1.0 }, ds3_{ 1.0 };

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
	auto SingularProcessor::setMaxPoss(const double* max_poss, const double* min_poss)->void {
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
	auto SingularProcessor::setMaxVels(const double* max_vels, const double* min_vels)->void {
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
	auto SingularProcessor::setMaxAccs(const double* max_accs, const double* min_accs)->void {
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
	auto SingularProcessor::setMaxJerks(const double* max_jerks, const double* min_jerks) -> void {
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
	auto SingularProcessor::setMaxVelRatio(double vel_ratio)->void {
		//imp_->max_vel_ratio_ = vel_ratio;
	}
	auto SingularProcessor::setMaxAccRatio(double acc_ratio)->void {
		//imp_->max_acc_ratio_ = acc_ratio;
	}
	auto SingularProcessor::setModel(aris::dynamic::ModelBase& model)->void {
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
		core::allocMem(mem_size, imp_->curve_params_, imp_->input_size_);
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
		imp_->curve_params_ = core::getMem(imp_->mem_.data(), imp_->curve_params_);
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
	auto SingularProcessor::setTrajectoryGenerator(TrajectoryGenerator& tg)->void {
		imp_->tg_ = &tg;
	}
	auto SingularProcessor::init()->void {
		imp_->model_->getInputPos(imp_->input_pos_this_);
		std::fill_n(imp_->input_vel_this_, imp_->input_size_, 0.0);
		std::fill_n(imp_->input_acc_this_, imp_->input_size_, 0.0);
		std::fill_n(imp_->input_vel_last_, imp_->input_size_, 0.0);

		imp_->model_->getInputPos(imp_->p0_);
		aris::dynamic::s_vc(imp_->input_size_, imp_->p0_, imp_->p1_);
		aris::dynamic::s_vc(imp_->input_size_, imp_->p0_, imp_->p2_);
		aris::dynamic::s_vc(imp_->input_size_, imp_->p0_, imp_->p3_);

		imp_->ds1_ = imp_->ds2_ = imp_->ds3_ = 1.0;

		imp_->state_ = Imp::SingularState::NORMAL;
	}
	auto SingularProcessor::setDs(double ds)->void {
		imp_->ds3_ = imp_->ds2_ = imp_->ds1_ = ds;
	}
	auto SingularProcessor::currentDs() -> double {
		return imp_->ds3_;
	}
	auto SingularProcessor::setTargetDs(double ds)->void {
		imp_->target_ds_ = ds;
	}
	auto SingularProcessor::setModelPosAndMoveDt()->std::int64_t {

#ifdef ARIS_DEBUG_SINGULAR_PROCESSOR
		static int count{ 0 }, tg_count{0};
		count++;
		if (count % 1000 == 0)
			std::cout << "count: " << count << std::endl;
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
		auto move_tg_step = [this]()->std::int64_t {
#ifdef ARIS_DEBUG_SINGULAR_PROCESSOR
			tg_count++;
#endif

			// 当前处于非奇异状态，正常求反解 //
			auto ret = imp_->tg_->getEePosAndMoveDt(imp_->output_pos_);
			if (imp_->inv_func_) {
				if (auto ret = imp_->inv_func_(*imp_->model_, imp_->output_pos_); ret) {
#ifdef ARIS_DEBUG_SINGULAR_PROCESSOR
					std::cout << "inverse kin failed" << ret << std::endl;
#endif
					return ret;
				}
					
			}
			else {
				imp_->model_->setOutputPos(imp_->output_pos_);
				if (auto ret = imp_->model_->inverseKinematics(); ret) {
#ifdef ARIS_DEBUG_SINGULAR_PROCESSOR
					std::cout << "inverse kin failed" << ret << std::endl;
#endif
					return ret;
				
				}
			}
			
			// update ds //
			imp_->ds1_ = imp_->ds2_;
			imp_->ds2_ = imp_->ds3_;
			imp_->ds3_ = imp_->tg_->currentDs();

			// update input pos //
			std::swap(imp_->p0_, imp_->p1_);
			std::swap(imp_->p1_, imp_->p2_);
			std::swap(imp_->p2_, imp_->p3_);
			imp_->model_->getInputPos(imp_->p3_);

			SmoothParam param{
				imp_->tg_->dt(),
				imp_->input_size_,
				imp_->smooth_min_poss_, imp_->smooth_max_poss_, 
				imp_->smooth_min_vels_, imp_->smooth_max_vels_, 
				imp_->smooth_min_accs_, imp_->smooth_max_accs_, 
				imp_->smooth_min_jerks_, imp_->smooth_max_jerks_,
				0.005,1.0,-100.0,100.0,-10000.0,10000.0,
				imp_->ds1_, imp_->ds2_, imp_->ds3_,
				imp_->p0_, imp_->p1_, imp_->p2_, imp_->p3_,
				imp_->target_ds_
			};
			SmoothRet smooth_ret;
			s_smooth_curve3(param, smooth_ret);

#ifdef ARIS_DEBUG_SINGULAR_PROCESSOR
			if(smooth_ret.state)
				std::cout << "count:"<<count <<" tg:" << tg_count<<"  smooth ret : " << smooth_ret.state <<"  ds : " << smooth_ret.next_ds << std::endl;
#endif

			imp_->tg_->setCurrentDs(smooth_ret.next_ds);
			imp_->tg_->setCurrentDds(0.0);
			imp_->tg_->setTargetDs(smooth_ret.next_ds);

			return ret;
		};

		// move in singular state
		// 不考虑 max_vel_ratio 与 max_acc_ratio，奇异降速时，只考虑真实电机限制
		auto move_in_singular = [this]()->std::int64_t {
			auto dt = imp_->tg_->dt();
			
			// 取出位置
			if (imp_->current_singular_count_ > imp_->total_singular_count_) {
				std::swap(imp_->input_pos_this_, imp_->input_pos_last_);
				std::swap(imp_->input_vel_this_, imp_->input_vel_last_);
				for (int i = 0; i < imp_->input_size_; ++i) {
					imp_->input_pos_this_[i] = imp_->p3_[i];
				}
			}
			else {
				std::swap(imp_->input_pos_this_, imp_->input_pos_last_);
				std::swap(imp_->input_vel_this_, imp_->input_vel_last_);
				for (int i = 0; i < imp_->input_size_; ++i) {
					imp_->input_pos_this_[i] = s_tcurve_value(imp_->curve_params_[i], imp_->current_singular_count_ * dt);
				}
			}
			

#ifdef ARIS_DEBUG_SINGULAR_PROCESSOR
			auto curve_param = imp_->curve_params_[5];
			//std::cout << curve_param.a << std::endl;

			auto pos_11 = imp_->input_pos_end_[5];
			auto vel_11 = imp_->input_vel_end_[5];

			auto v1 = (imp_->p3_[5] - imp_->p2_[5])/dt;
			auto v0 = (imp_->p2_[5] - imp_->p1_[5])/dt;

			auto a1 = (v1 - v0) / dt;

			static double last_p5 = imp_->input_pos_this_[5], last_last_p5 = imp_->input_pos_this_[5];


			double this_p5 = imp_->input_pos_this_[5];
			double last_v5 = (last_p5 - last_last_p5) / dt;
			double this_v5 = (this_p5 - last_p5) / dt;
			double this_a5 = (this_v5 - last_v5) / dt;

			last_last_p5 = last_p5;
			last_p5 = this_p5;

			//auto acc_11 = imp_->input_acc_end_[5];
#endif

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
			if (imp_->current_singular_count_ > imp_->total_singular_count_ + 1) {
				imp_->state_ = Imp::SingularState::NORMAL;
				return imp_->tg_ret_;
			}

			return imp_->tg_ret_begin_;
		};

		// check if singular //
		auto check_if_singular = [](aris::Size input_size, double dt, const double* max_vel, const double* max_acc, const double* p1, const double* p2, const double *p3)->int {
			// here is condition //
			int idx = 0;
			for (idx = 0; idx < input_size; ++idx) {
				double v2 = (p3[idx] - p2[idx]) / dt;
				double v1 = (p2[idx] - p1[idx]) / dt;
				double a = (v2 - v1) / dt;

				if (v2 > max_vel[idx] || v2 < -max_vel[idx] || a > max_acc[idx] || a < -max_acc[idx]) {
#ifdef ARIS_DEBUG_SINGULAR_PROCESSOR
					//std::cout << "singular idx" << idx << " vel:" << v2 << "  max_vel:" << max_vel[idx] << "  acc:" << a << "  max_acc:" << max_acc[idx] << std::endl;
#endif
					return idx;
				}
			}
			return input_size;
		};

		auto prepare_singular = [&]()->std::int64_t {
#ifdef ARIS_DEBUG_SINGULAR_PROCESSOR
			std::cout << "count:" << count << " singular prepare : " << std::endl;
#endif
			imp_->state_ = Imp::SingularState::SINGULAR_PREPARE;

			// 将当前值置为起始值
			aris::dynamic::s_vc(imp_->input_size_, imp_->input_pos_this_, imp_->input_pos_begin_);
			aris::dynamic::s_vc(imp_->input_size_, imp_->input_vel_this_, imp_->input_vel_begin_);


			// 尝试迭代到下一个非奇异的时刻，最多迭代 10 次 //
			const int MAX_ITER_COUNT = 10;

			int idx;
			int while_count{ 0 };
			auto dt = imp_->tg_->dt();
			do {
				while_count++;

				// 处理速度突变 //
				/*
				double vel_ratio = 1.0;
				for (int i = 0; i < imp_->input_size_; ++i) {
					aris::dynamic::s_vc(imp_->input_size_, imp_->p3_, imp_->input_vel_end_);
					aris::dynamic::s_vs(imp_->input_size_, imp_->p2_, imp_->input_vel_end_);
					aris::dynamic::s_nv(imp_->input_size_, 1.0 / imp_->tg_->dt(), imp_->input_vel_end_);

					aris::dynamic::s_vc(imp_->input_size_, imp_->p2_, imp_->input_pos_end_);

					double pt = imp_->input_pos_end_[i] - imp_->input_pos_begin_[i];
					double vb = imp_->input_vel_begin_[i];
					double ve = imp_->input_vel_end_[i];
					
					if (ve > imp_->max_accs_[i] * dt || ve < imp_->min_accs_[i] * dt) {
						if ((vb >= 0 && ve >= 0 && pt < 0) || (vb <= 0 && ve <= 0 && pt > 0)) {
							vel_ratio = 0.0;
							break;
						}

						// pt / T = (vb + ve)/2
						// T = (ve - vb)/a
						//
						// => pt*a*2 = ve*ve - vb*vb
						double ve_max;
						if (pt >= 0)
							ve_max = std::sqrt(2.0 * pt * imp_->max_accs_[i] + vb * vb);
						else
							ve_max = -std::sqrt(2.0 * pt * imp_->min_accs_[i] + vb * vb);

						if (std::abs(ve) > 1e-10)
							vel_ratio = std::min(vel_ratio, std::abs(ve_max / ve));
					}
				}
				double ds_should_be = std::max(std::min(imp_->ds3_ * vel_ratio, imp_->tg_->currentDs()), 0.005);
				//ds_should_be = std::min(ds_should_be, imp_->tg_->currentDs());
				imp_->tg_->setCurrentDs(ds_should_be);
				imp_->tg_->setCurrentDds(0.0);
				imp_->tg_->setTargetDs(ds_should_be);*/

				// 迭代1步 //
				auto target_ds_store = imp_->target_ds_;
				imp_->target_ds_ = 0.005;
				imp_->tg_ret_ = move_tg_step();
				imp_->target_ds_ = target_ds_store;

				idx = check_if_singular(imp_->input_size_, dt, imp_->max_vels_, imp_->max_accs_, imp_->p1_, imp_->p2_, imp_->p3_);

				// 尝试处理奇异情况 //
				if (idx == imp_->input_size_ || while_count >= MAX_ITER_COUNT) {
					// 保存终止时刻的位置与速度 //
					aris::dynamic::s_vc(imp_->input_size_, imp_->p2_, imp_->input_pos_end_);

					aris::dynamic::s_vc(imp_->input_size_, imp_->p3_, imp_->input_vel_end_);
					aris::dynamic::s_vs(imp_->input_size_, imp_->p2_, imp_->input_vel_end_);
					aris::dynamic::s_nv(imp_->input_size_, 1.0 / imp_->tg_->dt(), imp_->input_vel_end_);

					// 计算每根轴所需要的时间 //
					auto Ts_count = imp_->Ts_count_;
					for (int i = 0; i < imp_->input_size_; ++i) {
						auto& tcurve_param = imp_->curve_params_[i];

						tcurve_param.pb_ = imp_->input_pos_begin_[i];
						tcurve_param.pe_ = imp_->input_pos_end_[i];
						tcurve_param.vb_ = imp_->input_vel_begin_[i];
						tcurve_param.ve_ = imp_->input_vel_end_[i];
						tcurve_param.vmax_ = imp_->max_vels_[i];
						tcurve_param.amax_ = imp_->max_accs_[i];

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
						imp_->curve_params_[i].T_ = Tmin_count * dt;
						s_tcurve_param(imp_->curve_params_[i]);
					}

					imp_->total_singular_count_ = Tmin_count;
					imp_->current_singular_count_ = 1;



					if (imp_->tg_ret_ && imp_->total_singular_count_ > 1)
					{
#ifdef ARIS_DEBUG_SINGULAR_PROCESSOR
						std::cout <<"count:" << count <<" tg:" << tg_count << " singular prepare end:" << imp_->total_singular_count_ << std::endl;
#endif
						bool is_singular_prepared = true;
						for (int i = 0; i < imp_->input_size_; ++i) {
							auto& tparam = imp_->curve_params_[i];
							if (!((tparam.vb_ >= 0 && tparam.ve_ >= 0 && tparam.v_ >= 0) || (tparam.vb_ <= 0 && tparam.ve_ <= 0 && tparam.v_ <= 0)))
								is_singular_prepared = false;

						}

						if(is_singular_prepared)
							imp_->state_ = Imp::SingularState::SINGULAR;
					}
					else
						imp_->state_ = Imp::SingularState::SINGULAR;

				}
			} while (imp_->state_ != Imp::SingularState::SINGULAR && while_count < MAX_ITER_COUNT);

			// 移动一步
			return move_in_singular();
		};

		if (imp_->state_ == Imp::SingularState::SINGULAR) {
			// 当前已经处于奇异状态 //
			return move_in_singular();
		}
		else if (imp_->state_ == Imp::SingularState::SINGULAR_PREPARE) {
			return prepare_singular();
		}
		else {
			imp_->tg_ret_ = move_tg_step();

			if ((imp_->singular_idx = check_if_singular(imp_->input_size_, imp_->tg_->dt(), imp_->max_vels_, imp_->max_accs_, imp_->p1_, imp_->p2_, imp_->p3_)) == imp_->input_size_) {
				// 正常保存位置 //
				std::swap(imp_->input_pos_this_, imp_->input_pos_last_);
				aris::dynamic::s_vc(imp_->input_size_, imp_->p3_, imp_->input_pos_this_);

				// 保存速度 //
				std::swap(imp_->input_vel_this_, imp_->input_vel_last_);
				aris::dynamic::s_vc(imp_->input_size_, imp_->input_pos_this_, imp_->input_vel_this_);
				aris::dynamic::s_vs(imp_->input_size_, imp_->input_pos_last_, imp_->input_vel_this_);
				aris::dynamic::s_nv(imp_->input_size_, 1.0 / imp_->tg_->dt(), imp_->input_vel_this_);

				// 保存加速度 //
				aris::dynamic::s_vc(imp_->input_size_, imp_->input_vel_this_, imp_->input_acc_this_);
				aris::dynamic::s_vs(imp_->input_size_, imp_->input_vel_last_, imp_->input_acc_this_);
				aris::dynamic::s_nv(imp_->input_size_, 1.0 / imp_->tg_->dt(), imp_->input_acc_this_);
				
				return imp_->tg_ret_;
			}
			else {
#ifdef ARIS_DEBUG_SINGULAR_PROCESSOR
				std::cout << "singular" << std::endl;
#endif
				imp_->tg_ret_begin_ = imp_->tg_ret_;
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
