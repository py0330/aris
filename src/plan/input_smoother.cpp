#include"aris/plan/input_smoother.hpp"
#include"aris/plan/function.hpp"

//#define ARIS_DEBUG_SINGULAR_PROCESSOR

namespace aris::plan {
	
	int test_depth = 0;

	// 计算出来 s4
	struct SmoothParam2 {
		double dt;
		int dim;
		const double* min_p, * max_p, * min_dp, * max_dp, * min_d2p, * max_d2p, * min_d3p, * max_d3p;
		double min_ds, max_ds, min_d2s, max_d2s, min_d3s, max_d3s;
		double s0, s1, s2, s3;
		double* p0, * p1, * p2, * p3;
	};
	struct SmoothRet2 {
		double d3s_lhs, d3s_rhs;
	};
	auto s_smooth_curve3(const SmoothParam2& param, SmoothRet2& ret) -> int {

		///////////////////////////// PART 1 计算 d3s 的可选范围 ///////////////////////////////// 
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
		
		//auto k = (dp_ds_t25 + (3 * d2p_ds2_t25 * ds_t25 * dt) / 2);
		//auto g = d3p_ds3_t25 * ds_t25 * ds_t25 * ds_t25 + 3 * d2p_ds2_t25 * ds3 * d2s3;


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

		const double MAX_DS = param.max_ds;
		const double MIN_DS = param.min_ds;
		const double MAX_D2S = param.max_d2s;
		const double MIN_D2S = param.min_d2s;
		const double MAX_D3S = param.max_d3s;
		const double MIN_D3S = param.min_d3s;

		double dt = param.dt;
		auto dim = param.dim;

		auto ds1 = (param.s1 - param.s0) / dt;
		auto ds2 = (param.s2 - param.s1) / dt;
		auto ds3 = (param.s3 - param.s2) / dt;

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

		double lhs_d3s{ MIN_D3S }, rhs_d3s{ MAX_D3S };

		// 限制 s //
		{
			auto k = 1.0;
			auto g = 0.0;

			auto f2 = k * dt * dt;
			auto f3 = k * dt;
			auto f4 = k;

			auto e2 = ds3 + d2s3 * dt + g * dt * dt;
			auto e3 = d2s3 + g * dt;
			auto e4 = g;
			
			auto lhs2_local = (MIN_DS - e2) / f2;
			auto rhs2_local = (MAX_DS - e2) / f2;

			auto lhs3_local = (MIN_D2S - e3) / f3;
			auto rhs3_local = (MAX_D2S - e3) / f3;

			auto lhs4_local = (MIN_D3S - e4) / f4;
			auto rhs4_local = (MAX_D3S - e4) / f4;

			auto lhs_d3s_local = std::max({ lhs2_local,lhs3_local });
			auto rhs_d3s_local = std::min({ rhs2_local,rhs3_local });

			lhs_d3s = std::max(lhs_d3s_local, lhs_d3s);
			rhs_d3s = std::min(rhs_d3s_local, rhs_d3s);
		}


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

			if (std::abs(k) > zero_check){
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

				//auto lhs_d3s_local = std::max({ lhs1_local,lhs2_local,lhs3_local,lhs4_local });
				//auto rhs_d3s_local = std::min({ rhs1_local,rhs2_local,rhs3_local,rhs4_local });
				auto lhs_d3s_local = std::max({ lhs2_local,lhs3_local });
				auto rhs_d3s_local = std::min({ rhs2_local,rhs3_local });

				// 最终更新数据 //
				lhs_d3s = std::max(lhs_d3s_local, lhs_d3s);
				rhs_d3s = std::min(rhs_d3s_local, rhs_d3s);
			}
		}


		ret.d3s_lhs = lhs_d3s;
		ret.d3s_rhs = rhs_d3s;






		return ret.d3s_rhs > ret.d3s_lhs;
	};

	// 计算出来 s3
	struct SmoothParam4 {
		double dt;
		int dim;
		const double* min_p, * max_p, * min_dp, * max_dp, * min_d2p, * max_d2p, * min_d3p, * max_d3p;
		double min_ds, max_ds, min_d2s, max_d2s, min_d3s, max_d3s;
		double s0, s1, s2, s3;
		double* p0, * p1, * p2, * p3;
	};
	struct SmoothRet4 {
		double d3s_lhs, d3s_rhs;
	};
	auto s_smooth_curve4(const SmoothParam4& param, SmoothRet4& ret) -> int {

		//		%已知数据
		//% t0        t1   t15   t2   t25   t3
		//% 
		//% p0        p1         p2         p3
		//%     dp1        dp2        dp3
		//%          d2p2       d2p3
		//%               d3p3
		//%  
		//% s0        s1         s2         s3
		//%     ds1        ds2        ds3
		//%          d2s2       d2s3
		//%               d3s3
		//%
		//% 给定 d3s4
		//% d2s4 = d2s3 + d3s4 * dt
		//% ds4  = ds3  + d2s4 * dt
		//%      = ds3  + d2s3*dt + d3s4*dt^2
		//% s4   = s3 + ds3*dt + d2s3*dt^2 + d3s4*dt^3
		//% 
		//% 给定 d3p4
		//% d2p4 = d2p3 + d3p4 * dt
		//% dp4  = dp3  + d2p4 * dt
		//%      = dp3  + d2p3*dt + d3p4*dt^2
		//% p4   = p3 + dp3*dt + d2p3*dt^2 + d3p4*dt^3
		//%
		//% dp与ds的关系：
		//% dp  = dp_ds * ds
		//% d2p = d2p_ds2 * ds^2 + dp_ds * d2s
		//% d3p = d3p_ds3 * ds^3 + 2*d2p_ds2*ds*d2s + d2p_ds2*ds*d2s + dp_ds*d3s
		//%     = d3p_ds3 * ds^3 + 3*d2p_ds2*ds*d2s + dp_ds*d3s
		//% 
		//% t=1.5:
		//% dp_ds_t15   = dp_t15/ds_t15
		//% d2p_ds2_t15 = (d2p_t15 - dp_ds_t15 * d2s_t15)/ds_t15^2
		//% d3p_ds3_t15 = (d3p_t15 - 3 * d2p_ds2_t15 * ds_t15 * d2s_t15 - dp_ds_t15 * d3s_t15)/ds_t15^3
		//% 
		//% 
		//% 进一步可求：
		//% d3p3 = d3p_t15
		//%      = d3p_ds3_t15*ds_t15^3 + 3 * d2p_ds2_t15 * ds_t15 * d2s_t15 + dp_ds_t15 * d3s_t15
		//%      = d3p_ds3_t15*ds2^3 + 3*d2p_ds2_t15*ds2*(d2s2+d3s2*dt/2)+dp_ds_t15 * d3s3
		//%      = d3p_ds3_t15*ds2^3 + 3*d2p_ds2_t15*ds2*d2s2 + (dp_ds_t15+3*d2p_ds2_t15*ds2*dt/2)*d3s3
		//%      = g + k * d3s3
		//%
		//%
		//% 其中：
		//%   k = (dp_ds_t15+3*d2p_ds2_t15*ds2*dt/2)
		//%   g = d3p_ds3_t15*ds2^3 + 3*d2p_ds2_t15*ds2*d2s2
		//% 
		//% 继而：
		//%    p4 =  dp4*dt +   p3 = f1 * d3s4 + e1
		//%   dp4 = d2p4*dt +  dp3 = f2 * d3s4 + e2
		//%  d2p4 = d3p4*dt + d2p3 = f3 * d3s4 + e3
		//%  d3p4                  = f4 * d3s4 + e4
		//%
		//%  其中：
		//%  f1 = k*dt^3
		//%  f2 = k*dt^2
		//%  f3 = k*dt
		//%  f4 = k
		//%  e1 = p2   + dp2*dt  + d2p2*dt^2 + g*dt^3
		//%  e2 = dp2  + d2p2*dt + g*dt^2
		//%  e3 = d2p2 + g*dt
		//%  e4 = g
		//%
		//% 对 p 做限制，有
		//%      p_min <   p <   p_max
		//%     dp_min <  dp <  dp_max
		//%    d2p_min < d2p < d2p_max
		//%    d3p_min < d3p < d3p_max
		//%
		//% 可转为对d3s的限制：
		//%    (p_min  -e1)/f1 <  d3s4  < (p_max  -e1)/f1
		//%    (dp_min -e2)/f2 <  d3s4  < (dp_max -e2)/f2
		//%    (d2p_min-e3)/f3 <  d3s4  < (d2p_max-e3)/f3
		//%    (d3p_min-e4)/f4 <  d3s4  < (d3p_max-e4)/f4
		//%
		//%  下讨论 d3s3 的限制条件，应有 lhs < d3s4 < rhs
		//%
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
		//%     value1 = ((d3p_max^2*dt^2)/16 + 2*d3p_max*dp3 - 2*d3p_max*dp_min + (d2p3*d3p_max*dt)/2)
		//%     value2 = ((d3p_min^2*dt^2)/16 + 2*d3p_min*dp3 - 2*d3p_min*dp_max + (d2p3*d3p_min*dt)/2)
		//%     lhs_33 = -sqrt(value1) + d3p_max*dt/4
		//%     rhs_33 =  sqrt(value2) + d3p_min*dt/4
		//%
		//%  -- COND 3.4 跃度不超的前提下，当前速度不能太快，否则未来位置可能超出边界
		//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%tbd
		//% 
		//% 【LEVEL 1】
		//%  -- COND 1.0
		//%  ds4 = ds3  + d2s3*dt + d3s4*dt^2
		//%  ds_min < ds4 < ds_max
		//%  => (ds_min - ds3 - d2s3*dt)/dt^2 < d3s4 < (ds_max - ds3 - d2s3*dt)/dt^2
		//%  -- COND 1.1
		//%  (dp_min -e2)/f2 <  d3s4  < (dp_max -e2)/f2
		//%
		//%  下讨论 COND 2.2 与 3.2 中的边界
		//%  当前状态为 p,v > 0, 当前最大加速度为a_max，全力减速时
		//%  p(t) = p + v*t + 0.5*a_min*t*t
		//%  其极值为 
		//%  t          = -v/a_min
		//%  p(v/a_max) = p - v^2/(2*a_min) < p_max
		//%
		//%  => v <  sqrt(-2*a_min*(p_max-p))
		//%  同理：
		//%     v > -sqrt( 2*a_max*(p-p_min))
		//%  
		//%  d2p4  = d2p_t3 
		//%  dp_t3 = dp3 + (d2p4 + d2p3)/2*dt/2
		//%
		//%  于是：
		//%  d2p4 < sqrt(-2*d3p_min*(dp_max-dp_t3))
		//%  => -d2p4^2 /2 / d3p_min < dp_max-dp_t3
		//%  => -d2p4^2 /2 / d3p_min < dp_max - dp3 - (d2p4 + d2p3)/2*dt/2
		//%  => (d2p4-d3p_min*dt/4)^2 -((d3p_min^2*dt^2)/16 + 2*d3p_min*dp3 - 2*d3p_min*dp_max + (d2p3*d3p_min*dt)/2) < 0
		//%  
		//%  => d2p4 < sqrt(value2) + d3p_min*dt/4
		//%  其中：
		//%  value2 = ((d3p_min^2*dt^2)/16 + 2*d3p_min*dp3 - 2*d3p_min*dp_max + (d2p3*d3p_min*dt)/2)

		
		double zero_check = 1e-10;

		const double MAX_DS = param.max_ds;
		const double MIN_DS = param.min_ds;
		const double MAX_D2S = param.max_d2s;
		const double MIN_D2S = param.min_d2s;
		const double MAX_D3S = param.max_d3s;
		const double MIN_D3S = param.min_d3s;

		double dt = param.dt;
		auto dim = param.dim;

		auto ds1 = (param.s1 - param.s0) / dt;
		auto ds2 = (param.s2 - param.s1) / dt;
		auto ds3 = (param.s3 - param.s2) / dt;

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
		//double ds_t25 = ds3;

		double lhs_d3s{ MIN_D3S }, rhs_d3s{ MAX_D3S };

		// 限制 s //
		{
			auto k = 1.0;
			auto g = 0.0;

			auto f2 = k * dt * dt;
			auto f3 = k * dt;
			auto f4 = k;

			auto e2 = ds2 + d2s2 * dt + g * dt * dt;
			auto e3 = d2s2 + g * dt;
			auto e4 = g;

			auto lhs2_local = (MIN_DS - e2) / f2;
			auto rhs2_local = (MAX_DS - e2) / f2;

			auto lhs3_local = (MIN_D2S - e3) / f3;
			auto rhs3_local = (MAX_D2S - e3) / f3;

			auto lhs4_local = (MIN_D3S - e4) / f4;
			auto rhs4_local = (MAX_D3S - e4) / f4;

			auto lhs_d3s_local = std::max({ lhs2_local,lhs3_local });
			auto rhs_d3s_local = std::min({ rhs2_local,rhs3_local });

			lhs_d3s = std::max(lhs_d3s_local, lhs_d3s);
			rhs_d3s = std::min(rhs_d3s_local, rhs_d3s);
		}


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

			//auto s25_s15 = (ds2 + ds3) / 2 * dt;
			//auto d3p_ds3_t25 = d3p_ds3_t15;
			//auto d2p_ds2_t25 = d2p_ds2_t15 + d3p_ds3_t15 * s25_s15;
			//auto dp_ds_t25 = dp_ds_t15 + d2p_ds2_t15 * s25_s15 + 0.5 * d3p_ds3_t15 * s25_s15 * s25_s15;

			//auto k = (dp_ds_t25 + (3 * d2p_ds2_t25 * ds_t25 * dt) / 2);
			//auto g = d3p_ds3_t25 * ds_t25 * ds_t25 * ds_t25 + 3 * d2p_ds2_t25 * ds3 * d2s3;

			auto k = (dp_ds_t15 + 3 * d2p_ds2_t15 * ds_t15 * dt / 2);
			auto g = d3p_ds3_t15 * ds_t15 * ds_t15 * ds_t15 + 3 * d2p_ds2_t15 * ds2 * d2s2;

			auto f1 = k * dt * dt * dt;
			auto f2 = k * dt * dt;
			auto f3 = k * dt;
			auto f4 = k;
			auto e1 = p2[i] + dp2 * dt + d2p2 * dt * dt + g * dt * dt * dt;
			auto e2 = dp2 + d2p2 * dt + g * dt * dt;
			auto e3 = d2p2 + g * dt;
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

				//auto lhs_d3s_local = std::max({ lhs1_local,lhs2_local,lhs3_local,lhs4_local });
				//auto rhs_d3s_local = std::min({ rhs1_local,rhs2_local,rhs3_local,rhs4_local });
				auto lhs_d3s_local = std::max({ lhs2_local,lhs3_local });
				auto rhs_d3s_local = std::min({ rhs2_local,rhs3_local });

				// 最终更新数据 //
				lhs_d3s = std::max(lhs_d3s_local, lhs_d3s);
				rhs_d3s = std::min(rhs_d3s_local, rhs_d3s);
			}
		}


		ret.d3s_lhs = lhs_d3s;
		ret.d3s_rhs = rhs_d3s;






		return ret.d3s_rhs > ret.d3s_lhs;
	};

	struct SmoothParam5 {
		double dt;
		int dim;
		const double* min_p, * max_p, * min_dp, * max_dp, * min_d2p, * max_d2p;
		double min_ds, max_ds, min_d2s, max_d2s;
		double s0, s1, s2, s3;
		double* p0, * p1, * p2, * p3;
	};
	struct SmoothRet5 {
		double d2s_lhs, d2s_rhs;
	};
	auto s_smooth_curve5(const SmoothParam5& param, SmoothRet5& ret) -> int {

		//		%已知数据
		//% t0        t1   t15   t2   t25   t3
		//% 
		//% p0        p1         p2         p3
		//%     dp1        dp2        dp3
		//%          d2p2       d2p3
		//%               d3p3
		//%  
		//% s0        s1         s2         s3
		//%     ds1        ds2        ds3
		//%          d2s2       d2s3
		//%               d3s3
		//%
		//% 给定 d3s4
		//% d2s4 = d2s3 + d3s4 * dt
		//% ds4  = ds3  + d2s4 * dt
		//%      = ds3  + d2s3*dt + d3s4*dt^2
		//% s4   = s3 + ds3*dt + d2s3*dt^2 + d3s4*dt^3
		//% 
		//% 给定 d3p4
		//% d2p4 = d2p3 + d3p4 * dt
		//% dp4  = dp3  + d2p4 * dt
		//%      = dp3  + d2p3*dt + d3p4*dt^2
		//% p4   = p3 + dp3*dt + d2p3*dt^2 + d3p4*dt^3
		//%
		//% dp与ds的关系：
		//% dp  = dp_ds * ds
		//% d2p = d2p_ds2 * ds^2 + dp_ds * d2s
		//% d3p = d3p_ds3 * ds^3 + 2*d2p_ds2*ds*d2s + d2p_ds2*ds*d2s + dp_ds*d3s
		//%     = d3p_ds3 * ds^3 + 3*d2p_ds2*ds*d2s + dp_ds*d3s
		//% 
		//% t=1.5:
		//% dp_ds_t15   = dp_t15/ds_t15
		//% d2p_ds2_t15 = (d2p_t15 - dp_ds_t15 * d2s_t15)/ds_t15^2
		//% d3p_ds3_t15 = (d3p_t15 - 3 * d2p_ds2_t15 * ds_t15 * d2s_t15 - dp_ds_t15 * d3s_t15)/ds_t15^3
		//% 
		//% 
		//% 进一步可求：
		//% d3p3 = d3p_t15
		//%      = d3p_ds3_t15*ds_t15^3 + 3 * d2p_ds2_t15 * ds_t15 * d2s_t15 + dp_ds_t15 * d3s_t15
		//%      = d3p_ds3_t15*ds2^3 + 3*d2p_ds2_t15*ds2*(d2s2+d3s2*dt/2)+dp_ds_t15 * d3s3
		//%      = d3p_ds3_t15*ds2^3 + 3*d2p_ds2_t15*ds2*d2s2 + (dp_ds_t15+3*d2p_ds2_t15*ds2*dt/2)*d3s3
		//%      = g + k * d3s3
		//%
		//%
		//% 其中：
		//%   k = (dp_ds_t15+3*d2p_ds2_t15*ds2*dt/2)
		//%   g = d3p_ds3_t15*ds2^3 + 3*d2p_ds2_t15*ds2*d2s2
		//% 
		//% 继而：
		//%    p4 =  dp4*dt +   p3 = f1 * d3s4 + e1
		//%   dp4 = d2p4*dt +  dp3 = f2 * d3s4 + e2
		//%  d2p4 = d3p4*dt + d2p3 = f3 * d3s4 + e3
		//%  d3p4                  = f4 * d3s4 + e4
		//%
		//%  其中：
		//%  f1 = k*dt^3
		//%  f2 = k*dt^2
		//%  f3 = k*dt
		//%  f4 = k
		//%  e1 = p2   + dp2*dt  + d2p2*dt^2 + g*dt^3
		//%  e2 = dp2  + d2p2*dt + g*dt^2
		//%  e3 = d2p2 + g*dt
		//%  e4 = g
		//%
		//% 对 p 做限制，有
		//%      p_min <   p <   p_max
		//%     dp_min <  dp <  dp_max
		//%    d2p_min < d2p < d2p_max
		//%    d3p_min < d3p < d3p_max
		//%
		//% 可转为对d3s的限制：
		//%    (p_min  -e1)/f1 <  d3s4  < (p_max  -e1)/f1
		//%    (dp_min -e2)/f2 <  d3s4  < (dp_max -e2)/f2
		//%    (d2p_min-e3)/f3 <  d3s4  < (d2p_max-e3)/f3
		//%    (d3p_min-e4)/f4 <  d3s4  < (d3p_max-e4)/f4
		//%
		//%  下讨论 d3s3 的限制条件，应有 lhs < d3s4 < rhs
		//%
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
		//%     value1 = ((d3p_max^2*dt^2)/16 + 2*d3p_max*dp3 - 2*d3p_max*dp_min + (d2p3*d3p_max*dt)/2)
		//%     value2 = ((d3p_min^2*dt^2)/16 + 2*d3p_min*dp3 - 2*d3p_min*dp_max + (d2p3*d3p_min*dt)/2)
		//%     lhs_33 = -sqrt(value1) + d3p_max*dt/4
		//%     rhs_33 =  sqrt(value2) + d3p_min*dt/4
		//%
		//%  -- COND 3.4 跃度不超的前提下，当前速度不能太快，否则未来位置可能超出边界
		//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%tbd
		//% 
		//% 【LEVEL 1】
		//%  -- COND 1.0
		//%  ds4 = ds3  + d2s3*dt + d3s4*dt^2
		//%  ds_min < ds4 < ds_max
		//%  => (ds_min - ds3 - d2s3*dt)/dt^2 < d3s4 < (ds_max - ds3 - d2s3*dt)/dt^2
		//%  -- COND 1.1
		//%  (dp_min -e2)/f2 <  d3s4  < (dp_max -e2)/f2
		//%
		//%  下讨论 COND 2.2 与 3.2 中的边界
		//%  当前状态为 p,v > 0, 当前最大加速度为a_max，全力减速时
		//%  p(t) = p + v*t + 0.5*a_min*t*t
		//%  其极值为 
		//%  t          = -v/a_min
		//%  p(v/a_max) = p - v^2/(2*a_min) < p_max
		//%
		//%  => v <  sqrt(-2*a_min*(p_max-p))
		//%  同理：
		//%     v > -sqrt( 2*a_max*(p-p_min))
		//%  
		//%  d2p4  = d2p_t3 
		//%  dp_t3 = dp3 + (d2p4 + d2p3)/2*dt/2
		//%
		//%  于是：
		//%  d2p4 < sqrt(-2*d3p_min*(dp_max-dp_t3))
		//%  => -d2p4^2 /2 / d3p_min < dp_max-dp_t3
		//%  => -d2p4^2 /2 / d3p_min < dp_max - dp3 - (d2p4 + d2p3)/2*dt/2
		//%  => (d2p4-d3p_min*dt/4)^2 -((d3p_min^2*dt^2)/16 + 2*d3p_min*dp3 - 2*d3p_min*dp_max + (d2p3*d3p_min*dt)/2) < 0
		//%  
		//%  => d2p4 < sqrt(value2) + d3p_min*dt/4
		//%  其中：
		//%  value2 = ((d3p_min^2*dt^2)/16 + 2*d3p_min*dp3 - 2*d3p_min*dp_max + (d2p3*d3p_min*dt)/2)


		double zero_check = 1e-10;

		const double MAX_DS = param.max_ds;
		const double MIN_DS = param.min_ds;
		const double MAX_D2S = param.max_d2s;
		const double MIN_D2S = param.min_d2s;

		double dt = param.dt;
		auto dim = param.dim;

		auto ds1 = (param.s1 - param.s0) / dt;
		auto ds2 = (param.s2 - param.s1) / dt;
		auto ds3 = (param.s3 - param.s2) / dt;

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

		auto d2s2 = (ds2 - ds1) / dt;
		auto d2s3 = (ds3 - ds2) / dt;
		auto d3s3 = (d2s3 - d2s2) / dt;

		double ds_t15 = ds2;
		double d2s_t15 = (d2s2 + d2s3) / 2;
		double d3s_t15 = d3s3;
		//double ds_t25 = ds3;

		double lhs_d2s{ MIN_D2S }, rhs_d2s{ MAX_D2S };

		// 限制 s //
		{
			auto k = 1.0;
			auto g = 0.0;

			auto f2 = k * dt * dt;
			auto f3 = k * dt;
			auto f4 = k;

			auto e2 = ds2 + d2s2 * dt + g * dt * dt;
			auto e3 = d2s2 + g * dt;
			auto e4 = g;

			auto lhs2_local = (MIN_DS - e2) / f2;
			auto rhs2_local = (MAX_DS - e2) / f2;

			auto lhs3_local = (MIN_D2S - e3) / f3;
			auto rhs3_local = (MAX_D2S - e3) / f3;

			auto lhs_d2s_local = std::max({ lhs2_local,lhs3_local });
			auto rhs_d2s_local = std::min({ rhs2_local,rhs3_local });

			lhs_d2s = std::max(lhs_d2s_local, lhs_d2s);
			rhs_d2s = std::min(rhs_d2s_local, rhs_d2s);
		}


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

			//auto s25_s15 = (ds2 + ds3) / 2 * dt;
			//auto d3p_ds3_t25 = d3p_ds3_t15;
			//auto d2p_ds2_t25 = d2p_ds2_t15 + d3p_ds3_t15 * s25_s15;
			//auto dp_ds_t25 = dp_ds_t15 + d2p_ds2_t15 * s25_s15 + 0.5 * d3p_ds3_t15 * s25_s15 * s25_s15;

			//auto k = (dp_ds_t25 + (3 * d2p_ds2_t25 * ds_t25 * dt) / 2);
			//auto g = d3p_ds3_t25 * ds_t25 * ds_t25 * ds_t25 + 3 * d2p_ds2_t25 * ds3 * d2s3;

			auto k = (dp_ds_t15 + 3 * d2p_ds2_t15 * ds_t15 * dt / 2);
			auto g = d3p_ds3_t15 * ds_t15 * ds_t15 * ds_t15 + 3 * d2p_ds2_t15 * ds2 * d2s2;

			auto f1 = k * dt * dt * dt;
			auto f2 = k * dt * dt;
			auto f3 = k * dt;
			auto f4 = k;
			auto e1 = p2[i] + dp2 * dt + d2p2 * dt * dt + g * dt * dt * dt;
			auto e2 = dp2 + d2p2 * dt + g * dt * dt;
			auto e3 = d2p2 + g * dt;
			auto e4 = g;

			if (std::abs(k) > zero_check) {
				auto lhs1_local = (p_min[i] - e1) / f1;
				auto rhs1_local = (p_max[i] - e1) / f1;

				auto lhs2_local = (dp_min[i] - e2) / f2;
				auto rhs2_local = (dp_max[i] - e2) / f2;

				auto lhs3_local = (d2p_min[i] - e3) / f3;
				auto rhs3_local = (d2p_max[i] - e3) / f3;

				if (k < 0) {
					std::swap(lhs1_local, rhs1_local);
					std::swap(lhs2_local, rhs2_local);
					std::swap(lhs3_local, rhs3_local);
					//std::swap(lhs4_local, rhs4_local);
				}

				//auto lhs_d3s_local = std::max({ lhs1_local,lhs2_local,lhs3_local,lhs4_local });
				//auto rhs_d3s_local = std::min({ rhs1_local,rhs2_local,rhs3_local,rhs4_local });
				auto lhs_d3s_local = std::max({ lhs2_local,lhs3_local });
				auto rhs_d3s_local = std::min({ rhs2_local,rhs3_local });

				// 最终更新数据 //
				lhs_d2s = std::max(lhs_d3s_local, lhs_d2s);
				rhs_d2s = std::min(rhs_d3s_local, rhs_d2s);
			}
		}


		ret.d2s_lhs = lhs_d2s;
		ret.d2s_rhs = rhs_d2s;

		return ret.d2s_rhs > ret.d2s_lhs;
	};

	struct SmoothParam6 {
		double dt;
		int dim;
		const double* min_p, * max_p, * min_dp, * max_dp, * min_d2p, * max_d2p, * min_d3p, * max_d3p;
		double min_ds, max_ds, min_d2s, max_d2s, min_d3s, max_d3s;
		double s0, s1, s2, s3;
		double* p0, * p1, * p2, * p3;
	};
	struct SmoothRet6 {
		double d2s_lhs, d2s_rhs;
	};
	auto s_smooth_curve4(const SmoothParam4& param, SmoothRet4& ret) -> int {

		//		%已知数据
		//% t0        t1   t15   t2   t25   t3
		//% 
		//% p0        p1         p2         p3
		//%     dp1        dp2        dp3
		//%          d2p2       d2p3
		//%               d3p3
		//%  
		//% s0        s1         s2         s3
		//%     ds1        ds2        ds3
		//%          d2s2       d2s3
		//%               d3s3
		//%
		//% 给定 d3s4
		//% d2s4 = d2s3 + d3s4 * dt
		//% ds4  = ds3  + d2s4 * dt
		//%      = ds3  + d2s3*dt + d3s4*dt^2
		//% s4   = s3 + ds3*dt + d2s3*dt^2 + d3s4*dt^3
		//% 
		//% 给定 d3p4
		//% d2p4 = d2p3 + d3p4 * dt
		//% dp4  = dp3  + d2p4 * dt
		//%      = dp3  + d2p3*dt + d3p4*dt^2
		//% p4   = p3 + dp3*dt + d2p3*dt^2 + d3p4*dt^3
		//%
		//% dp与ds的关系：
		//% dp  = dp_ds * ds
		//% d2p = d2p_ds2 * ds^2 + dp_ds * d2s
		//% d3p = d3p_ds3 * ds^3 + 2*d2p_ds2*ds*d2s + d2p_ds2*ds*d2s + dp_ds*d3s
		//%     = d3p_ds3 * ds^3 + 3*d2p_ds2*ds*d2s + dp_ds*d3s
		//% 
		//% t=1.5:
		//% dp_ds_t15   = dp_t15/ds_t15
		//% d2p_ds2_t15 = (d2p_t15 - dp_ds_t15 * d2s_t15)/ds_t15^2
		//% d3p_ds3_t15 = (d3p_t15 - 3 * d2p_ds2_t15 * ds_t15 * d2s_t15 - dp_ds_t15 * d3s_t15)/ds_t15^3
		//% 
		//% 
		//% 进一步可求：
		//% d3p3 = d3p_t15
		//%      = d3p_ds3_t15*ds_t15^3 + 3 * d2p_ds2_t15 * ds_t15 * d2s_t15 + dp_ds_t15 * d3s_t15
		//%      = d3p_ds3_t15*ds2^3 + 3*d2p_ds2_t15*ds2*(d2s2+d3s2*dt/2)+dp_ds_t15 * d3s3
		//%      = d3p_ds3_t15*ds2^3 + 3*d2p_ds2_t15*ds2*d2s2 + (dp_ds_t15+3*d2p_ds2_t15*ds2*dt/2)*d3s3
		//%      = g + k * d3s3
		//%
		//%
		//% 其中：
		//%   k = (dp_ds_t15+3*d2p_ds2_t15*ds2*dt/2)
		//%   g = d3p_ds3_t15*ds2^3 + 3*d2p_ds2_t15*ds2*d2s2
		//% 
		//% 继而：
		//%    p4 =  dp4*dt +   p3 = f1 * d3s4 + e1
		//%   dp4 = d2p4*dt +  dp3 = f2 * d3s4 + e2
		//%  d2p4 = d3p4*dt + d2p3 = f3 * d3s4 + e3
		//%  d3p4                  = f4 * d3s4 + e4
		//%
		//%  其中：
		//%  f1 = k*dt^3
		//%  f2 = k*dt^2
		//%  f3 = k*dt
		//%  f4 = k
		//%  e1 = p2   + dp2*dt  + d2p2*dt^2 + g*dt^3
		//%  e2 = dp2  + d2p2*dt + g*dt^2
		//%  e3 = d2p2 + g*dt
		//%  e4 = g
		//%
		//% 对 p 做限制，有
		//%      p_min <   p <   p_max
		//%     dp_min <  dp <  dp_max
		//%    d2p_min < d2p < d2p_max
		//%    d3p_min < d3p < d3p_max
		//%
		//% 可转为对d3s的限制：
		//%    (p_min  -e1)/f1 <  d3s4  < (p_max  -e1)/f1
		//%    (dp_min -e2)/f2 <  d3s4  < (dp_max -e2)/f2
		//%    (d2p_min-e3)/f3 <  d3s4  < (d2p_max-e3)/f3
		//%    (d3p_min-e4)/f4 <  d3s4  < (d3p_max-e4)/f4
		//%
		//%  下讨论 d3s3 的限制条件，应有 lhs < d3s4 < rhs
		//%
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
		//%     value1 = ((d3p_max^2*dt^2)/16 + 2*d3p_max*dp3 - 2*d3p_max*dp_min + (d2p3*d3p_max*dt)/2)
		//%     value2 = ((d3p_min^2*dt^2)/16 + 2*d3p_min*dp3 - 2*d3p_min*dp_max + (d2p3*d3p_min*dt)/2)
		//%     lhs_33 = -sqrt(value1) + d3p_max*dt/4
		//%     rhs_33 =  sqrt(value2) + d3p_min*dt/4
		//%
		//%  -- COND 3.4 跃度不超的前提下，当前速度不能太快，否则未来位置可能超出边界
		//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%tbd
		//% 
		//% 【LEVEL 1】
		//%  -- COND 1.0
		//%  ds4 = ds3  + d2s3*dt + d3s4*dt^2
		//%  ds_min < ds4 < ds_max
		//%  => (ds_min - ds3 - d2s3*dt)/dt^2 < d3s4 < (ds_max - ds3 - d2s3*dt)/dt^2
		//%  -- COND 1.1
		//%  (dp_min -e2)/f2 <  d3s4  < (dp_max -e2)/f2
		//%
		//%  下讨论 COND 2.2 与 3.2 中的边界
		//%  当前状态为 p,v > 0, 当前最大加速度为a_max，全力减速时
		//%  p(t) = p + v*t + 0.5*a_min*t*t
		//%  其极值为 
		//%  t          = -v/a_min
		//%  p(v/a_max) = p - v^2/(2*a_min) < p_max
		//%
		//%  => v <  sqrt(-2*a_min*(p_max-p))
		//%  同理：
		//%     v > -sqrt( 2*a_max*(p-p_min))
		//%  
		//%  d2p4  = d2p_t3 
		//%  dp_t3 = dp3 + (d2p4 + d2p3)/2*dt/2
		//%
		//%  于是：
		//%  d2p4 < sqrt(-2*d3p_min*(dp_max-dp_t3))
		//%  => -d2p4^2 /2 / d3p_min < dp_max-dp_t3
		//%  => -d2p4^2 /2 / d3p_min < dp_max - dp3 - (d2p4 + d2p3)/2*dt/2
		//%  => (d2p4-d3p_min*dt/4)^2 -((d3p_min^2*dt^2)/16 + 2*d3p_min*dp3 - 2*d3p_min*dp_max + (d2p3*d3p_min*dt)/2) < 0
		//%  
		//%  => d2p4 < sqrt(value2) + d3p_min*dt/4
		//%  其中：
		//%  value2 = ((d3p_min^2*dt^2)/16 + 2*d3p_min*dp3 - 2*d3p_min*dp_max + (d2p3*d3p_min*dt)/2)


		double zero_check = 1e-10;

		const double MAX_DS = param.max_ds;
		const double MIN_DS = param.min_ds;
		const double MAX_D2S = param.max_d2s;
		const double MIN_D2S = param.min_d2s;
		const double MAX_D3S = param.max_d3s;
		const double MIN_D3S = param.min_d3s;

		double dt = param.dt;
		auto dim = param.dim;

		auto ds1 = (param.s1 - param.s0) / dt;
		auto ds2 = (param.s2 - param.s1) / dt;
		auto ds3 = (param.s3 - param.s2) / dt;

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
		//double ds_t25 = ds3;

		double lhs_d3s{ MIN_D3S }, rhs_d3s{ MAX_D3S };

		// 限制 s //
		{
			auto k = 1.0;
			auto g = 0.0;

			auto f2 = k * dt * dt;
			auto f3 = k * dt;
			auto f4 = k;

			auto e2 = ds2 + d2s2 * dt + g * dt * dt;
			auto e3 = d2s2 + g * dt;
			auto e4 = g;

			auto lhs2_local = (MIN_DS - e2) / f2;
			auto rhs2_local = (MAX_DS - e2) / f2;

			auto lhs3_local = (MIN_D2S - e3) / f3;
			auto rhs3_local = (MAX_D2S - e3) / f3;

			auto lhs4_local = (MIN_D3S - e4) / f4;
			auto rhs4_local = (MAX_D3S - e4) / f4;

			auto lhs_d3s_local = std::max({ lhs2_local,lhs3_local });
			auto rhs_d3s_local = std::min({ rhs2_local,rhs3_local });

			lhs_d3s = std::max(lhs_d3s_local, lhs_d3s);
			rhs_d3s = std::min(rhs_d3s_local, rhs_d3s);
		}


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

			//auto s25_s15 = (ds2 + ds3) / 2 * dt;
			//auto d3p_ds3_t25 = d3p_ds3_t15;
			//auto d2p_ds2_t25 = d2p_ds2_t15 + d3p_ds3_t15 * s25_s15;
			//auto dp_ds_t25 = dp_ds_t15 + d2p_ds2_t15 * s25_s15 + 0.5 * d3p_ds3_t15 * s25_s15 * s25_s15;

			//auto k = (dp_ds_t25 + (3 * d2p_ds2_t25 * ds_t25 * dt) / 2);
			//auto g = d3p_ds3_t25 * ds_t25 * ds_t25 * ds_t25 + 3 * d2p_ds2_t25 * ds3 * d2s3;

			auto k = (dp_ds_t15 + 3 * d2p_ds2_t15 * ds_t15 * dt / 2);
			auto g = d3p_ds3_t15 * ds_t15 * ds_t15 * ds_t15 + 3 * d2p_ds2_t15 * ds2 * d2s2;

			auto f1 = k * dt * dt * dt;
			auto f2 = k * dt * dt;
			auto f3 = k * dt;
			auto f4 = k;
			auto e1 = p2[i] + dp2 * dt + d2p2 * dt * dt + g * dt * dt * dt;
			auto e2 = dp2 + d2p2 * dt + g * dt * dt;
			auto e3 = d2p2 + g * dt;
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

				//auto lhs_d3s_local = std::max({ lhs1_local,lhs2_local,lhs3_local,lhs4_local });
				//auto rhs_d3s_local = std::min({ rhs1_local,rhs2_local,rhs3_local,rhs4_local });
				auto lhs_d3s_local = std::max({ lhs2_local,lhs3_local });
				auto rhs_d3s_local = std::min({ rhs2_local,rhs3_local });

				// 最终更新数据 //
				lhs_d3s = std::max(lhs_d3s_local, lhs_d3s);
				rhs_d3s = std::min(rhs_d3s_local, rhs_d3s);
			}
		}


		ret.d3s_lhs = lhs_d3s;
		ret.d3s_rhs = rhs_d3s;

		ret.d3s_lhs = d2s3 + lhs_d3s * dt;




		return ret.d3s_rhs > ret.d3s_lhs;
	};

	auto check_if_ok(aris::Size input_size, double dt, const double* max_vel, const double* min_vel, const double* max_acc, const double* min_acc, const double* p1, const double* p2, const double* p3)->int {
		// here is condition //
		int idx = 0;
		for (idx = 0; idx < input_size; ++idx) {
			double v2 = (p3[idx] - p2[idx]) / dt;
			double v1 = (p2[idx] - p1[idx]) / dt;
			double a = (v2 - v1) / dt;

			if (v2 > max_vel[idx] || v2 < min_vel[idx] || a > max_acc[idx] || a < min_acc[idx]) {
				return idx;
			}
		}
		return input_size;
	};

	int begin_log = 0;

	struct InputSmoother::Imp {
		InverseKinematicMethod inv_func_ = [](aris::dynamic::ModelBase* model, const double* output_pos, double* input_pos)->std::int64_t {
			model->setOutputPos(output_pos);
			auto ret = model->inverseKinematics();
			model->getInputPos(input_pos);
			return ret;
			};

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
			* output_pos_,
			* input_poss_,// 对应 s 
			* s_; 

		std::int64_t* node_ids_;

		double
			* p0_,
			* p1_,
			* p2_,
			* p3_,
			* p4_,
			* p5_;

		double dt_{ 1e-3 };
		double s0_{ -3*dt_ }, s1_{ -2*dt_ }, s2_{ -1*dt_ }, s3_{ 0.0 };

		std::int64_t tg_idx_{ 0 };// 当前运行到的位置，与tg运行到的位置
		int look_head_size_{ 2000 };

		std::int64_t tg_ret_{ 0 };

		aris::dynamic::ModelBase* model_{ nullptr };
		aris::plan::TrajectoryGenerator* tg_{ nullptr };


		auto getInputByS(double s, double* p) -> int {
			auto pool_size = look_head_size_ + 7;// 数据池因为有一个起始数据,以及过去用来插值的3个数据，以及未来3个数据，因此应该比前瞻的数据多 7

			std::int64_t current_idx = s < 0 ? -std::int64_t(-s / dt_) : std::int64_t(s / dt_) + 1;
			auto s_local = s < 0 ? dt_ - std::fmod(-s, dt_) :std::fmod(s, dt_);

			double* p0 = input_poss_ + (std::max(current_idx - 3, 0i64) % pool_size) * input_size_;
			double* p1 = input_poss_ + (std::max(current_idx - 2, 0i64) % pool_size) * input_size_;
			double* p2 = input_poss_ + (std::max(current_idx - 1, 0i64) % pool_size) * input_size_;
			double* p3 = input_poss_ + (std::min(current_idx, tg_idx_) % pool_size) * input_size_;
			double* p4 = input_poss_ + (std::min(current_idx + 1, tg_idx_) % pool_size) * input_size_;
			double* p5 = input_poss_ + (std::min(current_idx + 2, tg_idx_) % pool_size) * input_size_;

			double s_series[6]{ -2*dt_, -dt_, 0.0, dt_, 2 * dt_, 3 * dt_ };
			for (Size i = 0; i < input_size_; ++i) {
				double x[6]{ p0[i], p1[i], p2[i], p3[i], p4[i], p5[i] };
				aris::dynamic::s_interp_scurve(s_series, x, s_local, p[i]);
			}
			
			
			/*
			double* p0 = input_poss_ + (std::max(current_idx - 2, 0i64) % pool_size) * input_size_;
			double* p1 = input_poss_ + (std::max(current_idx - 1, 0i64) % pool_size) * input_size_;
			double* p2 = input_poss_ + (std::min(current_idx, tg_idx_) % pool_size) * input_size_;
			double* p3 = input_poss_ + (std::min(current_idx + 1, tg_idx_) % pool_size) * input_size_;

			double s_series[4]{ -dt_, 0.0, dt_, 2 * dt_ };
			for (Size i = 0; i < input_size_; ++i) {
				double x[4]{ p0[i], p1[i], p2[i], p3[i] };
				aris::dynamic::s_interp_scurve2(s_series, x, s_local, p[i]);

				//p[i] = s_local * p2[i] + (dt_ - s_local) * p1[i];

			}
			*/
			return 0;
		}

		auto test_next_input(double s0, double s1, double s2, double s3, double *p0, double *p1, double *p2, double *p3) -> bool {
			
			getInputByS(s3, p3);

			test_depth++;

			// 判断是否成功 //
			if ((s3 - s2) < 1e-8)
				return true;

			// 没有在规定时间内把速度降为0 //
			if (s3 / dt_ > tg_idx_) {
				std::cout << "failed" << std::endl;
				return true;
				throw std::runtime_error("error");
			}

			// 没有在规定时间内把速度降为0 //
			if (check_if_ok(input_size_, dt_, max_vels_, min_vels_, max_accs_, min_accs_, p1, p2, p3) != input_size_) {
				check_if_ok(input_size_, dt_, max_vels_, min_vels_, max_accs_, min_accs_, p1, p2, p3);
				
				return false;
			}

			




			

			//std::cout << "--------" << s3 << std::endl;
			//aris::dynamic::dsp(1, 6, p0);
			//aris::dynamic::dsp(1, 6, p1);
			//aris::dynamic::dsp(1, 6, p2);
			//aris::dynamic::dsp(1, 6, p3);
			double p4[6];
			double s4 = s3 + (s2 - s1);
			getInputByS(s4, p4);

			SmoothParam4 param{
				dt_,
				input_size_,
				min_poss_, max_poss_, min_vels_, max_vels_,
				min_accs_, max_accs_, min_jerks_, max_jerks_,
				//min_ds, max_ds, min_d2s, max_d2s, min_d3s, max_d3s;
				-1.0, 1.0, -10.0, 10.0, -100000.0, 100000.0,
				s1, s2, s3, s4,
				p1, p2, p3, p4
			};
			SmoothRet4 ret;
			s_smooth_curve4(param, ret);

			std::swap(s0, s1);
			std::swap(s1, s2);
			std::swap(s2, s3);

			double ds1 = (s1 - s0) / dt_;
			double ds2 = (s2 - s1) / dt_;
			double d2s2 = (ds2 - ds1) / dt_;

			double d2s3 = d2s2 + ret.d3s_lhs * dt_;

			if (d2s3*(1000-test_depth)*dt_ > ds2 || ret.d3s_lhs > ret.d3s_rhs) {
				return false;
			}
			else {
				//d2s3 = std::min(0.99*d2s3, ds2/ (1000 - test_depth));
				d2s3 = std::min(0.99 * d2s3, d2s2 + ret.d3s_rhs * dt_);
			}

			if (begin_log) {
				std::cout << "test:"<<test_depth << std::endl;
				std::cout << "ds1:" << ds1 << "  ds2:" << ds2 << "  d2s3:" << d2s3 <<" lhs:" << ret.d3s_lhs <<"  rhs:" << ret.d3s_rhs << std::endl;
			}



			double ds3 = ds2 + d2s3 * dt_;
			s3 = s2 + ds3 * dt_;

			return test_next_input(s0, s1, s2, s3, p1, p2, p3, p0);
		}
	};
	auto InputSmoother::setModel(aris::dynamic::ModelBase& model) -> void {
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
		core::allocMem(mem_size, imp_->output_pos_, imp_->input_size_);
		core::allocMem(mem_size, imp_->node_ids_, imp_->look_head_size_ + 7);
		core::allocMem(mem_size, imp_->input_poss_, imp_->input_size_ * (imp_->look_head_size_ + 7));
		
		imp_->mem_.resize(mem_size, char(0));

		imp_->max_poss_ = core::getMem(imp_->mem_.data(), imp_->max_poss_);
		imp_->max_vels_ = core::getMem(imp_->mem_.data(), imp_->max_vels_);
		imp_->max_accs_ = core::getMem(imp_->mem_.data(), imp_->max_accs_);
		imp_->max_jerks_ = core::getMem(imp_->mem_.data(), imp_->max_jerks_);
		imp_->min_poss_ = core::getMem(imp_->mem_.data(), imp_->min_poss_);
		imp_->min_vels_ = core::getMem(imp_->mem_.data(), imp_->min_vels_);
		imp_->min_accs_ = core::getMem(imp_->mem_.data(), imp_->min_accs_);
		imp_->min_jerks_ = core::getMem(imp_->mem_.data(), imp_->min_jerks_);
		imp_->output_pos_ = core::getMem(imp_->mem_.data(), imp_->output_pos_);
		imp_->node_ids_ = core::getMem(imp_->mem_.data(), imp_->node_ids_);
		imp_->input_poss_ = core::getMem(imp_->mem_.data(), imp_->input_poss_);

		std::fill_n(imp_->max_poss_, imp_->input_size_, 1e10);
		std::fill_n(imp_->min_poss_, imp_->input_size_, -1e10);
		std::fill_n(imp_->max_vels_, imp_->input_size_, 1.0);
		std::fill_n(imp_->min_vels_, imp_->input_size_, -1.0);
		std::fill_n(imp_->max_accs_, imp_->input_size_, 10.0);
		std::fill_n(imp_->min_accs_, imp_->input_size_, -10.0);
		std::fill_n(imp_->max_jerks_, imp_->input_size_, 1000.0);
		std::fill_n(imp_->min_jerks_, imp_->input_size_, -1000.0);
	}
	auto InputSmoother::setTrajectoryGenerator(TrajectoryGenerator& tg) -> void {
		imp_->tg_ = &tg;
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
	auto InputSmoother::setJerkLimits(const double* max_jerks, const double* min_jerks) -> void {
		std::copy(max_jerks, max_jerks + imp_->input_size_, imp_->max_jerks_);
		if (min_jerks) {
			std::copy(min_jerks, min_jerks + imp_->input_size_, imp_->min_jerks_);
		}
		else {
			for (int i = 0; i < imp_->input_size_; ++i)
				imp_->min_jerks_[i] = -imp_->max_jerks_[i];
		}
	}
	auto InputSmoother::init(const double *init_input_pos) -> void {
		// 第一个数据应该为起始数据 //
		imp_->tg_idx_ = 0;
		aris::dynamic::s_vc(imp_->input_size_, init_input_pos, imp_->input_poss_);

		// 前瞻足够的数据 //
		auto pool_size = imp_->look_head_size_ + 7;
		for (int i = 0; i < imp_->look_head_size_; ++i) {
			imp_->tg_idx_ = imp_->tg_idx_ + 1;
			imp_->node_ids_[(imp_->tg_idx_ % pool_size)] = imp_->tg_->getEePosAndMoveDt(imp_->output_pos_);
			imp_->inv_func_(imp_->model_, imp_->output_pos_, imp_->input_poss_ + (imp_->tg_idx_ % pool_size) * imp_->input_size_);

			if (imp_->node_ids_[(imp_->tg_idx_ % pool_size)] == 0)
				break;
		}

		// 确保第一个数字有正确的值 //
		imp_->node_ids_[0] = imp_->node_ids_[1];

		imp_->s0_ = -3*imp_->dt_/5;
		imp_->s1_ = -2*imp_->dt_ / 5;
		imp_->s2_ = -1 * imp_->dt_ / 5;
		imp_->s3_ = -0 * imp_->dt_ / 5;
		//// 确保结尾多3个数值
		//aris::dynamic::s_vc(imp_->input_size_, imp_->input_poss_ + (imp_->tg_idx_ % pool_size) * imp_->input_size_, imp_->input_poss_ + ((imp_->tg_idx_ + 1) % pool_size) * imp_->input_size_);
		//aris::dynamic::s_vc(imp_->input_size_, imp_->input_poss_ + (imp_->tg_idx_ % pool_size) * imp_->input_size_, imp_->input_poss_ + ((imp_->tg_idx_ + 2) % pool_size) * imp_->input_size_);
		//aris::dynamic::s_vc(imp_->input_size_, imp_->input_poss_ + (imp_->tg_idx_ % pool_size) * imp_->input_size_, imp_->input_poss_ + ((imp_->tg_idx_ + 3) % pool_size) * imp_->input_size_);
	}

	

	auto InputSmoother::getNextInput(double* p) -> int {
		auto pool_size = imp_->look_head_size_ + 7;// 数据池因为有一个起始数据,以及过去用来插值的3个数据，以及未来3个数据，因此应该比前瞻的数据多 7

		// ----------------- PART 1 拿数据--------------------------- //
		{
			double p0[6], p1[6], p2[6], p3[6];

			imp_->getInputByS(imp_->s0_, p0);
			imp_->getInputByS(imp_->s1_, p1);
			imp_->getInputByS(imp_->s2_, p2);
			imp_->getInputByS(imp_->s3_, p3);

			double p4[6];
			double s4 = imp_->s3_ + (imp_->s2_ - imp_->s1_);
			imp_->getInputByS(s4, p4);

			SmoothParam4 param{
				imp_->dt_,
				imp_->input_size_,
				imp_->min_poss_, imp_->max_poss_, imp_->min_vels_, imp_->max_vels_,
				imp_->min_accs_, imp_->max_accs_, imp_->min_jerks_, imp_->max_jerks_,
				//min_ds, max_ds, min_d2s, max_d2s, min_d3s, max_d3s;
				-1.0, 1.0, -10.0, 10.0, -100000.0, 100000.0,
				imp_->s1_, imp_->s2_, imp_->s3_, s4,
				p1, p2, p3, p4
			};
			SmoothRet4 ret;
			s_smooth_curve4(param, ret);

			// fix ret to 0.99*ret
			{
				
				double ds1 = (imp_->s2_ - imp_->s1_) / imp_->dt_;
				double ds2 = (imp_->s3_ - imp_->s2_) / imp_->dt_;
				double d2s2 = (ds2 - ds1) / imp_->dt_;

				double d2s3 = d2s2 + ret.d3s_lhs * imp_->dt_;

				d2s3 = std::min(0.99 * d2s3, d2s2 + ret.d3s_rhs * imp_->dt_);
				ret.d3s_lhs = (d2s3 - d2s2) / imp_->dt_;

				
				if (begin_log){
					std::cout << "---------------------begin------------" << std::endl;
					std::cout << "ds1:" << ds1 <<"  ds2:" << ds2 << "  d2s3:"<< d2s3  << std::endl;
				}

			}




			// 二分法求解最优的 d3s
			{
				double l = ret.d3s_lhs;
				double r = ret.d3s_rhs;
				double mid = r;
				double last_mid = l;

				double s0{ imp_->s1_ }, s1{ imp_->s2_ }, s2{ imp_->s3_ }, s3{ 0.0 };


				double p1_back[6], p2_back[6], p3_back[6];
				aris::dynamic::s_vc(6, p1, p1_back);
				aris::dynamic::s_vc(6, p2, p2_back);
				aris::dynamic::s_vc(6, p3, p3_back);
				

				//if (l > r) {
				//	std::cout << "l failed:" << l << "  r:" << r << std::endl;
				//	return 0;
				//}
					

				for (; std::abs(r - l)>1e-10;) {
					// s0        s1         s2         s3
					//     ds1        ds2        ds3
					//          d2s2       d2s3
					//               d3s3
					
					last_mid = mid;
					mid = (l + r) / 2;

					double ds1 = (s1 - s0)/imp_->dt_;
					double ds2 = (s2 - s1)/imp_->dt_;
					double d2s2 = (ds2 - ds1)/imp_->dt_;

					double d2s3 = d2s2 + mid * imp_->dt_;
					double ds3 = ds2 + d2s3 * imp_->dt_;
					double s3 = s2 + ds3 * imp_->dt_;

					aris::dynamic::s_vc(6, p1_back, p1);
					aris::dynamic::s_vc(6, p2_back, p2);
					aris::dynamic::s_vc(6, p3_back, p3);

					test_depth = 0;
					if (imp_->test_next_input(s0, s1, s2, s3, p1, p2, p3, p0))
						l = mid;
					else
						r = mid;
				}
				
				double ds1 = (s1 - s0) / imp_->dt_;
				double ds2 = (s2 - s1) / imp_->dt_;
				double d2s2 = (ds2 - ds1) / imp_->dt_;

				double d2s3 = d2s2 + l * imp_->dt_;
				double ds3 = ds2 + d2s3 * imp_->dt_;

				s3 = s2 + ds3 * imp_->dt_;

				//////////////////////////////////////////////////
				//s3 = s2 + ds2 * imp_->dt_;
				//////////////////////////////////////////////////
				
				imp_->s0_ = s3;

				std::swap(imp_->s0_, imp_->s1_);
				std::swap(imp_->s1_, imp_->s2_);
				std::swap(imp_->s2_, imp_->s3_);

				//if (mid < -80)
				//	std::cout << "debug:" << std::endl;
				static int count_{ 0 };
				count_++;

				if (count_ > 9268 && count_ < 9300)
					begin_log = 1;
				else {
					begin_log = 0;
				}

				std::cout <<"count:" << count_ << "   mid : " << mid << "  s3:" << s3 << std::endl;

				imp_->getInputByS(s3, p);
			}
		}

		std::int64_t current_idx = std::int64_t(imp_->s3_ / imp_->dt_) + 1;
		// ----------------- PART 2 增数据--------------------------- //
		{
			// 如果轨迹已经结束，或已经满
			if (imp_->node_ids_[(imp_->tg_idx_ % pool_size)] && imp_->tg_idx_ - current_idx < imp_->look_head_size_) {
				imp_->tg_idx_ = imp_->tg_idx_ + 1;
				imp_->node_ids_[(imp_->tg_idx_ % pool_size)] = imp_->tg_->getEePosAndMoveDt(imp_->output_pos_);
				imp_->inv_func_(imp_->model_, imp_->output_pos_, imp_->input_poss_ + (imp_->tg_idx_ % pool_size) * imp_->input_size_);
			}

		}

		
		
		//aris::dynamic::dsp(1, 14, imp_->u_);
		//std::cout << "current:" << current_idx << "  tg:" << imp_->tg_idx_ << std::endl;
		return current_idx == imp_->tg_idx_ ? 0: imp_->node_ids_[current_idx % pool_size];
	}

	InputSmoother::~InputSmoother() = default;
	InputSmoother::InputSmoother() :imp_(new Imp) {

	}
}
