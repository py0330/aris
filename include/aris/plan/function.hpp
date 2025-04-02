#ifndef ARIS_PLAN_FUNCTION_H_
#define ARIS_PLAN_FUNCTION_H_


#include <aris/core/basic_type.hpp>

#include <string>
#include <map>
#include <cmath>
#include <limits>
#include <aris/core/core.hpp>

namespace aris::plan{







	auto inline acc_up(double n, double i)noexcept->double { return (-1.0 / 2 / n / n / n * i * i * i + 3.0 / 2.0 / n / n * i * i); }
	auto inline acc_down(double n, double i)noexcept->double { return (-1.0 * i * i * i / 2.0 / n / n / n + 3.0 * i * i / 2.0 / n / n); }
	auto inline dec_up(double n, double i)noexcept->double { return 1.0 - (-1.0 / 2.0 / n / n / n * (n - i) * (n - i) * (n - i) + 3.0 / 2.0 / n / n * (n - i) * (n - i)); }
	auto inline dec_down(double n, double i)noexcept->double { return 1.0 - (-1.0 * (n - i) * (n - i) * (n - i) / 2.0 / n / n / n + 3.0 * (n - i) * (n - i) / 2.0 / n / n); }

	auto inline acc_even(double n, double i)noexcept->double { return 1.0 / n / n * i * i; }
	auto inline dec_even(double n, double i)noexcept->double { return 1.0 - 1.0 / n / n * (n - i) * (n - i); }
	auto inline even(double n, double i)noexcept->double { return 1.0 / n * i; }

	auto inline s_p2p(double n, double i, double begin_pos, double end_pos)noexcept->double
	{
		double a = 4.0 * (end_pos - begin_pos) / n / n;
		return i <= n / 2.0 ? 0.5 * a * i * i + begin_pos : end_pos - 0.5 * a * (n - i) * (n - i);
	}
	auto inline s_v2v(int n, int i, double begin_vel, double end_vel)noexcept->double
	{
		double s = static_cast<double>(i) / n;
		double m = 1.0 - s;

		return (s * s * s - s * s) * end_vel * n + (m * m - m * m * m) * begin_vel * n;
	}
	auto inline s_interp(int n, int i, double begin_pos, double end_pos, double begin_vel, double end_vel)noexcept->double {
		double s = static_cast<double>(i) / n;

		double a, b, c, d;

		c = begin_vel * n;
		d = begin_pos;
		a = end_vel * n - 2.0 * end_pos + c + 2.0 * d;
		b = end_pos - c - d - a;

		return a * s * s * s + b * s * s + c * s + d;
	}

	auto ARIS_API moveAbsolute(double i, double begin_pos, double end_pos, double vel, double acc, double dec, double& current_pos, double& current_vel, double& current_acc, Size& total_count)->void;

	// pa : pos actual value
	// va : vel actual value
	// aa : acc actual value
	// pt : pos target value
	// vt : vel target value
	// at : acc target value
	// vm : vel max permitted value, always positive
	// am : acc max permitted value, always positive
	// dm : dec max permitted value, always positive
	// dt : time inteval, set to 1e-3
	// zero_check : set to 1e-10
	// pc : next planned pos
	// vc : next planned vel
	// ac : next planned acc
	// total_count : tbd, not finished yet
	auto ARIS_API moveAbsolute2(double pa, double va, double aa, double pt, double vt, double at, double vm, double am, double dm, double dt, double zero_check, double& pc, double& vc, double& ac, Size& total_count)->int;

	//  p0         p1
	//  v0  vavg   v1
	//      
	auto inline ARIS_API s_is_in_vavg_boundage(double v0, double v1, double vavg, double a_max, double a_min, double dt, double zero_check = 1e-10)->bool {
		if (v1 < v0)
			return s_is_in_vavg_boundage(v1, v0, vavg, -a_min, -a_max, dt, zero_check);

		if (a_max < 0 || a_min > 0)
			return false;

		// 
		// 从 v0 加速到 v1 所需时间为 t1，其他时间为 dt - t1
		// 其他时间可以在 v0 到 v1 的任何数值上进行加速减速，设若该加速所产生的平均速度偏移是 r
		// 
		// 其他时间的加速时间为 Ta，减速为 Tb
		// 有 Ta + Tb = dt - t1
		// r = Ta * a_max/2 = - Tb*a_min/2
		// Ta = -a_min/a_max * Tb
		// 
		// => Ta + Tb = (a_max - a_min)/a_max * Tb = dt - t1
		// => Tb = a_max/(a_max - a_min) * (dt - t1)
		// => r  = -Tb*a_min/2
		//       = -a_max*a_min/(a_max - a_min)*(dt - t1)/2
		// 


		// cpt r //
		double t1 = (v1 - v0) / a_max;
		double r = -a_max * a_min / (a_max - a_min) * (dt - t1) / 2;

		// too small margin //
		if (std::abs(dt - t1) < zero_check) {
			double dis = std::abs(vavg - (v1 + v0) / 2);
			return dis <= r + zero_check;
		}

		// normal check //
		// 
		// 若 va 是从 v0 加速到 v1 的平均速度，vb 是其他过程的平均速度
		// 于是针对平均速度 v_avg，有
		// v_avg * dt = va *t1 + vb*(dt - t1)
		//
		// 继而：
		// vb = （v_avg * dt - va *t1）/ (dt - t1)
		//
		double vavg_left = (vavg * dt - (v0 + v1) / 2 * t1) / (dt - t1);

		// 正常情况需求 vb 偏离  v0 -> v1 这根轴的距离

		double dis_d;
		double* dis = &dis_d;

		if (vavg_left < v0) {
			*dis = v0 - vavg_left;
		}
		else if (vavg_left > v1) {
			*dis = vavg_left - v1;
		}
		else {
			*dis = 0.0;
		}

		return *dis < r;
	};

	// time:   -dt/2    0    dt/2     dt  
	//  pos:           p0             p1
	//  vel:     v0           v1             vend
	//  acc:           a1
	// 
	auto inline ARIS_API s_is_vend(double v0, double v1, double vend, double a_max, double a_min, double dt, double zero_check = 1e-10)->bool {
		return ((v1 - v0) <= a_max * dt + zero_check)
			&& ((v1 - v0) >= a_min * dt - zero_check)
			&& ((vend - v1) <= a_max * dt + zero_check)
			&& ((vend - v1) >= a_min * dt - zero_check);
	};


	//          pa : pos actual value
	//          va : vel actual value
	//          pt : pos target value
	//       v_max : vel max permitted value
	//       v_min : vel min permitted value
	//       a_max : acc max permitted value
	//       a_min : acc min permitted value
	//          dt : time inteval
	//  zero_check : set to 1e-10
	//          pc : next planned pos
	//          vc : next planned vel
	//          ac : next planned acc
	// total_count : tbd, not finished yet
	// 
	// 
	// time: -dt   -dt/2    0    dt/2     dt  
	//  pos:               pa             pc
	//  vel:         va           vc
	//  acc:               ac
	// 
	auto ARIS_API inline s_follow_x(double pa, double va, double pt, double v_max, double v_min, double a_max, double a_min, double dt, double zero_check, double& pc, double& vc, double& ac, Size& total_count)->int {

		// 如果位置方向为负
		if (pt - pa < 0) {
			double pa2 = -pa;
			double pt2 = -pt;
			double va2 = -va;
			double v_max2 = -v_min;
			double v_min2 = -v_max;
			double a_max2 = -a_min;
			double a_min2 = -a_max;
			double pc2, vc2, ac2;

			s_follow_x(pa2, va2, pt2, v_max2, v_min2, a_max2, a_min2, dt, zero_check, pc2, vc2, ac2, total_count);

			pc = -pc2;
			vc = -vc2;
			ac = -ac2;
			return 0;
		}

		// 当前速度超过速度上下限 //
		{
			if (va + a_min * dt > v_max) {
				ac = a_min;
				vc = va + ac * dt;
				pc = pa + vc * dt;
				total_count = std::numeric_limits<std::decay_t<decltype(total_count)>>::max();
				return -1;
			}
			if (va + a_max * dt < v_min) {
				ac = a_max;
				vc = va + ac * dt;
				pc = pa + vc * dt;
				total_count = std::numeric_limits<std::decay_t<decltype(total_count)>>::max();
				return -2;
			}
		}

		// 查看当前速度是否过快 //
		{
			// a_min_real, a_max_real 还需考虑速度不超限
			double a_min_real = std::max((v_min - va) / dt, a_min);
			a_min_real = std::min(a_max, a_min_real);
			double a_max_real = std::min((v_max - va) / dt, a_max);
			a_max_real = std::max(a_min, a_max_real);

			// 【CASE 0】没有减速能力
			if (va + a_max * dt > 0 && a_min > -zero_check) {
				ac = va > 0.0 ? a_min : std::max(-va / dt, a_min);
				vc = va + ac * dt;
				pc = pa + vc * dt;
				total_count = std::numeric_limits<std::decay_t<decltype(total_count)>>::max();
				return 0;
			}
			if (va + a_min * dt < 0 && a_max < zero_check) {
				ac = va < 0.0 ? a_max : std::min(-va/dt, a_max);
				vc = va + ac * dt;
				pc = pa + vc * dt;
				total_count = std::numeric_limits<std::decay_t<decltype(total_count)>>::max();
				return 0;
			}

			// 【CASE 1】已经到达目标位置
			if (s_is_vend(va, (pt - pa) / dt, 0.0, a_max_real, a_min_real, dt, zero_check)) {
				pc = pt;
				vc = (pt - pa) / dt;
				ac = (vc - va) / dt;
				total_count = 0;
				return 0;
			}

			// 【CASE 2】最短的减速段距离，大于当前还剩的距离，最短减速距离还需要考虑圆整
			//     pa      p1      p2  ...     pn(pt)      pt ...
			// va      v1      v2  ...    vn        vend(0)
			//
			//
			// va 经过 n+1 个周期减速
			// pa 经过 n   个周期到pt
			// 
			{
				const double vdec = va;
				auto n = std::ceil(-vdec / a_min / dt - 1.0 - zero_check);
				const double sdec = dt * n * (vdec + dt*(n + 1) / 2 * a_min);
				if (n > 0 && pt - pa - sdec < zero_check) {
					ac = a_min;
					vc = va + ac * dt;
					pc = pa + vc * dt;

					double v1 = va + dt * a_min/2;

					auto Tdec = (v_min - v1) / a_min;
					auto Tacc = (0 - v_min) / a_max;
					auto ldec = Tdec * (v_min + v1) / 2;
					auto lacc = Tacc * (v_min + 0) / 2;
					if (lacc + ldec >= pt - pa) {
						total_count = Tacc + Tdec + (pt - pa - lacc - ldec) / v_min;
					}
					else {
						auto v_max2 = -std::sqrt((pt - pa + v1 * v1 / (2 * a_min)) * 2 * a_min * a_max / (a_max - a_min));
						auto Tacc2 = (v_max2 - v1) / a_min;
						auto Tdec2 = (0 - v_max2) / a_max;
						total_count = (Tacc2 + Tdec2) / dt - 2;
					}

					return 0;
				}
			}
			

			// 【CASE 3】当前还可以用 a_max_real 加速
			{
				const double vdec = va + a_max_real * dt;
				auto n = std::ceil(-vdec / a_min / dt - 1.0 - zero_check);
				const double sdec = dt * n * (vdec + dt * (n + 1) / 2 * a_min);
				if (n < 0 || sdec < pt - (pa + (va + a_max_real * dt) * dt)) {
					ac = a_max_real;
					vc = va + ac * dt;
					pc = pa + vc * dt;
					//////////////////////////  total count 还需推导 /////////////////////////////
					// nacc
					// ndec
					// 
					// COND 1 可以加速到最大速度
					auto Tacc = (v_max - va) / a_max;
					auto Tdec = (0 - v_max) / a_min;
					auto lacc = Tacc * (v_max + va) / 2;
					auto ldec = Tdec * (v_max + 0) / 2;
					if (lacc + ldec <= pt - pa) {
						total_count = Tacc + Tdec + (pt - pa - lacc - ldec)/v_max;
					}
					else {
						//auto Tacc = (v_max - va) / a_max;
						//auto Tdec = (0 - v_max) / a_min;
						//auto lacc = Tacc * (v_max + va) / 2;
						//auto ldec = Tdec * (v_max + 0) / 2;

						// given lacc + ldec == pt - pa, solve v_max
						// => (v_max^2 - va^2) / (2*a_max) - v_max^2/(2*a_min) = pt - pa
						// => v_max^2 * (1/(2*a_max) - 1/(2*a_min)) == pt - pa + va^2 / (2*a_max)
						// => v_max^2 = (pt - pa + va^2 / (2*a_max)) * 4*a_max*a_min/(2*a_min - 2*a_max)
						auto v_max2 = std::sqrt((pt - pa + va*va / (2 * a_max)) * 4 * a_max * a_min / (2 * a_min - 2 * a_max));
						auto Tacc2 = (v_max2 - va) / a_max;
						auto Tdec2 = (0 - v_max2) / a_min;
						total_count = (Tacc2 + Tdec2)/dt - 1;
					}
					
					
					//////////////////////////  total count 还需推导 /////////////////////////////
					return 0;
				}
			}


			// 【CASE 4】当前加速度需要小于 a_max 大于 a_min，减速的步数在 ndec 与 ndec1 之间
			// v_next = -(ndec + 1) * dt * a_min
			// 
			// pt - pa = v_next*dt + v_next * ndec * dt /2
			//         = dt*v_next*(ndec+2)/2
			//         = -dt^2*(ndec + 1)*(ndec+2)*amin/2
			//         = -(ndec^2 + 3*ndec+2)*dt^2*amin/2
			// 
			// =>   ndec^2 + 3*ndec + 2 + 2*(pt-pa)/dt^2/amin == 0
			// 
			// =>   A = 1
			//      B = 3
			//      C = 2 + 2*(pt-pa)/dt^2/amin
			//
			// 下考虑若要元整，此时vn的计算：
			// tn = ceil(ndec) * dt
			// v_next = vn - tn*a_min
			// => 
			// pt - pa = v_next*dt + vn * dt + (v_next + vn)/2*(tn-dt)
			//         = vn *dt - tn*a_min*dt + vn*dt + vn * (tn-dt) - tn*(tn-dt)*a_min/2
			//         = (tn+dt)*vn - amin*(tn*dt + tn*(tn-dt)/2)
			//         = (tn+dt)*vn - amin*(tn*(tn+dt)/2)
			// 
			// => vn = (pt - pa + amin*(tn*(tn+dt)/2))/(tn+dt)
			{
				double C = 2 + 2 * (pt - pa) / dt / dt / a_min;
				double n_ideal = (-3 + std::sqrt(std::max(9.0 - 4.0 * C, 0.0))) / 2.0;
				const double ndec2 = n_ideal;  // 尽量加速稍微低一些

				auto tn = std::ceil(n_ideal - zero_check) * dt;
				double vn = (pt - pa + a_min * (tn * (tn + dt) / 2)) / (tn + dt);
				double v_next = vn - tn * a_min;

				//std::cout << pt - pa - (v_next * dt + vn * dt + (v_next + vn) / 2 * (tn-dt)) << std::endl;

				//double v_next = -(ndec2 + 1) * dt * a_min;
				ac = (v_next - va) / dt;
				vc = va + ac * dt;
				pc = pa + vc * dt;
				total_count = ndec2 + 1;
				return 0;
			}
		}
	}

}



#endif
