#include <algorithm>

#include"aris/plan/function.hpp"
#include <aris/control/control.hpp>
#include <aris/dynamic/dynamic.hpp>

namespace aris::plan
{
	auto moveAbsolute(double i, double begin_pos, double end_pos, double vel, double acc, double dec, double &current_pos, double &current_vel, double &current_acc, Size& total_count)->void
	{
		auto v = std::abs(vel);
		auto a = std::abs(acc);
		auto d = std::abs(dec);
		auto s = std::abs(end_pos - begin_pos);

		Size n1 = static_cast<Size>(std::ceil(v / a));
		Size n3 = static_cast<Size>(std::ceil(v / d));

		a = v / n1;
		d = v / n3;

		double s1 = a * n1 * n1 / 2.0;
		double s3 = d * n3 * n3 / 2.0;
		double s2 = s - s1 - s3;

		// 判断是否有匀速段
		if (s2 > 0)
		{
			Size n2 = static_cast<Size>(std::ceil(s2 / v));
			double coe = s / (s1 + v * n2 + s3);

			a *= coe;
			d *= coe;
			v *= coe;
			s1 *= coe;
			s2 *= coe;
			s3 *= coe;

			total_count = n1 + n2 + n3;
			if (i < n1)
			{
				current_pos = a * i * i / 2.0;
				current_vel = a * i;
				current_acc = a;
			}
			else if (i < n1 + n2)
			{
				current_pos = s1 + v * (i - n1);
				current_vel = v;
				current_acc = 0.0;
			}
			else if (i < n1 + n2 + n3)
			{
				current_pos = s - d * (n1 + n2 + n3 - i) * (n1 + n2 + n3 - i) / 2.0;
				current_vel = d * (n1 + n2 + n3 - i);
				current_acc = -d;
			}
			else
			{
				current_pos = s;
				current_vel = 0;
				current_acc = 0;
			}
		}
		else
		{
			v = std::sqrt(2 * s * a * d / (a + d));
			n1 = static_cast<Size>(std::ceil(v / a));
			n3 = static_cast<Size>(std::ceil(v / d));

			double coe = s / (a*n1*n1 / 2.0 + d * n3*n3 / 2.0);
			a *= coe;
			d *= coe;

			total_count = n1 + n3;
			if (i < n1)
			{
				current_pos = a * i * i / 2.0;
				current_vel = a * i;
				current_acc = a;
			}
			else if (i < n1 + n3)
			{
				current_pos = s - d * (n1 + n3 - i) * (n1 + n3 - i) / 2.0;
				current_vel = d * (n1 + n3 - i);
				current_acc = -d;
			}
			else
			{
				current_pos = s;
				current_vel = 0;
				current_acc = 0;
			}

		}

		// 修正位置、速度、加速度方向
		if (end_pos < begin_pos)
		{
			current_pos = begin_pos - current_pos;
			current_vel = -current_vel;
			current_acc = -current_acc;
		}
		else
		{
			current_pos = begin_pos + current_pos;
			current_vel = current_vel;
			current_acc = current_acc;
		}
	}
	auto moveAbsolute2(double pa, double va, double aa, double pt, double vt, double at, double vm, double am, double dm, double dt, double zero_check, double &pc, double &vc, double &ac, Size& total_count)->int
	{
		vt = 0.0;
		at = 0.0;

		// 当前速度超过速度上限 //
		if (std::abs(va) > vm + dm * dt)
		{
			ac = -aris::dynamic::s_sgn2(va) * dm;
			goto return_flag;
		}
		
		// 查看当前速度是否过快 //
		{
			// 计算完全减速所产生的位移，vdec和sdec有方向 //
			const double vdec = va;
			const double ndec = std::abs(std::ceil(std::abs(vdec) / dm / dt - zero_check));
			const double tdec = ndec * dt;
			const double sdec = vdec * tdec / 2;

			// 当前速度需要完全减速到零，之后可能还需要反转 //
			if ((va > zero_check && (pt - pa) < sdec + zero_check) || (va < -zero_check && (pt - pa) > sdec - zero_check))
			{
				ac = -va / ndec / dt;
				goto return_flag;
			}
		}
		
		// 二分法算当前可以最快的加速度，并沿着该加速度加速 //
		{
			double lower_bound = pt - pa < 0.0 ? std::min(dm, -va / dt) : std::max(-dm, -va / dt);
			double upper_bound = pt - pa < 0.0 ? std::max(-am, (-vm - va) / dt) : std::min(am, (vm - va) / dt);

			while (std::abs(lower_bound - upper_bound) > zero_check)
			{
				const double a1 = (lower_bound + upper_bound) / 2.0;
				const double v1 = va + a1 * dt;
				const double p1 = pa + 0.5*(va + v1)*dt;
				const double ndec1 = std::abs(std::ceil(std::abs(v1) / dm / dt));
				const double tdec1 = ndec1 * dt;
				const double sdec1 = v1 * tdec1 / 2.0;

				if ((pt - pa >= 0 && (pt - p1) >= sdec1) || (pt - pa < 0 && (pt - p1) <= sdec1))
				{
					lower_bound = a1;
				}
				else
				{
					upper_bound = a1;
				}
			}
			ac = lower_bound;
		}

	return_flag:
		vc = va + ac * dt;
		pc = pa + 0.5 * (va + vc) * dt;
		total_count = 1;
		return std::abs(pt - pc)<zero_check && std::abs(vt - vc)<zero_check ? 0 : 1;
	}
}
