#include <algorithm>

#include"aris_plan_function.h"

#include <aris_control.h>
#include <aris_dynamic.h>

namespace aris::plan
{
	auto moveAbsolute(Size i, double begin_pos, double end_pos, double vel, double acc, double dec, double &current_pos, double &current_vel, double &current_acc, Size& total_count)->void
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
			v = std::sqrt(2 * s*a*d / (a + d));
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
}
