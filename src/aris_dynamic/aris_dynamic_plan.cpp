#include <algorithm>

#include"aris_dynamic_plan.h"

namespace Aris
{
	namespace Dynamic
	{
		const double dt = 0.001;
		
		/*只根据电机最大速度和最小速度来计算ds的最大和最小值*/
		bool FastPath::computeDsBundPure(FastPath::Data &data, std::vector<FastPath::MotionLimit> &limits)
		{
			static std::vector<double> J_dot_g(data.size);
			static std::vector<double> lhs(data.size), rhs(data.size);

			std::fill(J_dot_g.begin(), J_dot_g.end(), 0);
			std::fill(lhs.begin(), lhs.end(), 0);
			std::fill(rhs.begin(), rhs.end(), 0);

			Aris::Dynamic::s_dgemm(data.size, 1, data.size, 1, data.Ji, data.size, data.g, 1, 0, J_dot_g.data(), 1);

			for (int i = 0; i < data.size; ++i)
			{
				lhs[i] = (limits[i].minVel - data.Cv[i]) / J_dot_g[i];
				rhs[i] = (limits[i].maxVel - data.Cv[i]) / J_dot_g[i];

				if (J_dot_g[i]<0)
				{
					std::swap(lhs[i], rhs[i]);
				}
			}

			data.dsLhs = *std::max_element(lhs.begin(), lhs.end());
			data.dsRhs = *std::min_element(rhs.begin(), rhs.end());

			return (data.dsLhs < data.dsRhs);
		}
		/*只根据电机的最大最小加速度和当前ds来计算dds的最大和最小值*/
		bool FastPath::computeDdsBundPure(FastPath::Data &data, std::vector<FastPath::MotionLimit> &limits)
		{
			auto locData = data;
			
			static std::vector<double> J_dot_g(locData.size), J_dot_h(locData.size), dJ_dot_g(locData.size);
			static std::vector<double> lhs(locData.size), rhs(locData.size);

			std::fill(J_dot_g.begin(), J_dot_g.end(), 0);
			std::fill(J_dot_h.begin(), J_dot_h.end(), 0);
			std::fill(dJ_dot_g.begin(), dJ_dot_g.end(), 0);
			std::fill(lhs.begin(), lhs.end(), 0);
			std::fill(rhs.begin(), rhs.end(), 0);

			Aris::Dynamic::s_dgemm(locData.size, 1, locData.size, 1, locData.Ji, locData.size, locData.g, 1, 0, J_dot_g.data(), 1);
			Aris::Dynamic::s_dgemm(locData.size, 1, locData.size, 1, locData.Ji, locData.size, locData.h, 1, 0, J_dot_h.data(), 1);
			Aris::Dynamic::s_dgemm(locData.size, 1, locData.size, 1, locData.dJi, locData.size, locData.g, 1, 0, dJ_dot_g.data(), 1);

			for (int i = 0; i < locData.size; ++i)
			{
				lhs[i] = (limits[i].minAcc - J_dot_h[i] * locData.ds * locData.ds - dJ_dot_g[i] * locData.ds - locData.Ca[i]) / J_dot_g[i];
				rhs[i] = (limits[i].maxAcc - J_dot_h[i] * locData.ds * locData.ds - dJ_dot_g[i] * locData.ds - locData.Ca[i]) / J_dot_g[i];

				if (J_dot_g[i]<0)
				{
					std::swap(lhs[i], rhs[i]);
				}
			}

			data.ddsLhs = *std::max_element(lhs.begin(), lhs.end());
			data.ddsRhs = *std::min_element(rhs.begin(), rhs.end());

			return ((data.ddsLhs < 0) && (data.ddsRhs > 0));
		}
		/*考虑dds是否有合法取值的情况下，来计算ds的取值范围*/
		bool FastPath::computeDsBund(FastPath::Data &data, std::vector<FastPath::MotionLimit> &limits)
		{
			const double errorBund = 1e-7;
			
			auto locData = data;

			if (!computeDsBundPure(locData, limits))return false;

			/*迭代计算下限*/
			locData.ds = locData.dsLhs;
			//如果不成功，那么2分法求解
			if (!computeDdsBundPure(locData, limits))
			{
				double upper = 0;
				double lower = locData.dsLhs;

				while (std::abs(upper - lower) > errorBund)
				{
					double mid = (upper + lower) / 2;
					locData.ds = mid;

					if (computeDdsBundPure(locData, limits))
					{
						upper = mid;
					}
					else
					{
						lower = mid;
					}
				}

				locData.dsLhs = upper;
			}

			/*迭代计算上限*/
			locData.ds = locData.dsRhs;
			//如果不成功，那么2分法求解
			if (!computeDdsBundPure(locData, limits))
			{
				double upper = locData.dsRhs;
				double lower = 0;

				while (std::abs(upper - lower) > errorBund)
				{
					double mid = (upper + lower) / 2;
					locData.ds = mid;

					if (computeDdsBundPure(locData, limits))
					{
						lower = mid;
					}
					else
					{
						upper = mid;
					}
				}

				locData.dsRhs = lower;
			}

			locData.ds = locData.dsLhs;
			if (!computeDdsBundPure(locData, limits))return false;
			locData.ds = locData.dsRhs;
			if (!computeDdsBundPure(locData, limits))return false;


			data.dsLhs = locData.dsLhs;
			data.dsRhs = locData.dsRhs;

			return true;
		}
		/*考虑下一时刻ds能否让dds取到合法值的dds范围*/
		bool FastPath::computeDdsBund(FastPath::Data &data, std::vector<FastPath::MotionLimit> &limits)
		{
			auto locData = data;

			if (!computeDdsBundPure(locData, limits))return false;

			if (!computeDsBund(locData, limits))return false;

			double ddsRhs2 = (locData.dsRhs - locData.ds) / dt;
			double ddsLhs2 = (locData.dsLhs - locData.ds) / dt;

			data.ddsRhs = std::min(locData.ddsRhs, ddsRhs2);
			data.ddsLhs = std::max(locData.ddsLhs, ddsLhs2);

			return ((data.ddsLhs < 0) && (data.ddsRhs > 0));
		}

		/*经过多少个减速后加速*/
		int FastPath::computeForward(std::list<Node>::iterator iter, FastPath::Data &data, int num)
		{
			/*进行n个减速*/
			for (int i = 0; i < num; ++i)
			{
				auto c_iter = std::prev(iter, num - i);
				data.time = c_iter->time;
				data.s = c_iter->s;
				data.ds = c_iter->ds;
				getEveryThing(data);

				if (!computeDdsBund(data, this->motor_limits))return (num + 1);

				c_iter->isAccelerating = false;
				c_iter->dds = data.ddsLhs;

				std::next(c_iter)->ds = c_iter->ds + c_iter->dds * dt;
				std::next(c_iter)->s = c_iter->s + c_iter->ds * dt + 0.5 * c_iter->dds * dt*dt;
			}

			/*减速完了在最后一个结点进行加速*/
			data.time = iter->time;
			data.s = iter->s;
			data.ds = iter->ds;
			getEveryThing(data);

			if (!computeDdsBund(data, this->motor_limits))return (num + 1);

			iter->isAccelerating = true;
			iter->dds = data.ddsRhs;

			std::next(iter)->time = iter->time + dt;
			std::next(iter)->ds = iter->ds + iter->dds * dt;
			std::next(iter)->s = iter->s + iter->ds * dt + 0.5 * iter->dds * dt*dt;
			return 0;
		}
		int FastPath::computeBackward(std::list<Node>::iterator iter, FastPath::Data &data, int num)
		{
			/*进行n个加速*/
			for (int i = 0; i < num; ++i)
			{
				
				auto c_iter = std::next(iter, num - i);
				data.time = c_iter->time;
				data.s = c_iter->s;
				data.ds = c_iter->ds;
				getEveryThing(data);

				if (!computeDdsBund(data, this->motor_limits))return (num + 1);

				c_iter->isAccelerating = true;
				c_iter->dds = data.ddsRhs;

				std::prev(c_iter)->ds = c_iter->ds - c_iter->dds * dt;
				//note here is + 0.5*a*dt*dt
				std::prev(c_iter)->s = c_iter->s - c_iter->ds * dt + 0.5 * c_iter->dds * dt*dt;
				

			}

			/*加速完了在最后一个结点上进行减速*/
			data.time = iter->time;
			data.s = iter->s;
			data.ds = iter->ds;
			getEveryThing(data);

			if (!computeDdsBund(data, this->motor_limits))return (num + 1);

			iter->isAccelerating = false;
			iter->dds = data.ddsLhs;

			std::prev(iter)->time = iter->time - dt;
			std::prev(iter)->ds = iter->ds - iter->dds * dt;
			std::prev(iter)->s = iter->s - iter->ds * dt + 0.5 * iter->dds * dt*dt;
			return 0;
		}
		bool FastPath::compute(std::list<Node>::iterator iter, FastPath::Data &data)
		{
			/*获取反向迭代器的位置，即为正向迭代器的下一个位置*/
			auto r_iter = std::next(iter);

			/*迭代终止条件*/
			if ((list.back().s - list.front().s)*(r_iter->s - iter->s) < 0)
			{
				this->finalIter = iter;
				return true;
			}
			
/////////////////////debug
			static int count{ 0 };
			if ((list.size()>0)&& (count % 100 == 0))
			{
				int i = 0;
				++i;
				std::cout << this->list.size() << std::endl;
			}
			count++;
/////////////////////debug

			/*新添加一个结点*/
			static FastPath::Node node;
			this->list.insert(r_iter, node);

			/*判断从前添加还是从后添加*/
			if (iter->ds < r_iter->ds)
			{
				int ret = 0;
				while((ret = computeForward(iter, data, ret)) != 0);
				return compute(std::next(iter), data);
			}
			else
			{
				int ret = 0;
				while ((ret = computeBackward(r_iter, data, ret)) != 0);
				return compute(iter, data);
			}

			
		}
		
		void FastPath::concate(FastPath::Data &data)
		{
			static FastPath::Node node;

			/*使得双方速度接近*/
			if (finalIter->ds < std::next(finalIter)->ds)
			{
				while (finalIter->ds < std::next(finalIter)->ds)
				{
					this->list.erase(std::next(finalIter));

					while (std::next(finalIter)->s > (finalIter->s + finalIter->ds * dt))
					{
						this->list.insert(std::next(finalIter), node);
						int ret = 0;
						while ((ret = computeForward(finalIter, data, ret)) != 0);
						finalIter = ++finalIter;
					}
				}
			}
			else
			{
				while (finalIter->ds >= std::next(finalIter)->ds)
				{
					this->list.erase(finalIter--);

					while (finalIter->s < (std::next(finalIter)->s - std::next(finalIter)->ds * dt))
					{
						auto r_iter = std::next(finalIter);
						this->list.insert(r_iter, node);
						int ret = 0;
						while ((ret = computeBackward(r_iter, data, ret)) != 0);
					}
				}
			}

			/*使得双方位置接近*/
			double adjust_s = finalIter->s + (finalIter->ds + std::next(finalIter)->ds) / 2 * dt - std::next(finalIter)->s;

			for (auto p = std::next(finalIter); p != list.end(); ++p)
			{
				p->s += adjust_s;
			}

			/*使得双方时间一致*/
			int size = std::lround(this->list.back().time / dt);
			resultVec.resize(size);
			for (int i = 0; i < size; ++i)
			{
				double id = static_cast<double>(i + 1) * (this->list.size() - 1) / size;

				int down = std::lround(std::floor(id));
				int up = std::lround(std::ceil(id));

				down = std::max(down, 0);
				down = std::min(down, static_cast<int>(this->list.size()) - 1);
				up = std::max(up, 0);
				up = std::min(up, static_cast<int>(this->list.size()) - 1);


				double alpha = up - id;
				double beta = 1.0 - alpha;

				resultVec.at(i) = std::next(list.begin(), down)->s * alpha + std::next(list.begin(), up)->s * beta;
			}

			/*使得首尾位置严格符合设置值*/
			double ratio = (endNode.s - beginNode.s) / (resultVec.back() - beginNode.s);
			double beginValue = beginNode.s;

			for (int i = 0; i < size; ++i)
			{
				resultVec.at(i) = beginValue + (resultVec.at(i)-beginValue)*ratio;
			}


		}

		void FastPath::run()
		{
			const int size = motor_limits.size();
			std::vector<double> Ji(size*size), dJi(size*size), Cv(size), Ca(size), g(size), h(size);
			
			FastPath::Data data{ Ji.data(),dJi.data(),Cv.data() ,Ca.data() ,g.data() ,h.data(), size };

			list.clear();
			list.push_front(beginNode);
			list.push_back(endNode);

			std::cout << "begin to compute" << std::endl;

			this->compute(list.begin(),data);

			std::cout << "begin to concate" << std::endl;

			this->concate(data);
		}


	}
}
