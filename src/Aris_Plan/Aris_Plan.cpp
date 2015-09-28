#include <algorithm>

#include"Aris_Plan.h"


namespace Aris
{
	namespace Plan
	{
		bool ComputeBund(FAST_PATH::DATA &data, std::vector<FAST_PATH::MOTOR_LIMIT> &limits)
		{
			static std::vector<double> J_dot_g(data.size), J_dot_h(data.size);
			static std::vector<double> lhs(data.size * 2), rhs(data.size * 2);

			std::fill(J_dot_g.begin(), J_dot_g.end(), 0);
			std::fill(J_dot_h.begin(), J_dot_h.end(), 0);
			std::fill(lhs.begin(), lhs.end(), 0);
			std::fill(rhs.begin(), rhs.end(), 0);

			Aris::DynKer::s_dgemm(data.size, 1, data.size, 1, data.Ji, data.size, data.g, 1, 0, J_dot_g.data(), 1);
			Aris::DynKer::s_dgemm(data.size, 1, data.size, 1, data.Ji, data.size, data.h, 1, 0, J_dot_h.data(), 1);

			double dt = 0.001;

			for (int i = 0; i < data.size; ++i)
			{
				lhs[i] = (limits[i].minAcc - data.Ca[i] - J_dot_h[i] * data.ds * data.ds) / J_dot_g[i];
				rhs[i] = (limits[i].maxAcc - data.Ca[i] - J_dot_h[i] * data.ds * data.ds) / J_dot_g[i];
				lhs[i + data.size] = (limits[i].minVel - data.Cv[i] - J_dot_g[i] * data.ds) / J_dot_g[i] / dt;
				rhs[i + data.size] = (limits[i].maxVel - data.Cv[i] - J_dot_g[i] * data.ds) / J_dot_g[i] / dt;
				
				if (J_dot_g[i]<0)
				{
					std::swap(lhs[i], rhs[i]);
					std::swap(lhs[i + data.size], rhs[i + data.size]);
				}
			}

			data.lhs = *std::max_element(lhs.begin(), lhs.end());
			data.rhs = *std::min_element(rhs.begin(), rhs.end());

			return ((data.lhs < -2) && (data.rhs > data.lhs));
		}

		double dt = 0.001;


		bool FAST_PATH::Compute(std::list<NODE>::iterator iter, FAST_PATH::DATA &data)
		{
			auto r_iter = std::next(iter);
			
			if ((list.back().s - list.front().s)*(r_iter->s - iter->s) < 0)
			{
				return true;
			}
			
			auto c_iter = (std::abs(iter->ds) < std::abs(r_iter->ds)) ? iter : r_iter;
			data.time = c_iter->time;
			data.s = c_iter->s;
			data.ds = c_iter->ds;
			getEveryThing(data);

			static int count{ 0 };
			if ((count>1400)&& (count % 100 == 0))
			{
				int i=0;
				++i;
				std::cout << this->list.size() << std::endl;
			}
			count++;


			if (c_iter == iter)
			{
				if (ComputeBund(data, this->motor_limits))
				{
					static FAST_PATH::NODE node;
					double lhs = data.lhs;
					double rhs = data.rhs;
					/*首先尝试加速*/
					if (lhs < -4)
					{
						iter->isAccelerating = true;
						iter->dds = rhs;

						node.time = iter->time + dt;
						node.ds = iter->ds + iter->dds * dt;
						node.s = iter->s + iter->ds * dt + 0.5 * iter->dds * dt*dt;

						this->list.insert(std::next(iter), node);



						if (Compute(std::next(iter), data))
						{
							return true;
						}
						else
						{
							this->list.erase(std::next(iter));
						}
					}


					

					/*若没有返回，那么尝试减速*/
					iter->isAccelerating = false;
					iter->dds = lhs;

					node.time = iter->time + dt;
					node.ds = iter->ds + iter->dds * dt;
					node.s = iter->s + iter->ds * dt + 0.5 * iter->dds * dt*dt;

					this->list.insert(std::next(iter), node);

					if (Compute(std::next(iter), data))
					{
						return true;
					}
					else
					{
						this->list.erase(std::next(iter));
					}

					return false;
				}
				else
				{
					return false;
				};
			}
			else
			{
				if (ComputeBund(data, this->motor_limits))
				{
					static FAST_PATH::NODE node;
					double lhs = data.lhs;
					double rhs = data.rhs;
					/*首先尝试减速*/
					if (rhs > 2)
					{
						r_iter->isAccelerating = false;
						r_iter->dds = lhs;

						node.time = r_iter->time - dt;
						node.ds = r_iter->ds - r_iter->dds * dt;
						node.s = r_iter->s - r_iter->ds * dt - 0.5 * r_iter->dds * dt*dt;

						this->list.insert(r_iter, node);

						if (Compute(iter, data))
						{
							return true;
						}
						else
						{
							this->list.erase(std::prev(r_iter));
						}
					}



					

					/*若没有返回，那么尝试减速*/
					r_iter->isAccelerating = true;
					r_iter->dds = rhs;

					node.time = r_iter->time - dt;
					node.ds = r_iter->ds - r_iter->dds * dt;
					node.s = r_iter->s - r_iter->ds * dt - 0.5 * r_iter->dds * dt*dt;

					this->list.insert(r_iter, node);

					if (Compute(iter, data))
					{
						return true;
					}
					else
					{
						this->list.erase(std::prev(r_iter));
					}

					return false;
				}
				else
				{
					return false;
				};
			}

			


		}

		void FAST_PATH::Run()
		{
			const int size = motor_limits.size();
			std::vector<double> Ji(size*size), Cv(size), Ca(size), g(size), h(size);
			
			FAST_PATH::DATA data{ Ji.data(),Cv.data() ,Ca.data() ,g.data() ,h.data(),size };

			list.erase(++list.begin(), --list.end());
			iter = list.begin();

			this->Compute(iter,data);

			//double dt = 0.001;

			//while ((bL.back().s - fL.front().s)*(bL.front().s - fL.back().s)>0)
			//{
			//	if (std::abs(fL.back().ds) > std::abs(bL.front().ds))
			//	{
			//		data.time = bL.front().time;
			//		data.s = bL.front().s;
			//		data.ds = bL.front().ds;
			//		this->getEveryThing(data);
			//		ComputeBund(data, this->motor_limits);
			//		
			//	}
			//	else
			//	{
			//		data.time = fL.back().time;
			//		data.s = fL.back().s;
			//		data.ds = fL.back().ds;
			//		this->getEveryThing(data);
			//		ComputeBund(data, this->motor_limits);
			//	}





			//}



		}


	}
}