#ifndef ARIS_DYNAMIC_PLAN_H_
#define ARIS_DYNAMIC_PLAN_H_

#include <list>
#include <cmath>
#include <iostream>

#include"aris_dynamic_kernel.h"

namespace Aris
{
	/// \brief 轨迹规划命名空间
	/// \ingroup Aris
	/// 
	///
	///
	namespace Dynamic
	{
		inline auto acc_up(int n, int i)noexcept->double
		{
			return (-1.0 / 2 / n / n / n * i*i*i + 3.0 / 2 / n / n * i*i);
		}
		inline auto acc_down(int n, int i)noexcept->double
		{
			return (-1.0*i*i*i / 2.0 / n / n / n + 3.0 * i*i / 2 / n / n);
		}
		inline auto dec_up(int n, int i)noexcept->double
		{
			return 1 - (-1.0 / 2 / n / n / n * (n - i)*(n - i)*(n - i) + 3.0 / 2 / n / n * (n - i)*(n - i));
		}
		inline auto dec_down(int n, int i)noexcept->double
		{
			return 1 - (-1.0*(n - i)*(n - i)*(n - i) / 2.0 / n / n / n + 3.0 * (n - i)*(n - i) / 2 / n / n);
		}

		inline auto acc_even(int n, int i)noexcept->double
		{
			return 1.0 / n / n  * i * i;
		}
		inline auto dec_even(int n, int i)noexcept->double
		{
			return 1.0 - 1.0 / n / n * (n - i)*(n - i);
		}
		inline auto even(int n, int i)noexcept->double
		{
			return 1.0 / n*i;
		}

		inline auto s_p2p(int n, int i, double begin_pos, double end_pos)noexcept->double
		{
			double a = 4 * (end_pos - begin_pos) / n / n;
			return i <= n / 2 ? 0.5*a*i*i + begin_pos : end_pos - 0.5*a*(n - i)*(n - i);
		}
		inline auto s_v2v(int n, int i, double begin_vel, double end_vel)noexcept->double
		{
			double s = static_cast<double>(i) / n;
			double m = 1 - s;

			return (s*s*s - s*s)*end_vel*n + (m*m - m*m*m)*begin_vel*n;
		}

		inline auto s_interp(int n, int i, double begin_pos, double end_pos, double begin_vel, double end_vel)noexcept->double
		{
			double s = static_cast<double>(i) / n;
			
			double a, b, c, d;

			c = begin_vel*n;
			d = begin_pos;
			a = end_vel*n - 2 * end_pos + c + 2 * d;
			b = end_pos - c - d - a;

			return a*s*s*s+b*s*s+c*s+d;
		}

		class FastPath
		{
		public:
			struct MotionLimit
			{
				double maxVel, minVel, maxAcc, minAcc;
			};
			struct Node
			{
				double time, s, ds, dds;
				bool isAccelerating;
			};
			struct Data
			{
				double * const Ji;
				double * const dJi;
				double * const Cv;
				double * const Ca;
				double * const g;
				double * const h;
				const int size;
				double time, s, ds;
				double dsLhs, dsRhs;
				double ddsLhs, ddsRhs;
			};

			template <typename LimitArray>
			auto setMotionLimit(LimitArray limits)->void
			{
				motor_limits.clear();
				for (auto &limit : limits)
				{
					motor_limits.push_back(limit);
				}
			}
			auto setBeginNode(Node node)->void { beginNode=node; };
			auto setEndNode(Node node)->void { endNode = node; };
			auto setFunction(std::function<void(FastPath::Data &)> getEveryThing)->void { this->getEveryThing = getEveryThing; };
			auto result()->std::vector<double>& { return resultVec; };
			auto run()->void;
			

			FastPath() = default;
			~FastPath() = default;

		private:
			bool computeDsBundPure(FastPath::Data &data, std::vector<FastPath::MotionLimit> &limits);
			bool computeDdsBundPure(FastPath::Data &data, std::vector<FastPath::MotionLimit> &limits);
			bool computeDsBund(FastPath::Data &data, std::vector<FastPath::MotionLimit> &limits);
			bool computeDdsBund(FastPath::Data &data, std::vector<FastPath::MotionLimit> &limits);
			
			int computeForward(std::list<Node>::iterator iter, FastPath::Data &data, int num);
			int computeBackward(std::list<Node>::iterator iter, FastPath::Data &data, int num);
			bool compute(std::list<Node>::iterator iter, FastPath::Data &data);

			void concate(FastPath::Data &data);
		public:
			Node beginNode, endNode;
			std::list<Node> list;
			std::list<Node>::iterator finalIter;
			std::vector<MotionLimit> motor_limits;
			std::function<void(FastPath::Data &)> getEveryThing;

			std::vector<double> resultVec;
		};
	}
}


#endif