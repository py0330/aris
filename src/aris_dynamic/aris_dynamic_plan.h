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
		inline double acc_up(int n, int i)
		{
			return (-1.0 / 2 / n / n / n * i*i*i + 3.0 / 2 / n / n * i*i);
		}
		inline double acc_down(int n, int i)
		{
			return (-1.0*i*i*i / 2.0 / n / n / n + 3.0 * i*i / 2 / n / n);
		}
		inline double dec_up(int n, int i)
		{
			return 1 - (-1.0 / 2 / n / n / n * (n - i)*(n - i)*(n - i) + 3.0 / 2 / n / n * (n - i)*(n - i));
		}
		inline double dec_down(int n, int i)
		{
			return 1 - (-1.0*(n - i)*(n - i)*(n - i) / 2.0 / n / n / n + 3.0 * (n - i)*(n - i) / 2 / n / n);
		}

		inline double acc_even(int n, int i)
		{
			return 1.0 / n / n  * i * i;
		}
		inline double dec_even(int n, int i)
		{
			return 1.0 - 1.0 / n / n * (n - i)*(n - i);
		}
		inline double even(int n, int i)
		{
			return 1.0 / n*i;
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
			void setMotionLimit(LimitArray limits)
			{
				motor_limits.clear();
				for (auto &limit : limits)
				{
					motor_limits.push_back(limit);
				}
			}
			void setBeginNode(Node node) { beginNode=node; };
			void setEndNode(Node node) { endNode = node; };
			void setFunction(std::function<void(FastPath::Data &)> getEveryThing) { this->getEveryThing = getEveryThing; };
			std::vector<double> &result() { return resultVec; };
			void run();
			

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