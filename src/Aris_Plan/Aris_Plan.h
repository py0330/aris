#ifndef ARIS_PLAN_H_
#define ARIS_PLAN_H_

#include"Aris_DynKer.h"

#include <list>
#include <cmath>
#include <iostream>

namespace Aris
{
	/// \brief 轨迹规划命名空间
	/// \ingroup Aris
	/// 
	///
	///
	namespace Plan
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
			void SetMotorLimit(LimitArray limits)
			{
				motor_limits.clear();
				for (auto &limit : limits)
				{
					motor_limits.push_back(limit);
				}
			}
			void SetBeginNode(Node node) { beginNode=node; };
			void SetEndNode(Node node) { endNode = node; };
			void SetFunction(std::function<void(FastPath::Data &)> getEveryThing) { this->getEveryThing = getEveryThing; };
			std::vector<double> &Result() { return resultVec; };
			void Run();
			

			FastPath() = default;
			~FastPath() = default;

		private:
			bool ComputeDsBundPure(FastPath::Data &data, std::vector<FastPath::MotionLimit> &limits);
			bool ComputeDdsBundPure(FastPath::Data &data, std::vector<FastPath::MotionLimit> &limits);
			bool ComputeDsBund(FastPath::Data &data, std::vector<FastPath::MotionLimit> &limits);
			bool ComputeDdsBund(FastPath::Data &data, std::vector<FastPath::MotionLimit> &limits);
			
			int ComputeForward(std::list<Node>::iterator iter, FastPath::Data &data, int num);
			int ComputeBackward(std::list<Node>::iterator iter, FastPath::Data &data, int num);
			bool Compute(std::list<Node>::iterator iter, FastPath::Data &data);

			void Concate(FastPath::Data &data);
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