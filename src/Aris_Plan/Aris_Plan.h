#ifndef ARIS_PLAN_H_
#define ARIS_PLAN_H_

#include"Aris_DynKer.h"

#include <list>
#include <cmath>
#include <iostream>

namespace Aris
{
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

		class FAST_PATH
		{
		public:
			struct MOTOR_LIMIT
			{
				double maxVel, minVel, maxAcc, minAcc;
			};
			struct NODE
			{
				double time, s, ds, dds;
				bool isAccelerating;
			};
			struct DATA
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


			template <typename LIMIT_ARRAY>
			void SetMotorLimit(LIMIT_ARRAY limits)
			{
				motor_limits.clear();
				for (auto &limit : limits)
				{
					motor_limits.push_back(limit);
				}
			}
			void SetBeginNode(NODE node) { beginNode=node; };
			void SetEndNode(NODE node) { endNode = node; };
			void SetFunction(std::function<void(FAST_PATH::DATA &)> getEveryThing) { this->getEveryThing = getEveryThing; };
			std::vector<double> &Result() { return resultVec; };
			void Run();
			

			FAST_PATH() = default;
			~FAST_PATH() = default;

		private:
			bool ComputeDsBundPure(FAST_PATH::DATA &data, std::vector<FAST_PATH::MOTOR_LIMIT> &limits);
			bool ComputeDdsBundPure(FAST_PATH::DATA &data, std::vector<FAST_PATH::MOTOR_LIMIT> &limits);
			bool ComputeDsBund(FAST_PATH::DATA &data, std::vector<FAST_PATH::MOTOR_LIMIT> &limits);
			bool ComputeDdsBund(FAST_PATH::DATA &data, std::vector<FAST_PATH::MOTOR_LIMIT> &limits);
			
			int ComputeForward(std::list<NODE>::iterator iter, FAST_PATH::DATA &data, int num);
			int ComputeBackward(std::list<NODE>::iterator iter, FAST_PATH::DATA &data, int num);
			bool Compute(std::list<NODE>::iterator iter, FAST_PATH::DATA &data);

			void Concate(FAST_PATH::DATA &data);
		public:
			NODE beginNode, endNode;
			std::list<NODE> list;
			std::list<NODE>::iterator finalIter;
			std::vector<MOTOR_LIMIT> motor_limits;
			std::function<void(FAST_PATH::DATA &)> getEveryThing;

			std::vector<double> resultVec;
		};
	}
}


#endif