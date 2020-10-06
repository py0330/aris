#ifndef ARIS_DYNAMIC_PLAN_H_
#define ARIS_DYNAMIC_PLAN_H_

#include <list>
#include <cmath>
#include <iostream>

#include <aris/dynamic/model.hpp>
#include <aris/dynamic/matrix.hpp>

namespace aris::dynamic
{
	using PathPlan = std::function<void(double s, double *pm, double *vs_over_s, double *as_over_s)>;
	class ARIS_API OptimalTrajectory :public Element
	{
	public:
		struct MotionLimit
		{
			double max_vel, min_vel, max_acc, min_acc, max_tor, min_tor, max_jerk, min_jerk;
		};
		struct Node
		{
			double time, s, ds, dds;
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
		auto setBeginNode(Node node)->void { beg_ = node; }
		auto setEndNode(Node node)->void { end_ = node; }
		auto setFunction(const PathPlan &path_plan)->void { this->plan = path_plan; }
		auto setSolver(UniversalSolver *solver)->void { this->solver = solver; }
		auto result()->std::vector<double>& { return result_; }
		auto run()->void;

		virtual ~OptimalTrajectory() = default;
		explicit OptimalTrajectory() {}
		OptimalTrajectory(const OptimalTrajectory&) = default;
		OptimalTrajectory(OptimalTrajectory&&) = default;
		OptimalTrajectory& operator=(const OptimalTrajectory&) = default;
		OptimalTrajectory& operator=(OptimalTrajectory&&) = default;

	public:
		auto testForward()->void;
		auto join()->void;
		auto cptDdsConstraint(double s, double ds, double &max_dds, double &min_dds)->bool;
		auto cptInverseJacobi()->void;



		double failed_s;

		PathPlan plan;
		UniversalSolver *solver;

		Node beg_, end_;
		std::list<Node> list;
		std::list<Node>::iterator l_beg_, l_;
		std::vector<MotionLimit> motor_limits;

		std::vector<double> Ji_data_;

		std::vector<double> result_;
	};

	class ARIS_API FastPath
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
			const Size size;
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
		auto setBeginNode(Node node)->void { beginNode = node; };
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


#endif