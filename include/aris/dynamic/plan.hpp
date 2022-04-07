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
	class ARIS_API OptimalTrajectory :public Element{
	public:
		struct MotionLimit{
			double max_vel{ 0.0 }, min_vel{ 0.0 }, max_acc{ 0.0 }, min_acc{ 0.0 }, max_tor{ 0.0 }, min_tor{ 0.0 }, max_jerk{ 0.0 }, min_jerk{ 0.0 };
		};
		struct Node{
			double time{ 0.0 }, s{ 0.0 }, ds{ 0.0 }, dds{ 0.0 };
		};

		template <typename LimitArray>
		auto setMotionLimit(LimitArray limits)->void{
			motor_limits.clear();
			for (auto &limit : limits)
				motor_limits.push_back(limit);
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



		double failed_s{ 0.0 };

		PathPlan plan;
		UniversalSolver* solver{ nullptr };

		Node beg_, end_;
		std::list<Node> list;
		std::list<Node>::iterator l_beg_, l_;
		std::vector<MotionLimit> motor_limits;

		std::vector<double> Ji_data_;

		std::vector<double> result_;
	};

	class ARIS_API FastPath{
	public:
		struct MotionLimit{
			double maxVel{ 0.0 }, minVel{ 0.0 }, maxAcc{ 0.0 }, minAcc{ 0.0 };
		};
		struct Node{
			double time{ 0.0 }, s{ 0.0 }, ds{ 0.0 }, dds{ 0.0 };
			bool isAccelerating{ true };
		};
		struct Data{
			double * const Ji{nullptr};
			double * const dJi{ nullptr };
			double * const Cv{ nullptr };
			double * const Ca{ nullptr };
			double * const g{ nullptr };
			double * const h{ nullptr };
			const Size size{0};
			double time{ 0.0 }, s{ 0.0 }, ds{ 0.0 };
			double dsLhs{ 0.0 }, dsRhs{ 0.0 };
			double ddsLhs{ 0.0 }, ddsRhs{ 0.0 };
		};

		template <typename LimitArray>
		auto setMotionLimit(LimitArray limits)->void{
			motor_limits.clear();
			for (auto &limit : limits){
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