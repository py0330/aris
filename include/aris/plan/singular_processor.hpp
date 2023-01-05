#ifndef ARIS_PLAN_SINGULAR_PROCESSOR_H_
#define ARIS_PLAN_SINGULAR_PROCESSOR_H_

#include <list>
#include <cmath>
#include <iostream>
#include <functional>
#include <map>
#include <any>

#include <aris/core/object.hpp>
#include <aris/plan/trajectory.hpp>
#include <aris/dynamic/model_base.hpp>

/// \brief 轨迹规划命名空间
/// \ingroup aris
/// 
///
///
/// 
/// 
namespace aris::plan{
	class ARIS_API SingularProcessor {
	public:
		auto checkIfBeyondMaxVel()->int;
		auto setModelPosAndMoveDt()->int;

		auto setModel(aris::dynamic::ModelBase& model)->void;
		auto setTrajectoryGenerator(TrajectoryGenerator& tg)->void;
		auto setMaxVels(const std::vector<double> max_vels)->void;
		auto setMaxAccs(const std::vector<double> max_accs)->void;
		

		~SingularProcessor();
		SingularProcessor();
		ARIS_DELETE_BIG_FOUR(SingularProcessor);

	private:
		struct Imp;
		std::unique_ptr<Imp> imp_;
	};

	class ARIS_API TrajectoryModelAdapter {
	public:
		auto checkIfBeyondMaxVel()->int;
		auto setModelPosAndMoveDt()->int;

		auto setModel(aris::dynamic::ModelBase& model)->void;
		auto setTrajectoryGenerator(TrajectoryGenerator& tg)->void;
		auto setMaxVels(const std::vector<double> max_vels)->void;
		auto setMaxAccs(const std::vector<double> max_accs)->void;


		~TrajectoryModelAdapter();
		TrajectoryModelAdapter();
		ARIS_DELETE_BIG_FOUR(TrajectoryModelAdapter);

	private:
		struct Imp;
		std::unique_ptr<Imp> imp_;
	};




}

#endif