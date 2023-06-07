#ifndef ARIS_PLAN_MOVE_FOLLOWER_H_
#define ARIS_PLAN_MOVE_FOLLOWER_H_

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
	class ARIS_API MoveFollower {
	public:
		auto setMaxV(double v = 1.0)->void;
		auto setMaxW(double w = 1.0)->void;
		auto setMaxA(double a = 1.0)->void;
		auto setMaxX(double x = 1.0)->void;

		auto setInitTargetPm(const double* pm)->void;
		auto setInitFollowPm(const double* pm)->void;
		auto setInitTargetVa(const double* va)->void;
		auto setInitFollowVa(const double* va)->void;
		auto estimateLeftT()->double;

		auto setTargetPm(const double* pm)->void;
		auto setTargetVa(const double* va)->void;
		auto moveDtAndGetResult();

		~MoveFollower();
		MoveFollower();
		ARIS_DELETE_BIG_FOUR(MoveFollower);

	private:
		struct Imp;
		std::unique_ptr<Imp> imp_;
	};




}

#endif