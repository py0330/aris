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
		// 设置时间间隔 dt //
		auto setDt(double dt = 1e-3) -> void;


		// 设置最大【线速度】 //
		auto setMaxV(double v = 1.0)->void;
		// 设置最大【角速度】 //
		auto setMaxW(double w = 1.0)->void;
		// 设置最大【线加速度】 //
		auto setMaxA(double a = 1.0)->void;
		// 设置最大【角加速度】 //
		auto setMaxX(double x = 1.0)->void;

		// 设置起始的跟随坐标系的位姿矩阵，因为后续跟随的速度是跟随器算出的，因此一次追踪任务只需设置一次跟随位姿 //
		auto setFollowPm(const double* pm)->void;
		// 设置起始的跟随坐标系的速度与角速度，同上，只用设置一次 //
		auto setFollowVa(const double* va)->void;
		

		// 设置目标坐标系的位姿矩阵 //
		auto setTargetPm(const double* pm)->void;
		// 设置目标坐标系的速度 //
		auto setTargetVa(const double* va)->void;
		// 移动一个步长，并获取结果 //
		auto moveDtAndGetResult(double *follow_pm, double *follow_va)->void;
		// 估算完全追踪上所需要的时间 //
		auto estimateLeftT()->double;

		~MoveFollower();
		MoveFollower();
		ARIS_DELETE_BIG_FOUR(MoveFollower);

	private:
		struct Imp;
		std::unique_ptr<Imp> imp_;
	};




}

#endif