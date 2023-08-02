#ifndef ARIS_PLAN_TRAJECTORY_H_
#define ARIS_PLAN_TRAJECTORY_H_

#include <list>
#include <cmath>
#include <iostream>
#include <functional>
#include <map>
#include <any>

#include <aris/plan/scurve.hpp>
#include <aris/plan/path.hpp>

/// \brief 轨迹规划命名空间
/// \ingroup aris
/// 
///
///
/// 
/// 
namespace aris::plan{
	class ARIS_API TrajectoryGenerator {
	public:
		// 配置末端类型 //
		auto eeTypes()const-> const std::vector<aris::dynamic::EEType>&;
		auto setEeTypes(const std::vector<aris::dynamic::EEType>& ee_types)->void;

		// 真实世界的时间间隔 //
		auto dt()const->double;
		auto setDt(double dt)->void;

		// 真实世界的时间间隔 //
		auto maxReplanNum()const->int;
		auto setMaxReplanNum(int max_replan_num = 10) -> void;

		// 规划器内的时间间隔 //
		auto currentDs()const->double;
		auto setCurrentDs(double ds)->void;// 不要随便设置，会导致不连续

		auto targetDs()const->double;
		auto setTargetDs(double ds)->void;

		// 设置时间流逝的加速度
		auto currentDds()const->double;
		auto setCurrentDds(double dds)->void;// 不要随便设置，会导致不连续

		auto maxDds()const->double;
		auto setMaxDds(double max_dds)->void;

		// 设置时间流逝的加加速度
		auto maxDdds()const->double;
		auto setMaxDdds(double max_ddds)->void;

		// 剩余的时间长度
		auto leftNodeS()const->double;
		auto leftTotalS()const->double;

		// 获取末端数据，并移动dt //
		auto getEePosAndMoveDt(double *ee_pos = nullptr, double *ee_vel = nullptr, double *ee_acc = nullptr)->std::int64_t;

		// 插入新的数据，并重规划 //
		auto insertInitPos(std::int64_t id, const double* ee_pos)->void;

		// 插入新的数据，并重规划 //
		auto insertLinePos(std::int64_t id, const double* ee_pos, const double* vel, const double* acc, const double* jerk, const double* zone)->void;

		// 插入新的数据，并重规划 //
		auto insertCirclePos(std::int64_t id, const double* ee_pos, const double* mid_pos, const double* vel, const double* acc, const double* jerk, const double* zone)->void;

		// 删除已经不用的数据 //
		auto clearUsedPos()->void;

		// 删除全部数据 //
		auto clearAllPos()->void;

		// 当前还剩余的指令数 //
		auto unusedPosNum()->int;

		// 返回当前所有的节点 id //
		auto unusedNodeIds()const->std::vector<std::int64_t>;

		~TrajectoryGenerator();
		TrajectoryGenerator();
		ARIS_DELETE_BIG_FOUR(TrajectoryGenerator);

	private:
		struct Imp;
		std::unique_ptr<Imp> imp_;
	};
}

#endif