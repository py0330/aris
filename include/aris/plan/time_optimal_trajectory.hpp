#ifndef ARIS_PLAN_TIME_OPTIMAL_TRAJECTORY_H_
#define ARIS_PLAN_TIME_OPTIMAL_TRAJECTORY_H_

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

	auto ARIS_API s_cpt_d3u_lr(int p_size, const double* p0, const double* p1, const double* p2, const double* p3,
		const double* p_min, const double* p_max, const double* dp_min, const double* dp_max,
		const double* d2p_min, const double* d2p_max, const double* d3p_min, const double* d3p_max,
		double s_diff, double u0, double u1, double u2, double& d3u_ds3_3_L, double& d3u_ds3_3_R, double zero_check = 1e-10) -> void;


	class ARIS_API TimeOptimalTrajectoryGenerator {
	public:
		// 配置末端类型 //
		auto eeTypes()const-> const std::vector<aris::dynamic::EEType>&;
		auto setEeTypes(const std::vector<aris::dynamic::EEType>& ee_types)->void;

		// 真实世界的时间间隔 //
		auto dt()const->double;
		auto setDt(double dt)->void;

		// 最大的重规划点数 //
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
		// return
		//        0: 全部运行结束
		//  node_id: 当前节点的 id 号，对应插入时的 id
		auto getEePosAndMoveDt(double *ee_pos = nullptr, double *ee_vel = nullptr, double *ee_acc = nullptr)->std::int64_t;

		// 根据 s 获得数据
		// s 必须大于当前执行到的节点的起始 s
		auto getEePosByS(double s, double* ee_pos = nullptr, double* ee_vel = nullptr, double* ee_acc = nullptr, std::int64_t = 0)-> std::int64_t;
		auto clearNodesBefore(std::int64_t id)->int;




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

		~TimeOptimalTrajectoryGenerator();
		TimeOptimalTrajectoryGenerator();
		ARIS_DELETE_BIG_FOUR(TimeOptimalTrajectoryGenerator);

	private:
		struct Imp;
		std::unique_ptr<Imp> imp_;
	};


	class ARIS_API LookAheadProcessor {
	public:
		using InverseKinematicMethod = std::function<std::int64_t(aris::dynamic::ModelBase& model, const double* ee_pos)>;

		// 需要设置模型、TG、电机的最大速度与最大加速度
		auto setModel(aris::dynamic::ModelBase& model) -> void;
		auto setTrajectoryGenerator(TimeOptimalTrajectoryGenerator& tg) -> void;
		auto setMaxPoss(const double* max_poss, const double* min_poss = nullptr) -> void;
		auto setMaxVels(const double* max_vels, const double* min_vels = nullptr) -> void;
		auto setMaxAccs(const double* max_accs, const double* min_accs = nullptr) -> void;
		auto setMaxJerks(const double* max_jerks, const double* min_jerks = nullptr) -> void;
		auto init() -> void;

		auto lookAheadOneStep() -> int;
		auto lookAhead(double s_begin) -> int;


		// 设置速度百分比，类似 TG 中 setTargetDs
		// 用以下参数后，不能再设置tg中的对应参数
		auto setTargetDs(double ds) -> void;
		auto setDs(double ds) -> void;
		auto currentDs() -> double;

		// 每个实时周期调用这个函数，确保不超速
		// return
		//        0 : 全部运行结束
		//  node_id : 当前节点的 id 号，对应插入时的 id
		//     负数 : 反解计算错误 id，在默认的反解计算中，若反解无解，则返回 -1
		auto setModelPosAndMoveDt() -> std::int64_t;

		// 设置方法
		auto setInverseKinematicMethod(InverseKinematicMethod) -> void;

		// 设置最大的速度比和加速度比
		auto setMaxVelRatio(double vel_ratio) -> void;
		auto setMaxAccRatio(double acc_ratio) -> void;

		~LookAheadProcessor();
		LookAheadProcessor();
		ARIS_DELETE_BIG_FOUR(LookAheadProcessor);

	private:
		struct Imp;
		std::unique_ptr<Imp> imp_;
	};
}

#endif