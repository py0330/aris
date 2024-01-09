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
		using InverseKinematicMethod = std::function<std::int64_t(aris::dynamic::ModelBase& model, const double *ee_pos)>;

		// 需要设置模型、TG、电机的最大速度与最大加速度
		auto setModel(aris::dynamic::ModelBase& model)->void;
		auto setTrajectoryGenerator(TrajectoryGenerator& tg)->void;
		auto setMaxVels(const double* max_vels)->void;
		auto setMaxAccs(const double* max_accs)->void;
		auto init()->void;
		
		// 设置速度百分比，类似 TG 中 setTargetDs
		// 用以下参数后，不能再设置tg中的对应参数
		auto setTargetDs(double ds)->void;
		auto setDs(double ds)->void;
		auto currentDs() -> double;

		// 每个实时周期调用这个函数，确保不超速
		// return
		//        0 : 全部运行结束
		//  node_id : 当前节点的 id 号，对应插入时的 id
		//     负数 : 反解计算错误 id，在默认的反解计算中，若反解无解，则返回 -1
		auto setModelPosAndMoveDt()->std::int64_t;

		// 设置方法
		auto setInverseKinematicMethod(InverseKinematicMethod)->void;

		// 设置最大的速度比和加速度比
		auto setMaxVelRatio(double vel_ratio)->void;
		auto setMaxAccRatio(double acc_ratio)->void;

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