#ifndef ARIS_PLAN_INPUT_SMOOTHER_H_
#define ARIS_PLAN_INPUT_SMOOTHER_H_

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
	
	class ARIS_API InputSmoother {
	public:
		using InverseKinematicMethod = std::function<std::int64_t(aris::dynamic::ModelBase* model, const double* output_pos, double *input_pos)>;

		using InputGenerator = std::function<std::int64_t(double* input)>;

		// 需要设置模型、TG、电机的最大速度与最大加速度
		auto setModel(aris::dynamic::ModelBase& model) -> void;
		
		auto setInputGenerator(InputGenerator input_generator) -> void;
		auto setPosLimits(const double* max_poss, const double* min_poss = nullptr) -> void;
		auto setVelLimits(const double* max_vels, const double* min_vels = nullptr) -> void;
		auto setAccLimits(const double* max_accs, const double* min_accs = nullptr) -> void;
		auto setJerkLimits(const double* max_jerks, const double* min_jerks = nullptr) -> void;
		auto init(const double* init_input_pos) -> void;

		auto getNextInput(double* p) -> int;


		~InputSmoother();
		InputSmoother();
		ARIS_DELETE_BIG_FOUR(InputSmoother);

	private:
		struct Imp;
		std::unique_ptr<Imp> imp_;
	};

	
	
	
	
}

#endif