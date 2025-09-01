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
		using InputGenerator = std::function<std::int64_t(double* input)>;

		auto setInputGenerator(InputGenerator input_generator) -> void;
		auto setPosLimits(const double* max_poss, const double* min_poss = nullptr) -> void;
		auto setVelLimits(const double* max_vels, const double* min_vels = nullptr) -> void;
		auto setAccLimits(const double* max_accs, const double* min_accs = nullptr) -> void;

		auto allocateMemory(int input_size) -> void;
		auto setBeginInputPos(const double* init_input_pos) -> void;

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