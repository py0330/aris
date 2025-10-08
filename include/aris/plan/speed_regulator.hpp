#ifndef ARIS_PLAN_SPEED_REGULATOR_H_
#define ARIS_PLAN_SPEED_REGULATOR_H_

#include <list>
#include <cmath>
#include <iostream>
#include <functional>
#include <map>
#include <any>

#include <aris/core/object.hpp>
#include <aris/core/expression_calculator.hpp>
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
	
	class ARIS_API SpeedRegulator {
	public:
		using InputGenerator = std::function<std::int64_t(double* input)>;

		auto setInputGenerator(InputGenerator input_generator) -> void;
		auto setInputSize(int input_size) -> void;
		auto inputSize() -> int;
		auto setDt(double dt) -> void;
		auto dt() -> double;
		auto setMaxPos(aris::core::Matrix pos) -> void;
		auto maxPos() -> aris::core::Matrix;
		auto setMaxVel(aris::core::Matrix vel) -> void;
		auto maxVel() -> aris::core::Matrix;
		auto setMaxAcc(aris::core::Matrix acc) -> void;
		auto maxAcc() -> aris::core::Matrix;
		auto setMinPos(aris::core::Matrix pos) -> void;
		auto minPos() -> aris::core::Matrix;
		auto setMinVel(aris::core::Matrix vel) -> void;
		auto minVel() -> aris::core::Matrix;
		auto setMinAcc(aris::core::Matrix acc) -> void;
		auto minAcc() -> aris::core::Matrix;

		auto allocateMemory() -> void;
		auto init(double init_target_ds) -> void;
		auto setTargetSpeedRatio(double du) -> void; // 0 <= ds <= 1
		auto targetSpeedRatio() -> double;
		auto actualSpeedRatio() -> double;
		auto getNextInput(double* p) -> std::int64_t;

		~SpeedRegulator();
		SpeedRegulator();
		ARIS_DELETE_BIG_FOUR(SpeedRegulator);

	private:
		struct Imp;
		std::unique_ptr<Imp> imp_;
	};

}

#endif