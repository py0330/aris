#ifndef ARIS_PLAN_ASYNC_GENERATOR_H_
#define ARIS_PLAN_ASYNC_GENERATOR_H_

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

	class ARIS_API AsyncGenerator {
	public:
		using InputGenerator = std::function<std::int64_t(double* input)>;

		auto setInputGenerator(InputGenerator input_generator) -> void;
		auto setInputSize(int input_size) -> void;
		auto inputSize() -> int;
		auto setCacheSize(int cache_size) -> void;
		auto cacheSize() -> int;
		auto setDt(double dt) -> void;
		auto dt() -> double;

		auto allocateMemory() -> void;
		auto init() -> void;
		auto getNextInput(double* p) -> std::int64_t;

		~AsyncGenerator();
		AsyncGenerator();
		ARIS_DELETE_BIG_FOUR(AsyncGenerator);

	private:
		struct Imp;
		std::unique_ptr<Imp> imp_;
	};
	
}

#endif