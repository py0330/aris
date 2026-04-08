#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <vector>

#include <aris/plan/plan.hpp>

namespace {

struct JointLimitConfig {
	double min_vel{-3.14};
	double max_vel{3.14};
	double min_acc{-31.4};
	double max_acc{31.4};
	double dt{1e-3};
};

auto check_dynamic_limits(
	const std::vector<double> &curr,
	const std::vector<double> *prev,
	const std::vector<double> *prev_prev,
	const JointLimitConfig &cfg,
	int step) -> void {
	if (prev) {
		double vel = (curr[0] - (*prev)[0]) / cfg.dt;
		EXPECT_GE(vel, cfg.min_vel - 0.05) << "velocity below min at step " << step;
		EXPECT_LE(vel, cfg.max_vel + 0.05) << "velocity above max at step " << step;
	}

	if (prev && prev_prev) {
		double acc = (curr[0] - 2.0 * (*prev)[0] + (*prev_prev)[0]) / (cfg.dt * cfg.dt);
		EXPECT_GE(acc, cfg.min_acc - 0.5) << "acceleration below min at step " << step;
		EXPECT_LE(acc, cfg.max_acc + 0.5) << "acceleration above max at step " << step;
	}
}

} // namespace

TEST(InputSmootherTest, CosInputRespectsVelocityAndAccelerationLimits) {
	aris::plan::InputSmoother smoother;
	constexpr int input_size = 1;
	const JointLimitConfig cfg;

	smoother.setInputSize(input_size);
	smoother.setDt(cfg.dt);

	std::vector<double> max_vels{ cfg.max_vel };
	std::vector<double> min_vels{ cfg.min_vel };
	std::vector<double> max_accs{ cfg.max_acc };
	std::vector<double> min_accs{ cfg.min_acc };

	smoother.setMaxVel(aris::core::Matrix(input_size, 1, max_vels.data()));
	smoother.setMinVel(aris::core::Matrix(input_size, 1, min_vels.data()));
	smoother.setMaxAcc(aris::core::Matrix(input_size, 1, max_accs.data()));
	smoother.setMinAcc(aris::core::Matrix(input_size, 1, min_accs.data()));

	int count = 0;
	smoother.setInputGenerator([&count](double *p) -> std::int64_t {
		p[0] = std::cos(count * 0.001) * 100.0;
		++count;
		return count > 1000 ? 0 : 1;
	});

	smoother.allocateMemory();
	double init_input[1]{ 100.0 };
	smoother.init(init_input);

	std::vector<double> curr(1, 0.0), prev(1, 0.0), prev_prev(1, 0.0);
	bool seen_stop = false;
	for (int i = 0; i < 20000; ++i) {
		auto ret = smoother.getNextInput(curr.data());
		EXPECT_GE(ret, 0) << "smoother returned negative at step " << i;
		if (ret < 0) break;

		check_dynamic_limits(curr, i > 0 ? &prev : nullptr, i > 1 ? &prev_prev : nullptr, cfg, i);

		prev_prev = prev;
		prev = curr;

		if (ret == 0) {
			seen_stop = true;
			break;
		}
	}

	EXPECT_TRUE(seen_stop) << "InputSmoother did not finish in expected iterations";
}

TEST(InputSmootherTest, AsyncAndSpeedRegulatorPipelineRunsWithoutNegativeReturn) {
	constexpr int input_size = 1;
	const JointLimitConfig cfg;

	aris::plan::InputSmoother smoother;
	smoother.setInputSize(input_size);
	smoother.setDt(cfg.dt);

	std::vector<double> max_vels{ cfg.max_vel };
	std::vector<double> min_vels{ cfg.min_vel };
	std::vector<double> max_accs{ cfg.max_acc };
	std::vector<double> min_accs{ cfg.min_acc };

	smoother.setMaxVel(aris::core::Matrix(input_size, 1, max_vels.data()));
	smoother.setMinVel(aris::core::Matrix(input_size, 1, min_vels.data()));
	smoother.setMaxAcc(aris::core::Matrix(input_size, 1, max_accs.data()));
	smoother.setMinAcc(aris::core::Matrix(input_size, 1, min_accs.data()));

	int count = 0;
	smoother.setInputGenerator([&count](double *p) -> std::int64_t {
		p[0] = (count == 0) ? -1.0 : std::sin(count * 0.001) * 10.0;
		++count;
		return (count % 1000 == 0) ? 0 : 1;
	});

	smoother.allocateMemory();
	double init_input[1]{ -1.0 };
	smoother.init(init_input);

	aris::plan::AsyncGenerator async_gen;
	async_gen.setCacheSize(1000);
	async_gen.setDt(cfg.dt);
	async_gen.setInputSize(input_size);
	async_gen.setInputGenerator([&smoother](double *p) -> std::int64_t {
		return smoother.getNextInput(p);
	});
	async_gen.allocateMemory();
	async_gen.init();

	aris::plan::SpeedRegulator regulator;
	regulator.setInputSize(input_size);
	regulator.setDt(cfg.dt);
	regulator.setInputGenerator([&async_gen](double *p) -> std::int64_t {
		return async_gen.getNextInput(p);
	});
	regulator.setMaxVel(aris::core::Matrix(input_size, 1, max_vels.data()));
	regulator.setMinVel(aris::core::Matrix(input_size, 1, min_vels.data()));
	regulator.setMaxAcc(aris::core::Matrix(input_size, 1, max_accs.data()));
	regulator.setMinAcc(aris::core::Matrix(input_size, 1, min_accs.data()));
	regulator.allocateMemory();
	regulator.init(1.0);

	std::vector<double> input(1, 0.0), prev(1, 0.0), prev_prev(1, 0.0);
	int stop_count = 6;
	bool observed_zero = false;

	for (int i = 0; i < 30000; ++i) {
		auto ret = regulator.getNextInput(input.data());
		EXPECT_GE(ret, 0) << "regulator returned negative at step " << i;
		if (ret < 0) break;

		check_dynamic_limits(input, i > 0 ? &prev : nullptr, i > 1 ? &prev_prev : nullptr, cfg, i);

		prev_prev = prev;
		prev = input;

		if (ret == 0) {
			observed_zero = true;
			if (--stop_count <= 0) break;
		}
	}

	EXPECT_TRUE(observed_zero) << "Pipeline did not produce completion signal";
	EXPECT_LE(stop_count, 0) << "Pipeline did not reach required number of stop cycles";
}
