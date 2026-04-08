#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

#include <aris/plan/plan.hpp>
#include <aris/dynamic/mechanism_puma.hpp>

namespace {

auto create_single_puma_model() -> aris::dynamic::MultiModel {
	aris::dynamic::PumaParam puma_param;
	puma_param.d1 = 0.3;
	puma_param.a1 = 0.1;
	puma_param.a2 = 0.4;
	puma_param.a3 = 0.05;
	puma_param.d3 = 0.0;
	puma_param.d4 = 0.35;
	puma_param.install_method = 0;

	auto puma = aris::dynamic::createModelPuma(puma_param);

	aris::dynamic::MultiModel multi_model;
	multi_model.subModels().add(puma.release());
	multi_model.init();
	return multi_model;
}

auto expect_motion_finished(
	aris::plan::MultimodelPlanner &planner,
	aris::dynamic::MultiModel &model,
	std::int64_t expected_node,
	const char *scenario) -> std::vector<double> {
	std::vector<double> input(model.inputPosSize(), 0.0);
	std::vector<double> prev_input(model.inputPosSize(), 0.0);
	std::vector<double> prev_prev_input(model.inputPosSize(), 0.0);
	bool seen_node = false;
	bool finished = false;

	for (int i = 0; i < 50000; ++i) {
		auto ret = planner.getNextInput(input.data());
		EXPECT_GE(ret, 0) << "Planner returned negative ret in " << scenario;
		if (ret < 0) break;

		if (ret == expected_node) seen_node = true;
		if (ret == 0) {
			finished = true;
			break;
		}

		prev_prev_input = prev_input;
		prev_input = input;
	}

	EXPECT_TRUE(seen_node) << "Inserted node id was not observed in " << scenario;
	EXPECT_TRUE(finished) << "Planner did not finish in expected iterations in " << scenario;
	EXPECT_GE(planner.tgRet(), 0) << "tgRet should not be negative in " << scenario;
	EXPECT_GE(planner.ikRet(), 0) << "ikRet should not be negative in " << scenario;

	return input;
}

} // namespace

class MultimodelPlannerTest : public ::testing::Test {
protected:
	void SetUp() override {
		multi_model = create_single_puma_model();
		multi_model.tools().clear();
		multi_model.wobjs().clear();
		multi_model.tools().push_back(multi_model.findMarker("PumaModel.EE.tool0"));
		multi_model.tools().push_back(multi_model.findMarker("PumaModel.EE.tool1"));
		multi_model.wobjs().push_back(multi_model.findMarker("PumaModel.ground.wobj0"));
		multi_model.wobjs().push_back(multi_model.findMarker("PumaModel.ground.wobj1"));

		planner.setModel(multi_model);
		planner.setDt(0.001);
		planner.setSubModelId({ 0 });

		auto input_size = multi_model.inputPosSize();
		std::vector<double> max_vel(input_size, 3.14);
		std::vector<double> min_vel(input_size, -3.14);
		std::vector<double> max_acc(input_size, 31.4);
		std::vector<double> min_acc(input_size, -31.4);

		planner.setMaxVel(aris::core::Matrix(input_size, 1, max_vel.data()));
		planner.setMinVel(aris::core::Matrix(input_size, 1, min_vel.data()));
		planner.setMaxAcc(aris::core::Matrix(input_size, 1, max_acc.data()));
		planner.setMinAcc(aris::core::Matrix(input_size, 1, min_acc.data()));

		planner.allocateMemory();
		planner.init();
	}

	aris::dynamic::MultiModel multi_model;
	aris::plan::MultimodelPlanner planner;
};

TEST_F(MultimodelPlannerTest, ToolWobjSelectorRoundTripTwToEeToTw) {
	auto *tool = multi_model.findMarker("PumaModel.EE.tool0");
	auto *wobj = multi_model.findMarker("PumaModel.ground.wobj0");
	ASSERT_NE(tool, nullptr);
	ASSERT_NE(wobj, nullptr);

	aris::plan::ToolWobjSelector selector;
	selector.setModel(multi_model);
	selector.setSubModelId({ 0 });

	aris::dynamic::Marker *tools[1]{ tool };
	aris::dynamic::Marker *wobjs[1]{ wobj };
	ASSERT_EQ(selector.selectTw(tools, wobjs), 0);

	double tw_pos[6]{ 0.45, 0.1, 0.75, aris::PI / 4.0, aris::PI / 2.0, aris::PI / 4.0 };
	double ee_pos[6]{};
	double tw_roundtrip[6]{};

	selector.setTwPos(tw_pos);
	selector.getEePos(ee_pos);
	selector.setEePos(ee_pos);
	selector.getTwPos(tw_roundtrip);

	for (int i = 0; i < 3; ++i) {
		EXPECT_NEAR(tw_roundtrip[i], tw_pos[i], 1e-9) << "Position mismatch at index " << i;
	}

	double pm_expected[16]{};
	double pm_roundtrip[16]{};
	aris::dynamic::s_pe2pm(tw_pos, pm_expected, "321");
	aris::dynamic::s_pe2pm(tw_roundtrip, pm_roundtrip, "321");
	for (int i = 0; i < 16; ++i) {
		EXPECT_NEAR(pm_roundtrip[i], pm_expected[i], 1e-10)
			<< "Pose matrix mismatch at index " << i;
	}
}

TEST_F(MultimodelPlannerTest, InsertLineMotionFinishes) {
	auto *tool = multi_model.findMarker("PumaModel.EE.tool0");
	auto *wobj = multi_model.findMarker("PumaModel.ground.wobj0");
	ASSERT_NE(tool, nullptr);
	ASSERT_NE(wobj, nullptr);

	std::vector<std::pair<std::string, std::string>> tw{
		{ "PumaModel.EE.tool0", "PumaModel.ground.wobj0" }
	};

	double target_ee[6]{ 0.45, 0.1, 0.75, aris::PI / 4.0, aris::PI / 2.0, aris::PI / 4.0 };
	double vel[2]{ 0.2, 0.2 };
	double acc[2]{ 1.0, 1.0 };
	double jerk[2]{ 10.0, 10.0 };
	double zone[2]{ 0.0, 0.0 };

	auto node = planner.insertLinePos(tw, target_ee, vel, acc, jerk, zone);
	EXPECT_GT(node, 0);

	planner.updateInsertPos();
	auto final_input = expect_motion_finished(planner, multi_model, node, "line-motion");
	EXPECT_EQ(final_input.size(), static_cast<std::size_t>(multi_model.inputPosSize()));
}

TEST_F(MultimodelPlannerTest, InsertCircleMotionFinishes) {
	auto *tool = multi_model.findMarker("PumaModel.EE.tool0");
	auto *wobj = multi_model.findMarker("PumaModel.ground.wobj0");
	ASSERT_NE(tool, nullptr);
	ASSERT_NE(wobj, nullptr);

	std::vector<std::pair<std::string, std::string>> tw{
		{ "PumaModel.EE.tool0", "PumaModel.ground.wobj0" }
	};

	double mid_ee[6]{ 0.45, 0.0, 0.70, aris::PI / 4.0, aris::PI / 2.0, aris::PI / 4.0 };
	double target_ee[6]{ 0.45, -0.1, 0.75, aris::PI / 4.0, aris::PI / 2.0, aris::PI / 4.0 };
	double vel[2]{ 0.2, 0.2 };
	double acc[2]{ 1.0, 1.0 };
	double jerk[2]{ 10.0, 10.0 };
	double zone[2]{ 0.0, 0.0 };

	auto node = planner.insertCirclePos(tw, target_ee, mid_ee, vel, acc, jerk, zone);
	EXPECT_GT(node, 0);

	planner.updateInsertPos();
	auto final_input = expect_motion_finished(planner, multi_model, node, "circle-motion");
	EXPECT_EQ(final_input.size(), static_cast<std::size_t>(multi_model.inputPosSize()));
}
