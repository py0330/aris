#include <iostream>
#include <aris/core/core.hpp>
#include <aris/plan/plan.hpp>
#include <aris/robot/rokae.hpp>

#include <random>
using namespace aris::plan;

// 
auto test_tw()->void {
	
	// make multimodel
	{
		aris::dynamic::MultiModel multi_model;
		auto& model_0 = multi_model.subModels().add<aris::dynamic::Model>();
		auto& model_1 = multi_model.subModels().add<aris::dynamic::Model>();
		model_0.setName("model_0");
		model_1.setName("model_1");


		auto& part00 = model_0.partPool().add<aris::dynamic::Part>("part_0_0");
		auto& part01 = model_0.partPool().add<aris::dynamic::Part>("part_0_1");
		auto& part02 = model_0.partPool().add<aris::dynamic::Part>("part_0_2");
		auto& part03 = model_0.partPool().add<aris::dynamic::Part>("part_0_3");
		auto& part04 = model_0.partPool().add<aris::dynamic::Part>("part_0_4");

		auto& mak_i_0g0 = model_0.ground().addMarker("mak_i_0g0");
		auto& mak_i_0g1 = model_0.ground().addMarker("mak_i_0g1");
		auto& mak_i_0g2 = model_0.ground().addMarker("mak_i_0g2");

		auto &mak_i_000 = part00.addMarker("mak_i_000");
		auto &mak_i_001 = part00.addMarker("mak_i_001");
		auto &mak_i_002 = part00.addMarker("mak_i_002");

		auto& mak_i_010 = part00.addMarker("mak_i_010");
		auto& mak_i_011 = part00.addMarker("mak_i_011");
		auto& mak_i_012 = part00.addMarker("mak_i_012");

		auto& mak_i_020 = part00.addMarker("mak_i_020");
		auto& mak_i_021 = part00.addMarker("mak_i_021");
		auto& mak_i_022 = part00.addMarker("mak_i_022");

		auto& mak_i_030 = part00.addMarker("mak_i_030");
		auto& mak_i_031 = part00.addMarker("mak_i_031");
		auto& mak_i_032 = part00.addMarker("mak_i_032");

		auto& mak_i_040 = part00.addMarker("mak_i_040");
		auto& mak_i_041 = part00.addMarker("mak_i_041");
		auto& mak_i_042 = part00.addMarker("mak_i_042");

		auto& ee0 = model_0.generalMotionPool().add<aris::dynamic::GeneralMotion>("ee0", &mak_i_000, &mak_i_0g0);
		auto& ee1 = model_0.generalMotionPool().add<aris::dynamic::GeneralMotion>("ee1", &mak_i_010, &mak_i_0g0);
		auto& ee2 = model_0.generalMotionPool().add<aris::dynamic::GeneralMotion>("ee2", &mak_i_030, &mak_i_040);
		auto& ee3 = model_0.generalMotionPool().add<aris::dynamic::GeneralMotion>("ee3", &mak_i_020, &mak_i_010);
		auto& ee4 = model_0.generalMotionPool().add<aris::dynamic::GeneralMotion>("ee4", &mak_i_0g0, &mak_i_040);

		ee0.setPoseType(aris::dynamic::GeneralMotion::PoseType::EULER321);
		ee1.setPoseType(aris::dynamic::GeneralMotion::PoseType::EULER321);
		ee2.setPoseType(aris::dynamic::GeneralMotion::PoseType::EULER321);
		ee3.setPoseType(aris::dynamic::GeneralMotion::PoseType::EULER321);
		ee4.setPoseType(aris::dynamic::GeneralMotion::PoseType::EULER321);



		multi_model.init();
		// tool wobj selector
		aris::plan::ToolWobjSelector tw_selector;
		tw_selector.setModel(multi_model);
		// test computeEePos
		{
			ToolWobjSelector::MarkerVec tools{ nullptr, nullptr,nullptr, nullptr, nullptr };
			ToolWobjSelector::MarkerVec wobjs{ nullptr, nullptr,nullptr, nullptr, nullptr };
			double twpos[30]{ 0 };
			double eepos[30]{ 0 };
			tw_selector.computeEePos(tools, wobjs, twpos, eepos);
			std::cout << "eepos: ";
			for (auto i = 0; i < 12; ++i) {
				std::cout << eepos[i] << ", ";
			}
			std::cout << std::endl;
		}
		// test computeTwPos
		{
			ToolWobjSelector::MarkerVec tools(2, nullptr);
			ToolWobjSelector::MarkerVec wobjs(2, nullptr);
			double eepos[12]{ 0 };
			double twpos[2]{ 0.0, 0.0 };
			tw_selector.computeTwPos(tools, wobjs, eepos, twpos);
			std::cout << "twpos: ";
			for (auto i = 0; i < 2; ++i) {
				std::cout << twpos[i] << ", ";
			}
			std::cout << std::endl;
		}
	}

	
}


void test_multimodel_planner(){
	std::cout << std::endl << "-----------------test processor---------------------" << std::endl;

	test_tw();
	//test_input_smoother_2();

	std::cout << "-----------------test processor finished------------" << std::endl << std::endl;
}

