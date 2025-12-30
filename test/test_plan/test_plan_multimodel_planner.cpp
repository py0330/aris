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
		auto& part05 = model_0.partPool().add<aris::dynamic::Part>("part_0_5");
		auto& part06 = model_0.partPool().add<aris::dynamic::Part>("part_0_6");
		auto& part07 = model_0.partPool().add<aris::dynamic::Part>("part_0_7");
		auto& part08 = model_0.partPool().add<aris::dynamic::Part>("part_0_8");

		auto& mak_i_0g0 = model_0.ground().addMarker("mak_i_0g0");
		auto& mak_i_0g1 = model_0.ground().addMarker("mak_i_0g1", std::array<double, 16>{1, 0, 0, 100, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1}.data());
		auto& mak_i_0g2 = model_0.ground().addMarker("mak_i_0g2");

		auto& mak_i_000 = part00.addMarker("mak_i_000");
		auto &mak_i_001 = part00.addMarker("mak_i_001", std::array<double, 16>{1, 0, 0, 200, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1}.data());
		auto &mak_i_002 = part00.addMarker("mak_i_002");

		auto& mak_i_010 = part01.addMarker("mak_i_010");
		auto& mak_i_011 = part01.addMarker("mak_i_011", std::array<double, 16>{1, 0, 0, 200, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1}.data());
		auto& mak_i_012 = part01.addMarker("mak_i_012");

		auto& mak_i_020 = part02.addMarker("mak_i_020");
		auto& mak_i_021 = part02.addMarker("mak_i_021", std::array<double, 16>{1, 0, 0, 200, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1}.data());
		auto& mak_i_022 = part02.addMarker("mak_i_022");

		auto& mak_i_030 = part03.addMarker("mak_i_030");
		auto& mak_i_031 = part03.addMarker("mak_i_031", std::array<double, 16>{1, 0, 0, 200, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1}.data());
		auto& mak_i_032 = part03.addMarker("mak_i_032");

		auto& mak_i_040 = part04.addMarker("mak_i_040");
		auto& mak_i_041 = part04.addMarker("mak_i_041", std::array<double, 16>{1, 0, 0, 200, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1}.data());
		auto& mak_i_042 = part04.addMarker("mak_i_042");

		auto& mak_i_050 = part05.addMarker("mak_i_050");
		auto& mak_i_051 = part05.addMarker("mak_i_051", std::array<double, 16>{1, 0, 0, 200, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1}.data());
		auto& mak_i_052 = part05.addMarker("mak_i_052");

		auto& mak_i_060 = part06.addMarker("mak_i_060");
		auto& mak_i_061 = part06.addMarker("mak_i_061", std::array<double, 16>{1, 0, 0, 200, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1}.data());
		auto& mak_i_062 = part06.addMarker("mak_i_062");

		auto& mak_i_070 = part07.addMarker("mak_i_070");
		auto& mak_i_071 = part07.addMarker("mak_i_071", std::array<double, 16>{1, 0, 0, 200, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1}.data());
		auto& mak_i_072 = part07.addMarker("mak_i_072");

		auto& mak_i_080 = part08.addMarker("mak_i_080");
		auto& mak_i_081 = part08.addMarker("mak_i_081", std::array<double, 16>{1, 0, 0, 200, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1}.data());
		auto& mak_i_082 = part08.addMarker("mak_i_082");

		auto& ee0 = model_0.generalMotionPool().add<aris::dynamic::GeneralMotion>("ee0", &mak_i_000, &mak_i_0g0);
		auto& ee1 = model_0.generalMotionPool().add<aris::dynamic::GeneralMotion>("ee1", &mak_i_010, &mak_i_0g0);
		auto& ee2 = model_0.generalMotionPool().add<aris::dynamic::GeneralMotion>("ee2", &mak_i_020, &mak_i_030);
		auto& ee3 = model_0.generalMotionPool().add<aris::dynamic::GeneralMotion>("ee3", &mak_i_020, &mak_i_010);
		auto& ee4 = model_0.generalMotionPool().add<aris::dynamic::GeneralMotion>("ee4", &mak_i_0g0, &mak_i_040);
		auto& ee5 = model_0.generalMotionPool().add<aris::dynamic::GeneralMotion>("ee5", &mak_i_050, &mak_i_060);
		auto& ee6 = model_0.generalMotionPool().add<aris::dynamic::GeneralMotion>("ee6", &mak_i_060, &mak_i_070);
		auto& ee7 = model_0.generalMotionPool().add<aris::dynamic::GeneralMotion>("ee7", &mak_i_080, &mak_i_0g0);

		ee0.setPoseType(aris::dynamic::GeneralMotion::PoseType::EULER321);
		ee1.setPoseType(aris::dynamic::GeneralMotion::PoseType::EULER321);
		ee2.setPoseType(aris::dynamic::GeneralMotion::PoseType::EULER321);
		ee3.setPoseType(aris::dynamic::GeneralMotion::PoseType::EULER321);
		ee4.setPoseType(aris::dynamic::GeneralMotion::PoseType::EULER321);
		ee5.setPoseType(aris::dynamic::GeneralMotion::PoseType::EULER321);
		ee6.setPoseType(aris::dynamic::GeneralMotion::PoseType::EULER321);
		ee7.setPoseType(aris::dynamic::GeneralMotion::PoseType::EULER321);



		multi_model.init();
		// tool wobj selector
		aris::plan::ToolWobjSelector tw_selector;
		tw_selector.setModel(multi_model);
		// test computeEePos
		{
			/*
			ToolWobjSelector::MarkerVec tools(8, nullptr);
			ToolWobjSelector::MarkerVec wobjs(8, nullptr);
			double twpos[48]{ 
				1,0,0,0,0,0,
				2,0,0,0,0,0,
				3,0,0,0,0,0,
				4,0,0,0,0,0,
				5,0,0,0,0,0,
				6,0,0,0,0,0,
				7,0,0,0,0,0,
				8,0,0,0,0,0,
			};
			double eepos[48]{ 0 };
			tw_selector.computeEePos(tools, wobjs, twpos, eepos);
			std::cout << "eepos: " << std::endl;
			aris::dynamic::dsp(8, 6, eepos);
			*/
		}
		// test computeTwPos
		{
			ToolWobjSelector::MarkerVec tools{ &mak_i_001, &mak_i_011, &mak_i_021, &mak_i_031, &mak_i_041, &mak_i_051, &mak_i_061, &mak_i_081 };
			ToolWobjSelector::MarkerVec wobjs{ &mak_i_0g1, &mak_i_001, &mak_i_011, &mak_i_021, &mak_i_031, &mak_i_061, &mak_i_071, &mak_i_041 };
			double twpos[48]{
				1,0,0,0,0,0,
				2,0,0,0,0,0,
				3,0,0,0,0,0,
				4,0,0,0,0,0,
				5,0,0,0,0,0,
				6,0,0,0,0,0,
				7,0,0,0,0,0,
				8,0,0,0,0,0,
			};
			double eepos[48]{ 0 };
			tw_selector.selectTw(tools.data(), wobjs.data());
			tw_selector.setTwPos(twpos);
			tw_selector.getEePos(eepos);
			
			const double eepos_answer[48]{
				-99,0,0,0,0,0,
				-97,0,0,0,0,0,
				-4,0,0,0,0,0,
				3,0,0,0,0,0,
				85,0,0,0,0,0,
				6,0,0,0,0,0,
				7,0,0,0,0,0,
				-77,0,0,0,0,0,
			};

			std::fill_n(twpos, 48, 0.0);

			tw_selector.setEePos(eepos);
			tw_selector.getTwPos(twpos);

			if (!aris::dynamic::s_is_equal(48, eepos, eepos_answer, 1e-10)) {
				std::cout << "tw selector error" << std::endl;
			}

			aris::dynamic::dsp(8, 6, twpos);
		}
	}

	
}
auto test_multimodel_async_planner_1() -> void {
	// 构造 TG //
	const int PE_SIZE = 6;
	const int EE_NUM = 1;
	const int A_NUM = 0;

	//  INIT TG //
	double init_pe[EE_NUM * 6]{ 0.45, 0, 0.75,   aris::PI, 1.0,   aris::PI };
	double init_vel[EE_NUM * 2 + A_NUM]{ 1,1 };

	//  MAKE PQS ... //
	double pes[PE_SIZE][6 * EE_NUM]{
		{ 0.45, 0.4, 0.75,   aris::PI, 1.0,   aris::PI},
		{ 0.45, 0.0, 0.75,   aris::PI, 1.0,   aris::PI},
		{ 0.45, 0.0, 0.65,   aris::PI, 1.0,   aris::PI},
		{ 0.45, 0.0, 0.65,   aris::PI / 4, aris::PI / 2,   aris::PI / 4},
		{ 0.45, 0.0, 0.75,   aris::PI / 4, aris::PI / 2,   aris::PI / 4},
		{ 0.45, 0.1, 0.75,   aris::PI / 4, aris::PI / 2,   aris::PI / 4},
	};

	//  MAKE VELS ACCS JERKS ZONES ... //
	double vels[PE_SIZE][2 * EE_NUM + A_NUM]{
		{ 10, 10 },
		{ 0.1, 0.1 },
		{ 0.1, 0.1 },
		{ 1000, 1000 },
		{ 1000, 1000 },
		{ 1000, 1000 },
	};
	double accs[PE_SIZE][2 * EE_NUM + A_NUM]{
		{ 10, 10 },
		{ 0.1, 0.1 },
		{ 0.1, 0.1 },
		{ 10000, 10000 },
		{ 10000, 10000 },
		{ 10000, 10000 },
	};
	double jerks[PE_SIZE][2 * EE_NUM + A_NUM]{
		{ 100, 100 },
		{ 0.1, 0.1 },
		{ 0.1, 0.1 },
		{ 10000000, 1000000 },
		{ 10000000, 1000000 },
		{ 10000000, 1000000 },
	};
	double zones[PE_SIZE][2 * EE_NUM + A_NUM]{
		{ 0.2, 0.2 },
		{ 0.0, 0.0 },
		{ 0.1, 0.1 },
		{ 0.0, 0.0 },
		{ 0.0, 0.0 },
		{ 0.0, 0.0 },
	};

	double out_pe[7 * EE_NUM + A_NUM];

	// 构造模型 //
	aris::dynamic::PumaParam puma_param;
	puma_param.d1 = 0.3;
	puma_param.a1 = 0.1;
	puma_param.a2 = 0.4;
	puma_param.a3 = 0.05;
	puma_param.d3 = 0.0;
	puma_param.d4 = 0.35;
	puma_param.install_method = 0;
	auto puma = aris::dynamic::createModelPuma(puma_param);

	puma->setOutputPos(init_pe);
	puma->inverseKinematics();

	dynamic_cast<aris::dynamic::GeneralMotion&>(puma->generalMotionPool()[0]).setPoseType(aris::dynamic::GeneralMotion::PoseType::EULER321);
	double input_init[6]{ -1,0,0,0,0,0 };
	puma->getInputPos(input_init);

	aris::dynamic::MultiModel multi_model;
	multi_model.subModels().add(puma.release());
	multi_model.init();
	
	// 构造规划器 //
	MultimodelPlanner mmp;
	mmp.setModel(multi_model);

	// 最大速度、加速度 //
	std::vector<double> max_vels{ 3.14, 3.14, 3.14, 3.14, 3.14, 3.14 };
	std::vector<double> min_vels{ -3.14, -3.14, -3.14, -3.14, -3.14, -3.14 };
	std::vector<double> max_accs{ 31.4, 31.4, 31.4, 31.4, 31.4, 31.4 };
	std::vector<double> min_accs{ -31.4, -31.4, -31.4, -31.4, -31.4, -31.4 };

	mmp.setMaxVel(aris::core::Matrix(multi_model.inputPosSize(), 1, max_vels.data()));
	mmp.setMinVel(aris::core::Matrix(multi_model.inputPosSize(), 1, min_vels.data()));
	mmp.setMaxAcc(aris::core::Matrix(multi_model.inputPosSize(), 1, max_accs.data()));
	mmp.setMinAcc(aris::core::Matrix(multi_model.inputPosSize(), 1, min_accs.data()));

	mmp.setDt(1e-3);
	mmp.allocateMemory();

	mmp.init();
	
	for (int i = 0; i < PE_SIZE; ++i) {
		std::vector<std::pair<std::string, std::string>> tw;
		tw.push_back(std::make_pair<std::string, std::string>("tool0", "wobj0"));
		mmp.insertLinePos(tw, pes[i % PE_SIZE], vels[i % PE_SIZE], accs[i % PE_SIZE], jerks[i % PE_SIZE], zones[i % PE_SIZE]);
	}
	{
		std::vector<std::pair<std::string, std::string>> tw;
		tw.push_back(std::make_pair<std::string, std::string>("tool1", "wobj1"));
		mmp.insertCirclePos(tw, pes[PE_SIZE - 3], pes[PE_SIZE - 2], vels[PE_SIZE-1], accs[PE_SIZE - 1], jerks[PE_SIZE - 1], zones[PE_SIZE - 1]);
	}
	mmp.updateInsertPos();

	// 打印数据 //
	std::vector<double> vec, v_vec, a_vec;
	int m = 0;
	double out_vel[16]{}, out_acc[16]{}, ee_pos[16], input_pos[6];
	double s = 0;
	
	while (auto ret = mmp.getNextInput(input_pos)) {
		static auto last_ret = -1;
		if (ret != last_ret) {
			std::cout << "cmd: " << ret <<"  count: " <<m << std::endl;
			last_ret = ret;
		}

		m++;

		vec.resize(m * (multi_model.inputPosSize() * EE_NUM + A_NUM), 0.0);
		std::copy_n(input_pos, multi_model.inputPosSize(), vec.data() + (multi_model.inputPosSize() * EE_NUM + A_NUM) * (m - 1));
	}

	aris::dynamic::dlmwrite(m, (multi_model.inputPosSize() * EE_NUM + A_NUM), vec.data(), "/Mac/Home/Documents/MATLAB/test/data.txt");
	/**/
}

void test_multimodel_planner(){
	std::cout << std::endl << "-----------------test processor---------------------" << std::endl;

	//test_tw();
	test_multimodel_async_planner_1();

	std::cout << "-----------------test processor finished------------" << std::endl << std::endl;
}

