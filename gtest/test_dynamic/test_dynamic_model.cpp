#include <gtest/gtest.h>

#include <aris/dynamic/dynamic.hpp>


TEST(DynamicModelTest, MultiModel) {
	
	aris::dynamic::MultiModel multi;

	auto &sub0 = multi.subModels().add<aris::dynamic::Model>();
	auto& sub1 = multi.subModels().add<aris::dynamic::Model>();
	auto& sub2 = multi.subModels().add<aris::dynamic::Model>();
	auto& sub3 = multi.subModels().add<aris::dynamic::Model>();
	auto& sub4 = multi.subModels().add<aris::dynamic::Model>();

	auto &gm01 = sub0.generalMotionPool().add<aris::dynamic::GeneralMotion>();
	gm01.setPosType(aris::dynamic::PosType::PE123);
	auto &gm02 = sub0.generalMotionPool().add<aris::dynamic::GeneralMotion>();
	gm02.setPosType(aris::dynamic::PosType::PQ);
	auto& gm03 = sub0.generalMotionPool().add<aris::dynamic::GeneralMotion>();
	gm03.setPosType(aris::dynamic::PosType::PM);

	auto& gm21 = sub2.generalMotionPool().add<aris::dynamic::GeneralMotion>();
	gm21.setPosType(aris::dynamic::PosType::PE123);
	auto& gm22 = sub2.generalMotionPool().add<aris::dynamic::GeneralMotion>();
	gm22.setPosType(aris::dynamic::PosType::PQ);
	auto& gm23 = sub2.generalMotionPool().add<aris::dynamic::GeneralMotion>();
	gm23.setPosType(aris::dynamic::PosType::PE123);
	auto& gm24 = sub2.generalMotionPool().add<aris::dynamic::GeneralMotion>();
	gm24.setPosType(aris::dynamic::PosType::PE123);
	auto& gm25 = sub2.generalMotionPool().add<aris::dynamic::GeneralMotion>();
	gm25.setPosType(aris::dynamic::PosType::PM);

	auto& gm31 = sub3.generalMotionPool().add<aris::dynamic::GeneralMotion>();
	gm31.setPosType(aris::dynamic::PosType::PQ);

	auto& gm41 = sub4.generalMotionPool().add<aris::dynamic::GeneralMotion>();
	gm41.setPosType(aris::dynamic::PosType::PE123);
	auto& gm42 = sub4.generalMotionPool().add<aris::dynamic::GeneralMotion>();
	gm42.setPosType(aris::dynamic::PosType::PM);

	

	auto& mot01 = sub0.motionPool().add<aris::dynamic::Motion>();
	mot01.setAxis(5);
	auto& mot02 = sub0.motionPool().add<aris::dynamic::Motion>();
	auto& mot03 = sub0.motionPool().add<aris::dynamic::Motion>();

	auto& mot11 = sub1.motionPool().add<aris::dynamic::Motion>();
	auto& mot12 = sub1.motionPool().add<aris::dynamic::Motion>();
	mot12.setAxis(5);
	auto& mot13 = sub1.motionPool().add<aris::dynamic::Motion>();
	auto& mot14 = sub1.motionPool().add<aris::dynamic::Motion>();

	auto& mot31 = sub3.motionPool().add<aris::dynamic::Motion>();
	auto& mot32 = sub3.motionPool().add<aris::dynamic::Motion>();
	mot32.setAxis(5);
	auto& mot33 = sub3.motionPool().add<aris::dynamic::Motion>();
	auto& mot34 = sub3.motionPool().add<aris::dynamic::Motion>();
	auto& mot35 = sub3.motionPool().add<aris::dynamic::Motion>();
	auto& mot36 = sub3.motionPool().add<aris::dynamic::Motion>();
	mot36.setAxis(5);

	auto& mot41 = sub4.motionPool().add<aris::dynamic::Motion>();
	auto& mot42 = sub4.motionPool().add<aris::dynamic::Motion>();

	multi.init();

	// test input output size
	{
		EXPECT_FALSE(multi.outputPosSize() != 99) << "\"MultiModel::outputPosSize\" failed";

		EXPECT_FALSE(multi.outputVelSize() != 66) << "\"MultiModel::outputVelSize\" failed";

		EXPECT_FALSE(multi.outputAccSize() != 66) << "\"MultiModel::outputAccSize\" failed";

		EXPECT_FALSE(multi.outputFceSize() != 66) << "\"MultiModel::outputFceSize\" failed";

		EXPECT_FALSE(multi.inputPosSize() != 15) << "\"MultiModel::inputPosSize\" failed";

		EXPECT_FALSE(multi.inputVelSize() != 15) << "\"MultiModel::inputVelSize\" failed";

		EXPECT_FALSE(multi.inputAccSize() != 15) << "\"MultiModel::inputAccSize\" failed";

		EXPECT_FALSE(multi.inputFceSize() != 15) << "\"MultiModel::inputFceSize\" failed";
	}
	
	// test getEeNumOfSubModels
	/*
	{
		std::vector<aris::Size> result1{ 1,2,3,5,0 };
		EXPECT_FALSE(result1 != multi.getSubEeSize({ 3,4,0,2,1 })) << "\"MultiModel::getSubEeSize\" failed";

		aris::Size result1a[5], input1a[5]{ 3,4,0,2,1 };
		multi.getSubEeSize(5, input1a, result1a);
		EXPECT_FALSE(std::vector<aris::Size>(result1a, result1a+5) != result1) << "\"MultiModel::getSubEeSize\" failed";

		std::vector<aris::Size> result2{ 2,5,1,1 };
		EXPECT_FALSE(result2 != multi.getSubEeSize({ 4,2,3,3 })) << "\"MultiModel::getSubEeSize\" failed";
		
		aris::Size result2a[4], input2a[4]{ 4,2,3,3 };
		multi.getSubEeSize(4, input2a, result2a);
		EXPECT_FALSE(std::vector<aris::Size>(result2a, result2a + 4) != result2) << "\"MultiModel::getSubEeSize\" failed";
	}
	*/

	// test getEeTypes
	{
		//
		// 	ee_types:
		//aris::dynamic::PosType::PE123, aris::dynamic::PosType::PQ, aris::dynamic::PosType::PM,
		//
		//aris::dynamic::PosType::PE123, aris::dynamic::PosType::PQ, aris::dynamic::PosType::PE123, aris::dynamic::PosType::PE123, aris::dynamic::PosType::PM,
		//aris::dynamic::PosType::PQ,
		//aris::dynamic::PosType::PE123, aris::dynamic::PosType::PM


		std::vector<aris::dynamic::PosType> ee_types;
		std::vector<aris::dynamic::PosType> result0{
			aris::dynamic::PosType::PE123, aris::dynamic::PosType::PQ, aris::dynamic::PosType::PM,
			
			aris::dynamic::PosType::PE123, aris::dynamic::PosType::PQ, aris::dynamic::PosType::PE123, aris::dynamic::PosType::PE123, aris::dynamic::PosType::PM,
			aris::dynamic::PosType::PQ,
			aris::dynamic::PosType::PE123, aris::dynamic::PosType::PM
		};

		ee_types = std::vector<aris::dynamic::PosType>(multi.outputPosTypes(), multi.outputPosTypes() + multi.outputSize());
		EXPECT_FALSE(ee_types != result0) << "\"MultiModel::eePosTypes\" failed";

		//ee_types.clear();
		//ee_types.resize(11, aris::dynamic::PosType::PQ);
		//multi.getSubEeSize(ee_types.data());
		//if (ee_types != result0)
		//	std::cout << "\"MultiModel::getSubEeSize\" failed" << std::endl;


		std::vector<aris::dynamic::PosType> result1{
			aris::dynamic::PosType::PQ,
			aris::dynamic::PosType::PE123, aris::dynamic::PosType::PM,
			aris::dynamic::PosType::PE123, aris::dynamic::PosType::PQ, aris::dynamic::PosType::PM,
			aris::dynamic::PosType::PE123, aris::dynamic::PosType::PQ, aris::dynamic::PosType::PE123, aris::dynamic::PosType::PE123, aris::dynamic::PosType::PM,
		};
		aris::Size sub1[5]{ 3,4,0,2,1 };

		ee_types.resize(multi.subOutputSize(5, sub1));
		multi.getSubOutputPosTypes(5, sub1, ee_types.data());
		EXPECT_FALSE(ee_types != result1) << "\"MultiModel::getSubEeTypes\" failed";

		ee_types.clear();
		ee_types.resize(11, aris::dynamic::PosType::PQ);
		multi.getSubOutputPosTypes(5, sub1, ee_types.data());
		EXPECT_FALSE(ee_types != result1) << "\"MultiModel::getSubEeTypes\" failed";

		std::vector<aris::dynamic::PosType> result2{
			aris::dynamic::PosType::PE123, aris::dynamic::PosType::PM,
			aris::dynamic::PosType::PE123, aris::dynamic::PosType::PQ, aris::dynamic::PosType::PE123, aris::dynamic::PosType::PE123, aris::dynamic::PosType::PM,
			aris::dynamic::PosType::PQ, 
			aris::dynamic::PosType::PQ,
		};
		aris::Size sub2[4]{ 4,2,3,3 };

		ee_types.resize(multi.subOutputSize(4, sub2));
		multi.getSubOutputPosTypes(4, sub2, ee_types.data());
		EXPECT_FALSE(ee_types != result2) << "\"MultiModel::getSubEeTypes\" failed";

		ee_types.clear();
		ee_types.resize(9, aris::dynamic::PosType::PQ);
		multi.getSubOutputPosTypes(4, sub2, ee_types.data());
		EXPECT_FALSE(ee_types != result2) << "\"MultiModel::getSubEeTypes\" failed";
	}
		
	// test getEes
	{
		std::vector<aris::dynamic::MotionBase*> ees;
		std::vector<aris::dynamic::MotionBase*> result0{
			&gm01, &gm02,& gm03,

			&gm21,& gm22,& gm23,& gm24,& gm25,
			& gm31,
			& gm41,& gm42,
		};
		/*
		ees = multi.getEes();
		EXPECT_FALSE(ees != result0) << "\"MultiModel::getEes\" failed";

		ees.clear();
		ees.resize(11, nullptr);
		multi.getEes(ees.data());
		EXPECT_FALSE(ees != result0) << "\"MultiModel::getEes\" failed";
		*/

		std::vector<aris::dynamic::MotionBase*> result1{
			&gm31,
			& gm41,& gm42,
			& gm01,& gm02,& gm03,
			& gm21,& gm22,& gm23,& gm24,& gm25, 
		};
		aris::Size sub1[5]{ 3,4,0,2,1 };

		ees.clear();
		ees.resize(11, nullptr);
		multi.getSubOutputMotions(5, sub1, ees.data());
		EXPECT_FALSE(ees != result1) << "\"MultiModel::getSubOutputMotions\" failed";

		std::vector<aris::dynamic::MotionBase*> result2{
			&gm41,& gm42,
			& gm21,& gm22,& gm23,& gm24,& gm25,
			& gm31,
			& gm31,

		};
		aris::Size sub2[4]{ 4,2,3,3 };

		ees.clear();
		ees.resize(9, nullptr);
		multi.getSubOutputMotions(4, sub2, ees.data());
		EXPECT_FALSE(ees != result2) << "\"MultiModel::getSubOutputMotions\" failed";
	}

	// test getMotionNumOfSubModels
	{
		/*
		// 3 4 0 6 2
		std::vector<aris::Size> result1{ 6,2,3,0,4 };
		EXPECT_FALSE(result1 != multi.getMotionNumOfSubModels({ 3,4,0,2,1 })) << "\"MultiModel::getMotionNumOfSubModels\" failed";

		aris::Size result1a[5], input1a[5]{ 3,4,0,2,1 };
		multi.getMotionNumOfSubModels(5, input1a, result1a);
		EXPECT_FALSE(std::vector<aris::Size>(result1a, result1a + 5) != result1) << "\"MultiModel::getMotionNumOfSubModels\" failed";

		std::vector<aris::Size> result2{ 2,0,6,6 };
		EXPECT_FALSE(result2 != multi.getMotionNumOfSubModels({ 4,2,3,3 })) << "\"MultiModel::getMotionNumOfSubModels\" failed";

		aris::Size result2a[4], input2a[4]{ 4,2,3,3 };
		multi.getMotionNumOfSubModels(4, input2a, result2a);
		EXPECT_FALSE(std::vector<aris::Size>(result2a, result2a + 4) != result2) << "\"MultiModel::getMotionNumOfSubModels\" failed";
		*/
	}
	
	// test getMotionTypes
	{
		std::vector<aris::dynamic::PosType> ee_types;
		std::vector<aris::dynamic::PosType> result0{
			aris::dynamic::PosType::A, aris::dynamic::PosType::X, aris::dynamic::PosType::X,
			aris::dynamic::PosType::X, aris::dynamic::PosType::A, aris::dynamic::PosType::X, aris::dynamic::PosType::X,
			
			aris::dynamic::PosType::X, aris::dynamic::PosType::A, aris::dynamic::PosType::X, aris::dynamic::PosType::X,aris::dynamic::PosType::X, aris::dynamic::PosType::A,
			aris::dynamic::PosType::X, aris::dynamic::PosType::X,
		};

		//ee_types = multi.getMotionTypes();
		//if (ee_types != result0)
		//	std::cout << "\"MultiModel::getMotionTypes\" failed" << std::endl;

		//ee_types.clear();
		//ee_types.resize(15, aris::dynamic::PosType::PQ);
		//multi.getMotionTypes(ee_types.data());
		//if (ee_types != result0)
		//	std::cout << "\"MultiModel::getMotionTypes\" failed" << std::endl;


		std::vector<aris::dynamic::PosType> result1{
			aris::dynamic::PosType::X, aris::dynamic::PosType::A, aris::dynamic::PosType::X, aris::dynamic::PosType::X,aris::dynamic::PosType::X, aris::dynamic::PosType::A,
			aris::dynamic::PosType::X, aris::dynamic::PosType::X,
			aris::dynamic::PosType::A, aris::dynamic::PosType::X, aris::dynamic::PosType::X,
			aris::dynamic::PosType::X, aris::dynamic::PosType::A, aris::dynamic::PosType::X, aris::dynamic::PosType::X,
		};
		aris::Size sub1[5]{ 3,4,0,2,1 };

		//ee_types = multi.getMotionTypes({ 3,4,0,2,1 });
		//if (ee_types != result1)
		//	std::cout << "\"MultiModel::getMotionTypes\" failed" << std::endl;

		//ee_types.clear();
		//ee_types.resize(15, aris::dynamic::PosType::PQ);
		//multi.getMotionTypes(5, sub1, ee_types.data());
		//if (ee_types != result1)
		//	std::cout << "\"MultiModel::getMotionTypes\" failed" << std::endl;

		std::vector<aris::dynamic::PosType> result2{
			aris::dynamic::PosType::X, aris::dynamic::PosType::X,
			aris::dynamic::PosType::X, aris::dynamic::PosType::A, aris::dynamic::PosType::X, aris::dynamic::PosType::X,aris::dynamic::PosType::X, aris::dynamic::PosType::A,
			aris::dynamic::PosType::X, aris::dynamic::PosType::A, aris::dynamic::PosType::X, aris::dynamic::PosType::X,aris::dynamic::PosType::X, aris::dynamic::PosType::A,
		};
		aris::Size sub2[4]{ 4,2,3,3 };

		//ee_types = multi.getMotionTypes({ 4,2,3,3 });
		//if (ee_types != result2)
		//	std::cout << "\"MultiModel::getMotionTypes\" failed" << std::endl;

		//ee_types.clear();
		//ee_types.resize(14, aris::dynamic::PosType::PQ);
		//multi.getMotionTypes(4, sub2, ee_types.data());
		//if (ee_types != result2)
		//	std::cout << "\"MultiModel::getMotionTypes\" failed" << std::endl;
	}

	// test getMotions
	{
		std::vector<aris::dynamic::Motion*> mots;
		std::vector<aris::dynamic::Motion*> result0{
			&mot01, &mot02,&mot03,
			&mot11,&mot12,&mot13,&mot14,

			& mot31,& mot32,& mot33,& mot34,& mot35,& mot36,
			&mot41,&mot42,
		};


		std::vector<aris::dynamic::Motion*> result1{
			&mot31,& mot32,& mot33,& mot34,& mot35,& mot36,
			& mot41,& mot42,
			& mot01,& mot02,& mot03,
			& mot11,& mot12,& mot13,& mot14,
		};
		aris::Size sub1[5]{ 3,4,0,2,1 };

		mots.clear();
		mots.resize(15, nullptr);
		multi.getSubInputMotions(5, sub1, mots.data());
		EXPECT_FALSE(mots != result1) << "\"MultiModel::getMotions\" failed";

		std::vector<aris::dynamic::Motion*> result2{
			& mot41,& mot42,
			& mot31,& mot32,& mot33,& mot34,& mot35,& mot36,
			& mot31,& mot32,& mot33,& mot34,& mot35,& mot36,
		};
		aris::Size sub2[4]{ 4,2,3,3 };

		mots.clear();
		mots.resize(14, nullptr);
		multi.getSubInputMotions(4, sub2, mots.data());
		EXPECT_FALSE(mots != result2) << "\"MultiModel::getMotions\" failed";
	}

	// test getMotionIds
	{
		std::vector<aris::Size> mots;
		std::vector<aris::Size> result0{
			0,1,2,
			3,4,5,6,

			7,8,9,10,11,12,
			13,14,
		};

		/*
		mots = multi.getMotionIds();
		EXPECT_FALSE(mots != result0) << "\"MultiModel::getMotionIds\" failed";

		mots.clear();
		mots.resize(15, -1);
		multi.getMotionIds(mots.data());
		EXPECT_FALSE(mots != result0) << "\"MultiModel::getMotionIds\" failed";
			*/

		std::vector<aris::Size> result1{
			7,8,9,10,11,12,
			13,14,
			0,1,2,
			3,4,5,6,
		};
		aris::Size sub1[5]{ 3,4,0,2,1 };

		/*
		mots = multi.getMotionIds({ 3,4,0,2,1 });
		EXPECT_FALSE(mots != result1) << "\"MultiModel::getMotionIds\" failed";

		mots.clear();
		mots.resize(15, -1);
		multi.getMotionIds(5, sub1, mots.data());
		EXPECT_FALSE(mots != result1) << "\"MultiModel::getMotionIds\" failed";
		*/
		std::vector<aris::Size> result2{
			13,14,
			7,8,9,10,11,12,
			7,8,9,10,11,12,
		};
		aris::Size sub2[4]{ 4,2,3,3 };
		/*
		mots = multi.getMotionIds({ 4,2,3,3 });
		EXPECT_FALSE(mots != result2) << "\"MultiModel::getMotionIds\" failed";

		mots.clear();
		mots.resize(14, -1);
		multi.getMotionIds(4, sub2, mots.data());
		EXPECT_FALSE(mots != result2) << "\"MultiModel::getMotionIds\" failed";
			*/
	}

	


}

TEST(DynamicModelTest, MultiModel2) {

	aris::dynamic::MultiModel multi;
	
	// make multi model
	auto& sub0 = multi.subModels().add<aris::dynamic::Model>();
	auto& sub1 = multi.subModels().add<aris::dynamic::Model>();
	auto& sub2 = multi.subModels().add<aris::dynamic::Model>();
	auto& sub3 = multi.subModels().add<aris::dynamic::Model>();
	auto& sub4 = multi.subModels().add<aris::dynamic::Model>();
	auto& sub5 = multi.subModels().add<aris::dynamic::MultiModel>();

	auto& sub50 = sub5.subModels().add<aris::dynamic::Model>();
	auto& sub51 = sub5.subModels().add<aris::dynamic::Model>();
	auto& sub52 = sub5.subModels().add<aris::dynamic::Model>();
	auto& sub53 = sub5.subModels().add<aris::dynamic::Model>();
	auto& sub54 = sub5.subModels().add<aris::dynamic::Model>();

	sub0.setName("sub0");
	sub1.setName("sub1");
	sub2.setName("sub2");
	sub3.setName("sub3");
	sub4.setName("sub4");
	sub5.setName("sub5");

	sub50.setName("sub50");
	sub51.setName("sub51");
	sub52.setName("sub52");
	sub53.setName("sub53");
	sub54.setName("sub54");

	auto& gm01 = sub0.generalMotionPool().add<aris::dynamic::GeneralMotion>();
	gm01.setPosType(aris::dynamic::PosType::PE123);
	auto& gm02 = sub0.generalMotionPool().add<aris::dynamic::GeneralMotion>();
	gm02.setPosType(aris::dynamic::PosType::PQ);
	auto& gm03 = sub0.generalMotionPool().add<aris::dynamic::GeneralMotion>();
	gm03.setPosType(aris::dynamic::PosType::PM);

	auto& gm21 = sub2.generalMotionPool().add<aris::dynamic::GeneralMotion>();
	gm21.setPosType(aris::dynamic::PosType::PE123);
	auto& gm22 = sub2.generalMotionPool().add<aris::dynamic::GeneralMotion>();
	gm22.setPosType(aris::dynamic::PosType::PQ);
	auto& gm23 = sub2.generalMotionPool().add<aris::dynamic::GeneralMotion>();
	gm23.setPosType(aris::dynamic::PosType::PE123);
	auto& gm24 = sub2.generalMotionPool().add<aris::dynamic::GeneralMotion>();
	gm24.setPosType(aris::dynamic::PosType::PE123);
	auto& gm25 = sub2.generalMotionPool().add<aris::dynamic::GeneralMotion>();
	gm25.setPosType(aris::dynamic::PosType::PM);

	auto& gm31 = sub3.generalMotionPool().add<aris::dynamic::GeneralMotion>();
	gm31.setPosType(aris::dynamic::PosType::PQ);

	auto& gm41 = sub4.generalMotionPool().add<aris::dynamic::GeneralMotion>();
	gm41.setPosType(aris::dynamic::PosType::PE123);
	auto& gm42 = sub4.generalMotionPool().add<aris::dynamic::GeneralMotion>();
	gm42.setPosType(aris::dynamic::PosType::PM);

	auto& gm501 = sub50.generalMotionPool().add<aris::dynamic::GeneralMotion>();
	gm501.setPosType(aris::dynamic::PosType::PE123);
	auto& gm502 = sub50.generalMotionPool().add<aris::dynamic::GeneralMotion>();
	gm502.setPosType(aris::dynamic::PosType::PQ);
	auto& gm503 = sub50.generalMotionPool().add<aris::dynamic::GeneralMotion>();
	gm503.setPosType(aris::dynamic::PosType::PM);

	auto& gm521 = sub52.generalMotionPool().add<aris::dynamic::GeneralMotion>();
	gm521.setPosType(aris::dynamic::PosType::PE123);
	auto& gm522 = sub52.generalMotionPool().add<aris::dynamic::GeneralMotion>();
	gm522.setPosType(aris::dynamic::PosType::PQ);
	auto& gm523 = sub52.generalMotionPool().add<aris::dynamic::GeneralMotion>();
	gm523.setPosType(aris::dynamic::PosType::PE123);
	auto& gm524 = sub52.generalMotionPool().add<aris::dynamic::GeneralMotion>();
	gm524.setPosType(aris::dynamic::PosType::PE123);
	auto& gm525 = sub52.generalMotionPool().add<aris::dynamic::GeneralMotion>();
	gm525.setPosType(aris::dynamic::PosType::PM);

	auto& gm531 = sub53.generalMotionPool().add<aris::dynamic::GeneralMotion>();
	gm531.setPosType(aris::dynamic::PosType::PQ);

	auto& gm541 = sub54.generalMotionPool().add<aris::dynamic::GeneralMotion>();
	gm541.setPosType(aris::dynamic::PosType::PE123);
	auto& gm542 = sub54.generalMotionPool().add<aris::dynamic::GeneralMotion>();
	gm542.setPosType(aris::dynamic::PosType::PM);

	auto& mot01 = sub0.motionPool().add<aris::dynamic::Motion>();
	mot01.setAxis(5);
	auto& mot02 = sub0.motionPool().add<aris::dynamic::Motion>();
	auto& mot03 = sub0.motionPool().add<aris::dynamic::Motion>();

	auto& mot11 = sub1.motionPool().add<aris::dynamic::Motion>();
	auto& mot12 = sub1.motionPool().add<aris::dynamic::Motion>();
	mot12.setAxis(5);
	auto& mot13 = sub1.motionPool().add<aris::dynamic::Motion>();
	auto& mot14 = sub1.motionPool().add<aris::dynamic::Motion>();

	auto& mot31 = sub3.motionPool().add<aris::dynamic::Motion>();
	auto& mot32 = sub3.motionPool().add<aris::dynamic::Motion>();
	mot32.setAxis(5);
	auto& mot33 = sub3.motionPool().add<aris::dynamic::Motion>();
	auto& mot34 = sub3.motionPool().add<aris::dynamic::Motion>();
	auto& mot35 = sub3.motionPool().add<aris::dynamic::Motion>();
	auto& mot36 = sub3.motionPool().add<aris::dynamic::Motion>();
	mot36.setAxis(5);

	auto& mot41 = sub4.motionPool().add<aris::dynamic::Motion>();
	auto& mot42 = sub4.motionPool().add<aris::dynamic::Motion>();

	auto& mot501 = sub50.motionPool().add<aris::dynamic::Motion>();
	mot501.setAxis(5);
	auto& mot502 = sub50.motionPool().add<aris::dynamic::Motion>();
	auto& mot503 = sub50.motionPool().add<aris::dynamic::Motion>();

	auto& mot511 = sub51.motionPool().add<aris::dynamic::Motion>();
	auto& mot512 = sub51.motionPool().add<aris::dynamic::Motion>();
	mot512.setAxis(5);
	auto& mot513 = sub51.motionPool().add<aris::dynamic::Motion>();
	auto& mot514 = sub51.motionPool().add<aris::dynamic::Motion>();

	auto& mot531 = sub53.motionPool().add<aris::dynamic::Motion>();
	auto& mot532 = sub53.motionPool().add<aris::dynamic::Motion>();
	mot532.setAxis(5);
	auto& mot533 = sub53.motionPool().add<aris::dynamic::Motion>();
	auto& mot534 = sub53.motionPool().add<aris::dynamic::Motion>();
	auto& mot535 = sub53.motionPool().add<aris::dynamic::Motion>();
	auto& mot536 = sub53.motionPool().add<aris::dynamic::Motion>();
	mot536.setAxis(5);

	auto& mot541 = sub54.motionPool().add<aris::dynamic::Motion>();
	auto& mot542 = sub54.motionPool().add<aris::dynamic::Motion>();

	auto &p00 = sub0.partPool().add<aris::dynamic::Part>("p0");
	auto &mak000 = p00.markerPool().add<aris::dynamic::Marker>("mak0");
	auto& mak001 = p00.markerPool().add<aris::dynamic::Marker>("mak1");

	auto& p01 = sub0.partPool().add<aris::dynamic::Part>("p1");
	auto& mak010 = p01.markerPool().add<aris::dynamic::Marker>("mak0");
	auto& mak011 = p01.markerPool().add<aris::dynamic::Marker>("mak1");

	auto& p20 = sub2.partPool().add<aris::dynamic::Part>("p0");
	auto& mak200 = p20.markerPool().add<aris::dynamic::Marker>("mak0");
	auto& mak201 = p20.markerPool().add<aris::dynamic::Marker>("mak1");

	auto& p21 = sub2.partPool().add<aris::dynamic::Part>("p1");
	auto& mak210 = p21.markerPool().add<aris::dynamic::Marker>("mak0");
	auto& mak211 = p21.markerPool().add<aris::dynamic::Marker>("mak1");

	auto& p500 = sub50.partPool().add<aris::dynamic::Part>("p0");
	auto& mak5000 = p500.markerPool().add<aris::dynamic::Marker>("mak0");
	auto& mak5001 = p500.markerPool().add<aris::dynamic::Marker>("mak1");

	auto& p501 = sub50.partPool().add<aris::dynamic::Part>("p1");
	auto& mak5010 = p501.markerPool().add<aris::dynamic::Marker>("mak0");
	auto& mak5011 = p501.markerPool().add<aris::dynamic::Marker>("mak1");

	auto& p520 = sub52.partPool().add<aris::dynamic::Part>("p0");
	auto& mak5200 = p520.markerPool().add<aris::dynamic::Marker>("mak0");
	auto& mak5201 = p520.markerPool().add<aris::dynamic::Marker>("mak1");

	auto& p521 = sub52.partPool().add<aris::dynamic::Part>("p1");
	auto& mak5210 = p521.markerPool().add<aris::dynamic::Marker>("mak0");
	auto& mak5211 = p521.markerPool().add<aris::dynamic::Marker>("mak1");

	//auto& p500 = sub50.partPool().add<aris::dynamic::Part>("p0");
	//auto& mak5000 = p500.markerPool().add<aris::dynamic::Marker>("mak0");

	multi.init();


	// find marker
	{
		EXPECT_FALSE(multi.findMarker("sub0.p0.mak0") != &mak000) << "\"MultiModel::findMarker\" failed";
		EXPECT_FALSE(multi.findMarker("sub0.p0.mak1") != &mak001) << "\"MultiModel::findMarker\" failed";
		EXPECT_FALSE(multi.findMarker("sub0.p1.mak0") != &mak010) << "\"MultiModel::findMarker\" failed";
		EXPECT_FALSE(multi.findMarker("sub0.p1.mak1") != &mak011) << "\"MultiModel::findMarker\" failed";

		EXPECT_FALSE(multi.findMarker("sub2.p0.mak0") != &mak200) << "\"MultiModel::findMarker\" failed";
		EXPECT_FALSE(multi.findMarker("sub2.p0.mak1") != &mak201) << "\"MultiModel::findMarker\" failed";
		EXPECT_FALSE(multi.findMarker("sub2.p1.mak0") != &mak210) << "\"MultiModel::findMarker\" failed";
		EXPECT_FALSE(multi.findMarker("sub2.p1.mak1") != &mak211) << "\"MultiModel::findMarker\" failed";

		EXPECT_FALSE(multi.findMarker("sub5.sub50.p0.mak0") != &mak5000) << "\"MultiModel::findMarker\" failed";
		EXPECT_FALSE(multi.findMarker("sub5.sub50.p0.mak1") != &mak5001) << "\"MultiModel::findMarker\" failed";
		EXPECT_FALSE(multi.findMarker("sub5.sub50.p1.mak0") != &mak5010) << "\"MultiModel::findMarker\" failed";
		EXPECT_FALSE(multi.findMarker("sub5.sub50.p1.mak1") != &mak5011) << "\"MultiModel::findMarker\" failed";

		EXPECT_FALSE(multi.findMarker("sub5.sub52.p0.mak0") != &mak5200) << "\"MultiModel::findMarker\" failed";
		EXPECT_FALSE(multi.findMarker("sub5.sub52.p0.mak1") != &mak5201) << "\"MultiModel::findMarker\" failed";
		EXPECT_FALSE(multi.findMarker("sub5.sub52.p1.mak0") != &mak5210) << "\"MultiModel::findMarker\" failed";
		EXPECT_FALSE(multi.findMarker("sub5.sub52.p1.mak1") != &mak5211) << "\"MultiModel::findMarker\" failed";
	}

	// test input output size
	{
		EXPECT_FALSE(multi.outputPosSize() != 198) << "\"MultiModel::outputPosSize\" failed";

		EXPECT_FALSE(multi.outputVelSize() != 132) << "\"MultiModel::outputVelSize\" failed";

		EXPECT_FALSE(multi.outputAccSize() != 132) << "\"MultiModel::outputAccSize\" failed";

		EXPECT_FALSE(multi.outputFceSize() != 132) << "\"MultiModel::outputFceSize\" failed";

		EXPECT_FALSE(multi.inputPosSize() != 30) << "\"MultiModel::inputPosSize\" failed";

		EXPECT_FALSE(multi.inputVelSize() != 30) << "\"MultiModel::inputVelSize\" failed";

		EXPECT_FALSE(multi.inputAccSize() != 30) << "\"MultiModel::inputAccSize\" failed";

		EXPECT_FALSE(multi.inputFceSize() != 30) << "\"MultiModel::inputFceSize\" failed";

		EXPECT_FALSE(multi.outputSize() != 22) << "\"MultiModel::eeSize\" failed";
	}

	// test set&get input 1
	{
		const int result_size = 60;
		const int p_size = 30;
		
		double result[result_size];
		std::fill_n(result, result_size, -99.0);

		double mp0[p_size]{ 1,2,3,4,5,6,7,8,9,10,
			11,12,13,14,15,16,17,18,19,20,
			21,22,23,24,25,26,27,28,29,30
		};

		aris::Size sub_num1 = 6;
		aris::Size sub_id1[6]{ 5,3,4,0,2,1 };
		double sub_mp[result_size]{ -1,-2,-3,-4,-5,-6,-7,-8,-9,-10,
			-11,-12,-13,-14,-15,-16,-17,-18,-19,-20,
			-21,-22,-23,-24,-25,-26,-27,-28,-29,-30,
			-99,-99,-99,-99,-99,-99,-99,-99, -99,-99,-99,-99, -99,-99,-99,-99,-99,-99,-99,-99,-99,-99,-99, -99,-99,-99,-99, -99,-99,-99, };
		double actual_mp[result_size]{
			-24, -25, -26, -27, 
			-28, -29, -30,
			-16, -17, -18, -19, -20, -21, 
			-22, - 23, 
			-1, -2, -3, -4, -5, -6, -7, -8, -9, -10, -11, -12, -13, -14, -15,
			-99,-99,-99,-99,-99,-99,-99,-99, -99,-99,-99,-99, -99,-99,-99,-99,-99,-99,-99,-99,-99,-99,-99, -99,-99,-99,-99, -99,-99,-99, };
		
		multi.setInputPos(mp0);
		multi.getInputPos(result);
		EXPECT_FALSE(!aris::dynamic::s_is_equal(p_size, result, mp0, 1e-10)) << "\"MultiModel::setInputPos\" failed";

		multi.setSubInputPos(sub_num1, sub_id1, sub_mp);
		multi.getInputPos(result);
		EXPECT_FALSE(!aris::dynamic::s_is_equal(result_size, result, actual_mp, 1e-10)) << "\"MultiModel::setSubInputPos\" failed";

		multi.setInputPos(mp0);
		multi.getInputPos(result);
		EXPECT_FALSE(!aris::dynamic::s_is_equal(p_size, result, mp0, 1e-10)) << "\"MultiModel::setInputPos\" failed";

		multi.setInputPos(actual_mp);
		multi.getSubInputPos(sub_num1, sub_id1, result);
		EXPECT_FALSE(!aris::dynamic::s_is_equal(result_size, result, sub_mp, 1e-10)) << "\"MultiModel::getSubInputPos\" failed";
		
		
		multi.setInputVel(mp0);
		multi.getInputVel(result);
		EXPECT_FALSE(!aris::dynamic::s_is_equal(p_size, result, mp0, 1e-10)) << "\"MultiModel::setInputVel\" failed";

		multi.setSubInputVel(sub_num1, sub_id1, sub_mp);
		multi.getInputVel(result);
		EXPECT_FALSE(!aris::dynamic::s_is_equal(result_size, result, actual_mp, 1e-10)) << "\"MultiModel::setSubInputVel\" failed";

		multi.setInputVel(mp0);
		multi.getInputVel(result);
		EXPECT_FALSE(!aris::dynamic::s_is_equal(p_size, result, mp0, 1e-10)) << "\"MultiModel::setInputVel\" failed";

		multi.setInputVel(actual_mp);
		multi.getSubInputVel(sub_num1, sub_id1, result);
		EXPECT_FALSE(!aris::dynamic::s_is_equal(result_size, result, sub_mp, 1e-10)) << "\"MultiModel::getSubInputVel\" failed";

		multi.setInputAcc(mp0);
		multi.getInputAcc(result);
		EXPECT_FALSE(!aris::dynamic::s_is_equal(p_size, result, mp0, 1e-10)) << "\"MultiModel::setInputAcc\" failed";

		multi.setSubInputAcc(sub_num1, sub_id1, sub_mp);
		multi.getInputAcc(result);
		EXPECT_FALSE(!aris::dynamic::s_is_equal(result_size, result, actual_mp, 1e-10)) << "\"MultiModel::setSubInputAcc\" failed";

		multi.setInputAcc(mp0);
		multi.getInputAcc(result);
		EXPECT_FALSE(!aris::dynamic::s_is_equal(p_size, result, mp0, 1e-10)) << "\"MultiModel::setInputAcc\" failed";

		multi.setInputAcc(actual_mp);
		multi.getSubInputAcc(sub_num1, sub_id1, result);
		EXPECT_FALSE(!aris::dynamic::s_is_equal(result_size, result, sub_mp, 1e-10)) << "\"MultiModel::getSubInputAcc\" failed";

		multi.setInputFce(mp0);
		multi.getInputFce(result);
		EXPECT_FALSE(!aris::dynamic::s_is_equal(p_size, result, mp0, 1e-10)) << "\"MultiModel::setInputFce\" failed";

		multi.setSubInputFce(sub_num1, sub_id1, sub_mp);
		multi.getInputFce(result);
		EXPECT_FALSE(!aris::dynamic::s_is_equal(result_size, result, actual_mp, 1e-10)) << "\"MultiModel::setSubInputFce\" failed";

		multi.setInputFce(mp0);
		multi.getInputFce(result);
		EXPECT_FALSE(!aris::dynamic::s_is_equal(p_size, result, mp0, 1e-10)) << "\"MultiModel::setInputFce\" failed";

		multi.setInputFce(actual_mp);
		multi.getSubInputFce(sub_num1, sub_id1, result);
		EXPECT_FALSE(!aris::dynamic::s_is_equal(result_size, result, sub_mp, 1e-10)) << "\"MultiModel::getSubInputFce\" failed";

	}

	// test set&get input 2
	{
		const int margin_size = 30;
		const int p_size = 30;
		const int sub_p_size = 23;
		const int sub_num1 = 4;

		double result[margin_size + p_size];
		std::fill_n(result, margin_size + p_size, -99.0);

		double mp0[p_size]{ 1,2,3,4,5,6,7,8,9,10,
			11,12,13,14,15,16,17,18,19,20,
			21,22,23,24,25,26,27,28,29,30
		};

		aris::Size sub_id1[sub_num1]{ 5,4,2,3 };
		double sub_mp[margin_size + sub_p_size]{ 
			-1,-2,-3,-4,-5,-6,-7,-8,-9,-10,
			-11,-12,-13,-14,-15,-16,-17,-18,-19,-20,
			-21,-22,-23,
			-99,-99,-99,-99,-99,-99,-99,-99, -99,-99,-99,-99, -99,-99,-99,-99,-99,-99,-99,-99,-99,-99,-99, -99,-99,-99,-99, -99,-99,-99, };
		double actual_mp[margin_size + p_size]{
			1, 2, 3, 4,
			5, 6, 7,
			-18,-19,-20,-21,-22,-23,
			-16, -17,
			-1, -2, -3, -4, -5, -6, -7, -8, -9, -10, -11, -12, -13, -14, -15,
			-99,-99,-99,-99,-99,-99,-99,-99, -99,-99,-99,-99, -99,-99,-99,-99,-99,-99,-99,-99,-99,-99,-99, -99,-99,-99,-99, -99,-99,-99, };

		multi.setInputPos(mp0);
		multi.getInputPos(result);
		EXPECT_FALSE(!aris::dynamic::s_is_equal(p_size, result, mp0, 1e-10)) << "\"MultiModel::setInputPos\" failed";

		multi.setSubInputPos(sub_num1, sub_id1, sub_mp);
		multi.getInputPos(result);
		EXPECT_FALSE(!aris::dynamic::s_is_equal(margin_size + p_size, result, actual_mp, 1e-10)) << "\"MultiModel::setSubInputPos\" failed";

		multi.setInputPos(mp0);
		multi.getInputPos(result);
		EXPECT_FALSE(!aris::dynamic::s_is_equal(p_size, result, mp0, 1e-10)) << "\"MultiModel::setInputPos\" failed";

		std::fill_n(result, margin_size + sub_p_size, -99);
		multi.setInputPos(actual_mp);
		multi.getSubInputPos(sub_num1, sub_id1, result);
		EXPECT_FALSE(!aris::dynamic::s_is_equal(margin_size + sub_p_size, result, sub_mp, 1e-10)) << "\"MultiModel::getSubInputPos\" failed";

		multi.setInputVel(mp0);
		multi.getInputVel(result);
		EXPECT_FALSE(!aris::dynamic::s_is_equal(p_size, result, mp0, 1e-10)) << "\"MultiModel::setInputVel\" failed";

		multi.setSubInputVel(sub_num1, sub_id1, sub_mp);
		multi.getInputVel(result);
		EXPECT_FALSE(!aris::dynamic::s_is_equal(margin_size + p_size, result, actual_mp, 1e-10)) << "\"MultiModel::setSubInputVel\" failed";

		multi.setInputVel(mp0);
		multi.getInputVel(result);
		EXPECT_FALSE(!aris::dynamic::s_is_equal(p_size, result, mp0, 1e-10)) << "\"MultiModel::setInputVel\" failed";

		std::fill_n(result, margin_size + sub_p_size, -99);
		multi.setInputVel(actual_mp);
		multi.getSubInputVel(sub_num1, sub_id1, result);
		EXPECT_FALSE(!aris::dynamic::s_is_equal(margin_size + sub_p_size, result, sub_mp, 1e-10)) << "\"MultiModel::getSubInputVel\" failed";

		multi.setInputAcc(mp0);
		multi.getInputAcc(result);
		EXPECT_FALSE(!aris::dynamic::s_is_equal(p_size, result, mp0, 1e-10)) << "\"MultiModel::setInputAcc\" failed";

		multi.setSubInputAcc(sub_num1, sub_id1, sub_mp);
		multi.getInputAcc(result);
		EXPECT_FALSE(!aris::dynamic::s_is_equal(margin_size + p_size, result, actual_mp, 1e-10)) << "\"MultiModel::setSubInputAcc\" failed";

		multi.setInputAcc(mp0);
		multi.getInputAcc(result);
		EXPECT_FALSE(!aris::dynamic::s_is_equal(p_size, result, mp0, 1e-10)) << "\"MultiModel::setInputAcc\" failed";

		std::fill_n(result, margin_size + sub_p_size, -99);
		multi.setInputAcc(actual_mp);
		multi.getSubInputAcc(sub_num1, sub_id1, result);
		EXPECT_FALSE(!aris::dynamic::s_is_equal(margin_size + sub_p_size, result, sub_mp, 1e-10)) << "\"MultiModel::getSubInputAcc\" failed";

		multi.setInputFce(mp0);
		multi.getInputFce(result);
		EXPECT_FALSE(!aris::dynamic::s_is_equal(p_size, result, mp0, 1e-10)) << "\"MultiModel::setInputFce\" failed";

		multi.setSubInputFce(sub_num1, sub_id1, sub_mp);
		multi.getInputFce(result);
		EXPECT_FALSE(!aris::dynamic::s_is_equal(margin_size + p_size, result, actual_mp, 1e-10)) << "\"MultiModel::setSubInputFce\" failed";

		multi.setInputFce(mp0);
		multi.getInputFce(result);
		EXPECT_FALSE(!aris::dynamic::s_is_equal(p_size, result, mp0, 1e-10)) << "\"MultiModel::setInputFce\" failed";

		std::fill_n(result, margin_size + sub_p_size, -99);
		multi.setInputFce(actual_mp);
		multi.getSubInputFce(sub_num1, sub_id1, result);
		EXPECT_FALSE(!aris::dynamic::s_is_equal(margin_size + sub_p_size, result, sub_mp, 1e-10)) << "\"MultiModel::getSubInputFce\" failed";

	}

	// test getEeNumOfSubModels
	/*
	{
		std::vector<aris::Size> result1{ 11,1,2,3,5,0 };

		auto r = multi.getSubEeSize({ 5,3,4,0,2,1 });
		EXPECT_FALSE(result1 != multi.getSubEeSize({ 5,3,4,0,2,1 })) << "\"MultiModel::getSubEeSize\" failed";

		aris::Size result1a[6], input1a[6]{ 5,3,4,0,2,1 };
		multi.getSubEeSize(6, input1a, result1a);
		EXPECT_FALSE(std::vector<aris::Size>(result1a, result1a + 6) != result1) << "\"MultiModel::getSubEeSize\" failed";

		std::vector<aris::Size> result2{ 11,2,5,1,1 };
		EXPECT_FALSE(result2 != multi.getSubEeSize({ 5,4,2,3,3 })) << "\"MultiModel::getSubEeSize\" failed";

		aris::Size result2a[5], input2a[5]{ 5,4,2,3,3 };
		multi.getSubEeSize(5, input2a, result2a);
		EXPECT_FALSE(std::vector<aris::Size>(result2a, result2a + 5) != result2) << "\"MultiModel::getSubEeSize\" failed";
	}
	*/

	// test getEeTypes
	{
		//
		// 	ee_types:
		//aris::dynamic::PosType::PE123, aris::dynamic::PosType::PQ, aris::dynamic::PosType::PM,
		//
		//aris::dynamic::PosType::PE123, aris::dynamic::PosType::PQ, aris::dynamic::PosType::PE123, aris::dynamic::PosType::PE123, aris::dynamic::PosType::PM,
		//aris::dynamic::PosType::PQ,
		//aris::dynamic::PosType::PE123, aris::dynamic::PosType::PM


		std::vector<aris::dynamic::PosType> ee_types;
		std::vector<aris::dynamic::PosType> result0{
			aris::dynamic::PosType::PE123, aris::dynamic::PosType::PQ, aris::dynamic::PosType::PM,

			aris::dynamic::PosType::PE123, aris::dynamic::PosType::PQ, aris::dynamic::PosType::PE123, aris::dynamic::PosType::PE123, aris::dynamic::PosType::PM,
			aris::dynamic::PosType::PQ,
			aris::dynamic::PosType::PE123, aris::dynamic::PosType::PM,
			aris::dynamic::PosType::PE123, aris::dynamic::PosType::PQ, aris::dynamic::PosType::PM,

			aris::dynamic::PosType::PE123, aris::dynamic::PosType::PQ, aris::dynamic::PosType::PE123, aris::dynamic::PosType::PE123, aris::dynamic::PosType::PM,
			aris::dynamic::PosType::PQ,
			aris::dynamic::PosType::PE123, aris::dynamic::PosType::PM,
		};

		ee_types = std::vector<aris::dynamic::PosType>(multi.outputPosTypes(), multi.outputPosTypes() + multi.outputSize());
		EXPECT_FALSE(ee_types != result0) << "\"MultiModel::eePosTypes\" failed";

		//ee_types.clear();
		//ee_types.resize(11, aris::dynamic::PosType::PQ);
		//multi.getEeTypes(ee_types.data());
		//if (ee_types != result0)
		//	std::cout << "\"MultiModel::getEeTypes\" failed" << std::endl;


		std::vector<aris::dynamic::PosType> result1{
			aris::dynamic::PosType::PQ,
			aris::dynamic::PosType::PE123, aris::dynamic::PosType::PM,
			aris::dynamic::PosType::PE123, aris::dynamic::PosType::PQ, aris::dynamic::PosType::PM,
			aris::dynamic::PosType::PE123, aris::dynamic::PosType::PQ, aris::dynamic::PosType::PE123, aris::dynamic::PosType::PE123, aris::dynamic::PosType::PM,
		};
		aris::Size sub1[5]{ 3,4,0,2,1 };

		ee_types.resize(multi.subOutputSize(5, sub1));
		multi.getSubOutputPosTypes(5, sub1, ee_types.data());
		EXPECT_FALSE(ee_types != result1) << "\"MultiModel::getSubEeTypes\" failed";

		ee_types.clear();
		ee_types.resize(11, aris::dynamic::PosType::PQ);
		multi.getSubOutputPosTypes(5, sub1, ee_types.data());
		EXPECT_FALSE(ee_types != result1) << "\"MultiModel::getSubEeTypes\" failed";

		std::vector<aris::dynamic::PosType> result2{
			aris::dynamic::PosType::PE123, aris::dynamic::PosType::PM,
			aris::dynamic::PosType::PE123, aris::dynamic::PosType::PQ, aris::dynamic::PosType::PE123, aris::dynamic::PosType::PE123, aris::dynamic::PosType::PM,
			aris::dynamic::PosType::PQ,
			aris::dynamic::PosType::PQ,
		};
		aris::Size sub2[4]{ 4,2,3,3 };

		ee_types.resize(multi.subOutputSize(4, sub2));
		multi.getSubOutputPosTypes(4, sub2, ee_types.data());
		EXPECT_FALSE(ee_types != result2) << "\"MultiModel::getSubEeTypes\" failed";

		ee_types.clear();
		ee_types.resize(9, aris::dynamic::PosType::PQ);
		multi.getSubOutputPosTypes(4, sub2, ee_types.data());
		EXPECT_FALSE(ee_types != result2) << "\"MultiModel::getSubEeTypes\" failed";
	}

	// test getEes
	{
		std::vector<aris::dynamic::MotionBase*> ees;
		std::vector<aris::dynamic::MotionBase*> result0{
			&gm01, &gm02,&gm03,

			&gm21,&gm22,&gm23,&gm24,&gm25,
			&gm31,
			&gm41,&gm42,
		};
		/*
		ees = multi.getEes();
		EXPECT_FALSE(ees != result0) << "\"MultiModel::getEes\" failed";

		ees.clear();
		ees.resize(11, nullptr);
		multi.getEes(ees.data());
		EXPECT_FALSE(ees != result0) << "\"MultiModel::getEes\" failed";
		*/

		std::vector<aris::dynamic::MotionBase*> result1{
			&gm31,
			&gm41,&gm42,
			&gm01,&gm02,&gm03,
			&gm21,&gm22,&gm23,&gm24,&gm25,
		};
		aris::Size sub1[5]{ 3,4,0,2,1 };

		ees.clear();
		ees.resize(11, nullptr);
		multi.getSubOutputMotions(5, sub1, ees.data());
		EXPECT_FALSE(ees != result1) << "\"MultiModel::getSubOutputMotions\" failed";

		std::vector<aris::dynamic::MotionBase*> result2{
			&gm41,&gm42,
			&gm21,&gm22,&gm23,&gm24,&gm25,
			&gm31,
			&gm31,

		};
		aris::Size sub2[4]{ 4,2,3,3 };

		ees.clear();
		ees.resize(9, nullptr);
		multi.getSubOutputMotions(4, sub2, ees.data());
		EXPECT_FALSE(ees != result2) << "\"MultiModel::getSubOutputMotions\" failed";
	}

	// test getMotionNumOfSubModels
	{
		/*
		// 3 4 0 6 2
		std::vector<aris::Size> result1{ 6,2,3,0,4 };
		EXPECT_FALSE(result1 != multi.getMotionNumOfSubModels({ 3,4,0,2,1 })) << "\"MultiModel::getMotionNumOfSubModels\" failed";

		aris::Size result1a[5], input1a[5]{ 3,4,0,2,1 };
		multi.getMotionNumOfSubModels(5, input1a, result1a);
		EXPECT_FALSE(std::vector<aris::Size>(result1a, result1a + 5) != result1) << "\"MultiModel::getMotionNumOfSubModels\" failed";

		std::vector<aris::Size> result2{ 2,0,6,6 };
		EXPECT_FALSE(result2 != multi.getMotionNumOfSubModels({ 4,2,3,3 })) << "\"MultiModel::getMotionNumOfSubModels\" failed";

		aris::Size result2a[4], input2a[4]{ 4,2,3,3 };
		multi.getMotionNumOfSubModels(4, input2a, result2a);
		EXPECT_FALSE(std::vector<aris::Size>(result2a, result2a + 4) != result2) << "\"MultiModel::getMotionNumOfSubModels\" failed";
			*/
	}

	// test getMotionTypes
	{
		std::vector<aris::dynamic::PosType> ee_types;
		std::vector<aris::dynamic::PosType> result0{
			aris::dynamic::PosType::A, aris::dynamic::PosType::X, aris::dynamic::PosType::X,
			aris::dynamic::PosType::X, aris::dynamic::PosType::A, aris::dynamic::PosType::X, aris::dynamic::PosType::X,

			aris::dynamic::PosType::X, aris::dynamic::PosType::A, aris::dynamic::PosType::X, aris::dynamic::PosType::X,aris::dynamic::PosType::X, aris::dynamic::PosType::A,
			aris::dynamic::PosType::X, aris::dynamic::PosType::X,
		};

		//ee_types = multi.getMotionTypes();
		//if (ee_types != result0)
		//	std::cout << "\"MultiModel::getMotionTypes\" failed" << std::endl;

		//ee_types.clear();
		//ee_types.resize(15, aris::dynamic::PosType::PQ);
		//multi.getMotionTypes(ee_types.data());
		//if (ee_types != result0)
		//	std::cout << "\"MultiModel::getMotionTypes\" failed" << std::endl;


		std::vector<aris::dynamic::PosType> result1{
			aris::dynamic::PosType::X, aris::dynamic::PosType::A, aris::dynamic::PosType::X, aris::dynamic::PosType::X,aris::dynamic::PosType::X, aris::dynamic::PosType::A,
			aris::dynamic::PosType::X, aris::dynamic::PosType::X,
			aris::dynamic::PosType::A, aris::dynamic::PosType::X, aris::dynamic::PosType::X,
			aris::dynamic::PosType::X, aris::dynamic::PosType::A, aris::dynamic::PosType::X, aris::dynamic::PosType::X,
		};
		aris::Size sub1[5]{ 3,4,0,2,1 };

		//ee_types = multi.getMotionTypes({ 3,4,0,2,1 });
		//if (ee_types != result1)
		//	std::cout << "\"MultiModel::getMotionTypes\" failed" << std::endl;

		//ee_types.clear();
		//ee_types.resize(15, aris::dynamic::PosType::PQ);
		//multi.getMotionTypes(5, sub1, ee_types.data());
		//if (ee_types != result1)
		//	std::cout << "\"MultiModel::getMotionTypes\" failed" << std::endl;

		std::vector<aris::dynamic::PosType> result2{
			aris::dynamic::PosType::X, aris::dynamic::PosType::X,
			aris::dynamic::PosType::X, aris::dynamic::PosType::A, aris::dynamic::PosType::X, aris::dynamic::PosType::X,aris::dynamic::PosType::X, aris::dynamic::PosType::A,
			aris::dynamic::PosType::X, aris::dynamic::PosType::A, aris::dynamic::PosType::X, aris::dynamic::PosType::X,aris::dynamic::PosType::X, aris::dynamic::PosType::A,
		};
		aris::Size sub2[4]{ 4,2,3,3 };

		//ee_types = multi.getMotionTypes({ 4,2,3,3 });
		//if (ee_types != result2)
		//	std::cout << "\"MultiModel::getMotionTypes\" failed" << std::endl;

		//ee_types.clear();
		//ee_types.resize(14, aris::dynamic::PosType::PQ);
		//multi.getMotionTypes(4, sub2, ee_types.data());
		//if (ee_types != result2)
		//	std::cout << "\"MultiModel::getMotionTypes\" failed" << std::endl;
	}

	// test getMotions
	{
		std::vector<aris::dynamic::Motion*> mots;
		std::vector<aris::dynamic::Motion*> result0{
			&mot01, &mot02,&mot03,
			&mot11,&mot12,&mot13,&mot14,

			&mot31,&mot32,&mot33,&mot34,&mot35,&mot36,
			&mot41,&mot42,
		};

		std::vector<aris::dynamic::Motion*> result1{
			&mot31,&mot32,&mot33,&mot34,&mot35,&mot36,
			&mot41,&mot42,
			&mot01,&mot02,&mot03,
			&mot11,&mot12,&mot13,&mot14,
		};
		aris::Size sub1[5]{ 3,4,0,2,1 };

		mots.clear();
		mots.resize(15, nullptr);
		multi.getSubInputMotions(5, sub1, mots.data());
		EXPECT_FALSE(mots != result1) << "\"MultiModel::getMotions\" failed";

		std::vector<aris::dynamic::Motion*> result2{
			&mot41,&mot42,
			&mot31,&mot32,&mot33,&mot34,&mot35,&mot36,
			&mot31,&mot32,&mot33,&mot34,&mot35,&mot36,
		};
		aris::Size sub2[4]{ 4,2,3,3 };

		mots.clear();
		mots.resize(14, nullptr);
		multi.getSubInputMotions(4, sub2, mots.data());
		EXPECT_FALSE(mots != result2) << "\"MultiModel::getMotions\" failed";
	}

	// test getMotionIds
	{
		std::vector<aris::Size> mots;
		std::vector<aris::Size> result0{
			0,1,2,
			3,4,5,6,

			7,8,9,10,11,12,
			13,14,
		};
		/*
		mots = multi.getMotionIds();
		EXPECT_FALSE(mots != result0) << "\"MultiModel::getMotionIds\" failed";

		mots.clear();
		mots.resize(15, -1);
		multi.getMotionIds(mots.data());
		EXPECT_FALSE(mots != result0) << "\"MultiModel::getMotionIds\" failed";
			*/

		std::vector<aris::Size> result1{
			7,8,9,10,11,12,
			13,14,
			0,1,2,
			3,4,5,6,
		};
		aris::Size sub1[5]{ 3,4,0,2,1 };
		/*
		mots = multi.getMotionIds({ 3,4,0,2,1 });
		EXPECT_FALSE(mots != result1) << "\"MultiModel::getMotionIds\" failed";

		mots.clear();
		mots.resize(15, -1);
		multi.getMotionIds(5, sub1, mots.data());
		EXPECT_FALSE(mots != result1) << "\"MultiModel::getMotionIds\" failed";
			*/
		std::vector<aris::Size> result2{
			13,14,
			7,8,9,10,11,12,
			7,8,9,10,11,12,
		};
		aris::Size sub2[4]{ 4,2,3,3 };
		/*
		mots = multi.getMotionIds({ 4,2,3,3 });
		EXPECT_FALSE(mots != result2) << "\"MultiModel::getMotionIds\" failed";

		mots.clear();
		mots.resize(14, -1);
		multi.getMotionIds(4, sub2, mots.data());
		EXPECT_FALSE(mots != result2) << "\"MultiModel::getMotionIds\" failed";
			*/
	}




}


