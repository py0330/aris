#include "test_dynamic_model.h"
#include <aris/dynamic/dynamic.hpp>

void test_multi_model() {
	
	aris::dynamic::MultiModel multi;

	auto &sub0 = multi.subModels().add<aris::dynamic::Model>();
	auto& sub1 = multi.subModels().add<aris::dynamic::Model>();
	auto& sub2 = multi.subModels().add<aris::dynamic::Model>();
	auto& sub3 = multi.subModels().add<aris::dynamic::Model>();
	auto& sub4 = multi.subModels().add<aris::dynamic::Model>();

	auto &gm01 = sub0.generalMotionPool().add<aris::dynamic::GeneralMotion>();
	gm01.setPoseType(aris::dynamic::GeneralMotion::PoseType::EULER123);
	auto &gm02 = sub0.generalMotionPool().add<aris::dynamic::GeneralMotion>();
	gm02.setPoseType(aris::dynamic::GeneralMotion::PoseType::QUATERNION);
	auto& gm03 = sub0.generalMotionPool().add<aris::dynamic::GeneralMotion>();
	gm03.setPoseType(aris::dynamic::GeneralMotion::PoseType::POSE_MATRIX);

	auto& gm21 = sub2.generalMotionPool().add<aris::dynamic::GeneralMotion>();
	gm21.setPoseType(aris::dynamic::GeneralMotion::PoseType::EULER123);
	auto& gm22 = sub2.generalMotionPool().add<aris::dynamic::GeneralMotion>();
	gm22.setPoseType(aris::dynamic::GeneralMotion::PoseType::QUATERNION);
	auto& gm23 = sub2.generalMotionPool().add<aris::dynamic::GeneralMotion>();
	gm23.setPoseType(aris::dynamic::GeneralMotion::PoseType::EULER123);
	auto& gm24 = sub2.generalMotionPool().add<aris::dynamic::GeneralMotion>();
	gm24.setPoseType(aris::dynamic::GeneralMotion::PoseType::EULER123);
	auto& gm25 = sub2.generalMotionPool().add<aris::dynamic::GeneralMotion>();
	gm25.setPoseType(aris::dynamic::GeneralMotion::PoseType::POSE_MATRIX);

	auto& gm31 = sub3.generalMotionPool().add<aris::dynamic::GeneralMotion>();
	gm31.setPoseType(aris::dynamic::GeneralMotion::PoseType::QUATERNION);

	auto& gm41 = sub4.generalMotionPool().add<aris::dynamic::GeneralMotion>();
	gm41.setPoseType(aris::dynamic::GeneralMotion::PoseType::EULER123);
	auto& gm42 = sub4.generalMotionPool().add<aris::dynamic::GeneralMotion>();
	gm42.setPoseType(aris::dynamic::GeneralMotion::PoseType::POSE_MATRIX);

	multi.init();

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

	// test input output size
	{
		if (multi.outputPosSize() != 99)
			std::cout << "\"MultiModel::outputPosSize\" failed" << std::endl;

		if (multi.outputVelSize() != 66)
			std::cout << "\"MultiModel::outputVelSize\" failed" << std::endl;

		if (multi.outputAccSize() != 66)
			std::cout << "\"MultiModel::outputAccSize\" failed" << std::endl;

		if (multi.outputFceSize() != 66)
			std::cout << "\"MultiModel::outputFceSize\" failed" << std::endl;

		if (multi.inputPosSize() != 15)
			std::cout << "\"MultiModel::inputPosSize\" failed" << std::endl;

		if (multi.inputVelSize() != 15)
			std::cout << "\"MultiModel::inputVelSize\" failed" << std::endl;

		if (multi.inputAccSize() != 15)
			std::cout << "\"MultiModel::inputAccSize\" failed" << std::endl;

		if (multi.inputFceSize() != 15)
			std::cout << "\"MultiModel::inputFceSize\" failed" << std::endl;
	}
	
	// test getEeNumOfSubModels
	{
		std::vector<aris::Size> result1{ 1,2,3,5,0 };
		if (result1 != multi.getSubEeSize({ 3,4,0,2,1 }))
			std::cout << "\"MultiModel::getSubEeSize\" failed" << std::endl;

		aris::Size result1a[5], input1a[5]{ 3,4,0,2,1 };
		multi.getSubEeSize(5, input1a, result1a);
		if (std::vector<aris::Size>(result1a, result1a+5) != result1)
			std::cout << "\"MultiModel::getSubEeSize\" failed" << std::endl;

		std::vector<aris::Size> result2{ 2,5,1,1 };
		if (result2 != multi.getSubEeSize({ 4,2,3,3 }))
			std::cout << "\"MultiModel::getSubEeSize\" failed" << std::endl;
		
		aris::Size result2a[4], input2a[4]{ 4,2,3,3 };
		multi.getSubEeSize(4, input2a, result2a);
		if (std::vector<aris::Size>(result2a, result2a + 4) != result2)
			std::cout << "\"MultiModel::getSubEeSize\" failed" << std::endl;
	}
	
	// test getEeTypes
	{
		//
		// 	ee_types:
		//aris::dynamic::EEType::PE123, aris::dynamic::EEType::PQ, aris::dynamic::EEType::PM,
		//
		//aris::dynamic::EEType::PE123, aris::dynamic::EEType::PQ, aris::dynamic::EEType::PE123, aris::dynamic::EEType::PE123, aris::dynamic::EEType::PM,
		//aris::dynamic::EEType::PQ,
		//aris::dynamic::EEType::PE123, aris::dynamic::EEType::PM


		std::vector<aris::dynamic::EEType> ee_types;
		std::vector<aris::dynamic::EEType> result0{
			aris::dynamic::EEType::PE123, aris::dynamic::EEType::PQ, aris::dynamic::EEType::PM,
			
			aris::dynamic::EEType::PE123, aris::dynamic::EEType::PQ, aris::dynamic::EEType::PE123, aris::dynamic::EEType::PE123, aris::dynamic::EEType::PM,
			aris::dynamic::EEType::PQ,
			aris::dynamic::EEType::PE123, aris::dynamic::EEType::PM
		};

		ee_types = std::vector<aris::dynamic::EEType>(multi.eeTypes(), multi.eeTypes() + multi.eeSize());
		if (ee_types != result0)
			std::cout << "\"MultiModel::eeTypes\" failed" << std::endl;

		//ee_types.clear();
		//ee_types.resize(11, aris::dynamic::EEType::PQ);
		//multi.getSubEeSize(ee_types.data());
		//if (ee_types != result0)
		//	std::cout << "\"MultiModel::getSubEeSize\" failed" << std::endl;


		std::vector<aris::dynamic::EEType> result1{
			aris::dynamic::EEType::PQ,
			aris::dynamic::EEType::PE123, aris::dynamic::EEType::PM,
			aris::dynamic::EEType::PE123, aris::dynamic::EEType::PQ, aris::dynamic::EEType::PM,
			aris::dynamic::EEType::PE123, aris::dynamic::EEType::PQ, aris::dynamic::EEType::PE123, aris::dynamic::EEType::PE123, aris::dynamic::EEType::PM,
		};
		aris::Size sub1[5]{ 3,4,0,2,1 };

		ee_types = multi.getSubEeTypes({ 3,4,0,2,1 });
		if (ee_types != result1)
			std::cout << "\"MultiModel::getSubEeTypes\" failed" << std::endl;

		ee_types.clear();
		ee_types.resize(11, aris::dynamic::EEType::PQ);
		multi.getSubEeTypes(5, sub1, ee_types.data());
		if (ee_types != result1)
			std::cout << "\"MultiModel::getSubEeTypes\" failed" << std::endl;

		std::vector<aris::dynamic::EEType> result2{
			aris::dynamic::EEType::PE123, aris::dynamic::EEType::PM,
			aris::dynamic::EEType::PE123, aris::dynamic::EEType::PQ, aris::dynamic::EEType::PE123, aris::dynamic::EEType::PE123, aris::dynamic::EEType::PM,
			aris::dynamic::EEType::PQ, 
			aris::dynamic::EEType::PQ,
		};
		aris::Size sub2[4]{ 4,2,3,3 };

		ee_types = multi.getSubEeTypes({ 4,2,3,3 });
		if (ee_types != result2)
			std::cout << "\"MultiModel::getSubEeTypes\" failed" << std::endl;

		ee_types.clear();
		ee_types.resize(9, aris::dynamic::EEType::PQ);
		multi.getSubEeTypes(4, sub2, ee_types.data());
		if (ee_types != result2)
			std::cout << "\"MultiModel::getSubEeTypes\" failed" << std::endl;
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

		ees = multi.getEes();
		if (ees != result0)
			std::cout << "\"MultiModel::getEes\" failed" << std::endl;

		ees.clear();
		ees.resize(11, nullptr);
		multi.getEes(ees.data());
		if (ees != result0)
			std::cout << "\"MultiModel::getEes\" failed" << std::endl;


		std::vector<aris::dynamic::MotionBase*> result1{
			&gm31,
			& gm41,& gm42,
			& gm01,& gm02,& gm03,
			& gm21,& gm22,& gm23,& gm24,& gm25, 
		};
		aris::Size sub1[5]{ 3,4,0,2,1 };

		ees = multi.getSubEes({ 3,4,0,2,1 });
		if (ees != result1)
			std::cout << "\"MultiModel::getSubEes\" failed" << std::endl;

		ees.clear();
		ees.resize(11, nullptr);
		multi.getSubEes(5, sub1, ees.data());
		if (ees != result1)
			std::cout << "\"MultiModel::getSubEes\" failed" << std::endl;

		std::vector<aris::dynamic::MotionBase*> result2{
			&gm41,& gm42,
			& gm21,& gm22,& gm23,& gm24,& gm25,
			& gm31,
			& gm31,

		};
		aris::Size sub2[4]{ 4,2,3,3 };

		ees = multi.getSubEes({ 4,2,3,3 });
		if (ees != result2)
			std::cout << "\"MultiModel::getSubEes\" failed" << std::endl;

		ees.clear();
		ees.resize(9, nullptr);
		multi.getSubEes(4, sub2, ees.data());
		if (ees != result2)
			std::cout << "\"MultiModel::getSubEes\" failed" << std::endl;
	}

	// test getMotionNumOfSubModels
	{
		// 3 4 0 6 2
		std::vector<aris::Size> result1{ 6,2,3,0,4 };
		if (result1 != multi.getMotionNumOfSubModels({ 3,4,0,2,1 }))
			std::cout << "\"MultiModel::getMotionNumOfSubModels\" failed" << std::endl;

		aris::Size result1a[5], input1a[5]{ 3,4,0,2,1 };
		multi.getMotionNumOfSubModels(5, input1a, result1a);
		if (std::vector<aris::Size>(result1a, result1a + 5) != result1)
			std::cout << "\"MultiModel::getMotionNumOfSubModels\" failed" << std::endl;

		std::vector<aris::Size> result2{ 2,0,6,6 };
		if (result2 != multi.getMotionNumOfSubModels({ 4,2,3,3 }))
			std::cout << "\"MultiModel::getMotionNumOfSubModels\" failed" << std::endl;

		aris::Size result2a[4], input2a[4]{ 4,2,3,3 };
		multi.getMotionNumOfSubModels(4, input2a, result2a);
		if (std::vector<aris::Size>(result2a, result2a + 4) != result2)
			std::cout << "\"MultiModel::getMotionNumOfSubModels\" failed" << std::endl;
	}
	
	// test getMotionTypes
	{
		std::vector<aris::dynamic::EEType> ee_types;
		std::vector<aris::dynamic::EEType> result0{
			aris::dynamic::EEType::A, aris::dynamic::EEType::X, aris::dynamic::EEType::X,
			aris::dynamic::EEType::X, aris::dynamic::EEType::A, aris::dynamic::EEType::X, aris::dynamic::EEType::X,
			
			aris::dynamic::EEType::X, aris::dynamic::EEType::A, aris::dynamic::EEType::X, aris::dynamic::EEType::X,aris::dynamic::EEType::X, aris::dynamic::EEType::A,
			aris::dynamic::EEType::X, aris::dynamic::EEType::X,
		};

		ee_types = multi.getMotionTypes();
		if (ee_types != result0)
			std::cout << "\"MultiModel::getMotionTypes\" failed" << std::endl;

		ee_types.clear();
		ee_types.resize(15, aris::dynamic::EEType::PQ);
		multi.getMotionTypes(ee_types.data());
		if (ee_types != result0)
			std::cout << "\"MultiModel::getMotionTypes\" failed" << std::endl;


		std::vector<aris::dynamic::EEType> result1{
			aris::dynamic::EEType::X, aris::dynamic::EEType::A, aris::dynamic::EEType::X, aris::dynamic::EEType::X,aris::dynamic::EEType::X, aris::dynamic::EEType::A,
			aris::dynamic::EEType::X, aris::dynamic::EEType::X,
			aris::dynamic::EEType::A, aris::dynamic::EEType::X, aris::dynamic::EEType::X,
			aris::dynamic::EEType::X, aris::dynamic::EEType::A, aris::dynamic::EEType::X, aris::dynamic::EEType::X,
		};
		aris::Size sub1[5]{ 3,4,0,2,1 };

		ee_types = multi.getMotionTypes({ 3,4,0,2,1 });
		if (ee_types != result1)
			std::cout << "\"MultiModel::getMotionTypes\" failed" << std::endl;

		ee_types.clear();
		ee_types.resize(15, aris::dynamic::EEType::PQ);
		multi.getMotionTypes(5, sub1, ee_types.data());
		if (ee_types != result1)
			std::cout << "\"MultiModel::getMotionTypes\" failed" << std::endl;

		std::vector<aris::dynamic::EEType> result2{
			aris::dynamic::EEType::X, aris::dynamic::EEType::X,
			aris::dynamic::EEType::X, aris::dynamic::EEType::A, aris::dynamic::EEType::X, aris::dynamic::EEType::X,aris::dynamic::EEType::X, aris::dynamic::EEType::A,
			aris::dynamic::EEType::X, aris::dynamic::EEType::A, aris::dynamic::EEType::X, aris::dynamic::EEType::X,aris::dynamic::EEType::X, aris::dynamic::EEType::A,
		};
		aris::Size sub2[4]{ 4,2,3,3 };

		ee_types = multi.getMotionTypes({ 4,2,3,3 });
		if (ee_types != result2)
			std::cout << "\"MultiModel::getMotionTypes\" failed" << std::endl;

		ee_types.clear();
		ee_types.resize(14, aris::dynamic::EEType::PQ);
		multi.getMotionTypes(4, sub2, ee_types.data());
		if (ee_types != result2)
			std::cout << "\"MultiModel::getMotionTypes\" failed" << std::endl;
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

		mots = multi.getMotions();
		if (mots != result0)
			std::cout << "\"MultiModel::getMotions\" failed" << std::endl;

		mots.clear();
		mots.resize(15, nullptr);
		multi.getMotions(mots.data());
		if (mots != result0)
			std::cout << "\"MultiModel::getMotions\" failed" << std::endl;


		std::vector<aris::dynamic::Motion*> result1{
			&mot31,& mot32,& mot33,& mot34,& mot35,& mot36,
			& mot41,& mot42,
			& mot01,& mot02,& mot03,
			& mot11,& mot12,& mot13,& mot14,
		};
		aris::Size sub1[5]{ 3,4,0,2,1 };

		mots = multi.getMotions({ 3,4,0,2,1 });
		if (mots != result1)
			std::cout << "\"MultiModel::getMotions\" failed" << std::endl;

		mots.clear();
		mots.resize(15, nullptr);
		multi.getMotions(5, sub1, mots.data());
		if (mots != result1)
			std::cout << "\"MultiModel::getMotions\" failed" << std::endl;

		std::vector<aris::dynamic::Motion*> result2{
			& mot41,& mot42,
			& mot31,& mot32,& mot33,& mot34,& mot35,& mot36,
			& mot31,& mot32,& mot33,& mot34,& mot35,& mot36,
		};
		aris::Size sub2[4]{ 4,2,3,3 };

		mots = multi.getMotions({ 4,2,3,3 });
		if (mots != result2)
			std::cout << "\"MultiModel::getMotions\" failed" << std::endl;

		mots.clear();
		mots.resize(14, nullptr);
		multi.getMotions(4, sub2, mots.data());
		if (mots != result2)
			std::cout << "\"MultiModel::getMotions\" failed" << std::endl;
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

		mots = multi.getMotionIds();
		if (mots != result0)
			std::cout << "\"MultiModel::getMotionIds\" failed" << std::endl;

		mots.clear();
		mots.resize(15, -1);
		multi.getMotionIds(mots.data());
		if (mots != result0)
			std::cout << "\"MultiModel::getMotionIds\" failed" << std::endl;


		std::vector<aris::Size> result1{
			7,8,9,10,11,12,
			13,14,
			0,1,2,
			3,4,5,6,
		};
		aris::Size sub1[5]{ 3,4,0,2,1 };

		mots = multi.getMotionIds({ 3,4,0,2,1 });
		if (mots != result1)
			std::cout << "\"MultiModel::getMotionIds\" failed" << std::endl;

		mots.clear();
		mots.resize(15, -1);
		multi.getMotionIds(5, sub1, mots.data());
		if (mots != result1)
			std::cout << "\"MultiModel::getMotionIds\" failed" << std::endl;

		std::vector<aris::Size> result2{
			13,14,
			7,8,9,10,11,12,
			7,8,9,10,11,12,
		};
		aris::Size sub2[4]{ 4,2,3,3 };

		mots = multi.getMotionIds({ 4,2,3,3 });
		if (mots != result2)
			std::cout << "\"MultiModel::getMotionIds\" failed" << std::endl;

		mots.clear();
		mots.resize(14, -1);
		multi.getMotionIds(4, sub2, mots.data());
		if (mots != result2)
			std::cout << "\"MultiModel::getMotionIds\" failed" << std::endl;
	}

	


}

void test_multi_model2() {

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
	gm01.setPoseType(aris::dynamic::GeneralMotion::PoseType::EULER123);
	auto& gm02 = sub0.generalMotionPool().add<aris::dynamic::GeneralMotion>();
	gm02.setPoseType(aris::dynamic::GeneralMotion::PoseType::QUATERNION);
	auto& gm03 = sub0.generalMotionPool().add<aris::dynamic::GeneralMotion>();
	gm03.setPoseType(aris::dynamic::GeneralMotion::PoseType::POSE_MATRIX);

	auto& gm21 = sub2.generalMotionPool().add<aris::dynamic::GeneralMotion>();
	gm21.setPoseType(aris::dynamic::GeneralMotion::PoseType::EULER123);
	auto& gm22 = sub2.generalMotionPool().add<aris::dynamic::GeneralMotion>();
	gm22.setPoseType(aris::dynamic::GeneralMotion::PoseType::QUATERNION);
	auto& gm23 = sub2.generalMotionPool().add<aris::dynamic::GeneralMotion>();
	gm23.setPoseType(aris::dynamic::GeneralMotion::PoseType::EULER123);
	auto& gm24 = sub2.generalMotionPool().add<aris::dynamic::GeneralMotion>();
	gm24.setPoseType(aris::dynamic::GeneralMotion::PoseType::EULER123);
	auto& gm25 = sub2.generalMotionPool().add<aris::dynamic::GeneralMotion>();
	gm25.setPoseType(aris::dynamic::GeneralMotion::PoseType::POSE_MATRIX);

	auto& gm31 = sub3.generalMotionPool().add<aris::dynamic::GeneralMotion>();
	gm31.setPoseType(aris::dynamic::GeneralMotion::PoseType::QUATERNION);

	auto& gm41 = sub4.generalMotionPool().add<aris::dynamic::GeneralMotion>();
	gm41.setPoseType(aris::dynamic::GeneralMotion::PoseType::EULER123);
	auto& gm42 = sub4.generalMotionPool().add<aris::dynamic::GeneralMotion>();
	gm42.setPoseType(aris::dynamic::GeneralMotion::PoseType::POSE_MATRIX);

	auto& gm501 = sub50.generalMotionPool().add<aris::dynamic::GeneralMotion>();
	gm501.setPoseType(aris::dynamic::GeneralMotion::PoseType::EULER123);
	auto& gm502 = sub50.generalMotionPool().add<aris::dynamic::GeneralMotion>();
	gm502.setPoseType(aris::dynamic::GeneralMotion::PoseType::QUATERNION);
	auto& gm503 = sub50.generalMotionPool().add<aris::dynamic::GeneralMotion>();
	gm503.setPoseType(aris::dynamic::GeneralMotion::PoseType::POSE_MATRIX);

	auto& gm521 = sub52.generalMotionPool().add<aris::dynamic::GeneralMotion>();
	gm521.setPoseType(aris::dynamic::GeneralMotion::PoseType::EULER123);
	auto& gm522 = sub52.generalMotionPool().add<aris::dynamic::GeneralMotion>();
	gm522.setPoseType(aris::dynamic::GeneralMotion::PoseType::QUATERNION);
	auto& gm523 = sub52.generalMotionPool().add<aris::dynamic::GeneralMotion>();
	gm523.setPoseType(aris::dynamic::GeneralMotion::PoseType::EULER123);
	auto& gm524 = sub52.generalMotionPool().add<aris::dynamic::GeneralMotion>();
	gm524.setPoseType(aris::dynamic::GeneralMotion::PoseType::EULER123);
	auto& gm525 = sub52.generalMotionPool().add<aris::dynamic::GeneralMotion>();
	gm525.setPoseType(aris::dynamic::GeneralMotion::PoseType::POSE_MATRIX);

	auto& gm531 = sub53.generalMotionPool().add<aris::dynamic::GeneralMotion>();
	gm531.setPoseType(aris::dynamic::GeneralMotion::PoseType::QUATERNION);

	auto& gm541 = sub54.generalMotionPool().add<aris::dynamic::GeneralMotion>();
	gm541.setPoseType(aris::dynamic::GeneralMotion::PoseType::EULER123);
	auto& gm542 = sub54.generalMotionPool().add<aris::dynamic::GeneralMotion>();
	gm542.setPoseType(aris::dynamic::GeneralMotion::PoseType::POSE_MATRIX);

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
		if (multi.findMarker("sub0.p0.mak0") != &mak000)
			std::cout << "\"MultiModel::findMarker\" failed" << std::endl;
		if (multi.findMarker("sub0.p0.mak1") != &mak001)
			std::cout << "\"MultiModel::findMarker\" failed" << std::endl;
		if (multi.findMarker("sub0.p1.mak0") != &mak010)
			std::cout << "\"MultiModel::findMarker\" failed" << std::endl;
		if (multi.findMarker("sub0.p1.mak1") != &mak011)
			std::cout << "\"MultiModel::findMarker\" failed" << std::endl;

		if (multi.findMarker("sub2.p0.mak0") != &mak200)
			std::cout << "\"MultiModel::findMarker\" failed" << std::endl;
		if (multi.findMarker("sub2.p0.mak1") != &mak201)
			std::cout << "\"MultiModel::findMarker\" failed" << std::endl;
		if (multi.findMarker("sub2.p1.mak0") != &mak210)
			std::cout << "\"MultiModel::findMarker\" failed" << std::endl;
		if (multi.findMarker("sub2.p1.mak1") != &mak211)
			std::cout << "\"MultiModel::findMarker\" failed" << std::endl;

		if (multi.findMarker("sub5.sub50.p0.mak0") != &mak5000)
			std::cout << "\"MultiModel::findMarker\" failed" << std::endl;
		if (multi.findMarker("sub5.sub50.p0.mak1") != &mak5001)
			std::cout << "\"MultiModel::findMarker\" failed" << std::endl;
		if (multi.findMarker("sub5.sub50.p1.mak0") != &mak5010)
			std::cout << "\"MultiModel::findMarker\" failed" << std::endl;
		if (multi.findMarker("sub5.sub50.p1.mak1") != &mak5011)
			std::cout << "\"MultiModel::findMarker\" failed" << std::endl;

		if (multi.findMarker("sub5.sub52.p0.mak0") != &mak5200)
			std::cout << "\"MultiModel::findMarker\" failed" << std::endl;
		if (multi.findMarker("sub5.sub52.p0.mak1") != &mak5201)
			std::cout << "\"MultiModel::findMarker\" failed" << std::endl;
		if (multi.findMarker("sub5.sub52.p1.mak0") != &mak5210)
			std::cout << "\"MultiModel::findMarker\" failed" << std::endl;
		if (multi.findMarker("sub5.sub52.p1.mak1") != &mak5211)
			std::cout << "\"MultiModel::findMarker\" failed" << std::endl;
	}

	// test input output size
	{
		if (multi.outputPosSize() != 198)
			std::cout << "\"MultiModel::outputPosSize\" failed" << std::endl;

		if (multi.outputVelSize() != 132)
			std::cout << "\"MultiModel::outputVelSize\" failed" << std::endl;

		if (multi.outputAccSize() != 132)
			std::cout << "\"MultiModel::outputAccSize\" failed" << std::endl;

		if (multi.outputFceSize() != 132)
			std::cout << "\"MultiModel::outputFceSize\" failed" << std::endl;

		if (multi.inputPosSize() != 30)
			std::cout << "\"MultiModel::inputPosSize\" failed" << std::endl;

		if (multi.inputVelSize() != 30)
			std::cout << "\"MultiModel::inputVelSize\" failed" << std::endl;

		if (multi.inputAccSize() != 30)
			std::cout << "\"MultiModel::inputAccSize\" failed" << std::endl;

		if (multi.inputFceSize() != 30)
			std::cout << "\"MultiModel::inputFceSize\" failed" << std::endl;

		if (multi.eeSize() != 22)
			std::cout << "\"MultiModel::eeSize\" failed" << std::endl;
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
		if (!aris::dynamic::s_is_equal(p_size, result, mp0, 1e-10))
			std::cout << "\"MultiModel::setInputPos\" failed" << std::endl;

		multi.setSubInputPos(sub_num1, sub_id1, sub_mp);
		multi.getInputPos(result);
		if (!aris::dynamic::s_is_equal(result_size, result, actual_mp, 1e-10))
			std::cout << "\"MultiModel::setSubInputPos\" failed" << std::endl;

		multi.setInputPos(mp0);
		multi.getInputPos(result);
		if (!aris::dynamic::s_is_equal(p_size, result, mp0, 1e-10))
			std::cout << "\"MultiModel::setInputPos\" failed" << std::endl;

		multi.setInputPos(actual_mp);
		multi.getSubInputPos(sub_num1, sub_id1, result);
		if (!aris::dynamic::s_is_equal(result_size, result, sub_mp, 1e-10))
			std::cout << "\"MultiModel::getSubInputPos\" failed" << std::endl;
		
		
		multi.setInputVel(mp0);
		multi.getInputVel(result);
		if (!aris::dynamic::s_is_equal(p_size, result, mp0, 1e-10))
			std::cout << "\"MultiModel::setInputVel\" failed" << std::endl;

		multi.setSubInputVel(sub_num1, sub_id1, sub_mp);
		multi.getInputVel(result);
		if (!aris::dynamic::s_is_equal(result_size, result, actual_mp, 1e-10))
			std::cout << "\"MultiModel::setSubInputVel\" failed" << std::endl;

		multi.setInputVel(mp0);
		multi.getInputVel(result);
		if (!aris::dynamic::s_is_equal(p_size, result, mp0, 1e-10))
			std::cout << "\"MultiModel::setInputVel\" failed" << std::endl;

		multi.setInputVel(actual_mp);
		multi.getSubInputVel(sub_num1, sub_id1, result);
		if (!aris::dynamic::s_is_equal(result_size, result, sub_mp, 1e-10))
			std::cout << "\"MultiModel::getSubInputVel\" failed" << std::endl;

		multi.setInputAcc(mp0);
		multi.getInputAcc(result);
		if (!aris::dynamic::s_is_equal(p_size, result, mp0, 1e-10))
			std::cout << "\"MultiModel::setInputAcc\" failed" << std::endl;

		multi.setSubInputAcc(sub_num1, sub_id1, sub_mp);
		multi.getInputAcc(result);
		if (!aris::dynamic::s_is_equal(result_size, result, actual_mp, 1e-10))
			std::cout << "\"MultiModel::setSubInputAcc\" failed" << std::endl;

		multi.setInputAcc(mp0);
		multi.getInputAcc(result);
		if (!aris::dynamic::s_is_equal(p_size, result, mp0, 1e-10))
			std::cout << "\"MultiModel::setInputAcc\" failed" << std::endl;

		multi.setInputAcc(actual_mp);
		multi.getSubInputAcc(sub_num1, sub_id1, result);
		if (!aris::dynamic::s_is_equal(result_size, result, sub_mp, 1e-10))
			std::cout << "\"MultiModel::getSubInputAcc\" failed" << std::endl;

		multi.setInputFce(mp0);
		multi.getInputFce(result);
		if (!aris::dynamic::s_is_equal(p_size, result, mp0, 1e-10))
			std::cout << "\"MultiModel::setInputFce\" failed" << std::endl;

		multi.setSubInputFce(sub_num1, sub_id1, sub_mp);
		multi.getInputFce(result);
		if (!aris::dynamic::s_is_equal(result_size, result, actual_mp, 1e-10))
			std::cout << "\"MultiModel::setSubInputFce\" failed" << std::endl;

		multi.setInputFce(mp0);
		multi.getInputFce(result);
		if (!aris::dynamic::s_is_equal(p_size, result, mp0, 1e-10))
			std::cout << "\"MultiModel::setInputFce\" failed" << std::endl;

		multi.setInputFce(actual_mp);
		multi.getSubInputFce(sub_num1, sub_id1, result);
		if (!aris::dynamic::s_is_equal(result_size, result, sub_mp, 1e-10))
			std::cout << "\"MultiModel::getSubInputFce\" failed" << std::endl;

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
		if (!aris::dynamic::s_is_equal(p_size, result, mp0, 1e-10))
			std::cout << "\"MultiModel::setInputPos\" failed" << std::endl;

		multi.setSubInputPos(sub_num1, sub_id1, sub_mp);
		multi.getInputPos(result);
		if (!aris::dynamic::s_is_equal(margin_size + p_size, result, actual_mp, 1e-10))
			std::cout << "\"MultiModel::setSubInputPos\" failed" << std::endl;

		multi.setInputPos(mp0);
		multi.getInputPos(result);
		if (!aris::dynamic::s_is_equal(p_size, result, mp0, 1e-10))
			std::cout << "\"MultiModel::setInputPos\" failed" << std::endl;

		std::fill_n(result, margin_size + sub_p_size, -99);
		multi.setInputPos(actual_mp);
		multi.getSubInputPos(sub_num1, sub_id1, result);
		if (!aris::dynamic::s_is_equal(margin_size + sub_p_size, result, sub_mp, 1e-10))
			std::cout << "\"MultiModel::getSubInputPos\" failed" << std::endl;

		multi.setInputVel(mp0);
		multi.getInputVel(result);
		if (!aris::dynamic::s_is_equal(p_size, result, mp0, 1e-10))
			std::cout << "\"MultiModel::setInputVel\" failed" << std::endl;

		multi.setSubInputVel(sub_num1, sub_id1, sub_mp);
		multi.getInputVel(result);
		if (!aris::dynamic::s_is_equal(margin_size + p_size, result, actual_mp, 1e-10))
			std::cout << "\"MultiModel::setSubInputVel\" failed" << std::endl;

		multi.setInputVel(mp0);
		multi.getInputVel(result);
		if (!aris::dynamic::s_is_equal(p_size, result, mp0, 1e-10))
			std::cout << "\"MultiModel::setInputVel\" failed" << std::endl;

		std::fill_n(result, margin_size + sub_p_size, -99);
		multi.setInputVel(actual_mp);
		multi.getSubInputVel(sub_num1, sub_id1, result);
		if (!aris::dynamic::s_is_equal(margin_size + sub_p_size, result, sub_mp, 1e-10))
			std::cout << "\"MultiModel::getSubInputVel\" failed" << std::endl;

		multi.setInputAcc(mp0);
		multi.getInputAcc(result);
		if (!aris::dynamic::s_is_equal(p_size, result, mp0, 1e-10))
			std::cout << "\"MultiModel::setInputAcc\" failed" << std::endl;

		multi.setSubInputAcc(sub_num1, sub_id1, sub_mp);
		multi.getInputAcc(result);
		if (!aris::dynamic::s_is_equal(margin_size + p_size, result, actual_mp, 1e-10))
			std::cout << "\"MultiModel::setSubInputAcc\" failed" << std::endl;

		multi.setInputAcc(mp0);
		multi.getInputAcc(result);
		if (!aris::dynamic::s_is_equal(p_size, result, mp0, 1e-10))
			std::cout << "\"MultiModel::setInputAcc\" failed" << std::endl;

		std::fill_n(result, margin_size + sub_p_size, -99);
		multi.setInputAcc(actual_mp);
		multi.getSubInputAcc(sub_num1, sub_id1, result);
		if (!aris::dynamic::s_is_equal(margin_size + sub_p_size, result, sub_mp, 1e-10))
			std::cout << "\"MultiModel::getSubInputAcc\" failed" << std::endl;

		multi.setInputFce(mp0);
		multi.getInputFce(result);
		if (!aris::dynamic::s_is_equal(p_size, result, mp0, 1e-10))
			std::cout << "\"MultiModel::setInputFce\" failed" << std::endl;

		multi.setSubInputFce(sub_num1, sub_id1, sub_mp);
		multi.getInputFce(result);
		if (!aris::dynamic::s_is_equal(margin_size + p_size, result, actual_mp, 1e-10))
			std::cout << "\"MultiModel::setSubInputFce\" failed" << std::endl;

		multi.setInputFce(mp0);
		multi.getInputFce(result);
		if (!aris::dynamic::s_is_equal(p_size, result, mp0, 1e-10))
			std::cout << "\"MultiModel::setInputFce\" failed" << std::endl;

		std::fill_n(result, margin_size + sub_p_size, -99);
		multi.setInputFce(actual_mp);
		multi.getSubInputFce(sub_num1, sub_id1, result);
		if (!aris::dynamic::s_is_equal(margin_size + sub_p_size, result, sub_mp, 1e-10))
			std::cout << "\"MultiModel::getSubInputFce\" failed" << std::endl;

	}

	// test getEeNumOfSubModels
	{
		std::vector<aris::Size> result1{ 11,1,2,3,5,0 };

		auto r = multi.getSubEeSize({ 5,3,4,0,2,1 });
		if (result1 != multi.getSubEeSize({ 5,3,4,0,2,1 }))
			std::cout << "\"MultiModel::getSubEeSize\" failed" << std::endl;

		aris::Size result1a[6], input1a[6]{ 5,3,4,0,2,1 };
		multi.getSubEeSize(6, input1a, result1a);
		if (std::vector<aris::Size>(result1a, result1a + 6) != result1)
			std::cout << "\"MultiModel::getSubEeSize\" failed" << std::endl;

		std::vector<aris::Size> result2{ 11,2,5,1,1 };
		if (result2 != multi.getSubEeSize({ 5,4,2,3,3 }))
			std::cout << "\"MultiModel::getSubEeSize\" failed" << std::endl;

		aris::Size result2a[5], input2a[5]{ 5,4,2,3,3 };
		multi.getSubEeSize(5, input2a, result2a);
		if (std::vector<aris::Size>(result2a, result2a + 5) != result2)
			std::cout << "\"MultiModel::getSubEeSize\" failed" << std::endl;
	}

	// test getEeTypes
	{
		//
		// 	ee_types:
		//aris::dynamic::EEType::PE123, aris::dynamic::EEType::PQ, aris::dynamic::EEType::PM,
		//
		//aris::dynamic::EEType::PE123, aris::dynamic::EEType::PQ, aris::dynamic::EEType::PE123, aris::dynamic::EEType::PE123, aris::dynamic::EEType::PM,
		//aris::dynamic::EEType::PQ,
		//aris::dynamic::EEType::PE123, aris::dynamic::EEType::PM


		std::vector<aris::dynamic::EEType> ee_types;
		std::vector<aris::dynamic::EEType> result0{
			aris::dynamic::EEType::PE123, aris::dynamic::EEType::PQ, aris::dynamic::EEType::PM,

			aris::dynamic::EEType::PE123, aris::dynamic::EEType::PQ, aris::dynamic::EEType::PE123, aris::dynamic::EEType::PE123, aris::dynamic::EEType::PM,
			aris::dynamic::EEType::PQ,
			aris::dynamic::EEType::PE123, aris::dynamic::EEType::PM,
			aris::dynamic::EEType::PE123, aris::dynamic::EEType::PQ, aris::dynamic::EEType::PM,

			aris::dynamic::EEType::PE123, aris::dynamic::EEType::PQ, aris::dynamic::EEType::PE123, aris::dynamic::EEType::PE123, aris::dynamic::EEType::PM,
			aris::dynamic::EEType::PQ,
			aris::dynamic::EEType::PE123, aris::dynamic::EEType::PM,
		};

		ee_types = std::vector<aris::dynamic::EEType>(multi.eeTypes(), multi.eeTypes() + multi.eeSize());
		if (ee_types != result0)
			std::cout << "\"MultiModel::eeTypes\" failed" << std::endl;

		//ee_types.clear();
		//ee_types.resize(11, aris::dynamic::EEType::PQ);
		//multi.getEeTypes(ee_types.data());
		//if (ee_types != result0)
		//	std::cout << "\"MultiModel::getEeTypes\" failed" << std::endl;


		std::vector<aris::dynamic::EEType> result1{
			aris::dynamic::EEType::PQ,
			aris::dynamic::EEType::PE123, aris::dynamic::EEType::PM,
			aris::dynamic::EEType::PE123, aris::dynamic::EEType::PQ, aris::dynamic::EEType::PM,
			aris::dynamic::EEType::PE123, aris::dynamic::EEType::PQ, aris::dynamic::EEType::PE123, aris::dynamic::EEType::PE123, aris::dynamic::EEType::PM,
		};
		aris::Size sub1[5]{ 3,4,0,2,1 };

		ee_types = multi.getSubEeTypes({ 3,4,0,2,1 });
		if (ee_types != result1)
			std::cout << "\"MultiModel::getSubEeTypes\" failed" << std::endl;

		ee_types.clear();
		ee_types.resize(11, aris::dynamic::EEType::PQ);
		multi.getSubEeTypes(5, sub1, ee_types.data());
		if (ee_types != result1)
			std::cout << "\"MultiModel::getSubEeTypes\" failed" << std::endl;

		std::vector<aris::dynamic::EEType> result2{
			aris::dynamic::EEType::PE123, aris::dynamic::EEType::PM,
			aris::dynamic::EEType::PE123, aris::dynamic::EEType::PQ, aris::dynamic::EEType::PE123, aris::dynamic::EEType::PE123, aris::dynamic::EEType::PM,
			aris::dynamic::EEType::PQ,
			aris::dynamic::EEType::PQ,
		};
		aris::Size sub2[4]{ 4,2,3,3 };

		ee_types = multi.getSubEeTypes({ 4,2,3,3 });
		if (ee_types != result2)
			std::cout << "\"MultiModel::getSubEeTypes\" failed" << std::endl;

		ee_types.clear();
		ee_types.resize(9, aris::dynamic::EEType::PQ);
		multi.getSubEeTypes(4, sub2, ee_types.data());
		if (ee_types != result2)
			std::cout << "\"MultiModel::getSubEeTypes\" failed" << std::endl;
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

		ees = multi.getEes();
		if (ees != result0)
			std::cout << "\"MultiModel::getEes\" failed" << std::endl;

		ees.clear();
		ees.resize(11, nullptr);
		multi.getEes(ees.data());
		if (ees != result0)
			std::cout << "\"MultiModel::getEes\" failed" << std::endl;


		std::vector<aris::dynamic::MotionBase*> result1{
			&gm31,
			&gm41,&gm42,
			&gm01,&gm02,&gm03,
			&gm21,&gm22,&gm23,&gm24,&gm25,
		};
		aris::Size sub1[5]{ 3,4,0,2,1 };

		ees = multi.getSubEes({ 3,4,0,2,1 });
		if (ees != result1)
			std::cout << "\"MultiModel::getSubEes\" failed" << std::endl;

		ees.clear();
		ees.resize(11, nullptr);
		multi.getSubEes(5, sub1, ees.data());
		if (ees != result1)
			std::cout << "\"MultiModel::getSubEes\" failed" << std::endl;

		std::vector<aris::dynamic::MotionBase*> result2{
			&gm41,&gm42,
			&gm21,&gm22,&gm23,&gm24,&gm25,
			&gm31,
			&gm31,

		};
		aris::Size sub2[4]{ 4,2,3,3 };

		ees = multi.getSubEes({ 4,2,3,3 });
		if (ees != result2)
			std::cout << "\"MultiModel::getSubEes\" failed" << std::endl;

		ees.clear();
		ees.resize(9, nullptr);
		multi.getSubEes(4, sub2, ees.data());
		if (ees != result2)
			std::cout << "\"MultiModel::getSubEes\" failed" << std::endl;
	}

	// test getMotionNumOfSubModels
	{
		// 3 4 0 6 2
		std::vector<aris::Size> result1{ 6,2,3,0,4 };
		if (result1 != multi.getMotionNumOfSubModels({ 3,4,0,2,1 }))
			std::cout << "\"MultiModel::getMotionNumOfSubModels\" failed" << std::endl;

		aris::Size result1a[5], input1a[5]{ 3,4,0,2,1 };
		multi.getMotionNumOfSubModels(5, input1a, result1a);
		if (std::vector<aris::Size>(result1a, result1a + 5) != result1)
			std::cout << "\"MultiModel::getMotionNumOfSubModels\" failed" << std::endl;

		std::vector<aris::Size> result2{ 2,0,6,6 };
		if (result2 != multi.getMotionNumOfSubModels({ 4,2,3,3 }))
			std::cout << "\"MultiModel::getMotionNumOfSubModels\" failed" << std::endl;

		aris::Size result2a[4], input2a[4]{ 4,2,3,3 };
		multi.getMotionNumOfSubModels(4, input2a, result2a);
		if (std::vector<aris::Size>(result2a, result2a + 4) != result2)
			std::cout << "\"MultiModel::getMotionNumOfSubModels\" failed" << std::endl;
	}

	// test getMotionTypes
	{
		std::vector<aris::dynamic::EEType> ee_types;
		std::vector<aris::dynamic::EEType> result0{
			aris::dynamic::EEType::A, aris::dynamic::EEType::X, aris::dynamic::EEType::X,
			aris::dynamic::EEType::X, aris::dynamic::EEType::A, aris::dynamic::EEType::X, aris::dynamic::EEType::X,

			aris::dynamic::EEType::X, aris::dynamic::EEType::A, aris::dynamic::EEType::X, aris::dynamic::EEType::X,aris::dynamic::EEType::X, aris::dynamic::EEType::A,
			aris::dynamic::EEType::X, aris::dynamic::EEType::X,
		};

		ee_types = multi.getMotionTypes();
		if (ee_types != result0)
			std::cout << "\"MultiModel::getMotionTypes\" failed" << std::endl;

		ee_types.clear();
		ee_types.resize(15, aris::dynamic::EEType::PQ);
		multi.getMotionTypes(ee_types.data());
		if (ee_types != result0)
			std::cout << "\"MultiModel::getMotionTypes\" failed" << std::endl;


		std::vector<aris::dynamic::EEType> result1{
			aris::dynamic::EEType::X, aris::dynamic::EEType::A, aris::dynamic::EEType::X, aris::dynamic::EEType::X,aris::dynamic::EEType::X, aris::dynamic::EEType::A,
			aris::dynamic::EEType::X, aris::dynamic::EEType::X,
			aris::dynamic::EEType::A, aris::dynamic::EEType::X, aris::dynamic::EEType::X,
			aris::dynamic::EEType::X, aris::dynamic::EEType::A, aris::dynamic::EEType::X, aris::dynamic::EEType::X,
		};
		aris::Size sub1[5]{ 3,4,0,2,1 };

		ee_types = multi.getMotionTypes({ 3,4,0,2,1 });
		if (ee_types != result1)
			std::cout << "\"MultiModel::getMotionTypes\" failed" << std::endl;

		ee_types.clear();
		ee_types.resize(15, aris::dynamic::EEType::PQ);
		multi.getMotionTypes(5, sub1, ee_types.data());
		if (ee_types != result1)
			std::cout << "\"MultiModel::getMotionTypes\" failed" << std::endl;

		std::vector<aris::dynamic::EEType> result2{
			aris::dynamic::EEType::X, aris::dynamic::EEType::X,
			aris::dynamic::EEType::X, aris::dynamic::EEType::A, aris::dynamic::EEType::X, aris::dynamic::EEType::X,aris::dynamic::EEType::X, aris::dynamic::EEType::A,
			aris::dynamic::EEType::X, aris::dynamic::EEType::A, aris::dynamic::EEType::X, aris::dynamic::EEType::X,aris::dynamic::EEType::X, aris::dynamic::EEType::A,
		};
		aris::Size sub2[4]{ 4,2,3,3 };

		ee_types = multi.getMotionTypes({ 4,2,3,3 });
		if (ee_types != result2)
			std::cout << "\"MultiModel::getMotionTypes\" failed" << std::endl;

		ee_types.clear();
		ee_types.resize(14, aris::dynamic::EEType::PQ);
		multi.getMotionTypes(4, sub2, ee_types.data());
		if (ee_types != result2)
			std::cout << "\"MultiModel::getMotionTypes\" failed" << std::endl;
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

		mots = multi.getMotions();
		if (mots != result0)
			std::cout << "\"MultiModel::getMotions\" failed" << std::endl;

		mots.clear();
		mots.resize(15, nullptr);
		multi.getMotions(mots.data());
		if (mots != result0)
			std::cout << "\"MultiModel::getMotions\" failed" << std::endl;


		std::vector<aris::dynamic::Motion*> result1{
			&mot31,&mot32,&mot33,&mot34,&mot35,&mot36,
			&mot41,&mot42,
			&mot01,&mot02,&mot03,
			&mot11,&mot12,&mot13,&mot14,
		};
		aris::Size sub1[5]{ 3,4,0,2,1 };

		mots = multi.getMotions({ 3,4,0,2,1 });
		if (mots != result1)
			std::cout << "\"MultiModel::getMotions\" failed" << std::endl;

		mots.clear();
		mots.resize(15, nullptr);
		multi.getMotions(5, sub1, mots.data());
		if (mots != result1)
			std::cout << "\"MultiModel::getMotions\" failed" << std::endl;

		std::vector<aris::dynamic::Motion*> result2{
			&mot41,&mot42,
			&mot31,&mot32,&mot33,&mot34,&mot35,&mot36,
			&mot31,&mot32,&mot33,&mot34,&mot35,&mot36,
		};
		aris::Size sub2[4]{ 4,2,3,3 };

		mots = multi.getMotions({ 4,2,3,3 });
		if (mots != result2)
			std::cout << "\"MultiModel::getMotions\" failed" << std::endl;

		mots.clear();
		mots.resize(14, nullptr);
		multi.getMotions(4, sub2, mots.data());
		if (mots != result2)
			std::cout << "\"MultiModel::getMotions\" failed" << std::endl;
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

		mots = multi.getMotionIds();
		if (mots != result0)
			std::cout << "\"MultiModel::getMotionIds\" failed" << std::endl;

		mots.clear();
		mots.resize(15, -1);
		multi.getMotionIds(mots.data());
		if (mots != result0)
			std::cout << "\"MultiModel::getMotionIds\" failed" << std::endl;


		std::vector<aris::Size> result1{
			7,8,9,10,11,12,
			13,14,
			0,1,2,
			3,4,5,6,
		};
		aris::Size sub1[5]{ 3,4,0,2,1 };

		mots = multi.getMotionIds({ 3,4,0,2,1 });
		if (mots != result1)
			std::cout << "\"MultiModel::getMotionIds\" failed" << std::endl;

		mots.clear();
		mots.resize(15, -1);
		multi.getMotionIds(5, sub1, mots.data());
		if (mots != result1)
			std::cout << "\"MultiModel::getMotionIds\" failed" << std::endl;

		std::vector<aris::Size> result2{
			13,14,
			7,8,9,10,11,12,
			7,8,9,10,11,12,
		};
		aris::Size sub2[4]{ 4,2,3,3 };

		mots = multi.getMotionIds({ 4,2,3,3 });
		if (mots != result2)
			std::cout << "\"MultiModel::getMotionIds\" failed" << std::endl;

		mots.clear();
		mots.resize(14, -1);
		multi.getMotionIds(4, sub2, mots.data());
		if (mots != result2)
			std::cout << "\"MultiModel::getMotionIds\" failed" << std::endl;
	}




}

void test_model()
{
	std::cout << std::endl << "-----------------test model---------------------" << std::endl;
	
	test_multi_model();
	test_multi_model2();
	
	std::cout << "-----------------test model finished------------" << std::endl << std::endl;
}

