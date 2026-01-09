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
		if (result1 != multi.getEeNumOfSubModels({ 3,4,0,2,1 }))
			std::cout << "\"MultiModel::getEeNumOfSubModels\" failed" << std::endl;

		aris::Size result1a[5], input1a[5]{ 3,4,0,2,1 };
		multi.getEeNumOfSubModels(5, input1a, result1a);
		if (std::vector<aris::Size>(result1a, result1a+5) != result1)
			std::cout << "\"MultiModel::getEeNumOfSubModels\" failed" << std::endl;

		std::vector<aris::Size> result2{ 2,5,1,1 };
		if (result2 != multi.getEeNumOfSubModels({ 4,2,3,3 }))
			std::cout << "\"MultiModel::getEeNumOfSubModels\" failed" << std::endl;
		
		aris::Size result2a[4], input2a[4]{ 4,2,3,3 };
		multi.getEeNumOfSubModels(4, input2a, result2a);
		if (std::vector<aris::Size>(result2a, result2a + 4) != result2)
			std::cout << "\"MultiModel::getEeNumOfSubModels\" failed" << std::endl;
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

		ee_types = multi.getEeTypes();
		if (ee_types != result0)
			std::cout << "\"MultiModel::getEeTypes\" failed" << std::endl;

		ee_types.clear();
		ee_types.resize(11, aris::dynamic::EEType::PQ);
		multi.getEeTypes(ee_types.data());
		if (ee_types != result0)
			std::cout << "\"MultiModel::getEeTypes\" failed" << std::endl;


		std::vector<aris::dynamic::EEType> result1{
			aris::dynamic::EEType::PQ,
			aris::dynamic::EEType::PE123, aris::dynamic::EEType::PM,
			aris::dynamic::EEType::PE123, aris::dynamic::EEType::PQ, aris::dynamic::EEType::PM,
			aris::dynamic::EEType::PE123, aris::dynamic::EEType::PQ, aris::dynamic::EEType::PE123, aris::dynamic::EEType::PE123, aris::dynamic::EEType::PM,
		};
		aris::Size sub1[5]{ 3,4,0,2,1 };

		ee_types = multi.getEeTypes({ 3,4,0,2,1 });
		if (ee_types != result1)
			std::cout << "\"MultiModel::getEeTypes\" failed" << std::endl;

		ee_types.clear();
		ee_types.resize(11, aris::dynamic::EEType::PQ);
		multi.getEeTypes(5, sub1, ee_types.data());
		if (ee_types != result1)
			std::cout << "\"MultiModel::getEeTypes\" failed" << std::endl;

		std::vector<aris::dynamic::EEType> result2{
			aris::dynamic::EEType::PE123, aris::dynamic::EEType::PM,
			aris::dynamic::EEType::PE123, aris::dynamic::EEType::PQ, aris::dynamic::EEType::PE123, aris::dynamic::EEType::PE123, aris::dynamic::EEType::PM,
			aris::dynamic::EEType::PQ, 
			aris::dynamic::EEType::PQ,
		};
		aris::Size sub2[4]{ 4,2,3,3 };

		ee_types = multi.getEeTypes({ 4,2,3,3 });
		if (ee_types != result2)
			std::cout << "\"MultiModel::getEeTypes\" failed" << std::endl;

		ee_types.clear();
		ee_types.resize(9, aris::dynamic::EEType::PQ);
		multi.getEeTypes(4, sub2, ee_types.data());
		if (ee_types != result2)
			std::cout << "\"MultiModel::getEeTypes\" failed" << std::endl;
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

		ees = multi.getEes({ 3,4,0,2,1 });
		if (ees != result1)
			std::cout << "\"MultiModel::getEes\" failed" << std::endl;

		ees.clear();
		ees.resize(11, nullptr);
		multi.getEes(5, sub1, ees.data());
		if (ees != result1)
			std::cout << "\"MultiModel::getEes\" failed" << std::endl;

		std::vector<aris::dynamic::MotionBase*> result2{
			&gm41,& gm42,
			& gm21,& gm22,& gm23,& gm24,& gm25,
			& gm31,
			& gm31,

		};
		aris::Size sub2[4]{ 4,2,3,3 };

		ees = multi.getEes({ 4,2,3,3 });
		if (ees != result2)
			std::cout << "\"MultiModel::getEes\" failed" << std::endl;

		ees.clear();
		ees.resize(9, nullptr);
		multi.getEes(4, sub2, ees.data());
		if (ees != result2)
			std::cout << "\"MultiModel::getEes\" failed" << std::endl;
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



void test_model()
{
	std::cout << std::endl << "-----------------test model---------------------" << std::endl;
	
	test_multi_model();
	
	std::cout << "-----------------test model finished------------" << std::endl << std::endl;
}

