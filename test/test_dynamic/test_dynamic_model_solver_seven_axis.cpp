#include "test_dynamic_model_solver_seven_axis.h"
#include <iostream>
#include <array>
#include <aris/dynamic/dynamic.hpp>
#include <aris/core/core.hpp>

#include<type_traits>

using namespace aris::dynamic;

void test_seven_axis_forward_solver(){

}

auto createModelStandardSevenAxis()->std::unique_ptr<aris::dynamic::MultiModel> {
	std::unique_ptr<aris::dynamic::MultiModel>model(new aris::dynamic::MultiModel);

	aris::dynamic::SevenAxisParam param;
	param.d1 = 0.3705;
	param.d3 = 0.330;
	param.d5 = 0.320;
	param.tool0_pe[2] = 0.2205;
	model->subModels().push_back(aris::dynamic::createModelSevenAxis(param).release());

	for (aris::Size i = 0; i < 17; i++)
		model->tools().push_back(model->findMarker("StandardSevenAxis.EE.tool" + std::to_string(i)));

	for (aris::Size i = 0; i < 33; i++)
		model->wobjs().push_back(model->findMarker("StandardSevenAxis.ground.wobj" + std::to_string(i)));

	return model;
}

void test_seven_axis_inverse_solver(){
	aris::dynamic::SevenAxisParam param;

	param.d1 = 0.3705;
	param.d3 = 0.330;
	param.d5 = 0.320;
	param.tool0_pe[2] = 0.2205;

	auto m = aris::dynamic::createModelSevenAxis(param);
	
	//std::cout << aris::core::toXmlString(*m) << std::endl;

	double output[7]{ 0.2 , 0.2 , -0.1 , 0.1 , 0.2 , 2.8, 0.31 }; // pe321 & arm_angle
	double result[7];

	// 直接调用函数设置末端，之后计算反解 //
	m->setOutputPos(output);
	m->inverseKinematics();
	m->getInputPos(result);
	aris::dynamic::dsp(1, 7, result);

	// 使用ee设置末端，之后计算反解 //
	m->generalMotionPool()[0].setP(output);  // 末端
	m->generalMotionPool()[1].setP(output + 6);  // 臂角
	m->inverseKinematics();
	m->getInputPos(result);
	aris::dynamic::dsp(1, 7, result);
	
	auto &gm = dynamic_cast<aris::dynamic::GeneralMotion&>(m->generalMotionPool()[0]);
	m->init();
	
	m->setOutputPos(output);

	for (int i = 0; i < 9; ++i)	{
		double result[7], zeros[7]{0,0,0,0,0,0,0}, input[7];

		m->setOutputPos(output);
		m->inverseKinematics();
		m->getInputPos(input);
		dsp(1, 7, input);

		m->setInputPos(zeros);
		m->forwardKinematics();
		m->getOutputPos(result);
		dsp(1, 7, result);

		m->setInputPos(input);
		m->forwardKinematics();
		m->getOutputPos(result);
		dsp(1, 7, result);

		std::cout << std::endl << "---------------------" << std::endl;
	}
	
}
void test_seven_axis_inverse_solver2()
{
	aris::dynamic::SevenAxisParam2 param;

	param.d1 = 0;
	param.a2 = 0.1;
	param.d3 = 0.330;
	param.d5 = 0.320;
	param.tool0_pe[2] = 0;


	//param.d1 = 0.203;
	//param.a2 = -0.138;
	//param.d3 = 0.450;
	//param.d5 = 0.300;
	//param.tool0_pe[2] = 0.1048;

	auto m = aris::dynamic::createModelSevenAxis2(param);
	auto &gm = dynamic_cast<aris::dynamic::GeneralMotion&>(m->generalMotionPool()[0]);
	auto& arm_mot = dynamic_cast<aris::dynamic::Motion&>(m->generalMotionPool()[1]);
	m->init();

	//double pe1[]{ 0.4707, 0.206, 0.6817, 3.49, 0.8636, 3.3637 };
	//double pe2[]{ 0.027, 0.0217, 0, 0.7546, -0.92, -0.623 };
	//double pe3[]{ 0.501, 0.0252, 0.8176, 1.0, 0.0468, 0.845 };


	double pe[6]{ 0.2 , 0.2 , -0.1 , 0.1 , 0.2 , 2.8 };

	double input[7]{ 0.1, 0.2, 0.3, -0.8, 0.5, 0.6, 0.7 };
	m->setInputPos(input);
	m->forwardKinematics();
	gm.updP();
	gm.getMpe(pe);

	std::cout << "init pe:";
	aris::dynamic::dsp(1, 6, pe);
	
	//m->generalMotionPool()[0].setMpe(pe, "321");





	for (int i = 0; i < 9; ++i)
	{
		dynamic_cast<aris::dynamic::SevenAxisInverseKinematicSolver2&>(m->solverPool()[0]).setWhichRoot(i);
		arm_mot.setMp(0.1*i);

		double result[7];

		std::cout << "inverse ret:" << m->inverseKinematics() << std::endl;
		m->getInputPos(result);
		dsp(1, 7, result);

		std::cout << "forward ret:" << m->forwardKinematics() << std::endl;
		m->getOutputPos(result);
		dsp(1, 7, result);

		std::cout << std::endl << "---------------------" << std::endl;
	}

}
void test_seven_axis_inverse_solver3()
{
	aris::dynamic::SevenAxisParam3 param;

	param.d1 = 0.7;
	param.d2 = 0.05;
	param.d3 = 0.05;
	param.d5 = 0.1;
	param.tool0_pe[2] = 0.04;


	//param.d1 = 0.203;
	//param.a2 = -0.138;
	//param.d3 = 0.450;
	//param.d5 = 0.300;
	//param.tool0_pe[2] = 0.1048;

	auto m = aris::dynamic::createModelSevenAxis3(param);
	auto &gm = dynamic_cast<aris::dynamic::GeneralMotion&>(m->generalMotionPool()[0]);
	m->init();

	//double pe1[]{ 0.4707, 0.206, 0.6817, 3.49, 0.8636, 3.3637 };
	//double pe2[]{ 0.027, 0.0217, 0, 0.7546, -0.92, -0.623 };
	//double pe3[]{ 0.501, 0.0252, 0.8176, 1.0, 0.0468, 0.845 };


	double pe[6]{ 0.2 , 0.2 , -0.1 , 0.1 , 0.2 , 2.8 };

	double input[7]{ 0.1, 0.2, 0.1, 0.4, 0.5, 0.6, 0.7 };
	m->setInputPos(input);
	m->forwardKinematics();
	gm.updP();
	gm.getMpe(pe, "321");


	aris::dynamic::dsp(1, 6, pe);
	//aris::dynamic::dsp(4, 4, *m->generalMotionPool()[0].mpm());

	//m->generalMotionPool()[0].setMpe(pe, "321");





	for (int i = 0; i < 9; ++i)
	{
		dynamic_cast<aris::dynamic::SevenAxisInverseKinematicSolver3&>(m->solverPool()[0]).setWhichRoot(i);
		//dynamic_cast<aris::dynamic::SevenAxisInverseKinematicSolver3&>(m->solverPool()[0]).setAxisAngle(0.3);
		std::cout << "ret:" << m->solverPool()[0].kinPos() << std::endl;
		//m->solverPool()[1].kinPos();

		double result[6];
		gm.updP();
		gm.getMpe(result, "321");
		dsp(1, 6, result);




		for (int j = 0; j < 7; ++j)
		{
			std::cout << m->motionPool()[j].mp() << "  ";
		}

		std::cout << std::endl << "---------------------" << std::endl;
	}

}
void test_model_solver_seven_axis()
{
	std::cout << std::endl << "-----------------test model solver seven_axis---------------------" << std::endl;
	test_seven_axis_inverse_solver2();
	//test_seven_axis_inverse_solver3();


	auto m = createModelStandardSevenAxis();
	double output[7]{ 0.2 , 0.2 , -0.1 , 0.1 , 0.2 , 2.8, 0.31 }; // pe321 & arm_angle
	double result[7];

	// 直接调用函数设置末端，之后计算反解 //
	m->setOutputPos(output);
	m->inverseKinematics();
	m->getInputPos(result);
	aris::dynamic::dsp(1, 7, result);

	try {
		//std::cout << aris::core::toXmlString(*m) << std::endl;
		auto str = aris::core::toXmlString(*m);


		aris::dynamic::MultiModel m2;
		aris::core::fromXmlString(m2, str);
		m2.init();
		//std::cout << aris::core::toXmlString(m2) << std::endl;

	}
	catch (std::exception& e)
	{
		std::cout << e.what() << std::endl;
	}
	

	std::cout << "-----------------test model solver seven_axis finished------------" << std::endl << std::endl;
}

