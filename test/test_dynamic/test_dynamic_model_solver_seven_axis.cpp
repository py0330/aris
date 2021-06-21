#include "test_dynamic_model_solver_seven_axis.h"
#include <iostream>
#include <array>
#include <aris/dynamic/dynamic.hpp>

#include<type_traits>

using namespace aris::dynamic;

void test_seven_axis_forward_solver()
{

}
void test_seven_axis_inverse_solver(){
	aris::dynamic::SevenAxisParam param;

	param.d1 = 0.3705;
	param.d3 = 0.330;
	param.d5 = 0.320;
	param.tool0_pe[2] = 0.2205;
	std::cout << "finished***" << std::endl;
	auto m = aris::dynamic::createModelSevenAxis(param);
	auto &gm = dynamic_cast<aris::dynamic::GeneralMotion&>(m->generalMotionPool()[0]);
	std::cout << "finished***" << std::endl;
	m->init();
	std::cout << "finished***" << std::endl;
	double pe[6]{ 0.2 , 0.2 , -0.1 , 0.1 , 0.2 , 2.8 };
	gm.setMpe(pe, "321");

	for (int i = 0; i < 9; ++i)
	{
		dynamic_cast<aris::dynamic::SevenAxisInverseKinematicSolver&>(m->solverPool()[0]).setWhichRoot(8);
		dynamic_cast<aris::dynamic::SevenAxisInverseKinematicSolver&>(m->solverPool()[0]).setAxisAngle(0.3);
		std::cout << "ret:" << m->solverPool()[0].kinPos() << std::endl;
		m->solverPool()[1].kinPos();

		double result[6];
		gm.updP();
		gm.getMpe(result, "321");
		dsp(1, 6, result);

		/////////////////////////////////////////////////////////////////////
		double D[3];
		s_vc(3, (*m->jointPool().back().makI()->pm()) + 3, 4, D, 1);
		//dsp(1, 3, D);
		//dsp(1, 3, D);
		
		double z_Axis[3]{ 0,0,1 };
		double r2[3];
		s_c3(z_Axis, D, r2);

		s_nv(3, 1.0 / s_norm(3, r2), r2);

		std::cout << "axis angle:" << std::endl;;

		aris::dynamic::dsp(1, 3, r2);
		aris::dynamic::dsp(4, 4, *m->partPool().at(3).pm());

		double dir[3];
		s_c3(r2, 1, *m->partPool().at(3).pm() + 1, 4, dir, 1);

		std::cout << s_sgn(s_vv(3, dir, D)) * std::acos(std::max(-1.0, std::min(1.0, s_vv(3, r2, 1, *m->partPool().at(3).pm()+1,4)))) << std::endl;


		//::cout << m->partPool().at(3).name() << std::endl;
		/////////////////////////////////////////////////////////////////////



		for (int j = 0; j < 7; ++j)
		{
			std::cout << m->motionPool()[j].mp() << "  ";
		}

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

	aris::dynamic::dsp(4, 4, *gm.mpm());
	
	//m->generalMotionPool()[0].setMpe(pe, "321");





	for (int i = 0; i < 9; ++i)
	{
		dynamic_cast<aris::dynamic::SevenAxisInverseKinematicSolver2&>(m->solverPool()[0]).setWhichRoot(i);
		dynamic_cast<aris::dynamic::SevenAxisInverseKinematicSolver2&>(m->solverPool()[0]).setAxisAngle(0.3);
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

	test_seven_axis_inverse_solver3();

	std::cout << "-----------------test model solver seven_axis finished------------" << std::endl << std::endl;
}

