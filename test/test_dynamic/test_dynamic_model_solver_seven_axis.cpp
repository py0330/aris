#include "test_dynamic_model_solver_seven_axis.h"
#include <iostream>
#include <array>
#include <aris/dynamic/dynamic.hpp>
#include <aris/dynamic/seven_axis2.hpp>

#include<type_traits>

using namespace aris::dynamic;

void test_seven_axis_forward_solver()
{

}
void test_seven_axis_inverse_solver()
{
	aris::dynamic::SevenAxisParam param;

	param.d1 = 0.3705;
	param.d3 = 0.330;
	param.d5 = 0.320;
	param.tool0_pe[2] = 0.2205;
	std::cout << "finished***" << std::endl;
	auto m = aris::dynamic::createModelSevenAxis(param);
	std::cout << "finished***" << std::endl;
	m->init();
	std::cout << "finished***" << std::endl;
	double pe[6]{ 0.2 , 0.2 , -0.1 , 0.1 , 0.2 , 2.8 };
	m->generalMotionPool()[0].setMpe(pe, "321");

	for (int i = 0; i < 9; ++i)
	{
		dynamic_cast<aris::dynamic::SevenAxisInverseKinematicSolver&>(m->solverPool()[0]).setWhichRoot(8);
		dynamic_cast<aris::dynamic::SevenAxisInverseKinematicSolver&>(m->solverPool()[0]).setAxisAngle(0.3);
		std::cout << "ret:" << m->solverPool()[0].kinPos() << std::endl;
		m->solverPool()[1].kinPos();

		double result[6];
		m->generalMotionPool()[0].updMpm();
		m->generalMotionPool()[0].getMpe(result, "321");
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

	auto m = aris::dynamic::createModelSevenAxis2(param);
	m->init();

	


	double pe[6]{ 0.2 , 0.2 , -0.1 , 0.1 , 0.2 , 2.8 };

	double input[7]{ 0.1, 0.2, 0.3, -0.8, 0.5, 0.6, 0.7 };
	m->setMotionPos(input);
	m->forwardKinematics();
	m->generalMotionPool()[0].updMpm();
	m->generalMotionPool()[0].getMpe(pe);

	aris::dynamic::dsp(4, 4, *m->generalMotionPool()[0].mpm());
	
	//m->generalMotionPool()[0].setMpe(pe, "321");





	for (int i = 0; i < 9; ++i)
	{
		dynamic_cast<aris::dynamic::SevenAxisInverseKinematicSolver2&>(m->solverPool()[0]).setWhichRoot(i);
		dynamic_cast<aris::dynamic::SevenAxisInverseKinematicSolver2&>(m->solverPool()[0]).setAxisAngle(0.3);
		std::cout << "ret:" << m->solverPool()[0].kinPos() << std::endl;
		//m->solverPool()[1].kinPos();

		double result[6];
		m->generalMotionPool()[0].updMpm();
		m->generalMotionPool()[0].getMpe(result, "321");
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

	std::cout << "-----------------test model solver seven_axis finished------------" << std::endl << std::endl;
}

