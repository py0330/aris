#include "test_dynamic_screw.h"
#include <iostream>
#include <aris/dynamic/dynamic.hpp>

using namespace aris::dynamic;

const double error = 1e-10;


void test_optimal()
{
	//OptimalTrajectory planner;

	//planner.setBeginNode(OptimalTrajectory::Node{ 0,0,0,0 });
	//planner.setEndNode(OptimalTrajectory::Node{ 0,1,0,0 });
	//planner.run();

	//std::ofstream file;

	//file.open("C:\\Users\\py033\\Desktop\\test.txt");

	//file << std::setprecision(15);

	//for (auto &p : planner.list)
	//{
	//	file << p.s << "   " << p.ds << "   " << p.dds << std::endl;
	//}
}

void test_plan()
{
	std::cout << std::endl << "-----------------test plan--------------------" << std::endl;

	test_optimal();

	std::cout << "-----------------test plan finished-----------" << std::endl << std::endl;
}