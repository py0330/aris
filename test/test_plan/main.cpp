#include <iostream>
#include <aris/core/core.hpp>
#include <aris/plan/plan.hpp>

#include "test_plan_function.h"
#include "test_plan_path.h"
#include "test_plan_scurve.h"
#include "test_plan_trajectory.h"
#include "test_plan_input_smoother.h"
#include "test_plan_move_follower.h"
#include "test_plan_multimodel_planner.h"

int main(int argc, char *argv[]){
	//test_move_follower();
	//test_function();
	test_input_smoother();
	//test_multimodel_planner();
	//test_scurve();
	//test_path();
	//test_trajectory();
	//test_time_optimal();

	std::cout << "test_core finished, press any key to continue" << std::endl;
	std::cin.get();

	return 0;
}