#include <iostream>
#include "test_dynamic_matrix.h"
#include "test_dynamic_screw.h"
#include "test_dynamic_spline.h"
#include "test_dynamic_model.h"
#include "test_dynamic_model_coordinate.h"
#include "test_dynamic_model_solver.h"
#include "test_dynamic_model_solver_puma.h"
#include "test_dynamic_model_solver_seven_axis.h"
#include "test_dynamic_model_solver_stewart.h"
#include "test_dynamic_model_solver_universal.h"
#include "test_dynamic_model_interaction.h"
#include "test_dynamic_plan.h"

#include <aris/dynamic/dynamic.hpp>

int main(int argc, char *argv[])
{
	test_matrix();
	test_screw();
	test_spline();
	test_plan();
	test_model();
	test_model_coordinate();
	test_model_interaction();
	//test_model_solver();
	//test_model_solver_universal();
	//test_model_solver_puma();
	test_model_solver_seven_axis();
	test_model_solver_stewart();

	std::cout << "test_dynamic finished, press any key to continue" << std::endl;
	std::cin.get();
	return 0;
}