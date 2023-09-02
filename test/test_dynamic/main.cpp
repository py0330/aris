#include <iostream>
#include "test_dynamic_matrix.h"
#include "test_dynamic_screw.h"
#include "test_dynamic_spline.h"
#include "test_dynamic_model.h"
#include "test_dynamic_model_coordinate.h"
#include "test_dynamic_model_solver.h"
#include "test_dynamic_model_solver_scara.h"
#include "test_dynamic_model_solver_puma.h"
#include "test_dynamic_model_solver_ur.h"
#include "test_dynamic_model_solver_delta.h"
#include "test_dynamic_model_solver_seven_axis.h"
#include "test_dynamic_model_solver_stewart.h"
#include "test_dynamic_model_solver_universal.h"
#include "test_dynamic_model_solver_abenics.h"
#include "test_dynamic_model_solver_wafer_machine.h"
#include "test_dynamic_model_interaction.h"
#include "test_dynamic_plan.h"

#include <aris/dynamic/dynamic.hpp>

int main(int argc, char *argv[])
{
	std::cout << aris::dynamic::s_put_near_value(-4, 3, 3) << std::endl;
	std::cout << aris::dynamic::s_put_near_value(-5, 3, 3) << std::endl;
	std::cout << aris::dynamic::s_put_near_value(4, -3, 3) << std::endl;
	std::cout << aris::dynamic::s_put_near_value(5, -3, 3) << std::endl;

	aris::dynamic::PumaParam param;
	param.a1 = 0.1;
	param.d1 = 0.1;
	param.a2 = 0.3;
	param.a3 = 0.3;
	param.d3 = 0.1;
	param.d4 = 0.1;

	auto m = aris::dynamic::createModelPuma(param);

	m->forwardKinematics();

	double output_pos[6], input_pos[6];
	m->getOutputPos(output_pos);
	output_pos[0] += 0.01;
	output_pos[4] += 0.8;

	m->inverseKinematics(output_pos, input_pos, 8);


	//test_matrix();
	//test_screw();
	//test_spline();
	//test_plan();
	//test_model();
	//test_model_coordinate();
	//test_model_interaction();
	//test_model_solver();
	//test_model_solver_universal();
	//test_model_solver_scara();
	test_model_solver_puma();
	//test_model_solver_ur();
	//test_model_solver_delta();
	//test_model_solver_seven_axis();
	//test_model_solver_stewart();
	//test_model_solver_abenics();
	//test_model_solver_wafer_machine();

	std::cout << "test_dynamic finished, press any key to continue" << std::endl;
	std::cin.get();
	return 0;
}