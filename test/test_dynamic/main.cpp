#include <iostream>
#include "test_dynamic_matrix.h"
#include "test_dynamic_block_matrix.h"
#include "test_dynamic_cell.h"
#include "test_dynamic_screw.h"
#include "test_dynamic_spline.h"
#include "test_dynamic_model.h"
#include "test_dynamic_model_coordinate.h"
#include "test_dynamic_model_compute.h"
#include "test_dynamic_model_interaction.h"
#include "test_dynamic_model_multi.h"
#include "test_dynamic_simple_model.h"
#include "test_dynamic_plan.h"

#include <aris_dynamic.h>


int main(int argc, char *argv[])
{
	//test_matrix();
	//test_block_matrix();
	//test_cell();
	//test_screw();
	//test_spline();
	test_plan();
	//test_model();
	//test_model_coordinate();
	//test_model_interaction();
	//test_model_compute();
	//test_model_multi();
	//test_simple_model();
	//

	std::cout << "test_dynamic finished, press any key to continue" << std::endl;
	std::cin.get();
	return 0;
}