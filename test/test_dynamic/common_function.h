#include <iostream>
#include <aris/dynamic/dynamic.hpp>



auto test_model_kinematics_pos(aris::dynamic::ModelBase& m, int linspace_num, const double* range_below, const double* range_upper, double error = 1e-10)->int;