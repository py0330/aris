#include <iostream>
#include "test_dynamic_kernel.h"
#include "test_dynamic_model.h"

#include <aris.h>

int main(int argc, char *argv[])
{
	test_kernel();
	//test_model2();
	//test_model();
	
	double re[3] = { 0.1,0.2,0.3 };
	double we[3] = { 0.4,0.5,0.6 };

	double wa[3];

	aris::dynamic::s_we2wa(re, we, wa, "123");

	aris::dynamic::dsp(wa, 3, 1);


	char aaaaa;
	std::cin >> aaaaa;

	return 0;
}