#include <iostream>
#include "test_dynamic_kernel.h"
#include "test_dynamic_model.h"
#include "test_dynamic_model_stewart.h"

#include <aris.h>

#include <type_traits>

int main(int argc, char *argv[])
{
	test_kernel();
	test_model();
	test_model_stewart();
	std::cout << "test_dynamic finished, press any key to continue" << std::endl;
	std::cin.get();

	return 0;
}