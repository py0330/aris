#include <iostream>
#include "test_dynamic_kernel.h"
#include "test_dynamic_model.h"

#include <aris.h>

int main(int argc, char *argv[])
{
	test_kernel();
	test_model();
	//test_model2();
	

	std::cout << "finished" << std::endl;

	char aaaaa;
	std::cin >> aaaaa;

	return 0;
}