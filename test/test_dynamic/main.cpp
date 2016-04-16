#include <iostream>
#include "test_dynamic_kernel.h"
#include "test_dynamic_model.h"

int main(int argc, char *argv[])
{
	test_kernel();
	test_model2();
	test_model();
	
	char aaaaa;
	std::cin >> aaaaa;

	return 0;
}