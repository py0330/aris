#include <iostream>
#include "test_dynamic_kernel.h"
#include "test_dynamic_model.h"

int main(int argc, char *argv[])
{
	test_kernel();
	test_model();

	char a;
	std::cin >> a;

	return 0;
}