/// \example demo_core_structure_cpp/main.cpp
/// 本例子展示构造数据结构的过程:
///

#include <iostream>
#include "aris.h"

class Family :public aris::core::Root 
{

};



int main()
{

	Family family;



	std::cout << "demo_core_structure_cpp finished, press any key to continue" << std::endl;
	std::cin.get();
	return 0;
}

