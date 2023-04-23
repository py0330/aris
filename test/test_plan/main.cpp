#include <iostream>
#include <aris/core/core.hpp>
#include <aris/plan/plan.hpp>

#include "test_plan_function.h"
#include "test_plan_path.h"
#include "test_plan_scurve.h"
#include "test_plan_trajectory.h"
#include "test_plan_singular_processor.h"

int main(int argc, char *argv[]){
	//test_singular_processor();
	test_trajectory();
	
	test_scurve();
	test_path();
	
	//test_function();


	std::string program = 
		"0 :var\r\n"
		"1:function aaa\r\n"
		"2:\r\n"
		"3:if\r\n"
		"4:bbb\r\n"
		"5:\r\n"
		"6:endif\r\n"
		"7:endfunction\r\n"
		"10:var\r\n"
		"11:function bbb\r\n"
		"17:endfunction\r\n"
		"20:var\r\n"
		"21:main\r\n"
		"22:\r\n"
		"23:if\r\n"
		"24:\r\n"
		"45:    if\r\n"
		"46:\r\n"
		"47:    else\r\n"
		"48:\r\n"
		"49:    endif\r\n"
		"61:    if\r\n"
		"62:\r\n"
		"63:    elseif\r\n"
		"64:\r\n"
		"65:    endif\r\n"
		"81:    if\r\n"
		"82:    elseif\r\n"
		"83:    elseif\r\n"
		"84:    else\r\n"
		"85:    endif\r\n"
		"101:    if\r\n"
		"102:    aaa\r\n"
		"103:    elseif\r\n"
		"104:    \r\n"
		"105:    elseif\r\n"
		"106:    \r\n"
		"107:    else\r\n"
		"108:    \r\n"
		"109:    endif\r\n"
		"200:\r\n"
		"201:endif\r\n"
		"202:while\r\n"
		"203:\r\n"
		"204:endwhile\r\n"
		"205:aaa\r\n"
		"300:endmain \r\n";

	try 
	{
		aris::core::LanguageParser parser;
		//parser.setProgram(program);
		parser.parseLanguage();

		for (;!parser.isEnd();)
		{
			std::cout <<std::setw(5)<< parser.currentLine() <<":" << parser.currentCmd() << std::endl;
			parser.forward(parser.currentCmd() != "while");
		}
		std::cout << std::setw(5) << parser.currentLine() << ":" << parser.currentCmd() << std::endl;

	}
	catch (std::exception &e)
	{
		std::cout << e.what() << std::endl;
	}

	


	


	std::cout << "test_core finished, press any key to continue" << std::endl;
	std::cin.get();

	return 0;
}