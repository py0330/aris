#include <iostream>
#include <aris/core/core.hpp>
#include <aris/plan/plan.hpp>

#include "test_plan_function.h"
#include "test_plan_path.h"
#include "test_plan_scurve.h"
#include "test_plan_trajectory.h"
#include "test_plan_singular_processor.h"
#include "test_plan_move_follower.h"

int main(int argc, char *argv[]){
	aris::plan::SCurveParam param;

	//param.T_ = 0.025613027073041257;
	//param.Ta_ = 0;
	//param.Tb_ = 0;
	//param.a_ = 10;
	//param.j_ = 1000;
	//param.mode_ = 1;

	//param.va_ = 0.15613027073041255;
	//param.va_below_ = 0.15613027073041255;
	//param.va_upper_ = 0.15613027073041255;
	//param.vb_ = 0;
	//param.vb_below_ = 0;
	//param.vb_max_ = 0;
	//param.vb_upper_ = 0;
	//param.vc_ = 0.07806513536520628;
	//param.vc_max_ = 0.5;

	//param.pa_ = 0.04300274538003992;
	//param.pb_ = 0.04500222980560958;
	//param.t0_ = 0.1376311587441232;

	param.T_ = 0.025613027073041257;
	param.Ta_ = 0;
	param.Tb_ = 0;
	param.a_ = 10;
	param.j_ = 1000;
	param.mode_ = 1;

	param.va_ = 0.12;
	param.va_below_ = 0.12;
	param.va_upper_ = 0.12;
	param.vb_ = 0;
	param.vb_below_ = 0;
	param.vb_max_ = 0;
	param.vb_upper_ = 0;
	param.vc_ = 0.07806513536520628;
	param.vc_max_ = 0.5;

	param.pa_ = 0.0;
	param.pb_ = 0.0012;
	param.t0_ = 0.0;

	//auto T_upper = aris::plan::s_scurve_cpt_T_upper(param);
	//auto T_below = aris::plan::s_scurve_cpt_T_below(param);
	
	//test_move_follower();
	//test_singular_processor();
	test_trajectory();
	

	//test_scurve();
	//test_path();
	
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