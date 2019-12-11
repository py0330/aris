#include <iostream>
#include <aris/core/core.hpp>
#include <aris/plan/plan.hpp>

#include "test_plan_function.h"

int main(int argc, char *argv[])
{
	//test_function();

	std::map<int, aris::plan::CmdInfo> cmd_map;

	cmd_map[10] = { "main",0,0 };
	cmd_map[11] = { "if",0,0 };
	cmd_map[12] = { "",0,0 };
	cmd_map[13] = { "",0,0 };
	cmd_map[14] = { "",0,0 };
	cmd_map[15] = { "endif",0,0 };
	cmd_map[21] = { "if",0,0 };
	cmd_map[22] = { "endif",0,0 };
	cmd_map[23] = { "if",0,0 };
	cmd_map[24] = { "else",0,0 };
	cmd_map[25] = { "",0,0 };
	cmd_map[26] = { "endif",0,0 };
	cmd_map[41] = { "if",0,0 };
	cmd_map[42] = { "",0,0 };
	cmd_map[43] = { "",0,0 };
	cmd_map[51] = { "",0,0 };
	cmd_map[52] = { "if",0,0 };
	cmd_map[53] = { "",0,0 };
	cmd_map[54] = { "elseif",0,0 };
	cmd_map[55] = { "",0,0 };
	cmd_map[56] = { "elseif",0,0 };
	cmd_map[57] = { "",0,0 };
	cmd_map[58] = { "elseif",0,0 };
	cmd_map[59] = { "",0,0 };
	cmd_map[60] = { "endif",0,0 };
	cmd_map[70] = { "elseif",0,0 };
	cmd_map[71] = { "elseif",0,0 };
	cmd_map[80] = { "if",0,0 };
	cmd_map[81] = { "",0,0 };
	cmd_map[82] = { "",0,0 };
	cmd_map[90] = { "else",0,0 };
	cmd_map[91] = { "",0,0 };
	cmd_map[92] = { "",0,0 };
	cmd_map[95] = { "endif",0,0 };
	cmd_map[130] = { "while",0,0 };
	cmd_map[131] = { "",0,0 };
	cmd_map[132] = { "",0,0 };
	cmd_map[134] = { "endwhile",0,0 };
	cmd_map[180] = { "if",0,0 };
	cmd_map[181] = { "",0,0 };
	cmd_map[182] = { "",0,0 };
	cmd_map[190] = { "else",0,0 };
	cmd_map[191] = { "sfds",0,0 };
	cmd_map[192] = { "",0,0 };
	cmd_map[195] = { "endif",0,0 };
	cmd_map[200] = { "endif",0,0 };
	cmd_map[223] = { "if",0,0 };
	cmd_map[224] = { "else",0,0 };
	cmd_map[226] = { "endif",0,0 };
	cmd_map[234] = { "",0,0 };
	cmd_map[235] = { "",0,0 };

	try 
	{
		aris::plan::LanguageParser parser;
		parser.parseLanguage(cmd_map);
	}
	catch (std::exception &e)
	{
		std::cout << e.what() << std::endl;
	}

	


	for (auto &cmd : cmd_map)
	{
		std::cout << std::setw(4) << cmd.first << " : " << std::setw(15) << cmd.second.cmd << " | " << std::setw(5) << cmd.second.next_cmd_true_ << " | " << cmd.second.next_cmd_false_ << std::endl;
	}



	std::cout << "test_core finished, press any key to continue" << std::endl;
	std::cin.get();

	return 0;
}