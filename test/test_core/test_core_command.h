#ifndef TEST_CORE_COMMAND_H_
#define TEST_CORE_COMMAND_H_

void test_command();
void getcommandhelp(aris::core::CommandParser &parser, std::string commandname);
void getrobothelp(aris::core::CommandParser &parser);
std::stringstream getHelpStream(aris::core::CommandParser &parser, std::string commandname);

#endif
