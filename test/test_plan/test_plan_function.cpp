#include <iostream>
#include <aris_core.h>
#include <aris_plan.h>

#include "test_plan_command.h"

using namespace aris::plan;

void test_plan_root()
{
	PlanRoot root;
	
	root.planPool().add<Plan>("en").command().loadXmlStr(
		"		<en default_child_type=\"Param\" default=\"all\">"
		"			<all abbreviation=\"a\"/>"
		"			<first abbreviation=\"f\"/>"
		"			<second abbreviation=\"s\"/>"
		"			<motion_id abbreviation=\"m\" default=\"0\"/>"
		"			<physical_id abbreviation=\"p\" default=\"0\"/>"
		"			<leg abbreviation=\"l\" default=\"0\"/>"
		"		</en>");

	root.planPool().add<Plan>("ds").command().loadXmlStr(
		"		<ds default_child_type=\"Param\" default=\"all\">"
		"			<all abbreviation=\"a\"/>"
		"			<first abbreviation=\"f\"/>"
		"			<second abbreviation=\"s\"/>"
		"			<motion_id abbreviation=\"m\" default=\"0\"/>"
		"			<physical_id abbreviation=\"p\" default=\"0\"/>"
		"			<leg abbreviation=\"l\" default=\"0\"/>"
		"		</ds>");


	//aris::dynamic::Model model;
	simulateCommand("ds -m=1", &root, nullptr);

}

void test_function()
{
	std::cout << std::endl << "-----------------test function---------------------" << std::endl;
	test_plan_root();
	std::cout << "-----------------test function finished------------" << std::endl << std::endl;
}

