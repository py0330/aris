#include <iostream>
#include <aris.hpp>

// 示例轨迹规划 //
class MyPlan :public aris::plan::Plan
{
public:
	auto virtual prepairNrt()->void override
	{
	}
	auto virtual executeRT()->int override
	{
		return 0;
	}
	auto virtual collectNrt()->void override 
	{
	}

	explicit MyPlan(const std::string &name = "my_plan")
	{
		command().loadXmlStr(
			"<Command name=\"my_plan\">"
			"</Command>"
		);
	}
	ARIS_REGISTER_TYPE(MyPlan);
};

int main(int argc, char *argv[])
{

	return 0;
}