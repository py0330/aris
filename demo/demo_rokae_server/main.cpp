#include <aris.hpp>

class MoveCos1 :public aris::plan::Plan
{
public:
	auto virtual executeRT()->int override
	{
		
		
		
		
		return 0;
	}

	explicit MoveCos1()
	{
		command().loadXmlStr(
			"<Command name=\"mv_cos1\">"
			"</Command>"
		);
	}
	ARIS_REGISTER_TYPE(MoveCos1);
};

int main(int argc, char *argv[])
{
	auto &cs = aris::server::ControlServer::instance();

	cs.resetController(aris::robot::createControllerRokaeXB4().release());
	cs.resetModel(aris::robot::createModelRokaeXB4().release());
	cs.resetPlanRoot(aris::robot::createPlanRootRokaeXB4().release());
	cs.interfaceRoot().loadXmlStr(aris::robot::createRokaeXB4Interface());

	return 0;
}