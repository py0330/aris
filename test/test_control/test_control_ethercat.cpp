#include <iostream>
#include <aris.h>

#include "test_control_ethercat.h"

#ifdef UNIX
#include "rtdk.h"
#include "unistd.h"
#endif

using namespace aris::control;

class MyMaster :public aris::control::Master
{
protected:
	virtual auto controlStrategy()->void override
	{
		auto& motion = dynamic_cast<aris::control::Motion &>(slavePool().at(0));

		static int count{ 0 };
#ifdef UNIX
        if (count++ % 100 == 0)rt_printf("%d:%f\n", count, motion.rxData().feedback_pos);
#endif
	};
};

void test_control_ethercat()
{
	MyMaster master;

#ifdef WIN32
	master.loadXml("C:\\Robots\\resource\\Robot_Type_I\\Robot_III\\Robot_III.xml");
#endif
#ifdef UNIX
	master.loadXml("/usr/Robots/resource/Robot_Type_I/Robot_III/Robot_III.xml");
#endif

	master.start();

	char a;
	std::cin >> a;

	master.stop();

	std::cout << "finished test aris::control::master" << std::endl;
}
