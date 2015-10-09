#include "Platform.h"
#include <Aris_Device.h>

#ifdef PLATFORM_IS_LINUX
#include "ecrt.h"
#endif

int main()
{
	Aris::Core::DOCUMENT doc;
#ifdef PLATFORM_IS_WINDOWS
	doc.LoadFile("C:\\Robots\\resource\\Robot_Type_I\\Robot_VIII.xml");
#endif
#ifdef PLATFORM_IS_LINUX
	doc.LoadFile("/usr/Robots/resource/Robot_Type_I/Robot_VIII.xml");
#endif

	auto ele = doc.RootElement()->FirstChildElement("Server")
		->FirstChildElement("Control")->FirstChildElement("EtherCat");

	auto pMas = Aris::Control::ETHERCAT_MASTER::GetInstance();
	pMas->LoadXml(ele);

	return 0;
}
