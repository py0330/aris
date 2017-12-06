#include "aris.h"

using namespace std;
using namespace aris::dynamic;

int main()
{
	// 
	const double link1_position_and_eula321[6]{ 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 };
	const double link1_inertia_vector[10]{ 2.0 , 0.0 , 0.0 , 0.0 , 1.0 , 1.0, 10.0 , 0.0, 0.0, 0.0 };
	const double link2_position_and_eula321[6]{ 1.0 , 0.0 , 0.0 , PI/2 , 0.0 , 0.0 };
	const double link2_inertia_vector[10]{ 2.0 , 0.0 , 0.0 , 0.0 , 1.0 , 1.0, 10.0 , 0.0, 0.0, 0.0 };
	const double link3_position_and_eula321[6]{ 1.0 , 1.0 , 0.0 , PI , 0.0 , 0.0 };
	const double link3_inertia_vector[10]{ 2.0 , 0.0 , 0.0 , 0.0 , 1.0 , 1.0, 10.0 , 0.0, 0.0, 0.0 };

	const double joint1_position[3]{ 0.0 , 0.0 , 0.0 };
	const double joint1_axis[3]{ 0.0 , 0.0 , 1.0 };
	const double joint2_position[3]{ 1.0 , 0.0 , 0.0 };
	const double joint2_axis[3]{ 0.0 , 0.0 , 1.0 };
	const double joint3_position[3]{ 1.0 , 1.0 , 0.0 };
	const double joint3_axis[3]{ 0.0 , 0.0 , 1.0 };

	const double end_effector_position_and_eula321[6]{ 1.0 , 0.0 , 0.0 , PI , 0.0 , 0.0 };

	Model m;
	auto &link1 = m.addPartByPe(link1_position_and_eula321, "321", link1_inertia_vector);
	auto &link2 = m.addPartByPe(link1_position_and_eula321, "321", link1_inertia_vector);
	auto &link3 = m.addPartByPe(link1_position_and_eula321, "321", link1_inertia_vector);
	
	auto &joint1 = m.addRevoluteJoint(link1, m.ground(), joint1_position, joint1_axis);
	auto &joint2 = m.addRevoluteJoint(link2, link1, joint2_position, joint2_axis);
	auto &joint3 = m.addRevoluteJoint(link3, link2, joint3_position, joint3_axis);
	
	auto &motion1 = m.addMotion(joint1);
	auto &motion2 = m.addMotion(joint2);
	auto &motion3 = m.addMotion(joint3);

	auto &end_effector = m.addGeneralMotionByPe(link3, m.ground(), end_effector_position_and_eula321, "321");



	//auto &adams_simulator = m.simulatorPool().add<AdamsSimulator>();
	//adams_simulator.saveAdams("C:\\Users\\py033\\Desktop\\3R_robot.cmd", );
	
	//Model m;
	//auto &p1 = m.partPool().add<Part>("p1", nullptr, s_pe2pm(std::array<double,6>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}.data()), nullptr, nullptr);
	//auto &p2 = m.partPool().add<Part>("p2", nullptr, s_pe2pm(std::array<double,6>{1.0, 0.0, 0.0, PI/2, 0.0, 0.0}.data()), nullptr, nullptr);
	//auto &p3 = m.partPool().add<Part>("p3", nullptr, s_pe2pm(std::array<double,6>{1.0, 1.0, 0.0, 0.0, 0.0, 0.0}.data()), nullptr, nullptr);

	//auto &r1i = m.ground().markerPool().add<Marker>("r1i", s_pe2pm(std::array<double,6>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}.data()));
	//auto &r1j = p1.markerPool().add<Marker>("r1j", s_pe2pm(std::array<double,6>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}.data()));
	//auto &r2i = p1.markerPool().add<Marker>("r2i", s_pe2pm(std::array<double,6>{1.0, 0.0, 0.0, 0.0, 0.0, 0.0}.data()));
	//auto &r2j = p2.markerPool().add<Marker>("r2j", s_pe2pm(std::array<double,6>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}.data()));
	//auto &r3i = p2.markerPool().add<Marker>("r3i", s_pe2pm(std::array<double,6>{1.0, 0.0, 0.0, 0.0, 0.0, 0.0}.data()));
	//auto &r3j = p3.markerPool().add<Marker>("r3j", s_pe2pm(std::array<double,6>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}.data()));

	//m.jointPool().add<RevoluteJoint>("r1", &r1i, &r1j);
	//m.jointPool().add<RevoluteJoint>("r2", &r2i, &r2j);
	//m.jointPool().add<RevoluteJoint>("r3", &r3i, &r3j);

	//m.motionPool().add<Motion>("m1", &r1i, &r1j, 5);
	//m.motionPool().add<Motion>("m2", &r2i, &r2j, 5);
	//m.motionPool().add<Motion>("m3", &r3i, &r3j, 5);

	//auto &ee = p3.markerPool().add<Marker>("ee", s_pe2pm(std::array<double, 6>{1.0, 0.0, 0.0, 0.0, 0.0, 0.0}.data()));
	//auto &origin = m.ground().markerPool().add<Marker>("origin", s_pe2pm(std::array<double, 6>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}.data()));
	//auto &gm = m.generalMotionPool().add<GeneralMotion>("ge", &ee, &origin);

	//for (auto &mot : m.motionPool())mot.activate(false);
	////m.allocateMemory();

	//gm.setMpe(std::array<double, 6>{ 1.5, 0.6, 0.0, 0.0, 0.0, PI / 4}.data(), "123");
	////auto ret = m.kinPosInGlb(100);
	////std::cout << "computation finished, spend " << std::get<0>(ret) << " count with error " << std::get<1>(ret) << std::endl;





	
	std::cout << "demo_3R finished, press any key to continue" << std::endl;
	std::cin.get();
	return 0;
}

