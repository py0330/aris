#include <iostream>
#include <aris.h>

using namespace aris::dynamic;

int main()
{
	std::cout << std::endl << "-----------------test model stewart---------------------" << std::endl;

	auto&cs = aris::server::ControlServer::instance();

	cs.resetController(aris::robot::createControllerStewart().release());
	cs.resetModel(aris::robot::createModelStewart().release());
	cs.resetPlanRoot(aris::robot::createPlanRootStewart().release());
	cs.resetSensorRoot(new aris::sensor::SensorRoot);

	auto &m = cs.model();
	auto &inv = m.solverPool()[0];
	auto &fwd = m.solverPool()[1];
	auto &ee = m.generalMotionPool()[0];


	double mid_pe[6]{ 0, 0.515, 0.012, 0, 0, 0 };

	double x_max{ 0.04 }, y_max{ 0.04 }, z_max{ 0.04 }, a_max{ 0.085 }, b_max{ 0.0 }, c_max{0.085};

	std::vector < std::array<double, 6> > points;

	// 0 minus //
	points.push_back(std::array<double, 6>{x_max, y_max, z_max, a_max, 0.0, c_max });
	
	// 1 minus //
	points.push_back(std::array<double, 6>{-x_max, y_max, z_max, a_max, 0.0, c_max });
	points.push_back(std::array<double, 6>{x_max, -y_max, z_max, a_max, 0.0, c_max });
	points.push_back(std::array<double, 6>{x_max, y_max, -z_max, a_max, 0.0, c_max });
	points.push_back(std::array<double, 6>{x_max, y_max, z_max, -a_max, 0.0, c_max });
	points.push_back(std::array<double, 6>{x_max, y_max, z_max, a_max, 0.0, -c_max });

	// 2 minus //
	points.push_back(std::array<double, 6>{-x_max, -y_max, z_max, a_max, 0.0, c_max });
	points.push_back(std::array<double, 6>{-x_max, y_max, -z_max, a_max, 0.0, c_max });
	points.push_back(std::array<double, 6>{-x_max, y_max, z_max, -a_max, 0.0, c_max });
	points.push_back(std::array<double, 6>{-x_max, y_max, z_max, a_max, 0.0, -c_max });

	points.push_back(std::array<double, 6>{x_max, -y_max, -z_max, a_max, 0.0, c_max });
	points.push_back(std::array<double, 6>{x_max, -y_max, z_max, -a_max, 0.0, c_max });
	points.push_back(std::array<double, 6>{x_max, -y_max, z_max, a_max, 0.0, -c_max });

	points.push_back(std::array<double, 6>{x_max, y_max, -z_max, -a_max, 0.0, c_max });
	points.push_back(std::array<double, 6>{x_max, y_max, -z_max, a_max, 0.0, -c_max });

	points.push_back(std::array<double, 6>{x_max, y_max, z_max, -a_max, 0.0, -c_max });


	// 2 positive //
	points.push_back(std::array<double, 6>{x_max, y_max, -z_max, -a_max, 0.0, -c_max });
	points.push_back(std::array<double, 6>{x_max, -y_max, z_max, -a_max, 0.0, -c_max });
	points.push_back(std::array<double, 6>{x_max, -y_max, -z_max, a_max, 0.0, -c_max });
	points.push_back(std::array<double, 6>{x_max, -y_max, -z_max, -a_max, 0.0, c_max });

	points.push_back(std::array<double, 6>{-x_max, y_max, z_max, -a_max, 0.0, -c_max });
	points.push_back(std::array<double, 6>{-x_max, y_max, -z_max, a_max, 0.0, -c_max });
	points.push_back(std::array<double, 6>{-x_max, y_max, -z_max, -a_max, 0.0, c_max });

	points.push_back(std::array<double, 6>{-x_max, -y_max, z_max, a_max, 0.0, -c_max });
	points.push_back(std::array<double, 6>{-x_max, -y_max, z_max, -a_max, 0.0, c_max });

	points.push_back(std::array<double, 6>{-x_max, -y_max, -z_max, a_max, 0.0, c_max });

	// 1 positive //
	points.push_back(std::array<double, 6>{x_max, -y_max, -z_max, -a_max, 0.0, -c_max });
	points.push_back(std::array<double, 6>{-x_max, y_max, -z_max, -a_max, 0.0, -c_max });
	points.push_back(std::array<double, 6>{-x_max, -y_max, z_max, -a_max, 0.0, -c_max });
	points.push_back(std::array<double, 6>{-x_max, -y_max, -z_max, a_max, 0.0, -c_max });
	points.push_back(std::array<double, 6>{-x_max, -y_max, -z_max, -a_max, 0.0, c_max });


	auto cpt_margin = [&]() ->double
	{
		double max_max_margin(1);
		
		for (auto pnt : points)
		{
			s_va(6, mid_pe, pnt.data());
			ee.setMpe(pnt.data(), "123");

			bool ret = inv.kinPos();

			double max_margin{ 1 };
			for (auto &mot : m.motionPool())
			{
				max_margin = std::min(0.76 - mot.mp(), max_margin);
				max_margin = std::min(mot.mp() - 0.54, max_margin);
			}


			max_max_margin = std::min(max_max_margin, max_margin);
			if (max_max_margin < 0) return max_max_margin;
		}
		
		
		return max_max_margin;
	};




	std::cout << "result:" << cpt_margin() << std::endl;
	
	

	

	//std::cout << "input:";

	
	//std::cout <<"margin: "<<max_margin<< std::endl;
	//ret = fwd.kinPos();

	//

	//ee.updMpm();
	//dsp(4, 4, *ee.mpm());




	







	std::cout << "demo_stewart finished, press any key to continue" << std::endl;
	std::cin.get();
	return 0; 
}

