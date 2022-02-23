#include <iostream>
#include <array>
#include <aris.hpp>

using namespace aris::dynamic;

int main()
{
	std::cout << std::endl << "-----------------test model stewart---------------------" << std::endl;
	auto&cs = aris::server::ControlServer::instance();

	try
	{
		cs.resetController(aris::robot::createControllerStewart().release());
		cs.resetModel(aris::robot::createModelStewart().release());
		cs.resetPlanRoot(aris::robot::createPlanRootStewart().release());
	}
	catch (std::exception &e)
	{
		std::cout << "exception:" << e.what() << std::endl;
		return 0;
	}
	

	auto &m = cs.model();
	//auto &inv = m.solverPool()[0];
	//auto &fwd = m.solverPool()[1];
	//auto &ee = dynamic_cast<aris::dynamic::GeneralMotion&>(m.generalMotionPool()[0]);

	// new with yaw //
	double mid_pe[6]{ 0, -0.0103, 0.513, 0, 0, 0 };
	double x_max{ 0.035 }, y_max{ 0.035 }, z_max{ 0.0373 }, a_max{ 0.08 }, b_max{ 0.08 }, c_max{ 0.04 };
	double vx_max{ 0.1 }, vy_max{ 0.1 }, vz_max{ 0.11 }, va_max{ 0.12 }, vb_max{ 0.12 }, vc_max{ 0.1 };

	auto cpt_margin = [&]() ->double
	{
		std::vector < std::array<double, 6> > points;
		for (int i = 0; i < std::pow(2, 6); ++i)
		{
			points.push_back(std::array<double, 6>{x_max, y_max, z_max, a_max, b_max, c_max });

			for (int j = 0; j < 6; ++j)
			{
				auto v = (i >> j) % 2 == 0 ? 1 : -1;

				points.back()[j] *= v;
			}
		}
		
		
		double max_max_margin= 1.0;
		
		for (auto pnt : points){
			s_va(6, mid_pe, pnt.data());
			m.setOutputPos(pnt.data());

			m.inverseKinematics();
			double max_margin{ 1 };



			


			max_max_margin = std::min(max_max_margin, max_margin);
			if (max_max_margin < 0) return max_max_margin;
		}

		return max_max_margin;
	};
	auto cpt_exact = [&](double &p_ret, double &v_ret) ->void
	{
		const int num = 2;
		const int t_num = 2 * num + 1;
		std::vector < std::array<double, 6> > points;
		for (int i = 0; i < std::pow(t_num, 6); ++i)
		{
			points.push_back(std::array<double, 6>{x_max, y_max, z_max, a_max, b_max, c_max });

			for (int j = 0; j < 6; ++j)
			{
				auto v = (i / (int)std::pow(t_num, j)) % t_num;
				double r = (v - num) / double(num);
				points.back()[j] *= r;
			}
		}

		std::vector < std::array<double, 6> > vels;
		for (int i = 0; i < std::pow(2, 6); ++i)
		{
			vels.push_back(std::array<double, 6>{vx_max, vy_max, vz_max, va_max, vb_max, vc_max });

			for (int j = 0; j < 6; ++j)
			{
				auto v = (i >> j) % 2 == 0 ? 1 : -1;

				vels.back()[j] *= v;
			}
		}

		double max_max_margin(1), max_max_v_margin(1);



		double k = 0;
		int aaa = 0;
		for (auto pnt : points)
		{
			//s_va(6, mid_pe, pnt.data());
			//ee.setMpe(pnt.data(), "123");

			//bool ret = inv.kinPos();
			//if (ret) throw std::runtime_error("failed");
			//double max_margin{ 1 };
			//for (auto &mot : m.motionPool())
			//{
			//	max_margin = std::min(0.76 - mot.mp(), max_margin);
			//	max_margin = std::min(mot.mp() - 0.54, max_margin);
			//}

			//max_max_margin = std::min(max_max_margin, max_margin);
			//if (max_max_margin < 0) return max_max_margin;


			for (auto &vel : vels)
			{
				//ee.setMve(vel.data(), "123");
				//inv.kinVel();
				double max_v_margin{ 1 };

				max_max_v_margin = std::min(max_max_v_margin, max_v_margin);
			}

			if (double(aaa) / points.size() > k + 0.01)
			{
				std::cout << k + 0.01 << std::endl;
				k += 0.01;
			}
				
			aaa++;
		}


		p_ret = max_max_margin;
		v_ret = max_max_v_margin;
	};




	std::cout << "work space margin result:" << cpt_margin() << std::endl;

	double p_margin, v_margin;
	cpt_exact(p_margin, v_margin);
	std::cout << "work space margin result:" << p_margin << "  " << v_margin << std::endl;



	std::cout << "demo_stewart finished, press any key to continue" << std::endl;
	std::cin.get();
	return 0; 
}

