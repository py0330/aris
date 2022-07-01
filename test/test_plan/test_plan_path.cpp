#include <iostream>
#include <aris/core/core.hpp>
#include <aris/plan/plan.hpp>
#include <aris/robot/rokae.hpp>

using namespace aris::plan;

void test_blend_bezier(){
	const double error = 1e-10;

	// test s_blend_line_line_bezier3 //
	{
		double p0[3]{ 0.3, 0.8, -0.3 };
		double p1[3]{ -0.5, 0.3, -0.3 };
		double p2[3]{ 0.3, -0.5, 0.2 };

		double p[3], dp[3], d2p[3];
		aris::plan::s_blend_line_line_bezier3(0.56, p0, p1, p2, p, dp, d2p);

		const double p_[3]{
				  -0.29136,
				 0.2020992,
				 - 0.212192
		};
		const double dp_[3]{
					 0.288,
				  - 1.04304,
					0.4704,
		};
		const double d2p_[3]{
					   4.8,
					- 1.368,
					  1.68
		};

		if (!(aris::dynamic::s_is_equal(3, p, p_, error)
			&& aris::dynamic::s_is_equal(3, dp, dp_, error)
			&& aris::dynamic::s_is_equal(3, d2p, d2p_, error)))
			std::cout << "\"s_blend_line_circle_bezier3\" failed" << std::endl;
	}

	// test s_blend_line_circle_bezier3 //
	{
		double theta = aris::PI / 3;
		double center[3]{ -1.5, 0.8, 0.2 };
		double ax[3]{ 0.224859506698758,
			 0.374765844497931,
			 0.899438026795034 };

		double p0[3]{ 3.5,0.7,-2.8 };
		double p1[3]{ -1.5,2,-0.3 };

		double p[3], dp[3], d2p[3];
		aris::plan::s_blend_line_circle_bezier3(0.38, p0, p1, center, ax, theta, p, dp, d2p);

		const double p_[3]{
			-0.381107335021726,
			 1.6946498923771,
			-0.879498288068363
		};
		const double dp_[3]{
			-6.33968881728938,
			 1.51881554657884,
			 3.01823239324783
		};
		const double d2p_[3]{
			 15.5955561862872,
			-4.98042456129022,
			-8.4887121460342
		};

		if (!(aris::dynamic::s_is_equal(3, p, p_, error)
			&& aris::dynamic::s_is_equal(3, dp, dp_, error)
			&& aris::dynamic::s_is_equal(3, d2p, d2p_, error)))
			std::cout << "\"s_blend_line_circle_bezier3\" failed" << std::endl;
	}

	// test s_blend_circle_circle_bezier3 //
	{
		double p1[3]{ 0.8, 0.3, -1.5 };
		double c1[3]{ -1.5, 0.8, 0.2 };
		double ax1[3]{ -0.0385744143167205,
		  0.94331976829071,
		- 0.329635904161066 };
		double theta1 = aris::PI / 6;
		double c2[3]{ 1.2, 0.4, -0.3 };
		double ax2[3]{ -0.948683298050514,
						 0,
		 0.316227766016838 };
		double theta2 = aris::PI * 2 / 3;
		double p[3], dp[3], d2p[3];

		aris::plan::s_blend_circle_circle_bezier3(0.68, p1,
			c1, ax1, theta1,
			c2, ax2, theta2,
			p, dp, d2p);

		const double p_[3]{
		 0.933000643153169,
		- 0.438965706137216,
		 - 1.15386150235178
		};
		const double dp_[3]{
		 0.505784869512604,
		 - 2.86256037663394,
		  2.00103347181547
		};
		const double d2p_[3]{
		  6.49008174398832,
		 0.025236537507917,
		  16.6711242011704
		};

		if (!(aris::dynamic::s_is_equal(3, p, p_, error)
			&& aris::dynamic::s_is_equal(3, dp, dp_, error)
			&& aris::dynamic::s_is_equal(3, d2p, d2p_, error)))
			std::cout << "\"s_blend_circle_circle_bezier3\" failed" << std::endl;
	}


}

void test_path()
{
	std::cout << std::endl << "-----------------test path---------------------" << std::endl;
	//test_optimal();
	test_blend_bezier();
	std::cout << "-----------------test path finished------------" << std::endl << std::endl;
}

