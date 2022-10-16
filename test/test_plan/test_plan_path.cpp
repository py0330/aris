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
		aris::plan::s_bezier3_blend_line_line(0.56, p0, p1, p2, p, dp, d2p);

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
		aris::plan::s_bezier3_blend_line_circle(0.38, p0, p1, center, ax, theta, p, dp, d2p);

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

		aris::plan::s_bezier3_blend_circle_circle(0.68, p1,
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

	// test s_blend_quaternion_bezier3 //
	{
		double q0[4]{ 0.409512331537456,
			 0.283529229761645,
			0.0437129979471227,
			 0.866025403784439 };

		double q1[4]{ 0.0534299365105471,
			0.0637953790354169,
			0.0259136063121744,
			 0.996194698091746 };

		double q2[4]{ 0.147477479964183,
			 0.082527864550064,
			 0.039915336441851,
			 0.984807753012208 };

		double q[4], dq[4], d2q[4];
		aris::plan::s_bezier3_blend_quaternion(0.66,
			q0,	q1, q2,
			q, dq, d2q);

		const double q_[4]{ 0.095189031999334,
			0.0783817543850458,
			0.0307778043574334,
			 0.991891161128824
		};
		const double dq_[4]{ -0.00596189256812635,
		   - 0.0559156109598799,
			0.0112331920023273,
		   0.00464218017226164
		};
		const double d2q_[4]{ 1.13118014727655,
		 0.547101513143992,
		0.0971962711225237,
		- 0.158142392813288
		};

		if (!(aris::dynamic::s_is_equal(4, q, q_, error)
			&& aris::dynamic::s_is_equal(4, dq, dq_, error)
			&& aris::dynamic::s_is_equal(4, d2q, d2q_, error)))
			std::cout << "\"s_blend_quaternion_bezier3\" failed" << std::endl;
	}

	// test s_blend_quaternion_bezier3 //
	{
		double q0[4]{ std::sin(0.1), 0.0, 0.0, std::cos(0.1)};
		double q1[4]{ 0,0,0,1 };
		double q2[4]{ std::sin(0.1), 0.0, 0.0, std::cos(0.1) };

		double q[4], dq[4], d2q[4];
		aris::plan::s_bezier3_blend_quaternion(0.5, q0, q1, q2, q, dq, d2q);

		const double q_[4]{ 0.095189031999334,
			0.0783817543850458,
			0.0307778043574334,
			 0.991891161128824
		};
		const double dq_[4]{ -0.00596189256812635,
		   -0.0559156109598799,
			0.0112331920023273,
		   0.00464218017226164
		};
		const double d2q_[4]{ 1.13118014727655,
		 0.547101513143992,
		0.0971962711225237,
		-0.158142392813288
		};

		if (!(aris::dynamic::s_is_equal(4, q, q_, error)
			&& aris::dynamic::s_is_equal(4, dq, dq_, error)
			&& aris::dynamic::s_is_equal(4, d2q, d2q_, error)))
			std::cout << "\"s_blend_quaternion_bezier3\" failed" << std::endl;
	}
}

void test_path()
{
	std::cout << std::endl << "-----------------test path---------------------" << std::endl;
	//test_optimal();
	test_blend_bezier();
	std::cout << "-----------------test path finished------------" << std::endl << std::endl;
}

