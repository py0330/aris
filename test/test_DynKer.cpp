#include <iostream>
#include "aris_dynamic_kernel.h"

using namespace Aris::Dynamic;

int main(int argc, char *argv[])
{
	const double error = 0.0000001;
	
	//test isEqual
	{
		double a[] = { 0.1,0.2,0.3 };
		double b[] = { 0.1,0.2,0.3 };
		double c[] = { 0.1,0.2,0.35 };

		if (!s_is_equal(3, a, b, error))
		{
			std::cout << "\"isEqual\" failed" << std::endl;
		}

		if (s_is_equal(3, a, c, error))
		{
			std::cout << "\"isEqual\" failed" << std::endl;
		}
	}

	//test s_cro3
	{
		double a[] = { 0.12, 0.25, 0.6 };
		double b[] = { 0.13, -0.21, 0.33 };
		double c[] = { 0.11,0.22,0.33 };
		double answer1[] = { 0.066425,   0.0382,   0.028475 };
		double answer2[] = { 0.2085,0.0384,-0.0577 };

		s_cro3(0.25, a, b, 0.13, c);

		if (!s_is_equal(3, c, answer1, error))
		{
			std::cout << "\"s_cro3\" failed" << std::endl;
		}

		s_cro3(a, b, c);

		if (!s_is_equal(3, c, answer2, error))
		{
			std::cout << "\"s_cro3\" failed" << std::endl;
		}
	}

	//test s_cm3
	{
		double a[] = { 0.1, 0.2, 0.3 };
		double result[9];
		double answer[9] = {
			0,  -0.3, 0.2,
			0.3, 0,  -0.1,
		   -0.2, 0.1, 0};

		s_cm3(a, result);

		if (!s_is_equal(9, result, answer, error))
		{
			std::cout << "\"s_cro3\" failed" << std::endl;
		}
	}

	//test s_pe2pm
	{
		double pe[] = { 0.1,0.2,0.3,0.4,0.5,0.6};
		double result[16];
		double answer[16] =
		{
			0.808307066774345, -0.072065911490471, 0.584333971461272, 0.1,
			0.341746746490328,   0.865601553329486, -0.365982393206091, 0.2,
			-0.479425538604203,   0.495520388354132,   0.724300143351802, 0.3,
			0,0,0,1
		};

		s_pe2pm(pe, result, "321");

		if (!s_is_equal(16, result, answer, error))
		{
			std::cout << "\"s_pe2pm\" failed" << std::endl;
		}

		s_pe2pm(pe, result);

		double answer2[16] =
		{
			0.567219713641686, -0.802125918959455, 0.186697098503681,0.1,
			0.777805328452570,   0.447242474005492, -0.441580163137156,0.2,
			0.270704021926224,   0.395686971707304,0.877582561890373,0.3,
			0,0,0,1
		};

		if (!s_is_equal(16, result, answer2, error))
		{
			std::cout << "\"s_pe2pm\" failed" << std::endl;
		}


	}

	//test s_pm2pe
	{
		{
			double pe[] = { 0,0,0,0.3,0.2,0.1 };
			double pe2[6];
			double pm[16];
			double result[16];

			s_pe2pm(pe, pm);
			s_pm2pe(pm, pe2);
			s_pe2pm(pe2, result);

			if (!s_is_equal(16, result, pm, error))
			{
				std::cout << "\"s_pm2pe\" failed" << std::endl;
			}
		}

		{
			double pe[] = { 0,0,0,0.3,0.2,0.1 };
			double pe2[6];
			double pm[16];
			double result[16];

			s_pe2pm(pe, pm);
			s_pm2pe(pm, pe2, "321");
			s_pe2pm(pe2, result, "321");

			if (!s_is_equal(16, result, pm, error))
			{
				std::cout << "\"s_pm2pe\" failed" << std::endl;
			}
		}


	}
	
	return 0;
}