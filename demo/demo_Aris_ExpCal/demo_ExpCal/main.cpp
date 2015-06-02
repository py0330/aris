#include <iostream>
#include <iomanip>
#include <cmath>
#include <Aris_Core.h>
#include <Aris_Socket.h>
#include <Aris_DynKer.h>

#include <Aris_ExpCal.h>


using namespace std;


using namespace Aris::DynKer;

int main()
{
	CALCULATOR c;

	double pm_G02G[4][4] = 
	{
		{ -1, 0, 0, 0 },
		{ 0, 0, 1, 0 },
		{ 0, 1, 0, 0 },
		{ 0, 0, 0, 1 }
	};
	
	double ep[6] = {PI,0,PI,0,0,0};
	double pm_I2G0[4][4];
	s_ep2pm(ep, *pm_I2G0, "321");
	dsp(*pm_I2G0, 4, 4);

	double pm_R2I[4][4] =
	{
		{ 1, 0, 0, 0 },
		{ 0, 0, 1, 0 },
		{ 0, -1, 0,  0},
		{ 0, 0, 0, 1}
	};
	

	double pm_R2G0[4][4];

	double pm_R2G[4][4];

	s_pm_dot_pm(*pm_I2G0, *pm_R2I, *pm_R2G0);
	s_pm_dot_pm(*pm_G02G, *pm_R2G0, *pm_R2G);
	dsp(*pm_R2G, 4, 4);
	
	
	
	
	
	
	
	try
	{
		c.AddVariable("fiveee", 5);
	}
	catch (std::exception &e)
	{
		cout << e.what();
	}
	c.AddVariable("FIVE", 5);
	c.AddVariable("TWO", 2);
	c.AddVariable("PI", 3.141592653);
	c.AddFunction("multiply", [](std::vector<Aris::DynKer::MATRIX> matrices)
	{
		return matrices.at(0)*matrices.at(1);
	}, 2);
	c.AddFunction("addTwo", [](std::vector<Aris::DynKer::MATRIX> matrices)
	{
		return matrices.at(0) + 2;
	}, 1);

	std::vector<double> t = { 1, 2, 3 };

	double x = 0.1;

	MATRIX m = { { 1, 2.2, 3 }, { 2, 3.2, 4 } };
	MATRIX n = { { x, x + 0.1} };

	m.Transpose();

	MATRIX m2;

	//std::vector<MATRIX> v = { m, m2 };

	MATRIX k = { { m, m }, { n, n }, {1,2,3,4} };
	k.dsp();

	k(1, 1) = 31.415926532345e16;

	cout << k.ToString();


	k({ 1, 2 }, { 0, 1 }).dsp();

	k = {};





	
	try
	{
		m = c.CalculateExpression("{multiply(-100+multiply(1,2)+1.23*addTwo(2.1-3.2*FIVE*(3-1)*(TWO*PI - 1))-1.38, 8.1),1}");
		m.dsp();
		//m = c.CalculateExpression("multiply(multiply(1),2)");
		m = c.CalculateExpression("2e-2+{1,{2,3};{4;5},{5;6},{7;8}}*3");
		m.dsp();
	}
	catch (std::exception &e)
	{
		cout << e.what() << endl;
	}
	
	char aaa;
	cin>>aaa;
	return 0;
}

