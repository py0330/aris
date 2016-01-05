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
	/*
	Calculator c;


	{
		Matrix X(0.1);
		Matrix Y = X;
	}
	
	
	
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
	c.AddFunction("multiply", [](std::vector<Aris::DynKer::Matrix> matrices)
	{
		return matrices.at(0)*matrices.at(1);
	}, 2);
	c.AddFunction("addTwo", [](std::vector<Aris::DynKer::Matrix> matrices)
	{
		return matrices.at(0) + 2;
	}, 1);

	std::vector<double> t = { 1, 2, 3 };

	double x = 0.1;

	Matrix m = { 1, 2.2, 3,{}, 2, 3.2, 4 };
	Matrix n = { x,{}, x + 0.1 };

	m.Transpose();

	Matrix m2;

	//std::vector<Matrix> v = { m, m2 };

	Matrix k = { { m, m }, { n, n }, {1,2,3,4} };
	k.dsp();

	k(1, 1) = 31.415926532345e16;

	cout << k.ToString();


	k({ 1, 2 }, { 0, 1 }).dsp();

	//not compatible for clang
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
	*/
	char aaa;
	cin>>aaa;
	return 0;
}

