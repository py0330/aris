#include <iostream>
#include <iomanip>
#include <cmath>
#include <string>
#include <sstream>
#include "aris_core.h"

using namespace std;

int main()
{
	Aris::Core::Calculator c;


	{
		Aris::Core::Matrix X(0.1);
		Aris::Core::Matrix Y = X;
	}
	
	
	
	try
	{
		c.addVariable("fiveee", 5);
	}
	catch (std::exception &e)
	{
		cout << e.what();
	}
	c.addVariable("FIVE", 5);
	c.addVariable("TWO", 2);
	c.addVariable("PI", 3.141592653);
	c.addFunction("multiply", [](std::vector<Aris::Core::Matrix> matrices)
	{
		return matrices.at(0)*matrices.at(1);
	}, 2);
	c.addFunction("addTwo", [](std::vector<Aris::Core::Matrix> matrices)
	{
		return matrices.at(0) + 2;
	}, 1);

	std::vector<double> t = { 1, 2, 3 };

	double x = 0.1;

	Aris::Core::Matrix m = { 1, 2.2, 3,{}, 2, 3.2, 4 };
	Aris::Core::Matrix n = { x,{}, x + 0.1 };

	//m.transpose();

	Aris::Core::Matrix m2;

	//std::vector<Matrix> v = { m, m2 };

	Aris::Core::Matrix k = { { m,{}, m },{ n,{}, n },{ 1,{},2,{},{},3,{},4 } };
	k.dsp();

	k(1, 1) = 31.415926532345e16;

	cout << k.toString();


	//not compatible for clang
	k = {};





	
	try
	{
		m = c.calculateExpression("{multiply(-100+multiply(1,2)+1.23*addTwo(2.1-3.2*FIVE*(3-1)*(TWO*PI - 1))-1.38, 8.1),1}");
		m.dsp();
		//m = c.calculateExpression("multiply(multiply(1),2)");
		m = c.calculateExpression("2e-2+{1,{2,3};{4;5},{5;6},{7;8}}*3");
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

