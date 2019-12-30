#include <iostream>
#include <iomanip>
#include <cmath>
#include <string>
#include <sstream>
#include <aris/core/core.hpp>

using namespace std;

int main()
{
	aris::core::Calculator c;

	
	{
		aris::core::Matrix X(0.1);
		aris::core::Matrix Y = X;
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
	c.addFunction("multiply", [](std::vector<aris::core::Matrix> matrices)
	{
		return matrices.at(0)*matrices.at(1);
	}, 2);
	c.addFunction("addTwo", [](std::vector<aris::core::Matrix> matrices)
	{
		return matrices.at(0) + 2;
	}, 1);

	std::vector<double> t = { 1, 2, 3 };

	double x = 0.1;

	aris::core::Matrix m = { 1, 2.2, 3,{}, 2, 3.2, 4 };
	aris::core::Matrix n = { x,{}, x + 0.1 };

	//m.transpose();

	aris::core::Matrix m2;

	//std::vector<Matrix> v = { m, m2 };

	aris::core::Matrix k = { { m,{}, m },{ n,{}, n },{ 1,{},2,{},{},3,{},4 } };
	k.dsp();

	k(1, 1) = 31.415926532345e16;

	cout << k.toString();


	//not compatible for clang
	k = {};

	try
	{
		m = c.calculateExpression("{multiply(-100+multiply(1,2)+1.23*addTwo(2.1-3.2*FIVE*(3-1)*(TWO*PI - 1))-1.38, 8.1),1}");//{-2448.4937065177,1}
		m.dsp();
		//m = c.calculateExpression("multiply(multiply(1,2),2)");
		m = c.calculateExpression("2e-2+{1,{2,3};{4;5},{5;6},{7;8}}*3");
		m.dsp();
	}
	catch (std::exception &e)
	{
		cout << e.what() << endl;
	}
	
	m = c.calculateExpression("2e-2<=1.0");
	m.dsp();




	std::cout << "-------------------------------------------------" << std::endl;





	aris::core::Compiler cmp;

	auto v = cmp.calculateExpression("pi+1.25");
	//auto v = cmp.calculateExpression("{{2.0,1};pi+sin(pi*2/3)+(1+(+2) - 100*5)/6 + sin(1.5*2.5+sin(0.1)),1}");
	

	std::cout << v.first << std::endl;
	//std::any_cast<aris::core::Matrix&>(v.second).dsp();


	std::cout << std::any_cast<double>(v.second) << std::endl;


	char aaa;
	cin >> aaa;
	return 0;
}

