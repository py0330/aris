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
	s_pe2pm(ep, *pm_I2G0, "321");
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



	{
		MATRIX X(0.1);
		MATRIX Y = X;
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

	/*not compatible for clang*/
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

	cout.precision(15);

	cout << sqrt(0.000000002*0.000000002 + (PI - 0.0000000001)*(PI - 0.0000000001)) << endl<<endl;

	double u[3] = { 0.15, 0.3, -0.5 };
	double pq[7] = { 0.1, 0.3, 0.5, 0.2, 0.4, 0.6, 0 };
	double ap[6] = { 0, 0.000000002, PI - 0.0000000001, 0.1, 0.2, 0.3 };
	double pm[4][4];

	double mag = sqrt(u[0] * u[0] + u[1] * u[1] + u[2] * u[2]);
	u[0] /= mag;
	u[1] /= mag;
	u[2] /= mag;

	double theta = 0.123;

	ap[0] = u[0] * theta;
	ap[1] = u[1] * theta;
	ap[2] = u[2] * theta;

	s_pr2pm(ap, *pm);
	dsp(*pm, 4, 4);

	pq[3] = u[0] * sin(theta / 2);
	pq[4] = u[1] * sin(theta / 2);
	pq[5] = u[2] * sin(theta / 2);
	pq[6] = cos(theta / 2);

	s_pq2pm(pq, *pm);
	dsp(*pm, 4, 4);


	//dsp(ap, 6, 1);

	dsp(pq, 7, 1);

	double pq2[7];

	s_pm2pq(*pm, pq2);

	dsp(pq2, 7, 1);


	double v[6] = { 0.1, 0.2, 0.3, 0.1, -0.5, 0.23 };
	double vq[7];

	dsp(v, 6, 1);

	s_v2vq(*pm, v, vq);

	dsp(vq, 7, 1);

	s_vq2v(pq, vq, v);

	dsp(v, 6, 1);





	double xxx[] = { 2.0, 1.0, 4.0, 3.0, 5.0, 6.0, 7.0, 8.0, 9.0 };
	double yyy[] = { 0, 0, 1, 0, 2, 2.3, 1.4, 1.63, 2.56 };

	AKIMA a(9, xxx, yyy);

	cout << a(3.58,'2') << endl;

	//s_pm2ap(*pm, ap2);

	//dsp(ap2, 6, 1);

	//cout << ap2[0] << endl;
	//cout << ap2[1] << endl;
	//cout << ap2[2] << endl;
	//cout << ap2[3] << endl;
	//cout << ap2[4] << endl;
	//cout << ap2[5] << endl;

	
	char aaa;
	cin>>aaa;
	return 0;
}

