#include "test_dynamic_spline.h"
#include <iostream>
#include <aris/dynamic/dynamic.hpp>

using namespace aris::dynamic;

const double error = 1e-10;

void test_akima()
{
	const double x[]{ 0.0,1.5,2.1,3.6,5.8,6.0,6.2,7.8,9.5,10.0, 11.0, 12.0, 13.0 };
	const double y[]{ 0.8407,0.2543,0.8143,0.2435,0.9293,0.3500,0.1966,0.2511,0.6160,0.4733, 1.4733, 2.4733, 3.4733 };
	const double p1[]{ -1.05306666666667,0.273810249671485,0.0705084885406555,-0.179401258869751,-0.475344937235618,-1.19247750111601,-0.0285583757597363,0.14524415812522,0.153049699584549,1,1,1 };
	const double p2[]{ 0.439682055774566,3.63645168686062,-0.735477145474373,0.804240578707699,-32.7316631220637,0.562566889958843,0.00878755837140788,0.117884212138587,-4.3245987983382,0,0,0 };
	const double p3[]{ 0.00116011096510436,-4.22874424570702,0.28985506503892,-0.264091062305767,103.129439041209,7.82410307810617,0.0189690556115171,-0.0453288096668821,6.8953987983382,0,0,0 };

	double result_p1[12], result_p2[12], result_p3[12];

	s_akima(13, x, y, result_p1, result_p2, result_p3);
	if (!(s_is_equal(12, result_p1, p1, error) && s_is_equal(12, result_p2, p2, error) && s_is_equal(12, result_p3, p3, error)))std::cout << "\"s_akima\" failed" << std::endl;

	if (!s_is_equal(s_akima_at(13, x, y, result_p1, result_p2, result_p3, 0.1), 0.739791314002044, error))std::cout << "\"s_akima_at\" failed" << std::endl;
	if (!s_is_equal(s_akima_at(13, x, y, result_p1, result_p2, result_p3, -8.5), 40.8463420499343, error))std::cout << "\"s_akima_at\" failed" << std::endl;
	if (!s_is_equal(s_akima_at(13, x, y, result_p1, result_p2, result_p3, 18.5), 8.9733, error))std::cout << "\"s_akima_at\" failed" << std::endl;
	if (!s_is_equal(s_akima_at(13, x, y, result_p1, result_p2, result_p3, 6.2), 0.1966, error))std::cout << "\"s_akima_at\" failed" << std::endl;
	
	if (!s_is_equal(s_akima_at(13, x, y, result_p1, result_p2, result_p3, 0.1, '1'), -0.9650954521828, error))std::cout << "\"s_akima_at\" failed" << std::endl;
	if (!s_is_equal(s_akima_at(13, x, y, result_p1, result_p2, result_p3, -8.5, '1'), -8.27620756314791, error))std::cout << "\"s_akima_at\" failed" << std::endl;
	if (!s_is_equal(s_akima_at(13, x, y, result_p1, result_p2, result_p3, 18.5, '1'), 1.0, error))std::cout << "\"s_akima_at\" failed" << std::endl;
	if (!s_is_equal(s_akima_at(13, x, y, result_p1, result_p2, result_p3, 6.2, '1'), -0.0285583757597363, error))std::cout << "\"s_akima_at\" failed" << std::endl;

	if (!s_is_equal(s_akima_at(13, x, y, result_p1, result_p2, result_p3, 0.1, '2'), 0.880060178128194, error))std::cout << "\"s_akima_at\" failed" << std::endl;
	if (!s_is_equal(s_akima_at(13, x, y, result_p1, result_p2, result_p3, -8.5, '2'), 0.820198452328809, error))std::cout << "\"s_akima_at\" failed" << std::endl;
	if (!s_is_equal(s_akima_at(13, x, y, result_p1, result_p2, result_p3, 18.5, '2'), 0.0, error))std::cout << "\"s_akima_at\" failed" << std::endl;
	if (!s_is_equal(s_akima_at(13, x, y, result_p1, result_p2, result_p3, 6.2, '2'), 0.0175751167428158, error))std::cout << "\"s_akima_at\" failed" << std::endl;
}

void test_interp_scurve() {
	double x[6]{ 0, 1, 2, 3.1, 4, 4.3 };
	double y[6]{ 0, 0.18, 0.24, 0.33, 0.41, 0.5 };
	
	double y_at;

	s_interp_scurve(x, y, 2.1, y_at);
	if (!s_is_equal(y_at,0.246364159792592,1e-10))
		std::cout << "failed" << std::endl;

	s_interp_scurve(x, y, 2.3, y_at);
	if (!s_is_equal(y_at, 0.260731796195777, 1e-10))
		std::cout << "failed" << std::endl;

	s_interp_scurve(x, y, 2.6, y_at);
	if (!s_is_equal(y_at, 0.286688262281698, 1e-10))
		std::cout << "failed" << std::endl;

	s_interp_scurve(x, y, 2.9, y_at);
	if (!s_is_equal(y_at, 0.313432770210573, 1e-10))
		std::cout << "failed" << std::endl;

	s_interp_scurve(x, y, 3.0, y_at);
	if (!s_is_equal(y_at, 0.321788916436907, 1e-10))
		std::cout << "failed" << std::endl;



}

void test_interp_scurve_u5_range() {
	double x[6]{ 0, 1, 2, 3.1, 4, 4.3 };
	double y[6]{ 0, 0.18, 0.24, 0.33, 0.41, 0.5 };

	double u5_range[2];
	aris::Size u5_range_num;
	s_interp_scurve_u5_range(x, y, -0.0912355750399117, 0.0912355750399117,
		-0.0409789432258887, 0.1409789432258887,
		-0.2211555, 0.2211555, u5_range_num, u5_range);


	aris::dynamic::dsp(1, 2, u5_range);




}


void test_spline()
{
	std::cout << std::endl << "-----------------test spline--------------------------" << std::endl;

	test_akima();
	test_interp_scurve();
	test_interp_scurve_u5_range();

	std::cout << "-----------------test spline finished-----------------" << std::endl << std::endl;
}