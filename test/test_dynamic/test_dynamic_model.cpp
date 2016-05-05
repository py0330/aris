#include "test_dynamic_model.h"
#include <iostream>
#include <aris.h>

using namespace aris::dynamic;

void test_coordinate()
{
	aris::dynamic::Model model;

	auto &p = model.partPool().add<Part>("test_part");
	auto &r = model.partPool().add<Part>("relative_part");

	const double pp[3] = { 0.1, 0.2, 0.3 };
	const double re313[3] = { 0.000423769269879415,   1.38980987554835,   1.79253453841257 };
	const double re321[3] = { 2.46823966120654, -1.28551725555848,  5.40636866254317 };
	const double rq[4] = { 0.4,-0.5, 0.6, std::sqrt(1 - 0.4*0.4 - 0.5*0.5 - 0.6*0.6) };
	const double rm[9] = { -0.22, -0.975499782797526,   0.000416847668728071,
		0.175499782797526, -0.04, -0.983666521865018,
		0.959583152331272, -0.216333478134982,   0.18 };
	const double pe313[6] = { 0.1, 0.2, 0.3,0.000423769269879415,   1.38980987554835,   1.79253453841257 };
	const double pe321[6] = { 0.1, 0.2, 0.3,2.46823966120654, -1.28551725555848,  5.40636866254317 };
	const double pq[7] = { 0.1, 0.2, 0.3,0.4,-0.5, 0.6, std::sqrt(1 - 0.4*0.4 - 0.5*0.5 - 0.6*0.6) };
	const double pm[16] = { -0.22, -0.975499782797526,   0.000416847668728071, 0.1,
		0.175499782797526, -0.04, -0.983666521865018, 0.2,
		0.959583152331272, -0.216333478134982,   0.18,0.3,
		0,0,0,1 };

	const double vp[3] = { 0.307558670154491,   1.2433000508379, -1.04895965543501 };
	const double we313[3] = { -0.644213536852877, -0.245050866834802, -1.27836042009784 };
	const double we321[3] = { -4.19969388864156, -0.83045134600268,   3.46543753721832 };
	const double wq[4] = { 0.1, 0.2, -0.4, -(rq[0] * 0.1 + rq[1] * 0.2 - rq[2] * 0.4) / rq[3] };
	const double wm[9] = { 1.36, -0.30698536874045, -0.633709981238717,
		0.426985368740452,   0.8,   0.0436487757967661,
		0.233709981238715,   1.23635122420323,   0.24 , };
	const double ve313[6] = { 0.307558670154491,   1.2433000508379, -1.04895965543501,-0.644213536852877, -0.245050866834802, -1.27836042009784 };
	const double ve321[6] = { 0.307558670154491,   1.2433000508379, -1.04895965543501,-4.19969388864156, -0.83045134600268,   3.46543753721832 };
	const double vq[7] = { 0.307558670154491,   1.2433000508379, -1.04895965543501, 0.1, 0.2, -0.4, -(rq[0] * 0.1 + rq[1] * 0.2 - rq[2] * 0.4) / rq[3] };
	const double vm[16] = { 1.36, -0.30698536874045, -0.633709981238717,0.307558670154491,
		0.426985368740452,   0.8,   0.0436487757967661,1.2433000508379,
		0.233709981238715,   1.23635122420323,   0.24 , -1.04895965543501,
		0,0,0,0 };
	const double wa[3] = { -0.244517963270725,	1.25737650310373,	-0.874318412470487 };
	const double va[6] = { 0.307558670154491,   1.2433000508379, -1.04895965543501, -0.244517963270725,	1.25737650310373,	-0.874318412470487 };
	const double vs[6] = { -0.244517963270725,	1.25737650310373,	-0.874318412470487, -0.244517963270725,	1.25737650310373,	-0.874318412470487 };

	const double ap[3] = { 2.2628985000154, -0.843606386309081, -0.248846478459814 };
	const double xe313[3] = { 1.51734920338156,   1.71538128045296,   1.3693196878275 };
	const double xe321[3] = { -15.6049676192293,   4.50445705187534,   16.9352080725126 };
	const double xq[4] = { -0.033,   0.022, 0.011,   -(wq[0] * wq[0] + wq[1] * wq[1] + wq[2] * wq[2] + wq[3] * wq[3] + rq[0] * (-0.033) + rq[1] * (0.022) + rq[2] * (0.011)) / rq[3] };
	const double xm[9] = { -0.782400000000002,   2.58144759895694,   1.54784395313479,
		-2.32024759895695, -0.653600000000002,   0.450521351741563,
		-1.92944395313478, -1.05972135174157, -0.103200000000001 };
	const double ae313[6] = { 2.2628985000154, -0.843606386309081, -0.248846478459814, 1.51734920338156,   1.71538128045296,   1.3693196878275 };
	const double ae321[6] = { 2.2628985000154, -0.843606386309081, -0.248846478459814, -15.6049676192293,   4.50445705187534,   16.9352080725126 };
	const double aq[7] = { 2.2628985000154, -0.843606386309081, -0.248846478459814, -0.033,   0.022, 0.011,   -(wq[0] * wq[0] + wq[1] * wq[1] + wq[2] * wq[2] + wq[3] * wq[3] + rq[0] * (-0.033) + rq[1] * (0.022) + rq[2] * (0.011)) / rq[3] };
	const double am[16] = { -0.782400000000002,   2.58144759895694,   1.54784395313479,2.2628985000154,
		-2.32024759895695, -0.653600000000002,   0.450521351741563,-0.843606386309081,
		-1.92944395313478, -1.05972135174157, -0.103200000000001,-0.248846478459814,
		0,0,0,0 };
	const double xa[3] = { 0.904633672502324, -1.24440604199266,   1.45568007018557 };
	const double aa[6] = { 2.2628985000154, -0.843606386309081, -0.248846478459814, 0.904633672502324, -1.24440604199266,   1.45568007018557 };
	const double as[6] = { 3.15925342342501, -0.192390604845803,   0.136512424183815,   0.904633672502324, -1.24440604199266,   1.45568007018557 };



	const double relative_vs[16] = { 0.12, -0.35, 0.26, 0.58, 0.36, -0.135 };
	const double relative_as[16] = { 0.14, 1.35, -0.35, -0.56, -0.34, 0.14 };
	const double relative_pm[16] = { -0.22, -0.975499782797526,   0.000416847668728071,   0.1,
		0.175499782797526, -0.04, -0.983666521865018,   0.2,
		0.959583152331272, -0.216333478134982,   0.18,   0.3,
		0,   0,   0,   1 };

	const double to_pm[4][4] = { -0.1224,   0.253539765421328,   0.959549804517774, -0.116974902258887,
		-0.989539765421329,   0.0432, -0.137640156385781, -0.0855499782797527,
		-0.0763498045177736, -0.966359843614219,   0.2456,   0.406691619606131,
		0,   0,   0,   1 };
	const double to_vs[6] = { -1.4159950169102,   0.131290065274018, -0.0927140779913885, -0.593141011344806,   1.12682984222913, -0.799025264483263 };
	const double to_as[6] = { 0.25059773457297,   2.12918260428844,   2.93584830296579,   0.319978146055887, -1.01985580694063,   2.40639240365168 };

	const double error = 1e-10;

	double result[16], result2[16], result3[16];

	r.setPm(relative_pm);
	r.setVs(relative_vs);
	r.setAs(relative_as);

	p.setPm(pm);
	p.setVs(vs);
	p.setAs(as);

	p.getPp(result);
	if (!(s_is_equal(3, result, pp, error)))std::cout << "\"coordinate:getPp\" failed" << std::endl;

	p.getPe(result);
	if (!(s_is_equal(6, result, pe313, error)))std::cout << "\"coordinate:getPe\" failed" << std::endl;

	p.getPe(result, "321");
	if (!(s_is_equal(6, result, pe321, error)))std::cout << "\"coordinate:getPe 321\" failed" << std::endl;

	p.getPq(result);
	if (!(s_is_equal(7, result, pq, error)))std::cout << "\"coordinate:getPq\" failed" << std::endl;

	p.getPm(result);
	if (!(s_is_equal(16, result, pm, error)))std::cout << "\"coordinate:getPm\" failed" << std::endl;

	p.getVp(result, result2);
	if (!(s_is_equal(3, result, vp, error)&& s_is_equal(3, result2, pp, error)))std::cout << "\"coordinate:getVp\" failed" << std::endl;

	p.getVe(result, result2);
	if (!(s_is_equal(6, result, ve313, error) && s_is_equal(6, result2, pe313, error)))std::cout << "\"coordinate:getVe 313\" failed" << std::endl;

	p.getVe(result, result2, "321");
	if (!(s_is_equal(6, result, ve321, error) && s_is_equal(6, result2, pe321, error)))std::cout << "\"coordinate:getVe 321\" failed" << std::endl;

	p.getVq(result, result2);
	if (!(s_is_equal(7, result, vq, error) && s_is_equal(7, result2, pq, error)))std::cout << "\"coordinate:getVq\" failed" << std::endl;

	p.getVm(result, result2);
	if (!(s_is_equal(16, result, vm, error) && s_is_equal(16, result2, pm, error)))std::cout << "\"coordinate:getVm\" failed" << std::endl;

	p.getVa(result, result2);
	if (!(s_is_equal(6, result, va, error) && s_is_equal(3, result2, pp, error)))std::cout << "\"coordinate:getVa\" failed" << std::endl;

	p.getVs(result, result2);
	if (!(s_is_equal(6, result, vs, error) && s_is_equal(16, result2, pm, error)))std::cout << "\"coordinate:getVs\" failed" << std::endl;

	std::cout << "test coordinate finished" << std::endl;
}
void test_part()
{
	aris::dynamic::Model model;

	auto &p = model.partPool().add<Part>("test_part");
	auto &r = model.partPool().add<Part>("relative_part");

	const double pp[3] = { 0.1, 0.2, 0.3 };
	const double re313[3] = { 0.000423769269879415,   1.38980987554835,   1.79253453841257 };
	const double re321[3] = { 2.46823966120654, -1.28551725555848,  5.40636866254317 };
	const double rq[4] = { 0.4,-0.5, 0.6, std::sqrt(1 - 0.4*0.4 - 0.5*0.5 - 0.6*0.6) };
	const double rm[9] = { -0.22, -0.975499782797526,   0.000416847668728071,
		0.175499782797526, -0.04, -0.983666521865018,
		0.959583152331272, -0.216333478134982,   0.18 };
	const double pe313[6] = { 0.1, 0.2, 0.3,0.000423769269879415,   1.38980987554835,   1.79253453841257 };
	const double pe321[6] = { 0.1, 0.2, 0.3,2.46823966120654, -1.28551725555848,  5.40636866254317 };
	const double pq[7] = { 0.1, 0.2, 0.3,0.4,-0.5, 0.6, std::sqrt(1 - 0.4*0.4 - 0.5*0.5 - 0.6*0.6) };
	const double pm[16] = { -0.22, -0.975499782797526,   0.000416847668728071, 0.1,
		0.175499782797526, -0.04, -0.983666521865018, 0.2,
		0.959583152331272, -0.216333478134982,   0.18,0.3,
		0,0,0,1 };

	const double vp[3] = { 0.307558670154491,   1.2433000508379, -1.04895965543501 };
	const double we313[3] = { -0.644213536852877, -0.245050866834802, -1.27836042009784 };
	const double we321[3] = { -4.19969388864156, -0.83045134600268,   3.46543753721832 };
	const double wq[4] = { 0.1, 0.2, -0.4, -(rq[0] * 0.1 + rq[1] * 0.2 - rq[2] * 0.4) / rq[3] };
	const double wm[9] = { 1.36, -0.30698536874045, -0.633709981238717,
		0.426985368740452,   0.8,   0.0436487757967661,
		0.233709981238715,   1.23635122420323,   0.24 , };
	const double ve313[6] = { 0.307558670154491,   1.2433000508379, -1.04895965543501,-0.644213536852877, -0.245050866834802, -1.27836042009784 };
	const double ve321[6] = { 0.307558670154491,   1.2433000508379, -1.04895965543501,-4.19969388864156, -0.83045134600268,   3.46543753721832 };
	const double vq[7] = { 0.307558670154491,   1.2433000508379, -1.04895965543501, 0.1, 0.2, -0.4, -(rq[0] * 0.1 + rq[1] * 0.2 - rq[2] * 0.4) / rq[3] };
	const double vm[16] = { 1.36, -0.30698536874045, -0.633709981238717,0.307558670154491,
		0.426985368740452,   0.8,   0.0436487757967661,1.2433000508379,
		0.233709981238715,   1.23635122420323,   0.24 , -1.04895965543501,
		0,0,0,0 };
	const double wa[3] = { -0.244517963270725,	1.25737650310373,	-0.874318412470487 };
	const double va[6] = { 0.307558670154491,   1.2433000508379, -1.04895965543501, -0.244517963270725,	1.25737650310373,	-0.874318412470487 };
	const double vs[6] = { -0.244517963270725,	1.25737650310373,	-0.874318412470487, -0.244517963270725,	1.25737650310373,	-0.874318412470487 };

	const double ap[3] = { 2.2628985000154, -0.843606386309081, -0.248846478459814 };
	const double xe313[3] = { 1.51734920338156,   1.71538128045296,   1.3693196878275 };
	const double xe321[3] = { -15.6049676192293,   4.50445705187534,   16.9352080725126 };
	const double xq[4] = { -0.033,   0.022, 0.011,   -(wq[0] * wq[0] + wq[1] * wq[1] + wq[2] * wq[2] + wq[3] * wq[3] + rq[0] * (-0.033) + rq[1] * (0.022) + rq[2] * (0.011)) / rq[3] };
	const double xm[9] = { -0.782400000000002,   2.58144759895694,   1.54784395313479,
		-2.32024759895695, -0.653600000000002,   0.450521351741563,
		-1.92944395313478, -1.05972135174157, -0.103200000000001 };
	const double ae313[6] = { 2.2628985000154, -0.843606386309081, -0.248846478459814, 1.51734920338156,   1.71538128045296,   1.3693196878275 };
	const double ae321[6] = { 2.2628985000154, -0.843606386309081, -0.248846478459814, -15.6049676192293,   4.50445705187534,   16.9352080725126 };
	const double aq[7] = { 2.2628985000154, -0.843606386309081, -0.248846478459814, -0.033,   0.022, 0.011,   -(wq[0] * wq[0] + wq[1] * wq[1] + wq[2] * wq[2] + wq[3] * wq[3] + rq[0] * (-0.033) + rq[1] * (0.022) + rq[2] * (0.011)) / rq[3] };
	const double am[16] = { -0.782400000000002,   2.58144759895694,   1.54784395313479,2.2628985000154,
		-2.32024759895695, -0.653600000000002,   0.450521351741563,-0.843606386309081,
		-1.92944395313478, -1.05972135174157, -0.103200000000001,-0.248846478459814,
		0,0,0,0 };
	const double xa[3] = { 0.904633672502324, -1.24440604199266,   1.45568007018557 };
	const double aa[6] = { 2.2628985000154, -0.843606386309081, -0.248846478459814, 0.904633672502324, -1.24440604199266,   1.45568007018557 };
	const double as[6] = { 3.15925342342501, -0.192390604845803,   0.136512424183815,   0.904633672502324, -1.24440604199266,   1.45568007018557 };



	const double relative_vs[16] = { 0.12, -0.35, 0.26, 0.58, 0.36, -0.135 };
	const double relative_as[16] = { 0.14, 1.35, -0.35, -0.56, -0.34, 0.14 };
	const double relative_pm[16] = { -0.22, -0.975499782797526,   0.000416847668728071,   0.1,
		0.175499782797526, -0.04, -0.983666521865018,   0.2,
		0.959583152331272, -0.216333478134982,   0.18,   0.3,
		0,   0,   0,   1 };

	const double to_pm[4][4] = { -0.1224,   0.253539765421328,   0.959549804517774, - 0.116974902258887,
		- 0.989539765421329,   0.0432, - 0.137640156385781, - 0.0855499782797527,
		- 0.0763498045177736, - 0.966359843614219,   0.2456,   0.406691619606131,
		0,   0,   0,   1 };
	const double to_vs[6] = { -1.4159950169102,   0.131290065274018, - 0.0927140779913885, - 0.593141011344806,   1.12682984222913, - 0.799025264483263 };
	const double to_as[6] = { 0.25059773457297,   2.12918260428844,   2.93584830296579,   0.319978146055887, - 1.01985580694063,   2.40639240365168 };

	const double error = 1e-10;

	double result[16];

	r.setPm(relative_pm);
	r.setVs(relative_vs);
	r.setAs(relative_as);

	p.setPp(pp);
	if (!(s_is_equal(3, &pm[3], &p.pm()[0][3], error, 4, 4)))std::cout << "\"part:setPp\" failed" << std::endl;

	p.setPp(r, pp);
	if (!(s_is_equal(3, &to_pm[0][3], &p.pm()[0][3], error, 4, 4)))std::cout << "\"part:setPp relative\" failed" << std::endl;

	p.setPe(pe313);
	if (!(s_is_equal(16, pm, &p.pm()[0][0], error)))std::cout << "\"part:setPe\" failed" << std::endl;

	p.setPe(r, pe313);
	if (!(s_is_equal(16, &to_pm[0][0], &p.pm()[0][0], error)))std::cout << "\"part:setPe relative\" failed" << std::endl;

	p.setPe(pe321, "321");
	if (!(s_is_equal(16, pm, &p.pm()[0][0], error)))std::cout << "\"part:setPe\" failed" << std::endl;

	p.setPe(r, pe321, "321");
	if (!(s_is_equal(16, &to_pm[0][0], &p.pm()[0][0], error)))std::cout << "\"part:setPe 321 relative\" failed" << std::endl;

	p.setPq(pq);
	if (!(s_is_equal(16, pm, &p.pm()[0][0], error)))std::cout << "\"part:setPq\" failed" << std::endl;

	p.setPq(r, pq);
	if (!(s_is_equal(16, &to_pm[0][0], &p.pm()[0][0], error)))std::cout << "\"part:setPq relative\" failed" << std::endl;

	p.setPm(pm);
	if (!(s_is_equal(16, pm, &p.pm()[0][0], error)))std::cout << "\"part:setPm\" failed" << std::endl;

	p.setPm(r, pm);
	if (!(s_is_equal(16, &to_pm[0][0], &p.pm()[0][0], error)))std::cout << "\"part:setPm relative\" failed" << std::endl;


	p.setVe(ve313, pe313);
	if (!(s_is_equal(6, vs, p.vs(), error) && s_is_equal(16, pm, *p.pm(), error)))std::cout << "\"part:setVe\" failed" << std::endl;

	p.setVe(r, ve313, pe313);
	if (!(s_is_equal(6, to_vs, p.vs(), error) && s_is_equal(16, *to_pm, *p.pm(), error)))std::cout << "\"part:setVe relative\" failed" << std::endl;

	p.setPe(pe313);
	p.setVe(ve313);
	if (!(s_is_equal(6, vs, p.vs(), error) && s_is_equal(16, pm, *p.pm(), error)))std::cout << "\"part:setVe\" failed" << std::endl;

	p.setPe(r, pe313);
	p.setVe(r, ve313);
	if (!(s_is_equal(6, to_vs, p.vs(), error) && s_is_equal(16, *to_pm, *p.pm(), error)))std::cout << "\"part:setVe relative\" failed" << std::endl;

	p.setVe(ve321, pe321, "321");
	if (!(s_is_equal(6, vs, p.vs(), error) && s_is_equal(16, pm, *p.pm(), error)))std::cout << "\"part:setVe 321\" failed" << std::endl;

	p.setVe(r, ve321, pe321, "321");
	if (!(s_is_equal(6, to_vs, p.vs(), error) && s_is_equal(16, *to_pm, *p.pm(), error)))std::cout << "\"part:setVe 321 relative\" failed" << std::endl;

	p.setPe(pe321, "321");
	p.setVe(ve321, nullptr, "321");
	if (!(s_is_equal(6, vs, p.vs(), error) && s_is_equal(16, pm, *p.pm(), error)))std::cout << "\"part:setVe 321\" failed" << std::endl;

	p.setPe(r, pe321, "321");
	p.setVe(r, ve321, nullptr, "321");
	if (!(s_is_equal(6, to_vs, p.vs(), error) && s_is_equal(16, *to_pm, *p.pm(), error)))std::cout << "\"part:setVe 321 relative\" failed" << std::endl;

	p.setVq(vq, pq);
	if (!(s_is_equal(6, vs, p.vs(), error) && s_is_equal(16, pm, *p.pm(), error)))std::cout << "\"part:setVq\" failed" << std::endl;

	p.setVq(r, vq, pq);
	if (!(s_is_equal(6, to_vs, p.vs(), error) && s_is_equal(16, *to_pm, *p.pm(), error)))std::cout << "\"part:setVq relative\" failed" << std::endl;

	p.setPq(pq);
	p.setVq(vq);
	if (!(s_is_equal(6, vs, p.vs(), error) && s_is_equal(16, pm, *p.pm(), error)))std::cout << "\"part:setVq\" failed" << std::endl;

	p.setPq(r, pq);
	p.setVq(r, vq);
	if (!(s_is_equal(6, to_vs, p.vs(), error) && s_is_equal(16, *to_pm, *p.pm(), error)))std::cout << "\"part:setVq relative\" failed" << std::endl;

	p.setVm(vm, pm);
	if (!(s_is_equal(6, vs, p.vs(), error) && s_is_equal(16, pm, *p.pm(), error)))std::cout << "\"part:setVm\" failed" << std::endl;

	p.setVm(r, vm, pm);
	if (!(s_is_equal(6, to_vs, p.vs(), error) && s_is_equal(16, *to_pm, *p.pm(), error)))std::cout << "\"part:setVm relative\" failed" << std::endl;

	p.setPm(pm);
	p.setVm(vm);
	if (!(s_is_equal(6, vs, p.vs(), error) && s_is_equal(16, pm, *p.pm(), error)))std::cout << "\"part:setVm\" failed" << std::endl;

	p.setPm(r, pm);
	p.setVm(r, vm);
	if (!(s_is_equal(6, to_vs, p.vs(), error) && s_is_equal(16, *to_pm, *p.pm(), error)))std::cout << "\"part:setVm relative\" failed" << std::endl;

	p.setVa(va, pp);
	if (!(s_is_equal(6, vs, p.vs(), error) && s_is_equal(3, pp, &p.pm()[0][3], error, 1, 4)))std::cout << "\"part:setVa\" failed" << std::endl;

	p.setVa(r, va, pp);
	if (!(s_is_equal(6, to_vs, p.vs(), error) && s_is_equal(3, &to_pm[0][3], &p.pm()[0][3], error, 4, 4)))std::cout << "\"part:setVa relative\" failed" << std::endl;

	p.setPp(pp);
	p.setVa(va);
	if (!(s_is_equal(6, vs, p.vs(), error) && s_is_equal(3, pp, &p.pm()[0][3], error, 1, 4)))std::cout << "\"part:setVa\" failed" << std::endl;

	p.setPp(r, pp);
	p.setVa(r, va);
	if (!(s_is_equal(6, to_vs, p.vs(), error) && s_is_equal(3, &to_pm[0][3], &p.pm()[0][3], error,4,4)))std::cout << "\"part:setVa relative\" failed" << std::endl;

	p.setVs(vs, pm);
	if (!(s_is_equal(6, vs, p.vs(), error) && s_is_equal(16, pm, *p.pm(), error)))std::cout << "\"part:setVs\" failed" << std::endl;

	p.setVs(r, vs, pm);
	if (!(s_is_equal(6, to_vs, p.vs(), error) && s_is_equal(16, *to_pm, *p.pm(), error)))std::cout << "\"part:setVs relative\" failed" << std::endl;

	p.setPm(pm);
	p.setVs(vs);
	if (!(s_is_equal(6, vs, p.vs(), error) && s_is_equal(16, pm, *p.pm(), error)))std::cout << "\"part:setVs\" failed" << std::endl;

	p.setPm(r, pm);
	p.setVs(r, vs);
	if (!(s_is_equal(6, to_vs, p.vs(), error) && s_is_equal(16, *to_pm, *p.pm(), error)))std::cout << "\"part:setVs relative\" failed" << std::endl;


	p.setAe(ae313, ve313, pe313);
	if (!(s_is_equal(6, as, p.as(), error)))std::cout << "\"part:setAe\" failed" << std::endl;

	p.setAe(r, ae313, ve313, pe313);
	if (!(s_is_equal(6, to_as, p.as(), error)))std::cout << "\"part:setAe relative\" failed" << std::endl;

	p.setVe(ve313, pe313);
	p.setAe(ae313);
	if (!(s_is_equal(6, as, p.as(), error)))std::cout << "\"part:setAe\" failed" << std::endl;

	p.setVe(r, ve313, pe313);
	p.setAe(r, ae313);
	if (!(s_is_equal(6, to_as, p.as(), error)))std::cout << "\"part:setAe relative\" failed" << std::endl;

	p.setAe(ae321, ve321, pe321, "321");
	if (!(s_is_equal(6, as, p.as(), error)))std::cout << "\"part:setAe 321\" failed" << std::endl;

	p.setAe(r, ae321, ve321, pe321, "321");
	if (!(s_is_equal(6, to_as, p.as(), error)))std::cout << "\"part:setAe 321 relative\" failed" << std::endl;

	p.setVe(ve321, pe321, "321");
	p.setAe(ae321, nullptr, nullptr, "321");
	if (!(s_is_equal(6, as, p.as(), error)))std::cout << "\"part:setAe 321\" failed" << std::endl;

	p.setVe(r, ve321, pe321, "321");
	p.setAe(r, ae321, nullptr, nullptr, "321");
	if (!(s_is_equal(6, to_as, p.as(), error)))std::cout << "\"part:setAe 321 relative\" failed" << std::endl;

	p.setAq(aq, vq, pq);
	if (!(s_is_equal(6, as, p.as(), error)))std::cout << "\"part:setAq\" failed" << std::endl;

	p.setAq(r, aq, vq, pq);
	if (!(s_is_equal(6, to_as, p.as(), error)))std::cout << "\"part:setAq relative\" failed" << std::endl;

	p.setVq(vq, pq);
	p.setAq(aq);
	if (!(s_is_equal(6, as, p.as(), error)))std::cout << "\"part:setAq\" failed" << std::endl;

	p.setVq(r, vq, pq);
	p.setAq(r, aq);
	if (!(s_is_equal(6, to_as, p.as(), error)))std::cout << "\"part:setAq relative\" failed" << std::endl;

	p.setAm(am, vm, pm);
	if (!(s_is_equal(6, as, p.as(), error)))std::cout << "\"part:setAm\" failed" << std::endl;

	p.setAm(r, am, vm, pm);
	if (!(s_is_equal(6, to_as, p.as(), error)))std::cout << "\"part:setAm relative\" failed" << std::endl;

	p.setVm(vm, pm);
	p.setAm(am);
	if (!(s_is_equal(6, as, p.as(), error)))std::cout << "\"part:setAm\" failed" << std::endl;

	p.setVm(r, vm, pm);
	p.setAm(r, am);
	if (!(s_is_equal(6, to_as, p.as(), error)))std::cout << "\"part:setAm relative\" failed" << std::endl;

	p.setAa(aa, va, pp);
	if (!(s_is_equal(6, as, p.as(), error)))std::cout << "\"part:setAa\" failed" << std::endl;

	p.setAa(r, aa, va, pp);
	if (!(s_is_equal(6, to_as, p.as(), error)))std::cout << "\"part:setAa relative\" failed" << std::endl;

	p.setVa(va, pp);
	p.setAa(aa);
	if (!(s_is_equal(6, as, p.as(), error)))std::cout << "\"part:setAa\" failed" << std::endl;

	p.setVa(r, va, pp);
	p.setAa(r, aa);
	if (!(s_is_equal(6, to_as, p.as(), error)))std::cout << "\"part:setAa relative\" failed" << std::endl;

	p.setAs(as, vs, pm);
	if (!(s_is_equal(6, as, p.as(), error)))std::cout << "\"part:setAs\" failed" << std::endl;

	p.setAs(r, as, vs, pm);
	if (!(s_is_equal(6, to_as, p.as(), error)))std::cout << "\"part:setAs relative\" failed" << std::endl;

	p.setVs(vs, pm);
	p.setAs(as, vs, pm);
	if (!(s_is_equal(6, as, p.as(), error)))std::cout << "\"part:setAs\" failed" << std::endl;

	p.setVs(r, vs, pm);
	p.setAs(r, as);
	if (!(s_is_equal(6, to_as, p.as(), error)))std::cout << "\"part:setAs relative\" failed" << std::endl;

	std::cout << "test part finished" << std::endl;
}

void test_auto_kinematic()
{


	double peI[6]{ 0.5,0,0,0.5,0,0 };
	double pmI[16];

	double peJ[6]{ 0,0,0,0.2,0,0 };
	double pmJ[16];

	s_pe2pm(peI, pmI);
	s_pe2pm(peJ, pmJ);

	double b[12];
	double csp[12];
	double csm[12][12];
	double csm2[12][12];
	Model model;

	auto &prt1 = model.partPool().add<Part>("part1");
	auto &makJ = model.ground().markerPool().add<Marker>("makJ", pmJ);
	auto &makI = prt1.markerPool().add<Marker>("makI", pmI);
	auto &jnt = model.jointPool().add<aris::dynamic::RevoluteJoint>("joint1", std::ref(makI), std::ref(makJ));
	auto &mot = model.motionPool().add<aris::dynamic::Motion>("motion1", std::ref(makI), std::ref(makJ), 5);

	mot.setMotPos(0);

	model.dynPre();
	model.dynUpd();
	model.dynCstPot(csp);

	//dsp(csp, 12, 1);
	model.dynCstMtx(*csm2);

	dsp(*csm2, 12, 12);

	s_mtm(12, 12, *csm2, 12, *csm, 12);

	dsp(*csm, 12, 12);

	//Eigen::Map<Eigen::Matrix<double, 12, 12, Eigen::RowMajor> > csm_mtx(*csm);
	//Eigen::Map<Eigen::Matrix<double, 12, 1> > csp_mtx(csp);
	//Eigen::Map<Eigen::Matrix<double, 12, 1> > b_mtx(b);

	//b_mtx = csm_mtx.partialPivLu().solve(csp_mtx);

	dsp(csp, 12, 1);
	dsp(b, 12, 1);

	//auto &prismatic1 = model.jointPool().
	//dlmwrite("C:\\Users\\yang\\Desktop\\test.txt", *csm, 12, 12);


}

void test_model2()
{
	
	/*
	Model model;

	model.loadXml("C:\\Users\\yang\\Desktop\\Robot_III.xml");
	model.saveXml("C:\\Users\\yang\\Desktop\\Robot_III_save.xml");
	model.saveAdams("C:\\Users\\yang\\Desktop\\robot.cmd", true);
	
	std::cout << "finished" << std::endl;*/
}


void test_model()
{
	test_coordinate();
	test_part();


	std::cout << "test model finished" << std::endl;
}

