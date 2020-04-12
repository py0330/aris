#include "test_dynamic_model_coordinate.h"
#include <iostream>
#include <aris/dynamic/dynamic.hpp>

#include<type_traits>

using namespace aris::dynamic;

void test_part()
{
	aris::dynamic::Model model;

	const double prt_iv[10]{ 12.3,0.1,0.2228,0.356,5.8,6.4,3.9,0.85,0.75,0.98 };
	const double prt_im[36]{ 12.3,0,0,0,0.356, -0.2228,
		0,12.3,0,-0.356,0,0.1,
		0,0,12.3,0.2228, -0.1,0,
		0, -0.356,0.2228,5.8,0.85,0.75,
		0.356,0, -0.1,0.85,6.4,0.98,
		-0.2228,0.1,0,0.75,0.98,3.9 };

	auto &p = model.partPool().add<Part>("test_part", prt_iv);
	auto &r = model.partPool().add<Part>("relative_part", prt_iv);

	model.init();

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
	const double to_as[6] = { 0.25059773457297,   2.12918260428844,   2.93584830296579,   0.319978146055887, -1.01985580694063,   2.40639240365168 };

	const double error = 1e-10;

	double result[42], result2[16], result3[16];

	r.setPm(relative_pm);
	r.setVs(relative_vs);
	r.setAs(relative_as);

	p.setPp(pp);
	if (!(s_is_equal(3, &pm[3], 4, &p.pm()[0][3], 4, error)))std::cout << "\"part:setPp\" failed" << std::endl;

	p.setPp(r, pp);
	if (!(s_is_equal(3, &to_pm[0][3], 4, &p.pm()[0][3], 4, error)))std::cout << "\"part:setPp relative\" failed" << std::endl;

	double pp_tem[3]{ 0.4,0.5,0.6 }; p.setPp(pp_tem);
	p.setRe(re313);
	if (!(s_is_equal(3, 3, pm, 4, &p.pm()[0][0], 4, error)&& s_is_equal(3, pp_tem, 1, &p.pm()[0][3], 4, error)))std::cout << "\"part:setRe\" failed" << std::endl;

	p.setRe(r, re313);
	if (!(s_is_equal(3, 3, *to_pm, 4, &p.pm()[0][0], 4, error) && s_is_equal(3, pp_tem, 1, &p.pm()[0][3], 4, error)))std::cout << "\"part:setRe relative\" failed" << std::endl;

	p.setRe(re321, "321");
	if (!(s_is_equal(3, 3, pm, 4, &p.pm()[0][0], 4, error) && s_is_equal(3, pp_tem, 1, &p.pm()[0][3], 4, error)))std::cout << "\"part:setRe 321\" failed" << std::endl;

	p.setRe(r, re321, "321");
	if (!(s_is_equal(3, 3, *to_pm, 4, &p.pm()[0][0], 4, error) && s_is_equal(3, pp_tem, 1, &p.pm()[0][3], 4, error)))std::cout << "\"part:setRe 321 relative\" failed" << std::endl;

	p.setRq(rq);
	if (!(s_is_equal(3, 3, pm, 4, &p.pm()[0][0], 4, error) && s_is_equal(3, pp_tem, 1, &p.pm()[0][3], 4, error)))std::cout << "\"part:setRq\" failed" << std::endl;

	p.setRq(r, rq);
	if (!(s_is_equal(3, 3, *to_pm, 4, &p.pm()[0][0], 4, error) && s_is_equal(3, pp_tem, 1, &p.pm()[0][3], 4, error)))std::cout << "\"part:setRq relative\" failed" << std::endl;

	p.setRm(rm);
	if (!(s_is_equal(3, 3, pm, 4, &p.pm()[0][0], 4, error) && s_is_equal(3, pp_tem, 1, &p.pm()[0][3], 4, error)))std::cout << "\"part:setRm\" failed" << std::endl;

	p.setRm(r, rm);
	if (!(s_is_equal(3, 3, *to_pm, 4, &p.pm()[0][0], 4, error) && s_is_equal(3, pp_tem, 1, &p.pm()[0][3], 4, error)))std::cout << "\"part:setRm relative\" failed" << std::endl;

	p.setRm(pm, 4);
	if (!(s_is_equal(3, 3, pm, 4, &p.pm()[0][0], 4, error) && s_is_equal(3, pp_tem, 1, &p.pm()[0][3], 4, error)))std::cout << "\"part:setRm with ld\" failed" << std::endl;

	p.setRm(r, pm, 4);
	if (!(s_is_equal(3, 3, *to_pm, 4, &p.pm()[0][0], 4, error) && s_is_equal(3, pp_tem, 1, &p.pm()[0][3], 4, error)))std::cout << "\"part:setRm with ld relative\" failed" << std::endl;
	
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

	p.setWa(wa, rm);
	p.setVp(vp, pp);
	if (!(s_is_equal(6, vs, p.vs(), error) && s_is_equal(16, pm, *p.pm(), error)))std::cout << "\"part:setVp\" failed" << std::endl;

	p.setWa(r, wa, rm);
	p.setVp(r, vp, pp);
	if (!(s_is_equal(6, to_vs, p.vs(), error) && s_is_equal(16, *to_pm, *p.pm(), error)))std::cout << "\"part:setVp relative\" failed" << std::endl;

	p.setWa(wa, rm);
	p.setPp(pp);
	p.setVp(vp);
	if (!(s_is_equal(6, vs, p.vs(), error) && s_is_equal(16, pm, *p.pm(), error)))std::cout << "\"part:setVp\" failed" << std::endl;

	p.setWa(r, wa, rm);
	p.setPp(r, pp);
	p.setVp(r, vp);
	if (!(s_is_equal(6, to_vs, p.vs(), error) && s_is_equal(16, *to_pm, *p.pm(), error)))std::cout << "\"part:setVp relative\" failed" << std::endl;

	p.setVp(vp, pp);
	p.setWe(we313, re313);
	p.getVp(result);
	if (!(s_is_equal(3, vs + 3, p.vs() + 3, error)&& s_is_equal(3, vp, result, error) && s_is_equal(16, pm, *p.pm(), error)))std::cout << "\"part:setWe\" failed" << std::endl;

	p.setWe(r, we313, re313);
	p.getVp(result, result2);
	if (!(s_is_equal(3, to_vs + 3, p.vs() + 3, error) && s_is_equal(3, vp, result, error) && s_is_equal(3, 3, *to_pm, 4, *p.pm(), 4, error) && s_is_equal(3, pp, result2, error)))std::cout << "\"part:setWe relative\" failed" << std::endl;

	p.setRe(re313);
	p.setWe(we313);
	p.getVp(result, result2);
	if (!(s_is_equal(3, vs + 3, p.vs() + 3, error) && s_is_equal(3, vp, result, error) && s_is_equal(16, pm, *p.pm(), error)))std::cout << "\"part:setWe\" failed" << std::endl;

	p.setRe(r, re313);
	p.setWe(r, we313);
	p.getVp(result, result2);
	if (!(s_is_equal(3, to_vs + 3, p.vs() + 3, error) && s_is_equal(3, vp, result, error) && s_is_equal(3, 3, *to_pm, 4, *p.pm(), 4, error) && s_is_equal(3, pp, result2, error)))std::cout << "\"part:setWe relative\" failed" << std::endl;

	p.setWe(we321, re321, "321");
	p.getVp(result, result2);
	if (!(s_is_equal(3, vs + 3, p.vs() + 3, error) && s_is_equal(3, vp, result, error) && s_is_equal(16, pm, *p.pm(), error)))std::cout << "\"part:setWe 321\" failed" << std::endl;

	p.setWe(r, we321, re321, "321");
	p.getVp(result, result2);
	if (!(s_is_equal(3, to_vs + 3, p.vs() + 3, error) && s_is_equal(3, vp, result, error) && s_is_equal(3, 3, *to_pm, 4, *p.pm(), 4, error) && s_is_equal(3, pp, result2, error)))std::cout << "\"part:setWe 321 relative\" failed" << std::endl;

	p.setRe(re321, "321");
	p.setWe(we321, nullptr, "321");
	p.getVp(result, result2);
	if (!(s_is_equal(3, vs + 3, p.vs() + 3, error) && s_is_equal(3, vp, result, error) && s_is_equal(16, pm, *p.pm(), error)))std::cout << "\"part:setWe 321\" failed" << std::endl;

	p.setRe(r, re321, "321");
	p.setWe(r, we321, nullptr, "321");
	p.getVp(result, result2);
	if (!(s_is_equal(3, to_vs + 3, p.vs() + 3, error) && s_is_equal(3, vp, result, error) && s_is_equal(3, 3, *to_pm, 4, *p.pm(), 4, error) && s_is_equal(3, pp, result2, error)))std::cout << "\"part:setWe 321 relative\" failed" << std::endl;

	p.setWq(wq, rq);
	p.getVp(result, result2);
	if (!(s_is_equal(3, vs + 3, p.vs() + 3, error) && s_is_equal(3, vp, result, error) && s_is_equal(16, pm, *p.pm(), error)))std::cout << "\"part:setWq\" failed" << std::endl;

	p.setWq(r, wq, rq);
	p.getVp(result, result2);
	if (!(s_is_equal(3, to_vs + 3, p.vs() + 3, error) && s_is_equal(3, vp, result, error) && s_is_equal(3, 3, *to_pm, 4, *p.pm(), 4, error) && s_is_equal(3, pp, result2, error)))std::cout << "\"part:setWq relative\" failed" << std::endl;

	p.setRq(rq);
	p.setWq(wq);
	p.getVp(result, result2);
	if (!(s_is_equal(3, vs + 3, p.vs() + 3, error) && s_is_equal(3, vp, result, error) && s_is_equal(16, pm, *p.pm(), error)))std::cout << "\"part:setWq\" failed" << std::endl;

	p.setRq(r, rq);
	p.setWq(r, wq);
	p.getVp(result, result2);
	if (!(s_is_equal(3, to_vs + 3, p.vs() + 3, error) && s_is_equal(3, vp, result, error) && s_is_equal(3, 3, *to_pm, 4, *p.pm(), 4, error) && s_is_equal(3, pp, result2, error)))std::cout << "\"part:setWq relative\" failed" << std::endl;

	p.setWm(wm, rm);
	p.getVp(result, result2);
	if (!(s_is_equal(3, vs + 3, p.vs() + 3, error) && s_is_equal(3, vp, result, error) && s_is_equal(16, pm, *p.pm(), error)))std::cout << "\"part:setWm\" failed" << std::endl;

	p.setWm(r, wm, rm);
	p.getVp(result, result2);
	if (!(s_is_equal(3, to_vs + 3, p.vs() + 3, error) && s_is_equal(3, vp, result, error) && s_is_equal(3, 3, *to_pm, 4, *p.pm(), 4, error) && s_is_equal(3, pp, result2, error)))std::cout << "\"part:setWm relative\" failed" << std::endl;

	p.setRm(rm);
	p.setWm(wm);
	p.getVp(result, result2);
	if (!(s_is_equal(3, vs + 3, p.vs() + 3, error) && s_is_equal(3, vp, result, error) && s_is_equal(16, pm, *p.pm(), error)))std::cout << "\"part:setWm\" failed" << std::endl;

	p.setRm(r, rm);
	p.setWm(r, wm);
	p.getVp(result, result2);
	if (!(s_is_equal(3, to_vs + 3, p.vs() + 3, error) && s_is_equal(3, vp, result, error) && s_is_equal(3, 3, *to_pm, 4, *p.pm(), 4, error) && s_is_equal(3, pp, result2, error)))std::cout << "\"part:setWm relative\" failed" << std::endl;

	p.setWm(vm, pm, 4, 4);
	p.getVp(result, result2);
	if (!(s_is_equal(3, vs + 3, p.vs() + 3, error) && s_is_equal(3, vp, result, error) && s_is_equal(16, pm, *p.pm(), error)))std::cout << "\"part:setWm with ld\" failed" << std::endl;

	p.setWm(r, vm, pm, 4, 4);
	p.getVp(result, result2);
	if (!(s_is_equal(3, to_vs + 3, p.vs() + 3, error) && s_is_equal(3, vp, result, error) && s_is_equal(3, 3, *to_pm, 4, *p.pm(), 4, error) && s_is_equal(3, pp, result2, error)))std::cout << "\"part:setWm with ld relative\" failed" << std::endl;

	p.setRm(pm, 4);
	p.setWm(vm, nullptr, 4);
	p.getVp(result, result2);
	if (!(s_is_equal(3, vs + 3, p.vs() + 3, error) && s_is_equal(3, vp, result, error) && s_is_equal(16, pm, *p.pm(), error)))std::cout << "\"part:setWm with ld\" failed" << std::endl;

	p.setRm(r, pm, 4);
	p.setWm(r, vm, nullptr, 4);
	p.getVp(result, result2);
	if (!(s_is_equal(3, to_vs + 3, p.vs() + 3, error) && s_is_equal(3, vp, result, error) && s_is_equal(3, 3, *to_pm, 4, *p.pm(), 4, error) && s_is_equal(3, pp, result2, error)))std::cout << "\"part:setWm with ld relative\" failed" << std::endl;

	p.setWa(wa, rm);
	p.getVp(result, result2);
	if (!(s_is_equal(3, vs + 3, p.vs() + 3, error) && s_is_equal(3, vp, result, error) && s_is_equal(16, pm, *p.pm(), error)))std::cout << "\"part:setWm\" failed" << std::endl;

	p.setWa(r, wa, rm);
	p.getVp(result, result2);
	if (!(s_is_equal(3, to_vs + 3, p.vs() + 3, error) && s_is_equal(3, vp, result, error) && s_is_equal(3, 3, *to_pm, 4, *p.pm(), 4, error) && s_is_equal(3, pp, result2, error)))std::cout << "\"part:setWm relative\" failed" << std::endl;

	p.setRm(rm);
	p.setWa(wa);
	p.getVp(result, result2);
	if (!(s_is_equal(3, vs + 3, p.vs() + 3, error) && s_is_equal(3, vp, result, error) && s_is_equal(16, pm, *p.pm(), error)))std::cout << "\"part:setWm\" failed" << std::endl;

	p.setRm(r, rm);
	p.setWa(r, wa);
	p.getVp(result, result2);
	if (!(s_is_equal(3, to_vs + 3, p.vs() + 3, error) && s_is_equal(3, vp, result, error) && s_is_equal(3, 3, *to_pm, 4, *p.pm(), 4, error) && s_is_equal(3, pp, result2, error)))std::cout << "\"part:setWm relative\" failed" << std::endl;

	p.setWa(wa, pm, 4);
	p.getVp(result, result2);
	if (!(s_is_equal(3, vs + 3, p.vs() + 3, error) && s_is_equal(3, vp, result, error) && s_is_equal(16, pm, *p.pm(), error)))std::cout << "\"part:setWa with ld\" failed" << std::endl;

	p.setWa(r, wa, pm, 4);
	p.getVp(result, result2);
	if (!(s_is_equal(3, to_vs + 3, p.vs() + 3, error) && s_is_equal(3, vp, result, error) && s_is_equal(3, 3, *to_pm, 4, *p.pm(), 4, error) && s_is_equal(3, pp, result2, error)))std::cout << "\"part:setWa with ld relative\" failed" << std::endl;

	p.setRm(pm, 4);
	p.setWa(wa);
	p.getVp(result, result2);
	if (!(s_is_equal(3, vs + 3, p.vs() + 3, error) && s_is_equal(3, vp, result, error) && s_is_equal(16, pm, *p.pm(), error)))std::cout << "\"part:setWa with ld\" failed" << std::endl;

	p.setRm(r, pm, 4);
	p.setWa(r, wa);
	p.getVp(result, result2);
	if (!(s_is_equal(3, to_vs + 3, p.vs() + 3, error) && s_is_equal(3, vp, result, error) && s_is_equal(3, 3, *to_pm, 4, *p.pm(), 4, error) && s_is_equal(3, pp, result2, error)))std::cout << "\"part:setWa with ld relative\" failed" << std::endl;

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
	if (!(s_is_equal(6, vs, p.vs(), error) && s_is_equal(3, pp, 1, &p.pm()[0][3], 4, error)))std::cout << "\"part:setVa\" failed" << std::endl;

	p.setVa(r, va, pp);
	if (!(s_is_equal(6, to_vs, p.vs(), error) && s_is_equal(3, &to_pm[0][3], 4, &p.pm()[0][3], 4, error)))std::cout << "\"part:setVa relative\" failed" << std::endl;

	p.setPp(pp);
	p.setVa(va);
	if (!(s_is_equal(6, vs, p.vs(), error) && s_is_equal(3, pp, 1, &p.pm()[0][3], 4, error)))std::cout << "\"part:setVa\" failed" << std::endl;

	p.setPp(r, pp);
	p.setVa(r, va);
	if (!(s_is_equal(6, to_vs, p.vs(), error) && s_is_equal(3, &to_pm[0][3], 4, &p.pm()[0][3], 4, error)))std::cout << "\"part:setVa relative\" failed" << std::endl;

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


	p.setXa(xa, wa, rm);
	p.setAp(ap, vp, pp);
	if (!(s_is_equal(6, as, p.as(), error) && s_is_equal(6, vs, p.vs(), error) && s_is_equal(16, pm, *p.pm(), error)))std::cout << "\"part:setAp\" failed" << std::endl;

	p.setXa(r, xa, wa, rm);
	p.setAp(r, ap, vp, pp);
	if (!(s_is_equal(6, to_as, p.as(), error) && s_is_equal(6, to_vs, p.vs(), error) && s_is_equal(16, *to_pm, *p.pm(), error)))std::cout << "\"part:setAp relative\" failed" << std::endl;

	p.setXa(xa, wa, rm);
	p.setVp(vp, pp);
	p.setAp(ap);
	if (!(s_is_equal(6, as, p.as(), error) && s_is_equal(6, vs, p.vs(), error) && s_is_equal(16, pm, *p.pm(), error)))std::cout << "\"part:setAp\" failed" << std::endl;

	p.setXa(r, xa, wa, rm);
	p.setVp(r, vp, pp);
	p.setAp(r, ap);
	if (!(s_is_equal(6, to_as, p.as(), error) && s_is_equal(6, to_vs, p.vs(), error) && s_is_equal(16, *to_pm, *p.pm(), error)))std::cout << "\"part:setAp relative\" failed" << std::endl;

	p.setAp(ap, vp, pp);

	p.setXe(xe313, we313, re313);
	p.getAp(result, result2, result3);
	if (!(s_is_equal(3, as + 3, p.as() + 3, error) && s_is_equal(3, result, ap, error) && s_is_equal(3, vs + 3, p.vs() + 3, error) && s_is_equal(3, result2, vp, error)
		&& s_is_equal(3, 3, rm, 3, *p.pm(), 4, error) && s_is_equal(3, pp, result3, error)))std::cout << "\"part:setXe 313\" failed" << std::endl;

	p.setXe(r, xe313, we313, re313);
	p.getAp(result, result2, result3);
	if (!(s_is_equal(3, to_as + 3, p.as() + 3, error) && s_is_equal(3, result, ap, error) && s_is_equal(3, to_vs + 3, p.vs() + 3, error) && s_is_equal(3, result2, vp, error)
		&& s_is_equal(3, 3, *to_pm, 4, *p.pm(), 4, error) && s_is_equal(3, result3, pp, error)))std::cout << "\"part:setXe 313 relative\" failed" << std::endl;

	p.setWe(we313, re313);
	p.setXe(xe313, nullptr, nullptr);
	p.getAp(result, result2, result3);
	if (!(s_is_equal(3, as + 3, p.as() + 3, error) && s_is_equal(3, vs + 3, p.vs() + 3, error) && s_is_equal(3, result2, vp, error)
		&& s_is_equal(3, 3, rm, 3, *p.pm(), 4, error) && s_is_equal(3, pp, result3, error)))std::cout << "\"part:setXe 313\" failed" << std::endl;

	p.setWe(r, we313, re313);
	p.setXe(r, xe313, nullptr, nullptr);
	p.getAp(result, result2, result3);
	if (!(s_is_equal(3, to_as + 3, p.as() + 3, error) && s_is_equal(3, to_vs + 3, p.vs() + 3, error) && s_is_equal(3, result2, vp, error)
		&& s_is_equal(3, 3, *to_pm, 4, *p.pm(), 4, error) && s_is_equal(3, result3, pp, error)))std::cout << "\"part:setXe 313 relative\" failed" << std::endl;

	p.setXe(xe321, we321, re321, "321");
	p.getAp(result, result2, result3);
	if (!(s_is_equal(3, as + 3, p.as() + 3, error) && s_is_equal(3, result, ap, error) && s_is_equal(3, vs + 3, p.vs() + 3, error) && s_is_equal(3, result2, vp, error)
		&& s_is_equal(3, 3, rm, 3, *p.pm(), 4, error) && s_is_equal(3, pp, result3, error)))std::cout << "\"part:setXe 321\" failed" << std::endl;

	p.setXe(r, xe321, we321, re321, "321");
	p.getAp(result, result2, result3);
	if (!(s_is_equal(3, to_as + 3, p.as() + 3, error) && s_is_equal(3, result, ap, error) && s_is_equal(3, to_vs + 3, p.vs() + 3, error) && s_is_equal(3, result2, vp, error)
		&& s_is_equal(3, 3, *to_pm, 4, *p.pm(), 4, error) && s_is_equal(3, result3, pp, error)))std::cout << "\"part:setXe 321 relative\" failed" << std::endl;

	p.setWe(we321, re321, "321");
	p.setXe(xe321, nullptr, nullptr, "321");
	p.getAp(result, result2, result3);
	if (!(s_is_equal(3, as + 3, p.as() + 3, error) && s_is_equal(3, vs + 3, p.vs() + 3, error) && s_is_equal(3, result2, vp, error)
		&& s_is_equal(3, 3, rm, 3, *p.pm(), 4, error) && s_is_equal(3, pp, result3, error)))std::cout << "\"part:setXe 321\" failed" << std::endl;

	p.setWe(r, we321, re321, "321");
	p.setXe(r, xe321, nullptr, nullptr, "321");
	p.getAp(result, result2, result3);
	if (!(s_is_equal(3, to_as + 3, p.as() + 3, error) && s_is_equal(3, to_vs + 3, p.vs() + 3, error) && s_is_equal(3, result2, vp, error)
		&& s_is_equal(3, 3, *to_pm, 4, *p.pm(), 4, error) && s_is_equal(3, result3, pp, error)))std::cout << "\"part:setXe 321 relative\" failed" << std::endl;

	p.setXq(xq, wq, rq);
	p.getAp(result, result2, result3);
	if (!(s_is_equal(3, as + 3, p.as() + 3, error) && s_is_equal(3, result, ap, error) && s_is_equal(3, vs + 3, p.vs() + 3, error) && s_is_equal(3, result2, vp, error)
		&& s_is_equal(3, 3, rm, 3, *p.pm(), 4, error) && s_is_equal(3, pp, result3, error)))std::cout << "\"part:setXq\" failed" << std::endl;

	p.setXq(r, xq, wq, rq);
	p.getAp(result, result2, result3);
	if (!(s_is_equal(3, to_as + 3, p.as() + 3, error) && s_is_equal(3, result, ap, error) && s_is_equal(3, to_vs + 3, p.vs() + 3, error) && s_is_equal(3, result2, vp, error)
		&& s_is_equal(3, 3, *to_pm, 4, *p.pm(), 4, error) && s_is_equal(3, result3, pp, error)))std::cout << "\"part:setXq relative\" failed" << std::endl;

	p.setWq(wq, rq);
	p.setXq(xq);
	p.getAp(result, result2, result3);
	if (!(s_is_equal(3, as + 3, p.as() + 3, error) && s_is_equal(3, vs + 3, p.vs() + 3, error) && s_is_equal(3, result2, vp, error)
		&& s_is_equal(3, 3, rm, 3, *p.pm(), 4, error) && s_is_equal(3, pp, result3, error)))std::cout << "\"part:setXq\" failed" << std::endl;

	p.setWq(r, wq, rq);
	p.setXq(r, xq);
	p.getAp(result, result2, result3);
	if (!(s_is_equal(3, to_as + 3, p.as() + 3, error) && s_is_equal(3, to_vs + 3, p.vs() + 3, error) && s_is_equal(3, result2, vp, error)
		&& s_is_equal(3, 3, *to_pm, 4, *p.pm(), 4, error) && s_is_equal(3, result3, pp, error)))std::cout << "\"part:setXq relative\" failed" << std::endl;

	p.setXm(xm, wm, rm);
	p.getAp(result, result2, result3);
	if (!(s_is_equal(3, as + 3, p.as() + 3, error) && s_is_equal(3, result, ap, error) && s_is_equal(3, vs + 3, p.vs() + 3, error) && s_is_equal(3, result2, vp, error)
		&& s_is_equal(3, 3, rm, 3, *p.pm(), 4, error) && s_is_equal(3, pp, result3, error)))std::cout << "\"part:setXm\" failed" << std::endl;

	p.setXm(r, xm, wm, rm);
	p.getAp(result, result2, result3);
	if (!(s_is_equal(3, to_as + 3, p.as() + 3, error) && s_is_equal(3, result, ap, error) && s_is_equal(3, to_vs + 3, p.vs() + 3, error) && s_is_equal(3, result2, vp, error)
		&& s_is_equal(3, 3, *to_pm, 4, *p.pm(), 4, error) && s_is_equal(3, result3, pp, error)))std::cout << "\"part:setXm relative\" failed" << std::endl;

	p.setWm(wm, rm);
	p.setXm(xm);
	p.getAp(result, result2, result3);
	if (!(s_is_equal(3, as + 3, p.as() + 3, error) && s_is_equal(3, vs + 3, p.vs() + 3, error) && s_is_equal(3, result2, vp, error)
		&& s_is_equal(3, 3, rm, 3, *p.pm(), 4, error) && s_is_equal(3, pp, result3, error)))std::cout << "\"part:setXm\" failed" << std::endl;

	p.setWm(r, wm, rm);
	p.setXm(r, xm);
	p.getAp(result, result2, result3);
	if (!(s_is_equal(3, to_as + 3, p.as() + 3, error) && s_is_equal(3, to_vs + 3, p.vs() + 3, error) && s_is_equal(3, result2, vp, error)
		&& s_is_equal(3, 3, *to_pm, 4, *p.pm(), 4, error) && s_is_equal(3, result3, pp, error)))std::cout << "\"part:setXm relative\" failed" << std::endl;

	p.setXa(xa, wa, rm);
	p.getAp(result, result2, result3);
	if (!(s_is_equal(3, as + 3, p.as() + 3, error) && s_is_equal(3, result, ap, error) && s_is_equal(3, vs + 3, p.vs() + 3, error) && s_is_equal(3, result2, vp, error)
		&& s_is_equal(3, 3, rm, 3, *p.pm(), 4, error) && s_is_equal(3, pp, result3, error)))std::cout << "\"part:setXa\" failed" << std::endl;

	p.setXa(r, xa, wa, rm);
	p.getAp(result, result2, result3);
	if (!(s_is_equal(3, to_as + 3, p.as() + 3, error) && s_is_equal(3, result, ap, error) && s_is_equal(3, to_vs + 3, p.vs() + 3, error) && s_is_equal(3, result2, vp, error)
		&& s_is_equal(3, 3, *to_pm, 4, *p.pm(), 4, error) && s_is_equal(3, result3, pp, error)))std::cout << "\"part:setXa relative\" failed" << std::endl;

	p.setWa(wa, rm);
	p.setXa(xa);
	p.getAp(result, result2, result3);
	if (!(s_is_equal(3, as + 3, p.as() + 3, error) && s_is_equal(3, vs + 3, p.vs() + 3, error) && s_is_equal(3, result2, vp, error)
		&& s_is_equal(3, 3, rm, 3, *p.pm(), 4, error) && s_is_equal(3, pp, result3, error)))std::cout << "\"part:setXa\" failed" << std::endl;

	p.setWa(r, wa, rm);
	p.setXa(r, xa);
	p.getAp(result, result2, result3);
	if (!(s_is_equal(3, to_as + 3, p.as() + 3, error) && s_is_equal(3, to_vs + 3, p.vs() + 3, error) && s_is_equal(3, result2, vp, error)
		&& s_is_equal(3, 3, *to_pm, 4, *p.pm(), 4, error) && s_is_equal(3, result3, pp, error)))std::cout << "\"part:setXa relative\" failed" << std::endl;



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


	r.setPm(relative_pm);
	r.setVs(relative_vs);
	r.setAs(relative_as);

	p.setPm(pm);
	p.setVs(vs);
	p.setAs(as);

	p.getPp(result);
	if (!(s_is_equal(3, result, pp, error)))std::cout << "\"coordinate:getPp\" failed" << std::endl;

	p.getRe(result);
	if (!(s_is_equal(3, result, re313, error)))std::cout << "\"coordinate:getRe 313\" failed" << std::endl;

	p.getRe(result, "321");
	if (!(s_is_equal(3, result, re321, error)))std::cout << "\"coordinate:getRe 321\" failed" << std::endl;

	p.getRq(result);
	if (!(s_is_equal(3, result, rq, error)))std::cout << "\"coordinate:getRq\" failed" << std::endl;

	p.getRm(result);
	if (!(s_is_equal(9, result, rm, error)))std::cout << "\"coordinate:getRm\" failed" << std::endl;

	p.getPe(result);
	if (!(s_is_equal(6, result, pe313, error)))std::cout << "\"coordinate:getPe\" failed" << std::endl;

	p.getPe(result, "321");
	if (!(s_is_equal(6, result, pe321, error)))std::cout << "\"coordinate:getPe 321\" failed" << std::endl;

	p.getPq(result);
	if (!(s_is_equal(7, result, pq, error)))std::cout << "\"coordinate:getPq\" failed" << std::endl;

	p.getPm(result);
	if (!(s_is_equal(16, result, pm, error)))std::cout << "\"coordinate:getPm\" failed" << std::endl;

	p.getVp(result, result2);
	if (!(s_is_equal(3, result, vp, error) && s_is_equal(3, result2, pp, error)))std::cout << "\"coordinate:getVp\" failed" << std::endl;

	p.getWe(result, result2);
	if (!(s_is_equal(3, result, we313, error) && s_is_equal(3, result2, re313, error)))std::cout << "\"coordinate:getWe 313\" failed" << std::endl;

	p.getWe(result, result2, "321");
	if (!(s_is_equal(3, result, we321, error) && s_is_equal(3, result2, re321, error)))std::cout << "\"coordinate:getWe 321\" failed" << std::endl;

	p.getWq(result, result2);
	if (!(s_is_equal(4, result, wq, error) && s_is_equal(4, result2, rq, error)))std::cout << "\"coordinate:getWq\" failed" << std::endl;

	p.getWm(result, result2);
	if (!(s_is_equal(9, result, wm, error) && s_is_equal(9, result2, rm, error)))std::cout << "\"coordinate:getWm\" failed" << std::endl;

	p.getWa(result, result3);
	if (!(s_is_equal(3, result, wa, error) && s_is_equal(9, result3, rm, error)))std::cout << "\"coordinate:getWa\" failed" << std::endl;

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

	p.getAp(result, result2, result3);
	if (!(s_is_equal(3, result, ap, error) && s_is_equal(3, result2, vp, error) && s_is_equal(3, result3, pp, error)))std::cout << "\"coordinate:getAp\" failed" << std::endl;

	p.getXe(result, result2, result3);
	if (!(s_is_equal(3, result, xe313, error) && s_is_equal(3, result2, we313, error) && s_is_equal(3, result3, re313, error)))std::cout << "\"coordinate:getXe 313\" failed" << std::endl;

	p.getXe(result, result2, result3, "321");
	if (!(s_is_equal(3, result, xe321, error) && s_is_equal(3, result2, we321, error) && s_is_equal(3, result3, re321, error)))std::cout << "\"coordinate:getXe 313\" failed" << std::endl;

	p.getXq(result, result2, result3);
	if (!(s_is_equal(4, result, xq, error) && s_is_equal(4, result2, wq, error) && s_is_equal(4, result3, rq, error)))std::cout << "\"coordinate:getXq\" failed" << std::endl;

	p.getXm(result, result2, result3);
	if (!(s_is_equal(9, result, xm, error) && s_is_equal(9, result2, wm, error) && s_is_equal(9, result3, rm, error)))std::cout << "\"coordinate:getXm\" failed" << std::endl;

	p.getXa(result, result2, result3);
	if (!(s_is_equal(3, result, xa, error) && s_is_equal(3, result2, wa, error) && s_is_equal(9, result3, rm, error)))std::cout << "\"coordinate:getXm\" failed" << std::endl;

	p.getAe(result, result2, result3);
	if (!(s_is_equal(6, result, ae313, error) && s_is_equal(6, result2, ve313, error) && s_is_equal(6, result3, pe313, error)))std::cout << "\"coordinate:getAe 313\" failed" << std::endl;

	p.getAe(result, result2, result3, "321");
	if (!(s_is_equal(6, result, ae321, error) && s_is_equal(6, result2, ve321, error) && s_is_equal(6, result3, pe321, error)))std::cout << "\"coordinate:getAe 321\" failed" << std::endl;

	p.getAq(result, result2, result3);
	if (!(s_is_equal(7, result, aq, error) && s_is_equal(7, result2, vq, error) && s_is_equal(7, result3, pq, error)))std::cout << "\"coordinate:getAq\" failed" << std::endl;

	p.getAm(result, result2, result3);
	if (!(s_is_equal(16, result, am, error) && s_is_equal(16, result2, vm, error) && s_is_equal(16, result3, pm, error)))std::cout << "\"coordinate:getAm\" failed" << std::endl;

	p.getAa(result, result2, result3);
	if (!(s_is_equal(6, result, aa, error) && s_is_equal(3, result2, va, error) && s_is_equal(3, result3, pp, error)))std::cout << "\"coordinate:getAa\" failed" << std::endl;

	p.getAs(result, result2, result3);
	if (!(s_is_equal(6, result, as, error) && s_is_equal(6, result2, vs, error) && s_is_equal(16, result3, pm, error)))std::cout << "\"coordinate:getAs\" failed" << std::endl;

	p.setPm(r, pm);
	p.setVs(r, vs);
	p.setAs(r, as);

	p.getPp(r, result);
	if (!(s_is_equal(3, result, pp, error)))std::cout << "\"coordinate:getPp\" failed" << std::endl;

	p.getRe(r, result);
	if (!(s_is_equal(3, result, re313, error)))std::cout << "\"coordinate:getRe\" failed" << std::endl;

	p.getRe(r, result, "321");
	if (!(s_is_equal(3, result, re321, error)))std::cout << "\"coordinate:getRe 321\" failed" << std::endl;

	p.getRq(r, result);
	if (!(s_is_equal(4, result, rq, error)))std::cout << "\"coordinate:getRq\" failed" << std::endl;

	p.getRm(r, result);
	if (!(s_is_equal(9, result, rm, error)))std::cout << "\"coordinate:getRm\" failed" << std::endl;

	p.getRm(r, result, 4);
	if (!(s_is_equal(3, 3, result, 4, rm, 3, error)))std::cout << "\"coordinate:getRm\" failed" << std::endl;

	p.getPe(r, result);
	if (!(s_is_equal(6, result, pe313, error)))std::cout << "\"coordinate:getPe\" failed" << std::endl;

	p.getPe(r, result, "321");
	if (!(s_is_equal(6, result, pe321, error)))std::cout << "\"coordinate:getPe 321\" failed" << std::endl;

	p.getPq(r, result);
	if (!(s_is_equal(7, result, pq, error)))std::cout << "\"coordinate:getPq\" failed" << std::endl;

	p.getPm(r, result);
	if (!(s_is_equal(16, result, pm, error)))std::cout << "\"coordinate:getPm\" failed" << std::endl;

	p.getVp(r, result, result2);
	if (!(s_is_equal(3, result, vp, error) && s_is_equal(3, result2, pp, error)))std::cout << "\"coordinate:getVp\" failed" << std::endl;

	p.getWe(r, result, result2);
	if (!(s_is_equal(3, result, we313, error) && s_is_equal(3, result2, re313, error)))std::cout << "\"coordinate:getWe 313\" failed" << std::endl;

	p.getWe(r, result, result2, "321");
	if (!(s_is_equal(3, result, we321, error) && s_is_equal(3, result2, re321, error)))std::cout << "\"coordinate:getWe 321\" failed" << std::endl;

	p.getWq(r, result, result2);
	if (!(s_is_equal(4, result, wq, error) && s_is_equal(4, result2, rq, error)))std::cout << "\"coordinate:getWq\" failed" << std::endl;

	p.getWm(r, result, result2);
	if (!(s_is_equal(9, result, wm, error) && s_is_equal(9, result2, rm, error)))std::cout << "\"coordinate:getWm\" failed" << std::endl;

	p.getWm(r, result, result2, 4, 4);
	if (!(s_is_equal(3, 3, result, 4, wm, 3, error) && s_is_equal(3, 3, result2, 4, rm, 3, error)))std::cout << "\"coordinate:getWm\" failed" << std::endl;

	p.getWa(r, result, result2);
	if (!(s_is_equal(3, result, wa, error) && s_is_equal(9, result2, rm, error)))std::cout << "\"coordinate:getWa\" failed" << std::endl;

	p.getWa(r, result, result2, 4);
	if (!(s_is_equal(3, result, wa, error) && s_is_equal(3, 3, result2, 4, rm, 3, error)))std::cout << "\"coordinate:getWa\" failed" << std::endl;

	p.getVe(r, result, result2);
	if (!(s_is_equal(6, result, ve313, error) && s_is_equal(6, result2, pe313, error)))std::cout << "\"coordinate:getVe 313\" failed" << std::endl;

	p.getVe(r, result, result2, "321");
	if (!(s_is_equal(6, result, ve321, error) && s_is_equal(6, result2, pe321, error)))std::cout << "\"coordinate:getVe 321\" failed" << std::endl;

	p.getVq(r, result, result2);
	if (!(s_is_equal(7, result, vq, error) && s_is_equal(7, result2, pq, error)))std::cout << "\"coordinate:getVq\" failed" << std::endl;

	p.getVm(r, result, result2);
	if (!(s_is_equal(16, result, vm, error) && s_is_equal(16, result2, pm, error)))std::cout << "\"coordinate:getVm\" failed" << std::endl;

	p.getVa(r, result, result2);
	if (!(s_is_equal(6, result, va, error) && s_is_equal(3, result2, pp, error)))std::cout << "\"coordinate:getVa\" failed" << std::endl;

	p.getVs(r, result, result2);
	if (!(s_is_equal(6, result, vs, error) && s_is_equal(16, result2, pm, error)))std::cout << "\"coordinate:getVs\" failed" << std::endl;

	p.getAp(r, result, result2, result3);
	if (!(s_is_equal(3, result, ap, error) && s_is_equal(3, result2, vp, error) && s_is_equal(3, result3, pp, error)))std::cout << "\"coordinate:getAp\" failed" << std::endl;

	p.getXe(r, result, result2, result3);
	if (!(s_is_equal(3, result, xe313, error) && s_is_equal(3, result2, we313, error) && s_is_equal(3, result3, re313, error)))std::cout << "\"coordinate:getXe 313\" failed" << std::endl;

	p.getXe(r, result, result2, result3, "321");
	if (!(s_is_equal(3, result, xe321, error) && s_is_equal(3, result2, we321, error) && s_is_equal(3, result3, re321, error)))std::cout << "\"coordinate:getXe 321\" failed" << std::endl;

	p.getXq(r, result, result2, result3);
	if (!(s_is_equal(4, result, xq, error) && s_is_equal(4, result2, wq, error) && s_is_equal(4, result3, rq, error)))std::cout << "\"coordinate:getXq\" failed" << std::endl;

	p.getXm(r, result, result2, result3);
	if (!(s_is_equal(9, result, xm, error) && s_is_equal(9, result2, wm, error) && s_is_equal(9, result3, rm, error)))std::cout << "\"coordinate:getXm\" failed" << std::endl;

	p.getXa(r, result, result2, result3);
	if (!(s_is_equal(3, result, xa, error) && s_is_equal(3, result2, wa, error) && s_is_equal(9, result3, rm, error)))std::cout << "\"coordinate:getXa\" failed" << std::endl;

	p.getAe(r, result, result2, result3);
	if (!(s_is_equal(6, result, ae313, error) && s_is_equal(6, result2, ve313, error) && s_is_equal(6, result3, pe313, error)))std::cout << "\"coordinate:getAe 313\" failed" << std::endl;

	p.getAe(r, result, result2, result3, "321");
	if (!(s_is_equal(6, result, ae321, error) && s_is_equal(6, result2, ve321, error) && s_is_equal(6, result3, pe321, error)))std::cout << "\"coordinate:getAe 321\" failed" << std::endl;

	p.getAq(r, result, result2, result3);
	if (!(s_is_equal(7, result, aq, error) && s_is_equal(7, result2, vq, error) && s_is_equal(7, result3, pq, error)))std::cout << "\"coordinate:getAq\" failed" << std::endl;

	p.getAm(r, result, result2, result3);
	if (!(s_is_equal(16, result, am, error) && s_is_equal(16, result2, vm, error) && s_is_equal(16, result3, pm, error)))std::cout << "\"coordinate:getAm\" failed" << std::endl;

	p.getAa(r, result, result2, result3);
	if (!(s_is_equal(6, result, aa, error) && s_is_equal(6, result2, va, error) && s_is_equal(3, result3, pp, error)))std::cout << "\"coordinate:getAa\" failed" << std::endl;

	p.getAs(r, result, result2, result3);
	if (!(s_is_equal(6, result, as, error) && s_is_equal(6, result2, vs, error) && s_is_equal(16, result3, pm, error)))std::cout << "\"coordinate:getAs\" failed" << std::endl;



	const double im[36]{ 12.2999999999999954,0.0000000000000095,0.0000000000000008,-0.0000000000000028,3.8018392163046553,-2.1184526964958033,
		0.0000000000000094,12.2999999999999901,-0.0000000000000027,-3.8018392163046530,0.0000000000000031,0.9908070461627771,
		0.0000000000000008,-0.0000000000000031,12.3000000000000203,2.1184526964958104,-0.9908070461627823,-0.0000000000000005,
		-0.0000000000000027,-3.8018392163046539,2.1184526964958099,8.2643465287835660,0.8248630422298895,-1.1387229998146720,
		3.8018392163046548,0.0000000000000031,-0.9908070461627819,0.8248630422298896,5.0180208478483292,-0.8639469278395188,
		-2.1184526964958033,0.9908070461627773,-0.0000000000000004,-1.1387229998146717,-0.8639469278395188,6.0269246585954530 };
	const double glb_im[36]{ 12.3, 0, 0,   0,   4.86680056754638, 1.19059364505643,
		0,   12.3, 0, -4.86680056754639, 0, -1.05294290764011,
		0, 0,   12.3, -1.19059364505643,1.05294290764011, 0,
		0, -4.86680056754639, -1.19059364505643, 6.37494518313694, -0.759574604730052, -1.02350564428774,
		4.86680056754638, 0,   1.05294290764011, -0.759574604730052,   7.88731016704578,   1.28815826950323,
		1.19059364505643, -1.05294290764011, 0, -1.02350564428774, 1.28815826950323,   6.06954528699909 };
	const double fg[6]{ -21.1547438184137988,4.8215999999999539,118.5711625456093685,18.9314187850450608, - 16.0900876407826203,4.0319186454939082 };
	const double glb_fg[6]{ 0, -120.54,0,47.6946455619546,0,10.3188404948731 };
	const double prt_fg[6]{ 119.279123323887,-5.207328,16.5911444507421,0.45124518305897,3.31741898034387,-2.20293670541155 };
	const double fv[6]{ 11.6496106078792767, - 7.8762514847338121, - 8.5204704046205890, - 0.7076559679417713,2.8229614995915187, - 2.8804116565669942 };
	const double glb_fv[6]{ 5.11681554065846,10.7408556783149,11.3489822760672, -3.55177495791515,2.99639902169271,-1.75750746281364 };
	const double prt_fv[6]{ -12.121294608774,-9.20587956029489,6.21877634351408,0.260711372733768,1.34261769787459,1.5411918276253 };
	const double pf[6]{ 32.8043544262930737, -12.6978514847337607, -127.0916329502299504, -19.6390747529868364, 18.9130491403741381, -6.9123303020609024 };
	const double glb_pf[6]{ 5.1168155406584122, 131.2808556783149641, 11.3489822760671935, -51.2464205198697371, 2.9963990216926910, -12.0763479576867141};
	const double prt_pf[6]{ -131.4004179326610142, -3.9985515602948922, -10.3723681072279739, -0.1905338103252023, -1.9748012824692744, 3.7441285330368426 };
	const double prt_vs[6]{ -0.342374318815878, - 0.326839225000394, - 1.02664731827659, - 0.981437055212506,0.670440145936023, - 0.920485982149056 };
	const double prt_as[6]{ -1.91765344470424, - 2.61057756494702,0.488627252217732,0.786294961274511, - 2.28837157363616,1.03841805459299 };


	p.cptIm(r, result, 7);
	if (!(s_is_equal(6, 6, result, 7, im, 6, error)))std::cout << "\"part:cptIm\" failed" << std::endl;

	p.cptGlbIm(result, 7);
	if (!(s_is_equal(6, 6, result, 7, glb_im, 6, error)))std::cout << "\"part:cptGlbIm\" failed" << std::endl;

	p.cptPrtIm(result, 7);
	if (!(s_is_equal(6, 6, result, 7, prt_im, 6, error)))std::cout << "\"part:cptGlbIm\" failed" << std::endl;

	p.cptFg(r, result);
	if (!(s_is_equal(6, result, fg, error)))std::cout << "\"part:cptFg\" failed" << std::endl;

	p.cptGlbFg(result);
	if (!(s_is_equal(6, result, glb_fg, error)))std::cout << "\"part:cptGlbFg\" failed" << std::endl;

	p.cptPrtFg(result);
	if (!(s_is_equal(6, result, prt_fg, error)))std::cout << "\"part:cptPrtFg\" failed" << std::endl;

	p.cptFv(r, result);
	if (!(s_is_equal(6, result, fv, error)))std::cout << "\"part:cptFv\" failed" << std::endl;

	p.cptGlbFv(result);
	if (!(s_is_equal(6, result, glb_fv, error)))std::cout << "\"part:cptGlbFv\" failed" << std::endl;

	p.cptPrtFv(result);
	if (!(s_is_equal(6, result, prt_fv, error)))std::cout << "\"part:cptPrtFv\" failed" << std::endl;

	p.cptPf(r, result);
	if (!(s_is_equal(6, result, pf, error)))std::cout << "\"part:cptPf\" failed" << std::endl;

	p.cptGlbPf(result);
	if (!(s_is_equal(6, result, glb_pf, error)))std::cout << "\"part:cptGlbPf\" failed" << std::endl;

	p.cptPrtPf(result);
	if (!(s_is_equal(6, result, prt_pf, error)))std::cout << "\"part:cptPrtPf\" failed" << std::endl;

	p.cptPrtVs(result);
	if (!(s_is_equal(6, result, prt_vs, error)))std::cout << "\"part:cptPrtVs\" failed" << std::endl;

	p.cptPrtAs(result);
	if (!(s_is_equal(6, result, prt_as, error)))std::cout << "\"part:cptPrtAs\" failed" << std::endl;
}

void test_model_coordinate()
{
	std::cout << std::endl << "-----------------test model coordinate---------------------" << std::endl;
	test_part();
	std::cout << "-----------------test model coordinate finished------------" << std::endl << std::endl;
}

