#include <gtest/gtest.h>

#include <aris/dynamic/dynamic.hpp>

#include <type_traits>

using namespace aris::dynamic;

namespace {

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
	EXPECT_TRUE((s_is_equal(3, &pm[3], 4, &p.pm()[0][3], 4, error))) << "\"part:setPp\" failed";

	p.setPp(r, pp);
	EXPECT_TRUE((s_is_equal(3, &to_pm[0][3], 4, &p.pm()[0][3], 4, error))) << "\"part:setPp relative\" failed";

	double pp_tem[3]{ 0.4,0.5,0.6 }; p.setPp(pp_tem);
	p.setRe(re313);
	EXPECT_TRUE((s_is_equal(3, 3, pm, 4, &p.pm()[0][0], 4, error)&& s_is_equal(3, pp_tem, 1, &p.pm()[0][3], 4, error))) << "\"part:setRe\" failed";

	p.setRe(r, re313);
	EXPECT_TRUE((s_is_equal(3, 3, *to_pm, 4, &p.pm()[0][0], 4, error) && s_is_equal(3, pp_tem, 1, &p.pm()[0][3], 4, error))) << "\"part:setRe relative\" failed";

	p.setRe(re321, "321");
	EXPECT_TRUE((s_is_equal(3, 3, pm, 4, &p.pm()[0][0], 4, error) && s_is_equal(3, pp_tem, 1, &p.pm()[0][3], 4, error))) << "\"part:setRe 321\" failed";

	p.setRe(r, re321, "321");
	EXPECT_TRUE((s_is_equal(3, 3, *to_pm, 4, &p.pm()[0][0], 4, error) && s_is_equal(3, pp_tem, 1, &p.pm()[0][3], 4, error))) << "\"part:setRe 321 relative\" failed";

	p.setRq(rq);
	EXPECT_TRUE((s_is_equal(3, 3, pm, 4, &p.pm()[0][0], 4, error) && s_is_equal(3, pp_tem, 1, &p.pm()[0][3], 4, error))) << "\"part:setRq\" failed";

	p.setRq(r, rq);
	EXPECT_TRUE((s_is_equal(3, 3, *to_pm, 4, &p.pm()[0][0], 4, error) && s_is_equal(3, pp_tem, 1, &p.pm()[0][3], 4, error))) << "\"part:setRq relative\" failed";

	p.setRm(rm);
	EXPECT_TRUE((s_is_equal(3, 3, pm, 4, &p.pm()[0][0], 4, error) && s_is_equal(3, pp_tem, 1, &p.pm()[0][3], 4, error))) << "\"part:setRm\" failed";

	p.setRm(r, rm);
	EXPECT_TRUE((s_is_equal(3, 3, *to_pm, 4, &p.pm()[0][0], 4, error) && s_is_equal(3, pp_tem, 1, &p.pm()[0][3], 4, error))) << "\"part:setRm relative\" failed";

	p.setRm(pm, 4);
	EXPECT_TRUE((s_is_equal(3, 3, pm, 4, &p.pm()[0][0], 4, error) && s_is_equal(3, pp_tem, 1, &p.pm()[0][3], 4, error))) << "\"part:setRm with ld\" failed";

	p.setRm(r, pm, 4);
	EXPECT_TRUE((s_is_equal(3, 3, *to_pm, 4, &p.pm()[0][0], 4, error) && s_is_equal(3, pp_tem, 1, &p.pm()[0][3], 4, error))) << "\"part:setRm with ld relative\" failed";
	
	p.setPe(pe313);
	EXPECT_TRUE((s_is_equal(16, pm, &p.pm()[0][0], error))) << "\"part:setPe\" failed";

	p.setPe(r, pe313);
	EXPECT_TRUE((s_is_equal(16, &to_pm[0][0], &p.pm()[0][0], error))) << "\"part:setPe relative\" failed";

	p.setPe(pe321, "321");
	EXPECT_TRUE((s_is_equal(16, pm, &p.pm()[0][0], error))) << "\"part:setPe\" failed";

	p.setPe(r, pe321, "321");
	EXPECT_TRUE((s_is_equal(16, &to_pm[0][0], &p.pm()[0][0], error))) << "\"part:setPe 321 relative\" failed";

	p.setPq(pq);
	EXPECT_TRUE((s_is_equal(16, pm, &p.pm()[0][0], error))) << "\"part:setPq\" failed";

	p.setPq(r, pq);
	EXPECT_TRUE((s_is_equal(16, &to_pm[0][0], &p.pm()[0][0], error))) << "\"part:setPq relative\" failed";

	p.setPm(pm);
	EXPECT_TRUE((s_is_equal(16, pm, &p.pm()[0][0], error))) << "\"part:setPm\" failed";

	p.setPm(r, pm);
	EXPECT_TRUE((s_is_equal(16, &to_pm[0][0], &p.pm()[0][0], error))) << "\"part:setPm relative\" failed";

	p.setWa(wa, rm);
	p.setVp(vp, pp);
	EXPECT_TRUE((s_is_equal(6, vs, p.vs(), error) && s_is_equal(16, pm, *p.pm(), error))) << "\"part:setVp\" failed";

	p.setWa(r, wa, rm);
	p.setVp(r, vp, pp);
	EXPECT_TRUE((s_is_equal(6, to_vs, p.vs(), error) && s_is_equal(16, *to_pm, *p.pm(), error))) << "\"part:setVp relative\" failed";

	p.setWa(wa, rm);
	p.setPp(pp);
	p.setVp(vp);
	EXPECT_TRUE((s_is_equal(6, vs, p.vs(), error) && s_is_equal(16, pm, *p.pm(), error))) << "\"part:setVp\" failed";

	p.setWa(r, wa, rm);
	p.setPp(r, pp);
	p.setVp(r, vp);
	EXPECT_TRUE((s_is_equal(6, to_vs, p.vs(), error) && s_is_equal(16, *to_pm, *p.pm(), error))) << "\"part:setVp relative\" failed";

	p.setVp(vp, pp);
	p.setWe(we313, re313);
	p.getVp(result);
	EXPECT_TRUE((s_is_equal(3, vs + 3, p.vs() + 3, error)&& s_is_equal(3, vp, result, error) && s_is_equal(16, pm, *p.pm(), error))) << "\"part:setWe\" failed";

	p.setWe(r, we313, re313);
	p.getVp(result, result2);
	EXPECT_TRUE((s_is_equal(3, to_vs + 3, p.vs() + 3, error) && s_is_equal(3, vp, result, error) && s_is_equal(3, 3, *to_pm, 4, *p.pm(), 4, error) && s_is_equal(3, pp, result2, error))) << "\"part:setWe relative\" failed";

	p.setRe(re313);
	p.setWe(we313);
	p.getVp(result, result2);
	EXPECT_TRUE((s_is_equal(3, vs + 3, p.vs() + 3, error) && s_is_equal(3, vp, result, error) && s_is_equal(16, pm, *p.pm(), error))) << "\"part:setWe\" failed";

	p.setRe(r, re313);
	p.setWe(r, we313);
	p.getVp(result, result2);
	EXPECT_TRUE((s_is_equal(3, to_vs + 3, p.vs() + 3, error) && s_is_equal(3, vp, result, error) && s_is_equal(3, 3, *to_pm, 4, *p.pm(), 4, error) && s_is_equal(3, pp, result2, error))) << "\"part:setWe relative\" failed";

	p.setWe(we321, re321, "321");
	p.getVp(result, result2);
	EXPECT_TRUE((s_is_equal(3, vs + 3, p.vs() + 3, error) && s_is_equal(3, vp, result, error) && s_is_equal(16, pm, *p.pm(), error))) << "\"part:setWe 321\" failed";

	p.setWe(r, we321, re321, "321");
	p.getVp(result, result2);
	EXPECT_TRUE((s_is_equal(3, to_vs + 3, p.vs() + 3, error) && s_is_equal(3, vp, result, error) && s_is_equal(3, 3, *to_pm, 4, *p.pm(), 4, error) && s_is_equal(3, pp, result2, error))) << "\"part:setWe 321 relative\" failed";

	p.setRe(re321, "321");
	p.setWe(we321, nullptr, "321");
	p.getVp(result, result2);
	EXPECT_TRUE((s_is_equal(3, vs + 3, p.vs() + 3, error) && s_is_equal(3, vp, result, error) && s_is_equal(16, pm, *p.pm(), error))) << "\"part:setWe 321\" failed";

	p.setRe(r, re321, "321");
	p.setWe(r, we321, nullptr, "321");
	p.getVp(result, result2);
	EXPECT_TRUE((s_is_equal(3, to_vs + 3, p.vs() + 3, error) && s_is_equal(3, vp, result, error) && s_is_equal(3, 3, *to_pm, 4, *p.pm(), 4, error) && s_is_equal(3, pp, result2, error))) << "\"part:setWe 321 relative\" failed";

	p.setWq(wq, rq);
	p.getVp(result, result2);
	EXPECT_TRUE((s_is_equal(3, vs + 3, p.vs() + 3, error) && s_is_equal(3, vp, result, error) && s_is_equal(16, pm, *p.pm(), error))) << "\"part:setWq\" failed";

	p.setWq(r, wq, rq);
	p.getVp(result, result2);
	EXPECT_TRUE((s_is_equal(3, to_vs + 3, p.vs() + 3, error) && s_is_equal(3, vp, result, error) && s_is_equal(3, 3, *to_pm, 4, *p.pm(), 4, error) && s_is_equal(3, pp, result2, error))) << "\"part:setWq relative\" failed";

	p.setRq(rq);
	p.setWq(wq);
	p.getVp(result, result2);
	EXPECT_TRUE((s_is_equal(3, vs + 3, p.vs() + 3, error) && s_is_equal(3, vp, result, error) && s_is_equal(16, pm, *p.pm(), error))) << "\"part:setWq\" failed";

	p.setRq(r, rq);
	p.setWq(r, wq);
	p.getVp(result, result2);
	EXPECT_TRUE((s_is_equal(3, to_vs + 3, p.vs() + 3, error) && s_is_equal(3, vp, result, error) && s_is_equal(3, 3, *to_pm, 4, *p.pm(), 4, error) && s_is_equal(3, pp, result2, error))) << "\"part:setWq relative\" failed";

	p.setWm(wm, rm);
	p.getVp(result, result2);
	EXPECT_TRUE((s_is_equal(3, vs + 3, p.vs() + 3, error) && s_is_equal(3, vp, result, error) && s_is_equal(16, pm, *p.pm(), error))) << "\"part:setWm\" failed";

	p.setWm(r, wm, rm);
	p.getVp(result, result2);
	EXPECT_TRUE((s_is_equal(3, to_vs + 3, p.vs() + 3, error) && s_is_equal(3, vp, result, error) && s_is_equal(3, 3, *to_pm, 4, *p.pm(), 4, error) && s_is_equal(3, pp, result2, error))) << "\"part:setWm relative\" failed";

	p.setRm(rm);
	p.setWm(wm);
	p.getVp(result, result2);
	EXPECT_TRUE((s_is_equal(3, vs + 3, p.vs() + 3, error) && s_is_equal(3, vp, result, error) && s_is_equal(16, pm, *p.pm(), error))) << "\"part:setWm\" failed";

	p.setRm(r, rm);
	p.setWm(r, wm);
	p.getVp(result, result2);
	EXPECT_TRUE((s_is_equal(3, to_vs + 3, p.vs() + 3, error) && s_is_equal(3, vp, result, error) && s_is_equal(3, 3, *to_pm, 4, *p.pm(), 4, error) && s_is_equal(3, pp, result2, error))) << "\"part:setWm relative\" failed";

	p.setWm(vm, pm, 4, 4);
	p.getVp(result, result2);
	EXPECT_TRUE((s_is_equal(3, vs + 3, p.vs() + 3, error) && s_is_equal(3, vp, result, error) && s_is_equal(16, pm, *p.pm(), error))) << "\"part:setWm with ld\" failed";

	p.setWm(r, vm, pm, 4, 4);
	p.getVp(result, result2);
	EXPECT_TRUE((s_is_equal(3, to_vs + 3, p.vs() + 3, error) && s_is_equal(3, vp, result, error) && s_is_equal(3, 3, *to_pm, 4, *p.pm(), 4, error) && s_is_equal(3, pp, result2, error))) << "\"part:setWm with ld relative\" failed";

	p.setRm(pm, 4);
	p.setWm(vm, nullptr, 4);
	p.getVp(result, result2);
	EXPECT_TRUE((s_is_equal(3, vs + 3, p.vs() + 3, error) && s_is_equal(3, vp, result, error) && s_is_equal(16, pm, *p.pm(), error))) << "\"part:setWm with ld\" failed";

	p.setRm(r, pm, 4);
	p.setWm(r, vm, nullptr, 4);
	p.getVp(result, result2);
	EXPECT_TRUE((s_is_equal(3, to_vs + 3, p.vs() + 3, error) && s_is_equal(3, vp, result, error) && s_is_equal(3, 3, *to_pm, 4, *p.pm(), 4, error) && s_is_equal(3, pp, result2, error))) << "\"part:setWm with ld relative\" failed";

	p.setWa(wa, rm);
	p.getVp(result, result2);
	EXPECT_TRUE((s_is_equal(3, vs + 3, p.vs() + 3, error) && s_is_equal(3, vp, result, error) && s_is_equal(16, pm, *p.pm(), error))) << "\"part:setWm\" failed";

	p.setWa(r, wa, rm);
	p.getVp(result, result2);
	EXPECT_TRUE((s_is_equal(3, to_vs + 3, p.vs() + 3, error) && s_is_equal(3, vp, result, error) && s_is_equal(3, 3, *to_pm, 4, *p.pm(), 4, error) && s_is_equal(3, pp, result2, error))) << "\"part:setWm relative\" failed";

	p.setRm(rm);
	p.setWa(wa);
	p.getVp(result, result2);
	EXPECT_TRUE((s_is_equal(3, vs + 3, p.vs() + 3, error) && s_is_equal(3, vp, result, error) && s_is_equal(16, pm, *p.pm(), error))) << "\"part:setWm\" failed";

	p.setRm(r, rm);
	p.setWa(r, wa);
	p.getVp(result, result2);
	EXPECT_TRUE((s_is_equal(3, to_vs + 3, p.vs() + 3, error) && s_is_equal(3, vp, result, error) && s_is_equal(3, 3, *to_pm, 4, *p.pm(), 4, error) && s_is_equal(3, pp, result2, error))) << "\"part:setWm relative\" failed";

	p.setWa(wa, pm, 4);
	p.getVp(result, result2);
	EXPECT_TRUE((s_is_equal(3, vs + 3, p.vs() + 3, error) && s_is_equal(3, vp, result, error) && s_is_equal(16, pm, *p.pm(), error))) << "\"part:setWa with ld\" failed";

	p.setWa(r, wa, pm, 4);
	p.getVp(result, result2);
	EXPECT_TRUE((s_is_equal(3, to_vs + 3, p.vs() + 3, error) && s_is_equal(3, vp, result, error) && s_is_equal(3, 3, *to_pm, 4, *p.pm(), 4, error) && s_is_equal(3, pp, result2, error))) << "\"part:setWa with ld relative\" failed";

	p.setRm(pm, 4);
	p.setWa(wa);
	p.getVp(result, result2);
	EXPECT_TRUE((s_is_equal(3, vs + 3, p.vs() + 3, error) && s_is_equal(3, vp, result, error) && s_is_equal(16, pm, *p.pm(), error))) << "\"part:setWa with ld\" failed";

	p.setRm(r, pm, 4);
	p.setWa(r, wa);
	p.getVp(result, result2);
	EXPECT_TRUE((s_is_equal(3, to_vs + 3, p.vs() + 3, error) && s_is_equal(3, vp, result, error) && s_is_equal(3, 3, *to_pm, 4, *p.pm(), 4, error) && s_is_equal(3, pp, result2, error))) << "\"part:setWa with ld relative\" failed";

	p.setVe(ve313, pe313);
	EXPECT_TRUE((s_is_equal(6, vs, p.vs(), error) && s_is_equal(16, pm, *p.pm(), error))) << "\"part:setVe\" failed";

	p.setVe(r, ve313, pe313);
	EXPECT_TRUE((s_is_equal(6, to_vs, p.vs(), error) && s_is_equal(16, *to_pm, *p.pm(), error))) << "\"part:setVe relative\" failed";

	p.setPe(pe313);
	p.setVe(ve313);
	EXPECT_TRUE((s_is_equal(6, vs, p.vs(), error) && s_is_equal(16, pm, *p.pm(), error))) << "\"part:setVe\" failed";

	p.setPe(r, pe313);
	p.setVe(r, ve313);
	EXPECT_TRUE((s_is_equal(6, to_vs, p.vs(), error) && s_is_equal(16, *to_pm, *p.pm(), error))) << "\"part:setVe relative\" failed";

	p.setVe(ve321, pe321, "321");
	EXPECT_TRUE((s_is_equal(6, vs, p.vs(), error) && s_is_equal(16, pm, *p.pm(), error))) << "\"part:setVe 321\" failed";

	p.setVe(r, ve321, pe321, "321");
	EXPECT_TRUE((s_is_equal(6, to_vs, p.vs(), error) && s_is_equal(16, *to_pm, *p.pm(), error))) << "\"part:setVe 321 relative\" failed";

	p.setPe(pe321, "321");
	p.setVe(ve321, nullptr, "321");
	EXPECT_TRUE((s_is_equal(6, vs, p.vs(), error) && s_is_equal(16, pm, *p.pm(), error))) << "\"part:setVe 321\" failed";

	p.setPe(r, pe321, "321");
	p.setVe(r, ve321, nullptr, "321");
	EXPECT_TRUE((s_is_equal(6, to_vs, p.vs(), error) && s_is_equal(16, *to_pm, *p.pm(), error))) << "\"part:setVe 321 relative\" failed";

	p.setVq(vq, pq);
	EXPECT_TRUE((s_is_equal(6, vs, p.vs(), error) && s_is_equal(16, pm, *p.pm(), error))) << "\"part:setVq\" failed";

	p.setVq(r, vq, pq);
	EXPECT_TRUE((s_is_equal(6, to_vs, p.vs(), error) && s_is_equal(16, *to_pm, *p.pm(), error))) << "\"part:setVq relative\" failed";

	p.setPq(pq);
	p.setVq(vq);
	EXPECT_TRUE((s_is_equal(6, vs, p.vs(), error) && s_is_equal(16, pm, *p.pm(), error))) << "\"part:setVq\" failed";

	p.setPq(r, pq);
	p.setVq(r, vq);
	EXPECT_TRUE((s_is_equal(6, to_vs, p.vs(), error) && s_is_equal(16, *to_pm, *p.pm(), error))) << "\"part:setVq relative\" failed";

	p.setVm(vm, pm);
	EXPECT_TRUE((s_is_equal(6, vs, p.vs(), error) && s_is_equal(16, pm, *p.pm(), error))) << "\"part:setVm\" failed";

	p.setVm(r, vm, pm);
	EXPECT_TRUE((s_is_equal(6, to_vs, p.vs(), error) && s_is_equal(16, *to_pm, *p.pm(), error))) << "\"part:setVm relative\" failed";

	p.setPm(pm);
	p.setVm(vm);
	EXPECT_TRUE((s_is_equal(6, vs, p.vs(), error) && s_is_equal(16, pm, *p.pm(), error))) << "\"part:setVm\" failed";

	p.setPm(r, pm);
	p.setVm(r, vm);
	EXPECT_TRUE((s_is_equal(6, to_vs, p.vs(), error) && s_is_equal(16, *to_pm, *p.pm(), error))) << "\"part:setVm relative\" failed";

	p.setVa(va, pp);
	EXPECT_TRUE((s_is_equal(6, vs, p.vs(), error) && s_is_equal(3, pp, 1, &p.pm()[0][3], 4, error))) << "\"part:setVa\" failed";

	p.setVa(r, va, pp);
	EXPECT_TRUE((s_is_equal(6, to_vs, p.vs(), error) && s_is_equal(3, &to_pm[0][3], 4, &p.pm()[0][3], 4, error))) << "\"part:setVa relative\" failed";

	p.setPp(pp);
	p.setVa(va);
	EXPECT_TRUE((s_is_equal(6, vs, p.vs(), error) && s_is_equal(3, pp, 1, &p.pm()[0][3], 4, error))) << "\"part:setVa\" failed";

	p.setPp(r, pp);
	p.setVa(r, va);
	EXPECT_TRUE((s_is_equal(6, to_vs, p.vs(), error) && s_is_equal(3, &to_pm[0][3], 4, &p.pm()[0][3], 4, error))) << "\"part:setVa relative\" failed";

	p.setVs(vs, pm);
	EXPECT_TRUE((s_is_equal(6, vs, p.vs(), error) && s_is_equal(16, pm, *p.pm(), error))) << "\"part:setVs\" failed";

	p.setVs(r, vs, pm);
	EXPECT_TRUE((s_is_equal(6, to_vs, p.vs(), error) && s_is_equal(16, *to_pm, *p.pm(), error))) << "\"part:setVs relative\" failed";

	p.setPm(pm);
	p.setVs(vs);
	EXPECT_TRUE((s_is_equal(6, vs, p.vs(), error) && s_is_equal(16, pm, *p.pm(), error))) << "\"part:setVs\" failed";

	p.setPm(r, pm);
	p.setVs(r, vs);
	EXPECT_TRUE((s_is_equal(6, to_vs, p.vs(), error) && s_is_equal(16, *to_pm, *p.pm(), error))) << "\"part:setVs relative\" failed";


	p.setXa(xa, wa, rm);
	p.setAp(ap, vp, pp);
	EXPECT_TRUE((s_is_equal(6, as, p.as(), error) && s_is_equal(6, vs, p.vs(), error) && s_is_equal(16, pm, *p.pm(), error))) << "\"part:setAp\" failed";

	p.setXa(r, xa, wa, rm);
	p.setAp(r, ap, vp, pp);
	EXPECT_TRUE((s_is_equal(6, to_as, p.as(), error) && s_is_equal(6, to_vs, p.vs(), error) && s_is_equal(16, *to_pm, *p.pm(), error))) << "\"part:setAp relative\" failed";

	p.setXa(xa, wa, rm);
	p.setVp(vp, pp);
	p.setAp(ap);
	EXPECT_TRUE((s_is_equal(6, as, p.as(), error) && s_is_equal(6, vs, p.vs(), error) && s_is_equal(16, pm, *p.pm(), error))) << "\"part:setAp\" failed";

	p.setXa(r, xa, wa, rm);
	p.setVp(r, vp, pp);
	p.setAp(r, ap);
	EXPECT_TRUE((s_is_equal(6, to_as, p.as(), error) && s_is_equal(6, to_vs, p.vs(), error) && s_is_equal(16, *to_pm, *p.pm(), error))) << "\"part:setAp relative\" failed";

	p.setAp(ap, vp, pp);

	p.setXe(xe313, we313, re313);
	p.getAp(result, result2, result3);
	EXPECT_TRUE((s_is_equal(3, as + 3, p.as() + 3, error) && s_is_equal(3, result, ap, error) && s_is_equal(3, vs + 3, p.vs() + 3, error) && s_is_equal(3, result2, vp, error)
		&& s_is_equal(3, 3, rm, 3, *p.pm(), 4, error) && s_is_equal(3, pp, result3, error))) << "\"part:setXe 313\" failed";

	p.setXe(r, xe313, we313, re313);
	p.getAp(result, result2, result3);
	EXPECT_TRUE((s_is_equal(3, to_as + 3, p.as() + 3, error) && s_is_equal(3, result, ap, error) && s_is_equal(3, to_vs + 3, p.vs() + 3, error) && s_is_equal(3, result2, vp, error)
		&& s_is_equal(3, 3, *to_pm, 4, *p.pm(), 4, error) && s_is_equal(3, result3, pp, error))) << "\"part:setXe 313 relative\" failed";

	p.setWe(we313, re313);
	p.setXe(xe313, nullptr, nullptr);
	p.getAp(result, result2, result3);
	EXPECT_TRUE((s_is_equal(3, as + 3, p.as() + 3, error) && s_is_equal(3, vs + 3, p.vs() + 3, error) && s_is_equal(3, result2, vp, error)
		&& s_is_equal(3, 3, rm, 3, *p.pm(), 4, error) && s_is_equal(3, pp, result3, error))) << "\"part:setXe 313\" failed";

	p.setWe(r, we313, re313);
	p.setXe(r, xe313, nullptr, nullptr);
	p.getAp(result, result2, result3);
	EXPECT_TRUE((s_is_equal(3, to_as + 3, p.as() + 3, error) && s_is_equal(3, to_vs + 3, p.vs() + 3, error) && s_is_equal(3, result2, vp, error)
		&& s_is_equal(3, 3, *to_pm, 4, *p.pm(), 4, error) && s_is_equal(3, result3, pp, error))) << "\"part:setXe 313 relative\" failed";

	p.setXe(xe321, we321, re321, "321");
	p.getAp(result, result2, result3);
	EXPECT_TRUE((s_is_equal(3, as + 3, p.as() + 3, error) && s_is_equal(3, result, ap, error) && s_is_equal(3, vs + 3, p.vs() + 3, error) && s_is_equal(3, result2, vp, error)
		&& s_is_equal(3, 3, rm, 3, *p.pm(), 4, error) && s_is_equal(3, pp, result3, error))) << "\"part:setXe 321\" failed";

	p.setXe(r, xe321, we321, re321, "321");
	p.getAp(result, result2, result3);
	EXPECT_TRUE((s_is_equal(3, to_as + 3, p.as() + 3, error) && s_is_equal(3, result, ap, error) && s_is_equal(3, to_vs + 3, p.vs() + 3, error) && s_is_equal(3, result2, vp, error)
		&& s_is_equal(3, 3, *to_pm, 4, *p.pm(), 4, error) && s_is_equal(3, result3, pp, error))) << "\"part:setXe 321 relative\" failed";

	p.setWe(we321, re321, "321");
	p.setXe(xe321, nullptr, nullptr, "321");
	p.getAp(result, result2, result3);
	EXPECT_TRUE((s_is_equal(3, as + 3, p.as() + 3, error) && s_is_equal(3, vs + 3, p.vs() + 3, error) && s_is_equal(3, result2, vp, error)
		&& s_is_equal(3, 3, rm, 3, *p.pm(), 4, error) && s_is_equal(3, pp, result3, error))) << "\"part:setXe 321\" failed";

	p.setWe(r, we321, re321, "321");
	p.setXe(r, xe321, nullptr, nullptr, "321");
	p.getAp(result, result2, result3);
	EXPECT_TRUE((s_is_equal(3, to_as + 3, p.as() + 3, error) && s_is_equal(3, to_vs + 3, p.vs() + 3, error) && s_is_equal(3, result2, vp, error)
		&& s_is_equal(3, 3, *to_pm, 4, *p.pm(), 4, error) && s_is_equal(3, result3, pp, error))) << "\"part:setXe 321 relative\" failed";

	p.setXq(xq, wq, rq);
	p.getAp(result, result2, result3);
	EXPECT_TRUE((s_is_equal(3, as + 3, p.as() + 3, error) && s_is_equal(3, result, ap, error) && s_is_equal(3, vs + 3, p.vs() + 3, error) && s_is_equal(3, result2, vp, error)
		&& s_is_equal(3, 3, rm, 3, *p.pm(), 4, error) && s_is_equal(3, pp, result3, error))) << "\"part:setXq\" failed";

	p.setXq(r, xq, wq, rq);
	p.getAp(result, result2, result3);
	EXPECT_TRUE((s_is_equal(3, to_as + 3, p.as() + 3, error) && s_is_equal(3, result, ap, error) && s_is_equal(3, to_vs + 3, p.vs() + 3, error) && s_is_equal(3, result2, vp, error)
		&& s_is_equal(3, 3, *to_pm, 4, *p.pm(), 4, error) && s_is_equal(3, result3, pp, error))) << "\"part:setXq relative\" failed";

	p.setWq(wq, rq);
	p.setXq(xq);
	p.getAp(result, result2, result3);
	EXPECT_TRUE((s_is_equal(3, as + 3, p.as() + 3, error) && s_is_equal(3, vs + 3, p.vs() + 3, error) && s_is_equal(3, result2, vp, error)
		&& s_is_equal(3, 3, rm, 3, *p.pm(), 4, error) && s_is_equal(3, pp, result3, error))) << "\"part:setXq\" failed";

	p.setWq(r, wq, rq);
	p.setXq(r, xq);
	p.getAp(result, result2, result3);
	EXPECT_TRUE((s_is_equal(3, to_as + 3, p.as() + 3, error) && s_is_equal(3, to_vs + 3, p.vs() + 3, error) && s_is_equal(3, result2, vp, error)
		&& s_is_equal(3, 3, *to_pm, 4, *p.pm(), 4, error) && s_is_equal(3, result3, pp, error))) << "\"part:setXq relative\" failed";

	p.setXm(xm, wm, rm);
	p.getAp(result, result2, result3);
	EXPECT_TRUE((s_is_equal(3, as + 3, p.as() + 3, error) && s_is_equal(3, result, ap, error) && s_is_equal(3, vs + 3, p.vs() + 3, error) && s_is_equal(3, result2, vp, error)
		&& s_is_equal(3, 3, rm, 3, *p.pm(), 4, error) && s_is_equal(3, pp, result3, error))) << "\"part:setXm\" failed";

	p.setXm(r, xm, wm, rm);
	p.getAp(result, result2, result3);
	EXPECT_TRUE((s_is_equal(3, to_as + 3, p.as() + 3, error) && s_is_equal(3, result, ap, error) && s_is_equal(3, to_vs + 3, p.vs() + 3, error) && s_is_equal(3, result2, vp, error)
		&& s_is_equal(3, 3, *to_pm, 4, *p.pm(), 4, error) && s_is_equal(3, result3, pp, error))) << "\"part:setXm relative\" failed";

	p.setWm(wm, rm);
	p.setXm(xm);
	p.getAp(result, result2, result3);
	EXPECT_TRUE((s_is_equal(3, as + 3, p.as() + 3, error) && s_is_equal(3, vs + 3, p.vs() + 3, error) && s_is_equal(3, result2, vp, error)
		&& s_is_equal(3, 3, rm, 3, *p.pm(), 4, error) && s_is_equal(3, pp, result3, error))) << "\"part:setXm\" failed";

	p.setWm(r, wm, rm);
	p.setXm(r, xm);
	p.getAp(result, result2, result3);
	EXPECT_TRUE((s_is_equal(3, to_as + 3, p.as() + 3, error) && s_is_equal(3, to_vs + 3, p.vs() + 3, error) && s_is_equal(3, result2, vp, error)
		&& s_is_equal(3, 3, *to_pm, 4, *p.pm(), 4, error) && s_is_equal(3, result3, pp, error))) << "\"part:setXm relative\" failed";

	p.setXa(xa, wa, rm);
	p.getAp(result, result2, result3);
	EXPECT_TRUE((s_is_equal(3, as + 3, p.as() + 3, error) && s_is_equal(3, result, ap, error) && s_is_equal(3, vs + 3, p.vs() + 3, error) && s_is_equal(3, result2, vp, error)
		&& s_is_equal(3, 3, rm, 3, *p.pm(), 4, error) && s_is_equal(3, pp, result3, error))) << "\"part:setXa\" failed";

	p.setXa(r, xa, wa, rm);
	p.getAp(result, result2, result3);
	EXPECT_TRUE((s_is_equal(3, to_as + 3, p.as() + 3, error) && s_is_equal(3, result, ap, error) && s_is_equal(3, to_vs + 3, p.vs() + 3, error) && s_is_equal(3, result2, vp, error)
		&& s_is_equal(3, 3, *to_pm, 4, *p.pm(), 4, error) && s_is_equal(3, result3, pp, error))) << "\"part:setXa relative\" failed";

	p.setWa(wa, rm);
	p.setXa(xa);
	p.getAp(result, result2, result3);
	EXPECT_TRUE((s_is_equal(3, as + 3, p.as() + 3, error) && s_is_equal(3, vs + 3, p.vs() + 3, error) && s_is_equal(3, result2, vp, error)
		&& s_is_equal(3, 3, rm, 3, *p.pm(), 4, error) && s_is_equal(3, pp, result3, error))) << "\"part:setXa\" failed";

	p.setWa(r, wa, rm);
	p.setXa(r, xa);
	p.getAp(result, result2, result3);
	EXPECT_TRUE((s_is_equal(3, to_as + 3, p.as() + 3, error) && s_is_equal(3, to_vs + 3, p.vs() + 3, error) && s_is_equal(3, result2, vp, error)
		&& s_is_equal(3, 3, *to_pm, 4, *p.pm(), 4, error) && s_is_equal(3, result3, pp, error))) << "\"part:setXa relative\" failed";



	p.setAe(ae313, ve313, pe313);
	EXPECT_TRUE((s_is_equal(6, as, p.as(), error))) << "\"part:setAe\" failed";

	p.setAe(r, ae313, ve313, pe313);
	EXPECT_TRUE((s_is_equal(6, to_as, p.as(), error))) << "\"part:setAe relative\" failed";

	p.setVe(ve313, pe313);
	p.setAe(ae313);
	EXPECT_TRUE((s_is_equal(6, as, p.as(), error))) << "\"part:setAe\" failed";

	p.setVe(r, ve313, pe313);
	p.setAe(r, ae313);
	EXPECT_TRUE((s_is_equal(6, to_as, p.as(), error))) << "\"part:setAe relative\" failed";

	p.setAe(ae321, ve321, pe321, "321");
	EXPECT_TRUE((s_is_equal(6, as, p.as(), error))) << "\"part:setAe 321\" failed";

	p.setAe(r, ae321, ve321, pe321, "321");
	EXPECT_TRUE((s_is_equal(6, to_as, p.as(), error))) << "\"part:setAe 321 relative\" failed";

	p.setVe(ve321, pe321, "321");
	p.setAe(ae321, nullptr, nullptr, "321");
	EXPECT_TRUE((s_is_equal(6, as, p.as(), error))) << "\"part:setAe 321\" failed";

	p.setVe(r, ve321, pe321, "321");
	p.setAe(r, ae321, nullptr, nullptr, "321");
	EXPECT_TRUE((s_is_equal(6, to_as, p.as(), error))) << "\"part:setAe 321 relative\" failed";

	p.setAq(aq, vq, pq);
	EXPECT_TRUE((s_is_equal(6, as, p.as(), error))) << "\"part:setAq\" failed";

	p.setAq(r, aq, vq, pq);
	EXPECT_TRUE((s_is_equal(6, to_as, p.as(), error))) << "\"part:setAq relative\" failed";

	p.setVq(vq, pq);
	p.setAq(aq);
	EXPECT_TRUE((s_is_equal(6, as, p.as(), error))) << "\"part:setAq\" failed";

	p.setVq(r, vq, pq);
	p.setAq(r, aq);
	EXPECT_TRUE((s_is_equal(6, to_as, p.as(), error))) << "\"part:setAq relative\" failed";

	p.setAm(am, vm, pm);
	EXPECT_TRUE((s_is_equal(6, as, p.as(), error))) << "\"part:setAm\" failed";

	p.setAm(r, am, vm, pm);
	EXPECT_TRUE((s_is_equal(6, to_as, p.as(), error))) << "\"part:setAm relative\" failed";

	p.setVm(vm, pm);
	p.setAm(am);
	EXPECT_TRUE((s_is_equal(6, as, p.as(), error))) << "\"part:setAm\" failed";

	p.setVm(r, vm, pm);
	p.setAm(r, am);
	EXPECT_TRUE((s_is_equal(6, to_as, p.as(), error))) << "\"part:setAm relative\" failed";

	p.setAa(aa, va, pp);
	EXPECT_TRUE((s_is_equal(6, as, p.as(), error))) << "\"part:setAa\" failed";

	p.setAa(r, aa, va, pp);
	EXPECT_TRUE((s_is_equal(6, to_as, p.as(), error))) << "\"part:setAa relative\" failed";

	p.setVa(va, pp);
	p.setAa(aa);
	EXPECT_TRUE((s_is_equal(6, as, p.as(), error))) << "\"part:setAa\" failed";

	p.setVa(r, va, pp);
	p.setAa(r, aa);
	EXPECT_TRUE((s_is_equal(6, to_as, p.as(), error))) << "\"part:setAa relative\" failed";

	p.setAs(as, vs, pm);
	EXPECT_TRUE((s_is_equal(6, as, p.as(), error))) << "\"part:setAs\" failed";

	p.setAs(r, as, vs, pm);
	EXPECT_TRUE((s_is_equal(6, to_as, p.as(), error))) << "\"part:setAs relative\" failed";

	p.setVs(vs, pm);
	p.setAs(as, vs, pm);
	EXPECT_TRUE((s_is_equal(6, as, p.as(), error))) << "\"part:setAs\" failed";

	p.setVs(r, vs, pm);
	p.setAs(r, as);
	EXPECT_TRUE((s_is_equal(6, to_as, p.as(), error))) << "\"part:setAs relative\" failed";


	r.setPm(relative_pm);
	r.setVs(relative_vs);
	r.setAs(relative_as);

	p.setPm(pm);
	p.setVs(vs);
	p.setAs(as);

	p.getPp(result);
	EXPECT_TRUE((s_is_equal(3, result, pp, error))) << "\"coordinate:getPp\" failed";

	p.getRe(result);
	EXPECT_TRUE((s_is_equal(3, result, re313, error))) << "\"coordinate:getRe 313\" failed";

	p.getRe(result, "321");
	EXPECT_TRUE((s_is_equal(3, result, re321, error))) << "\"coordinate:getRe 321\" failed";

	p.getRq(result);
	EXPECT_TRUE((s_is_equal(3, result, rq, error))) << "\"coordinate:getRq\" failed";

	p.getRm(result);
	EXPECT_TRUE((s_is_equal(9, result, rm, error))) << "\"coordinate:getRm\" failed";

	p.getPe(result);
	EXPECT_TRUE((s_is_equal(6, result, pe313, error))) << "\"coordinate:getPe\" failed";

	p.getPe(result, "321");
	EXPECT_TRUE((s_is_equal(6, result, pe321, error))) << "\"coordinate:getPe 321\" failed";

	p.getPq(result);
	EXPECT_TRUE((s_is_equal(7, result, pq, error))) << "\"coordinate:getPq\" failed";

	p.getPm(result);
	EXPECT_TRUE((s_is_equal(16, result, pm, error))) << "\"coordinate:getPm\" failed";

	p.getVp(result, result2);
	EXPECT_TRUE((s_is_equal(3, result, vp, error) && s_is_equal(3, result2, pp, error))) << "\"coordinate:getVp\" failed";

	p.getWe(result, result2);
	EXPECT_TRUE((s_is_equal(3, result, we313, error) && s_is_equal(3, result2, re313, error))) << "\"coordinate:getWe 313\" failed";

	p.getWe(result, result2, "321");
	EXPECT_TRUE((s_is_equal(3, result, we321, error) && s_is_equal(3, result2, re321, error))) << "\"coordinate:getWe 321\" failed";

	p.getWq(result, result2);
	EXPECT_TRUE((s_is_equal(4, result, wq, error) && s_is_equal(4, result2, rq, error))) << "\"coordinate:getWq\" failed";

	p.getWm(result, result2);
	EXPECT_TRUE((s_is_equal(9, result, wm, error) && s_is_equal(9, result2, rm, error))) << "\"coordinate:getWm\" failed";

	p.getWa(result, result3);
	EXPECT_TRUE((s_is_equal(3, result, wa, error) && s_is_equal(9, result3, rm, error))) << "\"coordinate:getWa\" failed";

	p.getVe(result, result2);
	EXPECT_TRUE((s_is_equal(6, result, ve313, error) && s_is_equal(6, result2, pe313, error))) << "\"coordinate:getVe 313\" failed";

	p.getVe(result, result2, "321");
	EXPECT_TRUE((s_is_equal(6, result, ve321, error) && s_is_equal(6, result2, pe321, error))) << "\"coordinate:getVe 321\" failed";

	p.getVq(result, result2);
	EXPECT_TRUE((s_is_equal(7, result, vq, error) && s_is_equal(7, result2, pq, error))) << "\"coordinate:getVq\" failed";

	p.getVm(result, result2);
	EXPECT_TRUE((s_is_equal(16, result, vm, error) && s_is_equal(16, result2, pm, error))) << "\"coordinate:getVm\" failed";

	p.getVa(result, result2);
	EXPECT_TRUE((s_is_equal(6, result, va, error) && s_is_equal(3, result2, pp, error))) << "\"coordinate:getVa\" failed";

	p.getVs(result, result2);
	EXPECT_TRUE((s_is_equal(6, result, vs, error) && s_is_equal(16, result2, pm, error))) << "\"coordinate:getVs\" failed";

	p.getAp(result, result2, result3);
	EXPECT_TRUE((s_is_equal(3, result, ap, error) && s_is_equal(3, result2, vp, error) && s_is_equal(3, result3, pp, error))) << "\"coordinate:getAp\" failed";

	p.getXe(result, result2, result3);
	EXPECT_TRUE((s_is_equal(3, result, xe313, error) && s_is_equal(3, result2, we313, error) && s_is_equal(3, result3, re313, error))) << "\"coordinate:getXe 313\" failed";

	p.getXe(result, result2, result3, "321");
	EXPECT_TRUE((s_is_equal(3, result, xe321, error) && s_is_equal(3, result2, we321, error) && s_is_equal(3, result3, re321, error))) << "\"coordinate:getXe 313\" failed";

	p.getXq(result, result2, result3);
	EXPECT_TRUE((s_is_equal(4, result, xq, error) && s_is_equal(4, result2, wq, error) && s_is_equal(4, result3, rq, error))) << "\"coordinate:getXq\" failed";

	p.getXm(result, result2, result3);
	EXPECT_TRUE((s_is_equal(9, result, xm, error) && s_is_equal(9, result2, wm, error) && s_is_equal(9, result3, rm, error))) << "\"coordinate:getXm\" failed";

	p.getXa(result, result2, result3);
	EXPECT_TRUE((s_is_equal(3, result, xa, error) && s_is_equal(3, result2, wa, error) && s_is_equal(9, result3, rm, error))) << "\"coordinate:getXm\" failed";

	p.getAe(result, result2, result3);
	EXPECT_TRUE((s_is_equal(6, result, ae313, error) && s_is_equal(6, result2, ve313, error) && s_is_equal(6, result3, pe313, error))) << "\"coordinate:getAe 313\" failed";

	p.getAe(result, result2, result3, "321");
	EXPECT_TRUE((s_is_equal(6, result, ae321, error) && s_is_equal(6, result2, ve321, error) && s_is_equal(6, result3, pe321, error))) << "\"coordinate:getAe 321\" failed";

	p.getAq(result, result2, result3);
	EXPECT_TRUE((s_is_equal(7, result, aq, error) && s_is_equal(7, result2, vq, error) && s_is_equal(7, result3, pq, error))) << "\"coordinate:getAq\" failed";

	p.getAm(result, result2, result3);
	EXPECT_TRUE((s_is_equal(16, result, am, error) && s_is_equal(16, result2, vm, error) && s_is_equal(16, result3, pm, error))) << "\"coordinate:getAm\" failed";

	p.getAa(result, result2, result3);
	EXPECT_TRUE((s_is_equal(6, result, aa, error) && s_is_equal(3, result2, va, error) && s_is_equal(3, result3, pp, error))) << "\"coordinate:getAa\" failed";

	p.getAs(result, result2, result3);
	EXPECT_TRUE((s_is_equal(6, result, as, error) && s_is_equal(6, result2, vs, error) && s_is_equal(16, result3, pm, error))) << "\"coordinate:getAs\" failed";

	p.setPm(r, pm);
	p.setVs(r, vs);
	p.setAs(r, as);

	p.getPp(r, result);
	EXPECT_TRUE((s_is_equal(3, result, pp, error))) << "\"coordinate:getPp\" failed";

	p.getRe(r, result);
	EXPECT_TRUE((s_is_equal(3, result, re313, error))) << "\"coordinate:getRe\" failed";

	p.getRe(r, result, "321");
	EXPECT_TRUE((s_is_equal(3, result, re321, error))) << "\"coordinate:getRe 321\" failed";

	p.getRq(r, result);
	EXPECT_TRUE((s_is_equal(4, result, rq, error))) << "\"coordinate:getRq\" failed";

	p.getRm(r, result);
	EXPECT_TRUE((s_is_equal(9, result, rm, error))) << "\"coordinate:getRm\" failed";

	p.getRm(r, result, 4);
	EXPECT_TRUE((s_is_equal(3, 3, result, 4, rm, 3, error))) << "\"coordinate:getRm\" failed";

	p.getPe(r, result);
	EXPECT_TRUE((s_is_equal(6, result, pe313, error))) << "\"coordinate:getPe\" failed";

	p.getPe(r, result, "321");
	EXPECT_TRUE((s_is_equal(6, result, pe321, error))) << "\"coordinate:getPe 321\" failed";

	p.getPq(r, result);
	EXPECT_TRUE((s_is_equal(7, result, pq, error))) << "\"coordinate:getPq\" failed";

	p.getPm(r, result);
	EXPECT_TRUE((s_is_equal(16, result, pm, error))) << "\"coordinate:getPm\" failed";

	p.getVp(r, result, result2);
	EXPECT_TRUE((s_is_equal(3, result, vp, error) && s_is_equal(3, result2, pp, error))) << "\"coordinate:getVp\" failed";

	p.getWe(r, result, result2);
	EXPECT_TRUE((s_is_equal(3, result, we313, error) && s_is_equal(3, result2, re313, error))) << "\"coordinate:getWe 313\" failed";

	p.getWe(r, result, result2, "321");
	EXPECT_TRUE((s_is_equal(3, result, we321, error) && s_is_equal(3, result2, re321, error))) << "\"coordinate:getWe 321\" failed";

	p.getWq(r, result, result2);
	EXPECT_TRUE((s_is_equal(4, result, wq, error) && s_is_equal(4, result2, rq, error))) << "\"coordinate:getWq\" failed";

	p.getWm(r, result, result2);
	EXPECT_TRUE((s_is_equal(9, result, wm, error) && s_is_equal(9, result2, rm, error))) << "\"coordinate:getWm\" failed";

	p.getWm(r, result, result2, 4, 4);
	EXPECT_TRUE((s_is_equal(3, 3, result, 4, wm, 3, error) && s_is_equal(3, 3, result2, 4, rm, 3, error))) << "\"coordinate:getWm\" failed";

	p.getWa(r, result, result2);
	EXPECT_TRUE((s_is_equal(3, result, wa, error) && s_is_equal(9, result2, rm, error))) << "\"coordinate:getWa\" failed";

	p.getWa(r, result, result2, 4);
	EXPECT_TRUE((s_is_equal(3, result, wa, error) && s_is_equal(3, 3, result2, 4, rm, 3, error))) << "\"coordinate:getWa\" failed";

	p.getVe(r, result, result2);
	EXPECT_TRUE((s_is_equal(6, result, ve313, error) && s_is_equal(6, result2, pe313, error))) << "\"coordinate:getVe 313\" failed";

	p.getVe(r, result, result2, "321");
	EXPECT_TRUE((s_is_equal(6, result, ve321, error) && s_is_equal(6, result2, pe321, error))) << "\"coordinate:getVe 321\" failed";

	p.getVq(r, result, result2);
	EXPECT_TRUE((s_is_equal(7, result, vq, error) && s_is_equal(7, result2, pq, error))) << "\"coordinate:getVq\" failed";

	p.getVm(r, result, result2);
	EXPECT_TRUE((s_is_equal(16, result, vm, error) && s_is_equal(16, result2, pm, error))) << "\"coordinate:getVm\" failed";

	p.getVa(r, result, result2);
	EXPECT_TRUE((s_is_equal(6, result, va, error) && s_is_equal(3, result2, pp, error))) << "\"coordinate:getVa\" failed";

	p.getVs(r, result, result2);
	EXPECT_TRUE((s_is_equal(6, result, vs, error) && s_is_equal(16, result2, pm, error))) << "\"coordinate:getVs\" failed";

	p.getAp(r, result, result2, result3);
	EXPECT_TRUE((s_is_equal(3, result, ap, error) && s_is_equal(3, result2, vp, error) && s_is_equal(3, result3, pp, error))) << "\"coordinate:getAp\" failed";

	p.getXe(r, result, result2, result3);
	EXPECT_TRUE((s_is_equal(3, result, xe313, error) && s_is_equal(3, result2, we313, error) && s_is_equal(3, result3, re313, error))) << "\"coordinate:getXe 313\" failed";

	p.getXe(r, result, result2, result3, "321");
	EXPECT_TRUE((s_is_equal(3, result, xe321, error) && s_is_equal(3, result2, we321, error) && s_is_equal(3, result3, re321, error))) << "\"coordinate:getXe 321\" failed";

	p.getXq(r, result, result2, result3);
	EXPECT_TRUE((s_is_equal(4, result, xq, error) && s_is_equal(4, result2, wq, error) && s_is_equal(4, result3, rq, error))) << "\"coordinate:getXq\" failed";

	p.getXm(r, result, result2, result3);
	EXPECT_TRUE((s_is_equal(9, result, xm, error) && s_is_equal(9, result2, wm, error) && s_is_equal(9, result3, rm, error))) << "\"coordinate:getXm\" failed";

	p.getXa(r, result, result2, result3);
	EXPECT_TRUE((s_is_equal(3, result, xa, error) && s_is_equal(3, result2, wa, error) && s_is_equal(9, result3, rm, error))) << "\"coordinate:getXa\" failed";

	p.getAe(r, result, result2, result3);
	EXPECT_TRUE((s_is_equal(6, result, ae313, error) && s_is_equal(6, result2, ve313, error) && s_is_equal(6, result3, pe313, error))) << "\"coordinate:getAe 313\" failed";

	p.getAe(r, result, result2, result3, "321");
	EXPECT_TRUE((s_is_equal(6, result, ae321, error) && s_is_equal(6, result2, ve321, error) && s_is_equal(6, result3, pe321, error))) << "\"coordinate:getAe 321\" failed";

	p.getAq(r, result, result2, result3);
	EXPECT_TRUE((s_is_equal(7, result, aq, error) && s_is_equal(7, result2, vq, error) && s_is_equal(7, result3, pq, error))) << "\"coordinate:getAq\" failed";

	p.getAm(r, result, result2, result3);
	EXPECT_TRUE((s_is_equal(16, result, am, error) && s_is_equal(16, result2, vm, error) && s_is_equal(16, result3, pm, error))) << "\"coordinate:getAm\" failed";

	p.getAa(r, result, result2, result3);
	EXPECT_TRUE((s_is_equal(6, result, aa, error) && s_is_equal(6, result2, va, error) && s_is_equal(3, result3, pp, error))) << "\"coordinate:getAa\" failed";

	p.getAs(r, result, result2, result3);
	EXPECT_TRUE((s_is_equal(6, result, as, error) && s_is_equal(6, result2, vs, error) && s_is_equal(16, result3, pm, error))) << "\"coordinate:getAs\" failed";



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
	EXPECT_TRUE((s_is_equal(6, 6, result, 7, im, 6, error))) << "\"part:cptIm\" failed";

	p.cptGlbIm(result, 7);
	EXPECT_TRUE((s_is_equal(6, 6, result, 7, glb_im, 6, error))) << "\"part:cptGlbIm\" failed";

	p.cptPrtIm(result, 7);
	EXPECT_TRUE((s_is_equal(6, 6, result, 7, prt_im, 6, error))) << "\"part:cptGlbIm\" failed";

	p.cptFg(r, result);
	EXPECT_TRUE((s_is_equal(6, result, fg, error))) << "\"part:cptFg\" failed";

	p.cptGlbFg(result);
	EXPECT_TRUE((s_is_equal(6, result, glb_fg, error))) << "\"part:cptGlbFg\" failed";

	p.cptPrtFg(result);
	EXPECT_TRUE((s_is_equal(6, result, prt_fg, error))) << "\"part:cptPrtFg\" failed";

	p.cptFv(r, result);
	EXPECT_TRUE((s_is_equal(6, result, fv, error))) << "\"part:cptFv\" failed";

	p.cptGlbFv(result);
	EXPECT_TRUE((s_is_equal(6, result, glb_fv, error))) << "\"part:cptGlbFv\" failed";

	p.cptPrtFv(result);
	EXPECT_TRUE((s_is_equal(6, result, prt_fv, error))) << "\"part:cptPrtFv\" failed";

	p.cptPf(r, result);
	EXPECT_TRUE((s_is_equal(6, result, pf, error))) << "\"part:cptPf\" failed";

	p.cptGlbPf(result);
	EXPECT_TRUE((s_is_equal(6, result, glb_pf, error))) << "\"part:cptGlbPf\" failed";

	p.cptPrtPf(result);
	EXPECT_TRUE((s_is_equal(6, result, prt_pf, error))) << "\"part:cptPrtPf\" failed";

	p.cptPrtVs(result);
	EXPECT_TRUE((s_is_equal(6, result, prt_vs, error))) << "\"part:cptPrtVs\" failed";

	p.cptPrtAs(result);
	EXPECT_TRUE((s_is_equal(6, result, prt_as, error))) << "\"part:cptPrtAs\" failed";
}

TEST(ModelCoordinate, CoordinateApiCoverage) {
	test_part();
}

} // namespace

