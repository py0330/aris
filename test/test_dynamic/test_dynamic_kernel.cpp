#include "test_dynamic_kernel.h"
#include <iostream>
#include <aris.h>

using namespace aris::dynamic;

const double error = 1e-10;

void test_variable_change();
void test_coordinate_transform();


void test_kernel()
{
	std::cout << std::endl << "-----------------test kernel--------------------" << std::endl;

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

	//test s_c3
	{
		double a[] = { 0.12, 0.25, 0.6 };
		double b[] = { 0.13, -0.21, 0.33 };
		double c[] = { 0.11,0.22,0.33 };
		double answer1[] = { 0.066425,   0.0382,   0.028475 };
		double answer2[] = { 0.2085,0.0384,-0.0577 };

		s_c3(0.25, a, b, 0.13, c);

		if (!s_is_equal(3, c, answer1, error))
		{
			std::cout << "\"s_cro3\" failed" << std::endl;
		}

		s_c3(a, b, c);

		if (!s_is_equal(3, c, answer2, error))
		{
			std::cout << "\"s_cro3\" failed" << std::endl;
		}
	}

	//test s_cm3
	{
		const double a[3] = { 0.1, 0.2, 0.3 };
		const double cm3[9] = { 0,  -0.3, 0.2,
			0.3, 0,  -0.1,
			-0.2, 0.1, 0 };

		double result[9];

		
		s_cm3(a, result);
		if (!s_is_equal(9, result, cm3, error))std::cout << "\"s_cro3\" failed" << std::endl;
	}

	test_variable_change();
	test_coordinate_transform();

	std::cout << "-----------------test kernel finished-----------" << std::endl << std::endl;
}

void test_variable_change()
{
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
		0.959583152331272, -0.216333478134982,   0.18, 0.3,
		0,0,0,1 };

	const double vp[3] = { 0.307558670154491,   1.2433000508379, -1.04895965543501 };
	const double we313[3] = { -0.644213536852877, - 0.245050866834802, - 1.27836042009784 };
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

	double result_pm_for_position[16] = { -0.22, -0.975499782797526,   0.000416847668728071, 0,
		0.175499782797526, -0.04, -0.983666521865018, 0,
		0.959583152331272, -0.216333478134982,   0.18,0,
		0,0,0,1 };
	double result_vs_for_position[6] = { 0,	0,	0, -0.244517963270725,	1.25737650310373,	-0.874318412470487 };
	double result_as_for_position[6] = { 0,  0,   0,   0.904633672502324, -1.24440604199266,   1.45568007018557 };
	double result_pm_for_angle[16] = { 1,0,0,0.1,
		0,1,0,0.2,
		0,0,1,0.3,
		0,0,0,1 };
	double result_vs_for_angle[6] = { -0.244517963270725,	1.25737650310373,	-0.874318412470487, 0,0,0 };
	double result_as_for_angle[6] = { 3.15925342342501, -0.192390604845803,   0.136512424183815,   0,0,0 };
	double result[16];
	double result1[16];

	s_re2rm(re321, result, "321");
	if (!s_is_equal(9, result, rm, error))std::cout << "\"s_re2rm 321\" failed" << std::endl;

	s_re2rm(re313, result);
	if (!s_is_equal(9, result, rm, error))std::cout << "\"s_re2rm 313\" failed" << std::endl;

	s_rm2re(rm, result, "321");
	if (!s_is_equal(3, result, re321, error))std::cout << "\"s_rm2re 321\" failed" << std::endl;

	s_rm2re(rm, result);
	if (!s_is_equal(3, result, re313, error))std::cout << "\"s_rm2re 313\" failed" << std::endl;

	s_rq2rm(rq, result);
	if (!s_is_equal(9, result, rm, error))std::cout << "\"s_rq2rm\" failed" << std::endl;

	s_rm2rq(rm, result);
	if (!s_is_equal(4, result, rq, error))std::cout << "\"s_rm2rq\" failed" << std::endl;

	s_pp2pm(pp, result_pm_for_position);
	if (!s_is_equal(16, result_pm_for_position, pm, error))std::cout << "\"s_pp2pm\" failed" << std::endl;

	s_pm2pp(pm, result);
	if (!s_is_equal(3, result, pp, error))std::cout << "\"s_pm2pp\" failed" << std::endl;
	
	s_re2pm(re321, result_pm_for_angle, "321");
	if (!s_is_equal(16, result_pm_for_angle, pm, error))std::cout << "\"s_re2pm\" failed" << std::endl;

	s_re2pm(re313, result_pm_for_angle);
	if (!s_is_equal(16, result_pm_for_angle, pm, error))std::cout << "\"s_re2pm\" failed" << std::endl;
	
	s_pm2re(pm, result, "321");
	if (!s_is_equal(3, result, re321, error))std::cout << "\"s_pm2re\" failed" << std::endl;

	s_pm2re(pm, result);
	if (!s_is_equal(3, result, re313, error))std::cout << "\"s_pm2re\" failed" << std::endl;

	s_rq2pm(rq, result_pm_for_angle);
	if (!s_is_equal(16, result_pm_for_angle, pm, error))std::cout << "\"s_rq2pm\" failed" << std::endl;

	s_pm2rq(pm, result);
	if (!s_is_equal(4, result, rq, error))std::cout << "\"s_pm2rq\" failed" << std::endl;

	s_rm2pm(rm, result_pm_for_angle);
	if (!s_is_equal(16, result_pm_for_angle, pm, error))std::cout << "\"s_rm2pm\" failed" << std::endl;

	s_pm2rm(pm, result);
	if (!s_is_equal(9, result, rm, error))std::cout << "\"s_pm2rm\" failed" << std::endl;

	s_pe2pm(pe321, result, "321");
	if (!s_is_equal(16, pm, result, error))std::cout << "\"s_pe2pm\" failed" << std::endl;

	s_pe2pm(pe313, result);
	if (!s_is_equal(16, result, pm, error))std::cout << "\"s_pe2pm\" failed" << std::endl;
	
	s_pm2pe(pm, result, "321");
	if (!s_is_equal(6, result, pe321, error))std::cout << "\"s_pm2pe\" failed" << std::endl;

	s_pm2pe(pm, result);
	if (!s_is_equal(6, result, pe313, error))std::cout << "\"s_pm2pe\" failed" << std::endl;
	
	s_pq2pm(pq, result);
	if (!s_is_equal(16, pm, result, error))std::cout << "\"s_pq2pm\" failed" << std::endl;

	s_pm2pq(pm, result);
	if (!s_is_equal(7, pq, result, error))std::cout << "\"s_pm2pq\" failed" << std::endl;



	s_wa2we(wa, re313, result, "313");
	if (!s_is_equal(3, result, we313, error))std::cout << "\"s_wa2we 313\" failed" << std::endl;

	s_wa2we(wa, re321, result, "321");
	if (!s_is_equal(3, result, we321, error))std::cout << "\"s_wa2we 321\" failed" << std::endl;

	s_we2wa(re313, we313, result, "313");
	if (!s_is_equal(3, result, wa, error))std::cout << "\"s_we2wa 313\" failed" << std::endl;

	s_we2wa(re321, we321, result, "321");
	if (!s_is_equal(3, result, wa, error))std::cout << "\"s_we2wa 321\" failed" << std::endl;

	s_wq2wa(rq, wq, result);
	if (!s_is_equal(3, result, wa, error))std::cout << "\"s_wq2wa\" failed" << std::endl;

	s_wa2wq(wa, rq, result);
	if (!s_is_equal(4, result, wq, error))std::cout << "\"s_wa2wq\" failed" << std::endl;

	s_wa2wm(wa, rm, result);
	if (!s_is_equal(9, result, wm, error))std::cout << "\"s_wa2wm\" failed" << std::endl;

	s_wm2wa(rm, wm, result);
	if (!s_is_equal(3, result, wa, error))std::cout << "\"s_wm2wa\" failed" << std::endl;

	s_vs2vp(vs, pp, result);
	if (!s_is_equal(3, result, vp, error))std::cout << "\"s_vs2vp\" failed" << std::endl;

	s_vp2vs(pp, vp, result_vs_for_position);
	if (!s_is_equal(6, result_vs_for_position, vs, error))std::cout << "\"s_vp2vs\" failed" << std::endl;

	s_we2vs(re313, we313, result, "313");
	if (!s_is_equal(3, result + 3, vs + 3, error))std::cout << "\"s_we2vs 313\" failed" << std::endl;

	s_we2vs(re321, we321, result, "321");
	if (!s_is_equal(3, result + 3, vs + 3, error))std::cout << "\"s_we2vs 321\" failed" << std::endl;
	
	s_vs2we(vs, re313, result, "313");
	if (!s_is_equal(3, result, we313, error))std::cout << "\"s_vs2we 313\" failed" << std::endl;

	s_vs2we(vs, re321, result, "321");
	if (!s_is_equal(3, result, we321, error))std::cout << "\"s_vs2we 321\" failed" << std::endl;

	s_wq2vs(rq, wq, result_vs_for_angle);
	if (!s_is_equal(6, result_vs_for_angle, vs, error))std::cout << "\"s_wq2vs\" failed" << std::endl;

	s_vs2wq(vs, rq, result);
	if (!s_is_equal(3, result, wq, error))std::cout << "\"s_vs2wq\" failed" << std::endl;

	s_wm2vs(rm, wm, result_vs_for_angle);
	if (!s_is_equal(6, result_vs_for_angle, vs, error))std::cout << "\"s_wm2vs\" failed" << std::endl;

	s_vs2wm(vs, rm, result);
	if (!s_is_equal(9, result, wm, error))std::cout << "\"s_vs2wm\" failed" << std::endl;

	s_wa2vs(wa, result_vs_for_angle);
	if (!s_is_equal(6, result_vs_for_angle, vs, error))std::cout << "\"s_wa2vs\" failed" << std::endl;

	s_vs2wa(vs, result);
	if (!s_is_equal(3, result, wa, error))std::cout << "\"s_vs2wa\" failed" << std::endl;

	s_ve2vs(pe313, ve313, result, "313");
	if (!s_is_equal(6, result, vs, error))std::cout << "\"s_ve2vs 313\" failed" << std::endl;

	s_ve2vs(pe321, ve321, result, "321");
	if (!s_is_equal(6, result, vs, error))std::cout << "\"s_ve2vs 321\" failed" << std::endl;

	s_vs2ve(vs, pe313, result, "313");
	if (!s_is_equal(3, result, ve313, error))std::cout << "\"s_vs2ve 313\" failed" << std::endl;

	s_vs2ve(vs, pe321, result, "321");
	if (!s_is_equal(3, result, ve321, error))std::cout << "\"s_vs2ve 321\" failed" << std::endl;

	s_vs2vq(vs, pq, result);
	if (!s_is_equal(7, result, vq, error))std::cout << "\"s_vs2vq\" failed" << std::endl;

	s_vq2vs(pq, vq, result);
	if (!s_is_equal(6, result, vs, error))std::cout << "\"s_vq2vs\" failed" << std::endl;

	s_vs2vm(vs, pm, result);
	if (!s_is_equal(16, result, vm, error))std::cout << "\"s_vs2vm\" failed" << std::endl;

	s_vm2vs(pm, vm, result);
	if (!s_is_equal(6, result, vs, error))std::cout << "\"s_vm2vs\" failed" << std::endl;

	s_vs2va(vs, pp, result);
	if (!s_is_equal(6, result, va, error))std::cout << "\"s_vs2va\" failed" << std::endl;

	s_va2vs(pp, va, result);
	if (!s_is_equal(6, result, vs, error))std::cout << "\"s_va2vs\" failed" << std::endl;


	s_xa2xe(wa, xa, re313, result, result1, "313");
	if (!(s_is_equal(3, result, xe313, error)&&(s_is_equal(3, result1, we313, error))))std::cout << "\"s_xa2xe 313\" failed" << std::endl;

	s_xa2xe(wa, xa, re321, result, result1, "321");
	if (!(s_is_equal(3, result, xe321, error)&&(s_is_equal(3, result1, we321, error))))std::cout << "\"s_xa2xe 321\" failed" << std::endl;

	s_xe2xa(re313, we313, xe313, result, result1, "313");
	if (!(s_is_equal(3, result, xa, error)&&(s_is_equal(3, result1, wa, error))))std::cout << "\"s_xe2xa 313\" failed" << std::endl;

	s_xe2xa(re321, we321, xe321, result, result1, "321");
	if (!(s_is_equal(3, result, xa, error)&&(s_is_equal(3, result1, wa, error))))std::cout << "\"s_xe2xa 321\" failed" << std::endl;

	s_xq2xa(rq, wq, xq, result);
	if (!s_is_equal(3, result, xa, error))std::cout << "\"s_xq2xa\" failed" << std::endl;

	s_xa2xq(wa, xa, rq, result);
	if (!s_is_equal(4, result, xq, error))std::cout << "\"s_xa2xq\" failed" << std::endl;

	s_xa2xm(wa, xa, rm, result);
	if (!s_is_equal(9, result, xm, error))std::cout << "\"s_xa2xm\" failed" << std::endl;

	s_xm2xa(rm, wm, xm, result);
	if (!s_is_equal(3, result, xa, error))std::cout << "\"s_xm2xa\" failed" << std::endl;

	s_as2ap(vs, as, pp, result);
	if (!s_is_equal(3, result, ap, error))std::cout << "\"s_as2ap\" failed" << std::endl;

	s_ap2as(pp, vp, ap, result_as_for_position, result_vs_for_position);
	if (!(s_is_equal(6, result_as_for_position, as, error) && (s_is_equal(6, result_vs_for_position, vs, error))))std::cout << "\"s_ap2as\" failed" << std::endl;

	s_xe2as(re313, we313, xe313, result, result1);
	if (!(s_is_equal(3, result + 3, as + 3, error) && (s_is_equal(3, result1 + 3, vs + 3, error))))std::cout << "\"s_xe2as 313\" failed" << std::endl;

	s_xe2as(re321, we321, xe321, result, result1, "321");
	if (!(s_is_equal(3, result + 3, as + 3, error) && (s_is_equal(3, result1 + 3, vs + 3, error))))std::cout << "\"s_xe2as 321\" failed" << std::endl;

	s_as2xe(vs, as, re313, result, result1);
	if (!s_is_equal(3, result, xe313, error))std::cout << "\"s_as2xe 313\" failed" << std::endl;

	s_as2xe(vs, as, re321, result, result1, "321");
	if (!s_is_equal(3, result, xe321, error))std::cout << "\"s_as2xe 321\" failed" << std::endl;

	s_xq2as(rq, wq, xq, result_as_for_angle);
	if (!s_is_equal(6, result_as_for_angle, as, error))std::cout << "\"s_xq2as\" failed" << std::endl;

	s_as2xq(vs, as, rq, result);
	if (!s_is_equal(3, result, xq, error))std::cout << "\"s_as2xq\" failed" << std::endl;

	s_xm2as(rm, wm, xm, result_as_for_angle);
	if (!s_is_equal(6, result_as_for_angle, as, error))std::cout << "\"s_xm2as\" failed" << std::endl;

	s_as2xm(vs, as, rm, result);
	if (!s_is_equal(9, result, xm, error))std::cout << "\"s_as2xm\" failed" << std::endl;

	s_xa2as(xa, result_as_for_angle);
	if (!s_is_equal(6, result_as_for_angle, as, error))std::cout << "\"s_xa2as\" failed" << std::endl;

	s_as2xa(as, result);
	if (!s_is_equal(3, result, xa, error))std::cout << "\"s_as2xa\" failed" << std::endl;

	s_as2ae(vs, as, pe313, result, result1);
	if (!(s_is_equal(6, result, ae313, error)&&s_is_equal(6, result1, ve313, error)))std::cout << "\"s_as2ae\" failed" << std::endl;

	s_as2ae(vs, as, pe321, result, result1, "321");
	if (!(s_is_equal(6, result, ae321, error) && s_is_equal(6, result1, ve321, error)))std::cout << "\"s_as2ae\" failed" << std::endl;

	s_ae2as(pe313, ve313, ae313, result, result1);
	if (!(s_is_equal(6, result, as, error) && s_is_equal(6, result1, vs, error)))std::cout << "\"s_ae2as\" failed" << std::endl;

	s_ae2as(pe321, ve321, ae321, result, result1, "321");
	if (!(s_is_equal(6, result, as, error) && s_is_equal(6, result1, vs, error)))std::cout << "\"s_ae2as\" failed" << std::endl;

	s_as2aq(vs, as, pq, result);
	if (!s_is_equal(7, result, aq, error))std::cout << "\"s_as2aq\" failed" << std::endl;

	s_aq2as(pq, vq, aq, result);
	if (!s_is_equal(6, result, as, error))std::cout << "\"s_aq2as\" failed" << std::endl;

	s_as2am(vs, as, pm, result);
	if (!s_is_equal(16, result, am, error))std::cout << "\"s_as2am\" failed" << std::endl;

	s_am2as(pm, vm, am, result);
	if (!s_is_equal(6, result, as, error))std::cout << "\"s_am2as\" failed" << std::endl;

	s_as2aa(vs, as, pp, result);
	if (!s_is_equal(6, result, aa, error))std::cout << "\"s_as2aa\" failed" << std::endl;

	s_aa2as(pp, va, aa, result);
	if (!s_is_equal(6, result, as, error))std::cout << "\"s_aa2as\" failed" << std::endl;
}
void test_coordinate_transform()
{
	const double relative_vs[16] = { 0.12, -0.35, 0.26, 0.58, 0.36, -0.135 };
	const double relative_as[16] = { 0.14, 1.35, -0.35, -0.56, -0.34, 0.14 };
	const double relative_pm[16] = { -0.22, -0.975499782797526,   0.000416847668728071,   0.1,
		0.175499782797526, -0.04, -0.983666521865018,   0.2,
		0.959583152331272, -0.216333478134982,   0.18,   0.3,
		0,   0,   0,   1 };

	const double from_pp[3]{ 0.13, -0.22, 0.45 };
	const double to_pp[3]{ 0.286197533666383, -0.21103496307558,   0.553339174992761 };
	const double from_re_313[3]{ 4.83, 0.76, 0.45 };
	const double to_re_321[3]{ 6.03703796978214, -0.696087712802565,   2.29525788843731 };
	const double from_re_321[3]{ 3.856, -0.696087712802565,   2.29525788843731 };
	const double to_re_313[3]{ 3.42785042695091,   2.77225969913703,   4.30966052384328 };
	const double from_rq[4]{ 0.4,-0.5, 0.6, std::sqrt(1 - 0.4*0.4 - 0.5*0.5 - 0.6*0.6) };
	const double to_rq[4]{ -0.383666521865017,   0.479583152331272, -0.575499782797526,   0.54 };
	const double from_rm[9]{ 0.808307066774345, -0.072065911490471, 0.584333971461272,
		0.341746746490328,   0.865601553329486, -0.365982393206091,
		-0.479425538604203,   0.495520388354132,   0.724300143351802 };
	const double to_rm[9]{ -0.511401279081528, -0.828333070215518,   0.228764194184996,
		0.599782696845049, -0.534698430872375, -0.595280021996288,
		0.615409983928647, -0.16721815933072,   0.770265304210822 };
	const double from_pe_313[6]{ 0.13, -0.22, 0.45, 4.83, 0.76, 0.45 };
	const double to_pe_321[6]{ 0.286197533666383, -0.21103496307558,   0.553339174992761,6.03703796978214, -0.696087712802565,   2.29525788843731 };
	const double from_pe_321[6]{ 0.13, -0.22, 0.45, 3.856, -0.696087712802565,   2.29525788843731 };
	const double to_pe_313[6]{ 0.286197533666383, -0.21103496307558,   0.553339174992761,3.42785042695091,   2.77225969913703,   4.30966052384328 };
	const double from_pq[7]{ 0.13, -0.22, 0.45, 0.4,-0.5, 0.6, std::sqrt(1 - 0.4*0.4 - 0.5*0.5 - 0.6*0.6) };
	const double to_pq[7]{ 0.286197533666383, -0.21103496307558,   0.553339174992761, -0.383666521865017,   0.479583152331272, -0.575499782797526,   0.54 };
	const double from_pm[16]{ 0.808307066774345, -0.072065911490471, 0.584333971461272, 0.13,
		0.341746746490328,   0.865601553329486, -0.365982393206091,-0.22,
		-0.479425538604203,   0.495520388354132,   0.724300143351802,0.45,
		0,0,0,1 };
	const double to_pm[16]{ -0.511401279081528, -0.828333070215518,   0.228764194184996, 0.286197533666383,
		0.599782696845049, -0.534698430872375, -0.595280021996288, -0.21103496307558,
		0.615409983928647, -0.16721815933072,   0.770265304210822,   0.553339174992761,
		0 ,  0 ,  0,   1 };

	const double from_vp[3]{ 0.131, -0.221, 0.451 };
	const double to_vp[3]{ 0.47766583327904, -1.12137651835541,   0.289263700919493 };
	const double from_we_313[3]{ 1.03, 0.73, 0.25 };
	const double to_we_321[3]{ -0.677310010504109, -0.42894848193491,   1.82130169173659 };
	const double from_we_321[3]{ 2.15, 0.76, 1.25 };
	const double to_we_313[3]{ -8.60099441931481, - 2.91077197886926, - 9.69351986665146 };
	const double from_wq[4]{ 0.1, 0.2, -0.4, -(from_rq[0] * 0.1 + from_rq[1] * 0.2 - from_rq[2] * 0.4) / from_rq[3] };
	const double to_wq[4]{ -0.292793710222811,   0.286847417856529, -0.141803596258453, -0.613907911417607 };
	const double from_wm[9] = { 0.0291954874793394, -0.182528252419306, -0.0628972223520089,
		0.196923491067634, -0.108112316607135, -0.0718182822377014,
		0.189595409314217,   0.162310423942327,   0.0144536170779726, };
	const double to_wm[9]{ 0.104075460683106,   0.0133045900952464,   0.280834839144192,
		-0.477150412775713,   0.0214429907194124, -0.500020299468546,
		0.551519952476166, -0.134472030683956, -0.469834287697671 };
	const double from_wa[3]{ -0.918, 0.928, 0.458 };
	const double to_wa[3]{ -0.123112882203827, -0.288748067622307, -1.13421480154937 };
	const double from_ve_313[6]{ 0.131, -0.221, 0.451, 1.03, 0.73, 0.25 };
	const double to_ve_321[6]{ 0.47766583327904, -1.12137651835541,   0.289263700919493, -0.677310010504109, -0.42894848193491,   1.82130169173659 };
	const double from_ve_321[6]{ 0.131, -0.221, 0.451, 2.15, 0.76, 1.25 };
	const double to_ve_313[6]{ 0.47766583327904, -1.12137651835541,   0.289263700919493, -8.60099441931481, -2.91077197886926, -9.69351986665146 };
	const double from_vq[7] = { 0.03,   0.1525,   0.355,    0.1, 0.2, -0.4, -(from_rq[0] * 0.1 + from_rq[1] * 0.2 - from_rq[2] * 0.4) / from_rq[3] };
	const double to_vq[7] = { 0.135496647027967, -1.05961001031892,   0.0942652484506193,   -0.292793710222811,   0.286847417856529, -0.141803596258453, -0.613907911417607 };
	const double from_vm[16] = { 0.0291954874793394, -0.182528252419306, -0.0628972223520089,   0.03,
		0.196923491067634, -0.108112316607135, -0.0718182822377014,   0.1525,
		0.189595409314217,   0.162310423942327,   0.0144536170779726,   0.355,
		0 ,  0,   0,   0 };
	const double to_vm[16] = { 0.104075460683106,   0.0133045900952464,   0.280834839144192,   0.135496647027967,
		-0.477150412775713,   0.0214429907194124, -0.500020299468546, -1.05961001031892,
		0.551519952476166, -0.134472030683956, -0.469834287697671,   0.0942652484506192,
		0,   0 ,  0,   0 };
	const double from_va[6] = { 0.089,   0.26325 ,  0.3935 ,  0.2, -0.15,   0.125 };
	const double to_va[6] = { 0.0144960947183866, -1.09155668422567,   0.133851721734715,   0.68237707337822,   0.278141641326378,   0.111866652186502 };
	const double from_vs[6] = { -0.1873, -0.1412,   0.1365,   0.04982,   0.1345,   0.03744 };
	const double to_vs[6] = { 0.314132747625686, -0.556683321739415,   0.160469757942957,   0.43785048599045,   0.326534924600346, -0.109551220160011 };
	
	const double from_ap[3]{ 0.12,   0.13, -0.14 };
	const double to_ap[3]{ -0.183017842291836,   1.44829117674852, -1.20119377113485 };
	const double from_xe_313[3]{ 4.83, 0.76, 0.45 };
	const double to_xe_321[3]{ -0.916190408904469, - 4.34620837330883,   2.91658574391217 };
	const double from_xe_321[3]{ 3.856, -0.696087712802565,   2.29525788843731 };
	const double to_xe_313[3]{ 188.69146781591,   49.4283048797986,   191.948584567729 };
	const double from_xq[4]{ -0.033,   0.022, 0.011,   -(from_wq[0] * from_wq[0] + from_wq[1] * from_wq[1] + from_wq[2] * from_wq[2] + from_wq[3] * from_wq[3] + from_rq[0] * (-0.033) + from_rq[1] * 0.022 + from_rq[2] * 0.011) / from_rq[3] };
	const double to_xq[4]{ 0.0195967544652706, - 0.915878348828718,   0.85596957486878,   0.693274013402014 };
	const double from_xm[9] = { -0.19607150371156, -0.0824023375945621,   0.195817097864919,
		0.0345175399147836,   0.0110332817274978,   0.210315261589722,
		-0.148327659454663, -0.175246744763661, -0.0645778320357833};
	const double to_xm[9]{ -0.168189089112595,   0.0240777134273968, -0.681683032081897,
		0.0646710887930607,   0.0754796609525905,   0.800601976050679,
		-1.08460897398942, -0.248679220742461,   0.107617812109963};
	const double from_xa[3]{ -0.16,   0.17,   0.18 };
	const double to_xa[3]{ -1.13785824818199,   0.122524884812844, - 0.141063237283511 };
	const double from_aa[6] = { -0.28023125, -0.165775,   0.1247, -0.16,   0.17,   0.18 };
	const double to_aa[6] = { 0.0898265507450043,   1.43483887623279, -1.10554368182179, -0.612738814129007, -0.708943502357538, -0.10224359010281 };
	const double from_ae_313[6]{ 0.12,   0.13, -0.14,4.83, 0.76, 0.45 };
	const double to_ae_321[6]{ -0.183017842291836,   1.44829117674852, -1.20119377113485,-0.916190408904469, -4.34620837330883,   2.91658574391217 };
	const double from_ae_321[6]{ 0.12,   0.13, -0.14,3.856, -0.696087712802565,   2.29525788843731 };
	const double to_ae_313[6]{ -0.183017842291836,   1.44829117674852, -1.20119377113485,188.69146781591,   49.4283048797986,   191.948584567729 };
	const double from_aq[7] = { -0.1873125, -0.14125,   0.136,   from_xq[0],   from_xq[1],   from_xq[2],   from_xq[3] };
	const double to_aq[7] = { 0.025588345140825,   1.4522998248645, -1.06971424926844,   to_xq[0],   to_xq[1],   to_xq[2],   to_xq[3] };
	const double from_am[16] = { -0.19607150371156, -0.0824023375945621,   0.195817097864919, -0.1873125,
		0.0345175399147836,   0.0110332817274978,   0.210315261589722, -0.14125,
		-0.148327659454663, -0.175246744763661, -0.0645778320357833,   0.136,
		0,   0,   0,   0 };
	const double to_am[16] = { -0.168189089112595,   0.0240777134273968, -0.681683032081897,   0.025588345140825,
		0.0646710887930607,   0.0754796609525905,   0.800601976050679,   1.4522998248645,
		-1.08460897398942, -0.248679220742461,   0.107617812109963, -1.06971424926844,
		0,   0,   0,   0 };
	const double from_as[6] = { -0.1899, -0.1475,   0.3165,   0.9482,   0.3145,   0.7344 };
	const double to_as[6] = { 0.627235974450473,   0.750818078071529, -0.640716977516731, -1.07044877319847, -0.904145907524959,   1.1457959474787 };

	double result[16], result2[16], result3[16];

	s_pp2pp(relative_pm, from_pp, result);
	if (!s_is_equal(3, to_pp, result, error))std::cout << "\"s_pp2pp\" failed" << std::endl;

	s_inv_pp2pp(relative_pm, to_pp, result);
	if (!s_is_equal(3, from_pp, result, error))std::cout << "\"s_inv_pp2pp\" failed" << std::endl;

	s_re2re(relative_pm, from_re_313, result, "313", "321");
	if (!s_is_equal(3, to_re_321, result, error))std::cout << "\"s_re2re\" failed" << std::endl;

	s_inv_re2re(relative_pm, to_re_321, result, "321", "313");
	if (!s_is_equal(3, from_re_313, result, error))std::cout << "\"s_inv_re2re\" failed" << std::endl;

	s_re2re(relative_pm, from_re_321, result, "321", "313");
	if (!s_is_equal(3, to_re_313, result, error))std::cout << "\"s_re2re\" failed" << std::endl;

	s_inv_re2re(relative_pm, to_re_313, result, "313", "321");
	if (!s_is_equal(3, from_re_321, result, error))std::cout << "\"s_inv_re2re\" failed" << std::endl;

	s_rq2rq(relative_pm, from_rq, result);
	if (!s_is_equal(4, to_rq, result, error))std::cout << "\"s_rq2rq\" failed" << std::endl;

	s_inv_rq2rq(relative_pm, to_rq, result);
	if (!s_is_equal(4, from_rq, result, error))std::cout << "\"s_inv_rq2rq\" failed" << std::endl;

	s_rm2rm(relative_pm, from_rm, result);
	if (!s_is_equal(3, to_rm, result, error))std::cout << "\"s_rm2rm\" failed" << std::endl;

	s_inv_rm2rm(relative_pm, to_rm, result);
	if (!s_is_equal(3, from_rm, result, error))std::cout << "\"s_inv_rm2rm\" failed" << std::endl;

	s_pe2pe(relative_pm, from_pe_313, result, "313", "321");
	if (!s_is_equal(6, to_pe_321, result, error))std::cout << "\"s_pe2pe 313 to 321\" failed" << std::endl;

	s_inv_pe2pe(relative_pm, to_pe_321, result, "321", "313");
	if (!s_is_equal(6, from_pe_313, result, error))std::cout << "\"s_inv_pe2pe 321 to 313\" failed" << std::endl;

	s_pe2pe(relative_pm, from_pe_321, result, "321", "313");
	if (!s_is_equal(6, to_pe_313, result, error))std::cout << "\"s_pe2pe 321 to 313\" failed" << std::endl;

	s_inv_pe2pe(relative_pm, to_pe_313, result, "313", "321");
	if (!s_is_equal(6, from_pe_321, result, error))std::cout << "\"s_inv_pe2pe 313 to 321\" failed" << std::endl;

	s_pq2pq(relative_pm, from_pq, result);
	if (!s_is_equal(7, to_pq, result, error))std::cout << "\"s_pq2pq\" failed" << std::endl;

	s_inv_pq2pq(relative_pm, to_pq, result);
	if (!s_is_equal(7, from_pq, result, error))std::cout << "\"s_inv_pq2pq\" failed" << std::endl;

	s_pm2pm(relative_pm, from_pm, result);
	if (!s_is_equal(16, to_pm, result, error))std::cout << "\"s_pm2pm\" failed" << std::endl;

	s_inv_pm2pm(relative_pm, to_pm, result);
	if (!s_is_equal(16, from_pm, result, error))std::cout << "\"s_inv_pm2pm\" failed" << std::endl;



	s_vp2vp(relative_pm, relative_vs, from_pp, from_vp, result, result2);
	if (!(s_is_equal(3, to_vp, result, error) && s_is_equal(3, to_pp, result2, error)))std::cout << "\"s_vp2vp\" failed" << std::endl;

	s_inv_vp2vp(relative_pm, relative_vs, to_pp, to_vp, result, result2);
	if (!(s_is_equal(3, from_vp, result, error) && s_is_equal(3, from_pp, result2, error)))std::cout << "\"s_inv_vp2vp\" failed" << std::endl;

	s_we2we(relative_pm, relative_vs, from_re_313, from_we_313, result, result2, "313", "321");
	if (!(s_is_equal(3, to_we_321, result, error) && s_is_equal(3, to_re_321, result2, error)))std::cout << "\"s_we2we 313 to 321\" failed" << std::endl;

	s_inv_we2we(relative_pm, relative_vs, to_re_321, to_we_321, result, result2, "321", "313");
	if (!(s_is_equal(3, from_we_313, result, error) && s_is_equal(3, from_re_313, result2, error)))std::cout << "\"s_inv_we2we 313\" failed" << std::endl;

	s_we2we(relative_pm, relative_vs, from_re_321, from_we_321, result, result2, "321", "313");
	if (!(s_is_equal(3, to_we_313, result, error) && s_is_equal(3, to_re_313, result2, error)))std::cout << "\"s_we2we 313 to 321\" failed" << std::endl;

	s_inv_we2we(relative_pm, relative_vs, to_re_313, to_we_313, result, result2, "313", "321");
	if (!(s_is_equal(3, from_we_321, result, error) && s_is_equal(3, from_re_321, result2, error)))std::cout << "\"s_inv_we2we 313\" failed" << std::endl;

	s_wq2wq(relative_pm, relative_vs, from_rq, from_wq, result, result2);
	if (!(s_is_equal(4, to_wq, result, error) && s_is_equal(4, to_rq, result2, error)))std::cout << "\"s_wq2wq\" failed" << std::endl;

	s_inv_wq2wq(relative_pm, relative_vs, to_rq, to_wq, result, result2);
	if (!(s_is_equal(4, from_wq, result, error) && s_is_equal(4, from_rq, result2, error)))std::cout << "\"s_inv_wq2wq\" failed" << std::endl;

	s_wm2wm(relative_pm, relative_vs, from_rm, from_wm, result, result2);
	if (!(s_is_equal(9, to_wm, result, error) && s_is_equal(9, to_rm, result2, error)))std::cout << "\"s_wm2wm\" failed" << std::endl;

	s_inv_wm2wm(relative_pm, relative_vs, to_rm, to_wm, result, result2);
	if (!(s_is_equal(9, from_wm, result, error) && s_is_equal(9, from_rm, result2, error)))std::cout << "\"s_inv_wm2wm\" failed" << std::endl;

	s_wa2wa(relative_pm, relative_vs, from_wa, result);
	if (!s_is_equal(3, to_wa, result, error))std::cout << "\"s_wa2wa\" failed" << std::endl;

	s_inv_wa2wa(relative_pm, relative_vs, to_wa, result);
	if (!s_is_equal(3, from_wa, result, error))std::cout << "\"s_inv_wa2wa\" failed" << std::endl;

	s_ve2ve(relative_pm, relative_vs, from_pe_313, from_ve_313, result, result2, "313", "321");
	if (!(s_is_equal(6, to_ve_321, result, error) && s_is_equal(3, to_pe_321, result2, error)))std::cout << "\"s_ve2ve 313 to 321\" failed" << std::endl;

	s_inv_ve2ve(relative_pm, relative_vs, to_pe_321, to_ve_321, result, result2, "321", "313");
	if (!(s_is_equal(6, from_ve_313, result, error) && s_is_equal(3, from_pe_313, result2, error)))std::cout << "\"s_inv_ve2ve 313\" failed" << std::endl;

	s_ve2ve(relative_pm, relative_vs, from_pe_321, from_ve_321, result, result2, "321", "313");
	if (!(s_is_equal(6, to_ve_313, result, error) && s_is_equal(3, to_pe_313, result2, error)))std::cout << "\"s_ve2ve 313 to 321\" failed" << std::endl;

	s_inv_ve2ve(relative_pm, relative_vs, to_pe_313, to_ve_313, result, result2, "313", "321");
	if (!(s_is_equal(6, from_ve_321, result, error) && s_is_equal(3, from_pe_321, result2, error)))std::cout << "\"s_inv_ve2ve 313\" failed" << std::endl;

	s_vq2vq(relative_pm, relative_vs, from_pq, from_vq, result, result2);
	if (!(s_is_equal(7, to_vq, result, error) && s_is_equal(7, to_pq, result2, error)))std::cout << "\"s_vq2vq\" failed" << std::endl;

	s_inv_vq2vq(relative_pm, relative_vs, to_pq, to_vq, result, result2);
	if (!(s_is_equal(7, from_vq, result, error) && s_is_equal(7, from_pq, result2, error)))std::cout << "\"s_inv_vq2vq\" failed" << std::endl;

	s_vm2vm(relative_pm, relative_vs, from_pm, from_vm, result, result2);
	if (!(s_is_equal(16, to_vm, result, error) && s_is_equal(16, to_pm, result2, error)))std::cout << "\"s_vm2vm\" failed" << std::endl;

	s_inv_vm2vm(relative_pm, relative_vs, to_pm, to_vm, result, result2);
	if (!(s_is_equal(16, from_vm, result, error) && s_is_equal(16, from_pm, result2, error)))std::cout << "\"s_inv_vm2vm\" failed" << std::endl;

	s_va2va(relative_pm, relative_vs, from_pp, from_va, result, result2);
	if (!(s_is_equal(6, to_va, result, error) && s_is_equal(3, to_pp, result2, error)))std::cout << "\"s_va2va\" failed" << std::endl;

	s_inv_va2va(relative_pm, relative_vs, to_pp, to_va, result, result2);
	if (!(s_is_equal(6, from_va, result, error) && s_is_equal(3, from_pp, result2, error)))std::cout << "\"s_inv_va2va\" failed" << std::endl;

	s_vs2vs(relative_pm, relative_vs, from_vs, result);
	if (!s_is_equal(6, to_vs, result, error))std::cout << "\"s_vs2vs\" failed" << std::endl;

	s_inv_vs2vs(relative_pm, relative_vs, to_vs, result);
	if (!s_is_equal(6, from_vs, result, error))std::cout << "\"s_inv_vs2vs\" failed" << std::endl;



	s_ap2ap(relative_pm, relative_vs, relative_as, from_pp, from_vp, from_ap, result, result2, result3);
	if (!(s_is_equal(3, to_ap, result, error) && s_is_equal(3, to_vp, result2, error) && s_is_equal(3, to_pp, result3, error)))std::cout << "\"s_ap2ap\" failed" << std::endl;

	s_inv_ap2ap(relative_pm, relative_vs, relative_as, to_pp, to_vp, to_ap, result, result2, result3);
	if (!(s_is_equal(3, from_ap, result, error) && s_is_equal(3, from_vp, result2, error) && s_is_equal(3, from_pp, result3, error)))std::cout << "\"s_inv_ap2ap\" failed" << std::endl;

	s_xe2xe(relative_pm, relative_vs, relative_as, from_re_313, from_we_313, from_xe_313, result, result2, result3, "313", "321");
	if (!(s_is_equal(3, to_xe_321, result, error) && s_is_equal(3, to_we_321, result2, error) && s_is_equal(3, to_re_321, result3, error)))std::cout << "\"s_xe2xe 313 to 321\" failed" << std::endl;

	s_inv_xe2xe(relative_pm, relative_vs, relative_as, to_re_321, to_we_321, to_xe_321, result, result2, result3, "321", "313");
	if (!(s_is_equal(3, from_xe_313, result, error) && s_is_equal(3, from_we_313, result2, error) && s_is_equal(3, from_re_313, result3, error)))std::cout << "\"s_inv_xe2xe 321 to 313\" failed" << std::endl;
	
	s_xe2xe(relative_pm, relative_vs, relative_as, from_re_321, from_we_321, from_xe_321, result, result2, result3, "321", "313");
	if (!(s_is_equal(3, to_xe_313, result, error) && s_is_equal(3, to_we_313, result2, error) && s_is_equal(3, to_re_313, result3, error)))std::cout << "\"s_xe2xe 321 to 313\" failed" << std::endl;

	s_inv_xe2xe(relative_pm, relative_vs, relative_as, to_re_313, to_we_313, to_xe_313, result, result2, result3, "313", "321");
	if (!(s_is_equal(3, from_xe_321, result, error) && s_is_equal(3, from_we_321, result2, error) && s_is_equal(3, from_re_321, result3, error)))std::cout << "\"s_inv_xe2xe 313 to 321\" failed" << std::endl;

	s_xq2xq(relative_pm, relative_vs, relative_as, from_rq, from_wq, from_xq, result, result2, result3);
	if (!(s_is_equal(4, to_xq, result, error) && s_is_equal(4, to_wq, result2, error) && s_is_equal(4, to_rq, result3, error)))std::cout << "\"s_xq2xq\" failed" << std::endl;

	s_inv_xq2xq(relative_pm, relative_vs, relative_as, to_rq, to_wq, to_xq, result, result2, result3);
	if (!(s_is_equal(4, from_xq, result, error) && s_is_equal(4, from_wq, result2, error) && s_is_equal(4, from_rq, result3, error)))std::cout << "\"s_inv_xq2xq\" failed" << std::endl;

	s_xm2xm(relative_pm, relative_vs, relative_as, from_rm, from_wm, from_xm, result, result2, result3);
	if (!(s_is_equal(9, to_xm, result, error) && s_is_equal(9, to_wm, result2, error) && s_is_equal(9, to_rm, result3, error)))std::cout << "\"s_xm2xm\" failed" << std::endl;

	s_inv_xm2xm(relative_pm, relative_vs, relative_as, to_rm, to_wm, to_xm, result, result2, result3);
	if (!(s_is_equal(9, from_xm, result, error) && s_is_equal(9, from_wm, result2, error) && s_is_equal(9, from_rm, result3, error)))std::cout << "\"s_inv_xm2xm\" failed" << std::endl;

	s_xa2xa(relative_pm, relative_vs, relative_as, from_wa, from_xa, result, result2);
	if (!s_is_equal(3, to_xa, result, error))std::cout << "\"s_xa2xa\" failed" << std::endl;

	s_inv_xa2xa(relative_pm, relative_vs, relative_as, to_wa, to_xa, result, result2);
	if (!s_is_equal(3, from_xa, result, error))std::cout << "\"s_inv_xa2xa\" failed" << std::endl;
	
	s_ae2ae(relative_pm, relative_vs, relative_as, from_pe_313, from_ve_313, from_ae_313, result, result2, result3, "313", "321");
	if (!(s_is_equal(6, to_ae_321, result, error) && s_is_equal(6, to_ve_321, result2, error) && s_is_equal(6, to_pe_321, result3, error)))std::cout << "\"s_ae2ae 313 to 321\" failed" << std::endl;

	s_inv_ae2ae(relative_pm, relative_vs, relative_as, to_pe_321, to_ve_321, to_ae_321, result, result2, result3, "321", "313");
	if (!(s_is_equal(6, from_ae_313, result, error) && s_is_equal(6, from_ve_313, result2, error) && s_is_equal(6, from_pe_313, result3, error)))std::cout << "\"s_inv_ae2ae 321 to 313\" failed" << std::endl;
	
	s_ae2ae(relative_pm, relative_vs, relative_as, from_pe_321, from_ve_321, from_ae_321, result, result2, result3, "321", "313");
	if (!(s_is_equal(6, to_ae_313, result, error) && s_is_equal(6, to_ve_313, result2, error) && s_is_equal(6, to_pe_313, result3, error)))std::cout << "\"s_ae2ae 321 to 313\" failed" << std::endl;

	s_inv_ae2ae(relative_pm, relative_vs, relative_as, to_pe_313, to_ve_313, to_ae_313, result, result2, result3, "313", "321");
	if (!(s_is_equal(6, from_ae_321, result, error) && s_is_equal(6, from_ve_321, result2, error) && s_is_equal(6, from_pe_321, result3, error)))std::cout << "\"s_inv_ae2ae 313 to 321\" failed" << std::endl;

	s_aq2aq(relative_pm, relative_vs, relative_as, from_pq, from_vq, from_aq, result, result2, result3);
	if (!(s_is_equal(7, to_aq, result, error) && s_is_equal(7, to_vq, result2, error) && s_is_equal(7, to_pq, result3, error)))std::cout << "\"s_aq2aq\" failed" << std::endl;

	s_inv_aq2aq(relative_pm, relative_vs, relative_as, to_pq, to_vq, to_aq, result, result2, result3);
	if (!(s_is_equal(7, from_aq, result, error) && s_is_equal(7, from_vq, result2, error) && s_is_equal(7, from_pq, result3, error)))std::cout << "\"s_inv_aq2aq\" failed" << std::endl;

	s_am2am(relative_pm, relative_vs, relative_as, from_pm, from_vm, from_am, result, result2, result3);
	if (!(s_is_equal(16, to_am, result, error) && s_is_equal(16, to_vm, result2, error) && s_is_equal(16, to_pm, result3, error)))std::cout << "\"s_am2am\" failed" << std::endl;

	s_inv_am2am(relative_pm, relative_vs, relative_as, to_pm, to_vm, to_am, result, result2, result3);
	if (!(s_is_equal(16, from_am, result, error) && s_is_equal(16, from_vm, result2, error) && s_is_equal(16, from_pm, result3, error)))std::cout << "\"s_inv_am2am\" failed" << std::endl;

	s_aa2aa(relative_pm, relative_vs, relative_as, from_pp, from_va, from_aa, result, result2, result3);
	if (!(s_is_equal(6, to_aa, result, error) && s_is_equal(6, to_va, result2, error) && s_is_equal(3, to_pp, result3, error)))std::cout << "\"s_aa2aa\" failed" << std::endl;

	s_inv_aa2aa(relative_pm, relative_vs, relative_as, to_pp, to_va, to_aa, result, result2, result3);
	if (!(s_is_equal(6, from_aa, result, error) && s_is_equal(6, from_va, result2, error) && s_is_equal(3, from_pp, result3, error)))std::cout << "\"s_inv_aa2aa\" failed" << std::endl;

	s_as2as(relative_pm, relative_vs, relative_as, from_vs, from_as, result, result2);
	if (!(s_is_equal(6, to_as, result, error) && s_is_equal(6, to_vs, result2, error)))std::cout << "\"s_as2as\" failed" << std::endl;

	s_inv_as2as(relative_pm, relative_vs, relative_as, to_vs, to_as, result, result2);
	if (!(s_is_equal(6, from_as, result, error) && s_is_equal(6, from_vs, result2, error)))std::cout << "\"s_inv_as2as\" failed" << std::endl;

}