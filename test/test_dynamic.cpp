#include <iostream>
#include "aris_dynamic_kernel.h"

using namespace aris::dynamic;

int main(int argc, char *argv[])
{
	const double error = 1e-10;
	
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
		double a[] = { 0.1, 0.2, 0.3 };
		double result[9];
		double answer[9] = {
			0,  -0.3, 0.2,
			0.3, 0,  -0.1,
		   -0.2, 0.1, 0};

		s_cm3(a, result);

		if (!s_is_equal(9, result, answer, error))
		{
			std::cout << "\"s_cro3\" failed" << std::endl;
		}
	}

	//test s_re2rm & s_rm2re & s_we2wm & s_wm2we & s_xe2xm & s_xm2xe
	{
		const double we[3] = { -0.244517963270725,	1.25737650310373,	-0.874318412470487 };
		const double xe[3] = { 0.904633672502324, -1.24440604199266,   1.45568007018557 };
		const double rm321[9] = { -0.22, -0.975499782797526,   0.000416847668728071,
			0.175499782797526, -0.04, -0.983666521865018,
			0.959583152331272, -0.216333478134982,   0.18 };
		const double rm313[9] = { 0.567219713641686, -0.802125918959455, 0.186697098503681,
			0.777805328452570,   0.447242474005492, -0.441580163137156,
			0.270704021926224,   0.395686971707304,0.877582561890373 };
		const double wm[9] = { 1.36, - 0.30698536874045, - 0.633709981238717,
			0.426985368740452,   0.8,   0.0436487757967661,
			0.233709981238715,   1.23635122420323,   0.24 , };
		const double xm[9] = { -0.782400000000002,   2.58144759895694,   1.54784395313479,
			- 2.32024759895695, - 0.653600000000002,   0.450521351741563,
			- 1.92944395313478, - 1.05972135174157, - 0.103200000000001 };
		const double re[3] = { 2.46823966120654, - 1.28551725555848,  5.40636866254317 };
		const double re313[3] = { 0.4, 0.5, 0.6 };
		const double rm[9] = { -0.22, -0.975499782797526,   0.000416847668728071,
			0.175499782797526, -0.04, -0.983666521865018,
			0.959583152331272, -0.216333478134982,   0.18 };
		const double rq[4] = { 0.4,-0.5, 0.6, std::sqrt(1 - 0.4*0.4 - 0.5*0.5 - 0.6*0.6) };
		const double wq[4] = { 0.1, 0.2, -0.4, -(rq[0] * 0.1 + rq[1] * 0.2 - rq[2] * 0.4) / rq[3] };
		const double xq[4] = { -0.033,   0.022, 0.011,   -(wq[0] * wq[0] + wq[1] * wq[1] + wq[2] * wq[2] + wq[3] * wq[3] + rq[0] * (-0.033) + rq[1] * (0.022) + rq[2] * (0.011)) / rq[3] };
		
		
		double result[16];

		s_re2rm(re, result, "321");
		if (!s_is_equal(9, result, rm321, error))std::cout << "\"s_re2rm 321\" failed" << std::endl;

		s_rm2re(rm321, result, "321");
		if (!s_is_equal(3, result, re, error))std::cout << "\"s_rm2re 321\" failed" << std::endl;

		s_re2rm(re313, result);
		if (!s_is_equal(9, result, rm313, error))std::cout << "\"s_re2rm 313\" failed" << std::endl;

		s_rm2re(rm313, result);
		if (!s_is_equal(3, result, re313, error))std::cout << "\"s_rm2re 313\" failed" << std::endl;

		s_we2wm(we, rm, result);
		if (!s_is_equal(9, result, wm, error))std::cout << "\"s_we2wm\" failed" << std::endl;

		s_wm2we(rm, wm, result);
		if (!s_is_equal(3, result, we, error))std::cout << "\"s_wm2we\" failed" << std::endl;

		s_xe2xm(we, xe, rm, result);
		if (!s_is_equal(9, result, xm, error))std::cout << "\"s_xe2xm\" failed" << std::endl;

		s_xm2xe(rm, wm, xm, result);
		if (!s_is_equal(3, result, xe, error))std::cout << "\"s_xm2xe\" failed" << std::endl;

		s_rq2rm(rq, result);
		if (!s_is_equal(9, result, rm, error))std::cout << "\"s_rq2rm\" failed" << std::endl;

		s_rm2rq(rm, result);
		if (!s_is_equal(4, result, rq, error))std::cout << "\"s_rm2rq\" failed" << std::endl;

		s_wq2we(rq, wq, result);
		if (!s_is_equal(3, result, we, error))std::cout << "\"s_wq2we\" failed" << std::endl;

		s_we2wq(we, rq, result);
		if (!s_is_equal(4, result, wq, error))std::cout << "\"s_we2wq\" failed" << std::endl;

		s_xq2xe(rq, wq, xq, result);
		if (!s_is_equal(3, result, xe, error))std::cout << "\"s_xq2xe\" failed" << std::endl;

		s_xe2xq(we, xe, rq, result);
		if (!s_is_equal(4, result, xq, error))std::cout << "\"s_xe2xq\" failed" << std::endl;
	}

	//test s_pp2pm & s_pm2pp & s_vs2vp & s_vp2vs & s_as2ap & s_ap2as
	{
		const double pm[16] = { 1,0,0,0.4,0,1,0,-0.5,0,0,1,0.6,0,0,0,1 };
		const double pp[3] = { 0.4,-0.5, 0.6 };
		const double vp[3] = { 0.668482036729275,   1.20537650310373, - 1.52631841247049 };
		const double ap[3] = { 0.12,   0.13, -0.14 };
		const double vs[6] = { -0.244517963270725,	1.25737650310373,	-0.874318412470487, 0.32, 1.23, 0.35 };
		const double as[6] = { 3.15925342342501, - 0.192390604845803,   0.136512424183815,   0.36 ,- 0.85, - 0.46 };

		double result_pm[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
		double result_vs[6]{ -0.244517963270725,	1.25737650310373,	-0.874318412470487, 0.32, 1.23, 0.35 };
		double result_as[6]{ 0,	0,	0, 0.36, -0.85, -0.46 };

		s_pp2pm(pp, result_pm);
		if (!s_is_equal(16, result_pm, pm, error))std::cout << "\"s_pp2pm\" failed" << std::endl;

		s_pm2pp(pm, result_pm);
		if (!s_is_equal(3, result_pm, pp, error))std::cout << "\"s_pm2pp\" failed" << std::endl;

		s_vs2vp(vs, pp, result_vs);
		if (!s_is_equal(3, result_vs, vp, error))std::cout << "\"s_vs2vp\" failed" << std::endl;

		s_vp2vs(pp, vp, result_vs);
		if (!s_is_equal(6, result_vs, vs, error))std::cout << "\"s_vp2vs\" failed" << std::endl;

		s_ap2as(pp, vp, ap, result_as, result_vs);
		if (!(s_is_equal(6, result_as, as, error)&&(s_is_equal(6, result_vs, vs, error))))std::cout << "\"s_ap2as\" failed" << std::endl;

		s_as2ap(vs, as, pp, result_as);
		if (!s_is_equal(3, result_as, ap, error))std::cout << "\"s_as2ap\" failed" << std::endl;
	}

	//test s_re2pm & s_pm2re & s_vs2we & s_we2vs & s_as2xe & s_xe2as
	{
		const double re[3] = { 0.4,0.5,0.6 };
		const double pm321[16] = { 0.808307066774345, -0.072065911490471, 0.584333971461272, 0.1,
			0.341746746490328,   0.865601553329486, -0.365982393206091, 0.2,
			-0.479425538604203,   0.495520388354132,   0.724300143351802, 0.3,
			0,0,0,1 };
		const double pm313[16] =
		{
			0.567219713641686, -0.802125918959455, 0.186697098503681,0.1,
			0.777805328452570,   0.447242474005492, -0.441580163137156,0.2,
			0.270704021926224,   0.395686971707304, 0.877582561890373,0.3,
			0,0,0,1
		};
		const double we[] = { 0.4,-0.5, 0.6 };
		const double vs[] = { 0.1,0.2,0.3, 0.4,-0.5, 0.6 };
		const double xe[] = { 0.44,-0.55, 0.66 };
		const double as[] = { 0.11,0.22,0.33, 0.44,-0.55, 0.66 };

		double result_pm[16]{ 1,0,0,0.1,0,1,0,0.2,0,0,1,0.3,0,0,0,1 };
		double result_vs[6]{ 0.1,0.2,0.3, 0.32, 1.23, 0.35 };
		double result_as[6]{ 0.11,0.22,0.33, 0.32, 1.23, 0.35 };

		s_re2pm(re, result_pm, "321");
		if (!s_is_equal(16, result_pm, pm321, error))std::cout << "\"s_re2pm\" failed" << std::endl;

		s_pm2re(pm321, result_pm, "321");
		if (!s_is_equal(3, result_pm, re, error))std::cout << "\"s_pm2re\" failed" << std::endl;

		s_re2pm(re, result_pm);
		if (!s_is_equal(16, result_pm, pm313, error))std::cout << "\"s_re2pm\" failed" << std::endl;

		s_pm2re(pm313, result_pm);
		if (!s_is_equal(3, result_pm, re, error))std::cout << "\"s_pm2re\" failed" << std::endl;

		s_we2vs(we, result_vs);
		if (!s_is_equal(6, result_vs, vs, error))std::cout << "\"s_wq2vs\" failed" << std::endl;

		s_vs2we(vs, result_vs);
		if (!s_is_equal(3, result_vs, we, error))std::cout << "\"s_vs2wq\" failed" << std::endl;

		s_xe2as(xe, result_as);
		if (!s_is_equal(6, result_as, as, error))std::cout << "\"s_xe2as\" failed" << std::endl;

		s_as2xe(as, result_as);
		if (!s_is_equal(3, result_as, xe, error))std::cout << "\"s_as2xe\" failed" << std::endl;
	}

	//test s_rq2pm & s_pm2rq & s_vs2wq & s_wq2vs & s_as2xq & s_xq2as
	{
		const double pm[16] =
		{
			-0.22, -0.975499782797526,   0.000416847668728071, 0.1,
			0.175499782797526, -0.04, -0.983666521865018, 0.2,
			0.959583152331272, -0.216333478134982,   0.18,0.3,
			0,0,0,1
		};
		const double rq[4] = { 0.4,-0.5, 0.6, std::sqrt(1 - 0.4*0.4 - 0.5*0.5 - 0.6*0.6) };
		const double wq[4] = { 0.1, 0.2, -0.4, -(rq[0] * 0.1 + rq[1] * 0.2 - rq[2] * 0.4) / rq[3] };
		const double xq[4] = { -0.033,   0.022, 0.011,   -(wq[0] * wq[0] + wq[1] * wq[1] + wq[2] * wq[2] + wq[3] * wq[3] + rq[0] * (-0.033) + rq[1] * (0.022) + rq[2] * (0.011)) / rq[3] };
		const double vs[6] = { 0.533233304373004,   0.268943638683732, - 0.242072948342027, - 0.244517963270725,   1.25737650310373, - 0.874318412470487 };
		const double as[6] = { 0.1, 0.2, -0.4, 0.904633672502324, -1.24440604199266,   1.45568007018557 };

		double result_pm[16]{ 1,0,0,0.1,0,1,0,0.2,0,0,1,0.3,0,0,0,1 };
		double result_vs[6]{ 0.533233304373004,   0.268943638683732, -0.242072948342027,0,0,0 };
		double result_as[6]{ 0.1, 0.2, -0.4, 0,0,0 };

		s_rq2pm(rq, result_pm);
		if (!s_is_equal(16, result_pm, pm, error))std::cout << "\"s_rq2pm\" failed" << std::endl;

		s_pm2rq(pm, result_pm);
		if (!s_is_equal(4, result_pm, rq, error))std::cout << "\"s_pm2rq\" failed" << std::endl;

		s_wq2vs(rq, wq, result_vs);
		if (!s_is_equal(6, result_vs, vs, error))std::cout << "\"s_wq2vs\" failed" << std::endl;

		s_vs2wq(vs, rq, result_vs);
		if (!s_is_equal(3, result_vs, wq, error))std::cout << "\"s_vs2wq\" failed" << std::endl;

		s_xq2as(rq, wq, xq, result_as);
		if (!s_is_equal(6, result_as, as, error))std::cout << "\"s_xq2as\" failed" << std::endl;

		s_as2xq(vs, as, rq, result_as);
		if (!s_is_equal(3, result_as, xq, error))std::cout << "\"s_as2xq\" failed" << std::endl;
	}

	//test s_rm2pm & s_pm2rm & s_vs2wm & s_wm2vs & s_as2xm & s_xm2as
	{
		const double rm[9] =
		{
			0.808307066774345, -0.072065911490471, 0.584333971461272,
			0.341746746490328,   0.865601553329486, -0.365982393206091,
			-0.479425538604203,   0.495520388354132,   0.724300143351802
		};
		const double wm[9] =
		{
			0.0291954874793394, -0.182528252419306, -0.0628972223520089,
			0.196923491067634, -0.108112316607135, -0.0718182822377014,
			0.189595409314217,   0.162310423942327,   0.0144536170779726,
		};
		const double xm[9] =
		{
			-0.0823928145196289,   0.181593663160609,   0.0711244067844878,
			- 0.0363008673159893, - 0.154707835550555, - 0.247815325951341,
			- 0.00714759685349801,   0.152672922740678, - 0.19547046585364,
		};
		const double relative_pm[16] = { 0.808307066774345, -0.072065911490471, 0.584333971461272, 0.1,
			0.341746746490328,   0.865601553329486, -0.365982393206091, 0.2,
			-0.479425538604203,   0.495520388354132,   0.724300143351802, 0.3,
			0,0,0,1 };
		const double vs[6] = { 0.1,0.2,0.3, 0.2,-0.15,0.125 };
		const double as[6] = { 0.0291954874793394, -0.182528252419306, -0.0628972223520089, 0.22,0.156,-0.133 };
		
		double result_pm[16]{ 1,0,0,0.1,0,1,0,0.2,0,0,1,0.3,0,0,0,1 };
		double result_vs[9]{ 0.1,0.2,0.3 };
		double result_as[9]{ 0.0291954874793394, -0.182528252419306, -0.0628972223520089 };

		s_rm2pm(rm, result_pm);
		if (!s_is_equal(16, result_pm, relative_pm, error))std::cout << "\"s_rm2pm\" failed" << std::endl;

		s_pm2rm(relative_pm, result_pm);
		if (!s_is_equal(9, result_pm, rm, error))std::cout << "\"s_pm2rm\" failed" << std::endl;

		s_wm2vs(rm, wm, result_vs);
		if (!s_is_equal(6, result_vs, vs, error))std::cout << "\"s_wm2vs\" failed" << std::endl;

		s_vs2wm(vs, rm, result_vs);
		if (!s_is_equal(9, result_vs, wm, error))std::cout << "\"s_vs2wm\" failed" << std::endl;

		s_xm2as(rm, wm, xm, result_as);
		if (!s_is_equal(6, result_as, as, error))std::cout << "\"s_xm2as\" failed" << std::endl;

		s_as2xm(vs, as, rm, result_as);
		if (!s_is_equal(9, result_as, xm, error))std::cout << "\"s_as2xm\" failed" << std::endl;
	}

	//test s_pe2pm & s_pm2pe & s_vs2ve & s_ve2vs & s_as2ae & s_ae2as
	{
		const double pe[] = { 0.1,0.2,0.3,0.4,0.5,0.6 };
		const double pm321[16] =
		{
			0.808307066774345, -0.072065911490471, 0.584333971461272, 0.1,
			0.341746746490328,   0.865601553329486, -0.365982393206091, 0.2,
			-0.479425538604203,   0.495520388354132,   0.724300143351802, 0.3,
			0,0,0,1
		};
		const double pm313[16] =
		{
			0.567219713641686, -0.802125918959455, 0.186697098503681,0.1,
			0.777805328452570,   0.447242474005492, -0.441580163137156,0.2,
			0.270704021926224,   0.395686971707304,0.877582561890373,0.3,
			0,0,0,1
		};
		
		const double pp[3] = { 0.33,0.22,-0.11 };
		const double ve[6] = { 0.089,   0.26325 ,  0.3935 ,  0.2, - 0.15,   0.125};
		const double ae[6] = { -0.28023125, - 0.165775,   0.1247, - 0.16,   0.17,   0.18 };
		const double vs[6] = { 0.1,0.2,0.3, 0.2,-0.15,0.125 };
		const double as[6] = { -0.13,-0.14,0.15,-0.16,0.17,0.18 };
		
		double result[16];

		s_pe2pm(pe, result, "321");
		if (!s_is_equal(16, pm321, result, error))std::cout << "\"s_pe2pm\" failed" << std::endl;

		s_pm2pe(pm321, result, "321");
		if (!s_is_equal(6, result, pe, error))std::cout << "\"s_pm2pe\" failed" << std::endl;

		s_pe2pm(pe, result);
		if (!s_is_equal(16, result, pm313, error))std::cout << "\"s_pe2pm\" failed" << std::endl;

		s_pm2pe(pm313, result);
		if (!s_is_equal(6, result, pe, error))std::cout << "\"s_pm2pe\" failed" << std::endl;

		s_vs2ve(vs, pp, result);
		if (!s_is_equal(6, result, ve, error))std::cout << "\"s_vs2ve\" failed" << std::endl;

		s_ve2vs(pp, ve, result);
		if (!s_is_equal(6, result, vs, error))std::cout << "\"s_ve2vs\" failed" << std::endl;

		s_as2ae(vs, as, pp, result);
		if (!s_is_equal(6, result, ae, error))std::cout << "\"s_as2ae\" failed" << std::endl;

		s_ae2as(pp, ve, ae, result);
		if (!s_is_equal(6, result, as, error))std::cout << "\"s_ae2as\" failed" << std::endl;
	}

	//test s_vs2vm & s_vm2vs & s_as2am & s_am2as
	{
		const double vs[6] = { 0.1,0.2,0.3, 0.2,-0.15,0.125 };
		const double as[6] = { -0.13,-0.14,0.15,-0.16,0.17,0.18 };
		const double relative_pm[16] = { 0.808307066774345, -0.072065911490471, 0.584333971461272, 0.1,
			0.341746746490328,   0.865601553329486, -0.365982393206091, 0.2,
			-0.479425538604203,   0.495520388354132,   0.724300143351802, 0.3,
			0,0,0,1 };
		const double vm[16] = { 0.0291954874793394, - 0.182528252419306, - 0.0628972223520089,   0.03,
			0.196923491067634, - 0.108112316607135, - 0.0718182822377014,   0.1525,
			0.189595409314217,   0.162310423942327,   0.0144536170779726,   0.355,
			0 ,  0,   0,   0
		};
		const double am[16] = { -0.19607150371156, - 0.0824023375945621,   0.195817097864919, - 0.1873125,
			0.0345175399147836,   0.0110332817274978,   0.210315261589722, - 0.14125,
			- 0.148327659454663, - 0.175246744763661, - 0.0645778320357833,   0.136,
			0,   0,   0,   0
		};
		
		double result[16];

		s_vs2vm(vs, relative_pm, result);
		if (!s_is_equal(16, result, vm, error))std::cout << "\"s_vs2vm\" failed" << std::endl;

		s_vm2vs(relative_pm, vm, result);
		if (!s_is_equal(6, result, vs, error))std::cout << "\"s_vm2vs\" failed" << std::endl;

		s_as2am(vs, as, relative_pm, result);
		if (!s_is_equal(16, result, am, error))std::cout << "\"s_as2am\" failed" << std::endl;

		s_am2as(relative_pm, vm, am, result);
		if (!s_is_equal(6, result, as, error))std::cout << "\"s_am2as\" failed" << std::endl;
	}

	//test s_pq2pm & s_pm2pq & s_vs2vq & s_vq2vs & s_as2aq & s_aq2as 
	{
		const double relative_pm[16] =
		{
			-0.22, -0.975499782797526,   0.000416847668728071,   0.1,
			0.175499782797526, -0.04, -0.983666521865018,   0.2,
			0.959583152331272, -0.216333478134982,   0.18,   0.3,
			0,   0,   0,   1
		};
		const double vs[6] = { 0.1,0.2,0.3, 0.2,-0.15,0.125 };
		const double as[6] = { -0.13,-0.14,0.15,-0.16,0.17,0.18 };
		const double pq[7] = { 0.1,0.2,0.3,0.4,-0.5, 0.6, std::sqrt(1 - 0.4*0.4 - 0.5*0.5 - 0.6*0.6) };
		const double vq[7] = { 0.03,   0.1525,   0.355,   0.0342083152331272, - 0.0709687364248454,   0.00997394702070449, - 0.115};
		const double aq[7] = { -0.1873125, - 0.14125,   0.136,   0.0498208478134983,   0.134530192948158,   0.0374437337098145,   0.0111331415560299 };
		
		double result[16];

		s_pq2pm(pq, result);
		if (!s_is_equal(16, relative_pm, result, error))std::cout << "\"s_pq2pm\" failed" << std::endl;

		s_pm2pq(relative_pm, result);
		if (!s_is_equal(7, pq, result, error))std::cout << "\"s_pm2pq\" failed" << std::endl;

		s_vs2vq(vs, pq, result);
		if (!s_is_equal(7, result, vq, error))std::cout << "\"s_vs2vq\" failed" << std::endl;

		s_vq2vs(pq, vq, result);
		if (!s_is_equal(6, result, vs, error))std::cout << "\"s_vq2vs\" failed" << std::endl;

		s_as2aq(vs, as, pq, result);
		if (!s_is_equal(7, result, aq, error))std::cout << "\"s_as2aq\" failed" << std::endl;

		s_aq2as(pq, vq, aq, result);
		if (!s_is_equal(6, result, as, error))std::cout << "\"s_aq2as\" failed" << std::endl;
	}

	//test coordinate transformation of each physical variables
	{
		const double relative_pm[16] ={-0.22, -0.975499782797526,   0.000416847668728071,   0.1,
			0.175499782797526, -0.04, -0.983666521865018,   0.2,
			0.959583152331272, -0.216333478134982,   0.18,   0.3,
			0,   0,   0,   1};
		const double relative_vs[16] = { 0.12, -0.35, 0.26, 0.58, 0.36, -0.135 };
		const double relative_as[16] = { 0.14, 1.35, -0.35, -0.56, -0.34, 0.14 };
		const double from_pp[3]{ 0.13, -0.22, 0.45 };
		const double to_pp[3]{ 0.286197533666383, - 0.21103496307558,   0.553339174992761 };
		const double from_vp[3]{ 0.131, -0.221, 0.451 };
		const double to_vp[3]{ 0.47766583327904, - 1.12137651835541,   0.289263700919493 };
		const double from_ap[3]{ 0.12,   0.13, -0.14 };
		const double to_ap[3]{ -0.183017842291836,   1.44829117674852, - 1.20119377113485 };
		const double from_eu_313[3]{ 4.83, 0.76, 0.45 };
		const double to_eu_321[3]{ 6.03703796978214, - 0.696087712802565,   2.29525788843731 };
		const double from_eu_321[3]{ 3.856, -0.696087712802565,   2.29525788843731 };
		const double to_eu_313[3]{ 3.42785042695091,   2.77225969913703,   4.30966052384328 };
		const double from_we[3]{ -0.918, 0.928, 0.458 };
		const double to_we[3]{ -0.123112882203827, - 0.288748067622307, - 1.13421480154937 };
		const double from_xe[3]{ -0.16,   0.17,   0.18 };
		const double to_xe[3]{ -0.690559930495208, - 0.551939939183307, - 0.0179099956559505 };
		const double from_rq[4]{ 0.4,-0.5, 0.6, std::sqrt(1 - 0.4*0.4 - 0.5*0.5 - 0.6*0.6) };
		const double to_rq[4]{ -0.383666521865017,   0.479583152331272, -0.575499782797526,   0.54 };
		const double from_wq[4]{ 0.1, 0.2, -0.4, -(from_rq[0] * 0.1 + from_rq[1] * 0.2 - from_rq[2] * 0.4) / from_rq[3] };
		const double to_wq[4]{ -0.292793710222811,   0.286847417856529, - 0.141803596258453, - 0.613907911417607 };
		const double from_xq[4]{ -0.033,   0.022, 0.011,   -(from_wq[0] * from_wq[0] + from_wq[1] * from_wq[1] + from_wq[2] * from_wq[2] + from_wq[3] * from_wq[3] + from_rq[0] * (-0.033) + from_rq[1] * (0.022) + from_rq[2] * (0.011)) / from_rq[3] };
		const double to_xq[4]{ 0.420505003689937, - 0.857290696628415,   0.550089919830468,   0.600095768217536};
		const double from_rm[9]	{0.808307066774345, -0.072065911490471, 0.584333971461272,
			0.341746746490328,   0.865601553329486, -0.365982393206091,
			-0.479425538604203,   0.495520388354132,   0.724300143351802};
		const double to_rm[9]{ -0.511401279081528, - 0.828333070215518,   0.228764194184996,
			0.599782696845049, - 0.534698430872375, - 0.595280021996288,
			0.615409983928647, - 0.16721815933072,   0.770265304210822 };
		const double from_wm[9] = { 0.0291954874793394, -0.182528252419306, -0.0628972223520089,
			0.196923491067634, -0.108112316607135, -0.0718182822377014,
			0.189595409314217,   0.162310423942327,   0.0144536170779726, };
		const double to_wm[9]{ 0.104075460683106,   0.0133045900952464,   0.280834839144192,
			- 0.477150412775713,   0.0214429907194124, - 0.500020299468546,
			0.551519952476166, - 0.134472030683956, - 0.469834287697671 };
		const double from_xm[9] = { -0.136080891292102,   0.343121545873893,   0.165312163518771,
			-0.15936039137182, -0.0939308585319294, -0.218979365699774,
			-0.18538516858617,   0.069996220473776, -0.256886513635231 };
		const double to_xm[9]{ 0.0537651382559693,   0.0517963778254724, - 0.0851387929663614,
			0.124170360512967, - 0.169749127986427,   1.08082317639183,
			- 0.958154993466446,   0.398159392361586,   0.147009590475247 };
		const double from_pe_313[6]{ 0.13, -0.22, 0.45, 4.83, 0.76, 0.45 };
		const double to_pe_321[6]{ 0.286197533666383, -0.21103496307558,   0.553339174992761,6.03703796978214, -0.696087712802565,   2.29525788843731 };
		const double from_ve[6] = { 0.089,   0.26325 ,  0.3935 ,  0.2, -0.15,   0.125 };
		const double to_ve[6] = { 0.0144960947183866, -1.09155668422567,   0.133851721734715,   0.68237707337822,   0.278141641326378,   0.111866652186502 };
		const double from_ae[6] = { -0.28023125, -0.165775,   0.1247, -0.16,   0.17,   0.18 };
		const double to_ae[6] = { 0.0898265507450043,   1.43483887623279, - 1.10554368182179, - 0.612738814129007, - 0.708943502357538, - 0.10224359010281 };
		const double from_pq[7]{ 0.13, -0.22, 0.45, 0.4,-0.5, 0.6, std::sqrt(1 - 0.4*0.4 - 0.5*0.5 - 0.6*0.6) };
		const double to_pq[7]{ 0.286197533666383, -0.21103496307558,   0.553339174992761, -0.383666521865017,   0.479583152331272, -0.575499782797526,   0.54 };
		const double from_vq[7] = { 0.03,   0.1525,   0.355,   0.0342083152331272, -0.0709687364248454,   0.00997394702070449, -0.115 };
		const double to_vq[7] = { 0.135496647027967, -1.05961001031892,   0.0942652484506193,   0.0773819018788062,   0.249992427237171,   0.247189088111772,   0.0963962136185857 };
		const double from_aq[7] = { -0.1873125, -0.14125,   0.136,   0.0498208478134983,   0.134530192948158,   0.0374437337098145,   0.0111331415560299 };
		const double to_aq[7] = { 0.025588345140825,   1.4522998248645, - 1.06971424926844,   0.11635925207608, - 0.414720507129202, - 0.230609572535648, - 0.0519601646756437 };
		const double from_pm[16]{ 0.808307066774345, -0.072065911490471, 0.584333971461272, 0.13,
			0.341746746490328,   0.865601553329486, -0.365982393206091,-0.22,
			-0.479425538604203,   0.495520388354132,   0.724300143351802,0.45,
			0,0,0,1 };
		const double to_pm[16]{ -0.511401279081528, -0.828333070215518,   0.228764194184996, 0.286197533666383,
			0.599782696845049, -0.534698430872375, -0.595280021996288, -0.21103496307558,
			0.615409983928647, -0.16721815933072,   0.770265304210822,   0.553339174992761,
			0 ,  0 ,  0,   1 };
		const double from_vm[16] = { 0.0291954874793394, -0.182528252419306, -0.0628972223520089,   0.03,
			0.196923491067634, -0.108112316607135, -0.0718182822377014,   0.1525,
			0.189595409314217,   0.162310423942327,   0.0144536170779726,   0.355,
			0 ,  0,   0,   0};
		const double to_vm[16] = { 0.104075460683106,   0.0133045900952464,   0.280834839144192,   0.135496647027967,
			- 0.477150412775713,   0.0214429907194124, - 0.500020299468546, - 1.05961001031892,
			0.551519952476166, - 0.134472030683956, - 0.469834287697671,   0.0942652484506192,
			0,   0 ,  0,   0 };
		const double from_am[16] = { -0.19607150371156, -0.0824023375945621,   0.195817097864919, -0.1873125,
			0.0345175399147836,   0.0110332817274978,   0.210315261589722, -0.14125,
			-0.148327659454663, -0.175246744763661, -0.0645778320357833,   0.136,
			0,   0,   0,   0};
		const double to_am[16] = { -0.168189089112595,   0.0240777134273968, - 0.681683032081897,   0.025588345140825,
			0.0646710887930607,   0.0754796609525905,   0.800601976050679,   1.4522998248645,
			- 1.08460897398942, - 0.248679220742461,   0.107617812109963, - 1.06971424926844,
			0,   0,   0,   0 };
		const double from_vs[6] = { -0.1873, -0.1412,   0.1365,   0.04982,   0.1345,   0.03744 };
		const double to_vs[6] = { 0.314132747625686, - 0.556683321739415,   0.160469757942957,   0.43785048599045,   0.326534924600346, - 0.109551220160011 };
		const double from_as[6] = { -0.1899, -0.1475,   0.3165,   0.9482,   0.3145,   0.7344 };
		const double to_as[6] = { 0.627235974450473,   0.750818078071529, - 0.640716977516731, - 1.07044877319847, - 0.904145907524959,   1.1457959474787 };

		double result[16], result2[16], result3[16];

		s_pp2pp(relative_pm, from_pp, result);
		if (!s_is_equal(3, to_pp, result, error))std::cout << "\"s_pp2pp\" failed" << std::endl;

		s_inv_pp2pp(relative_pm, to_pp, result);
		if (!s_is_equal(3, from_pp, result, error))std::cout << "\"s_inv_pp2pp\" failed" << std::endl;

		s_vp2vp(relative_pm, relative_vs, from_pp, from_vp, result, result2);
		if (!(s_is_equal(3, to_vp, result, error)&& s_is_equal(3, to_pp, result2, error)))std::cout << "\"s_vp2vp\" failed" << std::endl;

		s_inv_vp2vp(relative_pm, relative_vs, to_pp, to_vp, result, result2);
		if (!(s_is_equal(3, from_vp, result, error) && s_is_equal(3, from_pp, result2, error)))std::cout << "\"s_inv_vp2vp\" failed" << std::endl;

		s_ap2ap(relative_pm, relative_vs, relative_as, from_pp, from_vp, from_ap, result, result2, result3);
		if (!(s_is_equal(3, to_ap, result, error) && s_is_equal(3, to_vp, result2, error) && s_is_equal(3, to_pp, result3, error)))std::cout << "\"s_ap2ap\" failed" << std::endl;

		s_inv_ap2ap(relative_pm, relative_vs, relative_as, to_pp, to_vp, to_ap, result, result2, result3);
		if (!(s_is_equal(3, from_ap, result, error) && s_is_equal(3, from_vp, result2, error) && s_is_equal(3, from_pp, result3, error)))std::cout << "\"s_inv_ap2ap\" failed" << std::endl;

		s_re2re(relative_pm, from_eu_313, result, "313", "321");
		if (!s_is_equal(3, to_eu_321, result, error))std::cout << "\"s_re2re\" failed" << std::endl;

		s_inv_re2re(relative_pm, to_eu_321, result, "321", "313");
		if (!s_is_equal(3, from_eu_313, result, error))std::cout << "\"s_inv_re2re\" failed" << std::endl;

		s_re2re(relative_pm, from_eu_321, result, "321", "313");
		if (!s_is_equal(3, to_eu_313, result, error))std::cout << "\"s_re2re\" failed" << std::endl;

		s_inv_re2re(relative_pm, to_eu_313, result, "313", "321");
		if (!s_is_equal(3, from_eu_321, result, error))std::cout << "\"s_inv_re2re\" failed" << std::endl;
		
		s_we2we(relative_pm, relative_vs, from_we, result);
		if (!s_is_equal(3, to_we, result, error))std::cout << "\"s_we2we\" failed" << std::endl;

		s_inv_we2we(relative_pm, relative_vs, to_we, result);
		if (!s_is_equal(3, from_we, result, error))std::cout << "\"s_inv_we2we\" failed" << std::endl;

		s_xe2xe(relative_pm, relative_as, from_xe, result);
		if (!s_is_equal(3, to_xe, result, error))std::cout << "\"s_xe2xe\" failed" << std::endl;

		s_inv_xe2xe(relative_pm, relative_as, to_xe, result);
		if (!s_is_equal(3, from_xe, result, error))std::cout << "\"s_inv_xe2xe\" failed" << std::endl;

		s_wq2wq(relative_pm, relative_vs, from_rq, from_wq, result, result2);
		if (!(s_is_equal(4, to_wq, result, error) && s_is_equal(4, to_rq, result2, error)))std::cout << "\"s_wq2wq\" failed" << std::endl;

		s_inv_wq2wq(relative_pm, relative_vs, to_rq, to_wq, result, result2);
		if (!(s_is_equal(4, from_wq, result, error) && s_is_equal(4, from_rq, result2, error)))std::cout << "\"s_inv_wq2wq\" failed" << std::endl;

		s_xq2xq(relative_pm, relative_vs, relative_as, from_rq, from_wq, from_xq, result, result2, result3);
		if (!(s_is_equal(4, to_xq, result, error) && s_is_equal(4, to_wq, result2, error) && s_is_equal(4, to_rq, result3, error)))std::cout << "\"s_xq2xq\" failed" << std::endl;

		s_inv_xq2xq(relative_pm, relative_vs, relative_as, to_rq, to_wq, to_xq, result, result2, result3);
		if (!(s_is_equal(4, from_xq, result, error) && s_is_equal(4, from_wq, result2, error) && s_is_equal(4, from_rq, result3, error)))std::cout << "\"s_inv_xq2xq\" failed" << std::endl;

		s_rq2rq(relative_pm, from_rq, result);
		if (!s_is_equal(4, to_rq, result, error))std::cout << "\"s_rq2rq\" failed" << std::endl;

		s_inv_rq2rq(relative_pm, to_rq, result);
		if (!s_is_equal(4, from_rq, result, error))std::cout << "\"s_inv_rq2rq\" failed" << std::endl;

		s_rm2rm(relative_pm, from_rm, result);
		if (!s_is_equal(3, to_rm, result, error))std::cout << "\"s_rm2rm\" failed" << std::endl;

		s_inv_rm2rm(relative_pm, to_rm, result);
		if (!s_is_equal(3, from_rm, result, error))std::cout << "\"s_inv_rm2rm\" failed" << std::endl;

		s_wm2wm(relative_pm, relative_vs, from_rm, from_wm, result, result2);
		if (!(s_is_equal(9, to_wm, result, error) && s_is_equal(9, to_rm, result2, error)))std::cout << "\"s_wm2wm\" failed" << std::endl;

		s_inv_wm2wm(relative_pm, relative_vs, to_rm, to_wm, result, result2);
		if (!(s_is_equal(9, from_wm, result, error) && s_is_equal(9, from_rm, result2, error)))std::cout << "\"s_inv_wm2wm\" failed" << std::endl;

		s_xm2xm(relative_pm, relative_vs, relative_as, from_rm, from_wm, from_xm, result, result2, result3);
		if (!(s_is_equal(9, to_xm, result, error) && s_is_equal(9, to_wm, result2, error) && s_is_equal(9, to_rm, result3, error)))std::cout << "\"s_xm2xm\" failed" << std::endl;

		s_inv_xm2xm(relative_pm, relative_vs, relative_as, to_rm, to_wm, to_xm, result, result2, result3);
		if (!(s_is_equal(9, from_xm, result, error) && s_is_equal(9, from_wm, result2, error) && s_is_equal(9, from_rm, result3, error)))std::cout << "\"s_inv_xm2xm\" failed" << std::endl;

		s_pe2pe(relative_pm, from_pe_313, result, "313", "321");
		if (!s_is_equal(6, to_pe_321, result, error))std::cout << "\"s_pe2pe\" failed" << std::endl;

		s_inv_pe2pe(relative_pm, to_pe_321, result, "321", "313");
		if (!s_is_equal(6, from_pe_313, result, error))std::cout << "\"s_inv_pe2pe\" failed" << std::endl;

		s_ve2ve(relative_pm, relative_vs, from_pp, from_ve, result, result2);
		if (!(s_is_equal(6, to_ve, result, error) && s_is_equal(3, to_pp, result2, error)))std::cout << "\"s_ve2ve\" failed" << std::endl;

		s_inv_ve2ve(relative_pm, relative_vs, to_pp, to_ve, result, result2);
		if (!(s_is_equal(6, from_ve, result, error) && s_is_equal(3, from_pp, result2, error)))std::cout << "\"s_inv_ve2ve\" failed" << std::endl;

		s_ae2ae(relative_pm, relative_vs, relative_as, from_pp, from_ve, from_ae, result, result2, result3);
		if (!(s_is_equal(6, to_ae, result, error) && s_is_equal(6, to_ve, result2, error) && s_is_equal(3, to_pp, result3, error)))std::cout << "\"s_ae2ae\" failed" << std::endl;

		s_inv_ae2ae(relative_pm, relative_vs, relative_as, to_pp, to_ve, to_ae, result, result2, result3);
		if (!(s_is_equal(6, from_ae, result, error) && s_is_equal(6, from_ve, result2, error) && s_is_equal(3, from_pp, result3, error)))std::cout << "\"s_inv_ae2ae\" failed" << std::endl;

		s_pq2pq(relative_pm, from_pq, result);
		if (!s_is_equal(7, to_pq, result, error))std::cout << "\"s_pq2pq\" failed" << std::endl;

		s_inv_pq2pq(relative_pm, to_pq, result);
		if (!s_is_equal(7, from_pq, result, error))std::cout << "\"s_inv_pq2pq\" failed" << std::endl;

		s_vq2vq(relative_pm, relative_vs, from_pq, from_vq, result, result2);
		if (!(s_is_equal(7, to_vq, result, error) && s_is_equal(7, to_pq, result2, error)))std::cout << "\"s_vq2vq\" failed" << std::endl;

		s_inv_vq2vq(relative_pm, relative_vs, to_pq, to_vq, result, result2);
		if (!(s_is_equal(7, from_vq, result, error) && s_is_equal(7, from_pq, result2, error)))std::cout << "\"s_inv_vq2vq\" failed" << std::endl;

		s_aq2aq(relative_pm, relative_vs, relative_as, from_pq, from_vq, from_aq, result, result2, result3);
		if (!(s_is_equal(7, to_aq, result, error) && s_is_equal(7, to_vq, result2, error) && s_is_equal(7, to_pq, result3, error)))std::cout << "\"s_aq2aq\" failed" << std::endl;

		s_inv_aq2aq(relative_pm, relative_vs, relative_as, to_pq, to_vq, to_aq, result, result2, result3);
		if (!(s_is_equal(7, from_aq, result, error) && s_is_equal(7, from_vq, result2, error) && s_is_equal(7, from_pq, result3, error)))std::cout << "\"s_inv_aq2aq\" failed" << std::endl;

		s_pm2pm(relative_pm, from_pm, result);
		if (!s_is_equal(16, to_pm, result, error))std::cout << "\"s_pm2pm\" failed" << std::endl;

		s_inv_pm2pm(relative_pm, to_pm, result);
		if (!s_is_equal(16, from_pm, result, error))std::cout << "\"s_inv_pm2pm\" failed" << std::endl;

		s_vm2vm(relative_pm, relative_vs, from_pm, from_vm, result, result2);
		if (!(s_is_equal(16, to_vm, result, error) && s_is_equal(16, to_pm, result2, error)))std::cout << "\"s_vm2vm\" failed" << std::endl;

		s_inv_vm2vm(relative_pm, relative_vs, to_pm, to_vm, result, result2);
		if (!(s_is_equal(16, from_vm, result, error) && s_is_equal(16, from_pm, result2, error)))std::cout << "\"s_inv_vm2vm\" failed" << std::endl;

		s_am2am(relative_pm, relative_vs, relative_as, from_pm, from_vm, from_am, result, result2, result3);
		if (!(s_is_equal(16, to_am, result, error) && s_is_equal(16, to_vm, result2, error) && s_is_equal(16, to_pm, result3, error)))std::cout << "\"s_am2am\" failed" << std::endl;

		s_inv_am2am(relative_pm, relative_vs, relative_as, to_pm, to_vm, to_am, result, result2, result3);
		if (!(s_is_equal(16, from_am, result, error) && s_is_equal(16, from_vm, result2, error) && s_is_equal(16, from_pm, result3, error)))std::cout << "\"s_inv_am2am\" failed" << std::endl;

		s_vs2vs(relative_pm, relative_vs, from_vs, result);
		if (!s_is_equal(6, to_vs, result, error))std::cout << "\"s_vs2vs\" failed" << std::endl;

		s_inv_vs2vs(relative_pm, relative_vs, to_vs, result);
		if (!s_is_equal(6, from_vs, result, error))std::cout << "\"s_inv_vs2vs\" failed" << std::endl;

		s_as2as(relative_pm, relative_vs, relative_as, from_vs, from_as, result, result2);
		if (!(s_is_equal(6, to_as, result, error)&& s_is_equal(6, to_vs, result2, error)))std::cout << "\"s_as2as\" failed" << std::endl;

		s_inv_as2as(relative_pm, relative_vs, relative_as, to_vs, to_as, result, result2);
		if (!(s_is_equal(6, from_as, result, error)&& s_is_equal(6, from_vs, result2, error)))std::cout << "\"s_inv_as2as\" failed" << std::endl;
	}


	//dlmwrite("C:\\Users\\yang\\Desktop\\test.txt", result, 1, 3);

	char a;
	std::cin >> a;

	return 0;
}