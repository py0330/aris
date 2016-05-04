#include "test_dynamic_model.h"
#include <iostream>
#include <aris.h>

using namespace aris::dynamic;

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

	const double to_pm[4][4] = { -0.588213973891872,   0.630556248624336,   0.506362654862241,   0.135,
		- 0.586243982337353,   0.0988346033872263, - 0.804083151388295,   0.246,
		- 0.55706580768485, - 0.769825005110963,   0.311523911464805, - 0.398,
		0,   0,   0,   1 };
	const double to_vs[6] = { 0.937854,   -1.0818, -2.64023, -2.0704, -0.9041,   0.3025 };
	const double to_as[6] = { -1.937854,   -1.3118, 0.44023, 0.1704, -0.3041,   1.6025 };

	const double from_pp[3]{ -0.669416050318542,   0.115018275340304, - 0.170874070337385 };
	const double from_re_313[3]{ 4.83, 0.76, 0.45 };
	const double from_re_321[3]{ 3.856, -0.696087712802565,   2.29525788843731 };
	const double from_rq[4]{ 0.4, -0.5, 0.6, std::sqrt(1 - 0.4*0.4 - 0.5*0.5 - 0.6*0.6) };
	const double from_rm[9]{ 0.808307066774345, -0.072065911490471, 0.584333971461272,
		0.341746746490328,   0.865601553329486, -0.365982393206091,
		-0.479425538604203,   0.495520388354132,   0.724300143351802 };
	const double from_pe_313[6]{ -0.669416050318542,   0.115018275340304, - 0.170874070337385,   0.0874897828198543,   0.560037752180822,   2.03015071219623 };
	const double from_pe_321[6]{ -0.669416050318542,   0.115018275340304, - 0.170874070337385,   2.18673429565011, - 0.496273081682159,   6.01203808167169 };
	const double from_pq[7]{ -0.669416050318542,   0.115018275340304, - 0.170874070337385,   0.155930520362764, - 0.228184452435451,   0.837822562491012,   0.470819373292057 };
	const double from_pm[16]{ -0.508029581104545, - 0.860088028417832,   0.0464168945004766, - 0.669416050318542,
		0.717764346790022, - 0.452521946799217, - 0.529186385068758,   0.115018275340304,
		0.476151538063078, - 0.235525945571554,   0.847235056972262, - 0.170874070337385,
		0,   0,   0,   1 };

	const double from_vp[3]{ 0.131, -0.221, 0.451 };
	const double from_we_313[3]{ 1.03, 0.73, 0.25 };
	const double from_we_321[3]{ 2.15, 0.76, 1.25 };
	const double from_wq[4]{ 0.1, 0.2, -0.4, -(from_rq[0] * 0.1 + from_rq[1] * 0.2 - from_rq[2] * 0.4) / from_rq[3] };
	const double from_wm[9] = { 0.0291954874793394, -0.182528252419306, -0.0628972223520089,
		0.196923491067634, -0.108112316607135, -0.0718182822377014,
		0.189595409314217,   0.162310423942327,   0.0144536170779726, };
	const double from_wa[3]{ -0.918, 0.928, 0.458 };
	const double from_ve_313[6]{ -3.81503012399803, - 0.382961959168423,   1.09120133335834,   5.31090041234504,   0.564629336172518, - 4.70920359383487 };
	const double from_ve_321[6]{ -3.81503012399803, - 0.382961959168423,   1.09120133335834,   0.442220922706803, - 2.10574187768738,   1.84579287110303 };
	const double from_vq[7] = { -3.81503012399803, - 0.382961959168423,   1.09120133335834,   1.39920914217107,   0.374072545901733,   0.0237472511951562, - 0.324366120191619 };
	const double from_vm[16] = { 0.261846224837118, - 0.000735714268009113,   2.85225563468484, - 3.81503012399803,
		- 1.0430580665844, - 0.952301569934064, - 0.600417056115713, - 0.382961959168423,
		1.85171200607782,   1.83236771793217, - 0.531287482309403,   1.09120133335834,
		0,   0,   0,   0 };
	const double from_va[6] = { -3.81503012399803, - 0.382961959168423,   1.09120133335834,   0.781056353710579,   2.54138272764251,   1.32109803722837 };
	const double from_vs[6] = { -3.22882329524316,   0.367939992675169, - 0.699876809275252,   0.781056353710579,   2.54138272764251,   1.32109803722837 };

	const double from_ap[3]{ 0.12,   0.13, -0.14 };
	const double from_xe_313[3]{ 4.83, 0.76, 0.45 };
	const double from_xe_321[3]{ 3.856, -0.696087712802565,   2.29525788843731 };
	const double from_xq[4]{ -0.033,   0.022, 0.011,   -(from_wq[0] * from_wq[0] + from_wq[1] * from_wq[1] + from_wq[2] * from_wq[2] + from_wq[3] * from_wq[3] + from_rq[0] * (-0.033) + from_rq[1] * 0.022 + from_rq[2] * 0.011) / from_rq[3] };
	const double from_xm[9] = { -0.19607150371156, -0.0824023375945621,   0.195817097864919,
		0.0345175399147836,   0.0110332817274978,   0.210315261589722,
		-0.148327659454663, -0.175246744763661, -0.0645778320357833 };
	const double from_xa[3]{ -0.16,   0.17,   0.18 };
	const double from_aa[6] = { -0.28023125, -0.165775,   0.1247, -0.16,   0.17,   0.18 };
	const double from_ae_313[6]{ 0.12,   0.13, -0.14,4.83, 0.76, 0.45 };
	const double from_ae_321[6]{ 0.12,   0.13, -0.14,3.856, -0.696087712802565,   2.29525788843731 };
	const double from_aq[7] = { -0.1873125, -0.14125,   0.136,   from_xq[0],   from_xq[1],   from_xq[2],   from_xq[3] };
	const double from_am[16] = { -0.19607150371156, -0.0824023375945621,   0.195817097864919, -0.1873125,
		0.0345175399147836,   0.0110332817274978,   0.210315261589722, -0.14125,
		-0.148327659454663, -0.175246744763661, -0.0645778320357833,   0.136,
		0,   0,   0,   0 };
	const double from_as[6] = { -0.1899, -0.1475,   0.3165,   0.9482,   0.3145,   0.7344 };
	



	const double error = 1e-10;

	r.setPm(relative_pm);
	r.setVs(relative_vs);
	r.setAs(relative_as);

	double pe_test[6]{ 0.135,0.246,-0.398,0.562,1.254,3.768 };
	double pm_test[16];
	s_pe2pm(pe_test, pm_test);

	double result[16];

	p.setPm(*to_pm);
	p.setVs(to_vs);
	p.setAs(to_as);
	p.getVs(r, result);
	dlmwrite("C:\\Users\\yang\\Desktop\\test.txt", result, 1, 6);


	p.setPp(pp);
	if (!(s_is_equal(3, &pm[3], &p.pm()[0][3], error, 4, 4)))std::cout << "\"part:setPp\" failed" << std::endl;

	p.setPp(r, from_pp);
	if (!(s_is_equal(3, &to_pm[0][3], &p.pm()[0][3], error, 4, 4)))std::cout << "\"part:setPp relative\" failed" << std::endl;

	p.setPe(pe313);
	if (!(s_is_equal(16, pm, &p.pm()[0][0], error)))std::cout << "\"part:setPe\" failed" << std::endl;

	p.setPe(r, from_pe_313);
	if (!(s_is_equal(16, &to_pm[0][0], &p.pm()[0][0], error)))std::cout << "\"part:setPe relative\" failed" << std::endl;

	p.setPe(pe321, "321");
	if (!(s_is_equal(16, pm, &p.pm()[0][0], error)))std::cout << "\"part:setPe\" failed" << std::endl;

	p.setPe(r, from_pe_321, "321");
	if (!(s_is_equal(16, &to_pm[0][0], &p.pm()[0][0], error)))std::cout << "\"part:setPe 321 relative\" failed" << std::endl;

	p.setPq(pq);
	if (!(s_is_equal(16, pm, &p.pm()[0][0], error)))std::cout << "\"part:setPq\" failed" << std::endl;

	p.setPq(r, from_pq);
	if (!(s_is_equal(16, &to_pm[0][0], &p.pm()[0][0], error)))std::cout << "\"part:setPq relative\" failed" << std::endl;

	p.setPm(pm);
	if (!(s_is_equal(16, pm, &p.pm()[0][0], error)))std::cout << "\"part:setPm\" failed" << std::endl;

	p.setPm(r, from_pm);
	if (!(s_is_equal(16, &to_pm[0][0], &p.pm()[0][0], error)))std::cout << "\"part:setPm relative\" failed" << std::endl;


	p.setPe(pe313);
	p.setVe(ve313);
	if (!(s_is_equal(6, vs, p.vs(), error)))std::cout << "\"part:setVe\" failed" << std::endl;

	p.setPe(r, from_pe_313);
	p.setVe(r, from_ve_313);
	if (!(s_is_equal(6, to_vs, p.vs(), error)))std::cout << "\"part:setVe relative\" failed" << std::endl;

	p.setPe(pe321, "321");
	p.setVe(ve321, "321");
	if (!(s_is_equal(6, vs, p.vs(), error)))std::cout << "\"part:setVe 321\" failed" << std::endl;

	p.setPe(r, from_pe_321, "321");
	p.setVe(r, from_ve_321, "321");
	if (!(s_is_equal(6, to_vs, p.vs(), error)))std::cout << "\"part:setVe 321 relative\" failed" << std::endl;

	p.setPq(pq);
	p.setVq(vq);
	if (!(s_is_equal(6, vs, p.vs(), error)))std::cout << "\"part:setVq\" failed" << std::endl;

	p.setPq(r, from_pq);
	p.setVq(r, from_vq);
	if (!(s_is_equal(6, to_vs, p.vs(), error)))std::cout << "\"part:setVq relative\" failed" << std::endl;

	p.setPm(pm);
	p.setVm(vm);
	if (!(s_is_equal(6, vs, p.vs(), error)))std::cout << "\"part:setVm\" failed" << std::endl;

	p.setPm(r, from_pm);
	p.setVm(r, from_vm);
	if (!(s_is_equal(6, to_vs, p.vs(), error)))std::cout << "\"part:setVm relative\" failed" << std::endl;

	p.setPp(pp);
	p.setVa(va);
	if (!(s_is_equal(6, vs, p.vs(), error)))std::cout << "\"part:setVa\" failed" << std::endl;

	p.setPp(r, from_pp);
	p.setVa(r, from_va);
	if (!(s_is_equal(6, to_vs, p.vs(), error)))std::cout << "\"part:setVa relative\" failed" << std::endl;

	p.setPm(pm);
	p.setVs(vs);
	if (!(s_is_equal(6, vs, p.vs(), error)))std::cout << "\"part:setVs\" failed" << std::endl;

	p.setPm(r, from_pm);
	p.setVs(r, from_vs);
	if (!(s_is_equal(6, to_vs, p.vs(), error)))std::cout << "\"part:setVs relative\" failed" << std::endl;

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
	test_part();


	std::cout << "test model finished" << std::endl;
}

