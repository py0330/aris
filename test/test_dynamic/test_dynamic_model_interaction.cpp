#include "test_dynamic_model_interaction.h"
#include <iostream>
#include <aris_dynamic.h>

#include<type_traits>

using namespace aris::dynamic;

void test_constraint()
{
	const double error = 1e-10;
	
	const double prt_pm_i[16]{ -0.22, -0.975499782797526,   0.000416847668728071, 0.1,
		0.175499782797526, -0.04, -0.983666521865018, 0.2,
		0.959583152331272, -0.216333478134982,   0.18,0.3,
		0,0,0,1 };
	const double prt_pm_j[16]{ -0.1224,   0.253539765421328,   0.959549804517774, -0.116974902258887,
		-0.989539765421329,   0.0432, -0.137640156385781, -0.0855499782797527,
		-0.0763498045177736, -0.966359843614219,   0.2456,   0.406691619606131,
		0,   0,   0,   1 };
	const double glb_pe_n[16]{ 0.58,0.64,0.35,1.25,0.31,0.22 };
	const double glb_vs_n[6]{ 0.15,0.3,0.289,0.12,4.35,2.31 };
	
	aris::dynamic::Model model;
	auto &prt_m = model.partPool().add<Part>("prt_m");
	auto &prt_n = model.partPool().add<Part>("prt_n");
	auto &mak_i = prt_m.markerPool().add<Marker>("mak_i", prt_pm_i);
	auto &mak_j = prt_n.markerPool().add<Marker>("mak_j", prt_pm_j);

	prt_n.setPe(glb_pe_n);
	prt_n.setVs(glb_vs_n);

	// test revolute joints //
	{
		const double relative_pe[6]{0,0,0,0.31,0,0};
		const double relative_vs[6]{ 0,0,0,0,0,0.515 };

		double relative_pm[16];
		s_pe2pm(relative_pe, relative_pm, "321");

		double mak_i_pm[16];
		s_pm_dot_pm(*mak_j.pm(), relative_pm, mak_i_pm);

		double glb_pm_m[16];
		s_pm_dot_inv_pm(mak_i_pm, *mak_i.prtPm(), glb_pm_m);

		prt_m.setPm(glb_pm_m);
		prt_m.setVs(mak_j, relative_vs);

		auto &jnt = model.add<RevoluteJoint>("r1", &mak_i, &mak_j);

		const double glb_cmI[30]{ 0.772732834750084, -0.55499915414649,0.307993352194134,0,0,
			-0.0834135974649058,   0.392232918710641,   0.916076148165476, 0, 0,
			-0.629226618840194, -0.733572952320633,   0.256796779120235,   0,   0,
			-0.241746251737219, -0.626453518771242, -0.522335646172068,   0.772732834750084, -0.55499915414649,
			1.02612412552421,   0.171279303870418,   0.0200980273534624, -0.0834135974649058,   0.392232918710641,
			-0.432909166577701,   0.565536873993989,   0.554775584177545, -0.629226618840194, -0.733572952320633 };
		const double glb_cmJ[30]{ -0.772732834750084, 0.55499915414649,-0.307993352194134,0,0,
			0.0834135974649058,   -0.392232918710641,   -0.916076148165476, 0, 0,
			0.629226618840194, 0.733572952320633,   -0.256796779120235,   0,   0,
			0.241746251737219, 0.626453518771242, 0.522335646172068,   -0.772732834750084, 0.55499915414649,
			-1.02612412552421,   -0.171279303870418,   -0.0200980273534624, 0.0834135974649058,   -0.392232918710641,
			0.432909166577701,   -0.565536873993989,   -0.554775584177545, 0.629226618840194, 0.733572952320633 };
		const double prt_cmI[30]{ -0.22, - 0.975499782797526,0.000416847668728071,0,0,
			0.175499782797526, - 0.04, - 0.983666521865018,   0,   0,
			0.959583152331272, - 0.216333478134982,   0.18,   0,   0,
			0.139266695626997, - 0.0312666956269964,   0.331099956559505, - 0.22, - 0.975499782797526,
			- 0.161958315233127, - 0.27101658702576, - 0.0178749456993816,0.175499782797526, - 0.04,
			0.0615499782797526,   0.191099956559505, - 0.0984500217202474,0.959583152331272, - 0.216333478134982, };
		const double prt_cmJ[30]{ 0.0392211338303902, - 0.278793607012357, - 0.959549804517775, - 0, - 0,
			0.929193404253209, - 0.343008461765058,   0.137640156385781,   0,   0,
			0.367506898103141,   0.897005752404413, - 0.2456, - 0, - 0,
			- 0.409335377653455,   0.0627598442188279, - 0.0349660234578675,   0.0392211338303902, - 0.278793607012357,
			0.0589399899253518, - 0.008455863358525, - 0.418969900086863,   0.929193404253209, - 0.343008461765058,
			- 0.105336940494824,   0.0162725942644976, - 0.0981899087749608,   0.367506898103141,   0.897005752404413 };
		const double cp[6]{ 0,0,0,0,0,0 };
		const double ca[6]{ 0.926329546106927,-1.72197953069011,0,	-0.0282942143129791,	0.887671869636436 };

		double result1[42], result2[48];

		jnt.cptGlbCm(result1, 5, result2, 7);
		if (!s_is_equal(6, jnt.dim(), result1, 5, glb_cmI, jnt.dim(), error) || !s_is_equal(6, jnt.dim(), result2, 7, glb_cmJ, jnt.dim(), error))std::cout << "\"RevoluteJoint:cptGlbCm\" failed" << std::endl;

		jnt.cptPrtCm(result1, 6, result2, 7);
		if (!s_is_equal(6, jnt.dim(), result1, 6, prt_cmI, jnt.dim(), error) || !s_is_equal(6, jnt.dim(), result2, 7, prt_cmJ, jnt.dim(), error))std::cout << "\"RevoluteJoint:cptPrtCm\" failed" << std::endl;

		jnt.cptCa(result1);
		if (!s_is_equal(jnt.dim(), result1, ca, error))std::cout << "\"RevoluteJoint:cptCa\" failed" << std::endl;

		jnt.cptCp(result1);
		if (!s_is_equal(jnt.dim(), result1, cp, error))std::cout << "\"RevoluteJoint:cptCp\" failed" << std::endl;
	}

	// test prismatic joints //
	{
		const double relative_pe[6]{ 0,0,0.326,0,0,0 };
		const double relative_vs[6]{ 0,0,0.518,0,0,0 };

		double relative_pm[16];
		s_pe2pm(relative_pe, relative_pm, "321");

		double mak_i_pm[16];
		s_pm_dot_pm(*mak_j.pm(), relative_pm, mak_i_pm);

		double glb_pm_m[16];
		s_pm_dot_inv_pm(mak_i_pm, *mak_i.prtPm(), glb_pm_m);

		prt_m.setPm(glb_pm_m);
		prt_m.setVs(mak_j, relative_vs);

		auto &jnt = model.add<PrismaticJoint>("p1", &mak_i, &mak_j);

		const double glb_cmI[30]{ 0.905206704276648, - 0.292815500847941,0,0,0,
			- 0.199091608400864,0.348090537398911, - 0, - 0, - 0,
			- 0.375450867620476, - 0.890557162812419,   0,   0,   0,
			- 0.134575867968274, - 0.96543688341334,   0.905206704276648, - 0.292815500847941,   0.307993352194134,
			1.03843973590868,   0.541046921795271, - 0.199091608400864,   0.348090537398911,   0.916076148165476,
			- 0.875117474759134,   0.528914052896916, - 0.375450867620476, - 0.890557162812419,   0.256796779120235};
		const double glb_cmJ[30]{ -0.905206704276648, 0.292815500847941,0,0,0,
			0.199091608400864,-0.348090537398911, -0, -0, -0,
			0.375450867620476, 0.890557162812419,   0,   0,   0,
			0.134575867968274, 0.96543688341334,   -0.905206704276648, 0.292815500847941,   -0.307993352194134,
			-1.03843973590868,   -0.541046921795271, 0.199091608400864,   -0.348090537398911,   -0.916076148165476,
			0.875117474759134,   -0.528914052896916, 0.375450867620476, 0.890557162812419,   -0.256796779120235, };
		const double prt_cmI[30]{ -0.22, - 0.975499782797526 ,  0,   0,   0,
			0.175499782797526, - 0.04,   0,   0,   0,
			0.959583152331272, - 0.216333478134982,   0,   0,   0,
			0.139266695626997, - 0.0312666956269964, - 0.22, - 0.975499782797526,   0.000416847668728071,
			- 0.161958315233127, - 0.27101658702576,   0.175499782797526, - 0.04, - 0.983666521865018,
			0.0615499782797526,   0.191099956559505,   0.959583152331272, - 0.216333478134982,   0.18 };
		const double prt_cmJ[30]{ 0.1224, - 0.253539765421328, - 0, - 0, - 0,
			0.989539765421329, - 0.0431999999999995, - 0, - 0, - 0,
			0.0763498045177739,   0.966359843614218, - 0, - 0, - 0,
			- 0.491623217509383, - 0.105005385664637,   0.1224, - 0.253539765421328, - 0.959549804517775,
			0.0446268651607419, - 0.312662613107425,   0.989539765421329, - 0.0431999999999995,   0.137640156385781,
			0.209753309018236, - 0.0415270419200584,   0.0763498045177739,   0.966359843614218, - 0.2456 };
		const double cp[6]{ 0,0,0,0,0,0 };
		const double ca[6]{ -0.299471893489825,   0.841602471649138, - 0, - 0, - 0 };


		double result1[42], result2[48];


		jnt.cptGlbCm(result1, 5, result2, 7);
		if (!s_is_equal(6, jnt.dim(), result1, 5, glb_cmI, jnt.dim(), error) || !s_is_equal(6, jnt.dim(), result2, 7, glb_cmJ, jnt.dim(), error))std::cout << "\"PrismaticJoint:cptGlbCm\" failed" << std::endl;

		jnt.cptPrtCm(result1, 6, result2, 7);
		if (!s_is_equal(6, jnt.dim(), result1, 6, prt_cmI, jnt.dim(), error) || !s_is_equal(6, jnt.dim(), result2, 7, prt_cmJ, jnt.dim(), error))std::cout << "\"PrismaticJoint:cptPrtCm\" failed" << std::endl;

		jnt.cptCa(result1);
		if (!s_is_equal(jnt.dim(), result1, ca, error))std::cout << "\"PrismaticJoint:cptCa\" failed" << std::endl;

		jnt.cptCp(result1);
		if (!s_is_equal(jnt.dim(), result1, cp, error))std::cout << "\"PrismaticJoint:cptCp\" failed" << std::endl;
	}

	// test spherical joints //
	{
		const double relative_pe[6]{ 0,0,0,0.221,0.654,0.123 };
		const double relative_vs[6]{ 0,0,0,0.514,0.356,0.777 };

		double relative_pm[16];
		s_pe2pm(relative_pe, relative_pm, "321");

		double mak_i_pm[16];
		s_pm_dot_pm(*mak_j.pm(), relative_pm, mak_i_pm);

		double glb_pm_m[16];
		s_pm_dot_inv_pm(mak_i_pm, *mak_i.prtPm(), glb_pm_m);

		prt_m.setPm(glb_pm_m);
		prt_m.setVs(mak_j, relative_vs);

		auto &jnt = model.add<SphericalJoint>("s1", &mak_i, &mak_j);

		const double glb_cmI[]{ 0.462635502589964, - 0.389341212341349,   0.796480892498936,
			- 0.650918208225164,   0.460769198258918,   0.603321831311263,
			- 0.601891915500013, - 0.797562014083169, - 0.0402610947109546,
			0.170858464032355, - 0.705263081708994, - 0.443994548974076,
			0.786857634821923,   0.336935956217407,   0.591608380223745,
			- 0.719622176761417,   0.538939525796233,   0.0818923769414457 };
		const double glb_cmJ[]{ -0.462635502589964, 0.389341212341349,   -0.796480892498936,
			0.650918208225164,   -0.460769198258918,   -0.603321831311263,
			0.601891915500013, 0.797562014083169, 0.0402610947109546,
			-0.170858464032355, 0.705263081708994, 0.443994548974076,
			-0.786857634821923,   -0.336935956217407,   -0.591608380223745,
			0.719622176761417,   -0.538939525796233,   -0.0818923769414457 };
		const double prt_cmI[]{ -0.22, - 0.975499782797526,   0.000416847668728071,
			0.175499782797526, - 0.04, - 0.983666521865018,
			0.959583152331272, - 0.216333478134982,   0.18,
			0.139266695626997, - 0.0312666956269964,   0.331099956559505,
			- 0.161958315233127, - 0.27101658702576, - 0.0178749456993816,
			0.0615499782797526,   0.191099956559505, - 0.0984500217202474};
		const double prt_cmJ[]{ 0.634429008738031, - 0.360802037735136, - 0.683609334662608,
			0.675002783167594, - 0.172345515887459,   0.717403837367413,
			0.376657769849079,   0.916580008902473, - 0.134201384838831,
			- 0.306741039154564, - 0.00832192286513729, - 0.280281202972532,
			0.302076466901929, - 0.039518308129878, - 0.293716381366335,
			- 0.0246829966679223, - 0.0107065066158384, - 0.142401007488425 };
		const double cp[6]{ 0,0,0,0,0,0 };
		const double ca[]{ 1.25497709355873, - 1.42034889864347,   0.552653481810815 };

		double result1[42], result2[48];

		jnt.cptGlbCm(result1, 5, result2, 7);
		if (!s_is_equal(6, jnt.dim(), result1, 5, glb_cmI, jnt.dim(), error) || !s_is_equal(6, jnt.dim(), result2, 7, glb_cmJ, jnt.dim(), error))std::cout << "\"SphericalJoint:cptGlbCm\" failed" << std::endl;

		jnt.cptPrtCm(result1, 6, result2, 7);
		if (!s_is_equal(6, jnt.dim(), result1, 6, prt_cmI, jnt.dim(), error) || !s_is_equal(6, jnt.dim(), result2, 7, prt_cmJ, jnt.dim(), error))std::cout << "\"SphericalJoint:cptPrtCm\" failed" << std::endl;

		jnt.cptCa(result1);
		if (!s_is_equal(jnt.dim(), result1, ca, error))std::cout << "\"SphericalJoint:cptCa\" failed" << std::endl;

		jnt.cptCp(result1);
		if (!s_is_equal(jnt.dim(), result1, cp, error))std::cout << "\"SphericalJoint:cptCp\" failed" << std::endl;
	}
	
	// test universal joints //
	{
		const double relative_pe[6]{ 0,0,0,0.856,aris::PI / 2,0.972 };
		const double relative_ve[6]{ 0,0,0,0.157,0,0.895 };

		double relative_pm[16];
		s_pe2pm(relative_pe, relative_pm, "313");

		double mak_i_pm[16];
		s_pm_dot_pm(*mak_j.pm(), relative_pm, mak_i_pm);

		double glb_pm_m[16];
		s_pm_dot_inv_pm(mak_i_pm, *mak_i.prtPm(), glb_pm_m);

		prt_m.setPm(glb_pm_m);
		prt_m.setVe(mak_j, relative_ve, nullptr, "313");

		auto &jnt = model.add<UniversalJoint>("u1", &mak_i, &mak_j);

		const double glb_cmI[]{ 0.464190255001298, - 0.133832675696459,   0.875566229406867,   0,
			0.831313762500364,   0.40698899884696, - 0.378519990350626,   0,
			- 0.305687480017286,   0.903575547330423,   0.300177272336208,   0,
			- 0.731260909855271,   0.144942403197629,   0.409840176342917, - 0.372187953763235,
			0.561015416631454, - 0.786498730845814,   0.386461770813631, - 0.132389983258812,
			0.415246475240747,   0.375723271514129, - 0.708110612260744,   0.918666979599392 };
		const double glb_cmJ[]{ -0.464190255001298,   0.133832675696459, - 0.875566229406867, - 0,
			- 0.831313762500364, - 0.40698899884696,   0.378519990350626, - 0,
			0.305687480017286, - 0.903575547330423, - 0.300177272336208, - 0,
			0.731260909855271, - 0.144942403197629, - 0.409840176342917,   0.372187953763235,
			- 0.561015416631454,   0.786498730845814, - 0.386461770813631,   0.132389983258812,
			- 0.415246475240747, - 0.375723271514129,   0.708110612260744, - 0.918666979599392 };
		const double prt_cmI[]{ -0.22, - 0.975499782797526,   0.000416847668728071,   0,
			0.175499782797526, - 0.04, - 0.983666521865018,   0,
			0.959583152331272, - 0.216333478134982,   0.18,   0,
			0.139266695626997, - 0.0312666956269964,   0.331099956559505, - 0.681774424596104,
			- 0.161958315233127, - 0.27101658702576, - 0.0178749456993816, - 0.131960798655384,
			0.0615499782797526,   0.191099956559505, - 0.0984500217202474, - 0.719562354202113 };
		const double prt_cmJ[]{ -0.855308832662975, - 0.448953753689671,   0.258625845221729, - 0,
			0.460889922612995, - 0.431229074578346,   0.775642936196864, - 0,
			0.23670082383145, - 0.782612300111783, - 0.575752297182787,   0,
			- 0.207689819425203,   0.242329716037551, - 0.266191885439452,   0.111251268906238,
			- 0.320158878686852, - 0.274131726528483,   0.0378323951769434, - 0.615981511798859,
			- 0.127084205706559,   0.0120349949732274, - 0.0686053212081227, - 0.779865329585016 };
		const double cp[6]{ 0,0,0,0,0,0 };
		const double ca[]{ 0.297342839758157,3.33820400487961,3.34479764841342,5.30996702214242 };


		double result1[42], result2[48];


		jnt.cptGlbCm(result1, 5, result2, 7);
		if (!s_is_equal(6, jnt.dim(), result1, 5, glb_cmI, jnt.dim(), error) || !s_is_equal(6, jnt.dim(), result2, 7, glb_cmJ, jnt.dim(), error))std::cout << "\"UniversalJoint:cptGlbCm\" failed" << std::endl;

		jnt.cptPrtCm(result1, 6, result2, 7);
		if (!s_is_equal(6, jnt.dim(), result1, 6, prt_cmI, jnt.dim(), error) || !s_is_equal(6, jnt.dim(), result2, 7, prt_cmJ, jnt.dim(), error))std::cout << "\"UniversalJoint:cptPrtCm\" failed" << std::endl;

		jnt.cptCa(result1);
		if (!s_is_equal(jnt.dim(), result1, ca, error))std::cout << "\"UniversalJoint:cptCa\" failed" << std::endl;

		jnt.cptCp(result1);
		if (!s_is_equal(jnt.dim(), result1, cp, error))std::cout << "\"UniversalJoint:cptCp\" failed" << std::endl;
	}
	
	// test motion //
	{
		const double relative_pe[6]{ 0,0,0.521,0,0,0 };
		const double relative_vs[6]{ 0,0,0.689,0,0,0 };
		const double relative_as[6]{ 0,0,0.123,0,0,0 };

		double relative_pm[16];
		s_pe2pm(relative_pe, relative_pm, "123");

		double mak_i_pm[16];
		s_pm_dot_pm(*mak_j.pm(), relative_pm, mak_i_pm);

		double glb_pm_m[16];
		s_pm_dot_inv_pm(mak_i_pm, *mak_i.prtPm(), glb_pm_m);

		prt_m.setPm(glb_pm_m);
		prt_m.setVs(mak_j, relative_vs);
		prt_m.setAs(mak_j, relative_as);

		auto &mot = model.add<Motion>("m1", &mak_i, &mak_j);

		const double glb_cmI[]{ 0.307993352194134,
			0.916076148165476,
			0.256796779120235,
			-0.522335646172068,
			0.0200980273534624,
			0.554775584177545 };
		const double glb_cmJ[]{ -0.307993352194134,
			- 0.916076148165476,
			- 0.256796779120235,
			0.522335646172068,
			- 0.0200980273534624,
			- 0.554775584177545 };
		const double prt_cmI[]{ 0.000416847668728071,
			- 0.983666521865018,
			0.18,
			0.331099956559505,
			- 0.0178749456993816,
			- 0.0984500217202474 };
		const double prt_cmJ[]{ -0.959549804517775,
			0.137640156385781,
			- 0.2456,
			- 0.0349660234578675,
			- 0.418969900086863,
			- 0.0981899087749607 };
		const double cp[6]{ -0.521,0,0,0,0,0 };
		const double ca[]{ 0 };


		double result1[42], result2[48];


		mot.cptGlbCm(result1, 5, result2, 7);
		if (!s_is_equal(6, mot.dim(), result1, 5, glb_cmI, mot.dim(), error) || !s_is_equal(6, mot.dim(), result2, 7, glb_cmJ, mot.dim(), error))std::cout << "\"Motion:cptGlbCm\" failed" << std::endl;

		mot.cptPrtCm(result1, 6, result2, 7);
		if (!s_is_equal(6, mot.dim(), result1, 6, prt_cmI, mot.dim(), error) || !s_is_equal(6, mot.dim(), result2, 7, prt_cmJ, mot.dim(), error))std::cout << "\"Motion:cptPrtCm\" failed" << std::endl;

		mot.cptCa(result1);
		if (!s_is_equal(mot.dim(), result1, ca, error))std::cout << "\"Motion:cptCa\" failed" << std::endl;

		mot.cptCp(result1);
		if (!s_is_equal(mot.dim(), result1, cp, error))std::cout << "\"Motion:cptCp\" failed" << std::endl;

		mot.updMp();
		if (!s_is_equal(mot.mp(), 0.521, error))std::cout << "\"Motion:updMp\" failed" << std::endl;

		mot.updMv();
		if (!s_is_equal(mot.mv(), 0.689, error))std::cout << "\"Motion:updMv\" failed" << std::endl;

		mot.updMa();
		if (!s_is_equal(mot.ma(), 0.123, error))std::cout << "\"Motion:updMa\" failed" << std::endl;
	}

	// test general motion //
	{
		const double relative_pe[6]{ 0.1,0.2,0.3,0.4,0.5,0.6 };
		const double relative_vs[6]{ 0,0,0.689,0,0,0 };

		double relative_pm[16];
		s_pe2pm(relative_pe, relative_pm, "123");

		double mak_i_pm[16];
		s_pm_dot_pm(*mak_j.pm(), relative_pm, mak_i_pm);

		double glb_pm_m[16];
		s_pm_dot_inv_pm(mak_i_pm, *mak_i.prtPm(), glb_pm_m);

		prt_m.setPm(glb_pm_m);
		prt_m.setVs(mak_j, relative_vs);

		auto &mot = model.add<GeneralMotion>("m1", &mak_i, &mak_j);
		
		const double mpm_default[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
		const double mpe313[6] = { 0.1, 0.2, 0.3,0.000423769269879415,   1.38980987554835,   1.79253453841257 };
		const double mpe321[6] = { 0.1, 0.2, 0.3,2.46823966120654, -1.28551725555848,  5.40636866254317 };
		const double mpq[7] = { 0.1, 0.2, 0.3,0.4,-0.5, 0.6, std::sqrt(1 - 0.4*0.4 - 0.5*0.5 - 0.6*0.6) };
		const double mpm[16] = { -0.22, -0.975499782797526,   0.000416847668728071, 0.1,
			0.175499782797526, -0.04, -0.983666521865018, 0.2,
			0.959583152331272, -0.216333478134982,   0.18,0.3,
			0,0,0,1 };

		const double mvs_default[6]{ 0,0,0,0,0,0 };
		const double mve313[6] = { 0.307558670154491,   1.2433000508379, -1.04895965543501,-0.644213536852877, -0.245050866834802, -1.27836042009784 };
		const double mve321[6] = { 0.307558670154491,   1.2433000508379, -1.04895965543501,-4.19969388864156, -0.83045134600268,   3.46543753721832 };
		const double mvq[7] = { 0.307558670154491,   1.2433000508379, -1.04895965543501, 0.1, 0.2, -0.4, -(mpq[3] * 0.1 + mpq[4] * 0.2 - mpq[5] * 0.4) / mpq[6] };
		const double mvm[16] = { 1.36, -0.30698536874045, -0.633709981238717,0.307558670154491,
			0.426985368740452,   0.8,   0.0436487757967661,1.2433000508379,
			0.233709981238715,   1.23635122420323,   0.24 , -1.04895965543501,
			0,0,0,0 };
		const double mva[6] = { 0.307558670154491,   1.2433000508379, -1.04895965543501, -0.244517963270725,	1.25737650310373,	-0.874318412470487 };
		const double mvs[6] = { -0.244517963270725,	1.25737650310373,	-0.874318412470487, -0.244517963270725,	1.25737650310373,	-0.874318412470487 };
		
		const double mas_default[6]{ 0,0,0,0,0,0 };
		const double mae313[6] = { 2.2628985000154, -0.843606386309081, -0.248846478459814, 1.51734920338156,   1.71538128045296,   1.3693196878275 };
		const double mae321[6] = { 2.2628985000154, -0.843606386309081, -0.248846478459814, -15.6049676192293,   4.50445705187534,   16.9352080725126 };
		const double maq[7] = { 2.2628985000154, -0.843606386309081, -0.248846478459814, -0.033,   0.022, 0.011,   -(mvq[3] * mvq[3] + mvq[4] * mvq[4] + mvq[5] * mvq[5] + mvq[6] * mvq[6] + mpq[3] * (-0.033) + mpq[4] * (0.022) + mpq[5] * (0.011)) / mpq[6] };
		const double mam[16] = { -0.782400000000002,   2.58144759895694,   1.54784395313479,2.2628985000154,
			-2.32024759895695, -0.653600000000002,   0.450521351741563,-0.843606386309081,
			-1.92944395313478, -1.05972135174157, -0.103200000000001,-0.248846478459814,
			0,0,0,0 };
		const double maa[6] = { 2.2628985000154, -0.843606386309081, -0.248846478459814, 0.904633672502324, -1.24440604199266,   1.45568007018557 };
		const double mas[6] = { 3.15925342342501, -0.192390604845803,   0.136512424183815,   0.904633672502324, -1.24440604199266,   1.45568007018557 };

		double result[36];

		mot.setMpm(mpm_default);
		if (!s_is_equal(16, *mot.mpm(), mpm_default, error))std::cout << "\"GeneralMotion:setMpm\" failed" << std::endl;

		mot.setMpe(mpe313, "313");
		if (!s_is_equal(16, *mot.mpm(), mpm, error))std::cout << "\"GeneralMotion:setMpe 313\" failed" << std::endl;

		mot.setMpm(mpm_default);
		if (!s_is_equal(16, *mot.mpm(), mpm_default, error))std::cout << "\"GeneralMotion:setMpm\" failed" << std::endl;

		mot.setMpe(mpe321, "321");
		if (!s_is_equal(16, *mot.mpm(), mpm, error))std::cout << "\"GeneralMotion:setMpe 321\" failed" << std::endl;

		mot.setMpm(mpm_default);
		if (!s_is_equal(16, *mot.mpm(), mpm_default, error))std::cout << "\"GeneralMotion:setMpm\" failed" << std::endl;

		mot.setMpq(mpq);
		if (!s_is_equal(16, *mot.mpm(), mpm, error))std::cout << "\"GeneralMotion:setMpq\" failed" << std::endl;
		
		mot.setMpm(mpm_default);
		if (!s_is_equal(16, *mot.mpm(), mpm_default, error))std::cout << "\"GeneralMotion:setMpm\" failed" << std::endl;

		mot.setMpm(mpm);
		if (!s_is_equal(16, *mot.mpm(), mpm, error))std::cout << "\"GeneralMotion:setMpm\" failed" << std::endl;

		mot.getMpe(result, "313");
		if (!s_is_equal(6, result, mpe313, error))std::cout << "\"GeneralMotion:getMpe\" failed" << std::endl;

		mot.getMpe(result, "321");
		if (!s_is_equal(6, result, mpe321, error))std::cout << "\"GeneralMotion:getMpe\" failed" << std::endl;

		mot.getMpq(result);
		if (!s_is_equal(7, result, mpq, error))std::cout << "\"GeneralMotion:getMpq\" failed" << std::endl;

		mot.getMpm(result);
		if (!s_is_equal(16, result, mpm, error))std::cout << "\"GeneralMotion:getMpm\" failed" << std::endl;


		mot.setMvs(mvs_default);
		if (!s_is_equal(6, mot.mvs(), mvs_default, error))std::cout << "\"GeneralMotion:setMvs\" failed" << std::endl;

		mot.setMve(mve313, "313");
		if (!s_is_equal(6, mot.mvs(), mvs, error))std::cout << "\"GeneralMotion:setMve 313\" failed" << std::endl;

		mot.setMvs(mvs_default);
		if (!s_is_equal(6, mot.mvs(), mvs_default, error))std::cout << "\"GeneralMotion:setMvs\" failed" << std::endl;

		mot.setMve(mve321, "321");
		if (!s_is_equal(6, mot.mvs(), mvs, error))std::cout << "\"GeneralMotion:setMve 321\" failed" << std::endl;

		mot.setMvs(mvs_default);
		if (!s_is_equal(6, mot.mvs(), mvs_default, error))std::cout << "\"GeneralMotion:setMvs\" failed" << std::endl;

		mot.setMvq(mvq);
		if (!s_is_equal(6, mot.mvs(), mvs, error))std::cout << "\"GeneralMotion:setMvq\" failed" << std::endl;

		mot.setMvs(mvs_default);
		if (!s_is_equal(6, mot.mvs(), mvs_default, error))std::cout << "\"GeneralMotion:setMvs\" failed" << std::endl;

		mot.setMvm(mvm);
		if (!s_is_equal(6, mot.mvs(), mvs, error))std::cout << "\"GeneralMotion:setMvm\" failed" << std::endl;

		mot.setMvs(mvs_default);
		if (!s_is_equal(6, mot.mvs(), mvs_default, error))std::cout << "\"GeneralMotion:setMvs\" failed" << std::endl;

		mot.setMva(mva);
		if (!s_is_equal(6, mot.mvs(), mvs, error))std::cout << "\"GeneralMotion:setMva\" failed" << std::endl;

		mot.setMvs(mvs_default);
		if (!s_is_equal(6, mot.mvs(), mvs_default, error))std::cout << "\"GeneralMotion:setMvs\" failed" << std::endl;

		mot.setMvs(mvs);
		if (!s_is_equal(6, mot.mvs(), mvs, error))std::cout << "\"GeneralMotion:setMvs\" failed" << std::endl;

		mot.getMve(result, "313");
		if (!s_is_equal(6, result, mve313, error))std::cout << "\"GeneralMotion:getMve\" failed" << std::endl;

		mot.getMve(result, "321");
		if (!s_is_equal(6, result, mve321, error))std::cout << "\"GeneralMotion:getMve\" failed" << std::endl;

		mot.getMvq(result);
		if (!s_is_equal(7, result, mvq, error))std::cout << "\"GeneralMotion:getMvq\" failed" << std::endl;

		mot.getMvm(result);
		if (!s_is_equal(16, result, mvm, error))std::cout << "\"GeneralMotion:getMvm\" failed" << std::endl;

		mot.getMva(result);
		if (!s_is_equal(6, result, mva, error))std::cout << "\"GeneralMotion:getMva\" failed" << std::endl;

		mot.getMvs(result);
		if (!s_is_equal(6, result, mvs, error))std::cout << "\"GeneralMotion:getMvs\" failed" << std::endl;


		mot.setMas(mas_default);
		if (!s_is_equal(6, mot.mas(), mas_default, error))std::cout << "\"GeneralMotion:setMas\" failed" << std::endl;

		mot.setMae(mae313, "313");
		if (!s_is_equal(6, mot.mas(), mas, error))std::cout << "\"GeneralMotion:setMae 313\" failed" << std::endl;

		mot.setMas(mas_default);
		if (!s_is_equal(6, mot.mas(), mas_default, error))std::cout << "\"GeneralMotion:setMas\" failed" << std::endl;

		mot.setMae(mae321, "321");
		if (!s_is_equal(6, mot.mas(), mas, error))std::cout << "\"GeneralMotion:setMae 321\" failed" << std::endl;

		mot.setMas(mas_default);
		if (!s_is_equal(6, mot.mas(), mas_default, error))std::cout << "\"GeneralMotion:setMas\" failed" << std::endl;

		mot.setMaq(maq);
		if (!s_is_equal(6, mot.mas(), mas, error))std::cout << "\"GeneralMotion:setMaq\" failed" << std::endl;

		mot.setMas(mas_default);
		if (!s_is_equal(6, mot.mas(), mas_default, error))std::cout << "\"GeneralMotion:setMas\" failed" << std::endl;

		mot.setMam(mam);
		if (!s_is_equal(6, mot.mas(), mas, error))std::cout << "\"GeneralMotion:setMam\" failed" << std::endl;

		mot.setMas(mas_default);
		if (!s_is_equal(6, mot.mas(), mas_default, error))std::cout << "\"GeneralMotion:setMas\" failed" << std::endl;

		mot.setMaa(maa);
		if (!s_is_equal(6, mot.mas(), mas, error))std::cout << "\"GeneralMotion:setMaa\" failed" << std::endl;

		mot.setMas(mas_default);
		if (!s_is_equal(6, mot.mas(), mas_default, error))std::cout << "\"GeneralMotion:setMas\" failed" << std::endl;

		mot.setMas(mas);
		if (!s_is_equal(6, mot.mas(), mas, error))std::cout << "\"GeneralMotion:setMas\" failed" << std::endl;
		
		mot.getMae(result, "313");
		if (!s_is_equal(6, result, mae313, error))std::cout << "\"GeneralMotion:getMae\" failed" << std::endl;

		mot.getMae(result, "321");
		if (!s_is_equal(6, result, mae321, error))std::cout << "\"GeneralMotion:getMae\" failed" << std::endl;

		mot.getMaq(result);
		if (!s_is_equal(7, result, maq, error))std::cout << "\"GeneralMotion:getMaq\" failed" << std::endl;

		mot.getMam(result);
		if (!s_is_equal(16, result, mam, error))std::cout << "\"GeneralMotion:getMam\" failed" << std::endl;

		mot.getMaa(result);
		if (!s_is_equal(6, result, maa, error))std::cout << "\"GeneralMotion:getMaa\" failed" << std::endl;

		mot.getMas(result);
		if (!s_is_equal(6, result, mas, error))std::cout << "\"GeneralMotion:getMas\" failed" << std::endl;

		const double glb_cmI[]{ 0.413710949602281, - 0.464491586871515,   0.783001159580726,   0,   0,   0,
			- 0.0419709404545899,   0.849409663646283,   0.526062414036267, 0, 0, 0,
			- 0.90944031708328, - 0.25050107590565,   0.331914929814216,   0,   0,   0,
			- 0.705694160625697, - 0.681200783491694, - 0.0312370311099321,   0.413710949602281, - 0.464491586871515,   0.783001159580726,
			1.04378672046201, - 0.0395661962367909,   0.147162423437692, - 0.0419709404545899,   0.849409663646283,   0.526062414036267,
			- 0.369196422575466,   1.12895372781328, - 0.159552895610227, - 0.90944031708328, - 0.25050107590565,   0.331914929814216 };
		const double glb_cmJ[]{ -0.413710949602281, 0.464491586871515,   -0.783001159580726,   0,   0,   0,
			0.0419709404545899,  - 0.849409663646283,   -0.526062414036267, 0, 0, 0,
			0.90944031708328, 0.25050107590565,   -0.331914929814216,   0,   0,   0,
			0.705694160625697, 0.681200783491694, 0.0312370311099321,   -0.413710949602281, 0.464491586871515,   -0.783001159580726,
			-1.04378672046201, 0.0395661962367909,   -0.147162423437692, 0.0419709404545899,   -0.849409663646283,   -0.526062414036267,
			0.369196422575466,   -1.12895372781328, 0.159552895610227, 0.90944031708328, 0.25050107590565,   -0.331914929814216 };
		const double prt_cmI[]{ -0.22, - 0.975499782797526,   0.000416847668728071,   0,   0,   0,
			0.175499782797526, - 0.04, - 0.983666521865018,   0,   0,   0,
			0.959583152331272, - 0.216333478134982,   0.18,   0,   0,   0,
			0.139266695626997, - 0.0312666956269964,   0.331099956559505, - 0.22, - 0.975499782797526,   0.000416847668728071,
			- 0.161958315233127, - 0.27101658702576, - 0.0178749456993816,   0.175499782797526, - 0.04, - 0.983666521865018,
			0.0615499782797526,   0.191099956559505, - 0.0984500217202474,   0.959583152331272, - 0.216333478134982,   0.18 };
		const double prt_cmJ[]{ 0.056450322927895, - 0.774310621053255, - 0.630282812049845,   0,   0,   0,
			0.667701575653235, - 0.440066920670111,   0.600429605534333, - 0, - 0, - 0,
			0.742285637010123,   0.454735271840714, - 0.492166501940581, - 0, - 0, - 0,
			- 0.347790781866912,   0.0242346634708827, - 0.0609219520773668,   0.056450322927895, - 0.774310621053255, - 0.630282812049845,
			- 0.139627559696223, - 0.311594926010587, - 0.0731027876832712,   0.667701575653235, - 0.440066920670111,   0.600429605534333,
			0.152047187678477, - 0.260277725507541, - 0.0111649587681636,   0.742285637010123,   0.454735271840714, - 0.492166501940581 };
		const double ce[6]{ 0,0,0,0,0,0 };
		const double ca[]{ 0,0,0,0,0,0 };


		double result1[42], result2[48];

		mot.setMpe(relative_pe, "123");
		

		mot.cptGlbCm(result1, 6, result2, 7);
		if (!s_is_equal(6, mot.dim(), result1, 6, glb_cmI, mot.dim(), error) || !s_is_equal(6, mot.dim(), result2, 7, glb_cmJ, mot.dim(), error))std::cout << "\"GeneralMotion:cptGlbCm\" failed" << std::endl;

		mot.cptPrtCm(result1, 6, result2, 7);
		if (!s_is_equal(6, mot.dim(), result1, 6, prt_cmI, mot.dim(), error) || !s_is_equal(6, mot.dim(), result2, 7, prt_cmJ, mot.dim(), error))std::cout << "\"GeneralMotion:cptPrtCm\" failed" << std::endl;

		mot.cptCp(result1);
		if (!s_is_equal(mot.dim(), result1, ce, error))std::cout << "\"GeneralMotion:cptCp\" failed" << std::endl;

		//mot.cptCa(result1);
		//if (!s_is_equal(mot.dim(), result1, ca, error))std::cout << "\"Motion:cptCa\" failed" << std::endl;



		//mot.updMp();
		//if (std::abs(mot.mp() - 0.521)>error)std::cout << "\"Motion:updMp\" failed" << std::endl;

		//mot.updMv();
		//if (std::abs(mot.mv() - 0.689)>error)std::cout << "\"Motion:updMv\" failed" << std::endl;

		//mot.updGlbCm();
		//mot.updPrtCm();
		//mot.updCa();
		//mot.updCe();

		//dlmwrite("C:\\Users\\py033\\Desktop\\cm.txt", mot.ca(), 1, mot.dim());

	}
}

void test_model_interaction()
{
	std::cout << std::endl << "-----------------test model interaction---------------------" << std::endl;
	test_constraint();
	std::cout << "-----------------test model interaction finished------------" << std::endl << std::endl;
}

