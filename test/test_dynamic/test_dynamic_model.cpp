#include "test_dynamic_model.h"
#include <iostream>
#include <aris.h>
#include <Eigen\Eigen>

using namespace aris::dynamic;




void test_model()
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
	auto &makJ = model.ground().markerPool().add("makJ",pmJ);
	auto &makI = prt1.markerPool().add("makI", pmI);
	auto &jnt = model.jointPool().add<aris::dynamic::RevoluteJoint>("joint1", std::ref(makI), std::ref(makJ));
	auto &mot = model.motionPool().add<aris::dynamic::SingleComponentMotion>("motion1", std::ref(makI), std::ref(makJ), 5);

	mot.setMotPos(0);

	model.dynPre();
	model.dynUpd();
	model.dynCstPot(csp);

	//dsp(csp, 12, 1);
	model.dynCstMtx(*csm2);

	dsp(*csm2, 12, 12);

	s_mtm(12, 12, *csm2,12, *csm, 12);

	dsp(*csm, 12, 12);

	Eigen::Map<Eigen::Matrix<double, 12, 12, Eigen::RowMajor> > csm_mtx(*csm);
	Eigen::Map<Eigen::Matrix<double, 12, 1> > csp_mtx(csp);
	Eigen::Map<Eigen::Matrix<double, 12, 1> > b_mtx(b);

	b_mtx = csm_mtx.partialPivLu().solve(csp_mtx);

	dsp(csp, 12, 1);
	dsp(b, 12, 1);

	//auto &prismatic1 = model.jointPool().
	dlmwrite("C:\\Users\\yang\\Desktop\\test.txt", *csm, 12, 12);



	std::cout << "finished" << std::endl;
}

