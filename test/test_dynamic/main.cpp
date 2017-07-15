#include <iostream>
#include "test_dynamic_matrix.h"
#include "test_dynamic_block_matrix.h"
#include "test_dynamic_cell.h"
#include "test_dynamic_screw.h"
#include "test_dynamic_spline.h"
#include "test_dynamic_model.h"
#include "test_dynamic_simple_model.h"
#include "test_dynamic_plan.h"

#include <aris.h>


int main(int argc, char *argv[])
{
	//test_matrix();
	//test_block_matrix();
	//test_cell();
	//test_screw();
	//test_spline();
	//test_model();
	//test_simple_model();
	//test_plan();

	
	aris::dynamic::Model m;
	
	double pe[]{ 0,0,0,0,0,0, };
	auto &p1 = m.addPartByPe(pe, "313");
	auto &p2 = m.addPartByPe(pe, "313");
	auto &p3 = m.addPartByPe(pe, "313");

	const double pos1[]{ -0.5164795, 0.9196533, 1.521986 };
	const double z1[]{ 0,0,1 };
	const double pos2[]{ -0.6173706, 1.058447, 1.521986 };
	const double z2[]{ 0,0,1 };
	const double pos3[]{ -0.5897217, 0.9725097, 1.521986 };
	const double z3[]{ 0,0,1 };
	auto &j1 = m.addRevoluteJoint(m.ground(), p1, pos1, z1);
	auto &j2 = m.addRevoluteJoint(p1, p2, pos2, z2);
	auto &j3 = m.addRevoluteJoint(p2, p3, pos3, z3);

	auto &m1 = m.addMotion(j1);
	auto &m2 = m.addMotion(j2);
	auto &m3 = m.addMotion(j3);

	double pm[16]
	{
		1,0,0,-0.6173706,
		0,1,0,1.117773,
		0,0,1,-1.502631,
		0,0,0,1
	};
	auto &ee = m.addGeneralMotion(p3, m.ground(), pm);

	auto &ds = m.solverPool().add<aris::dynamic::DiagSolver>("ds", 100, 1e-5);
	for (auto &mot : m.motionPool())mot.activate(false);

	ee.setMpm(pm);
	ds.allocateMemory();
	ds.kinPos();

	auto &as = m.simulatorPool().add<aris::dynamic::AdamsSimulator>("s", ds);
	auto &r = m.simResultPool().add<aris::dynamic::SimResult>("result");

	auto plan = [&](const aris::dynamic::PlanParam& param)->int
	{
		double pm[16]
		{
			1,0,0,-0.6173706,
			0,1,0,1.117773,
			0,0,1,-1.502631,
			0,0,0,1
		};

		pm[3] += 0.001 * std::sin(2 * PI*param.count_ / 1000);
		param.model_->generalMotionPool().at(0).setMpm(pm);
		param.model_->setTime(param.count_ * 0.001);
		return 1000 - param.count_;
	};

	r.allocateMemory();
	aris::dynamic::PlanParam p;
	as.simulate(plan, 0, r);

	as.saveAdams("C:\\Users\\py033\\Desktop\\meng.cmd", r, 0);
	m.saveXml("C:\\Users\\py033\\Desktop\\meng.xml");


	std::ofstream file;

	file.open("C:\\Users\\py033\\Desktop\\data.txt");
	auto size = r.size();

	for (int i(-1); ++i < 3;)
	{
		file << "float x" << i << "[" << size << "] = {";

		for (int j(-1); ++j < size;)
		{
			r.restore(j);
			m.motionPool().at(i).updMp();

			file << m.motionPool().at(i).mp() << ",";
		}
		file << "}" << std::endl;
	}



	aris::core::XmlDocument doc;
	doc.LoadFile("C:\\Users\\py033\\Desktop\\res.xml");
	auto motion = doc.RootElement()->FirstChildElement("MotionList");


	//int t{ 0 };
	//std::vector<std::array<float, 3> > data;
	//for (auto m = motion->FirstChildElement(); m = m->NextSiblingElement();)
	//{
	//	data.resize(t + 1);
	//	m->QueryFloatAttribute("motion0", &data[t][0]);
	//	m->QueryFloatAttribute("motion1", &data[t][1]);
	//	m->QueryFloatAttribute("motion2", &data[t][2]);
	//	++t;
	//}
	//std::cout << "total number" << t << std::endl;

	//std::ofstream file;

	//file.open("C:\\Users\\py033\\Desktop\\data.txt");

	//file << std::setprecision(7);

	//for (int i(-1); ++i < 3;)
	//{
	//	file << "float x1[" << i << "] = {";
	//	
	//	for (int j(-1); ++j < t;)
	//	{
	//		file << data[j][i] << ",";
	//	}
	//	file <<"}"<< std::endl;
	//}

	

	std::cout << "test_dynamic finished, press any key to continue" << std::endl;
	std::cin.get();
	return 0;
}