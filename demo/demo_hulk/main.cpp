#include "aris.h"

using namespace std;
using namespace aris::dynamic;

class HulkInverseSolver :public aris::dynamic::DiagSolver
{
public:
	static const std::string& Type() { static const std::string type("HulkInverseSolver"); return type; }
	auto virtual type() const->const std::string& override{ return Type(); }
	auto virtual kinPos()->void override
	{
		auto &ee = model().generalMotionPool().front();

		double mpm[16];
		ee.getMpm(mpm);

		const double x = ee.mpm()[0][3];
		const double y = ee.mpm()[1][3];
		const double z = ee.mpm()[2][3];

		double l = std::sqrt(x * x + (y - 0.29)*(y - 0.29) + z*z);

		double q[6];
		q[0] = atan2(-z, x);
		q[1] = atan2(y - 0.29, std::sqrt(x * x + z * z)) + std::acos((l*l + 0.27*0.27 - 0.104756) / 0.27 / l / 2.0) - PI / 2;
		q[2] = std::acos((0.104756 + 0.27*0.27 - l*l) / 0.27 / 0.323660315763301 / 2.0) - 1.788795031482338;

		double first_3_rm[9], last_3_rm[9];
		double first_q[3]{ q[0],q[1] + q[2],0 };

		aris::dynamic::s_re2rm(first_q, first_3_rm, "232");
		s_mm(3, 3, 3, first_3_rm, ColMajor{ 3 }, *ee.mpm(), 4, last_3_rm, 3);

		s_rm2re(last_3_rm, q + 3, "131");

		double pm[16];
		double pe1[6]{ 0,0,0,0,q[0],0 };
		double pe2[6]{ 0,0.29,0,0,0,q[1] };
		double pe3[6]{ 0,0.56,0,0,0,q[2] };
		double pe4[6]{ 0,0.63,0,q[3],0,0 };
		double pe5[6]{ 0.316,0.63,0,0,0,q[4] };
		double pe6[6]{ 0.316,0.63,0,q[5],0,0 };

		s_pe2pm(pe1, pm, "123");
		s_mms(3, 1, 3, pm, 4, pe1, 1, pm + 3, 4);
		model().partPool().at(1).setPm(model().partPool().at(0), pm);

		s_pe2pm(pe2, pm, "123");
		s_mms(3, 1, 3, pm, 4, pe2, 1, pm + 3, 4);
		model().partPool().at(2).setPm(model().partPool().at(1), pm);

		s_pe2pm(pe3, pm, "123");
		s_mms(3, 1, 3, pm, 4, pe3, 1, pm + 3, 4);
		model().partPool().at(3).setPm(model().partPool().at(2), pm);

		s_pe2pm(pe4, pm, "123");
		s_mms(3, 1, 3, pm, 4, pe4, 1, pm + 3, 4);
		model().partPool().at(4).setPm(model().partPool().at(3), pm);

		s_pe2pm(pe5, pm, "123");
		s_mms(3, 1, 3, pm, 4, pe5, 1, pm + 3, 4);
		model().partPool().at(5).setPm(model().partPool().at(4), pm);

		s_pe2pm(pe6, pm, "123");
		s_mms(3, 1, 3, pm, 4, pe6, 1, pm + 3, 4);
		model().partPool().at(6).setPm(model().partPool().at(5), pm);
	}

	HulkInverseSolver(const std::string &name) :DiagSolver(name) {};
	HulkInverseSolver(Object &father, const aris::core::XmlElement &xml_ele) :DiagSolver(father, xml_ele) {};
};
class HulkForwardSolver :public aris::dynamic::DiagSolver
{
public:
	static const std::string& Type() { static const std::string type("HulkForwardSolver"); return type; }
	auto virtual type() const->const std::string& override{ return Type(); }
	auto virtual kinPos()->void override
	{
		double q[6];

		double pm[16];
		double pe1[6]{ 0,0,0,0,model().motionPool().at(0).mp(),0 };
		double pe2[6]{ 0,0.29,0,0,0,model().motionPool().at(1).mp() };
		double pe3[6]{ 0,0.56,0,0,0,model().motionPool().at(2).mp() };
		double pe4[6]{ 0,0.63,0,model().motionPool().at(3).mp(),0,0 };
		double pe5[6]{ 0.316,0.63,0,0,0,model().motionPool().at(4).mp() };
		double pe6[6]{ 0.316,0.63,0,model().motionPool().at(5).mp(),0,0 };

		s_pe2pm(pe1, pm, "123");
		s_mms(3, 1, 3, pm, 4, pe1, 1, pm + 3, 4);
		model().partPool().at(1).setPm(model().partPool().at(0), pm);

		s_pe2pm(pe2, pm, "123");
		s_mms(3, 1, 3, pm, 4, pe2, 1, pm + 3, 4);
		model().partPool().at(2).setPm(model().partPool().at(1), pm);

		s_pe2pm(pe3, pm, "123");
		s_mms(3, 1, 3, pm, 4, pe3, 1, pm + 3, 4);
		model().partPool().at(3).setPm(model().partPool().at(2), pm);

		s_pe2pm(pe4, pm, "123");
		s_mms(3, 1, 3, pm, 4, pe4, 1, pm + 3, 4);
		model().partPool().at(4).setPm(model().partPool().at(3), pm);

		s_pe2pm(pe5, pm, "123");
		s_mms(3, 1, 3, pm, 4, pe5, 1, pm + 3, 4);
		model().partPool().at(5).setPm(model().partPool().at(4), pm);

		s_pe2pm(pe6, pm, "123");
		s_mms(3, 1, 3, pm, 4, pe6, 1, pm + 3, 4);
		model().partPool().at(6).setPm(model().partPool().at(5), pm);

		
	}

	HulkForwardSolver(const std::string &name) :DiagSolver(name) {};
	HulkForwardSolver(Object &father, const aris::core::XmlElement &xml_ele) :DiagSolver(father, xml_ele) {};
};


int main()
{
	try
	{
		double pm[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
		aris::dynamic::Model m;
		auto &p1 = m.addPartByPm(pm);
		auto &p2 = m.addPartByPm(pm);
		auto &p3 = m.addPartByPm(pm);
		auto &p4 = m.addPartByPm(pm);
		auto &p5 = m.addPartByPm(pm);
		auto &p6 = m.addPartByPm(pm);
		p1.geometryPool().add<ParasolidGeometry>("solid", "C:\\aris\\resource\\demo_hulk\\p1.xmt_txt");
		p2.geometryPool().add<ParasolidGeometry>("solid", "C:\\aris\\resource\\demo_hulk\\p2.xmt_txt");
		p3.geometryPool().add<ParasolidGeometry>("solid", "C:\\aris\\resource\\demo_hulk\\p3.xmt_txt");
		p4.geometryPool().add<ParasolidGeometry>("solid", "C:\\aris\\resource\\demo_hulk\\p4.xmt_txt");
		p5.geometryPool().add<ParasolidGeometry>("solid", "C:\\aris\\resource\\demo_hulk\\p5.xmt_txt");
		p6.geometryPool().add<ParasolidGeometry>("solid", "C:\\aris\\resource\\demo_hulk\\p6.xmt_txt");
		auto &j1 = m.addRevoluteJoint(m.ground(), p1, std::array<double, 3>{0, 0, 0}.data(), std::array<double, 3>{0, 1, 0}.data());
		auto &j2 = m.addRevoluteJoint(p1, p2, std::array<double, 3>{0, 0.29, 0}.data(), std::array<double, 3>{0, 0, 1}.data());
		auto &j3 = m.addRevoluteJoint(p2, p3, std::array<double, 3>{0, 0.56, 0}.data(), std::array<double, 3>{0, 0, 1}.data());
		auto &j4 = m.addRevoluteJoint(p3, p4, std::array<double, 3>{0, 0.63, 0}.data(), std::array<double, 3>{1, 0, 0}.data());
		auto &j5 = m.addRevoluteJoint(p4, p5, std::array<double, 3>{0.316, 0.63, 0}.data(), std::array<double, 3>{0, 0, 1}.data());
		auto &j6 = m.addRevoluteJoint(p5, p6, std::array<double, 3>{0.316, 0.63, 0}.data(), std::array<double, 3>{1, 0, 0}.data());
		auto &m1 = m.addMotion(j1);
		auto &m2 = m.addMotion(j2);
		auto &m3 = m.addMotion(j3);
		auto &m4 = m.addMotion(j4);
		auto &m5 = m.addMotion(j5);
		auto &m6 = m.addMotion(j6);

		double ee_pm[16]{ 1,0,0,0.316,0,1,0,0.63,0,0,1,0,0,0,0,1 };
		auto &ee = m.addGeneralMotion(p6, m.ground(), ee_pm);

		auto &ds = m.solverPool().add<aris::dynamic::DiagSolver>("ds");
		auto &s = m.simulatorPool().add<aris::dynamic::AdamsSimulator>("s", ds);
		auto &r = m.simResultPool().add<aris::dynamic::SimResult>("r");
		auto plan1 = [](const aris::dynamic::PlanParam &param)->int
		{
			std::array<double, 6> ee_pe{ 0.316, 0.63, param.count_ * 0.0001, 0, 0, 0 };
			param.model_->generalMotionPool().at(0).setMpe(ee_pe.data());
			param.model_->setTime(param.count_ * 0.001);
			return 3000 - param.count_;
		};
		auto plan2 = [](const aris::dynamic::PlanParam &param)->int
		{
			const double dt{0.01};
			
			aris::Size total_coult;
			double p, v, a;
			aris::dynamic::moveAbsolute(param.count_, 0.0, PI, 0.5 * dt, 0.3 * dt * dt, 0.5 * dt * dt, p, v, a, total_coult);
			
			param.model_->setTime(param.count_ * dt);
			param.model_->motionPool().at(0).setMp(p);
			return total_coult - param.count_;
		};

		m.registerChildType<HulkInverseSolver>();
		m.registerChildType<HulkForwardSolver>();
		auto &inv = m.solverPool().add<HulkInverseSolver>("inverse");
		auto &fwd = m.solverPool().add<HulkForwardSolver>("forward");
		auto &inv_sim = m.simulatorPool().add<aris::dynamic::AdamsSimulator>("inv_sim", inv);
		auto &fwd_sim = m.simulatorPool().add<aris::dynamic::AdamsSimulator>("fwd_sim", fwd);

		for (auto &mot : m.motionPool())mot.activate(true);
		m.generalMotionPool().front().activate(false);

		fwd_sim.simulate(plan2, 0, r);

		fwd_sim.saveAdams("C:\\Users\\py033\\Desktop\\hulk.cmd", r);
		m.saveXml("C:\\Users\\py033\\Desktop\\hulk.xml");

		//if (!m.inverseKinematic())std::cout << "failed" << std::endl;

	}
	catch (std::exception &e)
	{
		std::cout << e.what() << std::endl;
	}
	std::cout << "demo_3R finished, press any key to continue" << std::endl;
	std::cin.get();
	return 0;
}

