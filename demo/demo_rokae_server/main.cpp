#include <aris.hpp>

class Quad :public aris::dynamic::Model{
public:
	Quad() {
		// add part //
		const double default_iv[10]{ 1,0,0,0,0,0,0,0,0,0 };
		auto &body = this->partPool().add<aris::dynamic::Part>("BODY", default_iv);
		auto &lf_p1 = this->partPool().add<aris::dynamic::Part>("LF_P1", default_iv);
		auto &lf_p2 = this->partPool().add<aris::dynamic::Part>("LF_P2", default_iv);
		auto &lf_p3 = this->partPool().add<aris::dynamic::Part>("LF_P3", default_iv);
		auto &rf_p1 = this->partPool().add<aris::dynamic::Part>("RF_P1", default_iv);
		auto &rf_p2 = this->partPool().add<aris::dynamic::Part>("RF_P2", default_iv);
		auto &rf_p3 = this->partPool().add<aris::dynamic::Part>("RF_P3", default_iv);
		auto &lr_p1 = this->partPool().add<aris::dynamic::Part>("LR_P1", default_iv);
		auto &lr_p2 = this->partPool().add<aris::dynamic::Part>("LR_P2", default_iv);
		auto &lr_p3 = this->partPool().add<aris::dynamic::Part>("LR_P3", default_iv);
		auto &rr_p1 = this->partPool().add<aris::dynamic::Part>("RR_P1", default_iv);
		auto &rr_p2 = this->partPool().add<aris::dynamic::Part>("RR_P2", default_iv);
		auto &rr_p3 = this->partPool().add<aris::dynamic::Part>("RR_P3", default_iv);

		// add geometry //
		rf_p3.geometryPool().add<aris::dynamic::FileGeometry>("C:\\aris\\rf_leg.x_t");

		const double leg_pe[4][6]{
		{ -0.3, 0.0, -0.4, 0.0, 0.0, 0.0 },
		{ -0.3, 0.0, 0.4, 0.0, 0.0, 0.0 },
		{ 0.3, 0.0, -0.4, 0.0, 0.0, 0.0 },
		{ 0.3, 0.0, 0.4, 0.0, 0.0, 0.0 },
		};

		const double j2_pos[4][6]{
			{ -0.3, -0.4, -0.4, 0.0, 0.0, 0.0 },
			{ -0.3, -0.4, 0.4, 0.0, 0.0, 0.0 },
			{ 0.3, -0.4, -0.4, 0.0, 0.0, 0.0 },
			{ 0.3, -0.4, 0.4, 0.0, 0.0, 0.0 },
		};

		const double ee_pos[4][6]{
			{ -0.3, -0.8, -0.4, 0.0, 0.0, 0.0 },
		{ -0.3, -0.8, 0.4, 0.0, 0.0, 0.0 },
		{ 0.3, -0.8, -0.4, 0.0, 0.0, 0.0 },
		{ 0.3, -0.8, 0.4, 0.0, 0.0, 0.0 },
		};

		// add joints //
		auto &lf_r1 = this->addRevoluteJoint(lf_p1, body, leg_pe[0], std::array<double, 3>{0, 0, 1}.data());
		auto &lf_r2 = this->addRevoluteJoint(lf_p2, lf_p1, leg_pe[0], std::array<double, 3>{1, 0, 0}.data());
		auto &lf_r3 = this->addRevoluteJoint(lf_p3, lf_p2, j2_pos[0], std::array<double, 3>{1, 0, 0}.data());
		auto &rf_r1 = this->addRevoluteJoint(rf_p1, body, leg_pe[1], std::array<double, 3>{0, 0, 1}.data());
		auto &rf_r2 = this->addRevoluteJoint(rf_p2, rf_p1, leg_pe[1], std::array<double, 3>{1, 0, 0}.data());
		auto &rf_r3 = this->addRevoluteJoint(rf_p3, rf_p2, j2_pos[1], std::array<double, 3>{1, 0, 0}.data());
		auto &lr_r1 = this->addRevoluteJoint(lr_p1, body, leg_pe[2], std::array<double, 3>{0, 0, 1}.data());
		auto &lr_r2 = this->addRevoluteJoint(lr_p2, lr_p1, leg_pe[2], std::array<double, 3>{1, 0, 0}.data());
		auto &lr_r3 = this->addRevoluteJoint(lr_p3, lr_p2, j2_pos[2], std::array<double, 3>{1, 0, 0}.data());
		auto &rr_r1 = this->addRevoluteJoint(rr_p1, body, leg_pe[3], std::array<double, 3>{0, 0, 1}.data());
		auto &rr_r2 = this->addRevoluteJoint(rr_p2, rr_p1, leg_pe[3], std::array<double, 3>{1, 0, 0}.data());
		auto &rr_r3 = this->addRevoluteJoint(rr_p3, rr_p2, j2_pos[3], std::array<double, 3>{1, 0, 0}.data());

		auto &lf_m1 = this->addMotion(lf_r1);
		auto &lf_m2 = this->addMotion(lf_r2);
		auto &lf_m3 = this->addMotion(lf_r3);
		auto &rf_m1 = this->addMotion(rf_r1);
		auto &rf_m2 = this->addMotion(rf_r2);
		auto &rf_m3 = this->addMotion(rf_r3);
		auto &lr_m1 = this->addMotion(lr_r1);
		auto &lr_m2 = this->addMotion(lr_r2);
		auto &lr_m3 = this->addMotion(lr_r3);
		auto &rr_m1 = this->addMotion(rr_r1);
		auto &rr_m2 = this->addMotion(rr_r2);
		auto &rr_m3 = this->addMotion(rr_r3);

		// add end-effector //
		auto body_ee_maki = body.addMarker("body_ee_mak_i");
		auto body_ee_makj = ground().addMarker("body_ee_mak_j");

		auto &body_ee = this->generalMotionPool().add<aris::dynamic::GeneralMotion>("body_ee", &body_ee_maki, &body_ee_makj);
		auto &lf_ee = this->addPointMotion(lf_p3, ground(), ee_pos[0]);
		auto &rf_ee = this->addPointMotion(rf_p3, ground(), ee_pos[1]);
		auto &lr_ee = this->addPointMotion(lr_p3, ground(), ee_pos[2]);
		auto &rr_ee = this->addPointMotion(rr_p3, ground(), ee_pos[3]);

		lf_ee.activate(false);
		rf_ee.activate(false);
		lr_ee.activate(false);
		rr_ee.activate(false);

		auto &inverse_kinematic_solver = this->solverPool().add<aris::dynamic::InverseKinematicSolver>();
		auto &forward_kinematic_solver = this->solverPool().add<aris::dynamic::ForwardKinematicSolver>();
		auto &inverse_dynamic_solver = this->solverPool().add<aris::dynamic::InverseDynamicSolver>();
		auto &forward_dynamic_solver = this->solverPool().add<aris::dynamic::ForwardDynamicSolver>();

		auto &adams = this->simulatorPool().add<aris::dynamic::AdamsSimulator>();
		this->init();
	};
};



class MoveCos1 :public aris::core::CloneObject<MoveCos1, aris::plan::Plan> {
public:
	auto virtual executeRT()->int override{
		double h = (1 - std::cos(count() / 1000.0 * aris::PI))/2 + 0.18812625017241;

		//
		//
		//

		double ee[]{
			1,0,0,0,0,1,0,h/2,0,0,1,0,0,0,0,1,//body
			0,h,0,
			0,h,0,
			0,h,0,
			0,h,0,
		};

		model()->setOutputPos(ee);
		if(model()->inverseKinematics())std::cout << "inverse failed" << std::endl;

		model()->setTime(0.001 * count());
		return 2000 - count();
	}

	explicit MoveCos1()
	{
		//command().loadXmlStr(
		//	"<Command name=\"mv_cos1\">"
		//	"</Command>"
		//);
	}
};

int main(int argc, char *argv[])
{
	Quad quad;

	double mp[12]{ 0,-0.7,1.4, 0,-0.7,1.4, 0,-0.7,1.4, 0,-0.7,1.4 };
	quad.setInputPos(mp);
	if (quad.forwardKinematics())THROW_FILE_LINE("forward failed");


	auto &adams = dynamic_cast<aris::dynamic::AdamsSimulator&>(quad.simulatorPool().front());
	auto &result = quad.simResultPool().add<aris::dynamic::SimResult>();
	quad.init();


	MoveCos1 plan;
	adams.simulate(plan, result);
	adams.saveAdams("C:\\aris\\quad.cmd", result);

	char a;
	std::cin >> a;

	return 0;
}