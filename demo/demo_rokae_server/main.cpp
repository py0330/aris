#include <aris.hpp>
#include <iostream>

//#define EIGEN_BDCSVD_DEBUG_VERBOSE
//
//#include "eigen-3.3.9/Eigen/SVD"
//#include "eigen-3.3.9/Eigen/Dense"
//using namespace Eigen;

//class MoveCos1 :public aris::plan::Plan
//{
//public:
//	auto virtual executeRT()->int override
//	{
//		
//		
//		
//		
//		return 0;
//	}
//
//	explicit MoveCos1()
//	{
//		//command().loadXmlStr(
//		//	"<Command name=\"mv_cos1\">"
//		//	"</Command>"
//		//);
//	}
//};

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
/*
int main(int argc, char *argv[])
{
	Quad quad;

	auto &solver = quad.solverPool().add<aris::dynamic::UniversalSolver>();
	quad.generalMotionPool()[0].activate(false);
	quad.generalMotionPool()[1].activate(true);
	quad.generalMotionPool()[2].activate(false);
	quad.generalMotionPool()[3].activate(false);
	quad.generalMotionPool()[4].activate(true);
	solver.allocateMemory();



	double mp[12]{ 0,-0.7,1.4, 0,-0.7,1.4, 0,-0.7,1.4, 0,-0.7,1.4 };
	quad.setInputPos(mp);
	if (quad.forwardKinematics())THROW_FILE_LINE("forward failed");

	double mv[12]{ 0,0,0,0,0,0,0,0,0,0,0,0 };
	quad.setInputVel(mv);
	quad.forwardKinematicsVel();

	double ma[12]{ 0,0,0,0,0,0,0,0,0,0,0,0 };
	quad.setInputAcc(ma);

	solver.dynAccAndFce();

	double mf[12];
	quad.getInputFce(mf);

	aris::dynamic::dsp(1, 12, mf);




	char c;
	std::cin >> c;

	auto &adams = dynamic_cast<aris::dynamic::AdamsSimulator&>(quad.simulatorPool().front());
	auto &result = quad.simResultPool().add<aris::dynamic::SimResult>();
	quad.init();


	MoveCos1 plan;
	adams.simulate(plan, result);
	adams.saveAdams("C:\\aris\\quad.cmd", result);



	//MatrixXd A(21,20);
	//A << 1.0595376053415393 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 ,
	//	0.5078488308295366 , 23.4994794676354000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 ,
	//	0.0000000000000000 , 0.5856091257018784 , 44.9399596652721560 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 ,
	//	0.0000000000000000 , 0.0000000000000000 , 0.7628870959107409 , 33.3390692585205530 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 ,
	//	0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0829626491105439 , 28.0869884267800670 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 ,
	//	0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.6615961930827142 , 46.6488840384149770 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 ,
	//	0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.5169790147062131 , 5.2756875807179293 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 ,
	//	0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.1710480175254472 , 49.0348189720855320 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 ,
	//	0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.9385578643318424 , 5.8329014833849842 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 ,
	//	0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.5904831771425717 , 27.9178249372350250 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 ,
	//	0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.4406346807608401 , 7.7304251925131116 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 ,
	//	0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.9419189303112829 , 30.2060109097622130 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 ,
	//	0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.6559138202571413 , 0.2472996931554676 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 ,
	//	0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.4519457092603858 , 41.4008279255603210 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 ,
	//	0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.8396974207196690 , 45.8302982287472460 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 ,
	//	0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.5326235024743463 , 49.5083485937018540 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 ,
	//	0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.5538870657912750 , 53.2962868383175420 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 ,
	//	0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.6800655300833607 , 27.2771874971364170 , 0.0000000000000000 , 0.0000000000000000 ,
	//	0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.3671899053173674 , 14.6567677185458130 , 0.0000000000000000 ,
	//	0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.2392906061935454 , 5.4405276437467238 ,
	//	0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.0000000000000000 , 0.5789234924590936
	//	;
	//
	//A;

	//std::cout.precision(15);

	//MatrixXd B = A;
	//BDCSVD<Eigen::MatrixXd> svd(B, ComputeFullU | ComputeFullV);
	//MatrixXd V = svd.matrixV(), U = svd.matrixU();
	//MatrixXd  S = U.inverse() * A * V.transpose().inverse(); // S = U^-1 * A * VT * -1
	//std::cout << "A :\n" << A << std::endl;
	//std::cout << "U :\n" << U << std::endl;
	//std::cout << "S :\n" << S << std::endl;
	//std::cout << "V :\n" << V << std::endl;
	//std::cout << "U * S * VT :\n" << U * S * V.transpose() << std::endl;
	////system("pause");



	char a;
	std::cin >> a;

	return 0;
}
*/
struct B1 {
	int x;
	virtual ~B1() { }
};

struct B2 {
	int y;
	virtual ~B2() { }
};

struct D : B1, B2 { };

#define ERROR1 aris::core::LogLvl::kDebug, -1001, {"ERROR:%s is not correct", "错误%s"}

int print_out(int level, int code, std::initializer_list<const char*> msg) {
	std::cout << level << code << msg.begin()[1] << std::endl;
	return 0;
}


#define LOG_ERROR(E) print_out(E);





#define _LOG_I(level, code, info, ...)               \
	aris::core::log(level, code, \
	std::string(__FILE__).substr(std::string(__FILE__).find_last_of("/\\") + 1) + "|"+ __LINE__+"|"+aris::core::localeString(info, __VA_ARGS__)) 
#define _LOG_II(...) _LOG_I(__VA_ARGS__)

#define LOG(...) _LOG_II(__VA_ARGS__)

//#define PP_CONCAT(A, B) PP_CONCAT_IMPL(A, B)
//#define PP_CONCAT_IMPL(A, B) A##B

#define PP_CONCAT(A, B) A##B

#define FOO(N) PP_CONCAT(foo_, N)

#define BAR() bar

auto FOO(bar)()->void {

};    // -> foo_bar


auto FOO(BAR())(int) {
	return 0;
}  // -> foo_bar


#define ADD_TWO_IMPL(first, second) first + second
#define UNPACK(macro, args) macro args
#define ADD_TWO(...) UNPACK(ADD_TWO_IMPL, (__VA_ARGS__))

#define a_and_b a,b


#define ERROR_123 aris::core::LogLvl::kError, -999, { "AAAAAAAAAAAAAAAAAAAAAASSSSSSSDDDDDDDSSSSSSSDDDDDDDDDDDSSSSSSSSSSDDDDDDDDDD" }
int main() {

	int a=1, b=2;
	std::cout << ADD_TWO(a_and_b) << std::endl;
	std::cout << (1+(a,b)) << std::endl;

	D x;
	B1* b1_ptr = &x;
	B2* b2_ptr = &x;
	std::cout << "original address:     " << &x << "\n";

	std::cout << "b1_ptr:               " << b1_ptr << "\n";
	std::cout << "dynamic_cast b1_ptr:  " << dynamic_cast<void*>(b1_ptr) << "\n";

	std::cout << "b2_ptr:               " << b2_ptr << "\n";
	std::cout << "dynamic_cast b2_ptr:  " << dynamic_cast<void*>(b2_ptr) << "\n";

	auto b2_void = dynamic_cast<void*>(b2_ptr);

	std::cout << "dynamic_cast b2_ptr:  " << b2_void << "\n";


	auto b2_final = static_cast<B2*>(b2_void);
	std::cout << "dynamic_cast b2_ptr:  " << b2_final << "\n";



	aris::core::log(aris::core::LogData{std::chrono::system_clock::now(), __FILE__, 1000, aris::core::LogLvl::kError, -999, "AAAAAAAAAAAAAAAAAAAAAASSSSSSSDDDDDDDSSSSSSSDDDDDDDDDDDSSSSSSSSSSDDDDDDDDDD"});

	ARIS_LOG(aris::core::LogLvl::kError, -999, { "AAAAAAAAAAAAAAAAAAAAA\nAS\nSSSSSSDDDDDDDSSSSSSSDDDDDDDDDDDSSSSSSSSSSDDDDDDDDDD%s","sss:%s" },"bbb");
	ARIS_LOG(ERROR_123);


	std::cout << aris::core::logExeName() << std::endl;
}