#include "aris.hpp"

auto createModelPlanarScaraLansi()->std::unique_ptr<aris::dynamic::Model>
{
	aris::dynamic::ScaraParam param;
	param.a = 0.6;
	param.b = 0.85;
	auto m = aris::dynamic::createModelPlanarScara(param);
	return std::move(m);
}

int main(){
	
	// 构造 delta 的模型 //
	//---------------------------------------------------------------------------------------------
	// 包含4个自由度。每个支联4个杆件  link1~4
	// 尺寸示意：
	//                   上平台 
	//                      base(长度a，平台中心到关节的距离)
	//                    *-------o      -> R1
	//                             \     -> link1 (长度b)
	//                              \o   -> S2
	//                              /|   -> link1 (长度c)
	//                             / o   -> S3
	//                            / /
	//               S4 <-       o /     -> link2&3 (长度d)
	//           转轴Re <-  |----|/         Re 连接并联末端 up 与最终的末端 ee，从而变成 4 自由度
	//               S5 <-       o
	//                       ee(长度e)
	//---------------------------------------------------------------------------------------------
	// 零位：
	//                 (0,0,0)           o
	//                    |-------o------|       ->  x方向
	//                                 / o
	//                                / /
	//                               / /
	//                              o /
	//                    |---------|/
	//                              o
	//---------------------------------------------------------------------------------------------
	// 俯视图：
	//                       y
	//                       ^
	//                       |
	//             支链2  o
	//                     \
	//                      \
	//                       *------o      -> x
	//                      /      支链1
	//                     /
	//                    o
	//              支链3
	//


	aris::dynamic::DeltaFullParam param;
	param.a1 = 0.53;
	param.b1 = 0.21;
	param.c1 = 0.1;
	param.d1 = 0.67;
	param.e1 = 0.08;
	param.theta1 = 0.1;

	param.a2 = 0.49;
	param.b2 = 0.205;
	param.c2 = 0.11;
	param.d2 = 0.71;
	param.e2 = 0.1;
	param.theta2 = aris::PI * 2 / 3 - 0.01;

	param.a3 = 0.5;
	param.b3 = 0.2;
	param.c3 = 0.12;
	param.d3 = 0.73;
	param.e3 = 0.09;
	param.theta3 = -aris::PI * 2 / 3 +0.05;
	auto m1 = aris::dynamic::createModelDelta(param);

	//////////////////// 反解 ////////////////////
	double xyz_theta[4]{ -0.1, 0.1, -0.45, 0.3 };
	m1->setOutputPos(xyz_theta);
	
	// 反解，可能失败，失败意味着到达边缘 //
	if(m1->inverseKinematics())
		std::cout << "failed" << std::endl;

	// 获取电机位置 //
	double input[4];
	m1->getInputPos(input);
	
	// 打印电机位置 //
	aris::dynamic::dsp(1, 4, input);

	//////////////////// 正解 ////////////////////
	double tem[4]{ 0.0000101,0,0,0 };
	m1->setInputPos(tem);
	if (m1->forwardKinematics())
		std::cout << "failed" << std::endl;

	//0.07494404685677   0.00605077149338   -0.32691906113662   0.00000000000000

	// 以前的方法仍然可以使用，即用 generalMotionPool()[0]  来访问
	double output[4];
	m1->getOutputPos(output);
	aris::dynamic::dsp(1, 4, output);

	m1->setInputPos(input);
	if (m1->forwardKinematics())
		std::cout << "failed" << std::endl;
	m1->getOutputPos(output);
	aris::dynamic::dsp(1, 4, output);


	try {
		for (auto &p : m1->partPool())
		{
			p.geometryPool().clear();

		}
		aris::core::toXmlFile(*m1, "C:\\Users\\py033\\Desktop\\a.xml");
	}
	catch (std::exception &e) {
		std::cout << e.what() << std::endl;
	}


	////////////////////////////////////////// scara 机器人 ////////////////////////////////////
	//---------------------------------------------------------------------------------------------
	// 包含4个自由度。共4个杆件  link1~4
	// 零位尺寸示意：
	//
	//                           o
	//                           |        y
	//                           |  b     ^
	//                           |        |
	//           o---------------o        *---> x
	//        origin         a
	//---------------------------------------------------------------------------------------------

	aris::dynamic::ScaraParam param2;
	param2.a = 1;
	param2.b = 1;
	auto m2 = aris::dynamic::createModelScara(param2);


	

	//////////////////// 反解 ////////////////////
	double xyz_theta2[4]{ -0.8, -0.9, -0.45, 0.3 };
	m2->setOutputPos(xyz_theta2);
	if (m2->inverseKinematics())
		std::cout << "failed" << std::endl;
	m2->getInputPos(input);
	aris::dynamic::dsp(1, 4, input);

	//////////////////// 正解 ////////////////////
	m2->setInputPos(input);
	if (m2->forwardKinematics())
		std::cout << "failed" << std::endl;
	m2->getOutputPos(output);
	aris::dynamic::dsp(1, 4, output);

	auto tool1 = m2->generalMotionPool()[0].makI()->fatherPart().findMarker("tool1");
	double test_clb_pe[6]{ 0.1,0.2,0.3,0.5,0.4,0.6 };
	tool1->setPrtPe(test_clb_pe);

	double points[8]{ 0.5,0.4,0.3,0.6,0.4,0.9 };
	calibModelByTwoPoints(*m2, points, "tool1");
	double pe[6];
	aris::dynamic::s_pm2pe(*tool1->prtPm(), pe);
	aris::dynamic::dsp(1, 6, pe);

	////////////////////////////////////////// planar delta ////////////////////////////////////
	aris::dynamic::PlanarDeltaFullParam planar_delta_param;
	planar_delta_param.a1 = 0.175;
	planar_delta_param.b1 = 0.550;
	planar_delta_param.c1 = 1.160;
	planar_delta_param.a2 = -0.175;
	planar_delta_param.b2 = -0.550;
	planar_delta_param.c2 = -1.160;
	planar_delta_param.d = 0.118;
	auto planar_delta = aris::dynamic::createModelPlanarDelta(planar_delta_param);
	
	double input_000[3]{ 0.3,0.3,0.2 };
	planar_delta->setInputPos(input_000);
	planar_delta->forwardKinematics();

	planar_delta->getOutputPos(pe);
	aris::dynamic::dsp(1, 3, pe);

	planar_delta->inverseKinematics();
	planar_delta->getInputPos(input);
	aris::dynamic::dsp(1, 3, input);
	//planar_delta->setOutputPos();


	////////////////////////////////////////// planar scara ////////////////////////////////////
	{
		aris::dynamic::ScaraParam param2;
		param2.a = 1;
		param2.b = 1;
		//auto m2 = aris::dynamic::createModelPlanarScara(param2);
		auto m2 = createModelPlanarScaraLansi();
		
		//////////////////// 反解 ////////////////////
		double xyz_theta2[4]{ 0.61, 0.85, 0.3 };
		m2->setOutputPos(xyz_theta2);
		if (m2->inverseKinematics())
			std::cout << "failed" << std::endl;
		m2->getInputPos(input);
		aris::dynamic::dsp(1, 3, input);

		//////////////////// 正解 ////////////////////
		m2->setInputPos(input);
		if (m2->forwardKinematics())
			std::cout << "failed" << std::endl;
		m2->getOutputPos(output);
		aris::dynamic::dsp(1, 3, output);
	}

	////////////////////////////////////////// rppr ////////////////////////////////////
	{
		aris::dynamic::RpprParam param2;
		param2.a = 1;
		param2.b = 1;
		auto m2 = createModelRppr(param2);

		//////////////////// 反解 ////////////////////
		double xyz_theta2[4]{ 0.61, 0.85, 0.3, 0.1 };
		m2->setOutputPos(xyz_theta2);
		if (m2->inverseKinematics())
			std::cout << "failed" << std::endl;
		m2->getInputPos(input);
		aris::dynamic::dsp(1, 4, input);

		//////////////////// 正解 ////////////////////
		m2->setInputPos(input);
		if (m2->forwardKinematics())
			std::cout << "failed" << std::endl;
		m2->getOutputPos(output);
		aris::dynamic::dsp(1, 4, output);
	}

	std::cout << "demo_model_delta finished, press any key to continue" << std::endl;
	std::cin.get();
	return 0;
}

