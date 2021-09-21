#include "aris.hpp"

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
	//                    o
	//                     \
	//                      \
	//                       *------o      -> x
	//                      /
	//                     /
	//                    o
	//
	//


	aris::dynamic::DeltaParam param;
	param.a = 0.5;
	param.b = 0.2;
	param.c = 0.1;
	param.d = 0.7;
	param.e = 0.1;
	aris::dynamic::DeltaModel m1(param);





	// 设置末端 //
	double xyz_theta[4]{ -0.1, 0.1, -0.45, 0.3 };
	m1.setOutputPos(xyz_theta);
	
	// 反解，可能失败，失败意味着到达边缘 //
	if(m1.inverseKinematics())
		std::cout << "failed" << std::endl;

	// 获取电机位置 //
	double input[4];
	m1.getInputPos(input);
	
	// 打印电机位置 //
	aris::dynamic::dsp(1, 4, input);


	std::cout << "demo_model_delta finished, press any key to continue" << std::endl;
	std::cin.get();
	return 0;
}

