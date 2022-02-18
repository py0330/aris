/// \example demo_model_multi/main.cpp
/// 本例子展示基于双模型的建模:
///

#include "aris.hpp"

int main(){
	// 多模型 //
	aris::dynamic::MultiModel model;

	// 添加第一个机器人 //
	aris::dynamic::PumaParam param;
	param.d1 = 0.3295;
	param.a1 = 0.04;
	param.a2 = 0.275;
	param.d3 = 0.0;
	param.a3 = 0.025;
	param.d4 = 0.28;
	param.tool0_pe[2] = 0.078;
	model.subModels().push_back(aris::dynamic::createModelPuma(param).release());

	// 添加外部轴 //
	double pos[3]{ 1,-0.5,0 }, axis[3]{ 0,0,1 };
	model.subModels().push_back(aris::dynamic::createExternalAxisModel(pos, axis, false).release());

	// 添加 tools 和 wobjs，以下信息可以与xml进行反射
	model.tools().push_back(model.findMarker("PumaModel.EE.tool0"));
	model.tools().push_back(model.findMarker("PumaModel.EE.tool1"));

	model.wobjs().push_back(model.findMarker("ExAxisModel.EE.tool1"));
	model.wobjs().push_back(model.findMarker("ExAxisModel.ground.wobj1"));

	aris::core::toXmlString(model);

	auto v = model.findVariable("PumaModel.tool0_axis_home");
	
	auto ee_types = model.getEeTypes();
	// 求正解 //
	// 或者分别设置两个模型的输入
	//double robot_input_pos[6]{ 0.1,0.2,0.3,0.4,0.5,0.6 };
	//model.subModels()[0].setInputPos(robot_input_pos);
	//double ex_axis_input_pos[1]{ 0.5 };
	//model.subModels()[1].setInputPos(ex_axis_input_pos);
	
	// 七个数，对应两个模型的所有电机
	double input_pos[7]{ 0.1,0.2,0.3,0.4,0.5,0.6 ,0.5 };
	model.setInputPos(input_pos);

	// 正解所有子模型
	model.forwardKinematics();

	// 获得末端位姿 //
	double tool_pe[6];
	model.tools()[0]->getPe(*model.wobjs()[0], tool_pe, "321");

	// 打印 //
	aris::dynamic::dsp(1, 6, tool_pe);

	// 求反解 //
	// 因为两个模型相互耦合，所以首先设置外部轴并进行反解，否则外部轴转台的位姿不对，也就是wobj的位姿不对
	double ex_axis_output_pos[1]{0.4};
	model.subModels()[1].setOutputPos(ex_axis_output_pos);
	model.subModels()[1].inverseKinematics();

	// 整体求解
	double tool_wrt_wobj[6]{ 0.2,0.08,0.07,0.0,0.0,0.0 };
	model.tools()[0]->setPe(*model.wobjs()[0], tool_wrt_wobj, "321");

	model.updP();

	if (model.inverseKinematics())
		std::cout << "failed" << std::endl;

	// 打印所有电机输入
	model.getInputPos(input_pos);
	aris::dynamic::dsp(1, 7, input_pos);

	// 保存 xml
	try {
		std::cout << aris::core::toXmlString(model) << std::endl;
	}
	catch (std::exception& e){
		std::cout << e.what() << std::endl;
	}
	
	
	
	

	std::cout << "demo_model_multi finished, press any key to continue" << std::endl;
	std::cin.get();
	return 0;
}

