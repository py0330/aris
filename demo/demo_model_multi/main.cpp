/// \example demo_model_multi/main.cpp
/// 本例子展示基于双模型的建模:
///

#include "aris.hpp"

aris::plan::MultimodelPlanner mmp;

double input_pos[100];

class MoveL : public aris::core::CloneObject<MoveL, aris::plan::Plan>{
public:
	auto virtual prepareNrt()->void override;
	auto virtual executeRT()->int override;
	auto virtual collectNrt()->void override;

	virtual ~MoveL();
	explicit MoveL(const std::string& name = "MoveL");
	MoveL(const MoveL& other);

private:
	std::int64_t id_{ 0 };
};

auto MoveL::prepareNrt()->void{
	
	// p,v,a,j,z //
	auto pos_mtx = matrixParam("pos");
	auto vel_mtx = matrixParam("vel");
	auto acc_mtx = matrixParam("acc");
	auto jerk_mtx = matrixParam("jerk");
	auto zone_mtx = matrixParam("zone");

	// tool & wobj //
	auto tools = stringParam("tool");
	auto wobjs = stringParam("wobj");

	if (tools.size() > 0 && tools.front() == '{' && tools.back() == '}') {
		tools.erase(tools.begin());
		tools.erase(tools.end() - 1);
	}
	if (wobjs.size() > 0 && wobjs.front() == '{' && wobjs.back() == '}') {
		wobjs.erase(wobjs.begin());
		wobjs.erase(wobjs.end() - 1);
	}
		
	// insert line //
	id_ = mmp.insertLinePos(tools, wobjs, pos_mtx.data(), vel_mtx.data(), acc_mtx.data(), jerk_mtx.data(), zone_mtx.data());
	mmp.updateInsertPos();

	
}
auto MoveL::executeRT()->int{
	double p[100];
	auto ret = mmp.getNextInput(input_pos);
	return ret == id_ ? 0 : ret;
}
auto MoveL::collectNrt()->void{}
MoveL::~MoveL() = default;
MoveL::MoveL(const MoveL & other) = default;
MoveL::MoveL(const std::string & name) {
	aris::core::fromXmlString(command(),
		"<Command name=\"mvl\">"
		"	<GroupParam>"
		"		<Param name=\"pos\" default=\"{0,0,0,0,0,0,0}\"/>"
		"		<Param name=\"pos_type\" default=\"pe321\"/>"
		"		<Param name=\"acc\" default=\"3.0\"/>"
		"		<Param name=\"vel\" default=\"0.5\"/>"
		"		<Param name=\"dec\" default=\"3.0\"/>"
		"		<Param name=\"jerk\" default=\"5.0\"/>"
		"		<Param name=\"zone\" default=\"5.0\"/>"
		"		<Param name=\"tool\" default=\"0\"/>"
		"		<Param name=\"wobj\" default=\"0\"/>"
		"	</GroupParam>"
		"</Command>");
}

auto createMultiModel() -> std::unique_ptr<aris::dynamic::MultiModel> {
	// 真机数据 //
	double joints1[7]{ 149.982, -43.4585, -30.0078, -74.5814, 137.039, 49.1006, 55.3223 };
	double ee1[7]{ 0.203427, -0.420417, 0.263479, 117.546, 42.114, 222.485, -30.0078 * aris::PI / 180.0 };
	
	double joints2[7]{ 98.0283, -59.5074, -30.0078, -53.0331, 100.313, 68.8685, 2.34352 };
	double ee2[7]{ -0.202153, -0.476769, 0.346417, 95.5752, -19.3522, 279.021,-30.0078 * aris::PI / 180.0 };

	// 仿真时，人为给定初始状态 //
	for (int i = 0; i < 7; i++) {
		joints1[i] *= aris::PI / 180.0;
		joints2[i] *= aris::PI / 180.0;
	}
	for (int i = 0; i < 3; i++) {
		ee1[i + 3] *= aris::PI / 180.0;
		ee2[i + 3] *= aris::PI / 180.0;
	}

	// 构造模型 //
	aris::Size sub_num = 1;
	aris::Size sub_id[1]{ 1 };

	auto multi_model = std::make_unique<aris::dynamic::MultiModel>();
	aris::core::fromXmlFile(*multi_model, ARIS_INSTALL_PATH + std::string("/resource/test_plan/dual_arm.xml"));

	auto& sub0 = dynamic_cast<aris::dynamic::Model&>(multi_model->subModels()[0]);

	multi_model->init();

	multi_model->subModels()[1].setInputPos(joints1);
	multi_model->subForwardKinematics(sub_num, sub_id);

	return multi_model;
}

int main(){
	auto multi_model = createMultiModel();

	// 构造规划器 //
	mmp.setModel(*multi_model);
	mmp.setSubModelId({ 1 });

	// 最大速度、加速度 //
	std::vector<double> max_vels(7, 1.5);
	std::vector<double> min_vels(7, -1.5);
	std::vector<double> max_accs(7, 5.0);
	std::vector<double> min_accs(7, -5.0);

	mmp.setMaxVel(aris::core::Matrix(multi_model->inputPosSize(), 1, max_vels.data()));
	mmp.setMinVel(aris::core::Matrix(multi_model->inputPosSize(), 1, min_vels.data()));
	mmp.setMaxAcc(aris::core::Matrix(multi_model->inputPosSize(), 1, max_accs.data()));
	mmp.setMinAcc(aris::core::Matrix(multi_model->inputPosSize(), 1, min_accs.data()));

	mmp.setDt(1e-3);
	mmp.allocateMemory();

	mmp.init();
	
	// 构造mvl ，调试一下
	MoveL mvl;
	mvl.setModelBase(multi_model.get());

	mvl.command().init();
	mvl.parse("mvl --pos={-0.2021530000000000,-0.4767690000000000,0.3464170000000000,1.6681019232520844,-0.3377596075044466,4.8698351322070987,-0.5237349112799544} "
		"--vel={1000,1000,1000} --acc={100,100,100} --jerk={1000,1000,1000} --zone={0,0,0} "
		"--tool={RightArm.L7.tool0} --wobj={RightArm.ground.wobj0}");

	mvl.prepareNrt();
	mvl.setCount(1);


	while (auto ret = mvl.executeRT()) {
		mvl.setCount(mvl.count() + 1);

		std::cout << mvl.count() << "  " << ret << std::endl;
		if (mvl.count() < 100) {
			
			aris::dynamic::dsp(1, 7, input_pos);
		}
		

		if (mvl.count() == 5223)
			std::cout << "debug" << std::endl;

		if (ret < 0)
			break;
	}

	std::cout << "demo_model_multi finished, press any key to continue" << std::endl;
	std::cin.get();
	return 0;
}

