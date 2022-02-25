/// \example demo_model_multi/main.cpp
/// 本例子展示基于双模型的建模:
///

#include "aris.hpp"

struct MoveLParam{
	// 根据前端得到的参数，以下vec的size应该一样
	std::vector<aris::dynamic::MotionBase*> ees;
	std::vector<aris::dynamic::EEType> ee_types;
	std::vector<aris::dynamic::Marker*> ee_tools;
	std::vector<aris::dynamic::Marker*> ee_wobjs;
	std::vector<aris::dynamic::ModelBase*> submodels;
	std::vector<aris::Size> sub_ee_num; // 每个 submodel 的 ee 个数

	// 根据指令和机器人状态得到的目标位置，size是末端的总和
	std::vector<double> begin_output_pos;
	std::vector<double> end_output_pos;
	std::vector<double> vels;
	std::vector<double> accs;

	// 根据规划器算出来的
	aris::Size total_count;
	std::vector<double> time_ratio;
};

class MoveL : public aris::core::CloneObject<MoveL, aris::plan::Plan>
{
public:
	auto virtual prepareNrt()->void override;
	auto virtual executeRT()->int override;
	auto virtual collectNrt()->void override;

	virtual ~MoveL();
	explicit MoveL(const std::string& name = "MoveL");
	MoveL(const MoveL& other);
};

auto MoveL::prepareNrt()->void{
	
	MoveLParam param;

	auto model = dynamic_cast<aris::dynamic::MultiModel*>(modelBase());
	// 根据解析获得以下参数：
	param.ees = model->getEes();
	param.ee_types = model->getEeTypes();
	param.sub_ee_num = { 1,1 };// 子模型里末端的个数，其实就是 submodel().generalMotionPool().size()
	param.submodels = { &model->subModels()[0], &model->subModels()[1] };
	param.ee_tools = { nullptr, model->tools()[0] }; // 外部轴没有tool，所以是nullptr
	param.ee_wobjs = { nullptr, model->wobjs()[1] }; // 外部轴没有wobj，所以是nullptr
	param.vels = { 0.1, 0.5 * aris::PI, 0.3 }; // 外部轴速度只有一个，机器人有角速度和线速度，两维
	param.accs = { 0.1, 0.8 * aris::PI, 0.6 }; // 同上

	aris::Size pos_idx{ 0 }, vel_idx{ 0 }, acc_idx{ 0 }, ee_idx{0};
	std::vector<double> ee_periods; // 临时变量，param里面只需要存总的时间和time_ratio 即可
	for (int i = 0; i < param.submodels.size(); ++i) {
		// 提取子模型的数据
		auto sub_ee_types = param.ee_types.data() + ee_idx;

		// 对子模型做规划
		for (int j = 0; j < param.sub_ee_num[i]; ++j) {
			if (sub_ee_types[j] == aris::dynamic::EEType::A || sub_ee_types[j] == aris::dynamic::EEType::X) {
				// 起始位置
				param.begin_output_pos.push_back(0.1);

				// 结束位置
				param.end_output_pos.push_back(0.6);

				// 速度
				double vel = param.vels[vel_idx];

				// 加速度
				double acc = param.accs[acc_idx];

				// 处理id
				pos_idx++;
				vel_idx++;
				acc_idx++;

				//！！！！！！ 这里调用规划器 ！！！！！//
				// 对单轴做规划，得到period，例如调用sCurve函数
				ee_periods.push_back(5.0124);
			}
			else if (sub_ee_types[j] == aris::dynamic::EEType::PE123
				|| sub_ee_types[j] == aris::dynamic::EEType::PE321) {// 还可以有更多条件

				// 起始位置
				double begin_pe[6]{ 0,1,2,3,4,5 };
				param.begin_output_pos.insert(param.begin_output_pos.end(), begin_pe, begin_pe + 6);

				// 结束位置
				double end_pe[6]{ 0,1,2,3,4,5 };
				param.end_output_pos.insert(param.end_output_pos.end(), begin_pe, begin_pe + 6);

				// 速度
				double angular_vel = param.vels[vel_idx];
				double linear_vel = param.vels[vel_idx];

				// 加速度
				double angular_acc = param.accs[acc_idx];
				double linear_acc = param.accs[acc_idx];

				// 处理id
				pos_idx += 6;
				vel_idx += 2;
				acc_idx += 2;

				//！！！！！！ 这里调用规划器 ！！！！！//
				// 对六维坐标系做规划，得到period，例如调用sCurve函数
				ee_periods.push_back(5.0124);
			}
			else {
				// 对四轴等其他类型的做规划

			}
		
		}


		
		
	}

	//  统一处理所有的period等，得到time ratios和真实的total_count
	param.total_count = 1000;
	param.time_ratio = { 0.1,1 };

	this->param() = param;
}
auto MoveL::executeRT()->int{
	auto& param = std::any_cast<MoveLParam&>(this->param());
	
	aris::Size pos_idx{ 0 }, vel_idx{ 0 }, acc_idx{ 0 }, ee_idx{ 0 }, time_ratio_idx{0};
	for (int i = 0; i < param.submodels.size(); ++i) {
		// 提取子模型的数据
		auto sub_ee_types = param.ee_types.data() + ee_idx;

		// 对子模型做规划
		for (int j = 0; j < param.sub_ee_num[i]; ++j) {
			auto sub_begin_pos = param.begin_output_pos.data() + pos_idx;
			auto sub_end_pos = param.begin_output_pos.data() + pos_idx;
			auto sub_vel = param.vels.data() + vel_idx;
			auto sub_acc = param.accs.data() + acc_idx;
			auto time_ratio = param.time_ratio[i]; // 时间缩放比例

			if (sub_ee_types[j] == aris::dynamic::EEType::A || sub_ee_types[j] == aris::dynamic::EEType::X) {
				// 处理id
				pos_idx++;
				vel_idx++;
				acc_idx++;
				ee_idx++;
				time_ratio_idx++;

				//！！！！！！ 这里调用规划器 ！！！！！//
				// 基于sub_begin_pos sub_end_pos sub_vel sub_acc
				// 对单轴做规划，得到period，例如调用sCurve函数
				double current_pos[1]{0};

				// 做反解，需判断是否用tool 和 wobj设置
				if (param.ee_tools[i] && param.ee_wobjs[i]) {
					double pe[6]{ current_pos[0],0,0,0,0,0};
					param.ee_tools[i]->setPe(*param.ee_wobjs[i], pe);
					param.ees[i]->updP();
					param.submodels[i]->inverseKinematics();
				}
				else {
					param.submodels[i]->setOutputPos(current_pos);
					param.submodels[i]->inverseKinematics();
				}
			}
			else if (sub_ee_types[j] == aris::dynamic::EEType::PE123
				|| sub_ee_types[j] == aris::dynamic::EEType::PE321) {// 还可以有更多条件

				// 处理id
				pos_idx += 6;
				vel_idx += 2;
				acc_idx += 2;
				ee_idx++;
				time_ratio_idx++;
				

				//！！！！！！ 这里调用规划器 ！！！！！//
				// 对机器人做规划，例如调用sCurve函数
				double current_pe[6]{ 0,1,2,3,5,6 };
				//！！！！！！ 规划器调用完毕 ！！！！！//

				// 做反解，需判断是否用tool 和 wobj设置
				if (param.ee_tools[i] && param.ee_wobjs[i]) {
					param.ee_tools[i]->setPe(*param.ee_wobjs[i], current_pe);
					param.ees[i]->updP();
					param.submodels[i]->inverseKinematics();
				}
				else {
					param.submodels[i]->setOutputPos(current_pe);
					param.submodels[i]->inverseKinematics();
				}
			}
			else {
				// 对四轴等其他类型的做规划

			}

		}




	}

	return param.total_count - count();
}
auto MoveL::collectNrt()->void{}
MoveL::~MoveL() = default;
MoveL::MoveL(const MoveL & other) = default;
MoveL::MoveL(const std::string & name) {
	aris::core::fromXmlString(command(),
		"<Command name=\"mvl\">"
		"	<GroupParam>"
		"		<Param name=\"pos_unit\" default=\"mm\"/>"
		"		<Param name=\"pos_offset\" default=\"Offset{0,0,0,0,0,0}\"/>"
		"		<UniqueParam default=\"pq\">"
		"			<Param name=\"pq\" default=\"{0,0,0,0,0,0,1}\"/>"
		"			<Param name=\"pm\" default=\"{1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1}\"/>"
		"			<GroupParam>"
		"				<Param name=\"pe\" default=\"RobotTarget{0,0,0,0,0,0}\"/>"
		"               <Param name=\"epos\" default=\"ExAxisTarget{0.0,0.0,0.0,0.0,0.0,0.0}\"/>"
		"               <Param name=\"evel\" default=\"ExAxisVel{0.0,0.0,0.0,0.0,0.0,0.0}\"/>"
		"				<Param name=\"ori_unit\" default=\"degree\"/>"
		"				<Param name=\"eul_type\" default=\"321\"/>"
		"			</GroupParam>"
		"		</UniqueParam>"
		"		<Param name=\"acc\" default=\"3.0\"/>"
		"		<Param name=\"vel\" default=\"Speed{2.5,25,180}\"/>"
		"		<Param name=\"dec\" default=\"3.0\"/>"
		"		<Param name=\"jerk\" default=\"5.0\"/>"
		"		<Param name=\"angular_acc\" default=\"3\"/>"
		"		<Param name=\"angular_vel\" default=\"3\"/>"
		"		<Param name=\"angular_dec\" default=\"3\"/>"
		"		<Param name=\"angular_jerk\" default=\"5.0\"/>"
		"		<Param name=\"zone\" default=\"Zone{0.0,0.0}\"/>"
		"		<Param name=\"fc\" default=\"0\"/>"
		"		<Param name=\"wobj_is_binding\" default=\"0\"/>"
		"		<Param name=\"wobj_motor_id\" default=\"6\"/>"
		"		<Param name=\"wobj\" default=\"0\"/>"
		"	</GroupParam>"
		"</Command>");
}



int main(){
	// 多模型 //
	aris::dynamic::MultiModel model;

	// 添加外部轴 //
	double pos[3]{ 1,-0.5,0 }, axis[3]{ 0,0,1 };
	model.subModels().push_back(aris::dynamic::createExternalAxisModel(pos, axis, false).release());

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



	// 添加 tools 和 wobjs，以下信息可以与xml进行反射
	model.tools().push_back(model.findMarker("PumaModel.EE.tool0"));
	model.tools().push_back(model.findMarker("PumaModel.EE.tool1"));

	model.wobjs().push_back(model.findMarker("ExAxisModel.EE.tool1"));
	model.wobjs().push_back(model.findMarker("ExAxisModel.ground.wobj1"));
	

	// 构造mvl ，调试一下
	MoveL mvl;
	mvl.setModelBase(&model);


	mvl.prepareNrt();
	mvl.setCount(1);
	while (mvl.executeRT()) {
		mvl.setCount(mvl.count() + 1);
	
	}

	std::cout << "demo_model_multi finished, press any key to continue" << std::endl;
	std::cin.get();
	return 0;
}

