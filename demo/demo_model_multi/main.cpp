/// \example demo_model_scara/main.cpp
/// 本例子展示基于双模型的建模:
///

#include "aris.hpp"

// 多轴模型，覆盖了ModelBase类的虚方法 //
// 建议把这个直接放入 kaanh 层//
// 本质是在将所有子模型的对应方法全部调用了一遍
class MultiModel :public aris::dynamic::ModelBase {
public:
	// kinematics & dynamics //
	auto virtual inverseKinematics()noexcept->int { 
		for (auto &model : subModels()) 
			if(auto ret = model.inverseKinematics())
				return ret; 
		return 0; 
	}
	auto virtual forwardKinematics()noexcept->int {
		for (auto &model : subModels())
			if (auto ret = model.forwardKinematics())
				return ret;
		return 0;
	}
	auto virtual inverseKinematicsVel()noexcept->int { 
		for (auto &model : subModels())
			if (auto ret = model.inverseKinematicsVel())
				return ret;
		return 0;
	}
	auto virtual forwardKinematicsVel()noexcept->int {
		for (auto &model : subModels())
			if (auto ret = model.forwardKinematicsVel())
				return ret;
		return 0;
	}
	auto virtual inverseDynamics()noexcept->int {
		for (auto &model : subModels())
			if (auto ret = model.inverseDynamics())
				return ret;
		return 0;
	}
	auto virtual forwardDynamics()noexcept->int {
		for (auto &model : subModels())
			if (auto ret = model.forwardDynamics())
				return ret;
		return 0;
	}

	// inputs //
	auto virtual inputPosSize()const noexcept->aris::Size { 
		aris::Size size = 0;
		for (auto &m : subModels())size += m.inputPosSize();
		return size;
	}
	auto virtual getInputPos(double *mp)const noexcept->void {
		for (aris::Size pos = 0, idx = 0; idx < models_.size(); pos += models_[idx].inputPosSize(), ++idx)
			models_[idx].getInputPos(mp + pos);
	}
	auto virtual setInputPos(const double *mp)noexcept->void {
		for (aris::Size pos = 0, idx = 0; idx < models_.size(); pos += models_[idx].inputPosSize(), ++idx)
			models_[idx].setInputPos(mp + pos);
	}

	auto virtual inputVelSize()const noexcept->aris::Size {
		aris::Size size = 0;
		for (auto &m : subModels())size += m.inputVelSize();
		return size;
	}
	auto virtual getInputVel(double *mv)const noexcept->void {
		for (aris::Size pos = 0, idx = 0; idx < models_.size(); pos += models_[idx].inputVelSize(), ++idx)
			models_[idx].getInputVel(mv + pos);
	}
	auto virtual setInputVel(const double *mv)noexcept->void {
		for (aris::Size pos = 0, idx = 0; idx < models_.size(); pos += models_[idx].inputVelSize(), ++idx)
			models_[idx].setInputVel(mv + pos);
	}

	auto virtual inputAccSize()const noexcept->aris::Size {
		aris::Size size = 0;
		for (auto &m : subModels())size += m.inputAccSize();
		return size;
	}
	auto virtual getInputAcc(double *ma)const noexcept->void {
		for (aris::Size pos = 0, idx = 0; idx < models_.size(); pos += models_[idx].inputAccSize(), ++idx)
			models_[idx].getInputAcc(ma + pos);
	}
	auto virtual setInputAcc(const double *ma)noexcept->void {
		for (aris::Size pos = 0, idx = 0; idx < models_.size(); pos += models_[idx].inputAccSize(), ++idx)
			models_[idx].setInputAcc(ma + pos);
	}

	auto virtual inputFceSize()const noexcept->aris::Size {
		aris::Size size = 0;
		for (auto &m : subModels())size += m.inputFceSize();
		return size;
	}
	auto virtual getInputFce(double *mf)const noexcept->void {
		for (aris::Size pos = 0, idx = 0; idx < models_.size(); pos += models_[idx].inputFceSize(), ++idx)
			models_[idx].getInputFce(mf + pos);
	}
	auto virtual setInputFce(const double *mf)noexcept->void {
		for (aris::Size pos = 0, idx = 0; idx < models_.size(); pos += models_[idx].inputFceSize(), ++idx)
			models_[idx].setInputFce(mf + pos);
	}

	// outputs //
	auto virtual outputPosSize()const noexcept->aris::Size {
		aris::Size size = 0;
		for (auto &m : subModels())size += m.outputPosSize();
		return size;
	}
	auto virtual getOutputPos(double *mp)const noexcept->void {
		for (aris::Size pos = 0, idx = 0; idx < models_.size(); pos += models_[idx].outputPosSize(), ++idx)
			models_[idx].getOutputPos(mp + pos);
	}
	auto virtual setOutputPos(const double *mp)noexcept->void {
		for (aris::Size pos = 0, idx = 0; idx < models_.size(); pos += models_[idx].outputPosSize(), ++idx)
			models_[idx].setOutputPos(mp + pos);
	}

	auto virtual outputVelSize()const noexcept->aris::Size {
		aris::Size size = 0;
		for (auto &m : subModels())size += m.outputVelSize();
		return size;
	}
	auto virtual getOutputVel(double *mv)const noexcept->void {
		for (aris::Size pos = 0, idx = 0; idx < models_.size(); pos += models_[idx].outputVelSize(), ++idx)
			models_[idx].getOutputVel(mv + pos);
	}
	auto virtual setOutputVel(const double *mv)noexcept->void {
		for (aris::Size pos = 0, idx = 0; idx < models_.size(); pos += models_[idx].outputVelSize(), ++idx)
			models_[idx].setOutputVel(mv + pos);
	}

	auto virtual outputAccSize()const noexcept->aris::Size {
		aris::Size size = 0;
		for (auto &m : subModels())size += m.outputAccSize();
		return size;
	}
	auto virtual getOutputAcc(double *ma)const noexcept->void {
		for (aris::Size pos = 0, idx = 0; idx < models_.size(); pos += models_[idx].outputAccSize(), ++idx)
			models_[idx].getOutputAcc(ma + pos);
	}
	auto virtual setOutputAcc(const double *ma)noexcept->void {
		for (aris::Size pos = 0, idx = 0; idx < models_.size(); pos += models_[idx].outputAccSize(), ++idx)
			models_[idx].setOutputAcc(ma + pos);
	}

	auto virtual outputFceSize()const noexcept->aris::Size {
		aris::Size size = 0;
		for (auto &m : subModels())size += m.outputFceSize();
		return size;
	}
	auto virtual getOutputFce(double *mf)const noexcept->void {
		for (aris::Size pos = 0, idx = 0; idx < models_.size(); pos += models_[idx].outputFceSize(), ++idx)
			models_[idx].getOutputFce(mf + pos);
	}
	auto virtual setOutputFce(const double *mf)noexcept->void {
		for (aris::Size pos = 0, idx = 0; idx < models_.size(); pos += models_[idx].outputFceSize(), ++idx)
			models_[idx].setOutputFce(mf + pos);
	}

	auto subModels()->aris::core::PointerArray<ModelBase>& { return models_; }
	auto subModels()const->const aris::core::PointerArray<ModelBase>& { return models_; }
private:
	aris::core::PointerArray<ModelBase> models_;
};

// 实现一个外部轴模型,这里只实现位置即可 //
class ExternalAxisModel :public aris::dynamic::ModelBase {
public:
	// 反解 //
	auto virtual inverseKinematics()noexcept->int {
		double pe[6]{ 0,0,0,0,0,0 };
		target_->getPe(*reference_, pe, "123");
		mp_ = pe[5];
		return 0;
	}
	// 正解 //
	auto virtual forwardKinematics()noexcept->int {
		double pe[6]{ 0,0,0,0,0,mp_ };
		double pm[16];
		aris::dynamic::s_pe2pm(pe, pm, "123");

		double target_pm[16];
		aris::dynamic::s_pm_dot_pm(*reference_->pm(), pm, target_pm);
		target_->setPrtPm(target_pm);
		return 0;
	}

	// 输入 //
	auto virtual inputPosSize()const noexcept->aris::Size {	return 1; }
	auto virtual getInputPos(double *mp)const noexcept->void { *mp = mp_; }
	auto virtual setInputPos(const double *mp)noexcept->void { mp_ = *mp; }
	// 输出 //
	auto virtual outputPosSize()const noexcept->aris::Size { return 1; }
	auto virtual getOutputPos(double *mp)const noexcept->void {
		double pe[6]{ 0,0,0,0,0,0 };
		target_->getPe(*reference_, pe, "321");
		*mp = pe[0];
	}
	auto virtual setOutputPos(const double *mp)noexcept->void {
		double pe[6]{ 0,0,0,0,0,*mp };
		double pm[16];
		aris::dynamic::s_pe2pm(pe, pm, "123");
		
		double target_pm[16];
		aris::dynamic::s_pm_dot_pm(*reference_->pm(), pm, target_pm);
		target_->setPrtPm(target_pm);
	}

	ExternalAxisModel(aris::dynamic::Marker *target_marker, aris::dynamic::Marker *reference_marker) :target_(target_marker), reference_(reference_marker) {};
private:
	aris::dynamic::Marker *target_, *reference_;
	double mp_;

};


int main()
{
	// 多模型 //
	MultiModel model;

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
	// 对第一个model做类型转化，从 ModelBase 转为 Model 
	auto &m = dynamic_cast<aris::dynamic::Model&>(model.subModels()[0]);
	model.subModels().push_back(new ExternalAxisModel(m.findPart("ground")->findMarker("wobj1"), m.findPart("ground")->findMarker("wobj0")));


	// 计算反解：方式1，统一计算 //
	// 前16维是pm，最后1个是外部轴 //
	double ee[17];
	aris::dynamic::s_pe2pm(std::array<double, 6>{0.3, 0.5, 0.4, 0.4, 0.1, 0}.data(), ee); // 机器人末端，现在是pm，16维
	ee[16] = 0.5;                                                                         // 外部轴

	// 设置
	model.setOutputPos(ee);

	// 反解
	if (model.inverseKinematics()) std::cout << "inverse failed" <<std::endl;

	// 取出
	double input[7];
	model.getInputPos(input);
	aris::dynamic::dsp(1, 7, input);


	// 计算反解：方式2，分开计算 //
	// 设置6轴机械臂 //
	double ee1[16];
	aris::dynamic::s_pe2pm(std::array<double, 6>{0.3, 0.5, 0.4, 0.4, 0.1, 0}.data(), ee1); // 机器人末端，现在是pm，16维
	model.subModels()[0].setOutputPos(ee1);
	
	// 设置外部轴 //
	double ee2[1]{ 0.3 };
	model.subModels()[1].setOutputPos(ee2);

	// 反解
	if (model.inverseKinematics()) std::cout << "inverse failed" << std::endl;

	// 取出7个电机 //
	model.getInputPos(input);
	aris::dynamic::dsp(1, 7, input);



	std::cout << "demo_model_multi finished, press any key to continue" << std::endl;
	std::cin.get();
	return 0;
}

