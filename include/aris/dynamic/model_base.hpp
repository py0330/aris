#ifndef ARIS_DYNAMIC_MODEL_BASE_H_
#define ARIS_DYNAMIC_MODEL_BASE_H_

#include <aris/dynamic/kinematics.hpp>

namespace aris::dynamic{
	class ARIS_API ModelBase: public aris::core::NamedObject{
	public:
		// kinematic roots // 
		auto virtual inverseRootNumber()const->int { return 1; }
		auto virtual whichInverseRoot(const double* output, const double* input)->int { return 0; }
		auto virtual forwardRootNumber()const->int { return 1; }
		auto virtual whichForwardRoot(const double* input, const double* output)->int { return 0; }

		// kinematics & dynamics, not set state //
		auto virtual inverseKinematics(const double* output, double* input, int which_root = 0, const double *current_input = nullptr)const noexcept->int { return -1; }
		auto virtual forwardKinematics(const double* input, double* output, int which_root = 0, const double* current_input = nullptr)const noexcept->int { return -1; }

		// kinematics & dynamics, set state //
		auto virtual inverseKinematics()noexcept->int { return -1; }
		auto virtual forwardKinematics()noexcept->int { return -1; }
		auto virtual inverseKinematicsVel()noexcept->int { return -1; }
		auto virtual forwardKinematicsVel()noexcept->int { return -1; }
		auto virtual inverseKinematicsAcc()noexcept->int { return -1; }
		auto virtual forwardKinematicsAcc()noexcept->int { return -1; }
		auto virtual inverseDynamics()noexcept->int { return -1; }
		auto virtual forwardDynamics()noexcept->int { return -1; }

		// singular check //
		auto virtual isSingular(double zero_check = 1e-7)noexcept->bool { return false; }

		// EE & Motion types //
		auto virtual eeTypes()const noexcept->const EEType* { return nullptr; }
		auto virtual eeSize()const noexcept->aris::Size { return 0; }
		auto virtual motTypes()const noexcept->const EEType* { return nullptr; }
		auto virtual motSize()const noexcept->aris::Size { return 0; }

		// input variables //
		auto virtual inputPosSize()const noexcept->Size { return 0; }
		auto virtual getInputPos(double* pos)const noexcept->void;
		auto virtual setInputPos(const double* pos)noexcept->void;
		auto virtual inputPosAt(Size idx)const noexcept->double { return 0.0; }
		auto virtual setInputPosAt(Size idx, double pos)noexcept->void { }

		auto virtual inputVelSize()const noexcept->Size { return 0; }
		auto virtual getInputVel(double* vel)const noexcept->void;
		auto virtual setInputVel(const double* vel)noexcept->void;
		auto virtual inputVelAt(Size idx)const noexcept->double { return 0; }
		auto virtual setInputVelAt(Size idx, double vel)noexcept->void { }

		auto virtual inputAccSize()const noexcept->Size { return 0; }
		auto virtual getInputAcc(double* acc)const noexcept->void;
		auto virtual setInputAcc(const double* acc)noexcept->void;
		auto virtual inputAccAt(Size idx)const noexcept->double { return 0; }
		auto virtual setInputAccAt(Size idx, double acc)noexcept->void { }

		auto virtual inputFceSize()const noexcept->Size { return 0; }
		auto virtual getInputFce(double* fce)const noexcept->void;
		auto virtual setInputFce(const double* fce)noexcept->void;
		auto virtual inputFceAt(Size idx)const noexcept->double { return 0; }
		auto virtual setInputFceAt(Size idx, double fce)noexcept->void { }

		// output variables //
		// 相比于 input 变量，output 变量不提供对特定维数的访问，因为末端一般为多维末端，需整体访问
		auto virtual outputPosSize()const noexcept->Size { return 0; }
		auto virtual getOutputPos(double* pos)const noexcept->void;
		auto virtual setOutputPos(const double* pos)noexcept->void;
		auto virtual outputPosAt(Size idx)const noexcept->const double* { return nullptr; }
		auto virtual setOutputPosAt(Size idx, const double *pos)noexcept->void {}

		auto virtual outputVelSize()const noexcept->Size { return 0; }
		auto virtual getOutputVel(double* vel)const noexcept->void;
		auto virtual setOutputVel(const double* vel)noexcept->void;
		auto virtual outputVelAt(Size idx)const noexcept->const double* { return nullptr; }
		auto virtual setOutputVelAt(Size idx, const double* pos)noexcept->void {}

		auto virtual outputAccSize()const noexcept->Size { return 0; }
		auto virtual getOutputAcc(double* acc)const noexcept->void;
		auto virtual setOutputAcc(const double* acc)noexcept->void;
		auto virtual outputAccAt(Size idx)const noexcept->const double* { return nullptr; }
		auto virtual setOutputAccAt(Size idx, const double* pos)noexcept->void {}

		auto virtual outputFceSize()const noexcept->Size { return 0; }
		auto virtual getOutputFce(double* fce)const noexcept->void;
		auto virtual setOutputFce(const double* fce)noexcept->void;
		auto virtual outputFceAt(Size idx)const noexcept->const double* { return nullptr; }
		auto virtual setOutputFceAt(Size idx, const double* pos)noexcept->void {}

		auto virtual init()->void {};
	};
}

#endif
