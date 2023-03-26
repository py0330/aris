#ifndef ARIS_DYNAMIC_MODEL_BASE_H_
#define ARIS_DYNAMIC_MODEL_BASE_H_

namespace aris::dynamic{
	class ARIS_API ModelBase: public aris::core::NamedObject{
	public:
		// kinematics & dynamics, set state //
		auto virtual inverseKinematics()noexcept->int { return -1; }
		auto virtual forwardKinematics()noexcept->int { return -1; }
		auto virtual inverseKinematicsVel()noexcept->int { return -1; }
		auto virtual forwardKinematicsVel()noexcept->int { return -1; }
		auto virtual inverseKinematicsAcc()noexcept->int { return -1; }
		auto virtual forwardKinematicsAcc()noexcept->int { return -1; }
		auto virtual inverseDynamics()noexcept->int { return -1; }
		auto virtual forwardDynamics()noexcept->int { return -1; }

		// kinematics, not set state //
		auto virtual inverseKinematics(const double* output, double* input, int which_root = 0)const noexcept->int { return -1; }
		auto virtual forwardKinematics(const double* input, double* output, int which_root = 0)const noexcept->int { return -1; }

		// input variables //
		auto virtual inputPosSize()const noexcept->Size { return 0; }
		auto virtual getInputPos(double* pos)const noexcept->void {
			for (int i = 0; i < inputPosSize(); ++i) pos[i] = inputPosAt(i);
		}
		auto virtual setInputPos(const double* pos)noexcept->void {
			for (int i = 0; i < inputPosSize(); ++i) setInputPosAt(pos[i], i);
		}
		auto virtual inputPosAt(Size idx)const noexcept->double { return 0; }
		auto virtual setInputPosAt(double pos, Size idx)noexcept->void { }

		auto virtual inputVelSize()const noexcept->Size { return 0; }
		auto virtual getInputVel(double* vel)const noexcept->void {
			for (int i = 0; i < inputVelSize(); ++i) vel[i] = inputVelAt(i);
		}
		auto virtual setInputVel(const double* vel)noexcept->void {
			for (int i = 0; i < inputVelSize(); ++i) setInputVelAt(vel[i], i);
		}
		auto virtual inputVelAt(Size idx)const noexcept->double { return 0; }
		auto virtual setInputVelAt(double vel, Size idx)noexcept->void { }

		auto virtual inputAccSize()const noexcept->Size { return 0; }
		auto virtual getInputAcc(double* acc)const noexcept->void {
			for (int i = 0; i < inputAccSize(); ++i) acc[i] = inputAccAt(i);
		}
		auto virtual setInputAcc(const double* acc)noexcept->void {
			for (int i = 0; i < inputAccSize(); ++i) setInputAccAt(acc[i], i);
		}
		auto virtual inputAccAt(Size idx)const noexcept->double { return 0; }
		auto virtual setInputAccAt(double acc, Size idx)noexcept->void { }

		auto virtual inputFceSize()const noexcept->Size { return 0; }
		auto virtual getInputFce(double* fce)const noexcept->void {
			for (int i = 0; i < inputFceSize(); ++i) fce[i] = inputFceAt(i);
		}
		auto virtual setInputFce(const double* fce)noexcept->void {
			for (int i = 0; i < inputFceSize(); ++i) setInputFceAt(fce[i], i);
		}
		auto virtual inputFceAt(Size idx)const noexcept->double { return 0; }
		auto virtual setInputFceAt(double fce, Size idx)noexcept->void { }

		// output variables //
		// 相比于 input 变量，output 变量不提供对特定维数的访问，因为末端一般为多维末端，需整体访问
		auto virtual outputPosSize()const noexcept->Size { return 0; }
		auto virtual getOutputPos(double* pos)const noexcept->void {}
		auto virtual setOutputPos(const double* pos)noexcept->void {}

		auto virtual outputVelSize()const noexcept->Size { return 0; }
		auto virtual getOutputVel(double* vel)const noexcept->void {}
		auto virtual setOutputVel(const double* vel)noexcept->void {}

		auto virtual outputAccSize()const noexcept->Size { return 0; }
		auto virtual getOutputAcc(double* acc)const noexcept->void {}
		auto virtual setOutputAcc(const double* acc)noexcept->void {}

		auto virtual outputFceSize()const noexcept->Size { return 0; }
		auto virtual getOutputFce(double* fce)const noexcept->void {}
		auto virtual setOutputFce(const double* fce)noexcept->void {}

		auto virtual init()->void {};
	};
}

#endif
