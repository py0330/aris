#ifndef ARIS_DYNAMIC_MODEL_BASE_H_
#define ARIS_DYNAMIC_MODEL_BASE_H_

namespace aris::dynamic{
	class ARIS_API ModelBase{
	public:
		// kinematics & dynamics //
		auto virtual inverseKinematics()noexcept->int { return -1; }
		auto virtual forwardKinematics()noexcept->int { return -1; }
		auto virtual inverseKinematicsVel()noexcept->int { return -1; }
		auto virtual forwardKinematicsVel()noexcept->int { return -1; }
		auto virtual inverseKinematicsAcc()noexcept->int { return -1; }
		auto virtual forwardKinematicsAcc()noexcept->int { return -1; }
		auto virtual inverseDynamics()noexcept->int { return -1; }
		auto virtual forwardDynamics()noexcept->int { return -1; }

		// inputs //
		auto virtual inputPosSize()const noexcept->Size { return 0; }
		auto virtual getInputPos(double *mp)const noexcept->void { }
		auto virtual setInputPos(const double *mp)noexcept->void { }

		auto virtual inputVelSize()const noexcept->Size { return 0; }
		auto virtual getInputVel(double *mv)const noexcept->void { }
		auto virtual setInputVel(const double *mv)noexcept->void { }

		auto virtual inputAccSize()const noexcept->Size { return 0; }
		auto virtual getInputAcc(double *ma)const noexcept->void { }
		auto virtual setInputAcc(const double *ma)noexcept->void { }

		auto virtual inputFceSize()const noexcept->Size { return 0; }
		auto virtual getInputFce(double *mf)const noexcept->void { }
		auto virtual setInputFce(const double *mf)noexcept->void { }

		// outputs //
		auto virtual outputPosSize()const noexcept->Size { return 0; }
		auto virtual getOutputPos(double *mp)const noexcept->void { }
		auto virtual setOutputPos(const double *mp)noexcept->void { }

		auto virtual outputVelSize()const noexcept->Size { return 0; }
		auto virtual getOutputVel(double *mv)const noexcept->void { }
		auto virtual setOutputVel(const double *mv)noexcept->void { }

		auto virtual outputAccSize()const noexcept->Size { return 0; }
		auto virtual getOutputAcc(double *ma)const noexcept->void { }
		auto virtual setOutputAcc(const double *ma)noexcept->void { }

		auto virtual outputFceSize()const noexcept->Size { return 0; }
		auto virtual getOutputFce(double *mf)const noexcept->void { }
		auto virtual setOutputFce(const double *mf)noexcept->void { }


	};
}

#endif
