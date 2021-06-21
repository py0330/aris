#ifndef ARIS_DYNAMIC_MODEL_BASE_H_
#define ARIS_DYNAMIC_MODEL_BASE_H_

namespace aris::dynamic
{
	class ARIS_API EndEffectorBase{
	public:
		auto virtual dim()const noexcept->Size { return 0; };
		auto virtual pSize()const noexcept->Size { return dim(); }
		auto virtual p()const noexcept->const double* { return nullptr; }
		auto virtual setP(const double *mp) noexcept->void {}
		auto virtual v()const noexcept->const double* { return nullptr; }
		auto virtual setV(const double *mp) noexcept->void {}
		auto virtual a()const noexcept->const double* { return nullptr; }
		auto virtual setA(const double *mp) noexcept->void {}
	};

	class ARIS_API ModelBase{
	public:
		// kinematics & dynamics //
		auto virtual inverseKinematics()noexcept->int { return -1; }
		auto virtual forwardKinematics()noexcept->int { return -1; }
		auto virtual inverseKinematicsVel()noexcept->int { return -1; }
		auto virtual forwardKinematicsVel()noexcept->int { return -1; }
		auto virtual inverseDynamics()noexcept->int { return -1; }
		auto virtual forwardDynamics()noexcept->int { return -1; }

		// inputs //
		auto virtual inputPosSize()const noexcept->Size { return 0; }
		auto virtual inputDim()const noexcept->Size { return 0; }
		auto virtual getInputPos(double *mp)const noexcept->void { }
		auto virtual setInputPos(const double *mp)noexcept->void { }
		auto virtual getInputVel(double *mv)const noexcept->void { }
		auto virtual setInputVel(const double *mv)noexcept->void { }
		auto virtual getInputAcc(double *ma)const noexcept->void { }
		auto virtual setInputAcc(const double *ma)noexcept->void { }
		auto virtual getInputFce(double *mf)const noexcept->void { }
		auto virtual setInputFce(const double *mf)noexcept->void { }

		// outputs //
		auto virtual outputPosSize()const noexcept->Size { return 0; }
		auto virtual outputDim()const noexcept->Size { return 0; }
		auto virtual getOutputPos(double *mp)const noexcept->void { }
		auto virtual setOutputPos(const double *mp)noexcept->void { }
		auto virtual getOutputVel(double *mv)const noexcept->void { }
		auto virtual setOutputVel(const double *mv)noexcept->void { }
		auto virtual getOutputAcc(double *ma)const noexcept->void { }
		auto virtual setOutputAcc(const double *ma)noexcept->void { }
		auto virtual getOutputFce(double *mf)const noexcept->void { }
		auto virtual setOutputFce(const double *mf)noexcept->void { }
	};
}

#endif
