#ifndef ARIS_DYNAMIC_MODEL_BASE_H_
#define ARIS_DYNAMIC_MODEL_BASE_H_

namespace aris::dynamic{
	class ARIS_API ModelBase: public aris::core::NamedObject{
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

#define ARIS_INPUT_DATA_DEFINATION(TYPE, VARIABLE)                                             \
		auto virtual input##TYPE##Size()const noexcept->Size { return 0; }                     \
		auto virtual getInput##TYPE(double* VARIABLE)const noexcept->void {                    \
			for (int i = 0; i < input##TYPE##Size(); ++i) VARIABLE[i] = input##TYPE##At(i);    \
		}                                                                                      \
		auto virtual setInput##TYPE(const double* VARIABLE)noexcept->void {                    \
			for (int i = 0; i < input##TYPE##Size(); ++i) setInput##TYPE##At(VARIABLE[i], i);  \
		}                                                                                      \
		auto virtual input##TYPE##At(Size idx)const noexcept->double { return 0; }             \
		auto virtual setInput##TYPE##At(double VARIABLE, Size idx)noexcept->void { }           \


		ARIS_INPUT_DATA_DEFINATION(Pos, mp)
		ARIS_INPUT_DATA_DEFINATION(Vel, mv)
		ARIS_INPUT_DATA_DEFINATION(Acc, ma)
		ARIS_INPUT_DATA_DEFINATION(Fce, mf)
#undef ARIS_INPUT_DATA_DEFINATION

		// 相比于 input 变量，output 变量不提供对特定维数的访问，因为末端一般为多维末端，需整体访问
#define ARIS_OUTPUT_DATA_DEFINATION(TYPE, VARIABLE)                                \
		auto virtual output##TYPE##Size()const noexcept->Size { return 0; }        \
		auto virtual getOutput##TYPE(double* VARIABLE)const noexcept->void {       \
		}                                                                          \
		auto virtual setOutput##TYPE(const double* VARIABLE)noexcept->void {       \
		}                                                                          \


		ARIS_OUTPUT_DATA_DEFINATION(Pos, mp)
		ARIS_OUTPUT_DATA_DEFINATION(Vel, mv)
		ARIS_OUTPUT_DATA_DEFINATION(Acc, ma)
		ARIS_OUTPUT_DATA_DEFINATION(Fce, mf)
#undef ARIS_OUTPUT_DATA_DEFINATION




		auto virtual init()->void {};
	};
}

#endif
