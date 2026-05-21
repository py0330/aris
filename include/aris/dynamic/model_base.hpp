#ifndef ARIS_DYNAMIC_MODEL_BASE_H_
#define ARIS_DYNAMIC_MODEL_BASE_H_

#include <cstdint>

#include <aris/dynamic/kinematics.hpp>

namespace aris::dynamic{
	class ARIS_API ModelBase: public aris::core::NamedObject{
	public:
		// kinematic roots //

		/// @brief 单模型返回 1，多模型返回所有子模型的个数（包含子模型的子模型）
		/// @return 单模型： 1，多模型：子模型个数（含子模型的子模型）
		auto virtual inverseRootSize()const->int { return 1; }

		/// @brief 逆解个数，多模型为所有子模型的逆解个数之积
		/// @return 逆解个数，多模型返回：子模型的逆解个数之积
		auto virtual inverseRootNumber()const->std::int64_t { return 1; }
	
		/// @brief 根据当前的输入和输出，确认当前的逆解是哪一个
		/// @param output 输出位置
		/// @param input 输入位置
		/// @param which_root 逆解的编号，编号在 [0, inverseRootNumber()) 区间内
		/// @return 0 成功，-1 失败
		auto virtual getWhichInverseRoot(const double* output, const double* input, std::int64_t *which_root)->int { return 0; }

		/// @brief 单模型返回 1，多模型返回所有子模型的个数（包含子模型的子模型）
		/// @return 单模型： 1，多模型：子模型个数（含子模型的子模型）
		auto virtual forwardRootSize()const->int { return 1; }

		/// @brief 正解个数，多模型为所有子模型的正解个数之积
		/// @return 正解个数，多模型返回：子模型的正解个数之积
		auto virtual forwardRootNumber()const->std::int64_t { return 1; }

		/// @brief 根据当前的输入和输出，确认当前的正解是哪一个
		/// @param input 输入位置
		/// @param output 输出位置
		/// @param which_root 正解的编号，编号在 [0, forwardRootNumber()) 区间内
		/// @return 0 成功，-1 失败
		auto virtual getWhichForwardRoot(const double* input, const double* output, std::int64_t *which_root)->int { return 0; }
		
		/// @brief 求反解，不改变模型内部状态
		/// @param output 输出位置
		/// @param input 输入位置
		/// @param which_root 反解的编号，编号在 [0, inverseRootNumber()) 区间内
		/// @param current_input 输入位置的初值，某些情况可能会需要初值来进行计算，例如奇异时，或求解器基于迭代法时
		/// @return 成功返回值 >=0，失败返回值 < 0
		auto virtual inverseKinematics(const double* output, double* input, const std::int64_t *which_root = nullptr, const double *current_input = nullptr)const noexcept->int { return -1; }
		
		/// @brief 求正解，不改变模型内部状态
		/// @param input 输入位置
		/// @param output 输出位置
		/// @param which_root 正解的编号，编号在 [0, forwardRootNumber()) 区间内
		/// @param current_input 输入位置的初值，某些情况可能会需要初值来进行计算，例如奇异时，或求解器基于迭代法时
		/// @return 成功返回值 >=0，失败返回值 < 0
		auto virtual forwardKinematics(const double* input, double* output, const std::int64_t *which_root = nullptr, const double* current_input = nullptr)const noexcept->int { return -1; }

		/// @brief 设置模型内部求解时的逆解编号
		/// @param which_root 逆解编号
		auto virtual setWhichInverseRoot(const std::int64_t *which_root)->void { }

		/// @brief 设置模型内部求解时的正解编号
		/// @param which_root 正解编号
		auto virtual setWhichForwardRoot(const std::int64_t *which_root)->void { }


		/// @brief 基于当前模型状态求反解，解出的输入会被设置到模型内部状态中
		/// @return 成功返回值 >=0，失败返回值 < 0
		auto virtual inverseKinematics()noexcept->int { return -1; }

		/// @brief 基于当前模型状态求正解，解出的输出会被设置到模型内部状态中
		/// @return 成功返回值 >=0，失败返回值 < 0
		auto virtual forwardKinematics()noexcept->int { return -1; }

		/// @brief 基于当前模型状态求反解速度，解出的速度会被设置到模型内部状态中
		/// @return 成功返回值 >=0，失败返回值 < 0
		auto virtual inverseKinematicsVel()noexcept->int { return -1; }

		/// @brief 基于当前模型状态求正解速度，解出的速度会被设置到模型内部状态中
		/// @return 成功返回值 >=0，失败返回值 < 0
		auto virtual forwardKinematicsVel()noexcept->int { return -1; }

		/// @brief 基于当前模型状态求反解加速度，解出的加速度会被设置到模型内部状态中
		/// @return 成功返回值 >=0，失败返回值 < 0
		auto virtual inverseKinematicsAcc()noexcept->int { return -1; }

		/// @brief 基于当前模型状态求正解加速度，解出的加速度会被设置到模型内部状态中
		/// @return 成功返回值 >=0，失败返回值 < 0
		auto virtual forwardKinematicsAcc()noexcept->int { return -1; }

		/// @brief 基于当前模型状态求逆动力学，解出的力会被设置到模型内部状态中
		/// @return 成功返回值 >=0，失败返回值 < 0
		auto virtual inverseDynamics()noexcept->int { return -1; }

		/// @brief 基于当前模型状态求正动力学，解出的力会被设置到模型内部状态中
		/// @return 成功返回值 >=0，失败返回值 < 0
		auto virtual forwardDynamics()noexcept->int { return -1; }

		// singular check //
		auto virtual isSingular(double zero_check = 1e-7)noexcept->bool { return false; }

		// input limits //
		auto virtual minInputPos()const noexcept->const double * { return nullptr; }
		auto virtual maxInputPos()const noexcept->const double * { return nullptr; }
		auto virtual minInputVel()const noexcept->const double * { return nullptr; }
		auto virtual maxInputVel()const noexcept->const double * { return nullptr; }
		auto virtual minInputAcc()const noexcept->const double * { return nullptr; }
		auto virtual maxInputAcc()const noexcept->const double * { return nullptr; }

		// input variables //
		// num of motion //
		auto virtual inputSize()const noexcept->aris::Size { return 0; }
		auto virtual inputPosTypes()const noexcept->const PosType* { return nullptr; }
		auto virtual inputVelTypes()const noexcept->const VelType* { return nullptr; }
		auto virtual inputAccTypes()const noexcept->const AccType* { return nullptr; }
		auto virtual inputFceTypes()const noexcept->const FceType* { return nullptr; }

		auto virtual inputPosSize()const noexcept->Size;
		auto virtual inputPosMagSize()const noexcept->Size;
		auto virtual getInputPos(double* pos)const noexcept->void;
		auto virtual setInputPos(const double* pos)noexcept->void;
		auto virtual inputPosAt(Size idx)const noexcept->double { return 0.0; }
		auto virtual setInputPosAt(Size idx, double pos)noexcept->void { }

		auto virtual inputVelSize()const noexcept->Size;
		auto virtual getInputVel(double* vel)const noexcept->void;
		auto virtual setInputVel(const double* vel)noexcept->void;
		auto virtual inputVelAt(Size idx)const noexcept->double { return 0; }
		auto virtual setInputVelAt(Size idx, double vel)noexcept->void { }

		auto virtual inputAccSize()const noexcept->Size;
		auto virtual getInputAcc(double* acc)const noexcept->void;
		auto virtual setInputAcc(const double* acc)noexcept->void;
		auto virtual inputAccAt(Size idx)const noexcept->double { return 0; }
		auto virtual setInputAccAt(Size idx, double acc)noexcept->void { }

		auto virtual inputFceSize()const noexcept->Size;
		auto virtual getInputFce(double* fce)const noexcept->void;
		auto virtual setInputFce(const double* fce)noexcept->void;
		auto virtual inputFceAt(Size idx)const noexcept->double { return 0; }
		auto virtual setInputFceAt(Size idx, double fce)noexcept->void { }

		// output variables //
		// num of ee //
		auto virtual outputSize()const noexcept->aris::Size { return 0; }
		auto virtual outputPosTypes()const noexcept->const PosType* { return nullptr; }
		auto virtual outputVelTypes()const noexcept->const VelType* { return nullptr; }
		auto virtual outputAccTypes()const noexcept->const AccType* { return nullptr; }
		auto virtual outputFceTypes()const noexcept->const FceType* { return nullptr; }

		auto virtual outputPosSize()const noexcept->Size;
		// the mag size is norm num of output, e.g. for 6D pos xyzabs, the mag size is 2
		auto virtual outputPosMagSize()const noexcept->Size; 
		auto virtual getOutputPos(double* pos)const noexcept->void;
		auto virtual setOutputPos(const double* pos)noexcept->void;
		auto virtual outputPosAt(Size idx)const noexcept->const double* { return nullptr; }
		auto virtual setOutputPosAt(Size idx, const double *pos)noexcept->void {}

		auto virtual outputVelSize()const noexcept->Size;
		auto virtual getOutputVel(double* vel)const noexcept->void;
		auto virtual setOutputVel(const double* vel)noexcept->void;
		auto virtual outputVelAt(Size idx)const noexcept->const double* { return nullptr; }
		auto virtual setOutputVelAt(Size idx, const double* pos)noexcept->void {}

		auto virtual outputAccSize()const noexcept->Size;
		auto virtual getOutputAcc(double* acc)const noexcept->void;
		auto virtual setOutputAcc(const double* acc)noexcept->void;
		auto virtual outputAccAt(Size idx)const noexcept->const double* { return nullptr; }
		auto virtual setOutputAccAt(Size idx, const double* pos)noexcept->void {}

		auto virtual outputFceSize()const noexcept->Size;
		auto virtual getOutputFce(double* fce)const noexcept->void;
		auto virtual setOutputFce(const double* fce)noexcept->void;
		auto virtual outputFceAt(Size idx)const noexcept->const double* { return nullptr; }
		auto virtual setOutputFceAt(Size idx, const double* pos)noexcept->void {}

		auto virtual init()->void {};
	};
}

#endif
