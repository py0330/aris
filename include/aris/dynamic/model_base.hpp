#ifndef ARIS_DYNAMIC_MODEL_BASE_H_
#define ARIS_DYNAMIC_MODEL_BASE_H_

#include <cstdint>

#include <aris/dynamic/kinematics.hpp>

namespace aris::dynamic{
	/// @brief 机器人/机构模型的抽象基类。
	///
	/// @details
	/// ModelBase 统一定义求解接口与状态读写接口，
	/// 派生类（如 Model、MultiModel）负责具体实现。
	///
	/// 接口可分为以下模块：
	///
	/// 1. 求解模块（运动学/动力学）
	///    - 1.1 根管理：用于多解系统（逆解/正解编号）
	///      关键函数：`inverseRootSize`、`inverseRootNumber`、`getWhichInverseRoot`、
	///      `forwardRootSize`、`forwardRootNumber`、`getWhichForwardRoot`。
	///    - 1.2 运动学：位置/速度/加速度求解
	///      关键函数：`inverseKinematics`、`forwardKinematics`、
	///      `inverseKinematicsVel`、`forwardKinematicsVel`、
	///      `inverseKinematicsAcc`、`forwardKinematicsAcc`。
	///    - 1.3 动力学：逆动力学/正动力学
	///      关键函数：`inverseDynamics`、`forwardDynamics`。
	///
	/// 2. 输入输出接口模块（状态访问）
	///    - 2.1 驱动空间电机限制（位置/速度/加速度）
	///      关键函数：`minInputPos/maxInputPos`、`minInputVel/maxInputVel`、
	///      `minInputAcc/maxInputAcc`。
	///    - 2.2 输入状态接口（驱动空间）
	///      类型与尺寸：`inputSize`、`inputPosTypes/VelTypes/AccTypes/FceTypes`、
	///      `inputPosSize/VelSize/AccSize/FceSize`。
	///      批量读写：`getInput*` / `setInput*`。
	///      单元素读写：`input*At` / `setInput*At`。
	///    - 2.3 输出侧（任务空间/末端空间）
	///      类型与尺寸：`outputSize`、`outputPosTypes/VelTypes/AccTypes/FceTypes`、
	///      `outputPosSize/VelSize/AccSize/FceSize`。
	///      批量读写：`getOutput*` / `setOutput*`。
	///      单元素读写：`output*At` / `setOutput*At`。
	///
	/// 3. 诊断与生命周期
	///    - 奇异性检测：`isSingular`。
	///    - 初始化：`init`。
	///
	/// 使用建议：
	/// - 带参数的 inverseKinematics/forwardKinematics 为“无状态”接口，不修改内部状态。
	/// - 无参数的 inverseKinematics/forwardKinematics/... 为“有状态”接口，会回写内部状态。
	/// - 多解场景建议先固定根，或先识别根，避免分支跳变。
	/// - 求速度/加速度/动力学前，先保证位置状态一致。
	class ARIS_API ModelBase: public aris::core::NamedObject{
	public:
		// ---------------------------------------------------------------------
		// 1. 求解模块
		// 1.1 根管理与根识别（多解系统）
		// ---------------------------------------------------------------------

		/// @brief which_root 指针的 size。
		/// @return 单模型返回 1；多模型返回全部子模型数量（含嵌套）。
		auto virtual inverseRootSize()const->int { return 1; }

		/// @brief 逆解总个数。
		/// @return 单模型为自身逆解数；多模型为各子模型逆解数乘积。
		auto virtual inverseRootNumber()const->std::int64_t { return 1; }
	
		/// @brief 根据当前输入/输出判定逆解编号。
		/// @param output 输出位置
		/// @param input 输入位置
		/// @param which_root 逆解编号，范围为 [0, inverseRootNumber())
		/// @return 0 成功，-1 失败
		auto virtual getWhichInverseRoot(const double* output, const double* input, std::int64_t *which_root)const->int { return 0; }

		/// @brief 设置模型内部求解时的逆解编号
		/// @param which_root 逆解编号
		auto virtual setWhichInverseRoot(const std::int64_t *which_root)->void { }

		/// @brief which_root 指针的 size。
		/// @return 单模型返回 1；多模型返回全部子模型数量（含嵌套）。
		auto virtual forwardRootSize()const->int { return 1; }

		/// @brief 正解总个数。
		/// @return 单模型为自身正解数；多模型为各子模型正解数乘积。
		auto virtual forwardRootNumber()const->std::int64_t { return 1; }

		/// @brief 根据当前输入/输出判定正解编号。
		/// @param input 输入位置
		/// @param output 输出位置
		/// @param which_root 正解编号，范围为 [0, forwardRootNumber())
		/// @return 0 成功，-1 失败
		auto virtual getWhichForwardRoot(const double* input, const double* output, std::int64_t *which_root)const->int { return 0; }
		
		/// @brief 设置模型内部求解时的正解编号
		/// @param which_root 正解编号
		auto virtual setWhichForwardRoot(const std::int64_t *which_root)->void { }

		/// @brief 无状态逆运动学求解。
		/// @param output 输出位置
		/// @param input 输入位置
		/// @param which_root 逆解编号，范围为 [0, inverseRootNumber())
		/// @param current_input 输入初值，用于奇异或迭代求解场景
		/// @return 成功返回值 >=0，失败返回值 < 0
		auto virtual inverseKinematics(const double* output, double* input, const std::int64_t *which_root = nullptr, const double *current_input = nullptr)const noexcept->int { return -1; }
		
		/// @brief 无状态正运动学求解。
		/// @param input 输入位置
		/// @param output 输出位置
		/// @param which_root 正解编号，范围为 [0, forwardRootNumber())
		/// @param current_input 输入初值，用于奇异或迭代求解场景
		/// @return 成功返回值 >=0，失败返回值 < 0
		auto virtual forwardKinematics(const double* input, double* output, const std::int64_t *which_root = nullptr, const double* current_input = nullptr)const noexcept->int { return -1; }

		/// @brief 无状态逆速度求解。
		/// @details 基于当前 output 相关状态，计算输入速度，不修改模型内部状态。
		/// @param output 输出速度
		/// @param input 输入速度
		/// @return 成功返回值 >=0，失败返回值 < 0
		auto virtual inverseKinematicsVel(const double* output, double* input)const noexcept->int { return -1; }

		/// @brief 无状态正速度求解。
		/// @details 基于当前 input 相关状态，计算输出速度，不修改模型内部状态。
		/// @param input 输入速度
		/// @param output 输出速度
		/// @return 成功返回值 >=0，失败返回值 < 0
		auto virtual forwardKinematicsVel(const double* input, double* output)const noexcept->int { return -1; }

		/// @brief 无状态逆加速度求解。
		/// @details 基于当前 output 相关状态，计算输入加速度，不修改模型内部状态。
		/// @param output 输出加速度
		/// @param input 输入加速度
		/// @return 成功返回值 >=0，失败返回值 < 0
		auto virtual inverseKinematicsAcc(const double* output, double* input)const noexcept->int { return -1; }

		/// @brief 无状态正加速度求解。
		/// @details 基于当前 input 相关状态，计算输出加速度，不修改模型内部状态。
		/// @param input 输入加速度
		/// @param output 输出加速度
		/// @return 成功返回值 >=0，失败返回值 < 0
		auto virtual forwardKinematicsAcc(const double* input, double* output)const noexcept->int { return -1; }

		/// @brief 无状态逆动力学求解。
		/// @details 基于当前状态和输入加速度，计算输入力，不修改模型内部状态。
		/// @param input_a 输入加速度
		/// @param input_f 输入力
		/// @return 成功返回值 >=0，失败返回值 < 0
		auto virtual inverseDynamics(const double* input_a, double* input_f)const noexcept->int { return -1; }

		/// @brief 无状态正动力学求解。
		/// @details 基于当前状态和输入力，计算输入加速度，不修改模型内部状态。
		/// @param input_f 输入力
		/// @param input_a 输入加速度
		/// @return 成功返回值 >=0，失败返回值 < 0
		auto virtual forwardDynamics(const double* input_f, double* input_a)const noexcept->int { return -1; }

		// ---------------------------------------------------------------------
		// 1.2 运动学/1.3 动力学：有状态求解接口（读写模型内部状态）
		// ---------------------------------------------------------------------

		/// @brief 有状态逆运动学求解（结果回写内部输入状态）。
		/// @return 成功返回值 >=0，失败返回值 < 0
		auto virtual inverseKinematics()noexcept->int { return -1; }

		/// @brief 有状态正运动学求解（结果回写内部输出状态）。
		/// @return 成功返回值 >=0，失败返回值 < 0
		auto virtual forwardKinematics()noexcept->int { return -1; }

		/// @brief 有状态逆速度求解（结果回写内部输入速度）。
		/// @return 成功返回值 >=0，失败返回值 < 0
		auto virtual inverseKinematicsVel()noexcept->int { return -1; }

		/// @brief 有状态正速度求解（结果回写内部输出速度）。
		/// @return 成功返回值 >=0，失败返回值 < 0
		auto virtual forwardKinematicsVel()noexcept->int { return -1; }

		/// @brief 有状态逆加速度求解（结果回写内部输入加速度）。
		/// @return 成功返回值 >=0，失败返回值 < 0
		auto virtual inverseKinematicsAcc()noexcept->int { return -1; }

		/// @brief 有状态正加速度求解（结果回写内部输出加速度）。
		/// @return 成功返回值 >=0，失败返回值 < 0
		auto virtual forwardKinematicsAcc()noexcept->int { return -1; }

		/// @brief 有状态逆动力学求解（结果回写内部输入力）。
		/// @return 成功返回值 >=0，失败返回值 < 0
		auto virtual inverseDynamics()noexcept->int { return -1; }

		/// @brief 有状态正动力学求解（结果回写内部输出力）。
		/// @return 成功返回值 >=0，失败返回值 < 0
		auto virtual forwardDynamics()noexcept->int { return -1; }

		// singular check //
		// ---------------------------------------------------------------------
		// 3. 诊断接口
		// ---------------------------------------------------------------------
		/// @brief 判断模型在当前状态是否处于奇异位形。
		/// @param zero_check 数值零阈值，越小越严格。
		/// @return `true` 表示奇异，`false` 表示非奇异。
		auto virtual isSingular(double zero_check = 1e-7)noexcept->bool { return false; }

		// input limits //
		// ---------------------------------------------------------------------
		// 2. 输入输出接口
		// 2.1 驱动空间电机限制（位置/速度/加速度）
		// ---------------------------------------------------------------------
		/// @brief 获取输入位置下限数组。
		/// @return 指向下限数组首元素的指针，未实现时可为 `nullptr`。
		auto virtual minInputPos()const noexcept->const double * { return nullptr; }
		/// @brief 获取输入位置上限数组。
		/// @return 指向上限数组首元素的指针，未实现时可为 `nullptr`。
		auto virtual maxInputPos()const noexcept->const double * { return nullptr; }

		// ---------------------------------------------------------------------
		// 2.2 输入侧状态接口（驱动空间）
		// ---------------------------------------------------------------------
		/// @brief 获取输入速度下限数组。
		/// @return 指向下限数组首元素的指针，未实现时可为 `nullptr`。
		auto virtual minInputVel()const noexcept->const double * { return nullptr; }
		/// @brief 获取输入速度上限数组。
		/// @return 指向上限数组首元素的指针，未实现时可为 `nullptr`。
		auto virtual maxInputVel()const noexcept->const double * { return nullptr; }
		/// @brief 获取输入加速度下限数组。
		/// @return 指向下限数组首元素的指针，未实现时可为 `nullptr`。
		auto virtual minInputAcc()const noexcept->const double * { return nullptr; }
		/// @brief 获取输入加速度上限数组。
		/// @return 指向上限数组首元素的指针，未实现时可为 `nullptr`。
		auto virtual maxInputAcc()const noexcept->const double * { return nullptr; }

		// input variables //
		// num of motion //
		/// @brief 输入变量逻辑数量（通常对应关节/驱动数）。
		/// @return 输入变量个数。
		auto virtual inputSize()const noexcept->aris::Size { return 0; }
		/// @brief 获取输入位置类型数组。
		/// @return 输入位置类型数组指针。
		auto virtual inputPosTypes()const noexcept->const PosType* { return nullptr; }
		/// @brief 获取输入速度类型数组。
		/// @return 输入速度类型数组指针。
		auto virtual inputVelTypes()const noexcept->const VelType* { return nullptr; }
		/// @brief 获取输入加速度类型数组。
		/// @return 输入加速度类型数组指针。
		auto virtual inputAccTypes()const noexcept->const AccType* { return nullptr; }
		/// @brief 获取输入力/力矩类型数组。
		/// @return 输入力类型数组指针。
		auto virtual inputFceTypes()const noexcept->const FceType* { return nullptr; }

		/// @brief 输入位置分量总维度。
		/// @return 输入位置展开后的标量维度。
		auto virtual inputPosSize()const noexcept->Size;
		/// @brief 输入位置模量维度。
		/// @return 输入位置按模量统计后的维度。
		auto virtual inputPosMagSize()const noexcept->Size;
		/// @brief 批量读取输入位置。
		/// @param pos 输出缓冲区，长度至少为 `inputPosSize()`。
		auto virtual getInputPos(double* pos)const noexcept->void;
		/// @brief 批量写入输入位置。
		/// @param pos 输入数组，长度至少为 `inputPosSize()`。
		auto virtual setInputPos(const double* pos)noexcept->void;
		/// @brief 按下标读取单个输入位置分量。
		/// @param idx 分量下标。
		/// @return 对应位置分量值。
		auto virtual inputPosAt(Size idx)const noexcept->double { return 0.0; }
		/// @brief 按下标写入单个输入位置分量。
		/// @param idx 分量下标。
		/// @param pos 分量值。
		auto virtual setInputPosAt(Size idx, double pos)noexcept->void { }

		/// @brief 输入速度分量总维度。
		/// @return 输入速度展开后的标量维度。
		auto virtual inputVelSize()const noexcept->Size;
		/// @brief 批量读取输入速度。
		/// @param vel 输出缓冲区，长度至少为 `inputVelSize()`。
		auto virtual getInputVel(double* vel)const noexcept->void;
		/// @brief 批量写入输入速度。
		/// @param vel 输入数组，长度至少为 `inputVelSize()`。
		auto virtual setInputVel(const double* vel)noexcept->void;
		/// @brief 按下标读取单个输入速度分量。
		/// @param idx 分量下标。
		/// @return 对应速度分量值。
		auto virtual inputVelAt(Size idx)const noexcept->double { return 0; }
		/// @brief 按下标写入单个输入速度分量。
		/// @param idx 分量下标。
		/// @param vel 分量值。
		auto virtual setInputVelAt(Size idx, double vel)noexcept->void { }

		/// @brief 输入加速度分量总维度。
		/// @return 输入加速度展开后的标量维度。
		auto virtual inputAccSize()const noexcept->Size;
		/// @brief 批量读取输入加速度。
		/// @param acc 输出缓冲区，长度至少为 `inputAccSize()`。
		auto virtual getInputAcc(double* acc)const noexcept->void;
		/// @brief 批量写入输入加速度。
		/// @param acc 输入数组，长度至少为 `inputAccSize()`。
		auto virtual setInputAcc(const double* acc)noexcept->void;
		/// @brief 按下标读取单个输入加速度分量。
		/// @param idx 分量下标。
		/// @return 对应加速度分量值。
		auto virtual inputAccAt(Size idx)const noexcept->double { return 0; }
		/// @brief 按下标写入单个输入加速度分量。
		/// @param idx 分量下标。
		/// @param acc 分量值。
		auto virtual setInputAccAt(Size idx, double acc)noexcept->void { }

		/// @brief 输入力/力矩分量总维度。
		/// @return 输入力展开后的标量维度。
		auto virtual inputFceSize()const noexcept->Size;
		/// @brief 批量读取输入力/力矩。
		/// @param fce 输出缓冲区，长度至少为 `inputFceSize()`。
		auto virtual getInputFce(double* fce)const noexcept->void;
		/// @brief 批量写入输入力/力矩。
		/// @param fce 输入数组，长度至少为 `inputFceSize()`。
		auto virtual setInputFce(const double* fce)noexcept->void;
		/// @brief 按下标读取单个输入力分量。
		/// @param idx 分量下标。
		/// @return 对应力分量值。
		auto virtual inputFceAt(Size idx)const noexcept->double { return 0; }
		/// @brief 按下标写入单个输入力分量。
		/// @param idx 分量下标。
		/// @param fce 分量值。
		auto virtual setInputFceAt(Size idx, double fce)noexcept->void { }

		// ---------------------------------------------------------------------
		// 2.3 输出侧（任务空间/末端空间）
		// ---------------------------------------------------------------------
		/// @brief 输出变量逻辑数量（通常对应末端/任务对象数量）。
		/// @return 输出变量个数。
		auto virtual outputSize()const noexcept->aris::Size { return 0; }
		/// @brief 获取输出位置类型数组。
		/// @return 输出位置类型数组指针。
		auto virtual outputPosTypes()const noexcept->const PosType* { return nullptr; }
		/// @brief 获取输出速度类型数组。
		/// @return 输出速度类型数组指针。
		auto virtual outputVelTypes()const noexcept->const VelType* { return nullptr; }
		/// @brief 获取输出加速度类型数组。
		/// @return 输出加速度类型数组指针。
		auto virtual outputAccTypes()const noexcept->const AccType* { return nullptr; }
		/// @brief 获取输出力/力矩类型数组。
		/// @return 输出力类型数组指针。
		auto virtual outputFceTypes()const noexcept->const FceType* { return nullptr; }

		/// @brief 输出位置分量总维度。
		/// @return 输出位置展开后的标量维度。
		auto virtual outputPosSize()const noexcept->Size;
		/// @brief 输出位置模量维度。
		/// @details 例如 6D 位姿 xyz+abc，其模量维度可视为 2。
		/// @return 输出位置按模量统计后的维度。
		auto virtual outputPosMagSize()const noexcept->Size; 
		/// @brief 批量读取输出位置。
		/// @param pos 输出缓冲区，长度至少为 `outputPosSize()`。
		auto virtual getOutputPos(double* pos)const noexcept->void;
		/// @brief 批量写入输出位置。
		/// @param pos 输入数组，长度至少为 `outputPosSize()`。
		auto virtual setOutputPos(const double* pos)noexcept->void;
		/// @brief 按下标读取单个输出位置对象。
		/// @param idx 对象下标。
		/// @return 指向该输出对象数据的指针。
		auto virtual outputPosAt(Size idx)const noexcept->const double* { return nullptr; }
		/// @brief 按下标写入单个输出位置对象。
		/// @param idx 对象下标。
		/// @param pos 指向该输出对象数据的指针。
		auto virtual setOutputPosAt(Size idx, const double *pos)noexcept->void {}

		/// @brief 输出速度分量总维度。
		/// @return 输出速度展开后的标量维度。
		auto virtual outputVelSize()const noexcept->Size;
		/// @brief 批量读取输出速度。
		/// @param vel 输出缓冲区，长度至少为 `outputVelSize()`。
		auto virtual getOutputVel(double* vel)const noexcept->void;
		/// @brief 批量写入输出速度。
		/// @param vel 输入数组，长度至少为 `outputVelSize()`。
		auto virtual setOutputVel(const double* vel)noexcept->void;
		/// @brief 按下标读取单个输出速度对象。
		/// @param idx 对象下标。
		/// @return 指向该输出对象数据的指针。
		auto virtual outputVelAt(Size idx)const noexcept->const double* { return nullptr; }
		/// @brief 按下标写入单个输出速度对象。
		/// @param idx 对象下标。
		/// @param pos 指向该输出对象数据的指针。
		auto virtual setOutputVelAt(Size idx, const double* pos)noexcept->void {}

		/// @brief 输出加速度分量总维度。
		/// @return 输出加速度展开后的标量维度。
		auto virtual outputAccSize()const noexcept->Size;
		/// @brief 批量读取输出加速度。
		/// @param acc 输出缓冲区，长度至少为 `outputAccSize()`。
		auto virtual getOutputAcc(double* acc)const noexcept->void;
		/// @brief 批量写入输出加速度。
		/// @param acc 输入数组，长度至少为 `outputAccSize()`。
		auto virtual setOutputAcc(const double* acc)noexcept->void;
		/// @brief 按下标读取单个输出加速度对象。
		/// @param idx 对象下标。
		/// @return 指向该输出对象数据的指针。
		auto virtual outputAccAt(Size idx)const noexcept->const double* { return nullptr; }
		/// @brief 按下标写入单个输出加速度对象。
		/// @param idx 对象下标。
		/// @param pos 指向该输出对象数据的指针。
		auto virtual setOutputAccAt(Size idx, const double* pos)noexcept->void {}

		/// @brief 输出力/力矩分量总维度。
		/// @return 输出力展开后的标量维度。
		auto virtual outputFceSize()const noexcept->Size;
		/// @brief 批量读取输出力/力矩。
		/// @param fce 输出缓冲区，长度至少为 `outputFceSize()`。
		auto virtual getOutputFce(double* fce)const noexcept->void;
		/// @brief 批量写入输出力/力矩。
		/// @param fce 输入数组，长度至少为 `outputFceSize()`。
		auto virtual setOutputFce(const double* fce)noexcept->void;
		/// @brief 按下标读取单个输出力对象。
		/// @param idx 对象下标。
		/// @return 指向该输出对象数据的指针。
		auto virtual outputFceAt(Size idx)const noexcept->const double* { return nullptr; }
		/// @brief 按下标写入单个输出力对象。
		/// @param idx 对象下标。
		/// @param pos 指向该输出对象数据的指针。
		auto virtual setOutputFceAt(Size idx, const double* pos)noexcept->void {}


		// ---------------------------------------------------------------------
		// 3. 生命周期
		// ---------------------------------------------------------------------
		/// @brief 初始化模型内部资源与状态。
		/// @details 派生类可在此完成缓存分配、尺寸检查、参数预计算等操作。
		auto virtual init()->void {};
	};
}

#endif
