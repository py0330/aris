#ifndef ARIS_PLAN_SCURVE_H_
#define ARIS_PLAN_SCURVE_H_

#include <list>
#include <cmath>
#include <iostream>
#include <functional>
#include <map>
#include <any>

#include <aris/dynamic/dynamic.hpp>

/// \brief 轨迹规划命名空间
/// \ingroup aris
/// 
///
///
namespace aris::plan{
	struct ARIS_API LargeNum {
		std::int64_t count_;
		double value_;

		LargeNum() {
			count_ = 0;
			value_ = 0.0;
		};
		LargeNum(std::int64_t count, double value) {
			count_ = count;
			value_ = value;
		};
		LargeNum(double value) {
			double r = std::fmod(value, 1000.0);
			count_ = std::lround((value - r) / 1000.0);
			value_ = r;
		};

		operator double()const { return count_ * 1000.0 + value_; }

		friend auto operator+(LargeNum left, LargeNum right)->LargeNum {
			double v = left.value_ + right.value_;
			double r = std::fmod(v, 1000.0);
			return LargeNum{
				left.count_ + right.count_ + std::lround((v - r) / 1000.0),
				r
			};
		}
		friend auto operator+(LargeNum left, double right)->LargeNum {
			double v = left.value_ + right;
			double r = std::fmod(v, 1000.0);
			return LargeNum{
				left.count_ + std::lround((v - r) / 1000.0),
				r
			};
		}
		friend auto operator+(double left, LargeNum right)->LargeNum {
			double v = left + right.value_;
			double r = std::fmod(v, 1000.0);
			return LargeNum{
				right.count_ + std::lround((v - r) / 1000.0),
				r
			};
		}
		friend auto operator-(LargeNum left, LargeNum right)->LargeNum {
			double v = left.value_ - right.value_;
			double r = std::fmod(v, 1000.0);
			return LargeNum{
				left.count_ - right.count_ + std::lround((v - r) / 1000.0),
				r
			};
		}
		friend auto operator-(LargeNum left, double right)->LargeNum {
			double v = left.value_ - right;
			double r = std::fmod(v, 1000.0);
			return LargeNum{
				left.count_ + std::lround((v - r) / 1000.0),
				r
			};
		}
		friend auto operator-(double left, LargeNum right)->LargeNum {
			double v = left - right.value_;
			double r = std::fmod(v, 1000.0);
			return LargeNum{
				-right.count_ + std::lround((v - r) / 1000.0),
				r
			};
		}
	};

	struct ARIS_API SCurveParam {
		LargeNum pb_{ 0.0 };     // 结束位置
		double vc_max_{ 1.0 };   // 允许的最大速度
		double vb_max_{ 1.0 };   // 允许的最大结束速度
		double a_{ 1.0 };        // 过程中最大加速度
		double j_{ 1.0 };        // 过程中最大加加速度

		LargeNum pa_{ 0.0 };     // 起始位置
		double va_{ 0.0 };       // 起始速度
		double T_{ 0.0 };        // 总时长，T = Ta + Tb + Tc，Tc是匀速段时长

		double vb_{ 0.0 };       // 结束速度
		double vc_{ 0.0 };       // 匀速段速度
		double Ta_{ 0.0 };       // 起始段加速时长
		double Tb_{ 0.0 };       // 结束段加速时长
		int    mode_{ 0 };       // A or B 模式

		LargeNum t0_{ 0.0 };     // 起始时间
		double va_upper_{ 0.0 }; // va 上限
		double va_below_{ 0.0 }; // va 下限
		double vb_upper_{ 0.0 }; // vb 上限
		double vb_below_{ 0.0 }; // vb 下限

		double smooth_Ta_{ 0.0 };       // 平滑后的 Ta
		double smooth_Tb_{ 0.0 };       // 平滑后的 Tb
		double smooth_vc_{ 0.0 };       // 平滑后的 vc
		double smooth_a_{ 0.0 };        // 平滑后的 a
		double smooth_j1_{ 0.0 };       // 平滑后的 j1
		double smooth_j2_{ 0.0 };       // 平滑后的 j2
	};

	struct ARIS_API SCurveNode {
		std::vector<SCurveParam> params_;
	};

	/// @brief 构造单节点的 S 曲线参数（仅处理单个节点，支持正向/反向位移）
	///
	/// 起始/结束速度 va、vb，起始时间 t0 与 mode 均固定为 0。按各维位移幅值
	/// |pb_ - pa_| 计算时间最优的 T、vc、Ta、Tb，取各维最大值后统一缩放到
	/// 最大 T（缩放即平滑），并把最终轨迹参数写入 smooth_*。
	///
	/// 输出的 smooth_* 系列（smooth_Ta_、smooth_Tb_、smooth_vc_、smooth_a_、
	/// smooth_j1_、smooth_j2_）即为 s_scurve_at() 计算任意时刻 p、v、a、j
	/// 具体数值所用的最终轨迹参数。其中 smooth_vc_ 可能为正或负（方向与位移
	/// 一致），smooth_a_、smooth_j1_、smooth_j2_ 恒为正（取绝对值意义上的
	/// 幅值）。
	///
	/// 注意：va_upper_、va_below_、vb_upper_、vb_below_ 等参数完全不参与
	/// 本函数的计算，本函数也不会修改它们的值。
	///
	/// @param param_num params 中的参数数量（维度）
	/// @param params SCurveParam 数组（长度为 param_num）
	///   - 输入：pa_、pb_、vc_max_、a_、j_（pb_ 可大于或小于 pa_，位移非零时
	///     需 vc_max_ > 0、a_ > 0、j_ > 0）
	///   - 输出：T_、vc_、Ta_、Tb_ 及 smooth_*（vc_ 与 smooth_vc_ 带位移方向
	///     符号）
	/// @param T_min 最小总时长，最终 T 不小于该值
	/// @return 0 表示成功
	auto ARIS_API s_scurve_make(Size param_num, SCurveParam *params, double T_min = 0.001)->int;

	/// @brief 构造多节点的 S 曲线参数（整段轨迹由多个节点组成）
	///
	/// 与 s_scurve_make() 的单节点不同，本函数处理一段完整的多节点轨迹：
	/// 严格要求各节点 pb_ >= pa_（不允许反向位移），且相邻节点的位置连续，
	/// 即本节点的 pb_ 等于下一节点的 pa_（本函数会自动用上一节点的 pb_ 设置
	/// 下一节点的 pa_）。
	///
	/// 轨迹首节点的起始速度 va 与末节点的结束速度 vb 均固定为 0，中间节点的
	/// 速度保持连续。计算时先求各节点时间最优的 T，再取统一可行值，最后完成
	/// 平滑并把最终轨迹参数写入各节点的 smooth_*。
	///
	/// @param begin_iter 轨迹首节点的迭代器（前闭后开区间 [begin_iter, end_iter)）
	/// @param end_iter 轨迹末节点之后（不包含）的迭代器
	/// @param T_min 最小总时长，最终 T 不小于该值
	/// @return 0 表示成功，-1 表示失败
	auto ARIS_API s_scurve_make_nodes(std::list<SCurveNode>::iterator begin_iter, std::list<SCurveNode>::iterator end_iter, double T_min = 0.001)->int;

	/// @brief 计算指定时间处的位置、速度、加速度与加加速度
	///
	/// 根据 param 中的轨迹参数（使用 smooth_* 系列：smooth_Ta_、smooth_Tb_、
	/// smooth_vc_、smooth_a_、smooth_j1_、smooth_j2_）计算时刻 t 处的
	/// p、v、a、j。时间 t 需在 [t0_, t0_ + T_] 区间内。
	///
	/// @param param SCurveParam 轨迹参数（由 s_scurve_make() 或
	///              s_scurve_make_nodes() 构造）
	/// @param t 待求值的时刻（绝对时间）
	/// @param p_out 输出位置（可为 nullptr 表示不需要）
	/// @param v_out 输出速度（可为 nullptr 表示不需要）
	/// @param a_out 输出加速度（可为 nullptr 表示不需要）
	/// @param j_out 输出加加速度（可为 nullptr 表示不需要）
	auto ARIS_API s_scurve_at(const SCurveParam& param, LargeNum t, LargeNum *p_out, double* v_out = nullptr, double* a_out = nullptr, double* j_out = nullptr)noexcept->void;
}

#endif