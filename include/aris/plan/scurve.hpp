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
		double vb_max_{ 1.0 };   // 允许的最大末端速度
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
	};

	struct ARIS_API SCurveNode {
		std::vector<SCurveParam> params_;
	};

	// to be hide //
	auto s_scurve_plan_eliminate_optimization(double vb1, SCurveParam& param1)->void;

	// 构造scurve
	auto ARIS_API s_compute_scurve(std::list<SCurveNode>::iterator begin_iter, std::list<SCurveNode>::iterator end_iter, double T_min = 0.001)->int;

	// 构造scurve2：
	auto ARIS_API s_scurve_make_nodes(std::list<SCurveNode>::iterator begin_iter, std::list<SCurveNode>::iterator end_iter, double T_min = 0.001)->int;

	// 平滑优化scurve
	auto ARIS_API s_optimize_scurve_adjacent_nodes(std::list<SCurveNode>::iterator begin_iter, std::list<SCurveNode>::iterator end_iter, double T_min = 0.001)->int;

	// 计算指定时间处的 p v a j
	auto ARIS_API s_scurve_at(const SCurveParam& param, LargeNum t, LargeNum *p_out, double* v_out = nullptr, double* a_out = nullptr, double* j_out = nullptr)noexcept->void;
}

#endif