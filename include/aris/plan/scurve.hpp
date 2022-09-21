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
	struct ARIS_API SCurveParam {
		double pb_;       // 结束位置
		int    pb_count_; // 结束位置对应的计数器，为避免累计误差，实际的 pa 应为：pa + pa_count * 1000
		double vc_max_;   // 允许的最大速度
		double vb_max_;   // 允许的最大末端速度
		double a_;        // 过程中最大加速度
		double j_;        // 过程中最大加加速度

		double pa_;       // 起始位置
		int    pa_count_; // 起始位置对应的计数器，为避免累计误差，实际的 pa 应为：pa + pa_count * 1000
		double va_;       // 起始速度
		double T_;        // 总时长，T = Ta + Tb + Tc，Tc是匀速段时长

		double vb_;       // 结束速度
		double vc_;       // 匀速段速度
		double Ta_;       // 起始段加速时长
		double Tb_;       // 结束段加速时长
		int    mode_;     // A or B 模式

		double t0_;       // 开始的时间
		int    t0_count_;  // 开始时间对应的计数器，为避免累计误差，实际的 t0 应为：pa + pa_count * 1000
	};

	struct ARIS_API SCurveNode {
		std::vector<SCurveParam> params_;
	};

	struct ARIS_API SCurveStruct {
		std::list<SCurveNode> nodes_;
		using iterator = std::list<SCurveNode>::iterator;

		// 应当处理每个节点的如下信息：
		// - pb
		// - vc_max_
		// - vb_max_
		// - a
		// - j
		//
		// 以及首节点的如下信息：
		// - pa
		// - va
		auto insertNodes(std::list<SCurveNode>& ins_nodes)->void;
	};

	// 计算指定时间处的 p v a j
	auto ARIS_API s_compute_scurve(std::list<SCurveNode>::iterator& begin_iter, std::list<SCurveNode>::iterator& end_iter)noexcept->void;

	// 计算指定时间处的 p v a j
	auto ARIS_API s_scurve_at(const SCurveParam& param, int t_count, double t, int* p_count = nullptr, double* p_out = nullptr, double* v_out = nullptr, double* a_out = nullptr, double* j_out = nullptr)noexcept->void;
}

#endif