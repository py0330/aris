#ifndef ARIS_PLAN_TRAJECTORY_H_
#define ARIS_PLAN_TRAJECTORY_H_

#include <list>
#include <cmath>
#include <iostream>
#include <functional>
#include <map>
#include <any>

//#include <aris/core/core.hpp>
//#include <aris/control/control.hpp>
#include <aris/dynamic/dynamic.hpp>

/// \brief 轨迹规划命名空间
/// \ingroup aris
/// 
///
///
namespace aris::plan{
	struct ARIS_API SCurveParam {
		double pb_;     // 结束位置
		double vc_max_; // 允许的最大速度
		double vb_max_; // 允许的最大末端速度
		double a_;      // 过程中最大加速度
		double j_;      // 过程中最大加加速度

		double pa_;     // 起始位置
		double va_;     // 起始速度
		double T_;      // 总时长，T = Ta + Tb + Tc，Tc是匀速段时长

		double vb_;     // 结束速度
		double vc_;     // 匀速段速度
		double Ta_;     // 起始段加速时长
		double Tb_;     // 结束段加速时长
		int    mode_;   // A or B 模式

		double t0_;     // 开始的时间
	};

	struct ARIS_API SCurveNode {
		std::vector<SCurveParam> params_;
	};

	struct ARIS_API SCurveStruct {
		std::list<SCurveNode> nodes_;





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

		// 循环计算每个节点：
		auto compute()->void;

	};

	
	// 给定时间，计算加加速段的终止速度
	auto ARIS_API s_acc_vend(double va, double a, double j, double T)noexcept->double;
	// 给定速度，计算加加速段的终止速度
	auto ARIS_API s_acc_vend(double va, double a, double j, double vb)noexcept->double;


	// 计算指定时间处的 p v a j
	auto ARIS_API s_s_curve_at(const SCurveParam& param, double t, double* p = nullptr, double* v = nullptr, double* a = nullptr, double* j = nullptr)noexcept->void;

	// 根据起始条件，终止位置，总时间来计算 S 曲线
	//
	// param 中的以下信息为输入：
	// - pb
	// - vc_max_
	// - vb_max_
	// - a
	// - j
	// - pa
	// - va
	// - T
	// 
	// 以下信息为输出：
	// - vb_
	// - vc_
	// - Ta_
	// - Tb_
	// - mode_
	auto ARIS_API s_compute_s_curve(SCurveParam& param)->void;

	// 计算 S 曲线的最大最小时间
	//
	// param 中的以下为输入：
	// - pb
	// - vc_max_
	// - vb_max_
	// - a
	// - j
	// - pa
	// - va
	// 
	// 以下不变：
	// - vb_
	// - vc_
	// - Ta_
	// - Tb_
	// - mode_
	// - T_
	//
	// 函数输出：
	// Tmax, Tmin
	auto ARIS_API s_compute_s_curve_Tmax_Tmin(const SCurveParam &param)->std::tuple<double,double>;
}

#endif