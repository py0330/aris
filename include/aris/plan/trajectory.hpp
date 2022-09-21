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
#include <aris/plan/scurve.hpp>

/// \brief 轨迹规划命名空间
/// \ingroup aris
/// 
///
///
namespace aris::plan{
	class ARIS_API TrajectoryGenerator {
	public:
		struct Node {
			enum class MoveType {
				Line,
				Circle
			};
			MoveType motion_type_;
			std::vector<double> ee_pos, mid_pos;
			std::vector<double> vel, acc, jerk, zone;
			std::vector<double> s1, s2, s3;
			SCurveNode scurve;


			std::list<SCurveNode>::iterator scurve_iter_;
			std::atomic<Node*> next_node_;

			~Node() = default;
			Node() = default;
			Node(const Node& other):
				motion_type_(other.motion_type_),
				ee_pos(other.ee_pos),
				scurve_iter_(other.scurve_iter_),
				next_node_(other.next_node_.load())
			{
			}
		};
		// 配置末端类型 //
		auto setEeTypes(const std::vector<aris::dynamic::EEType>& ee_types)->void {
			ee_types_ = ee_types;
		}

		// 设置时间间隔 //
		auto setDt(double dt)->void {
			dt_ = dt;
		}

		// 设置时间间隔 //
		auto setTargetDs(double ds)->void {
			ds_ = ds;
		}

		// 设置时间流逝的加速度
		auto setTargetDds(double dds)->void {
			dds_ = dds;
		}

		// 设置时间流逝的加加速度
		auto setTargetDdds(double ddds)->void {
			ddds_ = ddds;
		}

		// 获取末端数据，并移动dt //
		auto getEePosAndMoveDt(double *ee_pos)->void;

		// 插入新的数据，并重规划 //
		auto insertLinePos(const double* ee_pos)->void {
			
		}
		// 插入新的数据，并重规划 //
		auto insertCirclePos(const double* ee_pos, const double *mid_pos)->void {
			Node node;
		}

	private:
		double dt_, ds_, dds_, ddds_;
		std::vector<aris::dynamic::EEType> ee_types_;
		std::list<Node> nodes_;
		// 当前正在使用的节点 //
		std::atomic<Node*> next_node_;


		//std::list<Node>::iterator current_node_;
		//// 下次所需重规划的节点 //
		//std::list<Node>::iterator deceleration_node_;


		//std::list<Node>::iterator current_node_;
	};


}

#endif