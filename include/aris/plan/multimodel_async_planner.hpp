#ifndef ARIS_PLAN_MULTIMODEL_ASYNC_PLANNER_H_
#define ARIS_PLAN_MULTIMODEL_ASYNC_PLANNER_H_

#include <list>
#include <cmath>
#include <iostream>
#include <functional>
#include <map>
#include <any>

#include <aris/core/object.hpp>
#include <aris/core/expression_calculator.hpp>
#include <aris/plan/trajectory.hpp>
#include <aris/dynamic/model_base.hpp>

/// \brief 轨迹规划命名空间
/// \ingroup aris
/// 
///
///
/// 
/// 
namespace aris::plan{

	class ARIS_API ToolWobjSelector {
	public:
		using MarkerVec = std::vector<aris::dynamic::Marker*>;
		
		// 配置模型 //
		auto setModel(aris::dynamic::MultiModel& model) -> void;
		auto model() -> aris::dynamic::MultiModel&;

		auto computeEePos(MarkerVec& tools, MarkerVec& wobjs, const double* twpos, double* eepos)->int;
		auto computeTwPos(MarkerVec& tools, MarkerVec& wobjs, const double* eepos, double* twpos)->int;

		~ToolWobjSelector();
		ToolWobjSelector();
		ARIS_DELETE_BIG_FOUR(ToolWobjSelector);

	private:
		struct Imp;
		std::unique_ptr<Imp> imp_;
	};

	class ARIS_API MultimodelAsyncPlanner {
	public:
		////////////////// PART 1 config ////////////////
		
		// 配置末端类型 //
		auto setModel(aris::dynamic::MultiModel& model)->void;
		auto model() -> aris::dynamic::MultiModel&;

		auto setDt(double dt) -> void;
		auto dt() -> double;
		
		auto eeTypes()const -> const std::vector<aris::dynamic::EEType>&;
		auto setEeTypes(const std::vector<aris::dynamic::EEType>& ee_types) -> void;
		
		auto setInputSize(int input_size) -> void;
		auto inputSize() -> int;
		


		auto setMaxPos(aris::core::Matrix pos) -> void;
		auto maxPos() -> aris::core::Matrix;
		auto setMaxVel(aris::core::Matrix vel) -> void;
		auto maxVel() -> aris::core::Matrix;
		auto setMaxAcc(aris::core::Matrix acc) -> void;
		auto maxAcc() -> aris::core::Matrix;
		auto setMinPos(aris::core::Matrix pos) -> void;
		auto minPos() -> aris::core::Matrix;
		auto setMinVel(aris::core::Matrix vel) -> void;
		auto minVel() -> aris::core::Matrix;
		auto setMinAcc(aris::core::Matrix acc) -> void;
		auto minAcc() -> aris::core::Matrix;
		
		// 笛卡尔空间中重规划个数 //
		auto maxReplanNum()const -> int;
		auto setMaxReplanNum(int max_replan_num = 10) -> void;
		
		// 前瞻个数 //
		auto setLookAheadCount(int count) -> void;
		auto lookAheadCount() -> int;

		// 异步规划时缓存个数 //
		auto setCacheSize(int cache_size) -> void;
		auto cacheSize() -> int;

		////////////////// PART 2 NRT operation ////////////////

		auto allocateMemory() -> void;
		auto init() -> void;
		auto stop() -> void;

		// 插入新的数据，并重规划 //
		auto insertInitPos(std::int64_t id, const double* ee_pos) -> void;

		// 插入新的数据，并重规划 //
		auto insertLinePos(std::int64_t id, const double* ee_pos, const double* vel, const double* acc, const double* jerk, const double* zone) -> void;

		// 插入新的数据，并重规划 //
		auto insertCirclePos(std::int64_t id, const double* ee_pos, const double* mid_pos, const double* vel, const double* acc, const double* jerk, const double* zone) -> void;


		// 插入新的数据，并重规划 //
		auto insertLinePos(std::vector<std::pair<std::string, std::string>> tool_wobjs, const double* ee_pos, const double* vel, const double* acc, const double* jerk, const double* zone) -> void;

		// 插入新的数据，并重规划 //
		auto insertCirclePos(std::vector<std::pair<std::string, std::string>> tool_wobjs, const double* ee_pos, const double* mid_pos, const double* vel, const double* acc, const double* jerk, const double* zone) -> void;



		// 删除已经不用的数据 //
		auto clearUsedPos() -> void;

		// 删除全部数据 //
		auto clearAllPos() -> void;

		// 当前还剩余的指令数 //
		auto unusedPosNum() -> int;

		// 返回当前所有的节点 id //
		auto unusedNodeIds()const -> std::vector<std::int64_t>;

		// 调速设置 //
		auto setTargetSpeedRatio(double ds) -> void; // 0 <= ds <= 1
		auto targetSpeedRatio() -> double;
		auto actualSpeedRatio() -> double;


		////////////////// PART 3 RT operation ////////////////
		auto getNextInput(double* p) -> std::int64_t;



		~MultimodelAsyncPlanner();
		MultimodelAsyncPlanner();
		ARIS_DELETE_BIG_FOUR(MultimodelAsyncPlanner);

	private:
		struct Imp;
		std::unique_ptr<Imp> imp_;
	};
	
}

#endif