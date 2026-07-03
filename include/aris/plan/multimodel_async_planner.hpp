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

		auto setSubModelId(std::vector<aris::Size> id_list) -> void;
		auto subModelId() -> const std::vector<aris::Size>&;

		auto selectTw(aris::dynamic::Marker** tools, aris::dynamic::Marker** wobjs) -> int;
		auto setTwPos(const double* twpos)->void;
		auto getTwPos(double* twpos)->void;
		auto setEePos(const double* eepos) -> void;
		auto getEePos(double* eepos) -> void;

		~ToolWobjSelector();
		ToolWobjSelector();
		ARIS_DELETE_BIG_FOUR(ToolWobjSelector);

	private:
		struct Imp;
		std::unique_ptr<Imp> imp_;
	};

	class ARIS_API MultimodelPlanner {
	public:
		using TW = std::vector<std::pair<std::string, std::string>>;
		
		////////////////// PART 1 config ////////////////

		auto setDt(double dt) -> void;
		auto dt() -> double;
		
		auto setModel(aris::dynamic::MultiModel& model)->void;
		auto model() -> aris::dynamic::MultiModel&;

		auto setSubModelId(std::vector<aris::Size> id_list) -> void;
		auto subModelId() -> const std::vector<aris::Size> &;

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

		// 是否启用异步规划 //
		auto setAsync(bool is_async = false) -> void;
		auto isAsync() -> bool;

		// 异步规划时缓存个数 //
		auto setAsyncCacheSize(int cache_size) -> void;
		auto asyncCacheSize() -> int;

		////////////////// PART 2 NRT operation ////////////////

		auto allocateMemory() -> void;
		auto init() -> void;
		auto stop() -> void;

		// 插入新的数据 //
		auto insertLinePos(TW& tw, const double* ee_pos, const double* vel, const double* acc, const double* jerk, const double* zone) -> std::int64_t;
		auto insertLinePos(std::string_view tools, std::string_view wobjs, const double* ee_pos, const double* vel, const double* acc, const double* jerk, const double* zone) -> std::int64_t;

		// 插入新的数据 //
		auto insertCirclePos(TW& tw, const double* ee_pos, const double* mid_pos, const double* vel, const double* acc, const double* jerk, const double* zone) -> std::int64_t;
		auto insertCirclePos(std::string_view tools, std::string_view wobjs, const double* ee_pos, const double* mid_pos, const double* vel, const double* acc, const double* jerk, const double* zone) -> std::int64_t;

		// 插入新的数据 //
		auto insertMoveJ(TW& tw, const double* ee_pos, const double* joint_v, const double* joint_a, const double* joint_j, const double* zone, const std::int64_t *which_root = nullptr) -> std::int64_t;
		auto insertMoveJ(std::string_view tools, std::string_view wobjs, const double* ee_pos, const double* joint_v, const double* joint_a, const double* joint_j, const double* zone, const std::int64_t *which_root = nullptr) -> std::int64_t;

		// 插入新的数据 //
		auto insertMoveAbsJ(const double* joint_p, const double* joint_v, const double* joint_a, const double* joint_j, const double* zone) -> std::int64_t;


		// 重规划 //
		auto updateInsertPos()->void;

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

		/// @brief 获取规划器返回值
		/// @param chanel 通道
		/// @return 返回规划器内节点的 id，如果为 0 则规划执行完毕
		auto tgRet() -> std::int64_t;


		/// @brief 获取逆运动学返回值
		/// @param chanel 通道
		/// @return 逆运动学返回值，一般来说 ret < 0 为报错
		auto ikRet() -> std::int64_t;

		/// @brief 获取输出位置类型
		/// @return 输出位置类型向量
		auto outputPosTypes()const -> const std::vector<aris::dynamic::PosType>&;

		/// @brief 获取输入维数
		/// @return 输入维数
		auto inputSize() -> int;

		~MultimodelPlanner();
		MultimodelPlanner();
		ARIS_DELETE_BIG_FOUR(MultimodelPlanner);

	private:
		struct Imp;
		std::unique_ptr<Imp> imp_;
	};
	
}

#endif