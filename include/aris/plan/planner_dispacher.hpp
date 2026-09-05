#ifndef ARIS_PLAN_PLANNER_DISPACHER_H_
#define ARIS_PLAN_PLANNER_DISPACHER_H_

#include <cstdint>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

#include <aris/core/expression_calculator.hpp>
#include <aris/core/object.hpp>
#include <aris/dynamic/model.hpp>

namespace aris::plan {

	class MultimodelPlanner;

	/// @brief 规划调度器
	class ARIS_API PlannerDispacher {
	public:
		////////////////// PART 1 config ////////////////

		/// @brief 设置时间步长
		/// @param dt 时间步长
		auto setDt(double dt) -> void;

		/// @brief 返回时间步长
		/// @return 时间步长
		auto dt() -> double;

		/// @brief 设置模型
		/// @param model 模型
		auto setModel(aris::dynamic::MultiModel& model) -> void;
		
		/// @brief 返回模型
		/// @return 模型
		auto model() -> aris::dynamic::MultiModel&;

		/// @brief 设置最大通道数
		/// @param chanel_size 通道数
		auto setChanelSize(int chanel_size) -> void;

		/// @brief 返回最大通道数
		/// @return 通道数
		auto chanelSize() -> int;

		/// @brief 返回子模型的转换矩阵，每个模型对应一个转换矩阵
		/// @return 转换矩阵
		auto transferMatrice() -> std::vector<aris::core::Matrix>&;

		/// @brief 返回指定通道的规划器
		/// @param chanel 通道 id
		/// @return 规划器
		auto plannerAt(int chanel) -> MultimodelPlanner&;

		/// @brief 初始化规划器，必须在设置完模型、通道数和 dt 后调用
		auto init() -> void;

		////////////////// PART 2 NRT operation ////////////////
		
		/// @brief 尝试锁定通道
		/// @param chanel 通道 id
		/// @param submodel_ids 需要锁定的子模型 id 列表
		/// @return 成功则返回锁定重数，每次锁定，锁定重数+1，每次释放，锁定重数-1；失败返回：通道不存在 -1，通道被不同子模型占用 -2，子模型已被其他通道占用 -3
		auto tryLockChanel(int chanel, std::vector<aris::Size> submodel_ids) -> int;
		
		/// @brief 释放通道
		/// @param chanel 通道 id
		/// @return 成功则返回锁定重数，每次锁定，锁定重数+1，每次释放，锁定重数-1；失败返回：通道不存在 -1，通道未被锁定 -2
		auto releaseChanel(int chanel) -> int;

		////////////////// PART 3 RT operation ////////////////

		/// @brief 获取下一个位置
		/// @param chanel 通道 id
		/// @param p 电机位置
		/// @return 返回规划器内当前节点的 id，如果为 0 则规划执行完毕
		auto getNextInput(int chanel, double* p) -> std::int64_t;

		~PlannerDispacher();
		PlannerDispacher();
		ARIS_DELETE_BIG_FOUR(PlannerDispacher);

	private:
		struct Imp;
		std::unique_ptr<Imp> imp_;
	};

}

#endif
