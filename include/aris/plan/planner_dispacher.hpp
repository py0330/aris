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

		auto setModel(aris::dynamic::MultiModel& model) -> void;
		auto model() -> aris::dynamic::MultiModel&;

		auto setChanelSize(int chanel_size) -> void;
		auto chanelSize() -> int;

		auto transferMatrice() -> std::vector<aris::core::Matrix>&;

		auto init() -> void;
        
		////////////////// PART 2 NRT operation ////////////////
		auto tryLockChanel(int chanel, std::vector<aris::Size> submodel_ids) -> int;
		auto releaseChanel(int chanel) -> int;

		// 插入新的数据，并重规划 //
		auto insertLinePos(int chanel, std::string_view tools, std::string_view wobjs, const double* ee_pos, const double* vel, const double* acc, const double* jerk, const double* zone) -> std::int64_t;
		auto insertCirclePos(int chanel, std::string_view tools, std::string_view wobjs, const double* ee_pos, const double* mid_pos, const double* vel, const double* acc, const double* jerk, const double* zone) -> std::int64_t;
		auto updateInsertPos(int chanel) -> void;

		////////////////// PART 3 RT operation ////////////////

		/// @brief 获取下一个位置
		/// @param chanel 通道
		/// @param p 电机位置
		/// @return 返回规划器内当前节点的 id，如果为 0 则规划执行完毕
		auto getNextInput(int chanel, double* p) -> std::int64_t;

		/// @brief 获取前瞻节点处的规划器返回值
		/// @param chanel 通道
		/// @return 返回规划器内节点的 id，如果为 0 则规划执行完毕
		auto tgRet(int chanel) -> std::int64_t;

		/// @brief 获取前瞻节点处的逆运动学返回值
		/// @param chanel 通道
		/// @return 逆运动学返回值，一般来说 ret < 0 为报错
		auto ikRet(int chanel) -> std::int64_t;

		auto setTargetSpeedRatio(int chanel, double ds) -> void; // 0 <= ds <= 1
		auto targetSpeedRatio(int chanel) -> double;
		auto actualSpeedRatio(int chanel) -> double;

		~PlannerDispacher();
		PlannerDispacher();
		ARIS_DELETE_BIG_FOUR(PlannerDispacher);

	private:
		struct Imp;
		std::unique_ptr<Imp> imp_;
	};

}

#endif
