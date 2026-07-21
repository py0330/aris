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

		/// @brief 插入直线数据，插入数据时并不直接生效，而是等到调用 updateInsertPos 后才会统一规划
		/// @param chanel 通道 id
		/// @param tools 工具坐标系字符串，格式为 "tool1;tool2; ... ;tooln"，如果某个工具坐标系为空字符串,则用对应的 ee 的 makI 代替
		/// @param wobjs 工件坐标系字符串，格式为 "wobj1;wobj2; ... ;wobjn"，如果某个工件坐标系为空字符串,则用对应的 ee 的 makJ 代替
		/// @param tw_pos tool 相对于 wobj 的位姿，其中 twi_pos 的格式由对应 ee 的 pos type 决定，例如 pos type 为 PE321 则格式为 {x,y,z,rz,ry,rx}
		/// @param vel tool 相对于 wobj 的速度，大小取决于 pos type 的dim（维度），例如 PE321 的速度大小为 2，只有线速度和角速度
		/// @param acc tool 相对于 wobj 的加速度，同上
		/// @param jerk tool 相对于 wobj 的跃度，同上
		/// @param zone tool 相对于 wobj 的转弯区，同上
		/// @return 返回规划器内当前节点的 id，如果 tool 和 wobj 对应不上，会抛异常
		auto insertLinePos(int chanel, std::string_view tools, std::string_view wobjs, const double* tw_pos, const double* vel, const double* acc, const double* jerk, const double* zone, double time_zone = 0.0) -> std::int64_t;
		
		/// @brief 插入圆弧数据，插入数据时并不直接生效，而是等到调用 updateInsertPos 后才会统一规划
		/// @param chanel 通道 id
		/// @param tools 工具坐标系字符串，格式为 "tool1;tool2; ... ;tooln"，如果某个工具坐标系为空字符串,则用对应的 ee 的 makI 代替
		/// @param wobjs 工件坐标系字符串，格式为 "wobj1;wobj2; ... ;wobjn"，如果某个工件坐标系为空字符串,则用对应的 ee 的 makJ 代替
		/// @param tw_pos tool 相对于 wobj 的位姿，其中 twi_pos 的格式由对应 ee 的 pos type 决定，例如 pos type 为 PE321 则格式为 {x,y,z,rz,ry,rx}
		/// @param tw_mid_pos 圆弧中间点的位姿
		/// @param vel tool 相对于 wobj 的速度，大小取决于 pos type 的dim（维度），例如 PE321 的速度大小为 2，只有线速度和角速度
		/// @param acc tool 相对于 wobj 的加速度，同上
		/// @param jerk tool 相对于 wobj 的跃度，同上
		/// @param zone tool 相对于 wobj 的转弯区，同上
		/// @return 返回规划器内当前节点的 id，如果 tool 和 wobj 对应不上，会抛异常
		auto insertCirclePos(int chanel, std::string_view tools, std::string_view wobjs, const double* tw_pos, const double* tw_mid_pos, const double* vel, const double* acc, const double* jerk, const double* zone, double time_zone = 0.0) -> std::int64_t;

		/// @brief 插入工具相对于工件的位姿，但用关节空间走过去，插入数据时并不直接生效，而是等到调用 updateInsertPos 后才会统一规划
		/// @param chanel 通道 id
		/// @param tools 工具坐标系字符串，格式为 "tool1;tool2; ... ;tooln"，如果某个工具坐标系为空字符串,则用对应的 ee 的 makI 代替
		/// @param wobjs 工件坐标系字符串，格式为 "wobj1;wobj2; ... ;wobjn"，如果某个工件坐标系为空字符串,则用对应的 ee 的 makJ 代替
		/// @param tw_pos tool 相对于 wobj 的位姿，其中 twi_pos 的格式由对应 ee 的 pos type 决定，例如 pos type 为 PE321 则格式为 {x,y,z,rz,ry,rx}
		/// @param joint_v 关节速度
		/// @param joint_a 关节加速度
		/// @param joint_j 关节跃度
		/// @param joint_z 关节转弯区
		/// @param which_root 指定运动学的反解（例如6轴为 0-7，在此范围外采用默认解），nullptr时全部选默认解，默认用距离上个节点最近的解
		/// @return 返回规划器内当前节点的 id，如果 tool 和 wobj 对应不上，会抛异常
		auto insertMoveJPos(int chanel, std::string_view tools, std::string_view wobjs, const double* tw_pos, const double* joint_v, const double* joint_a, const double* joint_j, const double* joint_z, const std::int64_t *which_root, double time_zone = 0.0) -> std::int64_t;
		
		/// @brief 插入轴空间位置，插入数据时并不直接生效，而是等到调用 updateInsertPos 后才会统一规划
		/// @param chanel 通道 id
		/// @param joint_p 关节位置
		/// @param joint_v 关节速度
		/// @param joint_a 关节加速度
		/// @param joint_j 关节跃度
		/// @param joint_z 关节转弯区
		/// @return 返回规划器内当前节点的 id
		auto insertMoveAbsJPos(int chanel, const double* joint_p, const double* joint_v, const double* joint_a, const double* joint_j, const double* joint_z, double time_zone = 0.0) -> std::int64_t;
		
		
		/// @brief 统一计算已插入的数据，并重规划
		/// @param chanel 通道 id
		auto updateInsertPos(int chanel) -> void;

		////////////////// PART 3 RT operation ////////////////

		/// @brief 获取下一个位置
		/// @param chanel 通道 id
		/// @param p 电机位置
		/// @return 返回规划器内当前节点的 id，如果为 0 则规划执行完毕
		auto getNextInput(int chanel, double* p) -> std::int64_t;

		/// @brief 获取前瞻节点处的规划器返回值
		/// @param chanel 通道 id
		/// @return 返回规划器内节点的 id，如果为 0 则规划执行完毕
		auto tgRet(int chanel) -> std::int64_t;

		/// @brief 获取前瞻节点处的逆运动学返回值
		/// @param chanel 通道 id
		/// @return 逆运动学返回值，一般来说 ret < 0 为报错
		auto ikRet(int chanel) -> std::int64_t;

		/// @brief 获取当前节点剩余时间
		/// @param chanel 通道 id
		/// @return 剩余时间
		auto leftNodeS(int chanel) -> double;

		/// @brief 设置目标速度系数，应取 [0, 1] 之间的值，0 表示停止，1 表示全速
		/// @param chanel 通道 id
		/// @param ds 目标速度系数
		auto setTargetSpeedRatio(int chanel, double ds) -> void; // 0 <= ds <= 1
		
		/// @brief 获取目标速度系数
		/// @param chanel 通道 id
		/// @return 目标速度系数
		auto targetSpeedRatio(int chanel) -> double;

		/// @brief 获取实际速度系数
		/// @param chanel 通道 id
		/// @return 实际速度系数
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
