#ifndef ARIS_PLAN_ROOT_H_
#define ARIS_PLAN_ROOT_H_

#include <list>
#include <cmath>
#include <iostream>
#include <functional>
#include <map>
#include <any>
#include <atomic>
#include <future>

#include <aris/core/core.hpp>
#include <aris/control/control.hpp>
#include <aris/dynamic/dynamic.hpp>


namespace aris::server { class ControlServer; }

#define ARIS_COUT_PLAN(p) ARIS_COUT << "    " << p->cmdId() << "---"
#define ARIS_MOUT_PLAN(p) p->controller()->mout() << "RT  " << p->cmdId() << "---"

/// \brief 轨迹规划命名空间
/// \ingroup aris
/// 
///
///
namespace aris::plan
{
	// plan 依次执行3个函数：
	// ## prepareNrt
	// ## executeRT
	// ## collectNrt
	//
	// 所有的plan都必须执行 prepare，但在prepare中可以通过选项来决定是否执行 executeRT 或 collect
	//
	// 其中 prepare 可以抛异常，在此情况下后面两个函数一定不执行；executeRT 和 collect 一定不能抛异常
	//
	// 若要报错，请在不同阶段按如下方式来做：
	// prepare   :直接 throw std::exception 的继承类，在此之前可以决定是否要 collect
	// executeRT :返回小于0的数，并且可以通过 setErrMsgRT 来设置当前错误信息
	// collect   :不要报错，这个相当于析构函数，只要 prepare 不抛异常而且未设置NOT_RUN_COLLECT_FUNCTION，就一定会执行
	// 
	class Plan :public aris::core::Object
	{
	public:
		enum Option : std::uint64_t
		{
			NOT_PRINT_CMD_INFO = 0x01ULL << 0,
			NOT_PRINT_EXECUTE_COUNT = 0x01ULL << 1,
			NOT_LOG_CMD_INFO = 0x01ULL << 2,
			
			NOT_RUN_EXECUTE_FUNCTION = 0x01ULL << 3,
			NOT_RUN_COLLECT_FUNCTION = 0x01ULL << 4,

			WAIT_FOR_EXECUTION = 0x01ULL << 5,
			WAIT_IF_CMD_POOL_IS_FULL = 0x01ULL << 6,
			WAIT_FOR_COLLECTION = 0x01ULL << 7,
		};
		enum MotorOption : std::uint64_t
		{
			USE_TARGET_POS = 0x01ULL << 16,
			USE_TARGET_VEL = 0x01ULL << 17,
			USE_TARGET_TOQ = 0x01ULL << 18,
			USE_OFFSET_VEL = 0x01ULL << 19,
			USE_OFFSET_TOQ = 0x01ULL << 20,

			NOT_CHECK_MODE = 0x01ULL << 22,
			NOT_CHECK_ENABLE = 0x01ULL << 23,
			NOT_CHECK_POS_MIN = 0x01ULL << 24,
			NOT_CHECK_POS_MAX = 0x01ULL << 25,
			NOT_CHECK_POS_CONTINUOUS = 0x01ULL << 26,
			NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER = 0x01ULL << 28,
			NOT_CHECK_POS_FOLLOWING_ERROR = 0x01ULL << 30,

			NOT_CHECK_VEL_MIN = 0x01ULL << 31,
			NOT_CHECK_VEL_MAX = 0x01ULL << 32,
			NOT_CHECK_VEL_CONTINUOUS = 0x01ULL << 33,
			NOT_CHECK_VEL_FOLLOWING_ERROR = 0x01ULL << 35,

			CHECK_NONE = NOT_CHECK_MODE | NOT_CHECK_ENABLE | NOT_CHECK_POS_MIN | NOT_CHECK_POS_MAX | NOT_CHECK_POS_CONTINUOUS | NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER | NOT_CHECK_POS_FOLLOWING_ERROR |
				NOT_CHECK_VEL_MIN | NOT_CHECK_VEL_MAX | NOT_CHECK_VEL_CONTINUOUS | NOT_CHECK_VEL_FOLLOWING_ERROR, 
		};
		enum RetStatus
		{
			SUCCESS = 0,
			PARSE_EXCEPTION = -1,
			PREPARE_EXCEPTION = -2,
			SERVER_IN_ERROR = -10,
			SERVER_NOT_STARTED = -11,
			COMMAND_POOL_IS_FULL = -12,
			PREPARE_CANCELLED = -40,
			EXECUTE_CANCELLED = -41,
			
			SLAVE_AT_INIT = -101,
			SLAVE_AT_SAFEOP = -102,
			SLAVE_AT_PREOP = -103,
			SLAVE_AT_OP = -104,
			MOTION_NOT_ENABLED = -501,
			MOTION_POS_BEYOND_MIN = -502,
			MOTION_POS_BEYOND_MAX = -503,
			MOTION_POS_NOT_CONTINUOUS = -504,
			MOTION_POS_NOT_CONTINUOUS_SECOND_ORDER = -505,
			MOTION_POS_FOLLOWING_ERROR = -506,
			MOTION_VEL_BEYOND_MIN = -507,
			MOTION_VEL_BEYOND_MAX = -508,
			MOTION_VEL_NOT_CONTINUOUS = -509,
			MOTION_VEL_FOLLOWING_ERROR = -510,
			MOTION_INVALID_MODE = -511,

			PLAN_OVER_TIME = -1001,
			INVERSE_KINEMATIC_POSITION_FAILED = -1002,

			PROGRAM_EXCEPTION = -2000,
		};

		auto virtual prepareNrt()->void {}
		auto virtual executeRT()->int { return 0; }
		auto virtual collectNrt()->void {}
		// 指令分析相关 //
		auto command()->aris::core::Command &;
		auto command()const->const aris::core::Command & { return const_cast<std::decay_t<decltype(*this)> *>(this)->command(); }
		auto cmdString()->std::string_view;
		auto cmdName()->std::string_view;
		auto cmdParams()->const std::map<std::string_view, std::string_view> &;
		auto cmdId()->std::int64_t;
		auto doubleParam(std::string_view param_name)->double;
		auto floatParam(std::string_view param_name)->float;
		auto int32Param(std::string_view param_name)->std::int32_t;
		auto int64Param(std::string_view param_name)->std::int64_t;
		auto uint32Param(std::string_view param_name)->std::uint32_t;
		auto uint64Param(std::string_view param_name)->std::uint64_t;
		auto matrixParam(std::string_view param_name)->aris::core::Matrix;
		auto matrixParam(std::string_view param_name, int m, int n)->aris::core::Matrix;

		// shared ptr for this //
		auto sharedPtrForThis()->std::shared_ptr<Plan>;

		// 索引子模块 //
		auto controlServer()->aris::server::ControlServer*;
		auto model()->aris::dynamic::Model*;
		auto master()->aris::control::Master*;
		auto controller()->aris::control::Controller*;
		auto ecMaster()->aris::control::EthercatMaster*;
		auto ecController()->aris::control::EthercatController*;
		
		// 日志与打印 //
		auto lout()->aris::core::MsgStream & { return master()->lout(); }
		auto mout()->aris::core::MsgStream & { return master()->mout(); }

		// 规划与统计相关 //
		auto count()->std::int64_t;
		auto option()->std::uint64_t&;
		auto motorOptions()->std::vector<std::uint64_t>&;
		auto beginGlobalCount()->std::int64_t;
		auto rtStastic()->aris::control::Master::RtStasticsData &;

		// 输入参数与返回值 //
		auto param()->std::any&;
		auto ret()->std::any&;

		// 错误处理 //
		auto setErrMsgRT(const char *msg)->void;
		auto retCode()->std::int32_t;
		auto retMsg()->const char *;

		virtual ~Plan();
		explicit Plan(const std::string &name = "plan");
		ARIS_REGISTER_TYPE(Plan);
		ARIS_DECLARE_BIG_FOUR(Plan);

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
		friend class aris::server::ControlServer;
	};
	class PlanRoot :public aris::core::Object
	{
	public:
		auto planPool()->aris::core::ObjectPool<Plan> &;
		auto planPool()const->const aris::core::ObjectPool<Plan> & { return const_cast<std::decay_t<decltype(*this)> *>(this)->planPool(); }
		auto planParser()->aris::core::CommandParser &;
		auto init()->void;

		virtual ~PlanRoot();
		explicit PlanRoot(const std::string &name = "plan_root");
		ARIS_REGISTER_TYPE(PlanRoot);
		ARIS_DECLARE_BIG_FOUR(PlanRoot);

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};
	
	/// \brief 让电机使能
	///
	/// ### 参数定义 ###
	/// 指定电机，默认指定所有电机：
	/// + 指定所有电机：“en -a” 或 “en --all” 或 “en”
	/// + 按照绝对地址（absID），例如绝对的 0 号电机：“en -m=0” 或 “en --motion_id=0”
	/// + 按照物理地址（phyID），例如物理的 2 号电机：“en -p=2” 或 “en --physical_id=2”
	/// + 按照从站地址（slaID），例如 5 号从站：“en -s=5” 或 “en --slave_id=5”
	/// 
	/// 指定本指令的最长运行时间（默认为5000ms）：
	/// + 使能0号电机，并指定其最长时间为5000ms： “en -m=0 --limit_time=5000”
	/// 
	class Enable : public Plan
	{
	public:
		auto virtual prepareNrt()->void override;
		auto virtual executeRT()->int override;

		virtual ~Enable();
		explicit Enable(const std::string &name = "enable_plan");
		ARIS_REGISTER_TYPE(Enable);
		ARIS_DECLARE_BIG_FOUR(Enable);

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};
	/// \brief 让电机去使能
	/// 
	/// ### 参数定义 ###
	/// 指定电机，默认指定所有电机：
	/// + 指定所有电机：“ds -a” 或 “ds --all” 或 “ds”
	/// + 按照绝对地址（absID），例如绝对的 0 号电机：“ds -m=0” 或 “ds --motion_id=0”
	/// + 按照物理地址（phyID），例如物理的 2 号电机：“ds -p=2” 或 “ds --physical_id=2”
	/// + 按照从站地址（slaID），例如 5 号从站：“ds -s=5” 或 “ds --slave_id=5”
	/// 
	/// 指定本指令的最长运行时间（默认为5000ms）：
	/// + 去使能0号电机，并指定其最长时间为5000ms： “ds -m=0 --limit_time=5000”
	/// 
	class Disable : public Plan
	{
	public:
		auto virtual prepareNrt()->void override;
		auto virtual executeRT()->int override;

		virtual ~Disable();
		explicit Disable(const std::string &name = "disable_plan");
		ARIS_REGISTER_TYPE(Disable);
		ARIS_DECLARE_BIG_FOUR(Disable);

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};
	/// \brief 让电机回零
	/// 
	/// ### 参数定义 ###
	/// 指定电机，默认指定所有电机：
	/// + 指定所有电机：“hm -a” 或 “hm --all” 或 “hm”
	/// + 按照绝对地址（absID），例如绝对的 0 号电机：“en -m=0” 或 “en --motion_id=0”
	/// + 按照物理地址（phyID），例如物理的 2 号电机：“en -p=2” 或 “en --physical_id=2”
	/// + 按照从站地址（slaID），例如 5 号从站：“en -s=5” 或 “en --slave_id=5”
	/// 
	/// 指定本指令的最长运行时间（默认为5000ms）：
	/// + 使能0号电机，并指定其最长时间为5000ms： “en -m=0 --limit_time=5000”
	/// 
	class Home : public Plan
	{
	public:
		auto virtual prepareNrt()->void override;
		auto virtual executeRT()->int override;

		virtual ~Home();
		explicit Home(const std::string &name = "home_plan");
		ARIS_REGISTER_TYPE(Home);
		ARIS_DECLARE_BIG_FOUR(Home);

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};
	/// \brief 让电机切换模式
	/// 
	/// ### 参数定义 ###
	/// 指定电机，默认指定所有电机：
	/// + 指定所有电机：“md -a” 或 “md --all” 或 “md”
	/// + 按照绝对地址（absID），例如绝对的 0 号电机：“md -m=0” 或 “md --motion_id=0”
	/// + 按照物理地址（phyID），例如物理的 2 号电机：“md -p=2” 或 “md --physical_id=2”
	/// + 按照从站地址（slaID），例如 5 号从站：“md -s=5” 或 “md --slave_id=5”
	/// 
	/// 指定本指令的最长运行时间（默认为5000ms）：
	/// + 使能0号电机，并指定其最长时间为5000ms： “md -m=0 --limit_time=5000”
	/// 
	/// 指定模式（请参考canopen DS402标准，默认为8，同步位置控制模式）：
	/// + 使能0号电机，并指定模式为9： “md -m=0 --mode=9 --limit_time=5000”
	/// 
	class Mode : public Plan
	{
	public:
		auto virtual prepareNrt()->void override;
		auto virtual executeRT()->int override;

		virtual ~Mode();
		explicit Mode(const std::string &name = "mode_plan");
		ARIS_REGISTER_TYPE(Mode);
		ARIS_DECLARE_BIG_FOUR(Mode);

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};
	/// \brief 清理错误
	///
	/// 
	class Clear : public Plan
	{
	public:
		auto virtual prepareNrt()->void override;

		virtual ~Clear() = default;
		explicit Clear(const std::string &name = "clear_plan");
		ARIS_REGISTER_TYPE(Clear);
		ARIS_DEFINE_BIG_FOUR(Clear);
	};
	/// \brief 复位，机器人从轴空间按照指定速度运行到指定位置处
	///
	/// 指令不使用正反解，结束后Model和Controller位置不对应，此时需用rc指令来让两者重合
	///
	/// ### 参数定义 ###
	/// 指定电机，默认指定所有电机：
	/// + 指定所有电机：“rs -a” 或 “rs --all” 或 “rs”
	/// + 按照绝对地址（absID），例如绝对的 0 号电机：“rs -m=0” 或 “rs --motion_id=0”
	/// + 按照物理地址（phyID），例如物理的 2 号电机：“rs -p=2” 或 “rs --physical_id=2”
	/// + 按照从站地址（slaID），例如 5 号从站：“rs -s=5” 或 “rs --slave_id=5”
	/// 
	/// 指定复位位置，0为最小位置，1为最大位置，参数应在0到1之间，默认为0.5：
	/// + 将1号电机走到最大位置：“rs -m=1 --pos=1”
	/// + 将2号电机走到中间位置：“rs -m=2 --pos=0.5”
	/// + 将所有电机走到最小位置：“rs -a --pos=0”
	/// + 假设系统中有6个电机，分别走到0.1到0.6，注意大括号中不能有空格：“rs --pos={0.1,0.2,0.3,0.4,0.5,0.6}”
	/// 
	/// 指定复位速度，单位是额定速度的倍数，参数应在0到1之间，默认为0.1：
	/// + 将1号电机复位到0.5，速度为额定速度的20\%：“rs -m=1 --pos=0.5 --vel=0.2”
	/// + 将所有电机复位到0处，速度为额定的5\%：“rs -a --pos=0 --vel=0.05”
	/// + 假设系统中有6个电机，速度和位置都是0.1到0.6：“rs --pos={0.1,0.2,0.3,0.4,0.5,0.6} --vel={0.1,0.2,0.3,0.4,0.5,0.6}”
	///
	/// 指定复位加速度，单位是额定加速度的倍数，参数应在0到1之间，默认为0.1：
	/// + 将1号电机复位到0.5，加速度为额定速度的\20%：“rs -m=1 --pos=0.5 --acc=0.2”
	/// + 假设系统中有6个电机，加速度是0.1到0.6：“rs --acc={0.1,0.2,0.3,0.4,0.5,0.6}”
	/// 
	/// 指定复位减速度，单位是额定加速度的倍数，参数应在0到1之间，默认为0.1：
	/// + 将1号电机复位到0.5，减速度为额定速度的\20%：“rs -m=1 --pos=0.5 --dec=0.2”
	/// + 假设系统中有6个电机，减速度是0.1到0.6：“rs --dec={0.1,0.2,0.3,0.4,0.5,0.6}”
	/// 
	class Reset : public Plan
	{
	public:
		auto virtual prepareNrt()->void override;
		auto virtual executeRT()->int override;

		virtual ~Reset();
		explicit Reset(const std::string &name = "reset_plan");
		ARIS_REGISTER_TYPE(Reset);
		ARIS_DECLARE_BIG_FOUR(Reset);

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};
	/// \brief 恢复，消除错误但不运动
	/// 
	/// 在报错后，让模型（Model）根据控制器（Controller）读到的电机位置来更新。需要在非实时循环
	/// 计算正解（使用1号求解器），实时循环内无计算。
	///
	/// ### 参数定义 ###
	/// 无
	class Recover : public Plan
	{
	public:
		auto virtual prepareNrt()->void override;
		auto virtual executeRT()->int override;
		auto virtual collectNrt()->void override;

		virtual ~Recover();
		explicit Recover(const std::string &name = "recover_plan");
		ARIS_REGISTER_TYPE(Recover);
		ARIS_DECLARE_BIG_FOUR(Recover);
	};
	/// \brief 实时循环内休息指定时间
	/// 
	///
	/// ### 参数定义 ###
	/// 指定休息时间，单位是count（1ms），默认为1000：
	/// + 休息5000ms：“sl -c=5000” 或 “sl --count=5000”
	class Sleep : public Plan
	{
	public:
		auto virtual prepareNrt()->void override;
		auto virtual executeRT()->int override;

		virtual ~Sleep();
		explicit Sleep(const std::string &name = "sleep_plan");
		ARIS_REGISTER_TYPE(Sleep);
		ARIS_DECLARE_BIG_FOUR(Sleep);

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};
	/// \brief 打印机器人当前所有轴的位置
	/// 
	/// 
	class Show :public Plan
	{
	public:
		auto virtual prepareNrt()->void override;
		auto virtual executeRT()->int override;

		explicit Show(const std::string &name = "show");
		ARIS_REGISTER_TYPE(Show);
		ARIS_DECLARE_BIG_FOUR(Show);
	};
	/// \brief 将机器人的某根或全部轴移动到指定位置。
	/// 
	/// ### 参数定义 ###
	///
	/// 指定电机，默认指定所有电机：
	/// + 指定所有电机：“mvaj -a” 或 “mvaj --all” 或 “mvaj”
	/// + 按照绝对地址（absID），例如绝对的 0 号电机：“mvaj -m=0” 或 “mvaj --motion_id=0”
	/// + 按照物理地址（phyID），例如物理的 2 号电机：“mvaj -p=2” 或 “mvaj --physical_id=2”
	/// + 按照从站地址（slaID），例如 5 号从站：“mvaj -s=5” 或 “mvaj --slave_id=5”
	/// 
	/// 指定目标位置，单位和电机定义有关，一般是 m （直线电机）或 rad （转动电机），默认为0.0
	/// + 指定单个电机的位置，例如指定 0 号电机走到 1.5 处：“mvaj -m=0 --pos=1.5”
	/// + 指定所有电机的位置，例如：“mvaj -a --pos={1.5,1.2,1.5,1.3,1.2,0.6}”
	///
	/// 指定目标速度，单位一般是 m/s 或 rad/s ，应永远为正数，默认为1.0
	/// + 指定单个电机的速度，例如指定 0 号电机走到 1.5 处，速度为 0.5：“mvaj -m=0 --pos=1.5 --vel=0.5”
	/// + 指定所有电机的位置，例如：“mvaj -a --pos={1.5,1.2,1.5,1.3,1.2,0.6} --vel={1.5,1.2,1.5,1.3,1.2,0.6}”
	///
	/// 指定目标加速度，单位一般是 m/s^2 或 rad/s^2 ，应永远为正数，默认为1.0
	/// + 指定单个电机的加速度，例如指定 0 号电机走到 1.5 处，速度为 0.5 ，加速度为 0.3 ：“mvaj -m=0 --pos=1.5 --vel=0.5 --acc=0.3”
	/// + 指定所有电机的加速度，例如：“mvaj -a --acc={1.5,1.2,1.5,1.3,1.2,0.6}”
	///
	/// 指定目标减速度，单位一般是 m/s^2 或 rad/s^2 ，应永远为正数，默认为1.0
	/// + 指定单个电机的加速度，例如指定 0 号电机走到 1.5 处，速度为 0.5 ，加速度为 0.3 ，减速度为 0.2 ：“mvaj -m=0 --pos=1.5 --vel=0.5 --acc=0.3 --dec=0.2”
	/// + 指定所有电机的加速度，例如：“mvaj -a --dec={1.5,1.2,1.5,1.3,1.2,0.6}”
	///
	class MoveAbsJ :public Plan
	{
	public:
		auto virtual prepareNrt()->void override;
		auto virtual executeRT()->int override;

		virtual ~MoveAbsJ();
		explicit MoveAbsJ(const std::string &name = "move_abs_j");
		ARIS_REGISTER_TYPE(MoveAbsJ);
		ARIS_DECLARE_BIG_FOUR(MoveAbsJ);

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};
	/// \brief 将机器人从轴空间移动到某个位姿。
	/// 
	/// 
	/// ### 参数定义 ###
	///
	/// 指定目标位姿，可以使用以下参数：
	/// + 位置与四元数，例如位置 xyz 为[0 , 0.5 , 1.1]，姿态为原始姿态[0 , 0 , 0 , 1]：“mvj --pe={0,0.5,1.1,0,0,0,1}”
	/// + 位姿矩阵，位姿仍然如上：“mvj --pm={1,0,0,0,0,1,0,0.5,0,0,1,1.1,0,0,0,1}”
	/// + 位置与欧拉角，位置姿态仍然如上：“mvj --pe={0,0.5,1.1,0,0,0}”
	/// 
	/// 此外，还可以指定位置和角度的单位，长度单位默认为 m ，角度默认为 rad ：
	/// + 指定位置单位，例如将单位设置为 m （米）：“mvj --pq={0,0.5,1.1,0,0,0,1} --pos_unit=m”
	/// + 指定角度单位，例如将单位设置为 rad ：“mvj --pe={0,0.5,1.1,0,0,0} --ori_unit=rad”
	///
	/// 还可以指定欧拉角的种类，可以是 321 313 123 212 ... 等任意类型的欧拉角，默认为 321 的欧拉角
	/// + 指定欧拉角为 123 的，“mvj --pe={0,0.5,1.1,0,0,0} --ori_unit=rad --eul_type=321”
	///
	/// 指定关节速度，单位一般是 m/s 或 rad/s ，应永远为正数，默认为0.1
	/// + 指定所有电机的速度都为0.5：“mvj --pe={0,0.5,1.1,0,0,0} --joint_vel=0.5”
	/// + 指定所有电机的速度：“mvj --pe={0,0.5,1.1,0,0,0} --joint_vel={0.2,0.2,0.2,0.3,0.3,0.3}”
	///
	/// 指定关节加速度，单位一般是 m/s^2 或 rad/s^2 ，应永远为正数，默认为0.1
	/// + 指定所有电机的加速度都为0.3：“mvj --pe={0,0.5,1.1,0,0,0} --joint_vel=0.5 --joint_acc=0.3”
	/// + 指定所有电机的加速度：“mvj --pe={0,0.5,1.1,0,0,0} --joint_vel=0.5 --joint_acc={0.2,0.2,0.2,0.3,0.3,0.3}”
	///
	/// 指定关节减速度，单位一般是 m/s^2 或 rad/s^2 ，应永远为正数，默认为0.1
	/// + 指定所有电机的加速度都为0.3：“mvj --pe={0,0.5,1.1,0,0,0} --joint_vel=0.5 --joint_dec=0.3”
	/// + 指定所有电机的加速度：“mvj --pe={0,0.5,1.1,0,0,0} --joint_vel=0.5 --joint_dec={0.2,0.2,0.2,0.3,0.3,0.3}”
	///
	class MoveJ : public Plan
	{
	public:
		auto virtual prepareNrt()->void override;
		auto virtual executeRT()->int override;

		virtual ~MoveJ();
		explicit MoveJ(const std::string &name = "move_j");
		ARIS_REGISTER_TYPE(MoveJ);
		ARIS_DECLARE_BIG_FOUR(MoveJ);

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};
	/// \brief 将机器人从笛卡尔空间线性移动到某个位姿。
	/// 
	/// 
	/// ### 参数定义 ###
	///
	/// 指定目标位姿，可以使用以下参数：
	/// + 位置与四元数，例如位置 xyz 为[0 , 0.5 , 1.1]，姿态为原始姿态[0 , 0 , 0 , 1]：“mvl --pq={0,0.5,1.1,0,0,0,1}”
	/// + 位姿矩阵，位姿仍然如上：“mvl --pm={1,0,0,0,0,1,0,0.5,0,0,1,1.1,0,0,0,1}”
	/// + 位置与欧拉角，位置姿态仍然如上：“mvl --pe={0,0.5,1.1,0,0,0}”
	///
	/// 此外，还可以指定位置和角度的单位，长度单位默认为 m ，角度默认为 rad ：
	/// + 指定位置单位，例如将单位设置为 m （米）：“mvl --pq={0,0.5,1.1,0,0,0,1} --pos_unit=m”
	/// + 指定角度单位，例如将单位设置为 rad ：“mvl --pe={0,0.5,1.1,0,0,0} --ori_unit=rad”
	///
	/// 还可以指定欧拉角的种类，可以是 321 313 123 212 ... 等任意类型的欧拉角，默认为 321 的欧拉角
	/// + 指定欧拉角为 123 的，“mvl --pe={0,0.5,1.1,0,0,0} --ori_unit=rad --eul_type=321”
	///
	/// 指定目标速度以及角速度，单位一般是 m/s 或 rad/s ，应永远为正数，默认为0.1
	/// + 指定末端的速度和角速度，例如指定末端速度为 0.5，角速度为 0.3：“mvl --pe={0.1,1.2,0,0,0,0} --vel=0.5 --angular_vel=0.3”
	///
	/// 指定目标加速度以及角加速度，单位一般是 m/s^2 或 rad/s^2 ，应永远为正数，默认为0.1
	/// + 指定末端的加速度和角加速度，例如指定末端加速度为 0.5，角加速度为 0.3：“mvl --pe={0.1,1.2,0,0,0,0} --acc=0.5 --angular_acc=0.3”
	///
	/// 指定目标减速度以及角减速度，单位一般是 m/s^2 或 rad/s^2 ，应永远为正数，默认为0.1
	/// + 指定末端的减速度和角减速度，例如指定末端减速度为 0.5，角减速度为 0.3：“mvl --pe={0.1,1.2,0,0,0,0} --dec=0.5 --angular_dec=0.3”
	class MoveL : public Plan
	{
	public:
		auto virtual prepareNrt()->void override;
		auto virtual executeRT()->int override;

		virtual ~MoveL();
		explicit MoveL(const std::string &name = "move_l");
		ARIS_REGISTER_TYPE(MoveL);
		ARIS_DECLARE_BIG_FOUR(MoveL);

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};
	/// \brief 让机器人自动运行，随时可以改变其目标位姿。
	/// 
	/// 典型流程为：
	/// 1. 开启自动运行模式
	/// 2. 不断刷入目标位姿
	/// 3. 终止自动运行模式
	/// 
	/// ### 参数定义 ###
	///
	/// #### 开启自动运行模式 ####
	/// 
	/// 此时至少要包含“start”参数，例如：“am --start”
	///
	/// 指定起始的目标位姿，移动到目标的最大速度、加速度和减速度，其中旋转项必须用欧拉角及其导数来表达：
	/// + 使用123的欧拉角设置起始目标位姿：“am --start --eul_type=123 --init_pe={0,0,0.5,0,PI/3,0} --init_ve={0.1,0.1,0.1,0.2,0.2,0.2} --init_ae={0.1,0.1,0.1,0.2,0.2,0.2}  --init_de={0.1,0.1,0.1,0.2,0.2,0.2}”
	///
	/// 此外，还可以指定最大的位姿和最小的位姿 ：
	/// + 指定最大和最小位姿，“am --start --max_pe={1,1,1,PI/3,PI/2,pi/4} --min_pe={-1,-1,-1,0,-PI/2,-pi/4}”
	///
	/// #### 终止自动运行模式 ####
	/// 
	/// 指令：“am --stop”
	///
	/// #### 在自动运行模式中刷入目标位姿 ####
	///
	/// 指定目标位姿，移动到目标的最大速度、加速度和减速度，其中旋转项必须用欧拉角及其导数来表达：
	/// + 使用123的欧拉角设置起始目标位姿：“am --pe={0,0,0.5,0,PI/3,0} --ve={0.1,0.1,0.1,0.2,0.2,0.2} --ae={0.1,0.1,0.1,0.2,0.2,0.2}  --de={0.1,0.1,0.1,0.2,0.2,0.2}”
	///
	///
	class AutoMove :public Plan
	{
	public:
		auto virtual prepareNrt()->void override;
		auto virtual executeRT()->int override;
		auto virtual collectNrt()->void override;

		virtual ~AutoMove();
		explicit AutoMove(const std::string &name = "auto_move");
		ARIS_REGISTER_TYPE(AutoMove);
		ARIS_DECLARE_BIG_FOUR(AutoMove);

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};
	/// \brief 让机器人手动运行。
	/// 
	/// 典型流程为：
	/// 1. 开启手动运行模式
	/// 2. 不断刷入移动指令
	/// 3. 终止手动运行模式
	/// 
	/// ### 参数定义 ###
	///
	/// #### 开启手动运行模式 ####
	/// 
	/// 此时至少要包含“start”参数，例如：“mm --start”
	///
	/// 笛卡尔空间下手动运行的速度、加速度和减速度，需要用欧拉角的形式来表达：
	/// + 使用123的欧拉角设置起始目标位姿：“mm --start --eul_type=123 --ve={0.1,0.1,0.1,0.2,0.2,0.2} --ae={0.1,0.1,0.1,0.2,0.2,0.2}  --de={0.1,0.1,0.1,0.2,0.2,0.2}”
	///
	/// 此外，还可以指定每次刷入数据，机器人连续运行的周期数，默认为 50 ，既运行 50 ms ：
	/// + 指定最大和最小位姿，“mm --start --increase_count=50”
	///
	/// #### 终止手动运行模式 ####
	/// 
	/// 指令：“mm --stop”
	///
	/// #### 在手动运行模式中刷入新的运动指令 ####
	///
	/// 这种指令请尽量用较高频率往下刷，建议小于开启时所设置的 increase_count 参数。
	/// 
	/// 指定x,y,z,a,b,c这六项，设置为1时正向动，-1时为反向动：
	/// + 手动让机器人延 z 运动：“mm --z=1”
	/// + 手动让机器人延 x 运动，同时绕第一根轴的负方向转：“mm --x=1 --a=-1”
	///
	class ManualMove :public Plan
	{
	public:
		auto virtual prepareNrt()->void override;
		auto virtual executeRT()->int override;
		auto virtual collectNrt()->void override;

		virtual ~ManualMove();
		explicit ManualMove(const std::string &name = "manual_move");
		ARIS_REGISTER_TYPE(ManualMove);
		ARIS_DECLARE_BIG_FOUR(ManualMove);

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};

	class GetXml :public Plan
	{
	public:
		auto virtual prepareNrt()->void override;

		virtual ~GetXml();
		explicit GetXml(const std::string &name = "get_xml");
		ARIS_REGISTER_TYPE(GetXml);
		ARIS_DEFINE_BIG_FOUR(GetXml);
	};
	class SetXml :public Plan
	{
	public:
		auto virtual prepareNrt()->void override;

		virtual ~SetXml();
		explicit SetXml(const std::string &name = "set_xml");
		ARIS_REGISTER_TYPE(SetXml);
		ARIS_DEFINE_BIG_FOUR(SetXml);
	};
	class Start :public Plan
	{
	public:
		auto virtual prepareNrt()->void override;

		virtual ~Start();
		explicit Start(const std::string &name = "start");
		ARIS_REGISTER_TYPE(plan::Start);
		ARIS_DEFINE_BIG_FOUR(Start);
	};
	class Stop :public Plan
	{
	public:
		auto virtual prepareNrt()->void override;

		virtual ~Stop();
		explicit Stop(const std::string &name = "stop");
		ARIS_REGISTER_TYPE(Stop);
		ARIS_DEFINE_BIG_FOUR(Stop);
	};

	class RemoveFile : public aris::plan::Plan
	{
	public:
		auto virtual prepareNrt()->void override;
		
		virtual ~RemoveFile();
		explicit RemoveFile(const std::string &name = "rm_file");
		ARIS_REGISTER_TYPE(RemoveFile);
		ARIS_DEFINE_BIG_FOUR(RemoveFile);
	};

	class UniversalPlan :public Plan
	{
	public:
		using PrepareFunc = std::function<void(Plan *plan)>;
		using ExecuteFunc = std::function<int(Plan *plan)>;
		using CollectFunc = std::function<void(Plan *plan)>;

		auto virtual prepareNrt()->void override;
		auto virtual executeRT()->int override;
		auto virtual collectNrt()->void override;
		auto virtual setprepareFunc(PrepareFunc func)->void;
		auto virtual setExecuteFunc(ExecuteFunc func)->void;
		auto virtual setCollectFunc(CollectFunc func)->void;

		virtual ~UniversalPlan();
		explicit UniversalPlan(const std::string &name = "universal_plan", PrepareFunc prepare_func = nullptr, ExecuteFunc execute_func = nullptr, CollectFunc collect_func = nullptr, const std::string & cmd_xml_str = "<universal_plan/>");
		ARIS_REGISTER_TYPE(UniversalPlan);
		ARIS_DECLARE_BIG_FOUR(UniversalPlan);

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};

	class MoveSeries :public Plan
	{
	public:
		auto virtual prepareNrt()->void override;
		auto virtual executeRT()->int override;

		virtual ~MoveSeries();
		explicit MoveSeries(const std::string &name = "move_series");
		ARIS_REGISTER_TYPE(MoveSeries);
		ARIS_DEFINE_BIG_FOUR(MoveSeries);
	};




}

#endif