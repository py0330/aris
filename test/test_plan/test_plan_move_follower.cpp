#include <iostream>
#include <aris/core/core.hpp>
#include <aris/plan/plan.hpp>
#include <aris/robot/rokae.hpp>

#include <random>
using namespace aris::plan;

auto test_move_follower_1()->void {
	
	// 设置 follower 的 dt acc v 等。
	aris::plan::MoveFollower follower;

	double dt = 0.002;
	follower.setMaxA(12.0);
	follower.setMaxV(2.5);
	follower.setDt(dt);

	// 设置起始位姿与速度 //
	double begin_follow_pm[16]{
		1,0,0,-0.15,
		0,1,0,0.01,
		0,0,1,0.05,
		0,0,0,1
	};
	double begin_follow_v[6]{ 0,0,0,0,0,0 };
	follower.setFollowPm(begin_follow_pm);
	follower.setFollowVa(begin_follow_v);


	// 不断更新追踪的目标，这个应该在实时循环调用 //
	double target_pm[16]{
		1,0,0,-0.2,
		0,1,0,0.02,
		0,0,1,0.3,
		0,0,0,1
	};
	double target_v[6]{ 0.3,0.01,-0.4 ,0,0,0 };
	for (int i = 0; i < 1000; ++i) {
		// 目标位置是根据速度算出来的，也可以根据传感器等信息更新 //
		target_pm[3] += target_v[0] * dt;
		target_pm[7] += target_v[1] * dt;
		target_pm[11] += target_v[2] * dt;

		// 设置追踪目标的位姿与速度 //
		follower.setTargetPm(target_pm);
		follower.setTargetVa(target_v);

		// 得到追踪的结果 //
		double follow_pm[16], follow_v[6];
		follower.moveDtAndGetResult(follow_pm, follow_v);

		// 打印 //
		std::cout << i <<" : " << follower.estimateLeftT() << "  ---   " << follow_pm[3] << "  " << follow_pm[7] << "  " << follow_pm[11] << std::endl;
	}


	for (int i = 0; i < 1000; ++i) {
		// 目标位置是根据速度算出来的，也可以根据传感器等信息更新 //
		target_pm[3] = 0.5;
		target_pm[7] = 0.6;
		target_pm[11] = 0.8;
		target_v[0] = 0.0;
		target_v[1] = 0.0;
		target_v[2] = 0.0;

		// 设置追踪目标的位姿与速度 //
		follower.setTargetPm(target_pm);
		follower.setTargetVa(target_v);

		// 得到追踪的结果 //
		double follow_pm[16], follow_v[6];
		follower.moveDtAndGetResult(follow_pm, follow_v);

		// 打印 //
		std::cout << i << " : " << follower.estimateLeftT() << "  ---   " << follow_pm[3] << "  " << follow_pm[7] << "  " << follow_pm[11] << std::endl;
	}


	

}

auto test_move_follower_2() -> void {
	/////////////////////////////////// PART 1 参数设置 /////////////////////////////////////////
	// 一共包含3个过程 FOLLOW_WOBJ、PICKING、LEAVE_WOBJ
	// FOLLOW_WOBJ 是把 tool 移动到 wobj（在动） 处
	// PICKING     是在 wobj 中进行规划，进行抓取
	// LEAVE_WOBJ  是把 tool 重新移动到地面坐标系下
	// 
	
	// 工件的起始位置与速度
	double wobj_begin_pe[6]{ -0.15,0.01,0.05,0,0,0 }; // 工件位姿，这个起始应该位于工件正上方 0.1 处
	double wobj_begin_va[6]{ 0,0.2,0.3,0,0,0 };       // 工件速度，位于传送带，速度不为0

	// 起始点位，FOLLOW_WOBJ 过程的起始状态
	double tool_pe_begin[6]{ 0.1,0.1,0.2,0,0,0 };     // 工具位姿，相对于地面
	double tool_va_begin[6]{ 0,0,0,0,0,0 };

	// 飞抓的3组点位，相对于工件
	double tool_wrt_wobj_pe0[6]{ 0,0,0 ,0,0,0 };
	double tool_wrt_wobj_pe1[6]{ 0,0,-0.1 ,0,0,0 };   // 机械臂向下抓取
	double tool_wrt_wobj_pe2[6]{ 0,0,0 ,0,0,0 };

	// 结束点位，相对于地面
	double tool_pe_end[6]{ -0.1,0.3,0.3,0,0,0 };     // 相对于地面
	double tool_va_end[6]{ 0,0,0,0,0,0 };

	// 飞抓过程中的速度、加速度、转弯区 //
	const double vel[2]{ 1.5, 2.0 };
	const double acc[2]{ 10, 15 };
	const double jerk[2]{ 80, 90 };
	const double zone[2]{ 0.0, 0.0 };  // 不要转弯区

	////////////////////////////////////////// PART 2 设置 追踪器 ///////////////////////
	// 设置 follower 的 dt acc v 等。
	aris::plan::MoveFollower follower;
	double dt = 0.002;
	follower.setMaxA(12.0);
	follower.setMaxV(2.5);
	follower.setDt(dt);

	// 更新追踪器的起始位置（tool的位置），在后续的实时循环中设置被追踪的位置（wobj的位置） //
	double pm[16];
	aris::dynamic::s_pe2pm(tool_pe_begin, pm, "321");
	follower.setFollowPm(pm);
	follower.setFollowVa(tool_va_begin);

	////////////////////////////////////////// PART 3 设置 规划器 ///////////////////////
	aris::plan::TrajectoryGenerator tg;    // 对应3个过程，走到工件上方、抓件、走回地面
	tg.setEeTypes({ aris::dynamic::EEType::PE321, });
	tg.setDt(dt);

	// 插入 picking 过程中的点位
	tg.insertInitPos(1, tool_wrt_wobj_pe0);
	tg.insertLinePos(2, tool_wrt_wobj_pe1, vel, acc, jerk, zone);
	tg.insertLinePos(3, tool_wrt_wobj_pe2, vel, acc, jerk, zone);


	////////////////////////////////////////// PART 4 进入实时循环 ///////////////////////
	enum PICKING_STATE {
		FOLLOW_WOBJ,
		PICKING,
		LEAVE_WOBJ
	};
	PICKING_STATE state = FOLLOW_WOBJ;
	
	// 起始的 wobj（即传送带） 的位置
	double wobj_pe[6];
	aris::dynamic::s_vc(6, wobj_begin_pe, wobj_pe);

	double tool_pm[16];

	// 以下应该放到实时循环 //
	for (int i=0;i<1000;++i) {
		// 在跟踪 wobj 和 抓取的过程中，follower的目标都是 wobj
		if (state == FOLLOW_WOBJ || state == PICKING) {
			// 更新传送带速度与位置
			double wobj_va[6];
			aris::dynamic::s_vc(6, wobj_begin_va, wobj_va);// 速度起始可以时变
			wobj_pe[0] += dt * wobj_va[0];
			wobj_pe[1] += dt * wobj_va[1];
			wobj_pe[2] += dt * wobj_va[2];

			// 设置追踪目标的位姿与速度 //
			aris::dynamic::s_pe2pm(wobj_pe, pm, "321");
			follower.setTargetPm(pm);
			follower.setTargetVa(wobj_va);

			// 得到追踪的结果 //
			double follow_pm[16], follow_v[6];
			follower.moveDtAndGetResult(follow_pm, follow_v);

			// 在追踪过程中，最终的 tool_pm 就是上述 follow_pm
			if (state == FOLLOW_WOBJ) {
				aris::dynamic::s_vc(16, follow_pm, tool_pm);

				// 在这里设置model，求反解等 //
				///////////////////////////////////////////////////
				// model.setOutputPos(tool_pm);
				///////////////////////////////////////////////////

				// 已经跟踪到位，改为 picking //
				if (follower.estimateLeftT() < 1e-10)
					state = PICKING;
			}
			// 在 picking 过程中，还需迭加规划器的位置 //
			else {
				double pick_pe[6];
				if (tg.getEePosAndMoveDt(pick_pe) == 0) 
					state = LEAVE_WOBJ;
				

				// 将 pick_pe 转到 wobj 的坐标系中 //
				double tool_pe[6];
				aris::dynamic::s_pe2pe(follow_pm, pick_pe, tool_pe);

				
				aris::dynamic::s_pe2pm(tool_pe, tool_pm, "321");
				// 在这里设置model，求反解等 //
				///////////////////////////////////////////////////
				// model.setOutputPos(tool_pe);
				///////////////////////////////////////////////////
			}

		}
		// 在回位的过程中，目标位置是 tool_pe_end
		else {
			// 重新设置 follower的目标，现在追踪地面下固定的坐标系
			aris::dynamic::s_pe2pm(tool_pe_end, pm, "321");
			follower.setTargetPm(pm);
			follower.setTargetVa(tool_va_end);

			// 得到追踪的结果 //
			double follow_pm[16], follow_v[6];
			follower.moveDtAndGetResult(follow_pm, follow_v);
			
			aris::dynamic::s_vc(16, follow_pm, tool_pm);

			// 在这里设置model，求反解等 //
			///////////////////////////////////////////////////
			// model.setOutputPos(tool_pm);
			///////////////////////////////////////////////////

			// 追踪结束 //
			if (follower.estimateLeftT() < 1e-10)
				break;
		}



		std::cout << state <<"-" << i << " : " << follower.estimateLeftT() << "  ---   " << tool_pm[3] << "  " << tool_pm[7] << "  " << tool_pm[11] << std::endl;
	}
}


void test_move_follower(){
	std::cout << std::endl << "-----------------test processor---------------------" << std::endl;

	test_move_follower_2();

	std::cout << "-----------------test processor finished------------" << std::endl << std::endl;
}

