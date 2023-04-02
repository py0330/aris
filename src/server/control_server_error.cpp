#include <cstring>
#include <thread>
#include <algorithm>
#include <memory>
#include <cinttypes>
#include <queue>

#include <aris/core/core.hpp>
#include <aris/control/control.hpp>

#include "aris/ext/json.hpp"
#include "aris/ext/fifo_map.hpp"
#include "aris/server/control_server_error.hpp"
#include "aris/server/api.hpp"

namespace aris::server{
	struct PVC { double p; double v; double c; };
	
	struct ControlServerErrorChecker::Imp {
		std::vector<char> mem_pool_;
		PVC* last_pvc_, * last_last_pvc_;
		aris::server::ControlServer* cs_;
	};
	auto ControlServerErrorChecker::init(aris::server::ControlServer *cs)->void {
		imp_->cs_ = cs;


		Size mem_pool_size = 0;
		
		core::allocMem(mem_pool_size, imp_->last_pvc_, cs->controller().motorPool().size());
		core::allocMem(mem_pool_size, imp_->last_last_pvc_, cs->controller().motorPool().size());

		imp_->mem_pool_.resize(mem_pool_size, 0);

		imp_->last_pvc_ = core::getMem(imp_->mem_pool_.data(), imp_->last_pvc_);
		imp_->last_last_pvc_ = core::getMem(imp_->mem_pool_.data(), imp_->last_last_pvc_);
	}
	auto ControlServerErrorChecker::checkError(std::int64_t count, const std::uint64_t* mot_options, char* error_msg)->std::int32_t {
		
		int error_code = aris::plan::Plan::SUCCESS;

		auto controller = &imp_->cs_->controller();
		double dt = imp_->cs_->master().samplePeriodNs() / 1e9;

		// 检查规划的指令是否合理（包括电机是否已经跟随上） //
		for (std::size_t i = 0; i < controller->motorPool().size(); ++i) {
			const auto& cm = controller->motorPool()[i];
			const auto& ld = imp_->last_pvc_[i];
			const auto& lld = imp_->last_last_pvc_[i];
			const auto option = mot_options[i];

			auto display_id = i + 1;

			// 检查使能 //
			if (!(option & aris::plan::Plan::NOT_CHECK_ENABLE)
				&& ((cm.statusWord() & 0x6f) != 0x27))
			{
				error_code = aris::plan::Plan::MOTION_NOT_ENABLED;
				sprintf(error_msg,
					aris::core::currentLanguage() == (int)aris::core::Language::kSimplifiedChinese ?
					u8"电机 %zd 没在使能模式，当前周期: %zd\n" :
					u8"Motor %zd is not in OPERATION_ENABLE mode in count %zd\n",
					display_id, count);
				return error_code;
			}

			// 使能时才检查 //
			if (cm.isEnabled()) {
				switch (cm.modeOfOperation()) {
				case 6:break;
				case 8: {
					// check pos infinite //
					if (!std::isfinite(cm.targetPos())) {
						error_code = aris::plan::Plan::MOTION_POS_INFINITE;
						sprintf(error_msg,
							aris::core::currentLanguage() == (int)aris::core::Language::kSimplifiedChinese ?
							u8"电机 %zu 目标位置不是有效值，当前周期: %zu\t目标位置: %f\n" :
							u8"Motor %zu target position is INFINITE in count %zu:\nvalue: %f\n",
							display_id, count, cm.targetPos());
						return error_code;
					}

					// check pos max //
					if (!(option & aris::plan::Plan::NOT_CHECK_POS_MAX)
						&& (cm.targetPos() > cm.maxPos())
						&& (cm.targetPos() > ld.p))
					{
						error_code = aris::plan::Plan::MOTION_POS_BEYOND_MAX;
						sprintf(error_msg,
							aris::core::currentLanguage() == (int)aris::core::Language::kSimplifiedChinese ?
							u8"电机 %zu 超出正限位，当前周期: %zu\t允许最大位置: %f\t目标位置: %f\n" :
							u8"Motor %zu target position beyond MAX in count %zu:\nmax: %f\tnow: %f\n",
							display_id, count, cm.maxPos(), cm.targetPos());
						return error_code;
					}

					// check pos min //
					if (!(option & aris::plan::Plan::NOT_CHECK_POS_MIN)
						&& (cm.targetPos() < cm.minPos())
						&& (cm.targetPos() < ld.p))
					{
						error_code = aris::plan::Plan::MOTION_POS_BEYOND_MIN;
						sprintf(error_msg,
							aris::core::currentLanguage() == (int)aris::core::Language::kSimplifiedChinese ?
							u8"电机 %zu 超出负限位，当前周期: %zu\t允许最小值: %f\t目标位置: %f\n" :
							u8"Motor %zu target position beyond MIN in count %zu:\nmin: %f\tnow: %f\n",
							display_id, count, cm.minPos(), cm.targetPos());
						return error_code;
					}

					// check pos continuous //
					if (!(option & aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS)
						&& ((cm.targetPos() - ld.p) > dt * cm.maxVel() || (cm.targetPos() - ld.p) < dt * cm.minVel()))
					{
						error_code = aris::plan::Plan::MOTION_POS_NOT_CONTINUOUS;
						sprintf(error_msg,
							aris::core::currentLanguage() == (int)aris::core::Language::kSimplifiedChinese ?
							u8"电机 %zu 速度过大，当前周期: %zu\t上次位置: %f\t本次位置: %f\n" :
							u8"Motor %zu target position NOT CONTINUOUS in count %zu:\nlast: %f\tnow: %f\n",
							display_id, count, ld.p, cm.targetPos());
						return error_code;
					}

					// check pos continuous second order //
					if (!(option & aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER)
						&& ((cm.targetPos() + lld.p - 2 * ld.p) > dt * dt * cm.maxAcc() || (cm.targetPos() + lld.p - 2 * ld.p) < dt * dt * cm.minAcc()))
					{
						error_code = aris::plan::Plan::MOTION_POS_NOT_CONTINUOUS_SECOND_ORDER;
						sprintf(error_msg,
							aris::core::currentLanguage() == (int)aris::core::Language::kSimplifiedChinese ?
							u8"电机 %zu 加速度过大，当前Count %zu:\n上上次位置: %f\t上次位置: %f\t本次位置: %f\n" :
							u8"Motor %zu target position NOT SECOND CONTINUOUS in count %zu:\nlast last: %f\tlast: %f\tnow: %f\n",
							display_id, count, lld.p, ld.p, cm.targetPos());
						return error_code;
					}

					// check pos following error //
					if (!(option & aris::plan::Plan::NOT_CHECK_POS_FOLLOWING_ERROR)
						&& (std::abs(cm.targetPos() - cm.actualPos()) > cm.maxPosFollowingError()))
					{
						error_code = aris::plan::Plan::MOTION_POS_FOLLOWING_ERROR;
						sprintf(error_msg,
							aris::core::currentLanguage() == (int)aris::core::Language::kSimplifiedChinese ?
							u8"电机 %zu 位置跟随误差过大，当前Count %zu:\n实际位置: %f\t目标位置: %f\n" :
							u8"Motion %zu target position has FOLLOW ERROR in count %zu:\nactual: %f\ttarget: %f\n",
							display_id, count, cm.actualPos(), cm.targetPos());
						return error_code;
					}

					break;
				}
				case 9: {
					// check vel infinite //
					if (!std::isfinite(cm.targetVel())) {
						error_code = aris::plan::Plan::MOTION_VEL_INFINITE;
						sprintf(error_msg,
							aris::core::currentLanguage() == (int)aris::core::Language::kSimplifiedChinese ?
							u8"电机 %zu 目标速度不是有效值，当前Count %zu:\n目标速度: %f\n" :
							u8"Motion %zu target velocity is INFINITE in count %zu:\nvalue: %f\n",
							display_id, count, cm.targetVel());
						return error_code;
					}

					// check vel max //
					if (!(option & aris::plan::Plan::NOT_CHECK_VEL_MAX)
						&& (cm.targetVel() > cm.maxVel()))
					{
						error_code = aris::plan::Plan::MOTION_VEL_BEYOND_MAX;
						sprintf(error_msg,
							aris::core::currentLanguage() == (int)aris::core::Language::kSimplifiedChinese ?
							u8"电机 %zu 速度超限，当前Count %zu:\n允许最大速度: %f\t目标速度: %f\n" :
							u8"Motion %zu target velocity beyond MAX in count %zu:\nmax: %f\tnow: %f\n",
							display_id, count, cm.maxVel(), cm.targetVel());
						return error_code;
					}

					// check vel min //
					if (!(option & aris::plan::Plan::NOT_CHECK_VEL_MIN)
						&& (cm.targetVel() < cm.minVel()))
					{
						error_code = aris::plan::Plan::MOTION_VEL_BEYOND_MIN;
						sprintf(error_msg,
							aris::core::currentLanguage() == (int)aris::core::Language::kSimplifiedChinese ?
							u8"电机 %zu 速度超限，当前Count %zu:\n允许最小速度: %f\t目标速度: %f\n" :
							u8"Motion %zu target velocity beyond MIN in count %zu:\nmin: %f\tnow: %f\n",
							display_id, count, cm.minVel(), cm.targetVel());
						return error_code;
					}

					// check vel continuous //
					if (!(option & aris::plan::Plan::NOT_CHECK_VEL_CONTINUOUS)
						&& ((cm.targetVel() - ld.v) > dt * cm.maxAcc() || (cm.targetVel() - ld.v) < dt * cm.minAcc()))
					{
						error_code = aris::plan::Plan::MOTION_VEL_NOT_CONTINUOUS;
						sprintf(error_msg,
							aris::core::currentLanguage() == (int)aris::core::Language::kSimplifiedChinese ?
							u8"电机 %zu 加速度超限，当前Count %zu:\n上次速度: %f\t本次速度: %f\n" :
							u8"Motion %zu target velocity NOT CONTINUOUS in count %zu:\nlast: %f\tnow: %f\n", display_id, count, ld.v, cm.targetVel());
						return error_code;
					}

					// check vel following error //
					if (!(option & aris::plan::Plan::NOT_CHECK_VEL_FOLLOWING_ERROR)
						&& (std::abs(cm.targetVel() - cm.actualVel()) > cm.maxVelFollowingError()))
					{
						error_code = aris::plan::Plan::MOTION_VEL_FOLLOWING_ERROR;
						sprintf(error_msg,
							aris::core::currentLanguage() == (int)aris::core::Language::kSimplifiedChinese ?
							u8"电机 %zu 速度跟随误差过大，当前Count %zu:\n实际速度: %f\t目标速度: %f\n" :
							u8"Motion %zu target velocity has FOLLOW ERROR in count %zu:\nactual: %f\ttarget: %f\n", display_id, count, cm.actualVel(), cm.targetVel());
						return error_code;
					}

					break;
				}
				case 10: {
					// check pos max //
					if (!(option & aris::plan::Plan::NOT_CHECK_POS_MAX)
						&& (cm.actualPos() > cm.maxPos()))
					{
						error_code = aris::plan::Plan::MOTION_POS_BEYOND_MAX;
						sprintf(error_msg,
							aris::core::currentLanguage() == (int)aris::core::Language::kSimplifiedChinese ?
							u8"电机 %zu 实际位置超出最大值，当前Count %zu:\n允许最大位置: %f\t实际位置: %f\n" :
							u8"Motion %zu target position beyond MAX in count %zu:\nmax: %f\tnow: %f\n", display_id, count, cm.maxPos(), cm.targetPos());
						return error_code;
					}

					// check pos min //
					if (!(option & aris::plan::Plan::NOT_CHECK_POS_MIN)
						&& (cm.actualPos() < cm.minPos()))
					{
						error_code = aris::plan::Plan::MOTION_POS_BEYOND_MIN;
						sprintf(error_msg,
							aris::core::currentLanguage() == (int)aris::core::Language::kSimplifiedChinese ?
							u8"电机 %zu 实际位置超出最小值，当前Count %zu:\n允许最小位置: %f\t实际位置: %f\n" :
							u8"Motion %zu target position beyond MIN in count %zu:\nmin: %f\tnow: %f\n",
							display_id, count, cm.minPos(), cm.targetPos());
						return error_code;
					}

					// check vel max //
					if (!(option & aris::plan::Plan::NOT_CHECK_VEL_MAX)
						&& (cm.actualVel() > cm.maxVel()))
					{
						error_code = aris::plan::Plan::MOTION_VEL_BEYOND_MAX;
						sprintf(error_msg,
							aris::core::currentLanguage() == (int)aris::core::Language::kSimplifiedChinese ?
							u8"电机 %zu 实际速度超出最大值，当前Count %zu:\n允许最大速度: %f\t实际速度: %f\n" :
							u8"Motion %zu target velocity beyond MAX in count %zu:\nmax: %f\tnow: %f\n",
							display_id, count, cm.maxVel(), cm.actualVel());
						return error_code;
					}

					// check vel min //
					if (!(option & aris::plan::Plan::NOT_CHECK_VEL_MIN)
						&& (cm.actualVel() < cm.minVel()))
					{
						error_code = aris::plan::Plan::MOTION_VEL_BEYOND_MIN;
						sprintf(error_msg,
							aris::core::currentLanguage() == (int)aris::core::Language::kSimplifiedChinese ?
							u8"电机 %zu 实际速度超出最小值，当前Count %zu:\n允许最小速度: %f\t实际速度: %f\n" :
							u8"Motion %zu target velocity beyond MIN in count %zu:\nmin: %f\tnow: %f\n", display_id, count, cm.minVel(), cm.actualVel());
						return error_code;
					}

					// check vel continuous //
					if (!(option & aris::plan::Plan::NOT_CHECK_VEL_CONTINUOUS)
						&& ((cm.actualVel() - ld.v) > dt * cm.maxAcc() || (cm.actualVel() - ld.v) < dt * cm.minAcc()))
					{
						error_code = aris::plan::Plan::MOTION_VEL_NOT_CONTINUOUS;
						sprintf(error_msg,
							aris::core::currentLanguage() == (int)aris::core::Language::kSimplifiedChinese ?
							u8"电机 %zu 加速度过大，当前Count %zu:\n上次速度: %f\t本次速度: %f\n" :
							u8"Motion %zu velocity NOT CONTINUOUS in count %zu:\nlast: %f\tnow: %f\n",
							display_id, count, ld.v, cm.actualVel());
						return error_code;
					}
					break;
				}
				default: {
					// invalid mode //
					if (!(option & aris::plan::Plan::NOT_CHECK_MODE)) {
						error_code = aris::plan::Plan::MOTION_INVALID_MODE;
						sprintf(error_msg,
							aris::core::currentLanguage() == (int)aris::core::Language::kSimplifiedChinese ?
							u8"电机 %zu 模式不合法，当前Count %zu:\n模式: %d\n" :
							u8"Motion %zu MODE INVALID in count %zu:\nmode: %d\n",
							display_id, count, cm.modeOfOperation());
						return error_code;
					}
				}
				}
			}
		}


		return 0;

	}
	auto ControlServerErrorChecker::fixError()->std::int32_t {
		auto controller = &imp_->cs_->controller();
		
		// 只要在check里面执行，都说明修复没有结束 //
		std::int32_t fix_finished{ 0 };
		for (std::size_t i = 0; i < controller->motorPool().size(); ++i) {
			// correct
			auto& cm = controller->motorPool().at(i);
			switch (cm.modeOfOperation()) {
			case 1:
			case 8:
				cm.setTargetPos(std::abs(imp_->last_pvc_[i].p - cm.actualPos()) > cm.maxPosFollowingError() ? cm.actualPos() : imp_->last_pvc_[i].p);
				break;
			case 9:
				cm.setTargetVel(0.0);
				break;
			case 10:
				cm.setTargetToq(0.0);
				fix_finished = cm.disable() || fix_finished;
				break;
			default:
				fix_finished = cm.disable() || fix_finished;
				cm.setTargetPos(0.0);
				cm.setTargetVel(0.0);
				cm.setTargetToq(0.0);
			}

			// store correct data
			imp_->last_pvc_[i].p = imp_->last_last_pvc_[i].p = controller->motorPool().at(i).targetPos();
			imp_->last_pvc_[i].v = imp_->last_last_pvc_[i].v = controller->motorPool().at(i).targetVel();
			imp_->last_pvc_[i].c = imp_->last_last_pvc_[i].c = controller->motorPool().at(i).targetToq();
		}

		return fix_finished;
		
		return 0;
	}
	auto ControlServerErrorChecker::storeServerData()->void {
		auto controller_ = &imp_->cs_->controller();
		
		for (std::size_t i = 0; i < controller_->motorPool().size(); ++i) {
			imp_->last_last_pvc_[i].p = controller_->motorPool()[i].targetPos();
			imp_->last_last_pvc_[i].v = controller_->motorPool()[i].targetVel();
			imp_->last_last_pvc_[i].c = controller_->motorPool()[i].targetToq();
		}
		std::swap(imp_->last_pvc_, imp_->last_last_pvc_);
	}
	ControlServerErrorChecker::ControlServerErrorChecker() = default;
	ControlServerErrorChecker::~ControlServerErrorChecker() = default;
	ARIS_DEFINE_BIG_FOUR_CPP_NOEXCEPT(ControlServerErrorChecker);

	struct PVC_Plus { double p; double v; double c; double pm; };
	struct ControlServerErrorCheckerByModel::Imp {
		std::vector<char> mem_pool_;
		PVC_Plus* last_pvc_, * last_last_pvc_;
		aris::server::ControlServer* cs_;
	};
	auto ControlServerErrorCheckerByModel::init(aris::server::ControlServer* cs)->void {
		imp_->cs_ = cs;


		Size mem_pool_size = 0;

		core::allocMem(mem_pool_size, imp_->last_pvc_, cs->controller().motorPool().size());
		core::allocMem(mem_pool_size, imp_->last_last_pvc_, cs->controller().motorPool().size());

		imp_->mem_pool_.resize(mem_pool_size, 0);

		imp_->last_pvc_ = core::getMem(imp_->mem_pool_.data(), imp_->last_pvc_);
		imp_->last_last_pvc_ = core::getMem(imp_->mem_pool_.data(), imp_->last_last_pvc_);
	}
	auto ControlServerErrorCheckerByModel::checkError(std::int64_t count, const std::uint64_t* mot_options, char* error_msg)->std::int32_t {

		int error_code = aris::plan::Plan::SUCCESS;

		auto controller = &imp_->cs_->controller();
		auto model = &imp_->cs_->model();
		double dt = imp_->cs_->master().samplePeriodNs() / 1e9;

		// 检查规划的指令是否合理（包括电机是否已经跟随上） //
		for (std::size_t i = 0; i < controller->motorPool().size(); ++i) {
			const auto& cm = controller->motorPool()[i];
			
			const auto& ld = imp_->last_pvc_[i];
			const auto& lld = imp_->last_last_pvc_[i];
			const auto option = mot_options[i];

			auto input_pos_at_i = model->inputPosAt(i);

			auto display_id = i + 1;

			// 检查使能 //
			if (!(option & aris::plan::Plan::NOT_CHECK_ENABLE)
				&& ((cm.statusWord() & 0x6f) != 0x27))
			{
				error_code = aris::plan::Plan::MOTION_NOT_ENABLED;
				sprintf(error_msg,
					aris::core::currentLanguage() == (int)aris::core::Language::kSimplifiedChinese ?
					u8"电机 %zd 没在使能模式，当前周期: %zd\n" :
					u8"Motor %zd is not in OPERATION_ENABLE mode in count %zd\n",
					display_id, count);
				return error_code;
			}

			// 使能时才检查 //
			if (cm.isEnabled()) {
				switch (cm.modeOfOperation()) {
				case 6:break;
				case 8: {
					// check pos infinite //
					if (!std::isfinite(cm.targetPos())) {
						error_code = aris::plan::Plan::MOTION_POS_INFINITE;
						sprintf(error_msg,
							aris::core::currentLanguage() == (int)aris::core::Language::kSimplifiedChinese ?
							u8"电机 %zu 目标位置不是有效值，当前周期: %zu\t目标位置: %f\n" :
							u8"Motor %zu target position is INFINITE in count %zu:\nvalue: %f\n",
							display_id, count, cm.targetPos());
						return error_code;
					}

					// check pos max //
					if (!(option & aris::plan::Plan::NOT_CHECK_POS_MAX)
						&& (input_pos_at_i > cm.maxPos())
						&& (input_pos_at_i > ld.pm))
					{
						error_code = aris::plan::Plan::MOTION_POS_BEYOND_MAX;
						sprintf(error_msg,
							aris::core::currentLanguage() == (int)aris::core::Language::kSimplifiedChinese ?
							u8"电机 %zu 超出正限位，当前周期: %zu\t允许最大位置: %f\t目标位置: %f\n" :
							u8"Motor %zu target position beyond MAX in count %zu:\nmax: %f\tnow: %f\n",
							display_id, count, cm.maxPos(), input_pos_at_i);
						return error_code;
					}

					// check pos min //
					if (!(option & aris::plan::Plan::NOT_CHECK_POS_MIN)
						&& (input_pos_at_i < cm.minPos())
						&& (input_pos_at_i < ld.pm))
					{
						error_code = aris::plan::Plan::MOTION_POS_BEYOND_MIN;
						sprintf(error_msg,
							aris::core::currentLanguage() == (int)aris::core::Language::kSimplifiedChinese ?
							u8"电机 %zu 超出负限位，当前周期: %zu\t允许最小值: %f\t目标位置: %f\n" :
							u8"Motor %zu target position beyond MIN in count %zu:\nmin: %f\tnow: %f\n",
							display_id, count, cm.minPos(), input_pos_at_i);
						return error_code;
					}

					// check pos continuous //
					if (!(option & aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS)
						&& ((cm.targetPos() - ld.p) > dt * cm.maxVel() || (cm.targetPos() - ld.p) < dt * cm.minVel()))
					{
						error_code = aris::plan::Plan::MOTION_POS_NOT_CONTINUOUS;
						sprintf(error_msg,
							aris::core::currentLanguage() == (int)aris::core::Language::kSimplifiedChinese ?
							u8"电机 %zu 速度过大，当前周期: %zu\t上次位置: %f\t本次位置: %f\n" :
							u8"Motor %zu target position NOT CONTINUOUS in count %zu:\nlast: %f\tnow: %f\n",
							display_id, count, ld.p, cm.targetPos());
						return error_code;
					}

					// check pos continuous second order //
					if (!(option & aris::plan::Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER)
						&& ((cm.targetPos() + lld.p - 2 * ld.p) > dt * dt * cm.maxAcc() || (cm.targetPos() + lld.p - 2 * ld.p) < dt * dt * cm.minAcc()))
					{
						error_code = aris::plan::Plan::MOTION_POS_NOT_CONTINUOUS_SECOND_ORDER;
						sprintf(error_msg,
							aris::core::currentLanguage() == (int)aris::core::Language::kSimplifiedChinese ?
							u8"电机 %zu 加速度过大，当前Count %zu:\n上上次位置: %f\t上次位置: %f\t本次位置: %f\n" :
							u8"Motor %zu target position NOT SECOND CONTINUOUS in count %zu:\nlast last: %f\tlast: %f\tnow: %f\n",
							display_id, count, lld.p, ld.p, cm.targetPos());
						return error_code;
					}

					// check pos following error //
					if (!(option & aris::plan::Plan::NOT_CHECK_POS_FOLLOWING_ERROR)
						&& (std::abs(cm.targetPos() - cm.actualPos()) > cm.maxPosFollowingError()))
					{
						error_code = aris::plan::Plan::MOTION_POS_FOLLOWING_ERROR;
						sprintf(error_msg,
							aris::core::currentLanguage() == (int)aris::core::Language::kSimplifiedChinese ?
							u8"电机 %zu 位置跟随误差过大，当前Count %zu:\n实际位置: %f\t目标位置: %f\n" :
							u8"Motion %zu target position has FOLLOW ERROR in count %zu:\nactual: %f\ttarget: %f\n",
							display_id, count, cm.actualPos(), cm.targetPos());
						return error_code;
					}

					break;
				}
				case 9: {
					// check vel infinite //
					if (!std::isfinite(cm.targetVel())) {
						error_code = aris::plan::Plan::MOTION_VEL_INFINITE;
						sprintf(error_msg,
							aris::core::currentLanguage() == (int)aris::core::Language::kSimplifiedChinese ?
							u8"电机 %zu 目标速度不是有效值，当前Count %zu:\n目标速度: %f\n" :
							u8"Motion %zu target velocity is INFINITE in count %zu:\nvalue: %f\n",
							display_id, count, cm.targetVel());
						return error_code;
					}

					// check vel max //
					if (!(option & aris::plan::Plan::NOT_CHECK_VEL_MAX)
						&& (cm.targetVel() > cm.maxVel()))
					{
						error_code = aris::plan::Plan::MOTION_VEL_BEYOND_MAX;
						sprintf(error_msg,
							aris::core::currentLanguage() == (int)aris::core::Language::kSimplifiedChinese ?
							u8"电机 %zu 速度超限，当前Count %zu:\n允许最大速度: %f\t目标速度: %f\n" :
							u8"Motion %zu target velocity beyond MAX in count %zu:\nmax: %f\tnow: %f\n",
							display_id, count, cm.maxVel(), cm.targetVel());
						return error_code;
					}

					// check vel min //
					if (!(option & aris::plan::Plan::NOT_CHECK_VEL_MIN)
						&& (cm.targetVel() < cm.minVel()))
					{
						error_code = aris::plan::Plan::MOTION_VEL_BEYOND_MIN;
						sprintf(error_msg,
							aris::core::currentLanguage() == (int)aris::core::Language::kSimplifiedChinese ?
							u8"电机 %zu 速度超限，当前Count %zu:\n允许最小速度: %f\t目标速度: %f\n" :
							u8"Motion %zu target velocity beyond MIN in count %zu:\nmin: %f\tnow: %f\n",
							display_id, count, cm.minVel(), cm.targetVel());
						return error_code;
					}

					// check vel continuous //
					if (!(option & aris::plan::Plan::NOT_CHECK_VEL_CONTINUOUS)
						&& ((cm.targetVel() - ld.v) > dt * cm.maxAcc() || (cm.targetVel() - ld.v) < dt * cm.minAcc()))
					{
						error_code = aris::plan::Plan::MOTION_VEL_NOT_CONTINUOUS;
						sprintf(error_msg,
							aris::core::currentLanguage() == (int)aris::core::Language::kSimplifiedChinese ?
							u8"电机 %zu 加速度超限，当前Count %zu:\n上次速度: %f\t本次速度: %f\n" :
							u8"Motion %zu target velocity NOT CONTINUOUS in count %zu:\nlast: %f\tnow: %f\n", display_id, count, ld.v, cm.targetVel());
						return error_code;
					}

					// check vel following error //
					if (!(option & aris::plan::Plan::NOT_CHECK_VEL_FOLLOWING_ERROR)
						&& (std::abs(cm.targetVel() - cm.actualVel()) > cm.maxVelFollowingError()))
					{
						error_code = aris::plan::Plan::MOTION_VEL_FOLLOWING_ERROR;
						sprintf(error_msg,
							aris::core::currentLanguage() == (int)aris::core::Language::kSimplifiedChinese ?
							u8"电机 %zu 速度跟随误差过大，当前Count %zu:\n实际速度: %f\t目标速度: %f\n" :
							u8"Motion %zu target velocity has FOLLOW ERROR in count %zu:\nactual: %f\ttarget: %f\n", display_id, count, cm.actualVel(), cm.targetVel());
						return error_code;
					}

					break;
				}
				case 10: {
					// check pos max //
					if (!(option & aris::plan::Plan::NOT_CHECK_POS_MAX)
						&& (cm.actualPos() > cm.maxPos()))
					{
						error_code = aris::plan::Plan::MOTION_POS_BEYOND_MAX;
						sprintf(error_msg,
							aris::core::currentLanguage() == (int)aris::core::Language::kSimplifiedChinese ?
							u8"电机 %zu 实际位置超出最大值，当前Count %zu:\n允许最大位置: %f\t实际位置: %f\n" :
							u8"Motion %zu target position beyond MAX in count %zu:\nmax: %f\tnow: %f\n", display_id, count, cm.maxPos(), input_pos_at_i);
						return error_code;
					}

					// check pos min //
					if (!(option & aris::plan::Plan::NOT_CHECK_POS_MIN)
						&& (cm.actualPos() < cm.minPos()))
					{
						error_code = aris::plan::Plan::MOTION_POS_BEYOND_MIN;
						sprintf(error_msg,
							aris::core::currentLanguage() == (int)aris::core::Language::kSimplifiedChinese ?
							u8"电机 %zu 实际位置超出最小值，当前Count %zu:\n允许最小位置: %f\t实际位置: %f\n" :
							u8"Motion %zu target position beyond MIN in count %zu:\nmin: %f\tnow: %f\n",
							display_id, count, cm.minPos(), input_pos_at_i);
						return error_code;
					}

					// check vel max //
					if (!(option & aris::plan::Plan::NOT_CHECK_VEL_MAX)
						&& (cm.actualVel() > cm.maxVel()))
					{
						error_code = aris::plan::Plan::MOTION_VEL_BEYOND_MAX;
						sprintf(error_msg,
							aris::core::currentLanguage() == (int)aris::core::Language::kSimplifiedChinese ?
							u8"电机 %zu 实际速度超出最大值，当前Count %zu:\n允许最大速度: %f\t实际速度: %f\n" :
							u8"Motion %zu target velocity beyond MAX in count %zu:\nmax: %f\tnow: %f\n",
							display_id, count, cm.maxVel(), cm.actualVel());
						return error_code;
					}

					// check vel min //
					if (!(option & aris::plan::Plan::NOT_CHECK_VEL_MIN)
						&& (cm.actualVel() < cm.minVel()))
					{
						error_code = aris::plan::Plan::MOTION_VEL_BEYOND_MIN;
						sprintf(error_msg,
							aris::core::currentLanguage() == (int)aris::core::Language::kSimplifiedChinese ?
							u8"电机 %zu 实际速度超出最小值，当前Count %zu:\n允许最小速度: %f\t实际速度: %f\n" :
							u8"Motion %zu target velocity beyond MIN in count %zu:\nmin: %f\tnow: %f\n", display_id, count, cm.minVel(), cm.actualVel());
						return error_code;
					}

					// check vel continuous //
					if (!(option & aris::plan::Plan::NOT_CHECK_VEL_CONTINUOUS)
						&& ((cm.actualVel() - ld.v) > dt * cm.maxAcc() || (cm.actualVel() - ld.v) < dt * cm.minAcc()))
					{
						error_code = aris::plan::Plan::MOTION_VEL_NOT_CONTINUOUS;
						sprintf(error_msg,
							aris::core::currentLanguage() == (int)aris::core::Language::kSimplifiedChinese ?
							u8"电机 %zu 加速度过大，当前Count %zu:\n上次速度: %f\t本次速度: %f\n" :
							u8"Motion %zu velocity NOT CONTINUOUS in count %zu:\nlast: %f\tnow: %f\n",
							display_id, count, ld.v, cm.actualVel());
						return error_code;
					}
					break;
				}
				default: {
					// invalid mode //
					if (!(option & aris::plan::Plan::NOT_CHECK_MODE)) {
						error_code = aris::plan::Plan::MOTION_INVALID_MODE;
						sprintf(error_msg,
							aris::core::currentLanguage() == (int)aris::core::Language::kSimplifiedChinese ?
							u8"电机 %zu 模式不合法，当前Count %zu:\n模式: %d\n" :
							u8"Motion %zu MODE INVALID in count %zu:\nmode: %d\n",
							display_id, count, cm.modeOfOperation());
						return error_code;
					}
				}
				}
			}
		}


		return 0;

	}
	auto ControlServerErrorCheckerByModel::fixError()->std::int32_t {
		auto controller = &imp_->cs_->controller();

		// 只要在check里面执行，都说明修复没有结束 //
		std::int32_t fix_finished{ 0 };
		for (std::size_t i = 0; i < controller->motorPool().size(); ++i) {
			// correct
			auto& cm = controller->motorPool().at(i);
			switch (cm.modeOfOperation()) {
			case 1:
			case 8:
				cm.setTargetPos(std::abs(imp_->last_pvc_[i].p - cm.actualPos()) > cm.maxPosFollowingError() ? cm.actualPos() : imp_->last_pvc_[i].p);
				break;
			case 9:
				cm.setTargetVel(0.0);
				break;
			case 10:
				cm.setTargetToq(0.0);
				fix_finished = cm.disable() || fix_finished;
				break;
			default:
				fix_finished = cm.disable() || fix_finished;
				cm.setTargetPos(0.0);
				cm.setTargetVel(0.0);
				cm.setTargetToq(0.0);
			}

			// store correct data
			imp_->last_pvc_[i].p = imp_->last_last_pvc_[i].p = controller->motorPool().at(i).targetPos();
			imp_->last_pvc_[i].v = imp_->last_last_pvc_[i].v = controller->motorPool().at(i).targetVel();
			imp_->last_pvc_[i].c = imp_->last_last_pvc_[i].c = controller->motorPool().at(i).targetToq();
		}

		return fix_finished;

		return 0;
	}
	auto ControlServerErrorCheckerByModel::storeServerData()->void {
		auto controller_ = &imp_->cs_->controller();
		auto model_ = &imp_->cs_->model();
		for (std::size_t i = 0; i < controller_->motorPool().size(); ++i) {
			imp_->last_last_pvc_[i].pm = model_->inputPosAt(i);
			imp_->last_last_pvc_[i].p = controller_->motorPool()[i].targetPos();
			imp_->last_last_pvc_[i].v = controller_->motorPool()[i].targetVel();
			imp_->last_last_pvc_[i].c = controller_->motorPool()[i].targetToq();
		}
		std::swap(imp_->last_pvc_, imp_->last_last_pvc_);
	}
	ControlServerErrorCheckerByModel::ControlServerErrorCheckerByModel() = default;
	ControlServerErrorCheckerByModel::~ControlServerErrorCheckerByModel() = default;
	ARIS_DEFINE_BIG_FOUR_CPP_NOEXCEPT(ControlServerErrorCheckerByModel);

	ARIS_REGISTRATION {
		aris::core::class_<ControlServerErrorChecker>("ControlServerErrorChecker")
			;

		aris::core::class_<ControlServerErrorCheckerByModel>("ControlServerErrorCheckerByModel")
			.inherit<ControlServerErrorChecker>()
			;
		//aris::core::class_<ProgramMiddleware>("ProgramMiddleware")
		//	.inherit<MiddleWare>()
		//	;

		//aris::core::class_<ScaraTransferModelController>("ScaraTransferModelController")
		//	.inherit<TransferModelController>()
		//	.prop("pitch", &ScaraTransferModelController::setPitch, &ScaraTransferModelController::pitch)
		//	;

		//aris::core::class_<GeneralTransferModelController>("GeneralTransferModelController")
		//	.inherit<TransferModelController>()
		//	.prop("mat", &GeneralTransferModelController::setMat, &GeneralTransferModelController::mat)
		//	;
	}
}