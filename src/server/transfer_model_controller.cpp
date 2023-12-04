#include <cstring>
#include <thread>
#include <algorithm>
#include <memory>
#include <cinttypes>
#include <queue>

#include "aris/server/transfer_model_controller.hpp"

namespace aris::server{
	auto TransferModelController::updateDataController2Model(
		const std::vector<std::uint64_t>& options, 
		const aris::control::Controller* controller, 
		aris::dynamic::ModelBase* model)->void
	{
		for (std::size_t i = 0; i < std::min(controller->motorPool().size(), model->inputPosSize()); ++i) {
			auto& cm = controller->motorPool()[i];
			if ((options[i] & aris::plan::Plan::UPDATE_MODEL_POS_FROM_CONTROLLER))
				model->setInputPosAt(cm.targetPos(), i);
			if ((options[i] & aris::plan::Plan::UPDATE_MODEL_VEL_FROM_CONTROLLER))
				model->setInputVelAt(cm.targetVel(), i);
		}
	}
	auto TransferModelController::updateDataModel2Controller(
		const std::vector<std::uint64_t>& options, 
		const aris::dynamic::ModelBase* model,
		aris::control::Controller* controller) ->void
	{
		for (std::size_t i = 0; i < std::min(controller->motorPool().size(), model->inputPosSize()); ++i) {
			auto& cm = controller->motorPool()[i];
			if ((options[i] & aris::plan::Plan::USE_TARGET_POS))
				cm.setTargetPos(model->inputPosAt(i));
			if ((options[i] & aris::plan::Plan::USE_TARGET_VEL))
				cm.setTargetVel(model->inputVelAt(i));
			if ((options[i] & aris::plan::Plan::USE_TARGET_TOQ))
				cm.setTargetToq(model->inputFceAt(i));
			if ((options[i] & aris::plan::Plan::USE_OFFSET_VEL))
				cm.setOffsetVel(model->inputVelAt(i));
			if ((options[i] & aris::plan::Plan::USE_OFFSET_TOQ))
				cm.setOffsetToq(model->inputFceAt(i));
		}
	}
	
	ARIS_REGISTRATION {
		aris::core::class_<TransferModelController>("TransferModelController");

		aris::core::class_<GeneralTransferModelController>("GeneralTransferModelController")
			.inherit<TransferModelController>()
			.prop("mat", &GeneralTransferModelController::setMat, &GeneralTransferModelController::mat)
			;
	}
}