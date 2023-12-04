#ifndef ARIS_SERVER_TRANSFER_MODEL_CONTROLLER_H_
#define ARIS_SERVER_TRANSFER_MODEL_CONTROLLER_H_

#include <string>
#include <sstream>
#include <map>
#include <memory>
#include <vector>

#include <aris/core/core.hpp>
#include <aris/dynamic/dynamic.hpp>
#include <aris/control/control.hpp>
#include <aris/plan/plan.hpp>


namespace aris::server{
	class ARIS_API TransferModelController {
	public:
		auto virtual updateDataController2Model(
			const std::vector<std::uint64_t>& options, 
			const aris::control::Controller *controller, 
			aris::dynamic::ModelBase *model)->void;
		auto virtual updateDataModel2Controller(
			const std::vector<std::uint64_t>& options, 
			const aris::dynamic::ModelBase* model,
			aris::control::Controller* controller)->void;
	};

	// mat 应该是从 model 到 controller 的转换矩阵
	//
	// 
	class ARIS_API GeneralTransferModelController :public TransferModelController {
	public:
		auto updateDataController2Model(
			const std::vector<std::uint64_t>& options,
			const aris::control::Controller* controller,
			aris::dynamic::ModelBase* model)->void override
		{
			for (std::size_t i = 0; i < std::min(controller->motorPool().size(), model->inputPosSize()); ++i) {
				auto& cm = controller->motorPool()[i];
				controller_pos_[i] = cm.targetPos();
				controller_vel_[i] = cm.targetVel();
			}

			aris::dynamic::s_mm(inv_mat_.m(), 1, inv_mat_.m(), inv_mat_.data(), controller_pos_.data(), model_pos_.data());
			aris::dynamic::s_mm(inv_mat_.m(), 1, inv_mat_.m(), inv_mat_.data(), controller_vel_.data(), model_vel_.data());
			
			for (std::size_t i = 0; i < std::min(controller->motorPool().size(), model->inputPosSize()); ++i) {
				auto& cm = controller->motorPool()[i];
				if ((options[i] & aris::plan::Plan::UPDATE_MODEL_POS_FROM_CONTROLLER))
					model->setInputPosAt(model_pos_[i], i);
				if ((options[i] & aris::plan::Plan::UPDATE_MODEL_VEL_FROM_CONTROLLER))
					model->setInputVelAt(model_vel_[i], i);
			}
		}
		auto updateDataModel2Controller(
			const std::vector<std::uint64_t>& options,
			const aris::dynamic::ModelBase* model,
			aris::control::Controller* controller)->void override
		{
			model->getInputPos(model_pos_.data());
			model->getInputVel(model_vel_.data());
			model->getInputFce(model_toq_.data());

			aris::dynamic::s_mm(mat_.m(), 1, mat_.m(), mat_.data(), model_pos_.data(), controller_pos_.data());
			aris::dynamic::s_mm(mat_.m(), 1, mat_.m(), mat_.data(), model_vel_.data(), controller_vel_.data());
			aris::dynamic::s_mm(mat_.m(), 1, mat_.m(), mat_.data(), model_toq_.data(), controller_toq_.data());

			for (std::size_t i = 0; i < std::min(controller->motorPool().size(), model->inputPosSize()); ++i) {
				auto& cm = controller->motorPool()[i];
				if ((options[i] & aris::plan::Plan::USE_TARGET_POS))
					cm.setTargetPos(controller_pos_[i]);
				if ((options[i] & aris::plan::Plan::USE_TARGET_VEL))
					cm.setTargetVel(controller_vel_[i]);
				if ((options[i] & aris::plan::Plan::USE_TARGET_TOQ))
					cm.setTargetToq(controller_toq_[i]);
				if ((options[i] & aris::plan::Plan::USE_OFFSET_VEL))
					cm.setOffsetVel(controller_vel_[i]);
				if ((options[i] & aris::plan::Plan::USE_OFFSET_TOQ))
					cm.setOffsetToq(controller_toq_[i]);
			}
		}

		auto mat()const->aris::core::Matrix { return mat_; }
		auto setMat(aris::core::Matrix mat)->void { 
			if (mat.m() != mat.n()) {
				THROW_FILE_LINE("invalid mat for GeneralTransferModelContrller");
			}
			
			std::vector<double> u(mat.m()*mat.n()), t(mat.m()), t2(mat.m());
			std::vector<aris::Size> p(mat.m());
			aris::Size r;
			aris::dynamic::s_householder_utp(mat.m(), mat.n(), mat.data(), u.data(), t.data(), p.data(), r);
			if (r != mat.m())
				THROW_FILE_LINE("invalid mat for GeneralTransferModelContrller, singular mat");

			mat_ = mat;
			inv_mat_.resize(mat.m(), mat.n());
			aris::dynamic::s_householder_utp2pinv(mat.m(), mat.n(), r, u.data(), t.data(), p.data(), inv_mat_.data(), t2.data());

			model_pos_.resize(mat_.m());
			model_vel_.resize(mat_.m());
			model_toq_.resize(mat_.m());
			controller_pos_.resize(mat_.m());
			controller_vel_.resize(mat_.m());
			controller_toq_.resize(mat_.m());
		}

	private:
		aris::core::Matrix mat_, inv_mat_;
		std::vector<double> model_pos_, model_vel_, model_toq_, controller_pos_, controller_vel_, controller_toq_;
	};
}

#endif

