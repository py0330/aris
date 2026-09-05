#include "aris/plan/planner_dispacher.hpp"
#include "aris/plan/multimodel_async_planner.hpp"

#include <algorithm>

#include "aris/core/error.hpp"
#include "aris/core/reflection.hpp"

namespace aris::plan {

	struct PlannerDispacher::Imp {
		struct ChanelData {
			std::vector<aris::Size> submodel_ids;
			std::vector<aris::core::Matrix*> sub_transfer_mat_;
			std::vector<double> sub_input_pos_; // for transfer matrix compute
			int lock_count{ 0 };
			MultimodelPlanner planner;
		};

		int chanel_size_{ 0 };
		aris::dynamic::MultiModel* model_{ nullptr };
		std::vector<aris::core::Matrix> transfer_matrice_;
		std::vector<std::unique_ptr<ChanelData>> chanel_data_vec_;
		std::vector<double> input_;
		double dt_{ 0.004 };
	};

	auto PlannerDispacher::setDt(double dt) -> void {
		imp_->dt_ = dt;
	}
	auto PlannerDispacher::dt() -> double {
		return imp_->dt_;
	}
	auto PlannerDispacher::setModel(aris::dynamic::MultiModel& model) -> void {
		imp_->model_ = &model;
	}
	auto PlannerDispacher::model() -> aris::dynamic::MultiModel& {
		return *imp_->model_;
	}
	auto PlannerDispacher::setChanelSize(int chanel_size) -> void {
		imp_->chanel_size_ = chanel_size;
	}
	auto PlannerDispacher::chanelSize() -> int {
		return imp_->chanel_size_;
	}
	auto PlannerDispacher::transferMatrice() -> std::vector<aris::core::Matrix>& {
		return imp_->transfer_matrice_;
	}
	auto PlannerDispacher::plannerAt(int chanel) -> MultimodelPlanner& {
		return imp_->chanel_data_vec_[chanel]->planner;
	}
	auto PlannerDispacher::init() -> void {
		imp_->chanel_data_vec_.clear();
		for (int i = 0; i < imp_->chanel_size_; ++i) {
			imp_->chanel_data_vec_.push_back(std::make_unique<Imp::ChanelData>());
			imp_->chanel_data_vec_[i]->planner.setModel(*imp_->model_);
			imp_->chanel_data_vec_[i]->planner.setDt(imp_->dt_);
		}

		imp_->input_.resize(imp_->model_->inputSize(), 0.0);
	}

	auto PlannerDispacher::tryLockChanel(int chanel, std::vector<aris::Size> submodel_ids) -> int {
		if (chanel < 0 || chanel >= imp_->chanel_size_)
			return -1; // chanel does not exist, return -1 to indicate lock failed //

		// check if chanel is already locked with different submodel, if locked return -2 to indicate lock failed //
		if (imp_->chanel_data_vec_[chanel]->lock_count > 0 && imp_->chanel_data_vec_[chanel]->submodel_ids != submodel_ids) {
				return -2; // chanel is already locked with different submodel, return -2 to indicate lock failed //
		}

		// check submodel conflict with other locked chanel, if conflict return -3 to indicate lock failed //
		auto getActiveSubIds = [&]() -> std::vector<aris::Size> {
			std::vector<aris::Size> active_sub_ids;
			for (int i = 0; i < imp_->chanel_size_; ++i) {
				if (i == chanel)
					continue;
				if (imp_->chanel_data_vec_[i]->lock_count > 0)
					active_sub_ids.insert(active_sub_ids.end(), imp_->chanel_data_vec_[i]->submodel_ids.begin(), imp_->chanel_data_vec_[i]->submodel_ids.end());
			}
			return active_sub_ids;
		};

		auto active_sub_ids = getActiveSubIds();

		for (const auto& sub_id : submodel_ids) {
			if (std::find(active_sub_ids.begin(), active_sub_ids.end(), sub_id) != active_sub_ids.end())
				return -3; // submodel conflict with other locked chanel, return -3 to indicate lock failed //
		}


		// If the same channel is already locked with the same submodels, allow re-entrant lock.
		if (imp_->chanel_data_vec_[chanel]->lock_count > 0
			&& imp_->chanel_data_vec_[chanel]->submodel_ids == submodel_ids) {
			imp_->chanel_data_vec_[chanel]->lock_count++;
			return imp_->chanel_data_vec_[chanel]->lock_count;
		}

		// lock chanel with new submodel //
		imp_->chanel_data_vec_[chanel]->lock_count++;
		imp_->chanel_data_vec_[chanel]->submodel_ids = submodel_ids;
		imp_->chanel_data_vec_[chanel]->planner.setSubModelId(submodel_ids);

		// transfer matrix //
		imp_->chanel_data_vec_[chanel]->sub_transfer_mat_.resize(submodel_ids.size(), nullptr);
		for (size_t i = 0; i < submodel_ids.size(); ++i) {
			imp_->chanel_data_vec_[chanel]->sub_transfer_mat_[i] = imp_->transfer_matrice_.size() > submodel_ids[i] ? &imp_->transfer_matrice_[submodel_ids[i]] : nullptr;
		}
		imp_->chanel_data_vec_[chanel]->sub_input_pos_.resize(imp_->model_->subInputPosSize(submodel_ids.size(), submodel_ids.data()), 0.0);

		// limits //
		aris::core::Matrix mat(imp_->model_->subInputPosSize(submodel_ids.size(), submodel_ids.data()), 1, 0.0);
		imp_->model_->getSubMinInputPos(submodel_ids.size(), submodel_ids.data(), mat.data());
		imp_->chanel_data_vec_[chanel]->planner.setMinPos(mat);
		imp_->model_->getSubMaxInputPos(submodel_ids.size(), submodel_ids.data(), mat.data());
		imp_->chanel_data_vec_[chanel]->planner.setMaxPos(mat);
		imp_->model_->getSubMinInputVel(submodel_ids.size(), submodel_ids.data(), mat.data());
		imp_->chanel_data_vec_[chanel]->planner.setMinVel(mat);
		imp_->model_->getSubMaxInputVel(submodel_ids.size(), submodel_ids.data(), mat.data());
		imp_->chanel_data_vec_[chanel]->planner.setMaxVel(mat);
		imp_->model_->getSubMinInputAcc(submodel_ids.size(), submodel_ids.data(), mat.data());
		imp_->chanel_data_vec_[chanel]->planner.setMinAcc(mat);
		imp_->model_->getSubMaxInputAcc(submodel_ids.size(), submodel_ids.data(), mat.data());
		imp_->chanel_data_vec_[chanel]->planner.setMaxAcc(mat);

		// target_ds init 时会被重置 //
		auto target_ds = imp_->chanel_data_vec_[chanel]->planner.targetSpeedRatio();

		imp_->chanel_data_vec_[chanel]->planner.allocateMemory();
		imp_->chanel_data_vec_[chanel]->planner.init();

		imp_->chanel_data_vec_[chanel]->planner.setTargetSpeedRatio(target_ds);
		return 1; // successfully locked with new submodel //
	}

	auto PlannerDispacher::releaseChanel(int chanel) -> int {
		if (chanel < 0 || chanel >= imp_->chanel_size_)
			return -1; // chanel does not exist, return -1 to indicate release failed
		if (imp_->chanel_data_vec_[chanel]->lock_count > 0) {
			imp_->chanel_data_vec_[chanel]->lock_count--;
			imp_->chanel_data_vec_[chanel]->planner.clearUsedPos();
			return 0; // successfully released
		}
		return -2; // channel was not locked, return -2 to indicate release failed
	}

	auto PlannerDispacher::getNextInput(int chanel, double* p) -> std::int64_t {
		auto &c = imp_->chanel_data_vec_[chanel];
		
		// get input from planner //
		std::int64_t ret = 0;
		switch (c->planner.state()) {
		case PlannerState::Uninitialized:
		case PlannerState::Paused:
			ret = 0;
			break;
		case PlannerState::Idle:
		case PlannerState::Running:
			ret = c->planner.runOneStep(c->sub_input_pos_.data());
			break;
		case PlannerState::Pausing:
			ret = c->planner.pauseOneStep(c->sub_input_pos_.data());
			break;
		case PlannerState::Resuming:
			ret = c->planner.resumeOneStep(c->sub_input_pos_.data());
			break;
		case PlannerState::Stopping:
			ret = c->planner.stopOneStep(c->sub_input_pos_.data());
			break;
		case PlannerState::MovingToTarget:
			ret = c->planner.moveToTargetOneStep(c->sub_input_pos_.data());
			break;
		default:
			break;
		}

		
		// apply transfer matrix //
		auto idx = 0;
		for(int i = 0; i< c->submodel_ids.size(); ++i) {
			auto mat = c->sub_transfer_mat_[i];
			
			auto n = imp_->model_->subInputPosSize(1, &c->submodel_ids[i]);
			if(mat) {
				aris::dynamic::s_mm(n, 1, n, mat->data(), c->sub_input_pos_.data()+idx, p+idx);
			}
			else{
				aris::dynamic::s_vc(n, c->sub_input_pos_.data()+idx, p+idx);
			}
			idx += n;
		}

		return ret;
	}

	PlannerDispacher::PlannerDispacher() : imp_(new Imp) {}
	PlannerDispacher::~PlannerDispacher() = default;

	ARIS_REGISTRATION {

		struct LocalMatList {
			typedef aris::core::Matrix value_type;
			std::vector<aris::core::Matrix> mats_;
		};

		auto local_mat_size = [](LocalMatList* pool) -> std::size_t {
			return pool->mats_.size();
		};
		auto local_mat_at = [](LocalMatList* pool, std::size_t i) -> aris::core::Matrix& {
			return pool->mats_.at(i);
		};
		auto local_mat_pushback = [](LocalMatList* pool, aris::core::Matrix* value) -> void {
			pool->mats_.push_back(*value);
		};
		auto local_mat_clear = [](LocalMatList* pool) -> void {
			pool->mats_.clear();
		};

		aris::core::class_<LocalMatList>("PlannerDispacher::TransferMatrix")
			.asRefArray(&local_mat_size, &local_mat_at, &local_mat_pushback, &local_mat_clear)
			;

		auto getTm = [](PlannerDispacher* m) -> LocalMatList {
			LocalMatList name_list;
			name_list.mats_ = m->transferMatrice();
			return name_list;
		};
		auto setTm = [](PlannerDispacher* m, LocalMatList name_list) -> void {
			m->transferMatrice() = name_list.mats_;
		};

		aris::core::class_<PlannerDispacher>("PlannerDispacher")
			.prop("dt", &PlannerDispacher::setDt, &PlannerDispacher::dt)
			.prop("chanelSize", &PlannerDispacher::setChanelSize, &PlannerDispacher::chanelSize)
			.prop("transferMatrix", &setTm, &getTm)
			;
	}

}
