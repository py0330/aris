#include "aris/plan/planner_dispacher.hpp"
#include "aris/plan/multimodel_async_planner.hpp"

#include <algorithm>

#include "aris/core/error.hpp"
#include "aris/core/reflection.hpp"

namespace aris::plan {

    #define CHANEL_NODE_NUM 10000

    enum class ChanelDataNodeType {
		ResetInitPos,
		Line,
		Circle,
        MoveJoint,
        MoveAbsoluteJoint,
	};

    struct ChanelDataNode {
        std::int64_t id_{ 0 };
        ChanelDataNodeType type_{ ChanelDataNodeType::ResetInitPos };
    };
    

	struct PlannerDispacher::Imp {
		struct ChanelData {
			std::vector<aris::Size> submodel_ids;
			std::vector<double> sub_transfer_mat_;
			int lock_count{ 0 };
			MultimodelPlanner planner;
            ChanelDataNode node_pool_[CHANEL_NODE_NUM];
            std::int64_t node_id_{ 1 };
		};

		int chanel_size_{ 0 };
		aris::dynamic::MultiModel* model_{ nullptr };
		std::vector<aris::core::Matrix> transfer_matrix_;
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
		return imp_->transfer_matrix_;
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
			THROW_FILE_LINE("invalid chanel");

		// check submodel conflict with other locked chanel, if conflict return -1 to indicate lock failed //
		auto getActiveSubIds = [&]() -> std::vector<aris::Size> {
			std::vector<aris::Size> active_sub_ids;
			for (int i = 0; i < imp_->chanel_size_; ++i) {
				if (imp_->chanel_data_vec_[i]->lock_count > 0)
					active_sub_ids.insert(active_sub_ids.end(), imp_->chanel_data_vec_[i]->submodel_ids.begin(), imp_->chanel_data_vec_[i]->submodel_ids.end());
			}
			return active_sub_ids;
		};

		auto active_sub_ids = getActiveSubIds();

		for (const auto& sub_id : submodel_ids) {
			if (std::find(active_sub_ids.begin(), active_sub_ids.end(), sub_id) != active_sub_ids.end())
				return -1; // submodel conflict with other locked chanel, return -1 to indicate lock failed //
		}

		// if same submodel, directly lock and return lock count //
		if (imp_->chanel_data_vec_[chanel]->submodel_ids == submodel_ids) {
			// same submodel, directly lock //
			imp_->chanel_data_vec_[chanel]->lock_count++;
			return imp_->chanel_data_vec_[chanel]->lock_count;
		}
		else {
			imp_->chanel_data_vec_[chanel]->lock_count++;
			imp_->chanel_data_vec_[chanel]->submodel_ids = submodel_ids;
			imp_->chanel_data_vec_[chanel]->planner.setSubModelId(submodel_ids);

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
	}

	auto PlannerDispacher::releaseChanel(int chanel) -> int {
		if (chanel < 0 || chanel >= imp_->chanel_size_)
			THROW_FILE_LINE("invalid chanel");
		if (imp_->chanel_data_vec_[chanel]->lock_count > 0) {
			imp_->chanel_data_vec_[chanel]->lock_count--;
			imp_->chanel_data_vec_[chanel]->planner.clearUsedPos();
			return 0; // successfully released
		}
		return -1; // channel was not locked, return -1 to indicate release failed
	}

	auto PlannerDispacher::insertLinePos(int chanel, std::string_view tools, std::string_view wobjs, const double* ee_pos, const double* vel, const double* acc, const double* jerk, const double* zone) -> std::int64_t {
		return imp_->chanel_data_vec_[chanel]->planner.insertLinePos(tools, wobjs, ee_pos, vel, acc, jerk, zone);
	}
	auto PlannerDispacher::insertCirclePos(int chanel, std::string_view tools, std::string_view wobjs, const double* ee_pos, const double* mid_pos, const double* vel, const double* acc, const double* jerk, const double* zone) -> std::int64_t {
		return imp_->chanel_data_vec_[chanel]->planner.insertCirclePos(tools, wobjs, ee_pos, mid_pos, vel, acc, jerk, zone);
	}
	auto PlannerDispacher::updateInsertPos(int chanel) -> void {
		imp_->chanel_data_vec_[chanel]->planner.updateInsertPos();
	}

	auto PlannerDispacher::getNextInput(int chanel, double* p) -> std::int64_t {
		return imp_->chanel_data_vec_[chanel]->planner.getNextInput(p);
	}

	auto PlannerDispacher::tgRet(int chanel) -> std::int64_t {
		return imp_->chanel_data_vec_[chanel]->planner.tgRet();
	}

	auto PlannerDispacher::ikRet(int chanel) -> std::int64_t {
		return imp_->chanel_data_vec_[chanel]->planner.ikRet();
	}

	auto PlannerDispacher::setTargetSpeedRatio(int chanel, double ds) -> void {
		imp_->chanel_data_vec_[chanel]->planner.setTargetSpeedRatio(ds);
	}
	auto PlannerDispacher::targetSpeedRatio(int chanel) -> double {
		return imp_->chanel_data_vec_[chanel]->planner.targetSpeedRatio();
	}
	auto PlannerDispacher::actualSpeedRatio(int chanel) -> double {
		return imp_->chanel_data_vec_[chanel]->planner.actualSpeedRatio();
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
