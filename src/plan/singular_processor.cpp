#include"aris/plan/singular_processor.hpp"

namespace aris::plan {
	struct SingularProcessor::Imp {
		std::vector<double> max_vels_, 
			                max_accs_, 
			                input_pos_begin_,
			                input_pos_end_,
							input_pos_last_,
							input_pos_this_,
							input_pos_diff_,
			                output_pos_begin_,
			                output_pos_end_;

		aris::Size input_pos_size_;

		double T;
		double check_rate_{ 0.99 };
		aris::dynamic::ModelBase* model_{nullptr};
		aris::plan::TrajectoryGenerator* tg_{nullptr};

		bool is_in_singular{ false };
	};
	auto SingularProcessor::setMaxVels(const std::vector<double> max_vels)->void {
		imp_->max_vels_ = max_vels;
	}
	auto SingularProcessor::setMaxAccs(const std::vector<double> max_accs)->void {
		imp_->max_accs_ = max_accs;
	}
	auto SingularProcessor::setModel(aris::dynamic::ModelBase& model)->void {
		imp_->model_ = &model;
		imp_->input_pos_begin_.resize(model.inputPosSize());
		imp_->input_pos_end_.resize(model.inputPosSize());
		imp_->output_pos_begin_.resize(model.outputPosSize());
		imp_->output_pos_end_.resize(model.outputPosSize());
		imp_->input_pos_size_ = model.inputPosSize();
	}
	auto SingularProcessor::setTrajectoryGenerator(TrajectoryGenerator& tg)->void {
		imp_->tg_ = &tg;
	}

	auto SingularProcessor::setModelPosAndMoveDt()->int {
		// check if singular //
		auto check_if_singular = [this]()->int {
			imp_->model_->getInputPos(imp_->input_pos_this_.data());
			aris::dynamic::s_vc(imp_->input_pos_size_, imp_->input_pos_last_.data(), imp_->input_pos_last_.data());
			aris::dynamic::s_vs(imp_->input_pos_size_, imp_->input_pos_this_.data(), imp_->input_pos_this_.data());
			int beyond_idx = 0;
			for (beyond_idx = 0; beyond_idx < imp_->input_pos_size_; ++beyond_idx) {
				double dt = 0.001;
				if (std::abs(imp_->input_pos_this_[beyond_idx]) / dt > imp_->check_rate_ * imp_->max_vels_[beyond_idx]) {
					return beyond_idx;
				}
			}
			return imp_->input_pos_size_;
		};
		
		
		if (imp_->is_in_singular) {
		
		}
		else {
			auto ret = imp_->tg_->getEePosAndMoveDt(imp_->output_pos_end_.data());
			imp_->model_->setOutputPos(imp_->output_pos_end_.data());
			imp_->model_->inverseKinematics();

			if (check_if_singular() == imp_->input_pos_size_) {
				
				
				return ret;
			}
			else {
				// 奇异情况 //



				return ret;

			}
		}

		return 0;
	}


	SingularProcessor::~SingularProcessor() = default;
	SingularProcessor::SingularProcessor() :imp_(new Imp) {

	}

	

	struct TrajectoryModelAdapter::Imp {
		std::vector<double> max_vels_, 
			                max_accs_, 
			                input_pos_begin_,
			                input_pos_end_,
							input_pos_last_,
							input_pos_this_,
							input_pos_diff_,
			                output_pos_begin_,
			                output_pos_end_;

		aris::Size input_pos_size_;

		double T;
		double check_rate_{ 0.99 };
		aris::dynamic::ModelBase* model_{nullptr};
		aris::plan::TrajectoryGenerator* tg_{nullptr};

		bool is_in_singular{ false };
	};
	auto TrajectoryModelAdapter::setMaxVels(const std::vector<double> max_vels)->void {
		imp_->max_vels_ = max_vels;
	}
	auto TrajectoryModelAdapter::setMaxAccs(const std::vector<double> max_accs)->void {
		imp_->max_accs_ = max_accs;
	}
	auto TrajectoryModelAdapter::setModel(aris::dynamic::ModelBase& model)->void {
		imp_->model_ = &model;
		imp_->input_pos_begin_.resize(model.inputPosSize());
		imp_->input_pos_end_.resize(model.inputPosSize());
		imp_->output_pos_begin_.resize(model.outputPosSize());
		imp_->output_pos_end_.resize(model.outputPosSize());
		imp_->input_pos_size_ = model.inputPosSize();
	}
	auto TrajectoryModelAdapter::setTrajectoryGenerator(TrajectoryGenerator& tg)->void {
		imp_->tg_ = &tg;
	}
	auto TrajectoryModelAdapter::setModelPosAndMoveDt()->int {
		// check if singular //
		
		



		return 0;
	}
	TrajectoryModelAdapter::~TrajectoryModelAdapter() = default;
	TrajectoryModelAdapter::TrajectoryModelAdapter() :imp_(new Imp) {

	}





}
