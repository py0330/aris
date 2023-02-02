#include"aris/plan/singular_processor.hpp"

namespace aris::plan {
	struct SingularProcessor::Imp {
		aris::Size input_pos_size_{ 0 };
		double dt{ 0.001 };



		std::vector<char> mem_;
		//std::vector<double> max_vels_, 
		//	                max_accs_, 
		//	                input_pos_begin_,
		//					input_vel_begin_,
		//					input_acc_begin_,
		//	                input_pos_end_,
		//					input_vel_end_,
		//					input_acc_end_,
		//					input_pos_last_,
		//					input_pos_this_,
		//	                output_pos_begin_,
		//	                output_pos_end_;
		
		double *max_vels_, 
			   *max_accs_,
			   *input_pos_begin_,
			   *input_vel_begin_,
			   *input_acc_begin_,
			   *input_pos_end_,
			   *input_vel_end_,
			   *input_acc_end_,
			   *input_pos_last_,
			   *input_vel_last_,
			   *input_pos_this_,
			   *input_vel_this_,
			   *input_acc_this_,
			   *output_pos_begin_,
			   *output_pos_end_;




		double T;
		double check_rate_{ 0.99 };
		aris::dynamic::ModelBase* model_{nullptr};
		aris::plan::TrajectoryGenerator* tg_{nullptr};

		bool is_in_singular{ false };
	};
	auto SingularProcessor::setMaxVels(const double* max_vels)->void {
		std::copy(max_vels, max_vels + imp_->input_pos_size_, imp_->max_vels_);
	}
	auto SingularProcessor::setMaxAccs(const double* max_accs)->void {
		std::copy(max_accs, max_accs + imp_->input_pos_size_, imp_->max_accs_);
	}
	auto SingularProcessor::setModel(aris::dynamic::ModelBase& model)->void {
		imp_->model_ = &model;
		imp_->input_pos_size_ = model.inputPosSize();

		Size mem_size = 0;
		core::allocMem(mem_size, imp_->max_vels_, imp_->input_pos_size_);
		core::allocMem(mem_size, imp_->max_accs_, imp_->input_pos_size_);
		core::allocMem(mem_size, imp_->input_pos_begin_, imp_->input_pos_size_);
		core::allocMem(mem_size, imp_->input_vel_begin_, imp_->input_pos_size_);
		core::allocMem(mem_size, imp_->input_acc_begin_, imp_->input_pos_size_);
		core::allocMem(mem_size, imp_->input_pos_end_, imp_->input_pos_size_);
		core::allocMem(mem_size, imp_->input_vel_end_, imp_->input_pos_size_);
		core::allocMem(mem_size, imp_->input_acc_end_, imp_->input_pos_size_);
		core::allocMem(mem_size, imp_->input_pos_last_, imp_->input_pos_size_);
		core::allocMem(mem_size, imp_->input_vel_last_, imp_->input_pos_size_);
		core::allocMem(mem_size, imp_->input_pos_this_, imp_->input_pos_size_);
		core::allocMem(mem_size, imp_->input_vel_this_, imp_->input_pos_size_);
		core::allocMem(mem_size, imp_->input_acc_this_, imp_->input_pos_size_);
		core::allocMem(mem_size, imp_->output_pos_begin_, imp_->input_pos_size_);
		core::allocMem(mem_size, imp_->output_pos_end_, imp_->input_pos_size_);
		
		imp_->mem_.resize(mem_size, char(0));

		imp_->max_vels_ = core::getMem(imp_->mem_.data(), imp_->max_vels_);
		imp_->max_accs_ = core::getMem(imp_->mem_.data(), imp_->max_accs_);
		imp_->input_pos_begin_ = core::getMem(imp_->mem_.data(), imp_->input_pos_begin_);
		imp_->input_vel_begin_ = core::getMem(imp_->mem_.data(), imp_->input_vel_begin_);
		imp_->input_acc_begin_ = core::getMem(imp_->mem_.data(), imp_->input_acc_begin_);
		imp_->input_pos_end_ = core::getMem(imp_->mem_.data(), imp_->input_pos_end_);
		imp_->input_vel_end_ = core::getMem(imp_->mem_.data(), imp_->input_vel_end_);
		imp_->input_acc_end_ = core::getMem(imp_->mem_.data(), imp_->input_acc_end_);
		imp_->input_pos_last_ = core::getMem(imp_->mem_.data(), imp_->input_pos_last_);
		imp_->input_vel_last_ = core::getMem(imp_->mem_.data(), imp_->input_vel_last_);
		imp_->input_pos_this_ = core::getMem(imp_->mem_.data(), imp_->input_pos_this_);
		imp_->input_vel_this_ = core::getMem(imp_->mem_.data(), imp_->input_vel_this_);
		imp_->input_acc_this_ = core::getMem(imp_->mem_.data(), imp_->input_acc_this_);
		imp_->output_pos_begin_ = core::getMem(imp_->mem_.data(), imp_->output_pos_begin_);
		imp_->output_pos_end_ = core::getMem(imp_->mem_.data(), imp_->output_pos_end_);
	}
	auto SingularProcessor::setTrajectoryGenerator(TrajectoryGenerator& tg)->void {
		imp_->tg_ = &tg;
	}

	auto SingularProcessor::setModelPosAndMoveDt()->int {
		// move tg step //
		auto move_tg_step = [this]()->std::int64_t {
			// 当前处于非奇异状态，正常求反解 //
			auto ret = imp_->tg_->getEePosAndMoveDt(imp_->output_pos_end_);
			imp_->model_->setOutputPos(imp_->output_pos_end_);
			imp_->model_->inverseKinematics();

			// 取出位置
			std::swap(imp_->input_pos_this_, imp_->input_pos_last_);
			std::swap(imp_->input_vel_this_, imp_->input_vel_last_);
			imp_->model_->getInputPos(imp_->input_pos_this_);

			// 计算速度
			aris::dynamic::s_vc(imp_->input_pos_size_, imp_->input_pos_this_, imp_->input_vel_this_);
			aris::dynamic::s_vs(imp_->input_pos_size_, imp_->input_pos_last_, imp_->input_vel_this_);
			aris::dynamic::s_nv(imp_->input_pos_size_, 1.0 / imp_->dt, imp_->input_vel_this_);

			// 计算加速度
			aris::dynamic::s_vc(imp_->input_pos_size_, imp_->input_vel_this_, imp_->input_acc_this_);
			aris::dynamic::s_vs(imp_->input_pos_size_, imp_->input_vel_last_, imp_->input_acc_this_);
			aris::dynamic::s_nv(imp_->input_pos_size_, 1.0 / imp_->dt, imp_->input_acc_this_);

			return ret;
		};
		
		// check if singular //
		auto check_if_singular = [](aris::Size input_size, const double *max_vel, const double*max_acc, const double*vel, const double *acc)->int {
			// here is condition //
			int idx = 0;
			for (idx = 0; idx < input_size; ++idx) {
				if (vel[idx] > max_vel[idx] || vel[idx] < -max_vel[idx]) {
					return idx;
				}
			}
			return input_size;
		};
		
		
		if (imp_->is_in_singular) {
			// 当前已经处于奇异点 //



		}
		else {
			auto ret = move_tg_step();


			if (check_if_singular(imp_->input_pos_size_, imp_->max_vels_, imp_->max_accs_, imp_->input_vel_this_, imp_->input_acc_this_) == imp_->input_pos_size_) {
				// 正常情况什么都不做 //
				return ret;
			}
			else {
				// 储存起始时刻 //
				aris::dynamic::s_vc(imp_->input_pos_size_, imp_->input_pos_last_, imp_->input_pos_begin_);
				aris::dynamic::s_vc(imp_->input_pos_size_, imp_->input_vel_last_, imp_->input_vel_begin_);

				// 迭代到下一个非奇异的时刻 //
				int idx;
				do {
					ret = move_tg_step();
					idx = check_if_singular(imp_->input_pos_size_, imp_->max_vels_, imp_->max_accs_, imp_->input_vel_this_, imp_->input_acc_this_);
				} while (idx != imp_->input_pos_size_);

				// 获取终止时刻 //
				aris::dynamic::s_vc(imp_->input_pos_size_, imp_->input_pos_this_, imp_->input_pos_end_);
				aris::dynamic::s_vc(imp_->input_pos_size_, imp_->input_vel_this_, imp_->input_vel_end_);

				// 插补，目前采用三次样条插补 //







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
