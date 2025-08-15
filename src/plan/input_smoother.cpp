#include"aris/plan/input_smoother.hpp"
#include"aris/plan/function.hpp"

//#define ARIS_DEBUG_SINGULAR_PROCESSOR

namespace aris::plan {
	
	struct InputSmoother::Imp {
		InverseKinematicMethod inv_func_ = [](aris::dynamic::ModelBase* model, const double* output_pos, double* input_pos)->std::int64_t {
			model->setOutputPos(output_pos);
			auto ret = model->inverseKinematics();
			model->getInputPos(input_pos);
			return ret;
			};

		aris::Size input_size_{ 0 };

		std::vector<char> mem_;
		double
			* max_poss_,
			* max_vels_,
			* max_accs_,
			* max_jerks_,
			* min_poss_,
			* min_vels_,
			* min_accs_,
			* min_jerks_,
			* output_pos_,
			* u_,
			* input_poss_;

		std::int64_t* node_ids_;

		double
			* p0_,
			* p1_,
			* p2_,
			* p3_,
			* p4_,
			* p5_;

		double dt_{ 1e-3 };

		double current_u_{0.0};
		std::int64_t current_idx_{ 0 }, tg_idx_{ 0 };// 当前运行到的位置，与tg运行到的位置
		int look_head_size_{ 10 };

		double init_dt_du_{ 2.0 };

		std::int64_t tg_ret_{ 0 };

		aris::dynamic::ModelBase* model_{ nullptr };
		aris::plan::TrajectoryGenerator* tg_{ nullptr };
	};
	auto InputSmoother::setModel(aris::dynamic::ModelBase& model) -> void {
		imp_->model_ = &model;
		imp_->input_size_ = model.inputPosSize();

		Size mem_size = 0;
		core::allocMem(mem_size, imp_->max_poss_, imp_->input_size_);
		core::allocMem(mem_size, imp_->max_vels_, imp_->input_size_);
		core::allocMem(mem_size, imp_->max_accs_, imp_->input_size_);
		core::allocMem(mem_size, imp_->max_jerks_, imp_->input_size_);
		core::allocMem(mem_size, imp_->min_poss_, imp_->input_size_);
		core::allocMem(mem_size, imp_->min_vels_, imp_->input_size_);
		core::allocMem(mem_size, imp_->min_accs_, imp_->input_size_);
		core::allocMem(mem_size, imp_->min_jerks_, imp_->input_size_);
		core::allocMem(mem_size, imp_->output_pos_, imp_->input_size_);
		core::allocMem(mem_size, imp_->u_, imp_->look_head_size_ + 7);
		core::allocMem(mem_size, imp_->node_ids_, imp_->look_head_size_ + 7);
		core::allocMem(mem_size, imp_->input_poss_, imp_->input_size_ * (imp_->look_head_size_ + 7));
		
		imp_->mem_.resize(mem_size, char(0));

		imp_->max_poss_ = core::getMem(imp_->mem_.data(), imp_->max_poss_);
		imp_->max_vels_ = core::getMem(imp_->mem_.data(), imp_->max_vels_);
		imp_->max_accs_ = core::getMem(imp_->mem_.data(), imp_->max_accs_);
		imp_->max_jerks_ = core::getMem(imp_->mem_.data(), imp_->max_jerks_);
		imp_->min_poss_ = core::getMem(imp_->mem_.data(), imp_->min_poss_);
		imp_->min_vels_ = core::getMem(imp_->mem_.data(), imp_->min_vels_);
		imp_->min_accs_ = core::getMem(imp_->mem_.data(), imp_->min_accs_);
		imp_->min_jerks_ = core::getMem(imp_->mem_.data(), imp_->min_jerks_);
		imp_->output_pos_ = core::getMem(imp_->mem_.data(), imp_->output_pos_);
		imp_->node_ids_ = core::getMem(imp_->mem_.data(), imp_->node_ids_);
		imp_->u_ = core::getMem(imp_->mem_.data(), imp_->u_);
		imp_->input_poss_ = core::getMem(imp_->mem_.data(), imp_->input_poss_);

		std::fill_n(imp_->max_poss_, imp_->input_size_, 1e10);
		std::fill_n(imp_->min_poss_, imp_->input_size_, -1e10);
		std::fill_n(imp_->max_vels_, imp_->input_size_, 1.0);
		std::fill_n(imp_->min_vels_, imp_->input_size_, -1.0);
		std::fill_n(imp_->max_accs_, imp_->input_size_, 10.0);
		std::fill_n(imp_->min_accs_, imp_->input_size_, -10.0);
		std::fill_n(imp_->max_jerks_, imp_->input_size_, 1000.0);
		std::fill_n(imp_->min_jerks_, imp_->input_size_, -1000.0);
	}
	auto InputSmoother::setTrajectoryGenerator(TrajectoryGenerator& tg) -> void {
		imp_->tg_ = &tg;
	}
	auto InputSmoother::setPosLimits(const double* max_poss, const double* min_poss) -> void {
		std::copy(max_poss, max_poss + imp_->input_size_, imp_->max_poss_);
		if (min_poss) {
			std::copy(min_poss, min_poss + imp_->input_size_, imp_->min_poss_);
		}
		else {
			for (int i = 0; i < imp_->input_size_; ++i)
				imp_->min_poss_[i] = -imp_->max_poss_[i];
		}
	}
	auto InputSmoother::setVelLimits(const double* max_vels, const double* min_vels) -> void {
		std::copy(max_vels, max_vels + imp_->input_size_, imp_->max_vels_);
		if (min_vels) {
			std::copy(min_vels, min_vels + imp_->input_size_, imp_->min_vels_);
		}
		else {
			for (int i = 0; i < imp_->input_size_; ++i)
				imp_->min_vels_[i] = -imp_->max_vels_[i];
		}
	}
	auto InputSmoother::setAccLimits(const double* max_accs, const double* min_accs) -> void {
		std::copy(max_accs, max_accs + imp_->input_size_, imp_->max_accs_);
		if (min_accs) {
			std::copy(min_accs, min_accs + imp_->input_size_, imp_->min_accs_);
		}
		else {
			for (int i = 0; i < imp_->input_size_; ++i)
				imp_->min_accs_[i] = -imp_->max_accs_[i];
		}
	}
	auto InputSmoother::setJerkLimits(const double* max_jerks, const double* min_jerks) -> void {
		std::copy(max_jerks, max_jerks + imp_->input_size_, imp_->max_jerks_);
		if (min_jerks) {
			std::copy(min_jerks, min_jerks + imp_->input_size_, imp_->min_jerks_);
		}
		else {
			for (int i = 0; i < imp_->input_size_; ++i)
				imp_->min_jerks_[i] = -imp_->max_jerks_[i];
		}
	}
	auto InputSmoother::init(const double *init_input_pos) -> void {
		// 第一个数据应该为起始数据 //
		imp_->current_idx_ = imp_->tg_idx_ = 0;
		aris::dynamic::s_vc(imp_->input_size_, init_input_pos, imp_->input_poss_);

		// 前瞻足够的数据 //
		auto pool_size = imp_->look_head_size_ + 7;
		for (int i = 0; i < imp_->look_head_size_; ++i) {
			imp_->tg_idx_ = imp_->tg_idx_ + 1;
			imp_->node_ids_[(imp_->tg_idx_ % pool_size)] = imp_->tg_->getEePosAndMoveDt(imp_->output_pos_);
			imp_->inv_func_(imp_->model_, imp_->output_pos_, imp_->input_poss_ + (imp_->tg_idx_ % pool_size) * imp_->input_size_);
			imp_->u_[(imp_->tg_idx_ % pool_size)] = 4 * imp_->dt_ * imp_->tg_idx_;

			if (imp_->node_ids_[(imp_->tg_idx_ % pool_size)] == 0)
				break;
		}

		// 确保第一个数字有正确的值 //
		imp_->node_ids_[0] = imp_->node_ids_[1];
		imp_->u_[0] = 0.0;
		imp_->current_u_ = 0.0;

		//// 确保结尾多3个数值
		//aris::dynamic::s_vc(imp_->input_size_, imp_->input_poss_ + (imp_->tg_idx_ % pool_size) * imp_->input_size_, imp_->input_poss_ + ((imp_->tg_idx_ + 1) % pool_size) * imp_->input_size_);
		//aris::dynamic::s_vc(imp_->input_size_, imp_->input_poss_ + (imp_->tg_idx_ % pool_size) * imp_->input_size_, imp_->input_poss_ + ((imp_->tg_idx_ + 2) % pool_size) * imp_->input_size_);
		//aris::dynamic::s_vc(imp_->input_size_, imp_->input_poss_ + (imp_->tg_idx_ % pool_size) * imp_->input_size_, imp_->input_poss_ + ((imp_->tg_idx_ + 3) % pool_size) * imp_->input_size_);
	}

	auto getInputPos(Size input_size, double u_at, const double *u, const double*p0, const double* p1, const double* p2, const double* p3, double* p) -> void {
		for (Size i = 0; i < input_size; ++i) {
			double x[4]{ p0[i], p1[i], p2[i], p3[i] };
			aris::dynamic::s_interp_scurve(u, x, u_at, p[i]);
		}
	};


	struct TextNextNodeRet {
		int input_size;
		const double* max_poss, * min_poss, * max_vels, * min_vels, * max_accs, * min_accs, * max_jerks, * min_jerks;
		const double* p;
		double* u;
		std::int64_t current_idx;
		std::int64_t tg_idx;
		int pool_size;
		double* mem;
		double dt;

		auto cptRange(std::int64_t i, double *range)->int {
			double range_init[2]{ u[(i - 1) % pool_size] + dt, std::numeric_limits<double>::infinity() };
			mem[0] = range_init[0];
			mem[1] = range_init[1];

			aris::Size range_num{ 0 }, range_num_sum{ 1 };

			const double x[6]{
					i < 5 ? -u[(i - 3) % pool_size] : u[(i - 5) % pool_size],
					u[(i - 4) % pool_size],
					u[(i - 3) % pool_size],
					u[(i - 2) % pool_size],
					u[(i - 1) % pool_size] };

			const double s[6]{
					0, dt, 2 * dt, 3 * dt, 4 * dt, 5 * dt
			};

			aris::dynamic::s_interp_scurve_u5_range(x, s, -5, 5, -50, 50, -500, 500, range_num, mem + range_num_sum * 2);
			auto range_num_sum_old = range_num_sum;
			aris::dynamic::s_interval_intersect(range_num_sum_old, range_num, mem, mem + range_num_sum_old * 2, range_num_sum, mem + (range_num_sum_old + range_num) * 2);
			aris::dynamic::s_vc(range_num_sum * 2, mem + (range_num_sum_old + range_num) * 2, mem);
			

			for (int j = 0; j < input_size; ++j) {
				const double y[6]{ 
					p[(std::max(i - 5, 0i64) % pool_size) * input_size],
					p[((i - 4) % pool_size) * input_size],
					p[((i - 3) % pool_size) * input_size],
					p[((i - 2) % pool_size) * input_size],
					p[((i - 1) % pool_size) * input_size],
					p[(i % pool_size)*input_size] 
				};

				aris::dynamic::s_interp_scurve_u5_range(x, y, min_vels[j], max_vels[j], min_accs[j], max_accs[j], min_jerks[j], max_jerks[j], range_num, mem + range_num_sum * 2);
				
				auto range_num_sum_old = range_num_sum;
				aris::dynamic::s_interval_intersect(range_num_sum_old, range_num, mem, mem + range_num_sum_old * 2, range_num_sum, mem + (range_num_sum_old + range_num) * 2);
				aris::dynamic::s_vc(range_num_sum * 2, mem + (range_num_sum_old + range_num) * 2, mem);
			}

			range[0] = mem[0];
			range[1] = mem[1];
			return range_num_sum;
		};

		auto test(std::int64_t i) -> bool {
			if (i >= tg_idx + 3)
				return true;
			
			double range[2];
			auto ret = cptRange(i, range);

			if (ret == 0)
				return false;
			else if (range[1] == std::numeric_limits<double>::infinity())
				return true;
			else {
				u[i] = range[1];
				return test(i + 1);
			}
		}

		auto setU5() -> void {
			double range[2];

			cptRange(current_idx+3, range);
			
			
			// 二分法求出最合适的 u5
			double up = range[0];
			double below = range[1];
			double mid = (range[0] + range[1]) / 2;
			double last_mid = below;

			while (mid != last_mid) {
				u[(current_idx + 3) % pool_size] = mid;
				(test(current_idx + 4) ? below : up) = mid;
				last_mid = mid;
				mid = (up + below) / 2;
			}
		}
	};

	auto InputSmoother::getNextInput(double* p) -> int {
		auto pool_size = imp_->look_head_size_ + 7;// 数据池因为有一个起始数据,以及过去用来插值的3个数据，以及未来3个数据，因此应该比前瞻的数据多 7

		// ----------------- PART 1 拿数据--------------------------- //
		{
			imp_->current_u_ += imp_->dt_;

			if (imp_->current_u_ > imp_->u_[imp_->current_idx_ % pool_size]) {
				imp_->current_idx_ = imp_->current_idx_ + 1;
			}

			double* p0 = imp_->input_poss_ + (std::max(imp_->current_idx_ - 3, 0i64) % pool_size) * imp_->input_size_;
			double* p1 = imp_->input_poss_ + (std::max(imp_->current_idx_ - 2, 0i64) % pool_size) * imp_->input_size_;
			double* p2 = imp_->input_poss_ + (std::max(imp_->current_idx_ - 1, 0i64) % pool_size) * imp_->input_size_;
			double* p3 = imp_->input_poss_ + (std::min(imp_->current_idx_, imp_->tg_idx_) % pool_size) * imp_->input_size_;
			double* p4 = imp_->input_poss_ + (std::min(imp_->current_idx_ + 1, imp_->tg_idx_) % pool_size) * imp_->input_size_;
			double* p5 = imp_->input_poss_ + (std::min(imp_->current_idx_ + 2, imp_->tg_idx_) % pool_size) * imp_->input_size_;

			double u[6]{
				imp_->current_idx_ < 3 ? (-3 + imp_->current_idx_) * imp_->init_dt_du_ * imp_->dt_ : imp_->u_[(imp_->current_idx_ - 3) % pool_size],
				imp_->current_idx_ < 2 ? (-2 + imp_->current_idx_) * imp_->init_dt_du_ * imp_->dt_ : imp_->u_[(imp_->current_idx_ - 2) % pool_size],
				imp_->current_idx_ < 1 ? (-1 + imp_->current_idx_) * imp_->init_dt_du_ * imp_->dt_ : imp_->u_[(imp_->current_idx_ - 1) % pool_size],
				imp_->u_[imp_->current_idx_ % pool_size],
				imp_->tg_idx_ - imp_->current_idx_ < 1 ? 2 * imp_->u_[(imp_->current_idx_) % pool_size] - imp_->u_[(imp_->current_idx_ - 1) % pool_size]: imp_->u_[(imp_->current_idx_ + 1) % pool_size],
				imp_->tg_idx_ - imp_->current_idx_ < 2 ? imp_->u_[imp_->tg_idx_ % pool_size] + (2 - imp_->tg_idx_ + imp_->current_idx_)*(imp_->u_[imp_->tg_idx_ % pool_size] - imp_->u_[(imp_->tg_idx_ - 1) % pool_size]) : imp_->u_[(imp_->current_idx_ + 2) % pool_size],
			};

			for (Size i = 0; i < imp_->input_size_; ++i) {
				double x[6]{ p0[i], p1[i], p2[i], p3[i], p4[i], p5[i] };
				aris::dynamic::s_interp_scurve(u, x, imp_->current_u_, p[i]);
			}
		}
		//aris::dynamic::s_vc(imp_->input_size_, imp_->input_poss_ + (imp_->current_idx_% pool_size)* imp_->input_size_, p);
		//imp_->current_idx_ = imp_->current_idx_ + 1;

		// ----------------- PART 2 增数据--------------------------- //
		{
			if (imp_->node_ids_[(imp_->tg_idx_ % pool_size)] && imp_->tg_idx_ - imp_->current_idx_ < imp_->look_head_size_) {
				imp_->tg_idx_ = imp_->tg_idx_ + 1;
				imp_->node_ids_[(imp_->tg_idx_ % pool_size)] = imp_->tg_->getEePosAndMoveDt(imp_->output_pos_);
				imp_->inv_func_(imp_->model_, imp_->output_pos_, imp_->input_poss_ + (imp_->tg_idx_ % pool_size) * imp_->input_size_);
				
				
				
				
				double mem[1920];

				TextNextNodeRet t{  imp_->input_size_,
					imp_->max_poss_, imp_->min_poss_, imp_->max_vels_, imp_->min_vels_, imp_->max_accs_, imp_->min_accs_, imp_->max_jerks_, imp_->min_jerks_,
					imp_->input_poss_,
					imp_->u_,
					imp_->current_idx_,
					imp_->tg_idx_,
					pool_size,
					mem,
					imp_->dt_};


				t.setU5();
				//imp_->u_[(imp_->tg_idx_ % pool_size)] = imp_->u_[(imp_->tg_idx_ - 1) % pool_size] + imp_->dt_ * 2;
			}

		}

		
		
		//aris::dynamic::dsp(1, 14, imp_->u_);

		return imp_->current_idx_ == imp_->tg_idx_ ? 0: imp_->node_ids_[imp_->current_idx_ % pool_size];
	}

	InputSmoother::~InputSmoother() = default;
	InputSmoother::InputSmoother() :imp_(new Imp) {

	}
}
