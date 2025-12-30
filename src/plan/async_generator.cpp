#include"aris/plan/async_generator.hpp"
#include"aris/plan/function.hpp"
#include"aris/control/rt_timer.hpp"

namespace aris::plan {

	struct AsyncGenerator::Imp {
		static auto rt_task_func(void* ge) -> void {
			auto imp = reinterpret_cast<Imp*>(ge);

			aris::control::aris_rt_task_set_periodic(imp->sample_period_ns_);

			std::int64_t store_pos{ 0 };
			while (imp->is_rt_thread_running_.load()) {
				// sleeping //
				aris::control::aris_rt_task_wait_period();

				if (imp->ret_ids_[store_pos] == 0 && imp->is_suspending_.load()) {
					continue;
				}


				// 尽可能一次性的读入较多的数据 //
				auto store_id = imp->stored_id_.load();
				auto current_id = imp->current_id_.load();
				auto store_num = 0;
				while ((store_id + store_num) - current_id < imp->cache_size_) {
					store_pos = (store_id+store_num) % imp->cache_size_;
					imp->ret_ids_[store_pos] = imp->input_generator_(imp->cache_ + store_pos * imp->input_size_);

					store_num++;

					// 执行完毕，退出 //
					if (imp->ret_ids_[store_pos] == 0) {
						//std::cout << "stored 0:" << store_id + store_num << std::endl;
						break;
					}
				}
				store_id += store_num;
				imp->stored_id_.store(store_id);
			}
		}
		
		/////////////////////////////////////////////////////////////
		InputGenerator input_generator_{ nullptr };
		int input_size_{ 0 };
		int cache_size_{ 100 };
		std::atomic_int64_t current_id_, stored_id_;
		double dt_{ 1e-3 };

		// 
		int sample_period_ns_{ 1000000 };
		std::atomic_bool is_rt_thread_running_{ false }, is_suspending_{ true };
		std::any rt_task_handle_;

		/////////////////////////////////////////////////////////////
		std::vector<char> mem_;
		double* cache_;
		std::int64_t *ret_ids_;

		auto allocate_mem() -> void {
			Size mem_size = 0;
			core::allocMem(mem_size, cache_, input_size_*cache_size_);
			core::allocMem(mem_size, ret_ids_, cache_size_);

			mem_.resize(mem_size, char(0));

			cache_ = core::getMem(mem_.data(), cache_);
			ret_ids_ = core::getMem(mem_.data(), ret_ids_);
		};
		auto stop() {
			if (is_rt_thread_running_) {
				is_rt_thread_running_.store(false);
				if (aris::control::aris_rt_task_join(rt_task_handle_))
					THROW_FILE_LINE("aris_rt_task_join failed");
			}
		}
		auto init() -> void {
			stop();

			current_id_.store(0);
			stored_id_.store(0);
			sample_period_ns_ = dt_ / 2 * 1e9;
			is_rt_thread_running_ = true;
			
			std::fill_n(ret_ids_, cache_size_, 0);
			std::fill_n(cache_, cache_size_*input_size_, 0);

			rt_task_handle_ = aris::control::aris_rt_task_create();
			if (!rt_task_handle_.has_value()) THROW_FILE_LINE("rt_task_create failed");
			if (aris::control::aris_rt_task_start(rt_task_handle_, &Imp::rt_task_func, this))
				THROW_FILE_LINE("rt_task_start failed");
			
		}
		auto get_next_input(double* p) -> std::int64_t {
			auto store_id = stored_id_.load();
			auto current_id = current_id_.load();
			
			// 已正常结束且没有新的数据，直接返回0，且不增加current_id //
			if (ret_ids_[current_id % cache_size_] == 0 && current_id == store_id) {
				std::copy_n(cache_ + (current_id % cache_size_) * input_size_, input_size_, p);
				return 0;
			}

			// 【错误！】没有结束，但是读取速度已经超过了存储速度 //
			if (current_id >= store_id && ret_ids_[(current_id - 1) % cache_size_]) {
				std::cout << "failed async, current :" << current_id << "  store:" << store_id << std::endl;
				return -1;
			}

			// 正常读取，也可能返回0 //
			std::copy_n(cache_ + (current_id %cache_size_)*input_size_, input_size_, p);
			current_id++;
			current_id_.store(current_id);

			return ret_ids_[current_id % cache_size_];
		}
	};
	auto AsyncGenerator::setInputGenerator(InputGenerator generator) -> void {
		imp_->input_generator_ = generator;
	}
	auto AsyncGenerator::setInputSize(int input_size) -> void {
		imp_->input_size_ = input_size;
	}
	auto AsyncGenerator::inputSize() -> int {
		return imp_->input_size_;
	}
	auto AsyncGenerator::setCacheSize(int input_size) -> void {
		imp_->cache_size_ = input_size;
	}
	auto AsyncGenerator::cacheSize() -> int {
		return imp_->cache_size_;
	}
	auto AsyncGenerator::setDt(double dt) -> void {
		imp_->dt_ = dt;
	}
	auto AsyncGenerator::dt() -> double {
		return imp_->dt_;
	}
	auto AsyncGenerator::allocateMemory() -> void {
		imp_->allocate_mem();
	}
	auto AsyncGenerator::init() -> void {
		imp_->init();
	}
	auto AsyncGenerator::stop() -> void {
		imp_->stop();
	}
	auto AsyncGenerator::suspend() -> void {
		imp_->is_suspending_.store(true);
	}
	auto AsyncGenerator::resume() -> void {
		imp_->is_suspending_.store(false);
	}
	auto AsyncGenerator::cachedDataSize() -> int {
		return imp_->stored_id_.load() - imp_->current_id_.load();
	}
	auto AsyncGenerator::getNextInput(double* p) -> std::int64_t {
		return imp_->get_next_input(p);
	}
	AsyncGenerator::~AsyncGenerator() {
		stop();
	}
	AsyncGenerator::AsyncGenerator() :imp_(new Imp) {

	}

}
