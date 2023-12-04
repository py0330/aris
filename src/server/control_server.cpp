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
#include "aris/server/control_server.hpp"


namespace aris::server{
	struct ControlServer::Imp{
		enum { CMD_POOL_SIZE = 10000 };
		
		struct InternalData{
			std::shared_ptr<aris::plan::Plan> plan_;
			std::function<void(aris::plan::Plan&)> post_callback_;
			bool has_prepared_{ false };

			~InternalData()	{
				// step 4a. 同步收集4a //
				if (has_prepared_ && (!(plan_->option() & aris::plan::Plan::NOT_RUN_COLLECT_FUNCTION))){
					ARIS_LOG(aris::core::LogLvl::kDebug, 0, { "server collect cmd %ji" }, plan_->cmdId());
					plan_->collectNrt();
				}

				// step 5a & 5b //
				if (post_callback_)post_callback_(*plan_);
			}
		};
		struct ChanelData {
			// 实时循环中的轨迹参数 //
			std::shared_ptr<InternalData> internal_data_queue_[CMD_POOL_SIZE];

			// cmd系列参数
			std::atomic<std::int64_t> cmd_now_, cmd_end_, cmd_collect_;
		};

		auto tg()->void;
		auto executeCmd(aris::plan::Plan &plan)->int;

		Imp(ControlServer *server) :server_(server) {}
		Imp(const Imp&) = delete;

		std::recursive_mutex mu_running_, mu_collect_;
		std::atomic_bool is_running_{ false };

		ControlServer *server_;

		// 主线程ID
		std::thread::id main_thread_id_;

		// mem pool //
		std::vector<char> mempool_;
		
		// chanel //
		enum { CHANEL_SIZE = 4 };
		ChanelData chanels[CHANEL_SIZE];
		int default_chanel_{ 0 };

		// 全局count //
		std::atomic<std::int64_t> global_count_{ 0 };

		// collect系列参数
		std::thread collect_thread_;
		std::atomic_bool is_collect_running_;

		// 交换controller与model所需的缓存 //
		double *mem_transfer_p_, *mem_transfer_v_, *mem_transfer_a_, *mem_transfer_f_;

		// Error 相关
		std::uint64_t *idle_mot_check_options_, *global_mot_check_options_;
		std::atomic<std::int64_t> err_code_and_fixed_{ 0 };
		char err_msg_[1024]{ 0 };
		// error handle //
		std::function<void(aris::plan::Plan *p, int error_num, const char *error_msg)> error_handle_;

		// log 相关
		std::atomic<bool> is_rt_log_started_{ false };

		// 储存Model, Controller, SensorRoot, PlanRoot //
		std::unique_ptr<aris::dynamic::ModelBase> model_;
		std::unique_ptr<aris::control::Controller> controller_;
		std::unique_ptr<aris::control::Master> master_;
		std::unique_ptr<aris::plan::PlanRoot> plan_root_;
		std::unique_ptr<TransferModelController> transfer_model_controller_;
		std::unique_ptr<aris::core::PointerArray<aris::server::Interface>> interface_pool_{new aris::core::PointerArray<aris::server::Interface> };
		std::unique_ptr<MiddleWare> middle_ware_{new MiddleWare};
		std::unique_ptr<CustomModule> custom_module_{new CustomModule};
		std::unique_ptr<ControlServerErrorChecker> error_checker_{ new ControlServerErrorChecker };

		// 打洞，读取数据 //
		std::atomic_bool if_get_data_{ false }, if_get_data_ready_{ false };
		const std::function<void(ControlServer&, const aris::plan::Plan *, std::any&)>* get_data_func_;
		std::any *get_data_;

		// callbacks //
		std::atomic<PreCallback> pre_callback_{ nullptr };
		std::atomic<PostCallback> post_callback_{ nullptr };

		// execute in cmd line
		std::vector<std::pair<std::string, std::function<void(aris::plan::Plan&)> > > cmdline_cmd_vec_;
		std::atomic_bool cmdline_msg_received_ = false;
		std::shared_ptr<std::promise<std::vector<std::shared_ptr<aris::plan::Plan>> > > cmdline_execute_promise_;
	};
	auto ControlServer::Imp::tg()->void{
		// pre callback //
		if (auto call = pre_callback_.load())call(ControlServer::instance());

		// 原子操作
		auto global_count = ++global_count_;
		
		union { std::int64_t err_code_and_fixed; struct { std::int32_t code; std::int32_t fix; } err; };
		err_code_and_fixed = err_code_and_fixed_.load();

		// 如果处于错误状态,或者错误还未清理完 //
		if (err_code_and_fixed){
			err.fix = server_->errorChecker().fixError();
			err_code_and_fixed_.store(err_code_and_fixed);
			server_->master().resetRtStasticData(nullptr, false);
			server_->master().lout() << std::flush;
			
			for (int i = 0; i < CHANEL_SIZE; ++i) {
				auto& chanel = chanels[i];
				auto cmd_now = chanel.cmd_now_.load();
				auto cmd_end = chanel.cmd_end_.load();

				// 清理掉所有当前在执行的plan //
				if (cmd_now < cmd_end) {
					auto& p = *chanel.internal_data_queue_[cmd_now % CMD_POOL_SIZE]->plan_;
					p.setExecuteRetCode(err.code);
					p.setExecuteRetMsg(err_msg_);
					for (auto cmd_id = cmd_now + 1; cmd_id < cmd_end; ++cmd_id) {
						auto& p = *chanel.internal_data_queue_[cmd_id % CMD_POOL_SIZE]->plan_;
						p.setExecuteRetCode(aris::plan::Plan::EXECUTE_CANCELLED);
						p.setExecuteRetMsg("execute has been cancelled.");
					}
				}
				chanel.cmd_now_.store(cmd_end);
			}
		}
		// 否则执行cmd queue中的cmd //
		else {
			bool is_idle = true;
			for (int i = 0; i < CHANEL_SIZE; ++i) {
				auto& chanel = chanels[i];
				auto cmd_now = chanel.cmd_now_.load();
				auto cmd_end = chanel.cmd_end_.load();
			
				if (cmd_end > cmd_now) {
					auto& plan = *chanel.internal_data_queue_[cmd_now % CMD_POOL_SIZE]->plan_;

					is_idle = false;

					// 在第一回合初始化，包括log，初始化target等 //
					if (plan.setCount(plan.count() + 1), plan.count() == 1) {
						// 初始化target
						plan.setBeginGlobalCount(global_count);

						// 创建rt_log文件 //
						if (is_rt_log_started_) {
							char name[1000];
							std::sprintf(name, "%" PRId64 "", plan.cmdId());
							server_->master().logFile(name);
						}

						// 初始化统计数据 //
						server_->master().resetRtStasticData(&plan.rtStastic(), true);
					}

					// 执行命令
					auto ret = executeCmd(plan);

					// 错误，包含系统检查出的错误以及用户返回错误 //
					if (((err.code = server_->errorChecker().checkError(plan.count(), plan.motorOptions().data(), err_msg_)) < 0) || ((err.code = ret) < 0)) {
						err.fix = server_->errorChecker().fixError();
						err_code_and_fixed_.store(err_code_and_fixed);
						if (error_handle_)error_handle_(&plan, err.code, err_msg_);

						// finish //
						if (ret >= 0) { // 只有 plan 认为自己正确的时候，才更改其返回值 //
							plan.setExecuteRetMsg(err_msg_);
							plan.setExecuteRetCode(err.code);
						}
						else {
							std::copy_n(plan.executeRetMsg(), 1024, err_msg_);
							plan.setExecuteRetCode(err.code);
						}
						for (auto cmd_id = cmd_now + 1; cmd_id < cmd_end; ++cmd_id) {
							auto& p = *chanel.internal_data_queue_[cmd_id % CMD_POOL_SIZE]->plan_;
							p.setExecuteRetCode(aris::plan::Plan::EXECUTE_CANCELLED);
							p.setExecuteRetMsg("execute has been cancelled.");
						}

						server_->master().resetRtStasticData(nullptr, false);
						server_->master().lout() << std::flush;
						chanel.cmd_now_.store(cmd_end);// 原子操作
					}
					// 命令正常结束，结束统计数据 //
					else if (ret == 0) {
						// print info //
						if (!(plan.option() & aris::plan::Plan::NOT_PRINT_EXECUTE_COUNT))
							plan.master()->mout() << "RT" << i << " " << plan.cmdId() << "---" 
							<< "cmd finished, spend " << plan.count() << " counts\n";

						// finish //
						server_->master().resetRtStasticData(nullptr, false);
						server_->master().lout() << std::flush;
						chanel.cmd_now_.store(cmd_now + 1);//原子操作
					}
					// 命令仍在执行 //
					else {
						// print info //
						if (plan.count() % 1000 == 0 && !(plan.option() & aris::plan::Plan::NOT_PRINT_EXECUTE_COUNT))
							plan.master()->mout() << "RT" << i << " " << plan.cmdId() << "---" 
							<< "execute cmd in count: " << plan.count() << "\n";
					}
				}
			}
			
			if (is_idle) {
				if (err.code = server_->errorChecker().checkError(0, idle_mot_check_options_, err_msg_); err.code < 0) {
					err.fix = server_->errorChecker().fixError();
					err_code_and_fixed_.store(err_code_and_fixed);
					if (error_handle_)error_handle_(nullptr, err.code, err_msg_);
					server_->master().mout() << "RT  ---failed when idle " << err.code << ":\nRT  ---" << err_msg_ << "\n";
				}
			}
		} 
		

		// 储存本次的数据 //
		server_->errorChecker().storeServerData();

		// 给与外部想要的数据 //
		if (if_get_data_.exchange(false)){ // 原子操作
			auto cmd_end = chanels[default_chanel_].cmd_end_.load();
			auto cmd_now = chanels[default_chanel_].cmd_now_.load();
			get_data_func_->operator()(ControlServer::instance(), cmd_end > cmd_now ? &*chanels[0].internal_data_queue_[cmd_now % CMD_POOL_SIZE]->plan_ : nullptr, *get_data_);
			if_get_data_ready_.store(true); // 原子操作
		}

		// post callback //
		if (auto call = post_callback_.load())call(ControlServer::instance());
	}
	auto ControlServer::Imp::executeCmd(aris::plan::Plan &plan)->int{
		// 从controller 向model更新数据 //
		server_->updateDataController2Model(plan.motorOptions());

		// 执行plan函数 //
		int ret = plan.executeRT();

		// 控制电机 //
		server_->updateDataModel2Controller(plan.motorOptions());


		return ret;
	}
	auto ControlServer::instance()noexcept->ControlServer & { static ControlServer instance; return instance; }
	auto ControlServer::resetModel(dynamic::ModelBase *model)->void { imp_->model_.reset(model); }
	auto ControlServer::model()->dynamic::ModelBase& { return *imp_->model_; }

	auto ControlServer::resetMaster(control::Master *master)->void { imp_->master_.reset(master); }
	auto ControlServer::master()->control::Master& { return *imp_->master_; }

	auto ControlServer::resetController(control::Controller *controller)->void{	imp_->controller_.reset(controller);}
	auto ControlServer::controller()->control::Controller& { return *imp_->controller_; }

	auto ControlServer::resetPlanRoot(plan::PlanRoot *plan_root)->void{	imp_->plan_root_.reset(plan_root);}
	auto ControlServer::planRoot()->plan::PlanRoot& { return *imp_->plan_root_; }
	
	auto ControlServer::resetTransferModelController(TransferModelController*method)->void { imp_->transfer_model_controller_.reset(method); }
	auto ControlServer::transferModelController()->TransferModelController& { return *imp_->transfer_model_controller_; }
	
	auto ControlServer::resetInterfacePool(aris::core::PointerArray<aris::server::Interface> *pool)->void {
		imp_->interface_pool_.reset(pool);
	}
	auto ControlServer::interfacePool()->aris::core::PointerArray<aris::server::Interface>& { return *imp_->interface_pool_; }

	auto ControlServer::resetMiddleWare(aris::server::MiddleWare *middle_ware)->void { imp_->middle_ware_.reset(middle_ware); }
	auto ControlServer::middleWare()->MiddleWare& { return *imp_->middle_ware_; }

	auto ControlServer::resetCustomModule(server::CustomModule *custom_module)->void { imp_->custom_module_.reset(custom_module); }
	auto ControlServer::customModule()->CustomModule& { return *imp_->custom_module_; }
	
	auto ControlServer::resetErrorChecker(server::ControlServerErrorChecker* error_checker)->void {	imp_->error_checker_.reset(error_checker);}
	auto ControlServer::errorChecker()->ControlServerErrorChecker& {return *imp_->error_checker_;}

	auto ControlServer::updateDataController2Model(const std::vector<std::uint64_t>& options)noexcept->void {
		imp_->transfer_model_controller_->updateDataController2Model(options, &controller(), &model());
	}
	auto ControlServer::updateDataModel2Controller(const std::vector<std::uint64_t>& options)noexcept->void {
		imp_->transfer_model_controller_->updateDataModel2Controller(options, &model(), &controller());
	}
	
	auto ControlServer::setRtErrorCallback(std::function<void(aris::plan::Plan *p, int error_num, const char *error_msg)> call_back)noexcept->void {
		imp_->error_handle_ = call_back;
	}
	auto ControlServer::errorMsg()const noexcept->const char * { return imp_->err_msg_; }
	auto ControlServer::setRtPlanPreCallback(PreCallback pre_callback)noexcept->void { imp_->pre_callback_.store(pre_callback); }
	auto ControlServer::setRtPlanPostCallback(PostCallback post_callback)noexcept->void { imp_->post_callback_.store(post_callback); }
	auto ControlServer::running()noexcept->bool { return imp_->is_running_; }
	auto ControlServer::globalCount()noexcept->std::int64_t { return imp_->global_count_.load(); }
	auto ControlServer::currentExecutePlanRt(int chanel)noexcept->aris::plan::Plan *{
		auto cmd_now = imp_->chanels[chanel].cmd_now_.load();
		auto cmd_end = imp_->chanels[chanel].cmd_end_.load();
		return cmd_end > cmd_now ? imp_->chanels[chanel].internal_data_queue_[cmd_now % Imp::CMD_POOL_SIZE]->plan_.get() : nullptr;
	}
	auto ControlServer::globalMotionCheckOption()noexcept->std::uint64_t* { return imp_->global_mot_check_options_; }
	auto ControlServer::idleMotionCheckOption()noexcept->std::uint64_t* { return imp_->idle_mot_check_options_; }
	auto ControlServer::setAutoLogActive(bool auto_log)noexcept->void { imp_->is_rt_log_started_.store(auto_log); }
	auto ControlServer::autoLogActive()noexcept->bool { return imp_->is_rt_log_started_.load(); }

	auto ControlServer::open()->void{ for (auto &inter : interfacePool()) inter.open();	}
	auto ControlServer::close()->void { for (auto &inter : interfacePool()) inter.close(); }
	auto ControlServer::runCmdLine()->void{
		static TerminalInterface terminal;

		auto ret = std::async(std::launch::async, []()->std::string{
			std::string command_in;
			std::getline(std::cin, command_in);
			if (command_in.empty())std::this_thread::sleep_for(std::chrono::milliseconds(1));
			return command_in;
		});

		imp_->main_thread_id_ = std::this_thread::get_id();

		for (;;){
			// 检测是否有数据从executeCmdInMain过来
			if (imp_->cmdline_msg_received_){
				auto ret_plan = executeCmd(imp_->cmdline_cmd_vec_);
				imp_->cmdline_msg_received_ = false;
				imp_->cmdline_execute_promise_->set_value(ret_plan);
			}
			// 检测是否有数据从command line过来
			else if (ret.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready){
				// 在linux后台可能getline失败，得到空字符串 //
				if (auto cmd_str = ret.get(); !cmd_str.empty())	{
					imp_->middle_ware_->executeCmd(cmd_str, dynamic_cast<Interface*>(&terminal));
				}

				ret = std::async(std::launch::async, []()->std::string{
					std::string command_in;
					std::getline(std::cin, command_in);
					if (command_in.empty())std::this_thread::sleep_for(std::chrono::milliseconds(1));
					return command_in;
				});
			}
			// 休息
			else{
				std::this_thread::sleep_for(std::chrono::milliseconds(1));
			}
		}
	}
	auto ControlServer::setDefaultChanel(int chanel)noexcept->void {
		imp_->default_chanel_ = chanel;
	}
	auto ControlServer::defaultChanel()const noexcept->int {
		return imp_->default_chanel_;
	}
	auto ControlServer::executeCmd(std::vector<std::pair<std::string, std::function<void(aris::plan::Plan&)> > > cmd_vec, int chanel)->std::vector<std::shared_ptr<aris::plan::Plan>>{
		std::unique_lock<std::recursive_mutex> running_lck(imp_->mu_running_);

		auto setRetCodeAndOption = [](const std::shared_ptr<aris::plan::Plan>& plan, std::int32_t code)->void {
			plan->setPrepareRetCode(code);
			plan->option() |= aris::plan::Plan::NOT_RUN_EXECUTE_FUNCTION;
		};

		// pre process //
		chanel = chanel == -1 ? defaultChanel() : chanel;

		// step 1.  parse //
		std::vector<std::shared_ptr<Imp::InternalData>> internal_data(cmd_vec.size());
		std::vector<std::shared_ptr<aris::plan::Plan>> ret_plan(cmd_vec.size());
		for (aris::Size i = 0; i < cmd_vec.size(); ++i) {
			auto& str           = cmd_vec[i].first;
			auto& post_callback = cmd_vec[i].second;
			
			internal_data[i] = std::shared_ptr<Imp::InternalData>(new Imp::InternalData{ std::shared_ptr<aris::plan::Plan>(nullptr), post_callback });
			
			auto &plan = internal_data[i]->plan_;
			try{ // case 1.1 : success
				std::vector<char> cmd_str_local(str.size());
				std::copy(str.begin(), str.end(), cmd_str_local.begin());

				static std::uint64_t cmd_id{ 0 };
				++cmd_id;
				ARIS_LOG(aris::core::LogLvl::kDebug, 0, { "server parse cmd %ji : %s", "服务器分析指令 %ji : %s" }, cmd_id, str.data());
				auto[cmd, params] = planRoot().planParser().parse(std::string_view(cmd_str_local.data(), cmd_str_local.size())); // may throw
				auto plan_iter = std::find_if(planRoot().planPool().begin(), planRoot().planPool().end(), [&](const plan::Plan &p) {return p.command().name() == cmd; });
				plan = std::shared_ptr<aris::plan::Plan>(dynamic_cast<aris::plan::Plan*>(plan_iter->clone()));
				ret_plan[i] = plan;
				plan->setSharedPtrForThis(plan);

				plan->setCmdId(cmd_id);
				plan->setCount(0);
				plan->setBeginGlobalCount(0);
				
				plan->setControlServer(this);
				plan->setModelBase(imp_->model_.get());
				plan->setMaster(imp_->master_.get());
				plan->setController(imp_->controller_.get());
				
				plan->option() = 0;
				plan->motorOptions().resize(plan->controller()->motorPool().size(), 0);
				std::copy_n(imp_->global_mot_check_options_, plan->controller()->motorPool().size(), plan->motorOptions().data());
				
				plan->rtStastic() = aris::control::Master::RtStasticsData{ 0,0,0,0x8fffffff,0,0,0 };
				plan->setPrepareRetCode(aris::plan::Plan::SUCCESS);
				std::fill_n(plan->prepareRetMsg(), 1024, '\0');
				plan->setExecuteRetCode(aris::plan::Plan::SUCCESS);
				std::fill_n(plan->executeRetMsg(), 1024, '\0');

				plan->command().init();
				plan->parse(str); // may throw, set cmd_string cmd_name & cmd_params 
			}
			catch (std::exception &e){ // case 1.2 : exception
				for (aris::Size j = 0; j < i; j++)
					setRetCodeAndOption(ret_plan[j], aris::plan::Plan::PREPARE_CANCELLED);
				for (aris::Size j = i + 1; j < cmd_vec.size(); j++) {
					internal_data[j] = std::shared_ptr<Imp::InternalData>(new Imp::InternalData{ std::shared_ptr<aris::plan::Plan>(new aris::plan::Plan), cmd_vec[j].second });
					ret_plan[j] = internal_data[j]->plan_;
					setRetCodeAndOption(ret_plan[j], aris::plan::Plan::PREPARE_CANCELLED);
				}
				
				internal_data[i]->plan_ =  std::shared_ptr<aris::plan::Plan>(new aris::plan::Plan);
				ret_plan[i] = internal_data[i]->plan_;
				setRetCodeAndOption(ret_plan[i], aris::plan::Plan::PREPARE_CANCELLED);
				std::copy_n(e.what(), std::strlen(e.what()), ret_plan[i]->prepareRetMsg());
				return ret_plan;
			}
		}

		// step 2.  prepare //
		bool prepare_error = false;
		for (auto p = internal_data.begin(); p < internal_data.end(); ++p){
			auto &plan = (*p)->plan_;
			try	{
				ARIS_LOG(aris::core::LogLvl::kDebug, 0, { "server prepare cmd %ji" }, plan->cmdId());
				plan->prepareNrt();
				(*p)->has_prepared_ = true;

				// false : case 2.1    true : case 2.2 
				if (plan->prepareRetCode() < 0) { 
					for (auto pp = internal_data.begin(); pp < internal_data.end(); ++pp) {
						if (pp < p) setRetCodeAndOption((*pp)->plan_, aris::plan::Plan::EXECUTE_CANCELLED);
						if (pp > p) setRetCodeAndOption((*pp)->plan_, aris::plan::Plan::PREPARE_CANCELLED);
					}
					plan->option() |= aris::plan::Plan::NOT_RUN_EXECUTE_FUNCTION;
					prepare_error = true;
					break;
				}
			}
			catch (std::exception &e){ // case 2.3
				for (auto pp = internal_data.begin(); pp < internal_data.end(); ++pp){
					if (pp < p) setRetCodeAndOption((*pp)->plan_, aris::plan::Plan::EXECUTE_CANCELLED);
					if (pp > p) setRetCodeAndOption((*pp)->plan_, aris::plan::Plan::PREPARE_CANCELLED);
				}
				setRetCodeAndOption(plan, aris::plan::Plan::PREPARE_EXCEPTION);
				std::copy_n(e.what(), std::strlen(e.what()), plan->prepareRetMsg());
				prepare_error = true;
				break;
			}
		}

		// print and log cmd info /////////////////////////////////////////////////////////////////////////////////////////////////////////////
		for (auto &plan : ret_plan){
			auto print_size = plan->cmdParams().empty() ? 2 : 2 + std::max_element(plan->cmdParams().begin(), plan->cmdParams().end(), [](const auto& a, const auto& b){
				return a.first.length() < b.first.length();
			})->first.length();
			// print
			if (!(plan->option() & aris::plan::Plan::NOT_PRINT_CMD_INFO)){
				ARIS_COUT << "cmd " << plan->cmdId() << "---" << plan->cmdString() << "\n";
				ARIS_COUT_PLAN(plan) << plan->cmdName() << "\n";
				for (auto &p : plan->cmdParams())ARIS_COUT_PLAN(plan) << std::string(print_size - p.first.length(), ' ') << p.first << " : " << p.second << "\n";
				ARIS_COUT << std::endl;
			}
			// log
			if (!(plan->option() & aris::plan::Plan::NOT_LOG_CMD_INFO)){
				std::stringstream ss;
				ss << plan->cmdName() << "\n";
				for (auto &p : plan->cmdParams()){
					ss << std::string(print_size - p.first.length(), ' ') << p.first << " : " << p.second << std::endl;
				}
				ARIS_LOG(aris::core::LogLvl::kDebug, 0, { ss.str().c_str() });
			}
		}
		// print over ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		if(prepare_error)return ret_plan;

		// step 3.  execute //
		{
			// 构造需要 execute 的 cmd_list //
			std::list<std::shared_ptr<Imp::InternalData>> need_run_internal;
			for (auto& inter : internal_data)
				if (!(inter->plan_->option() & aris::plan::Plan::NOT_RUN_EXECUTE_FUNCTION))
					need_run_internal.push_back(inter);

			// 检查 server 是否已经在运行 //
			if (!imp_->is_running_) {
				for (auto& inter : need_run_internal) {
					setRetCodeAndOption(inter->plan_, aris::plan::Plan::SERVER_NOT_STARTED);
					inter->plan_->setPrepareRetMsg("server not started, use cs_start to start");
				}
				return ret_plan;
			}

			// 检查 server 是否处于错误状态 //
			if (auto err = this->errorCode()) {
				for (auto& inter : need_run_internal) {
					setRetCodeAndOption(inter->plan_, err);
					inter->plan_->setPrepareRetMsg("server in error, use cl to clear");
				}
				return ret_plan;
			}

			// 查看是否 plan 池已满 //
			auto cmd_end = imp_->chanels[chanel].cmd_end_.load();
			if ((cmd_end - imp_->chanels[chanel].cmd_collect_.load() + need_run_internal.size()) >= Imp::CMD_POOL_SIZE) {//原子操作(cmd_now)
				for (auto& inter : need_run_internal) {
					setRetCodeAndOption(inter->plan_, aris::plan::Plan::COMMAND_POOL_IS_FULL);
					inter->plan_->setPrepareRetMsg("command pool is full");
				}
				return ret_plan;
			}

			// 添加命令 //
			for (auto& inter : need_run_internal) {
				imp_->chanels[chanel].internal_data_queue_[cmd_end++ % Imp::CMD_POOL_SIZE] = inter;
				ARIS_LOG(aris::core::LogLvl::kDebug, 0, { "server execute cmd %ji" }, inter->plan_->cmdId());
			}
			imp_->chanels[chanel].cmd_end_.store(cmd_end);
		}

		// step 4.   in RT

		// step 5&6. USE RAII //
		return ret_plan;
	}
	auto ControlServer::executeCmd(std::string cmd_str, std::function<void(aris::plan::Plan&)> post_callback, int chanel)->std::shared_ptr<aris::plan::Plan>{
		std::vector<std::pair<std::string, std::function<void(aris::plan::Plan&)> > > cmd_vec{ std::make_pair(cmd_str, post_callback) };
		auto ret = executeCmd(cmd_vec, chanel);
		return ret.front();
	}
	auto ControlServer::executeCmdInCmdLine(std::vector<std::pair<std::string, std::function<void(aris::plan::Plan&)>>> cmd_vec, int chanel)->std::vector<std::shared_ptr<aris::plan::Plan>>{
		static std::mutex mu_;
		std::unique_lock<std::mutex> lck(mu_);

		const auto &cur_thread_id = std::this_thread::get_id();
		if (cur_thread_id == imp_->main_thread_id_) {
			return executeCmd(cmd_vec, chanel);
		} 
		else {
			imp_->cmdline_execute_promise_ = std::make_shared<std::promise<std::vector<std::shared_ptr<aris::plan::Plan>>>>();
			auto ret = imp_->cmdline_execute_promise_->get_future();
			imp_->cmdline_cmd_vec_ = cmd_vec;
			imp_->cmdline_msg_received_ = true;

			return ret.get();
		}
	}
	auto ControlServer::executeCmdInCmdLine(std::string cmd_string, std::function<void(aris::plan::Plan&)> post_callback, int chanel)->std::shared_ptr<aris::plan::Plan>{
		std::vector<std::pair<std::string, std::function<void(aris::plan::Plan&)> > > cmd_vec{ std::make_pair(cmd_string, post_callback) };
		auto ret = executeCmdInCmdLine(cmd_vec, chanel);
		return ret.front();
	}
	auto ControlServer::init()->void{
		model().init();
		master().init();
		controller().init();
		planRoot().init();
		middleWare().init();

		errorChecker().init(this);

		// 更新每个 plan 的初值 //
		for (auto &p : planRoot().planPool()) {
			p.setCmdId(0);
			p.setCount(0);
			p.setBeginGlobalCount(0);

			p.setControlServer(this);
			p.setModelBase(imp_->model_.get());
			p.setMaster(imp_->master_.get());
			p.setController(imp_->controller_.get());

			p.option() = 0;
			p.motorOptions().resize(p.controller()->motorPool().size(), 0);

			p.rtStastic() = aris::control::Master::RtStasticsData{ 0,0,0,0x8fffffff,0,0,0 };
			p.setPrepareRetCode(aris::plan::Plan::SUCCESS);
			std::fill_n(p.prepareRetMsg(), 1024, '\0');

			p.setExecuteRetCode(aris::plan::Plan::SUCCESS);
			std::fill_n(p.executeRetMsg(), 1024, '\0');
		}

		// 分配自身所需要的内存 //
		Size mem_size = 0;

		core::allocMem(mem_size, imp_->idle_mot_check_options_, controller().motorPool().size());
		core::allocMem(mem_size, imp_->global_mot_check_options_, controller().motorPool().size());

		core::allocMem(mem_size, imp_->mem_transfer_p_, imp_->model_->inputPosSize());
		core::allocMem(mem_size, imp_->mem_transfer_v_, imp_->model_->inputVelSize());
		core::allocMem(mem_size, imp_->mem_transfer_a_, imp_->model_->inputAccSize());
		core::allocMem(mem_size, imp_->mem_transfer_f_, imp_->model_->inputFceSize());

		imp_->mempool_.resize(mem_size, char(0));

		imp_->idle_mot_check_options_ = core::getMem(imp_->mempool_.data(), imp_->idle_mot_check_options_);
		std::fill_n(imp_->idle_mot_check_options_, controller().motorPool().size(), aris::plan::Plan::NOT_CHECK_ENABLE | aris::plan::Plan::NOT_CHECK_POS_MAX | aris::plan::Plan::NOT_CHECK_POS_MIN);
		imp_->global_mot_check_options_ = core::getMem(imp_->mempool_.data(), imp_->global_mot_check_options_);
		std::fill_n(imp_->global_mot_check_options_, controller().motorPool().size(), std::uint64_t(0));

		imp_->mem_transfer_p_ = core::getMem(imp_->mempool_.data(), imp_->mem_transfer_p_);
		std::fill_n(imp_->mem_transfer_p_, model().inputPosSize(), 0.0);
		imp_->mem_transfer_v_ = core::getMem(imp_->mempool_.data(), imp_->mem_transfer_v_);
		std::fill_n(imp_->mem_transfer_v_, model().inputVelSize(), 0.0);
		imp_->mem_transfer_a_ = core::getMem(imp_->mempool_.data(), imp_->mem_transfer_a_);
		std::fill_n(imp_->mem_transfer_a_, model().inputAccSize(), 0.0);
		imp_->mem_transfer_f_ = core::getMem(imp_->mempool_.data(), imp_->mem_transfer_f_);
		std::fill_n(imp_->mem_transfer_f_, model().inputFceSize(), 0.0);

		// 赋予初值 //
		master().setControlStrategy([this]() {this->imp_->tg(); }); // controller可能被reset，因此这里必须重新设置//

		for (int i = 0; i < Imp::CHANEL_SIZE; ++i) {
			imp_->chanels[i].cmd_now_.store(0);
			imp_->chanels[i].cmd_end_.store(0);
			imp_->chanels[i].cmd_collect_.store(0);
		}

	}
	auto ControlServer::start()->void{
		std::unique_lock<std::recursive_mutex> running_lck(imp_->mu_running_);
		if (imp_->is_running_)THROW_FILE_LINE("failed to start server, because it is already started ");
		
		struct RaiiCollector{
			ControlServer *cs_;
			auto reset()->void { cs_ = nullptr; }
			RaiiCollector(ControlServer *cs) :cs_(cs) {}
			~RaiiCollector(){
				if (cs_){
					cs_->imp_->is_running_ = false;
					cs_->imp_->is_collect_running_ = false;
					if (cs_->imp_->collect_thread_.joinable())cs_->imp_->collect_thread_.join();
				}
			}
		};
		RaiiCollector raii_collector(this);

		imp_->is_running_ = true;

		// start collect thread //
		imp_->is_collect_running_ = true;
		imp_->collect_thread_ = std::thread([this](){
			while (this->imp_->is_collect_running_){
				bool has_collect_plan = false;
				for (int i = 0; i < Imp::CHANEL_SIZE; ++i) {
					auto cmd_collect = imp_->chanels[i].cmd_collect_.load();//原子操作
					auto cmd_now = imp_->chanels[i].cmd_now_.load();//原子操作

					// step 4b. //
					if (cmd_collect < cmd_now) {
						has_collect_plan = true;

						auto& internal_data = imp_->chanels[i].internal_data_queue_[cmd_collect % Imp::CMD_POOL_SIZE];
						auto& plan = *internal_data->plan_;

						// make rt stastic thread safe //
						while (globalCount() == plan.beginGlobalCount() + plan.count() - 1) { std::this_thread::sleep_for(std::chrono::milliseconds(1)); }

						std::stringstream ss;
						ss << "cmd " << plan.cmdId() << " stastics:" << std::endl
							<< std::setw(20) << "avg time(ns):" << std::int64_t(plan.rtStastic().avg_time_consumed) << std::endl
							<< std::setw(20) << "max time(ns):" << plan.rtStastic().max_time_consumed << std::endl
							<< std::setw(20) << "in count:" << plan.rtStastic().max_time_occur_count << std::endl
							<< std::setw(20) << "min time(ns):" << plan.rtStastic().min_time_consumed << std::endl
							<< std::setw(20) << "in count:" << plan.rtStastic().min_time_occur_count << std::endl
							<< std::setw(20) << "total count:" << plan.rtStastic().total_count << std::endl
							<< std::setw(20) << "overruns:" << plan.rtStastic().overrun_count << std::endl;

						ARIS_LOG(aris::core::LogLvl::kDebug, 0, { ss.str().data() });

						// step 4b&5b //
						internal_data.reset();
						aris::server::ControlServer::instance().imp_->chanels[i].cmd_collect_++;
					}
				}
			
				if (!has_collect_plan) {
					std::this_thread::sleep_for(std::chrono::milliseconds(1));
				}
			}
		});

		master().start();

		raii_collector.reset();
	}
	auto ControlServer::stop()->void
	{
		std::unique_lock<std::recursive_mutex> running_lck(imp_->mu_running_);
		if (!imp_->is_running_)THROW_FILE_LINE("failed to stop server, because it is not running");
		imp_->is_running_ = false;

		// 清除所有指令，并回收所有指令 //
		for (int i = 0; i < Imp::CHANEL_SIZE; ++i) {
			imp_->chanels[i].cmd_now_.store(imp_->chanels[i].cmd_end_.load());
			while (imp_->chanels[i].cmd_collect_.load() < imp_->chanels[i].cmd_end_.load()) { std::this_thread::yield(); }
		}
		imp_->is_collect_running_ = false;
		imp_->collect_thread_.join();

		// 停止控制器 //
		master().stop();
	}
	auto ControlServer::waitForAllExecution(int chanel)->void {
		// pre process //
		chanel = chanel == -1 ? defaultChanel() : chanel;
		while (imp_->chanels[chanel].cmd_end_.load() != imp_->chanels[chanel].cmd_now_.load())std::this_thread::sleep_for(std::chrono::milliseconds(1));//原子操作
	}
	auto ControlServer::waitForAllCollection(int chanel)->void {
		// pre process //
		chanel = chanel == -1 ? defaultChanel() : chanel;
		while (imp_->chanels[chanel].cmd_end_.load() != imp_->chanels[chanel].cmd_collect_.load()) std::this_thread::sleep_for(std::chrono::milliseconds(1));//原子操作
	}
	auto ControlServer::currentExecutePlan(int chanel)->std::shared_ptr<aris::plan::Plan>{
		// pre process //
		chanel = chanel == -1 ? defaultChanel() : chanel;

		std::unique_lock<std::recursive_mutex> running_lck(imp_->mu_collect_);
		if (!imp_->is_running_)
			THROW_FILE_LINE("failed to get current TARGET, because ControlServer is not running");

		auto execute_internal = imp_->chanels[chanel].internal_data_queue_[imp_->chanels[chanel].cmd_now_.load() % Imp::CMD_POOL_SIZE];
		return execute_internal ? execute_internal->plan_ : std::shared_ptr<aris::plan::Plan>();
	}
	auto ControlServer::getRtData(const std::function<void(ControlServer&, const aris::plan::Plan *, std::any&)>& get_func, std::any& data)->void
	{
		std::unique_lock<std::recursive_mutex> running_lck(imp_->mu_running_);
		if (!imp_->is_running_)
			THROW_FILE_LINE("failed getRtData");

		imp_->get_data_func_ = &get_func;
		imp_->get_data_ = &data;

		imp_->if_get_data_ready_.store(false);
		imp_->if_get_data_.store(true);

		while (!imp_->if_get_data_ready_.load()) std::this_thread::sleep_for(std::chrono::milliseconds(1));

		imp_->if_get_data_ready_.store(false);
	}
	auto ControlServer::setErrorCode(std::int32_t err_code, const char* err_msg)noexcept->void {
		union { std::int64_t err_code_and_fixed; struct { std::int32_t code; std::int32_t fix; } err; };
		err.code = err_code;
		imp_->err_code_and_fixed_.store(err_code_and_fixed);
		if (err_msg)std::strcpy(imp_->err_msg_, err_msg);
	}
	auto ControlServer::errorCode()const noexcept->int {
		union { std::int64_t err_code_and_fixed; struct { std::int32_t err_code; std::int32_t is_fixed; } err; };
		err_code_and_fixed = imp_->err_code_and_fixed_.load();
		return err.err_code;
	}
	auto ControlServer::clearError()->void 
	{ 
		std::unique_lock<std::recursive_mutex> running_lck(imp_->mu_running_);
		if (!imp_->is_running_)
		{
			imp_->err_code_and_fixed_.store(0);
			std::fill_n(imp_->err_msg_, 1024, '\0');
		}
		else
		{
			//while (imp_->err_code_and_fixed_.load())
			//{
			//	union { std::int64_t err_code_and_fixed; struct { std::int32_t err_code; std::int32_t is_fixed; } err; };
			//	err.err_code = 0;
			//	err.is_fixed = 0xFFFF'FFFF;
			//	imp_->err_code_and_fixed_ &= err_code_and_fixed;
			//	std::this_thread::sleep_for(std::chrono::nanoseconds(master().samplePeriodNs()));
			//}

			// 本函数只负责清理code标志位，fix标志位系统内部会自行清理掉 //
			union { std::int64_t err_code_and_fixed; struct { std::int32_t err_code; std::int32_t is_fixed; } err; };
			err.err_code = 0;
			err.is_fixed = 0xFFFF'FFFF;
			imp_->err_code_and_fixed_ &= err_code_and_fixed;
			std::fill_n(imp_->err_msg_, 1024, '\0');
		}
	}
	ControlServer::~ControlServer() { 
		close();
		if(running())stop();
	}
	ControlServer::ControlServer() :imp_(new Imp(this))	{
		// create members //
		makeModel<aris::dynamic::Model>();
		makeMaster<aris::control::Master>();
		makeController<aris::control::Controller>("controller");
		makePlanRoot<aris::plan::PlanRoot>("plan_root");
		makeTransferModelController<TransferModelController>();
	}



	ARIS_REGISTRATION {
		aris::core::class_<CustomModule>("CustomModule");

		typedef aris::control::Master &(ControlServer::*MasterFunc)();
		typedef aris::control::Controller &(ControlServer::*ControllerFunc)();
		typedef aris::dynamic::ModelBase &(ControlServer::*ModelFunc)();
		typedef aris::plan::PlanRoot &(ControlServer::*PlanRootFunc)();
		typedef TransferModelController& (ControlServer::* TransferModelControllerFunc)();
		typedef aris::core::PointerArray<aris::server::Interface>&(ControlServer::*InterfacePoolFunc)();
		typedef aris::server::MiddleWare &(ControlServer::*MiddleWareFunc)();
		typedef aris::server::CustomModule &(ControlServer::*CustomModuleFunc)();

		aris::core::class_<ControlServer>("ControlServer")
			.prop("model",         &ControlServer::resetModel, ModelFunc(&ControlServer::model))
			.prop("master",        &ControlServer::resetMaster, MasterFunc(&ControlServer::master))
			.prop("controller",    &ControlServer::resetController, ControllerFunc(&ControlServer::controller))
			.prop("plan_root",     &ControlServer::resetPlanRoot, PlanRootFunc(&ControlServer::planRoot))
			.prop("interface",     &ControlServer::resetInterfacePool, InterfacePoolFunc(&ControlServer::interfacePool))
			.prop("middle_ware",   &ControlServer::resetMiddleWare, MiddleWareFunc(&ControlServer::middleWare))
			.prop("model", &ControlServer::resetModel, ModelFunc(&ControlServer::model))
			.prop("master", &ControlServer::resetMaster, MasterFunc(&ControlServer::master))
			.prop("controller", &ControlServer::resetController, ControllerFunc(&ControlServer::controller))
			.prop("plan_root", &ControlServer::resetPlanRoot, PlanRootFunc(&ControlServer::planRoot))
			.prop("model_controller_transfer", &ControlServer::resetTransferModelController, TransferModelControllerFunc(&ControlServer::transferModelController))
			.prop("interface", &ControlServer::resetInterfacePool, InterfacePoolFunc(&ControlServer::interfacePool))
			.prop("middle_ware", &ControlServer::resetMiddleWare, MiddleWareFunc(&ControlServer::middleWare))
			.prop("custom_module", &ControlServer::resetCustomModule, CustomModuleFunc(&ControlServer::customModule))
			.prop("error_checker", &ControlServer::resetErrorChecker, &ControlServer::errorChecker)
			;
	}
}