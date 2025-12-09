#include"aris/plan/multimodel_async_planner.hpp"
#include"aris/plan/function.hpp"
#include"aris/control/rt_timer.hpp"
#include"aris/plan/input_smoother.hpp"
#include"aris/plan/async_generator.hpp"
#include"aris/plan/speed_regulator.hpp"

namespace aris::plan {


	// 根据 tools 和 wobjs 计算反解前每个tools 和 wobjs的设置顺序，连接地面的最先设置，之后连接已经设置的part的再进行设置
	// 
	// 可能报错
	// 
	// 
	//
	auto computeToolWobjOrder(aris::Size ee_size, aris::Size part_size, aris::dynamic::Part** parts, aris::dynamic::MotionBase** ees, aris::dynamic::Marker** tools, aris::dynamic::Marker** wobjs, char *mem_need, aris::Size* order, bool* set_tool) -> int {
		// Step 0 给定变量 //
		Size *parts_setted = reinterpret_cast<Size*>(mem_need);
		std::fill_n(parts_setted, part_size, 0);
		
		// Step 1 补全所有的tool wobj //
		for (auto i = 0; i < ee_size; ++i) {
			tools[i] = tools[i] ? tools[i] : ees[i]->makI();
			wobjs[i] = wobjs[i] ? wobjs[i] : ees[i]->makJ();
		}

		// Step 2 计算顺序 //
		// sub 1：找到所有连接地面的 part
		// sub 2：依次连接其他 part
		// sub 3：若还有不连已有part的tw，则加入第一组tw
		{
			std::iota(order, order + ee_size, 0);
			auto parts_num_ = 0;

			for (int i = 0; i < ee_size; ++i) {
				// 如果 wobj 是ground，则把 tool 定义成需要被设置的 part
				auto found = std::find_if(order + i, order + ee_size, [wobjs](const aris::Size& idx) {
					return (&wobjs[idx]->fatherPart() == &wobjs[idx]->model()->ground());
					});

				if (found < order + ee_size) {
					auto found_part = std::find(parts, parts + part_size, &tools[*found]->fatherPart());
					if (found_part >= parts + part_size)
						return -2002;

					if (parts_setted[found_part - parts] > 0)
						return -2001;

					parts_setted[found_part - parts] = 1;
					std::swap(order[i], *found);
					set_tool[i] = true;
					continue;
				}
				
				// 如果 tool 是ground，则把 wobj 定义成需要被设置的 part
				found = std::find_if(order + i, order + ee_size, [tools](const aris::Size& idx) {
					return (&tools[idx]->fatherPart() == &tools[idx]->model()->ground());
					});

				if (found < order + ee_size) {
					auto found_part = std::find(parts, parts + part_size, &wobjs[*found]->fatherPart());
					if (found_part >= parts + part_size)
						return -2002;

					if (parts_setted[found_part - parts] > 0)
						return -2001;

					parts_setted[found_part - parts] = 1;
					std::swap(order[i], *found);
					set_tool[i] = false;
					continue;
				}

				// 如果 ee 的 makj 被设置过，则把它的 maki 设置一下
				found = std::find_if(order + i, order + ee_size, [wobjs, parts, part_size, parts_setted](const aris::Size& idx) {
					auto found_part = std::find(parts, parts + part_size, &wobjs[idx]->fatherPart());
					return (found_part < parts + part_size) && parts_setted[found_part - parts] > 0;
					});

				if (found < order + ee_size) {
					auto found_part = std::find(parts, parts + part_size, &tools[*found]->fatherPart());
					if (found_part >= parts + part_size)
						return -2002;

					if (parts_setted[found_part - parts] > 0)
						return -2001;

					parts_setted[found_part - parts] = 1;
					std::swap(order[i], *found);
					set_tool[i] = true;
					continue;
				}

				// 如果 ee 的 maki 被设置过，则把它的 makj 设置一下
				found = std::find_if(order + i, order + ee_size, [tools, parts, part_size, parts_setted](const aris::Size& idx) {
					auto found_part = std::find(parts, parts + part_size, &tools[idx]->fatherPart());
					return (found_part < parts + part_size) && parts_setted[found_part - parts] > 0;
				});

				if (found < order + ee_size) {
					auto found_part = std::find(parts, parts + part_size, &wobjs[*found]->fatherPart());
					if (found_part >= parts + part_size)
						return -2002;

					if (parts_setted[found_part - parts] > 0)
						return -2001;

					parts_setted[found_part - parts] = 1;
					std::swap(order[i], *found);
					set_tool[i] = false;
					continue;
				}

				// 如果 ee 凭空出现，则将其 maki 和 makj 均设置
				// 此时 order 无需改变
				auto found_part_i = std::find(parts, parts + part_size, &tools[i]->fatherPart());
				auto found_part_j = std::find(parts, parts + part_size, &wobjs[i]->fatherPart());
				if (found_part_j >= parts + part_size || found_part_i >= parts + part_size){
					return -2002;
				}
				parts_setted[found_part_i - parts] = 1;
				parts_setted[found_part_j - parts] = 1;
				set_tool[i] = true;
			}

			if (std::find(parts_setted, parts_setted + part_size, 0) < parts_setted + part_size) {
				return -2001;
			}
		}

		return 0;
	}
	
	// 根据 tools 和 wobjs 的相对 pos（pos的表达取决于 ee_types）,来计算对应part的位姿
	auto computePartPmByTwPos(aris::Size ee_size, aris::Size part_size, aris::dynamic::EEType *ee_types, 
		aris::dynamic::Part** parts, aris::dynamic::Marker** tools, aris::dynamic::Marker** wobjs, 
		aris::Size* order, bool* set_tool, const double *twpos, const aris::Size* tw_mem_pos, double *part_pms)->void
	{
		// 初始化所有的 part_pm，因为可能有环，不一定都会被设置，所有需要初始化 //
		for (aris::Size i = 0; i < part_size; ++i) {
			aris::dynamic::s_vc(16, *parts[i]->pm(), part_pms + 16 * i);
		}

		double ground_pm[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
		// 计算所有的part_pm //
		for (aris::Size i = 0, idx = 0; i < ee_size; ++i) {
			auto tw_id = order[i];

			double relative_pm[16];
			s_ee_pos2pm(ee_types[i], twpos + tw_mem_pos[tw_id], relative_pm);

			auto tool_pm_in_part = *tools[tw_id]->prtPm();
			auto wobj_pm_in_part = *wobjs[tw_id]->prtPm();

			auto part_i_id = std::find(parts, parts + part_size, &tools[tw_id]->fatherPart()) - parts;
			auto part_i_pm = part_i_id < part_size ? part_pms + 16 * part_i_id : ground_pm;

			auto part_j_id = std::find(parts, parts + part_size, &wobjs[tw_id]->fatherPart()) - parts;
			auto part_j_pm = part_j_id < part_size ? part_pms + 16 * part_j_id : ground_pm;

			if (set_tool[i]) {
				// 
				// relative = wobj(-1) * tool
				//          = (part_j * wobj_in_prt)^(-1) * part_i * tool_in_part
				//     
				// =>
				// 
				// part_j * wobj_in_prt * relative = part_i * tool_in_part
				// 
				// =>
				// 
				// part_i = part_j * wobj_in_prt * relative * tool_in_part(-1)
				// 
				//
				double result1[16], result2[16];
				aris::dynamic::s_pm_dot_pm(part_j_pm, wobj_pm_in_part, result1);
				aris::dynamic::s_pm_dot_pm(result1, relative_pm, result2);
				aris::dynamic::s_pm_dot_inv_pm(result2, tool_pm_in_part, part_i_pm);
			}
			else {
				// 
				// part_j = part_i * tool_in_part * relative(-1) * wobj_in_prt(-1)
				// 
				double result1[16], result2[16];
				aris::dynamic::s_pm_dot_pm(part_i_pm, tool_pm_in_part, result1);
				aris::dynamic::s_pm_dot_inv_pm(result1, relative_pm, result2);
				aris::dynamic::s_pm_dot_inv_pm(result2, wobj_pm_in_part, part_j_pm);
			}
		}
	}
	
	// 根据 part 的位姿，计算 tools 和 wobjs 的 pos
	// 当 ee 的 makI 和 makJ 作为 tools 和 wobjs 时，可以直接计算 eepos
	// 
	// 
	auto computeTwPosByPartPm(aris::Size ee_size, aris::Size part_size, aris::dynamic::EEType* ee_types,
		aris::dynamic::Part** parts, aris::dynamic::Marker** tools, aris::dynamic::Marker** wobjs,
		const double* part_pms, double* tw_pos)->void
	{
		double ground_pm[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
		// 计算所有的末端位姿 //
		for (aris::Size i = 0, idx = 0; i < ee_size; ++i) {
			auto eei_pm_in_part = *tools[i]->prtPm();
			auto eej_pm_in_part = *wobjs[i]->prtPm();

			auto part_i_id = std::find(parts, parts + part_size, &tools[i]->fatherPart()) - parts;
			auto part_i_pm = part_i_id < part_size ? part_pms + 16 * part_i_id : ground_pm;

			auto part_j_id = std::find(parts, parts + part_size, &wobjs[i]->fatherPart()) - parts;
			auto part_j_pm = part_j_id < part_size ? part_pms + 16 * part_j_id : ground_pm;

			double relative_pm[16], result1[16], result2[16];

			// relative = eej(-1) * eei
			//          = (part_j * eej_in_prt)^(-1) * part_i * eei_in_part
			//          = eej_in_prt(-1) * part_j(-1) * part_i * eei_in_part

			aris::dynamic::s_pm_dot_pm(part_j_pm, eej_pm_in_part, result1);
			aris::dynamic::s_pm_dot_pm(part_i_pm, eei_pm_in_part, result2);
			aris::dynamic::s_inv_pm_dot_pm(result1, result2, relative_pm);

			aris::dynamic::s_ee_pm2pos(ee_types[i], relative_pm, tw_pos + idx);
			idx += s_ee_type_pos_size(ee_types[i]);
		}
	}

	struct ToolWobjSelector::Imp {
		aris::dynamic::MultiModel* model_{ nullptr };

		aris::Size ee_size_{ 0 }, part_size_{ 0 }, ee_pos_size_{0}; // parts_num 不包含地面
		aris::dynamic::MotionBase** ees_; // equals ee_size
		aris::dynamic::Marker** tools_, **wobjs_, **ee_makIs_, **ee_makJs_; // equals ee_size
		aris::dynamic::Part** parts_; // equals parts_num，所有需要被设置的parts，每引进一个ee，就需要设置一个part的位姿，但是可能有环，因此不是ee_size
		
		aris::dynamic::EEType* ee_types_; // equals ee_size
		aris::Size *tw_order_, *ee_order_, *ee_pos_mem_pos_;
		bool* tw_set_tool_, *ee_set_tool_;
		double* part_pms_;
		char* mem_need_;

		std::vector<char> mem_;

		auto allocateMem()->void {
			// 计算 part_set，得到内存大小 //
			std::vector<aris::dynamic::Part*> part_set;
			{
				std::vector<aris::dynamic::MotionBase*> ee_vec(model_->eeSize());
				model_->getEes(ee_vec.data());

				for (auto i = 0; i < model_->eeSize(); ++i) {
					if ((&ee_vec[i]->makI()->fatherPart() != &ee_vec[i]->makI()->model()->ground())
						&&std::find(part_set.begin(), part_set.end(), &ee_vec[i]->makI()->fatherPart()) == part_set.end()) {
						part_set.push_back(&ee_vec[i]->makI()->fatherPart());
					}
					if ((&ee_vec[i]->makJ()->fatherPart() != &ee_vec[i]->makJ()->model()->ground())
						&& std::find(part_set.begin(), part_set.end(), &ee_vec[i]->makJ()->fatherPart()) == part_set.end()) {
						part_set.push_back(&ee_vec[i]->makJ()->fatherPart());
					}
				}
			}
			
			// 计算 ee_pos_size //
			{
				std::vector<aris::dynamic::EEType> ee_types(model_->eeSize());
				model_->getEeTypes(ee_types.data());
				ee_pos_size_ = aris::dynamic::s_ee_type_pos_size(model_->eeSize(), ee_types.data());
			}
			part_size_ = part_set.size();
			ee_size_ = model_->eeSize();

			Size mem_size = 0;

			core::allocMem(mem_size, ees_, ee_size_);
			core::allocMem(mem_size, tools_, ee_size_);
			core::allocMem(mem_size, wobjs_, ee_size_);
			core::allocMem(mem_size, ee_makIs_, ee_size_);
			core::allocMem(mem_size, ee_makJs_, ee_size_);
			core::allocMem(mem_size, parts_, part_size_);
			core::allocMem(mem_size, ee_types_, ee_size_);
			core::allocMem(mem_size, tw_order_, ee_size_);
			core::allocMem(mem_size, tw_set_tool_, ee_size_);
			core::allocMem(mem_size, ee_order_, ee_size_);
			core::allocMem(mem_size, ee_set_tool_, ee_size_);
			core::allocMem(mem_size, ee_pos_mem_pos_, ee_size_);
			core::allocMem(mem_size, part_pms_, part_size_ * 16);
			core::allocMem(mem_size, mem_need_, part_size_ * sizeof(double));

			mem_.resize(mem_size, char(0));

			ees_ = core::getMem(mem_.data(), ees_);
			tools_ = core::getMem(mem_.data(), tools_);
			wobjs_ = core::getMem(mem_.data(), wobjs_);
			ee_makIs_ = core::getMem(mem_.data(), ee_makIs_);
			ee_makJs_ = core::getMem(mem_.data(), ee_makJs_);
			parts_ = core::getMem(mem_.data(), parts_);
			ee_types_ = core::getMem(mem_.data(), ee_types_);
			tw_order_ = core::getMem(mem_.data(), tw_order_);
			tw_set_tool_ = core::getMem(mem_.data(), tw_set_tool_);
			ee_order_ = core::getMem(mem_.data(), ee_order_);
			ee_set_tool_ = core::getMem(mem_.data(), ee_set_tool_);
			ee_pos_mem_pos_ = core::getMem(mem_.data(), ee_pos_mem_pos_);
			part_pms_ = core::getMem(mem_.data(), part_pms_);
			mem_need_ = core::getMem(mem_.data(), mem_need_);

			// 设置 ees、ee_makIs、ee_makJs、parts //
			model_->getEes(ees_);
			for (int i = 0; i < ee_size_; ++i) {
				ee_makIs_[i] = ees_[i]->makI();
				ee_makJs_[i] = ees_[i]->makJ();
			}
			std::copy(part_set.begin(), part_set.end(), parts_);
			
			// 设置 ees_types, ee_order, ee_set_tool //
			model_->getEeTypes(ee_types_);
			computeToolWobjOrder(ee_size_, part_size_, parts_, ees_, ee_makIs_, ee_makJs_, mem_need_, ee_order_, ee_set_tool_);

			// 设置 ee_pos_mem_pos //
			for (aris::Size i = 0, idx = 0; i < ee_size_; ++i) {
				ee_pos_mem_pos_[i] = idx;
				idx += s_ee_type_pos_size(ee_types_[i]);
			}
		}

		auto selectTw(aris::dynamic::Marker** tools, aris::dynamic::Marker** wobjs) -> int {
			std::copy(tools, tools + ee_size_, tools_);
			std::copy(wobjs, wobjs + ee_size_, wobjs_);

			// 计算 tw 顺序 //
			return computeToolWobjOrder(ee_size_, part_size_, parts_, ees_, tools_, wobjs_, mem_need_, tw_order_, tw_set_tool_);
		}
		auto setTwPos(const double* twpos) -> void {
			computePartPmByTwPos(ee_size_, part_size_, ee_types_, parts_, tools_, wobjs_, tw_order_, tw_set_tool_, twpos, ee_pos_mem_pos_, part_pms_);
		}
		auto getTwPos(double* twpos) -> void {
			computeTwPosByPartPm(ee_size_, part_size_, ee_types_, parts_, tools_, wobjs_, part_pms_, twpos);
		}
		auto setEePos(const double* eepos) -> void {
			computePartPmByTwPos(ee_size_, part_size_, ee_types_, parts_, ee_makIs_, ee_makJs_, ee_order_, ee_set_tool_, eepos, ee_pos_mem_pos_, part_pms_);
		}
		auto getEePos(double* eepos) -> void {
			computeTwPosByPartPm(ee_size_, part_size_, ee_types_, parts_, ee_makIs_, ee_makJs_, part_pms_, eepos);
		}
	};

	auto ToolWobjSelector::setModel(aris::dynamic::MultiModel& model) -> void {
		imp_->model_ = &model;
		imp_->allocateMem();
	}
	auto ToolWobjSelector::model() -> aris::dynamic::MultiModel& {
		return *imp_->model_;
	}

	auto ToolWobjSelector::selectTw(aris::dynamic::Marker** tools, aris::dynamic::Marker** wobjs) -> int {
		return imp_->selectTw(tools, wobjs);
	}
	auto ToolWobjSelector::setTwPos(const double* twpos) -> void {
		imp_->setTwPos(twpos);
	}
	auto ToolWobjSelector::getTwPos(double* twpos) -> void {
		imp_->getTwPos(twpos);
	}
	auto ToolWobjSelector::setEePos(const double* eepos) -> void {
		imp_->setEePos(eepos);
	}
	auto ToolWobjSelector::getEePos(double* eepos) -> void {
		imp_->getEePos(eepos);
	}

	ToolWobjSelector::~ToolWobjSelector() {}
	ToolWobjSelector::ToolWobjSelector():imp_(new Imp) {}




#define TW_POOL_SIZE 10000

	struct MultimodelAsyncPlanner::Imp {
		using MarkerVec = std::vector<aris::dynamic::Marker*>;
		using ToolWobjNode = std::tuple<std::int64_t, MarkerVec, MarkerVec>;
		
		std::list<ToolWobjNode> tool_wobjs_;
		std::list<ToolWobjNode>::iterator current_node_;
		std::int64_t id_{ 1 };

		ToolWobjNode tw_pool_[TW_POOL_SIZE];

		MarkerVec last_tool_, last_wobj_;
		std::vector<double> last_tw_pos_;

		TrajectoryGenerator tg_;
		InputSmoother is_;
		AsyncGenerator ag_;
		SpeedRegulator sr_;
		ToolWobjSelector tw_;
		aris::dynamic::MultiModel* model_{nullptr};

		std::vector<char> mem_;

		double* ee_pos_, *tw_pos_;

		auto get_next_input(double* p) -> std::int64_t {
			return sr_.getNextInput(p);
		};
		auto init() -> void {
			is_.setInputGenerator([this](double* p)->std::int64_t {
				auto ret = tg_.getEePosAndMoveDt(tw_pos_);
				
				auto& tw = tw_pool_[ret % TW_POOL_SIZE];
				tw_.selectTw(std::get<1>(tw).data(), std::get<2>(tw).data());
				tw_.setTwPos(tw_pos_);
				tw_.getEePos(ee_pos_);
				
				model_->setOutputPos(ee_pos_);
				if (model_->inverseKinematics())
					std::cout << "ik failed" << std::endl;
				model_->getInputPos(p);

				return ret;
			});

			ag_.setInputGenerator([this](double* p)->std::int64_t {
				return is_.getNextInput(p);
			});

			sr_.setInputGenerator([this](double* p)->std::int64_t {
				return ag_.getNextInput(p);
			});

			std::vector<double> init_ee_pos(model_->inputPosSize());
			model_->getInputPos(init_ee_pos.data());

			tg_.clearAllPos();
			//is_.init(init_ee_pos.data());
			//ag_.init();
			//sr_.init(1.0);
		}

	};

	////////////////// PART 1 config ////////////////
	auto MultimodelAsyncPlanner::setModel(aris::dynamic::MultiModel& model) -> void {
		imp_->model_ = &model;
		imp_->tw_.setModel(model);
		imp_->last_tool_.resize(model.eeSize(), nullptr);
		imp_->last_wobj_.resize(model.eeSize(), nullptr);
		imp_->last_tw_pos_.resize(model.outputPosSize(), 0.0);

		imp_->tg_.setEeTypes(model.getEeTypes());
		
		imp_->is_.setInputSize(model.inputPosSize());
		imp_->ag_.setInputSize(model.inputPosSize());
		imp_->sr_.setInputSize(model.inputPosSize());
	}
	auto MultimodelAsyncPlanner::model() -> aris::dynamic::MultiModel&{
		return *imp_->model_;
	}

	// 配置末端类型 //
	auto MultimodelAsyncPlanner::eeTypes()const -> const std::vector<aris::dynamic::EEType>& {
		return imp_->tg_.eeTypes();
	}

	auto MultimodelAsyncPlanner::inputSize() -> int {
		return imp_->is_.inputSize();
	}

	auto MultimodelAsyncPlanner::setDt(double dt) -> void {
		imp_->tg_.setDt(dt);
		imp_->is_.setDt(dt);
		imp_->sr_.setDt(dt);
		imp_->ag_.setDt(dt);
	}
	auto MultimodelAsyncPlanner::dt() -> double {
		return imp_->tg_.dt();
	}

	auto MultimodelAsyncPlanner::setMaxPos(aris::core::Matrix pos) -> void {
		imp_->is_.setMaxPos(pos);
		imp_->sr_.setMaxPos(pos);
	}
	auto MultimodelAsyncPlanner::maxPos() -> aris::core::Matrix {
		return imp_->is_.maxPos();
	}
	auto MultimodelAsyncPlanner::setMaxVel(aris::core::Matrix vel) -> void {
		imp_->is_.setMaxVel(vel);
		imp_->sr_.setMaxVel(vel);
	}
	auto MultimodelAsyncPlanner::maxVel() -> aris::core::Matrix {
		return imp_->is_.maxVel();
	}
	auto MultimodelAsyncPlanner::setMaxAcc(aris::core::Matrix acc) -> void {
		imp_->is_.setMaxAcc(acc);
		imp_->sr_.setMaxAcc(acc);
	}
	auto MultimodelAsyncPlanner::maxAcc() -> aris::core::Matrix {
		return imp_->is_.maxAcc();
	}
	auto MultimodelAsyncPlanner::setMinPos(aris::core::Matrix pos) -> void {
		imp_->is_.setMinPos(pos);
		imp_->sr_.setMinPos(pos);
	}
	auto MultimodelAsyncPlanner::minPos() -> aris::core::Matrix {
		return imp_->is_.minPos();
	}
	auto MultimodelAsyncPlanner::setMinVel(aris::core::Matrix vel) -> void {
		imp_->is_.setMinVel(vel);
		imp_->sr_.setMinVel(vel);
	}
	auto MultimodelAsyncPlanner::minVel() -> aris::core::Matrix {
		return imp_->is_.minVel();
	}
	auto MultimodelAsyncPlanner::setMinAcc(aris::core::Matrix acc) -> void {
		imp_->is_.setMinAcc(acc);
		imp_->sr_.setMinAcc(acc);
	}
	auto MultimodelAsyncPlanner::minAcc() -> aris::core::Matrix {
		return imp_->is_.minAcc();
	}

	// 笛卡尔空间中重规划个数 //
	auto MultimodelAsyncPlanner::maxReplanNum()const -> int {
		return imp_->tg_.maxReplanNum();
	}
	auto MultimodelAsyncPlanner::setMaxReplanNum(int max_replan_num) -> void {
		imp_->tg_.setMaxReplanNum(max_replan_num);
	}

	// 前瞻个数 //
	auto MultimodelAsyncPlanner::setLookAheadCount(int count) -> void {
		imp_->is_.setLookAheadCount(count);
	}
	auto MultimodelAsyncPlanner::lookAheadCount() -> int {
		return imp_->is_.lookAheadCount();
	}

	// 异步规划时缓存个数 //
	auto MultimodelAsyncPlanner::setCacheSize(int cache_size) -> void {
		imp_->ag_.setCacheSize(cache_size);
	}
	auto MultimodelAsyncPlanner::cacheSize() -> int {
		return imp_->ag_.cacheSize();
	}

	////////////////// PART 2 NRT operation ////////////////
	auto MultimodelAsyncPlanner::allocateMemory() -> void {
		imp_->is_.allocateMemory();
		imp_->ag_.allocateMemory();
		imp_->sr_.allocateMemory();


		Size mem_size = 0;

		core::allocMem(mem_size, imp_->tw_pos_, imp_->model_->eeSize());
		core::allocMem(mem_size, imp_->ee_pos_, imp_->model_->eeSize());

		imp_->mem_.resize(mem_size, char(0));

		imp_->tw_pos_ = core::getMem(imp_->mem_.data(), imp_->tw_pos_);
		imp_->ee_pos_ = core::getMem(imp_->mem_.data(), imp_->ee_pos_);
	}
	auto MultimodelAsyncPlanner::init() -> void {
		imp_->init();
	}
	auto MultimodelAsyncPlanner::stop() -> void {
		imp_->ag_.stop();
	}

	// 插入新的数据，并重规划 //
	auto MultimodelAsyncPlanner::insertLinePos(TW& tool_wobjs, const double* ee_pos, const double* vel, const double* acc, const double* jerk, const double* zone) -> std::int64_t {
		// 获取坐标系 //
		auto ee_size = imp_->model_->eeSize();
		Imp::MarkerVec tools(ee_size, nullptr), wobjs(ee_size, nullptr);

		for (int i = 0; i < std::min(tool_wobjs.size(), ee_size); ++i) {
			tools[i] = imp_->model_->findTool(tool_wobjs[i].first);
			wobjs[i] = imp_->model_->findTool(tool_wobjs[i].second);
		}
		
		// 如果坐标系有变化，重新插入 INIT //
		if (tools != imp_->last_tool_ || wobjs != imp_->last_tool_) {
			std::vector<double> tw_init_pos(imp_->model_->inputPosSize());
			imp_->tw_.selectTw(imp_->last_tool_.data(), imp_->last_wobj_.data());
			imp_->tw_.setTwPos(imp_->last_tw_pos_.data());
			imp_->tw_.selectTw(tools.data(), wobjs.data());
			imp_->tw_.getTwPos(tw_init_pos.data());
			imp_->tg_.insertInitPos(imp_->id_, tw_init_pos.data()); // 下面还会插入坐标系，因此这里不用改变id
		}

		// 正常插入指令 //
		imp_->tg_.insertLinePos(imp_->id_, ee_pos, vel, acc, jerk, zone);
		imp_->tw_pool_[imp_->id_ % TW_POOL_SIZE] = std::make_tuple(imp_->id_, tools, wobjs);
		auto ret = imp_->id_;
		imp_->id_++;
		return ret;
	}

	// 插入新的数据，并重规划 //
	auto MultimodelAsyncPlanner::insertCirclePos(TW& tool_wobjs, const double* ee_pos, const double* mid_pos, const double* vel, const double* acc, const double* jerk, const double* zone) -> void {
		imp_->tg_.insertCirclePos(imp_->id_, ee_pos, mid_pos, vel, acc, jerk, zone);
	}

	// 删除已经不用的数据 //
	auto MultimodelAsyncPlanner::clearUsedPos() -> void {
		imp_->tg_.clearUsedPos();
	}

	// 删除全部数据 //
	auto MultimodelAsyncPlanner::clearAllPos() -> void {
		imp_->tg_.clearAllPos();
	}

	// 当前还剩余的指令数 //
	auto MultimodelAsyncPlanner::unusedPosNum() -> int {
		return imp_->tg_.unusedPosNum();
	}

	// 返回当前所有的节点 id //
	auto MultimodelAsyncPlanner::unusedNodeIds()const -> std::vector<std::int64_t> {
		return imp_->tg_.unusedNodeIds();
	}

	// 调速设置 //
	auto MultimodelAsyncPlanner::setTargetSpeedRatio(double ds) -> void {
		imp_->sr_.setTargetSpeedRatio(ds);
	}
	auto MultimodelAsyncPlanner::targetSpeedRatio() -> double {
		return imp_->sr_.targetSpeedRatio();
	}
	auto MultimodelAsyncPlanner::actualSpeedRatio() -> double {
		return imp_->sr_.actualSpeedRatio();
	}


	////////////////// PART 3 RT operation ////////////////
	auto MultimodelAsyncPlanner::getNextInput(double* p) -> std::int64_t {
		return imp_->get_next_input(p);
	}


	MultimodelAsyncPlanner::~MultimodelAsyncPlanner() {
		stop();
	}
	MultimodelAsyncPlanner::MultimodelAsyncPlanner():imp_(new Imp) {
	}
}
