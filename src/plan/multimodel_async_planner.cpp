#include"aris/plan/multimodel_async_planner.hpp"
#include"aris/plan/function.hpp"
#include"aris/control/rt_timer.hpp"
#include"aris/plan/input_smoother.hpp"
#include"aris/plan/async_generator.hpp"
#include"aris/plan/speed_regulator.hpp"

#include "aris/core/error.hpp"
#include "aris/core/etc.hpp"

//#define DEBUG_ARIS_MMP

#ifdef DEBUG_ARIS_MMP
std::vector<double> input_;
#endif

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
		for (auto i = decltype(ee_size)(0); i < ee_size; ++i) {
			tools[i] = tools[i] ? tools[i] : ees[i]->makI();
			wobjs[i] = wobjs[i] ? wobjs[i] : ees[i]->makJ();
		}

		// Step 2 计算顺序 //
		// sub 1：找到所有连接地面的 part
		// sub 2：依次连接其他 part
		// sub 3：若还有不连已有 part 的 tw ，则加入第一组 tw 
		{
			std::iota(order, order + ee_size, 0);
			auto parts_num_ = 0;

			for (auto i = decltype(ee_size)(0); i < ee_size; ++i) {
				// 如果 wobj 是ground，则把 tool 定义成需要被设置的 part
				auto found = std::find_if(order + i, order + ee_size, [wobjs](const aris::Size& idx) {
					return (&wobjs[idx]->fatherPart() == &wobjs[idx]->model()->ground());
					});

				if (found < order + ee_size) {
					auto found_part = std::find(parts, parts + part_size, &tools[*found]->fatherPart());
					if (found_part >= parts + part_size)
						return ARIS_ERROR_CODE_PLAN_TOOL_WOBJ_PART_UNRWCOGNIZED;

					if (parts_setted[found_part - parts] > 0)
						return ARIS_ERROR_CODE_PLAN_TOOL_WOBJ_PART_SET_TWICE;

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
						return ARIS_ERROR_CODE_PLAN_TOOL_WOBJ_PART_UNRWCOGNIZED;

					if (parts_setted[found_part - parts] > 0)
						return ARIS_ERROR_CODE_PLAN_TOOL_WOBJ_PART_SET_TWICE;

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
						return ARIS_ERROR_CODE_PLAN_TOOL_WOBJ_PART_UNRWCOGNIZED;

					if (parts_setted[found_part - parts] > 0)
						return ARIS_ERROR_CODE_PLAN_TOOL_WOBJ_PART_SET_TWICE;

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
						return ARIS_ERROR_CODE_PLAN_TOOL_WOBJ_PART_UNRWCOGNIZED;

					if (parts_setted[found_part - parts] > 0)
						return ARIS_ERROR_CODE_PLAN_TOOL_WOBJ_PART_SET_TWICE;

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
					return ARIS_ERROR_CODE_PLAN_TOOL_WOBJ_PART_UNRWCOGNIZED;
				}
				parts_setted[found_part_i - parts] = 1;
				parts_setted[found_part_j - parts] = 1;
				set_tool[i] = true;
			}

			if (std::find(parts_setted, parts_setted + part_size, 0) < parts_setted + part_size) {
				return ARIS_ERROR_CODE_PLAN_TOOL_WOBJ_PART_SET_TWICE;
			}
		}

		return 0;
	}
	
	// 根据 tools 和 wobjs 的相对 pos（pos的表达取决于 ee_types）,来计算对应part的位姿
	auto computePartPmByTwPos(aris::Size ee_size, aris::Size part_size, aris::dynamic::PosType *ee_types, 
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
			s_pos2pm(ee_types[i], twpos + tw_mem_pos[tw_id], relative_pm);

			auto tool_pm_in_part = *tools[tw_id]->prtPm();
			auto wobj_pm_in_part = *wobjs[tw_id]->prtPm();

			auto part_i_id = std::find(parts, parts + part_size, &tools[tw_id]->fatherPart()) - parts;
			auto part_i_pm = part_i_id < static_cast<int>(part_size) ? part_pms + 16 * part_i_id : ground_pm;

			auto part_j_id = std::find(parts, parts + part_size, &wobjs[tw_id]->fatherPart()) - parts;
			auto part_j_pm = part_j_id < static_cast<int>(part_size) ? part_pms + 16 * part_j_id : ground_pm;

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
	auto computeTwPosByPartPm(aris::Size ee_size, aris::Size part_size, aris::dynamic::PosType* ee_types,
		aris::dynamic::Part** parts, aris::dynamic::Marker** tools, aris::dynamic::Marker** wobjs,
		const double* part_pms, double* tw_pos)->void
	{
		double ground_pm[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
		// 计算所有的末端位姿 //
		for (aris::Size i = 0, idx = 0; i < ee_size; ++i) {
			auto eei_pm_in_part = *tools[i]->prtPm();
			auto eej_pm_in_part = *wobjs[i]->prtPm();

			auto part_i_id = std::find(parts, parts + part_size, &tools[i]->fatherPart()) - parts;
			auto part_i_pm = part_i_id < static_cast<int>(part_size) ? part_pms + 16 * part_i_id : ground_pm;

			auto part_j_id = std::find(parts, parts + part_size, &wobjs[i]->fatherPart()) - parts;
			auto part_j_pm = part_j_id < static_cast<int>(part_size) ? part_pms + 16 * part_j_id : ground_pm;

			double relative_pm[16], result1[16], result2[16];

			// relative = eej(-1) * eei
			//          = (part_j * eej_in_prt)^(-1) * part_i * eei_in_part
			//          = eej_in_prt(-1) * part_j(-1) * part_i * eei_in_part

			aris::dynamic::s_pm_dot_pm(part_j_pm, eej_pm_in_part, result1);
			aris::dynamic::s_pm_dot_pm(part_i_pm, eei_pm_in_part, result2);
			aris::dynamic::s_inv_pm_dot_pm(result1, result2, relative_pm);

			aris::dynamic::s_pm2pos(relative_pm, ee_types[i], tw_pos + idx);
			idx += s_pos_type_size(ee_types[i]);
		}
	}

	struct ToolWobjSelector::Imp {
		aris::dynamic::MultiModel* model_{ nullptr };
		std::vector<aris::Size> sub_id_list_;

		aris::Size ee_size_{ 0 }, part_size_{ 0 }, ee_pos_size_{0}; // parts_num 不包含地面
		aris::dynamic::MotionBase** ees_; // equals ee_size
		aris::dynamic::Marker** tools_, **wobjs_, **ee_makIs_, **ee_makJs_; // equals ee_size
		aris::dynamic::Part** parts_; // equals parts_num，所有需要被设置的parts，每引进一个ee，就需要设置一个part的位姿，但是可能有环，因此不是ee_size
		
		aris::dynamic::PosType* ee_types_; // equals ee_size
		aris::Size *tw_order_, *ee_order_, *ee_pos_mem_pos_;
		bool* tw_set_tool_, *ee_set_tool_;
		double* part_pms_;
		char* mem_need_;

		std::vector<char> mem_;

		auto allocateMem()->void {
			ee_size_ = model_->subOutputSize(sub_id_list_.size(), sub_id_list_.data());
			
			// 计算 part_set，得到内存大小 //
			std::vector<aris::dynamic::Part*> part_set;
			{
				std::vector<aris::dynamic::MotionBase*> ee_vec(ee_size_);
				model_->getSubOutputMotions(sub_id_list_.size(), sub_id_list_.data(), ee_vec.data());

				for (auto i = decltype(ee_size_)(0); i < ee_size_; ++i) {
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
				std::vector<aris::dynamic::PosType> ee_types(ee_size_);
				model_->getSubOutputPosTypes(sub_id_list_.size(), sub_id_list_.data(), ee_types.data());
				ee_pos_size_ = aris::dynamic::s_pos_type_size(ee_size_, ee_types.data());
			}
			part_size_ = part_set.size();
			

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
			model_->getSubOutputMotions(sub_id_list_.size(), sub_id_list_.data(), ees_);
			for (auto i = decltype(ee_size_)(0); i < ee_size_; ++i) {
				ee_makIs_[i] = ees_[i]->makI();
				ee_makJs_[i] = ees_[i]->makJ();
			}
			std::copy(part_set.begin(), part_set.end(), parts_);
			
			// 设置 ees_types, ee_order, ee_set_tool //
			model_->getSubOutputPosTypes(sub_id_list_.size(), sub_id_list_.data(), ee_types_);
			computeToolWobjOrder(ee_size_, part_size_, parts_, ees_, ee_makIs_, ee_makJs_, mem_need_, ee_order_, ee_set_tool_);

			// 设置 ee_pos_mem_pos //
			for (aris::Size i = 0, idx = 0; i < ee_size_; ++i) {
				ee_pos_mem_pos_[i] = idx;
				idx += s_pos_type_size(ee_types_[i]);
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
		imp_->sub_id_list_.clear();
		imp_->sub_id_list_.resize(model.subModels().size());
		std::iota(imp_->sub_id_list_.begin(), imp_->sub_id_list_.end(), 0);

		imp_->allocateMem();
	}
	auto ToolWobjSelector::model() -> aris::dynamic::MultiModel& {
		return *imp_->model_;
	}

	auto ToolWobjSelector::setSubModelId(std::vector<aris::Size> id_list) -> void {
		imp_->sub_id_list_ = id_list;
		imp_->allocateMem();
	}
	auto ToolWobjSelector::subModelId() -> const std::vector<aris::Size>& {
		return imp_->sub_id_list_;
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

	enum class MAPNodeType {
		ResetInitPos,
		Line,
		Circle,
	};
	struct MAPNode {
		std::int64_t id_{ 0 };
		MAPNodeType type_{ MAPNodeType::ResetInitPos };
		std::vector<double> ee_pos_, mid_pos_, vel_, acc_, jerk_, zone_;
	};

#define TW_POOL_SIZE 10000

	struct MultimodelPlanner::Imp {
		using MarkerVec = std::vector<aris::dynamic::Marker*>;
		using ToolWobjNode = std::tuple<std::int64_t, MarkerVec, MarkerVec>;
		
		std::list<ToolWobjNode> tool_wobjs_;
		std::list<ToolWobjNode>::iterator current_node_;
		std::int64_t id_{ 1 };

		ToolWobjNode tw_pool_[TW_POOL_SIZE];

		MarkerVec last_tool_, last_wobj_;
		std::vector<double> last_tw_pos_;

		std::vector<aris::Size> sub_id_list_;
		aris::Size psize_{ 0 }, vdim_{ 0 }; // zone 和 a 的dim 同v

		bool is_aysnc_{ false };

		TrajectoryGenerator tg_;
		InputSmoother is_;
		AsyncGenerator ag_;
		SpeedRegulator sr_;
		ToolWobjSelector tw_, tw_rt_;
		aris::dynamic::MultiModel* model_{nullptr};

		std::vector<char> mem_;

		double* ee_pos_{ nullptr }, * tw_pos_{ nullptr };
		aris::dynamic::MotionBase** ees_;

		std::list<MAPNode> nodes_;

		auto get_next_input(double* p) -> std::int64_t {
			return sr_.getNextInput(p);
		};
		auto allocateMemory() -> void {
			// init tw... //
			tw_.setModel(*model_);
			tw_.setSubModelId(sub_id_list_);
			tw_rt_.setModel(*model_);
			tw_rt_.setSubModelId(sub_id_list_);

			auto ee_size = model_->subOutputSize(sub_id_list_.size(), sub_id_list_.data());
			psize_ = model_->subOutputPosSize(sub_id_list_.size(), sub_id_list_.data());
			vdim_ = model_->subOutputPosMagSize(sub_id_list_.size(), sub_id_list_.data());

			auto input_psize = model_->subInputPosSize(sub_id_list_.size(), sub_id_list_.data());

			last_tool_.resize(ee_size, nullptr);
			last_wobj_.resize(ee_size, nullptr);
			last_tw_pos_.resize(psize_, 0.0);

			// init tg //
			std::vector<aris::dynamic::PosType> ee_pos_types(model_->subOutputSize(sub_id_list_.size(), sub_id_list_.data()));
			model_->getSubOutputPosTypes(sub_id_list_.size(), sub_id_list_.data(), ee_pos_types.data());
			tg_.setEeTypes(ee_pos_types);

			is_.setInputSize(input_psize);
			ag_.setInputSize(input_psize);
			sr_.setInputSize(input_psize);

			is_.allocateMemory();
			ag_.allocateMemory();
			sr_.allocateMemory();

			// allocate mem //
			Size mem_size = 0;

			core::allocMem(mem_size, tw_pos_, psize_);
			core::allocMem(mem_size, ee_pos_, psize_);
			core::allocMem(mem_size, ees_, ee_size);

			mem_.resize(mem_size, char(0));

			tw_pos_ = core::getMem(mem_.data(), tw_pos_);
			ee_pos_ = core::getMem(mem_.data(), ee_pos_);
			ees_ = core::getMem(mem_.data(), ees_);

			model_->getSubOutputMotions(sub_id_list_.size(), sub_id_list_.data(), ees_);

			// 更新 tw 仓 //
			for (int i = 0; i < TW_POOL_SIZE; ++i) {
				auto& tw = tw_pool_[i];
				std::get<1>(tw).resize(input_psize, nullptr);
				std::get<2>(tw).resize(input_psize, nullptr);
			}

			// 设置回调 //
			is_.setInputGenerator([this](double* p)->std::int64_t {
				auto ret = tg_.getEePosAndMoveDt(tw_pos_);

				auto& tw = tw_pool_[ret % TW_POOL_SIZE];
				tw_rt_.selectTw(std::get<1>(tw).data(), std::get<2>(tw).data());
				tw_rt_.setTwPos(tw_pos_);
				tw_rt_.getEePos(ee_pos_);

				model_->setSubOutputPos(sub_id_list_.size(), sub_id_list_.data(), ee_pos_);

				auto ik_ret = model_->subInverseKinematics(sub_id_list_.size(), sub_id_list_.data());
				if (ik_ret)
					return ik_ret;

				model_->getSubInputPos(sub_id_list_.size(), sub_id_list_.data(), p);

#ifdef DEBUG_ARIS_MMP
				::input_.resize(::input_.size() + this->is_.inputSize());
				std::copy(p, p + this->is_.inputSize(), ::input_.end() - this->is_.inputSize());
				if(ret == 0)
					aris::dynamic::dlmwrite(input_.size()/ is_.inputSize(), is_.inputSize(), input_.data(), "/Mac/Home/Documents/MATLAB/test/data_ori.txt");


				static int count_ = 0;
				count_++;
				if (count_ > 100 && count_ < 102) {
					std::cout << "-------------- "<<count_ << "---------" << std::endl;
					std::cout << "tw:" << std::endl;
					aris::dynamic::dsp(1, 6, tw_pos_);
					std::cout << "ee:" << std::endl;
					aris::dynamic::dsp(1, 6, ee_pos_);
					std::cout << "input:" << std::endl;
					aris::dynamic::dsp(1, 6, p);

				}
#endif

				return ret;
				});

			ag_.setInputGenerator([this](double* p)->std::int64_t {
				//static int count_{ 0 };
				//auto ret = is_.getNextInput(p);
				//if (count_++ % 1 == 0) {
				//	std::cout << "ag called: " << count_ <<"  ret:" << ret << std::endl;
				//}
				//return ret;

				return is_.getNextInput(p);
				});


			if (is_aysnc_) {
				sr_.setInputGenerator([this](double* p)->std::int64_t {
					return ag_.getNextInput(p);
				});
			}
			else {
				sr_.setInputGenerator([this](double* p)->std::int64_t {
					return is_.getNextInput(p);
				});
			}
		}
		auto stop() -> void {
			if(is_aysnc_)
				ag_.stop();
		}

		auto init() -> void {
			stop();
			
			nodes_.clear();

			std::vector<double> init_input_pos(model_->subInputPosSize(sub_id_list_.size(), sub_id_list_.data()));
			model_->getSubInputPos(sub_id_list_.size(), sub_id_list_.data(), init_input_pos.data());

			std::vector<double> init_ee_pos(model_->subOutputPosSize(sub_id_list_.size(), sub_id_list_.data()));
			model_->getSubOutputPos(sub_id_list_.size(), sub_id_list_.data(), init_ee_pos.data());

			// 更新 tw //
			std::fill(last_tool_.begin(), last_tool_.end(), nullptr);
			std::fill(last_wobj_.begin(), last_wobj_.end(), nullptr);
			std::copy(init_ee_pos.begin(), init_ee_pos.end(), last_tw_pos_.begin());

			tg_.clearAllPos();
			tg_.insertInitPos(0, init_ee_pos.data());
			
			
			is_.init(init_input_pos.data());
			
			sr_.init(1.0);
			
			if (is_aysnc_) {
				ag_.init();
				ag_.suspend();
			}
		}
		auto insLine(TW& tool_wobjs, const double* ee_pos, const double* vel, const double* acc, const double* jerk, const double* zone) -> std::int64_t {
			// 获取坐标系 //
			auto ee_size = model_->subOutputSize(sub_id_list_.size(), sub_id_list_.data());
			Imp::MarkerVec tools(ee_size, nullptr), wobjs(ee_size, nullptr);

			for (int i = 0; i < static_cast<int>(std::min(tool_wobjs.size(),ee_size)); ++i) {
				tools[i] = model_->findTool(tool_wobjs[i].first);
				wobjs[i] = model_->findWobj(tool_wobjs[i].second);

				// check tool and wobj default value //
				if (tools[i] == nullptr) {
					if (tool_wobjs[i].first == "") {
						tools[i] = ees_[i]->makI();
					}
					else {
						THROW_FILE_LINE(tool_wobjs[i].first + " not found");
					}
				}
				if (wobjs[i] == nullptr) {
					if (tool_wobjs[i].second == "") {
						wobjs[i] = ees_[i]->makJ();
					}
					else {
						THROW_FILE_LINE(tool_wobjs[i].second + " not found");
					}
				}

			}

			tw_pool_[id_ % TW_POOL_SIZE] = std::make_tuple(id_, tools, wobjs);

			// 如果坐标系有变化，重新插入 INIT //
			if (tools != last_tool_ || wobjs != last_wobj_) {
				std::vector<double> tw_init_pos(model_->subOutputPosSize(sub_id_list_.size(), sub_id_list_.data()));
				if(tw_.selectTw(last_tool_.data(), last_wobj_.data()))
					THROW_FILE_LINE("invalid last tool and wobj");
				tw_.setTwPos(last_tw_pos_.data());
				if (tw_.selectTw(tools.data(), wobjs.data()))
					THROW_FILE_LINE("invalid tool and wobj");
				tw_.getTwPos(tw_init_pos.data());
				
				std::copy(tools.begin(), tools.end(), last_tool_.begin());
				std::copy(wobjs.begin(), wobjs.end(), last_wobj_.begin());

				nodes_.push_back({ 
					id_, 
					MAPNodeType::ResetInitPos, 
					tw_init_pos
					});
			}

			// 正常插入指令 //
			nodes_.push_back({
					id_,
					MAPNodeType::Line,
					std::vector<double>(ee_pos, ee_pos + psize_),
					std::vector<double>(),
					std::vector<double>(vel, vel + vdim_),
					std::vector<double>(acc, acc + vdim_),
					std::vector<double>(jerk, jerk + vdim_),
					std::vector<double>(zone, zone + vdim_)
				});

			id_++;
			return id_;
		}
		auto insCircle(TW& tool_wobjs, const double* ee_pos, const double* mid_pos, const double* vel, const double* acc, const double* jerk, const double* zone) -> std::int64_t {
			// 获取坐标系 //
			auto ee_size = model_->subOutputSize(sub_id_list_.size(), sub_id_list_.data());
			Imp::MarkerVec tools(ee_size, nullptr), wobjs(ee_size, nullptr);

			for (int i = 0; i < static_cast<int>(std::min(tool_wobjs.size(), ee_size)); ++i) {
				tools[i] = model_->findTool(tool_wobjs[i].first);
				wobjs[i] = model_->findWobj(tool_wobjs[i].second);

				// check tool and wobj default value //
				if (tools[i] == nullptr) {
					if (tool_wobjs[i].first == "") {
						tools[i] = ees_[i]->makI();
					}
					else {
						THROW_FILE_LINE(tool_wobjs[i].first + " not found");
					}
				}
				if (wobjs[i] == nullptr) {
					if (tool_wobjs[i].second == "") {
						wobjs[i] = ees_[i]->makJ();
					}
					else {
						THROW_FILE_LINE(tool_wobjs[i].second + " not found");
					}
				}
			}

			tw_pool_[id_ % TW_POOL_SIZE] = std::make_tuple(id_, tools, wobjs);

			// 如果坐标系有变化，重新插入 INIT //
			if (tools != last_tool_ || wobjs != last_wobj_) {
				std::vector<double> tw_init_pos(model_->subOutputPosSize(sub_id_list_.size(), sub_id_list_.data()));
				tw_.selectTw(last_tool_.data(), last_wobj_.data());
				tw_.setTwPos(last_tw_pos_.data());
				tw_.selectTw(tools.data(), wobjs.data());
				tw_.getTwPos(tw_init_pos.data());

				std::copy(tools.begin(), tools.end(), last_tool_.begin());
				std::copy(wobjs.begin(), wobjs.end(), last_wobj_.begin());

				nodes_.push_back({
					id_,
					MAPNodeType::ResetInitPos,
					tw_init_pos
					});
			}

			// 正常插入指令 //
			nodes_.push_back({
					id_,
					MAPNodeType::Circle,
					std::vector<double>(ee_pos, ee_pos + psize_),
					std::vector<double>(mid_pos, mid_pos + psize_),
					std::vector<double>(vel, vel + vdim_),
					std::vector<double>(acc, acc + vdim_),
					std::vector<double>(jerk, jerk + vdim_),
					std::vector<double>(zone, zone + vdim_)
				});

			id_++;
			return id_;
		}
		auto updateIns() -> void {
			// 插入数据 //
			for (auto& node : nodes_) {
				switch (node.type_) {
				case MAPNodeType::ResetInitPos:
					tg_.insertInitPos(node.id_, node.ee_pos_.data());
					break;
				case MAPNodeType::Line:
					tg_.insertLinePos(node.id_, node.ee_pos_.data(), node.vel_.data(), node.acc_.data(), node.jerk_.data(), node.zone_.data());
					break;
				case MAPNodeType::Circle:
					tg_.insertCirclePos(node.id_, node.ee_pos_.data(), node.mid_pos_.data(), node.vel_.data(), node.acc_.data(), node.jerk_.data(), node.zone_.data());
					break;
				}
			}

			// resume //
			if (is_aysnc_) {
				ag_.resume();
				while (ag_.cachedDataSize() < 1)
					std::this_thread::sleep_for(std::chrono::milliseconds(1));
				ag_.suspend();
			}
		}
	};

	////////////////// PART 1 config ////////////////
	auto MultimodelPlanner::setModel(aris::dynamic::MultiModel& model) -> void {
		imp_->model_ = &model;
		imp_->sub_id_list_.clear();
		imp_->sub_id_list_.resize(model.subModels().size());
		std::iota(imp_->sub_id_list_.begin(), imp_->sub_id_list_.end(), 0);
	}
	auto MultimodelPlanner::model() -> aris::dynamic::MultiModel&{
		return *imp_->model_;
	}

	auto MultimodelPlanner::setSubModelId(std::vector<aris::Size> id_list) -> void {
		imp_->sub_id_list_ = id_list;
	}
	auto MultimodelPlanner::subModelId() -> const std::vector<aris::Size> &{
		return imp_->sub_id_list_;
	}

	// 配置末端类型 //
	auto MultimodelPlanner::outputPosTypes()const -> const std::vector<aris::dynamic::PosType>& {
		return imp_->tg_.outputPosTypes();
	}

	auto MultimodelPlanner::inputSize() -> int {
		return imp_->is_.inputSize();
	}

	auto MultimodelPlanner::setDt(double dt) -> void {
		imp_->tg_.setDt(dt);
		imp_->is_.setDt(dt);
		imp_->sr_.setDt(dt);
		imp_->ag_.setDt(dt);
	}
	auto MultimodelPlanner::dt() -> double {
		return imp_->tg_.dt();
	}

	auto MultimodelPlanner::setMaxPos(aris::core::Matrix pos) -> void {
		imp_->is_.setMaxPos(pos);
		imp_->sr_.setMaxPos(pos);
	}
	auto MultimodelPlanner::maxPos() -> aris::core::Matrix {
		return imp_->is_.maxPos();
	}
	auto MultimodelPlanner::setMaxVel(aris::core::Matrix vel) -> void {
		imp_->is_.setMaxVel(vel);
		imp_->sr_.setMaxVel(vel);
	}
	auto MultimodelPlanner::maxVel() -> aris::core::Matrix {
		return imp_->is_.maxVel();
	}
	auto MultimodelPlanner::setMaxAcc(aris::core::Matrix acc) -> void {
		imp_->is_.setMaxAcc(acc);
		imp_->sr_.setMaxAcc(acc);
	}
	auto MultimodelPlanner::maxAcc() -> aris::core::Matrix {
		return imp_->is_.maxAcc();
	}
	auto MultimodelPlanner::setMinPos(aris::core::Matrix pos) -> void {
		imp_->is_.setMinPos(pos);
		imp_->sr_.setMinPos(pos);
	}
	auto MultimodelPlanner::minPos() -> aris::core::Matrix {
		return imp_->is_.minPos();
	}
	auto MultimodelPlanner::setMinVel(aris::core::Matrix vel) -> void {
		imp_->is_.setMinVel(vel);
		imp_->sr_.setMinVel(vel);
	}
	auto MultimodelPlanner::minVel() -> aris::core::Matrix {
		return imp_->is_.minVel();
	}
	auto MultimodelPlanner::setMinAcc(aris::core::Matrix acc) -> void {
		imp_->is_.setMinAcc(acc);
		imp_->sr_.setMinAcc(acc);
	}
	auto MultimodelPlanner::minAcc() -> aris::core::Matrix {
		return imp_->is_.minAcc();
	}

	// 笛卡尔空间中重规划个数 //
	auto MultimodelPlanner::maxReplanNum()const -> int {
		return imp_->tg_.maxReplanNum();
	}
	auto MultimodelPlanner::setMaxReplanNum(int max_replan_num) -> void {
		imp_->tg_.setMaxReplanNum(max_replan_num);
	}

	// 前瞻个数 //
	auto MultimodelPlanner::setLookAheadCount(int count) -> void {
		imp_->is_.setLookAheadCount(count);
	}
	auto MultimodelPlanner::lookAheadCount() -> int {
		return imp_->is_.lookAheadCount();
	}
	
	// 是否启用异步规划 //
	auto MultimodelPlanner::setAsync(bool is_async) -> void {
		imp_->is_aysnc_ = is_async;
	}
	auto MultimodelPlanner::isAsync() -> bool {
		return imp_->is_aysnc_;
	}

	// 异步规划时缓存个数 //
	auto MultimodelPlanner::setAsyncCacheSize(int cache_size) -> void {
		imp_->ag_.setCacheSize(cache_size);
	}
	auto MultimodelPlanner::asyncCacheSize() -> int {
		return imp_->ag_.cacheSize();
	}

	////////////////// PART 2 NRT operation ////////////////
	auto MultimodelPlanner::allocateMemory() -> void {
		imp_->allocateMemory();
	}
	auto MultimodelPlanner::init() -> void {
		imp_->init();
	}
	auto MultimodelPlanner::stop() -> void {
		imp_->stop();
	}

	// 插入新的数据，并重规划 //
	auto MultimodelPlanner::insertLinePos(TW& tw, const double* ee_pos, const double* vel, const double* acc, const double* jerk, const double* zone) -> std::int64_t {
		return imp_->insLine(tw, ee_pos, vel, acc, jerk, zone);
	}
	auto MultimodelPlanner::insertLinePos(std::string_view tools, std::string_view wobjs, const double* ee_pos, const double* vel, const double* acc, const double* jerk, const double* zone) -> std::int64_t {
		auto tool_str_vec = aris::core::split(tools, ';');
		auto wobj_str_vec = aris::core::split(wobjs, ';');

		TW tw;
		for (Size i = 0; i < std::max(tool_str_vec.size(), wobj_str_vec.size()); ++i) {
			auto tool = i < tool_str_vec.size() ? aris::core::trimLR(tool_str_vec[i]) : std::string("");
			auto wobj = i < wobj_str_vec.size() ? aris::core::trimLR(wobj_str_vec[i]) : std::string("");
			tw.push_back(std::pair(tool, wobj));
		}

		return insertLinePos(tw, ee_pos, vel, acc, jerk, zone);
	}

	// 插入新的数据，并重规划 //
	auto MultimodelPlanner::insertCirclePos(TW& tw, const double* ee_pos, const double* mid_pos, const double* vel, const double* acc, const double* jerk, const double* zone) -> std::int64_t {
		return imp_->insCircle(tw, ee_pos, mid_pos, vel, acc, jerk, zone);
	}

	auto MultimodelPlanner::updateInsertPos() -> void {
		imp_->updateIns();
	}

	// 删除已经不用的数据 //
	auto MultimodelPlanner::clearUsedPos() -> void {
		imp_->tg_.clearUsedPos();
	}

	// 删除全部数据 //
	auto MultimodelPlanner::clearAllPos() -> void {
		imp_->tg_.clearAllPos();
	}

	// 当前还剩余的指令数 //
	auto MultimodelPlanner::unusedPosNum() -> int {
		return imp_->tg_.unusedPosNum();
	}

	// 返回当前所有的节点 id //
	auto MultimodelPlanner::unusedNodeIds()const -> std::vector<std::int64_t> {
		return imp_->tg_.unusedNodeIds();
	}

	// 调速设置 //
	auto MultimodelPlanner::setTargetSpeedRatio(double ds) -> void {
		imp_->sr_.setTargetSpeedRatio(ds);
	}
	auto MultimodelPlanner::targetSpeedRatio() -> double {
		return imp_->sr_.targetSpeedRatio();
	}
	auto MultimodelPlanner::actualSpeedRatio() -> double {
		return imp_->sr_.actualSpeedRatio();
	}

	////////////////// PART 3 RT operation ////////////////
	auto MultimodelPlanner::getNextInput(double* p) -> std::int64_t {
		return imp_->get_next_input(p);
	}


	MultimodelPlanner::~MultimodelPlanner() {
		stop();
	}
	MultimodelPlanner::MultimodelPlanner():imp_(new Imp) {








	}
}
