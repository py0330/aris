
#include "aris/plan/function.hpp"
#include "aris/plan/input_smoother.hpp"
#include "aris/plan/async_generator.hpp"
#include "aris/plan/speed_regulator.hpp"
#include "aris/plan/multimodel_async_planner.hpp"

#include "aris/core/error.hpp"
#include "aris/core/etc.hpp"
#include "aris/control/rt_timer.hpp"

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
		CartesianInitPos,
		Line,
		Circle,
		JointInitPos,
		MoveJ,
		MoveAbsJ,
	};
	struct MAPNode {
		std::int64_t id_{ 0 };
		MAPNodeType type_{ MAPNodeType::CartesianInitPos };
		int cartesian_planner_idx_{ 0 }; // 0: inv_tg1_, 1: inv_tg2_
		double time_zone_{ 0.0 };

		aris::Size ee_size_{ 0 }, input_psize_{ 0 }, input_vdim_{ 0 }, output_psize_{ 0 }, output_vdim_{ 0 };
		std::vector<char> mem_;

		MAPNode(std::int64_t id, MAPNodeType type, aris::Size ee_size, aris::Size output_psize, aris::Size output_vdim, aris::Size input_psize, aris::Size input_vdim)
			:id_(id), type_(type), ee_size_(ee_size), output_psize_(output_psize), output_vdim_(output_vdim), input_psize_(input_psize), input_vdim_(input_vdim) 
		{
			mem_.resize(
				(output_psize_*3 + output_vdim_*4 + input_psize_ + input_vdim_*4) * sizeof(double) // eepos, twpos, midpos, twvel, twacc, twjerk, zone, jointpos, jointvel, jointacc, jointjerk,jointzone
				 + 2 * ee_size_ * sizeof(aris::dynamic::Marker*) // tools, wobjs
				+ sizeof(std::int64_t) * ee_size_ // whichInverseRoots
				+ sizeof(std::int64_t) * input_psize_ // whichForwardRoots
				+ (output_psize_ + input_psize_) * sizeof(double) // initOutputPos, initInputPos
				,  char(0));
		}
		MAPNode():MAPNode(0, MAPNodeType::CartesianInitPos, 0, 0, 0, 0, 0) {}

		auto init(){
			 std::fill(mem_.begin(), mem_.end(), char(0));
			 id_ = 0;
			 type_ = MAPNodeType::CartesianInitPos;
			 cartesian_planner_idx_ = 0;
			 time_zone_ = 0.0;
		}
		auto isSameTools(aris::dynamic::Marker** tools) -> bool {
			for (aris::Size i = 0; i < ee_size_; ++i) {
				if (tools[i] != this->tools()[i])
					return false;
			}
			return true;
		}
		auto isSameWobjs(aris::dynamic::Marker** wobjs) -> bool {
			for (aris::Size i = 0; i < ee_size_; ++i) {
				if (wobjs[i] != this->wobjs()[i])
					return false;
			}
			return true;
		}

		auto tools()->aris::dynamic::Marker** { return reinterpret_cast<aris::dynamic::Marker**>(mem_.data()); }
		auto wobjs()->aris::dynamic::Marker** { return tools() + ee_size_; }
		auto eePos()->double* { return reinterpret_cast<double*>(wobjs() + ee_size_); }
		auto twPos()->double* { return eePos() + output_psize_; }
		auto twVel()->double* { return twPos() + output_psize_; }
		auto twAcc()->double* { return twVel() + output_vdim_; }
		auto twJerk()->double* { return twAcc() + output_vdim_; }
		auto twZone()->double* { return twJerk() + output_vdim_; }
		auto midPos()->double* { return twZone() + output_vdim_; }

		auto jointPos()->double* { return midPos() + output_psize_; }
		auto jointVel()->double* { return jointPos() + input_psize_; }
		auto jointAcc()->double* { return jointVel() + input_vdim_; }
		auto jointJerk()->double* { return jointAcc() + input_vdim_; }
		auto jointZone()->double* { return jointJerk() + input_vdim_; }

		auto whichInverseRoots()->std::int64_t* { return reinterpret_cast<std::int64_t*>(jointZone() + input_vdim_); }
		auto whichForwardRoots()->std::int64_t* { return reinterpret_cast<std::int64_t*>(whichInverseRoots() + ee_size_); }
		auto initOutputPos()->double* { return reinterpret_cast<double*>(whichForwardRoots() + input_psize_); }
		auto initInputPos()->double* { return initOutputPos() + output_psize_; }
	};

	#define TW_POOL_SIZE 10000

	struct MultimodelPlanner::Imp {
		using MarkerVec = std::vector<aris::dynamic::Marker*>;
		
		// config data //
		aris::dynamic::MultiModel* model_{nullptr};
		std::vector<aris::Size> sub_id_list_;
		bool is_aysnc_{ false };
		
		// allocate data //
		aris::Size ee_size_{ 0 }, output_psize_{ 0 }, output_vdim_{ 0 }, input_psize_{ 0 }, input_vdim_{ 0 };

		// internal data //
		aris::dynamic::MotionBase** ees_;
		double* ee_pos_{ nullptr }, * tw_pos_{ nullptr }, * input_pos_{ nullptr }, * trans_prev_p_{ nullptr };

		std::int64_t get_id_{ 1 }, insert_id_{ 1 };
		std::int64_t ik_ret_{ 0 }, tg_ret_{ 0 };
		MAPNode map_nodes_[TW_POOL_SIZE];
		std::list<MAPNode> nodes_;

		MAPNode last_node_, ins_node_;
		ToolWobjSelector tw_, tw_rt_;

		TrajectoryGenerator inv_tg1_, inv_tg2_, fwd_tg_;
		InputSmoother is_;
		AsyncGenerator ag_;
		SpeedRegulator sr_;

		std::vector<char> mem_;

		auto get_next_input(double* p) -> std::int64_t {
			auto ret = sr_.getNextInput(p);
			// 调用正解更新模型的当前位置 //
			model_->setSubInputPos(sub_id_list_.size(), sub_id_list_.data(), p);
			model_->subForwardKinematics(sub_id_list_.size(), sub_id_list_.data());
			return ret;
		};
		auto allocateMemory() -> void {
			// allocate datas //
			ee_size_ = model_->subOutputSize(sub_id_list_.size(), sub_id_list_.data());
			output_psize_ = model_->subOutputPosSize(sub_id_list_.size(), sub_id_list_.data());
			output_vdim_ = model_->subOutputPosMagSize(sub_id_list_.size(), sub_id_list_.data());
			input_psize_ = model_->subInputPosSize(sub_id_list_.size(), sub_id_list_.data());
			input_vdim_ = input_psize_; // 电机的输入维数和位置维数相同

			// allocate mem //
			Size mem_size = 0;
			core::allocMem(mem_size, ees_, ee_size_);
			core::allocMem(mem_size, tw_pos_, output_psize_);
			core::allocMem(mem_size, ee_pos_, output_psize_);
			core::allocMem(mem_size, input_pos_, input_psize_);
			core::allocMem(mem_size, trans_prev_p_, input_psize_);

			mem_.resize(mem_size, char(0));

			ees_ = core::getMem(mem_.data(), ees_);
			tw_pos_ = core::getMem(mem_.data(), tw_pos_);
			ee_pos_ = core::getMem(mem_.data(), ee_pos_);
			input_pos_ = core::getMem(mem_.data(), input_pos_);
			trans_prev_p_ = core::getMem(mem_.data(), trans_prev_p_);
			
			// nodes //
			for (int i = 0; i < TW_POOL_SIZE; ++i) {
				map_nodes_[i] = MAPNode(0, MAPNodeType::CartesianInitPos, ee_size_, output_psize_, output_vdim_, input_psize_, input_vdim_);
			}
			last_node_ = MAPNode(0, MAPNodeType::CartesianInitPos, ee_size_, output_psize_, output_vdim_, input_psize_, input_vdim_);
			ins_node_ = MAPNode(0, MAPNodeType::CartesianInitPos, ee_size_, output_psize_, output_vdim_, input_psize_, input_vdim_);

			tw_.setModel(*model_);
			tw_.setSubModelId(sub_id_list_);
			tw_rt_.setModel(*model_);
			tw_rt_.setSubModelId(sub_id_list_);

			// init tg //
			std::vector<aris::dynamic::PosType> ee_pos_types(model_->subOutputSize(sub_id_list_.size(), sub_id_list_.data()));
			model_->getSubOutputPosTypes(sub_id_list_.size(), sub_id_list_.data(), ee_pos_types.data());
			inv_tg1_.setPosTypes(ee_pos_types);
			inv_tg1_.allocateMemory();
			inv_tg2_.setPosTypes(ee_pos_types);
			inv_tg2_.allocateMemory();

			std::vector<aris::dynamic::PosType> input_pos_types(model_->subInputSize(sub_id_list_.size(), sub_id_list_.data()));
			model_->getSubInputPosTypes(sub_id_list_.size(), sub_id_list_.data(), input_pos_types.data());
			fwd_tg_.setPosTypes(input_pos_types);
			fwd_tg_.allocateMemory();

			is_.setInputSize(input_psize_);
			ag_.setInputSize(input_psize_);
			sr_.setInputSize(input_psize_);

			is_.allocateMemory();
			ag_.allocateMemory();
			sr_.allocateMemory();

			// 设置回调 //
			is_.setInputGenerator([this](double* p)->std::int64_t {

				// no data //
				if(get_id_ >= insert_id_) {
					std::copy(input_pos_, input_pos_ + input_psize_, p);
					return 0;
				}

				auto& node = map_nodes_[get_id_ % TW_POOL_SIZE];

				auto get_tg = [this](MAPNode& node)->TrajectoryGenerator* {
					if (node.type_ == MAPNodeType::Line || node.type_ == MAPNodeType::Circle || node.type_ == MAPNodeType::CartesianInitPos) {
						return (node.cartesian_planner_idx_ == 0) ? &inv_tg1_ : &inv_tg2_;
					}
					else {
						return &fwd_tg_;
					}
				};

				// default_p 用来表示如果反解失败时所用的值，其就是 input_pos //
				auto getNodePos = [this](MAPNode &node, double* p, double* default_p)->void {
					// Cartesian space move //
					if(node.type_ == MAPNodeType::Line || node.type_ == MAPNodeType::Circle) {
						auto& cart_tg = (node.cartesian_planner_idx_ == 0) ? inv_tg1_ : inv_tg2_;
						tg_ret_ = cart_tg.getEePosAndMoveDt(tw_pos_);

						tw_rt_.selectTw(node.tools(), node.wobjs());
						tw_rt_.setTwPos(tw_pos_);
						tw_rt_.getEePos(ee_pos_);

						// 无状态反解：用 initInputPos 作为初值 //
						ik_ret_ = model_->subInverseKinematics(
							sub_id_list_.size(), sub_id_list_.data(),
							ee_pos_, p, node.whichInverseRoots(), node.initInputPos());

						// 缓存成功结果，失败则返回缓存值 //
						if(ik_ret_ >= 0) {
							std::copy(p, p + input_psize_, default_p);
						} else {
							std::copy(default_p, default_p + input_psize_, p);
						}
					}

					// Joint space move //
					else {
						// 更新 input_pos_（TG 输出总是有效的）//
						tg_ret_ = fwd_tg_.getEePosAndMoveDt(p);
						std::copy(p, p + input_psize_, default_p);
					}
				};

				getNodePos(node, p, input_pos_);

				// 判断是否切换节点 //
				auto tg = get_tg(node);
				auto current_tg_id = tg->currentNodeId();
				auto is_switch_node = tg->isCurrentNodeFinished();

				// time_zone 混合：下一节点使用不同规划器且进入 real_zone 时，同时运行两个规划器并 smoothstep 混合 //
#ifdef DEBUG_ARIS_MMP
				static int tz_check = 0;
				if (tz_check < 3 || get_id_ \!= 1) {
					std::cerr << "[time_zone_check] tz=" << node.time_zone_ << " get_id=" << get_id_
						<< " insert_id=" << insert_id_ << " cond=" << (node.time_zone_ > 0.0 && get_id_ + 1 < insert_id_) << std::endl;
					if (get_id_ \!= 1) tz_check++;
				}
#endif
				if (node.time_zone_ > 0.0 && get_id_ + 1 < insert_id_) {
					auto& next_node = map_nodes_[(get_id_ + 1) % TW_POOL_SIZE];
					auto next_tg = get_tg(next_node);
					auto curr_tg = get_tg(node);

					auto cur_duration = curr_tg->currentNodeDuration();
					auto next_duration = (next_tg->isCurrentNodeMove() && (!next_tg->isCurrentNodeFinished())) 
						? next_tg->currentNodeDuration() 
						: next_tg->nextMoveNodeDuration();

					auto next_max_ta = (next_tg->isCurrentNodeMove() && (!next_tg->isCurrentNodeFinished()))
						? next_tg->currentNodeMaxTa()
						: next_tg->nextMoveNodeMaxTa();

					auto curr_max_tb = curr_tg->currentNodeMaxTb();

#ifdef DEBUG_ARIS_MMP
					std::cout << "[time_zone] cur_dur=" << cur_duration << " next_dur=" << next_duration
						<< " curr_max_tb=" << curr_max_tb << " next_max_ta=" << next_max_ta
						<< " curr_tz=" << node.time_zone_ << " next_tz=" << next_node.time_zone_
						<< " cur_type=" << static_cast<int>(node.type_)
						<< " next_type=" << static_cast<int>(next_node.type_)
						<< " next_isMove=" << next_tg->isCurrentNodeMove()
						<< " next_isFin=" << next_tg->isCurrentNodeFinished() << std::endl;
#endif

					double real_tz = std::min({ node.time_zone_, next_node.time_zone_, cur_duration * 0.5, next_duration * 0.5,
						curr_max_tb, next_max_ta });
					double left_s = curr_tg->leftNodeS();

#ifdef DEBUG_ARIS_MMP
					std::cout << "[time_zone] real_tz=" << real_tz << " left_s=" << left_s
						<< " diff_tg=" << (next_tg != curr_tg)
						<< " blend=" << (next_tg != curr_tg && left_s <= real_tz && real_tz > 0.0) << std::endl;
#endif

					if (next_tg != curr_tg && left_s <= real_tz && real_tz > 0.0) {
						double zone_t = 1.0 - left_s / real_tz;
						getNodePos(next_node, trans_prev_p_, trans_prev_p_);
						// double f = zone_t < 0.5 ? 4*zone_t*zone_t*zone_t : 1.0 - 4.0*std::pow(1.0 - zone_t, 3);
						// for (aris::Size i = 0; i < input_psize_; ++i)
						// 	p[i] = p[i] * (1.0 - f) + trans_prev_p_[i] * f;

						for (aris::Size i = 0; i < input_psize_; ++i)
							p[i] = p[i] + trans_prev_p_[i] - node.jointPos()[i];
					}

					if(is_switch_node) {
						std::copy(trans_prev_p_, trans_prev_p_ + input_psize_, input_pos_);
					}
				}

				// switch node and return //
				get_id_ = tg->isCurrentNodeFinished() ? current_tg_id + 1 : current_tg_id;
				return current_tg_id;
			});

			ag_.setInputGenerator([this](double* p)->std::int64_t {
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
			
			// ees 相关 //
			model_->getSubOutputMotions(sub_id_list_.size(), sub_id_list_.data(), ees_);
			std::fill(ee_pos_, ee_pos_ + output_psize_, 0.0);
			std::fill(tw_pos_, tw_pos_ + output_psize_, 0.0);

			// nodes 相关 //
			get_id_ = 1;
			insert_id_ = 1;
			tg_ret_ = 0;
			ik_ret_ = 0;

			for (int i = 0; i < TW_POOL_SIZE; ++i) {
				map_nodes_[i].init();
			}
			nodes_.clear();

			// 用当前model的数据初始化 last data //
			last_node_ = MAPNode(0, MAPNodeType::CartesianInitPos, ee_size_, output_psize_, output_vdim_, input_psize_, input_vdim_);
			model_->getSubInputPos(sub_id_list_.size(), sub_id_list_.data(), last_node_.jointPos());
			model_->getSubOutputPos(sub_id_list_.size(), sub_id_list_.data(), last_node_.eePos());
			model_->getSubOutputPos(sub_id_list_.size(), sub_id_list_.data(), last_node_.twPos());
			model_->getSubWhichInverseRoot(sub_id_list_.size(), sub_id_list_.data(), last_node_.eePos(), last_node_.jointPos(), last_node_.whichInverseRoots());
			model_->getSubWhichForwardRoot(sub_id_list_.size(), sub_id_list_.data(), last_node_.jointPos(), last_node_.eePos(), last_node_.whichForwardRoots());

			// 初始化缓存 //
			std::copy(last_node_.jointPos(), last_node_.jointPos() + input_psize_, input_pos_);
			std::copy(last_node_.eePos(), last_node_.eePos() + output_psize_, ee_pos_);

			// 初始化 tg 等
			inv_tg1_.clearAllPos();
			inv_tg1_.insertInitPos(0, last_node_.eePos());

			inv_tg2_.clearAllPos();
			inv_tg2_.insertInitPos(0, last_node_.eePos());

			fwd_tg_.clearAllPos();
			fwd_tg_.insertInitPos(0, last_node_.jointPos());

			is_.init(last_node_.jointPos());
			
			sr_.init(1.0);
			
			if (is_aysnc_) {
				ag_.init();
				ag_.suspend();
			}
		}
		auto resolveToolsAndWobjs(TW& tool_wobjs, MarkerVec& tools, MarkerVec& wobjs) -> void {
			for (int i = 0; i < static_cast<int>(std::min(tool_wobjs.size(), tools.size())); ++i) {
				tools[i] = model_->findTool(tool_wobjs[i].first);
				wobjs[i] = model_->findWobj(tool_wobjs[i].second);

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
		}
		auto insertInitNode(MarkerVec& tools, MarkerVec& wobjs, MAPNodeType init_type, int cartesian_planner_idx = 0) -> void {
			// 初始化为上一个节点的数据 //
			ins_node_.init();
			std::copy(last_node_.mem_.begin(), last_node_.mem_.end(), ins_node_.mem_.begin());
			
			// 填入本节点信息 //
			ins_node_.id_ = insert_id_;
			ins_node_.type_ = init_type;
			ins_node_.cartesian_planner_idx_ = cartesian_planner_idx;
			std::copy(tools.begin(), tools.end(), ins_node_.tools());
			std::copy(wobjs.begin(), wobjs.end(), ins_node_.wobjs());

			// 填入 tw 信息 //
			if (tw_.selectTw(last_node_.tools(), last_node_.wobjs()))
				THROW_FILE_LINE("invalid last tool and wobj");
			tw_.setTwPos(last_node_.twPos());
			if (tw_.selectTw(tools.data(), wobjs.data()))
				THROW_FILE_LINE("invalid tool and wobj");
			tw_.getTwPos(ins_node_.twPos());

			// 填入 whichroot 信息 //
			if(last_node_.type_ == MAPNodeType::Line || last_node_.type_ == MAPNodeType::Circle || last_node_.type_ == MAPNodeType::CartesianInitPos) {
				std::copy(last_node_.whichInverseRoots(), last_node_.whichInverseRoots() + sub_id_list_.size(), ins_node_.whichInverseRoots());
				model_->getSubWhichForwardRoot(sub_id_list_.size(), sub_id_list_.data(), ins_node_.jointPos(), ins_node_.eePos(), ins_node_.whichForwardRoots());
			}
			else {
				std::copy(last_node_.whichForwardRoots(), last_node_.whichForwardRoots() + sub_id_list_.size(), ins_node_.whichForwardRoots());
				model_->getSubWhichInverseRoot(sub_id_list_.size(), sub_id_list_.data(), ins_node_.eePos(), ins_node_.jointPos(), ins_node_.whichInverseRoots());
			}

			// 插入 //
			nodes_.push_back(ins_node_);
			std::swap(last_node_, ins_node_);
		}
		auto insLine(TW& tool_wobjs, const double* tw_pos, const double* vel, const double* acc, const double* jerk, const double* zone, double time_zone = 0.0) -> std::int64_t {
			
			auto ee_size = model_->subOutputSize(sub_id_list_.size(), sub_id_list_.data());
			
			// 获取坐标系 //
			Imp::MarkerVec tools(ee_size, nullptr), wobjs(ee_size, nullptr);
			resolveToolsAndWobjs(tool_wobjs, tools, wobjs);

			// 计算 cartesian_planner_idx //
			int cartesian_idx = 0;
			if (last_node_.type_ == MAPNodeType::Line || last_node_.type_ == MAPNodeType::Circle || last_node_.type_ == MAPNodeType::CartesianInitPos) {
				// 从笛卡尔空间来，判断 tool/wobj 是否变化 //
				if (!last_node_.isSameTools(tools.data()) || !last_node_.isSameWobjs(wobjs.data())) {
					cartesian_idx = 1 - last_node_.cartesian_planner_idx_; // 切换规划器
				} else {
					cartesian_idx = last_node_.cartesian_planner_idx_; // 保持
				}
			} else {
				// 从关节空间来，默认使用 inv_tg1_ (0)
				cartesian_idx = 0;
			}

			// 查看是否插入 INIT //
			if ((last_node_.type_ != MAPNodeType::Line && last_node_.type_ != MAPNodeType::Circle && last_node_.type_ != MAPNodeType::CartesianInitPos)
				|| !last_node_.isSameTools(tools.data())
				|| !last_node_.isSameWobjs(wobjs.data())) 
			{
				insertInitNode(tools, wobjs, MAPNodeType::CartesianInitPos, cartesian_idx);
			}

			// 更新 last_tw_pos_ 等 //
			ins_node_.init();
			ins_node_.id_ = insert_id_;
			ins_node_.type_ = MAPNodeType::Line;
			ins_node_.cartesian_planner_idx_ = cartesian_idx;
			ins_node_.time_zone_ = time_zone;
			std::copy(tools.begin(), tools.end(), ins_node_.tools());
			std::copy(wobjs.begin(), wobjs.end(), ins_node_.wobjs());
			std::copy(tw_pos, tw_pos + output_psize_, ins_node_.twPos());
			std::copy(vel, vel + output_vdim_, ins_node_.twVel());
			std::copy(acc, acc + output_vdim_, ins_node_.twAcc());
			std::copy(jerk, jerk + output_vdim_, ins_node_.twJerk());
			std::copy(zone, zone + output_vdim_, ins_node_.twZone());

			// 记录 init 位置（last_node_ 结束时的位置作为本节点初值）//
			std::copy(last_node_.eePos(), last_node_.eePos() + output_psize_, ins_node_.initOutputPos());
			std::copy(last_node_.jointPos(), last_node_.jointPos() + input_psize_, ins_node_.initInputPos());

			// 做反解计算 //
			{
				tw_.selectTw(tools.data(), wobjs.data());
				tw_.setTwPos(ins_node_.twPos());
				tw_.getEePos(ins_node_.eePos());

				std::copy(last_node_.whichInverseRoots(), last_node_.whichInverseRoots() + sub_id_list_.size(), ins_node_.whichInverseRoots());
				model_->getSubWhichForwardRoot(sub_id_list_.size(), sub_id_list_.data(), ins_node_.jointPos(), ins_node_.eePos(), ins_node_.whichForwardRoots());

				auto ret = model_->subInverseKinematics(
					sub_id_list_.size(), sub_id_list_.data(),
					ins_node_.eePos(), ins_node_.jointPos(), ins_node_.whichInverseRoots(), ins_node_.jointPos());

				if(ret < 0)
					return ret;
			}
			

			// 正常插入指令 //
			nodes_.push_back(ins_node_);
			std::swap(last_node_, ins_node_);

			insert_id_++;
			return insert_id_ - 1;
		}
		auto insCircle(TW& tool_wobjs, const double* tw_pos, const double* tw_mid_pos, const double* vel, const double* acc, const double* jerk, const double* zone, double time_zone = 0.0) -> std::int64_t {
			
			auto ee_size = model_->subOutputSize(sub_id_list_.size(), sub_id_list_.data());

			// 获取坐标系 //
			Imp::MarkerVec tools(ee_size, nullptr), wobjs(ee_size, nullptr);
			resolveToolsAndWobjs(tool_wobjs, tools, wobjs);

			// 计算 cartesian_planner_idx //
			int cartesian_idx = 0;
			if (last_node_.type_ == MAPNodeType::Line || last_node_.type_ == MAPNodeType::Circle || last_node_.type_ == MAPNodeType::CartesianInitPos) {
				// 从笛卡尔空间来，判断 tool/wobj 是否变化 //
				if (!last_node_.isSameTools(tools.data()) || !last_node_.isSameWobjs(wobjs.data())) {
					cartesian_idx = 1 - last_node_.cartesian_planner_idx_; // 切换规划器
				} else {
					cartesian_idx = last_node_.cartesian_planner_idx_; // 保持
				}
			} else {
				// 从关节空间来，默认使用 inv_tg1_ (0)
				cartesian_idx = 0;
			}
			
			// 查看是否插入 INIT //
			if ((last_node_.type_ != MAPNodeType::Line && last_node_.type_ != MAPNodeType::Circle && last_node_.type_ != MAPNodeType::CartesianInitPos)
				|| !last_node_.isSameTools(tools.data())
				|| !last_node_.isSameWobjs(wobjs.data())) 
			{
				insertInitNode(tools, wobjs, MAPNodeType::CartesianInitPos, cartesian_idx);
			}

			// 更新 last_tw_pos_ 等 //
			ins_node_.init();
			ins_node_.id_ = insert_id_;
			ins_node_.type_ = MAPNodeType::Circle;
			ins_node_.cartesian_planner_idx_ = cartesian_idx;
			ins_node_.time_zone_ = time_zone;
			std::copy(tools.begin(), tools.end(), ins_node_.tools());
			std::copy(wobjs.begin(), wobjs.end(), ins_node_.wobjs());
			std::copy(tw_pos, tw_pos + output_psize_, ins_node_.twPos());
			std::copy(tw_mid_pos, tw_mid_pos + output_psize_, ins_node_.midPos());
			std::copy(vel, vel + output_vdim_, ins_node_.twVel());
			std::copy(acc, acc + output_vdim_, ins_node_.twAcc());
			std::copy(jerk, jerk + output_vdim_, ins_node_.twJerk());
			std::copy(zone, zone + output_vdim_, ins_node_.twZone());

			// 记录 init 位置（last_node_ 结束时的位置作为本节点初值）//
			std::copy(last_node_.eePos(), last_node_.eePos() + output_psize_, ins_node_.initOutputPos());
			std::copy(last_node_.jointPos(), last_node_.jointPos() + input_psize_, ins_node_.initInputPos());

			{
				tw_.selectTw(tools.data(), wobjs.data());
				tw_.setTwPos(ins_node_.twPos());
				tw_.getEePos(ins_node_.eePos());

				std::copy(last_node_.whichInverseRoots(), last_node_.whichInverseRoots() + sub_id_list_.size(), ins_node_.whichInverseRoots());
				model_->getSubWhichForwardRoot(sub_id_list_.size(), sub_id_list_.data(), ins_node_.jointPos(), ins_node_.eePos(), ins_node_.whichForwardRoots());

				auto ret = model_->subInverseKinematics(
					sub_id_list_.size(), sub_id_list_.data(),
					ins_node_.eePos(), ins_node_.jointPos(), ins_node_.whichInverseRoots(), ins_node_.jointPos());

				if(ret < 0)
					return ret;
			}
			

			// 正常插入指令 //
			nodes_.push_back(ins_node_);
			std::swap(last_node_, ins_node_);

			insert_id_++;
			return insert_id_ - 1;
		}
		auto insMoveJ(TW& tool_wobjs, const double* tw_pos, const double* joint_v, const double* joint_a, const double* joint_j, const double* zone, const std::int64_t *which_root, double time_zone = 0.0) -> std::int64_t {
			
			// 获取坐标系 //
			Imp::MarkerVec tools(ee_size_, nullptr), wobjs(ee_size_, nullptr);
			resolveToolsAndWobjs(tool_wobjs, tools, wobjs);

			// 查看是否插入 INIT //
			if ((last_node_.type_ != MAPNodeType::MoveJ && last_node_.type_ != MAPNodeType::MoveAbsJ && last_node_.type_ != MAPNodeType::JointInitPos)) {
				insertInitNode(tools, wobjs, MAPNodeType::JointInitPos);
			}

			// 更新 last_tw_pos_ 等 //
			ins_node_.init();
			ins_node_.id_ = insert_id_;
			ins_node_.type_ = MAPNodeType::MoveJ;
			ins_node_.time_zone_ = time_zone;
			std::copy(tools.begin(), tools.end(), ins_node_.tools());
			std::copy(wobjs.begin(), wobjs.end(), ins_node_.wobjs());
			std::copy(tw_pos, tw_pos + output_psize_, ins_node_.twPos());
			std::copy(joint_v, joint_v + input_vdim_, ins_node_.jointVel());
			std::copy(joint_a, joint_a + input_vdim_, ins_node_.jointAcc());
			std::copy(joint_j, joint_j + input_vdim_, ins_node_.jointJerk());
			std::copy(zone, zone + input_vdim_, ins_node_.jointZone());

			// 记录 init 位置（last_node_ 结束时的位置作为本节点初值）//
			std::copy(last_node_.eePos(), last_node_.eePos() + output_psize_, ins_node_.initOutputPos());
			std::copy(last_node_.jointPos(), last_node_.jointPos() + input_psize_, ins_node_.initInputPos());

			{
				tw_.selectTw(tools.data(), wobjs.data());
				tw_.setTwPos(ins_node_.twPos());
				tw_.getEePos(ins_node_.eePos());

				if(which_root != nullptr){
					std::copy(which_root, which_root + sub_id_list_.size(), ins_node_.whichInverseRoots());
				}
				else{
					std::fill(ins_node_.whichInverseRoots(), ins_node_.whichInverseRoots() + sub_id_list_.size(), -1);
				}
				model_->getSubWhichForwardRoot(sub_id_list_.size(), sub_id_list_.data(), ins_node_.jointPos(), ins_node_.eePos(), ins_node_.whichForwardRoots());

				auto ret = model_->subInverseKinematics(
					sub_id_list_.size(), sub_id_list_.data(),
					ins_node_.eePos(), ins_node_.jointPos(), ins_node_.whichInverseRoots(), ins_node_.jointPos());
				
				if(ret < 0)
					return ret;
			}

			// 正常插入指令 //
			nodes_.push_back(ins_node_);
			std::swap(last_node_, ins_node_);

			insert_id_++;
			return insert_id_ - 1;
		}
		auto insMoveAbsJ(const double* joint_p, const double* joint_v, const double* joint_a, const double* joint_j, const double* zone, double time_zone = 0.0) -> std::int64_t {

			// 如果运动方式有变化，重新插入 INIT //
			if ((last_node_.type_ != MAPNodeType::MoveJ && last_node_.type_ != MAPNodeType::MoveAbsJ && last_node_.type_ != MAPNodeType::JointInitPos)) 
			{
				auto tools = std::vector<aris::dynamic::Marker*>(ee_size_, nullptr);
				auto wobjs = std::vector<aris::dynamic::Marker*>(ee_size_, nullptr);
				insertInitNode(tools, wobjs, MAPNodeType::JointInitPos);
			}

			// 更新 last_tw_pos_ 等 //
			ins_node_.init();
			ins_node_.id_ = insert_id_;
			ins_node_.type_ = MAPNodeType::MoveAbsJ;
			ins_node_.time_zone_ = time_zone;
			std::copy(joint_p, joint_p + input_psize_, ins_node_.jointPos());
			std::copy(joint_v, joint_v + input_vdim_, ins_node_.jointVel());
			std::copy(joint_a, joint_a + input_vdim_, ins_node_.jointAcc());
			std::copy(joint_j, joint_j + input_vdim_, ins_node_.jointJerk());
			std::copy(zone, zone + input_vdim_, ins_node_.jointZone());

			// 记录 init 位置（last_node_ 结束时的位置作为本节点初值）//
			std::copy(last_node_.eePos(), last_node_.eePos() + output_psize_, ins_node_.initOutputPos());
			std::copy(last_node_.jointPos(), last_node_.jointPos() + input_psize_, ins_node_.initInputPos());

			{
				std::copy(last_node_.whichForwardRoots(), last_node_.whichForwardRoots() + sub_id_list_.size(), ins_node_.whichForwardRoots());
				model_->getSubWhichInverseRoot(sub_id_list_.size(), sub_id_list_.data(), ins_node_.eePos(), ins_node_.jointPos(), ins_node_.whichInverseRoots());
				
				auto ret = model_->subForwardKinematics(
					sub_id_list_.size(), sub_id_list_.data(),
					ins_node_.jointPos(), ins_node_.eePos(), ins_node_.whichForwardRoots(), ins_node_.jointPos());

				if(ret < 0)
					return ret;
			}

			// 正常插入指令 //
#ifdef DEBUG_ARIS_MMP
			std::cerr << "[insMoveAbsJ] BEFORE push: ins_node_.tz=" << ins_node_.time_zone_ << " time_zone_param=" << time_zone << " insert_id=" << insert_id_ << std::endl;
#endif
			nodes_.push_back(ins_node_);
#ifdef DEBUG_ARIS_MMP
			std::cerr << "[insMoveAbsJ] AFTER push: nodes_.back().tz=" << nodes_.back().time_zone_ << " size=" << nodes_.size() << std::endl;
#endif
			std::swap(last_node_, ins_node_);

			insert_id_++;
			return insert_id_ - 1;
		}


		auto updateIns() -> void {
			// 插入数据 //
			for (auto& node : nodes_) {
				switch (node.type_) {
				case MAPNodeType::CartesianInitPos:
					(node.cartesian_planner_idx_ == 0 ? inv_tg1_ : inv_tg2_).insertInitPos(node.id_, node.eePos());
					break;
				case MAPNodeType::Line:
					map_nodes_[node.id_ % TW_POOL_SIZE] = node;
					(node.cartesian_planner_idx_ == 0 ? inv_tg1_ : inv_tg2_).insertLinePos(node.id_, node.twPos(), node.twVel(), node.twAcc(), node.twJerk(), node.twZone());
					break;
				case MAPNodeType::Circle:
					map_nodes_[node.id_ % TW_POOL_SIZE] = node;
					(node.cartesian_planner_idx_ == 0 ? inv_tg1_ : inv_tg2_).insertCirclePos(node.id_, node.twPos(), node.midPos(), node.twVel(), node.twAcc(), node.twJerk(), node.twZone());
					break;
				case MAPNodeType::JointInitPos:
					fwd_tg_.insertInitPos(node.id_, node.jointPos());
					break;
				case MAPNodeType::MoveJ:
					map_nodes_[node.id_ % TW_POOL_SIZE] = node;
					fwd_tg_.insertLinePos(node.id_, node.jointPos(), node.jointVel(), node.jointAcc(), node.jointJerk(), node.jointZone());
					break;
				case MAPNodeType::MoveAbsJ:
#ifdef DEBUG_ARIS_MMP
					std::cerr << "[updateIns::MoveAbsJ] node.id=" << node.id_ << " node.tz=" << node.time_zone_ << std::endl;
#endif
					map_nodes_[node.id_ % TW_POOL_SIZE] = node;
					fwd_tg_.insertLinePos(node.id_, node.jointPos(), node.jointVel(), node.jointAcc(), node.jointJerk(), node.jointZone());
					break;
				}
			}

			// 批量插入后统一重规划提交
			inv_tg1_.updateInsertPos();
			inv_tg2_.updateInsertPos();
			fwd_tg_.updateInsertPos();

#ifdef DEBUG_ARIS_MMP
			std::cerr << "[updateIns] insert_id=" << insert_id_ << " map_tz:";
			for (int _i = 1; _i < insert_id_; ++_i)
				std::cerr << " [" << _i << "]=" << map_nodes_[_i].time_zone_;
			std::cerr << std::endl;
#endif

			// 更新完后清除 nodes_ //
			nodes_.clear();

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


	auto MultimodelPlanner::setDt(double dt) -> void {
		imp_->inv_tg1_.setDt(dt);
		imp_->inv_tg2_.setDt(dt);
		imp_->fwd_tg_.setDt(dt);
		imp_->is_.setDt(dt);
		imp_->sr_.setDt(dt);
		imp_->ag_.setDt(dt);
	}
	auto MultimodelPlanner::dt() -> double {
		return imp_->inv_tg1_.dt();
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
		return imp_->inv_tg1_.maxReplanNum();
	}
	auto MultimodelPlanner::setMaxReplanNum(int max_replan_num) -> void {
		imp_->inv_tg1_.setMaxReplanNum(max_replan_num);
		imp_->inv_tg2_.setMaxReplanNum(max_replan_num);
		imp_->fwd_tg_.setMaxReplanNum(max_replan_num);
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
	auto MultimodelPlanner::insertLinePos(TW& tw, const double* ee_pos, const double* vel, const double* acc, const double* jerk, const double* zone, double time_zone) -> std::int64_t {
		return imp_->insLine(tw, ee_pos, vel, acc, jerk, zone, time_zone);
	}
	auto MultimodelPlanner::insertLinePos(std::string_view tools, std::string_view wobjs, const double* ee_pos, const double* vel, const double* acc, const double* jerk, const double* zone, double time_zone) -> std::int64_t {
		auto tool_str_vec = aris::core::split(tools, ';');
		auto wobj_str_vec = aris::core::split(wobjs, ';');

		TW tw;
		for (Size i = 0; i < std::max(tool_str_vec.size(), wobj_str_vec.size()); ++i) {
			auto tool = i < tool_str_vec.size() ? aris::core::trimLR(tool_str_vec[i]) : std::string("");
			auto wobj = i < wobj_str_vec.size() ? aris::core::trimLR(wobj_str_vec[i]) : std::string("");
			tw.push_back(std::pair(tool, wobj));
		}

		return insertLinePos(tw, ee_pos, vel, acc, jerk, zone, time_zone);
	}

	// 插入新的数据，并重规划 //
	auto MultimodelPlanner::insertCirclePos(TW& tw, const double* ee_pos, const double* mid_pos, const double* vel, const double* acc, const double* jerk, const double* zone, double time_zone) -> std::int64_t {
		return imp_->insCircle(tw, ee_pos, mid_pos, vel, acc, jerk, zone, time_zone);
	}
	auto MultimodelPlanner::insertCirclePos(std::string_view tools, std::string_view wobjs, const double* ee_pos, const double* mid_pos, const double* vel, const double* acc, const double* jerk, const double* zone, double time_zone) -> std::int64_t {
		auto tool_str_vec = aris::core::split(tools, ';');
		auto wobj_str_vec = aris::core::split(wobjs, ';');

		TW tw;
		for (Size i = 0; i < std::max(tool_str_vec.size(), wobj_str_vec.size()); ++i) {
			auto tool = i < tool_str_vec.size() ? aris::core::trimLR(tool_str_vec[i]) : std::string("");
			auto wobj = i < wobj_str_vec.size() ? aris::core::trimLR(wobj_str_vec[i]) : std::string("");
			tw.push_back(std::pair(tool, wobj));
		}

		return insertCirclePos(tw, ee_pos, mid_pos, vel, acc, jerk, zone, time_zone);
	}

	// 插入新的数据，并重规划 //
	auto MultimodelPlanner::insertMoveJ(TW& tw, const double* ee_pos, const double* joint_v, const double* joint_a, const double* joint_j, const double* zone, const std::int64_t *which_root, double time_zone) -> std::int64_t{
		return imp_->insMoveJ(tw, ee_pos, joint_v, joint_a, joint_j, zone, which_root, time_zone);
	}
	auto MultimodelPlanner::insertMoveJ(std::string_view tools, std::string_view wobjs, const double* ee_pos, const double* joint_v, const double* joint_a, const double* joint_j, const double* zone, const std::int64_t *which_root, double time_zone) -> std::int64_t{
		auto tool_str_vec = aris::core::split(tools, ';');
		auto wobj_str_vec = aris::core::split(wobjs, ';');

		TW tw;
		for (Size i = 0; i < std::max(tool_str_vec.size(), wobj_str_vec.size()); ++i) {
			auto tool = i < tool_str_vec.size() ? aris::core::trimLR(tool_str_vec[i]) : std::string("");
			auto wobj = i < wobj_str_vec.size() ? aris::core::trimLR(wobj_str_vec[i]) : std::string("");
			tw.push_back(std::pair(tool, wobj));
		}

		return insertMoveJ(tw, ee_pos, joint_v, joint_a, joint_j, zone, which_root, time_zone);
	}

	// 插入新的数据，并重规划 //
	auto MultimodelPlanner::insertMoveAbsJ(const double* joint_p, const double* joint_v, const double* joint_a, const double* joint_j, const double* zone, double time_zone) -> std::int64_t{
#ifdef DEBUG_ARIS_MMP
		std::cerr << "[MMP::insertMoveAbsJ] time_zone=" << time_zone << std::endl;
#endif
		return imp_->insMoveAbsJ(joint_p, joint_v, joint_a, joint_j, zone, time_zone);
	}

	auto MultimodelPlanner::updateInsertPos() -> void {
		imp_->updateIns();
	}

	// 删除已经不用的数据 //
	auto MultimodelPlanner::clearUsedPos() -> void {
		imp_->inv_tg1_.clearUsedPos();
		imp_->inv_tg2_.clearUsedPos();
		imp_->fwd_tg_.clearUsedPos();
	}

	// 删除全部数据 //
	auto MultimodelPlanner::clearAllPos() -> void {
		imp_->inv_tg1_.clearAllPos();
		imp_->inv_tg2_.clearAllPos();
		imp_->fwd_tg_.clearAllPos();
	}

	// 当前还剩余的指令数 //
	auto MultimodelPlanner::unusedPosNum() -> int {
		return imp_->inv_tg1_.unusedPosNum() + imp_->inv_tg2_.unusedPosNum() + imp_->fwd_tg_.unusedPosNum();
	}

	// 返回当前所有的节点 id //
	auto MultimodelPlanner::unusedNodeIds()const -> std::vector<std::int64_t> {
		auto ids = imp_->inv_tg1_.unusedNodeIds();
		auto ids2 = imp_->inv_tg2_.unusedNodeIds();
		auto fwd_ids = imp_->fwd_tg_.unusedNodeIds();
		ids.insert(ids.end(), ids2.begin(), ids2.end());
		ids.insert(ids.end(), fwd_ids.begin(), fwd_ids.end());
		std::sort(ids.begin(), ids.end());
		return ids;
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

    auto MultimodelPlanner::tgRet() -> std::int64_t {
        return imp_->tg_ret_;
    }

    auto MultimodelPlanner::ikRet() -> std::int64_t{
        return imp_->ik_ret_;
    }

    auto MultimodelPlanner::leftNodeS() -> double {
        auto& node = imp_->map_nodes_[imp_->get_id_ % TW_POOL_SIZE];
        if (node.type_ == MAPNodeType::Line || node.type_ == MAPNodeType::Circle || node.type_ == MAPNodeType::CartesianInitPos) {
            auto& tg = (node.cartesian_planner_idx_ == 0) ? imp_->inv_tg1_ : imp_->inv_tg2_;
            return tg.leftNodeS();
        }
        return imp_->fwd_tg_.leftNodeS();
    }

    MultimodelPlanner::~MultimodelPlanner(){
        stop();
    }
    MultimodelPlanner::MultimodelPlanner():imp_(new Imp) {}
}
