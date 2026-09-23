
#include "aris/plan/function.hpp"
#include "aris/plan/input_smoother.hpp"
#include "aris/plan/async_generator.hpp"
#include "aris/plan/speed_regulator.hpp"
#include "aris/plan/scurve.hpp"
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
				+ (output_psize_ + input_psize_) * sizeof(double) // begEePos, begJointPos
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
		auto begEePos()->double* { return reinterpret_cast<double*>(whichForwardRoots() + input_psize_); }
		auto begJointPos()->double* { return begEePos() + output_psize_; }
	};

	#define TW_POOL_SIZE 10000

	struct MultimodelPlanner::Imp {
		using MarkerVec = std::vector<aris::dynamic::Marker*>;
		
		// config data //
		aris::dynamic::MultiModel* model_{nullptr};
		std::vector<aris::Size> sub_id_list_;
		aris::core::Matrix min_pos_mat_, max_pos_mat_, min_vel_mat_, max_vel_mat_, min_acc_mat_, max_acc_mat_, min_jerk_mat_, max_jerk_mat_;
		
		// public data //
		aris::Size ee_size_{ 0 }, output_psize_{ 0 }, output_vdim_{ 0 }, input_psize_{ 0 }, input_vdim_{ 0 };
		double* max_poss_{ nullptr }, * min_poss_{ nullptr }, * max_vels_{ nullptr }, * min_vels_{ nullptr }, * max_accs_{ nullptr }, * min_accs_{ nullptr }, * max_jerks_{ nullptr }, * min_jerks_{ nullptr };
		aris::dynamic::MotionBase** ees_;
		std::atomic<PlannerState> state_{ PlannerState::Uninitialized };

		// record data //
		double *this_input_{ nullptr }, *last_input_{ nullptr }, *last2_input_{ nullptr };
		double *this_vel_{nullptr}, *last_vel_{nullptr};
		double *this_acc_{nullptr};
		
		// run data //
		double* ee_pos_{ nullptr }, * tw_pos_{ nullptr };
		double* node_input_{ nullptr }, * next_node_input_{ nullptr }; // node_input_ 缓存当前的 next_node_input_ 在time_zone 融合时缓存下一node 的pos

		std::atomic<std::int64_t> get_id_{ 1 }, ins_id_{ 1 };
		std::int64_t ik_ret_{ 0 }, tg_ret_{ 0 };
		MAPNode map_nodes_[TW_POOL_SIZE];
		std::list<MAPNode> nodes_;

		MAPNode last_node_, ins_node_, init_node_;
		ToolWobjSelector tw_, tw_rt_;

		TrajectoryGenerator inv_tg1_, inv_tg2_, fwd_tg_;
		InputSmoother is_;
		SpeedRegulator sr_;

		// pause/resume data //
		std::int64_t paused_tg_ret_{0};
		double resume_target_ratio_{ 1.0 }, speed_epsilon_{ 1e-10 };
		bool pausing_from_resume_{ false }; // true：Pausing 由 Resuming 触发（走 stopOneStep 的减速逻辑）//
		double* pause_pos_{ nullptr }, *resume_from_pos_{ nullptr };
		SCurveParam* resume_scurve_params_{ nullptr };
		double resume_t_{ 0.0 }, resume_T_{ 0.0 };

		// stop data //
		double stop_t_{ 0.0 };

		// goto data //
		TrajectoryGenerator goto_inv_tg_, goto_fwd_tg_;
		InputSmoother goto_is_;
		SpeedRegulator goto_sr_;
		double* goto_input_cache_{ nullptr }; // 反解失败时的关节位置缓存 //
		bool goto_line_{ false };                    // true：笛卡尔（Line/Circle）；false：关节（MoveJ/MoveAbsJ）//
		aris::dynamic::Marker** goto_tools_{ nullptr }, **goto_wobjs_{ nullptr };  // Line 的 tool/wobj（反解 tw→ee 用）//
		std::int64_t* goto_which_roots_{ nullptr };  // Line 的反解根号 //
		double* goto_beg_joint_{ nullptr };          // Line 反解初值（起始关节）//
		// goto 管线工作缓冲（预分配于 mem 块，避免每次 goto 重复分配）//
		double* goto_joint_zone_{ nullptr };         // MoveAbsJ 的 zone（零填）//
		double* goto_tw_zone_{ nullptr };            // Line 的 zone（零填）//
		double* goto_beg_ee_{ nullptr };             // Line 起始末端位置 //
		double* goto_cur_tw_{ nullptr };             // Line 起始 tw（tg init）//
		double* goto_target_ee_{ nullptr };          // Line 目标末端位置（反解输入）//
		double* goto_target_joint_{ nullptr };       // Line 目标关节（反解输出）//

		std::vector<char> mem_;

		////////////// NRT //////////////
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
			core::allocMem(mem_size, node_input_, input_psize_);
			core::allocMem(mem_size, next_node_input_, input_psize_);
			core::allocMem(mem_size, this_input_, input_psize_);
			core::allocMem(mem_size, last_input_, input_psize_);
			core::allocMem(mem_size, last2_input_, input_psize_);
			core::allocMem(mem_size, this_vel_, input_psize_);
			core::allocMem(mem_size, last_vel_, input_psize_);
			core::allocMem(mem_size, this_acc_, input_psize_);
			core::allocMem(mem_size, max_poss_, input_psize_);
			core::allocMem(mem_size, min_poss_, input_psize_);
			core::allocMem(mem_size, max_vels_, input_psize_);
			core::allocMem(mem_size, min_vels_, input_psize_);
			core::allocMem(mem_size, max_accs_, input_psize_);
			core::allocMem(mem_size, min_accs_, input_psize_);
			core::allocMem(mem_size, max_jerks_, input_psize_);
			core::allocMem(mem_size, min_jerks_, input_psize_);
			core::allocMem(mem_size, pause_pos_, input_psize_);
			core::allocMem(mem_size, resume_from_pos_, input_psize_);
			core::allocMem(mem_size, resume_scurve_params_, input_psize_);
			core::allocMem(mem_size, goto_input_cache_, input_psize_);
			core::allocMem(mem_size, goto_beg_joint_, input_psize_);
			core::allocMem(mem_size, goto_joint_zone_, input_vdim_);
			core::allocMem(mem_size, goto_tw_zone_, output_vdim_);
			core::allocMem(mem_size, goto_beg_ee_, output_psize_);
			core::allocMem(mem_size, goto_cur_tw_, output_psize_);
			core::allocMem(mem_size, goto_target_ee_, output_psize_);
			core::allocMem(mem_size, goto_target_joint_, input_psize_);
			core::allocMem(mem_size, goto_which_roots_, ee_size_);
			core::allocMem(mem_size, goto_tools_, ee_size_);
			core::allocMem(mem_size, goto_wobjs_, ee_size_);

			mem_.resize(mem_size, char(0));

			ees_ = core::getMem(mem_.data(), ees_);
			tw_pos_ = core::getMem(mem_.data(), tw_pos_);
			ee_pos_ = core::getMem(mem_.data(), ee_pos_);
			node_input_ = core::getMem(mem_.data(), node_input_);
			next_node_input_ = core::getMem(mem_.data(), next_node_input_);
			this_input_ = core::getMem(mem_.data(), this_input_);
			last_input_ = core::getMem(mem_.data(), last_input_);
			last2_input_ = core::getMem(mem_.data(), last2_input_);
			this_vel_ = core::getMem(mem_.data(), this_vel_);
			last_vel_ = core::getMem(mem_.data(), last_vel_);
			this_acc_ = core::getMem(mem_.data(), this_acc_);
			max_poss_ = core::getMem(mem_.data(), max_poss_);
			min_poss_ = core::getMem(mem_.data(), min_poss_);
			max_vels_ = core::getMem(mem_.data(), max_vels_);
			min_vels_ = core::getMem(mem_.data(), min_vels_);
			max_accs_ = core::getMem(mem_.data(), max_accs_);
			min_accs_ = core::getMem(mem_.data(), min_accs_);
			max_jerks_ = core::getMem(mem_.data(), max_jerks_);
			min_jerks_ = core::getMem(mem_.data(), min_jerks_);
			pause_pos_ = core::getMem(mem_.data(), pause_pos_);
			resume_from_pos_ = core::getMem(mem_.data(), resume_from_pos_);
			resume_scurve_params_ = core::getMem(mem_.data(), resume_scurve_params_);
			goto_input_cache_ = core::getMem(mem_.data(), goto_input_cache_);
			goto_beg_joint_ = core::getMem(mem_.data(), goto_beg_joint_);
			goto_joint_zone_ = core::getMem(mem_.data(), goto_joint_zone_);
			goto_tw_zone_ = core::getMem(mem_.data(), goto_tw_zone_);
			goto_beg_ee_ = core::getMem(mem_.data(), goto_beg_ee_);
			goto_cur_tw_ = core::getMem(mem_.data(), goto_cur_tw_);
			goto_target_ee_ = core::getMem(mem_.data(), goto_target_ee_);
			goto_target_joint_ = core::getMem(mem_.data(), goto_target_joint_);
			goto_which_roots_ = core::getMem(mem_.data(), goto_which_roots_);
			goto_tools_ = core::getMem(mem_.data(), goto_tools_);
			goto_wobjs_ = core::getMem(mem_.data(), goto_wobjs_);

			// 填充不缩放的限幅（原始量纲）//
			std::fill_n(max_poss_, input_psize_, 1e10);
			std::fill_n(min_poss_, input_psize_, -1e10);
			std::fill_n(max_vels_, input_psize_, 1e10);
			std::fill_n(min_vels_, input_psize_, -1e10);
			std::fill_n(max_accs_, input_psize_, 1e10);
			std::fill_n(min_accs_, input_psize_, -1e10);
			std::fill_n(max_jerks_, input_psize_, 1e10);
			std::fill_n(min_jerks_, input_psize_, -1e10);

			if (!max_pos_mat_.empty()) aris::dynamic::s_vc(input_psize_, max_pos_mat_.data(), max_poss_);
			if (!min_pos_mat_.empty()) aris::dynamic::s_vc(input_psize_, min_pos_mat_.data(), min_poss_);
			if (!max_vel_mat_.empty()) aris::dynamic::s_vc(input_psize_, max_vel_mat_.data(), max_vels_);
			if (!min_vel_mat_.empty()) aris::dynamic::s_vc(input_psize_, min_vel_mat_.data(), min_vels_);
			if (!max_acc_mat_.empty()) aris::dynamic::s_vc(input_psize_, max_acc_mat_.data(), max_accs_);
			if (!min_acc_mat_.empty()) aris::dynamic::s_vc(input_psize_, min_acc_mat_.data(), min_accs_);
			if (!max_jerk_mat_.empty()) aris::dynamic::s_vc(input_psize_, max_jerk_mat_.data(), max_jerks_);
			if (!min_jerk_mat_.empty()) aris::dynamic::s_vc(input_psize_, min_jerk_mat_.data(), min_jerks_);
			
			// nodes //
			for (int i = 0; i < TW_POOL_SIZE; ++i) {
				map_nodes_[i] = MAPNode(0, MAPNodeType::CartesianInitPos, ee_size_, output_psize_, output_vdim_, input_psize_, input_vdim_);
			}
			last_node_ = MAPNode(0, MAPNodeType::CartesianInitPos, ee_size_, output_psize_, output_vdim_, input_psize_, input_vdim_);
			ins_node_ = MAPNode(0, MAPNodeType::CartesianInitPos, ee_size_, output_psize_, output_vdim_, input_psize_, input_vdim_);
			init_node_ = MAPNode(0, MAPNodeType::CartesianInitPos, ee_size_, output_psize_, output_vdim_, input_psize_, input_vdim_);

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
			sr_.setInputSize(input_psize_);

			is_.allocateMemory();
			sr_.allocateMemory();

			// 设置回调 //
			is_.setInputGenerator([this](double* p)->std::int64_t {
				auto get_id = get_id_.load();
				auto ins_id = ins_id_.load();

				// no data //
				if(get_id >= ins_id) {
					std::copy(node_input_, node_input_ + input_psize_, p);
					return 0;
				}

				auto& node = map_nodes_[get_id % TW_POOL_SIZE];

				auto get_tg = [this](MAPNode& node)->TrajectoryGenerator* {
					if (node.type_ == MAPNodeType::Line || node.type_ == MAPNodeType::Circle || node.type_ == MAPNodeType::CartesianInitPos) {
						return (node.cartesian_planner_idx_ == 0) ? &inv_tg1_ : &inv_tg2_;
					}
					else {
						return &fwd_tg_;
					}
				};

				// 反解成功：p 和 default_p 为反解值；反解失败：p 被 default_p 替代，default_p 不变 //
				auto getNodePos = [this](MAPNode &node, double* p, double* default_p)->void {
					// Cartesian space move //
					if(node.type_ == MAPNodeType::Line || node.type_ == MAPNodeType::Circle) {
						auto& cart_tg = (node.cartesian_planner_idx_ == 0) ? inv_tg1_ : inv_tg2_;
						tg_ret_ = cart_tg.getEePosAndMoveDt(tw_pos_);

						tw_rt_.selectTw(node.tools(), node.wobjs());
						tw_rt_.setTwPos(tw_pos_);
						tw_rt_.getEePos(ee_pos_);

						// 无状态反解：用 begJointPos 作为初值 //
						ik_ret_ = model_->subInverseKinematics(
							sub_id_list_.size(), sub_id_list_.data(),
							ee_pos_, p, node.whichInverseRoots(), node.begJointPos());

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

				getNodePos(node, p, node_input_);

				// 判断是否切换节点 //
				auto tg = get_tg(node);
				auto current_tg_id = tg->currentNodeId();
				auto is_switch_node = tg->isCurrentNodeFinished();

				// time_zone 混合：下一节点使用不同规划器且进入 real_zone 时，同时运行两个规划器并 smoothstep 混合 //
#ifdef DEBUG_ARIS_MMP
				static int tz_check = 0;
				if (tz_check < 3 || get_id \!= 1) {
					std::cerr << "[time_zone_check] tz=" << node.time_zone_ << " get_id=" << get_id
						<< " insert_id=" << ins_id << " cond=" << (node.time_zone_ > 0.0 && get_id + 1 < ins_id) << std::endl;
					if (get_id \!= 1) tz_check++;
				}
#endif
				if (node.time_zone_ > 0.0 && get_id + 1 < ins_id) {
					auto& next_node = map_nodes_[(get_id + 1) % TW_POOL_SIZE];
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
						// 设置 next_node_input_ 的初值，以防反解失败 //
						if(left_s + curr_tg->dt() >= real_tz)
							std::copy(node.jointPos(), node.jointPos() + input_psize_, next_node_input_);
						
						double zone_t = 1.0 - left_s / real_tz;
						getNodePos(next_node, next_node_input_, next_node_input_);
						for (aris::Size i = 0; i < input_psize_; ++i)
							p[i] = p[i] + next_node_input_[i] - node.jointPos()[i];
					}

					if(is_switch_node) {
						std::copy(next_node_input_, next_node_input_ + input_psize_, node_input_);
					}
				}

				// switch node and return //
				get_id_ = is_switch_node ? current_tg_id + 1 : current_tg_id;
				return get_id;
			});


			sr_.setInputGenerator([this](double* p)->std::int64_t {
				return is_.getNextInput(p);
			});
		}
		auto init() -> void {
			// ees 相关 //
			model_->getSubOutputMotions(sub_id_list_.size(), sub_id_list_.data(), ees_);
			std::fill(ee_pos_, ee_pos_ + output_psize_, 0.0);
			std::fill(tw_pos_, tw_pos_ + output_psize_, 0.0);

			// nodes 相关 //
			get_id_.store(1);
			ins_id_.store(1);
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
			std::copy(last_node_.jointPos(), last_node_.jointPos() + input_psize_, node_input_);
			std::copy(last_node_.jointPos(), last_node_.jointPos() + input_psize_, next_node_input_);
			std::copy(last_node_.jointPos(), last_node_.jointPos() + input_psize_, this_input_);
			std::copy(last_node_.jointPos(), last_node_.jointPos() + input_psize_, last_input_);
			std::copy(last_node_.jointPos(), last_node_.jointPos() + input_psize_, last2_input_);
			std::fill_n(this_vel_, input_psize_, 0.0);
			std::fill_n(last_vel_, input_psize_, 0.0);
			std::fill_n(this_acc_, input_psize_, 0.0);

			std::copy(last_node_.eePos(), last_node_.eePos() + output_psize_, ee_pos_);

			// 初始化 tg 等
			inv_tg1_.clearAllPos();
			inv_tg2_.clearAllPos();
			fwd_tg_.clearAllPos();

			// 插入起始节点，后续 update 时会初始化 tg //
			auto tools = std::vector<aris::dynamic::Marker*>(ee_size_, nullptr);
			auto wobjs = std::vector<aris::dynamic::Marker*>(ee_size_, nullptr);
			insertInitNode(tools.data(), wobjs.data(), MAPNodeType::JointInitPos);

			is_.init(last_node_.jointPos());
			
			sr_.init(1.0);
		}

		// 工具和工件解析函数：根据 tool_wobjs 填充 tools 和 wobjs（裸指针数组，长度 ee_size_）//
		auto resolveToolsAndWobjs(TW& tool_wobjs, aris::dynamic::Marker** tools, aris::dynamic::Marker** wobjs) -> void {
			for (int i = 0; i < static_cast<int>(std::min(tool_wobjs.size(), static_cast<std::size_t>(ee_size_))); ++i) {
				tools[i] = model_->findTool(tool_wobjs[i].first);
				wobjs[i] = model_->findWobj(tool_wobjs[i].second);

				if (tools[i] == nullptr) {
					if (tool_wobjs[i].first == "") {
						tools[i] = ees_[i]->makI();
					}
					else {
						THROW_FILE_LINE("tool \"" + tool_wobjs[i].first + "\" not found");
					}
				}
				if (wobjs[i] == nullptr) {
					if (tool_wobjs[i].second == "") {
						wobjs[i] = ees_[i]->makJ();
					}
					else {
						THROW_FILE_LINE("wobj \"" + tool_wobjs[i].second + "\" not found");
					}
				}
			}
		}
		
		// run-resume-stop 系列 config //
		
		// request 状态机请求函数 //
		// 并发设计：request 系列由 NRT 线程调用，同一时刻最多一个 request 在执行；
		// onestep 系列由 RT 线程串行调用。最多只有一个 request 线程 + 一个 onestep
		// 线程并发。各 request 函数直接在自己的函数体内用 state_ 的 CAS 完成状态切换；
		// requestResume 先备好 resume scurve 再 CAS 到 Resuming，借助 happens-before 保证数据可见。
		auto requestInit() -> std::int64_t {
			if (state_.load() != PlannerState::Uninitialized)
				return -1;

			init();

			// 必定切换成功：实时循环里不会在 Uninitialized 状态下改变 state //
			state_.store(PlannerState::Idle);
			return 0;
		}
		auto requestStop() -> std::int64_t {
			// 实时循环可能切换状态，用循环保证 CAS 最终成功 //
			for (;;) {
				auto cur = state_.load();

				// 因为实时循环可能从IDLE -> RUNNING，所以IDLE状态需特殊处理 //
				if (cur == PlannerState::Idle) {
					// 静止状态：没有运动需要平滑减速，直接切入 Uninitialized //
					if (state_.compare_exchange_strong(cur, PlannerState::Uninitialized))
						break;
				}
				else if(cur == PlannerState::Paused) {
					// 暂停状态：实时循环不会Paused状态，因此直接切入 Uninitialized //
					state_.store(PlannerState::Uninitialized);
					break;
				}
				else if (cur != PlannerState::Uninitialized) {
					// 运动状态：切入 Stopping，由 stopOneStep 完成平滑减速 //
					if (state_.compare_exchange_strong(cur, PlannerState::Stopping))
						break;
				}
				else {
					break; // Uninitialized：无需停止
				}
			}
			return 0;
		}
		auto requestPause() -> std::int64_t {
			// Running / Resuming 时可暂停。实时线程可能恰好把 Resuming 切换为 Running
			//（resumeOneStep 返回 0 的那一拍），因此这里用循环 CAS（同 requestStop）：
			// - 若 CAS 时仍是 Resuming → 恢复中暂停（pausing_from_resume_ = true）；
			// - 若实时线程已抢先切到 Running → 循环重读，按 Running 正常暂停（flag = false）。
			for (;;) {
				auto cur = state_.load();
				if (cur == PlannerState::Pausing || cur == PlannerState::Paused) {
					return 0; // 已在暂停中或已暂停，视为成功 //
				}
				else if (cur == PlannerState::Running || cur == PlannerState::Resuming) {
					// 在 CAS 之前写 flag，借助 state_ 的原子 CAS 建立 happens-before，
					// 保证实时线程观察到 Pausing 时 flag 已就绪 //
					pausing_from_resume_ = (cur == PlannerState::Resuming);
					if (state_.compare_exchange_strong(cur, PlannerState::Pausing))
						return 0;
					// CAS 失败：状态已被实时线程/其它 request 改变，重读后重试 //
				}
				else {
					return -1; // Uninitialized / Idle / Stopping / Goto / Error 不可暂停 //
				}
			}
		}
		auto requestResume() -> std::int64_t {
			auto cur = state_.load();
			if (cur == PlannerState::Resuming || cur == PlannerState::Running) {
				// 已在恢复中或已运行，视为成功 //
				return 0;
			}

			// 如果不是 Paused 状态，无法恢复 //
			if (cur != PlannerState::Paused)
				return -1;

			const auto n = input_psize_;

			// 当前位置读取 model 的 input //
			model_->getSubInputPos(sub_id_list_.size(), sub_id_list_.data(), resume_from_pos_);

			auto vel_ratio = std::min(resume_target_ratio_ * 0.1, 0.01);
			for (aris::Size i = 0; i < n; ++i) {
				resume_scurve_params_[i].pa_ = resume_from_pos_[i];
				resume_scurve_params_[i].pb_ = pause_pos_[i];
				resume_scurve_params_[i].vc_max_ = max_vels_[i] * vel_ratio;
				resume_scurve_params_[i].a_ = max_accs_[i] * vel_ratio;
				resume_scurve_params_[i].j_ = max_jerks_[i] * vel_ratio;
			}

			aris::plan::s_scurve_make(n, resume_scurve_params_, inv_tg1_.dt());
			resume_t_ = 0.0;
			resume_T_ = n > 0 ? resume_scurve_params_[0].T_ : 0.0;

			// 实时循环里无法在 paused 状态下改变状态，因此当前肯定还是 paused 状态 //
			state_.store(PlannerState::Resuming);
			return 0;
		}

		// run 系列指令插入 //
		auto insertInitNode(aris::dynamic::Marker** tools, aris::dynamic::Marker** wobjs, MAPNodeType init_type, int cartesian_planner_idx = 0) -> void {
			// 在独立的 init_node_ 中构建：tools/wobjs 可能指向 ins_node_ 自身，
			// 用 init_node_ 作为构建缓冲，避免 ins_node_.init() 清空源数据 //
			init_node_.init();
			std::copy(last_node_.mem_.begin(), last_node_.mem_.end(), init_node_.mem_.begin());

			// 填入本节点信息 //
			init_node_.id_ = ins_id_.load();
			init_node_.type_ = init_type;
			init_node_.cartesian_planner_idx_ = cartesian_planner_idx;
			std::copy(tools, tools + ee_size_, init_node_.tools());
			std::copy(wobjs, wobjs + ee_size_, init_node_.wobjs());

			// 填入 tw 信息 //
			if (tw_.selectTw(last_node_.tools(), last_node_.wobjs()))
				THROW_FILE_LINE("invalid last tool and wobj");
			tw_.setTwPos(last_node_.twPos());
			if (tw_.selectTw(init_node_.tools(), init_node_.wobjs()))
				THROW_FILE_LINE("invalid tool and wobj");
			tw_.getTwPos(init_node_.twPos());

			// 填入 whichroot 信息 //
			if(last_node_.type_ == MAPNodeType::Line || last_node_.type_ == MAPNodeType::Circle || last_node_.type_ == MAPNodeType::CartesianInitPos) {
				std::copy(last_node_.whichInverseRoots(), last_node_.whichInverseRoots() + sub_id_list_.size(), init_node_.whichInverseRoots());
				model_->getSubWhichForwardRoot(sub_id_list_.size(), sub_id_list_.data(), init_node_.jointPos(), init_node_.eePos(), init_node_.whichForwardRoots());
			}
			else {
				std::copy(last_node_.whichForwardRoots(), last_node_.whichForwardRoots() + sub_id_list_.size(), init_node_.whichForwardRoots());
				model_->getSubWhichInverseRoot(sub_id_list_.size(), sub_id_list_.data(), init_node_.eePos(), init_node_.jointPos(), init_node_.whichInverseRoots());
			}

			// 插入 //
			nodes_.push_back(init_node_);
			std::swap(last_node_, init_node_);
		}
		auto insLine(TW& tool_wobjs, const double* tw_pos, const double* vel, const double* acc, const double* jerk, const double* zone, double time_zone = 0.0) -> std::int64_t {
			// 根据当前状态判断可行性，state在 rt 线程只可能在以下几个状态间流转，因此并发不影响判断结果 //
			auto state = state_.load();
			if(state != PlannerState::Idle && state != PlannerState::Running && state != PlannerState::Paused && state != PlannerState::Pausing && state != PlannerState::Resuming && requestInit() != 0)
				return -1;

			auto ins_id = ins_id_.load();

			// init & 获取坐标系 //
			ins_node_.init();

			// 获取坐标系（直接写入 ins_node_ 的 tools/wobjs）//
			resolveToolsAndWobjs(tool_wobjs, ins_node_.tools(), ins_node_.wobjs());

			// 计算 cartesian_planner_idx //
			int cartesian_idx = 0;
			if (last_node_.type_ == MAPNodeType::Line || last_node_.type_ == MAPNodeType::Circle || last_node_.type_ == MAPNodeType::CartesianInitPos) {
				// 从笛卡尔空间来，判断 tool/wobj 是否变化 //
				cartesian_idx = !last_node_.isSameTools(ins_node_.tools()) || !last_node_.isSameWobjs(ins_node_.wobjs()) 
					? 1 - last_node_.cartesian_planner_idx_ : last_node_.cartesian_planner_idx_;
			} else {
				// 从关节空间来，默认使用 inv_tg1_ (0)
				cartesian_idx = 0;
			}

			// 查看是否插入 INIT //
			if ((last_node_.type_ != MAPNodeType::Line && last_node_.type_ != MAPNodeType::Circle && last_node_.type_ != MAPNodeType::CartesianInitPos)
				|| !last_node_.isSameTools(ins_node_.tools())
				|| !last_node_.isSameWobjs(ins_node_.wobjs())) 
			{
				insertInitNode(ins_node_.tools(), ins_node_.wobjs(), MAPNodeType::CartesianInitPos, cartesian_idx);
			}

			// 更新 last_tw_pos_ 等 //
			ins_node_.id_ = ins_id;
			ins_node_.type_ = MAPNodeType::Line;
			ins_node_.cartesian_planner_idx_ = cartesian_idx;
			ins_node_.time_zone_ = time_zone;
			std::copy(tw_pos, tw_pos + output_psize_, ins_node_.twPos());
			std::copy(vel, vel + output_vdim_, ins_node_.twVel());
			std::copy(acc, acc + output_vdim_, ins_node_.twAcc());
			std::copy(jerk, jerk + output_vdim_, ins_node_.twJerk());
			std::copy(zone, zone + output_vdim_, ins_node_.twZone());

			// 记录 init 位置（last_node_ 结束时的位置作为本节点初值）//
			std::copy(last_node_.eePos(), last_node_.eePos() + output_psize_, ins_node_.begEePos());
			std::copy(last_node_.jointPos(), last_node_.jointPos() + input_psize_, ins_node_.begJointPos());

			// 做反解计算 //
			{
				tw_.selectTw(ins_node_.tools(), ins_node_.wobjs());
				tw_.setTwPos(ins_node_.twPos());
				tw_.getEePos(ins_node_.eePos());

				std::copy(last_node_.whichInverseRoots(), last_node_.whichInverseRoots() + sub_id_list_.size(), ins_node_.whichInverseRoots());

				// ik 求解末端，初值采用 begJointPos
				auto ret = model_->subInverseKinematics(
					sub_id_list_.size(), sub_id_list_.data(),
					ins_node_.eePos(), ins_node_.jointPos(), ins_node_.whichInverseRoots(), ins_node_.begJointPos());

				if(ret < 0)
					return ret;

				// 检查反解得到的关节位置是否超出运动范围限制，超限则返回 -2 //
				for (int i = 0; i < input_psize_; ++i) {
					if (ins_node_.jointPos()[i] > max_poss_[i] + 1e-10
						|| ins_node_.jointPos()[i] < min_poss_[i] - 1e-10)
						return -2;
				}

				// 根据求出的末端，计算它在用的哪组解
				model_->getSubWhichForwardRoot(sub_id_list_.size(), sub_id_list_.data(), ins_node_.jointPos(), ins_node_.eePos(), ins_node_.whichForwardRoots());

			}

			// 正常插入指令 //
			nodes_.push_back(ins_node_);
			std::swap(last_node_, ins_node_);

			ins_id_++;
			return ins_id;
		}
		auto insCircle(TW& tool_wobjs, const double* tw_pos, const double* tw_mid_pos, const double* vel, const double* acc, const double* jerk, const double* zone, double time_zone = 0.0) -> std::int64_t {
			// 根据当前状态判断可行性，state在 rt 线程只可能在以下几个状态间流转，因此并发不影响判断结果 //
			auto state = state_.load();
			if(state != PlannerState::Idle && state != PlannerState::Running && state != PlannerState::Paused && state != PlannerState::Pausing && state != PlannerState::Resuming && requestInit() != 0)
				return -1;

			auto ins_id = ins_id_.load();

			// init & 获取坐标系 //
			ins_node_.init();

			// 获取坐标系（直接写入 ins_node_ 的 tools/wobjs）//
			resolveToolsAndWobjs(tool_wobjs, ins_node_.tools(), ins_node_.wobjs());

			// 计算 cartesian_planner_idx //
			int cartesian_idx = 0;
			if (last_node_.type_ == MAPNodeType::Line || last_node_.type_ == MAPNodeType::Circle || last_node_.type_ == MAPNodeType::CartesianInitPos) {
				// 从笛卡尔空间来，判断 tool/wobj 是否变化 //
				cartesian_idx = !last_node_.isSameTools(ins_node_.tools()) || !last_node_.isSameWobjs(ins_node_.wobjs()) 
					? 1 - last_node_.cartesian_planner_idx_ : last_node_.cartesian_planner_idx_;
			} else {
				// 从关节空间来，默认使用 inv_tg1_ (0)
				cartesian_idx = 0;
			}

			// 查看是否插入 INIT //
			if ((last_node_.type_ != MAPNodeType::Line && last_node_.type_ != MAPNodeType::Circle && last_node_.type_ != MAPNodeType::CartesianInitPos)
				|| !last_node_.isSameTools(ins_node_.tools())
				|| !last_node_.isSameWobjs(ins_node_.wobjs())) 
			{
				insertInitNode(ins_node_.tools(), ins_node_.wobjs(), MAPNodeType::CartesianInitPos, cartesian_idx);
			}

			// 更新 last_tw_pos_ 等 //
			ins_node_.id_ = ins_id;
			ins_node_.type_ = MAPNodeType::Circle;
			ins_node_.cartesian_planner_idx_ = cartesian_idx;
			ins_node_.time_zone_ = time_zone;
			std::copy(tw_pos, tw_pos + output_psize_, ins_node_.twPos());
			std::copy(tw_mid_pos, tw_mid_pos + output_psize_, ins_node_.midPos());
			std::copy(vel, vel + output_vdim_, ins_node_.twVel());
			std::copy(acc, acc + output_vdim_, ins_node_.twAcc());
			std::copy(jerk, jerk + output_vdim_, ins_node_.twJerk());
			std::copy(zone, zone + output_vdim_, ins_node_.twZone());

			// 记录 init 位置（last_node_ 结束时的位置作为本节点初值）//
			std::copy(last_node_.eePos(), last_node_.eePos() + output_psize_, ins_node_.begEePos());
			std::copy(last_node_.jointPos(), last_node_.jointPos() + input_psize_, ins_node_.begJointPos());

			{
				tw_.selectTw(ins_node_.tools(), ins_node_.wobjs());
				tw_.setTwPos(ins_node_.twPos());
				tw_.getEePos(ins_node_.eePos());

				std::copy(last_node_.whichInverseRoots(), last_node_.whichInverseRoots() + sub_id_list_.size(), ins_node_.whichInverseRoots());

				// ik 求解末端，初值采用 begJointPos
				auto ret = model_->subInverseKinematics(
					sub_id_list_.size(), sub_id_list_.data(),
					ins_node_.eePos(), ins_node_.jointPos(), ins_node_.whichInverseRoots(), ins_node_.begJointPos());

				if(ret < 0)
					return ret;

				// 检查反解得到的关节位置是否超出运动范围限制，超限则返回 -2 //
				for (int i = 0; i < input_psize_; ++i) {
					if (ins_node_.jointPos()[i] > max_poss_[i] + 1e-10
						|| ins_node_.jointPos()[i] < min_poss_[i] - 1e-10)
						return -2;
				}

				// 根据求出的末端，计算它在用的哪组解
				model_->getSubWhichForwardRoot(sub_id_list_.size(), sub_id_list_.data(), ins_node_.jointPos(), ins_node_.eePos(), ins_node_.whichForwardRoots());

			}

			// 正常插入指令 //
			nodes_.push_back(ins_node_);
			std::swap(last_node_, ins_node_);

			ins_id_++;
			return ins_id;
		}
		auto insMoveJ(TW& tool_wobjs, const double* tw_pos, const double* joint_v, const double* joint_a, const double* joint_j, const double* zone, const std::int64_t *which_root, double time_zone = 0.0) -> std::int64_t {
			// 根据当前状态判断可行性，state在 rt 线程只可能在以下几个状态间流转，因此并发不影响判断结果 //
			auto state = state_.load();
			if(state != PlannerState::Idle && state != PlannerState::Running && state != PlannerState::Paused && state != PlannerState::Pausing && state != PlannerState::Resuming && requestInit() != 0)
				return -1;

			auto ins_id = ins_id_.load();

			// init & 获取坐标系 //
			ins_node_.init();

			// 获取坐标系（直接写入 ins_node_ 的 tools/wobjs）//
			resolveToolsAndWobjs(tool_wobjs, ins_node_.tools(), ins_node_.wobjs());

			// 查看是否插入 INIT //
			if ((last_node_.type_ != MAPNodeType::MoveJ && last_node_.type_ != MAPNodeType::MoveAbsJ && last_node_.type_ != MAPNodeType::JointInitPos)) {
				insertInitNode(ins_node_.tools(), ins_node_.wobjs(), MAPNodeType::JointInitPos);
			}

			// 更新 last_tw_pos_ 等 //
			ins_node_.id_ = ins_id;
			ins_node_.type_ = MAPNodeType::MoveJ;
			ins_node_.time_zone_ = time_zone;
			std::copy(tw_pos, tw_pos + output_psize_, ins_node_.twPos());
			std::copy(joint_v, joint_v + input_vdim_, ins_node_.jointVel());
			std::copy(joint_a, joint_a + input_vdim_, ins_node_.jointAcc());
			std::copy(joint_j, joint_j + input_vdim_, ins_node_.jointJerk());
			std::copy(zone, zone + input_vdim_, ins_node_.jointZone());

			// 记录 init 位置（last_node_ 结束时的位置作为本节点初值）//
			std::copy(last_node_.eePos(), last_node_.eePos() + output_psize_, ins_node_.begEePos());
			std::copy(last_node_.jointPos(), last_node_.jointPos() + input_psize_, ins_node_.begJointPos());

			{
				tw_.selectTw(ins_node_.tools(), ins_node_.wobjs());
				tw_.setTwPos(ins_node_.twPos());
				tw_.getEePos(ins_node_.eePos());

				// 用上一个节点的 whichInverseRoots 作为本节点的初值，若 which_root 不为空，则使用 which_root 作为本节点的初值
				if(which_root != nullptr){
					std::copy(which_root, which_root + sub_id_list_.size(), ins_node_.whichInverseRoots());
				}
				else{
					std::fill(ins_node_.whichInverseRoots(), ins_node_.whichInverseRoots() + sub_id_list_.size(), -1);
				}

				// ik 求解末端，初值采用 begJointPos
				auto ret = model_->subInverseKinematics(
					sub_id_list_.size(), sub_id_list_.data(),
					ins_node_.eePos(), ins_node_.jointPos(), ins_node_.whichInverseRoots(), ins_node_.begJointPos());
				
				if(ret < 0)
					return ret;

				// 检查反解得到的关节位置是否超出运动范围限制，超限则返回 -2 //
				for (int i = 0; i < input_psize_; ++i) {
					if (ins_node_.jointPos()[i] > max_poss_[i] + 1e-10
						|| ins_node_.jointPos()[i] < min_poss_[i] - 1e-10)
						return -2;
				}

				// 根据求出的末端，计算它在用的哪组解
				model_->getSubWhichForwardRoot(sub_id_list_.size(), sub_id_list_.data(), ins_node_.jointPos(), ins_node_.eePos(), ins_node_.whichForwardRoots());
			}

			// 正常插入指令 //
			nodes_.push_back(ins_node_);
			std::swap(last_node_, ins_node_);

			ins_id_++;
			return ins_id;
		}
		auto insMoveAbsJ(const double* joint_p, const double* joint_v, const double* joint_a, const double* joint_j, const double* zone, double time_zone = 0.0) -> std::int64_t {
			// 根据当前状态判断可行性，state在 rt 线程只可能在以下几个状态间流转，因此并发不影响判断结果 //
			auto state = state_.load();
			if(state != PlannerState::Idle && state != PlannerState::Running && state != PlannerState::Paused && state != PlannerState::Pausing && state != PlannerState::Resuming && requestInit() != 0)
				return -1;

			auto ins_id = ins_id_.load();
			
			// 如果运动方式有变化，重新插入 INIT //
			if ((last_node_.type_ != MAPNodeType::MoveJ && last_node_.type_ != MAPNodeType::MoveAbsJ && last_node_.type_ != MAPNodeType::JointInitPos)) 
			{
				auto tools = std::vector<aris::dynamic::Marker*>(ee_size_, nullptr);
				auto wobjs = std::vector<aris::dynamic::Marker*>(ee_size_, nullptr);
				insertInitNode(tools.data(), wobjs.data(), MAPNodeType::JointInitPos);
			}

			// 更新 last_tw_pos_ 等 //
			ins_node_.init();
			ins_node_.id_ = ins_id;
			ins_node_.type_ = MAPNodeType::MoveAbsJ;
			ins_node_.time_zone_ = time_zone;
			std::copy(joint_p, joint_p + input_psize_, ins_node_.jointPos());
			std::copy(joint_v, joint_v + input_vdim_, ins_node_.jointVel());
			std::copy(joint_a, joint_a + input_vdim_, ins_node_.jointAcc());
			std::copy(joint_j, joint_j + input_vdim_, ins_node_.jointJerk());
			std::copy(zone, zone + input_vdim_, ins_node_.jointZone());

			// 记录 init 位置（last_node_ 结束时的位置作为本节点初值）//
			std::copy(last_node_.eePos(), last_node_.eePos() + output_psize_, ins_node_.begEePos());
			std::copy(last_node_.jointPos(), last_node_.jointPos() + input_psize_, ins_node_.begJointPos());

			{
				std::copy(last_node_.whichForwardRoots(), last_node_.whichForwardRoots() + sub_id_list_.size(), ins_node_.whichForwardRoots());
				model_->getSubWhichInverseRoot(sub_id_list_.size(), sub_id_list_.data(), ins_node_.eePos(), ins_node_.jointPos(), ins_node_.whichInverseRoots());
				
				auto ret = model_->subForwardKinematics(
					sub_id_list_.size(), sub_id_list_.data(),
					ins_node_.jointPos(), ins_node_.eePos(), ins_node_.whichForwardRoots(), ins_node_.jointPos());

				if(ret < 0)
					return ret;

				// 检查目标关节位置是否超出运动范围限制，超限则返回 -2 //
				for (int i = 0; i < input_psize_; ++i) {
					if (ins_node_.jointPos()[i] > max_poss_[i] + 1e-10
						|| ins_node_.jointPos()[i] < min_poss_[i] - 1e-10)
						return -2;
				}
			}

			// 正常插入指令 //
#ifdef DEBUG_ARIS_MMP
			std::cerr << "[insMoveAbsJ] BEFORE push: ins_node_.tz=" << ins_node_.time_zone_ << " time_zone_param=" << time_zone << " insert_id=" << ins_id_ << std::endl;
#endif
			nodes_.push_back(ins_node_);
#ifdef DEBUG_ARIS_MMP
			std::cerr << "[insMoveAbsJ] AFTER push: nodes_.back().tz=" << nodes_.back().time_zone_ << " size=" << nodes_.size() << std::endl;
#endif
			std::swap(last_node_, ins_node_);

			ins_id_++;
			return ins_id;
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
			std::cerr << "[updateIns] insert_id=" << ins_id_ << " map_tz:";
			for (int _i = 1; _i < ins_id_; ++_i)
				std::cerr << " [" << _i << "]=" << map_nodes_[_i].time_zone_;
			std::cerr << std::endl;
#endif

			// 更新完后清除 nodes_ //
			nodes_.clear();
		}

		// goto 管线 config //
		auto configGotoPipeline() -> void {
			const auto dt = inv_tg1_.dt();

			// 工作缓冲：zone 每次零填；其余缓冲在 goto 前都会被覆盖填充 //
			std::fill_n(goto_joint_zone_, input_vdim_, 0.0);
			std::fill_n(goto_tw_zone_, output_vdim_, 0.0);
			std::fill_n(goto_tools_, ee_size_, nullptr);
			std::fill_n(goto_wobjs_, ee_size_, nullptr);

			// 专用 tg：与主管线相同的位置类型，allocate 后 clear 再插 init + goto 节点 //
			std::vector<aris::dynamic::PosType> ee_pos_types(ee_size_);
			model_->getSubOutputPosTypes(sub_id_list_.size(), sub_id_list_.data(), ee_pos_types.data());
			std::vector<aris::dynamic::PosType> input_pos_types(model_->subInputSize(sub_id_list_.size(), sub_id_list_.data()));
			model_->getSubInputPosTypes(sub_id_list_.size(), sub_id_list_.data(), input_pos_types.data());

			goto_inv_tg_.setPosTypes(ee_pos_types);
			goto_inv_tg_.setDt(dt);
			goto_inv_tg_.allocateMemory();
			goto_fwd_tg_.setPosTypes(input_pos_types);
			goto_fwd_tg_.setDt(dt);
			goto_fwd_tg_.allocateMemory();

			// 专用 is / sr：从主管线 is_ 拷贝限幅，allocate 后设置回调 //
			goto_is_.setInputSize(input_psize_);
			goto_is_.setDt(dt);
			goto_is_.setMaxPos(is_.maxPos());
			goto_is_.setMinPos(is_.minPos());
			goto_is_.setMaxVel(is_.maxVel());
			goto_is_.setMinVel(is_.minVel());
			goto_is_.setMaxAcc(is_.maxAcc());
			goto_is_.setMinAcc(is_.minAcc());
			goto_is_.allocateMemory();

			goto_sr_.setInputSize(input_psize_);
			goto_sr_.setDt(dt);
			goto_sr_.setMaxPos(is_.maxPos());
			goto_sr_.setMinPos(is_.minPos());
			goto_sr_.setMaxVel(is_.maxVel());
			goto_sr_.setMinVel(is_.minVel());
			goto_sr_.setMaxAcc(is_.maxAcc());
			goto_sr_.setMinAcc(is_.minAcc());
			goto_sr_.allocateMemory();

			// 回调：单节点，推进 goto_tg_ 并（笛卡尔时）反解输出关节位置 //
			goto_is_.setInputGenerator([this](double* p)->std::int64_t {
				if (goto_line_) {
					tg_ret_ = goto_inv_tg_.getEePosAndMoveDt(tw_pos_);
					tw_rt_.selectTw(goto_tools_, goto_wobjs_);
					tw_rt_.setTwPos(tw_pos_);
					tw_rt_.getEePos(ee_pos_);
					ik_ret_ = model_->subInverseKinematics(
						sub_id_list_.size(), sub_id_list_.data(),
						ee_pos_, p, goto_which_roots_, goto_beg_joint_);
					if (ik_ret_ >= 0)
						std::copy(p, p + input_psize_, goto_input_cache_);
					else
						std::copy(goto_input_cache_, goto_input_cache_ + input_psize_, p);
					return tg_ret_;
				}
				else {
					tg_ret_ = goto_fwd_tg_.getEePosAndMoveDt(p);
					std::copy(p, p + input_psize_, goto_input_cache_);
					return tg_ret_;
				}
			});
			goto_sr_.setInputGenerator([this](double* p)->std::int64_t {
				return goto_is_.getNextInput(p);
			});
		}
		auto gotoJ(TW& tool_wobjs, const double* tw_pos, const double* joint_v, const double* joint_a, const double* joint_j, const std::int64_t *which_root) -> std::int64_t {
			auto cur = state_.load();
			PlannerState goto_state;
			if (cur == PlannerState::Uninitialized) goto_state = PlannerState::UninitializedGoto;
			else if (cur == PlannerState::Paused) goto_state = PlannerState::PausedGoto;
			else return -1;

			configGotoPipeline();
			goto_line_ = false;

			// 解析工具/工件 //
			resolveToolsAndWobjs(tool_wobjs, goto_tools_, goto_wobjs_);

			// 起点：当前关节/末端位置 //
			model_->getSubInputPos(sub_id_list_.size(), sub_id_list_.data(), goto_beg_joint_);
			model_->getSubOutputPos(sub_id_list_.size(), sub_id_list_.data(), goto_beg_ee_);

			// 反解目标（tw → ee → joint；根号用 which_root 或当前逆解根）//
			tw_.selectTw(goto_tools_, goto_wobjs_);
			tw_.setTwPos(tw_pos);
			tw_.getEePos(goto_target_ee_);
			if (which_root != nullptr) {
				std::copy(which_root, which_root + sub_id_list_.size(), goto_which_roots_);
			}
			else {
				model_->getSubWhichInverseRoot(sub_id_list_.size(), sub_id_list_.data(), goto_beg_ee_, goto_beg_joint_, goto_which_roots_);
			}
			auto ret = model_->subInverseKinematics(sub_id_list_.size(), sub_id_list_.data(),
				goto_target_ee_, goto_target_joint_, goto_which_roots_, goto_beg_joint_);
			if (ret < 0)
				return ret;
			for (int i = 0; i < input_psize_; ++i) {
				if (goto_target_joint_[i] > max_poss_[i] + 1e-10 || goto_target_joint_[i] < min_poss_[i] - 1e-10)
					return -2;
			}

			// 专用关节 tg：clear 后插入一条 init（当前位置），再插入关节目标（zone=0）//
			goto_fwd_tg_.clearAllPos();
			goto_fwd_tg_.insertInitPos(1, goto_beg_joint_);
			goto_fwd_tg_.insertLinePos(1, goto_target_joint_, joint_v, joint_a, joint_j, goto_joint_zone_);
			goto_fwd_tg_.updateInsertPos();

			// 初始化专用 is / sr（缓存用目标关节）//
			std::copy(goto_target_joint_, goto_target_joint_ + input_psize_, goto_input_cache_);
			goto_is_.init(goto_beg_joint_);
			goto_sr_.init(resume_target_ratio_*0.1);

			// 切入 goto 状态 //
			state_.compare_exchange_strong(cur, goto_state);
			return 1;
		}
		auto gotoAbsJ(const double* joint_pos, const double* joint_v, const double* joint_a, const double* joint_j) -> std::int64_t {
			auto cur = state_.load();
			PlannerState goto_state;
			if (cur == PlannerState::Uninitialized) goto_state = PlannerState::UninitializedGoto;
			else if (cur == PlannerState::Paused) goto_state = PlannerState::PausedGoto;
			else return -1;

			configGotoPipeline();
			goto_line_ = false;

			// 起点 = 当前位置 //
			model_->getSubInputPos(sub_id_list_.size(), sub_id_list_.data(), goto_beg_joint_);

			// 检查目标关节位置是否超出运动范围限制 //
			for (int i = 0; i < input_psize_; ++i) {
				if (joint_pos[i] > max_poss_[i] + 1e-10 || joint_pos[i] < min_poss_[i] - 1e-10)
					return -2;
			}

			// 专用关节 tg：clear 后插入一条 init（当前位置），再插入 MoveAbsJ 目标（zone=0）//
			goto_fwd_tg_.clearAllPos();
			goto_fwd_tg_.insertInitPos(1, goto_beg_joint_);
			goto_fwd_tg_.insertLinePos(1, joint_pos, joint_v, joint_a, joint_j, goto_joint_zone_);
			goto_fwd_tg_.updateInsertPos();

			// 初始化专用 is / sr //
			std::copy(goto_beg_joint_, goto_beg_joint_ + input_psize_, goto_input_cache_);
			goto_is_.init(goto_beg_joint_);
			goto_sr_.init(resume_target_ratio_*0.1);

			// 切入 goto 状态 //
			state_.compare_exchange_strong(cur, goto_state);
			return 1;
		}
		auto gotoL(TW& tool_wobjs, const double* tw_pos, const double* vel, const double* acc, const double* jerk) -> std::int64_t {
			auto cur = state_.load();
			PlannerState goto_state;
			if (cur == PlannerState::Uninitialized) goto_state = PlannerState::UninitializedGoto;
			else if (cur == PlannerState::Paused) goto_state = PlannerState::PausedGoto;
			else return -1;

			configGotoPipeline();
			goto_line_ = true;

			// 解析工具/工件（直接写入成员缓冲，供 RT 反解使用）//
			resolveToolsAndWobjs(tool_wobjs, goto_tools_, goto_wobjs_);

			// 起点：当前关节/末端位置 //
			model_->getSubInputPos(sub_id_list_.size(), sub_id_list_.data(), goto_beg_joint_);
			model_->getSubOutputPos(sub_id_list_.size(), sub_id_list_.data(), goto_beg_ee_);

			// 反解目标（初值用当前关节，根用当前逆解根）//
			tw_.selectTw(goto_tools_, goto_wobjs_);
			tw_.setTwPos(tw_pos);
			tw_.getEePos(goto_target_ee_);
			model_->getSubWhichInverseRoot(sub_id_list_.size(), sub_id_list_.data(), goto_beg_ee_, goto_beg_joint_, goto_which_roots_);
			auto ret = model_->subInverseKinematics(sub_id_list_.size(), sub_id_list_.data(),
				goto_target_ee_, goto_target_joint_, goto_which_roots_, goto_beg_joint_);
			if (ret < 0)
				return ret;
			for (int i = 0; i < input_psize_; ++i) {
				if (goto_target_joint_[i] > max_poss_[i] + 1e-10 || goto_target_joint_[i] < min_poss_[i] - 1e-10)
					return -2;
			}

			// 当前 tw 位置作为专用 tg 的 init //
			tw_.selectTw(goto_tools_, goto_wobjs_);
			tw_.setEePos(goto_beg_ee_);
			tw_.getTwPos(goto_cur_tw_);

			// 专用笛卡尔 tg：clear 后插入一条 init（当前 tw），再插入 Line 目标（zone=0）//
			goto_inv_tg_.clearAllPos();
			goto_inv_tg_.insertInitPos(1, goto_cur_tw_);
			goto_inv_tg_.insertLinePos(1, tw_pos, vel, acc, jerk, goto_tw_zone_);
			goto_inv_tg_.updateInsertPos();

			// 初始化专用 is / sr（缓存用目标关节，保证反解失败时有值）//
			std::copy(goto_target_joint_, goto_target_joint_ + input_psize_, goto_input_cache_);
			goto_is_.init(goto_beg_joint_);
			goto_sr_.init(resume_target_ratio_*0.1);

			// 切入 goto 状态 //
			state_.compare_exchange_strong(cur, goto_state);
			return 1;
		}
		auto gotoC(TW& tool_wobjs, const double* tw_pos, const double* tw_mid_pos, const double* vel, const double* acc, const double* jerk) -> std::int64_t {
			auto cur = state_.load();
			PlannerState goto_state;
			if (cur == PlannerState::Uninitialized) goto_state = PlannerState::UninitializedGoto;
			else if (cur == PlannerState::Paused) goto_state = PlannerState::PausedGoto;
			else return -1;

			configGotoPipeline();
			goto_line_ = true;

			// 解析工具/工件（直接写入成员缓冲，供 RT 反解使用）//
			resolveToolsAndWobjs(tool_wobjs, goto_tools_, goto_wobjs_);

			// 起点：当前关节/末端位置 //
			model_->getSubInputPos(sub_id_list_.size(), sub_id_list_.data(), goto_beg_joint_);
			model_->getSubOutputPos(sub_id_list_.size(), sub_id_list_.data(), goto_beg_ee_);

			// 反解目标（初值用当前关节，根用当前逆解根）//
			tw_.selectTw(goto_tools_, goto_wobjs_);
			tw_.setTwPos(tw_pos);
			tw_.getEePos(goto_target_ee_);
			model_->getSubWhichInverseRoot(sub_id_list_.size(), sub_id_list_.data(), goto_beg_ee_, goto_beg_joint_, goto_which_roots_);
			auto ret = model_->subInverseKinematics(sub_id_list_.size(), sub_id_list_.data(),
				goto_target_ee_, goto_target_joint_, goto_which_roots_, goto_beg_joint_);
			if (ret < 0)
				return ret;
			for (int i = 0; i < input_psize_; ++i) {
				if (goto_target_joint_[i] > max_poss_[i] + 1e-10 || goto_target_joint_[i] < min_poss_[i] - 1e-10)
					return -2;
			}

			// 当前 tw 位置作为专用 tg 的 init //
			tw_.selectTw(goto_tools_, goto_wobjs_);
			tw_.setEePos(goto_beg_ee_);
			tw_.getTwPos(goto_cur_tw_);

			// 专用笛卡尔 tg：clear 后插入一条 init（当前 tw），再插入 Circle 目标（zone=0）//
			goto_inv_tg_.clearAllPos();
			goto_inv_tg_.insertInitPos(1, goto_cur_tw_);
			goto_inv_tg_.insertCirclePos(1, tw_pos, tw_mid_pos, vel, acc, jerk, goto_tw_zone_);
			goto_inv_tg_.updateInsertPos();

			// 初始化专用 is / sr（缓存用目标关节，保证反解失败时有值）//
			std::copy(goto_target_joint_, goto_target_joint_ + input_psize_, goto_input_cache_);
			goto_is_.init(goto_beg_joint_);
			goto_sr_.init(resume_target_ratio_*0.1);

			// 切入 goto 状态 //
			state_.compare_exchange_strong(cur, goto_state);
			return 1;
		}

		////////////// RT //////////////
		
		// 记录最新 input，并用差分计算速度/加速度：
		// 位置：把新数据写入最旧缓冲区 last2_input_，再轮换三个指针；
		// 速度：差分 (this_input - last_input)/dt 写入最旧速度缓冲区 last_vel_，再轮换指针；
		// 加速度：差分 (this_vel - last_vel)/dt
		auto record_input(const double* p) -> void {
			// 位置 //
			std::copy(p, p + input_psize_, last2_input_);
			std::swap(last2_input_, last_input_);
			std::swap(last_input_, this_input_);

			// 速度 //
			const double dt = inv_tg1_.dt();
			for (aris::Size i = 0; i < input_psize_; ++i)
				last_vel_[i] = (this_input_[i] - last_input_[i]) / dt;
			std::swap(this_vel_, last_vel_);

			// 加速度 //
			for (aris::Size i = 0; i < input_psize_; ++i)
				this_acc_[i] = (this_vel_[i] - last_vel_[i]) / dt;
		};

		// 实时步进入口：根据当前状态分发到各 OneStep，并在其返回 0 时切换终态 //
		auto getNextInput(double* p) -> std::int64_t {
			std::int64_t ret = 0;
			switch (state_.load()) {
			case PlannerState::Uninitialized:
			case PlannerState::Paused:
				ret = 0;
				break;
			case PlannerState::Idle:
				// 首次运行：一旦产生运动（ret != 0），从 Idle 切入 Running //
				ret = runOneStep(p);
				if (ret != 0) {
					auto cur = PlannerState::Idle;
					if(!state_.compare_exchange_strong(cur, PlannerState::Running)) {
						// 如果不是 IDLE，说明非实时循环执行了 stop //
						ret = 0;
					};
				}
				break;
			case PlannerState::Running:
				ret = runOneStep(p);
				if (ret == 0) {
					// 状态可能已被 request 系列函数改变，依次尝试：
					// Running→Idle，失败则 Pausing→Idle，再失败则 Stopping→Uninitialized //
					auto cur1 = PlannerState::Running;
					auto cur2 = PlannerState::Pausing;
					auto cur3 = PlannerState::Stopping;
					if (!state_.compare_exchange_strong(cur1, PlannerState::Idle)
						&& !state_.compare_exchange_strong(cur2, PlannerState::Idle)) {
						state_.compare_exchange_strong(cur3, PlannerState::Uninitialized);
					}
				}
				break;
			case PlannerState::Pausing:
				if (pausing_from_resume_) {
					// 恢复中暂停：用当前速度逐维减速到 0（复用 stopOneStep），
					// pause_pos_ 保持原值，后续 resume 会从当前停止位置继续
					// 平滑运动回 pause_pos_ //
					ret = stopOneStep(p);
					if (ret == 0) {
						auto cur1 = PlannerState::Pausing;
						auto cur2 = PlannerState::Stopping;
						if (!state_.compare_exchange_strong(cur1, PlannerState::Paused)) {
							state_.compare_exchange_strong(cur2, PlannerState::Uninitialized);
						}
						pausing_from_resume_ = false;
					}
				}
				else {
					ret = pauseOneStep(p);
					if (ret == 0) {
						// 状态可能已被 request 系列函数改变，依次尝试：
						// Pausing→Paused，失败则 Stopping→Uninitialized //
						auto cur1 = PlannerState::Pausing;
						auto cur2 = PlannerState::Stopping;
						if (!state_.compare_exchange_strong(cur1, PlannerState::Paused)) {
							state_.compare_exchange_strong(cur2, PlannerState::Uninitialized);
						}
					}
				}
				break;
			case PlannerState::Resuming:
				ret = resumeOneStep(p);
				if (ret == 0) {
					// 状态可能已被 request 系列函数改变，依次尝试：
					// Resuming→Running，失败则 Stopping→Uninitialized //
					auto cur1 = PlannerState::Resuming;
					auto cur2 = PlannerState::Stopping;
					if (!state_.compare_exchange_strong(cur1, PlannerState::Running)) {
						state_.compare_exchange_strong(cur2, PlannerState::Uninitialized);
					}
					else {
						// 恢复完成：恢复目标速度比（pause 时被置为 0），并返回暂停时的节点 id，
						// 保持上层指令存活，下一拍进入 Running 后继续推进原轨迹 //
						sr_.setTargetSpeedRatio(resume_target_ratio_);
						ret = paused_tg_ret_;
					}
				}
				break;
			case PlannerState::Stopping:
				ret = stopOneStep(p);
				if (ret == 0) {
					// 因为状态可能被 request 系列函数改变，所以这里要尝试切换到 Idle 或 Uninitialized //
					auto cur1 = PlannerState::Stopping;
					state_.compare_exchange_strong(cur1, PlannerState::Uninitialized);
				}
				break;
			case PlannerState::PausedGoto:
			case PlannerState::UninitializedGoto:
				ret = gotoOneStep(p);
				if (ret == 0) {
					// 到达目标：回到进入前状态（Paused 或 Uninitialized）//
					auto cur = state_.load();
					auto target = (cur == PlannerState::PausedGoto) ? PlannerState::Paused : PlannerState::Uninitialized;
					state_.compare_exchange_strong(cur, target);
				}
				break;
			default:
				break;
			}
			// 统一：正解更新模型 + 记录 input //
			model_->setSubInputPos(sub_id_list_.size(), sub_id_list_.data(), p);
			model_->subForwardKinematics(sub_id_list_.size(), sub_id_list_.data());
			record_input(p);
			return ret;
		}

		// run-resume-stop 系列步进函数 //
		auto runOneStep(double* p) -> std::int64_t {
			// 原始推进 //
			return sr_.getNextInput(p);
		};
		auto stopOneStep(double* p) -> std::int64_t {
			// 每维在速度域用 s_follow_x 减速一步：
			//   位置 pa = 当前速度 this_vel_，速度 va = 当前加速度 this_acc_，目标 pt = 0（速度降为 0）；
			//   速度上限 v_max/v_min = 加速度上限，加速度上限 a_max/a_min = jerk 上限；
			//   输出 pc = 下一速度，vc = 下一加速度，ac = 下一 jerk；
			//   位置由速度积分递增：p = this_input_ + v_new * dt //
			bool all_stopped = true;
			for (aris::Size i = 0; i < input_psize_; ++i) {
				double v_new, a_new, j_new;
				aris::Size total_count;
				s_follow_x(this_vel_[i], this_acc_[i], 0.0,
					max_accs_[i], -max_accs_[i], max_jerks_[i], -max_jerks_[i],
					inv_tg1_.dt(), 1e-10, v_new, a_new, j_new, total_count);
				p[i] = this_input_[i] + v_new * inv_tg1_.dt();
				if (total_count != 0)
					all_stopped = false;
			}

			// return 
			return all_stopped ? 0 : 1;
		};
		auto pauseOneStep(double* p) -> std::int64_t {
			// Running / Pausing：推进暂停 //
			sr_.setTargetSpeedRatio(0.0);
			
			// 原始推进 //
			paused_tg_ret_ = sr_.getNextInput(p);

			// 速度降到 0 → 已完全暂停：记录暂停位置并返回 0（状态切换为 Paused）//
			if (sr_.actualSpeedRatio() <= speed_epsilon_) {
				std::copy(p, p + input_psize_, pause_pos_);
				return 0;
			}
			return paused_tg_ret_;
		};
		auto resumeOneStep(double* p) -> std::int64_t {
			// Resuming：沿 scurve 走一步 //
			const auto n = input_psize_;
			for (aris::Size i = 0; i < n; ++i) {
				aris::plan::LargeNum pos;
				aris::plan::s_scurve_at(resume_scurve_params_[i], resume_t_, &pos);
				p[i] = static_cast<double>(pos);
			}
			resume_t_ += inv_tg1_.dt();

			// 设置：走完 → 0（状态切换为 Running），否则返回暂停时的节点 id //
			return resume_t_ >= resume_T_ ? 0 : paused_tg_ret_;
		};

		// goto 系列步进函数 //
		auto gotoOneStep(double* p) -> std::int64_t {
			// 推进 goto 专用管线（goto_sr_ → goto_is_ → goto_tg_）//
			return goto_sr_.getNextInput(p);
		};
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
	}
	auto MultimodelPlanner::dt() -> double {
		return imp_->inv_tg1_.dt();
	}

	auto MultimodelPlanner::setMaxPos(aris::core::Matrix pos) -> void {
		imp_->max_pos_mat_ = pos;
		imp_->is_.setMaxPos(pos);
		imp_->sr_.setMaxPos(pos);
	}
	auto MultimodelPlanner::maxPos() -> aris::core::Matrix {
		return imp_->is_.maxPos();
	}
	auto MultimodelPlanner::setMaxVel(aris::core::Matrix vel) -> void {
		imp_->max_vel_mat_ = vel;
		imp_->is_.setMaxVel(vel);
		imp_->sr_.setMaxVel(vel);
	}
	auto MultimodelPlanner::maxVel() -> aris::core::Matrix {
		return imp_->is_.maxVel();
	}
	auto MultimodelPlanner::setMaxAcc(aris::core::Matrix acc) -> void {
		imp_->max_acc_mat_ = acc;
		imp_->is_.setMaxAcc(acc);
		imp_->sr_.setMaxAcc(acc);
	}
	auto MultimodelPlanner::maxAcc() -> aris::core::Matrix {
		return imp_->is_.maxAcc();
	}
	auto MultimodelPlanner::setMaxJerk(aris::core::Matrix jerk) -> void {
		imp_->max_jerk_mat_ = jerk;
	}
	auto MultimodelPlanner::maxJerk() -> aris::core::Matrix {
		return imp_->max_jerk_mat_;
	}
	auto MultimodelPlanner::setMinPos(aris::core::Matrix pos) -> void {
		imp_->min_pos_mat_ = pos;
		imp_->is_.setMinPos(pos);
		imp_->sr_.setMinPos(pos);
	}
	auto MultimodelPlanner::minPos() -> aris::core::Matrix {
		return imp_->is_.minPos();
	}
	auto MultimodelPlanner::setMinVel(aris::core::Matrix vel) -> void {
		imp_->min_vel_mat_ = vel;
		imp_->is_.setMinVel(vel);
		imp_->sr_.setMinVel(vel);
	}
	auto MultimodelPlanner::minVel() -> aris::core::Matrix {
		return imp_->is_.minVel();
	}
	auto MultimodelPlanner::setMinAcc(aris::core::Matrix acc) -> void {
		imp_->min_acc_mat_ = acc;
		imp_->is_.setMinAcc(acc);
		imp_->sr_.setMinAcc(acc);
	}
	auto MultimodelPlanner::minAcc() -> aris::core::Matrix {
		return imp_->is_.minAcc();
	}
	auto MultimodelPlanner::setMinJerk(aris::core::Matrix jerk) -> void {
		imp_->min_jerk_mat_ = jerk;
	}
	auto MultimodelPlanner::minJerk() -> aris::core::Matrix {
		return imp_->min_jerk_mat_;
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
	
	////////////////// PART 2 NRT operation ////////////////

	auto MultimodelPlanner::allocateMemory() -> void {
		imp_->allocateMemory();
	}
	auto MultimodelPlanner::init() -> void {
		imp_->init();
	}

	// 插入新的数据，并重规划 //
	auto MultimodelPlanner::insertMoveL(TW& tw, const double* ee_pos, const double* vel, const double* acc, const double* jerk, const double* zone, double time_zone) -> std::int64_t {	
		return imp_->insLine(tw, ee_pos, vel, acc, jerk, zone, time_zone);
	}
	auto MultimodelPlanner::insertMoveL(std::string_view tools, std::string_view wobjs, const double* ee_pos, const double* vel, const double* acc, const double* jerk, const double* zone, double time_zone) -> std::int64_t {
		auto tool_str_vec = aris::core::split(tools, ';');
		auto wobj_str_vec = aris::core::split(wobjs, ';');

		TW tw;
		for (Size i = 0; i < std::max(tool_str_vec.size(), wobj_str_vec.size()); ++i) {
			auto tool = i < tool_str_vec.size() ? aris::core::trimLR(tool_str_vec[i]) : std::string("");
			auto wobj = i < wobj_str_vec.size() ? aris::core::trimLR(wobj_str_vec[i]) : std::string("");
			tw.push_back(std::pair(tool, wobj));
		}

		return insertMoveL(tw, ee_pos, vel, acc, jerk, zone, time_zone);
	}

	// 插入新的数据，并重规划 //
	auto MultimodelPlanner::insertMoveC(TW& tw, const double* ee_pos, const double* mid_pos, const double* vel, const double* acc, const double* jerk, const double* zone, double time_zone) -> std::int64_t {
		return imp_->insCircle(tw, ee_pos, mid_pos, vel, acc, jerk, zone, time_zone);
	}
	auto MultimodelPlanner::insertMoveC(std::string_view tools, std::string_view wobjs, const double* ee_pos, const double* mid_pos, const double* vel, const double* acc, const double* jerk, const double* zone, double time_zone) -> std::int64_t {
		auto tool_str_vec = aris::core::split(tools, ';');
		auto wobj_str_vec = aris::core::split(wobjs, ';');

		TW tw;
		for (Size i = 0; i < std::max(tool_str_vec.size(), wobj_str_vec.size()); ++i) {
			auto tool = i < tool_str_vec.size() ? aris::core::trimLR(tool_str_vec[i]) : std::string("");
			auto wobj = i < wobj_str_vec.size() ? aris::core::trimLR(wobj_str_vec[i]) : std::string("");
			tw.push_back(std::pair(tool, wobj));
		}

		return insertMoveC(tw, ee_pos, mid_pos, vel, acc, jerk, zone, time_zone);
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

	// 更新插入位置 //
	auto MultimodelPlanner::updateInsertPos() -> void {
		imp_->updateIns();
	}

	
	auto MultimodelPlanner::requestInit() -> std::int64_t {
		return imp_->requestInit();
	}
	auto MultimodelPlanner::requestStop() -> std::int64_t {
		return imp_->requestStop();
	}
	auto MultimodelPlanner::requestPause() -> std::int64_t {
		return imp_->requestPause();
	}
	auto MultimodelPlanner::requestResume() -> std::int64_t {
		return imp_->requestResume();
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
		imp_->resume_target_ratio_ = ds;
		imp_->sr_.setTargetSpeedRatio(ds);
		imp_->goto_sr_.setTargetSpeedRatio(ds*0.1);
	}
	auto MultimodelPlanner::targetSpeedRatio() -> double {
		return imp_->sr_.targetSpeedRatio();
	}
	auto MultimodelPlanner::actualSpeedRatio() -> double {
		return imp_->sr_.actualSpeedRatio();
	}

	////////////////// PART 3 RT operation ////////////////
	auto MultimodelPlanner::getPausedPos(double *input)->void{
		std::copy_n(imp_->pause_pos_, imp_->input_psize_, input);
	}
	
	auto MultimodelPlanner::tgRet() -> std::int64_t {
        return imp_->tg_ret_;
    }
    auto MultimodelPlanner::ikRet() -> std::int64_t{
        return imp_->ik_ret_;
    }
    auto MultimodelPlanner::currnetId() -> std::int64_t {
        return imp_->get_id_;
    }
    auto MultimodelPlanner::finalId() -> std::int64_t {
        return imp_->ins_id_;
    }
    auto MultimodelPlanner::leftNodeS() -> double {
        const auto& node = imp_->map_nodes_[imp_->get_id_ % TW_POOL_SIZE];
        if (node.type_ == MAPNodeType::Line || node.type_ == MAPNodeType::Circle || node.type_ == MAPNodeType::CartesianInitPos) {
            auto& tg = (node.cartesian_planner_idx_ == 0) ? imp_->inv_tg1_ : imp_->inv_tg2_;
            return tg.leftNodeS();
        }
        return imp_->fwd_tg_.leftNodeS();
    }
	auto MultimodelPlanner::inputSize() -> int{
		return imp_->input_psize_;
	}


	// 状态机及相关操作 //
	auto MultimodelPlanner::state() const -> PlannerState {
		return imp_->state_;
	}

	auto MultimodelPlanner::getNextInput(double* p) -> std::int64_t {
		return imp_->getNextInput(p);
	}





	auto MultimodelPlanner::gotoJ(TW& tw, const double* tw_pos, const double* joint_v, const double* joint_a, const double* joint_j, const std::int64_t *which_root) -> std::int64_t {
		return imp_->gotoJ(tw, tw_pos, joint_v, joint_a, joint_j, which_root);
	}
	auto MultimodelPlanner::gotoJ(std::string_view tools, std::string_view wobjs, const double* tw_pos, const double* joint_v, const double* joint_a, const double* joint_j, const std::int64_t *which_root) -> std::int64_t {
		auto tool_str_vec = aris::core::split(tools, ';');
		auto wobj_str_vec = aris::core::split(wobjs, ';');

		TW tw;
		for (Size i = 0; i < std::max(tool_str_vec.size(), wobj_str_vec.size()); ++i) {
			auto tool = i < tool_str_vec.size() ? aris::core::trimLR(tool_str_vec[i]) : std::string("");
			auto wobj = i < wobj_str_vec.size() ? aris::core::trimLR(wobj_str_vec[i]) : std::string("");
			tw.push_back(std::pair(tool, wobj));
		}

		return gotoJ(tw, tw_pos, joint_v, joint_a, joint_j, which_root);
	}
	auto MultimodelPlanner::gotoAbsJ(const double* joint_pos, const double* joint_v, const double* joint_a, const double* joint_j) -> std::int64_t {
		return imp_->gotoAbsJ(joint_pos, joint_v, joint_a, joint_j);
	}
	auto MultimodelPlanner::gotoL(TW& tw, const double* tw_pos, const double* vel, const double* acc, const double* jerk) -> std::int64_t {
		return imp_->gotoL(tw, tw_pos, vel, acc, jerk);
	}
	auto MultimodelPlanner::gotoL(std::string_view tools, std::string_view wobjs, const double* tw_pos, const double* vel, const double* acc, const double* jerk) -> std::int64_t {
		auto tool_str_vec = aris::core::split(tools, ';');
		auto wobj_str_vec = aris::core::split(wobjs, ';');

		TW tw;
		for (Size i = 0; i < std::max(tool_str_vec.size(), wobj_str_vec.size()); ++i) {
			auto tool = i < tool_str_vec.size() ? aris::core::trimLR(tool_str_vec[i]) : std::string("");
			auto wobj = i < wobj_str_vec.size() ? aris::core::trimLR(wobj_str_vec[i]) : std::string("");
			tw.push_back(std::pair(tool, wobj));
		}

		return gotoL(tw, tw_pos, vel, acc, jerk);
	}
	auto MultimodelPlanner::gotoC(TW& tw, const double* tw_pos, const double* tw_mid_pos, const double* vel, const double* acc, const double* jerk) -> std::int64_t {
		return imp_->gotoC(tw, tw_pos, tw_mid_pos, vel, acc, jerk);
	}
	auto MultimodelPlanner::gotoC(std::string_view tools, std::string_view wobjs, const double* tw_pos, const double* tw_mid_pos, const double* vel, const double* acc, const double* jerk) -> std::int64_t {
		auto tool_str_vec = aris::core::split(tools, ';');
		auto wobj_str_vec = aris::core::split(wobjs, ';');

		TW tw;
		for (Size i = 0; i < std::max(tool_str_vec.size(), wobj_str_vec.size()); ++i) {
			auto tool = i < tool_str_vec.size() ? aris::core::trimLR(tool_str_vec[i]) : std::string("");
			auto wobj = i < wobj_str_vec.size() ? aris::core::trimLR(wobj_str_vec[i]) : std::string("");
			tw.push_back(std::pair(tool, wobj));
		}

		return gotoC(tw, tw_pos, tw_mid_pos, vel, acc, jerk);
	}


    MultimodelPlanner::~MultimodelPlanner(){
        imp_->state_.store(PlannerState::Uninitialized);
    }
    MultimodelPlanner::MultimodelPlanner():imp_(new Imp) {}
}
