#include"aris/plan/multimodel_async_planner.hpp"
#include"aris/plan/function.hpp"
#include"aris/control/rt_timer.hpp"
#include"aris/plan/input_smoother.hpp"
#include"aris/plan/async_generator.hpp"
#include"aris/plan/speed_regulator.hpp"

namespace aris::plan {

	struct MultimodelAsyncPlanner::Imp {

		using MarkerVec = std::vector<aris::dynamic::Marker*>;
		using ToolWobjNode = std::tuple<std::int64_t, MarkerVec, MarkerVec>;
		
		std::list<ToolWobjNode> tool_wobjs_;
		std::list<ToolWobjNode>::iterator current_node_;
		std::int64_t id_{ 1 };

		TrajectoryGenerator tg_;
		InputSmoother is_;
		AsyncGenerator ag_;
		SpeedRegulator sr_;
		aris::dynamic::MultiModel* model_{nullptr};
		std::unique_ptr<aris::dynamic::MultiModel> local_model_{nullptr};

		auto setModelMarker(MarkerVec& tools, MarkerVec& wobjs, aris::dynamic::MultiModel* model, const double* eepos)->void {
			
			aris::Size internal_idx{ 0 }, out_idx{ 0 };
			

			


			std::vector<aris::Size> order;
			
			// Step 1 补全所有的tool wobj //
			for (auto i = 0; i < model->eeSize(); ++i) {
				auto &tool = tools[i];
				auto &wobj = wobjs[i];

				if (tool == nullptr)
					tool = model->getEes()[i]->makI();
				if (wobj == nullptr)
					wobj = model->getEes()[i]->makJ();
			}

			// Step 2 计算有限顺序 //
			// 原则1：是否连接地面
			// 原则2：是否连接已知杆件
			// 原则3：（尚未支持）
			// 
			// 编码（注意是10进制）：1e10*is_ground + 1e7*connect_known_num
			//
			std::vector<aris::dynamic::Part*> part_connected;
			auto weight_compute = [](const std::vector<aris::dynamic::Part*> &part_connected, const aris::dynamic::Marker *tool, const aris::dynamic::Marker *wobj)->std::int64_t {
				std::int64_t weight = 0;
				
				// 原则1，判断是否连接地面 //
				if (&wobj->fatherPart() == &wobj->model()->ground()) {
					weight += 2e10;
				}

				if (&tool->fatherPart() == &tool->model()->ground()) {
					weight += 1e10;
				}

				// 原则2，判断是否连接已知杆件 //
				if (std::find(part_connected.begin(), part_connected.end(), &wobj->fatherPart()) != part_connected.end()) {
					weight += 2e7;
				}

				if (std::find(part_connected.begin(), part_connected.end(), &tool->fatherPart()) != part_connected.end()) {
					weight += 1e7;
				}
			};
			for (auto i = 0; i < model->eeSize(); ++i) {
				auto& tool = tools[i];
				auto& wobj = wobjs[i];

				if (tool == nullptr)
					tool = model->getEes()[i]->makI();
				if (wobj == nullptr)
					wobj = model->getEes()[i]->makJ();
			}
				/*
				switch (model->eeTypes()[i]) {
				case aris::dynamic::EEType::PE313: {
					tool->setPe(*wobj, eepos + out_idx);
					aris::dynamic::s_pe2pq(out_pos + out_idx, internal_pos + internal_idx, "313");
					internal_idx += 7;
					out_idx += 6;
					break;
				}
				case aris::dynamic::EEType::PE321: {
					aris::dynamic::s_pe2pq(out_pos + out_idx, internal_pos + internal_idx, "321");
					internal_idx += 7;
					out_idx += 6;
					break;
				}
				case aris::dynamic::EEType::PE123: {
					aris::dynamic::s_pe2pq(out_pos + out_idx, internal_pos + internal_idx, "123");
					internal_idx += 7;
					out_idx += 6;
					break;
				}
				case aris::dynamic::EEType::PQ: {
					aris::dynamic::s_vc(7, out_pos + out_idx, internal_pos + internal_idx);
					internal_idx += 7;
					out_idx += 7;
					break;
				}
				case aris::dynamic::EEType::PM: {
					aris::dynamic::s_pm2pq(out_pos + out_idx, internal_pos + internal_idx);
					internal_idx += 7;
					out_idx += 16;
					break;
				}
				case aris::dynamic::EEType::RE313: {
					aris::dynamic::s_re2rq(out_pos + out_idx, internal_pos + internal_idx, "313");
					internal_idx += 4;
					out_idx += 3;
					break;
				}
				case aris::dynamic::EEType::RE321: {
					aris::dynamic::s_re2rq(out_pos + out_idx, internal_pos + internal_idx, "321");
					internal_idx += 4;
					out_idx += 3;
					break;
				}
				case aris::dynamic::EEType::RE123: {
					aris::dynamic::s_re2rq(out_pos + out_idx, internal_pos + internal_idx, "123");
					internal_idx += 4;
					out_idx += 3;
					break;
				}
				case aris::dynamic::EEType::RQ: {
					aris::dynamic::s_vc(4, out_pos + out_idx, internal_pos + internal_idx);
					internal_idx += 4;
					out_idx += 4;
					break;
				}
				case aris::dynamic::EEType::RM: {
					aris::dynamic::s_rm2rq(out_pos + out_idx, internal_pos + internal_idx);
					internal_idx += 9;
					out_idx += 9;
					break;
				}
				case aris::dynamic::EEType::XYZT: {
					aris::dynamic::s_vc(4, out_pos + out_idx, internal_pos + internal_idx);
					internal_idx += 4;
					out_idx += 4;
					break;
				}
				case aris::dynamic::EEType::XYZ: {
					aris::dynamic::s_vc(3, out_pos + out_idx, internal_pos + internal_idx);
					internal_idx += 3;
					out_idx += 3;
					break;
				}
				case aris::dynamic::EEType::XYT: {
					aris::dynamic::s_vc(3, out_pos + out_idx, internal_pos + internal_idx);
					internal_idx += 3;
					out_idx += 3;
					break;
				}
				case aris::dynamic::EEType::RTZ: {
					aris::dynamic::s_vc(3, out_pos + out_idx, internal_pos + internal_idx);
					internal_idx += 3;
					out_idx += 3;
					break;
				}
				case aris::dynamic::EEType::XY: {
					aris::dynamic::s_vc(2, out_pos + out_idx, internal_pos + internal_idx);
					internal_idx += 2;
					out_idx += 2;
					break;
				}
				case aris::dynamic::EEType::RT: {
					aris::dynamic::s_vc(2, out_pos + out_idx, internal_pos + internal_idx);
					internal_idx += 2;
					out_idx += 2;
					break;
				}
				case aris::dynamic::EEType::X: {
					aris::dynamic::s_vc(1, out_pos + out_idx, internal_pos + internal_idx);
					internal_idx += 1;
					out_idx += 1;
					break;
				}
				case aris::dynamic::EEType::A: {
					aris::dynamic::s_vc(1, out_pos + out_idx, internal_pos + internal_idx);
					internal_idx += 1;
					out_idx += 1;
					break;
				}
				case aris::dynamic::EEType::UNKNOWN:
					break;
				default:
					break;
				}
				*/
		
			
			for (auto i = 0; i < model->eeSize(); ++i) {
				//enum class EEType {
				//	PE313,   // 位置与313欧拉角，6维末端， 6维向量
				//	PE321,   // 位置与321欧拉角，6维末端， 6维向量
				//	PE123,   // 位置与123欧拉角，6维末端， 6维向量
				//	PQ,      // 位置与四元数，   6维末端， 7维向量
				//	PM,      // 位置与位姿矩阵， 6维末端，16维向量
				//	RE313,   // 313欧拉角，      3维末端， 3维向量
				//	RE321,   // 321欧拉角，      3维末端， 3维向量
				//	RE123,   // 123欧拉角，      3维末端， 3维向量
				//	RQ,      // 四元数，         3维末端， 4维向量
				//	RM,      // 位姿矩阵，       3维末端， 9维向量
				//	XYZT,    // x,y,z,theta，    4维末端， 4维向量
				//	XYZ,     // x,y,z，          3维末端， 3维向量
				//	RTZ,     // 极坐标r,theta,z，3维末端， 3维向量
				//	XYT,     // x,y,theta，      3维末端， 3维向量
				//	XY,      // x,y，            2维末端， 2维向量
				//	RT,      // 极坐标r,theta，  2维末端， 2维向量
				//	X,       // 位置x，          1维末端， 1维向量
				//	A,       // 角度a，          1维末端， 1维向量
				//	UNKNOWN,
				//};

				switch (model->eeTypes()[i]) {
				
				}

			}
		}
		auto getModelEEPos(MarkerVec& tools, MarkerVec& wobjs, aris::dynamic::MultiModel* model, double* eepos);
	};

	


	////////////////// PART 1 config ////////////////

	auto MultimodelAsyncPlanner::setModel(aris::dynamic::MultiModel& model) -> void {
		imp_->model_ = &model;
	}
	auto MultimodelAsyncPlanner::model() -> aris::dynamic::MultiModel&{
		return *imp_->model_;
	}

	// 配置末端类型 //
	auto MultimodelAsyncPlanner::eeTypes()const -> const std::vector<aris::dynamic::EEType>& {
		return imp_->tg_.eeTypes();
	}
	auto MultimodelAsyncPlanner::setEeTypes(const std::vector<aris::dynamic::EEType>& ee_types) -> void {
		imp_->tg_.setEeTypes(ee_types);
	}

	auto MultimodelAsyncPlanner::setInputSize(int input_size) -> void {
		imp_->is_.setInputSize(input_size);
		imp_->ag_.setInputSize(input_size);
		imp_->sr_.setInputSize(input_size);
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
	}
	auto MultimodelAsyncPlanner::init() -> void {
		
		
		
		std::vector<double> init_ee_pos_;
		
		imp_->local_model_ = std::make_unique<aris::dynamic::MultiModel>();
		aris::core::fromJsonString(*imp_->local_model_, aris::core::toJsonString(*imp_->model_));
		
		//imp_->is_.init();
		//imp_->sr_.init();
		//imp_->ag_.init();
	}
	auto MultimodelAsyncPlanner::stop() -> void {
		imp_->ag_.stop();
	}

	// 插入新的数据，并重规划 //
	auto MultimodelAsyncPlanner::insertInitPos(std::int64_t id, const double* ee_pos) -> void {
		
		
		imp_->tg_.insertInitPos(id, ee_pos);
	}

	// 插入新的数据，并重规划 //
	auto MultimodelAsyncPlanner::insertLinePos(std::vector<std::pair<std::string, std::string>> tool_wobjs, const double* ee_pos, const double* vel, const double* acc, const double* jerk, const double* zone) -> void {
		
		imp_->tool_wobjs_.erase(imp_->tool_wobjs_.begin(), imp_->current_node_);

		Imp::MarkerVec tools, wobjs, tools_local, wobjs_local;
		for (auto& tool_wobj : tool_wobjs) {
			tools.push_back(imp_->model_->findTool(tool_wobj.first));
			wobjs.push_back(imp_->model_->findWobj(tool_wobj.second));
			tools_local.push_back(imp_->local_model_->findTool(tool_wobj.first));
			wobjs_local.push_back(imp_->local_model_->findWobj(tool_wobj.second));
		}
		
		imp_->id_++;

		if (imp_->tool_wobjs_.empty() || tools != std::get<1>(imp_->tool_wobjs_.back()) || wobjs != std::get<2>(imp_->tool_wobjs_.back())) {
			//imp_->model_->inverseKinematics()
			std::vector<double> init_ee_pos(aris::dynamic::s_ee_type_pos_size(imp_->tg_.eeTypes().size(), imp_->tg_.eeTypes().data()));

			
			
			imp_->tg_.insertInitPos(imp_->id_, ee_pos);
		}


		imp_->tg_.insertLinePos(imp_->id_, ee_pos, vel, acc, jerk, zone);
	}

	// 插入新的数据，并重规划 //
	auto MultimodelAsyncPlanner::insertCirclePos(std::vector<std::pair<std::string, std::string>> tool_wobjs, const double* ee_pos, const double* mid_pos, const double* vel, const double* acc, const double* jerk, const double* zone) -> void {
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
		return 0;
	}


	MultimodelAsyncPlanner::~MultimodelAsyncPlanner() {
		stop();
	}
	MultimodelAsyncPlanner::MultimodelAsyncPlanner() {
		imp_->is_.setInputGenerator([this](double* p)->std::int64_t {
			static int count_{ 0 };

			if (count_ == 0) {
				p[0] = -1;
			}
			else {
				p[0] = std::sin(count_ * 0.001) * 10;
			}

			count_++;

			if (count_ > 10000)
				return 0;


			return 1;
		});
	}
}
