#ifndef ARIS_DYNAMIC_EXTERNAL_AXIS_H_
#define ARIS_DYNAMIC_EXTERNAL_AXIS_H_

#include <array>
#include <aris/dynamic/model_solver.hpp>

namespace aris::dynamic {
	/// @defgroup dynamic_model_group 动力学建模模块
	/// @{
	///
	
	// pos  为外部轴的位置
	// axis 为外部轴的轴方向，例如 x 向，axis应为[1,0,0]
	// is_revolute: 是否为转轴
	// is_coupling: 是否与其他模型耦合，仅影响规划
	auto inline createExternalAxisModel(const double* pos, const double* axis, bool is_revolute, bool is_coupling = false)->std::unique_ptr < Model > {
		auto m = std::make_unique<aris::dynamic::Model>();
		m->setName("ExAxisModel");

		auto& platform = m->partPool().add<Part>("EE");

		m->variablePool().add<MatrixVariable>("pos", aris::core::Matrix(1, 3, pos));
		m->variablePool().add<MatrixVariable>("axis", aris::core::Matrix(1, 3, axis));
		m->variablePool().add<BoolVariable>("is_revolute", is_revolute);
		m->variablePool().add<BoolVariable>("is_coupling", is_coupling);

		if (is_revolute) {
			auto& joint = m->addRevoluteJoint(platform, m->ground(), pos, axis);
			auto& motion = m->addMotion(joint);
			motion.setRotateRange(std::numeric_limits<double>::quiet_NaN());
			auto& ee_mot = m->generalMotionPool().add<Motion>("ee", joint.makI(), joint.makJ(), 5);
			ee_mot.setRotateRange(std::numeric_limits<double>::quiet_NaN());
		}
		else {
			auto& joint = m->addPrismaticJoint(platform, m->ground(), pos, axis);
			auto& motion = m->addMotion(joint);
			auto& ee_mot = m->generalMotionPool().add<Motion>("ee", joint.makI(), joint.makJ(), 2);
		}

		// add tool & wobj
		for (int i = 1; i < 8; ++i) {
			// 把 tool 的pe 默认设置到转台位置处
			double tool_pe[6]{ pos[0], pos[1], pos[2], 0.0,0.0,0.0 };
			platform.addMarker("tool" + std::to_string(i)).setPrtPe(tool_pe);
			m->ground().markerPool().add<aris::dynamic::Marker>("wobj" + std::to_string(i));
		}

		// add solver
		auto& inverse_kinematic = m->solverPool().add<aris::dynamic::InverseKinematicSolver>();
		auto& forward_kinematic = m->solverPool().add<aris::dynamic::SerialForwardKinematicSolver>();
		auto& inverse_dynamic = m->solverPool().add<aris::dynamic::InverseDynamicSolver>();
		auto& forward_dynamic = m->solverPool().add<aris::dynamic::ForwardDynamicSolver>();

		m->init();

		return m;
	}

	///
	/// @}
}

#endif
