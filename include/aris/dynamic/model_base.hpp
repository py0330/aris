#ifndef ARIS_DYNAMIC_MODEL_BASE_H_
#define ARIS_DYNAMIC_MODEL_BASE_H_

#include <aris/core/expression_calculator.hpp>

#include <aris/dynamic/model_solver.hpp>
#include <aris/dynamic/model_simulation.hpp>
#include <aris/dynamic/plan.hpp>

namespace aris::dynamic
{
	class ARIS_API EndEffector{
	public:
		auto virtual dim()->Size;
		auto virtual pos()->double*;
		auto virtual setPos(double *pos)->void;
		auto virtual vel()->double*;
		auto virtual setVel(double *vel)->void;
		auto virtual acc()->double*;
		auto virtual setAcc(double *acc)->void;
	};

	class ARIS_API ModelBase{
	public:
		// kinematics & dynamics //
		auto virtual inverseKinematics()->int;
		auto virtual forwardKinematics()->int;
		auto virtual inverseKinematicsVel()->void;
		auto virtual forwardKinematicsVel()->void;
		auto virtual inverseDynamics()->void;
		auto virtual forwardDynamics()->void;

		// inputs //
		auto virtual motionDim()->Size;
		auto virtual getMotionPos(double *mp)const ->void;
		auto virtual setMotionPos(const double *mp)->void;
		auto virtual getMotionVel(double *mv)const ->void;
		auto virtual setMotionVel(const double *mv)->void;
		auto virtual getMotionAcc(double *ma)const ->void;
		auto virtual setMotionAcc(const double *ma)->void;
		auto virtual getMotionFce(double *mf)const ->void;
		auto virtual setMotionFce(const double *mf)->void;

		// end-effectors //
		auto virtual endEffectorSize()->Size;
		auto virtual endEffector(Size i = 0)->EndEffector*;
	};


}

#endif
