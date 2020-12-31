#include <cmath>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <algorithm>
#include <limits>
#include <sstream>
#include <regex>
#include <limits>
#include <type_traits>
#include <ios>

#include "aris/core/core.hpp"
#include "aris/dynamic/model.hpp"

namespace aris::dynamic
{
	struct Marker::Imp{
		double prt_pm_[4][4]{ { 0 } };
		double pm_[4][4]{ { 0 } };
		Part *part_;
	};
	struct Model::Imp{
		double time_{ 0.0 };
		aris::core::Calculator calculator_;
		Environment environment_;
		std::unique_ptr<aris::core::PointerArray<Variable,          Element>> variable_pool_;
		std::unique_ptr<aris::core::PointerArray<Part,              Element>> part_pool_;
		std::unique_ptr<aris::core::PointerArray<Joint,             Element>> joint_pool_;
		std::unique_ptr<aris::core::PointerArray<Motion,            Element>> motion_pool_;
		std::unique_ptr<aris::core::PointerArray<GeneralMotionBase, Element>> general_motion_pool_;
		std::unique_ptr<aris::core::PointerArray<Force,             Element>> force_pool_;
		std::unique_ptr<aris::core::PointerArray<Solver,            Element>> solver_pool_;
		std::unique_ptr<aris::core::PointerArray<Simulator,         Element>> simulator_pool_;
		std::unique_ptr<aris::core::PointerArray<SimResult,         Element>> sim_result_pool_;
		std::unique_ptr<aris::core::PointerArray<Calibrator,        Element>> calibrator_pool_;

		Part* ground_;
	};
	auto Model::init()->void { 
		auto init_interaction = [](Interaction &interaction, Model*m)->void
		{
			if (interaction.prtNameM().empty() && interaction.prtNameN().empty() && interaction.makNameI().empty() && interaction.makNameJ().empty())return;

			auto find_part = [m](std::string_view name)->Part*
			{
				auto found = std::find_if(m->partPool().begin(), m->partPool().end(), [name](const auto &part)->bool 
				{
					return part.name() == name;
				});
				return found == m->partPool().end() ? nullptr : &*found;
			};

			auto find_marker = [](Part *part, std::string_view name)->Marker*
			{
				auto found = std::find_if(part->markerPool().begin(), part->markerPool().end(), [name](const auto &marker)->bool
				{
					return marker.name() == name;
				});
				return found == part->markerPool().end() ? nullptr : &*found;
			};


			auto prt_m = find_part(interaction.prtNameM());
			auto mak_i = find_marker(prt_m, interaction.makNameI());
			auto prt_n = find_part(interaction.prtNameN());
			auto mak_j = find_marker(prt_n, interaction.makNameJ());

			interaction.makI_ = &*mak_i;
			interaction.makJ_ = &*mak_j;
		};

		auto ground = std::find_if(partPool().begin(), partPool().end(), [](const auto &part)->bool
		{
			return part.name() == "ground";
		});
		imp_->ground_ = ground == partPool().end() ? &partPool().add<Part>("ground") : &*ground;

		variablePool().model_ = this;
		for (auto &ele : variablePool())ele.model_ = this;
		partPool().model_ = this;
		for (Size i = 0; i< partPool().size(); ++i)
		{
			partPool()[i].model_ = this;
			partPool()[i].id_ = i;
			for (Size j = 0; j < partPool()[i].markerPool().size(); ++j)
			{
				partPool()[i].markerPool()[j].model_ = this;
				partPool()[i].markerPool()[j].id_ = j;
				partPool()[i].markerPool()[j].imp_->part_ = &partPool()[i];
			}
		}
		jointPool().model_ = this;
		for (Size i = 0; i< jointPool().size(); ++i)
		{
			jointPool()[i].model_ = this;
			jointPool()[i].id_ = i;
			init_interaction(jointPool()[i], this);
		}
		motionPool().model_ = this;
		for (Size i = 0; i< motionPool().size(); ++i) {
			motionPool()[i].model_ = this;
			motionPool()[i].id_ = i;
			init_interaction(motionPool()[i], this);
		}
		generalMotionPool().model_ = this;
		for (Size i = 0; i< generalMotionPool().size(); ++i) {
			generalMotionPool()[i].model_ = this;
			generalMotionPool()[i].id_ = i;
			init_interaction(generalMotionPool()[i], this);
		}
		forcePool().model_ = this;
		for (Size i = 0; i< forcePool().size(); ++i) {
			forcePool()[i].model_ = this;
			forcePool()[i].id_ = i;
			init_interaction(forcePool()[i], this);
		}
		solverPool().model_ = this;
		for (Size i = 0; i< solverPool().size(); ++i){
			solverPool()[i].model_ = this;
			solverPool()[i].id_ = i;
		}
		simulatorPool().model_ = this;
		for (Size i = 0; i< simulatorPool().size(); ++i) {
			simulatorPool()[i].model_ = this;
			simulatorPool()[i].id_ = i;
		}
		simResultPool().model_ = this;
		for (Size i = 0; i< simResultPool().size(); ++i) {
			simResultPool()[i].model_ = this;
			simResultPool()[i].id_ = i;
		}
		calibratorPool().model_ = this;
		for (Size i = 0; i< calibratorPool().size(); ++i) {
			calibratorPool()[i].model_ = this;
			calibratorPool()[i].id_ = i;
		}
		
		// alloc mem for solvers //
		for (auto &s : this->solverPool()) s.allocateMemory();
			
	}
	auto Model::findVariable(std::string_view name)->Variable*{
		auto found = std::find_if(variablePool().begin(), variablePool().end(), [name](const auto &variable)->auto{
			return variable.name() == name;
		});
		return found == variablePool().end() ? nullptr: &*found;
	}
	auto Model::findPart(std::string_view name)->Part*{
		auto found = std::find_if(partPool().begin(), partPool().end(), [name](const auto &variable)->auto{
			return variable.name() == name;
		});
		return found == partPool().end() ? nullptr : &*found;
	}
	auto Model::inverseKinematics()->int { return solverPool()[0].kinPos(); }
	auto Model::forwardKinematics()->int { return solverPool()[1].kinPos(); }
	auto Model::inverseKinematicsVel()->int { return solverPool()[0].kinVel(); }
	auto Model::forwardKinematicsVel()->int { return solverPool()[1].kinVel(); }
	auto Model::inverseDynamics()->int { return solverPool()[2].dynAccAndFce(); }
	auto Model::forwardDynamics()->int { return solverPool()[3].dynAccAndFce(); }
	auto Model::motionDim()->Size { return motionPool().size(); }
	auto Model::setMotionPos(const double *mp)->void { for (Size i = 0; i < motionPool().size(); ++i) motionPool()[i].setMp(mp[i]); }
	//auto Model::setMotionPos(double mp, Size which_motion)->void { motionPool()[which_motion].setMp(mp); }
	auto Model::getMotionPos(double *mp)const ->void { for (Size i = 0; i < motionPool().size(); ++i) mp[i] = motionPool()[i].mp(); }
	//auto Model::getMotionPos(Size which_motion)->double { return motionPool()[which_motion].mp(); }
	auto Model::setMotionVel(const double *mv)->void { for (Size i = 0; i < motionPool().size(); ++i) motionPool()[i].setMv(mv[i]); }
	//auto Model::setMotionVel(double mv, Size which_motion)->void { motionPool()[which_motion].setMv(mv); }
	auto Model::getMotionVel(double *mv)const ->void { for (Size i = 0; i < motionPool().size(); ++i) mv[i] = motionPool()[i].mv(); }
	//auto Model::getMotionVel(Size which_motion)->double { return motionPool()[which_motion].mv(); }
	auto Model::setMotionAcc(const double *ma)->void { for (Size i = 0; i < motionPool().size(); ++i) motionPool()[i].setMv(ma[i]); }
	auto Model::getMotionAcc(double *ma)const ->void { for (Size i = 0; i < motionPool().size(); ++i) ma[i] = motionPool()[i].ma(); }
	auto Model::setMotionFce(const double *mf)->void { for (Size i = 0; i < motionPool().size(); ++i) motionPool()[i].setMf(mf[i]); }
	//auto Model::setMotionFce(double mf, Size which_motion)->void { motionPool()[which_motion].setMf(mf); }
	auto Model::getMotionFce(double *mf)const->void { for (Size i = 0; i < motionPool().size(); ++i) mf[i] = motionPool()[i].mf(); }
	//auto Model::getMotionFce(Size which_motion)->double { return motionPool()[which_motion].mf(); }
	//auto Model::setEndEffectorPm(const double *pm, Size which_ee)->void { generalMotionPool()[which_ee].setMpm(pm); }
	//auto Model::getEndEffectorPm(double *pm, Size which_ee)->void { generalMotionPool()[which_ee].getMpm(pm); }
	auto Model::endEffectorSize()->Size { return 0; }
	auto Model::endEffector(Size i)->EndEffectorBase* { return nullptr; }
	
	auto Model::time()const->double { return imp_->time_; }
	auto Model::setTime(double time)->void { imp_->time_ = time; }
	auto Model::calculator()->aris::core::Calculator& { return imp_->calculator_; }
	auto Model::environment()->aris::dynamic::Environment& { return imp_->environment_; }
	auto Model::resetVariablePool(aris::core::PointerArray<Variable, Element> *pool)->void { imp_->variable_pool_.reset(pool); }
	auto Model::variablePool()->aris::core::PointerArray<Variable, Element>& { return *imp_->variable_pool_; }
	auto Model::resetPartPool(aris::core::PointerArray<Part, Element> *pool)->void { imp_->part_pool_.reset(pool); }
	auto Model::partPool()->aris::core::PointerArray<Part, Element>& { return *imp_->part_pool_; }
	auto Model::resetJointPool(aris::core::PointerArray<Joint, Element> *pool)->void { imp_->joint_pool_.reset(pool); }
	auto Model::jointPool()->aris::core::PointerArray<Joint, Element>& { return *imp_->joint_pool_; }
	auto Model::resetMotionPool(aris::core::PointerArray<Motion, Element> *pool)->void { imp_->motion_pool_.reset(pool); }
	auto Model::motionPool()->aris::core::PointerArray<Motion, Element>& { return *imp_->motion_pool_; }
	auto Model::resetGeneralMotionPool(aris::core::PointerArray<GeneralMotionBase, Element> *pool)->void { imp_->general_motion_pool_.reset(pool); }
	auto Model::generalMotionPool()->aris::core::PointerArray<GeneralMotionBase, Element>& { return *imp_->general_motion_pool_; }
	auto Model::resetForcePool(aris::core::PointerArray<Force, Element> *pool)->void { imp_->force_pool_.reset(pool); }
	auto Model::forcePool()->aris::core::PointerArray<Force, Element>& { return *imp_->force_pool_; }
	auto Model::resetSolverPool(aris::core::PointerArray<Solver, Element> *pool)->void { imp_->solver_pool_.reset(pool); }
	auto Model::solverPool()->aris::core::PointerArray<Solver, Element>& { return *imp_->solver_pool_; }
	auto Model::simulatorPool()->aris::core::PointerArray<Simulator, Element>& { return *imp_->simulator_pool_; }
	auto Model::simResultPool()->aris::core::PointerArray<SimResult, Element>& { return *imp_->sim_result_pool_; }
	auto Model::calibratorPool()->aris::core::PointerArray<Calibrator, Element>& { return *imp_->calibrator_pool_; }
	auto Model::ground()->Part& { return *imp_->ground_; }
	auto Model::addPartByPm(const double*pm, const double *prt_im)->Part& { 
		auto &ret = partPool().add<Part>("part_" + std::to_string(partPool().size()), prt_im, pm);
		ret.Element::model_ = this;
		return ret;
	}
	auto Model::addPartByPe(const double*pe, const char* eul_type, const double *prt_im)->Part&{
		double pm[16];
		s_pe2pm(pe, pm, eul_type);
		auto &ret = partPool().add<Part>("part_" + std::to_string(partPool().size()), prt_im, pm);
		ret.Element::model_ = this;
		return ret;
	}
	auto Model::addPartByPq(const double*pq, const double *prt_im)->Part&{
		double pm[16];
		s_pq2pm(pq, pm);
		auto &ret = partPool().add<Part>("part_" + std::to_string(partPool().size()), prt_im, pm);
		ret.Element::model_ = this;
		return ret;
	}
	auto Model::addRevoluteJoint(Part &first_part, Part &second_part, const double *position, const double *axis)->RevoluteJoint&{
		double glb_pm[16], loc_pm[16];
		s_sov_axes2pm(position, axis, axis, glb_pm, "zx");
		auto name = "joint_" + std::to_string(jointPool().size());
		s_inv_pm_dot_pm(*first_part.pm(), glb_pm, loc_pm);
		auto &mak_i = first_part.addMarker(name + "_i", loc_pm);
		s_inv_pm_dot_pm(*second_part.pm(), glb_pm, loc_pm);
		auto &mak_j = second_part.addMarker(name + "_j", loc_pm);

		auto &ret = jointPool().add<RevoluteJoint>(name, &mak_i, &mak_j);
		ret.Element::model_ = this;
		return ret;
	}
	auto Model::addPrismaticJoint(Part &first_part, Part &second_part, const double *position, const double *axis)->PrismaticJoint&{
		double glb_pm[16], loc_pm[16];
		s_sov_axes2pm(position, axis, axis, glb_pm, "zx");
		auto name = "joint_" + std::to_string(jointPool().size());
		s_inv_pm_dot_pm(*first_part.pm(), glb_pm, loc_pm);
		auto &mak_i = first_part.addMarker(name + "_i", loc_pm);
		s_inv_pm_dot_pm(*second_part.pm(), glb_pm, loc_pm);
		auto &mak_j = second_part.addMarker(name + "_j", loc_pm);

		auto &ret = jointPool().add<PrismaticJoint>(name, &mak_i, &mak_j);
		ret.Element::model_ = this;
		return ret;
	}
	auto Model::addUniversalJoint(Part &first_part, Part &second_part, const double *position, const double *first_axis, const double *second_axis)->UniversalJoint&{
		double glb_pm[16], loc_pm[16];
		s_sov_axes2pm(position, first_axis, second_axis, glb_pm, "zx");
		auto name = "joint_" + std::to_string(jointPool().size());
		s_inv_pm_dot_pm(*first_part.pm(), glb_pm, loc_pm);
		auto &mak_i = first_part.addMarker(name + "_i", loc_pm);
		s_sov_axes2pm(position, second_axis, first_axis, glb_pm, "zx");
		s_inv_pm_dot_pm(*second_part.pm(), glb_pm, loc_pm);
		auto &mak_j = second_part.addMarker(name + "_j", loc_pm);

		auto &ret = jointPool().add<UniversalJoint>(name, &mak_i, &mak_j);
		ret.Element::model_ = this;
		return ret;
	}
	auto Model::addSphericalJoint(Part &first_part, Part &second_part, const double *position)->SphericalJoint&{
		double glb_pm[16]{ 1,0,0,position[0],0,1,0,position[1],0,0,1,position[2],0,0,0,1 }, loc_pm[16];
		auto name = "joint_" + std::to_string(jointPool().size());
		s_inv_pm_dot_pm(*first_part.pm(), glb_pm, loc_pm);
		auto &mak_i = first_part.addMarker(name + "_i", loc_pm);
		s_inv_pm_dot_pm(*second_part.pm(), glb_pm, loc_pm);
		auto &mak_j = second_part.addMarker(name + "_j", loc_pm);
		auto &ret = jointPool().add<SphericalJoint>(name, &mak_i, &mak_j);
		ret.Element::model_ = this;
		return ret;
	}
	auto Model::addMotion(Joint &joint)->Motion&{
		Size dim;

		if (dynamic_cast<RevoluteJoint*>(&joint))
		{
			dim = 5;
		}
		else if (dynamic_cast<PrismaticJoint*>(&joint))
		{
			dim = 2;
		}
		else
		{
			THROW_FILE_LINE("wrong joint when Model::addMotion(joint)");
		}

		auto &ret = motionPool().add<Motion>("motion_" + std::to_string(motionPool().size()), joint.makI(), joint.makJ(), dim);
		ret.Element::model_ = this;
		return ret;
	}
	auto Model::addMotion()->Motion&{
		auto &ret = motionPool().add<Motion>("motion_" + std::to_string(motionPool().size()), nullptr, nullptr, 0);
		ret.Element::model_ = this;
		return ret;
	}
	auto Model::addGeneralMotionByPm(Part &end_effector, Coordinate &reference, const double* pm)->GeneralMotion&{
		double pm_prt[16], pm_target_in_ground[16];
		s_pm_dot_pm(*reference.pm(), pm, pm_target_in_ground);
		s_inv_pm_dot_pm(*end_effector.pm(), pm_target_in_ground, pm_prt);

		auto name = "general_motion_" + std::to_string(generalMotionPool().size());
		auto &mak_i = end_effector.addMarker(name + "_i", pm_prt);
		auto &mak_j = dynamic_cast<Part*>(&reference) ? dynamic_cast<Part&>(reference).addMarker(name + "_j") : dynamic_cast<Marker&>(reference);
		
		auto &ret = generalMotionPool().add<GeneralMotion>(name, &mak_i, &mak_j);
		ret.Element::model_ = this;
		return ret;
	}
	auto Model::addGeneralMotionByPe(Part &end_effector, Coordinate &reference, const double* pe, const char* eul_type)->GeneralMotion&{
		auto &ret = addGeneralMotionByPm(end_effector, reference, s_pe2pm(pe, nullptr, eul_type));
		ret.Element::model_ = this;
		return ret;
	}
	auto Model::addGeneralMotionByPq(Part &end_effector, Coordinate &reference, const double* pq)->GeneralMotion&{
		auto &ret = addGeneralMotionByPm(end_effector, reference, s_pq2pm(pq));
		ret.Element::model_ = this;
		return ret;
	}
	Model::~Model() = default;
	Model::Model(){
		imp_->variable_pool_.reset(new aris::core::PointerArray<Variable, Element>);
		imp_->part_pool_.reset(new aris::core::PointerArray<Part, Element>);
		imp_->joint_pool_.reset(new aris::core::PointerArray<Joint, Element>);
		imp_->motion_pool_.reset(new aris::core::PointerArray<Motion, Element>);
		imp_->general_motion_pool_.reset(new aris::core::PointerArray<GeneralMotionBase, Element>);
		imp_->force_pool_.reset(new aris::core::PointerArray<Force, Element>);
		imp_->solver_pool_.reset(new aris::core::PointerArray<Solver, Element>);
		imp_->simulator_pool_.reset(new aris::core::PointerArray<Simulator, Element>);
		imp_->calibrator_pool_.reset(new aris::core::PointerArray<Calibrator, Element>);
		imp_->sim_result_pool_.reset(new aris::core::PointerArray<SimResult, Element>);

		imp_->ground_ = &partPool().add<Part>("ground");
	}
	Model::Model(Model&&) = default;
	Model& Model::operator=(Model&&) = default;

	ARIS_REGISTRATION{
		typedef Environment&(Model::*EnvironmentFunc)();
		typedef aris::core::PointerArray<Variable,          Element> &(Model::*VarablePoolFunc)();
		typedef aris::core::PointerArray<Part,              Element> &(Model::*PartPoolFunc)();
		typedef aris::core::PointerArray<Joint,             Element> &(Model::*JointPoolFunc)();
		typedef aris::core::PointerArray<Motion,            Element> &(Model::*MotionPoolFunc)();
		typedef aris::core::PointerArray<GeneralMotionBase, Element> &(Model::*GeneralMotionPoolFunc)();
		typedef aris::core::PointerArray<Force,             Element> &(Model::*ForcePoolFunc)();
		typedef aris::core::PointerArray<Solver,            Element> &(Model::*SolverPoolFunc)();
		typedef aris::core::PointerArray<Simulator,         Element> &(Model::*SimulatorPoolFunc)();
		typedef aris::core::PointerArray<SimResult,         Element> &(Model::*SimResultPoolFunc)();
		typedef aris::core::PointerArray<Calibrator,        Element> &(Model::*CalibratorPoolFunc)();

		aris::core::class_<aris::core::PointerArray<Variable, Element>>("VariablePoolElement")
			.asRefArray()
			;
		aris::core::class_<aris::core::PointerArray<Part, Element>>("PartPoolElement")
			.asRefArray()
			;
		aris::core::class_<aris::core::PointerArray<Joint, Element>>("JointPoolElement")
			.asRefArray()
			;
		aris::core::class_<aris::core::PointerArray<Motion, Element>>("MotionPoolElement")
			.asRefArray()
			;
		aris::core::class_<aris::core::PointerArray<GeneralMotionBase, Element>>("GeneralMotionPoolElement")
			.asRefArray()
			;
		aris::core::class_<aris::core::PointerArray<Force, Element>>("ForcePoolElement")
			.asRefArray()
			;
		aris::core::class_<aris::core::PointerArray<Solver, Element>>("SolverPoolElement")
			.asRefArray()
			;
		aris::core::class_<aris::core::PointerArray<Simulator, Element>>("SimulatorPoolElement")
			.asRefArray()
			;
		aris::core::class_<aris::core::PointerArray<SimResult, Element>>("SimResultPoolElement")
			.asRefArray()
			;
		aris::core::class_<aris::core::PointerArray<Calibrator, Element>>("CalibratorPoolElement")
			.asRefArray()
			;

		aris::core::class_<Model>("Model")
			.prop("time", &Model::setTime, &Model::time)
			.prop("environment", EnvironmentFunc(&Model::environment))
			.prop("variable_pool", &Model::resetVariablePool, VarablePoolFunc(&Model::variablePool))
			.prop("part_pool", &Model::resetPartPool,  PartPoolFunc(&Model::partPool))
			.prop("motion_pool", &Model::resetMotionPool, MotionPoolFunc(&Model::motionPool))
			.prop("joint_pool", &Model::resetJointPool, JointPoolFunc(&Model::jointPool))
			.prop("general_motion_pool", &Model::resetGeneralMotionPool, GeneralMotionPoolFunc(&Model::generalMotionPool))
			.prop("force_pool", &Model::resetForcePool, ForcePoolFunc(&Model::forcePool))
			.prop("solver_pool", &Model::resetSolverPool, SolverPoolFunc(&Model::solverPool))
			//.prop<SimulatorPoolFunc>("simulator_pool", &Model::simulatorPool)
			//.prop<SimResultPoolFunc>("sim_result_pool", &Model::simResultPool)
			//.prop<CalibratorPoolFunc>("calibrator_pool", &Model::calibratorPool)
			;
	}

}
