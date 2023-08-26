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
#include "aris/dynamic/kinematics.hpp"
#include "aris/dynamic/model.hpp"

namespace aris::dynamic{
	struct Marker::Imp{
		double prt_pm_[4][4]{ { 0 } };
		double pm_[4][4]{ { 0 } };
		Part* part_{ nullptr };
	};
	struct Model::Imp {
		double time_{ 0.0 };
		aris::core::Calculator calculator_;
		Environment environment_;
		std::unique_ptr<aris::core::PointerArray<Variable, Element>> variable_pool_;
		std::unique_ptr<aris::core::PointerArray<Part, Element>> part_pool_;
		std::unique_ptr<aris::core::PointerArray<Joint, Element>> joint_pool_;
		std::unique_ptr<aris::core::PointerArray<Motion, Element>> motion_pool_;
		std::unique_ptr<aris::core::PointerArray<MotionBase, Element>> general_motion_pool_;
		std::unique_ptr<aris::core::PointerArray<Force, Element>> force_pool_;
		std::unique_ptr<aris::core::PointerArray<Solver, Element>> solver_pool_;
		std::unique_ptr<aris::core::PointerArray<Simulator, Element>> simulator_pool_;
		std::unique_ptr<aris::core::PointerArray<SimResult, Element>> sim_result_pool_;
		std::unique_ptr<aris::core::PointerArray<Calibrator, Element>> calibrator_pool_;
		Part* ground_{ nullptr };

		std::vector<EEType> ee_types_;
		Size end_effector_pos_size_{ 0 }, end_effector_vel_size_{ 0 }, end_effector_acc_size_{ 0 }, end_effector_fce_size_{ 0 };
	};
	auto Model::init()->void { 
		auto init_interaction = [](Interaction &interaction, Model*m)->void{
			if (interaction.prtNameM().empty() && interaction.prtNameN().empty() && interaction.makNameI().empty() && interaction.makNameJ().empty())return;

			auto find_part = [m](std::string_view name)->Part*{
				auto found = std::find_if(m->partPool().begin(), m->partPool().end(), [name](const auto &part)->bool{
					return part.name() == name;
				});
				return found == m->partPool().end() ? nullptr : &*found;
			};

			auto find_marker = [](Part *part, std::string_view name)->Marker*{
				auto found = std::find_if(part->markerPool().begin(), part->markerPool().end(), [name](const auto &marker)->bool{
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
		auto ground = std::find_if(partPool().begin(), partPool().end(), [](const auto &part)->bool{
			return part.name() == "ground";
		});
		imp_->ground_ = ground == partPool().end() ? &partPool().add<Part>("ground") : &*ground;

		variablePool().model_ = this;
		for (auto &ele : variablePool())ele.model_ = this;
		partPool().model_ = this;
		for (Size i = 0; i< partPool().size(); ++i){
			partPool()[i].model_ = this;
			partPool()[i].id_ = i;
			for (Size j = 0; j < partPool()[i].markerPool().size(); ++j){
				partPool()[i].markerPool()[j].model_ = this;
				partPool()[i].markerPool()[j].id_ = j;
				partPool()[i].markerPool()[j].imp_->part_ = &partPool()[i];
			}
		}
		jointPool().model_ = this;
		for (Size i = 0; i< jointPool().size(); ++i){
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

		imp_->end_effector_pos_size_ = 0;
		imp_->end_effector_vel_size_ = 0;
		imp_->end_effector_acc_size_ = 0;
		imp_->end_effector_fce_size_ = 0;

		for (auto &m : generalMotionPool()) {
			imp_->end_effector_pos_size_ += m.pSize();
			imp_->end_effector_vel_size_ += m.vSize();
			imp_->end_effector_acc_size_ += m.aSize();
			imp_->end_effector_fce_size_ += m.fSize();
			imp_->ee_types_.push_back(m.eeType());
		}

		// alloc mem for solvers //
		for (auto &s : this->solverPool()) s.allocateMemory();
	}
	auto Model::inverseRootNumber()const->int { 
		return solverPool()[0].rootNumber();
	}
	auto Model::whichInverseRoot(const double* output, const double* input)->int { 
		return solverPool()[0].whichRootOfAnswer(output, input);
	}
	auto Model::forwardRootNumber()const->int { 
		return solverPool()[1].rootNumber();
	}
	auto Model::whichForwardRoot(const double* input, const double* output)->int { 
		return solverPool()[1].whichRootOfAnswer(input, output);
	}
	auto Model::inverseKinematics()noexcept->int { return solverPool()[0].kinPos(); }
	auto Model::forwardKinematics()noexcept->int { return solverPool()[1].kinPos(); }
	auto Model::inverseKinematicsVel()noexcept->int { return solverPool()[0].kinVel(); }
	auto Model::forwardKinematicsVel()noexcept->int { return solverPool()[1].kinVel(); }
	auto Model::inverseKinematicsAcc()noexcept->int { return solverPool()[0].dynAccAndFce(); }
	auto Model::forwardKinematicsAcc()noexcept->int { return solverPool()[1].dynAccAndFce(); }
	auto Model::inverseDynamics()noexcept->int { return solverPool()[2].dynAccAndFce(); }
	auto Model::forwardDynamics()noexcept->int { return solverPool()[3].dynAccAndFce(); }
	
	auto Model::inverseKinematics(const double* output, double* input, int which_root)const noexcept->int {
		if (auto c_inv = dynamic_cast<const aris::dynamic::InverseKinematicSolver*>(&solverPool()[0])) {
			auto inv = const_cast<aris::dynamic::InverseKinematicSolver*>(c_inv);
			return inv->kinPosPure(output, input, which_root);
		}
		return -1;
	}
	auto Model::forwardKinematics(const double* input, double* output, int which_root)const noexcept->int {
		if (auto c_fwd = dynamic_cast<const aris::dynamic::ForwardKinematicSolver*>(&solverPool()[1])) {
			auto fwd = const_cast<aris::dynamic::ForwardKinematicSolver*>(c_fwd);
			return fwd->kinPosPure(input, output, which_root);
		}
		return -1;
	}

	auto Model::isSingular(double zero_check)noexcept->bool {
		double U[144], tau[12]; // MAX SUPPORT 12*12
		Size p[12], rank;

		auto& u = dynamic_cast<aris::dynamic::ForwardKinematicSolver&>(this->solverPool()[1]);
		u.cptJacobi();

		s_householder_utp(u.mJf(), u.nJf(), u.Jf(), U, tau, p, rank, zero_check);

		aris::dynamic::dsp(u.mJf(), u.nJf(), u.Jf());

		return rank < u.nJf();
	}

	auto Model::eeTypes()const noexcept->const EEType* {
		return imp_->ee_types_.data();
	}
	auto Model::eeSize()const noexcept->aris::Size {
		return imp_->ee_types_.size();
	}
	
	auto Model::inputPosSize()const noexcept->Size { return motionPool().size(); }
	auto Model::inputVelSize()const noexcept->Size { return motionPool().size(); }
	auto Model::inputAccSize()const noexcept->Size { return motionPool().size(); }
	auto Model::inputFceSize()const noexcept->Size { return motionPool().size(); }

	auto Model::inputPosAt(Size idx)const noexcept->double {
		return this->motionPool()[idx].mp();
	}
	auto Model::setInputPosAt(double mp, Size idx)noexcept->void {
		this->motionPool()[idx].setP(&mp);
	}
	auto Model::inputVelAt(Size idx)const noexcept->double {
		return this->motionPool()[idx].mv();
	}
	auto Model::setInputVelAt(double mv, Size idx)noexcept->void {
		this->motionPool()[idx].setV(&mv);
	}
	auto Model::inputAccAt(Size idx)const noexcept->double {
		return this->motionPool()[idx].ma();
	}
	auto Model::setInputAccAt(double ma, Size idx)noexcept->void {
		this->motionPool()[idx].setA(&ma);
	}
	auto Model::inputFceAt(Size idx)const noexcept->double {
		return this->motionPool()[idx].mf();
	}
	auto Model::setInputFceAt(double mf, Size idx)noexcept->void {
		this->motionPool()[idx].setF(&mf);
	}

	auto Model::outputPosSize()const noexcept->Size { return imp_->end_effector_pos_size_;}
	auto Model::outputVelSize()const noexcept->Size { return imp_->end_effector_vel_size_; }
	auto Model::outputAccSize()const noexcept->Size { return imp_->end_effector_acc_size_; }
	auto Model::outputFceSize()const noexcept->Size { return imp_->end_effector_fce_size_; }
	auto Model::setOutputPos(const double *mp)noexcept->void { for (Size i = 0, pos = 0; i < generalMotionPool().size(); pos += generalMotionPool()[i].pSize(), ++i) generalMotionPool()[i].setP(mp + pos);	}
	auto Model::getOutputPos(double *mp)const noexcept->void { for (Size i = 0, pos = 0; i < generalMotionPool().size(); pos += generalMotionPool()[i].pSize(), ++i) generalMotionPool()[i].getP(mp + pos); }
	auto Model::setOutputVel(const double *mv)noexcept->void { for (Size i = 0, pos = 0; i < generalMotionPool().size(); pos += generalMotionPool()[i].vSize(), ++i) generalMotionPool()[i].setV(mv + pos); }
	auto Model::getOutputVel(double *mv)const noexcept->void { for (Size i = 0, pos = 0; i < generalMotionPool().size(); pos += generalMotionPool()[i].vSize(), ++i) generalMotionPool()[i].getV(mv + pos); }
	auto Model::setOutputAcc(const double *ma)noexcept->void { for (Size i = 0, pos = 0; i < generalMotionPool().size(); pos += generalMotionPool()[i].aSize(), ++i) generalMotionPool()[i].setA(ma + pos); }
	auto Model::getOutputAcc(double *ma)const noexcept->void { for (Size i = 0, pos = 0; i < generalMotionPool().size(); pos += generalMotionPool()[i].aSize(), ++i) generalMotionPool()[i].getA(ma + pos); }
	auto Model::setOutputFce(const double *mf)noexcept->void { for (Size i = 0, pos = 0; i < generalMotionPool().size(); pos += generalMotionPool()[i].fSize(), ++i) generalMotionPool()[i].setF(mf + pos); }
	auto Model::getOutputFce(double *mf)const noexcept->void { for (Size i = 0, pos = 0; i < generalMotionPool().size(); pos += generalMotionPool()[i].fSize(), ++i) generalMotionPool()[i].getF(mf + pos); }
	
	auto Model::findVariable(std::string_view name)->Variable* {
		auto found = std::find_if(variablePool().begin(), variablePool().end(), [name](const auto& variable)->auto {
			return variable.name() == name;
			});
		return found == variablePool().end() ? nullptr : &*found;
	}
	auto Model::findPart(std::string_view name)->Part* {
		auto found = std::find_if(partPool().begin(), partPool().end(), [name](const auto& variable)->auto {
			return variable.name() == name;
			});
		return found == partPool().end() ? nullptr : &*found;
	}
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
	auto Model::resetGeneralMotionPool(aris::core::PointerArray<MotionBase, Element> *pool)->void { imp_->general_motion_pool_.reset(pool); }
	auto Model::generalMotionPool()->aris::core::PointerArray<MotionBase, Element>& { return *imp_->general_motion_pool_; }
	auto Model::resetForcePool(aris::core::PointerArray<Force, Element> *pool)->void { imp_->force_pool_.reset(pool); }
	auto Model::forcePool()->aris::core::PointerArray<Force, Element>& { return *imp_->force_pool_; }
	auto Model::resetSolverPool(aris::core::PointerArray<Solver, Element> *pool)->void { imp_->solver_pool_.reset(pool); }
	auto Model::solverPool()->aris::core::PointerArray<Solver, Element>& { return *imp_->solver_pool_; }
	auto Model::simulatorPool()->aris::core::PointerArray<Simulator, Element>& { return *imp_->simulator_pool_; }
	auto Model::simResultPool()->aris::core::PointerArray<SimResult, Element>& { return *imp_->sim_result_pool_; }
	auto Model::resetCalibratorPool(aris::core::PointerArray<Calibrator, Element> *pool)->void { imp_->calibrator_pool_.reset(pool); }
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
	auto Model::addScrewJoint(Part& first_part, Part& second_part, const double* position, const double* axis, double pitch)->ScrewJoint& {
		double glb_pm[16], loc_pm[16];
		s_sov_axes2pm(position, axis, axis, glb_pm, "zx");
		auto name = "joint_" + std::to_string(jointPool().size());
		s_inv_pm_dot_pm(*first_part.pm(), glb_pm, loc_pm);
		auto& mak_i = first_part.addMarker(name + "_i", loc_pm);
		s_inv_pm_dot_pm(*second_part.pm(), glb_pm, loc_pm);
		auto& mak_j = second_part.addMarker(name + "_j", loc_pm);

		auto& ret = jointPool().add<ScrewJoint>(name, &mak_i, &mak_j, pitch);
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


		s_swap_v(3, &glb_pm[0], 4, &glb_pm[2], 4);
		s_iv(3, glb_pm, 4);


		//s_sov_axes2pm(position, second_axis, first_axis, glb_pm, "zx");
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
		double pitch{ 0.0 };

		if (dynamic_cast<RevoluteJoint*>(&joint)){
			dim = 5;
		}
		else if (dynamic_cast<ScrewJoint*>(&joint)) {
			dim = 5;
			pitch = dynamic_cast<ScrewJoint*>(&joint)->pitch();
		}
		else if (dynamic_cast<PrismaticJoint*>(&joint)){
			dim = 2;
		}else{
			THROW_FILE_LINE("wrong joint when Model::addMotion(joint)");
		}

		auto &ret = motionPool().add<Motion>("motion_" + std::to_string(motionPool().size()), joint.makI(), joint.makJ(), dim);
		ret.setPitch(pitch);
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
	auto Model::addPointMotion(Part &end_effector, Part &reference, const double* pos_in_ground)->PointMotion& {
		double pm_i[16], pm_j[16], pp_i[3], pp_j[3];

		
		s_eye(4, pm_j);
		aris::dynamic::s_inv_pp2pp(*reference.pm(), pos_in_ground, pp_j);
		s_vc(3, pp_j, 1, pm_j + 3, 4);

		s_eye(4, pm_i);
		aris::dynamic::s_inv_pp2pp(*end_effector.pm(), pos_in_ground, pp_i);
		s_vc(3, pp_i, 1, pm_i + 3, 4);

		auto name = "point_motion_" + std::to_string(generalMotionPool().size());
		auto &mak_i = dynamic_cast<Part&>(end_effector).addMarker(name + "_i", pm_i);
		auto &mak_j = dynamic_cast<Part&>(reference).addMarker(name + "_j", pm_j);

		auto &ret = generalMotionPool().add<PointMotion>(name, &mak_i, &mak_j);
		ret.Element::model_ = this;
		return ret;
	}
	Model::~Model() = default;
	Model::Model(){
		imp_->variable_pool_.reset(new aris::core::PointerArray<Variable, Element>);
		imp_->part_pool_.reset(new aris::core::PointerArray<Part, Element>);
		imp_->joint_pool_.reset(new aris::core::PointerArray<Joint, Element>);
		imp_->motion_pool_.reset(new aris::core::PointerArray<Motion, Element>);
		imp_->general_motion_pool_.reset(new aris::core::PointerArray<MotionBase, Element>);
		imp_->force_pool_.reset(new aris::core::PointerArray<Force, Element>);
		imp_->solver_pool_.reset(new aris::core::PointerArray<Solver, Element>);
		imp_->simulator_pool_.reset(new aris::core::PointerArray<Simulator, Element>);
		imp_->calibrator_pool_.reset(new aris::core::PointerArray<Calibrator, Element>);
		imp_->sim_result_pool_.reset(new aris::core::PointerArray<SimResult, Element>);

		imp_->ground_ = &partPool().add<Part>("ground");
	}
	Model::Model(Model&&)noexcept = default;
	Model& Model::operator=(Model&&)noexcept = default;

	auto trimLR(std::string_view input)->std::string {
		std::string ret(input);
		ret.erase(0, ret.find_first_not_of(" \t\n\r\f\v"));// trim l
		ret.erase(ret.find_last_not_of(" \t\n\r\f\v") + 1);// trim r
		return ret;
	}

	struct MultiModel::Imp {
		std::unique_ptr<aris::core::PointerArray<ModelBase>> models_;
		std::vector<aris::dynamic::Marker*> tools_, wobjs_;
		std::vector<EEType> ee_types_;
	};

	auto MultiModel::init()->void {
		for (auto& m : subModels()) {
			// init sub model //
			m.init();

			// init ee_types //
			auto begin_idx = imp_->ee_types_.size();
			imp_->ee_types_.resize(imp_->ee_types_.size() + m.eeSize());
			std::copy_n(m.eeTypes(), m.eeSize(), imp_->ee_types_.begin() + begin_idx);
		}
	}

	auto MultiModel::isSingular(double zero_check)noexcept->bool {
		for (auto& m : this->subModels()) {
			if (m.isSingular(zero_check))
				return true;
		}

		return false;
	}

	auto MultiModel::eeTypes()const noexcept->const EEType* {
		return imp_->ee_types_.data();
	}
	auto MultiModel::eeSize()const noexcept->aris::Size {
		return imp_->ee_types_.size();
	}

	auto MultiModel::inputPosSize()const noexcept->aris::Size {
		aris::Size size = 0;
		for (auto& m : subModels())size += m.inputPosSize();
		return size;
	}
	auto MultiModel::getInputPos(double* mp)const noexcept->void {
		for (aris::Size k = 0, i = 0; i < subModels().size(); k += subModels()[i].inputPosSize(), ++i)
			subModels()[i].getInputPos(mp + k);
	}
	auto MultiModel::setInputPos(const double* mp)noexcept->void {
		for (aris::Size k = 0, i = 0; i < subModels().size(); k += subModels()[i].inputPosSize(), ++i)
			subModels()[i].setInputPos(mp + k);
	}
	auto MultiModel::inputPosAt(Size idx)const noexcept->double {
		for (aris::Size k = 0, i = 0; i < subModels().size(); k += subModels()[i].inputPosSize(), ++i)
			if (k + subModels()[i].inputPosSize() > idx) {
				return subModels()[i].inputPosAt(idx - k);
			};
		return 0.0;
	}
	auto MultiModel::setInputPosAt(double VARIABLE, Size idx)noexcept->void {
		for (aris::Size k = 0, i = 0; i < subModels().size(); k += subModels()[i].inputPosSize(), ++i)
			if (k + subModels()[i].inputPosSize() > idx) {
				return subModels()[i].setInputPosAt(VARIABLE, idx - k);
			};
	}

	auto MultiModel::inputVelSize()const noexcept->aris::Size {
		aris::Size size = 0;
		for (auto& m : subModels())size += m.inputVelSize();
		return size;
	}
	auto MultiModel::getInputVel(double* mp)const noexcept->void {
		for (aris::Size k = 0, i = 0; i < subModels().size(); k += subModels()[i].inputVelSize(), ++i)
			subModels()[i].getInputVel(mp + k);
	}
	auto MultiModel::setInputVel(const double* mp)noexcept->void {
		for (aris::Size k = 0, i = 0; i < subModels().size(); k += subModels()[i].inputVelSize(), ++i)
			subModels()[i].setInputVel(mp + k);
	}
	auto MultiModel::inputVelAt(Size idx)const noexcept->double {
		for (aris::Size k = 0, i = 0; i < subModels().size(); k += subModels()[i].inputVelSize(), ++i)
			if (k + subModels()[i].inputVelSize() > idx) {

				return subModels()[i].inputVelAt(idx - k);
			};
		return 0.0;
	}
	auto MultiModel::setInputVelAt(double VARIABLE, Size idx)noexcept->void {
		for (aris::Size k = 0, i = 0; i < subModels().size(); k += subModels()[i].inputVelSize(), ++i)
			if (k + subModels()[i].inputVelSize() > idx) {
				return subModels()[i].setInputVelAt(VARIABLE, idx - k);
			};
	}

	auto MultiModel::inputAccSize()const noexcept->aris::Size {
		aris::Size size = 0;
		for (auto& m : subModels())size += m.inputAccSize();
		return size;
	}
	auto MultiModel::getInputAcc(double* mp)const noexcept->void {
		for (aris::Size k = 0, i = 0; i < subModels().size(); k += subModels()[i].inputAccSize(), ++i)
			subModels()[i].getInputAcc(mp + k);
	}
	auto MultiModel::setInputAcc(const double* mp)noexcept->void {
		for (aris::Size k = 0, i = 0; i < subModels().size(); k += subModels()[i].inputAccSize(), ++i)
			subModels()[i].setInputAcc(mp + k);
	}
	auto MultiModel::inputAccAt(Size idx)const noexcept->double {
		for (aris::Size k = 0, i = 0; i < subModels().size(); k += subModels()[i].inputAccSize(), ++i)
			if (k + subModels()[i].inputAccSize() > idx) {

				return subModels()[i].inputAccAt(idx - k);
			};
		return 0.0;
	}
	auto MultiModel::setInputAccAt(double VARIABLE, Size idx)noexcept->void {
		for (aris::Size k = 0, i = 0; i < subModels().size(); k += subModels()[i].inputAccSize(), ++i)
			if (k + subModels()[i].inputAccSize() > idx) {

				return subModels()[i].setInputAccAt(VARIABLE, idx - k);
			};
	}

	auto MultiModel::inputFceSize()const noexcept->aris::Size {
		aris::Size size = 0;
		for (auto& m : subModels())size += m.inputFceSize();
		return size;
	}
	auto MultiModel::getInputFce(double* mp)const noexcept->void {
		for (aris::Size k = 0, i = 0; i < subModels().size(); k += subModels()[i].inputFceSize(), ++i)
			subModels()[i].getInputFce(mp + k);
	}
	auto MultiModel::setInputFce(const double* mp)noexcept->void {
		for (aris::Size k = 0, i = 0; i < subModels().size(); k += subModels()[i].inputFceSize(), ++i)
			subModels()[i].setInputFce(mp + k);
	}
	auto MultiModel::inputFceAt(Size idx)const noexcept->double {

		for (aris::Size k = 0, i = 0; i < subModels().size(); k += subModels()[i].inputFceSize(), ++i)
			if (k + subModels()[i].inputFceSize() > idx) {

				return subModels()[i].inputFceAt(idx - k);
			};
		return 0.0;
	}
	auto MultiModel::setInputFceAt(double VARIABLE, Size idx)noexcept->void {
		for (aris::Size k = 0, i = 0; i < subModels().size(); k += subModels()[i].inputFceSize(), ++i)
			if (k + subModels()[i].inputFceSize() > idx) {

				return subModels()[i].setInputFceAt(VARIABLE, idx - k);
			};
	}

	auto MultiModel::outputPosSize()const noexcept->aris::Size {
		aris::Size size = 0;
		for (auto& m : subModels())size += m.outputPosSize();
		return size;
	}
	auto MultiModel::getOutputPos(double* mp)const noexcept->void {
		for (aris::Size pos = 0, idx = 0; idx < subModels().size(); pos += subModels()[idx].outputPosSize(), ++idx)
			subModels()[idx].getOutputPos(mp + pos);
	}
	auto MultiModel::setOutputPos(const double* mp)noexcept->void {
		for (aris::Size pos = 0, idx = 0; idx < subModels().size(); pos += subModels()[idx].outputPosSize(), ++idx)
			subModels()[idx].setOutputPos(mp + pos);
	}

	auto MultiModel::outputVelSize()const noexcept->aris::Size {
		aris::Size size = 0;
		for (auto& m : subModels())size += m.outputVelSize();
		return size;
	}
	auto MultiModel::getOutputVel(double* mv)const noexcept->void {
		for (aris::Size pos = 0, idx = 0; idx < subModels().size(); pos += subModels()[idx].outputVelSize(), ++idx)
			subModels()[idx].getOutputVel(mv + pos);
	}
	auto MultiModel::setOutputVel(const double* mv)noexcept->void {
		for (aris::Size pos = 0, idx = 0; idx < subModels().size(); pos += subModels()[idx].outputVelSize(), ++idx)
			subModels()[idx].setOutputVel(mv + pos);
	}

	auto MultiModel::outputAccSize()const noexcept->aris::Size {
		aris::Size size = 0;
		for (auto& m : subModels())size += m.outputAccSize();
		return size;
	}
	auto MultiModel::getOutputAcc(double* ma)const noexcept->void {
		for (aris::Size pos = 0, idx = 0; idx < subModels().size(); pos += subModels()[idx].outputAccSize(), ++idx)
			subModels()[idx].getOutputAcc(ma + pos);
	}
	auto MultiModel::setOutputAcc(const double* ma)noexcept->void {
		for (aris::Size pos = 0, idx = 0; idx < subModels().size(); pos += subModels()[idx].outputAccSize(), ++idx)
			subModels()[idx].setOutputAcc(ma + pos);
	}

	auto MultiModel::outputFceSize()const noexcept->aris::Size {
		aris::Size size = 0;
		for (auto& m : subModels())size += m.outputFceSize();
		return size;
	}
	auto MultiModel::getOutputFce(double* mf)const noexcept->void {
		for (aris::Size pos = 0, idx = 0; idx < subModels().size(); pos += subModels()[idx].outputFceSize(), ++idx)
			subModels()[idx].getOutputFce(mf + pos);
	}
	auto MultiModel::setOutputFce(const double* mf)noexcept->void {
		for (aris::Size pos = 0, idx = 0; idx < subModels().size(); pos += subModels()[idx].outputFceSize(), ++idx)
			subModels()[idx].setOutputFce(mf + pos);
	}

	auto MultiModel::resetSubModelPool(aris::core::PointerArray<ModelBase>* pool)->void {
		imp_->models_.reset(pool);
	}
	auto MultiModel::subModels()->aris::core::PointerArray<ModelBase>& {
		return *imp_->models_;
	}
	auto MultiModel::tools()->std::vector<aris::dynamic::Marker*>& { return imp_->tools_; }
	auto MultiModel::wobjs()->std::vector<aris::dynamic::Marker*>& { return imp_->wobjs_; }
	auto MultiModel::findTool(std::string_view name)->aris::dynamic::Marker* {
		auto found_marker = std::find_if(tools().begin(), tools().end(), [name](const auto& variable)->auto{
			return trimLR(variable->name()) == trimLR(name);
		});

		return (found_marker == tools().end()) ? nullptr : *found_marker;
	}
	auto MultiModel::findWobj(std::string_view name)->aris::dynamic::Marker* {
		auto found_marker = std::find_if(wobjs().begin(), wobjs().end(), [name](const auto& variable)->auto{
			return trimLR(variable->name()) == trimLR(name);
		});

		return (found_marker == wobjs().end()) ? nullptr : *found_marker;
	}
	auto MultiModel::findMarker(std::string_view name)->aris::dynamic::Marker* {
		auto model_name = name.substr(0, name.find_first_of('.'));
		name = name.substr(name.find_first_of('.') + 1);
		auto part_name = name.substr(0, name.find_first_of('.'));
		name = name.substr(name.find_first_of('.') + 1);
		auto marker_name = name.substr(0, name.find_first_of('.'));

		auto found_model = std::find_if(subModels().begin(), subModels().end(), [model_name](const auto& variable)->auto{
			return trimLR(variable.name()) == trimLR(model_name);
		});

		if (found_model == subModels().end() || !dynamic_cast<aris::dynamic::Model*>(&*found_model)) return nullptr;
		
		auto model = dynamic_cast<aris::dynamic::Model*>(&*found_model);
		auto found_part = std::find_if(model->partPool().begin(), model->partPool().end(), [part_name](const auto& variable)->auto{
			return trimLR(variable.name()) == trimLR(part_name);
		});

		if (found_part == model->partPool().end()) return nullptr;

		auto found_marker = std::find_if(found_part->markerPool().begin(), found_part->markerPool().end(), [marker_name](const auto& variable)->auto{
			return trimLR(variable.name()) == trimLR(marker_name);
		});
		
		if (found_marker == found_part->markerPool().end()) return nullptr;

		return &*found_marker;
	}
	auto MultiModel::findVariable(std::string_view name)->aris::dynamic::Variable* {
		auto model_name = name.substr(0, name.find_first_of('.'));
		name = name.substr(name.find_first_of('.') + 1);
		auto variable_name = name.substr(0, name.find_first_of('.'));

		auto found_model = std::find_if(subModels().begin(), subModels().end(), [model_name](const auto& variable)->auto{
			return trimLR(variable.name()) == trimLR(model_name);
		});

		if (found_model == subModels().end() || !dynamic_cast<aris::dynamic::Model*>(&*found_model)) return nullptr;

		auto model = dynamic_cast<aris::dynamic::Model*>(&*found_model);
		auto found_variable = std::find_if(model->variablePool().begin(), model->variablePool().end(), [variable_name](const auto& variable)->auto{
			return trimLR(variable.name()) == trimLR(variable_name);
		});

		if (found_variable == model->variablePool().end()) return nullptr;

		return &*found_variable;
	}

	auto MultiModel::getEeTypes()->std::vector<EEType> {
		std::vector<Size> model_ids(subModels().size());
		std::iota(model_ids.begin(), model_ids.end(), 0);
		return getEeTypes(model_ids);
	}
	auto MultiModel::getEes()->std::vector<MotionBase*> {
		std::vector<Size> model_ids(subModels().size());
		std::iota(model_ids.begin(), model_ids.end(), 0);
		return getEes(model_ids);
	}
	auto MultiModel::getMotionIds()->std::vector<Size> {
		std::vector<Size> motions;

		Size id = 0;
		for (auto& m : subModels()) 
			for (int i = 0; i < m.inputPosSize(); ++i) 
				motions.push_back(id++);

		return motions;
	}
	auto MultiModel::getEeNumOfSubModels(const std::vector<Size>& submodel_ids)->std::vector<Size> {
		std::vector<Size> ee_nums;

		for (auto id : submodel_ids) {
			auto& m = subModels()[id];

			if (auto model = dynamic_cast<aris::dynamic::Model*>(&m)) {
				ee_nums.push_back(model->generalMotionPool().size());
			}
			else {
				ee_nums.push_back(0);
			}
		}

		return ee_nums;
	
	}
	auto MultiModel::getEeTypes(const std::vector<Size>& submodel_ids)->std::vector<EEType> {
		std::vector<EEType> ee_types;

		for (auto id:submodel_ids) {
			auto& m = subModels()[id];
			
			if (auto model = dynamic_cast<aris::dynamic::Model*>(&m)) {
				for (auto& gm : model->generalMotionPool()) {
					ee_types.push_back(gm.eeType());
				}
			}
			else {
				ee_types.push_back(aris::dynamic::EEType::UNKNOWN);
			}
		}
		return ee_types;
	}
	auto MultiModel::getEes(const std::vector<Size>& submodel_ids)->std::vector<MotionBase*> {
		std::vector<aris::dynamic::MotionBase*> ees;

		for (auto id : submodel_ids) {
			auto& m = subModels()[id];

			if (auto model = dynamic_cast<aris::dynamic::Model*>(&m)) {
				for (auto& gm : model->generalMotionPool()) {
					ees.push_back(&gm);
				}
			}
		}

		return ees;
	}
	auto MultiModel::getMotionNumOfSubModels(const std::vector<Size>& submodel_ids)->std::vector<Size> {
		std::vector<Size> mot_nums;

		for (auto id : submodel_ids) {
			auto& m = subModels()[id];
			mot_nums.push_back(m.inputPosSize());
		}

		return mot_nums;
	}
	// 获取对应子模型中的电机种类
	auto MultiModel::getMotionTypes(const std::vector<Size>& submodel_ids)->std::vector<EEType> {
		std::vector<EEType> mot_types;

		for (auto id : submodel_ids) {
			auto& m = subModels()[id];

			if (auto model = dynamic_cast<aris::dynamic::Model*>(&m)) {
				for (auto& mot : model->motionPool()) {
					switch (mot.axis()) {
					case 0:
					case 1:
					case 2:
						mot_types.push_back(aris::dynamic::EEType::X);
						break;
					case 3:
					case 4:
					case 5:
						mot_types.push_back(aris::dynamic::EEType::A);
						break;
					default:
						mot_types.push_back(aris::dynamic::EEType::UNKNOWN);
					}
				}
			}
			else {
				mot_types.push_back(aris::dynamic::EEType::UNKNOWN);
			}
		}
		return mot_types;
	}
	auto MultiModel::getMotionTypes()->std::vector<EEType> {
		std::vector<Size> model_ids(subModels().size());
		std::iota(model_ids.begin(), model_ids.end(), 0);
		return getMotionTypes(model_ids);
	}
	// 获取对应子模型中的电机
	auto MultiModel::getMotions(const std::vector<Size>& submodel_ids)->std::vector<Motion*> {
		std::vector<aris::dynamic::Motion*> mots;

		for (auto id : submodel_ids) {
			auto& m = subModels()[id];

			if (auto model = dynamic_cast<aris::dynamic::Model*>(&m)) {
				for (auto& gm : model->motionPool()) {
					mots.push_back(&gm);
				}
			}
		}

		return mots;
	}
	auto MultiModel::getMotionIds(const std::vector<Size>& submodel_ids)->std::vector<Size> {
		std::vector<Size> motion_ids;

		std::vector<Size> model_motion_poses{ 0 };
		for (auto& model : subModels()) {
			model_motion_poses.push_back(model.inputPosSize() + model_motion_poses.back());
		}

		for (auto id : submodel_ids) {
			auto& m = subModels()[id];
			std::vector<Size> ids(m.inputPosSize());
			std::iota(ids.begin(), ids.end(), model_motion_poses[id]);
			motion_ids.insert(motion_ids.end(), ids.begin(), ids.end());
		}

		return motion_ids;
	}
	MultiModel::~MultiModel() = default;
	MultiModel::MultiModel() {
		imp_->models_.reset(new aris::core::PointerArray<ModelBase>);
	}
	MultiModel::MultiModel(MultiModel&&) = default;
	MultiModel& MultiModel::operator=(MultiModel&&) = default;

	ARIS_REGISTRATION{
		aris::core::class_<ModelBase>("ModelBase")
			.prop("name", &ModelBase::setName, &ModelBase::name)
			;

		typedef Environment&(Model::*EnvironmentFunc)();
		typedef aris::core::PointerArray<Variable,          Element> &(Model::*VarablePoolFunc)();
		typedef aris::core::PointerArray<Part,              Element> &(Model::*PartPoolFunc)();
		typedef aris::core::PointerArray<Joint,             Element> &(Model::*JointPoolFunc)();
		typedef aris::core::PointerArray<Motion,            Element> &(Model::*MotionPoolFunc)();
		typedef aris::core::PointerArray<MotionBase,        Element> &(Model::*GeneralMotionPoolFunc)();
		typedef aris::core::PointerArray<Force,             Element> &(Model::*ForcePoolFunc)();
		typedef aris::core::PointerArray<Solver,            Element> &(Model::*SolverPoolFunc)();
		typedef aris::core::PointerArray<Simulator,         Element> &(Model::*SimulatorPoolFunc)();
		typedef aris::core::PointerArray<SimResult,         Element> &(Model::*SimResultPoolFunc)();
		typedef aris::core::PointerArray<Calibrator,        Element> &(Model::*CalibratorPoolFunc)();

		auto variable_size = [](aris::core::PointerArray<Variable, Element>* pool)->aris::Size {
			return pool->size();
		};
		auto variable_at = [](aris::core::PointerArray<Variable, Element>* pool, aris::Size i)->Variable& {
			return pool->at(i);
		};
		auto variable_pushback = [](aris::core::PointerArray<Variable, Element>* pool, Variable *value)->void {
			return pool->push_back(value);
		};
		auto variable_clear = [](aris::core::PointerArray<Variable, Element>* pool)->void {
			return pool->clear();
		};

		auto joint_size = [](aris::core::PointerArray<Joint, Element>* pool)->aris::Size {
			return pool->size();
		};
		auto joint_at = [](aris::core::PointerArray<Joint, Element>* pool, aris::Size i)->Joint& {
			return pool->at(i);
		};
		auto joint_pushback = [](aris::core::PointerArray<Joint, Element>* pool, Joint* value)->void {
			return pool->push_back(value);
		};
		auto joint_clear = [](aris::core::PointerArray<Joint, Element>* pool)->void {
			return pool->clear();
		};

		aris::core::class_<aris::core::PointerArray<Variable, Element>>("VariablePoolElement")
			.asRefArray(&variable_size, &variable_at, &variable_pushback, &variable_clear)
			;
		aris::core::class_<aris::core::PointerArray<Part, Element>>("PartPoolElement")
			.asRefArray()
			;
		aris::core::class_<aris::core::PointerArray<Joint, Element>>("JointPoolElement")
			.asRefArray(&joint_size, &joint_at, &joint_pushback, &joint_clear)
			;
		aris::core::class_<aris::core::PointerArray<Motion, Element>>("MotionPoolElement")
			.asRefArray()
			;
		aris::core::class_<aris::core::PointerArray<MotionBase, Element>>("GeneralMotionPoolElement")
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



		auto getVariablePool = [](aris::dynamic::Model* m)->aris::core::PointerArray<Variable, Element>&{
			return m->variablePool();
		};
		auto setVariablePool = [](aris::dynamic::Model* m, aris::core::PointerArray<Variable, Element>* pool)->void {
			m->resetVariablePool(pool);
			pool->resetModel(m);
		};
		auto getPartPool = [](aris::dynamic::Model* m)->aris::core::PointerArray<Part, Element>&{
			return m->partPool();
		};
		auto setPartPool = [](aris::dynamic::Model* m, aris::core::PointerArray<Part, Element>* pool)->void {
			m->resetPartPool(pool);
			pool->resetModel(m);
		};
		auto getMotionPool = [](aris::dynamic::Model* m)->aris::core::PointerArray<Motion, Element>&{
			return m->motionPool();
		};
		auto setMotionPool = [](aris::dynamic::Model* m, aris::core::PointerArray<Motion, Element>* pool)->void {
			m->resetMotionPool(pool);
			pool->resetModel(m);
		};
		auto getJointPool = [](aris::dynamic::Model* m)->aris::core::PointerArray<Joint, Element>&{
			return m->jointPool();
		};
		auto setJointPool = [](aris::dynamic::Model* m, aris::core::PointerArray<Joint, Element>* pool)->void {
			m->resetJointPool(pool);
			pool->resetModel(m);
		};
		auto getGeneralMotionPool = [](aris::dynamic::Model* m)->aris::core::PointerArray<MotionBase, Element>&{
			return m->generalMotionPool();
		};
		auto setGeneralMotionPool = [](aris::dynamic::Model* m, aris::core::PointerArray<MotionBase, Element>* pool)->void {
			m->resetGeneralMotionPool(pool);
			pool->resetModel(m);
		};
		auto getForcePool = [](aris::dynamic::Model* m)->aris::core::PointerArray<Force, Element>&{
			return m->forcePool();
		};
		auto setForcePool = [](aris::dynamic::Model* m, aris::core::PointerArray<Force, Element>* pool)->void {
			m->resetForcePool(pool);
			pool->resetModel(m);
		};
		auto getSolverPool = [](aris::dynamic::Model* m)->aris::core::PointerArray<Solver, Element>&{
			return m->solverPool();
		};
		auto setSolverPool = [](aris::dynamic::Model* m, aris::core::PointerArray<Solver, Element>* pool)->void {
			m->resetSolverPool(pool);
			pool->resetModel(m);
		};
		auto getCalibratorPool = [](aris::dynamic::Model* m)->aris::core::PointerArray<Calibrator, Element>&{
			return m->calibratorPool();
		};
		auto setCalibratorPool = [](aris::dynamic::Model* m, aris::core::PointerArray<Calibrator, Element>* pool)->void {
			m->resetCalibratorPool(pool);
			pool->resetModel(m);
		};

		aris::core::class_<Model>("Model")
			.inherit<ModelBase>()
			.prop("time", &Model::setTime, &Model::time)
			.prop("environment", EnvironmentFunc(&Model::environment))
			.prop("variable_pool", &setVariablePool, &getVariablePool)
			.prop("part_pool", &setPartPool, &getPartPool)
			.prop("motion_pool", &setMotionPool, &getMotionPool)
			.prop("joint_pool", &setJointPool, &getJointPool)
			.prop("general_motion_pool", &setGeneralMotionPool, &getGeneralMotionPool)
			.prop("force_pool", &setForcePool, &getForcePool)
			.prop("solver_pool", &setSolverPool, &getSolverPool)
			.prop("calibrator_pool", &setCalibratorPool, &getCalibratorPool)
			//.prop("variable_pool", &Model::resetVariablePool, VarablePoolFunc(&Model::variablePool))
			//.prop("part_pool", &Model::resetPartPool,  PartPoolFunc(&Model::partPool))
			//.prop("motion_pool", &Model::resetMotionPool, MotionPoolFunc(&Model::motionPool))
			//.prop("joint_pool", &Model::resetJointPool, JointPoolFunc(&Model::jointPool))
			//.prop("general_motion_pool", &Model::resetGeneralMotionPool, GeneralMotionPoolFunc(&Model::generalMotionPool))
			//.prop("force_pool", &Model::resetForcePool, ForcePoolFunc(&Model::forcePool))
			//.prop("solver_pool", &Model::resetSolverPool, SolverPoolFunc(&Model::solverPool))
			//.prop("calibrator_pool", &Model::resetCalibratorPool, CalibratorPoolFunc(&Model::calibratorPool))
			;

		aris::core::class_<aris::core::PointerArray<ModelBase>>("ModelBasePool")
			.asRefArray()
			;


		

		struct LocalStringList {
			std::vector<std::string> strs;
		};

		aris::core::class_<LocalStringList>("LocalStringList" + std::string(__FILE__) + std::to_string(__LINE__))
			.textMethod([](LocalStringList*list)->std::string {
					std::string ret = "{";
					for (auto i =0;i< list->strs.size();++i)
						ret += i==(list->strs.size()-1) ? list->strs[i] : (list->strs[i] + ",");
					ret += "}";
					return ret;
				}, [](LocalStringList*list, std::string_view str)->void {
					str = str.substr(1);
					
					while (str.find_first_of(',') != std::string_view::npos) {
						list->strs.push_back(std::string(str.substr(0, str.find_first_of(','))));
						str = str.substr(str.find_first_of(',') + 1);
					}
					if (auto last_name = std::string(str.substr(0, str.size() - 1)); last_name.size() > 1)
						list->strs.push_back(last_name);
				})
			;

		auto getTools = [](MultiModel* m)->LocalStringList {
			LocalStringList name_list;
			for(auto s:m->tools())name_list.strs.push_back(s->model()->name() + "." + s->fatherPart().name() + "." + s->name());
			return name_list; 
		};
		auto setTools = [](MultiModel* m, LocalStringList name_list)->void {
			m->tools().clear();
			for (auto name : name_list.strs)
				if (auto tool = m->findMarker(name))
					m->tools().push_back(tool);
				else
					THROW_FILE_LINE("tool 【" + name + "】 not found");
		};
		auto getWobjs = [](MultiModel* m)->LocalStringList {
			LocalStringList name_list;
			for (auto s : m->wobjs())name_list.strs.push_back(s->model()->name() + "." + s->fatherPart().name() + "." + s->name());
			return name_list;
		};
		auto setWobjs = [](MultiModel* m, LocalStringList name_list)->void {
			m->wobjs().clear();
			for (auto name : name_list.strs)
				if (auto wobj = m->findMarker(name))
					m->wobjs().push_back(wobj);
				else
					THROW_FILE_LINE("wobj 【" + name + "】 not found");
		};

		typedef aris::core::PointerArray<ModelBase>& (MultiModel::* ModelBasePoolFunc)();
		aris::core::class_<MultiModel>("MultiModel")
			.inherit<ModelBase>()
			.prop("submodel_pool", &MultiModel::resetSubModelPool, ModelBasePoolFunc(&MultiModel::subModels))
			.prop("tools", &setTools, &getTools)
			.prop("wobjs", &setWobjs, &getWobjs)
			;
	}
}
