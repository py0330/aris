#include "aris_dynamic_simple_model.h"


namespace aris
{
	namespace dynamic
	{
		struct SimpleModel::Imp
		{ 
			Model m_;
			DiagSolver *inv_solver_;
			DiagSolver *fwd_solver_;
		};

		SimpleModel::~SimpleModel() {}
		SimpleModel::SimpleModel() :imp_(new SimpleModel::Imp) 
		{
			imp_->m_.ground().markerPool().add<Marker>("origin"); 
			imp_->inv_solver_ = &model().solverPool().add<DiagSolver>("inv_solver");
			imp_->fwd_solver_ = &model().solverPool().add<DiagSolver>("fwd_solver");
		}
		auto SimpleModel::loadXml(const std::string &file)->void { imp_->m_.loadXml(file); }
		auto SimpleModel::saveXml(const std::string &file)->void { imp_->m_.saveXml(file); }
		auto SimpleModel::model()->Model& { return imp_->m_; }
		auto SimpleModel::ground()->Part& { return imp_->m_.ground(); }
		auto SimpleModel::addPart(const double *pm)->Part* { return &imp_->m_.partPool().add<Part>("part_" + std::to_string(imp_->m_.partPool().size()), nullptr, pm); }
		auto SimpleModel::addRevoluteJoint(Part *first_part, Part *second_part, const double *position, const double *axis)->RevoluteJoint* 
		{ 
			double glb_pm[16], loc_pm[16];
			s_sov_axes2pm(position, axis, axis, glb_pm, "zx");
			auto name = "joint_" + std::to_string(imp_->m_.jointPool().size());
			s_inv_pm_dot_pm(*first_part->glbPm(), glb_pm, loc_pm);
			auto &mak_i = first_part->markerPool().add<Marker>(name + "_i", loc_pm);
			s_inv_pm_dot_pm(*second_part->glbPm(), glb_pm, loc_pm);
			auto &mak_j = second_part->markerPool().add<Marker>(name + "_j", loc_pm);
			return &imp_->m_.jointPool().add<RevoluteJoint>(name, mak_i, mak_j);
		}
		auto SimpleModel::addPrismaticJoint(Part *first_part, Part *second_part, const double *position, const double *axis)->PrismaticJoint*
		{
			double glb_pm[16], loc_pm[16];
			s_sov_axes2pm(position, axis, axis, glb_pm, "zx");
			auto name = "joint_" + std::to_string(imp_->m_.jointPool().size());
			s_inv_pm_dot_pm(*first_part->glbPm(), glb_pm, loc_pm);
			auto &mak_i = first_part->markerPool().add<Marker>(name + "_i", loc_pm);
			s_inv_pm_dot_pm(*second_part->glbPm(), glb_pm, loc_pm);
			auto &mak_j = second_part->markerPool().add<Marker>(name + "_j", loc_pm);
			return &imp_->m_.jointPool().add<PrismaticJoint>(name, mak_i, mak_j);
		}
		auto SimpleModel::addMotion(Joint *joint)->Motion*
		{
			Size dim;
			
			if (dynamic_cast<RevoluteJoint*>(joint))
			{
				dim = 5;
			}
			else if (dynamic_cast<PrismaticJoint*>(joint))
			{
				dim = 2;
			}
			else
			{
				throw std::runtime_error("wrong joint when Simple::addMotion(joint)");
			}
			
			return &imp_->m_.motionPool().add<Motion>("motion_" + std::to_string(imp_->m_.motionPool().size()), joint->makI(), joint->makJ(), dim);
		}
		auto SimpleModel::addEndEffector(Part *end_effector, const double* pm)->GeneralMotion*
		{
			double pm_prt[16];
			s_inv_pm_dot_pm(*end_effector->pm(), pm, pm_prt);
			
			auto name = "general_motion_" + std::to_string(imp_->m_.generalMotionPool().size());
			auto &mak_i = end_effector->markerPool().add<Marker>(name + "_i", pm_prt);
			return &imp_->m_.generalMotionPool().add<GeneralMotion>(name, mak_i, *imp_->m_.ground().markerPool().findByName("origin"));
		}
		auto SimpleModel::allocateMemory()->void
		{
			for (auto &mot : imp_->m_.motionPool())mot.activate(true);
			for (auto &gmt : imp_->m_.generalMotionPool())gmt.activate(false);
			imp_->fwd_solver_->allocateMemory();

			for (auto &mot : imp_->m_.motionPool())mot.activate(false);
			for (auto &gmt : imp_->m_.generalMotionPool())gmt.activate(true);
			imp_->inv_solver_->allocateMemory();
		}
		auto SimpleModel::forwardKinematic(int max_count, double error)->bool
		{
			imp_->m_.saveDynEle("temp");
			
			imp_->fwd_solver_->setMaxIterCount(max_count);
			imp_->fwd_solver_->setMaxError(error);
			imp_->fwd_solver_->kinPos();

			if (imp_->fwd_solver_->iterCount() == imp_->fwd_solver_->maxIterCount())
			{
				imp_->m_.loadDynEle("temp");
				return false;
			}

			return true;
		}
		auto SimpleModel::inverseKinematic(int max_count, double error)->bool
		{
			imp_->m_.saveDynEle("temp");

			imp_->inv_solver_->setMaxIterCount(max_count);
			imp_->inv_solver_->setMaxError(error);
			imp_->inv_solver_->kinPos();

			if (imp_->inv_solver_->iterCount() == imp_->inv_solver_->maxIterCount())
			{
				imp_->m_.loadDynEle("temp");
				return false;
			}

			return true;
		}
	}
}
