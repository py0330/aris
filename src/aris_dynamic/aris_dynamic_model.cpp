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

#include "aris_core.h"
#include "aris_dynamic_matrix.h"
#include "aris_dynamic_screw.h"
#include "aris_dynamic_model.h"

namespace aris
{
	namespace dynamic
	{
		struct Model::Imp
		{
			double time_{0.0};
			aris::core::Calculator calculator_;
			Environment *environment_;
			Part* ground_;
			aris::core::ObjectPool<Variable, Element> *variable_pool_;
			aris::core::ObjectPool<Part, Element> *part_pool_;
			aris::core::ObjectPool<Joint, Element> *joint_pool_;
			aris::core::ObjectPool<Motion, Element> *motion_pool_;
			aris::core::ObjectPool<GeneralMotion, Element> *general_motion_pool_;
			aris::core::ObjectPool<Force, Element> *force_pool_;
			aris::core::ObjectPool<Solver, Element> *solver_pool_;
			aris::core::ObjectPool<Simulator, Element> *simulator_pool_;
			aris::core::ObjectPool<SimResult, Element> *sim_result_pool_;
		};
		auto Model::loadDynEle(const std::string &name)->void
		{
			partPool().load(name);
			jointPool().load(name);
			motionPool().load(name);
			forcePool().load(name);
		}
		auto Model::saveDynEle(const std::string &name)->void
		{
			partPool().save(name);
			jointPool().save(name);
			motionPool().save(name);
			forcePool().save(name);
		}
		auto Model::loadXml(const aris::core::XmlDocument &xml_doc)->void
        {
            auto model_xml_ele = xml_doc.RootElement()->FirstChildElement("model");

            if (!model_xml_ele)throw std::runtime_error("can't find \"model\" element in xml file");

            loadXml(*model_xml_ele);
		}
        auto Model::loadXml(const aris::core::XmlElement &xml_ele)->void
		{
            Root::loadXml(xml_ele);

			setTime(Object::attributeDouble(xml_ele, "time", 0.0));

			imp_->environment_ = findOrInsert<Environment>("environment");
			imp_->variable_pool_ = findOrInsert<aris::core::ObjectPool<Variable, Element>>("variable_pool");
			imp_->part_pool_ = findOrInsert<aris::core::ObjectPool<Part, Element>>("part_pool");
			imp_->joint_pool_ = findOrInsert<aris::core::ObjectPool<Joint, Element>>("joint_pool");
			imp_->motion_pool_ = findOrInsert<aris::core::ObjectPool<Motion, Element>>("motion_pool");
			imp_->general_motion_pool_ = findOrInsert<aris::core::ObjectPool<GeneralMotion, Element>>("general_motion_pool");
			imp_->force_pool_ = findOrInsert<aris::core::ObjectPool<Force, Element>>("force_pool");
			imp_->solver_pool_ = findOrInsert<aris::core::ObjectPool<Solver, Element>>("solver_pool");
			imp_->simulator_pool_ = findOrInsert<aris::core::ObjectPool<Simulator, Element>>("simulator_pool");
			imp_->sim_result_pool_ = findOrInsert<aris::core::ObjectPool<SimResult, Element>>("sim_result_pool");
			imp_->ground_ = partPool().findOrInsert<Part>("ground");
        }
		auto Model::saveXml(aris::core::XmlDocument &xml_doc)const->void
		{
			xml_doc.DeleteChildren();

			auto header_xml_ele = xml_doc.NewDeclaration("xml version=\"1.0\" encoding=\"UTF-8\" ");
			xml_doc.InsertEndChild(header_xml_ele);

			auto root_xml_ele = xml_doc.NewElement("Root");
			xml_doc.InsertEndChild(root_xml_ele);

			auto model_xml_ele = xml_doc.NewElement("Model");
			root_xml_ele->InsertEndChild(model_xml_ele);
			saveXml(*model_xml_ele);
		}
		auto Model::saveXml(aris::core::XmlElement &xml_ele)const->void
		{
			Root::saveXml(xml_ele);
			xml_ele.SetAttribute("time", time());
		}
		auto Model::time()const->double { return imp_->time_; }
		auto Model::setTime(double time)->void { imp_->time_ = time; }
		auto Model::calculator()->aris::core::Calculator& { return imp_->calculator_; }
		auto Model::environment()->aris::dynamic::Environment& { return *imp_->environment_; }
		auto Model::variablePool()->aris::core::ObjectPool<Variable, Element>& { return *imp_->variable_pool_; }
		auto Model::partPool()->aris::core::ObjectPool<Part, Element>& { return *imp_->part_pool_; }
		auto Model::jointPool()->aris::core::ObjectPool<Joint, Element>& { return *imp_->joint_pool_; }
		auto Model::motionPool()->aris::core::ObjectPool<Motion, Element>& { return *imp_->motion_pool_; }
		auto Model::generalMotionPool()->aris::core::ObjectPool<GeneralMotion, Element>& { return *imp_->general_motion_pool_; }
		auto Model::forcePool()->aris::core::ObjectPool<Force, Element>& { return *imp_->force_pool_; }
		auto Model::solverPool()->aris::core::ObjectPool<Solver, Element>& { return *imp_->solver_pool_; }
		auto Model::simulatorPool()->aris::core::ObjectPool<Simulator, Element>& { return *imp_->simulator_pool_; }
		auto Model::simResultPool()->aris::core::ObjectPool<SimResult, Element>& { return *imp_->sim_result_pool_; }
		auto Model::ground()->Part& { return *imp_->ground_; }
		auto Model::addPartByPm(const double*pm, const double *prt_im)->Part& { return partPool().add<Part>("part_" + std::to_string(partPool().size()), prt_im, pm); }
		auto Model::addPartByPe(const double*pe, const char* eul_type, const double *prt_im)->Part& 
		{ 
			double pm[16];
			s_pe2pm(pe, pm, eul_type);
			return partPool().add<Part>("part_" + std::to_string(partPool().size()), prt_im, pm); 
		}
		auto Model::addPartByPq(const double*pq, const double *prt_im)->Part&
		{
			double pm[16];
			s_pq2pm(pq, pm);
			return partPool().add<Part>("part_" + std::to_string(partPool().size()), prt_im, pm);
		}
		auto Model::addRevoluteJoint(Part &first_part, Part &second_part, const double *position, const double *axis)->RevoluteJoint&
		{
			double glb_pm[16], loc_pm[16];
			s_sov_axes2pm(position, axis, axis, glb_pm, "zx");
			auto name = "joint_" + std::to_string(jointPool().size());
			s_inv_pm_dot_pm(*first_part.glbPm(), glb_pm, loc_pm);
			auto &mak_i = first_part.markerPool().add<Marker>(name + "_i", loc_pm);
			s_inv_pm_dot_pm(*second_part.glbPm(), glb_pm, loc_pm);
			auto &mak_j = second_part.markerPool().add<Marker>(name + "_j", loc_pm);
			return jointPool().add<RevoluteJoint>(name, &mak_i, &mak_j);
		}
		auto Model::addPrismaticJoint(Part &first_part, Part &second_part, const double *position, const double *axis)->PrismaticJoint&
		{
			double glb_pm[16], loc_pm[16];
			s_sov_axes2pm(position, axis, axis, glb_pm, "zx");
			auto name = "joint_" + std::to_string(jointPool().size());
			s_inv_pm_dot_pm(*first_part.glbPm(), glb_pm, loc_pm);
			auto &mak_i = first_part.markerPool().add<Marker>(name + "_i", loc_pm);
			s_inv_pm_dot_pm(*second_part.glbPm(), glb_pm, loc_pm);
			auto &mak_j = second_part.markerPool().add<Marker>(name + "_j", loc_pm);
			return jointPool().add<PrismaticJoint>(name, &mak_i, &mak_j);
		}
		auto Model::addUniversalJoint(Part &first_part, Part &second_part, const double *position, const double *first_axis, const double *second_axis)->UniversalJoint&
		{
			double glb_pm[16], loc_pm[16];
			s_sov_axes2pm(position, first_axis, second_axis, glb_pm, "zx");
			auto name = "joint_" + std::to_string(jointPool().size());
			s_inv_pm_dot_pm(*first_part.glbPm(), glb_pm, loc_pm);
			auto &mak_i = first_part.markerPool().add<Marker>(name + "_i", loc_pm);
			s_sov_axes2pm(position, second_axis, first_axis, glb_pm, "zx");
			s_inv_pm_dot_pm(*second_part.glbPm(), glb_pm, loc_pm);
			auto &mak_j = second_part.markerPool().add<Marker>(name + "_j", loc_pm);
			return jointPool().add<UniversalJoint>(name, &mak_i, &mak_j);
		}
		auto Model::addSphericalJoint(Part &first_part, Part &second_part, const double *position)->SphericalJoint&
		{
			double glb_pm[16]{ 1,0,0,position[0],0,1,0,position[1],0,0,1,position[2],0,0,0,1 }, loc_pm[16];
			auto name = "joint_" + std::to_string(jointPool().size());
			s_inv_pm_dot_pm(*first_part.glbPm(), glb_pm, loc_pm);
			auto &mak_i = first_part.markerPool().add<Marker>(name + "_i", loc_pm);
			s_inv_pm_dot_pm(*second_part.glbPm(), glb_pm, loc_pm);
			auto &mak_j = second_part.markerPool().add<Marker>(name + "_j", loc_pm);
			return jointPool().add<SphericalJoint>(name, &mak_i, &mak_j);
		}
		auto Model::addMotion(Joint &joint)->Motion&
		{
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
				throw std::runtime_error("wrong joint when Model::addMotion(joint)");
			}

			return motionPool().add<Motion>("motion_" + std::to_string(motionPool().size()), &joint.makI(), &joint.makJ(), dim);
		}
		auto Model::addMotion()->Motion&
		{
			if (ground().markerPool().findByName("origin") == ground().markerPool().end())
			{
				double pm[16] = { 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
				this->ground().markerPool().add<Marker>("origin", pm);
			}
			
			auto mak = ground().markerPool().findByName("origin");
			return motionPool().add<Motion>("motion_" + std::to_string(motionPool().size()), &*mak, &*mak, 0);
		}
		auto Model::addGeneralMotion(Part &end_effector, Coordinate &reference, const double* pm)->GeneralMotion&
		{
			double pm_prt[16], pm_target_in_ground[16];
			s_pm_dot_pm(*reference.pm(), pm, pm_target_in_ground);
			s_inv_pm_dot_pm(*end_effector.pm(), pm_target_in_ground, pm_prt);

			auto name = "general_motion_" + std::to_string(generalMotionPool().size());
			auto &mak_i = end_effector.markerPool().add<Marker>(name + "_i", pm_prt);
			auto &mak_j = dynamic_cast<Part*>(&reference) ? dynamic_cast<Part&>(reference).markerPool().add<Marker>(name + "_j") : dynamic_cast<Marker&>(reference);
			return generalMotionPool().add<GeneralMotion>(name, &mak_i, &mak_j);
		}
		Model::~Model() = default;
		Model::Model(const std::string &name):Root(name)
		{
			registerChildType<Environment>();

			registerChildType<aris::core::ObjectPool<Variable, Element>>();
			registerChildType<MatrixVariable>();
			registerChildType<StringVariable>();

			registerChildType<aris::core::ObjectPool<Part, Element>>();
			registerChildType<Part>();

			registerChildType<aris::core::ObjectPool<Marker, Element>>();
			registerChildType<Marker>();

			registerChildType<aris::core::ObjectPool<Joint, Element>>();
			registerChildType<RevoluteJoint>();
			registerChildType<PrismaticJoint>();
			registerChildType<UniversalJoint>();
			registerChildType<SphericalJoint>();

			registerChildType<aris::core::ObjectPool<Motion, Element>>();
			registerChildType<Motion>();

			registerChildType<aris::core::ObjectPool<GeneralMotion, Element>>();
			registerChildType<GeneralMotion>();

			registerChildType<aris::core::ObjectPool<Force, Element>>();
			registerChildType<SingleComponentForce>();

			registerChildType<aris::core::ObjectPool<Geometry, Element>>();
			registerChildType<ParasolidGeometry>();

			registerChildType<aris::core::ObjectPool<Solver, Element>>();
			registerChildType<GroundCombineSolver>();
			registerChildType<LltGroundDividedSolver>();
			registerChildType<LltPartDividedSolver>();
			registerChildType<DiagSolver>();

			registerChildType<aris::core::ObjectPool<Simulator, Element>>();
			registerChildType<Simulator>();
			registerChildType<SolverSimulator>();
			registerChildType<AdamsSimulator>();
			
			registerChildType<aris::core::ObjectPool<SimResult, Element>>();
			registerChildType<SimResult>();
			registerChildType<aris::core::ObjectPool<SimResult::PartResult, Element>>();
			registerChildType<aris::core::ObjectPool<SimResult::ConstraintResult, Element>>();
			registerChildType<SimResult::PartResult>();
			registerChildType<SimResult::ConstraintResult>();
			registerChildType<SimResult::TimeResult>();

			imp_->environment_ = &this->add<Environment>("environment");
			imp_->variable_pool_ = &this->add<aris::core::ObjectPool<Variable, Element>>("variable_pool");
			imp_->part_pool_ = &this->add<aris::core::ObjectPool<Part, Element>>("part_pool");
			imp_->joint_pool_ = &this->add<aris::core::ObjectPool<Joint, Element>>("joint_pool");
			imp_->motion_pool_ = &this->add<aris::core::ObjectPool<Motion, Element>>("motion_pool");
			imp_->general_motion_pool_ = &this->add<aris::core::ObjectPool<GeneralMotion, Element>>("general_motion_pool");
			imp_->force_pool_ = &this->add<aris::core::ObjectPool<Force, Element>>("force_pool");
			imp_->solver_pool_ = &this->add<aris::core::ObjectPool<Solver, Element>>("solver_pool");
			imp_->simulator_pool_ = &this->add<aris::core::ObjectPool<Simulator, Element>>("simulator_pool");
			imp_->sim_result_pool_ = &this->add<aris::core::ObjectPool<SimResult, Element>>("sim_result_pool");

			imp_->ground_ = &imp_->part_pool_->add<Part>("ground");
		}
	}
}
