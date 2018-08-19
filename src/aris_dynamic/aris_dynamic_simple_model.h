#ifndef ARIS_DYNAMIC_SIMPLE_MODEL_
#define ARIS_DYNAMIC_SIMPLE_MODEL_

#include <aris_dynamic_model.h>

namespace aris::dynamic
{
	class SimpleModel
	{
	public:
		auto loadXmlFile(const std::string &file)->void;
		auto saveXmlFile(const std::string &file)->void;
		auto model()->Model&;
		auto ground()->Part&;
		auto addPart(const double *pm = nullptr)->Part*;
		auto addRevoluteJoint(Part *first_part, Part *second_part, const double *position, const double *axis)->RevoluteJoint*;
		auto addPrismaticJoint(Part *first_part, Part *second_part, const double *position, const double *axis)->PrismaticJoint*;
		auto addMotion(Joint *joint)->Motion*;
		auto addEndEffector(Part *end_effector, const double* pm)->GeneralMotion*;
		auto allocateMemory()->void;
		auto forwardKinematic(int max_count = 100, double error = 1e-10)->bool;
		auto inverseKinematic(int max_count = 100, double error = 1e-10)->bool;

		virtual ~SimpleModel();
		SimpleModel();

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};
}

#endif
