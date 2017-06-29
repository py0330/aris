#ifndef ARIS_DYNAMIC_MODEL_JOINT_
#define ARIS_DYNAMIC_MODEL_JOINT_

#include <vector>
#include <array>
#include <map>
#include <string>
#include <memory>
#include <functional>
#include <algorithm>

#include <aris_dynamic_model.h>

namespace aris
{
	namespace dynamic
	{
		template<Size DIM> class JointTemplate :public Joint, public ConstraintData<DIM>
		{
		public:
			auto virtual dim() const->Size { return DIM; }
			auto virtual cfPtr() const->const double* override { return ConstraintData<DIM>::cf(); }
			auto virtual prtCmPtrI() const->const double* override { return *ConstraintData<DIM>::prtCmI(); }
			auto virtual locCmPtrI() const->const double* override { return *ConstraintData<DIM>::locCmI(); }

		protected:
			explicit JointTemplate(const std::string &name, Marker &makI, Marker &makJ)	:Joint(name, makI, makJ) {}
			explicit JointTemplate(Object &father, const aris::core::XmlElement &xml_ele):Joint(father, xml_ele) {}
			JointTemplate(const JointTemplate &other) = default;
			JointTemplate(JointTemplate &&other) = default;
			JointTemplate& operator=(const JointTemplate &other) = default;
			JointTemplate& operator=(JointTemplate &&other) = default;

		private:
			friend class Model;
		};

		class RevoluteJoint final :public JointTemplate<5>
		{
		public:
			static const std::string& Type() { static const std::string type("RevoluteJoint"); return type; }
			auto virtual type() const->const std::string& override{ return Type(); }

		private:
			virtual ~RevoluteJoint() = default;
			explicit RevoluteJoint(const std::string &name, Marker &makI, Marker &makJ);
			explicit RevoluteJoint(Object &father, const aris::core::XmlElement &xml_ele);
			RevoluteJoint(const RevoluteJoint &other) = default;
			RevoluteJoint(RevoluteJoint &&other) = default;
			RevoluteJoint& operator=(const RevoluteJoint &other) = default;
			RevoluteJoint& operator=(RevoluteJoint &&other) = default;

			friend class Model;
			friend class aris::core::Root;
			friend class aris::core::Object;
		};
		class PrismaticJoint final :public JointTemplate<5>
		{
		public:
			static const std::string& Type() { static const std::string type("PrismaticJoint"); return type; }
			auto virtual type() const->const std::string& override{ return Type(); }

		private:
			virtual ~PrismaticJoint() = default;
			explicit PrismaticJoint(const std::string &name, Marker &makI, Marker &makJ);
			explicit PrismaticJoint(Object &father, const aris::core::XmlElement &xml_ele);
			PrismaticJoint(const PrismaticJoint &other) = default;
			PrismaticJoint(PrismaticJoint &&other) = default;
			PrismaticJoint& operator=(const PrismaticJoint &other) = default;
			PrismaticJoint& operator=(PrismaticJoint &&other) = default;

			friend class Model;
			friend class aris::core::Root;
			friend class aris::core::Object;
		};
		class UniversalJoint final :public JointTemplate<4>
		{
		public:
			using Constraint::cptGlbCm;
			using Constraint::cptPrtCm;
			static const std::string& Type() { static const std::string type("UniversalJoint"); return type; }
			auto virtual type() const->const std::string& override{ return Type(); }
			auto virtual cptCp(double *cp)const->void override;
			auto virtual cptCv(double *cv)const->void override;
			auto virtual cptCa(double *ca)const->void override;

		protected:
			auto virtual updPrtCmI()->void override;

		private:
			virtual ~UniversalJoint() = default;
			explicit UniversalJoint(const std::string &name, Marker &makI, Marker &makJ);
			explicit UniversalJoint(Object &father, const aris::core::XmlElement &xml_ele);
			UniversalJoint(const UniversalJoint &other) = default;
			UniversalJoint(UniversalJoint &&other) = default;
			UniversalJoint& operator=(const UniversalJoint &other) = default;
			UniversalJoint& operator=(UniversalJoint &&other) = default;

			friend class Model;
			friend class aris::core::Root;
			friend class aris::core::Object;
		};
		class SphericalJoint final :public JointTemplate<3>
		{
		public:
			static const std::string& Type() { static const std::string type("SphericalJoint"); return type; }
			auto virtual type() const->const std::string& override{ return Type(); }

		private:
			virtual ~SphericalJoint() = default;
			explicit SphericalJoint(const std::string &Name, Marker &makI, Marker &makJ);
			explicit SphericalJoint(Object &father, const aris::core::XmlElement &xml_ele);
			SphericalJoint(const SphericalJoint &other) = default;
			SphericalJoint(SphericalJoint &&other) = default;
			SphericalJoint& operator=(const SphericalJoint &other) = default;
			SphericalJoint& operator=(SphericalJoint &&other) = default;

			friend class Model;
			friend class aris::core::Root;
			friend class aris::core::Object;
		};
	}
}

#endif
