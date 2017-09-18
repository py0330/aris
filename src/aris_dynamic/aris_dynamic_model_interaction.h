#ifndef ARIS_DYNAMIC_MODEL_INTERACTION_
#define ARIS_DYNAMIC_MODEL_INTERACTION_

#include <vector>
#include <array>
#include <map>
#include <string>
#include <memory>
#include <functional>
#include <algorithm>

#include <aris_dynamic_model_basic.h>
#include <aris_dynamic_model_coordinate.h>

namespace aris
{
	namespace dynamic
	{
		class Interaction :public DynEle
		{
		public:
			static auto Type()->const std::string &{ static const std::string type{ "Interaction" }; return type; }
			auto virtual type() const->const std::string& override{ return Type(); }
			auto virtual saveXml(aris::core::XmlElement &xml_ele) const->void override;
			auto virtual loadXml(const aris::core::XmlElement &xml_ele)->void override;
			auto makI()->Marker& { return *makI_; }
			auto makI() const->const Marker&{ return *makI_; }
			auto makJ()->Marker& { return *makJ_; }
			auto makJ() const->const Marker&{ return *makJ_; }

		protected:
			virtual ~Interaction() = default;
			explicit Interaction(const std::string &name = "interaction", Marker *makI = nullptr, Marker *makJ = nullptr, bool is_active = true)
				: DynEle(name, is_active), makI_(makI), makJ_(makJ) {}
			Interaction(const Interaction &) = default;
			Interaction(Interaction &&) = default;
			Interaction& operator=(const Interaction &) = default;
			Interaction& operator=(Interaction &&) = default;

		private:
			Marker *makI_;
			Marker *makJ_;
		};
		class Constraint :public Interaction
		{
		public:
			auto virtual dim() const->Size = 0;
			auto virtual cptCp(double *cp)const->void;
			auto virtual cptCv(double *cv)const->void;
			auto virtual cptCa(double *ca)const->void;
			auto cf() const->const double*;
			auto setCf(const double *cf)->void;
			auto prtCmI() const->const double*;
			auto locCmI() const->const double*;
			template<typename CMI_TYPE, typename CMJ_TYPE>
			auto cptPrtCm(double *cmI, CMI_TYPE cmi_type, double *cmJ, CMJ_TYPE cmj_type)->void
			{
				updPrtCmI();
				s_mc(6, dim(), prtCmI(), dim(), cmI, cmi_type);

				double pm_M2N[4][4];
				s_inv_pm_dot_pm(*makJ().fatherPart().pm(), *makI().fatherPart().pm(), *pm_M2N);
				s_tf_n(dim(), -1.0, *pm_M2N, prtCmI(), dim(), cmJ, cmj_type);
			}
			auto cptPrtCm(double *cmI, double *cmJ)->void { cptPrtCm(cmI, dim(), cmJ, dim()); }
			template<typename CMI_TYPE, typename CMJ_TYPE>
			auto cptGlbCm(double *cmI, CMI_TYPE cmi_type, double *cmJ, CMJ_TYPE cmj_type)->void
			{
				updPrtCmI();
				s_tf_n(dim(), *makI().fatherPart().pm(), prtCmI(), dim(), cmI, cmi_type);
				s_mi(6, dim(), cmI, cmi_type, cmJ, cmj_type);
			}
			auto cptGlbCm(double *cmI, double *cmJ)->void { cptGlbCm(cmI, dim(), cmJ, dim()); }

		protected:
			auto virtual updPrtCmI()->void {};

			virtual ~Constraint();
			explicit Constraint(const std::string &name = "constraint", Marker *makI = nullptr, Marker *makJ = nullptr, bool is_active = true);
			Constraint(const Constraint&);
			Constraint(Constraint&&);
			Constraint& operator=(const Constraint&);
			Constraint& operator=(Constraint&&);

		private:
			struct Imp;
			aris::core::ImpPtr<Imp> imp_;

			friend class Model;
			friend class Motion;
			friend class GeneralMotion;
		};
		class Joint :public Constraint
		{
		public:
			static auto Type()->const std::string &{ static const std::string type{ "Joint" }; return type; }
			auto virtual type() const->const std::string& override{ return Type(); }

		protected:
			virtual ~Joint() = default;
			explicit Joint(const std::string &name = "joint", Marker *makI = nullptr, Marker *makJ = nullptr, bool active = true) : Constraint(name, makI, makJ, active) {}
			Joint(const Joint &other);
			Joint(Joint &&other);
			Joint& operator=(const Joint &other);
			Joint& operator=(Joint &&other);

			friend class aris::core::Root;
		};
		class Motion final :public Constraint
		{
		public:
			static auto Type()->const std::string &{ static const std::string type{ "Motion" }; return type; }
			static auto Dim()->Size { return 1; }
			auto virtual type() const->const std::string& override{ return Type(); }
			auto virtual saveXml(aris::core::XmlElement &xml_ele) const->void override;
			auto virtual loadXml(const aris::core::XmlElement &xml_ele)->void override;
			auto virtual dim() const ->Size override { return Dim(); }
			auto virtual cptCp(double *cp)const->void override;
			auto virtual cptCv(double *cv)const->void override;
			auto virtual cptCa(double *ca)const->void override;
			auto axis()const->Size;
			auto mp() const->double;
			auto updMp()->void;
			auto setMp(double mp)->void;
			auto mv() const->double;
			auto updMv()->void;
			auto setMv(double mv)->void;
			auto ma() const->double;
			auto updMa()->void;
			auto setMa(double ma)->void;
			auto mf() const->double;
			auto setMf(double mf)->void;
			auto mfDyn() const->double;
			auto setMfDyn(double mf_dyn)->void;
			auto mfFrc() const->double;
			auto frcCoe() const ->const double3&;
			auto setFrcCoe(const double *frc_coe)->void;

		protected:
			virtual ~Motion();
			explicit Motion(const std::string &name = "motion", Marker *makI = nullptr, Marker *makJ = nullptr, Size component_axis = 2, const double *frc_coe = nullptr, double mp_offset = 0.0, double mp_factor = 1.0, bool active = true);
			Motion(const Motion &other);
			Motion(Motion &&other);
			Motion& operator=(const Motion &other);
			Motion& operator=(Motion &&other);

			struct Imp;
			aris::core::ImpPtr<Imp> imp_;

			friend class Model;
			friend class aris::core::Root;
			friend class aris::core::Object;
		};
		class GeneralMotion final :public Constraint
		{
		public:
			static auto Type()->const std::string &{ static const std::string type{ "GeneralMotion" }; return type; }
			static auto Dim()->Size { return 6; }
			auto virtual type() const->const std::string& override{ return Type(); }
			auto virtual dim() const ->Size override { return Dim(); }
			auto virtual cptCp(double *cp)const->void override;
			auto virtual cptCv(double *cv)const->void override;
			auto virtual cptCa(double *ca)const->void override;
			auto mpm()const->const double4x4&;
			auto updMpm()->void;
			auto setMpe(const double* pe, const char *type = "313")->void;
			auto setMpq(const double* pq)->void;
			auto setMpm(const double* pm)->void;
			auto getMpe(double* pe, const char *type = "313")const->void;
			auto getMpq(double* pq)const->void;
			auto getMpm(double* pm)const->void;
			auto mvs()const->const double6&;
			auto updMvs()->void;
			auto setMve(const double* ve, const char *type = "313")->void;
			auto setMvq(const double* vq)->void;
			auto setMvm(const double* vm)->void;
			auto setMva(const double* va)->void;
			auto setMvs(const double* vs)->void;
			auto getMve(double* ve, const char *type = "313")const->void;
			auto getMvq(double* vq)const->void;
			auto getMvm(double* vm)const->void;
			auto getMva(double* va)const->void;
			auto getMvs(double* vs)const->void;
			auto mas()const->const double6&;
			auto updMas()->void;
			auto setMae(const double* ae, const char *type = "313")->void;
			auto setMaq(const double* aq)->void;
			auto setMam(const double* am)->void;
			auto setMaa(const double* aa)->void;
			auto setMas(const double* as)->void;
			auto getMae(double* ae, const char *type = "313")const->void;
			auto getMaq(double* aq)const->void;
			auto getMam(double* am)const->void;
			auto getMaa(double* aa)const->void;
			auto getMas(double* as)const->void;
			auto mfs() const->const double6&;
			auto setMfs(const double * mfs)->void;

		protected:
			virtual ~GeneralMotion();
			explicit GeneralMotion(const std::string &name = "general_motion", Marker *makI = nullptr, Marker *makJ = nullptr, const std::string& freedom = "xyz123", bool active = true);
			GeneralMotion(const GeneralMotion &other);
			GeneralMotion(GeneralMotion &&other);
			GeneralMotion& operator=(const GeneralMotion &other);
			GeneralMotion& operator=(GeneralMotion &&other);

			struct Imp;
			aris::core::ImpPtr<Imp> imp_;

			friend class Model;
			friend class aris::core::Root;
			friend class aris::core::Object;
		};
		class Force :public Interaction
		{
		public:
			static auto Type()->const std::string &{ static const std::string type{ "Force" }; return type; }
			auto virtual type()const->const std::string & override{ return Type(); }
			auto fsI() const->const double* { return fsI_; }
			auto fsJ() const->const double* { return fsJ_; }
			auto virtual updFs()->void = 0;

		protected:
			virtual ~Force() = default;
			explicit Force(const std::string &name = "force", Marker *makI = nullptr, Marker *makJ = nullptr, bool active = true):Interaction(name, makI, makJ, active) {}
			Force(const Force &other) = default;
			Force(Force &&other) = default;
			Force& operator=(const Force &other) = default;
			Force& operator=(Force &&other) = default;

			double fsI_[6]{ 0 };
			double fsJ_[6]{ 0 };

			friend class Model;
			friend class aris::core::Root;
		};

		class RevoluteJoint final :public Joint
		{
		public:
			static const std::string& Type() { static const std::string type("RevoluteJoint"); return type; }
			static auto Dim()->Size { return 5; }
			auto virtual type() const->const std::string& override{ return Type(); }
			auto virtual dim() const ->Size override { return Dim(); }
		private:
			virtual ~RevoluteJoint() = default;
			explicit RevoluteJoint(const std::string &name = "revolute_joint", Marker *makI = nullptr, Marker *makJ = nullptr);
			RevoluteJoint(const RevoluteJoint &other) = default;
			RevoluteJoint(RevoluteJoint &&other) = default;
			RevoluteJoint& operator=(const RevoluteJoint &other) = default;
			RevoluteJoint& operator=(RevoluteJoint &&other) = default;

			friend class Model;
			friend class aris::core::Root;
			friend class aris::core::Object;
		};
		class PrismaticJoint final :public Joint
		{
		public:
			static const std::string& Type() { static const std::string type("PrismaticJoint"); return type; }
			static auto Dim()->Size { return 5; }
			auto virtual type() const->const std::string& override{ return Type(); }
			auto virtual dim() const->Size override { return Dim(); }
		private:
			virtual ~PrismaticJoint() = default;
			explicit PrismaticJoint(const std::string &name = "prismatic_joint", Marker *makI = nullptr, Marker *makJ = nullptr);
			PrismaticJoint(const PrismaticJoint &other) = default;
			PrismaticJoint(PrismaticJoint &&other) = default;
			PrismaticJoint& operator=(const PrismaticJoint &other) = default;
			PrismaticJoint& operator=(PrismaticJoint &&other) = default;

			friend class Model;
			friend class aris::core::Root;
			friend class aris::core::Object;
		};
		class UniversalJoint final :public Joint
		{
		public:
			static const std::string& Type() { static const std::string type("UniversalJoint"); return type; }
			static auto Dim()->Size { return 4; }
			auto virtual type() const->const std::string& override{ return Type(); }
			auto virtual dim() const->Size override { return Dim(); }
			auto virtual cptCp(double *cp)const->void override;
			auto virtual cptCv(double *cv)const->void override;
			auto virtual cptCa(double *ca)const->void override;

		protected:
			auto virtual updPrtCmI()->void override;

		private:
			virtual ~UniversalJoint() = default;
			explicit UniversalJoint(const std::string &name = "universal_joint", Marker *makI = nullptr, Marker *makJ = nullptr);
			UniversalJoint(const UniversalJoint &other) = default;
			UniversalJoint(UniversalJoint &&other) = default;
			UniversalJoint& operator=(const UniversalJoint &other) = default;
			UniversalJoint& operator=(UniversalJoint &&other) = default;

			friend class Model;
			friend class aris::core::Root;
			friend class aris::core::Object;
		};
		class SphericalJoint final :public Joint
		{
		public:
			static const std::string& Type() { static const std::string type("SphericalJoint"); return type; }
			static auto Dim()->Size { return 3; }
			auto virtual type() const->const std::string& override{ return Type(); }
			auto virtual dim() const->Size override { return Dim(); }
		private:
			virtual ~SphericalJoint() = default;
			explicit SphericalJoint(const std::string &name = "spherical_joint", Marker *makI = nullptr, Marker *makJ = nullptr);
			SphericalJoint(const SphericalJoint &other) = default;
			SphericalJoint(SphericalJoint &&other) = default;
			SphericalJoint& operator=(const SphericalJoint &other) = default;
			SphericalJoint& operator=(SphericalJoint &&other) = default;

			friend class Model;
			friend class aris::core::Root;
			friend class aris::core::Object;
		};

		class SingleComponentForce final :public Force
		{
		public:
			static const std::string& Type() { static const std::string type("SingleComponentForce"); return type; }
			auto virtual type() const->const std::string& override{ return Type(); }
			auto virtual saveXml(aris::core::XmlElement &xml_ele) const->void override;
			auto virtual loadXml(const aris::core::XmlElement &xml_ele)->void override;
			auto virtual updFs()->void override;
			auto setComponentID(Size id)->void { component_axis_ = id; }
			auto setFce(double value)->void { std::fill_n(fce_value_, 6, 0); fce_value_[component_axis_] = value; }
			auto setFce(double value, Size componentID)->void { this->component_axis_ = componentID; setFce(value); }
			auto fce()const->double { return fce_value_[component_axis_]; }

		private:
			virtual ~SingleComponentForce() = default;
			explicit SingleComponentForce(const std::string &name = "single_component_force", Marker *makI = nullptr, Marker *makJ = nullptr, Size componentID = 0);
			explicit SingleComponentForce(Object &father, const aris::core::XmlElement &xml_ele);
			SingleComponentForce(const SingleComponentForce &other) = default;
			SingleComponentForce(SingleComponentForce &&other) = default;
			SingleComponentForce& operator=(const SingleComponentForce &other) = default;
			SingleComponentForce& operator=(SingleComponentForce &&other) = default;

			Size component_axis_;
			double fce_value_[6]{ 0 };

			friend class Model;
			friend class aris::core::Root;
			friend class aris::core::Object;
		};
	}
}

#endif
