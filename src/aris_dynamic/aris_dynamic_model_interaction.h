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
		/// @defgroup dynamic_model_group 动力学建模模块
		/// @{
		///
		
		class Interaction :public DynEle
		{
		public:
			static auto Type()->const std::string &{ static const std::string type{ "Interaction" }; return type; }
			auto virtual type() const->const std::string& override{ return Type(); }
			auto virtual saveXml(aris::core::XmlElement &xml_ele) const->void override;
			auto virtual loadXml(const aris::core::XmlElement &xml_ele)->void override;
			auto makI() noexcept->Marker& { return *makI_; }
			auto makI() const noexcept->const Marker&{ return *makI_; }
			auto makJ() noexcept->Marker& { return *makJ_; }
			auto makJ() const noexcept->const Marker&{ return *makJ_; }

			virtual ~Interaction() = default;
			explicit Interaction(const std::string &name = "interaction", Marker *makI = nullptr, Marker *makJ = nullptr, bool is_active = true):DynEle(name, is_active), makI_(makI), makJ_(makJ) {}
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
			auto virtual locCmI() const->const double* = 0;
			auto virtual saveXml(aris::core::XmlElement &xml_ele) const->void override;
			auto virtual loadXml(const aris::core::XmlElement &xml_ele)->void override;
			auto virtual cptCpFromPm(double *cp, const double *makI_pm, const double *makJ_pm)const->void;
			auto virtual cptGlbDmFromPm(double *dm, const double *makI_pm, const double *makJ_pm)const->void
			{
				double cmI[36], cmJ[36];
				double U[36], tau[6];
				double Q[36], R[36];

				cptGlbCmFromPm(cmI, cmJ, makI_pm, makJ_pm);

				s_householder_ut(6, dim(), cmI, U, tau);
				s_householder_ut2qr(6, dim(), U, tau, Q, R);

				double tem[36];
				s_fill(6, 6, 0.0, tem);
				s_fill(6, 1, 1.0, tem, 7);
				s_inv_um(dim(), R, dim(), tem, 6);
				s_mm(6, 6, 6, tem, 6, Q, dynamic::ColMajor{ 6 }, dm, 6);
			}
			auto virtual cptGlbCmFromPm(double *cmI, double *cmJ, const double *makI_pm, const double *makJ_pm)const->void
			{
				s_tf_n(dim(), makI_pm, locCmI(), cmI);
				s_mi(6, dim(), cmI, cmJ);
			}
			auto virtual cptCp(double *cp)const->void { cptCpFromPm(cp, *makI().pm(), *makJ().pm()); }
			auto virtual cptCv(double *cv)const->void;
			auto virtual cptCa(double *ca)const->void;
			template<typename CMI_TYPE, typename CMJ_TYPE>
			auto cptCm(const Coordinate &relative_to_I, double *cmI, CMI_TYPE cmi_type, const Coordinate &relative_to_J, double *cmJ, CMJ_TYPE cmj_type)->void
			{
				double pm[16];
				makI().getPm(relative_to_I, pm);
				s_tf_n(dim(), pm, locCmI(), dim(), cmI, cmi_type);

				makI().getPm(relative_to_J, pm);
				s_tf_n(dim(), -1.0, pm, locCmI(), dim(), cmJ, cmj_type);
			}
			template<typename CMI_TYPE, typename CMJ_TYPE>
			auto cptCm(const Coordinate &relative_to, double *cmI, CMI_TYPE cmi_type, double *cmJ, CMJ_TYPE cmj_type)->void
			{
				double pm[16];
				makI().getPm(relative_to, pm);
				s_tf_n(dim(), pm, locCmI(), dim(), cmI, cmi_type);

				s_mi(6, dim(), cmI, cmi_type, cmJ, cmj_type);
			}
			template<typename CMI_TYPE, typename CMJ_TYPE>
			auto cptPrtCm(double *cmI, CMI_TYPE cmi_type, double *cmJ, CMJ_TYPE cmj_type)->void
			{
				cptCm(makI().fatherPart(), cmI, cmi_type, makJ().fatherPart(), cmJ, cmj_type);
			}
			auto cptPrtCm(double *cmI, double *cmJ)->void { cptPrtCm(cmI, dim(), cmJ, dim()); }
			template<typename CMI_TYPE, typename CMJ_TYPE>
			auto cptGlbCm(double *cmI, CMI_TYPE cmi_type, double *cmJ, CMJ_TYPE cmj_type)->void
			{
				s_tf_n(dim(), *makI().pm(), locCmI(), dim(), cmI, cmi_type);
				s_mi(6, dim(), cmI, cmi_type, cmJ, cmj_type);
			}
			auto cptGlbCm(double *cmI, double *cmJ)->void { cptGlbCm(cmI, dim(), cmJ, dim()); }
			auto cf() const->const double*;
			auto setCf(const double *cf)->void;

			
			virtual ~Constraint();
			explicit Constraint(const std::string &name = "constraint", Marker *makI = nullptr, Marker *makJ = nullptr, bool is_active = true);
			Constraint(const Constraint&);
			Constraint(Constraint&&);
			Constraint& operator=(const Constraint&);
			Constraint& operator=(Constraint&&);

		private:
			struct Imp;
			aris::core::ImpPtr<Imp> imp_;
			friend class Motion;
			friend class GeneralMotion;
		};
		class Joint :public Constraint
		{
		public:
			static auto Type()->const std::string &{ static const std::string type{ "Joint" }; return type; }
			auto virtual type() const->const std::string& override{ return Type(); }

			virtual ~Joint() = default;
			explicit Joint(const std::string &name = "joint", Marker *makI = nullptr, Marker *makJ = nullptr, bool active = true) : Constraint(name, makI, makJ, active) {}
			Joint(const Joint &other);
			Joint(Joint &&other);
			Joint& operator=(const Joint &other);
			Joint& operator=(Joint &&other);
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
			auto virtual locCmI() const->const double* override;
			auto virtual cptCpFromPm(double *cp, const double *makI_pm, const double *makJ_pm)const->void override;
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

			virtual ~Motion();
			explicit Motion(const std::string &name = "motion", Marker *makI = nullptr, Marker *makJ = nullptr, Size component_axis = 2, const double *frc_coe = nullptr, double mp_offset = 0.0, double mp_factor = 1.0, bool active = true);
			Motion(const Motion &other);
			Motion(Motion &&other);
			Motion& operator=(const Motion &other);
			Motion& operator=(Motion &&other);
		
		private:
			struct Imp;
			aris::core::ImpPtr<Imp> imp_;
		};
		class GeneralMotion final :public Constraint
		{
		public:
			
			static auto Type()->const std::string &{ static const std::string type{ "GeneralMotion" }; return type; }
			static auto Dim()->Size { return 6; }
			auto virtual type() const->const std::string& override{ return Type(); }
			auto virtual dim() const ->Size override { return Dim(); }
			auto virtual locCmI() const->const double* override;
			auto virtual cptCpFromPm(double *cp, const double *makI_pm, const double *makJ_pm)const->void override;
			auto virtual cptGlbDmFromPm(double *dm, const double *makI_pm, const double *makJ_pm)const->void override
			{
				double pm[16];
				s_inv_pm(makI_pm, pm);
				s_tmf(pm, dm);
			}
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

			virtual ~GeneralMotion();
			explicit GeneralMotion(const std::string &name = "general_motion", Marker *makI = nullptr, Marker *makJ = nullptr, bool active = true);
			GeneralMotion(const GeneralMotion &other);
			GeneralMotion(GeneralMotion &&other);
			GeneralMotion& operator=(const GeneralMotion &other);
			GeneralMotion& operator=(GeneralMotion &&other);

		private:
			struct Imp;
			aris::core::ImpPtr<Imp> imp_;
		};
		class Force :public Interaction
		{
		public:
			static auto Type()->const std::string &{ static const std::string type{ "Force" }; return type; }
			auto virtual type()const->const std::string & override{ return Type(); }
			auto virtual cptGlbFs(double *fsI, double *fsJ)->void = 0;

			virtual ~Force() = default;
			explicit Force(const std::string &name = "force", Marker *makI = nullptr, Marker *makJ = nullptr, bool active = true):Interaction(name, makI, makJ, active) {}
			Force(const Force &other) = default;
			Force(Force &&other) = default;
			Force& operator=(const Force &other) = default;
			Force& operator=(Force &&other) = default;
		};

		class RevoluteJoint final :public Joint
		{
		public:
			static const std::string& Type() { static const std::string type("RevoluteJoint"); return type; }
			static auto Dim()->Size { return 5; }
			auto virtual type() const->const std::string& override{ return Type(); }
			auto virtual dim() const ->Size override { return Dim(); }
			auto virtual locCmI() const->const double* override;
			auto virtual cptCpFromPm(double *cp, const double *makI_pm, const double *makJ_pm)const->void override;
			auto virtual cptGlbDmFromPm(double *dm, const double *makI_pm, const double *makJ_pm)const->void override
			{
				double pm[16];
				s_inv_pm(makI_pm, pm);
				s_tmf(pm, dm);
			}


			virtual ~RevoluteJoint() = default;
			explicit RevoluteJoint(const std::string &name = "revolute_joint", Marker *makI = nullptr, Marker *makJ = nullptr);
			RevoluteJoint(const RevoluteJoint &other) = default;
			RevoluteJoint(RevoluteJoint &&other) = default;
			RevoluteJoint& operator=(const RevoluteJoint &other) = default;
			RevoluteJoint& operator=(RevoluteJoint &&other) = default;
		};
		class PrismaticJoint final :public Joint
		{
		public:
			static const std::string& Type() { static const std::string type("PrismaticJoint"); return type; }
			static auto Dim()->Size { return 5; }
			auto virtual type() const->const std::string& override{ return Type(); }
			auto virtual dim() const->Size override { return Dim(); }
			auto virtual locCmI() const->const double* override;
			auto virtual cptCpFromPm(double *cp, const double *makI_pm, const double *makJ_pm)const->void override;
			auto virtual cptGlbDmFromPm(double *dm, const double *makI_pm, const double *makJ_pm)const->void override
			{
				double pm[16];
				s_inv_pm(makI_pm, pm);
				s_tmf(pm, dm);

				s_swap_m(1, 6, dm + 12, dm + 18);
				s_swap_m(1, 6, dm + 18, dm + 24);
				s_swap_m(1, 6, dm + 24, dm + 30);
			}
			
			virtual ~PrismaticJoint() = default;
			explicit PrismaticJoint(const std::string &name = "prismatic_joint", Marker *makI = nullptr, Marker *makJ = nullptr);
			PrismaticJoint(const PrismaticJoint &other) = default;
			PrismaticJoint(PrismaticJoint &&other) = default;
			PrismaticJoint& operator=(const PrismaticJoint &other) = default;
			PrismaticJoint& operator=(PrismaticJoint &&other) = default;
		};
		class UniversalJoint final :public Joint
		{
		public:
			static const std::string& Type() { static const std::string type("UniversalJoint"); return type; }
			static auto Dim()->Size { return 4; }
			auto virtual type() const->const std::string& override{ return Type(); }
			auto virtual dim() const->Size override { return Dim(); }
			auto virtual locCmI() const->const double* override;
			auto virtual cptCpFromPm(double *cp, const double *makI_pm, const double *makJ_pm)const->void override;
			auto virtual cptGlbDmFromPm(double *dm, const double *makI_pm, const double *makJ_pm)const->void override
			{
				double x2 = makI_pm[0] * makJ_pm[2] + makI_pm[4] * makJ_pm[6] + makI_pm[8] * makJ_pm[10];
				double y2 = makI_pm[1] * makJ_pm[2] + makI_pm[5] * makJ_pm[6] + makI_pm[9] * makJ_pm[10];

				double norm = std::sqrt(x2*x2 + y2*y2);

				double pm[16];
				s_inv_pm(makI_pm, pm);
				s_tmf(pm, dm);

				double r[4]{ -y2 / norm, x2 / norm, -x2 / norm, -y2 / norm };
				s_mm(2, 6, 2, r, dm + 18, pm);
				s_mc(2, 6, pm, dm + 18);
			}
			auto virtual cptGlbCmFromPm(double *cmI, double *cmJ, const double *makI_pm, const double *makJ_pm)const->void override
			{
				static double loc_cst[6][4]{
					1,0,0,0,
					0,1,0,0,
					0,0,1,0,
					0,0,0,0,
					0,0,0,0,
					0,0,0,0,
				};

				double x2 = makI_pm[0] * makJ_pm[2] + makI_pm[4] * makJ_pm[6] + makI_pm[8] * makJ_pm[10];
				double y2 = makI_pm[1] * makJ_pm[2] + makI_pm[5] * makJ_pm[6] + makI_pm[9] * makJ_pm[10];

				double norm = std::sqrt(x2*x2 + y2*y2);

				loc_cst[3][3] = -y2 / norm;
				loc_cst[4][3] = x2 / norm;

				s_tf_n(dim(), makI_pm, *loc_cst, cmI);
				s_mi(6, dim(), cmI, cmJ);
			}
			auto virtual cptCa(double *ca)const->void override;

			
			virtual ~UniversalJoint();
			explicit UniversalJoint(const std::string &name = "universal_joint", Marker *makI = nullptr, Marker *makJ = nullptr);
			UniversalJoint(const UniversalJoint &other);
			UniversalJoint(UniversalJoint &&other);
			UniversalJoint& operator=(const UniversalJoint &other);
			UniversalJoint& operator=(UniversalJoint &&other);
		
		private:
			struct Imp;
			aris::core::ImpPtr<Imp> imp_;
		};
		class SphericalJoint final :public Joint
		{
		public:
			static const std::string& Type() { static const std::string type("SphericalJoint"); return type; }
			static auto Dim()->Size { return 3; }
			auto virtual type() const->const std::string& override{ return Type(); }
			auto virtual dim() const->Size override { return Dim(); }
			auto virtual locCmI() const->const double* override;
			auto virtual cptCpFromPm(double *cp, const double *makI_pm, const double *makJ_pm)const->void override;
			auto virtual cptGlbDmFromPm(double *dm, const double *makI_pm, const double *makJ_pm)const->void override
			{
				double pm[16];
				s_inv_pm(makI_pm, pm);
				s_tmf(pm, dm);
			}

			virtual ~SphericalJoint() = default;
			explicit SphericalJoint(const std::string &name = "spherical_joint", Marker *makI = nullptr, Marker *makJ = nullptr);
			SphericalJoint(const SphericalJoint &other) = default;
			SphericalJoint(SphericalJoint &&other) = default;
			SphericalJoint& operator=(const SphericalJoint &other) = default;
			SphericalJoint& operator=(SphericalJoint &&other) = default;
		};

		class GeneralForce final :public Force
		{
		public:
			static const std::string& Type() { static const std::string type("GeneralForce"); return type; }
			auto virtual type() const->const std::string& override{ return Type(); }
			auto virtual cptGlbFs(double *fsI, double *fsJ)->void override { s_vc(6, fce_value_, fsI); s_vi(6, fce_value_, fsJ); }
			auto setFce(const double *value)->void { std::copy(value, value + 6, fce_value_); }
			auto fce()const->const double* { return fce_value_; }

			virtual ~GeneralForce() = default;
			explicit GeneralForce(const std::string &name = "general_force", Marker *makI = nullptr, Marker *makJ = nullptr) : Force(name, makI, makJ) {};
			GeneralForce(const GeneralForce &other) = default;
			GeneralForce(GeneralForce &&other) = default;
			GeneralForce& operator=(const GeneralForce &other) = default;
			GeneralForce& operator=(GeneralForce &&other) = default;

		private:
			double fce_value_[6]{ 0 };
		};
		class SingleComponentForce final :public Force
		{
		public:
			static const std::string& Type() { static const std::string type("SingleComponentForce"); return type; }
			auto virtual type() const->const std::string& override{ return Type(); }
			auto virtual saveXml(aris::core::XmlElement &xml_ele) const->void override;
			auto virtual loadXml(const aris::core::XmlElement &xml_ele)->void override;
			auto virtual cptGlbFs(double *fsI, double *fsJ)->void override;
			auto setComponentID(Size id)->void { component_axis_ = id; }
			auto setFce(double value)->void { std::fill_n(fce_value_, 6, 0); fce_value_[component_axis_] = value; }
			auto setFce(double value, Size componentID)->void { this->component_axis_ = componentID; setFce(value); }
			auto fce()const->double { return fce_value_[component_axis_]; }

			virtual ~SingleComponentForce() = default;
			explicit SingleComponentForce(const std::string &name = "single_component_force", Marker *makI = nullptr, Marker *makJ = nullptr, Size componentID = 0);
			SingleComponentForce(const SingleComponentForce &other) = default;
			SingleComponentForce(SingleComponentForce &&other) = default;
			SingleComponentForce& operator=(const SingleComponentForce &other) = default;
			SingleComponentForce& operator=(SingleComponentForce &&other) = default;

		private:
			Size component_axis_;
			double fce_value_[6]{ 0 };
		};

		/// @}
	}
}

#endif
