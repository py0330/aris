#ifndef ARIS_DYNAMIC_MODEL_INTERACTION_H_
#define ARIS_DYNAMIC_MODEL_INTERACTION_H_

#include <cmath>

#include <aris/dynamic/model_coordinate.hpp>

namespace aris::dynamic
{
	/// @defgroup dynamic_model_group 动力学建模模块
	/// @{
	///

	class Interaction :public DynEle
	{
	public:
		auto virtual saveXml(aris::core::XmlElement &xml_ele) const->void override;
		auto virtual loadXml(const aris::core::XmlElement &xml_ele)->void override;
		auto makI() noexcept->Marker* { return makI_; }
		auto makI() const noexcept->const Marker* { return makI_; }
		auto makJ() noexcept->Marker* { return makJ_; }
		auto makJ() const noexcept->const Marker* { return makJ_; }
		auto prtNameM()const->std::string;
		auto setPrtNameM(std::string_view name)->void;
		auto prtNameN()const->std::string;
		auto setPrtNameN(std::string_view name)->void;
		auto makNameI()const->std::string;
		auto setMakNameI(std::string_view name)->void;
		auto makNameJ()const->std::string;
		auto setMakNameJ(std::string_view name)->void;

		virtual ~Interaction() = default;
		explicit Interaction(const std::string &name = "interaction", Marker *makI = nullptr, Marker *makJ = nullptr, bool is_active = true);
		ARIS_REGISTER_TYPE(Interaction);
		ARIS_DEFINE_BIG_FOUR(Interaction);

	private:
		Marker *makI_{ nullptr }, *makJ_{nullptr};
		std::string prt_name_M_, prt_name_N_, mak_name_I_, mak_name_J_;
		friend class Model;
	};
	class Constraint :public Interaction
	{
	public:
		auto virtual dim() const->Size = 0;
		auto virtual locCmI() const->const double* = 0;
		auto virtual saveXml(aris::core::XmlElement &xml_ele) const->void override;
		auto virtual loadXml(const aris::core::XmlElement &xml_ele)->void override;
		auto virtual cptCpFromPm(double *cp, const double *makI_pm, const double *makJ_pm)const noexcept->void;
		auto virtual cptCp(double *cp)const noexcept->void { cptCpFromPm(cp, *makI()->pm(), *makJ()->pm()); }
		auto virtual cptCv(double *cv)const noexcept->void;
		auto virtual cptCa(double *ca)const noexcept->void;
		auto virtual cptGlbDmFromPm(double *dm, const double *makI_pm, const double *makJ_pm)const noexcept->void
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
		auto virtual cptGlbCmFromPm(double *cmI, double *cmJ, const double *makI_pm, const double *makJ_pm)const noexcept->void
		{
			s_tf_n(dim(), makI_pm, locCmI(), cmI);
			s_mi(6, dim(), cmI, cmJ);
		}
		template<typename CMI_TYPE, typename CMJ_TYPE>
		auto cptCm(const Coordinate &relative_to_I, double *cmI, CMI_TYPE cmi_type, const Coordinate &relative_to_J, double *cmJ, CMJ_TYPE cmj_type)const noexcept->void
		{
			double pm[16];
			makI()->getPm(relative_to_I, pm);
			s_tf_n(dim(), pm, locCmI(), dim(), cmI, cmi_type);

			makI()->getPm(relative_to_J, pm);
			s_tf_n(dim(), -1.0, pm, locCmI(), dim(), cmJ, cmj_type);
		}
		template<typename CMI_TYPE, typename CMJ_TYPE>
		auto cptCm(const Coordinate &relative_to, double *cmI, CMI_TYPE cmi_type, double *cmJ, CMJ_TYPE cmj_type)const noexcept->void
		{
			double pm[16];
			makI()->getPm(relative_to, pm);
			s_tf_n(dim(), pm, locCmI(), dim(), cmI, cmi_type);

			s_mi(6, dim(), cmI, cmi_type, cmJ, cmj_type);
		}
		template<typename CMI_TYPE, typename CMJ_TYPE>
		auto cptPrtCm(double *cmI, CMI_TYPE cmi_type, double *cmJ, CMJ_TYPE cmj_type)const noexcept->void
		{
			cptCm(makI()->fatherPart(), cmI, cmi_type, makJ()->fatherPart(), cmJ, cmj_type);
		}
		auto cptPrtCm(double *cmI, double *cmJ)->void { cptPrtCm(cmI, dim(), cmJ, dim()); }
		template<typename CMI_TYPE, typename CMJ_TYPE>
		auto cptGlbCm(double *cmI, CMI_TYPE cmi_type, double *cmJ, CMJ_TYPE cmj_type)const noexcept->void
		{
			s_tf_n(dim(), *makI()->pm(), locCmI(), dim(), cmI, cmi_type);
			s_mi(6, dim(), cmI, cmi_type, cmJ, cmj_type);
		}
		auto cptGlbCm(double *cmI, double *cmJ)const noexcept->void{ cptGlbCm(cmI, dim(), cmJ, dim()); }
		auto cf() const noexcept->const double*;
		auto setCf(const double *cf) noexcept->void;


		virtual ~Constraint();
		explicit Constraint(const std::string &name = "constraint", Marker *makI = nullptr, Marker *makJ = nullptr, bool is_active = true);
		ARIS_REGISTER_TYPE(Constraint);
		ARIS_DECLARE_BIG_FOUR(Constraint);

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
		friend class Motion;
		friend class GeneralMotion;
	};
	class Joint :public Constraint
	{
	public:
		virtual ~Joint() = default;
		explicit Joint(const std::string &name = "joint", Marker *makI = nullptr, Marker *makJ = nullptr, bool active = true) : Constraint(name, makI, makJ, active) {}
		ARIS_REGISTER_TYPE(Joint);
		ARIS_DEFINE_BIG_FOUR(Joint);
	};
	class Motion final :public Constraint
	{
	public:
		static auto Dim()->Size { return 1; }
		auto virtual saveXml(aris::core::XmlElement &xml_ele) const->void override;
		auto virtual loadXml(const aris::core::XmlElement &xml_ele)->void override;
		auto virtual dim() const noexcept ->Size override { return Dim(); }
		auto virtual locCmI() const noexcept->const double* override;
		auto virtual cptCpFromPm(double *cp, const double *makI_pm, const double *makJ_pm)const noexcept->void override;
		auto virtual cptCv(double *cv)const noexcept->void override;
		auto virtual cptCa(double *ca)const noexcept->void override;

		auto setAxis(Size axis)->void;
		auto axis()const noexcept->Size;
		auto mp() const noexcept->double;
		auto updMp() noexcept->void;
		auto setMp(double mp) noexcept->void;
		auto mv() const noexcept->double;
		auto updMv() noexcept->void;
		auto setMv(double mv) noexcept->void;
		auto ma() const noexcept->double;
		auto updMa() noexcept->void;
		auto setMa(double ma) noexcept->void;
		auto mf() const noexcept->double;
		auto setMf(double mf) noexcept->void;
		auto mfDyn() const noexcept->double;
		auto setMfDyn(double mf_dyn) noexcept->void;
		auto mfFrc() const noexcept->double;
		auto frcCoe() const noexcept ->const double3&;
		auto setFrcCoe(const double *frc_coe) noexcept->void;
		auto frcZeroCheck()const noexcept ->double { return 1e-3; }
		auto mpOffset()const noexcept->double;
		auto setMpOffset(double mp_offset)noexcept->void;
		auto mpFactor()const noexcept->double;
		auto setMpFactor(double mp_factor)noexcept->void;
		auto mpInternal()const noexcept->double;
		auto setMpInternal(double mp_internal)noexcept->void;

		virtual ~Motion();
		explicit Motion(const std::string &name = "motion", Marker *makI = nullptr, Marker *makJ = nullptr, Size component_axis = 2, const double *frc_coe = nullptr, double mp_offset = 0.0, double mp_factor = 1.0, bool active = true);
		ARIS_REGISTER_TYPE(Motion);
		ARIS_DECLARE_BIG_FOUR(Motion);

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};
	class GeneralMotion final :public Constraint
	{
	public:
		static auto Dim()->Size { return 6; }
		auto virtual dim() const noexcept ->Size override { return Dim(); }
		auto virtual locCmI() const noexcept->const double* override;
		auto virtual cptCpFromPm(double *cp, const double *makI_pm, const double *makJ_pm)const noexcept->void override;
		auto virtual cptGlbDmFromPm(double *dm, const double *makI_pm, const double *makJ_pm)const noexcept->void override
		{
			double pm[16];
			s_inv_pm(makI_pm, pm);
			s_tmf(pm, dm);
		}
		auto virtual cptCv(double *cv)const noexcept->void override;
		auto virtual cptCa(double *ca)const noexcept->void override;

		auto mpm()const noexcept->const double4x4&;
		auto updMpm() noexcept->void;
		auto setMpe(const double* pe, const char *type = "313") noexcept->void;
		auto setMpq(const double* pq) noexcept->void;
		auto setMpm(const double* pm) noexcept->void;
		auto getMpe(double* pe, const char *type = "313")const noexcept->void;
		auto getMpq(double* pq)const noexcept->void;
		auto getMpm(double* pm)const noexcept->void;
		auto mvs()const noexcept->const double6&;
		auto updMvs() noexcept->void;
		auto setMve(const double* ve, const char *type = "313") noexcept->void;
		auto setMvq(const double* vq) noexcept->void;
		auto setMvm(const double* vm) noexcept->void;
		auto setMva(const double* va) noexcept->void;
		auto setMvs(const double* vs) noexcept->void;
		auto getMve(double* ve, const char *type = "313")const noexcept->void;
		auto getMvq(double* vq)const noexcept->void;
		auto getMvm(double* vm)const noexcept->void;
		auto getMva(double* va)const noexcept->void;
		auto getMvs(double* vs)const noexcept->void;
		auto mas()const noexcept->const double6&;
		auto updMas() noexcept->void;
		auto setMae(const double* ae, const char *type = "313") noexcept->void;
		auto setMaq(const double* aq) noexcept->void;
		auto setMam(const double* am) noexcept->void;
		auto setMaa(const double* aa) noexcept->void;
		auto setMas(const double* as) noexcept->void;
		auto getMae(double* ae, const char *type = "313")const noexcept->void;
		auto getMaq(double* aq)const noexcept->void;
		auto getMam(double* am)const noexcept->void;
		auto getMaa(double* aa)const noexcept->void;
		auto getMas(double* as)const noexcept->void;
		auto mfs() const noexcept->const double6&;
		auto setMfs(const double * mfs) noexcept->void;

		virtual ~GeneralMotion();
		explicit GeneralMotion(const std::string &name = "general_motion", Marker *makI = nullptr, Marker *makJ = nullptr, bool active = true);
		ARIS_REGISTER_TYPE(GeneralMotion);
		ARIS_DECLARE_BIG_FOUR(GeneralMotion);

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};
	class Force :public Interaction
	{
	public:
		auto virtual cptGlbFs(double *fsI, double *fsJ)const noexcept->void = 0;

		virtual ~Force() = default;
		explicit Force(const std::string &name = "force", Marker *makI = nullptr, Marker *makJ = nullptr, bool active = true) :Interaction(name, makI, makJ, active) {}
		ARIS_REGISTER_TYPE(Force);
		ARIS_DEFINE_BIG_FOUR(Force);
	};

	/*
	class Friction :public Element
	{
	public:
		auto frc()const noexcept ->double { return s_vv(frc_coe_.size(), frc_coe_.data(), frc_value_.data()); };
		auto frcValue()const noexcept ->std::tuple<const double *, Size> 
		{ 
			frc_coe_



			
			
			return std::make_tuple(frc_value_.data(), static_cast<Size>(frc_value_.size()));
		}
		auto frcCoe() const noexcept ->std::tuple<const double *, Size> { return std::make_tuple(frc_coe_.data(), static_cast<Size>(frc_coe_.size())); }
		auto setFrcCoe(const double *frc_coe, Size dim) noexcept->void { frc_coe_.assign(frc_coe, frc_coe + dim); frc_value_.resize(dim, 0.0); }
		


		



	private:
		std::vector<double> frc_coe_{0.0, 0.0, 0.0};
		std::vector<double> frc_value_{ 0.0, 0.0, 0.0 };
	};*/

	class RevoluteJoint final :public Joint
	{
	public:
		static auto Dim()->Size { return 5; }
		auto virtual dim() const noexcept->Size override { return Dim(); }
		auto virtual locCmI() const noexcept->const double* override;
		auto virtual cptCpFromPm(double *cp, const double *makI_pm, const double *makJ_pm)const noexcept->void override;
		auto virtual cptGlbDmFromPm(double *dm, const double *makI_pm, const double *makJ_pm)const noexcept->void override;

		virtual ~RevoluteJoint() = default;
		explicit RevoluteJoint(const std::string &name = "revolute_joint", Marker *makI = nullptr, Marker *makJ = nullptr);
		ARIS_REGISTER_TYPE(RevoluteJoint);
		ARIS_DEFINE_BIG_FOUR(RevoluteJoint);
	};
	class PrismaticJoint final :public Joint
	{
	public:
		static auto Dim()->Size { return 5; }
		auto virtual dim() const noexcept->Size override { return Dim(); }
		auto virtual locCmI() const noexcept->const double* override;
		auto virtual cptCpFromPm(double *cp, const double *makI_pm, const double *makJ_pm)const noexcept->void override;
		auto virtual cptGlbDmFromPm(double *dm, const double *makI_pm, const double *makJ_pm)const noexcept->void override;
		
		virtual ~PrismaticJoint() = default;
		explicit PrismaticJoint(const std::string &name = "prismatic_joint", Marker *makI = nullptr, Marker *makJ = nullptr);
		ARIS_REGISTER_TYPE(PrismaticJoint);
		ARIS_DEFINE_BIG_FOUR(PrismaticJoint);
	};
	class UniversalJoint final :public Joint
	{
	public:
		static auto Dim()->Size { return 4; }
		auto virtual dim() const noexcept->Size override { return Dim(); }
		auto virtual locCmI() const noexcept->const double* override;
		auto virtual cptCpFromPm(double *cp, const double *makI_pm, const double *makJ_pm)const noexcept->void override;
		auto virtual cptGlbDmFromPm(double *dm, const double *makI_pm, const double *makJ_pm)const noexcept->void override;
		auto virtual cptGlbCmFromPm(double *cmI, double *cmJ, const double *makI_pm, const double *makJ_pm)const noexcept->void override;
		auto virtual cptCa(double *ca)const noexcept->void override;

		virtual ~UniversalJoint();
		explicit UniversalJoint(const std::string &name = "universal_joint", Marker *makI = nullptr, Marker *makJ = nullptr);
		ARIS_REGISTER_TYPE(UniversalJoint);
		ARIS_DECLARE_BIG_FOUR(UniversalJoint);

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};
	class SphericalJoint final :public Joint
	{
	public:
		static auto Dim()->Size { return 3; }
		auto virtual dim() const noexcept->Size override { return Dim(); }
		auto virtual locCmI() const noexcept->const double* override;
		auto virtual cptCpFromPm(double *cp, const double *makI_pm, const double *makJ_pm)const noexcept->void override;
		auto virtual cptGlbDmFromPm(double *dm, const double *makI_pm, const double *makJ_pm)const noexcept->void override;

		virtual ~SphericalJoint() = default;
		explicit SphericalJoint(const std::string &name = "spherical_joint", Marker *makI = nullptr, Marker *makJ = nullptr);
		ARIS_REGISTER_TYPE(SphericalJoint);
		ARIS_DEFINE_BIG_FOUR(SphericalJoint);
	};

	class GeneralForce final :public Force
	{
	public:
		auto virtual cptGlbFs(double *fsI, double *fsJ)const noexcept->void override { s_vc(6, fce_value_, fsI); s_vi(6, fce_value_, fsJ); }
		auto setFce(const double *value) noexcept->void { std::copy(value, value + 6, fce_value_); }
		auto fce()const noexcept->const double* { return fce_value_; }

		virtual ~GeneralForce() = default;
		explicit GeneralForce(const std::string &name = "general_force", Marker *makI = nullptr, Marker *makJ = nullptr) : Force(name, makI, makJ) {};
		ARIS_REGISTER_TYPE(GeneralForce);
		ARIS_DEFINE_BIG_FOUR(GeneralForce);

	private:
		double fce_value_[6]{ 0 };
	};
	class SingleComponentForce final :public Force
	{
	public:
		auto virtual saveXml(aris::core::XmlElement &xml_ele) const->void override;
		auto virtual loadXml(const aris::core::XmlElement &xml_ele)->void override;
		auto virtual cptGlbFs(double *fsI, double *fsJ)const noexcept->void override;
		auto setComponentAxis(Size id) noexcept->void { component_axis_ = id; }
		auto componentAxis()const noexcept->Size { return component_axis_; }
		auto setFce(double value) noexcept->void { std::fill_n(fce_value_, 6, 0); fce_value_[component_axis_] = value; }
		auto setFce(double value, Size componentID) noexcept->void { this->component_axis_ = componentID; setFce(value); }
		auto fce()const noexcept->double { return fce_value_[component_axis_]; }

		virtual ~SingleComponentForce() = default;
		explicit SingleComponentForce(const std::string &name = "single_component_force", Marker *makI = nullptr, Marker *makJ = nullptr, Size componentID = 0);
		ARIS_REGISTER_TYPE(SingleComponentForce);
		ARIS_DEFINE_BIG_FOUR(SingleComponentForce);

	private:
		Size component_axis_;
		double fce_value_[6]{ 0 };
	};

	/// @}
}

#endif
