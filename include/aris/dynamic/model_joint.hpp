#ifndef ARIS_DYNAMIC_MODEL_JOINT_H_
#define ARIS_DYNAMIC_MODEL_JOINT_H_

#include <cmath>

#include <aris/dynamic/model_interaction.hpp>

namespace aris::dynamic {
	/// @defgroup dynamic_model_group 动力学建模模块
	/// @{
	///

	class ARIS_API Joint :public Constraint{
	public:
		auto virtual cptCpFromPm(double* cp, const double* makI_pm, const double* makJ_pm)const noexcept->void;
		auto virtual cptCp(double* cp)const noexcept->void override { cptCpFromPm(cp, *makI()->pm(), *makJ()->pm()); }

		virtual ~Joint() = default;
		explicit Joint(const std::string &name = "joint", Marker *makI = nullptr, Marker *makJ = nullptr, bool active = true) : Constraint(name, makI, makJ, active) {}
		ARIS_DEFINE_BIG_FOUR(Joint);
	};

	class ARIS_API RevoluteJoint final :public Joint{
	public:
		static auto Dim()->Size { return 5; }
		auto virtual dim() const noexcept->Size override { return Dim(); }
		auto virtual locCmI() const noexcept->const double* override;
		auto virtual cptCpFromPm(double *cp, const double *makI_pm, const double *makJ_pm)const noexcept->void override;
		auto virtual cptGlbDmFromPm(double *dm, const double *makI_pm, const double *makJ_pm)const noexcept->void override;

		virtual ~RevoluteJoint() = default;
		explicit RevoluteJoint(const std::string &name = "revolute_joint", Marker *makI = nullptr, Marker *makJ = nullptr);
		ARIS_DEFINE_BIG_FOUR(RevoluteJoint);
	};
	class ARIS_API PrismaticJoint final :public Joint{
	public:
		static auto Dim()->Size { return 5; }
		auto virtual dim() const noexcept->Size override { return Dim(); }
		auto virtual locCmI() const noexcept->const double* override;
		auto virtual cptCpFromPm(double *cp, const double *makI_pm, const double *makJ_pm)const noexcept->void override;
		auto virtual cptGlbDmFromPm(double *dm, const double *makI_pm, const double *makJ_pm)const noexcept->void override;
		
		virtual ~PrismaticJoint() = default;
		explicit PrismaticJoint(const std::string &name = "prismatic_joint", Marker *makI = nullptr, Marker *makJ = nullptr);
		ARIS_DEFINE_BIG_FOUR(PrismaticJoint);
	};
	class ARIS_API UniversalJoint final :public Joint{
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
		ARIS_DECLARE_BIG_FOUR(UniversalJoint);

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};
	class ARIS_API SphericalJoint final :public Joint{
	public:
		static auto Dim()->Size { return 3; }
		auto virtual dim() const noexcept->Size override { return Dim(); }
		auto virtual locCmI() const noexcept->const double* override;
		auto virtual cptCpFromPm(double *cp, const double *makI_pm, const double *makJ_pm)const noexcept->void override;
		auto virtual cptGlbDmFromPm(double *dm, const double *makI_pm, const double *makJ_pm)const noexcept->void override;

		virtual ~SphericalJoint() = default;
		explicit SphericalJoint(const std::string &name = "spherical_joint", Marker *makI = nullptr, Marker *makJ = nullptr);
		ARIS_DEFINE_BIG_FOUR(SphericalJoint);
	};

	class ARIS_API ScrewJoint final :public Joint{
	public:
		static auto Dim()->Size { return 5; }
		auto virtual dim() const noexcept->Size override { return Dim(); }
		auto virtual locCmI() const noexcept->const double* override;
		auto virtual cptCpFromPm(double* cp, const double* makI_pm, const double* makJ_pm)const noexcept->void override;
		auto virtual cptGlbDmFromPm(double* dm, const double* makI_pm, const double* makJ_pm)const noexcept->void override;
		auto pitch()const noexcept->double;
		auto setPitch(double pitch)noexcept->void;

		virtual ~ScrewJoint() = default;
		explicit ScrewJoint(const std::string & name = "screw_joint", Marker * makI = nullptr, Marker * makJ = nullptr, double pitch = 0.0);
		ARIS_DEFINE_BIG_FOUR(ScrewJoint);
	private:
		double loc_cm_i_[30];
		double pitch_{ 0.0 };
	};

	/// @}
}

#endif
