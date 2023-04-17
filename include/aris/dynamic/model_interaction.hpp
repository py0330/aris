#ifndef ARIS_DYNAMIC_MODEL_INTERACTION_H_
#define ARIS_DYNAMIC_MODEL_INTERACTION_H_

#include <cmath>

#include <aris/dynamic/model_coordinate.hpp>

namespace aris::dynamic{
	/// @defgroup dynamic_model_group 动力学建模模块
	/// @{
	///
	class ARIS_API Interaction :public DynEle{
	public:
		auto setMakI(Marker* mak_i) noexcept->void { makI_ = mak_i; }
		auto makI() noexcept->Marker* { return makI_; }
		auto makI() const noexcept->const Marker* { return makI_; }
		auto setMakJ(Marker* mak_j) noexcept->void { makI_ = mak_j; }
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
		ARIS_DEFINE_BIG_FOUR(Interaction);

	private:
		Marker *makI_{ nullptr }, *makJ_{nullptr};
		std::string prt_name_M_, prt_name_N_, mak_name_I_, mak_name_J_;
		friend class Model;
	};
	class ARIS_API Constraint :public Interaction{
	public:
		auto virtual dim() const->Size = 0;
		auto virtual locCmI() const->const double* = 0;
		auto virtual cptCp(double* cp)const noexcept->void {}
		auto virtual cptCv(double *cv)const noexcept->void { std::fill_n(cv, dim(), 0.0); }
		auto virtual cptCvDiff(double *cv)const noexcept->void;
		auto virtual cptCa(double *ca)const noexcept->void;
		auto virtual cptGlbDmFromPm(double *dm, const double *makI_pm, const double *makJ_pm)const noexcept->void{
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
		auto virtual cptGlbCmFromPm(double *cmI, double *cmJ, const double *makI_pm, const double *makJ_pm)const noexcept->void	{
			s_tf_n(dim(), makI_pm, locCmI(), cmI);
			s_mi(6, dim(), cmI, cmJ);
		}
		template<typename CMI_TYPE, typename CMJ_TYPE>
		auto cptCm(const Coordinate &relative_to_I, double *cmI, CMI_TYPE cmi_type, const Coordinate &relative_to_J, double *cmJ, CMJ_TYPE cmj_type)const noexcept->void{
			double pm[16];
			makI()->getPm(relative_to_I, pm);
			s_tf_n(dim(), pm, locCmI(), dim(), cmI, cmi_type);

			makI()->getPm(relative_to_J, pm);
			s_tf_n(dim(), -1.0, pm, locCmI(), dim(), cmJ, cmj_type);
		}
		template<typename CMI_TYPE, typename CMJ_TYPE>
		auto cptCm(const Coordinate &relative_to, double *cmI, CMI_TYPE cmi_type, double *cmJ, CMJ_TYPE cmj_type)const noexcept->void{
			double pm[16];
			makI()->getPm(relative_to, pm);
			s_tf_n(dim(), pm, locCmI(), dim(), cmI, cmi_type);

			s_mi(6, dim(), cmI, cmi_type, cmJ, cmj_type);
		}
		template<typename CMI_TYPE, typename CMJ_TYPE>
		auto cptPrtCm(double *cmI, CMI_TYPE cmi_type, double *cmJ, CMJ_TYPE cmj_type)const noexcept->void{
			cptCm(makI()->fatherPart(), cmI, cmi_type, makJ()->fatherPart(), cmJ, cmj_type);
		}
		auto cptPrtCm(double *cmI, double *cmJ)->void { cptPrtCm(cmI, dim(), cmJ, dim()); }
		template<typename CMI_TYPE, typename CMJ_TYPE>
		auto cptGlbCm(double *cmI, CMI_TYPE cmi_type, double *cmJ, CMJ_TYPE cmj_type)const noexcept->void{
			s_tf_n(dim(), *makI()->pm(), locCmI(), dim(), cmI, cmi_type);
			s_mi(6, dim(), cmI, cmi_type, cmJ, cmj_type);
		}
		auto cptGlbCm(double *cmI, double *cmJ)const noexcept->void{ cptGlbCm(cmI, dim(), cmJ, dim()); }
		auto cf() const noexcept->const double*;
		auto setCf(const double *cf) noexcept->void;

		virtual ~Constraint();
		explicit Constraint(const std::string &name = "constraint", Marker *makI = nullptr, Marker *makJ = nullptr, bool is_active = true);
		ARIS_DECLARE_BIG_FOUR(Constraint);

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
		friend class Motion;
		friend class GeneralMotion;
	};

	/// @}
}

#endif
