#ifndef ARIS_DYNAMIC_MODEL_FORCE_H_
#define ARIS_DYNAMIC_MODEL_FORCE_H_

#include <cmath>
#include <aris/dynamic/model_interaction.hpp>

namespace aris::dynamic {
	/// @defgroup dynamic_model_group 动力学建模模块
	/// @{
	///
	class ARIS_API Force :public Interaction{
	public:
		auto virtual cptGlbFs(double *fsI, double *fsJ)const noexcept->void { cptGlbFsFromPm(*makI()->pm(), *makJ()->pm(), makI()->vs(), makJ()->vs(), fce(), fsI, fsJ); };
		auto virtual cptGlbFsFromPm(const double *pmI, const double *pmJ, const double *vsI, const double *vsJ, const double *fce_in, double *fsI, double *fsJ)const noexcept->void = 0;
		auto virtual dim()const noexcept->Size = 0; // fce的维度，真正计算时，都会转为6维的力
		auto virtual fce()const noexcept->const double* = 0;
		auto virtual setFce(const double *value) noexcept->void	= 0;

		virtual ~Force() = default;
		explicit Force(const std::string &name = "force", Marker *makI = nullptr, Marker *makJ = nullptr, bool active = true) :Interaction(name, makI, makJ, active) {}
		ARIS_DEFINE_BIG_FOUR(Force);
	};

	class ARIS_API GeneralForce final :public Force{
	public:
		auto virtual cptGlbFs(double *fsI, double *fsJ)const noexcept->void override;
		auto virtual cptGlbFsFromPm(const double *pmI, const double *pmJ, const double *vsI, const double *vsJ, const double *fce_in, double *fsI, double *fsJ)const noexcept->void override;
		auto virtual dim()const noexcept->Size override{ return 6; } // fce的维度，真正计算时，都会转为6维的力
		auto virtual fce()const noexcept->const double* override{ return fce_value_; }
		auto virtual setFce(const double *value) noexcept->void override { std::copy(value, value + 6, fce_value_); }
		
		virtual ~GeneralForce() = default;
		explicit GeneralForce(const std::string &name = "general_force", Marker *makI = nullptr, Marker *makJ = nullptr) : Force(name, makI, makJ) {};
		ARIS_DEFINE_BIG_FOUR(GeneralForce);

	private:
		double fce_value_[6]{ 0 };
	};
	class ARIS_API SingleComponentForce final :public Force{
	public:
		auto virtual cptGlbFs(double *fsI, double *fsJ)const noexcept->void override;
		auto virtual cptGlbFsFromPm(const double *pmI, const double *pmJ, const double *vsI, const double *vsJ, const double *fce_in, double *fsI, double *fsJ)const noexcept->void override;
		auto virtual dim()const noexcept->Size override{ return 1; } // fce的维度，真正计算时，都会转为6维的力
		auto virtual fce()const noexcept->const double* override{ return fce_value_; }
		auto virtual setFce(const double *value) noexcept->void override { fce_value_[0] = value[0]; }

		auto setComponentAxis(Size id) noexcept->void { component_axis_ = id; }
		auto componentAxis()const noexcept->Size { return component_axis_; }

		virtual ~SingleComponentForce() = default;
		explicit SingleComponentForce(const std::string &name = "single_component_force", Marker *makI = nullptr, Marker *makJ = nullptr, Size componentID = 0);
		ARIS_DEFINE_BIG_FOUR(SingleComponentForce);

	private:
		Size component_axis_;
		double fce_value_[6]{ 0 };
	};

	/// @}
}

#endif
