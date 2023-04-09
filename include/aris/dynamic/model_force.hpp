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
		auto virtual cptGlbFs(double *fsI, double *fsJ)const noexcept->void = 0;

		virtual ~Force() = default;
		explicit Force(const std::string &name = "force", Marker *makI = nullptr, Marker *makJ = nullptr, bool active = true) :Interaction(name, makI, makJ, active) {}
		ARIS_DEFINE_BIG_FOUR(Force);
	};

	class ARIS_API GeneralForce final :public Force{
	public:
		auto virtual cptGlbFs(double *fsI, double *fsJ)const noexcept->void override { s_vc(6, fce_value_, fsI); s_vi(6, fce_value_, fsJ); }
		auto setFce(const double *value) noexcept->void { std::copy(value, value + 6, fce_value_); }
		auto fce()const noexcept->const double* { return fce_value_; }

		virtual ~GeneralForce() = default;
		explicit GeneralForce(const std::string &name = "general_force", Marker *makI = nullptr, Marker *makJ = nullptr) : Force(name, makI, makJ) {};
		ARIS_DEFINE_BIG_FOUR(GeneralForce);

	private:
		double fce_value_[6]{ 0 };
	};
	class ARIS_API SingleComponentForce final :public Force{
	public:
		auto virtual cptGlbFs(double *fsI, double *fsJ)const noexcept->void override;
		auto setComponentAxis(Size id) noexcept->void { component_axis_ = id; }
		auto componentAxis()const noexcept->Size { return component_axis_; }
		auto setFce(double value) noexcept->void { std::fill_n(fce_value_, 6, 0); fce_value_[component_axis_] = value; }
		auto setFce(double value, Size componentID) noexcept->void { this->component_axis_ = componentID; setFce(value); }
		auto fce()const noexcept->double { return fce_value_[component_axis_]; }

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
