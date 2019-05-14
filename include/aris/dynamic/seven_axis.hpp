#ifndef ARIS_DYNAMIC_SEVEN_AXIS_H_
#define ARIS_DYNAMIC_SEVEN_AXIS_H_

#include <array>
#include <aris/dynamic/model_solver.hpp>

namespace aris::dynamic
{
	/// @defgroup dynamic_model_group 动力学建模模块
	/// @{
	///
	struct SevenAxisParam
	{
		// DH PARAM //
		double d1{ 0.0 };
		double d3{ 0.0 };
		double d5{ 0.0 };

		// TOOL 0, by default is 321 type
		double tool0_pe[6]{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
		std::string tool0_pe_type;

		// BASE wrt REF, by default is 321 type 
		double base2ref_pe[6]{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
		std::string base2ref_pe_type = "321";

		// inertia vector, size must be 7
		std::vector<std::array<double, 10> > iv_vec;

		// mot friction vector, size must be 7
		std::vector<std::array<double, 3> > mot_frc_vec;
	};
	auto createModelSevenAxis(const SevenAxisParam &param)->std::unique_ptr<aris::dynamic::Model>;

	class SevenAxisInverseKinematicSolver :public aris::dynamic::InverseKinematicSolver
	{
	public:
		auto virtual saveXml(aris::core::XmlElement &xml_ele) const->void override;
		auto virtual loadXml(const aris::core::XmlElement &xml_ele)->void override;		
		auto virtual allocateMemory()->void override;
		auto virtual kinPos()->int override;
		auto setWhichRoot(int root_of_0_to_7)->void;
		auto setAxisAngle(double axis_angle)->void;

		virtual ~SevenAxisInverseKinematicSolver() = default;
		explicit SevenAxisInverseKinematicSolver(const std::string &name = "seven_axis_inverse_solver");
		ARIS_REGISTER_TYPE(SevenAxisInverseKinematicSolver);
		ARIS_DECLARE_BIG_FOUR(SevenAxisInverseKinematicSolver);

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};
	///
	/// @}
}

#endif
