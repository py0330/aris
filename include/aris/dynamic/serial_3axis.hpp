#ifndef ARIS_DYNAMIC_SERIAL_3_AXIS_H_
#define ARIS_DYNAMIC_SERIAL_3_AXIS_H_

#include <array>
#include <aris/dynamic/model_solver.hpp>

namespace aris::dynamic
{
	struct ExternalAxis
	{
		enum AxisType
		{
			RotationalAxis,
			TranslationalAxis
		};

		AxisType type_;
	};
	
	struct Serial3Param
	{
		// DH PARAM //
		double a1{ 0.5 };
		double a2{ 0.5 };
		double a3{ 0.5 };

		// TOOL 0, by default is 321 type
		double tool0_pe[6]{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
		std::string tool0_pe_type;

		// BASE wrt REF, by default is 321 type 
		double base2ref_pe[6]{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
		std::string base2ref_pe_type;

		// inertia vector, size must be 6
		std::vector<std::array<double, 10> > iv_vec;

		// mot friction vector, size must be 6
		std::vector<std::array<double, 3> > mot_frc_vec;

		// external axes
		std::vector<ExternalAxis> external_axes;
	};
	auto createModelSerial3Axis(const Serial3Param &param)->std::unique_ptr<aris::dynamic::Model>;

	class Serial3InverseKinematicSolver :public aris::dynamic::InverseKinematicSolver
	{
	public:
		auto virtual allocateMemory()->void override;
		auto virtual kinPos()->int override;
		auto setWhichRoot(int root_of_0_to_4)->void;
		auto whichRoot()->int;
		auto setPmEE(const double *ee_pos, const double *extnal_axes)->void;
		auto setEulaAngle(const double *eul, const char *type = "321")->void;
		auto setQuaternionAngle(const double *q)->void;
		auto setPqEEAngle(const double *pq)->void;
		auto setPeEEAngle(const double *pe, const char *type = "321")->void;

		virtual ~Serial3InverseKinematicSolver();
		explicit Serial3InverseKinematicSolver(const Serial3Param &param, const std::string &name = "puma_inverse_solver");
		ARIS_DECLARE_BIG_FOUR(Serial3InverseKinematicSolver);

	private:
		struct Imp;
		aris::core::ImpPtr<Imp> imp_;
	};
}

#endif
