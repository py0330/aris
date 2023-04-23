#include <cmath>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <algorithm>
#include <limits>
#include <sstream>
#include <regex>
#include <limits>
#include <type_traits>

#include "aris/core/reflection.hpp"

#include "aris/dynamic/model_force.hpp"

namespace aris::dynamic{

	auto SingleComponentForce::cptGlbFs(double *fsI, double *fsJ)const noexcept->void
	{
		s_tf(*makI()->prtPm(), fce_value_, fsJ);
		s_tf(*makI()->fatherPart().pm(), fsJ, fsI);
		s_vi(6, fsI, fsJ);
	}
	SingleComponentForce::SingleComponentForce(const std::string &name, Marker* makI, Marker* makJ, Size componentID) : Force(name, makI, makJ), component_axis_(componentID) {}

	ARIS_REGISTRATION{
		aris::core::class_<Force>("Force")
			.inherit<aris::dynamic::Interaction>()
			;

		aris::core::class_<GeneralForce>("GeneralForce")
			.inherit<aris::dynamic::Force>()
			;

		aris::core::class_<SingleComponentForce>("SingleComponentForce")
			.inherit<aris::dynamic::Force>()
			.prop("component", &SingleComponentForce::setComponentAxis, &SingleComponentForce::componentAxis)
			;

	}
}
