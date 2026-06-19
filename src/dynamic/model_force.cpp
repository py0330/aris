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
	auto GeneralForce::cptGlbFs(double *fsI, double *fsJ)const noexcept->void { 
		cptGlbFsFromPm(nullptr, nullptr, nullptr, nullptr, fce(), fsI, fsJ);
	}
	auto GeneralForce::cptGlbFsFromPm(const double *pmI, const double *pmJ, const double *vsI, const double *vsJ, const double *fce_in, double *fsI, double *fsJ)const noexcept->void{
		s_vc(6, fce_in, fsI); 
		s_vi(6, fce_in, fsJ);
	}

	auto SingleComponentForce::cptGlbFsFromPm(const double *pmI, const double *pmJ, const double *vsI, const double *vsJ, const double *fce_in, double *fsI, double *fsJ)const noexcept->void {
		// double fs[6]{0,0,0,0,0,0};
		// fs[component_axis_] = fce_in[0];
		// s_tf(pmI, fs, fsI);
		// s_vi(6, fsI, fsJ);

		// 以下来优化计算 //
		// tmf = [rm (3x3),  pp x rm (3x3); O (3x3), rm (3x3)]

		if(component_axis_ < 3){
			fsI[0] = pmI[0 + component_axis_] * fce_in[0];
			fsI[1] = pmI[4 + component_axis_] * fce_in[0];
			fsI[2] = pmI[8 + component_axis_] * fce_in[0];
			fsI[3] = -pmI[11] * fsI[1] + pmI[7] * fsI[2];
			fsI[4] = pmI[11] * fsI[0] - pmI[3] * fsI[2];
			fsI[5] = -pmI[7] * fsI[0] + pmI[3] * fsI[1];
			s_vi(6, fsI, fsJ);
		}
		else{
			fsI[0] = 0;
			fsI[1] = 0;
			fsI[2] = 0;
			fsI[3] = pmI[component_axis_ - 3] * fce_in[0];
			fsI[4] = pmI[component_axis_ + 1] * fce_in[0];
			fsI[5] = pmI[component_axis_ + 5] * fce_in[0];
			s_vi(6, fsI, fsJ);
		}


	}
	auto SingleComponentForce::cptGlbFs(double *fsI, double *fsJ)const noexcept->void{
		cptGlbFsFromPm(*makI()->pm(), nullptr, nullptr, nullptr, fce(), fsI, fsJ);
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
