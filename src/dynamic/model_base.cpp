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
#include <ios>

#include "aris/core/core.hpp"
#include "aris/dynamic/model_base.hpp"
#include "aris/dynamic/kinematics.hpp"
#include "aris/dynamic/math_matrix.hpp"

namespace aris::dynamic{
	auto ModelBase::getInputPos(double* pos)const noexcept->void {
		for (int i = 0; i < inputPosSize(); ++i)
			pos[i] = inputPosAt(i);
	}
	auto ModelBase::setInputPos(const double* pos)noexcept->void {
		for (int i = 0; i < inputPosSize(); ++i)
			setInputPosAt(i, pos[i]);
	}
	auto ModelBase::getInputVel(double* vel)const noexcept->void {
		for (int i = 0; i < inputVelSize(); ++i) 
			vel[i] = inputVelAt(i);
	}
	auto ModelBase::setInputVel(const double* vel)noexcept->void {
		for (int i = 0; i < inputVelSize(); ++i) 
			setInputVelAt(i, vel[i]);
	}
	auto ModelBase::getInputAcc(double* acc)const noexcept->void {
		for (int i = 0; i < inputAccSize(); ++i) 
			acc[i] = inputAccAt(i);
	}
	auto ModelBase::setInputAcc(const double* acc)noexcept->void {
		for (int i = 0; i < inputAccSize(); ++i) 
			setInputAccAt(i, acc[i]);
	}
	auto ModelBase::getInputFce(double* fce)const noexcept->void {
		for (int i = 0; i < inputFceSize(); ++i) 
			fce[i] = inputFceAt(i);
	}
	auto ModelBase::setInputFce(const double* fce)noexcept->void {
		for (int i = 0; i < inputFceSize(); ++i) 
			setInputFceAt(i, fce[i]);
	}
	auto ModelBase::getOutputPos(double* pos)const noexcept->void {
		Size idx = 0;
		for (int i = 0; i < eeSize(); ++i) {
			auto s = s_pos_type_size(eePosTypes()[i]);
			aris::dynamic::s_vc(s, outputPosAt(i), pos + idx);
			idx += s;
		}
	}
	auto ModelBase::setOutputPos(const double* pos)noexcept->void {
		Size idx = 0;
		for (int i = 0; i < eeSize(); ++i) {
			auto s = s_pos_type_size(eePosTypes()[i]);
			setOutputPosAt(i, pos + idx);
			idx += s;
		}
	}
	auto ModelBase::getOutputVel(double* vel)const noexcept->void {
		Size idx = 0;
		for (int i = 0; i < eeSize(); ++i) {
			auto s = s_vel_type_size(eeVelTypes()[i]);
			aris::dynamic::s_vc(s, outputVelAt(i), vel + idx);
			idx += s;
		}
	}
	auto ModelBase::setOutputVel(const double* vel)noexcept->void {
		Size idx = 0;
		for (int i = 0; i < eeSize(); ++i) {
			auto s = s_vel_type_size(eeVelTypes()[i]);
			setOutputVelAt(i, vel + idx);
			idx += s;
		}
	}
	auto ModelBase::getOutputAcc(double* acc)const noexcept->void {
		Size idx = 0;
		for (int i = 0; i < eeSize(); ++i) {
			auto s = s_acc_type_size(eeAccTypes()[i]);
			aris::dynamic::s_vc(s, outputAccAt(i), acc + idx);
			idx += s;
		}
	}
	auto ModelBase::setOutputAcc(const double* acc)noexcept->void {
		Size idx = 0;
		for (int i = 0; i < eeSize(); ++i) {
			auto s = s_acc_type_size(eeAccTypes()[i]);
			setOutputAccAt(i, acc + idx);
			idx += s;
		}
	}
	auto ModelBase::getOutputFce(double* fce)const noexcept->void {
		Size idx = 0;
		for (int i = 0; i < eeSize(); ++i) {
			auto s = s_fce_type_size(eeFceTypes()[i]);
			aris::dynamic::s_vc(s, outputFceAt(i), fce + idx);
			idx += s;
		}
	}
	auto ModelBase::setOutputFce(const double* fce)noexcept->void {
		Size idx = 0;
		for (int i = 0; i < eeSize(); ++i) {
			auto s = s_fce_type_size(eeFceTypes()[i]);
			setOutputFceAt(i, fce + idx);
			idx += s;
		}
	}


	ARIS_REGISTRATION
	{

	}
}
