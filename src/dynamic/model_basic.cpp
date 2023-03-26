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
#include "aris/dynamic/matrix.hpp"
#include "aris/dynamic/screw.hpp"
#include "aris/dynamic/model.hpp"

namespace aris::dynamic{
	auto MatrixVariable::fromString(std::string_view str)->void {
		static aris::core::Calculator c;
		static int i = 0;
		if (i == 0){
			c.addVariable("PI", "Number", double(3.141592653));
			i = 1;
		}

		auto mat = c.calculateExpression(std::string("Matrix({") + std::string(str) + "})").second;
		data() = std::any_cast<const aris::core::Matrix&>(mat);
	}
	auto DynEle::activate(bool active)noexcept->void { active_ = active; }

	ARIS_REGISTRATION
	{
		aris::core::class_<DynEle>("DynEle")
			.prop("name", &DynEle::setName, &DynEle::name)
			.prop("active", &DynEle::activate, &DynEle::active)
			;

		auto getGravity =[](Environment *env)->aris::core::Matrix{
			return aris::core::Matrix(1, 6, env->gravity());
		};
		auto setGravity = [](Environment *env, aris::core::Matrix gra)->void{
			env->setGravity(gra.data());
		};

		aris::core::class_<Environment>("Environment")
			.prop("gravity", &setGravity, &getGravity)
			;

		auto variableToText = [](Variable *b)->std::string{
			return b->toString();
		};
		auto variableFromText = [](Variable *b, std::string_view str)->void{
			b->fromString(str);
		};
		aris::core::class_<Variable>("Variable")
			.textMethod(variableToText, variableFromText)
			.prop("name", &Variable::setName, &Variable::name)
			;

		aris::core::class_<MatrixVariable>("MatrixVariable")
			.inherit<Variable>()
			;

		aris::core::class_<StringVariable>("StringVariable")
			.inherit<Variable>()
			;

		aris::core::class_<BoolVariable>("BoolVariable")
			.inherit<Variable>()
			;
	}
}
