#ifndef ARIS_DYNAMIC_MODEL_
#define ARIS_DYNAMIC_MODEL_

#include <vector>
#include <array>
#include <map>
#include <string>
#include <memory>
#include <functional>
#include <algorithm>
#include <deque>
#include <type_traits>

#include <aris_core.h>
#include <aris_dynamic_model_basic.h>
#include <aris_dynamic_model_coordinate.h>
#include <aris_dynamic_model_interaction.h>
#include <aris_dynamic_model_compute.h>

namespace aris
{
	namespace dynamic
	{
		/// @defgroup dynamic_model_group 动力学建模模块
		/// 本模块可以对任意机构的机器人做运动学与动力学计算。
		/// 
		/// 
		/// @{
		///
		class Model :public aris::core::Object
		{
		public:
			static auto Type()->const std::string &{ static const std::string type("Model"); return type; }
			auto virtual type() const->const std::string& override{ return Type(); }
            auto virtual loadXml(const aris::core::XmlElement &xml_ele)->void override;
			auto virtual saveXml(aris::core::XmlElement &xml_ele)const->void override;
			auto time()const->double;
			auto setTime(double time)->void;
			/// @{
			auto calculator()->aris::core::Calculator&;
			auto calculator()const ->const aris::core::Calculator&{ return const_cast<std::decay_t<decltype(*this)> *>(this)->calculator(); }
			auto environment()->aris::dynamic::Environment&;
			auto environment()const ->const aris::dynamic::Environment&{ return const_cast<std::decay_t<decltype(*this)> *>(this)->environment(); }
			auto variablePool()->aris::core::ObjectPool<Variable, Element>&;
			auto variablePool()const->const aris::core::ObjectPool<Variable, Element>&{ return const_cast<std::decay_t<decltype(*this)> *>(this)->variablePool(); }
			auto partPool()->aris::core::ObjectPool<Part, Element>&;
			auto partPool()const->const aris::core::ObjectPool<Part, Element>&{ return const_cast<std::decay_t<decltype(*this)> *>(this)->partPool(); }
			auto jointPool()->aris::core::ObjectPool<Joint, Element>&;
			auto jointPool()const->const aris::core::ObjectPool<Joint, Element>&{ return const_cast<std::decay_t<decltype(*this)> *>(this)->jointPool(); }
			auto motionPool()->aris::core::ObjectPool<Motion, Element>&;
			auto motionPool()const->const aris::core::ObjectPool<Motion, Element>&{ return const_cast<std::decay_t<decltype(*this)> *>(this)->motionPool(); }
			auto generalMotionPool()->aris::core::ObjectPool<GeneralMotion, Element>&;
			auto generalMotionPool()const->const aris::core::ObjectPool<GeneralMotion, Element>&{ return const_cast<std::decay_t<decltype(*this)> *>(this)->generalMotionPool(); }
			auto forcePool()->aris::core::ObjectPool<Force, Element>&;
			auto forcePool()const->const aris::core::ObjectPool<Force, Element>&{ return const_cast<std::decay_t<decltype(*this)> *>(this)->forcePool(); }
			auto solverPool()->aris::core::ObjectPool<Solver, Element>&;
			auto solverPool()const->const aris::core::ObjectPool<Solver, Element>&{ return const_cast<std::decay_t<decltype(*this)> *>(this)->solverPool(); }
			auto simulatorPool()->aris::core::ObjectPool<Simulator, Element>&;
			auto simulatorPool()const->const aris::core::ObjectPool<Simulator, Element>&{ return const_cast<std::decay_t<decltype(*this)> *>(this)->simulatorPool(); }
			auto simResultPool()->aris::core::ObjectPool<SimResult, Element>&;
			auto simResultPool()const->const aris::core::ObjectPool<SimResult, Element>&{ return const_cast<std::decay_t<decltype(*this)> *>(this)->simResultPool(); }
			auto ground()->Part&;
			auto ground()const->const Part&{ return const_cast<std::decay_t<decltype(*this)> *>(this)->ground(); }
			/// @}
			/// @{
			auto addPartByPm(const double*pm, const double *prt_iv = nullptr)->Part&;
			auto addPartByPe(const double*pe, const char* eul_type, const double *prt_iv = nullptr)->Part&;
			auto addPartByPq(const double*pq, const double *prt_iv = nullptr)->Part&;
			auto addRevoluteJoint(Part &first_part, Part &second_part, const double *position, const double *axis)->RevoluteJoint&;
			auto addPrismaticJoint(Part &first_part, Part &second_part, const double *position, const double *axis)->PrismaticJoint&;
			auto addUniversalJoint(Part &first_part, Part &second_part, const double *position, const double *first_axis, const double *second_axis)->UniversalJoint&;
			auto addSphericalJoint(Part &first_part, Part &second_part, const double *position)->SphericalJoint&;
			auto addMotion(Joint &joint)->Motion&;
			auto addMotion()->Motion&;
			auto addGeneralMotionByPm(Part &end_effector, Coordinate &reference, const double* pm)->GeneralMotion&;
			auto addGeneralMotionByPe(Part &end_effector, Coordinate &reference, const double* pe, const char* eul_type)->GeneralMotion&;
			auto addGeneralMotionByPq(Part &end_effector, Coordinate &reference, const double* pq)->GeneralMotion&;
			/// @}
			virtual ~Model();
			explicit Model(const std::string &name = "model");
			Model(const Model &);
			Model(Model &&);
			Model &operator=(const Model &);
			Model &operator=(Model &&);


		private:
			struct Imp;
			aris::core::ImpPtr<Imp> imp_;
			friend class Motion;
		};
		/// @}
	}
}

#endif
