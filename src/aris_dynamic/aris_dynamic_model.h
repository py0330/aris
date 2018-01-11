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
		/// \page dynamic_model_page 机器人建模
		/// 本文以scara机器人为例，介绍aris的建模以及运动学和动力学计算方法。
		/// \section Scara机器人
		///
		/// \subsection 机器人参数定义
		/// Scara机器人一般为4轴，为RRPR机构，简易机构与拓扑图如下：
		/// 
		/// ![Scara Robot](demo_model_scara/scara.png "Scara Robot")
		///
		/// \dot Scara机器人机构拓扑
		/// digraph G{
		/// rankdir=LR;
		/// ground[label = "ground"];
		/// link1[label = "link1"];
		/// link2[label = "link2"];
		/// link3[label = "link3"];
		/// link4[label = "link4"];
		/// ground->link1[label = "R1"];
		/// link1->link2[label = "R2"];
		/// link2->link3[label = "P3"];
		/// link3->link4[label = "R4"];
		/// }
		/// \enddot
		///
		/// 各个关节的位置和运动方向如下：
		/// 关节 | R1 | R2 | P3 | R4 |
		/// --------: | : -------- : | : ----- : | : ----- : | : ----------- : |
		/// 轴线位置 | [0 , 0 , 0] | [1 , 0 , 0] | [0 , 0 , 1] | [1 , 1 , 0] |
		/// 轴线方向 | [0 , 0 , 1] | [0 , 0 , 1] | [0 , 0 , 1] | [0 , 0 , 1] |
		///
		/// 假设该机器人各个杆件参数如下：
		/// 杆件 | link1 | link2 | link3 | link4 |
		/// --------: | : -------- : | : ----- : | : ----- : | : ----------- : |
		/// 位置与313欧拉角 | [0 , 0 , 0 , 0 , 0 , 0] | [1 , 0 , 0 , PI/2 , 0 , 0] | [ 1 , 1 , 0 , PI/2 , 0 , 0] | [1 , 1 , 0 , PI/2 , 0 , 0] |
		/// 惯性向量 | [1 , 0 , 0 , 0 , 1 , 0.5 , 0.5 , 0 , 0 , 0] | [1 , 0 , 0 , 0 , 1 , 0.5 , 0.5 , 0 , 0 , 0] | [1 , 0 , 0 , 0 , 1 , 0.5 , 0.5 , 0 , 0 , 0] | [1 , 0 , 0 , 0 , 1 , 0.5 , 0.5 , 0 , 0 , 0] |
		///
		/// 其中位置与欧拉角是杆件的惯性坐标系（不一定是质心坐标系），惯性向量为杆件在该坐标系中的惯性参数，10维，定义如下：
		/// 
		/// \dot aris中的惯量定义
		/// digraph html{
		/// abc [shape=none, margin=0, label=<
		/// <TABLE BORDER="0" CELLBORDER="1" CELLSPACING="0" CELLPADDING="4">
		/// <TR>
		///    <TD>m</TD>
		///    <TD>cx</TD>
		///    <TD>cy</TD>
		///    <TD>cz</TD>
		///    <TD>Ixx</TD>
		///    <TD>Iyy</TD>
		///    <TD>Izz</TD>
		///    <TD>Ixy</TD>
		///    <TD>Ixz</TD>
		///    <TD>Iyz</TD>
		/// </TR>
		/// <TR>
		///    <TD>mass</TD>
		///    <TD COLSPAN="3">m dot center of mass</TD>
		///    <TD COLSPAN="6">inertia</TD>
		/// </TR>
		/// </TABLE> > ];
		/// }
		/// \enddot
		///
		/// 以下代码定义了该SCARA机构的参数：
		///
		/// \snippet demo_model_scara/main.cpp Parameter
		/// 
		/// \subsection 机器人建模
		///
		/// Model 目前可以通过以下方式添加杆件：
		/// + Model::addPartByPe : 需指定惯性坐标系的位置与欧拉角，以及惯性向量
		/// + Model::addPartByPq : 需指定惯性坐标系的位置与四元数，以及惯性向量
		/// + Model::addPartByPm : 需指定惯性坐标系的位姿矩阵（4x4），以及惯性向量
		///
		/// Model 目前可以添加以下4种运动副：
		/// + 转动副（R副）:通过 Model::addRevoluteJoint 函数添加，需要指定转动副的位置和轴线方向
		/// + 移动副（P副）:通过 Model::addPrismaticJoint 函数添加，需要指定移动副的位置以及轴线方向
		/// + 万向节（虎克铰、U副）: 通过 Model::addUniversalJoint 函数添加，需要依次指定第一根转轴、第二根转轴以及两根转轴的交点 
		/// + 球铰（S副）: 通过 Model::addSphericalJoint 函数添加，需要指定球铰的位置 
		///
		/// 驱动通过 Model::addMotion 函数在转动副或移动副上添加。
		///  
		/// 末端通过可以用以下3种方式添加：
		/// + Model::addGeneralMotionByPe 需要指定末端所在的杆件，相对的坐标系或杆件（一般为地面），当前的末端位置与欧拉角
		/// + Model::addGeneralMotionByPq 需要指定末端所在的杆件，相对的坐标系或杆件（一般为地面），当前的末端位置与四元数
		/// + Model::addGeneralMotionByPm 需要指定末端所在的杆件，相对的坐标系或杆件（一般为地面），当前的末端位置与位姿矩阵
		///
		/// 以下代码为上文中Scara机器人的建模过程：
		/// \snippet demo_model_scara/main.cpp Modeling
		///
		/// \subsection 添加求解器
		/// aris的基础求解器（ aris::dynamic::UniversalSolver ）可以求解任意机构的运动学和动力学，
		/// 但是该求解器需要用户手动管理机构拓扑，为此还提供其他四种求解器：
		/// + 运动学逆解 aris::dynamic::InverseKinematicSolver
		/// + 运动学正解 aris::dynamic::ForwardKinematicSolver
		/// + 动力学逆解 aris::dynamic::InverseDynamicSolver
		/// + 动力学正解 aris::dynamic::ForwardDynamicSolver
		/// 
		/// \snippet demo_model_scara/main.cpp Solver
		/// 
		/// \subsection 求解位置反解
		/// 
		/// 机器人运动学逆解指根据末端输出求解驱动输入，因此首先设置末端的输出位姿，有如下函数：
		/// + 
		/// +
		/// 
		/// 以下代码展示求逆解的过程：
		/// \snippet demo_model_scara/main.cpp Inverse_Position
		///
		/// \subsection 求解位置反解
		/// 
		/// 以下代码展示求逆解的过程：
		/// \snippet demo_model_scara/main.cpp Inverse_Velocity
		///
		/// 




		
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
