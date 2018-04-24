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
		///
		/// \page dynamic_model_page 机器人建模
		/// 本文以scara机器人为例，介绍aris的建模以及运动学和动力学计算方法。
		/// \section Scara机器人
		///
		/// 如果熟悉C++及机构建模，建议直接看代码 \ref demo_model_scara/main.cpp "demo_model_scara"
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
		/// \dot 四杆机器人拓扑
		/// digraph G{
		/// rankdir=LR;
		/// ground[label = "link1(ground)"];
		/// link1[label = "link2"];
		/// link2[label = "link3"];
		/// link3[label = "link4"];
		/// ground->link1[label = "R1"];
		/// link1->link2[label = "R2"];
		/// link2->link3[label = "R3"];
		/// link3->ground[label = "R4"];
		/// }
		/// \enddot
		///
		/// \dot 树形拓扑
		/// digraph G{
		/// rankdir=LR;
		/// ground[label = "link1(ground)"];
		/// link1[label = "link2"];
		/// link2[label = "link3"];
		/// link3[label = "link4"];
		/// link4[label = "link2"];
		/// link5[label = "link3"];
		/// link6[label = "link4"];
		/// link7[label = "link2"];
		/// link8[label = "link3"];
		/// link9[label = "link4"];
		/// ground->link1[label = "R1"];
		/// link1->link2[label = "R2"];
		/// link2->link3[label = "R3"];
		/// link1->link4[label = "R4"];
		/// link4->link5[label = "R5"];
		/// link5->link6[label = "R6"];
		/// link4->link7[label = "R7"];
		/// link4->link8[label = "R8"];
		/// link5->link9[label = "R9"];
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
		/// 惯性向量 | [2 , 0 , 0 , 0 , 1 , 1 , 10 , 0 , 0 , 0] | [2 , 0 , 0 , 0 , 1 , 1 , 10 , 0 , 0 , 0] | [2 , 0 , 0 , 0 , 1 , 1 , 10 , 0 , 0 , 0] | [2 , 0 , 0 , 0 , 1 , 1 , 10 , 0 , 0 , 0] |
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
		/// Model 目前可以通过位置与欧拉角，位置与四元数、位姿矩阵等3种方式添加杆件，对应以下函数：
		/// + Model::addPartByPe : 指定惯性坐标系的位置与欧拉角，以及惯性向量
		/// + Model::addPartByPq : 指定惯性坐标系的位置与四元数，以及惯性向量
		/// + Model::addPartByPm : 指定惯性坐标系的位姿矩阵（4x4），以及惯性向量
		///
		/// Model 目前可以添加以下4种运动副：
		/// + \ref RevoluteJoint "R副" : 通过 Model::addRevoluteJoint 函数添加，需要指定转动副的位置和轴线方向
		/// + \ref PrismaticJoint "P副" : 通过 Model::addPrismaticJoint 函数添加，需要指定移动副的位置以及轴线方向
		/// + \ref UniversalJoint "U副" : 通过 Model::addUniversalJoint 函数添加，需要依次指定第一根转轴、第二根转轴以及两根转轴的交点 
		/// + \ref SphericalJoint "S副" : 通过 Model::addSphericalJoint 函数添加，需要指定球铰的位置 
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
		/// \subsection 位置反解
		/// 
		/// 机器人位置逆解指根据末端输出求解驱动输入，以下函数等效：
		/// + GeneralMotion::setMpe : 设置末端位置与欧拉角
		/// + GeneralMotion::setMpq : 设置末端位置与四元数
		/// + GeneralMotion::setMpm : 设置末端位姿矩阵
		///
		/// 之后调用求解器的 InverseKinematicSolver::kinPos 函数来求解，该函数需要进行迭代，且跟初值相关，返回值代表是否计算成功。
		/// 
		/// 计算完成后，可以调用 Motion::mp 函数来获取输入的位置。
		///
		/// 以下代码展示求逆解的过程：
		/// \snippet demo_model_scara/main.cpp Inverse_Position
		///
		/// \subsection 速度反解
		/// 
		/// 速度模型的求解与位置模型相似， aris 提供2种速度表达方式，角速度与线速度(va)，速度螺旋(vs)，分别对应以下函数：
		/// + GeneralMotion::setMva
		/// + GeneralMotion::setMvs
		///
		/// InverseKinematicSolver::kinVel 函数无需迭代，直接可以求出输入输出的速度关系。
		/// 求解完毕后可以使用 Motion::mv 函数来查看输入的速度。
		///
		/// 以下代码展示求逆解的过程：
		/// \snippet demo_model_scara/main.cpp Inverse_Velocity
		///
		/// \subsection 动力学反解
		///
		/// 机器人动力学是指根据输入的加速度，来求解驱动力，事实上，很多时候还关心输出的加速度。
		/// InverseDynamicSolver::dynAccAndFce 函数可以求出两者。
		///
		/// 以下代码展示动力学求解的过程：
		/// \snippet demo_model_scara/main.cpp Inverse_Dynamic
		///
		/// \subsection 用Adams验证
		/// 为了验证计算结果是否正确，可以使用其他软件对结构进行验证，比如Adams。aris提供插件，让用户可以直接保存成Adams模型。
		/// 
		/// 以下代码展示使用Adams验证的过程：
		/// \snippet demo_model_scara/main.cpp Verify_By_Adams
		///
		/// 注意，这里只验证单点的输入位置、速度、驱动力，因此用户只需关注adams中0时刻的仿真数据。
		/// 
		/// \subsection 仿真整段轨迹
		/// 以下代码展示使用Adams验证的过程：
		/// \snippet demo_model_scara/main.cpp Simulate
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
