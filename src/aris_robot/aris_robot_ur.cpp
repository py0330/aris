#include <algorithm>

#include"aris_robot_ur.h"

using namespace aris::dynamic;

namespace aris
{
	namespace robot
	{
		auto create_ur5()->std::unique_ptr<aris::dynamic::Model>
		{
			std::unique_ptr<aris::dynamic::Model> rbt = std::make_unique<aris::dynamic::Model>("ur5");

			// 设置重力 //
			const double gravity[6]{ 0.0,0.0,-9.8,0.0,0.0,0.0 };
			rbt->environment().setGravity(gravity);

			// add parts //
			// iv : inertia vector : m cx cy cz Ixx Iyy Izz Ixy Ixz Iyz
			// pm : 4x4 pose matrix
			// pe : 6x1 position and euler angle
			double iv[10], pm[16];

			const double p1_pe[6]{ 0.0, 0.00193, 0.089159 - 0.02561, 0, 0, 0 };
			const double p1_iv[10]{ 3.7, 0, 0, 0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0 };
			s_pe2pm(p1_pe, pm, "321");
			s_iv2iv(pm, p1_iv, iv);
			auto &p1 = rbt->partPool().add<Part>("L1", iv);

			const double p2_pe[6]{ 0.2125, -0.024201 + 0.13585, 0.089159, 0.0, 0.0, 0.0 };
			const double p2_iv[10]{ 8.393, 0, 0, 0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0 };
			s_pe2pm(p2_pe, pm, "321");
			s_iv2iv(pm, p2_iv, iv);
			auto &p2 = rbt->partPool().add<Part>("L2", iv);

			const double p3_pe[6]{ 0.425 + 0.110949, 0.13585 - 0.1197, 0.089159 + 0.01634, 0, 0, 0 };
			const double p3_iv[10]{ 2.275, 0, 0, 0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0 };
			s_pe2pm(p3_pe, pm, "321");
			s_iv2iv(pm, p3_iv, iv);
			auto &p3 = rbt->partPool().add<Part>("L3", iv);

			const double p4_pe[6]{ 0.425 + 0.39225, 0.13585 - 0.1197 + 0.093, 0.089159, 0, 0, 0 };
			const double p4_iv[10]{ 1.219, 0, 0, 0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0 };
			s_pe2pm(p4_pe, pm, "321");
			s_iv2iv(pm, p4_iv, iv);
			auto &p4 = rbt->partPool().add<Part>("L4", iv);

			const double p5_pe[6]{ 0.425 + 0.39225, 0.13585 - 0.1197 + 0.093, 0.089159 - 0.09465, 0, 0, 0 };
			const double p5_iv[10]{ 1.219, 0, 0, 0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0 };
			s_pe2pm(p5_pe, pm, "321");
			s_iv2iv(pm, p5_iv, iv);
			auto &p5 = rbt->partPool().add<Part>("L5", iv);

			const double p6_pe[6]{ 0.425 + 0.39225, 0.13585 - 0.1197 + 0.093, 0.089159 - 0.09465, 0, 0, 0 };
			const double p6_iv[10]{ 0.1879, 0, 0, 0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0 };
			s_pe2pm(p6_pe, pm, "321");
			s_iv2iv(pm, p6_iv, iv);
			auto &p6 = rbt->partPool().add<Part>("L6", iv);

			// add solid //
			rbt->ground().geometryPool().add<aris::dynamic::FileGeometry>("file_geometry", ARIS_INSTALL_PATH + std::string("/resource/aris_robot/ur5/ground.DWG")
				, s_pe2pm(std::array<double, 6>{0, 0, 0.003, 2.3561944875, -PI / 2, 0.0 }.data(), nullptr, "313"));
			p1.geometryPool().add<aris::dynamic::FileGeometry>("file_geometry", ARIS_INSTALL_PATH + std::string("/resource/aris_robot/ur5/L1.DWG")
				, s_pe2pm(std::array<double, 6>{0.0, 0.0, 0.089159, 0, -PI / 2, 0}.data(), nullptr, "313"));
			p2.geometryPool().add<aris::dynamic::FileGeometry>("file_geometry", ARIS_INSTALL_PATH + std::string("/resource/aris_robot/ur5/L2.DWG")
				, s_pe2pm(std::array<double, 6>{ 0.0, 0.13585, 0.089159, 0, -PI / 2, PI / 2}.data(), nullptr, "313"));
			p3.geometryPool().add<aris::dynamic::FileGeometry>("file_geometry", ARIS_INSTALL_PATH + std::string("/resource/aris_robot/ur5/L3.DWG")
				, s_pe2pm(std::array<double, 6>{0.425, 0.13585 - 0.1197, 0.089159, 0, -PI / 2, PI / 2}.data(), nullptr, "313"));
			p4.geometryPool().add<aris::dynamic::FileGeometry>("file_geometry", ARIS_INSTALL_PATH + std::string("/resource/aris_robot/ur5/L4.DWG")
				, s_pe2pm(std::array<double, 6>{0.425 + 0.39225, 0.13585 - 0.1197 + 0.093, 0.089159, 0, 0, PI/2}.data(), nullptr, "313"));
			p5.geometryPool().add<aris::dynamic::FileGeometry>("file_geometry", ARIS_INSTALL_PATH + std::string("/resource/aris_robot/ur5/L5.DWG")
				, s_pe2pm(std::array<double, 6>{0.425 + 0.39225, 0.13585 - 0.1197 + 0.093, 0.089159 - 0.09465, 0, 0, 0}.data(), nullptr, "313"));
			p6.geometryPool().add<aris::dynamic::FileGeometry>("file_geometry", ARIS_INSTALL_PATH + std::string("/resource/aris_robot/ur5/L6.DWG")
				, s_pe2pm(std::array<double, 6>{0.425 + 0.39225, 0.13585 - 0.1197 + 0.093, 0.089159 - 0.09465, 0, -PI/2, 0}.data(), nullptr, "313"));

			// add joints //
			const double j1_pos[6]{ 0.0, 0.0, 0.089159 }, j1_axis[6]{ 0.0, 0.0, 1.0 };
			const double j2_pos[6]{ 0.0, 0.13585, 0.089159 }, j2_axis[6]{ 0.0, 1.0, 0.0 };
			const double j3_pos[6]{ 0.425, 0.13585 - 0.1197, 0.089159 }, j3_axis[6]{ 0.0, 1.0, 0.0 };
			const double j4_pos[6]{ 0.425 + 0.39225, 0.13585 - 0.1197, 0.089159 }, j4_axis[6]{ 0.0, 1.0, 0.0 };
			const double j5_pos[6]{ 0.425 + 0.39225, 0.13585 - 0.1197 + 0.093, 0.089159 }, j5_axis[6]{ 0.0, 0.0, 1.0 };
			const double j6_pos[6]{ 0.425 + 0.39225, 0.13585 - 0.1197 + 0.093, 0.089159 - 0.09465 }, j6_axis[6]{ 0.0, 1.0, 0.0 };

			auto &j1 = rbt->addRevoluteJoint(p1, rbt->ground(), j1_pos, j1_axis);
			auto &j2 = rbt->addRevoluteJoint(p2, p1, j2_pos, j2_axis);
			auto &j3 = rbt->addRevoluteJoint(p3, p2, j3_pos, j3_axis);
			auto &j4 = rbt->addRevoluteJoint(p4, p3, j4_pos, j4_axis);
			auto &j5 = rbt->addRevoluteJoint(p5, p4, j5_pos, j5_axis);
			auto &j6 = rbt->addRevoluteJoint(p6, p5, j6_pos, j6_axis);

			// add actuation //
			auto &m1 = rbt->addMotion(j1);
			auto &m2 = rbt->addMotion(j2);
			auto &m3 = rbt->addMotion(j3);
			auto &m4 = rbt->addMotion(j4);
			auto &m5 = rbt->addMotion(j5);
			auto &m6 = rbt->addMotion(j6);

			return rbt;
		};
	}
}
