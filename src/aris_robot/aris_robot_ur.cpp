#include <algorithm>

#include"aris_robot_ur.h"

using namespace aris::dynamic;

namespace aris
{
	namespace robot
	{
		// 具体参数参考MR里面147-158页 //
		auto create_ur5()->std::unique_ptr<aris::dynamic::Model>
		{
			std::unique_ptr<aris::dynamic::Model> rbt = std::make_unique<aris::dynamic::Model>("ur5");

			// 设置重力 //
			const double gravity[6]{ 0.0,0.0,-9.8,0.0,0.0,0.0 };
			rbt->environment().setGravity(gravity);

			// 相对位移 //
			const double pe_j1[6]{ 0.0, 0.0, 0.089159, 0, 0, 0 };
			const double pe_j2[6]{ 0.0, 0.13585, 0.0, 0, PI / 2, 0 };
			const double pe_j3[6]{ 0.0, -0.1197, 0.425, 0, 0, 0 };
			const double pe_j4[6]{ 0.0, 0.0, 0.39225, 0, PI / 2, 0 };
			const double pe_j5[6]{ 0.0, 0.10915 + 0.1197 - 0.13585, 0.0, 0, 0, 0 };
			const double pe_j6[6]{ 0.0, 0.0, 0.09465, 0, 0, 0 };
			const double pe_ee[6]{ 0.0, 0.0823, 0.0, 0, 0, -PI / 2 };
			
			double pm1[16], pm2[16], pm3[16], pm4[16], pm5[16], pm6[16], pm_ee[16];

			s_pe2pm(pe_j1, pm1, "321");
			s_pm_dot_pm(pm1, s_pe2pm(pe_j2, nullptr, "321"), pm2);
			s_pm_dot_pm(pm2, s_pe2pm(pe_j3, nullptr, "321"), pm3);
			s_pm_dot_pm(pm3, s_pe2pm(pe_j4, nullptr, "321"), pm4);
			s_pm_dot_pm(pm4, s_pe2pm(pe_j5, nullptr, "321"), pm5);
			s_pm_dot_pm(pm5, s_pe2pm(pe_j6, nullptr, "321"), pm6);
			s_pm_dot_pm(pm5, s_pe2pm(pe_j6, nullptr, "321"), pm6);
			s_pm_dot_pm(pm6, s_pe2pm(pe_ee, nullptr, "321"), pm_ee);
			
			double iv[10], pm[16];

			const double p1_pe[6]{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
			const double p1_iv[10]{ 3.7, 0, 0, 0, 0.010267495893,0.010267495893, 0.00666, 0.0, 0.0, 0.0 };
			s_pe2pm(p1_pe, pm, "321");
			s_iv2iv(s_pm_dot_pm(pm1, pm), p1_iv, iv);
			auto &p1 = rbt->partPool().add<Part>("L1", iv);

			const double p2_pe[6]{ 0.0, 0.0, 0.28, 0.0, 0.0, 0.0 };
			const double p2_iv[10]{ 8.393, 0, 0, 0, 0.22689067591, 0.22689067591, 0.0151074, 0.0, 0.0, 0.0 };
			s_pe2pm(p2_pe, pm, "321");
			s_iv2iv(s_pm_dot_pm(pm2, pm), p2_iv, iv);
			auto &p2 = rbt->partPool().add<Part>("L2", iv);

			const double p3_pe[6]{ 0.0, 0.0, 0.25, 0.0, 0.0, 0.0 };
			const double p3_iv[10]{ 2.275, 0, 0, 0, 0.049443313556, 0.049443313556, 0.004095, 0.0, 0.0, 0.0 };
			s_pe2pm(p3_pe, pm, "321");
			s_iv2iv(s_pm_dot_pm(pm3, pm), p3_iv, iv);
			auto &p3 = rbt->partPool().add<Part>("L3", iv);

			const double p4_pe[6]{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
			const double p4_iv[10]{ 1.219, 0, 0, 0, 0.111172755531, 0.111172755531, 0.21942, 0.0, 0.0, 0.0 };
			s_pe2pm(p4_pe, pm, "321");
			s_iv2iv(s_pm_dot_pm(pm4, pm), p4_iv, iv);
			auto &p4 = rbt->partPool().add<Part>("L4", iv);

			const double p5_pe[6]{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.00 };
			const double p5_iv[10]{ 1.219, 0, 0, 0, 0.111172755531, 0.111172755531, 0.21942, 0.0, 0.0, 0.0 };
			s_pe2pm(p5_pe, pm, "321");
			s_iv2iv(s_pm_dot_pm(pm5, pm), p5_iv, iv);
			auto &p5 = rbt->partPool().add<Part>("L5", iv);

			const double p6_pe[6]{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
			const double p6_iv[10]{ 0.1879, 0, 0, 0, 0.0171364731454, 0.0171364731454, 0.033822, 0.0, 0.0, 0.0 };
			s_pe2pm(p6_pe, pm, "321");
			s_iv2iv(s_pm_dot_pm(pm6, pm), p6_iv, iv);
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
				, s_pe2pm(std::array<double, 6>{0.425 + 0.39225, 0.13585 - 0.1197 + 0.093, 0.089159, 0, -PI / 2, -PI / 2}.data(), nullptr, "321"));
			p5.geometryPool().add<aris::dynamic::FileGeometry>("file_geometry", ARIS_INSTALL_PATH + std::string("/resource/aris_robot/ur5/L5.DWG")
				, s_pe2pm(std::array<double, 6>{0.425 + 0.39225, 0.13585 - 0.1197 + 0.093, 0.089159 - 0.09465, 0, -PI / 2, PI}.data(), nullptr, "313"));
			p6.geometryPool().add<aris::dynamic::FileGeometry>("file_geometry", ARIS_INSTALL_PATH + std::string("/resource/aris_robot/ur5/L6.DWG")
				, s_pe2pm(std::array<double, 6>{0.425 + 0.39225, 0.13585 - 0.1197 + 0.093, 0.089159 - 0.09465, 0, -PI / 2, 0}.data(), nullptr, "313"));

			const double L1 = 0.425;
			const double L2 = 0.39225;
			const double W1 = 0.13585 - 0.1197 + 0.093;
			const double W2 = 0.0823;
			const double H1 = 0.089159;
			const double H2 = 0.09465;


			// add joints //
			const double j1_axis[6]{ 0.0, 0.0, 1.0 };
			const double j2_axis[6]{ 0.0, 1.0, 0.0 };
			const double j3_axis[6]{ 0.0, 1.0, 0.0 };
			const double j4_axis[6]{ 0.0, 1.0, 0.0 };
			const double j5_axis[6]{ 0.0, 0.0, -1.0 };
			const double j6_axis[6]{ 0.0, 1.0, 0.0 };

			auto &j1 = rbt->addRevoluteJoint(p1, rbt->ground(), s_pm2pp(pm1), j1_axis);
			auto &j2 = rbt->addRevoluteJoint(p2, p1, s_pm2pp(pm2), j2_axis);
			auto &j3 = rbt->addRevoluteJoint(p3, p2, s_pm2pp(pm3), j3_axis);
			auto &j4 = rbt->addRevoluteJoint(p4, p3, s_pm2pp(pm4), j4_axis);
			auto &j5 = rbt->addRevoluteJoint(p5, p4, s_pm2pp(pm5), j5_axis);
			auto &j6 = rbt->addRevoluteJoint(p6, p5, s_pm2pp(pm6), j6_axis);

			// add actuation //
			auto &m1 = rbt->addMotion(j1);
			auto &m2 = rbt->addMotion(j2);
			auto &m3 = rbt->addMotion(j3);
			auto &m4 = rbt->addMotion(j4);
			auto &m5 = rbt->addMotion(j5);
			auto &m6 = rbt->addMotion(j6);

			// add ee general motion //
			auto &makI = p6.markerPool().add<Marker>("ee_makI", pm_ee);
			auto &makJ = rbt->ground().markerPool().add<Marker>("ee_makJ");
			auto &ee = rbt->generalMotionPool().add<aris::dynamic::GeneralMotion>("ee", &makI, &makJ, false);

			return rbt;
		};
		auto Ur5InverseSolver::kinPos()->bool
		{
			double q[6];

			double j6_pnt_loc[3]{ 0.0, 0.0, -0.0823 };
			double joint_pnt[3];
			s_pp2pp(*model().generalMotionPool().at(0).mpm(), j6_pnt_loc, joint_pnt);
			q[0] = std::atan2(joint_pnt[1], joint_pnt[0]) - std::asin(0.10915 / std::sqrt(joint_pnt[0] * joint_pnt[0] + joint_pnt[1] * joint_pnt[1]));

			double pe[6]{ 0,0,0,q[0],0,0 };
			double pm1[16], pm2[16], pm3[16], pm4[16];
			s_pe2pm(pe, pm1, "321");

			s_inv_pm_dot_pm(pm1, *model().generalMotionPool().at(0).mpm(), pm2);
			s_pe2pm(std::array<double, 6>{0.425 + 0.39225, 0.13585 - 0.1197 + 0.093 + 0.0823, 0.089159 - 0.09465, PI, 0, PI / 2}.data(), pm3, "321");
			s_pm_dot_inv_pm(pm2, pm3, pm4);
			s_pm2pe(pm4, pe, "232");

			if (std::abs(pe[4] - 0) < 1e-10)
			{
				pe[5] = (pe[3] + pe[5]) > PI ? pe[3] + pe[5] - 2 * PI : pe[3] + pe[5];
				pe[3] = 0.0;
			}

			q[4] = -pe[4];
			q[5] = pe[5];


			double pe2[6]{ 0,0,0,q[0],pe[3],0.0 };
			s_pe2pm(pe2, pm2, "323");
			s_va(3, 0.09465, pm2 + 2, 4, joint_pnt, 1);
			s_va(3, -(0.13585 - 0.1197 + 0.093), pm2 + 1, 4, joint_pnt, 1);

			double pp[3];
			s_inv_pp2pp(pm1, joint_pnt, pp);
			pp[2] -= 0.089159;

			q[2] = PI - std::acos(-(pp[0] * pp[0] + pp[2] * pp[2] - 0.425 * 0.425 - 0.39225 * 0.39225) / (2 * 0.425*0.39225));
			q[1] = -PI + std::acos((-pp[0] * pp[0] - pp[2] * pp[2] - 0.425 * 0.425 + 0.39225 * 0.39225) / (2 * std::sqrt(pp[0] * pp[0] + pp[2] * pp[2]) *0.425)) - std::atan2(pp[2], pp[0]);
			q[3] = pe[3] - q[1] - q[2];

			double pe3[6]{ 0.0,0.0,0.0,0.0,0.0,0.0 }, pm[16];
			for (aris::Size i = 0; i < 6; ++i)
			{
				pe3[5] = q[i];
				s_pm_dot_pm(*model().jointPool().at(i).makJ().pm(), s_pe2pm(pe3, pm, "123"), pm1);
				s_pm_dot_inv_pm(pm1, *model().jointPool().at(i).makI().prtPm(), const_cast<double*>(*model().jointPool().at(i).makI().fatherPart().pm()));
				model().motionPool().at(i).updMp();
			}

			return true;
		};
	}
}
