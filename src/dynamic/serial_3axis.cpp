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
#include <array>

#include "aris/dynamic/model.hpp"
#include "aris/dynamic/model_solver.hpp"
#include "aris/dynamic/serial_3axis.hpp"

namespace aris::dynamic
{
	auto createModelSerial3Axis(const Serial3Param &param)->std::unique_ptr<aris::dynamic::Model>
	{
		std::unique_ptr<aris::dynamic::Model> model = std::make_unique<aris::dynamic::Model>("model");

		// 设置重力 //
		const double gravity[6]{ 0.0,0.0,-9.8,0.0,0.0,0.0 };
		model->environment().setGravity(gravity);

		// 添加变量 //
		model->calculator().addVariable("PI", "Matrix", aris::core::Matrix(PI));

		// add part //
		const double default_iv[10]{ 1,0,0,0,0,0,0,0,0,0 };
		auto &p1 = model->partPool().add<Part>("L1", param.iv_vec.size() == 3 ? param.iv_vec[0].data() : default_iv);
		auto &p2 = model->partPool().add<Part>("L2", param.iv_vec.size() == 3 ? param.iv_vec[1].data() : default_iv);
		auto &p3 = model->partPool().add<Part>("L3", param.iv_vec.size() == 3 ? param.iv_vec[2].data() : default_iv);

		// add joint //
		const double j1_pos[3]{ param.a1, 0.0, 0.0 };
		const double j2_pos[3]{ param.a1, 0.0, 0.0 };
		const double j3_pos[3]{ param.a1 + param.a2, 0.0, 0.0 };

		const double j1_axis[6]{ 1.0, 0.0, 0.0 };
		const double j2_axis[6]{ 0.0, 0.0, 1.0 };
		const double j3_axis[6]{ 0.0, 1.0, 0.0 };

		auto &j1 = model->addRevoluteJoint(p1, model->ground(), j1_pos, j1_axis);
		auto &j2 = model->addRevoluteJoint(p2, p1, j2_pos, j2_axis);
		auto &j3 = model->addRevoluteJoint(p3, p2, j3_pos, j3_axis);

		// add actuation //
		auto &m1 = model->addMotion(j1);
		auto &m2 = model->addMotion(j2);
		auto &m3 = model->addMotion(j3);

		const double default_mot_frc[3]{ 0.0, 0.0, 0.0 };

		m1.setFrcCoe(param.mot_frc_vec.size() == 3 ? param.mot_frc_vec[0].data() : default_mot_frc);
		m2.setFrcCoe(param.mot_frc_vec.size() == 3 ? param.mot_frc_vec[1].data() : default_mot_frc);
		m3.setFrcCoe(param.mot_frc_vec.size() == 3 ? param.mot_frc_vec[2].data() : default_mot_frc);

		// add ee general motion //
		const double axis_6_pe[]{ param.a1 + param.a2 + param.a3, 0.0, 0.0, 0.0, 0.0 ,0.0 };
		double axis_6_pm[16];
		double ee_i_pm[16], ee_i_wrt_axis_6_pm[16];
		double ee_j_pm[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };

		s_pe2pm(axis_6_pe, axis_6_pm, "321");
		s_pe2pm(param.tool0_pe, ee_i_wrt_axis_6_pm, param.tool0_pe_type.empty() ? "321" : param.tool0_pe_type.c_str());
		s_pm2pm(axis_6_pm, ee_i_wrt_axis_6_pm, ee_i_pm);

		auto &makI = p3.markerPool().add<Marker>("tool0", ee_i_pm);
		auto &makJ = model->ground().markerPool().add<Marker>("wobj0", ee_j_pm);
		model->variablePool().add<aris::dynamic::MatrixVariable>("tool0_axis_home", aris::core::Matrix(1, 6, 0.0));
		auto &ee = model->generalMotionPool().add<aris::dynamic::GeneralMotion>("ee", &makI, &makJ, false);

		// change robot pose wrt ground //
		double robot_pm[16];
		s_pe2pm(param.base2ref_pe, robot_pm, param.base2ref_pe_type.empty() ? "321" : param.base2ref_pe_type.c_str());

		p1.setPm(s_pm_dot_pm(robot_pm, *p1.pm()));
		p2.setPm(s_pm_dot_pm(robot_pm, *p2.pm()));
		p3.setPm(s_pm_dot_pm(robot_pm, *p3.pm()));
		j1.makJ().setPrtPm(s_pm_dot_pm(robot_pm, *j1.makJ().prtPm()));
		ee.makJ().setPrtPm(s_pm_dot_pm(robot_pm, *ee.makJ().prtPm()));

		// add tools and wobj //
		for (int i = 1; i < 17; ++i)
		{
			p3.markerPool().add<aris::dynamic::Marker>("tool" + std::to_string(i), ee_i_pm);
		}
		for (int i = 1; i < 33; ++i) model->ground().markerPool().add<aris::dynamic::Marker>("wobj" + std::to_string(i), ee_j_pm);

		// add solver
		auto &inverse_kinematic = model->solverPool().add<aris::dynamic::Serial3InverseKinematicSolver>(param);
		auto &forward_kinematic = model->solverPool().add<ForwardKinematicSolver>();
		auto &inverse_dynamic = model->solverPool().add<aris::dynamic::InverseDynamicSolver>();
		auto &forward_dynamic = model->solverPool().add<aris::dynamic::ForwardDynamicSolver>();
		inverse_kinematic.setWhichRoot(4);

		inverse_kinematic.allocateMemory();
		forward_kinematic.allocateMemory();
		inverse_dynamic.allocateMemory();
		forward_dynamic.allocateMemory();

		// external axes
		for (auto &ext : param.external_axes)
		{
			model->motionPool().add<aris::dynamic::Motion>("ext");
		}


		// make topology correct // 
		for (auto &m : model->motionPool())m.activate(true);
		for (auto &gm : model->generalMotionPool())gm.activate(false);
		for (auto &f : model->forcePool())f.activate(false);

		return model;
	}

	

	struct Serial3AxisParamLocal
	{
		double a1;
		double a2;
		double a3;

		// 杆件1-6在上文中零位处的位姿矩阵 //
		double pm_at_init[6][16];

		// 驱动在零位处的偏移，以及系数
		double mp_offset[6];// mp_real = (mp_theoretical - mp_offset) * mp_factor
		double mp_factor[6];
	};
	auto serial_3axis_inverse(const Serial3AxisParamLocal &param, const double *ee_point, int which_root, double *input)->bool
	{
		const double &a1 = param.a1;
		const double &a2 = param.a2;
		const double &a3 = param.a3;

		double q[3];

		double ee_wrt_R2[3];
		s_vc(3, ee_point, ee_wrt_R2);
		ee_wrt_R2[0] -= a1;

		double l = aris::dynamic::s_norm(3, ee_wrt_R2);

		// 确认是否在工作空间内 //
		auto theta = (a2 * a2 + a3 * a3 - l * l) / (2.0 * a2 * a3);
		if (theta > (1.0 + 1e-10) || theta < (-1.0 - 1e-10)) return false;
		theta = std::max(-1.0, std::min(1.0, theta));

		// 求解 theta 3 //
		if (which_root & 0x01)
			q[2] = -aris::PI + std::acos((a2*a2 + a3 * a3 - l * l) / (2.0 * a2 * a3));
		else
			q[2] = aris::PI - std::acos((a2*a2 + a3 * a3 - l * l) / (2.0 * a2 * a3));
		
		auto c3 = std::cos(q[2]);
		auto s3 = std::sin(q[2]);
		
		// 求解 theta 2 //
		if(which_root & 0x02)
			q[1] = -std::acos(ee_wrt_R2[0] / (a2 + c3 * a3));
		else
			q[1] = std::acos(ee_wrt_R2[0] / (a2 + c3 * a3));

		auto c2 = std::cos(q[1]);
		auto s2 = std::sin(q[1]);

		// 求解 theta 1 //
		double x = a2 + a3 * c3;
		double y = 0.0;
		double z = -s3 * a3;
		auto ee = ee_wrt_R2;

		q[0] = atan2((x*s2*ee_wrt_R2[2] - z * ee_wrt_R2[1]), (x*s2*ee_wrt_R2[1] + z * ee_wrt_R2[2]));

		const double *offset = param.mp_offset;
		const double *factor = param.mp_factor;

		// 将q copy到input中
		s_vc(3, q, input);
		return true;
	}
	struct Serial3InverseKinematicSolver::Imp
	{
		int which_root_{ 0 };
		Serial3AxisParamLocal mechnism_param_;
		union
		{
			struct { Part* GR, *L1, *L2, *L3; };
			Part* parts[4];
		};
		union
		{
			struct { RevoluteJoint *R1, *R2, *R3; };
			RevoluteJoint* joints[3];
		};
		union
		{
			struct { Motion *M1, *M2, *M3; };
			Motion* motions[3];
		};
		GeneralMotion *ee;

		bool use_angle_ = false;
	};
	auto Serial3InverseKinematicSolver::saveXml(aris::core::XmlElement &xml_ele) const->void
	{
		InverseKinematicSolver::saveXml(xml_ele);
		xml_ele.SetAttribute("which_root", imp_->which_root_);
	}
	auto Serial3InverseKinematicSolver::loadXml(const aris::core::XmlElement &xml_ele)->void
	{
		InverseKinematicSolver::loadXml(xml_ele);
		setWhichRoot(Object::attributeInt32(xml_ele, "which_root"));
	}
	auto Serial3InverseKinematicSolver::allocateMemory()->void
	{
		InverseKinematicSolver::allocateMemory();

		this->imp_->GR;
		imp_->GR = &ancestor<Model>()->partPool().at(0);
		imp_->L1 = &ancestor<Model>()->partPool().at(1);
		imp_->L2 = &ancestor<Model>()->partPool().at(2);
		imp_->L3 = &ancestor<Model>()->partPool().at(3);

		imp_->R1 = dynamic_cast<RevoluteJoint*>(&ancestor<Model>()->jointPool().at(0));
		imp_->R2 = dynamic_cast<RevoluteJoint*>(&ancestor<Model>()->jointPool().at(1));
		imp_->R3 = dynamic_cast<RevoluteJoint*>(&ancestor<Model>()->jointPool().at(2));

		imp_->M1 = &ancestor<Model>()->motionPool().at(0);
		imp_->M2 = &ancestor<Model>()->motionPool().at(1);
		imp_->M3 = &ancestor<Model>()->motionPool().at(2);
		imp_->ee = &ancestor<Model>()->generalMotionPool().at(0);
	}
	auto Serial3InverseKinematicSolver::kinPos()->int
	{
		// for 华尔康
		if (imp_->use_angle_)
		{
			//if (imp_->which_root_ == 4)
			
				int solution_num = 0;
				double diff_q[2][3];
				double diff_norm[2];
				for (int i = 0; i < 2; ++i)
				{
					// inverse
					s_rm2re(*imp_->ee->mpm(), diff_q[i], "123", 4);
					diff_q[i][1] = -diff_q[i][1];
					
					// solution 2
					if (i == 0)
					{
						diff_q[i][0] = diff_q[i][0] + aris::PI;
						diff_q[i][1] = aris::PI - diff_q[i][1];
						diff_q[i][2] = diff_q[i][2] + aris::PI;

						

					}

					{
						diff_norm[solution_num] = 0;
						for (int j = 0; j < 3; ++j)
						{
							diff_q[solution_num][j] -= imp_->motions[j]->mpInternal();

							while (diff_q[solution_num][j] >  PI) diff_q[solution_num][j] -= 2 * PI;
							while (diff_q[solution_num][j] < -PI) diff_q[solution_num][j] += 2 * PI;

							diff_norm[solution_num] += std::abs(diff_q[solution_num][j]);
						}

						++solution_num;
					}
				}

				auto real_solution = std::min_element(diff_norm, diff_norm + solution_num) - diff_norm;

				// 更新所有杆件 //
				for (aris::Size i = 0; i < 3; ++i)
				{
					if (&imp_->joints[i]->makI().fatherPart() == imp_->parts[i + 1])
					{
						double pm_prt_i[16], pm_mak_i[16], pm_rot[16];
						s_pe2pm(std::array<double, 6>{0, 0, 0, 0, 0, imp_->motions[i]->mpInternal() + diff_q[real_solution][i]}.data(), pm_rot);
						s_pm_dot_pm(*imp_->joints[i]->makJ().pm(), pm_rot, pm_mak_i);
						s_pm_dot_inv_pm(pm_mak_i, *imp_->joints[i]->makI().prtPm(), pm_prt_i);
						imp_->parts[i + 1]->setPm(pm_prt_i);
					}
					else
					{
						double pm_prt_j[16], pm_mak_j[16], pm_rot[16];
						s_pe2pm(std::array<double, 6>{0, 0, 0, 0, 0, -imp_->motions[i]->mpInternal() - diff_q[real_solution][i]}.data(), pm_rot);
						s_pm_dot_pm(*imp_->joints[i]->makI().pm(), pm_rot, pm_mak_j);
						s_pm_dot_inv_pm(pm_mak_j, *imp_->joints[i]->makJ().prtPm(), pm_prt_j);
						imp_->parts[i + 1]->setPm(pm_prt_j);
					}

					imp_->motions[i]->setMpInternal(imp_->motions[i]->mpInternal() + diff_q[real_solution][i]);
				}

				// 更新位姿 //
				imp_->ee->updMpm();

				return 0;
			



			return 0;
		}
		
		// normal
		double ee[3]{ imp_->ee->mpm()[0][3],imp_->ee->mpm()[1][3],imp_->ee->mpm()[2][3] };

		if (imp_->which_root_ == 4)
		{
			int solution_num = 0;
			double diff_q[4][3];
			double diff_norm[4];
			for (int i = 0; i < 4; ++i)
			{
				if (serial_3axis_inverse(imp_->mechnism_param_, ee, i, diff_q[solution_num]))
				{
					diff_norm[solution_num] = 0;
					for (int j = 0; j < 3; ++j)
					{
						diff_q[solution_num][j] -= imp_->motions[j]->mpInternal();

						while (diff_q[solution_num][j] >  PI) diff_q[solution_num][j] -= 2 * PI;
						while (diff_q[solution_num][j] < -PI) diff_q[solution_num][j] += 2 * PI;

						diff_norm[solution_num] += std::abs(diff_q[solution_num][j]);
					}

					++solution_num;
				}
			}

			if (solution_num == 0) return -1;

			auto real_solution = std::min_element(diff_norm, diff_norm + solution_num) - diff_norm;

			// 更新所有杆件 //
			for (aris::Size i = 0; i < 3; ++i)
			{
				if (&imp_->joints[i]->makI().fatherPart() == imp_->parts[i + 1])
				{
					double pm_prt_i[16], pm_mak_i[16], pm_rot[16];
					s_pe2pm(std::array<double, 6>{0, 0, 0, 0, 0, imp_->motions[i]->mpInternal() + diff_q[real_solution][i]}.data(), pm_rot);
					s_pm_dot_pm(*imp_->joints[i]->makJ().pm(), pm_rot, pm_mak_i);
					s_pm_dot_inv_pm(pm_mak_i, *imp_->joints[i]->makI().prtPm(), pm_prt_i);
					imp_->parts[i + 1]->setPm(pm_prt_i);
				}
				else
				{
					double pm_prt_j[16], pm_mak_j[16], pm_rot[16];
					s_pe2pm(std::array<double, 6>{0, 0, 0, 0, 0, -imp_->motions[i]->mpInternal() - diff_q[real_solution][i]}.data(), pm_rot);
					s_pm_dot_pm(*imp_->joints[i]->makI().pm(), pm_rot, pm_mak_j);
					s_pm_dot_inv_pm(pm_mak_j, *imp_->joints[i]->makJ().prtPm(), pm_prt_j);
					imp_->parts[i + 1]->setPm(pm_prt_j);
				}

				imp_->motions[i]->setMpInternal(imp_->motions[i]->mpInternal() + diff_q[real_solution][i]);
			}

			// 更新位姿 //
			imp_->ee->updMpm();

			return 0;
		}
		else
		{
			if (double q[3]; serial_3axis_inverse(imp_->mechnism_param_, ee, imp_->which_root_, q))
			{
				// 更新所有杆件 //
				for (aris::Size i = 0; i < 3; ++i)
				{
					if (&imp_->joints[i]->makI().fatherPart() == imp_->parts[i + 1])
					{
						double pm_prt_i[16], pm_mak_i[16], pm_rot[16];
						s_pe2pm(std::array<double, 6>{0, 0, 0, 0, 0, q[i]}.data(), pm_rot);
						s_pm_dot_pm(*imp_->joints[i]->makJ().pm(), pm_rot, pm_mak_i);
						s_pm_dot_inv_pm(pm_mak_i, *imp_->joints[i]->makI().prtPm(), pm_prt_i);
						imp_->parts[i + 1]->setPm(pm_prt_i);
					}
					else
					{
						double pm_prt_j[16], pm_mak_j[16], pm_rot[16];
						s_pe2pm(std::array<double, 6>{0, 0, 0, 0, 0, -q[i]}.data(), pm_rot);
						s_pm_dot_pm(*imp_->joints[i]->makI().pm(), pm_rot, pm_mak_j);
						s_pm_dot_inv_pm(pm_mak_j, *imp_->joints[i]->makJ().prtPm(), pm_prt_j);
						imp_->parts[i + 1]->setPm(pm_prt_j);
					}

					double last_mp = imp_->motions[i]->mpInternal();
					imp_->motions[i]->updMp();
					while (imp_->motions[i]->mpInternal() - last_mp > PI)imp_->motions[i]->setMpInternal(imp_->motions[i]->mpInternal() - 2 * PI);
					while (imp_->motions[i]->mpInternal() - last_mp < -PI)imp_->motions[i]->setMpInternal(imp_->motions[i]->mpInternal() + 2 * PI);
				}

				// 更新位姿 //
				imp_->ee->updMpm();

				return 0;
			}
			else return -2;
		}
	}
	auto Serial3InverseKinematicSolver::setWhichRoot(int root_of_0_to_4)->void 
	{ 
		imp_->which_root_ = root_of_0_to_4;
	}
	auto Serial3InverseKinematicSolver::setPmEE(const double *ee_pm, const double *extnal_axes)->void
	{
		imp_->ee->setMpe(std::array<double, 6>{ee_pm[3], ee_pm[7], ee_pm[11], 0, 0, 0}.data());
		
		if (extnal_axes)
		{
			for (int i = 3; i < model().motionPool().size(); ++i)
			{
				model().motionPool()[i].setMp(extnal_axes[i - 3]);
			}
		}

		imp_->use_angle_ = false;

	}
	auto Serial3InverseKinematicSolver::setEulaAngle(const double *eul, const char *type)->void
	{
		imp_->ee->setMpe(std::array<double, 6>{0, 0, 0, eul[0], eul[1], eul[2]}.data(), type);
		imp_->use_angle_ = true;
	}
	auto Serial3InverseKinematicSolver::setQuaternionAngle(const double *q)->void
	{
		imp_->ee->setMpq(std::array<double, 7>{0, 0, 0, q[0], q[1], q[2], q[3]}.data());
		imp_->use_angle_ = true;
	}
	auto Serial3InverseKinematicSolver::setPqEEAngle(const double *pq)->void
	{
		imp_->ee->setMpq(pq);
		imp_->use_angle_ = true;
	}
	auto Serial3InverseKinematicSolver::setPeEEAngle(const double *pe, const char *type)->void
	{
		imp_->ee->setMpe(pe, type);
		imp_->use_angle_ = true;
	}
	Serial3InverseKinematicSolver::~Serial3InverseKinematicSolver() = default;
	Serial3InverseKinematicSolver::Serial3InverseKinematicSolver(const Serial3Param &param, const std::string &name) :InverseKinematicSolver(name, 1, 0.0), imp_(new Imp) 
	{
		imp_->mechnism_param_.a1 = param.a1;
		imp_->mechnism_param_.a2 = param.a2;
		imp_->mechnism_param_.a3 = param.a3;
	}
	ARIS_DEFINE_BIG_FOUR_CPP(Serial3InverseKinematicSolver);
}