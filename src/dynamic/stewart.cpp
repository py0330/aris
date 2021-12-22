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
#include "aris/dynamic/stewart.hpp"

namespace aris::dynamic
{
	auto createModelStewart()->std::unique_ptr<aris::dynamic::Model>
	{
		std::unique_ptr<aris::dynamic::Model> model = std::make_unique<aris::dynamic::Model>();

		// 设置重力 //
		const double gravity[6]{ 0.0,0.0,-9.8,0.0,0.0,0.0 };
		model->environment().setGravity(gravity);

		// 尺寸变量 //
		//const double down_pos[6][3]
		//{
		//{  0.475, 0.0, 0.038 },
		//{  0.475, 0.0,-0.038 },
		//{ -0.205, 0.0,-0.430 },
		//{ -0.270, 0.0,-0.393 },
		//{ -0.270, 0.0, 0.393 },
		//{ -0.205, 0.0, 0.430 },
		//};
		//const double up_pos[6][3]
		//{
		//{  0.272, 0.0, 0.377 },
		//{  0.272, 0.0,-0.377 },
		//{  0.203, 0.0,-0.418 },
		//{ -0.500, 0.0,-0.040 },
		//{ -0.500, 0.0, 0.040 },
		//{  0.203, 0.0, 0.418 },
		//};

		//const double down_pos[6][3]
		//{
		//{ -0.43011, 0.0,-0.20502 },
		//{ -0.03750, 0.0, 0.47500 },
		//{  0.03750, 0.0, 0.47500 },
		//{  0.43011, 0.0,-0.20502 },
		//{  0.38261, 0.0,-0.26998 },
		//{ -0.38261, 0.0,-0.26998 },
		//};
		//const double up_pos[6][3]
		//{
		//{ -0.41960, 0.0, 0.19895 },
		//{ -0.38210, 0.0, 0.26391 },
		//{  0.38210, 0.0, 0.26391 },
		//{  0.41960, 0.0, 0.19895 },
		//{  0.03750, 0.0,-0.46286 },
		//{ -0.03750, 0.0,-0.46286 },
		//};

		const double down_pos[6][3]
		{
			{ -0.43011, 0.20502, 0.0 },
		{ -0.03750,-0.47500, 0.0 },
		{ 0.03750,-0.47500, 0.0 },
		{ 0.43011, 0.20502, 0.0 },
		{ 0.38261, 0.26998, 0.0 },
		{ -0.38261, 0.26998, 0.0 },
		};
		const double up_pos[6][3]
		{
			{ -0.41960,-0.19895, 0.0 },
		{ -0.38210,-0.26391, 0.0 },
		{ 0.38210,-0.26391, 0.0 },
		{ 0.41960,-0.19895, 0.0 },
		{ 0.03750, 0.46286, 0.0 },
		{ -0.03750, 0.46286, 0.0 },
		};


		// 移动方向，U副第一根轴、第二根轴
		double prismatic_direction[6][3];
		double first_axis[6][3];
		//const double second_axis[6][3]
		//{
		//{ 0,1,0 },
		//{ 0,1,0 },
		//{ 0,1,0 },
		//{ 0,1,0 },
		//{ 0,1,0 },
		//{ 0,1,0 },
		//};
		const double second_axis[6][3]
		{
			{ 0,0,1 },
		{ 0,0,1 },
		{ 0,0,1 },
		{ 0,0,1 },
		{ 0,0,1 },
		{ 0,0,1 },
		};
		for (Size i = 0; i < 6; ++i)
		{
			s_vc(3, up_pos[i], prismatic_direction[i]);
			s_vs(3, down_pos[i], prismatic_direction[i]);
			s_nv(3, 1.0 / s_norm(3, prismatic_direction[i]), prismatic_direction[i]);

			s_c3(prismatic_direction[i], second_axis[i], first_axis[i]);
		}

		double p1a_pm[16], p1b_pm[16], p2a_pm[16], p2b_pm[16], p3a_pm[16], p3b_pm[16], p4a_pm[16], p4b_pm[16], p5a_pm[16], p5b_pm[16], p6a_pm[16], p6b_pm[16];
		s_sov_axes2pm(down_pos[0], first_axis[0], second_axis[0], p1a_pm, "xz");
		s_sov_axes2pm(up_pos[0], first_axis[0], second_axis[0], p1b_pm, "xz");
		s_sov_axes2pm(down_pos[1], first_axis[1], second_axis[1], p2a_pm, "xz");
		s_sov_axes2pm(up_pos[1], first_axis[1], second_axis[1], p2b_pm, "xz");
		s_sov_axes2pm(down_pos[2], first_axis[2], second_axis[2], p3a_pm, "xz");
		s_sov_axes2pm(up_pos[2], first_axis[2], second_axis[2], p3b_pm, "xz");
		s_sov_axes2pm(down_pos[3], first_axis[3], second_axis[3], p4a_pm, "xz");
		s_sov_axes2pm(up_pos[3], first_axis[3], second_axis[3], p4b_pm, "xz");
		s_sov_axes2pm(down_pos[4], first_axis[4], second_axis[4], p5a_pm, "xz");
		s_sov_axes2pm(up_pos[4], first_axis[4], second_axis[4], p5b_pm, "xz");
		s_sov_axes2pm(down_pos[5], first_axis[5], second_axis[5], p6a_pm, "xz");
		s_sov_axes2pm(up_pos[5], first_axis[5], second_axis[5], p6b_pm, "xz");

		// add part //
		auto &p1a = model->partPool().add<Part>("p1a", nullptr, p1a_pm);
		auto &p1b = model->partPool().add<Part>("p1b", nullptr, p1b_pm);
		auto &p2a = model->partPool().add<Part>("p2a", nullptr, p2a_pm);
		auto &p2b = model->partPool().add<Part>("p2b", nullptr, p2b_pm);
		auto &p3a = model->partPool().add<Part>("p3a", nullptr, p3a_pm);
		auto &p3b = model->partPool().add<Part>("p3b", nullptr, p3b_pm);
		auto &p4a = model->partPool().add<Part>("p4a", nullptr, p4a_pm);
		auto &p4b = model->partPool().add<Part>("p4b", nullptr, p4b_pm);
		auto &p5a = model->partPool().add<Part>("p5a", nullptr, p5a_pm);
		auto &p5b = model->partPool().add<Part>("p5b", nullptr, p5b_pm);
		auto &p6a = model->partPool().add<Part>("p6a", nullptr, p6a_pm);
		auto &p6b = model->partPool().add<Part>("p6b", nullptr, p6b_pm);
		auto &up = model->partPool().add<Part>("up");

		p1a.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\aris\\aris-1.5.0\\resource\\test_dynamic\\stewart\\pa.xmt_txt");
		p1b.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\aris\\aris-1.5.0\\resource\\test_dynamic\\stewart\\pb.xmt_txt");
		p2a.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\aris\\aris-1.5.0\\resource\\test_dynamic\\stewart\\pa.xmt_txt");
		p2b.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\aris\\aris-1.5.0\\resource\\test_dynamic\\stewart\\pb.xmt_txt");
		p3a.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\aris\\aris-1.5.0\\resource\\test_dynamic\\stewart\\pa.xmt_txt");
		p3b.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\aris\\aris-1.5.0\\resource\\test_dynamic\\stewart\\pb.xmt_txt");
		p4a.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\aris\\aris-1.5.0\\resource\\test_dynamic\\stewart\\pa.xmt_txt");
		p4b.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\aris\\aris-1.5.0\\resource\\test_dynamic\\stewart\\pb.xmt_txt");
		p5a.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\aris\\aris-1.5.0\\resource\\test_dynamic\\stewart\\pa.xmt_txt");
		p5b.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\aris\\aris-1.5.0\\resource\\test_dynamic\\stewart\\pb.xmt_txt");
		p6a.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\aris\\aris-1.5.0\\resource\\test_dynamic\\stewart\\pa.xmt_txt");
		p6b.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\aris\\aris-1.5.0\\resource\\test_dynamic\\stewart\\pb.xmt_txt");
		up.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\aris\\aris-1.5.0\\resource\\test_dynamic\\stewart\\up.xmt_txt");

		// add joint //
		auto &u1 = model->addUniversalJoint(p1a, model->ground(), down_pos[0], second_axis[0], first_axis[0]);
		auto &p1 = model->addPrismaticJoint(p1b, p1a, down_pos[0], prismatic_direction[0]);
		auto &s1 = model->addSphericalJoint(p1b, up, up_pos[0]);
		auto &u2 = model->addUniversalJoint(p2a, model->ground(), down_pos[1], second_axis[1], first_axis[1]);
		auto &p2 = model->addPrismaticJoint(p2b, p2a, down_pos[1], prismatic_direction[1]);
		auto &s2 = model->addSphericalJoint(p2b, up, up_pos[1]);
		auto &u3 = model->addUniversalJoint(p3a, model->ground(), down_pos[2], second_axis[2], first_axis[2]);
		auto &p3 = model->addPrismaticJoint(p3b, p3a, down_pos[2], prismatic_direction[2]);
		auto &s3 = model->addSphericalJoint(p3b, up, up_pos[2]);
		auto &u4 = model->addUniversalJoint(p4a, model->ground(), down_pos[3], second_axis[3], first_axis[3]);
		auto &p4 = model->addPrismaticJoint(p4b, p4a, down_pos[3], prismatic_direction[3]);
		auto &s4 = model->addSphericalJoint(p4b, up, up_pos[3]);
		auto &u5 = model->addUniversalJoint(p5a, model->ground(), down_pos[4], second_axis[4], first_axis[4]);
		auto &p5 = model->addPrismaticJoint(p5b, p5a, down_pos[4], prismatic_direction[4]);
		auto &s5 = model->addSphericalJoint(p5b, up, up_pos[4]);
		auto &u6 = model->addUniversalJoint(p6a, model->ground(), down_pos[5], second_axis[5], first_axis[5]);
		auto &p6 = model->addPrismaticJoint(p6b, p6a, down_pos[5], prismatic_direction[5]);
		auto &s6 = model->addSphericalJoint(p6b, up, up_pos[5]);

		// add actuation //
		auto &m1 = model->addMotion(p1);
		auto &m2 = model->addMotion(p2);
		auto &m3 = model->addMotion(p3);
		auto &m4 = model->addMotion(p4);
		auto &m5 = model->addMotion(p5);
		auto &m6 = model->addMotion(p6);

		for (int i = 0; i < 6; ++i)
		{
			double line[3];
			s_vc(3, up_pos[i], line);
			s_vs(3, down_pos[i], line);

			auto v = s_norm(3, line);
			model->motionPool()[i].setMpOffset(-v);
		}

		// add ee general motion //
		double pm_ee_i[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
		double pm_ee_j[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };

		auto &makI = up.addMarker("ee_makI", pm_ee_i);
		auto &makJ = model->ground().addMarker("ee_makJ", pm_ee_j);
		auto &ee = model->generalMotionPool().add<aris::dynamic::GeneralMotion>("ee", &makI, &makJ, false);

		// add solver
		auto &inverse_kinematic = model->solverPool().add<aris::dynamic::StewartInverseKinematicSolver>();
		auto &forward_kinematic = model->solverPool().add<aris::dynamic::ForwardKinematicSolver>();
		auto &inverse_dynamic = model->solverPool().add<aris::dynamic::InverseDynamicSolver>();
		auto &forward_dynamic = model->solverPool().add<aris::dynamic::ForwardDynamicSolver>();

		model->init();

		// make topology correct // 
		for (auto &m : model->motionPool())m.activate(true);
		for (auto &gm : model->generalMotionPool())gm.activate(false);
		for (auto &f : model->forcePool())f.activate(false);

		return model;
	}
	
	struct StewartInverseKinematicSolver::Imp
	{
		UniversalJoint *u_[6];
		PrismaticJoint *p_[6];
		SphericalJoint *s_[6];
	};
	auto StewartInverseKinematicSolver::allocateMemory()->void
	{
		InverseKinematicSolver::allocateMemory();

		int u_num{ 0 }, p_num{ 0 }, s_num{ 0 };
		for (auto &j : model()->jointPool())
		{
			if (auto u = dynamic_cast<UniversalJoint*>(&j))
			{
				imp_->u_[u_num] = u;
				++u_num;
			}
			if (auto p = dynamic_cast<PrismaticJoint*>(&j))
			{
				imp_->p_[p_num] = p;
				++p_num;
			}
			if (auto s = dynamic_cast<SphericalJoint*>(&j))
			{
				imp_->s_[s_num] = s;
				++s_num;
			}
		}
	}
	auto StewartInverseKinematicSolver::kinPos()->int
	{
		model()->generalMotionPool()[0].makI()->setPm(*model()->generalMotionPool()[0].makJ(), *dynamic_cast<GeneralMotion&>(model()->generalMotionPool()[0]).mpm());
		
		for (aris::Size i = 0; i < 6; ++i)
		{
			auto u_pmi = imp_->u_[i]->makI()->pm();
			auto u_pmj = imp_->u_[i]->makJ()->pm();
			auto p_pmi = imp_->p_[i]->makI()->pm();
			auto p_pmj = imp_->p_[i]->makJ()->pm();
			auto s_pmi = imp_->s_[i]->makI()->pm();
			auto s_pmj = imp_->s_[i]->makJ()->pm();

			const double p_dir_global[3]{ s_pmj[0][3] - u_pmj[0][3], s_pmj[1][3] - u_pmj[1][3],s_pmj[2][3] - u_pmj[2][3] };
			const double p_dir_in_pa[3]{ imp_->p_[i]->makJ()->prtPm()[0][2],imp_->p_[i]->makJ()->prtPm()[1][2],imp_->p_[i]->makJ()->prtPm()[2][2] };
			const double p_dir_in_pb[3]{ imp_->p_[i]->makI()->prtPm()[0][2],imp_->p_[i]->makI()->prtPm()[1][2],imp_->p_[i]->makI()->prtPm()[2][2] };

			double second_axis_global[3];
			s_c3(&u_pmj[0][2], 4, p_dir_global, 1, second_axis_global, 1);
			const double second_axis_in_pa[3]{ imp_->u_[i]->makI()->prtPm()[0][2], imp_->u_[i]->makI()->prtPm()[1][2], imp_->u_[i]->makI()->prtPm()[2][2] };

			double pm1[16], pm2[16];
			aris::dynamic::s_sov_axes2pm(&u_pmj[0][3], 4, p_dir_global, 1, second_axis_global, 1, pm1,"xy");
			aris::dynamic::s_sov_axes2pm(&imp_->u_[i]->makI()->prtPm()[0][3], 4, p_dir_in_pa, 1, second_axis_in_pa, 1, pm2, "xy");

			double p1a_pm[16];
			s_pm_dot_inv_pm(pm1, pm2, p1a_pm);
			model()->partPool()[i * 2 + 1].setPm(p1a_pm);


			s_vc(16, *imp_->s_[i]->makJ()->pm(), pm1);
			s_mc(3, 3, *imp_->p_[i]->makJ()->pm(), 4, pm1, 4);
			
			s_vc(16, *imp_->s_[i]->makI()->prtPm(), pm2);
			s_mc(3, 3, *imp_->p_[i]->makI()->prtPm(), 4, pm2, 4);

			//aris::dynamic::s_sov_axes2pm(&s_pmj[0][3], 4, p_dir_global, 1, second_axis_global, 1, pm1, "xy");
			//aris::dynamic::s_sov_axes2pm(&imp_->s_[i]->makI()->prtPm()[0][3], 4, p_dir_in_pb, 1, second_axis_in_pb, 1, pm2, "xy");

			double p1b_pm[16];
			s_pm_dot_inv_pm(pm1, pm2, p1b_pm);
			model()->partPool()[i * 2 + 2].setPm(p1b_pm);
		}

		for (auto &mot : model()->motionPool())
		{
			mot.updP();
		}

		return 0;
	}
	StewartInverseKinematicSolver::~StewartInverseKinematicSolver() = default;
	StewartInverseKinematicSolver::StewartInverseKinematicSolver() :InverseKinematicSolver(1, 0.0), imp_(new Imp) {}
	ARIS_DEFINE_BIG_FOUR_CPP(StewartInverseKinematicSolver);
}
