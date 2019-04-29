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
		for (auto &j : model().jointPool())
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
	auto StewartInverseKinematicSolver::kinPos()->bool
	{
		model().generalMotionPool()[0].makI().setPm(model().generalMotionPool()[0].makJ(), *model().generalMotionPool()[0].mpm());
		
		for (auto i = 0; i < 6; ++i)
		{
			auto u_pmi = imp_->u_[i]->makI().pm();
			auto u_pmj = imp_->u_[i]->makJ().pm();
			auto p_pmi = imp_->p_[i]->makI().pm();
			auto p_pmj = imp_->p_[i]->makJ().pm();
			auto s_pmi = imp_->s_[i]->makI().pm();
			auto s_pmj = imp_->s_[i]->makJ().pm();

			const double p_dir_global[3]{ s_pmj[0][3] - u_pmj[0][3], s_pmj[1][3] - u_pmj[1][3],s_pmj[2][3] - u_pmj[2][3] };
			const double p_dir_in_pa[3]{ imp_->p_[i]->makJ().prtPm()[0][2],imp_->p_[i]->makJ().prtPm()[1][2],imp_->p_[i]->makJ().prtPm()[2][2] };
			const double p_dir_in_pb[3]{ imp_->p_[i]->makI().prtPm()[0][2],imp_->p_[i]->makI().prtPm()[1][2],imp_->p_[i]->makI().prtPm()[2][2] };

			double second_axis_global[3];
			s_c3(&u_pmj[0][2], 4, p_dir_global, 1, second_axis_global, 1);
			const double second_axis_in_pa[3]{ imp_->u_[i]->makI().prtPm()[0][2], imp_->u_[i]->makI().prtPm()[1][2], imp_->u_[i]->makI().prtPm()[2][2] };

			double pm1[16], pm2[16];
			aris::dynamic::s_sov_axes2pm(&u_pmj[0][3], 4, p_dir_global, 1, second_axis_global, 1, pm1,"xy");
			aris::dynamic::s_sov_axes2pm(&imp_->u_[i]->makI().prtPm()[0][3], 4, p_dir_in_pa, 1, second_axis_in_pa, 1, pm2, "xy");

			double p1a_pm[16];
			s_pm_dot_inv_pm(pm1, pm2, p1a_pm);
			model().partPool()[i * 2 + 1].setPm(p1a_pm);


			s_vc(16, *imp_->s_[i]->makJ().pm(), pm1);
			s_mc(3, 3, *imp_->p_[i]->makJ().pm(), 4, pm1, 4);
			
			s_vc(16, *imp_->s_[i]->makI().prtPm(), pm2);
			s_mc(3, 3, *imp_->p_[i]->makI().prtPm(), 4, pm2, 4);

			//aris::dynamic::s_sov_axes2pm(&s_pmj[0][3], 4, p_dir_global, 1, second_axis_global, 1, pm1, "xy");
			//aris::dynamic::s_sov_axes2pm(&imp_->s_[i]->makI().prtPm()[0][3], 4, p_dir_in_pb, 1, second_axis_in_pb, 1, pm2, "xy");

			double p1b_pm[16];
			s_pm_dot_inv_pm(pm1, pm2, p1b_pm);
			model().partPool()[i * 2 + 2].setPm(p1b_pm);
		}

		for (auto &mot : model().motionPool())
		{
			mot.updMp();
		}

		return true;
	}
	StewartInverseKinematicSolver::StewartInverseKinematicSolver(const std::string &name) :InverseKinematicSolver(name, 1, 0.0), imp_(new Imp) {}
	ARIS_DEFINE_BIG_FOUR_CPP(StewartInverseKinematicSolver);
}
