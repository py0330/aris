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

#include "aris_dynamic_model_joint.h"

namespace aris
{
	namespace dynamic
	{
		RevoluteJoint::RevoluteJoint(const std::string &name, Marker &makI, Marker &makJ): JointTemplate(name, makI, makJ)
		{
			const static double loc_cst[6][Dim()]
			{
				1,0,0,0,0,
				0,1,0,0,0,
				0,0,1,0,0,
				0,0,0,1,0,
				0,0,0,0,1,
				0,0,0,0,0
			};
			s_mc(6, Dim(), *loc_cst, const_cast<double*>(locCmPtrI()));
			s_tf_n(Dim(), *this->makI().prtPm(), *loc_cst, *prtCmI_);
		}
		RevoluteJoint::RevoluteJoint(Object &father, const aris::core::XmlElement &xml_ele): JointTemplate(father, xml_ele)
		{
			const static double loc_cst[6][Dim()]
			{
				1,0,0,0,0,
				0,1,0,0,0,
				0,0,1,0,0,
				0,0,0,1,0,
				0,0,0,0,1,
				0,0,0,0,0
			};
			s_mc(6, Dim(), *loc_cst, const_cast<double*>(locCmPtrI()));
			s_tf_n(Dim(), *this->makI().prtPm(), *loc_cst, *prtCmI_);
		}

		PrismaticJoint::PrismaticJoint(const std::string &name, Marker &makI, Marker &makJ): JointTemplate(name, makI, makJ)
		{
			const static double loc_cst[6][Dim()]
			{
				1,0,0,0,0,
				0,1,0,0,0,
				0,0,0,0,0,
				0,0,1,0,0,
				0,0,0,1,0,
				0,0,0,0,1
			};
			s_mc(6, Dim(), *loc_cst, const_cast<double*>(locCmPtrI()));
			s_tf_n(Dim(), *this->makI().prtPm(), *loc_cst, *prtCmI_);
		}
		PrismaticJoint::PrismaticJoint(Object &father, const aris::core::XmlElement &xml_ele): JointTemplate(father, xml_ele)
		{
			const static double loc_cst[6][Dim()]
			{
				1,0,0,0,0,
				0,1,0,0,0,
				0,0,0,0,0,
				0,0,1,0,0,
				0,0,0,1,0,
				0,0,0,0,1
			};
			s_mc(6, Dim(), *loc_cst, const_cast<double*>(locCmPtrI()));
			s_tf_n(Dim(), *this->makI().prtPm(), *loc_cst, *prtCmI_);
		}

		auto UniversalJoint::cptCa(double *ca)const->void
		{
			Constraint::cptCa(ca);

			double tem[6];

			// update makI的z轴 和 makJ的z轴
			const double axis_i_m[3]{ makI().prtPm()[0][2] ,makI().prtPm()[1][2] ,makI().prtPm()[2][2] };
			double axis_j_m[3];
			s_pm_dot_v3(*makJ().fatherPart().pm(), &makJ().prtPm()[0][2], 4, tem, 1);
			s_inv_pm_dot_v3(*makI().fatherPart().pm(), tem, axis_j_m);

			// compute c_dot //
			double wm_in_m[3], wn_in_m[3];
			s_inv_pm_dot_v3(*makI().fatherPart().pm(), makI().fatherPart().glbVs() + 3, wm_in_m);
			s_inv_pm_dot_v3(*makI().fatherPart().pm(), makJ().fatherPart().glbVs() + 3, wn_in_m);

			double iwm = s_vv(3, axis_i_m, wm_in_m);
			double jwm = s_vv(3, axis_j_m, wm_in_m);
			double iwn = s_vv(3, axis_i_m, wn_in_m);
			double jwn = s_vv(3, axis_j_m, wn_in_m);

			ca[3] += 2 * jwm*iwn - jwm*iwm - jwn*iwn;
		}
		auto UniversalJoint::cptCp(double *cp)const->void
		{
			const double *pm_i = *makI().pm();
			const double *pm_j = *makJ().pm();
			const double origin[3]{ 0,0,0 };
			double pm_t[16];

			double v[3], new_z[3];
			s_c3(pm_i + 2, 4, pm_j + 2, 4, v, 1);
			if (s_norm(3, v) == 0)s_vc(3, pm_i, 4, new_z, 1);
			else s_c3(v, 1, pm_i + 2, 4, new_z, 1);
			
			s_sov_pnts2pm(origin, 1, new_z, 1, pm_j, 4, pm_t, "zx");
			s_vc(3, pm_i + 3, 4, pm_t + 3, 4);

			double pq_j2i[7];
			double pm_j2i[4][4];
			double diff[6];

			s_inv_pm_dot_pm(pm_t, pm_j, *pm_j2i);
			s_pm2pq(*pm_j2i, pq_j2i);

			double theta = atan2(s_norm(3, pq_j2i + 3, 1), pq_j2i[6]) * 2;

			double coe = theta < 1e-3 ? 2.0 : theta / std::sin(theta / 2.0);
			s_nv(3, coe, pq_j2i + 3);

			// 此时位移差值在makI()坐标系中。需要转换到部件坐标系下。
			double diff2[6];
			s_tv(pm_t, pq_j2i, diff2);
			s_inv_tv(*makI().fatherPart().pm(), diff2, diff);


			const_cast<UniversalJoint*>(this)->updPrtCmI();
			s_mm(dim(), 1, 6, prtCmPtrI(), ColMajor{ dim() }, diff, 1, cp, 1);
		}
		auto UniversalJoint::cptCv(double *cv)const->void
		{
			const_cast<UniversalJoint*>(this)->updPrtCmI();
			Constraint::cptCv(cv);
		}
		auto UniversalJoint::updPrtCmI()->void 
		{
			double tem[3];
			const double axis_i_m[3]{ makI().prtPm()[0][2] ,makI().prtPm()[1][2] ,makI().prtPm()[2][2] };
			double axis_j_m[3];
			s_pm_dot_v3(*makJ().fatherPart().pm(), &makJ().prtPm()[0][2], 4, tem, 1);
			s_inv_pm_dot_v3(*makI().fatherPart().pm(), tem, axis_j_m);

			s_c3(axis_i_m, 1, axis_j_m, 1, const_cast<double*>(prtCmPtrI()) + 3 * 4 + 3, 4);
			s_nv(3, 1.0 / s_norm(3, prtCmPtrI() + 3 * 4 + 3, 4), const_cast<double*>(prtCmPtrI()) + 3 * 4 + 3, 4);
		}
		UniversalJoint::UniversalJoint(const std::string &name, Marker &makI, Marker &makJ) : JointTemplate(name, makI, makJ)
		{
			const static double loc_cst[6][Dim()]
			{
				1,0,0,0,
				0,1,0,0,
				0,0,1,0,
				0,0,0,0,
				0,0,0,0,
				0,0,0,0,
			};
			s_mc(6, Dim(), *loc_cst, const_cast<double*>(locCmPtrI()));
			s_tf_n(Dim(), *this->makI().prtPm(), *loc_cst, *prtCmI_);
		}
		UniversalJoint::UniversalJoint(Object &father, const aris::core::XmlElement &xml_ele) : JointTemplate(father, xml_ele)
		{
			const static double loc_cst[6][Dim()]
			{
				1,0,0,0,
				0,1,0,0,
				0,0,1,0,
				0,0,0,0,
				0,0,0,0,
				0,0,0,0,
			};
			s_mc(6, Dim(), *loc_cst, const_cast<double*>(locCmPtrI()));
			s_tf_n(Dim(), *this->makI().prtPm(), *loc_cst, *prtCmI_);
		}

		SphericalJoint::SphericalJoint(const std::string &name, Marker &makI, Marker &makJ): JointTemplate(name, makI, makJ)
		{
			const static double loc_cst[6][Dim()]
			{
				1,0,0,
				0,1,0,
				0,0,1,
				0,0,0,
				0,0,0,
				0,0,0,
			};

			s_mc(6, Dim(), *loc_cst, const_cast<double*>(locCmPtrI()));
			s_tf_n(Dim(), *this->makI().prtPm(), *loc_cst, *prtCmI_);
		}
		SphericalJoint::SphericalJoint(Object &father, const aris::core::XmlElement &xml_ele): JointTemplate(father, xml_ele)
		{
			const static double loc_cst[6][Dim()]
			{
				1,0,0,
				0,1,0,
				0,0,1,
				0,0,0,
				0,0,0,
				0,0,0,
			};

			s_mc(6, Dim(), *loc_cst, const_cast<double*>(locCmPtrI()));
			s_tf_n(Dim(), *this->makI().prtPm(), *loc_cst, *prtCmI_);
		}
	}
}
