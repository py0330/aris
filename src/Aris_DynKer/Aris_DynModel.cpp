#include <Platform.h>

#include <cmath>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <algorithm>
#include <limits>
#include <sstream>

#include "Aris_ExpCal.h"
#include "Aris_DynKer.h"
#include "Aris_DynModel.h"

namespace Aris
{
	namespace DynKer
	{
		const char *const TranslationalJoint::type = "translational";
		TranslationalJoint::TranslationalJoint(ModelBase &model, const std::string &Name, int id, Marker &makI, Marker &makJ)
			: JointBaseDim(model, Name, id, makI, makJ)
		{
		}
		TranslationalJoint::TranslationalJoint(ModelBase &model, const std::string &Name, int id, const Aris::Core::XmlElement *ele)
			: JointBaseDim(model, Name, id, ele)
		{
		}
		void TranslationalJoint::Init()
		{
			double loc_cst[6][Dim()];
			std::fill_n(static_cast<double*>(*loc_cst), 6 * Dim(), 0);

			loc_cst[0][0] = 1;
			loc_cst[1][1] = 1;
			loc_cst[3][2] = 1;
			loc_cst[4][3] = 1;
			loc_cst[5][4] = 1;

			s_tf_n(Dim(), *MakI().PrtPm(), *loc_cst, PrtCstMtxI());
		};
		
		const char *const UniversalJoint::type = "universal";
		UniversalJoint::UniversalJoint(ModelBase &model, const std::string &Name, int id, Marker &makI, Marker &makJ)
			: JointBaseDim(model, Name, id, makI, makJ)
		{
		}
		UniversalJoint::UniversalJoint(ModelBase &model, const std::string &Name, int id, const Aris::Core::XmlElement *ele)
			: JointBaseDim(model, Name, id, ele)
		{
		}
		void UniversalJoint::ToAdamsCmd(std::ofstream &file) const
		{
			double pe[6] = { 0, 0, 0, PI / 2, 0, 0 };
			double pe2[6] = { 0, 0, 0, -PI / 2, 0, 0 };
			double pm[4][4], pm2[4][4];
			
			s_pe2pm(pe, *pm, "213");
			s_pm_dot_pm(*this->MakI().PrtPm(), *pm, *pm2);
			s_pm2pe(*pm2, pe, "313");

			file << "marker modify &\r\n"
				<< "    marker_name = ." << Model().Name() << "." << this->MakI().Father().Name() << "." << this->MakI().Name() << " &\r\n"
				<< "    orientation = (" << Matrix(1, 3, &pe[3]).ToString() << ") \r\n"
				<< "!\r\n";

			s_pe2pm(pe2, *pm, "123");
			s_pm_dot_pm(*this->MakJ().PrtPm(), *pm, *pm2);
			s_pm2pe(*pm2, pe, "313");

			file << "marker modify &\r\n"
				<< "    marker_name = ." << Model().Name() << "." << this->MakJ().Father().Name() << "." << this->MakJ().Name() << " &\r\n"
				<< "    orientation = (" << Matrix(1, 3, &pe[3]).ToString() << ") \r\n"
				<< "!\r\n";

			JointBase::ToAdamsCmd(file);
		}
		void UniversalJoint::Init()
		{
			double loc_cst[6][Dim()];
			std::fill_n(static_cast<double*>(*loc_cst), 6 * Dim(), 0);

			loc_cst[0][0] = 1;
			loc_cst[1][1] = 1;
			loc_cst[2][2] = 1;

			s_tf_n(Dim(), *MakI().PrtPm(), *loc_cst, PrtCstMtxI());

		};
		void UniversalJoint::Update()
		{
			/*update PrtCstMtxI*/
			MakI().Update();
			MakJ().Update();

			//get sin(a) and cos(a)
			double s = MakI().Pm()[0][2] * MakJ().Pm()[0][1]
				+ MakI().Pm()[1][2] * MakJ().Pm()[1][1]
				+ MakI().Pm()[2][2] * MakJ().Pm()[2][1];

			double c = MakI().Pm()[0][1] * MakJ().Pm()[0][1]
				+ MakI().Pm()[1][1] * MakJ().Pm()[1][1]
				+ MakI().Pm()[2][1] * MakJ().Pm()[2][1];
			
			_PrtCstMtxI[3][3] = -(MakI().PrtPm()[0][1]) * s + (MakI().PrtPm()[0][2]) * c;
			_PrtCstMtxI[4][3] = -(MakI().PrtPm()[1][1]) * s + (MakI().PrtPm()[1][2]) * c;
			_PrtCstMtxI[5][3] = -(MakI().PrtPm()[2][1]) * s + (MakI().PrtPm()[2][2]) * c;

			/*edit CstMtxJ*/
			std::fill_n(this->PrtCstMtxJ(), this->CstDim() * 6, 0);
			double _pm_M2N[4][4];
			s_pm_dot_pm(*MakJ().Father().InvPm(), *MakI().Father().Pm(), *_pm_M2N);
			s_tf_n(Dim(), -1, *_pm_M2N, *_PrtCstMtxI, 0, *_PrtCstMtxJ);
			
			
			
			/*update A_c*/
			std::fill_n(_a_c, UniversalJoint::CstDim(), 0);
			
			/*calculate a_dot*/
			double v[3];
			v[0] = MakJ().Vel()[3] - MakI().Vel()[3];
			v[1] = MakJ().Vel()[4] - MakI().Vel()[4];
			v[2] = MakJ().Vel()[5] - MakI().Vel()[5];

			double a_dot = MakI().Pm()[0][0] * v[0] + MakI().Pm()[1][0] * v[1] + MakI().Pm()[2][0] * v[2];
			
			/*calculate part m*/
			v[0] = -c*a_dot;
			v[1] = -s*a_dot;

			double _tem_v1[6]{ 0 }, _tem_v2[6]{ 0 };
			s_inv_tv(*MakI().PrtPm(), MakI().Father().PrtVel(), _tem_v1);
			_a_c[3] -= v[0] * _tem_v1[4] + v[1] * _tem_v1[5];
			
			/*calculate part n*/
			s_inv_tv(*_pm_M2N, MakJ().Father().PrtVel(), _tem_v1);
			s_cv(-1, MakI().Father().PrtVel(), _tem_v1, 0, _tem_v2);
			s_dgemmTN(4, 1, 6, 1, *_PrtCstMtxI, Dim(), _tem_v2, 1, 1, &_a_c[0], 1);
			s_inv_tv(*MakI().PrtPm(), _tem_v1, _tem_v2);
			_a_c[3] += v[0] * _tem_v2[4] + v[1] * _tem_v2[5];
		};
		
		const char *const SphericalJoint::type = "spherical";
		SphericalJoint::SphericalJoint(ModelBase &model, const std::string &Name, int id, Marker &makI, Marker &makJ)
			: JointBaseDim(model, Name, id, makI, makJ)
		{
		}
		SphericalJoint::SphericalJoint(ModelBase &model, const std::string &Name, int id, const Aris::Core::XmlElement *ele)
			: JointBaseDim(model, Name, id, ele)
		{
		}
		void SphericalJoint::Init()
		{
			double loc_cst[6][Dim()];
			std::fill_n(static_cast<double*>(*loc_cst), 6 * Dim(), 0);

			loc_cst[0][0] = 1;
			loc_cst[1][1] = 1;
			loc_cst[2][2] = 1;

			s_tf_n(Dim(), *MakI().PrtPm(), *loc_cst, PrtCstMtxI());
		};

		const char *const LinearMotion::type = "linear";
		LinearMotion::LinearMotion(ModelBase &model, const std::string &Name, int id, Marker &makI, Marker &makJ)
			: MotionBase(model, Name, id, makI, makJ)
		{
		}
		LinearMotion::LinearMotion(ModelBase &model, const std::string &Name, int id, const Aris::Core::XmlElement *pEle)
			: MotionBase(model, Name, id, pEle)
		{

		}
		void LinearMotion::Init()
		{
			double loc_cst[6]{0};

			std::fill_n(_PrtCstMtxI, 6, 0);
			std::fill_n(_PrtCstMtxJ, 6, 0);

			/* Get tm I2M */
			loc_cst[2] = 1;
			s_tf(*MakI().PrtPm(), loc_cst, _PrtCstMtxI);
		}
		void LinearMotion::Update()
		{
			/*update motPos motVel,  motAcc should be given, not computed by part acc*/
			MakI().Update();
			MakJ().Update();

			double pm_I2J[4][4];
			s_inv_pm_dot_pm(*MakJ().Pm(), *MakI().Pm(), *pm_I2J);
			motPos = pm_I2J[2][3];

			double velDiff[6],velDiff_in_J[6];
			std::copy_n(MakI().Vel(), 6, velDiff);
			s_daxpy(6, -1, MakJ().Vel(), 1, velDiff, 1);
			s_inv_tv(*MakJ().Pm(), velDiff, velDiff_in_J);
			motVel = velDiff_in_J[2];
			
			/*update cst fce*/
			std::fill_n(_PrtCstMtxJ, 6, 0);
			double pm_M2N[4][4];
			s_pm_dot_pm(*MakJ().Father().InvPm(), *MakI().Father().Pm(), *pm_M2N);
			s_tf(-1, *pm_M2N, _PrtCstMtxI, 0, _PrtCstMtxJ);

			/*update a_c*/
			std::fill_n(_a_c, 6, 0);
			double tem_v1[6]{ 0 }, tem_v2[6]{ 0 };
			s_inv_tv(-1, *pm_M2N, MakJ().Father().PrtVel(), 0, tem_v1);
			s_cv(MakI().Father().PrtVel(), tem_v1, tem_v2);
			s_dgemmTN(1, 1, 6, 1, _PrtCstMtxI, 1, tem_v2, 1, 0, &_a_c[0], 1);

			_a_c[0] += motAcc;

			/*update motPos motVel motAcc*/
		}

		const char *const SingleComponentForce::type = "single_component_force";
		void SingleComponentForce::Update()
		{
			s_tf(*MakI().PrtPm(), fceI, _PrtFceI);
			double pm_M2N[16];
			s_inv_pm_dot_pm(*MakJ().Father().Pm(), *MakI().Father().Pm(), pm_M2N);
			s_tf(-1, pm_M2N, _PrtFceI, 0, _PrtFceJ);
		}
		SingleComponentForce::SingleComponentForce(ModelBase &model, const std::string &name, int id, Marker& makI, Marker& makJ, int componentID)
			: ForceBase(model, name, id, makI, makJ)
			, componentID(componentID)
		{

		}
		SingleComponentForce::SingleComponentForce(ModelBase &model, const std::string &name, int id, const Aris::Core::XmlElement *xmlEle)
			: ForceBase(model, name, id, xmlEle)
			, componentID(std::stoi(xmlEle->Attribute("Component")))
		{
		}
		void SingleComponentForce::ToAdamsCmd(std::ofstream &file) const
		{
			if (fceCurve == nullptr)
			{
				std::string type = "translational";

				file << "force create direct single_component_force  &\r\n"
					<< "    single_component_force_name = ." << Model().Name() << "." << Name() << "  &\r\n"
					<< "    adams_id = " << ID() + 1 << "  &\r\n"
					<< "    type_of_freedom = " << type << "  &\r\n"
					<< "    i_marker_name = ." << Model().Name() << "." << MakI().Father().Name() << "." << MakI().Name() << "  &\r\n"
					<< "    j_marker_name = ." << Model().Name() << "." << MakJ().Father().Name() << "." << MakJ().Name() << "  &\r\n"
					<< "    action_only = off  &\r\n"
					<< "    function = \"" << GetFce() << "\"  \r\n"
					<< "!\r\n";
			}
			else
			{
				std::string type = "translational";

				file << "data_element create spline &\r\n"
					<< "    spline_name = ." << Model().Name() << "." << Name() << "_fce_spl  &\r\n"
					<< "    adams_id = " << ID() * 2 + 1 << "  &\r\n"
					<< "    units = N &\r\n"
					<< "    x = " << fceCurve->x().at(0);
				for (auto p = fceCurve->x().begin() + 1; p < fceCurve->x().end(); ++p)
				{
					file << "," << *p;
				}
				file << "    y = " << fceCurve->y().at(0);
				for (auto p = fceCurve->y().begin() + 1; p < fceCurve->y().end(); ++p)
				{
					file << "," << *p;
				}
				file << " \r\n!\r\n";

				file << "force create direct single_component_force  &\r\n"
					<< "    single_component_force_name = ." << Model().Name() << "." << Name() << "  &\r\n"
					<< "    adams_id = " << ID() + 1 << "  &\r\n"
					<< "    type_of_freedom = " << type << "  &\r\n"
					<< "    i_marker_name = ." << Model().Name() << "." << MakI().Father().Name() << "." << MakI().Name() << "  &\r\n"
					<< "    j_marker_name = ." << Model().Name() << "." << MakJ().Father().Name() << "." << MakJ().Name() << "  &\r\n"
					<< "    action_only = off  &\r\n"
					<< "    function = \"AKISPL(time,0," << Name() << "_fce_spl)\"  \r\n"
					<< "!\r\n";
			}



		}
	
		void Model::LoadXml(const Aris::Core::XmlElement *pModel)
		{
			ModelBase::LoadXml(pModel);
			
			const Aris::Core::XmlElement *pJnt = pModel->FirstChildElement("Joint");
			if (pJnt == nullptr)throw(std::logic_error("Model must have joint element"));
			const Aris::Core::XmlElement *pMot = pModel->FirstChildElement("Motion");
			if (pMot == nullptr)throw(std::logic_error("Model must have motion element"));
			const Aris::Core::XmlElement *pFce = pModel->FirstChildElement("Force");
			if (pFce == nullptr)throw(std::logic_error("Model must have force element"));
			
			for (auto ele = pJnt->FirstChildElement(); ele != nullptr; ele = ele->NextSiblingElement())
			{
				if (strcmp(TranslationalJoint::type, ele->Attribute("Type")) == 0)
				{
					AddJoint<TranslationalJoint>(ele->Name(), ele);
				}
				if (strcmp(SphericalJoint::type, ele->Attribute("Type")) == 0)
				{
					AddJoint<SphericalJoint>(ele->Name(), ele);
				}
				if (strcmp(UniversalJoint::type, ele->Attribute("Type")) == 0)
				{
					AddJoint<UniversalJoint>(ele->Name(), ele);
				}

				InitiateElement(GetJoint(ele->Name()));
			}

			for (auto ele = pMot->FirstChildElement(); ele != nullptr; ele = ele->NextSiblingElement())
			{
				if (strcmp(LinearMotion::type, ele->Attribute("Type")) == 0)
				{
					AddMotion<LinearMotion>(ele->Name(), ele);
				}
				InitiateElement(GetMotion(ele->Name()));
			}

			for (auto ele = pFce->FirstChildElement(); ele != nullptr; ele = ele->NextSiblingElement())
			{
				if (strcmp(SingleComponentForce::type, ele->Attribute("Type")) == 0)
				{
					AddForce<SingleComponentForce>(ele->Name(), ele);
				}
			}
		}
	}
}
