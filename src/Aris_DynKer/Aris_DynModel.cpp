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
		RevoluteJoint::RevoluteJoint(ModelBase &model, const std::string &name, int id, Marker &makI, Marker &makJ)
			: JointBaseDim(model, name, id, makI, makJ)
		{
		}
		RevoluteJoint::RevoluteJoint(ModelBase &model, const std::string &name, int id, const Aris::Core::XmlElement &xml_ele)
			: JointBaseDim(model, name, id, xml_ele)
		{
		}
		void RevoluteJoint::Init() 
		{
			double loc_cst[6][Dim()];
			std::fill_n(&loc_cst[0][0], 6 * Dim(), 0);

			loc_cst[0][0] = 1;
			loc_cst[1][1] = 1;
			loc_cst[2][2] = 1;
			loc_cst[3][3] = 1;
			loc_cst[4][4] = 1;

			s_tf_n(Dim(), *MakI().PrtPm(), *loc_cst, CsmI());
		}
		
		TranslationalJoint::TranslationalJoint(ModelBase &model, const std::string &name, int id, Marker &makI, Marker &makJ)
			: JointBaseDim(model, name, id, makI, makJ)
		{
		}
		TranslationalJoint::TranslationalJoint(ModelBase &model, const std::string &name, int id, const Aris::Core::XmlElement &xml_ele)
			: JointBaseDim(model, name, id, xml_ele)
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

			s_tf_n(Dim(), *MakI().PrtPm(), *loc_cst, CsmI());
		};
		
		UniversalJoint::UniversalJoint(ModelBase &model, const std::string &name, int id, Marker &makI, Marker &makJ)
			: JointBaseDim(model, name, id, makI, makJ)
		{
		}
		UniversalJoint::UniversalJoint(ModelBase &model, const std::string &name, int id, const Aris::Core::XmlElement &xml_ele)
			: JointBaseDim(model, name, id, xml_ele)
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

			Constraint::ToAdamsCmd(file);
		}
		void UniversalJoint::Init()
		{
			double loc_cst[6][Dim()];
			std::fill_n(static_cast<double*>(*loc_cst), 6 * Dim(), 0);

			loc_cst[0][0] = 1;
			loc_cst[1][1] = 1;
			loc_cst[2][2] = 1;

			s_tf_n(Dim(), *MakI().PrtPm(), *loc_cst, CsmI());

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
			
			csmI_[3][3] = -(MakI().PrtPm()[0][1]) * s + (MakI().PrtPm()[0][2]) * c;
			csmI_[4][3] = -(MakI().PrtPm()[1][1]) * s + (MakI().PrtPm()[1][2]) * c;
			csmI_[5][3] = -(MakI().PrtPm()[2][1]) * s + (MakI().PrtPm()[2][2]) * c;

			/*edit CstMtxJ*/
			std::fill_n(static_cast<double *>(CsmJ()), this->CstDim() * 6, 0);
			double _pm_M2N[4][4];
			s_pm_dot_pm(*MakJ().Father().InvPm(), *MakI().Father().Pm(), *_pm_M2N);
			s_tf_n(Dim(), -1, *_pm_M2N, CsmI(), 0, CsmJ());
			
			
			
			/*update A_c*/
			std::fill_n(Csa(), UniversalJoint::CstDim(), 0);
			
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
			Csa()[3] -= v[0] * _tem_v1[4] + v[1] * _tem_v1[5];
			
			/*calculate part n*/
			s_inv_tv(*_pm_M2N, MakJ().Father().PrtVel(), _tem_v1);
			s_cv(-1, MakI().Father().PrtVel(), _tem_v1, 0, _tem_v2);
			s_dgemmTN(4, 1, 6, 1, CsmI(), Dim(), _tem_v2, 1, 1, &Csa()[0], 1);
			s_inv_tv(*MakI().PrtPm(), _tem_v1, _tem_v2);
			Csa()[3] += v[0] * _tem_v2[4] + v[1] * _tem_v2[5];
		};
		
		SphericalJoint::SphericalJoint(ModelBase &model, const std::string &name, int id, Marker &makI, Marker &makJ)
			: JointBaseDim(model, name, id, makI, makJ)
		{
		}
		SphericalJoint::SphericalJoint(ModelBase &model, const std::string &name, int id, const Aris::Core::XmlElement &xml_ele)
			: JointBaseDim(model, name, id, xml_ele)
		{
		}
		void SphericalJoint::Init()
		{
			double loc_cst[6][Dim()];
			std::fill_n(static_cast<double*>(*loc_cst), 6 * Dim(), 0);

			loc_cst[0][0] = 1;
			loc_cst[1][1] = 1;
			loc_cst[2][2] = 1;

			s_tf_n(Dim(), *MakI().PrtPm(), *loc_cst, CsmI());
		};

		SingleComponentMotion::SingleComponentMotion(ModelBase &model, const std::string &name, int id, Marker &makI, Marker &makJ, int component_axis)
			: MotionBase(model, name, id, makI, makJ)
			, component_axis_(component_axis)
		{
		}
		SingleComponentMotion::SingleComponentMotion(ModelBase &model, const std::string &name, int id, const Aris::Core::XmlElement &xml_ele)
			: MotionBase(model, name, id, xml_ele)
		{
			component_axis_ = std::stoi(xml_ele.Attribute("Component"));
		}
		void SingleComponentMotion::Init()
		{
			double loc_cst[6]{ 0 };

			std::fill_n(CsmI(), 6, 0);
			std::fill_n(CsmJ(), 6, 0);

			/* Get tm I2M */
			loc_cst[component_axis_] = 1;
			s_tf(*MakI().PrtPm(), loc_cst, CsmI());
		}
		void SingleComponentMotion::Update()
		{
			/*update motPos motVel,  motAcc should be given, not computed by part acc*/
			MakI().Update();
			MakJ().Update();

			double pm_I2J[4][4], pe[6];
			s_inv_pm_dot_pm(*MakJ().Pm(), *MakI().Pm(), *pm_I2J);
			s_pm2pe(&pm_I2J[0][0], pe,"123");
			mot_pos_ = pe[component_axis_];

			double velDiff[6], velDiff_in_J[6];
			std::copy_n(MakI().Vel(), 6, velDiff);
			s_daxpy(6, -1, MakJ().Vel(), 1, velDiff, 1);
			s_inv_tv(*MakJ().Pm(), velDiff, velDiff_in_J);
			mot_vel_ = velDiff_in_J[component_axis_];

			/*update cst mtx*/
			std::fill_n(CsmJ(), 6, 0);
			double pm_M2N[4][4];
			s_pm_dot_pm(*MakJ().Father().InvPm(), *MakI().Father().Pm(), *pm_M2N);
			s_tf(-1, *pm_M2N, CsmI(), 0, CsmJ());

			/*update a_c*/
			std::fill_n(Csa(), 1, 0);
			double tem_v1[6]{ 0 }, tem_v2[6]{ 0 };
			s_inv_tv(-1, *pm_M2N, MakJ().Father().PrtVel(), 0, tem_v1);
			s_cv(MakI().Father().PrtVel(), tem_v1, tem_v2);
			s_dgemmTN(1, 1, 6, 1, CsmI(), 1, tem_v2, 1, 0, Csa(), 1);

			Csa()[0] += mot_acc_;
			/*update motPos motVel motAcc*/
		}
		void SingleComponentMotion::ToAdamsCmd(std::ofstream &file) const
		{
			std::string s;

			switch (component_axis_)
			{
			case 0:
				s = "x";
				break;
			case 1:
				s = "y";
				break;
			case 2:
				s = "z";
				break;
			case 3:
				s = "B1";
				break;
			case 4:
				s = "B2";
				break;
			case 5:
				s = "B3";
				break;
			}
			
			
			if (this->posCurve == nullptr)
			{
				file << "constraint create motion_generator &\r\n"
					<< "    motion_name = ." << Model().Name() << "." << this->Name() << "  &\r\n"
					<< "    adams_id = " << this->ID() + 1 << "  &\r\n"
					<< "    i_marker_name = ." << Model().Name() << "." << this->MakI().Father().Name() << "." << this->MakI().Name() << "  &\r\n"
					<< "    j_marker_name = ." << Model().Name() << "." << this->MakJ().Father().Name() << "." << this->MakJ().Name() << "  &\r\n"
					<< "    axis = " << s << "  &\r\n"
					<< "    function = \"" << this->MotPos() << "\"  \r\n"
					<< "!\r\n";
			}
			else
			{
				file << "data_element create spline &\r\n"
					<< "    spline_name = ." << Model().Name() << "." << this->Name() << "_pos_spl  &\r\n"
					<< "    adams_id = " << this->ID() * 2 << "  &\r\n"
					<< "    units = m &\r\n"
					<< "    x = " << this->posCurve->x().at(0);
				for (auto p = this->posCurve->x().begin() + 1; p < this->posCurve->x().end(); ++p)
				{
					file << "," << *p;
				}
				file << "    y = " << this->posCurve->y().at(0);
				for (auto p = this->posCurve->y().begin() + 1; p < this->posCurve->y().end(); ++p)
				{
					file << "," << *p;
				}
				file << " \r\n!\r\n";

				file << "constraint create motion_generator &\r\n"
					<< "    motion_name = ." << Model().Name() << "." << this->Name() << "  &\r\n"
					<< "    adams_id = " << this->ID() + 1 << "  &\r\n"
					<< "    i_marker_name = ." << Model().Name() << "." << this->MakI().Father().Name() << "." << this->MakI().Name() << "  &\r\n"
					<< "    j_marker_name = ." << Model().Name() << "." << this->MakJ().Father().Name() << "." << this->MakJ().Name() << "  &\r\n"
					<< "    axis = " << s << "  &\r\n"
					<< "    function = \"AKISPL(time,0," << this->Name() << "_pos_spl)\"  \r\n"
					<< "!\r\n";
			}
		}

		void SingleComponentForce::Update()
		{
			s_tf(*MakI().PrtPm(), fceI, fceI_);
			double pm_M2N[16];
			s_inv_pm_dot_pm(*MakJ().Father().Pm(), *MakI().Father().Pm(), pm_M2N);
			s_tf(-1, pm_M2N, fceI_, 0, fceJ_);
		}
		SingleComponentForce::SingleComponentForce(ModelBase &model, const std::string &name, int id, Marker& makI, Marker& makJ, int componentID)
			: ForceBase(model, name, id, makI, makJ)
			, componentID(componentID)
		{

		}
		SingleComponentForce::SingleComponentForce(ModelBase &model, const std::string &name, int id, const Aris::Core::XmlElement &xml_ele)
			: ForceBase(model, name, id, xml_ele)
			, componentID(std::stoi(xml_ele.Attribute("Component")))
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
					<< "    function = \"" << Fce() << "\"  \r\n"
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
	
		void Model::LoadXml(const Aris::Core::XmlElement &xml_ele)
		{
			ModelBase::LoadXml(xml_ele);
			
			auto pJnt = xml_ele.FirstChildElement("Joint");
			if (!pJnt)throw(std::logic_error("Model must have joint element"));
			auto pMot = xml_ele.FirstChildElement("Motion");
			if (!pMot)throw(std::logic_error("Model must have motion element"));
			auto pFce = xml_ele.FirstChildElement("Force");
			if (!pFce)throw(std::logic_error("Model must have force element"));
			
			for (auto ele = pJnt->FirstChildElement(); ele != nullptr; ele = ele->NextSiblingElement())
			{
				JointBase *pJnt;

				if (RevoluteJoint::Type() == ele->Attribute("Type"))
				{
					pJnt = AddJoint<RevoluteJoint>(ele->Name(), std::ref(*ele));
				}
				else if (TranslationalJoint::Type() == ele->Attribute("Type"))
				{
					pJnt = AddJoint<TranslationalJoint>(ele->Name(), std::ref(*ele));
				}
				else if (SphericalJoint::Type() == ele->Attribute("Type"))
				{
					pJnt = AddJoint<SphericalJoint>(ele->Name(), std::ref(*ele));
				}
				else if (UniversalJoint::Type() == ele->Attribute("Type"))
				{
					pJnt = AddJoint<UniversalJoint>(ele->Name(), std::ref(*ele));
				}
				else
				{
					throw std::runtime_error(std::string("invalid joint type in Aris::Model::LoadXml of \"") + ele->Name() + "\"");
				}

				if (pJnt)
					pJnt->Init();
				else 
					throw std::runtime_error(std::string("failed to add joint \"")+ ele->Name() +"\", because it already has one");
			}

			for (auto ele = pMot->FirstChildElement(); ele != nullptr; ele = ele->NextSiblingElement())
			{
				MotionBase *pMot;
				
				if (SingleComponentMotion::Type() == ele->Attribute("Type"))
				{
					pMot = AddMotion<SingleComponentMotion>(ele->Name(), std::ref(*ele));
				}
				else
				{
					throw std::runtime_error(std::string("invalid motion type in Aris::Model::LoadXml of \"") + ele->Name() + "\"");
				}

				if (pMot)
					pMot->Init();
				else
					throw std::runtime_error(std::string("failed to add motion \"") + ele->Name() + "\", because it already has one");
			}

			for (auto ele = pFce->FirstChildElement(); ele != nullptr; ele = ele->NextSiblingElement())
			{
				ForceBase *pFce;
				
				if (SingleComponentForce::Type()==ele->Attribute("Type")) 
				{
					pFce = AddForce<SingleComponentForce>(ele->Name(), std::ref(*ele));
				}
				else
				{
					throw std::runtime_error(std::string("invalid force type in Aris::Model::LoadXml of \"") + ele->Name() + "\"");
				}

				if(!pFce)throw std::runtime_error(std::string("failed to add force \"") + ele->Name() + "\", because it already has one");
			}
		}
	}
}
