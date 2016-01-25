#include <cmath>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <algorithm>
#include <limits>
#include <sstream>

#include "aris_core.h"
#include "aris_dynamic_model.h"

namespace Aris
{
	namespace Dynamic
	{
		RevoluteJoint::RevoluteJoint(ModelBase &model, const std::string &name, int id, Marker &makI, Marker &makJ)
			: JointBaseDim(model, name, id, makI, makJ)
		{
		}
		RevoluteJoint::RevoluteJoint(ModelBase &model, const std::string &name, int id, const Aris::Core::XmlElement &xml_ele)
			: JointBaseDim(model, name, id, xml_ele)
		{
		}
		void RevoluteJoint::init() 
		{
			double loc_cst[6][dim()];
			std::fill_n(&loc_cst[0][0], 6 * dim(), 0);

			loc_cst[0][0] = 1;
			loc_cst[1][1] = 1;
			loc_cst[2][2] = 1;
			loc_cst[3][3] = 1;
			loc_cst[4][4] = 1;

			s_tf_n(dim(), *makI().prtPm(), *loc_cst, csmI());
		}
		
		TranslationalJoint::TranslationalJoint(ModelBase &model, const std::string &name, int id, Marker &makI, Marker &makJ)
			: JointBaseDim(model, name, id, makI, makJ)
		{
		}
		TranslationalJoint::TranslationalJoint(ModelBase &model, const std::string &name, int id, const Aris::Core::XmlElement &xml_ele)
			: JointBaseDim(model, name, id, xml_ele)
		{
		}
		void TranslationalJoint::init()
		{
			double loc_cst[6][dim()];
			std::fill_n(static_cast<double*>(*loc_cst), 6 * dim(), 0);

			loc_cst[0][0] = 1;
			loc_cst[1][1] = 1;
			loc_cst[3][2] = 1;
			loc_cst[4][3] = 1;
			loc_cst[5][4] = 1;

			s_tf_n(dim(), *makI().prtPm(), *loc_cst, csmI());
		};
		
		UniversalJoint::UniversalJoint(ModelBase &model, const std::string &name, int id, Marker &makI, Marker &makJ)
			: JointBaseDim(model, name, id, makI, makJ)
		{
		}
		UniversalJoint::UniversalJoint(ModelBase &model, const std::string &name, int id, const Aris::Core::XmlElement &xml_ele)
			: JointBaseDim(model, name, id, xml_ele)
		{
		}
		void UniversalJoint::toAdamsCmd(std::ofstream &file) const
		{
			double pe[6] = { 0, 0, 0, PI / 2, 0, 0 };
			double pe2[6] = { 0, 0, 0, -PI / 2, 0, 0 };
			double pm[4][4], pm2[4][4];
			
			s_pe2pm(pe, *pm, "213");
			s_pm_dot_pm(*this->makI().prtPm(), *pm, *pm2);
			s_pm2pe(*pm2, pe, "313");

			file << "marker modify &\r\n"
				<< "    marker_name = ." << model().name() << "." << this->makI().father().name() << "." << this->makI().name() << " &\r\n"
				<< "    orientation = (" << Core::Matrix(1, 3, &pe[3]).toString() << ") \r\n"
				<< "!\r\n";

			s_pe2pm(pe2, *pm, "123");
			s_pm_dot_pm(*this->makJ().prtPm(), *pm, *pm2);
			s_pm2pe(*pm2, pe, "313");

			file << "marker modify &\r\n"
				<< "    marker_name = ." << model().name() << "." << this->makJ().father().name() << "." << this->makJ().name() << " &\r\n"
				<< "    orientation = (" << Core::Matrix(1, 3, &pe[3]).toString() << ") \r\n"
				<< "!\r\n";

			Constraint::toAdamsCmd(file);
		}
		void UniversalJoint::init()
		{
			double loc_cst[6][dim()];
			std::fill_n(static_cast<double*>(*loc_cst), 6 * dim(), 0);

			loc_cst[0][0] = 1;
			loc_cst[1][1] = 1;
			loc_cst[2][2] = 1;

			s_tf_n(dim(), *makI().prtPm(), *loc_cst, csmI());

		};
		void UniversalJoint::update()
		{
			/*update PrtCstMtxI*/
			makI().update();
			makJ().update();

			//get sin(a) and cos(a)
			double s = makI().pm()[0][2] * makJ().pm()[0][1]
				+ makI().pm()[1][2] * makJ().pm()[1][1]
				+ makI().pm()[2][2] * makJ().pm()[2][1];

			double c = makI().pm()[0][1] * makJ().pm()[0][1]
				+ makI().pm()[1][1] * makJ().pm()[1][1]
				+ makI().pm()[2][1] * makJ().pm()[2][1];
			
			csmI_[3][3] = -(makI().prtPm()[0][1]) * s + (makI().prtPm()[0][2]) * c;
			csmI_[4][3] = -(makI().prtPm()[1][1]) * s + (makI().prtPm()[1][2]) * c;
			csmI_[5][3] = -(makI().prtPm()[2][1]) * s + (makI().prtPm()[2][2]) * c;

			/*edit CstMtxJ*/
			std::fill_n(static_cast<double *>(csmJ()), this->cstDim() * 6, 0);
			double pm_M2N[4][4];
			s_pm_dot_pm(*makJ().father().invPm(), *makI().father().pm(), *pm_M2N);
			s_tf_n(dim(), -1, *pm_M2N, csmI(), 0, csmJ());
			
			
			
			/*update A_c*/
			std::fill_n(csa(), UniversalJoint::cstDim(), 0);
			
			/*calculate a_dot*/
			double v[3];
			v[0] = makJ().vel()[3] - makI().vel()[3];
			v[1] = makJ().vel()[4] - makI().vel()[4];
			v[2] = makJ().vel()[5] - makI().vel()[5];

			double a_dot = makI().pm()[0][0] * v[0] + makI().pm()[1][0] * v[1] + makI().pm()[2][0] * v[2];
			
			/*calculate part m*/
			v[0] = -c*a_dot;
			v[1] = -s*a_dot;

			double tem_v1[6]{ 0 }, tem_v2[6]{ 0 };
			s_inv_tv(*makI().prtPm(), makI().father().prtVel(), tem_v1);
			csa()[3] -= v[0] * tem_v1[4] + v[1] * tem_v1[5];
			
			/*calculate part n*/
			s_inv_tv(*pm_M2N, makJ().father().prtVel(), tem_v1);
			s_cv(-1, makI().father().prtVel(), tem_v1, 0, tem_v2);
			s_dgemmTN(4, 1, 6, 1, csmI(), dim(), tem_v2, 1, 1, &csa()[0], 1);
			s_inv_tv(*makI().prtPm(), tem_v1, tem_v2);
			csa()[3] += v[0] * tem_v2[4] + v[1] * tem_v2[5];
		};
		
		SphericalJoint::SphericalJoint(ModelBase &model, const std::string &name, int id, Marker &makI, Marker &makJ)
			: JointBaseDim(model, name, id, makI, makJ)
		{
		}
		SphericalJoint::SphericalJoint(ModelBase &model, const std::string &name, int id, const Aris::Core::XmlElement &xml_ele)
			: JointBaseDim(model, name, id, xml_ele)
		{
		}
		void SphericalJoint::init()
		{
			double loc_cst[6][dim()];
			std::fill_n(static_cast<double*>(*loc_cst), 6 * dim(), 0);

			loc_cst[0][0] = 1;
			loc_cst[1][1] = 1;
			loc_cst[2][2] = 1;

			s_tf_n(dim(), *makI().prtPm(), *loc_cst, csmI());
		};

		SingleComponentMotion::SingleComponentMotion(ModelBase &model, const std::string &name, int id, Marker &makI, Marker &makJ, int component_axis)
			: MotionBase(model, name, id, makI, makJ), component_axis_(component_axis)
		{
		}
		SingleComponentMotion::SingleComponentMotion(ModelBase &model, const std::string &name, int id, const Aris::Core::XmlElement &xml_ele)
			: MotionBase(model, name, id, xml_ele)
		{
			component_axis_ = std::stoi(xml_ele.Attribute("Component"));
		}
		void SingleComponentMotion::init()
		{
			double loc_cst[6]{ 0 };

			std::fill_n(csmI(), 6, 0);
			std::fill_n(csmJ(), 6, 0);

			/* Get tm I2M */
			loc_cst[component_axis_] = 1;
			s_tf(*makI().prtPm(), loc_cst, csmI());
		}
		void SingleComponentMotion::update()
		{
			/*update motPos motVel,  motAcc should be given, not computed by part acc*/
			makI().update();
			makJ().update();

			double pm_I2J[4][4], pe[6];
			s_inv_pm_dot_pm(*makJ().pm(), *makI().pm(), *pm_I2J);
			s_pm2pe(&pm_I2J[0][0], pe,"123");
			mot_pos_ = pe[component_axis_];

			double velDiff[6], velDiff_in_J[6];
			std::copy_n(makI().vel(), 6, velDiff);
			s_daxpy(6, -1, makJ().vel(), 1, velDiff, 1);
			s_inv_tv(*makJ().pm(), velDiff, velDiff_in_J);
			mot_vel_ = velDiff_in_J[component_axis_];

			/*update cst mtx*/
			std::fill_n(csmJ(), 6, 0);
			double pm_M2N[4][4];
			s_pm_dot_pm(*makJ().father().invPm(), *makI().father().pm(), *pm_M2N);
			s_tf(-1, *pm_M2N, csmI(), 0, csmJ());

			/*update a_c*/
			std::fill_n(csa(), 1, 0);
			double tem_v1[6]{ 0 }, tem_v2[6]{ 0 };
			s_inv_tv(-1, *pm_M2N, makJ().father().prtVel(), 0, tem_v1);
			s_cv(makI().father().prtVel(), tem_v1, tem_v2);
			s_dgemmTN(1, 1, 6, 1, csmI(), 1, tem_v2, 1, 0, csa(), 1);

			csa()[0] += mot_acc_;
			/*update motPos motVel motAcc*/
		}
		void SingleComponentMotion::toAdamsCmd(std::ofstream &file) const
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
					<< "    motion_name = ." << model().name() << "." << this->name() << "  &\r\n"
					<< "    adams_id = " << this->id() + 1 << "  &\r\n"
					<< "    i_marker_name = ." << model().name() << "." << this->makI().father().name() << "." << this->makI().name() << "  &\r\n"
					<< "    j_marker_name = ." << model().name() << "." << this->makJ().father().name() << "." << this->makJ().name() << "  &\r\n"
					<< "    axis = " << s << "  &\r\n"
					<< "    function = \"" << this->motPos() << "\"  \r\n"
					<< "!\r\n";
			}
			else
			{
				file << "data_element create spline &\r\n"
					<< "    spline_name = ." << model().name() << "." << this->name() << "_pos_spl  &\r\n"
					<< "    adams_id = " << this->id() * 2 << "  &\r\n"
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
					<< "    motion_name = ." << model().name() << "." << this->name() << "  &\r\n"
					<< "    adams_id = " << this->id() + 1 << "  &\r\n"
					<< "    i_marker_name = ." << model().name() << "." << this->makI().father().name() << "." << this->makI().name() << "  &\r\n"
					<< "    j_marker_name = ." << model().name() << "." << this->makJ().father().name() << "." << this->makJ().name() << "  &\r\n"
					<< "    axis = " << s << "  &\r\n"
					<< "    function = \"AKISPL(time,0," << this->name() << "_pos_spl)\"  \r\n"
					<< "!\r\n";
			}
		}

		void SingleComponentForce::update()
		{
			s_tf(*makI().prtPm(), fce_value_, fceI_);
			double pm_M2N[16];
			s_inv_pm_dot_pm(*makJ().father().pm(), *makI().father().pm(), pm_M2N);
			s_tf(-1, pm_M2N, fceI_, 0, fceJ_);
		}
		SingleComponentForce::SingleComponentForce(ModelBase &model, const std::string &name, int id, Marker& makI, Marker& makJ, int componentID)
			: ForceBase(model, name, id, makI, makJ), component_axis_(componentID)
		{

		}
		SingleComponentForce::SingleComponentForce(ModelBase &model, const std::string &name, int id, const Aris::Core::XmlElement &xml_ele)
			: ForceBase(model, name, id, xml_ele), component_axis_(std::stoi(xml_ele.Attribute("Component")))
		{
		}
		void SingleComponentForce::toAdamsCmd(std::ofstream &file) const
		{
			if (fce_akima_ == nullptr)
			{
				std::string type = "translational";

				file << "force create direct single_component_force  &\r\n"
					<< "    single_component_force_name = ." << model().name() << "." << name() << "  &\r\n"
					<< "    adams_id = " << id() + 1 << "  &\r\n"
					<< "    type_of_freedom = " << type << "  &\r\n"
					<< "    i_marker_name = ." << model().name() << "." << makI().father().name() << "." << makI().name() << "  &\r\n"
					<< "    j_marker_name = ." << model().name() << "." << makJ().father().name() << "." << makJ().name() << "  &\r\n"
					<< "    action_only = off  &\r\n"
					<< "    function = \"" << fce() << "\"  \r\n"
					<< "!\r\n";
			}
			else
			{
				std::string type = "translational";

				file << "data_element create spline &\r\n"
					<< "    spline_name = ." << model().name() << "." << name() << "_fce_spl  &\r\n"
					<< "    adams_id = " << id() * 2 + 1 << "  &\r\n"
					<< "    units = N &\r\n"
					<< "    x = " << fce_akima_->x().at(0);
				for (auto p = fce_akima_->x().begin() + 1; p < fce_akima_->x().end(); ++p)
				{
					file << "," << *p;
				}
				file << "    y = " << fce_akima_->y().at(0);
				for (auto p = fce_akima_->y().begin() + 1; p < fce_akima_->y().end(); ++p)
				{
					file << "," << *p;
				}
				file << " \r\n!\r\n";

				file << "force create direct single_component_force  &\r\n"
					<< "    single_component_force_name = ." << model().name() << "." << name() << "  &\r\n"
					<< "    adams_id = " << id() + 1 << "  &\r\n"
					<< "    type_of_freedom = " << type << "  &\r\n"
					<< "    i_marker_name = ." << model().name() << "." << makI().father().name() << "." << makI().name() << "  &\r\n"
					<< "    j_marker_name = ." << model().name() << "." << makJ().father().name() << "." << makJ().name() << "  &\r\n"
					<< "    action_only = off  &\r\n"
					<< "    function = \"AKISPL(time,0," << name() << "_fce_spl)\"  \r\n"
					<< "!\r\n";
			}



		}
	
		void Model::loadXml(const Aris::Core::XmlElement &xml_ele)
		{
			ModelBase::loadXml(xml_ele);
			
			auto pJnt = xml_ele.FirstChildElement("Joint");
			if (!pJnt)throw(std::logic_error("Model must have joint element"));
			auto pMot = xml_ele.FirstChildElement("Motion");
			if (!pMot)throw(std::logic_error("Model must have motion element"));
			auto pFce = xml_ele.FirstChildElement("Force");
			if (!pFce)throw(std::logic_error("Model must have force element"));
			
			for (auto ele = pJnt->FirstChildElement(); ele != nullptr; ele = ele->NextSiblingElement())
			{
				JointBase *pJnt;

				if (RevoluteJoint::type() == ele->Attribute("Type"))
				{
					pJnt = addJoint<RevoluteJoint>(ele->name(), std::ref(*ele));
				}
				else if (TranslationalJoint::type() == ele->Attribute("Type"))
				{
					pJnt = addJoint<TranslationalJoint>(ele->name(), std::ref(*ele));
				}
				else if (SphericalJoint::type() == ele->Attribute("Type"))
				{
					pJnt = addJoint<SphericalJoint>(ele->name(), std::ref(*ele));
				}
				else if (UniversalJoint::type() == ele->Attribute("Type"))
				{
					pJnt = addJoint<UniversalJoint>(ele->name(), std::ref(*ele));
				}
				else
				{
					throw std::runtime_error(std::string("invalid joint type in Aris::Model::LoadXml of \"") + ele->name() + "\"");
				}

				if (pJnt)
					pJnt->init();
				else 
					throw std::runtime_error(std::string("failed to add joint \"")+ ele->name() +"\", because it already has one");
			}

			for (auto ele = pMot->FirstChildElement(); ele != nullptr; ele = ele->NextSiblingElement())
			{
				MotionBase *pMot;
				
				if (SingleComponentMotion::type() == ele->Attribute("Type"))
				{
					pMot = addMotion<SingleComponentMotion>(ele->name(), std::ref(*ele));
				}
				else
				{
					throw std::runtime_error(std::string("invalid motion type in Aris::Model::LoadXml of \"") + ele->name() + "\"");
				}

				if (pMot)
					pMot->init();
				else
					throw std::runtime_error(std::string("failed to add motion \"") + ele->name() + "\", because it already has one");
			}

			for (auto ele = pFce->FirstChildElement(); ele != nullptr; ele = ele->NextSiblingElement())
			{
				ForceBase *pFce;
				
				if (SingleComponentForce::type()==ele->Attribute("Type")) 
				{
					pFce = addForce<SingleComponentForce>(ele->name(), std::ref(*ele));
				}
				else
				{
					throw std::runtime_error(std::string("invalid force type in Aris::Model::LoadXml of \"") + ele->name() + "\"");
				}

				if(!pFce)throw std::runtime_error(std::string("failed to add force \"") + ele->name() + "\", because it already has one");
			}
		}
	}
}
