#include <Platform.h>

#ifdef PLATFORM_IS_WINDOWS
#define _SCL_SECURE_NO_WARNINGS
#endif

#include <cmath>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <algorithm>
#include <limits>
#include <sstream>

using namespace std;


#include <Aris_DynModel.h>

#ifdef PLATFORM_IS_WINDOWS
#include <stdlib.h>
#endif
#ifdef PLATFORM_IS_LINUX
#include <stdlib.h>
#endif

extern "C"
{
#include <cblas.h>
}
#include <lapacke.h>

namespace Aris
{
	namespace DynKer
	{
		const char *const TRANSLATIONAL_JOINT::type = "translational";
		TRANSLATIONAL_JOINT::TRANSLATIONAL_JOINT(MODEL_BASE *pModel, const std::string &Name, int id, MARKER *pMakI, MARKER *pMakJ)
			: JOINT_BASE_DIM(pModel, Name, id, pMakI, pMakJ)
		{
		}
		TRANSLATIONAL_JOINT::TRANSLATIONAL_JOINT(MODEL_BASE *pModel, const std::string &Name, int id, const Aris::Core::ELEMENT *ele)
			: JOINT_BASE_DIM(pModel, Name, id, ele)
		{
		}
		void TRANSLATIONAL_JOINT::Initiate()
		{
			double loc_cst[6][GetDim()];
			std::fill_n(static_cast<double*>(*loc_cst), 6 * GetDim(), 0);

			loc_cst[0][0] = 1;
			loc_cst[1][1] = 1;
			loc_cst[3][2] = 1;
			loc_cst[4][3] = 1;
			loc_cst[5][4] = 1;

			s_tf_n(GetDim(), GetMakI()->GetPrtPmPtr(), *loc_cst, GetPrtCstMtxIPtr());
		};
		
		const char *const UNIVERSAL_JOINT::type = "universal";
		UNIVERSAL_JOINT::UNIVERSAL_JOINT(MODEL_BASE *pModel, const std::string &Name, int id, MARKER *pMakI, MARKER *pMakJ)
			: JOINT_BASE_DIM(pModel, Name, id, pMakI, pMakJ)
		{
		}
		UNIVERSAL_JOINT::UNIVERSAL_JOINT(MODEL_BASE *pModel, const std::string &Name, int id, const Aris::Core::ELEMENT *ele)
			: JOINT_BASE_DIM(pModel, Name, id, ele)
		{
		}
		void UNIVERSAL_JOINT::ToAdamsCmd(std::ofstream &file) const
		{
			double pe[6] = { 0, 0, 0, PI / 2, 0, 0 };
			double pe2[6] = { 0, 0, 0, -PI / 2, 0, 0 };
			double pm[4][4], pm2[4][4];
			
			s_pe2pm(pe, *pm, "213");
			s_pm_dot_pm(this->GetMakI()->GetPrtPmPtr(), *pm, *pm2);
			s_pm2pe(*pm2, pe, "313");

			file << "marker modify &\r\n"
				<< "    marker_name = ." << Model()->Name() << "." << this->GetMakI()->GetFatherPrt()->Name() << "." << this->GetMakI()->Name() << " &\r\n"
				<< "    orientation = (" << MATRIX(1, 3, &pe[3]).ToString() << ") \r\n"
				<< "!\r\n";

			s_pe2pm(pe2, *pm, "123");
			s_pm_dot_pm(this->GetMakJ()->GetPrtPmPtr(), *pm, *pm2);
			s_pm2pe(*pm2, pe, "313");

			file << "marker modify &\r\n"
				<< "    marker_name = ." << Model()->Name() << "." << this->GetMakJ()->GetFatherPrt()->Name() << "." << this->GetMakJ()->Name() << " &\r\n"
				<< "    orientation = (" << MATRIX(1, 3, &pe[3]).ToString() << ") \r\n"
				<< "!\r\n";

			JOINT_BASE::ToAdamsCmd(file);
		}
		void UNIVERSAL_JOINT::Initiate()
		{
			double loc_cst[6][GetDim()];
			std::fill_n(static_cast<double*>(*loc_cst), 6 * GetDim(), 0);

			loc_cst[0][0] = 1;
			loc_cst[1][1] = 1;
			loc_cst[2][2] = 1;

			s_tf_n(GetDim(), GetMakI()->GetPrtPmPtr(), *loc_cst, GetPrtCstMtxIPtr());

		};
		void UNIVERSAL_JOINT::Update()
		{
			double _pm_M2N[4][4];
			double _tem_v1[6], _tem_v2[6];

			/* Get pm M2N */
			s_pm_dot_pm(GetMakJ()->GetFatherPrt()->GetPrtPmPtr(), GetMakI()->GetFatherPrt()->GetPmPtr(), *_pm_M2N);

			double v[3];
			double a, a_dot;

			double _tm_M2N[6][6];
			double _tm_I2M[6][6];
			s_tmf(*_pm_M2N, *_tm_M2N);
			s_tmf(GetMakI()->GetPrtPmPtr(), *_tm_I2M);

			/*update PrtCstMtx*/
			//get sin(a) and cos(a)
			GetMakI()->Update();
			GetMakJ()->Update();

			double s = GetMakI()->GetPmPtr()[2] * GetMakJ()->GetPmPtr()[1]
				+ GetMakI()->GetPmPtr()[6] * GetMakJ()->GetPmPtr()[5]
				+ GetMakI()->GetPmPtr()[10] * GetMakJ()->GetPmPtr()[9];

			double c = GetMakI()->GetPmPtr()[1] * GetMakJ()->GetPmPtr()[1]
				+ GetMakI()->GetPmPtr()[5] * GetMakJ()->GetPmPtr()[5]
				+ GetMakI()->GetPmPtr()[9] * GetMakJ()->GetPmPtr()[9];

			a = std::atan2(s, c);

			/*edit CstMtxI*/
			_PrtCstMtxI[3][3] = -(GetMakI()->GetPrtPmPtr()[0 * 4 + 1]) * s + (GetMakI()->GetPrtPmPtr()[0 * 4 + 2]) * c;
			_PrtCstMtxI[4][3] = -(GetMakI()->GetPrtPmPtr()[1 * 4 + 1]) * s + (GetMakI()->GetPrtPmPtr()[1 * 4 + 2]) * c;
			_PrtCstMtxI[5][3] = -(GetMakI()->GetPrtPmPtr()[2 * 4 + 1]) * s + (GetMakI()->GetPrtPmPtr()[2 * 4 + 2]) * c;


			s_tf_n(GetDim(), -1, *_pm_M2N, *_PrtCstMtxI, 0, *_PrtCstMtxJ);
			/*update A_c*/
			std::fill_n(_a_c, UNIVERSAL_JOINT::GetCstDim(), 0);
			
			/*calculate a_dot*/
			v[0] = GetMakJ()->GetVelPtr()[3] - GetMakI()->GetVelPtr()[3];
			v[1] = GetMakJ()->GetVelPtr()[4] - GetMakI()->GetVelPtr()[4];
			v[2] = GetMakJ()->GetVelPtr()[5] - GetMakI()->GetVelPtr()[5];

			a_dot = GetMakI()->GetPmPtr()[0] * v[0] + GetMakI()->GetPmPtr()[4] * v[1] + GetMakI()->GetPmPtr()[8] * v[2];
			/*calculate part m*/
			v[0] = -c*a_dot;
			v[1] = -s*a_dot;

			s_inv_tv(GetMakI()->GetPrtPmPtr(), GetMakI()->GetFatherPrt()->GetPrtVelPtr(), _tem_v1);
			_a_c[3] -= v[0] * _tem_v1[4] + v[1] * _tem_v1[5];
			/*calculate part n*/
			s_inv_tv(*_pm_M2N, GetMakJ()->GetFatherPrt()->GetPrtVelPtr(), _tem_v1);
			s_cv(-1, GetMakI()->GetFatherPrt()->GetPrtVelPtr(), _tem_v1, 0, _tem_v2);
			s_dgemmTN(4, 1, 6, 1, *_PrtCstMtxI, GetDim(), _tem_v2, 1, 1, &_a_c[0], 1);
			s_inv_tv(GetMakI()->GetPrtPmPtr(), _tem_v1, _tem_v2);
			_a_c[3] += v[0] * _tem_v2[4] + v[1] * _tem_v2[5];
		};
		
		const char *const SPHERICAL_JOINT::type = "spherical";
		SPHERICAL_JOINT::SPHERICAL_JOINT(MODEL_BASE *pModel, const std::string &Name, int id, MARKER *pMakI, MARKER *pMakJ)
			: JOINT_BASE_DIM(pModel, Name, id, pMakI, pMakJ)
		{
		}
		SPHERICAL_JOINT::SPHERICAL_JOINT(MODEL_BASE *pModel, const std::string &Name, int id, const Aris::Core::ELEMENT *ele)
			: JOINT_BASE_DIM(pModel, Name, id, ele)
		{
		}
		void SPHERICAL_JOINT::Initiate()
		{
			double loc_cst[6][GetDim()];
			std::fill_n(static_cast<double*>(*loc_cst), 6 * GetDim(), 0);

			loc_cst[0][0] = 1;
			loc_cst[1][1] = 1;
			loc_cst[2][2] = 1;

			s_tf_n(GetDim(), GetMakI()->GetPrtPmPtr(), *loc_cst, GetPrtCstMtxIPtr());
		};

		const char *const LINEAR_MOTION::type = "linear";
		LINEAR_MOTION::LINEAR_MOTION(MODEL_BASE *pModel, const std::string &Name, int id, MARKER *pMakI, MARKER *pMakJ)
			: MOTION_BASE(pModel, Name, id, pMakI, pMakJ)
		{
		}
		LINEAR_MOTION::LINEAR_MOTION(MODEL_BASE *pModel, const std::string &Name, int id, const Aris::Core::ELEMENT *pEle)
			: MOTION_BASE(pModel, Name, id, pEle)
		{

		}
		void LINEAR_MOTION::Initiate()
		{
			double loc_cst[6]{0};

			std::fill_n(_PrtCstMtxI, 6, 0);
			std::fill_n(_PrtCstMtxJ, 6, 0);

			/* Get tm I2M */
			loc_cst[2] = 1;
			s_tf(_pMakI->GetPrtPmPtr(), loc_cst, _PrtCstMtxI);
		}
		void LINEAR_MOTION::Update()
		{
			double pm_M2N[4][4];
			double pm_I2J[4][4];
			
			/*update motPos motVel motAcc*/
			_pMakI->Update();
			_pMakJ->Update();

			s_inv_pm_dot_pm(_pMakJ->GetPmPtr(), _pMakI->GetPmPtr(), *pm_I2J);
			MotPos = pm_I2J[2][3];

			double velDiff[6],velDiff_in_J[6];
			std::copy_n(_pMakI->GetVelPtr(), 6, velDiff);
			s_daxpy(6, -1, _pMakJ->GetVelPtr(), 1, velDiff, 1);
			s_inv_tv(_pMakJ->GetPmPtr(), velDiff, velDiff_in_J);
			MotVel = velDiff_in_J[2];
			
			/*std::copy_n(_pMakI->GetAccPtr(), 6, velDiff);
			s_daxpy(6, -1, _pMakJ->GetAccPtr(), 1, velDiff, 1);
			s_inv_tv(_pMakJ->GetPmPtr(), velDiff, velDiff_in_J);
			MotAcc = velDiff_in_J[2];*/

			/*update cst fce*/
			double tem_v1[6], tem_v2[6];

			std::fill_n(_PrtCstMtxJ, 6, 0);
			memset(_a_c, 0, sizeof(double) * 6);

			/* Get tmf M2N */
			s_pm_dot_pm(_pMakJ->GetFatherPrt()->GetPrtPmPtr(), _pMakI->GetFatherPrt()->GetPmPtr(), *pm_M2N);
			s_tf(-1, *pm_M2N, _PrtCstMtxI, 0, _PrtCstMtxJ);

			s_inv_tv(-1, *pm_M2N, _pMakJ->GetFatherPrt()->GetPrtVelPtr(), 0, tem_v1);
			s_cv(_pMakI->GetFatherPrt()->GetPrtVelPtr(), tem_v1, tem_v2);
			s_dgemmTN(1, 1, 6, 1, _PrtCstMtxI, 1, tem_v2, 1, 0, &_a_c[0], 1);

			_a_c[0] += MotAcc;

			/*update motPos motVel motAcc*/
		}

		const char *const SINGLE_COMPONENT_FORCE::type = "single_component_force";
		void SINGLE_COMPONENT_FORCE::Update()
		{
			s_tf(this->pMakI->GetPrtPmPtr(), fceI, _PrtFceI);
			double pm[16];
			s_inv_pm_dot_pm(_pPrtN->GetPmPtr(), pMakI->GetPmPtr(), pm);
			s_tf(-1, pm, fceI, 0, _PrtFceJ);
		}
		SINGLE_COMPONENT_FORCE::SINGLE_COMPONENT_FORCE(MODEL_BASE *pModel, const std::string &name, int id, MARKER* makI, MARKER* makJ, int componentID)
			: FORCE_BASE(pModel, name, id, makI->GetFatherPrt(), makJ->GetFatherPrt())
			, pMakI(makI)
			, componentID(componentID)
		{

		}
		SINGLE_COMPONENT_FORCE::SINGLE_COMPONENT_FORCE(MODEL_BASE *pModel, const std::string &name, int id, const Aris::Core::ELEMENT *xmlEle)
			: FORCE_BASE(pModel, name, id, pModel->GetPart(xmlEle->Attribute("PrtM")), pModel->GetPart(xmlEle->Attribute("PrtN")))
			, pMakI(pModel->GetPart(xmlEle->Attribute("PrtM"))->GetMarker(xmlEle->Attribute("MakI")))
			, pMakJ(pModel->GetPart(xmlEle->Attribute("PrtN"))->GetMarker(xmlEle->Attribute("MakJ")))
			, componentID(std::stoi(xmlEle->Attribute("Component")))
		{
		}
		void SINGLE_COMPONENT_FORCE::ToAdamsCmd(std::ofstream &file) const
		{
			if (fceCurve == nullptr)
			{
				std::string type = "translational";

				file << "force create direct single_component_force  &\r\n"
					<< "    single_component_force_name = ." << Model()->Name() << "." << Name() << "  &\r\n"
					<< "    adams_id = " << GetID() + 1 << "  &\r\n"
					<< "    type_of_freedom = " << type << "  &\r\n"
					<< "    i_marker_name = ." << Model()->Name() << "." << pMakI->GetFatherPrt()->Name() << "." << pMakI->Name() << "  &\r\n"
					<< "    j_marker_name = ." << Model()->Name() << "." << pMakJ->GetFatherPrt()->Name() << "." << pMakJ->Name() << "  &\r\n"
					<< "    action_only = off  &\r\n"
					<< "    function = \"" << GetFce() << "\"  \r\n"
					<< "!\r\n";
			}
			else
			{
				std::string type = "translational";

				file << "data_element create spline &\r\n"
					<< "    spline_name = ." << Model()->Name() << "." << Name() << "_fce_spl  &\r\n"
					<< "    adams_id = " << GetID() * 2 + 1 << "  &\r\n"
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
					<< "    single_component_force_name = ." << Model()->Name() << "." << Name() << "  &\r\n"
					<< "    adams_id = " << GetID() + 1 << "  &\r\n"
					<< "    type_of_freedom = " << type << "  &\r\n"
					<< "    i_marker_name = ." << Model()->Name() << "." << pMakI->GetFatherPrt()->Name() << "." << pMakI->Name() << "  &\r\n"
					<< "    j_marker_name = ." << Model()->Name() << "." << pMakJ->GetFatherPrt()->Name() << "." << pMakJ->Name() << "  &\r\n"
					<< "    action_only = off  &\r\n"
					<< "    function = \"AKISPL(time,0," << Name() << "_fce_spl)\"  \r\n"
					<< "!\r\n";
			}



		}
	
		void MODEL::FromXmlElement(const Aris::Core::ELEMENT *pModel)
		{
			MODEL_BASE::FromXmlElement(pModel);
			
			const Aris::Core::ELEMENT *pJnt = pModel->FirstChildElement("Joint");
			if (pJnt == nullptr)throw(std::logic_error("Model must have joint element"));
			const Aris::Core::ELEMENT *pMot = pModel->FirstChildElement("Motion");
			if (pMot == nullptr)throw(std::logic_error("Model must have motion element"));
			const Aris::Core::ELEMENT *pFce = pModel->FirstChildElement("Force");
			if (pFce == nullptr)throw(std::logic_error("Model must have force element"));
			
			for (auto ele = pJnt->FirstChildElement(); ele != nullptr; ele = ele->NextSiblingElement())
			{
				if (strcmp(TRANSLATIONAL_JOINT::type, ele->Attribute("Type")) == 0)
				{
					AddJoint<TRANSLATIONAL_JOINT>(ele->Name(), ele);
				}
				if (strcmp(SPHERICAL_JOINT::type, ele->Attribute("Type")) == 0)
				{
					AddJoint<SPHERICAL_JOINT>(ele->Name(), ele);
				}
				if (strcmp(UNIVERSAL_JOINT::type, ele->Attribute("Type")) == 0)
				{
					AddJoint<UNIVERSAL_JOINT>(ele->Name(), ele);
				}

				InitiateElement(GetJoint(ele->Name()));
			}

			for (auto ele = pMot->FirstChildElement(); ele != nullptr; ele = ele->NextSiblingElement())
			{
				if (strcmp(LINEAR_MOTION::type, ele->Attribute("Type")) == 0)
				{
					AddMotion<LINEAR_MOTION>(ele->Name(), ele);
				}
				InitiateElement(GetMotion(ele->Name()));
			}

			for (auto ele = pFce->FirstChildElement(); ele != nullptr; ele = ele->NextSiblingElement())
			{
				if (strcmp(SINGLE_COMPONENT_FORCE::type, ele->Attribute("Type")) == 0)
				{
					AddForce<SINGLE_COMPONENT_FORCE>(ele->Name(), ele);
				}
			}
		}
	}
}
