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


#include "Aris_DynKer.h"
#include "Aris_DynModelBase.h"
#include "Aris_DynModel.h"

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
		PART::PART(MODEL_BASE *pModel, const string &Name, int id, const double *Im, const double *pm, const double *Vel, const double *Acc)
			:ELEMENT(pModel, Name, id)
		{
			if (Im == nullptr)
			{
				std::fill_n(static_cast<double *>(*_PrtIm), 36, 0);
				_PrtIm[0][0] = 1;
				_PrtIm[1][1] = 1;
				_PrtIm[2][2] = 1;
				_PrtIm[3][3] = 1;
				_PrtIm[4][4] = 1;
				_PrtIm[5][5] = 1;
			}
			else
			{
				std::copy_n(Im, 36, *_PrtIm);
			}

			if (pm == nullptr)
			{
				std::fill_n(static_cast<double *>(*_Pm), 16, 0);
				_Pm[0][0] = 1;
				_Pm[1][1] = 1;
				_Pm[2][2] = 1;
				_Pm[3][3] = 1;
			}
			else
			{
				SetPm(pm);
			}

			if (Vel == nullptr)
			{
				std::fill_n(_Vel, 6, 0);
			}
			else
			{
				SetVel(Vel);
			}

			if (Acc == nullptr)
			{
				std::fill_n(_Acc, 6, 0);
			}
			else
			{
				SetAcc(Acc);
			}
		}
		PART::PART(MODEL_BASE *pModel, const std::string &Name, int id, const Aris::Core::ELEMENT *ele)
			:ELEMENT(pModel, Name, id)
		{
			if (ele->Attribute("Active", "true"))
			{
				this->Activate();
			}
			else if (ele->Attribute("Active", "false"))
			{
				this->Deactivate();
			}
			else
			{
				throw std::logic_error("failed load xml file in part");
			}

			MATRIX m;

			m = Model()->calculator.CalculateExpression(ele->Attribute("Inertia"));
			s_gamma2im(m.Data(), *_PrtIm);

			m = Model()->calculator.CalculateExpression(ele->Attribute("Pos"));
			s_pe2pm(m.Data(), *_Pm);

			m = Model()->calculator.CalculateExpression(ele->Attribute("Vel"));
			memcpy(_Vel, m.Data(), sizeof(_Vel));

			m = Model()->calculator.CalculateExpression(ele->Attribute("Acc"));
			memcpy(_Acc, m.Data(), sizeof(_Acc));

			_markerNames.clear();

			for (auto makEle = ele->FirstChildElement("ChildMarker")->FirstChildElement(); makEle != nullptr; makEle = makEle->NextSiblingElement())
			{
				AddMarker(makEle->Name(), makEle);
			}

			if (ele->Attribute("Graphic_File_Path") && (!ele->Attribute("Graphic_File_Path", "")))
				graphicFilePath = ele->Attribute("Graphic_File_Path");
		}
		void PART::Update()
		{
			double tem[6];
		
			s_inv_pm(*_Pm, *_PrtPm);
			s_tv(*_PrtPm, _Vel, _PrtVel);
			s_tv(*_PrtPm, _Acc, _PrtAcc);
			s_tv(*_PrtPm, Model()->_Environment.Gravity, _PrtGravity);
			s_m6_dot_v6(*_PrtIm, _PrtGravity, _PrtFg);
			s_m6_dot_v6(*_PrtIm, _PrtVel, tem);
			s_cf(_PrtVel, tem, _PrtFv);
		}
		MARKER* PART::GetMarker(const std::string &Name)
		{
			auto pMak = _markerNames.find(Name);
			if (pMak != _markerNames.end())
			{
				return Model()->_markers.at(pMak->second).get();
			}
			else
			{
				return nullptr;
			}
		}
		const MARKER* PART::GetMarker(const std::string &Name)const
		{
			auto pMak = _markerNames.find(Name);
			if (pMak != _markerNames.end())
			{
				return Model()->_markers.at(pMak->second).get();
			}
			else
			{
				return nullptr;
			}
		}
		void PART::ToXmlElement(Aris::Core::ELEMENT *pEle) const
		{
			double value[10];
			
			pEle->DeleteChildren();
			pEle->SetName(this->Name().data());

			Aris::Core::ELEMENT *pActive = pEle->GetDocument()->NewElement("Active");
			if (this->Active())
				pActive->SetText("True");
			else
				pActive->SetText("False");
			pEle->InsertEndChild(pActive);
			
			Aris::Core::ELEMENT *pInertia = pEle->GetDocument()->NewElement("Inertia");
			s_im2gamma(this->GetPrtImPtr(),value);
			pInertia->SetText(MATRIX(1,10,value).ToString().c_str());
			pEle->InsertEndChild(pInertia);

			Aris::Core::ELEMENT *pPE = pEle->GetDocument()->NewElement("Pos");
			s_pm2pe(*_Pm, value);
			pPE->SetText(MATRIX(1, 6, value).ToString().c_str());
			pEle->InsertEndChild(pPE);

			Aris::Core::ELEMENT *pVel = pEle->GetDocument()->NewElement("Vel");
			pVel->SetText(MATRIX(1, 6, _Vel).ToString().c_str());
			pEle->InsertEndChild(pVel);

			Aris::Core::ELEMENT *pAcc = pEle->GetDocument()->NewElement("Acc");
			pAcc->SetText(MATRIX(1, 6, _Acc).ToString().c_str());
			pEle->InsertEndChild(pAcc);

			Aris::Core::ELEMENT *pChildMak = pEle->GetDocument()->NewElement("ChildMarker");
			pEle->InsertEndChild(pChildMak);

			for (auto &m:_markerNames)
			{
				Aris::Core::ELEMENT *ele = pEle->GetDocument()->NewElement("");

				Model()->_markers.at(m.second)->ToXmlElement(ele);
				pChildMak->InsertEndChild(ele);
			}

			Aris::Core::ELEMENT *pGraphicFilePath = pEle->GetDocument()->NewElement("Graphic_File_Path");
			pGraphicFilePath->SetText(this->graphicFilePath.c_str());
			pEle->InsertEndChild(pGraphicFilePath);
		}
		void PART::ToAdamsCmd(std::ofstream &file) const
		{
			if (this == Model()->pGround)
			{
				file << "!----------------------------------- ground -----------------------------------!\r\n"
					<< "!\r\n"
					<< "!\r\n"
					<< "! ****** Ground Part ******\r\n"
					<< "!\r\n"
					<< "defaults model  &\r\n"
					<< "    part_name = ground\r\n"
					<< "!\r\n"
					<< "defaults coordinate_system  &\r\n"
					<< "    default_coordinate_system = ." << Model()->Name() << ".ground\r\n"
					<< "!\r\n"
					<< "! ****** Markers for current part ******\r\n"
					<< "!\r\n";
			}
			else
			{
				double pe[6];
				s_pm2pe(this->GetPmPtr(), pe, "313");
				MATRIX ori(1, 3, &pe[3]), loc(1, 3, &pe[0]);

				file << "!----------------------------------- " << this->Name() << " -----------------------------------!\r\n"
					<< "!\r\n"
					<< "!\r\n"
					<< "defaults coordinate_system  &\r\n"
					<< "    default_coordinate_system = ." << Model()->Name() << ".ground\r\n"
					<< "!\r\n"
					<< "part create rigid_body name_and_position  &\r\n"
					<< "    part_name = ." << Model()->Name() << "." << this->Name() << "  &\r\n"
					<< "    adams_id = " << this->GetID() + 1 << "  &\r\n"
					<< "    location = (" << loc.ToString() << ")  &\r\n"
					<< "    orientation = (" << ori.ToString() << ")\r\n"
					<< "!\r\n"
					<< "defaults coordinate_system  &\r\n"
					<< "    default_coordinate_system = ." << Model()->Name() << "." << this->Name() << " \r\n"
					<< "!\r\n";

				
				double mass = this->GetPrtImPtr()[0] == 0 ? 1 : GetPrtImPtr()[0];
				std::fill_n(pe, 6, 0);
				pe[0] = this->GetPrtImPtr()[11] / mass;
				pe[1] = -this->GetPrtImPtr()[5] / mass;
				pe[2] = this->GetPrtImPtr()[4] / mass;				

				file << "! ****** cm and mass for current part ******\r\n"
					<< "marker create  &\r\n"
					<< "    marker_name = ." << Model()->Name() << "." << this->Name() << ".cm  &\r\n"
					<< "    adams_id = " << this->GetID() + Model()->GetMarkerNum() << "  &\r\n"
					<< "    location = ({" << pe[0] << "," << pe[1] << "," << pe[2] << "})  &\r\n"
					<< "    orientation = (" << "{0,0,0}" << ")\r\n"
					<< "!\r\n";

				double pm[16];
				double im[6][6];

				pe[0] = -pe[0];
				pe[1] = -pe[1];
				pe[2] = -pe[2];

				s_pe2pm(pe, pm);
				s_i2i(pm, this->GetPrtImPtr(), *im);

				/*！注意！*/
				//Adams里对惯量矩阵的定义貌似和我自己的定义在Ixy，Ixz，Iyz上互为相反数。别问我为什么，我也不知道。
				file << "part create rigid_body mass_properties  &\r\n"
					<< "    part_name = ." << Model()->Name() << "." << this->Name() << "  &\r\n"
					<< "    mass = " << this->GetPrtImPtr()[0] << "  &\r\n"
					<< "    center_of_mass_marker = ." << Model()->Name() << "." << this->Name() << ".cm  &\r\n"
					<< "    inertia_marker = ." << Model()->Name() << "." << this->Name() << ".cm  &\r\n"
					<< "    ixx = " << im[3][3] << "  &\r\n"
					<< "    iyy = " << im[4][4] << "  &\r\n"
					<< "    izz = " << im[5][5] << "  &\r\n"
					<< "    ixy = " << -im[4][3] << "  &\r\n"
					<< "    izx = " << -im[5][3] << "  &\r\n"
					<< "    iyz = " << -im[5][4] << "\r\n"
					<< "!\r\n";

				
			}

			//导入marker
			for (auto &mak : this->_markerNames)
			{
				double pe[6];

				s_pm2pe(Model()->GetMarker(mak.second)->GetPrtPmPtr(), pe, "313");
				MATRIX ori(1, 3, &pe[3]), loc(1, 3, &pe[0]);

				file << "marker create  &\r\n"
					<< "marker_name = ." << Model()->Name() << "." << this->Name() << "." << mak.first << "  &\r\n"
					<< "adams_id = " << mak.second + 1 << "  &\r\n"
					<< "location = (" << loc.ToString() << ")  &\r\n"
					<< "orientation = (" << ori.ToString() << ")\r\n"
					<< "!\r\n";
			}
			//导入parasolid
			std::stringstream stream(this->graphicFilePath);
			string path;
			while (stream >> path)
			{
				file << "file parasolid read &\r\n"
					<< "file_name = \"" << path << "\" &\r\n"
					<< "type = ASCII" << " &\r\n"
					<< "part_name = " << this->Name() << " \r\n"
					<< "\r\n";
			}
		}

		MARKER::MARKER(PART *pPrt, const string &Name, int id, const double *pLocPm, MARKER *pRelativeTo)
			: ELEMENT(pPrt->Model(), Name, id)
			, _pPrt(pPrt)
		{
			if (pRelativeTo == nullptr)
			{
				if (pLocPm == nullptr)
				{
					memset(_PrtPm, 0, sizeof(_PrtPm));
					_PrtPm[0][0] = 1;
					_PrtPm[1][1] = 1;
					_PrtPm[2][2] = 1;
					_PrtPm[3][3] = 1;
				}
				else
				{
					memcpy(_PrtPm, pLocPm, sizeof(_PrtPm));
				}
			}
			else
			{
				if (pLocPm == nullptr)
				{
					memcpy(_PrtPm, pRelativeTo->GetPrtPmPtr(), sizeof(_PrtPm));
				}
				else
				{
					s_pm_dot_pm(pRelativeTo->GetPrtPmPtr(), pLocPm, *_PrtPm);
				}
			}
			
			
		}
		MARKER::MARKER(PART *pPrt, const std::string &Name, int id, const Aris::Core::ELEMENT *ele)
			: ELEMENT(pPrt->Model(), Name, id)
			, _pPrt(pPrt)
		{
			double pm[16];
			MATRIX m = Model()->calculator.CalculateExpression(ele->Attribute("Pos"));
			s_pe2pm(m.Data(), pm);

			if (ele->Attribute("RelativeTo")&&(!ele->Attribute("RelativeTo","")))
			{
				MARKER *pRelativeMak = _pPrt->GetMarker(ele->Attribute("RelativeTo"));
				s_pm_dot_pm(pRelativeMak->GetPrtPmPtr(), pm, *_PrtPm);
			}
			else
			{
				std::copy_n(pm, 16, static_cast<double*>(*_PrtPm));
			}
		}
		void MARKER::Update()
		{
			s_pm_dot_pm(_pPrt->GetPmPtr(), *_PrtPm, *_Pm);
		}
		void MARKER::ToXmlElement(Aris::Core::ELEMENT *pEle) const
		{
			double value[10];

			pEle->DeleteChildren();
			pEle->SetName(this->Name().data());

			Aris::Core::ELEMENT *pPE = pEle->GetDocument()->NewElement("Pos");
			s_pm2pe(*_PrtPm, value);
			pPE->SetText(MATRIX(1,6,value).ToString().c_str());
			pEle->InsertEndChild(pPE);

			Aris::Core::ELEMENT *pRelativeMakEle = pEle->GetDocument()->NewElement("RelativeTo");
			pRelativeMakEle->SetText("");
			pEle->InsertEndChild(pRelativeMakEle);
		}

		JOINT_BASE::JOINT_BASE(MODEL_BASE *pModel, const std::string &Name, int id, MARKER *pMakI, MARKER *pMakJ)
			: ELEMENT(pModel, Name, id)
			, _pMakI(pMakI)
			, _pMakJ(pMakJ)
		{

		}
		JOINT_BASE::JOINT_BASE(MODEL_BASE *pModel, const std::string &Name, int id, const Aris::Core::ELEMENT *ele)
			: ELEMENT(pModel, Name, id)
		{
			if (strcmp("true", ele->Attribute("Active")) == 0)
			{
				this->Activate();
			}
			else if (strcmp("false", ele->Attribute("Active")) == 0)
			{
				this->Deactivate();
			}
			else
			{
				throw std::logic_error("failed load xml file in joint");
			}

			_pMakI = Model()->GetPart(ele->Attribute("PrtM"))->GetMarker(ele->Attribute("MakI"));
			_pMakJ = Model()->GetPart(ele->Attribute("PrtN"))->GetMarker(ele->Attribute("MakJ"));

		}
		void JOINT_BASE::Update()
		{
			double _pm_M2N[4][4];
			double _tem_v1[6], _tem_v2[6];

			/* Get pm M2N */
			s_pm_dot_pm(GetMakJ()->GetFatherPrt()->GetPrtPmPtr(), GetMakI()->GetFatherPrt()->GetPmPtr(), *_pm_M2N);

			/*update PrtCstMtx*/
			s_tf_n(GetCstDim(), -1, *_pm_M2N, this->GetPrtCstMtxIPtr(), 0, this->GetPrtCstMtxJPtr());

			/*update A_c*/
			s_inv_tv(-1, *_pm_M2N, GetMakJ()->GetFatherPrt()->GetPrtVelPtr(), 0, _tem_v1);
			s_cv(GetMakI()->GetFatherPrt()->GetPrtVelPtr(), _tem_v1, _tem_v2);
			s_dgemmTN(GetCstDim(), 1, 6, 1, GetPrtCstMtxIPtr(), GetCstDim(), _tem_v2, 1, 0, GetPrtA_cPtr(), 1);
		};
		void JOINT_BASE::ToXmlElement(Aris::Core::ELEMENT *pEle) const
		{
			pEle->DeleteChildren();
			pEle->SetName(this->Name().data());

			Aris::Core::ELEMENT *pActive = pEle->GetDocument()->NewElement("Active");
			if (this->Active())
				pActive->SetText("True");
			else
				pActive->SetText("False");
			pEle->InsertEndChild(pActive);

			Aris::Core::ELEMENT *pType = pEle->GetDocument()->NewElement("Type");
			pType->SetText(this->GetType());
			pEle->InsertEndChild(pType);

			Aris::Core::ELEMENT *pPrtI = pEle->GetDocument()->NewElement("iPart");
			pPrtI->SetText(_pMakI->GetFatherPrt()->Name().data());
			pEle->InsertEndChild(pPrtI);

			Aris::Core::ELEMENT *pPrtJ = pEle->GetDocument()->NewElement("jPart");
			pPrtJ->SetText(_pMakJ->GetFatherPrt()->Name().data());
			pEle->InsertEndChild(pPrtJ);

			Aris::Core::ELEMENT *pMakI = pEle->GetDocument()->NewElement("iMarker");
			pMakI->SetText(_pMakI->Name().data());
			pEle->InsertEndChild(pMakI);

			Aris::Core::ELEMENT *pMakJ = pEle->GetDocument()->NewElement("jMarker");
			pMakJ->SetText(_pMakJ->Name().data());
			pEle->InsertEndChild(pMakJ);
		}
		void JOINT_BASE::ToAdamsCmd(std::ofstream &file) const
		{
			file << "constraint create joint " << this->GetType() << "  &\r\n"
				<< "    joint_name = ." <<	Model()->Name() << "." << this->Name() << "  &\r\n"
				<< "    adams_id = " << this->GetID() + 1 << "  &\r\n"
				<< "    i_marker_name = ." << Model()->Name() << "." << this->_pMakI->GetFatherPrt()->Name() << "." << this->_pMakI->Name() << "  &\r\n"
				<< "    j_marker_name = ." << Model()->Name() << "." << this->_pMakJ->GetFatherPrt()->Name() << "." << this->_pMakJ->Name() << "  \r\n"
				<< "!\r\n";
		}

		MOTION_BASE::MOTION_BASE(MODEL_BASE *pModel, const std::string &Name, int id, MARKER *pMakI, MARKER *pMakJ)
			: ELEMENT(pModel, Name, id)
			, _pMakI(pMakI)
			, _pMakJ(pMakJ)
		{
			memset(_frc_coe, 0, sizeof(double) * 3);
		}
		MOTION_BASE::MOTION_BASE(MODEL_BASE *pModel, const std::string &Name, int id, const Aris::Core::ELEMENT *ele)
			: ELEMENT(pModel, Name, id)
		{
			if (strcmp("true", ele->Attribute("Active")) == 0)
			{
				this->Activate();
			}
			else if (strcmp("false", ele->Attribute("Active")) == 0)
			{
				this->Deactivate();
			}
			else
			{
				throw std::logic_error("failed load xml file in motion");
			}

			MATRIX m = Model()->calculator.CalculateExpression(ele->Attribute("FrcCoe"));
			std::copy_n(m.Data(), 3, _frc_coe);

			_pMakI = Model()->GetPart(ele->Attribute("PrtM"))->GetMarker(ele->Attribute("MakI"));
			_pMakJ = Model()->GetPart(ele->Attribute("PrtN"))->GetMarker(ele->Attribute("MakJ"));

		}
		void MOTION_BASE::ToXmlElement(Aris::Core::ELEMENT *pEle) const
		{
			pEle->DeleteChildren();
			pEle->SetName(this->Name().data());

			Aris::Core::ELEMENT *pActive = pEle->GetDocument()->NewElement("Active");
			if (this->Active())
				pActive->SetText("True");
			else
				pActive->SetText("False");
			pEle->InsertEndChild(pActive);

			Aris::Core::ELEMENT *pType = pEle->GetDocument()->NewElement("Type");
			pType->SetText(this->GetType());
			pEle->InsertEndChild(pType);

			Aris::Core::ELEMENT *pPrtI = pEle->GetDocument()->NewElement("iPart");
			pPrtI->SetText(_pMakI->GetFatherPrt()->Name().data());
			pEle->InsertEndChild(pPrtI);

			Aris::Core::ELEMENT *pPrtJ = pEle->GetDocument()->NewElement("jPart");
			pPrtJ->SetText(_pMakJ->GetFatherPrt()->Name().data());
			pEle->InsertEndChild(pPrtJ);

			Aris::Core::ELEMENT *pMakI = pEle->GetDocument()->NewElement("iMarker");
			pMakI->SetText(_pMakI->Name().data());
			pEle->InsertEndChild(pMakI);

			Aris::Core::ELEMENT *pMakJ = pEle->GetDocument()->NewElement("jMarker");
			pMakJ->SetText(_pMakJ->Name().data());
			pEle->InsertEndChild(pMakJ);

			Aris::Core::ELEMENT *pFrictionCoefficients = pEle->GetDocument()->NewElement("Friction_Coefficients");

			pFrictionCoefficients->SetText(MATRIX(1, 3, _frc_coe).ToString().c_str());
			pEle->InsertEndChild(pFrictionCoefficients);
		}
		void MOTION_BASE::ToAdamsCmd(std::ofstream &file) const
		{
			std::string s;
			s = "z";

			if (this->posCurve == nullptr)
			{
				file << "constraint create motion_generator &\r\n"
					<< "    motion_name = ." << Model()->Name() << "." << this->Name() << "  &\r\n"
					<< "    adams_id = " << this->GetID() + 1 << "  &\r\n"
					<< "    i_marker_name = ." << Model()->Name() << "." << this->_pMakI->GetFatherPrt()->Name() << "." << this->_pMakI->Name() << "  &\r\n"
					<< "    j_marker_name = ." << Model()->Name() << "." << this->_pMakJ->GetFatherPrt()->Name() << "." << this->_pMakJ->Name() << "  &\r\n"
					<< "    axis = " << s << "  &\r\n"
					<< "    function = \"" << this->GetMotPos() << "\"  \r\n"
					<< "!\r\n";
			}
			else
			{
				file << "data_element create spline &\r\n"
					<< "    spline_name = ." << Model()->Name() << "." << this->Name() << "_pos_spl  &\r\n"
					<< "    adams_id = " << this->GetID() * 2 << "  &\r\n"
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
					<< "    motion_name = ." << Model()->Name() << "." << this->Name() << "  &\r\n"
					<< "    adams_id = " << this->GetID() + 1 << "  &\r\n"
					<< "    i_marker_name = ." << Model()->Name() << "." << this->_pMakI->GetFatherPrt()->Name() << "." << this->_pMakI->Name() << "  &\r\n"
					<< "    j_marker_name = ." << Model()->Name() << "." << this->_pMakJ->GetFatherPrt()->Name() << "." << this->_pMakJ->Name() << "  &\r\n"
					<< "    axis = " << s << "  &\r\n"
					<< "    function = \"AKISPL(time,0," << this->Name() << "_pos_spl)\"  \r\n"
					<< "!\r\n";
			}
		}

		FORCE_BASE::FORCE_BASE(MODEL_BASE *pModel, const std::string &Name, int id, MARKER *pMakI, MARKER *pMakJ)
			: ELEMENT(pModel, Name, id)
			, _pMakI(pMakI)
			, _pMakJ(pMakJ)
		{
		}
		FORCE_BASE::FORCE_BASE(MODEL_BASE *pModel, const std::string &Name, int id, const Aris::Core::ELEMENT *xmlEle)
			: ELEMENT(pModel, Name, id)
			, _pMakI(pModel->GetPart(xmlEle->Attribute("PrtM"))->GetMarker(xmlEle->Attribute("MakI")))
			, _pMakJ(pModel->GetPart(xmlEle->Attribute("PrtN"))->GetMarker(xmlEle->Attribute("MakJ")))
		{

		}

		ENVIRONMENT::ENVIRONMENT(MODEL_BASE *pModel)
			:OBJECT(pModel,"Environment")
		{
			double data[] = { 0, -9.8, 0, 0, 0, 0 };
			memcpy(Gravity, data, sizeof(Gravity));
		}
		ENVIRONMENT::~ENVIRONMENT()
		{
		}
		void ENVIRONMENT::ToXmlElement(Aris::Core::ELEMENT *pEle) const
		{
			pEle->DeleteChildren();
			pEle->SetName("Enviroment");

			Aris::Core::ELEMENT *pGravity = pEle->GetDocument()->NewElement("Gravity");
			pGravity->SetText(MATRIX(1, 6, Gravity).ToString().c_str());
			pEle->InsertEndChild(pGravity);
		}
		void ENVIRONMENT::FromXmlElement(const Aris::Core::ELEMENT *pEle)
		{
			MATRIX m = Model()->calculator.CalculateExpression(pEle->FirstChildElement("Gravity")->GetText());
			memcpy(Gravity, m.Data(), sizeof(Gravity));
		}
		void ENVIRONMENT::ToAdamsCmd(std::ofstream &file) const
		{
			file << "!-------------------------- Default Units for Model ---------------------------!\r\n"
				<< "!\r\n"
				<< "!\r\n"
				<< "defaults units  &\r\n"
				<< "    length = meter  &\r\n"
				<< "    angle = rad  &\r\n"
				<< "    force = newton  &\r\n"
				<< "    mass = kg  &\r\n"
				<< "    time = sec\r\n"
				<< "!\n"
				<< "defaults units  &\r\n"
				<< "    coordinate_system_type = cartesian  &\r\n"
				<< "    orientation_type = body313\r\n"
				<< "!\r\n"
				<< "!------------------------ Default Attributes for Model ------------------------!\r\n"
				<< "!\r\n"
				<< "!\r\n"
				<< "defaults attributes  &\r\n"
				<< "    inheritance = bottom_up  &\r\n"
				<< "    icon_visibility = off  &\r\n"
				<< "    grid_visibility = off  &\r\n"
				<< "    size_of_icons = 5.0E-002  &\r\n"
				<< "    spacing_for_grid = 1.0\r\n"
				<< "!\r\n"
				<< "!------------------------------ Adams/View Model ------------------------------!\r\n"
				<< "!\r\n"
				<< "!\r\n"
				<< "model create  &\r\n"
				<< "   model_name = " << this->Model()->Name() << "\r\n"
				<< "!\r\n"
				<< "view erase\r\n"
				<< "!\r\n"
				<< "!---------------------------------- Accgrav -----------------------------------!\r\n"
				<< "!\r\n"
				<< "!\r\n"
				<< "force create body gravitational  &\r\n"
				<< "    gravity_field_name = gravity  &\r\n"
				<< "    x_component_gravity = " << this->Gravity[0] << "  &\r\n"
				<< "    y_component_gravity = " << this->Gravity[1] << "  &\r\n"
				<< "    z_component_gravity = " << this->Gravity[2] << "\r\n"
				<< "!\r\n";
		};


		MODEL_BASE::MODEL_BASE(const std::string & Name)
			: OBJECT(this , Name)
			, _Environment(this)
			, pGround(nullptr)
		{
			AddPart("Ground");
			pGround = GetPart("Ground");
		}
		MODEL_BASE::~MODEL_BASE()
		{
		}

		template<class T>
		typename T::value_type::element_type * GetContent(const T &container, const string &Name)
		{
			auto p = std::find_if(container.begin(), container.end(), [Name](typename T::const_reference p)
			{
				return (p->Name() == Name);
			});

			if (p == container.end())
			{
				return nullptr;
			}
			else
			{
				return p->get();
			}
		}

		const PART *MODEL_BASE::GetPart(int id) const
		{
			return _parts.at(id).get();
		}
		const JOINT_BASE *MODEL_BASE::GetJoint(int id)const
		{
			return _joints.at(id).get();
		}
		const MOTION_BASE *MODEL_BASE::GetMotion(int id)const
		{
			return _motions.at(id).get();
		}
		const FORCE_BASE *MODEL_BASE::GetForce(int id)const
		{
			return _forces.at(id).get();
		}
		const MARKER *MODEL_BASE::GetMarker(int id)const
		{
			return _markers.at(id).get();
		}
		PART *MODEL_BASE::GetPart(int id)
		{
			return _parts.at(id).get();
		}
		JOINT_BASE *MODEL_BASE::GetJoint(int id)
		{
			return _joints.at(id).get();
		}
		MOTION_BASE *MODEL_BASE::GetMotion(int id)
		{
			return _motions.at(id).get();
		}
		FORCE_BASE *MODEL_BASE::GetForce(int id)
		{
			return _forces.at(id).get();
		}
		MARKER *MODEL_BASE::GetMarker(int id)
		{
			return _markers.at(id).get();
		}
		const PART *MODEL_BASE::GetPart(const std::string &Name)const
		{
			return GetContent<decltype(_parts)>(_parts, Name);
		}
		const JOINT_BASE *MODEL_BASE::GetJoint(const std::string &Name)const
		{
			return GetContent<decltype(_joints)>(_joints, Name);
		}
		const MOTION_BASE *MODEL_BASE::GetMotion(const std::string &Name)const
		{
			return GetContent<decltype(_motions)>(_motions, Name);
		}
		const FORCE_BASE *MODEL_BASE::GetForce(const std::string &Name)const
		{
			return GetContent<decltype(_forces)>(_forces, Name);
		}
		PART *MODEL_BASE::GetPart(const std::string &Name)
		{
			return GetContent<decltype(_parts)>(_parts, Name);
		}
		JOINT_BASE *MODEL_BASE::GetJoint(const std::string &Name)
		{
			return GetContent<decltype(_joints)>(_joints, Name);
		}
		MOTION_BASE *MODEL_BASE::GetMotion(const std::string &Name)
		{
			return GetContent<decltype(_motions)>(_motions, Name);
		}
		FORCE_BASE *MODEL_BASE::GetForce(const std::string &Name)
		{
			return GetContent<decltype(_forces)>(_forces, Name);
		}

		void MODEL_BASE::DynPre(int &I_dim, int &C_dim)
		{
			int pid = 0;//part id
			int cid = 6;//Constraint id

			for (auto &part:_parts)
			{
				if (part->Active())
				{
					part->_RowId = pid;
					pid += 6;
				}
				else
				{
					part->_RowId = 0;
				}
			}
			for (auto &joint:_joints)
			{
				if (joint->Active())
				{
					joint->Initiate();
					joint->_ColId = cid;
					cid += joint->GetCstDim();
				}
				else
				{
					joint->_ColId = 0;
				}
			}
			for (auto &motion:_motions)
			{
				if (motion->Active())
				{
					motion->_ColId = cid;
					//cid += motion->GetCstDim();
					cid++;
					motion->Initiate();
				}
				else
				{
					motion->_ColId = 0;
					motion->Initiate();
				}
			}

			I_dim = pid;
			C_dim = cid;
			this->I_dim = I_dim;
			this->C_dim = C_dim;
		}
		void MODEL_BASE::DynMtx(double *C, double*a_c, double *I_mat, double*f, double *D, double *b)
		{
			std::fill_n(C, I_dim*C_dim, 0);
			std::fill_n(a_c, C_dim, 0);
			std::fill_n(I_mat, I_dim*I_dim, 0);
			std::fill_n(f, I_dim, 0);
			std::fill_n(D, (I_dim + C_dim)*(I_dim + C_dim), 0);
			std::fill_n(b, (I_dim + C_dim), 0);
			
			for (int i = 0; i < 6; ++i)
			{
				I_mat[I_dim*(pGround->_RowId + i) + pGround->_RowId + i] = 1;
				C[C_dim*(pGround->_RowId + i) + i] = 1;
			}

			/*Update all elements*/
			for (auto &prt : _parts)
			{
				if (prt->Active())
				{
					prt->Update();
					s_block_cpy(6, 6, *(prt->_PrtIm), 0, 0, 6, I_mat, prt->_RowId, prt->_RowId, I_dim);

					s_daxpy(6, -1, prt->_PrtFg, 1, &f[prt->_RowId], 1);
					s_daxpy(6, 1, prt->_PrtFv, 1, &f[prt->_RowId], 1);
				}
			}
			for (auto &jnt : _joints)
			{
				if (jnt->Active())
				{
					jnt->Update();

					s_block_cpy(6, jnt->GetCstDim(), jnt->GetPrtCstMtxIPtr(), 0, 0, jnt->GetCstDim(), C, jnt->_pMakI->GetFatherPrt()->_RowId, jnt->_ColId, C_dim);
					s_block_cpy(6, jnt->GetCstDim(), jnt->GetPrtCstMtxJPtr(), 0, 0, jnt->GetCstDim(), C, jnt->_pMakJ->GetFatherPrt()->_RowId, jnt->_ColId, C_dim);

					std::copy_n(jnt->GetPrtA_cPtr(), jnt->GetCstDim(), &a_c[jnt->_ColId]);
				}
			}
			for (auto &mot : _motions)
			{
				if (mot->Active())
				{
					mot->Update();

					s_block_cpy(6, 1, mot->GetPrtCstMtxIPtr(), 0, 0, 1, C, mot->_pMakI->GetFatherPrt()->_RowId, mot->_ColId, C_dim);
					s_block_cpy(6, 1, mot->GetPrtCstMtxJPtr(), 0, 0, 1, C, mot->_pMakJ->GetFatherPrt()->_RowId, mot->_ColId, C_dim);

					a_c[mot->_ColId] = *mot->GetPrtA_cPtr();
				}
			}
			for (auto &fce : _forces)
			{
				if (fce->Active())
				{
					fce->Update();

					s_daxpy(6, -1, fce->GetPrtFceIPtr(), 1, &f[fce->_pMakI->GetFatherPrt()->_RowId], 1);
					s_daxpy(6, -1, fce->GetPrtFceJPtr(), 1, &f[fce->_pMakJ->GetFatherPrt()->_RowId], 1);
				}
			}
			/*calculate D and b*/
			/* for D*/
			s_block_cpy(I_dim, I_dim, -1, I_mat, 0, 0, I_dim, 0, D, 0, 0, C_dim + I_dim);
			s_block_cpy(I_dim, C_dim, C, 0, 0, C_dim, D, 0, I_dim, C_dim + I_dim);
			s_block_cpyT(I_dim, C_dim, C, 0, 0, C_dim, D, I_dim, 0, C_dim + I_dim);

			s_block_cpy(I_dim, 1, f, 0, 0, 1, b, 0, 0, 1);
			s_block_cpy(C_dim, 1, a_c, 0, 0, 1, b, I_dim, 0, 1);
		}
		void MODEL_BASE::DynEnd(const double *x)
		{
			for (auto &prt : _parts)
			{
				if (prt->Active())
				{
					std::copy_n(&x[prt->_RowId], 6, prt->GetPrtAccPtr());
				}
			}
			for (auto &jnt : _joints)
			{
				if (jnt->Active())
				{
					std::copy_n(&x[jnt->_ColId + I_dim], jnt->GetCstDim(), jnt->GetCstFcePtr());
				}
			}
			for (auto &mot : _motions)
			{
				if (mot->Active())
				{
					mot->MotDynFce = x[mot->_ColId + I_dim];
				}
			}
		}
		void MODEL_BASE::DynUkn(double *a, double*f_c)
		{
			for (auto &prt : _parts)
			{
				if (prt->Active())
				{
					prt->Update();
					std::copy_n(prt->GetPrtAccPtr(), 6, &a[prt->_RowId]);
				}
			}
			for (auto &jnt : _joints)
			{
				if (jnt->Active())
				{
					std::copy_n(jnt->GetCstFcePtr(), jnt->GetCstDim(), &f_c[jnt->_ColId]);
				}
			}
			for (auto &mot : _motions)
			{
				if (mot->Active())
				{
					f_c[mot->_ColId] = mot->GetMotFceDyn();
				}
			}
		}
		void MODEL_BASE::Dyn()
		{
			DynPre(this->I_dim, this->C_dim);
			
			static std::vector<double> C(I_dim * C_dim);
			static std::vector<double> a_c(C_dim);
			static std::vector<double> I_mat(I_dim * I_dim);
			static std::vector<double> f(I_dim);
			static std::vector<double> D((I_dim + C_dim) * (I_dim + C_dim));
			static std::vector<double> b(I_dim + C_dim);

			DynMtx(C.data(), a_c.data(), I_mat.data(), f.data(), D.data(), b.data());

			static std::vector<double> s(I_dim + C_dim);
			double rcond = 0.000000000001;
			int rank;
			s_dgelsd(I_dim + C_dim, I_dim + C_dim, 1, D.data(), I_dim + C_dim, b.data(), 1,s.data(), rcond, &rank);

			DynEnd(b.data());
		}

		void MODEL_BASE::ClbPre(int &clb_dim_m, int &clb_dim_n, int &gamma_dim, int &frc_coe_dim)
		{
			DynPre(clb_dim_m, clb_dim_n);

			if (C_dim != I_dim)
			{
				throw std::logic_error("must calibrate square matrix");
			}

			clb_dim_m = 0;
			clb_dim_n = 0;
			gamma_dim = 0;
			frc_coe_dim = 0;

			for (auto &i : _motions)
			{
				/*对于未激活的motion,认为其正在受到力控制*/
				if (i->Active())
				{
					clb_dim_m++;
				}
				frc_coe_dim += 3;
				clb_dim_n += 3;
			}
			for (auto &i : _parts)
			{
				if (i->Active())
				{
					clb_dim_n += 10;
					gamma_dim += 10;
				}
			}

		}
		void MODEL_BASE::ClbMtx(double *clb_d_ptr, double *clb_b_ptr)
		{
			int clb_dim_m, clb_dim_n, gamma_dim, frc_coe_dim;
			ClbPre(clb_dim_m, clb_dim_n, gamma_dim, frc_coe_dim);

			int dim = I_dim;

			/*初始化*/
			static MATRIX _clb_d, _clb_b;

			_clb_d.Resize(clb_dim_m, clb_dim_n);
			_clb_b.Resize(clb_dim_m, 1);

			memset(_clb_d.Data(), 0, _clb_d.Length() * sizeof(double));
			memset(_clb_b.Data(), 0, _clb_b.Length() * sizeof(double));

			/*求A，即C的逆*/
			MATRIX A(dim, dim), B(dim, dim);
			std::vector<int> ipiv(dim);
			
			std::vector<double> C(dim * dim);
			std::vector<double> f(dim);
			
			for (int i = 0; i < 6; ++i)
			{
				C.data()[C_dim*(pGround->_RowId + i) + i] = 1;
			}
			for (auto &jnt : _joints)
			{
				if (jnt->Active())
				{
					jnt->Update();

					s_block_cpy(6, jnt->GetCstDim(), jnt->GetPrtCstMtxIPtr(), 0, 0, jnt->GetCstDim(), C.data(), jnt->_pMakI->GetFatherPrt()->_RowId, jnt->_ColId, C_dim);
					s_block_cpy(6, jnt->GetCstDim(), jnt->GetPrtCstMtxJPtr(), 0, 0, jnt->GetCstDim(), C.data(), jnt->_pMakJ->GetFatherPrt()->_RowId, jnt->_ColId, C_dim);
				}
			}
			for (auto &mot : _motions)
			{
				if (mot->Active())
				{
					mot->Update();

					s_block_cpy(6, 1, mot->GetPrtCstMtxIPtr(), 0, 0, 1, C.data(), mot->_pMakI->GetFatherPrt()->_RowId, mot->_ColId, C_dim);
					s_block_cpy(6, 1, mot->GetPrtCstMtxJPtr(), 0, 0, 1, C.data(), mot->_pMakJ->GetFatherPrt()->_RowId, mot->_ColId, C_dim);
				}
			}

			std::copy(C.begin(), C.end(), A.Data());
			s_dgeinv(dim, A.Data(), dim, ipiv.data());

			/*求B*/
			const int beginRow = dim - clb_dim_m;

			for (auto &i:_parts)
			{
				if (i->Active())
				{
					double cm[6][6];
					s_cmf(i->GetPrtVelPtr(), *cm);
					s_dgemm(clb_dim_m, 6, 6, 1, &A(beginRow,i->_RowId), dim, *cm, 6, 0, &B(beginRow, i->_RowId), dim);
				}
			}

			/*求解clb_d*/
			int col1 = 0, col2 = 0;

			for (auto &i:_parts)
			{
				if (i->Active())
				{
					double q[6]{0};
					std::copy_n(i->GetPrtAccPtr(), 6, q);
					s_daxpy(6, -1, i->GetPrtGravityPtr(), 1, q, 1);
					
					double v[6];
					std::copy_n(i->GetPrtVelPtr(), 6, v);

					for (int j = 0; j < clb_dim_m; ++j)
					{
						/*_clb_d[j][col1] = A[beginRow + j][col2 + 0] * q[0] + A[beginRow + j][col2 + 1] * q[1] + A[beginRow + j][col2 + 2] * q[2];
						_clb_d[j][col1 + 1] = A[beginRow + j][col2 + 1] * q[5] + A[beginRow + j][col2 + 5] * q[1] - A[beginRow + j][col2 + 2] * q[4] - A[beginRow + j][col2 + 4] * q[2];
						_clb_d[j][col1 + 2] = A[beginRow + j][col2 + 2] * q[3] + A[beginRow + j][col2 + 3] * q[2] - A[beginRow + j][col2 + 0] * q[5] - A[beginRow + j][col2 + 5] * q[0];
						_clb_d[j][col1 + 3] = A[beginRow + j][col2 + 0] * q[4] + A[beginRow + j][col2 + 4] * q[0] - A[beginRow + j][col2 + 1] * q[3] - A[beginRow + j][col2 + 3] * q[1];
						_clb_d[j][col1 + 4] = A[beginRow + j][col2 + 3] * q[3];
						_clb_d[j][col1 + 5] = A[beginRow + j][col2 + 4] * q[4];
						_clb_d[j][col1 + 6] = A[beginRow + j][col2 + 5] * q[5];
						_clb_d[j][col1 + 7] = A[beginRow + j][col2 + 3] * q[4] + A[beginRow + j][col2 + 4] * q[3];
						_clb_d[j][col1 + 8] = A[beginRow + j][col2 + 3] * q[5] + A[beginRow + j][col2 + 5] * q[3];
						_clb_d[j][col1 + 9] = A[beginRow + j][col2 + 4] * q[5] + A[beginRow + j][col2 + 5] * q[4];

						_clb_d[j][col1] += B[beginRow + j][col2 + 0] * v[0] + B[beginRow + j][col2 + 1] * v[1] + B[beginRow + j][col2 + 2] * v[2];
						_clb_d[j][col1 + 1] += B[beginRow + j][col2 + 1] * v[5] + B[beginRow + j][col2 + 5] * v[1] - B[beginRow + j][col2 + 2] * v[4] - B[beginRow + j][col2 + 4] * v[2];
						_clb_d[j][col1 + 2] += B[beginRow + j][col2 + 2] * v[3] + B[beginRow + j][col2 + 3] * v[2] - B[beginRow + j][col2 + 0] * v[5] - B[beginRow + j][col2 + 5] * v[0];
						_clb_d[j][col1 + 3] += B[beginRow + j][col2 + 0] * v[4] + B[beginRow + j][col2 + 4] * v[0] - B[beginRow + j][col2 + 1] * v[3] - B[beginRow + j][col2 + 3] * v[1];
						_clb_d[j][col1 + 4] += B[beginRow + j][col2 + 3] * v[3];
						_clb_d[j][col1 + 5] += B[beginRow + j][col2 + 4] * v[4];
						_clb_d[j][col1 + 6] += B[beginRow + j][col2 + 5] * v[5];
						_clb_d[j][col1 + 7] += B[beginRow + j][col2 + 3] * v[4] + B[beginRow + j][col2 + 4] * v[3];
						_clb_d[j][col1 + 8] += B[beginRow + j][col2 + 3] * v[5] + B[beginRow + j][col2 + 5] * v[3];
						_clb_d[j][col1 + 9] += B[beginRow + j][col2 + 4] * v[5] + B[beginRow + j][col2 + 5] * v[4];*/

						_clb_d(j, col1) = A(beginRow + j, col2 + 0) * q[0] + A(beginRow + j, col2 + 1) * q[1] + A(beginRow + j, col2 + 2) * q[2];
						_clb_d(j, col1 + 1) = A(beginRow + j, col2 + 1) * q[5] + A(beginRow + j, col2 + 5) * q[1] - A(beginRow + j, col2 + 2) * q[4] - A(beginRow + j, col2 + 4) * q[2];
						_clb_d(j, col1 + 2) = A(beginRow + j, col2 + 2) * q[3] + A(beginRow + j, col2 + 3) * q[2] - A(beginRow + j, col2 + 0) * q[5] - A(beginRow + j, col2 + 5) * q[0];
						_clb_d(j, col1 + 3) = A(beginRow + j, col2 + 0) * q[4] + A(beginRow + j, col2 + 4) * q[0] - A(beginRow + j, col2 + 1) * q[3] - A(beginRow + j, col2 + 3) * q[1];
						_clb_d(j, col1 + 4) = A(beginRow + j, col2 + 3) * q[3];
						_clb_d(j, col1 + 5) = A(beginRow + j, col2 + 4) * q[4];
						_clb_d(j, col1 + 6) = A(beginRow + j, col2 + 5) * q[5];
						_clb_d(j, col1 + 7) = A(beginRow + j, col2 + 3) * q[4] + A(beginRow + j, col2 + 4) * q[3];
						_clb_d(j, col1 + 8) = A(beginRow + j, col2 + 3) * q[5] + A(beginRow + j, col2 + 5) * q[3];
						_clb_d(j, col1 + 9) = A(beginRow + j, col2 + 4) * q[5] + A(beginRow + j, col2 + 5) * q[4];

						_clb_d(j, col1) += B(beginRow + j, col2 + 0) * v[0] + B(beginRow + j, col2 + 1) * v[1] + B(beginRow + j, col2 + 2) * v[2];
						_clb_d(j, col1 + 1) += B(beginRow + j, col2 + 1) * v[5] + B(beginRow + j, col2 + 5) * v[1] - B(beginRow + j, col2 + 2) * v[4] - B(beginRow + j, col2 + 4) * v[2];
						_clb_d(j, col1 + 2) += B(beginRow + j, col2 + 2) * v[3] + B(beginRow + j, col2 + 3) * v[2] - B(beginRow + j, col2 + 0) * v[5] - B(beginRow + j, col2 + 5) * v[0];
						_clb_d(j, col1 + 3) += B(beginRow + j, col2 + 0) * v[4] + B(beginRow + j, col2 + 4) * v[0] - B(beginRow + j, col2 + 1) * v[3] - B(beginRow + j, col2 + 3) * v[1];
						_clb_d(j, col1 + 4) += B(beginRow + j, col2 + 3) * v[3];
						_clb_d(j, col1 + 5) += B(beginRow + j, col2 + 4) * v[4];
						_clb_d(j, col1 + 6) += B(beginRow + j, col2 + 5) * v[5];
						_clb_d(j, col1 + 7) += B(beginRow + j, col2 + 3) * v[4] + B(beginRow + j, col2 + 4) * v[3];
						_clb_d(j, col1 + 8) += B(beginRow + j, col2 + 3) * v[5] + B(beginRow + j, col2 + 5) * v[3];
						_clb_d(j, col1 + 9) += B(beginRow + j, col2 + 4) * v[5] + B(beginRow + j, col2 + 5) * v[4];

					}
					col1 += 10;
					col2 += 6;
				}
			}

			/*求解clb_b*/
			std::fill(f.begin(), f.end(), 0);
			
			int row = 0;
			for (auto &mot : _motions)
			{
				if (mot->Active())
				{
					_clb_b(row, 0) = mot->GetMotFce();
					row++;
				}
				else
				{
					mot->Update();

					s_daxpy(6, mot->GetMotFce(), mot->GetPrtCstMtxIPtr(), 1, &f[mot->_pMakI->GetFatherPrt()->_RowId], 1);
					s_daxpy(6, mot->GetMotFce(), mot->GetPrtCstMtxJPtr(), 1, &f[mot->_pMakJ->GetFatherPrt()->_RowId], 1);
				}
			}

			s_dgemm(clb_dim_m, 1, dim, 1, &A(beginRow,0), dim, f.data(), 1, 1, _clb_b.Data(), 1);

			/*以下添加驱动摩擦系数*/
			row = 0;
			int num = 0;
			for (auto &mot : _motions)
			{
				//默认未激活的motion处于力控模式
				if (mot->Active())
				{
					_clb_d(row, gamma_dim + num * 3) += s_sgn(mot->GetMotVel());
					_clb_d(row, gamma_dim + num * 3 + 1) += mot->GetMotVel();
					_clb_d(row, gamma_dim + num * 3 + 2) += mot->GetMotAcc();
					++row;
				}
				else
				{
					s_dgemm(clb_dim_m, 1, 6, s_sgn(mot->GetMotVel()), &A(beginRow, mot->_pMakI->_pPrt->_RowId), dim, mot->GetPrtCstMtxIPtr(), 1, 1, &_clb_d(0, gamma_dim + num * 3), clb_dim_n);
					s_dgemm(clb_dim_m, 1, 6, s_sgn(mot->GetMotVel()), &A(beginRow, mot->_pMakJ->_pPrt->_RowId), dim, mot->GetPrtCstMtxJPtr(), 1, 1, &_clb_d(0, gamma_dim + num * 3), clb_dim_n);
					s_dgemm(clb_dim_m, 1, 6, mot->GetMotVel(), &A(beginRow, mot->_pMakI->_pPrt->_RowId), dim, mot->GetPrtCstMtxIPtr(), 1, 1, &_clb_d(0, gamma_dim + num * 3 + 1), clb_dim_n);
					s_dgemm(clb_dim_m, 1, 6, mot->GetMotVel(), &A(beginRow, mot->_pMakJ->_pPrt->_RowId), dim, mot->GetPrtCstMtxJPtr(), 1, 1, &_clb_d(0, gamma_dim + num * 3 + 1), clb_dim_n);
					s_dgemm(clb_dim_m, 1, 6, mot->GetMotAcc(), &A(beginRow, mot->_pMakI->_pPrt->_RowId), dim, mot->GetPrtCstMtxIPtr(), 1, 1, &_clb_d(0, gamma_dim + num * 3 + 2), clb_dim_n);
					s_dgemm(clb_dim_m, 1, 6, mot->GetMotAcc(), &A(beginRow, mot->_pMakJ->_pPrt->_RowId), dim, mot->GetPrtCstMtxJPtr(), 1, 1, &_clb_d(0, gamma_dim + num * 3 + 2), clb_dim_n);

					//s_dgemm(clb_dim_m, 1, 6, s_sgn(*i->GetV_mPtr()), &A(beginRow, i->_pMakI->_pPrt->_RowId), dim, i->_PrtCstMtxI[0], 6, 1, &_clb_d(0, clb_prt_dim_n + num * 3), clb_dim_n);
					//s_dgemm(clb_dim_m, 1, 6, s_sgn(*i->GetV_mPtr()), &A(beginRow, i->_pMakJ->_pPrt->_RowId), dim, i->_PrtCstMtxJ[0], 6, 1, &_clb_d(0, clb_prt_dim_n + num * 3), clb_dim_n);
					//s_dgemm(clb_dim_m, 1, 6, *i->GetV_mPtr(), &A(beginRow, i->_pMakI->_pPrt->_RowId), dim, i->_PrtCstMtxI[0], 6, 1, &_clb_d(0, clb_prt_dim_n + num * 3 + 1), clb_dim_n);
					//s_dgemm(clb_dim_m, 1, 6, *i->GetV_mPtr(), &A(beginRow, i->_pMakJ->_pPrt->_RowId), dim, i->_PrtCstMtxJ[0], 6, 1, &_clb_d(0, clb_prt_dim_n + num * 3 + 1), clb_dim_n);
					//s_dgemm(clb_dim_m, 1, 6, *i->GetA_mPtr(), &A(beginRow, i->_pMakI->_pPrt->_RowId), dim, i->_PrtCstMtxI[0], 6, 1, &_clb_d(0, clb_prt_dim_n + num * 3 + 2), clb_dim_n);
					//s_dgemm(clb_dim_m, 1, 6, *i->GetA_mPtr(), &A(beginRow, i->_pMakJ->_pPrt->_RowId), dim, i->_PrtCstMtxJ[0], 6, 1, &_clb_d(0, clb_prt_dim_n + num * 3 + 2), clb_dim_n);

				}

				num++;
			}

			std::copy_n(_clb_d.Data(), _clb_d.Length(), clb_d_ptr);
			std::copy_n(_clb_b.Data(), _clb_b.Length(), clb_b_ptr);
		}
		void MODEL_BASE::ClbUkn(double *clb_gamma_and_frcCoe_ptr)
		{
			int clb_dim_m, clb_dim_n, gamma_dim, frc_coe_dim;
			ClbPre(clb_dim_m, clb_dim_n, gamma_dim, frc_coe_dim);
			
			int row = 0;
			for (auto &prt : _parts)
			{
				if (prt->Active())
				{
					s_im2gamma(prt->GetPrtImPtr(), clb_gamma_and_frcCoe_ptr + row);
				}
				row += 10;
			}

			for (auto &mot : _motions)
			{
				std::copy_n(mot->GetFrcCoePtr(),3, clb_gamma_and_frcCoe_ptr + row);
				row += 3;
			}
		}
		
		void MODEL_BASE::LoadXml(const char *filename)
		{
			Aris::Core::DOCUMENT XML_Doc;
			
			if (XML_Doc.LoadFile(filename) != 0)
			{
				throw std::logic_error((string("could not open file:") + string(filename)));
			}

			const Aris::Core::ELEMENT *pModel = XML_Doc.RootElement()->FirstChildElement("Model");
			
			FromXmlElement(pModel);
		}
		void MODEL_BASE::FromXmlElement(const Aris::Core::ELEMENT *pModel)
		{
			if (pModel == nullptr)throw(std::logic_error("XML file must have model element"));

			const Aris::Core::ELEMENT *pVar = pModel->FirstChildElement("Variable");
			if (pModel == nullptr)throw(std::logic_error("Model must have variable element"));
			const Aris::Core::ELEMENT *pEnv = pModel->FirstChildElement("Environment");
			if (pEnv == nullptr)throw(std::logic_error("Model must have environment element"));
			const Aris::Core::ELEMENT *pPrt = pModel->FirstChildElement("Part");
			if (pPrt == nullptr)throw(std::logic_error("Model must have part element"));

			calculator.ClearVariables();
			for (auto ele = pVar->FirstChildElement(); ele != nullptr; ele = ele->NextSiblingElement())
			{
				calculator.AddVariable(ele->Name(), calculator.CalculateExpression(ele->GetText()));
			}

			_Environment.FromXmlElement(pEnv);

			_parts.clear();
			_joints.clear();
			_motions.clear();
			_forces.clear();

			/*读入地面*/
			for (auto ele = pPrt->FirstChildElement(); ele != nullptr; ele = ele->NextSiblingElement())
			{
				if (std::string(ele->Name()) == "Ground")
				{
					AddPart(ele->Name(), ele);
					pGround = GetPart("Ground");
					break;
				}
			}

			if (this->GetPart("Ground") == nullptr)
			{
				throw std::logic_error("Model must contain a Ground");
			}

			/*读入其他部件*/
			for (auto ele = pPrt->FirstChildElement(); ele != nullptr; ele = ele->NextSiblingElement())
			{
				if (std::string(ele->Name()) != "Ground")
				{
					AddPart(ele->Name(), ele);
				}
			}
		}
		void MODEL_BASE::SaveSnapshotXml(const char *filename) const
		{
			Aris::Core::DOCUMENT XML_Doc;
			XML_Doc.DeleteChildren();

			Aris::Core::DECLARATION *pHeader = XML_Doc.NewDeclaration("xml version=\"1.0\" encoding=\"UTF-8\" ");
			XML_Doc.InsertFirstChild(pHeader);

			Aris::Core::ELEMENT *pModel = XML_Doc.NewElement("Model");
			XML_Doc.InsertEndChild(pModel);

			Aris::Core::ELEMENT *pEnvironment = XML_Doc.NewElement("");
			_Environment.ToXmlElement(pEnvironment);
			pModel->InsertEndChild(pEnvironment);

			Aris::Core::ELEMENT *pVar = XML_Doc.NewElement("Variable");
			pModel->InsertEndChild(pVar);

			Aris::Core::ELEMENT *pPrt = XML_Doc.NewElement("Part");
			pModel->InsertEndChild(pPrt);

			Aris::Core::ELEMENT *pJnt = XML_Doc.NewElement("Joint");
			pModel->InsertEndChild(pJnt);

			Aris::Core::ELEMENT *pMot = XML_Doc.NewElement("Motion");
			pModel->InsertEndChild(pMot);

			Aris::Core::ELEMENT *pFce = XML_Doc.NewElement("Force");
			pModel->InsertEndChild(pFce);

			for (auto &p:_parts)
			{
				Aris::Core::ELEMENT *ele = XML_Doc.NewElement("");
				p->ToXmlElement(ele);
				pPrt->InsertEndChild(ele);
			}

			for (auto &j : _joints)
			{
				Aris::Core::ELEMENT *ele = XML_Doc.NewElement("");
				j->ToXmlElement(ele);
				pJnt->InsertEndChild(ele);
			}

			for (auto &m : _motions)
			{
				Aris::Core::ELEMENT *ele = XML_Doc.NewElement("");
				m->ToXmlElement(ele);
				pMot->InsertEndChild(ele);
			}


			XML_Doc.SaveFile(filename);
		}
		void MODEL_BASE::SaveAdams(const char *filename, const SIMULATE_SCRIPT* pScript) const
		{
			ofstream file;

			std::string cmdName = std::string(filename) + std::string(".cmd");
			std::string acfName = std::string(filename) + std::string(".acf");
			std::string fullAcfPath;
			/*******写acf文件********/
			if (pScript)
			{
				const int maxPath = 2000;

#ifdef PLATFORM_IS_WINDOWS
				char fullPath[maxPath];
				if (_fullpath(fullPath, acfName.c_str(), maxPath) == nullptr)
				{
					throw std::logic_error("save adams failed, because can not identify fullpath");
				};
				fullAcfPath = std::string(fullPath);
#endif
#ifdef PLATFORM_IS_LINUX
				const char *fullPath = realpath(acfName.c_str(), nullptr);
				fullAcfPath = std::string(fullPath);
				delete[] fullPath;
#endif

				/*创建acf文件*/
				file.open(acfName, std::ios::out | std::ios::trunc);

				file << setprecision(15);

				double beginTime = 0;
				double endTime = pScript->endTime/1000.0;
				double dt = pScript->dt/1000.0;


				file << "\r\n\r\n";
				/*设置起始的关节和电机状态*/
				
				/*设置脚本*/
				for (auto &i : pScript->script)
				{
					double now = double(i.first) / 1000.0;
					if (now > beginTime)
					{
						file << "simulate/transient, dur=" << now - beginTime << ", dtout=" << dt << "\r\n";
						beginTime = now;
					}

					for (auto &j : i.second.elements)
					{
						if (auto ele = dynamic_cast<JOINT_BASE *>(j.first))
						{
							if (j.second)
							{
								file << "activate/joint, id=" << ele->GetID() + 1 << "\r\n";
							}
							else
							{
								file << "deactivate/joint, id=" << ele->GetID() + 1 << "\r\n";
							}
						}
						if (auto ele = dynamic_cast<MOTION_BASE *>(j.first))
						{
							if (j.second)
							{
								file << "activate/motion, id=" << ele->GetID() + 1 << "\r\n";
							}
							else
							{
								file << "deactivate/motion, id=" << ele->GetID() + 1 << "\r\n";
							}
						}
						if (auto ele = dynamic_cast<FORCE_BASE *>(j.first))
						{
							if (j.second)
							{
								file << "activate/sforce, id=" << ele->GetID() + 1 << "\r\n";
							}
							else
							{
								file << "deactivate/sforce, id=" << ele->GetID() + 1 << "\r\n";
							}
						}

						
						
					}

					for (auto &j : i.second.markers)
					{
						auto mak = j.first;
						auto pe = j.second.data();
						
						file << "marker/" << mak->GetID() + 1
							<< " , QP = " << pe[0] << "," << pe[1] << "," << pe[2]
							<< " , REULER =" << pe[3] << "," << pe[4] << "," << pe[5] << "\r\n";
					}
				}

				if (endTime > beginTime)
				{
					file << "simulate/transient, dur=" << endTime - beginTime << ", dtout=" << dt << "\r\n";
				}

				file.close();
			}
			
			/*******写cmd文件********/
			file.open(cmdName, std::ios::out | std::ios::trunc);

			file << setprecision(15);

			/*  Basic  */
			file << "!-------------------------- Default Units for Model ---------------------------!\r\n"
				<< "!\r\n"
				<< "!\r\n"
				<< "defaults units  &\r\n"
				<< "    length = meter  &\r\n"
				<< "    angle = rad  &\r\n"
				<< "    force = newton  &\r\n"
				<< "    mass = kg  &\r\n"
				<< "    time = sec\r\n"
				<< "!\n"
				<< "defaults units  &\r\n"
				<< "    coordinate_system_type = cartesian  &\r\n"
				<< "    orientation_type = body313\r\n"
				<< "!\r\n"
				<< "!------------------------ Default Attributes for Model ------------------------!\r\n"
				<< "!\r\n"
				<< "!\r\n"
				<< "defaults attributes  &\r\n"
				<< "    inheritance = bottom_up  &\r\n"
				<< "    icon_visibility = off  &\r\n"
				<< "    grid_visibility = off  &\r\n"
				<< "    size_of_icons = 5.0E-002  &\r\n"
				<< "    spacing_for_grid = 1.0\r\n"
				<< "!\r\n"
				<< "!------------------------------ Adams/View Model ------------------------------!\r\n"
				<< "!\r\n"
				<< "!\r\n"
				<< "model create  &\r\n"
				<< "   model_name = " << this->Name() << "\r\n"
				<< "!\r\n"
				<< "view erase\r\n"
				<< "!\r\n"
				<< "!---------------------------------- Accgrav -----------------------------------!\r\n"
				<< "!\r\n"
				<< "!\r\n"
				<< "force create body gravitational  &\r\n"
				<< "    gravity_field_name = gravity  &\r\n"
				<< "    x_component_gravity = 0.0  &\r\n"
				<< "    y_component_gravity = -9.8  &\r\n"
				<< "    z_component_gravity = 0.0\r\n"
				<< "!\r\n";

			/*  Create Parts  */
			file << "!---------------------------------- Parts ---------------------------------!\r\n"
				<< "!\r\n"
				<< "!\r\n";

			pGround->ToAdamsCmd(file);

			for (auto &prt : _parts)
			{
				if (prt.get() != pGround)
				{
					if (prt->Active() || (pScript != nullptr))
						prt->ToAdamsCmd(file);
				}
			}

			file << "!----------------------------------- Joints -----------------------------------!\r\n"
				<< "!\r\n"
				<< "!\r\n";

			for (auto &jnt : _joints)
			{
				if ((jnt->Active()) || (pScript!=nullptr))
				{
					jnt->ToAdamsCmd(file);
				}
			}

			file << "!----------------------------------- Motions -----------------------------------!\r\n"
				<< "!\r\n"
				<< "!\r\n";

			for (auto &mot : _motions)
			{
				if (mot->Active() || (pScript != nullptr))
				{
					mot->ToAdamsCmd(file);
				}
			}

			file << "!----------------------------------- Forces -----------------------------------!\r\n"
				<< "!\r\n"
				<< "!\r\n";

			for (auto &fce : _forces)
			{
				if (fce->Active() || (pScript != nullptr))
				{
					fce->ToAdamsCmd(file);
				}
			}

			/*读入acf文件*/
			if (pScript)
			{
				file << "!-------------------------- Reading ACF File ---------------------------!\r\n"
					<< "simulation script read_acf &\n"
					<< "    sim_script_name = SIM_SCRIPT_1 &\n"
					<< "file_name = \"" << fullAcfPath << "\"";
				
			}
			file.close();
		}
		void MODEL_BASE::SaveAdams(const char *filename, bool isModifyActive) const
		{
			ofstream file;

			std::string cmdName = std::string(filename) + std::string(".cmd");

			/*******写cmd文件********/
			file.open(cmdName, std::ios::out | std::ios::trunc);

			file << setprecision(15);

			file << "!----------------------------------- Environment -------------------------------!\r\n!\r\n!\r\n";
			_Environment.ToAdamsCmd(file);

			file << "!----------------------------------- Parts -------------------------------------!\r\n!\r\n!\r\n";
			pGround->ToAdamsCmd(file);
			for (auto &prt : _parts)
			{
				if (prt.get() != pGround)
				{
					prt->ToAdamsCmd(file);
				}
			}

			file << "!----------------------------------- Joints ------------------------------------!\r\n!\r\n!\r\n";
			for (auto &jnt : _joints)
			{
				jnt->ToAdamsCmd(file);
			}

			file << "!----------------------------------- Motions -----------------------------------!\r\n!\r\n!\r\n";
			for (auto &mot : _motions)
			{
				mot->ToAdamsCmd(file);
			}

			file << "!----------------------------------- Forces ------------------------------------!\r\n!\r\n!\r\n";
			for (auto &fce : _forces)
			{
				fce->ToAdamsCmd(file);
			}

			if (isModifyActive)
			{
				file << "!----------------------------------- Motify Active -------------------------------------!\r\n!\r\n!\r\n";
				for (auto &prt : _parts)
				{
					if ((prt.get() != pGround) && (!prt->Active()))
					{
						file << "part attributes  &\r\n"
							<< "    constraint_name = ." << Name() << "." << prt->Name() << "  &\r\n"
							<< "    active = off \r\n!\r\n";
					}
				}
				for (auto &jnt : _joints)
				{
					if (!jnt->Active())
					{
						file << "constraint attributes  &\r\n"
							<< "    constraint_name = ." << Name() << "." << jnt->Name() << "  &\r\n"
							<< "    active = off \r\n!\r\n";
					}
					
				}
				for (auto &mot : _motions)
				{
					if (!mot->Active())
					{
						file << "constraint attributes  &\r\n"
							<< "    constraint_name = ." << Name() << "." << mot->Name() << "  &\r\n"
							<< "    active = off \r\n!\r\n";
					}
					
				}
				for (auto &fce : _forces)
				{
					if (!fce->Active())
					{
						file << "force attributes  &\r\n"
							<< "    force_name = ." << Name() << "." << fce->Name() << "  &\r\n"
							<< "    active = off \r\n!\r\n";
					}
				}
			}
			
			file.close();


		}
	}
}
