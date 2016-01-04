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
#include "Aris_DynModelBase.h"

namespace Aris
{
	namespace DynKer
	{
		Marker::Marker(const Part *pPrt, const double *_prtPe, const char* eulType)
			: Element(nullptr, "", 0), pPrt(pPrt)
		{
			static const double defaultPe[6] = { 0 };
			_prtPe = _prtPe ? _prtPe : defaultPe;

			s_pe2pm(_prtPe, *prtPm, eulType);
		}
		Marker::Marker(ModelBase *pModel, Part *pPrt, const std::string &Name, int id)
			: Element(pModel, Name, id), pPrt(pPrt)
		{
			std::fill_n(static_cast<double *>(*prtPm), 16, 0);
			prtPm[0][0] = 1;
			prtPm[1][1] = 1;
			prtPm[2][2] = 1;
			prtPm[3][3] = 1;

			std::fill_n(static_cast<double *>(*this->pm), 16, 0);
			this->pm[0][0] = 1;
			this->pm[1][1] = 1;
			this->pm[2][2] = 1;
			this->pm[3][3] = 1;
		}
		Marker::Marker(Part *pPrt, const std::string &Name, int id, const double *pPrtPm, Marker *pRelativeTo)
			: Marker(&pPrt->Model(), pPrt, Name, id)
		{
			static const double default_pm_in[16] = { 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
			pPrtPm = pPrtPm ? pPrtPm : default_pm_in;
			
			
			if (pRelativeTo)
			{
				if (&pRelativeTo->Father() != pPrt)
					throw std::logic_error("relative marker must has same father part with this marker");
				
				s_pm_dot_pm(*pRelativeTo->PrtPm(), pPrtPm, *prtPm);
			}
			else
			{
				std::copy_n(pPrtPm, 16, static_cast<double *>(*prtPm));
			}
		}
		Marker::Marker(Part *pPrt, const std::string &Name, int id, const Aris::Core::XmlElement *ele)
			: Marker(&pPrt->Model(), pPrt, Name, id)
		{
			double pm[16];
			Matrix m = Model().calculator.CalculateExpression(ele->Attribute("Pos"));
			s_pe2pm(m.Data(), pm);

			if (ele->Attribute("RelativeTo") && (!ele->Attribute("RelativeTo", "")))
			{
				const Marker *pRelativeMak = Father().FindMarker(ele->Attribute("RelativeTo"));
				s_pm_dot_pm(*pRelativeMak->PrtPm(), pm, *prtPm);
			}
			else
			{
				std::copy_n(pm, 16, static_cast<double*>(*prtPm));
			}
		}
		const double6& Marker::Vel() const { return Father().Vel(); };
		const double6& Marker::Acc() const { return Father().Acc(); };
		void Marker::Update()
		{
			s_pm_dot_pm(*Father().Pm(), *prtPm, *this->pm);
		}
		void Marker::ToXmlElement(Aris::Core::XmlElement *pEle) const
		{
			double value[10];

			pEle->DeleteChildren();
			pEle->SetName(this->Name().data());

			Aris::Core::XmlElement *pPE = pEle->GetDocument()->NewElement("Pos");
			s_pm2pe(*prtPm, value);
			pPE->SetText(Matrix(1, 6, value).ToString().c_str());
			pEle->InsertEndChild(pPE);

			Aris::Core::XmlElement *pRelativeMakEle = pEle->GetDocument()->NewElement("RelativeTo");
			pRelativeMakEle->SetText("");
			pEle->InsertEndChild(pRelativeMakEle);
		}
		
		Part::Part(ModelBase *pModel, const std::string &Name, int id, const double *Im, const double *pm, const double *Vel, const double *Acc)
			: Marker(pModel, this, Name, id)
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
				std::fill_n(static_cast<double *>(*this->pm), 16, 0);
				this->pm[0][0] = 1;
				this->pm[1][1] = 1;
				this->pm[2][2] = 1;
				this->pm[3][3] = 1;
			}
			else
			{
				SetPm(pm);
			}

			if (Vel == nullptr)
			{
				std::fill_n(vel, 6, 0);
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
		Part::Part(ModelBase *pModel, const std::string &Name, int id, const Aris::Core::XmlElement *ele)
			: Marker(pModel, this, Name, id)
		{
			if (ele->Attribute("Active", "true"))
			{
				this->Activate();
			}
			else if (ele->Attribute("Active", "false"))
			{
				this->Activate(false);
			}
			else
			{
				throw std::logic_error("failed load xml file in part");
			}

			Matrix m;

			m = Model().calculator.CalculateExpression(ele->Attribute("Inertia"));
			s_gamma2im(m.Data(), *_PrtIm);

			m = Model().calculator.CalculateExpression(ele->Attribute("Pos"));
			s_pe2pm(m.Data(), *pm);

			m = Model().calculator.CalculateExpression(ele->Attribute("Vel"));
			std::copy_n(m.Data(), 6, Vel());

			m = Model().calculator.CalculateExpression(ele->Attribute("Acc"));
			std::copy_n(m.Data(), 6, Acc());

			_markerNames.clear();

			for (auto makEle = ele->FirstChildElement("ChildMarker")->FirstChildElement(); makEle != nullptr; makEle = makEle->NextSiblingElement())
			{
				AddMarker(makEle->Name(), makEle);
			}

			if (ele->Attribute("Graphic_File_Path") && (!ele->Attribute("Graphic_File_Path", "")))
				graphicFilePath = ele->Attribute("Graphic_File_Path");
		}
		void Part::Update()
		{
			double tem[6];
		
			s_inv_pm(*this->pm, *_InvPm);
			s_tv(*_InvPm, vel, _PrtVel);
			s_tv(*_InvPm, _Acc, _PrtAcc);
			s_tv(*_InvPm, Model()._Environment.Gravity, _PrtGravity);
			s_m6_dot_v6(*_PrtIm, _PrtGravity, _PrtFg);
			s_m6_dot_v6(*_PrtIm, _PrtVel, tem);
			s_cf(_PrtVel, tem, _PrtFv);
		}
		Marker* Part::FindMarker(const std::string &Name)
		{
			auto pMak = _markerNames.find(Name);
			if (pMak != _markerNames.end())
			{
				return Model()._markers.at(pMak->second).get();
			}
			else
			{
				return nullptr;
			}
		}
		const Marker* Part::FindMarker(const std::string &Name)const
		{
			auto pMak = _markerNames.find(Name);
			if (pMak != _markerNames.end())
			{
				return Model()._markers.at(pMak->second).get();
			}
			else
			{
				return nullptr;
			}
		}
		void Part::ToXmlElement(Aris::Core::XmlElement *pEle) const
		{
			double value[10];
			
			pEle->DeleteChildren();
			pEle->SetName(this->Name().data());

			Aris::Core::XmlElement *pActive = pEle->GetDocument()->NewElement("Active");
			if (this->IsActive())
				pActive->SetText("True");
			else
				pActive->SetText("False");
			pEle->InsertEndChild(pActive);
			
			Aris::Core::XmlElement *pInertia = pEle->GetDocument()->NewElement("Inertia");
			s_im2gamma(*this->PrtIm(),value);
			pInertia->SetText(Matrix(1,10,value).ToString().c_str());
			pEle->InsertEndChild(pInertia);

			Aris::Core::XmlElement *pPE = pEle->GetDocument()->NewElement("Pos");
			s_pm2pe(*this->Pm(), value);
			pPE->SetText(Matrix(1, 6, value).ToString().c_str());
			pEle->InsertEndChild(pPE);

			Aris::Core::XmlElement *pVel = pEle->GetDocument()->NewElement("Vel");
			pVel->SetText(Matrix(1, 6, vel).ToString().c_str());
			pEle->InsertEndChild(pVel);

			Aris::Core::XmlElement *pAcc = pEle->GetDocument()->NewElement("Acc");
			pAcc->SetText(Matrix(1, 6, _Acc).ToString().c_str());
			pEle->InsertEndChild(pAcc);

			Aris::Core::XmlElement *pChildMak = pEle->GetDocument()->NewElement("ChildMarker");
			pEle->InsertEndChild(pChildMak);

			for (auto &m:_markerNames)
			{
				Aris::Core::XmlElement *ele = pEle->GetDocument()->NewElement("");

				Model()._markers.at(m.second)->ToXmlElement(ele);
				pChildMak->InsertEndChild(ele);
			}

			Aris::Core::XmlElement *pGraphicFilePath = pEle->GetDocument()->NewElement("Graphic_File_Path");
			pGraphicFilePath->SetText(this->graphicFilePath.c_str());
			pEle->InsertEndChild(pGraphicFilePath);
		}
		void Part::ToAdamsCmd(std::ofstream &file) const
		{
			if (this == Model().pGround)
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
					<< "    default_coordinate_system = ." << Model().Name() << ".ground\r\n"
					<< "!\r\n"
					<< "! ****** Markers for current part ******\r\n"
					<< "!\r\n";
			}
			else
			{
				double pe[6];
				s_pm2pe(*this->Pm(), pe, "313");
				Matrix ori(1, 3, &pe[3]), loc(1, 3, &pe[0]);

				file << "!----------------------------------- " << this->Name() << " -----------------------------------!\r\n"
					<< "!\r\n"
					<< "!\r\n"
					<< "defaults coordinate_system  &\r\n"
					<< "    default_coordinate_system = ." << Model().Name() << ".ground\r\n"
					<< "!\r\n"
					<< "part create rigid_body name_and_position  &\r\n"
					<< "    part_name = ." << Model().Name() << "." << this->Name() << "  &\r\n"
					<< "    adams_id = " << this->ID() + 1 << "  &\r\n"
					<< "    location = (" << loc.ToString() << ")  &\r\n"
					<< "    orientation = (" << ori.ToString() << ")\r\n"
					<< "!\r\n"
					<< "defaults coordinate_system  &\r\n"
					<< "    default_coordinate_system = ." << Model().Name() << "." << this->Name() << " \r\n"
					<< "!\r\n";

				
				double mass = this->PrtIm()[0][0] == 0 ? 1 : PrtIm()[0][0];
				std::fill_n(pe, 6, 0);
				pe[0] = this->PrtIm()[1][5] / mass;
				pe[1] = -this->PrtIm()[0][5] / mass;
				pe[2] = this->PrtIm()[0][4] / mass;				

				file << "! ****** cm and mass for current part ******\r\n"
					<< "marker create  &\r\n"
					<< "    marker_name = ." << Model().Name() << "." << this->Name() << ".cm  &\r\n"
					<< "    adams_id = " << this->ID() + Model().GetMarkerNum() << "  &\r\n"
					<< "    location = ({" << pe[0] << "," << pe[1] << "," << pe[2] << "})  &\r\n"
					<< "    orientation = (" << "{0,0,0}" << ")\r\n"
					<< "!\r\n";

				double pm[16];
				double im[6][6];

				pe[0] = -pe[0];
				pe[1] = -pe[1];
				pe[2] = -pe[2];

				s_pe2pm(pe, pm);
				s_i2i(pm, *this->PrtIm(), *im);

				/*！注意！*/
				//Adams里对惯量矩阵的定义貌似和我自己的定义在Ixy，Ixz，Iyz上互为相反数。别问我为什么，我也不知道。
				file << "part create rigid_body mass_properties  &\r\n"
					<< "    part_name = ." << Model().Name() << "." << this->Name() << "  &\r\n"
					<< "    mass = " << this->PrtIm()[0][0] << "  &\r\n"
					<< "    center_of_mass_marker = ." << Model().Name() << "." << this->Name() << ".cm  &\r\n"
					<< "    inertia_marker = ." << Model().Name() << "." << this->Name() << ".cm  &\r\n"
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

				s_pm2pe(*Model().GetMarker(mak.second)->PrtPm(), pe, "313");
				Matrix ori(1, 3, &pe[3]), loc(1, 3, &pe[0]);

				file << "marker create  &\r\n"
					<< "marker_name = ." << Model().Name() << "." << this->Name() << "." << mak.first << "  &\r\n"
					<< "adams_id = " << mak.second + 1 << "  &\r\n"
					<< "location = (" << loc.ToString() << ")  &\r\n"
					<< "orientation = (" << ori.ToString() << ")\r\n"
					<< "!\r\n";
			}
			//导入parasolid
			std::stringstream stream(this->graphicFilePath);
			std::string path;
			while (stream >> path)
			{
				file << "file parasolid read &\r\n"
					<< "file_name = \"" << path << "\" &\r\n"
					<< "type = ASCII" << " &\r\n"
					<< "part_name = " << this->Name() << " \r\n"
					<< "\r\n";
			}
		}

		JointBase::JointBase(ModelBase *pModel, const std::string &Name, int id, Marker *pMakI, Marker *pMakJ)
			: Element(pModel, Name, id)
			, _pMakI(pMakI)
			, _pMakJ(pMakJ)
		{

		}
		JointBase::JointBase(ModelBase *pModel, const std::string &Name, int id, const Aris::Core::XmlElement *ele)
			: Element(pModel, Name, id)
		{
			if (strcmp("true", ele->Attribute("Active")) == 0)
			{
				this->Activate();
			}
			else if (strcmp("false", ele->Attribute("Active")) == 0)
			{
				this->Activate(false);
			}
			else
			{
				throw std::logic_error("failed load xml file in joint");
			}

			_pMakI = Model().GetPart(ele->Attribute("PrtM"))->FindMarker(ele->Attribute("MakI"));
			_pMakJ = Model().GetPart(ele->Attribute("PrtN"))->FindMarker(ele->Attribute("MakJ"));

		}
		void JointBase::Update()
		{
			double _pm_M2N[4][4];
			double _tem_v1[6]{ 0 }, _tem_v2[6]{ 0 };

			/* Get pm M2N */
			s_pm_dot_pm(*MakJ().Father().InvPm(), *MakI().Father().Pm(), *_pm_M2N);

			/*update PrtCstMtx*/
			std::fill_n(this->PrtCstMtxJ(), this->CstDim() * 6, 0);
			s_tf_n(CstDim(), -1, *_pm_M2N, this->PrtCstMtxI(), 0, this->PrtCstMtxJ());

			/*update A_c*/
			std::fill_n(this->PrtA_c(), this->CstDim(), 0);
			s_inv_tv(-1, *_pm_M2N, MakJ().Father().PrtVel(), 0, _tem_v1);
			s_cv(MakI().Father().PrtVel(), _tem_v1, _tem_v2);
			s_dgemmTN(CstDim(), 1, 6, 1, PrtCstMtxI(), CstDim(), _tem_v2, 1, 0, PrtA_c(), 1);
		};
		void JointBase::ToXmlElement(Aris::Core::XmlElement *pEle) const
		{
			pEle->DeleteChildren();
			pEle->SetName(this->Name().data());

			Aris::Core::XmlElement *pActive = pEle->GetDocument()->NewElement("Active");
			if (this->IsActive())
				pActive->SetText("True");
			else
				pActive->SetText("False");
			pEle->InsertEndChild(pActive);

			Aris::Core::XmlElement *pType = pEle->GetDocument()->NewElement("Type");
			pType->SetText(this->GetType());
			pEle->InsertEndChild(pType);

			Aris::Core::XmlElement *pPrtI = pEle->GetDocument()->NewElement("iPart");
			pPrtI->SetText(_pMakI->Father().Name().data());
			pEle->InsertEndChild(pPrtI);

			Aris::Core::XmlElement *pPrtJ = pEle->GetDocument()->NewElement("jPart");
			pPrtJ->SetText(_pMakJ->Father().Name().data());
			pEle->InsertEndChild(pPrtJ);

			Aris::Core::XmlElement *pMakI = pEle->GetDocument()->NewElement("iMarker");
			pMakI->SetText(_pMakI->Name().data());
			pEle->InsertEndChild(pMakI);

			Aris::Core::XmlElement *pMakJ = pEle->GetDocument()->NewElement("jMarker");
			pMakJ->SetText(_pMakJ->Name().data());
			pEle->InsertEndChild(pMakJ);
		}
		void JointBase::ToAdamsCmd(std::ofstream &file) const
		{
			file << "constraint create joint " << this->GetType() << "  &\r\n"
				<< "    joint_name = ." <<	Model().Name() << "." << this->Name() << "  &\r\n"
				<< "    adams_id = " << this->ID() + 1 << "  &\r\n"
				<< "    i_marker_name = ." << Model().Name() << "." << this->_pMakI->Father().Name() << "." << this->_pMakI->Name() << "  &\r\n"
				<< "    j_marker_name = ." << Model().Name() << "." << this->_pMakJ->Father().Name() << "." << this->_pMakJ->Name() << "  \r\n"
				<< "!\r\n";
		}

		MotionBase::MotionBase(ModelBase *pModel, const std::string &Name, int id, Marker *pMakI, Marker *pMakJ)
			: Element(pModel, Name, id)
			, _pMakI(pMakI)
			, _pMakJ(pMakJ)
		{
			memset(_frc_coe, 0, sizeof(double) * 3);
		}
		MotionBase::MotionBase(ModelBase *pModel, const std::string &Name, int id, const Aris::Core::XmlElement *ele)
			: Element(pModel, Name, id)
		{
			if (strcmp("true", ele->Attribute("Active")) == 0)
			{
				this->Activate();
			}
			else if (strcmp("false", ele->Attribute("Active")) == 0)
			{
				this->Activate(false);
			}
			else
			{
				throw std::logic_error("failed load xml file in motion");
			}

			Matrix m = Model().calculator.CalculateExpression(ele->Attribute("FrcCoe"));
			std::copy_n(m.Data(), 3, _frc_coe);

			_pMakI = Model().GetPart(ele->Attribute("PrtM"))->FindMarker(ele->Attribute("MakI"));
			_pMakJ = Model().GetPart(ele->Attribute("PrtN"))->FindMarker(ele->Attribute("MakJ"));

		}
		void MotionBase::ToXmlElement(Aris::Core::XmlElement *pEle) const
		{
			pEle->DeleteChildren();
			pEle->SetName(this->Name().data());

			Aris::Core::XmlElement *pActive = pEle->GetDocument()->NewElement("Active");
			if (this->IsActive())
				pActive->SetText("True");
			else
				pActive->SetText("False");
			pEle->InsertEndChild(pActive);

			Aris::Core::XmlElement *pType = pEle->GetDocument()->NewElement("Type");
			pType->SetText(this->GetType());
			pEle->InsertEndChild(pType);

			Aris::Core::XmlElement *pPrtI = pEle->GetDocument()->NewElement("iPart");
			pPrtI->SetText(_pMakI->Father().Name().data());
			pEle->InsertEndChild(pPrtI);

			Aris::Core::XmlElement *pPrtJ = pEle->GetDocument()->NewElement("jPart");
			pPrtJ->SetText(_pMakJ->Father().Name().data());
			pEle->InsertEndChild(pPrtJ);

			Aris::Core::XmlElement *pMakI = pEle->GetDocument()->NewElement("iMarker");
			pMakI->SetText(_pMakI->Name().data());
			pEle->InsertEndChild(pMakI);

			Aris::Core::XmlElement *pMakJ = pEle->GetDocument()->NewElement("jMarker");
			pMakJ->SetText(_pMakJ->Name().data());
			pEle->InsertEndChild(pMakJ);

			Aris::Core::XmlElement *pFrictionCoefficients = pEle->GetDocument()->NewElement("Friction_Coefficients");

			pFrictionCoefficients->SetText(Matrix(1, 3, _frc_coe).ToString().c_str());
			pEle->InsertEndChild(pFrictionCoefficients);
		}
		void MotionBase::ToAdamsCmd(std::ofstream &file) const
		{
			std::string s;
			s = "z";

			if (this->posCurve == nullptr)
			{
				file << "constraint create motion_generator &\r\n"
					<< "    motion_name = ." << Model().Name() << "." << this->Name() << "  &\r\n"
					<< "    adams_id = " << this->ID() + 1 << "  &\r\n"
					<< "    i_marker_name = ." << Model().Name() << "." << this->_pMakI->Father().Name() << "." << this->_pMakI->Name() << "  &\r\n"
					<< "    j_marker_name = ." << Model().Name() << "." << this->_pMakJ->Father().Name() << "." << this->_pMakJ->Name() << "  &\r\n"
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
					<< "    i_marker_name = ." << Model().Name() << "." << this->_pMakI->Father().Name() << "." << this->_pMakI->Name() << "  &\r\n"
					<< "    j_marker_name = ." << Model().Name() << "." << this->_pMakJ->Father().Name() << "." << this->_pMakJ->Name() << "  &\r\n"
					<< "    axis = " << s << "  &\r\n"
					<< "    function = \"AKISPL(time,0," << this->Name() << "_pos_spl)\"  \r\n"
					<< "!\r\n";
			}
		}

		ForceBase::ForceBase(ModelBase *pModel, const std::string &Name, int id, Marker *pMakI, Marker *pMakJ)
			: Element(pModel, Name, id)
			, _pMakI(pMakI)
			, _pMakJ(pMakJ)
		{
		}
		ForceBase::ForceBase(ModelBase *pModel, const std::string &Name, int id, const Aris::Core::XmlElement *xmlEle)
			: Element(pModel, Name, id)
			, _pMakI(pModel->GetPart(xmlEle->Attribute("PrtM"))->FindMarker(xmlEle->Attribute("MakI")))
			, _pMakJ(pModel->GetPart(xmlEle->Attribute("PrtN"))->FindMarker(xmlEle->Attribute("MakJ")))
		{

		}

		Environment::Environment(ModelBase *pModel)
			:Object(pModel,"Environment")
		{
		}
		Environment::~Environment()
		{
		}
		void Environment::ToXmlElement(Aris::Core::XmlElement *pEle) const
		{
			pEle->DeleteChildren();
			pEle->SetName("Enviroment");

			Aris::Core::XmlElement *pGravity = pEle->GetDocument()->NewElement("Gravity");
			pGravity->SetText(Matrix(1, 6, Gravity).ToString().c_str());
			pEle->InsertEndChild(pGravity);
		}
		void Environment::FromXmlElement(const Aris::Core::XmlElement *pEle)
		{
			Matrix m = Model().calculator.CalculateExpression(pEle->FirstChildElement("Gravity")->GetText());
			memcpy(Gravity, m.Data(), sizeof(Gravity));
		}
		void Environment::ToAdamsCmd(std::ofstream &file) const
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
				<< "   model_name = " << this->Model().Name() << "\r\n"
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

		ModelBase::ModelBase(const std::string & Name)
			: Object(this , Name)
			, _Environment(this)
			, pGround(nullptr)
		{
			AddPart("Ground");
			pGround = GetPart("Ground");
		}
		ModelBase::~ModelBase()
		{
		}

		template<class T>
		typename T::value_type::element_type * GetContent(const T &container, const std::string &Name)
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

		const Part *ModelBase::GetPart(int id) const
		{
			return _parts.at(id).get();
		}
		const JointBase *ModelBase::GetJoint(int id)const
		{
			return _joints.at(id).get();
		}
		const MotionBase *ModelBase::GetMotion(int id)const
		{
			return _motions.at(id).get();
		}
		const ForceBase *ModelBase::GetForce(int id)const
		{
			return _forces.at(id).get();
		}
		const Marker *ModelBase::GetMarker(int id)const
		{
			return _markers.at(id).get();
		}
		Part *ModelBase::GetPart(int id)
		{
			return _parts.at(id).get();
		}
		JointBase *ModelBase::GetJoint(int id)
		{
			return _joints.at(id).get();
		}
		MotionBase *ModelBase::GetMotion(int id)
		{
			return _motions.at(id).get();
		}
		ForceBase *ModelBase::GetForce(int id)
		{
			return _forces.at(id).get();
		}
		Marker *ModelBase::GetMarker(int id)
		{
			return _markers.at(id).get();
		}
		const Part *ModelBase::GetPart(const std::string &Name)const
		{
			return GetContent<decltype(_parts)>(_parts, Name);
		}
		const JointBase *ModelBase::GetJoint(const std::string &Name)const
		{
			return GetContent<decltype(_joints)>(_joints, Name);
		}
		const MotionBase *ModelBase::GetMotion(const std::string &Name)const
		{
			return GetContent<decltype(_motions)>(_motions, Name);
		}
		const ForceBase *ModelBase::GetForce(const std::string &Name)const
		{
			return GetContent<decltype(_forces)>(_forces, Name);
		}
		Part *ModelBase::GetPart(const std::string &Name)
		{
			return GetContent<decltype(_parts)>(_parts, Name);
		}
		JointBase *ModelBase::GetJoint(const std::string &Name)
		{
			return GetContent<decltype(_joints)>(_joints, Name);
		}
		MotionBase *ModelBase::GetMotion(const std::string &Name)
		{
			return GetContent<decltype(_motions)>(_motions, Name);
		}
		ForceBase *ModelBase::GetForce(const std::string &Name)
		{
			return GetContent<decltype(_forces)>(_forces, Name);
		}

		void ModelBase::DynPre(int *pI_dim, int *pC_dim)
		{
			int pid = 0;//part id
			int cid = 6;//Constraint id

			for (auto &part:_parts)
			{
				if (part->IsActive())
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
				if (joint->IsActive())
				{
					joint->Init();
					joint->_ColId = cid;
					cid += joint->CstDim();
				}
				else
				{
					joint->_ColId = 0;
				}
			}
			for (auto &motion:_motions)
			{
				if (motion->IsActive())
				{
					motion->_ColId = cid;
					cid++;
					motion->Init();
				}
				else
				{
					motion->_ColId = 0;
					motion->Init();
				}
			}

			I_dim = pid;
			C_dim = cid;

			if (pI_dim) *pI_dim = pid;
			if (pC_dim) *pC_dim = cid;
		}
		void ModelBase::DynMtx(double *C, double*a_c, double *I_mat, double*f, double *D, double *b)
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
				if (prt->IsActive())
				{
					prt->Update();
					s_block_cpy(6, 6, *(prt->_PrtIm), 0, 0, 6, I_mat, prt->_RowId, prt->_RowId, I_dim);

					s_daxpy(6, -1, prt->_PrtFg, 1, &f[prt->_RowId], 1);
					s_daxpy(6, 1, prt->_PrtFv, 1, &f[prt->_RowId], 1);
				}
			}
			for (auto &jnt : _joints)
			{
				if (jnt->IsActive())
				{
					jnt->Update();

					s_block_cpy(6, jnt->CstDim(), jnt->PrtCstMtxI(), 0, 0, jnt->CstDim(), C, jnt->_pMakI->Father()._RowId, jnt->_ColId, C_dim);
					s_block_cpy(6, jnt->CstDim(), jnt->PrtCstMtxJ(), 0, 0, jnt->CstDim(), C, jnt->_pMakJ->Father()._RowId, jnt->_ColId, C_dim);

					std::copy_n(jnt->PrtA_c(), jnt->CstDim(), &a_c[jnt->_ColId]);
				}
			}
			for (auto &mot : _motions)
			{
				if (mot->IsActive())
				{
					mot->Update();

					s_block_cpy(6, 1, mot->PrtCstMtxI(), 0, 0, 1, C, mot->_pMakI->Father()._RowId, mot->_ColId, C_dim);
					s_block_cpy(6, 1, mot->PrtCstMtxJ(), 0, 0, 1, C, mot->_pMakJ->Father()._RowId, mot->_ColId, C_dim);

					a_c[mot->_ColId] = *mot->PrtA_c();
				}
			}
			for (auto &fce : _forces)
			{
				if (fce->IsActive())
				{
					fce->Update();

					s_daxpy(6, -1, fce->GetPrtFceIPtr(), 1, &f[fce->_pMakI->Father()._RowId], 1);
					s_daxpy(6, -1, fce->GetPrtFceJPtr(), 1, &f[fce->_pMakJ->Father()._RowId], 1);
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
		void ModelBase::DynEnd(const double *x)
		{
			for (auto &prt : _parts)
			{
				if (prt->IsActive())
				{
					std::copy_n(&x[prt->_RowId], 6, prt->_PrtAcc);
				}
			}
			for (auto &jnt : _joints)
			{
				if (jnt->IsActive())
				{
					std::copy_n(&x[jnt->_ColId + I_dim], jnt->CstDim(), jnt->CstFce());
				}
			}
			for (auto &mot : _motions)
			{
				if (mot->IsActive())
				{
					mot->motDynFce = x[mot->_ColId + I_dim];
				}
			}
		}
		void ModelBase::DynUkn(double *a, double*f_c)
		{
			for (auto &prt : _parts)
			{
				if (prt->IsActive())
				{
					prt->Update();
					std::copy_n(prt->PrtAcc(), 6, &a[prt->_RowId]);
				}
			}
			for (auto &jnt : _joints)
			{
				if (jnt->IsActive())
				{
					std::copy_n(jnt->CstFce(), jnt->CstDim(), &f_c[jnt->_ColId]);
				}
			}
			for (auto &mot : _motions)
			{
				if (mot->IsActive())
				{
					f_c[mot->_ColId] = mot->MotFceDyn();
				}
			}
		}
		void ModelBase::Dyn(std::function<void(int dim, const double *D, const double *b, double *x)> solveMethod)
		{
			DynPre();
			
			std::vector<double> C(I_dim * C_dim);
			std::vector<double> a_c(C_dim);
			std::vector<double> I_mat(I_dim * I_dim);
			std::vector<double> f(I_dim);
			std::vector<double> D((I_dim + C_dim) * (I_dim + C_dim));
			std::vector<double> b(I_dim + C_dim);

			DynMtx(C.data(), a_c.data(), I_mat.data(), f.data(), D.data(), b.data());

			std::vector<double> x(I_dim + C_dim);
			solveMethod(I_dim + C_dim, D.data(), b.data(), x.data());

			DynEnd(x.data());
		}

		void ModelBase::ClbPre(int &clb_dim_m, int &clb_dim_n, int &gamma_dim, int &frc_coe_dim)
		{
			DynPre(&clb_dim_m, &clb_dim_n);

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
				if (i->IsActive())
				{
					clb_dim_m++;
				}
				frc_coe_dim += 3;
				clb_dim_n += 3;
			}
			for (auto &i : _parts)
			{
				if (i->IsActive())
				{
					clb_dim_n += 10;
					gamma_dim += 10;
				}
			}

		}
		void ModelBase::ClbMtx(double *clb_d_ptr, double *clb_b_ptr, std::function<void(int n, double *A)> inverseMethod)
		{
			int clb_dim_m, clb_dim_n, gamma_dim, frc_coe_dim;
			ClbPre(clb_dim_m, clb_dim_n, gamma_dim, frc_coe_dim);

			int dim = I_dim;

			/*初始化*/
			static Matrix _clb_d, _clb_b;

			_clb_d.Resize(clb_dim_m, clb_dim_n);
			_clb_b.Resize(clb_dim_m, 1);

			memset(_clb_d.Data(), 0, _clb_d.Length() * sizeof(double));
			memset(_clb_b.Data(), 0, _clb_b.Length() * sizeof(double));

			/*求A，即C的逆*/
			Matrix A(dim, dim), B(dim, dim);
			std::vector<int> ipiv(dim);
			
			std::vector<double> C(dim * dim);
			std::vector<double> f(dim);
			
			for (int i = 0; i < 6; ++i)
			{
				C.data()[C_dim*(pGround->_RowId + i) + i] = 1;
			}
			for (auto &jnt : _joints)
			{
				if (jnt->IsActive())
				{
					jnt->Update();

					s_block_cpy(6, jnt->CstDim(), jnt->PrtCstMtxI(), 0, 0, jnt->CstDim(), C.data(), jnt->_pMakI->Father()._RowId, jnt->_ColId, C_dim);
					s_block_cpy(6, jnt->CstDim(), jnt->PrtCstMtxJ(), 0, 0, jnt->CstDim(), C.data(), jnt->_pMakJ->Father()._RowId, jnt->_ColId, C_dim);
				}
			}
			for (auto &mot : _motions)
			{
				if (mot->IsActive())
				{
					mot->Update();

					s_block_cpy(6, 1, mot->PrtCstMtxI(), 0, 0, 1, C.data(), mot->_pMakI->Father()._RowId, mot->_ColId, C_dim);
					s_block_cpy(6, 1, mot->PrtCstMtxJ(), 0, 0, 1, C.data(), mot->_pMakJ->Father()._RowId, mot->_ColId, C_dim);
				}
			}

			std::copy(C.begin(), C.end(), A.Data());
			inverseMethod(dim, A.Data());

			/*求B*/
			const int beginRow = dim - clb_dim_m;

			for (auto &i:_parts)
			{
				if (i->IsActive())
				{
					double cm[6][6];
					s_cmf(i->PrtVel(), *cm);
					s_dgemm(clb_dim_m, 6, 6, 1, &A(beginRow,i->_RowId), dim, *cm, 6, 0, &B(beginRow, i->_RowId), dim);
				}
			}

			/*求解clb_d*/
			int col1 = 0, col2 = 0;

			for (auto &i:_parts)
			{
				if (i->IsActive())
				{
					double q[6]{0};
					std::copy_n(i->PrtAcc(), 6, q);
					s_daxpy(6, -1, i->PrtGravity(), 1, q, 1);
					
					double v[6];
					std::copy_n(i->PrtVel(), 6, v);

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
				if (mot->IsActive())
				{
					_clb_b(row, 0) = mot->MotFce();
					row++;
				}
				else
				{
					mot->Update();

					s_daxpy(6, mot->MotFce(), mot->PrtCstMtxI(), 1, &f[mot->_pMakI->Father()._RowId], 1);
					s_daxpy(6, mot->MotFce(), mot->PrtCstMtxJ(), 1, &f[mot->_pMakJ->Father()._RowId], 1);
				}
			}

			s_dgemm(clb_dim_m, 1, dim, 1, &A(beginRow,0), dim, f.data(), 1, 1, _clb_b.Data(), 1);

			/*以下添加驱动摩擦系数*/
			row = 0;
			int num = 0;
			for (auto &mot : _motions)
			{
				//默认未激活的motion处于力控模式
				if (mot->IsActive())
				{
					_clb_d(row, gamma_dim + num * 3) += s_sgn(mot->MotVel());
					_clb_d(row, gamma_dim + num * 3 + 1) += mot->MotVel();
					_clb_d(row, gamma_dim + num * 3 + 2) += mot->MotAcc();
					++row;
				}
				else
				{
					s_dgemm(clb_dim_m, 1, 6, s_sgn(mot->MotVel()), &A(beginRow, mot->_pMakI->Father()._RowId), dim, mot->PrtCstMtxI(), 1, 1, &_clb_d(0, gamma_dim + num * 3), clb_dim_n);
					s_dgemm(clb_dim_m, 1, 6, s_sgn(mot->MotVel()), &A(beginRow, mot->_pMakJ->Father()._RowId), dim, mot->PrtCstMtxJ(), 1, 1, &_clb_d(0, gamma_dim + num * 3), clb_dim_n);
					s_dgemm(clb_dim_m, 1, 6, mot->MotVel(), &A(beginRow, mot->_pMakI->Father()._RowId), dim, mot->PrtCstMtxI(), 1, 1, &_clb_d(0, gamma_dim + num * 3 + 1), clb_dim_n);
					s_dgemm(clb_dim_m, 1, 6, mot->MotVel(), &A(beginRow, mot->_pMakJ->Father()._RowId), dim, mot->PrtCstMtxJ(), 1, 1, &_clb_d(0, gamma_dim + num * 3 + 1), clb_dim_n);
					s_dgemm(clb_dim_m, 1, 6, mot->MotAcc(), &A(beginRow, mot->_pMakI->Father()._RowId), dim, mot->PrtCstMtxI(), 1, 1, &_clb_d(0, gamma_dim + num * 3 + 2), clb_dim_n);
					s_dgemm(clb_dim_m, 1, 6, mot->MotAcc(), &A(beginRow, mot->_pMakJ->Father()._RowId), dim, mot->PrtCstMtxJ(), 1, 1, &_clb_d(0, gamma_dim + num * 3 + 2), clb_dim_n);

					//s_dgemm(clb_dim_m, 1, 6, s_sgn(*i->GetV_mPtr()), &A(beginRow, i->_pMakI->Father()._RowId), dim, i->_PrtCstMtxI[0], 6, 1, &_clb_d(0, clb_prt_dim_n + num * 3), clb_dim_n);
					//s_dgemm(clb_dim_m, 1, 6, s_sgn(*i->GetV_mPtr()), &A(beginRow, i->_pMakJ->Father()._RowId), dim, i->_PrtCstMtxJ[0], 6, 1, &_clb_d(0, clb_prt_dim_n + num * 3), clb_dim_n);
					//s_dgemm(clb_dim_m, 1, 6, *i->GetV_mPtr(), &A(beginRow, i->_pMakI->Father()._RowId), dim, i->_PrtCstMtxI[0], 6, 1, &_clb_d(0, clb_prt_dim_n + num * 3 + 1), clb_dim_n);
					//s_dgemm(clb_dim_m, 1, 6, *i->GetV_mPtr(), &A(beginRow, i->_pMakJ->Father()._RowId), dim, i->_PrtCstMtxJ[0], 6, 1, &_clb_d(0, clb_prt_dim_n + num * 3 + 1), clb_dim_n);
					//s_dgemm(clb_dim_m, 1, 6, *i->GetA_mPtr(), &A(beginRow, i->_pMakI->Father()._RowId), dim, i->_PrtCstMtxI[0], 6, 1, &_clb_d(0, clb_prt_dim_n + num * 3 + 2), clb_dim_n);
					//s_dgemm(clb_dim_m, 1, 6, *i->GetA_mPtr(), &A(beginRow, i->_pMakJ->Father()._RowId), dim, i->_PrtCstMtxJ[0], 6, 1, &_clb_d(0, clb_prt_dim_n + num * 3 + 2), clb_dim_n);

				}

				num++;
			}

			std::copy_n(_clb_d.Data(), _clb_d.Length(), clb_d_ptr);
			std::copy_n(_clb_b.Data(), _clb_b.Length(), clb_b_ptr);
		}
		void ModelBase::ClbUkn(double *clb_gamma_and_frcCoe_ptr)
		{
			int clb_dim_m, clb_dim_n, gamma_dim, frc_coe_dim;
			ClbPre(clb_dim_m, clb_dim_n, gamma_dim, frc_coe_dim);
			
			int row = 0;
			for (auto &prt : _parts)
			{
				if (prt->IsActive())
				{
					s_im2gamma(*prt->PrtIm(), clb_gamma_and_frcCoe_ptr + row);
				}
				row += 10;
			}

			for (auto &mot : _motions)
			{
				std::copy_n(mot->GetFrcCoePtr(),3, clb_gamma_and_frcCoe_ptr + row);
				row += 3;
			}
		}
		
		void ModelBase::LoadXml(const char *filename)
		{
			Aris::Core::XmlDocument xmlDoc;
			
			if (xmlDoc.LoadFile(filename) != 0)
			{
				throw std::logic_error((std::string("could not open file:") + std::string(filename)));
			}

			LoadXml(xmlDoc);
		}
		void ModelBase::LoadXml(const Aris::Core::XmlDocument &xmlDoc)
		{
			const Aris::Core::XmlElement *pModel = xmlDoc.RootElement()->FirstChildElement("Model");

			LoadXml(pModel);
		}
		void ModelBase::LoadXml(const Aris::Core::XmlElement *pModel)
		{
			if (pModel == nullptr)throw(std::logic_error("XML file must have model element"));

			const Aris::Core::XmlElement *pVar = pModel->FirstChildElement("Variable");
			if (pModel == nullptr)throw(std::logic_error("Model must have variable element"));
			const Aris::Core::XmlElement *pEnv = pModel->FirstChildElement("Environment");
			if (pEnv == nullptr)throw(std::logic_error("Model must have environment element"));
			const Aris::Core::XmlElement *pPrt = pModel->FirstChildElement("Part");
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
		void ModelBase::SaveSnapshotXml(const char *filename) const
		{
			Aris::Core::XmlDocument XML_Doc;
			XML_Doc.DeleteChildren();

			Aris::Core::XmlDeclaration *pHeader = XML_Doc.NewDeclaration("xml version=\"1.0\" encoding=\"UTF-8\" ");
			XML_Doc.InsertFirstChild(pHeader);

			Aris::Core::XmlElement *pModel = XML_Doc.NewElement("Model");
			XML_Doc.InsertEndChild(pModel);

			Aris::Core::XmlElement *pEnvironment = XML_Doc.NewElement("");
			_Environment.ToXmlElement(pEnvironment);
			pModel->InsertEndChild(pEnvironment);

			Aris::Core::XmlElement *pVar = XML_Doc.NewElement("Variable");
			pModel->InsertEndChild(pVar);

			Aris::Core::XmlElement *pPrt = XML_Doc.NewElement("Part");
			pModel->InsertEndChild(pPrt);

			Aris::Core::XmlElement *pJnt = XML_Doc.NewElement("Joint");
			pModel->InsertEndChild(pJnt);

			Aris::Core::XmlElement *pMot = XML_Doc.NewElement("Motion");
			pModel->InsertEndChild(pMot);

			Aris::Core::XmlElement *pFce = XML_Doc.NewElement("Force");
			pModel->InsertEndChild(pFce);

			for (auto &p:_parts)
			{
				Aris::Core::XmlElement *ele = XML_Doc.NewElement("");
				p->ToXmlElement(ele);
				pPrt->InsertEndChild(ele);
			}

			for (auto &j : _joints)
			{
				Aris::Core::XmlElement *ele = XML_Doc.NewElement("");
				j->ToXmlElement(ele);
				pJnt->InsertEndChild(ele);
			}

			for (auto &m : _motions)
			{
				Aris::Core::XmlElement *ele = XML_Doc.NewElement("");
				m->ToXmlElement(ele);
				pMot->InsertEndChild(ele);
			}


			XML_Doc.SaveFile(filename);
		}
		void ModelBase::SaveAdams(const char *filename, const SimulateScript* pScript) const
		{
			std::ofstream file;

			std::string cmdName = std::string(filename) + std::string(".cmd");
			std::string acfName = std::string(filename) + std::string(".acf");
			std::string fullAcfPath;
			/*******写acf文件********/
			if (pScript)
			{
#ifdef PLATFORM_IS_WINDOWS
				const int maxPath = 2000;
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

				file << std::setprecision(15);

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
						if (auto ele = dynamic_cast<JointBase *>(j.first))
						{
							if (j.second)
							{
								file << "activate/joint, id=" << ele->ID() + 1 << "\r\n";
							}
							else
							{
								file << "deactivate/joint, id=" << ele->ID() + 1 << "\r\n";
							}
						}
						if (auto ele = dynamic_cast<MotionBase *>(j.first))
						{
							if (j.second)
							{
								file << "activate/motion, id=" << ele->ID() + 1 << "\r\n";
							}
							else
							{
								file << "deactivate/motion, id=" << ele->ID() + 1 << "\r\n";
							}
						}
						if (auto ele = dynamic_cast<ForceBase *>(j.first))
						{
							if (j.second)
							{
								file << "activate/sforce, id=" << ele->ID() + 1 << "\r\n";
							}
							else
							{
								file << "deactivate/sforce, id=" << ele->ID() + 1 << "\r\n";
							}
						}

						
						
					}

					for (auto &j : i.second.markers)
					{
						auto mak = j.first;
						auto pe = j.second.data();
						
						file << "marker/" << mak->ID() + 1
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

			file << std::setprecision(15);

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
					if (prt->IsActive() || (pScript != nullptr))
						prt->ToAdamsCmd(file);
				}
			}

			file << "!----------------------------------- Joints -----------------------------------!\r\n"
				<< "!\r\n"
				<< "!\r\n";

			for (auto &jnt : _joints)
			{
				if ((jnt->IsActive()) || (pScript!=nullptr))
				{
					jnt->ToAdamsCmd(file);
				}
			}

			file << "!----------------------------------- Motions -----------------------------------!\r\n"
				<< "!\r\n"
				<< "!\r\n";

			for (auto &mot : _motions)
			{
				if (mot->IsActive() || (pScript != nullptr))
				{
					mot->ToAdamsCmd(file);
				}
			}

			file << "!----------------------------------- Forces -----------------------------------!\r\n"
				<< "!\r\n"
				<< "!\r\n";

			for (auto &fce : _forces)
			{
				if (fce->IsActive() || (pScript != nullptr))
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
		void ModelBase::SaveAdams(const char *filename, bool isModifyActive) const
		{
			std::ofstream file;

			std::string cmdName = std::string(filename) + std::string(".cmd");

			/*******写cmd文件********/
			file.open(cmdName, std::ios::out | std::ios::trunc);

			file << std::setprecision(15);

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
					if ((prt.get() != pGround) && (!prt->IsActive()))
					{
						file << "part attributes  &\r\n"
							<< "    constraint_name = ." << Name() << "." << prt->Name() << "  &\r\n"
							<< "    active = off \r\n!\r\n";
					}
				}
				for (auto &jnt : _joints)
				{
					if (!jnt->IsActive())
					{
						file << "constraint attributes  &\r\n"
							<< "    constraint_name = ." << Name() << "." << jnt->Name() << "  &\r\n"
							<< "    active = off \r\n!\r\n";
					}
					
				}
				for (auto &mot : _motions)
				{
					if (!mot->IsActive())
					{
						file << "constraint attributes  &\r\n"
							<< "    constraint_name = ." << Name() << "." << mot->Name() << "  &\r\n"
							<< "    active = off \r\n!\r\n";
					}
					
				}
				for (auto &fce : _forces)
				{
					if (!fce->IsActive())
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
