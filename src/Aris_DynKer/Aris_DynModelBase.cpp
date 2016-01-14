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
		Interaction::Interaction(ModelBase &model, const std::string &name, int id, const Aris::Core::XmlElement *ele)
			: Element(model, name, id)
			, makI_(*Model().FindPart(ele->Attribute("PrtM"))->FindMarker(ele->Attribute("MakI")))
			, makJ_(*Model().FindPart(ele->Attribute("PrtN"))->FindMarker(ele->Attribute("MakJ"))) 
		{
		}
		void Constraint::Update()
		{
			double pm_M2N[4][4];
			double _tem_v1[6]{ 0 }, _tem_v2[6]{ 0 };

			/* Get pm M2N */
			s_pm_dot_pm(*MakJ().Father().InvPm(), *MakI().Father().Pm(), *pm_M2N);

			/*update CstMtx*/
			std::fill_n(this->CsmJ(), this->CstDim() * 6, 0);
			s_tf_n(CstDim(), -1, *pm_M2N, this->CsmI(), 0, this->CsmJ());

			/*update CstAcc*/
			std::fill_n(this->Csa(), this->CstDim(), 0);
			s_inv_tv(-1, *pm_M2N, MakJ().Father().PrtVel(), 0, _tem_v1);
			s_cv(MakI().Father().PrtVel(), _tem_v1, _tem_v2);
			s_dgemmTN(CstDim(), 1, 6, 1, CsmI(), CstDim(), _tem_v2, 1, 0, Csa(), 1);
		}
		void Constraint::ToXmlElement(Aris::Core::XmlElement *pEle) const
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
			pType->SetText(this->AdamsType());
			pEle->InsertEndChild(pType);

			Aris::Core::XmlElement *pPrtI = pEle->GetDocument()->NewElement("iPart");
			pPrtI->SetText(MakI().Father().Name().data());
			pEle->InsertEndChild(pPrtI);

			Aris::Core::XmlElement *pPrtJ = pEle->GetDocument()->NewElement("jPart");
			pPrtJ->SetText(MakJ().Father().Name().data());
			pEle->InsertEndChild(pPrtJ);

			Aris::Core::XmlElement *pMakI = pEle->GetDocument()->NewElement("iMarker");
			pMakI->SetText(MakI().Name().data());
			pEle->InsertEndChild(pMakI);

			Aris::Core::XmlElement *pMakJ = pEle->GetDocument()->NewElement("jMarker");
			pMakJ->SetText(MakJ().Name().data());
			pEle->InsertEndChild(pMakJ);
		}
		void Constraint::ToAdamsCmd(std::ofstream &file) const
		{
			file << "constraint create joint " << this->AdamsType() << "  &\r\n"
				<< "    joint_name = ." << Model().Name() << "." << this->Name() << "  &\r\n"
				<< "    adams_id = " << this->ID() + 1 << "  &\r\n"
				<< "    i_marker_name = ." << Model().Name() << "." << this->MakI().Father().Name() << "." << this->MakI().Name() << "  &\r\n"
				<< "    j_marker_name = ." << Model().Name() << "." << this->MakJ().Father().Name() << "." << this->MakJ().Name() << "  \r\n"
				<< "!\r\n";
		}

		Marker::Marker(const Part &prt, const double *prt_pe, const char* eulType)
			: Element(const_cast<ModelBase &>(prt.Model()), "", 0), prt_(prt)
		{
			static const double defaultPe[6] = { 0 };
			prt_pe = prt_pe ? prt_pe : defaultPe;

			s_pe2pm(prt_pe, *prt_pm_, eulType);
		}
		Marker::Marker(ModelBase &model, Part &prt, const std::string &name, int id)
			: Element(model, name, id), prt_(prt)
		{
			std::fill_n(static_cast<double *>(*prt_pm_), 16, 0);
			prt_pm_[0][0] = 1;
			prt_pm_[1][1] = 1;
			prt_pm_[2][2] = 1;
			prt_pm_[3][3] = 1;

			std::fill_n(static_cast<double *>(*this->pm_), 16, 0);
			this->pm_[0][0] = 1;
			this->pm_[1][1] = 1;
			this->pm_[2][2] = 1;
			this->pm_[3][3] = 1;
		}
		Marker::Marker(Part &prt, const std::string &name, int id, const double *pPrtPm, Marker *pRelativeTo)
			: Marker(prt.Model(), prt, name, id)
		{
			static const double default_pm_in[16] = { 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
			pPrtPm = pPrtPm ? pPrtPm : default_pm_in;
			
			
			if (pRelativeTo)
			{
				if (&pRelativeTo->Father() != &prt_)
					throw std::logic_error("relative marker must has same father part with this marker");
				
				s_pm_dot_pm(*pRelativeTo->PrtPm(), pPrtPm, *prt_pm_);
			}
			else
			{
				std::copy_n(pPrtPm, 16, static_cast<double *>(*prt_pm_));
			}
		}
		Marker::Marker(Part &prt, const std::string &Name, int id, const Aris::Core::XmlElement *ele)
			: Marker(prt.Model(), prt, Name, id)
		{
			double pm[16];

			Matrix m = prt.Model().calculator.CalculateExpression(ele->Attribute("Pos"));
			s_pe2pm(m.Data(), pm);

			if (ele->Attribute("RelativeTo") && (!ele->Attribute("RelativeTo", "")))
			{
				const Marker *pRelativeMak = Father().FindMarker(ele->Attribute("RelativeTo"));
				s_pm_dot_pm(*pRelativeMak->PrtPm(), pm, *prt_pm_);
			}
			else
			{
				std::copy_n(pm, 16, static_cast<double*>(*prt_pm_));
			}
		}
		const double6& Marker::Vel() const { return Father().Vel(); };
		const double6& Marker::Acc() const { return Father().Acc(); };
		void Marker::Update()
		{
			s_pm_dot_pm(*Father().Pm(), *PrtPm(), *pm_);
		}
		void Marker::ToXmlElement(Aris::Core::XmlElement *pEle) const
		{
			double value[10];

			pEle->DeleteChildren();
			pEle->SetName(this->Name().data());

			Aris::Core::XmlElement *pPE = pEle->GetDocument()->NewElement("Pos");
			s_pm2pe(*PrtPm(), value);
			pPE->SetText(Matrix(1, 6, value).ToString().c_str());
			pEle->InsertEndChild(pPE);

			Aris::Core::XmlElement *pRelativeMakEle = pEle->GetDocument()->NewElement("RelativeTo");
			pRelativeMakEle->SetText("");
			pEle->InsertEndChild(pRelativeMakEle);
		}
		
		Part::Part(ModelBase &model, const std::string &Name, int id, const double *im, const double *pm, const double *vel, const double *acc)
			: Marker(model, *this, Name, id)
		{
			if (im == nullptr)
			{
				std::fill_n(static_cast<double *>(*prt_im_), 36, 0);
				prt_im_[0][0] = 1;
				prt_im_[1][1] = 1;
				prt_im_[2][2] = 1;
				prt_im_[3][3] = 1;
				prt_im_[4][4] = 1;
				prt_im_[5][5] = 1;
			}
			else
			{
				std::copy_n(im, 36, *prt_im_);
			}

			if (pm == nullptr)
			{
				std::fill_n(static_cast<double *>(*this->Pm()), 16, 0);
				this->pm_[0][0] = 1;
				this->pm_[1][1] = 1;
				this->pm_[2][2] = 1;
				this->pm_[3][3] = 1;
			}
			else
			{
				SetPm(pm);
			}

			if (vel == nullptr)
			{
				std::fill_n(Vel(), 6, 0);
			}
			else
			{
				SetVel(vel);
			}

			if (acc == nullptr)
			{
				std::fill_n(Acc(), 6, 0);
			}
			else
			{
				SetAcc(acc);
			}
		}
		Part::Part(ModelBase &model, const std::string &Name, int id, const Aris::Core::XmlElement *ele)
			: Marker(model, *this, Name, id)
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
			s_gamma2im(m.Data(), *prt_im_);

			m = Model().calculator.CalculateExpression(ele->Attribute("Pos"));
			s_pe2pm(m.Data(), *Pm());

			m = Model().calculator.CalculateExpression(ele->Attribute("Vel"));
			std::copy_n(m.Data(), 6, Vel());

			m = Model().calculator.CalculateExpression(ele->Attribute("Acc"));
			std::copy_n(m.Data(), 6, Acc());

			marker_names_.clear();

			for (auto makEle = ele->FirstChildElement("ChildMarker")->FirstChildElement(); makEle != nullptr; makEle = makEle->NextSiblingElement())
			{
				AddMarker(makEle->Name(), makEle);
			}

			if (ele->Attribute("Graphic_File_Path") && (!ele->Attribute("Graphic_File_Path", "")))
				graphic_file_path_ = ele->Attribute("Graphic_File_Path");
		}
		void Part::Update()
		{
			double tem[6];
		
			s_inv_pm(*Pm(), *inv_pm_);
			s_tv(*inv_pm_, Vel(), prt_vel_);
			s_tv(*inv_pm_, Acc(), prt_acc_);
			s_tv(*inv_pm_, Model().Environment().Gravity, prt_gravity_);
			s_m6_dot_v6(*prt_im_, prt_gravity_, prt_fg_);
			s_m6_dot_v6(*prt_im_, prt_vel_, tem);
			s_cf(prt_vel_, tem, prt_fv_);
		}
		Marker* Part::FindMarker(const std::string &Name)
		{
			auto pMak = marker_names_.find(Name);
			if (pMak != marker_names_.end())
			{
				return Model().markers_.at(pMak->second).get();
			}
			else
			{
				return nullptr;
			}
		}
		const Marker* Part::FindMarker(const std::string &Name)const
		{
			auto pMak = marker_names_.find(Name);
			if (pMak != marker_names_.end())
			{
				return Model().markers_.at(pMak->second).get();
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
			pVel->SetText(Matrix(1, 6, Vel()).ToString().c_str());
			pEle->InsertEndChild(pVel);

			Aris::Core::XmlElement *pAcc = pEle->GetDocument()->NewElement("Acc");
			pAcc->SetText(Matrix(1, 6, Acc()).ToString().c_str());
			pEle->InsertEndChild(pAcc);

			Aris::Core::XmlElement *pChildMak = pEle->GetDocument()->NewElement("ChildMarker");
			pEle->InsertEndChild(pChildMak);

			for (auto &m : marker_names_)
			{
				Aris::Core::XmlElement *ele = pEle->GetDocument()->NewElement("");

				Model().markers_.at(m.second)->ToXmlElement(ele);
				pChildMak->InsertEndChild(ele);
			}

			Aris::Core::XmlElement *pGraphicFilePath = pEle->GetDocument()->NewElement("Graphic_File_Path");
			pGraphicFilePath->SetText(this->graphic_file_path_.c_str());
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
					<< "    adams_id = " << this->ID() + Model().MarkerNum() << "  &\r\n"
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
			for (auto &mak : this->marker_names_)
			{
				double pe[6];

				s_pm2pe(*Model().MarkerAt(mak.second).PrtPm(), pe, "313");
				Matrix ori(1, 3, &pe[3]), loc(1, 3, &pe[0]);

				file << "marker create  &\r\n"
					<< "marker_name = ." << Model().Name() << "." << this->Name() << "." << mak.first << "  &\r\n"
					<< "adams_id = " << mak.second + 1 << "  &\r\n"
					<< "location = (" << loc.ToString() << ")  &\r\n"
					<< "orientation = (" << ori.ToString() << ")\r\n"
					<< "!\r\n";
			}
			//导入parasolid
			std::stringstream stream(this->graphic_file_path_);
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

		MotionBase::MotionBase(ModelBase &model, const std::string &name, int id, Marker &makI, Marker &makJ)
			: Constraint(model, name, id, makI, makJ)
		{
			std::fill_n(static_cast<double*>(frc_coe_), 3, 0);
		}
		MotionBase::MotionBase(ModelBase &model, const std::string &name, int id, const Aris::Core::XmlElement *ele)
			: Constraint(model, name, id, ele)
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
			std::copy_n(m.Data(), 3, static_cast<double*>(frc_coe_));
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
			pType->SetText(this->AdamsType());
			pEle->InsertEndChild(pType);

			Aris::Core::XmlElement *pPrtI = pEle->GetDocument()->NewElement("iPart");
			pPrtI->SetText(MakI().Father().Name().data());
			pEle->InsertEndChild(pPrtI);

			Aris::Core::XmlElement *pPrtJ = pEle->GetDocument()->NewElement("jPart");
			pPrtJ->SetText(MakJ().Father().Name().data());
			pEle->InsertEndChild(pPrtJ);

			Aris::Core::XmlElement *pMakI = pEle->GetDocument()->NewElement("iMarker");
			pMakI->SetText(MakI().Name().data());
			pEle->InsertEndChild(pMakI);

			Aris::Core::XmlElement *pMakJ = pEle->GetDocument()->NewElement("jMarker");
			pMakJ->SetText(MakJ().Name().data());
			pEle->InsertEndChild(pMakJ);

			Aris::Core::XmlElement *pFrictionCoefficients = pEle->GetDocument()->NewElement("Friction_Coefficients");

			pFrictionCoefficients->SetText(Matrix(1, 3, FrcCoe()).ToString().c_str());
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

		ForceBase::ForceBase(ModelBase &model, const std::string &name, int id, Marker &makI, Marker &makJ)
			: Interaction(model, name, id, makI, makJ)
		{
		}
		ForceBase::ForceBase(ModelBase &model, const std::string &name, int id, const Aris::Core::XmlElement *xmlEle)
			: Interaction(model, name, id, xmlEle)
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

		
		class Script::Imp 
		{
		public:
			struct Node
			{
				virtual void Simulate() = 0;
				virtual std::string AdamsScript() = 0;
			};
			struct ActivateNode final :public Node
			{
				bool isActive;
				Element *pEle;

				explicit ActivateNode(Element &ele, bool isActive) :pEle(&ele), isActive(isActive) {};
				virtual void Simulate()override { pEle->Activate(isActive); };
				virtual std::string AdamsScript()override
				{
					std::stringstream ss;
					std::string cmd = isActive ? "activate/" : "deactivate/";
					if (dynamic_cast<ForceBase*>(pEle))
					{
						ss << cmd << "sforce" << ", id=" << pEle->ID() + 1;
					}
					else
					{
						ss << cmd << pEle->GroupName() << ", id=" << pEle->ID() + 1;
					}
					return std::move(ss.str());
				};
			};
			struct MoveMarkerNode final :public Node
			{
				Marker *pMak;
				double prtPe[6];
				virtual void Simulate()override { s_pe2pm(prtPe, *pMak->prt_pm_); };
				virtual std::string AdamsScript()override
				{
					std::stringstream ss;
					ss << "marker/" << pMak->ID() + 1
						<< " , QP = " << prtPe[0] << "," << prtPe[1] << "," << prtPe[2]
						<< " , REULER =" << prtPe[3] << "," << prtPe[4] << "," << prtPe[5];
					return std::move(ss.str());
				};
				explicit MoveMarkerNode(Marker &mak, double *pe) :pMak(&mak) { std::copy_n(pe, 6, prtPe); };
			};
			struct SimulateNode final :public Node
			{
				std::uint32_t ms_dur_;
				std::uint32_t ms_dt_;
				virtual void Simulate()override { };
				virtual std::string AdamsScript()override
				{
					std::stringstream ss;
					ss << "simulate/transient, dur=" << double(ms_dur_)/1000.0 << ", dtout=" << double(ms_dt_) / 1000.0;
					return std::move(ss.str());
				};
				explicit SimulateNode(std::uint32_t ms_dur, std::uint32_t ms_dt) :ms_dur_(ms_dur), ms_dt_(ms_dt) { };
			};

			void Activate(Element &ele, bool isActive)
			{
				node_list.push_back(std::unique_ptr<Node>(new ActivateNode(ele,isActive)));
			}
			void MoveMarker(Marker &mak, double *pe)
			{
				node_list.push_back(std::unique_ptr<Node>(new MoveMarkerNode(mak, pe)));
			}
			void Simulate(std::uint32_t ms_dur, std::uint32_t ms_dt)
			{
				node_list.push_back(std::unique_ptr<Node>(new SimulateNode(ms_dur, ms_dt)));
			}
			void ToAdamsScript(std::ofstream &file) const
			{
				for (auto &pNode : node_list)
				{
					file << pNode->AdamsScript();
				}
			}
			void ToAdamsCmd(std::ofstream &file) const
			{
				file << "simulation script create &\r\n"
					<< "sim_script_name = default_script &\r\n"
					<< "solver_commands = ";
				
				for (auto &pNode : node_list)
				{
					file <<"&\r\n\""<< pNode->AdamsScript() << "\",";
				}

				file << "\"\"\r\n\r\n";
			}
		private:
			std::list<std::unique_ptr<Node> > node_list;

			friend Script;
		};
		Script::Script() :pImp(new Imp) {};
		Script::~Script() {};
		void Script::Activate(Element &ele, bool isActive) { pImp->Activate(ele, isActive); };
		void Script::MoveMarker(Marker &mak, double *pe) { pImp->MoveMarker(mak, pe); };
		void Script::Simulate(std::uint32_t ms_dur, std::uint32_t ms_dt) { pImp->Simulate(ms_dur, ms_dt); };
		void Script::ToAdamsCmd(std::ofstream &file) const
		{
			pImp->ToAdamsCmd(file);
		}
		void Script::ToAdamsScript(std::ofstream &file) const
		{
			pImp->ToAdamsScript(file);
		}
		bool Script::Empty()const { return pImp->node_list.empty(); };
		void Script::Clear() { pImp->node_list.clear(); };

		ModelBase::ModelBase(const std::string & name)
			: Object(*this, name)
			, environment_(*this)
			, pGround(nullptr)
		{
			AddPart("Ground");
			pGround = FindPart("Ground");
		}
		ModelBase::~ModelBase()
		{
		}

		template<class T>
		typename T::value_type::element_type * FindElement(const T &container, const std::string &name)
		{
			auto p = std::find_if(container.begin(), container.end(), [name](typename T::const_reference p)
			{
				return (p->Name() == name);
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
		Part *ModelBase::FindPart(const std::string &Name)
		{
			return FindElement<decltype(parts_)>(parts_, Name);
		}
		JointBase *ModelBase::FindJoint(const std::string &Name)
		{
			return FindElement<decltype(joints_)>(joints_, Name);
		}
		MotionBase *ModelBase::FindMotion(const std::string &Name)
		{
			return FindElement<decltype(motions_)>(motions_, Name);
		}
		ForceBase *ModelBase::FindForce(const std::string &Name)
		{
			return FindElement<decltype(forces_)>(forces_, Name);
		}

		void ModelBase::DynSetSolveMethod(std::function<void(int dim, const double *D, const double *b, double *x)> solve_method)
		{
			this->dyn_solve_method_ = solve_method;
		}
		void ModelBase::DynCstMtx(double *cst_mtx) const
		{
			std::fill_n(cst_mtx, DynDimN()*DynDimM(), 0);

			for (int i = 0; i < 6; ++i)
			{
				cst_mtx[DynDimN()*(pGround->row_id_ + i) + i] = 1;
			}

			for (auto &jnt : joints_)
			{
				if (jnt->IsActive())
				{
					s_block_cpy(6, jnt->CstDim(), jnt->CsmI(), 0, 0, jnt->CstDim(), cst_mtx, jnt->MakI().Father().row_id_, jnt->col_id_, DynDimN());
					s_block_cpy(6, jnt->CstDim(), jnt->CsmJ(), 0, 0, jnt->CstDim(), cst_mtx, jnt->MakJ().Father().row_id_, jnt->col_id_, DynDimN());
				}
			}
			for (auto &mot : motions_)
			{
				if (mot->IsActive())
				{
					s_block_cpy(6, 1, mot->CsmI(), 0, 0, 1, cst_mtx, mot->MakI().Father().row_id_, mot->col_id_, DynDimN());
					s_block_cpy(6, 1, mot->CsmJ(), 0, 0, 1, cst_mtx, mot->MakJ().Father().row_id_, mot->col_id_, DynDimN());
				}
			}
		}
		void ModelBase::DynIneMtx(double *ine_mtx) const
		{
			std::fill_n(ine_mtx, DynDimM()*DynDimM(), 0);

			for (int i = 0; i < 6; ++i)
			{
				ine_mtx[DynDimM()*(pGround->row_id_ + i) + pGround->row_id_ + i] = 1;
			}

			for (auto &prt : parts_)
			{
				if (prt->IsActive())
				{
					s_block_cpy(6, 6, *(prt->prt_im_), 0, 0, 6, ine_mtx, prt->row_id_, prt->row_id_, DynDimM());
				}
			}
		}
		void ModelBase::DynCstAcc(double *cst_acc) const
		{
			std::fill_n(cst_acc, DynDimN(), 0);

			for (auto &jnt : joints_)
			{
				if (jnt->IsActive())
				{
					std::copy_n(jnt->Csa(), jnt->CstDim(), &cst_acc[jnt->col_id_]);
				}
			}
			for (auto &mot : motions_)
			{
				if (mot->IsActive())
				{
					cst_acc[mot->col_id_] = *mot->Csa();
				}
			}
		}
		void ModelBase::DynPrtFce(double *prt_fce) const
		{
			std::fill_n(prt_fce, DynDimM(), 0);

			for (auto &prt : parts_)
			{
				if (prt->IsActive())
				{
					s_daxpy(6, -1, prt->prt_fg_, 1, &prt_fce[prt->row_id_], 1);
					s_daxpy(6, 1, prt->prt_fv_, 1, &prt_fce[prt->row_id_], 1);
				}
			}

			for (auto &fce : forces_)
			{
				if (fce->IsActive())
				{
					s_daxpy(6, -1, fce->FceI(), 1, &prt_fce[fce->MakI().Father().row_id_], 1);
					s_daxpy(6, -1, fce->FceJ(), 1, &prt_fce[fce->MakJ().Father().row_id_], 1);
				}
			}
		}
		void ModelBase::DynCstFce(double *cst_fce) const
		{
			for (auto &jnt : joints_)
			{
				if (jnt->IsActive())
				{
					std::copy_n(jnt->Csf(), jnt->CstDim(), &cst_fce[jnt->col_id_]);
				}
			}
			for (auto &mot : motions_)
			{
				if (mot->IsActive())
				{
					cst_fce[mot->col_id_] = mot->MotFceDyn();
				}
			}

		}
		void ModelBase::DynPrtAcc(double *cst_acc) const
		{
			for (auto &prt : parts_)
			{
				if (prt->IsActive())
				{
					std::copy_n(prt->PrtAcc(), 6, &cst_acc[prt->row_id_]);
				}
			}
		}
		void ModelBase::DynPre()
		{
			int pid = 0;//part id
			int cid = 6;//JointBase id

			for (auto &part:parts_)
			{
				if (part->IsActive())
				{
					part->row_id_ = pid;
					pid += 6;
				}
				else
				{
					part->row_id_ = 0;
				}
			}
			for (auto &joint:joints_)
			{
				if (joint->IsActive())
				{
					joint->Init();
					joint->col_id_ = cid;
					cid += joint->CstDim();
				}
				else
				{
					joint->col_id_ = 0;
				}
			}
			for (auto &motion:motions_)
			{
				if (motion->IsActive())
				{
					motion->col_id_ = cid;
					cid++;
					motion->Init();
				}
				else
				{
					motion->col_id_ = 0;
					motion->Init();
				}
			}

			dyn_prt_dim_ = pid;
			dyn_cst_dim_ = cid;
		}
		void ModelBase::DynUpd()
		{
			for (auto &prt : parts_)
			{
				if (prt->IsActive())prt->Update();
			}
			for (auto &jnt : joints_)
			{
				if (jnt->IsActive())jnt->Update();
			}
			for (auto &mot : motions_)
			{
				if (mot->IsActive())mot->Update();
				//std::cout << *mot->Csa() << std::endl;
			}
			for (auto &fce : forces_)
			{
				if (fce->IsActive())fce->Update();
			}
		}
		void ModelBase::DynMtx(double *D, double *b) const
		{
			DynCstMtx(&D[(DynDim())*DynDimM()]);
			s_block_cpy(DynDimM(), DynDimN(), &D[(DynDim())*DynDimM()], 0, 0, DynDimN(), D, 0, DynDimM(), DynDim());
			
			DynIneMtx(&D[(DynDim())*DynDimM()]);
			s_block_cpy(DynDimM(), DynDimM(), -1, &D[(DynDim())*DynDimM()], 0, 0, DynDimM(), 0, D, 0, 0, DynDim());

			std::fill_n(&D[(DynDim())*DynDimM()], DynDimN()*(DynDim()), 0);
			s_block_cpyT(DynDimM(), DynDimN(), D, 0, DynDimM(), DynDim(), D, DynDimM(), 0, DynDim());

			DynPrtFce(b);
			DynCstAcc(b + DynDimM());
		}
		void ModelBase::DynSov(const double *D, const double *b, double *x) const
		{
			if (dyn_solve_method_)
			{
				dyn_solve_method_(DynDim(), D, b, x);
			}
			else
			{
				throw std::runtime_error("please set solve_method before use DynSov");
			}
		}
		void ModelBase::DynEnd(const double *x)
		{
			for (auto &prt : parts_)
			{
				if (prt->IsActive())
				{
					std::copy_n(&x[prt->row_id_], 6, prt->prt_acc_);
				}
			}
			for (auto &jnt : joints_)
			{
				if (jnt->IsActive())
				{
					std::copy_n(&x[jnt->col_id_ + DynDimM()], jnt->CstDim(), jnt->Csf());
				}
			}
			for (auto &mot : motions_)
			{
				if (mot->IsActive())
				{
					mot->mot_fce_dyn_ = x[mot->col_id_ + DynDimM()];
				}
			}
		}
		void ModelBase::DynUkn(double *x) const
		{
			this->DynPrtAcc(x);
			this->DynCstFce(x + DynDimM());
		}
		void ModelBase::Dyn()
		{
			/*
			DynPre();
			
			std::vector<double> C(DynDimM() * DynDimN());
			std::vector<double> a_c(DynDimN());
			std::vector<double> I_mat(DynDimM() * DynDimM());
			std::vector<double> f(DynDimM());
			std::vector<double> D((DynDim()) * (DynDim()));
			std::vector<double> b(DynDim());

			DynMtx(C.data(), a_c.data(), I_mat.data(), f.data(), D.data(), b.data());

			std::vector<double> x(DynDim());
			solveMethod(DynDim(), D.data(), b.data(), x.data());

			DynEnd(x.data());
			*/
			DynPre();
			std::vector<double> D(DynDim() * DynDim());
			std::vector<double> b(DynDim());
			std::vector<double> x(DynDim());
			DynUpd();
			DynMtx(D.data(), b.data());
			DynSov(D.data(), b.data(), x.data());
			DynEnd(x.data());
		}

		void ModelBase::ClbSetInverseMethod(std::function<void(int n, double *A)> inverse_method)
		{
			this->clb_inverse_method_ = inverse_method;
		}
		void ModelBase::ClbPre()
		{
			DynPre();

			if (DynDimN() != DynDimM())
			{
				throw std::logic_error("must calibrate square matrix");
			}

			clb_dim_m_ = 0;
			clb_dim_n_ = 0;
			clb_dim_gam_ = 0;
			clb_dim_frc_ = 0;

			for (auto &i : motions_)
			{
				/*对于未激活的motion,认为其正在受到力控制*/
				if (i->IsActive())
				{
					clb_dim_m_++;
				}
				clb_dim_frc_ += 3;
				clb_dim_n_ += 3;
			}
			for (auto &i : parts_)
			{
				if (i->IsActive())
				{
					clb_dim_n_ += 10;
					clb_dim_gam_ += 10;
				}
			}

		}
		void ModelBase::ClbMtx(double *clb_D, double *clb_b)const
		{
			if (!clb_inverse_method_)throw std::runtime_error("please set inverse method before calibrate");
			if (DynDimN() != DynDimM()) throw std::logic_error("must calibrate square matrix");

			/*初始化*/
			static Matrix clb_d_m, clb_b_m;

			clb_d_m.Resize(ClbDimM(), ClbDimN());
			clb_b_m.Resize(ClbDimM(), 1);

			memset(clb_d_m.Data(), 0, clb_d_m.Length() * sizeof(double));
			memset(clb_b_m.Data(), 0, clb_b_m.Length() * sizeof(double));

			/*求A，即C的逆*/
			Matrix A(DynDimM(), DynDimM()), B(DynDimM(), DynDimM());

			std::vector<double> C(DynDimM() * DynDimM());
			std::vector<double> f(DynDimM());

			DynCstMtx(C.data());
			std::copy(C.begin(), C.end(), A.Data());
			clb_inverse_method_(DynDimM(), A.Data());

			/*求B*/
			const int beginRow = DynDimM() - ClbDimM();

			for (auto &i:parts_)
			{
				if (i->IsActive())
				{
					double cm[6][6];
					s_cmf(i->PrtVel(), *cm);
					s_dgemm(ClbDimM(), 6, 6, 1, &A(beginRow,i->row_id_), DynDimM(), *cm, 6, 0, &B(beginRow, i->row_id_), DynDimM());
				}
			}

			/*求解clb_d*/
			int col1 = 0, col2 = 0;

			for (auto &i:parts_)
			{
				if (i->IsActive())
				{
					double q[6]{0};
					std::copy_n(i->PrtAcc(), 6, q);
					s_daxpy(6, -1, i->PrtGravity(), 1, q, 1);
					
					double v[6];
					std::copy_n(i->PrtVel(), 6, v);

					for (int j = 0; j < ClbDimM(); ++j)
					{
						clb_d_m(j, col1) = A(beginRow + j, col2 + 0) * q[0] + A(beginRow + j, col2 + 1) * q[1] + A(beginRow + j, col2 + 2) * q[2];
						clb_d_m(j, col1 + 1) = A(beginRow + j, col2 + 1) * q[5] + A(beginRow + j, col2 + 5) * q[1] - A(beginRow + j, col2 + 2) * q[4] - A(beginRow + j, col2 + 4) * q[2];
						clb_d_m(j, col1 + 2) = A(beginRow + j, col2 + 2) * q[3] + A(beginRow + j, col2 + 3) * q[2] - A(beginRow + j, col2 + 0) * q[5] - A(beginRow + j, col2 + 5) * q[0];
						clb_d_m(j, col1 + 3) = A(beginRow + j, col2 + 0) * q[4] + A(beginRow + j, col2 + 4) * q[0] - A(beginRow + j, col2 + 1) * q[3] - A(beginRow + j, col2 + 3) * q[1];
						clb_d_m(j, col1 + 4) = A(beginRow + j, col2 + 3) * q[3];
						clb_d_m(j, col1 + 5) = A(beginRow + j, col2 + 4) * q[4];
						clb_d_m(j, col1 + 6) = A(beginRow + j, col2 + 5) * q[5];
						clb_d_m(j, col1 + 7) = A(beginRow + j, col2 + 3) * q[4] + A(beginRow + j, col2 + 4) * q[3];
						clb_d_m(j, col1 + 8) = A(beginRow + j, col2 + 3) * q[5] + A(beginRow + j, col2 + 5) * q[3];
						clb_d_m(j, col1 + 9) = A(beginRow + j, col2 + 4) * q[5] + A(beginRow + j, col2 + 5) * q[4];

						clb_d_m(j, col1) += B(beginRow + j, col2 + 0) * v[0] + B(beginRow + j, col2 + 1) * v[1] + B(beginRow + j, col2 + 2) * v[2];
						clb_d_m(j, col1 + 1) += B(beginRow + j, col2 + 1) * v[5] + B(beginRow + j, col2 + 5) * v[1] - B(beginRow + j, col2 + 2) * v[4] - B(beginRow + j, col2 + 4) * v[2];
						clb_d_m(j, col1 + 2) += B(beginRow + j, col2 + 2) * v[3] + B(beginRow + j, col2 + 3) * v[2] - B(beginRow + j, col2 + 0) * v[5] - B(beginRow + j, col2 + 5) * v[0];
						clb_d_m(j, col1 + 3) += B(beginRow + j, col2 + 0) * v[4] + B(beginRow + j, col2 + 4) * v[0] - B(beginRow + j, col2 + 1) * v[3] - B(beginRow + j, col2 + 3) * v[1];
						clb_d_m(j, col1 + 4) += B(beginRow + j, col2 + 3) * v[3];
						clb_d_m(j, col1 + 5) += B(beginRow + j, col2 + 4) * v[4];
						clb_d_m(j, col1 + 6) += B(beginRow + j, col2 + 5) * v[5];
						clb_d_m(j, col1 + 7) += B(beginRow + j, col2 + 3) * v[4] + B(beginRow + j, col2 + 4) * v[3];
						clb_d_m(j, col1 + 8) += B(beginRow + j, col2 + 3) * v[5] + B(beginRow + j, col2 + 5) * v[3];
						clb_d_m(j, col1 + 9) += B(beginRow + j, col2 + 4) * v[5] + B(beginRow + j, col2 + 5) * v[4];
					}
					col1 += 10;
					col2 += 6;
				}
			}

			/*求解clb_b*/
			std::fill(f.begin(), f.end(), 0);
			int row = 0;
			for (auto &mot : motions_)
			{
				if (mot->IsActive())
				{
					clb_b_m(row, 0) = mot->MotFce();
					++row;
				}
			}
			for (auto &fce : forces_)
			{
				if (fce->IsActive())
				{
					s_daxpy(6, 1, fce->FceI(), 1, &f[fce->MakI().Father().row_id_], 1);
					s_daxpy(6, 1, fce->FceJ(), 1, &f[fce->MakJ().Father().row_id_], 1);
				}
			}
			s_dgemm(ClbDimM(), 1, DynDimM(), 1, &A(beginRow,0), DynDimM(), f.data(), 1, 1, clb_b_m.Data(), 1);

			/*以下添加驱动摩擦系数*/
			row = 0;
			int num = 0;
			for (auto &mot : motions_)
			{
				//默认未激活的motion处于力控模式
				if (mot->IsActive())
				{
					clb_d_m(row, ClbDimGam() + num * 3) += s_sgn(mot->MotVel());
					clb_d_m(row, ClbDimGam() + num * 3 + 1) += mot->MotVel();
					clb_d_m(row, ClbDimGam() + num * 3 + 2) += mot->MotAcc();
					++row;
				}
				else
				{
					s_dgemm(ClbDimM(), 1, 6, s_sgn(mot->MotVel()), &A(beginRow, mot->MakI().Father().row_id_), DynDimM(), mot->CsmI(), 1, 1, &clb_d_m(0, ClbDimGam() + num * 3), ClbDimN());
					s_dgemm(ClbDimM(), 1, 6, s_sgn(mot->MotVel()), &A(beginRow, mot->MakJ().Father().row_id_), DynDimM(), mot->CsmJ(), 1, 1, &clb_d_m(0, ClbDimGam() + num * 3), ClbDimN());
					s_dgemm(ClbDimM(), 1, 6, mot->MotVel(), &A(beginRow, mot->MakI().Father().row_id_), DynDimM(), mot->CsmI(), 1, 1, &clb_d_m(0, ClbDimGam() + num * 3 + 1), ClbDimN());
					s_dgemm(ClbDimM(), 1, 6, mot->MotVel(), &A(beginRow, mot->MakJ().Father().row_id_), DynDimM(), mot->CsmJ(), 1, 1, &clb_d_m(0, ClbDimGam() + num * 3 + 1), ClbDimN());
					s_dgemm(ClbDimM(), 1, 6, mot->MotAcc(), &A(beginRow, mot->MakI().Father().row_id_), DynDimM(), mot->CsmI(), 1, 1, &clb_d_m(0, ClbDimGam() + num * 3 + 2), ClbDimN());
					s_dgemm(ClbDimM(), 1, 6, mot->MotAcc(), &A(beginRow, mot->MakJ().Father().row_id_), DynDimM(), mot->CsmJ(), 1, 1, &clb_d_m(0, ClbDimGam() + num * 3 + 2), ClbDimN());
				}

				num++;
			}

			std::copy_n(clb_d_m.Data(), clb_d_m.Length(), clb_D);
			std::copy_n(clb_b_m.Data(), clb_b_m.Length(), clb_b);
		}
		void ModelBase::ClbUkn(double *clb_x)const
		{
			int row = 0;
			for (auto &prt : parts_)
			{
				if (prt->IsActive())
				{
					s_im2gamma(*prt->PrtIm(), clb_x + row);
				}
				row += 10;
			}

			for (auto &mot : motions_)
			{
				std::copy_n(mot->FrcCoe(),3, clb_x + row);
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

			Environment().FromXmlElement(pEnv);

			parts_.clear();
			joints_.clear();
			motions_.clear();
			forces_.clear();

			/*读入地面*/
			for (auto ele = pPrt->FirstChildElement(); ele != nullptr; ele = ele->NextSiblingElement())
			{
				if (std::string(ele->Name()) == "Ground")
				{
					AddPart(ele->Name(), ele);
					pGround = FindPart("Ground");
					break;
				}
			}

			if (this->FindPart("Ground") == nullptr)
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
			Environment().ToXmlElement(pEnvironment);
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

			for (auto &p:parts_)
			{
				Aris::Core::XmlElement *ele = XML_Doc.NewElement("");
				p->ToXmlElement(ele);
				pPrt->InsertEndChild(ele);
			}

			for (auto &j : joints_)
			{
				Aris::Core::XmlElement *ele = XML_Doc.NewElement("");
				j->ToXmlElement(ele);
				pJnt->InsertEndChild(ele);
			}

			for (auto &m : motions_)
			{
				Aris::Core::XmlElement *ele = XML_Doc.NewElement("");
				m->ToXmlElement(ele);
				pMot->InsertEndChild(ele);
			}


			XML_Doc.SaveFile(filename);
		}
		void ModelBase::SaveAdams(const char *filename, bool isModifyActive) const
		{
			std::string cmdName = std::string(filename) + std::string(".cmd");
			
			std::ofstream file;
			file.open(cmdName, std::ios::out | std::ios::trunc);
			file << std::setprecision(15);

			file << "!----------------------------------- Environment -------------------------------!\r\n!\r\n!\r\n";
			Environment().ToAdamsCmd(file);

			file << "!----------------------------------- Parts -------------------------------------!\r\n!\r\n!\r\n";
			pGround->ToAdamsCmd(file);
			for (auto &prt : parts_)
			{
				if (prt.get() != pGround)
				{
					prt->ToAdamsCmd(file);
				}
			}

			file << "!----------------------------------- Joints ------------------------------------!\r\n!\r\n!\r\n";
			for (auto &jnt : joints_)
			{
				jnt->ToAdamsCmd(file);
			}

			file << "!----------------------------------- Motions -----------------------------------!\r\n!\r\n!\r\n";
			for (auto &mot : motions_)
			{
				mot->ToAdamsCmd(file);
			}

			file << "!----------------------------------- Forces ------------------------------------!\r\n!\r\n!\r\n";
			for (auto &fce : forces_)
			{
				fce->ToAdamsCmd(file);
			}

			if (isModifyActive)
			{
				file << "!----------------------------------- Motify Active -------------------------------------!\r\n!\r\n!\r\n";
				for (auto &prt : parts_)
				{
					if ((prt.get() != pGround) && (!prt->IsActive()))
					{
						file << "part attributes  &\r\n"
							<< "    part_name = ." << Name() << "." << prt->Name() << "  &\r\n"
							<< "    active = off \r\n!\r\n";
					}
				}
				for (auto &jnt : joints_)
				{
					if (!jnt->IsActive())
					{
						file << "constraint attributes  &\r\n"
							<< "    constraint_name = ." << Name() << "." << jnt->Name() << "  &\r\n"
							<< "    active = off \r\n!\r\n";
					}
					
				}
				for (auto &mot : motions_)
				{
					if (!mot->IsActive())
					{
						file << "constraint attributes  &\r\n"
							<< "    constraint_name = ." << Name() << "." << mot->Name() << "  &\r\n"
							<< "    active = off \r\n!\r\n";
					}
					
				}
				for (auto &fce : forces_)
				{
					if (!fce->IsActive())
					{
						file << "force attributes  &\r\n"
							<< "    force_name = ." << Name() << "." << fce->Name() << "  &\r\n"
							<< "    active = off \r\n!\r\n";
					}
				}
			}
			
			file << "!----------------------------------- Script ------------------------------------!\r\n!\r\n!\r\n";
			this->Script().ToAdamsCmd(file);

			file.close();


		}
	}
}
