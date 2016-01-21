#include <cmath>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <algorithm>
#include <limits>
#include <sstream>

#include "aris_exp_cal.h"
#include "aris_dyn_kernel.h"
#include "aris_dyn_model_base.h"

namespace Aris
{
	namespace DynKer
	{
		Interaction::Interaction(ModelBase &model, const std::string &name, int id, const Aris::Core::XmlElement &xml_ele)
			: Element(model, name, id)
		{
			if (!Model().FindPart(xml_ele.Attribute("PrtM")))
				throw std::runtime_error(std::string("can't find part M for element \"") + this->Name() + "\"");
			
			makI_ = Model().FindPart(xml_ele.Attribute("PrtM"))->FindMarker(xml_ele.Attribute("MakI"));
			if(!makI_)throw std::runtime_error(std::string("can't find marker I for element \"") + this->Name() + "\"");

			if (!Model().FindPart(xml_ele.Attribute("PrtN")))
				throw std::runtime_error(std::string("can't find part N for element \"") + this->Name() + "\"");

			makJ_ = Model().FindPart(xml_ele.Attribute("PrtN"))->FindMarker(xml_ele.Attribute("MakJ"));
			if (!makJ_)throw std::runtime_error(std::string("can't find marker J for element \"") + this->Name() + "\"");
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
		void Constraint::ToXmlElement(Aris::Core::XmlElement &xml_ele) const
		{
			xml_ele.DeleteChildren();
			xml_ele.SetName(this->Name().data());

			Aris::Core::XmlElement *pActive = xml_ele.GetDocument()->NewElement("Active");
			if (this->IsActive())
				pActive->SetText("True");
			else
				pActive->SetText("False");
			xml_ele.InsertEndChild(pActive);

			Aris::Core::XmlElement *pType = xml_ele.GetDocument()->NewElement("Type");
			pType->SetText(this->AdamsType().c_str());
			xml_ele.InsertEndChild(pType);

			Aris::Core::XmlElement *pPrtI = xml_ele.GetDocument()->NewElement("iPart");
			pPrtI->SetText(MakI().Father().Name().data());
			xml_ele.InsertEndChild(pPrtI);

			Aris::Core::XmlElement *pPrtJ = xml_ele.GetDocument()->NewElement("jPart");
			pPrtJ->SetText(MakJ().Father().Name().data());
			xml_ele.InsertEndChild(pPrtJ);

			Aris::Core::XmlElement *pMakI = xml_ele.GetDocument()->NewElement("iMarker");
			pMakI->SetText(MakI().Name().data());
			xml_ele.InsertEndChild(pMakI);

			Aris::Core::XmlElement *pMakJ = xml_ele.GetDocument()->NewElement("jMarker");
			pMakJ->SetText(MakJ().Name().data());
			xml_ele.InsertEndChild(pMakJ);
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
		void Marker::ToXmlElement(Aris::Core::XmlElement &xml_ele) const
		{
			double value[10];

			xml_ele.DeleteChildren();
			xml_ele.SetName(this->Name().data());

			Aris::Core::XmlElement *pPE = xml_ele.GetDocument()->NewElement("Pos");
			s_pm2pe(*PrtPm(), value);
			pPE->SetText(Matrix(1, 6, value).ToString().c_str());
			xml_ele.InsertEndChild(pPE);

			Aris::Core::XmlElement *pRelativeMakEle = xml_ele.GetDocument()->NewElement("RelativeTo");
			pRelativeMakEle->SetText("");
			xml_ele.InsertEndChild(pRelativeMakEle);
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
		void Part::ToXmlElement(Aris::Core::XmlElement &xml_ele) const
		{
			double value[10];
			
			xml_ele.DeleteChildren();
			xml_ele.SetName(this->Name().data());

			Aris::Core::XmlElement *pActive = xml_ele.GetDocument()->NewElement("Active");
			if (this->IsActive())
				pActive->SetText("True");
			else
				pActive->SetText("False");
			xml_ele.InsertEndChild(pActive);
			
			Aris::Core::XmlElement *pInertia = xml_ele.GetDocument()->NewElement("Inertia");
			s_im2gamma(*this->PrtIm(),value);
			pInertia->SetText(Matrix(1,10,value).ToString().c_str());
			xml_ele.InsertEndChild(pInertia);

			Aris::Core::XmlElement *pPE = xml_ele.GetDocument()->NewElement("Pos");
			s_pm2pe(*this->Pm(), value);
			pPE->SetText(Matrix(1, 6, value).ToString().c_str());
			xml_ele.InsertEndChild(pPE);

			Aris::Core::XmlElement *pVel = xml_ele.GetDocument()->NewElement("Vel");
			pVel->SetText(Matrix(1, 6, Vel()).ToString().c_str());
			xml_ele.InsertEndChild(pVel);

			Aris::Core::XmlElement *pAcc = xml_ele.GetDocument()->NewElement("Acc");
			pAcc->SetText(Matrix(1, 6, Acc()).ToString().c_str());
			xml_ele.InsertEndChild(pAcc);

			Aris::Core::XmlElement *pChildMak = xml_ele.GetDocument()->NewElement("ChildMarker");
			xml_ele.InsertEndChild(pChildMak);

			for (auto &m : marker_names_)
			{
				Aris::Core::XmlElement *ele = xml_ele.GetDocument()->NewElement("");

				Model().markers_.at(m.second)->ToXmlElement(*ele);
				pChildMak->InsertEndChild(ele);
			}

			Aris::Core::XmlElement *pGraphicFilePath = xml_ele.GetDocument()->NewElement("Graphic_File_Path");
			pGraphicFilePath->SetText(this->graphic_file_path_.c_str());
			xml_ele.InsertEndChild(pGraphicFilePath);
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
		MotionBase::MotionBase(ModelBase &model, const std::string &name, int id, const Aris::Core::XmlElement &xml_ele)
			: Constraint(model, name, id, xml_ele)
		{
			if (strcmp("true", xml_ele.Attribute("Active")) == 0)
			{
				this->Activate();
			}
			else if (strcmp("false", xml_ele.Attribute("Active")) == 0)
			{
				this->Activate(false);
			}
			else
			{
				throw std::logic_error("failed load xml file in motion");
			}

			Matrix m = Model().calculator.CalculateExpression(xml_ele.Attribute("FrcCoe"));
			std::copy_n(m.Data(), 3, static_cast<double*>(frc_coe_));
		}
		void MotionBase::ToXmlElement(Aris::Core::XmlElement &xml_ele) const
		{
			xml_ele.DeleteChildren();
			xml_ele.SetName(this->Name().data());

			Aris::Core::XmlElement *pActive = xml_ele.GetDocument()->NewElement("Active");
			if (this->IsActive())
				pActive->SetText("True");
			else
				pActive->SetText("False");
			xml_ele.InsertEndChild(pActive);

			Aris::Core::XmlElement *pType = xml_ele.GetDocument()->NewElement("Type");
			pType->SetText(this->AdamsType().c_str());
			xml_ele.InsertEndChild(pType);

			Aris::Core::XmlElement *pPrtI = xml_ele.GetDocument()->NewElement("iPart");
			pPrtI->SetText(MakI().Father().Name().data());
			xml_ele.InsertEndChild(pPrtI);

			Aris::Core::XmlElement *pPrtJ = xml_ele.GetDocument()->NewElement("jPart");
			pPrtJ->SetText(MakJ().Father().Name().data());
			xml_ele.InsertEndChild(pPrtJ);

			Aris::Core::XmlElement *pMakI = xml_ele.GetDocument()->NewElement("iMarker");
			pMakI->SetText(MakI().Name().data());
			xml_ele.InsertEndChild(pMakI);

			Aris::Core::XmlElement *pMakJ = xml_ele.GetDocument()->NewElement("jMarker");
			pMakJ->SetText(MakJ().Name().data());
			xml_ele.InsertEndChild(pMakJ);

			Aris::Core::XmlElement *pFrictionCoefficients = xml_ele.GetDocument()->NewElement("Friction_Coefficients");

			pFrictionCoefficients->SetText(Matrix(1, 3, FrcCoe()).ToString().c_str());
			xml_ele.InsertEndChild(pFrictionCoefficients);
		}

		ForceBase::ForceBase(ModelBase &model, const std::string &name, int id, Marker &makI, Marker &makJ)
			: Interaction(model, name, id, makI, makJ)
		{
		}
		ForceBase::ForceBase(ModelBase &model, const std::string &name, int id, const Aris::Core::XmlElement &xml_ele)
			: Interaction(model, name, id, xml_ele)
		{

		}

		void Environment::ToXmlElement(Aris::Core::XmlElement &xml_ele) const
		{
			xml_ele.DeleteChildren();
			xml_ele.SetName("Enviroment");

			Aris::Core::XmlElement *pGravity = xml_ele.GetDocument()->NewElement("Gravity");
			pGravity->SetText(Matrix(1, 6, Gravity).ToString().c_str());
			xml_ele.InsertEndChild(pGravity);
		}
		void Environment::FromXmlElement(const Aris::Core::XmlElement &xml_ele)
		{
			Matrix m = Model().calculator.CalculateExpression(xml_ele.FirstChildElement("Gravity")->GetText());
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
				virtual void DoNode() {};
				virtual void Update() {};
				virtual std::uint32_t MsConsumed()const { return 0; };
				virtual std::string AdamsScript()const = 0;
			};
			struct ActivateNode final :public Node
			{
				virtual void DoNode()override { pEle->Activate(isActive); };
				virtual std::string AdamsScript()const override
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
				
				explicit ActivateNode(Element &ele, bool isActive) :pEle(&ele), isActive(isActive) {};
				bool isActive;
				Element *pEle;
			};
			struct MoveMarkerNode final :public Node
			{
				virtual void DoNode()override 
				{
					s_pm2pe(prtPe, *mak_move_.prt_pm_);
				};
				void Update() override 
				{
					double pm_target_g[16];

					s_pm_dot_pm(*mak_target_.Father().Pm(), *mak_target_.PrtPm(), pm_target_g);
					s_inv_pm_dot_pm(*mak_move_.Father().Pm(), pm_target_g, &mak_move_.prt_pm_[0][0]);
					s_pm2pe(*mak_move_.prt_pm_, prtPe);
				};
				virtual std::string AdamsScript()const override
				{
					std::stringstream ss;
					ss << "marker/" << mak_move_.ID() + 1
						<< " , QP = " << prtPe[0] << "," << prtPe[1] << "," << prtPe[2]
						<< " , REULER =" << prtPe[3] << "," << prtPe[4] << "," << prtPe[5];
					return std::move(ss.str());
				};
				
				explicit MoveMarkerNode(Marker &mak_move, const Marker &mak_target) :mak_move_(mak_move), mak_target_(mak_target){};
				Marker &mak_move_;
				const Marker &mak_target_;
				double prtPe[6];
			};
			struct SimulateNode final :public Node
			{
				virtual std::uint32_t MsConsumed()const { return ms_dur_; };
				virtual std::string AdamsScript()const override
				{
					std::stringstream ss;
					ss << "simulate/transient, dur=" << double(ms_dur_) / 1000.0 << ", dtout=" << double(ms_dt_) / 1000.0;
					return std::move(ss.str());
				};
				
				explicit SimulateNode(std::uint32_t ms_dur, std::uint32_t ms_dt) :ms_dur_(ms_dur), ms_dt_(ms_dt) { };
				std::uint32_t ms_dur_;
				std::uint32_t ms_dt_;
			};

			void Activate(Element &ele, bool isActive)
			{
				node_list_.push_back(std::unique_ptr<Node>(new ActivateNode(ele,isActive)));
			}
			void MoveMarker(Marker &mak, const Marker& mak_target)
			{
				node_list_.push_back(std::unique_ptr<Node>(new MoveMarkerNode(mak, mak_target)));
			}
			void Simulate(std::uint32_t ms_dur, std::uint32_t ms_dt)
			{
				node_list_.push_back(std::unique_ptr<Node>(new SimulateNode(ms_dur, ms_dt)));
			}
			void ToAdamsCmd(std::ofstream &file) const
			{
				file << "simulation script create &\r\n"
					<< "sim_script_name = default_script &\r\n"
					<< "solver_commands = ";
				
				for (auto &pNode : node_list_)
				{
					file <<"&\r\n\""<< pNode->AdamsScript() << "\",";
				}

				file << "\"\"\r\n\r\n";
			}
			std::int32_t EndTime()const
			{
				std::uint32_t end_time{0};

				for (auto& node : node_list_)end_time += node->MsConsumed();
				
				return end_time;
			}
			void SetTopologyAt(std::uint32_t ms_time) 
			{
				std::uint32_t now{ 0 };
				for (auto p = node_list_.begin(); (p != node_list_.end()) && (now + (*p)->MsConsumed() <= ms_time); ++p)
				{
					(*p)->DoNode();
				}
			};

		private:
			std::list<std::unique_ptr<Node> > node_list_;

			friend Script;
		};
		Script::Script() :pImp(new Imp) {};
		Script::~Script() {};
		void Script::Activate(Element &ele, bool isActive) { pImp->Activate(ele, isActive); };
		void Script::MoveMarker(Marker &mak, const Marker& mak_target) { pImp->MoveMarker(mak, mak_target); };
		void Script::Simulate(std::uint32_t ms_dur, std::uint32_t ms_dt) { pImp->Simulate(ms_dur, ms_dt); };
		void Script::ToAdamsCmd(std::ofstream &file) const
		{
			pImp->ToAdamsCmd(file);
		}
		bool Script::Empty()const { return pImp->node_list_.empty(); };
		std::uint32_t Script::EndTime()const 
		{
			return pImp->EndTime();
		};
		void Script::SetTopologyAt(std::uint32_t ms_time) { pImp->SetTopologyAt(ms_time); };
		void Script::UpdateAt(std::uint32_t ms_time)
		{
			std::uint32_t now = 0;
			for (auto&node : pImp->node_list_)
			{
				now += node->MsConsumed();
				if (now == ms_time)node->Update();
			}
		}
		void Script::Clear() { pImp->node_list_.clear(); };

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
				if (i->IsActive())
				{
					clb_dim_m_++;
					clb_dim_frc_ += 3;
					clb_dim_n_ += 3;
				}
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
		void ModelBase::ClbUpd()
		{
			DynUpd();
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
					clb_b_m(row, 0) = mot->mot_fce_;
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
			for (auto &mot : motions_)
			{
				//默认未激活的motion处于力控模式
				if (mot->IsActive())
				{
					clb_d_m(row, ClbDimGam() + row * 3) += s_sgn(mot->MotVel());
					clb_d_m(row, ClbDimGam() + row * 3 + 1) += mot->MotVel();
					clb_d_m(row, ClbDimGam() + row * 3 + 2) += mot->MotAcc();
					++row;
				}
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
					row += 10;
				}
			}

			for (auto &mot : motions_)
			{
				if (mot->IsActive())
				{
					std::copy_n(mot->FrcCoe(), 3, clb_x + row);
					row += 3;
				}
			}
		}
		
		void ModelBase::SimKin(const PlanFunc &func, const PlanParamBase &param, SimResult &result, bool using_script)
		{
			//储存起始的每个杆件的位置
			result.begin_prt_pe_.clear();
			for (std::size_t i = 0; i < PartNum(); ++i)
			{
				std::array<double, 6>pe;
				PartAt(i).GetPe(pe.data());
				result.begin_prt_pe_.push_back(pe);
			}

			//初始化变量
			result.Pin_.clear();
			result.Pin_.resize(this->MotionNum());

			//起始位置
			result.time_.push_back(0);
			for (std::size_t i = 0; i < MotionNum(); ++i)
			{
				MotionAt(i).Update();
				result.Pin_.at(i).push_back(MotionAt(i).MotPos());
			}

			if (using_script)
			{
				this->Script().UpdateAt(0);
				this->Script().SetTopologyAt(0);
			}

			//其他位置
			for (param.count = 0; true; ++param.count)
			{
				auto is_sim = func(*this, param);

				result.time_.push_back(param.count + 1);
				for (std::size_t i = 0; i < MotionNum(); ++i)
				{
					result.Pin_.at(i).push_back(MotionAt(i).MotPos());
				}

				if (using_script)
				{
					this->Script().UpdateAt(param.count + 1);
					this->Script().SetTopologyAt(param.count + 1);
				}

				if (!is_sim)break;
			}

			//恢复起始位置
			auto pe = result.begin_prt_pe_.begin();
			for (std::size_t i = 0; i < PartNum(); ++i)
			{
				PartAt(i).SetPe(pe->data());
				++pe;
			}
		}
		void ModelBase::SimDyn(const PlanFunc &func, const PlanParamBase &param, SimResult &result, bool using_script)
		{
			
		}
		void ModelBase::SimKinAkima(const PlanFunc &func, const PlanParamBase &param, SimResult &result, int akima_interval, bool using_script)
		{
			SimKin(func, param, result, using_script);
			
			//生成Akima
			std::list<double> time_akima_data;
			std::vector<std::list<double> > pos_akima_data(MotionNum());

			//初始化迭代器
			auto p_time = result.time_.begin();
			std::vector<std::list<double>::iterator> p_Pin(MotionNum());
			for (std::size_t i = 0; i < MotionNum(); ++i)
				p_Pin.at(i) = result.Pin_.at(i).begin();

			//生成Akima
			for (std::size_t i = 0; i < result.time_.size(); ++i)
			{
				if ((i == (result.time_.size() - 1)) || (i%akima_interval == 0))
				{
					time_akima_data.push_back(*p_time / 1000.0);

					for (std::size_t j = 0; j < MotionNum(); ++j)
					{
						pos_akima_data.at(j).push_back(*p_Pin.at(j));
					}
				}

				++p_time;
				for (auto &j : p_Pin)++j;
			}

			//设置Akima
			for (std::size_t i = 0; i < MotionNum(); ++i)
			{
				MotionAt(i).posCurve.reset(new Akima(time_akima_data, pos_akima_data.at(i)));
			}
		}
		void ModelBase::SimDynAkima(const PlanFunc &func, const PlanParamBase &param, SimResult &result, int akima_interval, bool using_script)
		{
			SimKinAkima(func, param, result, akima_interval, using_script);
			
			result.Pin_.clear();
			result.Fin_.clear();
			result.Ain_.clear();
			result.Vin_.clear();

			result.Pin_.resize(MotionNum());
			result.Fin_.resize(MotionNum());
			result.Vin_.resize(MotionNum());
			result.Ain_.resize(MotionNum());

			//仿真计算
			for (std::size_t t = 0; t < result.time_.size();++t)
			{
				std::cout << t << std::endl;
				
				if (using_script)
				{
					this->Script().SetTopologyAt(t);
				}
				for (std::size_t j = 0; j < MotionNum(); ++j)
				{
					MotionAt(j).mot_pos_ = MotionAt(j).PosAkima(t / 1000.0, '0');
				}
				KinFromPin();
				for (std::size_t j = 0; j < MotionNum(); ++j)
				{
					MotionAt(j).mot_vel_ = MotionAt(j).PosAkima(t / 1000.0, '1');
				}
				KinFromVin();
				for (std::size_t j = 0; j < MotionNum(); ++j)
				{
					MotionAt(j).mot_acc_ = MotionAt(j).PosAkima(t / 1000.0, '2');
				}
				Dyn();
				for (std::size_t j = 0; j < MotionNum(); ++j)
				{
					result.Fin_.at(j).push_back(MotionAt(j).mot_fce_dyn_);
					result.Pin_.at(j).push_back(MotionAt(j).MotPos());
					result.Vin_.at(j).push_back(MotionAt(j).MotVel());
					result.Ain_.at(j).push_back(MotionAt(j).MotAcc());
				}
			}

			//恢复起始位置
			auto pe = result.begin_prt_pe_.begin();
			for (std::size_t i = 0; i < PartNum(); ++i)
			{
				PartAt(i).SetPe(pe->data());
				++pe;
			}
		}
		void ModelBase::SimToAdams(const std::string &adams_file, const PlanFunc &fun, const PlanParamBase &param, int ms_dt, bool using_script)
		{
			SimResult result;
			SimKinAkima(fun, param, result, ms_dt, using_script);
			this->SaveAdams(adams_file, using_script);
		}

		void ModelBase::LoadXml(const std::string &filename)
		{
			Aris::Core::XmlDocument xmlDoc;
			
			if (xmlDoc.LoadFile(filename.c_str()) != 0)
			{
				throw std::logic_error((std::string("could not open file:") + std::string(filename)));
			}

			LoadXml(xmlDoc);
		}
		void ModelBase::LoadXml(const Aris::Core::XmlDocument &xml_doc)
		{
			auto pModel = xml_doc.RootElement()->FirstChildElement("Model");

			if (!pModel)throw std::logic_error("can't find Model element in xml file");

			LoadXml(*pModel);
		}
		void ModelBase::LoadXml(const Aris::Core::XmlElement &xml_ele)
		{
			auto pVar = xml_ele.FirstChildElement("Variable");
			if (!pVar)throw(std::logic_error("Model must have variable element"));
			auto pEnv = xml_ele.FirstChildElement("Environment");
			if (!pEnv)throw(std::logic_error("Model must have environment element"));
			auto pPrt = xml_ele.FirstChildElement("Part");
			if (!pPrt)throw(std::logic_error("Model must have part element"));

			calculator.ClearVariables();
			for (auto ele = pVar->FirstChildElement(); ele != nullptr; ele = ele->NextSiblingElement())
			{
				calculator.AddVariable(ele->Name(), calculator.CalculateExpression(ele->GetText()));
			}

			Environment().FromXmlElement(*pEnv);

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
			Environment().ToXmlElement(*pEnvironment);
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
				p->ToXmlElement(*ele);
				pPrt->InsertEndChild(ele);
			}

			for (auto &j : joints_)
			{
				Aris::Core::XmlElement *ele = XML_Doc.NewElement("");
				j->ToXmlElement(*ele);
				pJnt->InsertEndChild(ele);
			}

			for (auto &m : motions_)
			{
				Aris::Core::XmlElement *ele = XML_Doc.NewElement("");
				m->ToXmlElement(*ele);
				pMot->InsertEndChild(ele);
			}


			XML_Doc.SaveFile(filename);
		}
		void ModelBase::SaveAdams(const std::string &filename, bool using_script) const
		{
			std::string filename_ = filename;
			if (filename_.size() < 4 || filename_.substr(filename.size() - 4, 4) != ".cmd")
			{
				filename_ += ".cmd";
			}
			
			std::ofstream file;
			file.open(filename_, std::ios::out | std::ios::trunc);
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

			if (using_script)
			{
				file << "!----------------------------------- Script ------------------------------------!\r\n!\r\n!\r\n";
				this->Script().ToAdamsCmd(file);
			}
			else
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
			
			file.close();


		}
	}
}
