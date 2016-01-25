#include <cmath>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <algorithm>
#include <limits>
#include <sstream>

#include "aris_core.h"
#include "aris_dynamic_kernel.h"
#include "aris_dynamic_model_base.h"

namespace Aris
{
	namespace Dynamic
	{
		Interaction::Interaction(ModelBase &model_in, const std::string &name, int id, const Aris::Core::XmlElement &xml_ele)
			: Element(model_in, name, id)
		{
			if (!model().findPart(xml_ele.Attribute("PrtM")))
				throw std::runtime_error(std::string("can't find part M for element \"") + this->name() + "\"");
			
			makI_ = model().findPart(xml_ele.Attribute("PrtM"))->findMarker(xml_ele.Attribute("MakI"));
			if(!makI_)throw std::runtime_error(std::string("can't find marker I for element \"") + this->name() + "\"");

			if (!model().findPart(xml_ele.Attribute("PrtN")))
				throw std::runtime_error(std::string("can't find part N for element \"") + this->name() + "\"");

			makJ_ = model().findPart(xml_ele.Attribute("PrtN"))->findMarker(xml_ele.Attribute("MakJ"));
			if (!makJ_)throw std::runtime_error(std::string("can't find marker J for element \"") + this->name() + "\"");
		}
		void Constraint::update()
		{
			double pm_M2N[4][4];
			double _tem_v1[6]{ 0 }, _tem_v2[6]{ 0 };

			/* Get pm M2N */
			s_pm_dot_pm(*makJ().father().invPm(), *makI().father().pm(), *pm_M2N);

			/*update CstMtx*/
			std::fill_n(this->csmJ(), this->cstDim() * 6, 0);
			s_tf_n(cstDim(), -1, *pm_M2N, this->csmI(), 0, this->csmJ());

			/*update CstAcc*/
			std::fill_n(this->csa(), this->cstDim(), 0);
			s_inv_tv(-1, *pm_M2N, makJ().father().prtVel(), 0, _tem_v1);
			s_cv(makI().father().prtVel(), _tem_v1, _tem_v2);
			s_dgemmTN(cstDim(), 1, 6, 1, csmI(), cstDim(), _tem_v2, 1, 0, csa(), 1);
		}
		void Constraint::toXmlElement(Aris::Core::XmlElement &xml_ele) const
		{
			xml_ele.DeleteChildren();
			xml_ele.SetName(this->name().data());

			Aris::Core::XmlElement *pActive = xml_ele.GetDocument()->NewElement("Active");
			if (this->isActive())
				pActive->SetText("True");
			else
				pActive->SetText("False");
			xml_ele.InsertEndChild(pActive);

			Aris::Core::XmlElement *pType = xml_ele.GetDocument()->NewElement("Type");
			pType->SetText(this->adamsType().c_str());
			xml_ele.InsertEndChild(pType);

			Aris::Core::XmlElement *pPrtI = xml_ele.GetDocument()->NewElement("iPart");
			pPrtI->SetText(makI().father().name().data());
			xml_ele.InsertEndChild(pPrtI);

			Aris::Core::XmlElement *pPrtJ = xml_ele.GetDocument()->NewElement("jPart");
			pPrtJ->SetText(makJ().father().name().data());
			xml_ele.InsertEndChild(pPrtJ);

			Aris::Core::XmlElement *pMakI = xml_ele.GetDocument()->NewElement("iMarker");
			pMakI->SetText(makI().name().data());
			xml_ele.InsertEndChild(pMakI);

			Aris::Core::XmlElement *pMakJ = xml_ele.GetDocument()->NewElement("jMarker");
			pMakJ->SetText(makJ().name().data());
			xml_ele.InsertEndChild(pMakJ);
		}
		void Constraint::toAdamsCmd(std::ofstream &file) const
		{
			file << "constraint create joint " << this->adamsType() << "  &\r\n"
				<< "    joint_name = ." << model().name() << "." << this->name() << "  &\r\n"
				<< "    adams_id = " << this->id() + 1 << "  &\r\n"
				<< "    i_marker_name = ." << model().name() << "." << this->makI().father().name() << "." << this->makI().name() << "  &\r\n"
				<< "    j_marker_name = ." << model().name() << "." << this->makJ().father().name() << "." << this->makJ().name() << "  \r\n"
				<< "!\r\n";
		}

		Marker::Marker(const Part &prt, const double *prt_pe, const char* eulType)
			: Element(const_cast<ModelBase &>(prt.model()), "", 0), prt_(prt)
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
			: Marker(prt.model(), prt, name, id)
		{
			static const double default_pm_in[16] = { 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
			pPrtPm = pPrtPm ? pPrtPm : default_pm_in;
			
			
			if (pRelativeTo)
			{
				if (&pRelativeTo->father() != &prt_)
					throw std::logic_error("relative marker must has same father part with this marker");
				
				s_pm_dot_pm(*pRelativeTo->prtPm(), pPrtPm, *prt_pm_);
			}
			else
			{
				std::copy_n(pPrtPm, 16, static_cast<double *>(*prt_pm_));
			}
		}
		Marker::Marker(Part &prt, const std::string &Name, int id, const Aris::Core::XmlElement *ele)
			: Marker(prt.model(), prt, Name, id)
		{
			double pm[16];

			Core::Matrix m = prt.model().calculator.CalculateExpression(ele->Attribute("Pos"));
			s_pe2pm(m.data(), pm);

			if (ele->Attribute("RelativeTo") && (!ele->Attribute("RelativeTo", "")))
			{
				const Marker *pRelativeMak = father().findMarker(ele->Attribute("RelativeTo"));
				s_pm_dot_pm(*pRelativeMak->prtPm(), pm, *prt_pm_);
			}
			else
			{
				std::copy_n(pm, 16, static_cast<double*>(*prt_pm_));
			}
		}
		const double6& Marker::vel() const { return father().vel(); };
		const double6& Marker::acc() const { return father().acc(); };
		void Marker::update()
		{
			s_pm_dot_pm(*father().pm(), *prtPm(), *pm_);
		}
		void Marker::toXmlElement(Aris::Core::XmlElement &xml_ele) const
		{
			double value[10];

			xml_ele.DeleteChildren();
			xml_ele.SetName(this->name().data());

			Aris::Core::XmlElement *pPE = xml_ele.GetDocument()->NewElement("Pos");
			s_pm2pe(*prtPm(), value);
			pPE->SetText(Core::Matrix(1, 6, value).toString().c_str());
			xml_ele.InsertEndChild(pPE);

			Aris::Core::XmlElement *pRelativeMakEle = xml_ele.GetDocument()->NewElement("RelativeTo");
			pRelativeMakEle->SetText("");
			xml_ele.InsertEndChild(pRelativeMakEle);
		}
		
		Part::Part(ModelBase &model, const std::string &Name, int id, const double *im, const double *pm_in, const double *vel_in, const double *acc_in)
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

			if (pm_in == nullptr)
			{
				std::fill_n(static_cast<double *>(*this->pm()), 16, 0);
				this->pm_[0][0] = 1;
				this->pm_[1][1] = 1;
				this->pm_[2][2] = 1;
				this->pm_[3][3] = 1;
			}
			else
			{
				setPm(pm_in);
			}

			if (vel_in == nullptr)
			{
				std::fill_n(vel(), 6, 0);
			}
			else
			{
				setVel(vel_in);
			}

			if (acc_in == nullptr)
			{
				std::fill_n(acc(), 6, 0);
			}
			else
			{
				setAcc(acc_in);
			}
		}
		Part::Part(ModelBase &model_in, const std::string &Name, int id, const Aris::Core::XmlElement *ele)
			: Marker(model_in, *this, Name, id)
		{
			if (ele->Attribute("Active", "true"))
			{
				this->activate();
			}
			else if (ele->Attribute("Active", "false"))
			{
				this->activate(false);
			}
			else
			{
				throw std::logic_error("failed load xml file in part");
			}

			Core::Matrix m;

			m = model().calculator.CalculateExpression(ele->Attribute("Inertia"));
			s_gamma2im(m.data(), *prt_im_);

			m = model().calculator.CalculateExpression(ele->Attribute("Pos"));
			s_pe2pm(m.data(), *pm());

			m = model().calculator.CalculateExpression(ele->Attribute("Vel"));
			std::copy_n(m.data(), 6, vel());

			m = model().calculator.CalculateExpression(ele->Attribute("Acc"));
			std::copy_n(m.data(), 6, acc());

			marker_names_.clear();

			for (auto makEle = ele->FirstChildElement("ChildMarker")->FirstChildElement(); makEle != nullptr; makEle = makEle->NextSiblingElement())
			{
				addMarker(makEle->name(), makEle);
			}

			if (ele->Attribute("Graphic_File_Path") && (!ele->Attribute("Graphic_File_Path", "")))
				graphic_file_path_ = ele->Attribute("Graphic_File_Path");
		}
		void Part::update()
		{
			double tem[6];
		
			s_inv_pm(*pm(), *inv_pm_);
			s_tv(*inv_pm_, vel(), prt_vel_);
			s_tv(*inv_pm_, acc(), prt_acc_);
			s_tv(*inv_pm_, model().environment().gravity_, prt_gravity_);
			s_m6_dot_v6(*prt_im_, prt_gravity_, prt_fg_);
			s_m6_dot_v6(*prt_im_, prt_vel_, tem);
			s_cf(prt_vel_, tem, prt_fv_);
		}
		Marker* Part::findMarker(const std::string &Name)
		{
			auto pMak = marker_names_.find(Name);
			if (pMak != marker_names_.end())
			{
				return model().markers_.at(pMak->second).get();
			}
			else
			{
				return nullptr;
			}
		}
		const Marker* Part::findMarker(const std::string &Name)const
		{
			auto pMak = marker_names_.find(Name);
			if (pMak != marker_names_.end())
			{
				return model().markers_.at(pMak->second).get();
			}
			else
			{
				return nullptr;
			}
		}
		void Part::toXmlElement(Aris::Core::XmlElement &xml_ele) const
		{
			double value[10];
			
			xml_ele.DeleteChildren();
			xml_ele.SetName(this->name().data());

			Aris::Core::XmlElement *pActive = xml_ele.GetDocument()->NewElement("Active");
			if (this->isActive())
				pActive->SetText("True");
			else
				pActive->SetText("False");
			xml_ele.InsertEndChild(pActive);
			
			Aris::Core::XmlElement *pInertia = xml_ele.GetDocument()->NewElement("Inertia");
			s_im2gamma(*this->prtIm(),value);
			pInertia->SetText(Core::Matrix(1,10,value).toString().c_str());
			xml_ele.InsertEndChild(pInertia);

			Aris::Core::XmlElement *pPE = xml_ele.GetDocument()->NewElement("Pos");
			s_pm2pe(*this->pm(), value);
			pPE->SetText(Core::Matrix(1, 6, value).toString().c_str());
			xml_ele.InsertEndChild(pPE);

			Aris::Core::XmlElement *pVel = xml_ele.GetDocument()->NewElement("Vel");
			pVel->SetText(Core::Matrix(1, 6, vel()).toString().c_str());
			xml_ele.InsertEndChild(pVel);

			Aris::Core::XmlElement *pAcc = xml_ele.GetDocument()->NewElement("Acc");
			pAcc->SetText(Core::Matrix(1, 6, acc()).toString().c_str());
			xml_ele.InsertEndChild(pAcc);

			Aris::Core::XmlElement *pChildMak = xml_ele.GetDocument()->NewElement("ChildMarker");
			xml_ele.InsertEndChild(pChildMak);

			for (auto &m : marker_names_)
			{
				Aris::Core::XmlElement *ele = xml_ele.GetDocument()->NewElement("");

				model().markers_.at(m.second)->toXmlElement(*ele);
				pChildMak->InsertEndChild(ele);
			}

			Aris::Core::XmlElement *pGraphicFilePath = xml_ele.GetDocument()->NewElement("Graphic_File_Path");
			pGraphicFilePath->SetText(this->graphic_file_path_.c_str());
			xml_ele.InsertEndChild(pGraphicFilePath);
		}
		void Part::toAdamsCmd(std::ofstream &file) const
		{
			if (this == model().pGround)
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
					<< "    default_coordinate_system = ." << model().name() << ".ground\r\n"
					<< "!\r\n"
					<< "! ****** Markers for current part ******\r\n"
					<< "!\r\n";
			}
			else
			{
				double pe[6];
				s_pm2pe(*this->pm(), pe, "313");
				Core::Matrix ori(1, 3, &pe[3]), loc(1, 3, &pe[0]);

				file << "!----------------------------------- " << this->name() << " -----------------------------------!\r\n"
					<< "!\r\n"
					<< "!\r\n"
					<< "defaults coordinate_system  &\r\n"
					<< "    default_coordinate_system = ." << model().name() << ".ground\r\n"
					<< "!\r\n"
					<< "part create rigid_body name_and_position  &\r\n"
					<< "    part_name = ." << model().name() << "." << this->name() << "  &\r\n"
					<< "    adams_id = " << this->id() + 1 << "  &\r\n"
					<< "    location = (" << loc.toString() << ")  &\r\n"
					<< "    orientation = (" << ori.toString() << ")\r\n"
					<< "!\r\n"
					<< "defaults coordinate_system  &\r\n"
					<< "    default_coordinate_system = ." << model().name() << "." << this->name() << " \r\n"
					<< "!\r\n";

				
				double mass = this->prtIm()[0][0] == 0 ? 1 : prtIm()[0][0];
				std::fill_n(pe, 6, 0);
				pe[0] = this->prtIm()[1][5] / mass;
				pe[1] = -this->prtIm()[0][5] / mass;
				pe[2] = this->prtIm()[0][4] / mass;				

				file << "! ****** cm and mass for current part ******\r\n"
					<< "marker create  &\r\n"
					<< "    marker_name = ." << model().name() << "." << this->name() << ".cm  &\r\n"
					<< "    adams_id = " << this->id() + model().markerNum() << "  &\r\n"
					<< "    location = ({" << pe[0] << "," << pe[1] << "," << pe[2] << "})  &\r\n"
					<< "    orientation = (" << "{0,0,0}" << ")\r\n"
					<< "!\r\n";

				double pm[16];
				double im[6][6];

				pe[0] = -pe[0];
				pe[1] = -pe[1];
				pe[2] = -pe[2];

				s_pe2pm(pe, pm);
				s_i2i(pm, *this->prtIm(), *im);

				/*！注意！*/
				//Adams里对惯量矩阵的定义貌似和我自己的定义在Ixy，Ixz，Iyz上互为相反数。别问我为什么，我也不知道。
				file << "part create rigid_body mass_properties  &\r\n"
					<< "    part_name = ." << model().name() << "." << this->name() << "  &\r\n"
					<< "    mass = " << this->prtIm()[0][0] << "  &\r\n"
					<< "    center_of_mass_marker = ." << model().name() << "." << this->name() << ".cm  &\r\n"
					<< "    inertia_marker = ." << model().name() << "." << this->name() << ".cm  &\r\n"
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

				s_pm2pe(*model().markerAt(mak.second).prtPm(), pe, "313");
				Core::Matrix ori(1, 3, &pe[3]), loc(1, 3, &pe[0]);

				file << "marker create  &\r\n"
					<< "marker_name = ." << model().name() << "." << this->name() << "." << mak.first << "  &\r\n"
					<< "adams_id = " << mak.second + 1 << "  &\r\n"
					<< "location = (" << loc.toString() << ")  &\r\n"
					<< "orientation = (" << ori.toString() << ")\r\n"
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
					<< "part_name = " << this->name() << " \r\n"
					<< "\r\n";
			}
		}

		MotionBase::MotionBase(ModelBase &model_in, const std::string &name, int id, Marker &makI, Marker &makJ)
			: Constraint(model_in, name, id, makI, makJ)
		{
			std::fill_n(static_cast<double*>(frc_coe_), 3, 0);
		}
		MotionBase::MotionBase(ModelBase &model_in, const std::string &name, int id, const Aris::Core::XmlElement &xml_ele)
			: Constraint(model_in, name, id, xml_ele)
		{
			if (strcmp("true", xml_ele.Attribute("Active")) == 0)
			{
				this->activate();
			}
			else if (strcmp("false", xml_ele.Attribute("Active")) == 0)
			{
				this->activate(false);
			}
			else
			{
				throw std::logic_error("failed load xml file in motion");
			}

			Core::Matrix m = model().calculator.CalculateExpression(xml_ele.Attribute("FrcCoe"));
			std::copy_n(m.data(), 3, static_cast<double*>(frc_coe_));
		}
		void MotionBase::toXmlElement(Aris::Core::XmlElement &xml_ele) const
		{
			xml_ele.DeleteChildren();
			xml_ele.SetName(this->name().data());

			Aris::Core::XmlElement *pActive = xml_ele.GetDocument()->NewElement("Active");
			if (this->isActive())
				pActive->SetText("True");
			else
				pActive->SetText("False");
			xml_ele.InsertEndChild(pActive);

			Aris::Core::XmlElement *pType = xml_ele.GetDocument()->NewElement("Type");
			pType->SetText(this->adamsType().c_str());
			xml_ele.InsertEndChild(pType);

			Aris::Core::XmlElement *pPrtI = xml_ele.GetDocument()->NewElement("iPart");
			pPrtI->SetText(makI().father().name().data());
			xml_ele.InsertEndChild(pPrtI);

			Aris::Core::XmlElement *pPrtJ = xml_ele.GetDocument()->NewElement("jPart");
			pPrtJ->SetText(makJ().father().name().data());
			xml_ele.InsertEndChild(pPrtJ);

			Aris::Core::XmlElement *pMakI = xml_ele.GetDocument()->NewElement("iMarker");
			pMakI->SetText(makI().name().data());
			xml_ele.InsertEndChild(pMakI);

			Aris::Core::XmlElement *pMakJ = xml_ele.GetDocument()->NewElement("jMarker");
			pMakJ->SetText(makJ().name().data());
			xml_ele.InsertEndChild(pMakJ);

			Aris::Core::XmlElement *pFrictionCoefficients = xml_ele.GetDocument()->NewElement("Friction_Coefficients");

			pFrictionCoefficients->SetText(Core::Matrix(1, 3, frcCoe()).toString().c_str());
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

		void Environment::toXmlElement(Aris::Core::XmlElement &xml_ele) const
		{
			xml_ele.DeleteChildren();
			xml_ele.SetName("Enviroment");

			Aris::Core::XmlElement *pGravity = xml_ele.GetDocument()->NewElement("Gravity");
			pGravity->SetText(Core::Matrix(1, 6, gravity_).toString().c_str());
			xml_ele.InsertEndChild(pGravity);
		}
		void Environment::fromXmlElement(const Aris::Core::XmlElement &xml_ele)
		{
			Core::Matrix m = model().calculator.CalculateExpression(xml_ele.FirstChildElement("Gravity")->GetText());
			memcpy(gravity_, m.data(), sizeof(gravity_));
		}
		void Environment::toAdamsCmd(std::ofstream &file) const
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
				<< "   model_name = " << this->model().name() << "\r\n"
				<< "!\r\n"
				<< "view erase\r\n"
				<< "!\r\n"
				<< "!---------------------------------- Accgrav -----------------------------------!\r\n"
				<< "!\r\n"
				<< "!\r\n"
				<< "force create body gravitational  &\r\n"
				<< "    gravity_field_name = gravity  &\r\n"
				<< "    x_component_gravity = " << this->gravity_[0] << "  &\r\n"
				<< "    y_component_gravity = " << this->gravity_[1] << "  &\r\n"
				<< "    z_component_gravity = " << this->gravity_[2] << "\r\n"
				<< "!\r\n";
		};

		class Script::Imp 
		{
		public:
			struct Node
			{
				virtual void DoNode() {};
				virtual void update() {};
				virtual std::uint32_t MsConsumed()const { return 0; };
				virtual std::string AdamsScript()const = 0;
			};
			struct ActivateNode final :public Node
			{
				virtual void DoNode()override { pEle->activate(isActive); };
				virtual std::string AdamsScript()const override
				{
					std::stringstream ss;
					std::string cmd = isActive ? "activate/" : "deactivate/";
					if (dynamic_cast<ForceBase*>(pEle))
					{
						ss << cmd << "sforce" << ", id=" << pEle->id() + 1;
					}
					else
					{
						ss << cmd << pEle->groupName() << ", id=" << pEle->id() + 1;
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
				void update() override 
				{
					double pm_target_g[16];

					s_pm_dot_pm(*mak_target_.father().pm(), *mak_target_.prtPm(), pm_target_g);
					s_inv_pm_dot_pm(*mak_move_.father().pm(), pm_target_g, &mak_move_.prt_pm_[0][0]);
					s_pm2pe(*mak_move_.prt_pm_, prtPe);
				};
				virtual std::string AdamsScript()const override
				{
					std::stringstream ss;
					ss << "marker/" << mak_move_.id() + 1
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

			void activate(Element &ele, bool isActive)
			{
				node_list_.push_back(std::unique_ptr<Node>(new ActivateNode(ele,isActive)));
			}
			void alignMarker(Marker &mak, const Marker& mak_target)
			{
				node_list_.push_back(std::unique_ptr<Node>(new MoveMarkerNode(mak, mak_target)));
			}
			void simulate(std::uint32_t ms_dur, std::uint32_t ms_dt)
			{
				node_list_.push_back(std::unique_ptr<Node>(new SimulateNode(ms_dur, ms_dt)));
			}
			void toAdamsCmd(std::ofstream &file) const
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
			std::int32_t endTime()const
			{
				std::uint32_t end_time{0};

				for (auto& node : node_list_)end_time += node->MsConsumed();
				
				return end_time;
			}
			void setTopologyAt(std::uint32_t ms_time) 
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
		void Script::activate(Element &ele, bool isActive) { pImp->activate(ele, isActive); };
		void Script::alignMarker(Marker &mak, const Marker& mak_target) { pImp->alignMarker(mak, mak_target); };
		void Script::simulate(std::uint32_t ms_dur, std::uint32_t ms_dt) { pImp->simulate(ms_dur, ms_dt); };
		void Script::toAdamsCmd(std::ofstream &file) const
		{
			pImp->toAdamsCmd(file);
		}
		bool Script::empty()const { return pImp->node_list_.empty(); };
		std::uint32_t Script::endTime()const 
		{
			return pImp->endTime();
		};
		void Script::setTopologyAt(std::uint32_t ms_time) { pImp->setTopologyAt(ms_time); };
		void Script::updateAt(std::uint32_t ms_time)
		{
			std::uint32_t now = 0;
			for (auto&node : pImp->node_list_)
			{
				now += node->MsConsumed();
				if (now == ms_time)node->update();
			}
		}
		void Script::clear() { pImp->node_list_.clear(); };

		ModelBase::ModelBase(const std::string & name)
			: Object(*this, name)
			, environment_(*this)
			, pGround(nullptr)
		{
			addPart("Ground");
			pGround = findPart("Ground");
		}
		ModelBase::~ModelBase()
		{
		}

		template<class T>
		typename T::value_type::element_type * FindElement(const T &container, const std::string &name)
		{
			auto p = std::find_if(container.begin(), container.end(), [name](typename T::const_reference p)
			{
				return (p->name() == name);
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
		Part *ModelBase::findPart(const std::string &Name)
		{
			return FindElement<decltype(parts_)>(parts_, Name);
		}
		JointBase *ModelBase::findJoint(const std::string &Name)
		{
			return FindElement<decltype(joints_)>(joints_, Name);
		}
		MotionBase *ModelBase::findMotion(const std::string &Name)
		{
			return FindElement<decltype(motions_)>(motions_, Name);
		}
		ForceBase *ModelBase::findForce(const std::string &Name)
		{
			return FindElement<decltype(forces_)>(forces_, Name);
		}

		void ModelBase::dynSetSolveMethod(std::function<void(int dim, const double *D, const double *b, double *x)> solve_method)
		{
			this->dyn_solve_method_ = solve_method;
		}
		void ModelBase::dynCstMtx(double *cst_mtx) const
		{
			std::fill_n(cst_mtx, dynDimN()*dynDimM(), 0);

			for (int i = 0; i < 6; ++i)
			{
				cst_mtx[dynDimN()*(pGround->row_id_ + i) + i] = 1;
			}

			for (auto &jnt : joints_)
			{
				if (jnt->isActive())
				{
					s_block_cpy(6, jnt->cstDim(), jnt->csmI(), 0, 0, jnt->cstDim(), cst_mtx, jnt->makI().father().row_id_, jnt->col_id_, dynDimN());
					s_block_cpy(6, jnt->cstDim(), jnt->csmJ(), 0, 0, jnt->cstDim(), cst_mtx, jnt->makJ().father().row_id_, jnt->col_id_, dynDimN());
				}
			}
			for (auto &mot : motions_)
			{
				if (mot->isActive())
				{
					s_block_cpy(6, 1, mot->csmI(), 0, 0, 1, cst_mtx, mot->makI().father().row_id_, mot->col_id_, dynDimN());
					s_block_cpy(6, 1, mot->csmJ(), 0, 0, 1, cst_mtx, mot->makJ().father().row_id_, mot->col_id_, dynDimN());
				}
			}
		}
		void ModelBase::dynIneMtx(double *ine_mtx) const
		{
			std::fill_n(ine_mtx, dynDimM()*dynDimM(), 0);

			for (int i = 0; i < 6; ++i)
			{
				ine_mtx[dynDimM()*(pGround->row_id_ + i) + pGround->row_id_ + i] = 1;
			}

			for (auto &prt : parts_)
			{
				if (prt->isActive())
				{
					s_block_cpy(6, 6, *(prt->prt_im_), 0, 0, 6, ine_mtx, prt->row_id_, prt->row_id_, dynDimM());
				}
			}
		}
		void ModelBase::dynCstAcc(double *cst_acc) const
		{
			std::fill_n(cst_acc, dynDimN(), 0);

			for (auto &jnt : joints_)
			{
				if (jnt->isActive())
				{
					std::copy_n(jnt->csa(), jnt->cstDim(), &cst_acc[jnt->col_id_]);
				}
			}
			for (auto &mot : motions_)
			{
				if (mot->isActive())
				{
					cst_acc[mot->col_id_] = *mot->csa();
				}
			}
		}
		void ModelBase::dynPrtFce(double *prt_fce) const
		{
			std::fill_n(prt_fce, dynDimM(), 0);

			for (auto &prt : parts_)
			{
				if (prt->isActive())
				{
					s_daxpy(6, -1, prt->prt_fg_, 1, &prt_fce[prt->row_id_], 1);
					s_daxpy(6, 1, prt->prt_fv_, 1, &prt_fce[prt->row_id_], 1);
				}
			}

			for (auto &fce : forces_)
			{
				if (fce->isActive())
				{
					s_daxpy(6, -1, fce->fceI(), 1, &prt_fce[fce->makI().father().row_id_], 1);
					s_daxpy(6, -1, fce->fceJ(), 1, &prt_fce[fce->makJ().father().row_id_], 1);
				}
			}
		}
		void ModelBase::dynCstFce(double *cst_fce) const
		{
			for (auto &jnt : joints_)
			{
				if (jnt->isActive())
				{
					std::copy_n(jnt->csf(), jnt->cstDim(), &cst_fce[jnt->col_id_]);
				}
			}
			for (auto &mot : motions_)
			{
				if (mot->isActive())
				{
					cst_fce[mot->col_id_] = mot->motFceDyn();
				}
			}

		}
		void ModelBase::dynPrtAcc(double *cst_acc) const
		{
			for (auto &prt : parts_)
			{
				if (prt->isActive())
				{
					std::copy_n(prt->prtAcc(), 6, &cst_acc[prt->row_id_]);
				}
			}
		}
		void ModelBase::dynPre()
		{
			int pid = 0;//part id
			int cid = 6;//JointBase id

			for (auto &part:parts_)
			{
				if (part->isActive())
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
				if (joint->isActive())
				{
					joint->init();
					joint->col_id_ = cid;
					cid += joint->cstDim();
				}
				else
				{
					joint->col_id_ = 0;
				}
			}
			for (auto &motion:motions_)
			{
				if (motion->isActive())
				{
					motion->col_id_ = cid;
					cid++;
					motion->init();
				}
				else
				{
					motion->col_id_ = 0;
					motion->init();
				}
			}

			dyn_prt_dim_ = pid;
			dyn_cst_dim_ = cid;
		}
		void ModelBase::dynUpd()
		{
			for (auto &prt : parts_)
			{
				if (prt->isActive())prt->update();
			}
			for (auto &jnt : joints_)
			{
				if (jnt->isActive())jnt->update();
			}
			for (auto &mot : motions_)
			{
				if (mot->isActive())mot->update();
				//std::cout << *mot->csa() << std::endl;
			}
			for (auto &fce : forces_)
			{
				if (fce->isActive())fce->update();
			}
		}
		void ModelBase::dynMtx(double *D, double *b) const
		{
			dynCstMtx(&D[(dynDim())*dynDimM()]);
			s_block_cpy(dynDimM(), dynDimN(), &D[(dynDim())*dynDimM()], 0, 0, dynDimN(), D, 0, dynDimM(), dynDim());
			
			dynIneMtx(&D[(dynDim())*dynDimM()]);
			s_block_cpy(dynDimM(), dynDimM(), -1, &D[(dynDim())*dynDimM()], 0, 0, dynDimM(), 0, D, 0, 0, dynDim());

			std::fill_n(&D[(dynDim())*dynDimM()], dynDimN()*(dynDim()), 0);
			s_block_cpyT(dynDimM(), dynDimN(), D, 0, dynDimM(), dynDim(), D, dynDimM(), 0, dynDim());

			dynPrtFce(b);
			dynCstAcc(b + dynDimM());
		}
		void ModelBase::dynSov(const double *D, const double *b, double *x) const
		{
			if (dyn_solve_method_)
			{
				dyn_solve_method_(dynDim(), D, b, x);
			}
			else
			{
				throw std::runtime_error("please set solve_method before use DynSov");
			}
		}
		void ModelBase::dynEnd(const double *x)
		{
			for (auto &prt : parts_)
			{
				if (prt->isActive())
				{
					std::copy_n(&x[prt->row_id_], 6, prt->prt_acc_);
				}
			}
			for (auto &jnt : joints_)
			{
				if (jnt->isActive())
				{
					std::copy_n(&x[jnt->col_id_ + dynDimM()], jnt->cstDim(), jnt->csf());
				}
			}
			for (auto &mot : motions_)
			{
				if (mot->isActive())
				{
					mot->mot_fce_dyn_ = x[mot->col_id_ + dynDimM()];
				}
			}
		}
		void ModelBase::dynUkn(double *x) const
		{
			this->dynPrtAcc(x);
			this->dynCstFce(x + dynDimM());
		}
		void ModelBase::dyn()
		{
			/*
			DynPre();
			
			std::vector<double> C(dynDimM() * dynDimN());
			std::vector<double> a_c(dynDimN());
			std::vector<double> I_mat(dynDimM() * dynDimM());
			std::vector<double> f(dynDimM());
			std::vector<double> D((dynDim()) * (dynDim()));
			std::vector<double> b(dynDim());

			DynMtx(C.data(), a_c.data(), I_mat.data(), f.data(), D.data(), b.data());

			std::vector<double> x(dynDim());
			solveMethod(dynDim(), D.data(), b.data(), x.data());

			DynEnd(x.data());
			*/
			dynPre();
			std::vector<double> D(dynDim() * dynDim());
			std::vector<double> b(dynDim());
			std::vector<double> x(dynDim());
			dynUpd();
			dynMtx(D.data(), b.data());
			dynSov(D.data(), b.data(), x.data());
			dynEnd(x.data());
		}

		void ModelBase::clbSetInverseMethod(std::function<void(int n, double *A)> inverse_method)
		{
			this->clb_inverse_method_ = inverse_method;
		}
		void ModelBase::clbPre()
		{
			dynPre();

			if (dynDimN() != dynDimM())
			{
				throw std::logic_error("must calibrate square matrix");
			}

			clb_dim_m_ = 0;
			clb_dim_n_ = 0;
			clb_dim_gam_ = 0;
			clb_dim_frc_ = 0;

			for (auto &i : motions_)
			{
				if (i->isActive())
				{
					clb_dim_m_++;
					clb_dim_frc_ += 3;
					clb_dim_n_ += 3;
				}
			}
			for (auto &i : parts_)
			{
				if (i->isActive())
				{
					clb_dim_n_ += 10;
					clb_dim_gam_ += 10;
				}
			}

		}
		void ModelBase::clbUpd()
		{
			dynUpd();
		}
		void ModelBase::clbMtx(double *clb_D, double *clb_b)const
		{
			if (!clb_inverse_method_)throw std::runtime_error("please set inverse method before calibrate");
			if (dynDimN() != dynDimM()) throw std::logic_error("must calibrate square matrix");

			/*初始化*/
			static Core::Matrix clb_d_m, clb_b_m;

			clb_d_m.resize(clbDimM(), clbDimN());
			clb_b_m.resize(clbDimM(), 1);

			memset(clb_d_m.data(), 0, clb_d_m.length() * sizeof(double));
			memset(clb_b_m.data(), 0, clb_b_m.length() * sizeof(double));

			/*求A，即C的逆*/
			Core::Matrix A(dynDimM(), dynDimM()), B(dynDimM(), dynDimM());

			std::vector<double> C(dynDimM() * dynDimM());
			std::vector<double> f(dynDimM());

			dynCstMtx(C.data());
			std::copy(C.begin(), C.end(), A.data());
			clb_inverse_method_(dynDimM(), A.data());

			/*求B*/
			const int beginRow = dynDimM() - clbDimM();

			for (auto &i:parts_)
			{
				if (i->isActive())
				{
					double cm[6][6];
					s_cmf(i->prtVel(), *cm);
					s_dgemm(clbDimM(), 6, 6, 1, &A(beginRow,i->row_id_), dynDimM(), *cm, 6, 0, &B(beginRow, i->row_id_), dynDimM());
				}
			}

			/*求解clb_d*/
			int col1 = 0, col2 = 0;

			for (auto &i:parts_)
			{
				if (i->isActive())
				{
					double q[6]{0};
					std::copy_n(i->prtAcc(), 6, q);
					s_daxpy(6, -1, i->prtGravity(), 1, q, 1);
					
					double v[6];
					std::copy_n(i->prtVel(), 6, v);

					for (int j = 0; j < clbDimM(); ++j)
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
				if (mot->isActive())
				{
					clb_b_m(row, 0) = mot->mot_fce_;
					++row;
				}
			}
			for (auto &fce : forces_)
			{
				if (fce->isActive())
				{
					s_daxpy(6, 1, fce->fceI(), 1, &f[fce->makI().father().row_id_], 1);
					s_daxpy(6, 1, fce->fceJ(), 1, &f[fce->makJ().father().row_id_], 1);
				}
			}
			s_dgemm(clbDimM(), 1, dynDimM(), 1, &A(beginRow,0), dynDimM(), f.data(), 1, 1, clb_b_m.data(), 1);

			/*以下添加驱动摩擦系数*/
			row = 0;
			for (auto &mot : motions_)
			{
				//默认未激活的motion处于力控模式
				if (mot->isActive())
				{
					clb_d_m(row, clbDimGam() + row * 3) += s_sgn(mot->motVel());
					clb_d_m(row, clbDimGam() + row * 3 + 1) += mot->motVel();
					clb_d_m(row, clbDimGam() + row * 3 + 2) += mot->motAcc();
					++row;
				}
			}

			std::copy_n(clb_d_m.data(), clb_d_m.length(), clb_D);
			std::copy_n(clb_b_m.data(), clb_b_m.length(), clb_b);
		}
		void ModelBase::clbUkn(double *clb_x)const
		{
			int row = 0;
			for (auto &prt : parts_)
			{
				if (prt->isActive())
				{
					s_im2gamma(*prt->prtIm(), clb_x + row);
					row += 10;
				}
			}

			for (auto &mot : motions_)
			{
				if (mot->isActive())
				{
					std::copy_n(mot->frcCoe(), 3, clb_x + row);
					row += 3;
				}
			}
		}
		
		void ModelBase::simKin(const PlanFunc &func, const PlanParamBase &param, SimResult &result, bool using_script)
		{
			//储存起始的每个杆件的位置
			result.begin_prt_pe_.clear();
			for (std::size_t i = 0; i < partNum(); ++i)
			{
				std::array<double, 6>pe;
				partAt(i).getPe(pe.data());
				result.begin_prt_pe_.push_back(pe);
			}

			//初始化变量
			result.Pin_.clear();
			result.Pin_.resize(this->motionNum());

			//起始位置
			result.time_.push_back(0);
			for (std::size_t i = 0; i < motionNum(); ++i)
			{
				motionAt(i).update();
				result.Pin_.at(i).push_back(motionAt(i).motPos());
			}

			if (using_script)
			{
				this->script().updateAt(0);
				this->script().setTopologyAt(0);
			}

			//其他位置
			for (param.count = 0; true; ++param.count)
			{
				auto is_sim = func(*this, param);

				result.time_.push_back(param.count + 1);
				for (std::size_t i = 0; i < motionNum(); ++i)
				{
					result.Pin_.at(i).push_back(motionAt(i).motPos());
				}

				if (using_script)
				{
					this->script().updateAt(param.count + 1);
					this->script().setTopologyAt(param.count + 1);
				}

				if (!is_sim)break;
			}

			//恢复起始位置
			auto pe = result.begin_prt_pe_.begin();
			for (std::size_t i = 0; i < partNum(); ++i)
			{
				partAt(i).setPe(pe->data());
				++pe;
			}
		}
		void ModelBase::simDyn(const PlanFunc &func, const PlanParamBase &param, SimResult &result, bool using_script)
		{
			
		}
		void ModelBase::simKinAkima(const PlanFunc &func, const PlanParamBase &param, SimResult &result, int akima_interval, bool using_script)
		{
			simKin(func, param, result, using_script);
			
			//生成Akima
			std::list<double> time_akima_data;
			std::vector<std::list<double> > pos_akima_data(motionNum());

			//初始化迭代器
			auto p_time = result.time_.begin();
			std::vector<std::list<double>::iterator> p_Pin(motionNum());
			for (std::size_t i = 0; i < motionNum(); ++i)
				p_Pin.at(i) = result.Pin_.at(i).begin();

			//生成Akima
			for (std::size_t i = 0; i < result.time_.size(); ++i)
			{
				if ((i == (result.time_.size() - 1)) || (i%akima_interval == 0))
				{
					time_akima_data.push_back(*p_time / 1000.0);

					for (std::size_t j = 0; j < motionNum(); ++j)
					{
						pos_akima_data.at(j).push_back(*p_Pin.at(j));
					}
				}

				++p_time;
				for (auto &j : p_Pin)++j;
			}

			//设置Akima
			for (std::size_t i = 0; i < motionNum(); ++i)
			{
				motionAt(i).posCurve.reset(new Akima(time_akima_data, pos_akima_data.at(i)));
			}
		}
		void ModelBase::simDynAkima(const PlanFunc &func, const PlanParamBase &param, SimResult &result, int akima_interval, bool using_script)
		{
			simKinAkima(func, param, result, akima_interval, using_script);
			
			result.Pin_.clear();
			result.Fin_.clear();
			result.Ain_.clear();
			result.Vin_.clear();

			result.Pin_.resize(motionNum());
			result.Fin_.resize(motionNum());
			result.Vin_.resize(motionNum());
			result.Ain_.resize(motionNum());

			//仿真计算
			for (std::size_t t = 0; t < result.time_.size();++t)
			{
				std::cout << t << std::endl;
				
				if (using_script)
				{
					this->script().setTopologyAt(t);
				}
				for (std::size_t j = 0; j < motionNum(); ++j)
				{
					motionAt(j).mot_pos_ = motionAt(j).posAkima(t / 1000.0, '0');
				}
				kinFromPin();
				for (std::size_t j = 0; j < motionNum(); ++j)
				{
					motionAt(j).mot_vel_ = motionAt(j).posAkima(t / 1000.0, '1');
				}
				kinFromVin();
				for (std::size_t j = 0; j < motionNum(); ++j)
				{
					motionAt(j).mot_acc_ = motionAt(j).posAkima(t / 1000.0, '2');
				}
				dyn();
				for (std::size_t j = 0; j < motionNum(); ++j)
				{
					result.Fin_.at(j).push_back(motionAt(j).mot_fce_dyn_);
					result.Pin_.at(j).push_back(motionAt(j).motPos());
					result.Vin_.at(j).push_back(motionAt(j).motVel());
					result.Ain_.at(j).push_back(motionAt(j).motAcc());
				}
			}

			//恢复起始位置
			auto pe = result.begin_prt_pe_.begin();
			for (std::size_t i = 0; i < partNum(); ++i)
			{
				partAt(i).setPe(pe->data());
				++pe;
			}
		}
		void ModelBase::simToAdams(const std::string &adams_file, const PlanFunc &fun, const PlanParamBase &param, int ms_dt, bool using_script)
		{
			SimResult result;
			simKinAkima(fun, param, result, ms_dt, using_script);
			this->saveAdams(adams_file, using_script);
		}

		void ModelBase::loadXml(const std::string &filename)
		{
			Aris::Core::XmlDocument xmlDoc;
			
			if (xmlDoc.LoadFile(filename.c_str()) != 0)
			{
				throw std::logic_error((std::string("could not open file:") + std::string(filename)));
			}

			loadXml(xmlDoc);
		}
		void ModelBase::loadXml(const Aris::Core::XmlDocument &xml_doc)
		{
			auto pModel = xml_doc.RootElement()->FirstChildElement("Model");

			if (!pModel)throw std::logic_error("can't find Model element in xml file");

			loadXml(*pModel);
		}
		void ModelBase::loadXml(const Aris::Core::XmlElement &xml_ele)
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
				calculator.AddVariable(ele->name(), calculator.CalculateExpression(ele->GetText()));
			}

			environment().fromXmlElement(*pEnv);

			parts_.clear();
			joints_.clear();
			motions_.clear();
			forces_.clear();

			/*读入地面*/
			for (auto ele = pPrt->FirstChildElement(); ele != nullptr; ele = ele->NextSiblingElement())
			{
				if (std::string(ele->name()) == "Ground")
				{
					addPart(ele->name(), ele);
					pGround = findPart("Ground");
					break;
				}
			}

			if (this->findPart("Ground") == nullptr)
			{
				throw std::logic_error("Model must contain a Ground");
			}

			/*读入其他部件*/
			for (auto ele = pPrt->FirstChildElement(); ele != nullptr; ele = ele->NextSiblingElement())
			{
				if (std::string(ele->name()) != "Ground")
				{
					addPart(ele->name(), ele);
				}
			}
		}
		void ModelBase::saveSnapshotXml(const char *filename) const
		{
			Aris::Core::XmlDocument XML_Doc;
			XML_Doc.DeleteChildren();

			Aris::Core::XmlDeclaration *pHeader = XML_Doc.NewDeclaration("xml version=\"1.0\" encoding=\"UTF-8\" ");
			XML_Doc.InsertFirstChild(pHeader);

			Aris::Core::XmlElement *pModel = XML_Doc.NewElement("Model");
			XML_Doc.InsertEndChild(pModel);

			Aris::Core::XmlElement *pEnvironment = XML_Doc.NewElement("");
			environment().toXmlElement(*pEnvironment);
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
				p->toXmlElement(*ele);
				pPrt->InsertEndChild(ele);
			}

			for (auto &j : joints_)
			{
				Aris::Core::XmlElement *ele = XML_Doc.NewElement("");
				j->toXmlElement(*ele);
				pJnt->InsertEndChild(ele);
			}

			for (auto &m : motions_)
			{
				Aris::Core::XmlElement *ele = XML_Doc.NewElement("");
				m->toXmlElement(*ele);
				pMot->InsertEndChild(ele);
			}


			XML_Doc.SaveFile(filename);
		}
		void ModelBase::saveAdams(const std::string &filename, bool using_script) const
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
			environment().toAdamsCmd(file);

			file << "!----------------------------------- Parts -------------------------------------!\r\n!\r\n!\r\n";
			pGround->toAdamsCmd(file);
			for (auto &prt : parts_)
			{
				if (prt.get() != pGround)
				{
					prt->toAdamsCmd(file);
				}
			}

			file << "!----------------------------------- Joints ------------------------------------!\r\n!\r\n!\r\n";
			for (auto &jnt : joints_)
			{
				jnt->toAdamsCmd(file);
			}

			file << "!----------------------------------- Motions -----------------------------------!\r\n!\r\n!\r\n";
			for (auto &mot : motions_)
			{
				mot->toAdamsCmd(file);
			}

			file << "!----------------------------------- Forces ------------------------------------!\r\n!\r\n!\r\n";
			for (auto &fce : forces_)
			{
				fce->toAdamsCmd(file);
			}

			if (using_script)
			{
				file << "!----------------------------------- Script ------------------------------------!\r\n!\r\n!\r\n";
				this->script().toAdamsCmd(file);
			}
			else
			{
				file << "!----------------------------------- Motify Active -------------------------------------!\r\n!\r\n!\r\n";
				for (auto &prt : parts_)
				{
					if ((prt.get() != pGround) && (!prt->isActive()))
					{
						file << "part attributes  &\r\n"
							<< "    part_name = ." << name() << "." << prt->name() << "  &\r\n"
							<< "    active = off \r\n!\r\n";
					}
				}
				for (auto &jnt : joints_)
				{
					if (!jnt->isActive())
					{
						file << "constraint attributes  &\r\n"
							<< "    constraint_name = ." << name() << "." << jnt->name() << "  &\r\n"
							<< "    active = off \r\n!\r\n";
					}
					
				}
				for (auto &mot : motions_)
				{
					if (!mot->isActive())
					{
						file << "constraint attributes  &\r\n"
							<< "    constraint_name = ." << name() << "." << mot->name() << "  &\r\n"
							<< "    active = off \r\n!\r\n";
					}
					
				}
				for (auto &fce : forces_)
				{
					if (!fce->isActive())
					{
						file << "force attributes  &\r\n"
							<< "    force_name = ." << name() << "." << fce->name() << "  &\r\n"
							<< "    active = off \r\n!\r\n";
					}
				}
			}
			
			file.close();


		}
	}
}
