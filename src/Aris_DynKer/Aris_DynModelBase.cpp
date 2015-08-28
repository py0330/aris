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
		PART::PART(MODEL *pModel, const string &Name, int id, const double *Im, const double *pm, const double *Vel, const double *Acc)
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
		MARKER* PART::AddMarker(const std::string &Name, const double *pm, MARKER *pRelativeTo)
		{
			if (_markerNames.find(Name) != _markerNames.end())
			{
				return nullptr;
			}
			
			Model()->_markers.push_back(std::unique_ptr<MARKER>(new MARKER(Model(), Name, Model()->_markers.size(), this, pm, pRelativeTo)));
			_markerNames[Name] = Model()->_markers.size() - 1;
			return Model()->_markers.back().get();
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
		void PART::FromXmlElement(const Aris::Core::ELEMENT *pEle)
		{
			SetName(pEle->Name());

			if (strcmp("True", pEle->FirstChildElement("Active")->GetText()) == 0)
			{
				this->Activate();
				//this->Active() = true;
			}
			else if (strcmp("False", pEle->FirstChildElement("Active")->GetText()) == 0)
			{
				this->Deactivate();
			}
			else
			{
				return;
			}

			MATRIX m;

			m = Model()->calculator.CalculateExpression(pEle->FirstChildElement("Inertia")->GetText());
			s_gamma2im(m.Data(), *_PrtIm);

			m = Model()->calculator.CalculateExpression(pEle->FirstChildElement("Pos")->GetText());
			s_pe2pm(m.Data(), *_Pm);

			m = Model()->calculator.CalculateExpression(pEle->FirstChildElement("Vel")->GetText());
			memcpy(_Vel, m.Data(), sizeof(_Vel));

			m = Model()->calculator.CalculateExpression(pEle->FirstChildElement("Acc")->GetText());
			memcpy(_Acc, m.Data(), sizeof(_Acc));

			_markerNames.clear();

			for (const Aris::Core::ELEMENT *ele = pEle->FirstChildElement("ChildMarker")->FirstChildElement(); ele != 0; ele = ele->NextSiblingElement())
			{
				AddMarker(ele->Name())->FromXmlElement(ele);
			}

			if (pEle->FirstChildElement("Graphic_File_Path")->GetText() != nullptr)
				graphicFilePath = pEle->FirstChildElement("Graphic_File_Path")->GetText();
		}

		MARKER::MARKER(MODEL *pModel, const string &Name, int id, PART* pPart, const double *pLocPm, MARKER *pRelativeTo)
			: ELEMENT(pModel, Name, id)
			, _pPrt(pPart)
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
		void MARKER::FromXmlElement(const Aris::Core::ELEMENT *pEle)
		{
			double pm[4][4];

			SetName(pEle->Name());

			MATRIX m = Model()->calculator.CalculateExpression(pEle->FirstChildElement("Pos")->GetText());
			s_pe2pm(m.Data(), *pm);

			if (pEle->FirstChildElement("RelativeTo")->GetText() != nullptr)
			{
				MARKER *pRelativeMak = _pPrt->GetMarker(pEle->FirstChildElement("RelativeTo")->GetText());
				s_pm_dot_pm(pRelativeMak->GetPrtPmPtr(), *pm, *_PrtPm);
			}
			else
			{
				memcpy(_PrtPm, *pm, sizeof(_PrtPm));
			}
		}

		JOINT_BASE::JOINT_BASE(MODEL *pModel, const std::string &Name, int id, MARKER *pMakI, MARKER *pMakJ
			, double *PrtCstMtxI, double *PrtCstMtxJ, double *CstFce, double *a_c)
			: ELEMENT(pModel, Name, id)
			, _PrtCstMtxIPtr(PrtCstMtxI)
			, _PrtCstMtxJPtr(PrtCstMtxJ)
			, _CstFcePtr(CstFce)
			, _a_cPtr(a_c)
		{

		}
		void JOINT_BASE::Initiate()
		{
			std::fill_n(_PrtCstMtxIPtr, this->GetCstDim() * 6, 0);
			std::fill_n(_PrtCstMtxJPtr, this->GetCstDim() * 6, 0);

			s_tmf(_pMakI->GetPrtPmPtr(), *_tm_I2M);
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
		void JOINT_BASE::FromXmlElement(const Aris::Core::ELEMENT *pEle)
		{
			SetName(pEle->Name());

			if (strcmp("True", pEle->FirstChildElement("Active")->GetText()) == 0)
			{
				this->Activate();
			}
			else if (strcmp("False", pEle->FirstChildElement("Active")->GetText()) == 0)
			{
				this->Deactivate();
			}
			else
			{
				throw std::logic_error("failed load xml file in joint");
			}

			_pMakI = Model()->GetPart(pEle->FirstChildElement("iPart")->GetText())->GetMarker(pEle->FirstChildElement("iMarker")->GetText());
			_pMakJ = Model()->GetPart(pEle->FirstChildElement("jPart")->GetText())->GetMarker(pEle->FirstChildElement("jMarker")->GetText());
		}
		
		MOTION_BASE::MOTION_BASE(MODEL *pModel, const std::string &Name, int id, MOTION_BASE::MOTION_MODE mode, MARKER *pMakI, MARKER *pMakJ)
			: ELEMENT(pModel, Name, id)
			, _Mode(mode)
			, _pMakI(pMakI)
			, _pMakJ(pMakJ)
		{
			memset(_frc_coe, 0, sizeof(double) * 3);
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

			Aris::Core::ELEMENT *pMode = pEle->GetDocument()->NewElement("Mode");
			switch (this->_Mode)
			{
			case MOTION_BASE::POS_CONTROL:
				pMode->SetText("Pos_Control");
				break;
			case MOTION_BASE::FCE_CONTROL:
				pMode->SetText("Fce_Control");
				break;
			}
			pEle->InsertEndChild(pMode);

			Aris::Core::ELEMENT *pFrictionCoefficients = pEle->GetDocument()->NewElement("Friction_Coefficients");

			pFrictionCoefficients->SetText(MATRIX(1, 3, _frc_coe).ToString().c_str());
			pEle->InsertEndChild(pFrictionCoefficients);
		}
		void MOTION_BASE::FromXmlElement(const Aris::Core::ELEMENT *pEle)
		{
			SetName(pEle->Name());

			if (strcmp("True", pEle->FirstChildElement("Active")->GetText()) == 0)
			{
				this->Activate();
			}
			else if (strcmp("False", pEle->FirstChildElement("Active")->GetText()) == 0)
			{
				this->Deactivate();
			}
			else
			{
				throw std::logic_error("failed load xml file in motion");
			}

			if (strcmp(pEle->FirstChildElement("Mode")->GetText(), "Pos_Control") == 0)
			{
				_Mode = POS_CONTROL;
			}
			else if (strcmp(pEle->FirstChildElement("Mode")->GetText(), "Fce_Control") == 0)
			{
				_Mode = FCE_CONTROL;
			}

			MATRIX m = Model()->calculator.CalculateExpression(pEle->FirstChildElement("Friction_Coefficients")->GetText());
			memcpy(_frc_coe, m.Data(), sizeof(_frc_coe));

			_pMakI = Model()->GetPart(pEle->FirstChildElement("iPart")->GetText())->GetMarker(pEle->FirstChildElement("iMarker")->GetText());
			_pMakJ = Model()->GetPart(pEle->FirstChildElement("jPart")->GetText())->GetMarker(pEle->FirstChildElement("jMarker")->GetText());
		}

		FORCE_BASE::FORCE_BASE(MODEL *pModel, const std::string &Name, int id, PART *pPrtI, PART *pPrtJ)
			: ELEMENT(pModel, Name, id)
			, _pPrtM(pPrtI)
			, _pPrtN(pPrtJ)
		{
		}

		ENVIRONMENT::ENVIRONMENT(MODEL *pModel)
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

		MODEL::MODEL(const std::string & Name)
			: OBJECT(this , Name)
			, _Environment(this)
			, pGround(nullptr)
		{
			AddPart("Ground");
			pGround = GetPart("Ground");
		}
		MODEL::~MODEL()
		{
		}

		PART* MODEL::AddPart(const std::string & Name, const double *Im, const double *pm, const double *Vel, const double *Acc)
		{
			if (GetPart(Name)!=nullptr)
			{
				return nullptr;
			}
			
			_parts.push_back(std::unique_ptr<PART>(new PART(this, Name, _parts.size(), Im, pm, Vel, Acc)));
			return _parts.back().get();
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

		const PART *MODEL::GetPart(int id) const
		{
			return _parts.at(id).get();
		}
		const JOINT_BASE *MODEL::GetJoint(int id)const
		{
			return _joints.at(id).get();
		}
		const MOTION_BASE *MODEL::GetMotion(int id)const
		{
			return _motions.at(id).get();
		}
		const FORCE_BASE *MODEL::GetForce(int id)const
		{
			return _forces.at(id).get();
		}
		const MARKER *MODEL::GetMarker(int id)const
		{
			return _markers.at(id).get();
		}
		PART *MODEL::GetPart(int id)
		{
			return _parts.at(id).get();
		}
		JOINT_BASE *MODEL::GetJoint(int id)
		{
			return _joints.at(id).get();
		}
		MOTION_BASE *MODEL::GetMotion(int id)
		{
			return _motions.at(id).get();
		}
		FORCE_BASE *MODEL::GetForce(int id)
		{
			return _forces.at(id).get();
		}
		MARKER *MODEL::GetMarker(int id)
		{
			return _markers.at(id).get();
		}
		const PART *MODEL::GetPart(const std::string &Name)const
		{
			return GetContent<decltype(_parts)>(_parts, Name);
		}
		const JOINT_BASE *MODEL::GetJoint(const std::string &Name)const
		{
			return GetContent<decltype(_joints)>(_joints, Name);
		}
		const MOTION_BASE *MODEL::GetMotion(const std::string &Name)const
		{
			return GetContent<decltype(_motions)>(_motions, Name);
		}
		const FORCE_BASE *MODEL::GetForce(const std::string &Name)const
		{
			return GetContent<decltype(_forces)>(_forces, Name);
		}
		PART *MODEL::GetPart(const std::string &Name)
		{
			return GetContent<decltype(_parts)>(_parts, Name);
		}
		JOINT_BASE *MODEL::GetJoint(const std::string &Name)
		{
			return GetContent<decltype(_joints)>(_joints, Name);
		}
		MOTION_BASE *MODEL::GetMotion(const std::string &Name)
		{
			return GetContent<decltype(_motions)>(_motions, Name);
		}
		FORCE_BASE *MODEL::GetForce(const std::string &Name)
		{
			return GetContent<decltype(_forces)>(_forces, Name);
		}

		void MODEL::DynPre()
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
				if ((motion->Active()) && (motion->_Mode == MOTION_BASE::POS_CONTROL))
				{
					motion->_ColId = cid;
					cid += motion->GetCstDim();
					motion->_Initiate();
				}
				else
				{
					motion->_ColId = 0;
					motion->_Initiate();
				}
			}

			I_dim = pid;
			C_dim = cid;

			_C.resize(C_dim*I_dim);
			C = _C.data();
			memset(C, 0, sizeof(double)*I_dim*C_dim);

			_I.resize(I_dim*I_dim);
			pI = _I.data();
			memset(pI, 0, sizeof(double)*I_dim*I_dim);
				
			_f.resize(I_dim);
			f = _f.data();
			memset(f, 0, sizeof(double)*I_dim);

			_a_c.resize(C_dim);
			a_c = _a_c.data();
			memset(a_c, 0, sizeof(double)*C_dim);
				
			_D.resize((I_dim + C_dim)*(I_dim + C_dim));
			D = _D.data();
			memset(D, 0, sizeof(double)*(I_dim + C_dim)*(I_dim + C_dim));

			_b.resize(I_dim + C_dim);
			b = _b.data();
			memset(b, 0, sizeof(double)*(I_dim + C_dim));

			_s.resize(I_dim + C_dim);
			s = _s.data();
			memset(s, 0, sizeof(double)*(I_dim + C_dim));

			_x.resize(I_dim + C_dim);
			x = _x.data();
			memset(x, 0, sizeof(double)*(I_dim + C_dim));

			for (int i = 0; i < 6; ++i)
			{
				pI[I_dim*pGround->_RowId + pGround->_RowId] = 1;
				C[C_dim*(pGround->_RowId + i) + i] = 1;
			}

		}
		void MODEL::DynPrtMtx()
		{
			memset(f, 0, I_dim*sizeof(double));
			memset(D, 0, (C_dim + I_dim)*(C_dim + I_dim)*sizeof(double));
			/*Update pI,and fces*/
			for (auto &p:_parts)
			{
				if (p->Active())
				{
					p->Update();
					s_block_cpy(6, 6, *(p->_PrtIm), 0, 0, 6, pI, p->_RowId, p->_RowId, I_dim);

					s_daxpy(6, -1, p->_PrtFg, 1, &f[p->_RowId], 1);
					s_daxpy(6, 1, p->_PrtFv, 1, &f[p->_RowId], 1);
				}
			}
			/*Update C , a_c and force-controlled-motion force*/
			for (auto &j:_joints)
			{
				if (j->Active())
				{
					j->Update();

					s_block_cpy(6, j->GetCstDim(), j->GetPrtCstMtxIPtr(), 0, 0, j->GetCstDim(), C, j->_pMakI->GetFatherPrt()->_RowId, j->_ColId, C_dim);
					s_block_cpy(6, j->GetCstDim(), j->GetPrtCstMtxJPtr(), 0, 0, j->GetCstDim(), C, j->_pMakJ->GetFatherPrt()->_RowId, j->_ColId, C_dim);

					memcpy(&a_c[j->_ColId], j->GetPrtA_cPtr(), j->GetCstDim() * sizeof(double));
				}
			}
			for (auto &m : _motions)
			{
				if (m->Active())
				{
					double tem_f[6];
					m->Update();
					switch (m->_Mode)
					{
					case MOTION_BASE::POS_CONTROL:
						s_block_cpy(6, m->GetCstDim(), m->GetPrtCstMtxIPtr(), 0, 0, 6, C, m->_pMakI->GetFatherPrt()->_RowId, m->_ColId, C_dim);
						s_block_cpy(6, m->GetCstDim(), m->GetPrtCstMtxJPtr(), 0, 0, 6, C, m->_pMakJ->GetFatherPrt()->_RowId, m->_ColId, C_dim);

						memcpy(&a_c[m->_ColId], m->GetPrtA_cPtr(), m->GetCstDim() * sizeof(double));
						break;
					case MOTION_BASE::FCE_CONTROL:
						/*补偿摩擦力*/
						for (int j = 0; j < m->GetCstDim(); j++)
						{
							tem_f[j] = m->_f_m[j]
								- s_sgn(m->GetV_mPtr()[j]) * m->_frc_coe[0]
								- m->GetV_mPtr()[j] * m->_frc_coe[1]
								- m->GetA_mPtr()[j] * m->_frc_coe[2];
						}
						/*补偿摩擦力完毕*/
						s_dgemm(6, 1, m->GetCstDim(), -1, m->GetPrtCstMtxIPtr(), 6, tem_f, 1, 1, &f[m->_pMakI->GetFatherPrt()->_RowId], 1);
						s_dgemm(6, 1, m->GetCstDim(), -1, m->GetPrtCstMtxJPtr(), 6, tem_f, 1, 1, &f[m->_pMakJ->GetFatherPrt()->_RowId], 1);
						break;
					}
				}
			}

			/*calculate D and b*/
			/* for D*/
			for (int i = 0; i < 6; ++i)
			{
				D[(C_dim + I_dim)*(pGround->_RowId + i) + (i + I_dim)] = 1;
				D[(C_dim + I_dim)*(i + I_dim) + (pGround->_RowId + i)] = 1;
			}

			for (auto &p : _parts)
			{
				if (p->Active())
				{
					for (int i = 0; i < 6; ++i)
					{
						for (int j = 0; j < 6; ++j)
						{
							D[(I_dim + C_dim)*(p->_RowId + i) + (p->_RowId + j)]
								= -pI[I_dim*(p->_RowId + i) + (p->_RowId + j)];
						}
					}
				}
			}

			for (auto &jnt : _joints)
			{
				if (jnt->Active())
				{
					for (int i = 0; i < 6; i++)
					{
						for (int j = 0; j < jnt->GetCstDim(); j++)
						{
							D[(C_dim + I_dim)*(jnt->_pMakI->GetFatherPrt()->_RowId + i) + (I_dim + jnt->_ColId + j)]
								= C[C_dim*(jnt->_pMakI->GetFatherPrt()->_RowId + i) + jnt->_ColId + j];
							D[(C_dim + I_dim)*(jnt->_pMakJ->GetFatherPrt()->_RowId + i) + (I_dim + jnt->_ColId + j)]
								= C[C_dim*(jnt->_pMakJ->GetFatherPrt()->_RowId + i) + jnt->_ColId + j];
							D[(jnt->_pMakI->GetFatherPrt()->_RowId + i) + (C_dim + I_dim)*(I_dim + jnt->_ColId + j)]
								= C[C_dim*(jnt->_pMakI->GetFatherPrt()->_RowId + i) + jnt->_ColId + j];
							D[(jnt->_pMakJ->GetFatherPrt()->_RowId + i) + (C_dim + I_dim)*(I_dim + jnt->_ColId + j)]
								= C[C_dim*(jnt->_pMakJ->GetFatherPrt()->_RowId + i) + jnt->_ColId + j];
						}
					}

				}
			}

			for (auto &m : _motions)
			{
				if (m->Active())
				{
					switch (m->_Mode)
					{
					case MOTION_BASE::POS_CONTROL:
						for (int i = 0; i < 6; i++)
						{
							for (int j = 0; j < m->GetCstDim(); j++)
							{
								D[(C_dim + I_dim)*(m->_pMakI->GetFatherPrt()->_RowId + i) + (I_dim + m->_ColId + j)]
									= C[C_dim*(m->_pMakI->GetFatherPrt()->_RowId + i) + m->_ColId + j];
								D[(C_dim + I_dim)*(m->_pMakJ->GetFatherPrt()->_RowId + i) + (I_dim + m->_ColId + j)]
									= C[C_dim*(m->_pMakJ->GetFatherPrt()->_RowId + i) + m->_ColId + j];
								D[(m->_pMakI->GetFatherPrt()->_RowId + i) + (C_dim + I_dim)*(I_dim + m->_ColId + j)]
									= C[C_dim*(m->_pMakI->GetFatherPrt()->_RowId + i) + m->_ColId + j];
								D[(m->_pMakJ->GetFatherPrt()->_RowId + i) + (C_dim + I_dim)*(I_dim + m->_ColId + j)]
									= C[C_dim*(m->_pMakJ->GetFatherPrt()->_RowId + i) + m->_ColId + j];
							}
						}
						break;
					case MOTION_BASE::FCE_CONTROL:

						break;
					}
				}
			}

			s_block_cpy(I_dim, 1, f, 0, 0, 1, b, 0, 0, 1);
			s_block_cpy(C_dim, 1, a_c, 0, 0, 1, b, I_dim, 0, 1);


			/*以下求解*/
			memcpy(x, b, (C_dim + I_dim)*sizeof(double));
		}
		void MODEL::Dyn()
		{
			double rcond = 0.000000000001;
			int rank;
			s_dgelsd(C_dim + I_dim,
				C_dim + I_dim,
				1,
				D,
				C_dim + I_dim,
				x,
				1,
				s,
				rcond,
				&rank);

			for (auto &p:_parts)
			{
				if (p->Active())
				{
					//p->SetPrtAcc(&x[p->_RowId]);
					memcpy(p->GetPrtAccPtr(), &x[p->_RowId], sizeof(double) * 6);
				}
			}
			for (auto &j : _joints)
			{
				if (j->Active())
				{
					memcpy(j->_CstFcePtr, &x[j->_ColId + I_dim], j->GetCstDim() * sizeof(double));
				}
			}
			for (auto &m:_motions)
			{
				if (m->Active())
				{
					switch (m->_Mode)
					{
					case MOTION_BASE::POS_CONTROL:
						/*补偿摩擦力*/
						/*x[m->_ColId + I_dim] += m->_frc_coe[0] * s_sgn(*m->GetV_mPtr())
							+ m->_frc_coe[1] * (*m->GetV_mPtr())
							+ m->_frc_coe[2] * (*m->GetA_mPtr());*/

						memcpy(m->_f_m, &x[m->_ColId + I_dim], m->GetCstDim() * sizeof(double));
						/*补偿摩擦力*/
						*m->_f_m += m->_frc_coe[0] * s_sgn(*m->GetV_mPtr())
							+ m->_frc_coe[1] * (*m->GetV_mPtr())
							+ m->_frc_coe[2] * (*m->GetA_mPtr());
						break;
					case MOTION_BASE::FCE_CONTROL:
						break;
					}
				}
			}

		}

		void MODEL::ClbEqnTo(double *&clb_d_ptr, double *&clb_b_ptr, int &clb_dim_m, int &clb_dim_n)
		{
			this->DynPre();
			if (C_dim != I_dim)
			{
				throw std::logic_error("must calibrate square matrix");
			}
			
			int dim = I_dim;

			static MATRIX _clb_d;
			static MATRIX _clb_b;

			/*初始化*/
			clb_dim_m = 0;
			clb_dim_n = 0;
			int clb_prt_dim_n = 0;
			for (auto &i : _motions)
			{
				if (i->Active() && (i->GetMode() == MOTION_BASE::POS_CONTROL))
				{
					clb_dim_m += i->GetCstDim();
				}

				clb_dim_n += 3 * i->GetCstDim();
			}
			
			for (auto &i : _parts)//不算地面
			{
				if (i->Active())
				{
					clb_dim_n += 10;
					clb_prt_dim_n += 10;
				}
			}

			_clb_d.Resize(clb_dim_m, clb_dim_n);
			_clb_b.Resize(clb_dim_m, 1);

			memset(_clb_d.Data(), 0, _clb_d.Length() * sizeof(double));
			memset(_clb_b.Data(), 0, _clb_b.Length() * sizeof(double));

			/*开始计算*/
			memset(C, 0, dim*dim * sizeof(double));

			/*Update all*/
			for (auto &i:_parts)
			{
				if (i->Active())
				{
					i->Update();
				}
			}
			for (auto &i : _joints)
			{
				if (i->Active())
				{
					i->Update();
				}
			}

			/*计算C以及f*/
			for (auto &j : _joints)
			{
				if (j->Active())
				{
					j->Update();

					s_block_cpy(6, j->GetCstDim(), j->GetPrtCstMtxIPtr(), 0, 0, j->GetCstDim(), C, j->_pMakI->GetFatherPrt()->_RowId, j->_ColId, dim);
					s_block_cpy(6, j->GetCstDim(), j->GetPrtCstMtxJPtr(), 0, 0, j->GetCstDim(), C, j->_pMakJ->GetFatherPrt()->_RowId, j->_ColId, dim);

					memcpy(&a_c[j->_ColId], j->GetPrtA_cPtr(), j->GetCstDim() * sizeof(double));
				}
			}
			for (auto &m : _motions)
			{
				if (m->Active())
				{
					m->Update();
					switch (m->_Mode)
					{
					case MOTION_BASE::POS_CONTROL:
						s_block_cpy(6, m->GetCstDim(), m->GetPrtCstMtxIPtr(), 0, 0, 6, C, m->_pMakI->GetFatherPrt()->_RowId, m->_ColId, dim);
						s_block_cpy(6, m->GetCstDim(), m->GetPrtCstMtxJPtr(), 0, 0, 6, C, m->_pMakJ->GetFatherPrt()->_RowId, m->_ColId, dim);
						break;
					case MOTION_BASE::FCE_CONTROL:
						s_dgemm(6, 1, m->GetCstDim(), 1, m->GetPrtCstMtxIPtr(), 6, m->_f_m, 1, 1, &f[m->_pMakI->GetFatherPrt()->_RowId], 1);
						s_dgemm(6, 1, m->GetCstDim(), 1, m->GetPrtCstMtxJPtr(), 6, m->_f_m, 1, 1, &f[m->_pMakJ->GetFatherPrt()->_RowId], 1);
						break;
					}
				}
			}
			for (int i = 0; i < 6; ++i)
			{
				C[dim*(pGround->_RowId + i) + i] = 1;
			}

			/*求解C的逆，即A*/
			MATRIX A(dim, dim), B(dim, dim);
			std::vector<int> ipiv(dim);

			memcpy(A.Data(), C, sizeof(double)*A.Length());
			s_dgeinv(dim, A.Data(), dim, ipiv.data());

			

			/*求解B*/
			int beginRow = dim - clb_dim_m;

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
					double q[6], v[6];

					memset(q, 0, sizeof(double) * 6);

					memcpy(q, i->GetPrtAccPtr(), sizeof(double) * 6);
					s_daxpy(6, -1, i->GetPrtGravityPtr(), 1, q, 1);

					memcpy(v, i->GetPrtVelPtr(), sizeof(double) * 6);

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
			int row = 0;
			for (auto &i : _motions)
			{
				if ((i->Active()) && (i->_Mode == MOTION_BASE::POS_CONTROL))
				{
					memcpy(&_clb_b(row, 0), i->_f_m, sizeof(double)*i->GetCstDim());
					row += i->GetCstDim();
				}
			}

			s_dgemm(clb_dim_m, 1, dim, 1, &A(beginRow,0), dim, f, 1, 1, _clb_b.Data(), 1);

			//dsp(f, dim, 1);


			/*以下添加驱动摩擦系数*/
			row = 0;
			int num = 0;
			for (auto &i:_motions)
			{
				if (i->Active())
				{
					if (i->_Mode == MOTION_BASE::POS_CONTROL)
					{

						_clb_d(row, clb_prt_dim_n + num * 3) += s_sgn(*i->GetV_mPtr());
						_clb_d(row, clb_prt_dim_n + num * 3 + 1) += *i->GetV_mPtr();
						_clb_d(row, clb_prt_dim_n + num * 3 + 2) += *i->GetA_mPtr();
						++row;
					}
					else
					{
						s_dgemm(clb_dim_m, 1, 6, s_sgn(*i->GetV_mPtr()), &A(beginRow, i->_pMakI->_pPrt->_RowId), dim, i->_PrtCstMtxI[0], 6, 1, &_clb_d(0, clb_prt_dim_n + num * 3), clb_dim_n);
						s_dgemm(clb_dim_m, 1, 6, s_sgn(*i->GetV_mPtr()), &A(beginRow, i->_pMakJ->_pPrt->_RowId), dim, i->_PrtCstMtxJ[0], 6, 1, &_clb_d(0, clb_prt_dim_n + num * 3), clb_dim_n);
						s_dgemm(clb_dim_m, 1, 6, *i->GetV_mPtr(), &A(beginRow, i->_pMakI->_pPrt->_RowId), dim, i->_PrtCstMtxI[0], 6, 1, &_clb_d(0, clb_prt_dim_n + num * 3 + 1), clb_dim_n);
						s_dgemm(clb_dim_m, 1, 6, *i->GetV_mPtr(), &A(beginRow, i->_pMakJ->_pPrt->_RowId), dim, i->_PrtCstMtxJ[0], 6, 1, &_clb_d(0, clb_prt_dim_n + num * 3 + 1), clb_dim_n);
						s_dgemm(clb_dim_m, 1, 6, *i->GetA_mPtr(), &A(beginRow, i->_pMakI->_pPrt->_RowId), dim, i->_PrtCstMtxI[0], 6, 1, &_clb_d(0, clb_prt_dim_n + num * 3 + 2), clb_dim_n);
						s_dgemm(clb_dim_m, 1, 6, *i->GetA_mPtr(), &A(beginRow, i->_pMakJ->_pPrt->_RowId), dim, i->_PrtCstMtxJ[0], 6, 1, &_clb_d(0, clb_prt_dim_n + num * 3 + 2), clb_dim_n);
					}
				}
				num++;
			}


			clb_d_ptr = _clb_d.Data();
			clb_b_ptr = _clb_b.Data();
		}

		void MODEL::LoadXml(const char *filename)
		{
			Aris::Core::DOCUMENT XML_Doc;
			
			if (XML_Doc.LoadFile(filename) != 0)
			{
				throw std::logic_error((string("could not open file:") + string(filename)));
			}

			const Aris::Core::ELEMENT *pModel = XML_Doc.RootElement()->FirstChildElement("Model");
			
			FromXmlElement(pModel);
		}
		void MODEL::FromXmlElement(const Aris::Core::ELEMENT *pModel)
		{
			if (pModel == nullptr)throw(std::logic_error("XML file must have model element"));

			const Aris::Core::ELEMENT *pVar = pModel->FirstChildElement("Variable");
			if (pModel == nullptr)throw(std::logic_error("Model must have variable element"));
			const Aris::Core::ELEMENT *pEnv = pModel->FirstChildElement("Environment");
			if (pEnv == nullptr)throw(std::logic_error("Model must have environment element"));
			const Aris::Core::ELEMENT *pPrt = pModel->FirstChildElement("Part");
			if (pPrt == nullptr)throw(std::logic_error("Model must have part element"));
			const Aris::Core::ELEMENT *pJnt = pModel->FirstChildElement("Joint");
			if (pJnt == nullptr)throw(std::logic_error("Model must have joint element"));
			const Aris::Core::ELEMENT *pMot = pModel->FirstChildElement("Motion");
			if (pMot == nullptr)throw(std::logic_error("Model must have motion element"));
			const Aris::Core::ELEMENT *pFce = pModel->FirstChildElement("Force");
			if (pFce == nullptr)throw(std::logic_error("Model must have force element"));

			calculator.ClearVariables();
			for (const Aris::Core::ELEMENT *ele = pVar->FirstChildElement();
			ele != nullptr;
				ele = ele->NextSiblingElement())
			{
				calculator.AddVariable(ele->Name(), calculator.CalculateExpression(ele->GetText()));
			}

			_Environment.FromXmlElement(pEnv);

			_parts.clear();
			_joints.clear();
			_motions.clear();
			_forces.clear();

			/*读入地面*/
			for (const Aris::Core::ELEMENT *ele = pPrt->FirstChildElement();
				ele != nullptr;
				ele = ele->NextSiblingElement())
			{
				if (std::string(ele->Name()) == "Ground")
				{
					AddPart(ele->Name());
					GetPart(ele->Name())->FromXmlElement(ele);
					pGround = GetPart("Ground");
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
					AddPart(ele->Name());
					GetPart(ele->Name())->FromXmlElement(ele);
				}
			}

			for (auto ele = pJnt->FirstChildElement(); ele != nullptr; ele = ele->NextSiblingElement())
			{
				if (strcmp(TRANSLATIONAL_JOINT::type, ele->FirstChildElement("Type")->GetText()) == 0)
				{
					AddJoint<TRANSLATIONAL_JOINT>(ele->Name(), nullptr, nullptr);
				}
				if (strcmp(SPHERICAL_JOINT::type, ele->FirstChildElement("Type")->GetText()) == 0)
				{
					AddJoint<SPHERICAL_JOINT>(ele->Name(), nullptr, nullptr);
				}
				if (strcmp(UNIVERSAL_JOINT::type, ele->FirstChildElement("Type")->GetText()) == 0)
				{
					AddJoint<UNIVERSAL_JOINT>(ele->Name(), nullptr, nullptr);
				}
				

				GetJoint(ele->Name())->FromXmlElement(ele);
				GetJoint(ele->Name())->Initiate();
			}

			for (auto ele = pMot->FirstChildElement(); ele != nullptr; ele = ele->NextSiblingElement())
			{
				if (strcmp(LINEAR_MOTION::type, ele->FirstChildElement("Type")->GetText()) == 0)
				{
					AddMotion<LINEAR_MOTION>(ele->Name(), MOTION_BASE::POS_CONTROL, nullptr, nullptr);
				}
				GetMotion(ele->Name())->FromXmlElement(ele);
				GetMotion(ele->Name())->_Initiate();
			}
		}
		void MODEL::SaveSnapshotXml(const char *filename) const
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
		void MODEL::SaveAdams(const char *filename, SIMULATE_SCRIPT* pScript) const
		{
			ofstream file;

			const int maxPath = 1000;

			std::string cmdName = std::string(filename) + std::string(".cmd");
			std::string acfName = std::string(filename) + std::string(".acf");
			std::string fullAcfPath;
			/*******写acf文件********/
			if (pScript)
			{
				file.open(acfName, std::ios::out | std::ios::trunc);
				file.close();
				char fullPath[maxPath];
#ifdef PLATFORM_IS_WINDOWS
				_fullpath(fullPath, acfName.c_str(), maxPath);
#endif
#ifdef PLATFORM_IS_LINUX
				if(realpath(acfName.c_str(), fullPath)==nullptr)
					throw std::logic_error("can't identify realpath");
#endif
				fullAcfPath = std::string(fullPath);

				/*创建acf文件*/
				file.open(acfName, std::ios::out | std::ios::trunc);

				file << setprecision(15);

				double beginTime = 0;
				double endTime = pScript->endTime/1000.0;
				double dt = pScript->dt/1000.0;


				file << "\r\n\r\n";
				/*设置起始的关节和电机状态*/
				if (std::find_if(_joints.begin(), _joints.end(), [](const std::unique_ptr<JOINT_BASE> &j)
				{
					return (j->Active()==false);
				}) != _joints.end())
				{
					file << "deactivate/joint, id= ";

					bool isFirst = true;
					for (auto &j : _joints)
					{
						if (j->Active() == false)
						{
							if (isFirst)
							{
								file << j->GetID() + 1;
								isFirst = false;
							}
							else
							{
								file << "," << j->GetID() + 1;
							}
						}

					}
					file << "\r\n";
				};
				if (std::find_if(_motions.begin(), _motions.end(), [](const std::unique_ptr<MOTION_BASE> &m)
					{
						return (m->GetMode() == MOTION_BASE::FCE_CONTROL);
					}) != _motions.end())
				{
					file << "deactivate/motion, id= ";

					bool isFirst = true;
					for (auto &m : _motions)
					{
						if (m->GetMode() == MOTION_BASE::FCE_CONTROL)
						{
							if (isFirst)
							{
								file << m->GetID() + 1;
								isFirst = false;
							}
							else
							{
								file << "," << m->GetID() + 1;
							}
						}
						
					}
					file << "\r\n";
				};
				if (std::find_if(_motions.begin(), _motions.end(), [](const std::unique_ptr<MOTION_BASE> &m)
					{
						return (m->GetMode() == MOTION_BASE::POS_CONTROL);
					}) != _motions.end())
				{
					file << "deactivate/sforce, id= ";

					bool isFirst = true;
					for (auto &m : _motions)
					{
						if (m->GetMode() == MOTION_BASE::POS_CONTROL)
						{
							if (isFirst)
							{
								file << m->GetID() + 1;
								isFirst = false;
							}
							else
							{
								file << "," << m->GetID() + 1;
							}
						}
						
					}
					file << "\r\n";
				};

				/*设置脚本*/
				for (auto &i : pScript->script)
				{
					double now = double(i.first) / 1000.0;
					if (now > beginTime)
					{
						file << "simulate/transient, dur=" << now - beginTime << ", dtout=" << dt << "\r\n";
						beginTime = now;
					}

					for (auto &j : i.second.joints)
					{
						JOINT_BASE *pJnt = j.first;

						if (j.second.active)
						{
							file << "activate/joint, id=" << pJnt->GetID() + 1 << "\r\n";
							if (j.second.isModifyMakI)
							{
								file << "marker/" << pJnt->GetMakI()->GetID() + 1 << ", QP=" << j.second.peMakI[0] << "," << j.second.peMakI[1] << "," << j.second.peMakI[2] << "\r\n";
								file << "marker/" << pJnt->GetMakI()->GetID() + 1 << ", REULER=" << j.second.peMakI[3] << "," << j.second.peMakI[4] << "," << j.second.peMakI[5] << "\r\n";
							}
							if (j.second.isModifyMakJ)
							{
								file << "marker/" << pJnt->GetMakJ()->GetID() + 1 << ", QP=" << j.second.peMakJ[0] << "," << j.second.peMakJ[1] << "," << j.second.peMakJ[2] << "\r\n";
								file << "marker/" << pJnt->GetMakJ()->GetID() + 1 << ", REULER=" << j.second.peMakJ[3] << "," << j.second.peMakJ[4] << "," << j.second.peMakJ[5] << "\r\n";
							}

						}
						else
						{
							file << "deactivate/joint, id=" << pJnt->GetID() + 1 << "\r\n";
						}
					}

					for (auto &j : i.second.motions)
					{
						if (j.second == MOTION_BASE::FCE_CONTROL)
						{
							file << "deactivate/motion, id= " << j.first->GetID() + 1 << "\r\n";
							file << "activate/sforce, id= " << j.first->GetID() + 1 << "\r\n";
						}
						else
						{
							file << "activate/motion, id= " << j.first->GetID() + 1 << "\r\n";
							file << "deactivate/sforce, id= " << j.first->GetID() + 1 << "\r\n";
						}
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
			file << "!-------------------------------- Rigid Parts ---------------------------------!\r\n"
				<< "!\r\n"
				<< "! Create parts and their dependent markers and graphics\r\n"
				<< "!\r\n"
				<< "!----------------------------------- ground -----------------------------------!\r\n"
				<< "!\r\n"
				<< "!\r\n"
				<< "! ****** Ground Part ******\r\n"
				<< "!\r\n"
				<< "defaults model  &\r\n"
				<< "    part_name = ground\r\n"
				<< "!\r\n"
				<< "defaults coordinate_system  &\r\n"
				<< "    default_coordinate_system = ." << this->Name() << ".ground\r\n"
				<< "!\r\n"
				<< "! ****** Markers for current part ******\r\n"
				<< "!\r\n";

			for (auto &i : pGround->_markerNames)
			{
				double pe[6];

				s_pm2pe(_markers.at(i.second)->GetPrtPmPtr(), pe, "313");
				MATRIX ori(1,3,&pe[3]),loc(1, 3, &pe[0]);
				
				file << "marker create  &\r\n"
					<< "    marker_name = ." << this->Name() << ".ground." << _markers.at(i.second)->Name() << "  &\r\n"
					<< "    adams_id = " << i.second + 1 << "  &\r\n"
					<< "    location = (" << loc.ToString() << ")  &\r\n"
					<< "    orientation = (" << ori.ToString() << ") \r\n"
					<< "!\r\n";
			}
			
			for (auto &i : _parts)
			{
				if (i.get() == pGround)
					continue;

				double pe[6];

				s_pm2pe(i->GetPmPtr(), pe, "313");
				MATRIX ori(1, 3, &pe[3]), loc(1, 3, &pe[0]);

				file << "!----------------------------------- " << i->Name() << " -----------------------------------!\r\n"
					<< "!\r\n"
					<< "!\r\n"
					<< "defaults coordinate_system  &\r\n"
					<< "    default_coordinate_system = ." << Name() << ".ground\r\n"
					<< "!\r\n"
					<< "part create rigid_body name_and_position  &\r\n"
					<< "    part_name = ." << Name() << "." << i->Name() << "  &\r\n"
					<< "    adams_id = " << i->GetID()+1 << "  &\r\n"
					<< "    location = (" << loc.ToString() << ")  &\r\n"
					<< "    orientation = (" << ori.ToString() << ")\r\n"
					<< "!\r\n"
					<< "defaults coordinate_system  &\r\n"
					<< "    default_coordinate_system = ." << Name() << "." << i->Name() << " \r\n"
					<< "!\r\n";

				double mass = 1;
				if (i->GetPrtImPtr()[0] != 0)
					mass = i->GetPrtImPtr()[0];

				file << "! ****** Markers for current part ******\r\n"
					<< "marker create  &\r\n"
					<< "    marker_name = ." << Name() << "." << i->Name() << ".cm  &\r\n"
					<< "    adams_id = " << i->GetID() + _markers.size() << "  &\r\n"
					<< "    location = ({" << i->GetPrtImPtr()[11] / mass << "," << -i->GetPrtImPtr()[5] / mass << "," << -i->GetPrtImPtr()[4] / mass << "})  &\r\n"
					<< "    orientation = (" << "{0,0,0}" << ")\r\n"
					<< "!\r\n";

				for (auto &j : i->_markerNames)
				{
					double pe[6];

					s_pm2pe(_markers.at(j.second)->GetPrtPmPtr(), pe, "313");
					MATRIX ori(1, 3, &pe[3]), loc(1, 3, &pe[0]);
					
					file << "marker create  &\r\n"
						<< "marker_name = ." << Name() << "." << i->Name() << "." << _markers.at(j.second)->Name() <<"  &\r\n"
						<< "adams_id = " << j.second + 1 << "  &\r\n"
						<< "location = (" << loc.ToString() << ")  &\r\n"
						<< "orientation = (" << ori.ToString() << ")\r\n"
						<< "!\r\n";
				}


				file << "part create rigid_body mass_properties  &\r\n"
					<< "    part_name = ." << Name() << "." << i->Name() << "  &\r\n"
					<< "    mass = " << i->GetPrtImPtr()[0] << "  &\r\n"
					<< "    center_of_mass_marker = ." << Name() << "." << i->Name() << ".cm  &\r\n"
					<< "    inertia_marker = ." << Name() << "." << i->Name() << ".cm  &\r\n"
					<< "    ixx = " << i->GetPrtImPtr()[21] << "  &\r\n"
					<< "    iyy = " << i->GetPrtImPtr()[28] << "  &\r\n"
					<< "    izz = " << i->GetPrtImPtr()[35] << "  &\r\n"
					<< "    ixy = " << i->GetPrtImPtr()[27] << "  &\r\n"
					<< "    izx = " << i->GetPrtImPtr()[33] << "  &\r\n"
					<< "    iyz = " << i->GetPrtImPtr()[34] << "\r\n"
					<< "!\r\n";

				std::stringstream stream(i->graphicFilePath);

				string path;
				while (stream >> path)
				{
					file << "file parasolid read &\r\n"
						<< "file_name = \"" << path << "\" &\r\n"
						<< "type = ASCII" << " &\r\n"
						<< "part_name = " << i->Name() << " \r\n"
						<< "\r\n";
				}


			}

			file << "!----------------------------------- Joints -----------------------------------!\r\n"
				<< "!\r\n"
				<< "!\r\n";

			for (auto &i : _joints)
			{
				double pe[6] = { 0, 0, 0, PI / 2, 0, 0 };
				double pe2[6] = { 0, 0, 0, -PI / 2, 0, 0 };
				double pm[4][4], pm2[4][4];

				if (strcmp(i->GetType(), "universal") == 0)
				{
					s_pe2pm(pe, *pm, "213");
					s_pm_dot_pm(i->_pMakI->GetPrtPmPtr(), *pm, *pm2);
					s_pm2pe(*pm2, pe, "313");

					file << "marker modify &\r\n"
						<< "    marker_name = ." << Name() << "." << i->GetMakI()->GetFatherPrt()->Name() << "." << i->GetMakI()->Name() << " &\r\n"
						<< "    orientation = (" << MATRIX(1, 3, &pe[3]).ToString() << ") \r\n"
						<< "!\r\n";

					s_pe2pm(pe2, *pm, "123");
					s_pm_dot_pm(i->_pMakJ->GetPrtPmPtr(), *pm, *pm2);
					s_pm2pe(*pm2, pe, "313");

					file << "marker modify &\r\n"
						<< "    marker_name = ." << Name() << "." << i->GetMakJ()->GetFatherPrt()->Name() << "." << i->GetMakJ()->Name() << " &\r\n"
						<< "    orientation = (" << MATRIX(1, 3, &pe[3]).ToString() << ") \r\n"
						<< "!\r\n";
				}

				file << "constraint create joint " << i->GetType() << "  &\r\n"
					<< "    joint_name = ." << Name() << "." << i->Name() << "  &\r\n"
					<< "    adams_id = " << i->GetID() + 1 << "  &\r\n"
					<< "    i_marker_name = ." << Name() << "." << i->_pMakI->_pPrt->Name() << "." << i->_pMakI->Name() << "  &\r\n"
					<< "    j_marker_name = ." << Name() << "." << i->_pMakJ->_pPrt->Name() << "." << i->_pMakJ->Name() << "  \r\n"
					<< "!\r\n";

				/*如果有脚本，那么关节的active在脚本里设置*/
				if (pScript == nullptr)
				{
					if (i->Active() == false)
					{
						file << "constraint attributes  &\r\n"
							<< "constraint_name = ." << Name() << "." << i->Name() << "  &\r\n"
							<< "active = off \r\n"
							<< "!\r\n";
					}
				}
			}

			file << "!----------------------------------- Motions -----------------------------------!\r\n"
				<< "!\r\n"
				<< "!\r\n";

			for (auto &i : _motions)
			{
				std::string s;
				s = "z";

				if (i->posCurve == nullptr)
				{
					file << "constraint create motion_generator &\r\n"
						<< "    motion_name = ." << Name() << "." << i->Name() << "  &\r\n"
						<< "    adams_id = " << i->GetID() + 1 << "  &\r\n"
						<< "    i_marker_name = ." << Name() << "." << i->_pMakI->_pPrt->Name() << "." << i->_pMakI->Name() << "  &\r\n"
						<< "    j_marker_name = ." << Name() << "." << i->_pMakJ->_pPrt->Name() << "." << i->_pMakJ->Name() << "  &\r\n"
						<< "    axis = " << s << "  &\r\n"
						<< "    function = \"" << *i->GetP_mPtr() << "\"  \r\n"
						<< "!\r\n";
				}
				else
				{
					file << "data_element create spline &\r\n"
						<< "    spline_name = ." << Name() << "." << i->Name() << "_pos_spl  &\r\n"
						<< "    adams_id = " << i->GetID() * 2 << "  &\r\n"
						<< "    units = m &\r\n"
						<< "    x = " << i->posCurve->x().at(0);
					for (auto p = i->posCurve->x().begin() + 1; p < i->posCurve->x().end();++p)
					{
						file << "," << *p;
					}
					file << "    y = " << i->posCurve->y().at(0);
					for (auto p = i->posCurve->y().begin() + 1; p < i->posCurve->y().end(); ++p)
					{
						file << "," << *p;
					}
					file << " \r\n!\r\n";

					file << "constraint create motion_generator &\r\n"
						<< "    motion_name = ." << Name() << "." << i->Name() << "  &\r\n"
						<< "    adams_id = " << i->GetID() + 1 << "  &\r\n"
						<< "    i_marker_name = ." << Name() << "." << i->_pMakI->_pPrt->Name() << "." << i->_pMakI->Name() << "  &\r\n"
						<< "    j_marker_name = ." << Name() << "." << i->_pMakJ->_pPrt->Name() << "." << i->_pMakJ->Name() << "  &\r\n"
						<< "    axis = " << s << "  &\r\n"
						<< "    function = \"AKISPL(time,0," << i->Name() << "_pos_spl)\"  \r\n"
						<< "!\r\n";
				}

				if (i->fceCurve == nullptr)
				{
					std::string type = "translational";

					file << "force create direct single_component_force  &\r\n"
						<< "    single_component_force_name = ." << Name() << "." << i->Name() << "_fce  &\r\n"
						<< "    adams_id = " << i->GetID() + 1 << "  &\r\n"
						<< "    type_of_freedom = " << type << "  &\r\n"
						<< "    i_marker_name = ." << Name() << "." << i->_pMakI->_pPrt->Name() << "." << i->_pMakI->Name() << "  &\r\n"
						<< "    j_marker_name = ." << Name() << "." << i->_pMakJ->_pPrt->Name() << "." << i->_pMakJ->Name() << "  &\r\n"
						<< "    action_only = off  &\r\n"
						<< "    function = \"" << *i->GetF_mPtr() << "\"  \r\n"
						<< "!\r\n";
				}
				else
				{
					std::string type = "translational";

					file << "data_element create spline &\r\n"
						<< "    spline_name = ." << Name() << "." << i->Name() << "_fce_spl  &\r\n"
						<< "    adams_id = " << i->GetID() * 2 + 1 << "  &\r\n"
						<< "    units = N &\r\n"
						<< "    x = " << i->fceCurve->x().at(0);
					for (auto p = i->fceCurve->x().begin() + 1; p < i->fceCurve->x().end(); ++p)
					{
						file << "," << *p;
					}
					file << "    y = " << i->fceCurve->y().at(0);
					for (auto p = i->fceCurve->y().begin() + 1; p < i->fceCurve->y().end(); ++p)
					{
						file << "," << *p;
					}
					file << " \r\n!\r\n";

					file << "force create direct single_component_force  &\r\n"
						<< "    single_component_force_name = ." << Name() << "." << i->Name() << "_fce  &\r\n"
						<< "    adams_id = " << i->GetID() + 1 << "  &\r\n"
						<< "    type_of_freedom = " << type << "  &\r\n"
						<< "    i_marker_name = ." << Name() << "." << i->_pMakI->_pPrt->Name() << "." << i->_pMakI->Name() << "  &\r\n"
						<< "    j_marker_name = ." << Name() << "." << i->_pMakJ->_pPrt->Name() << "." << i->_pMakJ->Name() << "  &\r\n"
						<< "    action_only = off  &\r\n"
						<< "    function = \"AKISPL(time,0," << i->Name() << "_fce_spl)\"  \r\n"
						<< "!\r\n";
				}

				/*没有仿真脚本时，才需要设置力控或者位置控制*/
				if (pScript == nullptr)
				{
					if (i->GetMode() == MOTION_BASE::FCE_CONTROL)
					{
						file << "constraint attributes  &\r\n"
							<< "constraint_name = ." << Name() << "." << i->Name() << "  &\r\n"
							<< "active = off \r\n"
							<< "!\r\n";
					}
					else
					{
						file << "force attributes  &\r\n"
							<< "force_name = ." << Name() << "." << i->Name() << "_fce  &\r\n"
							<< "active = off \r\n"
							<< "!\r\n";
					}
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
	}
}
