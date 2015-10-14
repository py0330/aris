/*!
* \file Aris_DynKer.h
* \brief 概述
* \author 潘阳
* \version 1.0.0.0
* \date 2014/4/5
*/

#ifndef Aris_DynModelBase_H
#define Aris_DynModelBase_H

#ifndef PI
#define PI 3.141592653589793
#endif

#include <vector>
#include <array>
#include <map>
#include <string>
#include <memory>
#include <functional>
#include <algorithm>

#include <Aris_XML.h>
#include <Aris_ExpCal.h>
#include <Aris_DynKer.h>

namespace Aris
{
	namespace DynKer
	{
		class PART;
		class MARKER;
		class JOINT_BASE;
		class MOTION_BASE;
		class FORCE_BASE;

		class ENVIRONMENT;
		class MODEL_BASE;

		class OBJECT
		{
		public:
			const std::string& Name() const { return _Name; };
			const MODEL_BASE* Model()const { return _pModel; };
			MODEL_BASE* Model() { return _pModel; };
		
		protected:
			explicit OBJECT(MODEL_BASE *pModel, const std::string &name) :_pModel(pModel), _Name(name) {};
			virtual ~OBJECT() {};

		private:
			OBJECT(const OBJECT &) = delete;
			OBJECT(OBJECT &&) = delete;
			OBJECT &operator=(const OBJECT &) = delete;
			OBJECT &operator=(OBJECT &&) = delete;

			MODEL_BASE *_pModel;
			std::string _Name;
		};
		class ELEMENT :public OBJECT
		{
		public:
			bool Active() const{ return _IsActive; };
			void SetActive(bool isActive) { _IsActive = isActive; };
			void Activate(){ _IsActive = true; };
			void Deactivate(){ _IsActive = false; };
			int GetID() const { return _id; };

			virtual void SaveResult(int id){};
			virtual void SetResultSize(int size){};

		private:
			virtual void ToXmlElement(Aris::Core::ELEMENT *pEle) const = 0;
			virtual void ToAdamsCmd(std::ofstream &file) const = 0;

		protected:
			explicit ELEMENT(MODEL_BASE *pModel, const std::string &name, int id)
				: OBJECT(pModel, name)
				, _id(id)
			{
			};
			virtual ~ELEMENT() = default;

		private:
			bool _IsActive{true};
			const int _id;

			friend class MODEL_BASE;
		};
		
		class PART :public ELEMENT
		{
		public:
			virtual ~PART() = default;
			virtual void Update();

			double* GetPmPtr() { return *_Pm; };
			double* GetInvPmPtr() { return *_InvPm; };
			double* GetVelPtr() { return _Vel; };
			double* GetAccPtr() { return _Acc; };
			const double* GetPmPtr() const { return *_Pm; };
			const double* GetVelPtr() const { return _Vel; };
			const double* GetAccPtr() const { return _Acc; };
			double* GetPrtVelPtr() { return _PrtVel; };
			double* GetPrtAccPtr() { return _PrtAcc; };
			const double* GetPrtImPtr() const { return *_PrtIm; };
			const double* GetInvPmPtr() const { return *_InvPm; };
			const double* GetPrtVelPtr() const { return _PrtVel; };
			const double* GetPrtAccPtr() const { return _PrtAcc; };
			const double* GetPrtFgPtr() const { return _PrtFg; };
			const double* GetPrtFvPtr() const { return _PrtFv; };
			const double* GetPrtGravityPtr() const { return _PrtGravity; };
			void SetPm(const double *pPm) { std::copy_n(pPm, 16, static_cast<double*>(*_Pm)); };
			void SetVel(const double *pVel) { std::copy_n(pVel, 6, _Vel); };
			void SetAcc(const double *pAcc) { std::copy_n(pAcc, 6, _Acc); };

			MARKER* GetMarker(const std::string &Name);
			const MARKER* GetMarker(const std::string &Name)const;

			template<typename ...Args>
			MARKER* AddMarker(const std::string & Name, Args ...args);

		private:
			explicit PART(MODEL_BASE *pModel, const std::string &Name, int id
				, const double *PrtIm = nullptr, const double *pm = nullptr
				, const double *Vel = nullptr, const double *Acc = nullptr);
			explicit PART(MODEL_BASE *pModel, const std::string &Name, int id, const Aris::Core::ELEMENT *ele);
			virtual void ToXmlElement(Aris::Core::ELEMENT *pEle) const;
			virtual void ToAdamsCmd(std::ofstream &file) const;

		private:
			std::map<std::string, int> _markerNames;

		private:
			double _Pm[4][4]{ { 0 } };
			double _InvPm[4][4]{ { 0 } };
			double _Vel[6]{ 0 };
			double _Acc[6]{ 0 };

			double _PrtIm[6][6]{ { 0 } };

			double _PrtGravity[6]{ 0 };
			double _PrtAcc[6]{ 0 };
			double _PrtVel[6]{ 0 };
			double _PrtFg[6]{ 0 };
			double _PrtFv[6]{ 0 };


		private:
			int _RowId;

		private:
			std::string graphicFilePath;

			friend class MARKER;
			friend class MODEL_BASE;
		};
		class MARKER :public ELEMENT
		{
		public:
			virtual ~MARKER() = default;
			virtual void Update();

			const double* GetPrtPmPtr() const { return *_PrtPm; };
			const double* GetPmPtr() const { return *_Pm; };
			const double* GetVelPtr() const { return _pPrt->_Vel; };
			const double* GetAccPtr() const { return _pPrt->_Acc; };
			const PART* GetFatherPrt() const { return _pPrt; };
			PART* GetFatherPrt() { return _pPrt; };

		private:
			explicit MARKER(PART *pPrt, const std::string &Name, int id, const double *pPrtPm = nullptr, MARKER *pRelativeTo = nullptr);
			explicit MARKER(PART *pPrt, const std::string &Name, int id, const Aris::Core::ELEMENT *ele);
			virtual void ToXmlElement(Aris::Core::ELEMENT *pEle) const;
			virtual void ToAdamsCmd(std::ofstream &file) const {};

		private:
			PART *_pPrt;

			double _Pm[4][4]{ { 0 } };
			double _PrtPm[4][4]{ { 0 } };

			friend class PART;
			friend class MODEL_BASE;
		};
		
		class JOINT_BASE :public ELEMENT
		{
		public:
			virtual ~JOINT_BASE() = default;
			virtual int GetCstDim() const = 0;
			virtual const char* GetType() const = 0;
			virtual double* GetCstFcePtr() = 0;
			virtual double* GetPrtCstMtxIPtr() = 0;
			virtual double* GetPrtCstMtxJPtr()  = 0;
			virtual double* GetPrtA_cPtr() = 0;

			virtual void Update();

			const MARKER* GetMakI() const { return _pMakI; };
			const MARKER* GetMakJ() const { return _pMakJ; };
			MARKER* GetMakI() { return _pMakI; };
			MARKER* GetMakJ() { return _pMakJ; };

		protected:
			explicit JOINT_BASE(MODEL_BASE *pModel, const std::string &Name, int id, MARKER *pMakI, MARKER *pMakJ);
			explicit JOINT_BASE(MODEL_BASE *pModel, const std::string &Name, int id, const Aris::Core::ELEMENT *ele);
			virtual void ToXmlElement(Aris::Core::ELEMENT *pEle) const;
			virtual void ToAdamsCmd(std::ofstream &file) const;
			virtual void Initiate() = 0;
		private:
			MARKER *_pMakI;
			MARKER *_pMakJ;

			int _ColId;

		private:
			friend class MODEL_BASE;
		};
		class MOTION_BASE :public ELEMENT
		{
		public:
			virtual ~MOTION_BASE() = default;

			virtual const char* GetType() const = 0;
			const double* GetFrcCoePtr() const { return _frc_coe; };
			double GetMotPos() const { return MotPos; };
			double GetMotVel() const { return MotVel; };
			double GetMotAcc() const { return MotAcc; };
			double GetMotFce() const { return GetMotFceDyn() + GetMotFceFrc();	};
			double GetMotFceFrc() const { return s_sgn(MotVel)*_frc_coe[0] + MotVel*_frc_coe[1] + MotAcc*_frc_coe[2]; };
			double GetMotFceDyn() const { return MotDynFce; }
			
			const double* GetPrtCstMtxIPtr() const { return _PrtCstMtxI; };
			const double* GetPrtCstMtxJPtr() const { return _PrtCstMtxJ; };
			const double* GetPrtA_cPtr() const { return _a_c; };

			void SetFrcCoe(const double *frc_coe) { std::copy_n(frc_coe, 3, _frc_coe); };
			void SetMotAcc(double motAcc) { this->MotAcc = motAcc; };

			virtual void Update() = 0;

			void SetPosAkimaCurve(const int num, const double* time, const double *pos)
			{
				this->posCurve.reset(new AKIMA(num, time, pos));
			}
			double PosAkima(double t, char derivativeOrder = '0') { return posCurve->operator()(t, derivativeOrder); };
			void PosAkima(int length, const double *t, double *pos, char order = '0') { posCurve->operator()(length, t, pos, order); };

		protected:
			explicit MOTION_BASE(MODEL_BASE *pModel, const std::string &Name, int id, MARKER *pMakI = 0, MARKER *pMakJ = 0);
			explicit MOTION_BASE(MODEL_BASE *pModel, const std::string &Name, int id, const Aris::Core::ELEMENT *ele);
			virtual void ToXmlElement(Aris::Core::ELEMENT *pEle) const;
			virtual void ToAdamsCmd(std::ofstream &file) const;
			virtual void Initiate() = 0;

		protected:
			MARKER *_pMakI, *_pMakJ;

			/*pos\vel\acc\fce of motion*/
			double MotPos{ 0 }, MotVel{ 0 }, MotAcc{ 0 }, MotDynFce{ 0 };

			double _frc_coe[3]{0};

			int _ColId{ 0 };

			double _PrtCstMtxI[6]{ 0 };
			double _PrtCstMtxJ[6]{ 0 };
			double _a_c[6]{ 0 };

			/*for adams*/
			std::unique_ptr<AKIMA> posCurve;

			friend class MODEL_BASE;
		};
		class FORCE_BASE :public ELEMENT
		{
		public:
			const double* GetPrtFceIPtr() const { return _PrtFceI; };
			const double* GetPrtFceJPtr() const { return _PrtFceJ; };
			
			void SetFceAkimaCurve(const int num, const double* time, const double *fce)
			{
				this->fceCurve.reset(new AKIMA(num, time, fce));
			}
			double FceAkima(double t, char derivativeOrder = '0') { return fceCurve->operator()(t, derivativeOrder); };
			void FceAkima(int length, const double *t, double *pos, char order = '0') { fceCurve->operator()(length, t, pos, order); };

			virtual ~FORCE_BASE() = default;
			virtual void Update() = 0;

		protected:
			explicit FORCE_BASE(MODEL_BASE *pModel, const std::string &Name, int id, MARKER *pMakI, MARKER *pMakJ);
			explicit FORCE_BASE(MODEL_BASE *pModel, const std::string &Name, int id, const Aris::Core::ELEMENT *pEle);
			virtual void ToXmlElement(Aris::Core::ELEMENT *pEle) const {};
			virtual void ToAdamsCmd(std::ofstream &file) const {};

			MARKER *_pMakI, *_pMakJ;

			double _PrtFceI[6]{ 0 };
			double _PrtFceJ[6]{ 0 };

			std::unique_ptr<AKIMA> fceCurve;

			friend class MODEL_BASE;
		};

		template<int DIMENSION>
		class JOINT_BASE_DIM :public JOINT_BASE
		{
		public:
			static constexpr int GetDim() { return DIMENSION; };
			virtual int GetCstDim() const { return DIMENSION; };
			virtual double* GetCstFcePtr() { return _CstFce; };
			virtual double* GetPrtCstMtxIPtr() { return *_PrtCstMtxI; };
			virtual double* GetPrtCstMtxJPtr() { return *_PrtCstMtxJ; };
			virtual double* GetPrtA_cPtr() { return _a_c; };
			virtual const double* GetCstFcePtr() const { return _CstFce; };
			virtual const double* GetPrtCstMtxIPtr() const { return *_PrtCstMtxI; };
			virtual const double* GetPrtCstMtxJPtr() const { return *_PrtCstMtxJ; };
			virtual const double* GetPrtA_cPtr() const { return _a_c; };

		protected:
			explicit JOINT_BASE_DIM(MODEL_BASE *pModel, const std::string &Name, int id, MARKER *pMakI, MARKER *pMakJ)
				:JOINT_BASE(pModel, Name, id, pMakI, pMakJ) {};
			explicit JOINT_BASE_DIM(MODEL_BASE *pModel, const std::string &Name, int id, const Aris::Core::ELEMENT *ele)
				:JOINT_BASE(pModel, Name, id, ele) {};

		protected:
			double _PrtCstMtxI[6][DIMENSION]{ { 0 } };
			double _PrtCstMtxJ[6][DIMENSION]{ { 0 } };
			double _CstFce[DIMENSION]{ 0 };
			double _a_c[DIMENSION]{ 0 };

		private:
			friend class MODEL_BASE;
		};

		class ENVIRONMENT:public OBJECT
		{
		private:
			explicit ENVIRONMENT(MODEL_BASE *pModel);
			~ENVIRONMENT();

			void ToXmlElement(Aris::Core::ELEMENT *pEle) const;
			void FromXmlElement(const Aris::Core::ELEMENT *pEle);
			void ToAdamsCmd(std::ofstream &file) const;
		private:
			double Gravity[6]{ 0, -9.8, 0, 0, 0, 0 };

			friend class PART;
			friend class MODEL_BASE;
		};
		class SIMULATE_SCRIPT
		{
		public:
			void ScriptActivate(int time, ELEMENT * ele)
			{
				script[time].elements[ele] = true;
			};
			void ScriptDeactivate(int time, ELEMENT * ele)
			{
				script[time].elements[ele] = false;
			}
			void ScriptMoveMarker(int time, MARKER * ele, double * data)
			{
				std::copy_n(data, 6, script[time].markers[ele].data());
			}
			void ScriptClear(){ script.clear(); };
			void SetEndTime(int endTime){ this->endTime = endTime; };
			void SetDt(int dt){ this->dt = dt; };
			int GetEndTime() const { return endTime; };
			int GetDt() const { return dt; };

			struct NODE
			{
				std::map<ELEMENT *, bool> elements;
				std::map<MARKER *, std::array<double, 6> > markers;
			};
			std::map < int/*time*/, NODE > script;

		private:
			int endTime{ 0 };
			int dt{ 1 };


			friend class MODEL_BASE;
		};

		class MODEL_BASE :public OBJECT
		{
		public:
			explicit MODEL_BASE(const std::string & Name = "Model");
			virtual ~MODEL_BASE();

			template<typename ...Args>
			PART* AddPart(const std::string & Name, Args ...args)
			{
				if (GetPart(Name) != nullptr)
				{
					return nullptr;
				}

				_parts.push_back(std::unique_ptr<PART>(new PART(this, Name, _parts.size(), args...)));
				return _parts.back().get();
			}
			template<typename JOINT, typename ...Args>
			JOINT* AddJoint(const std::string & Name, Args ...args)
			{
				if (GetJoint(Name) != nullptr)
				{
					return nullptr;
				}

				auto ret = new JOINT(this, Name, _joints.size(), args...);
				_joints.push_back(std::unique_ptr<JOINT_BASE>(ret));
				return ret;
			}
			template<typename MOTION, typename ...Args>
			MOTION* AddMotion(const std::string & Name, Args ...args)
			{
				if (GetMotion(Name) != nullptr)
				{
					return nullptr;
				}

				auto ret = new MOTION(this, Name, _motions.size(), args...);
				_motions.push_back(std::unique_ptr<MOTION_BASE>(ret));
				return ret;
			};
			template<typename FORCE, typename ...Args>
			FORCE* AddForce(const std::string & Name, Args ...args)
			{
				if (GetForce(Name) != nullptr)
				{
					return nullptr;
				}
				auto ret = new FORCE(this, Name, _joints.size(), args...);
				_forces.push_back(std::unique_ptr<FORCE_BASE>(ret));
				return ret;
			}

			const PART *GetPart(int id) const;
			const JOINT_BASE *GetJoint(int id) const;
			const MOTION_BASE *GetMotion(int id) const;
			const FORCE_BASE *GetForce(int id) const;
			const MARKER *GetMarker(int id) const;
			PART *GetPart(int id);
			JOINT_BASE *GetJoint(int id);
			MOTION_BASE *GetMotion(int id);
			FORCE_BASE *GetForce(int id);
			MARKER *GetMarker(int id);
			const PART *GetPart(const std::string &Name)const;
			const JOINT_BASE *GetJoint(const std::string &Name)const;
			const MOTION_BASE *GetMotion(const std::string &Name)const;
			const FORCE_BASE *GetForce(const std::string &Name)const;
			PART *GetPart(const std::string &Name);
			JOINT_BASE *GetJoint(const std::string &Name);
			MOTION_BASE *GetMotion(const std::string &Name);
			FORCE_BASE *GetForce(const std::string &Name);
			int GetPartNum() const{ return _parts.size(); };
			int GetMotionNum() const{ return _motions.size(); };
			int GetJointNum() const{ return _joints.size(); };
			int GetForceNum() const{ return _forces.size(); };
			int GetMarkerNum() const{ return _markers.size(); };

			void ForEachPart(std::function<void(PART*)> fun)
			{
				for (auto&i : _parts)
				{
					fun(i.get());
				}
			}
			void ForEachMarker(std::function<void(MARKER*)> fun)
			{
				for (auto&i : _markers)
				{
					fun(i.get());
				}
			}
			void ForEachMotion(std::function<void(MOTION_BASE*)> fun)
			{
				for (auto&i : _motions)
				{
					fun(i.get());
				}
			}
			void ForEachForce(std::function<void(FORCE_BASE*)> fun)
			{
				for (auto&i : _forces)
				{
					fun(i.get());
				}
			}
			void ForEachJoint(std::function<void(JOINT_BASE*)> fun)
			{
				for (auto&i : _joints)
				{
					fun(i.get());
				}
			}
			void ForEachElement(std::function<void(ELEMENT*)> fun)
			{
				for (auto&i : _parts)
				{
					fun(i.get());
				}
				for (auto&i : _markers)
				{
					fun(i.get());
				}
				for (auto&i : _joints)
				{
					fun(i.get());
				}
				for (auto&i : _motions)
				{
					fun(i.get());
				}
				for (auto&i : _forces)
				{
					fun(i.get());
				}
			}

			void DynPre(int *pI_dim = nullptr, int *pC_dim = nullptr);
			void DynMtx(double *C, double*a_c, double *I_mat, double*f, double *D, double *b);
			void DynUkn(double *a, double*f_c);
			void DynEnd(const double *x);
			void Dyn(std::function<void(int dim, const double *D, const double *b, double *x)> solveMethod);

			void ClbPre(int &clb_dim_m, int &clb_dim_n, int &gamma_dim, int &frc_coe_dim);
			void ClbMtx(double *clb_d_ptr, double *clb_b_ptr, std::function<void(int n, double *A)> inverseMethod);
			void ClbUkn(double *clb_gamma_and_frcCoe_ptr);

			void LoadXml(const char *filename);
			void LoadXml(const Aris::Core::DOCUMENT &xmlDoc);
			virtual void FromXmlElement(const Aris::Core::ELEMENT *pEle);
			void SaveSnapshotXml(const char *filename) const;
			void SaveAdams(const char *filename, const SIMULATE_SCRIPT* pScript) const;
			void SaveAdams(const char *filename, bool isModifyActive = true) const;

		private:
			CALCULATOR calculator;
			
			int C_dim;//real dimension of constraint matrix
			int I_dim;//real dimension of inertia matrix

		protected:
			template<typename CONSTRAINT>
			void InitiateElement(CONSTRAINT *pConstraint) { pConstraint->Initiate(); };

			std::vector<std::unique_ptr<PART> > _parts;
			std::vector<std::unique_ptr<JOINT_BASE> > _joints;
			std::vector<std::unique_ptr<MOTION_BASE> > _motions;
			std::vector<std::unique_ptr<FORCE_BASE> > _forces;
			std::vector<std::unique_ptr<MARKER> > _markers;
			
			ENVIRONMENT _Environment;
			PART* pGround;
		
			friend class ENVIRONMENT;
			friend class PART;
			friend class MOTION_BASE;
			friend class JOINT;
			friend class MARKER;
			friend class FORCE_BASE;
		};

		template<typename ...Args>
		MARKER* PART::AddMarker(const std::string & Name, Args ...args)
		{
			if (GetMarker(Name) != nullptr)
			{
				return nullptr;
			}

			Model()->_markers.push_back(std::unique_ptr<MARKER>(new MARKER(this, Name, Model()->_markers.size(), args...)));
			_markerNames.insert(std::make_pair(Name, Model()->_markers.size() - 1));
			return Model()->_markers.back().get();
		}
	}
}

#endif
