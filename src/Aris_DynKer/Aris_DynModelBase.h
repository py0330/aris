/*!
* \file Aris_DynKer.h
* \brief 概述
* \author 潘阳
* \version 1.0.0.0
* \date 2014/4/5
*/

#ifndef Aris_DynModelBase_H
#define Aris_DynModelBase_H

#include <Platform.h>

#ifdef PLATFORM_IS_WINDOWS
#ifndef _SCL_SECURE_NO_WARNINGS
#define _SCL_SECURE_NO_WARNINGS
#endif
#endif

#ifndef PI
#define PI 3.141592653589793
#endif

#include <vector>
#include <map>
#include <string>
#include <memory>
#include <functional>
#include <algorithm>

#include <cstdlib>

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
		class MODEL;

		class OBJECT
		{
		public:
			const std::string& Name() const { return _Name; };
			const MODEL* Model()const { return _pModel; };
			MODEL* Model() { return _pModel; };
		
		protected:
			void SetName(const std::string &name) { _Name = name; };

			explicit OBJECT(MODEL *pModel, const std::string &name) :_pModel(pModel), _Name(name) {};
			OBJECT(const OBJECT &) = delete;
			OBJECT(OBJECT &&) = delete;

			virtual ~OBJECT() {};

			OBJECT &operator=(const OBJECT &) = delete;
			OBJECT &operator=(OBJECT &&) = delete;

		private:
			MODEL *_pModel;
			std::string _Name;
		};
		class ELEMENT :public OBJECT
		{
		public:
			bool Active() const{ return _IsActive; };
			void Activate(){ _IsActive = true; };
			void Deactivate(){ _IsActive = false; };
			int GetID() const { return _id; };

			virtual void SaveResult(int id){};
			virtual void SetResultSize(int size){};

		private:
			virtual void ToXmlElement(Aris::Core::ELEMENT *pEle) const = 0;
			virtual void FromXmlElement(const Aris::Core::ELEMENT *pEle) = 0;

		protected:
			explicit ELEMENT(MODEL *pModel, const std::string &name, int id)
				: OBJECT(pModel, name)
				, _id(id)
			{
			};
			virtual ~ELEMENT() = default;

		private:
			bool _IsActive;
			const int _id;
		};
		/** \brief 部件类型
		*
		* 定义模型中每个部件的数据结构
		* 
		*/
		class PART :public ELEMENT
		{
		public:
			virtual ~PART() = default;

			double* GetPmPtr() { return *_Pm; };
			double* GetVelPtr() { return _Vel; };
			double* GetAccPtr() { return _Acc; };
			const double* GetPmPtr() const{ return *_Pm; };
			const double* GetVelPtr() const{ return _Vel; };
			const double* GetAccPtr() const{ return _Acc; };
			double* GetPrtPmPtr() { return *_PrtPm; };
			double* GetPrtVelPtr() { return _PrtVel; };
			double* GetPrtAccPtr() { return _PrtAcc; };
			const double* GetPrtImPtr() const{ return *_PrtIm; };
			const double* GetPrtPmPtr() const{ return *_PrtPm; };
			const double* GetPrtVelPtr() const { return _PrtVel; };
			const double* GetPrtAccPtr() const{ return _PrtAcc; };
			const double* GetPrtFgPtr() const{ return _PrtFg; };
			const double* GetPrtFvPtr() const{ return _PrtFv; };
			const double* GetPrtGravityPtr() const{ return _PrtGravity; };
			void SetPm(const double *pPm) { std::copy_n(pPm, 16, static_cast<double*>(*_Pm)); };
			void SetVel(const double *pVel) { std::copy_n(pVel, 6, _Vel); };
			void SetAcc(const double *pAcc) { std::copy_n(pAcc, 6, _Acc); };

			void Update();

			

			MARKER* GetMarker(const std::string &Name);
			const MARKER* GetMarker(const std::string &Name)const;
			MARKER* AddMarker(const std::string &Name, const double *pm = 0, MARKER *pRelativeTo = 0);
			
		

			virtual void SaveResult(int id)
			{
				s_pm2pe(*_Pm, result.at(id).pe);
				std::copy_n(_Vel, 6, result.at(id).vel);
				std::copy_n(_Acc, 6, result.at(id).acc);
			}
			virtual void SetResultSize(int size)
			{
				result.resize(size);
			}
			struct RESULT_NODE
			{
				bool active;
				double pe[6];
				double vel[6];
				double acc[6];
			};

		private:
			virtual void ToXmlElement(Aris::Core::ELEMENT *pEle) const;
			virtual void FromXmlElement(const Aris::Core::ELEMENT *pEle);

		private:
			std::vector<RESULT_NODE> result;

		private:
			explicit PART(MODEL *pModel
				, const std::string &Name
				, int id
				, const double *PrtIm = nullptr
				, const double *pm = nullptr
				, const double *Vel = nullptr
				, const double *Acc = nullptr);

			PART(const PART&) = delete;
			PART(PART&&) = delete;
			PART & operator =(const PART &) = delete;
			PART & operator =(PART &&) = delete;

		private:
			std::map<std::string, int> _markerNames;
		
		private:
			double _Pm[4][4];
			double _Vel[6];
			double _Acc[6];

		private:
			double _PrtIm[6][6];
			double _PrtPm[4][4];//inverse of the _Pm
			double _PrtGravity[6];
			double _PrtAcc[6];
			double _PrtVel[6];
			double _PrtFg[6];
			double _PrtFv[6];

			int _RowId;

		private:
			std::string graphicFilePath;

			

			friend class MODEL;
		};
		class MARKER :public ELEMENT
		{
		public:
			virtual ~MARKER() = default;

			void Update();

			const double* GetPrtPmPtr() const{ return *_PrtPm; };
			const double* GetPmPtr() const{ return *_Pm; };
			const double* GetVelPtr() const{ return _pPrt->GetVelPtr(); };
			const double* GetAccPtr() const{ return _pPrt->GetAccPtr(); };
			const PART* GetFatherPrt() const{ return _pPrt; };
			PART* GetFatherPrt() { return _pPrt; };

		private:
			MARKER(MODEL *pModel, const std::string &Name, int id, PART* pPart = 0, const double *pLocPm = 0, MARKER *pRelativeTo = 0);

			MARKER(const MARKER &) = delete;
			MARKER(MARKER &&) = delete;
			MARKER& operator=(const MARKER &) = delete;
			MARKER& operator=(MARKER &&) = delete;

			virtual void ToXmlElement(Aris::Core::ELEMENT *pEle) const;
			virtual void FromXmlElement(const Aris::Core::ELEMENT *pEle);

		private:
			double _Pm[4][4];
			PART *_pPrt;
			double _PrtPm[4][4];

			friend class PART;
			friend class MODEL;
		};
		
		

		class JOINT_BASE :public ELEMENT
		{
		public:
			virtual ~JOINT_BASE() = default;
			virtual int GetCstDim() const = 0;
			virtual const char* GetType() const = 0;
			virtual void Update() = 0;

			const double* GetCstFcePtr() const { return _CstFcePtr; };
			const double* GetPrtCstMtxIPtr() const { return _PrtCstMtxIPtr; };
			const double* GetPrtCstMtxJPtr() const { return _PrtCstMtxJPtr; };
			const double* GetPrtA_cPtr() const { return _a_cPtr; };
			const MARKER* GetMakI() const { return _pMakI; };
			const MARKER* GetMakJ() const { return _pMakJ; };
			MARKER* GetMakI() { return _pMakI; };
			MARKER* GetMakJ() { return _pMakJ; };

			/*for simulation*/
			virtual void SaveResult(int id)
			{
				std::copy_n(_CstFcePtr, 6, result.at(id).fce);
			}
			virtual void SetResultSize(int size)
			{
				result.resize(size);
			}
			struct RESULT_NODE
			{
				bool active;
				double fce[6];
			};


		protected:
			explicit JOINT_BASE(MODEL *pModel, const std::string &Name, int id, MARKER *pMakI, MARKER *pMakJ
				, double *PrtCstMtxI, double *PrtCstMtxJ, double *CstFce, double *a_c);
			virtual void Initiate();

			double _tm_I2M[6][6];

		private:
			JOINT_BASE(const JOINT_BASE &) = delete;
			JOINT_BASE(JOINT_BASE &&) = delete;
			JOINT_BASE& operator=(const JOINT_BASE &) = delete;
			JOINT_BASE& operator=(JOINT_BASE &&) = delete;

			virtual void ToXmlElement(Aris::Core::ELEMENT *pEle) const;
			virtual void FromXmlElement(const Aris::Core::ELEMENT *pEle);

		private:
			MARKER *_pMakI;
			MARKER *_pMakJ;

			double*const _PrtCstMtxIPtr;
			double*const _PrtCstMtxJPtr;
			double*const _CstFcePtr;
			double*const _a_cPtr;

			int _ColId;

			/*for simulation*/
		private:
			std::vector<RESULT_NODE> result;

			friend class MODEL;
		};
		class MOTION_BASE :public ELEMENT
		{
		public:
			enum MOTION_MODE
			{
				POS_CONTROL,
				FCE_CONTROL
			};

			virtual ~MOTION_BASE() = default;

			MOTION_MODE GetMode() const { return _Mode; };
			virtual int GetCstDim() const = 0;
			virtual const char* GetType() const = 0;
			const double* GetFrcCoePtr() const { return _frc_coe; };
			const double* GetF_mPtr() const { return _f_m; };
			const double* GetA_mPtr() const { return _a_m; };
			const double* GetV_mPtr() const { return _v_m; };
			const double* GetP_mPtr() const { return _p_m; };
			const double* GetPrtCstMtxIPtr() const { return *_PrtCstMtxI; };
			const double* GetPrtCstMtxJPtr() const { return *_PrtCstMtxJ; };
			const double* GetPrtA_cPtr() const { return _a_c; };

			void SetP_m(const double *p_m) { std::copy_n(p_m, GetCstDim(), _p_m); };
			void SetV_m(const double *v_m) { std::copy_n(v_m, GetCstDim(), _v_m); };
			void SetA_m(const double *a_m) { std::copy_n(a_m, GetCstDim(), _a_m); };
			void SetF_m(const double *f_m) { std::copy_n(f_m, GetCstDim(), _f_m); };
			void SetMode(MOTION_MODE mode) { _Mode = mode; };
			void SetFrcCoe(const double *frc_coe) { std::copy_n(frc_coe, 3 * GetCstDim(), _frc_coe); };

			virtual void Update() = 0;

			double FceAkima(double t, char derivativeOrder = '0') { return fceCurve->operator()(t, derivativeOrder); };
			double PosAkima(double t, char derivativeOrder = '0') { return posCurve->operator()(t, derivativeOrder); };
			void FceAkima(int length, const double *t, double *fce, char order = '0') { fceCurve->operator()(length, t, fce, order); };
			void PosAkima(int length, const double *t, double *pos, char order = '0') { posCurve->operator()(length, t, pos, order); };

			void SetPosAkimaCurve(const int num, const double* time, const double *pos)
			{
				this->posCurve.reset(new AKIMA(num, time, pos));
			}
			void SetFceAkimaCurve(const int num, const double* time, const double *fce)
			{
				this->fceCurve.reset(new AKIMA(num, time, fce));
			}



			virtual void SaveResult(int id)
			{
				std::copy_n(GetF_mPtr(), 6, result.at(id).fce);
				std::copy_n(GetP_mPtr(), 6, result.at(id).pos);
				std::copy_n(GetV_mPtr(), 6, result.at(id).vel);
				std::copy_n(GetA_mPtr(), 6, result.at(id).acc);
			}
			virtual void SetResultSize(int size)
			{
				result.resize(size);
			}
			struct RESULT_NODE
			{
				bool active;
				MOTION_MODE mode;
				double fce[6];
				double pos[6];
				double vel[6];
				double acc[6];
			};

		private:
			std::vector<RESULT_NODE> result;

		protected:
			explicit MOTION_BASE(MODEL *pModel, const std::string &Name, int id, MOTION_MODE mode = POS_CONTROL, MARKER *pMakI = 0, MARKER *pMakJ = 0);
		
		private:
			MOTION_BASE(const MOTION_BASE &) = delete;
			MOTION_BASE(MOTION_BASE &&) = delete;
			MOTION_BASE& operator=(const MOTION_BASE &) = delete;
			MOTION_BASE& operator=(MOTION_BASE &&) = delete;

			virtual void ToXmlElement(Aris::Core::ELEMENT *pEle) const;
			virtual void FromXmlElement(const Aris::Core::ELEMENT *pEle);

			virtual void _Initiate() = 0;

		protected:
			MOTION_MODE _Mode;
			MARKER *_pMakI, *_pMakJ;

			/*pos\vel\acc\fce of motion*/
			double _p_m[6];
			double _v_m[6];
			double _a_m[6];
			double _f_m[6];
			double _frc_coe[3];

			int _ColId;

			double _PrtCstMtxI[6][6];
			double _PrtCstMtxJ[6][6];
			double _a_c[6];

			/*for adams*/
			std::unique_ptr<AKIMA> posCurve;
			std::unique_ptr<AKIMA> fceCurve;

			friend class MODEL;
		};
		class FORCE_BASE :public ELEMENT
		{
		public:
			virtual ~FORCE_BASE() = default;
			virtual void Update() = 0;
			const double* GetPrtFceIPtr() const { return _PrtFceI; };
			const double* GetPrtFceJPtr() const { return _PrtFceJ; };

		protected:
			explicit FORCE_BASE(MODEL *pModel, const std::string &Name, int id, PART *pPrtM, PART *pPrtN);

			FORCE_BASE(const FORCE_BASE &) = delete;
			FORCE_BASE(FORCE_BASE &&) = delete;
			FORCE_BASE& operator=(const FORCE_BASE &) = delete;
			FORCE_BASE& operator=(FORCE_BASE &&) = delete;

			virtual void ToXmlElement(Aris::Core::ELEMENT *pEle) const {};
			virtual void FromXmlElement(const Aris::Core::ELEMENT *pEle) {};

			PART *_pPrtM;
			PART *_pPrtN;

			double _PrtFceI[6];
			double _PrtFceJ[6];

			friend class MODEL;
		};

		


		class ENVIRONMENT:public OBJECT
		{
		private:
			explicit ENVIRONMENT(MODEL *pModel);
			~ENVIRONMENT();

			void ToXmlElement(Aris::Core::ELEMENT *pEle) const;
			void FromXmlElement(const Aris::Core::ELEMENT *pEle);

		private:
			double Gravity[6];

			friend class PART;
			friend class MODEL;
		};
		class SIMULATE_SCRIPT
		{
		public:
			void ScriptActivate(int time, JOINT_BASE * jnt, const double *peMakI=nullptr, const double *peMakJ=nullptr)
			{
				JOINT_STRUCT jnt_struct;
				jnt_struct.active = true;

				jnt_struct.isModifyMakI = peMakI!=nullptr;
				jnt_struct.isModifyMakJ = peMakJ!=nullptr;

				if (peMakI)
					std::copy_n(peMakI, 6, jnt_struct.peMakI);
				if (peMakJ)
					std::copy_n(peMakJ, 6, jnt_struct.peMakJ);
				
				script[time].joints[jnt] = jnt_struct;
			};
			void ScriptDeactivate(int time, JOINT_BASE * jnt)
			{
				JOINT_STRUCT jnt_struct;
				jnt_struct.active = false;
				script[time].joints[jnt] = jnt_struct;
			}
			void ScriptSwitchMode(int time, MOTION_BASE * ele, MOTION_BASE::MOTION_MODE mode)
			{
				script[time].motions[ele] = mode;
			}
			void ScriptClear(){ script.clear(); };
			void ScriptEndTime(int endTime){ this->endTime = endTime; };
			void ScriptDt(int dt){ this->dt = dt; };

			struct JOINT_STRUCT
			{
				bool active;
				bool isModifyMakI;
				bool isModifyMakJ;
				double peMakI[6];
				double peMakJ[6];
			};
			struct NODE
			{
				std::map<JOINT_BASE *, JOINT_STRUCT> joints;
				std::map<MOTION_BASE *, MOTION_BASE::MOTION_MODE> motions;
			};
			std::map < int/*time*/, NODE > script;

		private:
			int endTime{0};
			int dt{ 1 };


			friend class MODEL;
		};

		class MODEL :public OBJECT
		{
		public:
			explicit MODEL(const std::string & Name = "Model");
			virtual ~MODEL();

			PART* AddPart(const std::string & Name
				, const double *Im = nullptr
				, const double *pm = nullptr
				, const double *Vel = nullptr
				, const double *Acc = nullptr);
			template<typename JOINT, typename ...Args>
			JOINT_BASE* AddJoint(const std::string & Name, Args ...args)
			{
				if (GetJoint(Name) != nullptr)
				{
					return nullptr;
				}

				_joints.push_back(std::unique_ptr<JOINT_BASE>(new JOINT(this, Name, _joints.size(), args...)));
				return _joints.back().get();
			}
			template<typename MOTION, typename ...Args>
			MOTION_BASE* AddMotion(const std::string & Name, Args ...args)
			{
				if (GetMotion(Name) != nullptr)
				{
					return nullptr;
				}

				_motions.push_back(std::unique_ptr<MOTION_BASE>(new MOTION(this, Name, _motions.size(), args...)));
				return _motions.back().get();
			};
			template<typename FORCE, typename ...Args>
			FORCE_BASE* AddForce(const std::string & Name, Args ...args)
			{
				if (GetForce(Name) != nullptr)
				{
					return nullptr;
				}

				_forces.push_back(std::unique_ptr<FORCE_BASE>(new FORCE(this, Name, _forces.size(), args...)));
				return _forces.back().get();
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

			void DynPre();
			void DynPrtMtx();
			void Dyn();

			void ClbEqnTo(double *&clb_d_ptr, double *&clb_b_ptr, int &clb_dim_m, int &clb_dim_n);

			void LoadXml(const char *filename);
			void FromXmlElement(const Aris::Core::ELEMENT *pEle);
			void SaveSnapshotXml(const char *filename) const;
			void SaveAdams(const char *filename, SIMULATE_SCRIPT* pScript=nullptr) const;

		private:
			MODEL(const MODEL &) = delete;
			MODEL(MODEL &&) = delete;
			MODEL& operator=(const MODEL &) = delete;
			MODEL& operator=(MODEL &&) = delete;

			CALCULATOR calculator;
			
			int C_dim;//real dimension of constraint matrix
			int I_dim;//real dimension of inertia matrix

		protected:
			std::vector<std::unique_ptr<PART> > _parts;
			std::vector<std::unique_ptr<JOINT_BASE> > _joints;
			std::vector<std::unique_ptr<MOTION_BASE> > _motions;
			std::vector<std::unique_ptr<FORCE_BASE> > _forces;
			std::vector<std::unique_ptr<MARKER> > _markers;
			
			std::vector<double> _C;
			std::vector<double> _I;

			std::vector<double> _f;
			std::vector<double> _a_c;

			std::vector<double> _D;
			std::vector<double> _b;

			std::vector<double> _s;
			std::vector<double> _x;

			double *C, *pI, *f, *a_c, *D, *b,*s,*x;

			ENVIRONMENT _Environment;

			PART* pGround;
		
			friend class ENVIRONMENT;
			friend class PART;
			friend class MOTION_BASE;
			friend class JOINT;
			friend class MARKER;
			friend class FORCE_BASE;
		};
	}
}

#endif
