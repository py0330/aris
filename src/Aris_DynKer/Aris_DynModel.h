/*!
* \file Aris_DynKer.h
* \brief 概述
* \author 潘阳
* \version 1.0.0.0
* \date 2014/4/5
*/

#ifndef Aris_DynModel_H
#define Aris_DynModel_H

#ifndef PI
#define PI 3.141592653589793
#endif

#include <vector>
#include <map>
#include <string>
#include <memory>
#include <functional>

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
		class JOINT;
		class MOTION;
		class FORCE;

		class ENVIRONMENT;
		class MODEL;

		class OBJECT
		{
		public:
			OBJECT(MODEL *pModel, const std::string &);
			virtual ~OBJECT();

			std::string GetName() const;
		
		protected:
			std::string _Name;
			MODEL *_pModel;
		};
		class ELEMENT :public OBJECT
		{
		public:
			ELEMENT(MODEL *pModel, const std::string &name)
				:OBJECT(pModel, name)
			{

			};
			virtual ~ELEMENT() = default;

			bool GetActive() const{ return _IsActive; };
			void Activate(){ _IsActive = true; };
			void Deactivate(){ _IsActive = false; };

			virtual unsigned GetID() const = 0;

			virtual void SaveResult(unsigned id){};
			virtual void SetResultSize(unsigned size){};
			virtual void ToXmlElement(Aris::Core::ELEMENT *pEle) const{};
			virtual void FromXmlElement(const Aris::Core::ELEMENT *pEle){};

		protected:
			bool _IsActive;
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
			const double* GetPrtTmfPtr() const{ return *_PrtTmf; };
			const double* GetPrtTmvPtr() const{ return *_PrtTmv; };
			const double* GetPrtCmfPtr() const{ return *_PrtCmf; };
			const double* GetPrtCmvPtr() const{ return *_PrtCmv; };
			const double* GetPrtAccPtr() const{ return _PrtAcc; };
			const double* GetPrtVelPtr() const{ return _PrtVel; };
			const double* GetPrtFgPtr() const{ return _PrtFg; };
			const double* GetPrtFvPtr() const{ return _PrtFv; };
			const double* GetPrtGravityPtr() const{ return _PrtGravity; };

			void UpdateInPrt();

			void SetPrtIm(const double *pPrtIm){ memcpy(_PrtIm, pPrtIm, sizeof(_PrtIm)); };
			void SetPm(const double *pPm){ memcpy(_Pm, pPm, pm_size); };
			void SetVel(const double *pVel){ memcpy(_Vel, pVel, vel_size); };
			void SetAcc(const double *pAcc){ memcpy(_Acc, pAcc, acc_size); };

			MARKER* GetMarker(const std::string &Name);
			const MARKER* GetMarker(const std::string &Name)const;
			MARKER* AddMarker(const std::string &Name, const double *pm = 0, MARKER *pRelativeTo = 0);

			virtual unsigned GetID() const;
			
			virtual void ToXmlElement(Aris::Core::ELEMENT *pEle) const;
			virtual void FromXmlElement(const Aris::Core::ELEMENT *pEle);

			virtual void SaveResult(unsigned id)
			{
				s_pm2pe(*_Pm, result.at(id).pe);
				memcpy(result.at(id).vel, _Vel, vel_size);
				memcpy(result.at(id).acc, _Acc, acc_size);
			}
			virtual void SetResultSize(unsigned size)
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
			std::vector<RESULT_NODE> result;

		private:
			explicit PART(MODEL *pModel
				, const std::string &Name = ""
				, const double *PrtIm = nullptr
				, const double *pm = nullptr
				, const double *Vel = nullptr
				, const double *Acc = nullptr);

			PART(const PART&) = delete;
			PART(PART&&) = delete;
			PART & operator =(const PART &) = delete;
			PART & operator =(PART &&) = delete;

		private:
			std::map<std::string, unsigned> _markerNames;
		
		private:
			double _Pm[4][4];
			double _Vel[6];
			double _Acc[6];

		private:
			double _PrtIm[6][6];
			double _PrtPm[4][4];//inverse of the _Pm
			double _PrtTmf[6][6];//inverse of the _Tmf
			double _PrtTmv[6][6];//inverse of the _Tmv
			double _PrtCmf[6][6];
			double _PrtCmv[6][6];
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

			virtual unsigned GetID() const;

			virtual void ToXmlElement(Aris::Core::ELEMENT *pEle) const;
			virtual void FromXmlElement(const Aris::Core::ELEMENT *pEle);

		private:
			MARKER(MODEL *pModel, const std::string &Name = "", PART* pPart = 0, const double *pLocPm = 0, MARKER *pRelativeTo = 0);

			MARKER(const MARKER &) = delete;
			MARKER(MARKER &&) = delete;
			MARKER& operator=(const MARKER &) = delete;
			MARKER& operator=(MARKER &&) = delete;

		private:
			double _Pm[4][4];
			PART *_pPrt;
			double _PrtPm[4][4];

			friend class PART;
			friend class MODEL;
			friend struct std::pair < const std::string, MARKER >;
			friend struct std::pair < std::string, MARKER >;
		};
		class JOINT :public ELEMENT
		{
		public:
			enum JOINT_TYPE
			{
				ROTATIONAL,
				PRISMATIC,
				UNIVERSAL,
				SPHERICAL
			};

			virtual ~JOINT() = default;

			int GetCstDim() const;

			const MARKER* GetMakI() const{ return _pMakI; };
			const MARKER* GetMakJ() const{ return _pMakJ; };
			const double* GetCstFcePtr() const{ return _CstFce; };
			const double* GetPrtCstMtxIPtr() const{ return *_PrtCstMtxI; };
			const double* GetPrtCstMtxJPtr() const{ return *_PrtCstMtxJ; };
			const double* GetPrtA_cPtr() const{ return _a_c; };
			JOINT_TYPE GetType() const{ return _Type; };

			MARKER* GetMakI() { return _pMakI; };
			MARKER* GetMakJ() { return _pMakJ; };

			void UpdateInPrt();

			virtual unsigned GetID() const;

			virtual void ToXmlElement(Aris::Core::ELEMENT *pEle) const;
			virtual void FromXmlElement(const Aris::Core::ELEMENT *pEle);
			virtual void SaveResult(unsigned id)
			{
				memcpy(result.at(id).fce, _CstFce, fce_size);
			}
			virtual void SetResultSize(unsigned size)
			{
				result.resize(size);
			}
			struct RESULT_NODE
			{
				bool active;
				double fce[6];
			};

		private:
			std::vector<RESULT_NODE> result;

		private:
			explicit JOINT(MODEL *pModel, const std::string &Name = "", JOINT_TYPE Type = ROTATIONAL, MARKER *pMakI = 0, MARKER *pMakJ = 0);

			JOINT(const JOINT &) = delete;
			JOINT(JOINT &&) = delete;
			JOINT& operator=(const JOINT &) = delete;
			JOINT& operator=(JOINT &&) = delete;

			void _Initiate();

		private:
			JOINT_TYPE _Type;
			MARKER *_pMakI;
			MARKER *_pMakJ;

			int _ColId;

			/*for calculation in part coordinate*/
			double _tm_I2M[6][6];

			double _PrtCstMtxI[6][6];
			double _PrtCstMtxJ[6][6];
			double _CstFce[6];
			double _a_c[6];

			friend class MODEL;
		};
		class MOTION :public ELEMENT
		{
		public:
			enum MOTION_TYPE
			{
				LINEAR
			};
			enum MOTION_MODE
			{
				POS_CONTROL,
				FCE_CONTROL
			};

			virtual ~MOTION() = default;

			MOTION_TYPE GetType() const{ return _Type; };
			MOTION_MODE GetMode() const{ return _Mode; };
			int GetCstDim() const;

			const double* GetFrcCoePtr() const{ return _frc_coe; };
			const double* GetF_mPtr() const{ return _f_m; };
			const double* GetA_mPtr() const{ return _a_m; };
			const double* GetV_mPtr() const{ return _v_m; };
			const double* GetP_mPtr() const{ return _p_m; };
			const double* GetPrtCstMtxIPtr() const{ return *_PrtCstMtxI; };
			const double* GetPrtCstMtxJPtr() const{ return *_PrtCstMtxJ; };
			const double* GetPrtA_cPtr() const{ return _a_c; };

			void SetP_m(const double *p_m){ memcpy(_p_m, p_m, sizeof(double)*GetCstDim()); };
			void SetV_m(const double *v_m){ memcpy(_v_m, v_m, sizeof(double)*GetCstDim()); };
			void SetA_m(const double *a_m){ memcpy(_a_m, a_m, sizeof(double)*GetCstDim()); };
			void SetF_m(const double *f_m){ memcpy(_f_m, f_m, sizeof(double)*GetCstDim()); };
			void SetMode(MOTION_MODE mode){ _Mode = mode; };
			void SetFrcCoe(const double *frc_coe){ memcpy(_frc_coe, frc_coe, sizeof(double) * 3 * GetCstDim()); };

			void UpdateInPrt();

			virtual unsigned GetID() const;

			double FceAkima(double t, char derivativeOrder = '0'){ return fceCurve->operator()(t, derivativeOrder); };
			double PosAkima(double t, char derivativeOrder = '0'){ return posCurve->operator()(t, derivativeOrder); };
			void FceAkima(unsigned length, const double *t, double *fce, char order = '0'){ fceCurve->operator()(length, t, fce, order); };
			void PosAkima(unsigned length, const double *t, double *pos, char order = '0'){ posCurve->operator()(length, t, pos, order); };

			void SetPosAkimaCurve(const unsigned num, const double* time, const double *pos)
			{
				this->posCurve.reset(new AKIMA(num, time, pos));
			}
			void SetFceAkimaCurve(const unsigned num, const double* time, const double *fce)
			{
				this->fceCurve.reset(new AKIMA(num, time, fce));
			}

			virtual void ToXmlElement(Aris::Core::ELEMENT *pEle) const;
			virtual void FromXmlElement(const Aris::Core::ELEMENT *pEle);

			virtual void SaveResult(unsigned id)
			{
				memcpy(result.at(id).fce, GetF_mPtr(), fce_size);
				memcpy(result.at(id).pos, GetP_mPtr(), pe_size);
				memcpy(result.at(id).vel, GetV_mPtr(), vel_size);
				memcpy(result.at(id).acc, GetA_mPtr(), acc_size);
			}
			virtual void SetResultSize(unsigned size)
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

		private:
			explicit MOTION(MODEL *pModel, const std::string &Name = "", MOTION_TYPE type = LINEAR, MOTION_MODE mode = POS_CONTROL, MARKER *pMakI = 0, MARKER *pMakJ = 0);

			MOTION(const MOTION &) = delete;
			MOTION(MOTION &&) = delete;
			MOTION& operator=(const MOTION &) = delete;
			MOTION& operator=(MOTION &&) = delete;

			void _Initiate();

		private:
			MOTION_TYPE _Type;
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
		class FORCE :public ELEMENT
		{
		public:
			enum FORCE_TYPE
			{
				BODY2BODY
			};

			virtual ~FORCE() = default;

			void Update();

			double* GetFceMtxPtr() const;
			void SetFce(const double* pFce);

			virtual unsigned GetID() const;

			virtual void ToXmlElement(Aris::Core::ELEMENT *pEle) const{};
			virtual void FromXmlElement(const Aris::Core::ELEMENT *pEle){};

		private:
			explicit FORCE(MODEL *pModel, const std::string &Name = "", FORCE_TYPE type = BODY2BODY, PART *pPrtI = 0, PART *pPrtJ = 0, MARKER *pMakA = 0, MARKER *pMakP = 0, const double *force = 0);

			FORCE(const FORCE &) = delete;
			FORCE(FORCE &&) = delete;
			FORCE& operator=(const FORCE &) = delete;
			FORCE& operator=(FORCE &&) = delete;

		private:
			FORCE_TYPE _Type;

			PART *_pPrtI;
			PART *_pPrtJ;
			MARKER *_pMakA;
			MARKER *_pMakP;

			double _LocFce[6];
			double _Fce[6];

			friend class MODEL;
			friend struct std::pair < const std::string, FORCE >;
			friend struct std::pair < std::string, FORCE >;
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
			void ScriptActivate(unsigned time, JOINT * jnt, const double *peMakI=nullptr, const double *peMakJ=nullptr)
			{
				JOINT_STRUCT jnt_struct;
				jnt_struct.active = true;

				jnt_struct.isModifyMakI = peMakI!=nullptr;
				jnt_struct.isModifyMakJ = peMakJ!=nullptr;

				if (peMakI)
					memcpy(jnt_struct.peMakI, peMakI, pe_size);
				if (peMakJ)
					memcpy(jnt_struct.peMakJ, peMakJ, pe_size);
				
				script[time].joints[jnt] = jnt_struct;
			};
			void ScriptDeactivate(unsigned time, JOINT * jnt)
			{
				JOINT_STRUCT jnt_struct;
				jnt_struct.active = false;
				script[time].joints[jnt] = jnt_struct;
			}
			void ScriptSwitchMode(unsigned time, MOTION * ele, MOTION::MOTION_MODE mode)
			{
				script[time].motions[ele] = mode;
			}
			void ScriptClear(){ script.clear(); };
			void ScriptEndTime(unsigned endTime){ this->endTime = endTime; };
			void ScriptDt(unsigned dt){ this->dt = dt; };

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
				std::map<JOINT *, JOINT_STRUCT> joints;
				std::map<MOTION *, MOTION::MOTION_MODE> motions;
			};
			std::map < unsigned/*time*/, NODE > script;

		private:
			unsigned endTime{0};
			unsigned dt{ 1 };


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
			JOINT* AddJoint(const std::string & Name
				, Aris::DynKer::JOINT::JOINT_TYPE type = JOINT::ROTATIONAL
				, MARKER* pmaki = nullptr
				, MARKER* pmakj = nullptr);
			MOTION* AddMotion(const std::string & Name
				, Aris::DynKer::MOTION::MOTION_TYPE type = MOTION::LINEAR
				, MOTION::MOTION_MODE mode = MOTION::POS_CONTROL
				, MARKER *pMakI = nullptr
				, MARKER *pMakJ = nullptr);
			FORCE* AddForce(const std::string & Name
				, FORCE::FORCE_TYPE Type = FORCE::BODY2BODY
				, PART *pprti = nullptr
				, PART *pprtj = nullptr
				, MARKER *pmaka = nullptr
				, MARKER *pmakp = nullptr
				, const double *fce = nullptr);


			const PART *GetPart(unsigned id) const;
			const JOINT *GetJoint(unsigned id) const;
			const MOTION *GetMotion(unsigned id) const;
			const FORCE *GetForce(unsigned id) const;
			const MARKER *GetMarker(unsigned id) const;
			PART *GetPart(unsigned id);
			JOINT *GetJoint(unsigned id);
			MOTION *GetMotion(unsigned id);
			FORCE *GetForce(unsigned id);
			MARKER *GetMarker(unsigned id);
			const PART *GetPart(const std::string &Name)const;
			const JOINT *GetJoint(const std::string &Name)const;
			const MOTION *GetMotion(const std::string &Name)const;
			const FORCE *GetForce(const std::string &Name)const;
			PART *GetPart(const std::string &Name);
			JOINT *GetJoint(const std::string &Name);
			MOTION *GetMotion(const std::string &Name);
			FORCE *GetForce(const std::string &Name);
			unsigned GetPartNum() const{ return _parts.size(); };
			unsigned GetMotionNum() const{ return _motions.size(); };
			unsigned GetJointNum() const{ return _joints.size(); };
			unsigned GetForceNum() const{ return _forces.size(); };
			unsigned GetMarkerNum() const{ return _markers.size(); };

			void ForEachPart(std::function<void(PART*)> fun)
			{
				for (auto&i : _parts)
				{
					fun(i.get());
				}
			}
			void ForEachMotion(std::function<void(MOTION*)> fun)
			{
				for (auto&i : _motions)
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
			void ForEachForce(std::function<void(FORCE*)> fun)
			{
				for (auto&i : _forces)
				{
					fun(i.get());
				}
			}
			void ForEachJoint(std::function<void(JOINT*)> fun)
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

			void ClbEqnTo(double *&clb_d_ptr, double *&clb_b_ptr, unsigned int &clb_dim_m, unsigned int &clb_dim_n);

			void LoadXML(const char *filename);
			void SaveSnapshotXML(const char *filename) const;
			virtual void SaveAdams(const char *filename, SIMULATE_SCRIPT* pScript=nullptr) const;

		private:
			MODEL(const MODEL &) = delete;
			MODEL(MODEL &&) = delete;
			MODEL& operator=(const MODEL &) = delete;
			MODEL& operator=(MODEL &&) = delete;

			Aris::Core::DOCUMENT XML_Doc;
			CALCULATOR calculator;
			
			unsigned int C_dim;//real dimension of constraint matrix
			unsigned int I_dim;//real dimension of inertia matrix

		protected:
			std::vector<std::unique_ptr<PART> > _parts;
			std::vector<std::unique_ptr<JOINT> > _joints;
			std::vector<std::unique_ptr<MOTION> > _motions;
			std::vector<std::unique_ptr<FORCE> > _forces;
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
			friend class MOTION;
			friend class JOINT;
			friend class MARKER;
			friend class FORCE;
		};
	}
}




























#endif
