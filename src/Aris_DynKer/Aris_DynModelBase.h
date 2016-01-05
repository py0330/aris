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
		typedef double double6x6[6][6];
		typedef double double4x4[4][4];
		typedef double double3[3];
		typedef double double6[6];
		
		class Part;
		class Marker;
		class JointBase;
		class MotionBase;
		class ForceBase;

		class Environment;
		class ModelBase;

		class Object
		{
		public:
			const std::string& Name() const { return name_; };
			const ModelBase& Model()const { return model_; };
			ModelBase& Model() { return model_; };
		
		protected:
			virtual ~Object() {};
			explicit Object(ModelBase &model, const std::string &name) :model_(model), name_(name) {};

		private:
			Object(const Object &) = delete;
			Object(Object &&) = delete;
			Object &operator=(const Object &) = delete;
			Object &operator=(Object &&) = delete;

			ModelBase &model_;
			std::string name_;
		};
		class Element :public Object
		{
		public:
			bool IsActive() const{ return is_active_; };
			void Activate(bool isActive = true) { is_active_ = isActive; };
			int ID() const { return id_; };

		private:
			virtual void ToXmlElement(Aris::Core::XmlElement *pEle) const = 0;
			virtual void ToAdamsCmd(std::ofstream &file) const = 0;

		protected:
			virtual ~Element() = default;
			explicit Element(ModelBase &model, const std::string &name, int id) : Object(model, name), id_(id) {};

		private:
			bool is_active_{true};
			const int id_;

			friend class ModelBase;
		};
		
		class Interaction :public Element
		{
		public:
			const Marker& MakI() const { return makI_; };
			Marker& MakI() { return makI_; };
			const Marker& MakJ() const { return makJ_; };
			Marker& MakJ() { return makJ_; };

		protected:
			virtual ~Interaction() = default;
			explicit Interaction(ModelBase &model, const std::string &name, int id, Marker &makI, Marker &makJ)
				: Element(model, name, id), makI_(makI), makJ_(makJ) {};
			explicit Interaction(ModelBase &model, const std::string &name, int id, const Aris::Core::XmlElement *ele);

		private:
			Marker &makI_;
			Marker &makJ_;
		};
		class Constraint :public Interaction
		{
		public:
			virtual const char* Type() const = 0;
			virtual int CstDim() const = 0;
			virtual const double* PrtCstMtxI() const = 0;
			virtual const double* PrtCstMtxJ() const = 0;
			virtual const double* PrtAc() const = 0;
			virtual const double* CstFce() const = 0;

		protected:
			virtual ~Constraint() = default;
			explicit Constraint(ModelBase &model, const std::string &name, int id, Marker &makI, Marker &makJ)
				: Interaction(model, name, id, makI, makJ) {};
			explicit Constraint(ModelBase &model, const std::string &name, int id, const Aris::Core::XmlElement *ele)
				: Interaction(model, name, id, ele) {};

		private:
			virtual void Update();
			virtual void Init() = 0;
		};





		class Marker :public Element
		{
		public:
			virtual ~Marker() = default;
			virtual void Update();
			
			const ModelBase& Model() { return Object::Model(); };

			const double4x4& Pm() const { return pm_; };
			const double6& Vel() const;
			const double6& Acc() const;
			const double4x4& PrtPm() const { return prt_pm_; };
			const Part& Father() const { return prt_; };

			void GetPm(double *pm) { std::copy_n(static_cast<const double *>(*Pm()), 16, pm); };
			void GetPe(double *pe, const char *type = "313")const { s_pm2pe(*Pm(), pe, type); };
			void GetPq(double *pq)const { s_pm2pq(*Pm(), pq); };

			explicit Marker(const Part &prt, const double *prtPe = nullptr, const char* eulType = "313");//for constructing marker outside model
		private:
			explicit Marker(ModelBase &model, Part &prt, const std::string &name, int id);//only for child class Part to construct
			explicit Marker(Part &prt, const std::string &name, int id, const double *pPrtPm = nullptr, Marker *pRelativeTo = nullptr);
			explicit Marker(Part &prt, const std::string &name, int id, const Aris::Core::XmlElement *ele);
			virtual void ToXmlElement(Aris::Core::XmlElement *pEle) const;
			virtual void ToAdamsCmd(std::ofstream &file) const {};

		private:
			const Part &prt_;

			double pm_[4][4];
			double prt_pm_[4][4];

			friend class Part;
			friend class ModelBase;
		};
		class Part :public Marker
		{
		public:
			virtual ~Part() = default;
			virtual void Update();
			const ModelBase& Model()const { return Object::Model(); };
			ModelBase& Model() { return Object::Model(); };

			const double4x4& Pm()const { return pm_; };
			double4x4& Pm() { return pm_; };
			const double6& Vel()const { return vel_; };
			double6& Vel() { return vel_; };
			const double6& Acc()const { return acc_; };
			double6& Acc() { return acc_; };
			const double4x4& InvPm() const { return inv_pm_; };
			const double6x6& PrtIm() const { return prt_im_; };
			const double6& PrtVel() const { return prt_vel_; };
			const double6& PrtAcc() const { return prt_acc_; };
			const double6& PrtFg() const { return prt_fg_; };
			const double6& PrtFv() const { return prt_fv_; };
			const double6& PrtGravity() const { return prt_gravity_; };

			void SetPm(const double *pPm) { std::copy_n(pPm, 16, static_cast<double*>(*pm_)); };
			void SetVel(const double *pVel) { std::copy_n(pVel, 6, Vel()); };
			void SetAcc(const double *pAcc) { std::copy_n(pAcc, 6, Acc()); };
			
			Marker* FindMarker(const std::string &Name);
			const Marker* FindMarker(const std::string &Name)const;

			template<typename ...Args>
			Marker* AddMarker(const std::string & Name, Args ...args);

		private:
			explicit Part(ModelBase &model, const std::string &name, int id, const double *prtIm = nullptr
				, const double *pm = nullptr, const double *vel = nullptr, const double *acc = nullptr);
			explicit Part(ModelBase &model, const std::string &name, int id, const Aris::Core::XmlElement *ele);
			virtual void ToXmlElement(Aris::Core::XmlElement *pEle) const;
			virtual void ToAdamsCmd(std::ofstream &file) const;

		private:
			std::map<std::string, int> marker_names_;

		private:
			double inv_pm_[4][4]{ { 0 } };
			double vel_[6]{ 0 };
			double acc_[6]{ 0 };

			double prt_im_[6][6]{ { 0 } };
			double prt_gravity_[6]{ 0 };
			double prt_acc_[6]{ 0 };
			double prt_vel_[6]{ 0 };
			double prt_fg_[6]{ 0 };
			double prt_fv_[6]{ 0 };


		private:
			int row_id_;

		private:
			std::string graphic_file_path_;

			friend class Marker;
			friend class ModelBase;
		};
		class JointBase :public Element
		{
		public:
			virtual ~JointBase() = default;
			virtual int CstDim() const = 0;
			virtual const char* GetType() const = 0;
			virtual double* CstFce() = 0;
			virtual double* PrtCstMtxI() = 0;
			virtual double* PrtCstMtxJ() = 0;
			virtual double* PrtAc() = 0;

			virtual void Update();

			const Marker& MakI() const { return mak_i_; };
			Marker& MakI() { return mak_i_; };
			const Marker& MakJ() const { return mak_j_; };
			Marker& MakJ() { return mak_j_; };

		protected:
			explicit JointBase(ModelBase &model, const std::string &name, int id, Marker &makI, Marker &makJ);
			explicit JointBase(ModelBase &model, const std::string &name, int id, const Aris::Core::XmlElement *ele);
			virtual void ToXmlElement(Aris::Core::XmlElement *pEle) const;
			virtual void ToAdamsCmd(std::ofstream &file) const;
			virtual void Init() = 0;
		private:
			Marker &mak_i_;
			Marker &mak_j_;

			int _ColId;

		private:
			friend class ModelBase;
		};
		class MotionBase :public Element
		{
		public:
			virtual ~MotionBase() = default;

			const Marker& MakI() const { return *_pMakI; };
			Marker& MakI() { return *_pMakI; };
			const Marker& MakJ() const { return *_pMakJ; };
			Marker& MakJ() { return *_pMakJ; };

			virtual const char* GetType() const = 0;
			const double* GetFrcCoePtr() const { return _frc_coe; };
			double MotPos() const { return motPos; };
			double MotVel() const { return motVel; };
			double MotAcc() const { return motAcc; };
			double MotFce() const { return MotFceDyn() + MotFceFrc();	};
			double MotFceFrc() const { return s_sgn(motVel)*_frc_coe[0] + motVel*_frc_coe[1] + motAcc*_frc_coe[2]; };
			double MotFceDyn() const { return motDynFce; }
			
			const double* PrtCstMtxI() const { return _PrtCstMtxI; };
			const double* PrtCstMtxJ() const { return _PrtCstMtxJ; };
			const double* PrtAc() const { return _a_c; };

			void SetFrcCoe(const double *frc_coe) { std::copy_n(frc_coe, 3, _frc_coe); };
			void SetMotAcc(double motAcc) { this->motAcc = motAcc; };

			virtual void Update() = 0;

			void SetPosAkimaCurve(const int num, const double* time, const double *pos)
			{
				this->posCurve.reset(new Akima(num, time, pos));
			}
			double PosAkima(double t, char derivativeOrder = '0') { return posCurve->operator()(t, derivativeOrder); };
			void PosAkima(int length, const double *t, double *pos, char order = '0') { posCurve->operator()(length, t, pos, order); };

		protected:
			explicit MotionBase(ModelBase &model, const std::string &Name, int id, Marker &makI, Marker &makJ);
			explicit MotionBase(ModelBase &model, const std::string &Name, int id, const Aris::Core::XmlElement *ele);
			virtual void ToXmlElement(Aris::Core::XmlElement *pEle) const;
			virtual void ToAdamsCmd(std::ofstream &file) const;
			virtual void Init() = 0;

		protected:
			Marker *_pMakI, *_pMakJ;

			/*pos\vel\acc\fce of motion*/
			double motPos{ 0 }, motVel{ 0 }, motAcc{ 0 }, motDynFce{ 0 };

			double _frc_coe[3]{0};

			int _ColId{ 0 };

			double _PrtCstMtxI[6]{ 0 };
			double _PrtCstMtxJ[6]{ 0 };
			double _a_c[6]{ 0 };

			/*for adams*/
			std::unique_ptr<Akima> posCurve;

			friend class ModelBase;
		};
		class ForceBase :public Element
		{
		public:
			const Marker& MakI() const { return *_pMakI; };
			Marker& MakI() { return *_pMakI; };
			const Marker& MakJ() const { return *_pMakJ; };
			Marker& MakJ() { return *_pMakJ; };

			const double* GetPrtFceIPtr() const { return _PrtFceI; };
			const double* GetPrtFceJPtr() const { return _PrtFceJ; };
			
			void SetFceAkimaCurve(const int num, const double* time, const double *fce)
			{
				this->fceCurve.reset(new Akima(num, time, fce));
			}
			double FceAkima(double t, char derivativeOrder = '0') { return fceCurve->operator()(t, derivativeOrder); };
			void FceAkima(int length, const double *t, double *pos, char order = '0') { fceCurve->operator()(length, t, pos, order); };

			virtual ~ForceBase() = default;
			virtual void Update() = 0;

		protected:
			explicit ForceBase(ModelBase &model, const std::string &Name, int id, Marker &makI, Marker &makJ);
			explicit ForceBase(ModelBase &model, const std::string &Name, int id, const Aris::Core::XmlElement *pEle);
			virtual void ToXmlElement(Aris::Core::XmlElement *pEle) const {};
			virtual void ToAdamsCmd(std::ofstream &file) const {};

			Marker *_pMakI, *_pMakJ;

			double _PrtFceI[6]{ 0 };
			double _PrtFceJ[6]{ 0 };

			std::unique_ptr<Akima> fceCurve;

			friend class ModelBase;
		};

		template<int DIMENSION>
		class JointBaseDim :public JointBase
		{
		public:
			static constexpr int Dim() { return DIMENSION; };
			virtual int CstDim() const { return DIMENSION; };
			virtual double* CstFce() { return _CstFce; };
			virtual double* PrtCstMtxI() { return *_PrtCstMtxI; };
			virtual double* PrtCstMtxJ() { return *_PrtCstMtxJ; };
			virtual double* PrtAc() { return _a_c; };
			virtual const double* CstFce() const { return _CstFce; };
			virtual const double* PrtCstMtxI() const { return *_PrtCstMtxI; };
			virtual const double* PrtCstMtxJ() const { return *_PrtCstMtxJ; };
			virtual const double* PrtAc() const { return _a_c; };

		protected:
			explicit JointBaseDim(ModelBase &model, const std::string &Name, int id, Marker &pMakI, Marker &pMakJ)
				:JointBase(model, Name, id, pMakI, pMakJ) {};
			explicit JointBaseDim(ModelBase &model, const std::string &Name, int id, const Aris::Core::XmlElement *ele)
				:JointBase(model, Name, id, ele) {};

		protected:
			double _PrtCstMtxI[6][DIMENSION]{ { 0 } };
			double _PrtCstMtxJ[6][DIMENSION]{ { 0 } };
			double _CstFce[DIMENSION]{ 0 };
			double _a_c[DIMENSION]{ 0 };

		private:
			friend class ModelBase;
		};

		class Environment:public Object
		{
		private:
			explicit Environment(ModelBase &model);
			~Environment();

			void ToXmlElement(Aris::Core::XmlElement *pEle) const;
			void FromXmlElement(const Aris::Core::XmlElement *pEle);
			void ToAdamsCmd(std::ofstream &file) const;
		private:
			double Gravity[6]{ 0, -9.8, 0, 0, 0, 0 };

			friend class Part;
			friend class ModelBase;
		};
		class SimulateScript
		{
		public:
			void ScriptActivate(int time, Element * ele)
			{
				script[time].elements[ele] = true;
			};
			void ScriptDeactivate(int time, Element * ele)
			{
				script[time].elements[ele] = false;
			}
			void ScriptMoveMarker(int time, Marker * ele, double * data)
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
				std::map<Element *, bool> elements;
				std::map<Marker *, std::array<double, 6> > markers;
			};
			std::map < int/*time*/, NODE > script;

		private:
			int endTime{ 0 };
			int dt{ 1 };


			friend class ModelBase;
		};

		class ModelBase :public Object
		{
		public:
			explicit ModelBase(const std::string & Name = "Model");
			virtual ~ModelBase();

			template<typename ...Args>
			Part* AddPart(const std::string & Name, Args ...args)
			{
				if (GetPart(Name) != nullptr)
				{
					return nullptr;
				}

				_parts.push_back(std::unique_ptr<Part>(new Part(*this, Name, _parts.size(), args...)));
				return _parts.back().get();
			}
			template<typename JOINT, typename ...Args>
			JOINT* AddJoint(const std::string & Name, Args ...args)
			{
				if (GetJoint(Name) != nullptr)
				{
					return nullptr;
				}

				auto ret = new JOINT(*this, Name, _joints.size(), args...);
				_joints.push_back(std::unique_ptr<JointBase>(ret));
				return ret;
			}
			template<typename MOTION, typename ...Args>
			MOTION* AddMotion(const std::string & Name, Args ...args)
			{
				if (GetMotion(Name) != nullptr)
				{
					return nullptr;
				}

				auto ret = new MOTION(*this, Name, _motions.size(), args...);
				_motions.push_back(std::unique_ptr<MotionBase>(ret));
				return ret;
			};
			template<typename FORCE, typename ...Args>
			FORCE* AddForce(const std::string & Name, Args ...args)
			{
				if (GetForce(Name) != nullptr)
				{
					return nullptr;
				}
				auto ret = new FORCE(*this, Name, _forces.size(), args...);
				_forces.push_back(std::unique_ptr<ForceBase>(ret));
				return ret;
			}

			const Part *GetPart(int id) const;
			const JointBase *GetJoint(int id) const;
			const MotionBase *GetMotion(int id) const;
			const ForceBase *GetForce(int id) const;
			const Marker *GetMarker(int id) const;
			Part *GetPart(int id);
			JointBase *GetJoint(int id);
			MotionBase *GetMotion(int id);
			ForceBase *GetForce(int id);
			Marker *GetMarker(int id);
			const Part *GetPart(const std::string &Name)const;
			const JointBase *GetJoint(const std::string &Name)const;
			const MotionBase *GetMotion(const std::string &Name)const;
			const ForceBase *GetForce(const std::string &Name)const;
			Part *GetPart(const std::string &Name);
			JointBase *GetJoint(const std::string &Name);
			MotionBase *GetMotion(const std::string &Name);
			ForceBase *GetForce(const std::string &Name);
			int GetPartNum() const{ return _parts.size(); };
			int GetMotionNum() const{ return _motions.size(); };
			int GetJointNum() const{ return _joints.size(); };
			int GetForceNum() const{ return _forces.size(); };
			int GetMarkerNum() const{ return _markers.size(); };

			void ForEachPart(std::function<void(Part*)> fun)
			{
				for (auto&i : _parts)
				{
					fun(i.get());
				}
			}
			void ForEachMarker(std::function<void(Aris::DynKer::Marker*)> fun)
			{
				for (auto&i : _markers)
				{
					fun(i.get());
				}
			}
			void ForEachMotion(std::function<void(MotionBase*)> fun)
			{
				for (auto&i : _motions)
				{
					fun(i.get());
				}
			}
			void ForEachForce(std::function<void(ForceBase*)> fun)
			{
				for (auto&i : _forces)
				{
					fun(i.get());
				}
			}
			void ForEachJoint(std::function<void(JointBase*)> fun)
			{
				for (auto&i : _joints)
				{
					fun(i.get());
				}
			}
			void ForEachElement(std::function<void(Element*)> fun)
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

			virtual void LoadXml(const char *filename);
			virtual void LoadXml(const Aris::Core::XmlDocument &xmlDoc);
			virtual void LoadXml(const Aris::Core::XmlElement *pEle);
			void SaveSnapshotXml(const char *filename) const;
			void SaveAdams(const char *filename, const SimulateScript* pScript) const;
			void SaveAdams(const char *filename, bool isModifyActive = true) const;

		private:
			Calculator calculator;
			
			int C_dim;//real dimension of constraint matrix
			int I_dim;//real dimension of inertia matrix

		protected:
			template<typename CONSTRAINT>
			void InitiateElement(CONSTRAINT *pConstraint) { pConstraint->Init(); };

			std::vector<std::unique_ptr<Part> > _parts;
			std::vector<std::unique_ptr<JointBase> > _joints;
			std::vector<std::unique_ptr<MotionBase> > _motions;
			std::vector<std::unique_ptr<ForceBase> > _forces;
			std::vector<std::unique_ptr<Aris::DynKer::Marker> > _markers;
			
			Environment _Environment;
			Part* pGround;
		
			friend class Environment;
			friend class Part;
			friend class MotionBase;
			friend class JOINT;
			friend class Marker;
			friend class ForceBase;
		};

		template<typename ...Args>
		Marker* Part::AddMarker(const std::string & Name, Args ...args)
		{
			if (FindMarker(Name))return nullptr;

			Model()._markers.push_back(std::unique_ptr<Aris::DynKer::Marker>(new Aris::DynKer::Marker(*this, Name, Model()._markers.size(), args...)));
			marker_names_.insert(std::make_pair(Name, Model()._markers.size() - 1));
			return Model()._markers.back().get();
		}
	}
}

#endif
