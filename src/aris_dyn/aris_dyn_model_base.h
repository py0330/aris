#ifndef Aris_DynModelBase_H
#define Aris_DynModelBase_H

#include <vector>
#include <array>
#include <map>
#include <string>
#include <memory>
#include <functional>
#include <algorithm>

#include "aris_xml.h"
#include "aris_exp_cal.h"
#include "aris_dyn_kernel.h"

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

		struct PlanParamBase
		{
			std::int32_t cmd_type{ 0 };
			std::int32_t cmd_ID{ 0 };
			mutable std::int32_t count{ 0 };
		};
		typedef std::function<int(ModelBase &, const PlanParamBase &)> PlanFunc;

		struct SimResult
		{
			std::list<std::array<double, 6> > begin_prt_pe_;
			
			std::list<double> time_;
			std::vector<std::list<double> > Pin_, Fin_, Vin_, Ain_;//vector18维，但list维数为时间的维数
		
			void SaveToFile(const std::string &filename) 
			{
				auto f_name = filename + "_Fin.txt";
				auto p_name = filename + "_Pin.txt";
				auto v_name = filename + "_Vin.txt";
				auto a_name = filename + "_Ain.txt";

				dlmwrite(f_name.c_str(), Fin_);
				dlmwrite(p_name.c_str(), Pin_);
				dlmwrite(v_name.c_str(), Vin_);
				dlmwrite(a_name.c_str(), Ain_);
			};
		};

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
			virtual std::string GroupName() = 0;

		private:
			virtual void ToXmlElement(Aris::Core::XmlElement &xml_ele) const = 0;
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
			virtual std::string AdamsType() const = 0;
			const Marker& MakI() const { return *makI_; };
			Marker& MakI() { return *makI_; };
			const Marker& MakJ() const { return *makJ_; };
			Marker& MakJ() { return *makJ_; };

		protected:
			virtual ~Interaction() = default;
			explicit Interaction(ModelBase &model, const std::string &name, int id, Marker &makI, Marker &makJ)
				: Element(model, name, id), makI_(&makI), makJ_(&makJ) {};
			explicit Interaction(ModelBase &model, const std::string &name, int id, const Aris::Core::XmlElement &xml_ele);

		private:
			Marker *makI_;
			Marker *makJ_;
		};
		class Constraint :public Interaction
		{
		public:
			virtual int CstDim() const = 0;
			virtual void Update();
			virtual void ToXmlElement(Aris::Core::XmlElement &xml_ele) const override;
			virtual void ToAdamsCmd(std::ofstream &file) const override;

			const double* CstMtxI() const { return const_cast<Constraint*>(this)->CsmI(); };
			const double* CstMtxJ() const { return const_cast<Constraint*>(this)->CsmJ(); };
			const double* CstAcc() const { return const_cast<Constraint*>(this)->Csa(); };
			const double* CstFce() const { return const_cast<Constraint*>(this)->Csf(); };

		protected:
			virtual ~Constraint() = default;
			explicit Constraint(ModelBase &model, const std::string &name, int id, Marker &makI, Marker &makJ)
				: Interaction(model, name, id, makI, makJ) {};
			explicit Constraint(ModelBase &model, const std::string &name, int id, const Aris::Core::XmlElement &xml_ele)
				: Interaction(model, name, id, xml_ele) {};

		private:
			virtual void Init() = 0;
			virtual double* CsmI() = 0;
			virtual double* CsmJ() = 0;
			virtual double* Csa() = 0;
			virtual double* Csf() = 0;

			int col_id_;

			friend class ModelBase;
			friend class Model;
		};

		class Marker :public Element
		{
		public:
			virtual ~Marker() = default;
			virtual void Update();
			virtual std::string GroupName()override { return "marker"; };

			const ModelBase& Model() { return Object::Model(); };

			const double4x4& Pm() const { return pm_; };
			const double6& Vel() const;
			const double6& Acc() const;
			const double4x4& PrtPm() const { return prt_pm_; };
			const Part& Father() const { return prt_; };

			void GetPm(double *pm) { std::copy_n(static_cast<const double *>(*Pm()), 16, pm); };
			void GetPe(double *pe, const char *type = "313")const { s_pm2pe(*Pm(), pe, type); };
			void GetPq(double *pq)const { s_pm2pq(*Pm(), pq); };
			void GetVel(double *vel)const { std::copy_n(Vel(), 6, vel); };
			void GetAcc(double *acc)const { std::copy_n(Acc(), 6, acc); };

			explicit Marker(const Part &prt, const double *prtPe = nullptr, const char* eulType = "313");//for constructing marker outside model
		private:
			explicit Marker(ModelBase &model, Part &prt, const std::string &name, int id);//only for child class Part to construct
			explicit Marker(Part &prt, const std::string &name, int id, const double *pPrtPm = nullptr, Marker *pRelativeTo = nullptr);
			explicit Marker(Part &prt, const std::string &name, int id, const Aris::Core::XmlElement *ele);
			virtual void ToXmlElement(Aris::Core::XmlElement &xml_ele) const;
			virtual void ToAdamsCmd(std::ofstream &file) const {};

		private:
			const Part &prt_;

			double pm_[4][4];
			double prt_pm_[4][4];

			friend class Part;
			friend class FloatMarker;
			friend class Script;
			friend class ModelBase;
		};
		class Part :public Marker
		{
		public:
			virtual ~Part() = default;
			virtual void Update();
			virtual std::string GroupName()override { return "part"; };
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

			void SetPm(const double *pm) { std::copy_n(pm, 16, static_cast<double*>(*pm_)); };
			void SetPe(const double *pe, const char *type = "313") { s_pe2pm(pe, *Pm(), type); };
			void SetPq(const double *pq) { s_pq2pm(pq, *Pm()); };
			void SetVel(const double *vel) { std::copy_n(vel, 6, Vel()); };
			void SetAcc(const double *acc) { std::copy_n(acc, 6, Acc()); };
			
			Marker* FindMarker(const std::string &name);
			const Marker* FindMarker(const std::string &name)const;

			template<typename ...Args>
			Marker* AddMarker(const std::string & name, Args ...args);

		private:
			explicit Part(ModelBase &model, const std::string &name, int id, const double *prtIm = nullptr
				, const double *pm = nullptr, const double *vel = nullptr, const double *acc = nullptr);
			explicit Part(ModelBase &model, const std::string &name, int id, const Aris::Core::XmlElement *ele);
			virtual void ToXmlElement(Aris::Core::XmlElement &xml_ele) const;
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
		class JointBase :public Constraint
		{
		public:
			virtual ~JointBase() = default;
			virtual std::string GroupName()override { return "joint"; };

		protected:
			explicit JointBase(ModelBase &model, const std::string &name, int id, Marker &makI, Marker &makJ)
				: Constraint(model, name, id, makI, makJ) {};
			explicit JointBase(ModelBase &model, const std::string &name, int id, const Aris::Core::XmlElement &xml_ele)
				: Constraint(model, name, id, xml_ele) {};

			friend class ModelBase;
		};
		class MotionBase :public Constraint
		{
		public:
			virtual ~MotionBase() = default;
			virtual std::string GroupName()override { return "motion"; };
			virtual int CstDim() const override { return 1; };

			const double* FrcCoe() const { return frc_coe_; };
			void SetFrcCoe(const double *frc_coe) { std::copy_n(frc_coe, 3, frc_coe_); };

			double MotPos() const { return mot_pos_; };
			double MotVel() const { return mot_vel_; };
			double MotAcc() const { return mot_acc_; };
			double MotFce() const { return MotFceDyn() + MotFceFrc();	};
			double MotFceFrc() const { return s_sgn(mot_vel_)*frc_coe_[0] + mot_vel_*frc_coe_[1] + mot_acc_*frc_coe_[2]; };
			double MotFceDyn() const { return mot_fce_dyn_; }
			void SetMotAcc(double mot_acc) { this->mot_acc_ = mot_acc; };
			void SetMotFce(double mot_fce) { this->mot_fce_ = mot_fce; };

			void SetPosAkimaCurve(const int num, const double* time, const double *pos)
			{
				this->posCurve.reset(new Akima(num, time, pos));
			}
			double PosAkima(double t, char derivativeOrder = '0') { return posCurve->operator()(t, derivativeOrder); };
			void PosAkima(int length, const double *t, double *pos, char order = '0') { posCurve->operator()(length, t, pos, order); };

		protected:
			explicit MotionBase(ModelBase &model, const std::string &name, int id, Marker &makI, Marker &makJ);
			explicit MotionBase(ModelBase &model, const std::string &name, int id, const Aris::Core::XmlElement &xml_ele);
			virtual void ToXmlElement(Aris::Core::XmlElement &xml_ele) const override;

		protected:
			double* CsmI() override { return csmI_; };
			double* CsmJ() override { return csmJ_; };
			double* Csa() override { return &csa_; };
			double* Csf() override { return &mot_fce_dyn_; };
			void SetMotFceDyn(double mot_dyn_fce) { this->mot_fce_dyn_ = mot_dyn_fce; };

			/*pos\vel\acc\fce of motion*/
			double mot_pos_{ 0 }, mot_vel_{ 0 }, mot_acc_{ 0 }, mot_fce_dyn_{ 0 };
			double mot_fce_{ 0 };//仅仅用于标定
			double frc_coe_[3]{ 0 };

			double csmI_[6]{ 0 };
			double csmJ_[6]{ 0 };
			double csa_{ 0 };

			/*for adams*/
			std::unique_ptr<Akima> posCurve;

			friend class ModelBase;
		};
		class ForceBase :public Interaction
		{
		public:
			virtual ~ForceBase() = default;
			virtual std::string GroupName()override { return "force"; };
			const double* FceI() const { return fceI_; };
			const double* FceJ() const { return fceJ_; };
			
			void SetFceAkimaCurve(const int num, const double* time, const double *fce)
			{
				this->fceCurve.reset(new Akima(num, time, fce));
			}
			double FceAkima(double t, char derivativeOrder = '0') { return fceCurve->operator()(t, derivativeOrder); };
			void FceAkima(int length, const double *t, double *pos, char order = '0') { fceCurve->operator()(length, t, pos, order); };

			
			virtual void Update() = 0;

		protected:
			explicit ForceBase(ModelBase &model, const std::string &name, int id, Marker &makI, Marker &makJ);
			explicit ForceBase(ModelBase &model, const std::string &name, int id, const Aris::Core::XmlElement &xml_ele);
			virtual void ToXmlElement(Aris::Core::XmlElement &xml_ele) const {};
			virtual void ToAdamsCmd(std::ofstream &file) const {};

			double fceI_[6]{ 0 };
			double fceJ_[6]{ 0 };

			std::unique_ptr<Akima> fceCurve;

			friend class ModelBase;
		};

		class FloatMarker final:public Marker
		{
		public:
			void SetPrtPm(const double *prtPm) { std::copy_n(prtPm, 16, static_cast<double *>(*prt_pm_)); };
			void SetPrtPe(const double *prtPe, const char *type = "313") { s_pe2pm(prtPe, *prt_pm_, type); };
			void SetPrtPq(const double *prtPq) { s_pq2pm(prtPq, *prt_pm_); };

			explicit FloatMarker(const Part &prt, const double *prtPe = nullptr, const char* eulType = "313")
				:Marker(prt, prtPe, eulType) {};
		};

		template<int DIMENSION>
		class JointBaseDim :public JointBase
		{
		public:
			static constexpr int Dim() { return DIMENSION; };
			virtual int CstDim() const { return DIMENSION; };
			
		protected:
			explicit JointBaseDim(ModelBase &model, const std::string &name, int id, Marker &makI, Marker &makJ)
				:JointBase(model, name, id, makI, makJ) {};
			explicit JointBaseDim(ModelBase &model, const std::string &name, int id, const Aris::Core::XmlElement &xml_ele)
				:JointBase(model, name, id, xml_ele) {};

			virtual double* CsmI() override { return *csmI_; };
			virtual double* CsmJ() override { return *csmJ_; };
			virtual double* Csf() override { return csf_; };
			virtual double* Csa() override { return csa_; };

			double csmI_[6][DIMENSION]{ { 0 } };
			double csmJ_[6][DIMENSION]{ { 0 } };
			double csf_[DIMENSION]{ 0 };
			double csa_[DIMENSION]{ 0 };

		private:
			friend class ModelBase;
		};

		class Environment:public Object
		{
		private:
			explicit Environment(ModelBase &model) :Object(model, "Environment") {};
			~Environment() = default;

			void ToXmlElement(Aris::Core::XmlElement &xml_ele) const;
			void FromXmlElement(const Aris::Core::XmlElement &xml_ele);
			void ToAdamsCmd(std::ofstream &file) const;
		private:
			double Gravity[6]{ 0, -9.8, 0, 0, 0, 0 };

			friend class Part;
			friend class ModelBase;
		};
		class Script
		{
		public:
			void Activate(Element &ele, bool isActive);
			void MoveMarker(Marker &mak_move, const Marker& mak_target);
			void Simulate(std::uint32_t ms_dur, std::uint32_t ms_dt);
			void ToAdamsCmd(std::ofstream &file) const;

			bool Empty() const;
			std::uint32_t EndTime()const;
			void SetTopologyAt(std::uint32_t ms_time);
			void UpdateAt(std::uint32_t ms_time);

			void Clear();
		private:
			class Imp;
			std::unique_ptr<Imp> pImp;

		private:
			explicit Script();
			~Script();
			friend class ModelBase;
		};

		class ModelBase :public Object
		{
		public:
			explicit ModelBase(const std::string & name = "Model");
			virtual ~ModelBase();

			Aris::DynKer::Environment& Environment() { return environment_; };
			const Aris::DynKer::Environment& Environment()const { return environment_; };
			Aris::DynKer::Script& Script() { return script_; };
			const Aris::DynKer::Script& Script()const { return script_; };

			template<typename ...Args>
			Part* AddPart(const std::string & name, Args ...args)
			{
				if (FindPart(name))	return nullptr;
				parts_.push_back(std::unique_ptr<Part>(new Part(*this, name, parts_.size(), args...)));
				return parts_.back().get();
			}
			template<typename Joint, typename ...Args>
			Joint* AddJoint(const std::string & name, Args ...args)
			{
				if (FindJoint(name))return nullptr;
				auto ret = new Joint(*this, name, joints_.size(), args...);
				joints_.push_back(std::unique_ptr<JointBase>(ret));
				return ret;
			}
			template<typename Motion, typename ...Args>
			Motion* AddMotion(const std::string & name, Args ...args)
			{
				if (FindMotion(name))return nullptr;
				auto ret = new Motion(*this, name, motions_.size(), args...);
				motions_.push_back(std::unique_ptr<MotionBase>(ret));
				return ret;
			};
			template<typename Force, typename ...Args>
			Force* AddForce(const std::string & name, Args ...args)
			{
				if (FindForce(name))return nullptr;
				auto ret = new Force(*this, name, forces_.size(), args...);
				forces_.push_back(std::unique_ptr<ForceBase>(ret));
				return ret;
			}
			std::size_t PartNum() const { return parts_.size(); };
			std::size_t MotionNum() const { return motions_.size(); };
			std::size_t JointNum() const { return joints_.size(); };
			std::size_t ForceNum() const { return forces_.size(); };
			std::size_t MarkerNum() const { return markers_.size(); };
			Part &PartAt(int id) { return *parts_.at(id).get(); };
			const Part &PartAt(int id) const { return *parts_.at(id).get(); };
			JointBase &JointAt(int id) { return *joints_.at(id).get(); };
			const JointBase &JointAt(int id) const { return *joints_.at(id).get(); };
			MotionBase &MotionAt(int id) { return *motions_.at(id).get(); };
			const MotionBase &MotionAt(int id) const { return *motions_.at(id).get(); };
			ForceBase &ForceAt(int id) { return *forces_.at(id).get(); };
			const ForceBase &ForceAt(int id) const { return *forces_.at(id).get(); };
			Marker &MarkerAt(int id) { return *markers_.at(id).get(); };
			const Marker &MarkerAt(int id) const { return *markers_.at(id).get(); };
			const Part *FindPart(const std::string &name)const { return const_cast<ModelBase*>(this)->FindPart(name); };
			const JointBase *FindJoint(const std::string &name)const { return const_cast<ModelBase*>(this)->FindJoint(name); };
			const MotionBase *FindMotion(const std::string &name)const { return const_cast<ModelBase*>(this)->FindMotion(name); };
			const ForceBase *FindForce(const std::string &name)const { return const_cast<ModelBase*>(this)->FindForce(name); };
			Part *FindPart(const std::string &name);
			JointBase *FindJoint(const std::string &name);
			MotionBase *FindMotion(const std::string &name);
			ForceBase *FindForce(const std::string &name);

			void ForEachPart(std::function<void(Part*)> fun)
			{
				for (auto&i : parts_)
				{
					fun(i.get());
				}
			}
			void ForEachMarker(std::function<void(Marker*)> fun)
			{
				for (auto&i : markers_)
				{
					fun(i.get());
				}
			}
			void ForEachMotion(std::function<void(MotionBase*)> fun)
			{
				for (auto&i : motions_)
				{
					fun(i.get());
				}
			}
			void ForEachForce(std::function<void(ForceBase*)> fun)
			{
				for (auto&i : forces_)
				{
					fun(i.get());
				}
			}
			void ForEachJoint(std::function<void(JointBase*)> fun)
			{
				for (auto&i : joints_)
				{
					fun(i.get());
				}
			}
			void ForEachElement(std::function<void(Element*)> fun)
			{
				for (auto&i : parts_)
				{
					fun(i.get());
				}
				for (auto&i : markers_)
				{
					fun(i.get());
				}
				for (auto&i : joints_)
				{
					fun(i.get());
				}
				for (auto&i : motions_)
				{
					fun(i.get());
				}
				for (auto&i : forces_)
				{
					fun(i.get());
				}
			}

			/// 约束矩阵C为m x n维的矩阵，惯量矩阵为m x m维的矩阵
			/// 约束力为n维的向量，约束加速度为n维向量
			/// 部件力为m维的向量，部件加速度为m维向量
			/// 动力学为所求的未知量为部件加速度和约束力，其他均为已知
			int DynDimM()const { return dyn_prt_dim_; };
			int DynDimN()const { return dyn_cst_dim_; };
			int DynDim()const { return DynDimN() + DynDimM(); };
			void DynSetSolveMethod(std::function<void(int dim, const double *D, const double *b, double *x)> solve_method);
			void DynCstMtx(double *cst_mtx) const;
			void DynIneMtx(double *ine_mtx) const;
			void DynCstAcc(double *cst_acc) const;
			void DynPrtFce(double *prt_fce) const;
			void DynCstFce(double *cst_fce) const;
			void DynPrtAcc(double *prt_acc) const;
			void DynPre();
			void DynUpd();
			void DynMtx(double *D, double *b) const;
			void DynSov(const double *D, const double *b, double *x) const;
			void DynUkn(double *x) const;
			void DynEnd(const double *x);
			virtual void Dyn();

			/// 标定矩阵为m x n维的矩阵，其中m为驱动的数目，n为部件个数*10+驱动数*3
			int ClbDimM()const { return clb_dim_m_; };
			int ClbDimN()const { return clb_dim_n_; };
			int ClbDimGam()const { return clb_dim_gam_; };
			int ClbDimFrc()const { return clb_dim_frc_; };
			void ClbSetInverseMethod(std::function<void(int n, double *A)> inverse_method);
			void ClbPre();
			void ClbUpd();
			void ClbMtx(double *clb_D, double *clb_b) const;
			void ClbUkn(double *clb_x) const;

			/// 仿真函数
			virtual void KinFromPin() {};
			virtual void KinFromVin() {};
			virtual void KinFromAin() {};
			/// 静态仿真，结果仅仅返回驱动的位置
			void SimKin(const PlanFunc &func, const PlanParamBase &param, SimResult &result, bool using_script = false);
			/// 动态仿真，待完善
			void SimDyn(const PlanFunc &func, const PlanParamBase &param, SimResult &result, bool using_script = false);
			/// 静态仿真，并将结果设置到驱动的Akima函数中
			void SimKinAkima(const PlanFunc &func, const PlanParamBase &param, SimResult &result, int akima_interval = 1, bool using_script = false);
			/// 动态仿真，根据静态得到的Akima插值，计算驱动的速度和加速度，并且计算动力学。依赖KinFrom系列函数
			void SimDynAkima(const PlanFunc &func, const PlanParamBase &param, SimResult &result, int akima_interval = 1, bool using_script = false);
			/// 直接生成Adams模型，依赖SimDynAkima
			void SimToAdams(const std::string &adams_file, const PlanFunc &fun, const PlanParamBase &param, int ms_dt = 10, bool using_script = false);

			void LoadXml(const char* filename) { LoadXml(std::string(filename)); };
			void LoadXml(const std::string &filename);
			void LoadXml(const Aris::Core::XmlDocument &xml_doc);
			virtual void LoadXml(const Aris::Core::XmlElement &xml_ele);
			void SaveSnapshotXml(const char *filename) const;
			void SaveAdams(const std::string &filename, bool using_script = false) const;

		private:
			Calculator calculator;
			
			int dyn_cst_dim_;//real dimension of constraint matrix
			int dyn_prt_dim_;//real dimension of inertia matrix

			int clb_dim_m_, clb_dim_n_, clb_dim_gam_, clb_dim_frc_;

			std::function<void(int dim, const double *D, const double *b, double *x)> dyn_solve_method_{ nullptr };
			std::function<void(int n, double *A)> clb_inverse_method_{ nullptr };

		protected:

			Aris::DynKer::Environment environment_;
			Aris::DynKer::Script script_;

			std::vector<std::unique_ptr<Part> > parts_;
			std::vector<std::unique_ptr<JointBase> > joints_;
			std::vector<std::unique_ptr<MotionBase> > motions_;
			std::vector<std::unique_ptr<ForceBase> > forces_;
			std::vector<std::unique_ptr<Marker> > markers_;
			
			


			Part* pGround;
		
			friend class Environment;
			friend class Part;
			friend class MotionBase;
			friend class Marker;
			friend class ForceBase;
		};

		template<typename ...Args>
		Marker* Part::AddMarker(const std::string & name, Args ...args)
		{
			if (FindMarker(name))return nullptr;
			Model().markers_.push_back(std::unique_ptr<Marker>(new Marker(*this, name, Model().markers_.size(), args...)));
			marker_names_.insert(std::make_pair(name, Model().markers_.size() - 1));
			return Model().markers_.back().get();
		}
	}
}

#endif