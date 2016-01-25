#ifndef ARIS_DYNAMIC_MODELBASE_
#define ARIS_DYNAMIC_MODELBASE_

#include <vector>
#include <array>
#include <map>
#include <string>
#include <memory>
#include <functional>
#include <algorithm>

#include <aris_core.h>
#include <aris_dynamic_kernel.h>

namespace Aris
{
	namespace Dynamic
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
			mutable std::int32_t count{ 0 };
		};
		typedef std::function<int(ModelBase &, const PlanParamBase &)> PlanFunc;

		struct SimResult
		{
			std::list<std::array<double, 6> > begin_prt_pe_;
			
			std::list<double> time_;
			std::vector<std::list<double> > Pin_, Fin_, Vin_, Ain_;//vector18维，但list维数为时间的维数
		
			void saveToTxt(const std::string &filename) 
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
			const std::string& name() const { return name_; };
			const ModelBase& model()const { return model_; };
			ModelBase& model() { return model_; };
		
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
			bool isActive() const{ return is_active_; };
			void activate(bool isActive = true) { is_active_ = isActive; };
			int id() const { return id_; };
			virtual std::string groupName() = 0;

		private:
			virtual void toXmlElement(Aris::Core::XmlElement &xml_ele) const = 0;
			virtual void toAdamsCmd(std::ofstream &file) const = 0;

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
			virtual std::string adamsType() const = 0;
			const Marker& makI() const { return *makI_; };
			Marker& makI() { return *makI_; };
			const Marker& makJ() const { return *makJ_; };
			Marker& makJ() { return *makJ_; };

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
			virtual int cstDim() const = 0;
			virtual void update();
			virtual void toXmlElement(Aris::Core::XmlElement &xml_ele) const override;
			virtual void toAdamsCmd(std::ofstream &file) const override;

			const double* cstMtxI() const { return const_cast<Constraint*>(this)->csmI(); };
			const double* cstMtxJ() const { return const_cast<Constraint*>(this)->csmJ(); };
			const double* cstAcc() const { return const_cast<Constraint*>(this)->csa(); };
			const double* cstFce() const { return const_cast<Constraint*>(this)->csf(); };

		protected:
			virtual ~Constraint() = default;
			explicit Constraint(ModelBase &model, const std::string &name, int id, Marker &makI, Marker &makJ)
				: Interaction(model, name, id, makI, makJ) {};
			explicit Constraint(ModelBase &model, const std::string &name, int id, const Aris::Core::XmlElement &xml_ele)
				: Interaction(model, name, id, xml_ele) {};

		private:
			virtual void init() = 0;
			virtual double* csmI() = 0;
			virtual double* csmJ() = 0;
			virtual double* csa() = 0;
			virtual double* csf() = 0;

			int col_id_;

			friend class ModelBase;
			friend class Model;
		};

		class Marker :public Element
		{
		public:
			virtual ~Marker() = default;
			virtual void update();
			virtual std::string groupName()override { return "marker"; };

			const ModelBase& model() { return Object::model(); };

			const double4x4& pm() const { return pm_; };
			const double6& vel() const;
			const double6& acc() const;
			const double4x4& prtPm() const { return prt_pm_; };
			const Part& father() const { return prt_; };

			void getPm(double *pm_in) { std::copy_n(static_cast<const double *>(*pm()), 16, pm_in); };
			void getPe(double *pe, const char *type = "313")const { s_pm2pe(*pm(), pe, type); };
			void getPq(double *pq)const { s_pm2pq(*pm(), pq); };
			void getVel(double *vel_in)const { std::copy_n(vel(), 6, vel_in); };
			void getAcc(double *acc_in)const { std::copy_n(acc(), 6, acc_in); };

			explicit Marker(const Part &prt, const double *prtPe = nullptr, const char* eulType = "313");//for constructing marker outside model
		private:
			explicit Marker(ModelBase &model, Part &prt, const std::string &name, int id);//only for child class Part to construct
			explicit Marker(Part &prt, const std::string &name, int id, const double *pPrtPm = nullptr, Marker *pRelativeTo = nullptr);
			explicit Marker(Part &prt, const std::string &name, int id, const Aris::Core::XmlElement *ele);
			virtual void toXmlElement(Aris::Core::XmlElement &xml_ele) const;
			virtual void toAdamsCmd(std::ofstream &file) const {};

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
			virtual void update();
			virtual std::string groupName()override { return "part"; };
			const ModelBase& model()const { return Object::model(); };
			ModelBase& model() { return Object::model(); };

			const double4x4& pm()const { return pm_; };
			double4x4& pm() { return pm_; };
			const double6& vel()const { return vel_; };
			double6& vel() { return vel_; };
			const double6& acc()const { return acc_; };
			double6& acc() { return acc_; };
			const double4x4& invPm() const { return inv_pm_; };
			const double6x6& prtIm() const { return prt_im_; };
			const double6& prtVel() const { return prt_vel_; };
			const double6& prtAcc() const { return prt_acc_; };
			const double6& prtFg() const { return prt_fg_; };
			const double6& prtFv() const { return prt_fv_; };
			const double6& prtGravity() const { return prt_gravity_; };

			void setPm(const double *pm_in) { std::copy_n(pm_in, 16, static_cast<double*>(*pm_)); };
			void setPe(const double *pe, const char *type = "313") { s_pe2pm(pe, *pm(), type); };
			void setPq(const double *pq) { s_pq2pm(pq, *pm()); };
			void setVel(const double *vel_in) { std::copy_n(vel_in, 6, vel()); };
			void setAcc(const double *acc_in) { std::copy_n(acc_in, 6, acc()); };
			
			Marker* findMarker(const std::string &name);
			const Marker* findMarker(const std::string &name)const;

			template<typename ...Args>
			Marker* addMarker(const std::string & name, Args ...args);

		private:
			explicit Part(ModelBase &model, const std::string &name, int id, const double *prtIm = nullptr
				, const double *pm = nullptr, const double *vel = nullptr, const double *acc = nullptr);
			explicit Part(ModelBase &model, const std::string &name, int id, const Aris::Core::XmlElement *ele);
			virtual void toXmlElement(Aris::Core::XmlElement &xml_ele) const;
			virtual void toAdamsCmd(std::ofstream &file) const;

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
			virtual std::string groupName()override { return "joint"; };

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
			virtual std::string groupName()override { return "motion"; };
			virtual int cstDim() const override { return 1; };

			const double* frcCoe() const { return frc_coe_; };
			void SetFrcCoe(const double *frc_coe) { std::copy_n(frc_coe, 3, frc_coe_); };

			double motPos() const { return mot_pos_; };
			void setMotPos(double mot_pos) { mot_pos_ = mot_pos; };
			double motVel() const { return mot_vel_; };
			void setMotVel(double mot_vel) { mot_vel_ = mot_vel; };
			double motAcc() const { return mot_acc_; };
			void setMotAcc(double mot_acc) { this->mot_acc_ = mot_acc; };
			double motFce() const { return motFceDyn() + motFceFrc();	};
			void setMotFce(double mot_fce) { this->mot_fce_ = mot_fce; };
			double motFceDyn() const { return mot_fce_dyn_; }
			void setMotFceDyn(double mot_dyn_fce) { this->mot_fce_dyn_ = mot_dyn_fce; };
			double motFceFrc() const { return s_sgn(mot_vel_)*frc_coe_[0] + mot_vel_*frc_coe_[1] + mot_acc_*frc_coe_[2]; };

			void setPosAkimaCurve(const int num, const double* time, const double *pos)
			{
				this->posCurve.reset(new Akima(num, time, pos));
			}
			double posAkima(double t, char derivativeOrder = '0') { return posCurve->operator()(t, derivativeOrder); };
			void posAkima(int length, const double *t, double *pos, char order = '0') { posCurve->operator()(length, t, pos, order); };

		protected:
			explicit MotionBase(ModelBase &model, const std::string &name, int id, Marker &makI, Marker &makJ);
			explicit MotionBase(ModelBase &model, const std::string &name, int id, const Aris::Core::XmlElement &xml_ele);
			virtual void toXmlElement(Aris::Core::XmlElement &xml_ele) const override;

		protected:
			double* csmI() override { return csmI_; };
			double* csmJ() override { return csmJ_; };
			double* csa() override { return &csa_; };
			double* csf() override { return &mot_fce_dyn_; };
			

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
			virtual std::string groupName()override { return "force"; };
			const double* fceI() const { return fceI_; };
			const double* fceJ() const { return fceJ_; };
			
			void setFceAkimaCurve(const int num, const double* time, const double *fce)
			{
				this->fce_akima_.reset(new Akima(num, time, fce));
			}
			double fceAkima(double t, char derivativeOrder = '0') { return fce_akima_->operator()(t, derivativeOrder); };
			void fceAkima(int length, const double *t, double *pos, char order = '0') { fce_akima_->operator()(length, t, pos, order); };

			
			virtual void update() = 0;

		protected:
			explicit ForceBase(ModelBase &model, const std::string &name, int id, Marker &makI, Marker &makJ);
			explicit ForceBase(ModelBase &model, const std::string &name, int id, const Aris::Core::XmlElement &xml_ele);
			virtual void toXmlElement(Aris::Core::XmlElement &xml_ele) const {};
			virtual void toAdamsCmd(std::ofstream &file) const {};

			double fceI_[6]{ 0 };
			double fceJ_[6]{ 0 };

			std::unique_ptr<Akima> fce_akima_;

			friend class ModelBase;
		};

		class FloatMarker final:public Marker
		{
		public:
			void setPrtPm(const double *prtPm) { std::copy_n(prtPm, 16, static_cast<double *>(*prt_pm_)); };
			void setPrtPe(const double *prtPe, const char *type = "313") { s_pe2pm(prtPe, *prt_pm_, type); };
			void setPrtPq(const double *prtPq) { s_pq2pm(prtPq, *prt_pm_); };

			explicit FloatMarker(const Part &prt, const double *prtPe = nullptr, const char* eulType = "313")
				:Marker(prt, prtPe, eulType) {};
		};

		template<int DIMENSION>
		class JointBaseDim :public JointBase
		{
		public:
			static constexpr int dim() { return DIMENSION; };
			virtual int cstDim() const { return DIMENSION; };
			
		protected:
			explicit JointBaseDim(ModelBase &model, const std::string &name, int id, Marker &makI, Marker &makJ)
				:JointBase(model, name, id, makI, makJ) {};
			explicit JointBaseDim(ModelBase &model, const std::string &name, int id, const Aris::Core::XmlElement &xml_ele)
				:JointBase(model, name, id, xml_ele) {};

			virtual double* csmI() override { return *csmI_; };
			virtual double* csmJ() override { return *csmJ_; };
			virtual double* csf() override { return csf_; };
			virtual double* csa() override { return csa_; };

			double csmI_[6][DIMENSION]{ { 0 } };
			double csmJ_[6][DIMENSION]{ { 0 } };
			double csf_[DIMENSION]{ 0 };
			double csa_[DIMENSION]{ 0 };

		private:
			friend class ModelBase;
		};

		class Environment final:public Object
		{
		private:
			explicit Environment(ModelBase &model) :Object(model, "Environment") {};
			virtual ~Environment() = default;

			void toXmlElement(Aris::Core::XmlElement &xml_ele) const;
			void fromXmlElement(const Aris::Core::XmlElement &xml_ele);
			void toAdamsCmd(std::ofstream &file) const;
		private:
			double gravity_[6]{ 0, -9.8, 0, 0, 0, 0 };

			friend class Part;
			friend class ModelBase;
		};
		class Script
		{
		public:
			void activate(Element &ele, bool isActive);
			void alignMarker(Marker &mak_move, const Marker& mak_target);
			void simulate(std::uint32_t ms_dur, std::uint32_t ms_dt);
			void toAdamsCmd(std::ofstream &file) const;

			bool empty() const;
			std::uint32_t endTime()const;
			void setTopologyAt(std::uint32_t ms_time);
			void updateAt(std::uint32_t ms_time);

			void clear();
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

			Aris::Dynamic::Environment& environment() { return environment_; };
			const Aris::Dynamic::Environment& environment()const { return environment_; };
			Aris::Dynamic::Script& script() { return script_; };
			const Aris::Dynamic::Script& script()const { return script_; };

			template<typename ...Args>
			Part* addPart(const std::string & name, Args ...args)
			{
				if (findPart(name))	return nullptr;
				parts_.push_back(std::unique_ptr<Part>(new Part(*this, name, parts_.size(), args...)));
				return parts_.back().get();
			}
			template<typename Joint, typename ...Args>
			Joint* addJoint(const std::string & name, Args ...args)
			{
				if (findJoint(name))return nullptr;
				auto ret = new Joint(*this, name, joints_.size(), args...);
				joints_.push_back(std::unique_ptr<JointBase>(ret));
				return ret;
			}
			template<typename Motion, typename ...Args>
			Motion* addMotion(const std::string & name, Args ...args)
			{
				if (findMotion(name))return nullptr;
				auto ret = new Motion(*this, name, motions_.size(), args...);
				motions_.push_back(std::unique_ptr<MotionBase>(ret));
				return ret;
			};
			template<typename Force, typename ...Args>
			Force* addForce(const std::string & name, Args ...args)
			{
				if (findForce(name))return nullptr;
				auto ret = new Force(*this, name, forces_.size(), args...);
				forces_.push_back(std::unique_ptr<ForceBase>(ret));
				return ret;
			}
			std::size_t partNum() const { return parts_.size(); };
			std::size_t motionNum() const { return motions_.size(); };
			std::size_t jointNum() const { return joints_.size(); };
			std::size_t forceNum() const { return forces_.size(); };
			std::size_t markerNum() const { return markers_.size(); };
			Part &partAt(int id) { return *parts_.at(id).get(); };
			const Part &partAt(int id) const { return *parts_.at(id).get(); };
			JointBase &jointAt(int id) { return *joints_.at(id).get(); };
			const JointBase &jointAt(int id) const { return *joints_.at(id).get(); };
			MotionBase &motionAt(int id) { return *motions_.at(id).get(); };
			const MotionBase &motionAt(int id) const { return *motions_.at(id).get(); };
			ForceBase &forceAt(int id) { return *forces_.at(id).get(); };
			const ForceBase &forceAt(int id) const { return *forces_.at(id).get(); };
			Marker &markerAt(int id) { return *markers_.at(id).get(); };
			const Marker &markerAt(int id) const { return *markers_.at(id).get(); };
			const Part *findPart(const std::string &name)const { return const_cast<ModelBase*>(this)->findPart(name); };
			const JointBase *findJoint(const std::string &name)const { return const_cast<ModelBase*>(this)->findJoint(name); };
			const MotionBase *findMotion(const std::string &name)const { return const_cast<ModelBase*>(this)->findMotion(name); };
			const ForceBase *findForce(const std::string &name)const { return const_cast<ModelBase*>(this)->findForce(name); };
			Part *findPart(const std::string &name);
			JointBase *findJoint(const std::string &name);
			MotionBase *findMotion(const std::string &name);
			ForceBase *findForce(const std::string &name);

			void forEachPart(std::function<void(Part*)> fun)
			{
				for (auto&i : parts_)
				{
					fun(i.get());
				}
			}
			void forEachMarker(std::function<void(Marker*)> fun)
			{
				for (auto&i : markers_)
				{
					fun(i.get());
				}
			}
			void forEachMotion(std::function<void(MotionBase*)> fun)
			{
				for (auto&i : motions_)
				{
					fun(i.get());
				}
			}
			void forEachForce(std::function<void(ForceBase*)> fun)
			{
				for (auto&i : forces_)
				{
					fun(i.get());
				}
			}
			void forEachJoint(std::function<void(JointBase*)> fun)
			{
				for (auto&i : joints_)
				{
					fun(i.get());
				}
			}
			void forEachElement(std::function<void(Element*)> fun)
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
			int dynDimM()const { return dyn_prt_dim_; };
			int dynDimN()const { return dyn_cst_dim_; };
			int dynDim()const { return dynDimN() + dynDimM(); };
			void dynSetSolveMethod(std::function<void(int dim, const double *D, const double *b, double *x)> solve_method);
			void dynCstMtx(double *cst_mtx) const;
			void dynIneMtx(double *ine_mtx) const;
			void dynCstAcc(double *cst_acc) const;
			void dynPrtFce(double *prt_fce) const;
			void dynCstFce(double *cst_fce) const;
			void dynPrtAcc(double *prt_acc) const;
			void dynPre();
			void dynUpd();
			void dynMtx(double *D, double *b) const;
			void dynSov(const double *D, const double *b, double *x) const;
			void dynUkn(double *x) const;
			void dynEnd(const double *x);
			virtual void dyn();

			/// 标定矩阵为m x n维的矩阵，其中m为驱动的数目，n为部件个数*10+驱动数*3
			int clbDimM()const { return clb_dim_m_; };
			int clbDimN()const { return clb_dim_n_; };
			int clbDimGam()const { return clb_dim_gam_; };
			int clbDimFrc()const { return clb_dim_frc_; };
			void clbSetInverseMethod(std::function<void(int n, double *A)> inverse_method);
			void clbPre();
			void clbUpd();
			void clbMtx(double *clb_D, double *clb_b) const;
			void clbUkn(double *clb_x) const;

			/// 仿真函数
			virtual void kinFromPin() {};
			virtual void kinFromVin() {};
			/// 静态仿真，结果仅仅返回驱动的位置
			void simKin(const PlanFunc &func, const PlanParamBase &param, SimResult &result, bool using_script = false);
			/// 动态仿真，待完善
			void simDyn(const PlanFunc &func, const PlanParamBase &param, SimResult &result, bool using_script = false);
			/// 静态仿真，并将结果设置到驱动的Akima函数中
			void simKinAkima(const PlanFunc &func, const PlanParamBase &param, SimResult &result, int akima_interval = 1, bool using_script = false);
			/// 动态仿真，根据静态得到的Akima插值，计算驱动的速度和加速度，并且计算动力学。依赖KinFrom系列函数
			void simDynAkima(const PlanFunc &func, const PlanParamBase &param, SimResult &result, int akima_interval = 1, bool using_script = false);
			/// 直接生成Adams模型，依赖SimDynAkima
			void simToAdams(const std::string &adams_file, const PlanFunc &fun, const PlanParamBase &param, int ms_dt = 10, bool using_script = false);

			void loadXml(const char* filename) { loadXml(std::string(filename)); };
			void loadXml(const std::string &filename);
			void loadXml(const Aris::Core::XmlDocument &xml_doc);
			virtual void loadXml(const Aris::Core::XmlElement &xml_ele);
			void saveSnapshotXml(const char *filename) const;
			void saveAdams(const std::string &filename, bool using_script = false) const;

		private:
			Core::Calculator calculator;
			
			int dyn_cst_dim_;//real dimension of constraint matrix
			int dyn_prt_dim_;//real dimension of inertia matrix

			int clb_dim_m_, clb_dim_n_, clb_dim_gam_, clb_dim_frc_;

			std::function<void(int dim, const double *D, const double *b, double *x)> dyn_solve_method_{ nullptr };
			std::function<void(int n, double *A)> clb_inverse_method_{ nullptr };

		protected:

			Aris::Dynamic::Environment environment_;
			Aris::Dynamic::Script script_;

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
		Marker* Part::addMarker(const std::string & name, Args ...args)
		{
			if (findMarker(name))return nullptr;
			model().markers_.push_back(std::unique_ptr<Marker>(new Marker(*this, name, model().markers_.size(), args...)));
			marker_names_.insert(std::make_pair(name, model().markers_.size() - 1));
			return model().markers_.back().get();
		}
	}
}

#endif