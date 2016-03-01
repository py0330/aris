#ifndef ARIS_DYNAMIC_MODEL_
#define ARIS_DYNAMIC_MODEL_

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

		class Element;
		class Part;
		class Marker;
		class Joint;
		class Motion;
		class Force;

		class Environment;
		class Model;

		struct PlanParamBase
		{
			std::int32_t cmd_type{ 0 };
			mutable std::int32_t count{ 0 };
		};
		typedef std::function<int(Model &, const PlanParamBase &)> PlanFunc;
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
			virtual auto model()->Model& { return *model_; };
			virtual auto model()const->const Model&{ return *model_; };
			virtual auto saveXml(Aris::Core::XmlElement &xml_ele) const->void;
			virtual auto saveAdams(std::ofstream &file) const->void {};
			auto name() const->const std::string&{ return name_; };
			auto father()->Object& { return *father_; };
			auto father()const->const Object&{ return *father_; };

		protected:
			virtual ~Object() = default;
			Object(const Object &) = default;
			Object(Object &&) = default;
			Object &operator=(const Object &) = default;
			Object &operator=(Object &&) = default;
			explicit Object(Object &father, const std::string &name) :model_(&father.model()), father_(&father), name_(name) {};
			explicit Object(Object &father, const Aris::Core::XmlElement &xml_ele) :model_(&father.model()), father_(&father), name_(xml_ele.name()) {};

		private:
			Model *model_;
			Object *father_;
			std::string name_;

			friend class Model;
		};
		class Environment final :public Object
		{
		public:
			virtual ~Environment() = default;
			Environment(const Environment &) = default;
			Environment(Environment &&) = default;
			Environment &operator=(const Environment &) = default;
			Environment &operator=(Environment &&) = default;
			explicit Environment(Object &father, const std::string &name) :Object(father, name) {};
			explicit Environment(Object &father, const Aris::Core::XmlElement &xml_ele);
			virtual auto saveXml(Aris::Core::XmlElement &xml_ele) const->void override;
			virtual auto saveAdams(std::ofstream &file) const->void override;

		private:
			double gravity_[6]{ 0, -9.8, 0, 0, 0, 0 };

			friend class Part;
			friend class Model;
		};
		class Script
		{
		public:
			void activate(Element &ele, bool isActive);
			void alignMarker(Marker &mak_move, const Marker& mak_target);
			void simulate(std::uint32_t ms_dur, std::uint32_t ms_dt);
			void saveAdams(std::ofstream &file) const;

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
			friend class Model;
		};

		class Element :public Object
		{
		public:
			virtual auto saveXml(Aris::Core::XmlElement &xml_ele) const->void override;
			virtual auto typeName() const->const std::string& = 0;
			virtual auto groupName()const->const std::string& = 0;
			virtual auto adamsID()const->std::size_t { return id() + 1; };
			virtual auto adamsTypeName() const->const std::string&{ return typeName(); };
			virtual auto adamsGroupName()const->const std::string&{ return groupName(); };
			auto isActive() const->bool { return is_active_; };
			auto activate(bool isActive = true)->void { is_active_ = isActive; };
			auto id() const->std::size_t { return id_; };

		protected:
			virtual ~Element() = default;
			Element(const Element &) = delete;
			Element(Element &&) = delete;
			Element &operator=(const Element &) = delete;
			Element &operator=(Element &&) = delete;
			explicit Element(Object &father, const std::string &name, std::size_t id, bool active = true) : Object(father, name), id_(id), is_active_(active) {};
			explicit Element(Object &father, const Aris::Core::XmlElement &xml_ele, std::size_t id);
			virtual auto init()->void {};


		private:
			bool is_active_{ true };
			const std::size_t id_;

			friend class Model;
			template<typename Type>friend class ElementPool;
		};
		template <typename ElementType>	class ElementPool :Object
		{
		public:
			template<typename ChildType>
			static auto registerType()->void
			{
				if (typeInfoMap().find(ChildType::TypeName()) != typeInfoMap().end())
					throw std::runtime_error("can not register joint of " + ChildType::TypeName() + " cause it already registered");

				typeInfoMap().insert(std::make_pair(ChildType::TypeName(), TypeInfo::template create<ChildType>()));
			};
			virtual auto saveXml(Aris::Core::XmlElement &xml_ele) const->void override
			{
				Object::saveXml(xml_ele);
				for (auto &ele : element_vec_)
				{
					auto xml_iter = xml_ele.GetDocument()->NewElement("");
					ele->saveXml(*xml_iter);
					xml_ele.InsertEndChild(xml_iter);
				}
			}
			virtual auto saveAdams(std::ofstream &file) const->void override
			{
				for (auto &ele : element_vec_)ele->saveAdams(file);
			}
			template<typename ChildType, typename ...Args>
			auto add(const std::string & name, Args ...args)->ElementType&
			{
				if (find(name))throw std::runtime_error("joint \"" + name + "\" already exists, can't addJoint()");
				auto ret = new ChildType(this->father(), name, element_vec_.size(), args...);
				element_vec_.push_back(std::unique_ptr<ElementType>(ret));
				return std::ref(*ret);
			}
			auto add(const Aris::Core::XmlElement &xml_ele)->ElementType&
			{
				if (find(xml_ele.name()))throw std::runtime_error(ElementType::TypeName() + " \"" + xml_ele.name() + "\" already exists, can't add element");
				std::string type = xml_ele.Attribute("type") ? xml_ele.Attribute("type") : ElementType::TypeName();
				if (typeInfoMap().find(type) == typeInfoMap().end())throw std::runtime_error(std::string("can't find type ") + type);

				auto ret = typeInfoMap().at(type).newFromXml(this->father(), xml_ele, element_vec_.size());
				element_vec_.push_back(std::unique_ptr<ElementType>(ret));
				return std::ref(*element_vec_.back().get());
			};
			auto at(std::size_t id) ->ElementType& { return std::ref(*element_vec_.at(id).get()); };
			auto at(std::size_t id) const->const ElementType&{ return std::ref(*element_vec_.at(id).get()); };
			auto find(const std::string &name)->ElementType *
			{
				auto p = std::find_if(element_vec_.begin(), element_vec_.end(), [&name](typename decltype(element_vec_)::const_reference p)
				{
					return (p->name() == name);
				});

				if (p == element_vec_.end())
				{
					return nullptr;
				}
				else
				{
					return p->get();
				}
			}
			auto find(const std::string &name) const->const ElementType *{ return const_cast<ElementPool *>(this)->find(name); }
			auto findID(const std::string &name) const->std::size_t
			{
				auto ele = std::find_if(element_vec_.begin(), element_vec_.end(), [name](typename decltype(element_vec_)::const_reference p)
				{
					return (p->name() == name);
				});

				if (ele == element_vec_.end())	throw std::runtime_error("can't find part " + name);
				return ele - element_vec_.begin();
			}
			auto size() const ->std::size_t { return element_vec_.size(); };
			auto begin()->typename std::vector<std::unique_ptr<ElementType>>::iterator { return element_vec_.begin(); };
			auto begin() const ->typename std::vector<std::unique_ptr<ElementType>>::const_iterator { return element_vec_.begin(); };
			auto end()->typename std::vector<std::unique_ptr<ElementType>>::iterator { return element_vec_.end(); };
			auto end() const ->typename std::vector<std::unique_ptr<ElementType>>::const_iterator { return element_vec_.end(); };
			auto clear() -> void { element_vec_.clear(); };

		private:
			~ElementPool() = default;
			ElementPool(const ElementPool &other) = delete;
			ElementPool(ElementPool && other) :element_vec_(std::move(other.element_vec_)) {};
			ElementPool& operator=(const ElementPool &) = delete;
			ElementPool& operator=(ElementPool &&other)
			{
				element_vec_ = std::move(other.element_vec_);
				return *this;
			};
			explicit ElementPool(Object &father, const std::string &name) :Object(father, name) {};
			explicit ElementPool(Object &father, const Aris::Core::XmlElement &xml_ele) :Object(father, xml_ele)
			{
				for (auto ele = xml_ele.FirstChildElement(); ele != nullptr; ele = ele->NextSiblingElement())
				{
					add(*ele).init();
				}
			};

		private:
			std::vector<std::unique_ptr<ElementType> > element_vec_;

			struct TypeInfo
			{
				std::function<ElementType*(Object &, const Aris::Core::XmlElement &xml_ele, std::size_t id)> newFromXml;

				template<typename ChildType> static auto create()->TypeInfo
				{
					TypeInfo info;

					info.newFromXml = [](Object &father, const Aris::Core::XmlElement &xml_ele, std::size_t id)->ElementType*
					{
						return new ChildType(father, xml_ele, id);
					};

					return info;
				}
			};
			static auto typeInfoMap()->std::map<std::string, TypeInfo>&;

			friend class Part;
			friend class Model;
		};
		class Interaction :public Element
		{
		public:
			virtual auto saveXml(Aris::Core::XmlElement &xml_ele) const->void override;
			auto makI()->Marker& { return *makI_; };
			auto makI() const->const Marker&{ return *makI_; };
			auto makJ()->Marker& { return *makJ_; };
			auto makJ() const->const Marker&{ return *makJ_; };

		protected:
			virtual ~Interaction() = default;
			explicit Interaction(Object &father, const std::string &name, std::size_t id, Marker &makI, Marker &makJ, bool is_active = true)
				: Element(father, name, id, is_active), makI_(&makI), makJ_(&makJ) {};
			explicit Interaction(Object &father, const Aris::Core::XmlElement &xml_ele, std::size_t id);

		private:
			Marker *makI_;
			Marker *makJ_;
		};
		class Constraint :public Interaction
		{
		public:
			virtual auto saveAdams(std::ofstream &file) const->void override;
			virtual auto update()->void;
			virtual auto dim() const->std::size_t = 0;
			auto cstMtxI() const->const double* { return const_cast<Constraint*>(this)->csmI(); };
			auto cstMtxJ() const->const double* { return const_cast<Constraint*>(this)->csmJ(); };
			auto cstAcc() const->const double* { return const_cast<Constraint*>(this)->csa(); };
			auto cstFce() const->const double* { return const_cast<Constraint*>(this)->csf(); };

		protected:
			virtual ~Constraint() = default;
			explicit Constraint(Object &father, const std::string &name, std::size_t id, Marker &makI, Marker &makJ, bool is_active = true)
				: Interaction(father, name, id, makI, makJ, is_active) {};
			explicit Constraint(Object &father, const Aris::Core::XmlElement &xml_ele, std::size_t id)
				: Interaction(father, xml_ele, id) {};

		private:
			virtual auto csmI()->double* = 0;
			virtual auto csmJ()->double* = 0;
			virtual auto csa()->double* = 0;
			virtual auto csf()->double* = 0;

			std::size_t col_id_;

			friend class Model;
		};

		class Variable:public Element
		{
		public:
			static auto TypeName()->const std::string &{ static const std::string type{ "variable" }; return type; };
			virtual ~Variable() = default;
			virtual auto saveXml(Aris::Core::XmlElement &xml_ele) const->void override
			{
				Element::saveXml(xml_ele);
				xml_ele.SetText(this->toString().c_str());
			}
			virtual auto typeName() const->const std::string& override{ return TypeName(); };
			virtual auto groupName()const->const std::string& override{ return TypeName(); };
			virtual auto toString() const->std::string { return ""; };
			
		protected:
			explicit Variable(Object &father, const std::string &name, std::size_t id, bool active = true): Element(father, name, id, active) {};
			explicit Variable(Object &father, const Aris::Core::XmlElement &xml_ele, std::size_t id): Element(father, xml_ele, id) {};
		
		private:
			friend class ElementPool<Variable>;
		};
		class Marker :public Element
		{
		public:
			static auto TypeName()->const std::string &{ static const std::string type{ "marker" }; return type; };
			virtual ~Marker() = default;
			virtual auto saveXml(Aris::Core::XmlElement &xml_ele) const->void override;
			virtual auto saveAdams(std::ofstream &file) const->void override;
			virtual auto typeName() const->const std::string& override{ return TypeName(); };
			virtual auto groupName()const->const std::string& override{ return TypeName(); };
			virtual auto adamsID()const->std::size_t override;
			virtual auto update()->void;
			virtual auto fatherPart() const->const Part&;
			virtual auto fatherPart()->Part&;
			auto pm() const->const double4x4&{ return pm_; };
			auto vel() const->const double6&;
			auto acc() const->const double6&;
			auto prtPm() const->const double4x4&{ return prt_pm_; };
			auto getPm(double *pm_in)->void { std::copy_n(static_cast<const double *>(*pm()), 16, pm_in); };
			auto getPe(double *pe, const char *type = "313")const->void { s_pm2pe(*pm(), pe, type); };
			auto getPq(double *pq)const->void { s_pm2pq(*pm(), pq); };
			auto getVel(double *vel_in)const->void { std::copy_n(vel(), 6, vel_in); };
			auto getAcc(double *acc_in)const->void { std::copy_n(acc(), 6, acc_in); };

		protected:
			explicit Marker(Object &father, const std::string &name, std::size_t id, const double *prt_pm = nullptr, Marker *relative_mak = nullptr, bool active = true);//only for child class Part to construct
			explicit Marker(Object &father, const Aris::Core::XmlElement &ele, std::size_t id);
			explicit Marker(Part &prt, const double *prt_pe = nullptr, const char* eul_type = "313");

		private:
			double pm_[4][4];
			double prt_pm_[4][4];

			friend class ElementPool<Marker>;
			friend class Part;
			friend class FloatMarker;
			friend class Script;
			friend class Model;
		};
		class Part :public Marker
		{
		public:
			static auto TypeName()->const std::string &{ static const std::string type{ "part" }; return type; };
			virtual ~Part() = default;
			virtual auto saveXml(Aris::Core::XmlElement &xml_ele) const->void override;
			virtual auto saveAdams(std::ofstream &file) const->void override;
			virtual auto typeName() const->const std::string& override{ return TypeName(); };
			virtual auto groupName()const->const std::string& override{ return TypeName(); };
			virtual auto adamsID()const->std::size_t override { return id() + 1; };
			virtual auto update()->void;
			virtual auto fatherPart() const->const Part&{ return *this; };
			virtual auto fatherPart() ->Part& { return *this; };
			auto pm()const->const double4x4&{ return pm_; };
			auto pm()->double4x4& { return pm_; };
			auto vel()const->const double6&{ return vel_; };
			auto vel()->double6& { return vel_; };
			auto acc()const->const double6&{ return acc_; };
			auto acc()->double6& { return acc_; };
			auto invPm() const->const double4x4&{ return inv_pm_; };
			auto prtIm() const->const double6x6&{ return prt_im_; };
			auto prtVel() const->const double6&{ return prt_vel_; };
			auto prtAcc() const->const double6&{ return prt_acc_; };
			auto prtFg() const->const double6&{ return prt_fg_; };
			auto prtFv() const->const double6&{ return prt_fv_; };
			auto prtGravity() const->const double6&{ return prt_gravity_; };
			auto setPm(const double *pm_in)->void { std::copy_n(pm_in, 16, static_cast<double*>(*pm_)); };
			auto setPe(const double *pe, const char *type = "313")->void { s_pe2pm(pe, *pm(), type); };
			auto setPq(const double *pq)->void { s_pq2pm(pq, *pm()); };
			auto setVel(const double *vel_in)->void { std::copy_n(vel_in, 6, vel()); };
			auto setAcc(const double *acc_in)->void { std::copy_n(acc_in, 6, acc()); };

			auto markerPool()->ElementPool<Marker>& { return std::ref(marker_pool_); };
			auto markerPool()const->const ElementPool<Marker>& { return std::ref(marker_pool_); };

		private:
			explicit Part(Object &father, const std::string &name, std::size_t id, const double *prt_im = nullptr
				, const double *pm = nullptr, const double *vel = nullptr, const double *acc = nullptr, bool active = true);
			explicit Part(Object &father, const Aris::Core::XmlElement &xml_ele, std::size_t id);


		private:
			ElementPool<Marker> marker_pool_;

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
			friend class Model;
			friend class ElementPool<Part>;
		};
		class Joint :public Constraint
		{
		public:
			static auto TypeName()->const std::string &{ static const std::string type{ "joint" }; return type; };
			virtual ~Joint() = default;
			virtual auto typeName() const->const std::string& override{ return TypeName(); };
			virtual auto groupName()const->const std::string& override{ return TypeName(); };

		protected:
			explicit Joint(Object &father, const std::string &name, std::size_t id, Marker &makI, Marker &makJ, bool active = true)
				: Constraint(father, name, id, makI, makJ, active) {};
			explicit Joint(Object &father, const Aris::Core::XmlElement &xml_ele, std::size_t id)
				: Constraint(father, xml_ele, id) {};

			friend class ElementPool<Motion>;
			friend class Model;
		};
		class Motion :public Constraint
		{
		public:
			static const std::string &TypeName() { static const std::string type{ "motion" }; return type; };
			virtual ~Motion() = default;
			virtual auto saveXml(Aris::Core::XmlElement &xml_ele) const->void override;
			virtual auto typeName() const->const std::string& override{ return TypeName(); };
			virtual auto groupName()const->const std::string& override{ return TypeName(); };
			virtual auto dim() const ->std::size_t override { return 1; };
			auto frcCoe() const ->const double* { return frc_coe_; };
			auto SetFrcCoe(const double *frc_coe)->void { std::copy_n(frc_coe, 3, frc_coe_); };
			auto motPos() const->double { return mot_pos_; };
			auto setMotPos(double mot_pos)->void { mot_pos_ = mot_pos; };
			auto motVel() const->double { return mot_vel_; };
			auto setMotVel(double mot_vel)->void { mot_vel_ = mot_vel; };
			auto motAcc() const->double { return mot_acc_; };
			auto setMotAcc(double mot_acc)->void { this->mot_acc_ = mot_acc; };
			auto motFce() const->double { return motFceDyn() + motFceFrc(); };
			auto setMotFce(double mot_fce)->void { this->mot_fce_ = mot_fce; };
			auto motFceDyn() const->double { return mot_fce_dyn_; }
			auto setMotFceDyn(double mot_dyn_fce)->void { this->mot_fce_dyn_ = mot_dyn_fce; };
			auto motFceFrc() const->double { return s_sgn(mot_vel_)*frc_coe_[0] + mot_vel_*frc_coe_[1] + mot_acc_*frc_coe_[2]; };

			auto setPosAkimaCurve(const int num, const double* time, const double *pos)->void
			{
				this->posCurve.reset(new Akima(num, time, pos));
			}
			auto posAkima(double t, char derivativeOrder = '0')->double { return posCurve->operator()(t, derivativeOrder); };
			auto posAkima(int length, const double *t, double *pos, char order = '0')->void { posCurve->operator()(length, t, pos, order); };

		protected:
			explicit Motion(Object &father, const std::string &name, std::size_t id, Marker &makI, Marker &makJ, const double *frc_coe = nullptr, bool active = true);
			explicit Motion(Object &father, const Aris::Core::XmlElement &xml_ele, std::size_t id);
			virtual auto csmI()->double* override { return csmI_; };
			virtual auto csmJ()->double* override { return csmJ_; };
			virtual auto csa()->double* override { return &csa_; };
			virtual auto csf()->double* override { return &mot_fce_dyn_; };

			/*pos\vel\acc\fce of motion*/
			double mot_pos_{ 0 }, mot_vel_{ 0 }, mot_acc_{ 0 }, mot_fce_dyn_{ 0 };
			double mot_fce_{ 0 };//仅仅用于标定
			double frc_coe_[3]{ 0 };

			double csmI_[6]{ 0 };
			double csmJ_[6]{ 0 };
			double csa_{ 0 };

			/*for adams*/
			std::unique_ptr<Akima> posCurve;

			friend class ElementPool<Motion>;
			friend class Model;
		};
		class Force :public Interaction
		{
		public:
			static auto TypeName()->const std::string &{ static const std::string type{ "force" }; return type; };
			virtual ~Force() = default;
			virtual auto typeName() const->const std::string& override{ return TypeName(); };
			virtual auto groupName()const->const std::string& override{ return TypeName(); };
			virtual auto update()->void = 0;
			auto fceI() const->const double* { return fceI_; };
			auto fceJ() const->const double* { return fceJ_; };
			auto setFceAkimaCurve(const int num, const double* time, const double *fce)->void
			{
				this->fce_akima_.reset(new Akima(num, time, fce));
			}
			auto fceAkima(double t, char derivativeOrder = '0')->double { return fce_akima_->operator()(t, derivativeOrder); };
			auto fceAkima(int length, const double *t, double *pos, char order = '0')->void { fce_akima_->operator()(length, t, pos, order); };

		protected:
			explicit Force(Object &father, const std::string &name, std::size_t id, Marker &makI, Marker &makJ, bool active = true)
				:Interaction(father, name, id, makI, makJ, active) {};
			explicit Force(Object &father, const Aris::Core::XmlElement &xml_ele, std::size_t id)
				:Interaction(father, xml_ele, id) {};

			double fceI_[6]{ 0 };
			double fceJ_[6]{ 0 };

			std::unique_ptr<Akima> fce_akima_;

			friend class ElementPool<Force>;
			friend class Model;
		};

		class Model :public Object
		{
		public:
			virtual ~Model();
			Model(const Model & other) = default;
			Model(Model &&) = default;
			Model &operator=(const Model &) = default;
			Model &operator=(Model &&) = default;
			explicit Model(const std::string & name = "Model");

			virtual auto loadXml(const char* filename)->void { loadXml(std::string(filename)); };
			virtual auto loadXml(const std::string &filename)->void;
			virtual auto loadXml(const Aris::Core::XmlDocument &xml_doc)->void;
			virtual auto loadXml(const Aris::Core::XmlElement &xml_ele)->void;
			virtual auto saveXml(const char *filename) const->void { saveXml(std::string(filename)); };
			virtual auto saveXml(const std::string &filename) const->void;
			virtual auto saveXml(Aris::Core::XmlDocument &xml_doc)const->void;
			virtual auto saveXml(Aris::Core::XmlElement &xml_ele)const->void override;
			virtual auto saveAdams(const std::string &filename, bool using_script = false) const->void;
			virtual auto saveAdams(std::ofstream &file, bool using_script = false) const->void;
			virtual auto model()->Model& override { return *this; };
			virtual auto model()const->const Model& override{ return *this; };
			auto environment()->Aris::Dynamic::Environment& { return environment_; };
			auto environment()const ->const Aris::Dynamic::Environment&{ return environment_; };
			auto script()->Aris::Dynamic::Script& { return script_; };
			auto script()const->const Aris::Dynamic::Script&{ return script_; };
			auto partPool()->ElementPool<Part>& { return std::ref(part_pool_); };
			auto partPool()const->const ElementPool<Part>& { return std::ref(part_pool_); };
			auto jointPool()->ElementPool<Joint>& { return std::ref(joint_pool_); };
			auto jointPool()const->const ElementPool<Joint>& { return std::ref(joint_pool_); };
			auto motionPool()->ElementPool<Motion>& { return std::ref(motion_pool_); };
			auto motionPool()const->const ElementPool<Motion>& { return std::ref(motion_pool_); };
			auto forcePool()->ElementPool<Force>& { return std::ref(force_pool_); };
			auto forcePool()const->const ElementPool<Force>& { return std::ref(force_pool_); };
			auto ground()->Part& { return *ground_; };
			auto ground()const->const Part&{ return *ground_; };

			template<typename JointType>
			static auto registerJointType()->void { decltype(joint_pool_)::registerType<JointType>(); };
			template<typename MotionType>
			static auto registerMotionType()->void { decltype(motion_pool_)::registerType<MotionType>(); };
			template<typename ForceType>
			static auto registerForceType()->void { decltype(force_pool_)::registerType<ForceType>(); };

			/// 约束矩阵C为m x n维的矩阵，惯量矩阵为m x m维的矩阵
			/// 约束力为n维的向量，约束加速度为n维向量
			/// 部件力为m维的向量，部件加速度为m维向量
			/// 动力学为所求的未知量为部件加速度和约束力，其他均为已知
			virtual auto dyn()->void;
			auto dynDimM()const->std::size_t { return dyn_prt_dim_; };
			auto dynDimN()const->std::size_t { return dyn_cst_dim_; };
			auto dynDim()const->std::size_t { return dynDimN() + dynDimM(); };
			auto dynSetSolveMethod(std::function<void(int dim, const double *D, const double *b, double *x)> solve_method)->void;
			auto dynCstMtx(double *cst_mtx) const->void;
			auto dynIneMtx(double *ine_mtx) const->void;
			auto dynCstAcc(double *cst_acc) const->void;
			auto dynPrtFce(double *prt_fce) const->void;
			auto dynCstFce(double *cst_fce) const->void;
			auto dynPrtAcc(double *prt_acc) const->void;
			auto dynPre()->void;
			auto dynUpd()->void;
			auto dynMtx(double *D, double *b) const->void;
			auto dynSov(const double *D, const double *b, double *x) const->void;
			auto dynUkn(double *x) const->void;
			auto dynEnd(const double *x)->void;
			

			/// 标定矩阵为m x n维的矩阵，其中m为驱动的数目，n为部件个数*10+驱动数*3
			auto clbDimM()const->std::size_t { return clb_dim_m_; };
			auto clbDimN()const->std::size_t { return clb_dim_n_; };
			auto clbDimGam()const->std::size_t { return clb_dim_gam_; };
			auto clbDimFrc()const->std::size_t { return clb_dim_frc_; };
			auto clbSetInverseMethod(std::function<void(int n, double *A)> inverse_method)->void;
			auto clbPre()->void;
			auto clbUpd()->void;
			auto clbMtx(double *clb_D, double *clb_b) const->void;
			auto clbUkn(double *clb_x) const->void;

			/// 仿真函数
			virtual auto kinFromPin()->void {};
			virtual auto kinFromVin()->void {};
			/// 静态仿真，结果仅仅返回驱动的位置
			auto simKin(const PlanFunc &func, const PlanParamBase &param, SimResult &result, bool using_script = false)->void;
			/// 动态仿真，待完善
			auto simDyn(const PlanFunc &func, const PlanParamBase &param, SimResult &result, bool using_script = false)->void;
			/// 静态仿真，并将结果设置到驱动的Akima函数中
			auto simKinAkima(const PlanFunc &func, const PlanParamBase &param, SimResult &result, int akima_interval = 1, bool using_script = false)->void;
			/// 动态仿真，根据静态得到的Akima插值，计算驱动的速度和加速度，并且计算动力学。依赖KinFrom系列函数
			auto simDynAkima(const PlanFunc &func, const PlanParamBase &param, SimResult &result, int akima_interval = 1, bool using_script = false)->void;
			/// 直接生成Adams模型，依赖SimDynAkima
			auto simToAdams(const std::string &adams_file, const PlanFunc &fun, const PlanParamBase &param, int ms_dt = 10, bool using_script = false)->void;

			

		private:
			Aris::Core::Calculator calculator;
			
			Environment environment_{ *this,"Environment" };
			ElementPool<Variable> variable_pool_{ *this,"Variable" };
			ElementPool<Part> part_pool_{ *this,"Part" };
			ElementPool<Joint> joint_pool_{ *this,"Joint" };
			ElementPool<Motion> motion_pool_{ *this,"Motion" };
			ElementPool<Force> force_pool_{ *this,"Force" };


			Aris::Dynamic::Script script_;
			Part* ground_;

			std::size_t dyn_cst_dim_, dyn_prt_dim_;
			std::size_t clb_dim_m_, clb_dim_n_, clb_dim_gam_, clb_dim_frc_;

			std::function<void(int dim, const double *D, const double *b, double *x)> dyn_solve_method_{ nullptr };
			std::function<void(int n, double *A)> clb_inverse_method_{ nullptr };

		protected:
			friend class Environment;
			friend class Part;
			friend class Motion;
			friend class Marker;
			friend class Force;
			friend class MatrixVariable;
		};

		template <typename Type>
		class VariableTemplate: public Variable
		{
		public:
			virtual ~VariableTemplate() = default;
			auto data()->Type& { return data_; };
			auto data()const->const Type&{ return data_; };

		protected:
			explicit VariableTemplate(Aris::Dynamic::Object &father, const std::string &name, std::size_t id, const Type &data, bool active = true)
				: Variable(father, name, id, active), data_(data) {};
			explicit VariableTemplate(Aris::Dynamic::Object &father, const Aris::Core::XmlElement &xml_ele, std::size_t id)
				: Variable(father, xml_ele, id) {};

			Type data_;
		};

		class MatrixVariable : VariableTemplate<Aris::Core::Matrix>
		{
		public:
			static auto TypeName()->const std::string &{ static const std::string type{ "matrix" }; return type; };
			virtual ~MatrixVariable() = default;
			virtual auto typeName() const->const std::string& override{ return TypeName(); };
			virtual auto toString() const->std::string override{ return data_.toString(); };

		protected:
			explicit MatrixVariable(Aris::Dynamic::Object &father, const std::string &name, std::size_t id, const Aris::Core::Matrix &data, bool active = true)
				: VariableTemplate(father, name, id, data, active) {};
			explicit MatrixVariable(Aris::Dynamic::Object &father, const Aris::Core::XmlElement &xml_ele, std::size_t id)
				: VariableTemplate(father, xml_ele, id) 
			{
				this->data_ = model().calculator.calculateExpression(xml_ele.GetText());
				model().calculator.addVariable(name(), this->data_);
			};
		
			friend class ElementPool<Variable>;
		};

		class FloatMarker final :public Marker
		{
		public:
			void setPrtPm(const double *prtPm) { std::copy_n(prtPm, 16, static_cast<double *>(*prt_pm_)); };
			void setPrtPe(const double *prtPe, const char *type = "313") { s_pe2pm(prtPe, *prt_pm_, type); };
			void setPrtPq(const double *prtPq) { s_pq2pm(prtPq, *prt_pm_); };

			explicit FloatMarker(Part &prt, const double *prt_pe = nullptr, const char* eulType = "313")
				:Marker(prt, prt_pe, eulType) {};
		};

		template<int DIMENSION>class JointBaseDim :public Joint
		{
		public:
			static constexpr int Dim() { return DIMENSION; };
			virtual auto dim() const->std::size_t { return DIMENSION; };

		protected:
			explicit JointBaseDim(Object &father, const std::string &name, std::size_t id, Marker &makI, Marker &makJ)
				:Joint(father, name, id, makI, makJ) {};
			explicit JointBaseDim(Object &father, const Aris::Core::XmlElement &xml_ele, std::size_t id)
				:Joint(father, xml_ele, id) {};

			virtual auto csmI()->double* override { return *csmI_; };
			virtual auto csmJ()->double* override { return *csmJ_; };
			virtual auto csf()->double* override { return csf_; };
			virtual auto csa()->double* override { return csa_; };

			double csmI_[6][DIMENSION]{ { 0 } };
			double csmJ_[6][DIMENSION]{ { 0 } };
			double csf_[DIMENSION]{ 0 };
			double csa_[DIMENSION]{ 0 };

		private:
			friend class Model;
		};
		class RevoluteJoint final :public JointBaseDim<5>
		{
		public:
			static const std::string& TypeName() { static const std::string type_name("revolute"); return std::ref(type_name); };
			virtual ~RevoluteJoint() = default;
			virtual auto typeName() const->const std::string& override{ return TypeName(); };

		private:
			explicit RevoluteJoint(Object &father, const std::string &name, std::size_t id, Marker &makI, Marker &makJ);
			explicit RevoluteJoint(Object &father, const Aris::Core::XmlElement &xml_ele, std::size_t id);
			virtual void init() override;

			friend class ElementPool<Joint>;
		};
		class TranslationalJoint final :public JointBaseDim<5>
		{
		public:
			static const std::string& TypeName() { static const std::string type_name("translational"); return std::ref(type_name); };
			virtual ~TranslationalJoint() = default;
			virtual auto typeName() const->const std::string& override{ return TypeName(); };

		private:
			explicit TranslationalJoint(Object &father, const std::string &name, std::size_t id, Marker &makI, Marker &makJ);
			explicit TranslationalJoint(Object &father, const Aris::Core::XmlElement &xml_ele, std::size_t id);
			virtual void init() override;

			friend class ElementPool<Joint>;
		};
		class UniversalJoint final :public JointBaseDim<4>
		{
		public:
			static const std::string& TypeName() { static const std::string type_name("universal"); return std::ref(type_name); };
			virtual ~UniversalJoint() = default;
			virtual auto saveAdams(std::ofstream &file) const -> void override;
			virtual auto typeName() const->const std::string& override{ return TypeName(); };
			virtual auto update()->void override;

		private:
			explicit UniversalJoint(Object &father, const std::string &name, std::size_t id, Marker &makI, Marker &makJ);
			explicit UniversalJoint(Object &father, const Aris::Core::XmlElement &xml_ele, std::size_t id);
			virtual void init() override;

			friend class ElementPool<Joint>;
		};
		class SphericalJoint final :public JointBaseDim<3>
		{
		public:
			static const std::string& TypeName() { static const std::string type_name("spherical"); return std::ref(type_name); };
			virtual ~SphericalJoint() = default;
			virtual auto typeName() const->const std::string& override{ return TypeName(); };

		private:
			explicit SphericalJoint(Object &father, const std::string &Name, std::size_t id, Marker &makI, Marker &makJ);
			explicit SphericalJoint(Object &father, const Aris::Core::XmlElement &xml_ele, std::size_t id);
			virtual void init() override;

			friend class ElementPool<Joint>;
		};

		class SingleComponentMotion final :public Motion
		{
		public:
			static const std::string& TypeName() { static const std::string type_name("single_component_motion"); return std::ref(type_name); };
			virtual ~SingleComponentMotion() = default;
			virtual auto saveXml(Aris::Core::XmlElement &xml_ele) const->void override;
			virtual auto saveAdams(std::ofstream &file) const->void override;
			virtual auto typeName() const->const std::string& override{ return TypeName(); };
			virtual auto update()->void override;

		private:
			explicit SingleComponentMotion(Object &father, const std::string &Name, std::size_t id, Marker &makI, Marker &makJ, int componentAxis);
			explicit SingleComponentMotion(Object &father, const Aris::Core::XmlElement &xml_ele, std::size_t id);
			virtual auto init()->void override;


			int component_axis_;
		private:
			friend class Model;

			friend class ElementPool<Motion>;
		};

		class SingleComponentForce final :public Force
		{
		public:
			static const std::string& TypeName() { static const std::string name("single_component_force"); return std::ref(name); };
			virtual ~SingleComponentForce() = default;
			virtual auto saveXml(Aris::Core::XmlElement &xml_ele) const->void override;
			virtual auto typeName() const->const std::string& override{ return TypeName(); };
			virtual auto groupName()const->const std::string& override{ static const std::string name("sforce"); return std::ref(name); };
			virtual auto update()->void override;

			void setComponentID(std::size_t id) { component_axis_ = id; };
			void setFce(double value) { std::fill_n(fce_value_, 6, 0); fce_value_[component_axis_] = value; };
			void setFce(double value, int componentID) { this->component_axis_ = componentID; setFce(value); };
			double fce()const { return fce_value_[component_axis_]; };

		private:
			explicit SingleComponentForce(Object &father, const std::string &name, std::size_t id, Marker& makI, Marker& makJ, int componentID);
			explicit SingleComponentForce(Object &father, const Aris::Core::XmlElement &xml_ele, std::size_t id);
			virtual void saveAdams(std::ofstream &file) const;

			int component_axis_;
			double fce_value_[6]{ 0 };

			friend class Model;

			friend class ElementPool<Force>;
		};
	}
}

#endif