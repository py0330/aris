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

namespace aris
{
	namespace dynamic
	{
		typedef double double6x6[6][6];
		typedef double double4x4[4][4];
		typedef double double3[3];
		typedef double double6[6];

		template<typename Data> class ImpPtr
		{
		public:
			~ImpPtr() = default;
			ImpPtr() :data_unique_ptr_(new Data) {};
			ImpPtr(const ImpPtr &other):data_unique_ptr_(new Data(*other.data_unique_ptr_)){};
			ImpPtr(ImpPtr &&other) :data_unique_ptr_(std::move(other.data_unique_ptr_)) {};
			ImpPtr& operator=(const ImpPtr &other) { *data_unique_ptr_ = *other.data_unique_ptr_; return *this; };
			ImpPtr& operator=(ImpPtr &&other) { *data_unique_ptr_ = std::move(*other.data_unique_ptr_); return *this; };
			template<typename ...Args>
			ImpPtr(Args ...args) :data_unique_ptr_(new Data(args...)) {};

			auto operator->()const -> Data* { return data_unique_ptr_.get(); };
			auto operator*()const -> Data& { return *data_unique_ptr_; };

		private:
			const std::unique_ptr<Data> data_unique_ptr_;
		};

		class Marker;
		class Part;
		class Model;

		struct SimResult
		{
			std::list<double> time_;
			std::vector<std::list<double> > Pin_, Fin_, Vin_, Ain_;//vector的维数为电机个数，但list维数为时间的维数

			auto clear()->void
			{
				time_.clear();
				Pin_.clear();
				Fin_.clear();
				Vin_.clear();
				Ain_.clear();
			};
			auto resize(std::size_t size)->void
			{
				clear();

				Pin_.resize(size);
				Fin_.resize(size);
				Vin_.resize(size);
				Ain_.resize(size);
			};
			auto saveToTxt(const std::string &filename)const->void
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
		struct PlanParamBase
		{
			std::int32_t cmd_type{ 0 };
			mutable std::int32_t count{ 0 };
		};
		typedef std::function<int(Model &, const PlanParamBase &)> PlanFunc;

		class Object
		{
		public:
			virtual ~Object() = default;
			virtual auto model()->Model& { return *model_; };
			virtual auto model()const->const Model&{ return *model_; };
			virtual auto saveXml(aris::core::XmlElement &xml_ele) const->void { xml_ele.DeleteChildren(); xml_ele.SetName(name().c_str()); };
			virtual auto saveAdams(std::ofstream &file) const->void {};
			auto name() const->const std::string&{ return name_; };
			auto father()->Object& { return *father_; };
			auto father()const->const Object&{ return *father_; };

		protected:
			Object(const Object &) = default;
			Object(Object &&) = default;
			Object &operator=(const Object &) = default;
			Object &operator=(Object &&) = default;
			explicit Object(Object &father, const std::string &name) :model_(&father.model()), father_(&father), name_(name) {};
			explicit Object(Object &father, const aris::core::XmlElement &xml_ele) :model_(&father.model()), father_(&father), name_(xml_ele.name()) {};

		private:
			Model *model_;
			Object *father_;
			std::string name_;
		};
		class Element :public Object
		{
		public:
			virtual ~Element() = default;
			virtual auto typeName() const->const std::string& = 0;
			virtual auto groupName()const->const std::string& = 0;
			virtual auto adamsTypeName() const->const std::string&{ return typeName(); };
			virtual auto adamsGroupName()const->const std::string&{ return groupName(); };
			virtual auto saveXml(aris::core::XmlElement &xml_ele) const->void override;
			auto save(const std::string &name)->void;
			auto load(const std::string &name)->void;
			auto id() const->std::size_t { return id_; };
			auto adamsID()const->std::size_t { return id() + 1; };

		protected:
			Element(const Element &) = default;
			Element(Element &&) = default;
			Element &operator=(const Element &) = default;
			Element &operator=(Element &&) = default;
			explicit Element(Object &father, const std::string &name, std::size_t id) : Object(father, name), id_(id) {};
			explicit Element(Object &father, const aris::core::XmlElement &xml_ele, std::size_t id) : Object(father, xml_ele), id_(id) {};

		private:
			std::size_t id_;
			std::map<std::string, std::shared_ptr<Element> > save_data_map_;
		};
		class DynEle : public Element
		{
		public:
			virtual ~DynEle() = default;
			virtual auto saveXml(aris::core::XmlElement &xml_ele) const->void override;
			virtual auto update()->void = 0;
			auto active() const->bool { return active_; };
			auto activate(bool active = true)->void { active_ = active; };

		protected:
			DynEle(const DynEle &) = default;
			DynEle(DynEle &&) = default;
			DynEle &operator=(const DynEle &) = default;
			DynEle &operator=(DynEle &&) = default;
			explicit DynEle(Object &father, const std::string &name, std::size_t id, bool active = true)
				: Element(father, name, id), active_(active) {};
			explicit DynEle(Object &father, const aris::core::XmlElement &xml_ele, std::size_t id);
			
		private:
			bool active_;
		};
		class Coordinate :public DynEle
		{
		public:
			virtual ~Coordinate() = default;
			virtual auto vel() const->const double6& = 0;
			virtual auto acc() const->const double6& = 0;
			auto pm() const->const double4x4&{ return pm_; };
			auto pm()->double4x4& { return pm_; };
			auto getPm(double *pm)->void { std::copy_n(static_cast<const double *>(*this->pm()), 16, pm); };
			auto getPe(double *pe, const char *type = "313")const->void { s_pm2pe(*pm(), pe, type); };
			auto getPq(double *pq)const->void { s_pm2pq(*pm(), pq); };
			auto setPm(const double *pm)->void { std::copy_n(pm, 16, static_cast<double*>(*this->pm())); };
			auto setPe(const double *pe, const char *type = "313")->void { s_pe2pm(pe, *pm(), type); };
			auto setPq(const double *pq)->void { s_pq2pm(pq, *pm()); };
			auto getVel(double *vel)const->void { std::copy_n(this->vel(), 6, vel); };
			auto getAcc(double *acc)const->void { std::copy_n(this->acc(), 6, acc); };

		protected:
			explicit Coordinate(Object &father, const std::string &name, std::size_t id, const double *pm = nullptr, bool active = true);
			explicit Coordinate(Object &father, const aris::core::XmlElement &xml_ele, std::size_t id) :DynEle(father, xml_ele, id) {};

		private:
			double pm_[4][4];
		};
		class Interaction :public DynEle
		{
		public:
			virtual auto saveXml(aris::core::XmlElement &xml_ele) const->void override;
			auto makI()->Marker& { return *makI_; };
			auto makI() const->const Marker&{ return *makI_; };
			auto makJ()->Marker& { return *makJ_; };
			auto makJ() const->const Marker&{ return *makJ_; };
			
		protected:
			virtual ~Interaction() = default;
			explicit Interaction(Object &father, const std::string &name, std::size_t id, Marker &makI, Marker &makJ, bool is_active = true)
				: DynEle(father, name, id, is_active), makI_(&makI), makJ_(&makJ) {};
			explicit Interaction(Object &father, const aris::core::XmlElement &xml_ele, std::size_t id);

		private:
			Marker *makI_;
			Marker *makJ_;
		};
		class Constraint :public Interaction
		{
		public:
			virtual auto saveAdams(std::ofstream &file) const->void override;
			virtual auto update()->void override;
			virtual auto dim() const->std::size_t = 0;
			auto cstMtxI() const->const double* { return const_cast<Constraint*>(this)->csmI(); };
			auto cstMtxJ() const->const double* { return const_cast<Constraint*>(this)->csmJ(); };
			auto cstAcc() const->const double* { return const_cast<Constraint*>(this)->csa(); };
			auto cstFce() const->const double* { return const_cast<Constraint*>(this)->csf(); };

		protected:
			virtual ~Constraint() = default;
			explicit Constraint(Object &father, const std::string &name, std::size_t id, Marker &makI, Marker &makJ, bool is_active = true)
				: Interaction(father, name, id, makI, makJ, is_active) {};
			explicit Constraint(Object &father, const aris::core::XmlElement &xml_ele, std::size_t id)
				: Interaction(father, xml_ele, id) {};

		private:
			virtual auto csmI()->double* = 0;
			virtual auto csmJ()->double* = 0;
			virtual auto csa()->double* = 0;
			virtual auto csf()->double* = 0;

			std::size_t col_id_;

			friend class Model;
		};

		template <typename ElementType>	class ElementPool : public Object
		{
		public:
			virtual auto saveXml(aris::core::XmlElement &xml_ele) const->void override
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
			auto save(const std::string &name)->void { for (auto &ele : element_vec_)ele->save(name); };
			auto load(const std::string &name)->void { for (auto &ele : element_vec_)ele->load(name); };
			template<typename ChildType, typename ...Args>
			auto add(const std::string & name, Args ...args)->ElementType&
			{
				if (find(name))throw std::runtime_error("element \"" + name + "\" already exists, can't add()");
				auto ret = new ChildType(this->father(), name, element_vec_.size(), args...);
				element_vec_.push_back(std::unique_ptr<ElementType>(ret));
				return std::ref(*ret);
			}
			auto add(const aris::core::XmlElement &xml_ele)->ElementType&
			{
				if (find(xml_ele.name()))throw std::runtime_error(ElementType::TypeName() + " \"" + xml_ele.name() + "\" already exists, can't add element");
				std::string type = xml_ele.Attribute("type") ? xml_ele.Attribute("type") : ElementType::TypeName();
				if (model().typeInfoMap().find(type) == model().typeInfoMap().end())
					throw std::runtime_error(std::string("can't find type ") + type);

				auto new_ele = model().typeInfoMap().at(type).newFromXml(this->father(), xml_ele, element_vec_.size());
				if (!dynamic_cast<ElementType*>(new_ele))
				{
					delete new_ele;
					throw std::runtime_error("can't add \"" + type + "\" element to " + ElementType::TypeName() + " group");
				}
				element_vec_.push_back(std::unique_ptr<ElementType>(dynamic_cast<ElementType*>(new_ele)));

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

				return p == element_vec_.end() ? nullptr : p->get();
			}
			auto find(const std::string &name) const->const ElementType *{ return const_cast<ElementPool *>(this)->find(name); }
			auto size() const ->std::size_t { return element_vec_.size(); };
			auto begin()->typename std::vector<std::unique_ptr<ElementType>>::iterator { return element_vec_.begin(); };
			auto begin() const ->typename std::vector<std::unique_ptr<ElementType>>::const_iterator { return element_vec_.begin(); };
			auto end()->typename std::vector<std::unique_ptr<ElementType>>::iterator { return element_vec_.end(); };
			auto end() const ->typename std::vector<std::unique_ptr<ElementType>>::const_iterator { return element_vec_.end(); };
			auto clear() -> void { element_vec_.clear(); };

		private:
			~ElementPool() = default;
			ElementPool(const ElementPool &other) = delete;
			ElementPool(ElementPool && other) = delete;
			ElementPool& operator=(const ElementPool &) = delete;
			ElementPool& operator=(ElementPool &&other) = default;
			explicit ElementPool(Object &father, const std::string &name) :Object(father, name) {};
			explicit ElementPool(Object &father, const aris::core::XmlElement &xml_ele) :Object(father, xml_ele)
			{
				for (auto ele = xml_ele.FirstChildElement(); ele != nullptr; ele = ele->NextSiblingElement())
				{
					add(*ele);
				}
			};

		private:
			std::vector<std::unique_ptr<ElementType> > element_vec_;

			friend class Model;
		};
		template <>	class ElementPool<Marker> : public Object
		{
		public:
			virtual auto saveXml(aris::core::XmlElement &xml_ele) const->void override;
			virtual auto saveAdams(std::ofstream &file) const->void override;
			auto save(const std::string &name)->void;
			auto load(const std::string &name)->void;
			auto add(const std::string & name, const double *prt_pm = nullptr, Marker *relative_mak = nullptr, bool active = true)->Marker&;
			auto add(const aris::core::XmlElement &xml_ele)->Marker&;
			auto at(std::size_t id)->Marker&;
			auto at(std::size_t id) const->const Marker&;
			auto find(const std::string &name)->Marker *;
			auto find(const std::string &name) const->const Marker *;
			auto size() const->std::size_t;
			auto begin()->std::vector<std::shared_ptr<Marker>>::iterator;
			auto begin() const->std::vector<std::shared_ptr<Marker>>::const_iterator;
			auto end()->std::vector<std::shared_ptr<Marker>>::iterator;
			auto end() const->std::vector<std::shared_ptr<Marker>>::const_iterator;
			auto clear() -> void;

		private:
			~ElementPool() = default;
			ElementPool(const ElementPool &other) = default;
			ElementPool(ElementPool && other) = default;
			ElementPool& operator=(const ElementPool &) = default;
			ElementPool& operator=(ElementPool &&other) = default;
			explicit ElementPool(Object &father, const std::string &name) :Object(father, name) {};
			explicit ElementPool(Object &father, const aris::core::XmlElement &xml_ele);

		private:
			std::vector<std::shared_ptr<Marker> > element_vec_;

			friend class Part;
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
			explicit Environment(Object &father, const aris::core::XmlElement &xml_ele);
			virtual auto saveXml(aris::core::XmlElement &xml_ele) const->void override;
			virtual auto saveAdams(std::ofstream &file) const->void override;

		private:
			double gravity_[6]{ 0, -9.8, 0, 0, 0, 0 };

			friend class Part;
			friend class Model;
		};
		class Akima final:public Element
		{
		public:
			static auto TypeName()->const std::string &{ static const std::string type{ "akima" }; return type; };
			virtual ~Akima();
			virtual auto saveAdams(std::ofstream &file) const->void override;
			virtual auto saveXml(aris::core::XmlElement &xml_ele) const->void override;
			virtual auto typeName() const->const std::string& override{ return TypeName(); };
			virtual auto groupName()const->const std::string& override final{ return TypeName(); };
			auto x() const->const std::vector<double> &;
			auto y() const->const std::vector<double> &;
			auto operator()(double x, char derivativeOrder = '0') const ->double;
			auto operator()(int length, const double *x_in, double *y_out, char derivativeOrder = '0') const->void;

		protected:
			explicit Akima(Object &father, const std::string &name, std::size_t id, int num, const double *x_in, const double *y_in);
			explicit Akima(Object &father, const aris::core::XmlElement &xml_ele, std::size_t id);
			explicit Akima(Object &father, const std::string &name, std::size_t id, const std::list<double> &x_in, const std::list<double> &y_in);
			explicit Akima(Object &father, const std::string &name, std::size_t id, const std::list<std::pair<double, double> > &data_in);
			explicit Akima(Object &father, const std::string &name, std::size_t id, const std::list<std::pair<double, double> > &data_in, double begin_slope, double end_slope);

		private:
			struct Imp;
			ImpPtr<Imp> imp;
			
			friend class ElementPool<Akima>;
			friend class Model;
		};
		class Script final :public Element
		{
		public:
			static auto TypeName()->const std::string &{ static const std::string type{ "script" }; return type; };
			virtual ~Script();
			virtual auto saveXml(aris::core::XmlElement &xml_ele) const->void override;
			virtual auto saveAdams(std::ofstream &file) const->void override final;
			virtual auto typeName() const->const std::string& override{ return TypeName(); };
			virtual auto groupName()const->const std::string& override final{ return TypeName(); };
			auto act(DynEle &ele, bool isActive)->void;
			auto aln(Marker &mak_move, const Marker& mak_target)->void;
			auto sim(std::uint32_t ms_dur, std::uint32_t ms_dt)->void;
			auto empty() const->bool;
			auto endTime()const->std::uint32_t;
			auto doScript(std::uint32_t ms_begin, std::uint32_t ms_end)->void;
			auto clear()->void;

		protected:
			explicit Script(Object &father, const std::string &name, std::size_t id);
			explicit Script(Object &father, const aris::core::XmlElement &ele, std::size_t id);

		private:
			struct Imp;
			ImpPtr<Imp> imp;

			friend class ElementPool<Script>;
			friend class Model;
		};
		class Variable :public Element
		{
		public:
			static auto TypeName()->const std::string &{ static const std::string type{ "variable" }; return type; };
			virtual ~Variable() = default;
			virtual auto groupName()const->const std::string& override final{ return TypeName(); };
			virtual auto saveXml(aris::core::XmlElement &xml_ele) const->void override
			{
				Element::saveXml(xml_ele);
				xml_ele.SetText(this->toString().c_str());
			}
			virtual auto toString() const->std::string { return ""; };

		protected:
			explicit Variable(Object &father, const std::string &name, std::size_t id) : Element(father, name, id) {};
			explicit Variable(Object &father, const aris::core::XmlElement &xml_ele, std::size_t id) : Element(father, xml_ele, id) {};

		private:
			friend class ElementPool<Variable>;
		};
		class Marker :public Coordinate
		{
		public:
			static auto TypeName()->const std::string &{ static const std::string type{ "marker" }; return type; };
			virtual ~Marker();
			virtual auto typeName() const->const std::string& override{ return TypeName(); };
			virtual auto groupName()const->const std::string& override final{ return TypeName(); };
			virtual auto saveXml(aris::core::XmlElement &xml_ele) const->void override;
			virtual auto saveAdams(std::ofstream &file) const->void override;
			virtual auto update()->void override;
			virtual auto vel() const->const double6& override final;
			virtual auto acc() const->const double6& override final;
			auto prtPm() const->const double4x4&;
			auto fatherPart() const->const Part&;
			auto fatherPart()->Part&;
			
		protected:
			explicit Marker(Object &father, const std::string &name, std::size_t id, const double *prt_pm = nullptr, Marker *relative_mak = nullptr, bool active = true);//only for child class Part to construct
			explicit Marker(Object &father, const aris::core::XmlElement &ele, std::size_t id);

		private:
			struct Imp;
			ImpPtr<Imp> imp;

			friend class ElementPool<Marker>;
			//friend class Part;
			//friend class FloatMarker;
			//friend class Script;
			friend class Model;
		};
		class Part :public Coordinate
		{
		public:
			static auto TypeName()->const std::string &{ static const std::string type{ "part" }; return type; };
			virtual ~Part();
			virtual auto typeName() const->const std::string& override{ return TypeName(); };
			virtual auto groupName()const->const std::string& override final{ return TypeName(); };
			virtual auto saveXml(aris::core::XmlElement &xml_ele) const->void override;
			virtual auto saveAdams(std::ofstream &file) const->void override;
			virtual auto update()->void override;
			virtual auto vel()const->const double6& override final;
			virtual auto acc()const->const double6& override final;
			auto rowID()const->std::size_t;
			auto vel()->double6&;
			auto acc()->double6&;
			auto invPm() const->const double4x4&;
			auto prtIm() const->const double6x6&;
			auto prtVel() const->const double6&;
			auto prtAcc() const->const double6&;
			auto prtFg() const->const double6&;
			auto prtFv() const->const double6&;
			auto prtGravity() const->const double6&;
			auto setVel(const double *vel_in)->void { std::copy_n(vel_in, 6, vel()); };
			auto setAcc(const double *acc_in)->void { std::copy_n(acc_in, 6, acc()); };
			auto markerPool()->ElementPool<Marker>&;
			auto markerPool()const->const ElementPool<Marker>&;

		private:
			explicit Part(Object &father, const std::string &name, std::size_t id, const double *prt_im = nullptr
				, const double *pm = nullptr, const double *vel = nullptr, const double *acc = nullptr, bool active = true);
			explicit Part(Object &father, const aris::core::XmlElement &xml_ele, std::size_t id);

		private:
			struct Imp;
			ImpPtr<Imp> imp;
			
			friend class Model;
			friend class ElementPool<Part>;
		};
		class Joint :public Constraint
		{
		public:
			static auto TypeName()->const std::string &{ static const std::string type{ "joint" }; return type; };
			virtual ~Joint() = default;
			virtual auto groupName()const->const std::string& override final{ return TypeName(); };

		protected:
			explicit Joint(Object &father, const std::string &name, std::size_t id, Marker &makI, Marker &makJ, bool active = true)
				: Constraint(father, name, id, makI, makJ, active) {};
			explicit Joint(Object &father, const aris::core::XmlElement &xml_ele, std::size_t id)
				: Constraint(father, xml_ele, id) {};

			friend class ElementPool<Joint>;
		};
		class Motion :public Constraint
		{
		public:
			static const std::string &TypeName() { static const std::string type{ "motion" }; return type; };
			virtual ~Motion() = default;
			virtual auto saveXml(aris::core::XmlElement &xml_ele) const->void override;
			virtual auto groupName()const->const std::string& override final{ return TypeName(); };
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

		protected:
			explicit Motion(Object &father, const std::string &name, std::size_t id, Marker &makI, Marker &makJ, const double *frc_coe = nullptr, bool active = true);
			explicit Motion(Object &father, const aris::core::XmlElement &xml_ele, std::size_t id);
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

			friend class ElementPool<Motion>;
			friend class Model;
		};
		class Force :public Interaction
		{
		public:
			static auto TypeName()->const std::string &{ static const std::string type{ "force" }; return type; };
			virtual ~Force() = default;
			virtual auto groupName()const->const std::string& override final{ return TypeName(); };
			virtual auto update()->void = 0;
			auto fceI() const->const double* { return fceI_; };
			auto fceJ() const->const double* { return fceJ_; };

		protected:
			explicit Force(Object &father, const std::string &name, std::size_t id, Marker &makI, Marker &makJ, bool active = true)
				:Interaction(father, name, id, makI, makJ, active) {};
			explicit Force(Object &father, const aris::core::XmlElement &xml_ele, std::size_t id)
				:Interaction(father, xml_ele, id) {};

			double fceI_[6]{ 0 };
			double fceJ_[6]{ 0 };

			friend class ElementPool<Force>;
			friend class Model;
		};

		class Model :public Object
		{
		public:
			struct TypeInfo
			{
				std::function<Element*(Object &, const aris::core::XmlElement &xml_ele, std::size_t id)> newFromXml;
				std::function<Element*(const Element &)> newFromElement;
				std::function<Element&(Element &, const Element &)> assignOperator;
				
				template<typename ChildType> static auto create()->TypeInfo
				{
					TypeInfo info;

					info.newFromXml = [](Object &father, const aris::core::XmlElement &xml_ele, std::size_t id)->Element*
					{
						return new ChildType(father, xml_ele, id);
					};
					info.newFromElement = [](const Element &otherElement)->Element*
					{
						auto &t = typeid(ChildType);
						auto p = dynamic_cast<const ChildType *>(&otherElement);
						
						if (!dynamic_cast<const ChildType *>(&otherElement))throw std::runtime_error("invalid type in type_info_map of assignOperator of cast 2");
						return new ChildType(dynamic_cast<const ChildType &>(otherElement));
					};
					info.assignOperator = [](Element &thisElement, const Element &otherElement)->Element&
					{
						if (!dynamic_cast<ChildType *>(&thisElement))throw std::runtime_error("invalid type in type_info_map of assignOperator of cast 1");
						if (!dynamic_cast<const ChildType *>(&otherElement))throw std::runtime_error("invalid type in type_info_map of assignOperator of cast 2");

						return dynamic_cast<ChildType &>(thisElement) = dynamic_cast<const ChildType &>(otherElement);
					};

					return info;
				}
			};

		public:
			virtual ~Model();
			Model(const Model & other) = delete;
			Model(Model &&) = delete;
			Model &operator=(const Model &) = delete;
			Model &operator=(Model &&) = delete;
			explicit Model(const std::string & name = "Model");

			virtual auto model()->Model& override { return *this; };
			virtual auto model()const->const Model& override{ return *this; };
			virtual auto load(const std::string &name)->void;
			virtual auto save(const std::string &name)->void;
			virtual auto loadDynEle(const std::string &name)->void;
			virtual auto saveDynEle(const std::string &name)->void;
			virtual auto loadXml(const char* filename)->void { loadXml(std::string(filename)); };
			virtual auto loadXml(const std::string &filename)->void;
			virtual auto loadXml(const aris::core::XmlDocument &xml_doc)->void;
			virtual auto loadXml(const aris::core::XmlElement &xml_ele)->void;
			virtual auto saveXml(const char *filename) const->void { saveXml(std::string(filename)); };
			virtual auto saveXml(const std::string &filename) const->void;
			virtual auto saveXml(aris::core::XmlDocument &xml_doc)const->void;
			virtual auto saveXml(aris::core::XmlElement &xml_ele)const->void override;
			virtual auto saveAdams(const std::string &filename, bool using_script = false) const->void;
			virtual auto saveAdams(std::ofstream &file, bool using_script = false) const->void;
			virtual auto clear()->void;
			auto calculator()->aris::core::Calculator&;
			auto calculator()const ->const aris::core::Calculator&;
			auto environment()->aris::dynamic::Environment&;
			auto environment()const ->const aris::dynamic::Environment&;
			auto scriptPool()->ElementPool<Script>&;
			auto scriptPool()const->const ElementPool<Script>&;
			auto variablePool()->ElementPool<Variable>&;
			auto variablePool()const->const ElementPool<Variable>&;
			auto akimaPool()->ElementPool<Akima>&;
			auto akimaPool()const->const ElementPool<Akima>&;
			auto markerPool()->ElementPool<Marker>&;
			auto markerPool()const->const ElementPool<Marker>&;
			auto partPool()->ElementPool<Part>&;
			auto partPool()const->const ElementPool<Part>&;
			auto jointPool()->ElementPool<Joint>&;
			auto jointPool()const->const ElementPool<Joint>&;
			auto motionPool()->ElementPool<Motion>&;
			auto motionPool()const->const ElementPool<Motion>&;
			auto forcePool()->ElementPool<Force>&;
			auto forcePool()const->const ElementPool<Force>&;
			auto ground()->Part&;
			auto ground()const->const Part&;

			/// 约束矩阵C为m x n维的矩阵，惯量矩阵为m x m维的矩阵
			/// 约束力为n维的向量，约束加速度为n维向量
			/// 部件力为m维的向量，部件加速度为m维向量
			/// 动力学为所求的未知量为部件加速度和约束力，其他均为已知
			virtual auto dyn()->void;
			auto dynDimM()const->std::size_t;
			auto dynDimN()const->std::size_t;
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
			auto clbDimM()const->std::size_t;
			auto clbDimN()const->std::size_t;
			auto clbDimGam()const->std::size_t;
			auto clbDimFrc()const->std::size_t;
			auto clbSetInverseMethod(std::function<void(int n, double *A)> inverse_method)->void;
			auto clbPre()->void;
			auto clbUpd()->void;
			auto clbMtx(double *clb_D, double *clb_b) const->void;
			auto clbUkn(double *clb_x) const->void;

			/// 仿真函数
			virtual auto kinFromPin()->void {};
			virtual auto kinFromVin()->void {};
			/// 静态仿真
			auto simKin(const PlanFunc &func, const PlanParamBase &param, std::size_t akima_interval = 1)->SimResult;
			/// 动态仿真
			auto simDyn(const PlanFunc &func, const PlanParamBase &param, std::size_t akima_interval = 1, Script *script = nullptr)->SimResult;
			/// 直接生成Adams模型，依赖SimDynAkima
			auto simToAdams(const std::string &filename, const PlanFunc &func, const PlanParamBase &param, int ms_dt = 10, Script *script = nullptr)->SimResult;

			template<typename ChildType>
			auto registerElementType()->void
			{
				if (typeInfoMap().find(ChildType::TypeName()) != typeInfoMap().end())
					throw std::runtime_error("can not register element type \"" + ChildType::TypeName() + "\" cause it already registered");

				typeInfoMap().insert(std::make_pair(ChildType::TypeName(), TypeInfo::template create<ChildType>()));
			};
			auto typeInfoMap()->std::map<std::string, TypeInfo>&;
			
		private:
			struct Imp;
			ImpPtr<Imp> imp;

		protected:
			friend class Environment;
			friend class Part;
			friend class Motion;
			friend class Marker;
			friend class Force;
			friend class MatrixVariable;
			template<typename ElementType> friend class ElementPool;
		};

		template <typename Type> class VariableTemplate : public Variable
		{
		public:
			virtual ~VariableTemplate() = default;
			auto data()->Type& { return data_; };
			auto data()const->const Type&{ return data_; };

		protected:
			explicit VariableTemplate(aris::dynamic::Object &father, const std::string &name, std::size_t id, const Type &data, bool active = true)
				: Variable(father, name, id), data_(data) {};
			explicit VariableTemplate(aris::dynamic::Object &father, const aris::core::XmlElement &xml_ele, std::size_t id)
				: Variable(father, xml_ele, id) {};

			Type data_;
			friend class Model;
		};
		class MatrixVariable final: public VariableTemplate<aris::core::Matrix>
		{
		public:
			static auto TypeName()->const std::string &{ static const std::string type{ "matrix" }; return type; };
			virtual ~MatrixVariable() = default;
			virtual auto typeName() const->const std::string& override{ return TypeName(); };
			virtual auto toString() const->std::string override { return data_.toString(); };

		protected:
			explicit MatrixVariable(aris::dynamic::Object &father, const std::string &name, std::size_t id, const aris::core::Matrix &data)
				: VariableTemplate(father, name, id, data) {};
			explicit MatrixVariable(aris::dynamic::Object &father, const aris::core::XmlElement &xml_ele, std::size_t id)
				: VariableTemplate(father, xml_ele, id)
			{
				data_ = model().calculator().calculateExpression(xml_ele.GetText());
				model().calculator().addVariable(name(), data_);
			};

			friend class ElementPool<Variable>;
			friend class Model;
		};
		class StringVariable final : public VariableTemplate<std::string>
		{
		public:
			static auto TypeName()->const std::string &{ static const std::string type{ "string" }; return type; };
			virtual ~StringVariable() = default;
			virtual auto typeName() const->const std::string& override{ return TypeName(); };
			virtual auto toString() const->std::string override { return data_; };

		protected:
			explicit StringVariable(aris::dynamic::Object &father, const std::string &name, std::size_t id, const std::string &data)
				: VariableTemplate(father, name, id, data) {};
			explicit StringVariable(aris::dynamic::Object &father, const aris::core::XmlElement &xml_ele, std::size_t id)
				: VariableTemplate(father, xml_ele, id)
			{
				data_ = std::string(xml_ele.GetText());
				model().calculator().addVariable(name(), data_);
			};

			friend class ElementPool<Variable>;
			friend class Model;
		};
		class FloatMarker final :public Marker
		{
		public:
			void setPrtPm(const double *prtPm) { std::copy_n(prtPm, 16, const_cast<double *>(*this->prtPm())); };
			void setPrtPe(const double *prtPe, const char *type = "313") { s_pe2pm(prtPe, const_cast<double *>(*prtPm()), type); };
			void setPrtPq(const double *prtPq) { s_pq2pm(prtPq, const_cast<double *>(*prtPm())); };

			explicit FloatMarker(Part &prt, const double *prt_pe = nullptr, const char* eulType = "313")
				:Marker(prt, "float_marker", 0) 
			{
				static const double default_prt_pe[6]{ 0,0,0,0,0,0 };
				prt_pe = prt_pe ? prt_pe : default_prt_pe;
				setPrtPe(prt_pe, eulType);
			};
		};
		template<std::size_t DIMENSION>	class JointTemplate :public Joint
		{
		public:
			static constexpr int Dim() { return DIMENSION; };
			virtual auto dim() const->std::size_t { return DIMENSION; };

		protected:
			explicit JointTemplate(Object &father, const std::string &name, std::size_t id, Marker &makI, Marker &makJ)
				:Joint(father, name, id, makI, makJ) {};
			explicit JointTemplate(Object &father, const aris::core::XmlElement &xml_ele, std::size_t id)
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
		class RevoluteJoint final :public JointTemplate<5>
		{
		public:
			static const std::string& TypeName() { static const std::string type_name("revolute"); return std::ref(type_name); };
			virtual ~RevoluteJoint() = default;
			virtual auto typeName() const->const std::string& override{ return TypeName(); };

		private:
			explicit RevoluteJoint(Object &father, const std::string &name, std::size_t id, Marker &makI, Marker &makJ);
			explicit RevoluteJoint(Object &father, const aris::core::XmlElement &xml_ele, std::size_t id);

			friend class ElementPool<Joint>;
			friend class Model;
		};
		class TranslationalJoint final :public JointTemplate<5>
		{
		public:
			static const std::string& TypeName() { static const std::string type_name("translational"); return std::ref(type_name); };
			virtual ~TranslationalJoint() = default;
			virtual auto typeName() const->const std::string& override{ return TypeName(); };

		private:
			explicit TranslationalJoint(Object &father, const std::string &name, std::size_t id, Marker &makI, Marker &makJ);
			explicit TranslationalJoint(Object &father, const aris::core::XmlElement &xml_ele, std::size_t id);

			friend class ElementPool<Joint>;
			friend class Model;
		};
		class UniversalJoint final :public JointTemplate<4>
		{
		public:
			static const std::string& TypeName() { static const std::string type_name("universal"); return std::ref(type_name); };
			virtual ~UniversalJoint() = default;
			virtual auto saveAdams(std::ofstream &file) const -> void override;
			virtual auto typeName() const->const std::string& override{ return TypeName(); };
			virtual auto update()->void override;

		private:
			explicit UniversalJoint(Object &father, const std::string &name, std::size_t id, Marker &makI, Marker &makJ);
			explicit UniversalJoint(Object &father, const aris::core::XmlElement &xml_ele, std::size_t id);

			friend class ElementPool<Joint>;
			friend class Model;
		};
		class SphericalJoint final :public JointTemplate<3>
		{
		public:
			static const std::string& TypeName() { static const std::string type_name("spherical"); return std::ref(type_name); };
			virtual ~SphericalJoint() = default;
			virtual auto typeName() const->const std::string& override{ return TypeName(); };

		private:
			explicit SphericalJoint(Object &father, const std::string &Name, std::size_t id, Marker &makI, Marker &makJ);
			explicit SphericalJoint(Object &father, const aris::core::XmlElement &xml_ele, std::size_t id);
	
			friend class ElementPool<Joint>;
			friend class Model;
		};
		class SingleComponentMotion final :public Motion
		{
		public:
			static const std::string& TypeName() { static const std::string type_name("single_component_motion"); return std::ref(type_name); };
			virtual ~SingleComponentMotion() = default;
			virtual auto saveXml(aris::core::XmlElement &xml_ele) const->void override;
			virtual auto saveAdams(std::ofstream &file) const->void override;
			virtual auto typeName() const->const std::string& override{ return TypeName(); };
			virtual auto update()->void override;

		private:
			explicit SingleComponentMotion(Object &father, const std::string &Name, std::size_t id, Marker &makI, Marker &makJ, int componentAxis);
			explicit SingleComponentMotion(Object &father, const aris::core::XmlElement &xml_ele, std::size_t id);

		private:
			int component_axis_;
			friend class ElementPool<Motion>;
			friend class Model;
		};
		class SingleComponentForce final :public Force
		{
		public:
			static const std::string& TypeName() { static const std::string name("single_component_force"); return std::ref(name); };
			virtual ~SingleComponentForce() = default;
			virtual auto saveXml(aris::core::XmlElement &xml_ele) const->void override;
			virtual void saveAdams(std::ofstream &file) const override;
			virtual auto typeName() const->const std::string& override{ return TypeName(); };
			virtual auto adamsGroupName()const->const std::string& override{ static const std::string name("sforce"); return std::ref(name); };
			virtual auto update()->void override;

			void setComponentID(std::size_t id) { component_axis_ = id; };
			void setFce(double value) { std::fill_n(fce_value_, 6, 0); fce_value_[component_axis_] = value; };
			void setFce(double value, int componentID) { this->component_axis_ = componentID; setFce(value); };
			double fce()const { return fce_value_[component_axis_]; };

		private:
			explicit SingleComponentForce(Object &father, const std::string &name, std::size_t id, Marker& makI, Marker& makJ, int componentID);
			explicit SingleComponentForce(Object &father, const aris::core::XmlElement &xml_ele, std::size_t id);
			

			int component_axis_;
			double fce_value_[6]{ 0 };

			friend class ElementPool<Force>;
			friend class Model;
		};
	}
}

#endif