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
		
		class Marker;
		class Part;
		class Model;

		struct PlanParamBase
		{
			std::int32_t cmd_type{ 0 };
			mutable std::int32_t count{ 0 };
		};
		typedef std::function<int(Model &, const PlanParamBase &)> PlanFunc;

		class Element :public aris::core::Object
		{
		public:
			virtual auto saveAdams(std::ofstream &file) const->void { for (auto &ele : *this)static_cast<const Element &>(ele).saveAdams(file); };
			virtual auto adamsID()const->std::size_t { return id() + 1; };
			virtual auto adamsType()const->const std::string &{ return type(); };
			virtual auto adamsScriptType()const->const std::string &{ return adamsType(); };
			auto model()->Model&;
			auto model()const->const Model&;
			
		protected:
			~Element() = default;
			explicit Element(aris::core::Object &father, std::size_t id, const std::string &name) :Object(father, id, name) {};
			explicit Element(aris::core::Object &father, std::size_t id, const aris::core::XmlElement &xml_ele) :Object(father, id, xml_ele) {};
		};
		class DynEle : public Element
		{
		public:
			virtual auto saveXml(aris::core::XmlElement &xml_ele) const->void override;
			virtual auto update()->void = 0;
			auto active() const->bool { return active_; };
			auto activate(bool active = true)->void { active_ = active; };

		protected:
			virtual ~DynEle() = default;
			DynEle(const DynEle &) = default;
			DynEle(DynEle &&) = default;
			DynEle &operator=(const DynEle &) = default;
			DynEle &operator=(DynEle &&) = default;
			explicit DynEle(aris::core::Object &father, std::size_t id, const std::string &name, bool active = true): Element(father,id, name), active_(active) {};
			explicit DynEle(aris::core::Object &father, std::size_t id, const aris::core::XmlElement &xml_ele);

		private:
			bool active_;
		};
		class Coordinate :public DynEle
		{
		public:
			virtual auto vs() const->const double6& = 0;
			virtual auto as() const->const double6& = 0;
			auto pm() const->const double4x4&{ return pm_; };
			auto pm()->double4x4& { return pm_; };
			auto getPm(double *pm)const->void { std::copy(&this->pm()[0][0], &this->pm()[0][0] + 16, pm); };
			auto getPm(double *pm, const Coordinate &relative_to)const->void { s_inv_pm2pm(*relative_to.pm(), *this->pm(), pm); };
			auto setPm(const double *pm)->void { std::copy(pm, pm + 16, &this->pm()[0][0]); };
			auto setPm(const double *pm, const Coordinate &relative_to)->void { s_pm2pm(*relative_to.pm(), pm, *this->pm()); };
			auto getPe(double *pe, const char *type = "313")const->void { s_pm2pe(*pm(), pe, type); };
			auto getPe(double *pe, const Coordinate &relative_to, const char *type = "313")const->void { double pe_o[6]; getPe(pe_o); s_inv_pe2pe(*relative_to.pm(), pe_o, pe, "313", type); };
			auto setPe(const double *pe, const char *type = "313")->void { s_pe2pm(pe, *pm(), type); };
			auto setPe(const double *pe, const Coordinate &relative_to, const char *type = "313")->void { double pe_o[6]; s_pe2pe(*relative_to.pm(), pe, pe_o, type, "313"); setPe(pe_o); };
			auto getPq(double *pq)const->void { s_pm2pq(*pm(), pq); };
			auto getPq(double *pq, const Coordinate &relative_to)const->void { double pq_o[7]; getPq(pq_o);	s_pq2pq(*relative_to.pm(), pq_o, pq); };
			auto setPq(const double *pq)->void { s_pq2pm(pq, *pm()); };
			auto setPq(const double *pq, const Coordinate &relative_to)->void { double pq_o[7]; s_pq2pq(*relative_to.pm(), pq, pq_o); setPq(pq_o); };
			auto getPp(double *pp)const->void { s_pm2pp(*pm(), pp); };
			auto getPp(double *pp, const Coordinate &relative_to)const->void { double pp_o[3]; getPp(pp_o); s_inv_pp2pp(*relative_to.pm(), pp_o, pp); };
			auto setPp(const double *pp)->void { s_pp2pm(pp, *pm()); };
			auto setPp(const double *pp, const Coordinate &relative_to)->void { double pp_o[3]; s_pp2pp(*relative_to.pm(), pp, pp_o); setPp(pp_o); };
			auto getVs(double *vs)const->void { std::copy_n(this->vs(), 6, vs); };
			auto getVs(double *vs, const Coordinate &relative_to)const->void { s_inv_vs2vs(*relative_to.pm(), relative_to.vs(), this->vs(), vs); };
			auto getVe(double *va)const->void { double pp[3]; getPp(pp); s_vs2va(vs(), pp, va); };
			auto getVe(double *va, const Coordinate &relative_to)const->void;
			auto getVq(double *vq)const->void { double pq[7]; getPq(pq); s_vs2vq(vs(), pq, vq); };
			auto getVq(double *vq, const Coordinate &relative_to)const->void;
			auto getVp(double *vp)const->void { double pp[3]; getPp(pp); s_vs2vp(vs(), pp, vp); };
			auto getVp(double *vp, const Coordinate &relative_to)const->void;
			auto getAs(double *as)const->void { std::copy_n(this->as(), 6, as); };
			auto getAs(double *as, const Coordinate &relative_to)const->void { s_inv_as2as(*relative_to.pm(), relative_to.vs(), relative_to.as(), this->vs(), this->as(), as); };
			auto getAe(double *aa)const->void { double pp[3]; getPp(pp); s_as2aa(vs(), as(), pp, aa); };
			auto getAe(double *aa, const Coordinate &relative_to)const->void;
			auto getAq(double *aq)const->void { double pq[7]; getPq(pq); s_as2aq(vs(), as(), pq, aq); };
			auto getAq(double *aq, const Coordinate &relative_to)const->void;
			auto getAp(double *ap)const->void { double pp[3]; getPp(pp); s_as2ap(vs(), as(), pp, ap); };
			auto getAp(double *ap, const Coordinate &relative_to)const->void;

		protected:
			virtual ~Coordinate() = default;
			explicit Coordinate(Object &father, std::size_t id, const std::string &name, const double *pm = nullptr, bool active = true);
			explicit Coordinate(Object &father, std::size_t id, const aris::core::XmlElement &xml_ele) :DynEle(father, id, xml_ele) {};

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
			explicit Interaction(Object &father, std::size_t id, const std::string &name, Marker &makI, Marker &makJ, bool is_active = true)
				: DynEle(father, id, name, is_active), makI_(&makI), makJ_(&makJ) {};
			explicit Interaction(Object &father, std::size_t id, const aris::core::XmlElement &xml_ele);

		private:
			Marker *makI_;
			Marker *makJ_;
		};
		class Constraint :public Interaction
		{
		public:
			virtual auto update()->void override;
			virtual auto dim() const->std::size_t = 0;
			virtual auto csmPtrI() const->const double* = 0;
			virtual auto csmPtrJ() const->const double* = 0;
			virtual auto csaPtr() const->const double* = 0;
			virtual auto csfPtr() const->const double* = 0;
			virtual auto cspPtr() const->const double* = 0;
			auto colID()const->std::size_t;

		protected:
			virtual ~Constraint();
			explicit Constraint(Object &father, std::size_t id, const std::string &name, Marker &makI, Marker &makJ, bool is_active = true);
			explicit Constraint(Object &father, std::size_t id, const aris::core::XmlElement &xml_ele);

		private:
			struct Imp;
			aris::core::ImpPtr<Imp> imp_;

			friend class Model;
		};
		template<std::size_t DIMENSION>	class ConstraintData
		{
		public:
			typedef double double6xd[6][DIMENSION];
			typedef double doubled[DIMENSION];

			static constexpr int Dim() { return DIMENSION; };
			auto csmI() const->const double6xd &{ return csmI_; };
			auto csmJ() const->const double6xd &{ return csmJ_; };
			auto csa() const->const doubled &{ return csa_; };
			auto csf() const->const doubled &{ return csf_; };
			auto csp() const->const doubled &{ return csp_; };

		protected:
			~ConstraintData() = default;
			ConstraintData() = default;

			double csmI_[6][DIMENSION]{ { 0 } };
			double csmJ_[6][DIMENSION]{ { 0 } };
			double csf_[DIMENSION]{ 0 };
			double csa_[DIMENSION]{ 0 };
			double csp_[DIMENSION]{ 0 };

		private:
			friend class Model;
		};
		template<typename ElementType> class ElementPool:public Element
		{
		public:
			class iterator
			{
			public:
				typedef typename Element::difference_type difference_type;
				typedef typename Element::value_type value_type;
				typedef typename ElementType& reference;
				typedef typename ElementType* pointer;
				typedef std::random_access_iterator_tag iterator_category; //or another tag

				iterator() = default;
				iterator(const iterator& other) = default;
				iterator(const typename Element::iterator iter) :iter_(iter) {}; // 自己添加的
				~iterator() = default;

				auto operator=(const iterator&other)->iterator& = default;
				auto operator==(const iterator&other) const->bool { return iter_ == other.iter_; };
				auto operator!=(const iterator&other) const->bool { return iter_ != other.iter_; };
				auto operator<(const iterator&other) const->bool { return iter_ < other.iter_; }; //optional
				auto operator>(const iterator&other) const->bool { return iter_ > other.iter_; }; //optional
				auto operator<=(const iterator&other) const->bool { return iter_ <= other.iter_; }; //optional
				auto operator>=(const iterator&other) const->bool { return iter_ >= other.iter_; }; //optional

				auto operator++()->iterator& { ++iter_; return *this; };
				auto operator++(int)->iterator { iterator ret(*this); operator++(); return ret; }; //optional
				auto operator--()->iterator& { --iter_; return *this; }; //optional
				auto operator--(int)->iterator { iterator ret(*this); operator--(); return ret; }; //optional
				auto operator+=(size_type size)->iterator& { iter_ += size; return *this; }; //optional
				auto operator+(size_type size) const->iterator { return iterator(iter_ + size); }; //optional
				friend auto operator+(size_type size, const iterator&)->iterator { return iterator(size + iter_); }; //optional
				auto operator-=(size_type size)->iterator& { iter_ -= size; return *this; }; //optional
				auto operator-(size_type size) const->iterator { return iterator(iter_ - size); }; //optional
				auto operator-(iterator iter) const->difference_type { return iterator(iter_ - iter); }; //optional

				auto operator*() const->reference { return static_cast<reference>(iter_.operator*()); };
				auto operator->() const->pointer { return static_cast<pointer>(iter_.operator->()); };
				auto operator[](size_type size) const->reference { return static_cast<reference>(std::ref(*iter_.operator[](size))); }; //optional

			private:
				typename Element::iterator iter_;
				friend class const_iterator;
			};
			class const_iterator
			{
			public:
				typedef typename Element::difference_type difference_type;
				typedef typename Element::value_type value_type;
				typedef typename const ElementType& const_reference;
				typedef typename const ElementType* const_pointer;
				typedef std::random_access_iterator_tag iterator_category; //or another tag

				const_iterator() = default;
				const_iterator(const const_iterator&) = default;
				const_iterator(const iterator& other) :iter_(other.iter_) {};
				const_iterator(const typename Element::const_iterator iter) :iter_(iter) {}; // 自己添加的
				~const_iterator() = default;

				auto operator=(const const_iterator&)->const_iterator& = default;
				auto operator==(const const_iterator& other) const->bool { return iter_ == other.iter_; };
				auto operator!=(const const_iterator& other) const->bool { return iter_ != other.iter_; };
				auto operator<(const const_iterator& other) const->bool { return iter_ < other.iter_; }; //optional
				auto operator>(const const_iterator& other) const->bool { return iter_ > other.iter_; }; //optional
				auto operator<=(const const_iterator& other) const->bool { return iter_ <= other.iter_; }; //optional
				auto operator>=(const const_iterator& other) const->bool { return iter_ >= other.iter_; }; //optional

				auto operator++()->const_iterator& { ++iter_; return *this; };
				auto operator++(int)->const_iterator { const_iterator ret(*this); operator++(); return ret; };  //optional
				auto operator--()->const_iterator& { --iter_; return *this; }; //optional
				auto operator--(int)->const_iterator { const_iterator ret(*this); operator--(); return ret; }; //optional
				auto operator+=(size_type size)->const_iterator& { iter_ += size; return *this; }; //optional
				auto operator+(size_type size) const->const_iterator { return const_iterator(iter_ + size); }; //optional
				friend auto operator+(size_type size, const const_iterator& iter)->const_iterator { return const_iterator(size + iter); }; //optional
				auto operator-=(size_type size)->const_iterator& { iter_ -= size; return *this; }; //optional
				auto operator-(size_type size) const->const_iterator { return const_iterator(iter_ - size); }; //optional
				auto operator-(const_iterator iter) const->difference_type { return iterator(iter_ - iter); }; //optional

				auto operator*() const->const_reference { return static_cast<const_reference>(*iter_); };
				auto operator->() const->const_pointer { return static_cast<const_pointer>(iter_.operator->()); };
				auto operator[](size_type) const->const_reference { return static_cast<const_reference>(std::ref(*iter_->operator[](size))); }; //optional

			private:
				typename Element::const_iterator iter_;
			};
			static auto Type()->const std::string &{ static const std::string type{ ElementType::Type()+"_pool" }; return type; };
			virtual auto type()const->const std::string & override{ return Type(); };
			virtual auto saveAdams(std::ofstream &file) const->void 
			{
				file << "!----------------------------------- " << name() << " -------------------------------------!\r\n!\r\n!\r\n";
				Element::saveAdams(file);
			};
			auto begin()->iterator { return Element::begin(); };
			auto begin()const->const_iterator { return Element::begin(); };
			auto end()->iterator { return Element::end(); };
			auto end()const->const_iterator { return Element::end(); };
			auto at(std::size_t id) const->const ElementType&{ return static_cast<const ElementType&>(Element::at(id)); };
			auto at(std::size_t id)->ElementType& { return static_cast<ElementType&>(Element::at(id)); };
			auto findByName(const std::string &name)const->const_iterator { return Element::findByName(name); };
			auto findByName(const std::string &name)->iterator { return Element::findByName(name); };

		protected:
			~ElementPool() = default;
			explicit ElementPool(aris::core::Object &father, std::size_t id, const std::string &name) :Element(father, id, name) {};
			explicit ElementPool(aris::core::Object &father, std::size_t id, const aris::core::XmlElement &xml_ele) : Element(father, id, xml_ele) {};

		private:
			friend class Model;
			friend class aris::core::Root;
			friend class aris::core::Object;
		};

		class Environment final :public Element
		{
		public:
			static auto Type()->const std::string &{ static const std::string type{ "environment" }; return type; };
			virtual auto type() const->const std::string& override{ return Type(); };
			virtual auto saveXml(aris::core::XmlElement &xml_ele) const->void override;
			virtual auto saveAdams(std::ofstream &file) const->void override;
			auto gravity()const ->const double6&{ return gravity_; };

		private:
			virtual auto operator=(const Object &other)->Object&{ return dynamic_cast<Environment&>(*this) = dynamic_cast<const Environment&>(other); };
			virtual auto operator=(Object &&other)->Object&{ return dynamic_cast<Environment&>(*this) = dynamic_cast<Environment&&>(other); };
			Environment &operator=(const Environment &) = default;
			Environment &operator=(Environment &&) = default;
			virtual ~Environment() = default;
			Environment(const Environment &) = default;
			Environment(Environment &&) = default;
			explicit Environment(Object &father, std::size_t id, const std::string &name) :Element(father, id, name) {};
			explicit Environment(Object &father, std::size_t id, const aris::core::XmlElement &xml_ele);

		private:
			double gravity_[6]{ 0, -9.8, 0, 0, 0, 0 };

			friend class Model;
			friend class aris::core::Root;
			friend class aris::core::Object;
		};
		class Akima final :public Element
		{
		public:
			static auto Type()->const std::string &{ static const std::string type{ "akima" }; return type; };
			virtual auto type() const->const std::string& override{ return Type(); };
			virtual auto saveAdams(std::ofstream &file) const->void override;
			virtual auto saveXml(aris::core::XmlElement &xml_ele) const->void override;
			auto x() const->const std::vector<double> &;
			auto y() const->const std::vector<double> &;
			auto operator()(double x, char derivativeOrder = '0') const ->double;
			auto operator()(int length, const double *x_in, double *y_out, char derivativeOrder = '0') const->void;

		private:
			virtual ~Akima();
			explicit Akima(Object &father, std::size_t id, const std::string &name, int num, const double *x_in, const double *y_in);
			explicit Akima(Object &father, std::size_t id, const aris::core::XmlElement &xml_ele);
			explicit Akima(Object &father, std::size_t id, const std::string &name, const std::list<double> &x_in, const std::list<double> &y_in);
			explicit Akima(Object &father, std::size_t id, const std::string &name, const std::list<std::pair<double, double> > &data_in);
			explicit Akima(Object &father, std::size_t id, const std::string &name, const std::list<std::pair<double, double> > &data_in, double begin_slope, double end_slope);

		private:
			struct Imp;
			aris::core::ImpPtr<Imp> imp;

			friend class Model;
			friend class aris::core::Root;
			friend class aris::core::Object;
		};
		class Script final :public Element
		{
		public:
			static auto Type()->const std::string &{ static const std::string type{ "script" }; return type; };
			virtual auto type() const->const std::string& override{ return Type(); };
			virtual auto saveXml(aris::core::XmlElement &xml_ele) const->void override;
			virtual auto saveAdams(std::ofstream &file) const->void override final;
			auto act(DynEle &ele, bool isActive)->void;
			auto aln(Marker &mak_move, const Marker& mak_target)->void;
			auto sim(std::uint32_t ms_dur, std::uint32_t ms_dt)->void;
			auto empty() const->bool;
			auto endTime()const->std::uint32_t;
			auto doScript(std::uint32_t ms_begin, std::uint32_t ms_end)->void;
			auto clear()->void;

		private:
			virtual ~Script();
			explicit Script(Object &father, std::size_t id, const std::string &name);
			explicit Script(Object &father, std::size_t id, const aris::core::XmlElement &ele);

		private:
			struct Imp;
			aris::core::ImpPtr<Imp> imp_;

			friend class Model;
			friend class aris::core::Root;
			friend class aris::core::Object;
		};
		class Variable :public Element
		{
		public:
			static auto Type()->const std::string &{ static const std::string type{ "variable" }; return type; };
			virtual auto type() const->const std::string& override{ return Type(); };
			virtual auto saveXml(aris::core::XmlElement &xml_ele) const->void override;
			virtual auto toString() const->std::string { return ""; };

		protected:
			virtual ~Variable() = default;
			explicit Variable(aris::core::Object &father, std::size_t id, const std::string &name) : Element(father, id, name) {	};
			explicit Variable(aris::core::Object &father, std::size_t id, const aris::core::XmlElement &xml_ele) : Element(father, id, xml_ele) {};
		
			friend class Model;
			friend class aris::core::Root;
		};
		class Marker :public Coordinate
		{
		public:
			static auto Type()->const std::string &{ static const std::string type{ "marker" }; return type; };
			virtual auto type() const->const std::string& override{ return Type(); };
			virtual auto adamsID()const->std::size_t;
			virtual auto saveXml(aris::core::XmlElement &xml_ele) const->void override;
			virtual auto saveAdams(std::ofstream &file) const->void override;
			virtual auto update()->void override;
			virtual auto vs() const->const double6& override final;
			virtual auto as() const->const double6& override final;
			auto prtPm() const->const double4x4&;
			auto fatherPart() const->const Part&;
			auto fatherPart()->Part&;

		protected:
			virtual ~Marker();
			explicit Marker(Object &father, std::size_t id, const std::string &name, const double *prt_pm = nullptr, Marker *relative_mak = nullptr, bool active = true);//only for child class Part to construct
			explicit Marker(Object &father, std::size_t id, const aris::core::XmlElement &ele);

		private:
			struct Imp;
			aris::core::ImpPtr<Imp> imp;

			friend class Model;
			friend class aris::core::Root;
			friend class aris::core::Object;
		};
		class Part final:public Coordinate
		{
		public:
			static auto Type()->const std::string &{ static const std::string type{ "part" }; return type; };
			virtual auto type() const->const std::string& override{ return Type(); };
			virtual auto saveXml(aris::core::XmlElement &xml_ele) const->void override;
			virtual auto saveAdams(std::ofstream &file) const->void override;
			virtual auto update()->void override;
			virtual auto vs()const->const double6& override final;
			virtual auto as()const->const double6& override final;
			auto rowID()const->std::size_t;
			auto vs()->double6&;
			auto setVs(const double *vs_in)->void { std::copy_n(vs_in, 6, vs()); };
			auto as()->double6&;
			auto setAs(const double *as_in)->void { std::copy_n(as_in, 6, as()); };
			auto invPm() const->const double4x4&;
			auto prtIs() const->const double6x6&;
			auto prtVs() const->const double6&;
			auto prtAs() const->const double6&;
			auto prtFg() const->const double6&;
			auto prtFv() const->const double6&;
			auto prtGravity() const->const double6&;
			auto markerPool()->ElementPool<Marker>&;
			auto markerPool()const->const ElementPool<Marker>&;

		private:
			virtual ~Part();
			Part(Part &&other);
			Part(const Part &other);
			explicit Part(Object &father, std::size_t id, const std::string &name, const double *prt_im = nullptr
				, const double *pm = nullptr, const double *vel = nullptr, const double *acc = nullptr, bool active = true);
			explicit Part(Object &father, std::size_t id, const aris::core::XmlElement &xml_ele);
			Part&operator=(const Part &other);
			Part&operator=(Part &&other);

		private:
			struct Imp;
			aris::core::ImpPtr<Imp> imp_;

			friend class Model;
			friend class aris::core::Root;
			friend class aris::core::Object;
		};
		class Joint :public Constraint
		{
		public:
			static auto Type()->const std::string &{ static const std::string type{ "joint" }; return type; };
			virtual auto type() const->const std::string& override{ return Type(); };
			virtual auto adamsScriptType()const->const std::string& override{ return Type(); };
			virtual auto saveAdams(std::ofstream &file) const->void override;

		protected:
			virtual ~Joint() = default;
			explicit Joint(Object &father, std::size_t id, const std::string &name, Marker &makI, Marker &makJ, bool active = true)
				: Constraint(father, id, name, makI, makJ, active) {};
			explicit Joint(Object &father, std::size_t id, const aris::core::XmlElement &xml_ele)
				: Constraint(father, id, xml_ele) {};

			friend class aris::core::Root;
		};
		class Motion final:public Constraint, public ConstraintData<1>
		{
		public:
			static auto Type()->const std::string & { static const std::string type{ "motion" }; return type; };
			static auto Dim()->std::size_t { return 6; };
			virtual auto type() const->const std::string& override{ return Type(); };
			virtual auto adamsType()const->const std::string& override{ static const std::string type{ "single_component_motion" }; return type; };
			virtual auto adamsScriptType()const->const std::string& override{ return Type(); };
			virtual auto saveXml(aris::core::XmlElement &xml_ele) const->void override;
			virtual auto saveAdams(std::ofstream &file) const->void override;
			virtual auto update()->void override;
			virtual auto dim() const ->std::size_t override { return 1; };
			virtual auto csmPtrI() const->const double* override { return *csmI(); };
			virtual auto csmPtrJ() const->const double* override { return *csmJ(); };
			virtual auto csaPtr() const->const double* override { return csa(); };
			virtual auto csfPtr() const->const double* override { return csf(); };
			virtual auto cspPtr() const->const double* override { return csp(); };
			auto axis()const->int;
			auto frcCoe() const ->const double3&;
			auto setFrcCoe(const double *frc_coe)->void;
			auto motPos() const->double;
			auto setMotPos(double mot_pos)->void;
			auto motVel() const->double;
			auto setMotVel(double mot_vel)->void ;
			auto motAcc() const->double;
			auto setMotAcc(double mot_acc)->void;
			auto motFce() const->double;
			auto setMotFce(double mot_fce)->void;
			auto motFceDyn() const->double;
			auto setMotFceDyn(double mot_dyn_fce)->void;
			auto motFceFrc() const->double;

		protected:
			virtual ~Motion();
			explicit Motion(Object &father, std::size_t id, const std::string &name, Marker &makI, Marker &makJ, int component_axis = 2, const double *frc_coe = nullptr, bool active = true);
			explicit Motion(Object &father, std::size_t id, const aris::core::XmlElement &xml_ele);

			struct Imp;
			aris::core::ImpPtr<Imp> imp_;

			friend class Model;
			friend class aris::core::Root;
			friend class aris::core::Object;
		};
		class GeneralMotion final:public Constraint, public ConstraintData<6>
		{
		public:
			static auto Type()->const std::string &{ static const std::string type{ "general_motion" }; return type; };
			static auto Dim()->std::size_t { return 6; };
			virtual auto type() const->const std::string& override{ return Type(); };
			virtual auto saveAdams(std::ofstream &file) const->void override;
			virtual auto update()->void override;
			virtual auto dim() const ->std::size_t override { return Dim(); };
			virtual auto csmPtrI() const->const double* override { return *csmI(); };
			virtual auto csmPtrJ() const->const double* override { return *csmJ(); };
			virtual auto csaPtr() const->const double* override { return csa(); };
			virtual auto csfPtr() const->const double* override { return csf(); };
			virtual auto cspPtr() const->const double* override { return csp(); };
			auto motPos() const->const double6&;
			auto setMotPos(const double *mot_pos)->void;
			auto motVel() const->const double6&;
			auto setMotVel(const double * mot_vel)->void;
			auto motAcc() const->const double6&;
			auto setMotAcc(const double * mot_acc)->void;
			auto motFce() const->const double6&;
			auto setMotFce(const double * mot_fce)->void;

		protected:
			virtual ~GeneralMotion();
			explicit GeneralMotion(Object &father, std::size_t id, const std::string &name, Marker &makI, Marker &makJ, const std::string& freedom = "xyz123", bool active = true);
			explicit GeneralMotion(Object &father, std::size_t id, const aris::core::XmlElement &xml_ele);

			struct Imp;
			aris::core::ImpPtr<Imp> imp_;

			friend class Model;
			friend class aris::core::Root;
		};
		class Force :public Interaction
		{
		public:
			static auto Type()->const std::string &{ static const std::string type{ "force" }; return type; };
			virtual auto type()const->const std::string & override{ return Type(); };
			auto fceI() const->const double* { return fceI_; };
			auto fceJ() const->const double* { return fceJ_; };

		protected:
			virtual ~Force() = default;
			explicit Force(Object &father, std::size_t id, const std::string &name, Marker &makI, Marker &makJ, bool active = true)
				:Interaction(father, id, name, makI, makJ, active) {};
			explicit Force(Object &father, std::size_t id, const aris::core::XmlElement &xml_ele)
				:Interaction(father, id, xml_ele) {};

			double fceI_[6]{ 0 };
			double fceJ_[6]{ 0 };

			friend class Model;
			friend class aris::core::Root;
		};

		class Model :public aris::core::Root
		{
		public:
			struct SimResult
			{
				std::list<double> time_;
				std::vector<std::list<double> > Pin_, Fin_, Vin_, Ain_;//vector的维数为电机个数，但list维数为时间的维数

				auto clear()->void;
				auto resize(std::size_t size)->void;
				auto saveToTxt(const std::string &filename)const->void;
			};
			using Root::loadXml;
			using Root::saveXml;
			virtual auto loadXml(const aris::core::XmlDocument &xml_doc)->void override;
			virtual auto loadXml(const aris::core::XmlElement &xml_ele)->void override;
			virtual auto saveXml(aris::core::XmlDocument &xml_doc)const->void override;
			virtual auto loadDynEle(const std::string &name)->void;
			virtual auto saveDynEle(const std::string &name)->void;
			virtual auto saveAdams(const std::string &filename, bool using_script = false) const->void;
			virtual auto saveAdams(std::ofstream &file, bool using_script = false) const->void;
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
			auto partPool()->ElementPool<Part>&;
			auto partPool()const->const ElementPool<Part>&;
			auto jointPool()->ElementPool<Joint>&;
			auto jointPool()const->const ElementPool<Joint>&;
			auto motionPool()->ElementPool<Motion>&;
			auto motionPool()const->const ElementPool<Motion>&;
			auto generalMotionPool()->ElementPool<GeneralMotion>&;
			auto generalMotionPool()const->const ElementPool<GeneralMotion>&;
			auto forcePool()->ElementPool<Force>&;
			auto forcePool()const->const ElementPool<Force>&;
			auto markerAt(std::size_t id)const->const Marker&;
			auto markerAt(std::size_t id)->Marker&;
			auto markerSize()const->std::size_t { std::size_t size{ 0 }; for (auto &prt : partPool())size += prt.markerPool().size(); return size; };
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
			auto dynCstPot(double *cst_pot) const->void;
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

			virtual ~Model();
			Model();
		private:
			struct Imp;
			aris::core::ImpPtr<Imp> imp_;
		};

		template<typename VariableType> class VariableTemplate : public Variable
		{
		public:
			auto data()->VariableType& { return data_; };
			auto data()const->const VariableType&{ return data_; };

		protected:
			explicit VariableTemplate(aris::core::Object &father, std::size_t id, const std::string &name, const VariableType &data, bool active = true)
				: Variable(father, id, name), data_(data) {};
			explicit VariableTemplate(aris::core::Object &father, std::size_t id, const aris::core::XmlElement &xml_ele)
				: Variable(father, id, xml_ele) {};

			VariableType data_;
			friend class Model;
		};
		template<std::size_t DIMENSION>	class JointTemplate :public Joint, public ConstraintData<DIMENSION>
		{
		public:
			virtual auto dim() const->std::size_t { return DIMENSION; };
			virtual auto csmPtrI() const->const double* override { return *csmI(); };
			virtual auto csmPtrJ() const->const double* override { return *csmJ(); };
			virtual auto csaPtr() const->const double* override { return csa(); };
			virtual auto csfPtr() const->const double* override { return csf(); };
			virtual auto cspPtr() const->const double* override { return csp(); };

		protected:
			explicit JointTemplate(Object &father, std::size_t id, const std::string &name, Marker &makI, Marker &makJ)
				:Joint(father, id, name, makI, makJ) {};
			explicit JointTemplate(Object &father, std::size_t id, const aris::core::XmlElement &xml_ele)
				:Joint(father, id, xml_ele) {};

		private:
			friend class Model;
		};

		class MatrixVariable final : public VariableTemplate<aris::core::Matrix>
		{
		public:
			static auto Type()->const std::string &{ static const std::string type{ "matrix" }; return type; };
			virtual auto type() const->const std::string& override{ return Type(); };
			virtual auto toString() const->std::string override { return data_.toString(); };

		private:
			virtual ~MatrixVariable() = default;
			explicit MatrixVariable(aris::core::Object &father, std::size_t id, const std::string &name, const aris::core::Matrix &data)
				: VariableTemplate(father, id, name, data) {};
			explicit MatrixVariable(aris::core::Object &father, std::size_t id, const aris::core::XmlElement &xml_ele)
				: VariableTemplate(father, id, xml_ele)
			{
				data_ = model().calculator().calculateExpression(xml_ele.GetText());
				model().calculator().addVariable(name(), data_);
			};

			friend class Model;
			friend class aris::core::Root;
			friend class aris::core::Object;
		};
		class StringVariable final : public VariableTemplate<std::string>
		{
		public:
			static auto Type()->const std::string &{ static const std::string type{ "string" }; return type; };
			virtual auto type() const->const std::string& override{ return Type(); };
			virtual auto toString() const->std::string override { return data_; };

		private:
			virtual ~StringVariable() = default;
			explicit StringVariable(aris::core::Object &father, std::size_t id, const std::string &name, const std::string &data)
				: VariableTemplate(father, id, name, data) {};
			explicit StringVariable(aris::core::Object &father, std::size_t id, const aris::core::XmlElement &xml_ele)
				: VariableTemplate(father, id, xml_ele)
			{
				data_ = std::string(xml_ele.GetText());
				model().calculator().addVariable(name(), data_);
			};

			friend class Model;
			friend class aris::core::Root;
			friend class aris::core::Object;
		};
		class FloatMarker final :public Marker
		{
		public:
			void setPrtPm(const double *prtPm) { std::copy_n(prtPm, 16, const_cast<double *>(*this->prtPm())); };
			void setPrtPe(const double *prtPe, const char *type = "313") { s_pe2pm(prtPe, const_cast<double *>(*prtPm()), type); };
			void setPrtPq(const double *prtPq) { s_pq2pm(prtPq, const_cast<double *>(*prtPm())); };

			explicit FloatMarker(Part &prt, const double *prt_pe = nullptr, const char* eulType = "313")
				:Marker(prt.markerPool(), 0, "float_marker")
			{
				static const double default_prt_pe[6]{ 0,0,0,0,0,0 };
				prt_pe = prt_pe ? prt_pe : default_prt_pe;
				setPrtPe(prt_pe, eulType);
			};
		};
		class RevoluteJoint final :public JointTemplate<5>
		{
		public:
			static const std::string& Type() { static const std::string type_name("revolute"); return std::ref(type_name); };
			virtual auto type() const->const std::string& override{ return Type(); };

		private:
			virtual ~RevoluteJoint() = default;
			explicit RevoluteJoint(Object &father, std::size_t id, const std::string &name, Marker &makI, Marker &makJ);
			explicit RevoluteJoint(Object &father, std::size_t id, const aris::core::XmlElement &xml_ele);

			friend class Model;
			friend class aris::core::Root;
			friend class aris::core::Object;
		};
		class TranslationalJoint final :public JointTemplate<5>
		{
		public:
			static const std::string& Type() { static const std::string type_name("translational"); return std::ref(type_name); };
			virtual auto type() const->const std::string& override{ return Type(); };

		private:
			virtual ~TranslationalJoint() = default;
			explicit TranslationalJoint(Object &father, std::size_t id, const std::string &name, Marker &makI, Marker &makJ);
			explicit TranslationalJoint(Object &father, std::size_t id, const aris::core::XmlElement &xml_ele);

			friend class Model;
			friend class aris::core::Root;
			friend class aris::core::Object;
		};
		class UniversalJoint final :public JointTemplate<4>
		{
		public:
			static const std::string& Type() { static const std::string type_name("universal"); return std::ref(type_name); };
			virtual auto type() const->const std::string& override{ return Type(); };
			virtual auto saveAdams(std::ofstream &file) const -> void override;
			virtual auto update()->void override;
		
		private:
			virtual ~UniversalJoint() = default;
			explicit UniversalJoint(Object &father, std::size_t id, const std::string &name, Marker &makI, Marker &makJ);
			explicit UniversalJoint(Object &father, std::size_t id, const aris::core::XmlElement &xml_ele);

			friend class Model;
			friend class aris::core::Root;
			friend class aris::core::Object;
		};
		class SphericalJoint final :public JointTemplate<3>
		{
		public:
			static const std::string& Type() { static const std::string type_name("spherical"); return std::ref(type_name); };
			virtual auto type() const->const std::string& override{ return Type(); };

		private:
			virtual ~SphericalJoint() = default;
			explicit SphericalJoint(Object &father, std::size_t id, const std::string &Name, Marker &makI, Marker &makJ);
			explicit SphericalJoint(Object &father, std::size_t id, const aris::core::XmlElement &xml_ele);

			friend class Model;
			friend class aris::core::Root;
			friend class aris::core::Object;
		};
		class SingleComponentForce final :public Force
		{
		public:
			static const std::string& Type() { static const std::string name("single_component_force"); return std::ref(name); };
			virtual auto type() const->const std::string& override{ return Type(); };
			virtual auto adamsScriptType()const->const std::string& override{ static const std::string type("sforce"); return std::ref(type); };
			virtual auto saveXml(aris::core::XmlElement &xml_ele) const->void override;
			virtual auto saveAdams(std::ofstream &file) const->void override;
			virtual auto update()->void override;
			auto setComponentID(std::size_t id)->void { component_axis_ = id; };
			auto setFce(double value)->void { std::fill_n(fce_value_, 6, 0); fce_value_[component_axis_] = value; };
			auto setFce(double value, int componentID)->void { this->component_axis_ = componentID; setFce(value); };
			auto fce()const->double { return fce_value_[component_axis_]; };

		private:
			virtual ~SingleComponentForce() = default;
			explicit SingleComponentForce(Object &father, std::size_t id, const std::string &name, Marker& makI, Marker& makJ, int componentID);
			explicit SingleComponentForce(Object &father, std::size_t id, const aris::core::XmlElement &xml_ele);

			int component_axis_;
			double fce_value_[6]{ 0 };

			friend class Model;
			friend class aris::core::Root;
			friend class aris::core::Object;
		};
	}
}

#endif