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
		using double6x6 = double[6][6];
		using double4x4 = double[4][4];
		using double3 = double[3];
		using double6 = double[6];
		
		class Marker;
		class Part;
		class Model;

		struct PlanParamBase
		{
			Model* model_;
			std::int32_t cmd_type_{ 0 };
			mutable std::int32_t count_{ 0 };
		};
		using PlanFunc = std::function<int(Model &, const PlanParamBase &)>;

		class Element :public aris::core::Object
		{
		public:
			virtual auto saveAdams(std::ofstream &file) const->void { for (auto &ele : children())static_cast<const Element &>(ele).saveAdams(file); }
			virtual auto adamsID()const->std::size_t { return id() + 1; }
			virtual auto adamsType()const->const std::string &{ return type(); }
			virtual auto adamsScriptType()const->const std::string &{ return adamsType(); }
			auto model()->Model&;
			auto model()const->const Model&;
			
			auto attributeMatrix(const aris::core::XmlElement &xml_ele, const std::string &attribute_name)const->aris::core::Matrix;
			auto attributeMatrix(const aris::core::XmlElement &xml_ele, const std::string &attribute_name, const aris::core::Matrix& default_value)const->aris::core::Matrix;
			auto attributeMatrix(const aris::core::XmlElement &xml_ele, const std::string &attribute_name, std::size_t m, std::size_t n)const->aris::core::Matrix;
			auto attributeMatrix(const aris::core::XmlElement &xml_ele, const std::string &attribute_name, std::size_t m, std::size_t n, const aris::core::Matrix& default_value)const->aris::core::Matrix;

		protected:
			~Element() = default;
			explicit Element(const std::string &name) :Object(name) {}
			explicit Element(Object &father, const aris::core::XmlElement &xml_ele) :Object(father, xml_ele) {}
		};
		class DynEle : public Element
		{
		public:
			virtual auto saveXml(aris::core::XmlElement &xml_ele) const->void override;
			virtual auto update()->void = 0;
			auto active() const->bool { return active_; }
			auto activate(bool active = true)->void { active_ = active; }

		protected:
			virtual ~DynEle() = default;
			explicit DynEle(const std::string &name, bool active = true) : Element(name), active_(active) {};
			explicit DynEle(Object &father, const aris::core::XmlElement &xml_ele);
			DynEle(const DynEle &) = default;
			DynEle(DynEle &&) = default;
			DynEle& operator=(const DynEle &) = default;
			DynEle& operator=(DynEle &&) = default;

		private:
			bool active_;
		};
		class Coordinate :public DynEle
		{
		public:
			virtual auto pm() const->const double4x4& = 0;
			virtual auto vs() const->const double6& = 0;
			virtual auto as() const->const double6& = 0;
			
			auto getPp(double *pp)const->void;
			auto getPp(const Coordinate &relative_to, double *pp)const->void;
			auto getRe(double *re, const char *type = "313")const->void;
			auto getRe(const Coordinate &relative_to, double *re, const char *type = "313")const->void;
			auto getRq(double *rq)const->void;
			auto getRq(const Coordinate &relative_to, double *rq)const->void;
			auto getRm(double *rm, std::size_t rm_ld = 3)const->void;
			auto getRm(const Coordinate &relative_to, double *rm, std::size_t rm_ld = 3)const->void;
			auto getPe(double *pe, const char *type = "313")const->void;
			auto getPe(const Coordinate &relative_to, double *pe, const char *type = "313")const->void;
			auto getPq(double *pq)const->void;
			auto getPq(const Coordinate &relative_to, double *pq)const->void;
			auto getPm(double *pm)const->void;
			auto getPm(const Coordinate &relative_to, double *pm)const->void;
			auto getVp(double *vp, double *pp = nullptr)const->void;
			auto getVp(const Coordinate &relative_to, double *vp, double *pp = nullptr)const->void;
			auto getWe(double *we, double *re = nullptr, const char *type = "313")const->void;
			auto getWe(const Coordinate &relative_to, double *we, double *re = nullptr, const char *type = "313")const->void;
			auto getWq(double *wq, double *rq = nullptr)const->void;
			auto getWq(const Coordinate &relative_to, double *wq, double *rq = nullptr)const->void;
			auto getWm(double *wm, double *rm = nullptr, std::size_t wm_ld = 3, std::size_t rm_ld = 3)const->void;
			auto getWm(const Coordinate &relative_to, double *wm, double *rm = nullptr, std::size_t wm_ld = 3, std::size_t rm_ld = 3)const->void;
			auto getWa(double *wa, double *rm = nullptr, std::size_t rm_ld = 3)const->void;
			auto getWa(const Coordinate &relative_to, double *wa, double *rm = nullptr, std::size_t rm_ld = 3)const->void;
			auto getVe(double *ve, double *pe = nullptr, const char *type = "313")const->void;
			auto getVe(const Coordinate &relative_to, double *ve, double *pe = nullptr, const char *type = "313")const->void;
			auto getVq(double *vq, double *pq = nullptr)const->void;
			auto getVq(const Coordinate &relative_to, double *vq, double *pq = nullptr)const->void;
			auto getVm(double *vm, double *pm = nullptr)const->void;
			auto getVm(const Coordinate &relative_to, double *vm, double *pm = nullptr)const->void;
			auto getVa(double *va, double *pp = nullptr)const->void;
			auto getVa(const Coordinate &relative_to, double *va, double *pp = nullptr)const->void;
			auto getVs(double *vs, double *pm = nullptr)const->void;
			auto getVs(const Coordinate &relative_to, double *vs, double *pm = nullptr)const->void;
			auto getAp(double *ap, double *vp = nullptr, double *pp = nullptr)const->void;
			auto getAp(const Coordinate &relative_to, double *ap, double *vp = nullptr, double *pp = nullptr)const->void;
			auto getXe(double *xe, double *we = nullptr, double *re = nullptr, const char *type = "313")const->void;
			auto getXe(const Coordinate &relative_to, double *xe, double *we = nullptr, double *re = nullptr, const char *type = "313")const->void;
			auto getXq(double *xq, double *wq = nullptr, double *rq = nullptr)const->void;
			auto getXq(const Coordinate &relative_to, double *xq, double *wq = nullptr, double *rq = nullptr)const->void;
			auto getXm(double *xm, double *wm = nullptr, double *rm = nullptr, std::size_t xm_ld = 3, std::size_t wm_ld = 3, std::size_t rm_ld = 3)const->void;
			auto getXm(const Coordinate &relative_to, double *xm, double *wm = nullptr, double *rm = nullptr, std::size_t xm_ld = 3, std::size_t wm_ld = 3, std::size_t rm_ld = 3)const->void;
			auto getXa(double *xa, double *wa = nullptr, double *rm = nullptr, std::size_t rm_ld = 3)const->void;
			auto getXa(const Coordinate &relative_to, double *xa, double *wa = nullptr, double *rm = nullptr, std::size_t rm_ld = 3)const->void;
			auto getAe(double *ae, double *ve = nullptr, double *pe = nullptr, const char *type = "313")const->void;
			auto getAe(const Coordinate &relative_to, double *ae, double *ve = nullptr, double *pe = nullptr, const char *type = "313")const->void;
			auto getAq(double *aq, double *vq = nullptr, double *pq = nullptr)const->void;
			auto getAq(const Coordinate &relative_to, double *aq, double *vq = nullptr, double *pq = nullptr)const->void;
			auto getAm(double *am, double *vm = nullptr, double *pm = nullptr)const->void;
			auto getAm(const Coordinate &relative_to, double *am, double *vm = nullptr, double *pm = nullptr)const->void;
			auto getAa(double *aa, double *va = nullptr, double *pp = nullptr)const->void;
			auto getAa(const Coordinate &relative_to, double *aa, double *va = nullptr, double *pp = nullptr)const->void;
			auto getAs(double *as, double *vs = nullptr, double *pm = nullptr)const->void;
			auto getAs(const Coordinate &relative_to, double *as, double *vs = nullptr, double *pm = nullptr)const->void;

		protected:
			virtual ~Coordinate() = default;
			explicit Coordinate(const std::string &name, bool active = true);
			explicit Coordinate(Object &father, const aris::core::XmlElement &xml_ele) :DynEle(father, xml_ele) {}
		};
		class Interaction :public DynEle
		{
		public:
			virtual auto saveXml(aris::core::XmlElement &xml_ele) const->void override;
			auto makI()->Marker& { return *makI_; }
			auto makI() const->const Marker&{ return *makI_; }
			auto makJ()->Marker& { return *makJ_; }
			auto makJ() const->const Marker&{ return *makJ_; }

		protected:
			virtual ~Interaction() = default;
			explicit Interaction(const std::string &name, Marker &makI, Marker &makJ, bool is_active = true)
				: DynEle(name, is_active), makI_(&makI), makJ_(&makJ) {}
			explicit Interaction(Object &father, const aris::core::XmlElement &xml_ele);

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
			explicit Constraint(const std::string &name, Marker &makI, Marker &makJ, bool is_active = true);
			explicit Constraint(Object &father, const aris::core::XmlElement &xml_ele);

		private:
			struct Imp;
			aris::core::ImpPtr<Imp> imp_;

			friend class Model;
		};
		template<std::size_t DIM> class ConstraintData
		{
		public:
			using double6xd = double[6][DIM];
			using doubled = double[DIM];

			static constexpr int Dim() { return DIM; }
			auto csmI() const->const double6xd &{ return csmI_; }
			auto csmJ() const->const double6xd &{ return csmJ_; }
			auto csa() const->const doubled &{ return csa_; }
			auto csf() const->const doubled &{ return csf_; }
			auto csp() const->const doubled &{ return csp_; }

		protected:
			~ConstraintData() = default;
			ConstraintData() = default;

			double csmI_[6][DIM]{ { 0 } };
			double csmJ_[6][DIM]{ { 0 } };
			double csf_[DIM]{ 0 };
			double csa_[DIM]{ 0 };
			double csp_[DIM]{ 0 };

		private:
			friend class Model;
		};

		class Environment final :public Element
		{
		public:
			static auto Type()->const std::string &{ static const std::string type{ "Environment" }; return type; }
			virtual auto type() const->const std::string& override{ return Type(); }
			virtual auto adamsType()const->const std::string &{ static const std::string type{ "environment" }; return type; }
			virtual auto saveXml(aris::core::XmlElement &xml_ele) const->void override;
			virtual auto saveAdams(std::ofstream &file) const->void override;
			auto gravity()const ->const double6&{ return gravity_; }

		private:
			virtual auto operator=(const Object &other)->Object&{ return dynamic_cast<Environment&>(*this) = dynamic_cast<const Environment&>(other); }
			virtual auto operator=(Object &&other)->Object&{ return dynamic_cast<Environment&>(*this) = dynamic_cast<Environment&&>(other); }
			Environment &operator=(const Environment &) = default;
			Environment &operator=(Environment &&) = default;
			virtual ~Environment() = default;
			Environment(const Environment &) = default;
			Environment(Environment &&) = default;
			explicit Environment(const std::string &name) :Element(name) {}
			explicit Environment(Object &father, const aris::core::XmlElement &xml_ele);

		private:
			double gravity_[6]{ 0, -9.8, 0, 0, 0, 0 };

			friend class Model;
			friend class aris::core::Root;
			friend class aris::core::Object;
		};
		class Akima final :public Element
		{
		public:
			static auto Type()->const std::string &{ static const std::string type{ "Akima" }; return type; }
			virtual auto type() const->const std::string& override{ return Type(); }
			virtual auto adamsType()const->const std::string &{ static const std::string type{ "akima" }; return type; }
			virtual auto saveAdams(std::ofstream &file) const->void override;
			virtual auto saveXml(aris::core::XmlElement &xml_ele) const->void override;
			auto x() const->const std::vector<double> &;
			auto y() const->const std::vector<double> &;
			auto operator()(double x, char derivativeOrder = '0') const ->double;
			auto operator()(int length, const double *x_in, double *y_out, char derivativeOrder = '0') const->void;

		private:
			virtual ~Akima();
			explicit Akima(const std::string &name, int num, const double *x_in, const double *y_in);
			explicit Akima(Object &father, const aris::core::XmlElement &xml_ele);
			explicit Akima(const std::string &name, const std::list<double> &x_in, const std::list<double> &y_in);
			explicit Akima(const std::string &name, const std::list<std::pair<double, double> > &data_in);
			explicit Akima(const std::string &name, const std::list<std::pair<double, double> > &data_in, double begin_slope, double end_slope);

		private:
			struct Imp;
			aris::core::ImpPtr<Imp> imp_;

			friend class Model;
			friend class aris::core::Root;
			friend class aris::core::Object;
		};
		class Script final :public Element
		{
		public:
			static auto Type()->const std::string &{ static const std::string type{ "Script" }; return type; }
			virtual auto type() const->const std::string& override{ return Type(); }
			virtual auto adamsType()const->const std::string &{ static const std::string type{ "script" }; return type; }
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
			explicit Script(const std::string &name);
			explicit Script(Object &father, const aris::core::XmlElement &ele);

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
			static auto Type()->const std::string &{ static const std::string type{ "Variable" }; return type; }
			virtual auto type() const->const std::string& override{ return Type(); }
			virtual auto adamsType()const->const std::string &{ static const std::string type{ "variable" }; return type; }
			virtual auto saveXml(aris::core::XmlElement &xml_ele) const->void override;
			virtual auto toString() const->std::string { return ""; }

		protected:
			virtual ~Variable() = default;
			explicit Variable(const std::string &name) : Element(name) {	}
			explicit Variable(Object &father, const aris::core::XmlElement &xml_ele) : Element(father, xml_ele) {}
		
			friend class Model;
			friend class aris::core::Root;
		};
		class Marker :public Coordinate
		{
		public:
			static auto Type()->const std::string &{ static const std::string type{ "Marker" }; return type; }
			virtual auto type() const->const std::string& override{ return Type(); }
			virtual auto adamsType()const->const std::string &{ static const std::string type{ "marker" }; return type; }
			virtual auto adamsID()const->std::size_t;
			virtual auto saveXml(aris::core::XmlElement &xml_ele) const->void override;
			virtual auto saveAdams(std::ofstream &file) const->void override;
			virtual auto update()->void override;
			virtual auto pm() const->const double4x4& override final;
			virtual auto vs() const->const double6& override final;
			virtual auto as() const->const double6& override final;
			auto prtPm() const->const double4x4&;
			auto fatherPart() const->const Part&;
			auto fatherPart()->Part&;

		protected:
			virtual ~Marker();
			explicit Marker(const std::string &name, const double *prt_pm = nullptr, Marker *relative_mak = nullptr, bool active = true);//only for child class Part to construct
			explicit Marker(Object &father, const aris::core::XmlElement &xml_ele);

		private:
			struct Imp;
			aris::core::ImpPtr<Imp> imp_;

			friend class Model;
			friend class aris::core::Root;
			friend class aris::core::Object;
		};
		class Part final:public Coordinate
		{
		public:
			static auto Type()->const std::string &{ static const std::string type{ "Part" }; return type; }
			virtual auto type() const->const std::string& override{ return Type(); }
			virtual auto adamsType()const->const std::string &{ static const std::string type{ "part" }; return type; }
			virtual auto saveXml(aris::core::XmlElement &xml_ele) const->void override;
			virtual auto saveAdams(std::ofstream &file) const->void override;
			virtual auto update()->void override;
			virtual auto pm()const->const double4x4& override final;
			virtual auto vs()const->const double6& override final;
			virtual auto as()const->const double6& override final;
			auto pm()->double4x4&;
			auto vs()->double6&;
			auto as()->double6&;
			auto invPm() const->const double4x4&;
			auto prtIs() const->const double6x6&;
			auto prtVs() const->const double6&;
			auto prtAs() const->const double6&;
			auto prtFg() const->const double6&;
			auto prtFv() const->const double6&;
			auto prtGravity() const->const double6&;
			auto markerPool()->aris::core::ObjectPool<Marker, Element>&;
			auto markerPool()const->const aris::core::ObjectPool<Marker, Element>&;
			auto rowID()const->std::size_t;
			auto setPp(const double *pp)->void;
			auto setPp(const Coordinate &relative_to, const double *pp)->void;
			auto setRe(const double *re, const char *type = "313")->void;
			auto setRe(const Coordinate &relative_to, const double *re, const char *type = "313")->void;
			auto setRq(const double *rq)->void;
			auto setRq(const Coordinate &relative_to, const double *rq)->void;
			auto setRm(const double *rm, std::size_t rm_ld = 3)->void;
			auto setRm(const Coordinate &relative_to, const double *rm, std::size_t rm_ld = 3)->void;
			auto setPe(const double *pe, const char *type = "313")->void;
			auto setPe(const Coordinate &relative_to, const double *pe, const char *type = "313")->void;
			auto setPq(const double *pq)->void;
			auto setPq(const Coordinate &relative_to, const double *pq)->void;
			auto setPm(const double *pm)->void;
			auto setPm(const Coordinate &relative_to, const double *pm)->void;
			auto setVp(const double *vp, const double *pp = nullptr)->void;
			auto setVp(const Coordinate &relative_to, const double *vp, const double *pp = nullptr)->void;
			auto setWe(const double *we, const double *re = nullptr, const char *type = "313")->void;
			auto setWe(const Coordinate &relative_to, const double *we, const double *re = nullptr, const char *type = "313")->void;
			auto setWq(const double *wq, const double *rq = nullptr)->void;
			auto setWq(const Coordinate &relative_to, const double *wq, const double *rq = nullptr)->void;
			auto setWm(const double *wm, const double *rm = nullptr, std::size_t wm_ld = 3, std::size_t rm_ld = 3)->void;
			auto setWm(const Coordinate &relative_to, const double *wm, const double *rm = nullptr, std::size_t wm_ld = 3, std::size_t rm_ld = 3)->void;
			auto setWa(const double *wa, const double *rm = nullptr, std::size_t rm_ld = 3)->void;
			auto setWa(const Coordinate &relative_to, const double *wa, const double *rm = nullptr, std::size_t rm_ld = 3)->void;
			auto setVe(const double *ve, const double *pe = nullptr, const char *type = "313")->void;
			auto setVe(const Coordinate &relative_to, const double *ve, const double *pe = nullptr, const char *type = "313")->void;
			auto setVq(const double *vq, const double *pq = nullptr)->void;
			auto setVq(const Coordinate &relative_to, const double *vq, const double *pq = nullptr)->void;
			auto setVm(const double *vm, const double *pm = nullptr)->void;
			auto setVm(const Coordinate &relative_to, const double *vm, const double *pm = nullptr)->void;
			auto setVa(const double *va, const double *pp = nullptr)->void;
			auto setVa(const Coordinate &relative_to, const double *va, const double *pp = nullptr)->void;
			auto setVs(const double *vs, const double *pm = nullptr)->void;
			auto setVs(const Coordinate &relative_to, const double *vs, const double *pm = nullptr)->void;
			auto setAp(const double *ap, const double *vp = nullptr, const double *pp = nullptr)->void;
			auto setAp(const Coordinate &relative_to, const double *ap, const double *vp = nullptr, const double *pp = nullptr)->void;
			auto setXe(const double *xe, const double *we = nullptr, const double *re = nullptr, const char *type = "313")->void;
			auto setXe(const Coordinate &relative_to, const double *xe, const double *we = nullptr, const double *re = nullptr, const char *type = "313")->void;
			auto setXq(const double *xq, const double *wq = nullptr, const double *rq = nullptr)->void;
			auto setXq(const Coordinate &relative_to, const double *xq, const double *wq = nullptr, const double *rq = nullptr)->void;
			auto setXm(const double *xm, const double *wm = nullptr, const double *rm = nullptr, std::size_t xm_ld = 3, std::size_t wm_ld = 3, std::size_t rm_ld = 3)->void;
			auto setXm(const Coordinate &relative_to, const double *xm, const double *vm = nullptr, const double *rm = nullptr, std::size_t xm_ld = 3, std::size_t wm_ld = 3, std::size_t rm_ld = 3)->void;
			auto setXa(const double *xa, const double *wa = nullptr, const double *rm = nullptr, std::size_t rm_ld = 3)->void;
			auto setXa(const Coordinate &relative_to, const double *xa, const double *wa = nullptr, const double *rm = nullptr, std::size_t rm_ld = 3)->void;
			auto setAe(const double *ae, const double *ve = nullptr, const double *pe = nullptr, const char *type = "313")->void;
			auto setAe(const Coordinate &relative_to, const double *ae, const double *ve = nullptr, const double *pe = nullptr, const char *type = "313")->void;
			auto setAq(const double *aq, const double *vq = nullptr, const double *pq = nullptr)->void;
			auto setAq(const Coordinate &relative_to, const double *aq, const double *vq = nullptr, const double *pq = nullptr)->void;
			auto setAm(const double *am, const double *vm = nullptr, const double *pm = nullptr)->void;
			auto setAm(const Coordinate &relative_to, const double *am, const double *vm = nullptr, const double *pm = nullptr)->void;
			auto setAa(const double *aa, const double *va = nullptr, const double *pp = nullptr)->void;
			auto setAa(const Coordinate &relative_to, const double *aa, const double *va = nullptr, const double *pp = nullptr)->void;
			auto setAs(const double *as, const double *vs = nullptr, const double *pm = nullptr)->void;
			auto setAs(const Coordinate &relative_to, const double *as, const double *vs = nullptr, const double *pm = nullptr)->void;

		private:
			virtual ~Part();
			Part(Part &&other);
			Part(const Part &other);
			explicit Part(const std::string &name, const double *prt_im = nullptr
				, const double *pm = nullptr, const double *vel = nullptr, const double *acc = nullptr, bool active = true);
			explicit Part(Object &father, const aris::core::XmlElement &xml_ele);
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
			static auto Type()->const std::string &{ static const std::string type{ "Joint" }; return type; }
			virtual auto type() const->const std::string& override{ return Type(); }
			virtual auto adamsType()const->const std::string &{ static const std::string type{ "joint" }; return type; }
			virtual auto saveAdams(std::ofstream &file) const->void override;

		protected:
			virtual ~Joint() = default;
			explicit Joint(const std::string &name, Marker &makI, Marker &makJ, bool active = true)
				: Constraint(name, makI, makJ, active) {}
			explicit Joint(Object &father, const aris::core::XmlElement &xml_ele): Constraint(father, xml_ele) {}

			friend class aris::core::Root;
		};
		class Motion final:public Constraint, public ConstraintData<1>
		{
		public:
			static auto Type()->const std::string & { static const std::string type{ "Motion" }; return type; }
			static auto Dim()->std::size_t { return 6; }
			virtual auto type() const->const std::string& override{ return Type(); }
			virtual auto adamsType()const->const std::string& override{ static const std::string type{ "single_component_motion" }; return type; }
			virtual auto adamsScriptType()const->const std::string& override{ return Type(); }
			virtual auto saveXml(aris::core::XmlElement &xml_ele) const->void override;
			virtual auto saveAdams(std::ofstream &file) const->void override;
			virtual auto update()->void override;
			virtual auto dim() const ->std::size_t override { return 1; }
			virtual auto csmPtrI() const->const double* override { return *csmI(); }
			virtual auto csmPtrJ() const->const double* override { return *csmJ(); }
			virtual auto csaPtr() const->const double* override { return csa(); }
			virtual auto csfPtr() const->const double* override { return csf(); }
			virtual auto cspPtr() const->const double* override { return csp(); }
			auto axis()const->int;
			auto frcCoe() const ->const double3&;
			auto setFrcCoe(const double *frc_coe)->void;
			auto motPos() const->double;
			auto setMotPos(double mot_pos)->void;
			auto motVel() const->double;
			auto setMotVel(double mot_vel)->void;
			auto motAcc() const->double;
			auto setMotAcc(double mot_acc)->void;
			auto motFce() const->double;
			auto setMotFce(double mot_fce)->void;
			auto motFceDyn() const->double;
			auto setMotFceDyn(double mot_dyn_fce)->void;
			auto motFceFrc() const->double;
			auto absID()const->std::size_t;
			auto slaID()const->std::size_t;
			auto phyID()const->std::size_t;

		protected:
			virtual ~Motion();
			explicit Motion(const std::string &name, Marker &makI, Marker &makJ, int component_axis = 2, const double *frc_coe = nullptr, bool active = true);
			explicit Motion(Object &father, const aris::core::XmlElement &xml_ele);

			struct Imp;
			aris::core::ImpPtr<Imp> imp_;

			friend class Model;
			friend class aris::core::Root;
			friend class aris::core::Object;
		};
		class GeneralMotion final:public Constraint, public ConstraintData<6>
		{
		public:
			static auto Type()->const std::string &{ static const std::string type{ "GeneralMotion" }; return type; }
			static auto Dim()->std::size_t { return 6; }
			virtual auto type() const->const std::string& override{ return Type(); }
			virtual auto adamsType()const->const std::string& override{ static const std::string type{ "general_motion" }; return type; }
			virtual auto saveAdams(std::ofstream &file) const->void override;
			virtual auto update()->void override;
			virtual auto dim() const ->std::size_t override { return Dim(); }
			virtual auto csmPtrI() const->const double* override { return *csmI(); }
			virtual auto csmPtrJ() const->const double* override { return *csmJ(); }
			virtual auto csaPtr() const->const double* override { return csa(); }
			virtual auto csfPtr() const->const double* override { return csf(); }
			virtual auto cspPtr() const->const double* override { return csp(); }
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
			explicit GeneralMotion(const std::string &name, Marker &makI, Marker &makJ, const std::string& freedom = "xyz123", bool active = true);
			explicit GeneralMotion(Object &father, const aris::core::XmlElement &xml_ele);

			struct Imp;
			aris::core::ImpPtr<Imp> imp_;

			friend class Model;
			friend class aris::core::Root;
		};
		class Force :public Interaction
		{
		public:
			static auto Type()->const std::string &{ static const std::string type{ "Force" }; return type; }
			virtual auto type()const->const std::string & override{ return Type(); }
			virtual auto adamsType()const->const std::string& override{ static const std::string type{ "force" }; return type; }
			auto fceI() const->const double* { return fceI_; }
			auto fceJ() const->const double* { return fceJ_; }

		protected:
			virtual ~Force() = default;
			explicit Force(const std::string &name, Marker &makI, Marker &makJ, bool active = true)
				:Interaction(name, makI, makJ, active) {}
			explicit Force(Object &father, const aris::core::XmlElement &xml_ele):Interaction(father, xml_ele) {}

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
			auto scriptPool()->aris::core::ObjectPool<Script, Element>&;
			auto scriptPool()const->const aris::core::ObjectPool<Script, Element>&;
			auto variablePool()->aris::core::ObjectPool<Variable, Element>&;
			auto variablePool()const->const aris::core::ObjectPool<Variable, Element>&;
			auto akimaPool()->aris::core::ObjectPool<Akima, Element>&;
			auto akimaPool()const->const aris::core::ObjectPool<Akima, Element>&;
			auto partPool()->aris::core::ObjectPool<Part, Element>&;
			auto partPool()const->const aris::core::ObjectPool<Part, Element>&;
			auto jointPool()->aris::core::ObjectPool<Joint, Element>&;
			auto jointPool()const->const aris::core::ObjectPool<Joint, Element>&;
			auto motionPool()->aris::core::ObjectPool<Motion, Element>&;
			auto motionPool()const->const aris::core::ObjectPool<Motion, Element>&;
			auto generalMotionPool()->aris::core::ObjectPool<GeneralMotion, Element>&;
			auto generalMotionPool()const->const aris::core::ObjectPool<GeneralMotion, Element>&;
			auto forcePool()->aris::core::ObjectPool<Force, Element>&;
			auto forcePool()const->const aris::core::ObjectPool<Force, Element>&;
			auto markerSize()const->std::size_t { std::size_t size{ 0 }; for (auto &prt : partPool())size += prt.markerPool().size(); return size; }
			auto motionAtAbs(std::size_t abs_id)->Motion&;
			auto motionAtAbs(std::size_t abs_id)const->const Motion&;
			auto motionAtPhy(std::size_t phy_id)->Motion&;
			auto motionAtPhy(std::size_t phy_id)const->const Motion&;
			auto motionAtSla(std::size_t sla_id)->Motion&;
			auto motionAtSla(std::size_t sla_id)const->const Motion&;
			auto ground()->Part&;
			auto ground()const->const Part&;

			/// 约束矩阵C为m x n维的矩阵，惯量矩阵为m x m维的矩阵
			/// 约束力为n维的向量，约束加速度为n维向量
			/// 部件力为m维的向量，部件加速度为m维向量
			/// 动力学为所求的未知量为部件加速度和约束力，其他均为已知
			virtual auto dyn()->void;
			auto dynDimM()const->std::size_t;
			auto dynDimN()const->std::size_t;
			auto dynDim()const->std::size_t { return dynDimN() + dynDimM(); }
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
			virtual auto kinFromPin()->void {}
			virtual auto kinFromVin()->void {}
			/// 静态仿真
			auto simKin(const PlanFunc &func, const PlanParamBase &param, std::size_t akima_interval = 1)->SimResult;
			/// 动态仿真
			auto simDyn(const PlanFunc &func, const PlanParamBase &param, std::size_t akima_interval = 1, Script *script = nullptr)->SimResult;
			/// 直接生成Adams模型，依赖SimDynAkima
			auto simToAdams(const std::string &filename, const PlanFunc &func, const PlanParamBase &param, int ms_dt = 10, Script *script = nullptr)->SimResult;

			virtual ~Model();
			Model(const std::string &name = "Root");
		private:
			struct Imp;
			aris::core::ImpPtr<Imp> imp_;
			friend class Motion;
		};

		template<typename VariableType> class VariableTemplate : public Variable
		{
		public:
			auto data()->VariableType& { return data_; }
			auto data()const->const VariableType&{ return data_; }

		protected:
			explicit VariableTemplate(const std::string &name, const VariableType &data, bool active = true): Variable(name), data_(data) {}
			explicit VariableTemplate(Object &father, const aris::core::XmlElement &xml_ele): Variable(father, xml_ele) {}

			VariableType data_;
			friend class Model;
		};
		template<std::size_t DIM>	class JointTemplate :public Joint, public ConstraintData<DIM>
		{
		public:
			virtual auto dim() const->std::size_t { return DIM; }
			virtual auto csmPtrI() const->const double* override { return *ConstraintData<DIM>::csmI(); }
			virtual auto csmPtrJ() const->const double* override { return *ConstraintData<DIM>::csmJ(); }
			virtual auto csaPtr() const->const double* override { return ConstraintData<DIM>::csa(); }
			virtual auto csfPtr() const->const double* override { return ConstraintData<DIM>::csf(); }
			virtual auto cspPtr() const->const double* override { return ConstraintData<DIM>::csp(); }

		protected:
			explicit JointTemplate(const std::string &name, Marker &makI, Marker &makJ)	:Joint(name, makI, makJ) {}
			explicit JointTemplate(Object &father, const aris::core::XmlElement &xml_ele):Joint(father, xml_ele) {}

		private:
			friend class Model;
		};

		class MatrixVariable final : public VariableTemplate<aris::core::Matrix>
		{
		public:
			static auto Type()->const std::string &{ static const std::string type{ "MatrixVariable" }; return type; }
			virtual auto type() const->const std::string& override{ return Type(); }
			virtual auto adamsType()const->const std::string& override{ static const std::string type{ "matrix" }; return type; }
			virtual auto toString() const->std::string override { return data_.toString(); }

		private:
			virtual ~MatrixVariable() = default;
			explicit MatrixVariable(const std::string &name, const aris::core::Matrix &data)
				: VariableTemplate(name, data) {}
			explicit MatrixVariable(Object &father, const aris::core::XmlElement &xml_ele): VariableTemplate(father, xml_ele)
			{
				data_ = model().calculator().calculateExpression(xml_ele.GetText());
				model().calculator().addVariable(name(), data_);
			}

			friend class Model;
			friend class aris::core::Root;
			friend class aris::core::Object;
		};
		class StringVariable final : public VariableTemplate<std::string>
		{
		public:
			static auto Type()->const std::string &{ static const std::string type{ "StringVariable" }; return type; }
			virtual auto type() const->const std::string& override{ return Type(); }
			virtual auto adamsType()const->const std::string& override{ static const std::string type{ "string" }; return type; }
			virtual auto toString() const->std::string override { return data_; }

		private:
			virtual ~StringVariable() = default;
			explicit StringVariable(const std::string &name, const std::string &data): VariableTemplate(name, data) {}
			explicit StringVariable(Object &father, const aris::core::XmlElement &xml_ele): VariableTemplate(father, xml_ele)
			{
				data_ = std::string(xml_ele.GetText());
				model().calculator().addVariable(name(), data_);
			}

			friend class Model;
			friend class aris::core::Root;
			friend class aris::core::Object;
		};
		class FloatMarker final :public Marker
		{
		public:
			void setPrtPm(const double *prtPm) { std::copy_n(prtPm, 16, const_cast<double *>(*this->prtPm())); }
			void setPrtPe(const double *prtPe, const char *type = "313") { s_pe2pm(prtPe, const_cast<double *>(*prtPm()), type); }
			void setPrtPq(const double *prtPq) { s_pq2pm(prtPq, const_cast<double *>(*prtPm())); }

			explicit FloatMarker(Part &prt, const double *prt_pe = nullptr, const char* eu_type = "313") :Marker("float_marker")
			{
				static const double default_prt_pe[6]{ 0,0,0,0,0,0 };
				prt_pe = prt_pe ? prt_pe : default_prt_pe;
				setPrtPe(prt_pe, eu_type);
			}
		};
		class RevoluteJoint final :public JointTemplate<5>
		{
		public:
			static const std::string& Type() { static const std::string type("RevoluteJoint"); return type; }
			virtual auto type() const->const std::string& override{ return Type(); }
			virtual auto adamsType()const->const std::string& override{ static const std::string type("revolute"); return type; }

		private:
			virtual ~RevoluteJoint() = default;
			explicit RevoluteJoint(const std::string &name, Marker &makI, Marker &makJ);
			explicit RevoluteJoint(Object &father, const aris::core::XmlElement &xml_ele);

			friend class Model;
			friend class aris::core::Root;
			friend class aris::core::Object;
		};
		class TranslationalJoint final :public JointTemplate<5>
		{
		public:
			static const std::string& Type() { static const std::string type("TranslationalJoint"); return type; }
			virtual auto type() const->const std::string& override{ return Type(); }
			virtual auto adamsType()const->const std::string& override{ static const std::string type("translational"); return type;}

		private:
			virtual ~TranslationalJoint() = default;
			explicit TranslationalJoint(const std::string &name, Marker &makI, Marker &makJ);
			explicit TranslationalJoint(Object &father, const aris::core::XmlElement &xml_ele);

			friend class Model;
			friend class aris::core::Root;
			friend class aris::core::Object;
		};
		class UniversalJoint final :public JointTemplate<4>
		{
		public:
			static const std::string& Type() { static const std::string type("UniversalJoint"); return type; }
			virtual auto type() const->const std::string& override{ return Type(); }
			virtual auto adamsType()const->const std::string& override{ static const std::string type("universal"); return type; }
			virtual auto saveAdams(std::ofstream &file) const -> void override;
			virtual auto update()->void override;
		
		private:
			virtual ~UniversalJoint() = default;
			explicit UniversalJoint(const std::string &name, Marker &makI, Marker &makJ);
			explicit UniversalJoint(Object &father, const aris::core::XmlElement &xml_ele);

			friend class Model;
			friend class aris::core::Root;
			friend class aris::core::Object;
		};
		class SphericalJoint final :public JointTemplate<3>
		{
		public:
			static const std::string& Type() { static const std::string type("SphericalJoint"); return type; }
			virtual auto type() const->const std::string& override{ return Type(); }
			virtual auto adamsType()const->const std::string& override{ static const std::string type("spherical"); return type; }

		private:
			virtual ~SphericalJoint() = default;
			explicit SphericalJoint(const std::string &Name, Marker &makI, Marker &makJ);
			explicit SphericalJoint(Object &father, const aris::core::XmlElement &xml_ele);

			friend class Model;
			friend class aris::core::Root;
			friend class aris::core::Object;
		};
		class SingleComponentForce final :public Force
		{
		public:
			static const std::string& Type() { static const std::string type("SingleComponentForce"); return type; }
			virtual auto type() const->const std::string& override{ return Type(); }
			virtual auto adamsType()const->const std::string& override{ static const std::string type("single_component_force"); return type; }
			virtual auto adamsScriptType()const->const std::string& override{ static const std::string type("sforce"); return std::ref(type); }
			virtual auto saveXml(aris::core::XmlElement &xml_ele) const->void override;
			virtual auto saveAdams(std::ofstream &file) const->void override;
			virtual auto update()->void override;
			auto setComponentID(std::size_t id)->void { component_axis_ = id; }
			auto setFce(double value)->void { std::fill_n(fce_value_, 6, 0); fce_value_[component_axis_] = value; }
			auto setFce(double value, int componentID)->void { this->component_axis_ = componentID; setFce(value); }
			auto fce()const->double { return fce_value_[component_axis_]; }

		private:
			virtual ~SingleComponentForce() = default;
			explicit SingleComponentForce(const std::string &name, Marker& makI, Marker& makJ, int componentID);
			explicit SingleComponentForce(Object &father, const aris::core::XmlElement &xml_ele);

			int component_axis_;
			double fce_value_[6]{ 0 };

			friend class Model;
			friend class aris::core::Root;
			friend class aris::core::Object;
		};
	}
}

#endif
