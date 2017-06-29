#ifndef ARIS_DYNAMIC_MODEL_
#define ARIS_DYNAMIC_MODEL_

#include <vector>
#include <array>
#include <map>
#include <string>
#include <memory>
#include <functional>
#include <algorithm>
#include <deque>
#include <type_traits>

#include <aris_core.h>
#include <aris_dynamic_matrix.h>
#include <aris_dynamic_block_matrix.h>
#include <aris_dynamic_screw.h>

namespace aris
{
	namespace dynamic
	{
		using double6x6 = double[6][6];
		using double4x4 = double[4][4];
		using double3 = double[3];
		using double6 = double[6];
		using double7 = double[7];

		class Marker;
		class Part;
		class RevoluteJoint;
		class PrismaticJoint;
		class UniversalJoint;
		class SphericalJoint;
		class Model;

		struct PlanParamBase
		{
			Model* model_;
			std::int32_t cmd_type_{ 0 };
			std::int32_t count_{ 0 };
			double dt{ 1e-3 };
		};
		using PlanFunc = std::function<int(Model &, const PlanParamBase &)>;

		struct PlanParam 
		{
			Model* model_;
			std::int32_t count_;
			void *param_;
			std::int32_t param_size_;
		};
		using PlanFunction = std::function<int(const PlanParam &)>;
		using ParseFunction = std::function<void(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::MsgBase &msg_out)>;

		class Element :public aris::core::Object
		{
		public:
			static auto Type()->const std::string &{ static const std::string type{ "Element" }; return type; }
			auto virtual type() const->const std::string& override{ return Type(); }
			auto model()->Model&;
			auto model()const->const Model&;
			
			auto attributeMatrix(const aris::core::XmlElement &xml_ele, const std::string &attribute_name)const->aris::core::Matrix;
			auto attributeMatrix(const aris::core::XmlElement &xml_ele, const std::string &attribute_name, const aris::core::Matrix& default_value)const->aris::core::Matrix;
			auto attributeMatrix(const aris::core::XmlElement &xml_ele, const std::string &attribute_name, Size m, Size n)const->aris::core::Matrix;
			auto attributeMatrix(const aris::core::XmlElement &xml_ele, const std::string &attribute_name, Size m, Size n, const aris::core::Matrix& default_value)const->aris::core::Matrix;

		protected:
			~Element() = default;
			explicit Element(const std::string &name) :Object(name) {}
			explicit Element(Object &father, const aris::core::XmlElement &xml_ele) :Object(father, xml_ele) {}
			Element(const Element&) = default;
			Element(Element&&) = default;
			Element& operator=(const Element&) = default;
			Element& operator=(Element&&) = default;
		};
		class DynEle : public Element
		{
		public:
			static auto Type()->const std::string &{ static const std::string type{ "DynEle" }; return type; }
			auto virtual type() const->const std::string& override{ return Type(); }
			auto virtual saveXml(aris::core::XmlElement &xml_ele) const->void override;
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
			static auto Type()->const std::string &{ static const std::string type{ "Coordinate" }; return type; }
			auto virtual type() const->const std::string& override{ return Type(); }
			auto virtual prtPm()const->const double4x4& = 0;
			auto virtual prtVs()const->const double6& = 0;
			auto virtual prtAs()const->const double6& = 0;
			auto virtual glbPm()const->const double4x4& = 0;
			auto virtual glbVs()const->const double6& = 0;
			auto virtual glbAs()const->const double6& = 0;
			auto virtual pm() const->const double4x4&{ return glbPm(); };
			auto virtual vs() const->const double6&{ return glbVs(); };
			auto virtual as() const->const double6&{ return glbAs(); };
			auto getPp(double *pp)const->void;
			auto getPp(const Coordinate &relative_to, double *pp)const->void;
			auto getRe(double *re, const char *type = "313")const->void;
			auto getRe(const Coordinate &relative_to, double *re, const char *type = "313")const->void;
			auto getRq(double *rq)const->void;
			auto getRq(const Coordinate &relative_to, double *rq)const->void;
			auto getRm(double *rm, Size rm_ld = 3)const->void;
			auto getRm(const Coordinate &relative_to, double *rm, Size rm_ld = 3)const->void;
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
			auto getWm(double *wm, double *rm = nullptr, Size wm_ld = 3, Size rm_ld = 3)const->void;
			auto getWm(const Coordinate &relative_to, double *wm, double *rm = nullptr, Size wm_ld = 3, Size rm_ld = 3)const->void;
			auto getWa(double *wa, double *rm = nullptr, Size rm_ld = 3)const->void;
			auto getWa(const Coordinate &relative_to, double *wa, double *rm = nullptr, Size rm_ld = 3)const->void;
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
			auto getXm(double *xm, double *wm = nullptr, double *rm = nullptr, Size xm_ld = 3, Size wm_ld = 3, Size rm_ld = 3)const->void;
			auto getXm(const Coordinate &relative_to, double *xm, double *wm = nullptr, double *rm = nullptr, Size xm_ld = 3, Size wm_ld = 3, Size rm_ld = 3)const->void;
			auto getXa(double *xa, double *wa = nullptr, double *rm = nullptr, Size rm_ld = 3)const->void;
			auto getXa(const Coordinate &relative_to, double *xa, double *wa = nullptr, double *rm = nullptr, Size rm_ld = 3)const->void;
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
			Coordinate(const Coordinate &) = default;
			Coordinate(Coordinate &&) = default;
			Coordinate& operator=(const Coordinate &) = default;
			Coordinate& operator=(Coordinate &&) = default;
		};
		class Interaction :public DynEle
		{
		public:
			static auto Type()->const std::string &{ static const std::string type{ "Interaction" }; return type; }
			auto virtual type() const->const std::string& override{ return Type(); }
			auto virtual saveXml(aris::core::XmlElement &xml_ele) const->void override;
			auto makI()->Marker& { return *makI_; }
			auto makI() const->const Marker&{ return *makI_; }
			auto makJ()->Marker& { return *makJ_; }
			auto makJ() const->const Marker&{ return *makJ_; }

		protected:
			virtual ~Interaction() = default;
			explicit Interaction(const std::string &name, Marker &makI, Marker &makJ, bool is_active = true)
				: DynEle(name, is_active), makI_(&makI), makJ_(&makJ) {}
			explicit Interaction(Object &father, const aris::core::XmlElement &xml_ele);
			Interaction(const Interaction &) = default;
			Interaction(Interaction &&) = default;
			Interaction& operator=(const Interaction &) = default;
			Interaction& operator=(Interaction &&) = default;

		private:
			Marker *makI_;
			Marker *makJ_;
		};
		template<Size DIM> class ConstraintData
		{
		public:
			using double6xd = double[6][DIM];
			using doubled = double[DIM];

			static constexpr Size Dim() { return DIM; }
			auto cf() const->const doubled &{ return cf_; }
			auto prtCmI() const->const double6xd &{ return prtCmI_; }
			auto locCmI() const->const double6xd &{ return prtCmI_; }

		protected:
			~ConstraintData() = default;
			ConstraintData() = default;

			double cf_[DIM]{ 0 };
			double prtCmI_[6][DIM]{ { 0 } };
			double locCmI_[6][DIM]{ { 0 } };

		private:
			friend class Model;
		};

		class Environment final :public Element
		{
		public:
			static auto Type()->const std::string &{ static const std::string type{ "Environment" }; return type; }
			auto virtual type() const->const std::string& override{ return Type(); }
			auto virtual saveXml(aris::core::XmlElement &xml_ele) const->void override;
			auto gravity()const ->const double6&{ return gravity_; }

		private:
			auto virtual operator=(const Object &other)->Object&{ return dynamic_cast<Environment&>(*this) = dynamic_cast<const Environment&>(other); }
			auto virtual operator=(Object &&other)->Object&{ return dynamic_cast<Environment&>(*this) = dynamic_cast<Environment&&>(other); }
			
			virtual ~Environment() = default;
			explicit Environment(Object &father, const aris::core::XmlElement &xml_ele);
			explicit Environment(const std::string &name) :Element(name) {}
			Environment(const Environment &) = default;
			Environment(Environment &&) = default;
			Environment &operator=(const Environment &) = default;
			Environment &operator=(Environment &&) = default;

		private:
			double gravity_[6]{ 0, -9.8, 0, 0, 0, 0 };

			friend class Model;
			friend class aris::core::Root;
			friend class aris::core::Object;
		};
		class Variable :public Element
		{
		public:
			static auto Type()->const std::string &{ static const std::string type{ "Variable" }; return type; }
			auto virtual type() const->const std::string& override{ return Type(); }
			auto virtual saveXml(aris::core::XmlElement &xml_ele) const->void override;
			auto virtual toString() const->std::string { return ""; }

		protected:
			virtual ~Variable() = default;
			explicit Variable(Object &father, const aris::core::XmlElement &xml_ele) : Element(father, xml_ele) {}
			explicit Variable(const std::string &name) : Element(name) {}
			Variable(const Variable&) = default;
			Variable(Variable&&) = default;
			Variable& operator=(const Variable&) = default;
			Variable& operator=(Variable&&) = default;

			friend class Model;
			friend class aris::core::Root;
		};
		class Geometry :public Element
		{
		public:
			static auto Type()->const std::string &{ static const std::string type{ "Geometry" }; return type; }
			auto virtual type() const->const std::string& override{ return Type(); }
			auto fatherPart() const->const Part&;
			auto fatherPart()->Part&;

		protected:
			virtual ~Geometry() = default;
			explicit Geometry(Object &father, const aris::core::XmlElement &xml_ele) : Element(father, xml_ele) {}
			explicit Geometry(const std::string &name) : Element(name) {}
			Geometry(const Geometry&) = default;
			Geometry(Geometry&&) = default;
			Geometry& operator=(const Geometry&) = default;
			Geometry& operator=(Geometry&&) = default;

			friend class Model;
			friend class aris::core::Root;
		};
		class Marker :public Coordinate
		{
		public:
			static auto Type()->const std::string &{ static const std::string type{ "Marker" }; return type; }
			auto virtual type() const->const std::string& override{ return Type(); }
			auto virtual saveXml(aris::core::XmlElement &xml_ele) const->void override;
			auto virtual glbPm()const->const double4x4& override;
			auto virtual glbVs()const->const double6& override;
			auto virtual glbAs()const->const double6& override;
			auto virtual prtPm()const->const double4x4& override;
			auto virtual prtVs()const->const double6& override;
			auto virtual prtAs()const->const double6& override;
			auto fatherPart() const->const Part&;
			auto fatherPart()->Part&;

		protected:
			virtual ~Marker();
			explicit Marker(Object &father, const aris::core::XmlElement &xml_ele);
			explicit Marker(const std::string &name, const double *prt_pm = nullptr, bool active = true);
			Marker(const Marker&);
			Marker(Marker&&);
			Marker& operator=(const Marker&);
			Marker& operator=(Marker&&);

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
			auto static Type()->const std::string &{ static const std::string type{ "Part" }; return type; }
			auto virtual type() const->const std::string& override{ return Type(); }
			auto virtual saveXml(aris::core::XmlElement &xml_ele) const->void override;
			auto markerPool()->aris::core::ObjectPool<Marker, Element>&;
			auto markerPool()const->const aris::core::ObjectPool<Marker, Element>&;
			auto geometryPool()->aris::core::ObjectPool<Geometry, Element>&;
			auto geometryPool()const->const aris::core::ObjectPool<Geometry, Element>&;
			auto virtual glbPm()const->const double4x4& override;
			auto virtual glbVs()const->const double6& override;
			auto virtual glbAs()const->const double6& override;
			auto virtual prtPm()const->const double4x4& override;
			auto virtual prtVs()const->const double6& override;
			auto virtual prtAs()const->const double6& override;
			auto updPrtVs()->void;
			auto updPrtAs()->void;
			auto prtIm() const->const double6x6&;
			template<typename IM_TYPE>
			auto cptGlbIm(double *im, IM_TYPE i_t)const->void 
			{
				double tem[36];
				s_tf_n(6, *pm(), *prtIm(), tem);
				s_tf_n(6, *pm(), tem, T(6), im, i_t);
			}
			auto cptGlbFg(double *fg)const->void;
			auto cptGlbFv(double *fv)const->void;
			auto cptGlbPf(double *pf)const->void;
			auto cptPrtFg(double *fg)const->void;
			auto cptPrtFv(double *fv)const->void;
			auto cptPrtPf(double *pf)const->void;
			auto cptPrtVs(double *vs)const->void;
			auto cptPrtAs(double *as)const->void;
			auto setPp(const double *pp)->void;
			auto setPp(const Coordinate &relative_to, const double *pp)->void;
			auto setRe(const double *re, const char *type = "313")->void;
			auto setRe(const Coordinate &relative_to, const double *re, const char *type = "313")->void;
			auto setRq(const double *rq)->void;
			auto setRq(const Coordinate &relative_to, const double *rq)->void;
			auto setRm(const double *rm, Size rm_ld = 3)->void;
			auto setRm(const Coordinate &relative_to, const double *rm, Size rm_ld = 3)->void;
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
			auto setWm(const double *wm, const double *rm = nullptr, Size wm_ld = 3, Size rm_ld = 3)->void;
			auto setWm(const Coordinate &relative_to, const double *wm, const double *rm = nullptr, Size wm_ld = 3, Size rm_ld = 3)->void;
			auto setWa(const double *wa, const double *rm = nullptr, Size rm_ld = 3)->void;
			auto setWa(const Coordinate &relative_to, const double *wa, const double *rm = nullptr, Size rm_ld = 3)->void;
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
			auto setXm(const double *xm, const double *wm = nullptr, const double *rm = nullptr, Size xm_ld = 3, Size wm_ld = 3, Size rm_ld = 3)->void;
			auto setXm(const Coordinate &relative_to, const double *xm, const double *vm = nullptr, const double *rm = nullptr, Size xm_ld = 3, Size wm_ld = 3, Size rm_ld = 3)->void;
			auto setXa(const double *xa, const double *wa = nullptr, const double *rm = nullptr, Size rm_ld = 3)->void;
			auto setXa(const Coordinate &relative_to, const double *xa, const double *wa = nullptr, const double *rm = nullptr, Size rm_ld = 3)->void;
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
			Part(const Part &other);
			Part(Part &&other);
			Part& operator=(const Part &other);
			Part& operator=(Part &&other);
			explicit Part(Object &father, const aris::core::XmlElement &xml_ele);
			explicit Part(const std::string &name, const double *prt_im = nullptr, const double *pm = nullptr, const double *vs = nullptr, const double *as = nullptr, bool active = true);
			

		private:
			struct Imp;
			aris::core::ImpPtr<Imp> imp_;

			friend class Model;
			friend class aris::core::Root;
			friend class aris::core::Object;
		};
		class Constraint :public Interaction
		{
		public:
			auto virtual dim() const->Size = 0;
			auto virtual cptCp(double *cp)const->void;
			auto virtual cptCv(double *cv)const->void;
			auto virtual cptCa(double *ca)const->void;
			template<typename CMI_TYPE, typename CMJ_TYPE>
			auto cptPrtCm(double *cmI, CMI_TYPE cmi_type, double *cmJ, CMJ_TYPE cmj_type)->void
			{
				updPrtCmI();
				s_mc(6, dim(), prtCmPtrI(), dim(), cmI, cmi_type);

				double pm_M2N[4][4];
				s_inv_pm_dot_pm(*makJ().fatherPart().pm(), *makI().fatherPart().pm(), *pm_M2N);
				s_tf_n(dim(), -1.0, *pm_M2N, prtCmPtrI(), dim(), cmJ, cmj_type);
			}
			auto cptPrtCm(double *cmI, double *cmJ)->void { cptPrtCm(cmI, dim(), cmJ, dim()); }
			template<typename CMI_TYPE, typename CMJ_TYPE>
			auto cptGlbCm(double *cmI, CMI_TYPE cmi_type, double *cmJ, CMJ_TYPE cmj_type)->void
			{
				updPrtCmI();
				s_tf_n(dim(), *makI().fatherPart().pm(), prtCmPtrI(), dim(), cmI, cmi_type);
				s_mi(6, dim(), cmI, cmi_type, cmJ, cmj_type);
			}
			auto cptGlbCm(double *cmI, double *cmJ)->void { cptGlbCm(cmI, dim(), cmJ, dim()); }

			auto virtual cfPtr() const->const double* = 0;
			auto virtual setCf(const double *cf)const->void;
			auto virtual prtCmPtrI() const->const double* = 0;
			auto virtual locCmPtrI() const->const double* = 0;



		protected:
			auto virtual updPrtCmI()->void {};

			virtual ~Constraint();
			explicit Constraint(const std::string &name, Marker &makI, Marker &makJ, bool is_active = true);
			explicit Constraint(Object &father, const aris::core::XmlElement &xml_ele);
			Constraint(const Constraint&);
			Constraint(Constraint&&);
			Constraint& operator=(const Constraint&);
			Constraint& operator=(Constraint&&);

		private:
			struct Imp;
			aris::core::ImpPtr<Imp> imp_;

			friend class Model;
		};
		class Joint :public Constraint
		{
		public:
			static auto Type()->const std::string &{ static const std::string type{ "Joint" }; return type; }
			auto virtual type() const->const std::string& override{ return Type(); }

		protected:
			virtual ~Joint() = default;
			explicit Joint(Object &father, const aris::core::XmlElement &xml_ele) : Constraint(father, xml_ele) {}
			explicit Joint(const std::string &name, Marker &makI, Marker &makJ, bool active = true): Constraint(name, makI, makJ, active) {}
			Joint(const Joint &other);
			Joint(Joint &&other);
			Joint& operator=(const Joint &other);
			Joint& operator=(Joint &&other);

			friend class aris::core::Root;
		};
		class Motion final:public Constraint, public ConstraintData<1>
		{
		public:
			static auto Type()->const std::string & { static const std::string type{ "Motion" }; return type; }
			static auto Dim()->Size { return 1; }
			auto virtual type() const->const std::string& override{ return Type(); }
			auto virtual saveXml(aris::core::XmlElement &xml_ele) const->void override;
			auto virtual dim() const ->Size override { return 1; }
			auto virtual cptCp(double *cp)const->void override;
			auto virtual cptCv(double *cv)const->void override;
			auto virtual cptCa(double *ca)const->void override;
			auto virtual cfPtr() const->const double* override { return cf(); }
			auto virtual prtCmPtrI() const->const double* override { return *prtCmI(); }
			auto virtual locCmPtrI() const->const double* override { return *locCmI(); }
			auto axis()const->Size;
			auto mp() const->double;
			auto updMp()->void;
			auto setMp(double mp)->void;
			auto mv() const->double;
			auto updMv()->void;
			auto setMv(double mv)->void;
			auto ma() const->double;
			auto updMa()->void;
			auto setMa(double ma)->void;
			auto mf() const->double;
			auto setMf(double mf)->void;
			auto mfDyn() const->double;
			auto setMfDyn(double mf_dyn)->void;
			auto mfFrc() const->double;
			auto frcCoe() const ->const double3&;
			auto setFrcCoe(const double *frc_coe)->void;
			auto absID()const->Size;
			auto slaID()const->Size;
			auto phyID()const->Size;

		protected:
			virtual ~Motion();
			explicit Motion(Object &father, const aris::core::XmlElement &xml_ele);
			explicit Motion(const std::string &name, Marker &makI, Marker &makJ, Size component_axis = 2, const double *frc_coe = nullptr, Size sla_id = -1, bool active = true);
			Motion(const Motion &other);
			Motion(Motion &&other);
			Motion& operator=(const Motion &other);
			Motion& operator=(Motion &&other);

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
			static auto Dim()->Size { return 6; }
			auto virtual type() const->const std::string& override{ return Type(); }
			auto virtual dim() const ->Size override { return Dim(); }
			auto virtual cptCp(double *cp)const->void override;
			auto virtual cptCv(double *cv)const->void override;
			auto virtual cptCa(double *ca)const->void override;
			auto virtual cfPtr() const->const double* override { return cf(); }
			auto virtual prtCmPtrI() const->const double* override { return *prtCmI(); }
			auto virtual locCmPtrI() const->const double* override { return *locCmI(); }
			auto mpm()const->const double4x4&;
			auto updMpm()->void;
			auto setMpe(const double* pe, const char *type = "313")->void;
			auto setMpq(const double* pq)->void;
			auto setMpm(const double* pm)->void;
			auto getMpe(double* pe, const char *type = "313")const->void;
			auto getMpq(double* pq)const->void;
			auto getMpm(double* pm)const->void;
			auto mvs()const->const double6&;
			auto updMvs()->void;
			auto setMve(const double* ve, const char *type = "313")->void;
			auto setMvq(const double* vq)->void;
			auto setMvm(const double* vm)->void;
			auto setMva(const double* va)->void;
			auto setMvs(const double* vs)->void;
			auto getMve(double* ve, const char *type = "313")const->void;
			auto getMvq(double* vq)const->void;
			auto getMvm(double* vm)const->void;
			auto getMva(double* va)const->void;
			auto getMvs(double* vs)const->void;
			auto mas()const->const double6&;
			auto updMas()->void;
			auto setMae(const double* ae, const char *type = "313")->void;
			auto setMaq(const double* aq)->void;
			auto setMam(const double* am)->void;
			auto setMaa(const double* aa)->void;
			auto setMas(const double* as)->void;
			auto getMae(double* ae, const char *type = "313")const->void;
			auto getMaq(double* aq)const->void;
			auto getMam(double* am)const->void;
			auto getMaa(double* aa)const->void;
			auto getMas(double* as)const->void;
			auto mfs() const->const double6&;
			auto setMfs(const double * mfs)->void;

		protected:
			virtual ~GeneralMotion();
			explicit GeneralMotion(Object &father, const aris::core::XmlElement &xml_ele);
			explicit GeneralMotion(const std::string &name, Marker &makI, Marker &makJ, const std::string& freedom = "xyz123", bool active = true);
			GeneralMotion(const GeneralMotion &other);
			GeneralMotion(GeneralMotion &&other);
			GeneralMotion& operator=(const GeneralMotion &other);
			GeneralMotion& operator=(GeneralMotion &&other);

			struct Imp;
			aris::core::ImpPtr<Imp> imp_;

			friend class Model;
			friend class aris::core::Root;
			friend class aris::core::Object;
		};
		class Force :public Interaction
		{
		public:
			static auto Type()->const std::string &{ static const std::string type{ "Force" }; return type; }
			auto virtual type()const->const std::string & override{ return Type(); }
			auto fsI() const->const double* { return fsI_; }
			auto fsJ() const->const double* { return fsJ_; }
			auto virtual updFs()->void = 0;

		protected:
			virtual ~Force() = default;
			Force(const Force &other) = default;
			Force(Force &&other) = default;
			Force& operator=(const Force &other) = default;
			Force& operator=(Force &&other) = default;
			explicit Force(Object &father, const aris::core::XmlElement &xml_ele) :Interaction(father, xml_ele) {}
			explicit Force(const std::string &name, Marker &makI, Marker &makJ, bool active = true):Interaction(name, makI, makJ, active) {}

			double fsI_[6]{ 0 };
			double fsJ_[6]{ 0 };

			friend class Model;
			friend class aris::core::Root;
		};
		class Solver :public Element
		{
		public:
			static auto Type()->const std::string &{ static const std::string type{ "Solver" }; return type; }
			auto virtual type() const->const std::string& override{ return Type(); }
			auto virtual allocateMemory()->void = 0;
			auto virtual kinPos()->void = 0;
			auto virtual kinVel()->void = 0;
			auto virtual dynAccAndFce()->void = 0;
			auto error()const->double;
			auto setError(double error)->void;
			auto maxError()const->double;
			auto setMaxError(double max_error)->void;
			auto iterCount()const->Size;
			auto setIterCount(Size iter_count)->void;
			auto maxIterCount()const->Size;
			auto setMaxIterCount(Size max_count)->void;


		protected:
			virtual ~Solver();
			explicit Solver(Object &father, const aris::core::XmlElement &xml_ele);
			explicit Solver(const std::string &name, Size max_iter_count = 100, double max_error = 1e-10);
			Solver(const Solver&);
			Solver(Solver&&);
			Solver& operator=(const Solver&);
			Solver& operator=(Solver&&);

			struct Imp;
			aris::core::ImpPtr<Imp> imp_;

			friend class Model;
			friend class aris::core::Root;
			friend class aris::core::Object;
		};
		class Calibrator :public Element
		{
		public:
			static auto Type()->const std::string &{ static const std::string type{ "Calibrator" }; return type; }
			auto virtual type() const->const std::string& override{ return Type(); }
			auto virtual allocateMemory()->void = 0;

		protected:
			virtual ~Calibrator();
			explicit Calibrator(Object &father, const aris::core::XmlElement &xml_ele);
			explicit Calibrator(const std::string &name);
			Calibrator(const Calibrator&);
			Calibrator(Calibrator&&);
			Calibrator& operator=(const Calibrator&);
			Calibrator& operator=(Calibrator&&);

			struct Imp;
			aris::core::ImpPtr<Imp> imp_;

			friend class Model;
			friend class aris::core::Root;
			friend class aris::core::Object;
		};
		class SimResult : public Element
		{
		public:
			class TimeResult : public Element
			{
			public:
				static auto Type()->const std::string &{ static const std::string type{ "TimeResult" }; return type; }
				auto virtual type() const->const std::string& override{ return Type(); }
				auto virtual saveXml(aris::core::XmlElement &xml_ele)const->void override;
				auto record()->void;
				auto restore(Size pos)->void;

			private:
				virtual ~TimeResult();
				explicit TimeResult(Object &father, const aris::core::XmlElement &xml_ele);
				explicit TimeResult(const std::string &name);
				TimeResult(const TimeResult&);
				TimeResult(TimeResult&&);
				TimeResult& operator=(const TimeResult&);
				TimeResult& operator=(TimeResult&&);

				struct Imp;
				aris::core::ImpPtr<Imp> imp_;

				friend class SimResult;
				friend class Model;
				friend class aris::core::Root;
				friend class aris::core::Object;

			};
			class PartResult : public Element
			{
			public:
				static auto Type()->const std::string &{ static const std::string type{ "PartResult" }; return type; }
				auto virtual type() const->const std::string& override{ return Type(); }
				auto virtual saveXml(aris::core::XmlElement &xml_ele)const->void override;
				auto part()->Part&;
				auto part()const->const Part&{ return const_cast<PartResult*>(this)->part(); }
				auto record()->void;
				auto restore(Size pos)->void;

			private:
				virtual ~PartResult();
				explicit PartResult(Object &father, const aris::core::XmlElement &xml_ele);
				explicit PartResult(const std::string &name, Part &part);
				PartResult(const PartResult&);
				PartResult(PartResult&&);
				PartResult& operator=(const PartResult&);
				PartResult& operator=(PartResult&&);

				struct Imp;
				aris::core::ImpPtr<Imp> imp_;

				friend class SimResult;
				friend class Model;
				friend class aris::core::Root;
				friend class aris::core::Object;

			};
			class ConstraintResult : public Element
			{
			public:
				static auto Type()->const std::string &{ static const std::string type{ "ConstraintResult" }; return type; }
				auto virtual type() const->const std::string& override{ return Type(); }
				auto virtual saveXml(aris::core::XmlElement &xml_ele)const->void override;
				auto constraint()->Constraint&;
				auto constraint()const->const Constraint&{ return const_cast<ConstraintResult*>(this)->constraint(); }
				auto record()->void;
				auto restore(Size pos)->void;

			private:
				virtual ~ConstraintResult();
				explicit ConstraintResult(Object &father, const aris::core::XmlElement &xml_ele);
				explicit ConstraintResult(const std::string &name, Constraint &constraint);
				ConstraintResult(const ConstraintResult&);
				ConstraintResult(ConstraintResult&&);
				ConstraintResult& operator=(const ConstraintResult&);
				ConstraintResult& operator=(ConstraintResult&&);

				struct Imp;
				aris::core::ImpPtr<Imp> imp_;

				friend class SimResult;
				friend class Model;
				friend class aris::core::Root;
				friend class aris::core::Object;
			};
			
			static auto Type()->const std::string &{ static const std::string type{ "SimResult" }; return type; }
			auto virtual type() const->const std::string& override{ return Type(); }
			auto timeResult()->TimeResult&;
			auto timeResult()const->const TimeResult&{ return const_cast<SimResult*>(this)->timeResult(); }
			auto partResultPool()->aris::core::ObjectPool<PartResult, Element>&;
			auto partResultPool()const->const aris::core::ObjectPool<PartResult, Element>&{return const_cast<SimResult*>(this)->partResultPool(); };
			auto constraintResultPool()->aris::core::ObjectPool<ConstraintResult, Element>&;
			auto constraintResultPool()const->const aris::core::ObjectPool<ConstraintResult, Element>&{return const_cast<SimResult*>(this)->constraintResultPool(); };
			
			auto allocateMemory()->void;
			auto record()->void;
			auto restore(Size pos)->void;
			auto size()const->Size;
			auto clear()->void;

		protected:
			virtual ~SimResult();
			explicit SimResult(Object &father, const aris::core::XmlElement &xml_ele);
			explicit SimResult(const std::string &name);
			SimResult(const SimResult&);
			SimResult(SimResult&&);
			SimResult& operator=(const SimResult&);
			SimResult& operator=(SimResult&&);

			struct Imp;
			aris::core::ImpPtr<Imp> imp_;

			friend class Model;
			friend class aris::core::Root;
			friend class aris::core::Object;
		};
		class Simulator :public Element
		{
		public:
			static auto Type()->const std::string &{ static const std::string type{ "Simulator" }; return type; }
			auto virtual type() const->const std::string& override{ return Type(); }
			auto virtual simulate(const PlanFunction &plan, void *param, std::int32_t param_size, SimResult &result)->void;
			template<typename ParamType>
			auto simulate(const PlanFunction &plan, ParamType type, SimResult &result)->void 
			{
				static_assert(std::is_trivial<ParamType>::value, "ParamType must be trivial type");
				simulate(plan, &type, std::int32_t(sizeof(type)), result);
			}

		protected:
			virtual ~Simulator();
			explicit Simulator(Object &father, const aris::core::XmlElement &xml_ele);
			explicit Simulator(const std::string &name);
			Simulator(const Simulator&);
			Simulator(Simulator&&);
			Simulator& operator=(const Simulator&);
			Simulator& operator=(Simulator&&);

			struct Imp;
			aris::core::ImpPtr<Imp> imp_;

			friend class Model;
			friend class aris::core::Root;
			friend class aris::core::Object;
		};

		class Model :public aris::core::Root
		{
		public:
			static auto Type()->const std::string &{ static const std::string type("Model"); return std::ref(type); }
			auto virtual type() const->const std::string&{ return Type(); }
			using Root::loadXml;
			using Root::saveXml;
			auto virtual loadXml(const aris::core::XmlDocument &xml_doc)->void override;
            auto virtual loadXml(const aris::core::XmlElement &xml_ele)->void override;
			auto virtual saveXml(aris::core::XmlDocument &xml_doc)const->void override;
			auto virtual saveXml(aris::core::XmlElement &xml_ele)const->void override;
			auto virtual loadDynEle(const std::string &name)->void;
			auto virtual saveDynEle(const std::string &name)->void;
			auto time()const->double;
			auto setTime(double time)->void;
			auto calculator()->aris::core::Calculator&;
			auto calculator()const ->const aris::core::Calculator&{ return const_cast<std::decay_t<decltype(*this)> *>(this)->calculator(); }
			auto environment()->aris::dynamic::Environment&;
			auto environment()const ->const aris::dynamic::Environment&{ return const_cast<std::decay_t<decltype(*this)> *>(this)->environment(); }
			auto variablePool()->aris::core::ObjectPool<Variable, Element>&;
			auto variablePool()const->const aris::core::ObjectPool<Variable, Element>&{ return const_cast<std::decay_t<decltype(*this)> *>(this)->variablePool(); }
			auto partPool()->aris::core::ObjectPool<Part, Element>&;
			auto partPool()const->const aris::core::ObjectPool<Part, Element>&{ return const_cast<std::decay_t<decltype(*this)> *>(this)->partPool(); }
			auto jointPool()->aris::core::ObjectPool<Joint, Element>&;
			auto jointPool()const->const aris::core::ObjectPool<Joint, Element>&{ return const_cast<std::decay_t<decltype(*this)> *>(this)->jointPool(); }
			auto motionPool()->aris::core::ObjectPool<Motion, Element>&;
			auto motionPool()const->const aris::core::ObjectPool<Motion, Element>&{ return const_cast<std::decay_t<decltype(*this)> *>(this)->motionPool(); }
			auto generalMotionPool()->aris::core::ObjectPool<GeneralMotion, Element>&;
			auto generalMotionPool()const->const aris::core::ObjectPool<GeneralMotion, Element>&{ return const_cast<std::decay_t<decltype(*this)> *>(this)->generalMotionPool(); }
			auto forcePool()->aris::core::ObjectPool<Force, Element>&;
			auto forcePool()const->const aris::core::ObjectPool<Force, Element>&{ return const_cast<std::decay_t<decltype(*this)> *>(this)->forcePool(); }
			auto solverPool()->aris::core::ObjectPool<Solver, Element>&;
			auto solverPool()const->const aris::core::ObjectPool<Solver, Element>&{ return const_cast<std::decay_t<decltype(*this)> *>(this)->solverPool(); }
			auto simulatorPool()->aris::core::ObjectPool<Simulator, Element>&;
			auto simulatorPool()const->const aris::core::ObjectPool<Simulator, Element>&{ return const_cast<std::decay_t<decltype(*this)> *>(this)->simulatorPool(); }
			auto simResultPool()->aris::core::ObjectPool<SimResult, Element>&;
			auto simResultPool()const->const aris::core::ObjectPool<SimResult, Element>&{ return const_cast<std::decay_t<decltype(*this)> *>(this)->simResultPool(); }
			auto markerSize()const->Size { Size size{ 0 }; for (auto &prt : partPool())size += prt.markerPool().size(); return size; }
			auto ground()->Part&;
			auto ground()const->const Part&{ return const_cast<std::decay_t<decltype(*this)> *>(this)->ground(); }
			
			auto addPartByPm(const double*pm, const double *prt_im = nullptr)->Part&;
			auto addPartByPe(const double*pe, const char* eul_type, const double *prt_im = nullptr)->Part&;
			auto addPartByPq(const double*pq, const double *prt_im = nullptr)->Part&;
			auto addRevoluteJoint(Part &first_part, Part &second_part, const double *position, const double *axis)->RevoluteJoint&;
			auto addPrismaticJoint(Part &first_part, Part &second_part, const double *position, const double *axis)->PrismaticJoint&;
			auto addUniversalJoint(Part &first_part, Part &second_part, const double *position, const double *first_axis, const double *second_axis)->UniversalJoint&;
			auto addSphericalJoint(Part &first_part, Part &second_part, const double *position)->SphericalJoint&;
			auto addMotion(Joint &joint)->Motion&;
			auto addGeneralMotion(Part &end_effector, Coordinate &reference, const double* pm)->GeneralMotion&;
			
			
			
			
			
			
			
			
			
			
			
			auto updMotionID()->void;
			auto motionAtAbs(Size abs_id)->Motion&;
			auto motionAtAbs(Size abs_id)const->const Motion&;
			auto motionAtPhy(Size phy_id)->Motion&;
			auto motionAtPhy(Size phy_id)const->const Motion&;
			auto motionAtSla(Size sla_id)->Motion&;
			auto motionAtSla(Size sla_id)const->const Motion&;





			// 动力学计算以下变量的关系
			// I  ： 惯量矩阵,m*m
			// C  ： 约束矩阵,m*n
			// pa ： 杆件的螺旋加速度 m*1
			// pf ： 杆件的螺旋外力（不包括惯性力）m*1
			// ca ： 约束的加速度（不是螺旋）n*1
			// cf ： 约束力n*1
			// 动力学主要求解以下方程：
			// [ -I  C  ]  *  [ pa ]  = [ pf ]
			// [  C' O  ]     [ cf ]    [ ca ]
			//
			// A = [-I  C ]
			//     [ C' O ]
			//
			// x = [ pa ]
			//     [ cf ]
			//
			// b = [ pf ]
			//     [ ca ]
			// 
			// 约束矩阵C为m x n维的矩阵,惯量矩阵为m x m维的矩阵
			// 约束力为n维的向量,约束加速度为n维向量
			// 部件力为m维的向量,部件加速度为m维向量
			// 动力学为所求的未知量为部件加速度和约束力,其他均为已知
			// 动力学主要求解以下方程：
			// [  I  C  ]  *  [ as ]  = [ fs ]
			// [  C' O  ]     [ cf ]    [ ca ]
			//
			// A = [ I  C ]
			//     [ C' O ]
			//
			// x = [ as ]
			//     [ cf ]
			//
			// b = [ fs ]
			//     [ ca ]
			virtual ~Model();
			Model(const std::string &name = "model");
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
			VariableTemplate(const VariableTemplate &other) = default;
			VariableTemplate(VariableTemplate &&other) = default;
			VariableTemplate& operator=(const VariableTemplate &other) = default;
			VariableTemplate& operator=(VariableTemplate &&other) = default;

			VariableType data_;
			friend class Model;
		};
		class MatrixVariable final : public VariableTemplate<aris::core::Matrix>
		{
		public:
			static auto Type()->const std::string &{ static const std::string type{ "MatrixVariable" }; return type; }
			auto virtual type() const->const std::string& override{ return Type(); }
			auto virtual toString() const->std::string override { return data_.toString(); }

		private:
			virtual ~MatrixVariable() = default;
			explicit MatrixVariable(const std::string &name, const aris::core::Matrix &data): VariableTemplate(name, data) {}
			explicit MatrixVariable(Object &father, const aris::core::XmlElement &xml_ele): VariableTemplate(father, xml_ele)
			{
				data_ = model().calculator().calculateExpression(xml_ele.GetText());
				model().calculator().addVariable(name(), data_);
			}
			MatrixVariable(const MatrixVariable &other) = default;
			MatrixVariable(MatrixVariable &&other) = default;
			MatrixVariable& operator=(const MatrixVariable &other) = default;
			MatrixVariable& operator=(MatrixVariable &&other) = default;

			friend class Model;
			friend class aris::core::Root;
			friend class aris::core::Object;
		};
		class StringVariable final : public VariableTemplate<std::string>
		{
		public:
			static auto Type()->const std::string &{ static const std::string type{ "StringVariable" }; return type; }
			auto virtual type() const->const std::string& override{ return Type(); }
			auto virtual toString() const->std::string override { return data_; }

		private:
			virtual ~StringVariable() = default;
			explicit StringVariable(const std::string &name, const std::string &data): VariableTemplate(name, data) {}
			explicit StringVariable(Object &father, const aris::core::XmlElement &xml_ele): VariableTemplate(father, xml_ele)
			{
				data_ = std::string(xml_ele.GetText());
				model().calculator().addVariable(name(), data_);
			}
			StringVariable(const StringVariable &other) = default;
			StringVariable(StringVariable &&other) = default;
			StringVariable& operator=(const StringVariable &other) = default;
			StringVariable& operator=(StringVariable &&other) = default;

			friend class Model;
			friend class aris::core::Root;
			friend class aris::core::Object;
		};
		class ParasolidGeometry final :public Geometry
		{
		public:
			static auto Type()->const std::string &{ static const std::string type{ "ParasolidGeometry" }; return type; }
			auto virtual type() const->const std::string& override{ return Type(); }
			auto virtual saveXml(aris::core::XmlElement &xml_ele) const->void override;
			auto prtPm()const->const double4x4&;
			auto filePath()const->const std::string &;

		private:
			virtual ~ParasolidGeometry();
			ParasolidGeometry(const ParasolidGeometry &other);
			ParasolidGeometry(ParasolidGeometry &&other);
			ParasolidGeometry& operator=(const ParasolidGeometry &other);
			ParasolidGeometry& operator=(ParasolidGeometry &&other);
			explicit ParasolidGeometry(Object &father, const aris::core::XmlElement &xml_ele);
			explicit ParasolidGeometry(const std::string &name, const std::string &graphic_file_path, const double* prt_pm = nullptr);
			
			struct Imp;
			aris::core::ImpPtr<Imp> imp_;

			friend class Model;
			friend class aris::core::Root;
			friend class aris::core::Object;
		};
		class FloatMarker final :public Marker
		{
		public:
			void setPrtPm(const double *prt_pm) { std::copy_n(prt_pm, 16, const_cast<double *>(*this->prtPm())); }
			void setPrtPe(const double *prt_pe, const char *type = "313") { s_pe2pm(prt_pe, const_cast<double *>(*prtPm()), type); }
			void setPrtPq(const double *prt_pq) { s_pq2pm(prt_pq, const_cast<double *>(*prtPm())); }

			explicit FloatMarker(Part &prt, const double *prt_pe = nullptr, const char* eu_type = "313") :Marker("float_marker")
			{
				static const double default_prt_pe[6]{ 0,0,0,0,0,0 };
				prt_pe = prt_pe ? prt_pe : default_prt_pe;
				setPrtPe(prt_pe, eu_type);
			}
			FloatMarker(const FloatMarker &other) = default;
			FloatMarker(FloatMarker &&other) = default;
			FloatMarker& operator=(const FloatMarker &other) = default;
			FloatMarker& operator=(FloatMarker &&other) = default;
		};
		class SingleComponentForce final :public Force
		{
		public:
			static const std::string& Type() { static const std::string type("SingleComponentForce"); return type; }
			auto virtual type() const->const std::string& override{ return Type(); }
			auto virtual saveXml(aris::core::XmlElement &xml_ele) const->void override;
			auto virtual updFs()->void override;
			auto setComponentID(Size id)->void { component_axis_ = id; }
			auto setFce(double value)->void { std::fill_n(fce_value_, 6, 0); fce_value_[component_axis_] = value; }
			auto setFce(double value, Size componentID)->void { this->component_axis_ = componentID; setFce(value); }
			auto fce()const->double { return fce_value_[component_axis_]; }

		private:
			virtual ~SingleComponentForce() = default;
			explicit SingleComponentForce(const std::string &name, Marker& makI, Marker& makJ, Size componentID);
			explicit SingleComponentForce(Object &father, const aris::core::XmlElement &xml_ele);
			SingleComponentForce(const SingleComponentForce &other) = default;
			SingleComponentForce(SingleComponentForce &&other) = default;
			SingleComponentForce& operator=(const SingleComponentForce &other) = default;
			SingleComponentForce& operator=(SingleComponentForce &&other) = default;

			Size component_axis_;
			double fce_value_[6]{ 0 };

			friend class Model;
			friend class aris::core::Root;
			friend class aris::core::Object;
		};
		class SolverSimulator : public Simulator
		{
		public:
			static auto Type()->const std::string &{ static const std::string type{ "SolverSimulator" }; return type; }
			auto virtual type() const->const std::string& override{ return Type(); }
			auto virtual saveXml(aris::core::XmlElement &xml_ele) const->void override;
			auto virtual simulate(const PlanFunction &plan, void *param, std::int32_t param_size, SimResult &result)->void;
			using Simulator::simulate;
			auto solver()->Solver&;
			auto solver()const ->const Solver&{ return const_cast<SolverSimulator*>(this)->solver(); };


		protected:
			virtual ~SolverSimulator();
			explicit SolverSimulator(Object &father, const aris::core::XmlElement &xml_ele);
			explicit SolverSimulator(const std::string &name, Solver &solver);
			SolverSimulator(const SolverSimulator&);
			SolverSimulator(SolverSimulator&&);
			SolverSimulator& operator=(const SolverSimulator&);
			SolverSimulator& operator=(SolverSimulator&&);

			struct Imp;
			aris::core::ImpPtr<Imp> imp_;

			friend class Model;
			friend class aris::core::Root;
			friend class aris::core::Object;
		};
		class AdamsSimulator :public SolverSimulator
		{
		public:
			static auto Type()->const std::string &{ static const std::string type{ "AdamsSimulator" }; return type; }
			auto virtual type() const->const std::string& override{ return Type(); }
			auto saveAdams(const std::string &filename, SimResult &result, Size pos = -1)->void;
			auto saveAdams(std::ofstream &file, SimResult &result, Size pos = -1)->void;
			auto adamsID(const Marker &mak)const->Size;
			auto adamsID(const Part &prt)const->Size{ return (&prt == &model().ground()) ? 1 : prt.id() + (model().ground().id() < prt.id() ? 1 : 2); }
			auto adamsID(const Element &ele)const->Size { return ele.id() + 1; };
		
		protected:
			virtual ~AdamsSimulator();
			explicit AdamsSimulator(Object &father, const aris::core::XmlElement &xml_ele);
			explicit AdamsSimulator(const std::string &name, Solver &solver);
			AdamsSimulator(const AdamsSimulator&);
			AdamsSimulator(AdamsSimulator&&);
			AdamsSimulator& operator=(const AdamsSimulator&);
			AdamsSimulator& operator=(AdamsSimulator&&);

			struct Imp;
			aris::core::ImpPtr<Imp> imp_;

			friend class Model;
			friend class aris::core::Root;
			friend class aris::core::Object;
		};
	}
}

#endif
