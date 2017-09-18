#ifndef ARIS_DYNAMIC_MODEL_COORDINATE_
#define ARIS_DYNAMIC_MODEL_COORDINATE_

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
#include <aris_dynamic_model_basic.h>

namespace aris
{
	namespace dynamic
	{
		class Part;
		class Geometry;
		
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
			auto virtual pm() const->const double4x4&{ return glbPm(); }
			auto virtual vs() const->const double6&{ return glbVs(); }
			auto virtual as() const->const double6&{ return glbAs(); }
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
			explicit Coordinate(const std::string &name = "coordinate", bool active = true);
			Coordinate(const Coordinate &) = default;
			Coordinate(Coordinate &&) = default;
			Coordinate& operator=(const Coordinate &) = default;
			Coordinate& operator=(Coordinate &&) = default;
		};
		class Marker :public Coordinate
		{
		public:
			static auto Type()->const std::string &{ static const std::string type{ "Marker" }; return type; }
			auto virtual type() const->const std::string& override{ return Type(); }
			auto virtual saveXml(aris::core::XmlElement &xml_ele) const->void override;
			auto virtual loadXml(const aris::core::XmlElement &xml_ele)->void override;
			auto virtual glbPm()const->const double4x4& override;
			auto virtual glbVs()const->const double6& override;
			auto virtual glbAs()const->const double6& override;
			auto virtual prtPm()const->const double4x4& override;
			auto virtual prtVs()const->const double6& override;
			auto virtual prtAs()const->const double6& override;
			void setPrtPm(const double *prt_pm) { std::copy_n(prt_pm, 16, const_cast<double *>(*this->prtPm())); }
			void setPrtPe(const double *prt_pe, const char *type = "313") { s_pe2pm(prt_pe, const_cast<double *>(*prtPm()), type); }
			void setPrtPq(const double *prt_pq) { s_pq2pm(prt_pq, const_cast<double *>(*prtPm())); }
			auto fatherPart() const->const Part&;
			auto fatherPart()->Part&;

		protected:
			virtual ~Marker();
			explicit Marker(const std::string &name = "marker", const double *prt_pm = nullptr, bool active = true);
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
		class Part final :public Coordinate
		{
		public:
			auto static Type()->const std::string &{ static const std::string type{ "Part" }; return type; }
			auto virtual type() const->const std::string& override{ return Type(); }
			auto virtual saveXml(aris::core::XmlElement &xml_ele) const->void override;
			auto virtual loadXml(const aris::core::XmlElement &xml_ele)->void override;
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
			explicit Part(const std::string &name = "part", const double *prt_im = nullptr, const double *pm = nullptr, const double *vs = nullptr, const double *as = nullptr, bool active = true);
			Part(const Part &other);
			Part(Part &&other);
			Part& operator=(const Part &other);
			Part& operator=(Part &&other);


		private:
			struct Imp;
			aris::core::ImpPtr<Imp> imp_;

			friend class Model;
			friend class aris::core::Root;
			friend class aris::core::Object;
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
			explicit Geometry(const std::string &name = "geometry") : Element(name) {}
			Geometry(const Geometry&) = default;
			Geometry(Geometry&&) = default;
			Geometry& operator=(const Geometry&) = default;
			Geometry& operator=(Geometry&&) = default;

			friend class Model;
			friend class aris::core::Root;
		};

		class ParasolidGeometry final :public Geometry
		{
		public:
			static auto Type()->const std::string &{ static const std::string type{ "ParasolidGeometry" }; return type; }
			auto virtual type() const->const std::string& override{ return Type(); }
			auto virtual saveXml(aris::core::XmlElement &xml_ele) const->void override;
			auto virtual loadXml(const aris::core::XmlElement &xml_ele)->void override;
			auto prtPm()const->const double4x4&;
			auto filePath()const->const std::string &;

		private:
			virtual ~ParasolidGeometry();
			explicit ParasolidGeometry(const std::string &name = "parasolid_geometry", const std::string &graphic_file_path = "", const double* prt_pm = nullptr);
			ParasolidGeometry(const ParasolidGeometry &other);
			ParasolidGeometry(ParasolidGeometry &&other);
			ParasolidGeometry& operator=(const ParasolidGeometry &other);
			ParasolidGeometry& operator=(ParasolidGeometry &&other);

			struct Imp;
			aris::core::ImpPtr<Imp> imp_;

			friend class Model;
			friend class aris::core::Root;
			friend class aris::core::Object;
		};
	}
}

#endif
