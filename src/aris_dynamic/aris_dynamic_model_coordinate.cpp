#include <cmath>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <algorithm>
#include <limits>
#include <sstream>
#include <regex>
#include <limits>
#include <type_traits>
#include <ios>

#include "aris_core.h"
#include "aris_dynamic_matrix.h"
#include "aris_dynamic_screw.h"
#include "aris_dynamic_model.h"

namespace aris
{
	namespace dynamic
	{
		auto Coordinate::getPp(double *pp)const->void { if (pp)s_pm2pp(*pm(), pp); }
		auto Coordinate::getPp(const Coordinate &relative_to, double *pp)const->void
		{
			if (pp)
			{
				double pm[4][4];
				getPm(relative_to, *pm);
				s_pm2pp(*pm, pp);
			}
		}
		auto Coordinate::getRe(double *re, const char *type)const->void { if (re)s_pm2re(*pm(), re, type); }
		auto Coordinate::getRe(const Coordinate &relative_to, double *re, const char *type)const->void
		{
			if (re)
			{
				double pm[4][4];
				getPm(relative_to, *pm);
				s_pm2re(*pm, re, type);
			}
		}
		auto Coordinate::getRq(double *rq)const->void { if (rq)s_pm2rq(*pm(), rq); }
		auto Coordinate::getRq(const Coordinate &relative_to, double *rq)const->void
		{
			if (rq)
			{
				double pm[4][4];
				getPm(relative_to, *pm);
				s_pm2rq(*pm, rq);
			}
		}
		auto Coordinate::getRm(double *rm, Size rm_ld)const->void { if (rm)s_pm2rm(*pm(), rm, rm_ld); }
		auto Coordinate::getRm(const Coordinate &relative_to, double *rm, Size rm_ld)const->void
		{
			if (rm)
			{
				double pm[4][4];
				getPm(relative_to, *pm);
				s_pm2rm(*pm, rm, rm_ld);
			}
		}
		auto Coordinate::getPe(double *pe, const char *type)const->void { if (pe)s_pm2pe(*pm(), pe, type); }
		auto Coordinate::getPe(const Coordinate &relative_to, double *pe, const char *type)const->void
		{
			if (pe)
			{
				double pm[4][4];
				getPm(relative_to, *pm);
				s_pm2pe(*pm, pe, type);
			}
		}
		auto Coordinate::getPq(double *pq)const->void { if (pq)s_pm2pq(*pm(), pq); }
		auto Coordinate::getPq(const Coordinate &relative_to, double *pq)const->void
		{
			if (pq)
			{
				double pm[4][4];
				getPm(relative_to, *pm);
				s_pm2pq(*pm, pq);
			}
		}
		auto Coordinate::getPm(double *pm)const->void { if (pm)s_vc(16, *glbPm(), pm); }
		auto Coordinate::getPm(const Coordinate &relative_to, double *pm)const->void { if (pm)s_inv_pm2pm(*relative_to.pm(), *this->pm(), pm); }
		auto Coordinate::getVp(double *vp, double *pp)const->void
		{
			if (vp)
			{
				double pp_default[3];
				pp = pp ? pp : pp_default;
				getPp(pp);
				s_vs2vp(vs(), pp, vp);
			}
			else
			{
				getPp(pp);
			}
		}
		auto Coordinate::getVp(const Coordinate &relative_to, double *vp, double *pp)const->void
		{
			if (vp)
			{
				double vs[6], pp_default[3];
				pp = pp ? pp : pp_default;
				getPp(relative_to, pp);
				getVs(relative_to, vs);
				s_vs2vp(vs, pp, vp);
			}
			else
			{
				getPp(relative_to, pp);
			}
		}
		auto Coordinate::getWe(double *we, double *re, const char *type)const->void
		{
			if (we)
			{
				double re_default[3];
				re = re ? re : re_default;
				getRe(re, type);
				s_vs2we(vs(), re, we, type);
			}
			else
			{
				getRe(re, type);
			}
		}
		auto Coordinate::getWe(const Coordinate &relative_to, double *we, double *re, const char *type)const->void
		{
			if (we)
			{
				double vs[6], re_default[3];
				re = re ? re : re_default;
				getRe(relative_to, re, type);
				getVs(relative_to, vs);
				s_vs2we(vs, re, we, type);
			}
			else
			{
				getRe(relative_to, re);
			}
		}
		auto Coordinate::getWq(double *wq, double *rq)const->void
		{
			if (wq)
			{
				double rq_default[4];
				rq = rq ? rq : rq_default;
				getRq(rq);
				s_vs2wq(vs(), rq, wq);
			}
			else
			{
				getRq(rq);
			}
		}
		auto Coordinate::getWq(const Coordinate &relative_to, double *wq, double *rq)const->void
		{
			if (wq)
			{
				double vs[6], rq_default[4];
				rq = rq ? rq : rq_default;
				getRq(relative_to, rq);
				getVs(relative_to, vs);
				s_vs2wq(vs, rq, wq);
			}
			else
			{
				getRq(relative_to, rq);
			}
		}
		auto Coordinate::getWm(double *wm, double *rm, Size wm_ld, Size rm_ld)const->void
		{
			if (wm)
			{
				double rm_default[9];
				rm = rm ? rm : rm_default;
				getRm(rm, rm_ld);
				s_vs2wm(vs(), rm, wm, rm_ld, wm_ld);
			}
			else
			{
				getRm(rm, rm_ld);
			}
		}
		auto Coordinate::getWm(const Coordinate &relative_to, double *wm, double *rm, Size wm_ld, Size rm_ld)const->void
		{
			if (wm)
			{
				double vs[6], rm_default[9];
				rm = rm ? rm : rm_default;
				getRm(relative_to, rm, rm_ld);
				getVs(relative_to, vs);
				s_vs2wm(vs, rm, wm, rm_ld, wm_ld);
			}
			else
			{
				getRm(relative_to, rm, rm_ld);
			}
		}
		auto Coordinate::getWa(double *wa, double *rm, Size rm_ld)const->void
		{
			if (wa) { s_vs2wa(vs(), wa); }
			getRm(rm, rm_ld);
		}
		auto Coordinate::getWa(const Coordinate &relative_to, double *wa, double *rm, Size rm_ld)const->void
		{
			if (wa)
			{
				double vs[6], pm[16];
				getVs(relative_to, vs, pm);
				s_vs2wa(vs, wa);
				s_pm2rm(pm, rm, rm_ld);
			}
			else
			{
				getRm(rm, rm_ld);
			}

		}
		auto Coordinate::getVe(double *ve, double *pe, const char *type)const->void
		{
			if (ve)
			{
				double pe_default[6];
				pe = pe ? pe : pe_default;
				getPe(pe, type);
				s_vs2ve(vs(), pe, ve, type);
			}
			else
			{
				getPe(pe, type);
			}
		}
		auto Coordinate::getVe(const Coordinate &relative_to, double *ve, double *pe, const char *type)const->void
		{
			if (ve)
			{
				double vs[6], pe_default[6];
				pe = pe ? pe : pe_default;
				getPe(relative_to, pe, type);
				getVs(relative_to, vs);
				s_vs2ve(vs, pe, ve, type);
			}
			else
			{
				getPe(relative_to, pe, type);
			}
		}
		auto Coordinate::getVq(double *vq, double *pq)const->void
		{
			if (vq)
			{
				double pq_default[7];
				pq = pq ? pq : pq_default;
				getPq(pq);
				s_vs2vq(vs(), pq, vq);
			}
			else
			{
				getPq(pq);
			}
		}
		auto Coordinate::getVq(const Coordinate &relative_to, double *vq, double *pq)const->void
		{
			if (vq)
			{
				double vs[6], pq_default[7];
				pq = pq ? pq : pq_default;
				getPq(relative_to, pq);
				getVs(relative_to, vs);
				s_vs2vq(vs, pq, vq);
			}
			else
			{
				getPq(relative_to, pq);
			}
		}
		auto Coordinate::getVm(double *vm, double *pm)const->void
		{
			if (vm)
			{
				double pm_default[16];
				pm = pm ? pm : pm_default;
				getPm(pm);
				s_vs2vm(vs(), pm, vm);
			}
			else
			{
				getPm(pm);
			}
		}
		auto Coordinate::getVm(const Coordinate &relative_to, double *vm, double *pm)const->void
		{
			if (vm)
			{
				double vs[6], pm_default[16];
				pm = pm ? pm : pm_default;
				getPm(relative_to, pm);
				getVs(relative_to, vs);
				s_vs2vm(vs, pm, vm);
			}
			else
			{
				getPm(relative_to, pm);
			}
		}
		auto Coordinate::getVa(double *va, double *pp)const->void
		{
			if (va)
			{
				double pp_default[3];
				pp = pp ? pp : pp_default;
				getPp(pp);
				s_vs2va(vs(), pp, va);
			}
			else
			{
				getPp(pp);
			}
		}
		auto Coordinate::getVa(const Coordinate &relative_to, double *va, double *pp)const->void
		{
			if (va)
			{
				double vs[6], pp_default[3];
				pp = pp ? pp : pp_default;
				getPp(relative_to, pp);
				getVs(relative_to, vs);
				s_vs2va(vs, pp, va);
			}
			else
			{
				getPp(relative_to, pp);
			}
		}
		auto Coordinate::getVs(double *vs, double *pm)const->void
		{
			if (vs)std::copy(&this->vs()[0], &this->vs()[6], vs);
			getPm(pm);
		}
		auto Coordinate::getVs(const Coordinate &relative_to, double *vs, double *pm)const->void
		{
			if (vs)s_inv_vs2vs(*relative_to.pm(), relative_to.vs(), this->vs(), vs);
			getPm(relative_to, pm);
		}
		auto Coordinate::getAp(double *ap, double *vp, double *pp)const->void
		{
			if (ap)
			{
				double pp_default[3], vp_default[3];
				pp = pp ? pp : pp_default;
				vp = vp ? vp : vp_default;
				getPp(pp);
				s_as2ap(vs(), as(), pp, ap, vp);
			}
			else
			{
				getVp(vp, pp);
			}
		}
		auto Coordinate::getAp(const Coordinate &relative_to, double *ap, double *vp, double *pp)const->void
		{
			if (ap)
			{
				double vs[6], as[6], pp_default[3];
				pp = pp ? pp : pp_default;
				getPp(relative_to, pp);
				getAs(relative_to, as, vs);
				s_as2ap(vs, as, pp, ap, vp);
			}
			else
			{
				getVp(relative_to, vp, pp);
			}
		}
		auto Coordinate::getXe(double *xe, double *we, double *re, const char *type)const->void
		{
			if (xe)
			{
				double re_default[3], we_default[3];
				re = re ? re : re_default;
				we = we ? we : we_default;
				getRe(re, type);
				s_as2xe(vs(), as(), re, xe, we, type);
			}
			else
			{
				getVe(we, re, type);
			}
		}
		auto Coordinate::getXe(const Coordinate &relative_to, double *xe, double *we, double *re, const char *type)const->void
		{
			if (xe)
			{
				double vs[6], as[6], re_default[3];
				re = re ? re : re_default;
				getRe(relative_to, re, type);
				getAs(relative_to, as, vs);
				s_as2xe(vs, as, re, xe, we, type);
			}
			else
			{
				getVe(relative_to, we, re, type);
			}
		}
		auto Coordinate::getXq(double *xq, double *wq, double *rq)const->void
		{
			if (xq)
			{
				double rq_default[4], wq_default[4];
				rq = rq ? rq : rq_default;
				wq = wq ? wq : wq_default;
				getRq(rq);
				s_as2xq(vs(), as(), rq, xq, wq);
			}
			else
			{
				getWq(wq, rq);
			}
		}
		auto Coordinate::getXq(const Coordinate &relative_to, double *xq, double *wq, double *rq)const->void
		{
			if (xq)
			{
				double vs[6], as[6], rq_default[4];
				rq = rq ? rq : rq_default;
				getRq(relative_to, rq);
				getAs(relative_to, as, vs);
				s_as2xq(vs, as, rq, xq, wq);
			}
			else
			{
				getWq(relative_to, wq, rq);
			}
		}
		auto Coordinate::getXm(double *xm, double *wm, double *rm, Size xm_ld, Size wm_ld, Size rm_ld)const->void
		{
			if (xm)
			{
				double rm_default[9], wm_default[9];
				rm = rm ? rm : rm_default;
				wm = wm ? wm : wm_default;
				getRm(rm, rm_ld);
				s_as2xm(vs(), as(), rm, xm, wm, rm_ld, wm_ld, xm_ld);
			}
			else
			{
				getWm(wm, rm, wm_ld, rm_ld);
			}
		}
		auto Coordinate::getXm(const Coordinate &relative_to, double *xm, double *wm, double *rm, Size xm_ld, Size wm_ld, Size rm_ld)const->void
		{
			if (xm)
			{
				double vs[6], as[6], rm_default[9];
				rm = rm ? rm : rm_default;
				getRm(relative_to, rm, rm_ld);
				getAs(relative_to, as, vs);
				s_as2xm(vs, as, rm, xm, wm, rm_ld, wm_ld, xm_ld);
			}
			else
			{
				getWm(relative_to, wm, rm, wm_ld, rm_ld);
			}
		}
		auto Coordinate::getXa(double *xa, double *wa, double *rm, Size rm_ld)const->void
		{
			if (xa)s_as2xa(as(), xa);
			getWa(wa, rm, rm_ld);
		}
		auto Coordinate::getXa(const Coordinate &relative_to, double *xa, double *wa, double *rm, Size rm_ld)const->void
		{
			if (xa)
			{
				double vs[6], as[6], pm[16];
				getAs(relative_to, as, vs, pm);
				s_as2xa(as, xa);
				s_vs2wa(vs, wa);
				s_pm2rm(pm, rm, rm_ld);
			}
			else
			{
				getWa(relative_to, wa, rm, rm_ld);
			}

		}
		auto Coordinate::getAe(double *ae, double *ve, double *pe, const char *type)const->void
		{
			if (ae)
			{
				double pe_default[6], ve_default[6];
				pe = pe ? pe : pe_default;
				ve = ve ? ve : ve_default;
				getPe(pe, type);
				s_as2ae(vs(), as(), pe, ae, ve, type);
			}
			else
			{
				getVe(ve, pe, type);
			}
		}
		auto Coordinate::getAe(const Coordinate &relative_to, double *ae, double *ve, double *pe, const char *type)const->void
		{
			if (ae)
			{
				double vs[6], as[6], pe_default[6], ve_default[6];
				pe = pe ? pe : pe_default;
				ve = ve ? ve : ve_default;
				getPe(relative_to, pe, type);
				getAs(relative_to, as, vs);
				s_as2ae(vs, as, pe, ae, ve, type);
			}
			else
			{
				getVe(relative_to, ve, pe, type);
			}
		}
		auto Coordinate::getAq(double *aq, double *vq, double *pq)const->void
		{
			if (aq)
			{
				double pq_default[7], vq_default[7];
				pq = pq ? pq : pq_default;
				vq = vq ? vq : vq_default;
				getPq(pq);
				s_as2aq(vs(), as(), pq, aq, vq);
			}
			else
			{
				getVq(vq, pq);
			}
		}
		auto Coordinate::getAq(const Coordinate &relative_to, double *aq, double *vq, double *pq)const->void
		{
			if (aq)
			{
				double vs[6], as[6], pq_default[7], vq_default[7];
				pq = pq ? pq : pq_default;
				vq = vq ? vq : vq_default;
				getPq(relative_to, pq);
				getAs(relative_to, as, vs);
				s_as2aq(vs, as, pq, aq, vq);
			}
			else
			{
				getVq(relative_to, vq, pq);
			}
		}
		auto Coordinate::getAm(double *am, double *vm, double *pm)const->void
		{
			if (am)
			{
				double pm_default[16], vm_default[16];
				pm = pm ? pm : pm_default;
				vm = vm ? vm : vm_default;
				getPm(pm);
				s_as2am(vs(), as(), pm, am, vm);
			}
			else
			{
				getVm(vm, pm);
			}

		}
		auto Coordinate::getAm(const Coordinate &relative_to, double *am, double *vm, double *pm)const->void
		{
			if (am)
			{
				double vs[6], as[6], pm_default[16], vm_default[16];
				pm = pm ? pm : pm_default;
				vm = vm ? vm : vm_default;
				getPm(relative_to, pm);
				getAs(relative_to, as, vs);
				s_as2am(vs, as, pm, am, vm);
			}
			else
			{
				getVm(vm, pm);
			}

		}
		auto Coordinate::getAa(double *aa, double *va, double *pp)const->void
		{
			if (aa)
			{
				double pp_default[3], va_default[6];
				pp = pp ? pp : pp_default;
				va = va ? va : va_default;
				getPp(pp);
				s_as2aa(vs(), as(), pp, aa, va);
			}
			else
			{
				getVa(va, pp);
			}

		}
		auto Coordinate::getAa(const Coordinate &relative_to, double *aa, double *va, double *pp)const->void
		{
			if (aa)
			{
				double vs[6], as[6], pp_default[3], va_default[6];
				pp = pp ? pp : pp_default;
				va = va ? va : va_default;
				getPp(relative_to, pp);
				getAs(relative_to, as, vs);
				s_as2aa(vs, as, pp, aa, va);
			}
			else
			{
				getVa(relative_to, va, pp);
			}

		}
		auto Coordinate::getAs(double *as, double *vs, double *pm)const->void
		{
			if (as)std::copy(&this->as()[0], &this->as()[6], as);
			getVs(vs, pm);
		}
		auto Coordinate::getAs(const Coordinate &relative_to, double *as, double *vs, double *pm)const->void
		{
			if (as)s_inv_as2as(*relative_to.pm(), relative_to.vs(), relative_to.as(), this->vs(), this->as(), as, vs);
			getVs(relative_to, vs, pm);
		}
		Coordinate::Coordinate(const std::string &name, bool active) :DynEle(name, active) {}
		
		struct Marker::Imp
		{
			double prt_pm_[4][4]{ { 0 } };
			double pm_[4][4]{ { 0 } };
		};
		auto Marker::saveXml(aris::core::XmlElement &xml_ele) const->void
		{
			DynEle::saveXml(xml_ele);
			double pe[6];
			s_pm2pe(*prtPm(), pe);
			xml_ele.SetAttribute("pe", core::Matrix(1, 6, pe).toString().c_str());
		}
		auto Marker::loadXml(const aris::core::XmlElement &xml_ele)->void
		{
			Coordinate::loadXml(xml_ele);
			
			double pm[16];

			s_pe2pm(attributeMatrix(xml_ele, "pe", 1, 6).data(), pm);

			if (xml_ele.Attribute("relative_to"))
			{
				try { s_pm_dot_pm(*static_cast<aris::core::ObjectPool<Marker, Element>&>(this->father()).findByName(xml_ele.Attribute("relative_to"))->prtPm(), pm, *imp_->prt_pm_); }
				catch (std::exception &) { throw std::runtime_error(std::string("can't find relative marker for element \"") + this->name() + "\""); }
			}
			else
			{
				s_vc(16, pm, *imp_->prt_pm_);
			}
		}
		auto Marker::fatherPart() const->const Part&{ return static_cast<const Part &>(this->father().father()); }
		auto Marker::fatherPart()->Part& { return static_cast<Part &>(this->father().father()); }
		auto Marker::glbPm()const->const double4x4&{ s_pm_dot_pm(*fatherPart().pm(), *prtPm(), const_cast<double*>(*imp_->pm_)); return imp_->pm_; }
		auto Marker::glbVs()const->const double6&{ return fatherPart().glbVs(); }
		auto Marker::glbAs()const->const double6&{ return fatherPart().glbAs(); }
		auto Marker::prtPm()const->const double4x4&{ return imp_->prt_pm_; }
		auto Marker::prtVs()const->const double6&{ return fatherPart().prtVs(); }
		auto Marker::prtAs()const->const double6&{ return fatherPart().prtAs(); }
		Marker::~Marker() = default;
		Marker::Marker(const Marker&) = default;
		Marker::Marker(Marker&&) = default;
		Marker& Marker::operator=(const Marker&) = default;
		Marker& Marker::operator=(Marker&&) = default;
		Marker::Marker(const std::string &name, const double *prt_pm, bool active) : Coordinate(name, active)
		{
			static const double default_pm_in[16] = { 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
			prt_pm = prt_pm ? prt_pm : default_pm_in;
			s_vc(16, prt_pm, *imp_->prt_pm_);
		}

		struct Part::Imp
		{
			aris::core::ObjectPool<Marker, Element> *marker_pool_;
			aris::core::ObjectPool<Geometry, Element> *geometry_pool_;

			double prt_im_[6][6]{ { 0 } };
			double prt_gr_[6]{ 0 };
			double prt_as_[6]{ 0 };
			double prt_vs_[6]{ 0 };
			double prt_fg_[6]{ 0 };
			double prt_fv_[6]{ 0 };

			double glb_pm_[4][4]{ { 0 } };
			double glb_vs_[6]{ 0 };
			double glb_as_[6]{ 0 };
			double glb_im_[6][6]{ { 0 } };
			double glb_fg_[6]{ 0 };
			double glb_fv_[6]{ 0 };
			Size row_id_{ 0 }, blk_row_id_{ 0 }, clb_id_{ 0 };
		};
		auto Part::saveXml(aris::core::XmlElement &xml_ele) const->void
		{
			DynEle::saveXml(xml_ele);

			double pe[6];
			s_pm2pe(*pm(), pe);
			xml_ele.SetAttribute("pe", core::Matrix(1, 6, pe).toString().c_str());
			xml_ele.SetAttribute("vel", core::Matrix(1, 6, vs()).toString().c_str());
			xml_ele.SetAttribute("acc", core::Matrix(1, 6, as()).toString().c_str());

			double iv[10];
			s_im2iv(*this->prtIm(), iv);
			xml_ele.SetAttribute("inertia", core::Matrix(1, 10, iv).toString().c_str());
			//xml_ele.SetAttribute("graphic_file_path", imp_->graphic_file_path_.c_str());
		}
		auto Part::loadXml(const aris::core::XmlElement &xml_ele)->void
		{
			s_pe2pm(attributeMatrix(xml_ele, "pe", 1, 6).data(), *imp_->glb_pm_);
			std::copy_n(attributeMatrix(xml_ele, "vel", 1, 6).data(), 6, imp_->glb_vs_);
			std::copy_n(attributeMatrix(xml_ele, "acc", 1, 6).data(), 6, imp_->glb_as_);
			s_iv2im(attributeMatrix(xml_ele, "inertia", 1, 10).data(), *imp_->prt_im_);

			Coordinate::loadXml(xml_ele);

			imp_->marker_pool_ = findOrInsert<aris::core::ObjectPool<Marker, Element> >("marker_pool");
			imp_->geometry_pool_ = findOrInsert<aris::core::ObjectPool<Geometry, Element> >("geometry_pool");

		}
		auto Part::markerPool()->aris::core::ObjectPool<Marker, Element>& { return *imp_->marker_pool_; }
		auto Part::markerPool()const->const aris::core::ObjectPool<Marker, Element>& { return *imp_->marker_pool_; }
		auto Part::geometryPool()->aris::core::ObjectPool<Geometry, Element>& { return *imp_->geometry_pool_; }
		auto Part::geometryPool()const->const aris::core::ObjectPool<Geometry, Element>&{ return *imp_->geometry_pool_; }
		
		auto Part::cptFg(const Coordinate &relative_to, double *fg)const->void
		{
			double prt_gr[3], prt_fg[6];
			s_inv_pm_dot_v3(*pm(), model().environment().gravity(), prt_gr);
			s_mm(6, 1, 3, *prtIm(), 6, prt_gr, 1, prt_fg, 1);

			double pm[16];
			getPm(relative_to, pm);
			s_tf(pm, prt_fg, fg);
		}
		//auto Part::cptGlbFg(double *fg)const->void
		//{
		//	double prt_gr[3], prt_fg[6];
		//	s_inv_pm_dot_v3(*pm(), model().environment().gravity(), prt_gr);
		//	s_mm(6, 1, 3, *prtIm(), 6, prt_gr, 1, prt_fg, 1);
		//	s_tf(*pm(), prt_fg, fg);
		//}
		auto Part::cptPrtFg(double *fg)const->void
		{
			double prt_gr[3];
			s_inv_pm_dot_v3(*pm(), model().environment().gravity(), prt_gr);
			s_mm(6, 1, 3, *prtIm(), 6, prt_gr, 1, fg, 1);
		}

		auto Part::cptFv(const Coordinate &relative_to, double *fv)const->void
		{
			double prt_vs[6], tem[6], prt_fv[6];
			s_inv_tv(*pm(), vs(), prt_vs);
			s_mm(6, 1, 6, *prtIm(), prt_vs, tem);
			s_cf(prt_vs, tem, prt_fv);

			double pm[16];
			getPm(relative_to, pm);
			s_tf(pm, prt_fv, fv);
		}
		//auto Part::cptGlbFv(double *fv)const->void
		//{
		//	double prt_vs[6], prt_fv[6], tem[6];
		//	s_inv_tv(*pm(), vs(), prt_vs);
		//	s_mm(6, 1, 6, *prtIm(), prt_vs, tem);
		//	s_cf(prt_vs, tem, prt_fv);
		//	s_tf(*pm(), prt_fv, fv);
		//}
		auto Part::cptPrtFv(double *fv)const->void
		{
			double prt_vs[6], tem[6];
			s_inv_tv(*pm(), vs(), prt_vs);
			s_mm(6, 1, 6, *prtIm(), prt_vs, tem);
			s_cf(prt_vs, tem, fv);
		}


		auto Part::cptGlbFg(double *fg)const->void
		{
			double prt_gr[3], prt_fg[6];
			s_inv_pm_dot_v3(*pm(), model().environment().gravity(), prt_gr);
			s_mm(6, 1, 3, *prtIm(), 6, prt_gr, 1, prt_fg, 1);
			s_tf(*pm(), prt_fg, fg);
		}
		auto Part::cptGlbFv(double *fv)const->void
		{
			double prt_vs[6], prt_fv[6], tem[6];
			s_inv_tv(*pm(), vs(), prt_vs);
			s_mm(6, 1, 6, *prtIm(), prt_vs, tem);
			s_cf(prt_vs, tem, prt_fv);
			s_tf(*pm(), prt_fv, fv);
		}
		auto Part::cptGlbPf(double *pf)const->void
		{
			double fv[6];
			cptGlbFv(fv);
			cptGlbFg(pf);
			s_vs(6, fv, pf);
		}

		/*auto Part::cptPf(const Coordinate &relative_to, double *pf)const->void 
		{
			double fv[6], prt_pf[6];
			cptPrtFv(fv);
			cptPrtFg(prt_pf);
			s_vs(6, fv, prt_pf);

			double pm[16];
			getPm(relative_to, pm);
			s_tf(pm, prt_pf, pf);
		}*/
		//auto Part::cptGlbPf(double *pf)const->void
		//{
		//	double fv[6];
		//	cptGlbFv(fv);
		//	cptGlbFg(pf);
		//	s_vs(6, fv, pf);
		//}
		auto Part::cptPrtPf(double *pf)const->void
		{
			double fv[6];
			cptPrtFv(fv);
			cptPrtFg(pf);
			s_vs(6, fv, pf);
		}

		auto Part::cptPrtVs(double *prt_vs)const->void
		{
			s_inv_tv(*pm(), vs(), prt_vs);
		}
		auto Part::cptPrtAs(double *prt_as)const->void
		{
			s_inv_tv(*pm(), as(), prt_as);
		}

		auto Part::prtIm()const->const double6x6&{ return imp_->prt_im_; }
		auto Part::updPrtVs()->void { s_inv_tv(*pm(), vs(), imp_->prt_vs_); }
		auto Part::updPrtAs()->void { s_inv_tv(*pm(), as(), imp_->prt_as_); }
		auto Part::glbPm()const->const double4x4&{ return imp_->glb_pm_; }
		auto Part::glbVs()const->const double6&{ return imp_->glb_vs_; }
		auto Part::glbAs()const->const double6&{ return imp_->glb_as_; }
		auto Part::prtPm()const->const double4x4&{
			static const double prt_pm[4][4]
		{
			{ 1,0,0,0 },
			{ 0,1,0,0 },
			{ 0,0,1,0 },
			{ 0,0,0,1 },
		};
		return prt_pm;
		}
		auto Part::prtVs()const->const double6&{ return imp_->prt_vs_; }
		auto Part::prtAs()const->const double6&{ return imp_->prt_as_; }
		auto Part::setPp(const double *pp)->void { if (pp)s_pp2pm(pp, *imp_->glb_pm_); }
		auto Part::setPp(const Coordinate &relative_to, const double *pp)->void
		{
			if (pp)
			{
				double pp_o[3];
				s_pp2pp(*relative_to.pm(), pp, pp_o);
				setPp(pp_o);
			}
		}
		auto Part::setRe(const double *re, const char *type)->void { if (re)s_re2pm(re, *imp_->glb_pm_, type); }
		auto Part::setRe(const Coordinate &relative_to, const double *re, const char *type)->void
		{
			if (re)
			{
				double rm[9];
				s_re2rm(re, rm, type);
				setRm(relative_to, rm);
			}
		}
		auto Part::setRq(const double *rq)->void { if (rq)s_rq2pm(rq, *imp_->glb_pm_); }
		auto Part::setRq(const Coordinate &relative_to, const double *rq)->void
		{
			if (rq)
			{
				double rm[9];
				s_rq2rm(rq, rm);
				setRm(relative_to, rm);
			}
		}
		auto Part::setRm(const double *rm, Size rm_ld)->void { if (rm)s_rm2pm(rm, *imp_->glb_pm_, rm_ld); }
		auto Part::setRm(const Coordinate &relative_to, const double *rm, Size rm_ld)->void { if (rm) s_rm2rm(*relative_to.pm(), rm, *imp_->glb_pm_, rm_ld, 4); }
		auto Part::setPe(const double *pe, const char *type)->void { if (pe)s_pe2pm(pe, *imp_->glb_pm_, type); }
		auto Part::setPe(const Coordinate &relative_to, const double *pe, const char *type)->void
		{
			if (pe)
			{
				double pm[16];
				s_pe2pm(pe, pm, type);
				setPm(relative_to, pm);
			}
		}
		auto Part::setPq(const double *pq)->void { if (pq)s_pq2pm(pq, static_cast<double*>(*imp_->glb_pm_)); }
		auto Part::setPq(const Coordinate &relative_to, const double *pq)->void
		{
			if (pq)
			{
				double pm[16];
				s_pq2pm(pq, pm);
				setPm(relative_to, pm);
			}
		}
		auto Part::setPm(const double *pm)->void { if (pm)std::copy(pm, pm + 16, static_cast<double*>(*imp_->glb_pm_)); }
		auto Part::setPm(const Coordinate &relative_to, const double *pm)->void { if (pm)s_pm2pm(*relative_to.pm(), pm, *imp_->glb_pm_); }
		auto Part::setVp(const double *vp_in, const double *pp_in)->void
		{
			setPp(pp_in);
			double pp[3];
			if (pp_in) std::copy(pp_in, pp_in + 3, pp); else getPp(pp);
			if (vp_in) s_vp2vs(pp, vp_in, imp_->glb_vs_);
		}
		auto Part::setVp(const Coordinate &relative_to, const double *vp_in, const double *pp_in)->void
		{
			setPp(relative_to, pp_in);
			double pp[3], vp_o[3], pp_o[3];
			if (pp_in) std::copy(pp_in, pp_in + 3, pp); else getPp(relative_to, pp);
			if (vp_in)
			{
				s_vp2vp(*relative_to.pm(), relative_to.vs(), pp, vp_in, vp_o, pp_o);
				s_vp2vs(pp_o, vp_o, imp_->glb_vs_);
			}
		}
		auto Part::setWe(const double *we_in, const double *re_in, const char *type)->void
		{
			setRe(re_in, type);
			double re[3], wa[3];
			if (re_in) std::copy(re_in, re_in + 3, re); else getRe(re, type);
			if (we_in)
			{
				s_we2wa(re, we_in, wa, type);
				setWa(wa);
			}
		}
		auto Part::setWe(const Coordinate &relative_to, const double *we_in, const double *re_in, const char *type)->void
		{
			setRe(relative_to, re_in, type);
			double re[3], wa[3];
			if (re_in) std::copy(re_in, re_in + 3, re); else getRe(relative_to, re, type);
			if (we_in)
			{
				s_we2wa(re, we_in, wa, type);
				setWa(relative_to, wa);
			}
		}
		auto Part::setWq(const double *wq_in, const double *rq_in)->void
		{
			setRq(rq_in);
			double rq[4], wa[3];
			if (rq_in) std::copy(rq_in, rq_in + 4, rq); else getRq(rq);
			if (wq_in)
			{
				s_wq2wa(rq, wq_in, wa);
				setWa(wa);
			}
		}
		auto Part::setWq(const Coordinate &relative_to, const double *wq_in, const double *rq_in)->void
		{
			setRq(relative_to, rq_in);
			double rq[4], wa[3];
			if (rq_in) std::copy(rq_in, rq_in + 4, rq); else getRq(relative_to, rq);
			if (wq_in)
			{
				s_wq2wa(rq, wq_in, wa);
				setWa(relative_to, wa);
			}
		}
		auto Part::setWm(const double *wm_in, const double *rm_in, Size wm_ld, Size rm_ld)->void
		{
			setRm(rm_in, rm_ld);
			double rm[9], wa[3];
			if (rm_in) s_mc(3, 3, rm_in, rm_ld, rm, 3); else getRm(rm);
			if (wm_in)
			{
				s_wm2wa(rm, wm_in, wa, 3, wm_ld);
				setWa(wa);
			}
		}
		auto Part::setWm(const Coordinate &relative_to, const double *wm_in, const double *rm_in, Size wm_ld, Size rm_ld)->void
		{
			setRm(relative_to, rm_in, rm_ld);
			double rm[9], wa[3];
			if (rm_in) s_mc(3, 3, rm_in, rm_ld, rm, 3); else getRm(relative_to, rm);
			if (wm_in)
			{
				s_wm2wa(rm, wm_in, wa, 3, wm_ld);
				setWa(relative_to, wa);
			}
		}
		auto Part::setWa(const double *wa_in, const double *rm_in, Size rm_ld)->void
		{
			setRm(rm_in, rm_ld);
			if (wa_in)
			{
				double vp[3], pp[3];
				getVp(vp, pp);
				std::copy(wa_in, wa_in + 3, imp_->glb_vs_ + 3);
				setVp(vp, pp);
			}
		}
		auto Part::setWa(const Coordinate &relative_to, const double *wa_in, const double *rm_in, Size rm_ld)->void
		{
			setRm(relative_to, rm_in, rm_ld);
			if (wa_in)
			{
				double vp[3], pp[3];
				getVp(vp, pp);
				s_wa2wa(*relative_to.pm(), relative_to.vs(), wa_in, imp_->glb_vs_ + 3);
				setVp(vp, pp);
			}
		}
		auto Part::setVe(const double *ve_in, const double *pe_in, const char *type)->void
		{
			setPe(pe_in, type);
			double pe[6];
			if (pe_in) std::copy(pe_in, pe_in + 6, pe); else getPe(pe, type);
			if (ve_in) s_ve2vs(pe, ve_in, imp_->glb_vs_, type);
		}
		auto Part::setVe(const Coordinate &relative_to, const double *ve_in, const double *pe_in, const char *type)->void
		{
			setPe(relative_to, pe_in, type);

			double pe[6], vs[6];
			if (pe_in) std::copy(pe_in, pe_in + 6, pe); else getPe(relative_to, pe, type);
			if (ve_in)
			{
				s_ve2vs(pe, ve_in, vs, type);
				setVs(relative_to, vs);
			}
		}
		auto Part::setVq(const double *vq_in, const double *pq_in)->void
		{
			setPq(pq_in);
			double pq[7];
			if (pq_in) std::copy(pq_in, pq_in + 7, pq); else getPq(pq);
			if (vq_in) s_vq2vs(pq, vq_in, imp_->glb_vs_);
		}
		auto Part::setVq(const Coordinate &relative_to, const double *vq_in, const double *pq_in)->void
		{
			setPq(relative_to, pq_in);
			double pq[7], vs[6];
			if (pq_in) std::copy(pq_in, pq_in + 7, pq); else getPq(relative_to, pq);
			if (vq_in)
			{
				s_vq2vs(pq, vq_in, vs);
				setVs(relative_to, vs);
			}
		}
		auto Part::setVm(const double *vm_in, const double *pm_in)->void
		{
			if (pm_in) setPm(pm_in);
			if (vm_in) s_vm2vs(*pm(), vm_in, imp_->glb_vs_);
		}
		auto Part::setVm(const Coordinate &relative_to, const double *vm_in, const double *pm_in)->void
		{
			setPm(relative_to, pm_in);
			double pm[16], vs[6];
			if (pm_in) std::copy(pm_in, pm_in + 16, pm); else getPm(relative_to, pm);
			if (vm_in)
			{
				s_vm2vs(pm, vm_in, vs);
				setVs(relative_to, vs);
			}
		}
		auto Part::setVa(const double *va_in, const double *pp_in)->void
		{
			setPp(pp_in);
			double pp[3];
			if (pp_in) std::copy(pp_in, pp_in + 3, pp); else getPp(pp);
			if (va_in)s_va2vs(pp, va_in, imp_->glb_vs_);
		}
		auto Part::setVa(const Coordinate &relative_to, const double *va_in, const double *pp_in)->void
		{
			setPp(relative_to, pp_in);
			double pp[3], vs[6];
			if (pp_in) std::copy(pp_in, pp_in + 3, pp); else getPp(relative_to, pp);
			if (va_in)
			{
				s_va2vs(pp, va_in, vs);
				setVs(relative_to, vs);
			}
		}
		auto Part::setVs(const double *vs_in, const double *pm_in)->void
		{
			if (pm_in)setPm(pm_in);
			if (vs_in)std::copy_n(vs_in, 6, imp_->glb_vs_);
		}
		auto Part::setVs(const Coordinate &relative_to, const double *vs_in, const double *pm_in)->void
		{
			if (pm_in)setPm(relative_to, pm_in);
			if (vs_in)s_vs2vs(*relative_to.pm(), relative_to.vs(), vs_in, imp_->glb_vs_);
		}
		auto Part::setAp(const double *ap_in, const double *vp_in, const double *pp_in)->void
		{
			setVp(vp_in, pp_in);
			double pp[3], vp[3];
			if (pp_in) std::copy(pp_in, pp_in + 3, pp); else getPp(pp);
			if (vp_in) std::copy(vp_in, vp_in + 3, vp); else getVp(vp);
			if (ap_in) s_ap2as(pp, vp, ap_in, imp_->glb_as_, imp_->glb_vs_);
		}
		auto Part::setAp(const Coordinate &relative_to, const double *ap_in, const double *vp_in, const double *pp_in)->void
		{
			setVp(relative_to, vp_in, pp_in);
			double pp[3], vp[3], as[6], vs[6];
			if (pp_in) std::copy(pp_in, pp_in + 3, pp); else getPp(relative_to, pp);
			if (vp_in) std::copy(vp_in, vp_in + 3, vp); else getVp(relative_to, vp);
			if (ap_in)
			{
				getXa(relative_to, as + 3, vs + 3);
				s_ap2as(pp, vp, ap_in, as, vs);
				setAs(relative_to, as, vs);
			}
		}
		auto Part::setXe(const double *xe_in, const double *we_in, const double *re_in, const char *type)->void
		{
			if (xe_in)
			{
				double re[3], we[3], xa[3], wa[3], rm[9];
				if (re_in) std::copy(re_in, re_in + 3, re); else getRe(re, type);
				if (we_in) std::copy(we_in, we_in + 3, we); else getWe(we, nullptr, type);
				s_re2rm(re, rm, type);
				s_xe2xa(re, we, xe_in, xa, wa, type);
				setXa(xa, wa, rm);
			}
			else
			{
				setWe(we_in, re_in, type);
			}
		}
		auto Part::setXe(const Coordinate &relative_to, const double *xe_in, const double *we_in, const double *re_in, const char *type)->void
		{
			if (xe_in)
			{
				double re[3], we[3], xa[3], wa[3], rm[9];
				if (re_in) std::copy(re_in, re_in + 3, re); else getRe(relative_to, re, type);
				if (we_in) std::copy(we_in, we_in + 3, we); else getWe(relative_to, we, nullptr, type);
				s_re2rm(re, rm, type);
				s_xe2xa(re, we, xe_in, xa, wa, type);
				setXa(relative_to, xa, wa, rm);
			}
			else
			{
				setWe(relative_to, we_in, re_in, type);
			}
		}
		auto Part::setXq(const double *xq_in, const double *wq_in, const double *rq_in)->void
		{
			if (xq_in)
			{
				double rq[4], wq[4], xa[3], wa[3], rm[9];
				if (rq_in) std::copy(rq_in, rq_in + 4, rq); else getRq(rq);
				if (wq_in) std::copy(wq_in, wq_in + 4, wq); else getWq(wq);
				s_rq2rm(rq, rm);
				s_xq2xa(rq, wq, xq_in, xa, wa);
				setXa(xa, wa, rm);
			}
			else
			{
				setWq(wq_in, rq_in);
			}
		}
		auto Part::setXq(const Coordinate &relative_to, const double *xq_in, const double *wq_in, const double *rq_in)->void
		{
			if (xq_in)
			{
				double rq[4], wq[4], xa[3], wa[3], rm[9];
				if (rq_in) std::copy(rq_in, rq_in + 4, rq); else getRq(relative_to, rq);
				if (wq_in) std::copy(wq_in, wq_in + 4, wq); else getWq(relative_to, wq);
				s_rq2rm(rq, rm);
				s_xq2xa(rq, wq, xq_in, xa, wa);
				setXa(relative_to, xa, wa, rm);
			}
			else
			{
				setWq(relative_to, wq_in, rq_in);
			}
		}
		auto Part::setXm(const double *xm_in, const double *wm_in, const double *rm_in, Size xm_ld, Size wm_ld, Size rm_ld)->void
		{
			if (xm_in)
			{
				double rm[9], wm[9], xa[3], wa[3];
				if (rm_in) std::copy(rm_in, rm_in + 9, rm); else getRm(rm);
				if (wm_in) std::copy(wm_in, wm_in + 9, wm); else getWm(wm);
				s_xm2xa(rm, wm, xm_in, xa, wa);
				setXa(xa, wa, rm);
			}
			else
			{
				setWm(wm_in, rm_in);
			}
		}
		auto Part::setXm(const Coordinate &relative_to, const double *xm_in, const double *wm_in, const double *rm_in, Size xm_ld, Size wm_ld, Size rm_ld)->void
		{
			if (xm_in)
			{
				double rm[9], wm[9], xa[3], wa[3];
				if (rm_in) std::copy(rm_in, rm_in + 9, rm); else getRm(relative_to, rm);
				if (wm_in) std::copy(wm_in, wm_in + 9, wm); else getWm(relative_to, wm);
				s_xm2xa(rm, wm, xm_in, xa, wa);
				setXa(relative_to, xa, wa, rm);
			}
			else
			{
				setWm(relative_to, wm_in, rm_in);
			}
		}
		auto Part::setXa(const double *xa_in, const double *wa_in, const double *rm_in, Size rm_ld)->void
		{
			if (xa_in)
			{
				double pp[3], vp[3], ap[3], rm[9], wa[3];
				getAp(ap, vp, pp);
				setWa(wa_in, rm_in, rm_ld);
				if (rm_in) s_mc(3, 3, rm_in, rm_ld, rm, 3); else getRm(rm);
				if (wa_in) std::copy(wa_in, wa_in + 3, wa); else getWa(wa);
				s_xa2as(xa_in, imp_->glb_as_);
				s_ap2as(pp, vp, ap, imp_->glb_as_, imp_->glb_vs_);
			}
			else
			{
				setWa(wa_in, rm_in, rm_ld);
			}
		}
		auto Part::setXa(const Coordinate &relative_to, const double *xa_in, const double *wa_in, const double *rm_in, Size rm_ld)->void
		{
			if (xa_in)
			{
				double pp[3], vp[3], ap[3], rm[9], wa[3];
				getAp(ap, vp, pp);

				setWa(relative_to, wa_in, rm_in, rm_ld);
				if (rm_in) s_mc(3, 3, rm_in, rm_ld, rm, 3); else getRm(relative_to, rm);
				if (wa_in) std::copy(wa_in, wa_in + 3, wa); else getWa(relative_to, wa);
				s_xa2xa(*relative_to.pm(), relative_to.vs(), relative_to.as(), wa, xa_in, imp_->glb_as_ + 3, imp_->glb_vs_ + 3);

				s_ap2as(pp, vp, ap, imp_->glb_as_, imp_->glb_vs_);
			}
			else
			{
				setWa(relative_to, wa_in, rm_in, rm_ld);
			}
		}
		auto Part::setAe(const double *ae_in, const double *ve_in, const double *pe_in, const char *type)->void
		{
			setVe(ve_in, pe_in, type);
			double pe[6], ve[6];
			if (pe_in) std::copy(pe_in, pe_in + 6, pe); else getPe(pe, type);
			if (ve_in) std::copy(ve_in, ve_in + 6, ve); else getVe(ve, nullptr, type);
			if (ae_in) s_ae2as(pe, ve, ae_in, imp_->glb_as_, nullptr, type);
		}
		auto Part::setAe(const Coordinate &relative_to, const double *ae_in, const double *ve_in, const double *pe_in, const char *type)->void
		{
			setVe(relative_to, ve_in, pe_in, type);
			double pe[6], ve[6], as[6];
			if (pe_in) std::copy(pe_in, pe_in + 6, pe); else getPe(relative_to, pe, type);
			if (ve_in) std::copy(ve_in, ve_in + 6, ve); else getVe(relative_to, ve, nullptr, type);
			if (ae_in)
			{
				s_ae2as(pe, ve, ae_in, as, nullptr, type);
				setAs(relative_to, as);
			}
		}
		auto Part::setAq(const double *aq_in, const double *vq_in, const double *pq_in)->void
		{
			setVq(vq_in, pq_in);
			double pq[7], vq[7];
			if (pq_in) std::copy(pq_in, pq_in + 7, pq); else getPq(pq);
			if (vq_in) std::copy(vq_in, vq_in + 7, vq); else getVq(vq);
			if (aq_in) s_aq2as(pq, vq, aq_in, imp_->glb_as_, nullptr);
		}
		auto Part::setAq(const Coordinate &relative_to, const double *aq_in, const double *vq_in, const double *pq_in)->void
		{
			setVq(relative_to, vq_in, pq_in);
			double pq[7], vq[7], as[6];
			if (pq_in) std::copy(pq_in, pq_in + 7, pq); else getPq(relative_to, pq);
			if (vq_in) std::copy(vq_in, vq_in + 7, vq); else getVq(relative_to, vq);
			if (aq_in)
			{
				s_aq2as(pq, vq, aq_in, as);
				setAs(relative_to, as);
			}
		}
		auto Part::setAm(const double *am_in, const double *vm_in, const double *pm_in)->void
		{
			setVm(vm_in, pm_in);
			double pm[16], vm[16];
			if (pm_in) std::copy(pm_in, pm_in + 16, pm); else getPm(pm);
			if (vm_in) std::copy(vm_in, vm_in + 16, vm); else getVm(vm);
			if (am_in) s_am2as(pm, vm, am_in, imp_->glb_as_);
		}
		auto Part::setAm(const Coordinate &relative_to, const double *am_in, const double *vm_in, const double *pm_in)->void
		{
			setVm(relative_to, vm_in, pm_in);
			double pm[16], vm[16], as[6];
			if (pm_in) std::copy(pm_in, pm_in + 16, pm); else getPm(relative_to, pm);
			if (vm_in) std::copy(vm_in, vm_in + 16, vm); else getVm(relative_to, vm);
			if (am_in)
			{
				s_am2as(pm, vm, am_in, as);
				setAs(relative_to, as);
			}
		}
		auto Part::setAa(const double *aa_in, const double *va_in, const double *pp_in)->void
		{
			setVa(va_in, pp_in);
			double pp[3], va[6];
			if (pp_in) std::copy(pp_in, pp_in + 3, pp); else getPp(pp);
			if (va_in) std::copy(va_in, va_in + 6, va); else getVa(va);
			if (aa_in)s_aa2as(pp, va, aa_in, imp_->glb_as_);

		}
		auto Part::setAa(const Coordinate &relative_to, const double *aa_in, const double *va_in, const double *pp_in)->void
		{
			setVa(relative_to, va_in, pp_in);
			double pp[3], va[6], as[6];
			if (pp_in) std::copy(pp_in, pp_in + 3, pp); else getPp(relative_to, pp);
			if (va_in) std::copy(va_in, va_in + 6, va); else getVa(relative_to, va, nullptr);
			if (aa_in)
			{
				s_aa2as(pp, va, aa_in, as);
				setAs(relative_to, as);
			}
		}
		auto Part::setAs(const double *as_in, const double *vs_in, const double *pm_in)->void
		{
			setVs(vs_in, pm_in);
			if (as_in)std::copy_n(as_in, 6, imp_->glb_as_);
		}
		auto Part::setAs(const Coordinate &relative_to, const double *as_in, const double *vs_in, const double *pm_in)->void
		{
			setVs(relative_to, vs_in, pm_in);
			double vs[6];
			if (vs_in) std::copy(vs_in, vs_in + 6, vs); else getVs(relative_to, vs);
			if (as_in) s_as2as(*relative_to.pm(), relative_to.vs(), relative_to.as(), vs, as_in, imp_->glb_as_);
		}
		Part::~Part() = default;
		Part::Part(const std::string &name, const double *im, const double *pm, const double *vs, const double *as, bool active) : Coordinate(name, active)
		{
			registerType<aris::core::ObjectPool<Marker, Element>>();
			registerType<Marker>();

			registerType<aris::core::ObjectPool<Geometry, Element>>();
			registerType<ParasolidGeometry>();
			
			imp_->marker_pool_ = &add<aris::core::ObjectPool<Marker, Element> >("marker_pool");
			imp_->geometry_pool_ = &add<aris::core::ObjectPool<Geometry, Element> >("geometry_pool");

			static const double default_im[36]{
				1,0,0,0,0,0,
				0,1,0,0,0,0,
				0,0,1,0,0,0,
				0,0,0,1,0,0,
				0,0,0,0,1,0,
				0,0,0,0,0,1,
			};
			static const double default_pm[16]{
				1,0,0,0,
				0,1,0,0,
				0,0,1,0,
				0,0,0,1
			};
			static const double default_vs[6]{ 0,0,0,0,0,0 };
			static const double default_as[6]{ 0,0,0,0,0,0 };

			im = im ? im : default_im;
			pm = pm ? pm : default_pm;
			vs = vs ? vs : default_vs;
			as = as ? as : default_as;

			s_vc(36, im, *imp_->prt_im_);
			setPm(pm);
			setVs(vs);
			setAs(as);
		}
		Part::Part(Part &&other) :Coordinate(std::move(other)), imp_(std::move(other.imp_))
		{
			imp_->marker_pool_ = findType<aris::core::ObjectPool<Marker, Element> >("marker_pool");
			imp_->geometry_pool_ = findType<aris::core::ObjectPool<Geometry, Element> >("geometry_pool");
		};
		Part::Part(const Part &other) :Coordinate(other), imp_(other.imp_)
		{
			imp_->marker_pool_ = findType<aris::core::ObjectPool<Marker, Element> >("marker_pool");
			imp_->geometry_pool_ = findType<aris::core::ObjectPool<Geometry, Element> >("geometry_pool");
		};
		Part& Part::operator=(Part &&other)
		{
			Coordinate::operator=(other);
			imp_ = other.imp_;
			imp_->marker_pool_ = &static_cast<aris::core::ObjectPool<Marker, Element> &>(*findByName("marker_pool"));
			imp_->geometry_pool_ = &static_cast<aris::core::ObjectPool<Geometry, Element> &>(*findByName("geometry_pool"));
			return *this;
		}
		Part& Part::operator=(const Part &other)
		{
			Coordinate::operator=(other);
			imp_ = other.imp_;
			imp_->marker_pool_ = &static_cast<aris::core::ObjectPool<Marker, Element> &>(*findByName("marker_pool"));
			imp_->geometry_pool_ = &static_cast<aris::core::ObjectPool<Geometry, Element> &>(*findByName("geometry_pool"));
			return *this;
		}
		
		auto Geometry::fatherPart() const->const Part&{ return static_cast<const Part &>(this->father().father()); }
		auto Geometry::fatherPart()->Part& { return static_cast<Part &>(this->father().father()); }

		struct ParasolidGeometry::Imp
		{
			double prt_pm_[4][4]{ { 0 } };
			std::string graphic_file_path;
		};
		auto ParasolidGeometry::saveXml(aris::core::XmlElement &xml_ele) const->void
		{
			Element::saveXml(xml_ele);
			double pe[6];
			s_pm2pe(*imp_->prt_pm_, pe);
			xml_ele.SetAttribute("pe", core::Matrix(1, 6, pe).toString().c_str());
			xml_ele.SetAttribute("graphic_file_path", imp_->graphic_file_path.c_str());
		}
		auto ParasolidGeometry::loadXml(const aris::core::XmlElement &xml_ele)->void
		{
			double pm[16];
			s_pe2pm(attributeMatrix(xml_ele, "pe", 1, 6, core::Matrix(1, 6, 0.0)).data(), pm);

			if (xml_ele.Attribute("relative_to"))
			{
				try { s_pm_dot_pm(*static_cast<aris::core::ObjectPool<Marker, Element>&>(this->father()).findByName(xml_ele.Attribute("relative_to"))->prtPm(), pm, *imp_->prt_pm_); }
				catch (std::exception &) { throw std::runtime_error(std::string("can't find relative marker for element \"") + this->name() + "\""); }
			}
			else
			{
				s_vc(16, pm, *imp_->prt_pm_);
			}

			imp_->graphic_file_path = attributeString(xml_ele, "graphic_file_path", "");

			Geometry::loadXml(xml_ele);
		}
		auto ParasolidGeometry::prtPm()const->const double4x4&{ return imp_->prt_pm_; }
		auto ParasolidGeometry::filePath()const->const std::string &{ return imp_->graphic_file_path; }
		ParasolidGeometry::~ParasolidGeometry() = default;
		ParasolidGeometry::ParasolidGeometry(const std::string &name, const std::string &graphic_file_path, const double* prt_pm) : Geometry(name), imp_(new Imp)
		{
			static const double default_pm_in[16] = { 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
			prt_pm = prt_pm ? prt_pm : default_pm_in;
			s_vc(16, prt_pm, *imp_->prt_pm_);

			imp_->graphic_file_path = graphic_file_path;
		}
		ParasolidGeometry::ParasolidGeometry(const ParasolidGeometry &other) = default;
		ParasolidGeometry::ParasolidGeometry(ParasolidGeometry &&other) = default;
		ParasolidGeometry& ParasolidGeometry::operator=(const ParasolidGeometry &other) = default;
		ParasolidGeometry& ParasolidGeometry::operator=(ParasolidGeometry &&other) = default;

	}
}
