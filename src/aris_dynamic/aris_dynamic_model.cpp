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

#include "aris_core.h"
#include "aris_dynamic_kernel.h"
#include "aris_dynamic_model.h"

//#include "C:\Eigen\Eigen"

namespace aris
{
	namespace dynamic
	{
		auto Element::model()->Model& { return dynamic_cast<Model&>(root()); }
		auto Element::model()const->const Model&{ return dynamic_cast<const Model&>(root()); }
		auto Element::attributeMatrix(const aris::core::XmlElement &xml_ele, const std::string &attribute_name)const->aris::core::Matrix
		{
			std::string error = "failed to get Matrix attribute \"" + attribute_name + "\" in element \"" + xml_ele.Name() + "\", because ";

			aris::core::Matrix mat;
			try
			{
				mat = this->model().calculator().calculateExpression(xml_ele.Attribute(attribute_name.c_str()));
			}
			catch (std::exception &e)
			{
				throw std::runtime_error(error + "failed to evaluate matrix:" + e.what());
			}

			return mat;
		}
		auto Element::attributeMatrix(const aris::core::XmlElement &xml_ele, const std::string &attribute_name, const aris::core::Matrix& default_value)const->aris::core::Matrix
		{
			return xml_ele.Attribute(attribute_name.c_str()) ? attributeMatrix(xml_ele, attribute_name) : default_value;
		}
		auto Element::attributeMatrix(const aris::core::XmlElement &xml_ele, const std::string &attribute_name, std::size_t m, std::size_t n)const->aris::core::Matrix
		{
			std::string error = "failed to get Matrix attribute \"" + attribute_name + "\" in element \"" + xml_ele.Name() + "\", because ";
			
			aris::core::Matrix mat = attributeMatrix(xml_ele, attribute_name);

			if (mat.m() != m || mat.n() != n)
			{
				throw std::runtime_error(error + "matrix has wrong dimensions, it's dimentsion should be \"" +std::to_string(m)+","+ std::to_string(n)
					+"\", while the real value is \"" + std::to_string(mat.m()) + "," + std::to_string(mat.n())+"\"");
			}

			return mat;
		}
		auto Element::attributeMatrix(const aris::core::XmlElement &xml_ele, const std::string &attribute_name, std::size_t m, std::size_t n, const aris::core::Matrix& default_value)const->aris::core::Matrix
		{
			return xml_ele.Attribute(attribute_name.c_str()) ? attributeMatrix(xml_ele, attribute_name, m, n) : default_value;
		}

		auto DynEle::saveXml(aris::core::XmlElement &xml_ele) const->void
		{
			Element::saveXml(xml_ele);
			xml_ele.SetAttribute("active", active() ? "true" : "false");
		}
		DynEle::DynEle(Object &father, const aris::core::XmlElement &xml_ele) :Element(father, xml_ele)
		{
			active_ = attributeBool(xml_ele, "active", true);
		}
		
		auto Coordinate::getPp(double *pp)const->void {	if(pp)s_pm2pp(*pm(), pp); }
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
		auto Coordinate::getRm(double *rm, int rm_ld)const->void { if (rm)s_pm2rm(*pm(), rm, rm_ld); }
		auto Coordinate::getRm(const Coordinate &relative_to, double *rm, int rm_ld)const->void
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
		auto Coordinate::getPm(double *pm)const->void { if (pm)std::copy(&this->pm()[0][0], &this->pm()[0][0] + 16, pm); }
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
		auto Coordinate::getWm(double *wm, double *rm, int wm_ld, int rm_ld)const->void
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
		auto Coordinate::getWm(const Coordinate &relative_to, double *wm, double *rm, int wm_ld, int rm_ld)const->void
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
		auto Coordinate::getWa(double *wa, double *rm, int rm_ld)const->void
		{
			if (wa)	{ s_vs2wa(vs(), wa);}
			getRm(rm, rm_ld);
		}
		auto Coordinate::getWa(const Coordinate &relative_to, double *wa, double *rm, int rm_ld)const->void
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
		auto Coordinate::getXm(double *xm, double *wm, double *rm, int xm_ld, int wm_ld, int rm_ld)const->void
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
		auto Coordinate::getXm(const Coordinate &relative_to, double *xm, double *wm, double *rm, int xm_ld, int wm_ld, int rm_ld)const->void
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
		auto Coordinate::getXa(double *xa, double *wa, double *rm, int rm_ld)const->void
		{
			if (xa)s_as2xa(as(), xa);
			getWa(wa, rm, rm_ld);
		}
		auto Coordinate::getXa(const Coordinate &relative_to, double *xa, double *wa, double *rm, int rm_ld)const->void
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
		Coordinate::Coordinate(const std::string &name, bool active):DynEle(name, active){}

		auto Interaction::saveXml(aris::core::XmlElement &xml_ele) const->void
		{
			DynEle::saveXml(xml_ele);

			xml_ele.SetAttribute("prt_m", this->makI().fatherPart().name().c_str());
			xml_ele.SetAttribute("prt_n", this->makJ().fatherPart().name().c_str());
			xml_ele.SetAttribute("mak_i", this->makI().name().c_str());
			xml_ele.SetAttribute("mak_j", this->makJ().name().c_str());
		}
		Interaction::Interaction(Object &father, const aris::core::XmlElement &xml_ele)
			: DynEle(father, xml_ele)
		{
			if (model().findByName("part_pool") == model().children().end())
				throw std::runtime_error("you must insert \"part_pool\" node before insert " + type() + " \"" + name() + "\"" );
			auto &part_pool = static_cast<aris::core::ObjectPool<Part, Element>&>(*model().findByName("part_pool"));
			
			if (!xml_ele.Attribute("prt_m"))throw std::runtime_error(std::string("xml element \"") + name() + "\" must have Attribute \"prt_m\"");
			auto prt_m = part_pool.findByName(xml_ele.Attribute("prt_m"));
			if (prt_m == part_pool.end())	throw std::runtime_error(std::string("can't find part m for element \"") + this->name() + "\"");

			if (!xml_ele.Attribute("mak_i"))throw std::runtime_error(std::string("xml element \"") + name() + "\" must have Attribute \"mak_i\"");
			auto mak_i = prt_m->markerPool().findByName(xml_ele.Attribute("mak_i")); 
			if (mak_i == prt_m->markerPool().end()) 
				throw std::runtime_error(std::string("can't find marker i for element \"") + this->name() + "\"");
			makI_ = &(*mak_i);

			if (!xml_ele.Attribute("prt_n"))throw std::runtime_error(std::string("xml element \"") + name() + "\" must have Attribute \"prt_n\"");
			auto prt_n = part_pool.findByName(xml_ele.Attribute("prt_n"));
			if (prt_n == part_pool.end())throw std::runtime_error(std::string("can't find part n for element \"") + this->name() + "\"");

			if (!xml_ele.Attribute("mak_j"))throw std::runtime_error(std::string("xml element \"") + name() + "\" must have Attribute \"mak_j\"");
			auto mak_j = prt_n->markerPool().findByName(xml_ele.Attribute("mak_j"));
			if (mak_j == prt_n->markerPool().end())throw std::runtime_error(std::string("can't find marker j for element \"") + this->name() + "\"");
			makJ_ = &(*mak_j);
		}
		
		struct Constraint::Imp
		{
			std::size_t col_id_;
		};
		auto Constraint::colID()const->std::size_t { return imp_->col_id_; }
		auto Constraint::cptCa(double *ca)const->void
		{
			double vm_cross_vn[6], tem[6];
			s_cv(makI().fatherPart().glbVs(), makJ().fatherPart().glbVs(), vm_cross_vn);
			s_inv_tv(*makI().fatherPart().pm(), vm_cross_vn, tem);
			s_mmTN(dim(), 1, 6, -1.0, prtCmPtrI(), tem, ca);
		}
		auto Constraint::cptCp(double *cp)const->void
		{
			double pq_j2i[7];
			double pm_j2i[4][4];
			double diff[6];

			s_inv_pm_dot_pm(*makI().pm(), *makJ().pm(), *pm_j2i);
			s_pm2pq(*pm_j2i, pq_j2i);

			double theta = atan2(s_norm(3, pq_j2i + 3, 1), pq_j2i[6]) * 2;

			if (theta < 1e-3)
			{
				s_nv(3, 2.0, pq_j2i + 3);
			}
			else
			{
				s_nv(3, theta / std::sin(theta / 2.0), pq_j2i + 3);
			}

			// 此时位移差值在makI()坐标系中。需要转换到部件坐标系下。
			s_tv(*makI().prtPm(), pq_j2i, diff);

			s_mmTN(dim(), 1, 6, prtCmPtrI(), diff, cp);
		}
		auto Constraint::cptPrtCm(double *prt_cmI, double *prt_cmJ, int cmI_ld, int cmJ_ld)const->void
		{
			cmI_ld = std::max(cmI_ld, static_cast<int>(dim()));
			cmJ_ld = std::max(cmJ_ld, static_cast<int>(dim()));
			
			s_mc(6, dim(), const_cast<double*>(prtCmPtrI()), dim(), prt_cmI, cmI_ld);
			
			double pm_M2N[4][4];
			s_inv_pm_dot_pm(*makJ().fatherPart().pm(), *makI().fatherPart().pm(), *pm_M2N);
			s_tf_n(dim(), -1.0, *pm_M2N, prtCmPtrI(), dim(), prt_cmJ, cmJ_ld);
		}
		auto Constraint::cptGlbCm(double *glb_cmI, double *glb_cmJ, int cmI_ld, int cmJ_ld)const->void
		{
			cmI_ld = std::max(cmI_ld, static_cast<int>(dim()));
			cmJ_ld = std::max(cmJ_ld, static_cast<int>(dim()));
			
			s_tf_n(dim(), *makI().fatherPart().pm(), prtCmPtrI(), dim(), glb_cmI, cmI_ld);
			s_mc(6, dim(), -1.0, glb_cmI, cmI_ld, glb_cmJ, cmJ_ld);
		}
		auto Constraint::setCf(const double *cf)const->void { std::copy(cf, cf + dim(), const_cast<double *>(caPtr())); }
		
		auto Constraint::updPrtCm()->void
		{
			double pm_M2N[4][4];
			s_inv_pm_dot_pm(*makJ().fatherPart().pm(), *makI().fatherPart().pm(), *pm_M2N);
			s_tf_n(dim(), -1.0, *pm_M2N, prtCmPtrI(), const_cast<double*>(prtCmPtrJ()));
		}
		auto Constraint::updGlbCm()->void
		{
			s_tf_n(dim(), *makI().fatherPart().pm(), prtCmPtrI(), const_cast<double*>(glbCmPtrI()));
			s_mc(6, dim(), -1.0, glbCmPtrI(), const_cast<double*>(glbCmPtrJ()));
		}
		auto Constraint::updCa()->void
		{
			double vm_cross_vn[6], tem[6];
			s_cv(makI().fatherPart().glbVs(), makJ().fatherPart().glbVs(), vm_cross_vn);
			s_inv_tv(*makI().fatherPart().pm(), vm_cross_vn, tem);
			s_mmTN(dim(), 1, 6, -1.0, prtCmPtrI(), tem, const_cast<double*>(caPtr()));
		}
		auto Constraint::updCe()->void
		{
			double pq_n2m[7];
			double pm_n2m[4][4];
			double diff[6];

			s_inv_pm_dot_pm(*makI().pm(), *makJ().pm(), *pm_n2m);
			s_pm2pq(*pm_n2m, pq_n2m);

			double theta = atan2(s_norm(3, pq_n2m + 3, 1), pq_n2m[6]) * 2;
			
			if (theta < 1e-3)
			{
				s_nv(3, 2.0, pq_n2m + 3);
			}
			else
			{
				s_nv(3, theta / std::sin(theta / 2.0), pq_n2m + 3);
			}


			//s_nv(3, 2.0, pq_n2m + 3);
			s_tv(*makI().pm(), pq_n2m, diff);
			
			s_mmTN(dim(), 1, 6, glbCmPtrI(), diff, const_cast<double*>(cePtr()));
		}
		Constraint::~Constraint() = default;
		Constraint::Constraint(const std::string &name, Marker &makI, Marker &makJ, bool is_active): Interaction(name, makI, makJ, is_active) {}
		Constraint::Constraint(Object &father, const aris::core::XmlElement &xml_ele): Interaction(father, xml_ele) {}
		Constraint::Constraint(const Constraint&) = default;
		Constraint::Constraint(Constraint&&) = default;
		Constraint& Constraint::operator=(const Constraint&) = default;
		Constraint& Constraint::operator=(Constraint&&) = default;

		auto Environment::saveXml(aris::core::XmlElement &xml_ele) const->void
		{
			Object::saveXml(xml_ele);
			xml_ele.SetAttribute("gravity", core::Matrix(1, 6, gravity_).toString().c_str());
		}
		auto Environment::saveAdams(std::ofstream &file) const->void
		{
			file << "!----------------------------------- Environment -------------------------------!\r\n!\r\n!\r\n";
			file << "!-------------------------- Default Units for Model ---------------------------!\r\n"
				<< "!\r\n"
				<< "!\r\n"
				<< "defaults units  &\r\n"
				<< "    length = meter  &\r\n"
				<< "    angle = rad  &\r\n"
				<< "    force = newton  &\r\n"
				<< "    mass = kg  &\r\n"
				<< "    time = sec\r\n"
				<< "!\n"
				<< "defaults units  &\r\n"
				<< "    coordinate_system_type = cartesian  &\r\n"
				<< "    orientation_type = body313\r\n"
				<< "!\r\n"
				<< "!------------------------ Default Attributes for Model ------------------------!\r\n"
				<< "!\r\n"
				<< "!\r\n"
				<< "defaults attributes  &\r\n"
				<< "    inheritance = bottom_up  &\r\n"
				<< "    icon_visibility = off  &\r\n"
				<< "    grid_visibility = off  &\r\n"
				<< "    size_of_icons = 5.0E-002  &\r\n"
				<< "    spacing_for_grid = 1.0\r\n"
				<< "!\r\n"
				<< "!------------------------------ Adams/View Model ------------------------------!\r\n"
				<< "!\r\n"
				<< "!\r\n"
				<< "model create  &\r\n"
				<< "   model_name = " << this->model().name() << "\r\n"
				<< "!\r\n"
				<< "view erase\r\n"
				<< "!\r\n"
				<< "!---------------------------------- Accgrav -----------------------------------!\r\n"
				<< "!\r\n"
				<< "!\r\n"
				<< "force create body gravitational  &\r\n"
				<< "    gravity_field_name = gravity  &\r\n"
				<< "    x_component_gravity = " << this->gravity_[0] << "  &\r\n"
				<< "    y_component_gravity = " << this->gravity_[1] << "  &\r\n"
				<< "    z_component_gravity = " << this->gravity_[2] << "\r\n"
				<< "!\r\n";
		}
		Environment::Environment(Object &father, const aris::core::XmlElement &xml_ele):Element(father, xml_ele)
		{
			std::copy_n(attributeMatrix(xml_ele, "gravity", 1, 6).data(), 6, gravity_);
		}
		
		struct Akima::Imp
		{
			std::vector<double> x_, y_;
			std::vector<double> _p0;
			std::vector<double> _p1;
			std::vector<double> _p2;
			std::vector<double> _p3;
		};
		auto Akima::saveAdams(std::ofstream &file) const->void
		{
			file << "data_element create spline &\r\n"
				<< "    spline_name = ." << model().name() + "." + name() + " &\r\n"
				<< "    adams_id = " << adamsID() << "  &\r\n"
				<< "    units = m &\r\n"
				<< "    x = " << x().at(0);
			for (auto p = x().begin() + 1; p < x().end(); ++p)
			{
				file << "," << *p;
			}
			file << "    y = " << y().at(0);
			for (auto p = y().begin() + 1; p < y().end(); ++p)
			{
				file << "," << *p;
			}
			file << " \r\n!\r\n";
		}
		auto Akima::saveXml(aris::core::XmlElement &xml_ele) const->void
		{
			Element::saveXml(xml_ele);

			aris::core::Matrix mat_x(1, x().size(), imp_->x_.data());
			aris::core::Matrix mat_y(1, x().size(), imp_->y_.data());
			xml_ele.SetAttribute("x", mat_x.toString().c_str());
			xml_ele.SetAttribute("y", mat_y.toString().c_str());
		}
		auto Akima::x() const->const std::vector<double> & { return imp_->x_; }
		auto Akima::y() const->const std::vector<double> & { return imp_->y_; }
		auto Akima::operator()(double x, char order) const->double
		{
			// 寻找第一个大于x的位置 //
			auto bIn = std::upper_bound(imp_->x_.begin(), imp_->x_.end() - 1, x);

			int id = std::max<int>(bIn - imp_->x_.begin() - 1, 0);

			double w = x - imp_->x_[id];

			switch (order)
			{
			case '1':
				return (3 * w*imp_->_p3[id] + 2 * imp_->_p2[id])*w + imp_->_p1[id];
			case '2':
				return (6 * w*imp_->_p3[id] + 2 * imp_->_p2[id]);
			case '0':
			default:
				return ((w*imp_->_p3[id] + imp_->_p2[id])*w + imp_->_p1[id])*w + imp_->_p0[id];
			}
		}
		auto Akima::operator()(int length, const double *x_in, double *y_out, char order)const->void
		{
			for (int i = 0; i < length; ++i)
			{
				y_out[i] = this->operator()(x_in[i], order);
			}
		}
		Akima::~Akima() = default;
		Akima::Akima(const std::string &name, int num, const double *x_in, const double *y_in): Element(name)
		{
			std::list<std::pair<double, double> > data_list;

			for (int i = 0; i < num; ++i)
			{
				data_list.push_back(std::make_pair(x_in[i], y_in[i]));
			}

			this->operator=(Akima(name, data_list));
		}
		Akima::Akima(Object &father, const aris::core::XmlElement &xml_ele) : Element(father, xml_ele)
		{
			auto mat_x = attributeMatrix(xml_ele, "x");
			auto mat_y = attributeMatrix(xml_ele, "y");
			*this = Akima(name(), std::list<double>(mat_x.begin(), mat_x.end()), std::list<double>(mat_y.begin(), mat_y.end()));
		}
		Akima::Akima(const std::string &name, const std::list<double> &x_in, const std::list<double> &y_in): Element(name)
		{
			if (x_in.size() != y_in.size())throw std::runtime_error("input x and y must have same length");

			std::list<std::pair<double, double> > data_list;

			auto pX = x_in.begin();
			auto pY = y_in.begin();

			for (std::size_t i = 0; i < x_in.size(); ++i)
			{
				data_list.push_back(std::make_pair(*pX, *pY));
				++pX;
				++pY;
			}

			this->operator=(Akima(name, data_list));
		}
		Akima::Akima(const std::string &name, const std::list<std::pair<double, double> > &data_in): Element(name)
		{
			auto data_list = data_in;

			if (data_list.size() < 4)throw std::runtime_error("Akima must be inited with data size more than 4");

			// 对数据进行排序,并保存 //
			data_list.sort([](const std::pair<double, double> &a, const std::pair<double, double> &b)
			{
				return a.first < b.first;
			});

			for (auto &p : data_list)
			{
				imp_->x_.push_back(p.first);
				imp_->y_.push_back(p.second);
			}

			// 开始计算 //
			std::vector<double> s(data_list.size() + 3), ds(data_list.size() + 2), t(data_list.size());

			for (std::size_t i = 0; i < data_list.size() - 1; ++i)
			{
				s[i + 2] = (imp_->y_[i + 1] - imp_->y_[i]) / (imp_->x_[i + 1] - imp_->x_[i]);
			}

			s[1] = 2 * s[2] - s[3];
			s[0] = 2 * s[1] - s[2];
			s[data_list.size() + 1] = 2 * s[data_list.size()] - s[data_list.size() - 1];
			s[data_list.size() + 2] = 2 * s[data_list.size() + 1] - s[data_list.size()];

			for (std::size_t i = 0; i < data_list.size() + 2; ++i)
			{
				ds[i] = std::abs(s[i + 1] - s[i]);
			}

			for (std::size_t i = 0; i < data_list.size(); ++i)
			{
				if (ds[i] + ds[i + 2]<1e-12)// 前后两段的斜斜率都为0 //
				{
					t[i] = (s[i + 1] + s[i + 2]) / 2;
				}
				else
				{
					t[i] = (ds[i + 2] * s[i + 1] + ds[i] * s[i + 2]) / (ds[i] + ds[i + 2]);
				}
			}

			// 所需储存的变量 //
			imp_->_p0.resize(data_list.size() - 1);
			imp_->_p1.resize(data_list.size() - 1);
			imp_->_p2.resize(data_list.size() - 1);
			imp_->_p3.resize(data_list.size() - 1);

			for (std::size_t i = 0; i < data_list.size() - 1; ++i)
			{
				imp_->_p0[i] = imp_->y_[i];
				imp_->_p1[i] = t[i];
				imp_->_p2[i] = (3 * s[i + 2] - 2 * t[i] - t[i + 1]) / (imp_->x_[i + 1] - imp_->x_[i]);
				imp_->_p3[i] = (t[i] + t[i + 1] - 2 * s[i + 2]) / (imp_->x_[i + 1] - imp_->x_[i]) / (imp_->x_[i + 1] - imp_->x_[i]);
			}
		}
		Akima::Akima(const std::string &name, const std::list<std::pair<double, double> > &data_in, double begin_slope, double end_slope): Element(name)
		{
			auto data_list = data_in;

			if (data_list.size() < 4)throw std::runtime_error("Akima must be inited with data size more than 4");

			// 对数据进行排序,并保存 //
			data_list.sort([](const std::pair<double, double> &a, const std::pair<double, double> &b)
			{
				return a.first < b.first;
			});

			for (auto &p : data_list)
			{
				imp_->x_.push_back(p.first);
				imp_->y_.push_back(p.second);
			}

			// 开始计算 //
			std::vector<double> s(data_list.size() + 3), ds(data_list.size() + 2), t(data_list.size());

			for (std::size_t i = 0; i < data_list.size() - 1; ++i)
			{
				s[i + 2] = (imp_->y_[i + 1] - imp_->y_[i]) / (imp_->x_[i + 1] - imp_->x_[i]);
			}
			///////// this part is different
			s[1] = begin_slope;
			s[0] = begin_slope;
			s[data_list.size() + 1] = end_slope;
			s[data_list.size() + 2] = end_slope;
			///////// this part is different end
			for (std::size_t i = 0; i < data_list.size() + 2; ++i)
			{
				ds[i] = std::abs(s[i + 1] - s[i]);
			}

			for (std::size_t i = 0; i < data_list.size(); ++i)
			{
				if (ds[i] + ds[i + 2]<1e-12)// 前后两段的斜斜率都为0 //
				{
					t[i] = (s[i + 1] + s[i + 2]) / 2;
				}
				else
				{
					t[i] = (ds[i + 2] * s[i + 1] + ds[i] * s[i + 2]) / (ds[i] + ds[i + 2]);
				}
			}

			// 所需储存的变量 //
			imp_->_p0.resize(data_list.size() - 1);
			imp_->_p1.resize(data_list.size() - 1);
			imp_->_p2.resize(data_list.size() - 1);
			imp_->_p3.resize(data_list.size() - 1);

			for (std::size_t i = 0; i < data_list.size() - 1; ++i)
			{
				imp_->_p0[i] = imp_->y_[i];
				imp_->_p1[i] = t[i];
				imp_->_p2[i] = (3 * s[i + 2] - 2 * t[i] - t[i + 1]) / (imp_->x_[i + 1] - imp_->x_[i]);
				imp_->_p3[i] = (t[i] + t[i + 1] - 2 * s[i + 2]) / (imp_->x_[i + 1] - imp_->x_[i]) / (imp_->x_[i + 1] - imp_->x_[i]);
			}
		}
		Akima::Akima(const Akima&) = default;
		Akima::Akima(Akima&&) = default;
		Akima& Akima::operator=(const Akima&) = default;
		Akima& Akima::operator=(Akima&&) = default;

		struct Script::Imp
		{
			struct Node
			{
				virtual ~Node() = default;
				auto virtual doNode()->void {}
				auto virtual adamsScript()const->std::string = 0;
				auto virtual msConsumed()const->std::uint32_t { return 0; }

				static auto create(Model *model, const std::string &expression)->Node*
				{
					auto& joint_pool = static_cast<aris::core::ObjectPool<Joint, Element>&>(*model->findByName("Joint"));
					auto& force_pool = static_cast<aris::core::ObjectPool<Force, Element>&>(*model->findByName("Force"));
					auto& motion_pool = static_cast<aris::core::ObjectPool<Motion, Element>&>(*model->findByName("Motion"));
					auto& part_pool = static_cast<aris::core::ObjectPool<Part, Element>&>(*model->findByName("Part"));
					
					std::string split_exp = expression;

					std::string delim_str("/,=");
					for (auto delim : delim_str)
					{
						std::regex delim_regex(std::string("") + delim);
						std::string replace = std::string(" ") + delim + std::string(" ");
						split_exp = std::regex_replace(split_exp, delim_regex, replace);
					}

					std::stringstream stream(split_exp);
					std::string word;
					std::size_t id;

					if (!(stream >> word))return nullptr;

					if (word == "activate")
					{
						std::string type;
						stream >> word;
						stream >> type;
						stream >> word;
						stream >> word;
						stream >> word;
						stream >> id;
						if (type == "joint")return new ActNode(joint_pool.at(id - 1), true);
						else if (type == "sforce")return new ActNode(force_pool.at(id - 1), true);
						else if (type == "motion")return new ActNode(motion_pool.at(id - 1), true);
						else throw std::runtime_error("unrecognized deactivate element type");
					}
					else if (word == "deactivate")
					{
						std::string type;
						stream >> word;
						stream >> type;
						stream >> word;
						stream >> word;
						stream >> word;
						stream >> id;
						if (type == "joint")return new ActNode(joint_pool.at(id - 1), false);
						else if (type == "sforce")return new ActNode(force_pool.at(id - 1), false);
						else if (type == "motion")return new ActNode(motion_pool.at(id - 1), false);
						else throw std::runtime_error("unrecognized deactivate element type");
					}
					else if (word == "marker")
					{
						stream >> word;
						stream >> id;
						stream >> word;
						stream >> word;
						stream >> word;
						double prt_pe[6];
						stream >> prt_pe[0];
						stream >> word;
						stream >> prt_pe[1];
						stream >> word;
						stream >> prt_pe[2];
						stream >> word;
						stream >> word;
						stream >> word;
						stream >> prt_pe[3];
						stream >> word;
						stream >> prt_pe[4];
						stream >> word;
						stream >> prt_pe[5];
						stream >> word;

						std::size_t size{ 0 };
						for (auto &prt : part_pool)
						{
							size += prt.markerPool().size();
							if (size > id)
							{
								return new AlnNode(prt.markerPool().at(id - 1 + prt.markerPool().size() - size), prt_pe);

							}
						}
						throw std::runtime_error("invalid marker id in script");
						
					}
					else if (word == "simulate")
					{
						double dur, dt;
						stream >> word;
						stream >> word;
						stream >> word;
						stream >> word;
						stream >> word;
						stream >> dur;
						stream >> word;
						stream >> word;
						stream >> word;
						stream >> dt;

						return new SimNode(static_cast<std::uint32_t>(dur * 1000), static_cast<std::uint32_t>(dt * 1000));
					}
					else
					{
						throw std::runtime_error("failed parse script");
					}
				}
			};
			struct ActNode final :public Node
			{
				auto virtual doNode()->void override final { dyn_ele_->activate(isActive); }
				auto virtual adamsScript()const->std::string override final
				{
					std::stringstream ss;
					std::string cmd = isActive ? "activate/" : "deactivate/";
					ss << cmd << dyn_ele_->adamsScriptType() << ", id=" << dyn_ele_->adamsID();
					return std::move(ss.str());
				}
				explicit ActNode(DynEle &ele, bool isActive) :dyn_ele_(&ele), isActive(isActive) {}

				bool isActive;
				DynEle *dyn_ele_;
			};
			struct AlnNode final :public Node
			{
				auto virtual doNode()->void override final
				{
					if (mak_target_)
					{
						double pm_target_g[16];

						s_pm_dot_pm(*mak_target_->fatherPart().pm(), *mak_target_->prtPm(), pm_target_g);
						s_inv_pm_dot_pm(*mak_move_->fatherPart().pm(), pm_target_g, const_cast<double *>(&mak_move_->prtPm()[0][0]));
						s_pm2pe(*mak_move_->prtPm(), prt_pe_);
					}
				}
				auto virtual adamsScript()const->std::string override final
				{
					std::stringstream ss;
					ss << std::setprecision(15) << "marker/" << mak_move_->adamsID()
						<< " , QP = " << prt_pe_[0] << "," << prt_pe_[1] << "," << prt_pe_[2]
						<< " , REULER =" << prt_pe_[3] << "," << prt_pe_[4] << "," << prt_pe_[5];
					return std::move(ss.str());
				}
				explicit AlnNode(Marker &mak_move, const Marker &mak_target) :mak_move_(&mak_move), mak_target_(&mak_target) {}
				explicit AlnNode(Marker &mak_move, const double *prt_pe) :mak_move_(&mak_move), mak_target_(nullptr)
				{
					std::copy_n(prt_pe, 6, prt_pe_);
				}

				Marker *mak_move_;
				const Marker *mak_target_;
				double prt_pe_[6];
			};
			struct SimNode final :public Node
			{
				auto virtual msConsumed()const->std::uint32_t override final { return ms_dur_; }
				auto virtual adamsScript()const->std::string override final
				{
					std::stringstream ss;
					ss << "simulate/transient, dur=" << double(ms_dur_) / 1000.0 << ", dtout=" << double(ms_dt_) / 1000.0;
					return std::move(ss.str());
				}

				explicit SimNode(std::uint32_t ms_dur, std::uint32_t ms_dt) :ms_dur_(ms_dur), ms_dt_(ms_dt) { }
				std::uint32_t ms_dur_;
				std::uint32_t ms_dt_;
			};

			Imp(Model *model) :model_(model) {}
			Imp(const Imp &other) :model_(other.model_)
			{
				for (auto &node : other.node_list_)
				{
					this->node_list_.push_back(std::unique_ptr<Node>(Node::create(model_, node->adamsScript())));
				}
			}
			auto operator=(const Imp& other)->Imp&
			{
				this->node_list_.clear();
				for (auto &node : other.node_list_)
				{
					this->node_list_.push_back(std::unique_ptr<Node>(Node::create(model_, node->adamsScript())));
				}

				return *this;
			}

			std::list<std::unique_ptr<Node> > node_list_;
			Model *model_;
		};
		auto Script::saveXml(aris::core::XmlElement &xml_ele) const->void
		{
			Element::saveXml(xml_ele);

			std::stringstream stream;
			stream << "\n";
			for (auto &node : imp_->node_list_)
			{
				stream << node->adamsScript() << "\n";
			}

			xml_ele.SetText(stream.str().c_str());
		}
		auto Script::saveAdams(std::ofstream &file) const->void
		{
			file << "simulation script create &\r\n"
				<< "sim_script_name = default_script &\r\n"
				<< "solver_commands = ";

			for (auto &node : imp_->node_list_)
			{
				file << "&\r\n\"" << node->adamsScript() << "\",";
			}

			file << "\"\"\r\n\r\n";
		}
		auto Script::act(DynEle &ele, bool active)->void
		{
			imp_->node_list_.push_back(std::unique_ptr<Imp::Node>(new Imp::ActNode(ele, active)));
		}
		auto Script::aln(Marker &mak_move, const Marker& mak_target)->void
		{
			imp_->node_list_.push_back(std::unique_ptr<Imp::Node>(new Imp::AlnNode(mak_move, mak_target)));
		}
		auto Script::sim(std::uint32_t ms_dur, std::uint32_t ms_dt)->void
		{
			imp_->node_list_.push_back(std::unique_ptr<Imp::Node>(new Imp::SimNode(ms_dur, ms_dt)));
		}
		auto Script::empty() const->bool { return imp_->node_list_.empty(); }
		auto Script::endTime()const->std::uint32_t
		{
			std::uint32_t end_time{ 0 };
			for (auto& node : imp_->node_list_)end_time += node->msConsumed();
			return end_time;
		}
		auto Script::doScript(std::uint32_t ms_begin, std::uint32_t ms_end)->void
		{
			std::uint32_t now{ 0 };
			for (auto& node : imp_->node_list_)
			{
				if ((now >= ms_begin) && (now < ms_end)) node->doNode();
				if ((now += node->msConsumed()) >= ms_end)break;
			}
		}
		auto Script::clear()->void { return imp_->node_list_.clear(); }
		Script::~Script() = default;
		Script::Script(const std::string &name) :Element(name), imp_(new Imp(&model())) {}
		Script::Script(Object &father, const aris::core::XmlElement &xml_ele) :Element(father, xml_ele), imp_(new Imp(&model()))
		{
			std::stringstream stream(xml_ele.GetText());
			std::string line;
			while (std::getline(stream, line))
			{
				line.erase(0, line.find_first_not_of(" "));
				line.erase(line.find_last_not_of(" ") + 1);
				if (line != "")imp_->node_list_.push_back(std::unique_ptr<Imp::Node>(Imp::Node::create(&model(), line)));
			}
		}
		Script::Script(const Script&) = default;
		Script::Script(Script&&) = default;
		Script& Script::operator=(const Script&) = default;
		Script& Script::operator=(Script&&) = default;

		auto Variable::saveXml(aris::core::XmlElement &xml_ele) const->void
		{
			Element::saveXml(xml_ele);
			xml_ele.SetText(this->toString().c_str());
		}

		struct Marker::Imp
		{
			double prt_pm_[4][4]{ { 0 } };
			double pm_[4][4]{ { 0 } };
		};
		auto Marker::adamsID()const->std::size_t 
		{
			std::size_t size{ 0 };

			for (auto &prt : model().partPool())
			{
				if (&prt == &fatherPart()) break;
				size += prt.markerPool().size();
			}

			size += id() + 1;

			return size;
		}
		auto Marker::saveXml(aris::core::XmlElement &xml_ele) const->void
		{
			DynEle::saveXml(xml_ele);
			double pe[6];
			s_pm2pe(*prtPm(), pe);
			xml_ele.SetAttribute("pe", core::Matrix(1, 6, pe).toString().c_str());
		}
		auto Marker::saveAdams(std::ofstream &file) const->void
		{
			double pe[6];

			s_pm2pe(*prtPm(), pe, "313");
			core::Matrix ori(1, 3, &pe[3]), loc(1, 3, &pe[0]);

			file << "marker create  &\r\n"
				<< "marker_name = ." << model().name() << "." << fatherPart().name() << "." << this->name() << "  &\r\n"
				<< "adams_id = " << adamsID() << "  &\r\n"
				<< "location = (" << loc.toString() << ")  &\r\n"
				<< "orientation = (" << ori.toString() << ")\r\n"
				<< "!\r\n";
		}
		auto Marker::fatherPart() const->const Part&{ return static_cast<const Part &>(this->father().father()); }
		auto Marker::fatherPart()->Part& { return static_cast<Part &>(this->father().father()); }
		auto Marker::glbPm()const->const double4x4&{ s_pm_dot_pm(*fatherPart().pm(), *prtPm(), const_cast<double*>(*imp_->pm_)); return imp_->pm_; }
		auto Marker::glbVs()const->const double6& { return fatherPart().glbVs(); }
		auto Marker::glbAs()const->const double6& { return fatherPart().glbAs(); }
		auto Marker::prtPm()const->const double4x4&{ return imp_->prt_pm_; }
		auto Marker::prtVs()const->const double6&{ return fatherPart().prtVs(); }
		auto Marker::prtAs()const->const double6&{ return fatherPart().prtAs(); }
		auto Marker::updGlbPm()->void { s_pm_dot_pm(*fatherPart().pm(), *prtPm(), *imp_->pm_); }
		Marker::~Marker() = default;
		Marker::Marker(const std::string &name, const double *prt_pm, Marker *relative_mak, bool active): Coordinate(name, active)
		{
			static const double default_pm_in[16] = { 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
			prt_pm = prt_pm ? prt_pm : default_pm_in;

			if (relative_mak)
			{
				if (&relative_mak->fatherPart() != &fatherPart())throw std::logic_error("relative marker must has same father part with this marker");
				s_pm_dot_pm(*relative_mak->prtPm(), prt_pm, *imp_->prt_pm_);
			}
			else
			{
				std::copy_n(prt_pm, 16, static_cast<double *>(*imp_->prt_pm_));
			}
		}
		Marker::Marker(Object &father, const aris::core::XmlElement &xml_ele): Coordinate(father, xml_ele)
		{
			double pm[16];

			s_pe2pm(attributeMatrix(xml_ele, "pe", 1, 6).data(), pm);

			if (xml_ele.Attribute("relative_to"))
			{
				try { s_pm_dot_pm(*static_cast<aris::core::ObjectPool<Marker, Element>&>(this->father()).findByName(xml_ele.Attribute("relative_to"))->prtPm(), pm, *imp_->prt_pm_); }
				catch (std::exception &) { throw std::runtime_error(std::string("can't find relative marker for element \"") + this->name() + "\""); }
			}
			else
			{
				std::copy_n(pm, 16, static_cast<double*>(*imp_->prt_pm_));
			}
		}
		Marker::Marker(const Marker&) = default;
		Marker::Marker(Marker&&) = default;
		Marker& Marker::operator=(const Marker&) = default;
		Marker& Marker::operator=(Marker&&) = default;

		struct Part::Imp
		{
			aris::core::ObjectPool<Marker, Element> *marker_pool_;
			
			double prt_is_[6][6]{ { 0 } };
			double prt_gr_[6]{ 0 };
			double prt_as_[6]{ 0 };
			double prt_vs_[6]{ 0 };
			double prt_fg_[6]{ 0 };
			double prt_fv_[6]{ 0 };
			
			double glb_pm_[4][4]{ { 0 } };
			double glb_vs_[6]{ 0 };
			double glb_as_[6]{ 0 };
			double glb_is_[6][6]{ { 0 } };
			double glb_fg_[6]{ 0 };
			double glb_fv_[6]{ 0 };
			int row_id_;
			std::string graphic_file_path_;
		};
		auto Part::adamsID()const->std::size_t { return (this == &this->model().ground()) ? 1 : id() + (this->model().ground().id() < id() ? 1 : 2); }
		auto Part::saveXml(aris::core::XmlElement &xml_ele) const->void
		{
			DynEle::saveXml(xml_ele);

			double pe[6];
			s_pm2pe(*pm(), pe);
			xml_ele.SetAttribute("pe", core::Matrix(1, 6, pe).toString().c_str());
			xml_ele.SetAttribute("vel", core::Matrix(1, 6, vs()).toString().c_str());
			xml_ele.SetAttribute("acc", core::Matrix(1, 6, as()).toString().c_str());

			double iv[10];
			s_is2iv(*this->prtIs(), iv);
			xml_ele.SetAttribute("inertia", core::Matrix(1, 10, iv).toString().c_str());
			xml_ele.SetAttribute("graphic_file_path", imp_->graphic_file_path_.c_str());

			auto child_mak_group = xml_ele.GetDocument()->NewElement("ChildMarker");
			xml_ele.InsertEndChild(child_mak_group);
			markerPool().saveXml(*child_mak_group);
		}
		auto Part::saveAdams(std::ofstream &file) const->void
		{
			if (this == &model().ground())
			{
				file << "!----------------------------------- ground -----------------------------------!\r\n"
					<< "!\r\n"
					<< "!\r\n"
					<< "! ****** Ground Part ******\r\n"
					<< "!\r\n"
					<< "defaults model  &\r\n"
					<< "    part_name = ground\r\n"
					<< "!\r\n"
					<< "defaults coordinate_system  &\r\n"
					<< "    default_coordinate_system = ." << model().name() << ".ground\r\n"
					<< "!\r\n"
					<< "! ****** Markers for current part ******\r\n"
					<< "!\r\n";
			}
			else
			{
				double pe[6];
				s_pm2pe(*this->pm(), pe, "313");
				core::Matrix ori(1, 3, &pe[3]), loc(1, 3, &pe[0]);

				file << "!----------------------------------- " << this->name() << " -----------------------------------!\r\n"
					<< "!\r\n"
					<< "!\r\n"
					<< "defaults coordinate_system  &\r\n"
					<< "    default_coordinate_system = ." << model().name() << ".ground\r\n"
					<< "!\r\n"
					<< "part create rigid_body name_and_position  &\r\n"
					<< "    part_name = ." << model().name() << "." << this->name() << "  &\r\n"
					<< "    adams_id = " << this->adamsID() << "  &\r\n"
					<< "    location = (" << loc.toString() << ")  &\r\n"
					<< "    orientation = (" << ori.toString() << ")\r\n"
					<< "!\r\n"
					<< "defaults coordinate_system  &\r\n"
					<< "    default_coordinate_system = ." << model().name() << "." << this->name() << " \r\n"
					<< "!\r\n";


				double mass = this->prtIs()[0][0] == 0 ? 1 : prtIs()[0][0];
				std::fill_n(pe, 6, 0);
				pe[0] = this->prtIs()[1][5] / mass;
				pe[1] = -this->prtIs()[0][5] / mass;
				pe[2] = this->prtIs()[0][4] / mass;

				file << "! ****** cm and mass for current part ******\r\n"
					<< "marker create  &\r\n"
					<< "    marker_name = ." << model().name() << "." << this->name() << ".cm  &\r\n"
					<< "    adams_id = " << adamsID() + model().markerSize() << "  &\r\n"
					<< "    location = ({" << pe[0] << "," << pe[1] << "," << pe[2] << "})  &\r\n"
					<< "    orientation = (" << "{0,0,0}" << ")\r\n"
					<< "!\r\n";

				double pm[16];
				double im[6][6];

				pe[0] = -pe[0];
				pe[1] = -pe[1];
				pe[2] = -pe[2];

				s_pe2pm(pe, pm);
				s_is2is(pm, *this->prtIs(), *im);

				//！注意！//
				//Adams里对惯量矩阵的定义貌似和我自己的定义在Ixy,Ixz,Iyz上互为相反数。别问我为什么,我也不知道。
				file << "part create rigid_body mass_properties  &\r\n"
					<< "    part_name = ." << model().name() << "." << this->name() << "  &\r\n"
					<< "    mass = " << this->prtIs()[0][0] << "  &\r\n"
					<< "    center_of_mass_marker = ." << model().name() << "." << this->name() << ".cm  &\r\n"
					<< "    inertia_marker = ." << model().name() << "." << this->name() << ".cm  &\r\n"
					<< "    ixx = " << im[3][3] << "  &\r\n"
					<< "    iyy = " << im[4][4] << "  &\r\n"
					<< "    izz = " << im[5][5] << "  &\r\n"
					<< "    ixy = " << -im[4][3] << "  &\r\n"
					<< "    izx = " << -im[5][3] << "  &\r\n"
					<< "    iyz = " << -im[5][4] << "\r\n"
					<< "!\r\n";
			}

			//导入marker
			this->markerPool().saveAdams(file);

			//导入parasolid
			std::stringstream stream(imp_->graphic_file_path_);
			std::string path;
			while (stream >> path)
			{
				file << "file parasolid read &\r\n"
					<< "file_name = \"" << path << "\" &\r\n"
					<< "type = ASCII" << " &\r\n"
					<< "part_name = " << this->name() << " \r\n"
					<< "\r\n";
			}
		}
		auto Part::rowID()const->std::size_t { return imp_->row_id_; }
		auto Part::markerPool()->aris::core::ObjectPool<Marker, Element>& { return std::ref(*imp_->marker_pool_); }
		auto Part::markerPool()const->const aris::core::ObjectPool<Marker, Element>& { return std::ref(*imp_->marker_pool_); }
		auto Part::cptGlbIm(double *im, int ld)const->void
		{
			ld = std::max(ld, 6);
			
			double tem[36], tem2[36];
			s_tf_n(6, *pm(), *prtIs(), tem);
			s_transpose(6, 6, tem, tem2);
			s_tf_n(6, *pm(), tem2, 6, im, ld);
		}
		auto Part::cptGlbFg(double *fg)const->void
		{
			double prt_gr[3], prt_fg[6];
			s_inv_pm_dot_v3(*pm(), model().environment().gravity(), prt_gr);
			s_mm(6, 1, 3, *prtIs(), 6, prt_gr, 1, prt_fg, 1);
			s_tf(*pm(), prt_fg, fg);
		}
		auto Part::cptGlbFv(double *fv)const->void
		{
			double prt_vs[6], prt_fv[6], tem[6];
			s_inv_tv(*pm(), vs(), prt_vs);
			s_mm(6, 1, 6, *prtIs(), prt_vs, tem);
			s_cf(prt_vs, tem, prt_fv);
			s_tf(*pm(), prt_fv, fv);
		}
		auto Part::cptPrtFg(double *fg)const->void
		{
			double prt_gr[3];
			s_inv_pm_dot_v3(*pm(), model().environment().gravity(), prt_gr);
			s_mm(6, 1, 3, *prtIs(), 6, prt_gr, 1, fg, 1);
		}
		auto Part::cptPrtFv(double *fv)const->void
		{
			double prt_vs[6], tem[6];
			s_inv_tv(*pm(), vs(), prt_vs);
			s_mm(6, 1, 6, *prtIs(), prt_vs, tem);
			s_cf(prt_vs, tem, fv);
		}
		auto Part::cptPrtVs(double *prt_vs)const->void
		{
			s_inv_tv(*pm(), vs(), prt_vs);
		}
		auto Part::cptPrtAs(double *prt_as)const->void
		{
			s_inv_tv(*pm(), as(), prt_as);
		}
		auto Part::prtIs()const->const double6x6&{ return imp_->prt_is_; }
		auto Part::prtGr()const->const double6&{ return imp_->prt_gr_; }
		auto Part::prtFg()const->const double6&{ return imp_->prt_fg_; }
		auto Part::prtFv()const->const double6&{ return imp_->prt_fv_; }
		auto Part::updPrtVs()->void { s_inv_tv(*pm(), vs(), imp_->prt_vs_); }
		auto Part::updPrtAs()->void { s_inv_tv(*pm(), as(), imp_->prt_as_); }
		auto Part::updPrtGr()->void { s_inv_pm_dot_v3(*pm(), model().environment().gravity(), imp_->prt_gr_); }
		auto Part::updPrtFg()->void 
		{ 
			double prt_gr[3];
			s_inv_pm_dot_v3(*pm(), model().environment().gravity(), prt_gr);
			s_mm(6, 1, 3, *prtIs(), 6, prt_gr, 1, imp_->prt_fg_, 1);
		}
		auto Part::updPrtFv()->void 
		{
			double prt_vs[6], tem[6];
			s_inv_tv(*pm(), vs(), prt_vs);
			s_mm(6, 1, 6, *prtIs(), prt_vs, tem);
			s_cf(prt_vs, tem, imp_->prt_fv_);
		}
		auto Part::glbIs()const->const double6x6&{ return imp_->glb_is_; }
		auto Part::glbFg()const->const double6&{ return imp_->glb_fg_; }
		auto Part::glbFv()const->const double6&{ return imp_->glb_fv_; }
		auto Part::updGlbIs()->void
		{
			double tem[36], tem2[36];
			s_tf_n(6, *pm(), *prtIs(), tem);
			s_transpose(6, 6, tem, tem2);
			s_tf_n(6, *pm(), tem2, *imp_->glb_is_);
		}
		auto Part::updGlbFg()->void 
		{ 
			double prt_gr[3], prt_fg[6];
			s_inv_pm_dot_v3(*pm(), model().environment().gravity(), prt_gr);
			s_mm(6, 1, 3, *prtIs(), 6, prt_gr, 1, prt_fg, 1);
			s_tf(*pm(), prt_fg, imp_->glb_fg_);
		}
		auto Part::updGlbFv()->void
		{
			double prt_vs[6], prt_fv[6], tem[6];
			s_inv_tv(*pm(), vs(), prt_vs);
			s_mm(6, 1, 6, *prtIs(), prt_vs, tem);
			s_cf(prt_vs, tem, prt_fv);
			s_tf(*pm(), prt_fv, imp_->glb_fv_);
		}
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
		auto Part::setRm(const double *rm, int rm_ld)->void { if (rm)s_rm2pm(rm, *imp_->glb_pm_, rm_ld); }
		auto Part::setRm(const Coordinate &relative_to, const double *rm, int rm_ld)->void{if (rm) s_rm2rm(*relative_to.pm(), rm, *imp_->glb_pm_, rm_ld, 4);}
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
		auto Part::setPq(const double *pq)->void { if(pq)s_pq2pm(pq, static_cast<double*>(*imp_->glb_pm_));}
		auto Part::setPq(const Coordinate &relative_to, const double *pq)->void 
		{ 
			if (pq)
			{
				double pm[16];
				s_pq2pm(pq, pm);
				setPm(relative_to, pm);
			}
		}
		auto Part::setPm(const double *pm)->void { if(pm)std::copy(pm, pm + 16, static_cast<double*>(*imp_->glb_pm_)); }
		auto Part::setPm(const Coordinate &relative_to, const double *pm)->void { if(pm)s_pm2pm(*relative_to.pm(), pm, *imp_->glb_pm_);}
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
		auto Part::setWm(const double *wm_in, const double *rm_in, int wm_ld, int rm_ld)->void
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
		auto Part::setWm(const Coordinate &relative_to, const double *wm_in, const double *rm_in, int wm_ld, int rm_ld)->void
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
		auto Part::setWa(const double *wa_in, const double *rm_in, int rm_ld)->void
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
		auto Part::setWa(const Coordinate &relative_to, const double *wa_in, const double *rm_in, int rm_ld)->void
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
			if (pq_in) std::copy(pq_in, pq_in + 7, pq);else getPq(relative_to, pq);
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
		auto Part::setXm(const double *xm_in, const double *wm_in, const double *rm_in, int xm_ld, int wm_ld, int rm_ld)->void
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
		auto Part::setXm(const Coordinate &relative_to, const double *xm_in, const double *wm_in, const double *rm_in, int xm_ld, int wm_ld, int rm_ld)->void
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
		auto Part::setXa(const double *xa_in, const double *wa_in, const double *rm_in, int rm_ld)->void
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
		auto Part::setXa(const Coordinate &relative_to, const double *xa_in, const double *wa_in, const double *rm_in, int rm_ld)->void
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
		auto Part::operator=(Part &&other)->Part&
		{
			Coordinate::operator=(other);
			imp_ = other.imp_;
			imp_->marker_pool_ = &static_cast<aris::core::ObjectPool<Marker, Element> &>(*findByName("marker_pool"));
			return *this;
		}
		auto Part::operator=(const Part &other)->Part&
		{
			Coordinate::operator=(other);
			imp_ = other.imp_;
			imp_->marker_pool_ = &static_cast<aris::core::ObjectPool<Marker, Element> &>(*findByName("marker_pool"));
			return *this;
		}
		Part::~Part() = default;
		Part::Part(const std::string &name, const double *im, const double *pm, const double *vs, const double *as, bool active)
			: Coordinate(name, active)
		{
			imp_->marker_pool_ = &add<aris::core::ObjectPool<Marker, Element> >("marker_pool");

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

			std::copy_n(im, 36, static_cast<double *>(*imp_->prt_is_));
			setPm(pm);
			setVs(vs);
			setAs(as);
		}
		Part::Part(Object &father, const aris::core::XmlElement &xml_ele): Coordinate(father, xml_ele)
		{
			s_pe2pm(attributeMatrix(xml_ele, "pe", 1, 6).data(), *imp_->glb_pm_);
			std::copy_n(attributeMatrix(xml_ele, "vel", 1, 6).data(), 6, imp_->glb_vs_);
			std::copy_n(attributeMatrix(xml_ele, "acc", 1, 6).data(), 6, imp_->glb_as_);
			s_iv2is(attributeMatrix(xml_ele, "inertia", 1, 10).data(), *imp_->prt_is_);
			
			if (xml_ele.Attribute("graphic_file_path"))
				imp_->graphic_file_path_ = model().calculator().evaluateExpression(xml_ele.Attribute("graphic_file_path"));

			imp_->marker_pool_ = findOrInsert<aris::core::ObjectPool<Marker, Element> >("marker_pool");
		}
		Part::Part(Part &&other) :Coordinate(std::move(other)), imp_(std::move(other.imp_)) { imp_->marker_pool_ = findType<aris::core::ObjectPool<Marker, Element> >("marker_pool"); };
		Part::Part(const Part &other) :Coordinate(other), imp_(other.imp_) { imp_->marker_pool_ = findType<aris::core::ObjectPool<Marker, Element> >("marker_pool"); };

		auto Joint::saveAdams(std::ofstream &file) const->void
		{
			file << "constraint create joint " << this->adamsType() << "  &\r\n"
				<< "    joint_name = ." << model().name() << "." << this->name() << "  &\r\n"
				<< "    adams_id = " << adamsID() << "  &\r\n"
				<< "    i_marker_name = ." << model().name() << "." << this->makI().fatherPart().name() << "." << this->makI().name() << "  &\r\n"
				<< "    j_marker_name = ." << model().name() << "." << this->makJ().fatherPart().name() << "." << this->makJ().name() << "  \r\n"
				<< "!\r\n";
		}
		Joint::Joint(Joint &&other) = default;
		Joint::Joint(const Joint &other) = default;
		Joint& Joint::operator=(const Joint &other) = default;
		Joint& Joint::operator=(Joint &&other) = default;

		struct Motion::Imp
		{
			int component_axis_;
			double mot_fce_{ 0 };//仅仅用于标定
			double frc_coe_[3]{ 0 };
			double mot_pos_{ 0 }, mot_vel_{ 0 }, mot_acc_{ 0 }, mot_fce_dyn_{ 0 };
			std::size_t sla_id_{ 0 }, phy_id_{0};
		};
		auto Motion::saveXml(aris::core::XmlElement &xml_ele) const->void
		{
			Constraint::saveXml(xml_ele);

			xml_ele.SetAttribute("frc_coe", core::Matrix(1, 3, this->frcCoe()).toString().c_str());
			xml_ele.SetAttribute("component", axis());
		}
		auto Motion::saveAdams(std::ofstream &file) const->void
		{
			std::string s;
			std::stringstream mot_func;

			switch (axis())
			{
			case 0:
				s = "x";
				break;
			case 1:
				s = "y";
				break;
			case 2:
				s = "z";
				break;
			case 3:
				s = "B1";
				break;
			case 4:
				s = "B2";
				break;
			case 5:
				s = "B3";
				break;
			}

			if (model().akimaPool().findByName(this->name() + "_akima") == model().akimaPool().end())
			{
				mot_func << this->mp() << " + " << this->mv() << " * time + " << this->ma()*0.5 << " * time * time";
			}
			else
			{
				mot_func << "AKISPL(time,0," << this->name() << "_akima)";
			}

			file << "constraint create motion_generator &\r\n"
				<< "    motion_name = ." << model().name() << "." << this->name() << "  &\r\n"
				<< "    adams_id = " << adamsID() << "  &\r\n"
				<< "    i_marker_name = ." << model().name() << "." << this->makI().fatherPart().name() << "." << this->makI().name() << "  &\r\n"
				<< "    j_marker_name = ." << model().name() << "." << this->makJ().fatherPart().name() << "." << this->makJ().name() << "  &\r\n"
				<< "    axis = " << s << "  &\r\n"
				<< "    function = \"" << mot_func.str() << "\"  \r\n"
				<< "!\r\n";

		}
		auto Motion::cptCp(double *cp)const->void
		{
			Constraint::cptCp(cp);
			cp[0] += mp();
		}
		auto Motion::cptCv(double *cv)const->void
		{
			Constraint::cptCp(cv);
			cv[0] += mv();
		}
		auto Motion::cptCa(double *ca)const->void
		{
			Constraint::cptCa(ca);
			ca[0] += ma();
		}
		
		auto Motion::updCa()->void
		{
			Constraint::updCa();
			ca_[0] += ma();
		}
		auto Motion::updCe()->void
		{
			Constraint::updCe();
			const_cast<double*>(cePtr())[0] += mp();
		}
		auto Motion::updMp()->void
		{
			makI().updGlbPm();
			makJ().updGlbPm();
			setMp(s_sov_axis_distance(*makJ().pm(), *makI().pm(), axis()));
		}
		auto Motion::updMv()->void
		{
			double vs_diff[6], vs_diff_in_J[6];
			s_vc(6, makI().vs(), vs_diff);
			s_va(6, -1.0, makJ().vs(), vs_diff);
			s_inv_tv(*makJ().pm(), vs_diff, vs_diff_in_J);
			setMv(vs_diff_in_J[axis()]);
		}
		auto Motion::absID()const->std::size_t { return id(); }
		auto Motion::slaID()const->std::size_t { return imp_->sla_id_; }
		auto Motion::phyID()const->std::size_t { return imp_->phy_id_; }
		auto Motion::axis()const->int { return imp_->component_axis_; }
		auto Motion::frcCoe()const->const double3& { return imp_->frc_coe_; }
		auto Motion::setFrcCoe(const double *frc_coe)->void { std::copy_n(frc_coe, 3, imp_->frc_coe_); }
		auto Motion::mp() const->double { return imp_->mot_pos_; }
		auto Motion::setMp(double mot_pos)->void { imp_->mot_pos_ = mot_pos; }
		auto Motion::mv() const->double { return imp_->mot_vel_; }
		auto Motion::setMv(double mot_vel)->void { imp_->mot_vel_ = mot_vel; }
		auto Motion::ma() const->double { return imp_->mot_acc_; }
		auto Motion::setMa(double mot_acc)->void { imp_->mot_acc_ = mot_acc; }
		auto Motion::mf() const->double { return mfDyn() + mfFrc(); }
		auto Motion::setMf(double mot_fce)->void { imp_->mot_fce_ = mot_fce; }
		auto Motion::mfDyn() const->double { return imp_->mot_fce_dyn_; }
		auto Motion::setMfDyn(double mot_dyn_fce)->void { imp_->mot_fce_dyn_ = mot_dyn_fce; }
		auto Motion::mfFrc() const->double { return s_sgn(imp_->mot_vel_)*frcCoe()[0] + imp_->mot_vel_*frcCoe()[1] + imp_->mot_acc_*frcCoe()[2]; }
		Motion::~Motion() = default;
		Motion::Motion(const std::string &name, Marker &makI, Marker &makJ, int component_axis, const double *frc_coe, bool active): Constraint(name, makI, makJ, active)
		{
			static const double default_frc_coe[3]{ 0,0,0 };
			frc_coe = frc_coe ? frc_coe : default_frc_coe;
			setFrcCoe(frc_coe);

			imp_->component_axis_ = component_axis;

			double loc_cst[6]{ 0,0,0,0,0,0, };
			loc_cst[axis()] = 1;
			s_tf(*this->makI().prtPm(), loc_cst, *prtCmI_);
		}
		Motion::Motion(Object &father, const aris::core::XmlElement &xml_ele) : Constraint(father, xml_ele)
		{
			setFrcCoe(attributeMatrix(xml_ele, "frc_coe", 1, 3).data());
			imp_->component_axis_ = attributeInt32(xml_ele, "component");

			double loc_cst[6]{ 0,0,0,0,0,0, };
			loc_cst[axis()] = 1;
			s_tf(*this->makI().prtPm(), loc_cst, *prtCmI_);

			imp_->sla_id_ = attributeInt32(xml_ele, "slave_id");
		}
		Motion::Motion(const Motion &other) = default;
		Motion::Motion(Motion &&other) = default;
		Motion& Motion::operator=(const Motion &other) = default;
		Motion& Motion::operator=(Motion &&other) = default;

		struct GeneralMotion::Imp
		{
			double mot_pos_[6], mot_vel_[6], mot_acc_[6], mot_fce_[6];
		};
		auto GeneralMotion::saveAdams(std::ofstream &file) const->void
		{
			file << "ude create instance  &\r\n"
				<< "    instance_name = ." << model().name() << "." << this->name() << "  &\r\n"
				<< "    definition_name = .MDI.Constraints.general_motion  &\r\n"
				<< "    location = 0.0, 0.0, 0.0  &\r\n"
				<< "    orientation = 0.0, 0.0, 0.0  \r\n"
				<< "!\r\n";

			file << "variable modify  &\r\n"
				<< "	variable_name = ." << model().name() << "." << name() << ".i_marker  &\r\n"
				<< "	object_value = ." << model().name() << "." << makI().fatherPart().name() << "." << makI().name() << " \r\n"
				<< "!\r\n";

			file << "variable modify  &\r\n"
				<< "	variable_name = ." << model().name() << "." << name() << ".j_marker  &\r\n"
				<< "	object_value = ." << model().name() << "." << makJ().fatherPart().name() << "." << makJ().name() << " \r\n"
				<< "!\r\n";

			std::string names[6]{ "t1","t2", "t3", "r1", "r2", "r3" };

			for (std::size_t i = 0; i < 6; ++i)
			{
				std::string akima = name() + "_" + names[i] + "_akima";
				std::string func = model().akimaPool().findByName(akima) != model().akimaPool().end() ? "AKISPL(time,0," + akima + ")" : std::to_string(0);

				file << "variable modify  &\r\n"
					<< "	variable_name = ." << model().name() << "." << name() << "." << names[i] << "_type  &\r\n"
					<< "	integer_value = 1 \r\n"
					<< "!\r\n";

				file << "variable modify  &\r\n"
					<< "	variable_name = ." << model().name() << "." << name() << "." << names[i] << "_func  &\r\n"
					<< "	string_value = \"" + func + "\" \r\n"
					<< "!\r\n";

				file << "variable modify  &\r\n"
					<< "	variable_name = ." << model().name() << "." << name() << "." << names[i] << "_ic_disp  &\r\n"
					<< "	real_value = 0.0 \r\n"
					<< "!\r\n";

				file << "variable modify  &\r\n"
					<< "	variable_name = ." << model().name() << "." << name() << "." << names[i] << "_ic_velo  &\r\n"
					<< "	real_value = 0.0 \r\n"
					<< "!\r\n";
			}

			file << "ude modify instance  &\r\n"
				<< "instance_name = ." << model().name() << "." << name() << "\r\n"
				<< "!\r\n";
		}
		auto GeneralMotion::updCa()->void
		{
			Constraint::updCa();
			s_va(dim(), ma(), ca_);
		}
		auto GeneralMotion::updMp()->void
		{
			makI().updGlbPm();
			makJ().updGlbPm();

			double pm_I2J[4][4], pe[6];
			s_inv_pm_dot_pm(*makJ().pm(), *makI().pm(), *pm_I2J);
			s_pm2pe(&pm_I2J[0][0], pe, "123");
			setMp(pe);
		}
		auto GeneralMotion::updMv()->void
		{
			double velDiff[6], velDiff_in_J[6];
			std::copy_n(makI().vs(), 6, velDiff);
			s_va(6, -1.0, makJ().vs(), velDiff);
			s_inv_tv(*makJ().pm(), velDiff, velDiff_in_J);
			setMv(velDiff_in_J);
		}
		auto GeneralMotion::mp() const->const double6&{ return imp_->mot_pos_; }
		auto GeneralMotion::setMp(const double *mot_pos)->void { std::copy(mot_pos, mot_pos + 6, imp_->mot_pos_); }
		auto GeneralMotion::mv() const->const double6&{ return imp_->mot_vel_; }
		auto GeneralMotion::setMv(const double * mot_vel)->void { std::copy(mot_vel, mot_vel + 6, imp_->mot_vel_); }
		auto GeneralMotion::ma() const->const double6&{ return imp_->mot_acc_; }
		auto GeneralMotion::setMa(const double * mot_acc)->void { std::copy(mot_acc, mot_acc + 6, imp_->mot_acc_); }
		auto GeneralMotion::mf() const->const double6&{ return imp_->mot_fce_; }
		auto GeneralMotion::setMf(const double * mot_fce)->void { std::copy(mot_fce, mot_fce + 6, imp_->mot_fce_); }
		GeneralMotion::~GeneralMotion() = default;
		GeneralMotion::GeneralMotion(const std::string &name, Marker &makI, Marker &makJ, const std::string& freedom, bool active):Constraint(name, makI, makJ, active){}
		GeneralMotion::GeneralMotion(Object &father, const aris::core::XmlElement &xml_ele):Constraint(father, xml_ele){}
		GeneralMotion::GeneralMotion(const GeneralMotion &other) = default;
		GeneralMotion::GeneralMotion(GeneralMotion &&other) = default;
		GeneralMotion& GeneralMotion::operator=(const GeneralMotion &other) = default;
		GeneralMotion& GeneralMotion::operator=(GeneralMotion &&other) = default;

		struct Model::Imp
		{
			aris::core::Calculator calculator_;
			Environment *environment_;
			aris::core::ObjectPool<Script, Element> *script_pool_;
			aris::core::ObjectPool<Variable, Element> *variable_pool_;
			aris::core::ObjectPool<Akima, Element> *akima_pool_;
			aris::core::ObjectPool<Part, Element> *part_pool_;
			aris::core::ObjectPool<Joint, Element> *joint_pool_;
			aris::core::ObjectPool<Motion, Element> *motion_pool_;
			aris::core::ObjectPool<GeneralMotion, Element> *general_motion_pool_;
			aris::core::ObjectPool<Force, Element> *force_pool_;
			Part* ground_;

			std::vector<std::size_t> mot_vec_phy2abs_, mot_vec_sla2abs_;

			std::size_t dyn_cst_dim_, dyn_prt_dim_, active_cst_num_, active_prt_num_;
			std::unique_ptr<double[]> dyn_A_, dyn_b_, dyn_x_;
			std::size_t clb_dim_m_, clb_dim_n_, clb_dim_gam_, clb_dim_frc_;
			
			aris::core::RefPool<Part> active_part_pool_;
			aris::core::RefPool<Constraint> active_constraint_pool_;
			BlockSize p_size_, c_size_;
			BlockMatrix im_, cm_, cp_, cv_, ca_, pp_, pv_, pa_;
			BlockMatrix cct_, ctc_;
			BlockMatrix cct_llt_, cct_x_, cct_b_, ctc_llt_, ctc_x_, ctc_b_;

			std::function<void(int dim, const double *D, const double *b, double *x)> dyn_solve_method_{ nullptr };
			std::function<void(int n, double *A)> clb_inverse_method_{ nullptr };
		};
		auto Model::SimResult::clear()->void
		{
			time_.clear();
			Pin_.clear();
			Fin_.clear();
			Vin_.clear();
			Ain_.clear();
		}
		auto Model::SimResult::resize(std::size_t size)->void
		{
			clear();

			Pin_.resize(size);
			Fin_.resize(size);
			Vin_.resize(size);
			Ain_.resize(size);
		}
		auto Model::SimResult::saveToTxt(const std::string &filename)const->void
		{
			auto f_name = filename + "_Fin.txt";
			auto p_name = filename + "_Pin.txt";
			auto v_name = filename + "_Vin.txt";
			auto a_name = filename + "_Ain.txt";

			dlmwrite(f_name.c_str(), Fin_);
			dlmwrite(p_name.c_str(), Pin_);
			dlmwrite(v_name.c_str(), Vin_);
			dlmwrite(a_name.c_str(), Ain_);
		}
		auto Model::loadDynEle(const std::string &name)->void
		{
			partPool().load(name);
			jointPool().load(name);
			motionPool().load(name);
			forcePool().load(name);
		}
		auto Model::saveDynEle(const std::string &name)->void
		{
			partPool().save(name);
			jointPool().save(name);
			motionPool().save(name);
			forcePool().save(name);
		}
		auto Model::loadXml(const aris::core::XmlDocument &xml_doc)->void
        {
            auto model_xml_ele = xml_doc.RootElement()->FirstChildElement("model");

            if (!model_xml_ele)throw std::runtime_error("can't find \"model\" element in xml file");

            loadXml(*model_xml_ele);
		}
        auto Model::loadXml(const aris::core::XmlElement &xml_ele)->void
		{
            Root::loadXml(xml_ele);

			imp_->environment_ = findOrInsert<Environment>("environment");
			imp_->script_pool_ = findOrInsert<aris::core::ObjectPool<Script, Element>>("script_pool");
			imp_->variable_pool_ = findOrInsert<aris::core::ObjectPool<Variable, Element>>("variable_pool");
			imp_->akima_pool_ = findOrInsert<aris::core::ObjectPool<Akima, Element>>("akima_pool");
			imp_->part_pool_ = findOrInsert<aris::core::ObjectPool<Part, Element>>("part_pool");
			imp_->joint_pool_ = findOrInsert<aris::core::ObjectPool<Joint, Element>>("joint_pool");
			imp_->motion_pool_ = findOrInsert<aris::core::ObjectPool<Motion, Element>>("motion_pool");
			imp_->general_motion_pool_ = findOrInsert<aris::core::ObjectPool<GeneralMotion, Element>>("general_motion_pool");
			imp_->force_pool_ = findOrInsert<aris::core::ObjectPool<Force, Element>>("force_pool");
			imp_->ground_ = partPool().findOrInsert<Part>("ground");

			updMotionID();
        }
		auto Model::saveXml(aris::core::XmlDocument &xml_doc)const->void
		{
			xml_doc.DeleteChildren();

			auto header_xml_ele = xml_doc.NewDeclaration("xml version=\"1.0\" encoding=\"UTF-8\" ");
			xml_doc.InsertEndChild(header_xml_ele);

			auto root_xml_ele = xml_doc.NewElement("Root");
			xml_doc.InsertEndChild(root_xml_ele);

			auto model_xml_ele = xml_doc.NewElement("Model");
			root_xml_ele->InsertEndChild(model_xml_ele);
			saveXml(*model_xml_ele);
		}
		auto Model::saveAdams(const std::string &filename, bool using_script) const->void
		{
			std::string filename_ = filename;
			if (filename_.size() < 4 || filename_.substr(filename.size() - 4, 4) != ".cmd")
			{
				filename_ += ".cmd";
			}

			std::ofstream file;
			file.open(filename_, std::ios::out | std::ios::trunc);

			saveAdams(file, using_script);

			file.close();
		}
		auto Model::saveAdams(std::ofstream &file, bool using_script) const->void
		{
			file << std::setprecision(15);
			for (auto &ele : children()) 
			{
				static_cast<const Element &>(ele).saveAdams(file);
			}
			if (!using_script)
			{
				file << "!----------------------------------- Motify Active -------------------------------------!\r\n!\r\n!\r\n";
				for (auto &prt : partPool())
				{
					if ((&prt != &ground()) && (!prt.active()))
					{
						file << "part attributes  &\r\n"
							<< "    part_name = ." << name() << "." << prt.name() << "  &\r\n"
							<< "    active = off \r\n!\r\n";
					}
				}
				for (auto &jnt : jointPool())
				{
					if (!jnt.active())
					{
						file << "constraint attributes  &\r\n"
							<< "    constraint_name = ." << name() << "." << jnt.name() << "  &\r\n"
							<< "    active = off \r\n!\r\n";
					}
				}
				for (auto &mot : motionPool())
				{
					if (!mot.active())
					{
						file << "constraint attributes  &\r\n"
							<< "    constraint_name = ." << name() << "." << mot.name() << "  &\r\n"
							<< "    active = off \r\n!\r\n";
					}

				}
				for (auto &fce : forcePool())
				{
					if (!fce.active())
					{
						file << "force attributes  &\r\n"
							<< "    force_name = ." << name() << "." << fce.name() << "  &\r\n"
							<< "    active = off \r\n!\r\n";
					}
				}
			}
		}
		auto Model::calculator()->aris::core::Calculator& { return std::ref(imp_->calculator_); }
		auto Model::calculator()const ->const aris::core::Calculator&{ return std::ref(imp_->calculator_); }
		auto Model::environment()->aris::dynamic::Environment& { return std::ref(*imp_->environment_); }
		auto Model::environment()const ->const aris::dynamic::Environment&{ return std::ref(*imp_->environment_); }
		auto Model::scriptPool()->aris::core::ObjectPool<Script, Element>& { return std::ref(*imp_->script_pool_); }
		auto Model::scriptPool()const->const aris::core::ObjectPool<Script, Element>&{ return std::ref(*imp_->script_pool_); }
		auto Model::variablePool()->aris::core::ObjectPool<Variable, Element>& { return std::ref(*imp_->variable_pool_); }
		auto Model::variablePool()const->const aris::core::ObjectPool<Variable, Element>& { return std::ref(*imp_->variable_pool_); }
		auto Model::akimaPool()->aris::core::ObjectPool<Akima, Element>& { return std::ref(*imp_->akima_pool_); }
		auto Model::akimaPool()const->const aris::core::ObjectPool<Akima, Element>& { return std::ref(*imp_->akima_pool_); }
		auto Model::partPool()->aris::core::ObjectPool<Part, Element>& { return std::ref(*imp_->part_pool_); }
		auto Model::partPool()const->const aris::core::ObjectPool<Part, Element>& { return std::ref(*imp_->part_pool_); }
		auto Model::jointPool()->aris::core::ObjectPool<Joint, Element>& { return std::ref(*imp_->joint_pool_); }
		auto Model::jointPool()const->const aris::core::ObjectPool<Joint, Element>& { return std::ref(*imp_->joint_pool_); }
		auto Model::motionPool()->aris::core::ObjectPool<Motion, Element>& { return std::ref(*imp_->motion_pool_); }
		auto Model::motionPool()const->const aris::core::ObjectPool<Motion, Element>& { return std::ref(*imp_->motion_pool_); }
		auto Model::generalMotionPool()->aris::core::ObjectPool<GeneralMotion, Element>& { return std::ref(*imp_->general_motion_pool_); }
		auto Model::generalMotionPool()const->const aris::core::ObjectPool<GeneralMotion, Element>& { return std::ref(*imp_->general_motion_pool_); }
		auto Model::forcePool()->aris::core::ObjectPool<Force, Element>& { return std::ref(*imp_->force_pool_); }
		auto Model::forcePool()const->const aris::core::ObjectPool<Force, Element>& { return std::ref(*imp_->force_pool_); }
		auto Model::updMotionID()->void
		{
			// 更新 mot_vec_sla2abs_
			for (auto &mot : motionPool())
			{
				imp_->mot_vec_sla2abs_.resize(std::max(mot.slaID() + 1, imp_->mot_vec_sla2abs_.size()), std::numeric_limits<std::size_t>::max());
				if (imp_->mot_vec_sla2abs_.at(mot.slaID()) != std::numeric_limits<std::size_t>::max()) throw std::runtime_error("invalid model xml:\"slave_id\" of motion \"" + name() + "\" already exists");
				imp_->mot_vec_sla2abs_.at(mot.slaID()) = mot.id();
			}
			
			// 更新 mot_vec_phy2abs_
			imp_->mot_vec_phy2abs_.clear();
			for (auto id : imp_->mot_vec_sla2abs_)if (id != std::numeric_limits<std::size_t>::max())imp_->mot_vec_phy2abs_.push_back(id);
			
			// 更新 motion的phy_id
			for (std::size_t phy_id = 0; phy_id < imp_->mot_vec_phy2abs_.size(); ++phy_id)
			{
				std::size_t abs_id = imp_->mot_vec_phy2abs_.at(phy_id);
				motionPool().at(abs_id).imp_->phy_id_ = phy_id;
			}
		}
		auto Model::motionAtAbs(std::size_t abs_id)->Motion& { return motionPool().at(abs_id); }
		auto Model::motionAtAbs(std::size_t abs_id)const->const Motion&{ return motionPool().at(abs_id); }
		auto Model::motionAtPhy(std::size_t phy_id)->Motion& { return motionPool().at(imp_->mot_vec_phy2abs_.at(phy_id));}
		auto Model::motionAtPhy(std::size_t phy_id)const->const Motion&{ return motionPool().at(imp_->mot_vec_phy2abs_.at(phy_id)); }
		auto Model::motionAtSla(std::size_t sla_id)->Motion& { return motionPool().at(imp_->mot_vec_sla2abs_.at(sla_id)); }
		auto Model::motionAtSla(std::size_t sla_id)const->const Motion&{ return motionPool().at(imp_->mot_vec_sla2abs_.at(sla_id)); }
		auto Model::ground()->Part& { return std::ref(*imp_->ground_); }
		auto Model::ground()const->const Part&{ return std::ref(*imp_->ground_); }
		
		auto Model::allocateMemory()->void
		{
			// compute memory size //
			imp_->p_size_.resize(0);
			imp_->c_size_.resize(1, 6);
			imp_->active_part_pool_.clear();
			imp_->active_constraint_pool_.clear();

			for (auto &prt : partPool())
			{
				if (prt.active())
				{
					imp_->active_part_pool_.push_back_ptr(&prt);
					prt.Part::imp_->row_id_ = imp_->p_size_.size();
					imp_->p_size_.push_back(6);
				}
			}
			for (auto &jnt : jointPool())
			{
				if (jnt.active())
				{
					imp_->active_constraint_pool_.push_back_ptr(&jnt);
					jnt.Constraint::imp_->col_id_ = imp_->c_size_.size();
					imp_->c_size_.push_back(jnt.dim());
				}
			}
			for (auto &mot : motionPool()) 
			{
				if (mot.active()) 
				{
					imp_->active_constraint_pool_.push_back_ptr(&mot);
					mot.Constraint::imp_->col_id_ = imp_->c_size_.size();
					imp_->c_size_.push_back(mot.dim());
				} 
			}

			// allocate memory //
			aris::dynamic::s_blk_allocate(imp_->p_size_, imp_->p_size_, imp_->im_);
			aris::dynamic::s_blk_allocate(imp_->p_size_, imp_->c_size_, imp_->cm_);
			aris::dynamic::s_blk_allocate(imp_->c_size_, { 1 }, imp_->cp_);
			aris::dynamic::s_blk_allocate(imp_->c_size_, { 1 }, imp_->cv_);
			aris::dynamic::s_blk_allocate(imp_->c_size_, { 1 }, imp_->ca_);
			aris::dynamic::s_blk_allocate(imp_->p_size_, { 1 }, imp_->pp_);
			aris::dynamic::s_blk_allocate(imp_->p_size_, { 1 }, imp_->pv_);
			aris::dynamic::s_blk_allocate(imp_->p_size_, { 1 }, imp_->pa_);

			aris::dynamic::s_blk_allocate(imp_->p_size_, imp_->p_size_, imp_->cct_);
			aris::dynamic::s_blk_allocate(imp_->c_size_, imp_->c_size_, imp_->ctc_);

			aris::dynamic::s_blk_allocate(imp_->p_size_, imp_->p_size_, imp_->cct_llt_);
			aris::dynamic::s_blk_allocate(imp_->p_size_, { 1 }, imp_->cct_x_);
			aris::dynamic::s_blk_allocate(imp_->p_size_, { 1 }, imp_->cct_b_);
			aris::dynamic::s_blk_allocate(imp_->c_size_, imp_->c_size_, imp_->ctc_llt_);
			aris::dynamic::s_blk_allocate(imp_->c_size_, { 1 }, imp_->ctc_x_);
			aris::dynamic::s_blk_allocate(imp_->c_size_, { 1 }, imp_->ctc_b_);

			imp_->cm_[ground().rowID()][0].resize(36, 0);
			for (int i = 0; i < 6; ++i)imp_->cm_[ground().rowID()][0].data()[i * 6 + i] = 1.0;
		}
		auto Model::activePartPool()->aris::core::RefPool<Part>& { return imp_->active_part_pool_; }
		auto Model::activeConstraintPool()->aris::core::RefPool<Constraint>& { return imp_->active_constraint_pool_; }
		auto Model::cSize()->BlockSize& { return imp_->c_size_; }
		auto Model::pSize()->BlockSize& { return imp_->p_size_; }

		auto Model::glbIm()->BlockMatrix& { return imp_->im_; }
		auto Model::glbCm()->BlockMatrix& { return imp_->cm_; }
		auto Model::cp()->BlockMatrix& { return imp_->cp_; }
		auto Model::cv()->BlockMatrix& { return imp_->cv_; }
		auto Model::ca()->BlockMatrix& { return imp_->ca_; }

		auto Model::cptGlbIm()->void
		{
			for (auto &prt : activePartPool())
			{
				imp_->im_[prt.rowID()][prt.rowID()].resize(36, 0);
				prt.cptGlbIm(imp_->im_[prt.rowID()][prt.rowID()].data());
			}
		}
		auto Model::cptGlbCm()->void
		{
			for (auto &cst : activeConstraintPool())
			{
				imp_->cm_[cst.makI().fatherPart().rowID()][cst.colID()].resize(6 * cst.dim());
				imp_->cm_[cst.makJ().fatherPart().rowID()][cst.colID()].resize(6 * cst.dim());
				cst.cptGlbCm(imp_->cm_[cst.makI().fatherPart().rowID()][cst.colID()].data(), imp_->cm_[cst.makJ().fatherPart().rowID()][cst.colID()].data());
			}
		}
		auto Model::cptCp()->void
		{
			imp_->cp_[0][0].resize(6, 0.0);
			for (auto &cst : activeConstraintPool())
			{
				imp_->cp_[cst.colID()][0].resize(cst.dim());
				cst.cptCp(imp_->cp_[cst.colID()][0].data());
			}
		}
		auto Model::cptCv()->void 
		{
			imp_->cv_[0][0].resize(6, 0.0);
			for (auto &cst : activeConstraintPool())
			{
				imp_->cv_[cst.colID()][0].resize(cst.dim());
				cst.cptCv(imp_->cv_[cst.colID()][0].data());
			}
		}
		auto Model::cptCa()->void {}
		auto Model::setPartP()->void
		{
			for (auto &prt : activePartPool())
			{
				double pm[4][4];
				double pq[7];

				s_vc(6, imp_->pp_[prt.rowID()][0].data(), pq);

				double theta = s_norm(3, pq + 3);
				pq[6] = std::cos(theta / 2);

				double factor = theta < 1e-4 ? 0.5 : std::sin(theta / 2) / theta;
				s_nv(3, factor, pq + 3);

				s_pq2pm(pq, *pm);

				double final_pm[4][4];
				s_pm2pm(*pm, *prt.pm(), *final_pm);

				prt.setPm(*final_pm);
			}
		}
		auto Model::setPartV()->void
		{
		}
		auto Model::setPartA()->void
		{
		}
		auto Model::setConstraintF()->void
		{
		}

		auto Model::kinPos(double error)->void
		{
			for (;;)
			{
				cptGlbCm();
				cptCp();
				s_blk_mmNT(pSize(), pSize(), cSize(), glbCm(), glbCm(), imp_->cct_);
				s_blk_mm(pSize(), { 1 }, cSize(), glbCm(), cp(), imp_->cct_b_);

				if (s_blk_norm(cSize(), cp()) < error)
				{
					std::cout << "norm and break:" << s_blk_norm(cSize(), cp()) << std::endl;
					break;
				}
				else
				{
					std::cout << "norm:" << s_blk_norm(cSize(), cp()) << std::endl;
				}

				s_blk_llt(pSize(), imp_->cct_, imp_->cct_llt_);
				s_blk_sov_lm(pSize(), { 1 }, imp_->cct_llt_, imp_->cct_b_, imp_->cct_x_);
				s_blk_sov_um(pSize(), { 1 }, imp_->cct_llt_, imp_->cct_x_, imp_->pp_);

				setPartP();
			}
		}
		auto Model::kinVel()->void
		{
			cptCv();



		}


		auto Model::kinPre()->void
		{
			dynUpdDim();
			dynAllocate(imp_->dyn_A_, imp_->dyn_b_, imp_->dyn_x_);
		}
		auto Model::kinUpd()->void
		{
			for (auto &jnt : jointPool())
			{
				if (jnt.active())
				{
					jnt.makI().updGlbPm();
					jnt.makJ().updGlbPm();
					jnt.updGlbCm();
					jnt.updCe();
				}
			}
			for (auto &mot : motionPool())
			{
				if (mot.active())
				{
					mot.makI().updGlbPm();
					mot.makJ().updGlbPm();
					mot.updGlbCm();
					mot.updCe();
				}
			}
		}
		
		auto Model::kinCe(double *ce)const->void
		{
			for (auto &jnt : jointPool())
			{
				if (jnt.active())
				{
					std::copy_n(jnt.cePtr(), jnt.dim(), &ce[jnt.colID()]);
				}
			}
			for (auto &mot : motionPool())
			{
				if (mot.active())
				{
					ce[mot.colID()] = *mot.cePtr();
				}
			}
		}
		auto Model::kinGlbCmT(double *glb_cmT, int ld, bool is_mtx_inited)const->void
		{
			dynGlbCmT(glb_cmT, ld, is_mtx_inited);
		}
		auto Model::kinSov(const double *A, const double *b, double *x)->void
		{
			


		}

		auto Model::updCa(double *ca)->void
		{
			for (auto &jnt : jointPool())if (jnt.active())jnt.updCa();
			for (auto &mot : motionPool())if (mot.active())mot.updCa();
		}
		auto Model::getCf(double *cf) const->void
		{
			for (auto &jnt : jointPool())if (jnt.active())std::copy_n(jnt.cfPtr(), jnt.dim(), &cf[jnt.colID()]);
			for (auto &mot : motionPool())if (mot.active())cf[mot.colID()] = mot.mfDyn();
		}
		auto Model::setCf(const double *cf)->void 
		{
			for (auto &jnt : jointPool())if (jnt.active())std::copy_n(&cf[jnt.colID()], jnt.dim(), const_cast<double*>(jnt.cfPtr()));
			for (auto &mot : motionPool())if (mot.active())mot.setMfDyn(cf[mot.colID()]);
		}
		auto Model::getPrtIs(double *prt_is, int ld, bool is_mtx_inited)const->void
		{
			ld = std::max(ld, static_cast<int>(dynDimM()));

			if (!is_mtx_inited)s_fill(dynDimM(), dynDimM(), 0, prt_is, ld);

			for (int i = 0; i < 6; ++i)prt_is[ld*(ground().rowID() + i) + ground().rowID() + i] = 1;

			for (auto &prt : partPool())if (prt.active())s_mc(6, 6, *(prt.prtIs()), 6, prt_is + ld*prt.rowID() + prt.rowID(), ld);
		}
		auto Model::updPrtCm(double *prt_cm, int ld, bool is_mtx_inited)->void 
		{
			ld = std::max(ld, static_cast<int>(dynDimN()));

			if (!is_mtx_inited)s_fill(dynDimM(), dynDimN(), 0, prt_cm, ld);

			for (int i = 0; i < 6; ++i)prt_cm[ld*(ground().rowID() + i) + i] = 1;

			for (auto &jnt : jointPool())
			{
				if (jnt.active())
				{
					jnt.updPrtCm();
					s_mc(6, jnt.dim(), jnt.prtCmPtrI(), jnt.dim(), prt_cm + ld*jnt.makI().fatherPart().rowID() + jnt.colID(), ld);
					s_mc(6, jnt.dim(), jnt.prtCmPtrJ(), jnt.dim(), prt_cm + ld*jnt.makJ().fatherPart().rowID() + jnt.colID(), ld);
				}
			}
			for (auto &mot : motionPool())
			{
				if (mot.active())
				{
					mot.updPrtCm();
					s_mc(6, 1, *mot.prtCmI(), 1, prt_cm + ld*mot.makI().fatherPart().rowID() + mot.colID(), ld);
					s_mc(6, 1, *mot.prtCmJ(), 1, prt_cm + ld*mot.makJ().fatherPart().rowID() + mot.colID(), ld);
				}
			}
		}
		auto Model::updPrtCmT(double *prt_cmT, int ld, bool is_mtx_inited)->void 
		{
			ld = std::max(ld, static_cast<int>(dynDimM()));

			if (!is_mtx_inited)s_fill(dynDimN(), dynDimM(), 0, prt_cmT, ld);

			for (int i = 0; i < 6; ++i)prt_cmT[ld*(ground().rowID() + i) + i] = 1;

			for (auto &jnt : jointPool())
			{
				if (jnt.active())
				{
					jnt.updPrtCm();
					s_mcT(jnt.dim(), 6, jnt.prtCmPtrI(), jnt.dim(), prt_cmT + jnt.makI().fatherPart().rowID() + ld*jnt.colID(), ld);
					s_mcT(jnt.dim(), 6, jnt.prtCmPtrJ(), jnt.dim(), prt_cmT + jnt.makJ().fatherPart().rowID() + ld*jnt.colID(), ld);
				}
			}
			for (auto &mot : motionPool())
			{
				if (mot.active())
				{
					mot.updPrtCm();
					s_mcT(1, 6, *mot.prtCmI(), 1, prt_cmT + mot.makI().fatherPart().rowID() + ld*mot.colID(), ld);
					s_mcT(1, 6, *mot.prtCmJ(), 1, prt_cmT + mot.makJ().fatherPart().rowID() + ld*mot.colID(), ld);
				}
			}
		}
		auto Model::updPrtCct(double *prt_cct, int ld, bool is_mtx_inited)->void {}
		auto Model::updPrtFs(double *prt_fs)->void 
		{
			for (auto &prt : partPool())
			{
				if (prt.active())
				{
					prt.updPrtVs();
					prt.updPrtGr();
					prt.updPrtFv();
					prt.updPrtFg();

					s_vc(6, prt.prtFv(), &prt_fs[prt.rowID()]);
					s_va(6, -1.0, prt.prtFg(), &prt_fs[prt.rowID()]);
				}
			}
			// it is not correct maybe, because wrong coordinate frame;
			for (auto &fce : forcePool())
			{
				if (fce.active())
				{
					s_va(6, -1.0, fce.fsI(), &prt_fs[fce.makI().fatherPart().rowID()]);
					s_va(6, -1.0, fce.fsJ(), &prt_fs[fce.makJ().fatherPart().rowID()]);
				}
			}
		}
		auto Model::getPrtAs(double *prt_as) const->void { for (auto &prt : partPool())if (prt.active())std::copy(prt.prtAs(), prt.prtAs() + 6, &prt_as[prt.rowID()]); }
		auto Model::setPrtAs(const double *prt_as)->void 
		{
			for (auto &prt : partPool())
			{
				if (prt.active())
				{
					std::copy_n(&prt_as[prt.rowID()], 6, prt.imp_->prt_as_);
					s_tv(*prt.pm(), prt.imp_->prt_as_, prt.imp_->glb_as_);
				}
			}
		}

		auto Model::updPrtA(double *A, bool is_mtx_inited)->void
		{
			if (!is_mtx_inited)std::fill(A, A + dynDim()*dynDim(), 0);

			updPrtCm(A + dynDimM(), dynDim(), true);
			updPrtCmT(A + dynDimM()*dynDim(), dynDim(), true);
			getPrtIs(A, dynDim(), true);
			for (std::size_t i{ 0 }; i < dynDimM(); i += 6)s_nm(6, 6, -1.0, A + i*dynDim() + i, dynDim());
		}
		auto Model::updPrtB(double *b, bool is_mtx_inited)->void 
		{
			updPrtFs(b);
			updCa(b + dynDimM());
		}
		auto Model::getPrtX(double *x) const->void {}
		auto Model::setPrtX(const double *x)->void {}

		auto Model::dynPre()->void
		{
			dynUpdDim();
			dynAllocate(imp_->dyn_A_, imp_->dyn_b_, imp_->dyn_x_);
		}
		auto Model::dynPrt()->void
		{
			dynPrtUpd();
			dynPrtMtx(imp_->dyn_A_.get(), imp_->dyn_b_.get());
			dynSov(imp_->dyn_A_.get(), imp_->dyn_b_.get(), imp_->dyn_x_.get());
			dynPrtEnd(imp_->dyn_x_.get());
		}
		auto Model::dynGlb()->void
		{
			dynGlbUpd();
			dynGlbMtx(imp_->dyn_A_.get(), imp_->dyn_b_.get());
			dynSov(imp_->dyn_A_.get(), imp_->dyn_b_.get(), imp_->dyn_x_.get());
			dynGlbEnd(imp_->dyn_x_.get());
		}
		auto Model::dynUpdDim()->void
		{
			auto &pid = (imp_->dyn_prt_dim_ = 0);
			auto &cid = (imp_->dyn_cst_dim_ = 6);

			for (auto &prt : partPool()) prt.Part::imp_->row_id_ = prt.active() ? (pid += 6) - 6 : 0;
			for (auto &jnt : jointPool()) jnt.Constraint::imp_->col_id_ = jnt.active() ? (cid += jnt.dim()) - jnt.dim() : 0;
			for (auto &mot : motionPool()) mot.Constraint::imp_->col_id_ = mot.active() ? (cid += mot.dim()) - mot.dim() : 0;
		}
		auto Model::dynAllocate(std::unique_ptr<double[]> &D, std::unique_ptr<double[]> &b, std::unique_ptr<double[]> &x)->void
		{
			D.reset(new double[dynDim()*dynDim()]);
			b.reset(new double[dynDim()]);
			x.reset(new double[dynDim()]);

			std::fill(D.get(), D.get() + dynDim()*dynDim(), 0);
			std::fill(b.get(), b.get() + dynDim(), 0);
			std::fill(x.get(), x.get() + dynDim(), 0);
		}
		auto Model::dynSetSolveMethod(std::function<void(int dim, const double *D, const double *b, double *x)> solve_method)->void{imp_->dyn_solve_method_ = solve_method;}
		auto Model::dynSov(const double *D, const double *b, double *x) const->void
		{
			if (imp_->dyn_solve_method_)
			{
				imp_->dyn_solve_method_(dynDim(), D, b, x);
			}
			else
			{
				throw std::runtime_error("please set solve_method before use DynSov");
			}
		}
		auto Model::dynDimM()const->std::size_t { return imp_->dyn_prt_dim_; }
		auto Model::dynDimN()const->std::size_t { return imp_->dyn_cst_dim_; }
		auto Model::dynCa(double *ca) const->void
		{
			std::fill_n(ca, dynDimN(), 0);

			for (auto &jnt : jointPool())
			{
				if (jnt.active())
				{
					std::copy_n(jnt.caPtr(), jnt.dim(), &ca[jnt.colID()]);
				}
			}
			for (auto &mot : motionPool())
			{
				if (mot.active())
				{
					ca[mot.colID()] = *mot.ca();
				}
			}
		}
		auto Model::dynCf(double *cf) const->void
		{
			for (auto &jnt : jointPool())
			{
				if (jnt.active())
				{
					std::copy_n(jnt.cfPtr(), jnt.dim(), &cf[jnt.colID()]);
				}
			}
			for (auto &mot : motionPool())
			{
				if (mot.active())
				{
					cf[mot.colID()] = mot.mfDyn();
				}
			}

		}
		auto Model::dynPrtCm(double *prt_cm, int ld, bool is_mtx_inited) const->void
		{
			ld = std::max(ld, static_cast<int>(dynDimN()));

			if(!is_mtx_inited)s_fill(dynDimM(), dynDimN(), 0, prt_cm, ld);

			for (int i = 0; i < 6; ++i)prt_cm[ld*(ground().rowID() + i) + i] = 1;

			for (auto &jnt : jointPool())
			{
				if (jnt.active())
				{
					s_mc(6, jnt.dim(), jnt.prtCmPtrI(), jnt.dim(), prt_cm + ld*jnt.makI().fatherPart().rowID() + jnt.colID(), ld);
					s_mc(6, jnt.dim(), jnt.prtCmPtrJ(), jnt.dim(), prt_cm + ld*jnt.makJ().fatherPart().rowID() + jnt.colID(), ld);
				}
			}
			for (auto &mot : motionPool())
			{
				if (mot.active())
				{
					s_mc(6, 1, *mot.prtCmI(), 1, prt_cm + ld*mot.makI().fatherPart().rowID() + mot.colID(), ld);
					s_mc(6, 1, *mot.prtCmJ(), 1, prt_cm + ld*mot.makJ().fatherPart().rowID() + mot.colID(), ld);
				}
			}
		}
		auto Model::dynPrtCmT(double *prt_cmT, int ld, bool is_mtx_inited) const->void
		{
			ld = std::max(ld, static_cast<int>(dynDimM()));

			if (!is_mtx_inited)s_fill(dynDimN(), dynDimM(), 0, prt_cmT, ld);

			for (int i = 0; i < 6; ++i)prt_cmT[ld*(ground().rowID() + i) + i] = 1;

			for (auto &jnt : jointPool())
			{
				if (jnt.active())
				{
					s_mcT(jnt.dim(), 6, jnt.prtCmPtrI(), jnt.dim(), prt_cmT + jnt.makI().fatherPart().rowID() + ld*jnt.colID(), ld);
					s_mcT(jnt.dim(), 6, jnt.prtCmPtrJ(), jnt.dim(), prt_cmT + jnt.makJ().fatherPart().rowID() + ld*jnt.colID(), ld);
				}
			}
			for (auto &mot : motionPool())
			{
				if (mot.active())
				{
					s_mcT(1, 6, *mot.prtCmI(), 1, prt_cmT + mot.makI().fatherPart().rowID() + ld*mot.colID(), ld);
					s_mcT(1, 6, *mot.prtCmJ(), 1, prt_cmT + mot.makJ().fatherPart().rowID() + ld*mot.colID(), ld);
				}
			}

		}
		auto Model::dynPrtIs(double *prt_is, int ld, bool is_mtx_inited) const->void
		{
			ld = std::max(ld, static_cast<int>(dynDimM()));
			
			if (!is_mtx_inited)s_fill(dynDimM(), dynDimM(), 0, prt_is, ld);

			for (int i = 0; i < 6; ++i)prt_is[ld*(ground().rowID() + i) + ground().rowID() + i] = 1;

			for (auto &prt : partPool())
			{
				if (prt.active())
				{
					s_mc(6, 6, *(prt.prtIs()), 6, prt_is + ld*prt.rowID() + prt.rowID(), ld);
				}
			}
		}
		auto Model::dynPrtFs(double *prt_fs) const->void
		{
			for (auto &prt : partPool())
			{
				if (prt.active())
				{
					s_vc(6, prt.prtFv(), &prt_fs[prt.rowID()]);
					s_va(6, -1.0, prt.prtFg(), &prt_fs[prt.rowID()]);
				}
			}
			// it is not correct maybe, because wrong coordinate frame;
			for (auto &fce : forcePool())
			{
				if (fce.active())
				{
					s_va(6, -1.0, fce.fsI(), &prt_fs[fce.makI().fatherPart().rowID()]);
					s_va(6, -1.0, fce.fsJ(), &prt_fs[fce.makJ().fatherPart().rowID()]);
				}
			}
		}
		auto Model::dynPrtAs(double *prt_as) const->void
		{
			for (auto &prt : partPool())
			{
				if (prt.active())
				{
					std::copy(prt.prtAs(), prt.prtAs() + 6, &prt_as[prt.rowID()]);
				}
			}
		}
		auto Model::dynPrtUpd()->void
		{
			for (auto &prt : partPool())
			{
				if (prt.active())
				{
					prt.updPrtVs();
					prt.updPrtAs();
					prt.updPrtGr();
					prt.updPrtFv();
					prt.updPrtFg();
				}
			}
			for (auto &jnt : jointPool())
			{
				if (jnt.active())
				{
					jnt.updPrtCm();
					jnt.updCa();
				}
			}
			for (auto &mot : motionPool())
			{
				if (mot.active())
				{
					mot.updPrtCm();
					mot.updCa();
				}
			}

			// need modify
			for (auto &fce : forcePool())
			{
				if (fce.active())
					fce.updFs();
			}
		}
		auto Model::dynPrtMtx(double *A, double *b, bool is_mtx_inited) const->void
		{
			if (!is_mtx_inited)std::fill(A, A + dynDim()*dynDim(), 0);

			dynPrtCm(A + dynDimM(), dynDim(), true);
			dynPrtCmT(A + dynDimM()*dynDim(), dynDim(), true);
			dynPrtIs(A, dynDim(), true);
			for (std::size_t i{ 0 }; i < dynDimM(); i += 6)s_nm(6, 6, -1.0, A + i*dynDim() + i, dynDim());

			dynPrtFs(b);
			dynCa(b + dynDimM());
		}
		auto Model::dynPrtUkn(double *x) const->void
		{
			this->dynPrtAs(x);
			this->dynCf(x + dynDimM());
		}
		auto Model::dynPrtEnd(const double *x)->void
		{
			for (auto &prt : partPool())
			{
				if (prt.active())
				{
					std::copy_n(&x[prt.rowID()], 6, prt.imp_->prt_as_);
					s_tv(*prt.pm(), prt.imp_->prt_as_, prt.imp_->glb_as_);
				}
			}
			for (auto &jnt : jointPool())
			{
				if (jnt.active())
				{
					std::copy_n(&x[jnt.colID() + dynDimM()], jnt.dim(), const_cast<double*>(jnt.cfPtr()));
				}
			}
			for (auto &mot : motionPool())
			{
				if (mot.active())
				{
					mot.setMfDyn(x[mot.colID() + dynDimM()]);
				}
			}
		}
		auto Model::dynPrtSov()->void
		{
			///////////////////// allocate memory /////////////////////////////////
			auto active_prt_num = dynDimM() / 6;

			BlockMatrix block_CCT;
			BlockMatrix block_LLT;
			BlockMatrix block_b, block_x;
			BlockSize CCT_size(active_prt_num, 6);
			BlockSize cca_size{ 1 };

			s_blk_allocate(CCT_size, CCT_size, block_CCT);
			s_blk_allocate(CCT_size, CCT_size, block_LLT);
			s_blk_allocate(CCT_size, cca_size, block_b);
			s_blk_allocate(CCT_size, cca_size, block_x);

			/////////////////////////////////// init memories //////////////////////////////////
			for (auto &jnt : jointPool())
			{
				auto i = jnt.makI().fatherPart().rowID() / 6;
				auto j = jnt.makJ().fatherPart().rowID() / 6;

				block_CCT[i][j].resize(36, 0);
				block_CCT[j][i].resize(36, 0);
			}

			for (auto &mot : motionPool())
			{
				auto i = mot.makI().fatherPart().rowID() / 6;
				auto j = mot.makJ().fatherPart().rowID() / 6;

				block_CCT[i][j].resize(36, 0);
				block_CCT[j][i].resize(36, 0);
			}

			for (std::size_t i = 0; i < active_prt_num; ++i)
			{
				block_CCT[i][i].resize(36, 0);
				block_b[i][0].resize(6, 0);
			}

			for (std::size_t i = 0; i < 6; ++i)block_CCT[ground().rowID() / 6][ground().rowID() / 6].data()[i + 6 * i] = 1.0;
			/////////////////////////////////// update //////////////////////////////


			this->dynGlbUpd();
			this->dynPrtUpd();


			/////////////////////////////////// fill memory //////////////////////////////		
			for (auto &jnt : jointPool())
			{
				auto i = jnt.makI().fatherPart().rowID() / 6;
				auto j = jnt.makJ().fatherPart().rowID() / 6;

				//s_mmaNT(6, 6, jnt.dim(), jnt.glbCmPtrI(), jnt.glbCmPtrI(), block_D[i].get());
				//s_mmaNT(6, 6, jnt.dim(), jnt.glbCmPtrJ(), jnt.glbCmPtrJ(), block_D[j].get());
				//s_mmaNT(6, 6, jnt.dim(), jnt.glbCmPtrI(), jnt.glbCmPtrJ(), block_CCT[i][j].get());
				//s_mmaNT(6, 6, jnt.dim(), jnt.glbCmPtrJ(), jnt.glbCmPtrI(), block_CCT[j][i].get());

				//s_mma(6, 1, jnt.dim(), jnt.glbCmPtrI(), jnt.caPtr(), block_b[jnt.makI().fatherPart().rowID() / 6].get());
				//s_mma(6, 1, jnt.dim(), jnt.glbCmPtrJ(), jnt.caPtr(), block_b[jnt.makJ().fatherPart().rowID() / 6].get());

				s_mmaNT(6, 6, jnt.dim(), jnt.prtCmPtrI(), jnt.prtCmPtrI(), block_CCT[i][i].data());
				s_mmaNT(6, 6, jnt.dim(), jnt.prtCmPtrJ(), jnt.prtCmPtrJ(), block_CCT[j][j].data());
				s_mmaNT(6, 6, jnt.dim(), jnt.prtCmPtrI(), jnt.prtCmPtrJ(), block_CCT[i][j].data());
				s_mmaNT(6, 6, jnt.dim(), jnt.prtCmPtrJ(), jnt.prtCmPtrI(), block_CCT[j][i].data());

				s_mma(6, 1, jnt.dim(), jnt.prtCmPtrI(), jnt.caPtr(), block_b[jnt.makI().fatherPart().rowID() / 6][0].data());
				s_mma(6, 1, jnt.dim(), jnt.prtCmPtrJ(), jnt.caPtr(), block_b[jnt.makJ().fatherPart().rowID() / 6][0].data());
			}

			for (auto &mot : motionPool())
			{
				auto i = mot.makI().fatherPart().rowID() / 6;
				auto j = mot.makJ().fatherPart().rowID() / 6;

				//s_mmaNT(6, 6, mot.dim(), mot.glbCmPtrI(), mot.glbCmPtrI(), block_D[i].get());
				//s_mmaNT(6, 6, mot.dim(), mot.glbCmPtrJ(), mot.glbCmPtrJ(), block_D[j].get());
				//s_mmaNT(6, 6, mot.dim(), mot.glbCmPtrI(), mot.glbCmPtrJ(), block_CCT[i][j].get());
				//s_mmaNT(6, 6, mot.dim(), mot.glbCmPtrJ(), mot.glbCmPtrI(), block_CCT[j][i].get());

				//s_mma(6, 1, mot.dim(), mot.glbCmPtrI(), mot.caPtr(), block_b[mot.makI().fatherPart().rowID() / 6].get());
				//s_mma(6, 1, mot.dim(), mot.glbCmPtrJ(), mot.caPtr(), block_b[mot.makJ().fatherPart().rowID() / 6].get());

				s_mmaNT(6, 6, mot.dim(), mot.prtCmPtrI(), mot.prtCmPtrI(), block_CCT[i][i].data());
				s_mmaNT(6, 6, mot.dim(), mot.prtCmPtrJ(), mot.prtCmPtrJ(), block_CCT[j][j].data());
				s_mmaNT(6, 6, mot.dim(), mot.prtCmPtrI(), mot.prtCmPtrJ(), block_CCT[i][j].data());
				s_mmaNT(6, 6, mot.dim(), mot.prtCmPtrJ(), mot.prtCmPtrI(), block_CCT[j][i].data());

				s_mma(6, 1, mot.dim(), mot.prtCmPtrI(), mot.caPtr(), block_b[mot.makI().fatherPart().rowID() / 6][0].data());
				s_mma(6, 1, mot.dim(), mot.prtCmPtrJ(), mot.caPtr(), block_b[mot.makJ().fatherPart().rowID() / 6][0].data());
			}

			///////////////////////////////////////////////// check if memory is correct ///////////////////////////////////////////////////////////////////////
			double *CCT = new double[dynDimM()*dynDimM()];
			double *CCT2 = new double[dynDimM()*dynDimM()];
			double *C = new double[dynDimM()*dynDimN()];
			double *CT = new double[dynDimN()*dynDimM()];
			double *ca = new double[dynDimN()];
			double *cca = new double[dynDimM()];
			double *cca2 = new double[dynDimM()];
			double *x = new double[dynDimM()];
			double *D_inv = new double[dynDimM() * 6];
			double *D = new double[dynDimM() * 6];
			std::fill_n(CCT, dynDimM()*dynDimM(), 0);
			std::fill_n(CCT2, dynDimM()*dynDimM(), 0);
			std::fill_n(C, dynDimM()*dynDimN(), 0);
			std::fill_n(CT, dynDimN()*dynDimM(), 0);
			std::fill_n(ca, dynDimM(), 0);
			std::fill_n(cca, dynDimM(), 0);
			std::fill_n(cca2, dynDimM(), 0);
			std::fill_n(D_inv, dynDimM() * 6, 0);

			for (std::size_t i = 0; i < dynDimM() / 6; ++i)
			{
				for (std::size_t j = 0; j < dynDimM() / 6; ++j)
				{
					if (!block_CCT[i][j].empty())
						s_mc(6, 6, block_CCT[i][j].data(), 6, CCT + dynDimM() * i * 6 + j * 6, dynDimM());
				}

				s_mc(6, 1, block_b[i][0].data(), cca + 6 * i);
			}

			dlmwrite("C:\\Users\\py033\\Desktop\\CCT.txt", CCT, dynDimM(), dynDimM());
			dlmwrite("C:\\Users\\py033\\Desktop\\cca.txt", cca, dynDimM(), 1);

			//dynGlbCm(C);
			dynPrtCm(C);
			dynCa(ca);
			s_mmNT(dynDimM(), dynDimM(), dynDimN(), C, C, CCT2);
			s_mm(dynDimM(), 1, dynDimN(), C, ca, cca2);

			dlmwrite("C:\\Users\\py033\\Desktop\\CCT2.txt", CCT2, dynDimM(), dynDimM());
			dlmwrite("C:\\Users\\py033\\Desktop\\cca2.txt", cca2, dynDimM(), 1);

			if (!s_is_equal(dynDimM()*dynDimM(), CCT, CCT2, 1e-10)) std::cout << "calculate CCT not correct!" << std::endl;
			if (!s_is_equal(dynDimM(), cca, cca2, 1e-10)) std::cout << "calculation cca not correct!" << std::endl;
			//////////////////////////////////////////////////  check end   /////////////////////////////////////////////////////////////////////////////////


			s_blk_llt(CCT_size, block_CCT, block_LLT);

			std::cout << "CCT total num:" << block_CCT.size()*block_CCT[0].size() << std::endl;
			std::cout << "CCT empty block num:" << s_blk_check_empty_num(block_CCT) << std::endl;
			std::cout << "LLT empty block num:" << s_blk_check_empty_num(block_LLT) << std::endl;
			
			s_blk_sov_lm(CCT_size, cca_size, block_LLT, block_b, block_x);
			s_blk_sov_um(CCT_size, cca_size, block_LLT, block_x, block_x);
			
			s_blk_resolve(CCT_size, cca_size, block_x, x);

			dlmwrite("C:\\Users\\py033\\Desktop\\x.txt", x, dynDimM(), 1);

			dynPrtAs(x);
			dlmwrite("C:\\Users\\py033\\Desktop\\x2.txt", x, dynDimM(), 1);


			delete[]C;
			delete[]CT;
			delete[]CCT;
			delete[]CCT2;
			delete[]ca;
			delete[]cca;
			delete[]cca2;
			delete[]x;

		}
		auto Model::dynGlbCm(double *glb_cm, int ld, bool is_mtx_inited) const->void 
		{
			ld = std::max(ld, static_cast<int>(dynDimN()));

			if (!is_mtx_inited)s_fill(dynDimM(), dynDimN(), 0, glb_cm, ld);

			for (int i = 0; i < 6; ++i)glb_cm[ld*(ground().rowID() + i) + i] = 1;

			for (auto &jnt : jointPool())
			{
				if (jnt.active())
				{
					s_mc(6, jnt.dim(), jnt.glbCmPtrI(), jnt.dim(), glb_cm + ld*jnt.makI().fatherPart().rowID() + jnt.colID(), ld);
					s_mc(6, jnt.dim(), jnt.glbCmPtrJ(), jnt.dim(), glb_cm + ld*jnt.makJ().fatherPart().rowID() + jnt.colID(), ld);
				}
			}
			for (auto &mot : motionPool())
			{
				if (mot.active())
				{
					s_mc(6, 1, *mot.glbCmI(), 1, glb_cm + ld*mot.makI().fatherPart().rowID() + mot.colID(), ld);
					s_mc(6, 1, *mot.glbCmJ(), 1, glb_cm + ld*mot.makJ().fatherPart().rowID() + mot.colID(), ld);
				}
			}
		}
		auto Model::dynGlbCmT(double *glb_cmT, int ld, bool is_mtx_inited) const->void
		{
			ld = std::max(ld, static_cast<int>(dynDimM()));

			if (!is_mtx_inited)s_fill(dynDimN(), dynDimM(), 0, glb_cmT, ld);

			for (int i = 0; i < 6; ++i)glb_cmT[ld*(ground().rowID() + i) + i] = 1;

			for (auto &jnt : jointPool())
			{
				if (jnt.active())
				{
					s_mcT(jnt.dim(), 6, jnt.glbCmPtrI(), jnt.dim(), glb_cmT + jnt.makI().fatherPart().rowID() + ld*jnt.colID(), ld);
					s_mcT(jnt.dim(), 6, jnt.glbCmPtrJ(), jnt.dim(), glb_cmT + jnt.makJ().fatherPart().rowID() + ld*jnt.colID(), ld);
				}
			}
			for (auto &mot : motionPool())
			{
				if (mot.active())
				{
					s_mcT(1, 6, *mot.glbCmI(), 1, glb_cmT + mot.makI().fatherPart().rowID() + ld*mot.colID(), ld);
					s_mcT(1, 6, *mot.glbCmJ(), 1, glb_cmT + mot.makJ().fatherPart().rowID() + ld*mot.colID(), ld);
				}
			}
		}
		auto Model::dynGlbIs(double *glb_is, int ld, bool is_mtx_inited) const->void
		{
			ld = std::max(ld, static_cast<int>(dynDimM()));

			if (!is_mtx_inited)s_fill(dynDimM(), dynDimM(), 0, glb_is, ld);

			for (int i = 0; i < 6; ++i)glb_is[ld*(ground().rowID() + i) + ground().rowID() + i] = 1;

			for (auto &prt : partPool())
			{
				if (prt.active())
				{
					s_mc(6, 6, *(prt.glbIs()), 6, glb_is + ld*prt.rowID() + prt.rowID(), ld);
				}
			}
		}
		auto Model::dynGlbFs(double *glb_fs) const->void
		{
			for (auto &prt : partPool())
			{
				if (prt.active())
				{
					s_vc(6, prt.glbFv(), &glb_fs[prt.rowID()]);
					s_va(6, -1.0, prt.glbFg(), &glb_fs[prt.rowID()]);
				}
			}
			// it is not correct maybe, because wrong coordinate frame;
			for (auto &fce : forcePool())
			{
				if (fce.active())
				{
					s_va(6, -1.0, fce.fsI(), &glb_fs[fce.makI().fatherPart().rowID()]);
					s_va(6, -1.0, fce.fsJ(), &glb_fs[fce.makJ().fatherPart().rowID()]);
				}
			}
		}
		auto Model::dynGlbAs(double *glb_as) const->void
		{
			for (auto &prt : partPool())
			{
				if (prt.active())
				{
					std::copy(prt.as(), prt.as() + 6, &glb_as[prt.rowID()]);
				}
			}
		}
		auto Model::dynGlbMtx(double *A, double *b, bool is_mtx_inited) const->void
		{
			if (!is_mtx_inited)std::fill(A, A + dynDim()*dynDim(), 0);

			dynGlbCm(A + dynDimM(), dynDim(), true);
			dynGlbCmT(A + dynDimM()*dynDim(), dynDim(), true);
			dynGlbIs(A, dynDim(), true);
			for (std::size_t i{ 0 }; i < dynDimM(); i += 6)s_nm(6, 6, -1.0, A + i*dynDim() + i, dynDim());

			dynGlbFs(b);
			dynCa(b + dynDimM());
		}
		auto Model::dynGlbUkn(double *x) const->void
		{
			this->dynGlbAs(x);
			this->dynCf(x + dynDimM());
		}
		auto Model::dynGlbEnd(const double *x)->void
		{
			for (auto &prt : partPool())
			{
				if (prt.active())
				{
					std::copy_n(&x[prt.rowID()], 6, prt.imp_->glb_as_);
				}
			}
			for (auto &jnt : jointPool())
			{
				if (jnt.active())
				{
					std::copy_n(&x[jnt.colID() + dynDimM()], jnt.dim(), const_cast<double*>(jnt.cfPtr()));
				}
			}
			for (auto &mot : motionPool())
			{
				if (mot.active())
				{
					mot.setMfDyn(x[mot.colID() + dynDimM()]);
				}
			}
		}
		auto Model::dynGlbUpd()->void
		{
			for (auto &prt : partPool())
			{
				if (prt.active())
				{
					//prt.updVs();
					//prt.updAs();
					//prt.updGr();
					prt.updGlbIs();
					prt.updGlbFv();
					prt.updGlbFg();
				}
			}
			for (auto &jnt : jointPool())
			{
				if (jnt.active())
				{
					jnt.updGlbCm();
					jnt.updCa();
				}
			}
			for (auto &mot : motionPool())
			{
				if (mot.active())
				{
					mot.updGlbCm();
					mot.updCa();
				}
			}

			// need modify
			for (auto &fce : forcePool())
			{
				if (fce.active())
					fce.updFs();
			}
		}
		auto Model::dynGlbSovGaussSeidel()->void
		{
			///////////////////// allocate memory /////////////////////////////////
			auto active_prt_num = dynDimM() / 6;

			std::vector<std::vector<std::unique_ptr<double[]> > > block_CCT;
			std::vector<std::vector<std::unique_ptr<double[]> > > block_k;
			std::vector<std::unique_ptr<double[]> > block_b;
			std::vector<std::unique_ptr<double[]> > block_D;//diag of CCT
			std::vector<std::unique_ptr<double[]> > block_D_inv;
			std::vector<std::unique_ptr<double[]> > block_x_k;
			std::vector<std::unique_ptr<double[]> > block_x_k1;

			// make structure and allocate memory
			for (std::size_t i = 0; i < active_prt_num; ++i)
			{
				block_CCT.push_back(std::vector<std::unique_ptr<double[]> >());
				block_k.push_back(std::vector<std::unique_ptr<double[]> >());
				for (std::size_t j = 0; j < dynDimM() / 6; ++j) 
				{
					block_CCT.back().push_back(std::unique_ptr<double[]>());
					block_k.back().push_back(std::unique_ptr<double[]>());
				}

				block_D.push_back(std::unique_ptr<double[]>(new double[36]));
				block_D_inv.push_back(std::unique_ptr<double[]>(new double[36]));
				block_b.push_back(std::unique_ptr<double[]>(new double[6]));
				block_x_k.push_back(std::unique_ptr<double[]>(new double[6]));
				block_x_k1.push_back(std::unique_ptr<double[]>(new double[6]));
			}

			for (auto &jnt : jointPool())
			{
				auto i = jnt.makI().fatherPart().rowID() / 6;
				auto j = jnt.makJ().fatherPart().rowID() / 6;

				if (!block_k[i][j])block_k[i][j].reset(new double[36]);
				if (!block_k[j][i])block_k[j][i].reset(new double[36]);
				if (!block_CCT[i][j])block_CCT[i][j].reset(new double[36]);
				if (!block_CCT[j][i])block_CCT[j][i].reset(new double[36]);
			}

			for (auto &mot : motionPool())
			{
				auto i = mot.makI().fatherPart().rowID() / 6;
				auto j = mot.makJ().fatherPart().rowID() / 6;

				if (!block_k[i][j])block_k[i][j].reset(new double[36]);
				if (!block_k[j][i])block_k[j][i].reset(new double[36]);
				if (!block_CCT[i][j])block_CCT[i][j].reset(new double[36]);
				if (!block_CCT[j][i])block_CCT[j][i].reset(new double[36]);
			}

			/////////////////////////////////// init memories //////////////////////////////////
			for (std::size_t i = 0; i < active_prt_num; ++i) 
			{
				std::fill_n(block_D[i].get(), 36, 0);
				std::fill_n(block_b[i].get(), 6, 0);
				std::fill_n(block_x_k[i].get(), 6, 0);
				std::fill_n(block_x_k1[i].get(), 6, 0);
			}
			for (std::size_t i = 0; i < 6; ++i)block_D[ground().rowID() / 6].get()[i + 6 * i] = 1;
			
			for (auto &jnt : jointPool())
			{
				auto i = jnt.makI().fatherPart().rowID() / 6;
				auto j = jnt.makJ().fatherPart().rowID() / 6;

				std::fill_n(block_CCT[i][j].get(), 36, 0);
				std::fill_n(block_CCT[j][i].get(), 36, 0);
			}

			for (auto &mot : motionPool())
			{
				auto i = mot.makI().fatherPart().rowID() / 6;
				auto j = mot.makJ().fatherPart().rowID() / 6;

				std::fill_n(block_CCT[i][j].get(), 36, 0);
				std::fill_n(block_CCT[j][i].get(), 36, 0);
			}
		
			/////////////////////////////////// update //////////////////////////////

			
			this->dynGlbUpd();
			this->dynPrtUpd();

			
			/////////////////////////////////// fill memory //////////////////////////////		
			for (auto &jnt : jointPool())
			{
				auto i = jnt.makI().fatherPart().rowID() / 6;
				auto j = jnt.makJ().fatherPart().rowID() / 6;

				//s_mmaNT(6, 6, jnt.dim(), jnt.glbCmPtrI(), jnt.glbCmPtrI(), block_D[i].get());
				//s_mmaNT(6, 6, jnt.dim(), jnt.glbCmPtrJ(), jnt.glbCmPtrJ(), block_D[j].get());
				//s_mmaNT(6, 6, jnt.dim(), jnt.glbCmPtrI(), jnt.glbCmPtrJ(), block_CCT[i][j].get());
				//s_mmaNT(6, 6, jnt.dim(), jnt.glbCmPtrJ(), jnt.glbCmPtrI(), block_CCT[j][i].get());

				//s_mma(6, 1, jnt.dim(), jnt.glbCmPtrI(), jnt.caPtr(), block_b[jnt.makI().fatherPart().rowID() / 6].get());
				//s_mma(6, 1, jnt.dim(), jnt.glbCmPtrJ(), jnt.caPtr(), block_b[jnt.makJ().fatherPart().rowID() / 6].get());

				s_mmaNT(6, 6, jnt.dim(), jnt.prtCmPtrI(), jnt.prtCmPtrI(), block_D[i].get());
				s_mmaNT(6, 6, jnt.dim(), jnt.prtCmPtrJ(), jnt.prtCmPtrJ(), block_D[j].get());
				s_mmaNT(6, 6, jnt.dim(), jnt.prtCmPtrI(), jnt.prtCmPtrJ(), block_CCT[i][j].get());
				s_mmaNT(6, 6, jnt.dim(), jnt.prtCmPtrJ(), jnt.prtCmPtrI(), block_CCT[j][i].get());

				s_mma(6, 1, jnt.dim(), jnt.prtCmPtrI(), jnt.caPtr(), block_b[jnt.makI().fatherPart().rowID() / 6].get());
				s_mma(6, 1, jnt.dim(), jnt.prtCmPtrJ(), jnt.caPtr(), block_b[jnt.makJ().fatherPart().rowID() / 6].get());
			}

			for (auto &mot : motionPool())
			{
				auto i = mot.makI().fatherPart().rowID() / 6;
				auto j = mot.makJ().fatherPart().rowID() / 6;

				//s_mmaNT(6, 6, mot.dim(), mot.glbCmPtrI(), mot.glbCmPtrI(), block_D[i].get());
				//s_mmaNT(6, 6, mot.dim(), mot.glbCmPtrJ(), mot.glbCmPtrJ(), block_D[j].get());
				//s_mmaNT(6, 6, mot.dim(), mot.glbCmPtrI(), mot.glbCmPtrJ(), block_CCT[i][j].get());
				//s_mmaNT(6, 6, mot.dim(), mot.glbCmPtrJ(), mot.glbCmPtrI(), block_CCT[j][i].get());

				//s_mma(6, 1, mot.dim(), mot.glbCmPtrI(), mot.caPtr(), block_b[mot.makI().fatherPart().rowID() / 6].get());
				//s_mma(6, 1, mot.dim(), mot.glbCmPtrJ(), mot.caPtr(), block_b[mot.makJ().fatherPart().rowID() / 6].get());

				s_mmaNT(6, 6, mot.dim(), mot.prtCmPtrI(), mot.prtCmPtrI(), block_D[i].get());
				s_mmaNT(6, 6, mot.dim(), mot.prtCmPtrJ(), mot.prtCmPtrJ(), block_D[j].get());
				s_mmaNT(6, 6, mot.dim(), mot.prtCmPtrI(), mot.prtCmPtrJ(), block_CCT[i][j].get());
				s_mmaNT(6, 6, mot.dim(), mot.prtCmPtrJ(), mot.prtCmPtrI(), block_CCT[j][i].get());

				s_mma(6, 1, mot.dim(), mot.prtCmPtrI(), mot.caPtr(), block_b[mot.makI().fatherPart().rowID() / 6].get());
				s_mma(6, 1, mot.dim(), mot.prtCmPtrJ(), mot.caPtr(), block_b[mot.makJ().fatherPart().rowID() / 6].get());
			}

			///////// inverse mtx //////////
			for (std::size_t i = 0; i < active_prt_num; ++i)
			{
				//Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor> >D(block_D[i].get());
				//Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor> >D_inv(block_D_inv[i].get());
				//D_inv = D.inverse();
			}
			///////// inverse finished //////////

			///////// pre make mtx //////////////
			for (std::size_t i = 0; i < active_prt_num; ++i)
			{
				for (std::size_t j = 0; j < active_prt_num; ++j)
				{
					if(block_CCT[i][j])	s_mm(6, 6, 6, block_D_inv[i].get(), block_CCT[i][j].get(), block_k[i][j].get());
				}
			}

			for (std::size_t i = 0; i < active_prt_num; ++i)
			{
				double tem[36];
				std::copy_n(block_b[i].get(), 36, tem);
				s_mm(6, 1, 6, block_D_inv[i].get(), tem, block_b[i].get());
			}


			////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			double *CCT = new double[dynDimM()*dynDimM()];
			double *CCT2 = new double[dynDimM()*dynDimM()];
			double *C = new double[dynDimM()*dynDimN()];
			double *CT = new double[dynDimN()*dynDimM()];
			double *ca = new double[dynDimN()];
			double *cca = new double[dynDimM()];
			double *cca2 = new double[dynDimM()];
			double *x = new double[dynDimM()];
			double *D_inv = new double[dynDimM()*6];
			double *D = new double[dynDimM() * 6];
			std::fill_n(CCT, dynDimM()*dynDimM(), 0);
			std::fill_n(CCT2, dynDimM()*dynDimM(), 0);
			std::fill_n(C, dynDimM()*dynDimN(), 0);
			std::fill_n(CT, dynDimN()*dynDimM(), 0);
			std::fill_n(ca, dynDimM(), 0);
			std::fill_n(cca, dynDimM(), 0);
			std::fill_n(cca2, dynDimM(), 0);
			std::fill_n(D_inv, dynDimM()*6, 0);
			
			for (std::size_t i = 0; i < dynDimM() / 6; ++i)
			{
				for (std::size_t j = 0; j < dynDimM() / 6; ++j)
				{
					if (i == j)
					{
						s_mc(6, 6, block_D[i].get(), 6, CCT + dynDimM() * i * 6 + j * 6, dynDimM());
					}
					else if (block_CCT[i][j])
						s_mc(6, 6, block_CCT[i][j].get(), 6, CCT + dynDimM() * i * 6 + j * 6, dynDimM());
				}

				s_mc(6, 1, block_b[i].get(), cca + 6 * i);
			}
			
			dlmwrite("C:\\Users\\py033\\Desktop\\CCT.txt", CCT, dynDimM(), dynDimM());
			dlmwrite("C:\\Users\\py033\\Desktop\\cca.txt", cca, dynDimM(), 1);

			//dynGlbCm(C);
			dynPrtCm(C);
			dynCa(ca);
			s_mmNT(dynDimM(), dynDimM(), dynDimN(), C, C, CCT2);
			s_mm(dynDimM(), 1, dynDimN(), C, ca, cca2);
			
			dlmwrite("C:\\Users\\py033\\Desktop\\CCT2.txt", CCT2, dynDimM(), dynDimM());
			dlmwrite("C:\\Users\\py033\\Desktop\\cca2.txt", cca2, dynDimM(), 1);

			if (s_is_equal(dynDimM()*dynDimM(), CCT, CCT2, 1e-10)) std::cout << "calculation correct!" << std::endl;
			if (s_is_equal(dynDimM(), cca, cca2, 1e-10)) std::cout << "calculation correct!" << std::endl;
			///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

			
			
			// following is calculation of SOR relaxation method:
			// x_i(k+1) = w * a_ii^(-1) * [ b_i - sigma_j=1_to_i-1(a_ij * x_j(k+1)) - sigma_j=i+1_to_n(a_ij * x_j(k)) ] + (1 - w) x_i(k)
			// x_i(k+1) = w * [ b_i - sigma_j=1_to_i-1(k_ij * x_j(k+1)) - sigma_j=i+1_to_n(k_ij * x_j(k)) ] + (1 - w) x_i(k)
			// where k_ij = a_ii^(-1) * a_ij
			for (std::size_t i = 0; i < active_prt_num; ++i)
			{
				s_mc(6, 6, block_D[i].get(), D + i * 36);
				s_mc(6, 6, block_D_inv[i].get(), D_inv + i * 36);
			}

			dlmwrite("C:\\Users\\py033\\Desktop\\D_inv.txt", D_inv, dynDimM(), 6);

			for (int k = 0; k<3000; ++k)
			{ 
				double w = 1.0;

				//for (std::size_t i = 0; i < active_prt_num; ++i)
				//{
				//	Eigen::Map<Eigen::Matrix<double, 6, 1> >x_k1_i(block_x_k1[i].get());
				//	Eigen::Map<Eigen::Matrix<double, 6, 1> >x_k_i(block_x_k[i].get());

				//	Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor> >D_inv(block_D_inv[i].get());
				//	Eigen::Map<Eigen::Matrix<double, 6, 1> >b_i(block_b[i].get());

				//	Eigen::Matrix<double, 6, 1> tem;
				//
				//	tem.fill(0);

				//	for (std::size_t j = 0; j < i; ++j)
				//	{
				//		if (!block_k[i][j])continue;

				//		Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor> > k_ij(block_k[i][j].get());
				//		Eigen::Map<Eigen::Matrix<double, 6, 1> > x_k1_j(block_x_k1[j].get());
				//	
				//		tem += k_ij*x_k1_j;
				//	}
				//	for (std::size_t j = i+1; j < active_prt_num; ++j)
				//	{
				//		if (!block_k[i][j])continue;
				//	
				//		Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor> > k_ij(block_k[i][j].get());
				//		Eigen::Map<Eigen::Matrix<double, 6, 1> > x_k_j(block_x_k[j].get());

				//		tem += k_ij*x_k_j;
				//	}

				//	x_k1_i = w * (b_i - tem) + (1.0 - w)*x_k_i;
				//}

				std::swap(block_x_k, block_x_k1);

				for (std::size_t i = 0; i < block_x_k.size(); ++i)
					s_mc(6, 1, block_x_k[i].get(), x + 6 * i);
			}

			dlmwrite("C:\\Users\\py033\\Desktop\\x.txt", x, dynDimM(), 1);

			dynPrtAs(x);
			dlmwrite("C:\\Users\\py033\\Desktop\\x2.txt", x, dynDimM(), 1);


			delete[]C;
			delete[]CT;
			delete[]CCT;
			delete[]CCT2;
			delete[]ca;
			delete[]cca;
			delete[]cca2;
			delete[]x;
		}
		auto Model::clbSetInverseMethod(std::function<void(int n, double *A)> inverse_method)->void
		{
			imp_->clb_inverse_method_ = inverse_method;
		}
		auto Model::clbDimM()const->std::size_t { return imp_->clb_dim_m_; }
		auto Model::clbDimN()const->std::size_t { return imp_->clb_dim_n_; }
		auto Model::clbDimGam()const->std::size_t { return imp_->clb_dim_gam_; }
		auto Model::clbDimFrc()const->std::size_t { return imp_->clb_dim_frc_; }
		auto Model::clbPre()->void
		{
			dynPre();

			if (dynDimN() != dynDimM()) { throw std::runtime_error("must calibrate square matrix"); }

			imp_->clb_dim_m_ = 0;
			imp_->clb_dim_n_ = 0;
			imp_->clb_dim_gam_ = 0;
			imp_->clb_dim_frc_ = 0;

			for (auto &mot : motionPool())
			{
				if (mot.active())
				{
					imp_->clb_dim_m_++;
					imp_->clb_dim_frc_ += 3;
					imp_->clb_dim_n_ += 3;
				}
			}
			for (auto &prt : partPool())
			{
				if (prt.active())
				{
					imp_->clb_dim_n_ += 10;
					imp_->clb_dim_gam_ += 10;
				}
			}

		}
		auto Model::clbUpd()->void
		{
			dynPrtUpd();
		}
		auto Model::clbMtx(double *clb_D, double *clb_b)const->void
		{
			if (!imp_->clb_inverse_method_)throw std::runtime_error("please set inverse method before calibrate");
			if (dynDimN() != dynDimM()) throw std::logic_error("must calibrate square matrix");

			// 初始化 //
			core::Matrix clb_d_m(clbDimM(), clbDimN());
			core::Matrix clb_b_m(clbDimM(), 1);

			// 求A,即C的逆 //
			core::Matrix A(dynDimM(), dynDimM()), B(dynDimM(), dynDimM());

			std::vector<double> C(dynDimM() * dynDimM());
			std::vector<double> f(dynDimM());

			dynPrtCm(C.data());
			std::copy(C.begin(), C.end(), A.data());
			imp_->clb_inverse_method_(dynDimM(), A.data());

			// 求B //
			const int beginRow = dynDimM() - clbDimM();

			for (auto &i : partPool())
			{
				if (i.active())
				{
					double cm[6][6];
					s_cmf(i.prtVs(), *cm);
					s_mm(clbDimM(), 6, 6, &A(beginRow, i.rowID()), dynDimM(), *cm, 6, &B(beginRow, i.rowID()), dynDimM());
				}
			}

			// 求解clb_d //
			int col1 = 0, col2 = 0;

			for (auto &i : partPool())
			{
				if (i.active())
				{
					double q[6]{ 0 };
					std::copy_n(i.prtAs(), 6, q);
					s_va(6, -1.0, i.prtGr(), q);

					double v[6];
					std::copy_n(i.prtVs(), 6, v);

					for (std::size_t j = 0; j < clbDimM(); ++j)
					{
						clb_d_m(j, col1) = A(beginRow + j, col2 + 0) * q[0] + A(beginRow + j, col2 + 1) * q[1] + A(beginRow + j, col2 + 2) * q[2];
						clb_d_m(j, col1 + 1) = A(beginRow + j, col2 + 1) * q[5] + A(beginRow + j, col2 + 5) * q[1] - A(beginRow + j, col2 + 2) * q[4] - A(beginRow + j, col2 + 4) * q[2];
						clb_d_m(j, col1 + 2) = A(beginRow + j, col2 + 2) * q[3] + A(beginRow + j, col2 + 3) * q[2] - A(beginRow + j, col2 + 0) * q[5] - A(beginRow + j, col2 + 5) * q[0];
						clb_d_m(j, col1 + 3) = A(beginRow + j, col2 + 0) * q[4] + A(beginRow + j, col2 + 4) * q[0] - A(beginRow + j, col2 + 1) * q[3] - A(beginRow + j, col2 + 3) * q[1];
						clb_d_m(j, col1 + 4) = A(beginRow + j, col2 + 3) * q[3];
						clb_d_m(j, col1 + 5) = A(beginRow + j, col2 + 4) * q[4];
						clb_d_m(j, col1 + 6) = A(beginRow + j, col2 + 5) * q[5];
						clb_d_m(j, col1 + 7) = A(beginRow + j, col2 + 3) * q[4] + A(beginRow + j, col2 + 4) * q[3];
						clb_d_m(j, col1 + 8) = A(beginRow + j, col2 + 3) * q[5] + A(beginRow + j, col2 + 5) * q[3];
						clb_d_m(j, col1 + 9) = A(beginRow + j, col2 + 4) * q[5] + A(beginRow + j, col2 + 5) * q[4];

						clb_d_m(j, col1) += B(beginRow + j, col2 + 0) * v[0] + B(beginRow + j, col2 + 1) * v[1] + B(beginRow + j, col2 + 2) * v[2];
						clb_d_m(j, col1 + 1) += B(beginRow + j, col2 + 1) * v[5] + B(beginRow + j, col2 + 5) * v[1] - B(beginRow + j, col2 + 2) * v[4] - B(beginRow + j, col2 + 4) * v[2];
						clb_d_m(j, col1 + 2) += B(beginRow + j, col2 + 2) * v[3] + B(beginRow + j, col2 + 3) * v[2] - B(beginRow + j, col2 + 0) * v[5] - B(beginRow + j, col2 + 5) * v[0];
						clb_d_m(j, col1 + 3) += B(beginRow + j, col2 + 0) * v[4] + B(beginRow + j, col2 + 4) * v[0] - B(beginRow + j, col2 + 1) * v[3] - B(beginRow + j, col2 + 3) * v[1];
						clb_d_m(j, col1 + 4) += B(beginRow + j, col2 + 3) * v[3];
						clb_d_m(j, col1 + 5) += B(beginRow + j, col2 + 4) * v[4];
						clb_d_m(j, col1 + 6) += B(beginRow + j, col2 + 5) * v[5];
						clb_d_m(j, col1 + 7) += B(beginRow + j, col2 + 3) * v[4] + B(beginRow + j, col2 + 4) * v[3];
						clb_d_m(j, col1 + 8) += B(beginRow + j, col2 + 3) * v[5] + B(beginRow + j, col2 + 5) * v[3];
						clb_d_m(j, col1 + 9) += B(beginRow + j, col2 + 4) * v[5] + B(beginRow + j, col2 + 5) * v[4];
					}
					col1 += 10;
					col2 += 6;
				}
			}

			// 求解clb_b //
			std::fill(f.begin(), f.end(), 0);
			int row = 0;
			for (auto &mot : motionPool())
			{
				if (mot.active())
				{
					clb_b_m(row, 0) = mot.mf();
					++row;
				}
			}
			for (auto &fce : forcePool())
			{
				if (fce.active())
				{
					s_va(6, fce.fsI(), &f[fce.makI().fatherPart().rowID()]);
					s_va(6, fce.fsJ(), &f[fce.makJ().fatherPart().rowID()]);
				}
			}
			s_mma(clbDimM(), 1, dynDimM(), 1.0, &A(beginRow, 0), dynDimM(), f.data(), 1, clb_b_m.data(), 1);

			// 以下添加驱动摩擦系数 //
			row = 0;
			for (auto &mot : motionPool())
			{
				//默认未激活的motion处于力控模式
				if (mot.active())
				{
					clb_d_m(row, clbDimGam() + row * 3) += s_sgn(mot.mv());
					clb_d_m(row, clbDimGam() + row * 3 + 1) += mot.mv();
					clb_d_m(row, clbDimGam() + row * 3 + 2) += mot.ma();
					++row;
				}
			}

			std::copy_n(clb_d_m.data(), clb_d_m.size(), clb_D);
			std::copy_n(clb_b_m.data(), clb_b_m.size(), clb_b);
		}
		auto Model::clbUkn(double *clb_x)const->void
		{
			int row = 0;
			for (auto &prt : partPool())
			{
				if (prt.active())
				{
					s_is2iv(*prt.prtIs(), clb_x + row);
					row += 10;
				}
			}

			for (auto &mot : motionPool())
			{
				if (mot.active())
				{
					std::copy_n(mot.frcCoe(), 3, clb_x + row);
					row += 3;
				}
			}
		}
		auto Model::simKin(const PlanFunc &func, const PlanParamBase &param, std::size_t akima_interval)->SimResult
		{
			//初始化变量
			SimResult result;
			result.resize(motionPool().size());
			std::list<double> time_akima_data;
			std::vector<std::list<double> > pos_akima_data(motionPool().size());
			std::vector<std::list<double> > pos_akima_data_gm(generalMotionPool().size() * 6);

			//起始位置
			result.time_.push_back(0);
			time_akima_data.push_back(0);
			for (std::size_t i = 0; i < motionPool().size(); ++i)
			{
				motionPool().at(i).updMp();
				result.Pin_.at(i).push_back(motionPool().at(i).mp());
				pos_akima_data.at(i).push_back(motionPool().at(i).mp());
			}

			for (std::size_t i = 0; i < generalMotionPool().size(); ++i)
			{
				for (std::size_t j = 0; j < generalMotionPool().at(i).dim(); ++j)
				{
					pos_akima_data_gm.at(i * 6 + j).push_back(generalMotionPool().at(i).mp()[j]);
				}
			}
			

			//其他位置
			for (param.count_ = 0; true; ++param.count_)
			{
				auto is_sim = func(*this, param);

				result.time_.push_back(param.count_ + 1);
				for (std::size_t i = 0; i < motionPool().size(); ++i)
				{
					result.Pin_.at(i).push_back(motionPool().at(i).mp());
				}

				if ((!is_sim) || ((param.count_ + 1) % akima_interval == 0))
				{
					time_akima_data.push_back((param.count_ + 1) / 1000.0);

					for (std::size_t j = 0; j < motionPool().size(); ++j)
					{
						pos_akima_data.at(j).push_back(motionPool().at(j).mp());
					}

					for (std::size_t i = 0; i < generalMotionPool().size(); ++i)
					{
						for (std::size_t j = 0; j < generalMotionPool().at(i).dim(); ++j)
						{
							pos_akima_data_gm.at(i * 6 + j).push_back(generalMotionPool().at(i).mp()[j]);
						}
					}
				}

				if (!is_sim)break;
			}

			//使用Akima储存motion的位置数据
			for (std::size_t i = 0; i < motionPool().size(); ++i)
			{
				auto aki = akimaPool().findByName(motionPool().at(i).name() + "_akima");

				if (aki == akimaPool().end())
				{
					throw std::runtime_error("SimKin require motion akima element");
				}
				else
				{
					aki->operator=(Akima(aki->name(), time_akima_data, pos_akima_data.at(i)));
				}
			}

			//使用Akima储存GeneralMotion的位置数据
			for (std::size_t i = 0; i < generalMotionPool().size(); ++i)
			{
				std::string names[6]{ "_t1_akima", "_t2_akima", "_t3_akima", "_r1_akima", "_r2_akima", "_r3_akima" };
				
				for (std::size_t j = 0; j < generalMotionPool().at(i).dim(); ++j)
				{
					auto aki = akimaPool().findByName(generalMotionPool().at(i).name() + names[j]);
					if (aki == akimaPool().end())
					{
						throw std::runtime_error("SimKin require general motion akima element");
					}
					else
					{
						aki->operator=(Akima(aki->name(), time_akima_data, pos_akima_data_gm.at(i * 6 + j)));
					}
				}
			}

			return std::move(result);
		}
		auto Model::simDyn(const PlanFunc &func, const PlanParamBase &param, std::size_t akima_interval, Script *script)->SimResult
		{
			saveDynEle("before_simDyn_state");
			auto result = simKin(func, param, akima_interval);
			loadDynEle("before_simDyn_state");

			result.Pin_.clear();
			result.Vin_.clear();
			result.Ain_.clear();
			result.Fin_.clear();

			result.Pin_.resize(motionPool().size());
			result.Vin_.resize(motionPool().size());
			result.Ain_.resize(motionPool().size());
			result.Fin_.resize(motionPool().size());

			//仿真计算
			for (std::size_t t = 0; t < result.time_.size(); ++t)
			{
				if (t % 100 == 0)std::cout << t << std::endl;

				if (script)script->doScript(t, t + 1);

				dynPre();

				for (std::size_t j = 0; j < motionPool().size(); ++j)
				{
					motionPool().at(j).setMp(akimaPool().at(j).operator()(t / 1000.0, '0'));
				}
				kinFromPin();
				for (std::size_t j = 0; j < motionPool().size(); ++j)
				{
					motionPool().at(j).setMv(akimaPool().at(j).operator()(t / 1000.0, '1'));
				}
				kinFromVin();
				for (std::size_t j = 0; j < motionPool().size(); ++j)
				{
					motionPool().at(j).setMa(akimaPool().at(j).operator()(t / 1000.0, '2'));
				}
				dynPrt();
				for (std::size_t j = 0; j < motionPool().size(); ++j)
				{
					result.Fin_.at(j).push_back(motionPool().at(j).mfDyn());
					result.Pin_.at(j).push_back(motionPool().at(j).mp());
					result.Vin_.at(j).push_back(motionPool().at(j).mv());
					result.Ain_.at(j).push_back(motionPool().at(j).ma());
				}
			}

			return std::move(result);
		}
		auto Model::simToAdams(const std::string &filename, const PlanFunc &func, const PlanParamBase &param, int ms_dt, Script *script)->SimResult
		{
			saveDynEle("before_simToAdams_state");
			auto result = simDyn(func, param, ms_dt, script);
			loadDynEle("before_simToAdams_state");

			this->saveAdams(filename, true);
			return std::move(result);
		}
		Model::~Model() = default;
		Model::Model(const std::string &name):Root(name)
		{
			registerChildType<Environment>();

			registerChildType<aris::core::ObjectPool<Variable, Element>>();
			registerChildType<MatrixVariable>();
			registerChildType<StringVariable>();

			registerChildType<aris::core::ObjectPool<Akima, Element>>();
			registerChildType<Akima>();

			registerChildType<aris::core::ObjectPool<Script, Element>>();
			registerChildType<Script>();

			registerChildType<aris::core::ObjectPool<Part, Element>>();
			registerChildType<Part>();

			registerChildType<aris::core::ObjectPool<Marker, Element>>();
			registerChildType<Marker>();

			registerChildType<aris::core::ObjectPool<Joint, Element>>();
			registerChildType<RevoluteJoint>();
			registerChildType<PrismaticJoint>();
			registerChildType<UniversalJoint>();
			registerChildType<SphericalJoint>();

			registerChildType<aris::core::ObjectPool<Motion, Element>>();
			registerChildType<Motion>();

			registerChildType<aris::core::ObjectPool<GeneralMotion, Element>>();
			registerChildType<GeneralMotion>();

			registerChildType<aris::core::ObjectPool<Force, Element>>();
			registerChildType<SingleComponentForce>();


			imp_->environment_ = &this->add<Environment>("environment");
			imp_->script_pool_ = &this->add<aris::core::ObjectPool<Script, Element>>("script");
			imp_->variable_pool_ = &this->add<aris::core::ObjectPool<Variable, Element>>("variable_pool");
			imp_->akima_pool_ = &this->add<aris::core::ObjectPool<Akima, Element>>("akima_pool");
			imp_->part_pool_ = &this->add<aris::core::ObjectPool<Part, Element>>("part_pool");
			imp_->joint_pool_ = &this->add<aris::core::ObjectPool<Joint, Element>>("joint_pool");
			imp_->motion_pool_ = &this->add<aris::core::ObjectPool<Motion, Element>>("motion_pool");
			imp_->general_motion_pool_ = &this->add<aris::core::ObjectPool<GeneralMotion, Element>>("general_motion_pool");
			imp_->force_pool_ = &this->add<aris::core::ObjectPool<Force, Element>>("force_pool");
			imp_->ground_ = &imp_->part_pool_->add<Part>("ground");
		}
		
		RevoluteJoint::RevoluteJoint(const std::string &name, Marker &makI, Marker &makJ): JointTemplate(name, makI, makJ)
		{
			const static double loc_cst[6][Dim()]
			{
				1,0,0,0,0,
				0,1,0,0,0,
				0,0,1,0,0,
				0,0,0,1,0,
				0,0,0,0,1,
				0,0,0,0,0
			};

			s_tf_n(Dim(), *this->makI().prtPm(), *loc_cst, *prtCmI_);
		}
		RevoluteJoint::RevoluteJoint(Object &father, const aris::core::XmlElement &xml_ele): JointTemplate(father, xml_ele)
		{
			const static double loc_cst[6][Dim()]
			{
				1,0,0,0,0,
				0,1,0,0,0,
				0,0,1,0,0,
				0,0,0,1,0,
				0,0,0,0,1,
				0,0,0,0,0
			};

			s_tf_n(Dim(), *this->makI().prtPm(), *loc_cst, *prtCmI_);
		}

		PrismaticJoint::PrismaticJoint(const std::string &name, Marker &makI, Marker &makJ): JointTemplate(name, makI, makJ)
		{
			const static double loc_cst[6][Dim()]
			{
				1,0,0,0,0,
				0,1,0,0,0,
				0,0,0,0,0,
				0,0,1,0,0,
				0,0,0,1,0,
				0,0,0,0,1
			};

			s_tf_n(Dim(), *this->makI().prtPm(), *loc_cst, *prtCmI_);
		}
		PrismaticJoint::PrismaticJoint(Object &father, const aris::core::XmlElement &xml_ele): JointTemplate(father, xml_ele)
		{
			const static double loc_cst[6][Dim()]
			{
				1,0,0,0,0,
				0,1,0,0,0,
				0,0,0,0,0,
				0,0,1,0,0,
				0,0,0,1,0,
				0,0,0,0,1
			};

			s_tf_n(Dim(), *this->makI().prtPm(), *loc_cst, *prtCmI_);
		}

		auto UniversalJoint::cptCa(double *ca)const->void
		{
			Constraint::cptCa(ca);

			double tem[6];

			// update makI的z轴 和 makJ的z轴
			const double axis_i_m[3]{ makI().prtPm()[0][2] ,makI().prtPm()[1][2] ,makI().prtPm()[2][2] };
			double axis_j_m[3];
			s_pm_dot_v3(*makJ().fatherPart().pm(), &makJ().prtPm()[0][2], 4, tem, 1);
			s_inv_pm_dot_v3(*makI().fatherPart().pm(), tem, axis_j_m);

			// compute c_dot //
			double wm_in_m[3], wn_in_m[3];
			s_inv_pm_dot_v3(*makI().fatherPart().pm(), makI().fatherPart().glbVs() + 3, wm_in_m);
			s_inv_pm_dot_v3(*makI().fatherPart().pm(), makJ().fatherPart().glbVs() + 3, wn_in_m);

			double iwm = s_vv(3, axis_i_m, wm_in_m);
			double jwm = s_vv(3, axis_j_m, wm_in_m);
			double iwn = s_vv(3, axis_i_m, wn_in_m);
			double jwn = s_vv(3, axis_j_m, wn_in_m);

			ca[3] += 2 * jwm*iwn - jwm*iwm - jwn*iwn;
		}
		auto UniversalJoint::cptCp(double *cp)const->void
		{
			double pm_t[16];

			double origin[3]{ 0,0,0 };

			const double *pm_i = *makI().pm();
			const double *pm_j = *makJ().pm();

			double v[3], new_z[3];
			s_c3(pm_i + 2, 4, pm_j + 2, 4, v, 1);
			if (s_norm(3, v) == 0)s_vc(3, pm_i, 4, new_z, 1);
			else s_c3(v, 1, pm_i + 2, 4, new_z, 1);
			
			s_sov_axes2pm(origin, 1, new_z, 1, pm_j, 4, pm_t, "zx");
			s_vc(3, pm_i + 3, 4, pm_t + 3, 4);

			double pq_j2i[7];
			double pm_j2i[4][4];
			double diff[6];

			s_inv_pm_dot_pm(pm_t, pm_j, *pm_j2i);
			s_pm2pq(*pm_j2i, pq_j2i);

			double theta = atan2(s_norm(3, pq_j2i + 3, 1), pq_j2i[6]) * 2;

			if (theta < 1e-3)
			{
				s_nv(3, 2.0, pq_j2i + 3);
			}
			else
			{
				s_nv(3, theta / std::sin(theta / 2.0), pq_j2i + 3);
			}

			// 此时位移差值在makI()坐标系中。需要转换到部件坐标系下。
			double diff2[6];
			s_tv(pm_t, pq_j2i, diff2);
			s_inv_tv(*makI().fatherPart().pm(), diff2, diff);

			s_mmTN(dim(), 1, 6, prtCmPtrI(), diff, cp);

		}
		auto UniversalJoint::cptPrtCm(double *prt_cmI, double *prt_cmJ, int cmI_ld, int cmJ_ld)const->void
		{
			cmI_ld = std::max(cmI_ld, static_cast<int>(dim()));
			cmJ_ld = std::max(cmJ_ld, static_cast<int>(dim()));
			
			double tem[3];
			const double axis_i_m[3]{ makI().prtPm()[0][2] ,makI().prtPm()[1][2] ,makI().prtPm()[2][2] };
			double axis_j_m[3];
			s_pm_dot_v3(*makJ().fatherPart().pm(), &makJ().prtPm()[0][2], 4, tem, 1);
			s_inv_pm_dot_v3(*makI().fatherPart().pm(), tem, axis_j_m);

			s_c3(axis_i_m, 1, axis_j_m, 1, const_cast<double*>(prtCmPtrI()) + 3 * 4 + 3, 4);
			
			Constraint::cptPrtCm(prt_cmI, prt_cmJ, cmI_ld, cmJ_ld);
		}
		auto UniversalJoint::cptGlbCm(double *glb_cmI, double *glb_cmJ, int cmI_ld, int cmJ_ld)const->void
		{
			cmI_ld = std::max(cmI_ld, static_cast<int>(dim()));
			cmJ_ld = std::max(cmJ_ld, static_cast<int>(dim()));
			
			double axis_i_g[3], axis_j_g[3];
			s_pm_dot_v3(*makI().fatherPart().pm(), &makI().prtPm()[0][2], 4, axis_i_g, 1);
			s_pm_dot_v3(*makJ().fatherPart().pm(), &makJ().prtPm()[0][2], 4, axis_j_g, 1);

			s_fill(3, 1, 0.0, glb_cmI + 3, cmI_ld);
			s_c3(axis_i_g, 1, axis_j_g, 1, glb_cmI + 3 * cmI_ld + 3, cmI_ld);
			
			s_tf_n(3, *makI().fatherPart().pm(), prtCmPtrI(), dim(), glb_cmI, cmI_ld);

			s_mc(6, 4, -1.0, glb_cmI, cmI_ld, glb_cmJ, cmJ_ld);
		}
		auto UniversalJoint::updPrtCm()->void
		{
			// 在零位时，makI的x、y、z轴分别对应makJ的z，x，y轴。转轴分别为makI的z轴和makJ的z轴

			// update cmI
			// makI的z轴 和 makJ的z轴
			double tem[3];
			const double axis_i_m[3]{ makI().prtPm()[0][2] ,makI().prtPm()[1][2] ,makI().prtPm()[2][2] };
			double axis_j_m[3];
			s_pm_dot_v3(*makJ().fatherPart().pm(), &makJ().prtPm()[0][2], 4, tem, 1);
			s_inv_pm_dot_v3(*makI().fatherPart().pm(), tem, axis_j_m);

			s_c3(axis_i_m, 1, axis_j_m, 1, const_cast<double*>(&prtCmI()[3][3]), 4);

			// // update csmJ //
			Constraint::updPrtCm();
		}
		auto UniversalJoint::updGlbCm()->void
		{
			double axis_i_g[3], axis_j_g[3];
			s_pm_dot_v3(*makI().fatherPart().pm(), &makI().prtPm()[0][2], 4, axis_i_g, 1);
			s_pm_dot_v3(*makJ().fatherPart().pm(), &makJ().prtPm()[0][2], 4, axis_j_g, 1);

			s_fill(3, 1, 0.0, const_cast<double*>(&glbCmI()[0][3]), 4);
			s_c3(axis_i_g, 1, axis_j_g, 1, const_cast<double*>(&glbCmI()[3][3]), 4);

			s_tf_n(3, *makI().fatherPart().pm(), prtCmPtrI(), 4, const_cast<double*>(glbCmPtrI()), 4);
			s_mc(6, dim(), -1.0, glbCmPtrI(), const_cast<double*>(glbCmPtrJ()));
		}
		auto UniversalJoint::updCa()->void
		{
			Constraint::updCa();
			
			double tem[6];
			
			// update makI的z轴 和 makJ的z轴
			const double axis_i_m[3]{ makI().prtPm()[0][2] ,makI().prtPm()[1][2] ,makI().prtPm()[2][2] };
			double axis_j_m[3];
			s_pm_dot_v3(*makJ().fatherPart().pm(), &makJ().prtPm()[0][2], 4, tem, 1);
			s_inv_pm_dot_v3(*makI().fatherPart().pm(), tem, axis_j_m);

			// compute c_dot //
			double wm_in_m[3], wn_in_m[3];
			s_inv_pm_dot_v3(*makI().fatherPart().pm(), makI().fatherPart().glbVs() + 3, wm_in_m);
			s_inv_pm_dot_v3(*makI().fatherPart().pm(), makJ().fatherPart().glbVs() + 3, wn_in_m);

			double iwm = s_vv(3, axis_i_m, wm_in_m);
			double jwm = s_vv(3, axis_j_m, wm_in_m);
			double iwn = s_vv(3, axis_i_m, wn_in_m);
			double jwn = s_vv(3, axis_j_m, wn_in_m);

			ca_[3] += 2*jwm*iwn - jwm*iwm -jwn*iwn;

		}
		UniversalJoint::UniversalJoint(const std::string &name, Marker &makI, Marker &makJ) : JointTemplate(name, makI, makJ)
		{
			const static double loc_cst[6][Dim()]
			{
				1,0,0,0,
				0,1,0,0,
				0,0,1,0,
				0,0,0,0,
				0,0,0,0,
				0,0,0,0,
			};

			s_tf_n(Dim(), *this->makI().prtPm(), *loc_cst, *prtCmI_);
		}
		UniversalJoint::UniversalJoint(Object &father, const aris::core::XmlElement &xml_ele) : JointTemplate(father, xml_ele)
		{
			const static double loc_cst[6][Dim()]
			{
				1,0,0,0,
				0,1,0,0,
				0,0,1,0,
				0,0,0,0,
				0,0,0,0,
				0,0,0,0,
			};

			s_tf_n(Dim(), *this->makI().prtPm(), *loc_cst, *prtCmI_);
		}

		SphericalJoint::SphericalJoint(const std::string &name, Marker &makI, Marker &makJ): JointTemplate(name, makI, makJ)
		{
			const static double loc_cst[6][Dim()]
			{
				1,0,0,
				0,1,0,
				0,0,1,
				0,0,0,
				0,0,0,
				0,0,0,
			};

			s_tf_n(Dim(), *this->makI().prtPm(), *loc_cst, *prtCmI_);
		}
		SphericalJoint::SphericalJoint(Object &father, const aris::core::XmlElement &xml_ele): JointTemplate(father, xml_ele)
		{
			const static double loc_cst[6][Dim()]
			{
				1,0,0,
				0,1,0,
				0,0,1,
				0,0,0,
				0,0,0,
				0,0,0,
			};

			s_tf_n(Dim(), *this->makI().prtPm(), *loc_cst, *prtCmI_);
		}

		auto SingleComponentForce::saveXml(aris::core::XmlElement &xml_ele) const->void
		{
			Force::saveXml(xml_ele);
			xml_ele.SetAttribute("component", this->component_axis_);
		}
		auto SingleComponentForce::saveAdams(std::ofstream &file) const->void
		{
			if (true)
			{
				std::string type = "translational";

				file << "force create direct single_component_force  &\r\n"
					<< "    single_component_force_name = ." << model().name() << "." << name() << "  &\r\n"
					<< "    adams_id = " << adamsID() << "  &\r\n"
					<< "    type_of_freedom = " << type << "  &\r\n"
					<< "    i_marker_name = ." << model().name() << "." << makI().fatherPart().name() << "." << makI().name() << "  &\r\n"
					<< "    j_marker_name = ." << model().name() << "." << makJ().fatherPart().name() << "." << makJ().name() << "  &\r\n"
					<< "    action_only = off  &\r\n"
					<< "    function = \"" << fce() << "\"  \r\n"
					<< "!\r\n";
			}
			else
			{
				std::string type = "translational";



				file << "force create direct single_component_force  &\r\n"
					<< "    single_component_force_name = ." << model().name() << "." << name() << "  &\r\n"
					<< "    adams_id = " << adamsID() << "  &\r\n"
					<< "    type_of_freedom = " << type << "  &\r\n"
					<< "    i_marker_name = ." << model().name() << "." << makI().fatherPart().name() << "." << makI().name() << "  &\r\n"
					<< "    j_marker_name = ." << model().name() << "." << makJ().fatherPart().name() << "." << makJ().name() << "  &\r\n"
					<< "    action_only = off  &\r\n"
					<< "    function = \"AKISPL(time,0," << name() << "_fce_spl)\"  \r\n"
					<< "!\r\n";
			}
		}
		auto SingleComponentForce::updFs()->void
		{
			s_tf(*makI().prtPm(), fce_value_, fsI_);
			double pm_M2N[16];
			s_inv_pm_dot_pm(*makJ().fatherPart().pm(), *makI().fatherPart().pm(), pm_M2N);
			s_tf(-1.0, pm_M2N, fsI_, fsJ_);
		}
		SingleComponentForce::SingleComponentForce(const std::string &name, Marker& makI, Marker& makJ, int componentID) : Force(name, makI, makJ), component_axis_(componentID) {}
		SingleComponentForce::SingleComponentForce(Object &father, const aris::core::XmlElement &xml_ele) : Force(father, xml_ele), component_axis_(attributeInt32(xml_ele, "component")) {}
	}
}
