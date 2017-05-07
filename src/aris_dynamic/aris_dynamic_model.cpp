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
#include "aris_dynamic_matrix.h"
#include "aris_dynamic_screw.h"
#include "aris_dynamic_model.h"


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
		auto Element::attributeMatrix(const aris::core::XmlElement &xml_ele, const std::string &attribute_name, Size m, Size n)const->aris::core::Matrix
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
		auto Element::attributeMatrix(const aris::core::XmlElement &xml_ele, const std::string &attribute_name, Size m, Size n, const aris::core::Matrix& default_value)const->aris::core::Matrix
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
			if (wa)	{ s_vs2wa(vs(), wa);}
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
		
		struct Constraint::Imp { Size col_id_, blk_col_id_; };
		auto Constraint::cptCp(double *cp)const->void
		{
			double pq_j2i[7];
			double pm_j2i[4][4];
			double diff[6];

			s_inv_pm_dot_pm(*makI().pm(), *makJ().pm(), *pm_j2i);
			s_pm2pq(*pm_j2i, pq_j2i);

			double theta = atan2(s_norm(3, pq_j2i + 3, 1), pq_j2i[6]) * 2;

			double coe = theta < 1e-3 ? 2.0 : theta / std::sin(theta / 2.0);
			s_nv(3, coe, pq_j2i + 3);

			// 此时位移差值在makI()坐标系中。需要转换到部件坐标系下。
			s_tv(*makI().prtPm(), pq_j2i, diff);

			s_mm(dim(), 1, 6, prtCmPtrI(), ColMajor{ dim() }, diff, 1, cp, 1);
		}
		auto Constraint::cptCv(double *cv)const->void { std::fill(cv, cv + dim(), 0.0); };
		auto Constraint::cptCa(double *ca)const->void
		{
			double vm_cross_vn[6], tem[6];
			s_cv(makI().fatherPart().glbVs(), makJ().fatherPart().glbVs(), vm_cross_vn);
			s_inv_tv(*makI().fatherPart().pm(), vm_cross_vn, tem);
			s_mm(dim(), 1, 6, -1.0, prtCmPtrI(), ColMajor{ dim() }, tem, 1, ca, 1);
		}
		auto Constraint::cptPrtCm(double *prt_cmI, double *prt_cmJ, Size cmI_ld, Size cmJ_ld)const->void
		{
			cmI_ld = std::max(cmI_ld, static_cast<Size>(dim()));
			cmJ_ld = std::max(cmJ_ld, static_cast<Size>(dim()));
			
			s_mc(6, dim(), const_cast<double*>(prtCmPtrI()), dim(), prt_cmI, cmI_ld);
			
			double pm_M2N[4][4];
			s_inv_pm_dot_pm(*makJ().fatherPart().pm(), *makI().fatherPart().pm(), *pm_M2N);
			s_tf_n(dim(), -1.0, *pm_M2N, prtCmPtrI(), dim(), prt_cmJ, cmJ_ld);
		}
		auto Constraint::cptGlbCm(double *glb_cmI, double *glb_cmJ, Size cmI_ld, Size cmJ_ld)const->void
		{
			cmI_ld = std::max(cmI_ld, static_cast<Size>(dim()));
			cmJ_ld = std::max(cmJ_ld, static_cast<Size>(dim()));
			
			s_tf_n(dim(), *makI().fatherPart().pm(), prtCmPtrI(), dim(), glb_cmI, cmI_ld);
			s_mc(6, dim(), -1.0, glb_cmI, cmI_ld, glb_cmJ, cmJ_ld);
		}
		auto Constraint::setCf(const double *cf)const->void { std::copy(cf, cf + dim(), const_cast<double *>(cfPtr())); }
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
			s_mm(dim(), 1, 6, -1.0, prtCmPtrI(), ColMajor{ dim() }, tem, 1, const_cast<double*>(caPtr()), 1);
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
			
			s_mm(dim(), 1, 6, glbCmPtrI(), ColMajor{ dim() }, diff, 1, const_cast<double*>(cePtr()), 1);
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

			Size id = std::max<Size>(bIn - imp_->x_.begin() - 1, 0);

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
		auto Akima::operator()(Size length, const double *x_in, double *y_out, char order)const->void
		{
			for (Size i = 0; i < length; ++i)
			{
				y_out[i] = this->operator()(x_in[i], order);
			}
		}
		Akima::~Akima() = default;
		Akima::Akima(const std::string &name, Size num, const double *x_in, const double *y_in): Element(name)
		{
			std::list<std::pair<double, double> > data_list;

			for (Size i = 0; i < num; ++i)
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

			for (Size i = 0; i < x_in.size(); ++i)
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

			for (Size i = 0; i < data_list.size() - 1; ++i)
			{
				s[i + 2] = (imp_->y_[i + 1] - imp_->y_[i]) / (imp_->x_[i + 1] - imp_->x_[i]);
			}

			s[1] = 2 * s[2] - s[3];
			s[0] = 2 * s[1] - s[2];
			s[data_list.size() + 1] = 2 * s[data_list.size()] - s[data_list.size() - 1];
			s[data_list.size() + 2] = 2 * s[data_list.size() + 1] - s[data_list.size()];

			for (Size i = 0; i < data_list.size() + 2; ++i)
			{
				ds[i] = std::abs(s[i + 1] - s[i]);
			}

			for (Size i = 0; i < data_list.size(); ++i)
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

			for (Size i = 0; i < data_list.size() - 1; ++i)
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

			for (Size i = 0; i < data_list.size() - 1; ++i)
			{
				s[i + 2] = (imp_->y_[i + 1] - imp_->y_[i]) / (imp_->x_[i + 1] - imp_->x_[i]);
			}
			///////// this part is different
			s[1] = begin_slope;
			s[0] = begin_slope;
			s[data_list.size() + 1] = end_slope;
			s[data_list.size() + 2] = end_slope;
			///////// this part is different end
			for (Size i = 0; i < data_list.size() + 2; ++i)
			{
				ds[i] = std::abs(s[i + 1] - s[i]);
			}

			for (Size i = 0; i < data_list.size(); ++i)
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

			for (Size i = 0; i < data_list.size() - 1; ++i)
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
				auto virtual msConsumed()const->Size { return 0; }

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
					Size id;

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

						Size size{ 0 };
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
				auto virtual msConsumed()const->Size override final { return ms_dur_; }
				auto virtual adamsScript()const->std::string override final
				{
					std::stringstream ss;
					ss << "simulate/transient, dur=" << double(ms_dur_) / 1000.0 << ", dtout=" << double(ms_dt_) / 1000.0;
					return std::move(ss.str());
				}

				explicit SimNode(Size ms_dur, Size ms_dt) :ms_dur_(ms_dur), ms_dt_(ms_dt) { }
				Size ms_dur_;
				Size ms_dt_;
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
		auto Script::sim(Size ms_dur, Size ms_dt)->void
		{
			imp_->node_list_.push_back(std::unique_ptr<Imp::Node>(new Imp::SimNode(ms_dur, ms_dt)));
		}
		auto Script::empty() const->bool { return imp_->node_list_.empty(); }
		auto Script::endTime()const->Size
		{
			Size end_time{ 0 };
			for (auto& node : imp_->node_list_)end_time += node->msConsumed();
			return end_time;
		}
		auto Script::doScript(Size ms_begin, Size ms_end)->void
		{
			Size now{ 0 };
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

		auto Geometry::fatherPart() const->const Part&{ return static_cast<const Part &>(this->father().father()); }
		auto Geometry::fatherPart()->Part& { return static_cast<Part &>(this->father().father()); }

		struct Marker::Imp
		{
			double prt_pm_[4][4]{ { 0 } };
			double pm_[4][4]{ { 0 } };
		};
		auto Marker::adamsID()const->Size 
		{
			Size size{ 0 };

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
		Marker::~Marker() = default;
		Marker::Marker(const Marker&) = default;
		Marker::Marker(Marker&&) = default;
		Marker& Marker::operator=(const Marker&) = default;
		Marker& Marker::operator=(Marker&&) = default;
		Marker::Marker(Object &father, const aris::core::XmlElement &xml_ele) : Coordinate(father, xml_ele)
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
				s_vc(16, pm, *imp_->prt_pm_);
			}
		}
		Marker::Marker(const std::string &name, const double *prt_pm, bool active): Coordinate(name, active)
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
		auto Part::adamsID()const->Size { return (this == &this->model().ground()) ? 1 : id() + (this->model().ground().id() < id() ? 1 : 2); }
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


				double mass = this->prtIm()[0][0] == 0 ? 1 : prtIm()[0][0];
				std::fill_n(pe, 6, 0);
				pe[0] = this->prtIm()[1][5] / mass;
				pe[1] = -this->prtIm()[0][5] / mass;
				pe[2] = this->prtIm()[0][4] / mass;

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
				s_im2im(pm, *this->prtIm(), *im);

				//！注意！//
				//Adams里对惯量矩阵的定义貌似和我自己的定义在Ixy,Ixz,Iyz上互为相反数。别问我为什么,我也不知道。
				file << "part create rigid_body mass_properties  &\r\n"
					<< "    part_name = ." << model().name() << "." << this->name() << "  &\r\n"
					<< "    mass = " << this->prtIm()[0][0] << "  &\r\n"
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
			this->geometryPool().saveAdams(file);
		}
		auto Part::markerPool()->aris::core::ObjectPool<Marker, Element>& { return *imp_->marker_pool_; }
		auto Part::markerPool()const->const aris::core::ObjectPool<Marker, Element>& { return *imp_->marker_pool_; }
		auto Part::geometryPool()->aris::core::ObjectPool<Geometry, Element>& { return *imp_->geometry_pool_; }
		auto Part::geometryPool()const->const aris::core::ObjectPool<Geometry, Element>&{ return *imp_->geometry_pool_; }
		auto Part::cptGlbIm(double *im, Size ld)const->void
		{
			ld = std::max(ld, Size(6));
			
			double tem[36], tem2[36];
			s_tf_n(6, *pm(), *prtIm(), tem);
			s_mc(6, 6, tem, ColMajor{6}, tem2, 6);
			s_tf_n(6, *pm(), tem2, 6, im, ld);
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
			s_va(6, -1.0, fv, pf);
		}
		auto Part::cptPrtFg(double *fg)const->void
		{
			double prt_gr[3];
			s_inv_pm_dot_v3(*pm(), model().environment().gravity(), prt_gr);
			s_mm(6, 1, 3, *prtIm(), 6, prt_gr, 1, fg, 1);
		}
		auto Part::cptPrtFv(double *fv)const->void
		{
			double prt_vs[6], tem[6];
			s_inv_tv(*pm(), vs(), prt_vs);
			s_mm(6, 1, 6, *prtIm(), prt_vs, tem);
			s_cf(prt_vs, tem, fv);
		}
		auto Part::cptPrtPf(double *pf)const->void
		{
			double fv[6];
			cptPrtFv(fv);
			cptPrtFg(pf);
			s_va(6, -1.0, fv, pf);
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
			s_mm(6, 1, 3, *prtIm(), 6, prt_gr, 1, imp_->prt_fg_, 1);
		}
		auto Part::updPrtFv()->void 
		{
			double prt_vs[6], tem[6];
			s_inv_tv(*pm(), vs(), prt_vs);
			s_mm(6, 1, 6, *prtIm(), prt_vs, tem);
			s_cf(prt_vs, tem, imp_->prt_fv_);
		}
		auto Part::glbIm()const->const double6x6&{ return imp_->glb_im_; }
		auto Part::glbFg()const->const double6&{ return imp_->glb_fg_; }
		auto Part::glbFv()const->const double6&{ return imp_->glb_fv_; }
		auto Part::updGlbIm()->void
		{
			double tem[36], tem2[36];
			s_tf_n(6, *pm(), *prtIm(), tem);
			s_mc(6, 6, tem, ColMajor{6}, tem2, 6);
			s_tf_n(6, *pm(), tem2, *imp_->glb_im_);
		}
		auto Part::updGlbFg()->void 
		{ 
			double prt_gr[3], prt_fg[6];
			s_inv_pm_dot_v3(*pm(), model().environment().gravity(), prt_gr);
			s_mm(6, 1, 3, *prtIm(), 6, prt_gr, 1, prt_fg, 1);
			s_tf(*pm(), prt_fg, imp_->glb_fg_);
		}
		auto Part::updGlbFv()->void
		{
			double prt_vs[6], prt_fv[6], tem[6];
			s_inv_tv(*pm(), vs(), prt_vs);
			s_mm(6, 1, 6, *prtIm(), prt_vs, tem);
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
		auto Part::setRm(const double *rm, Size rm_ld)->void { if (rm)s_rm2pm(rm, *imp_->glb_pm_, rm_ld); }
		auto Part::setRm(const Coordinate &relative_to, const double *rm, Size rm_ld)->void{if (rm) s_rm2rm(*relative_to.pm(), rm, *imp_->glb_pm_, rm_ld, 4);}
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
		Part::Part(Object &father, const aris::core::XmlElement &xml_ele): Coordinate(father, xml_ele)
		{
			s_pe2pm(attributeMatrix(xml_ele, "pe", 1, 6).data(), *imp_->glb_pm_);
			std::copy_n(attributeMatrix(xml_ele, "vel", 1, 6).data(), 6, imp_->glb_vs_);
			std::copy_n(attributeMatrix(xml_ele, "acc", 1, 6).data(), 6, imp_->glb_as_);
			s_iv2im(attributeMatrix(xml_ele, "inertia", 1, 10).data(), *imp_->prt_im_);
			
			//if (xml_ele.Attribute("graphic_file_path"))
			//	imp_->graphic_file_path_ = model().calculator().evaluateExpression(xml_ele.Attribute("graphic_file_path"));

			imp_->marker_pool_ = findOrInsert<aris::core::ObjectPool<Marker, Element> >("marker_pool");
			imp_->geometry_pool_ = findOrInsert<aris::core::ObjectPool<Geometry, Element> >("geometry_pool");
		}
		Part::Part(const std::string &name, const double *im, const double *pm, const double *vs, const double *as, bool active)
			: Coordinate(name, active)
		{
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
			Size sla_id_{ 0 }, phy_id_{ 0 }, clb_frc_id_{ 0 }, clb_id_{0};
			Size component_axis_;
			double frc_coe_[3]{ 0,0,0 };
			double mp_{ 0 }, mv_{ 0 }, ma_{ 0 };
		};
		auto Motion::saveXml(aris::core::XmlElement &xml_ele) const->void
		{
			Constraint::saveXml(xml_ele);

			xml_ele.SetAttribute("slave_id", static_cast<int>(imp_->sla_id_));
			xml_ele.SetAttribute("frc_coe", core::Matrix(1, 3, this->frcCoe()).toString().c_str());
			xml_ele.SetAttribute("component", static_cast<int>(axis()));
		}
		auto Motion::saveAdams(std::ofstream &file) const->void
		{
			std::string axis_names[6]{ "x","y","z","B1","B2","B3" };
			std::string axis_name = axis_names[axis()];
			
			std::string akima = name() + "_akima";
			std::string akima_func = "AKISPL(time,0," + akima + ")";
			std::string polynomial_func = static_cast<std::stringstream &>(std::stringstream() << std::setprecision(16) << mp() << " + " << mv() << " * time + " << ma()*0.5 << " * time * time").str();
			std::string func = model().akimaPool().findByName(akima) != model().akimaPool().end() ? akima_func : polynomial_func;

			file << "constraint create motion_generator &\r\n"
				<< "    motion_name = ." << model().name() << "." << this->name() << "  &\r\n"
				<< "    adams_id = " << adamsID() << "  &\r\n"
				<< "    i_marker_name = ." << model().name() << "." << this->makI().fatherPart().name() << "." << this->makI().name() << "  &\r\n"
				<< "    j_marker_name = ." << model().name() << "." << this->makJ().fatherPart().name() << "." << this->makJ().name() << "  &\r\n"
				<< "    axis = " << axis_name << "  &\r\n"
				<< "    function = \"" << func << "\"  \r\n"
				<< "!\r\n";
		}
		auto Motion::cptCp(double *cp)const->void{ Constraint::cptCp(cp); cp[0] += mp(); }
		auto Motion::cptCv(double *cv)const->void{ cv[0] = mv(); }
		auto Motion::cptCa(double *ca)const->void{ Constraint::cptCa(ca); ca[0] += ma(); }

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
		auto Motion::updMp()->void { setMp(s_sov_axis_distance(*makJ().pm(), *makI().pm(), axis())); }
		auto Motion::updMv()->void
		{
			double vs_i2j[6];
			makI().getVs(makJ(), vs_i2j);
			setMv(vs_i2j[axis()]);
		}
		auto Motion::updMa()->void
		{
			double as_i2j[6];
			makI().getAs(makJ(), as_i2j);
			setMa(as_i2j[axis()]);
		}
		auto Motion::absID()const->Size { return id(); }
		auto Motion::slaID()const->Size { return imp_->sla_id_; }
		auto Motion::phyID()const->Size { return imp_->phy_id_; }
		auto Motion::clbID()const->Size { return imp_->clb_id_; }
		auto Motion::clbFrcID()const->Size { return imp_->clb_frc_id_; }
		auto Motion::axis()const->Size { return imp_->component_axis_; }
		auto Motion::frcCoe()const->const double3& { return imp_->frc_coe_; }
		auto Motion::setFrcCoe(const double *frc_coe)->void { std::copy_n(frc_coe, 3, imp_->frc_coe_); }
		auto Motion::mp() const->double { return imp_->mp_; }
		auto Motion::setMp(double mp)->void { imp_->mp_ = mp; }
		auto Motion::mv() const->double { return imp_->mv_; }
		auto Motion::setMv(double mv)->void { imp_->mv_ = mv; }
		auto Motion::ma() const->double { return imp_->ma_; }
		auto Motion::setMa(double ma)->void { imp_->ma_ = ma; }
		auto Motion::mf() const->double { return mfDyn() + mfFrc(); }
		auto Motion::setMf(double mf)->void { cf_[0] = mf - mfFrc(); }
		auto Motion::mfDyn() const->double { return cf_[0]; }
		auto Motion::setMfDyn(double mf_dyn)->void { cf_[0] = mf_dyn; }
		auto Motion::mfFrc() const->double { return s_sgn(imp_->mv_)*frcCoe()[0] + imp_->mv_*frcCoe()[1] + imp_->ma_*frcCoe()[2]; }
		Motion::~Motion() = default;
		Motion::Motion(const std::string &name, Marker &makI, Marker &makJ, Size component_axis, const double *frc_coe, Size sla_id, bool active): Constraint(name, makI, makJ, active)
		{
			static const double default_frc_coe[3]{ 0,0,0 };
			frc_coe = frc_coe ? frc_coe : default_frc_coe;
			setFrcCoe(frc_coe);

			imp_->component_axis_ = component_axis;

			double loc_cst[6]{ 0,0,0,0,0,0, };
			loc_cst[axis()] = 1;
			s_tf(*this->makI().prtPm(), loc_cst, *prtCmI_);

			imp_->sla_id_ = sla_id;
		}
		Motion::Motion(Object &father, const aris::core::XmlElement &xml_ele) : Constraint(father, xml_ele)
		{
			setFrcCoe(attributeMatrix(xml_ele, "frc_coe", 1, 3).data());
			imp_->component_axis_ = attributeInt32(xml_ele, "component");

			double loc_cst[6]{ 0,0,0,0,0,0, };
			loc_cst[axis()] = 1;
			s_tf(*this->makI().prtPm(), loc_cst, *prtCmI_);

			imp_->sla_id_ = attributeInt32(xml_ele, "slave_id", -1);
		}
		Motion::Motion(const Motion &other) = default;
		Motion::Motion(Motion &&other) = default;
		Motion& Motion::operator=(const Motion &other) = default;
		Motion& Motion::operator=(Motion &&other) = default;

		struct GeneralMotion::Imp
		{
			double mpm_[4][4]{ {0} }, mvs_[6]{ {0} }, mas_[6]{ {0} };
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

			std::string axis_names[6]{ "t1", "t2", "t3", "r1", "r2", "r3" };

			double pe123[6], ve123[6], ae123[6];
			getMpe(pe123, "123");
			getMve(ve123, "123");
			getMae(ae123, "123");
			for (Size i = 0; i < 6; ++i)
			{
				std::string akima = name() + "_" + axis_names[i] + "_akima";
				std::string akima_func = "AKISPL(time,0," + akima + ")";
				std::string polynomial_func = static_cast<std::stringstream &>(std::stringstream() << std::setprecision(16) << pe123[i] << " + " << ve123[i] << " * time + " << ae123[i]*0.5 << " * time * time").str();
				std::string func = model().akimaPool().findByName(akima) != model().akimaPool().end() ? akima_func : polynomial_func;

				file << "variable modify  &\r\n"
					<< "	variable_name = ." << model().name() << "." << name() << "." << axis_names[i] << "_type  &\r\n"
					<< "	integer_value = 1 \r\n"
					<< "!\r\n";

				file << "variable modify  &\r\n"
					<< "	variable_name = ." << model().name() << "." << name() << "." << axis_names[i] << "_func  &\r\n"
					<< "	string_value = \"" + func + "\" \r\n"
					<< "!\r\n";

				file << "variable modify  &\r\n"
					<< "	variable_name = ." << model().name() << "." << name() << "." << axis_names[i] << "_ic_disp  &\r\n"
					<< "	real_value = 0.0 \r\n"
					<< "!\r\n";

				file << "variable modify  &\r\n"
					<< "	variable_name = ." << model().name() << "." << name() << "." << axis_names[i] << "_ic_velo  &\r\n"
					<< "	real_value = 0.0 \r\n"
					<< "!\r\n";
			}

			file << "ude modify instance  &\r\n"
				<< "	instance_name = ." << model().name() << "." << name() << "\r\n"
				<< "!\r\n";
		}
		auto GeneralMotion::cptCp(double *cp)const->void 
		{
			double pm_real_j[16];

			s_pm_dot_pm(*makJ().fatherPart().pm(), *makJ().prtPm(), *mpm(), pm_real_j);
			
			double pq_j2i[7];
			double pm_j2i[4][4];
			double diff[6];

			s_inv_pm_dot_pm(*makI().pm(), pm_real_j, *pm_j2i);
			s_pm2pq(*pm_j2i, pq_j2i);

			double theta = atan2(s_norm(3, pq_j2i + 3, 1), pq_j2i[6]) * 2;

			double coe = theta < 1e-3 ? 2.0 : theta / std::sin(theta / 2.0);
			s_nv(3, coe, pq_j2i + 3);

			// 此时位移差值在makI()坐标系中。需要转换到部件坐标系下。
			s_tv(*makI().prtPm(), pq_j2i, diff);

			s_mm(dim(), 1, 6, prtCmPtrI(), ColMajor{ dim() }, diff, 1, cp, 1);
		}
		auto GeneralMotion::cptCv(double *cv)const->void { s_inv_tv(*mpm(), mvs(), cv); }
		auto GeneralMotion::cptCa(double *ca)const->void { s_inv_tv(*mpm(), mas(), ca); }
		auto GeneralMotion::updCa()->void
		{
			//Constraint::updCa();
			//s_va(dim(), ma(), ca_);
		}
		auto GeneralMotion::mpm()const->const double4x4&{ return imp_->mpm_;	}
		auto GeneralMotion::updMpm()->void {	s_inv_pm_dot_pm(*makJ().glbPm(), *makI().glbPm(), *imp_->mpm_);	}
		auto GeneralMotion::setMpe(const double* pe, const char *type)->void { s_pe2pm(pe, *imp_->mpm_, type); }
		auto GeneralMotion::setMpq(const double* pq)->void { s_pq2pm(pq, *imp_->mpm_); }
		auto GeneralMotion::setMpm(const double* pm)->void { s_vc(16, pm, *imp_->mpm_); }
		auto GeneralMotion::getMpe(double* pe, const char *type)const->void { s_pm2pe(*imp_->mpm_, pe, type); }
		auto GeneralMotion::getMpq(double* pq)const->void { s_pm2pq(*imp_->mpm_, pq); }
		auto GeneralMotion::getMpm(double* pm)const->void { s_vc(16, *imp_->mpm_, pm); }
		auto GeneralMotion::mvs()const->const double6&{ return imp_->mvs_; }
		auto GeneralMotion::updMvs()->void {	s_inv_vs2vs(*makJ().glbPm(), makJ().vs(), makI().glbVs(), imp_->mvs_);}
		auto GeneralMotion::setMve(const double* ve, const char *type)->void 
		{
			double pe[6];
			s_pm2pe(*mpm(), pe, type);
			s_ve2vs(pe, ve, imp_->mvs_, type);
		}
		auto GeneralMotion::setMvq(const double* vq)->void
		{
			double pq[7];
			s_pm2pq(*mpm(), pq);
			s_vq2vs(pq, vq, imp_->mvs_);
		}
		auto GeneralMotion::setMvm(const double* vm)->void { s_vm2vs(*mpm(), vm, imp_->mvs_); }
		auto GeneralMotion::setMva(const double* va)->void
		{
			double pp[3];
			s_pm2pp(*mpm(), pp);
			s_va2vs(pp, va, imp_->mvs_);
		}
		auto GeneralMotion::setMvs(const double* vs)->void { s_vc(6, vs, imp_->mvs_); }
		auto GeneralMotion::getMve(double* ve, const char *type)const->void
		{
			double pe[6];
			s_pm2pe(*mpm(), pe, type);
			s_vs2ve(imp_->mvs_, pe, ve, type);
		}
		auto GeneralMotion::getMvq(double* vq)const->void
		{
			double pq[7];
			s_pm2pq(*mpm(), pq);
			s_vs2vq(imp_->mvs_, pq, vq);
		}
		auto GeneralMotion::getMvm(double* vm)const->void { s_vs2vm(imp_->mvs_, *mpm(), vm); }
		auto GeneralMotion::getMva(double* va)const->void
		{
			double pp[3];
			s_pm2pp(*mpm(), pp);
			s_vs2va(imp_->mvs_, pp, va);
		}
		auto GeneralMotion::getMvs(double* vs)const->void { s_vc(6, imp_->mvs_, vs); }
		auto GeneralMotion::mas()const->const double6&{ return imp_->mas_; }
		auto GeneralMotion::updMas()->void { s_inv_as2as(*makJ().glbPm(), makJ().vs(), makJ().as(), makI().glbVs(), makI().glbAs(), imp_->mas_); }
		auto GeneralMotion::setMae(const double* ae, const char *type)->void
		{
			double pe[6], ve[6];
			s_pm2pe(*mpm(), pe, type);
			s_vs2ve(mvs(), pe, ve, type);
			s_ae2as(pe, ve, ae, imp_->mas_, nullptr, type);
		}
		auto GeneralMotion::setMaq(const double* aq)->void
		{
			double pq[7], vq[7];
			s_pm2pq(*mpm(), pq);
			s_vs2vq(mvs(), pq, vq);
			s_aq2as(pq, vq, aq, imp_->mas_);
		}
		auto GeneralMotion::setMam(const double* am)->void
		{
			double vm[16];
			getMvm(vm);
			s_am2as(*mpm(), vm, am, imp_->mas_);
		}
		auto GeneralMotion::setMaa(const double* aa)->void
		{
			double pp[3], va[6];
			s_pm2pp(*mpm(), pp);
			s_vs2va(mvs(), pp, va);
			s_aa2as(pp, va, aa, imp_->mas_);
		}
		auto GeneralMotion::setMas(const double* as)->void { s_vc(6, as, imp_->mas_); }
		auto GeneralMotion::getMae(double* ae, const char *type)const->void
		{
			double pe[6];
			s_pm2pe(*mpm(), pe, type);
			s_as2ae(mvs(), mas(), pe, ae, nullptr, type);
		}
		auto GeneralMotion::getMaq(double* aq)const->void
		{
			double pq[7];
			s_pm2pq(*mpm(), pq);
			s_as2aq(mvs(), mas(), pq, aq);
		}
		auto GeneralMotion::getMam(double* am)const->void { s_as2am(mvs(), mas(), *mpm(), am); }
		auto GeneralMotion::getMaa(double* aa)const->void
		{
			double pp[3];
			s_pm2pp(*mpm(), pp);
			s_as2aa(mvs(), mas(), pp, aa);
		}
		auto GeneralMotion::getMas(double* as)const->void { s_vc(6, imp_->mas_, as); }
		auto GeneralMotion::mfs() const->const double6&{ return this->cf_; }
		auto GeneralMotion::setMfs(const double * mfs)->void { s_vc(6, mfs, cf_); }
		GeneralMotion::~GeneralMotion() = default;
		GeneralMotion::GeneralMotion(const std::string &name, Marker &makI, Marker &makJ, const std::string& freedom, bool active):Constraint(name, makI, makJ, active)
		{
			const static double loc_cst[6][6]
			{
				1,0,0,0,0,0,
				0,1,0,0,0,0,
				0,0,1,0,0,0,
				0,0,0,1,0,0,
				0,0,0,0,1,0,
				0,0,0,0,0,1
			};

			s_tf_n(Dim(), *this->makI().prtPm(), *loc_cst, *prtCmI_);
		}
		GeneralMotion::GeneralMotion(Object &father, const aris::core::XmlElement &xml_ele):Constraint(father, xml_ele)
		{
			const static double loc_cst[6][6]
			{
				1,0,0,0,0,0,
				0,1,0,0,0,0,
				0,0,1,0,0,0,
				0,0,0,1,0,0,
				0,0,0,0,1,0,
				0,0,0,0,0,1
			};

			s_tf_n(Dim(), *this->makI().prtPm(), *loc_cst, *prtCmI_);
		}
		GeneralMotion::GeneralMotion(const GeneralMotion &other) = default;
		GeneralMotion::GeneralMotion(GeneralMotion &&other) = default;
		GeneralMotion& GeneralMotion::operator=(const GeneralMotion &other) = default;
		GeneralMotion& GeneralMotion::operator=(GeneralMotion &&other) = default;

		struct Solver::Imp
		{
			Size max_iter_count_, iter_count_;
			double max_error_, error_;

			Imp(Size max_iter_count, double max_error) :max_iter_count_(max_iter_count), max_error_(max_error) {};
		};
		auto Solver::error()const->double { return imp_->error_; }
		auto Solver::setError(double error)->void { imp_->error_ = error; }
		auto Solver::maxError()const->double { return imp_->max_error_; }
		auto Solver::setMaxError(double max_error)->void { imp_->max_error_ = max_error; }
		auto Solver::iterCount()const->Size { return imp_->iter_count_; }
		auto Solver::setIterCount(Size iter_count)->void { imp_->iter_count_ = iter_count; }
		auto Solver::maxIterCount()const->Size { return imp_->max_iter_count_; }
		auto Solver::setMaxIterCount(Size max_count)->void { imp_->max_iter_count_ = max_count; }
		Solver::~Solver() = default;
		Solver::Solver(Object &father, const aris::core::XmlElement &xml_ele) : Element(father, xml_ele), imp_(new Imp(0, 0.0))
		{
			imp_->max_iter_count_ = attributeInt32(xml_ele, "max_iter_count", 100);
			imp_->max_error_ = attributeDouble(xml_ele, "max_error", 1e-10);
		}
		Solver::Solver(const std::string &name, Size max_iter_count, double max_error): Element(name), imp_(new Imp(max_iter_count, max_error)) {}
		Solver::Solver(const Solver&) = default;
		Solver::Solver(Solver&&) = default;
		Solver& Solver::operator=(const Solver&) = default;
		Solver& Solver::operator=(Solver&&) = default;

		struct Model::Imp
		{
			aris::core::Calculator calculator_;
			Environment *environment_;
			Part* ground_;
			aris::core::ObjectPool<Script, Element> *script_pool_;
			aris::core::ObjectPool<Variable, Element> *variable_pool_;
			aris::core::ObjectPool<Akima, Element> *akima_pool_;
			aris::core::ObjectPool<Part, Element> *part_pool_;
			aris::core::ObjectPool<Joint, Element> *joint_pool_;
			aris::core::ObjectPool<Motion, Element> *motion_pool_;
			aris::core::ObjectPool<GeneralMotion, Element> *general_motion_pool_;
			aris::core::ObjectPool<Force, Element> *force_pool_;
			aris::core::ObjectPool<Solver, Element> *solver_pool_;
			

			std::vector<Size> mot_vec_phy2abs_, mot_vec_sla2abs_;

			// dynamic new //
			//GroundSolver solver_;


			// dynamic //
			//Size dyn_cst_dim_, dyn_prt_dim_, active_cst_num_, active_prt_num_;
			//std::unique_ptr<double[]> dyn_A_, dyn_b_, dyn_x_;
			//Size clb_dim_m_, clb_dim_n_, clb_dim_gam_, clb_dim_frc_;
			//
			//aris::core::RefPool<Part> active_part_pool_;
			//aris::core::RefPool<Constraint> active_constraint_pool_;

			//Size p_size_, c_size_;
			//std::vector<double> cp_, cv_, ca_, cf_;
			//std::vector<double> glb_im_, glb_cm_, glb_pp_, glb_pv_, glb_pa_, glb_pf_;
			//std::vector<double> prt_im_, prt_cm_, prt_pp_, prt_pv_, prt_pa_, prt_pf_;

			//BlockSize p_blk_size_, c_blk_size_;
			//BlockData cp_blk_, cv_blk_, ca_blk_, cf_blk_;
			//BlockData glb_im_blk_, glb_cm_blk_, glb_pp_blk_, glb_pv_blk_, glb_pa_blk_, glb_pf_blk_;
			//BlockData prt_im_blk_, prt_cm_blk_, prt_pp_blk_, prt_pv_blk_, prt_pa_blk_, prt_pf_blk_;
			//
			//BlockSize r_blk_size_;
			//BlockData cpr_blk_, cvr_blk_, car_blk_, cfr_blk_;
			//BlockData glb_cmr_blk_;
			//BlockData cctr_blk_, ctcr_blk_;
			//BlockData cctr_llt_blk_, cctr_x_blk_, cctr_b_blk_, ctcr_llt_blk_, ctcr_x_blk_, ctcr_b_blk_;

			//std::vector<double> cct_, ctc_, cct_llt_, cct_x_, cct_b_, ctc_llt_, ctc_x_, ctc_b_;
			//BlockData cct_blk_, ctc_blk_;
			//BlockData cct_llt_blk_, cct_x_blk_, cct_b_blk_, ctc_llt_blk_, ctc_x_blk_, ctc_b_blk_;

			// clb //
			//Size m_size_clb_, p_size_clb_, frc_size_clb_;
			//std::vector<double> clb_A_, clb_x_, clb_b_;

			//std::function<void(Size dim, const double *D, const double *b, double *x)> dyn_solve_method_{ nullptr };
			//std::function<void(Size n, double *A)> clb_inverse_method_{ nullptr };
		};
		auto Model::SimResult::clear()->void
		{
			time_.clear();
			Pin_.clear();
			Fin_.clear();
			Vin_.clear();
			Ain_.clear();
		}
		auto Model::SimResult::resize(Size size)->void
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
			imp_->solver_pool_ = findOrInsert<aris::core::ObjectPool<Solver, Element>>("solver_pool");
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
				for (auto &gmt : generalMotionPool())
				{
					if (!gmt.active())
					{
						file << "ude attributes  &\r\n"
							<< "    instance_name = ." << name() << "." << gmt.name() << "  &\r\n"
							<< "    active = off \r\n!\r\n"
							<< "!\r\n";
					}
				}
			}
		}
		auto Model::calculator()->aris::core::Calculator& { return imp_->calculator_; }
		auto Model::calculator()const ->const aris::core::Calculator&{ return imp_->calculator_; }
		auto Model::environment()->aris::dynamic::Environment& { return *imp_->environment_; }
		auto Model::environment()const ->const aris::dynamic::Environment&{ return *imp_->environment_; }
		auto Model::scriptPool()->aris::core::ObjectPool<Script, Element>& { return *imp_->script_pool_; }
		auto Model::scriptPool()const->const aris::core::ObjectPool<Script, Element>&{ return *imp_->script_pool_; }
		auto Model::variablePool()->aris::core::ObjectPool<Variable, Element>& { return *imp_->variable_pool_; }
		auto Model::variablePool()const->const aris::core::ObjectPool<Variable, Element>& { return *imp_->variable_pool_; }
		auto Model::akimaPool()->aris::core::ObjectPool<Akima, Element>& { return *imp_->akima_pool_; }
		auto Model::akimaPool()const->const aris::core::ObjectPool<Akima, Element>& { return *imp_->akima_pool_; }
		auto Model::partPool()->aris::core::ObjectPool<Part, Element>& { return *imp_->part_pool_; }
		auto Model::partPool()const->const aris::core::ObjectPool<Part, Element>& { return *imp_->part_pool_; }
		auto Model::jointPool()->aris::core::ObjectPool<Joint, Element>& { return *imp_->joint_pool_; }
		auto Model::jointPool()const->const aris::core::ObjectPool<Joint, Element>& { return *imp_->joint_pool_; }
		auto Model::motionPool()->aris::core::ObjectPool<Motion, Element>& { return *imp_->motion_pool_; }
		auto Model::motionPool()const->const aris::core::ObjectPool<Motion, Element>& { return *imp_->motion_pool_; }
		auto Model::generalMotionPool()->aris::core::ObjectPool<GeneralMotion, Element>& { return *imp_->general_motion_pool_; }
		auto Model::generalMotionPool()const->const aris::core::ObjectPool<GeneralMotion, Element>& { return *imp_->general_motion_pool_; }
		auto Model::forcePool()->aris::core::ObjectPool<Force, Element>& { return *imp_->force_pool_; }
		auto Model::forcePool()const->const aris::core::ObjectPool<Force, Element>& { return *imp_->force_pool_; }
		auto Model::solverPool()->aris::core::ObjectPool<Solver, Element>& { return *imp_->solver_pool_; }
		auto Model::solverPool()const->const aris::core::ObjectPool<Solver, Element>& { return *imp_->solver_pool_; }
		auto Model::updMotionID()->void
		{
			// 更新 mot_vec_sla2abs_
			for (auto &mot : motionPool())
			{
				Size mot_sla_id = (mot.imp_->sla_id_ == -1) ? mot.id() : mot.imp_->sla_id_;
				
				imp_->mot_vec_sla2abs_.resize(std::max(mot_sla_id + 1, imp_->mot_vec_sla2abs_.size()), -1);
				if (imp_->mot_vec_sla2abs_.at(mot_sla_id) != -1) throw std::runtime_error("invalid model xml:\"slave_id\" of motion \"" + name() + "\" already exists");
				imp_->mot_vec_sla2abs_.at(mot_sla_id) = mot.id();
			}
			
			// 更新 mot_vec_phy2abs_
			imp_->mot_vec_phy2abs_.clear();
			for (auto id : imp_->mot_vec_sla2abs_)if (id != std::numeric_limits<Size>::max())imp_->mot_vec_phy2abs_.push_back(id);
			
			// 更新 motion的phy_id
			for (Size phy_id = 0; phy_id < imp_->mot_vec_phy2abs_.size(); ++phy_id)
			{
				Size abs_id = imp_->mot_vec_phy2abs_.at(phy_id);
				motionPool().at(abs_id).imp_->phy_id_ = phy_id;
			}
		}
		auto Model::motionAtAbs(Size abs_id)->Motion& { return motionPool().at(abs_id); }
		auto Model::motionAtAbs(Size abs_id)const->const Motion&{ return motionPool().at(abs_id); }
		auto Model::motionAtPhy(Size phy_id)->Motion& { return motionPool().at(imp_->mot_vec_phy2abs_.at(phy_id));}
		auto Model::motionAtPhy(Size phy_id)const->const Motion&{ return motionPool().at(imp_->mot_vec_phy2abs_.at(phy_id)); }
		auto Model::motionAtSla(Size sla_id)->Motion& { return motionPool().at(imp_->mot_vec_sla2abs_.at(sla_id)); }
		auto Model::motionAtSla(Size sla_id)const->const Motion&{ return motionPool().at(imp_->mot_vec_sla2abs_.at(sla_id)); }
		auto Model::ground()->Part& { return *imp_->ground_; }
		auto Model::ground()const->const Part&{ return *imp_->ground_; }
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

			registerChildType<aris::core::ObjectPool<Geometry, Element>>();
			registerChildType<ParasolidGeometry>();

			registerChildType<aris::core::ObjectPool<Solver, Element>>();
			registerChildType<GroundFullMatrixSolver>();
			registerChildType<PartFullMatrixSolver>();
			registerChildType<DiagSolver>();

			imp_->environment_ = &this->add<Environment>("environment");
			imp_->script_pool_ = &this->add<aris::core::ObjectPool<Script, Element>>("script");
			imp_->variable_pool_ = &this->add<aris::core::ObjectPool<Variable, Element>>("variable_pool");
			imp_->akima_pool_ = &this->add<aris::core::ObjectPool<Akima, Element>>("akima_pool");
			imp_->part_pool_ = &this->add<aris::core::ObjectPool<Part, Element>>("part_pool");
			imp_->joint_pool_ = &this->add<aris::core::ObjectPool<Joint, Element>>("joint_pool");
			imp_->motion_pool_ = &this->add<aris::core::ObjectPool<Motion, Element>>("motion_pool");
			imp_->general_motion_pool_ = &this->add<aris::core::ObjectPool<GeneralMotion, Element>>("general_motion_pool");
			imp_->force_pool_ = &this->add<aris::core::ObjectPool<Force, Element>>("force_pool");
			imp_->solver_pool_ = &this->add<aris::core::ObjectPool<Solver, Element>>("solver_pool");
			imp_->ground_ = &imp_->part_pool_->add<Part>("ground");
		}
		
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
		auto ParasolidGeometry::saveAdams(std::ofstream &file) const->void
		{
			if (!imp_->graphic_file_path.empty())
			{
				double pe[6];
				s_pm2pe(*imp_->prt_pm_, pe, "313");
				core::Matrix ori(1, 3, &pe[3]), loc(1, 3, &pe[0]);

				file << "file parasolid read &\r\n"
					<< "	file_name = \"" << imp_->graphic_file_path << "\" &\r\n"
					<< "	type = ASCII" << " &\r\n"
					<< "	part_name = " << fatherPart().name() << " &\r\n"
					<< "	location = (" << loc.toString() << ") &\r\n"
					<< "	orientation = (" << ori.toString() << ") &\r\n"
					<< "	relative_to = ." << model().name() << "." << fatherPart().name() << " \r\n"
					<< "\r\n";
			}
		}
		ParasolidGeometry::~ParasolidGeometry() = default;
		ParasolidGeometry::ParasolidGeometry(const ParasolidGeometry &other) = default;
		ParasolidGeometry::ParasolidGeometry(ParasolidGeometry &&other) = default;
		ParasolidGeometry& ParasolidGeometry::operator=(const ParasolidGeometry &other) = default;
		ParasolidGeometry& ParasolidGeometry::operator=(ParasolidGeometry &&other) = default;
		ParasolidGeometry::ParasolidGeometry(Object &father, const aris::core::XmlElement &xml_ele) : Geometry(father, xml_ele), imp_(new Imp)
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
		}
		ParasolidGeometry::ParasolidGeometry(const std::string &name, const std::string &graphic_file_path, const double* prt_pm) : Geometry(name), imp_(new Imp)
		{
			static const double default_pm_in[16] = { 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
			prt_pm = prt_pm ? prt_pm : default_pm_in;
			s_vc(16, prt_pm, *imp_->prt_pm_);

			imp_->graphic_file_path = graphic_file_path;

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
			const double *pm_i = *makI().pm();
			const double *pm_j = *makJ().pm();
			const double origin[3]{ 0,0,0 };
			double pm_t[16];

			double v[3], new_z[3];
			s_c3(pm_i + 2, 4, pm_j + 2, 4, v, 1);
			if (s_norm(3, v) == 0)s_vc(3, pm_i, 4, new_z, 1);
			else s_c3(v, 1, pm_i + 2, 4, new_z, 1);
			
			s_sov_pnts2pm(origin, 1, new_z, 1, pm_j, 4, pm_t, "zx");
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


			// 以下计算PrtCm //
			double tem[3];
			const double axis_i_m[3]{ makI().prtPm()[0][2] ,makI().prtPm()[1][2] ,makI().prtPm()[2][2] };
			double axis_j_m[3];
			s_pm_dot_v3(*makJ().fatherPart().pm(), &makJ().prtPm()[0][2], 4, tem, 1);
			s_inv_pm_dot_v3(*makI().fatherPart().pm(), tem, axis_j_m);
			s_c3(axis_i_m, 1, axis_j_m, 1, const_cast<double*>(prtCmPtrI()) + 3 * 4 + 3, 4);
			s_nv(3, 1.0 / s_norm(3, prtCmPtrI() + 3 * 4 + 3, 4), const_cast<double*>(prtCmPtrI()) + 3 * 4 + 3, 4);
			// PrtCm计算完毕 //

			s_mm(dim(), 1, 6, prtCmPtrI(), ColMajor{ dim() }, diff, 1, cp, 1);
		}
		auto UniversalJoint::cptPrtCm(double *prt_cmI, double *prt_cmJ, Size cmI_ld, Size cmJ_ld)const->void
		{
			cmI_ld = std::max(cmI_ld, static_cast<Size>(dim()));
			cmJ_ld = std::max(cmJ_ld, static_cast<Size>(dim()));
			
			double tem[3];
			const double axis_i_m[3]{ makI().prtPm()[0][2] ,makI().prtPm()[1][2] ,makI().prtPm()[2][2] };
			double axis_j_m[3];
			s_pm_dot_v3(*makJ().fatherPart().pm(), &makJ().prtPm()[0][2], 4, tem, 1);
			s_inv_pm_dot_v3(*makI().fatherPart().pm(), tem, axis_j_m);

			s_c3(axis_i_m, 1, axis_j_m, 1, const_cast<double*>(prtCmPtrI()) + 3 * 4 + 3, 4);
			s_nv(3, 1.0 / s_norm(3, prtCmPtrI() + 3 * 4 + 3, 4), const_cast<double*>(prtCmPtrI()) + 3 * 4 + 3, 4);
			
			Constraint::cptPrtCm(prt_cmI, prt_cmJ, cmI_ld, cmJ_ld);
		}
		auto UniversalJoint::cptGlbCm(double *glb_cmI, double *glb_cmJ, Size cmI_ld, Size cmJ_ld)const->void
		{
			cmI_ld = std::max(cmI_ld, static_cast<Size>(dim()));
			cmJ_ld = std::max(cmJ_ld, static_cast<Size>(dim()));
			
			double axis_i_g[3], axis_j_g[3];
			s_pm_dot_v3(*makI().fatherPart().pm(), &makI().prtPm()[0][2], 4, axis_i_g, 1);
			s_pm_dot_v3(*makJ().fatherPart().pm(), &makJ().prtPm()[0][2], 4, axis_j_g, 1);

			s_fill(3, 1, 0.0, glb_cmI + dynamic::id(0, 3, cmI_ld), cmI_ld);
			s_c3(axis_i_g, 1, axis_j_g, 1, glb_cmI + dynamic::id(3, 3, cmI_ld), cmI_ld);
			s_nv(3, 1.0 / s_norm(3, glb_cmI + dynamic::id(3, 3, cmI_ld), cmI_ld), glb_cmI + dynamic::id(3, 3, cmI_ld), cmI_ld);
			
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
			xml_ele.SetAttribute("component", static_cast<int>(this->component_axis_));
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
		SingleComponentForce::SingleComponentForce(const std::string &name, Marker& makI, Marker& makJ, Size componentID) : Force(name, makI, makJ), component_axis_(componentID) {}
		SingleComponentForce::SingleComponentForce(Object &father, const aris::core::XmlElement &xml_ele) : Force(father, xml_ele), component_axis_(attributeInt32(xml_ele, "component")) {}
		
		struct FullMatrixSolver::Imp
		{
			Size p_size_, c_size_;
			std::vector<double> im_, cm_, pp_, pv_, pa_, pf_, cp_, cv_, ca_, cf_;

			std::vector<PartBlock> part_block_pool_;
			std::vector<ConstraintBlock> constraint_block_pool_;

			BlockSize p_blk_size_, c_blk_size_;
			BlockData im_blk_, cm_blk_, pp_blk_, pv_blk_, pa_blk_, pf_blk_, cp_blk_, cv_blk_, ca_blk_, cf_blk_;
		};
		auto FullMatrixSolver::allocateMemory()->void
		{
			// make active pool //
			imp_->part_block_pool_.clear();
			imp_->constraint_block_pool_.clear();

			for (auto &prt : model().partPool())if (prt.active())imp_->part_block_pool_.push_back(PartBlock{ &prt, 0, 0 });
			for (auto &jnt : model().jointPool())if (jnt.active())imp_->constraint_block_pool_.push_back(ConstraintBlock{ &jnt,0,0, nullptr, nullptr });
			for (auto &mot : model().motionPool())if (mot.active()) imp_->constraint_block_pool_.push_back(ConstraintBlock{ &mot,0,0, nullptr, nullptr });
			for (auto &gmt : model().generalMotionPool())if (gmt.active())imp_->constraint_block_pool_.push_back(ConstraintBlock{ &gmt,0,0, nullptr, nullptr });

			// compute memory size old //
			imp_->p_size_ = 0;
			imp_->c_size_ = 6;
			imp_->p_blk_size_.resize(0);
			imp_->c_blk_size_.resize(1, 6);

			for (auto &pb : activePartBlockPool())
			{
				pb.row_id_ = imp_->p_size_;
				pb.blk_row_id_ = imp_->p_blk_size_.size();
				imp_->p_size_ += 6;
				imp_->p_blk_size_.push_back(6);
			}
			for (auto &cb : activeConstraintBlockPool())
			{
				cb.col_id_ = imp_->c_size_;
				cb.blk_col_id_ = imp_->c_blk_size_.size();
				imp_->c_size_ += cb.constraint_->dim();
				imp_->c_blk_size_.push_back(cb.constraint_->dim());

				auto i_ = std::find_if(activePartBlockPool().begin(), activePartBlockPool().end(), [&cb](PartBlock &pb) {return pb.part_ == &cb.constraint_->makI().fatherPart(); });
				auto j_ = std::find_if(activePartBlockPool().begin(), activePartBlockPool().end(), [&cb](PartBlock &pb) {return pb.part_ == &cb.constraint_->makJ().fatherPart(); });
				if (i_ == activePartBlockPool().end()) throw std::runtime_error("i part not found");
				if (j_ == activePartBlockPool().end()) throw std::runtime_error("j part not found");
				cb.pb_i_ = &*i_;
				cb.pb_j_ = &*j_;
			}

			// allocate memory //
			imp_->im_.resize(imp_->p_size_ * imp_->p_size_, 0.0);
			imp_->cm_.resize(imp_->p_size_ * imp_->c_size_, 0.0);
			imp_->cp_.resize(imp_->c_size_ * 1, 0.0);
			imp_->cv_.resize(imp_->c_size_ * 1, 0.0);
			imp_->ca_.resize(imp_->c_size_ * 1, 0.0);
			imp_->cf_.resize(imp_->c_size_ * 1, 0.0);
			imp_->pp_.resize(imp_->p_size_ * 1, 0.0);
			imp_->pv_.resize(imp_->p_size_ * 1, 0.0);
			imp_->pa_.resize(imp_->p_size_ * 1, 0.0);
			imp_->pf_.resize(imp_->p_size_ * 1, 0.0);

			imp_->im_blk_.resize(imp_->p_blk_size_.size() * imp_->p_blk_size_.size());
			imp_->cm_blk_.resize(imp_->p_blk_size_.size() * imp_->c_blk_size_.size());
			imp_->cp_blk_.resize(imp_->c_blk_size_.size() * 1);
			imp_->cv_blk_.resize(imp_->c_blk_size_.size() * 1);
			imp_->ca_blk_.resize(imp_->c_blk_size_.size() * 1);
			imp_->cf_blk_.resize(imp_->c_blk_size_.size() * 1);
			imp_->pp_blk_.resize(imp_->p_blk_size_.size() * 1);
			imp_->pv_blk_.resize(imp_->p_blk_size_.size() * 1);
			imp_->pa_blk_.resize(imp_->p_blk_size_.size() * 1);
			imp_->pf_blk_.resize(imp_->p_blk_size_.size() * 1);
			
			s_blk_map(cBlkSize(), { 1 }, imp_->cp_.data(), imp_->cp_blk_);
			s_blk_map(cBlkSize(), { 1 }, imp_->cv_.data(), imp_->cv_blk_);
			s_blk_map(cBlkSize(), { 1 }, imp_->ca_.data(), imp_->ca_blk_);
			s_blk_map(cBlkSize(), { 1 }, imp_->cf_.data(), imp_->cf_blk_);
			s_blk_map(pBlkSize(), { 1 }, imp_->pp_.data(), imp_->pp_blk_);
			s_blk_map(pBlkSize(), { 1 }, imp_->pv_.data(), imp_->pv_blk_);
			s_blk_map(pBlkSize(), { 1 }, imp_->pa_.data(), imp_->pa_blk_);
			s_blk_map(pBlkSize(), { 1 }, imp_->pf_.data(), imp_->pf_blk_);
			s_blk_map(pBlkSize(), pBlkSize(), imp_->im_.data(), imp_->im_blk_);
			s_blk_map(pBlkSize(), cBlkSize(), imp_->cm_.data(), imp_->cm_blk_);

			for (auto &ele : imp_->im_blk_)ele.is_zero = true;
			for (auto &ele : imp_->cm_blk_)ele.is_zero = true;
			for (auto &pb : activePartBlockPool())imp_->im_blk_[dynamic::id(pb.blk_row_id_, pb.blk_row_id_, pBlkSize().size())].is_zero = false;
			for (auto &cb : activeConstraintBlockPool())
			{
				imp_->cm_blk_[dynamic::id(cb.pb_i_->blk_row_id_, cb.blk_col_id_, cBlkSize().size())].is_zero = false;
				imp_->cm_blk_[dynamic::id(cb.pb_j_->blk_row_id_, cb.blk_col_id_, cBlkSize().size())].is_zero = false;
			}

			auto ground_iter = std::find_if(imp_->part_block_pool_.begin(), imp_->part_block_pool_.end(), [&](PartBlock &pb) {return pb.part_ == &model().ground(); });

			imp_->cm_blk_[dynamic::id(ground_iter->blk_row_id_, 0, cBlkSize().size())].is_zero = false;
			for (Size i = 0; i < 6; ++i)imp_->cm_[aris::dynamic::id(ground_iter->row_id_ + i, i, cSize())] = 1.0;

		}
		auto FullMatrixSolver::activePartBlockPool()->std::vector<PartBlock>& { return imp_->part_block_pool_; }
		auto FullMatrixSolver::activeConstraintBlockPool()->std::vector<ConstraintBlock>& { return imp_->constraint_block_pool_; }
		auto FullMatrixSolver::cSize()->Size { return imp_->c_size_; }
		auto FullMatrixSolver::pSize()->Size { return imp_->p_size_; }
		auto FullMatrixSolver::im()->double * { return imp_->im_.data(); }
		auto FullMatrixSolver::cm()->double * { return imp_->cm_.data(); }
		auto FullMatrixSolver::pp()->double * { return imp_->pp_.data(); }
		auto FullMatrixSolver::pv()->double * { return imp_->pv_.data(); }
		auto FullMatrixSolver::pa()->double * { return imp_->pa_.data(); }
		auto FullMatrixSolver::pf()->double * { return imp_->pf_.data(); }
		auto FullMatrixSolver::cp()->double * { return imp_->cp_.data(); }
		auto FullMatrixSolver::cv()->double * { return imp_->cv_.data(); }
		auto FullMatrixSolver::ca()->double * { return imp_->ca_.data(); }
		auto FullMatrixSolver::cf()->double * { return imp_->cf_.data(); }
		auto FullMatrixSolver::cBlkSize()->BlockSize& { return imp_->c_blk_size_; }
		auto FullMatrixSolver::pBlkSize()->BlockSize& { return imp_->p_blk_size_; }
		auto FullMatrixSolver::imBlk()->BlockData& { return imp_->im_blk_; }
		auto FullMatrixSolver::cmBlk()->BlockData& { return imp_->cm_blk_; }
		auto FullMatrixSolver::ppBlk()->BlockData& { return imp_->pp_blk_; }
		auto FullMatrixSolver::pvBlk()->BlockData& { return imp_->pv_blk_; }
		auto FullMatrixSolver::paBlk()->BlockData& { return imp_->pa_blk_; }
		auto FullMatrixSolver::pfBlk()->BlockData& { return imp_->pf_blk_; }
		auto FullMatrixSolver::cpBlk()->BlockData& { return imp_->cp_blk_; }
		auto FullMatrixSolver::cvBlk()->BlockData& { return imp_->cv_blk_; }
		auto FullMatrixSolver::caBlk()->BlockData& { return imp_->ca_blk_; }
		auto FullMatrixSolver::cfBlk()->BlockData& { return imp_->cf_blk_; }
		FullMatrixSolver::~FullMatrixSolver() = default;
		FullMatrixSolver::FullMatrixSolver(const std::string &name, Size max_iter_count, double max_error) :Solver(name, max_iter_count, max_error) {}
		FullMatrixSolver::FullMatrixSolver(Object &father, const aris::core::XmlElement &xml_ele) :Solver(father, xml_ele){}
		FullMatrixSolver::FullMatrixSolver(const FullMatrixSolver &other) = default;
		FullMatrixSolver::FullMatrixSolver(FullMatrixSolver &&other) = default;
		FullMatrixSolver& FullMatrixSolver::operator=(const FullMatrixSolver &other) = default;
		FullMatrixSolver& FullMatrixSolver::operator=(FullMatrixSolver &&other) = default;

		struct GroundFullMatrixSolver::Imp
		{
			std::vector<double> cct_, ctc_, cct_llt_, cct_x_, cct_b_, ctc_llt_, ctc_x_, ctc_b_;
			BlockData cct_blk_, ctc_blk_;
			BlockData cct_llt_blk_, cct_x_blk_, cct_b_blk_, ctc_llt_blk_, ctc_x_blk_, ctc_b_blk_;
		};
		auto GroundFullMatrixSolver::allocateMemory()->void
		{
			FullMatrixSolver::allocateMemory();

			imp_->cct_.resize(pSize() * pSize());
			imp_->cct_llt_.resize(pSize() * pSize());
			imp_->cct_b_.resize(pSize() * 1);
			imp_->cct_x_.resize(pSize() * 1);

			imp_->cct_blk_.resize(pBlkSize().size() * pBlkSize().size());
			imp_->cct_llt_blk_.resize(pBlkSize().size() * pBlkSize().size());
			imp_->cct_b_blk_.resize(pBlkSize().size() * 1);
			imp_->cct_x_blk_.resize(pBlkSize().size() * 1);

			s_blk_map(pBlkSize(), pBlkSize(), imp_->cct_.data(), imp_->cct_blk_);
			s_blk_map(pBlkSize(), pBlkSize(), imp_->cct_llt_.data(), imp_->cct_llt_blk_);
			s_blk_map(pBlkSize(), { 1 }, imp_->cct_b_.data(), imp_->cct_b_blk_);
			s_blk_map(pBlkSize(), { 1 }, imp_->cct_x_.data(), imp_->cct_x_blk_);
		}
		auto GroundFullMatrixSolver::updCp()->void { for (auto &cb : activeConstraintBlockPool())cb.constraint_->cptCp(cp() + dynamic::id(cb.col_id_, 0, 1)); }
		auto GroundFullMatrixSolver::updCv()->void { for (auto &cb : activeConstraintBlockPool())cb.constraint_->cptCv(cv() + dynamic::id(cb.col_id_, 0, 1)); }
		auto GroundFullMatrixSolver::updCa()->void { for (auto &cb : activeConstraintBlockPool())cb.constraint_->cptCa(ca() + dynamic::id(cb.col_id_, 0, 1)); }
		auto GroundFullMatrixSolver::updPv()->void { for (auto &pb : activePartBlockPool())pb.part_->getVs(pv() + dynamic::id(pb.row_id_, 0, 1)); }
		auto GroundFullMatrixSolver::updPa()->void { for (auto &pb : activePartBlockPool())pb.part_->getAs(pa() + dynamic::id(pb.row_id_, 0, 1)); }
		auto GroundFullMatrixSolver::updPf()->void { for (auto &pb : activePartBlockPool())pb.part_->cptGlbPf(pf() + dynamic::id(pb.row_id_, 0, 1)); }
		auto GroundFullMatrixSolver::updIm()->void { for (auto &pb : activePartBlockPool())pb.part_->cptGlbIm(im() + dynamic::id(pb.row_id_, pb.row_id_, pSize()), pSize()); }
		auto GroundFullMatrixSolver::updCm()->void { for (auto &cb : activeConstraintBlockPool())cb.constraint_->cptGlbCm(cm() + dynamic::id(cb.pb_i_->row_id_, cb.col_id_, cSize()), cm() + dynamic::id(cb.pb_j_->row_id_, cb.col_id_, cSize()), cSize(), cSize()); }
		auto GroundFullMatrixSolver::updConstraintFce()->void { for (auto &cb : activeConstraintBlockPool())cb.constraint_->setCf(cf() + dynamic::id(cb.col_id_, 0, 1)); }
		auto GroundFullMatrixSolver::updPartPos()->void
		{
			for (auto pb : activePartBlockPool())
			{
				double pm[4][4];
				double pq[7];

				s_vc(6, pp() + dynamic::id(pb.row_id_, 0, 1), pq);

				double theta = s_norm(3, pq + 3);
				pq[6] = std::cos(theta / 2);

				double factor = theta < 1e-4 ? 0.5 : std::sin(theta / 2) / theta;
				s_nv(3, factor, pq + 3);

				s_pq2pm(pq, *pm);

				double final_pm[4][4];
				s_pm2pm(*pm, *pb.part_->pm(), *final_pm);

				pb.part_->setPm(*final_pm);
			}
		}
		auto GroundFullMatrixSolver::updPartVel()->void { for (auto &pb : activePartBlockPool())pb.part_->setVs(pv() + dynamic::id(pb.row_id_, 0, 1)); }
		auto GroundFullMatrixSolver::updPartAcc()->void { for (auto &pb : activePartBlockPool())pb.part_->setAs(pa() + dynamic::id(pb.row_id_, 0, 1)); }
		auto GroundFullMatrixSolver::kinPos()->void 
		{
			setIterCount(0);

			for (; iterCount() < maxIterCount(); setIterCount(iterCount() + 1))
			{
				updCm();
				updCp();
				s_blk_mm(pBlkSize(), pBlkSize(), cBlkSize(), cmBlk(), BlockStride{ cBlkSize().size(),1,cSize(),1 }, cmBlk(), T(BlockStride{ cBlkSize().size(),1,cSize(),1 }), imp_->cct_blk_, BlockStride{ pBlkSize().size(),1,pSize(),1 });
				s_blk_mm(pBlkSize(), { 1 }, cBlkSize(), cmBlk(), cpBlk(), imp_->cct_b_blk_);

				setError(s_blk_norm_fro(cBlkSize(), { 1 }, cpBlk(), BlockStride{ 1,1,1,1 }));

				if (error() < maxError()) return;

				s_blk_llt(pBlkSize(), imp_->cct_blk_, BlockStride{ pBlkSize().size(),1,pSize(),1 }, imp_->cct_llt_blk_, BlockStride{ pBlkSize().size(),1,pSize(),1 });
				s_blk_sov_lm(pBlkSize(), { 1 }, imp_->cct_llt_blk_, BlockStride{ pBlkSize().size(),1,pSize(),1 }, imp_->cct_b_blk_, BlockStride{ 1,1,1,1 }, imp_->cct_x_blk_, BlockStride{ 1,1,1,1 });
				s_blk_sov_um(pBlkSize(), { 1 }, imp_->cct_llt_blk_, BlockStride{ pBlkSize().size(),1,pSize(),1 }, imp_->cct_x_blk_, BlockStride{ 1,1,1,1 }, ppBlk(), BlockStride{ 1,1,1,1 });

				updPartPos();
			}
		}
		auto GroundFullMatrixSolver::kinVel()->void 
		{
			updCm();
			updCv();

			s_blk_mm(pBlkSize(), pBlkSize(), cBlkSize(), cmBlk(), BlockStride{ cBlkSize().size(),1,cSize(),1 }, cmBlk(), T(BlockStride{ cBlkSize().size(),1,cSize(),1 }), imp_->cct_blk_, BlockStride{ pBlkSize().size(),1,pSize(),1 });
			s_blk_mm(pBlkSize(), { 1 }, cBlkSize(), cmBlk(), cvBlk(), imp_->cct_b_blk_);

			s_blk_llt(pBlkSize(), imp_->cct_blk_, BlockStride{ pBlkSize().size(),1,pSize(),1 }, imp_->cct_llt_blk_, BlockStride{ pBlkSize().size(),1,pSize(),1 });
			s_blk_sov_lm(pBlkSize(), { 1 }, imp_->cct_llt_blk_, BlockStride{ pBlkSize().size(),1,pSize(),1 }, imp_->cct_b_blk_, BlockStride{ 1,1,1,1 }, imp_->cct_x_blk_, BlockStride{ 1,1,1,1 });
			s_blk_sov_um(pBlkSize(), { 1 }, imp_->cct_llt_blk_, BlockStride{ pBlkSize().size(),1,pSize(),1 }, imp_->cct_x_blk_, BlockStride{ 1,1,1,1 }, pvBlk(), BlockStride{ 1,1,1,1 });

			updPartVel();
		}
		auto GroundFullMatrixSolver::kinAcc()->void
		{
			updCm();
			updCa();
			s_blk_mm(pBlkSize(), pBlkSize(), cBlkSize(), cmBlk(), BlockStride{ cBlkSize().size(),1,cSize(),1 }, cmBlk(), T(BlockStride{ cBlkSize().size(),1,cSize(),1 }), imp_->cct_blk_, BlockStride{ pBlkSize().size(),1,pSize(),1 });
			s_blk_mm(pBlkSize(), { 1 }, cBlkSize(), cmBlk(), caBlk(), imp_->cct_b_blk_);

			s_blk_llt(pBlkSize(), imp_->cct_blk_, BlockStride{ pBlkSize().size(),1,pSize(),1 }, imp_->cct_llt_blk_, BlockStride{ pBlkSize().size(),1,pSize(),1 });
			s_blk_sov_lm(pBlkSize(), { 1 }, imp_->cct_llt_blk_, BlockStride{ pBlkSize().size(),1,pSize(),1 }, imp_->cct_b_blk_, BlockStride{ 1,1,1,1 }, imp_->cct_x_blk_, BlockStride{ 1,1,1,1 });
			s_blk_sov_um(pBlkSize(), { 1 }, imp_->cct_llt_blk_, BlockStride{ pBlkSize().size(),1,pSize(),1 }, imp_->cct_x_blk_, BlockStride{ 1,1,1,1 }, paBlk(), BlockStride{ 1,1,1,1 });

			updPartAcc();
		}
		auto GroundFullMatrixSolver::dynFce()->void 
		{
			updIm();
			updCm();
			updPf();
			updPa();

			s_blk_mm(pBlkSize(), pBlkSize(), cBlkSize(), cmBlk(), BlockStride{ cBlkSize().size(),1,cSize(),1 }, cmBlk(), T(BlockStride{ cBlkSize().size(),1,cSize(),1 }), imp_->cct_blk_, BlockStride{ pBlkSize().size(),1,pSize(),1 });
			s_vc(pSize(), pf(), imp_->cct_b_.data());
			s_blk_mma(pBlkSize(), { 1 }, pBlkSize(), -1.0, imBlk(), BlockStride{ pBlkSize().size(),1,pSize(),1 }, paBlk(), BlockStride{ 1,1,1,1 }, imp_->cct_b_blk_, BlockStride{ 1,1,1,1 });

			s_blk_llt(pBlkSize(), imp_->cct_blk_, BlockStride{ pBlkSize().size(),1,pSize(),1 }, imp_->cct_llt_blk_, BlockStride{ pBlkSize().size(),1,pSize(),1 });
			s_blk_sov_lm(pBlkSize(), { 1 }, imp_->cct_llt_blk_, BlockStride{ pBlkSize().size(),1,pSize(),1 }, imp_->cct_b_blk_, BlockStride{ 1,1,1,1 }, imp_->cct_x_blk_, BlockStride{ 1,1,1,1 });
			s_blk_sov_um(pBlkSize(), { 1 }, imp_->cct_llt_blk_, BlockStride{ pBlkSize().size(),1,pSize(),1 }, imp_->cct_x_blk_, BlockStride{ 1,1,1,1 }, imp_->cct_b_blk_, BlockStride{ 1,1,1,1 });
			s_vc(pSize(), imp_->cct_b_.data(), imp_->cct_x_.data());

			s_blk_mm(cBlkSize(), { 1 }, pBlkSize(), cmBlk(), T(BlockStride{ cBlkSize().size(),1,cSize(),1 }), imp_->cct_x_blk_, BlockStride{ 1,1,1,1 }, cfBlk(), BlockStride{ 1,1,1,1 });

			updConstraintFce();
		}
		GroundFullMatrixSolver::~GroundFullMatrixSolver() = default;
		GroundFullMatrixSolver::GroundFullMatrixSolver(const std::string &name) :FullMatrixSolver(name) {}
		GroundFullMatrixSolver::GroundFullMatrixSolver(Object &father, const aris::core::XmlElement &xml_ele) : FullMatrixSolver(father, xml_ele) {}
		GroundFullMatrixSolver::GroundFullMatrixSolver(const GroundFullMatrixSolver &other) = default;
		GroundFullMatrixSolver::GroundFullMatrixSolver(GroundFullMatrixSolver &&other) = default;
		GroundFullMatrixSolver& GroundFullMatrixSolver::operator=(const GroundFullMatrixSolver &other) = default;
		GroundFullMatrixSolver& GroundFullMatrixSolver::operator=(GroundFullMatrixSolver &&other) = default;

		struct PartFullMatrixSolver::Imp
		{
			std::vector<double> cct_, ctc_, cct_llt_, cct_x_, cct_b_, ctc_llt_, ctc_x_, ctc_b_;
			BlockData cct_blk_, ctc_blk_;
			BlockData cct_llt_blk_, cct_x_blk_, cct_b_blk_, ctc_llt_blk_, ctc_x_blk_, ctc_b_blk_;
		};
		auto PartFullMatrixSolver::allocateMemory()->void
		{
			FullMatrixSolver::allocateMemory();

			imp_->cct_.resize(pSize() * pSize());
			imp_->cct_llt_.resize(pSize() * pSize());
			imp_->cct_b_.resize(pSize() * 1);
			imp_->cct_x_.resize(pSize() * 1);

			imp_->cct_blk_.resize(pBlkSize().size() * pBlkSize().size());
			imp_->cct_llt_blk_.resize(pBlkSize().size() * pBlkSize().size());
			imp_->cct_b_blk_.resize(pBlkSize().size() * 1);
			imp_->cct_x_blk_.resize(pBlkSize().size() * 1);

			s_blk_map(pBlkSize(), pBlkSize(), imp_->cct_.data(), imp_->cct_blk_);
			s_blk_map(pBlkSize(), pBlkSize(), imp_->cct_llt_.data(), imp_->cct_llt_blk_);
			s_blk_map(pBlkSize(), { 1 }, imp_->cct_b_.data(), imp_->cct_b_blk_);
			s_blk_map(pBlkSize(), { 1 }, imp_->cct_x_.data(), imp_->cct_x_blk_);
		}
		auto PartFullMatrixSolver::updCp()->void { for (auto &cb : activeConstraintBlockPool())cb.constraint_->cptCp(cp() + dynamic::id(cb.col_id_, 0, 1)); }
		auto PartFullMatrixSolver::updCv()->void { for (auto &cb : activeConstraintBlockPool())cb.constraint_->cptCv(cv() + dynamic::id(cb.col_id_, 0, 1)); }
		auto PartFullMatrixSolver::updCa()->void { for (auto &cb : activeConstraintBlockPool())cb.constraint_->cptCa(ca() + dynamic::id(cb.col_id_, 0, 1)); }
		auto PartFullMatrixSolver::updPv()->void { for (auto &pb : activePartBlockPool())pb.part_->cptPrtVs(pv() + dynamic::id(pb.row_id_, 0, 1)); }
		auto PartFullMatrixSolver::updPa()->void { for (auto &pb : activePartBlockPool())pb.part_->cptPrtAs(pa() + dynamic::id(pb.row_id_, 0, 1)); }
		auto PartFullMatrixSolver::updPf()->void { for (auto &pb : activePartBlockPool())pb.part_->cptPrtPf(pf() + dynamic::id(pb.row_id_, 0, 1)); }
		auto PartFullMatrixSolver::updIm()->void { for (auto &pb : activePartBlockPool())s_mc(6, 6, *pb.part_->prtIm(), 6, im() + dynamic::id(pb.row_id_, pb.row_id_, pSize()), pSize()); }
		auto PartFullMatrixSolver::updCm()->void { for (auto &cb : activeConstraintBlockPool())cb.constraint_->cptPrtCm(cm() + dynamic::id(cb.pb_i_->row_id_, cb.col_id_, cSize()), cm() + dynamic::id(cb.pb_j_->row_id_, cb.col_id_, cSize()), cSize(), cSize()); }
		auto PartFullMatrixSolver::updConstraintFce()->void { for (auto &cb : activeConstraintBlockPool())cb.constraint_->setCf(cf() + dynamic::id(cb.col_id_, 0, 1)); }
		auto PartFullMatrixSolver::updPartPos()->void
		{
			for (auto pb : activePartBlockPool())
			{
				double pm[4][4];
				double pq[7];

				//s_vc(6, pp() + dynamic::id(pb.row_id_, 0, 1), pq);
				s_tv(*pb.part_->pm(), pp() + dynamic::id(pb.row_id_, 0, 1), pq);

				double theta = s_norm(3, pq + 3);
				pq[6] = std::cos(theta / 2);

				double factor = theta < 1e-4 ? 0.5 : std::sin(theta / 2) / theta;
				s_nv(3, factor, pq + 3);

				s_pq2pm(pq, *pm);

				double final_pm[4][4];
				s_pm2pm(*pm, *pb.part_->pm(), *final_pm);

				pb.part_->setPm(*final_pm);
			}
		}
		auto PartFullMatrixSolver::updPartVel()->void { for (auto &pb : activePartBlockPool())s_tv(*pb.part_->pm(), pv() + dynamic::id(pb.row_id_, 0, 1), const_cast<double6&>(pb.part_->vs()));}
		auto PartFullMatrixSolver::updPartAcc()->void { for (auto &pb : activePartBlockPool())s_tv(*pb.part_->pm(), pa() + dynamic::id(pb.row_id_, 0, 1), const_cast<double6&>(pb.part_->as()));}
		auto PartFullMatrixSolver::kinPos()->void
		{
			setIterCount(0);

			for (; iterCount() < maxIterCount(); setIterCount(iterCount() + 1))
			{
				updCm();
				updCp();
				s_blk_mm(pBlkSize(), pBlkSize(), cBlkSize(), cmBlk(), BlockStride{ cBlkSize().size(),1,cSize(),1 }, cmBlk(), T(BlockStride{ cBlkSize().size(),1,cSize(),1 }), imp_->cct_blk_, BlockStride{ pBlkSize().size(),1,pSize(),1 });
				s_blk_mm(pBlkSize(), { 1 }, cBlkSize(), cmBlk(), cpBlk(), imp_->cct_b_blk_);

				setError(s_blk_norm_fro(cBlkSize(), { 1 }, cpBlk(), BlockStride{ 1,1,1,1 }));

				if (error() < maxError()) return;

				s_blk_llt(pBlkSize(), imp_->cct_blk_, BlockStride{ pBlkSize().size(),1,pSize(),1 }, imp_->cct_llt_blk_, BlockStride{ pBlkSize().size(),1,pSize(),1 });
				s_blk_sov_lm(pBlkSize(), { 1 }, imp_->cct_llt_blk_, BlockStride{ pBlkSize().size(),1,pSize(),1 }, imp_->cct_b_blk_, BlockStride{ 1,1,1,1 }, imp_->cct_x_blk_, BlockStride{ 1,1,1,1 });
				s_blk_sov_um(pBlkSize(), { 1 }, imp_->cct_llt_blk_, BlockStride{ pBlkSize().size(),1,pSize(),1 }, imp_->cct_x_blk_, BlockStride{ 1,1,1,1 }, ppBlk(), BlockStride{ 1,1,1,1 });

				updPartPos();
			}
		}
		auto PartFullMatrixSolver::kinVel()->void
		{
			updCm();
			updCv();

			s_blk_mm(pBlkSize(), pBlkSize(), cBlkSize(), cmBlk(), BlockStride{ cBlkSize().size(),1,cSize(),1 }, cmBlk(), T(BlockStride{ cBlkSize().size(),1,cSize(),1 }), imp_->cct_blk_, BlockStride{ pBlkSize().size(),1,pSize(),1 });
			s_blk_mm(pBlkSize(), { 1 }, cBlkSize(), cmBlk(), cvBlk(), imp_->cct_b_blk_);

			s_blk_llt(pBlkSize(), imp_->cct_blk_, BlockStride{ pBlkSize().size(),1,pSize(),1 }, imp_->cct_llt_blk_, BlockStride{ pBlkSize().size(),1,pSize(),1 });
			s_blk_sov_lm(pBlkSize(), { 1 }, imp_->cct_llt_blk_, BlockStride{ pBlkSize().size(),1,pSize(),1 }, imp_->cct_b_blk_, BlockStride{ 1,1,1,1 }, imp_->cct_x_blk_, BlockStride{ 1,1,1,1 });
			s_blk_sov_um(pBlkSize(), { 1 }, imp_->cct_llt_blk_, BlockStride{ pBlkSize().size(),1,pSize(),1 }, imp_->cct_x_blk_, BlockStride{ 1,1,1,1 }, pvBlk(), BlockStride{ 1,1,1,1 });

			updPartVel();
		}
		auto PartFullMatrixSolver::kinAcc()->void
		{
			updCm();
			updCa();
			s_blk_mm(pBlkSize(), pBlkSize(), cBlkSize(), cmBlk(), BlockStride{ cBlkSize().size(),1,cSize(),1 }, cmBlk(), T(BlockStride{ cBlkSize().size(),1,cSize(),1 }), imp_->cct_blk_, BlockStride{ pBlkSize().size(),1,pSize(),1 });
			s_blk_mm(pBlkSize(), { 1 }, cBlkSize(), cmBlk(), caBlk(), imp_->cct_b_blk_);

			s_blk_llt(pBlkSize(), imp_->cct_blk_, BlockStride{ pBlkSize().size(),1,pSize(),1 }, imp_->cct_llt_blk_, BlockStride{ pBlkSize().size(),1,pSize(),1 });
			s_blk_sov_lm(pBlkSize(), { 1 }, imp_->cct_llt_blk_, BlockStride{ pBlkSize().size(),1,pSize(),1 }, imp_->cct_b_blk_, BlockStride{ 1,1,1,1 }, imp_->cct_x_blk_, BlockStride{ 1,1,1,1 });
			s_blk_sov_um(pBlkSize(), { 1 }, imp_->cct_llt_blk_, BlockStride{ pBlkSize().size(),1,pSize(),1 }, imp_->cct_x_blk_, BlockStride{ 1,1,1,1 }, paBlk(), BlockStride{ 1,1,1,1 });

			updPartAcc();
		}
		auto PartFullMatrixSolver::dynFce()->void
		{
			updIm();
			updCm();
			updPf();
			updPa();

			s_blk_mm(pBlkSize(), pBlkSize(), cBlkSize(), cmBlk(), BlockStride{ cBlkSize().size(),1,cSize(),1 }, cmBlk(), T(BlockStride{ cBlkSize().size(),1,cSize(),1 }), imp_->cct_blk_, BlockStride{ pBlkSize().size(),1,pSize(),1 });
			s_vc(pSize(), pf(), imp_->cct_b_.data());
			s_blk_mma(pBlkSize(), { 1 }, pBlkSize(), -1.0, imBlk(), BlockStride{ pBlkSize().size(),1,pSize(),1 }, paBlk(), BlockStride{ 1,1,1,1 }, imp_->cct_b_blk_, BlockStride{ 1,1,1,1 });

			s_blk_llt(pBlkSize(), imp_->cct_blk_, BlockStride{ pBlkSize().size(),1,pSize(),1 }, imp_->cct_llt_blk_, BlockStride{ pBlkSize().size(),1,pSize(),1 });
			s_blk_sov_lm(pBlkSize(), { 1 }, imp_->cct_llt_blk_, BlockStride{ pBlkSize().size(),1,pSize(),1 }, imp_->cct_b_blk_, BlockStride{ 1,1,1,1 }, imp_->cct_x_blk_, BlockStride{ 1,1,1,1 });
			s_blk_sov_um(pBlkSize(), { 1 }, imp_->cct_llt_blk_, BlockStride{ pBlkSize().size(),1,pSize(),1 }, imp_->cct_x_blk_, BlockStride{ 1,1,1,1 }, imp_->cct_b_blk_, BlockStride{ 1,1,1,1 });
			s_vc(pSize(), imp_->cct_b_.data(), imp_->cct_x_.data());

			s_blk_mm(cBlkSize(), { 1 }, pBlkSize(), cmBlk(), T(BlockStride{ cBlkSize().size(),1,cSize(),1 }), imp_->cct_x_blk_, BlockStride{ 1,1,1,1 }, cfBlk(), BlockStride{ 1,1,1,1 });

			updConstraintFce();
		}
		PartFullMatrixSolver::~PartFullMatrixSolver() = default;
		PartFullMatrixSolver::PartFullMatrixSolver(const std::string &name) :FullMatrixSolver(name) {}
		PartFullMatrixSolver::PartFullMatrixSolver(Object &father, const aris::core::XmlElement &xml_ele) : FullMatrixSolver(father, xml_ele) {}
		PartFullMatrixSolver::PartFullMatrixSolver(const PartFullMatrixSolver &other) = default;
		PartFullMatrixSolver::PartFullMatrixSolver(PartFullMatrixSolver &&other) = default;
		PartFullMatrixSolver& PartFullMatrixSolver::operator=(const PartFullMatrixSolver &other) = default;
		PartFullMatrixSolver& PartFullMatrixSolver::operator=(PartFullMatrixSolver &&other) = default;

		struct DiagSolver::Imp
		{
			std::vector<Relation> relation_pool_;
			std::vector<Part *> part_pool_;
			std::vector<Diag> diag_pool_;
			std::vector<Remainder> remainder_pool_;

			std::vector<double> A_;
			std::vector<double> x_;
			std::vector<double> b_;
			std::vector<double> U_, tau_;

			Size rows, cols;
		};
		auto DiagSolver::allocateMemory()->void 
		{
			// make active part pool //
			activePartPool().clear();
			for (auto &p : model().partPool())if (p.active())activePartPool().push_back(&p);
			
			// make active constraint pool //
			std::vector<Constraint*> cp;
			for (auto &jnt : model().jointPool())if (jnt.active())cp.push_back(&jnt);
			for (auto &mot : model().motionPool())if (mot.active()) cp.push_back(&mot);
			for (auto &gmt : model().generalMotionPool())if (gmt.active())cp.push_back( &gmt);
			
			// make relation pool //
			relationPool().clear();
			for (auto c : cp)
			{
				auto ret = std::find_if(relationPool().begin(), relationPool().end(), [&c](Relation &relation)
				{
					const auto ri{ relation.prtI }, rj{ relation.prtJ }, ci{ &c->makI().fatherPart() }, cj{ &c->makJ().fatherPart() };
					return ((ri == ci) && (rj == cj)) || ((ri == cj) && (rj == ci));
				});

				if (ret == relationPool().end()) relationPool().push_back(Relation{ &c->makI().fatherPart(), &c->makJ().fatherPart(), c->dim(),{ { c, true } } });
				else
				{
					ret->dim += c->dim();
					ret->cst_pool_.push_back({ c, &c->makI().fatherPart() == ret->prtI });
				}
			}
			
			// adjust order //
			for (Size i = 0; i < std::min(activePartPool().size(), relationPool().size()); ++i)
			{
				// 先对part排序，找出下一个跟上一个part联系的part
				std::sort(activePartPool().begin() + i, activePartPool().end(), [i, this](Part* a, Part* b)
				{
					if (a == &model().ground()) return true;
					if (b == &model().ground()) return false;
					if (i == 0)return a->id() < b->id();
					if (b == relationPool()[i - 1].prtI) return false;
					if (b == relationPool()[i - 1].prtJ) return false;
					if (a == relationPool()[i - 1].prtI) return true;
					if (a == relationPool()[i - 1].prtJ) return true;
					return a->id() < b->id();
				});
				// 再插入连接新part的relation
				std::sort(relationPool().begin() + i, relationPool().end(), [i, this](Relation a, Relation b)
				{
					auto pend = activePartPool().begin() + i + 1;
					auto a_part_i = std::find_if(activePartPool().begin(), pend, [a](Part* p)->bool { return p == a.prtI; });
					auto a_part_j = std::find_if(activePartPool().begin(), pend, [a](Part* p)->bool { return p == a.prtJ; });
					auto b_part_i = std::find_if(activePartPool().begin(), pend, [b](Part* p)->bool { return p == b.prtI; });
					auto b_part_j = std::find_if(activePartPool().begin(), pend, [b](Part* p)->bool { return p == b.prtJ; });

					bool a_is_ok = (a_part_i == pend) != (a_part_j == pend);
					bool b_is_ok = (b_part_i == pend) != (b_part_j == pend);

					if (a_is_ok && !b_is_ok) return true;
					else if (!a_is_ok && b_is_ok) return false;
					else if (a.dim != b.dim)return a.dim > b.dim;
					else return false;
				});
			}
			
			// make diag pool //
			diagPool().clear();
			diagPool().resize(activePartPool().size());
			diagPool().at(0).is_I = true;
			diagPool().at(0).rel = nullptr;
			diagPool().at(0).part = &model().ground();
			diagPool().at(0).rd = &diagPool().at(0);
			std::fill_n(diagPool().at(0).b, 6, 0.0);
			std::fill_n(diagPool().at(0).x, 6, 0.0);
			std::fill_n(diagPool().at(0).cm, 36, 0.0);
			for (Size i{ 0 }; i < 6; ++i)diagPool().at(0).cm[i * 6 + i] = 1.0;
			for (Size i = 1; i < diagPool().size(); ++i)
			{
				diagPool().at(i).rel = &relationPool().at(i - 1);
				diagPool().at(i).is_I = relationPool().at(i - 1).prtI == activePartPool().at(i);
				diagPool().at(i).part = diagPool().at(i).is_I ? relationPool().at(i - 1).prtI : relationPool().at(i - 1).prtJ;
				auto add_part = diagPool().at(i).is_I ? diagPool().at(i).rel->prtJ : diagPool().at(i).rel->prtI;
				diagPool().at(i).rd = &*std::find_if(diagPool().begin(), diagPool().end(), [&](Diag &d) {return d.part == add_part; });
			}
			
			// make remainder pool //
			remainderPool().clear();
			remainderPool().resize(relationPool().size() - activePartPool().size() + 1);
			for (Size i = 0; i < remainderPool().size(); ++i) 
			{
				auto &r = remainderPool().at(i);

				r.rel = &relationPool().at(i + diagPool().size() - 1);
				r.cm_blk_series.clear();
				r.cm_blk_series.push_back(Remainder::Block());
				r.cm_blk_series.back().diag = &*std::find_if(diagPool().begin(), diagPool().end(), [&r](Diag&d) {return r.rel->prtI == d.part; });
				r.cm_blk_series.back().is_I = true;
				r.cm_blk_series.push_back(Remainder::Block());
				r.cm_blk_series.back().diag = &*std::find_if(diagPool().begin(), diagPool().end(), [&r](Diag&d) {return r.rel->prtJ == d.part; });
				r.cm_blk_series.back().is_I = false;

				for (auto rd = diagPool().rbegin(); rd < diagPool().rend(); ++rd)
				{
					auto &d = *rd;
					
					// 判断是不是地 //
					if (d.rel)
					{
						auto diag_part = d.is_I ? d.rel->prtI : d.rel->prtJ;
						auto add_part = d.is_I ? d.rel->prtJ : d.rel->prtI;

						// 判断当前remainder加法元素是否存在（不为0）
						auto diag_blk = std::find_if(r.cm_blk_series.begin(), r.cm_blk_series.end(), [&](Remainder::Block &blk) {return blk.diag->part == diag_part; });
						auto add_blk = std::find_if(r.cm_blk_series.begin(), r.cm_blk_series.end(), [&](Remainder::Block &blk) {return blk.diag->part == add_part; });
						if (diag_blk != r.cm_blk_series.end())
						{
							if (add_blk != r.cm_blk_series.end())
							{
								r.cm_blk_series.erase(add_blk);
							}
							else
							{
								Remainder::Block blk;
								blk.is_I = diag_blk->is_I;
								
								blk.diag = &*std::find_if(diagPool().begin(), diagPool().end(), [&](Diag&d) {return d.part == add_part; });
								
								r.cm_blk_series.push_back(blk);
							}
						}
					}
				}


			}
			
			// allocate memory //
			imp_->rows = 0;
			imp_->cols = 0;
			for (auto &d : diagPool()) 
			{
				d.rows = 0;
				if (d.rel && d.rel->dim < 6)
				{
					d.rows = imp_->rows;
					imp_->rows += 6 - d.rel->dim;
				}
			}
			for (auto &r : remainderPool()) imp_->cols += r.rel->dim ;

			imp_->A_.clear();
			imp_->A_.resize(imp_->rows*imp_->cols, 0.0);
			imp_->x_.clear();
			imp_->x_.resize(imp_->rows*1, 0.0);
			imp_->b_.clear();
			imp_->b_.resize(imp_->cols*1, 0.0);

			imp_->U_.resize(imp_->rows*imp_->cols, 0.0);
			imp_->tau_.resize(std::max(imp_->rows, imp_->cols), 0.0);
		}
		auto DiagSolver::kinPos()->void 
		{
			double pm[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
			s_mc(4, 4, pm, const_cast<double *>(*model().ground().pm()));

			setIterCount(0);

			for (; iterCount() < maxIterCount(); setIterCount(iterCount() + 1))
			{
				updDiagCp();
				updRemainderCp();

				double error{ 0.0 };
				for (auto d = diagPool().begin() + 1; d < diagPool().end(); ++d)for (Size i{ 0 }; i < d->rel->dim; ++i)error = std::max(error, std::abs(d->b[i]));
				for (auto &r : remainderPool())for (Size i{ 0 }; i < r.rel->dim; ++i)error = std::max(error, std::abs(r.b[i]));
				if (error < maxError())return;

				updDiagCm();
				updRemainderCm();
				updA();
				updB();
				updX();

				// 将x写入diag, 重新乘Q, 并反向做行变换
				for (auto d = diagPool().begin() + 1; d<diagPool().end(); ++d)
				{
					double tem[6];
					s_vc(6 - d->rel->dim, imp_->x_.data() + d->rows, d->x + d->rel->dim);
					//s_mm(6, 1, 6, d->Q, d->x, tem);
					s_householder_ut_q_dot(6, d->rel->dim, 1, d->U, d->tau, d->x, tem);
					s_vc(6, tem, d->x);
					s_va(6, d->rd->x, d->x);
				}

				// 将速度更新为矩阵
				for (auto d = diagPool().begin() + 1; d<diagPool().end(); ++d)
				{
					double pm[4][4];
					double pq[7];

					s_vc(6, d->x, pq);

					double theta = s_norm(3, pq + 3);
					pq[6] = std::cos(theta / 2);

					double factor = theta < 1e-4 ? 0.5 : std::sin(theta / 2) / theta;
					s_nv(3, factor, pq + 3);

					s_pq2pm(pq, *pm);

					double final_pm[4][4];
					s_pm2pm(*pm, *d->part->pm(), *final_pm);

					d->part->setPm(*final_pm);
				}
			}
		}
		auto DiagSolver::kinVel()->void 
		{
			s_fill(6, 1, 0.0, const_cast<double *>(model().ground().vs()));
			// make A
			updDiagCm();
			updRemainderCm();
			updA();
			// make b
			updDiagCv();
			updRemainderCv();
			updB();
			// using qr to solve x
			updX();

			// 将x写入diag, 重新乘Q, 并反向做行变换
			for (auto d = diagPool().begin() + 1; d<diagPool().end(); ++d)
			{
				s_mc(6 - d->rel->dim, 1, imp_->x_.data() + d->rows, d->x + d->rel->dim);
				//s_mm(6, 1, 6, d->Q, d->x, const_cast<double *>(d->part->vs()));
				s_householder_ut_q_dot(6, d->rel->dim, 1, d->U, d->tau, d->x, const_cast<double *>(d->part->vs()));
				s_ma(6, 1, d->rd->part->vs(), const_cast<double *>(d->part->vs()));
			}
		}
		auto DiagSolver::kinAcc()->void 
		{
			// make A
			updDiagCm();
			updRemainderCm();
			updA();
			// make b
			updDiagCa();
			updRemainderCa();
			updB();
			// using qr to solve x
			updX();

			// 将x写入diag, 重新乘Q, 并反向做行变换
			for (auto d = diagPool().begin() + 1; d<diagPool().end(); ++d)
			{
				s_mc(6 - d->rel->dim, 1, imp_->x_.data() + d->rows, d->x + d->rel->dim);
				//s_mm(6, 1, 6, d->Q, d->x, const_cast<double *>(d->part->as()));
				s_householder_ut_q_dot(6, d->rel->dim, 1, d->U, d->tau, d->x, const_cast<double *>(d->part->as()));
				s_ma(6, 1, d->rd->part->as(), const_cast<double *>(d->part->as()));
			}
		}
		auto DiagSolver::dynFce()->void 
		{
			// make A
			updDiagCm();
			updRemainderCm();
			updA();
			// make b
			updDiagPf();
			updBf();
			// using qr to solve x
			updXf();
		}
		auto DiagSolver::updDiagCm()->void
		{
			// upd diag cm data //
			for (auto d = diagPool().begin() + 1; d < diagPool().end(); ++d)
			{
				Size pos{ 0 };
				for (auto &c : d->rel->cst_pool_)
				{
					double cm[36];
					double *cmI = d->is_I ? d->cm : cm;
					double *cmJ = d->is_I ? cm : d->cm;

					c.constraint->cptGlbCm(cmI + pos, cmJ + pos, d->rel->dim, d->rel->dim);
					pos += c.constraint->dim();
					
					// make ut and qr
					s_householder_ut(6, d->rel->dim, d->cm, d->U, d->tau);
					//s_householder_ut2qr(6, d->rel->dim, d->U, d->tau, d->Q, d->R);
				}
			}
		}
		auto DiagSolver::updDiagCp()->void 
		{
			for (auto d = diagPool().begin() + 1; d < diagPool().end(); ++d)
			{
				Size pos{ 0 };
				for (auto &c : d->rel->cst_pool_)
				{
					c.constraint->cptCp(d->b + pos);
					pos += c.constraint->dim();
				}
			}
		}
		auto DiagSolver::updDiagCv()->void 
		{
			for (auto d = diagPool().begin() + 1; d < diagPool().end(); ++d)
			{
				Size pos{ 0 };
				for (auto &c : d->rel->cst_pool_)
				{
					c.constraint->cptCv(d->b + pos);
					pos += c.constraint->dim();
				}
			}
		}
		auto DiagSolver::updDiagCa()->void 
		{
			for (auto d = diagPool().begin() + 1; d < diagPool().end(); ++d)
			{
				Size pos{ 0 };
				for (auto &c : d->rel->cst_pool_)
				{
					c.constraint->cptCa(d->b + pos);
					pos += c.constraint->dim();
				}
			}
		}
		auto DiagSolver::updDiagPf()->void 
		{
			for (auto d = diagPool().begin() + 1; d < diagPool().end(); ++d)
			{
				double prt_as[6], prt_f[6];
				
				d->part->cptPrtPf(prt_f);
				d->part->cptPrtAs(prt_as);
				s_mma(6, 1, 6, -1.0, *d->part->prtIm(), prt_as, prt_f);
				s_tf(*d->part->pm(), prt_f, d->b);
			}
		}
		auto DiagSolver::updRemainderCm()->void
		{
			// upd remainder data //
			for (auto &r : remainderPool())
			{
				Size pos{ 0 };
				for (auto &c : r.rel->cst_pool_)
				{
					c.constraint->cptGlbCm(r.cmI + pos, r.cmJ + pos, r.rel->dim, r.rel->dim);
					pos += c.constraint->dim();
				}
			}
		}
		auto DiagSolver::updRemainderCp()->void
		{
			// upd remainder data //
			for (auto &r : remainderPool())
			{
				Size pos{ 0 };
				for (auto &c : r.rel->cst_pool_)
				{
					c.constraint->cptCp(r.b + pos);
					pos += c.constraint->dim();
				}
			}
		}
		auto DiagSolver::updRemainderCv()->void
		{
			// upd remainder data //
			for (auto &r : remainderPool())
			{
				Size pos{ 0 };
				for (auto &c : r.rel->cst_pool_)
				{
					c.constraint->cptCv(r.b + pos);
					pos += c.constraint->dim();
				}
			}
		}
		auto DiagSolver::updRemainderCa()->void
		{
			// upd remainder data //
			for (auto &r : remainderPool())
			{
				Size pos{ 0 };
				for (auto &c : r.rel->cst_pool_)
				{
					c.constraint->cptCa(r.b + pos);
					pos += c.constraint->dim();
				}
			}
		}
		auto DiagSolver::updA()->void
		{
			auto cols{ 0 };
			for (auto &r : remainderPool())
			{
				for (auto &b : r.cm_blk_series)
				{
					//s_mm(6 - b.diag->rel->dim, r.rel->dim, 6, b.diag->Q + dynamic::id(0, b.diag->rel->dim, 6), ColMajor{ 6 }, b.is_I ? r.cmI : r.cmJ, r.rel->dim, imp_->A_.data() + dynamic::id(b.diag->rows, cols, imp_->rows), imp_->cols);
					
					double tem[36];
					s_householder_ut_qt_dot(6, b.diag->rel->dim, r.rel->dim, b.diag->U, b.diag->tau, b.is_I ? r.cmI : r.cmJ, tem);
					s_mc(6 - b.diag->rel->dim, r.rel->dim, tem + dynamic::id(b.diag->rel->dim, 0, r.rel->dim), r.rel->dim, imp_->A_.data() + dynamic::id(b.diag->rows, cols, imp_->rows), imp_->cols);
				}
				cols += r.rel->dim;
			}
		}
		auto DiagSolver::updB()->void
		{
			// 求解对角线上的未知数
			for (auto d = diagPool().begin() + 1; d<diagPool().end(); ++d)
			{
				s_fill(6, 1, 0.0, d->x);
				s_sov_lm(d->rel->dim, 1, d->U, ColMajor{ d->rel->dim }, d->b, 1, d->x, 1);
			}
			// 使用已求出的未知数，用以构建b
			auto cols{ 0 };
			for (auto &r : remainderPool())
			{
				for (auto &b : r.cm_blk_series)
				{
					double tem[6];
					auto cm = b.is_I ? r.cmJ : r.cmI;//这里是颠倒的，因为加到右侧需要乘-1.0
					//s_mm(6, 1, b.diag->rel->dim, b.diag->Q, 6, b.diag->x, 1, tem, 1);
					s_householder_ut_q_dot(6, b.diag->rel->dim, 1, b.diag->U, b.diag->tau, b.diag->x, tem);
					s_mma(r.rel->dim, 1, 6, cm, ColMajor{ r.rel->dim }, tem, 1, r.b, 1);
				}
				s_mc(r.rel->dim, 1, r.b, imp_->b_.data() + cols);
				cols += r.rel->dim;
			}
		}
		auto DiagSolver::updX()->void
		{
			// 求解x
			s_householder_ut(imp_->cols, imp_->rows, imp_->A_.data(), ColMajor{ imp_->rows }, imp_->U_.data(), ColMajor{ imp_->rows }, imp_->tau_.data(), 1);
			s_householder_ut_sov(imp_->cols, imp_->rows, 1, imp_->U_.data(), ColMajor{ imp_->rows }, imp_->tau_.data(), 1, imp_->b_.data(), 1, imp_->x_.data(), 1);
		}
		auto DiagSolver::updBf()->void
		{
			for (auto d = diagPool().rbegin(); d<diagPool().rend() -1; ++d)
			{
				// 做行变换
				s_va(6, d->b, d->rd->b);

				// dot Q //
				double tem[6];
				//s_mm(6, 1, 6, d->Q, ColMajor{ 6 }, d->b, 1, tem, 1);
				s_householder_ut_qt_dot(6, d->rel->dim, 1, d->U, d->tau, d->b, tem);
				s_vc(6, tem, d->b);
				s_vc(6 - d->rel->dim, d->b + d->rel->dim, imp_->b_.data() + d->rows);
			}
		}
		auto DiagSolver::updXf()->void
		{
			s_householder_ut(imp_->rows, imp_->cols, imp_->A_.data(), imp_->U_.data(), imp_->tau_.data());
			s_householder_ut_sov(imp_->rows, imp_->cols, 1, imp_->U_.data(), imp_->tau_.data(), imp_->b_.data(), imp_->x_.data());

			// 将已经求出的x更新到remainder中，此后将已知数移到右侧
			auto cols{ 0 };
			for (auto &r : remainderPool())
			{
				auto pos{ 0 };
				for (auto &c : r.rel->cst_pool_)
				{
					c.constraint->setCf(imp_->x_.data() + cols + pos);
					pos += c.constraint->dim();
				}
				for (auto &b : r.cm_blk_series)
				{
					double tem[6], tem2[6];
					s_mm(6, 1, r.rel->dim, b.is_I ? r.cmJ : r.cmI, imp_->x_.data() + cols, tem);
					//s_mma(6, 1, 6, b.diag->Q, ColMajor{ 6 }, tem, 1, b.diag->b, 1);
					s_householder_ut_qt_dot(6, b.diag->rel->dim, 1, b.diag->U, b.diag->tau, tem, tem2);
					s_ma(6, 1, tem2, b.diag->b);
				}
				
				cols += r.rel->dim;
			}

			for (auto d = diagPool().begin() + 1; d < diagPool().end(); ++d)
			{
				s_sov_um(d->rel->dim, 1, d->U, d->b, d->x);
				auto pos{ 0 };
				for (auto &c : d->rel->cst_pool_)
				{
					c.constraint->setCf(d->x + pos);
					pos += c.constraint->dim();
				}
			}

		}
		auto DiagSolver::relationPool()->std::vector<Relation>& { return imp_->relation_pool_; }
		auto DiagSolver::activePartPool()->std::vector<Part*>& { return imp_->part_pool_; }
		auto DiagSolver::diagPool()->std::vector<Diag>& { return imp_->diag_pool_; }
		auto DiagSolver::remainderPool()->std::vector<Remainder>& { return imp_->remainder_pool_; }
		auto DiagSolver::plotRelation()->void
		{
			std::size_t name_size{ 0 };
			for (auto prt : activePartPool())
			{
				name_size = std::max(prt->name().size(), name_size);
			}
			
			for (auto prt : activePartPool())
			{
				std::string s(name_size, ' ');
				s.replace(0, prt->name().size(), prt->name().data());

				std::cout << s << ":";
				
				if (prt == &model().ground())
				{
					std::cout << "  6x6 ";
				}
				else
					std::cout << "      ";



				for (auto &rel : relationPool())
				{
					std::cout << " ";
					if (rel.prtI == prt)
						std::cout << " 6x" << rel.dim;
					else if (rel.prtJ == prt)
						std::cout << "-6x" << rel.dim;
					else
						std::cout << "    ";

					std::cout << " ";
				}
				std::cout << std::endl;
			}
		}
		auto DiagSolver::plotDiag()->void
		{
			std::size_t name_size{ 0 };
			for (auto prt : activePartPool())
			{
				name_size = std::max(prt->name().size(), name_size);
			}
			
			for (auto prt : activePartPool())
			{
				std::string s(name_size, ' ');
				s.replace(0, prt->name().size(), prt->name().data());

				std::cout << s << ":";
				
				
				
				
				for (auto &d : diagPool())
				{
					if (d.rel == nullptr)
					{
						if (prt == &model().ground())
						{
							std::cout << "  6x6 ";
						}
						else
							std::cout << "      ";

						continue;
					}

					auto &rel = *d.rel;
					std::cout << " ";
					if (d.is_I && rel.prtI == prt)
					{
						std::cout << " 6x" << rel.dim;
					}
					else if (!d.is_I && rel.prtJ == prt)
					{
						std::cout << "-6x" << rel.dim;
					}
					else
						std::cout << "    ";
					std::cout << " ";

				}
				std::cout << std::endl;
			}
		}
		auto DiagSolver::plotRemainder()->void
		{
			std::size_t name_size{ 0 };
			for (auto prt : activePartPool())
			{
				name_size = std::max(prt->name().size(), name_size);
			}
			
			for (auto prt : activePartPool())
			{
				std::string s(name_size, ' ');
				s.replace(0, prt->name().size(), prt->name().data());

				std::cout << s << ":";

				for (auto &r : remainderPool())
				{
					std::cout << " ";

					bool found{ false };
					for (auto blk : r.cm_blk_series)
					{
						if (prt == blk.diag->part)
						{
							found = true;
							if (blk.is_I)
								std::cout << " 6x" << r.rel->dim;
							else
								std::cout << "-6x" << r.rel->dim;
						}
					}
					if (!found)std::cout << "    ";

					std::cout << " ";
				}
				std::cout << std::endl;
			}
		}
		DiagSolver::~DiagSolver() = default;
		DiagSolver::DiagSolver(const std::string &name, Size max_iter_count, double max_error) :Solver(name, max_iter_count, max_error){}
		DiagSolver::DiagSolver(Object &father, const aris::core::XmlElement &xml_ele) : Solver(father, xml_ele){}
		DiagSolver::DiagSolver(const DiagSolver &other) = default;
		DiagSolver::DiagSolver(DiagSolver &&other) = default;
		DiagSolver& DiagSolver::operator=(const DiagSolver &other) = default;
		DiagSolver& DiagSolver::operator=(DiagSolver &&other) = default;


	}
}
