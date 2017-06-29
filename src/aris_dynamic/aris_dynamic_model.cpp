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
#include "aris_dynamic_model_joint.h"
#include "aris_dynamic_model_solver.h"

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
		Interaction::Interaction(Object &father, const aris::core::XmlElement &xml_ele): DynEle(father, xml_ele)
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
		auto Constraint::cptCv(double *cv)const->void 
		{ 
			double dv[6], dv_in_I[6];
			s_vc(6, makJ().vs(), dv);
			s_vs(6, makI().vs(), dv);
			s_inv_tv(*makI().fatherPart().pm(), dv, dv_in_I);
			s_mm(dim(), 1, 6, prtCmPtrI(), ColMajor{ dim() }, dv_in_I, 1, cv, 1);
		};
		auto Constraint::cptCa(double *ca)const->void
		{
			double vm_cross_vn[6], tem[6];
			s_cv(makI().fatherPart().glbVs(), makJ().fatherPart().glbVs(), vm_cross_vn);
			s_inv_tv(*makI().fatherPart().pm(), vm_cross_vn, tem);
			s_mmi(dim(), 1, 6, prtCmPtrI(), ColMajor{ dim() }, tem, 1, ca, 1);
		}
		auto Constraint::setCf(const double *cf)const->void { std::copy(cf, cf + dim(), const_cast<double *>(cfPtr())); }
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
		Environment::Environment(Object &father, const aris::core::XmlElement &xml_ele):Element(father, xml_ele)
		{
			std::copy_n(attributeMatrix(xml_ele, "gravity", 1, 6).data(), 6, gravity_);
		}
		
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
		auto Marker::saveXml(aris::core::XmlElement &xml_ele) const->void
		{
			DynEle::saveXml(xml_ele);
			double pe[6];
			s_pm2pe(*prtPm(), pe);
			xml_ele.SetAttribute("pe", core::Matrix(1, 6, pe).toString().c_str());
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
		auto Part::markerPool()->aris::core::ObjectPool<Marker, Element>& { return *imp_->marker_pool_; }
		auto Part::markerPool()const->const aris::core::ObjectPool<Marker, Element>& { return *imp_->marker_pool_; }
		auto Part::geometryPool()->aris::core::ObjectPool<Geometry, Element>& { return *imp_->geometry_pool_; }
		auto Part::geometryPool()const->const aris::core::ObjectPool<Geometry, Element>&{ return *imp_->geometry_pool_; }
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
		Part::Part(const std::string &name, const double *im, const double *pm, const double *vs, const double *as, bool active): Coordinate(name, active)
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
		auto Motion::cptCp(double *cp)const->void { Constraint::cptCp(cp); cp[0] += mp(); }
		auto Motion::cptCv(double *cv)const->void { Constraint::cptCv(cv); cv[0] += mv(); }
		auto Motion::cptCa(double *ca)const->void { Constraint::cptCa(ca); ca[0] += ma(); }
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
		auto GeneralMotion::cptCv(double *cv)const->void { Constraint::cptCv(cv); s_inv_tva(*mpm(), mvs(), cv); }
		auto GeneralMotion::cptCa(double *ca)const->void { s_inv_tv(*mpm(), mas(), ca); }
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

		struct Calibrator::Imp{};
		Calibrator::~Calibrator() = default;
		Calibrator::Calibrator(Object &father, const aris::core::XmlElement &xml_ele) : Element(father, xml_ele), imp_(new Imp){}
		Calibrator::Calibrator(const std::string &name) : Element(name), imp_(new Imp) {}
		Calibrator::Calibrator(const Calibrator&) = default;
		Calibrator::Calibrator(Calibrator&&) = default;
		Calibrator& Calibrator::operator=(const Calibrator&) = default;
		Calibrator& Calibrator::operator=(Calibrator&&) = default;

		struct Simulator::Imp{ };
		auto Simulator::simulate(const PlanFunction &plan, void *param, std::int32_t param_size, SimResult &result)->void
		{
			result.allocateMemory();
			// 记录初始位置 //
			result.record();
			// 记录轨迹中的位置 //
			for (PlanParam plan_param{ &model(), 1, param, param_size }; plan(plan_param) != 0; ++plan_param.count_);
			// 记录结束位置 //
			result.record();
		}
		Simulator::~Simulator() = default;
		Simulator::Simulator(Object &father, const aris::core::XmlElement &xml_ele) : Element(father, xml_ele), imp_(new Imp){}
		Simulator::Simulator(const std::string &name) : Element(name), imp_(new Imp) {}
		Simulator::Simulator(const Simulator&) = default;
		Simulator::Simulator(Simulator&&) = default;
		Simulator& Simulator::operator=(const Simulator&) = default;
		Simulator& Simulator::operator=(Simulator&&) = default;

		struct SimResult::TimeResult::Imp { std::deque<double> time_; };
		auto SimResult::TimeResult::saveXml(aris::core::XmlElement &xml_ele)const->void
		{
			Element::saveXml(xml_ele);

			std::stringstream ss;
			ss << std::setprecision(15);
			ss.str().reserve((25 * 1 + 1)*imp_->time_.size());

			for (auto &t:imp_->time_)ss << t << std::endl;

			xml_ele.SetText(ss.str().c_str());
		}
		auto SimResult::TimeResult::record()->void { imp_->time_.push_back(model().time()); }
		auto SimResult::TimeResult::restore(Size pos)->void { model().setTime(imp_->time_.at(pos)); }
		SimResult::TimeResult::~TimeResult() = default;
		SimResult::TimeResult::TimeResult(Object &father, const aris::core::XmlElement &xml_ele) : Element(father, xml_ele), imp_(new Imp)
		{
			// 以下导入数据 //
			std::stringstream ss(std::string(xml_ele.GetText()));
			for (double t; ss >> t, !ss.eof(); imp_->time_.push_back(t));
		}
		SimResult::TimeResult::TimeResult(const std::string &name) : Element(name), imp_(new Imp) {}
		SimResult::TimeResult::TimeResult(const SimResult::TimeResult&) = default;
		SimResult::TimeResult::TimeResult(SimResult::TimeResult&&) = default;
		SimResult::TimeResult& SimResult::TimeResult::operator=(const TimeResult&) = default;
		SimResult::TimeResult& SimResult::TimeResult::operator=(TimeResult&&) = default;

		struct SimResult::PartResult::Imp
		{
			Part *part_;
			std::deque<std::array<double, 6> > pe_;
			std::deque<std::array<double, 6> > vs_;
			std::deque<std::array<double, 6> > as_;

			Imp(Part* part) :part_(part) {};
		};
		auto SimResult::PartResult::saveXml(aris::core::XmlElement &xml_ele)const->void
		{
			Element::saveXml(xml_ele);
			
			xml_ele.SetAttribute("part", part().name().c_str());
			std::stringstream ss;
			ss << std::setprecision(15);
			ss.str().reserve((25 * 18 + 1)*imp_->pe_.size());

			for (auto pe = imp_->pe_.begin(), vs = imp_->vs_.begin(), as = imp_->as_.begin(); pe < imp_->pe_.end(); ++pe, ++vs, ++as)
			{
				for (auto e : *pe) ss << e << " ";
				for (auto e : *vs)ss << e << " ";
				for (auto e : *as)ss << e << " ";
				ss << std::endl;
			}

			xml_ele.SetText(ss.str().c_str());
		}
		auto SimResult::PartResult::part()->Part& { return *imp_->part_; }
		auto SimResult::PartResult::record()->void
		{
			std::array<double, 6> result;
			s_pm2pe(*part().pm(), result.data());
			imp_->pe_.push_back(result);
			std::copy(static_cast<const double*>(part().vs()), static_cast<const double*>(part().vs()) + 6, result.data());
			imp_->vs_.push_back(result);
			std::copy(static_cast<const double*>(part().as()), static_cast<const double*>(part().as()) + 6, result.data());
			imp_->as_.push_back(result);
		}
		auto SimResult::PartResult::restore(Size pos)->void 
		{ 
			part().setPe(imp_->pe_.at(pos).data());
			part().setVs(imp_->vs_.at(pos).data());
			part().setAs(imp_->as_.at(pos).data());
		}
		SimResult::PartResult::~PartResult() = default;
		SimResult::PartResult::PartResult(Object &father, const aris::core::XmlElement &xml_ele) : Element(father, xml_ele), imp_(new Imp(nullptr))
		{
			// 以下寻找对应的part //
			if (model().findByName("part_pool") == model().children().end())
				throw std::runtime_error("you must insert \"part_pool\" node before insert " + type() + " \"" + name() + "\"");

			auto &part_pool = static_cast<aris::core::ObjectPool<Part, Element>&>(*model().findByName("part_pool"));

			if (!xml_ele.Attribute("part"))throw std::runtime_error(std::string("xml element \"") + name() + "\" must have Attribute \"part\"");
			auto p = part_pool.findByName(xml_ele.Attribute("part"));
			if (p == part_pool.end())	throw std::runtime_error(std::string("can't find part for PartResult \"") + this->name() + "\"");

			imp_->part_ = &*p;

			// 以下导入数据 //
			std::stringstream ss(std::string(xml_ele.GetText()));
			std::array<double, 6> pe, vs, as;
			for (Size i{ 0 }; !ss.eof(); ++i)
			{
				if (i < 6) ss >> pe[i];
				else if (i < 12) ss >> vs[i - 6];
				else if (i < 18) ss >> as[i - 12];

				if (i == 6)imp_->pe_.push_back(pe);
				if (i == 12)imp_->vs_.push_back(vs);
				if (i == 18) { imp_->as_.push_back(as); i = -1; }
			}

		}
		SimResult::PartResult::PartResult(const std::string &name, Part &part) : Element(name), imp_(new Imp(&part)) {}
		SimResult::PartResult::PartResult(const SimResult::PartResult&) = default;
		SimResult::PartResult::PartResult(SimResult::PartResult&&) = default;
		SimResult::PartResult& SimResult::PartResult::operator=(const PartResult&) = default;
		SimResult::PartResult& SimResult::PartResult::operator=(PartResult&&) = default;

		struct SimResult::ConstraintResult::Imp
		{
			Constraint *constraint_;
			std::deque<std::array<double, 6> > cf_;

			Imp(Constraint* constraint) :constraint_(constraint) {};
		};
		auto SimResult::ConstraintResult::saveXml(aris::core::XmlElement &xml_ele)const->void
		{
			Element::saveXml(xml_ele);
			
			xml_ele.SetAttribute("constraint", constraint().name().c_str());

			std::stringstream ss;
			ss << std::setprecision(15);
			ss.str().reserve((25 * 6 + 1)*imp_->cf_.size());
			for (auto &cf : imp_->cf_)
			{
				for (Size i(-1); ++i < constraint().dim();) ss << cf[i] << " ";
				ss << std::endl;
			}
			
			xml_ele.SetText(ss.str().c_str());
		}
		auto SimResult::ConstraintResult::constraint()->Constraint& { return *imp_->constraint_; }
		auto SimResult::ConstraintResult::record()->void
		{
			std::array<double, 6> result{0,0,0,0,0,0};
			std::copy(constraint().cfPtr(), constraint().cfPtr() + constraint().dim(), result.data());
			imp_->cf_.push_back(result);
		}
		auto SimResult::ConstraintResult::restore(Size pos)->void{	constraint().setCf(imp_->cf_.at(pos).data());}
		SimResult::ConstraintResult::~ConstraintResult() = default;
		SimResult::ConstraintResult::ConstraintResult(Object &father, const aris::core::XmlElement &xml_ele) : Element(father, xml_ele), imp_(new Imp(nullptr))
		{
			// 以下寻找对应的constraint //
			if (!xml_ele.Attribute("constraint"))throw std::runtime_error(std::string("xml element \"") + name() + "\" must have Attribute \"constraint\"");
			if (!imp_->constraint_ && model().findByName("joint_pool") != model().children().end())
			{
				auto &pool = static_cast<aris::core::ObjectPool<Joint, Element>&>(*model().findByName("joint_pool"));
				auto c = pool.findByName(xml_ele.Attribute("constraint"));
				if (c != pool.end())imp_->constraint_ = &*c;
			}
			if (!imp_->constraint_ && model().findByName("motion_pool") != model().children().end())
			{
				auto &pool = static_cast<aris::core::ObjectPool<Motion, Element>&>(*model().findByName("motion_pool"));
				auto c = pool.findByName(xml_ele.Attribute("constraint"));
				if (c != pool.end())imp_->constraint_ = &*c;
			}
			if (!imp_->constraint_ && model().findByName("general_motion_pool") != model().children().end())
			{
				auto &pool = static_cast<aris::core::ObjectPool<GeneralMotion, Element>&>(*model().findByName("general_motion_pool"));
				auto c = pool.findByName(xml_ele.Attribute("constraint"));
				if (c != pool.end())imp_->constraint_ = &*c;
			}
			if (!imp_->constraint_)throw std::runtime_error(std::string("can't find constraint for ConstraintResult \"") + this->name() + "\"");
			
			// 以下读取数据 //
			std::stringstream ss(std::string(xml_ele.GetText()));
			std::array<double, 6> cf{0,0,0,0,0,0};
			for (Size i{ 0 }; !ss.eof(); ss >> cf[i++])
			{
				if (i == constraint().dim()) 
				{
					i = 0;
					imp_->cf_.push_back(cf);
				}
			}

		}
		SimResult::ConstraintResult::ConstraintResult(const std::string &name, Constraint &constraint) : Element(name), imp_(new Imp(&constraint)) {}
		SimResult::ConstraintResult::ConstraintResult(const SimResult::ConstraintResult&) = default;
		SimResult::ConstraintResult::ConstraintResult(SimResult::ConstraintResult&&) = default;
		SimResult::ConstraintResult& SimResult::ConstraintResult::operator=(const ConstraintResult&) = default;
		SimResult::ConstraintResult& SimResult::ConstraintResult::operator=(ConstraintResult&&) = default;

		struct SimResult::Imp
		{
			TimeResult *time_result_;
			aris::core::ObjectPool<PartResult, Element> *part_result_pool_;
			aris::core::ObjectPool<ConstraintResult, Element> *constraint_result_pool_;
		};
		auto SimResult::timeResult()->TimeResult& { return *imp_->time_result_; }
		auto SimResult::partResultPool()->aris::core::ObjectPool<SimResult::PartResult, Element>& { return *imp_->part_result_pool_; }
		auto SimResult::constraintResultPool()->aris::core::ObjectPool<SimResult::ConstraintResult, Element>& { return *imp_->constraint_result_pool_; }
		auto SimResult::allocateMemory()->void
		{
			partResultPool().clear();
			for (auto &p : model().partPool())partResultPool().add<PartResult>(p.name() + "_result", p);
			constraintResultPool().clear();
			for (auto &c : model().jointPool())constraintResultPool().add<ConstraintResult>(c.name() + "_result", c);
			for (auto &c : model().motionPool())constraintResultPool().add<ConstraintResult>(c.name() + "_result", c);
			for (auto &c : model().generalMotionPool())constraintResultPool().add<ConstraintResult>(c.name() + "_result", c);
		}
		auto SimResult::record()->void
		{
			timeResult().record();
			for (auto &p : partResultPool())p.record();
			for (auto &p : constraintResultPool())p.record();
		}
		auto SimResult::restore(Size pos)->void
		{
			timeResult().restore(pos);
			for (auto &r : partResultPool())r.restore(pos);
			for (auto &r : constraintResultPool())r.restore(pos);
		}
		auto SimResult::size()const->Size { return timeResult().imp_->time_.size() == 0 ? 0 : timeResult().imp_->time_.size() - 1; }
		auto SimResult::clear()->void
		{
			timeResult().imp_->time_.clear();
			for (auto &r : partResultPool())
			{
				r.imp_->pe_.clear();
				r.imp_->vs_.clear();
				r.imp_->as_.clear();
			}
			for (auto &r : constraintResultPool())r.imp_->cf_.clear();
		}
		SimResult::~SimResult() = default;
		SimResult::SimResult(Object &father, const aris::core::XmlElement &xml_ele) : Element(father, xml_ele), imp_()
		{
			imp_->time_result_ = findOrInsert<TimeResult>("time_result");
			imp_->constraint_result_pool_ = findOrInsert<aris::core::ObjectPool<SimResult::ConstraintResult, Element> >("constraint_result_pool");
			imp_->part_result_pool_ = findOrInsert<aris::core::ObjectPool<SimResult::PartResult, Element> >("part_result_pool");
		}
		SimResult::SimResult(const std::string &name) : Element(name), imp_(new Imp())
		{
			imp_->time_result_ = &add<TimeResult>("time_result");
			imp_->part_result_pool_ = &add<aris::core::ObjectPool<SimResult::PartResult, Element> >("part_result_pool");
			imp_->constraint_result_pool_ = &add<aris::core::ObjectPool<SimResult::ConstraintResult, Element> >("constraint_result_pool");
		}
		SimResult::SimResult(const SimResult&other) : Element(other), imp_(other.imp_)
		{
			imp_->time_result_ = findType<TimeResult >("time_result");
			imp_->constraint_result_pool_ = findType<aris::core::ObjectPool<SimResult::ConstraintResult, Element> >("constraint_result_pool");
			imp_->part_result_pool_ = findType<aris::core::ObjectPool<SimResult::PartResult, Element> >("part_result_pool");
		}
		SimResult::SimResult(SimResult&&other) : Element(std::move(other)), imp_(std::move(other.imp_))
		{
			imp_->time_result_ = findType<TimeResult >("time_result");
			imp_->constraint_result_pool_ = findType<aris::core::ObjectPool<SimResult::ConstraintResult, Element> >("constraint_result_pool");
			imp_->part_result_pool_ = findType<aris::core::ObjectPool<SimResult::PartResult, Element> >("part_result_pool");
		}
		SimResult& SimResult::operator=(const SimResult&other)
		{
			Element::operator=(other);
			imp_ = other.imp_;
			imp_->time_result_ = findType<TimeResult >("time_result");
			imp_->constraint_result_pool_ = findType<aris::core::ObjectPool<SimResult::ConstraintResult, Element> >("constraint_result_pool");
			imp_->part_result_pool_ = findType<aris::core::ObjectPool<SimResult::PartResult, Element> >("part_result_pool");
			return *this;
		}
		SimResult& SimResult::operator=(SimResult&&other)
		{
			Element::operator=(std::move(other));
			imp_ = other.imp_;
			imp_->time_result_ = findType<TimeResult >("time_result");
			imp_->constraint_result_pool_ = findType<aris::core::ObjectPool<SimResult::ConstraintResult, Element> >("constraint_result_pool");
			imp_->part_result_pool_ = findType<aris::core::ObjectPool<SimResult::PartResult, Element> >("part_result_pool");
			return *this;
		}

		struct Model::Imp
		{
			double time_{0.0};
			aris::core::Calculator calculator_;
			Environment *environment_;
			Part* ground_;
			aris::core::ObjectPool<Variable, Element> *variable_pool_;
			aris::core::ObjectPool<Part, Element> *part_pool_;
			aris::core::ObjectPool<Joint, Element> *joint_pool_;
			aris::core::ObjectPool<Motion, Element> *motion_pool_;
			aris::core::ObjectPool<GeneralMotion, Element> *general_motion_pool_;
			aris::core::ObjectPool<Force, Element> *force_pool_;
			aris::core::ObjectPool<Solver, Element> *solver_pool_;
			aris::core::ObjectPool<Simulator, Element> *simulator_pool_;
			aris::core::ObjectPool<SimResult, Element> *sim_result_pool_;

			std::vector<Size> mot_vec_phy2abs_, mot_vec_sla2abs_;
		};
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

			setTime(Object::attributeDouble(xml_ele, "time", 0.0));

			imp_->environment_ = findOrInsert<Environment>("environment");
			imp_->variable_pool_ = findOrInsert<aris::core::ObjectPool<Variable, Element>>("variable_pool");
			imp_->part_pool_ = findOrInsert<aris::core::ObjectPool<Part, Element>>("part_pool");
			imp_->joint_pool_ = findOrInsert<aris::core::ObjectPool<Joint, Element>>("joint_pool");
			imp_->motion_pool_ = findOrInsert<aris::core::ObjectPool<Motion, Element>>("motion_pool");
			imp_->general_motion_pool_ = findOrInsert<aris::core::ObjectPool<GeneralMotion, Element>>("general_motion_pool");
			imp_->force_pool_ = findOrInsert<aris::core::ObjectPool<Force, Element>>("force_pool");
			imp_->solver_pool_ = findOrInsert<aris::core::ObjectPool<Solver, Element>>("solver_pool");
			imp_->simulator_pool_ = findOrInsert<aris::core::ObjectPool<Simulator, Element>>("simulator_pool");
			imp_->sim_result_pool_ = findOrInsert<aris::core::ObjectPool<SimResult, Element>>("sim_result_pool");
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
		auto Model::saveXml(aris::core::XmlElement &xml_ele)const->void
		{
			Root::saveXml(xml_ele);
			xml_ele.SetAttribute("time", time());
		}
		auto Model::time()const->double { return imp_->time_; }
		auto Model::setTime(double time)->void { imp_->time_ = time; }
		auto Model::calculator()->aris::core::Calculator& { return imp_->calculator_; }
		auto Model::environment()->aris::dynamic::Environment& { return *imp_->environment_; }
		auto Model::variablePool()->aris::core::ObjectPool<Variable, Element>& { return *imp_->variable_pool_; }
		auto Model::partPool()->aris::core::ObjectPool<Part, Element>& { return *imp_->part_pool_; }
		auto Model::jointPool()->aris::core::ObjectPool<Joint, Element>& { return *imp_->joint_pool_; }
		auto Model::motionPool()->aris::core::ObjectPool<Motion, Element>& { return *imp_->motion_pool_; }
		auto Model::generalMotionPool()->aris::core::ObjectPool<GeneralMotion, Element>& { return *imp_->general_motion_pool_; }
		auto Model::forcePool()->aris::core::ObjectPool<Force, Element>& { return *imp_->force_pool_; }
		auto Model::solverPool()->aris::core::ObjectPool<Solver, Element>& { return *imp_->solver_pool_; }
		auto Model::simulatorPool()->aris::core::ObjectPool<Simulator, Element>& { return *imp_->simulator_pool_; }
		auto Model::simResultPool()->aris::core::ObjectPool<SimResult, Element>& { return *imp_->sim_result_pool_; }
		auto Model::ground()->Part& { return *imp_->ground_; }
		auto Model::addPartByPm(const double*pm, const double *prt_im)->Part& { return partPool().add<Part>("part_" + std::to_string(partPool().size()), prt_im, pm); }
		auto Model::addPartByPe(const double*pe, const char* eul_type, const double *prt_im)->Part& 
		{ 
			double pm[16];
			s_pe2pm(pe, pm, eul_type);
			return partPool().add<Part>("part_" + std::to_string(partPool().size()), prt_im, pm); 
		}
		auto Model::addPartByPq(const double*pq, const double *prt_im)->Part&
		{
			double pm[16];
			s_pq2pm(pq, pm);
			return partPool().add<Part>("part_" + std::to_string(partPool().size()), prt_im, pm);
		}
		auto Model::addRevoluteJoint(Part &first_part, Part &second_part, const double *position, const double *axis)->RevoluteJoint&
		{
			double glb_pm[16], loc_pm[16];
			s_sov_axes2pm(position, axis, axis, glb_pm, "zx");
			auto name = "joint_" + std::to_string(jointPool().size());
			s_inv_pm_dot_pm(*first_part.glbPm(), glb_pm, loc_pm);
			auto &mak_i = first_part.markerPool().add<Marker>(name + "_i", loc_pm);
			s_inv_pm_dot_pm(*second_part.glbPm(), glb_pm, loc_pm);
			auto &mak_j = second_part.markerPool().add<Marker>(name + "_j", loc_pm);
			return jointPool().add<RevoluteJoint>(name, mak_i, mak_j);
		}
		auto Model::addPrismaticJoint(Part &first_part, Part &second_part, const double *position, const double *axis)->PrismaticJoint&
		{
			double glb_pm[16], loc_pm[16];
			s_sov_axes2pm(position, axis, axis, glb_pm, "zx");
			auto name = "joint_" + std::to_string(jointPool().size());
			s_inv_pm_dot_pm(*first_part.glbPm(), glb_pm, loc_pm);
			auto &mak_i = first_part.markerPool().add<Marker>(name + "_i", loc_pm);
			s_inv_pm_dot_pm(*second_part.glbPm(), glb_pm, loc_pm);
			auto &mak_j = second_part.markerPool().add<Marker>(name + "_j", loc_pm);
			return jointPool().add<PrismaticJoint>(name, mak_i, mak_j);
		}
		auto Model::addUniversalJoint(Part &first_part, Part &second_part, const double *position, const double *first_axis, const double *second_axis)->UniversalJoint&
		{
			double glb_pm[16], loc_pm[16];
			s_sov_axes2pm(position, first_axis, second_axis, glb_pm, "zx");
			auto name = "joint_" + std::to_string(jointPool().size());
			s_inv_pm_dot_pm(*first_part.glbPm(), glb_pm, loc_pm);
			auto &mak_i = first_part.markerPool().add<Marker>(name + "_i", loc_pm);
			s_sov_axes2pm(position, second_axis, first_axis, glb_pm, "zx");
			s_inv_pm_dot_pm(*second_part.glbPm(), glb_pm, loc_pm);
			auto &mak_j = second_part.markerPool().add<Marker>(name + "_j", loc_pm);
			return jointPool().add<UniversalJoint>(name, mak_i, mak_j);
		}
		auto Model::addSphericalJoint(Part &first_part, Part &second_part, const double *position)->SphericalJoint&
		{
			double glb_pm[16]{ 1,0,0,position[0],0,1,0,position[1],0,0,1,position[2],0,0,0,1 }, loc_pm[16];
			auto name = "joint_" + std::to_string(jointPool().size());
			s_inv_pm_dot_pm(*first_part.glbPm(), glb_pm, loc_pm);
			auto &mak_i = first_part.markerPool().add<Marker>(name + "_i", loc_pm);
			s_inv_pm_dot_pm(*second_part.glbPm(), glb_pm, loc_pm);
			auto &mak_j = second_part.markerPool().add<Marker>(name + "_j", loc_pm);
			return jointPool().add<SphericalJoint>(name, mak_i, mak_j);
		}
		auto Model::addMotion(Joint &joint)->Motion&
		{
			Size dim;

			if (dynamic_cast<RevoluteJoint*>(&joint))
			{
				dim = 5;
			}
			else if (dynamic_cast<PrismaticJoint*>(&joint))
			{
				dim = 2;
			}
			else
			{
				throw std::runtime_error("wrong joint when Model::addMotion(joint)");
			}

			return motionPool().add<Motion>("motion_" + std::to_string(motionPool().size()), joint.makI(), joint.makJ(), dim);
		}
		auto Model::addGeneralMotion(Part &end_effector, Coordinate &reference, const double* pm)->GeneralMotion&
		{
			double pm_prt[16], pm_target_in_ground[16];
			s_pm_dot_pm(*reference.pm(), pm, pm_target_in_ground);
			s_inv_pm_dot_pm(*end_effector.pm(), pm_target_in_ground, pm_prt);

			auto name = "general_motion_" + std::to_string(generalMotionPool().size());
			auto &mak_i = end_effector.markerPool().add<Marker>(name + "_i", pm_prt);
			auto &mak_j = dynamic_cast<Part*>(&reference) ? dynamic_cast<Part&>(reference).markerPool().add<Marker>(name + "_j") : dynamic_cast<Marker&>(reference);
			return generalMotionPool().add<GeneralMotion>(name, mak_i, mak_j);
		}
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
		auto Model::motionAtPhy(Size phy_id)->Motion& { return motionPool().at(imp_->mot_vec_phy2abs_.at(phy_id)); }
		auto Model::motionAtPhy(Size phy_id)const->const Motion&{ return motionPool().at(imp_->mot_vec_phy2abs_.at(phy_id)); }
		auto Model::motionAtSla(Size sla_id)->Motion& { return motionPool().at(imp_->mot_vec_sla2abs_.at(sla_id)); }
		auto Model::motionAtSla(Size sla_id)const->const Motion&{ return motionPool().at(imp_->mot_vec_sla2abs_.at(sla_id)); }
		Model::~Model() = default;
		Model::Model(const std::string &name):Root(name)
		{
			registerChildType<Environment>();

			registerChildType<aris::core::ObjectPool<Variable, Element>>();
			registerChildType<MatrixVariable>();
			registerChildType<StringVariable>();

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
			registerChildType<GroundCombineSolver>();
			registerChildType<LltGroundDividedSolver>();
			registerChildType<LltPartDividedSolver>();
			registerChildType<DiagSolver>();

			registerChildType<aris::core::ObjectPool<Simulator, Element>>();
			registerChildType<Simulator>();
			registerChildType<SolverSimulator>();
			registerChildType<AdamsSimulator>();
			
			registerChildType<aris::core::ObjectPool<SimResult, Element>>();
			registerChildType<SimResult>();
			registerChildType<aris::core::ObjectPool<SimResult::PartResult, Element>>();
			registerChildType<aris::core::ObjectPool<SimResult::ConstraintResult, Element>>();
			registerChildType<SimResult::PartResult>();
			registerChildType<SimResult::ConstraintResult>();
			registerChildType<SimResult::TimeResult>();

			imp_->environment_ = &this->add<Environment>("environment");
			imp_->variable_pool_ = &this->add<aris::core::ObjectPool<Variable, Element>>("variable_pool");
			imp_->part_pool_ = &this->add<aris::core::ObjectPool<Part, Element>>("part_pool");
			imp_->joint_pool_ = &this->add<aris::core::ObjectPool<Joint, Element>>("joint_pool");
			imp_->motion_pool_ = &this->add<aris::core::ObjectPool<Motion, Element>>("motion_pool");
			imp_->general_motion_pool_ = &this->add<aris::core::ObjectPool<GeneralMotion, Element>>("general_motion_pool");
			imp_->force_pool_ = &this->add<aris::core::ObjectPool<Force, Element>>("force_pool");
			imp_->solver_pool_ = &this->add<aris::core::ObjectPool<Solver, Element>>("solver_pool");
			imp_->simulator_pool_ = &this->add<aris::core::ObjectPool<Simulator, Element>>("simulator_pool");
			imp_->sim_result_pool_ = &this->add<aris::core::ObjectPool<SimResult, Element>>("sim_result_pool");

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
		auto ParasolidGeometry::prtPm()const->const double4x4&{return imp_->prt_pm_;}
		auto ParasolidGeometry::filePath()const->const std::string &{ return imp_->graphic_file_path; }
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

		auto SingleComponentForce::saveXml(aris::core::XmlElement &xml_ele) const->void
		{
			Force::saveXml(xml_ele);
			xml_ele.SetAttribute("component", static_cast<int>(this->component_axis_));
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
		
		struct SolverSimulator::Imp
		{
			Solver *solver_;

			Imp(Solver *solver) :solver_(solver) { };
		};
		auto SolverSimulator::saveXml(aris::core::XmlElement &xml_ele) const->void
		{
			Simulator::saveXml(xml_ele);
			xml_ele.SetAttribute("solver", solver().name().c_str());
		}
		auto SolverSimulator::solver()->Solver& { return *imp_->solver_; }
		auto SolverSimulator::simulate(const PlanFunction &plan, void *param, std::int32_t param_size, SimResult &result)->void
		{
			solver().allocateMemory();
			result.allocateMemory();
			// 记录初始位置 //
			result.record();
			// 记录轨迹中的位置 //
			for (PlanParam plan_param{ &model(), 1, param, param_size }; plan(plan_param) != 0; ++plan_param.count_)
			{
				solver().kinPos();
				if (solver().iterCount() == solver().maxIterCount())throw std::runtime_error("simulate failed because kinPos() failed at " + std::to_string(plan_param.count_) + " count");
				solver().kinVel();
				solver().dynAccAndFce();
				result.record();
			}
			// 记录结束位置 //
			result.record();
			result.restore(0);
		}
		SolverSimulator::~SolverSimulator() = default;
		SolverSimulator::SolverSimulator(Object &father, const aris::core::XmlElement &xml_ele) : Simulator(father, xml_ele), imp_(new Imp(nullptr))
		{
			if (model().findByName("solver_pool") == model().children().end())
				throw std::runtime_error("you must insert \"solver_pool\" node before insert " + type() + " \"" + name() + "\"");

			auto &solver_pool = static_cast<aris::core::ObjectPool<Solver, Element>&>(*model().findByName("solver_pool"));

			if (!xml_ele.Attribute("solver"))throw std::runtime_error(std::string("xml element \"") + name() + "\" must have Attribute \"solver\"");
			auto s = solver_pool.findByName(xml_ele.Attribute("solver"));
			if (s == solver_pool.end())	throw std::runtime_error(std::string("can't find solver for element \"") + this->name() + "\"");

			imp_->solver_ = &*s;
		}
		SolverSimulator::SolverSimulator(const std::string &name, Solver &solver) : Simulator(name), imp_(new Imp(&solver)) {}
		SolverSimulator::SolverSimulator(const SolverSimulator&) = default;
		SolverSimulator::SolverSimulator(SolverSimulator&&) = default;
		SolverSimulator& SolverSimulator::operator=(const SolverSimulator&) = default;
		SolverSimulator& SolverSimulator::operator=(SolverSimulator&&) = default;

		struct AdamsSimulator::Imp
		{
		};
		auto AdamsSimulator::saveAdams(const std::string &filename, SimResult &result, Size pos)->void
		{
			std::string filename_ = filename;
			if (filename_.size() < 4 || filename_.substr(filename.size() - 4, 4) != ".cmd")
			{
				filename_ += ".cmd";
			}

			std::ofstream file;
			file.open(filename_, std::ios::out | std::ios::trunc);

			saveAdams(file, result, pos);

			file.close();
		}
		auto AdamsSimulator::saveAdams(std::ofstream &file, SimResult &result, Size pos)->void
		{
			// 生成akima曲线 //
			std::vector<double> time(result.size() + 1);
			std::vector<std::vector<double>> mot_akima(model().motionPool().size(), std::vector<double>(result.size() + 1));
			std::vector<std::vector<std::array<double, 6>>> gm_akima(model().generalMotionPool().size(), std::vector<std::array<double, 6>>(result.size() + 1));
			if (pos == -1)
			{
				if (result.size() < 4)throw std::runtime_error("failed to AdamsSimulator::saveAdams: because result size is smaller than 4\n");
				
				for (Size i(-1); ++i < result.size() + 1;)
				{
					result.restore(i);
					time.at(i) = model().time();
					for (Size j(-1); ++j < model().motionPool().size();)
					{
						model().motionPool().at(j).updMp();
						mot_akima.at(j).at(i) = model().motionPool().at(j).mp();
					}
					for (Size j(-1); ++j < model().generalMotionPool().size();)
					{
						model().generalMotionPool().at(j).updMpm();
						model().generalMotionPool().at(j).getMpe(gm_akima.at(j).at(i).data(), "123");
					}
				}
			}

			// 生成ADAMS模型
			result.restore(pos == -1 ? 0 : pos);
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
				<< "    x_component_gravity = " << model().environment().gravity()[0] << "  &\r\n"
				<< "    y_component_gravity = " << model().environment().gravity()[1] << "  &\r\n"
				<< "    z_component_gravity = " << model().environment().gravity()[2] << "\r\n"
				<< "!\r\n";
			for (auto &part : model().partPool())
			{
				if (&part == &model().ground())
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
					s_pm2pe(*part.pm(), pe, "313");
					core::Matrix ori(1, 3, &pe[3]), loc(1, 3, &pe[0]);

					file << "!----------------------------------- " << part.name() << " -----------------------------------!\r\n"
						<< "!\r\n"
						<< "!\r\n"
						<< "defaults coordinate_system  &\r\n"
						<< "    default_coordinate_system = ." << model().name() << ".ground\r\n"
						<< "!\r\n"
						<< "part create rigid_body name_and_position  &\r\n"
						<< "    part_name = ." << model().name() << "." << part.name() << "  &\r\n"
						<< "    adams_id = " << adamsID(part) << "  &\r\n"
						<< "    location = (" << loc.toString() << ")  &\r\n"
						<< "    orientation = (" << ori.toString() << ")\r\n"
						<< "!\r\n"
						<< "defaults coordinate_system  &\r\n"
						<< "    default_coordinate_system = ." << model().name() << "." << part.name() << " \r\n"
						<< "!\r\n";


					double mass = part.prtIm()[0][0] == 0 ? 1 : part.prtIm()[0][0];
					std::fill_n(pe, 6, 0);
					pe[0] = part.prtIm()[1][5] / mass;
					pe[1] = -part.prtIm()[0][5] / mass;
					pe[2] = part.prtIm()[0][4] / mass;

					file << "! ****** cm and mass for current part ******\r\n"
						<< "marker create  &\r\n"
						<< "    marker_name = ." << model().name() << "." << part.name() << ".cm  &\r\n"
						<< "    adams_id = " << adamsID(part) + model().markerSize() << "  &\r\n"
						<< "    location = ({" << pe[0] << "," << pe[1] << "," << pe[2] << "})  &\r\n"
						<< "    orientation = (" << "{0,0,0}" << ")\r\n"
						<< "!\r\n";

					double pm[16];
					double im[6][6];

					pe[0] = -pe[0];
					pe[1] = -pe[1];
					pe[2] = -pe[2];

					s_pe2pm(pe, pm);
					s_im2im(pm, *part.prtIm(), *im);

					//！注意！//
					//Adams里对惯量矩阵的定义貌似和我自己的定义在Ixy,Ixz,Iyz上互为相反数。别问我为什么,我也不知道。
					file << "part create rigid_body mass_properties  &\r\n"
						<< "    part_name = ." << model().name() << "." << part.name() << "  &\r\n"
						<< "    mass = " << part.prtIm()[0][0] << "  &\r\n"
						<< "    center_of_mass_marker = ." << model().name() << "." << part.name() << ".cm  &\r\n"
						<< "    inertia_marker = ." << model().name() << "." << part.name() << ".cm  &\r\n"
						<< "    ixx = " << im[3][3] << "  &\r\n"
						<< "    iyy = " << im[4][4] << "  &\r\n"
						<< "    izz = " << im[5][5] << "  &\r\n"
						<< "    ixy = " << -im[4][3] << "  &\r\n"
						<< "    izx = " << -im[5][3] << "  &\r\n"
						<< "    iyz = " << -im[5][4] << "\r\n"
						<< "!\r\n";
				}

				//导入marker
				for (auto &marker : part.markerPool())
				{
					double pe[6];

					s_pm2pe(*marker.prtPm(), pe, "313");
					core::Matrix ori(1, 3, &pe[3]), loc(1, 3, &pe[0]);

					file << "marker create  &\r\n"
						<< "marker_name = ." << model().name() << "." << part.name() << "." << marker.name() << "  &\r\n"
						<< "adams_id = " << adamsID(marker) << "  &\r\n"
						<< "location = (" << loc.toString() << ")  &\r\n"
						<< "orientation = (" << ori.toString() << ")\r\n"
						<< "!\r\n";
				}
				for(auto &geometry:part.geometryPool())
				{
					if (ParasolidGeometry* geo = dynamic_cast<ParasolidGeometry*>(&geometry))
					{
						double pe[6];
						s_pm2pe(*geo->prtPm(), pe, "313");
						core::Matrix ori(1, 3, &pe[3]), loc(1, 3, &pe[0]);

						file << "file parasolid read &\r\n"
							<< "	file_name = \"" << geo->filePath() << "\" &\r\n"
							<< "	type = ASCII" << " &\r\n"
							<< "	part_name = " << part.name() << " &\r\n"
							<< "	location = (" << loc.toString() << ") &\r\n"
							<< "	orientation = (" << ori.toString() << ") &\r\n"
							<< "	relative_to = ." << model().name() << "." << part.name() << " \r\n"
							<< "\r\n";
					}
					else
					{
						throw std::runtime_error("unrecognized geometry type:" + geometry.type());
					}

				}
			}
			for (auto &joint : model().jointPool())
			{
				std::string type;
				if (dynamic_cast<RevoluteJoint*>(&joint))type = "revolute";
				else if (dynamic_cast<PrismaticJoint*>(&joint))type = "translational";
				else if (dynamic_cast<UniversalJoint*>(&joint))type = "universal";
				else if (dynamic_cast<SphericalJoint*>(&joint))type = "spherical";
				else throw std::runtime_error("unrecognized joint type:" + joint.type());
				
				file << "constraint create joint " << type << "  &\r\n"
					<< "    joint_name = ." << model().name() << "." << joint.name() << "  &\r\n"
					<< "    adams_id = " << adamsID(joint) << "  &\r\n"
					<< "    i_marker_name = ." << model().name() << "." << joint.makI().fatherPart().name() << "." << joint.makI().name() << "  &\r\n"
					<< "    j_marker_name = ." << model().name() << "." << joint.makJ().fatherPart().name() << "." << joint.makJ().name() << "  \r\n"
					<< "!\r\n";
			}
			for (auto &motion : model().motionPool())
			{
				std::string axis_names[6]{ "x","y","z","B1","B2","B3" };
				std::string axis_name = axis_names[motion.axis()];

				std::string akima = motion.name() + "_akima";
				std::string akima_func = "AKISPL(time,0," + akima + ")";
				std::string polynomial_func = static_cast<std::stringstream &>(std::stringstream() << std::setprecision(16) << motion.mp() << " + " << motion.mv() << " * time + " << motion.ma()*0.5 << " * time * time").str();

				// 构建akima曲线 //
				if (pos == -1)
				{
					file << "data_element create spline &\r\n"
						<< "    spline_name = ." << model().name() + "." + motion.name() + "_akima &\r\n"
						<< "    adams_id = " << adamsID(motion) << "  &\r\n"
						<< "    units = m &\r\n"
						<< "    x = " << time.at(0);
					for (auto p = time.begin() + 1; p < time.end(); ++p)
					{
						file << "," << *p;
					}
					file << "    y = " << mot_akima.at(motion.id()).at(0);
					for (auto p = mot_akima.at(motion.id()).begin() + 1; p < mot_akima.at(motion.id()).end(); ++p)
					{
						file << "," << *p;
					}
					file << " \r\n!\r\n";
				}

				file << "constraint create motion_generator &\r\n"
					<< "    motion_name = ." << model().name() << "." << motion.name() << "  &\r\n"
					<< "    adams_id = " << adamsID(motion) << "  &\r\n"
					<< "    i_marker_name = ." << model().name() << "." << motion.makI().fatherPart().name() << "." << motion.makI().name() << "  &\r\n"
					<< "    j_marker_name = ." << model().name() << "." << motion.makJ().fatherPart().name() << "." << motion.makJ().name() << "  &\r\n"
					<< "    axis = " << axis_name << "  &\r\n"
					<< "    function = \"" << (pos == -1 ? akima_func : polynomial_func) << "\"  \r\n"
					<< "!\r\n";
			}
			for (auto &gm : model().generalMotionPool())
			{
				file << "ude create instance  &\r\n"
					<< "    instance_name = ." << model().name() << "." << gm.name() << "  &\r\n"
					<< "    definition_name = .MDI.Constraints.general_motion  &\r\n"
					<< "    location = 0.0, 0.0, 0.0  &\r\n"
					<< "    orientation = 0.0, 0.0, 0.0  \r\n"
					<< "!\r\n";

				file << "variable modify  &\r\n"
					<< "	variable_name = ." << model().name() << "." << gm.name() << ".i_marker  &\r\n"
					<< "	object_value = ." << model().name() << "." << gm.makI().fatherPart().name() << "." << gm.makI().name() << " \r\n"
					<< "!\r\n";

				file << "variable modify  &\r\n"
					<< "	variable_name = ." << model().name() << "." << gm.name() << ".j_marker  &\r\n"
					<< "	object_value = ." << model().name() << "." << gm.makJ().fatherPart().name() << "." << gm.makJ().name() << " \r\n"
					<< "!\r\n";

				std::string axis_names[6]{ "t1", "t2", "t3", "r1", "r2", "r3" };

				double pe123[6], ve123[6], ae123[6];
				gm.getMpe(pe123, "123");
				gm.getMve(ve123, "123");
				gm.getMae(ae123, "123");
				for (Size i = 0; i < 6; ++i)
				{
					std::string akima = gm.name() + "_" + axis_names[i] + "_akima";
					std::string akima_func = "AKISPL(time,0," + akima + ")";
					std::string polynomial_func = static_cast<std::stringstream &>(std::stringstream() << std::setprecision(16) << pe123[i] << " + " << ve123[i] << " * time + " << ae123[i] * 0.5 << " * time * time").str();
					std::string func = pos == -1 ? akima_func : polynomial_func;

					// 构建akima曲线 //
					if (pos == -1)
					{
						file << "data_element create spline &\r\n"
							<< "    spline_name = ." << model().name() + "." + akima + " &\r\n"
							<< "    adams_id = " << model().motionPool().size() + adamsID(gm) * 6 + i << "  &\r\n"
							<< "    units = m &\r\n"
							<< "    x = " << time.at(0);
						for (auto p = time.begin() + 1; p < time.end(); ++p)
						{
							file << "," << *p;
						}
						file << "    y = " << gm_akima.at(gm.id()).at(0).at(i);
						for (auto p = gm_akima.at(gm.id()).begin() + 1; p < gm_akima.at(gm.id()).end(); ++p)
						{
							file << "," << p->at(i);
						}
						file << " \r\n!\r\n";
					}

					file << "variable modify  &\r\n"
						<< "	variable_name = ." << model().name() << "." << gm.name() << "." << axis_names[i] << "_type  &\r\n"
						<< "	integer_value = 1 \r\n"
						<< "!\r\n";

					file << "variable modify  &\r\n"
						<< "	variable_name = ." << model().name() << "." << gm.name() << "." << axis_names[i] << "_func  &\r\n"
						<< "	string_value = \"" + func + "\" \r\n"
						<< "!\r\n";

					file << "variable modify  &\r\n"
						<< "	variable_name = ." << model().name() << "." << gm.name() << "." << axis_names[i] << "_ic_disp  &\r\n"
						<< "	real_value = 0.0 \r\n"
						<< "!\r\n";

					file << "variable modify  &\r\n"
						<< "	variable_name = ." << model().name() << "." << gm.name() << "." << axis_names[i] << "_ic_velo  &\r\n"
						<< "	real_value = 0.0 \r\n"
						<< "!\r\n";
				}

				file << "ude modify instance  &\r\n"
					<< "	instance_name = ." << model().name() << "." << gm.name() << "\r\n"
					<< "!\r\n";
			}
			for (auto &force : model().forcePool())
			{
				if (dynamic_cast<SingleComponentForce *>(&force))
				{
					std::string type = "translational";

					file << "force create direct single_component_force  &\r\n"
						<< "    single_component_force_name = ." << model().name() << "." << force.name() << "  &\r\n"
						<< "    adams_id = " << adamsID(force) << "  &\r\n"
						<< "    type_of_freedom = " << type << "  &\r\n"
						<< "    i_marker_name = ." << model().name() << "." << force.makI().fatherPart().name() << "." << force.makI().name() << "  &\r\n"
						<< "    j_marker_name = ." << model().name() << "." << force.makJ().fatherPart().name() << "." << force.makJ().name() << "  &\r\n"
						<< "    action_only = off  &\r\n"
						<< "    function = \"" << dynamic_cast<SingleComponentForce&>(force).fce() << "\"  \r\n"
						<< "!\r\n";
				}
				
			}

			file << "!----------------------------------- Motify Active -------------------------------------!\r\n!\r\n!\r\n";
			for (auto &prt : model().partPool())
			{
				if ((&prt != &model().ground()) && (!prt.active()))
				{
					file << "part attributes  &\r\n"
						<< "    part_name = ." << model().name() << "." << prt.name() << "  &\r\n"
						<< "    active = off \r\n!\r\n";
				}
			}
			for (auto &jnt : model().jointPool())
			{
				if (!jnt.active())
				{
					file << "constraint attributes  &\r\n"
						<< "    constraint_name = ." << model().name() << "." << jnt.name() << "  &\r\n"
						<< "    active = off \r\n!\r\n";
				}
			}
			for (auto &mot : model().motionPool())
			{
				if (!mot.active())
				{
					file << "constraint attributes  &\r\n"
						<< "    constraint_name = ." << model().name() << "." << mot.name() << "  &\r\n"
						<< "    active = off \r\n!\r\n";
				}
			}
			for (auto &gm : model().generalMotionPool())
			{
				if (!gm.active())
				{
					file << "ude attributes  &\r\n"
						<< "    instance_name = ." << model().name() << "." << gm.name() << "  &\r\n"
						<< "    active = off \r\n!\r\n";
				}
			}
			for (auto &fce : model().forcePool())
			{
				if (!fce.active())
				{
					file << "force attributes  &\r\n"
						<< "    force_name = ." << model().name() << "." << fce.name() << "  &\r\n"
						<< "    active = off \r\n!\r\n";
				}
			}
		}
		auto AdamsSimulator::adamsID(const Marker &mak)const->Size
		{
			Size size{ 0 };

			for (auto &prt : model().partPool())
			{
				if (&prt == &mak.fatherPart()) break;
				size += prt.markerPool().size();
			}

			size += mak.id() + 1;

			return size;
		}
		AdamsSimulator::~AdamsSimulator() = default;
		AdamsSimulator::AdamsSimulator(Object &father, const aris::core::XmlElement &xml_ele) : SolverSimulator(father, xml_ele){}
		AdamsSimulator::AdamsSimulator(const std::string &name, Solver &solver) : SolverSimulator(name, solver){}
		AdamsSimulator::AdamsSimulator(const AdamsSimulator&) = default;
		AdamsSimulator::AdamsSimulator(AdamsSimulator&&) = default;
		AdamsSimulator& AdamsSimulator::operator=(const AdamsSimulator&) = default;
		AdamsSimulator& AdamsSimulator::operator=(AdamsSimulator&&) = default;
	}
}
