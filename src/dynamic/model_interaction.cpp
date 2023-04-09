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

#include "aris/core/reflection.hpp"

#include "aris/dynamic/model.hpp"

namespace aris::dynamic{
	auto Interaction::prtNameM()const->std::string { return prt_name_M_; }
	auto Interaction::setPrtNameM(std::string_view name)->void { prt_name_M_ = name; }
	auto Interaction::prtNameN()const->std::string { return prt_name_N_; }
	auto Interaction::setPrtNameN(std::string_view name)->void { prt_name_N_ = name; }
	auto Interaction::makNameI()const->std::string { return mak_name_I_; }
	auto Interaction::setMakNameI(std::string_view name)->void { mak_name_I_ = name; }
	auto Interaction::makNameJ()const->std::string { return mak_name_J_; }
	auto Interaction::setMakNameJ(std::string_view name)->void { mak_name_J_ = name; }
	Interaction::Interaction(const std::string &name, Marker *makI, Marker *makJ, bool is_active)
		: DynEle(name, is_active), makI_(makI), makJ_(makJ) 
		, prt_name_M_(makI ? makI->fatherPart().name() : std::string())
		, prt_name_N_(makJ ? makJ->fatherPart().name() : std::string())
		, mak_name_I_(makI ? makI->name() : std::string())
		, mak_name_J_(makJ ? makJ->name() : std::string()){
	}

	struct Constraint::Imp { Size col_id_, blk_col_id_; double cf_[6]{ 0 }; };
	auto Constraint::cf() const noexcept->const double* { return imp_->cf_; }
	auto Constraint::setCf(const double *cf) noexcept->void { return s_vc(dim(), cf, imp_->cf_); }
	auto Constraint::cptCvDiff(double *cv)const noexcept->void{
		// 获取 cv //
		cptCv(cv);
		
		// 获取当前状态下所产生的cv //
		double dv[6], dv_in_I[6];
		s_vc(6, makJ()->vs(), dv);
		s_vs(6, makI()->vs(), dv);
		s_inv_tv(*makI()->pm(), dv, dv_in_I);
		s_mma(dim(), 1, 6, locCmI(), ColMajor{ dim() }, dv_in_I, 1, cv, 1);
	};
	auto Constraint::cptCa(double *ca)const noexcept->void{
		double vi_cross_vj[6], tem[6];
		s_cv(makI()->vs(), makJ()->vs(), vi_cross_vj);
		s_inv_tv(*makI()->pm(), vi_cross_vj, tem);
		s_mmi(dim(), 1, 6, locCmI(), ColMajor{ dim() }, tem, 1, ca, 1);
	}
	Constraint::~Constraint() = default;
	Constraint::Constraint(const std::string &name, Marker* makI, Marker* makJ, bool is_active) : Interaction(name, makI, makJ, is_active) {}
	ARIS_DEFINE_BIG_FOUR_CPP(Constraint);

	ARIS_REGISTRATION{
		aris::core::class_<Interaction>("Interaction")
			.inherit<aris::dynamic::DynEle>()
			.prop("prt_m", &Interaction::setPrtNameM, &Interaction::prtNameM)
			.prop("prt_n", &Interaction::setPrtNameN, &Interaction::prtNameN)
			.prop("mak_i", &Interaction::setMakNameI, &Interaction::makNameI)
			.prop("mak_j", &Interaction::setMakNameJ, &Interaction::makNameJ)
			;

		auto setCf = [](Constraint* c, aris::core::Matrix cf_mat)->void {c->setCf(cf_mat.data()); };
		auto getCf = [](Constraint* c)->aris::core::Matrix {	return aris::core::Matrix(1, c->dim(), c->cf()); };
		aris::core::class_<Constraint>("Constraint")
			.inherit<aris::dynamic::Interaction>()
			.prop("cf", &setCf, &getCf)
			;
	}
}
