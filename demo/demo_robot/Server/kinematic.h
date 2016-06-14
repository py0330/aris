#ifndef KINEMATIC_H
#define KINEMATIC_H

#include <aris.h>

namespace robot
{
	class Robot:public aris::dynamic::Model 
	{
	public:
        auto pee()const->const double * { return pee_; }
		auto setPee(const double *pee)->void;
        auto pin()const->const double * { return pin_; }
        auto getPin(double *Pin)->void {std::copy_n(this->pin_,4,Pin);}
		auto setPin(const double *pin)->void;
        auto setVin(const double *vin)->void;

        auto up()->aris::dynamic::Part& { return *up_; }
        auto down()->aris::dynamic::Part& { return *down_; }
        auto p1a()->aris::dynamic::Part& { return *p1a_; }
        auto p1b()->aris::dynamic::Part& { return *p1b_; }
        auto p2a()->aris::dynamic::Part& { return *p2a_; }
        auto p2b()->aris::dynamic::Part& { return *p2b_; }
        auto p3a()->aris::dynamic::Part& { return *p3a_; }
        auto p3b()->aris::dynamic::Part& { return *p3b_; }
        auto p4a()->aris::dynamic::Part& { return *p4a_; }
        auto p4b()->aris::dynamic::Part& { return *p4b_; }

        auto r1j()->aris::dynamic::Marker& { return *r1j_; }
        auto r1i()->aris::dynamic::Marker& { return *r1i_; }
        auto p1j()->aris::dynamic::Marker& { return *p1j_; }
        auto p1i()->aris::dynamic::Marker& { return *p1i_; }
        auto u1j()->aris::dynamic::Marker& { return *u1j_; }
        auto u1i()->aris::dynamic::Marker& { return *u1i_; }
        auto r2j()->aris::dynamic::Marker& { return *r2j_; }
        auto r2i()->aris::dynamic::Marker& { return *r2i_; }
        auto p2j()->aris::dynamic::Marker& { return *p2j_; }
        auto p2i()->aris::dynamic::Marker& { return *p2i_; }
        auto u2j()->aris::dynamic::Marker& { return *u2j_; }
        auto u2i()->aris::dynamic::Marker& { return *u2i_; }
        auto s3j()->aris::dynamic::Marker& { return *s3j_; }
        auto s3i()->aris::dynamic::Marker& { return *s3i_; }
        auto p3j()->aris::dynamic::Marker& { return *p3j_; }
        auto p3i()->aris::dynamic::Marker& { return *p3i_; }
        auto u3j()->aris::dynamic::Marker& { return *u3j_; }
        auto u3i()->aris::dynamic::Marker& { return *u3i_; }
        auto s4j()->aris::dynamic::Marker& { return *s4j_; }
        auto s4i()->aris::dynamic::Marker& { return *s4i_; }
        auto p4j()->aris::dynamic::Marker& { return *p4j_; }
        auto p4i()->aris::dynamic::Marker& { return *p4i_; }
        auto u4j()->aris::dynamic::Marker& { return *u4j_; }
        auto u4i()->aris::dynamic::Marker& { return *u4i_; }

        auto r1()->aris::dynamic::RevoluteJoint& { return *r1_; }
        auto p1()->aris::dynamic::TranslationalJoint& { return *p1_; }
        auto u1()->aris::dynamic::UniversalJoint& { return *u1_; }
        auto r2()->aris::dynamic::RevoluteJoint& { return *r2_; }
        auto p2()->aris::dynamic::TranslationalJoint& { return *p2_; }
        auto u2()->aris::dynamic::UniversalJoint& { return *u2_; }
        auto s3()->aris::dynamic::SphericalJoint& { return *s3_; }
        auto p3()->aris::dynamic::TranslationalJoint& { return *p3_; }
        auto u3()->aris::dynamic::UniversalJoint& { return *u3_; }
        auto s4()->aris::dynamic::SphericalJoint& { return *s4_; }
        auto p4()->aris::dynamic::TranslationalJoint& { return *p4_; }
        auto u4()->aris::dynamic::UniversalJoint& { return *u4_; }

        auto m1()->aris::dynamic::Motion& { return *m1_; }
        auto m2()->aris::dynamic::Motion& { return *m2_; }
        auto m3()->aris::dynamic::Motion& { return *m3_; }
        auto m4()->aris::dynamic::Motion& { return *m4_; }

        auto gm()->aris::dynamic::GeneralMotion& { return *gm_; }

		virtual void loadXml(const aris::core::XmlElement &ele)override;
		using Model::loadXml;
	private:
		virtual void kinFromPin() final override;
		virtual void kinFromVin() final override;

		aris::dynamic::GeneralMotion *gm_;
		aris::dynamic::Part *p1a_, *p1b_, *p2a_, *p2b_, *p3a_, *p3b_, *p4a_, *p4b_, *up_, *down_;
		aris::dynamic::Marker *r1i_, *r1j_, *p1i_, *p1j_, *u1i_, *u1j_, *r2i_, *r2j_, *p2i_, *p2j_, *u2i_, *u2j_, *s3i_, *s3j_, *p3i_, *p3j_, *u3i_, *u3j_, *s4i_, *s4j_, *p4i_, *p4j_, *u4i_, *u4j_;
		aris::dynamic::RevoluteJoint *r1_, *r2_;
		aris::dynamic::TranslationalJoint *p1_, *p2_, *p3_, *p4_;
		aris::dynamic::UniversalJoint *u1_, *u2_, *u3_, *u4_;
		aris::dynamic::SphericalJoint *s3_, *s4_;
		aris::dynamic::Motion *m1_, *m2_, *m3_, *m4_;

		union
		{
			struct { double y_, z_, a_, c_; };
			double pee_[4];
		};
		union
		{
			struct { double l1_, l2_, l3_, l4_; };
			double pin_[4];
		};
	};
	
	struct PlanParam :public aris::server::GaitParamBase
	{
		int total_count_{ 1000 };
	};

}

#endif
