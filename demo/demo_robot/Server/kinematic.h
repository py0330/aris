#ifndef KINEMATIC_H
#define KINEMATIC_H

#include <aris.h>

namespace robot
{
	class Robot:public aris::dynamic::Model 
	{
	public:
        auto getpee()const->const double * { return pee_; }
		auto setPee(const double *pee)->void;
        auto getpin()const->const double * { return pin_; }
		auto setPin(const double *pin)->void;
        //auto getVin()const->const double*{return vin_;}
        auto setVin(const double *vin)->void;

        //get part
        auto p1()->aris::dynamic::Part& { return *p1_; }
        auto p2()->aris::dynamic::Part& { return *p2_; }
        auto p3()->aris::dynamic::Part& { return *p3_; }

        //get marker
        auto r1j()->aris::dynamic::Marker& { return *r1j_; }
        auto r1i()->aris::dynamic::Marker& { return *r1i_; }
        auto r2j()->aris::dynamic::Marker& { return *r2j_; }
        auto r2i()->aris::dynamic::Marker& { return *r2i_; }
        auto r3j()->aris::dynamic::Marker& { return *r3j_; }
        auto r3i()->aris::dynamic::Marker& { return *r3i_; }

        //get the revolute joint
        auto r1()->aris::dynamic::RevoluteJoint& { return *r1_; }
        auto r2()->aris::dynamic::RevoluteJoint& { return *r2_; }
        auto r3()->aris::dynamic::RevoluteJoint& { return *r3_; }

        //get motion
        auto m1()->aris::dynamic::Motion& { return *m1_; }
        auto m2()->aris::dynamic::Motion& { return *m2_; }
        auto m3()->aris::dynamic::Motion& { return *m3_; }

		virtual void loadXml(const aris::core::XmlElement &ele)override;
		using Model::loadXml;
	private:
		virtual void kinFromPin() final override;
		virtual void kinFromVin() final override;

        aris::dynamic::Part *p1_, *p2_, *p3_;
        aris::dynamic::Marker *r1i_, *r1j_,  *r2i_, *r2j_, *r3i_, *r3j_;
        aris::dynamic::RevoluteJoint *r1_, *r2_,*r3_;
        aris::dynamic::Motion *m1_, *m2_, *m3_;

		union
		{
            struct { double x_, y_, a_; };
            double pee_[3];
		};
		union
		{
            struct { double a1_, a2_, a3_; };
            double pin_[3];
		};
	};

}

#endif
