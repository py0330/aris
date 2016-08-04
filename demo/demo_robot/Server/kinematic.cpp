#include"kinematic.h"

using namespace aris::dynamic;

namespace robot
{
	auto Robot::setPee(const double *pee)->void
	{
		std::copy(pee, pee + 3, pee_);

        double target_angle=std::atan2(pee[1],pee[0]);
        double temp_angle=std::acos(std::sqrt(pee[0]*pee[0]+pee[1]*pee[1])/2);
        double a_1=target_angle-temp_angle;
        double a_2=2*temp_angle;
        double a_3=pee[2]-a_1-a_2;

        double pe[3][6];

        for (auto i = 0; i < 3; ++i)
        {
            std::fill(&pe[i][0], &pe[i][6], 0);
        }
        pe[0][5]=a_1;
        pe[1][5]=a_2;
        pe[2][5]=a_3;


		this->r1j().update();
        this->p1().setPe(r1j(), pe[0],"123");
		this->r2j().update();
        this->p2().setPe(r2j(), pe[1],"123");
		this->r3j().update();
        this->p3().setPe(r3j(), pe[2],"123");

        this->m1().update();
        this->m2().update();
        this->m3().update();

        pin_[0] = m1().motPos();
        pin_[1] = m2().motPos();
        pin_[2] = m3().motPos();
	}

	auto Robot::setPin(const double *pin)->void
	{
        this->m1().setMotPos(pin[0]);
        this->m2().setMotPos(pin[1]);
        this->m3().setMotPos(pin[2]);

        std::copy(pin, pin + 3, pin_);
	}

    auto Robot::setVin(const double *vin)->void
    {
        this->m1().setMotVel(vin[0]);
        this->m2().setMotVel(vin[1]);
        this->m3().setMotVel(vin[2]);

        std::copy(vin, vin + 3, vin_);
    }

	auto Robot::loadXml(const aris::core::XmlElement &ele)->void
	{
		Model::loadXml(ele);

        //part
        this->p1_ = &*partPool().findByName("part1");
        this->p2_ = &*partPool().findByName("part2");
        this->p3_ = &*partPool().findByName("part3");
        this->ground_ = &*partPool().findByName("ground");

        //marker
        this->r1j_ = &*ground_->markerPool().findByName("r1j");
        this->r1i_ = &*p1_->markerPool().findByName("r1i");
        this->r2j_ = &*p1_->markerPool().findByName("r2j");
        this->r2i_ = &*p2_->markerPool().findByName("r2i");
        this->r3j_ = &*p2_->markerPool().findByName("r3j");
        this->r3i_ = &*p3_->markerPool().findByName("r3i");

        //joint
		this->r1_ = static_cast<decltype(r1_)>(&*jointPool().findByName("r1"));
        this->r2_ = static_cast<decltype(r2_)>(&*jointPool().findByName("r2"));
        this->r3_ = static_cast<decltype(r3_)>(&*jointPool().findByName("r3"));

        //motion
		this->m1_ = static_cast<decltype(m1_)>(&*motionPool().findByName("m1"));
		this->m2_ = static_cast<decltype(m2_)>(&*motionPool().findByName("m2"));
		this->m3_ = static_cast<decltype(m3_)>(&*motionPool().findByName("m3"));
	
	}

	auto Robot::kinFromPin()->void
	{
		
    }
	auto Robot::kinFromVin()->void
	{

    }

}
