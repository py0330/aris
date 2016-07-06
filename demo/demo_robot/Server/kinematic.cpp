#include"kinematic.h"

using namespace aris::dynamic;

namespace robot
{
	auto Robot::setPee(const double *pee)->void
	{
        /*
		double pm[4][4];
		double pe[6]{ 0,pee[0],pee[1],pee[2],pee[3],0 };

		std::copy(pee, pee + 4, pee_);

		s_pe2pm(pe, *pm, "132");
		// A 是下平台（地面）的四个铰链点 B 是上平台的四个铰链点
		const double A[4][3]{ { 0,0,0.0435 },{ 0,0,-0.0435 },{ 0.29,-0.005,-0.3225 },{ 0.29,-0.005,0.3225 } };
		const double B[4][3]{ { 0,0,0.34315 },{ 0,0,-0.34315 },{ 0.29,0.015,-0.025 },{ 0.29,0.015,0.025 } };

		double B_g[4][3];
		double AB[4][3];

		std::fill(&B_g[0][0], &B_g[3][2], 0);
		std::fill(&AB[0][0], &AB[3][2], 0);

		for (auto i = 0; i < 4; ++i)
		{
			s_pp2pp(*pm, B[i], B_g[i]);
			s_vav(3, -1, A[i], 1, B_g[i], 0, AB[i]);
			pin_[i] = s_vnm(3, AB[i], 1);
		}

		double pe_a[4][6];
		double pe_b[4][6];

		for (auto i = 0; i < 4; ++i)
		{
			std::copy(&A[i][0], &A[i][3], pe_a[i]);
			std::fill(&pe_a[i][3], &pe_a[i][6], 0);
			std::copy(&B_g[i][0], &B_g[i][3], pe_b[i]);
			std::fill(&pe_b[i][3], &pe_b[i][6], 0);
		}

		pe_a[0][4] = atan2(AB[0][2], AB[0][1]);
		pe_b[0][4] = atan2(AB[0][2], AB[0][1]);
		pe_a[1][4] = atan2(AB[1][2], AB[1][1]);
		pe_b[1][4] = atan2(AB[1][2], AB[1][1]);

		aris::dynamic::s_nd(3, 1.0 / s_vnm(3, AB[2], 1), AB[2], 1);
		aris::dynamic::s_nd(3, 1.0 / s_vnm(3, AB[3], 1), AB[3], 1);

		pe_a[2][5] = -std::asin(AB[2][0]);
		pe_a[3][5] = -std::asin(AB[3][0]);
		pe_a[2][4] = std::atan2(AB[2][2] * std::cos(pe_a[2][5]), AB[2][1] * std::cos(pe_a[2][5]));
		pe_a[3][4] = std::atan2(AB[3][2] * std::cos(pe_a[3][5]), AB[3][1] * std::cos(pe_a[3][5]));

		std::copy(&pe_a[2][3], &pe_a[2][5], &pe_b[2][3]);
		std::copy(&pe_a[3][3], &pe_a[3][5], &pe_b[3][3]);

		this->up().setPm(down(), *pm);
		this->p1a().setPe(down(), pe_a[0]);
		this->p1b().setPe(down(), pe_b[0]);
		this->p2a().setPe(down(), pe_a[1]);
		this->p2b().setPe(down(), pe_b[1]);
		this->p3a().setPe(down(), pe_a[2]);
		this->p3b().setPe(down(), pe_b[2]);
		this->p4a().setPe(down(), pe_a[3]);
		this->p4b().setPe(down(), pe_b[3]);

		this->gm().update();
		this->m1().update();
		this->m2().update();
		this->m3().update();
        this->m4().update();

        pin_[0] = m1().motPos();
        pin_[1] = m2().motPos();
        pin_[2] = m3().motPos();
        pin_[3] = m4().motPos();

        // rt_printf( "pin:%f %f %f %f\n",pin_[0],pin_[1],pin_[2],pin_[3]);
        */
	}

	auto Robot::setPin(const double *pin)->void
	{
        this->m1().setMotPos(pin[0]);
        this->m2().setMotPos(pin[1]);
        this->m3().setMotPos(pin[2]);

        pin_[0] = m1().motPos();
        pin_[1] = m2().motPos();
        pin_[2] = m3().motPos();
	}

    auto Robot::setVin(const double *vin)->void
    {
        this->m1().setMotVel(vin[0]);
        this->m2().setMotVel(vin[1]);
        this->m3().setMotVel(vin[2]);
    }

	auto Robot::loadXml(const aris::core::XmlElement &ele)->void
	{
		Model::loadXml(ele);

        //part
        this->p1_ = &*partPool().findByName("part1");
        this->p2_ = &*partPool().findByName("part2");
        this->p3_ = &*partPool().findByName("part3");

        //marker
        //this->r1j_ = &*p1_->markerPool().findByName("r1j");
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
