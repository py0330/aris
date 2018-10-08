#include <iostream>
#include <aris_control.h>
#include "test_control_ethercat.h"
#include "test_control_motion.h"

using namespace aris::control;

void test_elmo_enable()
{
	try
	{
		aris::control::EthercatController m;
		m.registerType<EthercatMotion>();

		auto &s1 = m.slavePool().add<EthercatMotion>("s1", 0, 0x0000009a, 0x00030924, 0x000103F6, 0x0300, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 10.0, 110.0, 1.0, 0.0, 0.0);

		//auto &tx = s1.pdoPool().add<Pdo>("index_1A00", 0x1A00, true);
		//tx.add<PdoEntry>("index_6064", 0x6064, 0x00, sizeof(std::int32_t));
		//tx.add<PdoEntry>("index_606c", 0x606c, 0x00, sizeof(std::int32_t));
		//tx.add<PdoEntry>("index_6041", 0x6041, 0x00, sizeof(std::uint16_t));

		//auto &tx2 = s1.pdoPool().add<Pdo>("index_1A0B", 0x1A0B, true);
		//tx2.add<PdoEntry>("index_6061", 0x6061, 0x00, sizeof(std::uint8_t));

		//auto &tx3 = s1.pdoPool().add<Pdo>("index_1A1F", 0x1A1F, true);
		//tx3.add<PdoEntry>("index_6078", 0x6078, 0x00, sizeof(std::int16_t));

		//auto &rx = s1.pdoPool().add<Pdo>("index_1605", 0x1605, false);
		//rx.add<PdoEntry>("index_607A", 0x607A, 0x00, sizeof(std::int32_t));
		//rx.add<PdoEntry>("index_60FF", 0x60FF, 0x00, sizeof(std::int32_t));
		//rx.add<PdoEntry>("index_6071", 0x6071, 0x00, sizeof(std::int16_t));
		//rx.add<PdoEntry>("index_6072", 0x6072, 0x00, sizeof(std::int16_t));
		//rx.add<PdoEntry>("index_6040", 0x6040, 0x00, sizeof(std::uint16_t));
		//rx.add<PdoEntry>("index_6060", 0x6060, 0x00, sizeof(std::uint8_t));

		std::cout << m.xmlString() << std::endl;

		m.setControlStrategy([&]()
		{
			static int count{ 0 }, cos_count{ 0 };
			static int state{ 0 };// 0为 初始状态， 1为 使能结束， 2为 sin结束， 3为 去使能结束
			static double begin_pos;

			int ret;
			switch (state)
			{
			case 0:
				s1.setModeOfOperation(8);
				ret = s1.enable();
				if (ret == 0) 
				{
					m.mout() << "enabled at count " << count << std::endl;
					state = 1;
					cos_count = 0;
					begin_pos = s1.actualPos();
				}
				break;
			case 1:
				s1.setTargetPos(begin_pos + (1.0 - std::cos(++cos_count / 5000.0 * 2.0 * 3.141592653)) * 65536);
				if (cos_count == 5000)
				{
					m.mout() << "cos finished at count " << count << std::endl;
					state = 2;
				}
				break;
			case 2:
				ret = s1.disable();
				if (ret == 0)
				{
					m.mout() << "disabled at count " << count << std::endl;
					state = 3;
				}
				break;
			default:
				break;
			}

			if (++count % 1000 == 0) 
			{
				m.mout() << "count " << count << " : ret " << ret << '\0';
			}

		});
		std::cout << "test motion enable finished" << std::endl;
	}
	catch (std::exception &e)
	{
		std::cout << e.what() << std::endl;
	}
}

void test_control_motion()
{
	std::cout << std::endl << "-----------------test motion---------------------" << std::endl;
	test_elmo_enable();
	std::cout << "-----------------test motion finished------------" << std::endl << std::endl;

	return;
}
