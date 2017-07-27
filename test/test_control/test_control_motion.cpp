#include <iostream>
#include <aris.h>
#include "test_control_ethercat.h"
#include "test_control_motion.h"

using namespace aris::control;

void test_elmo_enable()
{
	try
	{
		aris::control::Master m;
		m.registerChildType<Motion>();

		auto &st = m.slaveTypePool().add<SlaveType>("st", 0x00030924, 0x0000009a, 0x0000, 0x0300);
		auto &s1 = m.slavePool().add<Motion>("s1", st, 0, 0, 0, 0, 0, 0);


		auto &tx = s1.pdoGroupPool().add<PdoGroup>("index_1A00", 0x1A00, true);
		tx.add<Pdo>("index_6064", DO::INT32, 0x6064, 0x00);
		tx.add<Pdo>("index_606c", DO::INT32, 0x606c, 0x00);
		tx.add<Pdo>("index_6041", DO::UINT16, 0x6041, 0x00);

		auto &tx2 = s1.pdoGroupPool().add<PdoGroup>("index_1A0B", 0x1A0B, true);
		tx2.add<Pdo>("index_6061", DO::UINT8, 0x6061, 0x00);

		auto &tx3 = s1.pdoGroupPool().add<PdoGroup>("index_1A1F", 0x1A1F, true);
		tx3.add<Pdo>("index_6078", DO::INT16, 0x6078, 0x00);

		auto &rx = s1.pdoGroupPool().add<PdoGroup>("index_1605", 0x1605, false);
		rx.add<Pdo>("index_607A", DO::INT32, 0x607A, 0x00);
		rx.add<Pdo>("index_60FF", DO::INT32, 0x60FF, 0x00);
		rx.add<Pdo>("index_6071", DO::INT16, 0x6071, 0x00);
		rx.add<Pdo>("index_6072", DO::INT16, 0x6072, 0x00);
		rx.add<Pdo>("index_6040", DO::UINT16, 0x6040, 0x00);
		rx.add<Pdo>("index_6060", DO::UINT8, 0x6060, 0x00);

		m.setControlStrategy([&]()
		{
			static int count{ 0 };

			auto ret = s1.enable();

			if (++count % 1000 == 0) 
			{
				m.mout() << "count " << count << " : ret " << ret << '\0';
			}

			m.dataLogger().lout() << "count " << count << " : ret " << ret <<"\n";
			m.dataLogger().send();
		});
		m.dataLogger().start();
		m.start();
		for (auto i{ 0 }; i < 20; ++i)
		{
			aris::core::Msg msg;
			while (!m.recvOut(msg));
			std::cout << msg.data() << std::endl;
		}
		m.stop();
		m.dataLogger().stop();
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
