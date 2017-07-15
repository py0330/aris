#include <iostream>
#include <aris.h>
#include "test_control_ethercat.h"
#include "test_control_motion.h"

using namespace aris::control;
/*
"                    <index_1600 type=\"PdoGroup\" default_child_type=\"Pdo\" index=\"0x1600\" is_tx=\"false\">"
"                        <control_word index=\"0x6040\" subindex=\"0x00\" datatype=\"uint16\"/>"
"                        <mode_of_operation index=\"0x6060\" subindex=\"0x00\" datatype=\"uint8\"/>"
"                        <target_pos index=\"0x607A\" subindex=\"0x00\" datatype=\"int32\"/>"
"                        <target_vel index=\"0x60FF\" subindex=\"0x00\" datatype=\"int32\"/>"
"                        <targer_tor index=\"0x6071\" subindex=\"0x00\" datatype=\"int16\"/>"
"                    </index_1600>"
"                    <index_1a00 type=\"PdoGroup\" default_child_type=\"Pdo\" index=\"0x1A00\" is_tx=\"true\">"
"                        <status_word index=\"0x6041\" subindex=\"0x00\" datatype=\"uint16\"/>"
"                        <mode_of_display index=\"0x6061\" subindex=\"0x00\" datatype=\"uint8\"/>"
"                        <pos_actual_value index=\"0x6064\" subindex=\"0x00\" datatype=\"int32\"/>"
"                        <vel_actual_value index=\"0x606c\" subindex=\"0x00\" datatype=\"int32\"/>"
"                        <cur_actual_value index=\"0x6078\" subindex=\"0x00\" datatype=\"int16\"/>"
"                    </index_1a00>"
*/
void test_elmo_enable()
{
	try
	{
		aris::control::Master m;
		m.registerChildType<MyMotion>();

		auto &st = m.slaveTypePool().add<SlaveType>("st", 0x00030924, 0x0000009a, 0x0000, 0x0300);
		auto &s1 = m.slavePool().add<MyMotion>("s1", st, 0, 0, 0, 0, 0, 0);


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
			static aris::core::MsgFix<8192> msg;
			static int count{ 0 };

			auto ret = s1.enable(8);

			msg.resize(1);
			sprintf(msg.data(), "count %d : pos %d", count, ret);
			msg.resize(std::strlen(msg.data()) + 1);
			

			if (++count % 1000 == 0)m.pipeOut().sendMsg(msg);

			m.dataLogger().lout() << "count " << count << " : ret " << ret <<"\n";
			m.dataLogger().send();
		});
		m.dataLogger().start();
		m.start();
		for (auto i{ 0 }; i < 20; ++i)
		{
			aris::core::Msg msg;
			while (!m.pipeOut().recvMsg(msg));
			std::cout << msg.data() << std::endl;
		}
		m.stop();
		m.dataLogger().stop();
		std::cout << "test pdo finished" << std::endl;
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
