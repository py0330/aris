#include <iostream>
#include <aris.h>

#include "test_control_ethercat.h"

const char xml_file[] =
"<?xml version=\"1.0\" encoding=\"UTF-8\" ?>"
"<root>"
"    <controller>"
"        <data_logger type=\"DataLogger\"/>"
"        <slave_type_pool type=\"SlaveTypePoolObject\">"
"            <elmo type=\"SlaveType\" product_code=\"0x00030924\" vender_id=\"0x0000009a\" alias=\"0\" distributed_clock=\"0x0300\">"
"            </elmo>"
"        </slave_type_pool>"
"        <slave_pool type=\"SlavePoolObject\">"
"            <sla type=\"EthercatSlave\" slave_type=\"elmo\" min_pos=\"0.676\" max_pos=\"1.091\" max_vel=\"0.2362\" home_pos=\"0.676\" input2count=\"22937600\">"
"                <pdo_group_pool type=\"PdoGroupPoolObject\">"
"                    <index_1a00 type=\"PdoGroup\" default_child_type=\"Pdo\" index=\"0x1A00\" is_tx=\"true\">"
"                        <pos_actual_value index=\"0x6064\" subindex=\"0x00\" size=\"4\"/>"
"                    </index_1a00>"
"                </pdo_group_pool>"
"                <sdo_pool type=\"SdoPoolObject\" default_child_type=\"Sdo\">"
"                    <home_mode index=\"0x6098\" subindex=\"0\" size=\"1\" config=\"17\" read=\"true\" write=\"true\"/>"
"                </sdo_pool>"
"            </sla>"
"        </slave_pool>"
"    </controller>"
"</root>";

using namespace aris::control;

void test_pdo() 
{
	try 
	{
		aris::control::EthercatMaster m;

		auto &st = m.slaveTypePool().add<SlaveType>("st", 0x00030924, 0x0000009a, 0x0000, 0x0300);
		auto &s1 = m.slavePool().add<EthercatSlave>("s1", st);

		auto &pdo_group = s1.pdoGroupPool().add<PdoGroup>("index_1a00", 0x1A00, true);
		pdo_group.add<Pdo>("index_6064", 0x6064, 0x00, 4);

		m.setControlStrategy([&]()
		{
			static int count{ 0 };

			std::int32_t value{ 0 };
			s1.readPdo(0x6064, 0x00, value);

			if (++count % 1000 == 0)
			{
				m.mout() << "count " << count << " : pos " << value << '\0';
				m.mout().update();
				m.sendOut();
			}
		});
		m.start();
		for (auto i{ 0 }; i < 20; ++i)
		{
			aris::core::Msg msg;
			while (!m.recvOut(msg));
			std::cout << msg.data() << std::endl;
		}
		m.stop();
		std::cout << "test pdo finished" << std::endl;
	}
	catch (std::exception &e)
	{
		std::cout << e.what() << std::endl;
	}
}
void test_sdo() 
{
	try
	{
		std::cout << "test sdo" << std::endl;
		aris::control::EthercatMaster m;

		auto &st = m.slaveTypePool().add<SlaveType>("st", 0x00030924, 0x0000009a, 0x0000, 0x0300);
		auto &s1 = m.slavePool().add<EthercatSlave>("s1", st);

		auto &home_mode = s1.sdoPool().add<Sdo>("index_6098", 0x6098, 0x00, sizeof(std::int16_t), Sdo::READ | Sdo::WRITE | Sdo::CONFIG, 17);

		m.start();

		// test read sdo //
		std::int8_t mode = 0;
		s1.readSdo(0x6098, 0x00, mode);//home_mode.read(mode);
		std::cout << "home mode before write and start:" << static_cast<int>(mode) << std::endl;
		// test write sdo //
		s1.writeSdo(0x6098, 0x00, static_cast<std::int8_t>(16));//home_mode.write(static_cast<std::int8_t>(16));
		std::this_thread::sleep_for(std::chrono::seconds(3));
		s1.readSdo(0x6098, 0x00, mode);//home_mode.read(mode);
		std::cout << "home mode after write before start:" << static_cast<int>(mode) << std::endl;
		m.stop();
		
		
		// test config sdo //
		m.start();
		std::this_thread::sleep_for(std::chrono::seconds(3));
		s1.readSdo(0x6098, 0x00, mode);//home_mode.read(mode);
		std::cout << "home mode after start and config:" << static_cast<int>(mode) << std::endl;
		// test write sdo when running //
		s1.writeSdo(0x6098, 0x00, static_cast<std::int8_t>(17));//home_mode.write(static_cast<std::int8_t>(16));
		std::this_thread::sleep_for(std::chrono::seconds(3));
		s1.readSdo(0x6098, 0x00, mode);//home_mode.read(mode);
		std::cout << "home mode after write before start:" << static_cast<int>(mode) << std::endl;
		m.stop();
		std::cout << "test sdo finished" << std::endl;
	}
	catch (std::exception &e)
	{
		std::cout << e.what() << std::endl;
	}
	

}
void test_data_logger()
{
	try 
	{
		aris::core::XmlDocument xml_doc;
		xml_doc.Parse(xml_file);

		aris::control::EthercatMaster m;
		m.loadXml(xml_doc);

		m.setControlStrategy([&]()
		{
			static aris::core::MsgFix<8192> msg;
			static int count{ 0 };

			std::int32_t value{ 0 };
			m.ecSlavePool().front().readPdo(0x6064, 0x00, value);

			if (++count % 1000 == 0)
			{
				m.mout() << "count " << count << " : pos " << value << '\0';
				m.mout().update();
				m.sendOut();
			}

			m.dataLogger().lout() << count << " pos:" << value << "\n";
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
		std::cout << "test data logger finished" << std::endl;
	}
	catch (std::exception &e)
	{
		std::cout << e.what() << std::endl;
	}
}

void test_control_ethercat()
{
	test_pdo();
	test_sdo();
	test_data_logger();
}
