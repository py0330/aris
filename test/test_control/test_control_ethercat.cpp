#include <iostream>
#include <aris.h>

#include "test_control_ethercat.h"

const char xml_file[] =
"<?xml version=\"1.0\" encoding=\"UTF-8\" ?>"
"<root>"
"    <controller>"
"        <data_logger type=\"DataLogger\"/>"
"        <slave_type_pool type=\"SlaveTypePoolElement\">"
"            <elmo type=\"SlaveType\" product_code=\"0x00010001\" vender_id=\"0x00007595\" alias=\"0\" distributed_clock=\"0x0300\">"
"            </elmo>"
"        </slave_type_pool>"
"        <slave_pool type=\"SlavePoolElement\">"
"            <sla type=\"Slave\" slave_type=\"elmo\" min_pos=\"0.676\" max_pos=\"1.091\" max_vel=\"0.2362\" home_pos=\"0.676\" input2count=\"22937600\">"
"                <pdo_group_pool type=\"PdoGroupPoolElement\">"
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
"                </pdo_group_pool>"
"                <sdo_pool type=\"SdoPoolElement\" default_child_type=\"Sdo\">"
"                    <home_mode index=\"0x6098\" subindex=\"0\" datatype=\"int8\" config=\"17\" read=\"true\" write=\"true\"/>"
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
		aris::control::Master m;

		auto &st = m.slaveTypePool().add<SlaveType>("st", 0x00030924, 0x0000009a, 0x0000, 0x0300);
		auto &s1 = m.slavePool().add<Slave>("s1", st);

		auto &pdo_group = s1.pdoGroupPool().add<PdoGroup>("index_1a00", 0x1A00, true);
		pdo_group.add<Pdo>("index_6064", DO::INT32, 0x6064, 0x00);

		m.setControlStrategy([&]()
		{
			static int count{ 0 };

			std::int32_t value{ 0 };
			s1.readPdoIndex(0x6064, 0x00, value);

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
		aris::control::Master m;

		auto &st = m.slaveTypePool().add<SlaveType>("st", 0x00030924, 0x0000009a, 0x0000, 0x0300);
		auto &s1 = m.slavePool().add<Slave>("s1", st);

		auto &home_mode = s1.sdoPool().add<Sdo>("index_6098", DO::INT16, 0x6098, 0x00, Sdo::READ | Sdo::WRITE | Sdo::CONFIG, 17);

		// test read sdo //
		std::int8_t mode = 0;
		home_mode.read(mode);
		std::cout << "home mode before write and start:" << static_cast<int>(mode) << std::endl;
		// test write sdo //
		home_mode.write(static_cast<std::int8_t>(16));
		std::this_thread::sleep_for(std::chrono::seconds(3));
		home_mode.read(mode);
		std::cout << "home mode after write before start:" << static_cast<int>(mode) << std::endl;
		// test config sdo //
		m.start();
		std::this_thread::sleep_for(std::chrono::seconds(3));
		home_mode.read(mode);
		std::cout << "home mode after start and config:" << static_cast<int>(mode) << std::endl;
		// test write sdo when running //
		home_mode.write(static_cast<std::int8_t>(16));
		std::this_thread::sleep_for(std::chrono::seconds(3));
		home_mode.read(mode);
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
	aris::core::XmlDocument xml_doc;
	xml_doc.Parse(xml_file);

	aris::control::Master m;
	m.loadXml(xml_doc);

	m.setControlStrategy([&]()
	{
		static aris::core::MsgFix<8192> msg;
		static int count{ 0 };

		std::int32_t value{ 0 };
		m.slavePool().front().readPdoIndex(0x6064, 0x00, value);

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

void test_control_ethercat()
{
	test_pdo();
	test_sdo();
	test_data_logger();
}
