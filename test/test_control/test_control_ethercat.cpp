#include <iostream>
#include <cstdint>
#include <aris_control.h>
#include "test_control_ethercat.h"



const char xml_file[] =
"<?xml version=\"1.0\" encoding=\"UTF-8\" ?>"
"<controller>"
"	<data_logger type=\"DataLogger\"/>"
"	<slave_type_pool type=\"SlaveTypePoolObject\"/>"
"	<slave_pool type=\"SlavePoolObject\">"
"		<sla type=\"EthercatSlave\" phy_id=\"0\" product_code=\"0x00030924\" vendor_id=\"0x0000009a\" revision_num=\"0x000103F6\" dc_assign_activate=\"0x0300\" min_pos=\"0.676\" max_pos=\"1.091\" max_vel=\"0.2362\" home_pos=\"0.676\" input2count=\"22937600\">"
"			<pdo_group_pool type=\"PdoGroupPoolObject\">"
"				<index_1a00 type=\"Pdo\" default_child_type=\"PdoEntry\" index=\"0x1A00\" is_tx=\"true\">"
"					<pos_actual_value index=\"0x6064\" subindex=\"0x00\" size=\"4\"/>"
"				</index_1a00>"
"			</pdo_group_pool>"
"			<sdo_pool type=\"SdoPoolObject\" default_child_type=\"Sdo\">"
"				<home_mode index=\"0x6098\" subindex=\"0\" size=\"1\" config=\"17\" read=\"true\" write=\"true\"/>"
"			</sdo_pool>"
"		</sla>"
"	</slave_pool>"
"</controller>";

using namespace aris::control;

void test_scan()
{
	try
	{
		std::cout << "test scan" << std::endl;

		aris::control::EthercatMaster mst;
		mst.scan();
		mst.setControlStrategy([&]()
		{
			static int count{ 0 };
			if (++count % 1000 == 0)
			{
				mst.mout() << "count:" << std::dec << count << std::endl;
				
				for (auto &sla : mst.ecSlavePool())
				{
					for (auto &sm : sla.smPool())
					{
						if (sm.tx())
						{
							for (auto &pdo : sm)
							{
								for (auto &entry : pdo)
								{
									std::uint32_t data;
									sla.readPdo(entry.index(), entry.subindex(), &data, entry.size());
									mst.mout() << "  index:0x" << std::setfill('0') << std::setw(sizeof(std::int16_t) * 2) << std::hex << static_cast<std::uint32_t>(entry.index())
										<< "  subindex:0x" << std::setfill('0') << std::setw(sizeof(std::int8_t) * 2) << std::hex << static_cast<std::uint32_t>(entry.subindex())
										<< "  value:0x" << std::setfill('0') << std::setw(sizeof(std::uint32_t) * 2) << std::hex << data << std::endl;
								}
							}
						}

					}
				}
			}
		});


		mst.start();

		std::this_thread::sleep_for(std::chrono::seconds(10));

		mst.stop();

	}
	catch (std::exception &e)
	{
		std::cout << e.what() << std::endl;
	}
}
void test_pdo()
{
	try
	{
		std::cout << "test pdo" << std::endl;

		aris::control::EthercatMaster mst;

		std::string xml_str =
			"<io type=\"EthercatSlave\" phy_id=\"0\" product_code=\"10020000\""
			" vendor_id=\"4550262\" revision_num=\"0x03\" dc_assign_activate=\"0x0300\">"
			"	<sm_pool type=\"SyncManagerPoolObject\">"
			"		<sm type=\"SyncManager\" is_tx=\"false\"/>"
			"		<sm type=\"SyncManager\" is_tx=\"true\"/>"
			"		<sm type=\"SyncManager\" is_tx=\"false\">"
			"			<index_1a00 type=\"Pdo\" default_child_type=\"PdoEntry\" index=\"0x1a00\" is_tx=\"false\">"
			"				<word0 index=\"0x7000\" subindex=\"0x01\" size=\"2\"/>"
			"				<word1 index=\"0x7000\" subindex=\"0x02\" size=\"1\"/>"
			"				<word2 index=\"0x7000\" subindex=\"0x03\" size=\"1\"/>"
			"				<word3 index=\"0x7000\" subindex=\"0x04\" size=\"8\"/>"
			"				<word4 index=\"0x7000\" subindex=\"0x05\" size=\"8\"/>"
			"			</index_1a00>"
			"		</sm>"
			"		<sm type=\"SyncManager\" is_tx=\"true\">"
			"			<index_1600 type=\"Pdo\" default_child_type=\"PdoEntry\" index=\"0x1600\" is_tx=\"true\">"
			"				<word0 index=\"0x6000\" subindex=\"0x01\" size=\"2\"/>"
			"				<word1 index=\"0x6000\" subindex=\"0x02\" size=\"2\"/>"
			"				<word2 index=\"0x6000\" subindex=\"0x03\" size=\"1\"/>"
			"				<word3 index=\"0x6000\" subindex=\"0x04\" size=\"1\"/>"
			"				<word4 index=\"0x6000\" subindex=\"0x05\" size=\"16\"/>"
			"				<word5 index=\"0x6000\" subindex=\"0x06\" size=\"6\"/>"
			"				<word6 index=\"0x6000\" subindex=\"0x07\" size=\"10\"/>"
			"				<word7 index=\"0x6000\" subindex=\"0x08\" size=\"16\"/>"
			"				<word8 index=\"0x6000\" subindex=\"0x09\" size=\"2\"/>"
			"				<word9 index=\"0x6000\" subindex=\"0x10\" size=\"14\"/>"
			"				<word10 index=\"0x6000\" subindex=\"0x11\" size=\"2\"/>"
			"				<word11 index=\"0x6000\" subindex=\"0x12\" size=\"10\"/>"
			"				<word12 index=\"0x6000\" subindex=\"0x13\" size=\"2\"/>"
			"				<word13 index=\"0x6000\" subindex=\"0x14\" size=\"6\"/>"
			"				<word14 index=\"0x6000\" subindex=\"0x15\" size=\"8\"/>"
			"				<word15 index=\"0x6000\" subindex=\"0x16\" size=\"8\"/>"
			"				<word16 index=\"0x6000\" subindex=\"0x17\" size=\"8\"/>"
			"				<word17 index=\"0x6000\" subindex=\"0x18\" size=\"8\"/>"
			"			</index_1600>"
			"		</sm>"
			"	</sm_pool>"
			"	<sdo_pool type=\"SdoPoolObject\" default_child_type=\"Sdo\">"
			"	</sdo_pool>"
			"</io>";

		mst.slavePool().add<aris::control::EthercatSlave>().loadXmlStr(xml_str);


		mst.setControlStrategy([&]()
		{
			static int count{ 0 };
			if (++count % 1000 == 0)
			{
				mst.mout() << "count:" << std::dec << count << std::endl;
				for (auto &sla : mst.ecSlavePool())
				{
					for (auto &sm : sla.smPool())
					{
						if (sm.tx())
						{
							for (auto &pdo : sm)
							{
								for (auto &entry : pdo)
								{
									std::uint32_t data;
									sla.readPdo(entry.index(), entry.subindex(), &data, entry.size());
									mst.mout() << "  index:0x" << std::setfill('0') << std::setw(sizeof(std::int16_t) * 2) << std::hex << static_cast<std::uint32_t>(entry.index())
										<< "  subindex:0x" << std::setfill('0') << std::setw(sizeof(std::int8_t) * 2) << std::hex << static_cast<std::uint32_t>(entry.subindex())
										<< "  value:0x" << std::setfill('0') << std::setw(sizeof(std::uint32_t) * 2) << std::hex << data << std::endl;
								}
							}
						}

					}
				}
			}
		});


		mst.start();

		std::this_thread::sleep_for(std::chrono::seconds(10));

		mst.stop();

	}
	catch (std::exception &e)
	{
		std::cout << e.what() << std::endl;
	}
}
void test_pdo_xml()
{
	try
	{
		//std::cout << "test pdo xml" << std::endl;
		//
		//aris::control::EthercatMaster m;

		//auto &s1 = m.slavePool().add<EthercatSlave>();
		//s1.loadXmlStr(
		//	"<sla type=\"EthercatSlave\" phy_id=\"0\" product_code=\"0x0\" vendor_id=\"0x000002E1\" revision_num=\"0x29001\" dc_assign_activate=\"0x0300\" min_pos=\"0.676\" max_pos=\"1.091\" max_vel=\"0.2362\" home_pos=\"0.676\" input2count=\"22937600\">"
		//	"	<pdo_group_pool type=\"PdoGroupPoolObject\">"
		//	"		<index_1a00 type=\"Pdo\" default_child_type=\"PdoEntry\" index=\"0x1A00\" is_tx=\"true\">"
		//	"			<pos_actual_value index=\"0x6064\" subindex=\"0x00\" size=\"4\"/>"
		//	"		</index_1a00>"
		//	"	</pdo_group_pool>"
		//	"</sla>");

		//m.setControlStrategy([&]()
		//{
		//	static int count{ 0 };

		//	std::int32_t value{ 0 };
		//	s1.readPdo(0x6064, 0x00, value);

		//	if (++count % 1000 == 0)
		//	{
		//		m.mout() << "count " << count << " : pos " << value;
		//	}
		//});
		//m.start();
		//std::this_thread::sleep_for(std::chrono::seconds(10));
		//m.stop();
		//std::cout << "test pdo xml finished" << std::endl;
	}
	catch (std::exception &e)
	{
		std::cout << e.what() << std::endl;
	}
}
void test_sdo_xml()
{
	try
	{
		std::cout << "test sdo xml" << std::endl;

		aris::control::EthercatMaster m;

		auto &s1 = m.slavePool().add<EthercatSlave>();
		s1.loadXmlStr(
			"<sla type=\"EthercatSlave\" phy_id=\"0\" product_code=\"0x00030924\" vendor_id=\"0x0000009a\" revision_num=\"0x000103F6\" dc_assign_activate=\"0x0300\" min_pos=\"0.676\" max_pos=\"1.091\" max_vel=\"0.2362\" home_pos=\"0.676\" input2count=\"22937600\">"
			"	<sdo_pool type=\"SdoPoolObject\" default_child_type=\"Sdo\">"
			"		<home_mode index=\"0x6098\" subindex=\"0\" size=\"1\" config=\"17\" read=\"true\" write=\"true\"/>"
			"	</sdo_pool>"
			"</sla>");

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
		std::cout << "test sdo xml finished" << std::endl;
	}
	catch (std::exception &e)
	{
		std::cout << e.what() << std::endl;
	}
}
void test_sdo_code() 
{
	try
	{
		



	}
	catch (std::exception &e)
	{
		std::cout << e.what() << std::endl;
	}
	

}

void test_control_ethercat()
{
	test_pdo();
	//test_scan();
	//test_pdo_xml();
	//test_sdo_code();
	//test_sdo_xml();
}
