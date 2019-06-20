#include <iostream>
#include <cstdint>
#include <iomanip>
#include <algorithm>
#include <thread>
#include <aris/control/control.hpp>
#include "test_control_ethercat.h"

using namespace aris::control;

namespace aris::control
{
	void read_bit(char *data, int bit_size, const char *pd, int offset, int bit_position);
	void write_bit(const char *data, int bit_size, char *pd, int offset, int bit_position);

	void read_bit2(char *data, int bit_size, const char *pd, int offset, int bit_position);
	void write_bit2(const char *data, int bit_size, char *pd, int offset, int bit_position);
}
void test_bit()
{
	union T
	{
		char data[4];
		std::uint32_t integer;
	};

	const char pd[8] = { (char)0b00110101, (char)0b101110111, (char)0b10010011, (char)0b11000110, (char)0b01010101, (char)0b10101010, (char)0b11110000, (char)0b00001111 };
	char r_pd[8];

	T t1;
	t1.data[0] = (char)0b1110'0011;
	t1.data[1] = (char)0b0010'1010;
	t1.data[2] = (char)0b1111'1111;
	t1.data[3] = (char)0b1111'1111;
	char data1[4] = { (char)0xff,(char)0xff,(char)0xff,(char)0xff };
	aris::control::read_bit(data1, 17, pd, 2, 7);
	if (*reinterpret_cast<std::uint32_t*>(data1) != t1.integer)std::cout << "read bit failed" << std::endl;
	std::fill_n(r_pd, 8, std::uint8_t(0x00));
	aris::control::write_bit(data1, 17, r_pd, 2, 7);
	const char r1[8]{ (char)0x00, (char)0x00, (char)0b0000000'1, (char)0b11000110, (char)0b01010101, (char)0x00, (char)0x00, (char)0x00 };
	if (*reinterpret_cast<std::uint64_t*>(r_pd) != *reinterpret_cast<const std::uint64_t*>(r1))std::cout << "write bit failed" << std::endl;

	T t2;
	t2.data[0] = (char)0b1110'0011;
	t2.data[1] = (char)0b0010'1010;
	t2.data[2] = (char)0b1000'0000;
	t2.data[3] = (char)0b0000'0000;
	char data2[4] = { (char)0x00,(char)0x00,(char)0x00,(char)0x00 };
	aris::control::read_bit(data2, 17, pd, 2, 7);
	if (*reinterpret_cast<std::uint32_t*>(data2) != t2.integer)std::cout << "read bit failed" << std::endl;
	std::fill_n(r_pd, 8, std::uint8_t(0xff));
	aris::control::write_bit(data2, 17, r_pd, 2, 7);
	const char r2[8]{ (char)0xff, (char)0xff, (char)0b1111111'1, (char)0b11000110, (char)0b01010101, (char)0xff, (char)0xff, (char)0xff };
	if (*reinterpret_cast<std::uint64_t*>(r_pd) != *reinterpret_cast<const std::uint64_t*>(r2))std::cout << "write bit failed" << std::endl;

	T t3;
	t3.data[0] = (char)0b1110'0011;
	t3.data[1] = (char)0b0010'1010;
	t3.data[2] = (char)0b1101'0111;
	t3.data[3] = (char)0b1111'1111;
	char data3[4] = { (char)0xff,(char)0xff,(char)0xff,(char)0xff };
	aris::control::read_bit(data3, 22, pd, 2, 7);
	if (*reinterpret_cast<std::uint32_t*>(data3) != t3.integer)std::cout << "read bit failed" << std::endl;
	std::fill_n(r_pd, 8, std::uint8_t(0x00));
	aris::control::write_bit(data3, 22, r_pd, 2, 7);
	const char r3[8]{ (char)0x00, (char)0x00, (char)0b0000000'1, (char)0b11000110, (char)0b01010101, (char)0b10101'000, (char)0x00, (char)0x00 };
	if (*reinterpret_cast<std::uint64_t*>(r_pd) != *reinterpret_cast<const std::uint64_t*>(r3))std::cout << "write bit failed" << std::endl;

	T t4;
	t4.data[0] = (char)0b1110'0011;
	t4.data[1] = (char)0b0010'1010;
	t4.data[2] = (char)0b1101'0100;
	t4.data[3] = (char)0b0000'0000;
	char data4[4] = { (char)0x00,(char)0x00,(char)0x00,(char)0x00 };
	aris::control::read_bit(data4, 22, pd, 2, 7);
	if (*reinterpret_cast<std::uint32_t*>(data4) != t4.integer)std::cout << "read bit failed" << std::endl;
	std::fill_n(r_pd, 8, std::uint8_t(0xff));
	aris::control::write_bit(data4, 22, r_pd, 2, 7);
	const char r4[8]{ (char)0xff, (char)0xff, (char)0b1111111'1, (char)0b11000110, (char)0b01010101, (char)0b10101'111, (char)0xff, (char)0xff };
	if (*reinterpret_cast<std::uint64_t*>(r_pd) != *reinterpret_cast<const std::uint64_t*>(r4))std::cout << "write bit failed" << std::endl;

	T t5;
	t5.data[0] = (char)0b0100'1111;
	t5.data[1] = (char)0b0001'1001;
	t5.data[2] = (char)0b0111'1111;
	t5.data[3] = (char)0b1111'1111;
	char data5[4] = { (char)0xff,(char)0xff,(char)0xff,(char)0xff };
	aris::control::read_bit(data5, 17, pd, 2, 2);
	if (*reinterpret_cast<std::uint32_t*>(data5) != t5.integer)std::cout << "read bit failed" << std::endl;
	std::fill_n(r_pd, 8, std::uint8_t(0x00));
	aris::control::write_bit(data5, 17, r_pd, 2, 2);
	const char r5[8]{ (char)0x00, (char)0x00, (char)0b00'010011, (char)0b11000110, (char)0b010'00000, (char)0x00, (char)0x00, (char)0x00 };
	if (*reinterpret_cast<std::uint64_t*>(r_pd) != *reinterpret_cast<const std::uint64_t*>(r5))std::cout << "write bit failed" << std::endl;

	T t6;
	t6.data[0] = (char)0b0100'1111;
	t6.data[1] = (char)0b0001'1001;
	t6.data[2] = (char)0b0000'0000;
	t6.data[3] = (char)0b0000'0000;
	char data6[4] = { (char)0x00,(char)0x00,(char)0x00,(char)0x00 };
	aris::control::read_bit(data6, 17, pd, 2, 2);
	if (*reinterpret_cast<std::uint32_t*>(data6) != t6.integer)std::cout << "read bit failed" << std::endl;
	std::fill_n(r_pd, 8, std::uint8_t(0xff));
	aris::control::write_bit(data6, 17, r_pd, 2, 2);
	const char r6[8]{ (char)0xff, (char)0xff, (char)0b11'010011, (char)0b11000110, (char)0b010'11111, (char)0xff, (char)0xff, (char)0xff };
	if (*reinterpret_cast<std::uint64_t*>(r_pd) != *reinterpret_cast<const std::uint64_t*>(r6))std::cout << "write bit failed" << std::endl;

	T t7;
	t7.data[0] = (char)0b0100'1111;
	t7.data[1] = (char)0b0001'1001;
	t7.data[2] = (char)0b0101'0111;
	t7.data[3] = (char)0b1111'1111;
	char data7[4] = { (char)0xff,(char)0xff,(char)0xff,(char)0xff };
	aris::control::read_bit(data7, 22, pd, 2, 2);
	if (*reinterpret_cast<std::uint32_t*>(data7) != t7.integer)std::cout << "read bit failed" << std::endl;
	std::fill_n(r_pd, 8, std::uint8_t(0x00));
	aris::control::write_bit(data7, 22, r_pd, 2, 2);
	const char r7[8]{ (char)0x00, (char)0x00, (char)0b00'010011, (char)0b11000110, (char)0b01'010101, (char)0x00, (char)0x00, (char)0x00 };
	if (*reinterpret_cast<std::uint64_t*>(r_pd) != *reinterpret_cast<const std::uint64_t*>(r7))std::cout << "write bit failed" << std::endl;

	T t8;
	t8.data[0] = (char)0b0100'1111;
	t8.data[1] = (char)0b0001'1001;
	t8.data[2] = (char)0b0101'0100;
	t8.data[3] = (char)0b0000'0000;
	char data8[4] = { (char)0x00,(char)0x00,(char)0x00,(char)0x00 };
	aris::control::read_bit(data8, 22, pd, 2, 2);
	if (*reinterpret_cast<std::uint32_t*>(data8) != t8.integer)std::cout << "read bit failed" << std::endl;
	std::fill_n(r_pd, 8, std::uint8_t(0xff));
	aris::control::write_bit(data8, 22, r_pd, 2, 2);
	const char r8[8]{ (char)0xff, (char)0xff, (char)0b11'010011, (char)0b11000110, (char)0b01'010101, (char)0xff, (char)0xff, (char)0xff };
	if (*reinterpret_cast<std::uint64_t*>(r_pd) != *reinterpret_cast<const std::uint64_t*>(r8))std::cout << "write bit failed" << std::endl;
}
void test_scan()
{
	try
	{
		std::cout << "test scan" << std::endl;

		aris::control::EthercatMaster mst;
		mst.scan();

		aris::control::Master::RtStasticsData stastics;
		mst.setControlStrategy([&]()
		{
			static int count{ 0 };

			if (count == 0)mst.resetRtStasticData(&stastics);

			if (++count % 1000 == 0)
			{
				mst.mout() << "count:" << std::dec << count << std::endl;
	
				for (auto &sla : mst.slavePool())
				{
					for (auto &sm : sla.smPool())
					{
						if (sm.tx())
						{
							for (auto &pdo : sm)
							{
								for (auto &entry : pdo)
								{
									if (entry.index())
									{
										std::uint32_t data;
										sla.readPdo(entry.index(), entry.subindex(), &data, entry.bitSize());
										mst.mout() << "  index:0x" << std::setfill('0') << std::setw(sizeof(std::int16_t) * 2) << std::hex << static_cast<std::uint32_t>(entry.index())
											<< "  subindex:0x" << std::setfill('0') << std::setw(sizeof(std::int8_t) * 2) << std::hex << static_cast<std::uint32_t>(entry.subindex())
											<< "  value:0x" << std::setfill('0') << std::setw(sizeof(std::uint32_t) * 2) << std::hex << data << std::endl;
									}
								}
							}
						}
					}
				}


				static std::int64_t all_max{ 0 };
				all_max = std::max(stastics.max_time_consumed, all_max);

				mst.mout() << "------------------------------------------------------------------------------------------------\n"
					<< "total count|   avg cost|  max count|   max cost|  min count|   min cost|   overruns|    all max|\n" << std::dec
					<< std::setw(11) << stastics.total_count << "|"
					<< std::setw(11) << stastics.avg_time_consumed << "|"
					<< std::setw(11) << stastics.max_time_occur_count << "|"
					<< std::setw(11) << stastics.max_time_consumed << "|"
					<< std::setw(11) << stastics.min_time_occur_count << "|"
					<< std::setw(11) << stastics.min_time_consumed << "|"
					<< std::setw(11) << stastics.overrun_count << "|" 
					<< std::setw(11) << all_max << "|" << std::endl;

				mst.resetRtStasticData(&stastics);

			}
		});


		mst.start();

		std::this_thread::sleep_for(std::chrono::seconds(100));

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
				for (auto &ec_sla : mst.slavePool())
				{
					auto &sla = dynamic_cast<aris::control::EthercatSlave&>(ec_sla);
					
					for (auto &sm : sla.smPool())
					{
						if (sm.tx())
						{
							for (auto &pdo : sm)
							{
								for (auto &entry : pdo)
								{
									if (entry.index())
									{
										std::uint32_t data;
										sla.readPdo(entry.index(), entry.subindex(), &data, entry.bitSize());
										mst.mout() << "  index:0x" << std::setfill('0') << std::setw(sizeof(std::int16_t) * 2) << std::hex << static_cast<std::uint32_t>(entry.index())
											<< "  subindex:0x" << std::setfill('0') << std::setw(sizeof(std::int8_t) * 2) << std::hex << static_cast<std::uint32_t>(entry.subindex())
											<< "  value:0x" << std::setfill('0') << std::setw(sizeof(std::uint32_t) * 2) << std::hex << data << std::endl;
									}
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
	test_bit();
	test_scan();
	//test_pdo();
	//test_pdo_xml();
	//test_sdo_code();
	//test_sdo_xml();
}
