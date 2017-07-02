#include <iostream>
#include <aris.h>

#include "test_control_ethercat.h"

#ifdef UNIX
#include "rtdk.h"
#include "unistd.h"
#endif

const char xml_file[] =
"<?xml version=\"1.0\" encoding=\"UTF-8\" ?>"
"<root>"
"    <controller>"
"        <data_logger type=\"DataLogger\"/>"
"        <slave_type_pool type=\"SlaveTypePoolObject\">"
"            <elmo type=\"SlaveType\" product_code=\"0x00010001\" vender_id=\"0x00007595\" alias=\"0\" distributed_clock=\"0x0300\">"
"                <pdo_group_pool type=\"PdoGroupPoolObject\">"
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
"                <sdo_pool type=\"SdoPoolObject\" default_child_type=\"Sdo\">"
"                    <home_mode index=\"0x6098\" subindex=\"0\" datatype=\"int8\" config=\"17\" read=\"true\" write=\"true\"/>"
"                </sdo_pool>"
"            </elmo>"
"        </slave_type_pool>"
"        <slave_pool type=\"SlavePoolObject\">"
"            <sla type=\"TestSlave\" slave_type=\"elmo\" min_pos=\"0.676\" max_pos=\"1.091\" max_vel=\"0.2362\" home_pos=\"0.676\" input2count=\"22937600\"/>"
"        </slave_pool>"
"    </controller>"
"</root>";

using namespace aris::control;

struct TxTestData :public Slave::TxType
{
	std::uint16_t control_word{ 0 };
	std::uint8_t mode_of_operation;
	std::int32_t target_pos;
	std::int32_t target_vel;
	std::int16_t target_cur;
};
struct RxTestData :public Slave::RxType
{
	std::uint16_t status_word;
	std::uint8_t mode_of_display;
	std::int32_t actual_pos;
	std::int32_t actual_vel;
	std::int16_t actual_cur;
};
class TestSlave :public SlaveTemplate<TxTestData, RxTestData>
{
public:
	static auto Type()->const std::string &{ static const std::string type("TestSlave"); return std::ref(type); }
	auto virtual type() const->const std::string&{ return Type(); }
		TestSlave(Object &father, const aris::core::XmlElement &xml_ele) :SlaveTemplate(father, xml_ele) {}

protected:
	auto virtual readUpdate()->void override
	{
		readPdoIndex(0x6041, 0, rxData().status_word);
		readPdoIndex(0x6061, 0, rxData().mode_of_display);
		readPdoIndex(0x6064, 0, rxData().actual_pos);
		readPdoIndex(0x606C, 0, rxData().actual_vel);
		readPdoIndex(0x6078, 0, rxData().actual_cur);
	};
	auto virtual writeUpdate()->void override
	{
		writePdoIndex(0x6040, 0, txData().control_word);
		writePdoIndex(0x6060, 0, txData().mode_of_operation);
		writePdoIndex(0x607A, 0, txData().target_pos);
		writePdoIndex(0x60FF, 0, txData().target_vel);
		writePdoIndex(0x6071, 0, txData().target_cur);
	}
};

class TestMaster :public aris::control::Master
{
protected:
	auto virtual controlStrategy()->void override
	{
		static int count{ 0 };

		auto& slave = dynamic_cast<aris::control::Slave &>(slavePool().at(0));

#ifdef UNIX
		if (count++ % 1000 == 0)rt_printf("%d:%d %d %d\n", count, static_cast<TestSlave&>(slave).rxData().actual_pos, static_cast<TestSlave&>(slave).rxData().actual_vel, static_cast<TestSlave&>(slave).rxData().actual_cur);
#endif


	};
};

void test_control_ethercat()
{
	std::cout << std::endl << "-----------------test ethercat---------------------" << std::endl;

	aris::core::XmlDocument xml_doc;
	xml_doc.Parse(xml_file);

	TestMaster master;

	master.registerChildType<TestSlave>();
	master.loadXml(xml_doc);


	master.start();

	// test sdo read and write
	std::this_thread::sleep_for(std::chrono::seconds(1));
	std::int8_t mode = 0;
	master.slavePool().front().sdoPool().front().read(mode);
	std::cout << "home mode:" << static_cast<int>(mode) << std::endl;
	master.slavePool().front().sdoPool().front().write(static_cast<std::int8_t>(18));
	master.slavePool().front().sdoPool().front().read(mode);
	std::cout << "home mode:" << static_cast<int>(mode) << std::endl;

	std::cin.get();
	std::cin.get();



	std::cout << "-----------------test ethercat finished------------" << std::endl << std::endl;
}
