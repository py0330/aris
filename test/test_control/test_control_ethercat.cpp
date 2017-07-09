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
	};
};

void test_pdo() 
{
	aris::control::Master m;

	auto &st = m.slaveTypePool().add<SlaveType>("st",1,1,1,1);
	auto &s1 = m.slavePool().add<Slave>("s1", st);

	auto &pdo_group = s1.pdoGroupPool().add<PdoGroup>("1600", 1600, true);
	pdo_group.add<Pdo>("index_1a00", DO::INT32, 0x120a, 0x12);
	s1.sdoPool().add<Sdo>("index_0601", DO::INT16, 0x120b, 0x12, Sdo::READ | Sdo::WRITE);
	s1.sdoPool().add<Sdo>("index_0602", DO::INT32, 0x120c, 0x13, Sdo::READ);
	s1.sdoPool().add<Sdo>("index_0603", DO::UINT16, 0x120d, 0x14, Sdo::READ | Sdo::WRITE | Sdo::CONFIG, 12);
	s1.sdoPool().add<Sdo>("index_0604", DO::UINT8, 0x120e, 0x15, Sdo::WRITE | Sdo::CONFIG);
	s1.sdoPool().add<Sdo>("index_0605", DO::INT8, 0x120f, 0x16, Sdo::READ | Sdo::WRITE, 12);
	std::cout << m.xmlString() <<"end" << std::endl;


	//m.loadString(xml_file);
	//m.xmlString();
	//std::string s;
	//m.saveString(s);
	//std::cout << m.xmlString() <<"end"<<std::endl;
}
void test_sdo() {}
void test_data_logger(){}

void test_control_ethercat()
{
	test_pdo();
	
	
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



	//std::cout << "-----------------test ethercat finished------------" << std::endl << std::endl;
}
