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
"            <elmo type=\"SlaveType\" product_code=\"0x00030924\" vender_id=\"0x0000009a\" alias=\"0\" distributed_clock=\"0x0300\">"
"                <pdo_group_pool type=\"PdoGroupPoolObject\">"
"                    <index_1a03 type=\"PdoGroup\" default_child_type=\"Pdo\" index=\"0x1A03\" is_tx=\"true\">"
"                        <pos_actual_value index=\"0x6064\" subindex=\"0x00\" datatype=\"int32\"/>"
"                        <vel_actual_value index=\"0x606c\" subindex=\"0x00\" datatype=\"int32\"/>"
"                        <digital_inputs index=\"0x60fd\" subindex=\"0x00\" datatype=\"uint32\"/>"
"                        <status_word index=\"0x6041\" subindex=\"0x00\" datatype=\"uint16\"/>"
"                    </index_1a03>"
"                </pdo_group_pool>"
"                <sdo_pool type=\"SdoPoolObject\" default_child_type=\"Sdo\">"
"                    <home_mode index=\"0x6098\" subindex=\"0\" datatype=\"int8\" config=\"-1\" read=\"true\" write=\"true\"/>"
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
    double target_pos{ 0 };
};
struct RxTestData :public Slave::RxType
{
    double feedback_pos{ 0 };
};
class TestSlave :public SlaveTemplate<TxTestData, RxTestData>
{
public:
    static auto Type()->const std::string &{ static const std::string type("TestSlave"); return std::ref(type); }
    virtual auto type() const->const std::string&{ return Type(); }
    TestSlave(Object &father, const aris::core::XmlElement &xml_ele):SlaveTemplate(father, xml_ele){}

protected:
    virtual auto readUpdate()->void override
    {
        std::int32_t feedback_pos;
        readPdoIndex(0x6064,0,feedback_pos);
        rxData().feedback_pos = feedback_pos;
    };
    virtual auto writeUpdate()->void override{}
    virtual auto logData(const Slave::TxType &tx_data, const Slave::RxType &rx_data, std::fstream &file)->void override
    {
		file << static_cast<const TxTestData&>(tx_data).target_pos << " " << static_cast<const RxTestData&>(rx_data).feedback_pos;
    }
};

class TestMaster :public aris::control::Master
{
protected:
	virtual auto controlStrategy()->void override
	{
        static int count{ 0 };

        auto& slave = dynamic_cast<aris::control::Slave &>(slavePool().at(0));

#ifdef UNIX
        if (count++ % 1000 == 0)rt_printf("%d:%f\n", count, static_cast<TestSlave&>(slave).rxData().feedback_pos);
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
	master.slavePool().front().sdoPool().front().write(static_cast<std::int8_t>(35));
	master.slavePool().front().sdoPool().front().read(mode);
	std::cout << "home mode:" << static_cast<int>(mode) << std::endl;


	std::cout << "press any key to start log" << std::endl;
	std::cin.get();
	std::cin.get();
    master.dataLogger().prepair("customized_data_name.txt");
    master.dataLogger().start();
    
	std::cout << "press any key to stop log" << std::endl;
	std::cin.get();
	std::cin.get();
    master.dataLogger().stop();

	std::cout << "press any key to start log" << std::endl;
	std::cin.get();
	std::cin.get();
    master.dataLogger().prepair();
	master.dataLogger().start();

	std::cout << "press any key to stop log" << std::endl;
	std::cin.get();
	std::cin.get();
	master.dataLogger().stop();
    
	std::cout << "press any key to stop master" << std::endl;
	std::cin.get();
	std::cin.get();
	master.stop();
    
	

	std::cout << "-----------------test ethercat finished------------" << std::endl << std::endl;
}
