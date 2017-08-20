#include <iostream>
#include <aris.h>

#include "test_control_master_slave.h"

static const char xml_file[] =
"<?xml version=\"1.0\" encoding=\"UTF-8\" ?>"
"<root>"
"    <controller>"
"        <data_logger type=\"DataLogger\"/>"
"        <slave_type_pool type=\"SlaveTypePoolElement\">"
"            <elmo type=\"SlaveType\" product_code=\"0x00010001\" vendor_id_=\"0x00007595\" alias=\"0\" distributed_clock=\"0x0300\">"
"            </elmo>"
"        </slave_type_pool>"
"        <slave_pool type=\"SlavePoolElement\">"
"            <sla type=\"EthercatSlave\" slave_type=\"elmo\" min_pos=\"0.676\" max_pos=\"1.091\" max_vel=\"0.2362\" home_pos=\"0.676\" input2count=\"22937600\">"
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

void test_construct()
{
	aris::control::Master m;
	
	m.setControlStrategy([]() 
	{
		static int count{0};
		std::cout << "count:" << ++count << std::endl;
	});
	m.start();
	std::this_thread::sleep_for(std::chrono::seconds(2));
	m.stop();


	std::cout << m.xmlString() << std::endl;
}

void test_control_master_slave()
{
	test_construct();
}
