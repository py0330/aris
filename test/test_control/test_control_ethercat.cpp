#include <iostream>
#include <aris.h>

#include "test_control_ethercat.h"

#ifdef UNIX
#include "rtdk.h"
#include "unistd.h"
#endif

const char xml_file[] =
"<?xml version=\"1.0\" encoding=\"UTF-8\" ?>"
"<Root>"
"    <Controller>"
"        <SlaveType type=\"slave_type_pool_element\">"
"            <ElmoSoloWhistle type=\"slave_type\" product_code=\"0x00030924\" vender_id=\"0x0000009a\" alias=\"0\" distributed_clock=\"0x0300\">"
"                <PDO type=\"pdo_group_pool_element\">"
"                    <index_1605 type=\"pdo_group\" default_child_type=\"pdo\" index=\"0x1605\" is_tx=\"false\">"
"                        <TargetPosition index=\"0x607A\" subindex=\"0x00\" datatype=\"int32\"/>"
"                        <TargetVelocity index=\"0x60FF\" subindex=\"0x00\" datatype=\"int32\"/>"
"                        <TargetTorque index=\"0x6071\" subindex=\"0x00\" datatype=\"int16\"/>"
"                        <MaxTorque index=\"0x6072\" subindex=\"0x00\" datatype=\"int16\"/>"
"                        <ControlWord index=\"0x6040\" subindex=\"0x00\" datatype=\"uint16\"/>"
"                        <ModeOfOperation index=\"0x6060\" subindex=\"0x00\" datatype=\"uint8\"/>"
"                    </index_1605>"
"                    <index_1a03 type=\"pdo_group\" default_child_type=\"pdo\" index=\"0x1A03\" is_tx=\"true\">"
"                        <PositionActualValue index=\"0x6064\" subindex=\"0x00\" datatype=\"int32\"/>"
"                        <DigitalInputs index=\"0x60fd\" subindex=\"0x00\" datatype=\"uint32\"/>"
"                        <VelocityActualValue index=\"0x606c\" subindex=\"0x00\" datatype=\"int32\"/>"
"                        <StatusWord index=\"0x6041\" subindex=\"0x00\" datatype=\"uint16\"/>"
"                    </index_1a03>"
"                    <index_1a1f type=\"pdo_group\" default_child_type=\"pdo\" index=\"0x1A1F\" is_tx=\"true\">"
"                        <CurrentActualValue index=\"0x6078\" subindex=\"0x00\" datatype=\"int16\"/>"
"                    </index_1a1f>"
"                    <index_1a13 type=\"pdo_group\" default_child_type=\"pdo\" index=\"0x1A13\" is_tx=\"true\">"
"                        <TorqueActualValue index=\"0x6077\" subindex=\"0x00\" datatype=\"int16\"/>"
"                    </index_1a13>"
"                    <index_1a0b type=\"pdo_group\" default_child_type=\"pdo\" index=\"0x1A0B\" is_tx=\"true\">"
"                        <ModeOfOperationDisplay index=\"0x6061\" subindex=\"0x00\" datatype=\"uint8\"/>"
"                    </index_1a0b>"
"                </PDO>"
"                <SDO type=\"sdo_pool_element\" default_child_type=\"sdo\">"
"                    <homeMode index=\"0x6098\" subindex=\"0\" datatype=\"int8\" config=\"-1\"/>"
"                    <homeAcc index=\"0x609A\" subindex=\"0\" datatype=\"uint32\" config=\"200000\"/>"
"                    <homeHighSpeed index=\"0x6099\" subindex=\"1\" datatype=\"uint32\" config=\"200000\"/>"
"                    <homeLowSpeed index=\"0x6099\" subindex=\"2\" datatype=\"uint32\" config=\"100000\"/>"
"                    <RatioOfPosDivVel_Numerator index=\"0x6096\" subindex=\"1\" datatype=\"uint32\" config=\"1\"/>"
"                    <RatioOfPosDivVel_Divisor index=\"0x6096\" subindex=\"2\" datatype=\"uint32\" config=\"1\"/>"
"                    <homeTorqueLimit index=\"0x2020\" subindex=\"1\" datatype=\"int32\" config=\"950\"/>"
"                    <p2pMaxSpeed index=\"0x607F\" subindex=\"0\" datatype=\"uint32\" config=\"2560\"/>"
"                    <p2pSpeed index=\"0x6081\" subindex=\"0\" datatype=\"uint32\" config=\"1792\"/>"
"                    <homeOffset index=\"0x607C\" subindex=\"0\" datatype=\"int32\" config=\"0\"/>"
"                </SDO>"
"            </ElmoSoloWhistle>"
"            <AtiForceSensor type=\"slave_type\" product_code=\"0x26483052\" vender_id=\"0x00000732\" alias=\"0\">"
"                <PDO type=\"pdo_group_pool_element\">"
"                    <index_1a00 type=\"pdo_group\" default_child_type=\"pdo\" index=\"0x1a00\" is_tx=\"true\">"
"                        <Fx index=\"0x6000\" subindex=\"0x01\" datatype=\"int32\"/>"
"                        <Fy index=\"0x6000\" subindex=\"0x02\" datatype=\"int32\"/>"
"                        <Fz index=\"0x6000\" subindex=\"0x03\" datatype=\"int32\"/>"
"                        <Mx index=\"0x6000\" subindex=\"0x04\" datatype=\"int32\"/>"
"                        <My index=\"0x6000\" subindex=\"0x05\" datatype=\"int32\"/>"
"                        <Mz index=\"0x6000\" subindex=\"0x06\" datatype=\"int32\"/>"
"                        <statusCode index=\"0x6010\" subindex=\"0x00\" datatype=\"int32\"/>"
"                        <sampleCount index=\"0x6020\" subindex=\"0x00\" datatype=\"int32\"/>"
"                    </index_1a00>"
"                    <index_1601 type=\"pdo_group\" default_child_type=\"pdo\" index=\"0x1601\" is_tx=\"false\">"
"                        <cw1 index=\"0x7010\" subindex=\"0x01\" datatype=\"int32\"/>"
"                        <cw2 index=\"0x7010\" subindex=\"0x02\" datatype=\"int32\"/>"
"                    </index_1601>"
"                </PDO>"
"                <SDO type=\"sdo_pool_element\" default_child_type=\"sdo\">"
"                    <ForceUnit index=\"0x2040\" subindex=\"0x31\" datatype=\"int32\" write=\"false\"/>"
"                    <TorqueUnit index=\"0x2040\" subindex=\"0x32\" datatype=\"int32\" write=\"false\"/>"
"                </SDO>"
"            </AtiForceSensor>"
"        </SlaveType>"
"        <Slave type=\"slave_pool_element\">"
"            <Motion1 type=\"slave\" slave_type=\"ElmoSoloWhistle\"  min_pos=\"0.676\" max_pos=\"1.091\" max_vel=\"0.2362\" home_pos=\"0.676\" input2count=\"22937600\"/>"
"            <Motion2 type=\"slave\" slave_type=\"ElmoSoloWhistle\"  min_pos=\"0.698\" max_pos=\"1.112\" max_vel=\"0.2362\" home_pos=\"0.698\" input2count=\"22937600\"/>"
"            <Motion3 type=\"slave\" slave_type=\"ElmoSoloWhistle\"  min_pos=\"0.698\" max_pos=\"1.112\" max_vel=\"0.2362\" home_pos=\"0.698\" input2count=\"22937600\"/>"
"            <Motion4 type=\"slave\" slave_type=\"ElmoSoloWhistle\"  min_pos=\"0.676\" max_pos=\"1.091\" max_vel=\"0.2362\" home_pos=\"0.676\" input2count=\"22937600\"/>"
"            <ForceSensor type=\"slave\" slave_type=\"AtiForceSensor\"/>"
"        </Slave>"
"    </Controller>"
"</Root>";

using namespace aris::control;

class TestMaster :public aris::control::Master
{
protected:
	virtual auto controlStrategy()->void override
	{
        auto& slave = dynamic_cast<aris::control::Slave &>(slavePool().at(0));

		static int count{ 0 };

        std::int32_t feedback_pos;
        slave.readPdoIndex(0x6064,0,feedback_pos);

#ifdef UNIX
        if (count++ % 100 == 0)rt_printf("%d:%d\n", count, feedback_pos);
#endif
	};
};

void test_control_ethercat()
{
	aris::core::XmlDocument xml_doc;
	xml_doc.Parse(xml_file);
	
	TestMaster master;

	master.loadXml(xml_doc);

    std::cout<<"2"<<std::endl;

	master.start();

	std::cout << "press any key to terminate test" << std::endl;
	std::cin.get();

	master.stop();

	std::cout << "test_control_ethercat finished" << std::endl;
}
