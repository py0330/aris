#include <iostream>
#include <aris.h>

#ifdef UNIX
#include "rtdk.h"
#include "unistd.h"
#endif

#ifdef WIN32
#define rt_printf printf
#endif


#include "test_control_motion.h"


using namespace aris::control;

const char xml_file[] =
"<?xml version=\"1.0\" encoding=\"UTF-8\" ?>"
"<Root>"
"	<Server ip = \"127.0.0.1\" port = \"5866\">"
"		<Commands type=\"command_pool_object\" default_child_type=\"command\">"
"			<start/>"
"			<stop/>"
"			<exit/>"
"			<en default_child_type=\"param\" default = \"all\">"
"				<all abbreviation = \"a\" />"
"				<first abbreviation = \"f\" />"
"				<second abbreviation = \"s\" />"
"				<motion_id abbreviation = \"m\" default = \"0\" />"
"				<physical_id abbreviation = \"p\" default = \"0\" />"
"				<leg abbreviation = \"l\" default = \"0\" />"
"			</en>"
"			<ds default_child_type=\"param\" default = \"all\">"
"				<all abbreviation = \"a\" />"
"				<first abbreviation = \"f\" />"
"				<second abbreviation = \"s\" />"
"				<motion_id abbreviation = \"m\" default = \"0\" />"
"				<physical_id abbreviation = \"p\" default = \"0\" />"
"				<leg abbreviation = \"l\" default = \"0\" />"
"			</ds>"
"			<hm default_child_type=\"param\" default = \"all\" >"
"				<all abbreviation = \"a\" />"
"				<first abbreviation = \"f\" />"
"				<second abbreviation = \"s\" />"
"				<motion_id abbreviation = \"m\" default = \"0\" />"
"				<physical_id abbreviation = \"p\" default = \"0\" />"
"				<leg abbreviation = \"l\" default = \"0\" />"
"			</hm>"
"			<test default_child_type=\"param\" default = \"all\" >"
"				<all abbreviation = \"a\" />"
"				<motion_id abbreviation = \"m\" default = \"0\" />"
"				<physical_id abbreviation = \"p\" default = \"0\" />"
"			</test>"
"			<rc default = \"rc_param\">"
"				<rc_param type = \"group\" default_child_type=\"param\">"
"					<leg_param type = \"unique\" default_child_type=\"param\" default = \"all\">"
"						<all abbreviation = \"a\" />"
"						<first abbreviation = \"f\" />"
"						<second abbreviation = \"s\" />"
"						<leg abbreviation = \"l\" default = \"0\" />"
"					</leg_param>"
"					<t1 abbreviation = \"t\" default = \"3000\" />"
"					<t2 default = \"3000\" />"
"					<margin_offset abbreviation = \"m\" default = \"0.01\" />"
"				</rc_param>"
"			</rc>"
"			<wk default_child_type=\"param\" default = \"wk_param\">"
"				<wk_param type = \"group\" default_child_type=\"param\">"
"					<totalCount abbreviation = \"t\" default = \"3000\" />"
"					<n abbreviation = \"n\" default = \"1\" />"
"					<distance abbreviation = \"d\" default = \"0.5\" />"
"					<height abbreviation = \"h\" default = \"0.05\" />"
"					<alpha abbreviation = \"a\" default = \"0\" />"
"					<beta abbreviation = \"b\" default = \"0\" />"
"				</wk_param>"
"			</wk>"
"		</Commands>"
"	</Server>"
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
"                    <index_1617 type=\"pdo_group\" default_child_type=\"pdo\" index=\"0x1617\" is_tx=\"false\">"
"                        <VelocityOffset index=\"0x60B1\" subindex=\"0x00\" datatype=\"int32\"/>"
"                    </index_1617>"
"                    <index_1618 type=\"pdo_group\" default_child_type=\"pdo\" index=\"0x1618\" is_tx=\"false\">"
"                        <TorqueOffset index=\"0x60B2\" subindex=\"0x00\" datatype=\"int16\"/>"
"                    </index_1618>"
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
"                    <homeMode index=\"0x6098\" subindex=\"0\" datatype=\"int8\" config=\"35\"/>"
"                    <homeAcc index=\"0x609A\" subindex=\"0\" datatype=\"uint32\" config=\"200000\"/>"
"                    <homeHighSpeed index=\"0x6099\" subindex=\"1\" datatype=\"uint32\" config=\"200000\"/>"
"                    <homeLowSpeed index=\"0x6099\" subindex=\"2\" datatype=\"uint32\" config=\"100000\"/>"
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
"            <Motion1 type=\"motion\" slave_type=\"ElmoSoloWhistle\"  min_pos=\"0.676\" max_pos=\"1.091\" max_vel=\"0.2362\" home_pos=\"0.676\" input2count=\"22937600\"/>"
"            <Motion2 type=\"motion\" slave_type=\"ElmoSoloWhistle\"  min_pos=\"0.698\" max_pos=\"1.112\" max_vel=\"0.2362\" home_pos=\"0.698\" input2count=\"22937600\"/>"
"            <Motion3 type=\"motion\" slave_type=\"ElmoSoloWhistle\"  min_pos=\"0.698\" max_pos=\"1.112\" max_vel=\"0.2362\" home_pos=\"0.698\" input2count=\"22937600\"/>"
"            <Motion4 type=\"motion\" slave_type=\"ElmoSoloWhistle\"  min_pos=\"0.676\" max_pos=\"1.091\" max_vel=\"0.2362\" home_pos=\"0.676\" input2count=\"22937600\"/>"
"            <ForceSensor type=\"slave\" slave_type=\"AtiForceSensor\"/>"
"        </Slave>"
"    </Controller>"
"</Root>";

#define SlaveNumber 5
enum { MAX_MOTOR_NUM = 100 };
//for enable, disable, and home
struct BasicFunctionParam
{
	std::uint8_t cmd_type;
	bool active_motor[MAX_MOTOR_NUM];

	BasicFunctionParam() {
		cmd_type = aris::control::Motion::IDLE;
		std::fill(active_motor, active_motor + MAX_MOTOR_NUM, true);
	}
};




static bool is_running = true;

aris::control::Pipe<aris::core::Msg> msg_pipe;
char cmd_char[aris::core::MsgRT::RT_MSG_LENGTH];

static std::string command_in("idle");//command input from terminal
static std::int32_t cmd_count{ 0 };
static bool cmd_success{ false };


aris::control::Controller controller;
aris::core::CommandParser parser;

BasicFunctionParam decode(const std::string input)
{
	std::string cmd;
	std::map<std::string, std::string> params;
	try
	{
		parser.parse(input, cmd, params);

		std::cout << "\n" << cmd << std::endl;
		int paramPrintLength;
		if (params.empty())
		{
			paramPrintLength = 2;
		}
		else
		{
			paramPrintLength = std::max_element(params.begin(), params.end(), [](decltype(*params.begin()) a, decltype(*params.begin()) b)
			{
				return a.first.length() < b.first.length();
			})->first.length() + 2;
		}

		for (auto &i : params)
		{
			std::cout << std::string(paramPrintLength - i.first.length(), ' ') << i.first << " : " << i.second << std::endl;
		}

		std::cout << std::endl;
	}
	catch (std::exception &e)
	{
		std::cout << e.what() << std::endl << std::endl;
	}

	BasicFunctionParam bfParam;
	if (cmd == "ds")
	{
		bfParam.cmd_type = aris::control::Motion::Cmd::DISABLE;
	}
	else if (cmd == "en")
	{
		bfParam.cmd_type = aris::control::Motion::Cmd::ENABLE;
	}
	else if (cmd == "hm")
	{
		bfParam.cmd_type = aris::control::Motion::Cmd::HOME;
	}
	else if (cmd == "test")
	{
		bfParam.cmd_type = aris::control::Motion::Cmd::RUN;
	}
	else if (cmd == "exit")
	{
		bfParam.cmd_type = 110;
	}
	else
		bfParam.cmd_type = 120;


	std::fill_n(bfParam.active_motor, SlaveNumber, false);
	for (auto &i : params) {
        if (i.first != "physical_id")
            std::cout << "the first param must be 'p', it means the physic id." << std::endl;
		else {
			if (stoi(i.second)>SlaveNumber - 1 || stoi(i.second)<0) {
				throw std::runtime_error("the second param is invalid");
			}
			else
				bfParam.active_motor[stoi(i.second)] = true;
		}
	}
	return bfParam;
}
void tg()
{
	BasicFunctionParam *param;
	if (msg_pipe.recvInRT(aris::core::MsgRT::instance[0]) > 0)
	{
		aris::core::MsgRT::instance[0].paste(cmd_char);
		param = reinterpret_cast<BasicFunctionParam *>(cmd_char);
		cmd_count = 0;
		cmd_success = false;
	}

	for (std::size_t i = 0; i<SlaveNumber && param->active_motor[i]; i++) {
		auto &txmotiondata = static_cast<aris::control::TxMotionData&>(controller.slavePool().at(i).txData());
		auto &rxmotiondata = static_cast<aris::control::RxMotionData&>(controller.slavePool().at(i).rxData());
		if (param->cmd_type != aris::control::Motion::Cmd::RUN)
		{
			if (rxmotiondata.ret == 0 && cmd_count != 0 && !cmd_success) {
				rt_printf("command finished, cmd_count: %d\n", cmd_count);
				cmd_success = true;
			}

			if (cmd_count % 1000 == 0 && !cmd_success)
				rt_printf("executing command: %d\n", cmd_count);
		}
		switch (param->cmd_type)
		{
		case aris::control::Motion::Cmd::ENABLE:
			txmotiondata.cmd = aris::control::Motion::Cmd::ENABLE;
			if (cmd_success) {
				txmotiondata.cmd = aris::control::Motion::Cmd::RUN;
				txmotiondata.target_pos = rxmotiondata.feedback_pos;
			}
			break;
		case aris::control::Motion::Cmd::DISABLE:
			txmotiondata.cmd = aris::control::Motion::Cmd::DISABLE;
			break;
		case aris::control::Motion::Cmd::HOME:
			txmotiondata.cmd = aris::control::Motion::Cmd::HOME;
			if (cmd_success) {
				txmotiondata.cmd = aris::control::Motion::Cmd::RUN;
				txmotiondata.target_pos = rxmotiondata.feedback_pos;
			}
			break;
		case aris::control::Motion::Cmd::RUN:
			txmotiondata.cmd = aris::control::Motion::Cmd::RUN;
			static double begin;
			if (cmd_count == 0)
				begin = rxmotiondata.feedback_pos;
			txmotiondata.target_pos = 0.020*std::sin(cmd_count / 10000.0 * 2 * PI) + begin;
			//txmotiondata.target_pos=0.002*cmd_count/1000.0+begin;
			if (cmd_count % 5000 == 0)
				rt_printf("executing command: %d\n", cmd_count);
			break;
		default:
			break;
		}

	}
	cmd_count++;
}

void test_control_motion()
{
	aris::core::XmlDocument xml_doc;
    //xml_doc.Parse(xml_file);
    xml_doc.LoadFile("/usr/aris/robot/resource/robot_motion.xml");


	controller.loadXml(xml_doc);
	parser.loadXml(xml_doc);


	controller.setControlStrategy(tg);
	controller.start();

	while (std::getline(std::cin, command_in) && is_running)
	{
		BasicFunctionParam param = decode(command_in);

		if (param.cmd_type<100) {
			aris::core::Msg cmd_msg;
			cmd_msg.copyStruct(param);
			if (cmd_msg.size() != sizeof(BasicFunctionParam))throw std::runtime_error("invalid msg length of parse function");
			cmd_msg.setMsgID(0);
			msg_pipe.sendToRT(cmd_msg);
		}
		if (param.cmd_type == 110) {
			controller.stop();
			is_running = false;
			return;
		}
	}

	controller.stop();

	return;
}
