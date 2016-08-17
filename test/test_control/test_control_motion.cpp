#include <iostream>
#include <aris.h>

#ifdef UNIX
#include "rtdk.h"
#include "unistd.h"
#endif

#ifdef WIN32
#define rt_printf printf
#endif

#include "test_control_ethercat.h"
#include "test_control_motion.h"

using namespace aris::control;

const char xml_file[] =
"<?xml version=\"1.0\" encoding=\"utf-8\" ?>"
"<root>"
"    <widget_root>"
"        <command_parser type=\"CommandParser\">"
"            <command_pool type=\"CommandPoolObject\" default_child_type=\"Command\">"
"                <exit/>"
"                <en default_child_type=\"Param\" default=\"all\">"
"                    <all abbreviation=\"a\"/>"
"                    <physical_id abbreviation=\"p\" default=\"0\"/>"
"                </en>"
"                <ds default_child_type=\"Param\" default=\"all\">"
"                    <all abbreviation=\"a\"/>"
"                    <physical_id abbreviation=\"p\" default=\"0\"/>"
"                </ds>"
"                <hm default_child_type=\"Param\" default=\"all\">"
"                    <all abbreviation=\"a\"/>"
"                    <physical_id abbreviation=\"p\" default=\"0\"/>"
"                </hm>"
"                <test default_child_type=\"Param\" default=\"all\">"
"                    <all abbreviation=\"a\"/>"
"                    <motion_id abbreviation=\"m\" default=\"0\"/>"
"                    <physical_id abbreviation=\"p\" default=\"0\"/>"
"                </test>"
"            </command_pool>"
"        </command_parser>"
"        <msg_pipe type=\"Pipe\"/>"
"    </widget_root>"
"    <controller>"
"        <slave_type_pool type=\"SlaveTypePoolObject\">"
"            <elmo type=\"SlaveType\" product_code=\"0x00030924\" vender_id=\"0x0000009a\" alias=\"0\" distributed_clock=\"0x0300\">"
"                <pdo_group_pool type=\"PdoGroupPoolObject\">"
"                    <index_1605 type=\"PdoGroup\" default_child_type=\"Pdo\" index=\"0x1605\" is_tx=\"false\">"
"                        <target_pos index=\"0x607A\" subindex=\"0x00\" datatype=\"int32\"/>"
"                        <target_vel index=\"0x60FF\" subindex=\"0x00\" datatype=\"int32\"/>"
"                        <targer_tor index=\"0x6071\" subindex=\"0x00\" datatype=\"int16\"/>"
"                        <max_torque index=\"0x6072\" subindex=\"0x00\" datatype=\"int16\"/>"
"                        <control_word index=\"0x6040\" subindex=\"0x00\" datatype=\"uint16\"/>"
"                        <mode_of_operation index=\"0x6060\" subindex=\"0x00\" datatype=\"uint8\"/>"
"                    </index_1605>"
"                    <index_1617 type=\"PdoGroup\" default_child_type=\"Pdo\" index=\"0x1617\" is_tx=\"false\">"
"                        <VelocityOffset index=\"0x60B1\" subindex=\"0x00\" datatype=\"int32\"/>"
"                    </index_1617>"
"                    <index_1618 type=\"PdoGroup\" default_child_type=\"Pdo\" index=\"0x1618\" is_tx=\"false\">"
"                        <TorqueOffset index=\"0x60B2\" subindex=\"0x00\" datatype=\"int16\"/>"
"                    </index_1618>"
"                    <index_1a03 type=\"PdoGroup\" default_child_type=\"Pdo\" index=\"0x1A03\" is_tx=\"true\">"
"                        <pos_actual_value index=\"0x6064\" subindex=\"0x00\" datatype=\"int32\"/>"
"                        <vel_actual_value index=\"0x606c\" subindex=\"0x00\" datatype=\"int32\"/>"
"                        <digital_inputs index=\"0x60fd\" subindex=\"0x00\" datatype=\"uint32\"/>"
"                        <status_word index=\"0x6041\" subindex=\"0x00\" datatype=\"uint16\"/>"
"                    </index_1a03>"
"                    <index_1a1f type=\"PdoGroup\" default_child_type=\"Pdo\" index=\"0x1A1F\" is_tx=\"true\">"
"                        <cur_actual_value index=\"0x6078\" subindex=\"0x00\" datatype=\"int16\"/>"
"                    </index_1a1f>"
"                    <index_1a13 type=\"PdoGroup\" default_child_type=\"Pdo\" index=\"0x1A13\" is_tx=\"true\">"
"                        <tor_actual_value index=\"0x6077\" subindex=\"0x00\" datatype=\"int16\"/>"
"                    </index_1a13>"
"                    <index_1a0b type=\"PdoGroup\" default_child_type=\"Pdo\" index=\"0x1A0B\" is_tx=\"true\">"
"                        <mode_of_operation_display index=\"0x6061\" subindex=\"0x00\" datatype=\"uint8\"/>"
"                    </index_1a0b>"
"                </pdo_group_pool>"
"                <sdo_pool type=\"SdoPoolObject\" default_child_type=\"Sdo\">"
"                    <home_mode index=\"0x6098\" subindex=\"0\" datatype=\"int8\" config=\"-1\"/>"
"                    <home_acc index=\"0x609A\" subindex=\"0\" datatype=\"uint32\" config=\"200000\"/>"
"                    <home_high_speed index=\"0x6099\" subindex=\"1\" datatype=\"uint32\" config=\"200000\"/>"
"                    <home_low_speed index=\"0x6099\" subindex=\"2\" datatype=\"uint32\" config=\"100000\"/>"
"                    <home_offset index=\"0x607C\" subindex=\"0\" datatype=\"int32\" config=\"0\"/>"
"                </sdo_pool>"
"            </elmo>"
"        </slave_type_pool>"
"        <slave_pool type=\"SlavePoolObject\">"
"            <m1 type=\"Motion\" slave_type=\"elmo\" min_pos=\"0.676\" max_pos=\"1.091\" max_vel=\"0.2362\" home_pos=\"0.676\" input2count=\"22937600\"/>"
"            <m2 type=\"Motion\" slave_type=\"elmo\" min_pos=\"0.676\" max_pos=\"1.091\" max_vel=\"0.2362\" home_pos=\"0.676\" input2count=\"22937600\"/>"
"        </slave_pool>"
"    </controller>"
"</root>"
;


enum { MOTION_NUM = 100 };
struct BasicFunctionParam
{
	std::uint8_t cmd_type;
	bool active_motor[MOTION_NUM];

	BasicFunctionParam() {
		cmd_type = aris::control::Motion::IDLE;
		std::fill(active_motor, active_motor + MOTION_NUM, true);
	}
};

static aris::control::Controller controller;
static aris::core::Root widget_root;
static aris::core::CommandParser *parser;
static aris::core::Pipe *msg_pipe;

char cmd_char[aris::core::MsgRT::RT_MSG_SIZE];

static bool is_running = true;

static std::string command_in("idle");//command input from terminal
static std::int32_t cmd_count{ 0 };
static bool cmd_success{ false };

BasicFunctionParam decode(const std::string input)
{
	std::string cmd;
	std::map<std::string, std::string> params;
	try
	{
		parser->parse(input, cmd, params);

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


	std::fill_n(bfParam.active_motor, MOTION_NUM, false);
	for (auto &i : params) {
        if (i.first != "physical_id")
            std::cout << "the first param must be 'p', it means the physic id." << std::endl;
		else {
			if (stoi(i.second)>MOTION_NUM - 1 || stoi(i.second)<0) {
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
	if (msg_pipe->recvMsg(aris::core::MsgRT::instance()[0]))
	{
		aris::core::MsgRT::instance()[0].paste(cmd_char);
		param = reinterpret_cast<BasicFunctionParam *>(cmd_char);
		cmd_count = 0;
		cmd_success = false;
	}

	for (std::size_t i = 0; i<MOTION_NUM && param->active_motor[i]; i++) {
		auto &txmotiondata = static_cast<aris::control::TxMotionData&>(controller.slavePool().at(i).txData());
		auto &rxmotiondata = static_cast<aris::control::RxMotionData&>(controller.slavePool().at(i).rxData());
		if (param->cmd_type != aris::control::Motion::Cmd::RUN)
		{
			if (rxmotiondata.ret == 0 && cmd_count != 0 && !cmd_success) {
				rt_printf("command finished, cmd_count: %d\n", cmd_count);
				cmd_success = true;
			}

			if (cmd_count % 1000 == 0 && !cmd_success)	rt_printf("executing command: %d\n", cmd_count);
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
			if (cmd_count == 0)	begin = rxmotiondata.feedback_pos;
			txmotiondata.target_pos = 0.020*std::sin(cmd_count / 10000.0 * 2 * PI) + begin;

			if (cmd_count % 5000 == 0)	rt_printf("executing command: %d\n", cmd_count);
			break;
		default:
			break;
		}

	}
	cmd_count++;
}

void test_control_motion()
{
	std::cout << std::endl << "-----------------test motion---------------------" << std::endl;
	
	aris::core::XmlDocument xml_doc;
    xml_doc.Parse(xml_file);

	controller.loadXml(xml_doc);
	widget_root.registerChildType<aris::core::Param>();
	widget_root.registerChildType<aris::core::UniqueParam>();
	widget_root.registerChildType<aris::core::GroupParam>();
	widget_root.registerChildType<aris::core::Command>();
	widget_root.registerChildType<aris::core::ObjectPool<aris::core::Command> >();
	widget_root.registerChildType<aris::core::CommandParser>();
	widget_root.registerChildType<aris::core::Pipe>();
	widget_root.loadXml(*xml_doc.RootElement()->FirstChildElement("widget_root"));
	parser = static_cast<aris::core::CommandParser*>(&*widget_root.findByName("command_parser"));
	msg_pipe = static_cast<aris::core::Pipe*>(&*widget_root.findByName("msg_pipe"));

	controller.setControlStrategy(tg);
	controller.start();

	while (std::getline(std::cin, command_in) && is_running)
	{
		BasicFunctionParam param = decode(command_in);

		if (param.cmd_type < 100)
		{
			aris::core::Msg cmd_msg;
			cmd_msg.copyStruct(param);
			if (cmd_msg.size() != sizeof(BasicFunctionParam))throw std::runtime_error("invalid msg length of parse function");
			cmd_msg.setMsgID(0);
			msg_pipe->sendMsg(cmd_msg);
		}
		else if (param.cmd_type == 110) {
			controller.stop();
			is_running = false;
			break;
		}
	}

	std::cout << "-----------------test motion finished------------" << std::endl << std::endl;

	return;
}
