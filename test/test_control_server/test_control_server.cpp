#include <iostream>
#include <condition_variable>
#include <future>
#include <aris.h>

#include "test_control_server.h"

const char xml_data[] =
"<?xml version=\"1.0\" encoding=\"utf-8\" ?>"
"<root>"
"    <widget_root type=\"WidgetRoot\">"
"        <message_pipe type=\"Pipe\" pool_size=\"16384\"/>"
"        <command_pipe type=\"Pipe\" pool_size=\"16384\"/>"
"        <command_socket type=\"Socket\" port=\"5866\"/>"
"        <command_parser type=\"CommandParser\">"
"            <command_pool type=\"CommandPoolObject\" default_child_type=\"Command\">"
"                <start/>"
"                <stop/>"
"                <exit/>"
"                <en default_child_type=\"Param\" default=\"all\">"
"                    <all abbreviation=\"a\"/>"
"                    <motion_id abbreviation=\"m\" default=\"0\"/>"
"                    <physical_id abbreviation=\"p\" default=\"0\"/>"
"                </en>"
"                <ds default_child_type=\"Param\" default=\"all\">"
"                    <all abbreviation=\"a\"/>"
"                    <motion_id abbreviation=\"m\" default=\"0\"/>"
"                    <physical_id abbreviation=\"p\" default=\"0\"/>"
"                </ds>"
"                <hm default_child_type=\"Param\" default=\"all\">"
"                    <all abbreviation=\"a\"/>"
"                    <motion_id abbreviation=\"m\" default=\"0\"/>"
"                    <physical_id abbreviation=\"p\" default=\"0\"/>"
"                </hm>"
"                <rc default=\"rc_param\">"
"                    <rc_param type=\"GroupParam\" default_child_type=\"Param\">"
"                        <t1 abbreviation=\"t\" default=\"3000\"/>"
"                        <t2 default=\"3000\"/>"
"                        <mag abbreviation=\"m\" default=\"1.0\"/>"
"                    </rc_param>"
"                </rc>"
"            </command_pool>"
"        </command_parser>"
"    </widget_root>"
"    <controller type=\"EthercatController\">"
"        <slave_type_pool type=\"SlaveTypePoolObject\">"
"            <elmo type=\"SlaveType\" product_code=\"0x00010001\" vendor_id=\"0x00007595\" alias=\"0\" distributed_clock=\"0x0300\">"
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
"                    <home_mode index=\"0x6098\" subindex=\"0\" datatype=\"int8\" config=\"35\"/>"
"                    <home_acc index=\"0x609A\" subindex=\"0\" datatype=\"uint32\" config=\"200000\"/>"
"                    <home_high_speed index=\"0x6099\" subindex=\"1\" datatype=\"uint32\" config=\"200000\"/>"
"                    <home_low_speed index=\"0x6099\" subindex=\"2\" datatype=\"uint32\" config=\"100000\"/>"
"                    <home_offset index=\"0x607C\" subindex=\"0\" datatype=\"int32\" config=\"0\"/>"
"                </sdo_pool>"
"            </elmo>"
"        </slave_type_pool>"
"        <slave_pool type=\"SlavePoolObject\">"
"            <m1 type=\"Motion\" slave_type=\"elmo\" min_pos=\"-10.0\" max_pos=\"10.0\" max_vel=\"10.0\" home_pos=\"0\" pos_factor=\"62914560\"/>"
"            <m2 type=\"Motion\" slave_type=\"elmo\" min_pos=\"-10.0\" max_pos=\"10.0\" max_vel=\"10.0\" home_pos=\"0\" pos_factor=\"22937600\"/>"
"            <m3 type=\"Motion\" slave_type=\"elmo\" min_pos=\"-10.0\" max_pos=\"10.0\" max_vel=\"10.0\" home_pos=\"0\" pos_factor=\"22937600\"/>"
"        </slave_pool>"
"    </controller>"
"    <model type=\"Model\">"
"        <environment type=\"Environment\" gravity=\"{0,-9.8,0,0,0,0}\"/>"
"        <variable_pool type=\"VariablePoolObject\" default_child_type=\"Matrix\">"
"            <PI type=\"MatrixVariable\">3.14159265358979</PI>"
"            <Mot_friction type=\"MatrixVariable\">{20, 30, 560}</Mot_friction>"
"        </variable_pool>"
"        <part_pool type=\"PartPoolObject\" default_child_type=\"Part\">"
"            <ground active=\"true\" inertia=\"{1,0,0,0,1,1,1,0,0,0}\" pe=\"{0,0,0,0,0,0}\" vel=\"{0,0,0,0,0,0}\" acc=\"{0,0,0,0,0,0}\" graphic_file_path=\"\">"
"                <marker_pool type=\"MarkerPoolElement\" default_child_type=\"Marker\">"
"                    <r1j pe=\"{ 0,0,0,0,0,0 }\"/>"
"                </marker_pool>"
"            </ground>"
"            <part1 active=\"true\" inertia=\"{1,0,0,0,1,1,1,0,0,0}\" pe=\"{0,0,0,0,0,0}\" vel=\"{0,0,0,0,0,0}\" acc=\"{0,0,0,0,0,0}\" graphic_file_path=\"C:\\aris\\robot\\resource\\graphic_file\\part1.x_t\">"
"                <marker_pool type=\"MarkerPoolObject\" default_child_type=\"Marker\">"
"                    <r1i pe=\"{ 0,0,0,0,0,0 }\"/>"
"                    <r2j pe=\"{ 1,0,0,0,0,0 }\"/>"
"                </marker_pool>"
"            </part1>"
"            <part2 active=\"true\" inertia=\"{1,0,0,0,1,1,1,0,0,0}\" pe=\"{1,0,0,PI/2,0,0}\" vel=\"{0,0,0,0,0,0}\" acc=\"{0,0,0,0,0,0}\" graphic_file_path=\"C:\\aris\\robot\\resource\\graphic_file\\part2.x_t\">"
"                <marker_pool type=\"MarkerPoolObject\" default_child_type=\"Marker\">"
"                    <r2i pe=\"{ 0,0,0,0,0,0 }\"/>"
"                    <r3j pe=\"{ 1,0,0,0,0,0 }\"/>"
"                </marker_pool>"
"            </part2>"
"            <part3 active=\"true\" inertia=\"{1,0,0,0,1,1,1,0,0,0}\" pe=\"{1,1,0,0,0,0}\" vel=\"{0,0,0,0,0,0}\" acc=\"{0,0,0,0,0,0}\" graphic_file_path=\"C:\\aris\\robot\\resource\\graphic_file\\part3.x_t\">"
"                <marker_pool type=\"MarkerPoolObject\" default_child_type=\"Marker\">"
"                    <r3i pe=\"{ 0,0,0,0,0,0 }\"/>"
"                </marker_pool>"
"            </part3>"
"        </part_pool>"
"        <joint_pool type=\"JointPoolObject\">"
"            <r1 active=\"true\" type=\"RevoluteJoint\" prt_m=\"part1\" prt_n=\"ground\" mak_i=\"r1i\" mak_j=\"r1j\"/>"
"            <r2 active=\"true\" type=\"RevoluteJoint\" prt_m=\"part2\" prt_n=\"part1\" mak_i=\"r2i\" mak_j=\"r2j\"/>"
"            <r3 active=\"true\" type=\"RevoluteJoint\" prt_m=\"part3\" prt_n=\"part2\" mak_i=\"r3i\" mak_j=\"r3j\"/>"
"        </joint_pool>"
"        <motion_pool type=\"MotionPoolObject\" default_child_type=\"Motion\">"
"            <m1 active=\"true\" prt_m=\"part1\" prt_n=\"ground\" mak_i=\"r1i\" mak_j=\"r1j\" frc_coe=\"Mot_friction\" component=\"5\"/>"
"            <m2 active=\"true\" prt_m=\"part2\" prt_n=\"part1\" mak_i=\"r2i\" mak_j=\"r2j\" frc_coe=\"Mot_friction\" component=\"5\"/>"
"            <m3 active=\"true\" prt_m=\"part3\" prt_n=\"part2\" mak_i=\"r3i\" mak_j=\"r3j\" frc_coe=\"Mot_friction\" component=\"5\"/>"
"        </motion_pool>"
"        <general_motion_pool type=\"GeneralMotionPoolObject\" default_child_type=\"GeneralMotion\"/>"
"    </model>"
"    <sensor_root type=\"SensorRoot\">"
"        <sensor_pool type=\"SensorPoolObject\"/>"
"    </sensor_root>"
"</root>";

struct RcParam
{
	int t1;
	int t2;
	double mag;
};
auto rc_parse_func(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg_out)->void
{
	RcParam param;
	param.t1 = std::atoi(params.at("t1").c_str());
	param.mag = std::atof(params.at("mag").c_str());
	msg_out.header().reserved1_ |= aris::server::ControlServer::USING_TARGET_POS;
	msg_out.copyStruct(param);
};
auto rc_plan_func(const aris::dynamic::PlanParam &param)->int
{
	auto p = reinterpret_cast<RcParam*>(param.param_);

	static double begin_pos;

	if (param.count_ == 1)
	{
		auto &cm = aris::server::ControlServer::instance().controller().motionPool().at(0);
		begin_pos = dynamic_cast<aris::control::EthercatMotion&>(cm).actualPos();
	}

	param.model_->motionPool().at(0).setMp(begin_pos + p->mag * std::sin(2 * PI * param.count_ / p->t1));

	return p->t1 - param.count_;
};

void test_xml()
{
	try
	{
		auto&cs = aris::server::ControlServer::instance();

		cs.loadXmlStr(xml_data);
		cs.addCmd("en", aris::server::default_parse, aris::server::default_enable_plan);
		cs.addCmd("ds", aris::server::default_parse, aris::server::default_disable_plan);
		cs.addCmd("rc", rc_parse_func, rc_plan_func);

		// 接收并打印信息 //
		auto t = std::thread([&]()
		{
			for (;;)
			{
				aris::core::Msg msg;
				cs.controller().recvOut(msg);
				if (!msg.empty())std::cout << msg.data() << std::endl;

				std::this_thread::sleep_for(std::chrono::milliseconds(10));
			}
		});

		// 接收命令 //
		for (std::string command_in; std::getline(std::cin, command_in);)
		{
			try
			{
				cs.executeCmd(command_in);
				std::cout << "cmd finished" << std::endl;
			}
			catch (std::exception &e)
			{
				std::cout << e.what() << std::endl;
			}
		}
	}
	catch (std::exception &e)
	{
		std::cout << e.what() << std::endl;
	}
}
void test_construct()
{
	auto&cs = aris::server::ControlServer::instance();

	cs.resetController(new aris::control::EthercatController);
	cs.resetModel(new aris::dynamic::Model);
	cs.resetSensorRoot(new aris::sensor::SensorRoot);
	cs.resetWidgetRoot(new aris::server::WidgetRoot);

	cs.widgetRoot().cmdParser().commandPool().add<aris::core::Command>(aris::server::default_enable_command());
	cs.widgetRoot().cmdParser().commandPool().add<aris::core::Command>(aris::server::default_disable_command());
	cs.addCmd("en", aris::server::default_parse, aris::server::default_enable_plan);
	cs.addCmd("ds", aris::server::default_parse, aris::server::default_disable_plan);

	auto &rc = cs.widgetRoot().cmdParser().commandPool().add<aris::core::Command>("rc", "", "");
	auto &gp = rc.add<aris::core::GroupParam>("gp","");
	auto &t1 = gp.add<aris::core::Param>("t1", "3000", "", 't');
	auto &t2 = gp.add<aris::core::Param>("t2", "3000", "");
	auto &mag = gp.add<aris::core::Param>("mag", "1.0", "", 'm');
	cs.addCmd("rc", rc_parse_func, rc_plan_func);


	auto &m = cs.model().addMotion();

	//auto &st = dynamic_cast<aris::control::EthercatMaster&>(cs.controller()).slaveTypePool().add<aris::control::EthercatSlaveType>("st", 0x00030924, , 0x0300);
	auto &s1 = cs.controller().slavePool().add<aris::control::EthercatMotion>("s1", nullptr, 0, 0x0000009a, 0x00030924, 0x000103F6, 0x0300, 65536, 10.0, -10.0, 10.0, 0, 0);

	auto &tx = s1.pdoGroupPool().add<aris::control::PdoGroup>("index_1A00", 0x1A00, true);
	tx.add<aris::control::Pdo>("index_6064", 0x6064, 0x00, sizeof(std::int32_t));
	tx.add<aris::control::Pdo>("index_606c", 0x606c, 0x00, sizeof(std::int32_t));
	tx.add<aris::control::Pdo>("index_6041", 0x6041, 0x00, sizeof(std::uint16_t));

	auto &tx2 = s1.pdoGroupPool().add<aris::control::PdoGroup>("index_1A0B", 0x1A0B, true);
	tx2.add<aris::control::Pdo>("index_6061", 0x6061, 0x00, sizeof(std::uint8_t));

	auto &tx3 = s1.pdoGroupPool().add<aris::control::PdoGroup>("index_1A1F", 0x1A1F, true);
	tx3.add<aris::control::Pdo>("index_6078", 0x6078, 0x00, sizeof(std::int16_t));

	auto &rx = s1.pdoGroupPool().add<aris::control::PdoGroup>("index_1605", 0x1605, false);
	rx.add<aris::control::Pdo>("index_607A", 0x607A, 0x00, sizeof(std::int32_t));
	rx.add<aris::control::Pdo>("index_60FF", 0x60FF, 0x00, sizeof(std::int32_t));
	rx.add<aris::control::Pdo>("index_6071", 0x6071, 0x00, sizeof(std::int16_t));
	rx.add<aris::control::Pdo>("index_6072", 0x6072, 0x00, sizeof(std::int16_t));
	rx.add<aris::control::Pdo>("index_6040", 0x6040, 0x00, sizeof(std::uint16_t));
	rx.add<aris::control::Pdo>("index_6060", 0x6060, 0x00, sizeof(std::uint8_t));

	cs.start();

	// 接收并打印信息 //
	auto t = std::thread([&]()
	{
		for (;;)
		{
			aris::core::Msg msg;
			cs.controller().recvOut(msg);
			if (!msg.empty())std::cout << msg.data() << std::endl;

			std::this_thread::sleep_for(std::chrono::milliseconds(10));
		}
	});

	// 接收命令 //
	for (std::string command_in; std::getline(std::cin, command_in);)
	{
		try
		{
			cs.executeCmd(command_in);
			std::cout << "cmd finished" << std::endl;
		}
		catch (std::exception &e)
		{
			std::cout << e.what() << std::endl;
		}
	}


	cs.saveXmlFile("C:\\Users\\py033\\Desktop\\cs.xml");

}



void test_control_server()
{
	test_xml();
	test_construct();

}

