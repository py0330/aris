#include <iostream>
#include <condition_variable>
#include <future>
#include <aris.h>

#include "test_control_server.h"

const char xml_data[] =
"<?xml version=\"1.0\" encoding=\"utf-8\" ?>"
"<root>"
"    <widget_root>"
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
"    <controller>"
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
"                    <home_mode index=\"0x6098\" subindex=\"0\" datatype=\"int8\" config=\"35\"/>"
"                    <home_acc index=\"0x609A\" subindex=\"0\" datatype=\"uint32\" config=\"200000\"/>"
"                    <home_high_speed index=\"0x6099\" subindex=\"1\" datatype=\"uint32\" config=\"200000\"/>"
"                    <home_low_speed index=\"0x6099\" subindex=\"2\" datatype=\"uint32\" config=\"100000\"/>"
"                    <home_offset index=\"0x607C\" subindex=\"0\" datatype=\"int32\" config=\"0\"/>"
"                </sdo_pool>"
"            </elmo>"
"        </slave_type_pool>"
"        <slave_pool type=\"SlavePoolObject\">"
"            <m1 type=\"Motion\" slave_type=\"elmo\" min_pos=\"-10.0\" max_pos=\"10.0\" max_vel=\"10.0\" home_pos=\"0\" input2count=\"62914560\"/>"
"            <m2 type=\"Motion\" slave_type=\"elmo\" min_pos=\"-10.0\" max_pos=\"10.0\" max_vel=\"10.0\" home_pos=\"0\" input2count=\"22937600\"/>"
"            <m3 type=\"Motion\" slave_type=\"elmo\" min_pos=\"-10.0\" max_pos=\"10.0\" max_vel=\"10.0\" home_pos=\"0\" input2count=\"22937600\"/>"
"        </slave_pool>"
"    </controller>"
"    <model>"
"        <environment type=\"Environment\" gravity=\"{0,-9.8,0,0,0,0}\"/>"
"        <variable_pool type=\"VariablePoolElement\" default_child_type=\"Matrix\">"
"            <PI type=\"MatrixVariable\">3.14159265358979</PI>"
"            <Mot_friction type=\"MatrixVariable\">{20, 30, 560}</Mot_friction>"
"        </variable_pool>"
"        <part_pool type=\"PartPoolElement\" default_child_type=\"Part\">"
"            <ground active=\"true\" inertia=\"{1,0,0,0,1,1,1,0,0,0}\" pe=\"{0,0,0,0,0,0}\" vel=\"{0,0,0,0,0,0}\" acc=\"{0,0,0,0,0,0}\" graphic_file_path=\"\">"
"                <marker_pool type=\"MarkerPoolElement\" default_child_type=\"Marker\">"
"                    <r1j pe=\"{ 0,0,0,0,0,0 }\"/>"
"                </marker_pool>"
"            </ground>"
"            <part1 active=\"true\" inertia=\"{1,0,0,0,1,1,1,0,0,0}\" pe=\"{0,0,0,0,0,0}\" vel=\"{0,0,0,0,0,0}\" acc=\"{0,0,0,0,0,0}\" graphic_file_path=\"C:\\aris\\robot\\resource\\graphic_file\\part1.x_t\">"
"                <marker_pool type=\"MarkerPoolElement\" default_child_type=\"Marker\">"
"                    <r1i pe=\"{ 0,0,0,0,0,0 }\"/>"
"                    <r2j pe=\"{ 1,0,0,0,0,0 }\"/>"
"                </marker_pool>"
"            </part1>"
"            <part2 active=\"true\" inertia=\"{1,0,0,0,1,1,1,0,0,0}\" pe=\"{1,0,0,PI/2,0,0}\" vel=\"{0,0,0,0,0,0}\" acc=\"{0,0,0,0,0,0}\" graphic_file_path=\"C:\\aris\\robot\\resource\\graphic_file\\part2.x_t\">"
"                <marker_pool type=\"MarkerPoolElement\" default_child_type=\"Marker\">"
"                    <r2i pe=\"{ 0,0,0,0,0,0 }\"/>"
"                    <r3j pe=\"{ 1,0,0,0,0,0 }\"/>"
"                </marker_pool>"
"            </part2>"
"            <part3 active=\"true\" inertia=\"{1,0,0,0,1,1,1,0,0,0}\" pe=\"{1,1,0,0,0,0}\" vel=\"{0,0,0,0,0,0}\" acc=\"{0,0,0,0,0,0}\" graphic_file_path=\"C:\\aris\\robot\\resource\\graphic_file\\part3.x_t\">"
"                <marker_pool type=\"MarkerPoolElement\" default_child_type=\"Marker\">"
"                    <r3i pe=\"{ 0,0,0,0,0,0 }\"/>"
"                </marker_pool>"
"            </part3>"
"        </part_pool>"
"        <joint_pool type=\"JointPoolElement\">"
"            <r1 active=\"true\" type=\"RevoluteJoint\" prt_m=\"part1\" prt_n=\"ground\" mak_i=\"r1i\" mak_j=\"r1j\"/>"
"            <r2 active=\"true\" type=\"RevoluteJoint\" prt_m=\"part2\" prt_n=\"part1\" mak_i=\"r2i\" mak_j=\"r2j\"/>"
"            <r3 active=\"true\" type=\"RevoluteJoint\" prt_m=\"part3\" prt_n=\"part2\" mak_i=\"r3i\" mak_j=\"r3j\"/>"
"        </joint_pool>"
"        <motion_pool type=\"MotionPoolElement\" default_child_type=\"Motion\">"
"            <m1 active=\"true\" slave_id=\"2\" prt_m=\"part1\" prt_n=\"ground\" mak_i=\"r1i\" mak_j=\"r1j\" frc_coe=\"Mot_friction\" component=\"5\"/>"
"            <m2 active=\"true\" slave_id=\"1\" prt_m=\"part2\" prt_n=\"part1\" mak_i=\"r2i\" mak_j=\"r2j\" frc_coe=\"Mot_friction\" component=\"5\"/>"
"            <m3 active=\"true\" slave_id=\"0\" prt_m=\"part3\" prt_n=\"part2\" mak_i=\"r3i\" mak_j=\"r3j\" frc_coe=\"Mot_friction\" component=\"5\"/>"
"        </motion_pool>"
"        <general_motion_pool type=\"GeneralMotionPoolElement\" default_child_type=\"GeneralMotion\"/>"
"    </model>"
"    <sensor_root>"
"        <sensor_pool type=\"SensorPoolObject\"/>"
"    </sensor_root>"
"</root>";


struct RcParam :aris::server::GaitParamBase
{
	int t1;
	int t2;
	double mag;
};

void test_control_server()
{
	try
	{
		auto rc_parse_func = [](aris::server::ControlServer &cs, const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg_out)->void
		{
			RcParam param;
			param.t1 = std::atoi(params.at("t1").c_str());
			param.mag = std::atof(params.at("mag").c_str());
			param.active_motor_[cs.model().motionAtPhy(0).absID()] = true;
			param.active_motor_[cs.model().motionAtPhy(1).absID()] = false;
			param.active_motor_[cs.model().motionAtPhy(2).absID()] = false;
			msg_out.copyStruct(param);
		};
		auto rc_plan_func = [](const aris::dynamic::PlanParam &param)->int
		{
			auto p = reinterpret_cast<RcParam*>(param.param_);

			static double begin_pos;

			if (param.count_ == 1)
			{
				auto &rx = aris::server::ControlServer::instance().controller().slavePool().at(param.model_->motionAtPhy(0).slaID()).rxData();
				begin_pos = static_cast<aris::control::Motion::RxType&>(rx).feedback_pos;
			}

			param.model_->motionAtPhy(0).setMp(begin_pos + p->mag * std::sin(2 * PI * param.count_ / p->t1));

			return p->t1 - param.count_;
		};

		aris::core::XmlDocument xml_doc;
		xml_doc.Parse(xml_data);

		auto&cs = aris::server::ControlServer::instance();

		cs.loadXml(xml_doc);
		cs.addCmd("rc", rc_parse_func, rc_plan_func);

		// 接收并打印信息 //
		auto t = std::thread([&]()
		{
			for (;;)
			{
				aris::core::Msg msg;
				cs.widgetRoot().msgPipe().recvMsg(msg);
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

