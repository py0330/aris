#include <iostream>
#include <aris.h>

#include "test_control_server.h"

#ifdef UNIX
#include "rtdk.h"
#include "unistd.h"
#endif

const char xml_file[] =
"<?xml version=\"1.0\" encoding=\"utf-8\"?>"
"<Robot>"
"  <Model>"
"    <Environment type=\"environment\" gravity=\"{0 , -9.8 , 0 , 0 , 0 , 0}\" />"
"    <Variable type=\"variable_pool_element\" default_child_type=\"matrix\">"
"      <PI type=\"matrix\">3.14159265358979</PI>"
"      <r1j_pe type=\"matrix\">{ 0,0,0,0,0,0 }</r1j_pe>"
"      <r1i_pe type=\"matrix\">{ 0,0,0,0,0,0 }</r1i_pe>"
"      <r2j_pe type=\"matrix\">{ 1,0,0,0,0,0 }</r2j_pe>"
"      <r2i_pe type=\"matrix\">{ 0,0,0,0,0,0 }</r2i_pe>"
"      <r3j_pe type=\"matrix\">{ 1,0,0,0,0,0 }</r3j_pe>"
"      <r3i_pe type=\"matrix\">{ 0,0,0,0,0,0 }</r3i_pe>"
"      <Mot_friction type=\"matrix\">{20, 30, 560}</Mot_friction>"
"    </Variable>"
"    <Akima type=\"akima_pool_element\" default_child_type=\"akima\">"
"      <m1_akima x=\"{0,1,2,3,4}\" y=\"{0,1,2,3,4}\" />"
"      <m2_akima x=\"{0,1,2,3,4}\" y=\"{0,1,2,3,4}\" />"
"      <m3_akima x=\"{0,1,2,3,4}\" y=\"{0,1,2,3,4}\" />"
"    </Akima>"
"    <Part type=\"part_pool_element\" default_child_type=\"part\">"
"      <Ground active=\"true\" inertia=\"{1 , 0 , 0 , 0 , 1 , 1 , 1 , 0 , 0 , 0}\" pe=\"{0 , 0 , 0 , 0 , 0 , 0}\" vel=\"{0 , 0 , 0 , 0 , 0 , 0}\" acc=\"{0 , 0 , 0 , 0 , 0 , 0}\" graphic_file_path=\"\">"
"        <ChildMarker type=\"marker_pool_element\" default_child_type=\"marker\">"
"          <r1j pe=\"r1j_pe\" />"
"        </ChildMarker>"
"      </Ground>"
"      <part1 active=\"true\" inertia=\"{1 , 0 , 0 , 0 , 1 , 1 , 1 , 0 , 0 , 0}\" pe=\"{0 , 0 , 0 , 0 , 0 , 0}\" vel=\"{0 , 0 , 0 , 0 , 0 , 0}\" acc=\"{0 , 0 , 0 , 0 , 0 , 0}\" graphic_file_path=\"C:\\Users\\py033\\Desktop\\part1.xmt_txt\">"
"        <ChildMarker type=\"marker_pool_element\" default_child_type=\"marker\">"
"          <r1i pe = \"r1i_pe\" />"
"	  	   <r2j pe = \"r2j_pe\" />"
"        </ChildMarker>"
"      </part1>"
"      <part2 active=\"true\" inertia=\"{1 , 0 , 0 , 0 , 1 , 1 , 1 , 0 , 0 , 0}\" pe=\"{1 , 0 , 0 , PI/2 , 0 , 0}\" vel=\"{0 , 0 , 0 , 0 , 0 , 0}\" acc=\"{0 , 0 , 0 , 0 , 0 , 0}\" graphic_file_path=\"C:\\Users\\py033\\Desktop\\part2.xmt_txt\">"
"        <ChildMarker type=\"marker_pool_element\" default_child_type=\"marker\">"
"          <r2i pe = \"r2i_pe\" />"
"	  	   <r3j pe = \"r3j_pe\" />"
"        </ChildMarker>"
"      </part2>"
"      <part3 active=\"true\" inertia=\"{1 , 0 , 0 , 0 , 1 , 1 , 1 , 0 , 0 , 0}\" pe=\"{1 , 1 , 0 , 0 , 0 , 0}\" vel=\"{0 , 0 , 0 , 0 , 0 , 0}\" acc=\"{0 , 0 , 0 , 0 , 0 , 0}\" graphic_file_path=\"C:\\Users\\py033\\Desktop\\part3.xmt_txt\">"
"        <ChildMarker type=\"marker_pool_element\" default_child_type=\"marker\">"
"          <r3i pe = \"r3i_pe\" />"
"        </ChildMarker>"
"      </part3>"
"    </Part>"
"    <Joint type=\"joint_pool_element\">"
"      <r1 active=\"true\" type=\"revolute\" prt_m=\"part1\" prt_n=\"Ground\" mak_i=\"r1i\" mak_j=\"r1j\" />"
"      <r2 active=\"true\" type=\"revolute\" prt_m=\"part2\" prt_n=\"part1\" mak_i=\"r2i\" mak_j=\"r2j\" />"
"      <r3 active=\"true\" type=\"revolute\" prt_m=\"part3\" prt_n=\"part2\" mak_i=\"r3i\" mak_j=\"r3j\" />"
"    </Joint>"
"    <Motion type=\"motion_pool_element\" default_child_type=\"motion\">"
"      <m1 active=\"true\" slave_id=\"0\" prt_m=\"part1\" prt_n=\"Ground\" mak_i=\"r1i\" mak_j=\"r1j\" frc_coe=\"Mot_friction\" component=\"5\" />"
"      <m2 active=\"true\" slave_id=\"1\" prt_m=\"part2\" prt_n=\"part1\" mak_i=\"r2i\" mak_j=\"r2j\" frc_coe=\"Mot_friction\" component=\"5\" />"
"      <m3 active=\"true\" slave_id=\"2\" prt_m=\"part3\" prt_n=\"part2\" mak_i=\"r3i\" mak_j=\"r3j\" frc_coe=\"Mot_friction\" component=\"5\" />"
"    </Motion>"
"    <General_Motion type=\"general_motion_pool_element\" default_child_type=\"general_motion\">"
"    </General_Motion>"
"  </Model>"
"  <Server ip=\"127.0.0.1\" port=\"5866\">"
"    <Commands type=\"command_pool_object\" default_child_type=\"command\">"
"      <start default_child_type=\"param\" />"
"      <stop />"
"      <exit />"
"      <en default_child_type=\"param\" default=\"all\">"
"        <all abbreviation=\"a\" />"
"        <motion_id abbreviation=\"m\" default=\"0\" />"
"        <physical_id abbreviation=\"p\" default=\"0\" />"
"        <slave_id abbreviation=\"s\" default=\"0\" />"
"      </en>"
"      <ds default_child_type=\"param\" default=\"all\">"
"        <all abbreviation=\"a\" />"
"        <motion_id abbreviation=\"m\" default=\"0\" />"
"        <physical_id abbreviation=\"p\" default=\"0\" />"
"        <slave_id abbreviation=\"s\" default=\"0\" />"
"      </ds>"
"      <hm default_child_type=\"param\" default=\"all\">"
"        <all abbreviation=\"a\" />"
"        <motion_id abbreviation=\"m\" default=\"0\" />"
"        <physical_id abbreviation=\"p\" default=\"0\" />"
"        <slave_id abbreviation=\"s\" default=\"0\" />"
"      </hm>"
"      <rc />"
"      <mvpee default_child_type=\"param\" default=\"movepee_param\">"
"		 <movepee_param type=\"group\" default_child_type=\"param\" >"
"           <totalCount abbreviation=\"t\" default=\"10000\"/>"
"           <y abbreviation=\"y\" default=\"0.435\"/>"
"           <z abbreviation=\"z\" default=\"0\"/>"
"           <a abbreviation=\"a\" default=\"0\"/>"
"           <c abbreviation=\"c\" default=\"0\"/>"
"        </movepee_param>"
"      </mvpee>"
"      <mvpin default=\"movepin_param\">"
"	     <movepin_param type=\"group\" default_child_type=\"param\" >"
"          <motion_param type=\"unique\" default_child_type=\"param\" default=\"all\">"
"	         <all abbreviation=\"a\" />"
"	         <motion_id abbreviation=\"m\" default=\"0\" />"
"	         <physical_id abbreviation=\"p\" default=\"0\" />"
"	         <slave_id abbreviation=\"s\" default=\"0\" />"
"	       </motion_param>"
"	       <totalCount abbreviation=\"t\" default=\"10000\"/>"
"          <velocity abbreviation=\"v\" default=\"0\"/>"
"          <force abbreviation=\"f\" default=\"no\"/>"
"        </movepin_param>"
"      </mvpin>"
"    </Commands>"
"  </Server>"
"  <Controller>"
"    <SlaveType type=\"slave_type_pool_element\">"
"      <ElmoSoloWhistle type=\"slave_type\" product_code=\"0x00030924\" vender_id=\"0x0000009a\" alias=\"0\" distributed_clock=\"0x0300\">"
"        <PDO type=\"pdo_group_pool_element\">"
"          <index_1605 type=\"pdo_group\" default_child_type=\"pdo\" index=\"0x1605\" is_tx=\"false\">"
"            <TargetPosition index=\"0x607A\" subindex=\"0x00\" datatype=\"int32\" />"
"            <TargetVelocity index=\"0x60FF\" subindex=\"0x00\" datatype=\"int32\" />"
"            <TargetTorque index=\"0x6071\" subindex=\"0x00\" datatype=\"int16\" />"
"            <MaxTorque index=\"0x6072\" subindex=\"0x00\" datatype=\"int16\" />"
"            <ControlWord index=\"0x6040\" subindex=\"0x00\" datatype=\"uint16\" />"
"            <ModeOfOperation index=\"0x6060\" subindex=\"0x00\" datatype=\"uint8\" />"
"          </index_1605>"
"          <index_1617 type=\"pdo_group\" default_child_type=\"pdo\" index=\"0x1617\" is_tx=\"false\">"
"            <VelocityOffset index=\"0x60B1\" subindex=\"0x00\" datatype=\"int32\" />"
"          </index_1617>"
"          <index_1618 type=\"pdo_group\" default_child_type=\"pdo\" index=\"0x1618\" is_tx=\"false\">"
"            <TorqueOffset index=\"0x60B2\" subindex=\"0x00\" datatype=\"int16\" />"
"          </index_1618>"
"          <index_1a03 type=\"pdo_group\" default_child_type=\"pdo\" index=\"0x1A03\" is_tx=\"true\">"
"            <PositionActualValue index=\"0x6064\" subindex=\"0x00\" datatype=\"int32\" />"
"            <DigitalInputs index=\"0x60fd\" subindex=\"0x00\" datatype=\"uint32\" />"
"            <VelocityActualValue index=\"0x606c\" subindex=\"0x00\" datatype=\"int32\" />"
"            <StatusWord index=\"0x6041\" subindex=\"0x00\" datatype=\"uint16\" />"
"          </index_1a03>"
"          <index_1a1f type=\"pdo_group\" default_child_type=\"pdo\" index=\"0x1A1F\" is_tx=\"true\">"
"            <CurrentActualValue index=\"0x6078\" subindex=\"0x00\" datatype=\"int16\" />"
"          </index_1a1f>"
"          <index_1a13 type=\"pdo_group\" default_child_type=\"pdo\" index=\"0x1A13\" is_tx=\"true\">"
"            <TorqueActualValue index=\"0x6077\" subindex=\"0x00\" datatype=\"int16\" />"
"          </index_1a13>"
"          <index_1a0b type=\"pdo_group\" default_child_type=\"pdo\" index=\"0x1A0B\" is_tx=\"true\">"
"            <ModeOfOperationDisplay index=\"0x6061\" subindex=\"0x00\" datatype=\"uint8\" />"
"          </index_1a0b>"
"        </PDO>"
"        <SDO type=\"sdo_pool_element\" default_child_type=\"sdo\">"
"          <homeMode index=\"0x6098\" subindex=\"0\" datatype=\"int8\" config=\"17\" />"
"          <homeAcc index=\"0x609A\" subindex=\"0\" datatype=\"uint32\" config=\"20000\" />"
"          <homeHighSpeed index=\"0x6099\" subindex=\"1\" datatype=\"uint32\" config=\"25000\" />"
"          <homeLowSpeed index=\"0x6099\" subindex=\"2\" datatype=\"uint32\" config=\"5000\" />"
"          <homeOffset index=\"0x607C\" subindex=\"0\" datatype=\"int32\" config=\"0\" />"
"        </SDO>"
"      </ElmoSoloWhistle>"
"    </SlaveType>"
"    <Slave type=\"slave_pool_element\">"
"      <motion1 type=\"motion\" slave_type=\"ElmoSoloWhistle\" min_pos=\"0.4587\" max_pos=\"0.59967\" max_vel=\"0.5\" home_pos=\"0.45879\" input2count=\"6553600\" />"
"      <motion2 type=\"motion\" slave_type=\"ElmoSoloWhistle\" min_pos=\"0.4587\" max_pos=\"0.59967\" max_vel=\"0.5\" home_pos=\"0.45879\" input2count=\"6553600\" />"
"      <motion3 type=\"motion\" slave_type=\"ElmoSoloWhistle\" min_pos=\"0.4739\" max_pos=\"0.61493\" max_vel=\"0.5\" home_pos=\"0.47399\" input2count=\"6553600\" />"
"    </Slave>"
"  </Controller>"
"  <Sensor>"
"    <SensorPool type=\"sensor_pool_object\">"
"    </SensorPool>"
"  </Sensor>"
"</Robot>";




void test_control_server()
{
	try 
	{
		aris::core::XmlDocument xml_doc;
		//xml_doc.Parse(xml_file);
		xml_doc.LoadFile("C:\\aris\\robot\\resource\\robot_motion.xml");
		
		auto &cs = aris::server::ControlServer::instance();
		cs.createModel<aris::dynamic::Model>();
		cs.createController<aris::control::Controller>();
		cs.createSensorRoot<aris::sensor::SensorRoot>();

		

		cs.loadXml(xml_doc);

		aris::dynamic::PlanParamBase p;

		aris::dynamic::PlanFunc f = [](aris::dynamic::Model &m, const aris::dynamic::PlanParamBase &p)
		{
			
			double pe2[6]{ 0,0,0,0,0,0 };
			pe2[5] = p.count_*0.001/3*PI /3 + PI / 2;
			auto &r2j = *m.partPool().findByName("part1")->markerPool().findByName("r2j");
			m.partPool().findByName("part2")->setPe(r2j, pe2, "123");

			double pe3[6]{ 0,0,0,0,0,0 };
			pe3[5] = p.count_*0.001/3*PI /3 - PI / 2;
			auto &r3j = *m.partPool().findByName("part2")->markerPool().findByName("r3j");
			m.partPool().findByName("part3")->setPe(r3j, pe3, "123");

			m.motionAtAbs(0).update();
			m.motionAtAbs(1).update();
			m.motionAtAbs(2).update();

			return 3000-p.count_;
		};


		cs.model().saveDynEle("before");
		cs.model().simKin(f,p);
		cs.model().loadDynEle("before");
		cs.model().saveAdams("C:\\aris\\robot\\resource\\test.cmd");
	}
	catch (std::exception &e)
	{
		std::cout << e.what()<<std::endl;
	}
}