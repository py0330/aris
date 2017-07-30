#include "aris.h"

const char xml_data[] =
"<?xml version=\"1.0\" encoding=\"utf-8\" ?>"
"<root>"
"    <widget_root>"
"        <message_pipe type=\"Pipe\" pool_size=\"16384\"/>"
"        <command_pipe type=\"Pipe\" pool_size=\"16384\"/>"
"        <command_socket type=\"Socket\" port=\"5866\"/>"
"        <command_parser type=\"CommandParser\">"
"            <command_pool type=\"CommandPoolObject\" default_child_type=\"Command\">"
"                <rc default=\"rc_param\">"
"                    <rc_param type=\"GroupParam\" default_child_type=\"Param\">"
"                        <p1 default=\"0.0\"/>"
"                        <p2 default=\"0.0\"/>"
"                        <p3 default=\"0.0\"/>"
"                        <p4 default=\"0.0\"/>"
"                        <p5 default=\"90.0\"/>"
"                        <p6 default=\"0.0\"/>"
"                    </rc_param>"
"                </rc>"
"                <mv default=\"mv_param\">"
"                    <mv_param type=\"UniqueParam\" default_child_type=\"Param\">"
"                        <x default=\"0.0\"/>"
"                        <y default=\"0.0\"/>"
"                        <z default=\"0.0\"/>"
"                        <a default=\"0.0\"/>"
"                        <b default=\"0.0\"/>"
"                        <c default=\"0.0\"/>"
"                    </mv_param>"
"                </mv>"
"                <ck/>"
"            </command_pool>"
"        </command_parser>"
"    </widget_root>"
"    <controller>"
"        <slave_type_pool type=\"SlaveTypePoolObject\">"
"            <elmo type=\"SlaveType\" product_code=\"0x00010001\" vender_id=\"0x00007595\" alias=\"0\" distributed_clock=\"0x0300\">"
"                <pdo_group_pool type=\"PdoGroupPoolObject\">"
"                    <index_1600 type=\"PdoGroup\" default_child_type=\"Pdo\" index=\"0x1600\" is_tx=\"false\">"
"                        <control_word index=\"0x6040\" subindex=\"0x00\" size=\"2\"/>"
"                        <mode_of_operation index=\"0x6060\" subindex=\"0x00\" size=\"1\"/>"
"                        <target_pos index=\"0x607A\" subindex=\"0x00\" size=\"4\"/>"
"                        <target_vel index=\"0x60FF\" subindex=\"0x00\" size=\"4\"/>"
"                        <targer_tor index=\"0x6071\" subindex=\"0x00\" size=\"2\"/>"
"                    </index_1600>"
"                    <index_1a00 type=\"PdoGroup\" default_child_type=\"Pdo\" index=\"0x1A00\" is_tx=\"true\">"
"                        <status_word index=\"0x6041\" subindex=\"0x00\" size=\"2\"/>"
"                        <mode_of_display index=\"0x6061\" subindex=\"0x00\" size=\"1\"/>"
"                        <pos_actual_value index=\"0x6064\" subindex=\"0x00\" size=\"4\"/>"
"                        <vel_actual_value index=\"0x606c\" subindex=\"0x00\" size=\"4\"/>"
"                        <cur_actual_value index=\"0x6078\" subindex=\"0x00\" size=\"2\"/>"
"                    </index_1a00>"
"                </pdo_group_pool>"
"                <sdo_pool type=\"SdoPoolObject\" default_child_type=\"Sdo\"/>"
"            </elmo>"
"        </slave_type_pool>"
"        <slave_pool type=\"SlavePoolObject\">"
"            <m1 type=\"Motion\" slave_type=\"elmo\" min_pos=\"-135\" max_pos=\"135\" max_vel=\"200\" pos_offset=\"10.7925888626948\" home_pos=\"0\" pos_factor=\"43691\">"
"                <pdo_group_pool type=\"PdoGroupPoolObject\">"
"                    <index_1600 type=\"PdoGroup\" default_child_type=\"Pdo\" index=\"0x1600\" is_tx=\"false\">"
"                        <control_word index=\"0x6040\" subindex=\"0x00\" size=\"2\"/>"
"                        <mode_of_operation index=\"0x6060\" subindex=\"0x00\" size=\"1\"/>"
"                        <target_pos index=\"0x607A\" subindex=\"0x00\" size=\"4\"/>"
"                        <target_vel index=\"0x60FF\" subindex=\"0x00\" size=\"4\"/>"
"                        <targer_tor index=\"0x6071\" subindex=\"0x00\" size=\"2\"/>"
"                    </index_1600>"
"                    <index_1a00 type=\"PdoGroup\" default_child_type=\"Pdo\" index=\"0x1A00\" is_tx=\"true\">"
"                        <status_word index=\"0x6041\" subindex=\"0x00\" size=\"2\"/>"
"                        <mode_of_display index=\"0x6061\" subindex=\"0x00\" size=\"1\"/>"
"                        <pos_actual_value index=\"0x6064\" subindex=\"0x00\" size=\"4\"/>"
"                        <vel_actual_value index=\"0x606c\" subindex=\"0x00\" size=\"4\"/>"
"                        <cur_actual_value index=\"0x6078\" subindex=\"0x00\" size=\"2\"/>"
"                    </index_1a00>"
"                </pdo_group_pool>"
"                <sdo_pool type=\"SdoPoolObject\" default_child_type=\"Sdo\"/>"
"            </m1>"
"            <m2 type=\"Motion\" slave_type=\"elmo\" min_pos=\"-135\" max_pos=\"135\" max_vel=\"200\" pos_offset=\"-8.85539355931428\" home_pos=\"0\" pos_factor=\"43691\">"
"                <pdo_group_pool type=\"PdoGroupPoolObject\">"
"                    <index_1600 type=\"PdoGroup\" default_child_type=\"Pdo\" index=\"0x1600\" is_tx=\"false\">"
"                        <control_word index=\"0x6040\" subindex=\"0x00\" size=\"2\"/>"
"                        <mode_of_operation index=\"0x6060\" subindex=\"0x00\" size=\"1\"/>"
"                        <target_pos index=\"0x607A\" subindex=\"0x00\" size=\"4\"/>"
"                        <target_vel index=\"0x60FF\" subindex=\"0x00\" size=\"4\"/>"
"                        <targer_tor index=\"0x6071\" subindex=\"0x00\" size=\"2\"/>"
"                    </index_1600>"
"                    <index_1a00 type=\"PdoGroup\" default_child_type=\"Pdo\" index=\"0x1A00\" is_tx=\"true\">"
"                        <status_word index=\"0x6041\" subindex=\"0x00\" size=\"2\"/>"
"                        <mode_of_display index=\"0x6061\" subindex=\"0x00\" size=\"1\"/>"
"                        <pos_actual_value index=\"0x6064\" subindex=\"0x00\" size=\"4\"/>"
"                        <vel_actual_value index=\"0x606c\" subindex=\"0x00\" size=\"4\"/>"
"                        <cur_actual_value index=\"0x6078\" subindex=\"0x00\" size=\"2\"/>"
"                    </index_1a00>"
"                </pdo_group_pool>"
"                <sdo_pool type=\"SdoPoolObject\" default_child_type=\"Sdo\"/>"
"            </m2>"
"            <m3 type=\"Motion\" slave_type=\"elmo\" min_pos=\"-120\" max_pos=\"70\" max_vel=\"191\" pos_offset=\"-7.15872044749377\" home_pos=\"0\" pos_factor=\"45766\">"
"                <pdo_group_pool type=\"PdoGroupPoolObject\">"
"                    <index_1600 type=\"PdoGroup\" default_child_type=\"Pdo\" index=\"0x1600\" is_tx=\"false\">"
"                        <control_word index=\"0x6040\" subindex=\"0x00\" size=\"2\"/>"
"                        <mode_of_operation index=\"0x6060\" subindex=\"0x00\" size=\"1\"/>"
"                        <target_pos index=\"0x607A\" subindex=\"0x00\" size=\"4\"/>"
"                        <target_vel index=\"0x60FF\" subindex=\"0x00\" size=\"4\"/>"
"                        <targer_tor index=\"0x6071\" subindex=\"0x00\" size=\"2\"/>"
"                    </index_1600>"
"                    <index_1a00 type=\"PdoGroup\" default_child_type=\"Pdo\" index=\"0x1A00\" is_tx=\"true\">"
"                        <status_word index=\"0x6041\" subindex=\"0x00\" size=\"2\"/>"
"                        <mode_of_display index=\"0x6061\" subindex=\"0x00\" size=\"1\"/>"
"                        <pos_actual_value index=\"0x6064\" subindex=\"0x00\" size=\"4\"/>"
"                        <vel_actual_value index=\"0x606c\" subindex=\"0x00\" size=\"4\"/>"
"                        <cur_actual_value index=\"0x6078\" subindex=\"0x00\" size=\"2\"/>"
"                    </index_1a00>"
"                </pdo_group_pool>"
"                <sdo_pool type=\"SdoPoolObject\" default_child_type=\"Sdo\"/>"
"            </m3>"
"            <m4 type=\"Motion\" slave_type=\"elmo\" min_pos=\"-160\" max_pos=\"160\" max_vel=\"480\" pos_offset=\"15.9680839375961\" home_pos=\"0\" pos_factor=\"-18204\">"
"                <pdo_group_pool type=\"PdoGroupPoolObject\">"
"                    <index_1600 type=\"PdoGroup\" default_child_type=\"Pdo\" index=\"0x1600\" is_tx=\"false\">"
"                        <control_word index=\"0x6040\" subindex=\"0x00\" size=\"2\"/>"
"                        <mode_of_operation index=\"0x6060\" subindex=\"0x00\" size=\"1\"/>"
"                        <target_pos index=\"0x607A\" subindex=\"0x00\" size=\"4\"/>"
"                        <target_vel index=\"0x60FF\" subindex=\"0x00\" size=\"4\"/>"
"                        <targer_tor index=\"0x6071\" subindex=\"0x00\" size=\"2\"/>"
"                    </index_1600>"
"                    <index_1a00 type=\"PdoGroup\" default_child_type=\"Pdo\" index=\"0x1A00\" is_tx=\"true\">"
"                        <status_word index=\"0x6041\" subindex=\"0x00\" size=\"2\"/>"
"                        <mode_of_display index=\"0x6061\" subindex=\"0x00\" size=\"1\"/>"
"                        <pos_actual_value index=\"0x6064\" subindex=\"0x00\" size=\"4\"/>"
"                        <vel_actual_value index=\"0x606c\" subindex=\"0x00\" size=\"4\"/>"
"                        <cur_actual_value index=\"0x6078\" subindex=\"0x00\" size=\"2\"/>"
"                    </index_1a00>"
"                </pdo_group_pool>"
"                <sdo_pool type=\"SdoPoolObject\" default_child_type=\"Sdo\"/>"
"            </m4>"
"            <m5 type=\"Motion\" slave_type=\"elmo\" min_pos=\"-120\" max_pos=\"120\" max_vel=\"403\" pos_offset=\"0.377648525135023\" home_pos=\"0\" pos_factor=\"21663\">"
"                <pdo_group_pool type=\"PdoGroupPoolObject\">"
"                    <index_1600 type=\"PdoGroup\" default_child_type=\"Pdo\" index=\"0x1600\" is_tx=\"false\">"
"                        <control_word index=\"0x6040\" subindex=\"0x00\" size=\"2\"/>"
"                        <mode_of_operation index=\"0x6060\" subindex=\"0x00\" size=\"1\"/>"
"                        <target_pos index=\"0x607A\" subindex=\"0x00\" size=\"4\"/>"
"                        <target_vel index=\"0x60FF\" subindex=\"0x00\" size=\"4\"/>"
"                        <targer_tor index=\"0x6071\" subindex=\"0x00\" size=\"2\"/>"
"                    </index_1600>"
"                    <index_1a00 type=\"PdoGroup\" default_child_type=\"Pdo\" index=\"0x1A00\" is_tx=\"true\">"
"                        <status_word index=\"0x6041\" subindex=\"0x00\" size=\"2\"/>"
"                        <mode_of_display index=\"0x6061\" subindex=\"0x00\" size=\"1\"/>"
"                        <pos_actual_value index=\"0x6064\" subindex=\"0x00\" size=\"4\"/>"
"                        <vel_actual_value index=\"0x606c\" subindex=\"0x00\" size=\"4\"/>"
"                        <cur_actual_value index=\"0x6078\" subindex=\"0x00\" size=\"2\"/>"
"                    </index_1a00>"
"                </pdo_group_pool>"
"                <sdo_pool type=\"SdoPoolObject\" default_child_type=\"Sdo\"/>"
"            </m5>"
"            <m6 type=\"Motion\" slave_type=\"elmo\" min_pos=\"-360\" max_pos=\"360\" max_vel=\"480\" pos_offset=\"-319.130136233795\" home_pos=\"0\" pos_factor=\"-18204\">"
"                <pdo_group_pool type=\"PdoGroupPoolObject\">"
"                    <index_1600 type=\"PdoGroup\" default_child_type=\"Pdo\" index=\"0x1600\" is_tx=\"false\">"
"                        <control_word index=\"0x6040\" subindex=\"0x00\" size=\"2\"/>"
"                        <mode_of_operation index=\"0x6060\" subindex=\"0x00\" size=\"1\"/>"
"                        <target_pos index=\"0x607A\" subindex=\"0x00\" size=\"4\"/>"
"                        <target_vel index=\"0x60FF\" subindex=\"0x00\" size=\"4\"/>"
"                        <targer_tor index=\"0x6071\" subindex=\"0x00\" size=\"2\"/>"
"                    </index_1600>"
"                    <index_1a00 type=\"PdoGroup\" default_child_type=\"Pdo\" index=\"0x1A00\" is_tx=\"true\">"
"                        <status_word index=\"0x6041\" subindex=\"0x00\" size=\"2\"/>"
"                        <mode_of_display index=\"0x6061\" subindex=\"0x00\" size=\"1\"/>"
"                        <pos_actual_value index=\"0x6064\" subindex=\"0x00\" size=\"4\"/>"
"                        <vel_actual_value index=\"0x606c\" subindex=\"0x00\" size=\"4\"/>"
"                        <cur_actual_value index=\"0x6078\" subindex=\"0x00\" size=\"2\"/>"
"                    </index_1a00>"
"                </pdo_group_pool>"
"                <sdo_pool type=\"SdoPoolObject\" default_child_type=\"Sdo\"/>"
"            </m6>"
"        </slave_pool>"
"    </controller>"
"    <model type=\"Model\" time=\"0\">"
"        <environment type=\"Environment\" gravity=\"{0 , -9.8 , 0 , 0 , 0 , 0}\"/>"
"        <variable_pool type=\"VariablePoolElement\"/>"
"        <part_pool type=\"PartPoolElement\">"
"            <ground type=\"Part\" active=\"true\" pe=\"{0 , 0 , 0 , -0 , 0 , -0}\" vel=\"{0 , 0 , 0 , 0 , 0 , 0}\" acc=\"{0 , 0 , 0 , 0 , 0 , 0}\" inertia=\"{1 , 0 , 0 , 0 , 1 , 1 , 1 , 0 , 0 , 0}\">"
"                <marker_pool type=\"MarkerPoolElement\">"
"                    <joint_0_i type=\"Marker\" active=\"true\" pe=\"{0 , 0 , 0 , 3.14159265358979 , 1.5707963267949 , 4.71238898038469}\"/>"
"                    <general_motion_0_j type=\"Marker\" active=\"true\" pe=\"{0 , 0 , 0 , -0 , 0 , -0}\"/>"
"                </marker_pool>"
"                <geometry_pool type=\"GeometryPoolElement\"/>"
"            </ground>"
"            <part_1 type=\"Part\" active=\"true\" pe=\"{0 , 0 , 0 , -0 , 0 , -0}\" vel=\"{0 , 0 , 0 , 0 , 0 , 0}\" acc=\"{0 , 0 , 0 , 0 , 0 , 0}\" inertia=\"{1 , 0 , 0 , 0 , 1 , 1 , 1 , 0 , 0 , 0}\">"
"                <marker_pool type=\"MarkerPoolElement\">"
"                    <joint_0_j type=\"Marker\" active=\"true\" pe=\"{0 , 0 , 0 , 3.14159265358979 , 1.5707963267949 , 4.71238898038469}\"/>"
"                    <joint_1_i type=\"Marker\" active=\"true\" pe=\"{0 , 0.29 , 0 , 0.785398163397448 , 0 , 0.785398163397448}\"/>"
"                </marker_pool>"
"                <geometry_pool type=\"GeometryPoolElement\">"
"                    <solid type=\"ParasolidGeometry\" pe=\"{0 , 0 , 0 , -0 , 0 , -0}\" graphic_file_path=\"C:\\aris\\resource\\demo_hulk\\p1.xmt_txt\"/>"
"                </geometry_pool>"
"            </part_1>"
"            <part_2 type=\"Part\" active=\"true\" pe=\"{0 , 0 , 0 , -0 , 0 , -0}\" vel=\"{0 , 0 , 0 , 0 , 0 , 0}\" acc=\"{0 , 0 , 0 , 0 , 0 , 0}\" inertia=\"{1 , 0 , 0 , 0 , 1 , 1 , 1 , 0 , 0 , 0}\">"
"                <marker_pool type=\"MarkerPoolElement\">"
"                    <joint_1_j type=\"Marker\" active=\"true\" pe=\"{0 , 0.29 , 0 , 0.785398163397448 , 0 , 0.785398163397448}\"/>"
"                    <joint_2_i type=\"Marker\" active=\"true\" pe=\"{0 , 0.56 , 0 , 0.785398163397448 , 0 , 0.785398163397448}\"/>"
"                </marker_pool>"
"                <geometry_pool type=\"GeometryPoolElement\">"
"                    <solid type=\"ParasolidGeometry\" pe=\"{0 , 0 , 0 , -0 , 0 , -0}\" graphic_file_path=\"C:\\aris\\resource\\demo_hulk\\p2.xmt_txt\"/>"
"                </geometry_pool>"
"            </part_2>"
"            <part_3 type=\"Part\" active=\"true\" pe=\"{0 , 0 , 0 , -0 , 0 , -0}\" vel=\"{0 , 0 , 0 , 0 , 0 , 0}\" acc=\"{0 , 0 , 0 , 0 , 0 , 0}\" inertia=\"{1 , 0 , 0 , 0 , 1 , 1 , 1 , 0 , 0 , 0}\">"
"                <marker_pool type=\"MarkerPoolElement\">"
"                    <joint_2_j type=\"Marker\" active=\"true\" pe=\"{0 , 0.56 , 0 , 0.785398163397448 , 0 , 0.785398163397448}\"/>"
"                    <joint_3_i type=\"Marker\" active=\"true\" pe=\"{0 , 0.63 , 0 , 1.5707963267949 , 1.5707963267949 , 1.5707963267949}\"/>"
"                </marker_pool>"
"                <geometry_pool type=\"GeometryPoolElement\">"
"                    <solid type=\"ParasolidGeometry\" pe=\"{0 , 0 , 0 , -0 , 0 , -0}\" graphic_file_path=\"C:\\aris\\resource\\demo_hulk\\p3.xmt_txt\"/>"
"                </geometry_pool>"
"            </part_3>"
"            <part_4 type=\"Part\" active=\"true\" pe=\"{0 , 0 , 0 , -0 , 0 , -0}\" vel=\"{0 , 0 , 0 , 0 , 0 , 0}\" acc=\"{0 , 0 , 0 , 0 , 0 , 0}\" inertia=\"{1 , 0 , 0 , 0 , 1 , 1 , 1 , 0 , 0 , 0}\">"
"                <marker_pool type=\"MarkerPoolElement\">"
"                    <joint_3_j type=\"Marker\" active=\"true\" pe=\"{0 , 0.63 , 0 , 1.5707963267949 , 1.5707963267949 , 1.5707963267949}\"/>"
"                    <joint_4_i type=\"Marker\" active=\"true\" pe=\"{0.316 , 0.63 , 0 , 0.785398163397448 , 0 , 0.785398163397448}\"/>"
"                </marker_pool>"
"                <geometry_pool type=\"GeometryPoolElement\">"
"                    <solid type=\"ParasolidGeometry\" pe=\"{0 , 0 , 0 , -0 , 0 , -0}\" graphic_file_path=\"C:\\aris\\resource\\demo_hulk\\p4.xmt_txt\"/>"
"                </geometry_pool>"
"            </part_4>"
"            <part_5 type=\"Part\" active=\"true\" pe=\"{0 , 0 , 0 , -0 , 0 , -0}\" vel=\"{0 , 0 , 0 , 0 , 0 , 0}\" acc=\"{0 , 0 , 0 , 0 , 0 , 0}\" inertia=\"{1 , 0 , 0 , 0 , 1 , 1 , 1 , 0 , 0 , 0}\">"
"                <marker_pool type=\"MarkerPoolElement\">"
"                    <joint_4_j type=\"Marker\" active=\"true\" pe=\"{0.316 , 0.63 , 0 , 0.785398163397448 , 0 , 0.785398163397448}\"/>"
"                    <joint_5_i type=\"Marker\" active=\"true\" pe=\"{0.316 , 0.63 , 0 , 1.5707963267949 , 1.5707963267949 , 1.5707963267949}\"/>"
"                </marker_pool>"
"                <geometry_pool type=\"GeometryPoolElement\">"
"                    <solid type=\"ParasolidGeometry\" pe=\"{0 , 0 , 0 , -0 , 0 , -0}\" graphic_file_path=\"C:\\aris\\resource\\demo_hulk\\p5.xmt_txt\"/>"
"                </geometry_pool>"
"            </part_5>"
"            <part_6 type=\"Part\" active=\"true\" pe=\"{0 , 0 , 0 , -0 , 0 , -0}\" vel=\"{0 , 0 , 0 , 0 , 0 , 0}\" acc=\"{0 , 0 , 0 , 0 , 0 , 0}\" inertia=\"{1 , 0 , 0 , 0 , 1 , 1 , 1 , 0 , 0 , 0}\">"
"                <marker_pool type=\"MarkerPoolElement\">"
"                    <joint_5_j type=\"Marker\" active=\"true\" pe=\"{0.316 , 0.63 , 0 , 1.5707963267949 , 1.5707963267949 , 1.5707963267949}\"/>"
"                    <general_motion_0_i type=\"Marker\" active=\"true\" pe=\"{0.316 , 0.63 , 0 , -0 , 0 , -0}\"/>"
"                </marker_pool>"
"                <geometry_pool type=\"GeometryPoolElement\">"
"                    <solid type=\"ParasolidGeometry\" pe=\"{0 , 0 , 0 , -0 , 0 , -0}\" graphic_file_path=\"C:\\aris\\resource\\demo_hulk\\p6.xmt_txt\"/>"
"                </geometry_pool>"
"            </part_6>"
"        </part_pool>"
"        <joint_pool type=\"JointPoolElement\">"
"            <joint_0 type=\"RevoluteJoint\" active=\"true\" prt_m=\"ground\" prt_n=\"part_1\" mak_i=\"joint_0_i\" mak_j=\"joint_0_j\"/>"
"            <joint_1 type=\"RevoluteJoint\" active=\"true\" prt_m=\"part_1\" prt_n=\"part_2\" mak_i=\"joint_1_i\" mak_j=\"joint_1_j\"/>"
"            <joint_2 type=\"RevoluteJoint\" active=\"true\" prt_m=\"part_2\" prt_n=\"part_3\" mak_i=\"joint_2_i\" mak_j=\"joint_2_j\"/>"
"            <joint_3 type=\"RevoluteJoint\" active=\"true\" prt_m=\"part_3\" prt_n=\"part_4\" mak_i=\"joint_3_i\" mak_j=\"joint_3_j\"/>"
"            <joint_4 type=\"RevoluteJoint\" active=\"true\" prt_m=\"part_4\" prt_n=\"part_5\" mak_i=\"joint_4_i\" mak_j=\"joint_4_j\"/>"
"            <joint_5 type=\"RevoluteJoint\" active=\"true\" prt_m=\"part_5\" prt_n=\"part_6\" mak_i=\"joint_5_i\" mak_j=\"joint_5_j\"/>"
"        </joint_pool>"
"        <motion_pool type=\"MotionPoolElement\">"
"            <motion_0 type=\"Motion\" active=\"true\" prt_m=\"ground\" prt_n=\"part_1\" mak_i=\"joint_0_i\" mak_j=\"joint_0_j\" slave_id=\"0\" frc_coe=\"{0 , 0 , 0}\" component=\"5\"/>"
"            <motion_1 type=\"Motion\" active=\"true\" prt_m=\"part_1\" prt_n=\"part_2\" mak_i=\"joint_1_i\" mak_j=\"joint_1_j\" slave_id=\"1\" frc_coe=\"{0 , 0 , 0}\" component=\"5\"/>"
"            <motion_2 type=\"Motion\" active=\"true\" prt_m=\"part_2\" prt_n=\"part_3\" mak_i=\"joint_2_i\" mak_j=\"joint_2_j\" slave_id=\"2\" frc_coe=\"{0 , 0 , 0}\" component=\"5\"/>"
"            <motion_3 type=\"Motion\" active=\"true\" prt_m=\"part_3\" prt_n=\"part_4\" mak_i=\"joint_3_i\" mak_j=\"joint_3_j\" slave_id=\"3\" frc_coe=\"{0 , 0 , 0}\" component=\"5\"/>"
"            <motion_4 type=\"Motion\" active=\"true\" prt_m=\"part_4\" prt_n=\"part_5\" mak_i=\"joint_4_i\" mak_j=\"joint_4_j\" slave_id=\"4\" frc_coe=\"{0 , 0 , 0}\" component=\"5\"/>"
"            <motion_5 type=\"Motion\" active=\"true\" prt_m=\"part_5\" prt_n=\"part_6\" mak_i=\"joint_5_i\" mak_j=\"joint_5_j\" slave_id=\"5\" frc_coe=\"{0 , 0 , 0}\" component=\"5\"/>"
"        </motion_pool>"
"        <general_motion_pool type=\"GeneralMotionPoolElement\">"
"            <general_motion_0 type=\"GeneralMotion\" active=\"false\" prt_m=\"part_6\" prt_n=\"ground\" mak_i=\"general_motion_0_i\" mak_j=\"general_motion_0_j\"/>"
"        </general_motion_pool>"
"        <force_pool type=\"ForcePoolElement\"/>"
"        <solver_pool type=\"SolverPoolElement\">"
"            <inverse type=\"HulkInverseSolver\"/>"
"            <forward type=\"HulkForwardSolver\"/>"
"        </solver_pool>"
"        <simulator_pool type=\"SimulatorPoolElement\">"
"            <inv_sim type=\"AdamsSimulator\" solver=\"inverse\"/>"
"            <fwd_sim type=\"AdamsSimulator\" solver=\"forward\"/>"
"        </simulator_pool>"
"        <sim_result_pool type=\"SimResultPoolElement\">"
"        </sim_result_pool>"
"    </model>"
"    <sensor_root>"
"        <sensor_pool type=\"SensorPoolObject\"/>"
"    </sensor_root>"
"</root>";

using namespace std;
using namespace aris::dynamic;

class HulkInverseSolver :public aris::dynamic::DiagSolver
{
public:
	static const std::string& Type() { static const std::string type("HulkInverseSolver"); return type; }
	auto virtual type() const->const std::string& override{ return Type(); }
	auto virtual kinPos()->void override
	{
		auto &ee = model().generalMotionPool().front();

		double mpm[16];
		ee.getMpm(mpm);

		const double x = ee.mpm()[0][3];
		const double y = ee.mpm()[1][3];
		const double z = ee.mpm()[2][3];

		double l = std::sqrt(x * x + (y - 0.29)*(y - 0.29) + z*z);

		double q[6];
		q[0] = atan2(-z, x);
		q[1] = atan2(y - 0.29, std::sqrt(x * x + z * z)) + std::acos((l*l + 0.27*0.27 - 0.104756) / 0.27 / l / 2.0) - PI / 2;
		q[2] = std::acos((0.104756 + 0.27*0.27 - l*l) / 0.27 / 0.323660315763301 / 2.0) - 1.788795031482338;

		double first_3_rm[9], last_3_rm[9];
		double first_q[3]{ q[0],q[1] + q[2],0 };

		aris::dynamic::s_re2rm(first_q, first_3_rm, "232");
		s_mm(3, 3, 3, first_3_rm, ColMajor{ 3 }, *ee.mpm(), 4, last_3_rm, 3);

		s_rm2re(last_3_rm, q + 3, "131");

		if (q[3] > PI * 150 / 180) 
		{
			q[3] -= PI;
			q[4] = -q[4];
			q[5] -= PI;
		}



		double pm[16];
		double pe1[6]{ 0,0,0,0,q[0],0 };
		double pe2[6]{ 0,0.29,0,0,0,q[1] };
		double pe3[6]{ 0,0.56,0,0,0,q[2] };
		double pe4[6]{ 0,0.63,0,q[3],0,0 };
		double pe5[6]{ 0.316,0.63,0,0,0,q[4] };
		double pe6[6]{ 0.316,0.63,0,q[5],0,0 };

		s_pe2pm(pe1, pm, "123");
		s_mms(3, 1, 3, pm, 4, pe1, 1, pm + 3, 4);
		model().partPool().at(1).setPm(model().partPool().at(0), pm);

		s_pe2pm(pe2, pm, "123");
		s_mms(3, 1, 3, pm, 4, pe2, 1, pm + 3, 4);
		model().partPool().at(2).setPm(model().partPool().at(1), pm);

		s_pe2pm(pe3, pm, "123");
		s_mms(3, 1, 3, pm, 4, pe3, 1, pm + 3, 4);
		model().partPool().at(3).setPm(model().partPool().at(2), pm);

		s_pe2pm(pe4, pm, "123");
		s_mms(3, 1, 3, pm, 4, pe4, 1, pm + 3, 4);
		model().partPool().at(4).setPm(model().partPool().at(3), pm);

		s_pe2pm(pe5, pm, "123");
		s_mms(3, 1, 3, pm, 4, pe5, 1, pm + 3, 4);
		model().partPool().at(5).setPm(model().partPool().at(4), pm);

		s_pe2pm(pe6, pm, "123");
		s_mms(3, 1, 3, pm, 4, pe6, 1, pm + 3, 4);
		model().partPool().at(6).setPm(model().partPool().at(5), pm);
	}

	HulkInverseSolver(const std::string &name) :DiagSolver(name) {};
	HulkInverseSolver(Object &father, const aris::core::XmlElement &xml_ele) :DiagSolver(father, xml_ele) {};
};
class HulkForwardSolver :public aris::dynamic::DiagSolver
{
public:
	static const std::string& Type() { static const std::string type("HulkForwardSolver"); return type; }
	auto virtual type() const->const std::string& override{ return Type(); }
	auto virtual kinPos()->void override
	{
		double pm[16];
		double pe1[6]{ 0,0,0,0,-model().motionPool().at(0).mp(),0 };
		double pe2[6]{ 0,0.29,0,0,0,-model().motionPool().at(1).mp() };
		double pe3[6]{ 0,0.56,0,0,0,-model().motionPool().at(2).mp() };
		double pe4[6]{ 0,0.63,0,-model().motionPool().at(3).mp(),0,0 };
		double pe5[6]{ 0.316,0.63,0,0,0,-model().motionPool().at(4).mp() };
		double pe6[6]{ 0.316,0.63,0,-model().motionPool().at(5).mp(),0,0 };

		s_pe2pm(pe1, pm, "123");
		s_mms(3, 1, 3, pm, 4, pe1, 1, pm + 3, 4);
		model().partPool().at(1).setPm(model().partPool().at(0), pm);

		s_pe2pm(pe2, pm, "123");
		s_mms(3, 1, 3, pm, 4, pe2, 1, pm + 3, 4);
		model().partPool().at(2).setPm(model().partPool().at(1), pm);

		s_pe2pm(pe3, pm, "123");
		s_mms(3, 1, 3, pm, 4, pe3, 1, pm + 3, 4);
		model().partPool().at(3).setPm(model().partPool().at(2), pm);

		s_pe2pm(pe4, pm, "123");
		s_mms(3, 1, 3, pm, 4, pe4, 1, pm + 3, 4);
		model().partPool().at(4).setPm(model().partPool().at(3), pm);

		s_pe2pm(pe5, pm, "123");
		s_mms(3, 1, 3, pm, 4, pe5, 1, pm + 3, 4);
		model().partPool().at(5).setPm(model().partPool().at(4), pm);

		s_pe2pm(pe6, pm, "123");
		s_mms(3, 1, 3, pm, 4, pe6, 1, pm + 3, 4);
		model().partPool().at(6).setPm(model().partPool().at(5), pm);
	}

	HulkForwardSolver(const std::string &name) :DiagSolver(name) {};
	HulkForwardSolver(Object &father, const aris::core::XmlElement &xml_ele) :DiagSolver(father, xml_ele) {};
};

struct RcParam { double p[6]; };
auto rc_parse_func = [](const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg_out)->void
{
	RcParam param;
	param.p[0] = std::atof(params.at("p1").c_str());
	param.p[1] = std::atof(params.at("p2").c_str());
	param.p[2] = std::atof(params.at("p3").c_str());
	param.p[3] = std::atof(params.at("p4").c_str());
	param.p[4] = std::atof(params.at("p5").c_str());
	param.p[5] = std::atof(params.at("p6").c_str());
	msg_out.copyStruct(param);
};
auto rc_plan_func = [](const aris::dynamic::PlanParam &param)->int
{
	auto &cs = aris::server::ControlServer::instance();

	// 取得起始位置 //
	static double begin_pos[6];
	if (param.count_ == 1)
	{
		for (int i = 0; i < 6; ++i)
		{
			auto &mot = dynamic_cast<aris::control::EthercatMotion&>(cs.master().slavePool().at(cs.model().motionAtAbs(i).slaID()));
			begin_pos[i] = mot.actualPos();
		}
	}

	// 计算轨迹 //
	auto p = static_cast<RcParam*>(param.param_);
	static aris::Size total_count[6];
	for (int i = 0; i < 6; ++i)
	{
		auto &mot = dynamic_cast<aris::control::EthercatMotion&>(cs.master().slavePool().at(cs.model().motionAtAbs(i).slaID()));
		double mp, mv, ma;
		aris::dynamic::moveAbsolute(param.count_, begin_pos[i], p->p[i], 0.5*mot.maxVel() / 1000, 0.5*mot.maxVel() / 1000 / 1000, 0.5*mot.maxVel() / 1000 / 1000, mp, mv, ma, total_count[i]);
		cs.model().motionAtAbs(i).setMp(mp);
	}

	// 正解确定末端,这里先将角度转成弧度 //
	double mp[6];
	for (int i = 0; i < 6; ++i)
	{
		mp[i] = cs.model().motionAtAbs(i).mp();
		cs.model().motionAtAbs(i).setMp(mp[i] / 360 * 2.0 * PI);
	}
	cs.model().solverPool().at(1).kinPos();
	for (int i = 0; i < 6; ++i)cs.model().motionAtAbs(i).setMp(mp[i]);

	return (static_cast<int>(*std::max_element(total_count, total_count + 6)) > param.count_) ? 1 : 0;
};

struct CkParam { double p[6]; };
auto ck_parse_func = [](const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg_out)->void
{
	CkParam param;
	msg_out.header().reserved1_ = 0;
	msg_out.copyStruct(param);
};
auto ck_plan_func = [](const aris::dynamic::PlanParam &param)->int
{
	auto &cs = aris::server::ControlServer::instance();

	// 取得起始位置 //
	static double begin_pos[6];
	if (param.count_ == 1)
	{
		for (int i = 0; i < 6; ++i)
		{
			auto &mot = dynamic_cast<aris::control::EthercatMotion&>(cs.master().slavePool().at(cs.model().motionAtAbs(i).slaID()));
			begin_pos[i] = mot.actualPos();
		}
	}

	cs.master().mout() << "pos:" << begin_pos[0] << "  " << begin_pos[1] << "  " << begin_pos[2] << "  " << begin_pos[3] << "  " << begin_pos[4] << "  " << begin_pos[5] << '\n';

	return 0;
};

struct MvParam { int axis; double value; };
auto mv_parse_func = [](const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg_out)->void
{
	MvParam param;
	if (params.find("x") != params.end())
	{
		param.axis = 0;
		param.value = std::atof(params.at("x").c_str());
	}
	else if (params.find("y") != params.end())
	{
		param.axis = 1;
		param.value = std::atof(params.at("y").c_str());
	}
	else if (params.find("z") != params.end())
	{
		param.axis = 2;
		param.value = std::atof(params.at("z").c_str());
	}
	else if (params.find("a") != params.end())
	{
		param.axis = 3;
		param.value = std::atof(params.at("a").c_str());
	}
	else if (params.find("b") != params.end())
	{
		param.axis = 4;
		param.value = std::atof(params.at("b").c_str());
	}
	else if (params.find("c") != params.end())
	{
		param.axis = 5;
		param.value = std::atof(params.at("c").c_str());
	}
	else
	{
		throw std::runtime_error("invalid param in mv parse");
	}
	msg_out.copyStruct(param);
};
auto mv_plan_func = [](const aris::dynamic::PlanParam &param)->int
{
	auto &cs = aris::server::ControlServer::instance();
	auto p = static_cast<MvParam*>(param.param_);

	// 取得起始位置 //
	static double pe[6], vel[6]{ 0.1 , 0.1 , 0.1 , 10.0 , 10.0 , 10.0 }, acc[6]{ 0.1 , 0.1 , 0.1 , 10.0 , 10.0 , 10.0 }, begin_value;
	if (param.count_ == 1)
	{
		cs.model().generalMotionPool().front().updMpm();
		cs.model().generalMotionPool().front().getMpe(pe, "123");
		for (int i = 3; i < 6; ++i)	if (pe[i] > PI)pe[i] -= 2 * PI;


		begin_value = pe[p->axis];
	}

	// 计算轨迹 //
	static aris::Size total_count;
	double p_value, v, a;
	aris::dynamic::moveAbsolute(param.count_, begin_value, p->value, vel[p->axis] / 1000, acc[p->axis] / 1000 / 1000, acc[p->axis] / 1000 / 1000, p_value, v, a, total_count);
	pe[p->axis] = p_value;
	cs.model().generalMotionPool().front().setMpe(pe, "123");
	cs.model().solverPool().at(0).kinPos();
	for (int i = 0; i < 6; ++i)
	{
		cs.model().motionAtAbs(i).updMp();
		cs.model().motionAtAbs(i).setMp(cs.model().motionAtAbs(i).mp()*360/2.0/PI);
	}

	return total_count - param.count_;
};

int main()
{
	//_CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);

	try
	{
		aris::core::XmlDocument xml_doc;
		xml_doc.Parse(xml_data);

		auto&cs = aris::server::ControlServer::instance();
		cs.model().registerChildType<HulkInverseSolver>();
		cs.model().registerChildType<HulkForwardSolver>();
		cs.resetMaster(new aris::control::EthercatMaster);
		cs.loadXml(xml_doc);

		cs.widgetRoot().cmdParser().commandPool().add<aris::core::Command>(aris::server::default_enable_command());
		cs.widgetRoot().cmdParser().commandPool().add<aris::core::Command>(aris::server::default_disable_command());
		cs.widgetRoot().cmdParser().commandPool().add<aris::core::Command>(aris::server::default_mode_command());
		cs.widgetRoot().cmdParser().commandPool().add<aris::core::Command>(aris::server::default_home_command());
		cs.addCmd("en", aris::server::default_parse, aris::server::default_enable_plan);
		cs.addCmd("ds", aris::server::default_parse, aris::server::default_disable_plan);
		cs.addCmd("md", aris::server::default_parse, aris::server::default_mode_plan);
		cs.addCmd("hm", aris::server::default_parse, aris::server::default_home_plan);

		double mp[6]{ 0.135,0.246,-0.001,0.2,PI / 2,0.1 };
		//for (auto &m : cs.model().motionPool())
		//{
		//	m.setMp(mp[m.id()]);
		//}
		//
		//cs.model().solverPool().at(1).kinPos();

		double pe[6]{0.4,0.63,0,0,0,0};
		cs.model().generalMotionPool().front().setMpe(pe, "131");
		//cs.model().generalMotionPool().front().updMpm();
		//cs.model().generalMotionPool().front().getMpe(pe, "123");
		//dsp(1, 6, pe);

		cs.model().solverPool().at(0).kinPos();
		for (auto &m : cs.model().motionPool())
		{
			m.updMp();
			mp[m.id()]=m.mp();
		}
		dsp(1, 6, mp);

		cs.model().generalMotionPool().front().updMpm();
		cs.model().generalMotionPool().front().getMpe(pe, "123");
		//dsp(1, 6, pe);


		cs.addCmd("rc", rc_parse_func, rc_plan_func);
		cs.addCmd("mv", mv_parse_func, mv_plan_func);
		cs.addCmd("ck", ck_parse_func, ck_plan_func);

		cs.start();


		std::atomic_bool flag{ true };
		// 接收并打印信息 //
		auto t = std::thread([&]()
		{
			for (; flag;)
			{
				aris::core::Msg msg;
				cs.master().recvOut(msg);
				if (!msg.empty())std::cout << msg.data() << std::endl;

				std::this_thread::sleep_for(std::chrono::milliseconds(10));
			}
		});

		// 接收命令 //
		for (std::string command_in; std::getline(std::cin, command_in);)
		{
			if (command_in == "exit")break;
			
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
		
		
		
		
		
		
		//double pm[16]{ 1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1 };
		//aris::dynamic::Model m;
		//





		//auto &p1 = m.addPartByPm(pm);
		//auto &p2 = m.addPartByPm(pm);
		//auto &p3 = m.addPartByPm(pm);
		//auto &p4 = m.addPartByPm(pm);
		//auto &p5 = m.addPartByPm(pm);
		//auto &p6 = m.addPartByPm(pm);
		//p1.geometryPool().add<ParasolidGeometry>("solid", "C:\\aris\\resource\\demo_hulk\\p1.xmt_txt");
		//p2.geometryPool().add<ParasolidGeometry>("solid", "C:\\aris\\resource\\demo_hulk\\p2.xmt_txt");
		//p3.geometryPool().add<ParasolidGeometry>("solid", "C:\\aris\\resource\\demo_hulk\\p3.xmt_txt");
		//p4.geometryPool().add<ParasolidGeometry>("solid", "C:\\aris\\resource\\demo_hulk\\p4.xmt_txt");
		//p5.geometryPool().add<ParasolidGeometry>("solid", "C:\\aris\\resource\\demo_hulk\\p5.xmt_txt");
		//p6.geometryPool().add<ParasolidGeometry>("solid", "C:\\aris\\resource\\demo_hulk\\p6.xmt_txt");
		//auto &j1 = m.addRevoluteJoint(m.ground(), p1, std::array<double, 3>{0, 0, 0}.data(), std::array<double, 3>{0, 1, 0}.data());
		//auto &j2 = m.addRevoluteJoint(p1, p2, std::array<double, 3>{0, 0.29, 0}.data(), std::array<double, 3>{0, 0, 1}.data());
		//auto &j3 = m.addRevoluteJoint(p2, p3, std::array<double, 3>{0, 0.56, 0}.data(), std::array<double, 3>{0, 0, 1}.data());
		//auto &j4 = m.addRevoluteJoint(p3, p4, std::array<double, 3>{0, 0.63, 0}.data(), std::array<double, 3>{1, 0, 0}.data());
		//auto &j5 = m.addRevoluteJoint(p4, p5, std::array<double, 3>{0.316, 0.63, 0}.data(), std::array<double, 3>{0, 0, 1}.data());
		//auto &j6 = m.addRevoluteJoint(p5, p6, std::array<double, 3>{0.316, 0.63, 0}.data(), std::array<double, 3>{1, 0, 0}.data());
		//auto &m1 = m.addMotion(j1);
		//auto &m2 = m.addMotion(j2);
		//auto &m3 = m.addMotion(j3);
		//auto &m4 = m.addMotion(j4);
		//auto &m5 = m.addMotion(j5);
		//auto &m6 = m.addMotion(j6);

		//double ee_pm[16]{ 1,0,0,0.316,0,1,0,0.63,0,0,1,0,0,0,0,1 };
		//auto &ee = m.addGeneralMotion(p6, m.ground(), ee_pm);

		//auto &ds = m.solverPool().add<aris::dynamic::DiagSolver>("ds");
		//auto &s = m.simulatorPool().add<aris::dynamic::AdamsSimulator>("s", ds);
		//auto &r = m.simResultPool().add<aris::dynamic::SimResult>("r");
		//auto plan1 = [](const aris::dynamic::PlanParam &param)->int
		//{
		//	std::array<double, 6> ee_pe{ 0.316, 0.63, param.count_ * 0.0001, 0, 0, 0 };
		//	param.model_->generalMotionPool().at(0).setMpe(ee_pe.data());
		//	param.model_->setTime(param.count_ * 0.001);
		//	return 3000 - param.count_;
		//};
		//auto plan2 = [](const aris::dynamic::PlanParam &param)->int
		//{
		//	const double dt{0.01};
		//	
		//	aris::Size total_coult;
		//	double p, v, a;
		//	aris::dynamic::moveAbsolute(param.count_, 0.0, PI, 0.5 * dt, 0.3 * dt * dt, 0.5 * dt * dt, p, v, a, total_coult);
		//	
		//	param.model_->setTime(param.count_ * dt);
		//	param.model_->motionPool().at(0).setMp(p);
		//	return total_coult - param.count_;
		//};

		//m.registerChildType<HulkInverseSolver>();
		//m.registerChildType<HulkForwardSolver>();
		//auto &inv = m.solverPool().add<HulkInverseSolver>("inverse");
		//auto &fwd = m.solverPool().add<HulkForwardSolver>("forward");
		//auto &inv_sim = m.simulatorPool().add<aris::dynamic::AdamsSimulator>("inv_sim", inv);
		//auto &fwd_sim = m.simulatorPool().add<aris::dynamic::AdamsSimulator>("fwd_sim", fwd);

		//for (auto &mot : m.motionPool())mot.activate(true);
		//m.generalMotionPool().front().activate(false);

		//fwd_sim.simulate(plan2, 0, r);

		//fwd_sim.saveAdams("C:\\Users\\py033\\Desktop\\hulk.cmd", r);
		//m.saveXml("C:\\Users\\py033\\Desktop\\hulk.xml");

		cs.stop();
		flag = false;
		t.join();
		std::cout << "exit before error" << std::endl;
	}
	catch (std::exception &e)
	{
		std::cout << e.what() << std::endl;
	}
	std::cout << "demo_hulk finished, press any key to continue" << std::endl;
	std::cin.get();

	return 0;
}

