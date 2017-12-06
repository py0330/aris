#include "test_dynamic_model_compute.h"
#include <iostream>
#include <aris_dynamic.h>

#include<type_traits>

using namespace aris::dynamic;

const char xml_file_under_constraint[] =
"<model>"
"    <environment type=\"Environment\" gravity=\"{0,-9.8,0,0,0,0}\"/>"
"    <variable_pool type=\"VariablePoolElement\" default_child_type=\"Matrix\">"
"        <PI type=\"MatrixVariable\">3.14159265358979</PI>"
"        <Mot_friction type=\"MatrixVariable\">{0, 0, 0}</Mot_friction>"
"    </variable_pool>"
"    <part_pool type=\"PartPoolElement\" default_child_type=\"Part\">"
"        <ground active=\"true\" inertia=\"{1,0,0,0,1,1,1,0,0,0}\" pe=\"{0,0,0,0,0,0}\" vel=\"{0,0,0,0,0,0}\" acc=\"{0,0,0,0,0,0}\" graphic_file_path=\"\">"
"            <marker_pool type=\"MarkerPoolElement\" default_child_type=\"Marker\">"
"                <origin pe=\"{ 0,0,0,0,0,0 }\"/>"
"                <r1j pe=\"{ 0,0,0,0,0,0 }\"/>"
"                <r4i pe=\"{ 1.5,1,0,0,0,0 }\"/>"
"            </marker_pool>"
"        </ground>"
"        <part1 active=\"true\" inertia=\"{1,0,0,0,1,1,1,0,0,0}\" pe=\"{0,0,0,0,0,0}\" vel=\"{0,0,0,0,0,0}\" acc=\"{0,0,0,0,0,0}\" graphic_file_path=\"C:\\aris\\robot\\resource\\graphic_file\\part1.x_t\">"
"            <marker_pool type=\"MarkerPoolElement\" default_child_type=\"Marker\">"
"                <r1i pe=\"{ 0,0,0,0,0,0 }\"/>"
"                <r2j pe=\"{ 1,0,0,0,0,0 }\"/>"
"            </marker_pool>"
"            <geometry_pool type=\"GeometryPoolElement\">"
"                <solid type=\"ParasolidGeometry\" graphic_file_path=\"C:\\aris\\resource\\test_dynamic\\3R\\part1.x_t\"/>"
"            </geometry_pool>"
"        </part1>"
"        <part2 active=\"true\" inertia=\"{1,0,0,0,1,1,1,0,0,0}\" pe=\"{1,0,0,PI/2,0,0}\" vel=\"{0,0,0,0,0,0}\" acc=\"{0,0,0,0,0,0}\" graphic_file_path=\"C:\\aris\\robot\\resource\\graphic_file\\part2.x_t\">"
"            <marker_pool type=\"MarkerPoolElement\" default_child_type=\"Marker\">"
"                <r2i pe=\"{ 0,0,0,0,0,0 }\"/>"
"                <r3j pe=\"{ 1,0,0,0,0,0 }\"/>"
"            </marker_pool>"
"            <geometry_pool type=\"GeometryPoolElement\">"
"                <solid type=\"ParasolidGeometry\" graphic_file_path=\"C:\\aris\\resource\\test_dynamic\\3R\\part2.x_t\"/>"
"            </geometry_pool>"
"        </part2>"
"        <part3 active=\"true\" inertia=\"{1,0,0,0,1,1,1,0,0,0}\" pe=\"{1,1,0,0,0,0}\" vel=\"{0,0,0,0,0,0}\" acc=\"{0,0,0,0,0,0}\">"
"            <marker_pool type=\"MarkerPoolElement\" default_child_type=\"Marker\">"
"                <r3i pe=\"{ 0,0,0,0,0,0 }\"/>"
"                <r4j pe=\"{ 0.5,0,0,0,0,0 }\"/>"
"            </marker_pool>"
"            <geometry_pool type=\"GeometryPoolElement\">"
"                <solid type=\"ParasolidGeometry\" graphic_file_path=\"C:\\aris\\resource\\test_dynamic\\3R\\part3.x_t\"/>"
"            </geometry_pool>"
"        </part3>"
"    </part_pool>"
"    <joint_pool type=\"JointPoolElement\">"
"        <r1 active=\"true\" type=\"RevoluteJoint\" prt_m=\"part1\" prt_n=\"ground\" mak_i=\"r1i\" mak_j=\"r1j\"/>"
"        <r2 active=\"true\" type=\"RevoluteJoint\" prt_m=\"part2\" prt_n=\"part1\" mak_i=\"r2i\" mak_j=\"r2j\"/>"
"        <r3 active=\"true\" type=\"RevoluteJoint\" prt_m=\"part3\" prt_n=\"part2\" mak_i=\"r3i\" mak_j=\"r3j\"/>"
"        <r4 active=\"true\" type=\"RevoluteJoint\" prt_m=\"ground\" prt_n=\"part3\" mak_i=\"r4i\" mak_j=\"r4j\"/>"
"    </joint_pool>"
"    <motion_pool type=\"MotionPoolElement\" default_child_type=\"Motion\">"
"    </motion_pool>"
"    <general_motion_pool type=\"GeneralMotionPoolElement\" default_child_type=\"GeneralMotion\">"
"    </general_motion_pool>"
"    <solver_pool type=\"SolverPoolElement\" default_child_type=\"Solver\">"
"        <gs type=\"LltGroundDividedSolver\"/>"
"        <ps type=\"LltPartDividedSolver\"/>"
"        <ds type=\"UniversalSolver\"/>"
"    </solver_pool>"
"</model>";

const char xml_file_3R[] =
"<model>"
"    <environment type=\"Environment\" gravity=\"{0,-9.8,0,0,0,0}\"/>"
"    <variable_pool type=\"VariablePoolElement\" default_child_type=\"Matrix\">"
"        <PI type=\"MatrixVariable\">3.14159265358979</PI>"
"        <Mot_friction type=\"MatrixVariable\">{0, 0, 0}</Mot_friction>"
"    </variable_pool>"
"    <part_pool type=\"PartPoolElement\" default_child_type=\"Part\">"
"        <ground active=\"true\" inertia=\"{1,0,0,0,1,1,1,0,0,0}\" pe=\"{0,0,0,0,0,0}\" vel=\"{0,0,0,0,0,0}\" acc=\"{0,0,0,0,0,0}\" graphic_file_path=\"\">"
"            <marker_pool type=\"MarkerPoolElement\" default_child_type=\"Marker\">"
"                <origin pe=\"{ 0,0,0,0,0,0 }\"/>"
"                <r1j pe=\"{ 0,0,0,0,0,0 }\"/>"
"            </marker_pool>"
"        </ground>"
"        <part1 active=\"true\" inertia=\"{1,0,0,0,1,1,1,0,0,0}\" pe=\"{0,0,0,0,0,0}\" vel=\"{0,0,0,0,0,0}\" acc=\"{0,0,0,0,0,0}\" graphic_file_path=\"C:\\aris\\robot\\resource\\graphic_file\\part1.x_t\">"
"            <marker_pool type=\"MarkerPoolElement\" default_child_type=\"Marker\">"
"                <r1i pe=\"{ 0,0,0,0,0,0 }\"/>"
"                <r2j pe=\"{ 1,0,0,0,0,0 }\"/>"
"            </marker_pool>"
"            <geometry_pool type=\"GeometryPoolElement\">"
"                <solid type=\"ParasolidGeometry\" graphic_file_path=\"C:\\aris\\resource\\test_dynamic\\3R\\part1.x_t\"/>"
"            </geometry_pool>"
"        </part1>"
"        <part2 active=\"true\" inertia=\"{1,0,0,0,1,1,1,0,0,0}\" pe=\"{1,0,0,PI/2,0,0}\" vel=\"{0,0,0,0,0,0}\" acc=\"{0,0,0,0,0,0}\" graphic_file_path=\"C:\\aris\\robot\\resource\\graphic_file\\part2.x_t\">"
"            <marker_pool type=\"MarkerPoolElement\" default_child_type=\"Marker\">"
"                <r2i pe=\"{ 0,0,0,0,0,0 }\"/>"
"                <r3j pe=\"{ 1,0,0,0,0,0 }\"/>"
"            </marker_pool>"
"            <geometry_pool type=\"GeometryPoolElement\">"
"                <solid type=\"ParasolidGeometry\" graphic_file_path=\"C:\\aris\\resource\\test_dynamic\\3R\\part2.x_t\"/>"
"            </geometry_pool>"
"        </part2>"
"        <part3 active=\"true\" inertia=\"{1,0,0,0,1,1,1,0,0,0}\" pe=\"{1,1,0,0.2,0.5,0}\" vel=\"{0,0,0,0,0,0}\" acc=\"{0,0,0,0,0,0}\">"
"            <marker_pool type=\"MarkerPoolElement\" default_child_type=\"Marker\">"
"                <r3i pe=\"{ 0,0,0,0,0,0 }\"/>"
"                <ee pe=\"{ 1,0,0,0,0,0 }\"/>"
"            </marker_pool>"
"            <geometry_pool type=\"GeometryPoolElement\">"
"                <solid type=\"ParasolidGeometry\" graphic_file_path=\"C:\\aris\\resource\\test_dynamic\\3R\\part3.x_t\"/>"
"            </geometry_pool>"
"        </part3>"
"        <part4 active=\"false\" inertia=\"{1,0,0,0,1,1,1,0,0,0}\" pe=\"{1,1,0,0.2,0.5,0}\" vel=\"{0,0,0,0,0,0}\" acc=\"{0,0,0,0,0,0}\">"
"            <geometry_pool type=\"GeometryPoolElement\">"
"                <solid type=\"ParasolidGeometry\" graphic_file_path=\"C:\\aris\\resource\\test_dynamic\\3R\\part3.x_t\"/>"
"            </geometry_pool>"
"        </part4>"
"    </part_pool>"
"    <joint_pool type=\"JointPoolElement\">"
"        <r1 active=\"true\" type=\"RevoluteJoint\" prt_m=\"part1\" prt_n=\"ground\" mak_i=\"r1i\" mak_j=\"r1j\"/>"
"        <r2 active=\"true\" type=\"RevoluteJoint\" prt_m=\"part2\" prt_n=\"part1\" mak_i=\"r2i\" mak_j=\"r2j\"/>"
"        <r3 active=\"true\" type=\"RevoluteJoint\" prt_m=\"part3\" prt_n=\"part2\" mak_i=\"r3i\" mak_j=\"r3j\"/>"
"        <r4 active=\"false\" type=\"RevoluteJoint\" prt_m=\"part3\" prt_n=\"part2\" mak_i=\"r3i\" mak_j=\"r3j\"/>"
"    </joint_pool>"
"    <motion_pool type=\"MotionPoolElement\" default_child_type=\"Motion\">"
"        <m1 active=\"true\" slave_id=\"0\" prt_m=\"part1\" prt_n=\"ground\" mak_i=\"r1i\" mak_j=\"r1j\" frc_coe=\"Mot_friction\" component=\"5\"/>"
"        <m2 active=\"true\" slave_id=\"1\" prt_m=\"part2\" prt_n=\"part1\" mak_i=\"r2i\" mak_j=\"r2j\" frc_coe=\"Mot_friction\" component=\"5\"/>"
"        <m3 active=\"true\" slave_id=\"2\" prt_m=\"part3\" prt_n=\"part2\" mak_i=\"r3i\" mak_j=\"r3j\" frc_coe=\"Mot_friction\" component=\"5\"/>"
"    </motion_pool>"
"    <general_motion_pool type=\"GeneralMotionPoolElement\" default_child_type=\"GeneralMotion\">"
"        <ee_mot type=\"GeneralMotion\" active=\"true\" prt_m=\"part3\" prt_n=\"ground\" mak_i=\"ee\" mak_j=\"origin\"/>"
"    </general_motion_pool>"
"    <solver_pool type=\"SolverPoolElement\" default_child_type=\"Solver\">"
"        <gs type=\"LltGroundDividedSolver\"/>"
"        <ps type=\"LltPartDividedSolver\"/>"
"        <ds type=\"UniversalSolver\"/>"
"    </solver_pool>"
"</model>";

const char xml_file_6R[] =
"<model>"
"    <environment type=\"Environment\" gravity=\"{0 , -9.8 , 0 , 0 , 0 , 0}\"/>"
"    <variable_pool type=\"VariablePoolElement\"/>"
"    <part_pool type=\"PartPoolElement\">"
"        <ground type=\"Part\" active=\"true\" pe=\"{0 , 0 , 0 , -0 , 0 , -0}\" vel=\"{0 , 0 , 0 , 0 , 0 , 0}\" acc=\"{0 , 0 , 0 , 0 , 0 , 0}\" inertia=\"{1 , 0 , 0 , 0 , 1 , 1 , 1 , 0 , 0 , 0}\">"
"            <marker_pool type=\"MarkerPoolElement\">"
"                <R0j type=\"Marker\" active=\"true\" pe=\"{0.1 , 0.2 , 0.3 , 2.64224593190966 , 0.649484790532536 , 5.12219242612033}\"/>"
"                <origin type=\"Marker\" active=\"true\" pe=\"{0 , 0 , 0 , -0 , 0 , -0}\"/>"
"            </marker_pool>"
"        </ground>"
"        <part0 type=\"Part\" active=\"true\" pe=\"{0.1 , 0.2 , 0.3 , 2.64224593190966 , 0.649484790532536 , 5.12219242612033}\" vel=\"{0 , 0 , 0 , 0 , 0 , 0}\" acc=\"{0 , 0 , 0 , 0 , 0 , 0}\" inertia=\"{1 , 0 , 0 , 0 , 1 , 1 , 1 , 0 , 0 , 0}\">"
"            <marker_pool type=\"MarkerPoolElement\">"
"                <R0i type=\"Marker\" active=\"true\" pe=\"{0 , 0 , 0 , -0 , 0 , -0}\"/>"
"                <R1j type=\"Marker\" active=\"true\" pe=\"{0.127581045131803 , -0.255551891814505 , 0.743785121868611 , 1.39801111882379 , 0.92874853281329 , 2.99910369947683}\"/>"
"            </marker_pool>"
"        </part0>"
"        <part1 type=\"Part\" active=\"true\" pe=\"{0.56 , 0.66 , 0.76 , 2.83142459280181 , 1.5646920289751 , 3.14159265358979}\" vel=\"{0 , 0 , 0 , 0 , 0 , 0}\" acc=\"{0 , 0 , 0 , 0 , 0 , 0}\" inertia=\"{1 , 0 , 0 , 0 , 1 , 1 , 1 , 0 , 0 , 0}\">"
"            <marker_pool type=\"MarkerPoolElement\">"
"                <R1i type=\"Marker\" active=\"true\" pe=\"{0 , 0 , 0 , -0 , 0 , -0}\"/>"
"                <R2j type=\"Marker\" active=\"true\" pe=\"{0 , 0 , 0 , 6.21426444075139 , 1.09203109440023 , 1.87858662805362}\"/>"
"            </marker_pool>"
"        </part1>"
"        <part2 type=\"Part\" active=\"true\" pe=\"{0.56 , 0.66 , 0.76 , 2.96488379751976 , 0.477268629930651 , 4.86969507322217}\" vel=\"{0 , 0 , 0 , 0 , 0 , 0}\" acc=\"{0 , 0 , 0 , 0 , 0 , 0}\" inertia=\"{1 , 0 , 0 , 0 , 1 , 1 , 1 , 0 , 0 , 0}\">"
"            <marker_pool type=\"MarkerPoolElement\">"
"                <R2i type=\"Marker\" active=\"true\" pe=\"{0 , 0 , 0 , -0 , 0 , -0}\"/>"
"                <R3j type=\"Marker\" active=\"true\" pe=\"{0 , -0.996734365258792 , 0.0807502638519182 , 1.5707963267949 , 7.66425882286152e-17 , 4.71238898038469}\"/>"
"            </marker_pool>"
"        </part2>"
"        <part3 type=\"Part\" active=\"true\" pe=\"{1.56 , 0.66 , 0.76 , 2.96488379751976 , 0.477268629930651 , 4.86969507322218}\" vel=\"{0 , 0 , 0 , 0 , 0 , 0}\" acc=\"{0 , 0 , 0 , 0 , 0 , 0}\" inertia=\"{1 , 0 , 0 , 0 , 1 , 1 , 1 , 0 , 0 , 0}\">"
"            <marker_pool type=\"MarkerPoolElement\">"
"                <R3i type=\"Marker\" active=\"true\" pe=\"{0 , 0 , 0 , -0 , 0 , -0}\"/>"
"                <R4j type=\"Marker\" active=\"true\" pe=\"{1.53280055883457 , 0.0630122434114775 , 0.777786541421676 , 4.71238898038469 , 1.11022302462516e-16 , 1.5707963267949}\"/>"
"            </marker_pool>"
"        </part3>"
"        <part4 type=\"Part\" active=\"true\" pe=\"{1.56 , 2.38 , 0.76 , 2.96488379751976 , 0.477268629930651 , 4.86969507322218}\" vel=\"{0 , 0 , 0 , 0 , 0 , 0}\" acc=\"{0 , 0 , 0 , 0 , 0 , 0}\" inertia=\"{1 , 0 , 0 , 0 , 1 , 1 , 1 , 0 , 0 , 0}\">"
"            <marker_pool type=\"MarkerPoolElement\">"
"                <R4i type=\"Marker\" active=\"true\" pe=\"{0 , 0 , 0 , -0 , 0 , -0}\"/>"
"                <R5j type=\"Marker\" active=\"true\" pe=\"{0 , 0 , 0 , 0.299854573312955 , 1.27790273658208 , 3.56732345865295}\"/>"
"            </marker_pool>"
"        </part4>"
"        <part5 type=\"Part\" active=\"true\" pe=\"{1.56 , 2.38 , 0.76 , 1.928105009803 , 1.5084212451233 , 3.14159265358979}\" vel=\"{0 , 0 , 0 , 0 , 0 , 0}\" acc=\"{0 , 0 , 0 , 0 , 0 , 0}\" inertia=\"{1 , 0 , 0 , 0 , 1 , 1 , 1 , 0 , 0 , 0}\">"
"            <marker_pool type=\"MarkerPoolElement\">"
"                <R5i type=\"Marker\" active=\"true\" pe=\"{0 , 0 , 0 , -0 , 0 , -0}\"/>"
"                <end_effector type=\"Marker\" active=\"true\" pe=\"{1.6840663068739 , 0.615533362716626 , -2.33680109411025 , -0 , 1.5084212451233 , 1.2134876437868}\"/>"
"            </marker_pool>"
"        </part5>"
"    </part_pool>"
"    <joint_pool type=\"JointPoolElement\">"
"        <R0 type=\"RevoluteJoint\" active=\"true\" prt_m=\"part0\" prt_n=\"ground\" mak_i=\"R0i\" mak_j=\"R0j\"/>"
"        <R1 type=\"RevoluteJoint\" active=\"true\" prt_m=\"part1\" prt_n=\"part0\" mak_i=\"R1i\" mak_j=\"R1j\"/>"
"        <R2 type=\"RevoluteJoint\" active=\"true\" prt_m=\"part2\" prt_n=\"part1\" mak_i=\"R2i\" mak_j=\"R2j\"/>"
"        <R3 type=\"RevoluteJoint\" active=\"true\" prt_m=\"part3\" prt_n=\"part2\" mak_i=\"R3i\" mak_j=\"R3j\"/>"
"        <R4 type=\"RevoluteJoint\" active=\"true\" prt_m=\"part4\" prt_n=\"part3\" mak_i=\"R4i\" mak_j=\"R4j\"/>"
"        <R5 type=\"RevoluteJoint\" active=\"true\" prt_m=\"part5\" prt_n=\"part4\" mak_i=\"R5i\" mak_j=\"R5j\"/>"
"    </joint_pool>"
"    <motion_pool type=\"MotionPoolElement\">"
"        <M0 type=\"Motion\" active=\"true\" prt_m=\"part0\" prt_n=\"ground\" mak_i=\"R0i\" mak_j=\"R0j\" frc_coe=\"{0 , 0 , 0}\" component=\"5\"/>"
"        <M1 type=\"Motion\" active=\"true\" prt_m=\"part1\" prt_n=\"part0\" mak_i=\"R1i\" mak_j=\"R1j\" frc_coe=\"{0 , 0 , 0}\" component=\"5\"/>"
"        <M2 type=\"Motion\" active=\"true\" prt_m=\"part2\" prt_n=\"part1\" mak_i=\"R2i\" mak_j=\"R2j\" frc_coe=\"{0 , 0 , 0}\" component=\"5\"/>"
"        <M3 type=\"Motion\" active=\"true\" prt_m=\"part3\" prt_n=\"part2\" mak_i=\"R3i\" mak_j=\"R3j\" frc_coe=\"{0 , 0 , 0}\" component=\"5\"/>"
"        <M4 type=\"Motion\" active=\"true\" prt_m=\"part4\" prt_n=\"part3\" mak_i=\"R4i\" mak_j=\"R4j\" frc_coe=\"{0 , 0 , 0}\" component=\"5\"/>"
"        <M5 type=\"Motion\" active=\"true\" prt_m=\"part5\" prt_n=\"part4\" mak_i=\"R5i\" mak_j=\"R5j\" frc_coe=\"{0 , 0 , 0}\" component=\"5\"/>"
"    </motion_pool>"
"    <general_motion_pool type=\"GeneralMotionPoolElement\">"
"        <ee_mot type=\"GeneralMotion\" active=\"true\" prt_m=\"part5\" prt_n=\"ground\" mak_i=\"end_effector\" mak_j=\"origin\"/>"
"    </general_motion_pool>"
"    <force_pool type=\"ForcePoolElement\"/>"
"    <solver_pool type=\"SolverPoolElement\" default_child_type=\"Solver\">"
"        <ds type=\"UniversalSolver\" max_iter_count=\"100\" max_error=\"1e-14\"/>"
"        <gs type=\"LltGroundDividedSolver\" max_iter_count=\"100\" max_error=\"1e-14\"/>"
"        <ps type=\"LltPartDividedSolver\" max_iter_count=\"100\" max_error=\"1e-14\"/>"
"    </solver_pool>"
"</model>";

const char xml_file_stewart[] =
"<model>"
"    <environment type=\"Environment\" gravity=\"{0,-9.8,0,0,0,0}\"/>"
"    <variable_pool type=\"VariablePoolElement\" default_child_type=\"Matrix\">"
"        <PI type=\"MatrixVariable\">3.14159265358979</PI>"
"        <Mot_friction type=\"MatrixVariable\">{0, 0, 0}</Mot_friction>"
"    </variable_pool>"
"    <part_pool type=\"PartPoolElement\" default_child_type=\"Part\">"
"        <p1a active=\"true\" inertia=\"{1 , -0.62 , 0.13 , -0.58 , 105.0 , 116.25 , 100.28 , 20.11015 , 12.2000345614 , 0.58539}\" pe=\"{0.999999999999974 , 1.22522177619812e-16 , -9.28869564848867e-18 , 6.38378239159465e-16 , 0.546497081697639 , 0.486611302448734}\" vel=\"{0,0,0,0,0,0}\" acc=\"{0,0,0,0,0,0}\">"
"            <marker_pool type=\"MarkerPoolElement\" default_child_type=\"Marker\">"
"                <u1i pe=\"{ 0,0,0,-PI/2,0,0 }\"/>"
"                <p1j pe=\"{ 0,0,0,0,-PI/2,0 }\"/>"
"            </marker_pool>"
"            <geometry_pool type=\"GeometryPoolElement\">"
"                <solid type=\"ParasolidGeometry\" graphic_file_path=\"C:\\aris\\resource\\test_dynamic\\stewart\\pa.xmt_txt\"/>"
"            </geometry_pool>"
"        </p1a>"
"        <p1b active=\"true\" inertia=\"{1 , -0.12 , 0.53 , -0.58 , 15.0 , 16.25 , 100.28 , 10.11015 , 12.2000345614 , 0.58539}\" pe=\"{0.0711481425892889 , 1.49999999999963 , 0.912443796234424 , 8.04911692853238e-16 , 0.546497081697639 , 0.486611302448734}\" vel=\"{0,0,0,0,0,0}\" acc=\"{0,0,0,0,0,0}\">"
"            <marker_pool type=\"MarkerPoolElement\" default_child_type=\"Marker\">"
"                <p1i pe=\"{ 0,0,0,0,-PI/2,0 }\"/>"
"                <s1j pe=\"{ 0,0,0,0,0,0 }\"/>"
"            </marker_pool>"
"            <geometry_pool type=\"GeometryPoolElement\">"
"                <solid type=\"ParasolidGeometry\" graphic_file_path=\"C:\\aris\\resource\\test_dynamic\\stewart\\pb.xmt_txt\"/>"
"            </geometry_pool>"
"        </p1b>"
"        <p2a active=\"true\" inertia=\"{1 , -0.622 , 0.113 , -0.538 , 105.2 , 116.75 , 100.88 , 21.11015 , 11.2000345614 , 1.58539}\" pe=\"{0.999999999999995 , 1.22524189323061e-16 , -9.2876368573046e-18 , 5.55111512312578e-17 , 0.721024145526766 , 0.308719565228027}\" vel=\"{0,0,0,0,0,0}\" acc=\"{0,0,0,0,0,0}\">"
"            <marker_pool type=\"MarkerPoolElement\" default_child_type=\"Marker\">"
"                <u2i pe=\"{ 0,0,0,-PI/2,0,0 }\"/>"
"                <p2j pe=\"{ 0,0,0,0,-PI/2,0 }\"/>"
"            </marker_pool>"
"            <geometry_pool type=\"GeometryPoolElement\">"
"                <solid type=\"ParasolidGeometry\" graphic_file_path=\"C:\\aris\\resource\\test_dynamic\\stewart\\pa.xmt_txt\"/>"
"            </geometry_pool>"
"        </p2a>"
"        <p2b active=\"true\" inertia=\"{1 , -0.621 , 0.131 , -0.581 , 105.1 , 116.251 , 100.281 , 20.110151 , 12.20003 , 0.585391}\" pe=\"{0.363127053316677 , 1.49999999999988 , 1.31832224563822 , 6.28318530717959 , 0.721024145526766 , 0.308719565228028}\" vel=\"{0,0,0,0,0,0}\" acc=\"{0,0,0,0,0,0}\">"
"            <marker_pool type=\"MarkerPoolElement\" default_child_type=\"Marker\">"
"                <p2i pe=\"{ 0,0,0,0,-PI/2,0 }\"/>"
"                <s2j pe=\"{ 0,0,0,0,0,0 }\"/>"
"            </marker_pool>"
"            <geometry_pool type=\"GeometryPoolElement\">"
"                <solid type=\"ParasolidGeometry\" graphic_file_path=\"C:\\aris\\resource\\test_dynamic\\stewart\\pb.xmt_txt\"/>"
"            </geometry_pool>"
"        </p2b>"
"        <p3a active=\"true\" inertia=\"{1.82156 , -0.621 , 0.131 , -0.581 , 105.1 , 116.251 , 100.281 , 20.110151 , 12.20003 , 0.585391}\" pe=\"{1.24902578429613e-16 , 3.6066466064807e-14 , 1.73199999999999 , 3.14159265358979 , 0.269096030174962 , 2.91232360862124}\" vel=\"{0,0,0,0,0,0}\" acc=\"{0,0,0,0,0,0}\">"
"            <marker_pool type=\"MarkerPoolElement\" default_child_type=\"Marker\">"
"                <u3i pe=\"{ 0,0,0,-PI/2,0,0 }\"/>"
"                <p3j pe=\"{ 0,0,0,0,-PI/2,0 }\"/>"
"            </marker_pool>"
"            <geometry_pool type=\"GeometryPoolElement\">"
"                <solid type=\"ParasolidGeometry\" graphic_file_path=\"C:\\aris\\resource\\test_dynamic\\stewart\\pa.xmt_txt\"/>"
"            </geometry_pool>"
"        </p3a>"
"        <p3b active=\"true\" inertia=\"{0.82156 , -0.6221 , 0.1312 , -0.5812 , 105.12 , 116.2512 , 100.2812 , 20.1101512 , 12.3 , 0.5853912}\" pe=\"{0.363127053316337 , 1.49999999999935 , 1.31832224563851 , 3.14159265358979 , 0.269096030174962 , 2.91232360862124}\" vel=\"{0,0,0,0,0,0}\" acc=\"{0,0,0,0,0,0}\">"
"            <marker_pool type=\"MarkerPoolElement\" default_child_type=\"Marker\">"
"                <p3i pe=\"{ 0,0,0,0,-PI/2,0 }\"/>"
"                <s3j pe=\"{ 0,0,0,0,0,0 }\"/>"
"            </marker_pool>"
"            <geometry_pool type=\"GeometryPoolElement\">"
"                <solid type=\"ParasolidGeometry\" graphic_file_path=\"C:\\aris\\resource\\test_dynamic\\stewart\\pb.xmt_txt\"/>"
"            </geometry_pool>"
"        </p3b>"
"        <p4a active=\"true\" inertia=\"{1.52156 , -0.521 , 0.231 , -0.481 , 115.1 , 106.251 , 110.281 , 21.110151 , 13.20003 , 0.555391}\" pe=\"{1.24898250620648e-16 , 1.52855080404276e-14 , 1.732 , 3.14159265358979 , 0.23791443370276 , 3.22843362729246}\" vel=\"{0,0,0,0,0,0}\" acc=\"{0,0,0,0,0,0}\">"
"            <marker_pool type=\"MarkerPoolElement\" default_child_type=\"Marker\">"
"                <u4i pe=\"{ 0,0,0,-PI/2,0,0 }\"/>"
"                <p4j pe=\"{ 0,0,0,0,-PI/2,0 }\"/>"
"            </marker_pool>"
"            <geometry_pool type=\"GeometryPoolElement\">"
"                <solid type=\"ParasolidGeometry\" graphic_file_path=\"C:\\aris\\resource\\test_dynamic\\stewart\\pa.xmt_txt\"/>"
"            </geometry_pool>"
"        </p4a>"
"        <p4b active=\"true\" inertia=\"{1.82756 , -0.621 , 0.131 , -0.581 , 105.1 , 116.251 , 100.281 , 20.110151 , 12.20003 , 0.585391}\" pe=\"{-0.134375029322252 , 1.49999999999964 , 1.36823895396183 , 3.14159265358979 , 0.23791443370276 , 3.22843362729246}\" vel=\"{0,0,0,0,0,0}\" acc=\"{0,0,0,0,0,0}\">"
"            <marker_pool type=\"MarkerPoolElement\" default_child_type=\"Marker\">"
"                <p4i pe=\"{ 0,0,0,0,-PI/2,0 }\"/>"
"                <s4j pe=\"{ 0,0,0,0,0,0 }\"/>"
"            </marker_pool>"
"            <geometry_pool type=\"GeometryPoolElement\">"
"                <solid type=\"ParasolidGeometry\" graphic_file_path=\"C:\\aris\\resource\\test_dynamic\\stewart\\pb.xmt_txt\"/>"
"            </geometry_pool>"
"        </p4b>"
"        <p5a active=\"true\" inertia=\"{1.82156 , -0.621 , 0.131 , -0.581 , 105.1 , 112.251 , 100.281 , 20.110151 , 12.20003 , 0.585391}\" pe=\"{-0.999999999999993 , -1.0082029353865e-16 , 4.19175032725778e-17 , 6.28318530717959 , 0.739492476881246 , 5.88016725548812}\" vel=\"{0,0,0,0,0,0}\" acc=\"{0,0,0,0,0,0}\">"
"            <marker_pool type=\"MarkerPoolElement\" default_child_type=\"Marker\">"
"                <u5i pe=\"{ 0,0,0,-PI/2,0,0 }\"/>"
"                <p5j pe=\"{ 0,0,0,0,-PI/2,0 }\"/>"
"            </marker_pool>"
"            <geometry_pool type=\"GeometryPoolElement\">"
"                <solid type=\"ParasolidGeometry\" graphic_file_path=\"C:\\aris\\resource\\test_dynamic\\stewart\\pa.xmt_txt\"/>"
"            </geometry_pool>"
"        </p5a>"
"        <p5b active=\"true\" inertia=\"{1.82156 , -0.521 , 0.131 , -0.581 , 105.1 , 116.251 , 100.281 , 20.110151 , 12.20003 , 0.585391}\" pe=\"{-0.134375029322406 , 1.49999999999987 , 1.36823895396163 , 2.77555756156289e-17 , 0.739492476881246 , 5.88016725548812}\" vel=\"{0,0,0,0,0,0}\" acc=\"{0,0,0,0,0,0}\">"
"            <marker_pool type=\"MarkerPoolElement\" default_child_type=\"Marker\">"
"                <p5i pe=\"{ 0,0,0,0,-PI/2,0 }\"/>"
"                <s5j pe=\"{ 0,0,0,0,0,0 }\"/>"
"            </marker_pool>"
"            <geometry_pool type=\"GeometryPoolElement\">"
"                <solid type=\"ParasolidGeometry\" graphic_file_path=\"C:\\aris\\resource\\test_dynamic\\stewart\\pb.xmt_txt\"/>"
"            </geometry_pool>"
"        </p5b>"
"        <p6a active=\"true\" inertia=\"{1.82156 , -0.621 , 0.131 , -0.581 , 125.1 , 116.251 , 100.281 , 20.110151 , 12.20003 , 0.585391}\" pe=\"{-0.999999999999969 , -1.00821934664985e-16 , 4.19165900651815e-17 , 4.44089209850063e-16 , 0.546497081697639 , 5.73537938754121}\" vel=\"{0,0,0,0,0,0}\" acc=\"{0,0,0,0,0,0}\">"
"            <marker_pool type=\"MarkerPoolElement\" default_child_type=\"Marker\">"
"                <u6i pe=\"{ 0,0,0,-PI/2,0,0 }\"/>"
"                <p6j pe=\"{ 0,0,0,0,-PI/2,0 }\"/>"
"            </marker_pool>"
"            <geometry_pool type=\"GeometryPoolElement\">"
"                <solid type=\"ParasolidGeometry\" graphic_file_path=\"C:\\aris\\resource\\test_dynamic\\stewart\\pa.xmt_txt\"/>"
"            </geometry_pool>"
"        </p6a>"
"        <p6b active=\"true\" inertia=\"{1.82156 , -0.621 , 0.131 , -0.581 , 105.1 , 126.251 , 100.281 , 20.110151 , 12.20003 , 0.585391}\" pe=\"{0.0711481425888235 , 1.49999999999959 , 0.912443796234401 , 4.44089209850063e-16 , 0.546497081697639 , 5.73537938754121}\" vel=\"{0,0,0,0,0,0}\" acc=\"{0,0,0,0,0,0}\">"
"            <marker_pool type=\"MarkerPoolElement\" default_child_type=\"Marker\">"
"                <p6i pe=\"{ 0,0,0,0,-PI/2,0 }\"/>"
"                <s6j pe=\"{ 0,0,0,0,0,0 }\"/>"
"            </marker_pool>"
"            <geometry_pool type=\"GeometryPoolElement\">"
"                <solid type=\"ParasolidGeometry\" graphic_file_path=\"C:\\aris\\resource\\test_dynamic\\stewart\\pb.xmt_txt\"/>"
"            </geometry_pool>"
"        </p6b>"
"        <up active=\"true\" inertia=\"{1.82156 , -0.651 , 0.1313 , -0.5814 , 105.1 , 116.271 , 100.221 , 20.120151 , 12.22003 , 0.583391}\" pe=\"{0.1 , 1.5 , 1.2 , 1.5707963267949 , 0.1 , 4.71238898038469}\" vel=\"{0,0,0,0,0,0}\" acc=\"{0,0,0,0,0,0}\">"
"            <marker_pool type=\"MarkerPoolElement\" default_child_type=\"Marker\">"
"                <ee pe=\"{ 0,0,0,0,0,0 }\"/>"
"                <s1i pe=\"{ 0,0,-0.289,0,0,0 }\"/>"
"                <s2i pe=\"{ 0.25,0,0.144,0,0,0 }\"/>"
"                <s3i pe=\"{ 0.25,0,0.144,0,0,0 }\"/>"
"                <s4i pe=\"{ -0.25,0,0.144,0,0,0 }\"/>"
"                <s5i pe=\"{ -0.25,0,0.144,0,0,0 }\"/>"
"                <s6i pe=\"{ 0,0,-0.289,0,0,0 }\"/>"
"            </marker_pool>"
"            <geometry_pool type=\"GeometryPoolElement\">"
"                <solid type=\"ParasolidGeometry\" graphic_file_path=\"C:\\aris\\resource\\test_dynamic\\stewart\\up.xmt_txt\"/>"
"            </geometry_pool>"
"        </up>"
"        <ground active=\"true\" inertia=\"{1,0,0,0,1,1,1,0,0,0}\" pe=\"{0,0,0,0,0,0}\" vel=\"{0,0,0,0,0,0}\" acc=\"{0,0,0,0,0,0}\">"
"            <marker_pool type=\"MarkerPoolElement\" default_child_type=\"Marker\">"
"                <origin pe=\"{ 0,0,0,0,0,0 }\"/>"
"                <u1o pe=\"{ 1,0,0,0,0,0 }\"/>"
"                <u2o pe=\"{ 1,0,0,0,0,0 }\"/>"
"                <u3o pe=\"{ 0,0,1.732,0,0,0 }\"/>"
"                <u4o pe=\"{ 0,0,1.732,0,0,0 }\"/>"
"                <u5o pe=\"{ -1,0,0,0,0,0 }\"/>"
"                <u6o pe=\"{ -1,0,0,0,0,0 }\"/>"
"                <u1j pe=\"{ 1,0,0,PI/2,PI/2,PI/2 }\"/>"
"                <u2j pe=\"{ 1,0,0,PI/2,PI/2,PI/2 }\"/>"
"                <u3j pe=\"{ 0,0,1.732,PI/2,PI/2,PI/2 }\"/>"
"                <u4j pe=\"{ 0,0,1.732,PI/2,PI/2,PI/2 }\"/>"
"                <u5j pe=\"{ -1,0,0,PI/2,PI/2,PI/2 }\"/>"
"                <u6j pe=\"{ -1,0,0,PI/2,PI/2,PI/2 }\"/>"
"            </marker_pool>"
"        </ground>"
"    </part_pool>"
"    <joint_pool type=\"JointPoolElement\">"
"        <u1 active=\"true\" type=\"UniversalJoint\" prt_m=\"p1a\" prt_n=\"ground\" mak_i=\"u1i\" mak_j=\"u1j\"/>"
"        <p1 active=\"true\" type=\"PrismaticJoint\" prt_m=\"p1b\" prt_n=\"p1a\" mak_i=\"p1i\" mak_j=\"p1j\"/>"
"        <s1 active=\"true\" type=\"SphericalJoint\" prt_m=\"up\" prt_n=\"p1b\" mak_i=\"s1i\" mak_j=\"s1j\"/>"
"        <u2 active=\"true\" type=\"UniversalJoint\" prt_m=\"p2a\" prt_n=\"ground\" mak_i=\"u2i\" mak_j=\"u2j\"/>"
"        <p2 active=\"true\" type=\"PrismaticJoint\" prt_m=\"p2b\" prt_n=\"p2a\" mak_i=\"p2i\" mak_j=\"p2j\"/>"
"        <s2 active=\"true\" type=\"SphericalJoint\" prt_m=\"up\" prt_n=\"p2b\" mak_i=\"s2i\" mak_j=\"s2j\"/>"
"        <u3 active=\"true\" type=\"UniversalJoint\" prt_m=\"p3a\" prt_n=\"ground\" mak_i=\"u3i\" mak_j=\"u3j\"/>"
"        <p3 active=\"true\" type=\"PrismaticJoint\" prt_m=\"p3b\" prt_n=\"p3a\" mak_i=\"p3i\" mak_j=\"p3j\"/>"
"        <s3 active=\"true\" type=\"SphericalJoint\" prt_m=\"up\" prt_n=\"p3b\" mak_i=\"s3i\" mak_j=\"s3j\"/>"
"        <u4 active=\"true\" type=\"UniversalJoint\" prt_m=\"p4a\" prt_n=\"ground\" mak_i=\"u4i\" mak_j=\"u4j\"/>"
"        <p4 active=\"true\" type=\"PrismaticJoint\" prt_m=\"p4b\" prt_n=\"p4a\" mak_i=\"p4i\" mak_j=\"p4j\"/>"
"        <s4 active=\"true\" type=\"SphericalJoint\" prt_m=\"up\" prt_n=\"p4b\" mak_i=\"s4i\" mak_j=\"s4j\"/>"
"        <u5 active=\"true\" type=\"UniversalJoint\" prt_m=\"p5a\" prt_n=\"ground\" mak_i=\"u5i\" mak_j=\"u5j\"/>"
"        <p5 active=\"true\" type=\"PrismaticJoint\" prt_m=\"p5b\" prt_n=\"p5a\" mak_i=\"p5i\" mak_j=\"p5j\"/>"
"        <s5 active=\"true\" type=\"SphericalJoint\" prt_m=\"up\" prt_n=\"p5b\" mak_i=\"s5i\" mak_j=\"s5j\"/>"
"        <u6 active=\"true\" type=\"UniversalJoint\" prt_m=\"p6a\" prt_n=\"ground\" mak_i=\"u6i\" mak_j=\"u6j\"/>"
"        <p6 active=\"true\" type=\"PrismaticJoint\" prt_m=\"p6b\" prt_n=\"p6a\" mak_i=\"p6i\" mak_j=\"p6j\"/>"
"        <s6 active=\"true\" type=\"SphericalJoint\" prt_m=\"up\" prt_n=\"p6b\" mak_i=\"s6i\" mak_j=\"s6j\"/>"
"    </joint_pool>"
"    <motion_pool type=\"MotionPoolElement\" default_child_type=\"Motion\">"
"        <m1 active=\"true\" slave_id=\"0\" prt_m=\"p1b\" prt_n=\"p1a\" mak_i=\"p1i\" mak_j=\"p1j\" frc_coe=\"Mot_friction\" component=\"2\"/>"
"        <m2 active=\"true\" slave_id=\"9\" prt_m=\"p2b\" prt_n=\"p2a\" mak_i=\"p2i\" mak_j=\"p2j\" frc_coe=\"Mot_friction\" component=\"2\"/>"
"        <m3 active=\"true\" slave_id=\"8\" prt_m=\"p3b\" prt_n=\"p3a\" mak_i=\"p3i\" mak_j=\"p3j\" frc_coe=\"Mot_friction\" component=\"2\"/>"
"        <m4 active=\"true\" slave_id=\"3\" prt_m=\"p4b\" prt_n=\"p4a\" mak_i=\"p4i\" mak_j=\"p4j\" frc_coe=\"Mot_friction\" component=\"2\"/>"
"        <m5 active=\"true\" slave_id=\"7\" prt_m=\"p5b\" prt_n=\"p5a\" mak_i=\"p5i\" mak_j=\"p5j\" frc_coe=\"Mot_friction\" component=\"2\"/>"
"        <m6 active=\"true\" slave_id=\"5\" prt_m=\"p6b\" prt_n=\"p6a\" mak_i=\"p6i\" mak_j=\"p6j\" frc_coe=\"Mot_friction\" component=\"2\"/>"
"    </motion_pool>"
"    <general_motion_pool type=\"GeneralMotionPoolElement\" default_child_type=\"GeneralMotion\">"
"        <ee_mot type=\"GeneralMotion\" active=\"false\" prt_m=\"up\" prt_n=\"ground\" mak_i=\"ee\" mak_j=\"origin\"/>"
"    </general_motion_pool>"
"    <solver_pool type=\"SolverPoolElement\" default_child_type=\"Solver\">"
"        <ds type=\"UniversalSolver\" max_iter_count=\"100\" max_error=\"1e-14\"/>"
"        <gs type=\"LltGroundDividedSolver\" max_iter_count=\"100\" max_error=\"1e-14\"/>"
"        <ps type=\"LltPartDividedSolver\" max_iter_count=\"100\" max_error=\"1e-14\"/>"
"    </solver_pool>"
"</model>";


void test_solver(Model &m, const double *ipo, const double *ivo, const double *iao, const double *ifo, 
	const double *opo, const double *ovo, const double *oao, const double *ofo, 
	const double *ipt, const double *ivt, const double *iat, const double *ift, 
	const double *opt, const double *ovt, const double *oat, const double *oft, const double *error)
{
	for (auto &s : m.solverPool())
	{
		double result1[16], result2[16], result3[16], result4[16];
		
		// set topology //
		for (auto &mot : m.motionPool())mot.activate(true);
		m.generalMotionPool().at(0).activate(false);
		// set input origin //
		for (aris::Size i = 0; i < m.motionPool().size(); ++i)
		{
			m.motionPool().at(i).setMp(ipo[i]);
			m.motionPool().at(i).setMv(ivo[i]);
			m.motionPool().at(i).setMa(iao[i]);
		}
		// compute //
		s.init();
		s.kinPos();
		s.init();
		s.kinVel();
		s.init();
		s.dynAccAndFce();

		// get result //
		m.generalMotionPool().at(0).updMpm();
		m.generalMotionPool().at(0).getMpm(result1);
		m.generalMotionPool().at(0).updMvs();
		m.generalMotionPool().at(0).getMva(result2);
		m.generalMotionPool().at(0).updMas();
		m.generalMotionPool().at(0).getMaa(result3);
		for (int i = 0; i < m.motionPool().size(); ++i)result4[i] = m.motionPool().at(i).mf();
		// check //
		if (!s_is_equal(16, result1, opo, error[0]))std::cout << s.type() << "::kinPos() forward origin failed" << std::endl;
		if (!s_is_equal(6, result2, ovo, error[1]))std::cout << s.type() << "::kinVel() forward origin failed" << std::endl;
		if (!s_is_equal(6, result3, oao, error[2]))std::cout << s.type() << "::kinAcc() forward origin failed" << std::endl;
		if (!s_is_equal(m.motionPool().size(), result4, ifo, error[3]))std::cout << s.type() << "::dynFce() forward origin failed" << std::endl;

		// set input //
		for (aris::Size i = 0; i < m.motionPool().size(); ++i)
		{
			m.motionPool().at(i).setMp(ipt[i]);
			m.motionPool().at(i).setMv(ivt[i]);
			m.motionPool().at(i).setMa(iat[i]);
		}
		// compute //
		s.init();
		s.kinPos();
		s.init();
		s.kinVel();
		s.init();
		s.dynAccAndFce();
		// get result //
		m.generalMotionPool().at(0).updMpm();
		m.generalMotionPool().at(0).getMpm(result1);
		m.generalMotionPool().at(0).updMvs();
		m.generalMotionPool().at(0).getMva(result2);
		m.generalMotionPool().at(0).updMas();
		m.generalMotionPool().at(0).getMaa(result3);
		for (int i = 0; i < m.motionPool().size(); ++i)result4[i] = m.motionPool().at(i).mf();
		// check //
		if (!s_is_equal(16, result1, opt, error[0]))std::cout << s.type() << "::kinPos() forward failed" << std::endl;
		if (!s_is_equal(6, result2, ovt, error[1]))std::cout << s.type() << "::kinVel() forward failed" << std::endl;
		if (!s_is_equal(6, result3, oat, error[2]))std::cout << s.type() << "::kinAcc() forward failed" << std::endl;
		if (!s_is_equal(m.motionPool().size(), result4, ift, error[3]))std::cout << s.type() << "::dynFce() forward failed" << std::endl;


		////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



		// set topology //
		for (auto &mot : m.motionPool())mot.activate(false);
		m.generalMotionPool().at(0).activate(true);
		// set ee origin status //
		m.generalMotionPool().at(0).setMpm(opo);
		m.generalMotionPool().at(0).setMva(ovo);
		m.generalMotionPool().at(0).setMaa(oao);
		// compute //
		s.init();
		s.kinPos();
		s.init();
		s.kinVel();
		s.init();
		s.dynAccAndFce();

		// get result //
		for (aris::Size i = 0; i < m.motionPool().size(); ++i)
		{
			m.motionPool().at(i).updMp();
			m.motionPool().at(i).updMv();
			m.motionPool().at(i).updMa();
			result1[i] = m.motionPool().at(i).mp();
			result2[i] = m.motionPool().at(i).mv();
			result3[i] = m.motionPool().at(i).ma();
		}
		// check //
		if (!s_is_equal(m.motionPool().size(), result1, ipo, error[4]))std::cout << s.type() << "::kinPos() inverse origin failed" << std::endl;
		if (!s_is_equal(m.motionPool().size(), result2, ivo, error[5]))std::cout << s.type() << "::kinVel() inverse origin failed" << std::endl;
		if (!s_is_equal(m.motionPool().size(), result3, iao, error[6]))std::cout << s.type() << "::kinAcc() inverse origin failed" << std::endl;
		if (!s_is_equal(6, m.generalMotionPool().at(0).mfs(), ofo, error[7]))std::cout << s.type() << "::dynFce() inverse origin failed" << std::endl;


		// set ee status //
		m.generalMotionPool().at(0).setMpm(opt);
		m.generalMotionPool().at(0).setMva(ovt);
		m.generalMotionPool().at(0).setMaa(oat);
		// compute //
		s.init();
		s.kinPos();
		s.init();
		s.kinVel();
		s.init();
		s.dynAccAndFce();

		// get result //
		for (aris::Size i = 0; i < m.motionPool().size(); ++i)
		{
			m.motionPool().at(i).updMp();
			m.motionPool().at(i).updMv();
			m.motionPool().at(i).updMa();
			result1[i] = m.motionPool().at(i).mp();
			result2[i] = m.motionPool().at(i).mv();
			result3[i] = m.motionPool().at(i).ma();
		}
		// check //
		if (!s_is_equal(m.motionPool().size(), result1, ipt, error[4]))std::cout << s.type() << "::kinPos() inverse failed" << std::endl;
		if (!s_is_equal(m.motionPool().size(), result2, ivt, error[5]))std::cout << s.type() << "::kinVel() inverse failed" << std::endl;
		if (!s_is_equal(m.motionPool().size(), result3, iat, error[6]))std::cout << s.type() << "::kinAcc() inverse failed" << std::endl;
		if (!s_is_equal(6, m.generalMotionPool().at(0).mfs(), oft, error[7]))std::cout << s.type() << "::dynFce() inverse failed" << std::endl;
	}
}

void bench_solver(Model &m, aris::Size i, aris::Size bench_count, const double *ipo, const double *ivo, const double *iao, const double *ifo,
	const double *opo, const double *ovo, const double *oao, const double *ofo,
	const double *ipt, const double *ivt, const double *iat, const double *ift,
	const double *opt, const double *ovt, const double *oat, const double *oft, const double *error)
{
	double result1[16];
	auto &s = m.solverPool().at(i);


	for (auto &mot : m.motionPool())mot.activate(true);
	m.generalMotionPool().at(0).activate(false);
	s.init();

	for (aris::Size i = 0; i < m.motionPool().size(); ++i)
	{
		m.motionPool().at(i).setMp(ipt[i]);
		m.motionPool().at(i).setMv(ivt[i]);
		m.motionPool().at(i).setMa(iat[i]);
	}
	s.kinPos();
	s.kinVel();
	s.dynAccAndFce();;



	int count{ 0 };
	std::cout << s.type() << "::forward computational pos time:" << aris::core::benchmark(bench_count, [&]()
	{
		if (count % 2)for (int i{ 0 }; i < m.motionPool().size(); ++i) m.motionPool().at(i).setMp(ipt[i]);
		else for (int i{ 0 }; i < m.motionPool().size(); ++i) m.motionPool().at(i).setMp(ipo[i]);

		s.kinPos();
		m.generalMotionPool().at(0).updMpm();
		m.generalMotionPool().at(0).getMpm(result1);

		if (count < 2 && count % 2 && !s_is_equal(16, result1, opt, error[0]))
			std::cout << s.type() << "::kinPos() forward bench failed" << std::endl;
		if (count < 2 && (count + 1) % 2 && !s_is_equal(16, result1, opo, error[0]))
			std::cout << s.type() << "::kinPos() forward bench origin pos failed" << std::endl;

		++count;
	}) << std::endl;


	for (aris::Size i = 0; i < m.motionPool().size(); ++i)
	{
		m.motionPool().at(i).setMp(ipt[i]);
		m.motionPool().at(i).setMv(ivt[i]);
		m.motionPool().at(i).setMa(iat[i]);
	}
	s.kinPos();
	count = 0;
	std::cout << s.type() << "::forward computational vel time:" << aris::core::benchmark(bench_count, [&]()
	{
		for (aris::Size i = 0; i < m.motionPool().size(); ++i)
		{
			m.motionPool().at(i).setMp(ipt[i]);
			m.motionPool().at(i).setMv(ivt[i]);
			m.motionPool().at(i).setMa(iat[i]);
		}
		s.kinVel();
		m.generalMotionPool().at(0).updMvs();
		m.generalMotionPool().at(0).getMva(result1);
		if (!s_is_equal(6, result1, ovt, error[1]))std::cout << s.type() << "::kinVel() forward bench vel failed" << std::endl;
			

	}) << std::endl;



	for (aris::Size i = 0; i < m.motionPool().size(); ++i)
	{
		m.motionPool().at(i).setMp(ipt[i]);
		m.motionPool().at(i).setMv(ivt[i]);
		m.motionPool().at(i).setMa(iat[i]);
	}
	s.kinPos();
	s.kinVel();
	count = 0;
	std::cout << s.type() << "::forward computational acc time:" << aris::core::benchmark(bench_count, [&]()
	{
		for (aris::Size i = 0; i < m.motionPool().size(); ++i)
		{
			m.motionPool().at(i).setMp(ipt[i]);
			m.motionPool().at(i).setMv(ivt[i]);
			m.motionPool().at(i).setMa(iat[i]);
		}
		s.dynAccAndFce();
		m.generalMotionPool().at(0).updMas();
		m.generalMotionPool().at(0).getMaa(result1);
		if (!s_is_equal(6, result1, oat, error[2]))
			std::cout << s.type() << "::kinAcc() forward bench acc failed" << std::endl;
	}) << std::endl;






	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	for (auto &mot : m.motionPool())mot.activate(false);
	m.generalMotionPool().at(0).activate(true);
	s.init();


	m.generalMotionPool().at(0).setMpm(opo);
	m.generalMotionPool().at(0).setMva(ovo);
	m.generalMotionPool().at(0).setMva(oao);
	s.kinPos();
	s.kinVel();
	s.dynAccAndFce();;




	count =  0;
	std::cout << s.type() << "::inverse computational pos time:" << aris::core::benchmark(bench_count, [&]()
	{
		if (count % 2)m.generalMotionPool().at(0).setMpm(opt);
		else m.generalMotionPool().at(0).setMpm(opo);

		// compute //
		s.kinPos();
		for (aris::Size i = 0; i < m.motionPool().size(); ++i) { m.motionPool().at(i).updMp(); result1[i] = m.motionPool().at(i).mp(); }

		if (count < 2 && count % 2 && !s_is_equal(6, result1, ipt, error[4]))
			std::cout << s.type() << "::kinPos() forward bench failed" << std::endl;
		if (count < 2 && (count + 1) % 2 && !s_is_equal(6, result1, ipo, error[4]))
			std::cout << s.type() << "::kinPos() forward bench origin failed" << std::endl;

		++count;
	}) << std::endl;



	m.generalMotionPool().at(0).setMpm(opt);
	s.kinPos();
	count = 0;
	std::cout << s.type() << "::inverse computational vel time:" << aris::core::benchmark(bench_count, [&]()
	{
		m.generalMotionPool().at(0).setMva(ovt);
		s.kinVel();
		for (aris::Size i = 0; i < m.motionPool().size(); ++i) { m.motionPool().at(i).updMv(); result1[i] = m.motionPool().at(i).mv(); }
		if (!s_is_equal(6, result1, ivt, error[5]))
			std::cout << s.type() << "::kinVel() inverse bench vel failed" << std::endl;
	}) << std::endl;



	m.generalMotionPool().at(0).setMpm(opt);
	m.generalMotionPool().at(0).setMva(ovt);
	s.kinPos();
	s.kinVel();
	count = 0;
	std::cout << s.type() << "::inverse computational acc time:" << aris::core::benchmark(bench_count, [&]()
	{
		m.generalMotionPool().at(0).setMaa(oat);
		s.dynAccAndFce();
		for (aris::Size i = 0; i < m.motionPool().size(); ++i) { m.motionPool().at(i).updMa(); result1[i] = m.motionPool().at(i).ma(); }
		if (!s_is_equal(m.motionPool().size(), result1, iat, error[6]))
			std::cout << s.type() << "::kinAcc() inverse bench acc failed" << std::endl;
	}) << std::endl;
}

void test_solver_compute()
{
	const double error = 1e-10;

	try
	{
		const double im[]{ 1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
			0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
			0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
			0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
			0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
			0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
			0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
			0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
			0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
			0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
			0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,
			0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,
			0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,
			0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,1,0,0,0,0,0,0,
			0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,-1,0,0,0,0,0,0,0,
			0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,
			0,0,0,0,0,0,0,0,0,0,0,0,0,0,-1,0,2,0,0,0,0,0,0,0,
			0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,2,0,0,0,0,0,0,
			0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,-0,0,0,-0,-1,
			0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-0,1,0,0,0,1,
			0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,-1,0,
			0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,2,-1,0,
			0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-0,0,-1,-1,2,0,
			0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-1,1,-0,0,0,3 };
		const double cm[]{
			1,0,0,0,0,0,-1,-0,-0,-0,-0,0,0,0,0,0,0,0,0,0,0,-0,0,0,
			0,1,0,0,0,0,-0,-1,-0,-0,-0,0,0,0,0,0,0,0,0,0,0,-0,0,0,
			0,0,1,0,0,0,-0,-0,-1,-0,-0,0,0,0,0,0,0,0,0,0,0,-0,0,0,
			0,0,0,1,0,0,-0,-0,-0,-1,-0,0,0,0,0,0,0,0,0,0,0,-0,0,0,
			0,0,0,0,1,0,-0,-0,-0,-0,-1,0,0,0,0,0,0,0,0,0,0,-0,0,0,
			0,0,0,0,0,1,-0,-0,-0,-0,-0,0,0,0,0,0,0,0,0,0,0,-1,0,0,
			0,0,0,0,0,0,1,0,0,0,0,-0,1,-0,-0,-0,0,0,0,0,0,0,-0,0,
			0,0,0,0,0,0,0,1,0,0,0,-1,-0,-0,-0,-0,0,0,0,0,0,0,-0,0,
			0,0,0,0,0,0,0,0,1,0,0,-0,-0,-1,-0,-0,0,0,0,0,0,0,-0,0,
			0,0,0,0,0,0,0,0,0,1,0,-0,-0,-0,-0,1,0,0,0,0,0,0,-0,0,
			0,0,0,0,0,0,0,0,0,0,1,-0,-0,1,-1,-0,0,0,0,0,0,0,-0,0,
			0,0,0,0,0,0,0,0,0,0,0,-1,-0,-0,-0,-0,0,0,0,0,0,1,-1,0,
			0,0,0,0,0,0,0,0,0,0,0,0,-1,0,0,0,-0.980066577841242,0.174348740288176,-0.0952471509205588,-0,-0,0,0,-0,
			0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,-0.198669330795061,-0.860089338205047,0.469868946949515,-0,-0,0,0,-0,
			0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,-0,-0.479425538604203,-0.877582561890373,-0,-0,0,0,-0,
			0,0,0,0,0,0,0,0,0,0,0,0,0,0,1.61554457443259e-15,-1,-0,-0.479425538604203,-0.877582561890373,-0.980066577841242,0.174348740288176,0,0,-0.0952471509205588,
			0,0,0,0,0,0,0,0,0,0,0,0,0,-1,1,1.61554457443259e-15,-0,0.479425538604203,0.877582561890373,-0.198669330795061,-0.860089338205047,0,0,0.469868946949515,
			0,0,0,0,0,0,0,0,0,0,0,1,1.61554457443259e-15,0,0,0,0.78139724704618,-1.03443807849322,0.565116097870074,-0,-0.479425538604203,0,1,-0.877582561890373,
			0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.980066577841242,-0.174348740288176,0.0952471509205588,0,0,0,0,0,
			0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.198669330795061,0.860089338205047,-0.469868946949515,0,0,0,0,0,
			0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.479425538604203,0.877582561890373,0,0,0,0,0,
			0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.479425538604203,0.877582561890373,0.980066577841242,-0.174348740288176,0,0,0.0952471509205588,
			0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-0.479425538604203,-0.877582561890373,0.198669330795061,0.860089338205047,0,0,-0.469868946949515,
			0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-0.78139724704618,1.03443807849322,-0.565116097870074,0,0.479425538604203,0,0,0.877582561890373,
		};
		const double cp[]{ 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,-0.418547288472974,0.342216321165367,0,-1.57079632679489,	1.34022772104518, };
		const double cv[]{ 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, };
		const double ca[]{ 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, };
		aris::core::XmlDocument xml_doc;
		xml_doc.Parse(xml_file_3R);
		Model m;
		m.loadXmlDoc(xml_doc);

		for (auto &mot : m.motionPool())mot.activate(true);
		m.generalMotionPool().at(0).activate(false);

		auto &gs = static_cast<GroundDividedSolver&>(*m.solverPool().findByName("gs"));
		auto &ps = static_cast<PartDividedSolver&>(*m.solverPool().findByName("ps"));
		auto &ds = static_cast<UniversalSolver&>(*m.solverPool().findByName("ds"));
		auto &gcs = m.solverPool().add<CombineSolver>("gcs");

		gs.init();

		gs.updCm();
		if (!s_is_equal(24, 24, gs.cm(), cm, error))std::cout << "GroundDividedSolver::updCm() failed" << std::endl;

		gs.updIm();
		if (!s_is_equal(24, 24, gs.im(), im, error))std::cout << "GroundDividedSolver::updIm() failed" << std::endl;

		gs.updCp();
		if (!s_is_equal(24, 1, gs.cp(), cp, error))std::cout << "GroundDividedSolver::updCp() failed" << std::endl;

		gs.updCv();
		if (!s_is_equal(24, 1, gs.cv(), cv, error))std::cout << "GroundDividedSolver::updCv() failed" << std::endl;

		gs.updCa();
		if (!s_is_equal(24, 1, gs.ca(), ca, error))std::cout << "GroundDividedSolver::updCa() failed" << std::endl;




	}
	catch (std::exception &e)
	{
		std::cout << e.what();
	}
}
void test_solver_under_constraint()
{
	const double error = 1e-10;

	try
	{
		aris::core::XmlDocument xml_doc;
		xml_doc.Parse(xml_file_under_constraint);
		Model m;
		m.loadXmlDoc(xml_doc);

		auto &gs = static_cast<GroundDividedSolver&>(*m.solverPool().findByName("gs"));
		auto &ps = static_cast<PartDividedSolver&>(*m.solverPool().findByName("ps"));
		auto &ds = static_cast<UniversalSolver&>(*m.solverPool().findByName("ds"));
		auto &gcs = m.solverPool().add<CombineSolver>("gcs");

		ps.setMaxError(1e-14);
		gs.setMaxError(1e-14);
		ds.setMaxError(1e-14);
		gcs.setMaxError(1e-14);

		ds.allocateMemory();
		ds.kinPos();
		ds.kinVel();
		ds.dynAccAndFce();

		for (auto &p : m.partPool())
		{
			std::cout << p.name() << std::endl;
			double aa[6];
			p.getAa(aa);
			dsp(1, 6, aa);
		}
		


		auto &r = m.simResultPool().add<SimResult>("result1");
		auto &adams_simulator = m.simulatorPool().add<AdamsSimulator>("adams_simulator", &ds);
		r.record();
		adams_simulator.saveAdams("C:\\Users\\py033\\Desktop\\m4.cmd", r, 0);
	}
	catch (std::exception &e)
	{
		std::cout << e.what() << std::endl;
	}
}
void test_solver_3R()
{
	const double error = 1e-10;

	try
	{
		aris::core::XmlDocument xml_doc;
		xml_doc.Parse(xml_file_3R);
		Model m;
		m.loadXmlDoc(xml_doc);

		for (auto &mot : m.motionPool())mot.activate(true);
		m.generalMotionPool().at(0).activate(false);

		auto &gs = static_cast<GroundDividedSolver&>(*m.solverPool().findByName("gs"));
		auto &ps = static_cast<PartDividedSolver&>(*m.solverPool().findByName("ps"));
		auto &ds = static_cast<UniversalSolver&>(*m.solverPool().findByName("ds"));
		auto &gcs = m.solverPool().add<CombineSolver>("gcs");

		const double input_origin_p[3]{ 0.0, 0.0, 0.0 };
		const double input_origin_v[3]{ 0.0, 0.0, 0.0 };
		const double input_origin_a[3]{ 0.0, 0.0, 0.0 };
		const double input_origin_mf[3]{ 29.4,	9.8,	0.0 };
		const double output_origin_pm[16]{ 1.0 , 0.0 , 0.0 , 3.0 , 0.0 , 1.0 , 0.0 , 0.0 , 0.0 , 0.0 , 1.0 , 0.0 , 0.0 , 0.0 , 0.0 , 1.0 };
		const double output_origin_va[6]{ 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 };
		const double output_origin_aa[6]{ 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 };
		const double output_origin_mfs[6]{ 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 };

		const double input_p[3]{ 0.585, 0.685, -0.312 };
		const double input_v[3]{ 0.235, 0.235, 0.235 };
		const double input_a[3]{ -1.567, -1.567, -1.567};
		const double input_mf[3]{ -0.17105260350807,	-9.24402272506392,	-4.70099999999998 };
		const double input_p2[3]{ 1.27, -0.685, 0.373 };
		const double input_v2[3]{ 0.47, -0.235, 0.47 };
		const double input_a2[3]{ -3.134, 1.567, -3.134 };
		const double output_pm[16]{ 0.5751572210772128, - 0.8180428907109568,   0.0000000000000000,   1.7051501798895701,
			0.8180428907109568,   0.5751572210772128,   0.0000000000000000,   2.3253431286258799,
			0.0000000000000000,   0.0000000000000000,   1.0000000000000000,   0.0000000000000000,
			0.0000000000000000,   0.0000000000000000,   0.0000000000000000,   1.0000000000000000, };
		const double output_va[6]{ -1.1553844949236325 , 0.7406601913177889 , 0.0 , 0.0 , 0.0 , 0.705 };
		const double output_aa[6]{ 7.3068444301678799 ,-5.5868499482603751 , 0.0 , 0.0 , 0.0 ,-4.701 };
		const double output_mfs[6]{ -0.17105260350807,	-9.24402272506392,	-4.70099999999998 };

		double result1[16], result2[16], result3[16], result4[16];

		const double ee_pq[7]{ 1.70515017988957 , 2.32534312862588, 0.0 , 0.0 , 0.0 , 0.460891949876968 , 0.887456258380438 };
		const double ee_va[6]{ -1.1553844949236325 , 0.7406601913177889 , 0.0 , 0.0 , 0.0 , 0.705 };
		const double ee_aa[6]{ 7.3068444301678799 ,-5.5868499482603751 , 0.0 , 0.0 , 0.0 ,-4.701 };
		const double mot_fs[3]{ 0.17105260350807,	9.24402272506392,	4.70099999999998 };

		m.motionPool().at(0).setMp(0.585);
		m.motionPool().at(0).setMv(0.235);
		m.motionPool().at(0).setMa(-1.567);

		m.motionPool().at(1).setMp(0.685);
		m.motionPool().at(1).setMv(0.235);
		m.motionPool().at(1).setMa(-1.567);

		m.motionPool().at(2).setMp(-0.312);
		m.motionPool().at(2).setMv(0.235);
		m.motionPool().at(2).setMa(-1.567);

		auto test_forward = [&](Solver &s)->void
		{
			// set topology //
			for (auto &mot : m.motionPool())mot.activate(true);
			m.generalMotionPool().at(0).activate(false);
			// set input origin //
			for (aris::Size i = 0; i < m.motionPool().size(); ++i)
			{
				m.motionPool().at(i).setMp(input_origin_p[i]);
				m.motionPool().at(i).setMv(input_origin_v[i]);
				m.motionPool().at(i).setMa(input_origin_a[i]);
			}
			// compute //
			s.init();
			s.kinPos();
			s.init();
			s.kinVel();
			s.init();
			s.dynAccAndFce();

			// get result //
			m.generalMotionPool().at(0).updMpm();
			m.generalMotionPool().at(0).getMpm(result1);
			m.generalMotionPool().at(0).updMvs();
			m.generalMotionPool().at(0).getMva(result2);
			m.generalMotionPool().at(0).updMas();
			m.generalMotionPool().at(0).getMaa(result3);
			for (aris::Size i = 0; i < m.motionPool().size(); ++i)result4[i] = m.motionPool().at(i).mf();
			// check //
			if (!s_is_equal(16, result1, output_origin_pm, error))std::cout << s.type() << "::kinPos() forward origin failed" << std::endl;
			if (!s_is_equal(6, result2, output_origin_va, error))std::cout << s.type() << "::kinVel() forward origin failed" << std::endl;
			if (!s_is_equal(6, result3, output_origin_aa, error))std::cout << s.type() << "::kinAcc() forward origin failed" << std::endl;
			if (!s_is_equal(m.motionPool().size(), result4, input_origin_mf, 1e-9))std::cout << s.type() << "::dynFce() forward origin failed" << std::endl;

			// set input //
			for (aris::Size i = 0; i < m.motionPool().size(); ++i)
			{
				m.motionPool().at(i).setMp(input_p[i]);
				m.motionPool().at(i).setMv(input_v[i]);
				m.motionPool().at(i).setMa(input_a[i]);
			}
			// compute //
			s.init();
			s.kinPos();
			s.init();
			s.kinVel();
			s.init();
			s.dynAccAndFce();
			// get result //
			m.generalMotionPool().at(0).updMpm();
			m.generalMotionPool().at(0).getMpm(result1);
			m.generalMotionPool().at(0).updMvs();
			m.generalMotionPool().at(0).getMva(result2);
			m.generalMotionPool().at(0).updMas();
			m.generalMotionPool().at(0).getMaa(result3);
			for (aris::Size i = 0; i < m.motionPool().size(); ++i)result4[i] = m.motionPool().at(i).mf();
			// check //
			if (!s_is_equal(16, result1, output_pm, error))std::cout << s.type() << "::kinPos() forward failed" << std::endl;
			if (!s_is_equal(6, result2, output_va, error))std::cout << s.type() << "::kinVel() forward failed" << std::endl;
			if (!s_is_equal(6, result3, output_aa, error))std::cout << s.type() << "::kinAcc() forward failed" << std::endl;
			if (!s_is_equal(m.motionPool().size(), result4, input_mf, 1e-9))std::cout << s.type() << "::dynFce() forward failed" << std::endl;

		};
		auto test_inverse = [&](Solver &s)->void
		{
			// set topology //
			for (auto &mot : m.motionPool())mot.activate(false);
			m.generalMotionPool().at(0).activate(true);

			// set ee origin status //
			m.generalMotionPool().at(0).setMpm(output_origin_pm);
			m.generalMotionPool().at(0).setMva(output_origin_va);
			m.generalMotionPool().at(0).setMaa(output_origin_aa);

			// compute //
			s.init();
			s.kinPos();
			s.init();
			s.kinVel();
			s.init();
			s.dynAccAndFce();

			// get result //
			for (aris::Size i = 0; i < m.motionPool().size(); ++i)
			{
				m.motionPool().at(i).updMp();
				m.motionPool().at(i).updMv();
				m.motionPool().at(i).updMa();
				result1[i] = m.motionPool().at(i).mp();
				result2[i] = m.motionPool().at(i).mv();
				result3[i] = m.motionPool().at(i).ma();
			}
			// check //
			if (!s_is_equal(m.motionPool().size(), result1, input_origin_p, 1e-6))std::cout << s.type() << "::kinPos() inverse origin failed" << std::endl;
			if (!s_is_equal(m.motionPool().size(), result2, input_origin_v, 1e-8))std::cout << s.type() << "::kinVel() inverse origin failed" << std::endl;
			if (!s_is_equal(m.motionPool().size(), result3, input_origin_a, error))std::cout << s.type() << "::kinAcc() inverse origin failed" << std::endl;
			//if (!s_is_equal(6, m.generalMotionPool().at(0).mfs(), output_origin_mfs, 1e-9))std::cout << s.type() << "::dynFce() inverse origin failed" << std::endl;


			// set ee status //
			m.generalMotionPool().at(0).setMpm(output_pm);
			m.generalMotionPool().at(0).setMva(output_va);
			m.generalMotionPool().at(0).setMaa(output_aa);
			// compute //
			s.init();
			s.kinPos();
			s.init();
			s.kinVel();
			s.init();
			s.dynAccAndFce();

			std::cout << s.iterCount() << std::endl;

			// get result //
			for (aris::Size i = 0; i < m.motionPool().size(); ++i)
			{
				m.motionPool().at(i).updMp();
				m.motionPool().at(i).updMv();
				m.motionPool().at(i).updMa();
				result1[i] = m.motionPool().at(i).mp();
				result2[i] = m.motionPool().at(i).mv();
				result3[i] = m.motionPool().at(i).ma();
			}
			// check //
			if (!(s_is_equal(m.motionPool().size(), result1, input_p, error) || s_is_equal(m.motionPool().size(), result1, input_p2, error)))std::cout << s.type() << "::kinPos() inverse failed" << std::endl;
			if (!(s_is_equal(m.motionPool().size(), result2, input_v, error) || s_is_equal(m.motionPool().size(), result2, input_v2, error)))std::cout << s.type() << "::kinVel() inverse failed" << std::endl;
			if (!(s_is_equal(m.motionPool().size(), result3, input_a, error) || s_is_equal(m.motionPool().size(), result3, input_a2, error)))std::cout << s.type() << "::kinAcc() inverse failed" << std::endl;
			//if (!(s_is_equal(6, m.generalMotionPool().at(0).mfs(), output_mfs, error)|| s_is_equal(6, m.generalMotionPool().at(0).mfs(), output_mfs, error)))std::cout << s.type() << "::dynFce() inverse failed" << std::endl;
		};
		auto bench_pos_forward = [&](Solver &s, aris::Size bench_count)
		{
			int count{ 0 };
			for (auto &mot : m.motionPool())mot.activate(true);
			m.generalMotionPool().at(0).activate(false);
			s.init();
			std::cout << s.type() << "::forward computational pos time:" << aris::core::benchmark(bench_count, [&]()
			{
				if (count % 2)for (aris::Size i{ 0 }; i < m.motionPool().size(); ++i) m.motionPool().at(i).setMp(input_p[i]);
				else for (aris::Size i{ 0 }; i < m.motionPool().size(); ++i) m.motionPool().at(i).setMp(input_origin_p[i]);

				// compute //
				s.kinPos();
				m.generalMotionPool().at(0).updMpm();
				m.generalMotionPool().at(0).getMpm(result1);

				if (count < 2 && count % 2 && !s_is_equal(16, result1, output_pm, error))
					std::cout << s.type() << "::kinPos() forward bench failed" << std::endl;
				if (count < 2 && (count + 1) % 2 && !s_is_equal(16, result1, output_origin_pm, error))
					std::cout << s.type() << "::kinPos() forward bench origin failed" << std::endl;

				++count;
			}) << std::endl;
		};
		auto bench_vel_forward = [&](Solver &s, aris::Size bench_count)
		{
			int count{ 0 };
			for (auto &mot : m.motionPool())mot.activate(true);
			m.generalMotionPool().at(0).activate(false);
			for (aris::Size i = 0; i < m.motionPool().size(); ++i)
			{
				m.motionPool().at(i).setMp(input_p[i]);
				m.motionPool().at(i).setMv(input_v[i]);
				m.motionPool().at(i).setMa(input_a[i]);
			}
			s.init();
			s.kinPos();
			s.kinVel();
			s.dynAccAndFce();
			std::cout << s.type() << "::forward computational vel time:" << aris::core::benchmark(bench_count, [&]()
			{
				s.kinVel();
			}) << std::endl;
		};
		auto bench_dyn_forward = [&](Solver &s, aris::Size bench_count)
		{
			int count{ 0 };
			for (auto &mot : m.motionPool())mot.activate(true);
			m.generalMotionPool().at(0).activate(false);
			for (aris::Size i = 0; i < m.motionPool().size(); ++i)
			{
				m.motionPool().at(i).setMp(input_p[i]);
				m.motionPool().at(i).setMv(input_v[i]);
				m.motionPool().at(i).setMa(input_a[i]);
			}
			s.init();
			s.kinPos();
			s.kinVel();
			s.dynAccAndFce();

			std::cout << s.type() << "::forward computational dyn time:" << aris::core::benchmark(bench_count, [&]() { s.dynAccAndFce(); }) << std::endl;
		};
		auto bench_pos_inverse = [&](Solver &s, aris::Size bench_count)
		{
			int count{ 0 };
			for (auto &mot : m.motionPool())mot.activate(false);
			m.generalMotionPool().at(0).activate(true);
			s.init();
			std::cout << s.type() << "::inverse computational pos time:" << aris::core::benchmark(bench_count, [&]()
			{
				if (count % 2)for (aris::Size i{ 0 }; i < m.motionPool().size(); ++i) m.generalMotionPool().at(0).setMpm(output_pm);
				else for (aris::Size i{ 0 }; i < m.motionPool().size(); ++i) m.generalMotionPool().at(0).setMpm(output_origin_pm);

				// compute //
				s.kinPos();
				for (aris::Size i = 0; i < m.motionPool().size(); ++i) { m.motionPool().at(i).updMp(); result1[i] = m.motionPool().at(i).mp(); }

				if (count < 2 && count % 2 && !s_is_equal(6, result1, input_p, error))
					std::cout << s.type() << "::kinPos() forward bench failed" << std::endl;
				if (count < 2 && (count + 1) % 2 && !s_is_equal(6, result1, input_origin_p, error))
					std::cout << s.type() << "::kinPos() forward bench origin failed" << std::endl;

				++count;
			}) << std::endl;
		};
		auto bench_vel_inverse = [&](Solver &s, aris::Size bench_count)
		{
			int count{ 0 };
			for (auto &mot : m.motionPool())mot.activate(false);
			m.generalMotionPool().at(0).activate(true);
			m.generalMotionPool().at(0).setMpm(output_origin_pm);
			m.generalMotionPool().at(0).setMva(output_origin_va);
			m.generalMotionPool().at(0).setMaa(output_origin_aa);
			s.init();
			s.kinPos();
			s.kinVel();
			s.dynAccAndFce();
			std::cout << s.type() << "::inverse computational vel time:" << aris::core::benchmark(bench_count, [&]()
			{
				s.kinVel();
			}) << std::endl;
		};
		auto bench_dyn_inverse = [&](Solver &s, aris::Size bench_count)
		{
			int count{ 0 };
			for (auto &mot : m.motionPool())mot.activate(false);
			m.generalMotionPool().at(0).activate(true);
			m.generalMotionPool().at(0).setMpm(output_origin_pm);
			m.generalMotionPool().at(0).setMva(output_origin_va);
			m.generalMotionPool().at(0).setMaa(output_origin_aa);
			s.init();
			s.kinPos();
			s.kinVel();
			s.dynAccAndFce();
			std::cout << s.type() << "::inverse computational dyn time:" << aris::core::benchmark(bench_count, [&]() {	s.dynAccAndFce(); }) << std::endl;
		};

		ps.setMaxError(1e-14);
		gs.setMaxError(1e-14);
		ds.setMaxError(1e-14);
		gcs.setMaxError(1e-14);

		std::cout << "test 3R robot:" << std::endl;

		//test_forward(gcs);
		//test_inverse(gcs);

		test_forward(ds);
		test_inverse(ds);

		m.simulatorPool().add<SolverSimulator>("simulator", &ds);
		m.simResultPool().add<SimResult>("result1");
		m.simResultPool().front().allocateMemory();
		m.simResultPool().front().allocateMemory();
		m.simResultPool().front().record();
		m.simResultPool().front().record();
		m.simResultPool().front().record();

		m.saveXmlFile("C:\\Users\\py033\\Desktop\\m.xml");


		Model m2;
		m2.loadXmlFile("C:\\Users\\py033\\Desktop\\m.xml");
		m2.saveXmlFile("C:\\Users\\py033\\Desktop\\m2.xml");
	}
	catch (std::exception &e)
	{
		std::cout << e.what() << std::endl;
	}

}
void test_solver_6R()
{
	try
	{
		const double error = 1e-10;

		const double input_origin_p[6]{ 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 };
		const double input_origin_v[6]{ 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 };
		const double input_origin_a[6]{ 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 };
		const double input_origin_mf[6]{ 34.8372347443935, 0.179465241344625, 26.1146353297101, -3.12638803734444e-13, -5.6843418860808e-13, 9.9475983006414e-14 };
		const double output_origin_pm[16]{ 1.0 , 0.0 , 0.0 , 0.0 , 0.0 , 1.0 , 0.0 , 0.0 , 0.0 , 0.0 , 1.0 , 0.0 , 0.0 , 0.0 , 0.0 , 1.0 };
		const double output_origin_va[6]{ 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 };
		const double output_origin_aa[6]{ 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 };
		const double output_origin_mfs[6]{ 1.53575836143848,38.0002468240561,16.8933419758238,8.4241178308505,-16.6881665634178,51.5626922658449 };

		const double input_p[6]{ -0.084321840829742,0.111235847475406,0.163501201249858,0.41316722587035, -0.0861578092597486,0.229246197281016 };
		const double input_v[6]{ 0.93426722257942, -0.024823760537999, -0.89419018046124,   0.245922301638701, -1.23100367003297, -0.48185561218356 };
		const double input_a[6]{ 0.70807836306709, -0.496581922752884, -0.159513727427361, -0.590163055515337,   0.131806583011732, -1.65802060177352 };
		const double input_mf[6]{ 24.6359418510515, -3.06678992657553, 13.4565070365958,   -15.0336821069307,   -0.786112551012351,   -1.93281931696021 };
		const double output_pm[16]{ 0.863013488544127, -0.284074444773496,   0.417743256579356, -0.137731283515364,
			0.387677110267304,   0.902605554641921, -0.187108714132569, -0.343275971674581,
			-0.323904579723239,   0.323426842664891,   0.889089928341408, -0.0474940394315194,
			0,   0,   0,   1 };
		const double output_va[6]{ -1.93242030056314,   0.500930573127293,   0.577926916892486, -0.399682310201935, -0.66053331463003, -0.857440373970742 };
		const double output_aa[6]{ 1.07075600145293,   0.349116022890415,   2.0925775293411, -1.77982973680254, -0.927893632540704,   0.0659817357654945 };
		const double output_mfs[6]{ 8.44990411304192, 54.7768126462764, 23.2058019399381, 18.6214939645874,   -51.751313528282, 82.047228392192 };

		const double error2[8]{ 1e-10, 1e-10, 1e-10, 1e-9, 1e-10, 1e-10, 1e-10, 1e-9 };

		std::cout << "test 6R robot:" << std::endl;

		Model m;
		m.loadXmlStr(xml_file_6R);

		test_solver(m, input_origin_p, input_origin_v, input_origin_a, input_origin_mf,
			output_origin_pm, output_origin_va, output_origin_aa, output_origin_mfs,
			input_p, input_v, input_a, input_mf,
			output_pm, output_va, output_aa, output_mfs, error2);

		bench_solver(m, 0, 10000, input_origin_p, input_origin_v, input_origin_a, input_origin_mf,
			output_origin_pm, output_origin_va, output_origin_aa, output_origin_mfs,
			input_p, input_v, input_a, input_mf,
			output_pm, output_va, output_aa, output_mfs, error2);

		//auto plan = [&](const aris::dynamic::PlanParam& param)->int
		//{
		//	double pm[16];
		//	std::copy(output_pm, output_pm + 16, pm);
		//	pm[3] += 0.01 * std::sin(2 * PI*param.count_ / 1000);
		//	param.model_->generalMotionPool().at(0).setMpm(pm);
		//	param.model_->setTime(param.count_ * 0.001);
		//	return 1000 - param.count_;
		//};
		//for (auto &mot : m.motionPool())mot.activate(false);
		//m.generalMotionPool().at(0).activate(true);
		//auto &s = m.simulatorPool().add<SolverSimulator>("simulator", &ds);
		//auto &r = m.simResultPool().add<SimResult>("result1");

		//s.simulate(plan, 0, r);
		//m.saveXmlFile("C:\\Users\\py033\\Desktop\\m3.xml");
		
		//auto &adams_simulator = m.simulatorPool().add<AdamsSimulator>("adams_simulator", &ds);
		//adams_simulator.saveAdams("C:\\Users\\py033\\Desktop\\m3.cmd", r);

	}
	catch (std::exception&e)
	{
		std::cout << e.what() << std::endl;
	}
}
void test_solver_stewart()
{
	try
	{	
		const double error = 1e-10;
		
		const double input_origin_p[6]{ 2.0 , 2.0 , 2.0 , 2.0 , 2.0 , 2.0 };
		const double input_origin_v[6]{ 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 };
		const double input_origin_a[6]{ 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 };
		const double input_origin_mf[6]{ 11.3871054640557503, 13.6991667810515523, -5.9717817946918421, 5.5825903452181880, 50.4497942853426551, 41.0950030749182460 };
		const double output_origin_pm[16]{ 1,0,0,0,
			0, 0.999999999751072,2.2312668404904e-05,1.7078344386197,
			0, -2.23126684049141e-05,0.999999999751072,0.577658198650165,
			0,0,0,1 };
		const double output_origin_va[6]{ 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 };
		const double output_origin_aa[6]{ 0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0 };
		const double output_origin_mfs[6]{ 27.1911376020517253, 99.2601846909161623, 30.9203302634324899, 5.1114448379402990, -0.6600651170996999, -10.3120957308179584 };

		const double input_p[6]{ 2.15,2.03,1.98,1.68,2.22,2.01 };
		const double input_v[6]{ 0.687,1.521,-0.325,0.665,1.225,-0.999 };
		const double input_a[6]{ 1.687,0.521,-1.325,1.665,0.225,-1.999 };
		const double input_mf[6]{ 2124.4812403533132965, 275.7606350348426645, 2757.4743715794970740,   -3994.5016219944400291, 3892.2226684026377370, -4810.0920919994805445 };
		const double output_pm[16]{ 0.654617242227831, -0.16813527373803,0.737025641279234,0.0674004103296998,
			0.286892301165042,0.957269694021347, -0.0364354283699648,1.66351811346172,
			-0.699406229390514,0.235298241883176,0.674881962758251,0.907546391448817,
			0,0,0,1 };
		const double output_va[6]{ -1.67602445813444,0.322144550146041,1.43386389933679, -4.13258637478856,0.229701802785213,2.06026880988191 };
		const double output_aa[6]{ -3.99625983193204, -4.52459258496676,3.82662285536541, -4.70386456087171,10.2271223856012,12.7760010719168 };
		const double output_mfs[6]{ -1752.8168759636657796,   -200.8968525620247192, 86.3334906336755807,   -816.6714933354393224, 1685.6093614991480081, 661.2063243054601571 };

		const double error2[8]{ 1e-10, 1e-9, 1e-8, 1e-8, 1e-10, 1e-9, 1e-8, 1e-8 };

		std::cout << "test stewart robot:" << std::endl;

		Model m;
		m.loadXmlStr(xml_file_stewart);

		test_solver(m, input_origin_p, input_origin_v, input_origin_a, input_origin_mf,
			output_origin_pm, output_origin_va, output_origin_aa, output_origin_mfs,
			input_p, input_v, input_a, input_mf,
			output_pm, output_va, output_aa, output_mfs, error2);

		bench_solver(m, 0, 10000, input_origin_p, input_origin_v, input_origin_a, input_origin_mf,
			output_origin_pm, output_origin_va, output_origin_aa, output_origin_mfs,
			input_p, input_v, input_a, input_mf,
			output_pm, output_va, output_aa, output_mfs, error2);

		//auto &r = m.simResultPool().add<SimResult>("result1");
		//auto &adams_simulator = m.simulatorPool().add<AdamsSimulator>("adams_simulator", &ds);
		//r.record();
		//adams_simulator.saveAdams("C:\\Users\\py033\\Desktop\\m4.cmd", r, 0);
	}
	catch (std::exception&e)
	{
		std::cout << e.what() << std::endl;
	}
}

void test_model_compute()
{
	std::cout << std::endl << "-----------------test model compute---------------------" << std::endl;
	test_solver_compute();
	//test_solver_under_constraint();
	//test_solver_3R();
	test_solver_6R();
	test_solver_stewart();
	std::cout << "-----------------test model compute finished------------" << std::endl << std::endl;
}

